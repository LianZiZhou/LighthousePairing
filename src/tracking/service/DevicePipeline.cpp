#include "tracking/service/DevicePipeline.h"

#include "tracking/protocol/Commands.h"

#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <limits>

namespace lhp::tracking::service {

using math::Pose;
using math::Vec3;
using protocol::kTicksPerSecond;
using protocol::Ticks;

namespace {

constexpr double kObsMaxAgeS = 0.030;     // angles older than this are not used in a single-frame solve
constexpr double kSolveEveryS = 0.012;    // bootstrapping: at most ~80 single-frame solves per second
constexpr double kCompareEveryS = 0.1;    // tracking: single-frame check of the filter 10x per second
constexpr double kRadToDeg = 180.0 / math::kPi;
constexpr double kFlushStaleS = 0.045;    // angles of a rotation whose closing sync never came
constexpr double kMaxFilterLagS = 0.1;    // the filter may trail the newest IMU sample by this much (waiting for light)
constexpr double kNoLightZvuS = 0.1;      // libsurvive kalman-zvu-no-light-time
constexpr double kImuOnlyAfterS = 0.25;   // no applied light update for this long: IMU only
constexpr double kLostAfterS = 3.0;
constexpr int kInconsistentRecover = 3;   // consecutive rotations that disagree as a whole -> re-anchor
constexpr int kBatchDisagreeRecover = 3;  // consecutive confident single-frame solves far from the filter -> re-anchor
constexpr double kDisagreeMm = 60, kDisagreeDeg = 6;

double Secs(Ticks t) { return t / kTicksPerSecond; }

}  // namespace

void DevicePipeline::Trace(const char* fmt, ...) const {
    if (!cfg_.trace) return;
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    cfg_.trace(info_.serial + "," + buf);
}

const char* ToString(TrackState s) {
    switch (s) {
        case TrackState::NoModel: return "无模型";
        case TrackState::WaitingLight: return "等待光数据";
        case TrackState::Bootstrapping: return "初始化中";
        case TrackState::Tracking: return "跟踪中";
        case TrackState::ImuOnly: return "仅 IMU";
        case TrackState::Lost: return "丢失";
    }
    return "?";
}

DevicePipeline::DevicePipeline(std::unique_ptr<io::ReportSource> source, PipelineConfig cfg)
    : info_(source->Info()), source_(std::move(source)), cfg_(std::move(cfg)) {}

DevicePipeline::~DevicePipeline() { Stop(); }

void DevicePipeline::Start() {
    thread_ = std::thread([this] { Run(); });
}

void DevicePipeline::Stop() {
    stop_ = true;
    if (source_) source_->Cancel();
    if (thread_.joinable()) thread_.join();
}

void DevicePipeline::SetRecorder(std::shared_ptr<io::RecordingWriter> w) {
    std::scoped_lock lk(recMtx_);
    recorder_ = std::move(w);
    if (recorder_) recorder_->AddSource(info_);
}

void DevicePipeline::Run() {
    io::RawReport r;
    while (!stop_) {
        if (source_->Read(r, 100)) {
            {
                std::scoped_lock lk(recMtx_);
                if (recorder_) recorder_->Write(r);
            }
            Handle(r);
        } else if (source_->Ended()) {
            break;
        }
        uint64_t now = io::HostMicros();
        TryLoadModel(now);
    }
    ended_ = true;
}

void DevicePipeline::TryLoadModel(uint64_t hostUs) {
    {
        std::scoped_lock lk(mtx_);
        if (model_) return;
    }
    if (hostUs - lastModelTryUs_ < 3'000'000 && lastModelTryUs_) return;
    lastModelTryUs_ = hostUs;
    if (!cfg_.models) return;
    auto json = cfg_.models->ConfigFor(info_.serial);
    std::scoped_lock lk(mtx_);
    if (!json) {
        modelNote_ = "还没有设备配置（需要设备上线后由 lighthouse_console 读取，或 SteamVR 缓存）";
        return;
    }
    std::string err;
    auto m = model::ParseDeviceModel(*json, &err);
    if (!m) {
        modelNote_ = "设备配置无法解析: " + err;
        return;
    }
    model_ = std::move(m);
    modelNote_.clear();
    filter_.emplace(*model_);
    state_ = TrackState::WaitingLight;
    if (cfg_.log) cfg_.log(info_.serial + ": 已加载设备模型 " + model_->serial + " (" + model_->modelNumber + ", " + std::to_string(model_->points.size()) + " 个传感器)");
}

void DevicePipeline::Handle(const io::RawReport& r) {
    std::scoped_lock lk(mtx_);
    ++reportCount_;
    // host-side report rate / max interval (1 s windows)
    if (lastReportUs_) windowMaxIntervalMs_ = std::max(windowMaxIntervalMs_, (r.hostUs - lastReportUs_) / 1000.0);
    lastReportUs_ = r.hostUs;
    if (!rateWindowStartUs_) rateWindowStartUs_ = r.hostUs;
    ++rateWindowCount_;
    if (r.hostUs - rateWindowStartUs_ >= 1'000'000) {
        reportRate_ = rateWindowCount_ * 1e6 / (double)(r.hostUs - rateWindowStartUs_);
        maxIntervalMs_ = windowMaxIntervalMs_;
        windowMaxIntervalMs_ = 0;
        rateWindowCount_ = 0;
        rateWindowStartUs_ = r.hostUs;
    }

    events_.clear();
    decoder_.Decode(r.bytes.data(), r.bytes.size(), events_);
    for (auto& e : events_) {
        if (auto* s = std::get_if<protocol::ImuSample>(&e)) OnImu(*s);
        else if (auto* y = std::get_if<protocol::Lh2Sync>(&e)) OnSync(*y);
        else if (auto* w = std::get_if<protocol::Lh2Sweep>(&e)) OnSweep(*w);
        else if (auto* c = std::get_if<protocol::ChannelConflict>(&e)) lh2_.OnConflict(c->channel);
        else if (auto* b = std::get_if<protocol::Battery>(&e)) {
            battery_ = b->percent;
            charging_ = b->charging;
        }
    }
    FlushLight(-1);   // stale leftovers only; complete rotations are flushed by their closing sync
    UpdateTrackState(Secs(std::max(lastImuTime_, lastLightTime_)));
    MaybeSwitchMode(r.hostUs);
}

void DevicePipeline::MaybeSwitchMode(uint64_t hostUs) {
    using protocol::LightMode;
    LightMode m = decoder_.CurrentLightMode();
    const auto& st = decoder_.Stats();
    bool lightSeen = st.pulses > 0 || st.syncs > 0 || st.sweeps > 0 || st.raw0Votes > 0;
    if (m == LightMode::Raw1) {
        modeNote_ = activeAllowed_ ? "raw1（LH2 解码）" : "raw1（LH2 解码，由当前主控设置）";
        return;
    }
    if (m == LightMode::Unknown && !lightSeen) {
        modeNote_ = "还没有收到光数据（设备未看到基站？）";
        return;
    }
    if (!activeAllowed_) {
        modeNote_ = m == LightMode::Raw0 ? "设备处于 raw0（脉冲）模式：无法计算 LH2 角度。开启\"主动模式\"后可切换到 raw1"
                                         : "正在识别光数据格式";
        return;
    }
    if (!cfg_.hidInfo) return;
    if (lastModeCmdUs_ && hostUs - lastModeCmdUs_ < 5'000'000) return;
    if (m == LightMode::Unknown && st.packets < 200) return;
    lastModeCmdUs_ = hostUs;
    std::string err;
    auto dev = hid::HidDevice::Open(*cfg_.hidInfo, hid::Access::ReadWrite, &err);
    bool ok = false;
    if (dev) {
        auto cmd = protocol::commands::RfSetLightMode(LightMode::Raw1);
        ok = dev->SetFeature(cmd.data(), cmd.size());
        if (!ok) err = dev->LastError();
    }
    if (cfg_.log) cfg_.log(info_.serial + (ok ? ": 已切换到 raw1 光数据模式" : ": 切换 raw1 失败: " + err));
    if (ok) {
        decoder_ = protocol::WatchmanDecoder{};   // re-detect the encoding
        // the new decoder unwraps the 32-bit device clock from scratch: every timestamp changes epoch
        OnTimeBaseReset("光数据模式已切换，设备时间重新起算");
    }
    modeNote_ = ok ? "已发送 raw1 切换命令" : "切换失败: " + err;
}

void DevicePipeline::OnTimeBaseReset(const std::string& reason) {
    ResetFilter(reason);
    latest_.clear();
    pending_.clear();
    imuQueue_.clear();
    lastLightMeas_.clear();
    lastLight_.clear();
    baseLocal_.clear();
    lh2_ = lighthouse::Lh2Processor{};   // holds per-channel sync times of the old epoch
    lastLightTime_ = lastImuTime_ = firstImuTime_ = 0;
    lastSolveTime_ = poseTime_ = 0;
    imuCount_ = 0;
    still_.Reset();
    history_.clear();
    lastHistoryT_ = lastUpSampleT_ = lightRateWindowT_ = lastLightUpdateT_ = 0;
    lightRateWindowN_ = 0;
    ++timeBaseResets_;
}

bool DevicePipeline::CheckClock(Ticks t) {
    // A device clock that jumps (another program reset the device / re-initialised the radio link, or our own
    // decoder restarted) would freeze the filter on stale times: start over on the new time base.
    const Ticks ref = std::max(lastImuTime_, lastLightTime_);
    if (!ref) return true;
    double d = ((double)t - (double)ref) / kTicksPerSecond;
    if (d < -0.002 || d > 0.05) Trace("C,%.6f,%.6f,%.6f", Secs(t), Secs(ref), d);
    if (d < -0.5 || d > 10.0) {
        OnTimeBaseReset("设备时钟跳变（可能被其它程序重置了设备）");
        return true;
    }
    return true;
}

void DevicePipeline::OnImu(const protocol::ImuSample& s) {
    CheckClock(s.time);
    ++imuCount_;
    if (!firstImuTime_) firstImuTime_ = s.time;
    if (lastImuTime_ && imuCount_ > 1) {
        double dt = (s.time - firstImuTime_) / kTicksPerSecond;
        if (dt > 0) imu_.rateHz = (imuCount_ - 1) / dt;
        if (dt > 5) {   // sliding: restart the average every 5 s
            firstImuTime_ = s.time;
            imuCount_ = 1;
        }
    }
    lastImuTime_ = s.time;
    imu_.valid = true;
    for (int i = 0; i < 3; ++i) {
        imu_.rawAccel[i] = s.accel[i];
        imu_.rawGyro[i] = s.gyro[i];
    }
    const filter::ImuSi si = filter::ImuToSi(s.accel, s.gyro, model_ ? &model_->imu : nullptr);
    imu_.accel = si.acc;
    imu_.gyro = si.gyro;
    // gravity direction (low-pass), expressed in the device frame
    Vec3 aDev = model_ ? model_->imu.mount.rot.Rotate(si.acc) : si.acc;
    Vec3 dir = math::Normalize(aDev);
    imu_.gravityDevice = math::Norm(imu_.gravityDevice) < 0.5 ? dir : math::Normalize(imu_.gravityDevice * 0.98 + dir * 0.02);

    const double t = Secs(s.time);
    still_.Add(t, si.acc, si.gyro);
    fs_.still = still_.Still();
    fs_.stillSeconds = still_.StillSeconds();
    // Delayed fusion: the filter runs on the light's clock. IMU samples wait in a queue until the light they
    // overlap with has been fused (light can trail the IMU by 50+ ms over the radio); extrapolating the state
    // back over that gap during fast hand motion is what made it diverge. The displayed pose is predicted
    // forward over the queue (Snapshot).
    imuQueue_.push_back({t, si, still_.Still(), still_.StillSeconds(), still_.MeanAcc()});
    while (!imuQueue_.empty() && t - imuQueue_.front().t > 0.5) imuQueue_.pop_front();
    FeedImuUpTo(ImuHorizon());
}

double DevicePipeline::ImuHorizon() const {
    // never run ahead of light that is still waiting to be fused, and never lag the IMU by more than kMaxFilterLagS
    double h = Secs(lastLightTime_);
    for (auto& [ch, vec] : pending_)
        for (auto& p : vec) h = std::min(h, Secs(p.o.time));
    return std::max(h, Secs(lastImuTime_) - kMaxFilterLagS);
}

void DevicePipeline::FeedImuUpTo(double horizon) {
    while (!imuQueue_.empty() && imuQueue_.front().t <= horizon) {
        QueuedImu q = imuQueue_.front();
        imuQueue_.pop_front();
        if (filter_ && filter_->Initialized() && q.t > filter_->Time() - 1e-6) ApplyImu(q);
    }
}

void DevicePipeline::ApplyImu(const QueuedImu& q) {
    const double t = q.t;
    const filter::ImuSi& si = q.si;
    SyncWorld();
    // the accelerometer only means something once world +Y is vertical
    auto u = filter_->UpdateImu(t, si.acc, si.gyro, worldAligned_ && !si.accSaturated, !si.gyroSaturated);
    ++fs_.imuUpdates;
    fs_.accResidual = u.accResidual;
    fs_.gyroResidual = u.gyroResidual;
    fs_.imuNis = u.nis;
    fs_.accUsed = u.accUsed;
    // zero-velocity updates (libsurvive "zvu"): still device, or no light for a moment
    const auto& prm = filter_->Params();
    fs_.zvu = false;
    if (q.still) {
        filter_->UpdateZeroVelocity(prm.zvuStationaryVar, true);
        fs_.zvu = true;
    } else if (t - lastLightUpdateT_ > kNoLightZvuS) {
        filter_->UpdateZeroVelocity(prm.zvuNoLightVar, false);
        fs_.zvu = true;
    }
    if (cfg_.trace) {
        const auto& P = filter_->WorldFromDevice().pos;
        const auto& V = filter_->Velocity();
        const auto& A = filter_->Acceleration();
        const auto& Ba = filter_->AccBias();
        const auto& Bg = filter_->GyroBias();
        const auto& W = filter_->AngularVelocity();
        Trace("I,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%d,%d,%.2f,%d,%d,"
              "%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
              t, si.acc.x, si.acc.y, si.acc.z, si.gyro.x, si.gyro.y, si.gyro.z, (int)state_, P.x, P.y, P.z, V.x, V.y, V.z, A.x, A.y, A.z,
              filter_->PositionSigma() * 1000, (int)q.still, fs_.zvu, u.nis, (int)si.accSaturated, (int)si.gyroSaturated, u.accResidual.x, u.accResidual.y,
              u.accResidual.z, u.gyroResidual.x, u.gyroResidual.y, u.gyroResidual.z, Ba.x, Ba.y, Ba.z, Bg.x, Bg.y, Bg.z, W.x, W.y, W.z,
              std::sqrt(filter_->Var(filter::kA)), std::sqrt(filter_->Var(filter::kTheta)) * kRadToDeg, std::sqrt(filter_->Var(filter::kV)));
    }
    if (!filter_->Healthy()) {
        ResetFilter("滤波器发散（协方差或状态异常）");
        return;
    }
    // World tilt refinement from this device's accelerometer while it rests (SteamVR "Best tilt from ...").
    if (state_ == TrackState::Tracking && q.stillSeconds > 1.0 && t - lastUpSampleT_ > 1.0 && filter_->RotationSigma() < 0.01 && cfg_.world) {
        lastUpSampleT_ = t;
        Vec3 upDev = model_->imu.mount.rot.Rotate(math::Normalize(q.meanAcc));
        Vec3 upW = filter_->WorldFromDevice().rot.Rotate(upDev);
        std::scoped_lock wl(cfg_.world->mutex);
        auto& sc = cfg_.world->scene;
        if (sc.GravityAligned()) sc.AddUpSample(sc.WorldFromRef().rot.Conj().Rotate(upW));
    }
}

void DevicePipeline::OnSync(const protocol::Lh2Sync& s) {
    CheckClock(s.time);
    Trace("S,%.6f,%d", Secs(s.time), s.channel);
    FlushLight(s.channel);   // the previous rotation of this base is complete
    std::optional<lighthouse::BaseCalibration> cal = lh2_.OnSync(s);
    if (!lh2_.Valid(s.channel)) return;   // not (yet) a plausible base station: e.g. misparsed data
    if (cfg_.world) {
        std::scoped_lock wl(cfg_.world->mutex);
        auto& sc = cfg_.world->scene;
        bool fresh = !sc.Base(s.channel).seen;
        sc.MarkSeen(s.channel);
        if (cal) {
            sc.SetCalibration(s.channel, *cal);
            if (cfg_.world->cache) cfg_.world->cache->Save(s.channel, *cal);
        } else if (fresh && cfg_.world->cache && !sc.Base(s.channel).cal) {
            // provisional: the calibration this channel had last time (replaced when OOTX arrives)
            if (auto c = cfg_.world->cache->Load(s.channel)) sc.SetCalibration(s.channel, *c, true);
        }
    }
    if (cal && cfg_.log && ootxLogged_[s.channel] != cal->id) {   // the packet repeats every ~10 s: log it once
        ootxLogged_[s.channel] = cal->id;
        char b[128];
        std::snprintf(b, sizeof b, "%s: 收到基站 %08X（频道 %d）的 OOTX 校准数据", info_.serial.c_str(), cal->id, s.channel + 1);
        cfg_.log(b);
    }
    TrySolve(s.time);
}

void DevicePipeline::OnSweep(const protocol::Lh2Sweep& w) {
    CheckClock(w.time);
    auto o = lh2_.OnSweep(w);
    if (!o || !model_) return;
    int idx = model_->MapSensor(o->sensor);
    if (idx < 0) return;
    auto& vec = latest_[o->channel];
    auto& hv = hits_[o->channel];
    if (vec.size() != model_->points.size()) {
        vec.assign(model_->points.size(), {});
        hv.assign(model_->points.size(), {});
    }
    vec[idx][o->plane] = {*o, true};
    ++hv[idx][o->plane];
    lastLightTime_ = std::max(lastLightTime_, o->time);
    if (filter_ && filter_->Initialized() && lh2_.Valid(o->channel)) pending_[o->channel].push_back({*o, idx});
}

void DevicePipeline::SyncWorld() {
    if (!cfg_.world) return;
    uint64_t v;
    Pose wfr;
    bool aligned;
    {
        std::scoped_lock wl(cfg_.world->mutex);
        const auto& sc = cfg_.world->scene;
        v = sc.Version();
        if (v == worldVersion_) return;
        wfr = sc.WorldFromRef();
        aligned = sc.GravityAligned();
    }
    if (worldVersion_ != 0) {
        // world re-levelled / floor moved / reference changed: carry the device along so the filter stays consistent
        Pose T = wfr * worldFromRefSeen_.Inverse();
        if (filter_ && filter_->Initialized()) filter_->TransformWorld(T);
        lastGoodPose_ = T * lastGoodPose_;
        for (auto& p : trail_) p = T.Apply(p);
    }
    worldVersion_ = v;
    worldFromRefSeen_ = wfr;
    worldAligned_ = aligned;
}

void DevicePipeline::ResetFilter(const std::string& reason) {
    if (!filter_) return;
    bool was = filter_->Initialized();
    filter_->Reset();
    pending_.clear();
    badLight_ = 0;
    batchDisagree_ = 0;
    state_ = TrackState::Bootstrapping;
    Trace("R,%.6f,%s", Secs(std::max(lastImuTime_, lastLightTime_)), reason.c_str());
    if (was) {
        ++fs_.resets;
        fs_.lastResetReason = reason;
        if (cfg_.log) cfg_.log(info_.serial + ": 跟踪重置 - " + reason);
    }
}

void DevicePipeline::FlushLight(int channel) {
    if (!filter_ || !filter_->Initialized() || !cfg_.world) {
        pending_.clear();
        return;
    }
    // One update per base rotation (libsurvive integrates its saved light at each sync, SteamVR fuses "optical
    // frames"): the rotation is complete when the next sync of that channel arrives. Stale leftovers (sync lost)
    // are flushed after kFlushStaleS. Take the due batches out first: a reset below clears pending_.
    // Staleness is judged on the light's own clock: light packets can trail the IMU by 50+ ms over the radio
    // (lots of sensors lit while moving fast), which must not cut rotations into fragments.
    const double devNow = Secs(lastLightTime_);
    std::vector<std::pair<int, std::vector<PendingObs>>> due;
    for (auto it = pending_.begin(); it != pending_.end();) {
        auto& vec = it->second;
        if (vec.empty()) {
            it = pending_.erase(it);
            continue;
        }
        double oldest = Secs(vec.front().o.time);
        for (auto& p : vec) oldest = std::min(oldest, Secs(p.o.time));
        if (it->first != channel && devNow - oldest < kFlushStaleS) {
            ++it;
            continue;
        }
        due.emplace_back(it->first, std::move(vec));
        it = pending_.erase(it);
    }
    for (auto& [ch, vec] : due) {
        if (!filter_->Initialized()) return;   // reset by an earlier batch
        SyncWorld();
        filter::LightBase lb;
        bool known = false;
        uint64_t calKey = 0;
        int refinements = 0;
        {
            std::scoped_lock wl(cfg_.world->mutex);
            const auto& sc = cfg_.world->scene;
            known = sc.HasPose(ch);
            if (known) {
                lb.worldFromBase = sc.WorldFromBase(ch);
                refinements = sc.Base(ch).refinements;
                if (sc.Base(ch).cal) {
                    for (int a = 0; a < 2; ++a) lb.cal[a] = sc.Base(ch).cal->axis[a];
                    for (uint8_t x : sc.Base(ch).cal->raw) calKey = calKey * 1099511628211ull + x + 1;
                } else {
                    lb.sigmaScale = 8;   // ideal model until OOTX arrives: errors of up to ~0.1 deg
                }
            }
        }
        if (!known) continue;
        // A base whose calibration changed (OOTX replaced the ideal model or a cached one) shifts every angle a
        // little: loosen the filter instead of rejecting the now-correct data.
        auto ck = calKeys_.find(ch);
        if (ck != calKeys_.end() && ck->second != calKey) filter_->Inflate(1e-4, 3e-4);
        calKeys_[ch] = calKey;
        // the same when the base itself jumped (a multi-position refinement was applied; the bootstrap average
        // only creeps, and loosening the filter for that costs more than the small bias)
        auto rc = refineCounts_.find(ch);
        if (rc != refineCounts_.end() && rc->second != refinements) {
            filter_->Inflate(1e-4, 3e-4);
            Trace("P,%.6f,%d", filter_->Time(), ch);
        }
        refineCounts_[ch] = refinements;
        std::vector<filter::LightMeas> meas;
        meas.reserve(vec.size());
        for (auto& p : vec) meas.push_back({0, p.sensor, p.o.plane, p.o.angle, Secs(p.o.time)});
        double newest = meas.empty() ? 0 : meas[0].t;
        for (auto& m : meas) newest = std::max(newest, m.t);
        FeedImuUpTo(newest);   // IMU up to the end of this rotation; later samples stay queued
        if (!filter_->Initialized()) return;
        auto t0 = std::chrono::steady_clock::now();
        filter::LightUpdate u = filter_->UpdateLight(meas, {lb});
        double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
        fs_.lightUpdateUs = fs_.lightUpdateUs * 0.95 + us * 0.05;
        fs_.lastChannel = ch;
        fs_.lastUsed = u.used;
        fs_.lastGated = u.gated;
        fs_.lastFacingAway = u.facingAway;
        fs_.lastIterations = u.iterations;
        fs_.lastRmsBeforeDeg = u.rmsBeforeRad * kRadToDeg;
        fs_.lastRmsAfterDeg = u.rmsAfterRad * kRadToDeg;
        fs_.lastNis = u.nis;
        fs_.lastLagMs = u.lagMs;
        fs_.lastSpanMs = u.spanMs;
        fs_.lastNote = u.note;
        fs_.lightMeasurements += u.n;
        fs_.lightGated += u.gated + u.facingAway;
        {
            const auto& P = filter_->WorldFromDevice().pos;
            Trace("L,%.6f,%d,%d,%d,%d,%d,%d,%.5f,%.5f,%.3f,%.1f,%.1f,%.4f,%.4f,%.4f,%.2f,%s", filter_->Time(), ch, u.n, u.used, u.gated, u.facingAway, (int)u.applied,
                  u.rmsBeforeRad * kRadToDeg, u.rmsAfterRad * kRadToDeg, u.nis, u.lagMs, u.spanMs, P.x, P.y, P.z, filter_->PositionSigma() * 1000, u.note.c_str());
        }
        lastLightMeas_[ch] = std::move(meas);
        lastLight_[ch] = u;
        if (u.applied) {
            ++fs_.lightUpdates;
            ++lightRateWindowN_;
            lastLightUpdateT_ = filter_->Time();
            badLight_ = 0;
            const double t = filter_->Time();
            if (t - lightRateWindowT_ >= 1.0) {
                if (lightRateWindowT_ > 0) fs_.lightUpdateHz = lightRateWindowN_ / (t - lightRateWindowT_);
                lightRateWindowT_ = t;
                lightRateWindowN_ = 0;
            }
            if (t - lastHistoryT_ >= 1.0 / 30) {
                lastHistoryT_ = t;
                const Pose& p = filter_->WorldFromDevice();
                history_.push_back({t, p.pos, fs_.lastRmsAfterDeg, filter_->PositionSigma() * 1000, u.used, fs_.hasBatch ? fs_.batchDiffMm : -1,
                                    math::Norm(filter_->Velocity())});
                while (!history_.empty() && t - history_.front().t > 60) history_.pop_front();
                if (trail_.empty() || math::Norm(trail_.back() - p.pos) > 0.002) trail_.push_back(p.pos);
                while (trail_.size() > 600) trail_.pop_front();
            }
        } else {
            ++fs_.lightSkipped;
            ++skipReasons_[u.note.empty() ? "?" : u.note];
            // a few whole rotations in a row that do not fit: the filter is what is wrong
            if (u.inconsistent && ++badLight_ >= kInconsistentRecover) Recover("连续多圈光学观测与滤波器不一致");
        }
    }
}

void DevicePipeline::UpdateTrackState(double t) {
    TrackState prev = state_;
    if (!model_) {
        state_ = TrackState::NoModel;
    } else if (!filter_ || !filter_->Initialized()) {
        bool light = lastLightTime_ && t - Secs(lastLightTime_) < 1.0;
        state_ = light ? TrackState::Bootstrapping : TrackState::WaitingLight;
    } else {
        double since = t - lastLightUpdateT_;
        state_ = since < kImuOnlyAfterS ? TrackState::Tracking : since < kLostAfterS ? TrackState::ImuOnly : TrackState::Lost;
        fs_.sinceLightMs = std::max(0.0, since * 1000);
    }
    if (state_ == TrackState::Tracking && prev != TrackState::Tracking && prev != TrackState::ImuOnly && prev != TrackState::Lost) trackingSince_ = t;
    fs_.trackingSeconds = (state_ == TrackState::Tracking || state_ == TrackState::ImuOnly) ? t - trackingSince_ : 0;
    switch (state_) {
        case TrackState::NoModel: fs_.note = "没有设备几何模型"; break;
        case TrackState::WaitingLight: fs_.note = "等待光数据"; break;
        case TrackState::Bootstrapping: fs_.note = "等待一次可靠的单帧解算来启动滤波器"; break;
        case TrackState::Tracking: fs_.note = "光学 + IMU 融合跟踪"; break;
        case TrackState::ImuOnly: fs_.note = "暂时没有光学数据：姿态由陀螺仪推算，位置保持"; break;
        case TrackState::Lost: fs_.note = "长时间没有光学数据：显示的是最后的位姿"; break;
    }
}

DevicePipeline::FrameSolve DevicePipeline::SolveRecent(Ticks ref, const Pose* seed, const Vec3& velocity, const Vec3& angularVelocity) {
    FrameSolve out;
    auto secs = [](Ticks a, Ticks b) { return ((double)a - (double)b) / kTicksPerSecond; };
    // recent observations, grouped by channel; dt relative to `ref` (the motion model compensates it)
    for (auto& [ch, vec] : latest_)
        for (size_t s = 0; s < vec.size() && lh2_.Valid(ch); ++s)
            for (int p = 0; p < 2; ++p) {
                const Obs& ob = vec[s][p];
                if (!ob.valid) continue;
                double age = secs(ref, ob.o.time);
                if (age < -0.005 || age > kObsMaxAgeS) continue;
                out.byChannel[ch].push_back({0, (int)s, p, ob.o.angle, -age});
            }
    if (out.byChannel.empty()) return out;
    {
        std::scoped_lock wl(cfg_.world->mutex);
        auto& sc = cfg_.world->scene;
        out.worldFromRef = sc.WorldFromRef();
        for (auto& [ch, m] : out.byChannel) {
            sc.MarkSeen(ch);
            BaseInfo bi{sc.HasPose(ch), sc.ReadyToBootstrap(ch), sc.Base(ch).cal.has_value(), sc.HasPose(ch) ? sc.WorldFromBase(ch) : Pose{}, {}};
            if (sc.Base(ch).cal)
                for (int a = 0; a < 2; ++a) bi.cal[a] = sc.Base(ch).cal->axis[a];
            out.info[ch] = bi;
        }
    }
    for (auto& [ch, m] : out.byChannel) {
        const BaseInfo& bi = out.info[ch];
        if (!bi.known) continue;
        out.allCalibrated &= bi.calibrated;
        int b = (int)out.bases.size();
        solver::BaseView v;
        v.worldFromBase = bi.worldFromBase;
        v.cal[0] = bi.cal[0];
        v.cal[1] = bi.cal[1];
        out.bases.push_back(v);
        for (auto mm : m) {
            mm.base = b;
            out.meas.push_back(mm);
            out.measCh.push_back(ch);
        }
    }
    solver::SolverOptions opt;
    // 6-dof pose with the motion held fixed (libsurvive mpfit-model-velocity): every angle is moved to `ref` with
    // the given velocities, so a fast moving device is solved without motion blur.
    opt.estimateVelocity = false;
    opt.velocityPrior = velocity;
    opt.angularVelocityPrior = angularVelocity;
    if (out.meas.size() >= 6) out.res = solver::SolvePose(*model_, out.meas, out.bases, seed, opt);
    else out.res.failReason = "已知位置的基站观测不足";
    return out;
}

bool DevicePipeline::GoodSolve(const FrameSolve& f) {
    const auto& r = f.res;
    return r.ok && r.used >= 8 && r.rmsRad < (f.allCalibrated ? 1e-3 : 4e-3) && r.positionSigma < 0.05 && r.rotationSigma < 0.05;
}

void DevicePipeline::Recover(const std::string& reason) {
    // Re-anchor on an independent, motion-compensated single-frame solve (keeps velocity and IMU biases);
    // if that is not trustworthy either, start over.
    const Ticks ref = lastLightTime_;   // the angles' time (the IMU may run 50+ ms ahead of the light)
    const double tRef = Secs(ref);
    Pose seed = filter_->PoseAt(tRef);
    FrameSolve moving = SolveRecent(ref, &seed, filter_->Velocity(), filter_->AngularVelocity());
    FrameSolve still = SolveRecent(ref, &seed, {}, {});   // the filter's motion may be part of what went wrong
    const FrameSolve* best = nullptr;
    if (GoodSolve(moving)) best = &moving;
    if (GoodSolve(still) && (!best || still.res.rmsRad < best->res.rmsRad)) best = &still;
    if (!best) {
        ResetFilter(reason + "，单帧解算也不可靠");
        return;
    }
    filter_->Reanchor(tRef, best->res.worldFromDevice, best->res.covariance);
    if (best == &still) filter_->UpdateZeroVelocity(1.0, false);   // loosely: the old motion was suspect
    ++fs_.recoveries;
    fs_.lastRecoveryReason = reason;
    badLight_ = 0;
    batchDisagree_ = 0;
    lastLightUpdateT_ = tRef;
    pending_.clear();
    FeedImuUpTo(ImuHorizon());   // catch up with the IMU queued since the solve's time
    Trace("V,%.6f,%s,%.5f,%d", tRef, reason.c_str(), best->res.rmsRad * kRadToDeg, best->res.used);
}

void DevicePipeline::TrySolve(Ticks now) {
    if (!model_ || !cfg_.world) return;
    // Syncs of different bases interleave, so "now" is not monotonic across calls: use signed differences.
    auto secs = [](Ticks a, Ticks b) { return ((double)a - (double)b) / kTicksPerSecond; };
    const bool tracking = filter_ && filter_->Initialized();
    const double every = tracking ? kCompareEveryS : kSolveEveryS;
    if (lastSolveTime_ && secs(now, lastSolveTime_) < every && secs(now, lastSolveTime_) > -0.1) return;
    lastSolveTime_ = now;
    const Ticks ref = std::max(now, lastLightTime_);
    const double tRef = Secs(ref);
    SyncWorld();

    auto t0 = std::chrono::steady_clock::now();
    std::optional<Pose> seed;
    Vec3 vel, angVel;
    if (tracking) {
        seed = filter_->PoseAt(tRef);
        vel = filter_->Velocity();
        angVel = filter_->AngularVelocity();
    } else if (poseTime_ && std::fabs(secs(ref, poseTime_)) < 0.5) {
        seed = lastGoodPose_;
    }
    FrameSolve fsol = SolveRecent(ref, seed ? &*seed : nullptr, vel, angVel);
    if (fsol.byChannel.empty()) return;
    const solver::SolveResult& res = fsol.res;
    ++solves_;
    if (!res.ok) ++solveFailures_;
    lastSolve_ = res;
    lastMeas_ = fsol.meas;
    lastMeasChannel_ = fsol.measCh;
    lastBases_ = fsol.bases;
    basesUsed_ = (int)fsol.bases.size();
    Trace("B,%.6f,%d,%.5f,%d,%d,%.4f,%.4f,%.4f,%.2f,%s", tRef, (int)res.ok, res.rmsRad * kRadToDeg, res.used, (int)fsol.bases.size(), res.worldFromDevice.pos.x,
          res.worldFromDevice.pos.y, res.worldFromDevice.pos.z, res.positionSigma * 1000, res.failReason.c_str());

    if (res.ok) {
        poseTime_ = ref;
        lastGoodPose_ = res.worldFromDevice;
        const Vec3 gyroWorld = imu_.valid ? res.worldFromDevice.rot.Rotate(model_->imu.mount.rot.Rotate(imu_.gyro)) : Vec3{};
        const bool good = GoodSolve(fsol);
        if (!tracking) {
            // history while bootstrapping comes from the single-frame solves
            if (tRef - lastHistoryT_ >= 1.0 / 30) {
                lastHistoryT_ = tRef;
                history_.push_back({tRef, res.worldFromDevice.pos, res.rmsRad * kRadToDeg, res.positionSigma * 1000, res.used, -1, 0});
                while (!history_.empty() && tRef - history_.front().t > 60) history_.pop_front();
            }
            if (good && filter_) {
                filter_->Initialize(tRef, res.worldFromDevice, res.covariance, gyroWorld);
                lastLightUpdateT_ = tRef;
                trackingSince_ = tRef;
                badLight_ = 0;
                batchDisagree_ = 0;
                pending_.clear();
                fs_.hasBatch = false;
                FeedImuUpTo(ImuHorizon());   // the IMU queued since the solve's time
                if (cfg_.log && fs_.resets == 0 && fs_.lightUpdates == 0) cfg_.log(info_.serial + ": 开始跟踪（光学 + IMU 滤波）");
            }
        } else {
            // the filter vs an independent, motion-compensated single-frame solve
            Pose fp = filter_->PoseAt(tRef);
            fs_.hasBatch = true;
            fs_.batchDiffMm = math::Norm(fp.pos - res.worldFromDevice.pos) * 1000;
            fs_.batchDiffDeg = math::Norm((fp.rot * res.worldFromDevice.rot.Conj()).ToRotVec()) * kRadToDeg;
            const bool apart = fs_.batchDiffMm > kDisagreeMm || fs_.batchDiffDeg > kDisagreeDeg;
            if (good && res.positionSigma < 0.01 && apart) {
                if (++batchDisagree_ >= kBatchDisagreeRecover) Recover("滤波器与单帧解算持续不一致");
            } else {
                batchDisagree_ = 0;
            }
        }
    }
    if (tracking && filter_->Initialized() && state_ == TrackState::Tracking) CollectRefineKeyframes(ref, fsol);

    // Bootstrap / refine bases we do not know (well) yet from this device, using the best pose we have.
    std::optional<Pose> devicePose;
    if (filter_ && filter_->Initialized() && state_ == TrackState::Tracking && filter_->PositionSigma() < 0.01) devicePose = filter_->PoseAt(tRef);
    else if (res.ok) devicePose = res.worldFromDevice;
    // Only while the device is still: the per-base solve below is static, and motion between the two bases'
    // sampling times would otherwise bias the base pose.
    if (devicePose && still_.Still()) {
        Pose refFromDevice = fsol.worldFromRef.Inverse() * *devicePose;
        solver::SolverOptions opt;
        opt.estimateVelocity = false;
        for (auto& [ch, m] : fsol.byChannel) {
            if (!fsol.info[ch].canBootstrap || m.size() < 8) continue;
            solver::BaseView v;
            v.cal[0] = fsol.info[ch].cal[0];
            v.cal[1] = fsol.info[ch].cal[1];
            std::vector<solver::BaseView> one{v};
            std::vector<solver::Measurement> mm = m;
            for (auto& x : mm) x.dt = 0;
            auto it = baseLocal_.find(ch);
            auto r1 = solver::SolvePose(*model_, mm, one, it != baseLocal_.end() ? &it->second : nullptr, opt);
            if (!r1.ok) continue;
            baseLocal_[ch] = r1.worldFromDevice;
            std::scoped_lock wl(cfg_.world->mutex);
            cfg_.world->scene.AddBootstrapSample(ch, refFromDevice, r1.worldFromDevice);
        }
    }
    solveUs_ = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
}

void DevicePipeline::CollectRefineKeyframes(Ticks ref, const FrameSolve& f) {
    // Every angle is moved to `ref` with the filter's motion; keep to moderate motion so that model is good
    // over the ~20 ms the angles of both bases span.
    const Vec3 vel = filter_->Velocity(), angVel = filter_->AngularVelocity();
    if (!still_.Still() && (math::Norm(angVel) > 1.5 || math::Norm(vel) > 0.6)) return;
    const double tRef = Secs(ref);
    const Pose filterPose = filter_->PoseAt(tRef);
    const Pose refFromWorld = f.worldFromRef.Inverse();
    int refCh;
    std::vector<int> want;
    {
        std::scoped_lock wl(cfg_.world->mutex);
        const auto& sc = cfg_.world->scene;
        refCh = sc.ReferenceChannel();
        for (auto& [ch, m] : f.byChannel)
            if (ch != refCh && m.size() >= 8 && sc.WantsRefineKeyframe(ch, refFromWorld * filterPose)) want.push_back(ch);
    }
    auto rit = f.byChannel.find(refCh);
    auto rinfo = f.info.find(refCh);
    if (want.empty() || rit == f.byChannel.end() || rinfo == f.info.end() || !rinfo->second.calibrated || rit->second.size() < 10) return;
    // the device from the reference base alone: independent of the poses being refined
    solver::BaseView view;
    view.worldFromBase = f.worldFromRef;
    view.cal[0] = rinfo->second.cal[0];
    view.cal[1] = rinfo->second.cal[1];
    std::vector<solver::Measurement> mm = rit->second;
    for (auto& m : mm) m.base = 0;
    solver::SolverOptions opt;
    opt.estimateVelocity = false;
    opt.velocityPrior = vel;
    opt.angularVelocityPrior = angVel;
    solver::SolveResult r = solver::SolvePose(*model_, mm, {view}, &filterPose, opt);
    if (!r.ok || r.used < 10 || r.rmsRad > 1e-3 || r.positionSigma > 0.02 || r.rotationSigma > 0.02) return;
    for (int ch : want) {
        solver::RefineKeyframe k;
        k.refFromDevice = refFromWorld * r.worldFromDevice;
        for (const auto& m : f.byChannel.at(ch)) {
            // the device where it was when this angle was measured (m.dt <= 0 relative to ref)
            Pose at{(math::Quat::FromRotVec(angVel * m.dt) * r.worldFromDevice.rot).Normalized(), r.worldFromDevice.pos + vel * m.dt};
            k.obs.push_back({refFromWorld.Apply(at.Apply(model_->points[m.sensor])), m.plane, m.angle});
        }
        std::scoped_lock wl(cfg_.world->mutex);
        cfg_.world->scene.AddRefineKeyframe(ch, std::move(k));
    }
    Trace("K,%.6f,%zu,%.5f,%d", tRef, want.size(), r.rmsRad * kRadToDeg, r.used);
}

double DevicePipeline::LowestSensorY() const {
    std::scoped_lock lk(mtx_);
    if (!model_) return std::numeric_limits<double>::quiet_NaN();
    Pose p;
    if (filter_ && filter_->Initialized()) p = filter_->WorldFromDevice();
    else if (lastSolve_ && lastSolve_->ok) p = lastSolve_->worldFromDevice;
    else return std::numeric_limits<double>::quiet_NaN();
    double lo = std::numeric_limits<double>::max();
    for (auto& pt : model_->points) lo = std::min(lo, p.Apply(pt).y);
    return lo;
}

DeviceSnapshot DevicePipeline::Snapshot() const {
    std::scoped_lock lk(mtx_);
    DeviceSnapshot d;
    d.receiverSerial = info_.serial;
    d.pid = info_.pid;
    d.product = info_.product;
    d.hasModel = model_.has_value();
    d.modelNote = modelNote_;
    if (model_) {
        d.deviceSerial = model_->serial;
        d.modelNumber = model_->modelNumber;
    }
    d.lightMode = decoder_.CurrentLightMode();
    d.modeNote = modeNote_;
    d.decoder = decoder_.Stats();
    d.reportRate = reportRate_;
    d.hostIntervalMaxMs = maxIntervalMs_;
    d.channels = lh2_.Channels();
    d.imu = imu_;
    d.batteryPct = battery_;
    d.charging = charging_;
    d.solves = solves_;
    d.solveFailures = solveFailures_;
    d.solveUs = solveUs_;
    d.basesUsed = basesUsed_;
    const Ticks devNowTicks = std::max(lastLightTime_, lastImuTime_);
    const double devNow = Secs(devNowTicks);

    // single-frame solve (bootstrap / check)
    if (lastSolve_) {
        const auto& r = *lastSolve_;
        d.rmsDeg = r.rmsRad * kRadToDeg;
        d.maxDeg = r.maxAbsRad * kRadToDeg;
        d.initialRmsDeg = r.initialRmsRad * kRadToDeg;
        d.positionSigmaMm = r.positionSigma * 1000;
        d.rotationSigmaDeg = r.rotationSigma * kRadToDeg;
        d.covariance = r.covariance;
        d.used = r.used;
        d.rejected = r.rejected;
        d.iterations = r.iterations;
        d.seedsTried = r.seedsTried;
        d.solveNote = r.ok ? "" : r.failReason;
    }

    // primary pose: the filter when it runs, the last good single-frame solve otherwise
    d.filter = fs_;
    d.filter.state = state_;
    d.filter.skipReasons.assign(skipReasons_.begin(), skipReasons_.end());
    const bool filt = filter_ && filter_->Initialized();
    if (filt) {
        const auto& f = *filter_;
        // The filter trails the IMU (delayed fusion): show a copy predicted over the queued IMU samples.
        filter::TrackerFilter shown = f;
        for (const auto& q : imuQueue_)
            if (q.t > shown.Time()) shown.UpdateImu(q.t, q.si.acc, q.si.gyro, worldAligned_ && !q.si.accSaturated, !q.si.gyroSaturated);
        d.worldFromDevice = shown.WorldFromDevice();
        d.poseTime = shown.Time();
        d.velocity = shown.Velocity();
        d.angularVelocity = shown.AngularVelocity();
        d.filter.filterLagMs = std::max(0.0, (Secs(lastImuTime_) - f.Time()) * 1000);
        d.velocityEstimated = true;
        d.hasPose = state_ == TrackState::Tracking || state_ == TrackState::ImuOnly;
        d.poseAgeMs = std::max(0.0, (devNow - lastLightUpdateT_) * 1000);
        auto& fsn = d.filter;
        fsn.velocity = f.Velocity();
        fsn.angularVelocity = f.AngularVelocity();
        fsn.acceleration = f.Acceleration();
        fsn.gyroBias = f.GyroBias();
        fsn.accBias = f.AccBias();
        fsn.posSigmaMm = f.PositionSigma() * 1000;
        fsn.rotSigmaDeg = f.RotationSigma() * kRadToDeg;
        auto tr3 = [&](int i) { return std::sqrt(std::max(0.0, f.Var(i) + f.Var(i + 1) + f.Var(i + 2))); };
        fsn.velSigma = tr3(filter::kV);
        fsn.angVelSigma = tr3(filter::kW);
        fsn.gyroBiasSigma = tr3(filter::kBg);
        fsn.accBiasSigma = tr3(filter::kBa);
        fsn.poseCovariance = f.PoseCovariance();
    } else if (poseTime_ && lastSolve_) {
        d.worldFromDevice = lastGoodPose_;
        d.poseTime = Secs(poseTime_);
        d.poseAgeMs = devNowTicks > poseTime_ ? (devNowTicks - poseTime_) / kTicksPerSecond * 1000 : 0;
        d.hasPose = d.poseAgeMs < 500;
    }
    d.trail.assign(trail_.begin(), trail_.end());
    d.history.assign(history_.begin(), history_.end());

    // gravity consistency: IMU "up" in the world through the optical orientation vs world +Y
    if (d.hasPose && imu_.valid && cfg_.world) {
        bool aligned;
        {
            std::scoped_lock wl(cfg_.world->mutex);
            aligned = cfg_.world->scene.GravityAligned();
        }
        if (aligned) {
            Vec3 upW = d.worldFromDevice.rot.Rotate(imu_.gravityDevice);
            d.imu.gravityErrorDeg = std::acos(std::clamp(upW.y, -1.0, 1.0)) * kRadToDeg;
        }
    }

    // per-sensor deep data
    if (model_) {
        d.sensors.resize(model_->points.size());
        for (size_t s = 0; s < model_->points.size(); ++s) {
            auto& ss = d.sensors[s];
            ss.index = (int)s;
            ss.reportedId = model_->ReportedId((int)s);
            ss.point = model_->points[s];
            ss.normal = model_->normals[s];
        }
        for (auto& [ch, vec] : latest_) {
            if (vec.size() != d.sensors.size()) continue;
            for (size_t s = 0; s < vec.size(); ++s) {
                SensorBaseData sb;
                sb.channel = (uint8_t)ch;
                bool any = false;
                for (int p = 0; p < 2; ++p) {
                    const Obs& ob = vec[s][p];
                    if (!ob.valid) continue;
                    any = true;
                    sb.has[p] = true;
                    sb.angleDeg[p] = ob.o.angle * kRadToDeg;
                    sb.ageMs[p] = devNowTicks > ob.o.time ? (devNowTicks - ob.o.time) / kTicksPerSecond * 1000 : 0;
                    sb.hits[p] = hits_.at(ch)[s][p];
                    if (sb.ageMs[p] < 50) d.sensors[s].litNow = true;
                }
                if (!any) continue;
                bool fromFilter = false;
                if (filt) {   // residuals from the filter's last optical update of this base
                    auto lm = lastLightMeas_.find(ch);
                    auto lu = lastLight_.find(ch);
                    if (lm != lastLightMeas_.end() && lu != lastLight_.end() && lu->second.residuals.size() == lm->second.size()) {
                        for (size_t i = 0; i < lm->second.size(); ++i) {
                            const auto& m = lm->second[i];
                            if (m.sensor != (int)s) continue;
                            double res = lu->second.residuals[i];
                            if (!std::isfinite(res)) continue;
                            sb.residualDeg[m.plane] = res * kRadToDeg;
                            sb.predictedDeg[m.plane] = (m.angle - res) * kRadToDeg;
                            sb.rejected[m.plane] = lu->second.rejected[i] != 0;
                            fromFilter = true;
                        }
                    }
                }
                if (!fromFilter) {   // residuals from the last single-frame solve
                    for (size_t i = 0; i < lastMeas_.size() && lastSolve_; ++i) {
                        const auto& m = lastMeas_[i];
                        if (lastMeasChannel_[i] != ch || m.sensor != (int)s) continue;
                        double res = lastSolve_->residuals[i];
                        sb.residualDeg[m.plane] = res * kRadToDeg;
                        sb.predictedDeg[m.plane] = (m.angle - res) * kRadToDeg;
                        sb.rejected[m.plane] = lastSolve_->rejectedMask[i] != 0;
                    }
                }
                d.sensors[s].bases.push_back(sb);
            }
        }
    }
    return d;
}

}  // namespace lhp::tracking::service

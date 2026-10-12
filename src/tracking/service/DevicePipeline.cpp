#include "tracking/service/DevicePipeline.h"

#include "tracking/protocol/Commands.h"

#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace lhp::tracking::service {

using math::Pose;
using math::Vec3;
using protocol::kTicksPerSecond;
using protocol::Ticks;

namespace {

constexpr double kObsMaxAgeS = 0.030;     // angles older than this are not used in a solve
constexpr double kSolveEveryS = 0.012;    // at most ~80 solves per second
constexpr double kRadToDeg = 180.0 / math::kPi;
// IMU range modes reported by trackers / controllers (GyroRangeMode 3, AccelRangeMode 2).
constexpr double kAccPerLsb = 8.0 / 32768.0 * 9.80665;
constexpr double kGyroPerLsb = 2000.0 / 32768.0 * math::kPi / 180.0;

}  // namespace

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
    if (ok) decoder_ = protocol::WatchmanDecoder{};   // re-detect the encoding
    modeNote_ = ok ? "已发送 raw1 切换命令" : "切换失败: " + err;
}

void DevicePipeline::OnImu(const protocol::ImuSample& s) {
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
    Vec3 a{s.accel[0] * kAccPerLsb, s.accel[1] * kAccPerLsb, s.accel[2] * kAccPerLsb};
    Vec3 g{s.gyro[0] * kGyroPerLsb, s.gyro[1] * kGyroPerLsb, s.gyro[2] * kGyroPerLsb};
    if (model_) {
        const auto& c = model_->imu;
        a = {(a.x - c.accBias.x) * c.accScale.x, (a.y - c.accBias.y) * c.accScale.y, (a.z - c.accBias.z) * c.accScale.z};
        g = {(g.x - c.gyroBias.x) * c.gyroScale.x, (g.y - c.gyroBias.y) * c.gyroScale.y, (g.z - c.gyroBias.z) * c.gyroScale.z};
    }
    imu_.accel = a;
    imu_.gyro = g;
    // gravity direction (low-pass), expressed in the device frame
    Vec3 aDev = model_ ? model_->imu.mount.rot.Rotate(a) : a;
    Vec3 dir = math::Normalize(aDev);
    imu_.gravityDevice = math::Norm(imu_.gravityDevice) < 0.5 ? dir : math::Normalize(imu_.gravityDevice * 0.98 + dir * 0.02);
}

void DevicePipeline::OnSync(const protocol::Lh2Sync& s) {
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
    if (cal && cfg_.log) {
        char b[128];
        std::snprintf(b, sizeof b, "%s: 收到基站 %08X（频道 %d）的 OOTX 校准数据", info_.serial.c_str(), cal->id, s.channel + 1);
        cfg_.log(b);
    }
    TrySolve(s.time);
}

void DevicePipeline::OnSweep(const protocol::Lh2Sweep& w) {
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
}

void DevicePipeline::TrySolve(Ticks now) {
    if (!model_ || !cfg_.world) return;
    // Syncs of different bases interleave, so "now" is not monotonic across calls: use signed differences.
    auto secs = [](Ticks a, Ticks b) { return ((double)a - (double)b) / kTicksPerSecond; };
    if (lastSolveTime_ && secs(now, lastSolveTime_) < kSolveEveryS && secs(now, lastSolveTime_) > -0.1) return;
    lastSolveTime_ = now;
    const Ticks ref = std::max(now, lastLightTime_);

    // Gather recent observations, grouped by channel.
    std::map<int, std::vector<solver::Measurement>> byChannel;
    for (auto& [ch, vec] : latest_)
        for (size_t s = 0; s < vec.size() && lh2_.Valid(ch); ++s)
            for (int p = 0; p < 2; ++p) {
                const Obs& ob = vec[s][p];
                if (!ob.valid) continue;
                double age = secs(ref, ob.o.time);
                if (age < -0.005 || age > kObsMaxAgeS) continue;
                byChannel[ch].push_back({0, (int)s, p, ob.o.angle, -age});
            }
    if (byChannel.empty()) return;

    // Snapshot the world for this solve.
    struct BaseInfo {
        bool known;
        bool canBootstrap;
        Pose worldFromBase;
        lighthouse::AxisCal cal[2];
    };
    std::map<int, BaseInfo> info;
    Pose worldFromRef;
    {
        std::scoped_lock wl(cfg_.world->mutex);
        auto& sc = cfg_.world->scene;
        worldFromRef = sc.WorldFromRef();
        for (auto& [ch, m] : byChannel) {
            sc.MarkSeen(ch);
            // non-reference bases keep refining from still devices until enough samples are averaged
            bool refine = sc.ReadyToBootstrap(ch) && sc.Base(ch).samples < 300;
            BaseInfo bi{sc.HasPose(ch), refine, sc.HasPose(ch) ? sc.WorldFromBase(ch) : Pose{}, {}};
            if (sc.Base(ch).cal)
                for (int a = 0; a < 2; ++a) bi.cal[a] = sc.Base(ch).cal->axis[a];
            info[ch] = bi;
        }
    }

    // Joint solve against every base with a known pose.
    std::vector<solver::Measurement> meas;
    std::vector<int> measCh;
    std::vector<solver::BaseView> bases;
    for (auto& [ch, m] : byChannel) {
        if (!info[ch].known) continue;
        int bi = (int)bases.size();
        solver::BaseView v;
        v.worldFromBase = info[ch].worldFromBase;
        v.cal[0] = info[ch].cal[0];
        v.cal[1] = info[ch].cal[1];
        bases.push_back(v);
        for (auto mm : m) {
            mm.base = bi;
            meas.push_back(mm);
            measCh.push_back(ch);
        }
    }
    auto t0 = std::chrono::steady_clock::now();
    bool recent = poseTime_ && std::fabs(secs(ref, poseTime_)) < 0.5;
    Pose init = recent ? lastGoodPose_ : Pose{};
    solver::SolveResult res;
    solver::SolverOptions opt;
    if (recent) {
        opt.velocityPrior = velocity_;
        // the gyro measures angular velocity directly: rotate it into the world frame
        if (imu_.valid) opt.angularVelocityPrior = lastGoodPose_.rot.Rotate(model_->imu.mount.rot.Rotate(imu_.gyro));
    }
    if (meas.size() >= 6) res = solver::SolvePose(*model_, meas, bases, recent ? &init : nullptr, opt);
    else res.failReason = "已知位置的基站观测不足";
    ++solves_;
    if (!res.ok) ++solveFailures_;
    lastSolve_ = res;
    lastMeas_ = meas;
    lastMeasChannel_ = measCh;
    lastBases_ = bases;
    basesUsed_ = (int)bases.size();

    if (res.ok) {
        double t = ref / kTicksPerSecond;
        {   // velocity_ is ALWAYS the finite difference of solved positions: it feeds the next solve's prior,
            // so it must not depend on the solver's own velocity estimate (that would be a feedback loop).
            // finite difference over ~200 ms (a per-frame difference would be dominated by position noise)
            const PoseSample* past = nullptr;
            for (auto it = history_.rbegin(); it != history_.rend(); ++it)
                if (t - it->t >= 0.2) {
                    past = &*it;
                    break;
                }
            if (past && t - past->t < 0.5) velocity_ = (res.worldFromDevice.pos - past->pos) / (t - past->t);
        }
        poseTime_ = ref;
        lastGoodPose_ = res.worldFromDevice;
        history_.push_back({t, res.worldFromDevice.pos, res.rmsRad * kRadToDeg, res.positionSigma * 1000, res.used});
        while (!history_.empty() && t - history_.front().t > 60) history_.pop_front();
        if (trail_.empty() || math::Norm(trail_.back() - res.worldFromDevice.pos) > 0.002) trail_.push_back(res.worldFromDevice.pos);
        while (trail_.size() > 600) trail_.pop_front();

        // Bootstrap bases we do not know yet from this device.
        Pose refFromDevice = worldFromRef.Inverse() * res.worldFromDevice;
        // Only while the device is (nearly) still: the per-base solve below is static, and motion between the
        // two bases' sampling times would otherwise bias the base pose.
        double gyroRate = imu_.valid ? math::Norm(imu_.gyro) : 0.0;
        bool still = math::Norm(velocity_) < 0.05 && gyroRate < 0.3;
        for (auto& [ch, m] : byChannel) {
            if (!still || !info[ch].canBootstrap || m.size() < 8) continue;
            solver::BaseView v;
            v.cal[0] = info[ch].cal[0];
            v.cal[1] = info[ch].cal[1];
            std::vector<solver::BaseView> one{v};
            auto it = baseLocal_.find(ch);
            auto r1 = solver::SolvePose(*model_, m, one, it != baseLocal_.end() ? &it->second : nullptr);
            if (!r1.ok) continue;
            baseLocal_[ch] = r1.worldFromDevice;
            std::scoped_lock wl(cfg_.world->mutex);
            cfg_.world->scene.AddBootstrapSample(ch, refFromDevice, r1.worldFromDevice);
        }
    }
    solveUs_ = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
}

double DevicePipeline::LowestSensorY() const {
    std::scoped_lock lk(mtx_);
    if (!model_ || !lastSolve_ || !lastSolve_->ok) return std::numeric_limits<double>::quiet_NaN();
    double lo = std::numeric_limits<double>::max();
    for (auto& p : model_->points) lo = std::min(lo, lastSolve_->worldFromDevice.Apply(p).y);
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
    d.velocity = velocity_;
    const Ticks devNow = std::max(lastLightTime_, lastImuTime_);
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
        d.velocityEstimated = r.velocityEstimated;
        d.angularVelocity = r.angularVelocity;
        d.solveNote = r.ok ? "" : r.failReason;
    }
    if (poseTime_ && lastSolve_) {
        d.poseAgeMs = devNow > poseTime_ ? (devNow - poseTime_) / kTicksPerSecond * 1000 : 0;
        d.hasPose = d.poseAgeMs < 500;
        // the last good pose (lastSolve_ may be a failure)
        if (!history_.empty()) d.worldFromDevice.pos = history_.back().pos;
    }
    d.worldFromDevice = lastGoodPose_;   // a failed solve never replaces the displayed pose
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
                    sb.ageMs[p] = devNow > ob.o.time ? (devNow - ob.o.time) / kTicksPerSecond * 1000 : 0;
                    sb.hits[p] = hits_.at(ch)[s][p];
                    if (sb.ageMs[p] < 50) d.sensors[s].litNow = true;
                }
                if (!any) continue;
                // residuals from the last solve
                for (size_t i = 0; i < lastMeas_.size() && lastSolve_; ++i) {
                    const auto& m = lastMeas_[i];
                    if (lastMeasChannel_[i] != ch || m.sensor != (int)s) continue;
                    double res = lastSolve_->residuals[i];
                    sb.residualDeg[m.plane] = res * kRadToDeg;
                    sb.predictedDeg[m.plane] = (m.angle - res) * kRadToDeg;
                    sb.rejected[m.plane] = lastSolve_->rejectedMask[i] != 0;
                }
                d.sensors[s].bases.push_back(sb);
            }
        }
    }
    return d;
}

}  // namespace lhp::tracking::service

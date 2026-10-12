#include "tracking/service/TrackingService.h"

#include "tracking/hid/HidDevice.h"

#include <windows.h>

#include <chrono>
#include <cmath>
#include <ctime>

namespace lhp::tracking::service {

namespace {

constexpr uint16_t kValve = 0x28de;
bool IsReceiver(uint16_t pid) { return pid == 0x2101 || pid == 0x2102; }

std::string Clock() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char b[16];
    std::strftime(b, sizeof b, "%H:%M:%S", &tm);
    return b;
}

}  // namespace

TrackingService::TrackingService(std::shared_ptr<model::ModelProvider> models, std::shared_ptr<CalibrationCache> cache) : models_(std::move(models)) {
    world_.scene = solver::Scene{};
    world_.cache = std::move(cache);
}

TrackingService::~TrackingService() { Stop(); }

void TrackingService::Log(const std::string& msg) {
    std::scoped_lock lk(logMtx_);   // leaf lock: never take another lock while holding it
    log_.push_back(Clock() + "  " + msg);
    while (log_.size() > 200) log_.pop_front();
}

void TrackingService::Start() {
    Stop();
    {
        std::scoped_lock lk(mtx_);
        replay_ = false;
        simulated_ = false;
        replayPath_.clear();
    }
    stop_ = false;
    running_ = true;
    Log("开始读取原始 HID 数据（被动模式：只读，不向设备发送任何命令）");
    thread_ = std::thread([this] { Loop(); });
}

bool TrackingService::StartReplay(const std::string& path, std::string* error) {
    auto rec = std::make_shared<io::Recording>();
    if (!io::LoadRecording(path, *rec, error)) return false;
    std::vector<std::unique_ptr<io::ReportSource>> sources;
    for (auto& s : rec->sources) sources.push_back(io::OpenReplay(rec, s.id, true));
    StartSources(std::move(sources), path, "回放录制文件: " + path);
    return true;
}

void TrackingService::StartSources(std::vector<std::unique_ptr<io::ReportSource>> sources, const std::string& label, const std::string& logLine,
                                   std::shared_ptr<model::ModelProvider> models, bool simulated) {
    Stop();
    {
        std::scoped_lock wl(world_.mutex);
        double h = world_.scene.ReferenceHeight();
        world_.scene.Reset();
        world_.scene.SetReferenceHeight(h);
    }
    {
        std::scoped_lock lk(mtx_);
        replay_ = true;
        simulated_ = simulated;
        replayPath_ = label;
        for (auto& src : sources) {
            std::string key = src->Info().serial.empty() ? std::to_string(src->Info().id) : src->Info().serial;
            PipelineConfig cfg{models ? models : models_, &world_, [this](const std::string& m) { Log(m); }, std::nullopt};
            auto p = std::make_unique<DevicePipeline>(std::move(src), cfg);
            if (recorder_) p->SetRecorder(recorder_);
            p->Start();
            pipelines_[key] = std::move(p);
        }
    }
    stop_ = false;
    running_ = true;
    Log(logLine);
    thread_ = std::thread([this] { Loop(); });
}

void TrackingService::Stop() {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    std::map<std::string, std::unique_ptr<DevicePipeline>> doomed;
    {
        std::scoped_lock lk(mtx_);
        doomed.swap(pipelines_);
    }
    doomed.clear();
    running_ = false;
    Publish();
}

void TrackingService::SetOptions(const ServiceOptions& o) {
    std::scoped_lock lk(mtx_);
    if (o.activeRequested != options_.activeRequested) {
        Log((o.activeRequested ? std::string("已开启主动模式（SteamVR 未运行时会把设备切换到 raw1 光数据模式）") : std::string("已关闭主动模式")));
    }
    options_ = o;
}

ServiceOptions TrackingService::Options() const {
    std::scoped_lock lk(mtx_);
    return options_;
}

bool TrackingService::StartRecording(const std::string& path, std::string* error) {
    auto w = io::RecordingWriter::Create(path, error);
    if (!w) return false;
    std::shared_ptr<io::RecordingWriter> sw = std::move(w);
    {
        std::scoped_lock lk(mtx_);
        recorder_ = sw;
        recordPath_ = path;
        for (auto& [k, p] : pipelines_) p->SetRecorder(sw);
    }
    Log("开始录制: " + path);
    return true;
}

void TrackingService::StopRecording() {
    size_t n = 0;
    std::string path;
    {
        std::scoped_lock lk(mtx_);
        if (!recorder_) return;
        for (auto& [k, p] : pipelines_) p->SetRecorder(nullptr);
        n = recorder_->Count();
        recorder_.reset();
        path = recordPath_;
        recordPath_.clear();
    }
    Log("录制结束（" + std::to_string(n) + " 条记录）: " + path);
}

void TrackingService::ResetWorld() {
    std::scoped_lock wl(world_.mutex);
    double h = world_.scene.ReferenceHeight();
    world_.scene.Reset();
    world_.scene.SetReferenceHeight(h);
}

void TrackingService::SetReferenceHeight(double m) {
    std::scoped_lock wl(world_.mutex);
    world_.scene.SetReferenceHeight(m);
}

bool TrackingService::SetFloorFromDevice(const std::string& serial) {
    double y;
    {
        std::scoped_lock lk(mtx_);
        auto it = pipelines_.find(serial);
        if (it == pipelines_.end()) return false;
        y = it->second->LowestSensorY();
    }
    if (std::isnan(y)) return false;
    std::scoped_lock wl(world_.mutex);
    world_.scene.SetFloorAt(y);
    return true;
}

void TrackingService::Discover() {
    auto list = hid::Enumerate(kValve);
    std::scoped_lock lk(mtx_);
    // drop ended pipelines (unplugged)
    for (auto it = pipelines_.begin(); it != pipelines_.end();) {
        if (it->second->Ended()) {
            Log(it->first + " 已断开");
            it = pipelines_.erase(it);
        } else {
            ++it;
        }
    }
    for (auto& d : list) {
        if (!IsReceiver(d.pid) || d.serial.empty()) continue;   // wired devices: not supported yet (different report set)
        if (pipelines_.count(d.serial)) continue;
        std::string err;
        auto src = io::OpenLive(d, (uint16_t)pipelines_.size(), hid::Access::ReadOnly, &err);
        if (!src) {
            Log("无法打开 " + d.serial + ": " + err);
            continue;
        }
        PipelineConfig cfg{models_, &world_, [this](const std::string& m) { Log(m); }, d};
        auto p = std::make_unique<DevicePipeline>(std::move(src), cfg);
        if (recorder_) p->SetRecorder(recorder_);
        p->Start();
        pipelines_[d.serial] = std::move(p);
    }
}

void TrackingService::Loop() {
    auto lastDiscover = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    auto lastHost = lastDiscover;
    while (!stop_) {
        auto now = std::chrono::steady_clock::now();
        if (now - lastHost > std::chrono::seconds(2)) {
            lastHost = now;
            host_ = ProbeHost();
        }
        bool replay;
        {
            std::scoped_lock lk(mtx_);
            replay = replay_;
            std::string reason;
            bool allowed = !replay_ && options_.activeRequested && ActiveModeAllowed(host_, &reason);
            for (auto& [k, p] : pipelines_) p->SetActiveAllowed(allowed);
        }
        if (!replay && now - lastDiscover > std::chrono::seconds(2)) {
            lastDiscover = now;
            Discover();
        }
        Publish();
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}

void TrackingService::Publish() {
    auto snap = std::make_shared<TrackingSnapshot>();
    snap->seq = ++seq_;
    snap->running = running_;
    snap->host = host_;
    std::vector<std::pair<std::string, const DevicePipeline*>> pl;
    {
        std::scoped_lock lk(mtx_);
        snap->replay = replay_;
        snap->simulated = simulated_;
        snap->replayPath = replayPath_;
        snap->recording = recorder_ != nullptr;
        snap->recordingPath = recordPath_;
        snap->recordedReports = recorder_ ? recorder_->Count() : 0;
        snap->activeRequested = options_.activeRequested;
        std::string reason;
        snap->activeAllowed = !replay_ && options_.activeRequested && ActiveModeAllowed(host_, &reason);
        snap->activeNote = simulated_ ? "模拟数据（演示）" : replay_ ? "回放中" : !options_.activeRequested ? "被动模式：只读监听" : snap->activeAllowed ? "主动模式：可切换设备光数据模式" : reason;
        {
            std::scoped_lock ll(logMtx_);
            snap->log.assign(log_.begin(), log_.end());
        }
        for (auto& [k, p] : pipelines_) snap->devices.push_back(p->Snapshot());
    }
    {
        std::scoped_lock wl(world_.mutex);
        const auto& sc = world_.scene;
        snap->gravityAligned = sc.GravityAligned();
        snap->referenceChannel = sc.ReferenceChannel();
        snap->referenceHeight = sc.ReferenceHeight();
        for (int ch = 0; ch < lighthouse::kMaxChannels; ++ch) {
            const auto& b = sc.Base(ch);
            if (!b.seen) continue;
            BaseSnapshot bs;
            bs.channel = ch;
            bs.isReference = b.isReference;
            bs.hasCalibration = b.cal.has_value();
            bs.calibrationFromCache = b.calFromCache;
            if (b.cal) bs.cal = *b.cal;
            bs.hasPose = b.hasPose;
            if (b.hasPose) bs.worldFromBase = sc.WorldFromBase(ch);
            bs.bootstrapSamples = b.samples;
            bs.bootstrapSpreadMm = b.lastSpreadMm;
            snap->bases.push_back(bs);
        }
    }
    // aggregate per-base statistics from every device that sees it
    for (auto& bs : snap->bases) {
        for (auto& d : snap->devices) {
            const auto& c = d.channels[bs.channel];
            if (!c.syncs) continue;
            bs.seenBy.push_back(d.receiverSerial);
            bs.syncs += c.syncs;
            bs.skippedSyncs += c.skippedSyncs;
            bs.sweeps += c.sweeps;
            bs.conflict |= c.conflict;
            if (c.measuredHz > 0) {
                bs.measuredHz = c.measuredHz;
                bs.freqErrorHz = c.freqErrorHz;
            }
            if (c.ootx.bits > bs.ootx.bits) bs.ootx = c.ootx;
        }
    }
    std::scoped_lock lk(snapMtx_);
    snapshot_ = std::move(snap);
}

SnapshotPtr TrackingService::Snapshot() const {
    std::scoped_lock lk(snapMtx_);
    return snapshot_;
}

}  // namespace lhp::tracking::service

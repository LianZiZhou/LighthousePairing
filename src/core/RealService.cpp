// Real backend: drives lighthouse_console.exe, USB topology and process scanning.
#include "backend/ConsoleParser.h"
#include "backend/ConsoleSession.h"
#include "backend/DeviceConfig.h"
#include "backend/SystemInfo.h"
#include "backend/UsbTopology.h"
#include "core/Service.h"
#include "util/Text.h"

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <map>
#include <queue>
#include <regex>
#include <set>
#include <thread>

namespace lhp {

namespace {

constexpr double kOnlineImuHz = 20.0;

/// Lines of `lines` that describe `serial`: from the first line mentioning it up to the next
/// line that mentions a different LHR serial. Falls back to everything.
std::vector<std::string> SectionFor(const std::vector<std::string>& lines, const std::string& serial) {
    if (serial.empty()) return lines;
    static const std::regex re(R"(LHR-[0-9A-Fa-f]{8})");
    std::vector<std::string> out;
    bool in = false;
    for (auto& l : lines) {
        std::smatch m;
        bool mentions = std::regex_search(l, m, re);
        if (mentions) {
            bool ours = IEquals(m.str(), serial) || Contains(l, serial);
            if (ours) in = true;
            else if (in) break;
        }
        if (in) out.push_back(l);
    }
    return out.empty() ? lines : out;
}

}  // namespace

class RealService;

// ============================================================================ Monitor
// One persistent console session per receiver, polling live statistics.
class Monitor {
public:
    Monitor(RealService& svc, std::string serial) : svc_(svc), serial_(std::move(serial)) {
        thread_ = std::jthread([this](std::stop_token st) { Run(st); });
    }
    ~Monitor() { Stop(); }

    void Stop() {
        thread_.request_stop();
        {
            std::scoped_lock lk(sessionMtx_);
            if (session_) session_->Stop();
        }
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }

    /// Execute a command on this monitor's session (used for identify / raw commands).
    bool Exec(const std::string& cmd, std::vector<std::string>* out, double timeout, std::string* err) {
        std::shared_ptr<ConsoleSession> s;
        {
            std::scoped_lock lk(sessionMtx_);
            s = session_;
        }
        if (!s || !s->Alive()) {
            if (err) *err = "监控会话未运行";
            return false;
        }
        return s->Execute(cmd, out, timeout, err);
    }

    void RequestDeviceInfo() {
        wantDeviceInfo_ = true;
        cv_.notify_all();
    }
    /// Re-read the device's JSON config (serial, product name ...) on the next poll.
    void RequestConfig() {
        wantConfig_ = true;
        wantDeviceInfo_ = true;
        cv_.notify_all();
    }

private:
    void Run(std::stop_token st);
    void OnLine(const std::string& line);
    void PollOnce(ConsoleSession& s, bool& online, double& lastBattery, double& lastInfo, double& lowSince);

    RealService& svc_;
    std::string serial_;
    std::jthread thread_;
    std::mutex sessionMtx_;
    std::shared_ptr<ConsoleSession> session_;
    std::mutex waitMtx_;
    std::condition_variable_any cv_;
    std::atomic<bool> wantDeviceInfo_{true};
    std::atomic<bool> wantConfig_{false};
    // Identity of the device last seen online. A receiver holds exactly one pairing, so when the
    // IMU stream comes back after a dip it is the same device - keep its serial instead of
    // falling back to "reading". A connect event naming another serial replaces it.
    std::optional<DeviceInfo> lastIdentity_;
    double lastConfigTry_ = -1e9;
    int configTries_ = 0;
};

// ============================================================================ RealService
class RealService final : public IService {
public:
    explicit RealService(Store& s) : IService(s), history_(sys::DataDir()) {}
    ~RealService() override { Shutdown(); }

    void Start() override {
        history_.Load();
        auto settings = history_.LoadSettings();
        consolePath_ = sys::FindLighthouseConsole(settings.consolePathOverride);
        {
            std::scoped_lock lk(store.mutex);
            store.state.settings = settings;
            store.state.consolePath = consolePath_;
            store.state.steamVrRuntime = sys::FindSteamVrRuntime();
            store.state.dataDir = sys::DataDir();
            if (consolePath_.empty())
                store.state.backendError = "找不到 lighthouse_console.exe，请安装 SteamVR 或在设置里指定路径。";
        }
        store.Log(LogLevel::Info, "main", consolePath_.empty() ? "lighthouse_console.exe 未找到" : "使用 " + consolePath_);
        hotplug_ = std::make_unique<UsbHotplug>([this] {
            usbDirtyAt_ = NowSeconds();
            usbDirty_ = true;
        });
        worker_ = std::jthread([this](std::stop_token st) { WorkerLoop(st); });
        Post({Command::Type::Refresh});
    }

    void Shutdown() override {
        if (shutdown_.exchange(true)) return;
        hotplug_.reset();
        worker_.request_stop();
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
        if (opThread_.joinable()) {
            opCancel_ = true;
            opThread_.join();
        }
        if (refreshThread_.joinable()) refreshThread_.join();
        StopAllMonitors();
        history_.Save();
    }

    void Post(Command c) override {
        {
            std::scoped_lock lk(qMtx_);
            queue_.push(std::move(c));
        }
        cv_.notify_all();
    }

    std::vector<PairingRecord> PairingsFor(const std::string& serial) override { return history_.PairingsFor(serial); }
    std::string BackupDir() override { return history_.BackupDir(); }
    std::string HistoryPath() override { return history_.Path(); }

    // ---- used by Monitor
    std::wstring ConsoleExe() const { return ToWide(consolePath_); }
    /// SteamVR tracker role assignments, re-read from steamvr.vrsettings at most every 10 s.
    std::map<std::string, std::string> TrackerRoles() {
        std::scoped_lock lk(rolesMtx_);
        double now = NowSeconds();
        if (now - rolesReadAt_ > 10.0) {
            roles_ = devcfg::ParseTrackerRoles(sys::ReadSteamVrSettings());
            rolesReadAt_ = now;
        }
        return roles_;
    }
    HistoryDb& History() { return history_; }
    void HandleConsoleLine(const std::string& receiver, const std::string& line, bool fromMonitor);

private:
    void WorkerLoop(std::stop_token st);
    void Handle(const Command& c);
    void StartRefresh();
    void DoRefresh();
    void MergeUsb(const std::vector<UsbReceiver>& list);
    void EnsureMonitors();
    void StartMonitor(const std::string& serial);
    void StopMonitor(const std::string& serial);
    void StopAllMonitors();
    Monitor* GetMonitor(const std::string& serial);
    void ScanConflicts();

    void StartOp(OpKind kind, const std::string& serial, const std::string& deviceType, double timeout);
    void RunOp(OpKind kind, std::string serial, std::string deviceType, double timeout, uint64_t id);
    void SetOp(uint64_t id, OpPhase phase, const std::string& msg);
    bool RunSimpleDongleCommand(const std::string& serial, const std::string& cmd, std::vector<std::string>& out, std::string& err);

    HistoryDb history_;
    std::string consolePath_;
    std::jthread worker_;
    std::mutex qMtx_;
    std::condition_variable_any cv_;
    std::queue<Command> queue_;
    std::atomic<bool> shutdown_{false};

    std::unique_ptr<UsbHotplug> hotplug_;
    std::atomic<bool> usbDirty_{false};
    std::atomic<double> usbDirtyAt_{0};

    std::mutex monMtx_;
    std::map<std::string, std::unique_ptr<Monitor>> monitors_;
    std::map<std::string, bool> paused_;   // receivers reserved by an operation
    std::map<std::string, bool> manualOff_;

    std::jthread refreshThread_;
    std::atomic<bool> refreshing_{false};
    std::atomic<bool> refreshAgain_{false};

    std::thread opThread_;
    std::atomic<bool> opCancel_{false};
    std::atomic<uint64_t> opSeq_{0};
    std::mutex opLinesMtx_;
    std::vector<std::string> opLines_;

    std::mutex rolesMtx_;
    std::map<std::string, std::string> roles_;
    double rolesReadAt_ = -1e9;
};

// ---------------------------------------------------------------------------- monitor impl

void Monitor::Run(std::stop_token st) {
    double backoff = 2.0;
    while (!st.stop_requested()) {
        svc_.store.With([&](AppState& s) {
            if (auto* r = s.Find(serial_)) {
                r->monitor = MonitorState::Starting;
                r->monitorError.clear();
            }
        });
        auto session = std::make_shared<ConsoleSession>(svc_.ConsoleExe(), serial_, [this](const std::string& l) { OnLine(l); });
        std::string err;
        bool ok = session->Start(&err, 10.0);
        if (ok) {
            // The console connects to the receiver we asked for; verify.
            bool matched = false;
            for (auto& l : session->StartupLines())
                if (auto ev = parser::Classify(l); std::holds_alternative<parser::EvSessionConnected>(ev))
                    matched = IEquals(std::get<parser::EvSessionConnected>(ev).receiver, serial_);
            if (!matched) {
                ok = false;
                err = "lighthouse_console 没有连接到该接收器";
                session->Stop();
            }
        }
        if (!ok) {
            svc_.store.With([&](AppState& s) {
                if (auto* r = s.Find(serial_)) {
                    r->monitor = MonitorState::Failed;
                    r->monitorError = err;
                }
            });
            std::unique_lock lk(waitMtx_);
            cv_.wait_for(lk, st, std::chrono::duration<double>(backoff), [] { return false; });
            backoff = std::min(backoff * 2, 30.0);
            continue;
        }
        backoff = 2.0;
        {
            std::scoped_lock lk(sessionMtx_);
            session_ = session;
        }
        svc_.store.With([&](AppState& s) {
            if (auto* r = s.Find(serial_)) {
                r->monitor = MonitorState::Running;
                r->seenByConsole = true;
            }
        });
        if (svc_.store.With([](AppState& s) { return s.settings.trackingDiagnostics; })) session->Execute("pose", nullptr, 3.0, nullptr);

        bool online = false;
        double lastBattery = -1e9, lastInfo = -1e9, lowSince = 0;
        while (!st.stop_requested() && session->Alive()) {
            PollOnce(*session, online, lastBattery, lastInfo, lowSince);
            // Shake-identify needs quick updates; otherwise use the configured interval.
            int interval = svc_.store.With([](AppState& s) { return s.fastPoll ? 250 : s.settings.pollIntervalMs; });
            std::unique_lock lk(waitMtx_);
            cv_.wait_for(lk, st, std::chrono::milliseconds(std::clamp(interval, 250, 10000)), [&] { return wantDeviceInfo_.load(); });
        }
        {
            std::scoped_lock lk(sessionMtx_);
            session_.reset();
        }
        session->Stop();
        if (!st.stop_requested()) {
            svc_.store.With([&](AppState& s) {
                if (auto* r = s.Find(serial_)) {
                    r->monitor = MonitorState::Failed;
                    r->monitorError = "会话意外结束，正在重连";
                }
            });
        }
    }
    svc_.store.With([&](AppState& s) {
        if (auto* r = s.Find(serial_)) r->monitor = MonitorState::Off;
    });
}

void Monitor::PollOnce(ConsoleSession& s, bool& online, double& lastBattery, double& lastInfo, double& lowSince) {
    std::vector<std::string> out;
    std::string err;
    SignalSample sample;
    sample.t = NowSeconds();

    std::optional<ImuStats> imu;
    if (s.Execute("imustats", &out, 3.0, &err))
        for (auto& l : out)
            if ((imu = parser::ParseImuStats(l))) break;
    // imustats accumulates since the last 'clear'; reset so every sample covers just one
    // poll interval (needed for live jitter and for spotting a shaken device via grav sigma).
    if (imu) s.Execute("clear", nullptr, 2.0, nullptr);

    std::vector<UsbRate> rates;
    if (s.Execute("usbstats", &out, 3.0, &err)) {
        for (auto& l : out)
            if (auto e = parser::ParseUsbStatLine(l)) rates.push_back({e->vid + ":" + e->pid, e->serial, e->iface, e->perSec, e->maxIntervalMs});
    }

    bool nowOnline = imu && imu->rateHz >= kOnlineImuHz;
    double now = NowSeconds();
    if (!nowOnline && online) {
        if (lowSince == 0) lowSince = now;
        if (now - lowSince < 3.0) nowOnline = true;   // debounce short dips
    } else {
        lowSince = 0;
    }

    bool justOnline = nowOnline && !online;
    bool justOffline = !nowOnline && online;
    online = nowOnline;

    // ---- device identity bookkeeping (before fetching, so we know what to ask for)
    bool needConfig = false;
    std::string knownSerial;
    svc_.store.With([&](AppState& st) {
        Receiver* r = st.Find(serial_);
        if (!r) return;
        if (justOffline) {
            if (r->device) lastIdentity_ = *r->device;
            SignalEvent ev{now, UnixNow(), SignalEvent::Type::Disconnected, 0, "设备离线（IMU 数据停止）"};
            Store::PushEvent(*r, ev);
            Store::LogUnlocked(st, LogLevel::Event, serial_, "设备离线");
            r->device.reset();
        }
        if (justOnline) {
            if (!r->device) {
                r->device = lastIdentity_ ? *lastIdentity_ : DeviceInfo{};
                r->device->batteryPct.reset();
                r->device->charging.reset();
            }
            r->device->connectedSince = UnixNow();
            SignalEvent ev{now, UnixNow(), SignalEvent::Type::Connected, 0, "设备上线（IMU 数据开始流入）"};
            Store::PushEvent(*r, ev);
            Store::LogUnlocked(st, LogLevel::Event, serial_, "设备上线");
        }
        if (online && r->device) {
            knownSerial = r->device->serial;
            bool have = r->device->configSource == ConfigSource::Device;
            double retry = configTries_ < 3 ? 8.0 : 120.0;
            needConfig = wantConfig_.load() || (!have && now - lastConfigTry_ > retry);
        }
    });

    // ---- the device's own JSON config: exact serial + product name (same data SteamVR uses)
    std::optional<devcfg::DeviceConfig> cfg;
    ConfigSource cfgSource = ConfigSource::None;
    if (needConfig) {
        wantConfig_ = false;
        lastConfigTry_ = now;
        ++configTries_;
        std::string path = sys::ConsoleTempFile("lhp_config_" + serial_ + ".json");
        sys::DeleteFileUtf8(path);
        if (s.Execute("downloadconfig \"" + path + "\"", nullptr, 12.0, nullptr)) {
            std::string text = sys::ReadTextFile(path);
            auto c = devcfg::Parse(text);
            if (c.ok) {
                // Keep the full config: the spatial view needs its sensor geometry.
                if (!c.serial.empty()) sys::WriteTextFile(sys::DeviceConfigPath(c.serial), text);
                cfg = std::move(c);
                cfgSource = ConfigSource::Device;
                configTries_ = 0;
            }
        }
        sys::DeleteFileUtf8(path);
        if (!cfg && !knownSerial.empty()) {
            auto c = devcfg::Parse(sys::ReadCachedDeviceConfig(knownSerial));
            if (c.ok && IEquals(c.serial, knownSerial)) {
                cfg = std::move(c);
                cfgSource = ConfigSource::SteamVrCache;
            }
        }
    }

    std::vector<std::string> info, version, battery;
    bool fetchInfo = online && (justOnline || wantDeviceInfo_.exchange(false) || now - lastInfo > 30.0);
    if (!online) wantDeviceInfo_ = false;
    if (fetchInfo) {
        lastInfo = now;
        s.Execute("deviceinfo", &info, 4.0, nullptr);
        s.Execute("version", &version, 4.0, nullptr);
    }
    int batteryEvery = svc_.store.With([](AppState& st) { return st.settings.batteryIntervalS; });
    bool fetchBattery = online && (justOnline || now - lastBattery > std::max(5, batteryEvery));
    if (fetchBattery) {
        lastBattery = now;
        s.Execute("battery", &battery, 3.0, nullptr);
    }
    std::optional<std::map<std::string, std::string>> roles;
    if (fetchInfo || needConfig) roles = svc_.TrackerRoles();

    std::string recordDevice, recordModel;
    svc_.store.With([&](AppState& st) {
        Receiver* r = st.Find(serial_);
        if (!r) return;
        if (imu) {
            r->imu = *imu;
            sample.imuRate = (float)imu->rateHz;
            sample.imuIntervalMs = (float)imu->intervalMs;
            sample.imuSigmaMs = (float)imu->sigmaMs;
            sample.grav = (float)imu->grav;
            sample.gravSigma = (float)imu->gravSigma;
        }
        r->usbRates = rates;
        for (auto& u : rates) {
            sample.usbRate += (float)u.perSec;
            sample.usbMaxIntervalMs = std::max(sample.usbMaxIntervalMs, (float)u.maxIntervalMs);
        }
        sample.gapSeconds = r->pendingGap;
        r->pendingGap = 0;
        Store::PushSample(*r, sample);

        if (r->state != LinkState::Pairing && r->state != LinkState::Busy) r->state = online ? LinkState::Online : LinkState::Idle;

        if (online) {
            if (!r->device) r->device = DeviceInfo{};
            DeviceInfo& d = *r->device;
            if (cfg) {
                if (cfgSource == ConfigSource::Device && !cfg->serial.empty() && !d.serial.empty() && !IEquals(cfg->serial, d.serial)) {
                    // A different device than we assumed: start from a clean slate.
                    DeviceInfo fresh;
                    fresh.connectedSince = d.connectedSince;
                    d = std::move(fresh);
                }
                devcfg::ApplyTo(*cfg, d, cfgSource);
                if (cfgSource == ConfigSource::Device) {
                    Store::LogUnlocked(st, LogLevel::Info, serial_, "设备配置: " + d.serial + " · " + devcfg::ProductName(d) + (d.renderModel.empty() ? "" : " · " + d.renderModel));
                }
            }
            if (roles) {
                auto it = roles->find(d.serial);
                d.trackerRole = it == roles->end() ? std::string() : it->second;
            }
            // deviceinfo lists *every* device the console can see, including wired ones such as the
            // HMD's own lighthouse module (LHR-BB18B08E on a Pimax). Never take those, nor serials
            // that a connect event already tied to another receiver.
            std::set<std::string> exclude;
            for (auto& o : st.receivers) {
                if (o.kind == ReceiverKind::Wired) exclude.insert(o.serial);
                if (&o != r && o.device && o.device->serialFromEvent && !o.device->serial.empty()) exclude.insert(o.device->serial);
            }
            auto pick = [&](const std::vector<std::string>& lines) -> std::string {
                for (auto& sn : parser::FindLhrSerials(lines))
                    if (!exclude.count(sn)) return sn;
                return {};
            };
            if (!d.serialFromEvent) {
                std::string ser = pick(version);   // version is per active device: most reliable
                if (ser.empty()) ser = pick(info);
                if (!ser.empty()) d.serial = ser;
            }
            if (!info.empty()) {
                d.rawDeviceInfo = info;
                // Only keep the block that belongs to our device.
                auto section = SectionFor(info, d.serial);
                auto kv = parser::ParseKeyValues(section);
                d.props = kv;
                std::string model = parser::GuessModel(kv, d.serial);
                if (!model.empty() && d.configSource == ConfigSource::None) d.model = model;
            }
            if (!version.empty()) {
                d.rawVersion = version;
                auto kv = parser::ParseKeyValues(version);
                for (auto& p : kv) {
                    // "Watchman Version: 1592875850 watchman@valvebuilder01 2020-06-23 FPGA 538(2.26/7/2) BL 0 VRC … Radio …"
                    if (d.firmware.empty() && Contains(p.first, "Watchman Version")) {
                        static const std::regex re(R"(^(\d+)\s+\S+\s+(\d{4}-\d{2}-\d{2}))");
                        std::smatch m;
                        std::string v = p.second;
                        d.firmware = std::regex_search(v, m, re) ? m.str(1) + "（" + m.str(2) + "）" : v;
                    }
                    if (d.firmware.empty() && (Contains(p.first, "Firmware") || Contains(p.first, "firmware") || Contains(p.second, "Firmware")))
                        d.firmware = p.second.empty() ? p.first : p.second;
                    if (d.hardware.empty() && (Contains(p.first, "Hardware") || Contains(p.first, "hardware"))) d.hardware = p.second;
                    if (d.hardware.empty() && StartsWith(Trim(p.second), "0x")) d.hardware = std::string(Trim(p.second));   // "0x110e0109 product 17 rev …"
                }
            }
            if (d.configSource != ConfigSource::None) {
                if (d.deviceClass != DeviceClass::Unknown) r->pairedClass = d.deviceClass;   // class / hand come from the config
            } else if (!info.empty() || !version.empty() || d.deviceClass == DeviceClass::Unknown) {
                d.deviceClass = parser::ClassifyDevice(d, r->pairedModel);
                if (d.model.empty() && d.deviceClass != DeviceClass::Unknown) d.model = ToString(d.deviceClass);
                if (d.deviceClass != DeviceClass::Unknown) r->pairedClass = d.deviceClass;
                if (d.deviceClass == DeviceClass::IndexController) d.hand = parser::DetectHand(d);
            }
            if (!battery.empty()) {
                auto b = parser::ParseBattery(battery);
                d.batteryPct = b.percent;
                d.charging = b.charging;
                d.batteryText = b.notProvided ? "设备未提供电量" : b.text;
            }
            if (!d.serial.empty() && (r->pairedSerial != d.serial || r->pairedModel != d.model)) {
                r->pairedSerial = d.serial;
                r->pairedModel = d.model;
                r->pairingSource = d.serialFromEvent ? PairingSource::Device : PairingSource::Session;
                r->pairedLastSeen = UnixNow();
                recordDevice = d.serial;
                recordModel = d.model;
            }
            if (!d.serial.empty()) r->pairedLastSeen = UnixNow();
        }
    });
    if (!recordDevice.empty()) svc_.History().RecordPairing(serial_, recordDevice, recordModel, "监控观察");
}

void Monitor::OnLine(const std::string& line) { svc_.HandleConsoleLine(serial_, line, true); }

// ---------------------------------------------------------------------------- service impl

void RealService::HandleConsoleLine(const std::string& receiver, const std::string& line, bool fromMonitor) {
    auto ev = parser::Classify(line);
    if (!fromMonitor) {
        std::scoped_lock lk(opLinesMtx_);
        opLines_.push_back(line);
    }
    // Poll responses are already turned into live stats; logging them would flood the log
    // (11 receivers x several lines per second) and push real events out of the buffer.
    if (std::holds_alternative<std::monostate>(ev) && parser::IsPollNoise(line)) return;
    std::string recordDevice;
    std::string recordReceiver;
    bool needConfig = false;
    store.With([&](AppState& s) {
        LogLevel lvl = std::holds_alternative<std::monostate>(ev) ? LogLevel::Debug : LogLevel::Event;
        if (parser::LooksLikeError(line)) lvl = LogLevel::Warning;
        Store::LogUnlocked(s, lvl, receiver, line);
        double now = NowSeconds();
        auto target = [&](const std::string& ser) -> Receiver* {
            if (auto* r = s.Find(ser)) return r;
            return s.Find(receiver);
        };
        std::visit(
            [&](auto&& e) {
                using T = std::decay_t<decltype(e)>;
                if constexpr (std::is_same_v<T, parser::EvKeepaliveGap>) {
                    if (auto* r = target(e.receiver)) {
                        char b[96];
                        std::snprintf(b, sizeof b, "无线丢包 %.3f 秒 (keepalive %d/%d)", e.seconds, e.a, e.b);
                        Store::PushEvent(*r, {now, UnixNow(), SignalEvent::Type::Gap, (float)e.seconds, b});
                    }
                } else if constexpr (std::is_same_v<T, parser::EvDeviceConnected>) {
                    if (auto* r = target(e.receiver)) {
                        Store::PushEvent(*r, {now, UnixNow(), SignalEvent::Type::Connected, 0, e.device + " 已连接"});
                        if (!r->device) r->device = DeviceInfo{};
                        if (!r->device->serial.empty() && !IEquals(r->device->serial, e.device)) r->device = DeviceInfo{};   // another device
                        if (r->device->configSource != ConfigSource::Device) needConfig = true;
                        r->device->serial = e.device;
                        r->device->serialFromEvent = true;
                        r->device->connectedSince = UnixNow();
                        r->pairedSerial = e.device;
                        r->pairingSource = PairingSource::Device;
                        r->pairedLastSeen = UnixNow();
                        recordDevice = e.device;
                        recordReceiver = r->serial;
                    }
                } else if constexpr (std::is_same_v<T, parser::EvDeviceDisconnected>) {
                    if (auto* r = target(e.receiver)) Store::PushEvent(*r, {now, UnixNow(), SignalEvent::Type::Disconnected, 0, e.device + " 已断开"});
                } else if constexpr (std::is_same_v<T, parser::EvTrackingReset>) {
                    if (auto* r = s.Find(receiver)) Store::PushEvent(*r, {now, UnixNow(), SignalEvent::Type::TrackingReset, 0, "追踪重置: " + e.reason});
                } else if constexpr (std::is_same_v<T, parser::EvBootstrapped>) {
                    if (auto* r = s.Find(receiver)) Store::PushEvent(*r, {now, UnixNow(), SignalEvent::Type::Bootstrapped, 0, e.text});
                } else if constexpr (std::is_same_v<T, parser::EvPairing>) {
                    if (auto* r = s.Find(receiver)) {
                        auto t = e.kind == parser::EvPairing::Kind::Success ? SignalEvent::Type::PairingSuccess
                                 : e.kind == parser::EvPairing::Kind::Failed ? SignalEvent::Type::PairingFailed
                                                                             : SignalEvent::Type::PairingStart;
                        Store::PushEvent(*r, {now, UnixNow(), t, 0, e.text});
                    }
                }
            },
            ev);
    });
    if (!recordDevice.empty()) {
        history_.RecordPairing(recordReceiver, recordDevice, "", "连接事件");
        if (auto* m = GetMonitor(recordReceiver)) {
            if (needConfig) m->RequestConfig();
            else m->RequestDeviceInfo();
        }
    }
}

void RealService::WorkerLoop(std::stop_token st) {
    double lastConflict = -100, lastSave = NowSeconds();
    while (!st.stop_requested()) {
        std::vector<Command> batch;
        {
            std::unique_lock lk(qMtx_);
            cv_.wait_for(lk, st, std::chrono::milliseconds(250), [&] { return !queue_.empty(); });
            while (!queue_.empty()) {
                batch.push_back(std::move(queue_.front()));
                queue_.pop();
            }
        }
        for (auto& c : batch) Handle(c);
        double now = NowSeconds();
        if (usbDirty_ && now - usbDirtyAt_ > 0.8) {
            usbDirty_ = false;
            store.Log(LogLevel::Info, "main", "检测到 USB 设备变化，重新扫描");
            StartRefresh();
        }
        if (now - lastConflict > 3.0) {
            lastConflict = now;
            ScanConflicts();
        }
        EnsureMonitors();
        if (now - lastSave > 10.0) {
            lastSave = now;
            history_.Save();
        }
    }
}

void RealService::Handle(const Command& c) {
    using T = Command::Type;
    switch (c.type) {
        case T::Refresh: StartRefresh(); break;
        case T::StartMonitor:
            manualOff_[c.serial] = false;
            StartMonitor(c.serial);
            break;
        case T::StopMonitor:
            manualOff_[c.serial] = true;
            StopMonitor(c.serial);
            break;
        case T::MonitorAll:
            manualOff_.clear();
            store.With([](AppState& s) { s.settings.autoMonitorAll = true; });
            history_.SaveSettings(store.With([](AppState& s) { return s.settings; }));
            break;
        case T::StopAllMonitors:
            store.With([](AppState& s) { s.settings.autoMonitorAll = false; });
            history_.SaveSettings(store.With([](AppState& s) { return s.settings; }));
            StopAllMonitors();
            break;
        case T::ClearStats:
            store.With([&](AppState& s) {
                for (auto& r : s.receivers) {
                    if (!c.serial.empty() && r.serial != c.serial) continue;
                    r.samples.clear();
                    r.events.clear();
                    r.gapCount = r.disconnectCount = r.connectCount = r.resetCount = 0;
                    r.worstGap = 0;
                }
            });
            break;
        case T::Pair: StartOp(OpKind::Pair, c.serial, c.text, c.seconds > 0 ? c.seconds : 30.0); break;
        case T::Unpair: StartOp(OpKind::Unpair, c.serial, "", 15); break;
        case T::FactoryReset: StartOp(OpKind::FactoryReset, c.serial, "", 15); break;
        case T::Identify: StartOp(OpKind::Identify, c.serial, "", 10); break;
        case T::PowerOff: StartOp(OpKind::PowerOff, c.serial, "", 10); break;
        case T::CancelOp: opCancel_ = true; break;
        case T::ReloadDevice:
            if (auto* m = GetMonitor(c.serial)) m->RequestConfig();
            break;
        case T::SetFastPoll:
            store.With([&](AppState& s) { s.fastPoll = c.value != 0; });
            break;
        case T::DismissOp:
            store.With([](AppState& s) {
                if (!s.op.Active()) s.op = Operation{};
            });
            break;
        case T::KillConflicts: {
            auto list = sys::FindConflicts(GetCurrentProcessId());
            int killed = 0;
            std::string errors;
            // SteamVR: kill monitor first so it does not respawn the server.
            std::stable_sort(list.begin(), list.end(), [](auto& a, auto& b) { return a.exe == "vrmonitor.exe" && b.exe != "vrmonitor.exe"; });
            for (auto& p : list) {
                if (!c.text.empty() && p.category != c.text) continue;
                auto e = sys::KillProcess(p.pid);
                if (e.empty()) ++killed;
                else errors += p.exe + ": " + e + "\n";
                store.Log(e.empty() ? LogLevel::Info : LogLevel::Warning, "main", "结束 " + p.exe + " (" + std::to_string(p.pid) + ") " + (e.empty() ? "成功" : e));
            }
            store.AddToast(errors.empty() ? Toast::Kind::Success : Toast::Kind::Warning, "结束冲突进程",
                           "已结束 " + std::to_string(killed) + " 个进程" + (errors.empty() ? "" : "\n" + errors));
            ScanConflicts();
            break;
        }
        case T::SetNickname:
            history_.SetNickname(c.serial, c.text);
            store.With([&](AppState& s) {
                if (auto* r = s.Find(c.serial)) r->nickname = c.text;
            });
            history_.Save();
            break;
        case T::SetNote:
            history_.SetNote(c.serial, c.text);
            store.With([&](AppState& s) {
                if (auto* r = s.Find(c.serial)) r->note = c.text;
            });
            history_.Save();
            break;
        case T::SetKind:
            history_.SetKindOverride(c.serial, c.value);
            history_.Save();
            StartRefresh();
            break;
        case T::SetDeviceNickname:
            history_.SetDeviceNickname(c.serial, c.text);
            history_.Save();
            break;
        case T::SaveSettings: {
            auto s = store.With([](AppState& st) { return st.settings; });
            history_.SaveSettings(s);
            std::string p = sys::FindLighthouseConsole(s.consolePathOverride);
            if (!p.empty() && p != consolePath_) {
                consolePath_ = p;
                store.With([&](AppState& st) {
                    st.consolePath = p;
                    st.backendError.clear();
                });
                StopAllMonitors();
                StartRefresh();
            }
            break;
        }
        case T::RawCommand: {
            std::string serial = c.serial, cmd = c.text;
            std::thread([this, serial, cmd] {
                std::vector<std::string> out;
                std::string err;
                store.Log(LogLevel::Command, serial, "lh> " + cmd);
                bool ok = false;
                if (auto* m = GetMonitor(serial)) ok = m->Exec(cmd, &out, 10.0, &err);
                else ok = RunSimpleDongleCommand(serial, cmd, out, err);
                for (auto& l : out) store.Log(LogLevel::Info, serial, l);
                if (!ok) store.Log(LogLevel::Error, serial, err);
            }).detach();
            break;
        }
    }
}

void RealService::StartRefresh() {
    if (consolePath_.empty()) {
        // Still show USB data.
        MergeUsb(EnumerateUsbReceivers());
        return;
    }
    if (refreshing_.exchange(true)) {
        refreshAgain_ = true;
        return;
    }
    if (refreshThread_.joinable()) refreshThread_.join();
    refreshThread_ = std::jthread([this](std::stop_token) {
        do {
            refreshAgain_ = false;
            DoRefresh();
        } while (refreshAgain_ && !shutdown_);
        refreshing_ = false;
    });
}

void RealService::DoRefresh() {
    store.With([](AppState& s) { s.refreshing = true; });
    MergeUsb(EnumerateUsbReceivers());

    ConsoleSession session(ConsoleExe(), "", [this](const std::string& l) {
        if (!parser::IsPollNoise(l)) store.Log(LogLevel::Debug, "refresh", l);
    });
    std::string err;
    std::vector<std::string> receiversListed;
    std::vector<parser::DongleEntry> dongles;
    std::string version;
    if (session.Start(&err, 12.0)) {
        for (auto& l : session.StartupLines())
            if (auto v = parser::ParseConsoleVersion(l)) version = *v;
        receiversListed = parser::ParseReceiverList(session.StartupLines());
        std::vector<std::string> out;
        if (session.Execute("dongleinfo", &out, 8.0, &err))
            for (auto& l : out)
                if (auto d = parser::ParseDongleInfo(l); !d.empty()) dongles = d;
        session.Stop();
    } else {
        store.Log(LogLevel::Error, "main", "刷新失败: " + err);
    }

    std::vector<Receiver> created;
    store.With([&](AppState& s) {
        if (!version.empty()) s.consoleVersion = version;
        for (auto& ser : receiversListed) {
            Receiver* r = s.Find(ser);
            if (!r) {
                Receiver nr;
                nr.serial = ser;
                nr.kind = parser::IsWiredDeviceSerial(ser) ? ReceiverKind::Wired
                          : Contains(ser, "-")             ? ReceiverKind::VrRadio
                                                           : ReceiverKind::WatchmanDongle;
                history_.ApplyTo(nr);
                s.receivers.push_back(std::move(nr));
                r = &s.receivers.back();
            }
            r->seenByConsole = true;
            r->lastSeen = UnixNow();
            if (r->state == LinkState::Unknown) r->state = LinkState::Idle;
        }
        for (auto& d : dongles) {
            if (Receiver* r = s.Find(d.serial)) {
                r->radioVersion = d.radioVersion;
                r->dongleInfoExtra = d.extra;
                // If the extra field holds a device serial, treat it as authoritative pairing info.
                for (auto& x : d.extra)
                    if (StartsWith(x, "LHR-")) {
                        r->pairedSerial = x;
                        r->pairingSource = PairingSource::Device;
                    }
            }
        }
        // Wired devices are never "paired" to a receiver: purge any record that says so
        // (older versions mistook the HMD's lighthouse module for the paired device).
        for (auto& w : s.receivers) {
            if (w.kind != ReceiverKind::Wired) continue;
            history_.ForgetDevice(w.serial);
            for (auto& r : s.receivers) {
                if (IEquals(r.pairedSerial, w.serial)) {
                    r.pairedSerial.clear();
                    r.pairedModel.clear();
                    r.pairingSource = PairingSource::None;
                }
                if (r.device && IEquals(r.device->serial, w.serial) && !r.device->serialFromEvent) r.device->serial.clear();
            }
        }
        for (auto& r : s.receivers)
            if (r.kind != ReceiverKind::Wired) history_.Observe(r);
        Store::SortReceivers(s);
        s.lastRefresh = UnixNow();
        s.backendReady = true;
        s.refreshing = false;
        Store::LogUnlocked(s, LogLevel::Info, "main",
                           "刷新完成: " + std::to_string(receiversListed.size()) + " 个接收器 (console), " + std::to_string(dongles.size()) + " 条 dongleinfo");
    });

    // Offline devices: take the product name from SteamVR's cached device config, if it has one.
    std::vector<std::string> need;
    store.With([&](AppState& s) {
        for (auto& r : s.receivers)
            if (r.kind != ReceiverKind::Wired && !r.pairedSerial.empty() && (r.pairedModel.empty() || r.pairedClass == DeviceClass::Unknown))
                need.push_back(r.pairedSerial);
    });
    std::map<std::string, devcfg::DeviceConfig> cached;
    for (auto& sn : need)
        if (auto c = devcfg::Parse(sys::ReadCachedDeviceConfig(sn)); c.ok && IEquals(c.serial, sn)) cached[sn] = std::move(c);
    if (!cached.empty()) {
        store.With([&](AppState& s) {
            for (auto& r : s.receivers) {
                auto it = cached.find(r.pairedSerial);
                if (it == cached.end()) continue;
                DeviceInfo t;
                devcfg::ApplyTo(it->second, t, ConfigSource::SteamVrCache);
                if (!t.model.empty()) r.pairedModel = t.model;
                if (t.deviceClass != DeviceClass::Unknown) r.pairedClass = t.deviceClass;
            }
        });
    }
}

void RealService::MergeUsb(const std::vector<UsbReceiver>& list) {
    store.With([&](AppState& s) {
        for (auto& r : s.receivers) r.usb.present = false;
        for (auto& u : list) {
            Receiver* r = s.Find(u.serial);
            if (!r) {
                Receiver nr;
                nr.serial = u.serial;
                history_.ApplyTo(nr);
                s.receivers.push_back(std::move(nr));
                r = &s.receivers.back();
            }
            r->pid = u.pid;
            r->usb = u.info;
            if (!r->kindOverridden) r->kind = ClassifyReceiver(u.pid, u.info, u.serial);
            r->lastSeen = UnixNow();
            if (r->firstSeen == 0) r->firstSeen = UnixNow();
        }
        for (auto& r : s.receivers)
            if (!r.usb.present && r.state != LinkState::Pairing) {
                r.state = LinkState::Unknown;
                r.device.reset();
            }
        Store::SortReceivers(s);
    });
}

void RealService::EnsureMonitors() {
    if (consolePath_.empty()) return;
    struct Want {
        std::string serial;
        bool on;
    };
    std::vector<Want> wants;
    store.With([&](AppState& s) {
        for (auto& r : s.receivers) {
            // Wired devices (HMD lighthouse module etc.) are not receivers: never open sessions on them.
            bool on = s.settings.autoMonitorAll && r.usb.present && r.seenByConsole && r.kind != ReceiverKind::Wired;
            wants.push_back({r.serial, on});
        }
    });
    std::vector<std::unique_ptr<Monitor>> doomed;   // destroyed outside the lock (their threads may call back into us)
    {
        std::scoped_lock lk(monMtx_);
        for (auto& w : wants) {
            bool exists = monitors_.count(w.serial) > 0;
            bool paused = paused_[w.serial];
            bool off = manualOff_.count(w.serial) && manualOff_[w.serial];
            bool manualOn = manualOff_.count(w.serial) && !manualOff_[w.serial];
            bool want = (w.on && !off) || manualOn;
            if (paused) want = false;
            if (want && !exists) monitors_[w.serial] = std::make_unique<Monitor>(*this, w.serial);
            else if (!want && exists) {
                doomed.push_back(std::move(monitors_[w.serial]));
                monitors_.erase(w.serial);
            }
        }
        for (auto it = monitors_.begin(); it != monitors_.end();) {
            bool still = std::any_of(wants.begin(), wants.end(), [&](auto& w) { return w.serial == it->first; });
            if (still) {
                ++it;
            } else {
                doomed.push_back(std::move(it->second));
                it = monitors_.erase(it);
            }
        }
    }
    doomed.clear();
}

void RealService::StartMonitor(const std::string& serial) {
    std::scoped_lock lk(monMtx_);
    if (!monitors_.count(serial) && !paused_[serial]) monitors_[serial] = std::make_unique<Monitor>(*this, serial);
}

void RealService::StopMonitor(const std::string& serial) {
    std::unique_ptr<Monitor> m;
    {
        std::scoped_lock lk(monMtx_);
        auto it = monitors_.find(serial);
        if (it == monitors_.end()) return;
        m = std::move(it->second);
        monitors_.erase(it);
    }
    m.reset();
}

void RealService::StopAllMonitors() {
    std::map<std::string, std::unique_ptr<Monitor>> all;
    {
        std::scoped_lock lk(monMtx_);
        all.swap(monitors_);
    }
    all.clear();
}

Monitor* RealService::GetMonitor(const std::string& serial) {
    std::scoped_lock lk(monMtx_);
    auto it = monitors_.find(serial);
    return it == monitors_.end() ? nullptr : it->second.get();
}

void RealService::ScanConflicts() {
    auto list = sys::FindConflicts(GetCurrentProcessId());
    store.With([&](AppState& s) { s.conflicts = std::move(list); });
}

// ---------------------------------------------------------------------------- operations

void RealService::SetOp(uint64_t id, OpPhase phase, const std::string& msg) {
    store.With([&](AppState& s) {
        if (s.op.id != id) return;
        s.op.phase = phase;
        if (!msg.empty()) s.op.message = msg;
    });
}

void RealService::StartOp(OpKind kind, const std::string& serial, const std::string& deviceType, double timeout) {
    bool busy = store.With([](AppState& s) { return s.op.Active(); });
    if (busy) {
        store.AddToast(Toast::Kind::Warning, "有操作正在进行", "请等待当前操作完成或取消。");
        return;
    }
    if (consolePath_.empty()) {
        store.AddToast(Toast::Kind::Error, "无法执行", "找不到 lighthouse_console.exe");
        return;
    }
    if (opThread_.joinable()) opThread_.join();
    uint64_t id = ++opSeq_;
    opCancel_ = false;
    store.With([&](AppState& s) {
        s.op = Operation{};
        s.op.kind = kind;
        s.op.phase = OpPhase::Preparing;
        s.op.target = serial;
        s.op.deviceType = deviceType;
        s.op.startedT = NowSeconds();
        s.op.timeoutS = timeout;
        s.op.id = id;
        s.op.message = "准备中…";
    });
    opThread_ = std::thread([=, this] { RunOp(kind, serial, deviceType, timeout, id); });
}

bool RealService::RunSimpleDongleCommand(const std::string& serial, const std::string& cmd, std::vector<std::string>& out, std::string& err) {
    ConsoleSession s(ConsoleExe(), serial, [this, serial](const std::string& l) { HandleConsoleLine(serial, l, false); });
    if (!s.Start(&err, 10.0)) return false;
    bool ok = s.Execute(cmd, &out, 10.0, &err);
    s.Stop();
    return ok;
}

void RealService::RunOp(OpKind kind, std::string serial, std::string deviceType, double timeout, uint64_t id) {
    auto fail = [&](const std::string& msg) {
        SetOp(id, OpPhase::Failed, msg);
        store.AddToast(Toast::Kind::Error, "操作失败", msg, 8);
        store.Log(LogLevel::Error, serial, msg);
    };
    const bool writeOp = kind == OpKind::Pair || kind == OpKind::Unpair || kind == OpKind::FactoryReset;
    const char* opName = kind == OpKind::Pair ? "pair" : kind == OpKind::Unpair ? "unpair" : kind == OpKind::FactoryReset ? "donglereset"
                         : kind == OpKind::Identify ? "identifycontroller" : "poweroff";

    // Identify / power off reuse the monitor session when available.
    if (!writeOp) {
        std::vector<std::string> out;
        std::string err;
        SetOp(id, OpPhase::Running, std::string("执行 ") + opName);
        store.Log(LogLevel::Command, serial, std::string("lh> ") + opName);
        bool ok = false;
        if (auto* m = GetMonitor(serial)) ok = m->Exec(opName, &out, 8.0, &err);
        else ok = RunSimpleDongleCommand(serial, opName, out, err);
        for (auto& l : out) store.Log(LogLevel::Info, serial, l);
        bool errLine = std::any_of(out.begin(), out.end(), [](auto& l) { return parser::LooksLikeError(l); });
        if (!ok || errLine) fail(ok ? out.back() : err);
        else {
            SetOp(id, OpPhase::Succeeded, kind == OpKind::Identify ? "已发送震动指令" : "已发送关机指令");
            store.AddToast(Toast::Kind::Success, kind == OpKind::Identify ? "识别" : "关机", "命令已发送到 " + serial);
        }
        return;
    }

    // Backup first.
    std::string backup = history_.WriteBackup(store.With([](AppState& s) { return s; }), opName, serial);
    store.With([&](AppState& s) {
        if (s.op.id == id) s.op.backupFile = backup;
        if (auto* r = s.Find(serial)) r->state = kind == OpKind::Pair ? LinkState::Pairing : LinkState::Busy;
    });
    store.Log(LogLevel::Info, serial, "已备份当前状态到 " + backup);

    // Reserve the receiver: stop its monitor.
    {
        std::scoped_lock lk(monMtx_);
        paused_[serial] = true;
    }
    StopMonitor(serial);
    auto release = [&] {
        {
            std::scoped_lock lk(monMtx_);
            paused_[serial] = false;
        }
        store.With([&](AppState& s) {
            if (auto* r = s.Find(serial)) r->state = LinkState::Idle;
        });
    };

    {
        std::scoped_lock lk(opLinesMtx_);
        opLines_.clear();
    }
    ConsoleSession session(ConsoleExe(), serial, [this, serial](const std::string& l) { HandleConsoleLine(serial, l, false); });
    std::string err;
    SetOp(id, OpPhase::Preparing, "连接接收器 " + serial + " …");
    if (!session.Start(&err, 12.0)) {
        release();
        fail("无法连接接收器: " + err);
        return;
    }
    bool matched = std::any_of(session.StartupLines().begin(), session.StartupLines().end(), [&](auto& l) {
        auto ev = parser::Classify(l);
        return std::holds_alternative<parser::EvSessionConnected>(ev) && IEquals(std::get<parser::EvSessionConnected>(ev).receiver, serial);
    });
    if (!matched) {
        session.Stop();
        release();
        fail("lighthouse_console 没有连接到 " + serial + "，已中止（不会对其他接收器执行操作）");
        return;
    }

    std::vector<std::string> out;
    SetOp(id, OpPhase::Running, std::string("发送 ") + opName);
    store.Log(LogLevel::Command, serial, std::string("lh> ") + opName);
    bool ok = session.Execute(opName, &out, 10.0, &err);
    auto errLine = std::find_if(out.begin(), out.end(), [](auto& l) {
        auto ev = parser::Classify(l);
        return std::holds_alternative<parser::EvCommandError>(ev) ||
               (std::holds_alternative<parser::EvPairing>(ev) && std::get<parser::EvPairing>(ev).kind == parser::EvPairing::Kind::Failed);
    });
    if (!ok || errLine != out.end()) {
        session.Stop();
        release();
        fail(ok ? *errLine : err);
        return;
    }

    if (kind == OpKind::Unpair || kind == OpKind::FactoryReset) {
        session.Stop();
        history_.ForgetPairing(serial);
        history_.Save();
        store.With([&](AppState& s) {
            if (auto* r = s.Find(serial)) {
                r->pairedSerial.clear();
                r->pairedModel.clear();
                r->pairingSource = PairingSource::None;
                r->device.reset();
            }
        });
        release();
        std::string msg = kind == OpKind::Unpair ? "已清除 " + serial + " 的配对" : serial + " 已恢复出厂设置";
        SetOp(id, OpPhase::Succeeded, msg);
        store.AddToast(Toast::Kind::Success, kind == OpKind::Unpair ? "解除配对完成" : "恢复出厂完成", msg, 6);
        StartRefresh();
        return;
    }

    // ---- Pair: wait for the device.
    SetOp(id, OpPhase::WaitingForDevice, "接收器已进入配对模式，请让设备进入配对状态");
    double t0 = NowSeconds();
    bool success = false, failed = false;
    std::string failText, device;
    double lastPoll = 0;
    while (!opCancel_ && !shutdown_) {
        double el = NowSeconds() - t0;
        if (el > timeout) break;
        {
            std::scoped_lock lk(opLinesMtx_);
            for (auto& l : opLines_) {
                auto ev = parser::Classify(l);
                if (auto* p = std::get_if<parser::EvPairing>(&ev)) {
                    if (p->kind == parser::EvPairing::Kind::Success) success = true;
                    if (p->kind == parser::EvPairing::Kind::Failed) {
                        failed = true;
                        failText = p->text;
                    }
                }
                if (auto* c = std::get_if<parser::EvDeviceConnected>(&ev)) {
                    success = true;
                    device = c->device;
                }
            }
            opLines_.clear();
        }
        if (success || failed) break;
        if (NowSeconds() - lastPoll > 2.0) {
            lastPoll = NowSeconds();
            std::vector<std::string> imuOut;
            if (session.Execute("imustats", &imuOut, 3.0, nullptr))
                for (auto& l : imuOut)
                    if (auto imu = parser::ParseImuStats(l); imu && imu->rateHz >= kOnlineImuHz) success = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    if (opCancel_) {
        session.Stop();
        release();
        SetOp(id, OpPhase::Cancelled, "已取消配对");
        store.AddToast(Toast::Kind::Info, "已取消", "配对已取消");
        return;
    }
    if (!success) {
        session.Stop();
        release();
        fail(failed ? "配对失败: " + failText : "配对超时，未检测到设备。请确认设备处于配对模式（指示灯蓝色闪烁）后重试。");
        return;
    }

    SetOp(id, OpPhase::Verifying, "设备已连接，正在读取设备信息并校验…");
    std::vector<std::string> info, version;
    session.Execute("deviceinfo", &info, 5.0, nullptr);
    session.Execute("version", &version, 5.0, nullptr);
    session.Stop();
    if (device.empty()) {
        // Skip wired devices (HMD lighthouse module) and devices already bound to other receivers.
        std::set<std::string> exclude = store.With([&](AppState& s) {
            std::set<std::string> ex;
            for (auto& o : s.receivers) {
                if (o.kind == ReceiverKind::Wired) ex.insert(o.serial);
                if (!IEquals(o.serial, serial) && o.device && o.device->serialFromEvent) ex.insert(o.device->serial);
            }
            return ex;
        });
        for (auto* src : {&version, &info})
            for (auto& sn : parser::FindLhrSerials(*src))
                if (device.empty() && !exclude.count(sn)) device = sn;
    }

    std::string model = parser::GuessModel(parser::ParseKeyValues(SectionFor(info, device)), device);
    DeviceInfo probe;
    probe.model = model;
    probe.rawDeviceInfo = SectionFor(info, device);
    probe.rawVersion = version;
    DeviceClass cls = parser::ClassifyDevice(probe, deviceType);
    probe.deviceClass = cls;
    if (cls == DeviceClass::IndexController) probe.hand = parser::DetectHand(probe);
    if (cls != DeviceClass::Unknown) model = DeviceDisplayModel(probe);
    history_.RecordPairing(serial, device.empty() ? "(未知序列号)" : device, model, "配对向导");
    history_.Save();
    store.With([&](AppState& s) {
        if (auto* r = s.Find(serial)) {
            r->pairedSerial = device;
            r->pairedModel = model;
            r->pairedClass = cls;
            r->pairingSource = device.empty() ? PairingSource::Session : PairingSource::Device;
            r->pairedLastSeen = UnixNow();
        }
        if (s.op.id == id) s.op.resultDevice = device;
    });
    release();
    SetOp(id, OpPhase::Succeeded, device.empty() ? "配对成功（未能读到设备序列号）" : "配对成功: " + device + " → " + serial);
    store.AddToast(Toast::Kind::Success, "配对成功", (device.empty() ? std::string("新设备") : device) + " 已配对到 " + serial, 8);
    StartRefresh();
}

std::unique_ptr<IService> CreateRealService(Store& store) { return std::make_unique<RealService>(store); }

}  // namespace lhp

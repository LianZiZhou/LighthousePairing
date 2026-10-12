// Demo backend: synthetic receivers and signals (mirrors a real Pimax + Index + 3x Tracker setup).
// Used for UI development, screenshots and when no hardware is attached.
#include "backend/ConsoleParser.h"
#include "backend/SystemInfo.h"
#include "core/Service.h"
#include "util/Text.h"

#include <cmath>
#include <random>
#include <thread>

namespace lhp {

namespace {

struct DemoDevice {
    const char* receiver;
    ReceiverKind kind;
    uint16_t pid;
    uint32_t radio;
    const char* device;   // nullptr = idle
    const char* model;
    const char* nickname;
    float baseRate;       // Hz
    float gapProb;        // per second
    int battery;
    const char* location;
    int port;
};

const DemoDevice kDevices[] = {
    {"109031C053", ReceiverKind::HmdBuiltin, 0x2101, 1525468496, "LHR-9E63AEE0", "Knuckles Right", "右手", 1000.f, 0.35f, 71, "Port_#0002.Hub_#0007", 2},
    {"18848BA78B", ReceiverKind::HmdBuiltin, 0x2101, 1525468496, "LHR-F0F4B423", "Knuckles Left", "左手", 1000.f, 0.06f, 84, "Port_#0001.Hub_#0007", 1},
    {"A04165BAAA-RYB", ReceiverKind::VrRadio, 0x2102, 1558748372, nullptr, nullptr, "备用", 0, 0, 0, "Port_#0004.Hub_#0003", 4},
    {"D1E1CD00DA-RYB", ReceiverKind::VrRadio, 0x2102, 1558748372, "LHR-0FCD6B1F", "VIVE Tracker 3.0 MV", "腰", 250.f, 0.02f, 63, "Port_#0001.Hub_#0003", 1},
    {"F51A4164A2-RYB", ReceiverKind::VrRadio, 0x2102, 1558748372, "LHR-6CF7103B", "VIVE Tracker 3.0 MV", "左脚", 250.f, 0.03f, 58, "Port_#0002.Hub_#0003", 2},
    {"68C52C2C3A-RYB", ReceiverKind::VrRadio, 0x2102, 1558748372, "LHR-4569CFC1", "Tundra Tracker", "右脚", 250.f, 0.12f, 46, "Port_#0003.Hub_#0003", 3},
    {"93F15DFEA6", ReceiverKind::WatchmanDongle, 0x2101, 1461100729, nullptr, nullptr, "", 0, 0, 0, "Port_#0003.Hub_#0001", 3},
};

}  // namespace

class DemoService final : public IService {
public:
    explicit DemoService(Store& s) : IService(s), history_(sys::DataDir() + "\\demo") {}
    ~DemoService() override { Shutdown(); }

    void Start() override {
        history_.Load();
        std::scoped_lock lk(store.mutex);
        auto& s = store.state;
        s.demo = true;
        s.consolePath = "C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR\\tools\\lighthouse\\bin\\win64\\lighthouse_console.exe";
        s.consoleVersion = "11000037";
        s.steamVrRuntime = "C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR";
        s.dataDir = sys::DataDir();
        s.backendReady = true;
        s.lastRefresh = UnixNow();
        s.conflicts = {{36368, "pi_server.exe", "Pimax", false}};
        double now = NowSeconds();
        for (auto& d : kDevices) {
            Receiver r;
            r.serial = d.receiver;
            r.kind = d.kind;
            r.pid = d.pid;
            r.radioVersion = d.radio;
            r.seenByConsole = true;
            r.nickname = d.nickname;
            r.firstSeen = UnixNow() - 86400 * 21;
            r.lastSeen = UnixNow();
            r.monitor = MonitorState::Running;
            UsbInfo& u = r.usb;
            u.present = true;
            u.instanceId = std::string("USB\\VID_28DE&PID_") + (d.pid == 0x2101 ? "2101\\" : "2102\\") + d.receiver;
            u.friendlyName = d.pid == 0x2101 ? "Watchman Dongle" : "Valve VR Radio";
            u.busDescription = u.friendlyName;
            u.manufacturer = "Valve Software";
            u.locationInfo = d.location;
            u.address = (uint32_t)d.port;
            u.driverVersion = "10.0.26100.1";
            u.driverDate = "2006-06-21";
            u.service = "HidUsb";
            u.hidInterfaces = {u.instanceId + "&MI_00 (HID)"};
            if (d.kind == ReceiverKind::HmdBuiltin) {
                u.behindHmd = true;
                u.hmdName = "Pimax Dream Air (34A4:0044)";
                u.locationPath = "PCIROOT(0)#PCI(0801)#PCI(0003)#USBROOT(0)#USB(4)#USB(1)#USB(" + std::to_string(d.port) + ")";
                u.parents = {{"USB\\VID_05E3&PID_0610\\6&2A1B", "Generic USB Hub (头显内部)", 0x05E3, 0x0610},
                             {"USB\\VID_34A4&PID_0044\\P501G0D202642800234", "Pimax Dream Air", 0x34A4, 0x0044},
                             {"USB\\ROOT_HUB30\\5&1E2F", "USB 根集线器(USB 3.0)", 0, 0}};
                u.siblings = {{"USB\\VID_34A4&PID_0044&MI_04\\7&8FA3", "Pimax Dream Air 麦克风", 0x34A4, 0x0044}};
            } else {
                u.locationPath = "PCIROOT(0)#PCI(0201)#PCI(0000)#USBROOT(0)#USB(2)#USB(" + std::to_string(d.port) + ")";
                u.parents = {{"USB\\VID_2109&PID_2817\\5&3C0A", "通用 USB 集线器 (VIA Labs)", 0x2109, 0x2817},
                             {"USB\\ROOT_HUB30\\4&2D9B", "USB 根集线器(USB 3.0)", 0, 0}};
            }
            if (d.device) {
                r.state = LinkState::Online;
                r.pairedSerial = d.device;
                r.pairedModel = d.model;
                r.pairedClass = parser::ClassifyText(d.model);
                r.pairingSource = PairingSource::Device;
                r.pairedLastSeen = UnixNow();
                DeviceInfo di;
                di.serial = d.device;
                di.model = d.model;
                di.batteryPct = d.battery;
                di.charging = false;
                di.connectedSince = UnixNow() - 1800;
                di.deviceClass = parser::ClassifyText(d.model);
                bool knuckles = std::string(d.model).find("Knuckles") != std::string::npos;
                di.firmware = knuckles ? "1637337317 (watchman v3)" : "1652741219";
                // Real values: Knuckles Left 0x110e0109, Knuckles Right 0x110e0009, VIVE Tracker 3.0 0x84020109.
                bool rightHand = std::string(d.model).find("Right") != std::string::npos;
                di.hardware = knuckles ? (rightHand ? "0x110e0009" : "0x110e0109") : "0x84020109";
                di.batteryText = std::to_string(d.battery) + "%";
                di.props = {{"Serial", d.device},
                            {"Model", d.model},
                            {"Manufacturer", knuckles ? "Valve" : di.deviceClass == DeviceClass::TundraTracker ? "Tundra Labs" : "HTC"},
                            {"Hardware Revision", di.hardware},
                            {"Firmware Version", di.firmware},
                            {"FPGA Version", knuckles ? "538" : "572"},
                            {"Radio Version", "1558748372"},
                            {"VRC Version", knuckles ? "1637337317" : "1652741219"},
                            {"Tracking Firmware", knuckles ? "1637337317 lighthouse_tx_knuckles" : "1652741219 vive_tracker3"},
                            {"Sensor Count", knuckles ? "32" : "18"},
                            {"IMU", "BMI085 (Gyro range 3, Accel range 2)"},
                            {"Connected Dongle", d.receiver}};
                di.rawDeviceInfo = {std::string("Device ") + d.device + " (" + d.model + ")"};
                if (di.deviceClass == DeviceClass::IndexController) di.hand = parser::DetectHand(di);
                r.device = di;
                // pre-fill 5 minutes of history
                std::mt19937 rng((unsigned)std::hash<std::string>{}(d.receiver));
                for (int i = 300; i > 0; --i) Synth(r, d, now - i, rng, false);
                history_.RecordPairing(d.receiver, d.device, d.model, "演示数据");
            } else {
                r.state = LinkState::Idle;
                r.monitor = MonitorState::Running;
                if (std::string(d.receiver) == "93F15DFEA6") {
                    r.pairedSerial = "LHR-593F5386";
                    r.pairedModel = "VIVE Tracker (2018)";
                    r.pairingSource = PairingSource::History;
                    r.pairedLastSeen = UnixNow() - 86400 * 9;
                }
                for (int i = 300; i > 0; --i) {
                    SignalSample s0;
                    s0.t = now - i;
                    Store::PushSample(r, s0);
                }
            }
            s.receivers.push_back(std::move(r));
        }
        {
            // The HMD's own lighthouse module shows up in lighthouse_console's list too.
            Receiver w;
            w.serial = "LHR-BB18B08E";
            w.kind = ReceiverKind::Wired;
            w.seenByConsole = true;
            w.state = LinkState::Idle;
            s.receivers.push_back(std::move(w));
        }
        Store::SortReceivers(s);
        Store::LogUnlocked(s, LogLevel::Info, "main", "演示模式：使用模拟数据（不会访问任何真实接收器）");
        Store::LogUnlocked(s, LogLevel::Info, "main", "Version:  lighthouse_console.exe (buildbot_vortex-windows_steamvr_rel_win64@vortex-windows)  11000037");
        Store::LogUnlocked(s, LogLevel::Info, "main", "Attached lighthouse receiver devices: 7");
        thread_ = std::jthread([this](std::stop_token st) { Loop(st); });
    }

    void Shutdown() override {
        thread_.request_stop();
        if (thread_.joinable()) thread_.join();
    }

    void Post(Command c) override {
        std::scoped_lock lk(store.mutex);
        auto& s = store.state;
        using T = Command::Type;
        switch (c.type) {
            case T::Refresh:
                s.lastRefresh = UnixNow();
                Store::LogUnlocked(s, LogLevel::Info, "main", "刷新完成 (演示)");
                Store::ToastUnlocked(s, Toast::Kind::Info, "已刷新", "7 个接收器");
                break;
            case T::Pair:
            case T::Unpair:
            case T::FactoryReset:
            case T::Identify:
            case T::PowerOff:
                if (s.op.Active()) break;
                s.op = Operation{};
                s.op.id = ++seq_;
                s.op.kind = c.type == T::Pair ? OpKind::Pair : c.type == T::Unpair ? OpKind::Unpair : c.type == T::FactoryReset ? OpKind::FactoryReset
                            : c.type == T::Identify ? OpKind::Identify : OpKind::PowerOff;
                s.op.phase = OpPhase::Preparing;
                s.op.target = c.serial;
                s.op.deviceType = c.text;
                s.op.startedT = NowSeconds();
                s.op.timeoutS = c.seconds > 0 ? c.seconds : 30;
                s.op.message = "连接接收器 " + c.serial + " …";
                s.op.backupFile = history_.BackupDir() + "\\" + FormatTimestampForFile() + "-demo-" + c.serial + ".json";
                Store::LogUnlocked(s, LogLevel::Command, c.serial, "(演示) " + std::string(c.type == T::Pair ? "pair" : c.type == T::Unpair ? "unpair" : "命令"));
                break;
            case T::CancelOp:
                if (s.op.Active()) {
                    s.op.phase = OpPhase::Cancelled;
                    s.op.message = "已取消";
                }
                break;
            case T::SetFastPoll:
                s.fastPoll = c.value != 0;
                shakeStart_ = NowSeconds();
                break;
            case T::DismissOp:
                if (!s.op.Active()) s.op = Operation{};
                break;
            case T::SetNickname:
                if (auto* r = s.Find(c.serial)) r->nickname = c.text;
                break;
            case T::SetNote:
                if (auto* r = s.Find(c.serial)) r->note = c.text;
                break;
            case T::StartMonitor:
                if (auto* r = s.Find(c.serial)) r->monitor = MonitorState::Running;
                break;
            case T::StopMonitor:
                if (auto* r = s.Find(c.serial)) r->monitor = MonitorState::Off;
                break;
            case T::ClearStats:
                for (auto& r : s.receivers)
                    if (c.serial.empty() || r.serial == c.serial) {
                        r.events.clear();
                        r.gapCount = r.disconnectCount = r.connectCount = r.resetCount = 0;
                        r.worstGap = 0;
                    }
                break;
            case T::KillConflicts:
                s.conflicts.clear();
                Store::ToastUnlocked(s, Toast::Kind::Success, "结束冲突进程", "(演示) 已结束 1 个进程");
                break;
            case T::RawCommand:
                Store::LogUnlocked(s, LogLevel::Command, c.serial, "lh> " + c.text);
                Store::LogUnlocked(s, LogLevel::Info, c.serial, "(演示模式不执行命令)");
                break;
            default: break;
        }
    }

    std::vector<PairingRecord> PairingsFor(const std::string& serial) override {
        auto v = history_.PairingsFor(serial);
        if (serial == "109031C053")
            v.push_back({"LHR-F0F4B423", "Knuckles Left", "监控观察", UnixNow() - 86400 * 3, UnixNow() - 86400 * 2});
        if (serial == "93F15DFEA6") v.push_back({"LHR-593F5386", "VIVE Tracker (2018)", "连接事件", UnixNow() - 86400 * 40, UnixNow() - 86400 * 9});
        return v;
    }
    std::string BackupDir() override { return history_.BackupDir(); }
    std::string HistoryPath() override { return history_.Path(); }

private:
    static void Synth(Receiver& r, const DemoDevice& d, double t, std::mt19937& rng, bool live, float shake = 0) {
        std::normal_distribution<float> n(0.f, 1.f);
        std::uniform_real_distribution<float> u(0.f, 1.f);
        SignalSample s;
        s.t = t;
        float gap = 0;
        if (u(rng) < d.gapProb) {
            gap = 0.05f + u(rng) * u(rng) * 0.8f;
            if (u(rng) < 0.04f) gap = 2.0f + u(rng) * 2.5f;
        }
        float loss = std::min(1.f, gap);
        s.imuRate = std::max(0.f, d.baseRate * (1.f - loss) + n(rng) * d.baseRate * 0.004f);
        s.imuIntervalMs = d.baseRate > 0 ? 1000.f / d.baseRate * (1.f + loss * 0.6f) + std::abs(n(rng)) * 0.05f : 0;
        s.imuSigmaMs = 0.25f + std::abs(n(rng)) * 0.08f + loss * 1.4f;
        s.grav = 9.80f + n(rng) * 0.02f + shake * 0.3f * n(rng);
        s.gravSigma = 0.015f + std::abs(n(rng)) * 0.006f + shake;
        s.usbRate = s.imuRate * (d.baseRate > 500 ? 0.5f : 1.f);
        s.usbMaxIntervalMs = 2.0f + std::abs(n(rng)) * 0.5f + gap * 1000.f * 0.9f;
        s.gapSeconds = gap;
        Store::PushSample(r, s);
        r.imu.rateHz = s.imuRate;
        r.imu.intervalMs = s.imuIntervalMs;
        r.imu.sigmaMs = s.imuSigmaMs;
        r.imu.grav = s.grav;
        r.imu.gravSigma = s.gravSigma;
        r.imu.count += (uint64_t)s.imuRate;
        r.usbRates = {{"28de:" + std::string(d.pid == 0x2101 ? "2101" : "2102"), d.receiver, "IMU", s.usbRate, s.usbMaxIntervalMs}};
        if (gap > 0) {
            char b[96];
            std::snprintf(b, sizeof b, "无线丢包 %.3f 秒 (keepalive 0/1)", gap);
            SignalEvent ev{t, UnixNow() - (int64_t)(NowSeconds() - t), SignalEvent::Type::Gap, gap, b};
            Store::PushEvent(r, ev);
            if (gap > 2.0f && u(rng) < 0.5f) {
                Store::PushEvent(r, {t, ev.unix, SignalEvent::Type::TrackingReset, 0, "追踪重置: no optical samples for 2004ms"});
            }
        }
        (void)live;
    }

    void Loop(std::stop_token st) {
        std::mt19937 rng(1234);
        while (!st.stop_requested()) {
            bool fast = store.With([](AppState& st) { return st.fastPoll; });
            std::this_thread::sleep_for(std::chrono::milliseconds(fast ? 250 : 1000));
            std::scoped_lock lk(store.mutex);
            auto& s = store.state;
            double now = NowSeconds();
            std::uniform_real_distribution<float> uni(0.f, 1.f);
            // Simulated shake: the waist tracker is "picked up and shaken" 2.5–9 s after shake-identify starts.
            double shakeEl = now - shakeStart_;
            auto shakeFor = [&](const char* rx) {
                return (s.fastPoll && std::string(rx) == "D1E1CD00DA-RYB" && shakeEl > 2.5 && shakeEl < 9.0) ? 1.2f + 1.8f * uni(rng) : 0.f;
            };
            for (auto& d : kDevices) {
                Receiver* r = s.Find(d.receiver);
                if (!r || r->monitor != MonitorState::Running) continue;
                if (r->device && d.device) {
                    size_t before = r->events.size();
                    Synth(*r, d, now, rng, true, shakeFor(d.receiver));
                    if (r->events.size() > before && r->events.back().type == SignalEvent::Type::Gap)
                        Store::LogUnlocked(s, LogLevel::Event, d.receiver, std::string(d.receiver) + ": Packet received after " + std::to_string(r->events.back().value).substr(0, 5) + "s, keepalive (0/1)");
                } else {
                    SignalSample s0;
                    s0.t = now;
                    Store::PushSample(*r, s0);
                }
            }
            // Operation simulation
            auto& op = s.op;
            if (op.Active()) {
                double el = now - op.startedT;
                Receiver* r = s.Find(op.target);
                if (op.kind == OpKind::Pair) {
                    if (el < 1.5) op.phase = OpPhase::Preparing;
                    else if (el < 2.5) {
                        op.phase = OpPhase::Running;
                        op.message = "发送 pair";
                    } else if (el < 9) {
                        op.phase = OpPhase::WaitingForDevice;
                        op.message = "接收器已进入配对模式，请让设备进入配对状态";
                        if (r) r->state = LinkState::Pairing;
                    } else if (el < 10.5) {
                        op.phase = OpPhase::Verifying;
                        op.message = "设备已连接，正在读取设备信息并校验…";
                    } else {
                        op.phase = OpPhase::Succeeded;
                        op.resultDevice = "LHR-C1A522DA";
                        op.message = "配对成功: LHR-C1A522DA → " + op.target;
                        if (r) {
                            r->state = LinkState::Online;
                            r->pairedSerial = "LHR-C1A522DA";
                            r->pairedModel = "Knuckles Right";
                            r->pairingSource = PairingSource::Device;
                            DeviceInfo di;
                            di.serial = "LHR-C1A522DA";
                            di.model = "Knuckles Right";
                            di.batteryPct = 92;
                            di.connectedSince = UnixNow();
                            r->device = di;
                            Store::PushEvent(*r, {now, UnixNow(), SignalEvent::Type::PairingSuccess, 0, "Pairing Success"});
                        }
                        Store::ToastUnlocked(s, Toast::Kind::Success, "配对成功", "LHR-C1A522DA 已配对到 " + op.target, 8);
                    }
                } else {
                    if (el > 2.0) {
                        op.phase = OpPhase::Succeeded;
                        op.message = op.kind == OpKind::Identify ? "已发送震动指令" : op.kind == OpKind::PowerOff ? "已发送关机指令"
                                     : op.kind == OpKind::Unpair ? "已清除 " + op.target + " 的配对" : op.target + " 已恢复出厂设置";
                        if (r && (op.kind == OpKind::Unpair || op.kind == OpKind::FactoryReset)) {
                            r->pairedSerial.clear();
                            r->pairingSource = PairingSource::None;
                            r->device.reset();
                            r->state = LinkState::Idle;
                        }
                        Store::ToastUnlocked(s, Toast::Kind::Success, "完成", op.message);
                    } else {
                        op.phase = OpPhase::Running;
                    }
                }
            }
        }
    }

    HistoryDb history_;
    std::jthread thread_;
    uint64_t seq_ = 0;
    double shakeStart_ = 0;
};

std::unique_ptr<IService> CreateDemoService(Store& store) { return std::make_unique<DemoService>(store); }

}  // namespace lhp

#pragma once
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace lhp {

// ----------------------------------------------------------------------------- enums

/// Wired = a tracked device attached by USB (e.g. the HMD's own lighthouse module, LHR-xxxx).
/// lighthouse_console lists those next to receivers, but they are not receivers.
enum class ReceiverKind { HmdBuiltin, VrRadio, WatchmanDongle, Unknown, Wired };
enum class Hand { Unknown, Left, Right };
enum class LinkState { Unknown, Idle, Online, Pairing, Busy, Error };
enum class PairingSource { None, Device, Session, History };
enum class MonitorState { Off, Starting, Running, Paused, Failed };
enum class LogLevel { Debug, Info, Event, Warning, Error, Command };
/// What kind of tracked device is on a receiver. Decides how it can be identified:
/// controllers have a haptic motor, trackers (VIVE / Tundra) do not.
enum class DeviceClass { Unknown, IndexController, ViveWand, ViveTracker, TundraTracker };
/// Where a device's identity came from: read from the device itself, or SteamVR's on-disk cache.
enum class ConfigSource { None, Device, SteamVrCache };
const char* ToString(ConfigSource s);

const char* ToString(DeviceClass c);
inline bool CanVibrate(DeviceClass c) { return c == DeviceClass::IndexController || c == DeviceClass::ViveWand; }
inline bool IsTracker(DeviceClass c) { return c == DeviceClass::ViveTracker || c == DeviceClass::TundraTracker; }

const char* ToString(ReceiverKind k);
const char* ToString(LinkState s);
const char* ToString(PairingSource s);
const char* ToString(MonitorState s);

// ----------------------------------------------------------------------------- USB topology

struct UsbNode {
    std::string instanceId;
    std::string name;
    uint16_t vid = 0, pid = 0;
};

struct UsbInfo {
    bool present = false;
    std::string instanceId;          // USB\VID_28DE&PID_2101\93F15DFEA6
    std::string friendlyName;
    std::string busDescription;      // bus reported description
    std::string manufacturer;
    std::string locationInfo;        // Port_#0003.Hub_#0005
    std::string locationPath;        // PCIROOT(0)#PCI(0801)#...#USB(3)
    std::string driverVersion;
    std::string driverDate;
    std::string service;
    std::string usbSpeed;            // "USB 2.0 Full-Speed" etc when known
    uint32_t address = 0;            // port number on parent hub
    uint32_t problemCode = 0;
    bool hasProblem = false;
    std::vector<UsbNode> parents;    // immediate parent first
    std::vector<UsbNode> siblings;   // other devices on the same hub (helps identify HMD hubs)
    std::vector<std::string> hidInterfaces;
    bool behindHmd = false;
    std::string hmdName;             // e.g. "Pimax Dream Air (34A4:0044)"
};

// ----------------------------------------------------------------------------- live signal

struct ImuStats {
    uint64_t count = 0;
    double rateHz = 0, intervalMs = 0, sigmaMs = 0, grav = 0, gravSigma = 0;
};

struct UsbRate {
    std::string vidpid, serial, iface;
    double perSec = 0, maxIntervalMs = 0;
};

struct SignalSample {
    double t = 0;                 // NowSeconds()
    float imuRate = 0;            // Hz
    float imuIntervalMs = 0;
    float imuSigmaMs = 0;
    float grav = 0;
    float gravSigma = 0;          // accel-magnitude std-dev; jumps when the device is moved/shaken
    float usbRate = 0;            // packets/s (sum over interfaces)
    float usbMaxIntervalMs = 0;
    float gapSeconds = 0;         // largest keepalive gap observed during this sample period
};

struct SignalEvent {
    double t = 0;
    int64_t unix = 0;
    enum class Type { Gap, Connected, Disconnected, TrackingReset, Bootstrapped, PairingStart, PairingSuccess, PairingFailed, Other } type = Type::Other;
    float value = 0;              // gap seconds, etc.
    std::string text;
};

struct DeviceInfo {
    std::string serial;           // LHR-xxxxxxxx
    std::string model;            // guess / reported
    std::string firmware;
    std::string hardware;
    std::optional<int> batteryPct;
    std::optional<bool> charging;
    std::string batteryText;
    std::vector<std::pair<std::string, std::string>> props;   // deviceinfo / version key-values
    std::vector<std::string> rawDeviceInfo;
    std::vector<std::string> rawVersion;
    int64_t connectedSince = 0;
    DeviceClass deviceClass = DeviceClass::Unknown;
    Hand hand = Hand::Unknown;
    bool serialFromEvent = false;   // serial confirmed by a connect event or the device config (authoritative)

    // Identity from the device's own JSON config (downloadconfig / SteamVR cache).
    ConfigSource configSource = ConfigSource::None;
    std::string manufacturer;       // "HTC" / "Valve" / "Tundra Labs"
    std::string renderModel;        // "{htc}vr_tracker_vive_3_0"
    std::string configClass;        // device_class: controller / generic_tracker
    std::string controllerRole;     // tracked_controller_role
    std::string trackerRole;        // SteamVR tracker role assignment (TrackerRole_Waist ...)
    std::vector<std::pair<std::string, std::string>> configProps;   // top-level scalar fields
};

const char* ToString(Hand h);
/// "Valve Index 手柄（左手）" etc.
std::string DeviceDisplayModel(const DeviceInfo& d);

// ----------------------------------------------------------------------------- receiver

struct Receiver {
    std::string serial;           // 109031C053 / A04165BAAA-RYB
    uint16_t pid = 0;             // 0x2101 / 0x2102
    ReceiverKind kind = ReceiverKind::Unknown;
    bool kindOverridden = false;
    bool seenByConsole = false;
    uint32_t radioVersion = 0;
    std::vector<std::string> dongleInfoExtra;   // remaining dongleinfo fields
    UsbInfo usb;

    LinkState state = LinkState::Unknown;
    std::optional<DeviceInfo> device;           // currently connected device (live)
    std::string pairedSerial;                   // best known paired device
    std::string pairedModel;
    DeviceClass pairedClass = DeviceClass::Unknown;
    PairingSource pairingSource = PairingSource::None;
    int64_t pairedLastSeen = 0;

    std::string nickname;
    std::string note;
    int64_t firstSeen = 0, lastSeen = 0;

    // live monitor
    MonitorState monitor = MonitorState::Off;
    std::string monitorError;
    ImuStats imu;
    std::vector<UsbRate> usbRates;
    std::deque<SignalSample> samples;           // ring buffer (kMaxSamples)
    std::deque<SignalEvent> events;             // ring buffer (kMaxEvents)
    uint32_t gapCount = 0, disconnectCount = 0, connectCount = 0, resetCount = 0;
    float worstGap = 0;
    double lastSampleT = 0;
    float pendingGap = 0;                       // accumulates until next sample
    std::string lastError;

    static constexpr size_t kMaxSamples = 900;  // 15 min at 1 Hz
    static constexpr size_t kMaxEvents = 300;

    const char* DisplayName() const { return nickname.empty() ? serial.c_str() : nickname.c_str(); }
};

// ----------------------------------------------------------------------------- misc

struct ConflictProcess {
    uint32_t pid = 0;
    std::string exe;
    std::string category;      // "SteamVR" / "Pimax" / "lighthouse_console"
    bool isService = false;
};

struct LogLine {
    int64_t unix = 0;
    LogLevel level = LogLevel::Info;
    std::string source;        // receiver serial or "main"
    std::string text;
};

enum class OpKind { None, Refresh, Pair, Unpair, FactoryReset, Identify, PowerOff, KillConflicts };
enum class OpPhase { Idle, Preparing, Running, WaitingForDevice, Verifying, Succeeded, Failed, Cancelled };

struct Operation {
    OpKind kind = OpKind::None;
    OpPhase phase = OpPhase::Idle;
    std::string target;        // receiver serial
    std::string deviceType;    // pairing hint
    std::string message;
    std::string resultDevice;  // newly paired device serial
    std::string backupFile;
    double startedT = 0;
    double timeoutS = 0;
    uint64_t id = 0;
    bool Active() const { return phase == OpPhase::Preparing || phase == OpPhase::Running || phase == OpPhase::WaitingForDevice || phase == OpPhase::Verifying; }
};

struct Toast {
    enum class Kind { Info, Success, Warning, Error } kind = Kind::Info;
    std::string title, text;
    double createdT = 0;
    float duration = 5.0f;
};

struct Settings {
    std::string consolePathOverride;
    bool autoMonitorAll = true;
    int pollIntervalMs = 1000;
    int batteryIntervalS = 15;
    bool expertMode = false;
    bool useMica = true;
    int theme = 0;              // 0 = dark, 1 = light, 2 = system
    float uiScale = 1.0f;
    bool trackingDiagnostics = false;   // enable 'pose' in monitor sessions
};

struct AppState {
    std::vector<Receiver> receivers;
    std::vector<ConflictProcess> conflicts;
    std::string consolePath;
    std::string consoleVersion;
    std::string steamVrRuntime;
    bool backendReady = false;
    bool refreshing = false;
    bool demo = false;
    bool fastPoll = false;          // shake-identify in progress: monitors poll ~4x/s
    std::string backendError;
    int64_t lastRefresh = 0;
    std::deque<LogLine> log;
    Operation op;
    std::vector<Toast> toasts;
    Settings settings;
    std::string dataDir;

    static constexpr size_t kMaxLog = 50000;

    Receiver* Find(std::string_view serial);
    const Receiver* Find(std::string_view serial) const;
};

}  // namespace lhp

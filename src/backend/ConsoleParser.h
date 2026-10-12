#pragma once
#include "core/Model.h"

#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace lhp::parser {

struct DongleEntry {
    std::string serial;
    uint32_t radioVersion = 0;
    std::vector<std::string> extra;
};

struct UsbStatEntry {
    std::string vid, pid, serial, iface;
    double perSec = 0, maxIntervalMs = 0;
};

struct BatteryInfo {
    std::optional<int> percent;
    std::optional<bool> charging;
    bool notProvided = false;
    std::string text;
};

// -- line classification --------------------------------------------------------

struct EvSessionConnected { std::string receiver; };                       // lighthouse_console: Connected to receiver X
struct EvDeviceConnected { std::string device, receiver; };                // LHR-x: Connected to receiver X
struct EvDeviceDisconnected { std::string device, receiver; };             // X: Wireless controller LHR-x disconnected / LHR-x: Disconnected from receiver X
struct EvKeepaliveGap { std::string receiver; double seconds = 0; int a = 0, b = 0; };
struct EvTrackingReset { std::string device; std::string reason; };
struct EvBootstrapped { std::string device; std::string base; std::string text; };
struct EvPairing { enum class Kind { Start, Success, Failed } kind; std::string text; };
struct EvCommandError { std::string text; };
struct EvDemodRate { std::string device; int rate = 0; };

using Event = std::variant<std::monostate, EvSessionConnected, EvDeviceConnected, EvDeviceDisconnected, EvKeepaliveGap,
                           EvTrackingReset, EvBootstrapped, EvPairing, EvCommandError, EvDemodRate>;

/// Strip a leading "lh> " prompt (possibly repeated) and trailing CR/LF.
std::string_view StripPrompt(std::string_view line, int* promptCount = nullptr);

/// "Attached lighthouse receiver devices: N" followed by tab indented serials.
std::vector<std::string> ParseReceiverList(const std::vector<std::string>& lines);

/// "Dongles:SER,VER,EXTRA;SER,VER,EXTRA"
std::vector<DongleEntry> ParseDongleInfo(std::string_view line);

/// "imu 62 rate 245.1Hz interval 4.1ms sigma 0.506ms grav 9.97m/s/s sigma 0.016"
std::optional<ImuStats> ParseImuStats(std::string_view line);

/// "  28de 2102 09BCE41887-LYM IMU   0/sec, max interval 0ms"
std::optional<UsbStatEntry> ParseUsbStatLine(std::string_view line);

BatteryInfo ParseBattery(const std::vector<std::string>& lines);

/// "Key: Value" / "Key = Value" lines.
std::vector<std::pair<std::string, std::string>> ParseKeyValues(const std::vector<std::string>& lines);

/// Version banner "Version:  lighthouse_console.exe (...)  11000037"
std::optional<std::string> ParseConsoleVersion(std::string_view line);

Event Classify(std::string_view line);

/// Guess a friendly model from a property list / serial.
std::string GuessModel(const std::vector<std::pair<std::string, std::string>>& props, std::string_view serial);

/// Work out the device family from every string we know about it
/// (model, deviceinfo / version output, hardware revision, registered device type).
///   Index controller : "knuckles" / "index_controller" / hw rev 0x110e…, 0x110f…
///   Vive wand        : "vive_controller" / "wand" / "controller" (HTC)
///   VIVE Tracker     : "vive_tracker" / "tracker" / hw rev 0x84xx… (Tracker 3.0 = 0x8402…)
///   Tundra Tracker   : "tundra"
DeviceClass ClassifyDevice(const DeviceInfo& d, std::string_view extraHint = {});
DeviceClass ClassifyText(std::string_view text);

/// Left / right for Index controllers.
///  1) explicit "left"/"right" (model, render model e.g. valve_controller_knu_1_0_left)
///  2) Knuckles hardware revision: 0x110e0109 = left, 0x110e0009 = right (bit 0x100),
///     as seen for LHR-F0F4B423 (Knuckles Left) / LHR-9E63AEE0 (Knuckles Right).
Hand DetectHand(const DeviceInfo& d);

/// All LHR-xxxxxxxx serials in the text, in order of appearance (deduplicated).
std::vector<std::string> FindLhrSerials(const std::vector<std::string>& lines);

/// True for console list entries that are wired tracked devices rather than receivers.
inline bool IsWiredDeviceSerial(std::string_view s) { return s.size() > 4 && s.substr(0, 4) == "LHR-"; }

/// Responses to our periodic polling (imustats / usbstats / battery / clear …). They are parsed
/// into live stats and must not be written to the log, otherwise they flood it within seconds.
bool IsPollNoise(std::string_view line);

/// True if the line indicates the command failed.
bool LooksLikeError(std::string_view line);

}  // namespace lhp::parser

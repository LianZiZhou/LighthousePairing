#include "backend/ConsoleParser.h"

#include "util/Text.h"

#include <regex>

namespace lhp::parser {

std::string_view StripPrompt(std::string_view line, int* promptCount) {
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.remove_suffix(1);
    int n = 0;
    while (StartsWith(line, "lh> ")) {
        line.remove_prefix(4);
        ++n;
    }
    if (line == "lh>") {
        line = {};
        ++n;
    }
    if (promptCount) *promptCount = n;
    return line;
}

std::vector<std::string> ParseReceiverList(const std::vector<std::string>& lines) {
    std::vector<std::string> out;
    bool in = false;
    for (auto& raw : lines) {
        std::string_view l = StripPrompt(raw);
        if (StartsWith(l, "Attached lighthouse receiver devices:")) {
            in = true;
            continue;
        }
        if (in) {
            if (!l.empty() && (l[0] == '\t' || l[0] == ' ')) {
                auto s = Trim(l);
                if (!s.empty()) out.emplace_back(s);
            } else {
                in = false;
            }
        }
    }
    return out;
}

std::vector<DongleEntry> ParseDongleInfo(std::string_view line) {
    std::vector<DongleEntry> out;
    line = StripPrompt(line);
    auto pos = line.find("Dongles:");
    if (pos == std::string_view::npos) return out;
    line.remove_prefix(pos + 8);
    for (auto item : Split(line, ';')) {
        item = Trim(item);
        if (item.empty()) continue;
        auto f = Split(item, ',');
        DongleEntry e;
        e.serial = std::string(Trim(f[0]));
        if (e.serial.empty()) continue;
        if (f.size() > 1) e.radioVersion = ParseNum<uint32_t>(f[1]).value_or(0);
        for (size_t i = 2; i < f.size(); ++i) e.extra.emplace_back(Trim(f[i]));
        while (!e.extra.empty() && e.extra.back().empty()) e.extra.pop_back();
        out.push_back(std::move(e));
    }
    return out;
}

std::optional<ImuStats> ParseImuStats(std::string_view line) {
    static const std::regex re(
        R"(imu\s+(\d+)\s+rate\s+(-?[\d.]+)Hz\s+interval\s+(-?[\d.]+)ms\s+sigma\s+(-?[\d.]+)ms\s+grav\s+(-?[\d.]+)m/s/s\s+sigma\s+(-?[\d.]+))");
    std::string s(StripPrompt(line));
    std::smatch m;
    if (!std::regex_search(s, m, re)) return std::nullopt;
    ImuStats st;
    st.count = std::stoull(m[1]);
    st.rateHz = std::stod(m[2]);
    st.intervalMs = std::stod(m[3]);
    st.sigmaMs = std::stod(m[4]);
    st.grav = std::stod(m[5]);
    st.gravSigma = std::stod(m[6]);
    if (st.rateHz < 0) st.rateHz = 0;
    return st;
}

std::optional<UsbStatEntry> ParseUsbStatLine(std::string_view line) {
    static const std::regex re(R"(^\s*([0-9a-fA-F]{4})\s+([0-9a-fA-F]{4})\s+(\S+)\s+(.+?)\s+(-?[\d.]+)/sec,\s*max interval\s+(-?[\d.]+)ms)");
    std::string s(StripPrompt(line));
    std::smatch m;
    if (!std::regex_search(s, m, re)) return std::nullopt;
    UsbStatEntry e;
    e.vid = m[1];
    e.pid = m[2];
    e.serial = m[3];
    e.iface = std::string(Trim(m[4].str()));
    e.perSec = std::stod(m[5]);
    e.maxIntervalMs = std::stod(m[6]);
    return e;
}

BatteryInfo ParseBattery(const std::vector<std::string>& lines) {
    BatteryInfo b;
    static const std::regex pct(R"((\d{1,3})\s*%)");
    static const std::regex lvl(R"((?:battery|charge)[^0-9]*?(\d{1,3}))", std::regex::icase);
    for (auto& raw : lines) {
        std::string s(StripPrompt(raw));
        if (s.empty()) continue;
        if (!b.text.empty()) b.text += " | ";
        b.text += s;
        if (Contains(s, "not provided")) b.notProvided = true;
        std::smatch m;
        if (!b.percent && (std::regex_search(s, m, pct) || std::regex_search(s, m, lvl))) {
            int v = std::stoi(m[1]);
            if (v >= 0 && v <= 100) b.percent = v;
        }
        std::string low = s;
        for (auto& c : low) c = (char)std::tolower((unsigned char)c);
        if (Contains(low, "not charging") || Contains(low, "discharging")) b.charging = false;
        else if (Contains(low, "charging")) b.charging = true;
    }
    return b;
}

std::vector<std::pair<std::string, std::string>> ParseKeyValues(const std::vector<std::string>& lines) {
    std::vector<std::pair<std::string, std::string>> out;
    for (auto& raw : lines) {
        std::string_view l = Trim(StripPrompt(raw));
        if (l.empty()) continue;
        size_t p = l.find(':');
        size_t q = l.find(" = ");
        if (q != std::string_view::npos && (p == std::string_view::npos || q < p)) p = q;
        if (p == std::string_view::npos || p == 0 || p > 48) {
            out.emplace_back("", std::string(l));
            continue;
        }
        auto k = Trim(l.substr(0, p));
        auto v = Trim(l.substr(p + (l[p] == ':' ? 1 : 3)));
        out.emplace_back(std::string(k), std::string(v));
    }
    return out;
}

std::optional<std::string> ParseConsoleVersion(std::string_view line) {
    line = StripPrompt(line);
    if (!StartsWith(line, "Version:")) return std::nullopt;
    auto t = Trim(line.substr(8));
    auto sp = t.find_last_of(' ');
    if (sp != std::string_view::npos) return std::string(Trim(t.substr(sp)));
    return std::string(t);
}

bool IsPollNoise(std::string_view raw) {
    std::string_view l = Trim(StripPrompt(raw));
    if (l.empty()) return true;
    if (StartsWith(l, "imu ") && Contains(l, " rate ")) return true;
    if (StartsWith(l, "USB status (rates")) return true;
    if (ParseUsbStatLine(l)) return true;
    if (Contains(l, "battery status not provided")) return true;
    if (StartsWith(l, "Error reading Lighthouse Error Report")) return true;
    if (StartsWith(l, "Error reading usb json configuration")) return true;
    return false;
}

bool LooksLikeError(std::string_view line) {
    // Routine noise printed for every receiver without a connected device.
    if (Contains(line, "Unable to request config start")) return false;
    if (Contains(line, "Error reading usb json configuration")) return false;   // printed for wired devices on every deviceinfo/version
    if (Contains(line, "Error reading Lighthouse Error Report")) return false;
    return Contains(line, "Could not") || Contains(line, "Failed") || Contains(line, "Unable to") || Contains(line, "Can't") ||
           Contains(line, "Error") || Contains(line, "Connect to a dongle first");
}

Event Classify(std::string_view raw) {
    std::string_view l = StripPrompt(raw);
    if (l.empty()) return {};
    std::string s(l);
    std::smatch m;

    static const std::regex reSession(R"(^lighthouse_console: Connected to receiver (\S+))");
    static const std::regex reDevConn(R"(^(LHR-[0-9A-Fa-f]+): Connected to receiver (\S+))");
    static const std::regex reWirelessDisc(R"(^(\S+): Wireless (?:controller|device) (\S+) disconnected)");
    static const std::regex reWirelessConn(R"(^(\S+): Wireless (?:controller|device) (\S+) connected)");
    static const std::regex reDevDisc(R"(^(LHR-[0-9A-Fa-f]+): Disconnected from receiver (\S+))");
    static const std::regex reGap(R"(^(\S+): Packet received after ([\d.]+)s, keepalive \((\d+)/(\d+)\))");
    static const std::regex reReset(R"(^(LHR-[0-9A-Fa-f]+) \w: Resetting tracking: (.*)$)");
    static const std::regex reBoot(R"(^(LHR-[0-9A-Fa-f]+) \w: ----- (BOOTSTRAPPED|CALIBRATED) base (\S+))");
    static const std::regex reDemod(R"(^(LHR-[0-9A-Fa-f]+) \w: Saw demod rate (\d+)/s)");

    if (std::regex_search(s, m, reSession)) return EvSessionConnected{m[1]};
    if (std::regex_search(s, m, reDevConn)) return EvDeviceConnected{m[1], m[2]};
    if (std::regex_search(s, m, reWirelessDisc)) return EvDeviceDisconnected{m[2], m[1]};
    if (std::regex_search(s, m, reWirelessConn)) return EvDeviceConnected{m[2], m[1]};
    if (std::regex_search(s, m, reDevDisc)) return EvDeviceDisconnected{m[1], m[2]};
    if (std::regex_search(s, m, reGap)) return EvKeepaliveGap{m[1], std::stod(m[2]), std::stoi(m[3]), std::stoi(m[4])};
    if (std::regex_search(s, m, reReset)) return EvTrackingReset{m[1], m[2]};
    if (std::regex_search(s, m, reBoot)) return EvBootstrapped{m[1], m[3], s};
    if (std::regex_search(s, m, reDemod)) return EvDemodRate{m[1], std::stoi(m[2])};
    if (Contains(s, "Pairing Success")) return EvPairing{EvPairing::Kind::Success, s};
    if (Contains(s, "Pairing Failed") || Contains(s, "Could not pair") || Contains(s, "Failed to start dongle pairing") ||
        Contains(s, "Unable to request wireless pairing"))
        return EvPairing{EvPairing::Kind::Failed, s};
    if (Contains(s, "Pairing Start")) return EvPairing{EvPairing::Kind::Start, s};
    if (Contains(s, "Could not unpair") || Contains(s, "Failed to erase dongle pairing") || Contains(s, "Could not reset dongle") ||
        Contains(s, "Failed to send factory reset") || Contains(s, "Connect to a dongle first") || Contains(s, "Unknown command"))
        return EvCommandError{s};
    return {};
}

DeviceClass ClassifyText(std::string_view text) {
    std::string t(text);
    for (auto& c : t) c = (char)std::tolower((unsigned char)c);
    auto has = [&](const char* k) { return t.find(k) != std::string::npos; };
    if (has("tundra")) return DeviceClass::TundraTracker;
    if (has("knuckles") || has("index_controller") || has("index controller") || has("indexcontroller")) return DeviceClass::IndexController;
    if (has("vive_tracker") || has("vive tracker") || has("tracker")) return DeviceClass::ViveTracker;
    if (has("vive_controller") || has("vive controller") || has("wand")) return DeviceClass::ViveWand;
    // Hardware revisions as reported by the lighthouse driver (e.g. Pimax/SteamVR logs):
    // Knuckles 0x110e0109 / alias 0x110f0009, VIVE Tracker 3.0 0x84020109.
    if (has("0x110e") || has("0x110f")) return DeviceClass::IndexController;
    if (has("0x840")) return DeviceClass::ViveTracker;
    return DeviceClass::Unknown;
}

std::vector<std::string> FindLhrSerials(const std::vector<std::string>& lines) {
    static const std::regex re(R"((LHR-[0-9A-Fa-f]{8}))", std::regex::icase);
    std::vector<std::string> out;
    for (auto& l : lines)
        for (std::sregex_iterator it(l.begin(), l.end(), re), end; it != end; ++it) {
            std::string s = (*it)[1];
            for (auto& c : s) c = (char)std::toupper((unsigned char)c);
            s[0] = 'L', s[1] = 'H', s[2] = 'R';
            if (std::find(out.begin(), out.end(), s) == out.end()) out.push_back(s);
        }
    return out;
}

Hand DetectHand(const DeviceInfo& d) {
    std::string all = d.model + " " + d.hardware + " " + d.firmware + " ";
    for (auto& [k, v] : d.props) all += k + " " + v + " ";
    for (auto& l : d.rawDeviceInfo) all += l + " ";
    for (auto& l : d.rawVersion) all += l + " ";
    std::string low = all;
    for (auto& c : low) c = (char)std::tolower((unsigned char)c);
    bool left = Contains(low, "left") || Contains(all, "左");
    bool right = Contains(low, "right") || Contains(all, "右");
    if (left != right) return left ? Hand::Left : Hand::Right;
    static const std::regex re(R"(0x110[ef]([0-9a-fA-F]{4}))");
    std::smatch m;
    if (std::regex_search(low, m, re)) {
        unsigned v = (unsigned)std::stoul(m[1], nullptr, 16);
        return (v & 0x100) ? Hand::Left : Hand::Right;
    }
    return Hand::Unknown;
}

DeviceClass ClassifyDevice(const DeviceInfo& d, std::string_view extraHint) {
    // Most specific sources first.
    for (auto s : {std::string_view(d.model), extraHint})
        if (auto c = ClassifyText(s); c != DeviceClass::Unknown) return c;
    std::string all = d.hardware + " " + d.firmware + " ";
    for (auto& [k, v] : d.props) all += k + " " + v + " ";
    for (auto& l : d.rawDeviceInfo) all += l + " ";
    for (auto& l : d.rawVersion) all += l + " ";
    if (auto c = ClassifyText(all); c != DeviceClass::Unknown) return c;
    if (Contains(all, "HTC") || Contains(all, "htc")) return DeviceClass::ViveTracker;   // HTC without "controller" is almost always a tracker
    return DeviceClass::Unknown;
}

std::string GuessModel(const std::vector<std::pair<std::string, std::string>>& props, std::string_view serial) {
    for (auto& [k, v] : props) {
        std::string lk = k;
        for (auto& c : lk) c = (char)std::tolower((unsigned char)c);
        if (lk == "model" || lk == "model number" || lk == "modelnumber" || lk == "product") return v;
    }
    for (auto& [k, v] : props) {
        if (Contains(v, "Knuckles") || Contains(v, "Index")) return "Valve Index 手柄";
        if (Contains(v, "Tracker")) return "VIVE Tracker";
        if (Contains(v, "Tundra")) return "Tundra Tracker";
    }
    (void)serial;
    return {};
}

}  // namespace lhp::parser

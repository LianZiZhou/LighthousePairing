#include "core/HistoryDb.h"

#include "backend/ConsoleParser.h"
#include "backend/SystemInfo.h"
#include "util/Text.h"

#include <algorithm>

namespace lhp {

using nlohmann::json;

HistoryDb::HistoryDb(std::string dataDir) : dir_(std::move(dataDir)), path_(dir_ + "\\history.json") {
    j_ = json::object();
}

void HistoryDb::Load() {
    std::scoped_lock lk(mtx_);
    std::string text = sys::ReadTextFile(path_);
    try {
        j_ = text.empty() ? json::object() : json::parse(text);
    } catch (...) {
        sys::WriteTextFile(path_ + ".corrupt", text);
        j_ = json::object();
    }
    if (!j_.is_object()) j_ = json::object();
    if (!j_.contains("receivers")) j_["receivers"] = json::object();
    if (!j_.contains("devices")) j_["devices"] = json::object();
    j_["version"] = 1;
}

bool HistoryDb::Save() {
    std::scoped_lock lk(mtx_);
    if (!dirty_) return true;
    dirty_ = false;
    return sys::WriteTextFile(path_, j_.dump(2));
}

json& HistoryDb::Rec(const std::string& serial) {
    auto& r = j_["receivers"][serial];
    if (!r.is_object()) r = json::object();
    return r;
}

void HistoryDb::ApplyTo(Receiver& r) {
    std::scoped_lock lk(mtx_);
    if (!j_["receivers"].contains(r.serial)) return;
    auto& rec = j_["receivers"][r.serial];
    r.nickname = rec.value("nickname", "");
    r.note = rec.value("note", "");
    r.firstSeen = rec.value("firstSeen", (int64_t)0);
    int kind = rec.value("kind", -1);
    if (kind >= 0 && kind <= (int)ReceiverKind::Unknown) {
        r.kind = (ReceiverKind)kind;
        r.kindOverridden = true;
    }
    if (r.pairingSource == PairingSource::None && rec.contains("pairings") && !rec["pairings"].empty()) {
        // Most recent record wins.
        const json* best = nullptr;
        for (auto& p : rec["pairings"])
            if (!best || p.value("lastSeen", (int64_t)0) > best->value("lastSeen", (int64_t)0)) best = &p;
        if (best && !best->value("forgotten", false)) {
            r.pairedSerial = best->value("device", "");
            r.pairedModel = best->value("model", "");
            r.pairedClass = parser::ClassifyText(r.pairedModel);
            r.pairedLastSeen = best->value("lastSeen", (int64_t)0);
            if (!r.pairedSerial.empty()) r.pairingSource = PairingSource::History;
        }
    }
}

void HistoryDb::Observe(const Receiver& r) {
    std::scoped_lock lk(mtx_);
    auto& rec = Rec(r.serial);
    int64_t now = UnixNow();
    if (!rec.contains("firstSeen")) rec["firstSeen"] = now;
    rec["lastSeen"] = now;
    rec["pid"] = r.pid;
    rec["radioVersion"] = r.radioVersion;
    if (!r.usb.locationPath.empty()) rec["lastLocation"] = r.usb.locationPath;
    dirty_ = true;
}

void HistoryDb::RecordPairing(const std::string& receiver, const std::string& device, const std::string& model, const std::string& source) {
    if (device.empty()) return;
    std::scoped_lock lk(mtx_);
    auto& rec = Rec(receiver);
    auto& list = rec["pairings"];
    if (!list.is_array()) list = json::array();
    int64_t now = UnixNow();
    bool found = false;
    for (auto& p : list) {
        if (p.value("device", "") == device) {
            p["lastSeen"] = now;
            p["forgotten"] = false;
            if (!model.empty()) p["model"] = model;
            p["source"] = source;
            found = true;
        }
    }
    if (!found) list.push_back({{"device", device}, {"model", model}, {"source", source}, {"firstSeen", now}, {"lastSeen", now}});
    auto& d = j_["devices"][device];
    if (!d.is_object()) d = json::object();
    d["lastReceiver"] = receiver;
    d["lastSeen"] = now;
    if (!model.empty()) d["model"] = model;
    dirty_ = true;
}

void HistoryDb::ForgetPairing(const std::string& receiver) {
    std::scoped_lock lk(mtx_);
    auto& rec = Rec(receiver);
    if (rec.contains("pairings"))
        for (auto& p : rec["pairings"]) p["forgotten"] = true;
    dirty_ = true;
}

void HistoryDb::ForgetDevice(const std::string& device) {
    std::scoped_lock lk(mtx_);
    for (auto& [ser, rec] : j_["receivers"].items()) {
        (void)ser;
        if (!rec.contains("pairings") || !rec["pairings"].is_array()) continue;
        auto& list = rec["pairings"];
        json kept = json::array();
        for (auto& p : list)
            if (p.value("device", "") != device) kept.push_back(p);
        if (kept.size() != list.size()) {
            list = kept;
            dirty_ = true;
        }
    }
}

void HistoryDb::SetNickname(const std::string& serial, const std::string& name) {
    std::scoped_lock lk(mtx_);
    Rec(serial)["nickname"] = name;
    dirty_ = true;
}

void HistoryDb::SetNote(const std::string& serial, const std::string& note) {
    std::scoped_lock lk(mtx_);
    Rec(serial)["note"] = note;
    dirty_ = true;
}

void HistoryDb::SetKindOverride(const std::string& serial, int kind) {
    std::scoped_lock lk(mtx_);
    Rec(serial)["kind"] = kind;
    dirty_ = true;
}

std::string HistoryDb::DeviceNickname(const std::string& device) const {
    std::scoped_lock lk(mtx_);
    if (!j_.contains("devices") || !j_["devices"].contains(device)) return {};
    return j_["devices"][device].value("nickname", "");
}

void HistoryDb::SetDeviceNickname(const std::string& device, const std::string& name) {
    std::scoped_lock lk(mtx_);
    auto& d = j_["devices"][device];
    if (!d.is_object()) d = json::object();
    d["nickname"] = name;
    dirty_ = true;
}

std::vector<PairingRecord> HistoryDb::PairingsFor(const std::string& receiver) const {
    std::scoped_lock lk(mtx_);
    std::vector<PairingRecord> out;
    if (!j_.contains("receivers") || !j_["receivers"].contains(receiver)) return out;
    auto& rec = j_["receivers"][receiver];
    if (!rec.contains("pairings")) return out;
    for (auto& p : rec["pairings"]) {
        PairingRecord pr;
        pr.device = p.value("device", "");
        pr.model = p.value("model", "");
        pr.source = p.value("source", "");
        if (p.value("forgotten", false)) pr.source += " (已解除)";
        pr.firstSeen = p.value("firstSeen", (int64_t)0);
        pr.lastSeen = p.value("lastSeen", (int64_t)0);
        out.push_back(pr);
    }
    std::sort(out.begin(), out.end(), [](auto& a, auto& b) { return a.lastSeen > b.lastSeen; });
    return out;
}

Settings HistoryDb::LoadSettings() const {
    std::scoped_lock lk(mtx_);
    Settings s;
    if (!j_.contains("settings")) return s;
    auto& j = j_["settings"];
    s.consolePathOverride = j.value("consolePathOverride", s.consolePathOverride);
    s.autoMonitorAll = j.value("autoMonitorAll", s.autoMonitorAll);
    s.pollIntervalMs = j.value("pollIntervalMs", s.pollIntervalMs);
    s.batteryIntervalS = j.value("batteryIntervalS", s.batteryIntervalS);
    s.expertMode = j.value("expertMode", s.expertMode);
    s.useMica = j.value("useMica", s.useMica);
    s.theme = j.value("theme", s.theme);
    s.uiScale = j.value("uiScale", s.uiScale);
    s.trackingDiagnostics = j.value("trackingDiagnostics", s.trackingDiagnostics);
    return s;
}

void HistoryDb::SaveSettings(const Settings& s) {
    {
        std::scoped_lock lk(mtx_);
        j_["settings"] = {{"consolePathOverride", s.consolePathOverride}, {"autoMonitorAll", s.autoMonitorAll},
                          {"pollIntervalMs", s.pollIntervalMs},           {"batteryIntervalS", s.batteryIntervalS},
                          {"expertMode", s.expertMode},                   {"useMica", s.useMica},
                          {"theme", s.theme},                             {"uiScale", s.uiScale},
                          {"trackingDiagnostics", s.trackingDiagnostics}};
        dirty_ = true;
    }
    Save();
}

json SnapshotJson(const AppState& s) {
    json j;
    j["time"] = FormatDateTime(UnixNow());
    j["consolePath"] = s.consolePath;
    j["consoleVersion"] = s.consoleVersion;
    j["receivers"] = json::array();
    for (auto& r : s.receivers) {
        json o = {{"serial", r.serial},
                  {"pid", r.pid},
                  {"kind", ToString(r.kind)},
                  {"radioVersion", r.radioVersion},
                  {"dongleInfoExtra", r.dongleInfoExtra},
                  {"state", ToString(r.state)},
                  {"pairedSerial", r.pairedSerial},
                  {"pairedModel", r.pairedModel},
                  {"pairingSource", ToString(r.pairingSource)},
                  {"nickname", r.nickname},
                  {"usbInstance", r.usb.instanceId},
                  {"usbLocation", r.usb.locationPath},
                  {"behindHmd", r.usb.behindHmd},
                  {"hmd", r.usb.hmdName}};
        if (r.device) {
            o["device"] = {{"serial", r.device->serial}, {"model", r.device->model}, {"firmware", r.device->firmware}};
            json props = json::array();
            for (auto& [k, v] : r.device->props) props.push_back({k, v});
            o["device"]["props"] = props;
        }
        j["receivers"].push_back(o);
    }
    return j;
}

std::string HistoryDb::WriteBackup(const AppState& s, const std::string& op, const std::string& target) {
    json j = SnapshotJson(s);
    j["operation"] = op;
    j["target"] = target;
    {
        std::scoped_lock lk(mtx_);
        j["history"] = j_;
    }
    std::string file = BackupDir() + "\\" + FormatTimestampForFile() + "-" + op + "-" + target + ".json";
    sys::WriteTextFile(file, j.dump(2));
    return file;
}

}  // namespace lhp

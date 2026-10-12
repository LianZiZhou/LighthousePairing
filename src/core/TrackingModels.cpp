#include "core/TrackingModels.h"

#include "backend/SystemInfo.h"
#include "tracking/solver/UniverseIo.h"
#include "util/Text.h"

#include <nlohmann/json.hpp>

namespace lhp {

std::optional<std::string> StoreModelProvider::ConfigFor(const std::string& receiverSerial) {
    std::string device = store_.With([&](AppState& s) -> std::string {
        const Receiver* r = s.Find(receiverSerial);
        if (!r) return {};
        if (r->device && !r->device->serial.empty()) return r->device->serial;
        return r->pairedSerial;
    });
    if (device.empty()) return std::nullopt;
    std::string text = sys::ReadTextFile(sys::DeviceConfigPath(device));
    if (text.empty()) text = sys::ReadCachedDeviceConfig(device);
    if (text.empty()) return std::nullopt;
    return text;
}

namespace {

std::string Hex(const std::vector<uint8_t>& b) {
    static const char* d = "0123456789ABCDEF";
    std::string s;
    for (uint8_t x : b) {
        s += d[x >> 4];
        s += d[x & 15];
    }
    return s;
}

std::vector<uint8_t> Unhex(const std::string& s) {
    std::vector<uint8_t> out;
    auto v = [](char c) { return c >= '0' && c <= '9' ? c - '0' : c >= 'A' && c <= 'F' ? c - 'A' + 10 : c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1; };
    for (size_t i = 0; i + 1 < s.size(); i += 2) {
        int hi = v(s[i]), lo = v(s[i + 1]);
        if (hi < 0 || lo < 0) return {};
        out.push_back((uint8_t)(hi * 16 + lo));
    }
    return out;
}

}  // namespace

FileCalibrationCache::FileCalibrationCache(std::string path) : path_(std::move(path)) {
    auto j = nlohmann::json::parse(sys::ReadTextFile(path_), nullptr, false);
    if (j.is_discarded() || !j.is_object()) return;
    if (j.contains("channels") && j["channels"].is_object())
        for (auto& [k, v] : j["channels"].items())
            if (v.is_object() && v.contains("ootx")) raw_[std::atoi(k.c_str())] = Unhex(v["ootx"].get<std::string>());
    if (j.contains("universe") && j["universe"].is_object()) universe_ = j["universe"].dump();
}

std::vector<tracking::solver::UniverseRecord> FileCalibrationCache::LoadUniverses() {
    // SteamVR's own room setup first (closest to what SteamVR shows), then what we learned last time.
    std::vector<tracking::solver::UniverseRecord> out;
    std::string steam = sys::FindSteamConfigDir();
    if (!steam.empty()) {
        std::string text = sys::ReadTextFile(steam + "\\lighthouse\\lighthousedb.json");
        if (!text.empty()) out = tracking::solver::ParseSteamVrLighthouseDb(text);
    }
    std::scoped_lock lk(mtx_);
    if (!universe_.empty())
        if (auto u = tracking::solver::UniverseFromJson(universe_)) out.push_back(*u);
    return out;
}

void FileCalibrationCache::SaveUniverse(const tracking::solver::UniverseRecord& u) {
    std::string text = tracking::solver::UniverseToJson(u);
    std::scoped_lock lk(mtx_);
    if (text == universe_) return;
    universe_ = text;
    WriteLocked();
}

std::optional<tracking::lighthouse::BaseCalibration> FileCalibrationCache::Load(int channel) {
    std::scoped_lock lk(mtx_);
    auto it = raw_.find(channel);
    if (it == raw_.end()) return std::nullopt;
    return tracking::lighthouse::ParseOotxV15(it->second.data(), it->second.size());
}

void FileCalibrationCache::Save(int channel, const tracking::lighthouse::BaseCalibration& cal) {
    std::scoped_lock lk(mtx_);
    if (cal.raw.empty() || raw_[channel] == cal.raw) return;   // unchanged: no disk write
    raw_[channel] = cal.raw;
    WriteLocked();
}

void FileCalibrationCache::WriteLocked() {
    nlohmann::json j;
    for (auto& [ch, raw] : raw_) {
        auto c = tracking::lighthouse::ParseOotxV15(raw.data(), raw.size());
        char id[16];
        std::snprintf(id, sizeof id, "%08X", c ? c->id : 0u);
        j["channels"][std::to_string(ch)] = {{"id", id}, {"ootx", Hex(raw)}};
    }
    if (!universe_.empty()) {
        auto u = nlohmann::json::parse(universe_, nullptr, false);
        if (!u.is_discarded()) j["universe"] = u;
    }
    sys::WriteTextFile(path_, j.dump(2));
}

}  // namespace lhp

#include "tracking/solver/UniverseIo.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <map>

namespace lhp::tracking::solver {

using math::Pose;
using math::Quat;
using math::Vec3;
using nlohmann::json;

namespace {

std::optional<Pose> PoseFrom7(const json& a) {   // [qx qy qz qw px py pz]
    if (!a.is_array() || a.size() != 7) return std::nullopt;
    for (auto& v : a)
        if (!v.is_number()) return std::nullopt;
    Quat q{a[3].get<double>(), a[0].get<double>(), a[1].get<double>(), a[2].get<double>()};
    if (std::fabs(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z - 1) > 0.05) return std::nullopt;
    return Pose{q.Normalized(), {a[4].get<double>(), a[5].get<double>(), a[6].get<double>()}};
}

bool IsIdentity(const Pose& p) {
    return std::fabs(std::fabs(p.rot.w) - 1) < 1e-6 && math::Norm(p.pos) < 1e-6;
}

uint32_t IdFrom(const json& v) {
    if (v.is_number_unsigned()) return (uint32_t)v.get<uint64_t>();
    if (v.is_number_integer()) return (uint32_t)v.get<int64_t>();
    if (v.is_string()) return (uint32_t)std::strtoull(v.get<std::string>().c_str(), nullptr, 10);
    return 0;
}

}  // namespace

std::vector<UniverseRecord> ParseSteamVrLighthouseDb(std::string_view text, std::string* error) {
    std::vector<UniverseRecord> out;
    json j = json::parse(text.begin(), text.end(), nullptr, false);
    if (j.is_discarded() || !j.is_object()) {
        if (error) *error = "invalid JSON";
        return out;
    }
    if (!j.contains("known_universes") || !j["known_universes"].is_array()) return out;
    for (auto& u : j["known_universes"]) {
        if (!u.is_object() || !u.contains("base_stations") || !u["base_stations"].is_array()) continue;
        UniverseRecord r;
        r.source = "steamvr";
        r.fixed = true;
        if (u.contains("id")) r.id = u["id"].is_string() ? u["id"].get<std::string>() : u["id"].dump();
        // first pass: poses relative to their targets
        struct Raw {
            uint32_t id, target;
            Pose pose;
        };
        std::vector<Raw> raw;
        for (auto& b : u["base_stations"]) {
            if (!b.is_object() || !b.contains("base_serial_number") || !b.contains("target_pose")) continue;
            const json& tp = b["target_pose"];
            if (!tp.is_object() || !tp.contains("pose")) continue;
            auto p = PoseFrom7(tp["pose"]);
            if (!p) continue;
            uint32_t target = tp.contains("target_serial_number") ? IdFrom(tp["target_serial_number"]) : 0;
            raw.push_back({IdFrom(b["base_serial_number"]), target, *p});
        }
        for (auto& x : raw)
            if (x.target == 0 && IsIdentity(x.pose)) r.referenceId = x.id;
        if (!r.referenceId) continue;
        // resolve (short) chains: origin <- target <- base, in both readings
        std::map<uint32_t, std::pair<Pose, Pose>> resolved;   // id -> (origin_from_base reading, inverse reading)
        resolved[r.referenceId] = {Pose{}, Pose{}};
        for (int pass = 0; pass < 4; ++pass)
            for (auto& x : raw) {
                if (resolved.count(x.id)) continue;
                uint32_t t = x.target ? x.target : r.referenceId;
                auto it = resolved.find(t);
                if (it == resolved.end()) continue;
                resolved[x.id] = {it->second.first * x.pose, it->second.second * x.pose.Inverse()};
            }
        for (auto& [id, poses] : resolved) {
            UniverseBase e;
            e.id = id;
            e.refFromBase = poses.first;
            if (id != r.referenceId) e.alternative = poses.second;
            r.bases.push_back(e);
        }
        if (r.bases.size() >= 2) out.push_back(std::move(r));
    }
    return out;
}

std::string UniverseToJson(const UniverseRecord& u) {
    json j;
    char id[16];
    std::snprintf(id, sizeof id, "%08X", u.referenceId);
    j["reference"] = id;
    j["bases"] = json::object();
    for (auto& b : u.bases) {
        std::snprintf(id, sizeof id, "%08X", b.id);
        const Pose& p = b.refFromBase;
        j["bases"][id] = {{"rotation", {p.rot.w, p.rot.x, p.rot.y, p.rot.z}}, {"position", {p.pos.x, p.pos.y, p.pos.z}}, {"samples", b.samples}};
    }
    if (u.hasUp) {
        j["up"] = {u.upInRef.x, u.upInRef.y, u.upInRef.z};
        j["upSamples"] = u.upSamples;
    }
    return j.dump();
}

std::optional<UniverseRecord> UniverseFromJson(std::string_view text) {
    json j = json::parse(text.begin(), text.end(), nullptr, false);
    if (j.is_discarded() || !j.is_object() || !j.contains("reference") || !j["reference"].is_string()) return std::nullopt;
    UniverseRecord u;
    u.source = "saved";
    u.referenceId = (uint32_t)std::strtoul(j["reference"].get<std::string>().c_str(), nullptr, 16);
    if (j.contains("bases") && j["bases"].is_object())
        for (auto& [k, v] : j["bases"].items()) {
            if (!v.is_object() || !v.contains("rotation") || !v.contains("position")) continue;
            const json& r = v["rotation"];
            const json& p = v["position"];
            if (!r.is_array() || r.size() != 4 || !p.is_array() || p.size() != 3) continue;
            UniverseBase b;
            b.id = (uint32_t)std::strtoul(k.c_str(), nullptr, 16);
            b.refFromBase.rot = Quat{r[0].get<double>(), r[1].get<double>(), r[2].get<double>(), r[3].get<double>()}.Normalized();
            b.refFromBase.pos = {p[0].get<double>(), p[1].get<double>(), p[2].get<double>()};
            b.samples = v.value("samples", 0);
            u.bases.push_back(b);
        }
    if (j.contains("up") && j["up"].is_array() && j["up"].size() == 3) {
        u.upInRef = {j["up"][0].get<double>(), j["up"][1].get<double>(), j["up"][2].get<double>()};
        u.upSamples = j.value("upSamples", 0);
        u.hasUp = u.upSamples > 0 && math::Norm(u.upInRef) > 0.5;
    }
    return u;
}

}  // namespace lhp::tracking::solver

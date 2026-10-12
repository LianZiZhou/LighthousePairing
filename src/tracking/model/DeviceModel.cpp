#include "tracking/model/DeviceModel.h"

#include <nlohmann/json.hpp>

#include <algorithm>

namespace lhp::tracking::model {

namespace {

using nlohmann::json;

math::Vec3 V(const json& j, math::Vec3 def = {}) {
    if (!j.is_array() || j.size() < 3) return def;
    return {j[0].get<double>(), j[1].get<double>(), j[2].get<double>()};
}

math::Pose PoseFrom(const json& j) {
    math::Pose p;
    if (!j.is_object()) return p;
    if (j.contains("plus_x") && j.contains("plus_z")) p.rot = math::Quat::FromAxes(V(j["plus_x"], {1, 0, 0}), V(j["plus_z"], {0, 0, 1}));
    if (j.contains("position")) p.pos = V(j["position"]);
    return p;
}

}  // namespace

std::optional<DeviceModel> ParseDeviceModel(std::string_view text, std::string* error) {
    json j = json::parse(text.begin(), text.end(), nullptr, false);
    if (j.is_discarded() || !j.is_object()) {
        if (error) *error = "invalid JSON";
        return std::nullopt;
    }
    DeviceModel m;
    try {
        m.serial = j.value("device_serial_number", "");
        for (auto& c : m.serial) c = (char)std::toupper((unsigned char)c);
        m.modelNumber = j.value("model_number", "");
        m.deviceClass = j.value("device_class", "");
        if (j.contains("lighthouse_config")) {
            const json& l = j["lighthouse_config"];
            if (l.contains("modelPoints"))
                for (auto& p : l["modelPoints"]) m.points.push_back(V(p));
            if (l.contains("modelNormals"))
                for (auto& n : l["modelNormals"]) m.normals.push_back(math::Normalize(V(n)));
            if (l.contains("channelMap"))
                for (auto& c : l["channelMap"]) m.channelMap.push_back(c.get<int>());
        }
        if (j.contains("head")) m.head = PoseFrom(j["head"]);
        if (j.contains("imu")) {
            const json& i = j["imu"];
            m.imu.accBias = V(i.value("acc_bias", json()));
            m.imu.accScale = V(i.value("acc_scale", json()), {1, 1, 1});
            m.imu.gyroBias = V(i.value("gyro_bias", json()));
            m.imu.gyroScale = V(i.value("gyro_scale", json()), {1, 1, 1});
            m.imu.mount = PoseFrom(i);
        }
    } catch (const std::exception& e) {
        if (error) *error = e.what();
        return std::nullopt;
    }
    if (!m.channelMap.empty()) {
        int maxId = 0;
        for (int id : m.channelMap) maxId = std::max(maxId, id);
        m.modelFromReported.assign(maxId + 1, -1);
        for (int i = 0; i < (int)m.channelMap.size() && i < (int)m.points.size(); ++i)
            if (m.channelMap[i] >= 0) m.modelFromReported[m.channelMap[i]] = i;
    }
    if (!m.HasGeometry()) {
        if (error) *error = "config has no lighthouse_config geometry";
        return std::nullopt;
    }
    return m;
}

}  // namespace lhp::tracking::model

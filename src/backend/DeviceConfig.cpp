#include "backend/DeviceConfig.h"

#include "backend/ConsoleParser.h"
#include "util/Text.h"

#include <nlohmann/json.hpp>

namespace lhp::devcfg {

DeviceConfig Parse(std::string_view json) {
    DeviceConfig c;
    nlohmann::json j = nlohmann::json::parse(json.begin(), json.end(), nullptr, false);
    if (j.is_discarded() || !j.is_object()) return c;
    auto str = [&](const char* k) -> std::string {
        auto it = j.find(k);
        return it != j.end() && it->is_string() ? it->get<std::string>() : std::string();
    };
    c.serial = str("device_serial_number");
    c.model = str("model_number");
    c.manufacturer = str("manufacturer");
    c.deviceClass = str("device_class");
    c.renderModel = str("render_model");
    c.controllerRole = str("tracked_controller_role");
    for (auto& [k, v] : j.items()) {
        if (v.is_string()) c.scalars.emplace_back(k, v.get<std::string>());
        else if (v.is_number() || v.is_boolean()) c.scalars.emplace_back(k, v.dump());
    }
    if (!c.serial.empty()) {
        for (auto& ch : c.serial) ch = (char)std::toupper((unsigned char)ch);
    }
    c.ok = !c.serial.empty() || !c.model.empty();
    return c;
}

void ApplyTo(const DeviceConfig& c, DeviceInfo& d, ConfigSource source) {
    if (!c.ok) return;
    if (!c.serial.empty()) {
        d.serial = c.serial;
        d.serialFromEvent = true;   // the device told us itself
    }
    if (!c.model.empty()) d.model = c.model;
    d.manufacturer = c.manufacturer;
    d.renderModel = c.renderModel;
    d.configClass = c.deviceClass;
    d.controllerRole = c.controllerRole;
    d.configProps = c.scalars;
    d.configSource = source;

    DeviceClass cls = parser::ClassifyText(c.model + " " + c.renderModel);
    if (cls == DeviceClass::Unknown) {
        std::string m = c.manufacturer;
        for (auto& ch : m) ch = (char)std::tolower((unsigned char)ch);
        if (Contains(m, "tundra")) cls = DeviceClass::TundraTracker;
        else if (c.deviceClass == "generic_tracker") cls = DeviceClass::ViveTracker;
        else if (c.deviceClass == "controller" && Contains(m, "htc")) cls = DeviceClass::ViveWand;
        else if (c.deviceClass == "controller" && Contains(m, "valve")) cls = DeviceClass::IndexController;
    }
    if (cls != DeviceClass::Unknown) d.deviceClass = cls;
    if (d.deviceClass == DeviceClass::IndexController || d.deviceClass == DeviceClass::ViveWand) {
        if (c.controllerRole == "LeftHand") d.hand = Hand::Left;
        else if (c.controllerRole == "RightHand") d.hand = Hand::Right;
        else {
            std::string t = c.model + " " + c.renderModel;
            for (auto& ch : t) ch = (char)std::tolower((unsigned char)ch);
            bool l = Contains(t, "left"), r = Contains(t, "right");
            if (l != r) d.hand = l ? Hand::Left : Hand::Right;
            else if (d.deviceClass == DeviceClass::IndexController) d.hand = parser::DetectHand(d);
        }
    }
}

std::map<std::string, std::string> ParseTrackerRoles(std::string_view vrsettingsJson) {
    std::map<std::string, std::string> out;
    nlohmann::json j = nlohmann::json::parse(vrsettingsJson.begin(), vrsettingsJson.end(), nullptr, false, true);
    if (j.is_discarded() || !j.is_object()) return out;
    auto it = j.find("trackers");
    if (it == j.end() || !it->is_object()) return out;
    for (auto& [k, v] : it->items()) {
        if (!v.is_string()) continue;
        // "/devices/htc/vive_trackerLHR-3A7C0E21": "TrackerRole_Waist"
        for (auto& sn : parser::FindLhrSerials({k})) out[sn] = v.get<std::string>();
    }
    return out;
}

std::string TrackerRoleText(std::string_view role) {
    static const std::pair<const char*, const char*> names[] = {
        {"TrackerRole_None", "未分配"},           {"TrackerRole_Handed", "手持"},
        {"TrackerRole_LeftFoot", "左脚"},          {"TrackerRole_RightFoot", "右脚"},
        {"TrackerRole_LeftShoulder", "左肩"},      {"TrackerRole_RightShoulder", "右肩"},
        {"TrackerRole_LeftElbow", "左肘"},         {"TrackerRole_RightElbow", "右肘"},
        {"TrackerRole_LeftKnee", "左膝"},          {"TrackerRole_RightKnee", "右膝"},
        {"TrackerRole_LeftWrist", "左手腕"},       {"TrackerRole_RightWrist", "右手腕"},
        {"TrackerRole_LeftAnkle", "左脚踝"},       {"TrackerRole_RightAnkle", "右脚踝"},
        {"TrackerRole_Waist", "腰部"},             {"TrackerRole_Chest", "胸部"},
        {"TrackerRole_Camera", "摄像机"},          {"TrackerRole_Keyboard", "键盘"},
    };
    for (auto& [k, v] : names)
        if (role == k) return v;
    if (StartsWith(role, "TrackerRole_")) return std::string(role.substr(12));
    return std::string(role);
}

std::string ProductName(const DeviceInfo& d) {
    if (d.model.empty()) return {};
    if (d.manufacturer.empty()) return d.model;
    std::string lm = d.model, lf = d.manufacturer;
    for (auto& ch : lm) ch = (char)std::tolower((unsigned char)ch);
    for (auto& ch : lf) ch = (char)std::tolower((unsigned char)ch);
    std::string brand = lf.substr(0, lf.find(' '));   // "tundra labs" -> "tundra"
    if (Contains(lm, brand)) return d.model;
    return d.manufacturer + " " + d.model;
}

}  // namespace lhp::devcfg

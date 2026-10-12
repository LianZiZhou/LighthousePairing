#pragma once
// Device identity from the JSON config stored on every lighthouse device.
//
// lighthouse_console `downloadconfig <file>` reads it over the radio link (read-only; it is the
// same blob SteamVR fetches on connect), and SteamVR caches a copy per device under
// <Steam>\config\lighthouse\<serial>\config.json. It carries the exact serial, model number
// ("VIVE Tracker 3.0 MV", "Knuckles Left"), manufacturer, render model and controller role -
// the same strings SteamVR / OpenVR tools such as Space Calibrator show.
#include "core/Model.h"

#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace lhp::devcfg {

struct DeviceConfig {
    bool ok = false;
    std::string serial;          // device_serial_number
    std::string model;           // model_number
    std::string manufacturer;
    std::string deviceClass;     // device_class: controller / generic_tracker / hmd ...
    std::string renderModel;     // render_model
    std::string controllerRole;  // tracked_controller_role (LeftHand / RightHand / "")
    std::vector<std::pair<std::string, std::string>> scalars;   // every top-level scalar, for display
};

DeviceConfig Parse(std::string_view json);

/// Copy identity fields into a DeviceInfo (serial, model, class, hand ...).
void ApplyTo(const DeviceConfig& c, DeviceInfo& d, ConfigSource source);

/// SteamVR tracker role assignments (steamvr.vrsettings "trackers"): LHR serial -> TrackerRole_*.
std::map<std::string, std::string> ParseTrackerRoles(std::string_view vrsettingsJson);
/// "TrackerRole_Waist" -> "腰部"; unknown roles are returned without the prefix.
std::string TrackerRoleText(std::string_view role);

/// "VIVE Tracker 3.0 MV" / "Valve Knuckles Left": manufacturer + model unless redundant.
std::string ProductName(const DeviceInfo& d);

}  // namespace lhp::devcfg

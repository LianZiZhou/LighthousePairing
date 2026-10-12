#pragma once
// L3: universe (base station layout) file formats.
//
//  * SteamVR <Steam>\config\lighthouse\lighthousedb.json, written by the lighthouse driver:
//      known_universes[].base_stations[] = { base_serial_number (decimal OOTX id),
//                                            target_pose { pose: [qx qy qz qw px py pz], target_serial_number, variance } }
//    The origin base has an identity pose (and target 0). Whether `pose` is origin_from_base or its inverse is
//    not documented (libsurvive reads it as the inverse); both readings are kept and live observations decide
//    (Scene::AddBootstrapSample).
//  * Our own saved universe (inside DataDir\base_stations.json, key "universe").
#include "tracking/solver/Scene.h"

#include <string>
#include <string_view>
#include <vector>

namespace lhp::tracking::solver {

std::vector<UniverseRecord> ParseSteamVrLighthouseDb(std::string_view json, std::string* error = nullptr);

/// Our format, as a JSON object string.
std::string UniverseToJson(const UniverseRecord& u);
std::optional<UniverseRecord> UniverseFromJson(std::string_view json);

}  // namespace lhp::tracking::solver

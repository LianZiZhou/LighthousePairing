#pragma once
// L2/L3 input: geometry of a tracked device, from its JSON config (the same file SteamVR caches as
// <Steam>\config\lighthouse\<serial>\config.json and lighthouse_console `downloadconfig` writes).
#include "tracking/math/Linalg.h"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lhp::tracking::model {

struct ImuCalibration {
    math::Vec3 accBias, accScale{1, 1, 1}, gyroBias, gyroScale{1, 1, 1};
    math::Pose mount;            // IMU frame -> device frame (plus_x / plus_z / position)
};

struct DeviceModel {
    std::string serial;          // device_serial_number
    std::string modelNumber;
    std::string deviceClass;
    std::vector<math::Vec3> points;    // sensor positions, device frame (metres)
    std::vector<math::Vec3> normals;
    std::vector<int> channelMap;       // reported sensor id -> model index (-1 = unused)
    math::Pose head;                   // device frame -> "head"/tracking origin (plus_x / plus_z / position)
    ImuCalibration imu;
    bool HasGeometry() const { return !points.empty() && points.size() == normals.size(); }
    /// Model index for a reported sensor id, or -1.
    int MapSensor(int reported) const {
        if (channelMap.empty()) return reported < (int)points.size() ? reported : -1;
        if (reported < 0 || reported >= (int)channelMap.size()) return -1;
        int m = channelMap[reported];
        return m >= 0 && m < (int)points.size() ? m : -1;
    }
};

std::optional<DeviceModel> ParseDeviceModel(std::string_view json, std::string* error = nullptr);

/// Provides device configs to the tracking pipeline. Implemented outside tracking (SteamVR cache,
/// lighthouse_console downloadconfig, recordings ...).
class ModelProvider {
public:
    virtual ~ModelProvider() = default;
    /// JSON config of the device currently behind `receiverSerial` (or of the wired device itself).
    virtual std::optional<std::string> ConfigFor(const std::string& receiverSerial) = 0;
};

}  // namespace lhp::tracking::model

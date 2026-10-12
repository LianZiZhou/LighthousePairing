#pragma once
// Synthetic Lighthouse 2.0 setup: virtual base stations and devices produce the same HID reports a real
// receiver would (watchman 0x23 packets, raw1 light data, OOTX, IMU). Used by demo mode and by tests to
// run the real decode -> solve pipeline against known ground truth.
#include "tracking/io/ReportSource.h"
#include "tracking/lighthouse/BaseStation.h"
#include "tracking/math/Linalg.h"
#include "tracking/model/DeviceModel.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace lhp::tracking::sim {

struct SimBase {
    int channel = 0;
    math::Pose worldFromBase;
    lighthouse::BaseCalibration cal;   // id / fcal; accelDir is derived from the pose
};

struct SimDevice {
    std::string receiverSerial;
    std::string serial, modelNumber;
    int sensorCount = 20;
    double radius = 0.04;
    /// Device pose at time t (seconds).
    std::function<math::Pose(double t)> trajectory;
};

struct SimConfig {
    std::vector<SimBase> bases;
    std::vector<SimDevice> devices;
    double angleNoiseRad = 2e-5;
    double missProbability = 0.02;    // per sweep
    uint32_t seed = 1;
};

/// A ready-made scene (2 base stations, several moving devices) for demo mode.
SimConfig DemoScene(const std::vector<std::string>& receiverSerials);

/// JSON config (same shape as a real device config) for a simulated device.
std::string DeviceConfigJson(const SimDevice& d);

/// ModelProvider answering with the simulated devices' configs (by receiver serial).
std::shared_ptr<model::ModelProvider> MakeModelProvider(const SimConfig& cfg);

/// One report stream per device. realtime = pace output to the host clock.
std::unique_ptr<io::ReportSource> OpenSimSource(const SimConfig& cfg, size_t deviceIndex, uint16_t sourceId, bool realtime);

/// OOTX bit stream (preamble, length, payload, CRC, stuffed sync bits) for a calibration, for tests.
std::vector<int> EncodeOotx(const lighthouse::BaseCalibration& c);
std::vector<uint8_t> EncodeOotxV15(const lighthouse::BaseCalibration& c);

}  // namespace lhp::tracking::sim

#pragma once
// L4 output: immutable picture of the whole tracking state, published ~30x per second.
// Everything the UI shows comes from here; the UI never touches pipeline objects.
#include "tracking/lighthouse/BaseStation.h"
#include "tracking/lighthouse/Lh2Processor.h"
#include "tracking/math/Linalg.h"
#include "tracking/protocol/WatchmanDecoder.h"
#include "tracking/service/HostGuard.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace lhp::tracking::service {

/// One sensor's latest data from one base station.
struct SensorBaseData {
    uint8_t channel = 0;
    bool has[2]{};
    double angleDeg[2]{};        // measured (centred) angle per plane
    double predictedDeg[2]{};    // from the latest pose
    double residualDeg[2]{};     // measured - predicted
    double ageMs[2]{};           // since the last hit
    uint64_t hits[2]{};
    bool rejected[2]{};          // dropped as an outlier by the solver
};

struct SensorSnapshot {
    int index = 0;               // model index
    int reportedId = -1;         // id on the wire (channelMap inverse)
    math::Vec3 point, normal;    // device frame
    std::vector<SensorBaseData> bases;
    bool litNow = false;         // hit within the last ~50 ms
};

struct PoseSample {
    double t = 0;                // seconds (device clock)
    math::Vec3 pos;
    double rmsDeg = 0;
    double sigmaMm = 0;
    int used = 0;
};

struct ImuSnapshot {
    bool valid = false;
    int16_t rawAccel[3]{}, rawGyro[3]{};
    math::Vec3 accel;            // m/s^2, device IMU frame, calibrated
    math::Vec3 gyro;             // rad/s
    double rateHz = 0;
    math::Vec3 gravityDevice;    // low-passed accel direction in the device frame
    double gravityErrorDeg = -1; // angle between IMU gravity and world up through the optical pose (-1 = n/a)
};

struct DeviceSnapshot {
    // identity
    std::string receiverSerial;
    uint16_t pid = 0;
    std::string product;
    std::string deviceSerial, modelNumber;
    bool hasModel = false;
    std::string modelNote;       // why there is no model (yet)
    // transport / decoding
    protocol::LightMode lightMode = protocol::LightMode::Unknown;
    std::string modeNote;        // passive/active explanation
    protocol::DecoderStats decoder;
    double reportRate = 0;       // HID reports per second (host clock)
    double hostIntervalMaxMs = 0;
    std::array<lighthouse::ChannelStats, lighthouse::kMaxChannels> channels{};
    ImuSnapshot imu;
    int batteryPct = -1;
    bool charging = false;
    // solve
    bool hasPose = false;
    double poseAgeMs = 0;
    math::Pose worldFromDevice;
    math::Vec3 velocity;         // m/s
    math::Vec3 angularVelocity;  // rad/s (when solved jointly with the pose)
    bool velocityEstimated = false;
    double rmsDeg = 0, maxDeg = 0, initialRmsDeg = 0;
    double positionSigmaMm = 0, rotationSigmaDeg = 0;
    std::array<double, 36> covariance{};
    int used = 0, rejected = 0, iterations = 0, seedsTried = 0, basesUsed = 0;
    double solveUs = 0;
    uint64_t solves = 0, solveFailures = 0;
    std::string solveNote;
    std::vector<SensorSnapshot> sensors;
    std::vector<math::Vec3> trail;     // recent world positions
    std::vector<PoseSample> history;   // ~60 s
};

struct BaseSnapshot {
    int channel = 0;
    bool isReference = false;
    bool hasCalibration = false;
    bool calibrationFromCache = false;
    lighthouse::BaseCalibration cal;
    bool hasPose = false;
    math::Pose worldFromBase;
    int bootstrapSamples = 0;
    double bootstrapSpreadMm = 0;
    // aggregated over devices
    double measuredHz = 0, freqErrorHz = 0;
    uint64_t syncs = 0, skippedSyncs = 0, sweeps = 0;
    bool conflict = false;
    lighthouse::OotxStats ootx;
    std::vector<std::string> seenBy;   // receiver serials
};

struct TrackingSnapshot {
    uint64_t seq = 0;
    bool running = false;
    bool replay = false;           // not live HID (recording or simulation)
    bool simulated = false;
    std::string replayPath;
    bool recording = false;
    std::string recordingPath;
    size_t recordedReports = 0;
    HostState host;
    bool activeRequested = false;
    bool activeAllowed = false;
    std::string activeNote;
    bool gravityAligned = false;
    int referenceChannel = -1;
    double referenceHeight = 2.0;
    std::vector<DeviceSnapshot> devices;
    std::vector<BaseSnapshot> bases;
    std::vector<std::string> log;      // recent service messages
};

using SnapshotPtr = std::shared_ptr<const TrackingSnapshot>;

}  // namespace lhp::tracking::service

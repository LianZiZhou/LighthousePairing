#pragma once
// L4: one report stream (one receiver / wired device) -> decode -> LH2 angles -> pose.
// Runs on its own thread; publishes into a DeviceSnapshot under its own mutex.
#include "tracking/hid/HidDevice.h"
#include "tracking/io/Recording.h"
#include "tracking/io/ReportSource.h"
#include "tracking/lighthouse/Lh2Processor.h"
#include "tracking/model/DeviceModel.h"
#include "tracking/protocol/WatchmanDecoder.h"
#include "tracking/service/Snapshot.h"
#include "tracking/solver/PoseSolver.h"
#include "tracking/solver/Scene.h"

#include <atomic>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace lhp::tracking::service {

/// Remembers base station calibrations between runs (OOTX takes ~10 s to arrive). Implemented outside tracking.
class CalibrationCache {
public:
    virtual ~CalibrationCache() = default;
    virtual std::optional<lighthouse::BaseCalibration> Load(int channel) = 0;
    virtual void Save(int channel, const lighthouse::BaseCalibration& cal) = 0;
};

/// The world shared by all pipelines (owned by TrackingService).
struct SharedWorld {
    std::mutex mutex;
    solver::Scene scene;
    std::shared_ptr<CalibrationCache> cache;   // may be null
};

struct PipelineConfig {
    std::shared_ptr<model::ModelProvider> models;
    SharedWorld* world = nullptr;
    std::function<void(const std::string&)> log;
    std::optional<hid::HidDeviceInfo> hidInfo;     // set for live sources (needed to send mode commands)
};

class DevicePipeline {
public:
    DevicePipeline(std::unique_ptr<io::ReportSource> source, PipelineConfig cfg);
    ~DevicePipeline();
    DevicePipeline(const DevicePipeline&) = delete;
    DevicePipeline& operator=(const DevicePipeline&) = delete;

    void Start();
    void Stop();
    bool Ended() const { return ended_; }
    const io::SourceInfo& Info() const { return info_; }

    void SetActiveAllowed(bool allowed) { activeAllowed_ = allowed; }
    void SetRecorder(std::shared_ptr<io::RecordingWriter> w);
    /// Copy of the current state (thread-safe).
    DeviceSnapshot Snapshot() const;
    /// Highest point of the device model, world Y (for "set floor here"); NaN if no pose.
    double LowestSensorY() const;

private:
    void Run();
    void Handle(const io::RawReport& r);
    void OnImu(const protocol::ImuSample& s);
    void OnSync(const protocol::Lh2Sync& s);
    void OnSweep(const protocol::Lh2Sweep& w);
    void TrySolve(protocol::Ticks now);
    void MaybeSwitchMode(uint64_t hostUs);
    void TryLoadModel(uint64_t hostUs);

    io::SourceInfo info_;
    std::unique_ptr<io::ReportSource> source_;
    PipelineConfig cfg_;
    std::thread thread_;
    std::atomic<bool> stop_{false}, ended_{false}, activeAllowed_{false};
    mutable std::mutex recMtx_;
    std::shared_ptr<io::RecordingWriter> recorder_;

    // ---- pipeline-thread state (guarded by mtx_ when read by Snapshot())
    mutable std::mutex mtx_;
    protocol::WatchmanDecoder decoder_;
    lighthouse::Lh2Processor lh2_;
    std::optional<model::DeviceModel> model_;
    std::string modelNote_ = "等待设备配置";
    uint64_t lastModelTryUs_ = 0;
    std::vector<protocol::Event> events_;

    struct Obs {
        lighthouse::Lh2Observation o;
        bool valid = false;
    };
    // [channel][model sensor][plane]
    std::map<int, std::vector<std::array<Obs, 2>>> latest_;
    std::map<int, std::vector<std::array<uint64_t, 2>>> hits_;
    protocol::Ticks lastLightTime_ = 0;
    protocol::Ticks lastSolveTime_ = 0;

    // solve results
    std::optional<solver::SolveResult> lastSolve_;
    std::vector<solver::Measurement> lastMeas_;
    std::vector<int> lastMeasChannel_;   // channel per measurement
    std::vector<solver::BaseView> lastBases_;
    std::map<int, math::Pose> baseLocal_;   // per channel: last device pose in that base's frame (bootstrap)
    protocol::Ticks poseTime_ = 0;
    math::Pose lastGoodPose_;
    uint64_t solves_ = 0, solveFailures_ = 0;
    double solveUs_ = 0;
    int basesUsed_ = 0;
    std::deque<PoseSample> history_;
    std::deque<math::Vec3> trail_;
    math::Vec3 velocity_;

    // imu
    ImuSnapshot imu_;
    protocol::Ticks lastImuTime_ = 0, firstImuTime_ = 0;
    uint64_t imuCount_ = 0;
    int battery_ = -1;
    bool charging_ = false;

    // host-side rates
    uint64_t reportCount_ = 0, rateWindowStartUs_ = 0, rateWindowCount_ = 0, lastReportUs_ = 0;
    double reportRate_ = 0, maxIntervalMs_ = 0, windowMaxIntervalMs_ = 0;

    // active mode
    uint64_t lastModeCmdUs_ = 0;
    std::string modeNote_;
};

}  // namespace lhp::tracking::service

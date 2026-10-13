#pragma once
// L4: one report stream (one receiver / wired device) -> decode -> LH2 angles -> pose.
// Runs on its own thread; publishes into a DeviceSnapshot under its own mutex.
#include "tracking/filter/Imu.h"
#include "tracking/filter/TrackerFilter.h"
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

/// Remembers base station calibrations and universes between runs (OOTX takes ~10 s to arrive, base poses
/// minutes of observation). Implemented outside tracking.
class CalibrationCache {
public:
    virtual ~CalibrationCache() = default;
    virtual std::optional<lighthouse::BaseCalibration> Load(int channel) = 0;
    virtual void Save(int channel, const lighthouse::BaseCalibration& cal) = 0;
    /// Universes to restore from, in priority order (e.g. SteamVR's lighthousedb, then our own).
    virtual std::vector<solver::UniverseRecord> LoadUniverses() { return {}; }
    virtual void SaveUniverse(const solver::UniverseRecord&) {}
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
    /// Diagnostics: one CSV-ish line per notable pipeline event (light update, IMU sample, reset, clock anomaly).
    std::function<void(const std::string&)> trace;
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
    struct QueuedImu {
        double t;
        filter::ImuSi si;
        bool still;
        double stillSeconds;
        math::Vec3 meanAcc;
    };
    /// Delayed fusion: the filter runs on the light's clock; IMU samples wait in imuQueue_ until this time.
    double ImuHorizon() const;
    void FeedImuUpTo(double horizon);
    void ApplyImu(const QueuedImu& q);
    void OnSync(const protocol::Lh2Sync& s);
    void OnSweep(const protocol::Lh2Sweep& w);
    /// Single-frame solve (SteamVR's CLighthousePoseSolver role): starts the filter, checks it, bootstraps bases.
    void TrySolve(protocol::Ticks now);
    /// Feeds buffered sweep angles to the filter: one update per completed base rotation (`channel`), plus any
    /// stale leftovers (channel = -1: only those).
    void FlushLight(int channel);
    struct BaseInfo {
        bool known;
        bool canBootstrap;
        bool calibrated;
        math::Pose worldFromBase;
        lighthouse::AxisCal cal[2];
    };
    struct FrameSolve {
        std::map<int, std::vector<solver::Measurement>> byChannel;
        std::map<int, BaseInfo> info;
        math::Pose worldFromRef;
        std::vector<solver::Measurement> meas;
        std::vector<int> measCh;
        std::vector<solver::BaseView> bases;
        bool allCalibrated = true;
        solver::SolveResult res;
    };
    /// Single-frame solve of the latest angles at `ref`, motion-compensated with the given velocities.
    FrameSolve SolveRecent(protocol::Ticks ref, const math::Pose* seed, const math::Vec3& velocity, const math::Vec3& angularVelocity);
    static bool GoodSolve(const FrameSolve& f);
    /// Keyframes for the multi-position base refinement (Scene / BaseRefiner): device pose from the reference
    /// base alone + the other bases' angles at the same moment.
    void CollectRefineKeyframes(protocol::Ticks ref, const FrameSolve& f);
    /// The filter disagrees with the light: re-anchor on a single-frame solve, or reset.
    void Recover(const std::string& reason);
    void SyncWorld();               // re-express the filter when the world frame changed
    void ResetFilter(const std::string& reason);
    void OnTimeBaseReset(const std::string& reason);   // device timestamps changed epoch
    void Trace(const char* fmt, ...) const;
    bool CheckClock(protocol::Ticks t);
    void UpdateTrackState(double t);
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
    protocol::Ticks poseTime_ = 0;          // last good single-frame solve
    math::Pose lastGoodPose_;
    uint64_t solves_ = 0, solveFailures_ = 0;
    double solveUs_ = 0;
    int basesUsed_ = 0;
    std::deque<PoseSample> history_;
    std::deque<math::Vec3> trail_;
    int batchDisagree_ = 0;

    // tracking filter
    std::optional<filter::TrackerFilter> filter_;
    filter::StillnessDetector still_;
    FilterSnapshot fs_;                     // diagnostics, updated as we go
    TrackState state_ = TrackState::NoModel;
    double trackingSince_ = 0;
    double lastLightUpdateT_ = 0;           // device seconds of the last applied light update
    double lastHistoryT_ = 0, lastUpSampleT_ = 0;
    uint64_t timeBaseResets_ = 0;
    int badLight_ = 0;                      // consecutive rejected light batches
    uint64_t worldVersion_ = 0;
    math::Pose worldFromRefSeen_;
    bool worldAligned_ = false;
    struct PendingObs {
        lighthouse::Lh2Observation o;
        int sensor;                         // model index
    };
    std::map<int, std::vector<PendingObs>> pending_;   // per channel, not yet fused
    std::map<int, std::vector<filter::LightMeas>> lastLightMeas_;   // per channel: last fused batch (for display)
    std::map<int, filter::LightUpdate> lastLight_;
    std::map<std::string, uint64_t> skipReasons_;
    std::map<int, uint64_t> calKeys_;       // per channel: calibration the filter last used
    std::map<int, int> refineCounts_;       // per channel: base refinements applied when the filter last used it
    std::map<int, uint32_t> ootxLogged_;    // per channel: base id already announced in the log
    double lightRateWindowT_ = 0;
    uint64_t lightRateWindowN_ = 0;

    // imu
    ImuSnapshot imu_;
    std::deque<QueuedImu> imuQueue_;        // not yet fused (newer than the light waiting to be fused)
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

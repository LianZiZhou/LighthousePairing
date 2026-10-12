#pragma once
// L4: owns the pipelines (one per receiver / wired device or per replayed source), the shared world,
// device discovery, the active-mode policy, recording, and publishes TrackingSnapshots.
#include "tracking/model/DeviceModel.h"
#include "tracking/service/DevicePipeline.h"
#include "tracking/service/Snapshot.h"

#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace lhp::tracking::service {

struct ServiceOptions {
    bool activeRequested = false;    // user allows sending light-mode commands (only when SteamVR is not running)
};

class TrackingService {
public:
    explicit TrackingService(std::shared_ptr<model::ModelProvider> models, std::shared_ptr<CalibrationCache> cache = nullptr);
    ~TrackingService();

    /// Live mode: discover receivers and start reading (passive by default).
    void Start();
    /// Replay a recording instead of live devices (stops live sources).
    bool StartReplay(const std::string& path, std::string* error);
    /// Run the pipelines on arbitrary sources (replay, simulation). `models` overrides the provider.
    void StartSources(std::vector<std::unique_ptr<io::ReportSource>> sources, const std::string& label, const std::string& logLine,
                      std::shared_ptr<model::ModelProvider> models = nullptr, bool simulated = false);
    void Stop();
    bool Running() const { return running_; }

    void SetOptions(const ServiceOptions& o);
    ServiceOptions Options() const;

    bool StartRecording(const std::string& path, std::string* error);
    void StopRecording();

    void ResetWorld();
    void SetReferenceHeight(double metres);
    /// Refine the world "up" with the accelerometers of still devices (on by default).
    void SetUseDeviceTilt(bool on);
    /// Put the floor at the lowest sensor of this device (device lying on the floor).
    bool SetFloorFromDevice(const std::string& receiverSerial);

    SnapshotPtr Snapshot() const;

private:
    void Loop();          // publishes snapshots (~30 Hz); must never block on slow work
    void Housekeeping();  // discovery, host probe, saving
    void Discover();
    void Publish();
    void Log(const std::string& msg);
    void MaybeSaveUniverse();

    std::shared_ptr<model::ModelProvider> models_;
    SharedWorld world_;
    int knownUniverses_ = 0;   // guarded by world_.mutex
    // Lock order: mtx_ -> pipeline mutex -> world_.mutex; logMtx_ is a leaf.
    mutable std::mutex mtx_;   // pipelines_, options_, recorder_, replay state
    std::map<std::string, std::unique_ptr<DevicePipeline>> pipelines_;   // key: source serial / id
    ServiceOptions options_;
    std::shared_ptr<io::RecordingWriter> recorder_;
    std::string recordPath_;
    mutable std::mutex logMtx_;   // leaf lock for log_
    std::deque<std::string> log_;
    bool replay_ = false;
    bool simulated_ = false;
    std::string replayPath_;

    std::thread thread_, hkThread_;
    std::atomic<bool> running_{false}, stop_{false};
    HostState host_;   // guarded by mtx_
    uint64_t seq_ = 0;
    mutable std::mutex snapMtx_;
    SnapshotPtr snapshot_;
};

}  // namespace lhp::tracking::service

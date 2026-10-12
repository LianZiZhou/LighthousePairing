#pragma once
// L3: the shared world: base station calibrations and poses.
//
// World frame: +Y up (gravity from the reference base's accelerometer, OOTX accel_dir), the reference
// base looks along world -Z, and sits at (0, height, 0) above the floor plane y = 0.
// Other bases are stored relative to the reference base (refFromBase), learned from devices that see
// both, so re-aligning the world (OOTX arrives, floor set) never invalidates them.
//
// Not thread-safe; the owner (TrackingService) serialises access.
#include "tracking/lighthouse/BaseStation.h"
#include "tracking/math/Linalg.h"

#include <array>
#include <optional>

namespace lhp::tracking::solver {

struct BaseState {
    bool seen = false;
    std::optional<lighthouse::BaseCalibration> cal;
    bool calFromCache = false;     // provisional calibration remembered from a previous run
    bool isReference = false;
    bool hasPose = false;          // reference: always; others once bootstrapped
    math::Pose refFromBase;        // identity for the reference
    int samples = 0;               // bootstrap samples averaged
    int rejectedSamples = 0;       // bootstrap samples rejected as outliers
    double lastSpreadMm = 0;       // disagreement of the latest bootstrap sample vs the average
};

class Scene {
public:
    void MarkSeen(int channel);
    void SetCalibration(int channel, const lighthouse::BaseCalibration& c, bool fromCache = false);
    /// Bootstrap a non-reference base from one device observation:
    /// refFromDevice (device solved against known bases) and baseFromDevice (device solved against this base alone).
    void AddBootstrapSample(int channel, const math::Pose& refFromDevice, const math::Pose& baseFromDevice);
    /// True when this base may be bootstrapped (it and the reference have their OOTX calibration).
    bool ReadyToBootstrap(int channel) const;
    void SetReferenceHeight(double h) { refHeight_ = h; }
    double ReferenceHeight() const { return refHeight_; }
    /// Move the floor so that world height `worldY` becomes 0 (e.g. a device lying on the floor).
    void SetFloorAt(double worldY) { refHeight_ -= worldY; }
    void Reset();

    int ReferenceChannel() const { return ref_; }
    bool GravityAligned() const;
    const BaseState& Base(int ch) const { return bases_[ch & 15]; }
    bool HasPose(int ch) const { return bases_[ch & 15].hasPose; }
    math::Pose WorldFromRef() const;
    math::Pose WorldFromBase(int ch) const { return WorldFromRef() * bases_[ch & 15].refFromBase; }

private:
    std::array<BaseState, lighthouse::kMaxChannels> bases_{};
    int ref_ = -1;
    double refHeight_ = 2.0;
};

}  // namespace lhp::tracking::solver

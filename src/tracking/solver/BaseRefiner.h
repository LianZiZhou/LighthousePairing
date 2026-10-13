#pragma once
// L3: refines a secondary base station's pose from many device positions (SteamVR keeps learning its universe
// from tracked devices; libsurvive runs a Kalman filter per lighthouse fed by object poses).
//
// Bootstrapping averages single-base solves taken where the devices happened to rest. Those share one error
// (a single base constrains depth and rotation poorly), so the average stays wrong away from that spot: a
// device lifted a metre higher then sees this base 0.2-0.8 deg off and the tracking filter is torn between
// the bases. Here each keyframe is a device pose known from the reference base plus this base's angles at the
// same moment; keyframes are kept spread out in space and the base pose is the least-squares fit over all of
// them (the keyframes' sensor positions form one rigid "device" whose pose relative to the base is solved).
#include "tracking/lighthouse/BaseStation.h"
#include "tracking/math/Linalg.h"

#include <string>
#include <vector>

namespace lhp::tracking::solver {

struct RefineObs {
    math::Vec3 pointRef;         // sensor position, reference base frame
    int plane = 0;
    double angle = 0;
};

struct RefineKeyframe {
    math::Pose refFromDevice;
    std::vector<RefineObs> obs;
};

struct RefineResult {
    bool ok = false;
    std::string note;
    math::Pose refFromBase;
    int keyframes = 0, measurements = 0, used = 0;
    double rmsBeforeRad = 0, rmsAfterRad = 0;
    double movedMm = 0, rotatedDeg = 0;
};

class BaseRefiner {
public:
    static constexpr int kMaxKeyframes = 80;
    static constexpr int kMinKeyframes = 12;
    static constexpr double kMinSpacingM = 0.15;   // keyframes closer than this (and similar orientation) are redundant
    static constexpr double kMinSpacingDeg = 20;
    static constexpr double kMinExtentM = 0.6;     // spatial extent needed before the result is trusted

    /// True if a device at this pose would add a new viewpoint.
    bool Wants(const math::Pose& refFromDevice) const;
    void Add(RefineKeyframe k);
    void Clear() { keyframes_.clear(); added_ = 0; }
    int Count() const { return (int)keyframes_.size(); }
    /// Keyframes added since the last Solve() (to decide whether solving again is worth it).
    int Added() const { return added_; }
    void ResetAdded() { added_ = 0; }
    /// Diagonal of the keyframe positions' bounding box (metres).
    double ExtentM() const;

    /// Least-squares base pose (reference frame) from all keyframes, starting at `seed`.
    RefineResult Solve(const lighthouse::AxisCal cal[2], const math::Pose& seedRefFromBase);

private:
    std::vector<RefineKeyframe> keyframes_;
    int added_ = 0;
};

}  // namespace lhp::tracking::solver

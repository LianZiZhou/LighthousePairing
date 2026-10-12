#pragma once
// L3: the shared world ("universe" in SteamVR terms): base station calibrations and poses.
//
// World frame: +Y up, the reference base looks along world -Z and sits at (0, height, 0) above the floor
// plane y = 0. Other bases are stored relative to the reference base (refFromBase), so re-levelling the world
// or moving the floor never invalidates them.
//
// Mirrors what SteamVR's lighthouse driver logs and stores (lighthousedb.json):
//  * one base is the origin of the universe, the others are "SECONDARY base ... distance" learned from devices
//    that see both ("Base X is learning to get to global from Y because of direct observation");
//  * the universe tilt (pitch / roll) starts from a base accelerometer and is refined ("Updating tilt for");
//    here the refinement uses the accelerometers of still, tracked devices;
//  * universes are remembered by base station serial (= OOTX id) and re-selected when the same bases are seen
//    ("Selected existing universe ... (%08X is ...)"). SteamVR's own lighthousedb universes can be imported.
//
// Not thread-safe; the owner (TrackingService) serialises access.
#include "tracking/lighthouse/BaseStation.h"
#include "tracking/math/Linalg.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace lhp::tracking::solver {

/// A remembered set of base poses, keyed by base station id (OOTX id = SteamVR base_serial_number).
struct UniverseBase {
    uint32_t id = 0;
    math::Pose refFromBase;      // relative to the universe's reference base
    int samples = 0;             // how much evidence produced it (0 = external / unknown)
    // External formats whose pose direction is not documented (SteamVR target_pose) also carry the other
    // reading; live bootstrap samples decide which one matches before the pose is trusted.
    std::optional<math::Pose> alternative;
};

struct UniverseRecord {
    std::string source;          // "steamvr" / "saved"
    std::string id;              // SteamVR universe id (informational)
    uint32_t referenceId = 0;
    std::vector<UniverseBase> bases;   // the reference itself may be listed with an identity pose
    bool hasUp = false;
    math::Vec3 upInRef;          // refined gravity "up", reference base frame
    int upSamples = 0;
    bool fixed = false;          // do not refine (SteamVR's solution is taken as is)
};

enum class PoseSource : uint8_t { None, Reference, Bootstrap, SavedUniverse, SteamVrUniverse };
const char* ToString(PoseSource s);

struct BaseState {
    bool seen = false;
    std::optional<lighthouse::BaseCalibration> cal;
    bool calFromCache = false;     // provisional calibration remembered from a previous run
    bool isReference = false;
    bool hasPose = false;          // reference: always; others once bootstrapped / restored
    PoseSource source = PoseSource::None;
    bool fixed = false;            // pose from a fixed universe: no further bootstrap samples
    math::Pose refFromBase;        // identity for the reference
    std::optional<math::Pose> altRefFromBase;   // unverified external pose: the other reading
    int votesPrimary = 0, votesAlt = 0, votesNone = 0;
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
    /// True when this base may (still) learn its pose from devices.
    bool ReadyToBootstrap(int channel) const;
    /// Gravity direction measured by a still, tracked device, expressed in the reference base frame.
    void AddUpSample(const math::Vec3& upInRef);
    void SetUseDeviceTilt(bool on);
    bool UseDeviceTilt() const { return useDeviceTilt_; }
    void SetReferenceHeight(double h);
    double ReferenceHeight() const { return refHeight_; }
    /// Move the floor so that world height `worldY` becomes 0 (e.g. a device lying on the floor).
    void SetFloorAt(double worldY);
    /// Universes to restore from, in priority order (applied as soon as base ids are known).
    void SetKnownUniverses(std::vector<UniverseRecord> u);
    /// The current universe, for saving (empty bases when nothing worth saving).
    UniverseRecord ExportUniverse() const;
    void Reset();

    int ReferenceChannel() const { return ref_; }
    bool GravityAligned() const;
    const BaseState& Base(int ch) const { return bases_[ch & 15]; }
    /// Pose good enough to track against: a freshly bootstrapped base needs a few averaged samples first
    /// (a single noisy sample would make every device's filter reject the light of the other bases).
    bool HasPose(int ch) const {
        const BaseState& b = bases_[ch & 15];
        return b.hasPose && (b.source != PoseSource::Bootstrap || b.samples >= kMinBootstrapSamples);
    }
    static constexpr int kMinBootstrapSamples = 10;
    math::Pose WorldFromRef() const;
    math::Pose WorldFromBase(int ch) const { return WorldFromRef() * bases_[ch & 15].refFromBase; }
    /// Incremented whenever WorldFromRef() changes (tracking filters re-express their state).
    uint64_t Version() const { return version_; }
    // tilt diagnostics
    math::Vec3 BaseAccelUp() const;     // reference base accelerometer, ref frame (zero if unknown)
    math::Vec3 AppliedUp() const { return appliedUp_; }
    int UpSamples() const { return upCount_; }
    double TiltCorrectionDeg() const;   // applied up vs base accelerometer
    const std::string& UniverseNote() const { return universeNote_; }

private:
    void TryApplyUniverses();
    void UpdateUp();

    std::array<BaseState, lighthouse::kMaxChannels> bases_{};
    int ref_ = -1;
    double refHeight_ = 2.0;
    uint64_t version_ = 1;
    std::vector<UniverseRecord> universes_;
    std::string universeNote_;
    // tilt
    bool useDeviceTilt_ = true;
    math::Vec3 upSum_;
    int upCount_ = 0;
    math::Vec3 appliedUp_;          // ref frame; zero = not aligned
};

}  // namespace lhp::tracking::solver

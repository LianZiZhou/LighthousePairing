#pragma once
// L2: Lighthouse 2.0 base station calibration ("fcal") as broadcast over OOTX, and its reprojection model.
// OOTX layout (v15) and the reprojection formula are ported from libsurvive (MIT):
//   src/ootx_decoder.c (init_lighthouse_info_v15), src/survive_reproject_gen2.c.
#include "tracking/math/Linalg.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace lhp::tracking::lighthouse {

constexpr int kMaxChannels = 16;

/// Rotor frequency per LH2 channel (Hz). Channel = base station "mode" - 1.
double ChannelFrequency(int channel);

/// Per-plane calibration of one rotor sweep plane.
struct AxisCal {
    double phase = 0, tilt = 0, curve = 0, gibPhase = 0, gibMag = 0, ogeePhase = 0, ogeeMag = 0;
};

struct BaseCalibration {
    uint32_t id = 0;               // unique base station id (as in SteamVR's lighthousedb, hex)
    uint16_t firmware = 0;         // bits 15..6 firmware, 5..0 protocol
    uint8_t mode = 0;              // channel (low 7 bits)
    uint8_t sysUnlockCount = 0;
    uint8_t ootxModel = 0;
    uint8_t sysFaults = 0;
    uint16_t nonce = 0;
    int8_t accelDir[3]{};          // base station "up" as measured by its accelerometer (base frame)
    AxisCal axis[2];
    std::vector<uint8_t> raw;      // OOTX payload as received
};

/// Parses an OOTX v15 payload. Empty optional if too short.
std::optional<BaseCalibration> ParseOotxV15(const uint8_t* data, size_t length);

/// Predicted LH2 sweep angle (radians, axis-centred like Lh2Observation::angle) for a point given in the
/// base station frame (base looks down its -Z axis). axis 0 = first plane, 1 = second.
double ReprojectAxis(const AxisCal& cal, const math::Vec3& ptInBase, int axis);

/// Ideal (uncalibrated) model, used for bootstrap before OOTX arrives.
inline double ReprojectAxisIdeal(const math::Vec3& p, int axis) { return ReprojectAxis(AxisCal{}, p, axis); }

}  // namespace lhp::tracking::lighthouse

#pragma once
// L1: every report this program can SEND to a device lives here, nowhere else.
// Only used in active mode (user opted in and SteamVR is not running). None of these is persistent:
// they select what the device streams until it is power-cycled or another host reconfigures it.
//
// Byte layouts from libsurvive (MIT) src/driver_vive.c (vive_switch_mode).
#include "tracking/protocol/Events.h"

#include <array>
#include <cstdint>

namespace lhp::tracking::protocol::commands {

constexpr uint8_t kReportCommand = 0xFF;
constexpr uint8_t kCommandChangeProtocol = 0x87;

/// Wireless devices behind a receiver (28DE:2101 / 2102): select the light data encoding.
/// raw0 = pulses (LH1 style), raw1 = LH2 decoded (channel + sync/sweep timecodes).
inline std::array<uint8_t, 64> RfSetLightMode(LightMode mode) {
    std::array<uint8_t, 64> b{};
    b[0] = kReportCommand;
    b[1] = kCommandChangeProtocol;
    b[2] = 6;
    b[3] = 1;
    b[4] = mode == LightMode::Raw0 ? 0 : 1;
    b[5] = 0;
    b[6] = 2;
    b[7] = 0;
    b[8] = 0;
    return b;
}

}  // namespace lhp::tracking::protocol::commands

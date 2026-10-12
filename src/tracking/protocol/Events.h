#pragma once
// L1 output: typed events decoded from raw HID reports. No interpretation beyond the wire format.
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace lhp::tracking::protocol {

/// Device clock: 48 MHz watchman ticks, unwrapped to 64 bit by the decoder.
using Ticks = uint64_t;
constexpr double kTicksPerSecond = 48'000'000.0;

/// Raw IMU sample (sensor units; scale depends on the IMU range modes reported by the device).
struct ImuSample {
    Ticks time = 0;
    int16_t accel[3]{};
    int16_t gyro[3]{};
};

/// "raw0" light mode: one envelope pulse on one sensor (LH1 sync / sweep, or an LH2 pulse without decoding).
struct LightPulse {
    Ticks time = 0;              // rising edge
    uint32_t length = 0;         // ticks
    uint8_t sensor = 0;          // as reported (before channelMap)
};

/// "raw1" light mode (LH2): the device decoded the base station's LFSR and reports a rotation start...
struct Lh2Sync {
    Ticks time = 0;
    uint8_t channel = 0;         // base station channel (mode) 0..15
    bool ootx = false;           // 1 OOTX bit per rotation (base station calibration stream)
    bool gen = false;            // "g" bit
};

/// ...and the moments a sensor was hit by one of the two slanted planes.
struct Lh2Sweep {
    Ticks time = 0;
    uint8_t channel = 0;
    uint8_t sensor = 0;          // as reported (before channelMap)
    bool halfClock = false;      // +0.5 tick (96 MHz resolution)
};

struct ChannelConflict {
    uint8_t channel = 0;         // two base stations on the same channel
};

struct Battery {
    uint8_t percent = 0;
    bool charging = false;
};

struct Buttons {
    uint8_t pressed = 0;
    bool hasTrigger = false;
    float trigger = 0;
};

struct DecodeError {
    std::string what;
    std::vector<uint8_t> bytes;
};

using Event = std::variant<ImuSample, LightPulse, Lh2Sync, Lh2Sweep, ChannelConflict, Battery, Buttons, DecodeError>;

enum class LightMode : uint8_t {
    Unknown,
    Raw0,   // pulses (sensor, start, length)
    Raw1,   // LH2 decoded: channel + sync / sweep timecodes
};

const char* ToString(LightMode m);

}  // namespace lhp::tracking::protocol

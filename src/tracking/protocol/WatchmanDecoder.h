#pragma once
// L1: decodes the wireless "watchman" reports a receiver (28DE:2101 / 2102) forwards for its paired device.
//
// Report 0x23 carries one packet, 0x24 two (29 bytes each):
//   [time MSB][size][time LSB][payload: size-1 bytes]
// Payload (watchman v2): flags byte 0bIMPL xxxx (IMU / Metadata / Input / Light), followed by those sections.
// Light data comes in one of two encodings selected by the host ("raw0" pulses or "raw1" LH2-decoded).
//
// Format knowledge ported from libsurvive (MIT, https://github.com/collabora/libsurvive, src/driver_vive.c).
#include "tracking/protocol/Events.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace lhp::tracking::protocol {

struct DecoderStats {
    uint64_t reports = 0, packets = 0;
    uint64_t imu = 0, pulses = 0, syncs = 0, sweeps = 0, battery = 0, input = 0;
    uint64_t heartbeats = 0, errors = 0, unknownReports = 0;
    uint64_t raw0Votes = 0, raw1Votes = 0;
    uint64_t modeResets = 0;   // detected encoding stopped parsing (another host switched it)
};

class WatchmanDecoder {
public:
    /// Mode forced by the host (active mode). Unknown = detect from the data (passive mode).
    void SetLightMode(LightMode m) { forcedMode_ = m; }
    LightMode CurrentLightMode() const { return forcedMode_ != LightMode::Unknown ? forcedMode_ : detectedMode_; }

    /// Decode one HID input report (bytes[0] = report id). Appends to `out`.
    void Decode(const uint8_t* report, size_t length, std::vector<Event>& out);

    const DecoderStats& Stats() const { return stats_; }

private:
    void DecodePacket(const uint8_t* p, std::vector<Event>& out);
    bool DecodeRaw0(uint16_t time, const uint8_t* begin, const uint8_t* end, std::vector<Event>& out, bool emit);
    int DecodeRaw1(const uint8_t* begin, const uint8_t* end, std::vector<Event>& out, bool emit);
    uint32_t Fix24(uint32_t time24) const;
    Ticks Unwrap(uint32_t t32);

    LightMode forcedMode_ = LightMode::Unknown;
    LightMode detectedMode_ = LightMode::Unknown;
    uint32_t lastImu32_ = 0;        // reference for 24-bit light timestamps
    bool haveImu_ = false;
    uint64_t epoch_ = 0;            // high bits for 32 -> 64 bit unwrap
    uint32_t last32_ = 0;
    bool haveTime_ = false;
    int windowPackets_ = 0, windowErrors_ = 0;
    DecoderStats stats_;
};

}  // namespace lhp::tracking::protocol

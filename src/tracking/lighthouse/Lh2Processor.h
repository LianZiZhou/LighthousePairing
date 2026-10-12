#pragma once
// L2: turns LH2 sync / sweep events of ONE device into sweep angles, and collects OOTX per channel.
// angle = (t_sweep - t_sync) / period * 2pi; planes split at pi, then centred by -2pi/3 / -4pi/3
// (libsurvive survive_process_gen2.c).
#include "tracking/lighthouse/BaseStation.h"
#include "tracking/lighthouse/Ootx.h"
#include "tracking/protocol/Events.h"

#include <array>
#include <optional>

namespace lhp::tracking::lighthouse {

struct Lh2Observation {
    protocol::Ticks time = 0;
    uint8_t channel = 0;
    uint8_t sensor = 0;          // reported id (apply the device channelMap to get the model index)
    uint8_t plane = 0;           // 0 or 1
    double angle = 0;            // radians, centred (input to ReprojectAxis comparisons)
    double rawAngle = 0;         // radians since sync, [0, 2pi)
};

struct ChannelStats {
    bool seen = false;
    protocol::Ticks lastSync = 0;
    double measuredHz = 0;       // from consecutive syncs
    double freqErrorHz = 0;
    uint64_t syncs = 0, skippedSyncs = 0, badSyncs = 0, sweeps = 0, droppedSweeps = 0, borderlineSweeps = 0;
    uint64_t sweepsPerPlane[2]{};
    OotxStats ootx;
    bool conflict = false;       // two base stations on this channel
};

class Lh2Processor {
public:
    /// Returns a calibration when an OOTX packet for that channel completed.
    std::optional<BaseCalibration> OnSync(const protocol::Lh2Sync& s);
    std::optional<Lh2Observation> OnSweep(const protocol::Lh2Sweep& w);
    void OnConflict(uint8_t channel) { channels_[channel & 15].conflict = true; }
    const std::array<ChannelStats, kMaxChannels>& Channels() const { return channels_; }
    /// A channel counts as a real base station once its syncs arrive at the channel's rotor frequency.
    bool Valid(int ch) const { return channels_[ch & 15].syncs >= 3 && channels_[ch & 15].measuredHz > 0; }

private:
    std::array<ChannelStats, kMaxChannels> channels_{};
    std::array<OotxDecoder, kMaxChannels> ootx_{};
    std::array<double, kMaxChannels> period_{};   // ticks per rotation, measured
};

}  // namespace lhp::tracking::lighthouse

#include "tracking/lighthouse/Lh2Processor.h"

#include <cmath>

namespace lhp::tracking::lighthouse {

using protocol::kTicksPerSecond;

std::optional<BaseCalibration> Lh2Processor::OnSync(const protocol::Lh2Sync& s) {
    const int ch = s.channel & 15;
    ChannelStats& c = channels_[ch];
    const double predicted = kTicksPerSecond / ChannelFrequency(ch);
    std::optional<BaseCalibration> cal;
    if (c.seen && s.time > c.lastSync) {
        double delta = (double)(s.time - c.lastSync);
        int rotations = (int)((delta + predicted / 2) / predicted);
        int skipped = rotations - 1;
        if (rotations < 1) {   // duplicate / glitch
            ++c.badSyncs;
            return std::nullopt;
        }
        double per = delta / rotations;
        double hz = kTicksPerSecond / per;
        double err = std::fabs(hz - ChannelFrequency(ch));
        if (err > 1.0) {
            ++c.badSyncs;
            if (skipped > 10) c.seen = false;   // lost: restart
            else {
                c.lastSync = s.time;
            }
            return std::nullopt;
        }
        c.measuredHz = hz;
        c.freqErrorHz = hz - ChannelFrequency(ch);
        period_[ch] = per;
        c.skippedSyncs += (uint64_t)skipped;
        for (int i = 0; i < skipped; ++i) ootx_[ch].Push(-1);
    }
    c.seen = true;
    c.lastSync = s.time;
    ++c.syncs;
    if (auto payload = ootx_[ch].Push(s.ootx ? 1 : 0)) cal = ParseOotxV15(payload->data(), payload->size());
    c.ootx = ootx_[ch].Stats();
    return cal;
}

std::optional<Lh2Observation> Lh2Processor::OnSweep(const protocol::Lh2Sweep& w) {
    const int ch = w.channel & 15;
    ChannelStats& c = channels_[ch];
    ++c.sweeps;
    if (!c.seen || w.time < c.lastSync) {
        ++c.droppedSweeps;
        return std::nullopt;
    }
    const double predicted = kTicksPerSecond / ChannelFrequency(ch);
    double per = period_[ch] > 0 ? period_[ch] : predicted;
    double since = (double)(w.time - c.lastSync) + (w.halfClock ? 0.5 : 0.0);
    int rotations = (int)(since / predicted);
    if (rotations > 10) {
        ++c.droppedSweeps;
        return std::nullopt;
    }
    since -= per * rotations;
    double angle = since / per * 2 * math::kPi;
    // Plane split at pi; drop sweeps right on the border (ambiguous plane).
    constexpr double m = 0.02;
    if (angle > math::kPi * (1 - m) && angle < math::kPi * (1 + m)) {
        ++c.borderlineSweeps;
        return std::nullopt;
    }
    Lh2Observation o;
    o.time = w.time;
    o.channel = (uint8_t)ch;
    o.sensor = w.sensor;
    o.plane = angle > math::kPi ? 1 : 0;
    o.rawAngle = angle;
    o.angle = angle - (o.plane ? 4.0 : 2.0) / 3.0 * math::kPi;
    ++c.sweepsPerPlane[o.plane];
    return o;
}

}  // namespace lhp::tracking::lighthouse

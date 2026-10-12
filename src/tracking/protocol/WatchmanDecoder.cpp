// Wire format ported from libsurvive (MIT) src/driver_vive.c:
//   survive_handle_watchman / handle_watchman_v2 / read_imu_data / read_light_data / parse_and_process_raw1_lightcap.
#include "tracking/protocol/WatchmanDecoder.h"

#include <algorithm>
#include <cstring>

namespace lhp::tracking::protocol {

const char* ToString(LightMode m) {
    switch (m) {
        case LightMode::Raw0: return "raw0 (pulses)";
        case LightMode::Raw1: return "raw1 (LH2 decoded)";
        default: return "unknown";
    }
}

namespace {

constexpr uint8_t kReportWatchman = 0x23;
constexpr uint8_t kReportWatchmanX2 = 0x24;
constexpr uint8_t kReportTurnOff = 0x26;
constexpr size_t kPacketSize = 29;

inline int16_t I16(const uint8_t* p) { return (int16_t)(p[0] | (p[1] << 8)); }

DecodeError Error(const char* what, const uint8_t* b, const uint8_t* e) {
    DecodeError d;
    d.what = what;
    if (b && e > b) d.bytes.assign(b, e);
    return d;
}

}  // namespace

Ticks WatchmanDecoder::Unwrap(uint32_t t32) {
    if (!haveTime_) {
        haveTime_ = true;
        last32_ = t32;
        return epoch_ | t32;
    }
    // Forward wrap of the 32-bit clock (~89 s) vs. a slightly older timestamp arriving late.
    int32_t diff = (int32_t)(t32 - last32_);
    if (diff > 0) {
        if (t32 < last32_) epoch_ += 0x100000000ull;
        last32_ = t32;
        return epoch_ | t32;
    }
    // Older than the newest seen: belongs to the current epoch unless it straddles the wrap.
    uint64_t e = (t32 > last32_) ? epoch_ - 0x100000000ull : epoch_;
    return e | t32;
}

uint32_t WatchmanDecoder::Fix24(uint32_t time24) const {
    uint32_t upper = lastImu32_ & 0xFF000000u;
    uint32_t lower = lastImu32_ & 0x00FFFFFFu;
    if (lower > time24 && lower - time24 > (1u << 23)) upper += 0x01000000u;
    else if (lower < time24 && time24 - lower > (1u << 23) && upper > 0) upper -= 0x01000000u;
    return upper | time24;
}

void WatchmanDecoder::Decode(const uint8_t* r, size_t n, std::vector<Event>& out) {
    if (n == 0) return;
    ++stats_.reports;
    uint8_t id = r[0];
    if (id == kReportWatchman && n >= 1 + kPacketSize) {
        DecodePacket(r + 1, out);
    } else if (id == kReportWatchmanX2 && n >= 1 + 2 * kPacketSize) {
        DecodePacket(r + 1, out);
        DecodePacket(r + 1 + kPacketSize, out);
    } else if (id == kReportTurnOff) {
        // device is switching off; nothing to decode
    } else {
        ++stats_.unknownReports;
    }
}

void WatchmanDecoder::DecodePacket(const uint8_t* p, std::vector<Event>& out) {
    ++stats_.packets;
    uint16_t time = (uint16_t)((p[0] << 8) | p[2]);
    int size = (int)p[1] - 1;
    if (size <= 0) return;
    if (size > (int)kPacketSize - 3) {
        ++stats_.errors;
        out.push_back(Error("packet size exceeds report", p, p + kPacketSize));
        return;
    }
    const uint8_t* q = p + 3;
    const uint8_t* end = q + size;
    const uint8_t* start = q;

    uint8_t flags = *q++;
    if (flags == 0xE2) {   // startup heartbeat
        ++stats_.heartbeats;
        return;
    }
    const bool hasImu = flags & 0x80, hasMeta = flags & 0x40, hasInput = flags & 0x20, hasLight = flags & 0x10;

    if (hasImu) {
        if (end - q < 13) {
            ++stats_.errors;
            out.push_back(Error("short IMU section", start, end));
            return;
        }
        uint8_t lsb = *q++;
        ImuSample s;
        uint32_t t32 = ((uint32_t)time << 16) | ((uint32_t)lsb << 8);
        for (int i = 0; i < 3; ++i) s.accel[i] = I16(q + 2 * i);
        for (int i = 0; i < 3; ++i) s.gyro[i] = I16(q + 6 + 2 * i);
        q += 12;
        lastImu32_ = t32;
        haveImu_ = true;
        s.time = Unwrap(t32);
        out.push_back(s);
        ++stats_.imu;
    }
    if (hasMeta) {
        if (q >= end) return;
        uint8_t marker = *q++;
        switch (marker) {
            case 0x80: {   // battery
                if (q >= end) return;
                uint8_t b = *q++;
                if ((b & 0x7F) || (b & 0x80)) {
                    out.push_back(Battery{(uint8_t)(b & 0x7F), (b & 0x80) != 0});
                    ++stats_.battery;
                }
                break;
            }
            case 0x20:   // mode?
            case 0x40:   // seen periodically with one byte (0x01 / 0x0b / 0x0c); meaning unknown
                if (q < end) ++q;
                break;
            case 0x30:
                q = std::min(end, q + 2);
                break;
            default:
                ++stats_.errors;
                out.push_back(Error("unknown metadata marker", start, end));
                return;   // unknown length: anything after is unreliable
        }
    }
    if (hasInput) {
        if (q >= end) return;
        uint8_t inFlags = *q++;
        Buttons b;
        if (inFlags & 0x7) {
            if (inFlags & 0x1) {
                if (q >= end) return;
                b.pressed = *q++;
            }
            if (inFlags & 0x4) {
                if (q >= end) return;
                b.hasTrigger = true;
                b.trigger = *q++ / 255.f;
            }
            if (inFlags & 0x2) q = std::min(end, q + 4);   // touchpad
        } else {
            if (q >= end) return;
            uint8_t gen2 = *q++;
            if (gen2 != 0xA1 || end - q < 7) {
                ++stats_.errors;
                out.push_back(Error("unknown input event", start, end));
                return;
            }
            q += 7;   // knuckles touch / proximity / force
        }
        out.push_back(b);
        ++stats_.input;
    }
    if (!hasLight || q >= end) return;

    LightMode mode = CurrentLightMode();
    if (mode == LightMode::Unknown) {
        // Passive mode: the host that configured the device picked the encoding. Vote on which one
        // parses cleanly; after enough evidence the winner is used exclusively.
        std::vector<Event> scratch;
        bool ok0 = DecodeRaw0(time, q, end, scratch, false);
        bool ok1 = DecodeRaw1(q, end, scratch, false) >= 0;
        if (ok0 && !ok1) ++stats_.raw0Votes;
        if (ok1 && !ok0) ++stats_.raw1Votes;
        if (stats_.raw0Votes >= 20 && stats_.raw0Votes > 4 * stats_.raw1Votes) detectedMode_ = LightMode::Raw0;
        else if (stats_.raw1Votes >= 20 && stats_.raw1Votes > 4 * stats_.raw0Votes) detectedMode_ = LightMode::Raw1;
        mode = CurrentLightMode();
        if (mode == LightMode::Unknown) return;
    }
    bool ok;
    if (mode == LightMode::Raw0) {
        ok = DecodeRaw0(time, q, end, out, true);
    } else {
        std::vector<Event> scratch;
        ok = DecodeRaw1(q, end, scratch, false) >= 0;   // validate first: never emit half a bad packet
        if (ok) DecodeRaw1(q, end, out, true);
    }
    if (!ok) {
        ++stats_.errors;
        out.push_back(Error(mode == LightMode::Raw0 ? "raw0 light data" : "raw1 light data", start, end));
    }
    // Another host (SteamVR, lighthouse_console) may switch the encoding at any time: if the detected
    // mode keeps failing, forget it and vote again.
    if (forcedMode_ == LightMode::Unknown) {
        ++windowPackets_;
        if (!ok) ++windowErrors_;
        if (windowPackets_ >= 50) {
            if (windowErrors_ > 15) {
                detectedMode_ = LightMode::Unknown;
                stats_.raw0Votes = stats_.raw1Votes = 0;
                ++stats_.modeResets;
            }
            windowPackets_ = windowErrors_ = 0;
        }
    }
}

// Light data, raw0: [sensor bytes ...][time deltas, varint, read backwards ...][3 bytes LSB of last edge time]
bool WatchmanDecoder::DecodeRaw0(uint16_t time, const uint8_t* begin, const uint8_t* end, std::vector<Event>& out, bool emit) {
    if (end - begin <= 3) return true;
    if ((*begin & 0xE0) == 0xE0) return false;   // non-light data

    const uint8_t* ids = begin;
    const uint8_t* ev = end - 4;
    uint32_t lastEventTime = ((uint32_t)(time >> 8) << 24) | ((uint32_t)ev[3] << 16) | ((uint32_t)ev[2] << 8) | ev[1];
    uint32_t ref = lastImu32_;
    if (lastEventTime > ref && lastEventTime - ref > (1u << 23)) lastEventTime -= (1u << 24);
    else if (ref > lastEventTime && ref - lastEventTime > (1u << 23)) lastEventTime += (1u << 24);

    constexpr size_t kMaxTimes = 16;
    uint32_t times[kMaxTimes] = {};
    size_t timeIndex = 0;
    times[0] = lastEventTime;

    while (ids + (timeIndex >> 1) < ev) {
        uint32_t delta = 0;
        const uint8_t* evStart = ev;
        bool complete = false;
        for (;;) {
            delta <<= 7;
            delta |= (*ev & 0x7F);
            bool stop = (*ev & 0x80) != 0;
            --ev;
            if (stop) {
                complete = true;
                break;
            }
            if (ids + (timeIndex >> 1) > ev) {
                ev = evStart;
                break;
            }
        }
        if (!complete) break;
        lastEventTime -= delta;
        if (timeIndex + 1 >= kMaxTimes) return false;
        times[++timeIndex] = lastEventTime;
    }
    if (timeIndex % 2 == 0 && timeIndex > 0) {
        --timeIndex;
        do {
            ++ev;
        } while ((*ev & 0x80) == 0);
    }
    if (timeIndex % 2 != 1) return false;

    size_t sensorBytes = (size_t)(ev - ids + 1);
    if (sensorBytes > (timeIndex >> 1) + 1) ids = ev - (timeIndex >> 1);   // gen2 packs extra bytes up front

    struct Sd {
        uint8_t sensor, edges;
    };
    const size_t eventCount = (timeIndex + 1) >> 1;
    if (eventCount > kMaxTimes / 2) return false;
    Sd sensors[kMaxTimes / 2];
    for (size_t i = 0; i < eventCount; ++i, ++ids) sensors[i] = {(uint8_t)((*ids >> 3) & 0x1F), (uint8_t)(*ids & 0x7)};

    LightPulse les[kMaxTimes / 2]{};
    uint8_t order[kMaxTimes] = {};
    int ti = -1;
    for (size_t i = 0; i < eventCount; ++i) {
        do {
            if (++ti >= (int)kMaxTimes) return false;
        } while (times[ti] == 0);
        size_t startIdx = (size_t)ti + sensors[i].edges + 1;
        if (startIdx >= kMaxTimes || order[startIdx] != 0) return false;
        order[startIdx] = (uint8_t)(i + 1);
        les[i].sensor = sensors[i].sensor;
        les[i].time = times[startIdx];          // 32-bit for now
        les[i].length = times[ti] - times[startIdx];
        times[startIdx] = 0;
    }
    if (!emit) return true;
    for (size_t i = 0; i < kMaxTimes; ++i) {
        if (!order[i]) continue;
        LightPulse le = les[order[i] - 1];
        if (le.length == 0 && le.time == 0) return false;
        le.time = Unwrap((uint32_t)le.time);
        out.push_back(le);
        ++stats_.pulses;
    }
    return true;
}

// Light data, raw1 (LH2): a stream of
//   channel byte  0bCCCC ?F?1   (bit0 = 1; F = two base stations share the channel)
//   sync   u32    0bXXXX GOTT TTTT ... TTSC with S=0 : 24-bit time @48 MHz, O = OOTX bit, G = gen bit
//   sweep  u32    0bSSSS STTT ... TFSC with S=1      : 5-bit sensor, 24-bit time, F = half-clock
int WatchmanDecoder::DecodeRaw1(const uint8_t* begin, const uint8_t* end, std::vector<Event>& out, bool emit) {
    int channel = -1;
    const uint8_t* q = begin;
    while (q < end) {
        uint8_t b = *q;
        if (b & 0x1) {
            if (b & 0x0A) return -1;   // unknown bits
            channel = b >> 4;
            if ((b & 0x04) && emit) out.push_back(ChannelConflict{(uint8_t)channel});
            ++q;
            continue;
        }
        if (end - q < 4 || channel < 0) return -1;
        uint32_t tc;
        std::memcpy(&tc, q, 4);
        q += 4;
        if (!(tc & 0x2)) {
            Lh2Sync s;
            s.channel = (uint8_t)channel;
            s.ootx = (tc >> 26) & 1;
            s.gen = (tc >> 27) & 1;
            if (emit) {
                s.time = Unwrap(Fix24((tc >> 2) & 0xFFFFFF));
                out.push_back(s);
                ++stats_.syncs;
            }
        } else {
            Lh2Sweep w;
            w.channel = (uint8_t)channel;
            w.halfClock = (tc & 0x4) != 0;
            w.sensor = (uint8_t)(tc >> 27);
            if (emit) {
                w.time = Unwrap(Fix24((tc >> 3) & 0xFFFFFF));
                out.push_back(w);
                ++stats_.sweeps;
            }
        }
    }
    return (int)(q - begin);
}

}  // namespace lhp::tracking::protocol

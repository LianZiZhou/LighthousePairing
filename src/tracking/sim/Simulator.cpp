#include "tracking/sim/Simulator.h"

#include "tracking/lighthouse/Ootx.h"
#include "tracking/protocol/Events.h"

#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <random>

namespace lhp::tracking::sim {

using math::Pose;
using math::Quat;
using math::Vec3;

namespace {

constexpr double kTicks = protocol::kTicksPerSecond;
constexpr double kAccPerLsb = 8.0 / 32768.0 * 9.80665;
constexpr double kGyroPerLsb = 2000.0 / 32768.0 * math::kPi / 180.0;

uint16_t FloatToHalf(float f) {
    uint32_t x;
    std::memcpy(&x, &f, 4);
    uint32_t sign = (x >> 16) & 0x8000;
    int exp = (int)((x >> 23) & 0xFF) - 127 + 15;
    uint32_t man = x & 0x7FFFFF;
    if (exp <= 0) return (uint16_t)sign;   // flush tiny values to zero
    if (exp >= 31) return (uint16_t)(sign | 0x7C00);
    return (uint16_t)(sign | (exp << 10) | (man >> 13));
}

struct Geometry {
    std::vector<Vec3> points, normals;
};

Geometry MakeGeometry(const SimDevice& d) {
    Geometry g;
    int side = d.sensorCount * 3 / 5, top = d.sensorCount - side;
    for (int i = 0; i < side; ++i) {
        double a = 2 * math::kPi * i / side;
        Vec3 n{std::cos(a), 0, std::sin(a)};
        g.points.push_back({n.x * d.radius, (i % 2 ? 0.012 : -0.004), n.z * d.radius});
        g.normals.push_back(math::Normalize(n + Vec3{0, 0.25, 0}));
    }
    for (int i = 0; i < top; ++i) {
        double a = 2 * math::kPi * (i + 0.5) / top;
        double r = d.radius * 0.6;
        g.points.push_back({std::cos(a) * r, 0.02, std::sin(a) * r});
        g.normals.push_back(math::Normalize(Vec3{std::cos(a) * 0.4, 1, std::sin(a) * 0.4}));
    }
    return g;
}

// ---------------------------------------------------------------- OOTX
void PushWordBits(std::vector<int>& bits, uint16_t w) {
    for (int b = 15; b >= 0; --b) bits.push_back((w >> b) & 1);
    bits.push_back(1);   // stuffed sync bit
}

}  // namespace

std::vector<uint8_t> EncodeOotxV15(const lighthouse::BaseCalibration& c) {
    std::vector<uint8_t> p;
    auto u16 = [&](uint16_t v) {
        p.push_back((uint8_t)v);
        p.push_back((uint8_t)(v >> 8));
    };
    auto h = [&](double v) { u16(FloatToHalf((float)v)); };
    u16(c.firmware);
    for (int i = 0; i < 4; ++i) p.push_back((uint8_t)(c.id >> (8 * i)));
    h(c.axis[0].phase), h(c.axis[1].phase);
    h(c.axis[0].tilt), h(c.axis[1].tilt);
    p.push_back(c.sysUnlockCount);
    p.push_back(c.ootxModel);
    h(c.axis[0].curve), h(c.axis[1].curve);
    for (int k = 0; k < 3; ++k) p.push_back((uint8_t)c.accelDir[k]);
    h(c.axis[0].gibPhase), h(c.axis[1].gibPhase);
    h(c.axis[0].gibMag), h(c.axis[1].gibMag);
    p.push_back(c.mode);
    p.push_back(c.sysFaults);
    h(c.axis[0].ogeePhase), h(c.axis[1].ogeePhase);
    h(c.axis[0].ogeeMag), h(c.axis[1].ogeeMag);
    u16(c.nonce);
    return p;   // 43 bytes
}

std::vector<int> EncodeOotx(const lighthouse::BaseCalibration& c) {
    std::vector<uint8_t> payload = EncodeOotxV15(c);
    std::vector<uint8_t> bytes;
    uint16_t len = (uint16_t)payload.size();
    bytes.push_back((uint8_t)len);
    bytes.push_back((uint8_t)(len >> 8));
    bytes.insert(bytes.end(), payload.begin(), payload.end());
    if (len & 1) bytes.push_back(0);
    uint32_t crc = lighthouse::Crc32(payload.data(), payload.size());
    for (int i = 0; i < 4; ++i) bytes.push_back((uint8_t)(crc >> (8 * i)));
    std::vector<int> bits(17, 0);
    bits.push_back(1);
    for (size_t i = 0; i + 1 < bytes.size(); i += 2) PushWordBits(bits, (uint16_t)((bytes[i] << 8) | bytes[i + 1]));
    // idle gap before the packet repeats
    for (int i = 0; i < 10; ++i) bits.push_back(1);
    return bits;
}

std::string DeviceConfigJson(const SimDevice& d) {
    Geometry g = MakeGeometry(d);
    auto v = [](const Vec3& p) {
        char b[96];
        std::snprintf(b, sizeof b, "[%.6f, %.6f, %.6f]", p.x, p.y, p.z);
        return std::string(b);
    };
    std::string pts, nrm, map;
    for (size_t i = 0; i < g.points.size(); ++i) {
        pts += (i ? ", " : "") + v(g.points[i]);
        nrm += (i ? ", " : "") + v(g.normals[i]);
        map += (i ? ", " : "") + std::to_string(i);
    }
    return "{\n  \"device_class\": \"generic_tracker\",\n  \"device_serial_number\": \"" + d.serial + "\",\n  \"manufacturer\": \"LighthousePairing\",\n" +
           "  \"model_number\": \"" + d.modelNumber + "\",\n  \"render_model\": \"{sim}tracker\",\n" +
           "  \"head\": {\"plus_x\": [1, 0, 0], \"plus_z\": [0, 0, 1], \"position\": [0, 0, 0]},\n" +
           "  \"imu\": {\"acc_bias\": [0, 0, 0], \"acc_scale\": [1, 1, 1], \"gyro_bias\": [0, 0, 0], \"gyro_scale\": [1, 1, 1], " +
           "\"plus_x\": [1, 0, 0], \"plus_z\": [0, 0, 1], \"position\": [0, 0, 0]},\n" + "  \"lighthouse_config\": {\n    \"channelMap\": [" + map +
           "],\n    \"modelPoints\": [" + pts + "],\n    \"modelNormals\": [" + nrm + "]\n  }\n}\n";
}

SimConfig DemoScene(const std::vector<std::string>& receivers) {
    SimConfig c;
    auto base = [](int ch, Vec3 pos, double yawDeg, uint32_t id, double tilt0, double tilt1) {
        SimBase b;
        b.channel = ch;
        // look towards the room centre, pitched down 25 degrees
        Quat yaw = Quat::FromRotVec({0, yawDeg * math::kPi / 180, 0});
        Quat pitch = Quat::FromRotVec({-25 * math::kPi / 180, 0, 0});
        b.worldFromBase = {(yaw * pitch).Normalized(), pos};
        b.cal.id = id;
        b.cal.firmware = (uint16_t)((560 << 6) | 7);
        b.cal.mode = (uint8_t)ch;
        b.cal.axis[0] = {0.0, tilt0, -0.02, 1.5, -0.004, 0.8, -0.03};
        b.cal.axis[1] = {-0.004, tilt1, 0.05, 1.9, -0.005, 1.4, -0.02};
        return b;
    };
    c.bases.push_back(base(0, {0, 2.0, 0}, 0, 0x5A1D0001, -0.045, 0.052));
    c.bases.push_back(base(2, {2.6, 2.1, -3.2}, 120, 0x5A1D0002, -0.038, 0.047));
    const char* models[] = {"LHP Sim Tracker", "LHP Sim Tracker", "LHP Sim Tracker", "LHP Sim Controller"};
    for (size_t i = 0; i < receivers.size(); ++i) {
        SimDevice d;
        d.receiverSerial = receivers[i];
        char sn[16];
        std::snprintf(sn, sizeof sn, "LHR-51D0%04X", (unsigned)(0x1000 + i * 0x111));
        d.serial = sn;
        d.modelNumber = models[i % 4];
        d.sensorCount = i % 4 == 3 ? 24 : 20;
        double phase = i * 1.7;
        Vec3 centre{0.4 * std::cos(phase), 0.2 + 0.45 * (i % 3), -1.8 + 0.4 * std::sin(phase)};
        d.trajectory = [centre, phase, i](double t) {
            // move for 6 s, hold still for 3 s (lets the second base station bootstrap)
            double tau = std::floor(t / 9.0) * 6.0 + std::min(std::fmod(t, 9.0), 6.0);
            double a = tau * (0.35 + 0.1 * i) + phase;
            Vec3 p = centre + Vec3{0.25 * std::cos(a), 0.06 * std::sin(2 * a), 0.25 * std::sin(a)};
            Quat q = Quat::FromRotVec({0.25 * std::sin(a * 0.7), a, 0.2 * std::cos(a * 0.9)});
            return Pose{q.Normalized(), p};
        };
        c.devices.push_back(d);
    }
    return c;
}

namespace {

class SimSource final : public io::ReportSource {
public:
    SimSource(const SimConfig& cfg, size_t idx, uint16_t id, bool realtime) : cfg_(cfg), dev_(cfg.devices[idx]), realtime_(realtime), rng_(cfg.seed + (uint32_t)idx * 7919) {
        info_.id = id;
        info_.vid = 0x28de;
        info_.pid = 0x2101;
        info_.serial = dev_.receiverSerial;
        info_.product = "Simulated Watchman Dongle";
        info_.inputReportLength = 59;
        geo_ = MakeGeometry(dev_);
        for (auto& b : cfg_.bases) {
            BaseRt rt;
            rt.b = b;
            // accelerometer "up" in the base frame
            Vec3 up = b.worldFromBase.rot.Conj().Rotate({0, 1, 0});
            rt.b.cal.accelDir[0] = (int8_t)std::lround(up.x * 127);
            rt.b.cal.accelDir[1] = (int8_t)std::lround(up.y * 127);
            rt.b.cal.accelDir[2] = (int8_t)std::lround(up.z * 127);
            rt.ootx = EncodeOotx(rt.b.cal);
            rt.period = kTicks / lighthouse::ChannelFrequency(b.channel);
            rt.nextSync = 0.01 * kTicks + (double)(rng_() % 1000) * 1e-3 * rt.period;
            bases_.push_back(rt);
        }
        clock0_ = 5.0 * kTicks;   // device clock starts at 5 s
        nextImu_ = 0;
    }
    const io::SourceInfo& Info() const override { return info_; }
    bool Ended() const override { return cancelled_; }
    void Cancel() override { cancelled_ = true; }
    bool Read(io::RawReport& out, int timeoutMs) override {
        if (cancelled_) return false;
        while (queue_.empty() || queue_.front().first > NextEventTime()) Generate();
        auto& front = queue_.front();
        if (realtime_) {
            uint64_t now = io::HostMicros();
            if (!hostStart_) hostStart_ = now;
            uint64_t due = hostStart_ + (uint64_t)(front.first / kTicks * 1e6);
            if (due > now) {
                uint64_t wait = due - now;
                if (timeoutMs >= 0 && wait > (uint64_t)timeoutMs * 1000) {
                    Sleep((DWORD)timeoutMs);
                    return false;
                }
                Sleep((DWORD)(wait / 1000));
            }
        }
        out.bytes = std::move(front.second);
        out.hostUs = realtime_ ? io::HostMicros() : (uint64_t)(front.first / kTicks * 1e6);
        out.sourceId = info_.id;
        queue_.pop_front();
        return true;
    }

private:
    struct BaseRt {
        SimBase b;
        std::vector<int> ootx;
        size_t ootxPos = 0;
        double period = 0;
        double nextSync = 0;   // ticks since start
    };

    uint32_t Dev32(double t) const { return (uint32_t)(uint64_t)(clock0_ + t); }

    void Push(double t, const uint8_t* payload, size_t n, uint32_t headerTime) {
        std::vector<uint8_t> r(59, 0);
        r[0] = 0x23;
        r[1] = (uint8_t)(headerTime >> 24);
        r[2] = (uint8_t)(n + 1);
        r[3] = (uint8_t)(headerTime >> 16);
        std::memcpy(&r[4], payload, n);
        // keep the queue in time order (light packets are sent after their rotation, IMU in between)
        auto it = std::upper_bound(queue_.begin(), queue_.end(), t, [](double v, const auto& e) { return v < e.first; });
        queue_.insert(it, {t, std::move(r)});
    }

    double NextEventTime() const {
        double t = nextImu_;
        for (auto& b : bases_) t = std::min(t, b.nextSync);
        return t;
    }

    void Generate() {
        // next event: IMU or a base rotation
        double tNext = nextImu_;
        int which = -1;
        for (size_t i = 0; i < bases_.size(); ++i)
            if (bases_[i].nextSync < tNext) {
                tNext = bases_[i].nextSync;
                which = (int)i;
            }
        if (which < 0) {
            EmitImu(nextImu_);
            nextImu_ += kTicks / 250.0;
        } else {
            EmitRotation(bases_[which], tNext);
            bases_[which].nextSync += bases_[which].period;
        }
    }

    void EmitImu(double t) {
        Pose p = dev_.trajectory(t / kTicks);
        Vec3 g = p.rot.Conj().Rotate({0, 9.80665, 0});   // accelerometer at rest reads "up"
        uint32_t t32 = Dev32(t) & 0xFFFFFF00u;
        uint8_t pl[16];
        size_t n = 0;
        pl[n++] = 0x80;
        pl[n++] = (uint8_t)(t32 >> 8);
        int16_t a[3] = {(int16_t)std::lround(g.x / kAccPerLsb), (int16_t)std::lround(g.y / kAccPerLsb), (int16_t)std::lround(g.z / kAccPerLsb)};
        for (int k = 0; k < 3; ++k) {
            pl[n++] = (uint8_t)a[k];
            pl[n++] = (uint8_t)(a[k] >> 8);
        }
        // gyro: angular velocity in the device frame from the trajectory
        const double hdt = 0.002;
        Pose p2 = dev_.trajectory(t / kTicks + hdt);
        Vec3 wWorld = (p2.rot * p.rot.Conj()).ToRotVec() / hdt;
        Vec3 wDev = p.rot.Conj().Rotate(wWorld);
        int16_t gy[3] = {(int16_t)std::lround(wDev.x / kGyroPerLsb), (int16_t)std::lround(wDev.y / kGyroPerLsb), (int16_t)std::lround(wDev.z / kGyroPerLsb)};
        for (int k = 0; k < 3; ++k) {
            pl[n++] = (uint8_t)gy[k];
            pl[n++] = (uint8_t)(gy[k] >> 8);
        }
        Push(t, pl, n, t32);
        if (++imuCount_ % 2500 == 0) {   // battery every ~10 s
            uint8_t b[3] = {0x40, 0x80, 87};
            Push(t, b, 3, t32);
        }
    }

    void EmitRotation(BaseRt& rt, double tSync) {
        std::normal_distribution<double> noise(0, cfg_.angleNoiseRad);
        std::uniform_real_distribution<double> uni(0, 1);
        struct Entry {
            double t;
            uint32_t code;
        };
        std::vector<Entry> entries;
        int bit = rt.ootx[rt.ootxPos];
        rt.ootxPos = (rt.ootxPos + 1) % rt.ootx.size();
        uint32_t t24 = Dev32(tSync) & 0xFFFFFF;
        entries.push_back({tSync, (t24 << 2) | ((uint32_t)bit << 26)});
        Pose dev = dev_.trajectory(tSync / kTicks);
        Pose baseInv = rt.b.worldFromBase.Inverse();
        Vec3 basePos = rt.b.worldFromBase.pos;
        for (size_t s = 0; s < geo_.points.size(); ++s) {
            Vec3 w = dev.Apply(geo_.points[s]);
            Vec3 nW = dev.rot.Rotate(geo_.normals[s]);
            Vec3 toBase = math::Normalize(basePos - w);
            if (math::Dot(nW, toBase) < 0.15) continue;   // facing away
            Vec3 pb = baseInv.Apply(w);
            if (pb.z > -0.05) continue;                   // behind the base
            // field of view ~ +-75 deg horizontally, +-55 vertically
            if (std::fabs(std::atan2(pb.x, -pb.z)) > 75 * math::kPi / 180 || std::fabs(std::atan2(pb.y, -pb.z)) > 55 * math::kPi / 180) continue;
            for (int plane = 0; plane < 2; ++plane) {
                if (uni(rng_) < cfg_.missProbability) continue;
                double ang = lighthouse::ReprojectAxis(rt.b.cal.axis[plane], pb, plane) + noise(rng_);
                double raw = ang + (plane ? 4.0 : 2.0) / 3.0 * math::kPi;
                if (raw < 0 || raw >= 2 * math::kPi) continue;
                double ts = tSync + raw / (2 * math::kPi) * rt.period;
                uint32_t s24 = Dev32(ts) & 0xFFFFFF;
                entries.push_back({ts, ((uint32_t)s << 27) | (s24 << 3) | 0x2});
            }
        }
        std::sort(entries.begin() + 1, entries.end(), [](const Entry& a, const Entry& b) { return a.t < b.t; });
        // packets of up to 6 entries: flags 0x10, channel byte, then u32 entries; sent after the rotation
        size_t i = 0;
        while (i < entries.size()) {
            uint8_t pl[26];
            size_t n = 0;
            pl[n++] = 0x10;
            pl[n++] = (uint8_t)((rt.b.channel << 4) | 1);
            double last = entries[i].t;
            for (int k = 0; k < 6 && i < entries.size(); ++k, ++i) {
                std::memcpy(&pl[n], &entries[i].code, 4);
                n += 4;
                last = entries[i].t;
            }
            Push(std::max(last, tSync) + 0.0005 * kTicks, pl, n, Dev32(last));
        }
    }

    SimConfig cfg_;
    SimDevice dev_;
    Geometry geo_;
    io::SourceInfo info_;
    bool realtime_;
    std::mt19937 rng_;
    std::vector<BaseRt> bases_;
    double clock0_ = 0, nextImu_ = 0;
    uint64_t imuCount_ = 0;
    std::deque<std::pair<double, std::vector<uint8_t>>> queue_;   // (sim ticks, report)
    uint64_t hostStart_ = 0;
    bool cancelled_ = false;
};

}  // namespace

std::unique_ptr<io::ReportSource> OpenSimSource(const SimConfig& cfg, size_t idx, uint16_t id, bool realtime) {
    return std::make_unique<SimSource>(cfg, idx, id, realtime);
}

namespace {
class SimModels final : public model::ModelProvider {
public:
    explicit SimModels(const SimConfig& c) {
        for (auto& d : c.devices) json_.emplace_back(d.receiverSerial, DeviceConfigJson(d));
    }
    std::optional<std::string> ConfigFor(const std::string& receiver) override {
        for (auto& [r, j] : json_)
            if (r == receiver) return j;
        return std::nullopt;
    }

private:
    std::vector<std::pair<std::string, std::string>> json_;
};
}  // namespace

std::shared_ptr<model::ModelProvider> MakeModelProvider(const SimConfig& cfg) { return std::make_shared<SimModels>(cfg); }

}  // namespace lhp::tracking::sim

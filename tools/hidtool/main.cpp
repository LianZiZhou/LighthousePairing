// lhp_hidtool - inspect Lighthouse HID traffic. Read-only: never sends anything to a device.
//   lhp_hidtool list
//   lhp_hidtool capture <serial-substring|all> <seconds> <out.lhrec>
//   lhp_hidtool dump <file.lhrec> [hex-per-id]
#include "tracking/hid/HidDevice.h"
#include "tracking/io/PrivacyNotice.h"
#include "tracking/io/Recording.h"
#include "tracking/lighthouse/Lh2Processor.h"
#include "tracking/model/DeviceModel.h"
#include "tracking/solver/PoseSolver.h"

#include <optional>
#include "tracking/protocol/Commands.h"
#include "tracking/protocol/WatchmanDecoder.h"
#include "tracking/service/HostGuard.h"
#include "tracking/service/TrackingService.h"
#include "tracking/filter/Imu.h"
#include "tracking/sim/Simulator.h"
#include "tracking/solver/UniverseIo.h"

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace lhp::tracking;

static int List() {
    auto all = hid::Enumerate(0x28de);
    std::printf("%zu Valve HID interface(s)\n", all.size());
    for (auto& d : all) {
        std::printf("%04x:%04x mi=%d col=%d serial=%-16s in=%3u out=%3u feat=%3u page=%04x usage=%04x  %s\n", d.vid, d.pid, d.interfaceNumber,
                    d.collection, d.serial.c_str(), d.inputReportLength, d.outputReportLength, d.featureReportLength, d.usagePage, d.usage,
                    d.product.c_str());
    }
    return 0;
}

static int Capture(const std::string& filter, double seconds, const std::string& path) {
    std::vector<hid::HidDeviceInfo> picked;
    for (auto& d : hid::Enumerate(0x28de))
        if (filter == "all" || d.serial.find(filter) != std::string::npos) picked.push_back(d);
    if (picked.empty()) {
        std::printf("no matching device\n");
        return 1;
    }
    std::fprintf(stderr, "注意：%s\n", io::kRecordingNoticeShort);
    std::string err;
    auto writer = io::RecordingWriter::Create(path, &err);
    if (!writer) {
        std::printf("%s\n", err.c_str());
        return 1;
    }
    std::vector<std::unique_ptr<io::ReportSource>> sources;
    for (auto& d : picked) {
        auto s = io::OpenLive(d, (uint16_t)sources.size(), hid::Access::ReadOnly, &err);
        if (!s) {
            std::printf("open %s mi=%d failed: %s\n", d.serial.c_str(), d.interfaceNumber, err.c_str());
            continue;
        }
        writer->AddSource(s->Info());
        std::printf("source %u: %04x:%04x mi=%d %s\n", s->Info().id, d.vid, d.pid, d.interfaceNumber, d.serial.c_str());
        sources.push_back(std::move(s));
    }
    std::atomic<bool> stop{false};
    std::vector<std::atomic<uint64_t>> counts(sources.size());
    std::vector<std::thread> threads;
    for (size_t i = 0; i < sources.size(); ++i)
        threads.emplace_back([&, i] {
            io::RawReport r;
            while (!stop)
                if (sources[i]->Read(r, 100)) {
                    writer->Write(r);
                    ++counts[i];
                } else if (sources[i]->Ended()) {
                    break;
                }
        });
    uint64_t t0 = io::HostMicros();
    std::vector<uint64_t> last(sources.size());
    while ((io::HostMicros() - t0) / 1e6 < seconds) {
        Sleep(1000);
        std::printf("%5.1fs", (io::HostMicros() - t0) / 1e6);
        for (size_t i = 0; i < sources.size(); ++i) {
            uint64_t c = counts[i];
            std::printf("  [%zu] %llu/s", i, (unsigned long long)(c - last[i]));
            last[i] = c;
        }
        std::printf("\n");
    }
    stop = true;
    for (auto& s : sources) s->Cancel();
    for (auto& t : threads) t.join();
    std::printf("wrote %zu records to %s\n", writer->Count(), path.c_str());
    return 0;
}

static int Dump(const std::string& path, int hexPerId) {
    io::Recording rec;
    std::string err;
    if (!io::LoadRecording(path, rec, &err)) {
        std::printf("%s\n", err.c_str());
        return 1;
    }
    std::printf("%zu sources, %zu reports, %zu markers\n", rec.sources.size(), rec.reports.size(), rec.markers.size());
    for (auto& s : rec.sources) {
        struct Stat {
            uint64_t n = 0;
            std::map<size_t, uint64_t> lengths;
            int shown = 0;
        };
        std::map<uint8_t, Stat> ids;
        uint64_t first = 0, lastT = 0;
        uint64_t n = 0;
        std::printf("\n== source %u  %04x:%04x mi=%d serial=%s product=%s\n", s.id, s.vid, s.pid, s.interfaceNumber, s.serial.c_str(), s.product.c_str());
        for (auto& r : rec.reports) {
            if (r.sourceId != s.id || r.bytes.empty()) continue;
            if (!n++) first = r.hostUs;
            lastT = r.hostUs;
            auto& st = ids[r.bytes[0]];
            ++st.n;
            ++st.lengths[r.bytes.size()];
            if (st.shown < hexPerId) {
                ++st.shown;
                std::printf("  id 0x%02x +%8.3fms:", r.bytes[0], (r.hostUs - first) / 1000.0);
                for (size_t i = 1; i < r.bytes.size(); ++i) std::printf(" %02x", r.bytes[i]);
                std::printf("\n");
            }
        }
        double dur = n > 1 ? (lastT - first) / 1e6 : 0;
        std::printf("  %llu reports over %.2fs (%.0f/s)\n", (unsigned long long)n, dur, dur > 0 ? n / dur : 0.0);
        for (auto& [id, st] : ids) {
            std::printf("  id 0x%02x: %llu (%.0f/s) lengths:", id, (unsigned long long)st.n, dur > 0 ? st.n / dur : 0.0);
            for (auto& [len, c] : st.lengths) std::printf(" %zu x%llu", len, (unsigned long long)c);
            std::printf("\n");
        }
    }
    return 0;
}

static int Decode(const std::string& path, int show) {
    io::Recording rec;
    std::string err;
    if (!io::LoadRecording(path, rec, &err)) {
        std::printf("%s\n", err.c_str());
        return 1;
    }
    for (auto& s : rec.sources) {
        protocol::WatchmanDecoder dec;
        lighthouse::Lh2Processor lh2;
        std::map<int, std::pair<double, double>> angleRange;   // (channel*100+sensor*2+plane) -> min/max deg
        std::vector<protocol::Event> ev;
        std::map<int, uint64_t> lenHist;   // pulse length buckets (µs)
        std::map<int, uint64_t> sensorHits;
        int shown = 0;
        protocol::Ticks firstImu = 0, lastImu = 0;
        uint64_t imuN = 0;
        for (auto& r : rec.reports) {
            if (r.sourceId != s.id) continue;
            ev.clear();
            dec.Decode(r.bytes.data(), r.bytes.size(), ev);
            for (auto& e : ev) {
                if (auto* p = std::get_if<protocol::LightPulse>(&e)) {
                    ++lenHist[(int)(p->length / 48.0 / 10) * 10];
                    ++sensorHits[p->sensor];
                    if (shown < show) {
                        ++shown;
                        std::printf("  pulse  s%-2u t=%.6f len=%u (%.1fus)\n", p->sensor, p->time / protocol::kTicksPerSecond, p->length, p->length / 48.0);
                    }
                } else if (auto* im = std::get_if<protocol::ImuSample>(&e)) {
                    if (!imuN++) firstImu = im->time;
                    lastImu = im->time;
                    if (imuN <= 3)
                        std::printf("  imu    t=%.6f acc=%d,%d,%d gyro=%d,%d,%d\n", im->time / protocol::kTicksPerSecond, im->accel[0], im->accel[1], im->accel[2],
                                    im->gyro[0], im->gyro[1], im->gyro[2]);
                } else if (auto* sy = std::get_if<protocol::Lh2Sync>(&e)) {
                    if (auto cal = lh2.OnSync(*sy)) {
                        std::printf("  OOTX ch%u: id=%08X fw=%u mode=%u faults=%u accel=%d,%d,%d\n", sy->channel, cal->id, cal->firmware, cal->mode, cal->sysFaults,
                                    cal->accelDir[0], cal->accelDir[1], cal->accelDir[2]);
                        for (int a = 0; a < 2; ++a)
                            std::printf("        axis%d phase=%.5f tilt=%.5f curve=%.5f gib=%.5f/%.5f ogee=%.5f/%.5f\n", a, cal->axis[a].phase, cal->axis[a].tilt,
                                        cal->axis[a].curve, cal->axis[a].gibPhase, cal->axis[a].gibMag, cal->axis[a].ogeePhase, cal->axis[a].ogeeMag);
                    }
                    if (shown < show) {
                        ++shown;
                        std::printf("  sync   ch%u t=%.6f ootx=%d\n", sy->channel, sy->time / protocol::kTicksPerSecond, sy->ootx);
                    }
                } else if (auto* sw = std::get_if<protocol::Lh2Sweep>(&e)) {
                    if (auto o = lh2.OnSweep(*sw)) {
                        int key = o->channel * 100 + o->sensor * 2 + o->plane;
                        double deg = o->angle * 180 / 3.14159265358979;
                        auto it = angleRange.find(key);
                        if (it == angleRange.end()) angleRange[key] = {deg, deg};
                        else it->second = {std::min(it->second.first, deg), std::max(it->second.second, deg)};
                    }
                    if (shown < show) {
                        ++shown;
                        std::printf("  sweep  ch%u s%u t=%.6f\n", sw->channel, sw->sensor, sw->time / protocol::kTicksPerSecond);
                    }
                } else if (auto* de = std::get_if<protocol::DecodeError>(&e)) {
                    std::printf("  error  %s:", de->what.c_str());
                    for (auto b : de->bytes) std::printf(" %02x", b);
                    std::printf("\n");
                }
            }
        }
        auto& st = dec.Stats();
        std::printf("== source %u %s  mode=%s\n  reports=%llu packets=%llu imu=%llu pulses=%llu syncs=%llu sweeps=%llu battery=%llu input=%llu hb=%llu err=%llu votes raw0/raw1=%llu/%llu\n",
                    s.id, s.serial.c_str(), protocol::ToString(dec.CurrentLightMode()), (unsigned long long)st.reports, (unsigned long long)st.packets,
                    (unsigned long long)st.imu, (unsigned long long)st.pulses, (unsigned long long)st.syncs, (unsigned long long)st.sweeps,
                    (unsigned long long)st.battery, (unsigned long long)st.input, (unsigned long long)st.heartbeats, (unsigned long long)st.errors,
                    (unsigned long long)st.raw0Votes, (unsigned long long)st.raw1Votes);
        if (imuN > 1) std::printf("  imu rate (device clock): %.1f Hz\n", (imuN - 1) / ((lastImu - firstImu) / protocol::kTicksPerSecond));
        std::printf("  pulse length histogram (us):");
        for (auto& [b, c] : lenHist) std::printf(" %d:%llu", b, (unsigned long long)c);
        std::printf("\n  hits per sensor:");
        for (auto& [k, c] : sensorHits) std::printf(" %d:%llu", k, (unsigned long long)c);
        std::printf("\n");
        for (int ch = 0; ch < lighthouse::kMaxChannels; ++ch) {
            auto& c = lh2.Channels()[ch];
            if (!c.syncs) continue;
            std::printf("  ch%d: syncs=%llu skipped=%llu bad=%llu hz=%.4f (err %+.4f) sweeps=%llu dropped=%llu border=%llu planes=%llu/%llu ootx bits=%u pre=%u badsync=%u crc=%u pk=%u guess=%u\n",
                        ch, (unsigned long long)c.syncs, (unsigned long long)c.skippedSyncs, (unsigned long long)c.badSyncs, c.measuredHz, c.freqErrorHz,
                        (unsigned long long)c.sweeps, (unsigned long long)c.droppedSweeps, (unsigned long long)c.borderlineSweeps,
                        (unsigned long long)c.sweepsPerPlane[0], (unsigned long long)c.sweepsPerPlane[1], c.ootx.bits, c.ootx.preambles, c.ootx.badSyncBits,
                        c.ootx.badCrc, c.ootx.packets, c.ootx.guessedBits);
        }
        for (auto& [k, r] : angleRange)
            std::printf("    ch%d s%-2d p%d  %+8.3f .. %+8.3f deg (spread %.3f)\n", k / 100, (k % 100) / 2, k % 2, r.first, r.second, r.second - r.first);
    }
    return 0;
}

// Time series of one sensor's angles (debugging the LH2 angle pipeline).
static int Angles(const std::string& path, int sensor, int maxLines) {
    io::Recording rec;
    std::string err;
    if (!io::LoadRecording(path, rec, &err)) return 1;
    protocol::WatchmanDecoder dec;
    lighthouse::Lh2Processor lh2;
    std::vector<protocol::Event> ev;
    int lines = 0;
    protocol::Ticks lastSync = 0;
    for (auto& r : rec.reports) {
        ev.clear();
        dec.Decode(r.bytes.data(), r.bytes.size(), ev);
        for (auto& e : ev) {
            if (auto* sy = std::get_if<protocol::Lh2Sync>(&e)) {
                lh2.OnSync(*sy);
                if (lines < maxLines) std::printf("sync  t=%llu  d=%lld\n", (unsigned long long)sy->time, (long long)(sy->time - lastSync));
                lastSync = sy->time;
            } else if (auto* sw = std::get_if<protocol::Lh2Sweep>(&e)) {
                auto o = lh2.OnSweep(*sw);
                if (sw->sensor == sensor && lines++ < maxLines)
                    std::printf("sweep t=%llu since=%lld  %s\n", (unsigned long long)sw->time, (long long)(sw->time - lastSync),
                                o ? (std::to_string(o->plane) + " " + std::to_string(o->angle * 180 / 3.14159265358979)).c_str() : "dropped");
            }
        }
    }
    return 0;
}

// Offline solve of a recording with a device config (JSON).
static int Solve(const std::string& path, const std::string& configPath, int every, int source, int channel) {
    io::Recording rec;
    std::string err;
    if (!io::LoadRecording(path, rec, &err)) return 1;
    std::FILE* f = std::fopen(configPath.c_str(), "rb");
    if (!f) {
        std::printf("cannot open %s\n", configPath.c_str());
        return 1;
    }
    std::string json;
    char buf[65536];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) json.append(buf, n);
    std::fclose(f);
    auto model = model::ParseDeviceModel(json, &err);
    if (!model) {
        std::printf("model: %s\n", err.c_str());
        return 1;
    }
    std::printf("model %s %s: %zu sensors\n", model->serial.c_str(), model->modelNumber.c_str(), model->points.size());
    protocol::WatchmanDecoder dec;
    lighthouse::Lh2Processor lh2;
    std::map<int, lighthouse::BaseCalibration> cals;
    std::map<int, lighthouse::Lh2Observation> latest;   // key ch*64 + sensor*2 + plane
    std::vector<protocol::Event> ev;
    int syncCount = 0;
    std::optional<math::Pose> last;
    for (auto& r : rec.reports) {
        if (source >= 0 && r.sourceId != source) continue;
        ev.clear();
        dec.Decode(r.bytes.data(), r.bytes.size(), ev);
        for (auto& e : ev) {
            if (auto* sw = std::get_if<protocol::Lh2Sweep>(&e)) {
                if (auto o = lh2.OnSweep(*sw)) latest[o->channel * 64 + o->sensor * 2 + o->plane] = *o;
            } else if (auto* sy = std::get_if<protocol::Lh2Sync>(&e)) {
                if (auto c = lh2.OnSync(*sy)) cals[sy->channel] = *c;
                if (++syncCount % every) continue;
                std::vector<solver::Measurement> meas;
                std::vector<solver::BaseView> bases;
                std::map<int, int> baseIndex;
                for (auto& [k, o] : latest) {
                    if (sy->time - o.time > 48'000'000 / 30) continue;   // older than ~33 ms
                    if (channel >= 0 && o.channel != channel) continue;  // bases are not registered here
                    int s = model->MapSensor(o.sensor);
                    if (s < 0) continue;
                    if (!baseIndex.count(o.channel)) {
                        baseIndex[o.channel] = (int)bases.size();
                        solver::BaseView b;
                        if (cals.count(o.channel))
                            for (int a = 0; a < 2; ++a) b.cal[a] = cals[o.channel].axis[a];
                        bases.push_back(b);
                    }
                    meas.push_back({baseIndex[o.channel], s, o.plane, o.angle});
                }
                auto res = solver::SolvePose(*model, meas, bases, last ? &*last : nullptr);
                auto& p = res.worldFromDevice.pos;
                auto rv = res.worldFromDevice.rot.ToRotVec();
                std::printf("t=%.3f n=%2d %s cal=%d pos=(%+.4f %+.4f %+.4f) d=%.3fm rot=(%+.3f %+.3f %+.3f) rms=%.4fdeg max=%.4fdeg sigma=%.2fmm/%.3fdeg it=%d seeds=%d rej=%d %s\n",
                            sy->time / protocol::kTicksPerSecond, (int)meas.size(), res.ok ? "OK " : "BAD", (int)cals.size(), p.x, p.y, p.z, math::Norm(p), rv.x, rv.y, rv.z,
                            res.rmsRad * 180 / math::kPi, res.maxAbsRad * 180 / math::kPi, res.positionSigma * 1000, res.rotationSigma * 180 / math::kPi,
                            res.iterations, res.seedsTried, res.rejected, res.ok ? "" : res.failReason.c_str());
                if (res.ok) last = res.worldFromDevice;
            }
        }
    }
    return 0;
}

static std::string ReadFile(const std::string& path);

// Gyro vs optics: angular velocity from consecutive single-frame solves (one base, its own frame) against the
// gyro averaged over the same interval, mapped through the IMU mount. Ratio ~1 and small angle = scale / axes right.
static int GyroCheck(const std::string& path, const std::string& configPath, int source, int channel) {
    io::Recording rec;
    std::string err;
    if (!io::LoadRecording(path, rec, &err)) return 1;
    auto model = model::ParseDeviceModel(ReadFile(configPath), &err);
    if (!model) {
        std::printf("model: %s\n", err.c_str());
        return 1;
    }
    protocol::WatchmanDecoder dec;
    lighthouse::Lh2Processor lh2;
    std::optional<lighthouse::BaseCalibration> cal;
    std::map<int, lighthouse::Lh2Observation> latest;
    struct Gs {
        double t;
        math::Vec3 g;
    };
    std::vector<Gs> gyro;
    struct Sol {
        double t;
        math::Quat q;
    };
    std::vector<Sol> sols;
    std::optional<math::Pose> last;
    std::vector<protocol::Event> ev;
    for (auto& r : rec.reports) {
        if (r.sourceId != source) continue;
        ev.clear();
        dec.Decode(r.bytes.data(), r.bytes.size(), ev);
        for (auto& e : ev) {
            if (auto* s = std::get_if<protocol::ImuSample>(&e)) {
                auto si = filter::ImuToSi(s->accel, s->gyro, &model->imu);
                gyro.push_back({s->time / protocol::kTicksPerSecond, si.gyro});
            } else if (auto* sw = std::get_if<protocol::Lh2Sweep>(&e)) {
                if (auto o = lh2.OnSweep(*sw); o && o->channel == channel) latest[o->sensor * 2 + o->plane] = *o;
            } else if (auto* sy = std::get_if<protocol::Lh2Sync>(&e)) {
                if (auto c = lh2.OnSync(*sy); c && sy->channel == channel) cal = c;
                if (sy->channel != channel) continue;
                // one rotation of this base: solve from the angles of the previous rotation only
                std::vector<solver::Measurement> meas;
                double tm = 0;
                for (auto& [k, o] : latest) {
                    double age = ((double)sy->time - (double)o.time) / protocol::kTicksPerSecond;
                    if (age < 0 || age > 0.021) continue;
                    int idx = model->MapSensor(o.sensor);
                    if (idx < 0) continue;
                    meas.push_back({0, idx, o.plane, o.angle, 0});
                    tm += o.time / protocol::kTicksPerSecond;
                }
                if (meas.size() < 10) {
                    last.reset();
                    continue;
                }
                tm /= meas.size();
                solver::BaseView v;
                if (cal)
                    for (int a = 0; a < 2; ++a) v.cal[a] = cal->axis[a];
                solver::SolverOptions opt;
                opt.estimateVelocity = false;
                auto res = solver::SolvePose(*model, meas, {v}, last ? &*last : nullptr, opt);
                if (!res.ok || res.rmsRad > 2e-3) {
                    last.reset();
                    continue;
                }
                last = res.worldFromDevice;
                sols.push_back({tm, res.worldFromDevice.rot});
            }
        }
    }
    std::printf("%zu solves, %zu gyro samples\n", sols.size(), gyro.size());
    // compare over ~40 ms windows where the optical rotation is large enough to measure
    double sumRatio = 0, sumAng = 0;
    int n = 0;
    std::vector<double> ratios, angles;
    size_t gi = 0;
    for (size_t i = 0; i + 2 < sols.size(); ++i) {
        const auto& a = sols[i];
        const auto& b = sols[i + 2];
        double dt = b.t - a.t;
        if (dt < 0.03 || dt > 0.05) continue;
        math::Vec3 wOpt = (b.q * a.q.Conj()).ToRotVec() / dt;   // base frame
        if (math::Norm(wOpt) < 1.0) continue;                    // need real rotation (> ~57 deg/s)
        while (gi < gyro.size() && gyro[gi].t < a.t) ++gi;
        math::Vec3 sum;
        int cnt = 0;
        for (size_t k = gi; k < gyro.size() && gyro[k].t <= b.t; ++k, ++cnt) sum += gyro[k].g;
        if (cnt < 5) continue;
        math::Vec3 gImu = sum / (double)cnt;
        math::Quat mid = math::Quat::FromRotVec((b.q * a.q.Conj()).ToRotVec() * 0.5) * a.q;
        math::Vec3 wGyro = mid.Rotate(model->imu.mount.rot.Rotate(gImu));   // IMU -> device -> base
        double ratio = math::Norm(wGyro) / math::Norm(wOpt);
        double ang = std::acos(std::clamp(math::Dot(math::Normalize(wGyro), math::Normalize(wOpt)), -1.0, 1.0)) * 57.2958;
        ratios.push_back(ratio);
        angles.push_back(ang);
        if (n < 25)
            std::printf("t=%.3f |w_opt|=%.2f |w_gyro|=%.2f ratio=%.3f angle=%.1fdeg  opt(%+.2f %+.2f %+.2f) gyro(%+.2f %+.2f %+.2f) imu(%+.2f %+.2f %+.2f)\n", a.t, math::Norm(wOpt),
                        math::Norm(wGyro), ratio, ang, wOpt.x, wOpt.y, wOpt.z, wGyro.x, wGyro.y, wGyro.z, gImu.x, gImu.y, gImu.z);
        sumRatio += ratio;
        sumAng += ang;
        ++n;
    }
    if (n) {
        std::sort(ratios.begin(), ratios.end());
        std::sort(angles.begin(), angles.end());
        std::printf("windows=%d  ratio median=%.3f  angle median=%.1fdeg  (mean ratio %.3f, mean angle %.1f)\n", n, ratios[n / 2], angles[n / 2], sumRatio / n, sumAng / n);
    }
    return 0;
}

// Runs the full TrackingService (as the app does) for N seconds and prints snapshots.
static std::string ReadFile(const std::string& path) {
    std::string s;
    if (std::FILE* f = std::fopen(path.c_str(), "rb")) {
        char buf[65536];
        size_t n;
        while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) s.append(buf, n);
        std::fclose(f);
    }
    return s;
}

// configPath is either one JSON file (used for every device) or a directory of <SERIAL>.json files.
struct OneFileProvider : model::ModelProvider {
    std::string json, dir;
    std::optional<std::string> ConfigFor(const std::string& serial) override {
        if (!dir.empty()) {
            auto s = ReadFile(dir + "\\" + serial + ".json");
            if (s.empty()) return std::nullopt;
            return s;
        }
        if (json.empty()) return std::nullopt;
        return json;
    }
};

// Read-only view of the app's DataDir\base_stations.json "channels" (OOTX payloads), so replays start calibrated
// like the app does. Minimal parsing: "<ch>": { ... "ootx": "<hex>" }.
struct AppCalibrationCache : service::CalibrationCache {
    std::map<int, std::vector<uint8_t>> raw;
    explicit AppCalibrationCache(const std::string& text) {
        size_t pos = 0;
        while ((pos = text.find("\"ootx\"", pos)) != std::string::npos) {
            size_t q1 = text.find('"', text.find(':', pos) + 1), q2 = text.find('"', q1 + 1);
            size_t key = text.rfind('{', pos);
            size_t k2 = text.rfind('"', key), k1 = text.rfind('"', k2 - 1);
            int ch = std::atoi(text.substr(k1 + 1, k2 - k1 - 1).c_str());
            std::string hex = text.substr(q1 + 1, q2 - q1 - 1);
            std::vector<uint8_t> b;
            for (size_t i = 0; i + 1 < hex.size(); i += 2) b.push_back((uint8_t)std::strtoul(hex.substr(i, 2).c_str(), nullptr, 16));
            raw[ch] = b;
            pos = q2;
        }
        size_t u = text.find("\"universe\"");
        if (u != std::string::npos) {
            size_t b = text.find('{', u);
            int depth = 0;
            for (size_t i = b; b != std::string::npos && i < text.size(); ++i) {
                depth += text[i] == '{' ? 1 : text[i] == '}' ? -1 : 0;
                if (depth == 0) {
                    universe = text.substr(b, i - b + 1);
                    break;
                }
            }
        }
    }
    std::string universe;   // the saved "universe" object, if any
    std::vector<solver::UniverseRecord> LoadUniverses() override {
        if (universe.empty()) return {};
        auto u = solver::UniverseFromJson(universe);
        if (!u) return {};
        return {*u};
    }
    std::optional<lighthouse::BaseCalibration> Load(int channel) override {
        auto it = raw.find(channel);
        if (it == raw.end()) return std::nullopt;
        return lighthouse::ParseOotxV15(it->second.data(), it->second.size());
    }
    void Save(int, const lighthouse::BaseCalibration&) override {}
};

static int Live(double seconds, const std::string& configPath, bool active, const std::string& replay, const std::string& tracePath = "") {
    auto prov = std::make_shared<OneFileProvider>();
    DWORD attr = GetFileAttributesA(configPath.c_str());
    if (attr != INVALID_FILE_ATTRIBUTES && (attr & FILE_ATTRIBUTE_DIRECTORY))
        prov->dir = configPath;
    else
        prov->json = ReadFile(configPath);
    std::shared_ptr<service::CalibrationCache> cache;
    if (const char* la = std::getenv("LOCALAPPDATA")) {
        std::string text = ReadFile(std::string(la) + "\\LighthousePairing\\base_stations.json");
        if (!text.empty() && std::getenv("LHP_NO_CACHE") == nullptr) cache = std::make_shared<AppCalibrationCache>(text);
    }
    service::TrackingService svc(prov, cache);
    svc.SetOptions({active});
    std::FILE* traceFile = tracePath.empty() ? nullptr : std::fopen(tracePath.c_str(), "wb");
    std::mutex traceMtx;
    if (traceFile)
        svc.SetTrace([&](const std::string& line) {
            std::scoped_lock lk(traceMtx);
            std::fputs(line.c_str(), traceFile);
            std::fputc('\n', traceFile);
        });
    sim::SimConfig simCfg;
    if (replay == "sim" || replay == "simfast") {   // simulated demo scene (simfast: as fast as possible)
        simCfg = sim::DemoScene({"SIM-A", "SIM-B"});
        std::vector<std::unique_ptr<io::ReportSource>> src;
        for (size_t i = 0; i < simCfg.devices.size(); ++i) src.push_back(sim::OpenSimSource(simCfg, i, (uint16_t)i, replay == "sim"));
        svc.StartSources(std::move(src), "sim", "sim", sim::MakeModelProvider(simCfg), true);
    } else if (!replay.empty()) {
        std::string err;
        if (!svc.StartReplay(replay, &err)) {
            std::printf("replay: %s\n", err.c_str());
            return 1;
        }
    } else {
        svc.Start();
    }
    auto t0 = io::HostMicros();
    size_t logShown = 0;
    uint64_t lastSeq = 0, lastChange = io::HostMicros();
    double maxGapMs = 0;
    while ((io::HostMicros() - t0) / 1e6 < seconds) {
        // poll the snapshot for a second, measuring the longest gap between publications (stalls show up here)
        uint64_t sliceStart = io::HostMicros();
        maxGapMs = 0;
        while (io::HostMicros() - sliceStart < 1'000'000) {
            Sleep(2);
            auto q = svc.Snapshot();
            if (q && q->seq != lastSeq) {
                uint64_t now = io::HostMicros();
                if (lastSeq) maxGapMs = std::max(maxGapMs, (now - lastChange) / 1000.0);
                lastSeq = q->seq;
                lastChange = now;
            }
        }
        std::printf("   snapshot max gap %.0f ms\n", maxGapMs);
        auto s = svc.Snapshot();
        if (!s) continue;
        for (; logShown < s->log.size(); ++logShown) std::printf("  log: %s\n", s->log[logShown].c_str());
        std::printf("%.0fs  %s  steamvr=%d devices=%zu bases=%zu gravity=%d tilt=%d samples %.2fdeg %s\n", (io::HostMicros() - t0) / 1e6, s->activeNote.c_str(),
                    s->host.steamVrRunning, s->devices.size(), s->bases.size(), s->gravityAligned, s->tiltSamples, s->tiltCorrectionDeg, s->universeNote.c_str());
        for (auto& d : s->devices) {
            if (d.decoder.packets == 0) continue;
            auto& p = d.worldFromDevice.pos;
            if (!simCfg.devices.empty() && d.poseTime > 0) {   // ground truth, mapped into the service world via the reference base
                math::Pose svcFromSim;
                for (auto& b : s->bases)
                    if (b.isReference)
                        for (auto& sb : simCfg.bases)
                            if (sb.channel == b.channel) svcFromSim = b.worldFromBase * sb.worldFromBase.Inverse();
                size_t i = d.receiverSerial == "SIM-A" ? 0 : 1;
                math::Pose t = svcFromSim * simCfg.devices[i].trajectory(d.poseTime - sim::kDeviceClockStart);
                std::printf("   truth %s (%+.4f %+.4f %+.4f) err=%.1fmm %.2fdeg  simT=%.1fs  lag scan:", d.receiverSerial.c_str(), t.pos.x, t.pos.y, t.pos.z,
                            math::Norm(t.pos - p) * 1000, math::Norm((d.worldFromDevice.rot * t.rot.Conj()).ToRotVec()) * 57.2958, d.poseTime - sim::kDeviceClockStart);
                for (int ms = -30; ms <= 30; ms += 10) {
                    math::Pose tl = svcFromSim * simCfg.devices[i].trajectory(d.poseTime - sim::kDeviceClockStart + ms * 1e-3);
                    std::printf(" %+d:%.1fmm/%.2fdeg", ms, math::Norm(tl.pos - p) * 1000, math::Norm((d.worldFromDevice.rot * tl.rot.Conj()).ToRotVec()) * 57.2958);
                }
                std::printf("\n");
            }
            std::printf("   %s %s mode=%s imu=%.0fHz rep=%.0f/s pose=%d (%+.3f %+.3f %+.3f) rms=%.4f sigma=%.1fmm used=%d bases=%d solve=%.0fus g=%.2fdeg %s %s\n", d.receiverSerial.c_str(),
                        d.deviceSerial.c_str(), protocol::ToString(d.lightMode), d.imu.rateHz, d.reportRate, d.hasPose, p.x, p.y, p.z, d.rmsDeg, d.positionSigmaMm, d.used,
                        d.basesUsed, d.solveUs, d.imu.gravityErrorDeg, d.modeNote.c_str(), d.solveNote.c_str());
            const auto& f = d.filter;
            std::printf("      filter %s pos=(%+.4f %+.4f %+.4f) sig=%.2fmm/%.3fdeg v=%.3f w=%.2fdeg/s light=%.0fHz rms=%.4f->%.4fdeg nis=%.2f used=%d gated=%d lag=%.1fms "
                        "upd=%.0fus imuNis=%.2f still=%d zvu=%d bg=(%+.4f %+.4f %+.4f) ba=(%+.3f %+.3f %+.3f) batchDiff=%.1fmm/%.2fdeg resets=%llu %s %s\n",
                        service::ToString(f.state), p.x, p.y, p.z, f.posSigmaMm, f.rotSigmaDeg, math::Norm(f.velocity), math::Norm(f.angularVelocity) * 57.2958,
                        f.lightUpdateHz, f.lastRmsBeforeDeg, f.lastRmsAfterDeg, f.lastNis, f.lastUsed, f.lastGated, f.lastLagMs, f.lightUpdateUs, f.imuNis, f.still, f.zvu,
                        f.gyroBias.x, f.gyroBias.y, f.gyroBias.z, f.accBias.x, f.accBias.y, f.accBias.z, f.batchDiffMm, f.batchDiffDeg,
                        (unsigned long long)f.resets, f.lastResetReason.c_str(), f.lastNote.c_str());
            std::printf("      light applied=%llu skipped=%llu:", (unsigned long long)f.lightUpdates, (unsigned long long)f.lightSkipped);
            for (auto& [why, n] : f.skipReasons) std::printf(" [%s]=%llu", why.c_str(), (unsigned long long)n);
            std::printf("\n");
        }
        for (auto& b : s->bases)
            std::printf("   base ch%d ref=%d cal=%d id=%08X pose=%d pos=(%+.3f %+.3f %+.3f) hz=%.4f syncs=%llu ootx=%u/%u\n", b.channel + 1, b.isReference, b.hasCalibration, b.cal.id,
                        b.hasPose, b.worldFromBase.pos.x, b.worldFromBase.pos.y, b.worldFromBase.pos.z, b.measuredHz, (unsigned long long)b.syncs, b.ootx.progressBytes,
                        b.ootx.expectedBytes);
    }
    svc.Stop();
    if (traceFile) std::fclose(traceFile);
    return 0;
}

// The only command that writes: select the light encoding of the device behind a receiver.
static int SetMode(const std::string& serial, const std::string& modeName) {
    std::string reason;
    if (!service::ActiveModeAllowed(service::ProbeHost(), &reason)) {
        std::printf("refused: %s\n", reason.c_str());
        return 1;
    }
    protocol::LightMode mode = modeName == "raw1" ? protocol::LightMode::Raw1 : protocol::LightMode::Raw0;
    for (auto& d : hid::Enumerate(0x28de)) {
        if (d.serial != serial) continue;
        std::string err;
        auto dev = hid::HidDevice::Open(d, hid::Access::ReadWrite, &err);
        if (!dev) {
            std::printf("open failed: %s\n", err.c_str());
            return 1;
        }
        auto cmd = protocol::commands::RfSetLightMode(mode);
        bool ok = dev->SetFeature(cmd.data(), cmd.size());
        std::printf("%s -> %s: %s\n", serial.c_str(), protocol::ToString(mode), ok ? "ok" : dev->LastError().c_str());
        return ok ? 0 : 1;
    }
    std::printf("receiver %s not found\n", serial.c_str());
    return 1;
}

int main(int argc, char** argv) {
    std::string cmd = argc > 1 ? argv[1] : "";
    if (cmd == "list") return List();
    if (cmd == "capture" && argc >= 5) return Capture(argv[2], std::atof(argv[3]), argv[4]);
    if (cmd == "dump" && argc >= 3) return Dump(argv[2], argc >= 4 ? std::atoi(argv[3]) : 4);
    if (cmd == "live" && argc >= 4) return Live(std::atof(argv[2]), argv[3], argc >= 5 && std::string(argv[4]) == "active", "");
    if (cmd == "replay" && argc >= 5) return Live(std::atof(argv[2]), argv[3], false, argv[4]);
    if (cmd == "gyrocheck" && argc >= 6) return GyroCheck(argv[2], argv[3], std::atoi(argv[4]), std::atoi(argv[5]));   // <file> <cfg> <source> <channel>
    if (cmd == "trace" && argc >= 6) return Live(std::atof(argv[2]), argv[3], false, argv[4], argv[5]);   // trace <s> <cfg> <file> <out.csv>
    if (cmd == "solve" && argc >= 4) return Solve(argv[2], argv[3], argc >= 5 ? std::atoi(argv[4]) : 50, argc >= 6 ? std::atoi(argv[5]) : -1, argc >= 7 ? std::atoi(argv[6]) : -1);
    if (cmd == "angles" && argc >= 4) return Angles(argv[2], std::atoi(argv[3]), argc >= 5 ? std::atoi(argv[4]) : 200);
    if (cmd == "setmode" && argc >= 4) return SetMode(argv[2], argv[3]);
    if (cmd == "decode" && argc >= 3) return Decode(argv[2], argc >= 4 ? std::atoi(argv[3]) : 20);
    std::printf("usage:\n  lhp_hidtool list\n  lhp_hidtool capture <serial|all> <seconds> <out.lhrec>\n  lhp_hidtool dump <file.lhrec> [hex-per-id]\n");
    return 2;
}

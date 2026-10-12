#include <doctest.h>

#include "tracking/io/Recording.h"
#include "tracking/lighthouse/BaseStation.h"
#include "tracking/lighthouse/Lh2Processor.h"
#include "tracking/lighthouse/Ootx.h"
#include "tracking/model/DeviceModel.h"
#include "tracking/protocol/WatchmanDecoder.h"
#include "tracking/sim/Simulator.h"
#include "tracking/solver/PoseSolver.h"

#include <cmath>
#include <cstdio>
#include <map>

using namespace lhp::tracking;
using math::Pose;
using math::Quat;
using math::Vec3;

TEST_CASE("quaternion basics") {
    Quat q = Quat::FromRotVec({0, math::kPi / 2, 0});
    Vec3 v = q.Rotate({1, 0, 0});
    CHECK(v.x == doctest::Approx(0).epsilon(1e-9));
    CHECK(v.z == doctest::Approx(-1).epsilon(1e-9));
    Vec3 r = Quat::FromRotVec({0.1, -0.2, 0.3}).ToRotVec();
    CHECK(r.x == doctest::Approx(0.1));
    CHECK(r.y == doctest::Approx(-0.2));
    CHECK(r.z == doctest::Approx(0.3));
    Pose p{Quat::FromRotVec({0.3, 0.2, 0.1}), {1, 2, 3}};
    Vec3 back = p.Inverse().Apply(p.Apply({0.5, -0.4, 0.2}));
    CHECK(back.x == doctest::Approx(0.5));
    CHECK(back.y == doctest::Approx(-0.4));
}

TEST_CASE("ideal reprojection and bearing are inverse") {
    Vec3 p = math::Normalize({0.3, -0.2, -1.0});
    double a0 = lighthouse::ReprojectAxisIdeal(p, 0), a1 = lighthouse::ReprojectAxisIdeal(p, 1);
    Vec3 b = solver::BearingFromAngles(a0, a1);
    CHECK(b.x == doctest::Approx(p.x).epsilon(1e-6));
    CHECK(b.y == doctest::Approx(p.y).epsilon(1e-6));
    CHECK(b.z == doctest::Approx(p.z).epsilon(1e-6));
    // straight ahead -> both angles zero
    CHECK(lighthouse::ReprojectAxisIdeal({0, 0, -2}, 0) == doctest::Approx(0).epsilon(1e-12));
}

TEST_CASE("OOTX encode / decode round trip") {
    lighthouse::BaseCalibration c;
    c.id = 0x12345678;
    c.firmware = (560 << 6) | 7;
    c.mode = 3;
    c.accelDir[0] = -8, c.accelDir[1] = 127, c.accelDir[2] = 84;
    c.axis[0] = {0, -0.0465, -0.0665, 1.53, -0.0042, 0.832, -0.0525};
    c.axis[1] = {-0.0054, 0.0535, 0.109, 1.91, -0.0048, 1.41, -0.0288};
    auto bits = sim::EncodeOotx(c);
    lighthouse::OotxDecoder dec;
    std::optional<std::vector<uint8_t>> pkt;
    for (int pass = 0; pass < 2 && !pkt; ++pass)
        for (int b : bits)
            if (auto p = dec.Push(b)) pkt = p;
    REQUIRE(pkt.has_value());
    auto parsed = lighthouse::ParseOotxV15(pkt->data(), pkt->size());
    REQUIRE(parsed.has_value());
    CHECK(parsed->id == 0x12345678u);
    CHECK(parsed->mode == 3);
    CHECK(parsed->accelDir[1] == 127);
    CHECK(parsed->axis[0].tilt == doctest::Approx(-0.0465).epsilon(1e-3));
    CHECK(parsed->axis[1].ogeePhase == doctest::Approx(1.41).epsilon(1e-3));
    CHECK(dec.Stats().badCrc == 0);
}

TEST_CASE("OOTX survives missed rotations on a repeating packet") {
    lighthouse::BaseCalibration c;
    c.id = 0xCAFEBABE;
    auto bits = sim::EncodeOotx(c);
    lighthouse::OotxDecoder dec;
    int packets = 0;
    for (int pass = 0; pass < 4; ++pass)
        for (size_t i = 0; i < bits.size(); ++i) {
            bool missed = pass >= 2 && (i % 37 == 5);   // after a clean pass, drop some bits
            if (dec.Push(missed ? -1 : bits[i])) ++packets;
        }
    CHECK(packets >= 3);
}

TEST_CASE("device model from config JSON") {
    sim::SimDevice d;
    d.serial = "LHR-0000ABCD";
    d.modelNumber = "Test Tracker";
    auto m = model::ParseDeviceModel(sim::DeviceConfigJson(d));
    REQUIRE(m.has_value());
    CHECK(m->serial == "LHR-0000ABCD");
    CHECK(m->points.size() == 20);
    // sim wire ids are sensorCount - index (id 0 unused)
    CHECK(m->MapSensor(20) == 0);
    CHECK(m->MapSensor(17) == 3);
    CHECK(m->MapSensor(0) == -1);
    CHECK(m->ReportedId(3) == 17);
    CHECK(m->MapSensor(40) == -1);
    CHECK_FALSE(model::ParseDeviceModel("{\"device_serial_number\":\"x\"}").has_value());
}

TEST_CASE("channelMap is model index -> wire id (Knuckles layout)") {
    // excerpt of a real Knuckles map: model 0 is wire 20, model 2 is wire 24; wire 0 and 11 unused
    std::string json = R"({"device_serial_number":"LHR-TEST","lighthouse_config":{
        "channelMap":[20,13,24],
        "modelPoints":[[0,0,0],[1,0,0],[0,1,0]],
        "modelNormals":[[0,0,1],[0,0,1],[0,0,1]]}})";
    auto m = model::ParseDeviceModel(json);
    REQUIRE(m.has_value());
    CHECK(m->MapSensor(20) == 0);
    CHECK(m->MapSensor(13) == 1);
    CHECK(m->MapSensor(24) == 2);
    CHECK(m->MapSensor(0) == -1);
    CHECK(m->MapSensor(2) == -1);
    CHECK(m->ReportedId(2) == 24);
}

namespace {

/// Runs the simulator for `seconds` and solves the device pose in the frame of base `ch`.
struct E2E {
    double posErrMm = 1e9, rotErrDeg = 1e9, rmsDeg = 1e9;
    uint64_t decodeErrors = 0, syncs = 0;
    bool calibrated = false;
};

E2E RunEndToEnd(double seconds, Pose truth) {
    sim::SimConfig cfg = sim::DemoScene({"RX"});
    cfg.devices[0].trajectory = [truth](double) { return truth; };
    cfg.bases.resize(1);   // single base: solve directly in its frame
    auto model = model::ParseDeviceModel(sim::DeviceConfigJson(cfg.devices[0]));
    REQUIRE(model.has_value());
    auto src = sim::OpenSimSource(cfg, 0, 0, false);
    protocol::WatchmanDecoder dec;
    lighthouse::Lh2Processor lh2;
    std::optional<lighthouse::BaseCalibration> cal;
    std::map<int, lighthouse::Lh2Observation> latest;
    io::RawReport r;
    std::vector<protocol::Event> ev;
    E2E out;
    protocol::Ticks t0 = 0, lastSync = 0;
    while (src->Read(r, 0)) {
        ev.clear();
        dec.Decode(r.bytes.data(), r.bytes.size(), ev);
        for (auto& e : ev) {
            if (auto* sw = std::get_if<protocol::Lh2Sweep>(&e)) {
                if (auto o = lh2.OnSweep(*sw)) latest[o->sensor * 2 + o->plane] = *o;
            } else if (auto* sy = std::get_if<protocol::Lh2Sync>(&e)) {
                if (auto c = lh2.OnSync(*sy)) cal = c;
                if (!t0) t0 = sy->time;
                lastSync = sy->time;
            }
        }
        if (lastSync && (lastSync - t0) / protocol::kTicksPerSecond > seconds) break;
    }
    out.decodeErrors = dec.Stats().errors;
    out.syncs = dec.Stats().syncs;
    out.calibrated = cal.has_value();
    std::vector<solver::Measurement> meas;
    for (auto& [k, o] : latest)
        if ((lastSync - o.time) / protocol::kTicksPerSecond < 0.03) meas.push_back({0, model->MapSensor(o.sensor), o.plane, o.angle, 0});
    solver::BaseView view;
    if (cal)
        for (int a = 0; a < 2; ++a) view.cal[a] = cal->axis[a];
    solver::SolverOptions opt;
    opt.estimateVelocity = false;
    auto res = solver::SolvePose(*model, meas, {view}, nullptr, opt);
    REQUIRE(res.ok);
    Pose expected = cfg.bases[0].worldFromBase.Inverse() * truth;   // device in the base frame
    out.posErrMm = math::Norm(res.worldFromDevice.pos - expected.pos) * 1000;
    Quat dq = res.worldFromDevice.rot * expected.rot.Conj();
    out.rotErrDeg = math::Norm(dq.ToRotVec()) * 180 / math::kPi;
    out.rmsDeg = res.rmsRad * 180 / math::kPi;
    return out;
}

}  // namespace

TEST_CASE("end to end: simulated reports -> decode -> LH2 -> OOTX -> pose") {
    Pose truth{Quat::FromRotVec({0.2, 0.9, -0.1}).Normalized(), {0.3, 0.9, -2.2}};
    E2E e = RunEndToEnd(25.0, truth);   // long enough for the OOTX calibration to arrive
    CHECK(e.decodeErrors == 0);
    CHECK(e.syncs > 1000);
    CHECK(e.calibrated);
    CHECK(e.posErrMm < 3.0);
    CHECK(e.rotErrDeg < 0.5);
    CHECK(e.rmsDeg < 0.01);
}

TEST_CASE("recording round trip") {
    char tmp[512];
    std::snprintf(tmp, sizeof tmp, "%s\\lhp_test_rec.lhrec", std::getenv("TEMP") ? std::getenv("TEMP") : ".");
    {
        std::string err;
        auto w = io::RecordingWriter::Create(tmp, &err);
        REQUIRE(w);
        io::SourceInfo s;
        s.id = 3;
        s.vid = 0x28de;
        s.pid = 0x2101;
        s.serial = "TEST123";
        w->AddSource(s);
        io::RawReport r;
        r.hostUs = 42;
        r.sourceId = 3;
        r.bytes = {0x23, 1, 2, 3};
        w->Write(r);
        w->Marker(43, "hello");
    }
    io::Recording rec;
    std::string err;
    REQUIRE(io::LoadRecording(tmp, rec, &err));
    REQUIRE(rec.sources.size() == 1);
    CHECK(rec.sources[0].serial == "TEST123");
    REQUIRE(rec.reports.size() == 1);
    CHECK(rec.reports[0].bytes.size() == 4);
    CHECK(rec.reports[0].hostUs == 42);
    REQUIRE(rec.markers.size() == 1);
    CHECK(rec.markers[0].second == "hello");
    std::remove(tmp);
}

// ---------------------------------------------------------------- tracking filter

#include "tracking/filter/TrackerFilter.h"
#include "tracking/service/TrackingService.h"
#include "tracking/solver/Scene.h"
#include "tracking/solver/UniverseIo.h"

#include <chrono>
#include <random>
#include <thread>

namespace {

struct FilterRig {
    model::DeviceModel model;
    std::vector<filter::LightBase> bases;
    std::mt19937 rng{7};
    double angleNoise = 2e-5, accNoise = 0.03, gyroNoise = 0.004;
    Vec3 gyroBias{0.006, -0.004, 0.003};

    FilterRig() {
        sim::SimDevice d;
        d.serial = "LHR-0000F17E";
        d.sensorCount = 22;
        model = *model::ParseDeviceModel(sim::DeviceConfigJson(d));
        auto cfg = sim::DemoScene({"RX"});
        for (auto& b : cfg.bases) {
            filter::LightBase lb;
            lb.worldFromBase = b.worldFromBase;
            lb.cal[0] = b.cal.axis[0];
            lb.cal[1] = b.cal.axis[1];
            bases.push_back(lb);
        }
    }
    /// One base rotation worth of angles at times around t for the trajectory.
    std::vector<filter::LightMeas> Light(int base, double t, const std::function<Pose(double)>& traj) {
        std::normal_distribution<double> n(0, angleNoise);
        std::vector<filter::LightMeas> out;
        const auto& b = bases[base];
        Pose inv = b.worldFromBase.Inverse();
        for (int s = 0; s < (int)model.points.size(); ++s)
            for (int p = 0; p < 2; ++p) {
                double ts = t - 0.012 + 0.006 * p + 0.0002 * s;
                Pose pose = traj(ts);
                Vec3 pw = pose.Apply(model.points[s]);
                Vec3 nW = pose.rot.Rotate(model.normals[s]);
                if (math::Dot(nW, math::Normalize(b.worldFromBase.pos - pw)) < 0.15) continue;
                Vec3 pb = inv.Apply(pw);
                if (pb.z > -0.05) continue;
                out.push_back({base, s, p, lighthouse::ReprojectAxis(b.cal[p], pb, p) + n(rng), ts});
            }
        return out;
    }
    void Imu(filter::TrackerFilter& f, double t, const std::function<Pose(double)>& traj) {
        std::normal_distribution<double> an(0, accNoise), gn(0, gyroNoise);
        const double h = 0.002;
        Pose p = traj(t);
        Vec3 acc = (traj(t + h).pos + traj(t - h).pos - p.pos * 2.0) / (h * h);
        Vec3 w = (traj(t + h).rot * traj(t - h).rot.Conj()).ToRotVec() / (2 * h);
        Quat imuFromWorld = (p.rot * model.imu.mount.rot).Conj();
        Vec3 a = imuFromWorld.Rotate(acc + Vec3{0, filter::kGravity, 0}) + Vec3{an(rng), an(rng), an(rng)};
        Vec3 g = imuFromWorld.Rotate(w) + gyroBias + Vec3{gn(rng), gn(rng), gn(rng)};
        f.UpdateImu(t, a, g);
    }
    /// Runs the filter along a trajectory; returns final (pos error mm, rot error deg).
    std::pair<double, double> Run(filter::TrackerFilter& f, const std::function<Pose(double)>& traj, double seconds, bool zvuWhenStill = false) {
        double nextLight[2] = {0.02, 0.031};
        const double period[2] = {1 / 50.0521, 1 / 50.5796};
        for (double t = 0.004; t < seconds; t += 0.004) {
            Imu(f, t, traj);
            if (zvuWhenStill) f.UpdateZeroVelocity(1e-5, true);
            for (int b = 0; b < 2; ++b)
                if (t >= nextLight[b]) {
                    f.UpdateLight(Light(b, nextLight[b], traj), bases);
                    nextLight[b] += period[b];
                }
        }
        Pose truth = traj(f.Time());
        double pe = math::Norm(f.WorldFromDevice().pos - truth.pos) * 1000;
        double re = math::Norm((f.WorldFromDevice().rot * truth.rot.Conj()).ToRotVec()) * 180 / math::kPi;
        return {pe, re};
    }
};

std::array<double, 36> PoseCov(double rotSigma, double posSigma) {
    std::array<double, 36> c{};
    for (int k = 0; k < 3; ++k) {
        c[k * 6 + k] = rotSigma * rotSigma;
        c[(k + 3) * 6 + k + 3] = posSigma * posSigma;
    }
    return c;
}

}  // namespace

TEST_CASE("filter: static device converges and learns the gyro bias") {
    FilterRig rig;
    Pose truth{Quat::FromRotVec({0.3, -0.8, 0.2}).Normalized(), {0.4, 1.0, -2.0}};
    auto traj = [&](double) { return truth; };
    filter::TrackerFilter f(rig.model);
    Pose start{(Quat::FromRotVec({0.01, -0.01, 0.015}) * truth.rot).Normalized(), truth.pos + Vec3{0.006, -0.004, 0.005}};
    f.Initialize(0, start, PoseCov(0.03, 0.01), {});
    auto [pe, re] = rig.Run(f, traj, 6.0, true);
    CHECK(f.Healthy());
    CHECK(pe < 0.5);
    CHECK(re < 0.05);
    CHECK(f.PositionSigma() * 1000 < 1.0);
    Vec3 bgErr = f.GyroBias() - rig.gyroBias;
    CHECK(math::Norm(bgErr) < 0.0015);
}

TEST_CASE("filter: tracks a fast moving device without motion blur") {
    FilterRig rig;
    auto traj = [](double t) {
        // ~1.3 m/s, ~3 rad/s: a brisk hand motion
        Vec3 p{0.25 * std::cos(5 * t), 1.0 + 0.1 * std::sin(10 * t), -2.0 + 0.25 * std::sin(5 * t)};
        Quat q = Quat::FromRotVec({0.3 * std::sin(3 * t), 2.5 * t, 0.2 * std::cos(4 * t)});
        return Pose{q.Normalized(), p};
    };
    filter::TrackerFilter f(rig.model);
    Pose t0 = traj(0);
    f.Initialize(0, t0, PoseCov(0.01, 0.005), {});
    // average the error over the last second
    double sumP = 0, sumR = 0;
    int n = 0;
    double nextLight[2] = {0.02, 0.031};
    const double period[2] = {1 / 50.0521, 1 / 50.5796};
    for (double t = 0.004; t < 4.0; t += 0.004) {
        rig.Imu(f, t, traj);
        for (int b = 0; b < 2; ++b)
            if (t >= nextLight[b]) {
                f.UpdateLight(rig.Light(b, nextLight[b], traj), rig.bases);
                nextLight[b] += period[b];
            }
        if (t > 3.0) {
            Pose truth = traj(f.Time());
            sumP += math::Norm(f.WorldFromDevice().pos - truth.pos) * 1000;
            sumR += math::Norm((f.WorldFromDevice().rot * truth.rot.Conj()).ToRotVec()) * 180 / math::kPi;
            ++n;
        }
    }
    CHECK(f.Healthy());
    CHECK(sumP / n < 2.0);
    CHECK(sumR / n < 0.2);
}

TEST_CASE("filter: re-expressing the world keeps the device where it is") {
    FilterRig rig;
    filter::TrackerFilter f(rig.model);
    Pose p{Quat::FromRotVec({0.1, 0.2, 0.3}).Normalized(), {1, 2, 3}};
    f.Initialize(0, p, PoseCov(0.01, 0.01), {0, 1, 0});
    Pose T{Quat::FromRotVec({0, 0.5, 0.05}).Normalized(), {0, -0.3, 0}};
    f.TransformWorld(T);
    Pose e = T * p;
    CHECK(math::Norm(f.WorldFromDevice().pos - e.pos) < 1e-9);
    CHECK(math::Norm(f.AngularVelocity() - T.rot.Rotate({0, 1, 0})) < 1e-9);
}

TEST_CASE("service end to end: simulated devices tracked by the filter vs ground truth") {
    sim::SimConfig cfg = sim::DemoScene({"SIM-A", "SIM-B"});
    std::vector<std::unique_ptr<io::ReportSource>> src;
    for (size_t i = 0; i < cfg.devices.size(); ++i) src.push_back(sim::OpenSimSource(cfg, i, (uint16_t)i, false));
    service::TrackingService svc(nullptr);
    svc.StartSources(std::move(src), "sim", "sim", sim::MakeModelProvider(cfg), true);
    // run until the second base is bootstrapped and the world tilt refined (simulation runs faster than real time)
    service::SnapshotPtr s;
    auto t0 = std::chrono::steady_clock::now();
    bool ready = false;
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(60)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        s = svc.Snapshot();
        if (!s || s->devices.size() < 2 || s->bases.size() < 2) continue;
        bool all = true;
        for (auto& b : s->bases) all &= b.hasPose && b.hasCalibration && (b.isReference || b.bootstrapSamples >= 100);
        for (auto& d : s->devices) all &= d.filter.state == service::TrackState::Tracking && d.poseTime > sim::kDeviceClockStart + 150;
        if (all) {
            ready = true;
            break;
        }
    }
    svc.Stop();
    REQUIRE(ready);
    // service world vs simulation world: both anchored on the reference base
    const service::BaseSnapshot* ref = nullptr;
    for (auto& b : s->bases)
        if (b.isReference) ref = &b;
    REQUIRE(ref);
    Pose simRef;
    for (auto& b : cfg.bases)
        if (b.channel == ref->channel) simRef = b.worldFromBase;
    Pose svcFromSim = ref->worldFromBase * simRef.Inverse();
    // the other base, learned from the devices
    for (auto& b : s->bases) {
        if (b.isReference) continue;
        for (auto& sb : cfg.bases)
            if (sb.channel == b.channel) CHECK(math::Norm(b.worldFromBase.pos - (svcFromSim * sb.worldFromBase).pos) * 1000 < 10.0);
    }
    for (auto& d : s->devices) {
        size_t i = d.receiverSerial == "SIM-A" ? 0 : 1;
        Pose truth = svcFromSim * cfg.devices[i].trajectory(d.poseTime - sim::kDeviceClockStart);
        double pe = math::Norm(d.worldFromDevice.pos - truth.pos) * 1000;
        double re = math::Norm((d.worldFromDevice.rot * truth.rot.Conj()).ToRotVec()) * 180 / math::kPi;
        INFO(d.receiverSerial << " pos err " << pe << " mm, rot err " << re << " deg, resets " << d.filter.resets);
        CHECK(pe < 5.0);
        CHECK(re < 0.5);
        CHECK(d.filter.resets == 0);
    }
    // the world "up" refined from the devices' accelerometers beats the base's 8-bit accelerometer
    CHECK(s->tiltSamples > 0);
}

TEST_CASE("SteamVR lighthousedb universe: parse both readings, live samples pick one") {
    const char* db = R"({"known_universes":[{"id":"1234","base_stations":[
        {"base_serial_number":11111111,"target_pose":{"pose":[0,0,0,1,0,0,0],"target_serial_number":0}},
        {"base_serial_number":22222222,"target_pose":{"pose":[0,0.9805807,0,0.1961161,3.0,-0.2,-5.0],"target_serial_number":0}}]}]})";
    auto u = solver::ParseSteamVrLighthouseDb(db);
    REQUIRE(u.size() == 1);
    CHECK(u[0].referenceId == 11111111u);
    REQUIRE(u[0].bases.size() == 2);
    const solver::UniverseBase* other = nullptr;
    for (auto& b : u[0].bases)
        if (b.id == 22222222u) other = &b;
    REQUIRE(other);
    REQUIRE(other->alternative.has_value());
    CHECK(math::Norm(other->refFromBase.pos - Vec3{3.0, -0.2, -5.0}) < 1e-9);

    // a scene whose second base really sits at the *inverse* reading
    solver::Scene sc;
    lighthouse::BaseCalibration c0, c1;
    c0.id = 11111111;
    c0.raw = {1};
    c0.accelDir[1] = 127;
    c1.id = 22222222;
    c1.raw = {2};
    sc.SetCalibration(0, c0);
    sc.SetKnownUniverses(u);
    sc.SetCalibration(5, c1);
    CHECK_FALSE(sc.HasPose(5));   // waiting for verification
    Pose real = *other->alternative;
    Pose refFromDevice{Quat::FromRotVec({0.1, 0.4, 0}).Normalized(), {0.5, -1.0, -2.0}};
    Pose baseFromDevice = real.Inverse() * refFromDevice;
    for (int i = 0; i < 6; ++i) sc.AddBootstrapSample(5, refFromDevice, baseFromDevice);
    REQUIRE(sc.HasPose(5));
    CHECK(math::Norm(sc.Base(5).refFromBase.pos - real.pos) < 1e-6);
    CHECK(sc.Base(5).source == solver::PoseSource::SteamVrUniverse);
}

TEST_CASE("saved universe round trip and restore by base id") {
    solver::UniverseRecord u;
    u.source = "saved";
    u.referenceId = 0xAABBCCDD;
    u.bases.push_back({0xAABBCCDD, Pose{}, 0, std::nullopt});
    u.bases.push_back({0x01020304, Pose{Quat::FromRotVec({0, 2.5, 0}).Normalized(), {2, 0.1, -4}}, 250, std::nullopt});
    u.hasUp = true;
    u.upInRef = math::Normalize({0.01, 0.9, 0.43});
    u.upSamples = 40;
    auto back = solver::UniverseFromJson(solver::UniverseToJson(u));
    REQUIRE(back.has_value());
    CHECK(back->referenceId == 0xAABBCCDDu);
    REQUIRE(back->bases.size() == 2);
    // restore: the bases appear on other channels than last time, ids decide
    solver::Scene sc;
    lighthouse::BaseCalibration a, b;
    a.id = 0x01020304;   // this time the *other* base is seen first and becomes our reference
    a.raw = {1};
    a.accelDir[1] = 127;
    b.id = 0xAABBCCDD;
    b.raw = {2};
    sc.SetKnownUniverses({*back});
    sc.SetCalibration(7, a);
    sc.SetCalibration(2, b);
    REQUIRE(sc.HasPose(2));
    CHECK(sc.Base(2).source == solver::PoseSource::SavedUniverse);
    // ref = 0x01020304, so base 0xAABBCCDD sits at inverse(saved pose)
    Pose expected = back->bases[1].refFromBase.Inverse();
    for (auto& e : back->bases)
        if (e.id == 0x01020304) expected = e.refFromBase.Inverse();
    CHECK(math::Norm(sc.Base(2).refFromBase.pos - expected.pos) < 1e-6);
    CHECK(sc.UpSamples() == 40);
}

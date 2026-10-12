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
    CHECK(m->MapSensor(3) == 3);
    CHECK(m->MapSensor(40) == -1);
    CHECK_FALSE(model::ParseDeviceModel("{\"device_serial_number\":\"x\"}").has_value());
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

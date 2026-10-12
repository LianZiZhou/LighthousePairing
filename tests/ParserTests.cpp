#include <doctest.h>

#include "backend/ConsoleParser.h"

#include <fstream>
#include <string>
#include <vector>

using namespace lhp;
using namespace lhp::parser;

static std::vector<std::string> ReadLines(const char* name) {
    std::ifstream f(std::string(LHP_TRANSCRIPTS) + "/" + name);
    std::vector<std::string> out;
    std::string l;
    while (std::getline(f, l)) out.push_back(l);
    return out;
}

TEST_CASE("prompt stripping") {
    int n = 0;
    CHECK(StripPrompt("lh> lh> battery status not provided by device\r", &n) == "battery status not provided by device");
    CHECK(n == 2);
    CHECK(StripPrompt("lh> ", &n).empty());
    CHECK(n == 1);
    CHECK(StripPrompt("plain") == "plain");
}

TEST_CASE("receiver list + version from startup") {
    auto lines = ReadLines("startup_dongle.txt");
    REQUIRE(!lines.empty());
    auto list = ParseReceiverList(lines);
    REQUIRE(list.size() == 11);
    CHECK(list.front() == "93F15DFEA6");
    CHECK(list[4] == "ADD66C2A2F-LYM");
    CHECK(list.back() == "7EA0D62A2B");
    CHECK(ParseConsoleVersion(lines[0]).value() == "11000037");
    auto ev = Classify(lines[13]);
    REQUIRE(std::holds_alternative<EvSessionConnected>(ev));
    CHECK(std::get<EvSessionConnected>(ev).receiver == "09BCE41887-LYM");
}

TEST_CASE("dongleinfo") {
    auto lines = ReadLines("startup_dongle.txt");
    std::vector<DongleEntry> d;
    for (auto& l : lines)
        if (auto x = ParseDongleInfo(l); !x.empty()) d = x;
    REQUIRE(d.size() == 11);
    CHECK(d[0].serial == "F3F93916B4");
    CHECK(d[0].radioVersion == 1461100729u);
    CHECK(d[0].extra.empty());
    CHECK(d[1].serial == "ADD66C2A2F-LYM");
    CHECK(d[1].radioVersion == 1558748372u);

    auto pimax = ParseDongleInfo("Dongles:A04165BAAA-RYB,1558748372,2,10;109031C053,1525468496,1,0");
    REQUIRE(pimax.size() == 2);
    CHECK(pimax[0].extra == std::vector<std::string>{"2", "10"});
    CHECK(pimax[1].extra == std::vector<std::string>{"1", "0"});
}

TEST_CASE("imustats") {
    auto s = ParseImuStats("lh> imu 62 rate 245.1Hz interval 4.1ms sigma 0.506ms grav 9.97m/s/s sigma 0.016");
    REQUIRE(s);
    CHECK(s->count == 62);
    CHECK(s->rateHz == doctest::Approx(245.1));
    CHECK(s->intervalMs == doctest::Approx(4.1));
    CHECK(s->sigmaMs == doctest::Approx(0.506));
    CHECK(s->grav == doctest::Approx(9.97));
    auto z = ParseImuStats("imu 0 rate -0.0Hz interval 0.0ms sigma 0.000ms grav 0.00m/s/s sigma 0.000");
    REQUIRE(z);
    CHECK(z->rateHz == 0.0);
    CHECK(!ParseImuStats("battery status not provided by device"));
}

TEST_CASE("usbstats") {
    auto e = ParseUsbStatLine("  28de 2102 09BCE41887-LYM IMU             0/sec, max interval 0ms");
    REQUIRE(e);
    CHECK(e->vid == "28de");
    CHECK(e->pid == "2102");
    CHECK(e->serial == "09BCE41887-LYM");
    CHECK(e->iface == "IMU");
    CHECK(e->perSec == 0.0);
    auto f = ParseUsbStatLine("  28de 2101 109031C053 IMU           998/sec, max interval 12ms");
    REQUIRE(f);
    CHECK(f->perSec == 998.0);
    CHECK(f->maxIntervalMs == 12.0);
}

TEST_CASE("battery") {
    auto b = ParseBattery({"battery status not provided by device"});
    CHECK(b.notProvided);
    CHECK(!b.percent);
    auto c = ParseBattery({"Battery: 85% (charging)"});
    CHECK(c.percent.value() == 85);
    CHECK(c.charging.value());
}

TEST_CASE("event classification") {
    auto lines = ReadLines("pimax_events.txt");
    REQUIRE(lines.size() >= 10);
    auto e0 = Classify(lines[0]);
    REQUIRE(std::holds_alternative<EvDeviceConnected>(e0));
    CHECK(std::get<EvDeviceConnected>(e0).device == "LHR-F0F4B423");
    CHECK(std::get<EvDeviceConnected>(e0).receiver == "18848BA78B");
    CHECK(std::holds_alternative<EvDemodRate>(Classify(lines[1])));
    CHECK(std::holds_alternative<EvBootstrapped>(Classify(lines[2])));
    auto gap = Classify(lines[3]);
    REQUIRE(std::holds_alternative<EvKeepaliveGap>(gap));
    CHECK(std::get<EvKeepaliveGap>(gap).seconds == doctest::Approx(0.484));
    CHECK(std::get<EvKeepaliveGap>(gap).receiver == "109031C053");
    auto d1 = Classify(lines[4]);
    REQUIRE(std::holds_alternative<EvDeviceDisconnected>(d1));
    CHECK(std::get<EvDeviceDisconnected>(d1).device == "LHR-9E63AEE0");
    CHECK(std::get<EvDeviceDisconnected>(d1).receiver == "109031C053");
    CHECK(std::holds_alternative<EvDeviceDisconnected>(Classify(lines[5])));
    auto rs = Classify(lines[6]);
    REQUIRE(std::holds_alternative<EvTrackingReset>(rs));
    CHECK(std::get<EvTrackingReset>(rs).reason.find("out of bounds") != std::string::npos);
    auto p = Classify(lines[8]);
    REQUIRE(std::holds_alternative<EvPairing>(p));
    CHECK(std::get<EvPairing>(p).kind == EvPairing::Kind::Success);
    CHECK(std::holds_alternative<EvCommandError>(Classify(lines[9])));
}

TEST_CASE("poll noise is not logged, events are") {
    CHECK(IsPollNoise("lh> imu 62 rate 245.1Hz interval 4.1ms sigma 0.506ms grav 9.97m/s/s sigma 0.016"));
    CHECK(IsPollNoise("USB status (rates over 10 sec)"));
    CHECK(IsPollNoise("  28de 2102 09BCE41887-LYM IMU             0/sec, max interval 0ms"));
    CHECK(IsPollNoise("battery status not provided by device"));
    CHECK_FALSE(IsPollNoise("109031C053: Packet received after 0.484s, keepalive (0/1)"));
    CHECK_FALSE(IsPollNoise("lighthouse_console: Connected to receiver 09BCE41887-LYM"));
}

TEST_CASE("device classification") {
    CHECK(ClassifyText("Knuckles Right") == DeviceClass::IndexController);
    CHECK(ClassifyText("valve/index_controllerLHR-F0F4B423") == DeviceClass::IndexController);
    CHECK(ClassifyText("VIVE Tracker 3.0 MV") == DeviceClass::ViveTracker);
    CHECK(ClassifyText("Tundra Tracker") == DeviceClass::TundraTracker);
    CHECK(ClassifyText("HardwareRevision:0x84020109") == DeviceClass::ViveTracker);
    CHECK(ClassifyText("HardwareRevision:0x110e0109") == DeviceClass::IndexController);
    CHECK(ClassifyText("vive_controller") == DeviceClass::ViveWand);
    CHECK(ClassifyText("something else") == DeviceClass::Unknown);
    DeviceInfo d;
    d.rawVersion = {"Manufacturer: HTC", "Hardware: 0x84020109"};
    CHECK(ClassifyDevice(d) == DeviceClass::ViveTracker);
    CHECK_FALSE(CanVibrate(DeviceClass::TundraTracker));
    CHECK(CanVibrate(DeviceClass::IndexController));
}

TEST_CASE("left / right hand and wired serials") {
    DeviceInfo l;
    l.deviceClass = DeviceClass::IndexController;
    l.rawVersion = {"0x110e0109 product 17 rev 14.1.9 lot 2020/8/27 0"};   // seen for LHR-F0F4B423 (Knuckles Left)
    CHECK(DetectHand(l) == Hand::Left);
    DeviceInfo r;
    r.hardware = "0x110e0009";                                             // LHR-9E63AEE0 (Knuckles Right)
    CHECK(DetectHand(r) == Hand::Right);
    DeviceInfo m;
    m.props = {{"Render Model", "valve_controller_knu_1_0_right"}};
    CHECK(DetectHand(m) == Hand::Right);
    CHECK(IsWiredDeviceSerial("LHR-BB18B08E"));
    CHECK_FALSE(IsWiredDeviceSerial("A04165BAAA-RYB"));
    auto all = FindLhrSerials({"Device LHR-BB18B08E (HMD)", "Device lhr-f0f4b423", "again LHR-BB18B08E"});
    REQUIRE(all.size() == 2);
    CHECK(all[0] == "LHR-BB18B08E");
    CHECK(all[1] == "LHR-F0F4B423");
    CHECK_FALSE(LooksLikeError("Error reading usb json configuration for serial LHR-BB18B08E"));
}

TEST_CASE("key values") {
    auto kv = ParseKeyValues({"Serial: LHR-9E63AEE0", "Model Number = Knuckles Right", "free text line"});
    REQUIRE(kv.size() == 3);
    CHECK(kv[0].first == "Serial");
    CHECK(kv[0].second == "LHR-9E63AEE0");
    CHECK(kv[1].first == "Model Number");
    CHECK(kv[1].second == "Knuckles Right");
    CHECK(kv[2].first.empty());
    CHECK(GuessModel(kv, "") == "Knuckles Right");
}

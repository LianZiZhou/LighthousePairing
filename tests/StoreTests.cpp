#include <doctest.h>

#include "core/Store.h"
#include "backend/ConsoleSession.h"
#include "util/Text.h"

using namespace lhp;

TEST_CASE("ring buffers and counters") {
    Receiver r;
    for (size_t i = 0; i < Receiver::kMaxSamples + 50; ++i) {
        SignalSample s;
        s.t = (double)i;
        Store::PushSample(r, s);
    }
    CHECK(r.samples.size() == Receiver::kMaxSamples);
    CHECK(r.samples.front().t == 50.0);
    Store::PushEvent(r, {1, 0, SignalEvent::Type::Gap, 0.4f, "gap"});
    Store::PushEvent(r, {2, 0, SignalEvent::Type::Gap, 1.2f, "gap"});
    Store::PushEvent(r, {3, 0, SignalEvent::Type::Disconnected, 0, "d"});
    CHECK(r.gapCount == 2);
    CHECK(r.disconnectCount == 1);
    CHECK(r.worstGap == doctest::Approx(1.2f));
    CHECK(r.pendingGap == doctest::Approx(1.2f));
}

TEST_CASE("sorting groups by kind") {
    AppState s;
    Receiver a;
    a.serial = "B";
    a.kind = ReceiverKind::WatchmanDongle;
    Receiver b;
    b.serial = "A";
    b.kind = ReceiverKind::HmdBuiltin;
    Receiver c;
    c.serial = "C-RYB";
    c.kind = ReceiverKind::VrRadio;
    s.receivers = {a, b, c};
    Store::SortReceivers(s);
    CHECK(s.receivers[0].serial == "A");
    CHECK(s.receivers[1].serial == "C-RYB");
    CHECK(s.receivers[2].serial == "B");
    CHECK(s.Find("c-ryb") != nullptr);
}

TEST_CASE("text helpers") {
    CHECK(ToUtf8(ToWide("接收器 ABC")) == "接收器 ABC");
    CHECK(Trim("  x \r\n") == "x");
    CHECK(Split("a,,b", ',').size() == 3);
}

// Spawns a fake console (cmd.exe) to exercise the pipe plumbing without hardware.
TEST_CASE("console session plumbing with cmd.exe") {
    std::vector<std::string> lines;
    std::mutex m;
    ConsoleSession s(L"C:\\Windows\\System32\\cmd.exe", "", [&](const std::string& l) {
        std::scoped_lock lk(m);
        lines.push_back(l);
    });
    // cmd.exe never prints "lh> ", so Start must time out cleanly.
    std::string err;
    CHECK_FALSE(s.Start(&err, 1.0));
    CHECK(!err.empty());
    CHECK_FALSE(s.Alive());
}

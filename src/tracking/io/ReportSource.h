#pragma once
// L0: where raw HID reports come from. Live HID and file replay implement the same interface,
// so decoding / solving / UI run identically on real hardware, recordings and tests.
#include "tracking/hid/HidDevice.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace lhp::tracking::io {

/// Microseconds on a monotonic host clock (QueryPerformanceCounter).
uint64_t HostMicros();

/// Identity of one report stream (one HID interface).
struct SourceInfo {
    uint16_t id = 0;             // stable index inside a session / recording
    uint16_t vid = 0, pid = 0;
    int interfaceNumber = -1;
    std::string serial;          // receiver serial (dongles) or device serial (wired)
    std::string product;
    uint16_t inputReportLength = 0;
};

struct RawReport {
    uint64_t hostUs = 0;         // arrival time on the host
    uint16_t sourceId = 0;
    std::vector<uint8_t> bytes;  // bytes[0] = report id
};

class ReportSource {
public:
    virtual ~ReportSource() = default;
    virtual const SourceInfo& Info() const = 0;
    /// Next report. Returns false on timeout (out untouched) or when the source ended (Ended() == true).
    virtual bool Read(RawReport& out, int timeoutMs) = 0;
    virtual bool Ended() const = 0;
    /// Unblocks Read from another thread.
    virtual void Cancel() = 0;
    /// Null when the source cannot accept commands (read-only HID, replay).
    virtual hid::HidDevice* Writable() { return nullptr; }
};

/// Live HID interface.
std::unique_ptr<ReportSource> OpenLive(const hid::HidDeviceInfo& info, uint16_t sourceId, hid::Access access, std::string* error);

}  // namespace lhp::tracking::io

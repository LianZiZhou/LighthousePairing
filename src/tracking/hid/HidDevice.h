#pragma once
// L0 transport: Windows HID enumeration and I/O. Knows nothing about Lighthouse.
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace lhp::tracking::hid {

struct HidDeviceInfo {
    std::wstring path;           // \\?\hid#vid_28de&pid_2101&mi_00#...
    uint16_t vid = 0, pid = 0;
    uint16_t version = 0;
    std::string serial;          // HidD_GetSerialNumberString (receiver serial for dongles)
    std::string product;
    std::string manufacturer;
    uint16_t usagePage = 0, usage = 0;
    uint16_t inputReportLength = 0, outputReportLength = 0, featureReportLength = 0;
    int interfaceNumber = -1;    // from "&mi_xx" in the path, -1 if single interface
    int collection = -1;         // from "&col0x" in the path, -1 if single collection
};

/// All present HID interfaces, optionally filtered by vendor id (0 = all).
std::vector<HidDeviceInfo> Enumerate(uint16_t vendorId = 0);

enum class Access {
    ReadOnly,    // passive: never writes, coexists with SteamVR / lighthouse_console
    ReadWrite,   // needed to send feature / output reports (active mode only)
};

/// One open HID interface. Reads are overlapped so they can time out and be cancelled from another thread.
class HidDevice {
public:
    static std::unique_ptr<HidDevice> Open(const HidDeviceInfo& info, Access access, std::string* error);
    ~HidDevice();
    HidDevice(const HidDevice&) = delete;
    HidDevice& operator=(const HidDevice&) = delete;

    const HidDeviceInfo& Info() const { return info_; }
    Access GetAccess() const { return access_; }

    /// Read one input report (first byte = report id). Returns bytes read, 0 on timeout, -1 on error/closed.
    int Read(uint8_t* buffer, size_t capacity, int timeoutMs);
    /// Unblocks a pending Read (it returns -1). Safe to call from another thread.
    void Cancel();

    bool GetFeature(uint8_t reportId, std::vector<uint8_t>& out);
    bool SetFeature(const uint8_t* data, size_t length);   // requires Access::ReadWrite
    std::string LastError() const { return lastError_; }

private:
    HidDevice() = default;
    HidDeviceInfo info_;
    Access access_ = Access::ReadOnly;
    void* handle_ = nullptr;     // HANDLE
    void* readEvent_ = nullptr;  // HANDLE
    void* overlapped_ = nullptr; // OVERLAPPED*
    std::vector<uint8_t> readBuf_;
    bool pending_ = false;
    bool cancelled_ = false;
    std::string lastError_;
};

}  // namespace lhp::tracking::hid

#include "tracking/hid/HidDevice.h"

#include <windows.h>
#include <setupapi.h>
extern "C" {
#include <hidsdi.h>
#include <hidpi.h>
}

#include <cstring>
#include <cwctype>

namespace lhp::tracking::hid {

namespace {

std::string Narrow(const wchar_t* w) {
    if (!w || !*w) return {};
    int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    std::string s(n > 0 ? n - 1 : 0, '\0');
    if (n > 1) WideCharToMultiByte(CP_UTF8, 0, w, -1, s.data(), n, nullptr, nullptr);
    return s;
}

int ParseHexAfter(const std::wstring& lowerPath, const wchar_t* key) {
    auto p = lowerPath.find(key);
    if (p == std::wstring::npos) return -1;
    p += wcslen(key);
    int v = 0, digits = 0;
    while (p < lowerPath.size() && iswxdigit(lowerPath[p]) && digits < 2) {
        wchar_t c = lowerPath[p++];
        v = v * 16 + (c <= L'9' ? c - L'0' : c - L'a' + 10);
        ++digits;
    }
    return digits ? v : -1;
}

std::string Win32Error(const char* what) {
    DWORD e = GetLastError();
    char buf[256]{};
    FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, e, MAKELANGID(LANG_ENGLISH, SUBLANG_DEFAULT), buf,
                   sizeof buf, nullptr);
    std::string msg = buf;
    while (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r')) msg.pop_back();
    return std::string(what) + " failed (" + std::to_string(e) + "): " + msg;
}

/// Opens with no access rights: enough for attributes / strings / caps, never fails on devices others hold.
HANDLE OpenForQuery(const std::wstring& path) {
    return CreateFileW(path.c_str(), 0, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
}

}  // namespace

std::vector<HidDeviceInfo> Enumerate(uint16_t vendorId) {
    std::vector<HidDeviceInfo> out;
    GUID guid;
    HidD_GetHidGuid(&guid);
    HDEVINFO set = SetupDiGetClassDevsW(&guid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (set == INVALID_HANDLE_VALUE) return out;
    SP_DEVICE_INTERFACE_DATA ifd{sizeof ifd};
    for (DWORD i = 0; SetupDiEnumDeviceInterfaces(set, nullptr, &guid, i, &ifd); ++i) {
        DWORD need = 0;
        SetupDiGetDeviceInterfaceDetailW(set, &ifd, nullptr, 0, &need, nullptr);
        if (!need) continue;
        std::vector<uint8_t> buf(need);
        auto* detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W*>(buf.data());
        detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
        if (!SetupDiGetDeviceInterfaceDetailW(set, &ifd, detail, need, nullptr, nullptr)) continue;

        HidDeviceInfo info;
        info.path = detail->DevicePath;
        HANDLE h = OpenForQuery(info.path);
        if (h == INVALID_HANDLE_VALUE) continue;
        HIDD_ATTRIBUTES attr{sizeof attr};
        if (HidD_GetAttributes(h, &attr)) {
            info.vid = attr.VendorID;
            info.pid = attr.ProductID;
            info.version = attr.VersionNumber;
        }
        if (vendorId && info.vid != vendorId) {
            CloseHandle(h);
            continue;
        }
        wchar_t s[256];
        if (HidD_GetSerialNumberString(h, s, sizeof s)) info.serial = Narrow(s);
        if (HidD_GetProductString(h, s, sizeof s)) info.product = Narrow(s);
        if (HidD_GetManufacturerString(h, s, sizeof s)) info.manufacturer = Narrow(s);
        PHIDP_PREPARSED_DATA pp = nullptr;
        if (HidD_GetPreparsedData(h, &pp)) {
            HIDP_CAPS caps{};
            if (HidP_GetCaps(pp, &caps) == HIDP_STATUS_SUCCESS) {
                info.usagePage = caps.UsagePage;
                info.usage = caps.Usage;
                info.inputReportLength = caps.InputReportByteLength;
                info.outputReportLength = caps.OutputReportByteLength;
                info.featureReportLength = caps.FeatureReportByteLength;
            }
            HidD_FreePreparsedData(pp);
        }
        CloseHandle(h);
        std::wstring lower = info.path;
        for (auto& c : lower) c = (wchar_t)towlower(c);
        info.interfaceNumber = ParseHexAfter(lower, L"&mi_");
        info.collection = ParseHexAfter(lower, L"&col");
        out.push_back(std::move(info));
    }
    SetupDiDestroyDeviceInfoList(set);
    return out;
}

std::unique_ptr<HidDevice> HidDevice::Open(const HidDeviceInfo& info, Access access, std::string* error) {
    DWORD rights = access == Access::ReadWrite ? (GENERIC_READ | GENERIC_WRITE) : GENERIC_READ;
    HANDLE h = CreateFileW(info.path.c_str(), rights, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        if (error) *error = Win32Error("CreateFile");
        return nullptr;
    }
    // The HID class driver keeps a per-handle ring of input reports (default 32). Bursts of light data
    // overflow that quickly, so ask for the maximum.
    HidD_SetNumInputBuffers(h, 512);

    std::unique_ptr<HidDevice> d(new HidDevice());
    d->info_ = info;
    d->access_ = access;
    d->handle_ = h;
    d->readEvent_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    d->overlapped_ = new OVERLAPPED{};
    d->readBuf_.resize(std::max<size_t>(info.inputReportLength, 65));
    return d;
}

HidDevice::~HidDevice() {
    auto* ov = static_cast<OVERLAPPED*>(overlapped_);
    if (handle_) {
        if (pending_) {
            CancelIoEx((HANDLE)handle_, ov);
            DWORD n = 0;
            GetOverlappedResult((HANDLE)handle_, ov, &n, TRUE);
        }
        CloseHandle((HANDLE)handle_);
    }
    if (readEvent_) CloseHandle((HANDLE)readEvent_);
    delete ov;
}

int HidDevice::Read(uint8_t* buffer, size_t capacity, int timeoutMs) {
    auto* ov = static_cast<OVERLAPPED*>(overlapped_);
    if (cancelled_) return -1;
    if (!pending_) {
        std::memset(ov, 0, sizeof *ov);
        ov->hEvent = (HANDLE)readEvent_;
        ResetEvent((HANDLE)readEvent_);
        DWORD n = 0;
        if (ReadFile((HANDLE)handle_, readBuf_.data(), (DWORD)readBuf_.size(), &n, ov)) {
            size_t c = std::min<size_t>(n, capacity);
            std::memcpy(buffer, readBuf_.data(), c);
            return (int)c;
        }
        if (GetLastError() != ERROR_IO_PENDING) {
            lastError_ = Win32Error("ReadFile");
            return -1;
        }
        pending_ = true;
    }
    // Keep the read pending across timeouts: cancelling would be safe (data stays in the driver ring)
    // but re-issuing every call is wasted work.
    DWORD w = WaitForSingleObject((HANDLE)readEvent_, timeoutMs < 0 ? INFINITE : (DWORD)timeoutMs);
    if (w == WAIT_TIMEOUT) return 0;
    DWORD n = 0;
    pending_ = false;
    if (!GetOverlappedResult((HANDLE)handle_, ov, &n, FALSE)) {
        lastError_ = Win32Error("GetOverlappedResult");
        return -1;
    }
    if (cancelled_) return -1;
    size_t c = std::min<size_t>(n, capacity);
    std::memcpy(buffer, readBuf_.data(), c);
    return (int)c;
}

void HidDevice::Cancel() {
    cancelled_ = true;
    if (handle_) CancelIoEx((HANDLE)handle_, static_cast<OVERLAPPED*>(overlapped_));
    if (readEvent_) SetEvent((HANDLE)readEvent_);
}

bool HidDevice::GetFeature(uint8_t reportId, std::vector<uint8_t>& out) {
    out.assign(std::max<size_t>(info_.featureReportLength, 65), 0);
    out[0] = reportId;
    if (!HidD_GetFeature((HANDLE)handle_, out.data(), (ULONG)out.size())) {
        lastError_ = Win32Error("HidD_GetFeature");
        return false;
    }
    return true;
}

bool HidDevice::SetFeature(const uint8_t* data, size_t length) {
    if (access_ != Access::ReadWrite) {
        lastError_ = "device opened read-only";
        return false;
    }
    // Windows requires the buffer to be exactly the feature report length.
    std::vector<uint8_t> buf(std::max<size_t>(info_.featureReportLength, length), 0);
    std::memcpy(buf.data(), data, length);
    if (!HidD_SetFeature((HANDLE)handle_, buf.data(), (ULONG)buf.size())) {
        lastError_ = Win32Error("HidD_SetFeature");
        return false;
    }
    return true;
}

}  // namespace lhp::tracking::hid

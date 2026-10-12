#include "backend/UsbTopology.h"

#include "util/Text.h"

#include <windows.h>
// clang-format off
#include <initguid.h>
#include <devpkey.h>
#include <setupapi.h>
#include <cfgmgr32.h>
#include <usbiodef.h>
// clang-format on

#include <cstdio>
#include <cwctype>

namespace lhp {

namespace {

std::wstring DevId(DEVINST inst) {
    wchar_t buf[MAX_DEVICE_ID_LEN + 1]{};
    if (CM_Get_Device_IDW(inst, buf, MAX_DEVICE_ID_LEN, 0) != CR_SUCCESS) return {};
    return buf;
}

std::wstring PropString(DEVINST inst, const DEVPROPKEY& key) {
    DEVPROPTYPE type = 0;
    ULONG size = 0;
    if (CM_Get_DevNode_PropertyW(inst, &key, &type, nullptr, &size, 0) != CR_BUFFER_SMALL || size == 0) return {};
    std::vector<BYTE> buf(size);
    if (CM_Get_DevNode_PropertyW(inst, &key, &type, buf.data(), &size, 0) != CR_SUCCESS) return {};
    if (type == DEVPROP_TYPE_STRING || type == DEVPROP_TYPE_STRING_LIST) return std::wstring(reinterpret_cast<wchar_t*>(buf.data()));
    return {};
}

std::wstring PropStringFirstOfList(DEVINST inst, const DEVPROPKEY& key) { return PropString(inst, key); }

std::optional<uint32_t> PropU32(DEVINST inst, const DEVPROPKEY& key) {
    DEVPROPTYPE type = 0;
    uint32_t v = 0;
    ULONG size = sizeof v;
    if (CM_Get_DevNode_PropertyW(inst, &key, &type, reinterpret_cast<PBYTE>(&v), &size, 0) != CR_SUCCESS) return std::nullopt;
    if (type != DEVPROP_TYPE_UINT32) return std::nullopt;
    return v;
}

std::string PropFileTimeDate(DEVINST inst, const DEVPROPKEY& key) {
    DEVPROPTYPE type = 0;
    FILETIME ft{};
    ULONG size = sizeof ft;
    if (CM_Get_DevNode_PropertyW(inst, &key, &type, reinterpret_cast<PBYTE>(&ft), &size, 0) != CR_SUCCESS || type != DEVPROP_TYPE_FILETIME) return {};
    SYSTEMTIME st{};
    FileTimeToSystemTime(&ft, &st);
    char buf[32];
    std::snprintf(buf, sizeof buf, "%04d-%02d-%02d", st.wYear, st.wMonth, st.wDay);
    return buf;
}

std::wstring NameOf(DEVINST inst) {
    auto n = PropString(inst, DEVPKEY_Device_FriendlyName);
    if (n.empty()) n = PropString(inst, DEVPKEY_Device_BusReportedDeviceDesc);
    if (n.empty()) n = PropString(inst, DEVPKEY_Device_DeviceDesc);
    return n;
}

void ParseVidPid(const std::wstring& id, uint16_t& vid, uint16_t& pid) {
    vid = pid = 0;
    auto v = id.find(L"VID_");
    auto p = id.find(L"PID_");
    if (v != std::wstring::npos && v + 8 <= id.size()) vid = (uint16_t)wcstoul(id.substr(v + 4, 4).c_str(), nullptr, 16);
    if (p != std::wstring::npos && p + 8 <= id.size()) pid = (uint16_t)wcstoul(id.substr(p + 4, 4).c_str(), nullptr, 16);
}

UsbNode MakeNode(DEVINST inst) {
    UsbNode n;
    auto id = DevId(inst);
    n.instanceId = ToUtf8(id);
    n.name = ToUtf8(NameOf(inst));
    ParseVidPid(id, n.vid, n.pid);
    return n;
}

void CollectHidDescendants(DEVINST inst, std::vector<std::string>& out, int depth = 0) {
    if (depth > 3) return;
    DEVINST child = 0;
    if (CM_Get_Child(&child, inst, 0) != CR_SUCCESS) return;
    do {
        auto id = DevId(child);
        if (id.rfind(L"HID\\", 0) == 0) out.push_back(ToUtf8(id));
        CollectHidDescendants(child, out, depth + 1);
    } while (CM_Get_Sibling(&child, child, 0) == CR_SUCCESS);
}

bool IsHmdVendor(uint16_t vid, uint16_t pid) {
    if (vid == 0x34A4) return true;                          // Pimax
    if (vid == 0x0BB4) return true;                          // HTC
    if (vid == 0x35BD) return true;                          // Bigscreen
    if (vid == 0x28DE) return pid == 0x2000 || pid >= 0x2300;   // Valve HMD lighthouse board / Index HMD & hub
    return false;
}

std::string VendorName(uint16_t vid) {
    switch (vid) {
        case 0x34A4: return "Pimax";
        case 0x0BB4: return "HTC Vive";
        case 0x35BD: return "Bigscreen";
        case 0x28DE: return "Valve";
        default: return {};
    }
}

std::string VidPidText(uint16_t vid, uint16_t pid) {
    char b[16];
    std::snprintf(b, sizeof b, "%04X:%04X", vid, pid);
    return b;
}

}  // namespace

std::vector<UsbReceiver> EnumerateUsbReceivers() {
    std::vector<UsbReceiver> out;
    HDEVINFO set = SetupDiGetClassDevsW(nullptr, L"USB", nullptr, DIGCF_PRESENT | DIGCF_ALLCLASSES);
    if (set == INVALID_HANDLE_VALUE) return out;
    SP_DEVINFO_DATA dd{sizeof dd};
    for (DWORD i = 0; SetupDiEnumDeviceInfo(set, i, &dd); ++i) {
        wchar_t idBuf[MAX_DEVICE_ID_LEN + 1]{};
        if (!SetupDiGetDeviceInstanceIdW(set, &dd, idBuf, MAX_DEVICE_ID_LEN, nullptr)) continue;
        std::wstring id = idBuf;
        std::wstring up = id;
        for (auto& c : up) c = (wchar_t)towupper(c);
        uint16_t pid = 0;
        if (up.rfind(L"USB\\VID_28DE&PID_2101\\", 0) == 0) pid = 0x2101;
        else if (up.rfind(L"USB\\VID_28DE&PID_2102\\", 0) == 0) pid = 0x2102;
        else continue;  // also skips interface children like ...&MI_00

        UsbReceiver r;
        r.pid = pid;
        r.serial = ToUtf8(id.substr(id.find_last_of(L'\\') + 1));
        UsbInfo& u = r.info;
        u.present = true;
        u.instanceId = ToUtf8(id);
        DEVINST inst = dd.DevInst;
        u.friendlyName = ToUtf8(NameOf(inst));
        u.busDescription = ToUtf8(PropString(inst, DEVPKEY_Device_BusReportedDeviceDesc));
        u.manufacturer = ToUtf8(PropString(inst, DEVPKEY_Device_Manufacturer));
        u.locationInfo = ToUtf8(PropString(inst, DEVPKEY_Device_LocationInfo));
        u.locationPath = ToUtf8(PropStringFirstOfList(inst, DEVPKEY_Device_LocationPaths));
        u.driverVersion = ToUtf8(PropString(inst, DEVPKEY_Device_DriverVersion));
        u.driverDate = PropFileTimeDate(inst, DEVPKEY_Device_DriverDate);
        u.service = ToUtf8(PropString(inst, DEVPKEY_Device_Service));
        u.address = PropU32(inst, DEVPKEY_Device_Address).value_or(0);
        ULONG status = 0, problem = 0;
        if (CM_Get_DevNode_Status(&status, &problem, inst, 0) == CR_SUCCESS) {
            u.hasProblem = (status & DN_HAS_PROBLEM) != 0;
            u.problemCode = problem;
        }
        CollectHidDescendants(inst, u.hidInterfaces);

        // Parent chain up to the root hub / PCI controller.
        DEVINST cur = inst;
        for (int depth = 0; depth < 8; ++depth) {
            DEVINST parent = 0;
            if (CM_Get_Parent(&parent, cur, 0) != CR_SUCCESS) break;
            UsbNode n = MakeNode(parent);
            if (n.instanceId.empty() || StartsWith(n.instanceId, "HTREE")) break;
            u.parents.push_back(n);
            cur = parent;
            if (!StartsWith(n.instanceId, "USB")) break;  // reached controller
        }

        // Siblings on the immediate hub – reveals HMD internal hubs.
        DEVINST parent = 0;
        if (CM_Get_Parent(&parent, inst, 0) == CR_SUCCESS) {
            DEVINST child = 0;
            if (CM_Get_Child(&child, parent, 0) == CR_SUCCESS) {
                do {
                    if (child == inst) continue;
                    u.siblings.push_back(MakeNode(child));
                } while (CM_Get_Sibling(&child, child, 0) == CR_SUCCESS);
            }
        }

        // HMD detection: parent hub itself, or a sibling device, belongs to a known HMD vendor.
        auto check = [&](const UsbNode& n) {
            if (IsHmdVendor(n.vid, n.pid)) {
                u.behindHmd = true;
                if (u.hmdName.empty()) u.hmdName = (n.name.empty() ? VendorName(n.vid) : n.name) + " (" + VidPidText(n.vid, n.pid) + ")";
            }
        };
        if (!u.parents.empty()) check(u.parents[0]);
        for (auto& s : u.siblings) check(s);
        out.push_back(std::move(r));
    }
    SetupDiDestroyDeviceInfoList(set);
    return out;
}

ReceiverKind ClassifyReceiver(uint16_t pid, const UsbInfo& info, std::string_view serial) {
    if (info.behindHmd) return ReceiverKind::HmdBuiltin;
    if (pid == 0x2102) return ReceiverKind::VrRadio;
    if (pid == 0x2101) return ReceiverKind::WatchmanDongle;
    if (Contains(serial, "-")) return ReceiverKind::VrRadio;   // VR Radio serials carry a -XXX suffix
    return ReceiverKind::Unknown;
}

static DWORD CALLBACK HotplugCb(HCMNOTIFICATION, PVOID ctx, CM_NOTIFY_ACTION action, PCM_NOTIFY_EVENT_DATA, DWORD) {
    if (action == CM_NOTIFY_ACTION_DEVICEINTERFACEARRIVAL || action == CM_NOTIFY_ACTION_DEVICEINTERFACEREMOVAL) {
        auto* self = static_cast<UsbHotplug*>(ctx);
        if (self && self->callback) self->callback();
    }
    return ERROR_SUCCESS;
}

UsbHotplug::UsbHotplug(std::function<void()> onChange) : callback(std::move(onChange)) {
    CM_NOTIFY_FILTER f{};
    f.cbSize = sizeof f;
    f.FilterType = CM_NOTIFY_FILTER_TYPE_DEVICEINTERFACE;
    f.u.DeviceInterface.ClassGuid = GUID_DEVINTERFACE_USB_DEVICE;
    HCMNOTIFICATION h = nullptr;
    if (CM_Register_Notification(&f, this, HotplugCb, &h) == CR_SUCCESS) handle_ = h;
}

UsbHotplug::~UsbHotplug() {
    if (handle_) CM_Unregister_Notification((HCMNOTIFICATION)handle_);
}

std::string DescribeUsb(const std::vector<UsbReceiver>& list) {
    std::string s;
    for (auto& r : list) {
        char head[128];
        std::snprintf(head, sizeof head, "%s  pid=%04X  kind=%s\n", r.serial.c_str(), r.pid, ToString(ClassifyReceiver(r.pid, r.info, r.serial)));
        s += head;
        s += "   name: " + r.info.friendlyName + " | loc: " + r.info.locationInfo + " | addr " + std::to_string(r.info.address) + "\n";
        s += "   path: " + r.info.locationPath + "\n";
        s += "   driver: " + r.info.driverVersion + " " + r.info.driverDate + " svc=" + r.info.service + "\n";
        for (auto& h : r.info.hidInterfaces) s += "   hid: " + h + "\n";
        for (auto& p : r.info.parents) s += "   <- " + p.instanceId + " [" + p.name + "]\n";
        for (auto& p : r.info.siblings) s += "   ~~ " + p.instanceId + " [" + p.name + "]\n";
        if (r.info.behindHmd) s += "   HMD: " + r.info.hmdName + "\n";
    }
    return s;
}

}  // namespace lhp

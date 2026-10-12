#pragma once
#include "core/Model.h"

#include <functional>
#include <string>
#include <vector>

namespace lhp {

struct UsbReceiver {
    std::string serial;
    uint16_t pid = 0;
    UsbInfo info;
};

/// Enumerates Valve lighthouse receivers (28DE:2101 Watchman dongle, 28DE:2102 VR Radio)
/// through SetupAPI / CfgMgr32 and works out where they are plugged in.
std::vector<UsbReceiver> EnumerateUsbReceivers();

/// Guess the receiver kind from USB data.
ReceiverKind ClassifyReceiver(uint16_t pid, const UsbInfo& info, std::string_view serial);

/// Registers for USB device arrival/removal notifications. Callback runs on a system thread.
class UsbHotplug {
public:
    explicit UsbHotplug(std::function<void()> onChange);
    ~UsbHotplug();
    UsbHotplug(const UsbHotplug&) = delete;
    UsbHotplug& operator=(const UsbHotplug&) = delete;
    std::function<void()> callback;

private:
    void* handle_ = nullptr;
};

/// Human readable description used by the debug dump.
std::string DescribeUsb(const std::vector<UsbReceiver>& list);

}  // namespace lhp

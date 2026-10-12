#pragma once
// L4 policy: decides whether this program may send commands to devices (active mode).
// Rule: never while SteamVR (vrserver) owns the devices - then we only listen.
#include <string>
#include <vector>

namespace lhp::tracking::service {

struct HostState {
    bool steamVrRunning = false;
    std::vector<std::string> owners;   // processes that may configure devices (vrserver.exe, lighthouse_console.exe ...)
};

HostState ProbeHost();

/// True when active mode is allowed; otherwise `reason` explains why.
bool ActiveModeAllowed(const HostState& h, std::string* reason);

}  // namespace lhp::tracking::service

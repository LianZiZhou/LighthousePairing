#include "tracking/service/HostGuard.h"

#include <windows.h>
#include <tlhelp32.h>

namespace lhp::tracking::service {

HostState ProbeHost() {
    HostState h;
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE) return h;
    PROCESSENTRY32W pe{sizeof pe};
    for (BOOL ok = Process32FirstW(snap, &pe); ok; ok = Process32NextW(snap, &pe)) {
        const wchar_t* n = pe.szExeFile;
        if (_wcsicmp(n, L"vrserver.exe") == 0) {
            h.steamVrRunning = true;
            h.owners.push_back("vrserver.exe");
        } else if (_wcsicmp(n, L"lighthouse_console.exe") == 0) {
            h.owners.push_back("lighthouse_console.exe");
        }
    }
    CloseHandle(snap);
    return h;
}

bool ActiveModeAllowed(const HostState& h, std::string* reason) {
    if (h.steamVrRunning) {
        if (reason) *reason = "SteamVR (vrserver.exe) is running: listening only";
        return false;
    }
    return true;
}

}  // namespace lhp::tracking::service

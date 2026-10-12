#include "backend/SystemInfo.h"

#include "util/Text.h"

#include <windows.h>
#include <shellapi.h>
#include <shlobj.h>
#include <tlhelp32.h>

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace fs = std::filesystem;

namespace lhp::sys {

static std::wstring KnownFolder(REFKNOWNFOLDERID id) {
    PWSTR p = nullptr;
    std::wstring out;
    if (SUCCEEDED(SHGetKnownFolderPath(id, 0, nullptr, &p))) out = p;
    CoTaskMemFree(p);
    return out;
}

bool FileExists(const std::string& path) {
    std::error_code ec;
    return !path.empty() && fs::exists(fs::path(ToWide(path)), ec);
}

std::string ReadTextFile(const std::string& path) {
    std::ifstream f(fs::path(ToWide(path)), std::ios::binary);
    if (!f) return {};
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

bool WriteTextFile(const std::string& path, const std::string& text) {
    std::error_code ec;
    fs::path p(ToWide(path));
    fs::create_directories(p.parent_path(), ec);
    fs::path tmp = p;
    tmp += L".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) return false;
        f << text;
        if (!f) return false;
    }
    fs::rename(tmp, p, ec);
    if (ec) {
        fs::remove(p, ec);
        fs::rename(tmp, p, ec);
    }
    return !ec;
}

std::string FindSteamVrRuntime() {
    std::wstring local = KnownFolder(FOLDERID_LocalAppData);
    std::string text = ReadTextFile(ToUtf8(local + L"\\openvr\\openvrpaths.vrpath"));
    if (!text.empty()) {
        try {
            auto j = nlohmann::json::parse(text);
            if (j.contains("runtime"))
                for (auto& r : j["runtime"]) {
                    std::string p = r.get<std::string>();
                    if (FileExists(p + "\\bin\\win64\\vrserver.exe") || FileExists(p + "\\tools\\lighthouse\\bin\\win64\\lighthouse_console.exe"))
                        return p;
                }
        } catch (...) {
        }
    }
    // Fallback: Steam install path from the registry.
    wchar_t buf[MAX_PATH]{};
    DWORD size = sizeof buf;
    if (RegGetValueW(HKEY_CURRENT_USER, L"Software\\Valve\\Steam", L"SteamPath", RRF_RT_REG_SZ, nullptr, buf, &size) == ERROR_SUCCESS) {
        std::string p = ToUtf8(buf) + "\\steamapps\\common\\SteamVR";
        for (auto& c : p)
            if (c == '/') c = '\\';
        if (FileExists(p)) return p;
    }
    return {};
}

std::string FindSteamConfigDir() {
    std::wstring local = KnownFolder(FOLDERID_LocalAppData);
    std::string text = ReadTextFile(ToUtf8(local + L"\\openvr\\openvrpaths.vrpath"));
    auto norm = [](std::string p) {
        for (auto& c : p)
            if (c == '/') c = '\\';
        while (!p.empty() && p.back() == '\\') p.pop_back();
        return p;
    };
    if (!text.empty()) {
        try {
            auto j = nlohmann::json::parse(text);
            if (j.contains("config"))
                for (auto& r : j["config"]) {
                    std::string p = norm(r.get<std::string>());
                    if (FileExists(p)) return p;
                }
        } catch (...) {
        }
    }
    wchar_t buf[MAX_PATH]{};
    DWORD size = sizeof buf;
    if (RegGetValueW(HKEY_CURRENT_USER, L"Software\\Valve\\Steam", L"SteamPath", RRF_RT_REG_SZ, nullptr, buf, &size) == ERROR_SUCCESS) {
        std::string p = norm(ToUtf8(buf)) + "\\config";
        if (FileExists(p)) return p;
    }
    return {};
}

std::string ReadCachedDeviceConfig(const std::string& deviceSerial) {
    if (deviceSerial.empty()) return {};
    std::string dir = FindSteamConfigDir();
    if (dir.empty()) return {};
    std::string s = deviceSerial;
    for (auto& c : s) c = (char)std::tolower((unsigned char)c);
    return ReadTextFile(dir + "\\lighthouse\\" + s + "\\config.json");
}

std::string ReadSteamVrSettings() {
    std::string dir = FindSteamConfigDir();
    return dir.empty() ? std::string() : ReadTextFile(dir + "\\steamvr.vrsettings");
}

std::string ConsoleTempFile(const std::string& name) {
    auto ascii = [](const std::wstring& w) {
        return std::all_of(w.begin(), w.end(), [](wchar_t c) { return c < 128; });
    };
    wchar_t tmp[MAX_PATH]{};
    GetTempPathW(MAX_PATH, tmp);
    std::wstring dir = tmp;
    if (!ascii(dir)) {
        wchar_t shortp[MAX_PATH]{};
        if (GetShortPathNameW(dir.c_str(), shortp, MAX_PATH) && ascii(shortp)) dir = shortp;
        else {
            dir = KnownFolder(FOLDERID_ProgramData) + L"\\LighthousePairing\\";
            std::error_code ec;
            fs::create_directories(dir, ec);
        }
    }
    if (!dir.empty() && dir.back() != L'\\') dir += L'\\';
    return ToUtf8(dir) + name;
}

bool DeleteFileUtf8(const std::string& path) {
    std::error_code ec;
    return fs::remove(fs::path(ToWide(path)), ec);
}

std::string FindLighthouseConsole(const std::string& overridePath) {
    if (!overridePath.empty() && FileExists(overridePath)) return overridePath;
    std::string rt = FindSteamVrRuntime();
    if (!rt.empty()) {
        std::string p = rt + "\\tools\\lighthouse\\bin\\win64\\lighthouse_console.exe";
        if (FileExists(p)) return p;
    }
    const char* fallbacks[] = {
        "C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR\\tools\\lighthouse\\bin\\win64\\lighthouse_console.exe",
        "C:\\Program Files\\Steam\\steamapps\\common\\SteamVR\\tools\\lighthouse\\bin\\win64\\lighthouse_console.exe",
    };
    for (auto f : fallbacks)
        if (FileExists(f)) return f;
    return {};
}

std::vector<ConflictProcess> FindConflicts(uint32_t ownPid) {
    struct Rule {
        const wchar_t* exe;
        const char* category;
        bool service;
    };
    static const Rule rules[] = {
        {L"vrserver.exe", "SteamVR", false},      {L"vrmonitor.exe", "SteamVR", false},     {L"vrcompositor.exe", "SteamVR", false},
        {L"vrdashboard.exe", "SteamVR", false},   {L"pi_server.exe", "Pimax", false},       {L"PiService.exe", "Pimax", true},
        {L"PiServiceLauncher.exe", "Pimax", true}, {L"PimaxClient.exe", "Pimax", false},    {L"lighthouse_console.exe", "lighthouse_console", false},
        {L"lighthouse_watchman_update.exe", "固件更新", false},
    };
    // Our own child consoles belong to our job – detect them via parent PID.
    std::vector<ConflictProcess> out;
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE) return out;
    PROCESSENTRY32W pe{sizeof pe};
    for (BOOL ok = Process32FirstW(snap, &pe); ok; ok = Process32NextW(snap, &pe)) {
        for (auto& r : rules) {
            if (_wcsicmp(pe.szExeFile, r.exe) != 0) continue;
            if (pe.th32ParentProcessID == ownPid) continue;
            ConflictProcess c;
            c.pid = pe.th32ProcessID;
            c.exe = ToUtf8(pe.szExeFile);
            c.category = r.category;
            c.isService = r.service;
            out.push_back(c);
        }
    }
    CloseHandle(snap);
    return out;
}

std::string KillProcess(uint32_t pid) {
    HANDLE h = OpenProcess(PROCESS_TERMINATE | SYNCHRONIZE, FALSE, pid);
    if (!h) return "无法打开进程 (错误 " + std::to_string(GetLastError()) + ")，可能需要管理员权限";
    std::string err;
    if (!TerminateProcess(h, 1)) err = "结束进程失败 (错误 " + std::to_string(GetLastError()) + ")";
    else WaitForSingleObject(h, 3000);
    CloseHandle(h);
    return err;
}

std::string DataDir() {
    std::wstring local = KnownFolder(FOLDERID_LocalAppData);
    std::wstring dir = local + L"\\LighthousePairing";
    std::error_code ec;
    fs::create_directories(dir, ec);
    return ToUtf8(dir);
}

void OpenInExplorer(const std::string& path) {
    ShellExecuteW(nullptr, L"open", ToWide(path).c_str(), nullptr, nullptr, SW_SHOWNORMAL);
}

void CopyToClipboard(const std::string& text) {
    if (!OpenClipboard(nullptr)) return;
    EmptyClipboard();
    std::wstring w = ToWide(text);
    HGLOBAL h = GlobalAlloc(GMEM_MOVEABLE, (w.size() + 1) * sizeof(wchar_t));
    if (h) {
        memcpy(GlobalLock(h), w.c_str(), (w.size() + 1) * sizeof(wchar_t));
        GlobalUnlock(h);
        SetClipboardData(CF_UNICODETEXT, h);
    }
    CloseClipboard();
}

uint32_t WindowsBuild() {
    using RtlGetVersionFn = LONG(WINAPI*)(PRTL_OSVERSIONINFOW);
    static uint32_t build = [] {
        RTL_OSVERSIONINFOW v{sizeof v};
        if (auto fn = (RtlGetVersionFn)GetProcAddress(GetModuleHandleW(L"ntdll.dll"), "RtlGetVersion")) fn(&v);
        return (uint32_t)v.dwBuildNumber;
    }();
    return build;
}

}  // namespace lhp::sys

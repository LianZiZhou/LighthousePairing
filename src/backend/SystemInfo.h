#pragma once
#include "core/Model.h"

#include <string>
#include <vector>

namespace lhp::sys {

/// SteamVR runtime directory from %LOCALAPPDATA%\openvr\openvrpaths.vrpath (or registry fallback).
std::string FindSteamVrRuntime();

/// Steam config directory (openvrpaths.vrpath "config", e.g. C:\Program Files (x86)\Steam\config).
std::string FindSteamConfigDir();

/// SteamVR's cached copy of a device's JSON config (<config>\lighthouse\<serial>\config.json); empty if none.
std::string ReadCachedDeviceConfig(const std::string& deviceSerial);

/// Where this program keeps device configs it downloaded: <DataDir>\device_configs\<SERIAL>.json
std::string DeviceConfigPath(const std::string& deviceSerial);

/// Contents of <config>\steamvr.vrsettings (tracker role assignments etc.).
std::string ReadSteamVrSettings();

/// A temp file path lighthouse_console can write to: no characters outside ASCII
/// (falls back to the 8.3 short name or %ProgramData%). Quote it, it may contain spaces.
std::string ConsoleTempFile(const std::string& name);
bool DeleteFileUtf8(const std::string& path);

/// Full path to lighthouse_console.exe (empty when not found).
std::string FindLighthouseConsole(const std::string& overridePath);

/// Processes that compete for the lighthouse receivers.
std::vector<ConflictProcess> FindConflicts(uint32_t ownPid);

/// Terminate a process. Returns error text on failure.
std::string KillProcess(uint32_t pid);

/// %LOCALAPPDATA%\LighthousePairing (created on demand).
std::string DataDir();

bool FileExists(const std::string& path);
bool WriteTextFile(const std::string& path, const std::string& text);
std::string ReadTextFile(const std::string& path);

/// Open a folder / file in Explorer.
void OpenInExplorer(const std::string& path);

/// Standard "open file" dialog. `filter` e.g. L"录制文件 (*.lhrec)\0*.lhrec\0". Empty on cancel.
std::string PickOpenFile(const wchar_t* filter, const std::string& initialDir);

/// Copy text to clipboard.
void CopyToClipboard(const std::string& text);

/// Windows build number (e.g. 26200).
uint32_t WindowsBuild();

}  // namespace lhp::sys

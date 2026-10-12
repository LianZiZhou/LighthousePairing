#pragma once
#include "core/Model.h"

#include <string>
#include <vector>

namespace lhp::sys {

/// SteamVR runtime directory from %LOCALAPPDATA%\openvr\openvrpaths.vrpath (or registry fallback).
std::string FindSteamVrRuntime();

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

/// Copy text to clipboard.
void CopyToClipboard(const std::string& text);

/// Windows build number (e.g. 26200).
uint32_t WindowsBuild();

}  // namespace lhp::sys

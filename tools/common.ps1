# Shared helpers for build.ps1 / tools\b.ps1.

function Import-VsDevEnv {
    $vs = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -property installationPath
    if (-not $vs) { throw 'Visual Studio not found' }
    $envDump = cmd /c "`"$vs\VC\Auxiliary\Build\vcvars64.bat`" >nul 2>&1 && set"
    foreach ($line in $envDump) { if ($line -match '^(.*?)=(.*)$') { Set-Item -Path "env:$($matches[1])" -Value $matches[2] } }
    # cl writes its localized messages in the console output code page. Pin it to UTF-8 so the
    # bytes ninja sees are always the same (see Repair-NinjaDepsPrefix).
    cmd /c "chcp 65001 >nul" | Out-Null
    try { [Console]::OutputEncoding = [Text.Encoding]::UTF8 } catch {}
}

# Ninja recognises header dependencies by matching cl's localized /showIncludes prefix byte-for-byte
# (e.g. "注意: 包含文件:" on Chinese Windows). CMake detects it with a different encoding and writes a
# mangled string into rules.ninja, so it never matches: edited headers then do NOT trigger rebuilds and
# stale objects with old struct layouts get linked (hangs / crashes). This runs cl exactly like ninja
# will (same console, same code page), takes the raw prefix bytes and writes them into rules.ninja.
function Repair-NinjaDepsPrefix([string]$BuildDir) {
    $rules = Join-Path $BuildDir 'CMakeFiles\rules.ninja'
    if (-not (Test-Path $rules)) { return }
    $tmp = Join-Path $env:TEMP 'lhp_showinc.cpp'
    Set-Content -Path $tmp -Value '#include <cstddef>' -Encoding ascii
    $psi = New-Object System.Diagnostics.ProcessStartInfo 'cl.exe', "/nologo /Zs /showIncludes `"$tmp`""
    $psi.UseShellExecute = $false; $psi.RedirectStandardOutput = $true
    $p = [Diagnostics.Process]::Start($psi)
    $ms = New-Object IO.MemoryStream
    $p.StandardOutput.BaseStream.CopyTo($ms); $p.WaitForExit()
    $raw = $ms.ToArray()
    $latin = [Text.Encoding]::GetEncoding(28591)                    # byte-transparent string view
    $out = $latin.GetString($raw)
    $line = ($out -split "`r?`n" | Where-Object { $_ -match '[A-Za-z]:\\.*cstddef' } | Select-Object -First 1)
    if (-not $line) { return }
    $prefix = ($line -replace '\s+[A-Za-z]:\\.*$', '')
    if ($prefix -match '^Note:') { return }                         # English cl: CMake already gets it right
    $text = $latin.GetString([IO.File]::ReadAllBytes($rules))
    $fixed = [regex]::Replace($text, '(?m)^msvc_deps_prefix = [^\r\n]*', { "msvc_deps_prefix = $prefix" })
    if ($fixed -ne $text) { [IO.File]::WriteAllBytes($rules, $latin.GetBytes($fixed)) }
}

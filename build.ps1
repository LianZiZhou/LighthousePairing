# Build script.
#   .\build.ps1                 incremental Release build  -> build\Release\LighthousePairing.exe
#   .\build.ps1 -Test           ... and run the unit tests
#   .\build.ps1 -Clean          wipe build\<Config> first
#   .\build.ps1 -Dist           clean, from-scratch, optimised build of ONE self-contained exe -> dist\LighthousePairing.exe
#                               (static CRT, FreeType/ImGui/ImPlot linked in, icon + version embedded; only Windows system DLLs)
param(
    [ValidateSet('Release','Debug')] [string]$Config = 'Release',
    [switch]$Clean,
    [switch]$Test,
    [switch]$Dist
)
$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
. "$root\tools\common.ps1"

# imgui / implot / freetype are git submodules; fetch them on a fresh clone.
if (-not (Test-Path "$root\third_party\imgui\imgui.cpp") -and (Test-Path "$root\.git")) {
    git -C $root submodule update --init --depth 1
    if ($LASTEXITCODE) { exit $LASTEXITCODE }
}
Import-VsDevEnv

function Invoke-Configure([string]$dir, [string[]]$extra) {
    cmake -S $root -B $dir -G Ninja @extra
    if ($LASTEXITCODE) { exit $LASTEXITCODE }
    Repair-NinjaDepsPrefix $dir   # keep header dependency tracking working on non-English MSVC
}

if ($Dist) {
    $build = Join-Path $root 'build\dist'
    if (Test-Path $build) { Remove-Item $build -Recurse -Force }
    Invoke-Configure $build @('-DCMAKE_BUILD_TYPE=Release', '-DLHP_DIST=ON')
    cmake --build $build --target LighthousePairing lhp_tests
    if ($LASTEXITCODE) { exit $LASTEXITCODE }
    & (Join-Path $build 'lhp_tests.exe') --minimal
    if ($LASTEXITCODE) { exit $LASTEXITCODE }
    $out = Join-Path $root 'dist'
    if (Test-Path $out) { Remove-Item $out -Recurse -Force }
    New-Item -ItemType Directory $out | Out-Null
    Copy-Item (Join-Path $build 'LighthousePairing.exe') $out
    $exe = Join-Path $out 'LighthousePairing.exe'
    $deps = (dumpbin /nologo /dependents $exe) -match '^\s+\S+\.dll\s*$' | ForEach-Object { $_.Trim() }
    $system = 'KERNEL32|USER32|GDI32|SHELL32|OLE32|ADVAPI32|SETUPAPI|CFGMGR32|DWMAPI|D3D11|DXGI|D3DCOMPILER_47|DCOMP|IMM32|COMDLG32|WINMM|SHLWAPI|OLEAUT32'
    # api-ms-win-* are Windows API-set contracts (resolved by the OS loader), i.e. system too.
    $foreign = $deps | Where-Object { $_ -notmatch "^($system)\.dll$" -and $_ -notmatch '^api-ms-win-' }
    Write-Host "Dependencies: $($deps -join ', ')"
    if ($foreign) { throw "unexpected non-system DLL dependencies: $($foreign -join ', ')" }
    $info = Get-Item $exe
    Write-Host ("OK -> {0}  ({1:N1} MB, SHA256 {2})" -f $exe, ($info.Length / 1MB), (Get-FileHash $exe).Hash)
    exit 0
}

$build = Join-Path $root "build\$Config"
if ($Clean -and (Test-Path $build)) { Remove-Item $build -Recurse -Force }
Invoke-Configure $build @("-DCMAKE_BUILD_TYPE=$Config")
cmake --build $build
if ($LASTEXITCODE) { exit $LASTEXITCODE }
if ($Test) {
    & (Join-Path $build 'lhp_tests.exe')
    if ($LASTEXITCODE) { exit $LASTEXITCODE }
}
Write-Host "OK -> $build\LighthousePairing.exe"

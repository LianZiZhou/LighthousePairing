# Quick build helper used during development: tools\b.ps1 <builddir-name> [target]
param([string]$Dir = 'Release', [string]$Target = '')
. "$PSScriptRoot\common.ps1"
Import-VsDevEnv
$r = Split-Path $PSScriptRoot -Parent
$b = Join-Path $r "build\$Dir"
$extra = @(); if ($Dir -eq 'core') { $extra += '-DLHP_CORE_ONLY=ON' }
cmake -S $r -B $b -G Ninja -DCMAKE_BUILD_TYPE=Release @extra | Out-Null
Repair-NinjaDepsPrefix $b   # header dependency tracking on Chinese-locale MSVC
$args2 = @('--build', $b); if ($Target) { $args2 += @('--target', $Target) }
$out = & cmake @args2 2>&1
$code = $LASTEXITCODE
$out | Select-String -Pattern 'error C|error LNK|fatal error|FAILED:|warning C4(?!819)' | Where-Object { $_.Line -notmatch 'third_party' } | Select-Object -First 60 | ForEach-Object { $_.Line.Substring(0, [Math]::Min(320, $_.Line.Length)) }
"exit: $code"

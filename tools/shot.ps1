# Render a screenshot via the app's --screenshot flag and convert it to a small PNG.
# Usage: tools\shot.ps1 -Name overview -AppArgs '--demo --tab 0' [-MaxWidth 1280]
param([string]$Name = 'shot', [string]$AppArgs = '--demo', [int]$MaxWidth = 1280, [int]$Frames = 150, [string]$BuildDir = 'Release')
$root = Split-Path $PSScriptRoot -Parent
$exe = Join-Path $root "build\$BuildDir\LighthousePairing.exe"
$dir = Join-Path $root 'build\shots'
New-Item -ItemType Directory -Force $dir | Out-Null
$bmp = Join-Path $dir "$Name.bmp"
$png = Join-Path $dir "$Name.png"
Remove-Item $bmp, $png -ErrorAction SilentlyContinue
$argList = "$AppArgs --screenshot `"$bmp`" --frames $Frames"
$p = Start-Process -FilePath $exe -ArgumentList $argList -PassThru
if (-not $p.WaitForExit(60000)) { $p.Kill(); throw 'app did not exit' }
if (-not (Test-Path $bmp)) { throw "no screenshot produced (exit $($p.ExitCode))" }
$py = 'C:\Users\lianzhou\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe'
& $py -c "from PIL import Image; im=Image.open(r'$bmp'); w,h=im.size; s=min(1.0,$MaxWidth/w); im=im.resize((int(w*s),int(h*s)), Image.LANCZOS) if s<1 else im; im.save(r'$png', optimize=True); import os; print(im.size, os.path.getsize(r'$png'))"
Remove-Item $bmp
$png

"""Probe lighthouse_console interactively (read-only commands only).

Usage: python probe_console.py [serial] [cmd ...]
Prints every stdout chunk with a relative timestamp so we can see buffering behaviour.
"""
import subprocess, sys, threading, time

LC = r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\tools\lighthouse\bin\win64\lighthouse_console.exe"
SAFE = {"help", "version", "deviceinfo", "dongleinfo", "battery", "imustats", "usbstats",
        "errors", "period", "sensorcheck", "userdata", "userdataalloc", "sleep", "clear", "quit"}

args = sys.argv[1:]
serial = None
if args and not args[0] in SAFE and not args[0].startswith("serial"):
    serial = args.pop(0)
cmds = args or ["version", "deviceinfo", "dongleinfo"]
for c in cmds:
    if c.split()[0] not in SAFE and not c.startswith("serial"):
        sys.exit(f"refusing non read-only command: {c}")

argv = [LC] + (["/serial", serial] if serial else [])
t0 = time.time()
p = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0)

def reader():
    while True:
        b = p.stdout.read1(65536) if hasattr(p.stdout, "read1") else p.stdout.read(1)
        if not b:
            break
        txt = b.decode("utf-8", "replace")
        for ln in txt.splitlines(True):
            sys.stdout.write(f"[{time.time()-t0:7.3f}] {ln!r}\n")
        sys.stdout.flush()

th = threading.Thread(target=reader, daemon=True); th.start()
time.sleep(2.5)
for c in cmds:
    sys.stdout.write(f"[{time.time()-t0:7.3f}] >>> {c}\n"); sys.stdout.flush()
    p.stdin.write((c + "\n").encode()); p.stdin.flush()
    time.sleep(2.0)
p.stdin.write(b"quit\n"); p.stdin.flush()
try:
    p.wait(8)
except subprocess.TimeoutExpired:
    p.kill()
th.join(2)

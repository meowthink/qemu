#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Expect-style driver for the Apollo DN3000 MD PROM / Domain-OS install.
# Usage: drive.py <logfile> <script.txt>
# script.txt lines:  EXPECT <substring>   or   SEND <text>   or  SENDLN <text>
#                    WAIT <seconds>        or   TIMEOUT <seconds>
import socket, subprocess, sys, time, os, select

LOG = sys.argv[1]
SCRIPT = sys.argv[2]
SOCK = "/tmp/apollo-serial.sock"
BOOT = "/tmp/tapes/019593-001.CRTG_STD_SFW_BOOT_1-REV.A.ct"
DISK = os.environ.get("DISK", "/tmp/domainos.awd")
TAPE = os.environ.get("TAPE", BOOT)
MEM  = os.environ.get("MEM", "8M")
EXTRA = os.environ.get("EXTRA", "")

if os.path.exists(SOCK):
    os.unlink(SOCK)

qemu = [
    "/tmp/apollo-build/qemu-system-m68k", "-M", "apollo-dn3000",
    "-m", MEM, "-icount", "shift=6",
    "-bios", "/workspace/files/apollo/3000_BOOT_8475_7.bin",
    "-drive", f"file={DISK},format=raw,if=mtd,index=0",
    "-drive", f"file={TAPE},format=raw,if=mtd,index=1,readonly=on",
    "-serial", f"unix:{SOCK},server,nowait",
    "-display", "none",
]
if EXTRA:
    qemu += EXTRA.split()

logf = open(LOG, "wb")
proc = subprocess.Popen(qemu, stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL)
time.sleep(1.0)
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
for _ in range(50):
    try:
        s.connect(SOCK); break
    except OSError:
        time.sleep(0.2)
s.setblocking(False)

buf = bytearray()
default_timeout = 60.0

def pump(seconds):
    end = time.time() + seconds
    while time.time() < end:
        r,_,_ = select.select([s], [], [], 0.2)
        if r:
            try:
                d = s.recv(4096)
            except BlockingIOError:
                continue
            if d:
                buf.extend(d); logf.write(d); logf.flush()
                sys.stdout.write(d.decode('latin1')); sys.stdout.flush()

search_from = 0
def expect(sub, timeout):
    global search_from
    end = time.time() + timeout
    while time.time() < end:
        idx = bytes(buf).find(sub.encode(), search_from)
        if idx >= 0:
            search_from = idx + len(sub)
            return True
        pump(0.2)
    return False

try:
    for line in open(SCRIPT):
        line = line.rstrip("\n")
        if not line or line.startswith("#"):
            continue
        op, _, arg = line.partition(" ")
        if op == "TIMEOUT":
            default_timeout = float(arg)
        elif op == "WAIT":
            pump(float(arg))
        elif op == "EXPECT":
            ok = expect(arg, default_timeout)
            print(f"\n[[expect {arg!r}: {'OK' if ok else 'TIMEOUT'}]]")
            if not ok:
                print("[[ABORT: prompt not seen]]"); break
        elif op == "SEND":
            s.sendall(arg.encode())
        elif op == "SENDLN":
            s.sendall(arg.encode() + b"\r")
        elif op == "CR":
            s.sendall(b"\r")
    pump(2.0)
finally:
    try: s.close()
    except: pass
    proc.terminate()
    try: proc.wait(5)
    except: proc.kill()
    logf.close()

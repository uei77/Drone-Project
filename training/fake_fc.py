"""Bench test without a drone: pretends to be the flight controller on a USB-TTL adapter.

Wiring: adapter RX <- ESP32 GPIO14, adapter TX -> ESP32 GPIO15, GND <-> GND.
usage: python fake_fc.py COM5 [--mode 2] [--ack]
  - sends $TEL at 10 Hz (hovering at 15 m, mission AUTO) so $HB turns OK
  - sends $CMD,MODE,<mode> every 2 s (0 off, 1 shadow, 2 active)
  - prints every line from the ESP32 with checksum status
  - with --ack answers every $EVT with $ACK, like the real FC script would
needs: pip install pyserial
"""
import argparse, time
import serial


def cs(p):
    c = 0
    for ch in p.encode():
        c ^= ch
    return c


def frame(p):
    return f"${p}*{cs(p):02X}\n".encode()


ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--mode", type=int, default=1)
ap.add_argument("--ack", action="store_true")
a = ap.parse_args()
s = serial.Serial(a.port, 115200, timeout=0.02)
t0 = time.time(); last_tel = last_cmd = 0.0; buf = b""
while True:
    now = time.time()
    if now - last_tel >= 0.1:
        last_tel = now
        ms = int((now - t0) * 1000)
        s.write(frame(f"TEL,{ms},15.00,90.0,0.00,0.00,2.00,0.0,3,10,95.0,1"))
    if now - last_cmd >= 2.0:
        last_cmd = now
        s.write(frame(f"CMD,MODE,{a.mode}"))
    buf += s.read(256)
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        txt = line.decode(errors="replace").strip()
        ok = False
        if txt.startswith("$") and "*" in txt:
            body, c = txt[1:].rsplit("*", 1)
            ok = c.upper() == f"{cs(body):02X}"
            if ok and a.ack and body.startswith("EVT,") and not body.endswith(",done"):
                s.write(frame("ACK," + body.split(",")[1]))
        if not txt.startswith("$HB") or int(now) % 5 == 0:   # thin out heartbeats
            print(f"{now - t0:8.2f}  {'ok ' if ok else 'BAD'}  {txt}")

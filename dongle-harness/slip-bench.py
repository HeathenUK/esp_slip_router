#!/usr/bin/env python3
"""
SLIP path microbenchmark. Sends pre-built SLIP frames at max pyserial
rate, measures bytes-per-second the dongle ACCEPTS (via blocking Serial
backpressure). No Mac echo server; isolates the dongle's intake path.

Tests two targets:
  A. Dongle's own SLIP IP (192.168.240.1) -- ICMP echo. Path: USB RX ->
     SLIP decode -> lwIP local delivery -> ICMP -> reply -> SLIP encode ->
     USB TX. Skips WiFi/NAPT entirely.
  B. A sentinel IP (192.168.240.99) -- ICMP echo. Path: USB RX -> SLIP
     decode -> lwIP local delivery -> no responder -> drop. Skips reply
     too -- pure intake measurement.

Plus payload-size sweep (64B, 256B, 1024B, 1400B) to separate
per-packet overhead from per-byte cost.
"""
import os, sys, time, struct, serial

PORT = os.environ.get("DONGLE", "/dev/cu.usbmodemF412FA44AC4C1")
SLIP_END, SLIP_ESC, SLIP_ESC_END, SLIP_ESC_ESC = 0xC0, 0xDB, 0xDC, 0xDD

def slip_enc(d):
    out = bytearray([SLIP_END])
    for b in d:
        if   b == SLIP_END: out += bytes([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC: out += bytes([SLIP_ESC, SLIP_ESC_ESC])
        else: out.append(b)
    out.append(SLIP_END); return bytes(out)

def csum(d):
    s = 0
    if len(d) & 1: d += b'\x00'
    for i in range(0, len(d), 2):
        s += (d[i] << 8) | d[i+1]; s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF

def icmp_pkt(src, dst, ident, seq, payload):
    icmp = struct.pack('!BBHHH', 8, 0, 0, ident, seq) + payload
    c = csum(icmp)
    icmp = struct.pack('!BBHHH', 8, 0, c, ident, seq) + payload
    total = 20 + len(icmp)
    h = struct.pack('!BBHHHBBH4s4s', 0x45, 0, total, 0xBEEF, 0, 64, 1, 0,
                    bytes(map(int, src.split('.'))), bytes(map(int, dst.split('.'))))
    h = struct.pack('!BBHHHBBH4s4s', 0x45, 0, total, 0xBEEF, 0, 64, 1, csum(h),
                    bytes(map(int, src.split('.'))), bytes(map(int, dst.split('.'))))
    return h + icmp

def bench(p, label, dst, payload_size, duration):
    payload = bytes((i & 0xFF for i in range(payload_size)))
    frame = slip_enc(icmp_pkt("192.168.240.2", dst, 0xABCD, 1, payload))
    t0 = time.time(); end = t0 + duration
    sent = 0; pkts = 0
    p.reset_input_buffer()
    while time.time() < end:
        n = p.write(frame)
        if n: sent += n; pkts += 1
    p.flush()
    dt = time.time() - t0
    print(f"  {label:30s} payload={payload_size:5d}B  {pkts:6d} pkts in {dt:.2f}s = {sent/dt/1024:6.1f} KB/s wire,  {pkts*payload_size/dt/1024:6.1f} KB/s payload")

def main():
    p = serial.Serial(PORT, 115200, timeout=0.1); time.sleep(1)
    p.reset_input_buffer()
    p.write(b"AT\rAT$MODE=SLIP\r"); p.flush(); time.sleep(0.7); p.reset_input_buffer()

    DUR = 3
    print(f"[+] each test runs {DUR}s")
    print(f"[A] dongle local (192.168.240.1) — ICMP RTT path")
    for sz in (64, 256, 1024, 1400):
        bench(p, "local-RTT", "192.168.240.1", sz, DUR)
    print(f"[B] sentinel (192.168.240.99) — RX/decode only, no reply")
    for sz in (64, 256, 1024, 1400):
        bench(p, "sentinel-RX-only", "192.168.240.99", sz, DUR)

    p.write(b"\xC0MODE=MODEM\xC0"); p.flush(); time.sleep(0.6)
    p.reset_input_buffer()
    p.write(b"AT\rAT$STATS\r"); p.flush(); time.sleep(0.5)
    print()
    print(p.read(8192).decode('latin-1','replace'))
    p.close()

if __name__ == "__main__":
    sys.exit(main())

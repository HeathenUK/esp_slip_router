#!/usr/bin/env python3
"""
hayes-throughput.py -- Mac-side equivalent of dos-throughput/httpget.c.

Drives the dongle in HAYES (modem) mode over CDC, GETs a URL, and times the
BODY phase to report KB/s -- the same measurement httpget.c prints on DOS,
but with the Mac reading the CDC stream directly (no FOSSIL/CHUSB in the
path). Use it as a consumer-side baseline to compare against the DOS number.

Reads as aggressively as pyserial allows (tight in_waiting loop, no sleeps)
so the number reflects the transport, not Python pacing.

Usage:  DONGLE=/dev/cu.usbmodem... ./hayes-throughput.py URL [--expect N]
"""
import sys, os, time, argparse, urllib.parse, hashlib
import serial

DONGLE = os.environ.get("DONGLE", "/dev/cu.usbmodemF412FA44AC4C1")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("url")
    ap.add_argument("--expect", type=int, default=0, help="expected body length (sanity)")
    a = ap.parse_args()
    u = urllib.parse.urlparse(a.url)
    host, port, path = u.hostname, (u.port or 80), (u.path or "/") + (("?"+u.query) if u.query else "")

    p = serial.Serial(DONGLE, 115200, timeout=2)
    time.sleep(0.3); p.reset_input_buffer()
    for c in (b"\rATZ\r", b"ATE0\r", b"ATNET0\r", b"AT$LECHO=OFF\r"):
        p.write(c); time.sleep(0.3); p.read(p.in_waiting or 1)

    print(f"[dial] ATDT {host}:{port}")
    p.write(f"ATDT {host}:{port}\r".encode())
    buf = b""; dl = time.time() + 25
    while time.time() < dl:
        n = p.read(p.in_waiting or 1)
        if n: buf += n
        if b"CONNECT" in buf: time.sleep(0.2); buf += p.read(p.in_waiting or 1); break
        if b"NO CARRIER" in buf or b"ERROR" in buf: sys.exit(f"dial failed: {buf!r}")
    after = buf.split(b"CONNECT", 1)[1].lstrip(b"\r\n")

    req = (f"GET {path} HTTP/1.0\r\nHost: {host}\r\n"
           f"User-Agent: hayes-throughput/1\r\nConnection: close\r\n\r\n").encode()
    p.write(req)

    body = after
    t_first = t_last = None
    dl = time.time() + 180
    while time.time() < dl:
        w = p.in_waiting
        chunk = p.read(w if w else 1)
        if chunk:
            if t_first is None: t_first = time.time()
            t_last = time.time(); body += chunk
            if body.rstrip().endswith(b"NO CARRIER"): break
    nc = body.rfind(b"\r\nNO CARRIER")
    if nc >= 0: body = body[:nc]
    p.close()

    sep = body.find(b"\r\n\r\n")
    headers, payload = body[:sep], body[sep+4:]
    elapsed = (t_last - t_first) if (t_first and t_last) else 0
    print(f"[http] {headers.split(chr(13).encode(),1)[0].decode('latin1','replace')}")
    print(f"body bytes:  {len(payload)}" + (f"  (expected {a.expect})" if a.expect else ""))
    print(f"md5:         {hashlib.md5(payload).hexdigest()}")
    print(f"body time:   {elapsed:.2f} s")
    if elapsed > 0:
        print(f"THROUGHPUT:  {len(payload)/elapsed/1024:.1f} KB/s")

if __name__ == "__main__":
    main()

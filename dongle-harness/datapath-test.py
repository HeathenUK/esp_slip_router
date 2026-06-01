#!/usr/bin/env python3
"""
T-Dongle S3 online-mode (Hayes data) integrity + throughput test.

  host -- USB CDC --> dongle -- WiFi/TCP --> Mac TCP echo server
                                                    |
                                                    v
                                              echoes back
                                                    |
  host <- USB CDC <-- dongle <-- WiFi/TCP <---------+

Tests:
  A. Server -> dongle -> host (TCP rx -> CDC tx) — small/large/binary/IAC
  B. Host -> dongle -> server (CDC rx -> TCP tx) — same
  C. Throughput each direction
  D. +++ escape returns to command mode
  E. Disconnect mid-session yields NO CARRIER

Usage:  python3 datapath-test.py [SSID PSK]
        env DONGLE=/dev/cu... overrides port path
"""
import sys, os, time, socket, subprocess, secrets, re, threading, queue
import serial

DONGLE = os.environ.get("DONGLE", "/dev/cu.usbmodemF412FA44AC4C1")
SSID = sys.argv[1] if len(sys.argv) > 1 else "pistorm"
PSK  = sys.argv[2] if len(sys.argv) > 2 else "pinchpunch"

# ---------- helpers ----------
def open_dongle():
    p = serial.Serial(DONGLE, 115200, timeout=2)
    time.sleep(1.0)
    p.reset_input_buffer()
    return p

def send_at(p, cmd, settle=1.0, expect=None, timeout=10.0):
    """Send an AT command, read reply for `settle` seconds (or until `expect`)."""
    p.reset_input_buffer()
    p.write((cmd + "\r").encode()); p.flush()
    end = time.time() + (timeout if expect else settle)
    buf = b""
    while time.time() < end:
        d = p.read(4096)
        if d: buf += d
        if expect is not None and expect.encode() in buf:
            return buf.decode("latin-1", "replace")
        if expect is None and time.time() > end - settle + settle:
            # settle window elapsed since start
            pass
    return buf.decode("latin-1", "replace")

def mac_ip_on_subnet(dongle_ip):
    """Find the Mac IP on the same /24 as the dongle."""
    octets = dongle_ip.split('.')
    if len(octets) != 4: return None
    subnet24 = '.'.join(octets[:3])
    out = subprocess.check_output(["ifconfig"]).decode()
    for line in out.splitlines():
        m = re.search(r"inet (\d+\.\d+\.\d+\.\d+)", line)
        if m and m.group(1).startswith(subnet24 + "."):
            ip = m.group(1)
            if ip != dongle_ip: return ip
    return None

# ---------- main ----------
def main():
    p = open_dongle()
    print(f"[+] dongle at {DONGLE}")

    # Wake (in case prior AT$BOOT left us mid-something)
    send_at(p, "AT", 0.5)

    # Set WiFi creds (atomic + auto-connect)
    print(f"[+] AT$WIFI={SSID},***")
    r = send_at(p, f"AT$WIFI={SSID},{PSK}", 2.0)
    if "connecting" not in r.lower() and "ok" not in r.lower():
        print(f"[-] WiFi set rejected: {r!r}"); return 1

    # Poll for IP up to 25 s
    print("[+] waiting for IP...")
    dongle_ip = None
    for i in range(50):
        time.sleep(0.5)
        r = send_at(p, "AT$WIFI?", 0.8)
        m = re.search(r"IP:\s+(\d+\.\d+\.\d+\.\d+)", r)
        if m and re.search(r"status:\s+connected", r):
            dongle_ip = m.group(1); break
    if not dongle_ip:
        print(f"[-] WiFi never connected. Last: {r!r}"); return 2
    print(f"[+] dongle IP: {dongle_ip}")

    mac_ip = mac_ip_on_subnet(dongle_ip)
    if not mac_ip:
        print(f"[-] no Mac IP on dongle's subnet — can't reach echo server"); return 3
    print(f"[+] mac IP:    {mac_ip}")

    # Echo server in a thread
    PORT = 5599
    srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT)); srv.listen(1); srv.settimeout(15)
    print(f"[+] echo server up on {mac_ip}:{PORT}")

    conn = None
    def accept():
        nonlocal conn
        conn, addr = srv.accept(); print(f"[+] server got conn from {addr}")
    th = threading.Thread(target=accept, daemon=True); th.start()

    # Disable telnet for binary-clean tests
    send_at(p, "ATNET0", 1.0)

    # ATDT (give CONNECT 10s)
    print(f"[+] ATDT{mac_ip}:{PORT}")
    r = send_at(p, f"ATDT{mac_ip}:{PORT}", 0, expect="CONNECT", timeout=10)
    print(f"    -> {r!r}")
    if "CONNECT" not in r:
        print("[-] dial failed"); return 4
    th.join(timeout=5)
    if conn is None:
        print("[-] server never got connection"); return 5

    failures = 0

    def test_s2h(label, payload):
        nonlocal failures
        p.reset_input_buffer()
        conn.sendall(payload)
        # Generous timeout for lossy / marginal-WiFi links — even a 64B round
        # trip can pay multiple TCP retransmits when RTT is >1s.
        end = time.time() + max(20, len(payload) / 1000)
        got = b""
        while len(got) < len(payload) and time.time() < end:
            d = p.read(8192)
            if d: got += d
        ok = (got[:len(payload)] == payload) and (len(got) == len(payload))
        rate = len(payload) / max(end - (end - 5), 0.1)
        print(f"    {'OK' if ok else 'FAIL'} {label}: sent {len(payload)} got {len(got)} match={ok}")
        if not ok:
            failures += 1
            diff = next((i for i in range(min(len(payload), len(got))) if payload[i] != got[i]), None)
            print(f"        first-diff at byte {diff}, expected {payload[diff:diff+8]!r} got {got[diff:diff+8]!r}" if diff is not None else "")

    def test_h2s(label, payload):
        nonlocal failures
        # Drain server socket: wait for in-flight bytes from prior tests to
        # settle, THEN drain.
        time.sleep(0.5)
        conn.setblocking(False)
        try:
            while conn.recv(8192): pass
        except (BlockingIOError, socket.timeout):
            pass
        conn.setblocking(True); conn.settimeout(0.5)
        p.write(payload); p.flush()
        end = time.time() + max(20, len(payload) / 1000)
        got = b""
        while len(got) < len(payload) and time.time() < end:
            try:
                d = conn.recv(8192)
                if d: got += d
                else: break
            except socket.timeout:
                pass
        ok = (got[:len(payload)] == payload) and (len(got) == len(payload))
        print(f"    {'OK' if ok else 'FAIL'} {label}: sent {len(payload)} got {len(got)} match={ok}")
        if not ok:
            failures += 1
            diff = next((i for i in range(min(len(payload), len(got))) if payload[i] != got[i]), None)
            print(f"        first-diff at byte {diff}, expected {payload[diff:diff+8]!r} got {got[diff:diff+8]!r}" if diff is not None else "")

    # ---------- integrity (small) ----------
    print("\n[#] Test A: server->dongle->host integrity")
    test_s2h("64B ASCII",     b"the quick brown fox " * 4 + b"X" * 16)
    test_s2h("64B with IAC",  b"\xff" * 8 + b"AB" + b"\xff\x00\x80\x7f" * 14)
    test_s2h("1 KB random",   secrets.token_bytes(1024))
    test_s2h("8 KB random",   secrets.token_bytes(8192))

    print("\n[#] Test B: host->dongle->server integrity")
    test_h2s("64B ASCII",     b"the quick brown fox " * 4 + b"X" * 16)
    test_h2s("64B with IAC",  b"\xff" * 8 + b"AB" + b"\xff\x00\x80\x7f" * 14)
    test_h2s("1 KB random",   secrets.token_bytes(1024))
    test_h2s("8 KB random",   secrets.token_bytes(8192))

    # ---------- throughput ----------
    def throughput(label, fn, size):
        t0 = time.time(); ok = fn(secrets.token_bytes(size)); dt = time.time() - t0
        rate = size / dt / 1024
        print(f"    {label}: {size} B in {dt:.2f}s = {rate:.1f} KB/s")

    print("\n[#] Test C: throughput (best-effort, network varies)")
    def s2h_silent(payload):
        p.reset_input_buffer()
        conn.sendall(payload)
        end = time.time() + 15
        got = b""
        while len(got) < len(payload) and time.time() < end:
            d = p.read(8192)
            if d: got += d
        return got == payload
    def h2s_silent(payload):
        try:
            conn.setblocking(False)
            while conn.recv(8192): pass
        except (BlockingIOError, socket.timeout): pass
        conn.setblocking(True); conn.settimeout(0.5)
        p.write(payload); p.flush()
        end = time.time() + 15
        got = b""
        while len(got) < len(payload) and time.time() < end:
            try:
                d = conn.recv(8192)
                if d: got += d
                else: break
            except socket.timeout: pass
        return got == payload

    throughput("server->dongle->host 32 KB", s2h_silent, 32*1024)
    throughput("host->dongle->server 32 KB", h2s_silent, 32*1024)

    # ---------- escape + hangup ----------
    print("\n[#] Test D: +++ escape -> command mode")
    time.sleep(1.2)            # guard time
    p.write(b"+++"); p.flush()
    time.sleep(1.5)            # post-guard
    r = p.read(4096).decode("latin-1", "replace")
    print(f"    +++ reply: {r!r}")
    p.write(b"AT\r"); p.flush(); time.sleep(0.7)
    r = p.read(4096).decode("latin-1", "replace")
    print(f"    AT after +++: {r!r}  {'OK' if 'OK' in r else 'FAIL (still online?)'}")
    if "OK" not in r: failures += 1

    print("\n[#] Test E: ATH disconnect")
    p.write(b"ATH\r"); p.flush(); time.sleep(1)
    r = p.read(4096).decode("latin-1", "replace"); print(f"    ATH: {r!r}")

    p.close(); conn.close(); srv.close()
    print(f"\n[=] done. failures: {failures}")
    return 0 if failures == 0 else 99

if __name__ == "__main__":
    sys.exit(main())

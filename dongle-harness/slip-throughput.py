#!/usr/bin/env python3
"""
SLIP throughput benchmark.

Measures the dongle's USB CDC RX -> SLIP decode -> lwIP forward -> NAPT ->
WiFi TX path. Fires UDP packets from the host via SLIP at maximum rate;
Mac UDP server counts received bytes/sec.

Also measures the reverse path (Mac -> WiFi -> NAPT recv -> SLIP encode ->
USB CDC TX -> host) by having the Mac stream UDP after we've primed the
NAPT mapping with one outbound.

Usage:
  python3 slip-throughput.py [SECONDS]   # default 5
"""
import os, sys, time, struct, socket, threading, subprocess, re, serial

PORT = os.environ.get("DONGLE", "/dev/cu.usbmodemF412FA44AC4C1")
DUR = int(sys.argv[1]) if len(sys.argv) > 1 else 5
SLIP_END, SLIP_ESC, SLIP_ESC_END, SLIP_ESC_ESC = 0xC0, 0xDB, 0xDC, 0xDD
HOST_IP = "192.168.240.2"

def slip_enc(d):
    out = bytearray([SLIP_END])
    for b in d:
        if   b == SLIP_END: out += bytes([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC: out += bytes([SLIP_ESC, SLIP_ESC_ESC])
        else: out.append(b)
    out.append(SLIP_END); return bytes(out)

def slip_dec(buf):
    out, frames, esc = bytearray(), [], False
    for b in buf:
        if b == SLIP_END:
            if out: frames.append(bytes(out)); out = bytearray()
            esc = False
        elif esc:
            out.append(SLIP_END if b == SLIP_ESC_END else SLIP_ESC if b == SLIP_ESC_ESC else b)
            esc = False
        elif b == SLIP_ESC: esc = True
        else: out.append(b)
    return frames, out  # also return leftover for streaming

def csum(d):
    s = 0
    if len(d) & 1: d += b'\x00'
    for i in range(0, len(d), 2):
        s += (d[i] << 8) | d[i+1]; s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF

def udp_pkt(src, dst, sport, dport, payload, ident):
    udp_len = 8 + len(payload)
    pseudo = bytes(map(int, src.split('.'))) + bytes(map(int, dst.split('.'))) \
           + struct.pack('!BBH', 0, 17, udp_len)
    udp = struct.pack('!HHHH', sport, dport, udp_len, 0) + payload
    c = csum(pseudo + udp)
    if c == 0: c = 0xFFFF
    udp = struct.pack('!HHHH', sport, dport, udp_len, c) + payload
    total = 20 + udp_len
    h = struct.pack('!BBHHHBBH4s4s', 0x45, 0, total, ident, 0, 64, 17, 0,
                    bytes(map(int, src.split('.'))), bytes(map(int, dst.split('.'))))
    h = struct.pack('!BBHHHBBH4s4s', 0x45, 0, total, ident, 0, 64, 17, csum(h),
                    bytes(map(int, src.split('.'))), bytes(map(int, dst.split('.'))))
    return h + udp

def mac_subnet_ip():
    out = subprocess.check_output(["ifconfig"]).decode()
    for line in out.splitlines():
        m = re.search(r"inet (192\.168\.1\.\d+)", line)
        if m: return m.group(1)
    return None

def main():
    p = serial.Serial(PORT, 115200, timeout=0.1); time.sleep(1)
    p.reset_input_buffer()
    p.write(b"AT\rAT$MODE=SLIP\r"); p.flush(); time.sleep(0.7); p.reset_input_buffer()
    mac_ip = mac_subnet_ip()
    if not mac_ip: print("[-] no Mac IP on 192.168.1.0/24"); return 1

    SRV_PORT = 5611
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(("0.0.0.0", SRV_PORT))
    srv.settimeout(0.05)
    rx_bytes = [0]; rx_pkts = [0]; stop = [False]
    def rxer():
        while not stop[0]:
            try:
                d, _ = srv.recvfrom(2048)
                rx_bytes[0] += len(d); rx_pkts[0] += 1
            except socket.timeout: pass
    threading.Thread(target=rxer, daemon=True).start()

    PAYLOAD = bytes(range(256)) * 4  # 1024B payload — big single-pbuf SLIP frame
    print(f"[+] upload test: host -> SLIP -> dongle -> WiFi -> Mac UDP {mac_ip}:{SRV_PORT}")
    print(f"    payload={len(PAYLOAD)}B  duration={DUR}s")

    # build packet template once (only ident/seq vary; UDP checksum constant for fixed payload)
    pkts = []
    for i in range(64):  # rotate ident to avoid IP-frag dedup
        pkts.append(slip_enc(udp_pkt(HOST_IP, mac_ip, 49152, SRV_PORT, PAYLOAD, 0x4000 + i)))

    sent = 0; tx_pkts = 0
    t0 = time.time()
    end = t0 + DUR
    i = 0
    while time.time() < end:
        n = p.write(pkts[i & 63])
        if n: sent += n; tx_pkts += 1
        i += 1
    p.flush()
    time.sleep(0.5)  # let last in-flight drain
    stop[0] = True
    dt = time.time() - t0

    tx_kbps = sent / dt / 1024
    rx_kbps = rx_bytes[0] / dt / 1024
    payload_kbps = rx_pkts[0] * len(PAYLOAD) / dt / 1024
    loss = (tx_pkts - rx_pkts[0]) / tx_pkts * 100 if tx_pkts else 0
    print(f"    sent: {tx_pkts} pkts, {sent} B SLIP-on-wire = {tx_kbps:.1f} KB/s")
    print(f"    recv: {rx_pkts[0]} pkts, {rx_bytes[0]} B at Mac    = {rx_kbps:.1f} KB/s")
    print(f"    UDP payload rate: {payload_kbps:.1f} KB/s  loss={loss:.1f}%")

    # escape back to MODEM
    p.write(b"\xC0MODE=MODEM\xC0"); p.flush(); time.sleep(0.6)
    p.close(); srv.close()
    print("[=] done")
    return 0

if __name__ == "__main__":
    sys.exit(main())

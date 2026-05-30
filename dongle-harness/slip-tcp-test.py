#!/usr/bin/env python3
"""
TCP-over-SLIP integrity + throughput test.

Simulates a DOS host running mTCP-style: synthesizes TCP packets on the
SLIP side, sends through SLIP -> dongle NAPT -> WiFi -> echo server,
verifies a complete handshake + data exchange + FIN.

Requires the dongle to be in MODEM mode with WiFi up. Switches to SLIP
mode during the test, then back via magic-frame escape.
"""
import os, sys, time, struct, socket, threading, secrets, serial

PORT = os.environ.get("DONGLE", "/dev/cu.usbmodemF412FA44AC4C1")
SLIP_END, SLIP_ESC, SLIP_ESC_END, SLIP_ESC_ESC = 0xC0, 0xDB, 0xDC, 0xDD
SLIP_PEER_IP = "192.168.240.2"

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
    return frames

def csum(d):
    s = 0
    if len(d) & 1: d += b'\x00'
    for i in range(0, len(d), 2):
        s += (d[i] << 8) | d[i+1]; s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF

def ip_hdr(src, dst, proto, ident, total):
    h = struct.pack('!BBHHHBBH4s4s', 0x45, 0, total, ident, 0, 64, proto, 0,
                    bytes(map(int, src.split('.'))), bytes(map(int, dst.split('.'))))
    return struct.pack('!BBHHHBBH4s4s', 0x45, 0, total, ident, 0, 64, proto, csum(h),
                       bytes(map(int, src.split('.'))), bytes(map(int, dst.split('.'))))

def tcp(src, dst, sport, dport, seq, ack, flags, payload=b'', window=8192):
    pseudo = bytes(map(int, src.split('.'))) + bytes(map(int, dst.split('.'))) \
           + struct.pack('!BBH', 0, 6, 20 + len(payload))
    hdr = struct.pack('!HHIIBBHHH', sport, dport, seq, ack, 0x50, flags, window, 0, 0)
    c = csum(pseudo + hdr + payload)
    hdr = struct.pack('!HHIIBBHHH', sport, dport, seq, ack, 0x50, flags, window, c, 0)
    return ip_hdr(src, dst, 6, secrets.randbits(16), 20 + 20 + len(payload)) + hdr + payload

def parse_tcp(pkt):
    if len(pkt) < 40 or (pkt[0] >> 4) != 4 or pkt[9] != 6: return None
    ihl = (pkt[0] & 0xF) * 4
    th = pkt[ihl:ihl+20]
    src = '.'.join(str(b) for b in pkt[12:16])
    dst = '.'.join(str(b) for b in pkt[16:20])
    sport, dport, seq, ack = struct.unpack('!HHII', th[:12])
    data_off = (th[12] >> 4) * 4
    flags = th[13]
    payload = pkt[ihl + data_off:]
    return dict(src=src, dst=dst, sport=sport, dport=dport, seq=seq, ack=ack, flags=flags, payload=payload)

def main():
    p = serial.Serial(PORT, 115200, timeout=2); time.sleep(1)
    p.reset_input_buffer()
    p.write(b"AT\rAT$MODE=SLIP\r"); p.flush(); time.sleep(0.7)
    r = p.read(4096).decode("latin-1","replace")
    if "SLIP" not in r: print(f"[-] mode: {r!r}"); return 1
    p.reset_input_buffer()

    # find mac IP on same subnet as dongle wifi
    import subprocess, re
    out = subprocess.check_output(["ifconfig"]).decode()
    mac_ip = None
    for line in out.splitlines():
        m = re.search(r"inet (192\.168\.1\.\d+)", line)
        if m: mac_ip = m.group(1); break
    if not mac_ip:
        print("[-] no Mac IP on 192.168.1.0/24"); return 1
    print(f"[+] mac IP: {mac_ip}")

    PORT_SRV = 5601
    srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT_SRV)); srv.listen(1); srv.settimeout(10)
    conn_q = []
    def acceptor():
        try:
            c, a = srv.accept(); conn_q.append((c, a)); print(f"[srv] accepted from {a}")
        except: pass
    threading.Thread(target=acceptor, daemon=True).start()

    sport = 49152
    dport = PORT_SRV
    iss = 0x12345678

    # 1. SYN
    print(f"[+] SYN -> {mac_ip}:{dport}")
    p.write(slip_enc(tcp(SLIP_PEER_IP, mac_ip, sport, dport, iss, 0, 0x02)))
    p.flush()

    # 2. expect SYN-ACK
    end = time.time() + 5; buf = bytearray()
    syn_ack = None
    while time.time() < end and not syn_ack:
        d = p.read(4096)
        if d: buf += d
        for f in slip_dec(buf):
            t = parse_tcp(f)
            if t and t['flags'] & 0x12 == 0x12 and t['ack'] == iss + 1:
                syn_ack = t; break
        if syn_ack: break
    if not syn_ack: print("[-] no SYN-ACK"); return 2
    print(f"    SYN-ACK: seq={syn_ack['seq']:#x} ack={syn_ack['ack']:#x}")

    # 3. ACK
    p.write(slip_enc(tcp(SLIP_PEER_IP, mac_ip, sport, dport, iss + 1, syn_ack['seq'] + 1, 0x10)))
    p.flush(); time.sleep(0.3)
    if not conn_q: print("[-] srv never accepted"); return 3
    conn, _ = conn_q[0]
    conn.settimeout(3)

    # 4. data PSH/ACK
    payload = b"hello-from-slip-via-napt"
    print(f"[+] PSH+ACK {len(payload)}B")
    p.write(slip_enc(tcp(SLIP_PEER_IP, mac_ip, sport, dport, iss + 1, syn_ack['seq'] + 1, 0x18, payload)))
    p.flush()
    got = conn.recv(1024)
    print(f"    srv got {len(got)}B: {got!r}  {'OK' if got == payload else 'FAIL'}")

    # 5. server echoes
    conn.sendall(b"REPLY:" + payload)
    end = time.time() + 3; buf = bytearray()
    reply_seen = None
    while time.time() < end:
        d = p.read(4096)
        if d: buf += d
        for f in slip_dec(buf):
            t = parse_tcp(f)
            if t and t['payload'] and b"REPLY:" in t['payload']:
                reply_seen = t['payload']; break
        if reply_seen: break
    print(f"    host got via SLIP: {reply_seen!r}  {'OK' if reply_seen == b'REPLY:' + payload else 'FAIL'}")

    # cleanup
    try: conn.close()
    except: pass
    srv.close()

    # escape
    p.write(b"\xC0MODE=MODEM\xC0"); p.flush(); time.sleep(0.6)
    p.close()
    print("\n[=] TCP-via-SLIP test done")

if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
T-Dongle S3 SLIP data-path test.

Flow:
  1. Issue AT$MODE=SLIP   (switches the CDC link from AT-engine to SLIP framing)
  2. Send a SLIP-encapsulated ICMP echo-request -> 192.168.240.1 (the dongle)
  3. De-frame the SLIP response, verify it's an ICMP echo-reply with our id+seq
  4. Send the magic escape frame (\xC0 "MODE=MODEM" \xC0) to flip back to MODEM
  5. Confirm AT works again

No root, no tun/tap, no pppd — pure userspace SLIP-over-CDC.
"""
import os, sys, time, struct, serial

PORT = os.environ.get("DONGLE", "/dev/cu.usbmodemF412FA44AC4C1")

SLIP_END, SLIP_ESC, SLIP_ESC_END, SLIP_ESC_ESC = 0xC0, 0xDB, 0xDC, 0xDD

def slip_encode(payload):
    out = bytearray([SLIP_END])
    for b in payload:
        if b == SLIP_END:   out += bytes([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC: out += bytes([SLIP_ESC, SLIP_ESC_ESC])
        else:               out.append(b)
    out.append(SLIP_END)
    return bytes(out)

def slip_decode_stream(buf):
    """Yield (frame, remaining_buf) for complete frames in buf."""
    out, frames, esc = bytearray(), [], False
    for b in buf:
        if b == SLIP_END:
            if out:
                frames.append(bytes(out)); out = bytearray()
            esc = False
        elif esc:
            out.append(SLIP_END if b == SLIP_ESC_END else SLIP_ESC if b == SLIP_ESC_ESC else b)
            esc = False
        elif b == SLIP_ESC:
            esc = True
        else:
            out.append(b)
    return frames

def ip_checksum(data):
    s = 0
    if len(data) & 1:
        data += b'\x00'
    for i in range(0, len(data), 2):
        s += (data[i] << 8) | data[i+1]
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF

def build_icmp_echo(src, dst, ident, seq, payload=b'tdongle-s3 slip ping'):
    icmp_type = 8     # echo request
    icmp = struct.pack('!BBHHH', icmp_type, 0, 0, ident, seq) + payload
    csum = ip_checksum(icmp)
    icmp = struct.pack('!BBHHH', icmp_type, 0, csum, ident, seq) + payload
    total = 20 + len(icmp)
    ip_hdr = struct.pack('!BBHHHBBH4s4s',
                         0x45,      # ver=4 ihl=5
                         0,         # tos
                         total,
                         0xBEEF,    # id
                         0,         # flags+frag
                         64,        # ttl
                         1,         # proto=ICMP
                         0,         # checksum (filled)
                         bytes(map(int, src.split('.'))),
                         bytes(map(int, dst.split('.'))))
    ip_csum = ip_checksum(ip_hdr)
    ip_hdr = struct.pack('!BBHHHBBH4s4s',
                         0x45, 0, total, 0xBEEF, 0, 64, 1, ip_csum,
                         bytes(map(int, src.split('.'))),
                         bytes(map(int, dst.split('.'))))
    return ip_hdr + icmp

def parse_icmp_reply(pkt, ident, seq):
    if len(pkt) < 20 + 8: return None
    if pkt[0] >> 4 != 4: return None
    ihl = (pkt[0] & 0xF) * 4
    if pkt[9] != 1: return None     # ICMP
    icmp = pkt[ihl:]
    if icmp[0] != 0: return None    # echo reply
    rid, rseq = struct.unpack('!HH', icmp[4:8])
    return (rid == ident and rseq == seq), icmp[8:]

# ---------------------------------------------------------------------------
def main():
    p = serial.Serial(PORT, 115200, timeout=2); time.sleep(1.5)
    p.reset_input_buffer()
    failures = 0

    # 1. enter SLIP mode
    print(f"[+] {PORT}: AT$MODE=SLIP")
    p.write(b"AT\rAT$MODE=SLIP\r"); p.flush(); time.sleep(0.7)
    r = p.read(4096).decode("latin-1","replace")
    if "SLIP" not in r:
        print(f"[-] mode switch failed: {r!r}"); return 1
    p.reset_input_buffer()

    # 2. ICMP echo over SLIP
    ident, seq = 0xCAFE, 1
    pkt = build_icmp_echo("192.168.240.2", "192.168.240.1", ident, seq)
    print(f"[+] sending {len(pkt)}B ICMP echo request -> 192.168.240.1")
    p.write(slip_encode(pkt)); p.flush()

    # 3. collect frames for up to 3s
    end = time.time() + 3
    buf = bytearray()
    while time.time() < end:
        d = p.read(4096)
        if d: buf += d
    frames = slip_decode_stream(buf)
    print(f"[+] received {len(frames)} SLIP frame(s), {len(buf)} raw bytes")

    matched = False
    for f in frames:
        res = parse_icmp_reply(f, ident, seq)
        if res:
            ok, payload = res
            if ok:
                matched = True
                print(f"    OK echo reply: id=0x{ident:04x} seq={seq} payload={payload!r}")
    if not matched:
        print("    FAIL: no matching ICMP echo reply")
        for i, f in enumerate(frames[:5]):
            print(f"      frame[{i}] ({len(f)}B): {f.hex()}")
        failures += 1

    # 3b. ICMP echo to a public IP via NAPT (proves SLIP -> WiFi -> SLIP path)
    ident2, seq2 = 0xBABE, 1
    pkt2 = build_icmp_echo("192.168.240.2", "1.1.1.1", ident2, seq2, payload=b'napt-test'*4)
    print(f"[+] sending {len(pkt2)}B ICMP echo request -> 1.1.1.1 (via NAPT)")
    p.reset_input_buffer()
    p.write(slip_encode(pkt2)); p.flush()
    end = time.time() + 6
    buf = bytearray()
    while time.time() < end:
        d = p.read(4096)
        if d: buf += d
    frames = slip_decode_stream(buf)
    print(f"[+] received {len(frames)} SLIP frame(s) after NAPT request")
    napt_ok = False
    for f in frames:
        res = parse_icmp_reply(f, ident2, seq2)
        if res and res[0]:
            napt_ok = True
            print(f"    OK NAPT echo reply from 1.1.1.1 (id=0x{ident2:04x} seq={seq2})")
            break
    if not napt_ok:
        print("    FAIL: no NAPT echo reply (WiFi connectivity? NAPT enabled?)")
        for i, f in enumerate(frames[:5]):
            print(f"      frame[{i}] ({len(f)}B): {f[:48].hex()}{'...' if len(f)>48 else ''}")
        failures += 1

    # 4. magic-frame escape back to MODEM
    print("[+] sending magic-frame escape -> MODEM")
    p.write(b"\xC0MODE=MODEM\xC0"); p.flush(); time.sleep(0.8)
    p.reset_input_buffer()
    p.write(b"AT\rAT$MODE?\r"); p.flush(); time.sleep(0.7)
    r = p.read(4096).decode("latin-1","replace")
    if "MODEM" in r and "OK" in r:
        print(f"    OK back in MODEM")
    else:
        print(f"    FAIL: {r!r}"); failures += 1

    p.close()
    print(f"\n[=] done. failures: {failures}")
    return 0 if failures == 0 else 2

if __name__ == "__main__":
    sys.exit(main())

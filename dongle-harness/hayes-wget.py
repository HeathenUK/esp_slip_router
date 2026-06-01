#!/usr/bin/env python3
"""
hayes-wget.py -- HTTP GET via the dongle in Hayes mode, then verify MD5
                 against an independent curl fetch of the same URL.

  host -> /dev/cu.usbmodem* (CDC) -> ATDT host:80 -> GET ... -> bytes back
  same host -> curl http://host/path -> reference bytes

  md5(bytes_via_dongle) == md5(bytes_via_curl)  ->  PASS

Usage:  ./hayes-wget.py [URL] [-o outfile.bin]
        Default URL: http://www.brutman.com/mTCP/download/1k.bin
"""
import sys, os, time, re, hashlib, subprocess, argparse, urllib.parse
import serial

DEFAULT_URL = "http://www.brutman.com/mTCP/download/1k.bin"
DONGLE = os.environ.get("DONGLE", "/dev/cu.usbmodemF412FA44AC4C1")

def split_http_url(url):
    u = urllib.parse.urlparse(url)
    if u.scheme not in ("http", ""):
        raise ValueError(f"only http:// supported (got {u.scheme!r})")
    host = u.hostname
    port = u.port or 80
    path = u.path or "/"
    if u.query: path += "?" + u.query
    return host, port, path

def hayes_get(url):
    host, port, path = split_http_url(url)
    print(f"[hayes] open {DONGLE}")
    p = serial.Serial(DONGLE, 115200, timeout=2)
    time.sleep(0.3); p.reset_input_buffer()

    # Configure for HTTP: raw byte stream, no AT echo, no local echo.
    p.write(b'\rATZ\r');         time.sleep(0.5); p.read(p.in_waiting or 1)
    p.write(b'ATE0\r');          time.sleep(0.3); p.read(p.in_waiting or 1)
    # ATNET0: disable telnet IAC processing in BOTH directions. The dongle's
    # NVT CR->CRLF expansion would turn our HTTP request line's "\r\n" into
    # "\r\n\n" on the wire, which servers reject with 400 Bad Request. HTTP
    # also doesn't contain 0xFF bytes so IAC stripping is unneeded on RX.
    p.write(b'ATNET0\r');        time.sleep(0.3); p.read(p.in_waiting or 1)
    # Disable data-mode local echo. Default AUTO would echo our typed GET
    # request back at us and contaminate the response stream.
    p.write(b'AT$LECHO=OFF\r');  time.sleep(0.3); p.read(p.in_waiting or 1)

    print(f"[hayes] ATDT {host}:{port}")
    p.write(f"ATDT {host}:{port}\r".encode())

    # Wait for CONNECT (timeout 25 s -- covers DNS + TCP + CPR + telnet IAC).
    deadline = time.time() + 25
    buf = b''
    while time.time() < deadline:
        n = p.read(p.in_waiting or 1)
        if n: buf += n
        if b"CONNECT" in buf:
            # consume any trailing CR/LF after CONNECT
            time.sleep(0.2)
            buf += p.read(p.in_waiting or 1)
            break
        if b"NO CARRIER" in buf or b"ERROR" in buf:
            p.close()
            raise RuntimeError(f"dial failed: {buf.decode('latin-1','replace')!r}")
        time.sleep(0.05)
    else:
        p.close()
        raise RuntimeError("dial timeout (no CONNECT)")

    # Discard everything we've buffered up to and including CONNECT\r\n so
    # the response stream starts clean. The dongle also injects an ESC[6n
    # CPR probe + reply somewhere in here; both get tossed.
    pre, _, _ = buf.partition(b"CONNECT")
    # everything AFTER "CONNECT\r\n" is server data
    after_connect = buf.split(b"CONNECT", 1)[1].lstrip(b"\r\n")

    # Send the HTTP/1.0 request.
    req = (f"GET {path} HTTP/1.0\r\n"
           f"Host: {host}\r\n"
           f"User-Agent: hayes-wget/1\r\n"
           f"Connection: close\r\n\r\n").encode()
    print(f"[hayes] GET {path}")
    p.write(req)

    # Read until peer closes (manifests as NO CARRIER from the dongle).
    body = after_connect
    deadline = time.time() + 60
    while time.time() < deadline:
        n = p.read(p.in_waiting or 1)
        if n:
            body += n
            if body.rstrip().endswith(b"NO CARRIER"):
                break
        time.sleep(0.02)

    # Strip the dongle's "\r\nNO CARRIER\r\n" suffix exactly -- a greedy
    # rstrip on \r\n would also eat the payload's own trailing newline
    # (HTML files often end with one, and the MD5 would mismatch by one
    # byte vs an independent curl fetch).
    nc = body.rfind(b"\r\nNO CARRIER")
    if nc >= 0:
        body = body[:nc]

    p.close()

    # Split HTTP headers from body.
    sep = body.find(b"\r\n\r\n")
    if sep < 0:
        raise RuntimeError(f"no HTTP header terminator in response (got {len(body)} bytes)")
    headers, payload = body[:sep], body[sep+4:]
    status_line = headers.split(b"\r\n", 1)[0]
    print(f"[hayes] status: {status_line.decode('latin-1','replace')}")
    if b" 200 " not in status_line:
        raise RuntimeError(f"non-200 status: {status_line!r}")

    # Honor Content-Length if present (defensive truncation; some servers
    # send a stray \r\n at EOF).
    m = re.search(rb"Content-Length:\s*(\d+)", headers, re.IGNORECASE)
    if m:
        clen = int(m.group(1))
        if len(payload) > clen:
            payload = payload[:clen]
        if len(payload) < clen:
            print(f"[hayes] WARN: got {len(payload)} bytes, Content-Length says {clen}")
    return payload

def curl_get(url):
    print(f"[curl ] GET {url}")
    r = subprocess.run(["curl", "-sf", url], capture_output=True, timeout=30)
    r.check_returncode()
    return r.stdout

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("url", nargs="?", default=DEFAULT_URL)
    ap.add_argument("-o", "--out", default=None,
                    help="save Hayes-fetched bytes to this file")
    args = ap.parse_args()

    hayes_bytes = hayes_get(args.url)
    curl_bytes  = curl_get(args.url)

    hayes_md5 = hashlib.md5(hayes_bytes).hexdigest()
    curl_md5  = hashlib.md5(curl_bytes).hexdigest()

    print()
    print(f"hayes bytes: {len(hayes_bytes):>8}   md5: {hayes_md5}")
    print(f"curl  bytes: {len(curl_bytes):>8}   md5: {curl_md5}")
    print()

    if args.out:
        with open(args.out, "wb") as f: f.write(hayes_bytes)
        print(f"saved Hayes payload to {args.out}")

    if hayes_md5 == curl_md5 and len(hayes_bytes) == len(curl_bytes):
        print("PASS: md5 match, byte counts match")
        return 0
    print("FAIL: mismatch")
    if len(hayes_bytes) != len(curl_bytes):
        print(f"       length delta: hayes={len(hayes_bytes)} vs curl={len(curl_bytes)}")
    # Show the first differing byte for diagnosis.
    n = min(len(hayes_bytes), len(curl_bytes))
    for i in range(n):
        if hayes_bytes[i] != curl_bytes[i]:
            print(f"       first diff at byte {i}: hayes=0x{hayes_bytes[i]:02x} curl=0x{curl_bytes[i]:02x}")
            print(f"       hayes context: {hayes_bytes[max(0,i-8):i+8]!r}")
            print(f"       curl  context: {curl_bytes[max(0,i-8):i+8]!r}")
            break
    return 1

if __name__ == "__main__":
    sys.exit(main())

# USBNET — Packet-driver TSR over USB-CDC

Design and implementation plan. Replaces SLIP as the high-throughput data
path between the Pocket386 (DOS) and the T-Dongle S3, while preserving the
existing Hayes/AT mode for byte-streaming use cases.

**Hypothesis** (not a target): end-to-end mTCP throughput on a 386SX-40
lands somewhere in 60-100 KB/s — 10-15x today's SLIP at 6 KB/s, 1.5-2.5x
today's Hayes at 42 KB/s. The wall after this work is mTCP's per-segment
TCP cost on the 386, not framing, not FOSSIL, not USB. Phase 8
measurements confirm or refute.

Document history: initial draft committed, then red-teamed and rewritten
(this revision). Red-team notes in commit history.

## Goals & non-goals

### Goals
- DOS runs **unmodified** mTCP via a new TSR (`USBNET.COM`) implementing
  the FTP Software Packet Driver Spec v1.11 on INT 60h.
- Heavy stuff that *can* be offloaded *is* offloaded to the dongle: ARP,
  DNS, IP routing/NAPT, link framing. DOS does TCP/UDP itself.
- Hayes/AT mode stays as a peer mode; mode switching is in-band.
- **CHUSB is not modified.** Dongle continues to enumerate as plain
  USB-CDC-ACM. TSR uses CHUSB's existing bulk-I/O API.
- No FOSSIL on the packet-driver data path.

### Non-goals
- We do *not* offload TCP. mTCP runs TCP on the 386 because that's what
  an mTCP-compatible packet driver requires.
- We do not accelerate Hayes.
- We do not retain SLIP as a third mode once PACKET works.

## Architecture

```
+-----------------------+         USB-CDC bulk         +------------------------+
| DOS 386SX             |  <------------------------>  | ESP32-S3 dongle        |
|                       |                              |                        |
|  mTCP / app           |                              |  lwIP netif "usbnet0"  |
|     |  INT 60h        |                              |   (point-to-point)     |
|  USBNET.COM (TSR) <---+-- IP frames over CDC ----->  |   IP_FORWARD + NAPT    |
|     |                 |  (length-prefixed framing,   |   ARP responder        |
|  CHUSB (existing) ----+   one frame type byte)       |   DNS forwarder        |
|     |                 |                              |   Hayes/AT mode        |
|  USB host HW          |                              |  WiFi STA              |
+-----------------------+                              +------------------------+
```

## Where the throughput gains come from

Honest accounting of which parts of the design contribute what:

| Source of win                          | Magnitude     |
|----------------------------------------|---------------|
| No FOSSIL on the data path             | **Large**     |
| Bulk-aligned CHUSB reads (no per-byte) | **Large**     |
| ARP/DHCP/DNS offload to dongle         | Small         |
| Length-prefixed framing (no stuffing)  | Small (~5%)   |
| L4 (TCP) offload                       | **Zero** — we don't do this |

The framing choice is for simplicity, not throughput. The big win is
removing FOSSIL and letting CHUSB do its bulk-shaped thing.

## Transport: framing on the CDC bulk pipe

Length-prefixed framing. No byte-stuffing.

```
+------+------+------+----------------------+
| 0xA5 | LEN  | TYPE |     PAYLOAD          |
| 1 B  | 2 B  | 1 B  |     LEN bytes        |
+------+------+------+----------------------+
```

- `0xA5` preamble: resync only; USB bulk is atomic but cheap insurance.
- `LEN` little-endian, ≤ 1500.
- `TYPE`:
  - `0x01` IP datagram (bulk traffic, both directions)
  - `0x10` MODE_SWITCH (payload = "MODEM" | "PACKET")
  - `0x11` PING / heartbeat (diagnostic)
  - `0xFF` reserved
- No link-layer CRC — USB has one already.

Mode-switch protocol:
- `MODEM → PACKET`: TSR writes literal AT command `AT$MODE=PACKET\r`
  to CDC. Dongle replies `\r\nOK\r\n` then **only after the OK has fully
  drained from the CDC TX FIFO** flips to PACKET mode. (Existing
  `cmd_dollar_mode` in `modem.c` already orders `r_ok()` before the
  mode flip; verify and document.)
- `PACKET → MODEM`: TSR sends framed MODE_SWITCH("MODEM"). Dongle flips
  to MODEM mode after draining current TX. No reply expected.

## DOS-side TSR (USBNET.COM)

Resident, ~10-15 KB code + RX ring buffer.

### Load sequence
1. Locate dongle via CHUSB (CDC-ACM with our VID/PID, first match).
2. Write `AT$MODE=PACKET\r` to CDC, read back until seeing `\r\nOK\r\n`
   or timeout (5s).
3. Switch own RX parser to framed mode.
4. Send framed PING, expect framed PING reply (handshake confirmation).
5. Hook INT 60h with packet driver dispatch.
6. Install RX servicing hook (see "Polling cadence" below — depends on
   Phase 0 outcome).
7. Print `usbnet: ready, mac=02:11:22:33:44:55, mtu=1500`. Resident.

### Packet Driver INT 60h interface
- `driver_info` — class=1 (Ethernet), type=our assigned constant,
  number=0, name="USBNET"
- `access_type`, `release_type`, `send_pkt`, `get_address`
- `reset_interface`, `as_send_pkt`, `get_parameters`, `terminate`
- Optional diagnostic subfunctions (0xF0+): expose packet/byte
  counters, ring depth, drop count.

### Synthetic MAC addresses
Hardcoded, locally-administered:
- Our MAC (DOS side): `02:11:22:33:44:55`
- Gateway MAC (dongle side): `02:11:22:33:44:01`

mTCP only ever sees these via ARP responses from the dongle and via
its own `get_address` query. No collision risk on the point-to-point
link.

### TX path (`send_pkt`)
- Caller hands a complete Ethernet frame in ES:DI, length CX.
- Read EtherType (offset 12). Only `0x0800` (IPv4) and `0x0806` (ARP)
  are expected; others dropped.
- Strip 14-byte ETH header.
- Wrap L3 body in framing (`A5 LEN 01 <bytes>`).
- Push via CHUSB bulk write.
- Drain RX opportunistically before returning.

### RX path
- Drain function reads non-blockingly from CHUSB into ring, parses
  frames:
  - `0x01` IP → prepend synthetic ETH header (`dst=our_mac,
    src=gw_mac, type=0x0800`) → deliver via registered handler.
  - `0x10` MODE_SWITCH → log + ignore.
  - Other → log + drop.
- Drain is called from: (a) timer hook (see polling cadence), (b)
  inside `send_pkt` after TX.

### Polling cadence (depends on Phase 0)
Decision tree resolved by Phase 0:
- **If CHUSB exposes an interrupt callback for endpoint completion**:
  register callback, drain there. Best case.
- **If CHUSB requires polling but has a fast non-blocking poll-for-data
  call**: hook INT 1Ch (~18 Hz, too slow on its own) **plus** poll
  inside `send_pkt`. Adequacy depends on application traffic pattern.
- **If neither works**: design needs revisiting. Possible alternatives:
  reprogram PIT to ~1 kHz (breaks DOS time-of-day unless we chain-call
  original handler), use INT 09h keyboard hook (cursed but possible),
  or accept polling at INT 1Ch rate as a hard ceiling (~25 KB/s with
  ring oversizing — about 4x SLIP, still useful).

### Unload (`/U`)
- Send framed MODE_SWITCH("MODEM").
- Unhook INT 60h and timer/interrupt hook.
- Release resident memory.

### Resident footprint estimate
- RX ring: 16 × 1536 = 24 KB
- TX scratch: 1.5 KB
- Code + state: ~6-10 KB
- **Total: ~32-36 KB**. Acceptable if loaded high (UMB); concerning
  in conventional RAM. Measure early in Phase 5; can shrink ring if
  needed.

## Dongle-side firmware changes

Most of this exists for SLIP and is being adapted.

1. **lwIP netif `usbnet0`** — replaces SLIP netif.
2. **Framer/deframer** — replaces SLIP encode/decode on the CDC path
   when in PACKET mode.
3. **ARP responder on usbnet0** — answers any ARP for `gw_ip` or any
   address mTCP probes locally. Pure synthesis with the hardcoded
   gateway MAC.
4. **DNS forwarder** — UDP socket bound on `192.168.240.1:53`. Resolves
   via `getaddrinfo` over WiFi, synthesizes DNS response.
5. **NAPT** — on usbnet0 netif (the "inverted" IDF API we already
   handle).
6. **Mode-switching** — extend existing MODEM/SLIP/PACKET state, retire
   SLIP after Phase 10.
7. **`/net-stats` HTTP endpoint** — replaces `/slip-stats`. Counters:
   pkts/bytes in/out, framer errors, ARP requests answered, DNS
   queries forwarded, NAPT translations.

## MTCP.CFG changes

```diff
  PACKETINT 0x60
  HOSTNAME bolo
  IPADDR 192.168.240.2
  NETMASK 255.255.255.0
  GATEWAY 192.168.240.1
- NAMESERVER 1.1.1.1
+ NAMESERVER 192.168.240.1
  MTU 1500
```

Plus our new TSR loaded before mTCP at boot.

## PROFILE.EXE changes

Out of scope for the firmware/TSR work but tracked separately:
- Phase 1 (raw FOSSIL SLIP UDP throughput): retire — measures something
  no longer in the data path.
- Phase 2 (Hayes HTTP): unchanged.
- Phase 3 (mTCP HTTP via LAN): unchanged BAT-level invocation; now goes
  through USBNET.COM instead of FOSSLIP.EXE.
- Phase 4 (mTCP HTTP via Internet + DNS): same change as Phase 3.

A separate PROFILE rebuild + redeploy after Phase 8.

## What we don't do (and why)

- **No custom CDC interface/descriptor/class.** Plain CDC-ACM.
- **No DOS-side ARP intercept.** ARP traffic is negligible; dongle
  answers over USB.
- **No DOS-side TCP offload API.** Out of scope.
- **No link-layer CRC.** USB has one.
- **No byte-stuffing.** Length-prefixed framing.
- **No `/UNIT:n` arg.** One dongle.
- **No `AT$MAC` round-trip.** Hardcoded synthetic MACs.

## Implementation plan

### Phase 0 — CHUSB feasibility (HARD GATE)

**Output**: A 1-page note in `dongle-harness/CHUSB_API_NOTES.md`
answering, with concrete CHUSB function/macro names where they exist:

1. Is there an **interrupt callback** API for endpoint completion?
2. If not, is there a **non-blocking poll** for "data available on
   bulk IN endpoint"?
3. What's the **bulk read/write API** for arbitrary lengths up to MTU?
4. Can **MSC and CDC traffic coexist** on the same composite device,
   or does CHUSB serialize them?
5. Are there **any hooks for high-rate servicing** other than INT 28h
   idle?

**Go criterion**: At least (1) or (2) is true and (4) is yes.
**No-go**: design needs revisiting before further work.

Source: read CHUSB headers/docs (user-supplied; do not reverse-engineer
without permission). Ask user pointed questions if docs are silent.

### Phase 1 — Dongle: PACKET mode skeleton

New netif, framer, mode-switch wiring (NVS, `/mode`, `AT$MODE`, GPIO0).
SLIP stays parallel for safety.

**Test from Mac**: Python script opens CDC, sends `AT$MODE=PACKET\r`,
reads `OK`, sends a framed PING, expects framed PING reply.

### Phase 2 — Dongle: ARP responder + DNS forwarder

Both bound to usbnet0. Mac script fires framed ARP request packets
(0x0806 EtherType wrapped in our framing? or — given we already
dropped DOS-side ARP — just direct IP traffic plus DNS queries) and
DNS queries, sees synthesized replies.

**Note**: re-evaluate during Phase 1 whether ARP should travel framed
as-is (Ethernet frame in payload) or whether we should run a thin
"pseudo-Ethernet" layer where the TSR-side strips/re-adds ETH headers.
The latter saves 14 bytes per frame on the wire; the former is simpler.
Lean toward saving bytes.

### Phase 3 — Dongle: IP forwarding + NAPT on usbnet0

Near-copy of today's SLIP NAPT setup.

**Test**: Mac script sends framed UDP to a public IP, sees reply.

### Phase 4 — DOS TSR: skeleton + CHUSB I/O proof

USBNET.COM opens CDC, completes the load handshake (AT$MODE=PACKET,
PING), exits without going resident. Confirms CHUSB I/O works on the
386 as Phase 0 predicted.

### Phase 5 — DOS TSR: packet driver INT 60h core

All required subfunctions, tested with a packet-driver diagnostic tool
(or hand-rolled INT 60h test program). Resident size measured here.

### Phase 6 — DOS TSR: TX path

mTCP sends ARP via TSR → frame appears on dongle's CDC RX → dongle
ARP responder synthesizes reply → mTCP receives.

### Phase 7 — DOS TSR: RX path + polling

End-to-end mTCP `ping <local-ip>` succeeds. Tune polling cadence.

### Phase 8 — Benchmark + tune

mTCP HTTP GET `192.168.1.226:8765/1M.bin`. Compare to SLIP and Hayes.
**This is where we validate or refute the 60-100 KB/s hypothesis.**

### Phase 9 — Mode-switch robustness

TSR `/U` unload, GPIO0 cycle MODEM↔PACKET, AT$MODE round-trip.

### Phase 10 — Internet HTTP GET via mTCP

PROFILE.LOG Phase 4 works end-to-end (DNS forwarder + NAPT).

### Phase 11 — Retire SLIP

Delete SLIP code path. Two modes only: MODEM, PACKET.

**Estimated calendar**: 8-12 days focused, gated on Phase 0 outcome.

## Risks (ranked)

1. **TSR polling cadence (Phase 0 gates the whole plan).** Without a
   high-rate RX servicing path on DOS, this architecture fails. Phase 0
   must resolve before any other work starts.
2. **MSC + CDC concurrency under CHUSB.** Novel; verify in Phase 0.
3. **mTCP packet driver corner cases.** Broadcast/multicast/odd
   EtherTypes will surface during integration. Mitigate by exercising
   against actual mTCP source during Phase 5-6.
4. **TSR resident size.** 32-36 KB estimate. Worth measuring early;
   could force UMB load.
5. **Mode-switch handshake races.** The MODEM→PACKET flip depends on
   the dongle fully draining its TX FIFO before flipping. Verify
   `cmd_dollar_mode` does this correctly today.

## Open questions

None blocking. Phase 0 starts immediately and will surface any new ones.

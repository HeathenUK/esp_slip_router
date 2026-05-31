# USBNET — Packet-driver TSR over USB-CDC

Design and implementation plan. Replaces SLIP as the high-throughput data
path between the Pocket386 (DOS) and the T-Dongle S3, while preserving the
existing Hayes/AT mode for byte-streaming use cases.

Target: end-to-end mTCP throughput on a 386SX-40 in the 60-100 KB/s
range (10-15x today's SLIP at 6 KB/s; 1.5-2.5x today's Hayes at 42 KB/s).
The new wall after this work is mTCP's per-segment TCP cost on the 386,
not framing, not FOSSIL, not USB.

## Goals & non-goals

### Goals
- DOS runs **unmodified** mTCP (and any other packet-driver client) via a
  new TSR (`USBNET.COM`).
- Heavy stuff that *can* be offloaded *is* offloaded to the dongle: ARP,
  DHCP, DNS, IP routing/NAPT, link framing. DOS does TCP/UDP itself.
- Hayes/AT mode stays as a peer mode; mode switching is in-band.
- **CHUSB is not modified.** The dongle continues to enumerate as plain
  USB-CDC-ACM. The TSR uses CHUSB's existing bulk-I/O API the same way
  any CDC user would.
- No FOSSIL on the packet-driver data path.

### Non-goals
- We do *not* offload TCP. mTCP runs TCP on the 386 because that's what an
  mTCP-compatible packet driver requires. If we ever want TCP-on-dongle
  with a DOS app, that's what Hayes is for; this design does not preclude
  layering a separate "socket" API on top of CDC later.
- We do not accelerate Hayes here.
- We do not retain SLIP as a third mode once PACKET works. (Retire it.)

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

The dongle remains a generic USB-CDC-ACM device. The new code is:
1. A DOS TSR using CHUSB for bulk RX/TX on the CDC endpoints.
2. Firmware additions to terminate ARP/DNS and route IP via NAPT (most of
   which we already have for SLIP).

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
  - `0x11` PING / heartbeat (diagnostic, optional)
  - `0xFF` reserved / error
- No link-layer CRC — USB already provides one at the bulk layer.

This eliminates SLIP byte-stuffing CPU on the 386. Modest win on its own
(~5%); the bigger framing-related win is just having a cheap deframer.

## DOS-side TSR (USBNET.COM)

Resident, ~10-15 KB code + RX ring buffer:

1. **On load**:
   - Locate dongle via CHUSB (CDC-ACM with our VID/PID; `/UNIT:n` for
     multi-device).
   - Send `AT$MODE=PACKET\r` once to force the dongle into PACKET mode.
     Wait for the `OK` then switch our own RX path to framed mode.
   - Hook INT 60h, presenting FTP Software Packet Driver Spec v1.11.
   - Hook a high-rate timer source for RX polling (see "Open question"
     below — INT 28h alone is *not* fast enough).
   - Print: `usbnet: ready, mac=02:01:xx:xx:xx:xx mtu=1500`. Resident.

2. **Packet Driver INT 60h interface**:
   - `driver_info` — class=1 (Ethernet), type=our assigned constant
   - `access_type`, `release_type`, `send_pkt`, `get_address`
   - `reset_interface`, `as_send_pkt`, `get_parameters`, `terminate`

3. **TX path** (`send_pkt`):
   - Caller hands a complete Ethernet frame in ES:DI.
   - We strip the 14-byte ETH header (we know dst=gw_mac, src=our_mac,
     type=0x0800 or 0x0806), framing wraps the L3 body, push to CHUSB.
   - **ARP (0x0806) traverses USB** — the dongle answers. (See red-team:
     this is simpler than DOS-side ARP intercept and ARP traffic is
     negligible.)
   - **IPv4 (0x0800)** is the bulk case.
   - Drop other EtherTypes (no IPv6 in mTCP).

4. **RX path**:
   - Drain function reads from CHUSB into ring, parses framing:
     - `0x01` IP → prepend synthetic ETH header
       (`dst=our_mac, src=gw_mac, type=0x0800`), deliver via registered
       handler.
     - `0x10` MODE_SWITCH — log and ignore (TSR doesn't gracefully
       reload mid-op; user issues UNLOAD first).
     - Other types → log + drop.
   - Drain is called from: (a) the periodic timer hook (b) inside
     `send_pkt` after TX completes.

5. **On unload (`/U`)**:
   - Send `MODE=MODEM` framed control message to dongle to return to
     Hayes.
   - Unhook INT 60h and the timer hook.

6. **Identity**:
   - Synthetic MAC, locally-administered prefix: `02:01:xx:xx:xx:xx`
     where the low 24 bits come from the dongle's WiFi MAC (read once
     at load via an AT query, e.g. `AT$MAC`).
   - Gateway MAC: `02:02:xx:xx:xx:xx` with the same low 24.
   - This keeps both stable across reboots and unique per dongle.

7. **No DHCP/CONFIG_REQ.** Network config (IP/mask/gw/ns) lives in
   `MTCP.CFG` as mTCP already expects. The TSR doesn't care about
   L3 — it only owns the synthetic MAC.

## Dongle-side firmware changes

Most of this already exists for SLIP mode and is being adapted.

1. **lwIP netif `usbnet0`** — replaces the SLIP netif. Same role,
   different framing on the wire.

2. **Framer/deframer** — replaces SLIP encode/decode on the CDC path
   when in PACKET mode. Counters surface via `/net-stats`.

3. **ARP responder on usbnet0** — answers any ARP for `gw_ip` or any
   address mTCP probes locally. Pure synthesis.

4. **DNS forwarder** — UDP socket on `192.168.240.1:53`. On query:
   resolve via dongle's WiFi resolver, synthesize DNS response.
   ~1-2 hours of code; bundled with this work since `MTCP.CFG` will
   point `NAMESERVER` at the dongle.

5. **NAPT** — keep on the usbnet0 netif (the "inverted" IDF API we
   already handle correctly).

6. **Mode-switching state machine**:

```
        ┌──────────┐  AT$MODE=PACKET / framed MODE_SWITCH("PACKET")  ┌──────────┐
        │  MODEM   │ ───────────────────────────────────────────────▶│  PACKET  │
        │ (Hayes)  │ ◀───────────────────────────────────────────── │ (usbnet) │
        └──────────┘  AT$MODE=MODEM (via TSR) / framed MODE_SWITCH    └──────────┘
                                                                        │
                          GPIO0 long-press cycles MODEM ↔ PACKET
                          /mode?to=PACKET|MODEM HTTP endpoint
                          AT$MODE=PACKET|MODEM AT command
```

   - Two modes only: MODEM (Hayes), PACKET (new).
   - SLIP retires once PACKET works.
   - Mode persisted in NVS as today.
   - Switching from MODEM→PACKET: AT command from TSR load.
   - Switching from PACKET→MODEM: framed MODE_SWITCH from TSR unload.

7. **`/net-stats` HTTP endpoint** — replaces `/slip-stats`. Same
   counters + ARP/DNS counters.

8. **`AT$MAC` AT command** — returns the dongle's WiFi MAC so the TSR
   can derive its synthetic MAC from the low 24 bits. (Trivial: already
   accessible via `esp_wifi_get_mac`.)

## What we don't do (and why)

- **No custom CDC interface, descriptor, or class.** Plain CDC-ACM.
  CHUSB sees it like any USB serial dongle.
- **No DOS-side ARP intercept.** ARP traffic is negligible; let the
  dongle answer over USB. Less code, fewer bugs.
- **No DOS-side TCP offload API.** Out of scope.
- **No link-layer CRC.** USB has one already.
- **No byte-stuffing.** Length-prefixed framing on atomic USB bulk.

## Implementation plan

Each phase ends with something testable.

**Phase 0 — CHUSB API spike** (before any other work).
Read CHUSB headers/docs. Confirm:
- Non-blocking bulk read (or timed-poll with short timeout).
- A path to high-rate RX servicing (interrupt hook, INT 1Ch, custom
  timer reprog — find out what's available).
- TX completion semantics.
If non-blocking RX isn't viable, the entire DOS-side architecture has
to be reconsidered. **Do not skip.**

**Phase 1 — dongle: PACKET mode skeleton.**
New netif, framer, mode-switch wiring (NVS, `/mode`, `AT$MODE`, GPIO0).
SLIP stays parallel for safety during transition. Mac-side Python test
script: open CDC, send a framed IP packet, see it counted.

**Phase 2 — dongle: ARP responder + DNS forwarder.**
Both bound to usbnet0. Mac script fires ARP and DNS, sees correct
synthesized replies.

**Phase 3 — dongle: IP forwarding + NAPT on usbnet0.**
Near-copy of today's SLIP NAPT. Mac script sends framed UDP/ICMP to
public IP, sees reply.

**Phase 4 — DOS TSR: skeleton + CHUSB I/O proof.**
USBNET.COM opens CDC, sends `AT$MODE=PACKET`, sends one framed PING,
exits. Confirms CHUSB I/O works on the 386.

**Phase 5 — DOS TSR: packet driver INT 60h core.**
`driver_info`, `access_type`, `release_type`, `send_pkt`,
`get_address`. Tested with a packet-driver dump tool.

**Phase 6 — DOS TSR: TX path.**
mTCP sends ARP via TSR → frame appears on dongle's CDC RX → dongle
ARP responder synthesizes reply → mTCP receives.

**Phase 7 — DOS TSR: RX path & polling.**
End-to-end mTCP `ping <local-ip>` succeeds. Tune polling cadence
based on actual loss.

**Phase 8 — benchmark + tune.**
mTCP HTTP GET `192.168.1.226:8765/1M.bin`. Compare to SLIP and
Hayes. Tune RX ring depth, CHUSB read sizing.

**Phase 9 — mode-switch robustness.**
TSR `/U` unload path, GPIO0 cycle MODEM↔PACKET, AT$MODE round-trip.

**Phase 10 — Internet HTTP GET via mTCP.**
PROFILE Phase 4 actually works end to end (DNS via forwarder + NAPT).

**Phase 11 — retire SLIP.**
Once PACKET is consistently better, delete SLIP code path. Reduces
firmware complexity to two modes (MODEM, PACKET).

**Estimated calendar**: 8-12 days focused. DOS TSR (Phases 4-7) is
the bulk; dongle side is mostly reassembly of existing pieces.

## Risks

- **TSR polling cadence (biggest open question).** INT 28h fires only
  on DOS idle — during heavy mTCP we're not idle. INT 1Ch is ~18 Hz,
  too slow for 60+ KB/s. Need to find a faster polling path through
  CHUSB or reprogram a timer. Phase 0 spike must resolve this.
- **CHUSB bulk RX behavior under bursty load.** If CHUSB doesn't
  buffer adequately or blocks on read, we drop frames → retransmits.
- **mTCP packet driver corner cases.** Spec is documented but
  broadcast/multicast/odd EtherTypes may surface during integration.
- **TSR resident size.** Estimate 30 KB (24 KB ring + ~10 KB code).
  Worth measuring early; might force loading high.
- **Concurrent USB MSC + CDC.** Dongle already does composite
  MSC+HID+CDC; new code shouldn't disturb that. But TSR holding the
  CDC open while DOS does MSC reads to the same device is novel —
  verify CHUSB can multiplex.

## Open questions for the user

None blocking. Plan is to proceed with Phase 0 immediately.

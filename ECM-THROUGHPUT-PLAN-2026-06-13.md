# ECM bridge throughput plan — 2026-06-13

Supersedes the throughput portion of `ECM-HOST-STALL-INVESTIGATION-2026-06-13.md`.
Spans BOTH codebases: the dongle firmware (this repo, `~/esp_slip_router`) and the
DOS USB host driver CHUSB (`~/CH375`).

## Problem

FTP download (Mac server → WiFi → DOSONGLE → CH375/ECM → DOS host → CF disk)
sustains only **~22 KB/s** (measured: 22.4 KB/s, 365 s for ~8 MB). Target:
**100–300 KB/s**. It also *snags* (recoverable RTO stalls) and *occasionally
hard-freezes* (separate bug, see bottom).

## Root cause — it's the host's draining CADENCE, not capacity (both sides verified)

Two reference points prove neither side lacks capacity:
- The dongle as a TCP endpoint (its **own** FTP over WiFi) does **300 KB/s** → WiFi
  + lwIP + heap are not the limit.
- CHUSB does **>300 KB/s for USB disk** over the same CH375 chip → the CH375/USB
  link is not the limit.

The ECM path is slow because of HOW the host drains the bulk-IN, contrasted with disk:

| | Disk read (358 KB/s) | Net RX (22 KB/s) |
|---|---|---|
| Trigger | app blocks on INT 13h | speculative poll |
| Chip | **seized + held** for whole xfer (`g_disk_busy`, `chusb_disk.c:688`) | polled in bursts, yields |
| Drain | tight asm loop, **back-to-back** 64B packets (`chusb_usbxfer.c:621` `scsi_read_10_far`) | `net_pump` rides only `tries=2..4` NAK-polls then **breaks/yields** (`chusb_net.c:625–638`) |
| Retry mode | `0xBF` (NAK-absorb; data guaranteed post-CBW) | `0x0F` fail-fast (MUST — speculative `0xBF` wedges the chip, `chusb_net.c:582`) |
| Device side | disk controller **streams** packets, no gap | stock dongle presents 1 frame, **re-arms in ~1 ms** (lwIP→pump→USB) |

**The bug:** after each frame `net_pump` rides ~2–4 NAK-polls (~tens of µs) then
yields, but the dongle re-arms in ~1 ms — so the ride gives up **~20× too early**
and catches ~1 frame per pump invocation (tick/idle/send-tail, ~18–100 Hz) → 22 KB/s.

## Ruled OUT (with evidence — do NOT revisit)

- **mTCP window size** — user states it is NOT an mTCP setting (corrected twice in
  prior sessions). See `[[ecm-throughput-bottleneck-is-frame-cadence]]`.
- **WiFi / RSSI / RF** — user corrected repeatedly. See `[[stop-blaming-rf-for-ecm-stalls]]`.
- **Dongle heap / WiFi-RX pool** — endpoint FTP does 300 KB/s; heap stable (`min_free`
  ~12–20 K during bursts). Deeper TX buffer OOM-crashed (depth-24 → rollback) — the
  q-drops are the heap SAFETY-VALVE, not the bug.
- **NAPT eviction** — measured `napt=e0` throughout; `IP_NAPT_TIMEOUT_MS_TCP`=30 min.

## Plan — "do both", dongle stays a fixed trusted reference

### 1. DOSONGLE → stock (this repo)  [in progress]

Strip investigation scaffolding so the dongle is a boring, trusted reference while
we tune the host. The TinyUSB **driver must be stock** (the user's objection to
managed-component edits).

- `ip4_napt.c` (IDF tree): revert the `g_napt_tcp_evict` / `g_napt_recv_nomatch`
  counters → stock IDF lwip.
- `ecm_rndis_device.c`: remove `tud_network_ep_in/_data_alt/_ep_in_busy` (diagnostics)
  and the dead `tud_network_tx_complete_cb` zero-gap hook. **Keep** the pre-existing
  notif-EP guard (Patch 4) and `tud_network_xmit_recover` (one justified robustness
  fn — guards the lost-completion wedge; scripted in `apply_iram_patches.sh`).
- `ecm.c`: keep the reference-style pump (linkoutput→queue→pump→`tud_network_xmit`,
  defer-to-USB-task for the dcd race, drop-on-200ms for a dead host) + the 1 Hz
  heartbeat (for MEASUREMENT). Remove napt fields, burst counters, the detailed
  TX-STALL diagnostic log. Keep `AT$NETFLOOD`/`AT$NETSLOW` as bench tools.
- No re-arm surgery on the dongle — the CHUSB ride bridges the ~1 ms gap.

### 2. CHUSB → continuous drain (`~/CH375/chusb_net.c`)  [the throughput fix]

Make `net_pump`'s ride **bridge the dongle's ~1 ms per-frame re-arm** during an
active download, instead of `tries=2..4`:
- Drain-while-flowing: keep polling as long as frames arrive, with enough NAK
  tolerance to span the re-arm gap; yield only when the endpoint goes **genuinely**
  empty (a run of NAKs longer than the re-arm).
- **Self-limiting**: the server stops at window-full (~16 KB ≈ 27 frames) → endpoint
  empties → ride expires → `net_pump` returns so the app can ACK → next window. The
  window boundary IS the natural yield (drain hard in a burst, breathe between).
- Stay `0x0F` (no `0xBF` — would wedge). No fairness disaster: bounded per window.
- **MEASURE first**: the dongle's actual re-arm gap and per-NAK-poll cost set the
  ride bound — not a magic number. (Mirror the disk path's held-drain model.)

Expected: bridging a 1 ms re-arm caps at ~1000 frames/s ≈ **~590 KB/s** — past target.

## SEPARATE unsolved: the intermittent HARD-freeze (do not conflate with throughput)

Distinct from the snags. Download locks up **forever** at a window boundary
(16384 / 212992 / 9944 across runs); needs `POST /usb-reconnect` to free the DOS
client. Captured signature (build with napt counters): device TX **idle and ready**
(`cx=1, qd=0`, heap fine), BOTH directions flat, `napt=e0` (no eviction) with
`nm` (inbound-TCP-no-match) creeping — i.e. an off-device standoff, the server's
packets possibly black-holed at NAPT reverse-lookup, OR the server genuinely silent.
**Unconfirmed.** When throughput is done, re-add the napt reverse-no-match counter
and catch one with it visible. Do NOT blame RF.

## Status / as-built

- Device currently on `e38ef48` (stable pump-based baseline, grinds through slowly,
  has heap+napt heartbeat). OTA via `dongle-harness/ota-http.sh`; read `/disk-log`
  over WiFi; `POST /usb-reconnect` to free a hung DOS client.
- Next: (1) dongle cleanup-to-stock, (2) measure dongle re-arm, (3) CHUSB ride fix.

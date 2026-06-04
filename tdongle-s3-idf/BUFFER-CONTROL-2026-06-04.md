# Adaptive relay window controller — design spec

**Goal:** size the relay's TCP receive window to the **largest the consumer can
actually drain, but never larger than heap can safely hold** — continuously,
from live conditions, with no port/command pinning. Targets the two observed
failure modes: 8 KB window + slow consumer → *bled heap dry → froze*; 4 KB →
*stable but slow*. The controller converges to fast-when-safe, small-when-not.

## Lever
`setsockopt(s_sock, SO_SNDBUF/SO_RCVBUF)` adjusted **live** during the session.
`LWIP_SO_RCVBUF` is already enabled. This spec implements the **RX window**
(download direction — the one that froze us). The SND_BUF/upload mirror is a
documented follow-up (same loop, tx counters).

## Signals (sampled every TICK_MS in modem_data_task, no new task)
1. **Consumer keep-up** — `s_tp_blk_us` delta (µs `cdc_write` blocked waiting on
   FIFO space) over the tick. High delta = host not draining = consumer-bound.
2. **Heap headroom** — `esp_get_free_heap_size()`.
(Direction is implicit: this loop runs on the RX relay = downloads.)

## Parameters (reliability-first)
| name | value | why |
|---|---|---|
| WIN_MIN | 2048 | floor (telnet / slow consumer) |
| WIN_START | 4096 | safe moderate start |
| WIN_MAX | 8192 | ceiling — proven safe *under good conditions*; not higher (reliability-first) |
| WIN_STEP | 2048 | coarse: 4 K↔6 K↔8 K |
| HEAP_LOW | 10240 | clamp: below → shrink hard (well above the 5 K guard) |
| HEAP_HIGH | 18432 | grow only above this (real headroom) |
| TICK_MS | 500 | control interval |
| SAT_US | 100000 | blocked >100 ms in a 500 ms tick (≈20%) = saturated |

## Control law (each tick)
```
blk_delta = s_tp_blk_us - prev_blk;  free = esp_get_free_heap_size()
if      free < HEAP_LOW        : win = max(WIN_MIN, win - 2*WIN_STEP)   # CLAMP: shrink hard
elif    blk_delta > SAT_US     : win = max(WIN_MIN, win -   WIN_STEP)   # consumer-bound: shrink to match
elif    free > HEAP_HIGH       : win = min(WIN_MAX, win +   WIN_STEP)   # headroom + draining: grow
else                           : hold
if win changed: setsockopt(SO_RCVBUF, win); disk_logf("winctl ...")
```
Hill-climber: grows toward the largest window the consumer drains within heap
limits; shrinks when saturated (a bigger window only piles up backlog for a
slow consumer — *the exact bleed*) or when heap tightens. Hysteresis from the
HEAP_LOW/HIGH gap + the coarse step.

## Safety
- The existing **5 KB low-heap guard (abort session)** stays as the absolute
  backstop. HEAP_LOW (10 K) acts well before it — the controller should mean the
  guard never fires.
- WIN_MAX (8 K) is only reachable when `free > 18 K` AND consumer keeps up —
  precisely the regime the Mac-CDC run proved safe (`cdc_block_us=0`, heap 20 K).
- Shrinking SO_RCVBUF mid-stream is "soft" (lwIP stops opening the window;
  can't retract advertised) — correct: it halts further growth and lets backlog
  drain.
- Subsumes the static telnet/binary split: a slow telnet consumer stays at
  WIN_MIN (saturated → never grows); a fast download climbs. No port/command.

## Test plan (from the Mac, both directions of the loop)
1. **Fast consumer** (`hayes-throughput.py`, reads flat-out): expect the window
   to **grow** 4 K→8 K, byte-perfect md5, heap floor healthy. winctl log shows
   growth.
2. **Slow consumer** (a throttled reader: read the CDC at ~5 KB/s with sleeps):
   expect the window to **stay small / shrink** (saturated), heap floor stays
   well clear of the cliff — i.e. NO bleed. winctl log shows it holding low.
3. Confirm the 5 KB guard never fires in either.

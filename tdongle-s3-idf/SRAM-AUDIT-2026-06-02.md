# SRAM audit — finding headroom for the status LCD

**Date:** 2026-06-02. All figures **measured** (`idf.py size` + linker `.map` + live `ATI heap`), not estimated.

## The budget

| | bytes |
|---|---|
| Internal DIRAM total | 341,760 |
| `.bss` (static, zero-init) | **153,760** (45%) |
| `.data` (static, init) | 20,800 |
| Static used (DIRAM) | 285,807 (83.6%) |
| **Static remain → the heap pool** | **~55,953** |
| Free heap at runtime, display **off** | ~21,600 (min 16,416) |
| Free heap at runtime, display **on** | **2,456 (min 32)** ← HTTP dies here |
| Display's measured cost | **~19,000** |

The ~56 KB "remain" minus WiFi/lwIP/TinyUSB/task-stack heap allocations leaves only ~21 KB free. The display wants ~19 KB → collision.

## Where the 154 KB of `.bss` goes (top consumers, from .map)

`libmain.a` (OUR code) owns **121,733 B** — the IDF stack is the minority. Breakdown:

| Symbol | bytes | What |
|---|---|---|
| **`disk.c`** | **65,920** | Write-back cache `s_wb[16] × 4096` + `s_wb_flush_buf[4096]` |
| `fat.c` | ~16,400 | Four ~4 KB buffers (2 are WL-sector RMW; rest FATFS) |
| `modem.c` | ~13,300 | Six static scratch bufs (2048×4 + 1024 + 2048) |
| `slip.c` | ~4,700 | SLIP packet assembly buffers |
| `dns_forwarder.c` | ~1,000 | two 512 B |
| lwip | 28,664 | IDF — TCP/pbuf pools (config-tunable) |
| freertos | 20,320 | IDF |
| net80211/pp/phy | ~39,000 | IDF WiFi (mostly fixed) |

Plus **heap** (not in `.bss`): modem StreamBuffers `s_to_cdc`+`s_to_tcp` = **2 × 8192 = 16 KB**; WiFi dynamic RX/TX buffers (config-tunable).

## Recovery options, by risk

### Tier A — low risk, our code, ~20 KB available
| Action | Saves | Risk |
|---|---|---|
| `fat.c`: the two 4 KB WL-RMW buffers (`d_read`/`d_write`) are mutually exclusive (both under `disk_fatfs_lock`) → **share one** | 4 KB bss | low |
| `fat.c`: investigate the other two 4 KB buffers; `CONFIG_FATFS_VOLUME_COUNT=2` but only one volume exists → drop to 1 | ~4 KB bss | low (verify) |
| `modem.c`: trim scratch bufs (inbuf/outbuf 2048→1024; keep txbuf 2× for IAC/CRLF) | ~5 KB bss | low–med (throughput) |
| `modem.c`: StreamBuffers `DATA_STREAM_BYTES` 8192→4096 | 8 KB **heap** | med (Hayes throughput) |
| `main.c`: disk-log ring 48×128 → 32×96 | 3 KB bss | low (less crash history) |

### Tier B — shrink the *consumer* (the display itself)
The display measured **19 KB** but buffers+stack+statics only explain ~8 KB. The unexplained ~11 KB is `esp_lcd` panel-io + SPI-master/GDMA runtime allocation sized off `max_transfer_sz`.
| Action | Saves | Risk |
|---|---|---|
| **Hand-roll SPI** (`spi_device_transmit` directly, drop the `esp_lcd_panel_io` layer) | likely 8–11 KB | med (more code we own) |
| Band buffer `BAND_W` 80→40 | 1.3 KB | low |
| Task stack 2560→2048 | 0.5 KB | low |
| **→ display from ~19 KB down to ~6 KB** | | |

### Tier C — break glass (the 66 KB elephant)
`disk.c` WB cache is **by far** the biggest single block, BUT it is **load-bearing**: it's the burst headroom for DOS/CHUSB writes outrunning the ~15-WL-sector/s flash drain. **History: 32→16→8→10→16; `WB_SLOTS=10` crashed under sustained CHUSB writes**, and the user explicitly vetoed shrinking it.
| Action | Saves | Risk |
|---|---|---|
| `WB_SLOTS` 16→12 | 16 KB bss | **high** — MSC write stability, prior crashes |
| `WB_SLOTS` 16→8 | 32 KB bss | **high** |

## Recommendation

**Don't touch the WB cache (Tier C).** It's the largest block but it's functional and has a crash history the user already hit.

The display fits cleanly with **Tier B + a little Tier A**:
- Tier B: hand-roll the SPI path → display drops from ~19 KB to **~6 KB**.
- Tier A: share the fat.c RMW buffer (−4 KB) + trim modem scratch (−5 KB) + disk-log ring (−3 KB) = **~12 KB freed**.
- Net: ~12 KB more free heap, display needs only ~6 KB → **~27 KB free with the display on** (was −19). Comfortable, WB cache untouched.

Tier-B (shrinking the consumer) is the highest-value single move and should be confirmed by `heap_caps` decomposition of where the display's 19 KB actually lands before committing.

---

## OUTCOME (measured, post-implementation)

What was actually done and the measured result (display-on, WiFi connected):

| stage | free heap | min free |
|---|---|---|
| esp_lcd display-on (start) | 2,456 | 32 |
| hand-roll SPI + Tier-A trims | 14,244 | 944 |
| + StreamBuf 8192→4096, OTA buf 2048→1024, disk-log 32→24 | **24,592** | **16,272** |

**Correction to the Tier-B premise above:** the "~11 KB of esp_lcd overhead" was **wrong**. Hand-rolling the SPI (dropping `esp_lcd_panel_io`) saved only **~2 KB** — the bulk of the display's ~17 KB is the **SPI DMA driver** (`spi_bus_initialize`), which the hand-roll keeps. The fit was actually won by:
- **Tier-A static trims**: fat.c shared RMW buffer (−4 KB), modem scratch (−3 KB), disk-log ring 48→32 (−2 KB) = ~9 KB more heap pool (`.bss` 153,760 → 144,544).
- **StreamBuffers `DATA_STREAM_BYTES` 8192→4096** = +8 KB heap (the single biggest runtime win).
- Plus OTA buf 2048→1024 and disk-log 32→24 (`.bss` → 142,496).

`.bss` net: **153,760 → 142,496**. Display kept (hand-rolled ST7735), renders correctly.

**Untouched / remaining levers** (not needed; available if ever required):
- `disk.c` format buffers `sbuf[4096]`+`zero[4096]` = **8 KB**, format-path-only; would need refactor to on-demand heap. Deferred (format path is FAT-corruption-sensitive).
- WB cache (66 KB) — load-bearing, off-limits per history.


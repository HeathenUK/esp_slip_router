# /fs HTTP transfer heap-hunger — large files wedge the dongle

**Date:** 2026-06-03
**Severity:** HIGH — can wedge the device (heap starvation kills BOTH OTA paths
→ stranded until physical Download mode; see the never-starve directive).

## Symptom

HTTP `/fs` file transfers consume heap proportional to file size, and large
files drive `min_free_heap` to / past the starvation cliff:

| file | size | result |
|---|---|---|
| WGET.EXE | 19 KB | fine |
| USBTERM.EXE | 27 KB | **survived but `min_free_heap` hit 504 B** (a hair from the cliff) |
| CHUSB.EXE | 109 KB | **wedged** — repeated network-dark + a reset; file write landed (correct size in /list) but read-back returned 0/partial and the device went dark for >75 s several times |

The **GET (read-back)** is the worse offender: the 109 KB *reads* are what
repeatedly darkened the device, and the 27 KB md5-verify GET is what dropped the
floor to 504. PUT (write) is milder.

## NOT the cause: neither handler buffers the file

- `h_fs_get` (main.c:597) streams: `fat_read(name, fat_out_to_httpd, req)`, and
  `fat_out_to_httpd` (main.c:592) is just `httpd_resp_send_chunk(req, in, n)`
  per chunk. No whole-file buffer.
- `h_fs_put` streams too: `fat_write(name, fat_in_from_httpd, content_len, req)`
  pulls from the request body. No whole-file buffer.

So the heap isn't going to a file-sized RAM buffer.

## Root cause (strong hypothesis): the lwIP TX send queue

`CONFIG_LWIP_TCP_SND_BUF_DEFAULT=65535` (sdkconfig). On a GET, `fat_read`
produces data from flash **fast**, and each chunk goes to
`httpd_resp_send_chunk` → lwIP `send()`. lwIP queues the outgoing bytes as **TX
pbufs in heap, up to SND_BUF (~64 KB)**, holding them until the client ACKs.
The WiFi/client drains slower than flash produces, so the TX queue fills:

- 109 KB file → the queue saturates at ~64 KB of held pbufs → heap exhausted →
  wedge. (Read-back returned 0/partial because httpd couldn't allocate.)
- 27 KB file → up to ~27 KB held → floor hit 504 B (barely survived).
- 19 KB file → smaller transient → survived comfortably.

This scales exactly with file size up to the 64 KB SND_BUF cap, which matches
the observations. PUT is milder because the *receive* side is bounded by
`TCP_WND_DEFAULT=4096` (only ~4 KB of RX pbufs hold at once).

Note: SND_BUF was deliberately set to 65535 for the **modem CDC→TCP upload**
path (so a Hayes upload can pipeline a full window). That same global setting is
what makes a `/fs` GET dangerous. The two uses are in tension.

## Why it matters

A `/fs` GET of a >~30–50 KB file can drive heap past the ~0.9 KB cliff and wedge
the device, which kills CDC + HTTP OTA → physical Download-mode recovery only.
**`/fs` is currently unsafe for files larger than ~30 KB.** (Workaround used in
practice: PUT lands the file correctly even when GET can't read it back; the
Pocket386 reads via MSC, a different path, so the file is still usable — but the
HTTP read-back and large PUTs are landmines.)

## Fix candidates

1. **Pace the chunked send (preferred, no global impact).** In `h_fs_get` /
   `fat_out_to_httpd`, bound how much sits unACKed: after each chunk, if the
   socket's unsent TX queue exceeds a threshold (e.g. 8–16 KB), wait for it to
   drain before producing the next chunk. Caps held pbufs regardless of file
   size. (Check `tcp_sndbuf()` / SO_SNDBUF-style backpressure, or a small
   per-chunk delay gated on send progress.)
2. **Cap SO_SNDBUF on the httpd `/fs` socket** to ~8–16 KB, leaving the global
   SND_BUF=65535 for the modem upload path. Per-connection bound, simple.
3. **Lower the global SND_BUF** — rejected: it throttles the modem CDC→TCP
   upload that 65535 was chosen for.

Recommend (1) or (2). Same exposure exists for *any* large outgoing HTTP
response, but `/fs` files are the only large ones today.

## Code locations

- `main/main.c`: `h_fs_get` (~597), `fat_out_to_httpd` (~592), `h_fs_put`.
- `main/fat.c`: `fat_read` / `fat_write` (the streaming callbacks).
- `sdkconfig` / `sdkconfig.defaults`: `CONFIG_LWIP_TCP_SND_BUF_DEFAULT=65535`,
  `CONFIG_LWIP_TCP_WND_DEFAULT=4096`.

## Verification of a fix

A `/fs` GET of a ≥109 KB file should complete intact (md5 matches) with
`min_free_heap` staying clear (e.g. >10 KB) throughout and no network-dark /
reset. Cross-check with `/status` `min_free_heap` immediately after.

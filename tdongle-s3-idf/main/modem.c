/* modem.c -- Hayes AT modem on TinyUSB CDC interface 0.
 *
 * Phase 1c (full set ported from the arduino-esp32 build's
 * tdongle-s3/src/modem.cpp). The wire-level behaviour matches that
 * firmware exactly so the DOS-side host stack (FOSSLIP + CHUSB +
 * mtcpget) sees a drop-in replacement.
 *
 * Command-mode bytes arrive via the CDC RX callback (TinyUSB task
 * context). When a successful ATD opens a TCP socket, we flip into
 * online mode and spawn modem_data_task to pump socket -> CDC; the
 * RX callback keeps draining CDC -> socket. Either direction can
 * trigger return-to-command (a +++ escape from CDC, a socket close
 * from the peer).
 *
 * Telnet IAC handling (RFC 854 + a few common options) is on by
 * default; binary-mode hosts negotiate it off via DO/WILL BINARY.
 *
 * NVS layout (shared with main.c's bootstrap path):
 *   slip-router/ssid  (string, max 32)
 *   slip-router/pass  (string, max 64)
 */

#include "modem.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"

#include "tusb.h"
#include "tusb_cdc_acm.h"
#include "class/cdc/cdc_device.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"

#include "disk.h"   /* disk_logf */
#include "slip.h"
#include "kbd.h"

#define TAG "modem"

/* ---- command-mode state ---- */
#define CMD_LINE_MAX 256
static char     s_cmd[CMD_LINE_MAX];
static uint16_t s_cmd_len      = 0;
static bool     s_cmd_overflow = false;

static bool s_echo    = true;
static bool s_verbose = true;
static bool s_quiet   = false;
static bool s_telnet  = true;     /* Telnet IAC handling on by default */

/* ---- NVS-persisted modem flags (E/V/N) ----
 *
 * Old build's AT&W packs E/V/N into a single byte; AT&F resets to
 * all-on (matching ATZ defaults). modem_load_evn() is called at
 * init; modem_save_evn() persists current state. */
static void modem_load_evn(void) {
    nvs_handle_t h;
    if (nvs_open("slip-router", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t b = 0x07;   /* default: all-on */
    nvs_get_u8(h, "evn", &b);
    nvs_close(h);
    s_echo    = (b & 1) != 0;
    s_verbose = (b & 2) != 0;
    s_telnet  = (b & 4) != 0;
}
static void modem_save_evn(void) {
    nvs_handle_t h;
    if (nvs_open("slip-router", NVS_READWRITE, &h) != ESP_OK) return;
    uint8_t b = (s_echo ? 1 : 0) | (s_verbose ? 2 : 0) | (s_telnet ? 4 : 0);
    nvs_set_u8(h, "evn", b);
    nvs_commit(h);
    nvs_close(h);
}

/* ---- online-mode state ---- */
static int       s_sock = -1;
static volatile bool s_online = false;
static char      s_peer[96] = {0};       /* "host:port" string for AT$STATUS */
static volatile uint16_t s_peer_port = 0; /* numeric port of the active dial (auto-telnet gate) */
static TaskHandle_t s_data_task = NULL;
static TaskHandle_t s_cdc_pump_task = NULL;
static TaskHandle_t s_tcp_pump_task = NULL;

/* --- throughput instrumentation (reset at dial connect, read via ATI) ---
 * Answers relay-vs-consumer: if s_tp_blk_us is a large fraction of the
 * session, cdc_write spent the time WAITING for the host to drain the CDC
 * FIFO => consumer-bound (dongle could feed faster). If blk is ~0 but the
 * session is slow, the limit is upstream (recv/TCP/relay). */
static volatile uint32_t s_tp_rx = 0;       /* bytes recv() pulled from TCP */
static volatile uint32_t s_tp_cdc = 0;      /* bytes cdc_write pushed to host */
static volatile uint64_t s_tp_blk_us = 0;   /* us cdc_write waited on CDC FIFO space */
static volatile int64_t  s_tp_start = 0;    /* session start (esp_timer) */

/* Pipeline: producer (modem_data_task, recv-side, telnet IAC + +++
 * detection) pushes bytes into s_to_cdc; consumer (modem_cdc_pump_task,
 * CDC-side) drains and calls cdc_write. Decouples WiFi RX from USB CDC
 * push, so a CDC FIFO stall doesn't block recv() and vice versa.
 *
 * 8 KB per stream, 2 streams = 16 KB heap. We tried 16 KB each (32 KB
 * total) for more CHUSB-side absorption but that combined with the
 * dial worker stack pushed first-dial allocations past free heap on
 * boot (panic at xStreamBufferCreate returning NULL -> pump task
 * derefs NULL handle). 8 KB absorbs ~80 ms of WiFi RX at 100 KB/s,
 * which empirically is enough. */
#define DATA_STREAM_BYTES 4096   /* 8192 -> 4096 (SRAM: +8 KB heap, 2026-06-02) */

/* Low-heap guard for the relay. If free heap falls below this DURING a
 * session, the data task aborts gracefully (NO CARRIER -> command mode)
 * instead of relaying until starvation WEDGES the device -- and a wedge
 * kills BOTH OTA paths, stranding the dongle until a physical Download
 * mode (see the never-starve directive). Tuned to fire only on a genuine
 * anomaly: the measured worst-case under-load floor is ~7.5 KB, the death
 * zone is ~900 B, so 5 KB sits below normal stress (no false aborts) yet
 * well clear of the cliff. Crucially the abort CLOSES the socket, which
 * releases up to a full TCP_WND of held RX pbufs (~8 KB) -- so tripping
 * the guard actively RECOVERS heap rather than just bailing. */
#define LOW_HEAP_GUARD_BYTES 5120

/* Adaptive RX-window controller (see BUFFER-CONTROL-2026-06-04.md). Sizes the
 * relay's receive window to the consumer: shrinks to 2K when the consumer
 * saturates (a bigger window only piles up backlog for a slow consumer -- the
 * bleed) or heap tightens. Consumer-driven, supersedes the static telnet/binary
 * split.
 *
 * CEILING 16K (2026-06-04, corrected): a download's throughput ceiling is the
 * advertised RECEIVE window: tput <= RWND / RTT. On a weak link RTT inflates
 * under load (802.11 retransmits), so the old 4K cap window-limited downloads to
 * ~7 KB/s in the same spot that did 70 KB/s with a 64K window. The earlier "4K
 * is safe, bigger bleeds" finding was a FALSE ALARM -- that test (fast LAN server
 * -> dongle) was confounded by the same weak-2.4GHz RF path, not a heap bug.
 *
 * CEILING SIZED FOR THE HEAP FLOOR, not for peak speed. A fast download buffers
 * up to a full window inside a single recv() before the loop can re-check heap,
 * so the floor is set by the window itself: floor ~= baseline - window - ~5K
 * overhead. Measured: a 16K window -> 200 KB/s but floor 6.2K (only ~1K over the
 * 5K guard -- NOT rock solid). The fast clamp can't help: the buffering happens
 * inside recv(), faster than any per-iteration check. The ONLY lever is the
 * ceiling. So size the window for a SAFE floor and accept the (still large)
 * throughput: 10K -> floor ~12K (7K over the guard), ~150+ KB/s -- 2x the prior
 * 70 KB/s baseline in this spot, with margin to spare. (Raise only after
 * recovering baseline heap; do not trade the floor for speed we don't need.)
 *
 * Heap still only piles up with a SLOW consumer -- the controller shrinks on
 * cdc_block saturation for that case; the heap clamp is the abnormal-pressure
 * backstop (set BELOW the natural floor so it doesn't fire every cycle). */
#define WINCTL_MIN     4096
#define WINCTL_START   10240
#define WINCTL_MAX     10240
#define WINCTL_STEP    4096
#define WINCTL_HEAP_LOW  8192    /* abnormal-pressure clamp: below the ~12K natural floor, above the 5K guard */
#define WINCTL_HEAP_HIGH 24576   /* re-grow gate (moot while MAX==START; kept for the shrink/recover path) */
#define WINCTL_TICK_US   (500*1000)
#define WINCTL_SAT_US    100000   /* cdc_write blocked >100ms in a 500ms tick => consumer-bound */

static StreamBufferHandle_t s_to_cdc = NULL;
/* Reverse pipeline: producer is on_cdc_rx (TinyUSB task, CPU1) pushing
 * raw bytes into s_to_tcp; consumer is modem_tcp_pump_task (CPU0)
 * running the +++ guard / telnet escape / CRLF expansion + send().
 * Same rationale as s_to_cdc -- a slow send() must not back-pressure
 * the USB OUT EP all the way to the host. */
static StreamBufferHandle_t s_to_tcp = NULL;

/* +++ escape sequence detector. Hayes rule: 1 s of guard, then exactly
 * three '+' within 1 s, then 1 s of guard with no other data. */
#define GUARD_US 1000000
static volatile uint8_t  s_plus_count    = 0;
static volatile int64_t  s_last_data_us  = 0;
static volatile int64_t  s_plus_time_us  = 0;

/* ---- telnet ---- */
#define TN_IAC  0xFF
#define TN_WILL 0xFB
#define TN_WONT 0xFC
#define TN_DO   0xFD
#define TN_DONT 0xFE
#define TN_SB   0xFA
#define TN_SE   0xF0
#define OPT_BINARY 0
#define OPT_ECHO   1
#define OPT_SGA    3
#define OPT_TTYPE  24
#define OPT_NAWS   31
#define TT_IS      0
#define TT_SEND    1

/* Telnet RX state machine -- this is what we feed bytes-from-TCP into
 * to strip the IAC negotiation noise before relaying payload to CDC. */
enum { T_DATA, T_IAC, T_OPT, T_SB_OPT, T_SB_DATA, T_SB_IAC };
static int     s_tstate = T_DATA;
static uint8_t s_tcmd   = 0;
static uint8_t s_sbopt  = 0;
static uint8_t s_sbbuf[16];
static uint8_t s_sblen  = 0;
/* Tracked option flags so we don't echo the same WILL/DO repeatedly. */
static uint8_t s_remote_on[32], s_local_on[32], s_local_offered[32];
/* NVT CR-NUL collapse: when remote isn't BINARY, a NUL immediately
 * following a CR is the "bare CR" wire encoding and must be dropped
 * before delivery to usbterm. Tracks "last byte emitted to outbuf
 * was a CR" -- cleared on any non-CR output. */
static bool    s_prev_was_cr = false;
/* Local-echo / password-safety state. See TELNET-LOCAL-ECHO-2026-06-01.md. */
static volatile bool s_remote_ever_echoed = false;
#define LECHO_AUTO 0
#define LECHO_ON   1
#define LECHO_OFF  2
static uint8_t s_lecho_mode = LECHO_AUTO;

/* Terminal advertised via telnet NAWS / TTYPE. Auto-detected at dial
 * time via a DSR-CPR probe over CDC (see tn_query_size); usbterm
 * answers natively, so no usbterm-side contract is needed. Manual
 * AT$NAWS=cols,rows / AT$TTYPE=name remain as overrides for hosts
 * that don't answer CPR (DOSBox/null-modem, ATNET0 raw peers). On
 * probe timeout we fall back to 80x24 -- the VT100 NVT default and
 * what usbterm's author recommended in
 * TELNET-LOCAL-ECHO-2026-06-01.md. */
static uint16_t s_term_cols = 80;
static uint16_t s_term_rows = 24;
static char     s_term_type[32] = "ANSI";

/* CPR (cursor-position-report) probe state. While pending, on_cdc_rx
 * intercepts incoming bytes and feeds them into the parser instead
 * of routing to SLIP / online / AT command paths. Times out so a
 * non-responding peer (DOSBox nullmodem, dumb device) doesn't wedge
 * the dial. */
static volatile bool s_cpr_pending = false;
static int64_t       s_cpr_deadline_us = 0;
static uint8_t       s_cpr_buf[32];
static size_t        s_cpr_len = 0;
#define CPR_TIMEOUT_US (500 * 1000)

/* AT dial runs off the TinyUSB task on a one-shot worker so DNS +
 * TCP connect + CPR wait don't starve CDC servicing -- under the
 * CH375 host, ~800+ ms of CDC quiet at high inbound rates can
 * escalate from NAK to chip-side timeout and trip the host's
 * disconnect-and-re-enumerate cascade. (Diagnosed cross-referencing
 * dongle on_cdc_rx + CHUSB chusb.c:2503 / ACTIVE_LOST path.)
 *
 * s_at_busy gates re-entry: any AT command issued while a dial is
 * in flight gets ERROR, matching today's effective behaviour (today
 * the TinyUSB task is blocked so no command runs anyway). */
static volatile bool s_at_busy = false;
static char          s_dial_arg[128];
static TaskHandle_t  s_dial_task = NULL;
static inline bool bget(uint8_t *a, uint8_t o) { return a[o >> 3] & (1 << (o & 7)); }
static inline void bset(uint8_t *a, uint8_t o) { a[o >> 3] |=  (1 << (o & 7)); }
static inline void bclr(uint8_t *a, uint8_t o) { a[o >> 3] &= ~(1 << (o & 7)); }

/* ---- CDC write helper ---- */

static size_t cdc_write(const void *buf, size_t n) {
    const uint8_t *p = (const uint8_t *)buf;
    size_t sent = 0;
    int64_t deadline = esp_timer_get_time() + 500000;  /* 500 ms */
    while (sent < n && esp_timer_get_time() < deadline) {
        if (!tud_cdc_n_connected(0)) break;
        size_t avail = tud_cdc_n_write_available(0);
        if (avail == 0) {
            int64_t bt = esp_timer_get_time();
            tud_cdc_n_write_flush(0);
            vTaskDelay(pdMS_TO_TICKS(2));
            s_tp_blk_us += (uint64_t)(esp_timer_get_time() - bt);  /* waiting on host */
            continue;
        }
        size_t chunk = (n - sent) < avail ? (n - sent) : avail;
        size_t w = tud_cdc_n_write(0, p + sent, chunk);
        if (w == 0) {
            int64_t bt = esp_timer_get_time();
            tud_cdc_n_write_flush(0);
            vTaskDelay(pdMS_TO_TICKS(2));
            s_tp_blk_us += (uint64_t)(esp_timer_get_time() - bt);
            continue;
        }
        sent += w;
        s_tp_cdc += (uint32_t)w;
    }
    tud_cdc_n_write_flush(0);
    return sent;
}

static inline void cdc_print(const char *s) { cdc_write(s, strlen(s)); }
static inline void cdc_byte(uint8_t b)      { cdc_write(&b, 1); }

/* Flush the USB CDC TX FIFO to the host and wait (bounded) for it to
 * actually EMPTY -- i.e. for the host to have pulled every byte. This is
 * the difference between "the stream buffer drained into the FIFO" and
 * "the host received it": tud_cdc_n_write_available() returns the full
 * FIFO depth only once the host has consumed all in-flight data. Without
 * this, connection teardown closed while up to a full TX FIFO (~4 KB) of
 * the file tail was still queued, so HTTP downloads truncated ~8 KB short
 * at end-of-file (2026-06-03). Bounded to ~2 s so a host that has stopped
 * reading can never wedge us here. Call only when no other task is writing
 * CDC (the relay pump must be idle/parked first). */
static void drain_cdc_fifo(void) {
    /* Wait for the host to pull the whole TX FIFO. We do NOT trust a fixed
     * "FIFO empty" constant (if it's wrong we'd time out every time and
     * strand the tail). Instead: free space only grows as the host pulls;
     * track the max seen (== true empty) and keep waiting while it's still
     * climbing. Stop when it's been STABLE for 1.5 s (host either finished
     * or stopped pulling) or an 8 s absolute cap. Log what was left so a
     * truncation can be attributed exactly. */
    int prev = -1, stall_ms = 0, waited = 0, maxseen = 0, avail = 0;
    while (tud_cdc_n_connected(0) && waited < 8000) {
        tud_cdc_n_write_flush(0);
        avail = (int)tud_cdc_n_write_available(0);
        if (avail > maxseen) maxseen = avail;
        if (avail != prev) { stall_ms = 0; prev = avail; }       /* progress */
        else { stall_ms += 2; if (stall_ms >= 1500) break; }     /* stable -> done/gone */
        vTaskDelay(pdMS_TO_TICKS(2));
        waited += 2;
    }
    /* stuck = bytes the host never pulled (max free minus current free). */
    int stuck = maxseen - avail;
    if (stuck > 0 || waited >= 1500)
        disk_logf("cdc drain: stuck=%d avail=%d/max=%d waited=%dms",
                  stuck, avail, maxseen, waited);
}

/* ---- result codes ---- */
static void r_ok(void)         { if (!s_quiet) cdc_print(s_verbose ? "\r\nOK\r\n"         : "0\r\n"); }
static void r_error(void)      { if (!s_quiet) cdc_print(s_verbose ? "\r\nERROR\r\n"      : "4\r\n"); }
static void r_connect(void)    { if (!s_quiet) cdc_print(s_verbose ? "\r\nCONNECT\r\n"    : "1\r\n"); }
static void r_nocarrier(void)  { if (!s_quiet) cdc_print(s_verbose ? "\r\nNO CARRIER\r\n" : "3\r\n"); }
/* DNS resolution failed -- distinct from a connect-time NO CARRIER so the
 * host (WGET) can say "host not resolved" instead of a generic failure.
 * Hayes code 6 = NO DIALTONE, the closest standard "couldn't even start
 * to dial" semantic to "the name doesn't resolve". */
static void r_nodialtone(void) { if (!s_quiet) cdc_print(s_verbose ? "\r\nNO DIALTONE\r\n" : "6\r\n"); }

/* ---- telnet helpers ---- */

static void tn_send3(uint8_t cmd, uint8_t opt) {
    uint8_t b[3] = {TN_IAC, cmd, opt};
    if (s_sock >= 0) send(s_sock, b, 3, 0);
}
static bool tn_accept_remote(uint8_t o) { return o == OPT_BINARY || o == OPT_SGA || o == OPT_ECHO; }
static bool tn_offer_local(uint8_t o)   { return o == OPT_BINARY || o == OPT_SGA || o == OPT_TTYPE || o == OPT_NAWS; }
static void tn_offer_will(uint8_t opt) {
    if (!bget(s_local_offered, opt)) { bset(s_local_offered, opt); tn_send3(TN_WILL, opt); }
}
/* Fire a DSR-CPR probe at usbterm to discover its current row/col size.
 * Park the cursor at extreme bottom-right (usbterm clamps to its usable
 * region) and ask for a cursor-position report. usbterm's reply
 * (ESC[<rows>;<cols>R) is captured by the on_cdc_rx interceptor below.
 * Caller waits briefly for s_cpr_pending to clear before proceeding. */
static void tn_query_size(void) {
    s_cpr_pending = true;
    s_cpr_len = 0;
    s_cpr_deadline_us = esp_timer_get_time() + CPR_TIMEOUT_US;
    /* Bottom-right park + DSR-CPR. Two CSI sequences, 11 bytes total. */
    cdc_print("\x1b[999;999H\x1b[6n");
}

/* Feed bytes received while s_cpr_pending into the CPR parser. State is
 * a simple scan for ESC '[' <digits> ';' <digits> 'R'. Anything before
 * the ESC is discarded (so prompt fragments don't confuse the parser).
 * Returns the number of bytes consumed; caller should NOT process those
 * via the regular CDC paths. */
static size_t cpr_feed(const uint8_t *buf, size_t n) {
    size_t i = 0;
    while (i < n && s_cpr_pending) {
        uint8_t b = buf[i++];
        if (s_cpr_len == 0) {
            if (b == 0x1B) s_cpr_buf[s_cpr_len++] = b;
            /* else: drop noise before ESC */
        } else if (s_cpr_len < sizeof s_cpr_buf - 1) {
            s_cpr_buf[s_cpr_len++] = b;
            if (b == 'R') {
                /* ESC [ rows ; cols R */
                s_cpr_buf[s_cpr_len] = 0;
                long rows = 0, cols = 0;
                if (s_cpr_buf[1] == '[' &&
                    sscanf((const char *)s_cpr_buf + 2, "%ld;%ld", &rows, &cols) == 2 &&
                    rows >= 1 && rows <= 65535 &&
                    cols >= 1 && cols <= 65535) {
                    s_term_rows = (uint16_t)rows;
                    s_term_cols = (uint16_t)cols;
                }
                s_cpr_pending = false;
                s_cpr_len = 0;
                break;
            }
        } else {
            /* Overflow without finding R -- give up, treat the rest as
             * normal input. Caller will fall back to defaults. */
            s_cpr_pending = false;
            s_cpr_len = 0;
            /* Pretend we didn't consume this byte so caller re-processes. */
            return i - 1;
        }
    }
    return i;
}

static void tn_send_naws(void) {
    uint8_t b[] = {
        TN_IAC, TN_SB, OPT_NAWS,
        (uint8_t)(s_term_cols >> 8), (uint8_t)(s_term_cols & 0xFF),
        (uint8_t)(s_term_rows >> 8), (uint8_t)(s_term_rows & 0xFF),
        TN_IAC, TN_SE
    };
    if (s_sock >= 0) send(s_sock, b, sizeof b, 0);
}
static void tn_after_local_enable(uint8_t opt) {
    if (opt == OPT_NAWS) tn_send_naws();
}
static void tn_handle_neg(uint8_t c, uint8_t opt) {
    switch (c) {
        case TN_WILL:
            /* Password-safety latch: a "server ever said WILL ECHO" in
             * this session disables auto-local-echo for the rest of
             * the dial. After login the server typically sends WONT
             * ECHO before a password prompt; the strict "echo when
             * remote isn't echoing" rule would leak the password. */
            if (opt == OPT_ECHO) s_remote_ever_echoed = true;
            if (tn_accept_remote(opt)) {
                if (!bget(s_remote_on, opt)) { bset(s_remote_on, opt); tn_send3(TN_DO, opt); }
            } else {
                bclr(s_remote_on, opt); tn_send3(TN_DONT, opt);
            }
            break;
        case TN_WONT:
            if (bget(s_remote_on, opt)) { bclr(s_remote_on, opt); tn_send3(TN_DONT, opt); }
            break;
        case TN_DO:
            if (tn_offer_local(opt)) {
                if (!bget(s_local_on, opt)) {
                    bset(s_local_on, opt);
                    if (!bget(s_local_offered, opt)) { bset(s_local_offered, opt); tn_send3(TN_WILL, opt); }
                    tn_after_local_enable(opt);
                }
            } else {
                tn_send3(TN_WONT, opt);
            }
            break;
        case TN_DONT:
            if (bget(s_local_on, opt)) { bclr(s_local_on, opt); tn_send3(TN_WONT, opt); }
            break;
    }
}
static void tn_handle_sb(void) {
    if (s_sbopt == OPT_TTYPE && s_sblen >= 1 && s_sbbuf[0] == TT_SEND) {
        uint8_t hdr[] = {TN_IAC, TN_SB, OPT_TTYPE, TT_IS};
        if (s_sock >= 0) {
            send(s_sock, hdr, sizeof hdr, 0);
            send(s_sock, (const uint8_t *)s_term_type, strlen(s_term_type), 0);
            uint8_t tail[] = {TN_IAC, TN_SE};
            send(s_sock, tail, sizeof tail, 0);
        }
    }
}
static void tn_start(void) {
    memset(s_remote_on, 0, sizeof s_remote_on);
    memset(s_local_on, 0, sizeof s_local_on);
    memset(s_local_offered, 0, sizeof s_local_offered);
    s_remote_ever_echoed = false;
    s_prev_was_cr = false;
    s_sblen = 0; s_tstate = T_DATA;
    if (!s_telnet) return;
    /* Opener: offer the options that make a BBS session clean. */
    tn_offer_will(OPT_TTYPE);
    tn_offer_will(OPT_NAWS);
    tn_offer_will(OPT_SGA);    tn_send3(TN_DO, OPT_SGA);
    tn_offer_will(OPT_BINARY); tn_send3(TN_DO, OPT_BINARY);
}

/* ---- TCP -> CDC pump (drains payload bytes, runs telnet IAC machine
 * inline so option negotiation never reaches the user terminal) ---- */

/* Helper: push bytes to the CDC pipeline. Producer side. Blocks
 * (with a generous timeout) only if the StreamBuffer is full, which
 * means the CDC drain side is genuinely behind -- back-pressure all
 * the way to recv() is the correct behaviour there. */
static inline void to_cdc(const void *buf, size_t n) {
    if (!s_to_cdc || n == 0) return;
    /* Drain n bytes fully into the StreamBuffer; retry the tail if the
     * ring is full (slow CHUSB / macOS host). Previously a single
     * xStreamBufferSend with 500 ms timeout would return short and the
     * caller (modem_data_task) ignored the return -- the unwritten tail
     * was silently dropped, truncating the byte stream the host sees.
     * Bail early if s_online drops (ATH or peer close) so we don't
     * spin forever holding telnet IAC bytes the host will never read. */
    const uint8_t *p = (const uint8_t *)buf;
    size_t off = 0;
    while (off < n) {
        size_t w = xStreamBufferSend(s_to_cdc, p + off, n - off,
                                     pdMS_TO_TICKS(100));
        off += w;
        if (w == 0 && !s_online) break;
    }
}

/* CDC pump (consumer side). Lives parallel to the recv loop so a CDC
 * FIFO stall doesn't starve WiFi RX.
 *
 * PERSISTENT TASK: created once in modem_init when boot heap is high
 * (~44 KB free), then parks on a task notification between dials. This
 * keeps task-stack allocation entirely off the dial path -- at
 * WB_SLOTS=16 the dial-time free heap is only ~25 KB and a telnet
 * server flooding its banner on connect could leave too little to
 * spawn three ~3-4 KB stacks (observed: pump-spawn failure -> NO
 * CARRIER on freechess). cmd_dial wakes this via xTaskNotifyGive; the
 * inner drain loop runs until the dial ends, then we loop back to
 * park. */
static void modem_cdc_pump_task(void *arg) {
    (void)arg;
    /* 2 KB drains: one chunk equals one full prior CDC TX FIFO, so
     * each cdc_write call moves a "USB-transfer-worth" of data in
     * one go and the per-call flush overhead amortizes over more
     * bytes. With TX_BUFSIZE=4096 the ring still has 2 KB headroom
     * for a USB IN transfer in flight while we stage the next
     * chunk -- continuous bulk-IN pipelining, no idle slots. */
    static uint8_t buf[1024];   /* SRAM trim 2026-06-02 (was 2048) */
    for (;;) {
        /* Park until cmd_dial signals a new session. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (s_online || (s_to_cdc && xStreamBufferBytesAvailable(s_to_cdc) > 0)) {
            size_t n = xStreamBufferReceive(s_to_cdc, buf, sizeof buf,
                                            pdMS_TO_TICKS(50));
            if (n > 0) {
                /* Drain n bytes fully -- cdc_write has a 500 ms internal
                 * deadline and returns the number it actually managed to
                 * push. If the host (CHUSB / macOS) was slow draining the
                 * USB IN endpoint, cdc_write would return short, the
                 * tail would be silently lost, and the TCP stream the
                 * host sees gets corrupted in the middle (HTTP body /
                 * telnet payload truncated). Retry the tail until either
                 * we drain it all or the host has gone away. The producer
                 * (modem_data_task) is already back-pressured by
                 * xStreamBufferSend so this can never run unbounded. */
                size_t off = 0;
                while (off < n) {
                    size_t w = cdc_write(buf + off, n - off);
                    off += w;
                    if (w == 0) {
                        if (!tud_cdc_n_connected(0)) break;
                        vTaskDelay(pdMS_TO_TICKS(5));
                    }
                }
            }
        }
    }
}

/* Auto-detect telnet by the connection OPENING with IAC negotiation -- works
 * on ANY port, no port gate. A telnet/BBS server leads with IAC WILL/WONT/DO/
 * DONT (or SB) as the very first bytes. An HTTP response ALWAYS opens with
 * "HTTP/" (byte 0 = 'H', 0x48), never 0xFF, regardless of header length or
 * port -- so a download can never false-trigger (its body's stray 0xFF only
 * ever appears AFTER the headers, never at offset 0). Checking just the
 * opening byte is what makes this both port-independent AND HTTP-safe.
 * Returns 1=telnet, 0=not telnet (commit binary), -1=need another byte
 * (a lone leading IAC split across recv chunks). */
static int telnet_opens_with_iac(const uint8_t *b, int n) {
    if (n < 1)            return -1;
    if (b[0] != TN_IAC)   return 0;     /* HTTP 'H', ANSI ESC, plain text -> binary */
    if (n < 2)            return -1;    /* lone IAC so far -- wait for the command byte */
    uint8_t c = b[1];
    return (c == TN_WILL || c == TN_WONT || c == TN_DO ||
            c == TN_DONT || c == TN_SB) ? 1 : 0;
}

static void modem_data_task(void *arg) {
    (void)arg;
    /* Sized to the CDC TX FIFO (2 KB) so one recv() worth of bytes fits
     * in a single FIFO drain without spinning on backpressure. outbuf
     * only used on the telnet path; binary path pushes inbuf directly. */
    static uint8_t inbuf[1024];   /* SRAM trim 2026-06-02 (was 2048) */
    static uint8_t outbuf[1024];  /* telnet path flushes incrementally as it fills */

    for (;;) {
    /* Park until cmd_dial signals a new session (persistent task --
     * see modem_cdc_pump_task comment for why we don't create/delete
     * per dial). */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* Loose 100 ms poll. The recv timeout means we wake periodically
     * to check for +++ guard expiry / socket close even when the peer
     * is silent. */
    struct timeval rcv_tv = { .tv_sec = 0, .tv_usec = 100 * 1000 };
    setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &rcv_tv, sizeof rcv_tv);

    /* Auto-telnet: if telnet handling is off (e.g. ATNET0 left over from a
     * WGET) and the peer OPENS the connection with IAC negotiation, switch
     * telnet handling on. Port-independent (works for BBSes on any port) and
     * HTTP-safe (HTTP opens with "HTTP/", never IAC -- see telnet_opens_with_iac). */
    bool autosniff = !s_telnet;

    /* Adaptive RX-window controller (BUFFER-CONTROL-2026-06-04.md). Owns the
     * receive window for this session from here: starts moderate, then sizes to
     * consumer keep-up + heap headroom each tick. */
    int      win      = WINCTL_START;
    setsockopt(s_sock, SOL_SOCKET, SO_RCVBUF, &win, sizeof win);
    int64_t  ctl_last = esp_timer_get_time();
    uint64_t ctl_blk  = s_tp_blk_us;

    bool peer_closed = false;

    while (s_online && s_sock >= 0) {
        /* Anti-wedge: bail before starvation can take out the OTA paths.
         * Close the socket FIRST (frees the held RX pbuf backlog -> heap
         * jumps back up) and deliberately SKIP the usual CDC-drain wait:
         * a stalled CDC is the very thing this guards against, so we must
         * not block on it. A few in-flight bytes are sacrificed -- the
         * stream is being torn down anyway and NO CARRIER tells DOS so. */
        uint32_t freeb = esp_get_free_heap_size();
        if (freeb < LOW_HEAP_GUARD_BYTES) {
            disk_logf("relay: LOW HEAP %u<%u -- abort session (anti-wedge)",
                      (unsigned)freeb, (unsigned)LOW_HEAP_GUARD_BYTES);
            s_online = false;
            if (s_sock >= 0) { close(s_sock); s_sock = -1; }
            s_peer[0] = 0;
            r_nocarrier();
            break;
        }

        /* --- fast heap clamp (per-iteration, NOT tick-gated) --- */
        /* A 16K window can pile into heap faster than the 500ms tick reacts: in
         * testing the floor dipped to 7.7K (below WINCTL_HEAP_LOW) before the
         * tick shrank it. So shrink to the floor IMMEDIATELY the moment heap
         * crosses the clamp line; the tick below re-grows once heap recovers
         * above WINCTL_HEAP_HIGH. Different heap regimes => no oscillation. */
        if (freeb < WINCTL_HEAP_LOW && win > WINCTL_MIN) {
            /* Gentle step (-2*STEP), same law as the tick's clamp branch but
             * per-iteration: re-fires next chunk if still low, so it converges
             * progressively rather than slamming to MIN and lurching throughput.
             * Already-buffered data still has to drain, so the floor overshoots
             * this trigger by a few KB -- WINCTL_HEAP_LOW carries the margin. */
            int newin = win - 2*WINCTL_STEP;
            if (newin < WINCTL_MIN) newin = WINCTL_MIN;
            setsockopt(s_sock, SOL_SOCKET, SO_RCVBUF, &newin, sizeof newin);
            disk_logf("winctl FAST-clamp %d->%d heap=%u", win, newin, (unsigned)freeb);
            win = newin;
            ctl_blk = s_tp_blk_us;   /* don't fold this into the next tick's blk_delta */
        }

        /* --- adaptive window controller tick (time-gated, ~500ms) --- */
        int64_t nowus = esp_timer_get_time();
        if (nowus - ctl_last >= WINCTL_TICK_US) {
            uint64_t blk = s_tp_blk_us, blk_delta = blk - ctl_blk;
            int newin = win;
            if      (freeb < WINCTL_HEAP_LOW)  newin = win - 2*WINCTL_STEP; /* clamp: heap tight */
            else if (blk_delta > WINCTL_SAT_US) newin = win - WINCTL_STEP;  /* consumer-bound */
            else if (freeb > WINCTL_HEAP_HIGH)  newin = win + WINCTL_STEP;  /* headroom: grow */
            if (newin < WINCTL_MIN) newin = WINCTL_MIN;
            if (newin > WINCTL_MAX) newin = WINCTL_MAX;
            if (newin != win) {
                setsockopt(s_sock, SOL_SOCKET, SO_RCVBUF, &newin, sizeof newin);
                disk_logf("winctl %d->%d heap=%u blk=%ums", win, newin,
                          (unsigned)freeb, (unsigned)(blk_delta/1000));
                win = newin;
            }
            ctl_last = nowus; ctl_blk = blk;
        }

        int n = recv(s_sock, inbuf, sizeof inbuf, 0);
        if (n > 0) s_tp_rx += (uint32_t)n;
        if (n > 0 && autosniff) {
            /* Decide from the connection OPENING; flip BEFORE the dispatch
             * below so this chunk routes through the IAC machine (negotiation
             * processed, not relayed raw). */
            int r = telnet_opens_with_iac(inbuf, n);
            if (r > 0) {
                s_telnet  = true;
                autosniff = false;
                tn_start();   /* proper entry: T_DATA reset + send our option offers */
                disk_logf("modem: auto-detected telnet (opens with IAC) -> telnet mode");
            } else if (r == 0) {
                autosniff = false;   /* opens with non-IAC (HTTP 'H', ANSI, text) -- binary */
            }
            /* r < 0: lone leading IAC split across recv chunks -- keep sniffing */
        }
        if (n > 0 && !s_telnet) {
            /* Binary fast path: skip the per-byte IAC state machine when
             * telnet is off -- push inbuf straight into the CDC pipeline.
             * Phase 2 (HTTP) hits this. */
            to_cdc(inbuf, (size_t)n);
        } else if (n > 0) {
            size_t outlen = 0;
            for (int i = 0; i < n; ++i) {
                uint8_t ch = inbuf[i];
                switch (s_tstate) {
                    case T_DATA:
                        if (ch == TN_IAC) s_tstate = T_IAC;
                        else if (ch == 0x00 && s_prev_was_cr &&
                                 !bget(s_remote_on, OPT_BINARY)) {
                            /* NVT CR-NUL collapses to CR -- drop the NUL.
                             * Without this, hosts like BSD telnetd would
                             * leak \0 bytes through to usbterm after every
                             * bare CR. Skipped in BINARY mode where the
                             * NUL is real data. */
                            s_prev_was_cr = false;
                        } else {
                            outbuf[outlen++] = ch;
                            s_prev_was_cr = (ch == 0x0D);
                            if (outlen >= sizeof outbuf) { to_cdc(outbuf, outlen); outlen = 0; }
                        }
                        break;
                    case T_IAC:
                        if (ch == TN_IAC) {
                            outbuf[outlen++] = TN_IAC;
                            s_prev_was_cr = false;
                            if (outlen >= sizeof outbuf) { to_cdc(outbuf, outlen); outlen = 0; }
                            s_tstate = T_DATA;
                        } else if (ch == TN_WILL || ch == TN_WONT || ch == TN_DO || ch == TN_DONT) {
                            s_tcmd = ch; s_tstate = T_OPT;
                        } else if (ch == TN_SB) {
                            s_tstate = T_SB_OPT;
                        } else {
                            s_tstate = T_DATA;
                        }
                        break;
                    case T_OPT:
                        tn_handle_neg(s_tcmd, ch); s_tstate = T_DATA;
                        break;
                    case T_SB_OPT:
                        s_sbopt = ch; s_sblen = 0; s_tstate = T_SB_DATA;
                        break;
                    case T_SB_DATA:
                        if (ch == TN_IAC) s_tstate = T_SB_IAC;
                        else if (s_sblen < sizeof s_sbbuf) s_sbbuf[s_sblen++] = ch;
                        break;
                    case T_SB_IAC:
                        if (ch == TN_SE) { tn_handle_sb(); s_tstate = T_DATA; }
                        else { if (s_sblen < sizeof s_sbbuf) s_sbbuf[s_sblen++] = ch; s_tstate = T_SB_DATA; }
                        break;
                }
            }
            if (outlen) to_cdc(outbuf, outlen);
        } else if (n == 0) {
            peer_closed = true;
            break;
        } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            disk_logf("modem: recv err=%d", errno);
            peer_closed = true;
            break;
        }

        /* +++ guard expired -> command mode, socket stays open (ATO returns). */
        int64_t now = esp_timer_get_time();
        if (s_plus_count == 3 && (now - s_plus_time_us) > GUARD_US) {
            s_plus_count = 0;
            /* Drain the full CDC pipeline (stream buffer AND USB FIFO) before
             * printing OK, so all in-flight payload lands before the prompt --
             * same EOF-tail issue as the peer_closed path. Socket stays open
             * here (ATO can resume), so we don't close it. */
            while (s_to_cdc && xStreamBufferBytesAvailable(s_to_cdc) > 0)
                vTaskDelay(pdMS_TO_TICKS(2));
            s_online = false;
            vTaskDelay(pdMS_TO_TICKS(10));
            drain_cdc_fifo();
            r_ok();
            break;
        }
    }

    if (peer_closed) {
        /* Drain the ENTIRE CDC pipeline to the host before closing, in order:
         *   1. stream buffer -> FIFO   (pump, while s_online still true)
         *   2. let the pump finish its last in-hand chunk and go idle
         *   3. FIFO -> host            (drain_cdc_fifo, the EOF-tail fix)
         * The old code did only (1), so the last FIFO-load of the file was
         * dropped when we close()'d -- downloads truncated ~8 KB short at EOF. */
        while (s_to_cdc && xStreamBufferBytesAvailable(s_to_cdc) > 0)
            vTaskDelay(pdMS_TO_TICKS(2));
        s_online = false;                      /* pump parks once s_to_cdc is empty */
        vTaskDelay(pdMS_TO_TICKS(10));          /* let it write its last chunk + idle */
        drain_cdc_fifo();                       /* wait for the host to pull the tail */
        if (s_sock >= 0) { close(s_sock); s_sock = -1; }
        s_peer[0] = 0;
        r_nocarrier();
        drain_cdc_fifo();                       /* and that NO CARRIER lands too */
    }
    /* Loop back to park for the next dial. */
    }
}

/* Whether the dongle should locally echo CDC RX back to the host in
 * data mode. AUTO follows the negotiated state with a "remote ever
 * WILL'd ECHO" latch so the password-prompt flow (server WILL ECHO ->
 * login -> server WONT ECHO) doesn't leak the password. ON / OFF are
 * explicit overrides settable via AT$LECHO. See
 * TELNET-LOCAL-ECHO-2026-06-01.md for the full rationale. */
static inline bool local_echo_active(void) {
    if (s_lecho_mode == LECHO_OFF) return false;
    if (s_lecho_mode == LECHO_ON)  return s_telnet && s_online;
    return s_telnet && s_online &&
           !bget(s_remote_on, OPT_ECHO) &&
           !s_remote_ever_echoed;
}

/* Echo CDC RX bytes back to the host per local-echo rules. Called only
 * when local_echo_active() is true. */
static void do_local_echo(const uint8_t *buf, size_t n) {
    uint8_t out[64];
    size_t  o = 0;
    bool    bin = bget(s_local_on, OPT_BINARY);
    for (size_t i = 0; i < n; ++i) {
        uint8_t b = buf[i];
        if (o + 4 > sizeof out) { cdc_write(out, o); o = 0; }
        if (b >= 0x20 && b <= 0x7E) {
            out[o++] = b;
        } else if (b == '\r') {
            out[o++] = '\r'; out[o++] = '\n';
        } else if (b == 0x08 || b == 0x7F) {
            /* Destructive BS: usbterm treats lone 0x08 as cursor-left
             * only, so emit BS-space-BS to actually erase the glyph. */
            out[o++] = 0x08; out[o++] = ' '; out[o++] = 0x08;
        } else if (bin && b >= 0x80) {
            out[o++] = b;
        }
        /* All other controls (IAC, NUL, LF, ESC, etc.) are skipped -- the
         * server's response will paint them if they have meaning. */
    }
    if (o) cdc_write(out, o);
}

/* ---- CDC -> TCP push (producer side, called from on_cdc_rx) ---- */

/* Hand raw CDC RX bytes off to the TCP pump task. Producer is the
 * TinyUSB callback on CPU1; consumer runs on CPU0 with lwIP. Same
 * back-pressure model as to_cdc(): if the stream is full the
 * TinyUSB task blocks briefly, which is correct -- the host should
 * also stall its OUT EP rather than letting bytes pile up nowhere. */
static void online_push_bytes(const uint8_t *buf, size_t n) {
    if (!s_to_tcp || n == 0) return;
    /* Drain n bytes fully -- same rationale as to_cdc(). The producer
     * is on_cdc_rx on the TinyUSB task; blocking here back-pressures
     * the USB OUT endpoint (host stops sending), which is what we
     * want when modem_tcp_pump can't keep up with send() to a slow
     * peer. Bail early if s_online drops. */
    size_t off = 0;
    while (off < n) {
        size_t w = xStreamBufferSend(s_to_tcp, buf + off, n - off,
                                     pdMS_TO_TICKS(100));
        off += w;
        if (w == 0 && !s_online) break;
    }
}

/* Drain a chunk to the TCP socket, retrying short sends until either
 * the whole chunk has gone out or the socket has been closed under us
 * (ATH, peer close). Without this, send() returning short on a tight
 * lwIP TX window silently truncates the byte stream the server sees
 * -- a half-written HTTP request hangs the server until it gives up.
 * Caller is the modem_tcp_pump_task, which has no time pressure --
 * the CDC RX path is already back-pressured by xStreamBufferSend so
 * we can wait as long as we need to. */
static void tcp_send_all(const uint8_t *buf, size_t n) {
    size_t off = 0;
    while (off < n && s_online && s_sock >= 0) {
        int w = send(s_sock, buf + off, n - off, 0);
        if (w > 0) {
            off += (size_t)w;
            continue;
        }
        if (w == 0) break;
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        /* Hard error -- log and bail; the data task's recv() will
         * pick up the disconnect and fire NO CARRIER. */
        disk_logf("tcp_send_all: send err=%d off=%u/%u", errno,
                  (unsigned)off, (unsigned)n);
        break;
    }
}

/* ---- CDC -> TCP pump (consumer side) ----
 *
 * Runs on CPU0 with lwIP, so send() doesn't bounce across cores. Does
 * the per-byte +++ guard, telnet IAC escaping, NVT CR->CRLF expansion
 * + a coalescing send(). Exits when online drops AND the inbound
 * stream is drained. */
static void modem_tcp_pump_task(void *arg) {
    (void)arg;
    static uint8_t inbuf[1024];
    static uint8_t txbuf[2048];  /* 2x for worst-case IAC + CRLF expansion */

    for (;;) {
    /* Park until cmd_dial signals a new session (persistent task). */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (s_online || (s_to_tcp && xStreamBufferBytesAvailable(s_to_tcp) > 0)) {
        size_t got = xStreamBufferReceive(s_to_tcp, inbuf, sizeof inbuf,
                                          pdMS_TO_TICKS(50));
        if (got == 0 || s_sock < 0) continue;

        size_t txlen = 0;
        int64_t now = esp_timer_get_time();
        for (size_t i = 0; i < got; ++i) {
            uint8_t ch = inbuf[i];
            if (ch == '+' && s_plus_count < 3 &&
                (s_plus_count > 0 || (now - s_last_data_us) > GUARD_US)) {
                /* Hold the +'s back until we know whether the escape
                 * completes (3 +'s + 1 s silence) or breaks. */
                if (txlen) { tcp_send_all(txbuf, txlen); txlen = 0; }
                s_plus_count++;
                s_plus_time_us = now;
                continue;
            }
            /* Any non-'+' (or 4th '+', etc.) breaks the escape: flush
             * the pending +'s as real data, then process this byte. */
            while (s_plus_count > 0) { txbuf[txlen++] = '+'; s_plus_count--; }

            if (s_telnet && ch == 0x0D && !bget(s_local_on, OPT_BINARY)) {
                txbuf[txlen++] = 0x0D;
                txbuf[txlen++] = 0x0A;
            } else {
                if (s_telnet && ch == TN_IAC) txbuf[txlen++] = TN_IAC;
                txbuf[txlen++] = ch;
            }
            s_last_data_us = now;

            if (txlen >= sizeof txbuf - 2) {
                tcp_send_all(txbuf, txlen); txlen = 0;
            }
        }
        if (txlen) tcp_send_all(txbuf, txlen);
    }
    /* Loop back to park for the next dial. */
    }
}

/* ---- AT$ command handlers ---- */

/* Defined in main.c. Applies creds to the running radio and connects, but
 * persists to NVS only after a verified GOT_IP -- so a bad/stray command (or
 * CDC-line garbage parsed as AT) can never overwrite the working network.
 * This is why none of the handlers below write NVS directly any more. */
extern void wifi_provision(const char *ssid, const char *pass);

/* Staged single-field creds from AT$SSID=/AT$PASS=. Held here (never in NVS)
 * until the next bare AT$WIFI applies them via wifi_provision. */
static char s_stage_ssid[33];
static bool s_have_stage_ssid = false;
static char s_stage_pass[65];
static bool s_have_stage_pass = false;

static void cmd_wifi_set(const char *arg) {
    /* AT$WIFI=ssid,password -- comma is the separator; ssids with
     * literal commas aren't supported (matches old firmware). */
    const char *comma = strchr(arg, ',');
    if (!comma) { r_error(); return; }
    char ssid[33] = {0};
    char pass[65] = {0};
    size_t sl = (size_t)(comma - arg);
    if (sl == 0 || sl >= sizeof ssid) { r_error(); return; }
    memcpy(ssid, arg, sl);
    strncpy(pass, comma + 1, sizeof pass - 1);

    cdc_print("\r\nconnecting...\r\n");
    wifi_provision(ssid, pass);   /* applies + connects; NVS write deferred to GOT_IP */
    r_ok();
}

static void cmd_wifi_query(void) {
    cdc_print("\r\n");
    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        char line[128];
        snprintf(line, sizeof line,
                 "SSID:    %.32s\r\n"
                 "RSSI:    %d dBm\r\n"
                 "Channel: %u\r\n",
                 (const char *)ap.ssid, (int)ap.rssi, (unsigned)ap.primary);
        cdc_print(line);
        esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (sta) {
            esp_netif_ip_info_t ip = {0};
            esp_netif_get_ip_info(sta, &ip);
            char ips[128];
            snprintf(ips, sizeof ips,
                     "IP:      " IPSTR "\r\n"
                     "GW:      " IPSTR "\r\n",
                     IP2STR(&ip.ip), IP2STR(&ip.gw));
            cdc_print(ips);
        }
        cdc_print("status:  connected\r\n");
    } else {
        cdc_print("status:  not connected\r\n");
    }
}

static void cmd_dns(const char *host) {
    cdc_print("\r\n");
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res = NULL;
    if (getaddrinfo(host, NULL, &hints, &res) != 0 || !res) {
        cdc_print("NXDOMAIN\r\n");
        r_error();
        return;
    }
    char ip[INET_ADDRSTRLEN];
    struct sockaddr_in *sin = (struct sockaddr_in *)res->ai_addr;
    inet_ntop(AF_INET, &sin->sin_addr, ip, sizeof ip);
    cdc_print(ip); cdc_print("\r\n");
    freeaddrinfo(res);
    r_ok();
}

static void cmd_ping(const char *host) {
    /* Lightweight TCP-handshake "ping" (port 80). Real ICMP needs
     * lwIP's raw socket setup which is heavier; the TCP variant is
     * what the old firmware shipped. */
    cdc_print("\r\n");
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res = NULL;
    if (getaddrinfo(host, "80", &hints, &res) != 0 || !res) {
        cdc_print("NXDOMAIN\r\n");
        r_error();
        return;
    }
    int s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (s < 0) { freeaddrinfo(res); r_error(); return; }
    struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    int64_t t0 = esp_timer_get_time();
    int ok = connect(s, res->ai_addr, res->ai_addrlen);
    int64_t dt = esp_timer_get_time() - t0;
    close(s);
    freeaddrinfo(res);
    char line[64];
    if (ok == 0)
        snprintf(line, sizeof line, "reply %d ms\r\n", (int)(dt / 1000));
    else
        snprintf(line, sizeof line, "timeout (%d ms)\r\n", (int)(dt / 1000));
    cdc_print(line);
    r_ok();
}

static void cmd_scan(void) {
    cdc_print("\r\nscanning...\r\n");
    /* The wifi event handler reconnects-on-disconnect aggressively, so a
     * scan started while connect-retry is in flight returns ESP_ERR_WIFI_STATE.
     * Disconnect first and the next reconnect attempt will fire after we
     * finish; this lets scan succeed even when creds are wrong. */
    esp_wifi_disconnect();
    wifi_scan_config_t sc = {0};
    esp_err_t e = esp_wifi_scan_start(&sc, true);
    if (e != ESP_OK) {
        char b[64];
        snprintf(b, sizeof b, "scan failed (%s)\r\n", esp_err_to_name(e));
        cdc_print(b);
        r_error();
        return;
    }
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n > 24) n = 24;
    char line[128];
    snprintf(line, sizeof line, "%u networks:\r\n", (unsigned)n);
    cdc_print(line);
    if (n == 0) { r_ok(); return; }
    wifi_ap_record_t *ap = calloc(n, sizeof *ap);
    if (!ap) { cdc_print("calloc fail\r\n"); r_error(); return; }
    esp_wifi_scan_get_ap_records(&n, ap);
    for (uint16_t i = 0; i < n; ++i) {
        snprintf(line, sizeof line,
                 "  %-32.32s  ch=%-2u  rssi=%4d  auth=%u\r\n",
                 (const char *)ap[i].ssid,
                 (unsigned)ap[i].primary,
                 (int)ap[i].rssi,
                 (unsigned)ap[i].authmode);
        cdc_print(line);
    }
    free(ap);
    r_ok();
}

static void cmd_netif(void) {
    cdc_print("\r\n");
    esp_netif_t *netif = NULL;
    char buf[160];
    for (netif = esp_netif_next_unsafe(NULL); netif; netif = esp_netif_next_unsafe(netif)) {
        esp_netif_ip_info_t ip = {0};
        esp_netif_get_ip_info(netif, &ip);
        snprintf(buf, sizeof buf,
                 "%-12s  ip=" IPSTR "  gw=" IPSTR "  mask=" IPSTR "\r\n",
                 esp_netif_get_ifkey(netif),
                 IP2STR(&ip.ip), IP2STR(&ip.gw), IP2STR(&ip.netmask));
        cdc_print(buf);
    }
    r_ok();
}

/* ---- ATD: dial a host:port ---- */

/* Worker that does the heavy dial work off the TinyUSB task.
 * Reads the dial argument from s_dial_arg (populated by cmd_dial under
 * the s_at_busy guard, so no concurrent writer). */
static void cmd_dial_impl(const char *arg) {
    /* Hang up any lingering connection before dialing (standard modem
     * behaviour). Without this, a +++ escape leaves s_sock OPEN (for ATO)
     * with no task draining it -- the peer keeps sending, lwIP buffers a
     * full TCP window of pbufs nobody consumes, and a re-dial then LEAKS
     * that socket (unclosable until reboot, still holding ~8 KB of pbufs).
     * Repeated across a session this bleeds heap to starvation -> wedge.
     * (2026-06-03 leak fix; see never-starve directive.) */
    if (s_sock >= 0) {
        s_online = false;
        vTaskDelay(pdMS_TO_TICKS(20));   /* let the pump/data tasks park */
        close(s_sock);
        s_sock = -1;
        s_peer[0] = 0;
    }

    char host[80];
    uint16_t port = 23;     /* Hayes default = telnet */
    const char *colon = strrchr(arg, ':');
    if (colon) {
        size_t n = (size_t)(colon - arg);
        if (n >= sizeof host) n = sizeof host - 1;
        memcpy(host, arg, n); host[n] = 0;
        int p = atoi(colon + 1);
        if (p > 0 && p < 65536) port = (uint16_t)p;
    } else {
        strncpy(host, arg, sizeof host - 1); host[sizeof host - 1] = 0;
    }

    /* Need WiFi up to resolve and dial. */
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) { r_nocarrier(); return; }

    /* Probe the host terminal size NOW, before we open the socket.
     * The CPR probe is a CDC round-trip to the host (usbterm), totally
     * independent of the TCP connection -- doing it here means the
     * up-to-500ms wait happens during the idle pre-connect phase, NOT
     * in the critical window between connect() and the recv() pump
     * spawning. That window matters because a telnet server (freechess,
     * BBSes) blasts its banner the instant we connect; with no recv()
     * draining yet, lwIP buffers it on the heap. A 500ms CPR wait in
     * that window let freechess's banner + IAC negotiation pile up
     * ~14 KB of pbufs, dropping free heap from 20 KB to 6 KB and
     * starving the pump-task stacks (observed: pump spawn fail). */
    if (s_telnet) {
        disk_logf("dial: tn_query_size (pre-connect)");
        tn_query_size();
        while (s_cpr_pending && esp_timer_get_time() < s_cpr_deadline_us) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        disk_logf("dial: cpr done cols=%u rows=%u", s_term_cols, s_term_rows);
        s_cpr_pending = false;
    }

    char portstr[8];
    snprintf(portstr, sizeof portstr, "%u", (unsigned)port);
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res = NULL;
    if (getaddrinfo(host, portstr, &hints, &res) != 0 || !res) {
        disk_logf("modem: dial DNS-fail '%s'", host);
        r_nodialtone();   /* distinct from connect-fail NO CARRIER (host shows "not resolved") */
        return;
    }

    int sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) { freeaddrinfo(res); r_nocarrier(); return; }
    /* 20 s timeouts for the connect() itself. Once we're in the data
     * phase, modem_data_task overrides RCVTIMEO to 100 ms and tcp_send_all
     * relies on a bounded SNDTIMEO (3 s) so the pump task doesn't block
     * forever if the TCP window stays closed. */
    struct timeval ctv = { .tv_sec = 20, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &ctv, sizeof ctv);
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &ctv, sizeof ctv);

    if (connect(sock, res->ai_addr, res->ai_addrlen) < 0) {
        close(sock);
        freeaddrinfo(res);
        r_nocarrier();
        return;
    }
    freeaddrinfo(res);

    int yes = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof yes);
    /* Shrink the post-connect SNDTIMEO so tcp_send_all can't be wedged
     * for 20 s on a closed TCP window. 3 s is enough for transient
     * lwIP queue-full while still letting the pump task retry on a
     * sensible cadence. */
    struct timeval stv = { .tv_sec = 3, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof stv);
    /* Cap the per-socket receive window (needs LWIP_SO_RCVBUF). This is
     * THE reliability lever: it HARD-bounds how many pbufs one session can
     * hold, keeping the heap floor clear of the starvation cliff no matter
     * how slowly the consumer drains. (Earlier a 32 KB value was set here
     * and removed because going ABOVE the window grew the accept queue;
     * going BELOW it -- as we do now -- correctly shrinks the window.)
     *   - Telnet/interactive (Star Wars, BBSes): 2 KB. The DOS side renders
     *     ANSI char-by-char and is the real bottleneck; a small window
     *     flow-controls the server SOON, pacing the stream to the render
     *     rate (smooth) instead of buffering then bursting (the observed
     *     freeze-then-jump-ahead), and holds almost no pbufs.
     *   - Binary/HTTP (WGET): 4 KB, matching the global TCP_WND so a bulk
     *     fetch still pipelines a full window but can never hold more than
     *     one window of backlog.
     * 2026-06-03 reliability reset: a long telnet session with the old
     * 8 KB window ratcheted the heap floor to ~1.2 KB and the WiFi stack
     * went intermittently dark. Bounding the window fixes that by design. */
    int rcvbuf = s_telnet ? 2048 : 4096;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof rcvbuf);

    s_sock = sock;
    s_peer_port = port;
    snprintf(s_peer, sizeof s_peer, "%s:%u", host, (unsigned)port);
    s_plus_count = 0;
    s_last_data_us = esp_timer_get_time();
    disk_logf("dial: connected sock=%d peer=%s telnet=%d", sock, s_peer, s_telnet);

    /* IMPORTANT ORDERING for the async dial path. Because this function
     * runs on a worker task off the TinyUSB CPU, on_cdc_rx can fire on
     * CPU1 at any moment a host byte arrives. The host (PROFILE.EXE,
     * usbterm, etc.) sees CONNECT and immediately starts sending data
     * (a GET request, login, etc.), so we MUST have:
     *
     *   1. Stream buffers reset       (before any producer can write)
     *   2. CPR probe + handshake      (before pump tasks exist so the
     *                                  CPR reply doesn't race the
     *                                  freshly-spawned tcp pump)
     *   3. s_online = true            (so spawned pump tasks don't
     *                                  short-circuit on their main
     *                                  loop condition)
     *   4. tn_start                   (telnet IAC -- only meaningful
     *                                  in telnet mode)
     *   5. Spawn pump tasks           (now everything they need is
     *                                  set up, and the pipeline can
     *                                  drain the moment data arrives)
     *   6. r_connect                  (CONNECT printed last -- once
     *                                  this lands, the host will send
     *                                  bytes and on_cdc_rx pushes to
     *                                  a stream that has a running
     *                                  consumer).
     *
     * The prior order had r_connect before the reset+spawn block,
     * which created a real race: PROFILE sees CONNECT -> sends GET ->
     * on_cdc_rx queues to s_to_tcp on CPU1 -> cmd_dial_task on CPU0
     * concurrently runs xStreamBufferReset(s_to_tcp) and DISCARDS the
     * GET bytes. The server never sees the request and PROFILE times
     * out reading the HTTP status. */

    /* (1) Streams reset while no concurrent producers exist. */
    xStreamBufferReset(s_to_cdc);
    xStreamBufferReset(s_to_tcp);

    /* Reset throughput counters for this session (read via ATI). */
    s_tp_rx = 0; s_tp_cdc = 0; s_tp_blk_us = 0; s_tp_start = esp_timer_get_time();

    /* (2) Flip online flag BEFORE spawning tasks; modem_data_task's
     *     main loop is `while (s_online && s_sock >= 0)`, so if we
     *     spawned it with s_online=false it would exit on the first
     *     iteration. */
    s_online = true;

    /* (3) Telnet handshake to server (no-op when s_telnet is false).
     *     CPR probe already ran pre-connect, so this is the only work
     *     between connect() and the recv() pump going live -- a few
     *     send() calls, microseconds. */
    disk_logf("dial: tn_start");
    tn_start();

    /* (4) Wake the pre-created pump tasks. They were created once at
     *     modem_init (heap-cheap here -- no per-dial stack alloc) and
     *     are parked on a notification. Wake data_task first so recv()
     *     starts draining the socket immediately. No failure path:
     *     the tasks already exist, so a telnet banner flood on connect
     *     can't starve a task-create the way it used to. */
    disk_logf("dial: waking pumps free=%u",
              (unsigned)esp_get_free_heap_size());
    if (s_data_task)     xTaskNotifyGive(s_data_task);
    if (s_cdc_pump_task) xTaskNotifyGive(s_cdc_pump_task);
    if (s_tcp_pump_task) xTaskNotifyGive(s_tcp_pump_task);

    /* (5) CONNECT only after the pipeline is live. */
    disk_logf("dial: r_connect (free=%u)",
              (unsigned)esp_get_free_heap_size());
    r_connect();
}

/* Worker task wrapper: run cmd_dial_impl off the TinyUSB task so DNS +
 * connect + CPR wait don't block CDC servicing. Clears the busy gate
 * when done (result codes already emitted by cmd_dial_impl). */
static void cmd_dial_task_fn(void *arg) {
    (void)arg;
    disk_logf("dial_task: enter arg=\"%s\"", s_dial_arg);
    cmd_dial_impl(s_dial_arg);
    disk_logf("dial_task: cmd_dial_impl returned, clearing busy");
    s_at_busy = false;
    s_dial_task = NULL;
    vTaskDelete(NULL);
}

/* Thin synchronous wrapper called from exec(). Does the cheap syntax
 * checks inline (so a malformed ATD gets an immediate ERROR) and then
 * hands off to cmd_dial_task_fn for the slow work. */
static void cmd_dial(const char *arg) {
    while (*arg == ' ' || *arg == '\t') arg++;
    if (*arg == 'T' || *arg == 'P' || *arg == 'R') arg++;
    while (*arg == ' ' || *arg == '\t') arg++;
    if (!*arg) { r_error(); return; }
    if (s_at_busy || s_dial_task) { r_error(); return; }

    size_t n = strnlen(arg, sizeof s_dial_arg);
    if (n >= sizeof s_dial_arg) { r_error(); return; }
    memcpy(s_dial_arg, arg, n);
    s_dial_arg[n] = 0;

    s_at_busy = true;
    /* CPU0 keeps the worker off the TinyUSB CPU (1); lwIP also lives
     * on CPU0 so getaddrinfo/connect/recv all stay local. 4 KB stack
     * matches modem_data_task and is sufficient for getaddrinfo +
     * connect + the small cdc_print chain (verified empirically --
     * the panic we chased to "stack overflow" was actually heap
     * exhaustion at xStreamBufferCreate, not stack). */
    BaseType_t ok = xTaskCreatePinnedToCore(cmd_dial_task_fn, "at_dial",
                                            4096, NULL, 15,
                                            &s_dial_task, 0);
    if (ok != pdPASS) {
        s_at_busy = false;
        s_dial_task = NULL;
        r_error();
    }
    /* Result code (CONNECT / NO CARRIER) is emitted by cmd_dial_impl. */
}

/* ---- exec one AT line (command mode only) ---- */

static char *strip_at(char *line) {
    while (*line == ' ') line++;
    if (line[0] != 'A' && line[0] != 'a') return NULL;
    if (line[1] != 'T' && line[1] != 't') return NULL;
    char *rest = line + 2;
    /* Uppercase the command part only -- stop at '=' so case-sensitive
     * values (passwords, URLs, SSIDs) survive intact. Without this,
     * AT$PASS=Crusty jugglers landed in NVS as CRUSTY JUGGLERS and
     * WPA2 auth silently failed. */
    for (char *p = rest; *p && *p != '='; ++p)
        *p = (char)toupper((unsigned char)*p);
    return rest;
}

static void handle_dollar(char *s) {
    /* AT$<KEY>[=<value>|?] */
    char *eq  = strchr(s, '=');
    char *qm  = strchr(s, '?');
    char *key = s;
    char *val = NULL;
    if (eq) { *eq = 0; val = eq + 1; }
    else if (qm) { *qm = 0; val = qm; }   /* '?' query keeps key clean */

    if (!strcmp(key, "WIFI")) {
        if (val && eq)        cmd_wifi_set(val);
        else if (val && qm)   { cmd_wifi_query(); r_ok(); }
        else {
            /* Bare AT$WIFI -- (re)connect using staged AT$SSID=/AT$PASS=
             * values where given, otherwise the current NVS creds. NVS is
             * NOT written here; wifi_provision persists only after a verified
             * GOT_IP, so this can never clobber the working network. */
            char ssid[33] = {0}, pass[65] = {0};
            size_t ns = sizeof ssid, np = sizeof pass;
            nvs_handle_t hr;
            if (nvs_open("slip-router", NVS_READONLY, &hr) == ESP_OK) {
                nvs_get_str(hr, "ssid", ssid, &ns);
                nvs_get_str(hr, "pass", pass, &np);
                nvs_close(hr);
            }
            if (s_have_stage_ssid) {
                strncpy(ssid, s_stage_ssid, sizeof ssid - 1); ssid[sizeof ssid - 1] = 0;
            }
            if (s_have_stage_pass) {
                strncpy(pass, s_stage_pass, sizeof pass - 1); pass[sizeof pass - 1] = 0;
            }
            s_have_stage_ssid = s_have_stage_pass = false;
            cdc_print("\r\nreconnecting...\r\n");
            wifi_provision(ssid, pass);
            r_ok();
        }
    } else if (!strcmp(key, "SSID")) {
        if (val && eq) {
            /* Stage only -- never written to NVS here. Applied + connected
             * (and only then eligible for NVS) on the next bare AT$WIFI. */
            strncpy(s_stage_ssid, val, sizeof s_stage_ssid - 1);
            s_stage_ssid[sizeof s_stage_ssid - 1] = 0;
            s_have_stage_ssid = true;
            disk_logf("AT$SSID=: staged \"%s\" (apply via AT$WIFI)", val);
            r_ok();
        } else {
            char ssid[33] = {0};
            size_t n = sizeof ssid;
            nvs_handle_t h;
            if (nvs_open("slip-router", NVS_READONLY, &h) == ESP_OK) {
                nvs_get_str(h, "ssid", ssid, &n);
                nvs_close(h);
            }
            /* Show the staged value if one is pending, else what's in NVS. */
            cdc_print("\r\n"); cdc_print(s_have_stage_ssid ? s_stage_ssid : ssid); cdc_print("\r\n");
            r_ok();
        }
    } else if (!strcmp(key, "PASS")) {
        nvs_handle_t h;
        if (val && eq) {
            /* Stage only -- never written to NVS here. */
            strncpy(s_stage_pass, val, sizeof s_stage_pass - 1);
            s_stage_pass[sizeof s_stage_pass - 1] = 0;
            s_have_stage_pass = true;
            disk_logf("AT$PASS=: staged len=%zu (apply via AT$WIFI)", strlen(val));
            r_ok();
        } else {
            char pass[65] = {0};
            size_t n = sizeof pass;
            bool has = false;
            if (nvs_open("slip-router", NVS_READONLY, &h) == ESP_OK) {
                if (nvs_get_str(h, "pass", pass, &n) == ESP_OK && pass[0]) has = true;
                nvs_close(h);
            }
            cdc_print(has ? "\r\n(set)\r\n" : "\r\n(none)\r\n");
            r_ok();
        }
    } else if (!strcmp(key, "RSSI")) {
        wifi_ap_record_t ap = {0};
        char b[32];
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
            snprintf(b, sizeof b, "\r\n%d\r\n", (int)ap.rssi);
        else
            snprintf(b, sizeof b, "\r\nnoconn\r\n");
        cdc_print(b); r_ok();
    } else if (!strcmp(key, "STATS")) {
        if (val && eq) {
            /* AT$STATS=0 -- zero the SLIP counters. The harness
             * brackets each test with a clear-and-run-and-read so
             * before/after deltas isolate the test's own traffic. */
            if (!strcmp(val, "0")) {
                /* slip.c uses volatile uint32_ts; race-free single-
                 * store overwrite is fine. */
                slip_stats_clear();
                r_ok();
            } else {
                r_error();
            }
        } else {
            char line[160];
            snprintf(line, sizeof line,
                "\r\n"
                "slip.pkts_to_host    %u\r\n"
                "slip.pkts_from_host  %u\r\n"
                "slip.bytes_to_host   %u\r\n"
                "slip.bytes_from_host %u\r\n",
                (unsigned)slip_stat_pkts_to_host(),
                (unsigned)slip_stat_pkts_from_host(),
                (unsigned)slip_stat_bytes_to_host(),
                (unsigned)slip_stat_bytes_from_host());
            cdc_print(line);
            r_ok();
        }
    } else if (!strcmp(key, "MODE")) {
        if (val && eq) {
            LinkMode want = (LinkMode)-1;
            if      (!strcmp(val, "SLIP"))  want = MODE_SLIP;
            else if (!strcmp(val, "MODEM")) want = MODE_MODEM;
            if (want == (LinkMode)-1) { r_error(); return; }
            /* Reply BEFORE flipping mode, so the OK lands while the
             * AT parser still owns the CDC stream. Once SLIP is up
             * any further bytes from the host are interpreted as
             * SLIP frames. */
            r_ok();
            slip_set_mode(want);
        } else {
            cdc_print(slip_get_mode() == MODE_SLIP ? "\r\nSLIP\r\n" : "\r\nMODEM\r\n");
            r_ok();
        }
    } else if (!strcmp(key, "DNS") && val && eq) {
        cmd_dns(val);
    } else if (!strcmp(key, "PING") && val && eq) {
        cmd_ping(val);
    } else if (!strcmp(key, "SCAN")) {
        cmd_scan();
    } else if (!strcmp(key, "NETIF")) {
        cmd_netif();
    } else if (!strcmp(key, "RESET")) {
        cdc_print("\r\nRESETTING\r\n");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else if (!strcmp(key, "BOOT")) {
        /* AT$BOOT -- jump to ESP32-S3 ROM bootloader. Same fallback
         * the old build had: if AT$OTASTART can't run (firmware too
         * broken, locked, pre-OTA), we set the "force download mode"
         * RTC option and reset. Bootloader then enumerates as
         * cu.usbmodem123401 -- esptool can reflash via the same cable. */
        cdc_print("\r\nENTERING BOOTLOADER\r\n");
        vTaskDelay(pdMS_TO_TICKS(150));
        REG_WRITE(RTC_CNTL_OPTION1_REG, 0x1);   /* RTC_CNTL_FORCE_DOWNLOAD_BOOT */
        esp_restart();
    } else if (!strcmp(key, "OTASTART") && val && eq) {
        /* AT$OTASTART=<size> -- CDC-side OTA. Streams <size> bytes
         * of raw firmware.bin into the inactive OTA slot, then
         * commits + reboots. Recovery path when WiFi is unreachable.
         *
         *   host    -> AT$OTASTART=<size>\r
         *   dongle  -> \r\nOTA READY\r\n
         *   host    -> <size> bytes of fw.bin
         *   dongle  -> \r\nOTA OK\r\n  (then esp_restart)
         *           or \r\nOTA <code>\r\n + ERROR */
        long sz = strtol(val, NULL, 10);
        if (sz < 16384L || sz > 6L * 1024L * 1024L) {
            cdc_print("\r\nOTA BADSIZE\r\n"); r_error(); return;
        }
        const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
        if (!next) { cdc_print("\r\nOTA NOPART\r\n"); r_error(); return; }
        esp_ota_handle_t h = 0;
        /* OTA_WITH_SEQUENTIAL_WRITES: erase per-sector lazily inside
         * esp_ota_write. No upfront 15 s full-partition erase, no risk
         * of writing past an undersized erased range. See h_ota in
         * main.c for the full reasoning. */
        if (esp_ota_begin(next, OTA_WITH_SEQUENTIAL_WRITES, &h) != ESP_OK) {
            cdc_print("\r\nOTA BEGIN-FAIL\r\n"); r_error(); return;
        }
        cdc_print("\r\nOTA READY\r\n");
        long got = 0;
        int64_t last_rx_us = esp_timer_get_time();
        static uint8_t buf[1024];   /* SRAM trim 2026-06-02 (was 2048) */
        while (got < sz) {
            size_t want = (size_t)(sz - got);
            if (want > sizeof buf) want = sizeof buf;
            size_t n = 0;
            if (tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, buf, want, &n) == ESP_OK && n > 0) {
                if (esp_ota_write(h, buf, n) != ESP_OK) {
                    esp_ota_abort(h);
                    cdc_print("\r\nOTA WRITE-FAIL\r\n"); r_error(); return;
                }
                got += (long)n;
                last_rx_us = esp_timer_get_time();
            } else {
                if (esp_timer_get_time() - last_rx_us > 8 * 1000 * 1000LL) {
                    esp_ota_abort(h);
                    cdc_print("\r\nOTA TIMEOUT\r\n"); r_error(); return;
                }
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
        if (esp_ota_end(h) != ESP_OK) {
            cdc_print("\r\nOTA END-FAIL\r\n"); r_error(); return;
        }
        if (esp_ota_set_boot_partition(next) != ESP_OK) {
            cdc_print("\r\nOTA SETBOOT-FAIL\r\n"); r_error(); return;
        }
        cdc_print("\r\nOTA OK\r\n");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else if (!strcmp(key, "TYPE") && val && eq) {
        /* AT$TYPE=<string> -- send <string> via the USB HID keyboard.
         * See kbd.h for the token DSL (<ENTER>, <F1>, <CTRL+C>,
         * <DELAY=ms>, etc). */
        int n = kbd_type(val, (int)strlen(val));
        if (n < 0) { r_error(); return; }
        r_ok();
    } else if (!strcmp(key, "LECHO")) {
        /* AT$LECHO=AUTO|ON|OFF -- override the data-mode local-echo
         * policy. AUTO is the negotiated rule with the password-safety
         * latch; ON bypasses the latch; OFF disables echo regardless.
         * See TELNET-LOCAL-ECHO-2026-06-01.md. */
        if (val && eq) {
            if      (!strcasecmp(val, "AUTO")) s_lecho_mode = LECHO_AUTO;
            else if (!strcasecmp(val, "ON"))   s_lecho_mode = LECHO_ON;
            else if (!strcasecmp(val, "OFF"))  s_lecho_mode = LECHO_OFF;
            else { r_error(); return; }
            r_ok();
        } else {
            const char *m = (s_lecho_mode == LECHO_ON)  ? "ON"  :
                            (s_lecho_mode == LECHO_OFF) ? "OFF" : "AUTO";
            cdc_print("\r\nLECHO="); cdc_print(m); cdc_print("\r\n");
            r_ok();
        }
    } else if (!strcmp(key, "NAWS")) {
        /* AT$NAWS=cols,rows -- usbterm declares its terminal size for
         * telnet NAWS subnegotiation. Bare AT$NAWS queries. */
        if (val && eq) {
            char *comma = strchr(val, ',');
            if (!comma) { r_error(); return; }
            *comma = 0;
            long c = strtol(val, NULL, 10);
            long r = strtol(comma + 1, NULL, 10);
            if (c < 1 || c > 65535 || r < 1 || r > 65535) { r_error(); return; }
            s_term_cols = (uint16_t)c;
            s_term_rows = (uint16_t)r;
            /* Live update: if connected and the server has DO'd NAWS,
             * push a fresh subnegotiation so the prompt redraws. */
            if (s_online && bget(s_local_on, OPT_NAWS)) tn_send_naws();
            r_ok();
        } else {
            char line[32];
            snprintf(line, sizeof line, "\r\n%u,%u\r\n",
                     (unsigned)s_term_cols, (unsigned)s_term_rows);
            cdc_print(line);
            r_ok();
        }
    } else if (!strcmp(key, "TTYPE")) {
        /* AT$TTYPE=name -- usbterm declares the terminal type used in
         * telnet TTYPE subnegotiation (replaces the hard-coded "ANSI").
         * Bare AT$TTYPE queries. */
        if (val && eq) {
            size_t vn = strnlen(val, sizeof s_term_type - 1);
            if (vn == 0) { r_error(); return; }
            memcpy(s_term_type, val, vn);
            s_term_type[vn] = 0;
            r_ok();
        } else {
            cdc_print("\r\n"); cdc_print(s_term_type); cdc_print("\r\n");
            r_ok();
        }
    } else if (!strcmp(key, "HELP")) {
        cdc_print(
            "\r\n"
            "ATE0/1  echo off/on        ATV0/1  numeric/verbose\r\n"
            "ATQ0/1  result codes       ATI     identity\r\n"
            "ATZ     reset settings     ATD<host>[:port]  dial out\r\n"
            "ATO     return online      ATH     hang up\r\n"
            "+++     escape to cmd      (1 s guard, 3 +'s, 1 s guard)\r\n"
            "AT$WIFI=ssid,pw    set wifi creds + reconnect\r\n"
            "AT$WIFI?           show wifi status\r\n"
            "AT$DNS=host        DNS lookup\r\n"
            "AT$PING=host       TCP-handshake ping\r\n"
            "AT$SCAN            list visible networks\r\n"
            "AT$NETIF           dump netif state\r\n"
            "AT$TYPE=<str>      send keystrokes via HID keyboard (DSL)\r\n"
            "AT$LECHO=mode     data-mode local echo (AUTO/ON/OFF)\r\n"
            "AT$NAWS=cols,rows  declare terminal size for telnet NAWS\r\n"
            "AT$TTYPE=name      declare terminal type for telnet TTYPE\r\n"
            "AT$RESET           reboot the dongle\r\n"
            "AT$HELP            this help\r\n"
        );
        r_ok();
    } else {
        r_error();
    }
}

static void exec(char *line) {
    char *p = strip_at(line);
    if (!p) return;                 /* non-AT line: silent */
    if (!*p) { r_ok(); return; }    /* bare AT */

    /* Dial worker in flight: reject other commands rather than let
     * them race against cmd_dial_impl's s_sock / s_online / s_peer
     * writes. Matches today's effective behaviour where the TinyUSB
     * task is blocked during ATDT anyway. */
    if (s_at_busy) { r_error(); return; }

    switch (*p) {
        case 'E': s_echo    = (p[1] != '0'); r_ok(); return;
        case 'V': s_verbose = (p[1] != '0'); r_ok(); return;
        case 'Q': s_quiet   = (p[1] == '1'); r_ok(); return;
        case 'I': {
            char b[160];
            int64_t dur = s_tp_start ? (esp_timer_get_time() - s_tp_start) : 0;
            snprintf(b, sizeof b,
                     "\r\nDOSongle Modem (Phase 1c)\r\nheap free=%u min=%u largest=%u\r\n"
                     "tput rx=%u cdc=%u blkus=%llu durus=%lld\r\n",
                     (unsigned)esp_get_free_heap_size(),
                     (unsigned)esp_get_minimum_free_heap_size(),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                     (unsigned)s_tp_rx, (unsigned)s_tp_cdc,
                     (unsigned long long)s_tp_blk_us, (long long)dur);
            cdc_print(b);
            r_ok(); return;
        }
        case 'N': {
            /* AT N<n> / AT NET<n> -- telnet IAC + CR-to-CRLF processing.
             * Old build accepts both "ATN0" and "ATNET0"; HTTPGET.EXE
             * sends the longer form. Skip over any non-digit letters
             * after N until we find the digit. */
            const char *q = p + 1;
            while (*q && (*q < '0' || *q > '9')) ++q;
            s_telnet = (*q != '0');
            r_ok();
        } return;
        case 'Z': s_echo = true; s_verbose = true; s_quiet = false; s_telnet = true; r_ok(); return;
        case 'D': cmd_dial(p + 1); return;
        case 'H': {
            if (s_sock >= 0) { close(s_sock); s_sock = -1; }
            s_online = false;
            s_peer[0] = 0;
            r_ok();
        } return;
        case 'O': {
            /* ATO -- return to online mode after a +++ escape. The
             * socket is still open (the +++ guard only paused the
             * data pumps, it didn't hang up). Re-arm the persistent
             * pump tasks; they were parked when s_online went false. */
            if (s_sock >= 0) {
                s_online = true;
                if (s_data_task)     xTaskNotifyGive(s_data_task);
                if (s_cdc_pump_task) xTaskNotifyGive(s_cdc_pump_task);
                if (s_tcp_pump_task) xTaskNotifyGive(s_tcp_pump_task);
                r_connect();
            } else {
                r_error();
            }
        } return;
        case '$': handle_dollar(p + 1); return;
        case '&': {
            /* AT&V dump config / AT&W save E/V/N / AT&F factory reset. */
            char c = (char)toupper((unsigned char)p[1]);
            if (c == 'V') {
                char b[160];
                snprintf(b, sizeof b,
                         "\r\nATE%d  ATV%d  ATQ%d  ATN%d\r\n"
                         "mode:  %s\r\n",
                         s_echo?1:0, s_verbose?1:0, s_quiet?1:0, s_telnet?1:0,
                         slip_get_mode() == MODE_SLIP ? "SLIP" : "MODEM");
                cdc_print(b);
                r_ok();
            } else if (c == 'W') {
                modem_save_evn();
                r_ok();
            } else if (c == 'F') {
                s_echo = true; s_verbose = true; s_quiet = false; s_telnet = true;
                modem_save_evn();
                r_ok();
            } else {
                r_error();
            }
        } return;
        default:
            /* Real modems are lenient: unknown single-letter commands
             * (AT&K0, ATS0=0, etc. that scripts commonly issue) return
             * OK so the script doesn't abort. We do the same. */
            r_ok();
            return;
    }
}

/* ---- CDC RX callback (TinyUSB task context) ---- */

static void on_cdc_rx(int itf, cdcacm_event_t *event) {
    (void)event;
    /* Drain in MTU-sized chunks. SLIP MTU is 1500; CDC RX FIFO is
     * sized to hold a full SLIP frame, so this loop typically does
     * one big read per callback. */
    static uint8_t buf[2048];
    size_t got = 0;
    if (tinyusb_cdcacm_read(itf, buf, sizeof buf, &got) != ESP_OK) return;
    if (got == 0) return;

    /* CPR probe in flight (just-dialed): consume the reply before any
     * other path sees it. Stale probes time out via the deadline so a
     * non-responding peer doesn't permanently eat input. */
    if (s_cpr_pending) {
        if (esp_timer_get_time() > s_cpr_deadline_us) {
            s_cpr_pending = false;
            s_cpr_len = 0;
            /* fall through; the bytes we just received belong to the
             * normal path (likely server greeting / AT input). */
        } else {
            size_t consumed = cpr_feed(buf, got);
            if (consumed >= got) return;
            /* Parser bailed out mid-buffer (overflow); reprocess the
             * remaining tail through the normal paths. */
            memmove(buf, buf + consumed, got - consumed);
            got -= consumed;
        }
    }

    /* SLIP mode: bytes are SLIP-framed IP packets. Feed them to the
     * de-framer; nothing else touches them. */
    if (slip_get_mode() == MODE_SLIP) {
        slip_feed(buf, got);
        return;
    }

    if (s_online) {
        /* Online: echo locally if the remote isn't echoing (gated by
         * AT$LECHO), then pipe CDC bytes to TCP with +++ escape +
         * telnet IAC/CRLF handling. on_cdc_rx and cdc_write both run
         * on CPU1, so the echo path stays intra-core. */
        if (local_echo_active()) do_local_echo(buf, got);
        online_push_bytes(buf, got);
        return;
    }

    /* Command mode: line-buffered AT parser. */
    for (size_t i = 0; i < got; ++i) {
        uint8_t b = buf[i];
        if (b == '\r' || b == '\n') {
            if (s_echo) cdc_print("\r\n");
            if (s_cmd_overflow) {
                r_error();
                s_cmd_overflow = false;
            } else {
                s_cmd[s_cmd_len] = '\0';
                exec(s_cmd);
            }
            s_cmd_len = 0;
            continue;
        }
        if (b == 0x08 || b == 0x7F) {
            if (s_cmd_len > 0) {
                s_cmd_len--;
                if (s_echo) cdc_print("\b \b");
            }
            continue;
        }
        if (b < 0x20 || b > 0x7E) continue;
        if (s_cmd_len + 1 < sizeof s_cmd) {
            s_cmd[s_cmd_len++] = (char)b;
            if (s_echo) cdc_byte(b);
        } else {
            s_cmd_overflow = true;
        }
    }
}

/* ---- public entry ---- */

bool modem_online_peer(const char **peer_out) {
    if (peer_out) *peer_out = s_peer;
    return s_online;
}

bool modem_get_tput(uint32_t *rx, uint32_t *cdc, uint64_t *blk_us) {
    if (rx)     *rx     = s_tp_rx;
    if (cdc)    *cdc    = s_tp_cdc;
    if (blk_us) *blk_us = s_tp_blk_us;
    return s_online;
}

esp_err_t modem_init(void) {
    modem_load_evn();      /* restore E/V/N from NVS (AT&W saved) */
    esp_err_t e = tinyusb_cdcacm_register_callback(
        TINYUSB_CDC_ACM_0, CDC_EVENT_RX, on_cdc_rx);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "cdc rx cb register: %s", esp_err_to_name(e));
        return e;
    }

    /* Pre-allocate the producer/consumer pipeline at init so first-dial
     * heap pressure can't fail one of them. The dial path's
     * setsockopt(SO_RCVBUF, 32 KB) grows lwIP's TCP recv buffer and
     * by the time we'd allocate s_to_tcp inline the free heap was
     * ~12 KB -- not enough for 8 KB + overhead in a contiguous block.
     * Allocating here, when boot free heap is highest, guarantees the
     * rings exist by the time ATDT fires. The rings are session-
     * resident (reset on each dial), never freed. */
    if (!s_to_cdc) s_to_cdc = xStreamBufferCreate(DATA_STREAM_BYTES, 1);
    if (!s_to_tcp) s_to_tcp = xStreamBufferCreate(DATA_STREAM_BYTES, 1);
    if (!s_to_cdc || !s_to_tcp) {
        ESP_LOGE(TAG, "modem stream alloc fail to_cdc=%p to_tcp=%p free=%u",
                 (void *)s_to_cdc, (void *)s_to_tcp,
                 (unsigned)esp_get_free_heap_size());
        return ESP_ERR_NO_MEM;
    }

    /* Pre-create the three data-pump tasks ONCE, here at boot, when
     * free heap is ~44 KB. They park on a task notification and run
     * their inner loop only while a dial is active. This keeps the
     * ~10 KB of task-stack allocation entirely off the dial path: at
     * WB_SLOTS=16 the dial-time free heap is only ~25 KB, and a telnet
     * server flooding its banner on connect could leave too little to
     * create three stacks (the freechess NO CARRIER bug). cmd_dial now
     * just xTaskNotifyGive()s these to start a session. */
    BaseType_t c1 = xTaskCreatePinnedToCore(modem_cdc_pump_task, "modem_cdc",
                                            3072, NULL, 16, &s_cdc_pump_task, 1);
    BaseType_t c2 = xTaskCreatePinnedToCore(modem_tcp_pump_task, "modem_tcp",
                                            3072, NULL, 16, &s_tcp_pump_task, 0);
    BaseType_t c3 = xTaskCreatePinnedToCore(modem_data_task, "modem_data",
                                            4096, NULL, 16, &s_data_task, 0);
    if (c1 != pdPASS || c2 != pdPASS || c3 != pdPASS) {
        ESP_LOGE(TAG, "modem pump task create fail c1=%d c2=%d c3=%d",
                 (int)c1, (int)c2, (int)c3);
        return ESP_ERR_NO_MEM;
    }

    disk_logf("modem: AT engine ready on CDC0 (E%d V%d N%d) streams=%uB pumps=up",
              s_echo?1:0, s_verbose?1:0, s_telnet?1:0,
              (unsigned)DATA_STREAM_BYTES);
    return ESP_OK;
}

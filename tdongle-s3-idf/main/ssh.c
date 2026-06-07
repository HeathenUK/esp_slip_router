/* ssh.c -- real SSH transport for the relay (Phase 3).
 *
 * Owns the libssh2 session + channel; modem.c owns the socket fd (s_sock) and
 * the relay pump tasks, and drives this through the xport vtable
 * (xport_read/xport_write/xport_close -> ssh_read/ssh_write/ssh_close). Keeping
 * libssh2 here means the vendored headers' `#define ESP32` + heavy includes
 * never leak into modem.c.
 *
 * Lifecycle: ssh_connect() does the BLOCKING handshake + password auth + a
 * PTY/shell channel (the kex bignum math takes seconds -- caller runs it on a
 * worker task off the TinyUSB CPU), then switches the session to NON-BLOCKING
 * so the relay's recv pump polls without wedging (the Phase-0 spike hung
 * precisely because it read a BLOCKING channel with no idle exit). ssh_read/
 * ssh_write map libssh2's EAGAIN to errno=EAGAIN so they satisfy the same
 * contract as recv()/send() in the pump.
 *
 * Credits: libssh2 (BSD-3-Clause, Daniel Stenberg & contributors). The
 * session/channel/PTY/shell flow follows Zimodem's WiFiSSHClient (Bo Zimmerman,
 * Apache-2.0). See components/libssh2/CREDITS.md. The password is never logged.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "libssh2_idf.h"   /* defines ESP32, then includes libssh2.h */
#include "disk.h"

static LIBSSH2_SESSION *s_session = NULL;
static LIBSSH2_CHANNEL *s_channel = NULL;
static int s_rx_seen = 0;   /* logged the first relayed channel byte yet? */

/* libssh2 is NOT reentrant on a single session: the relay reads (recv pump task)
 * and writes (keystroke pump task) the SAME s_session/s_channel concurrently, so
 * a keystroke arriving mid-output used to interleave two libssh2 calls -- which
 * corrupted the cipher state (-> libssh2_channel_read returns -12 DECRYPT) and
 * the session's internal buffers (-> PANIC reboot). This mutex serializes every
 * libssh2 session/channel call so read and write can never overlap. Created once
 * and never destroyed (lifetime static), so a task blocked on it during teardown
 * can't fault on a deleted handle. */
static SemaphoreHandle_t s_ssh_lock = NULL;
static void ssh_lock_init(void) { if (!s_ssh_lock) s_ssh_lock = xSemaphoreCreateMutex(); }
#define SSH_LOCK()   do { if (s_ssh_lock) xSemaphoreTake(s_ssh_lock, portMAX_DELAY); } while (0)
#define SSH_UNLOCK() do { if (s_ssh_lock) xSemaphoreGive(s_ssh_lock); } while (0)

/* Open an SSH session + interactive shell to user@host:port. Returns the
 * connected socket fd (caller adopts it as the relay's s_sock) on success, or
 * -1 on any failure (all libssh2 state freed, socket closed). The session is
 * left NON-BLOCKING. Never logs `pass`. */
int ssh_connect(const char *user, const char *pass, const char *host, uint16_t port,
                const char *term, int cols, int rows)
{
    struct addrinfo hints, *res = NULL;
    char ps[8];
    int sock = -1;

    s_session = NULL;
    s_channel = NULL;
    s_rx_seen = 0;
    ssh_lock_init();   /* serialize read/write/close across the relay tasks */

    if (libssh2_init(0) != 0) { disk_logf("ssh: libssh2_init fail"); return -1; }

    /* Allocate the session FIRST, while the heap is freshest/most contiguous:
     * the LIBSSH2_SESSION struct needs ONE contiguous ~11 KB block, and doing
     * DNS/socket/connect first fragments the largest free block below that
     * (the Phase-0 finding). */
    s_session = libssh2_session_init();
    if (!s_session) {
        disk_logf("ssh: session_init NULL lfb=%u",
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        libssh2_exit();
        return -1;
    }
    libssh2_session_set_blocking(s_session, 1);   /* blocking for setup */

    /* Per-step timing to locate the ~27 s handshake cost. */
    int64_t t_step = esp_timer_get_time();
#define SSH_STEP(name) do { int64_t _n = esp_timer_get_time(); \
    disk_logf("ssh: " name " +%dms", (int)((_n - t_step) / 1000)); t_step = _n; } while (0)

    snprintf(ps, sizeof ps, "%u", (unsigned)port);
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host, ps, &hints, &res) != 0 || !res) { disk_logf("ssh: DNS fail"); goto fail; }
    SSH_STEP("dns");
    sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) { freeaddrinfo(res); disk_logf("ssh: socket fail"); goto fail; }
    {
        struct timeval tv = { .tv_sec = 15, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    }
    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        freeaddrinfo(res); disk_logf("ssh: connect fail"); goto fail;
    }
    freeaddrinfo(res); res = NULL;
    SSH_STEP("tcp");

    if (libssh2_session_handshake(s_session, sock)) { disk_logf("ssh: handshake FAIL"); goto fail; }
    SSH_STEP("handshake");
    if (libssh2_userauth_password(s_session, user, pass)) { disk_logf("ssh: AUTH FAILED for %s", user); goto fail; }
    SSH_STEP("auth");

    /* Open with a SMALL channel window (4 KB) + packet size (2 KB) instead of
     * libssh2's defaults (2 MB window / 32 KB packet). On this no-PSRAM device
     * the defaults let the server's first burst make libssh2 allocate big
     * receive/queue buffers -> free heap craters to ~1.9 KB and the relay's
     * anti-wedge guard aborts the session (NO CARRIER right after connect). A
     * small window bounds libssh2's in-flight buffering AND back-pressures the
     * server to our (CDC-paced) consumption rate -- the heap-safe trade for an
     * interactive relay. open_session() is just open_ex(...,"session",...defaults). */
    s_channel = libssh2_channel_open_ex(s_session, "session", sizeof("session") - 1,
                                        4096 /*window*/, 2048 /*packet*/, NULL, 0);
    if (!s_channel) { disk_logf("ssh: channel-open fail"); goto fail; }
    SSH_STEP("channel");
    /* Advertise the caller's TERM + real cell size so the remote shell/apps
     * pick the right capability set (colour, alt-screen) and render to the
     * correct rows/cols. Default is a capable xterm-class TERM (the relay is
     * byte-transparent and usbterm is xterm-ish); other/limited DOS terminals
     * can downgrade via AT$TERM=. request_pty_ex carries the dimensions
     * (plain request_pty hardcodes 80x24). */
    if (!term || !*term) term = "vt100";
    if (cols <= 0) cols = 80;
    if (rows <= 0) rows = 25;
    libssh2_channel_request_pty_ex(s_channel, term, (unsigned int)strlen(term),
                                   NULL, 0, cols, rows, 0, 0);
    SSH_STEP("pty");
    if (libssh2_channel_shell(s_channel)) { disk_logf("ssh: shell fail"); goto fail; }
    SSH_STEP("shell");
#undef SSH_STEP

    /* Relay phase: keep the session BLOCKING but bound each call with a short
     * libssh2 timeout, so libssh2_channel_read returns LIBSSH2_ERROR_TIMEOUT on
     * an idle channel (-> the recv pump polls ~10x/s) instead of wedging. A
     * NON-blocking session over our blocking-with-SO_RCVTIMEO socket stalled the
     * transport after the first packet (only ~61 B ever relayed); blocking + a
     * short session timeout is the robust poll model for a single-channel relay. */
    /* 30 ms (was 100): an idle read holds the serialization lock for the whole
     * timeout, so a shorter timeout caps how long a keystroke write waits behind
     * an idle read (~30 ms vs ~100 ms) at the cost of a slightly faster poll. */
    libssh2_session_set_timeout(s_session, 30);   /* ms */
    disk_logf("ssh: up %s@%s:%u free=%u", user, host, (unsigned)port,
              (unsigned)esp_get_free_heap_size());
    return sock;

fail:
    if (s_channel) { libssh2_channel_free(s_channel); s_channel = NULL; }
    if (s_session) { libssh2_session_free(s_session); s_session = NULL; }
    libssh2_exit();
    if (sock >= 0) close(sock);
    return -1;
}

/* recv()-contract read: >0 bytes, 0 = EOF/closed, -1 with errno=EAGAIN when
 * no data is available this poll (non-blocking channel). */
int ssh_read(void *buf, size_t len)
{
    ssize_t r;
    SSH_LOCK();
    r = s_channel ? libssh2_channel_read(s_channel, (char *)buf, len) : 0;
    SSH_UNLOCK();
    if (r > 0) {
        if (!s_rx_seen) { s_rx_seen = 1; disk_logf("ssh: first channel data (%d B)", (int)r); }
        return (int)r;
    }
    /* Blocking session w/ libssh2 timeout: TIMEOUT == "idle this poll". */
    if (r == LIBSSH2_ERROR_EAGAIN || r == LIBSSH2_ERROR_TIMEOUT) { errno = EAGAIN; return -1; }
    if (r < 0) disk_logf("ssh: read err %d", (int)r);
    return 0;   /* 0 (EOF) or other negative error -> relay tears down */
}

/* send()-contract write: >0 bytes written, -1 with errno=EAGAIN when the
 * channel window is momentarily full (tcp_send_all retries), 0 on error. */
int ssh_write(const void *buf, size_t len)
{
    ssize_t w;
    SSH_LOCK();
    w = s_channel ? libssh2_channel_write(s_channel, (const char *)buf, len) : 0;
    SSH_UNLOCK();
    if (w > 0) return (int)w;
    if (w == LIBSSH2_ERROR_EAGAIN || w == LIBSSH2_ERROR_TIMEOUT) { errno = EAGAIN; return -1; }
    return 0;
}

/* Free the channel + session and shut libssh2 down. Does NOT close the socket
 * fd -- modem.c's xport_close owns s_sock. Idempotent. */
void ssh_close(void)
{
    /* Take the lock so we never free the session/channel while the recv or
     * keystroke task is mid-call inside libssh2. After this, s_channel is NULL,
     * so any later ssh_read/ssh_write returns EOF instead of using freed state. */
    SSH_LOCK();
    if (s_channel) { libssh2_channel_free(s_channel); s_channel = NULL; }
    if (s_session) { libssh2_session_free(s_session); s_session = NULL; }
    libssh2_exit();
    SSH_UNLOCK();
}

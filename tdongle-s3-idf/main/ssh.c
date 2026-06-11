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
#include "libssh2_sftp.h"
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

/**
 * @brief Open an SSH session + interactive shell to user@host:port.
 * @param user Login username.
 * @param pass Password (never logged); NULL/empty for keyboard-interactive.
 * @param host Target hostname or IP.
 * @param port TCP port (typically 22).
 * @param term TERM string advertised to the remote PTY.
 * @param cols PTY width in cells.
 * @param rows PTY height in cells.
 * @return Connected socket fd (caller adopts it as the relay's s_sock) on
 *         success, or -1 on any failure (all libssh2 state freed, socket
 *         closed). On success the session is left NON-BLOCKING.
 */
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
        /* Deliberately NO SO_RCVBUF here: on this lwIP build it is a no-op for TCP
         * (recv_bufsize is consulted only for UDP/RAW; the TCP receive window is
         * pinned to the global TCP_WND = 16 KB and there is no per-socket window
         * knob). A remote flood (e.g. a 256-colour dump) is bounded instead by two
         * things working together: (a) the small libssh2 channel window (4 KB,
         * opened below), and (b) READ-PACING in the relay loop (modem.c) -- it stops
         * draining the channel while the CDC downstream is backed up, so libssh2
         * stops calling recv (stops re-opening the TCP window) and the remote stalls
         * at the channel window instead of piling ~16 KB of RX pbufs into heap. */
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

/**
 * @brief recv()-contract read from the SSH channel.
 * @param buf Destination buffer.
 * @param len Maximum bytes to read.
 * @return >0 bytes read, 0 on EOF/closed, or -1 with errno=EAGAIN when no data
 *         is available this poll.
 */
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

/**
 * @brief send()-contract write to the SSH channel.
 * @param buf Source buffer.
 * @param len Bytes to write.
 * @return >0 bytes written, -1 with errno=EAGAIN when the channel window is
 *         momentarily full (tcp_send_all retries), or 0 on error.
 */
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

/**
 * @brief Free the channel + session and shut libssh2 down.
 *
 * Does NOT close the socket fd -- modem.c's xport_close owns s_sock.
 * Idempotent.
 */
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

/* ---- SFTP client (AT$SFTP) ---------------------------------------------------
 * Its own session/socket, separate from the shell-relay s_session. SFTP has no
 * server-side "current directory", so we track the cwd client-side (s_sftp_cwd)
 * and build absolute paths for every op (the server resolves '..' via realpath).
 * The download body is delivered through a sink callback (modem.c frames it as
 * OSC 5113 so ftpget captures it). Plaintext APIs; types stay in ssh.c. */
static LIBSSH2_SESSION *s_sftp_sess = NULL;
static LIBSSH2_SFTP    *s_sftp      = NULL;
static int              s_sftp_sock = -1;
static char             s_sftp_cwd[256] = "/";

typedef void (*sftp_sink_fn)(const unsigned char *data, size_t len);

/** @brief Build an absolute server path from @p p relative to the cwd. Manual
 *  bounded append (not a multi-%s snprintf, which trips -Werror=format-truncation);
 *  also avoids a double slash when cwd is "/". */
static void sftp_abspath(const char *p, char *out, size_t n)
{
    size_t len;
    if (!n) return;
    if (p && p[0] == '/') { snprintf(out, n, "%s", p); return; }   /* already absolute */
    snprintf(out, n, "%s", s_sftp_cwd);                            /* base = cwd */
    if (!p || !*p) return;
    len = strlen(out);
    if (!(len == 1 && out[0] == '/') && len + 1 < n) out[len++] = '/';
    while (*p && len + 1 < n) out[len++] = *p++;
    out[len] = '\0';
}

void sftp_quit(void)
{
    /* Stay BLOCKING here: libssh2_sftp_shutdown -> _libssh2_channel_free and
     * session_disconnect must run to completion to actually FREE the channel +
     * SFTP structs. (A non-blocking teardown returns EAGAIN and leaks them, which
     * degrades the heap a couple KB per session.) A dead socket bounds each send
     * at SO_SNDTIMEO; recovery during any slow teardown is covered by the
     * AT$RESET/AT$OTASTART busy-guard bypass, so we don't risk a leak to save it. */
    if (s_sftp)      { libssh2_sftp_shutdown(s_sftp); s_sftp = NULL; }
    if (s_sftp_sess) { libssh2_session_disconnect(s_sftp_sess, "bye");
                       libssh2_session_free(s_sftp_sess); s_sftp_sess = NULL; }
    if (s_sftp_sock >= 0) { close(s_sftp_sock); s_sftp_sock = -1; }
    libssh2_exit();
}

/**
 * @brief Connect + auth + open the SFTP subsystem (no shell). cwd <- realpath(".").
 * @return 0 on success, -1 on any failure (all state freed). Never logs the password.
 */
int sftp_open(const char *user, const char *pass, const char *host, uint16_t port)
{
    struct addrinfo hints, *res = NULL;
    char ps[8];
    int  sock = -1;

    s_sftp_sess = NULL; s_sftp = NULL; s_sftp_sock = -1;
    strcpy(s_sftp_cwd, "/");

    if (libssh2_init(0) != 0) { disk_logf("sftp: init fail"); return -1; }
    s_sftp_sess = libssh2_session_init();
    if (!s_sftp_sess) { disk_logf("sftp: session_init NULL lfb=%u",
                        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
                        libssh2_exit(); return -1; }
    libssh2_session_set_blocking(s_sftp_sess, 1);

    snprintf(ps, sizeof ps, "%u", (unsigned)port);
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host, ps, &hints, &res) != 0 || !res) { disk_logf("sftp: DNS fail"); goto fail; }
    sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) { freeaddrinfo(res); disk_logf("sftp: socket fail"); goto fail; }
    { struct timeval tv = { .tv_sec = 15, .tv_usec = 0 };
      setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
      setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv); }
    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) { freeaddrinfo(res); disk_logf("sftp: connect fail"); goto fail; }
    freeaddrinfo(res); res = NULL;
    { int yes = 1; setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof yes); }
    s_sftp_sock = sock;

    /* Bound EVERY blocking libssh2 op (read/readdir/teardown). Blocking mode uses
     * select(), which ignores SO_SNDTIMEO -- without this, an op on a dead socket
     * (WiFi dropped mid-session) blocks the worker forever, so s_at_busy never
     * clears and the device looks wedged. 10 s is generous for a live link. */
    libssh2_session_set_timeout(s_sftp_sess, 10000);

    if (libssh2_session_handshake(s_sftp_sess, sock)) { disk_logf("sftp: handshake FAIL"); goto fail; }
    if (libssh2_userauth_password(s_sftp_sess, user, pass)) { disk_logf("sftp: AUTH FAILED %s", user); goto fail; }
    s_sftp = libssh2_sftp_init(s_sftp_sess);
    if (!s_sftp) { disk_logf("sftp: sftp_init FAIL"); goto fail; }

    { const char *c = libssh2_session_methods(s_sftp_sess, LIBSSH2_METHOD_CRYPT_SC);
      const char *m = libssh2_session_methods(s_sftp_sess, LIBSSH2_METHOD_MAC_SC);
      disk_logf("sftp: negotiated crypt=%s mac=%s", c ? c : "?", m ? m : "?"); }

    /* Land in the login dir: realpath(".") -> absolute home. */
    { char home[256];
      int n = libssh2_sftp_realpath(s_sftp, ".", home, sizeof home - 1);
      if (n > 0) { home[n] = 0; strncpy(s_sftp_cwd, home, sizeof s_sftp_cwd - 1);
                   s_sftp_cwd[sizeof s_sftp_cwd - 1] = 0; } }
    disk_logf("sftp: up %s@%s:%u cwd=%s min_free=%u", user, host, (unsigned)port,
              s_sftp_cwd, (unsigned)esp_get_minimum_free_heap_size());
    return 0;

fail:
    sftp_quit();
    return -1;
}

int sftp_pwd(char *out, size_t n) { snprintf(out, n, "%s", s_sftp_cwd); return 0; }

/**
 * @brief Has the SFTP session hit a non-recoverable transport error?
 * @return 1 if the last libssh2 error was fatal (cipher desync / socket loss) so
 *         the REPL should tear down now rather than strand AT mode + httpd; 0 if
 *         the session is still usable (a healthy session reports ERROR_NONE, and
 *         an ordinary op failure like "no such file" is non-fatal). Sticky by
 *         design -- these codes are never set spuriously and never recover.
 */
int sftp_fatal(void)
{
    if (!s_sftp_sess) return 1;
    switch (libssh2_session_last_errno(s_sftp_sess)) {
        case LIBSSH2_ERROR_DECRYPT:
        case LIBSSH2_ERROR_SOCKET_SEND:
        case LIBSSH2_ERROR_SOCKET_RECV:
        case LIBSSH2_ERROR_SOCKET_DISCONNECT:
        case LIBSSH2_ERROR_ALLOC:        /* readdir of a big dir ran the heap dry */
            return 1;
        default:
            return 0;
    }
}

int sftp_cd(const char *dir)
{
    char want[256], canon[256];
    LIBSSH2_SFTP_HANDLE *d;
    int rc;
    if (!s_sftp) return -1;
    sftp_abspath(dir, want, sizeof want);
    rc = libssh2_sftp_realpath(s_sftp, want, canon, sizeof canon - 1);
    if (rc > 0) canon[rc] = 0; else { strncpy(canon, want, sizeof canon - 1); canon[sizeof canon - 1] = 0; }
    d = libssh2_sftp_opendir(s_sftp, canon);   /* verify it's a readable dir */
    if (!d) return -1;
    libssh2_sftp_closedir(d);
    strncpy(s_sftp_cwd, canon, sizeof s_sftp_cwd - 1); s_sftp_cwd[sizeof s_sftp_cwd - 1] = 0;
    return 0;
}

/** @brief Per-entry callback for the streaming readdir: format one "type size
 *  name" line and hand it to the sink (passed as @p ctx). */
static void sftp_ls_emit(void *ctx, const char *name, LIBSSH2_SFTP_ATTRIBUTES *at)
{
    sftp_sink_fn sink = (sftp_sink_fn)ctx;
    char line[320];
    char type = (at->flags & LIBSSH2_SFTP_ATTR_PERMISSIONS) &&
                LIBSSH2_SFTP_S_ISDIR(at->permissions) ? 'd' : '-';
    unsigned long sz = (at->flags & LIBSSH2_SFTP_ATTR_SIZE) ? (unsigned long)at->filesize : 0UL;
    int len = snprintf(line, sizeof line, "%c %10lu  %s\r\n", type, sz, name);
    if (sink) sink((const unsigned char *)line, (size_t)len);
}

int sftp_ls(const char *arg, sftp_sink_fn sink)
{
    char path[256];
    LIBSSH2_SFTP_HANDLE *d;
    int rc;
    if (!s_sftp) return -1;
    sftp_abspath(arg, path, sizeof path);
    d = libssh2_sftp_opendir(s_sftp, path);
    if (!d) {
        char *emsg = NULL;
        int serr = libssh2_session_last_error(s_sftp_sess, &emsg, NULL, 0);
        disk_logf("sftp: opendir(%s) FAIL libssh2=%d sftp=%lu msg=%s", path,
                  serr, (unsigned long)libssh2_sftp_last_error(s_sftp),
                  emsg ? emsg : "?");
        return -1;
    }
    /* Streaming readdir: O(one entry) memory, so the listing size is no longer
     * bounded by the heap -- lists directories of any size (see sftp.c). */
    rc = libssh2_sftp_readdir_stream(d, sftp_ls_emit, (void *)sink);
    libssh2_sftp_closedir(d);
    if (rc < 0) {
        char *emsg = NULL;
        int serr = libssh2_session_last_error(s_sftp_sess, &emsg, NULL, 0);
        disk_logf("sftp: readdir_stream FAIL rc=%d libssh2=%d free=%u msg=%s", rc,
                  serr, (unsigned)esp_get_free_heap_size(), emsg ? emsg : "?");
        return -1;
    }
    return 0;
}

long sftp_size(const char *file)
{
    char path[256];
    LIBSSH2_SFTP_ATTRIBUTES at;
    if (!s_sftp) return -1;
    sftp_abspath(file, path, sizeof path);
    if (libssh2_sftp_stat(s_sftp, path, &at) != 0) return -1;
    return (at.flags & LIBSSH2_SFTP_ATTR_SIZE) ? (long)at.filesize : -1;
}

int sftp_get(const char *file, sftp_sink_fn sink)
{
    char path[256], b[1024];
    LIBSSH2_SFTP_HANDLE *fh;
    ssize_t n;
    if (!s_sftp) return -1;
    sftp_abspath(file, path, sizeof path);
    fh = libssh2_sftp_open(s_sftp, path, LIBSSH2_FXF_READ, 0);
    if (!fh) return -1;
    while ((n = libssh2_sftp_read(fh, b, sizeof b)) > 0)
        if (sink) sink((const unsigned char *)b, (size_t)n);
    libssh2_sftp_close(fh);
    return (n < 0) ? -1 : 0;
}

int sftp_mkdir(const char *dir)
{
    char p[256];
    if (!s_sftp) return -1;
    sftp_abspath(dir, p, sizeof p);
    return libssh2_sftp_mkdir(s_sftp, p, 0755) == 0 ? 0 : -1;
}
int sftp_rmdir(const char *dir)
{
    char p[256];
    if (!s_sftp) return -1;
    sftp_abspath(dir, p, sizeof p);
    return libssh2_sftp_rmdir(s_sftp, p) == 0 ? 0 : -1;
}
int sftp_del(const char *file)
{
    char p[256];
    if (!s_sftp) return -1;
    sftp_abspath(file, p, sizeof p);
    return libssh2_sftp_unlink(s_sftp, p) == 0 ? 0 : -1;
}
int sftp_rename(const char *oldn, const char *newn)
{
    char po[256], pn[256];
    if (!s_sftp) return -1;
    sftp_abspath(oldn, po, sizeof po);
    sftp_abspath(newn, pn, sizeof pn);
    return libssh2_sftp_rename(s_sftp, po, pn) == 0 ? 0 : -1;
}

/* ---- upload (SFTP write) ---- */
static LIBSSH2_SFTP_HANDLE *s_sftp_wh = NULL;

int sftp_put_open(const char *remote)
{
    char p[256];
    if (!s_sftp) return -1;
    sftp_abspath(remote, p, sizeof p);
    s_sftp_wh = libssh2_sftp_open(s_sftp, p,
                    LIBSSH2_FXF_WRITE | LIBSSH2_FXF_CREAT | LIBSSH2_FXF_TRUNC, 0644);
    return s_sftp_wh ? 0 : -1;
}

int sftp_put_write(const void *buf, size_t n)
{
    const char *p = buf;
    size_t off = 0;
    if (!s_sftp_wh) return -1;
    while (off < n) {
        ssize_t w = libssh2_sftp_write(s_sftp_wh, p + off, n - off);
        if (w > 0) off += (size_t)w;
        else if (w == LIBSSH2_ERROR_EAGAIN) continue;   /* blocking session: rare */
        else return -1;
    }
    return 0;
}

int sftp_put_close(void)
{
    int rc = 0;
    if (s_sftp_wh) { rc = libssh2_sftp_close(s_sftp_wh); s_sftp_wh = NULL; }
    return rc == 0 ? 0 : -1;
}

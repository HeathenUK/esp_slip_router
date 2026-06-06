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
#include "libssh2_idf.h"   /* defines ESP32, then includes libssh2.h */
#include "disk.h"

static LIBSSH2_SESSION *s_session = NULL;
static LIBSSH2_CHANNEL *s_channel = NULL;

/* Open an SSH session + interactive shell to user@host:port. Returns the
 * connected socket fd (caller adopts it as the relay's s_sock) on success, or
 * -1 on any failure (all libssh2 state freed, socket closed). The session is
 * left NON-BLOCKING. Never logs `pass`. */
int ssh_connect(const char *user, const char *pass, const char *host, uint16_t port)
{
    struct addrinfo hints, *res = NULL;
    char ps[8];
    int sock = -1;

    s_session = NULL;
    s_channel = NULL;

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

    snprintf(ps, sizeof ps, "%u", (unsigned)port);
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host, ps, &hints, &res) != 0 || !res) { disk_logf("ssh: DNS fail"); goto fail; }
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

    if (libssh2_session_handshake(s_session, sock)) { disk_logf("ssh: handshake FAIL"); goto fail; }
    if (libssh2_userauth_password(s_session, user, pass)) { disk_logf("ssh: AUTH FAILED for %s", user); goto fail; }

    s_channel = libssh2_channel_open_session(s_session);
    if (!s_channel) { disk_logf("ssh: channel-open fail"); goto fail; }
    libssh2_channel_request_pty(s_channel, "vt100");
    if (libssh2_channel_shell(s_channel)) { disk_logf("ssh: shell fail"); goto fail; }

    libssh2_session_set_blocking(s_session, 0);   /* non-blocking for the relay */
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
    ssize_t r = libssh2_channel_read(s_channel, (char *)buf, len);
    if (r > 0) return (int)r;
    if (r == LIBSSH2_ERROR_EAGAIN) { errno = EAGAIN; return -1; }
    return 0;   /* 0 (EOF) or other negative error -> relay tears down */
}

/* send()-contract write: >0 bytes written, -1 with errno=EAGAIN when the
 * channel window is momentarily full (tcp_send_all retries), 0 on error. */
int ssh_write(const void *buf, size_t len)
{
    ssize_t w = libssh2_channel_write(s_channel, (const char *)buf, len);
    if (w > 0) return (int)w;
    if (w == LIBSSH2_ERROR_EAGAIN) { errno = EAGAIN; return -1; }
    return 0;
}

/* Free the channel + session and shut libssh2 down. Does NOT close the socket
 * fd -- modem.c's xport_close owns s_sock. Idempotent. */
void ssh_close(void)
{
    if (s_channel) { libssh2_channel_free(s_channel); s_channel = NULL; }
    if (s_session) { libssh2_session_free(s_session); s_session = NULL; }
    libssh2_exit();
}

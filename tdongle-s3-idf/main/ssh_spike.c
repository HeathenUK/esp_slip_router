/* SSH feasibility spike (Phase 0) -- THROWAWAY measurement code.
 *
 * Opens ONE libssh2 session to a test host and logs the peak (min-free) internal
 * heap through handshake + auth + a data burst, with the secure-session quiesce
 * active. Purpose: GATE whether an SSH session fits the no-PSRAM internal-heap
 * budget BEFORE investing in the real integration. Delete once the gate is decided.
 *
 * Credits: libssh2 (BSD-3-Clause, Daniel Stenberg & contributors). The
 * session/channel/PTY/shell flow follows Zimodem's WiFiSSHClient
 * (Bo Zimmerman, Apache-2.0). See components/libssh2/CREDITS.md.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "libssh2_idf.h"   /* sets ESP32 for the gated libssh2 headers, then includes libssh2.h */
#include "disk.h"

extern void app_secure_quiesce(bool on);

static uint32_t s_lo;
static void sample(const char *tag) {
    uint32_t f = esp_get_free_heap_size();
    if (f < s_lo) s_lo = f;
    disk_logf("sshspike %s: free=%u lo=%u", tag, (unsigned)f, (unsigned)s_lo);
}

/* Never logs the password. host/user only. */
static void ssh_spike_run(const char *user, const char *pass, const char *host, uint16_t port) {
    int sock = -1;
    LIBSSH2_SESSION *session = NULL;
    LIBSSH2_CHANNEL *channel = NULL;
    struct addrinfo *res = NULL;
    uint32_t base;

    disk_logf("sshspike: start %s@%s:%u (quiescing httpd)", user, host, (unsigned)port);
    app_secure_quiesce(true);
    s_lo = esp_get_free_heap_size();
    base = s_lo;
    disk_logf("sshspike: post-quiesce free=%u", (unsigned)base);

    char ps[8];
    snprintf(ps, sizeof ps, "%u", (unsigned)port);
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    if (getaddrinfo(host, ps, &hints, &res) != 0 || !res) { disk_logf("sshspike: DNS fail"); goto restore; }
    sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) { disk_logf("sshspike: socket fail"); goto restore; }
    struct timeval tv = { .tv_sec = 15, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) { disk_logf("sshspike: connect fail"); goto restore; }
    sample("connected");

    extern size_t libssh2_session_struct_size(void);
    disk_logf("sshspike: sizeof(LIBSSH2_SESSION)=%u", (unsigned)libssh2_session_struct_size());
    int ir = libssh2_init(0);
    disk_logf("sshspike: libssh2_init=%d free=%u lfb=%u", ir,
              (unsigned)esp_get_free_heap_size(),
              (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    if (ir != 0) { disk_logf("sshspike: libssh2_init fail"); goto restore; }
    session = libssh2_session_init();
    if (!session) {
        disk_logf("sshspike: session_init NULL free=%u lfb=%u (stack hw=%u)",
                  (unsigned)esp_get_free_heap_size(),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                  (unsigned)uxTaskGetStackHighWaterMark(NULL));
        goto exit_lib;
    }
    libssh2_session_set_blocking(session, 1);
    sample("session");

    if (libssh2_session_handshake(session, sock)) { disk_logf("sshspike: handshake FAIL"); goto teardown; }
    sample("handshake");   /* key-exchange peak */

    if (libssh2_userauth_password(session, user, pass)) { disk_logf("sshspike: AUTH FAILED"); goto teardown; }
    sample("auth");

    channel = libssh2_channel_open_session(session);
    if (!channel) { disk_logf("sshspike: channel-open fail"); goto teardown; }
    libssh2_channel_request_pty(channel, "vanilla");
    if (libssh2_channel_shell(channel)) { disk_logf("sshspike: shell fail"); goto teardown; }
    sample("shell");

    {   /* read the banner/prompt + whatever the shell emits for ~4 s */
        char buf[512];
        int64_t end = esp_timer_get_time() + 4LL * 1000 * 1000;
        unsigned long total = 0;
        while (esp_timer_get_time() < end) {
            ssize_t n = libssh2_channel_read(channel, buf, sizeof buf);
            if (n > 0) { total += (unsigned long)n; sample("read"); }
            else break;   /* blocking session: <=0 means EOF/err/timeout */
        }
        disk_logf("sshspike: read %lu bytes", total);
    }
    sample("burst-done");

teardown:
    if (channel) { libssh2_channel_close(channel); libssh2_channel_free(channel); }
    if (session) { libssh2_session_disconnect(session, "spike done"); libssh2_session_free(session); }
exit_lib:
    libssh2_exit();
restore:
    if (sock >= 0) close(sock);
    if (res) freeaddrinfo(res);
    disk_logf("sshspike: VERDICT min-free heap = %u (base %u, heap-peak ~%u); task stack used = %u/20480",
              (unsigned)s_lo, (unsigned)base, (unsigned)(base - s_lo),
              (unsigned)(20480 - uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
    app_secure_quiesce(false);
    disk_logf("sshspike: httpd restored, done");
}

/* Run the spike on its own task: the handshake (bignum kex) + the read burst
 * block for several seconds and would otherwise stall the AT command handler /
 * trip the task watchdog. Results land in /disk-log; the AT command returns OK
 * immediately. */
typedef struct { char user[48], pass[64], host[80]; uint16_t port; } spike_args_t;

static void spike_task(void *p) {
    spike_args_t *a = (spike_args_t *)p;
    ssh_spike_run(a->user, a->pass, a->host, a->port);
    free(a);
    vTaskDelete(NULL);
}

void ssh_spike_start(const char *user, const char *pass, const char *host, uint16_t port) {
    spike_args_t *a = calloc(1, sizeof *a);
    if (!a) { disk_logf("sshspike: oom"); return; }
    strncpy(a->user, user, sizeof a->user - 1);
    strncpy(a->pass, pass, sizeof a->pass - 1);
    strncpy(a->host, host, sizeof a->host - 1);
    a->port = port;
    /* 20 KB stack: libssh2 + mbedTLS key exchange does heavy bignum/SHA math on
     * the stack (8 KB overflowed). The stack is also RAM -- the spike's verdict
     * reports the stack high-water so we count stack + heap together. CPU0. */
    if (xTaskCreatePinnedToCore(spike_task, "sshspike", 16384, a, 5, NULL, 0) != pdPASS) {
        disk_logf("sshspike: task spawn fail");
        free(a);
    }
}

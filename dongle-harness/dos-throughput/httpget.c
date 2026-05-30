/* httpget.c -- DOS / FOSSIL HTTP download throughput benchmark.
 *
 * Drives the dongle in HAYES (modem) mode -- no SLIP needed on the DOS
 * side, no TCP stack on the host. The dongle terminates TCP itself.
 * Workflow:
 *
 *   1. Probe FOSSIL on COMn.
 *   2. Send "ATH" then "AT$MODE=MODEM" (the dongle may have been left in
 *      SLIP by a prior throughput.exe run -- AT will be silently dropped
 *      if so, then the magic-frame escape brings it back).
 *   3. Dial: "ATDT<host>:<port>".
 *   4. Wait for "CONNECT". Drop into raw transparent mode.
 *   5. Send "GET <path> HTTP/1.0\r\nHost: <host>\r\nConnection: close\r\n\r\n"
 *   6. Read response. Parse Content-Length header. Skip to end-of-headers
 *      (CRLF CRLF). Count body bytes; time the body phase.
 *   7. After server hangs up or Content-Length satisfied: +++ guard, ATH.
 *   8. Print KB/s (body bytes / elapsed).
 *
 * Args (position-insensitive):
 *   comN         COM port number (default: autodetect via BDA)
 *   URL          one of:
 *                  host[:port]/path
 *                  http://host[:port]/path
 *                Defaults to a small public file if not given.
 *
 * Built for DOSBox-FOSSIL correctness and real-DOS-hardware perf in the
 * same binary. On DOSBox the number reflects DOSBox's nullmodem cap
 * (~1 KB/s), NOT the dongle. On real DOS with CHUSB it reflects the
 * actual end-to-end TCP throughput your WiFi + dongle can deliver.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dos.h>
#include <i86.h>

#define IN_BUF_SIZE   2048U
#define LINE_BUF_SIZE 512U

/* Big buffers as globals -- OW DOS medium model has a small default stack
 * (~4 KB) and stuffing 3 KB into main()'s frame was overflowing it
 * silently. Hello world built with the same recipe worked fine until
 * httpget added local 512+2048+256+128 byte arrays. */
static char host[128];
static char path[256];
static char line[LINE_BUF_SIZE];
static unsigned char buf[IN_BUF_SIZE];

/* ---- INT 14h thin wrapper ---- */
static unsigned short int14(unsigned char ah, unsigned char al,
                            unsigned bx, unsigned cx, unsigned dx)
{
    union REGS r;
    r.h.ah = ah; r.h.al = al;
    r.x.bx = bx; r.x.cx = cx; r.x.dx = dx;
    int86(0x14, &r, &r);
    return r.x.ax;
}

static void dos_yield(void)
{
    union REGS r;
    int86(0x28, &r, &r);
}

static int probe_fossil(unsigned dx)
{
    union REGS r;
    r.h.ah = 0x04U; r.h.al = 0xFFU;
    r.x.bx = 0x4F50U; r.x.dx = dx;
    int86(0x14, &r, &r);
    return (r.x.ax == 0x1954U);
}

/* BIOS ticks @ 18.2 Hz from BDA 0040:006C. */
static unsigned long bios_ticks(void)
{
    return *(unsigned long __far *)MK_FP(0x0040U, 0x006CU);
}

/* FOSSIL AH=0Bh send-no-wait. Retries forever with INT 28h yields. */
static int fossil_send(unsigned port, unsigned char ch)
{
    unsigned long tries;
    for (tries = 0UL; ; ++tries) {
        if (int14(0x0B, ch, 0, 0, port) != 0U) return 1;
        if ((tries & 0xFFUL) == 0xFFUL) dos_yield();
    }
}

static int fossil_send_str(unsigned port, const char *s)
{
    while (*s) { if (!fossil_send(port, (unsigned char)*s)) return 0; ++s; }
    return 1;
}

/* FOSSIL AH=19h block-write. Returns bytes accepted. ES:DI=buf, CX=count. */
static unsigned fossil_send_block(unsigned port,
                                  const unsigned char __far *buf, unsigned count)
{
    struct SREGS s; union REGS r;
    segread(&s);
    s.es = FP_SEG(buf);
    r.x.di = FP_OFF(buf);
    r.h.ah = 0x19U;
    r.x.cx = count;
    r.x.dx = port;
    int86x(0x14, &r, &r, &s);
    return r.x.ax;
}

/* Read one byte non-blocking via FOSSIL status + read. AH=03h returns
 * line status in AH (bit 0 = data ready); if set we then call AH=02h
 * to actually consume the byte. (Tried AH=18h block-read and AH=0Ch
 * peek; AH=18h appears unimplemented in BNU 2.02 -- always returns 0 --
 * and AH=0Ch ambiguity made empty look like data.) */
static int fossil_recv_nowait(unsigned port)
{
    union REGS r;
    r.h.ah = 0x03U; r.x.dx = port;
    int86(0x14, &r, &r);
    if ((r.h.ah & 0x01U) == 0U) return -1;   /* no byte ready */
    r.h.ah = 0x02U; r.x.dx = port;
    int86(0x14, &r, &r);
    return (int)(unsigned)r.h.al;
}

/* Bulk receive: drain the FIFO via repeated AH=03h+AH=02h. Returns count.
 * For BNU + DOSBox nullmodem the FOSSIL bulk-read (AH=18h) doesn't work,
 * so this loop is the best we can do. Stops at count or when FIFO empties. */
static unsigned fossil_recv_block(unsigned port,
                                  unsigned char __far *buf, unsigned count)
{
    unsigned n = 0;
    while (n < count) {
        int c = fossil_recv_nowait(port);
        if (c < 0) break;
        buf[n++] = (unsigned char)c;
    }
    return n;
}

static void wait_ms(unsigned ms)
{
    unsigned long target = ((unsigned long)ms * 182UL + 9999UL) / 10000UL;
    unsigned long t0 = bios_ticks();
    if (target < 1UL) target = 1UL;
    while ((bios_ticks() - t0) < target) { dos_yield(); }
}

/* Drain RX into oblivion for `ms` milliseconds. */
static void drain_rx(unsigned port, unsigned ms)
{
    unsigned long target = ((unsigned long)ms * 182UL + 9999UL) / 10000UL;
    unsigned long t0 = bios_ticks();
    unsigned char buf[256];
    if (target < 1UL) target = 1UL;
    while ((bios_ticks() - t0) < target) {
        (void)fossil_recv_block(port, (unsigned char __far *)buf, sizeof(buf));
        dos_yield();
    }
}

/* Wait for a complete line (terminated by CRLF) into `line`, with a
 * wall-clock timeout in ms. Returns line length (without CRLF) or -1 on
 * timeout / overflow. Stops at LF; tolerates lone LF. */
static int read_line(unsigned port, char *line, unsigned cap, unsigned timeout_ms)
{
    unsigned long deadline = bios_ticks()
                           + ((unsigned long)timeout_ms * 182UL + 9999UL) / 10000UL;
    unsigned n = 0;
    while (bios_ticks() < deadline) {
        int c = fossil_recv_nowait(port);
        if (c < 0) { dos_yield(); continue; }
        if (c == '\r') continue;
        if (c == '\n') { line[n] = '\0'; return (int)n; }
        if (n + 1 < cap) line[n++] = (char)c;
        else return -1;
    }
    return -1;
}

/* Wait for `needle` to appear in the RX stream, with timeout. Tolerates
 * intermixed text -- used to spot "CONNECT" after ATDT. */
static int wait_for(unsigned port, const char *needle, unsigned timeout_ms)
{
    unsigned long deadline = bios_ticks()
                           + ((unsigned long)timeout_ms * 182UL + 9999UL) / 10000UL;
    const char *m = needle;
    while (bios_ticks() < deadline) {
        int c = fossil_recv_nowait(port);
        if (c < 0) { dos_yield(); continue; }
        if ((unsigned char)c == (unsigned char)*m) {
            ++m;
            if (*m == '\0') return 1;
        } else {
            m = (((unsigned char)c == (unsigned char)*needle) ? needle + 1 : needle);
        }
    }
    return 0;
}

/* Parse URL [http://]host[:port][/path] -> host, port, path. */
static int parse_url(const char *url, char *host, unsigned host_cap,
                     unsigned *port_out, char *path, unsigned path_cap)
{
    const char *p = url;
    unsigned n;
    if (!strncmp(p, "http://", 7)) p += 7;
    n = 0;
    while (*p && *p != ':' && *p != '/') {
        if (n + 1 >= host_cap) return 0;
        host[n++] = *p++;
    }
    host[n] = '\0';
    if (n == 0) return 0;
    if (*p == ':') {
        unsigned port = 0;
        ++p;
        while (*p >= '0' && *p <= '9') { port = port * 10U + (unsigned)(*p - '0'); ++p; }
        *port_out = port ? port : 80U;
    } else {
        *port_out = 80U;
    }
    if (*p == '/') {
        n = 0;
        while (*p) {
            if (n + 1 >= path_cap) return 0;
            path[n++] = *p++;
        }
        path[n] = '\0';
    } else {
        if (path_cap < 2) return 0;
        path[0] = '/'; path[1] = '\0';
    }
    return 1;
}

/* Autodetect COM index from BDA 0040:0000 (mirrors usbterm). Returns
 * 0..3 = COM1..COM4 or -1 if none. */
static int autodetect_com(void)
{
    unsigned i;
    unsigned short __far *bda = (unsigned short __far *)MK_FP(0x0040U, 0x0000U);
    for (i = 0; i < 4U; ++i) {
        if (bda[i] != 0) return (int)i;
    }
    return -1;
}

int main(int argc, char **argv)
{
    int port_index = -1;
    unsigned port_num = 80;
    const char *url = NULL;
    int argi;
    unsigned long t_dial = 0, t_connected = 0, t_body_start = 0, t_done = 0;

    /* Redirected stdout (DOSBox autoexec `> RESULT.TXT`) is fully buffered
     * by Open Watcom's CRT -- a hang or kill mid-run would lose every
     * printf. Make stdout unbuffered so the harness always sees progress. */
    setbuf(stdout, NULL);

    printf("HTTPGET: T-Dongle S3 HTTP download benchmark (DOS / FOSSIL)\n");

    /* Parse argv: comN | URL */
    for (argi = 1; argi < argc; ++argi) {
        const char *a = argv[argi];
        if ((a[0] == 'c' || a[0] == 'C') &&
            (a[1] == 'o' || a[1] == 'O') &&
            (a[2] == 'm' || a[2] == 'M') &&
             a[3] >= '1' && a[3] <= '4' && a[4] == '\0') {
            port_index = a[3] - '1';
        } else if (a[0] == '-' && (a[1] == 'h' || a[1] == 'H')) {
            fprintf(stderr,
                "usage: httpget [comN] URL\n"
                "  URL like  192.168.1.226:8000/big.bin   (local server)\n"
                "       or   http://speedtest.tele2.net/1MB.zip\n");
            return 0;
        } else {
            url = a;
        }
    }
    if (!url) {
        /* Default: a small Cloudflare speedtest mirror. Replace with your
         * own if it goes away. */
        url = "speed.cloudflare.com/__down?bytes=1048576";
    }
    if (port_index < 0) {
        port_index = autodetect_com();
        if (port_index < 0) {
            fprintf(stderr, "HTTPGET: no COM port found in BDA.\n");
            return 1;
        }
    }

    if (!probe_fossil((unsigned)port_index)) {
        fprintf(stderr,
            "HTTPGET: FOSSIL not detected on COM%d.\n"
            "         Load CHUSB (or BNU under DOSBox) and retry.\n",
            port_index + 1);
        return 1;
    }

    /* Line init 115200,N,8,1 via AH=1Eh. */
    (void)int14(0x1E, 0x01, 0x0000,
                (unsigned)((3 << 8) | 11), (unsigned)port_index);

    if (!parse_url(url, host, sizeof(host), &port_num, path, sizeof(path))) {
        fprintf(stderr, "HTTPGET: bad URL: %s\n", url);
        return 1;
    }

    printf("         port=COM%d  host=%s:%u  path=%s\n",
           port_index + 1, host, port_num, path);
    fflush(stdout);

    /* Send a bare CR to flush whatever line the AT engine was assembling,
     * then ATH to hang up any prior call. No +++ / no magic-frame here --
     * the harness is responsible for ensuring the dongle is in MODEM
     * command mode before invocation (run-httpget.sh's recovery does this). */
    drain_rx((unsigned)port_index, 100);
    fossil_send((unsigned)port_index, '\r');
    wait_ms(200);
    drain_rx((unsigned)port_index, 100);
    fossil_send_str((unsigned)port_index, "ATH\r");
    wait_ms(400);
    drain_rx((unsigned)port_index, 100);

    /* PHASE A: dial. Time ATDT -> CONNECT separately. */
    {
        char dial[200];
        snprintf(dial, sizeof(dial), "ATDT%s:%u\r", host, port_num);
        printf("         %s", dial);
        fflush(stdout);
        t_dial = bios_ticks();
        fossil_send_str((unsigned)port_index, dial);
    }
    if (!wait_for((unsigned)port_index, "CONNECT", 20000)) {
        printf("HTTPGET: no CONNECT (timeout after 20s).\n");
        printf("         RX buffer follows:\n");
        {
            int c, n = 0;
            while ((c = fossil_recv_nowait((unsigned)port_index)) >= 0 && n < 200) {
                putchar(c); ++n;
            }
            printf("\n         (%d bytes)\n", n);
        }
        return 2;
    }
    t_connected = bios_ticks();
    /* Eat the trailing CR/LF after CONNECT, plus any text up to LF. */
    {
        int c;
        unsigned long t = bios_ticks() + 18UL;   /* ~1s slack */
        while (bios_ticks() < t) {
            c = fossil_recv_nowait((unsigned)port_index);
            if (c < 0) { dos_yield(); continue; }
            if (c == '\n') break;
        }
    }
    printf("         connected, sending GET...\n");
    fflush(stdout);

    /* Build + send GET. HTTP/1.0 + Connection: close => server hangs
     * up after body, which is our reliable end-of-stream signal even
     * if Content-Length is absent. */
    {
        static char req[512];
        int reqlen = snprintf(req, sizeof(req),
            "GET %s HTTP/1.0\r\nHost: %s\r\n"
            "User-Agent: tdongle-httpget/1\r\nConnection: close\r\n\r\n",
            path, host);
        printf("         GET %s HTTP/1.0\n", path);
        fflush(stdout);
        fossil_send_block((unsigned)port_index,
                          (const unsigned char __far *)req, (unsigned)reqlen);
    }

    /* Parse response: status, headers, body. We require status 200.
     * Track Content-Length if present, but always rely on EOF too. */
    {
        int n;
        unsigned long content_length = 0UL;
        int have_clen = 0;
        n = read_line((unsigned)port_index, line, sizeof(line), 15000);
        if (n < 0) { fprintf(stderr, "HTTPGET: status-line timeout.\n"); return 3; }
        printf("         %s\n", line);
        if (strncmp(line, "HTTP/", 5) != 0 ||
            (strstr(line, " 200 ") == NULL && strstr(line, "200") == NULL)) {
            fprintf(stderr, "HTTPGET: non-200 status.\n"); return 4;
        }
        for (;;) {
            n = read_line((unsigned)port_index, line, sizeof(line), 15000);
            if (n < 0) { fprintf(stderr, "HTTPGET: header timeout.\n"); return 5; }
            if (n == 0) break;            /* blank line -- body starts */
            /* Case-insensitive prefix "Content-Length:" */
            if ((line[0] == 'C' || line[0] == 'c') &&
                (line[1] == 'o' || line[1] == 'O') &&
                strlen(line) > 16 &&
                ((line[7] == '-' || line[7] == '-') ||
                 (line[8] == 'L' || line[8] == 'l'))) {
                const char *p = line;
                while (*p && *p != ':') ++p;
                if (*p == ':') {
                    ++p;
                    while (*p == ' ' || *p == '\t') ++p;
                    {
                        unsigned long v = 0UL;
                        while (*p >= '0' && *p <= '9') {
                            v = v * 10UL + (unsigned long)(*p - '0');
                            ++p;
                        }
                        content_length = v; have_clen = 1;
                    }
                }
            }
        }
        if (have_clen)
            printf("         Content-Length: %lu\n", content_length);
        else
            printf("         (no Content-Length -- timing to EOF)\n");
        fflush(stdout);

        /* BODY: count bytes, time. End on Content-Length satisfied OR
         * long silence (server hung up). */
        {
            unsigned long got = 0UL;
            unsigned long last_byte_ticks;
            /* End-of-stream heuristic: 90 ticks (~5s) without any byte.
             * For very slow links bump this. */
            const unsigned long silence_ticks = 90UL;
            t_body_start = bios_ticks();
            last_byte_ticks = t_body_start;
            for (;;) {
                unsigned r = fossil_recv_block((unsigned)port_index,
                    (unsigned char __far *)buf, sizeof(buf));
                if (r) {
                    got += (unsigned long)r;
                    last_byte_ticks = bios_ticks();
                    if (have_clen && got >= content_length) break;
                } else {
                    if ((bios_ticks() - last_byte_ticks) > silence_ticks) break;
                    dos_yield();
                }
            }
            t_done = bios_ticks();
            {
                unsigned long body_ticks    = t_done - t_body_start;
                unsigned long connect_ticks = t_connected - t_dial;
                unsigned long header_ticks  = t_body_start - t_connected;
                unsigned long total_ticks   = t_done - t_dial;
                unsigned long centi_body, centi_conn, centi_hdr, centi_total;
                unsigned long tenths_kbps;
                unsigned long kb, frac;
                if (body_ticks == 0UL) body_ticks = 1UL;
                centi_body  = (body_ticks    * 10000UL + 910UL) / 1820UL;
                centi_conn  = (connect_ticks * 10000UL + 910UL) / 1820UL;
                centi_hdr   = (header_ticks  * 10000UL + 910UL) / 1820UL;
                centi_total = (total_ticks   * 10000UL + 910UL) / 1820UL;
                tenths_kbps = (got * 1820UL / body_ticks * 10UL) / 102400UL;
                kb   = tenths_kbps / 10UL;
                frac = tenths_kbps % 10UL;
                printf("         body %lu bytes in %lu.%02lus = %lu.%lu KB/s\n",
                       got, centi_body / 100UL, centi_body % 100UL, kb, frac);
                printf("         phases: dial=%lu.%02lus headers=%lu.%02lus body=%lu.%02lus total=%lu.%02lus\n",
                       centi_conn / 100UL, centi_conn % 100UL,
                       centi_hdr / 100UL, centi_hdr % 100UL,
                       centi_body / 100UL, centi_body % 100UL,
                       centi_total / 100UL, centi_total % 100UL);
            }
        }
    }

    /* Hang up: +++ guard, ATH. The dongle goes back to command mode. */
    wait_ms(1200);
    fossil_send_str((unsigned)port_index, "+++");
    wait_ms(1500);
    drain_rx((unsigned)port_index, 200);
    fossil_send_str((unsigned)port_index, "ATH\r");
    wait_ms(300);

    return 0;
}

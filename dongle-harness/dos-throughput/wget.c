/* wget.c -- DOS / FOSSIL HTTP downloader for the T-Dongle S3.
 *
 * Like httpget.c (the throughput benchmark) but SAVES the body to a file
 * and AUTO-SWITCHES the dongle SLIP->MODEM if needed:
 *
 *   1. Probe FOSSIL on COMn (CHUSB provides INT 14h).
 *   2. ensure_modem(): probe "AT"; if no "OK" (we're in SLIP), send the
 *      magic frame  C0 'MODE=MODEM' C0  to flip the dongle to MODEM, then
 *      re-probe. Harmless if already in Hayes -- the AT probe just OKs and
 *      the frame is skipped.
 *   3. ATH (hang up any prior call), ATNET0 (transparent pipe -- HTTP needs
 *      it; default telnet IAC corrupts the request).
 *   4. ATDT<host>:<port>, wait CONNECT, send GET (HTTP/1.0, Connection: close).
 *   5. Stream the body to <outfile>, using AH=18h block-read when available
 *      (CHUSB does -- the fast path). Cap at Content-Length, else hold back
 *      and strip the dongle's trailing "\r\nNO CARRIER".
 *   6. +++ / ATH to hang up. Print bytes + KB/s.
 *
 * Usage:  WGET [comN] <url> [outfile]
 *   url       host[:port]/path  or  http://host[:port]/path
 *   outfile   8.3 name; derived from the URL basename if omitted (else WGET.DAT)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dos.h>
#include <i86.h>

#define IN_BUF_SIZE   2048U

static char host[128];
static char path[256];
static char line[512];
static char outfile[16];
static unsigned char buf[IN_BUF_SIZE];

/* ---- INT 14h thin wrapper (verbatim from httpget.c) ---- */
static unsigned short int14(unsigned char ah, unsigned char al,
                            unsigned bx, unsigned cx, unsigned dx)
{
    union REGS r;
    r.h.ah = ah; r.h.al = al;
    r.x.bx = bx; r.x.cx = cx; r.x.dx = dx;
    int86(0x14, &r, &r);
    return r.x.ax;
}
static void dos_yield(void) { union REGS r; int86(0x28, &r, &r); }

static int g_has_block_read = 0;
static int probe_fossil(unsigned dx)
{
    union REGS r;
    r.h.ah = 0x04U; r.h.al = 0xFFU; r.x.bx = 0x4F50U; r.x.dx = dx;
    int86(0x14, &r, &r);
    if (r.x.ax != 0x1954U) return 0;
    g_has_block_read = (r.h.bl >= 0x18U);
    return 1;
}
static unsigned long bios_ticks(void)
{
    return *(unsigned long __far *)MK_FP(0x0040U, 0x006CU);
}
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
static unsigned fossil_send_block(unsigned port,
                                  const unsigned char __far *b, unsigned count)
{
    struct SREGS s; union REGS r;
    segread(&s); s.es = FP_SEG(b); r.x.di = FP_OFF(b);
    r.h.ah = 0x19U; r.x.cx = count; r.x.dx = port;
    int86x(0x14, &r, &r, &s);
    return r.x.ax;
}
static unsigned fossil_block_read(unsigned port, unsigned char __far *b, unsigned count)
{
    struct SREGS s; union REGS r;
    segread(&s); s.es = FP_SEG(b); r.x.di = FP_OFF(b);
    r.h.ah = 0x18U; r.x.cx = count; r.x.dx = port;
    int86x(0x14, &r, &r, &s);
    return r.x.ax;
}
static int fossil_recv_nowait(unsigned port)
{
    union REGS r;
    r.h.ah = 0x03U; r.x.dx = port; int86(0x14, &r, &r);
    if ((r.h.ah & 0x01U) == 0U) return -1;
    r.h.ah = 0x02U; r.x.dx = port; int86(0x14, &r, &r);
    return (int)(unsigned)r.h.al;
}
static unsigned fossil_recv_block(unsigned port, unsigned char __far *b, unsigned count)
{
    if (g_has_block_read) return fossil_block_read(port, b, count);
    {
        unsigned n = 0;
        while (n < count) { int c = fossil_recv_nowait(port); if (c < 0) break; b[n++] = (unsigned char)c; }
        return n;
    }
}
static void wait_ms(unsigned ms)
{
    unsigned long target = ((unsigned long)ms * 182UL + 9999UL) / 10000UL;
    unsigned long t0 = bios_ticks();
    if (target < 1UL) target = 1UL;
    while ((bios_ticks() - t0) < target) dos_yield();
}
static void drain_rx(unsigned port, unsigned ms)
{
    unsigned long target = ((unsigned long)ms * 182UL + 9999UL) / 10000UL;
    unsigned long t0 = bios_ticks();
    unsigned char d[256];
    if (target < 1UL) target = 1UL;
    while ((bios_ticks() - t0) < target) {
        (void)fossil_recv_block(port, (unsigned char __far *)d, sizeof(d));
        dos_yield();
    }
}
static int read_line(unsigned port, char *ln, unsigned cap, unsigned timeout_ms)
{
    unsigned long deadline = bios_ticks() + ((unsigned long)timeout_ms * 182UL + 9999UL) / 10000UL;
    unsigned n = 0;
    while (bios_ticks() < deadline) {
        int c = fossil_recv_nowait(port);
        if (c < 0) { dos_yield(); continue; }
        if (c == '\r') continue;
        if (c == '\n') { ln[n] = '\0'; return (int)n; }
        if (n + 1 < cap) ln[n++] = (char)c; else return -1;
    }
    return -1;
}
static int wait_for(unsigned port, const char *needle, unsigned timeout_ms)
{
    unsigned long deadline = bios_ticks() + ((unsigned long)timeout_ms * 182UL + 9999UL) / 10000UL;
    const char *m = needle;
    while (bios_ticks() < deadline) {
        int c = fossil_recv_nowait(port);
        if (c < 0) { dos_yield(); continue; }
        if ((unsigned char)c == (unsigned char)*m) { ++m; if (*m == '\0') return 1; }
        else m = (((unsigned char)c == (unsigned char)*needle) ? needle + 1 : needle);
    }
    return 0;
}
/* Watch the dial response for CONNECT *or* any failure result code, so a
 * failed dial bails immediately instead of stalling the full timeout
 * waiting for a CONNECT that will never come (the old "20s timeout" bug:
 * the dongle answered NO CARRIER at once and we ignored it). Each needle
 * is matched independently against the byte stream (echoes/ordering safe).
 * Returns 1=CONNECT, 0=timeout, -1=NO DIALTONE (host not resolved),
 * -2=NO CARRIER (connect failed/refused), -3=BUSY, -4=ERROR/NO ANSWER. */
#define DIAL_NEEDLES 6
static int wait_dial_result(unsigned port, unsigned timeout_ms)
{
    static const char * const needles[DIAL_NEEDLES] = {
        "CONNECT", "NO DIALTONE", "NO CARRIER", "BUSY", "NO ANSWER", "ERROR" };
    static const int codes[DIAL_NEEDLES] = { 1, -1, -2, -3, -4, -4 };
    unsigned mp[DIAL_NEEDLES];
    unsigned long deadline = bios_ticks() + ((unsigned long)timeout_ms * 182UL + 9999UL) / 10000UL;
    int i;
    for (i = 0; i < DIAL_NEEDLES; ++i) mp[i] = 0;
    while (bios_ticks() < deadline) {
        int c = fossil_recv_nowait(port);
        if (c < 0) { dos_yield(); continue; }
        for (i = 0; i < DIAL_NEEDLES; ++i) {
            if ((unsigned char)c == (unsigned char)needles[i][mp[i]]) {
                if (needles[i][++mp[i]] == '\0') return codes[i];
            } else {
                mp[i] = ((unsigned char)c == (unsigned char)needles[i][0]) ? 1u : 0u;
            }
        }
    }
    return 0;
}
static int parse_url(const char *url, char *h, unsigned hcap,
                     unsigned *port_out, char *pth, unsigned pcap)
{
    const char *p = url; unsigned n;
    if (!strncmp(p, "http://", 7)) p += 7;
    n = 0;
    while (*p && *p != ':' && *p != '/') { if (n + 1 >= hcap) return 0; h[n++] = *p++; }
    h[n] = '\0'; if (n == 0) return 0;
    if (*p == ':') { unsigned port = 0; ++p;
        while (*p >= '0' && *p <= '9') { port = port * 10U + (unsigned)(*p - '0'); ++p; }
        *port_out = port ? port : 80U;
    } else *port_out = 80U;
    if (*p == '/') { n = 0; while (*p) { if (n + 1 >= pcap) return 0; pth[n++] = *p++; } pth[n] = '\0'; }
    else { if (pcap < 2) return 0; pth[0] = '/'; pth[1] = '\0'; }
    return 1;
}
static int autodetect_com(void)
{
    unsigned i; unsigned short __far *bda = (unsigned short __far *)MK_FP(0x0040U, 0x0000U);
    for (i = 0; i < 4U; ++i) if (bda[i] != 0) return (int)i;
    return -1;
}

/* Flip the dongle to MODEM if it isn't already. Probe AT; on no "OK"
 * (SLIP mode swallows AT as frame data) send the magic SLIP frame the
 * firmware watches for, then re-probe. No-op (harmless) when already in
 * Hayes -- the first probe OKs and we skip the frame. */
static int ensure_modem(unsigned port)
{
    static const unsigned char frame[12] =
        { 0xC0, 'M','O','D','E','=','M','O','D','E','M', 0xC0 };
    unsigned i;
    drain_rx(port, 100);
    fossil_send_str(port, "\rAT\r");
    if (wait_for(port, "OK", 700)) return 1;            /* already MODEM */
    printf("         no AT reply -- sending SLIP->MODEM escape frame\n");
    fflush(stdout);
    /* The frame's leading C0 also closes any partial 'AT' frame the SLIP
     * decoder was accumulating, so the payload parses cleanly. */
    for (i = 0; i < sizeof(frame); ++i) fossil_send(port, frame[i]);
    wait_ms(400);
    drain_rx(port, 100);
    fossil_send_str(port, "\rAT\r");
    return wait_for(port, "OK", 1500) ? 1 : 0;
}

/* Derive an 8.3 output name from the URL path basename. */
static void derive_filename(const char *pth, char *out, unsigned cap)
{
    const char *base = pth, *p = pth;
    unsigned n = 0, dot = 0;
    for (; *p; ++p) if (*p == '/') base = p + 1;
    for (p = base; *p && *p != '?'; ++p) {
        char c = *p;
        if (c == '.') { if (dot) continue; dot = 1; }
        else if (!((c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')||c=='_'||c=='-')) continue;
        if (c>='a'&&c<='z') c = (char)(c - 'a' + 'A');
        if (n + 1 < cap) out[n++] = c;
    }
    out[n] = '\0';
    if (n == 0 || out[0] == '.') strcpy(out, "WGET.DAT");
}

/* Outcome of one dial+GET+stream attempt. */
enum { ATT_OK = 0, ATT_INCOMPLETE, ATT_DIAL_FAIL, ATT_HTTP_FATAL, ATT_IGNORED_RANGE };

/* Return to command mode and drop any active call, so the next ATDT dials clean.
 * +++ (with the Hayes 1s guards) escapes online-data mode; ATH hangs up. Safe to
 * call whether the dongle is online or already NO CARRIER'd. */
static void hangup_call(unsigned port)
{
    wait_ms(1200);
    fossil_send_str(port, "+++"); wait_ms(1500);
    drain_rx(port, 200);
    fossil_send_str(port, "ATH\r"); wait_ms(300);
    drain_rx(port, 100);
}

/* One download attempt: dial -> GET (with Range if resume_from>0) -> parse
 * status/headers -> stream body into f (positioned at resume_from). Sets total
 * and have_total when the response reveals the full size; sets written to the
 * absolute on-disk byte count. Returns an ATT_* code. */
static int do_attempt(unsigned port, const char *h, unsigned pnum, const char *pth,
                      FILE *f, unsigned long resume_from,
                      unsigned long *total, int *have_total, unsigned long *written)
{
    int n, status = 0;
    unsigned long clen = 0UL;     int have_clen = 0;
    unsigned long cr_start = 0UL, cr_total = 0UL; int have_cr = 0;
    unsigned long last_byte_ticks, last_progress_ticks, t0, max_ticks, silence_ticks;
    unsigned char hold[16]; unsigned held = 0;

    /* --- dial --- */
    drain_rx(port, 100);
    fossil_send_str(port, "ATH\r");    wait_ms(400); drain_rx(port, 100);
    fossil_send_str(port, "ATNET0\r"); wait_ms(300); drain_rx(port, 100);
    {
        char dial[200];
        snprintf(dial, sizeof(dial), "ATDT%s:%u\r", h, pnum);
        fossil_send_str(port, dial);
    }
    switch (wait_dial_result(port, 20000)) {
        case 1: break;   /* CONNECT */
        /* All dial failures are RETRYABLE: on a resume, the very drop we're
         * recovering from may still have WiFi down at re-dial time (getaddrinfo
         * fails -> NO DIALTONE). The no-progress guard (3 attempts) bounds a
         * genuinely bad host. */
        case -1: fprintf(stderr, "WGET: host not resolved (%s) -- retrying.\n", h); return ATT_DIAL_FAIL;
        default: fprintf(stderr, "WGET: dial failed -- retrying.\n");             return ATT_DIAL_FAIL;
    }
    { int c; unsigned long t = bios_ticks() + 18UL;   /* eat CR/LF after CONNECT */
      while (bios_ticks() < t) { c = fossil_recv_nowait(port);
          if (c < 0) { dos_yield(); continue; } if (c == '\n') break; } }

    /* --- GET (+ Range header on resume) --- */
    {
        static char req[600];
        int reqlen;
        if (resume_from > 0UL)
            reqlen = snprintf(req, sizeof(req),
                "GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: tdongle-wget/1\r\n"
                "Range: bytes=%lu-\r\nConnection: close\r\n\r\n", pth, h, resume_from);
        else
            reqlen = snprintf(req, sizeof(req),
                "GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: tdongle-wget/1\r\n"
                "Connection: close\r\n\r\n", pth, h);
        fossil_send_block(port, (const unsigned char __far *)req, (unsigned)reqlen);
    }

    /* --- status line (parse numeric code: accept 200 full / 206 partial) --- */
    n = read_line(port, line, sizeof(line), 15000);
    if (n < 0) { fprintf(stderr, "WGET: status timeout.\n"); return ATT_INCOMPLETE; }
    printf("         %s\n", line);
    if (strncmp(line, "HTTP/", 5) != 0) { fprintf(stderr, "WGET: bad status line.\n"); return ATT_HTTP_FATAL; }
    {
        const char *p = line + 5;
        while (*p && *p != ' ') ++p;        /* skip version */
        while (*p == ' ') ++p;
        while (*p >= '0' && *p <= '9') { status = status*10 + (int)(*p - '0'); ++p; }
    }

    /* --- headers: Content-Length and (for 206) Content-Range --- */
    for (;;) {
        n = read_line(port, line, sizeof(line), 15000);
        if (n < 0) { fprintf(stderr, "WGET: header timeout.\n"); return ATT_INCOMPLETE; }
        if (n == 0) break;
        if (!strncmp(line,"Content-Length:",15) || !strncmp(line,"content-length:",15)) {
            const char *p = line + 15; unsigned long v = 0UL;
            while (*p==' '||*p=='\t') ++p;
            while (*p>='0'&&*p<='9') { v = v*10UL + (unsigned long)(*p-'0'); ++p; }
            clen = v; have_clen = 1;
        } else if (!strncmp(line,"Content-Range:",14) || !strncmp(line,"content-range:",14)) {
            /* Content-Range: bytes START-END/TOTAL */
            const char *p = line + 14;
            while (*p && (*p<'0'||*p>'9')) ++p;                       /* skip " bytes " */
            while (*p>='0'&&*p<='9') { cr_start = cr_start*10UL + (unsigned long)(*p-'0'); ++p; }
            while (*p && *p != '/') ++p;                              /* skip -END */
            if (*p == '/') ++p;
            while (*p>='0'&&*p<='9') { cr_total = cr_total*10UL + (unsigned long)(*p-'0'); ++p; }
            have_cr = 1;
        }
    }

    /* --- reconcile status vs the Range we asked for --- */
    if (status == 200) {
        if (resume_from > 0UL) return ATT_IGNORED_RANGE;   /* server ignored Range -> restart fresh */
        *total = clen; *have_total = have_clen;
    } else if (status == 206) {
        if (!have_cr || cr_start != resume_from) return ATT_IGNORED_RANGE; /* not the range we hold */
        *total = cr_total; *have_total = 1;
    } else {
        fprintf(stderr, "WGET: HTTP status %d (not 200/206).\n", status);
        return ATT_HTTP_FATAL;
    }
    if (*have_total) printf("         total %lu, have %lu\n", *total, resume_from);
    else             printf("         (no size -- single attempt, no resume)\n");

    /* --- stream body into f at the resume offset --- */
    fseek(f, (long)resume_from, SEEK_SET);
    *written = resume_from;
    silence_ticks = *have_total ? 273UL : 90UL;   /* 15s sized / 5s unsized */
    max_ticks = 5460UL;                            /* ~300s */
    t0 = bios_ticks(); last_byte_ticks = t0; last_progress_ticks = t0;
    for (;;) {
        unsigned r = fossil_recv_block(port, (unsigned char __far *)buf, sizeof(buf));
        if (r) {
            last_byte_ticks = bios_ticks();
            if (*have_total) {
                unsigned tow = r;
                if (*written + (unsigned long)tow > *total)
                    tow = (unsigned)(*total - *written);
                if (tow) {
                    unsigned w = (unsigned)fwrite(buf, 1, tow, f);
                    *written += (unsigned long)w;
                    if (w < tow) { fprintf(stderr, "\nWGET: disk write failed (full?).\n"); return ATT_HTTP_FATAL; }
                }
                if (*written >= *total) break;     /* done; ignore trailing NO CARRIER */
            } else {
                /* unsized: hold back 16 B so the trailing "\r\nNO CARRIER" is stripped */
                unsigned ri = 0;
                while (ri < r) {
                    if (held == sizeof(hold)) { fputc(hold[0], f); (*written)++;
                        memmove(hold, hold+1, sizeof(hold)-1); held--; }
                    hold[held++] = buf[ri++];
                }
            }
        } else {
            if ((bios_ticks() - last_byte_ticks) > silence_ticks) break;
            dos_yield();
        }
        if ((bios_ticks() - last_progress_ticks) > 36UL) {
            last_progress_ticks = bios_ticks();
            printf("         ... %lu bytes\r", *written);
        }
        if ((bios_ticks() - t0) > max_ticks) { printf("\n         (5 min cap)\n"); break; }
    }
    if (!*have_total) {
        unsigned cut = held, i;
        for (i = 0; i + 1 < held; ++i)
            if (hold[i]=='\r' && hold[i+1]=='\n' && i+12 <= held && memcmp(hold+i,"\r\nNO CARRIER",12)==0) { cut=i; break; }
        if (cut) { fwrite(hold, 1, cut, f); *written += cut; }
    }
    fflush(f);
    printf("\n");
    if (!*have_total)              return ATT_OK;          /* unsized best-effort done */
    if (*written >= *total)        return ATT_OK;
    return ATT_INCOMPLETE;
}

int main(int argc, char **argv)
{
    int port_index = -1;
    unsigned port_num = 80;
    const char *url = NULL, *ofarg = NULL;
    int argi;
    unsigned long t_start;
    FILE *f;

    setbuf(stdout, NULL);
    printf("WGET: T-Dongle S3 HTTP downloader (DOS / FOSSIL)\n");

    for (argi = 1; argi < argc; ++argi) {
        const char *a = argv[argi];
        if ((a[0]=='c'||a[0]=='C')&&(a[1]=='o'||a[1]=='O')&&(a[2]=='m'||a[2]=='M')&&
             a[3]>='1'&&a[3]<='4'&&a[4]=='\0') port_index = a[3]-'1';
        else if (a[0]=='-'&&(a[1]=='h'||a[1]=='H')) {
            fprintf(stderr, "usage: WGET [comN] <url> [outfile]\n"); return 0;
        } else if (!url) url = a;
        else ofarg = a;
    }
    if (!url) { fprintf(stderr, "WGET: need a URL.  WGET [comN] <url> [outfile]\n"); return 1; }
    if (port_index < 0) {
        port_index = autodetect_com();
        if (port_index < 0) { fprintf(stderr, "WGET: no COM port in BDA.\n"); return 1; }
    }
    if (!probe_fossil((unsigned)port_index)) {
        fprintf(stderr, "WGET: FOSSIL not detected on COM%d (load CHUSB).\n", port_index+1);
        return 1;
    }
    (void)int14(0x1E, 0x01, 0x0000, (unsigned)((3<<8)|11), (unsigned)port_index);
    if (!parse_url(url, host, sizeof(host), &port_num, path, sizeof(path))) {
        fprintf(stderr, "WGET: bad URL: %s\n", url); return 1;
    }
    if (ofarg) { strncpy(outfile, ofarg, sizeof(outfile)-1); outfile[sizeof(outfile)-1]='\0'; }
    else derive_filename(path, outfile, sizeof(outfile));

    printf("         COM%d  %s:%u%s  -> %s\n", port_index+1, host, port_num, path, outfile);

    if (!ensure_modem((unsigned)port_index)) {
        fprintf(stderr, "WGET: dongle not responding to AT (SLIP escape failed?).\n");
        return 2;
    }
    /* Open the output ONCE, truncating. Resume is INTRA-invocation (across the
     * retry loop below); we deliberately do NOT resume a stale on-disk file from
     * a previous run (that would Range-request past a complete/changed file).
     * "wb+" so retries can fseek to the end and append. */
    f = fopen(outfile, "wb+");
    if (!f) { fprintf(stderr, "WGET: cannot create %s\n", outfile); return 7; }

    t_start = bios_ticks();
    {
        int attempts = 0, consec_no_progress = 0, res = 0;
        unsigned long total = 0UL, written = 0UL, n_before;
        int have_total = 0;
        const int MAX_ATTEMPTS = 6;

        for (;;) {
            fseek(f, 0L, SEEK_END);
            n_before = (unsigned long)ftell(f);   /* authoritative resume offset */
            if (attempts > 0) {
                unsigned shift = (attempts <= 3) ? (unsigned)(attempts - 1) : 2U;
                printf("         retry %d -- resume from %lu\n", attempts, n_before);
                wait_ms((unsigned)(1000UL << shift));   /* 1s / 2s / 4s backoff */
            }
            written = n_before;
            res = do_attempt((unsigned)port_index, host, port_num, path,
                             f, n_before, &total, &have_total, &written);

            if (res == ATT_OK || res == ATT_HTTP_FATAL) break;

            if (res == ATT_IGNORED_RANGE) {
                /* Server won't serve our Range (sent 200, or 206 from the wrong
                 * offset, or the size changed) -> the partial is unusable. Start
                 * over from byte 0 with a fresh truncate. */
                printf("         (range not honored -- restarting from 0)\n");
                fclose(f); f = fopen(outfile, "wb+");
                if (!f) { fprintf(stderr, "WGET: reopen failed.\n"); return 7; }
                total = 0UL; have_total = 0; consec_no_progress = 0;
                if (++attempts >= MAX_ATTEMPTS) break;
                hangup_call((unsigned)port_index);
                continue;
            }

            /* ATT_INCOMPLETE / ATT_DIAL_FAIL -> retry and resume from n_before. */
            if (written > n_before) consec_no_progress = 0;
            else if (++consec_no_progress >= 3) {
                fprintf(stderr, "WGET: no progress in 3 attempts -- giving up.\n");
                break;
            }
            if (++attempts >= MAX_ATTEMPTS) {
                fprintf(stderr, "WGET: gave up after %d attempts.\n", MAX_ATTEMPTS);
                break;
            }
            hangup_call((unsigned)port_index);
        }

        fflush(f); fclose(f);

        /* Final report + the INCOMPLETE guard (never let a partial look like success). */
        {
            unsigned long el = bios_ticks() - t_start;
            unsigned long centi, kb10;
            if (el == 0UL) el = 1UL;
            centi = (el * 10000UL + 910UL) / 1820UL;
            kb10  = (written * 1820UL / el * 10UL) / 102400UL;
            printf("         saved %lu bytes to %s in %lu.%02lus = %lu.%lu KB/s\n",
                   written, outfile, centi/100UL, centi%100UL, kb10/10UL, kb10%10UL);
            if (have_total && written < total) {
                fprintf(stderr,
                    "WGET: INCOMPLETE -- got %lu of %lu bytes (%lu short). File is PARTIAL.\n",
                    written, total, total - written);
                hangup_call((unsigned)port_index);
                return 8;
            }
        }
    }

    hangup_call((unsigned)port_index);
    return 0;
}

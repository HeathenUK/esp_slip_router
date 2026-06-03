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

int main(int argc, char **argv)
{
    int port_index = -1;
    unsigned port_num = 80;
    const char *url = NULL, *ofarg = NULL;
    int argi;
    unsigned long t_body_start, t_done;
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
    drain_rx((unsigned)port_index, 100);
    fossil_send_str((unsigned)port_index, "ATH\r"); wait_ms(400);
    drain_rx((unsigned)port_index, 100);
    fossil_send_str((unsigned)port_index, "ATNET0\r"); wait_ms(300);
    drain_rx((unsigned)port_index, 100);

    {
        char dial[200];
        snprintf(dial, sizeof(dial), "ATDT%s:%u\r", host, port_num);
        fossil_send_str((unsigned)port_index, dial);
    }
    switch (wait_dial_result((unsigned)port_index, 20000)) {
        case 1: break;   /* CONNECT */
        case -1: fprintf(stderr, "WGET: host not resolved (%s).\n", host); return 3;
        case -2: fprintf(stderr, "WGET: connection failed/refused (%s:%u).\n", host, port_num); return 3;
        case -3: fprintf(stderr, "WGET: line busy.\n"); return 3;
        case -4: fprintf(stderr, "WGET: dial error.\n"); return 3;
        default: fprintf(stderr, "WGET: no response from dongle (20s timeout).\n"); return 3;
    }
    { int c; unsigned long t = bios_ticks() + 18UL;        /* eat CR/LF after CONNECT */
      while (bios_ticks() < t) { c = fossil_recv_nowait((unsigned)port_index);
          if (c < 0) { dos_yield(); continue; } if (c == '\n') break; } }
    {
        static char req[512];
        int reqlen = snprintf(req, sizeof(req),
            "GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: tdongle-wget/1\r\nConnection: close\r\n\r\n",
            path, host);
        fossil_send_block((unsigned)port_index, (const unsigned char __far *)req, (unsigned)reqlen);
    }

    /* Status + headers. */
    {
        int n; unsigned long content_length = 0UL; int have_clen = 0;
        n = read_line((unsigned)port_index, line, sizeof(line), 15000);
        if (n < 0) { fprintf(stderr, "WGET: status timeout.\n"); return 4; }
        printf("         %s\n", line);
        if (strncmp(line, "HTTP/", 5) != 0 || strstr(line, " 200") == NULL) {
            fprintf(stderr, "WGET: non-200 status.\n"); return 5;
        }
        for (;;) {
            n = read_line((unsigned)port_index, line, sizeof(line), 15000);
            if (n < 0) { fprintf(stderr, "WGET: header timeout.\n"); return 6; }
            if (n == 0) break;
            if (!strncmp(line,"Content-Length:",15) || !strncmp(line,"content-length:",15)) {
                const char *p = line + 15; unsigned long v = 0UL;
                while (*p==' '||*p=='\t') ++p;
                while (*p>='0'&&*p<='9') { v = v*10UL + (unsigned long)(*p-'0'); ++p; }
                content_length = v; have_clen = 1;
            }
        }
        if (have_clen) printf("         Content-Length: %lu\n", content_length);
        else printf("         (no Content-Length -- saving to EOF)\n");

        f = fopen(outfile, "wb");
        if (!f) { fprintf(stderr, "WGET: cannot create %s\n", outfile); return 7; }

        /* BODY -> file. */
        {
            unsigned long written = 0UL;
            unsigned long last_byte_ticks, last_progress_ticks;
            /* Silence (no-byte) timeout. With Content-Length we KNOW more is
             * coming until written==len, so be patient (~15s) -- a slow
             * consumer (DOS disk write) or a retransmit can open a multi-second
             * gap near the tail, and bailing at 5s truncated the file. Without
             * Content-Length the silence IS how we detect EOF, so keep it short. */
            const unsigned long silence_ticks = have_clen ? 273UL : 90UL; /* 15s / 5s */
            const unsigned long max_ticks = 5460UL;     /* ~300s */
            unsigned char hold[16]; unsigned held = 0;  /* no-clen: defer tail */
            t_body_start = bios_ticks();
            last_byte_ticks = t_body_start; last_progress_ticks = t_body_start;
            for (;;) {
                unsigned r = fossil_recv_block((unsigned)port_index,
                                               (unsigned char __far *)buf, sizeof(buf));
                if (r) {
                    last_byte_ticks = bios_ticks();
                    if (have_clen) {
                        unsigned tow = r;
                        if (written + (unsigned long)tow > content_length)
                            tow = (unsigned)(content_length - written);
                        if (tow) { fwrite(buf, 1, tow, f); written += tow; }
                        if (written >= content_length) break;   /* ignore trailing NO CARRIER */
                    } else {
                        /* Hold back the last 16 bytes so the trailing
                         * "\r\nNO CARRIER" (14 B) is never written; strip at EOF. */
                        unsigned ri = 0;
                        while (ri < r) {
                            if (held == sizeof(hold)) { fputc(hold[0], f); written++;
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
                    printf("         ... %lu bytes\r", written);
                }
                if ((bios_ticks() - t_body_start) > max_ticks) {
                    printf("\n         (5 min cap)\n"); break;
                }
            }
            if (!have_clen) {
                /* hold[] is the final tail. Strip a trailing "\r\nNO CARRIER..." */
                unsigned cut = held, i;
                for (i = 0; i + 1 < held; ++i)
                    if (hold[i]=='\r' && hold[i+1]=='\n') {
                        /* candidate; check it's the NO CARRIER tail */
                        if (i+12 <= held && memcmp(hold+i, "\r\nNO CARRIER", 12)==0) { cut = i; break; }
                    }
                if (cut) { fwrite(hold, 1, cut, f); written += cut; }
            }
            fclose(f);
            printf("\n");
            t_done = bios_ticks();
            {
                unsigned long body_ticks = t_done - t_body_start;
                unsigned long centi, tenths_kbps, kb, frac;
                int incomplete = (have_clen && written < content_length);
                if (body_ticks == 0UL) body_ticks = 1UL;
                centi = (body_ticks * 10000UL + 910UL) / 1820UL;
                tenths_kbps = (written * 1820UL / body_ticks * 10UL) / 102400UL;
                kb = tenths_kbps / 10UL; frac = tenths_kbps % 10UL;
                printf("         saved %lu bytes to %s in %lu.%02lus = %lu.%lu KB/s\n",
                       written, outfile, centi/100UL, centi%100UL, kb, frac);
                if (incomplete) {
                    /* Never let a partial masquerade as success. The file on
                     * disk is the bytes we got, but the transfer FAILED. */
                    fprintf(stderr,
                        "WGET: INCOMPLETE -- got %lu of %lu bytes (%lu short). File is PARTIAL.\n",
                        written, content_length, content_length - written);
                    return 8;
                }
            }
        }
    }

    wait_ms(1200);
    fossil_send_str((unsigned)port_index, "+++"); wait_ms(1500);
    drain_rx((unsigned)port_index, 200);
    fossil_send_str((unsigned)port_index, "ATH\r"); wait_ms(300);
    return 0;
}

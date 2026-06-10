/* ftpget.c -- DOS / FOSSIL interactive FTP client for the T-Dongle S3.
 *
 * A standalone tool that WRAPS the dongle's AT$FTP engine (it does NOT touch
 * usbterm -- usbterm stays a generic terminal). It drives the dongle's FTP
 * REPL over the FOSSIL serial link and is the thing that writes downloads to a
 * DOS file:
 *
 *   1. Probe FOSSIL on COMn (CHUSB provides INT 14h); ensure the dongle is in
 *      MODEM mode (AT probe, SLIP->MODEM escape frame if needed).
 *   2. ATE0 (dongle echo off -- we local-echo, line mode), then AT$FTP=/AT$SFTP=
 *      <host> to open the interactive session; wait for CONNECT.
 *   3. Line mode: keyboard -> local line buffer (local echo); on Enter the whole
 *      line relays to the dongle's REPL -- EXCEPT `put`, which we handle locally
 *      (the local file is DOS-side). serial -> screen shows the REPL.
 *   4. On `get` the dongle frames the body as  ESC ] 5113 ; <size> ; <name> BEL
 *      + exactly <size> raw bytes; we capture those to <name> in the DOS cwd
 *      (length-prefixed, binary-clean). On `put` we send "put <remote> <size>",
 *      wait the dongle's [GO], then stream the local file up (it STOR/writes it).
 *   5. Exit when the session ends (the dongle prints NO CARRIER, e.g. after
 *      `bye`); Ctrl-C force-quits.
 *
 * Usage:  FTPGET [comN] [[sftp://|ftp://][user[:pass]@]host[:port]]
 *   Scheme picks the dial: sftp:// -> AT$SFTP (secure, over SSH), ftp:// or bare
 *   -> AT$FTP. With no host it just opens the link; type AT$FTP=/AT$SFTP= yourself.
 *   8.3 local names: in the REPL use  get remote LOCAL.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dos.h>
#include <i86.h>
#include <conio.h>

#define IN_BUF_SIZE 1024U

static unsigned char buf[IN_BUF_SIZE];

/* ---- INT 14h / FOSSIL thin wrappers (from wget.c) ---- */
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
static int fossil_recv_nowait(unsigned port)
{
    union REGS r;
    r.h.ah = 0x03U; r.x.dx = port; int86(0x14, &r, &r);
    if ((r.h.ah & 0x01U) == 0U) return -1;
    r.h.ah = 0x02U; r.x.dx = port; int86(0x14, &r, &r);
    return (int)(unsigned)r.h.al;
}
static unsigned fossil_block_read(unsigned port, unsigned char __far *b, unsigned count)
{
    struct SREGS s; union REGS r;
    segread(&s); s.es = FP_SEG(b); r.x.di = FP_OFF(b);
    r.h.ah = 0x18U; r.x.cx = count; r.x.dx = port;
    int86x(0x14, &r, &r, &s);
    return r.x.ax;
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
static int autodetect_com(void)
{
    unsigned i; unsigned short __far *bda = (unsigned short __far *)MK_FP(0x0040U, 0x0000U);
    for (i = 0; i < 4U; ++i) if (bda[i] != 0) return (int)i;
    return -1;
}
static int ensure_modem(unsigned port)
{
    static const unsigned char frame[12] =
        { 0xC0, 'M','O','D','E','=','M','O','D','E','M', 0xC0 };
    unsigned i;
    drain_rx(port, 100);
    fossil_send_str(port, "\rAT\r");
    if (wait_for(port, "OK", 700)) return 1;
    printf("FTPGET: no AT reply -- sending SLIP->MODEM escape\n");
    for (i = 0; i < sizeof(frame); ++i) fossil_send(port, frame[i]);
    wait_ms(400);
    drain_rx(port, 100);
    fossil_send_str(port, "\rAT\r");
    return wait_for(port, "OK", 1500) ? 1 : 0;
}

/* ---- console output (raw, INT 21h AH=02h so CR/LF/BS behave) ---- */
static void con_putc(unsigned char c)
{
    union REGS r; r.h.ah = 0x02U; r.h.dl = c; int86(0x21, &r, &r);
}
/* Display output with lone-LF -> CRLF. DOS AH=02h moves the cursor DOWN on LF
 * but doesn't return to column 0, so text with bare '\n' (the FTP greeting, or
 * a server's LF-only listing) staircases. Emit CR before every LF; a real CRLF
 * just gets a harmless extra CR (cursor is already at column 0). */
static void con_out(unsigned char c)
{
    if (c == 0x0AU) con_putc(0x0DU);
    con_putc(c);
}
static void con_str(const char *s) { while (*s) con_out((unsigned char)*s++); }

/* ---- OSC 5113 download capture + NO CARRIER watch ---- */
static int   ostate = 0;            /* 0=normal, 1=after-ESC, 2=in-OSC */
static char  oscbuf[200];
static unsigned osclen = 0;
static FILE *dl_file = NULL;
static long  dl_remaining = 0L;
static char  dl_name[16];
static long  dl_total = 0L;
static unsigned long dl_start = 0UL;   /* bios_ticks() when the capture began */
static unsigned nc_match = 0;        /* "NO CARRIER" matcher */
static int   g_done = 0;

static void watch_no_carrier(unsigned char c)
{
    static const char nc[] = "NO CARRIER";
    if (c == (unsigned char)nc[nc_match]) { if (nc[++nc_match] == '\0') { g_done = 1; nc_match = 0; } }
    else nc_match = (c == (unsigned char)nc[0]) ? 1u : 0u;
}

static void dispatch_osc(void)
{
    oscbuf[osclen] = '\0';
    if (osclen > 5U && !memcmp(oscbuf, "5113;", 5U)) {
        char *sp = strchr(oscbuf + 5, ';');
        long  sz = atol(oscbuf + 5);
        const char *name = (sp && sp[1]) ? sp + 1 : "download";
        if (sz > 0L) {
            strncpy(dl_name, name, sizeof dl_name - 1);
            dl_name[sizeof dl_name - 1] = '\0';
            dl_file = fopen(dl_name, "wb");   /* NULL -> consume + discard */
            dl_remaining = sz;
            dl_total = sz;
            dl_start = bios_ticks();          /* for the throughput report */
        }
    }
    /* other OSC (e.g. a title) -- ignore */
}

static void feed(unsigned char c)
{
    if (dl_remaining > 0L) {            /* capturing a download body */
        static unsigned long last_prog = 0UL;
        unsigned long now;
        if (dl_file) putc((int)c, dl_file);
        --dl_remaining;
        /* Progress line (throttled to ~1s, plus a final tick at completion).
         * Leading CR rewrites in place; trailing spaces clear any residue. */
        now = bios_ticks();
        if (dl_remaining == 0L || (now - last_prog) >= 18UL) {
            char pl[64];
            last_prog = now;
            snprintf(pl, sizeof pl, "\r  %ld / %ld bytes   ", dl_total - dl_remaining, dl_total);
            con_str(pl);
        }
        if (dl_remaining == 0L) {
            unsigned long el = bios_ticks() - dl_start;
            unsigned long centi, kb10;
            char fin[96];
            if (!el) el = 1UL;
            centi = (el * 10000UL + 910UL) / 1820UL;                 /* ticks -> centiseconds */
            kb10  = ((unsigned long)dl_total * 100UL / centi) * 10UL / 1024UL;
            if (dl_file) {
                fclose(dl_file); dl_file = NULL;
                snprintf(fin, sizeof fin,
                         "\r[ftpget: saved %s, %ld bytes in %lu.%02lu s = %lu.%lu KB/s]\n",
                         dl_name, dl_total, centi/100UL, centi%100UL, kb10/10UL, kb10%10UL);
            } else {
                snprintf(fin, sizeof fin,
                         "\r[ftpget: could not create %s -- discarded %ld bytes]\n",
                         dl_name, dl_total);
            }
            con_str(fin);
        }
        return;
    }
    switch (ostate) {
    case 0:
        if (c == 0x1BU) { ostate = 1; return; }
        watch_no_carrier(c);
        con_out(c);
        return;
    case 1:
        if (c == ']') { ostate = 2; osclen = 0; return; }
        con_putc(0x1BU); con_out(c); ostate = 0; return;   /* pass other escapes */
    case 2:
        if (c == 0x07U) { dispatch_osc(); ostate = 0; return; }   /* BEL = end OSC */
        if (osclen < sizeof oscbuf - 1) oscbuf[osclen++] = (char)c;
        return;
    }
}

/* ---- line-mode input + put intercept ---- */
static char g_line[256];
static int  g_linelen = 0;

/* Upload a local DOS file: tell the dongle "put <remote> <size>", wait its [GO],
 * then stream the body. The dongle (AT$FTP or AT$SFTP) STOR/writes it; its result
 * line comes back through the main loop's feed(). */
static void do_put(unsigned port, const char *localfile, const char *remote)
{
    FILE         *f;
    long          size, sent = 0;
    unsigned char b[512];
    size_t        nr;
    char          cmd[200];

    f = fopen(localfile, "rb");
    if (!f) { con_str("\r\nput: cannot open "); con_str(localfile); con_str("\r\n"); return; }
    fseek(f, 0L, SEEK_END); size = ftell(f); fseek(f, 0L, SEEK_SET);
    if (size < 0) { fclose(f); con_str("\r\nput: bad size\r\n"); return; }
    snprintf(cmd, sizeof cmd, "put %s %ld\r", remote, size);
    fossil_send_str(port, cmd);
    if (!wait_for(port, "[GO]", 15000)) { fclose(f); con_str("\r\nput: dongle not ready\r\n"); return; }
    while ((nr = fread(b, 1, sizeof b, f)) > 0) {
        size_t i;
        for (i = 0; i < nr; ++i) fossil_send(port, b[i]);
        sent += (long)nr;
        { char pl[48]; snprintf(pl, sizeof pl, "\r  put %ld / %ld  ", sent, size); con_str(pl); }
    }
    fclose(f);
    con_str("\r\n");   /* result line ("put ... -> ok") arrives via feed() */
}

/* A completed input line: intercept "put <local> [remote]" locally; relay all
 * else to the dongle's REPL verbatim. */
static void handle_line(unsigned port, char *line)
{
    char *p = line;
    while (*p == ' ') ++p;
    if ((p[0]=='p'||p[0]=='P') && (p[1]=='u'||p[1]=='U') && (p[2]=='t'||p[2]=='T') &&
        (p[3]==' ' || p[3]=='\0')) {
        char *a = p + 3, *local, *remote, *sp, *bs;
        while (*a == ' ') ++a;
        local = a;
        sp = strchr(a, ' ');
        if (sp) { *sp = 0; remote = sp + 1; while (*remote == ' ') ++remote; }
        else { char *fs = strrchr(local, '/'); bs = strrchr(local, '\\');
               if (bs && bs > fs) fs = bs; remote = fs ? fs + 1 : local; }
        if (!*local) { con_str("usage: put <localfile> [remote]\r\n"); return; }
        do_put(port, local, remote);
        return;
    }
    fossil_send_str(port, line);
    fossil_send(port, '\r');
}

int main(int argc, char **argv)
{
    int port_index = -1;
    const char *host = NULL;
    int argi;

    setbuf(stdout, NULL);
    printf("FTPGET: T-Dongle S3 interactive FTP (DOS / FOSSIL)\n");

    for (argi = 1; argi < argc; ++argi) {
        const char *a = argv[argi];
        if ((a[0]=='c'||a[0]=='C')&&(a[1]=='o'||a[1]=='O')&&(a[2]=='m'||a[2]=='M')&&
             a[3]>='1'&&a[3]<='4'&&a[4]=='\0') port_index = a[3]-'1';
        else if (a[0]=='-'&&(a[1]=='h'||a[1]=='H')) {
            fprintf(stderr, "usage: FTPGET [comN] [[sftp://|ftp://][user[:pass]@]host[:port]]\n"); return 0;
        } else if (!host) host = a;
    }
    if (port_index < 0) {
        port_index = autodetect_com();
        if (port_index < 0) { fprintf(stderr, "FTPGET: no COM port in BDA.\n"); return 1; }
    }
    if (!probe_fossil((unsigned)port_index)) {
        fprintf(stderr, "FTPGET: FOSSIL not detected on COM%d (load CHUSB).\n", port_index+1);
        return 1;
    }
    (void)int14(0x1E, 0x01, 0x0000, (unsigned)((3<<8)|11), (unsigned)port_index);

    if (!ensure_modem((unsigned)port_index)) {
        fprintf(stderr, "FTPGET: dongle not responding to AT.\n"); return 2;
    }
    drain_rx((unsigned)port_index, 100);
    fossil_send_str((unsigned)port_index, "ATE0\r");    /* echo OFF -- ftpget local-echoes (line mode) */
    wait_ms(150);
    drain_rx((unsigned)port_index, 100);

    if (host) {
        /* URL scheme picks the dongle dial: sftp:// -> AT$SFTP (secure, over SSH),
         * ftp:// or bare -> AT$FTP. The OSC-5113 capture below is the same either way. */
        char dial[160];
        const char *verb = "AT$FTP", *h = host;
        if      (!strncmp(h, "sftp://", 7)) { verb = "AT$SFTP"; h += 7; }
        else if (!strncmp(h, "ftp://",  6)) { h += 6; }
        snprintf(dial, sizeof dial, "%s=%s\r", verb, h);
        printf("         COM%d  %s=%s\n", port_index+1, verb, h);
        fossil_send_str((unsigned)port_index, dial);
        if (!wait_for((unsigned)port_index, "CONNECT", 20000)) {
            fprintf(stderr, "FTPGET: no CONNECT (login failed / host down?).\n");
            return 3;
        }
    } else {
        printf("         COM%d  (no host -- type AT$FTP=host or AT$SFTP=... yourself)\n", port_index+1);
    }
    printf("         --- interactive (bye to quit, Ctrl-C to force) ---\n");

    /* Relay loop: serial -> screen/capture, keyboard -> serial. */
    while (!g_done) {
        unsigned r = fossil_recv_block((unsigned)port_index,
                                       (unsigned char __far *)buf, sizeof buf);
        if (r) { unsigned i; for (i = 0; i < r; ++i) feed(buf[i]); }
        if (kbhit()) {
            int k = getch();
            if (k == 0) { (void)getch(); }            /* extended key -- ignore */
            else if (k == '\r' || k == '\n') {        /* line complete -> dispatch */
                con_out('\n');
                g_line[g_linelen] = 0;
                handle_line((unsigned)port_index, g_line);
                g_linelen = 0;
            } else if (k == 8 || k == 127) {          /* backspace */
                if (g_linelen) { --g_linelen; con_putc(8); con_putc(' '); con_putc(8); }
            } else if (k >= 0x20 && g_linelen < (int)sizeof g_line - 1) {
                g_line[g_linelen++] = (char)k;
                con_putc((unsigned char)k);           /* local echo (dongle echo is off) */
            }
        }
        if (!r) dos_yield();
    }

    if (dl_file) fclose(dl_file);
    printf("\r\nFTPGET: session closed.\n");
    return 0;
}

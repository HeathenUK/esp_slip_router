/* atq.c -- headless AT-command query via FOSSIL. Sends a one-line AT
 * command to the dongle and writes the reply to stdout (redirect via
 * DOS `>` to capture). Designed for use in batch profiling scripts
 * where USBTERM's interactive/script mode is too fragile.
 *
 * Usage:  ATQ [comN] <command...>
 *   ATQ AT$STATS                      > P-STATS.LOG
 *   ATQ COM1 AT$MODE=SLIP             > MODE-SLIP.LOG
 *   ATQ "AT$WIFI=JELLING,Pass Phrase" > WIFI.LOG
 *
 * Notes:
 * - Words after the optional comN are joined with single spaces, then a
 *   CR is appended. So `ATQ AT$WIFI=ssid,my password` becomes
 *   "AT$WIFI=ssid,my password\r" on the wire.
 * - Exit code 0 on any reply received, 1 on FOSSIL absent / timeout.
 * - Reads RX until 500 ms of silence after the last byte, with a 5 s
 *   wall-clock cap. Long enough for AT$SCAN-style multi-second replies
 *   when used interactively; bounded enough not to wedge a script.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dos.h>
#include <i86.h>

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

static unsigned long bios_ticks(void)
{
    return *(unsigned long __far *)MK_FP(0x0040U, 0x006CU);
}

static int autodetect_com(void)
{
    unsigned i;
    unsigned short __far *bda = (unsigned short __far *)MK_FP(0x0040U, 0x0000U);
    for (i = 0; i < 4U; ++i) if (bda[i] != 0) return (int)i;
    return -1;
}

/* FOSSIL AH=0Bh send-no-wait; spin until queued (with yield). */
static void send_byte(unsigned port, unsigned char ch)
{
    unsigned long tries;
    for (tries = 0UL; ; ++tries) {
        if (int14(0x0B, ch, 0, 0, port) != 0U) return;
        if ((tries & 0xFFUL) == 0xFFUL) dos_yield();
    }
}

static void send_str(unsigned port, const char *s)
{
    while (*s) send_byte(port, (unsigned char)*s++);
}

/* AH=03h line status + AH=02h byte read; non-blocking. */
static int recv_byte_nowait(unsigned port)
{
    union REGS r;
    r.h.ah = 0x03U; r.x.dx = port;
    int86(0x14, &r, &r);
    if ((r.h.ah & 0x01U) == 0U) return -1;
    r.h.ah = 0x02U; r.x.dx = port;
    int86(0x14, &r, &r);
    return (int)(unsigned)r.h.al;
}

int main(int argc, char **argv)
{
    int port = -1;
    int argi = 1;
    char cmd[512];
    unsigned cmdlen = 0;
    int c;

    setbuf(stdout, NULL);

    /* optional comN */
    if (argc > argi &&
        (argv[argi][0] == 'c' || argv[argi][0] == 'C') &&
        (argv[argi][1] == 'o' || argv[argi][1] == 'O') &&
        (argv[argi][2] == 'm' || argv[argi][2] == 'M') &&
         argv[argi][3] >= '1' && argv[argi][3] <= '4' && argv[argi][4] == '\0') {
        port = argv[argi][3] - '1';
        ++argi;
    }
    if (argc <= argi) {
        fprintf(stderr,
            "usage: ATQ [comN] <AT command>\n"
            "        ATQ AT$STATS         > stats.log\n"
            "        ATQ COM1 AT$MODE=SLIP\n");
        return 1;
    }
    if (port < 0) {
        port = autodetect_com();
        if (port < 0) { fprintf(stderr, "ATQ: no COM in BDA\n"); return 1; }
    }
    if (!probe_fossil((unsigned)port)) {
        fprintf(stderr, "ATQ: no FOSSIL on COM%d\n", port + 1);
        return 1;
    }

    /* join argv[argi..] with single spaces */
    for (; argi < argc; ++argi) {
        size_t l = strlen(argv[argi]);
        if (cmdlen + l + 2 > sizeof cmd) break;
        if (cmdlen) cmd[cmdlen++] = ' ';
        memcpy(cmd + cmdlen, argv[argi], l);
        cmdlen += l;
    }
    cmd[cmdlen] = '\0';

    /* Flush whatever line the AT engine was partway through assembling.
     * Critical after MAGICOUT in MODEM mode: those 0xC0 / "MODE=MODEM" bytes
     * sit in the AT input buffer until the next CR commits the line, so an
     * ATQ command right after a MAGICOUT would prepend that garbage to our
     * command and parse as one invalid line. A bare CR triggers an ERROR
     * (or OK) reply that we then drain along with the RX. */
    send_byte((unsigned)port, '\r');
    {
        unsigned long t = bios_ticks();
        unsigned long lastByte = t;
        for (;;) {
            c = recv_byte_nowait((unsigned)port);
            if (c >= 0) lastByte = bios_ticks();
            else        dos_yield();
            /* ~200 ms of silence after last drained byte, or 1 s cap */
            if ((bios_ticks() - lastByte) > 3UL) break;
            if ((bios_ticks() - t) > 18UL) break;
        }
    }

    /* send command + CR */
    send_str((unsigned)port, cmd);
    send_byte((unsigned)port, '\r');

    /* echo what we sent so the log is self-describing */
    fputs(cmd, stdout);
    fputc('\n', stdout);

    /* read reply: 500 ms of silence after last byte, or 5 s cap */
    {
        unsigned long t0 = bios_ticks();
        unsigned long lastByte = t0;
        const unsigned long silence_ticks = 9UL;   /* ~0.5 s @ 18.2 Hz */
        const unsigned long max_ticks     = 91UL;  /* ~5 s   */
        int got_any = 0;
        for (;;) {
            c = recv_byte_nowait((unsigned)port);
            if (c >= 0) {
                if (c == 0x0D) continue;        /* skip CR, keep LF */
                fputc(c, stdout);
                lastByte = bios_ticks();
                got_any = 1;
            } else {
                dos_yield();
                if (got_any && (bios_ticks() - lastByte) > silence_ticks) break;
                if ((bios_ticks() - t0) > max_ticks) {
                    if (!got_any) fputs("[ATQ: timeout]\n", stdout);
                    break;
                }
            }
        }
    }
    fflush(stdout);
    return 0;
}

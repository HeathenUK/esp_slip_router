/* magicout.c -- send the SLIP magic-frame escape to flip the T-Dongle S3
 * back from SLIP routing into MODEM (Hayes) mode.
 *
 * Frame on the wire: 0xC0 'M' 'O' 'D' 'E' '=' 'M' 'O' 'D' 'E' 'M' 0xC0
 * (firmware checks for "MODE=MODEM" payload inside a single SLIP frame).
 *
 * Usage: MAGICOUT [comN]    (default = autodetect from BDA)
 *
 * Self-contained: uses FOSSIL AH=01 (BIOS-compatible blocking send) which
 * any FOSSIL provides. Does NOT touch the AT engine -- works even when the
 * dongle is in pure SLIP mode and won't respond to AT commands.
 */
#include <stdio.h>
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

static int probe_fossil(unsigned dx)
{
    union REGS r;
    r.h.ah = 0x04U; r.h.al = 0xFFU;
    r.x.bx = 0x4F50U; r.x.dx = dx;
    int86(0x14, &r, &r);
    return (r.x.ax == 0x1954U);
}

static int autodetect_com(void)
{
    unsigned i;
    unsigned short __far *bda = (unsigned short __far *)MK_FP(0x0040U, 0x0000U);
    for (i = 0; i < 4U; ++i) if (bda[i] != 0) return (int)i;
    return -1;
}

/* BIOS tick counter at BDA 0040:006C, 18.2065 Hz. */
static unsigned long bios_ticks(void)
{
    return *(unsigned long __far *)MK_FP(0x0040U, 0x006CU);
}

/* Send one byte via FOSSIL AH=01 (blocking). FOSSIL AH=01 returns AH bit 7
 * set on failure -- retry. */
static void send_byte(unsigned port, unsigned char ch)
{
    int tries;
    for (tries = 0; tries < 65535; ++tries) {
        unsigned short ax = int14(0x01, ch, 0, 0, port);
        if ((ax & 0x8000U) == 0) return;
    }
}

int main(int argc, char **argv)
{
    int port_index = -1;
    static const unsigned char frame[12] = {
        0xC0, 'M', 'O', 'D', 'E', '=', 'M', 'O', 'D', 'E', 'M', 0xC0
    };
    unsigned i;

    setbuf(stdout, NULL);

    if (argc >= 2 &&
        (argv[1][0] == 'c' || argv[1][0] == 'C') &&
        (argv[1][1] == 'o' || argv[1][1] == 'O') &&
        (argv[1][2] == 'm' || argv[1][2] == 'M') &&
         argv[1][3] >= '1' && argv[1][3] <= '4' && argv[1][4] == '\0') {
        port_index = argv[1][3] - '1';
    }
    if (port_index < 0) {
        port_index = autodetect_com();
        if (port_index < 0) { printf("MAGICOUT: no COM in BDA\n"); return 1; }
    }
    if (!probe_fossil((unsigned)port_index)) {
        printf("MAGICOUT: no FOSSIL on COM%d (load CHUSB)\n", port_index + 1);
        return 1;
    }

    printf("MAGICOUT: COM%d -> SLIP escape frame\n", port_index + 1);
    for (i = 0; i < sizeof(frame); ++i) send_byte((unsigned)port_index, frame[i]);

    /* Give the dongle a few BIOS ticks to receive + process the frame.
     * Without this, an immediate ATQ that follows would race the SLIP
     * decoder and arrive while the dongle is still in SLIP. */
    {
        unsigned long t = bios_ticks();
        while ((bios_ticks() - t) < 6UL) {        /* ~330 ms */
            union REGS r;
            int86(0x28, &r, &r);                  /* DOS yield */
        }
    }

    /* Verify the dongle is responsive in MODEM mode. Bare CR -> OK or
     * ERROR; either way the AT engine is running. Silent for 1s = stuck
     * in SLIP (frame didn't take). */
    {
        int seen_at_response = 0;
        unsigned long t = bios_ticks();
        /* Drain anything pending first */
        while ((bios_ticks() - t) < 2UL) {
            union REGS r;
            r.h.ah = 0x03U; r.x.dx = (unsigned)port_index;
            int86(0x14, &r, &r);
            if (r.h.ah & 0x01U) { r.h.ah = 0x02U; int86(0x14, &r, &r); }
            else                { union REGS y; int86(0x28, &y, &y); }
        }
        /* Send CR, wait for any response byte */
        send_byte((unsigned)port_index, '\r');
        t = bios_ticks();
        while ((bios_ticks() - t) < 18UL) {       /* ~1 s */
            union REGS r;
            r.h.ah = 0x03U; r.x.dx = (unsigned)port_index;
            int86(0x14, &r, &r);
            if (r.h.ah & 0x01U) { seen_at_response = 1; break; }
            { union REGS y; int86(0x28, &y, &y); }
        }
        if (seen_at_response) {
            printf("MAGICOUT: dongle responsive in MODEM mode (OK)\n");
            return 0;
        } else {
            printf("MAGICOUT: warning -- no AT response in 1s; dongle may be stuck in SLIP\n");
            return 2;
        }
    }
}

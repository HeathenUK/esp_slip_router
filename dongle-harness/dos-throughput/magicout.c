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
    printf("MAGICOUT: sent %u bytes; dongle should be in MODEM mode now\n",
           (unsigned)sizeof(frame));
    return 0;
}

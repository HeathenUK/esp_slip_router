/* timer.c -- tiny DOS BIOS-tick timer + KB/s reporter.
 *
 * Usage:
 *   TIMER START               -- save current BIOS tick to TIMER.DAT
 *   TIMER STOP <filename>     -- read TIMER.DAT, read file size of
 *                                <filename>, print elapsed + KB/s
 *
 * Wraps anything that doesn't natively report throughput (mTCP HTGET).
 *
 *   C:\TDONGLE>TIMER START
 *   C:\TDONGLE>HTGET.EXE -o GOT.BIN http://...
 *   C:\TDONGLE>TIMER STOP GOT.BIN
 *   GOT.BIN: 1048576 bytes in 18.45s = 55.5 KB/s
 *
 * BIOS ticks are 18.2065 Hz from BDA 0040:006C. Tick counter wraps once
 * a day; we cope by treating result as unsigned subtraction.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dos.h>
#include <i86.h>

#define DATFILE "TIMER.DAT"

static unsigned long bios_ticks(void)
{
    return *(unsigned long __far *)MK_FP(0x0040U, 0x006CU);
}

static int save_start(void)
{
    FILE *f = fopen(DATFILE, "wb");
    unsigned long t;
    if (!f) { perror(DATFILE); return 1; }
    t = bios_ticks();
    if (fwrite(&t, sizeof(t), 1, f) != 1) { perror("write"); fclose(f); return 1; }
    fclose(f);
    return 0;
}

static int report_stop(const char *filename)
{
    FILE *f;
    unsigned long t0, t1, ticks, bytes;
    unsigned long centi, tenths_kbps, kb, frac;

    f = fopen(DATFILE, "rb");
    if (!f) {
        fprintf(stderr, "TIMER: no %s -- did you run TIMER START first?\n", DATFILE);
        return 1;
    }
    if (fread(&t0, sizeof(t0), 1, f) != 1) {
        fprintf(stderr, "TIMER: bad %s\n", DATFILE);
        fclose(f); return 1;
    }
    fclose(f);

    t1 = bios_ticks();
    ticks = t1 - t0;
    if (ticks == 0UL) ticks = 1UL;

    f = fopen(filename, "rb");
    if (!f) {
        fprintf(stderr, "TIMER: cannot open %s\n", filename);
        return 1;
    }
    fseek(f, 0L, SEEK_END);
    bytes = (unsigned long)ftell(f);
    fclose(f);

    /* Seconds (centi-seconds for printf): ticks / 18.2065 */
    centi = (ticks * 10000UL + 910UL) / 1820UL;
    /* KB/s tenths: bytes/sec/1024 = bytes * 1820 / ticks / 102400 */
    tenths_kbps = (bytes * 1820UL / ticks * 10UL) / 102400UL;
    kb   = tenths_kbps / 10UL;
    frac = tenths_kbps % 10UL;

    printf("%s: %lu bytes in %lu.%02lus = %lu.%lu KB/s\n",
           filename, bytes, centi / 100UL, centi % 100UL, kb, frac);
    return 0;
}

int main(int argc, char **argv)
{
    setbuf(stdout, NULL);
    if (argc >= 2 && (!strcmp(argv[1], "start") || !strcmp(argv[1], "START"))) {
        return save_start();
    }
    if (argc >= 3 && (!strcmp(argv[1], "stop")  || !strcmp(argv[1], "STOP"))) {
        return report_stop(argv[2]);
    }
    fprintf(stderr,
        "usage: TIMER START\n"
        "       TIMER STOP <filename>\n"
        "Wraps a command that writes to <filename> with start/stop and\n"
        "prints elapsed + KB/s.\n");
    return 1;
}

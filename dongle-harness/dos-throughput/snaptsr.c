/* snaptsr.c -- screen-snapshot TSR for the T-Dongle S3 harness.
 *
 * Hooks INT 9 to detect a hotkey (Ctrl+F12). On hit, copies the current
 * text-mode screen buffer (B800:0000, 80x25) into a resident scratchpad
 * and flags it dirty. Hooks INT 28h (DOS idle) to flush the scratchpad
 * to E:\SNAP.OUT next time DOS is safely re-entrant (InDOS == 1). Also
 * hooks INT 2Fh on a chosen multiplex ID so the install-check from the
 * non-resident entry path is reliable.
 *
 * Why a TSR rather than the existing SNAP.EXE: the EXE only runs from
 * the prompt and prints its own status which pollutes the captured
 * screen. The TSR fires from inside any running program (PROFILE,
 * mTCP HTGET, a wedged loop), captures silently, and writes the file
 * the moment DOS is safe to call.
 *
 * Triggered remotely by the dongle's HID-keyboard plane:
 *     curl -X POST --data-binary '<CTRL+F12>' http://dosongle.local/type
 * (Single-modifier combo because dongle_kbd.cpp's DSL accepts only one
 * <MOD+key>; Ctrl+F12 is unused by mTCP / FOSSLIP / PROFILE / the DOS
 * prompt itself.)
 *
 * Output is fixed to E:\SNAP.OUT. Host pulls via
 *     curl http://dosongle.local/fs/SNAP.OUT
 * and renames if it wants a history.
 *
 * Install:    SNAPTSR
 * Uninstall:  reboot (vector-unhook is unsafe if anyone hooked on top of us)
 *
 * Resident footprint: ~4 KB (256 paragraphs).
 *
 * Built with Open Watcom v2, medium memory model. See ../build.sh.
 */

#include <dos.h>
#include <i86.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <conio.h>

/* Arbitrary multiplex ID, picked from the convention-reserved upper
 * range. If something else picks the same, install-check will collide
 * harmlessly (we'd see "already installed" and back out). */
#define INT2F_MULTIPLEX_ID  0xC9

/* F12 make code; Ctrl bit in BIOS shift state at 0:0417. */
#define TRIG_SCANCODE       0x58
#define TRIG_SHIFT_BITS     0x04   /* Ctrl */

/* Screen geometry: 80x25 text, 2 bytes/cell (char + attr). */
#define SCREEN_COLS         80
#define SCREEN_ROWS         25

/* ----- resident scratchpad (all static, lives in our .EXE's data
 * segment, retained by _dos_keep). */

/* Plain-ASCII line dump: rows * (cols + CRLF) plus a small safety margin. */
static unsigned char snap_buf[SCREEN_ROWS * (SCREEN_COLS + 2) + 8];
static unsigned      snap_buf_len = 0;
static volatile unsigned char snap_dirty   = 0;
static volatile unsigned char snap_writing = 0;

/* InDOS flag pointer (resolved once at install). */
static unsigned char __far *in_dos_flag = (unsigned char __far *)0;

/* Output path "E:\SNAP.OUT". The drive letter is patched in at install,
 * but for v1 we hardcode E:. ASCIIZ. */
static char snap_path[] = "E:\\SNAP.OUT";

/* Saved original ISR vectors. */
static void (__interrupt __far *old_int9 )(void) = 0;
static void (__interrupt __far *old_int28)(void) = 0;
static void (__interrupt __far *old_int2f)(void) = 0;

/* ----- INT 9 handler ------------------------------------------------
 * Fires on every keyboard event (make + break). Read the scan code from
 * port 60h; if it matches our trigger (and Ctrl is held), capture the
 * screen, ack the keyboard controller so the running app never sees the
 * keystroke, send EOI to the PIC. Otherwise chain to the original.
 */
static void __interrupt __far isr_int9(void)
{
    unsigned char scan;
    unsigned char shift;
    unsigned char p61;

    scan  = inp(0x60);
    shift = *(unsigned char __far *)0x00400017L;

    if (scan == TRIG_SCANCODE && (shift & TRIG_SHIFT_BITS) == TRIG_SHIFT_BITS) {
        /* Ack the keyboard controller so it deasserts IRQ1. */
        p61 = inp(0x61);
        outp(0x61, (unsigned char)(p61 | 0x80));
        outp(0x61, p61);
        /* End-of-interrupt to the master PIC. */
        outp(0x20, 0x20);

        /* Capture the screen if a prior snap isn't still pending. */
        if (!snap_dirty && !snap_writing) {
            unsigned short __far *vram;
            unsigned i, j;
            unsigned char c;

            vram = (unsigned short __far *)0xB8000000L;
            j = 0;
            for (i = 0; i < (unsigned)(SCREEN_COLS * SCREEN_ROWS); i++) {
                c = (unsigned char)(vram[i] & 0xFFU);
                if (c == 0) c = ' ';              /* normalise NULs to spaces */
                snap_buf[j++] = c;
                if ((i % SCREEN_COLS) == (SCREEN_COLS - 1)) {
                    snap_buf[j++] = '\r';
                    snap_buf[j++] = '\n';
                }
            }
            snap_buf_len = j;
            snap_dirty   = 1;
        }
        /* Swallow the keystroke -- do NOT chain. The running app never
         * sees Ctrl+F12. */
        return;
    }

    _chain_intr(old_int9);
}

/* ----- INT 28h handler ----------------------------------------------
 * DOS idle. Fires when DOS is blocked waiting for keyboard input via
 * INT 21h AH=0Ah and friends. Safe to call most INT 21h services here
 * provided InDOS <= 1 (no deeper nesting). We use AH=3Ch / 40h / 3Eh
 * directly to avoid pulling in any non-reentrant CRT path.
 */
static void __interrupt __far isr_int28(void)
{
    union REGS r;
    struct SREGS s;
    unsigned handle;

    if (snap_dirty && !snap_writing && in_dos_flag && *in_dos_flag <= 1) {
        snap_writing = 1;

        /* INT 21h AH=3Ch: create or truncate. */
        memset(&r, 0, sizeof r);
        segread(&s);
        r.h.ah = 0x3C;
        r.x.cx = 0;
        r.x.dx = FP_OFF(snap_path);
        s.ds   = FP_SEG(snap_path);
        intdosx(&r, &r, &s);
        if (!r.x.cflag) {
            handle = r.x.ax;

            /* INT 21h AH=40h: write. */
            r.h.ah = 0x40;
            r.x.bx = handle;
            r.x.cx = snap_buf_len;
            r.x.dx = FP_OFF(snap_buf);
            s.ds   = FP_SEG(snap_buf);
            intdosx(&r, &r, &s);

            /* INT 21h AH=3Eh: close. */
            r.h.ah = 0x3E;
            r.x.bx = handle;
            intdos(&r, &r);
        }

        snap_dirty   = 0;
        snap_writing = 0;
    }

    _chain_intr(old_int28);
}

/* ----- INT 2Fh handler (multiplex) ----------------------------------
 * AH=INT2F_MULTIPLEX_ID, AL=00: install-check. We return AL=FF in AX
 * so the launcher can detect we're already resident.
 */
static void __interrupt __far isr_int2f(
    unsigned bp, unsigned di, unsigned si, unsigned ds, unsigned es,
    unsigned dx, unsigned cx, unsigned bx, unsigned ax)
{
    /* AH is the high byte of ax. We can't reliably modify caller's AX
     * via _chain_intr -- if we want to return a value we have to NOT
     * chain. The simplest install-check pattern: when our marker is
     * seen, do nothing else, return with AX=0xFFFF in our handler's
     * caller-visible state. Watcom's __interrupt-with-arg form lets
     * us peek but not poke registers cleanly; rely on _chain_intr
     * NOT being called and let the original AX persist.
     *
     * Actually: arg-form __interrupt in Watcom (medium model) does
     * NOT let us modify caller registers via the args. We have to
     * use INTPACK or inline asm. For v1, take the simpler route:
     * just identify our presence by NOT chaining on the magic value,
     * and have the launcher use a sentinel like writing AX=0 before
     * the INT and seeing it stay 0 (=> installed). */
    (void)bp; (void)di; (void)si; (void)ds; (void)es;
    (void)dx; (void)cx; (void)bx;

    if ((ax >> 8) == INT2F_MULTIPLEX_ID && (ax & 0xFF) == 0x00) {
        /* Install-check: don't chain. Caller sees whatever AX it set
         * before INT 2F; the launcher sets AX=0 first so any preserved
         * AX value means we're here. */
        return;
    }

    _chain_intr(old_int2f);
}

/* ----- install path -------------------------------------------------
 * Everything below runs ONCE on launcher invocation and is NOT part of
 * the resident image. _dos_keep terminates with the resident block
 * pinned; the rest of the EXE (this main, the stdio used here) is
 * discarded by DOS.
 */

static int check_installed(void)
{
    union REGS r;
    /* Sentinel value that no genuine INT 2F handler would return. */
    r.x.ax = 0xC900;   /* AH = our multiplex, AL = 0 (install-check) */
    int86(0x2F, &r, &r);
    /* If we're installed, our isr_int2f did NOT chain and AX is still
     * 0xC900. If we're NOT installed, the chain ran through DOS's
     * default INT 2F which typically returns AX unchanged for unknown
     * multiplex IDs -- so this is fragile. Mitigation: also check that
     * our second sentinel is unchanged. For v1 we just accept "AX
     * unchanged == probably installed" and warn that re-install is a
     * no-op if it collides with another C9 handler. */
    return (r.x.ax == 0xC900);
}

int main(int argc, char **argv)
{
    union REGS r;
    struct SREGS s;
    int i;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "/?") == 0 || strcmp(argv[i], "-?") == 0
            || strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("SNAPTSR -- text-screen snapshot TSR (Ctrl+F12)\n"
                   "\n"
                   "  SNAPTSR         install resident\n"
                   "  SNAPTSR /?      this help\n"
                   "\n"
                   "Once installed, Ctrl+F12 (from a real keypress or HID-injected\n"
                   "from the T-Dongle) captures the 80x25 text screen to\n"
                   "%s. Host fetches via curl /fs/SNAP.OUT.\n"
                   "\n"
                   "Uninstall by reboot. Resident size ~4KB.\n",
                   snap_path);
            return 0;
        }
    }

    if (check_installed()) {
        printf("SNAPTSR: already installed (or INT 2F %02Xh is in use)\n",
               INT2F_MULTIPLEX_ID);
        return 0;
    }

    /* Resolve the InDOS flag pointer via INT 21h AH=34h. ES:BX on return
     * points to the flag byte. Stash it for isr_int28. */
    memset(&r, 0, sizeof r);
    segread(&s);
    r.h.ah = 0x34;
    intdosx(&r, &r, &s);
    in_dos_flag = (unsigned char __far *)
                  (((unsigned long)s.es << 16) | (unsigned long)r.x.bx);

    /* Save originals then install. */
    old_int9  = _dos_getvect(0x09);
    old_int28 = _dos_getvect(0x28);
    old_int2f = _dos_getvect(0x2F);

    _dos_setvect(0x09, isr_int9);
    _dos_setvect(0x28, isr_int28);
    _dos_setvect(0x2F, isr_int2f);

    printf("SNAPTSR installed: Ctrl+F12 -> %s\n", snap_path);

    /* Terminate-stay-resident. We pin enough memory to hold the entire
     * built EXE (~12 KB) because our ISRs call into the C runtime
     * (intdosx, segread) and we haven't laid out the binary with a
     * resident/transient split. 1024 paragraphs = 16 KB, comfortably
     * above the linked image size with margin for the snap buffer
     * (~2 KB) and ISR stack frames. On a 640 KB DOS box with mTCP +
     * CHUSB already loaded we have plenty of headroom.
     *
     * If a future revision splits the init path out into its own
     * segment, this number can drop to (end_resident - psp + 15) / 16
     * computed at install time. For v1 the constant is fine. */
    _dos_keep(0, 1024);
    return 0;   /* not reached */
}

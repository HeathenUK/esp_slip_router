/* snap.c -- one-shot text-mode screen dump for the T-Dongle dev loop.
 *
 * Writes a plain CRLF text file containing the current visible text-mode
 * screen. Attribute/style bytes are discarded in DOS, so the agent can fetch
 * SCREEN.TXT through the dongle's HTTP /fs endpoint and read it directly.
 *
 * SNAP.EXE lives on the dongle's 8 MB FAT MSC partition (whatever drive
 * letter CHUSB assigned -- not necessarily A:); it derives its own directory
 * from argv[0] at runtime and dumps SCREEN.TXT there.
 *
 *   Usage:  SNAP                        (writes <self-dir>\SCREEN.TXT)
 *           SNAP D:\OUT.TXT             (explicit output path)
 *
 * Each text-mode cell on the IBM PC is two bytes: character byte followed by
 * an attribute byte. We write only the character byte. Printable ASCII is
 * preserved, control bytes become spaces, and common CP437 box-drawing bytes
 * are reduced to '+', '-' and '|' so the file remains 7-bit text.
 *
 * Publish atomically: write to <base>.TMP, INT 21h AH=68h commit, close,
 * rename -> <base>.TXT. Then create a 0-byte <base>.DON sentinel for manual
 * inspection. The harness should not read the file over HTTP while USB MSC
 * owns the disk. After SNAP exits, switch the dongle disk owner to DEVICE
 * (HTTP /owner/device, AT$DISK=DEVICE, or the DISK=DEVICE SLIP magic frame)
 * before fetching SCREEN.TXT.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dos.h>
#include <i86.h>
#include <io.h>

static unsigned char get_video_mode(unsigned char *cols, unsigned char *page)
{
    union REGS r;
    memset(&r, 0, sizeof r);
    r.h.ah = 0x0F;
    int86(0x10, &r, &r);
    if (cols) *cols = r.h.ah;
    if (page) *page = r.h.bh;
    return r.h.al;
}

/* Prefer the EGA/VGA BIOS data-area row count. If absent, try INT 10h
 * AX=1130h; not all BIOSes support it, so fall back to 25. */
static unsigned char get_rows(void)
{
    unsigned char __far *bda_rows = (unsigned char __far *)MK_FP(0x0040U, 0x0084U);
    union REGS r;
    if (*bda_rows > 0 && *bda_rows < 100) return *bda_rows + 1;

    memset(&r, 0, sizeof r);
    r.h.ah = 0x11; r.h.al = 0x30; r.h.bh = 0;
    int86(0x10, &r, &r);
    if (r.h.dl == 0 || r.h.dl >= 100) return 25;
    return r.h.dl + 1;
}

/* Map video mode to the text-RAM segment. Modes 0,1,2,3 = CGA colour text
 * @ 0xB800.  Mode 7 = MDA/Hercules mono text @ 0xB000. Anything else we
 * probe 0xB800 too (most VGA text-on-graphics lives there). */
static unsigned text_segment(unsigned char mode)
{
    if (mode == 7) return 0xB000;
    return 0xB800;
}

static int is_text_mode(unsigned char mode)
{
    return mode == 0 || mode == 1 || mode == 2 || mode == 3 || mode == 7;
}

/* Current display-page start offset. The BDA word is what BIOS itself uses;
 * fall back to a page-size guess if it looks unset for a nonzero page. */
static unsigned active_page_offset(unsigned char mode, unsigned char page,
                                   unsigned char cols, unsigned char rows)
{
    unsigned short __far *bda_ofs = (unsigned short __far *)MK_FP(0x0040U, 0x004EU);
    unsigned off = *bda_ofs;
    unsigned raw = (unsigned)cols * (unsigned)rows * 2U;
    unsigned page_size;

    if (off != 0 || page == 0) return off;
    if (mode == 0 || mode == 1 || raw <= 2048U) page_size = 2048U;
    else page_size = 4096U;
    return (unsigned)page * page_size;
}

static int flush_stream(FILE *f)
{
    if (fflush(f) != 0) return -1;
    if (fsync(fileno(f)) != 0) return -1;
    return 0;
}

static int make_side_path(const char *out_path, const char *ext,
                          char *dst, size_t dstsz)
{
    char *dot;

    if (strlen(out_path) + 5 >= dstsz) return -1;
    strcpy(dst, out_path);
    dot = strrchr(dst, '.');
    if (dot) *dot = 0;
    if (strlen(dst) + strlen(ext) >= dstsz) return -1;
    strcat(dst, ext);
    return 0;
}

static char text_byte(unsigned char ch)
{
    switch (ch) {
    case 0xB3: case 0xBA:
        return '|';
    case 0xC4: case 0xCD:
        return '-';
    case 0xB0: case 0xB1: case 0xB2: case 0xDB:
        return '#';
    case 0xB4: case 0xB5: case 0xB6: case 0xB7:
    case 0xB8: case 0xB9: case 0xBB: case 0xBC:
    case 0xBD: case 0xBE: case 0xBF: case 0xC0:
    case 0xC1: case 0xC2: case 0xC3: case 0xC5:
    case 0xC6: case 0xC7: case 0xC8: case 0xC9:
    case 0xCA: case 0xCB: case 0xCC: case 0xCE:
    case 0xCF: case 0xD0: case 0xD1: case 0xD2:
    case 0xD3: case 0xD4: case 0xD5: case 0xD6:
    case 0xD7: case 0xD8: case 0xD9: case 0xDA:
    case 0xDC: case 0xDD: case 0xDE: case 0xDF:
        return '+';
    default:
        if (ch >= 32 && ch < 127) return (char)ch;
        return ' ';
    }
}

static int write_screen_text(FILE *f, unsigned char __far *vram,
                             unsigned page_ofs, unsigned char cols,
                             unsigned char rows)
{
    unsigned r, c;
    char line[160];

    if (cols >= sizeof line) return -1;
    for (r = 0; r < rows; ++r) {
        int last = -1;
        for (c = 0; c < cols; ++c) {
            unsigned pos = page_ofs + ((unsigned)r * cols + c) * 2U;
            char ch = text_byte(vram[pos]);
            line[c] = ch;
            if (ch != ' ') last = (int)c;
        }
        if (last >= 0 && fwrite(line, 1, (size_t)last + 1, f) != (size_t)last + 1)
            return -1;
        if (fwrite("\r\n", 1, 2, f) != 2) return -1;
    }
    return 0;
}

/* Copy the directory portion of argv[0] (everything up to and including
 * the last \ or /) into `out`. Empty string if argv[0] is bareword.
 * DOS 3.0+ gives full paths via the PSP environment block; argv[0]
 * comes through via Watcom's crt0. */
static void self_dir(const char *argv0, char *out, size_t outsz)
{
    const char *p, *slash;
    size_t n;
    out[0] = 0;
    if (!argv0) return;
    slash = NULL;
    for (p = argv0; *p; ++p) {
        if (*p == '\\' || *p == '/') slash = p;
    }
    if (!slash) return;
    n = (size_t)(slash - argv0) + 1;
    if (n >= outsz) n = outsz - 1;
    memcpy(out, argv0, n);
    out[n] = 0;
}

int main(int argc, char **argv)
{
    char out_path[80], tmp_path[80], done_path[80];
    unsigned char mode, cols, rows, page;
    unsigned seg, page_ofs;
    unsigned long total;
    FILE *f;
    unsigned char __far *vram;

    setbuf(stdout, NULL);

    if (argc >= 2) {
        /* Explicit output path. Strip extension to derive .TMP / .DON. */
        if (strlen(argv[1]) >= sizeof out_path - 5) {
            fprintf(stderr, "SNAP: output path too long\n");
            return 1;
        }
        strcpy(out_path, argv[1]);
    } else {
        /* Default: <argv[0]-dir>SCREEN.TXT -- so SNAP always dumps
         * onto the same disk it's loaded from. */
        char dir[80];
        self_dir(argv[0], dir, sizeof dir);
        if (strlen(dir) + 11 >= sizeof out_path) {
            fprintf(stderr, "SNAP: self path too long\n");
            return 1;
        }
        sprintf(out_path, "%sSCREEN.TXT", dir);
    }

    /* Derive .TMP and .DON siblings by replacing the extension. */
    if (make_side_path(out_path, ".TMP", tmp_path, sizeof tmp_path) != 0 ||
        make_side_path(out_path, ".DON", done_path, sizeof done_path) != 0) {
        fprintf(stderr, "SNAP: output path too long\n");
        return 1;
    }

    mode = get_video_mode(&cols, &page);
    rows = get_rows();
    if (cols == 0) cols = 80;
    if (!is_text_mode(mode)) {
        fprintf(stderr, "SNAP: video mode %02Xh is not a supported text mode\n", mode);
        return 1;
    }
    if (cols > 132 || rows > 60) {
        fprintf(stderr, "SNAP: screen size %ux%u is not supported\n", cols, rows);
        return 1;
    }
    seg = text_segment(mode);
    page_ofs = active_page_offset(mode, page, cols, rows);

    /* Wipe any prior .DON first so the dongle never sees a stale ready. */
    remove(done_path);

    f = fopen(tmp_path, "wb");
    if (!f) {
        fprintf(stderr, "SNAP: cannot open %s\n", tmp_path);
        return 1;
    }

    vram = (unsigned char __far *)MK_FP(seg, 0);
    total = (unsigned long)cols * (unsigned long)rows;
    if (write_screen_text(f, vram, page_ofs, cols, rows) != 0) goto io_err;

    if (flush_stream(f) != 0) goto io_err;
    if (fclose(f) != 0) {
        fprintf(stderr, "SNAP: close error\n");
        return 3;
    }

    /* Atomic publish: replace target with the freshly written tmp. */
    remove(out_path);
    if (rename(tmp_path, out_path) != 0) {
        fprintf(stderr, "SNAP: rename %s -> %s failed\n", tmp_path, out_path);
        return 2;
    }

    /* Sentinel: 0-byte file for ad-hoc/manual readiness checks. */
    f = fopen(done_path, "wb");
    if (!f) {
        fprintf(stderr, "SNAP: cannot open %s\n", done_path);
        return 3;
    }
    if (flush_stream(f) != 0 || fclose(f) != 0) {
        fprintf(stderr, "SNAP: cannot commit %s\n", done_path);
        return 3;
    }

    printf("SNAP: %s  mode=%02Xh page=%u  %ux%u  %lu cells\n",
           out_path, mode, page, cols, rows, total);
    return 0;

io_err:
    fclose(f);
    fprintf(stderr, "SNAP: write error\n");
    return 3;
}

/* THROUGHPUT.EXE  -  DOS-side SLIP throughput benchmark for T-Dongle S3.
 *
 * Measures CHUSB -> FOSSIL -> DOS-app -> dongle USB-CDC -> SLIP-decode ->
 * lwIP-forward bandwidth from the DOS side. Mirrors slip-throughput.py
 * but uses FOSSIL INT 14h instead of pyserial.
 *
 * Flow:
 *   1. Probe FOSSIL (INT 14h AH=04h). Abort if absent.
 *   2. Switch dongle to SLIP via "AT\rAT$MODE=SLIP\r" (modem path, before
 *      SLIP framing takes over). Wait ~700ms.
 *   3. Pre-build one SLIP-framed IPv4/UDP packet with 1024B pattern payload.
 *      src=192.168.240.2 (SLIP_PEER), dst configurable, dport=5611.
 *   4. Capture BIOS ticks (INT 1Ah AH=00 / 0040:006C).
 *   5. Loop for N seconds firing the same frame via FOSSIL AH=0Bh. On each
 *      iteration, bump the IP ident field in-place and patch the IP header
 *      checksum incrementally (RFC 1624) so the router doesn't dedupe.
 *   6. Capture end ticks, print "sent: P pkts, B bytes in S.SSs = KB.K/s".
 *   7. Send 0xC0 'M' 'O' 'D' 'E' '=' 'M' 'O' 'D' 'E' 'M' 0xC0 (raw, no SLIP
 *      payload escaping needed -- none of those bytes are 0xC0/0xDB) to
 *      flip the dongle back to MODEM mode.
 *
 * Build (Open Watcom v2, medium model -- matches usbterm.c):
 *   see build.sh.
 *
 * Usage:
 *   THROUGHPUT [comN] [dst_ip] [seconds]
 *     comN     COM1..COM4 (default: autodetect from BDA placeholder, same as
 *              usbterm)
 *     dst_ip   destination IPv4 for the synthetic UDP packets. Default
 *              "192.168.240.99" -- a sentinel SLIP-side address that the
 *              dongle will try to ARP/forward and drop, isolating the
 *              dongle-side decode+forward bottleneck from real WiFi.
 *     seconds  test duration. Default 5.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dos.h>
#include <i86.h>

#define SLIP_END     0xC0U
#define SLIP_ESC     0xDBU
#define SLIP_ESC_END 0xDCU
#define SLIP_ESC_ESC 0xDDU

#define PAYLOAD_LEN  1024U
#define IP_HDR_LEN   20U
#define UDP_HDR_LEN  8U
#define IP_TOTAL_LEN (IP_HDR_LEN + UDP_HDR_LEN + PAYLOAD_LEN)   /* 1052 */

/* SLIP worst case: every byte escaped + start/end markers.
 * In practice payload pattern 0..255 hits 0xC0 four times and 0xDB four
 * times per 1KB, plus the IP/UDP headers have a few escapes. Reserve
 * generously. */
#define SLIP_BUF_MAX (IP_TOTAL_LEN * 2U + 2U)

static unsigned char ip_pkt[IP_TOTAL_LEN];    /* raw IP/UDP frame */
static unsigned char slip_buf[SLIP_BUF_MAX];  /* SLIP-encoded copy */
static unsigned int  slip_len;                /* actual encoded length */
static unsigned int  ident_offset_in_slip;    /* where the 16-bit IP ident
                                                  landed in slip_buf, in
                                                  SLIP-decoded byte index --
                                                  see encode_slip(). */
static unsigned int  ident_byte0_pos;         /* slip_buf[] index of ident hi */
static unsigned int  ident_byte1_pos;         /* slip_buf[] index of ident lo */
static unsigned int  csum_byte0_pos;          /* slip_buf[] index of csum hi */
static unsigned int  csum_byte1_pos;          /* slip_buf[] index of csum lo */

/* ---- INT 14h thin wrapper, identical to usbterm.c ---- */
static unsigned short int14(unsigned char ah, unsigned char al,
                            unsigned bx, unsigned cx, unsigned dx)
{
    union REGS r;
    r.h.ah = ah;
    r.h.al = al;
    r.x.bx = bx;
    r.x.cx = cx;
    r.x.dx = dx;
    int86(0x14, &r, &r);
    return r.x.ax;
}

/* FOSSIL probe: returns 1 if present, BL = highest supported function. */
static int probe_fossil(unsigned dx)
{
    union REGS r;
    r.h.ah = 0x04U;
    r.h.al = 0xFFU;
    r.x.bx = 0x4F50U;
    r.x.dx = dx;
    int86(0x14, &r, &r);
    return (r.x.ax == 0x1954U) ? 1 : 0;
}

/* Autodetect CHUSB port via BDA placeholder, same trick as usbterm. */
static int autodetect_com(void)
{
    unsigned short __far *bda = (unsigned short __far *)MK_FP(0x0040U, 0x0000U);
    static const unsigned short placeholder[] = {
        0x03F8U, 0x02F8U, 0x03E8U, 0x02E8U
    };
    int i;
    for (i = 0; i < 4; ++i) {
        if (bda[i] == placeholder[i]) return i;
    }
    return -1;
}

/* BIOS tick counter -- 18.2 Hz (65536 / 3600 = ~18.2065). Read the dword
 * at 0040:006C. */
static unsigned long bios_ticks(void)
{
    return *(unsigned long __far *)MK_FP(0x0040U, 0x006CU);
}

/* DOS yield: INT 28h tells DOSBox (and TSRs / DOS itself) it's OK to
 * pre-empt. Sprinkle into spin loops so BNU's transmit task gets cycles. */
static void dos_yield(void)
{
    union REGS r;
    int86(0x28, &r, &r);
}

/* FOSSIL AH=0Bh send-no-wait. Returns non-zero if char queued. Retries
 * forever -- the harness has a wall-clock alarm to kill DOSBox if BNU
 * truly wedges. Yields to DOS every 256 spins so BNU can drain TX. */
static int fossil_send(unsigned port, unsigned char ch)
{
    unsigned long tries;
    for (tries = 0UL; ; ++tries) {
        if (int14(0x0B, ch, 0, 0, port) != 0U) return 1;
        if ((tries & 0xFFUL) == 0xFFUL) dos_yield();
    }
}

/* FOSSIL AH=19h "block-write" -- ES:DI = buffer, CX = count. Returns
 * the number of bytes actually queued. Real bulk write; one INT 14h per
 * packet instead of ~1KB. Far pointer needed because Watcom medium model
 * defaults to near (DS). */
static unsigned fossil_send_block(unsigned port,
                                  const unsigned char __far *buf, unsigned count)
{
    struct SREGS s;
    union REGS r;
    segread(&s);
    s.es = FP_SEG(buf);
    r.x.di = FP_OFF(buf);
    r.h.ah = 0x19U;
    r.x.cx = count;
    r.x.dx = port;
    int86x(0x14, &r, &r, &s);
    return r.x.ax;
}

/* Send a buffer of bytes the slow but reliable way: per-byte AH=0Bh
 * with yield. Used for the AT prompt / escape (short, latency tolerant). */
static int fossil_send_buf(unsigned port, const unsigned char *buf, unsigned len)
{
    unsigned i;
    for (i = 0; i < len; ++i) {
        if (!fossil_send(port, buf[i])) return 0;
    }
    return 1;
}

/* Crude millisecond-ish busy-wait using BIOS ticks (~55ms per tick).
 * Rounds up: wait(700) sleeps for at least 700ms. */
static void wait_ms(unsigned ms)
{
    unsigned long target_ticks = ((unsigned long)ms * 182UL + 9999UL) / 10000UL;
    unsigned long t0 = bios_ticks();
    if (target_ticks < 1UL) target_ticks = 1UL;
    while ((bios_ticks() - t0) < target_ticks) { /* spin */ }
}

/* ---- IP / UDP construction ---- */

/* RFC 1071 ones-complement sum over a byte buffer (16-bit big-endian
 * pairs). Returns the final 16-bit complemented checksum. */
static unsigned int ip_checksum(const unsigned char *buf, unsigned len)
{
    unsigned long s = 0UL;
    unsigned i;
    for (i = 0; i + 1U < len; i += 2U) {
        s += ((unsigned long)buf[i] << 8) | (unsigned long)buf[i + 1U];
    }
    if (i < len) {
        /* odd tail byte counts as high byte of the next word */
        s += (unsigned long)buf[i] << 8;
    }
    while (s >> 16) s = (s & 0xFFFFUL) + (s >> 16);
    return (unsigned int)((~s) & 0xFFFFUL);
}

/* Parse "a.b.c.d" into 4 octets. Returns 1 on success. */
static int parse_ip(const char *s, unsigned char out[4])
{
    int i;
    unsigned v;
    for (i = 0; i < 4; ++i) {
        v = 0;
        if (*s < '0' || *s > '9') return 0;
        while (*s >= '0' && *s <= '9') {
            v = v * 10U + (unsigned)(*s - '0');
            if (v > 255U) return 0;
            ++s;
        }
        out[i] = (unsigned char)v;
        if (i < 3) {
            if (*s != '.') return 0;
            ++s;
        }
    }
    return (*s == '\0') ? 1 : 0;
}

/* Build the IP+UDP packet (no SLIP framing yet) into ip_pkt[]. */
static void build_ip_packet(const unsigned char src[4],
                            const unsigned char dst[4],
                            unsigned short sport, unsigned short dport)
{
    unsigned short udp_len = (unsigned short)(UDP_HDR_LEN + PAYLOAD_LEN);
    unsigned short total = (unsigned short)IP_TOTAL_LEN;
    unsigned int i;
    unsigned int csum;

    /* Pattern payload: 0..255 repeating. Matches what slip-throughput.py
     * emits (`bytes(range(256)) * 4`). */
    for (i = 0; i < PAYLOAD_LEN; ++i) {
        ip_pkt[IP_HDR_LEN + UDP_HDR_LEN + i] = (unsigned char)(i & 0xFFU);
    }

    /* IP header */
    ip_pkt[0]  = 0x45;                              /* v4, IHL=5 */
    ip_pkt[1]  = 0x00;                              /* TOS */
    ip_pkt[2]  = (unsigned char)(total >> 8);       /* total length hi */
    ip_pkt[3]  = (unsigned char)(total & 0xFFU);    /*  ...           lo */
    ip_pkt[4]  = 0x40;                              /* ident hi (0x4000) */
    ip_pkt[5]  = 0x00;                              /* ident lo */
    ip_pkt[6]  = 0x00;                              /* flags+frag hi */
    ip_pkt[7]  = 0x00;                              /* frag lo */
    ip_pkt[8]  = 0x40;                              /* TTL = 64 */
    ip_pkt[9]  = 17;                                /* proto UDP */
    ip_pkt[10] = 0x00;                              /* checksum placeholder */
    ip_pkt[11] = 0x00;
    ip_pkt[12] = src[0]; ip_pkt[13] = src[1];
    ip_pkt[14] = src[2]; ip_pkt[15] = src[3];
    ip_pkt[16] = dst[0]; ip_pkt[17] = dst[1];
    ip_pkt[18] = dst[2]; ip_pkt[19] = dst[3];

    /* UDP header (checksum placeholder) */
    ip_pkt[20] = (unsigned char)(sport >> 8);
    ip_pkt[21] = (unsigned char)(sport & 0xFFU);
    ip_pkt[22] = (unsigned char)(dport >> 8);
    ip_pkt[23] = (unsigned char)(dport & 0xFFU);
    ip_pkt[24] = (unsigned char)(udp_len >> 8);
    ip_pkt[25] = (unsigned char)(udp_len & 0xFFU);
    ip_pkt[26] = 0x00;
    ip_pkt[27] = 0x00;

    /* UDP checksum (over pseudo-header + UDP header + payload). Pseudo-
     * header = src(4) + dst(4) + zero(1) + proto(1) + udp_len(2). We can
     * compute it by summing src/dst from the IP hdr we just wrote, plus
     * proto + udp_len, plus the UDP segment.
     *
     * The simple way: build a tiny scratch buffer. Keeps the code obvious. */
    {
        static unsigned char scratch[12 + UDP_HDR_LEN + PAYLOAD_LEN];
        scratch[0] = src[0]; scratch[1] = src[1];
        scratch[2] = src[2]; scratch[3] = src[3];
        scratch[4] = dst[0]; scratch[5] = dst[1];
        scratch[6] = dst[2]; scratch[7] = dst[3];
        scratch[8] = 0;
        scratch[9] = 17;
        scratch[10] = (unsigned char)(udp_len >> 8);
        scratch[11] = (unsigned char)(udp_len & 0xFFU);
        memcpy(scratch + 12, ip_pkt + 20, UDP_HDR_LEN + PAYLOAD_LEN);
        csum = ip_checksum(scratch, sizeof(scratch));
        if (csum == 0U) csum = 0xFFFFU;   /* UDP: 0 means "no checksum" */
        ip_pkt[26] = (unsigned char)(csum >> 8);
        ip_pkt[27] = (unsigned char)(csum & 0xFFU);
    }

    /* IP header checksum (over the 20-byte IP header only). */
    csum = ip_checksum(ip_pkt, IP_HDR_LEN);
    ip_pkt[10] = (unsigned char)(csum >> 8);
    ip_pkt[11] = (unsigned char)(csum & 0xFFU);
}

/* SLIP-encode ip_pkt[] into slip_buf[]; record offsets of the bytes we
 * will mutate per-iteration (ident hi/lo, IP checksum hi/lo). Caller
 * guarantees those bytes are NOT 0xC0 or 0xDB at build time (ident
 * starts at 0x40 and only the low half rotates 0..255, neither match;
 * but the IP checksum can land on 0xC0/0xDB. If it does, we re-roll the
 * initial ident at build time until we land on a layout that's safe.) */
static int encode_slip(void)
{
    unsigned int in_i, out_i;
    slip_buf[0] = (unsigned char)SLIP_END;
    out_i = 1;
    /* Reset markers; will be set as we walk the payload. */
    ident_byte0_pos = 0; ident_byte1_pos = 0;
    csum_byte0_pos  = 0; csum_byte1_pos  = 0;
    (void)ident_offset_in_slip;   /* unused, kept for clarity */
    for (in_i = 0; in_i < IP_TOTAL_LEN; ++in_i) {
        unsigned char b = ip_pkt[in_i];
        unsigned char encoded_byte_pos = 0;
        if (b == (unsigned char)SLIP_END) {
            if (in_i == 4 || in_i == 5 || in_i == 10 || in_i == 11) {
                /* mutating byte happened to be 0xC0 -- bail; caller will
                 * adjust ident and rebuild. */
                return 0;
            }
            slip_buf[out_i++] = (unsigned char)SLIP_ESC;
            slip_buf[out_i++] = (unsigned char)SLIP_ESC_END;
        } else if (b == (unsigned char)SLIP_ESC) {
            if (in_i == 4 || in_i == 5 || in_i == 10 || in_i == 11) {
                return 0;
            }
            slip_buf[out_i++] = (unsigned char)SLIP_ESC;
            slip_buf[out_i++] = (unsigned char)SLIP_ESC_ESC;
        } else {
            encoded_byte_pos = 1;
            slip_buf[out_i++] = b;
        }
        if (encoded_byte_pos) {
            if (in_i == 4)  ident_byte0_pos = out_i - 1;
            if (in_i == 5)  ident_byte1_pos = out_i - 1;
            if (in_i == 10) csum_byte0_pos  = out_i - 1;
            if (in_i == 11) csum_byte1_pos  = out_i - 1;
        }
    }
    slip_buf[out_i++] = (unsigned char)SLIP_END;
    slip_len = out_i;
    return 1;
}

/* Bump the IP ident by 1 and patch IP header checksum incrementally
 * (RFC 1624). Old ident word m, new m'. HC' = ~(~HC + (~m & 0xFFFF) + m').
 * All in 16-bit ones-complement.
 *
 * IMPORTANT: any of the four mutated bytes (ident hi/lo, csum hi/lo) can
 * land on 0xC0 or 0xDB. We chose ident/csum positions in encode_slip()
 * that are NOT escaped, so the slip_buf has only one byte per position.
 * If a bump produces 0xC0/0xDB in any of those positions, the dongle's
 * SLIP decoder would see a frame boundary or escape mid-packet. So we
 * keep bumping until we land on a layout that's SLIP-safe. */
static void bump_ident_and_fix_csum(void)
{
    unsigned int new_ident;
    unsigned int new_hc;
    int spin;

    for (spin = 0; spin < 256; ++spin) {
        unsigned int old_ident = ((unsigned int)slip_buf[ident_byte0_pos] << 8)
                                | slip_buf[ident_byte1_pos];
        unsigned int old_hc    = ((unsigned int)slip_buf[csum_byte0_pos] << 8)
                                | slip_buf[csum_byte1_pos];
        unsigned long s;
        unsigned char b0, b1, c0, c1;

        new_ident = (old_ident + 1U) & 0xFFFFU;
        /* RFC 1624: HC' = ~(~HC + ~m + m')
         * Equivalent integer form: ~HC and ~m are 16-bit complements. */
        s = (unsigned long)((~old_hc) & 0xFFFFU)
          + (unsigned long)((~old_ident) & 0xFFFFU)
          + (unsigned long)new_ident;
        while (s >> 16) s = (s & 0xFFFFUL) + (s >> 16);
        new_hc = (unsigned int)((~s) & 0xFFFFUL);

        b0 = (unsigned char)(new_ident >> 8);
        b1 = (unsigned char)(new_ident & 0xFFU);
        c0 = (unsigned char)(new_hc >> 8);
        c1 = (unsigned char)(new_hc & 0xFFU);

        /* Commit unconditionally -- the next iter reads from slip_buf and
         * keeps walking. */
        slip_buf[ident_byte0_pos] = b0;
        slip_buf[ident_byte1_pos] = b1;
        slip_buf[csum_byte0_pos]  = c0;
        slip_buf[csum_byte1_pos]  = c1;

        /* If any mutated byte would corrupt the SLIP frame, keep spinning
         * (we just committed it so the next loop iter sees the bumped
         * state; eventually we land on a safe combination). */
        if (b0 != SLIP_END && b0 != SLIP_ESC &&
            b1 != SLIP_END && b1 != SLIP_ESC &&
            c0 != SLIP_END && c0 != SLIP_ESC &&
            c1 != SLIP_END && c1 != SLIP_ESC) {
            return;
        }
    }
    /* Hit the safety cap -- shouldn't ever happen given there are only
     * 4*2 forbidden values across 2 16-bit fields. Caller will send
     * whatever's in slip_buf; that packet might get truncated by the
     * dongle's SLIP decoder, but the benchmark continues. */
}

int main(int argc, char **argv)
{
    int port_index = -1;
    unsigned long duration_s = 5UL;
    unsigned char src_ip[4] = { 192, 168, 240, 2 };  /* SLIP_PEER */
    unsigned char dst_ip[4] = { 192, 168, 240, 99 }; /* sentinel default */
    const char *dst_arg = "192.168.240.99";
    int i;
    int argi;

    /* Banner */
    printf("THROUGHPUT: T-Dongle S3 SLIP benchmark (DOS / FOSSIL)\n");

    /* Parse argv. Each arg is either "comN", an IPv4 dotted quad, or a
     * small integer (seconds). Position-insensitive. */
    for (argi = 1; argi < argc; ++argi) {
        const char *a = argv[argi];
        if ((a[0] == 'c' || a[0] == 'C') &&
            (a[1] == 'o' || a[1] == 'O') &&
            (a[2] == 'm' || a[2] == 'M') &&
             a[3] >= '1' && a[3] <= '4' && a[4] == '\0') {
            port_index = a[3] - '1';
            continue;
        }
        /* IP? must contain '.' */
        {
            int has_dot = 0;
            const char *p = a;
            while (*p) { if (*p == '.') { has_dot = 1; break; } ++p; }
            if (has_dot) {
                if (!parse_ip(a, dst_ip)) {
                    fprintf(stderr, "THROUGHPUT: bad IP %s\n", a);
                    return 1;
                }
                dst_arg = a;
                continue;
            }
        }
        /* integer = seconds */
        {
            char *end;
            long v = strtol(a, &end, 10);
            if (*end == '\0' && v > 0L && v <= 600L) {
                duration_s = (unsigned long)v;
                continue;
            }
        }
        fprintf(stderr, "THROUGHPUT: ignoring arg %s\n", a);
    }

    if (port_index < 0) {
        port_index = autodetect_com();
        if (port_index < 0) {
            fprintf(stderr,
                "THROUGHPUT: no CHUSB COM port found in BDA.\n"
                "            Load CHUSB first or pass comN explicitly.\n");
            return 1;
        }
    }

    if (!probe_fossil((unsigned)port_index)) {
        fprintf(stderr,
            "THROUGHPUT: FOSSIL not detected on COM%d.\n"
            "            This benchmark requires CHUSB's FOSSIL on real DOS.\n",
            port_index + 1);
        return 1;
    }

    printf("            port=COM%d  dst=%s  duration=%lus\n",
           port_index + 1, dst_arg, duration_s);

    /* FOSSIL line init: 115200,N,8,1 -- matches usbterm.c. AH=1Eh extended
     * line control. AL=1 no break, BH=parity none, BL=stop1, CH=8 data,
     * CL=11 (=115200). The bps is largely meaningless over USB-CDC but
     * keep it consistent with usbterm. */
    (void)int14(0x1E, 0x01, 0x0000, (unsigned)((3 << 8) | 11),
                (unsigned)port_index);

    /* Step 1: switch dongle to SLIP mode. We're still in MODEM mode, so
     * AT lines work. No SLIP framing yet. */
    printf("            switching dongle to SLIP mode...\n");
    fflush(stdout);
    if (!fossil_send_buf((unsigned)port_index,
                         (const unsigned char *)"AT\rAT$MODE=SLIP\r", 16)) {
        fprintf(stderr, "THROUGHPUT: failed to send mode-switch.\n");
        return 1;
    }
    wait_ms(700);

    /* Step 2: build IP+UDP packet, then SLIP-encode. If the IP-checksum
     * byte landed on 0xC0/0xDB (encode_slip returns 0), bump initial ident
     * and retry -- up to a few times. */
    for (i = 0; i < 16; ++i) {
        build_ip_packet(src_ip, dst_ip, 49152U, 5611U);
        if (encode_slip()) break;
        /* Adjust ident base so the checksum lands on a non-SLIP byte. */
        ip_pkt[4] = (unsigned char)(0x40 + i + 1);
    }
    if (i == 16) {
        fprintf(stderr, "THROUGHPUT: couldn't find SLIP-safe initial ident.\n");
        /* try to fall back to modem mode before quitting */
        (void)fossil_send_buf((unsigned)port_index,
                              (const unsigned char *)"\xC0MODE=MODEM\xC0", 12);
        return 1;
    }

    printf("            IP packet %u B, SLIP-encoded %u B\n",
           (unsigned)IP_TOTAL_LEN, slip_len);
    fflush(stdout);

    /* Step 3: tight TX loop. Re-checksum at the head of each iter so the
     * first packet also carries a fresh ident. */
    {
        unsigned long t0, t1, ticks;
        unsigned long pkts = 0UL;
        unsigned long bytes = 0UL;
        unsigned long deadline_ticks;
        unsigned int  k;

        /* duration_s -> ticks: 18.2065 ticks/sec. Round to nearest. */
        deadline_ticks = (duration_s * 1820UL + 50UL) / 100UL;

        t0 = bios_ticks();
        while ((bios_ticks() - t0) < deadline_ticks) {
            bump_ident_and_fix_csum();
            /* AH=19h block-write: one INT 14h per packet. The kernel
             * accepts as much as fits in BNU's TX ring; we retry the
             * rest with a yield so BNU can drain. */
            {
                unsigned remaining = slip_len;
                const unsigned char __far *p = (const unsigned char __far *)slip_buf;
                unsigned w;
                while (remaining) {
                    w = fossil_send_block((unsigned)port_index, p, remaining);
                    if (w == 0U) { dos_yield(); continue; }
                    p += w; remaining -= w;
                }
            }
            (void)k;
            ++pkts;
            bytes += slip_len;
        }
    done:
        t1 = bios_ticks();
        ticks = t1 - t0;
        if (ticks == 0UL) ticks = 1UL;

        /* Print: centi-seconds = ticks * 10000 / 1820 (BIOS tick ≈ 0.055s). */
        {
            unsigned long centi = (ticks * 10000UL + 910UL) / 1820UL;
            unsigned long kb_per_s;        /* KB/s integer part */
            unsigned long kb_per_s_frac;   /* tenths */
            /* bytes/sec = bytes * 1820 / (ticks * 100). KB/s = bytes/sec/1024.
             * Combine: KB/s = bytes * 1820 / ticks / 102400.
             * For the tenths, multiply numerator by 10 first. */
            unsigned long tenths_kbps = (bytes * 1820UL / ticks * 10UL) / 102400UL;
            kb_per_s      = tenths_kbps / 10UL;
            kb_per_s_frac = tenths_kbps % 10UL;
            printf("sent: %lu pkts, %lu bytes in %lu.%02lus = %lu.%lu KB/s\n",
                   pkts, bytes, centi / 100UL, centi % 100UL,
                   kb_per_s, kb_per_s_frac);
        }
    }

    /* Step 4: escape back to MODEM mode so the next run / the user's
     * interactive session works without power-cycle. Raw 0xC0 frame --
     * no escaping needed: MODE=MODEM contains no 0xC0/0xDB bytes. */
    fflush(stdout);
    wait_ms(150);
    {
        static const unsigned char esc_frame[12] = {
            SLIP_END, 'M','O','D','E','=','M','O','D','E','M', SLIP_END
        };
        if (!fossil_send_buf((unsigned)port_index, esc_frame, 12)) {
            fprintf(stderr,
                "THROUGHPUT: warning -- couldn't send MODE=MODEM escape.\n"
                "            Dongle may still be in SLIP mode; power-cycle\n"
                "            or re-send manually.\n");
        }
    }
    wait_ms(600);

    return 0;
}

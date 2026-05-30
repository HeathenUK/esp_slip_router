// profile.cpp -- T-Dongle S3 SLIP/Hayes profiling, ONE BINARY.
//
// Replaces the P-ALL.BAT orchestration + ATQ + MAGICOUT + HTTPGET +
// THRPUT + MTCPGET dance. Runs every phase in sequence, writes a
// single PROFILE.LOG, prints summary to screen.
//
// What it does, in order:
//   PHASE 0  baseline dongle queries (AT, AT$WIFI?, AT$NETIF, AT$STATS)
//   PHASE 1  raw FOSSIL SLIP UDP throughput against sentinel IP
//   PHASE 2  Hayes-mode HTTP download (dongle terminates TCP)
//   PHASE 3  mTCP HTTP download via FOSSLIP+SLIP (LAN target)
//   PHASE 4  same as 3 against internet target
//
// Modes are managed internally: switches between MODEM and SLIP via
// AT$MODE / magic-frame escape; spawns and unloads FOSSLIP cleanly
// between MTCP phases.
//
// Usage:
//   PROFILE [LAN_URL] [INET_URL]
//   defaults: http://192.168.1.226:8765/1M.bin
//             http://ipv4.download.thinkbroadband.com/1MB.zip

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <dos.h>
#include <i86.h>
#include <process.h>

#include "types.h"
#include "utils.h"
#include "packet.h"
#include "arp.h"
#include "ip.h"
#include "tcp.h"
#include "tcpsockm.h"
#include "dns.h"
#include "timer.h"

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------

#define LOG_FILENAME       "PROFILE.LOG"
#define DEFAULT_LAN_URL    "http://192.168.1.226:8765/1M.bin"
#define DEFAULT_INET_URL   "http://ipv4.download.thinkbroadband.com/1MB.zip"
#define SLIP_TEST_SECONDS  5
#define SLIP_SENTINEL_IP   "192.168.240.99"

#define IN_BUF_SIZE   8192
#define LINE_BUF_SIZE 1024
#define REQ_BUF_SIZE  1024
#define TCP_RECV_BUF  16384     // Match stock HTGET so our numbers reflect
                                // what a stock-mTCP user would actually see.
                                // PACKET_BUFFERS also kept at stock 20.
#define CONNECT_TO_MS 10000
#define IDLE_TO_MS    30000

// ---------------------------------------------------------------------------
// Module state (globals -- DOS medium/large model has small default stack)
// ---------------------------------------------------------------------------

static int      g_port = -1;
static FILE    *g_log  = NULL;
static unsigned char  g_inbuf[IN_BUF_SIZE];
static unsigned char  g_slipbuf[3200];        // SLIP-encoded UDP packet
static unsigned       g_sliplen;
static unsigned       g_ident_lo_pos, g_ident_hi_pos;
static unsigned       g_csum_lo_pos,  g_csum_hi_pos;
static char           g_host[128];
static char           g_path[256];
static unsigned short g_port_num;
static char           g_outfile[80];
static char           g_line[LINE_BUF_SIZE];
static char           g_req[REQ_BUF_SIZE];

// mTCP socket teardown handled by Utils::endStack -- don't free socket
// ourselves (heap corruption otherwise).
static TcpSocket *g_sock = NULL;

static volatile unsigned char g_userBreak = 0;
static void __interrupt __far ctrlBreakHandler( ) { g_userBreak = 1; }

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

static void log_open( void )
{
    g_log = fopen( LOG_FILENAME, "w" );
    if ( !g_log ) {
        fprintf( stderr, "PROFILE: cannot open %s for writing\n", LOG_FILENAME );
        exit( 1 );
    }
    setbuf( g_log, NULL );      // unbuffered so we don't lose data on crash
}

/* DOS 3.3+ Commit File (INT 21h AH=68h): flush this handle's data and
 * update its directory entry NOW without closing. setbuf(NULL) only kills
 * the C-runtime buffer -- DOS still defers the FAT/dirent write until
 * close, so a hard crash leaves the file at 0 bytes on disk. */
static void log_commit( void )
{
    union REGS r;
    if ( !g_log ) return;
    r.h.ah = 0x68;
    r.x.bx = (unsigned)fileno( g_log );
    int86( 0x21, &r, &r );
}

static void log_print( const char *fmt, ... )
{
    va_list ap;
    va_start( ap, fmt );
    vfprintf( stdout, fmt, ap );
    va_end( ap );
    va_start( ap, fmt );
    if ( g_log ) vfprintf( g_log, fmt, ap );
    va_end( ap );
    log_commit();
}

static void log_section( const char *name )
{
    log_print( "\n===== %s =====\n", name );
}

// ---------------------------------------------------------------------------
// BIOS / DOS helpers
// ---------------------------------------------------------------------------

static unsigned long bios_ticks( void )
{
    return *(unsigned long __far *)MK_FP( 0x0040, 0x006C );
}

static void dos_yield( void )
{
    union REGS r;
    int86( 0x28, &r, &r );
}

static void wait_ms( unsigned ms )
{
    unsigned long target = ( (unsigned long)ms * 182UL + 9999UL ) / 10000UL;
    unsigned long t0 = bios_ticks();
    if ( target < 1 ) target = 1;
    while ( ( bios_ticks() - t0 ) < target ) dos_yield();
}

static int autodetect_com( void )
{
    unsigned i;
    unsigned short __far *bda = (unsigned short __far *)MK_FP( 0x0040, 0x0000 );
    for ( i = 0; i < 4; ++i ) if ( bda[i] != 0 ) return (int)i;
    return -1;
}

// ---------------------------------------------------------------------------
// FOSSIL (INT 14h) -- for direct dongle access between mTCP phases
// ---------------------------------------------------------------------------

static unsigned short int14( unsigned char ah, unsigned char al,
                             unsigned bx, unsigned cx, unsigned dx )
{
    union REGS r;
    r.h.ah = ah; r.h.al = al;
    r.x.bx = bx; r.x.cx = cx; r.x.dx = dx;
    int86( 0x14, &r, &r );
    return r.x.ax;
}

static int g_has_block_read = 0;
static int probe_fossil( unsigned dx )
{
    union REGS r;
    r.h.ah = 0x04; r.h.al = 0xFF;
    r.x.bx = 0x4F50; r.x.dx = dx;
    int86( 0x14, &r, &r );
    if ( r.x.ax != 0x1954 ) return 0;
    g_has_block_read = ( r.h.bl >= 0x18 );    // FOSSIL advertises AH=18h block-read
    return 1;
}

// FOSSIL AH=18h non-blocking block-read. ES:DI=buf, CX=count. Returns
// bytes actually copied. CHUSB supports this; without it we do per-byte
// AH=03+02 which is 2 INT 14h per byte (~100x slower for bulk RX).
static unsigned fos_recv_block_ah18( unsigned port,
                                     unsigned char __far *buf, unsigned count )
{
    struct SREGS s; union REGS r;
    segread( &s );
    s.es = FP_SEG( buf );
    r.x.di = FP_OFF( buf );
    r.h.ah = 0x18;
    r.x.cx = count;
    r.x.dx = port;
    int86x( 0x14, &r, &r, &s );
    return r.x.ax;
}

static void fos_send_byte( unsigned port, unsigned char ch )
{
    unsigned long tries;
    for ( tries = 0; ; ++tries ) {
        if ( int14( 0x0B, ch, 0, 0, port ) != 0 ) return;
        if ( ( tries & 0xFF ) == 0xFF ) dos_yield();
    }
}

static void fos_send_str( unsigned port, const char *s )
{
    while ( *s ) fos_send_byte( port, (unsigned char)*s++ );
}

static unsigned fos_send_block( unsigned port,
                                const unsigned char __far *buf, unsigned count )
{
    struct SREGS s; union REGS r;
    segread( &s );
    s.es = FP_SEG( buf );
    r.x.di = FP_OFF( buf );
    r.h.ah = 0x19;
    r.x.cx = count;
    r.x.dx = port;
    int86x( 0x14, &r, &r, &s );
    return r.x.ax;
}

static int fos_recv_nowait( unsigned port )
{
    union REGS r;
    r.h.ah = 0x03; r.x.dx = port;
    int86( 0x14, &r, &r );
    if ( ( r.h.ah & 0x01 ) == 0 ) return -1;
    r.h.ah = 0x02; r.x.dx = port;
    int86( 0x14, &r, &r );
    return (int)(unsigned)r.h.al;
}

static void fos_drain( unsigned port, unsigned ms )
{
    unsigned long target = ( (unsigned long)ms * 182UL + 9999UL ) / 10000UL;
    unsigned long t0 = bios_ticks();
    while ( ( bios_ticks() - t0 ) < target ) {
        int c = fos_recv_nowait( port );
        if ( c < 0 ) dos_yield();
    }
}

// ---------------------------------------------------------------------------
// AT command query (log-friendly): send CR-flush, then command, capture
// reply (skip CR, keep printable + LF) into log. Returns 0 on response,
// -1 on timeout.
// ---------------------------------------------------------------------------

static int at_query( const char *cmd )
{
    unsigned long t, lastByte;
    int got_any;
    int c;

    // Flush whatever was in the AT line buffer (e.g. MAGICOUT garbage)
    fos_send_byte( g_port, '\r' );
    t = bios_ticks(); lastByte = t; got_any = 0;
    while ( ( bios_ticks() - t ) < 36 ) {     // 2 s overall cap
        c = fos_recv_nowait( g_port );
        if ( c >= 0 ) { got_any = 1; lastByte = bios_ticks(); }
        else            dos_yield();
        if ( got_any && ( bios_ticks() - lastByte ) > 9 ) break;
        if ( !got_any && ( bios_ticks() - t ) > 18 ) break;
    }

    fos_send_str( g_port, cmd );
    fos_send_byte( g_port, '\r' );
    log_print( "> %s\n", cmd );

    // Capture reply with 500 ms silence terminator, 5 s overall cap
    t = bios_ticks(); lastByte = t; got_any = 0;
    while ( ( bios_ticks() - t ) < 91 ) {
        c = fos_recv_nowait( g_port );
        if ( c >= 0 ) {
            if ( c == 0x0D ) continue;
            if ( g_log ) fputc( c, g_log );
            fputc( c, stdout );
            lastByte = bios_ticks();
            got_any = 1;
        } else {
            dos_yield();
            if ( got_any && ( bios_ticks() - lastByte ) > 9 ) break;
        }
    }
    if ( g_log ) fputc( '\n', g_log );
    if ( !got_any ) {
        log_print( "[no response]\n" );
        return -1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// SLIP magic-frame escape: brings dongle back to MODEM from SLIP without
// touching the AT engine.
// ---------------------------------------------------------------------------

static int magic_escape( void )
{
    static const unsigned char frame[12] = {
        0xC0, 'M', 'O', 'D', 'E', '=', 'M', 'O', 'D', 'E', 'M', 0xC0
    };
    unsigned i;
    for ( i = 0; i < 12; ++i ) fos_send_byte( g_port, frame[i] );
    wait_ms( 330 );

    // Verify: bare CR should get OK or ERROR; either means AT engine running
    fos_drain( g_port, 100 );
    fos_send_byte( g_port, '\r' );
    {
        unsigned long t = bios_ticks();
        while ( ( bios_ticks() - t ) < 18 ) {
            int c = fos_recv_nowait( g_port );
            if ( c >= 0 ) { fos_drain( g_port, 200 ); return 0; }
            dos_yield();
        }
    }
    return -1;
}

// ---------------------------------------------------------------------------
// IP+UDP+SLIP packet builder for the throughput test
// ---------------------------------------------------------------------------

static unsigned short csum16( const unsigned char *d, unsigned len )
{
    unsigned long s = 0;
    unsigned i;
    for ( i = 0; i + 1 < len; i += 2 ) s += ( (unsigned long)d[i] << 8 ) | d[i+1];
    if ( len & 1 ) s += (unsigned long)d[len-1] << 8;
    while ( s >> 16 ) s = ( s & 0xFFFF ) + ( s >> 16 );
    return (unsigned short)( ( ~s ) & 0xFFFF );
}

static int parse_ipv4( const char *s, unsigned char out[4] )
{
    int i;
    unsigned v;
    for ( i = 0; i < 4; ++i ) {
        v = 0;
        if ( *s < '0' || *s > '9' ) return 0;
        while ( *s >= '0' && *s <= '9' ) { v = v * 10 + (unsigned)( *s - '0' ); ++s; }
        if ( v > 255 ) return 0;
        out[i] = (unsigned char)v;
        if ( i < 3 ) { if ( *s != '.' ) return 0; ++s; }
    }
    return ( *s == 0 );
}

// Builds a 1052-byte IP+UDP packet (20 IP + 8 UDP + 1024 data) and SLIP-
// encodes it into g_slipbuf. Records offsets of the ident bytes so we can
// bump them per-packet without re-encoding from scratch.
static void build_packet( const char *src_ip, const char *dst_ip, unsigned short dport )
{
    unsigned char src[4], dst[4];
    unsigned char ip[1052];
    unsigned i, j;

    if ( !parse_ipv4( src_ip, src ) || !parse_ipv4( dst_ip, dst ) ) {
        log_print( "PROFILE: bad IP in build_packet\n" ); exit( 1 );
    }

    // Payload: 0..255 repeating, 1024 bytes
    for ( i = 0; i < 1024; ++i ) ip[20 + 8 + i] = (unsigned char)( i & 0xFF );

    // UDP header (sport=49152, dport=dport, len=1032, csum filled)
    ip[20] = 49152 >> 8; ip[21] = 49152 & 0xFF;
    ip[22] = dport >> 8; ip[23] = dport & 0xFF;
    ip[24] = 1032 >> 8;  ip[25] = 1032 & 0xFF;
    ip[26] = 0; ip[27] = 0;
    {
        // UDP checksum with pseudo-header
        unsigned char pseudo[12];
        memcpy( pseudo + 0, src, 4 );
        memcpy( pseudo + 4, dst, 4 );
        pseudo[8] = 0; pseudo[9] = 17;
        pseudo[10] = 1032 >> 8; pseudo[11] = 1032 & 0xFF;
        {
            unsigned long s = 0;
            for ( i = 0; i + 1 < 12; i += 2 ) s += ( (unsigned long)pseudo[i] << 8 ) | pseudo[i+1];
            for ( i = 20; i + 1 < 1052; i += 2 ) s += ( (unsigned long)ip[i] << 8 ) | ip[i+1];
            while ( s >> 16 ) s = ( s & 0xFFFF ) + ( s >> 16 );
            {
                unsigned short c = (unsigned short)( ( ~s ) & 0xFFFF );
                if ( c == 0 ) c = 0xFFFF;
                ip[26] = c >> 8; ip[27] = c & 0xFF;
            }
        }
    }

    // IP header
    ip[0] = 0x45; ip[1] = 0;
    ip[2] = 1052 >> 8; ip[3] = 1052 & 0xFF;
    ip[4] = 0xCA; ip[5] = 0xFE;          // ident -- we bump this per packet
    ip[6] = 0; ip[7] = 0;
    ip[8] = 64; ip[9] = 17;
    ip[10] = 0; ip[11] = 0;              // checksum filled below
    memcpy( ip + 12, src, 4 );
    memcpy( ip + 16, dst, 4 );
    {
        unsigned short c = csum16( ip, 20 );
        ip[10] = c >> 8; ip[11] = c & 0xFF;
    }

    // SLIP-encode into g_slipbuf, record byte offsets of mutable fields
    j = 0;
    g_slipbuf[j++] = 0xC0;
    for ( i = 0; i < 1052; ++i ) {
        if ( i == 4 ) g_ident_hi_pos = j;
        if ( i == 5 ) g_ident_lo_pos = j;
        if ( i == 10 ) g_csum_hi_pos = j;
        if ( i == 11 ) g_csum_lo_pos = j;
        unsigned char b = ip[i];
        if      ( b == 0xC0 ) { g_slipbuf[j++] = 0xDB; g_slipbuf[j++] = 0xDC; }
        else if ( b == 0xDB ) { g_slipbuf[j++] = 0xDB; g_slipbuf[j++] = 0xDD; }
        else                  { g_slipbuf[j++] = b; }
    }
    g_slipbuf[j++] = 0xC0;
    g_sliplen = j;
}

// Bump IP ident; recompute the IP checksum (header bytes 10/11) via the
// quick incremental adjustment so we don't have to redo the whole header.
static void bump_packet( void )
{
    unsigned old_ident, new_ident;
    unsigned old_csum,  new_csum;
    old_ident = ( (unsigned)g_slipbuf[g_ident_hi_pos] << 8 ) | g_slipbuf[g_ident_lo_pos];
    new_ident = old_ident + 1;
    g_slipbuf[g_ident_hi_pos] = (unsigned char)( new_ident >> 8 );
    g_slipbuf[g_ident_lo_pos] = (unsigned char)( new_ident & 0xFF );

    // RFC1624: ~(~m + ~(~m') + m) = m' for incremental checksum update
    old_csum = ( (unsigned)g_slipbuf[g_csum_hi_pos] << 8 ) | g_slipbuf[g_csum_lo_pos];
    {
        unsigned long s = (unsigned long)( old_csum ^ 0xFFFF )
                        + (unsigned long)( old_ident ^ 0xFFFF )
                        + new_ident;
        while ( s >> 16 ) s = ( s & 0xFFFF ) + ( s >> 16 );
        new_csum = (unsigned short)( s ^ 0xFFFF );
    }
    g_slipbuf[g_csum_hi_pos] = (unsigned char)( new_csum >> 8 );
    g_slipbuf[g_csum_lo_pos] = (unsigned char)( new_csum & 0xFF );
}

// ---------------------------------------------------------------------------
// URL parsing (shared by Hayes + mTCP paths)
// ---------------------------------------------------------------------------

static int parse_url( const char *url )
{
    const char *p = url;
    unsigned i;
    if ( !strncmp( p, "http://", 7 ) ) p += 7;
    i = 0;
    while ( *p && *p != ':' && *p != '/' ) {
        if ( i + 1 >= sizeof g_host ) return -1;
        g_host[i++] = *p++;
    }
    g_host[i] = 0;
    if ( !i ) return -1;
    if ( *p == ':' ) {
        unsigned short v = 0;
        ++p;
        while ( *p >= '0' && *p <= '9' ) { v = (unsigned short)( v * 10 + ( *p - '0' ) ); ++p; }
        g_port_num = v ? v : 80;
    } else {
        g_port_num = 80;
    }
    if ( *p == '/' ) {
        i = 0;
        while ( *p ) {
            if ( i + 1 >= sizeof g_path ) return -1;
            g_path[i++] = *p++;
        }
        g_path[i] = 0;
    } else {
        g_path[0] = '/'; g_path[1] = 0;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// PHASE 0 -- baseline dongle queries
// ---------------------------------------------------------------------------

static void phase_0_baseline( void )
{
    log_section( "PHASE 0 -- baseline" );
    magic_escape();                     // belt + braces: ensure MODEM
    at_query( "AT" );
    at_query( "AT$WIFI?" );
    at_query( "AT$NETIF" );
    at_query( "AT$STATS" );
    at_query( "AT$STATS=0" );
}

// ---------------------------------------------------------------------------
// PHASE 1 -- raw FOSSIL SLIP UDP throughput
// ---------------------------------------------------------------------------

static void phase_1_slip_udp( void )
{
    unsigned long pkts = 0, bytes = 0;
    unsigned long t0, t1, ticks;
    unsigned long deadline_ticks;

    log_section( "PHASE 1 -- raw FOSSIL SLIP UDP throughput" );
    magic_escape();
    log_print( "switching dongle to SLIP...\n" );
    fos_send_str( g_port, "AT\rAT$MODE=SLIP\r" );
    wait_ms( 700 );
    fos_drain( g_port, 200 );

    build_packet( "192.168.240.2", SLIP_SENTINEL_IP, 5611 );
    log_print( "IP packet 1052B  SLIP-encoded %uB\n", g_sliplen );
    log_print( "running %us...\n", (unsigned)SLIP_TEST_SECONDS );

    deadline_ticks = ( (unsigned long)SLIP_TEST_SECONDS * 1820UL + 50UL ) / 100UL;
    t0 = bios_ticks();
    while ( ( bios_ticks() - t0 ) < deadline_ticks ) {
        unsigned remaining = g_sliplen;
        const unsigned char __far *p = (const unsigned char __far *)g_slipbuf;
        bump_packet();
        while ( remaining ) {
            unsigned w = fos_send_block( g_port, p, remaining );
            if ( !w ) { dos_yield(); continue; }
            p += w; remaining -= w;
        }
        ++pkts;
        bytes += g_sliplen;
        if ( g_userBreak ) break;
    }
    t1 = bios_ticks();
    ticks = t1 - t0;
    if ( !ticks ) ticks = 1;
    {
        unsigned long centi = ( ticks * 10000UL + 910UL ) / 1820UL;
        unsigned long tkb10 = ( bytes * 1820UL / ticks * 10UL ) / 102400UL;
        log_print( "PHASE 1 RESULT: %lu pkts, %lu bytes in %lu.%02lus = %lu.%lu KB/s\n",
                   pkts, bytes,
                   centi / 100, centi % 100,
                   tkb10 / 10, tkb10 % 10 );
    }
    magic_escape();
}

// ---------------------------------------------------------------------------
// PHASE 2 -- Hayes HTTP download (dongle terminates TCP via ATDT)
// ---------------------------------------------------------------------------

static int hayes_wait_for( const char *needle, unsigned ms )
{
    unsigned long target = ( (unsigned long)ms * 182UL + 9999UL ) / 10000UL;
    unsigned long t0 = bios_ticks();
    const char *m = needle;
    int c;
    while ( ( bios_ticks() - t0 ) < target ) {
        c = fos_recv_nowait( g_port );
        if ( c < 0 ) { dos_yield(); continue; }
        if ( (unsigned char)c == (unsigned char)*m ) {
            if ( !*++m ) return 0;
        } else {
            m = ( (unsigned char)c == (unsigned char)*needle ) ? needle + 1 : needle;
        }
    }
    return -1;
}

static int hayes_read_line( char *out, unsigned cap, unsigned ms )
{
    unsigned long target = ( (unsigned long)ms * 182UL + 9999UL ) / 10000UL;
    unsigned long t0 = bios_ticks();
    unsigned n = 0;
    int c;
    while ( ( bios_ticks() - t0 ) < target ) {
        c = fos_recv_nowait( g_port );
        if ( c < 0 ) { dos_yield(); continue; }
        if ( c == '\r' ) continue;
        if ( c == '\n' ) { out[n] = 0; return (int)n; }
        if ( n + 1 < cap ) out[n++] = (char)c;
    }
    return -1;
}

static void phase_2_hayes( const char *url )
{
    unsigned long got = 0;
    unsigned long content_length = 0;
    int have_clen = 0;
    int n;
    char dial[200];
    int reqlen;
    unsigned long t_dial, t_connected, t_body, t_done;

    log_section( "PHASE 2 -- Hayes HTTP download" );
    if ( parse_url( url ) ) { log_print( "bad URL: %s\n", url ); return; }
    log_print( "URL=%s\nhost=%s:%u path=%s\n", url, g_host, g_port_num, g_path );

    magic_escape();
    fos_send_byte( g_port, '\r' ); wait_ms( 200 ); fos_drain( g_port, 100 );
    fos_send_str( g_port, "ATH\r" );   wait_ms( 400 ); fos_drain( g_port, 100 );
    fos_send_str( g_port, "ATNET0\r" ); wait_ms( 300 ); fos_drain( g_port, 100 );

    snprintf( dial, sizeof dial, "ATDT%s:%u\r", g_host, g_port_num );
    log_print( "%s", dial );
    t_dial = bios_ticks();
    fos_send_str( g_port, dial );

    if ( hayes_wait_for( "CONNECT", 20000 ) ) {
        log_print( "PHASE 2 FAIL: no CONNECT (timeout)\n" );
        return;
    }
    t_connected = bios_ticks();

    // Consume up to LF after CONNECT
    {
        unsigned long t = bios_ticks();
        int c;
        while ( ( bios_ticks() - t ) < 18 ) {
            c = fos_recv_nowait( g_port );
            if ( c < 0 ) { dos_yield(); continue; }
            if ( c == '\n' ) break;
        }
    }

    reqlen = snprintf( g_req, sizeof g_req,
        "GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: tdongle-profile/1\r\n"
        "Connection: close\r\n\r\n", g_path, g_host );
    {
        unsigned i;
        for ( i = 0; i < (unsigned)reqlen; ++i ) fos_send_byte( g_port, (unsigned char)g_req[i] );
    }

    n = hayes_read_line( g_line, sizeof g_line, 15000 );
    if ( n < 0 ) { log_print( "PHASE 2 FAIL: status timeout\n" ); return; }
    log_print( "%s\n", g_line );
    if ( strncmp( g_line, "HTTP/", 5 ) || ( !strstr( g_line, " 200 " ) && !strstr( g_line, "200" ) ) ) {
        log_print( "PHASE 2 FAIL: non-200\n" ); return;
    }
    for (;;) {
        n = hayes_read_line( g_line, sizeof g_line, 15000 );
        if ( n < 0 ) { log_print( "PHASE 2 FAIL: header timeout\n" ); return; }
        if ( n == 0 ) break;
        if ( ( g_line[0] == 'C' || g_line[0] == 'c' ) && strlen( g_line ) > 16 ) {
            const char *p = g_line;
            int match = 1;
            const char *want = "Content-Length:";
            unsigned i;
            for ( i = 0; want[i]; ++i ) {
                if ( tolower( (unsigned char)g_line[i] ) != tolower( (unsigned char)want[i] ) )
                    { match = 0; break; }
            }
            if ( match ) {
                unsigned long v = 0;
                p = g_line + 15;
                while ( *p == ' ' || *p == '\t' ) ++p;
                while ( *p >= '0' && *p <= '9' ) { v = v * 10 + (unsigned long)( *p - '0' ); ++p; }
                content_length = v; have_clen = 1;
            }
        }
    }
    if ( have_clen ) log_print( "Content-Length: %lu\n", content_length );

    t_body = bios_ticks();
    {
        unsigned long last_byte = t_body;
        unsigned long last_tick = t_body;
        unsigned long last_bytes = 0;
        const unsigned long silence = 90;     // 5 s of silence => end
        for (;;) {
            unsigned r;
            if ( g_has_block_read ) {
                r = fos_recv_block_ah18( g_port,
                    (unsigned char __far *)g_inbuf, IN_BUF_SIZE );
            } else {
                int c;
                r = 0;
                while ( ( c = fos_recv_nowait( g_port ) ) >= 0 && r < IN_BUF_SIZE ) {
                    g_inbuf[r++] = (unsigned char)c;
                }
            }
            if ( r ) {
                got += r;
                last_byte = bios_ticks();
                if ( have_clen && got >= content_length ) break;
            } else {
                if ( ( bios_ticks() - last_byte ) > silence ) break;
                dos_yield();
            }
            if ( ( bios_ticks() - last_tick ) >= 18 ) {
                unsigned long db = got - last_bytes;
                unsigned long dt = bios_ticks() - last_tick;
                unsigned long kbps10;
                if ( !dt ) dt = 1;
                // bytes/sec = db * 18.2 / dt; KB/s = /1024; *10 for tenths
                kbps10 = db * 10UL * 18UL / dt / 1024UL;
                printf( "  ... %lu bytes (%lu.%lu KB/s)        \r",
                        got, kbps10 / 10, kbps10 % 10 );
                last_tick = bios_ticks();
                last_bytes = got;
            }
            if ( g_userBreak ) break;
        }
    }
    t_done = bios_ticks();
    printf( "\n" );

    {
        unsigned long body_ticks = t_done - t_body;
        if ( !body_ticks ) body_ticks = 1;
        {
            unsigned long centi  = ( body_ticks * 10000UL + 910UL ) / 1820UL;
            unsigned long centi2 = ( ( t_done - t_dial ) * 10000UL + 910UL ) / 1820UL;
            unsigned long tkb10  = ( got * 1820UL / body_ticks * 10UL ) / 102400UL;
            log_print( "PHASE 2 RESULT: %lu body bytes in %lu.%02lus = %lu.%lu KB/s (total %lu.%02lus)\n",
                       got, centi/100, centi%100, tkb10/10, tkb10%10,
                       centi2/100, centi2%100 );
        }
    }

    // Cleanup: +++ guard, ATH
    wait_ms( 1200 );
    fos_send_str( g_port, "+++" );
    wait_ms( 1500 );
    fos_drain( g_port, 200 );
    fos_send_str( g_port, "ATH\r" );
    wait_ms( 300 );
    fos_drain( g_port, 100 );
}

// ---------------------------------------------------------------------------
// mTCP plumbing
// ---------------------------------------------------------------------------

static inline void mtcp_pump( void )
{
    PACKET_PROCESS_SINGLE;
    Arp::driveArp();
    Tcp::drivePackets();
    Dns::drivePendingQuery();
}
// NOTE: the previous version invoked INT 0x1C from here to defeat
// FOSSLIP's 18.2 Hz RX cap. That broke mTCP -- calling FOSSLIP's
// timer-context handler from app context re-entered its upcall path
// while mTCP was already inside its own packet processing, corrupting
// the RX queue. Connect attempts then timed out. The clean fix lives
// in FOSSLIP itself (poll RX inside send_pkt / expose a custom AH).

static int mtcp_resolve( const char *name, IpAddr_t out )
{
    int8_t rc = Dns::resolve( name, out, 1 );
    if ( rc < 0 ) return -1;
    {
        unsigned long deadline = bios_ticks()
            + (unsigned long)( CONNECT_TO_MS * TIMER_TICKS_PER_SEC / 1000UL );
        while ( rc == 1 ) {
            if ( g_userBreak ) return -1;
            if ( bios_ticks() > deadline ) return -1;
            mtcp_pump();
            rc = Dns::resolve( name, out, 0 );
            if ( rc == 0 ) return 0;
            if ( rc <  0 ) return -1;
        }
    }
    return rc == 0 ? 0 : -1;
}

static int mtcp_connect( IpAddr_t host, unsigned short port )
{
    unsigned long deadline;
    g_sock = TcpSocketMgr::getSocket();
    if ( !g_sock ) return -1;
    if ( g_sock->setRecvBuffer( TCP_RECV_BUF ) ) return -1;
    {
        unsigned short lp = (unsigned short)( 49152 + ( bios_ticks() & 0x3FFF ) );
        if ( g_sock->connectNonBlocking( lp, host, port ) ) return -1;
    }
    deadline = bios_ticks() + (unsigned long)( CONNECT_TO_MS * TIMER_TICKS_PER_SEC / 1000UL );
    while ( !g_sock->isConnectComplete() ) {
        if ( g_userBreak )           return -1;
        if ( g_sock->isClosed() )    return -1;
        if ( bios_ticks() > deadline ) return -1;
        mtcp_pump();
    }
    return 0;
}

static int mtcp_send_all( const unsigned char *buf, unsigned len )
{
    unsigned sent = 0;
    unsigned long deadline = bios_ticks() + (unsigned long)( IDLE_TO_MS * TIMER_TICKS_PER_SEC / 1000UL );
    while ( sent < len ) {
        int16_t rc;
        if ( g_userBreak )         return -1;
        if ( g_sock->isClosed() )  return -1;
        if ( bios_ticks() > deadline ) return -1;
        mtcp_pump();
        rc = g_sock->send( (uint8_t *)( buf + sent ), (uint16_t)( len - sent ) );
        if ( rc > 0 ) sent += rc;
    }
    return 0;
}

// Refill g_inbuf from socket. Returns bytes added, 0 on remote-close, -1 on err
static int mtcp_refill( unsigned *inLen )
{
    unsigned long deadline = bios_ticks() + (unsigned long)( IDLE_TO_MS * TIMER_TICKS_PER_SEC / 1000UL );
    for (;;) {
        int16_t rc;
        if ( g_userBreak ) return -1;
        mtcp_pump();
        rc = g_sock->recv( g_inbuf, IN_BUF_SIZE );
        if ( rc > 0 ) { *inLen = rc; return rc; }
        if ( rc < 0 ) return -1;
        if ( g_sock->isRemoteClosed() ) return 0;
        if ( bios_ticks() > deadline ) return -1;
    }
}

// Helper: read one line from mTCP socket (uses g_inbuf as a temporary)
static int mtcp_read_line( char *out, unsigned cap )
{
    static unsigned char buf[1024];
    static unsigned blen = 0, bstart = 0;
    unsigned n = 0;
    for (;;) {
        while ( bstart >= blen ) {
            int16_t rc;
            if ( g_userBreak ) return -1;
            mtcp_pump();
            rc = g_sock->recv( buf, sizeof buf );
            if ( rc > 0 ) { blen = rc; bstart = 0; break; }
            if ( rc < 0 ) return -1;
            if ( g_sock->isRemoteClosed() ) return -2;
        }
        {
            unsigned char c = buf[bstart++];
            if ( c == '\r' ) continue;
            if ( c == '\n' ) { out[n] = 0; return (int)n; }
            if ( n + 1 < cap ) out[n++] = (char)c;
        }
    }
}

// ---------------------------------------------------------------------------
// Common mTCP HTTP download (used by phases 3 + 4). Spawns FOSSLIP itself.
// ---------------------------------------------------------------------------

// Spawn FOSSLIP + bring up mTCP. Returns 0 on success.
static int mtcp_session_setup( void )
{
    char *fosslip_argv[5];
    int rc;

    magic_escape();
    fos_send_str( g_port, "AT\rAT$MODE=SLIP\r" );
    wait_ms( 700 );
    fos_drain( g_port, 200 );

    fosslip_argv[0] = (char *)"FOSSLIP.EXE";
    fosslip_argv[1] = (char *)"0x60";
    fosslip_argv[2] = (char *)"1";
    fosslip_argv[3] = (char *)"115200";
    fosslip_argv[4] = NULL;
    rc = spawnvp( P_WAIT, "FOSSLIP.EXE", fosslip_argv );
    if ( rc < 0 ) { log_print( "FOSSLIP spawn failed (rc=%d)\n", rc ); return -1; }
    log_print( "FOSSLIP loaded\n" );

    if ( Utils::parseEnv() ) { log_print( "MTCPCFG parse failed\n" ); return -1; }
    if ( Utils::initStack( 1, TCP_SOCKET_RING_SIZE, ctrlBreakHandler, ctrlBreakHandler ) ) {
        log_print( "mTCP initStack failed\n" );
        return -1;
    }
    return 0;
}

static void mtcp_session_teardown( void )
{
    Utils::endStack();
    g_sock = NULL;
    {
        union REGS r;
        r.h.ah = 0x05;
        int86( 0x60, &r, &r );
        log_print( "FOSSLIP unloaded\n" );
    }
    magic_escape();
    log_print( "back in MODEM mode\n" );
}

// Do one HTTP download. Assumes mTCP + FOSSLIP already up via
// mtcp_session_setup(). Does NOT teardown -- caller does that.
// mtcp_read_line uses static state; reset between calls.
static unsigned char g_mread_buf[1024];
static unsigned      g_mread_len = 0;
static unsigned      g_mread_pos = 0;
static int mtcp_read_line_2( char *out, unsigned cap )
{
    unsigned n = 0;
    for (;;) {
        while ( g_mread_pos >= g_mread_len ) {
            int16_t rc;
            if ( g_userBreak ) return -1;
            mtcp_pump();
            rc = g_sock->recv( g_mread_buf, sizeof g_mread_buf );
            if ( rc > 0 ) { g_mread_len = rc; g_mread_pos = 0; break; }
            if ( rc < 0 ) return -1;
            if ( g_sock->isRemoteClosed() ) return -2;
        }
        {
            unsigned char c = g_mread_buf[g_mread_pos++];
            if ( c == '\r' ) continue;
            if ( c == '\n' ) { out[n] = 0; return (int)n; }
            if ( n + 1 < cap ) out[n++] = (char)c;
        }
    }
}

static void mtcp_run_phase( const char *label, const char *url )
{
    IpAddr_t host;
    unsigned long content_length = 0;
    int have_clen = 0;
    int reqlen;
    unsigned long t_body, t_done, got = 0;

    log_section( label );
    log_print( "URL=%s\n", url );
    if ( parse_url( url ) ) { log_print( "bad URL\n" ); return; }
    log_print( "host=%s:%u path=%s\n", g_host, g_port_num, g_path );

    // Reset mtcp_read_line state for fresh socket
    g_mread_len = 0; g_mread_pos = 0;
    g_sock = NULL;

    if ( mtcp_resolve( g_host, host ) ) {
        log_print( "DNS fail for %s\n", g_host );
        return;
    }
    log_print( "resolved %s -> %u.%u.%u.%u\n", g_host, host[0], host[1], host[2], host[3] );

    if ( mtcp_connect( host, g_port_num ) ) {
        log_print( "connect fail\n" );
        return;
    }
    log_print( "connected\n" );

    reqlen = snprintf( g_req, sizeof g_req,
        "GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: tdongle-profile/1\r\n"
        "Connection: close\r\n\r\n", g_path, g_host );
    if ( mtcp_send_all( (unsigned char *)g_req, (unsigned)reqlen ) ) {
        log_print( "GET send fail\n" );
        return;
    }

    // Status + headers
    {
        int n = mtcp_read_line_2( g_line, sizeof g_line );
        if ( n < 0 ) { log_print( "no status line\n" ); return; }
        log_print( "%s\n", g_line );
        if ( strncmp( g_line, "HTTP/", 5 ) || !strstr( g_line, "200" ) ) {
            log_print( "non-200\n" ); return;
        }
        for (;;) {
            n = mtcp_read_line_2( g_line, sizeof g_line );
            if ( n < 0 ) { log_print( "header timeout\n" ); return; }
            if ( n == 0 ) break;
            if ( ( g_line[0] == 'C' || g_line[0] == 'c' ) && strlen( g_line ) > 16 ) {
                const char *p = g_line;
                const char *want = "Content-Length:";
                int match = 1; unsigned i;
                for ( i = 0; want[i]; ++i ) {
                    if ( tolower( (unsigned char)g_line[i] ) != tolower( (unsigned char)want[i] ) ) { match = 0; break; }
                }
                if ( match ) {
                    unsigned long v = 0;
                    p = g_line + 15;
                    while ( *p == ' ' || *p == '\t' ) ++p;
                    while ( *p >= '0' && *p <= '9' ) { v = v * 10 + (unsigned long)( *p - '0' ); ++p; }
                    content_length = v; have_clen = 1;
                }
            }
        }
    }
    if ( have_clen ) log_print( "Content-Length: %lu\n", content_length );

    t_body = bios_ticks();
    {
        unsigned long last_tick = t_body;
        unsigned long last_bytes = 0;
        unsigned inLen;
        while ( !have_clen || got < content_length ) {
            int r;
            if ( g_userBreak ) break;
            r = mtcp_refill( &inLen );
            if ( r < 0 ) { log_print( "recv error/timeout\n" ); break; }
            if ( r == 0 ) break;
            got += inLen;
            if ( have_clen && got > content_length ) got = content_length;
            if ( ( bios_ticks() - last_tick ) >= 18 ) {
                unsigned long db = got - last_bytes;
                unsigned long dt = bios_ticks() - last_tick;
                unsigned long kbps10;
                if ( !dt ) dt = 1;
                kbps10 = db * 10UL * 18UL / dt / 1024UL;
                printf( "  ... %lu bytes (%lu.%lu KB/s)        \r",
                        got, kbps10 / 10, kbps10 % 10 );
                last_tick = bios_ticks();
                last_bytes = got;
            }
        }
    }
    t_done = bios_ticks();
    printf( "\n" );

    {
        unsigned long body_ticks = t_done - t_body;
        if ( !body_ticks ) body_ticks = 1;
        {
            unsigned long centi = ( body_ticks * 10000UL + 910UL ) / 1820UL;
            unsigned long tkb10 = ( got * 1820UL / body_ticks * 10UL ) / 102400UL;
            log_print( "%s RESULT: %lu bytes in %lu.%02lus = %lu.%lu KB/s\n",
                       label, got, centi/100, centi%100, tkb10/10, tkb10%10 );
        }
    }
    return;                   // caller does endStack + FOSSLIP unload
}

// ---------------------------------------------------------------------------
// MAIN
// ---------------------------------------------------------------------------

int main( int argc, char **argv )
{
    const char *url_lan  = DEFAULT_LAN_URL;
    const char *url_inet = DEFAULT_INET_URL;
    if ( argc >= 2 ) url_lan  = argv[1];
    if ( argc >= 3 ) url_inet = argv[2];

    setbuf( stdout, NULL );
    log_open();
    log_print( "T-Dongle S3 PROFILE -- single-binary harness\n" );
    log_print( "LAN URL:  %s\n", url_lan );
    log_print( "INET URL: %s\n", url_inet );

    g_port = autodetect_com();
    if ( g_port < 0 ) { log_print( "no COM in BDA\n" ); return 1; }
    if ( !probe_fossil( (unsigned)g_port ) ) {
        log_print( "no FOSSIL on COM%d -- load CHUSB first\n", g_port + 1 );
        return 1;
    }
    log_print( "FOSSIL ok on COM%d\n", g_port + 1 );

    phase_0_baseline();
    phase_1_slip_udp();
    phase_2_hayes( url_inet );

    // Phases 3 + 4 share one FOSSLIP install + one mTCP init. Loading
    // FOSSLIP and bringing up mTCP per-phase doesn't work reliably -- the
    // unload + reload + reinit sequence corrupts state.
    log_section( "PHASES 3+4 -- mTCP setup" );
    if ( mtcp_session_setup() == 0 ) {
        mtcp_run_phase( "PHASE 3 -- mTCP HTTP via LAN",      url_lan );
        mtcp_run_phase( "PHASE 4 -- mTCP HTTP via INTERNET", url_inet );
        mtcp_session_teardown();
    } else {
        log_print( "mTCP setup failed -- skipping phases 3+4\n" );
        // Try to put dongle back in a known state regardless
        magic_escape();
    }

    log_section( "DONE" );
    log_print( "log saved to %s\n", LOG_FILENAME );
    fclose( g_log );
    return 0;
}

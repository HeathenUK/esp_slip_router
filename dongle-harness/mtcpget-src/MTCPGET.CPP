// mtcpget.cpp -- minimal HTTP/1.0 downloader that uses mTCP as a library
// (TcpSocketMgr, TcpSocket, Dns, Utils, packet driver). Clean-room MIT --
// no copy-paste from mTCP source; the only mTCP code linked is the
// TCP/IP/ARP/DNS library, included via its public headers.
//
// This is the tool used by the T-Dongle S3 P-* profiling pack to measure
// the SLIP-path HTTP throughput. An earlier version was a small patch
// against mTCP HTGET, which made the binary GPL-derivative; this file is
// our own code and keeps the harness clean.
//
// Scope, on purpose:
//   * HTTP/1.0 only. We rely on "Connection: close" + Content-Length and
//     do not handle Transfer-Encoding: chunked, redirects, or auth.
//   * Status 200 only. Anything else aborts with the status line printed.
//   * Prints "... NNNN bytes (KK.K KB/s)" once per second during the body
//     read, and a "MTCPGET: <bytes> in <s>s = <KB/s>" summary on completion.
//
// Usage:  MTCPGET [-o filename] <URL>
//         URL = http://host[:port]/path  (the http:// is optional)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <dos.h>

#include "types.h"
#include "utils.h"
#include "packet.h"
#include "arp.h"
#include "ip.h"
#include "tcp.h"
#include "tcpsockm.h"
#include "dns.h"
#include "timer.h"


// ---- constants -------------------------------------------------------------

static const uint16_t IN_BUF_SIZE   = 8192;
static const uint16_t LINE_BUF_SIZE = 1024;
static const uint16_t REQ_BUF_SIZE  = 1024;
static const uint16_t TCP_RECV_BUF  = 16384;
static const uint32_t CONNECT_TO_MS = 10000;
static const uint32_t IDLE_TO_MS    = 30000;


// ---- module state ----------------------------------------------------------

static char     Host[128];
static uint16_t Port = 80;
static char     Path[256];
static char     OutFilename[80] = "GOT.BIN";

static uint8_t  inBuf[IN_BUF_SIZE];
static uint16_t inBufStart = 0;
static uint16_t inBufLen   = 0;

static TcpSocket *Sock    = NULL;
static FILE      *OutFile = NULL;

// Ctrl-Break / Ctrl-C handler -- set the flag and let the main loop unwind
// cleanly via Utils::endStack (which restores the original timer hook).
static volatile uint8_t s_userBreak = 0;
static void __interrupt __far ctrlBreakHandler( ) { s_userBreak = 1; }


// ---- mTCP plumbing ---------------------------------------------------------

// One pass of the mTCP packet-driving plumbing. Call any time you'd
// otherwise spin tight -- this lets ARP / TCP / DNS make forward progress.
static inline void pumpMtcp( )
{
    PACKET_PROCESS_SINGLE;
    Arp::driveArp( );
    Tcp::drivePackets( );
    Dns::drivePendingQuery( );
}

static void cleanup( int rc )
{
    if ( OutFile ) { fclose( OutFile ); OutFile = NULL; }
    /* Don't call Sock->close() / TcpSocketMgr::freeSocket() here -- mTCP's
     * Utils::endStack() handles socket teardown internally, and explicitly
     * freeing a socket while its TCP state machine still has pending
     * sends/receives corrupts the heap allocator (the classic
     * "heap is corrupted" CRT message you get on exit). Stock HTGET's
     * shutdown is just endStack + exit; we mirror that. */
    Sock = NULL;
    Utils::endStack( );
    exit( rc );
}


// ---- URL parse -------------------------------------------------------------

static int parseUrl( const char *url )
{
    const char *p = url;
    if ( !strncmp( p, "http://", 7 ) ) p += 7;

    size_t i = 0;
    while ( *p && *p != ':' && *p != '/' ) {
        if ( i + 1 >= sizeof Host ) return -1;
        Host[i++] = *p++;
    }
    Host[i] = 0;
    if ( !i ) return -1;

    if ( *p == ':' ) {
        ++p;
        uint16_t v = 0;
        while ( *p >= '0' && *p <= '9' ) {
            v = (uint16_t)( v * 10 + ( *p - '0' ) );
            ++p;
        }
        Port = v ? v : 80;
    } else {
        Port = 80;
    }

    if ( *p == '/' ) {
        i = 0;
        while ( *p ) {
            if ( i + 1 >= sizeof Path ) return -1;
            Path[i++] = *p++;
        }
        Path[i] = 0;
    } else {
        Path[0] = '/'; Path[1] = 0;
    }
    return 0;
}


// ---- DNS / TCP / IO --------------------------------------------------------

// Resolve hostname (or accept a dotted-quad) into ipDest. 0 on success.
// Dns::resolve handles dotted-quads itself, so this is just a thin wrapper
// with polling + timeout.
static int resolveHost( const char *name, IpAddr_t ipDest )
{
    int8_t rc = Dns::resolve( name, ipDest, 1 );
    if ( rc < 0 ) return -1;

    clockTicks_t deadline = TIMER_GET_CURRENT( )
                          + (clockTicks_t)( CONNECT_TO_MS * TIMER_TICKS_PER_SEC / 1000UL );
    while ( rc == 1 ) {
        if ( s_userBreak ) return -1;
        if ( TIMER_GET_CURRENT( ) > deadline ) return -1;
        pumpMtcp( );
        rc = Dns::resolve( name, ipDest, 0 );
        if ( rc == 0 ) return 0;
        if ( rc <  0 ) return -1;
    }
    return ( rc == 0 ) ? 0 : -1;
}


static int connectSock( IpAddr_t host )
{
    Sock = TcpSocketMgr::getSocket( );
    if ( !Sock ) return -1;
    if ( Sock->setRecvBuffer( TCP_RECV_BUF ) ) return -1;

    uint16_t localPort = (uint16_t)( 49152 + ( TIMER_GET_CURRENT( ) & 0x3FFF ) );
    if ( Sock->connectNonBlocking( localPort, host, Port ) ) return -1;

    clockTicks_t deadline = TIMER_GET_CURRENT( )
                          + (clockTicks_t)( CONNECT_TO_MS * TIMER_TICKS_PER_SEC / 1000UL );
    while ( !Sock->isConnectComplete( ) ) {
        if ( s_userBreak )                   return -1;
        if ( Sock->isClosed( ) )             return -1;
        if ( TIMER_GET_CURRENT( ) > deadline ) return -1;
        pumpMtcp( );
    }
    return 0;
}


// mTCP's enqueue is best-effort; on a 0 return we drive the stack and retry.
static int sendAll( const uint8_t *buf, uint16_t len )
{
    uint16_t sent = 0;
    clockTicks_t deadline = TIMER_GET_CURRENT( )
                          + (clockTicks_t)( IDLE_TO_MS * TIMER_TICKS_PER_SEC / 1000UL );
    while ( sent < len ) {
        if ( s_userBreak )                    return -1;
        if ( Sock->isClosed( ) )              return -1;
        if ( TIMER_GET_CURRENT( ) > deadline ) return -1;
        pumpMtcp( );
        int16_t rc = Sock->send( (uint8_t *)( buf + sent ),
                                 (uint16_t)( len - sent ) );
        if ( rc > 0 ) sent = (uint16_t)( sent + rc );
    }
    return 0;
}


// Refill inBuf. Returns bytes added, 0 on remote-close, -1 on error/timeout.
static int16_t refill( )
{
    if ( inBufStart == inBufLen ) { inBufStart = 0; inBufLen = 0; }
    if ( inBufLen == IN_BUF_SIZE ) return 0;
    uint16_t space = (uint16_t)( IN_BUF_SIZE - inBufLen );

    clockTicks_t deadline = TIMER_GET_CURRENT( )
                          + (clockTicks_t)( IDLE_TO_MS * TIMER_TICKS_PER_SEC / 1000UL );
    for (;;) {
        if ( s_userBreak ) return -1;
        pumpMtcp( );
        int16_t rc = Sock->recv( inBuf + inBufLen, space );
        if ( rc > 0 ) { inBufLen = (uint16_t)( inBufLen + rc ); return rc; }
        if ( rc < 0 ) return -1;
        if ( Sock->isRemoteClosed( ) ) return 0;
        if ( TIMER_GET_CURRENT( ) > deadline ) return -1;
    }
}


// Read one CRLF-terminated line into out (without CR/LF). Returns length,
// -1 on error, -2 on remote close before LF.
static int16_t readLine( char *out, uint16_t outCap )
{
    uint16_t n = 0;
    for (;;) {
        if ( inBufStart == inBufLen ) {
            int16_t r = refill( );
            if ( r < 0 ) return -1;
            if ( r == 0 ) return -2;
        }
        uint8_t c = inBuf[inBufStart++];
        if ( c == '\r' ) continue;
        if ( c == '\n' ) { out[n] = 0; return (int16_t)n; }
        if ( n + 1 < outCap ) out[n++] = (char)c;
    }
}


static int parseStatusAndHeaders( uint32_t *contentLength, int *haveLength )
{
    char line[LINE_BUF_SIZE];
    int16_t n;

    n = readLine( line, sizeof line );
    if ( n < 0 ) { printf( "MTCPGET: no status line\n" ); return -1; }
    printf( "  %s\n", line );

    if ( strncmp( line, "HTTP/", 5 ) ) { printf( "MTCPGET: not HTTP\n" ); return -1; }
    char *sp = strchr( line, ' ' );
    if ( !sp ) return -1;
    while ( *sp == ' ' ) ++sp;
    if ( strncmp( sp, "200", 3 ) ) {
        printf( "MTCPGET: non-200 status\n" );
        return -1;
    }

    *contentLength = 0;
    *haveLength = 0;
    static const char want[] = "Content-Length:";
    for (;;) {
        n = readLine( line, sizeof line );
        if ( n < 0 ) return -1;
        if ( n == 0 ) break;
        if ( strlen( line ) <= sizeof( want ) - 1 ) continue;
        int match = 1;
        for ( size_t i = 0; i < sizeof( want ) - 1; ++i ) {
            if ( tolower( (unsigned char)line[i] ) !=
                 tolower( (unsigned char)want[i] ) ) { match = 0; break; }
        }
        if ( !match ) continue;
        const char *p = line + sizeof( want ) - 1;
        while ( *p == ' ' || *p == '\t' ) ++p;
        uint32_t v = 0;
        while ( *p >= '0' && *p <= '9' ) { v = v * 10 + (uint32_t)( *p - '0' ); ++p; }
        *contentLength = v;
        *haveLength = 1;
    }
    return 0;
}


// ---- main ------------------------------------------------------------------

int main( int argc, char *argv[] )
{
    setbuf( stdout, NULL );
    printf( "MTCPGET 1.0 -- T-Dongle S3 SLIP HTTP benchmark\n" );

    const char *url = NULL;
    for ( int i = 1; i < argc; ++i ) {
        if ( !strcmp( argv[i], "-o" ) && i + 1 < argc ) {
            strncpy( OutFilename, argv[++i], sizeof OutFilename - 1 );
            OutFilename[sizeof OutFilename - 1] = 0;
        } else if ( argv[i][0] != '-' ) {
            url = argv[i];
        }
    }
    if ( !url ) {
        printf( "usage: MTCPGET [-o file] <URL>\n" );
        return 1;
    }
    if ( parseUrl( url ) ) { printf( "MTCPGET: bad URL\n" ); return 1; }
    printf( "  host=%s:%u  path=%s  out=%s\n", Host, Port, Path, OutFilename );

    if ( Utils::parseEnv( ) ) return 1;
    if ( Utils::initStack( 1, TCP_SOCKET_RING_SIZE,
                           ctrlBreakHandler, ctrlBreakHandler ) ) {
        printf( "MTCPGET: TCP/IP init failed\n" );
        return 1;
    }

    IpAddr_t host;
    if ( resolveHost( Host, host ) ) { printf( "MTCPGET: DNS fail\n" ); cleanup( 1 ); }
    printf( "  %s -> %u.%u.%u.%u\n", Host, host[0], host[1], host[2], host[3] );

    if ( connectSock( host ) ) { printf( "MTCPGET: connect fail\n" ); cleanup( 1 ); }
    printf( "  connected\n" );

    static char req[REQ_BUF_SIZE];
    int reqLen = snprintf( req, sizeof req,
        "GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: mtcpget/1\r\n"
        "Connection: close\r\n\r\n", Path, Host );
    if ( sendAll( (uint8_t *)req, (uint16_t)reqLen ) ) {
        printf( "MTCPGET: GET send fail\n" );
        cleanup( 1 );
    }

    uint32_t contentLength = 0;
    int haveLength = 0;
    if ( parseStatusAndHeaders( &contentLength, &haveLength ) ) cleanup( 1 );
    if ( haveLength ) printf( "  Content-Length: %lu\n", contentLength );
    else              printf( "  (no Content-Length, reading to remote close)\n" );

    OutFile = fopen( OutFilename, "wb" );
    if ( !OutFile ) { printf( "MTCPGET: cannot open %s\n", OutFilename ); cleanup( 1 ); }
    /* Big fully-buffered file buffer: turns the per-chunk disk traffic into one
     * large, efficient flush per 8 KB instead of ~16 tiny default-buffer writes,
     * which shrinks the window where the stack goes dark during disk I/O. */
    static char s_fileBuf[8192];
    setvbuf( OutFile, s_fileBuf, _IOFBF, sizeof s_fileBuf );

    clockTicks_t bodyStart = TIMER_GET_CURRENT( );
    clockTicks_t lastTick  = bodyStart;
    uint32_t lastBytes = 0;
    uint32_t got = 0;

    while ( !haveLength || got < contentLength ) {
        if ( s_userBreak ) break;

        if ( inBufStart < inBufLen ) {
            uint16_t avail = (uint16_t)( inBufLen - inBufStart );
            if ( haveLength && (uint32_t)avail > contentLength - got ) {
                avail = (uint16_t)( contentLength - got );
            }
            /* Write in small pieces and pump the stack between each, so ACKs
             * keep flowing and the receiver keeps draining DURING the disk
             * write. A real NIC's hardware buffer does this for free; the dongle
             * has no RX buffer, so if we stop ACKing here its tiny TX queue
             * overflows, full-size frames drop, and the path MSS collapses to
             * 512 (measured: -o NUL 97 KB/s vs -o CF 12.9 KB/s). */
            uint16_t w = 0;
            while ( w < avail ) {
                uint16_t chunk = (uint16_t)( avail - w );
                if ( chunk > 512U ) chunk = 512U;
                if ( fwrite( inBuf + inBufStart + w, 1, chunk, OutFile ) != chunk ) {
                    printf( "MTCPGET: file write fail\n" );
                    cleanup( 1 );
                }
                w = (uint16_t)( w + chunk );
                pumpMtcp( );   /* keep the net serviced through the write */
            }
            inBufStart = (uint16_t)( inBufStart + avail );
            got += avail;
        } else {
            int16_t r = refill( );
            if ( r < 0 ) { printf( "MTCPGET: recv error / timeout\n" ); break; }
            if ( r == 0 ) break;
        }

        clockTicks_t now = TIMER_GET_CURRENT( );
        clockTicks_t dt  = now - lastTick;
        if ( dt >= (clockTicks_t)TIMER_TICKS_PER_SEC ) {
            uint32_t db = got - lastBytes;
            uint32_t kbps10 = (uint32_t)( db * 10UL * TIMER_TICKS_PER_SEC
                                          / (uint32_t)dt / 1024UL );
            printf( "  ... %lu bytes (%lu.%lu KB/s)\r",
                    got,
                    (unsigned long)( kbps10 / 10 ),
                    (unsigned long)( kbps10 % 10 ) );
            lastTick  = now;
            lastBytes = got;
        }
    }

    clockTicks_t bodyEnd = TIMER_GET_CURRENT( );
    fclose( OutFile ); OutFile = NULL;

    clockTicks_t totalTicks = bodyEnd - bodyStart;
    if ( totalTicks == 0 ) totalTicks = 1;
    uint32_t kbps10 = (uint32_t)( got * 10UL * TIMER_TICKS_PER_SEC
                                  / (uint32_t)totalTicks / 1024UL );
    uint32_t centiS = (uint32_t)( totalTicks * 100UL / TIMER_TICKS_PER_SEC );
    printf( "\nMTCPGET: %lu bytes in %lu.%02lus = %lu.%lu KB/s\n",
            got,
            (unsigned long)( centiS / 100 ), (unsigned long)( centiS % 100 ),
            (unsigned long)( kbps10 / 10 ),  (unsigned long)( kbps10 % 10 ) );

    cleanup( 0 );
    return 0;
}

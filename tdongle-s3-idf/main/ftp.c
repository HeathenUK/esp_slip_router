/**
 * @file ftp.c
 * @brief Minimal FTP client engine for the AT$FTP terminating dial (Phase 1).
 *
 * Control-connection only: connect, log in, and run PWD/CWD/CDUP. Plaintext, so
 * no crypto -- far lighter on heap than ssh.c. PASV data transfers (LIST/RETR)
 * arrive in later phases and reuse ftp_command() + the control fd here.
 *
 * FTP replies are one or more lines: a multi-line reply repeats the 3-digit code
 * with a '-' after it on every line except the last, which uses a space
 * ("230-line one\r\n230 done\r\n"). We read line by line until a "DDD " terminal
 * line. Never logs the password.
 */
#include "ftp.h"

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "disk.h"   /* disk_logf */

static int s_ctrl = -1;   /* control socket fd, or -1 */

int ftp_ctrl_fd(void) { return s_ctrl; }

/**
 * @brief Read one full (possibly multi-line) FTP reply from the control socket.
 * @param out Optional buffer for the reply text (LF-separated, truncated).
 * @param outlen Size of @p out.
 * @return The 3-digit reply code, or -1 on socket error/timeout.
 */
static int ftp_read_reply(char *out, size_t outlen)
{
    int  code = -1;
    size_t outpos = 0;
    if (out && outlen) out[0] = '\0';

    for (;;) {
        char    line[256];
        size_t  li = 0;
        /* Read one CRLF-terminated line (control replies are small; byte-at-a-time
         * is fine and keeps us from over-reading into the next reply). */
        for (;;) {
            char c;
            int r = recv(s_ctrl, &c, 1, 0);
            if (r <= 0) return -1;                 /* closed / timeout */
            if (c == '\n') break;
            if (c != '\r' && li < sizeof line - 1) line[li++] = c;
        }
        line[li] = '\0';

        if (out && outpos + li + 1 < outlen) {
            memcpy(out + outpos, line, li);
            outpos += li;
            out[outpos++] = '\n';
            out[outpos]   = '\0';
        }

        if (li >= 3 && isdigit((unsigned char)line[0]) &&
            isdigit((unsigned char)line[1]) && isdigit((unsigned char)line[2])) {
            if (code < 0)
                code = (line[0]-'0')*100 + (line[1]-'0')*10 + (line[2]-'0');
            /* Terminal line: "DDD " (space at index 3) or a bare "DDD". A '-' at
             * index 3 marks a continuation line -- keep reading. */
            if (li == 3 || line[3] == ' ') break;
        }
        /* else: continuation text without a leading code -- keep reading */
    }
    return code;
}

int ftp_command(const char *cmd, char *resp, size_t resp_len)
{
    if (s_ctrl < 0) return -1;
    if (cmd) {
        char line[300];
        int n = snprintf(line, sizeof line, "%s\r\n", cmd);
        if (n <= 0 || n >= (int)sizeof line) return -1;
        if (send(s_ctrl, line, (size_t)n, 0) != n) return -1;
    }
    return ftp_read_reply(resp, resp_len);
}

int ftp_connect(const char *host, uint16_t port, const char *user, const char *pass,
                char *banner, size_t banner_len)
{
    struct addrinfo  hints, *res = NULL;
    char             ps[8];
    int              sock = -1, code;

    s_ctrl = -1;
    if (banner && banner_len) banner[0] = '\0';

    snprintf(ps, sizeof ps, "%u", (unsigned)port);
    memset(&hints, 0, sizeof hints);
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host, ps, &hints, &res) != 0 || !res) {
        disk_logf("ftp: DNS fail '%s'", host);
        return -1;
    }
    sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) { freeaddrinfo(res); disk_logf("ftp: socket fail"); return -1; }

    /* Generous timeout for connect + login; the REPL keeps it for command replies
     * (an interactive command's reply arrives fast -- this is just the safety net). */
    {
        struct timeval tv = { .tv_sec = 15, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    }
    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        freeaddrinfo(res); close(sock); disk_logf("ftp: connect fail"); return -1;
    }
    freeaddrinfo(res);
    { int yes = 1; setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof yes); }
    s_ctrl = sock;

    /* Greeting (220). */
    code = ftp_read_reply(banner, banner_len);
    if (code / 100 != 2) { disk_logf("ftp: bad greeting code=%d", code); ftp_close(); return -1; }

    /* Login. USER -> 331 means "send PASS"; 230 means already logged in. */
    {
        char cmd[160];
        snprintf(cmd, sizeof cmd, "USER %s", (user && *user) ? user : "anonymous");
        code = ftp_command(cmd, NULL, 0);
        if (code == 331) {
            snprintf(cmd, sizeof cmd, "PASS %s", (pass && *pass) ? pass : "ftp@");
            code = ftp_command(cmd, NULL, 0);   /* never logged */
        }
    }
    if (code / 100 != 2) { disk_logf("ftp: login failed code=%d user=%s", code,
                                     (user && *user) ? user : "anonymous"); ftp_close(); return -1; }

    disk_logf("ftp: connected fd=%d %s@%s:%u", s_ctrl,
              (user && *user) ? user : "anonymous", host, (unsigned)port);
    return s_ctrl;
}

int ftp_pwd(char *out, size_t n)
{
    char resp[160];
    int code = ftp_command("PWD", resp, sizeof resp);
    if (out && n) out[0] = '\0';
    if (code == 257 && out && n) {
        /* 257 "/path" possibly more -- extract the quoted path. */
        char *q1 = strchr(resp, '"');
        if (q1) {
            char *q2 = strchr(q1 + 1, '"');
            size_t len = q2 ? (size_t)(q2 - q1 - 1) : strlen(q1 + 1);
            if (len >= n) len = n - 1;
            memcpy(out, q1 + 1, len);
            out[len] = '\0';
        }
    }
    return code;
}

int ftp_cwd(const char *dir)
{
    if (!dir || !*dir || !strcmp(dir, ".."))
        return ftp_command("CDUP", NULL, 0);
    {
        char cmd[256];
        snprintf(cmd, sizeof cmd, "CWD %s", dir);
        return ftp_command(cmd, NULL, 0);
    }
}

/**
 * @brief Enter passive mode and open the data socket to the server's PASV addr.
 * @return The connected data socket fd, or -1 on failure.
 */
static int ftp_open_data(void)
{
    char resp[160];
    int  code = ftp_command("PASV", resp, sizeof resp);
    int  h0, h1, h2, h3, p0, p1;
    char *p;
    struct sockaddr_in sa;
    int  ds;

    if (code != 227) { disk_logf("ftp: PASV code=%d", code); return -1; }
    p = strchr(resp, '(');
    if (!p || sscanf(p + 1, "%d,%d,%d,%d,%d,%d", &h0, &h1, &h2, &h3, &p0, &p1) != 6) {
        disk_logf("ftp: PASV parse fail"); return -1;
    }
    memset(&sa, 0, sizeof sa);
    sa.sin_family      = AF_INET;
    sa.sin_port        = htons((uint16_t)(p0 * 256 + p1));
    sa.sin_addr.s_addr = htonl(((uint32_t)h0 << 24) | ((uint32_t)h1 << 16) |
                               ((uint32_t)h2 << 8)  |  (uint32_t)h3);
    /* 0.0.0.0 means "reuse the control connection's server IP". */
    if (sa.sin_addr.s_addr == 0) {
        struct sockaddr_in pa; socklen_t pl = sizeof pa;
        if (getpeername(s_ctrl, (struct sockaddr *)&pa, &pl) == 0) sa.sin_addr = pa.sin_addr;
    }
    ds = socket(AF_INET, SOCK_STREAM, 0);
    if (ds < 0) return -1;
    { struct timeval tv = { .tv_sec = 15, .tv_usec = 0 };
      setsockopt(ds, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
      setsockopt(ds, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv); }
    if (connect(ds, (struct sockaddr *)&sa, sizeof sa) != 0) {
        disk_logf("ftp: data connect fail"); close(ds); return -1;
    }
    return ds;
}

int ftp_list(const char *path, ftp_sink_fn sink)
{
    int ds, code;

    ftp_command("TYPE A", NULL, 0);              /* ASCII for listings */
    ds = ftp_open_data();
    if (ds < 0) return -1;

    {
        char cmd[256];
        if (path && *path) snprintf(cmd, sizeof cmd, "LIST %s", path);
        else               snprintf(cmd, sizeof cmd, "LIST");
        code = ftp_command(cmd, NULL, 0);        /* 150/125 = transfer starting */
    }
    if (code != 150 && code != 125) { close(ds); return code < 0 ? -1 : code; }

    for (;;) {
        uint8_t buf[256];
        int n = recv(ds, buf, sizeof buf, 0);
        if (n > 0) { if (sink) sink(buf, (size_t)n); }
        else break;                              /* 0 = EOF, <0 = timeout/error */
    }
    close(ds);
    return ftp_read_reply(NULL, 0);              /* final 226 */
}

long ftp_size(const char *path)
{
    char cmd[256], resp[80];
    int  code;
    ftp_command("TYPE I", NULL, 0);              /* binary -> SIZE is the byte count */
    snprintf(cmd, sizeof cmd, "SIZE %s", path);
    code = ftp_command(cmd, resp, sizeof resp);
    if (code != 213) return -1;
    { const char *q = resp + 3; while (*q == ' ') q++; return atol(q); }
}

int ftp_retr(const char *path, ftp_sink_fn sink)
{
    int ds, code;

    ftp_command("TYPE I", NULL, 0);
    ds = ftp_open_data();
    if (ds < 0) return -1;
    {
        char cmd[256];
        snprintf(cmd, sizeof cmd, "RETR %s", path);
        code = ftp_command(cmd, NULL, 0);        /* 150/125 = transfer starting */
    }
    if (code != 150 && code != 125) { close(ds); return code < 0 ? -1 : code; }

    for (;;) {
        uint8_t buf[512];
        int n = recv(ds, buf, sizeof buf, 0);
        if (n > 0) { if (sink) sink(buf, (size_t)n); }
        else break;                              /* 0 = EOF, <0 = timeout/error */
    }
    close(ds);
    return ftp_read_reply(NULL, 0);              /* final 226 */
}

int ftp_mkdir(const char *dir)  { char c[260]; snprintf(c, sizeof c, "MKD %s",  dir);  return ftp_command(c, NULL, 0); }
int ftp_rmdir(const char *dir)  { char c[260]; snprintf(c, sizeof c, "RMD %s",  dir);  return ftp_command(c, NULL, 0); }
int ftp_del(const char *file)   { char c[260]; snprintf(c, sizeof c, "DELE %s", file); return ftp_command(c, NULL, 0); }

int ftp_rename(const char *oldn, const char *newn)
{
    char c[260];
    int  code;
    snprintf(c, sizeof c, "RNFR %s", oldn);
    code = ftp_command(c, NULL, 0);
    if (code != 350) return code < 0 ? -1 : code;   /* 350 = ready for RNTO */
    snprintf(c, sizeof c, "RNTO %s", newn);
    return ftp_command(c, NULL, 0);
}

/* ---- upload (STOR) ---- */
static int s_stor_fd = -1;

int ftp_stor_open(const char *remote)
{
    int code;
    ftp_command("TYPE I", NULL, 0);
    s_stor_fd = ftp_open_data();
    if (s_stor_fd < 0) return -1;
    { char c[260]; snprintf(c, sizeof c, "STOR %s", remote); code = ftp_command(c, NULL, 0); }
    if (code != 150 && code != 125) { close(s_stor_fd); s_stor_fd = -1; return code < 0 ? -1 : code; }
    return 0;
}

int ftp_stor_write(const void *buf, size_t n)
{
    const char *p = buf;
    size_t off = 0;
    if (s_stor_fd < 0) return -1;
    while (off < n) {
        int w = send(s_stor_fd, p + off, n - off, 0);
        if (w > 0) off += (size_t)w;
        else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
        else return -1;
    }
    return 0;
}

int ftp_stor_close(void)
{
    if (s_stor_fd >= 0) { close(s_stor_fd); s_stor_fd = -1; }
    return ftp_read_reply(NULL, 0);   /* final 226 */
}

void ftp_quit(void)
{
    if (s_ctrl >= 0) ftp_command("QUIT", NULL, 0);   /* best-effort */
    ftp_close();
}

void ftp_close(void)
{
    if (s_ctrl >= 0) { close(s_ctrl); s_ctrl = -1; }
}

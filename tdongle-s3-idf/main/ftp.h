/**
 * @file ftp.h
 * @brief Minimal FTP client engine for the AT$FTP terminating dial (Phase 1).
 *
 * Protocol-only: opens + logs into an FTP control connection and runs the basic
 * navigation verbs (PWD/CWD/CDUP). The interactive REPL, AT dispatch and CDC I/O
 * live in modem.c; this layer never touches the modem state or CDC. Plaintext --
 * no crypto, so it is far lighter on heap than the SSH path. PASV data transfers
 * (LIST/RETR) come in later phases and will reuse ftp_command()/the control fd.
 */
#ifndef FTP_H
#define FTP_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Open + log into an FTP control connection.
 * @param host  Server hostname or IP.
 * @param port  Control port (usually 21).
 * @param user  Username ("anonymous" if NULL/empty).
 * @param pass  Password (may be NULL/empty; "ftp@" used for anonymous).
 * @param banner Optional buffer for the server's greeting/login text (truncated).
 * @param banner_len Size of @p banner.
 * @return The control socket fd on success (also retained internally), or -1 on
 *         DNS/connect/login failure (all state freed, socket closed).
 */
int ftp_connect(const char *host, uint16_t port, const char *user, const char *pass,
                char *banner, size_t banner_len);

/**
 * @brief Send one raw FTP command and read its (possibly multi-line) reply.
 * @param cmd  Command without CRLF (e.g. "CWD pub", "PWD"); NULL just reads a reply.
 * @param resp Optional buffer for the reply text (truncated to fit).
 * @param resp_len Size of @p resp.
 * @return The 3-digit FTP reply code (e.g. 250), or -1 on socket error/timeout.
 */
int ftp_command(const char *cmd, char *resp, size_t resp_len);

/**
 * @brief PWD -- print working directory.
 * @param out Buffer for the unquoted path.
 * @param n   Size of @p out.
 * @return FTP reply code (257 on success), or -1 on error.
 */
int ftp_pwd(char *out, size_t n);

/**
 * @brief Change directory (CWD <dir>, or CDUP when dir is ".." / NULL).
 * @param dir Target directory; "..", NULL or empty -> CDUP.
 * @return FTP reply code (250 on success), or -1 on error.
 */
int ftp_cwd(const char *dir);

/** @brief Sink callback for streamed data (directory listings; later, file bodies). */
typedef void (*ftp_sink_fn)(const uint8_t *data, size_t len);

/**
 * @brief List a directory: TYPE A + PASV + LIST, streaming the text to @p sink.
 * @param path Directory/glob, or NULL/"" for the current directory.
 * @param sink Called with each chunk of listing text (NULL to discard).
 * @return The final FTP reply code (226 on success), or -1 on socket error.
 */
int ftp_list(const char *path, ftp_sink_fn sink);

/** @brief Send QUIT (best-effort) and close the control connection. */
void ftp_quit(void);

/** @brief Close the control connection + free state, without sending QUIT. Idempotent. */
void ftp_close(void);

/** @brief The control socket fd, or -1 if not connected. */
int ftp_ctrl_fd(void);

#endif /* FTP_H */

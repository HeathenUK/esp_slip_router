/* dns_forwarder.c -- intercept DNS on the SLIP netif IP; resolve via
 * the dongle's WiFi resolver and synthesize the response. Bypasses
 * NAPT for DNS, fixing the Phase 4 timeouts where the NAPT UDP
 * mapping was GC'd before the response came back. */

#include "dns_forwarder.h"

#include <stdint.h>
#include <string.h>

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "disk.h"

struct dns_hdr {
    uint16_t id;
    uint16_t flags;
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
} __attribute__((packed));

#define DNS_QTYPE_A     1
#define DNS_QTYPE_AAAA  28
#define DNS_QCLASS_IN   1

#define DNS_FLAG_QR        0x8000
#define DNS_FLAG_AA        0x0400
#define DNS_FLAG_RA        0x0080
#define DNS_FLAG_RD        0x0100
#define DNS_RCODE_NXDOMAIN 3
#define DNS_RCODE_NOTIMP   4

static volatile uint32_t s_queries  = 0;
static volatile uint32_t s_resolved = 0;
static volatile uint32_t s_nxdomain = 0;

uint32_t dns_fwd_stat_queries(void)  { return s_queries; }
uint32_t dns_fwd_stat_resolved(void) { return s_resolved; }
uint32_t dns_fwd_stat_nxdomain(void) { return s_nxdomain; }

static size_t qname_decode(const uint8_t *p, size_t plen, char *out, size_t out_sz) {
    size_t pos  = 0;
    size_t opos = 0;
    while (pos < plen) {
        uint8_t lbl = p[pos];
        if (lbl == 0) {
            if (opos < out_sz) out[opos] = 0;
            else if (out_sz)   out[out_sz - 1] = 0;
            return pos + 1;
        }
        if (lbl & 0xC0) return 0;                /* no compression in query */
        if (pos + 1 + lbl > plen)         return 0;
        if (opos + lbl + 1 >= out_sz)     return 0;
        if (opos) out[opos++] = '.';
        memcpy(out + opos, p + pos + 1, lbl);
        opos += lbl;
        pos  += 1 + lbl;
    }
    return 0;
}

static void send_err_response(int sock, uint8_t *rbuf, size_t qlen_in_rbuf,
                              struct dns_hdr *rh, uint16_t flags_in,
                              uint16_t rcode,
                              const struct sockaddr *src, socklen_t srclen)
{
    uint16_t rflags = DNS_FLAG_QR | DNS_FLAG_RA | (flags_in & DNS_FLAG_RD) | rcode;
    rh->flags   = htons(rflags);
    rh->ancount = 0;
    sendto(sock, rbuf, qlen_in_rbuf, 0, src, srclen);
}

static void dns_task(void *arg) {
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        disk_logf("dns_fwd: socket() failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = { 0 };
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(53);
    /* 192.168.240.1 -- the SLIP netif's IP. */
    addr.sin_addr.s_addr = htonl((192U << 24) | (168U << 16) | (240U << 8) | 1U);
    if (bind(sock, (struct sockaddr *)&addr, sizeof addr) < 0) {
        disk_logf("dns_fwd: bind() failed errno=%d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    disk_logf("dns_fwd: listening on 192.168.240.1:53");

    static uint8_t qbuf[512];
    static uint8_t rbuf[512];
    static char    qname[256];

    while (1) {
        struct sockaddr_in src;
        socklen_t          srclen = sizeof src;
        int qlen = recvfrom(sock, qbuf, sizeof qbuf, 0,
                            (struct sockaddr *)&src, &srclen);
        if (qlen < (int)(sizeof(struct dns_hdr) + 5)) continue;
        s_queries++;

        struct dns_hdr h;
        memcpy(&h, qbuf, sizeof h);
        uint16_t flags_in = ntohs(h.flags);
        uint16_t qdcount  = ntohs(h.qdcount);
        if ((flags_in & DNS_FLAG_QR) || qdcount != 1) continue;

        size_t qname_len = qname_decode(qbuf + sizeof h, qlen - sizeof h,
                                        qname, sizeof qname);
        if (qname_len == 0) continue;
        if (sizeof h + qname_len + 4 > (size_t)qlen) continue;

        uint16_t qtype, qclass;
        memcpy(&qtype,  qbuf + sizeof h + qname_len,     2);
        memcpy(&qclass, qbuf + sizeof h + qname_len + 2, 2);
        qtype  = ntohs(qtype);
        qclass = ntohs(qclass);

        /* Copy header+question into the response buffer; we keep the
         * client's ID and question section verbatim. */
        size_t qsz = sizeof h + qname_len + 4;
        memcpy(rbuf, qbuf, qsz);
        struct dns_hdr *rh = (struct dns_hdr *)rbuf;

        if (qclass != DNS_QCLASS_IN ||
            (qtype != DNS_QTYPE_A && qtype != DNS_QTYPE_AAAA)) {
            send_err_response(sock, rbuf, qsz, rh, flags_in, DNS_RCODE_NOTIMP,
                              (struct sockaddr *)&src, srclen);
            continue;
        }

        if (qtype == DNS_QTYPE_AAAA) {
            /* IPv4 only -- return NOERROR with empty answer section. */
            send_err_response(sock, rbuf, qsz, rh, flags_in, 0,
                              (struct sockaddr *)&src, srclen);
            continue;
        }

        struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
        struct addrinfo *res = NULL;
        int rc = getaddrinfo(qname, NULL, &hints, &res);
        if (rc != 0 || !res) {
            s_nxdomain++;
            send_err_response(sock, rbuf, qsz, rh, flags_in, DNS_RCODE_NXDOMAIN,
                              (struct sockaddr *)&src, srclen);
            if (res) freeaddrinfo(res);
            continue;
        }
        uint32_t ip = ((struct sockaddr_in *)res->ai_addr)->sin_addr.s_addr;
        freeaddrinfo(res);
        s_resolved++;

        /* Append one A record after the question. NAME is a compression
         * pointer to QNAME at offset 12 (start of question). */
        size_t off = qsz;
        rbuf[off++] = 0xC0;
        rbuf[off++] = 0x0C;
        uint16_t t  = htons(DNS_QTYPE_A);   memcpy(rbuf + off, &t,  2); off += 2;
        uint16_t c  = htons(DNS_QCLASS_IN); memcpy(rbuf + off, &c,  2); off += 2;
        uint32_t ttl= htonl(60);            memcpy(rbuf + off, &ttl,4); off += 4;
        uint16_t rd = htons(4);             memcpy(rbuf + off, &rd, 2); off += 2;
        memcpy(rbuf + off, &ip, 4);         off += 4;

        rh->ancount = htons(1);
        rh->flags   = htons(DNS_FLAG_QR | DNS_FLAG_AA | DNS_FLAG_RA |
                            (flags_in & DNS_FLAG_RD));
        sendto(sock, rbuf, off, 0, (struct sockaddr *)&src, srclen);
    }
}

void dns_forwarder_init(void) {
    xTaskCreatePinnedToCore(dns_task, "dns_fwd", 3072, NULL, 5, NULL, 0);  /* SRAM: ~1.9K used (was 4096) */
}

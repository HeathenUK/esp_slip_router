/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 by Sergey Fetisov <fsenok@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file dhserver.h
 * @brief Tiny DHCP server for the CDC-ECM netif.
 *
 * Vendored from TinyUSB 0.19.0 `lib/networking/dhserver.{c,h}` (MIT, Sergey
 * Fetisov 2015 -- header above kept verbatim). Local modifications for the
 * T-Dongle S3:
 *  - `dhcp_config_t.netif`: the server answers ONLY on this netif. Upstream
 *    binds 0.0.0.0:67 and would happily serve leases to DHCP DISCOVERs heard
 *    on the WiFi STA side of the bridge -- poisoning the home LAN.
 *  - callers must hold LOCK_TCPIP_CORE() around dhserv_init()/dhserv_free()
 *    (this build runs full-OS lwIP with core locking, not the bare-metal
 *    NO_SYS loop the upstream example used).
 *  - dropped upstream's udp_init() call in dhserv_init: IDF's tcpip task
 *    already ran lwip_init(), and re-running udp_init would reset the
 *    ephemeral local-port counter under live sockets.
 *  - DHCP_REQUEST: RENEWING/REBINDING clients carry the address in ciaddr
 *    with no option 50 (RFC 2131 4.3.2); upstream ignored them, so leases
 *    could never renew. We fall back to ciaddr.
 */
#ifndef DHSERVER_H
#define DHSERVER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lwip/err.h"
#include "lwip/udp.h"
#include "lwip/netif.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dhcp_entry
{
	uint8_t    mac[6];
	ip4_addr_t addr;
	uint32_t   lease;
} dhcp_entry_t;

typedef struct dhcp_config
{
	ip4_addr_t    router;
	uint16_t      port;
	ip4_addr_t    dns;
	const char   *domain;
	int           num_entry;
	dhcp_entry_t *entries;
	struct netif *netif;   /* LOCAL MOD: serve ONLY this netif (NULL = any) */
} dhcp_config_t;

/** @brief Start the DHCP server. Caller must hold LOCK_TCPIP_CORE(). */
err_t dhserv_init(const dhcp_config_t *c);
/** @brief Stop the DHCP server. Caller must hold LOCK_TCPIP_CORE(). */
void dhserv_free(void);

#ifdef __cplusplus
}
#endif

#endif /* DHSERVER_H */

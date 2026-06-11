/* dns_forwarder.h -- UDP/53 listener on the SLIP netif IP that resolves
 * client queries via the dongle's WiFi resolver. Bypasses NAPT for DNS
 * entirely so mTCP's queries don't depend on UDP NAT timeouts.
 *
 * MTCP.CFG on the DOS side should set NAMESERVER = 192.168.240.1 to
 * point at us.
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void dns_forwarder_init(void);

/** @brief Start a forwarder instance bound to @p ip_hostorder:53 (one per
 *  netif IP -- the ECM bridge runs its own beside the SLIP one). */
void dns_forwarder_init_ip(uint32_t ip_hostorder);

uint32_t dns_fwd_stat_queries(void);
uint32_t dns_fwd_stat_resolved(void);
uint32_t dns_fwd_stat_nxdomain(void);

#ifdef __cplusplus
}
#endif

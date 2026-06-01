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

uint32_t dns_fwd_stat_queries(void);
uint32_t dns_fwd_stat_resolved(void);
uint32_t dns_fwd_stat_nxdomain(void);

#ifdef __cplusplus
}
#endif

/* dns_forwarder.h -- UDP/53 listener on a bridge netif IP that resolves
 * client queries via the dongle's WiFi resolver. Bypasses NAPT for DNS
 * entirely so mTCP's queries don't depend on UDP NAT timeouts.
 *
 * The ECM bridge serves 192.168.241.1 as the leased nameserver; the DOS
 * host gets it automatically via DHCP.
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Start a forwarder instance bound to @p ip_hostorder:53 (one per
 *  bridge netif IP). */
void dns_forwarder_init_ip(uint32_t ip_hostorder);

uint32_t dns_fwd_stat_queries(void);
uint32_t dns_fwd_stat_resolved(void);
uint32_t dns_fwd_stat_nxdomain(void);

#ifdef __cplusplus
}
#endif

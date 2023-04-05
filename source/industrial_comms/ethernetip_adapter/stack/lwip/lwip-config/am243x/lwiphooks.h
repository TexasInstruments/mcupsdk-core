#ifndef __LWIPHOOKS_H__
#define __LWIPHOOKS_H__

#include "lwip/prot/dhcp.h"
#include "lwip/dhcp.h"

void ethernetif_dhcp_parse_option_hook(struct netif *netif, struct dhcp *dhcp, u8_t state, struct dhcp_msg *msg,
                u8_t msg_type, u8_t option, u8_t option_len, struct pbuf *pbuf, u16_t option_value_offset);

#endif /* __LWIPHOOKS_H__ */

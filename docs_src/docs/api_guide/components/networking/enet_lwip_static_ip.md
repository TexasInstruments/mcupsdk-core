# Ethernet LwIP TCP/IP Static IP{#NETWORKING_LWIP_STATIC_IP}

[TOC]

# Introduction
A network interface (NettIf) in LwIP can only have one IP address assigned to it. There are three methods supported by lwIP for assigning the proper IP address to an interface:

Static IP:  The IP address for the netif can be set either at initialization or afterwards.
DHCP: [DHCP](https://lwip.fandom.com/wiki/DHCP) is an optional protocol to obtain an IP address from a DHCP server.
AUTOIP: [AUTOIP](https://lwip.fandom.com/wiki/AUTOIP) is an optional protocol to pick an IP address on a local subnet without needing a server to assign addresses.

Please follow the below guide to set the up the Static IP.

1. Disable DHCP, by removing calls to `dhcp_start` in the application.
2. Disable AUTOIP, by removing calls to `autoip_start` in the appliction.
3. Call `netif_set_addr()` instead of `dhcp_start` call.  in `App_allocateIPAddress` , as shown below. 

# Details

#### To Configure DHCP
\code
static void App_allocateIPAddress()
{
    sys_lock_tcpip_core();

    for (uint32_t  netifIdx = 0U; netifIdx < ENET_SYSCFG_NETIF_COUNT; netifIdx++)
    {
        dhcp_set_struct(g_pNetif[NETIF_INST_ID0 + i], &g_netifDhcp[NETIF_INST_ID0 + i]);
        const err_t err = dhcp_start(g_pNetif[NETIF_INST_ID0 + i]);
        EnetAppUtils_assert(err == ERR_OK);
    }
    sys_unlock_tcpip_core();
    return;
}
\endcode

#### To Configure Static IP
\code

#define IP_ADDR_POOL_COUNT  (2U)

const ip_addr_t gStaticIP[IP_ADDR_POOL_COUNT]   =  { IPADDR4_INIT_BYTES(192, 168, 1, 200) /* For NetifIdx = 0 */,  IPADDR4_INIT_BYTES(  10,  64,  1, 200) /* For NetifIdx = 1 */};

const ip_addr_t gStaticIPGateway[IP_ADDR_POOL_COUNT] =  { IPADDR4_INIT_BYTES(192, 168, 1, 1) /* For NetifIdx = 0 */, IPADDR4_INIT_BYTES(  10,   64, 1, 1) /* For NetifIdx = 1 */};

const ip_addr_t gStaticIPNetmask[IP_ADDR_POOL_COUNT] =  { IPADDR4_INIT_BYTES(255,255,255,0)  /* For NetifIdx = 0 */, IPADDR4_INIT_BYTES(255,255,252,0) /* For NetifIdx = 1 */};


static void App_allocateIPAddress()
{
    sys_lock_tcpip_core();

    for (uint32_t  netifIdx = 0U; netifIdx < ENET_SYSCFG_NETIF_COUNT; netifIdx++)
    {
        netif_set_addr(g_pNetif[NETIF_INST_ID0 + netifIdx],
                                &gStaticIP[NETIF_INST_ID0 + netifIdx],
                                &gStaticIPNetmask[NETIF_INST_ID0 + netifIdx],
                                &gStaticIPGateway[NETIF_INST_ID0 + netifIdx]);
    }
    sys_unlock_tcpip_core();
    return;
}

\endcode

# See Also

\ref NETWORKING

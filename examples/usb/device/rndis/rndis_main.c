/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *          Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *
 *          Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in the
 *          documentation and/or other materials provided with the
 *          distribution.
 *
 *          Neither the name of Texas Instruments Incorporated nor the names of
 *          its contributors may be used to endorse or promote products derived
 *          from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Adapted by TI for running on its platform and SDK */

#include <ctype.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/nortos/dpl/common/printf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usb/cdn/include/cdn_print.h>
#include <usb/cdn/include/usb_init.h>

#include "dhserver.h"
#include "dnserver.h"
#include "httpd.h"
#include "lwip/apps/lwiperf.h"
#include "lwip/ethip6.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "tusb.h"

#if LWIP_TCP
static void lwiperf_report(void *arg, enum lwiperf_report_type report_type,
                                                     const ip_addr_t *local_addr, u16_t local_port,
                                                     const ip_addr_t *remote_addr, u16_t remote_port,
                                                     u32_t bytes_transferred, u32_t ms_duration,
                                                     u32_t bandwidth_kbitpsec) {
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(local_addr);
    LWIP_UNUSED_ARG(local_port);

    DebugP_log("IPERF report: type=%d, remote: %s:%d, total bytes: %" U32_F
                         ", duration in ms: %" U32_F ", kbits/s: %" U32_F "\n",
                         (int)report_type, ipaddr_ntoa(remote_addr), (int)remote_port,
                         bytes_transferred, ms_duration, bandwidth_kbitpsec);
}
#endif /* LWIP_TCP */

void lwiperf_example_init(void) {
#if LWIP_TCP
    lwiperf_start_tcp_server_default(lwiperf_report, NULL);
#endif
}

#define INIT_IP4(a, b, c, d) \
    { PP_HTONL(LWIP_MAKEU32(a, b, c, d)) }

/* lwip context */
static struct netif netif_data;

/* shared between tud_network_recv_cb() and service_traffic() */
static struct pbuf *received_frame;

/* this is used by this code, ./class/net/net_driver.c, and usb_descriptors.c */
/* ideally speaking, this should be generated from the hardware's unique ID (if
 * available) */
/* it is suggested that the first byte is 0x02 to indicate a link-local address
 */
const uint8_t tud_network_mac_address[6] = {0x02, 0x02, 0x84, 0x6A, 0x96, 0x00};

/* network parameters of this MCU */
static const ip4_addr_t ipaddr = INIT_IP4(192, 168, 7, 1);
static const ip4_addr_t netmask = INIT_IP4(255, 255, 255, 0);
static const ip4_addr_t gateway = INIT_IP4(0, 0, 0, 0);

/* database IP addresses that can be offered to the host; this must be in RAM to
 * store assigned MAC addresses */
static dhcp_entry_t entries[] =
        {
                /* mac ip address lease time */
                {{0}, INIT_IP4(192, 168, 7, 2), 24 * 60 * 60},
                {{0}, INIT_IP4(192, 168, 7, 3), 24 * 60 * 60},
                {{0}, INIT_IP4(192, 168, 7, 4), 24 * 60 * 60},
};

static const dhcp_config_t dhcp_config = {
        .router = INIT_IP4(0, 0, 0, 0),  /* router address (if any) */
        .port = 67,                                          /* listen port */
        .dns = INIT_IP4(192, 168, 7, 1), /* dns server (if any) */
        "usb",                                                   /* dns suffix */
        TU_ARRAY_SIZE(entries),                  /* num entry */
        entries                                                  /* entries */
};

/* this function will transmit buffer on link and is called by ethernet_output()
 * function */
static err_t linkoutput_fn(struct netif *netif, struct pbuf *p) {
    (void)netif;

    for (;;) {
        /* if TinyUSB isn't ready, we must signal back to lwip that there is nothing
         * we can do */
        if (!tud_ready()) return ERR_USE;

        /* if the network driver can accept another packet, we make it happen */
        if (tud_network_can_xmit(p->tot_len)) {
            tud_network_xmit(p, 0 /* unused for this example */);
            return ERR_OK;
        }
        /* transfer execution to TinyUSB in the hopes that it will finish
         * transmitting the prior packet */
        cusbd_dsr();
        tud_task();
    }
}

static err_t ip4_output_fn(struct netif *netif, struct pbuf *p,
                                                     const ip4_addr_t *addr) {
    return etharp_output(netif, p, addr);
}

#if LWIP_IPV6
static err_t ip6_output_fn(struct netif *netif, struct pbuf *p,
                                                     const ip6_addr_t *addr) {
    return ethip6_output(netif, p, addr);
}
#endif

static err_t netif_init_cb(struct netif *netif) {
    LWIP_ASSERT("netif != NULL", (netif != NULL));
    netif->mtu = CFG_TUD_NET_MTU;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP |
                                 NETIF_FLAG_UP;
    netif->state = NULL;
    netif->name[0] = 'E';
    netif->name[1] = 'X';
    netif->linkoutput = linkoutput_fn;
    netif->output = ip4_output_fn;
#if LWIP_IPV6
    netif->output_ip6 = ip6_output_fn;
#endif
    return ERR_OK;
}

static void init_lwip(void) {
    struct netif *netif = &netif_data;

    lwip_init();

    /* the lwip virtual MAC address must be different from the host's; to ensure
     * this, we toggle the LSbit */
    netif->hwaddr_len = sizeof(tud_network_mac_address);
    memcpy(netif->hwaddr, tud_network_mac_address,
                 sizeof(tud_network_mac_address));
    netif->hwaddr[5] ^= 0x01;

    netif = netif_add(netif, &ipaddr, &netmask, &gateway, NULL, netif_init_cb,
                                        ip_input);
#if LWIP_IPV6
    netif_create_ip6_linklocal_address(netif, 1);
#endif
    netif_set_default(netif);
}

/* handle any DNS requests from dns-server */
bool dns_query_proc(const char *name, ip4_addr_t *addr) {
    if (0 == strcmp(name, "tiny.usb")) {
        *addr = ipaddr;
        return true;
    }
    return false;
}

bool tud_network_recv_cb(const uint8_t *src, uint16_t size) {
    /* this shouldn't happen, but if we get another packet before
    parsing the previous, we must signal our inability to accept it */
    if (received_frame) {
        return false;
    }

    if (size) {
        struct pbuf *p = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);
        if (p) {
            /* pbuf_alloc() has already initialized struct; all we need to do is copy
             * the data */
            memcpy(p->payload, src, size);

            /* store away the pointer for service_traffic() to later handle */
            received_frame = p;
        }
    } else {
        received_frame = NULL;
    }

    return true;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg) {
    struct pbuf *p = (struct pbuf *)ref;

    (void)arg; /* unused for this example */

    return pbuf_copy_partial(p, dst, p->tot_len, 0);
}

/* here the ethernet input is the input to stack
 tcp/ip */
static void service_traffic(void) {
    /* handle any packet received by tud_network_recv_cb() */
    if (received_frame) {
        /* loop_forever() ; */
        if (ERR_OK != ethernet_input(received_frame, &netif_data)) {
            pbuf_free(received_frame);
            DebugP_log("Free because of error \r\n");
        }
        received_frame = NULL;
        tud_network_recv_renew();
    }

    sys_check_timeouts();
}

void tud_network_init_cb(void) {
    /* if the network is re-initializing and we have a leftover packet, we must do
     * a cleanup */
    if (received_frame) {
        pbuf_free(received_frame);
        received_frame = NULL;
    }
}

int rndis_main(void) {
    Drivers_open();
    Board_driversOpen();

    /* initialize lwip, dhcp-server, dns-server, and http */
    init_lwip();
    while (!netif_is_up(&netif_data))
        ;
    while (dhserv_init(&dhcp_config) != ERR_OK)
        ;
    while (dnserv_init(IP_ADDR_ANY, 53, dns_query_proc) != ERR_OK)
        ;
    httpd_init();

    while (1) {
        cusbd_dsr();
        tud_task();
        service_traffic();
    }

    return 0;
}

/* lwip has provision for using a mutex, when applicable */
sys_prot_t sys_arch_protect(void) { return 0; }
void sys_arch_unprotect(sys_prot_t pval) { (void)pval; }

/* lwip needs a millisecond time source, and the TinyUSB board support code has
 * one available */
uint32_t sys_now(void) { return (ClockP_getTimeUsec() / 1000); }

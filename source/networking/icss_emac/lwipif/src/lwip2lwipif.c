//###########################################################################
//
// FILE:   lwip2lwipif.c
//
// TITLE:  lwIP Interface port file.
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright: $
//###########################################################################

/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/**
 * Copyright (c) 2018 Texas Instruments Incorporated
 *
 * This file is dervied from the ``ethernetif.c'' skeleton Ethernet network
 * interface driver for lwIP.
 *
 */

/* Standard language headers */
#include <stdio.h>

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>


/**
 * lwIP specific header files
 */
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include <lwip/netifapi.h>
#include "netif/etharp.h"
#include "netif/ppp/pppoe.h"

/* This module's header */
#include "lwip2icss_emac.h"
#include "lwip2lwipif.h"
#include "lwipopts.h"

/**
 * ICSS EMAC specific header files
 */
#include <networking/icss_emac/icss_emac.h>
#include <networking/icss_emac/source/icss_emac_local.h>

/* Define those to better describe your network interface. */
#define IFNAME0 't'
#define IFNAME1 'i'

#define EMACLWIPAPP_POLL_PERIOD      100

#define OS_TASKPRIHIGH               12

#define LWIP_POLL_TASK_PRI           (OS_TASKPRIHIGH)
#define LWIP_POLL_TASK_STACK         (4096)

#define ETH_MAX_PAYLOAD  1514


#define VLAN_TAG_SIZE         (4U)
#define ETH_FRAME_SIZE        (ETH_MAX_PAYLOAD + VLAN_TAG_SIZE)

/*---------------- */
/* Ethernet Header */
#define ETHHDR_SIZE     14

static uint8_t gLwip2LwipIfPollTaskStack[LWIP_POLL_TASK_STACK]
__attribute__ ((aligned(32)));

/*---------------------------------------------------------------------------*\
 |                             Extern Declarations                             |
 \*---------------------------------------------------------------------------*/
extern int32_t Lwip2Emac_sendTxPackets(Lwip2Emac_Handle hLwip2Emac, struct pbuf *p);

/*!
 *  @b LWIPIF_LWIP_send
 *  @n
 *  This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 *  \param[in]  netif
 *      The lwip network interface structure for this ethernetif
 *  \param[in]  p
 *      the MAC packet to send (e.g. IP packet including MAC addresses and type)
 *
 *  \retval
 *      ERR_OK if the packet could be sent
 *  \retval
 *      an err_t value if the packet couldn't be sent
 */
static err_t LWIPIF_LWIP_send(struct netif *netif,
                         struct pbuf *p)
{
    Lwip2Emac_Handle hLwip2Emac;

    /* Get the pointer to the private data */
    hLwip2Emac = (Lwip2Emac_Handle)netif->state;

    /*
     * When transmitting a packet, the buffer may be deleted before transmission by the
     * stack. The stack implements a 'ref' feature within the buffers. The following happens
     * internally:
     *  If p->ref > 1, ref--;
     *  If p->ref == 1, free(p);
     * pbuf_ref(p) increments the ref.
     */
    pbuf_ref(p);

    /* Send the packet from LWIP stack to the translation layer */
    Lwip2Emac_sendTxPackets(hLwip2Emac,p);
    /*Put a check and return proper status a/c to tx*/
    return ERR_OK;
}

/*!
 *  @b LWIPIF_LWIP_input
 *  @n
 *  This is currently a task which consumes the RX packets retrieved from
 *  the driver in RX packet task, and passes them to the LwIP stack via
 *  netif->input().
 *
 *  \param[in]  netif
 *      NETIF_DEVICE structure pointer.
 *
 *  \retval
 *      void
 */
void LWIPIF_LWIP_input(struct netif *netif,
                       struct pbuf *hPbufPacket)
{
    /* Pass the packet to the LwIP stack */
    if (netif->input(hPbufPacket, netif) != ERR_OK)
    {
        /*If packet is not passed to LwIP Stack*/
        LWIP_DEBUGF(NETIF_DEBUG, ("lwipif_input: IP input error\n"));
        pbuf_free(hPbufPacket);
        hPbufPacket = NULL;
    }
    else
    {
        /*Once packet is successfully passed to LwIP Stack this section is executed, Allocates a PBuf for next incoming packet*/
        Lwip2Emac_Handle hLwip2Emac = (Lwip2Emac_Handle)netif->state;
        struct pbuf* hPbufPacket;
        uint32_t bufSize;

        bufSize = PBUF_POOL_BUFSIZE;
        /*Allocating Pbuf for Rx*/
        hPbufPacket = pbuf_alloc(PBUF_RAW, bufSize, PBUF_POOL);
        if (hPbufPacket != NULL)
        {
            hPbufPacket->tot_len = hPbufPacket->len;
            hLwip2Emac->rxPbufPkt = hPbufPacket;
        }
        else
        {
            DebugP_log("[LWIPIF_LWIP]ERROR: Rx Pbuf_alloc() in LWIPIF_LWIP_INPUT failure.!\n");
        }
    }

}

/*
 * Periodically polls for changes in the link status and updates both the abstraction layer
 * as well as the stack
 * arg0 : netif
 * arg1 : Semaphore
 */
static void LWIPIF_LWIP_poll(void *arg0)
{
    /* Call the driver's periodic polling function */
    volatile bool flag = 1;
    struct netif* netif = (struct netif*) arg0;
    Lwip2Emac_Handle hLwip2Emac = (Lwip2Emac_Handle)netif->state;

    while (flag)
    {
        SemaphoreP_Object *hpollSem = (SemaphoreP_Object *)&hLwip2Emac->pollLinkSemObj;
        SemaphoreP_pend(hpollSem, SystemP_WAIT_FOREVER);

        if(arg0 != NULL)
        {
            struct netif* netif = (struct netif*) arg0;

            /* Get the pointer to the private data */
            Lwip2Emac_Handle hLwip2Emac = (Lwip2Emac_Handle)netif->state;

            /* Periodic Function to update Link status */
            Lwip2Emac_periodicFxn(hLwip2Emac);

            if(!(hLwip2Emac->linkIsUp == (netif->flags & 0x04U)>>2))
            {
                if(hLwip2Emac->linkIsUp)
                {
                    sys_lock_tcpip_core();
                    netif_set_link_up(netif);
                    sys_unlock_tcpip_core();
                }

                else
                {
                    sys_lock_tcpip_core();
                    netif_set_link_down(netif);
                    sys_unlock_tcpip_core();
                }
            }
        }
    }
}

static void LWIPIF_LWIP_postPollLink(ClockP_Object *clkObj, void *arg)
{
    if(arg != NULL)
    {
        SemaphoreP_Object *hpollSem = (SemaphoreP_Object *) arg;
        SemaphoreP_post(hpollSem);
    }
}

/*!
 *  @b LWIPIF_LWIP_Start
 *  @n
 *  The function is used to initialize and start the Enet
 *  controller and device.
 *
 *  \param[in]  pNETIFDevice
 *      NETIF_DEVICE structure pointer.
 *
 *  \retval
 *      Success -   0
 *  \retval
 *      Error   -   <0
 */
static int LWIPIF_LWIP_start(struct netif *netif)
{
    int retVal = -1;
    Lwip2Emac_Handle hLwip2Emac;
    TaskP_Params params;
    int32_t status;
    ClockP_Params clkPrms;

    /* Open the translation layer, It also gets hardware driver handle */
    hLwip2Emac = Lwip2Emac_open(netif);

    if (NULL != hLwip2Emac)
    {
        /* Save off a pointer to the translation layer */
        netif->state = (void *)hLwip2Emac;

        /*Initialize semaphore to call synchronize the poll function with a timer*/
        status = SemaphoreP_constructBinary(&hLwip2Emac->pollLinkSemObj, 0U);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Initialize the poll function as a thread */
        TaskP_Params_init(&params);
        params.name = "Lwipif_Lwip_poll";
        params.priority       = LWIP_POLL_TASK_PRI;
        params.stack          = gLwip2LwipIfPollTaskStack;
        params.stackSize      = sizeof(gLwip2LwipIfPollTaskStack);
        params.args           = netif;
        params.taskMain       = &LWIPIF_LWIP_poll;

        status = TaskP_construct(&hLwip2Emac->lwipif2lwipPollTaskObj, &params);
        DebugP_assert(status == SystemP_SUCCESS);

        ClockP_Params_init(&clkPrms);
        clkPrms.start     = 0;
        clkPrms.period    = EMACLWIPAPP_POLL_PERIOD;
        clkPrms.args      = &hLwip2Emac->pollLinkSemObj;
        clkPrms.callback  = &LWIPIF_LWIP_postPollLink;
        clkPrms.timeout   = EMACLWIPAPP_POLL_PERIOD;

        /* Creating timer and setting timer callback function*/
        status = ClockP_construct(&hLwip2Emac->pollLinkClkObj,
                                  &clkPrms);
        if (status == SystemP_SUCCESS)
        {
            /* Set timer expiry time in OS ticks */
            ClockP_setTimeout(&hLwip2Emac->pollLinkClkObj, EMACLWIPAPP_POLL_PERIOD);
            ClockP_start(&hLwip2Emac->pollLinkClkObj);
        }
        else
        {
            DebugP_assert (status == SystemP_SUCCESS);
        }

        /*Copy the MAC Address into the network interface object here. */
        memcpy(netif->hwaddr, (void*)(&(((ICSS_EMAC_Object *)hLwip2Emac->emacHandle->object)->macId[0])), (uint32_t)6U);
        netif->hwaddr_len = 6U;

        /* Filter not defined */
        /* Inform the world that we are operational. */
        DebugP_log("[LWIPIF_LWIP] Interface layer handle is Initialised \r\n");

        retVal = 0;
    }
    else
    {
        DebugP_log("[LWIPIF_LWIP] Failed to initialise Interface layer handle  \r\n");
    }

    return retVal;
}


/*!
 *  @b LWIPIF_LWIP_Stop
 *  @n
 *  The function is used to de-initialize and stop the Enet
 *  controller and device.
 *
 *  \param[in] netif
 *      NETIF structure pointer.
 */
static void LWIPIF_LWIP_stop(struct netif *netif)
{
    Lwip2Emac_Handle hLwip2Emac;

    /* Get the pointer to the private data */
    hLwip2Emac = (Lwip2Emac_Handle)netif->state;

    /* Stop and delete timer */
    ClockP_stop (&hLwip2Emac->pollLinkClkObj);
    ClockP_destruct (&hLwip2Emac->pollLinkClkObj);

    /* Call low-level close function */
    Lwip2Emac_close(hLwip2Emac);

}

/*!
 *  @b LWIPIF_LWIP_Init
 *  @n
 *  The function is used to initialize and register the peripheral
 *  with the stack.
 *
 *  \param[in]  *netif
 *      NETIF structure pointer
 *
 *  \retval
 *      Success -   ERR_OK
 */
err_t LWIPIF_LWIP_init(struct netif *netif)
{
#ifdef LWIPIF_CHECKSUM_SUPPORT
    /* TODO: Add checksum support */
#endif

    /* Populate the Network Interface Object */
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;

    /*
     * MTU is i total size of the (IP) packet that can fit into an Ethernet.
     * For Ethernet it is 1500bytes
     */
    netif->mtu = ETH_FRAME_SIZE - ETHHDR_SIZE - VLAN_TAG_SIZE;

    /* Populate the Driver Interface Functions. */
    netif->remove_callback      = LWIPIF_LWIP_stop;
    netif->output               = etharp_output;
    netif->linkoutput           = LWIPIF_LWIP_send;
    netif->flags               |= NETIF_FLAG_ETHARP;

    LWIPIF_LWIP_start(netif);

    DebugP_log("[LWIPIF_LWIP] NETIF INIT SUCCESS\r\n");

    return ERR_OK;
}


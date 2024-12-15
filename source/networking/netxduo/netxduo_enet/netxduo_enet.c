/*
 *  Copyright (c) Texas Instruments Incorporated 2024
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

/*!
 * \file     netxduo_enet.c
 *
 * \brief    NetxDuo Enet driver.
 */


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include <networking/enet/core/include/enet.h>
#include <networking/enet/core/include/core/enet_per.h>
#include <networking/enet/core/include/core/enet_utils.h>
#include <networking/enet/core/include/core/enet_dma.h>
#include <networking/enet/core/include/common/enet_utils_dflt.h>
#include <drivers/udma/udma_priv.h>
#include <networking/enet/core/include/core/enet_soc.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/EventP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/QueueP.h>

#include <enet_ioctlutils.h>
#include <enet_udmautils.h>
#include <enet_appmemutils.h>
#include <enet_apputils.h>

#include <tx_port.h>
#include <nx_api.h>

#include "netxduo_enet.h"


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define NX_DRIVER_DEFERRED_PACKET_RECEIVED    (1U)
#define NX_DRIVER_DEFERRED_DEVICE_RESET       (2U)
#define NX_DRIVER_DEFERRED_PACKET_TRANSMITTED (4U)

#define NX_DRIVER_ETHERNET_IP     (0x0800U)
#define NX_DRIVER_ETHERNET_IPV6   (0x86ddU)
#define NX_DRIVER_ETHERNET_ARP    (0x0806U)
#define NX_DRIVER_ETHERNET_RARP   (0x8035U)

#define NX_DRIVER_ETHERNET_MTU           (1514U)
#define NX_DRIVER_ETHERNET_FRAME_SIZE    (14U)
#define NX_DRIVER_PHYSICAL_ADDRESS_SIZE  (6U)

#define NX_DRIVER_MAX_RX_CHANNELS        (8u)
#define NX_DRIVER_MAX_TX_CHANNELS        (8u)

#define NX_DRIVER_MAX_INTERFACE_COUNT    (2u)

#define NX_DRIVER_PACKET_SIZE            ENET_UTILS_ALIGN(1536U, ENETDMA_CACHELINE_ALIGNMENT)


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct nx_enet_drv_rx_ch {
    EnetDma_RxChHandle hRxCh;
    EnetQ rxPktQ;
} nx_enet_drv_rx_ch_t;

typedef struct nx_enet_drv_tx_ch {
    EnetDma_TxChHandle hTxCh;
    EnetDma_PktQ freePktQ;
} nx_enet_drv_tx_ch_t;

typedef struct nx_enet_driver_if_data {
    const char *p_if_name;
    NX_IP *netx_ip_ptr;
    NX_INTERFACE *netx_interface_ptr;
    NX_PACKET_POOL *netx_packet_pool_ptr;
    Enet_MacPort macport;
    uint8_t macaddr[ENET_MAC_ADDR_LEN];
    size_t rx_ch_cnt;
    size_t tx_ch_cnt;
    nx_enet_drv_rx_ch_t *t_rx_ch[NX_DRIVER_MAX_RX_CHANNELS];
    nx_enet_drv_tx_ch_t *t_tx_ch[NX_DRIVER_MAX_TX_CHANNELS];
} nx_enet_drv_if_data_t;

typedef struct nx_enet_drv_data {
    ULONG deferred_events_flags;
    nx_enet_drv_rx_ch_t t_rx_ch[NX_DRIVER_MAX_RX_CHANNELS];
    nx_enet_drv_tx_ch_t t_tx_ch[NX_DRIVER_MAX_TX_CHANNELS];
    size_t rx_ch_cnt;
    size_t tx_ch_cnt;
    nx_enet_drv_if_data_t t_if_data[NX_DRIVER_MAX_INTERFACE_COUNT];
    uint32_t if_cnt;
} nx_enet_drv_data_t;


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static nx_enet_drv_data_t g_nx_enet_drv_data = { 0 };


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void nx_enet_drv_initialize(NX_IP_DRIVER *driver_req_ptr);

static void nx_enet_drv_interface_attach(NX_IP_DRIVER *driver_req_ptr);

static void nx_enet_drv_packet_send(NX_IP_DRIVER *driver_req_ptr);

static void nx_enet_drv_deferred_processing(NX_IP_DRIVER *driver_req_ptr);

static void nx_enet_drv_packet_received(NX_IP_DRIVER *driver_req_ptr);

static void nx_enet_drv_packet_transmitted(NX_IP_DRIVER *driver_req_ptr);

static VOID nx_enet_drv_transfer_to_netx(nx_enet_drv_if_data_t *p_if_data, NX_PACKET *packet_ptr);

static void nx_enet_drv_notifyRxPackets(void *cbArg);

static void nx_enet_drv_notifyTxPackets(void *cbArg);

static nx_enet_drv_if_data_t *nx_enet_drv_if_data_get(NX_INTERFACE *interface_ptr);

static void nx_enet_drv_link_enable(NX_IP_DRIVER *driver_req_ptr);

static void nx_enet_drv_link_disable(NX_IP_DRIVER *driver_req_ptr);


/* ========================================================================== */
/*                            Function Definitions                            */
/* ========================================================================== */

void NetxEnetDriver_allocRxCh(EnetDma_RxChHandle hRxCh, uint32_t numPkts, nx_enet_drv_rx_ch_hndl_t *pRxChHandle)
{
    nx_enet_drv_rx_ch_t *pRxCh;
    EnetDma_Pkt *pPkt;
    EnetQ submit_queue;
    uint32_t t_seg_sizes[] = { NX_DRIVER_PACKET_SIZE };
    int32_t ret;

     pRxCh = &g_nx_enet_drv_data.t_rx_ch[g_nx_enet_drv_data.rx_ch_cnt];
     g_nx_enet_drv_data.rx_ch_cnt++;

     pRxCh->hRxCh = hRxCh;

     ret = EnetDma_registerRxEventCb(hRxCh, nx_enet_drv_notifyRxPackets, (void *)pRxCh);
     EnetAppUtils_assert(ret == ENET_SOK);

     EnetQueue_initQ(&pRxCh->rxPktQ);

     /* Initialize the submit queue for RX descriptors. */
     EnetQueue_initQ(&submit_queue);

     /* Allocate netx receive packets and corresponding DMA descriptors. */
     for(size_t k = 0u; k < numPkts; k++) {

         pPkt = EnetMem_allocEthPkt(NULL, sizeof(ULONG), 1u, t_seg_sizes);
         DebugP_assert(pPkt != NULL);

         EnetQueue_enq(&submit_queue, &pPkt->node);
     }

     ret = EnetDma_submitRxPktQ(hRxCh, &submit_queue);
     DebugP_assert(ret == ENET_SOK);

     ret = EnetDma_enableRxEvent(hRxCh);
     DebugP_assert(ret == ENET_SOK);

    *pRxChHandle = pRxCh;
}


void NetxEnetDriver_allocTxCh(EnetDma_TxChHandle hTxCh, uint32_t numPkts, nx_enet_drv_tx_ch_hndl_t *pTxChHandle)
{
    nx_enet_drv_tx_ch_t *pTxCh;
    EnetDma_Pkt *p_pkt;
    uint32_t t_seg_sizes[] = { NX_DRIVER_PACKET_SIZE };
    int32_t ret;


    pTxCh = &g_nx_enet_drv_data.t_tx_ch[g_nx_enet_drv_data.tx_ch_cnt];
    g_nx_enet_drv_data.tx_ch_cnt++;

    pTxCh->hTxCh = hTxCh;

    ret = EnetDma_registerTxEventCb(hTxCh, nx_enet_drv_notifyTxPackets, (void *)pTxCh);
    EnetAppUtils_assert(ret == ENET_SOK);

    EnetQueue_initQ(&pTxCh->freePktQ);

    /* Fill the free TX packet queue. */
    for (size_t k = 0u; k < numPkts; k++) {

        p_pkt = EnetMem_allocEthPkt(NULL, sizeof(ULONG), 1u, t_seg_sizes);
        DebugP_assert(p_pkt != NULL);

        EnetQueue_enq(&pTxCh->freePktQ, &p_pkt->node);
    }

    ret = EnetDma_enableTxEvent(pTxCh->hTxCh);
    DebugP_assert(ret == ENET_SOK);

   *pTxChHandle = pTxCh;
}


void NetxEnetDriver_allocIf(const char *p_name, Enet_MacPort macport, uint8_t macAddr[ENET_MAC_ADDR_LEN],
                            nx_enet_drv_rx_ch_hndl_t hRxChs[], uint32_t numRxCh, nx_enet_drv_tx_ch_hndl_t hTxChs[], uint32_t numTxCh)
{
    nx_enet_drv_if_data_t *p_if_data;

    DebugP_assert(g_nx_enet_drv_data.if_cnt < NX_DRIVER_MAX_INTERFACE_COUNT);

    p_if_data = &g_nx_enet_drv_data.t_if_data[g_nx_enet_drv_data.if_cnt];
    g_nx_enet_drv_data.if_cnt++;

    p_if_data->p_if_name = p_name;
    p_if_data->macport = macport;

    memcpy(&p_if_data->macaddr[0], &macAddr[0], ENET_MAC_ADDR_LEN);

    p_if_data->rx_ch_cnt = 0u;
    for (size_t k = 0u; k < numRxCh; k++) {
        p_if_data->t_rx_ch[k] = hRxChs[k];
        p_if_data->rx_ch_cnt++;
    }

    p_if_data->tx_ch_cnt = 0u;
    for (size_t k = 0u; k < numTxCh; k++) {
        p_if_data->t_tx_ch[k] = hTxChs[k];
        p_if_data->tx_ch_cnt++;
    }
}


void _nx_enet_driver(NX_IP_DRIVER *driver_req_ptr)
{
    /* Verify that driver_req_ptr is not NULL. */
    DebugP_assert(driver_req_ptr != NULL);

    /* Initialize the return value to success. */
    driver_req_ptr->nx_ip_driver_status = NX_SUCCESS;

    /* Process the command from NETX. */
    switch (driver_req_ptr->nx_ip_driver_command) {

        case NX_LINK_INTERFACE_ATTACH:
            nx_enet_drv_interface_attach(driver_req_ptr);
            break;

        case NX_LINK_INITIALIZE:
            nx_enet_drv_initialize(driver_req_ptr);
            break;

        case NX_LINK_ENABLE:
             nx_enet_drv_link_enable(driver_req_ptr);
             break;

        case NX_LINK_DISABLE:
             nx_enet_drv_link_disable(driver_req_ptr);
             break;

        case NX_LINK_ARP_SEND:
        case NX_LINK_ARP_RESPONSE_SEND:
        case NX_LINK_PACKET_BROADCAST:
        case NX_LINK_RARP_SEND:
        case NX_LINK_PACKET_SEND:
            nx_enet_drv_packet_send(driver_req_ptr);
            break;

        case NX_LINK_MULTICAST_JOIN:
        case NX_LINK_MULTICAST_LEAVE:
        case NX_LINK_GET_STATUS:
        case NX_LINK_SET_PHYSICAL_ADDRESS:
            driver_req_ptr->nx_ip_driver_status = NX_UNHANDLED_COMMAND;
            break;

        case NX_LINK_DEFERRED_PROCESSING:
            nx_enet_drv_deferred_processing(driver_req_ptr);
            break;

        case NX_LINK_USER_COMMAND:
            driver_req_ptr->nx_ip_driver_status = NX_UNHANDLED_COMMAND;
            break;

        default:
            driver_req_ptr->nx_ip_driver_status = NX_UNHANDLED_COMMAND;
    }
}


static void nx_enet_drv_initialize(NX_IP_DRIVER *driver_req_ptr)
{
    NX_IP *ip_ptr;
    nx_enet_drv_if_data_t *p_if_data;
    NX_INTERFACE *interface_ptr;

    p_if_data = nx_enet_drv_if_data_get(driver_req_ptr->nx_ip_driver_interface);
    DebugP_assert(p_if_data != NULL);


    DebugP_assert(driver_req_ptr->nx_ip_driver_interface != NULL);
    DebugP_assert(driver_req_ptr->nx_ip_driver_ptr != NULL);
    DebugP_assert(driver_req_ptr->nx_ip_driver_ptr->nx_ip_default_packet_pool != NULL);


    /* Fetch the NETX IP instance and NETX interface. */
    ip_ptr = driver_req_ptr->nx_ip_driver_ptr;
    interface_ptr = p_if_data->netx_interface_ptr;


    /* Save the packet pool pointer. */
    p_if_data->netx_packet_pool_ptr = ip_ptr->nx_ip_default_packet_pool;

    /* Save the IP instance pointer. */
    p_if_data->netx_ip_ptr = ip_ptr;

    /* Save the MTU size. */
    interface_ptr->nx_interface_ip_mtu_size = NX_DRIVER_ETHERNET_MTU;

    /* Save phyiscal address. */
    interface_ptr->nx_interface_physical_address_msw = (ULONG)((p_if_data->macaddr[0] << 8) | (p_if_data->macaddr[1]));
    interface_ptr->nx_interface_physical_address_lsw = (ULONG)((p_if_data->macaddr[2] << 24) | (p_if_data->macaddr[3] << 16) |
                                                               (p_if_data->macaddr[4] << 8) | (p_if_data->macaddr[5]));

    /* Indicate to NETX that address mapping is required. */
    interface_ptr->nx_interface_address_mapping_needed = NX_TRUE;
    driver_req_ptr->nx_ip_driver_status = NX_SUCCESS;
}


static void nx_enet_drv_interface_attach(NX_IP_DRIVER *driver_req_ptr)
{
    nx_enet_drv_if_data_t *p_if_data;

    DebugP_assert(driver_req_ptr->nx_ip_driver_interface != NULL);

    p_if_data = NULL;
    for (size_t k = 0u; k < g_nx_enet_drv_data.if_cnt; k++) {
        if (strcmp(driver_req_ptr->nx_ip_driver_interface->nx_interface_name, g_nx_enet_drv_data.t_if_data[k].p_if_name) == 0) {
            p_if_data = &g_nx_enet_drv_data.t_if_data[k];
            break;
        }
    }

    DebugP_assert(p_if_data != NULL);

    /* Save the NETX interface instance associated with this driver instance. */
    p_if_data->netx_interface_ptr = driver_req_ptr->nx_ip_driver_interface;

    /* Set return status to success. */
    driver_req_ptr->nx_ip_driver_status = NX_SUCCESS;
}



static void nx_enet_drv_packet_send(NX_IP_DRIVER *driver_req_ptr)
{
    EnetDma_Pkt *p_pkt;
    NX_PACKET *p_nx_packet;
    nx_enet_drv_if_data_t *p_if_data;
    EnetQ submit_queue;
    ULONG *p_frame;
    UINT status;
    int32_t ret;


    driver_req_ptr->nx_ip_driver_status =  NX_SUCCESS;

    /* Place the Ethernet frame at the front of the packet. */
    p_nx_packet = driver_req_ptr->nx_ip_driver_packet;

    /* Adjust the prepend pointer and packet length. */
    p_nx_packet->nx_packet_prepend_ptr = p_nx_packet->nx_packet_prepend_ptr - NX_DRIVER_ETHERNET_FRAME_SIZE;

    p_nx_packet->nx_packet_length = p_nx_packet->nx_packet_length + NX_DRIVER_ETHERNET_FRAME_SIZE;

    /* Setup the Ethernet frame pointer to build the Ethernet frame. */
    p_frame = (ULONG *) (p_nx_packet->nx_packet_prepend_ptr - 2);

    /* Write the hardware addresses in the Ethernet header. */
    *p_frame = driver_req_ptr->nx_ip_driver_physical_address_msw;
    *(p_frame + 1) = driver_req_ptr->nx_ip_driver_physical_address_lsw;

    *(p_frame + 2) = (driver_req_ptr->nx_ip_driver_interface->nx_interface_physical_address_msw << 16) |
        (driver_req_ptr->nx_ip_driver_interface->nx_interface_physical_address_lsw >> 16);
    *(p_frame + 3) = (driver_req_ptr->nx_ip_driver_interface->nx_interface_physical_address_lsw << 16);

    /* Write the frame type field in the Ethernet harder. */
    if((driver_req_ptr->nx_ip_driver_command == NX_LINK_ARP_SEND) || (driver_req_ptr->nx_ip_driver_command == NX_LINK_ARP_RESPONSE_SEND))
    {
        *(p_frame + 3) |= NX_DRIVER_ETHERNET_ARP;
    }
    else if(driver_req_ptr->nx_ip_driver_command == NX_LINK_RARP_SEND)
    {
        *(p_frame + 3) |= NX_DRIVER_ETHERNET_RARP;
    }
#ifdef FEATURE_NX_IPV6
    else if(p_nx_packet->nx_packet_ip_version == NX_IP_VERSION_V6)
    {
        *(p_frame + 3) |= NX_DRIVER_ETHERNET_IPV6;
    }
#endif
    else
    {
        *(p_frame + 3) |= NX_DRIVER_ETHERNET_IP;
    }

    /* Endian swapping if NX_LITTLE_ENDIAN is defined. */
    NX_CHANGE_ULONG_ENDIAN(*(p_frame));
    NX_CHANGE_ULONG_ENDIAN(*(p_frame + 1));
    NX_CHANGE_ULONG_ENDIAN(*(p_frame + 2));
    NX_CHANGE_ULONG_ENDIAN(*(p_frame + 3));

    /* Determine if the packet exceeds the driver's MTU. */
    if(p_nx_packet->nx_packet_length > NX_DRIVER_ETHERNET_MTU) {

        /* Remove the Ethernet header. */
        p_nx_packet->nx_packet_prepend_ptr += NX_DRIVER_ETHERNET_FRAME_SIZE;
        p_nx_packet->nx_packet_length -= NX_DRIVER_ETHERNET_FRAME_SIZE;

        /* Indicate an unsuccessful packet send. */
        driver_req_ptr->nx_ip_driver_status =  NX_DRIVER_ERROR;

        status = nx_packet_transmit_release(p_nx_packet);
        DebugP_assert(status == NX_SUCCESS);
        return;
    }

    p_if_data = nx_enet_drv_if_data_get(driver_req_ptr->nx_ip_driver_interface);
    DebugP_assert(p_if_data != NULL);


    p_pkt = (EnetDma_Pkt *)EnetQueue_deq(&p_if_data->t_tx_ch[0]->freePktQ);
    DebugP_assert(p_pkt != NULL);


    p_pkt->sgList.numScatterSegments = 1;
    memcpy(p_pkt->sgList.list[0].bufPtr, p_nx_packet->nx_packet_prepend_ptr, p_nx_packet->nx_packet_length);
    p_pkt->sgList.list[0].disableCacheOps = false;
    p_pkt->sgList.list[0].segmentAllocLen = p_nx_packet->nx_packet_length;
    p_pkt->sgList.list[0].segmentFilledLen = p_nx_packet->nx_packet_length;

    p_pkt->appPriv    = (void *)p_nx_packet;
    p_pkt->txPortNum  = ENET_MAC_PORT_INV;
    p_pkt->node.next  = NULL;
    p_pkt->chkSumInfo = 0u;


    EnetQueue_initQ(&submit_queue);
    EnetQueue_enq(&submit_queue, &p_pkt->node);


    ret = EnetDma_submitTxPktQ(p_if_data->t_tx_ch[0]->hTxCh, &submit_queue);
    if (ret != ENET_SOK) {

        /* Remove the Ethernet header. */
        p_nx_packet->nx_packet_prepend_ptr += NX_DRIVER_ETHERNET_FRAME_SIZE;
        p_nx_packet->nx_packet_length -= NX_DRIVER_ETHERNET_FRAME_SIZE;

        /* Indicate an unsuccessful packet send. */
        driver_req_ptr->nx_ip_driver_status = NX_DRIVER_ERROR;

        /* Release the packet. */
        status = nx_packet_transmit_release(p_nx_packet);
        DebugP_assert(status == NX_SUCCESS);
    }
}


static void nx_enet_drv_packet_transmitted(NX_IP_DRIVER *driver_req_ptr)
{
    nx_enet_drv_if_data_t *p_if_data;
    EnetDma_Pkt *p_cur_pkt;
    NX_PACKET *p_nx_packet;
    EnetDma_PktQ retrieved_queue;
    int32_t ret;

    p_if_data = nx_enet_drv_if_data_get(driver_req_ptr->nx_ip_driver_interface);
    DebugP_assert(p_if_data != NULL);

    EnetQueue_initQ(&retrieved_queue);

    /* Retrieve all TX packets and keep them locally */
    ret = EnetDma_retrieveTxPktQ(p_if_data->t_tx_ch[0]->hTxCh, &retrieved_queue);
    if (ENET_SOK != ret) {
        DebugP_log("nx_enet_drv_packet_transmitted: Failed to retrieve TX pkts: %d\n", ret);
    }

    p_cur_pkt = (EnetDma_Pkt *)EnetQueue_deq(&retrieved_queue);

    while (p_cur_pkt != NULL) {

        p_nx_packet = (NX_PACKET *)p_cur_pkt->appPriv;
        DebugP_assert(p_nx_packet != NULL);


        /* Remove the Ethernet header. */
        p_nx_packet->nx_packet_prepend_ptr += NX_DRIVER_ETHERNET_FRAME_SIZE;
        p_nx_packet->nx_packet_length -= NX_DRIVER_ETHERNET_FRAME_SIZE;

        nx_packet_transmit_release(p_nx_packet);

        /* Return packet info to free pool */
        EnetQueue_enq(&p_if_data->t_tx_ch[0]->freePktQ, &p_cur_pkt->node);

        p_cur_pkt = (EnetDma_Pkt *)EnetQueue_deq(&retrieved_queue);
    }
}



static void nx_enet_drv_deferred_processing(NX_IP_DRIVER *driver_req_ptr)
{
    nx_enet_drv_if_data_t *p_if_data;
    ULONG deferred_event_flags;
    TX_INTERRUPT_SAVE_AREA


    p_if_data = nx_enet_drv_if_data_get(driver_req_ptr->nx_ip_driver_interface);
    DebugP_assert(p_if_data != NULL);

    TX_DISABLE

    /* Get the deferred event flags and clear them. */
    deferred_event_flags = g_nx_enet_drv_data.deferred_events_flags;
    g_nx_enet_drv_data.deferred_events_flags = 0u;

    TX_RESTORE


    if((deferred_event_flags & NX_DRIVER_DEFERRED_PACKET_TRANSMITTED) != 0u) {
        /* Process transmitted packets. Errors, if any, will be returned through driver_req_ptr->nx_ip_driver_status. */
        nx_enet_drv_packet_transmitted(driver_req_ptr);
    }

    if((deferred_event_flags & NX_DRIVER_DEFERRED_PACKET_RECEIVED) != 0u) {
        /* Process received packets. Errors, if any, will be returned through driver_req_ptr->nx_ip_driver_status. */
        nx_enet_drv_packet_received(driver_req_ptr);
    }
}


static void nx_enet_drv_packet_received(NX_IP_DRIVER *driver_req_ptr)
{
    nx_enet_drv_if_data_t *p_if_data;
    NX_PACKET *p_nx_packet;
    EnetDma_Pkt *p_cur_pkt;
    EnetDma_PktQ submit_queue;
    EnetDma_PktQ retrieved_queue;
    EnetDma_PktQ skip_queue;
    int32_t ret;
    UINT status;


    for (size_t ch_ix = 0u; ch_ix < g_nx_enet_drv_data.rx_ch_cnt; ch_ix++) {

        EnetQueue_initQ(&submit_queue);
        EnetQueue_initQ(&skip_queue);

        ret = EnetDma_retrieveRxPktQ(g_nx_enet_drv_data.t_rx_ch[ch_ix].hRxCh, &retrieved_queue);
        DebugP_assert(ret == ENET_SOK);

        if (EnetQueue_getQCount(&retrieved_queue) == 0) {
            continue;
        }

        EnetQueue_append(&g_nx_enet_drv_data.t_rx_ch[ch_ix].rxPktQ, &retrieved_queue);

        p_cur_pkt = (EnetDma_Pkt *)EnetQueue_deq(&g_nx_enet_drv_data.t_rx_ch[ch_ix].rxPktQ);
        while (p_cur_pkt != NULL) {

            p_if_data = NULL;
            for (size_t if_ix = 0u; if_ix < g_nx_enet_drv_data.if_cnt; if_ix++) {
                if (g_nx_enet_drv_data.t_if_data[if_ix].macport == ENET_MAC_PORT_INV) {
                    DebugP_assert(g_nx_enet_drv_data.if_cnt == 1u);
                    p_if_data = &g_nx_enet_drv_data.t_if_data[if_ix];
                    break;
                }
                if (g_nx_enet_drv_data.t_if_data[if_ix].macport == p_cur_pkt->rxPortNum) {
                    p_if_data = &g_nx_enet_drv_data.t_if_data[if_ix];
                    break;
                }
            }
            if (p_if_data == NULL) {
                p_cur_pkt = (EnetDma_Pkt *)EnetQueue_deq(&g_nx_enet_drv_data.t_rx_ch[ch_ix].rxPktQ);
                continue;
            }
            if (p_if_data->netx_ip_ptr == NULL) {
                p_cur_pkt = (EnetDma_Pkt *)EnetQueue_deq(&g_nx_enet_drv_data.t_rx_ch[ch_ix].rxPktQ);
                continue;
            }

            DebugP_assert(p_cur_pkt->sgList.numScatterSegments <= ENET_ARRAYSIZE(p_cur_pkt->sgList.list));
            DebugP_assert(p_cur_pkt->sgList.numScatterSegments != 0);

            EnetDma_checkPktState(&p_cur_pkt->pktState, ENET_PKTSTATE_MODULE_APP, ENET_PKTSTATE_APP_WITH_DRIVER, ENET_PKTSTATE_APP_WITH_FREEQ);

            status = nx_packet_allocate(p_if_data->netx_packet_pool_ptr, &p_nx_packet, NX_RECEIVE_PACKET, NX_NO_WAIT);
            if(status != NX_SUCCESS) {
                if(status == NX_NO_PACKET) {
                    /* No packet buffer available. Reset desc and exit. */
                    EnetQueue_enq(&submit_queue, &p_cur_pkt->node);
                    break;
                } else {
                    /* Unexpected error return. */
                    DebugP_assert(false);
                    driver_req_ptr->nx_ip_driver_status = NX_DRIVER_ERROR;
                    return;
                }
            }

            p_nx_packet->nx_packet_length = p_cur_pkt->sgList.list[0].segmentFilledLen;
            p_nx_packet->nx_packet_prepend_ptr += 2u;
            p_nx_packet->nx_packet_append_ptr = p_nx_packet->nx_packet_prepend_ptr + p_nx_packet->nx_packet_length;
            memcpy(p_nx_packet->nx_packet_prepend_ptr, p_cur_pkt->sgList.list[0].bufPtr, p_cur_pkt->sgList.list[0].segmentFilledLen);
            nx_enet_drv_transfer_to_netx(p_if_data, p_nx_packet);

            EnetQueue_enq(&submit_queue, &p_cur_pkt->node);
            p_cur_pkt = (EnetDma_Pkt *)EnetQueue_deq(&g_nx_enet_drv_data.t_rx_ch[ch_ix].rxPktQ);
        }

        if (EnetQueue_getQCount(&skip_queue) > 0u) {
            EnetQueue_append(&g_nx_enet_drv_data.t_rx_ch[ch_ix].rxPktQ, &skip_queue);
        }

        if (EnetQueue_getQCount(&submit_queue) > 0u) {
            ret = EnetDma_submitRxPktQ(g_nx_enet_drv_data.t_rx_ch[ch_ix].hRxCh, &submit_queue);
            DebugP_assert(ret == ENET_SOK);
            DebugP_assert(EnetQueue_getQCount(&submit_queue) == 0u);
        }
    }
}


static void nx_enet_drv_notifyRxPackets(void *cbArg)
{
    nx_enet_drv_rx_ch_t *p_rx_ch = (nx_enet_drv_rx_ch_t *)cbArg;

    g_nx_enet_drv_data.deferred_events_flags |= NX_DRIVER_DEFERRED_PACKET_RECEIVED;

    for (size_t if_ix = 0u; if_ix < NX_DRIVER_MAX_INTERFACE_COUNT; if_ix++) {
        for (size_t ch_ix = 0u; ch_ix < g_nx_enet_drv_data.t_if_data[if_ix].rx_ch_cnt; ch_ix++) {
            if (g_nx_enet_drv_data.t_if_data[if_ix].t_rx_ch[ch_ix] == p_rx_ch) {

                /* Make sure the ip was created, otherwise drop the packet. */
                if (g_nx_enet_drv_data.t_if_data[if_ix].netx_ip_ptr != NULL) {

                    /* Signal to NETX that there are received packets to process. */
                    _nx_ip_driver_deferred_processing(g_nx_enet_drv_data.t_if_data[if_ix].netx_ip_ptr);
                }
            }
        }
    }
}


static void nx_enet_drv_notifyTxPackets(void *cbArg)
{
    nx_enet_drv_tx_ch_t *p_tx_ch = (nx_enet_drv_tx_ch_t *)cbArg;

    g_nx_enet_drv_data.deferred_events_flags |= NX_DRIVER_DEFERRED_PACKET_TRANSMITTED;

    for (size_t if_ix = 0u; if_ix < NX_DRIVER_MAX_INTERFACE_COUNT; if_ix++) {
        for (size_t ch_ix = 0u; ch_ix < g_nx_enet_drv_data.t_if_data[if_ix].tx_ch_cnt; ch_ix++) {
            if (g_nx_enet_drv_data.t_if_data[if_ix].t_tx_ch[ch_ix] == p_tx_ch) {

                /* Should not be transmitting before the ip was created. */
                DebugP_assert(g_nx_enet_drv_data.t_if_data[if_ix].netx_ip_ptr != NULL);

                _nx_ip_driver_deferred_processing(g_nx_enet_drv_data.t_if_data[if_ix].netx_ip_ptr);
            }
        }
    }
}


static VOID nx_enet_drv_transfer_to_netx(nx_enet_drv_if_data_t *p_if_data, NX_PACKET *packet_ptr)
{
    USHORT packet_type;

    /* Verify that the packet buffer pointers are within bound. */
    DebugP_assert(packet_ptr->nx_packet_prepend_ptr <= packet_ptr->nx_packet_data_end);
    DebugP_assert(packet_ptr->nx_packet_prepend_ptr >= packet_ptr->nx_packet_data_start);

    packet_ptr->nx_packet_ip_interface = p_if_data->netx_interface_ptr;

    /* Pickup the packet header to determine where the packet needs to be sent. */
    packet_type = (USHORT)(((UINT) (*(packet_ptr->nx_packet_prepend_ptr + 12))) << 8) |
                    ((UINT) (*(packet_ptr->nx_packet_prepend_ptr + 13)));

    /* Route the incoming packet according to its Ethernet type. */
    if(packet_type == NX_DRIVER_ETHERNET_IP || packet_type == NX_DRIVER_ETHERNET_IPV6)
    {
        /* Remove the Ethernet header. */
        packet_ptr->nx_packet_prepend_ptr = packet_ptr->nx_packet_prepend_ptr + NX_DRIVER_ETHERNET_FRAME_SIZE;

        packet_ptr->nx_packet_length = packet_ptr->nx_packet_length - NX_DRIVER_ETHERNET_FRAME_SIZE;

        /* Route to the ip receive function. */
        _nx_ip_packet_deferred_receive(p_if_data->netx_ip_ptr, packet_ptr);
    }
    else if(packet_type == NX_DRIVER_ETHERNET_ARP)
    {

        /* Clean off the Ethernet header. */
        packet_ptr->nx_packet_prepend_ptr = packet_ptr->nx_packet_prepend_ptr + NX_DRIVER_ETHERNET_FRAME_SIZE;

        /* Adjust the packet length. */
        packet_ptr->nx_packet_length = packet_ptr->nx_packet_length - NX_DRIVER_ETHERNET_FRAME_SIZE;

        /* Route to the ARP receive function. */
        _nx_arp_packet_deferred_receive(p_if_data->netx_ip_ptr, packet_ptr);
    }
    else if(packet_type == NX_DRIVER_ETHERNET_RARP)
    {

        /* Clean off the Ethernet header. */
        packet_ptr->nx_packet_prepend_ptr = packet_ptr->nx_packet_prepend_ptr + NX_DRIVER_ETHERNET_FRAME_SIZE;

        /* Adjust the packet length. */
        packet_ptr->nx_packet_length = packet_ptr->nx_packet_length - NX_DRIVER_ETHERNET_FRAME_SIZE;

        /* Route to the RARP receive function. */
        _nx_rarp_packet_deferred_receive(p_if_data->netx_ip_ptr, packet_ptr);
    }
    else
    {
        /* Invalid Ethernet header, just release the packet. */
        nx_packet_release(packet_ptr);
    }
}


static nx_enet_drv_if_data_t *nx_enet_drv_if_data_get(NX_INTERFACE *interface_ptr)
{
    for (size_t k = 0u; k < g_nx_enet_drv_data.if_cnt; k++) {
        if (g_nx_enet_drv_data.t_if_data[k].netx_interface_ptr == interface_ptr) {
            return (&g_nx_enet_drv_data.t_if_data[k]);
        }
    }
    return (NULL);
}

static void nx_enet_drv_link_enable(NX_IP_DRIVER *driver_req_ptr)
{
    driver_req_ptr->nx_ip_driver_interface->nx_interface_link_up = true;
}

static void nx_enet_drv_link_disable(NX_IP_DRIVER *driver_req_ptr)
{
    driver_req_ptr->nx_ip_driver_interface->nx_interface_link_up = false;
}





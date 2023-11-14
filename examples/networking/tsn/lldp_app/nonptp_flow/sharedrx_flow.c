/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <tsn_combase/combase.h>
#include "dataflow.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define ETH_P_IPV4 (0x0800U)

/* ========================================================================== */
/*                                Function Declarations                       */
/* ========================================================================== */
extern EnetApp_Cfg gEnetAppCfg;
static uint8_t gIpv4TaskStackRx[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static TaskP_Object g_Ipv4RxTaskObj;
static SemaphoreP_Object gIpv4RxSemObj;

void rxDefaultDataCb(void *data, int size, int port, void *cbArg)
{
    if (size > 14) {
        uint16_t ethtype = ntohs(*(uint16_t *)((uint8_t *)data + 12));
        EnetAppUtils_print("%s:pkt ethtype=0x%x, size=%d, port=%d\r\n",
                           __func__, ethtype, size, port);
    } else {
        EnetAppUtils_print("%s:pkt size=%d, port=%d\r\n",
                           __func__, size, port);
    }
}

int EnetApp_lldCfgUpdateCb(cb_socket_lldcfg_update_t *update_cfg)
{
    int res = 0;
    if (update_cfg->proto == ETH_P_1588)
    {
        update_cfg->dmaTxChId = ENET_DMA_TX_CH_PTP;
        update_cfg->dmaRxChId = ENET_DMA_RX_CH_PTP;
        update_cfg->nTxPkts = ENET_DMA_TX_CH_PTP_NUM_PKTS;
        update_cfg->nRxPkts = ENET_DMA_RX_CH_PTP_NUM_PKTS;
        update_cfg->pktSize = ENET_MEM_LARGE_POOL_PKT_SIZE;
        /* We make the PTP RX DMA channel is shared betweens multiples app,
         * gptp2d apps is owner of this channel.
         * The rxDefaultDataCb is called inside gptp task when the packet
         * does not match any filters on this RX DMA channel.
         */
        update_cfg->dmaRxShared = true;
        update_cfg->dmaRxOwner = true;
        update_cfg->rxDefaultDataCb = rxDefaultDataCb;
        update_cfg->rxDefaultCbArg = NULL;
    }
    else if (update_cfg->proto == ETH_P_LLDP)
    {
        update_cfg->dmaTxChId = ENET_DMA_TX_CH_LLDP;
        update_cfg->nTxPkts = ENET_DMA_TX_CH_LLDP_NUM_PKTS;
        update_cfg->pktSize = ENET_MEM_LARGE_POOL_PKT_SIZE;

        /* share the RX Dma channel with PTP */
        update_cfg->dmaRxChId = ENET_DMA_RX_CH_PTP;
        update_cfg->dmaRxOwner = false; /* gptp is RX dma owner */

        // This CB is call while cb_raw_socket_open is called.
        // Delay some time in here to ensure GPTP is opened DMA.
        CB_USLEEP(1000000); //1s
    }
    else if (update_cfg->proto == ETH_P_NETLINK)
    {
        update_cfg->unusedDmaRx = true;
        update_cfg->unusedDmaTx = true;
    }
    else if (update_cfg->proto == ETH_P_IPV4)
    {
        /* share the RX Dma channel with PTP */
        update_cfg->dmaRxChId = ENET_DMA_RX_CH_PTP;
        update_cfg->dmaRxOwner = false; /* gptp is RX dma owner */
        update_cfg->unusedDmaTx = true;
    }
    else
    {
        EnetAppUtils_print("%s:unsupported other than PTP\r\n", __func__);
        res = -1;
    }
    return res;
}

static void rxNotifyCb(void *cbArg)
{
    SemaphoreP_post(cbArg);
}

static void EthIpv4RxTask(void *args)
{
    cb_rawsock_paras_t llrawp;
    CB_SOCKET_T lldsock;
    ub_macaddr_t bmac;
    uint8_t buf[1518];
    int port;
    int res;

    /* The RX DMA channel is shared with gptp, we need to make sure the
     * gptp is already done its DMA initialization before reusing that channel.
     * Some delays is needed otherwise it will fail to create socket */
    ClockP_usleep(3000000);
    EnetAppUtils_print("%s: RX task for IPv4 running\r\n", __func__);

    (void)memset(&llrawp, 0, sizeof(llrawp));
    llrawp.dev = "tilld0";
    llrawp.proto = ETH_P_IPV4;
    llrawp.vlan_proto = 0;
    llrawp.vlanid = 0;
    llrawp.rw_type = CB_RAWSOCK_RDWR;
    if(cb_rawsock_open(&llrawp, &lldsock, NULL, NULL, bmac) < 0)
    {
        EnetAppUtils_print("%s:failed to open raw socket\r\n", __func__);
        return;
    }
    cb_lld_set_rxnotify_cb(lldsock, rxNotifyCb, &gIpv4RxSemObj);

    EnetAppUtils_print("%s:Started IPv4 task\r\n", __func__);

    while (true)
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&gIpv4RxSemObj, SystemP_WAIT_FOREVER);
        res = cb_lld_recv(lldsock, buf, sizeof(buf), &port);
        if (res <= 0)
        {
            EnetAppUtils_print("%s:failed to recv pkt: res=%d\r\n",
                               __func__, res);
        }
        else
        {
            if (res > 14)
            {
                uint8_t *p = buf;
                uint16_t ethtype = ntohs(*(uint16_t *)(p + 12));
                EnetAppUtils_print("%s:pkt ethtype=0x%x, size=%d, port=%d\r\n",
                                __func__, ethtype, res, port);
            }
            else
            {
                EnetAppUtils_print("%s:pkt size=%d, port=%d\r\n",
                                   __func__, res, port);
            }
        }
    }
}

void EnetApp_createRxTask()
{
    TaskP_Params taskParams;
    int32_t status;
    EnetApp_GetMacAddrOutArgs outArgs;

    /* get the source mac address */
    EnetApp_getMacAddress(ENET_DMA_RX_CH_PTP, &outArgs);
    if (outArgs.macAddressCnt > 0)
    {
        EnetUtils_copyMacAddr(gEnetAppCfg.macAddr, outArgs.macAddr[0]);
    }

    status = SemaphoreP_constructBinary(&gIpv4RxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 2U;
    taskParams.stack          = gIpv4TaskStackRx;
    taskParams.stackSize      = sizeof(gIpv4TaskStackRx);
    taskParams.args           = NULL;
    taskParams.name           = "Rx Ipv4 Task";
    taskParams.taskMain       = &EthIpv4RxTask;

    status = TaskP_construct(&g_Ipv4RxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

void EnetApp_destroyRxTask()
{
    SemaphoreP_destruct(&gIpv4RxSemObj);
    TaskP_destruct(&g_Ipv4RxTaskObj);
}

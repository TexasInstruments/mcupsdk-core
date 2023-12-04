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
#include <nrt_flow/dataflow.h>


int EnetApp_lldCfgUpdateCb(cb_socket_lldcfg_update_t *update_cfg)
{
    if (update_cfg->proto == ETH_P_1588)
    {
        update_cfg->numRxChannels = 1;
        update_cfg->dmaTxChId = ENET_DMA_TX_CH_PTP;
        update_cfg->dmaRxChId[0] = ENET_DMA_RX_CH0;
        update_cfg->nTxPkts = ENET_DMA_TX_CH_PTP_NUM_PKTS;
        update_cfg->nRxPkts[0] = ENET_DMA_RX_CH0_NUM_PKTS;
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
    else if (update_cfg->proto == ETH_P_IPV4)
    {
        update_cfg->numRxChannels = 1;
        /* share the RX Dma channel with PTP */
        update_cfg->dmaRxChId[0] = ENET_DMA_RX_CH0;
        update_cfg->dmaRxOwner = false; /* gptp is RX dma owner */
        update_cfg->unusedDmaTx = true;
    }
    return 0;
}

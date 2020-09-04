/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 * \file  enet_udmautils.c
 *
 * \brief This file contains the implementation of the Enet UDMA utils functions.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>

#include <enet.h>
#include <enet_cfg.h>
#include <include/per/cpsw.h>
#if (ENET_ENABLE_PER_ICSSG == 1)
#include <include/per/icssg.h>
#include <priv/per/icssg_ioctl_priv.h>
#endif
#include <include/core/enet_dma.h>
#include "include/enet_appmemutils.h"
#include "include/enet_appmemutils_cfg.h"

#include "include/enet_apputils.h"
// #include "include/enet_appboardutils.h"

#include "include/enet_appsoc.h"
#include "include/enet_apprm.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetAppUtils_freePktInfoQ(EnetDma_PktQ *pPktInfoQ)
{
    EnetDma_Pkt *pktInfo;
    uint32_t pktCnt, i;

    pktCnt = EnetQueue_getQCount(pPktInfoQ);
    /* Free all retrieved packets from DMA */
    for (i = 0U; i < pktCnt; i++)
    {
        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(pPktInfoQ);
        EnetMem_freeEthPkt(pktInfo);
    }
}

int32_t EnetAppUtils_regDfltRxFlow(Enet_Handle hEnet,
                                   uint32_t coreKey,
                                   uint32_t coreId,
                                   uint32_t rxFlowStartIdx,
                                   uint32_t rxFlowIdx)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    Enet_DfltFlowInfo dfltFlow;

    dfltFlow.coreKey  = coreKey;
    dfltFlow.chIdx    = 0U;
    dfltFlow.startIdx = rxFlowStartIdx;
    dfltFlow.flowIdx  = rxFlowIdx;
    ENET_IOCTL_SET_IN_ARGS(&prms, &dfltFlow);
    ENET_IOCTL(hEnet,
               coreId,
               ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW,
               &prms,
               status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("EnetAppUtils_regDfltRxFlow() failed : %d\n", status);
    }

    return status;
}

int32_t EnetAppUtils_unregDfltRxFlow(Enet_Handle hEnet,
                                     uint32_t coreKey,
                                     uint32_t coreId,
                                     uint32_t rxFlowStartIdx,
                                     uint32_t rxFlowIdx)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    Enet_DfltFlowInfo dfltFlow;

    dfltFlow.coreKey  = coreKey;
    dfltFlow.chIdx    = 0U;
    dfltFlow.startIdx = rxFlowStartIdx;
    dfltFlow.flowIdx  = rxFlowIdx;

    ENET_IOCTL_SET_IN_ARGS(&prms, &dfltFlow);
    ENET_IOCTL(hEnet,
               coreId,
               ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW,
               &prms,
               status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("EnetAppUtils_unregDfltRxFlow() failed : %d\n", status);
    }

    return status;
}

int32_t EnetAppUtils_regDfltRxFlowForChIdx(Enet_Handle hEnet,
                                           uint32_t coreKey,
                                           uint32_t coreId,
                                           uint32_t chIdx,
                                           uint32_t rxFlowStartIdx,
                                           uint32_t rxFlowIdx)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    Enet_DfltFlowInfo dfltFlow;

    dfltFlow.coreKey  = coreKey;
    dfltFlow.chIdx    = chIdx;
    dfltFlow.startIdx = rxFlowStartIdx;
    dfltFlow.flowIdx  = rxFlowIdx;

    ENET_IOCTL_SET_IN_ARGS(&prms, &dfltFlow);
    ENET_IOCTL(hEnet,
               coreId,
               ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW,
               &prms,
               status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("EnetAppUtils_regDfltRxFlowForChIdx() failed : %d\n", status);
    }

    return status;
}

int32_t EnetAppUtils_unregDfltRxFlowForChIdx(Enet_Handle hEnet,
                                             uint32_t coreKey,
                                             uint32_t coreId,
                                             uint32_t chIdx,
                                             uint32_t rxFlowStartIdx,
                                             uint32_t rxFlowIdx)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    Enet_DfltFlowInfo dfltFlow;

    dfltFlow.coreKey  = coreKey;
    dfltFlow.chIdx    = chIdx;
    dfltFlow.startIdx = rxFlowStartIdx;
    dfltFlow.flowIdx  = rxFlowIdx;

    ENET_IOCTL_SET_IN_ARGS(&prms, &dfltFlow);
    ENET_IOCTL(hEnet,
               coreId,
               ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW,
               &prms,
               status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("EnetAppUtils_unregDfltRxFlowForChIdx() failed : %d\n", status);
    }

    return status;
}

int32_t EnetAppUtils_regDstMacRxFlow(Enet_Handle hEnet,
                                          uint32_t coreKey,
                                          uint32_t coreId,
                                          uint32_t rxFlowStartIdx,
                                          uint32_t rxFlowIdx,
                                          uint8_t macAddress[ENET_MAC_ADDR_LEN])
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    Enet_MacDstFlowInfo macDstFlow;

    macDstFlow.coreKey  = coreKey;
    macDstFlow.startIdx = rxFlowStartIdx;
    macDstFlow.flowIdx  = rxFlowIdx;
    EnetUtils_copyMacAddr(macDstFlow.macAddress, macAddress);
    ENET_IOCTL_SET_IN_ARGS(&prms, &macDstFlow);
    ENET_IOCTL(hEnet,
               coreId,
               ENET_IOCTL_REGISTER_DSTMAC_RX_FLOW,
               &prms,
               status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("EnetAppUtils_regDstMacRxFlow() failed : %d\r\n", status);
    }

    return status;
}

#if (ENET_ENABLE_PER_CPSW == 1)
int32_t EnetAppUtils_unregDstMacRxFlow(Enet_Handle hEnet,
                                            uint32_t coreKey,
                                            uint32_t coreId,
                                            uint32_t rxFlowStartIdx,
                                            uint32_t rxFlowIdx,
                                            uint8_t macAddress[ENET_MAC_ADDR_LEN])
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    Enet_MacDstFlowInfo macDstFlow;

    macDstFlow.coreKey  = coreKey;
    macDstFlow.startIdx = rxFlowStartIdx;
    macDstFlow.flowIdx  = rxFlowIdx;
    EnetUtils_copyMacAddr(macDstFlow.macAddress, macAddress);
    ENET_IOCTL_SET_IN_ARGS(&prms, &macDstFlow);
    ENET_IOCTL(hEnet,
               coreId,
               ENET_IOCTL_UNREGISTER_DSTMAC_RX_FLOW,
               &prms,
               status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("EnetAppUtils_regDstMacRxFlow() failed : %d\r\n", status);
    }

    return status;
}
#endif

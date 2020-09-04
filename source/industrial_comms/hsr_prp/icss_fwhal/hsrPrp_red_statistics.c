/**
 * \file hsrPrp_red_statistics.c
 * \brief Contains HSR PRP Statistics interface routines
 *
 * \par
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 * \par
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <networking/icss_emac/icss_emac.h>
#include <networking/icss_emac/source/icss_emac_local.h>
#include "hsrPrp_red_statistics.h"
#include "hsrPrp_firmwareOffsets.h"
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

RED_STATUS RedGetStatistics(RED_STATISTICS *pStatisitcs,
                            ICSS_EMAC_Handle icssEmacHandle)
{
    PRUICSS_Handle              pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const       *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    if(pStatisitcs == NULL)
    {
        return (RED_ERR);
    }


    pStatisitcs->cntTxA          = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_TX_A));

    pStatisitcs->cntTxB          = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_TX_B));

    pStatisitcs->cntTxC          = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_TX_C));

    pStatisitcs->cntErrWrongLanA = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_ERRWRONGLAN_A));

    pStatisitcs->cntErrWrongLanB = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_ERRWRONGLAN_B));

    pStatisitcs->cntErrWrongLanC = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_ERRWRONGLAN_C));

    pStatisitcs->cntRxA          = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_RX_A));

    pStatisitcs->cntRxB          = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_RX_B));

    pStatisitcs->cntRxC          = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_RX_C));

    pStatisitcs->cntErrorsA      = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_ERRORS_A));

    pStatisitcs->cntErrorsB      = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_ERRORS_B));

    pStatisitcs->cntErrorsC      = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_ERRORS_C));

    pStatisitcs->cntNodes        = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_NODES));

    pStatisitcs->cntProxyNodes   = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_PROXY_NODES));

    pStatisitcs->cntUniqueA      = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_UNIQUE_RX_A));

    pStatisitcs->cntUniqueB      = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_UNIQUE_RX_B));

    pStatisitcs->cntUniqueC      = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_UNIQUE_RX_C));

    pStatisitcs->cntDuplicateA   = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_DUPLICATE_RX_A));

    pStatisitcs->cntDuplicateB   = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_DUPLICATE_RX_B));

    pStatisitcs->cntDuplicateC   = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_DUPLICATE_RX_C));

    pStatisitcs->cntMultiA       = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_MULTIPLE_RX_A));

    pStatisitcs->cntMultiB       = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_MULTIPLE_RX_B));

    pStatisitcs->cntMultiC       = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_MULTIPLE_RX_C));

    pStatisitcs->cntOwnRxA       = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_OWN_RX_A));

    pStatisitcs->cntOwnRxB       = *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                                     LRE_CNT_OWN_RX_B));

#ifdef RED_STATS_DBG
    ICSS_EMAC_PruStatistics pruStats_1;
    ICSS_EMAC_PruStatistics pruStats_2;

    ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_STAT_CTRL_GET, ICSS_EMAC_PORT_1, (void *)(&pruStats_1));
    ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_STAT_CTRL_GET, ICSS_EMAC_PORT_2, (void *)(&pruStats_2));

    pStatisitcs->tx_bc_frames    = pruStats_1.txBcast;
    pStatisitcs->TX_MC_FRAMES    = pruStats_1.txMcast;
    pStatisitcs->TX_UC_FRAMES    = pruStats_1.txUcast;
    pStatisitcs->TX_BYTE_CNT     = pruStats_1.txOctets;
    pStatisitcs->RX_BC_FRAMES    = pruStats_1.rxBcast;
    pStatisitcs->RX_MC_FRAMES    = pruStats_1.rxMcast;
    pStatisitcs->RX_UC_FRAMES    = pruStats_1.rxUcast;

    pStatisitcs->TXB_BC_FRAMES   = pruStats_2.txBcast;
    pStatisitcs->TXB_MC_FRAMES   = pruStats_2.txMcast;
    pStatisitcs->TXB_UC_FRAMES   = pruStats_2.txUcast;
    pStatisitcs->TXB_BYTE_CNT    = pruStats_2.txOctets;
    pStatisitcs->RXB_BC_FRAMES   = pruStats_2.rxBcast;
    pStatisitcs->RXB_MC_FRAMES   = pruStats_2.rxMcast;
    pStatisitcs->RXB_UC_FRAMES   = pruStats_2.rxUcast;

    pStatisitcs->NODE_TABLE_INSERTION_ERROR = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_NODE_TABLE_INSERTION_ERROR));
    pStatisitcs->RXA_OVERFLOW               = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_RXA_OVERFLOW));
    pStatisitcs->RXB_OVERFLOW               = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_RXB_OVERFLOW));
    pStatisitcs->RXA_FWD_OVERFLOW           = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_RXA_FWD_OVERFLOW));
    pStatisitcs->RXB_FWD_OVERFLOW           = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_RXB_FWD_OVERFLOW));
    pStatisitcs->RXA_FAILACQU_QUEUE         = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_RXA_FAILACQU_QUEUE));
    pStatisitcs->RXB_FAILACQU_QUEUE         = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_RXB_FAILACQU_QUEUE));
    pStatisitcs->RXA_FWD_FAILACQU_QUEUE     = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_RXA_FWD_FAILACQU_QUEUE));
    pStatisitcs->RXB_FWD_FAILACQU_QUEUE     = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_RXB_FWD_FAILACQU_QUEUE));
    pStatisitcs->DEBUG_1                    = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_DEBUG_1));
    pStatisitcs->DEBUG_2                    = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_DEBUG_2));
    pStatisitcs->DEBUG_3                    = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_DEBUG_3));
    pStatisitcs->DEBUG_4                    = HW_RD_REG32((uint32_t *)((pruicssHwAttrs->pru0DramBase) +
            DBG_DEBUG_4));
#endif /* RED_STATS_DBG */

    return (RED_OK);
}

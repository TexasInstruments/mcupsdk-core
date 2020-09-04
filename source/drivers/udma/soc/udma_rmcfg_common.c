/*
 *  Copyright 2020-2021 (C) Texas Instruments Incorporated
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

/**
 *  \file udma_rmcfg_common.c
 *
 *  \brief File containing the UDMA driver RM init parameters configuration
 *  functions common to all SOC.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/udma/udma_priv.h>

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

static uint32_t Udma_getCoreSciDevId(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t UdmaRmInitPrms_init(uint32_t instId, Udma_RmInitPrms *rmInitPrms)
{
    const Udma_RmDefBoardCfgPrms                *rmDefBoardCfgPrms;
    int32_t                                      retVal = UDMA_SOK;
    Udma_RmDefBoardCfgResp                       rmDefBoardCfgResp[UDMA_RM_NUM_RES];
    uint32_t                                     splitResFlag[UDMA_RM_NUM_RES] = {0U};
    uint32_t                                     numRes = 0U;
    uint32_t                                     resIdx;

    /* Error check */
    if(NULL_PTR == rmInitPrms)
    {
        retVal = UDMA_EBADARGS;
    }

    if(UDMA_SOK == retVal)
    {
        rmDefBoardCfgPrms = Udma_rmGetDefBoardCfgPrms(instId);

        memset(rmDefBoardCfgResp, 0, sizeof(rmDefBoardCfgResp));
        memset(rmInitPrms, 0, sizeof(*rmInitPrms));

        if(UDMA_INST_ID_BCDMA_0 == instId)
        {
            numRes = UDMA_RM_NUM_BCDMA_RES;
        }
        else if(UDMA_INST_ID_PKTDMA_0 == instId)
        {
            numRes = UDMA_RM_NUM_PKTDMA_RES;
        }

        for(resIdx = 0U; resIdx < numRes; resIdx++)
        {
            /* Query all the resources range from Sciclient Default BoardCfg */
            retVal += Udma_rmGetSciclientDefaultBoardCfgRmRange(&rmDefBoardCfgPrms[resIdx], &rmDefBoardCfgResp[resIdx], &splitResFlag[resIdx]);
        }

        if((UDMA_INST_ID_BCDMA_0 == instId) || (UDMA_INST_ID_PKTDMA_0 == instId))
        {
            /* Ultra High Capacity Block Copy Channels */
            rmInitPrms->startBlkCopyUhcCh   = rmDefBoardCfgResp[UDMA_RM_RES_ID_BC_UHC].rangeStart;
            rmInitPrms->numBlkCopyUhcCh     = rmDefBoardCfgResp[UDMA_RM_RES_ID_BC_UHC].rangeNum;

            /* High Capacity Block Copy Channels */
            rmInitPrms->startBlkCopyHcCh    = rmDefBoardCfgResp[UDMA_RM_RES_ID_BC_HC].rangeStart;
            rmInitPrms->numBlkCopyHcCh      = rmDefBoardCfgResp[UDMA_RM_RES_ID_BC_HC].rangeNum;

            /* Normal Capacity Block Copy Channels */
            rmInitPrms->startBlkCopyCh      = rmDefBoardCfgResp[UDMA_RM_RES_ID_BC].rangeStart;
            rmInitPrms->numBlkCopyCh        = rmDefBoardCfgResp[UDMA_RM_RES_ID_BC].rangeNum;

            /* Ultra High Capacity TX Channels */
            rmInitPrms->startTxUhcCh        = rmDefBoardCfgResp[UDMA_RM_RES_ID_TX_UHC].rangeStart;
            rmInitPrms->numTxUhcCh          = rmDefBoardCfgResp[UDMA_RM_RES_ID_TX_UHC].rangeNum;

            /* High Capacity TX Channels */
            rmInitPrms->startTxHcCh         = rmDefBoardCfgResp[UDMA_RM_RES_ID_TX_HC].rangeStart;
            rmInitPrms->numTxHcCh           = rmDefBoardCfgResp[UDMA_RM_RES_ID_TX_HC].rangeNum;

            /* Normal Capacity TX Channels */
            rmInitPrms->startTxCh           = rmDefBoardCfgResp[UDMA_RM_RES_ID_TX].rangeStart;
            rmInitPrms->numTxCh             = rmDefBoardCfgResp[UDMA_RM_RES_ID_TX].rangeNum;

            /* Ultra High Capacity RX Channels */
            rmInitPrms->startRxUhcCh        = rmDefBoardCfgResp[UDMA_RM_RES_ID_RX_UHC].rangeStart;
            rmInitPrms->numRxUhcCh          = rmDefBoardCfgResp[UDMA_RM_RES_ID_RX_UHC].rangeNum;

             /* High Capacity RX Channels */
            rmInitPrms->startRxHcCh         = rmDefBoardCfgResp[UDMA_RM_RES_ID_RX_HC].rangeStart;
            rmInitPrms->numRxHcCh           = rmDefBoardCfgResp[UDMA_RM_RES_ID_RX_HC].rangeNum;

            /* Normal Capacity RX Channels */
            rmInitPrms->startRxCh           = rmDefBoardCfgResp[UDMA_RM_RES_ID_RX].rangeStart;
            rmInitPrms->numRxCh             = rmDefBoardCfgResp[UDMA_RM_RES_ID_RX].rangeNum;
        }

        /* Global Event */
        /* Shared resource - Split based on instance */
        retVal += Udma_rmSetSharedResRmInitPrms(Udma_rmGetSharedResPrms(UDMA_RM_RES_ID_GLOBAL_EVENT),
                                               instId,
                                               rmDefBoardCfgResp[UDMA_RM_RES_ID_GLOBAL_EVENT].rangeStart,
                                               rmDefBoardCfgResp[UDMA_RM_RES_ID_GLOBAL_EVENT].rangeNum,
                                               &rmInitPrms->startGlobalEvent,
                                               &rmInitPrms->numGlobalEvent);

        /* Virtual Interrupts */
        /* Shared resource - Split based on instance */
        retVal += Udma_rmSetSharedResRmInitPrms(Udma_rmGetSharedResPrms(UDMA_RM_RES_ID_VINTR),
                                               instId,
                                               rmDefBoardCfgResp[UDMA_RM_RES_ID_VINTR].rangeStart,
                                               rmDefBoardCfgResp[UDMA_RM_RES_ID_VINTR].rangeNum,
                                               &rmInitPrms->startVintr,
                                               &rmInitPrms->numVintr);

        if((UDMA_INST_ID_BCDMA_0 == instId) || (UDMA_INST_ID_PKTDMA_0 == instId))
        {
            /* One to one mapping exists from Virtual Interrupts.
             * So translate to corresponding range.
             * In case of devices like AM64x, where there are no Interrupt Routers,
             * this refers to core interrupt itslef. */
            retVal += Sciclient_rmIrqTranslateIaOutput(rmDefBoardCfgPrms[UDMA_RM_RES_ID_VINTR].sciclientReqType,
                                                       rmInitPrms->startVintr,
                                                       Udma_getCoreSciDevId(),
                                                       (uint16_t *) &rmInitPrms->startIrIntr);

            rmInitPrms->numIrIntr                          = rmInitPrms->numVintr;
        }

        if(UDMA_INST_ID_PKTDMA_0 == instId)
        {
        #if (UDMA_NUM_MAPPED_TX_GROUP > 0)

            /* Mapped TX Channels for CPSW */
            rmInitPrms->startMappedTxCh[UDMA_MAPPED_TX_GROUP_CPSW]     = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_CPSW].rangeStart;
            rmInitPrms->numMappedTxCh[UDMA_MAPPED_TX_GROUP_CPSW]       = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_CPSW].rangeNum;

            /* Mapped TX Channels for SAUL */
            if(0U != rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_SAUL_0].rangeNum)
            {
                rmInitPrms->startMappedTxCh[UDMA_MAPPED_TX_GROUP_SAUL] = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_SAUL_0].rangeStart;
            }
            else
            {
                rmInitPrms->startMappedTxCh[UDMA_MAPPED_TX_GROUP_SAUL] = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_SAUL_1].rangeStart;
            }
            rmInitPrms->numMappedTxCh[UDMA_MAPPED_TX_GROUP_SAUL]       = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_SAUL_0].rangeNum +
                                                                         rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_SAUL_1].rangeNum;

            /* Mapped TX Channels for ICSSG_0 */
            rmInitPrms->startMappedTxCh[UDMA_MAPPED_TX_GROUP_ICSSG_0]  = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_ICSSG_0].rangeStart;
            rmInitPrms->numMappedTxCh[UDMA_MAPPED_TX_GROUP_ICSSG_0]    = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_ICSSG_0].rangeNum;

            /* Mapped TX Channels for ICSSG_1 */
            rmInitPrms->startMappedTxCh[UDMA_MAPPED_TX_GROUP_ICSSG_1]  = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_ICSSG_1].rangeStart;
            rmInitPrms->numMappedTxCh[UDMA_MAPPED_TX_GROUP_ICSSG_1]    = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_ICSSG_1].rangeNum;
        #endif
        #if (UDMA_NUM_MAPPED_RX_GROUP > 0)

            /* Mapped RX Channels for CPSW */
            rmInitPrms->startMappedRxCh[UDMA_MAPPED_RX_GROUP_CPSW - UDMA_NUM_MAPPED_TX_GROUP]     = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_CPSW].rangeStart;
            rmInitPrms->numMappedRxCh[UDMA_MAPPED_RX_GROUP_CPSW - UDMA_NUM_MAPPED_TX_GROUP]       = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_CPSW].rangeNum;

            /* Mapped RX Channels for SAUL */
            if(0U != rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_SAUL_0].rangeNum)
            {
                rmInitPrms->startMappedRxCh[UDMA_MAPPED_RX_GROUP_SAUL - UDMA_NUM_MAPPED_TX_GROUP] = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_SAUL_0].rangeStart;
            }
            else
            {
                rmInitPrms->startMappedRxCh[UDMA_MAPPED_RX_GROUP_SAUL - UDMA_NUM_MAPPED_TX_GROUP] = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_SAUL_2].rangeStart;
            }
            rmInitPrms->numMappedRxCh[UDMA_MAPPED_RX_GROUP_SAUL - UDMA_NUM_MAPPED_TX_GROUP]       = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_SAUL_0].rangeNum +
                                                                                                    rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_SAUL_1].rangeNum +
                                                                                                    rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_SAUL_2].rangeNum +
                                                                                                    rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_SAUL_3].rangeNum;


            /* Mapped RX Channels for ICSSG_0 */
            rmInitPrms->startMappedRxCh[UDMA_MAPPED_RX_GROUP_ICSSG_0 - UDMA_NUM_MAPPED_TX_GROUP]  = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_ICSSG_0].rangeStart;
            rmInitPrms->numMappedRxCh[UDMA_MAPPED_RX_GROUP_ICSSG_0 - UDMA_NUM_MAPPED_TX_GROUP]    = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_ICSSG_0].rangeNum;

            /* Mapped RX Channels for ICSSG_1 */
            rmInitPrms->startMappedRxCh[UDMA_MAPPED_RX_GROUP_ICSSG_1 - UDMA_NUM_MAPPED_TX_GROUP]  = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_ICSSG_1].rangeStart;
            rmInitPrms->numMappedRxCh[UDMA_MAPPED_RX_GROUP_ICSSG_1 - UDMA_NUM_MAPPED_TX_GROUP]    = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_ICSSG_1].rangeNum;
        #endif
        #if ((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)

            /* Mapped TX Rings for CPSW */
            rmInitPrms->startMappedRing[UDMA_MAPPED_TX_GROUP_CPSW]     = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_CPSW].rangeStart;
            rmInitPrms->numMappedRing[UDMA_MAPPED_TX_GROUP_CPSW]       = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_CPSW].rangeNum;

            /* Mapped TX Rings for SAUL */
          /*
            Temp disabled until SYSFW-4170 is properly fixed and Sysconfig Tool is updated.
            (SAUL_RX_0_CHAN,SAUL_RX_1_CHAN is reserved is SYSFW, But the corresponding RX Rings are not reserved.)

            if(0U != rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_SAUL_0].rangeNum)
            {
                rmInitPrms->startMappedRing[UDMA_MAPPED_TX_GROUP_SAUL] = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_SAUL_0].rangeStart;
            }
            else
            {
                rmInitPrms->startMappedRing[UDMA_MAPPED_TX_GROUP_SAUL] = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_SAUL_1].rangeStart;
            }
            rmInitPrms->numMappedRing[UDMA_MAPPED_TX_GROUP_SAUL]       = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_SAUL_0].rangeNum +
                                                                         rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_SAUL_1].rangeNum;
          */
            rmInitPrms->startMappedRing[UDMA_MAPPED_TX_GROUP_SAUL]     = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_SAUL_1].rangeStart;
            rmInitPrms->numMappedRing[UDMA_MAPPED_TX_GROUP_SAUL]       = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_SAUL_1].rangeNum;

            /* Mapped TX Rings for ICSSG_0 */
            rmInitPrms->startMappedRing[UDMA_MAPPED_TX_GROUP_ICSSG_0]  = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_ICSSG_0].rangeStart;
            rmInitPrms->numMappedRing[UDMA_MAPPED_TX_GROUP_ICSSG_0]    = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_ICSSG_0].rangeNum;

            /* Mapped TX Rings for ICSSG_1 */
            rmInitPrms->startMappedRing[UDMA_MAPPED_TX_GROUP_ICSSG_1]  = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_ICSSG_1].rangeStart;
            rmInitPrms->numMappedRing[UDMA_MAPPED_TX_GROUP_ICSSG_1]    = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_TX_RING_ICSSG_1].rangeNum;

            /* Mapped RX Rings for CPSW */
            rmInitPrms->startMappedRing[UDMA_MAPPED_RX_GROUP_CPSW]     = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_CPSW].rangeStart;
            rmInitPrms->numMappedRing[UDMA_MAPPED_RX_GROUP_CPSW]       = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_CPSW].rangeNum;

            /* Mapped RX Rings for SAUL */
          /*
            Temp disabled until SYSFW-4170 is properly fixed and Sysconfig Tool is updated.
            (SAUL_RX_0_CHAN,SAUL_RX_1_CHAN is reserved is SYSFW, But the corresponding RX Rings are not reserved.)

            if(0U != rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_SAUL_0].rangeNum)
            {
                rmInitPrms->startMappedRing[UDMA_MAPPED_RX_GROUP_SAUL] = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_SAUL_0].rangeStart;
            }
            else
            {
                rmInitPrms->startMappedRing[UDMA_MAPPED_RX_GROUP_SAUL] = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_SAUL_2].rangeStart;
            }
            rmInitPrms->numMappedRing[UDMA_MAPPED_RX_GROUP_SAUL]       = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_SAUL_0].rangeNum +
                                                                         rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_SAUL_2].rangeNum;
          */
            rmInitPrms->startMappedRing[UDMA_MAPPED_RX_GROUP_SAUL]     = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_SAUL_2].rangeStart;
            rmInitPrms->numMappedRing[UDMA_MAPPED_RX_GROUP_SAUL]       = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_SAUL_2].rangeNum;

            /* Mapped RX Rings for ICSSG_0 */
            rmInitPrms->startMappedRing[UDMA_MAPPED_RX_GROUP_ICSSG_0]  = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_ICSSG_0].rangeStart;
            rmInitPrms->numMappedRing[UDMA_MAPPED_RX_GROUP_ICSSG_0]    = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_ICSSG_0].rangeNum;

            /* Mapped RX Rings for ICSSG_1 */
            rmInitPrms->startMappedRing[UDMA_MAPPED_RX_GROUP_ICSSG_1]  = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_ICSSG_1].rangeStart;
            rmInitPrms->numMappedRing[UDMA_MAPPED_RX_GROUP_ICSSG_1]    = rmDefBoardCfgResp[UDMA_RM_RES_ID_MAPPED_RX_RING_ICSSG_1].rangeNum;
        #endif
        }
    }

    return (retVal);
}

static uint32_t Udma_getCoreSciDevId(void)
{
    uint32_t devId;

    devId = Sciclient_getSelfDevIdCore();

    if (devId == TISCI_DEV_A53SS0_CORE_0)
    {
        /* Return device ID for GIC as interrupts to A53 core is routed through GIC */
        devId = TISCI_DEV_GICSS0;
    }

    return devId;
}

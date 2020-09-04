/*
 *  Copyright (C) 2020 Texas Instruments Incorporated
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
 *  \file udma_soc.c
 *
 *  \brief File containing the UDMA driver SOC related configuration functions.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/udma/udma_priv.h>
#include <drivers/hw_include/cslr_soc.h>

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

const Udma_MappedChRingAttributes gUdmaTxMappedChRingAttributes[CSL_DMSS_PKTDMA_NUM_TX_CHANS - CSL_DMSS_PKTDMA_TX_CHANS_UNMAPPED_CNT] =
{
    /* defaultRing, startFreeRing, numFreeRing */
    {16U, 17U, 7U}, /* Channel 16 - UDMA_MAPPED_TX_GROUP_CPSW Ch 0 */
    {24U, 25U, 7U}, /* Channel 17 - UDMA_MAPPED_TX_GROUP_CPSW Ch 1 */
    {32U, 33U, 7U}, /* Channel 18 - UDMA_MAPPED_TX_GROUP_CPSW Ch 2 */
    {40U, 41U, 7U}, /* Channel 19 - UDMA_MAPPED_TX_GROUP_CPSW Ch 3 */
    {48U, 49U, 7U}, /* Channel 20 - UDMA_MAPPED_TX_GROUP_CPSW Ch 4 */
    {56U, 57U, 7U}, /* Channel 21 - UDMA_MAPPED_TX_GROUP_CPSW Ch 5 */
    {64U, 65U, 7U}, /* Channel 22 - UDMA_MAPPED_TX_GROUP_CPSW Ch 6 */
    {72U, 73U, 7U}, /* Channel 23 - UDMA_MAPPED_TX_GROUP_CPSW Ch 7 */
    {80U, 81U, 7U}, /* Channel 24 - UDMA_MAPPED_TX_GROUP_SAUL Ch 0  */
    {88U, 89U, 7U}, /* Channel 25 - UDMA_MAPPED_TX_GROUP_SAUL Ch 1 */
    {96U, 0U, 0U}, /* Channel 26 - UDMA_MAPPED_TX_GROUP_ICSSG_0 Ch 0 */
    {97U, 0U, 0U}, /* Channel 27 - UDMA_MAPPED_TX_GROUP_ICSSG_0 Ch 1 */
    {98U, 0U, 0U}, /* Channel 28 - UDMA_MAPPED_TX_GROUP_ICSSG_0 Ch 2 */
    {99U, 0U, 0U}, /* Channel 29 - UDMA_MAPPED_TX_GROUP_ICSSG_0 Ch 3 */
    {100U, 0U, 0U}, /* Channel 30 - UDMA_MAPPED_TX_GROUP_ICSSG_0 Ch 4 */
    {101U, 0U, 0U}, /* Channel 31 - UDMA_MAPPED_TX_GROUP_ICSSG_0 Ch 5 */
    {102U, 0U, 0U}, /* Channel 32 - UDMA_MAPPED_TX_GROUP_ICSSG_0 Ch 6 */
    {103U, 0U, 0U}, /* Channel 33 - UDMA_MAPPED_TX_GROUP_ICSSG_0 Ch 7 */
    {104U, 0U, 0U}, /* Channel 34 - UDMA_MAPPED_TX_GROUP_ICSSG_1 Ch 0 */
    {105U, 0U, 0U}, /* Channel 35 - UDMA_MAPPED_TX_GROUP_ICSSG_1 Ch 1 */
    {106U, 0U, 0U}, /* Channel 36 - UDMA_MAPPED_TX_GROUP_ICSSG_1 Ch 2 */
    {107U, 0U, 0U}, /* Channel 37 - UDMA_MAPPED_TX_GROUP_ICSSG_1 Ch 3 */
    {108U, 0U, 0U}, /* Channel 38 - UDMA_MAPPED_TX_GROUP_ICSSG_1 Ch 4 */
    {109U, 0U, 0U}, /* Channel 39 - UDMA_MAPPED_TX_GROUP_ICSSG_1 Ch 5 */
    {110U, 0U, 0U}, /* Channel 40 - UDMA_MAPPED_TX_GROUP_ICSSG_1 Ch 6 */
    {111U, 0U, 0U}, /* Channel 41 - UDMA_MAPPED_TX_GROUP_ICSSG_1 Ch 7 */
};

const Udma_MappedChRingAttributes gUdmaRxMappedChRingAttributes[CSL_DMSS_PKTDMA_NUM_RX_CHANS - CSL_DMSS_PKTDMA_RX_CHANS_UNMAPPED_CNT] =
{
    /* defaultRing, startFreeRing, numFreeRing */
    /*RX Ring Offset of 112U added to the startRing */
    {128U, 129U, 15U}, /* Channel 16 - UDMA_MAPPED_RX_GROUP_CPSW Ch 0 */
    {144U, 146U, 6U}, /* Channel 17 - UDMA_MAPPED_RX_GROUP_SAUL Ch 0 */
    {145U, 146U, 6U}, /* Channel 18 - UDMA_MAPPED_RX_GROUP_SAUL Ch 1 */
    {152U, 154U, 6U}, /* Channel 19 - UDMA_MAPPED_RX_GROUP_SAUL Ch 2 */
    {153U, 154U, 6U}, /* Channel 20 - UDMA_MAPPED_RX_GROUP_SAUL Ch 3 */
    {160U, 161U, 15U}, /* Channel 21 - UDMA_MAPPED_RX_GROUP_ICSSG_0 Ch 0 */
    {176U, 177U, 15U}, /* Channel 22 - UDMA_MAPPED_RX_GROUP_ICSSG_0 Ch 1 */
    {192U, 193U, 15U}, /* Channel 23 - UDMA_MAPPED_RX_GROUP_ICSSG_0 Ch 2 */
    {208U, 209U, 15U}, /* Channel 24 - UDMA_MAPPED_RX_GROUP_ICSSG_0 Ch 3  */
    {224U, 225U, 15U}, /* Channel 25 - UDMA_MAPPED_RX_GROUP_ICSSG_1 Ch 0 */
    {240U, 241U, 15U}, /* Channel 26 - UDMA_MAPPED_RX_GROUP_ICSSG_1 Ch 1 */
    {256U, 257U, 15U}, /* Channel 27 - UDMA_MAPPED_RX_GROUP_ICSSG_1 Ch 2 */
    {272U, 273U, 15U}, /* Channel 28 - UDMA_MAPPED_RX_GROUP_ICSSG_1 Ch 3 */
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


void Udma_initDrvHandle(Udma_DrvHandleInt drvHandle)
{
    uint32_t                 instId;
    CSL_BcdmaCfg             *pBcdmaRegs;
    CSL_PktdmaCfg            *pPktdmaRegs;
    CSL_LcdmaRingaccCfg      *pLcdmaRaRegs;
    CSL_IntaggrCfg           *pIaRegs;

    instId = drvHandle->initPrms.instId;
    /*
     * BCDMA/PKTDMA config init
     */
    /* Init the config structure - one time step */
    if(UDMA_INST_ID_BCDMA_0 == instId)
    {
        drvHandle->instType = UDMA_INST_TYPE_LCDMA_BCDMA;
        pBcdmaRegs = &drvHandle->bcdmaRegs;
        pBcdmaRegs->pGenCfgRegs     = ((CSL_bcdma_gcfgRegs *) CSL_DMASS0_BCDMA_GCFG_BASE);
        pBcdmaRegs->pBcChanCfgRegs  = ((CSL_bcdma_bccfgRegs *) CSL_DMASS0_BCDMA_BCHAN_BASE);
        pBcdmaRegs->pTxChanCfgRegs  = ((CSL_bcdma_txccfgRegs *) CSL_DMASS0_BCDMA_TCHAN_BASE);
        pBcdmaRegs->pRxChanCfgRegs  = ((CSL_bcdma_rxccfgRegs *) CSL_DMASS0_BCDMA_RCHAN_BASE);
        pBcdmaRegs->pBcChanRtRegs   = ((CSL_bcdma_bcrtRegs *) CSL_DMASS0_BCDMA_BCHANRT_BASE);
        pBcdmaRegs->pTxChanRtRegs   = ((CSL_bcdma_txcrtRegs *) CSL_DMASS0_BCDMA_TCHANRT_BASE);
        pBcdmaRegs->pRxChanRtRegs   = ((CSL_bcdma_rxcrtRegs *) CSL_DMASS0_BCDMA_RCHANRT_BASE);
        drvHandle->trigGemOffset    = CSL_DMSS_GEM_BCDMA_TRIGGER_OFFSET;
        /* Fill other SOC specific parameters by reading from UDMA config
         * registers */
        CSL_bcdmaGetCfg(pBcdmaRegs);

        pPktdmaRegs = &drvHandle->pktdmaRegs;
        memset(pPktdmaRegs, 0, sizeof(*pPktdmaRegs));
    }
    else
    {
        drvHandle->instType = UDMA_INST_TYPE_LCDMA_PKTDMA;
        pPktdmaRegs = &drvHandle->pktdmaRegs;
        pPktdmaRegs->pGenCfgRegs     = ((CSL_pktdma_gcfgRegs *) CSL_DMASS0_PKTDMA_GCFG_BASE);
        pPktdmaRegs->pRxFlowCfgRegs  = ((CSL_pktdma_rxfcfgRegs *) CSL_DMASS0_PKTDMA_RFLOW_BASE);
        pPktdmaRegs->pTxChanCfgRegs  = ((CSL_pktdma_txccfgRegs *) CSL_DMASS0_PKTDMA_TCHAN_BASE);
        pPktdmaRegs->pRxChanCfgRegs  = ((CSL_pktdma_rxccfgRegs *) CSL_DMASS0_PKTDMA_RCHAN_BASE);
        pPktdmaRegs->pTxChanRtRegs   = ((CSL_pktdma_txcrtRegs *) CSL_DMASS0_PKTDMA_TCHANRT_BASE);
        pPktdmaRegs->pRxChanRtRegs   = ((CSL_pktdma_rxcrtRegs *) CSL_DMASS0_PKTDMA_RCHANRT_BASE);
        drvHandle->trigGemOffset     = 0;
        /* Fill other SOC specific parameters by reading from UDMA config
         * registers */
        CSL_pktdmaGetCfg(pPktdmaRegs);

        pBcdmaRegs = &drvHandle->bcdmaRegs;
        memset(pBcdmaRegs, 0, sizeof(*pBcdmaRegs));
    }

    /*
     * RA config init
     */
    drvHandle->raType = UDMA_RA_TYPE_LCDMA;
    pLcdmaRaRegs = &drvHandle->lcdmaRaRegs;
    if(UDMA_INST_ID_BCDMA_0 == instId)
    {
        pLcdmaRaRegs->pRingCfgRegs   = (CSL_lcdma_ringacc_ring_cfgRegs *) CSL_DMASS0_BCDMA_RING_BASE;
        pLcdmaRaRegs->pRingRtRegs    = (CSL_lcdma_ringacc_ringrtRegs *) CSL_DMASS0_BCDMA_RINGRT_BASE;
        pLcdmaRaRegs->pCredRegs      = (CSL_lcdma_ringacc_credRegs *) CSL_DMASS0_BCDMA_CRED_BASE;
        pLcdmaRaRegs->maxRings       = CSL_DMSS_BCDMA_NUM_BC_CHANS + CSL_DMSS_BCDMA_NUM_TX_CHANS + CSL_DMSS_BCDMA_NUM_RX_CHANS;
    }
    else
    {
        pLcdmaRaRegs->pRingCfgRegs   = (CSL_lcdma_ringacc_ring_cfgRegs *) CSL_DMASS0_PKTDMA_RING_BASE;
        pLcdmaRaRegs->pRingRtRegs    = (CSL_lcdma_ringacc_ringrtRegs *) CSL_DMASS0_PKTDMA_RINGRT_BASE;
        pLcdmaRaRegs->pCredRegs      = (CSL_lcdma_ringacc_credRegs *) CSL_DMASS0_PKTDMA_CRED_BASE;
        pLcdmaRaRegs->maxRings       = CSL_DMSS_PKTDMA_NUM_RX_FLOWS + CSL_DMSS_PKTDMA_NUM_TX_FLOWS;
    }
    drvHandle->ringDequeueRaw           = &Udma_ringDequeueRawLcdma;
    drvHandle->ringQueueRaw             = &Udma_ringQueueRawLcdma;
    drvHandle->ringFlushRaw             = &Udma_ringFlushRawLcdma;
    drvHandle->ringGetElementCnt        = &Udma_ringGetElementCntLcdma;
    drvHandle->ringGetMemPtr            = &Udma_ringGetMemPtrLcdma;
    drvHandle->ringGetMode              = &Udma_ringGetModeLcdma;
    drvHandle->ringGetForwardRingOcc    = &Udma_ringGetForwardRingOccLcdma;
    drvHandle->ringGetReverseRingOcc    = &Udma_ringGetReverseRingOccLcdma;
    drvHandle->ringGetWrIdx             = &Udma_ringGetWrIdxLcdma;
    drvHandle->ringGetRdIdx             = &Udma_ringGetRdIdxLcdma;
    drvHandle->ringPrime                = &Udma_ringPrimeLcdma;
    drvHandle->ringPrimeRead            = &Udma_ringPrimeReadLcdma;
    drvHandle->ringSetDoorBell          = &Udma_ringSetDoorBellLcdma;
    drvHandle->ringSetCfg               = &Udma_ringSetCfgLcdma;
    drvHandle->ringHandleClearRegs      = &Udma_ringHandleClearRegsLcdma;

    /* IA config init */
    pIaRegs = &drvHandle->iaRegs;
    pIaRegs->pCfgRegs       = (CSL_intaggr_cfgRegs *) CSL_DMASS0_INTAGGR_CFG_BASE;
    pIaRegs->pImapRegs      = (CSL_intaggr_imapRegs *) CSL_DMASS0_INTAGGR_IMAP_BASE;
    pIaRegs->pIntrRegs      = (CSL_intaggr_intrRegs *) CSL_DMASS0_INTAGGR_INTR_BASE;
    pIaRegs->pL2gRegs       = (CSL_intaggr_l2gRegs *) CSL_DMASS0_INTAGGR_L2G_BASE;
    pIaRegs->pMcastRegs     = (CSL_intaggr_mcastRegs *) CSL_DMASS0_INTAGGR_MCAST_BASE;
    pIaRegs->pGcntCfgRegs   = (CSL_intaggr_gcntcfgRegs *) CSL_DMASS0_INTAGGR_GCNTCFG_BASE;
    pIaRegs->pGcntRtiRegs   = (CSL_intaggr_gcntrtiRegs *) CSL_DMASS0_INTAGGR_GCNTRTI_BASE;
    CSL_intaggrGetCfg(pIaRegs);

    drvHandle->iaGemOffset  = CSL_DMSS_GEM_INTA0_SEVI_OFFSET;
    drvHandle->devIdIa      = TISCI_DEV_DMASS0_INTAGGR_0;
    drvHandle->devIdCore    = Sciclient_getSelfDevIdCore();

    /* Init other variables */
    if(UDMA_INST_ID_BCDMA_0 == instId)
    {
        drvHandle->txChOffset           = pBcdmaRegs->bcChanCnt;
        drvHandle->rxChOffset           = drvHandle->txChOffset + pBcdmaRegs->splitTxChanCnt;
       /* The srcIdx passed to Sciclient_rmIrqset API for configuring DMA Completion/Ring events,
        * will be ringNum + the corresponding following offset.
        * So setting the offset as TISCI Start Idx - corresponding ringNum Offset (if any) */
        drvHandle->blkCopyRingIrqOffset = TISCI_BCDMA0_BC_RC_OES_IRQ_SRC_IDX_START;
        drvHandle->txRingIrqOffset      = TISCI_BCDMA0_TX_RC_OES_IRQ_SRC_IDX_START - drvHandle->txChOffset;
        drvHandle->rxRingIrqOffset      = TISCI_BCDMA0_RX_RC_OES_IRQ_SRC_IDX_START - drvHandle->rxChOffset;
        drvHandle->udmapSrcThreadOffset = CSL_PSILCFG_DMSS_BCDMA_STRM_PSILS_THREAD_OFFSET;
        drvHandle->udmapDestThreadOffset= CSL_PSILCFG_DMSS_BCDMA_STRM_PSILD_THREAD_OFFSET;
        drvHandle->maxRings             = CSL_DMSS_BCDMA_NUM_BC_CHANS + CSL_DMSS_BCDMA_NUM_TX_CHANS + CSL_DMSS_BCDMA_NUM_RX_CHANS;
        drvHandle->devIdRing            = TISCI_DEV_DMASS0_BCDMA_0;
        drvHandle->devIdUdma            = TISCI_DEV_DMASS0_BCDMA_0;
       /* The srcIdx passed to Sciclient_rmIrqset API for configuring TR events,
        * will be chNum + the corresponding following offset.
        * So setting the offset as TISCI Start Idx - corresponding chNum Offset (if any) */
        drvHandle->srcIdTrIrq           = drvHandle->devIdIa;
        drvHandle->blkCopyTrIrqOffset   = TISCI_BCDMA0_BC_DC_OES_IRQ_SRC_IDX_START;
        drvHandle->txTrIrqOffset        = TISCI_BCDMA0_TX_DC_OES_IRQ_SRC_IDX_START;
        drvHandle->rxTrIrqOffset        = TISCI_BCDMA0_RX_DC_OES_IRQ_SRC_IDX_START;
    }
    else
    {
        drvHandle->txChOffset           = CSL_DMSS_PKTDMA_TX_FLOWS_UNMAPPED_START;
        drvHandle->rxChOffset           = CSL_DMSS_PKTDMA_RX_FLOWS_UNMAPPED_START;
        drvHandle->blkCopyRingIrqOffset = 0U; /* Not used for PktDMA Instance */
        drvHandle->txRingIrqOffset      = TISCI_PKTDMA0_TX_FLOW_OES_IRQ_SRC_IDX_START - drvHandle->txChOffset;
        drvHandle->rxRingIrqOffset      = TISCI_PKTDMA0_RX_FLOW_OES_IRQ_SRC_IDX_START - drvHandle->rxChOffset;
        drvHandle->udmapSrcThreadOffset = CSL_PSILCFG_DMSS_PKTDMA_STRM_PSILS_THREAD_OFFSET;
        drvHandle->udmapDestThreadOffset= CSL_PSILCFG_DMSS_PKTDMA_STRM_PSILD_THREAD_OFFSET;
        drvHandle->maxRings             = CSL_DMSS_PKTDMA_NUM_RX_FLOWS + CSL_DMSS_PKTDMA_NUM_TX_FLOWS;
        drvHandle->devIdRing            = TISCI_DEV_DMASS0_PKTDMA_0;
        drvHandle->devIdUdma            = TISCI_DEV_DMASS0_PKTDMA_0;
       /* TR Event is not supported for PKTMDA */
        drvHandle->srcIdTrIrq           = 0U;
        drvHandle->blkCopyTrIrqOffset   = 0U;
        drvHandle->txTrIrqOffset        = 0U;
        drvHandle->rxTrIrqOffset        = 0U;
    }
    drvHandle->devIdPsil     = TISCI_DEV_DMASS0;
    drvHandle->extChOffset  = 0U;
    drvHandle->srcIdRingIrq = drvHandle->devIdIa;

    return;
}

uint32_t Udma_isCacheCoherent(void)
{
    uint32_t isCacheCoherent;

    isCacheCoherent = FALSE;

    return (isCacheCoherent);
}

int32_t Udma_getMappedChRingAttributes(Udma_DrvHandleInt drvHandle,
                                       uint32_t mappedGrp,
                                       uint32_t chNum,
                                       Udma_MappedChRingAttributes *chAttr)
{
    const Udma_MappedChRingAttributes  *mappedChRingAttributes;
    uint32_t index = 0U;
    int32_t retVal = UDMA_SOK;

    if(mappedGrp < UDMA_NUM_MAPPED_TX_GROUP) /* Mapped TX Channel */
    {
        /* Check if the channel no. is out of range of mapped channel idx */
        if((chNum < CSL_DMSS_PKTDMA_TX_CHANS_CPSW_START) ||
           (chNum >= CSL_DMSS_PKTDMA_NUM_TX_CHANS))
        {
            retVal = UDMA_EINVALID_PARAMS;
            DebugP_logError("[UDMA] Incorrect Mapped Channel number!!!\r\n");
        }
        if(UDMA_SOK == retVal)
        {
            /* Calculate index by subtracting the start idx of mapped channels
            * (For AM64x, mapped channel starts with CPSW channel.) */
            index = chNum - CSL_DMSS_PKTDMA_TX_CHANS_CPSW_START;
            /* Check that, index is less than total no.of mapped TX channels */
            DebugP_assert(index < (CSL_DMSS_PKTDMA_NUM_TX_CHANS - CSL_DMSS_PKTDMA_TX_CHANS_UNMAPPED_CNT));
            mappedChRingAttributes = &gUdmaTxMappedChRingAttributes[index];
        }
    }
    else /* Mapped RX Channel */
    {
        /* Check if the channel no. is out of range of mapped channel idx */
        if((chNum < CSL_DMSS_PKTDMA_RX_CHANS_CPSW_START) ||
           (chNum >= CSL_DMSS_PKTDMA_NUM_RX_CHANS))
        {
            retVal = UDMA_EINVALID_PARAMS;
            DebugP_logError("[UDMA] Incorrect Mapped Channel number!!!\r\n");
        }
        if(UDMA_SOK == retVal)
        {
            /* Calculate index by subtracting the start idx of mapped channels
            * (For AM64x, mapped channel starts with CPSW channel.) */
            index = chNum - CSL_DMSS_PKTDMA_RX_CHANS_CPSW_START;
            /* Check that, index is less than total no.of mapped RX channels */
            DebugP_assert(index < (CSL_DMSS_PKTDMA_NUM_RX_CHANS - CSL_DMSS_PKTDMA_RX_CHANS_UNMAPPED_CNT));
            mappedChRingAttributes = &gUdmaRxMappedChRingAttributes[index];
        }
    }
    if(UDMA_SOK == retVal)
    {
        (void) memcpy(chAttr, mappedChRingAttributes, sizeof (Udma_MappedChRingAttributes));
    }

    return(retVal);
}

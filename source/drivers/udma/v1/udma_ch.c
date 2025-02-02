/*
 *  Copyright (C)2024 Texas Instruments Incorporated
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
 *  \file udma_ch.c
 *
 *  \brief File containing the UDMA driver channel related APIs.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/udma/udma_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define CSL_PSILCFG_REG_STATIC_TR               ((uint32_t) 0x400U)
#define CSL_PSILCFG_REG_RT_ENABLE               ((uint32_t) 0x408U)

/* PSILCFG_REG_RT_ENABLE */
#define CSL_PSILCFG_REG_RT_ENABLE_IDLE_SHIFT                (1U)
#define CSL_PSILCFG_REG_RT_ENABLE_IDLE_MASK                 ((uint32_t)0x01U<<CSL_PSILCFG_REG_RT_ENABLE_IDLE_SHIFT)
#define CSL_PSILCFG_REG_RT_ENABLE_FLUSH_SHIFT               (28U)
#define CSL_PSILCFG_REG_RT_ENABLE_FLUSH_MASK                ((uint32_t)0x01U << CSL_PSILCFG_REG_RT_ENABLE_FLUSH_SHIFT)
#define CSL_PSILCFG_REG_RT_ENABLE_PAUSE_SHIFT               (29U)
#define CSL_PSILCFG_REG_RT_ENABLE_PAUSE_MASK                ((uint32_t)0x01U<<CSL_PSILCFG_REG_RT_ENABLE_PAUSE_SHIFT)
#define CSL_PSILCFG_REG_RT_ENABLE_TDOWN_SHIFT               (30U)
#define CSL_PSILCFG_REG_RT_ENABLE_TDOWN_MASK                ((uint32_t)0x01U<<CSL_PSILCFG_REG_RT_ENABLE_TDOWN_SHIFT)
#define CSL_PSILCFG_REG_RT_ENABLE_ENABLE_SHIFT              (31U)
#define CSL_PSILCFG_REG_RT_ENABLE_ENABLE_MASK               ((uint32_t)0x01U<<CSL_PSILCFG_REG_RT_ENABLE_ENABLE_SHIFT)

/* PSILCFG_REG_STATIC_TR */
#define CSL_PSILCFG_REG_STATIC_TR_X_SHIFT                   (24U)
#define CSL_PSILCFG_REG_STATIC_TR_X_MASK                    (((uint32_t)0x0007U) << CSL_PSILCFG_REG_STATIC_TR_X_SHIFT)
#define CSL_PSILCFG_REG_STATIC_TR_Y_SHIFT                   (0U)
#define CSL_PSILCFG_REG_STATIC_TR_Y_MASK                    (((uint32_t)0x0FFFU) << CSL_PSILCFG_REG_STATIC_TR_Y_SHIFT)
#define CSL_PSILCFG_REG_STATIC_TR_Z_SHIFT                   (0U)
#define CSL_PSILCFG_REG_STATIC_TR_Z_MASK                    (((uint32_t)0x0FFFU) << CSL_PSILCFG_REG_STATIC_TR_Z_SHIFT)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void  Udma_chAssignRegOverlay(Udma_DrvHandleInt drvHandle, Udma_ChHandleInt chHandle);
static void  Udma_chInitRegs(Udma_ChHandleInt chHandle);
static void  Udma_chPauseTxLocal(Udma_DrvHandleInt drvHandle, uint32_t txChNum, uint32_t chType);
static void  Udma_chUnpauseTxLocal(Udma_DrvHandleInt drvHandle, uint32_t txChNum, uint32_t chType);
static void  Udma_chPauseRxLocal(Udma_DrvHandleInt drvHandle, uint32_t rxChNum);
static void  Udma_chUnpauseRxLocal(Udma_DrvHandleInt drvHandle, uint32_t rxChNum);
static int32_t Udma_chCheckParams(Udma_DrvHandleInt drvHandle,
                                  uint32_t chType,
                                  const Udma_ChPrms *chPrms);
static void Udma_chSetPeerReg(Udma_DrvHandleInt drvHandle,
                              const Udma_ChPdmaPrms *pdmaPrms,
                              volatile uint32_t *PEER8,
                              volatile uint32_t *PEER1,
                              volatile uint32_t *PEER0);
static int32_t Udma_chAllocResource(Udma_ChHandleInt chHandle);
static int32_t Udma_chFreeResource(Udma_ChHandleInt chHandle);
static int32_t Udma_chPair(Udma_ChHandleInt chHandle);
static int32_t Udma_chUnpair(Udma_ChHandleInt chHandle);
static void Udma_chEnableLocal(Udma_ChHandleInt chHandle);
static int32_t Udma_chDisableBlkCpyChan(Udma_ChHandleInt chHandle, uint32_t timeout);
static int32_t Udma_chDisableTxChan(Udma_ChHandleInt chHandle, uint32_t timeout);
static int32_t Udma_chDisableRxChan(Udma_ChHandleInt chHandle, uint32_t timeout);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Udma_chOpen(Udma_DrvHandle drvHandle,
                    Udma_ChHandle chHandle,
                    uint32_t chType,
                    const Udma_ChPrms *chPrms)
{
    int32_t             retVal = UDMA_SOK, tempRetVal;
    uint32_t            allocDone = (uint32_t) FALSE;
    Udma_ChHandleInt    chHandleInt;
    Udma_DrvHandleInt   drvHandleInt = (Udma_DrvHandleInt) drvHandle;

    /* Error check */
    if((drvHandleInt == NULL_PTR) || (NULL_PTR == chHandle) || (NULL_PTR == chPrms))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(drvHandleInt->drvInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        retVal = Udma_chCheckParams(drvHandleInt, chType, chPrms);
    }

    if(UDMA_SOK == retVal)
    {
        /* Copy and init parameters */
        chHandleInt = (Udma_ChHandleInt) chHandle;
        (void) memset(chHandleInt, 0, sizeof(Udma_ChObject));
        (void) memcpy(&chHandleInt->chPrms, chPrms, sizeof(Udma_ChPrms));
        chHandleInt->chType            = chType;
        chHandleInt->drvHandle         = drvHandleInt;
        chHandleInt->txChNum           = UDMA_DMA_CH_INVALID;
        chHandleInt->rxChNum           = UDMA_DMA_CH_INVALID;
        chHandleInt->extChNum          = UDMA_DMA_CH_INVALID;
        chHandleInt->pdmaChNum         = UDMA_DMA_CH_INVALID;
        chHandleInt->peerThreadId      = UDMA_THREAD_ID_INVALID;
        chHandleInt->fqRing            = (Udma_RingHandleInt) NULL_PTR;
        chHandleInt->cqRing            = (Udma_RingHandleInt) NULL_PTR;
        chHandleInt->tdCqRing          = (Udma_RingHandleInt) NULL_PTR;
        UdmaChTxPrms_init(&chHandleInt->txPrms, chType);
        UdmaChRxPrms_init(&chHandleInt->rxPrms, chType);
        Udma_chInitRegs(chHandleInt);
        chHandleInt->chOesAllocDone    = FALSE;
        chHandleInt->trigger           = CSL_UDMAP_TR_FLAGS_TRIGGER_NONE;
    }

    if(UDMA_SOK == retVal)
    {
        /* Alloc UDMA channel/rings */
        retVal = Udma_chAllocResource(chHandleInt);
        if(UDMA_SOK == retVal)
        {
            allocDone = (uint32_t) TRUE;
        }
        else
        {
            DebugP_logError("[UDMA] Channel resource allocation failed!!\r\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Pair channels */
        retVal = Udma_chPair(chHandleInt);
        if(UDMA_SOK != retVal)
        {
            DebugP_logError("[UDMA] Channel paring failed!!\r\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        chHandleInt->chInitDone = UDMA_INIT_DONE;
    }
    else
    {
        /* Error. Free-up resource if allocated */
        if(((uint32_t) TRUE) == allocDone)
        {
            tempRetVal = Udma_chFreeResource(chHandleInt);
            if(UDMA_SOK != tempRetVal)
            {
                DebugP_logError("[UDMA] Free resource failed!!!\r\n");
            }
        }
    }

    return (retVal);
}

int32_t Udma_chClose(Udma_ChHandle chHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) || (chHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        if(TRUE == chHandleInt->chOesAllocDone)
        {
            retVal = UDMA_EFAIL;
            DebugP_logError("[UDMA] Channel OES not de-allocated!!!\r\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Unpair channels */
        retVal = Udma_chUnpair(chHandleInt);
        if(UDMA_SOK != retVal)
        {
            DebugP_logError("[UDMA] Channel unparing failed!!\r\n");
        }

        /* Free-up UDMA channel/rings */
        retVal += Udma_chFreeResource(chHandleInt);
        if(UDMA_SOK != retVal)
        {
            DebugP_logError("[UDMA] Free resource failed!!!\r\n");
        }

        (void) memset(chHandleInt, 0, sizeof(*chHandleInt));
        chHandleInt->chInitDone = UDMA_DEINIT_DONE;
    }

    return (retVal);
}

int32_t Udma_chConfigTx(Udma_ChHandle chHandle, const Udma_ChTxPrms *txPrms)
{
    int32_t                 retVal = UDMA_SOK;
    Udma_DrvHandleInt       drvHandle;
    Udma_ChHandleInt        chHandleInt = (Udma_ChHandleInt) chHandle;
    struct tisci_msg_rm_udmap_tx_ch_cfg_req     rmUdmaTxReq;
    struct tisci_msg_rm_udmap_tx_ch_cfg_resp    rmUdmaTxResp;

    /* Error check */
    if((NULL_PTR == chHandleInt) ||
       (chHandleInt->chInitDone != UDMA_INIT_DONE) ||
       ((chHandleInt->chType & UDMA_CH_FLAG_TX) != UDMA_CH_FLAG_TX))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        DebugP_assert(chHandleInt->txChNum != UDMA_DMA_CH_INVALID);

        /* Copy params */
        rmUdmaTxReq.valid_params        = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERR_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_CHAN_TYPE_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_FETCH_SIZE_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_CQ_QNUM_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_PRIORITY_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_QOS_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_ORDER_ID_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIORITY_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_TX_FILT_EINFO_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_TX_FILT_PSWORDS_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_TX_SUPR_TDPKT_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_TX_FDEPTH_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_TX_CREDIT_COUNT_VALID;
        rmUdmaTxReq.nav_id              = drvHandle->devIdUdma;
        rmUdmaTxReq.index               = (uint16_t)chHandleInt->txChNum;
        rmUdmaTxReq.tx_pause_on_err     = txPrms->pauseOnError;
        rmUdmaTxReq.tx_filt_einfo       = txPrms->filterEinfo;
        rmUdmaTxReq.tx_filt_pswords     = txPrms->filterPsWords;
        rmUdmaTxReq.tx_atype            = txPrms->addrType;
        rmUdmaTxReq.tx_chan_type        = txPrms->chanType;
        rmUdmaTxReq.tx_fetch_size       = txPrms->fetchWordSize;
        rmUdmaTxReq.tx_priority         = txPrms->busPriority;
        rmUdmaTxReq.tx_qos              = txPrms->busQos;
        rmUdmaTxReq.tx_orderid          = txPrms->busOrderId;
        rmUdmaTxReq.fdepth              = txPrms->fifoDepth;
        rmUdmaTxReq.tx_burst_size       = txPrms->burstSize;
        rmUdmaTxReq.tx_sched_priority   = txPrms->dmaPriority;
        rmUdmaTxReq.tx_credit_count     = txPrms->txCredit;

        if(NULL_PTR != chHandleInt->tdCqRing)
        {
            DebugP_assert(chHandleInt->tdCqRing->ringNum != UDMA_RING_INVALID);
            /* used for pass by value and teardown */
            rmUdmaTxReq.txcq_qnum       = chHandleInt->tdCqRing->ringNum;
            rmUdmaTxReq.tx_supr_tdpkt   = txPrms->supressTdCqPkt;
        }
        else
        {
            /* TD CQ not used */
            rmUdmaTxReq.txcq_qnum       = UDMA_RING_INVALID;
            rmUdmaTxReq.tx_supr_tdpkt   = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_SUPPRESS_TD_ENABLED;
        }

        /* Config UDMAP TX channel */
        retVal = Sciclient_rmUdmapTxChCfg(
                     &rmUdmaTxReq, &rmUdmaTxResp, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            DebugP_logError("[UDMA] TX config failed!!!\r\n");
        }

        /* Copy the config */
        (void) memcpy(&chHandleInt->txPrms, txPrms, sizeof(chHandleInt->txPrms));
    }

    return (retVal);
}

int32_t Udma_chConfigRx(Udma_ChHandle chHandle, const Udma_ChRxPrms *rxPrms)
{
    int32_t                 retVal = UDMA_SOK;
    Udma_DrvHandleInt       drvHandle;
    Udma_ChHandleInt        chHandleInt = (Udma_ChHandleInt) chHandle;
    struct tisci_msg_rm_udmap_rx_ch_cfg_req     rmUdmaRxReq;
    struct tisci_msg_rm_udmap_rx_ch_cfg_resp    rmUdmaRxResp;
    Udma_FlowPrms           flowPrms;
    uint16_t                cqRing, fqRing;

    /* Error check */
    if((NULL_PTR == chHandleInt) ||
        (chHandleInt->chInitDone != UDMA_INIT_DONE) ||
        ((chHandleInt->chType & UDMA_CH_FLAG_RX) != UDMA_CH_FLAG_RX))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if ((UDMA_INST_TYPE_LCDMA_BCDMA                 == drvHandle->instType) &&
            ((chHandleInt->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY))
        {
            /* For BCDMA Block Copy, no need to configure Rx Channel.*/
        }
        else
        {
            /* Note: Block copy uses same RX channel as TX */
            DebugP_assert(chHandleInt->rxChNum != UDMA_DMA_CH_INVALID);

            /* Copy params */
            rmUdmaRxReq.valid_params        = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERR_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_CHAN_TYPE_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_FETCH_SIZE_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_CQ_QNUM_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_PRIORITY_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_QOS_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_ORDER_ID_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIORITY_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_RX_FLOWID_START_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_RX_FLOWID_CNT_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_RX_IGNORE_SHORT_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_RX_IGNORE_LONG_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_VALID;
            rmUdmaRxReq.nav_id              = drvHandle->devIdUdma;
            rmUdmaRxReq.index               = (uint16_t)chHandleInt->rxChNum;
            rmUdmaRxReq.rx_pause_on_err     = rxPrms->pauseOnError;
            rmUdmaRxReq.rx_atype            = rxPrms->addrType;
            rmUdmaRxReq.rx_chan_type        = rxPrms->chanType;
            rmUdmaRxReq.rx_fetch_size       = rxPrms->fetchWordSize;
            rmUdmaRxReq.rx_priority         = rxPrms->busPriority;
            rmUdmaRxReq.rx_qos              = rxPrms->busQos;
            rmUdmaRxReq.rx_orderid          = rxPrms->busOrderId;
            rmUdmaRxReq.rx_sched_priority   = rxPrms->dmaPriority;
            rmUdmaRxReq.flowid_start        = rxPrms->flowIdFwRangeStart;
            rmUdmaRxReq.flowid_cnt          = rxPrms->flowIdFwRangeCnt;
            rmUdmaRxReq.rx_ignore_short     = rxPrms->ignoreShortPkts;
            rmUdmaRxReq.rx_ignore_long      = rxPrms->ignoreLongPkts;
            rmUdmaRxReq.rx_burst_size       = rxPrms->burstSize;
            if(NULL_PTR != chHandleInt->tdCqRing)
            {
                DebugP_assert(chHandleInt->tdCqRing->ringNum != UDMA_RING_INVALID);
                /* used for pass by value and teardown */
                rmUdmaRxReq.rxcq_qnum          = chHandleInt->tdCqRing->ringNum;
            }
            else
            {
                /* TD CQ not used */
                rmUdmaRxReq.rxcq_qnum          = UDMA_RING_INVALID;
            }

            /* Config UDMAP RX channel */
            retVal = Sciclient_rmUdmapRxChCfg(
                         &rmUdmaRxReq, &rmUdmaRxResp, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                DebugP_logError("[UDMA] RX config failed!!!\r\n");
            }

            /* Configure default flow for PDMA and other PSIL channels */
            if((((chHandleInt->chType & UDMA_CH_FLAG_PDMA) == UDMA_CH_FLAG_PDMA) ||
                    ((chHandleInt->chType & UDMA_CH_FLAG_PSIL) == UDMA_CH_FLAG_PSIL)) &&
               (TRUE == rxPrms->configDefaultFlow))
            {
                UdmaFlowPrms_init(&flowPrms, chHandleInt->chType);
                flowPrms.psInfoPresent = rxPrms->flowPsInfoPresent;
                flowPrms.einfoPresent  = rxPrms->flowEInfoPresent;
                flowPrms.errorHandling = rxPrms->flowErrorHandling;
                flowPrms.sopOffset     = rxPrms->flowSopOffset;

                if(NULL_PTR == chHandleInt->cqRing)
                {
                    /* Ring not allocated */
                    cqRing = UDMA_RING_INVALID;
                }
                else
                {
                    cqRing = chHandleInt->cqRing->ringNum;
                    DebugP_assert(cqRing != UDMA_RING_INVALID);
                }
                if(NULL_PTR == chHandleInt->fqRing)
                {
                    /* Ring not allocated */
                    fqRing = UDMA_RING_INVALID;
                }
                else
                {
                    fqRing = chHandleInt->fqRing->ringNum;
                    DebugP_assert(fqRing != UDMA_RING_INVALID);
                }

                flowPrms.defaultRxCQ    = cqRing;
                /* Use the same free queue as default flow is not used in
                 * selecting different queues based on threshold */
                flowPrms.fdq0Sz0Qnum    = fqRing;
                flowPrms.fdq0Sz1Qnum    = fqRing;
                flowPrms.fdq0Sz2Qnum    = fqRing;
                flowPrms.fdq0Sz3Qnum    = fqRing;
                flowPrms.fdq1Qnum       = fqRing;
                flowPrms.fdq2Qnum       = fqRing;
                flowPrms.fdq3Qnum       = fqRing;

                /* Config default flow. Caller can overwite again if required */
                retVal = Udma_flowConfig(chHandleInt->defaultFlow, 0U, &flowPrms);
                if(UDMA_SOK != retVal)
                {
                    DebugP_logError("[UDMA] Default RX flow config failed!!!\r\n");
                }
            }

            if(UDMA_SOK == retVal)
            {
                /* Copy the config */
                (void) memcpy(&chHandleInt->rxPrms, rxPrms, sizeof(chHandleInt->rxPrms));
            }
        }
    }
    return (retVal);
}

int32_t Udma_chConfigPdma(Udma_ChHandle chHandle,
                          const Udma_ChPdmaPrms *pdmaPrms)
{
    int32_t             retVal = UDMA_SOK;
    volatile uint32_t  *PEER8=NULL, *PEER0=NULL, *PEER1=NULL;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) ||
       (NULL_PTR == pdmaPrms) ||
       (chHandleInt->chInitDone != UDMA_INIT_DONE) ||
       ((chHandleInt->chType & UDMA_CH_FLAG_PDMA) != UDMA_CH_FLAG_PDMA))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
        {
            if((chHandleInt->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
            {
                Udma_assert(drvHandle, chHandleInt->pTxRtRegs != NULL_PTR);

                PEER8 = &chHandleInt->pTxRtRegs->PEER8;
                PEER1 = &chHandleInt->pTxRtRegs->PEER1;
                PEER0 = &chHandleInt->pTxRtRegs->PEER0;
            }
            else
            {
                Udma_assert(drvHandle, chHandleInt->pRxRtRegs != NULL_PTR);

                PEER8 = &chHandleInt->pRxRtRegs->PEER8;
                PEER1 = &chHandleInt->pRxRtRegs->PEER1;
                PEER0 = &chHandleInt->pRxRtRegs->PEER0;
            }
            Udma_chSetPeerReg(drvHandle, pdmaPrms, PEER8, PEER1, PEER0);
        }
    }

    return (retVal);
}

int32_t Udma_chEnable(Udma_ChHandle chHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) || (chHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Enable channel based on channel type */
        Udma_chEnableLocal(chHandleInt);
    }

    return (retVal);
}

int32_t Udma_chDisable(Udma_ChHandle chHandle, uint32_t timeout)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) || (chHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Call disable sequence for respective modes */
        if((chHandleInt->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
        {
            retVal = Udma_chDisableBlkCpyChan(chHandleInt, timeout);
        }
        else
        {
            if((chHandleInt->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
            {
                retVal = Udma_chDisableTxChan(chHandleInt, timeout);
            }
            else
            {
                retVal = Udma_chDisableRxChan(chHandleInt, timeout);
            }
        }
    }

    return (retVal);
}

int32_t Udma_chPause(Udma_ChHandle chHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) || (chHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if((chHandleInt->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            DebugP_assert(chHandleInt->txChNum != UDMA_DMA_CH_INVALID);
            Udma_chPauseTxLocal(drvHandle, chHandleInt->txChNum, chHandleInt->chType);
        }

        if((chHandleInt->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
        {
            /* Note: Block copy uses same RX channel. So do for both TX/RX */
            if ((UDMA_INST_TYPE_LCDMA_BCDMA                 == chHandleInt->drvHandle->instType) &&
                ((chHandleInt->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY))
            {
                /* In case of BCDMA Block Copy, No need to do for RX */
            }
            else
            {
                DebugP_assert(chHandleInt->rxChNum != UDMA_DMA_CH_INVALID);
                Udma_chPauseRxLocal(drvHandle, chHandleInt->rxChNum);
            }
        }
    }

    return (retVal);
}

int32_t Udma_chResume(Udma_ChHandle chHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) || (chHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if((chHandleInt->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            DebugP_assert(chHandleInt->txChNum != UDMA_DMA_CH_INVALID);
            Udma_chUnpauseTxLocal(drvHandle, chHandleInt->txChNum, chHandleInt->chType);
        }
        if((chHandleInt->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
        {
            /* Note: Block copy uses same RX channel. So do for both TX/RX */
            if ((UDMA_INST_TYPE_LCDMA_BCDMA                 == chHandleInt->drvHandle->instType) &&
                ((chHandleInt->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY))
            {
                /* In case of BCDMA Block Copy, No need to do for RX */
            }
            else
            {
                DebugP_assert(chHandleInt->rxChNum != UDMA_DMA_CH_INVALID);
                Udma_chUnpauseRxLocal(drvHandle, chHandleInt->rxChNum);
            }
        }
    }

    return (retVal);
}

uint32_t Udma_chGetNum(Udma_ChHandle chHandle)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) || (chHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if((chHandleInt->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            DebugP_assert(chHandleInt->txChNum != UDMA_DMA_CH_INVALID);
            chNum = chHandleInt->txChNum;
        }
        else
        {
            DebugP_assert(chHandleInt->rxChNum != UDMA_DMA_CH_INVALID);
            chNum = chHandleInt->rxChNum;
        }
    }

    return (chNum);
}

Udma_RingHandle Udma_chGetFqRingHandle(Udma_ChHandle chHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_RingHandle     fqRing = (Udma_RingHandle) NULL_PTR;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) || (chHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        fqRing = (Udma_RingHandle) chHandleInt->fqRing;
    }

    return (fqRing);
}

Udma_RingHandle Udma_chGetCqRingHandle(Udma_ChHandle chHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_RingHandle     cqRing = (Udma_RingHandle) NULL_PTR;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) || (chHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        cqRing = (Udma_RingHandle) chHandleInt->cqRing;
    }

    return (cqRing);
}

Udma_RingHandle Udma_chGetTdCqRingHandle(Udma_ChHandle chHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_RingHandle     tdCqRing = (Udma_RingHandle) NULL_PTR;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) || (chHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        tdCqRing = (Udma_RingHandle) chHandleInt->tdCqRing;
    }

    return (tdCqRing);
}

uint16_t Udma_chGetFqRingNum(Udma_ChHandle chHandle)
{
    uint16_t            ringNum = UDMA_RING_INVALID;
    Udma_RingHandle     ringHandle;

    ringHandle = Udma_chGetFqRingHandle(chHandle);
    if(ringHandle != NULL_PTR)
    {
        ringNum = Udma_ringGetNum(ringHandle);
    }

    return (ringNum);
}

uint16_t Udma_chGetCqRingNum(Udma_ChHandle chHandle)
{
    uint16_t            ringNum = UDMA_RING_INVALID;
    Udma_RingHandle     ringHandle;

    ringHandle = Udma_chGetCqRingHandle(chHandle);
    if(ringHandle != NULL_PTR)
    {
        ringNum = Udma_ringGetNum(ringHandle);
    }

    return (ringNum);
}

Udma_FlowHandle Udma_chGetDefaultFlowHandle(Udma_ChHandle chHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_FlowHandle     defaultFlow = (Udma_FlowHandle) NULL_PTR;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) || (chHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        defaultFlow = (Udma_FlowHandle) chHandleInt->defaultFlow;
    }

    return (defaultFlow);
}

int32_t Udma_chDequeueTdResponse(Udma_ChHandle chHandle,
                                 CSL_UdmapTdResponse *tdResponse)
{
    int32_t     retVal = UDMA_SOK, cslRetVal;
    uint64_t    response;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    if((NULL_PTR != chHandleInt->tdCqRing) &&
       (chHandleInt->tdCqRing->ringNum != UDMA_RING_INVALID) &&
       (NULL_PTR != tdResponse))
    {
        cslRetVal = CSL_ringaccPop64(
            &chHandleInt->drvHandle->raRegs,
            &chHandleInt->tdCqRing->cfg,
            &response,
            &Udma_ringaccMemOps);

        if(0 != cslRetVal)
        {
            retVal = UDMA_ETIMEOUT;
        }
        else
        {
            CSL_udmapGetTdResponse(response, tdResponse);
        }
    }
    else
    {
        retVal = UDMA_EFAIL;
    }

    return (retVal);
}

uint32_t Udma_chGetTriggerEvent(Udma_ChHandle chHandle, uint32_t trigger)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            triggerEvent = UDMA_EVENT_INVALID;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) || (chHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if((CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0 == trigger) ||
        (CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL1 == trigger))
        {
            /* Global 0/1 triggers are interleaved - so multiply by 2 */
            if(((chHandleInt->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY) ||
            ((chHandleInt->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX))
            {
                /* For block copy return the TX channel trigger */
                triggerEvent = (chHandleInt->txChNum * 2U);
                if(CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL1 == trigger)
                {
                    triggerEvent++; /* Global1 trigger is next to global0 */
                }
                /* Add the global offset */
                triggerEvent += drvHandle->trigGemOffset;
            }
            else if((chHandleInt->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
            {
                /* RX trigger is after TX channel triggers
                * Note: There is no external channel triggers - hence not
                * using rxChOffset */
                if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
                {
                    triggerEvent  = (drvHandle->udmapRegs.txChanCnt * 2U);
                }
                triggerEvent += (chHandleInt->rxChNum * 2U);
                if(CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL1 == trigger)
                {
                    triggerEvent++; /* Global1 trigger is next to global0 */
                }
                /* Add the global offset */
                triggerEvent += drvHandle->trigGemOffset;
            }
            else
            {
                /* Trigger not supported for external channel -
                * return no event - already set */
            }
        }
    }

    return (triggerEvent);
}

void *Udma_chGetSwTriggerRegister(Udma_ChHandle chHandle)
{
    int32_t                 retVal = UDMA_SOK;
    Udma_DrvHandleInt       drvHandle;
    Udma_ChHandleInt        chHandleInt = (Udma_ChHandleInt) chHandle;
    void                   *pSwTriggerReg = NULL;

    /* Error check */
    if((NULL_PTR == chHandleInt) ||
       (chHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(((chHandleInt->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY) ||
           ((chHandleInt->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX))
        {
            if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
            {
                Udma_assert(drvHandle, chHandleInt->pTxRtRegs != NULL_PTR);
                pSwTriggerReg = (void *) &chHandleInt->pTxRtRegs->SWTRIG;
            }
        }
        else if((chHandleInt->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
        {
            DebugP_logError("[UDMA] SW trigger not supported for RX channels!!!\r\n");
        }
        else
        {
            DebugP_logError("[UDMA] Not supported!!!\r\n");
        }
    }

    return (pSwTriggerReg);
}

int32_t Udma_chSetSwTrigger(Udma_ChHandle chHandle, uint32_t trigger)
{
    int32_t                 retVal = UDMA_SOK;
    Udma_DrvHandleInt       drvHandle;
    Udma_ChHandleInt        chHandleInt = (Udma_ChHandleInt) chHandle;
    void                   *pSwTriggerReg = NULL;

    /* Error check */
    if((NULL_PTR == chHandleInt) ||
       (chHandleInt->chInitDone != UDMA_INIT_DONE) ||
       (trigger > CSL_UDMAP_TR_FLAGS_TRIGGER_LOCAL_EVENT) ||
       (trigger == CSL_UDMAP_TR_FLAGS_TRIGGER_NONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        pSwTriggerReg = Udma_chGetSwTriggerRegister(chHandleInt);
        if(pSwTriggerReg != NULL)
        {
            CSL_REG32_WR(pSwTriggerReg, ((uint32_t)1U << (trigger - 1U)));
        }
        else
        {
            retVal = UDMA_EFAIL;
        }
    }

    return (retVal);
}

int32_t Udma_chSetChaining(Udma_ChHandle triggerChHandle,
                           Udma_ChHandle chainedChHandle,
                           uint32_t trigger)
{
    int32_t                             retVal = UDMA_SOK;
    Udma_DrvHandleInt                   drvHandle;
    Udma_ChHandleInt                    triggerChHandleInt = (Udma_ChHandleInt) triggerChHandle;
    Udma_ChHandleInt                    chainedChHandleInt = (Udma_ChHandleInt) chainedChHandle;
    uint32_t                            triggerEvent;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    /* Error check */
    if((NULL_PTR == triggerChHandleInt) || (triggerChHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if((NULL_PTR == chainedChHandleInt) || (chainedChHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = triggerChHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        if(TRUE == triggerChHandleInt->chOesAllocDone)
        {
            retVal = UDMA_EFAIL;
            DebugP_logError("[UDMA] Trigger channel OES already allocated!!\r\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Get the global trigger event to set */
        triggerEvent =
            Udma_chGetTriggerEvent(chainedChHandle, trigger);
        if(UDMA_EVENT_INVALID == triggerEvent)
        {
            retVal = UDMA_EFAIL;
            DebugP_logError("[UDMA] Invalid trigger mode!!\r\n");
       }
    }

    if(UDMA_SOK == retVal)
    {
        rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
        rmIrqReq.src_id                 = drvHandle->srcIdTrIrq;
        rmIrqReq.global_event           = (uint16_t) triggerEvent;
        rmIrqReq.src_index              = 0U;
        rmIrqReq.dst_id                 = 0U;
        rmIrqReq.dst_host_irq           = 0U;
        rmIrqReq.ia_id                  = 0U;
        rmIrqReq.vint                   = 0U;
        rmIrqReq.vint_status_bit_index  = 0U;
        rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        if(UDMA_INST_TYPE_LCDMA_PKTDMA == drvHandle->instType)
        {
            retVal = UDMA_EFAIL;
            DebugP_logError("UDMA chaining not supported for PKTDMA instance!!!\r\n");
        }
        else if((triggerChHandleInt->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
        {
            DebugP_assert(
                triggerChHandleInt->txChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)triggerChHandleInt->txChNum;
            rmIrqReq.src_index += drvHandle->blkCopyTrIrqOffset;
            retVal = Sciclient_rmIrqSet(
                         &rmIrqReq, &rmIrqResp, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                DebugP_logError("[UDMA] RM Block Copy Channel chain config failed!!!\r\n");
            }
        }
        else if((triggerChHandleInt->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
        {
            DebugP_assert(
                triggerChHandleInt->rxChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)triggerChHandleInt->rxChNum;
            rmIrqReq.src_index += drvHandle->rxTrIrqOffset;
            retVal = Sciclient_rmIrqSet(
                         &rmIrqReq, &rmIrqResp, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                DebugP_logError("[UDMA] RM RX Channel chain config failed!!!\r\n");
            }
        }
        else if((triggerChHandleInt->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            DebugP_assert(
                triggerChHandleInt->txChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)triggerChHandleInt->txChNum;
            rmIrqReq.src_index += drvHandle->txTrIrqOffset;
            retVal = Sciclient_rmIrqSet(
                         &rmIrqReq, &rmIrqResp, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                DebugP_logError("[UDMA] RM TX Channel chain config failed!!!\r\n");
            }
        }
        else
        {
            retVal = UDMA_EFAIL;
            DebugP_logError("[UDMA] Chaining not supported for other channels!!\r\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Mark OES alloc flag */
        triggerChHandleInt->chOesAllocDone = TRUE;
        triggerChHandleInt->trigger        = trigger;
    }

    return (retVal);
}

int32_t Udma_chBreakChaining(Udma_ChHandle triggerChHandle,
                             Udma_ChHandle chainedChHandle)
{
    int32_t                             retVal = UDMA_SOK;
    Udma_DrvHandleInt                   drvHandle;
    Udma_ChHandleInt                    triggerChHandleInt = (Udma_ChHandleInt) triggerChHandle;
    Udma_ChHandleInt                    chainedChHandleInt = (Udma_ChHandleInt) chainedChHandle;
    uint32_t                            triggerEvent;
    struct tisci_msg_rm_irq_release_req rmIrqReq;

    /* Error check */
    if((NULL_PTR == triggerChHandleInt) || (triggerChHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if((NULL_PTR == chainedChHandleInt) || (chainedChHandleInt->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = triggerChHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        if(FALSE == triggerChHandleInt->chOesAllocDone)
        {
            retVal = UDMA_EFAIL;
            DebugP_logError("[UDMA] Trigger channel OES not allocated!!\r\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Get the global trigger event to set */
        triggerEvent =
            Udma_chGetTriggerEvent(chainedChHandle, triggerChHandleInt->trigger);
        if(UDMA_EVENT_INVALID == triggerEvent)
        {
            retVal = UDMA_EFAIL;
            DebugP_logError("[UDMA] Invalid trigger mode!!\r\n");
       }
    }

    if(UDMA_SOK == retVal)
    {
        rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
        rmIrqReq.src_id                 = drvHandle->srcIdTrIrq;
        rmIrqReq.src_index              = 0U;
        rmIrqReq.global_event           = (uint16_t)triggerEvent;
        rmIrqReq.dst_id                 = 0U;
        rmIrqReq.dst_host_irq           = 0U;
        rmIrqReq.ia_id                  = 0U;
        rmIrqReq.vint                   = 0U;
        rmIrqReq.vint_status_bit_index  = 0U;
        rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        if(UDMA_INST_TYPE_LCDMA_PKTDMA == drvHandle->instType)
        {
            retVal = UDMA_EFAIL;
            DebugP_logError("UDMA chaining not supported for PKTDMA instance!!!\r\n");
        }
        else if((triggerChHandleInt->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
        {
            DebugP_assert(
                triggerChHandleInt->txChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)triggerChHandleInt->txChNum;
            rmIrqReq.src_index += drvHandle->blkCopyTrIrqOffset;
            retVal = Sciclient_rmIrqRelease(&rmIrqReq, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                DebugP_logError("[UDMA] RM Block Copy Channel chain reset failed!!!\r\n");
            }
        }
        else if((triggerChHandleInt->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
        {
            DebugP_assert(
                triggerChHandleInt->rxChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)triggerChHandleInt->rxChNum;
            rmIrqReq.src_index += drvHandle->rxTrIrqOffset;
            retVal = Sciclient_rmIrqRelease(&rmIrqReq, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                DebugP_logError("[UDMA] RM RX Channel chain reset failed!!!\r\n");
            }
        }
        else if((triggerChHandleInt->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            DebugP_assert(
                triggerChHandleInt->txChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)triggerChHandleInt->txChNum;
            rmIrqReq.src_index += drvHandle->txTrIrqOffset;
            retVal = Sciclient_rmIrqRelease(&rmIrqReq, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                DebugP_logError("[UDMA] RM TX Channel chain reset failed!!!\r\n");
            }
        }
        else
        {
            retVal = UDMA_EFAIL;
            DebugP_logError("[UDMA] Chaining not supported for other channels!!\r\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Mark OES alloc flag as free */
        triggerChHandleInt->chOesAllocDone = FALSE;
    }

    return (retVal);
}

void UdmaChPrms_init(Udma_ChPrms *chPrms, uint32_t chType)
{
    if(NULL_PTR != chPrms)
    {
        chPrms->chNum       = UDMA_DMA_CH_ANY;
        chPrms->peerChNum   = UDMA_DMA_CH_INVALID;
        if(UDMA_CH_TYPE_TR_BLK_COPY == chType)
        {
            chPrms->peerChNum   = UDMA_DMA_CH_NA;
        }
        chPrms->mappedChGrp = UDMA_MAPPED_GROUP_INVALID;
        chPrms->appData     = NULL_PTR;
        UdmaRingPrms_init(&chPrms->fqRingPrms);
        UdmaRingPrms_init(&chPrms->cqRingPrms);
        UdmaRingPrms_init(&chPrms->tdCqRingPrms);
        /* TD and TR response is always 8 byte irrespective of mode */
        chPrms->tdCqRingPrms.elemSize = UDMA_RING_ES_8BYTES;
    }

    return;
}

void UdmaChTxPrms_init(Udma_ChTxPrms *txPrms, uint32_t chType)
{
    if(NULL_PTR != txPrms)
    {
        txPrms->pauseOnError    = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERROR_DISABLED;
        txPrms->filterEinfo     = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_EINFO_DISABLED;
        txPrms->filterPsWords   = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_PSWORDS_DISABLED;
        txPrms->addrType        = TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_PHYS;
        txPrms->chanType        = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_PACKET;
        if((chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
        {
            txPrms->chanType    = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_3P_BLOCK_REF;
        }
        txPrms->fetchWordSize   = 16U;  /* sizeof(CSL_UdmapTR15) / sizeof(uint32_t) */
        txPrms->busPriority     = UDMA_DEFAULT_TX_CH_BUS_PRIORITY;
        txPrms->busQos          = UDMA_DEFAULT_TX_CH_BUS_QOS;
        txPrms->busOrderId      = UDMA_DEFAULT_TX_CH_BUS_ORDERID;
        txPrms->dmaPriority     = UDMA_DEFAULT_TX_CH_DMA_PRIORITY;
        txPrms->txCredit        = 0U;
        if((chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
        {
            txPrms->fifoDepth   = (uint16_t)UDMA_TX_UHC_CHANS_FDEPTH;
            txPrms->burstSize   = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_256_BYTES;
        }
        else if((chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
        {
            txPrms->fifoDepth   = (uint16_t)UDMA_TX_HC_CHANS_FDEPTH;
            txPrms->burstSize   = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_256_BYTES;
        }
        else
        {
            txPrms->fifoDepth   = (uint16_t)UDMA_TX_CHANS_FDEPTH;
            txPrms->burstSize   = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_64_BYTES;
        }
        txPrms->supressTdCqPkt  = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_SUPPRESS_TD_DISABLED;
    }

    return;
}

void UdmaChRxPrms_init(Udma_ChRxPrms *rxPrms, uint32_t chType)
{
    if(NULL_PTR != rxPrms)
    {
        rxPrms->pauseOnError        = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERROR_DISABLED;
        rxPrms->addrType            = TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_PHYS;
        rxPrms->chanType            = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_PACKET;
        if((chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
        {
            rxPrms->chanType        = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_3P_BLOCK_REF;
        }
        rxPrms->fetchWordSize       = 16U;  /* sizeof(CSL_UdmapTR15) / sizeof(uint32_t) */
        rxPrms->busPriority         = UDMA_DEFAULT_RX_CH_BUS_PRIORITY;
        rxPrms->busQos              = UDMA_DEFAULT_RX_CH_BUS_QOS;
        rxPrms->busOrderId          = UDMA_DEFAULT_RX_CH_BUS_ORDERID;
        rxPrms->dmaPriority         = UDMA_DEFAULT_RX_CH_DMA_PRIORITY;
        rxPrms->flowIdFwRangeStart  = 0U;       /* Reset value - to use default flow */
        rxPrms->flowIdFwRangeCnt    = 0U;       /* Reset value - to use default flow */
        rxPrms->flowEInfoPresent    = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_EINFO_NOT_PRESENT;       /* Default no EINFO */
        rxPrms->flowPsInfoPresent   = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PSINFO_NOT_PRESENT;      /* Default no PSINFO */
        rxPrms->flowErrorHandling   = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_ERR_RETRY;       /* Default Re-try descriptor allocation operation on starvation error */
        rxPrms->flowSopOffset       = 0U;      /* Default SOP offset is 0 */
        rxPrms->ignoreShortPkts     = TISCI_MSG_VALUE_RM_UDMAP_RX_CH_PACKET_EXCEPTION;
        rxPrms->ignoreLongPkts      = TISCI_MSG_VALUE_RM_UDMAP_RX_CH_PACKET_EXCEPTION;
        rxPrms->configDefaultFlow   = TRUE;
        if((chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
        {
            rxPrms->burstSize   = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_256_BYTES;
        }
        else if((chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
        {
            rxPrms->burstSize   = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_256_BYTES;
        }
        else
        {
            rxPrms->burstSize   = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_64_BYTES;
        }
    }

    return;
}

void UdmaChPdmaPrms_init(Udma_ChPdmaPrms *pdmaPrms)
{
    if(NULL_PTR != pdmaPrms)
    {
        pdmaPrms->elemSize  = UDMA_PDMA_ES_8BITS;
        pdmaPrms->elemCnt   = 0U;
        pdmaPrms->fifoCnt   = 0U;
    }

    return;
}

int32_t Udma_chGetStats(Udma_ChHandle chHandle, Udma_ChStats *chStats)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;
    uint32_t            chNum;
    CSL_UdmapChanStats udmapChanStats;
    CSL_UdmapChanDir   udmapChDir;

    /* Error check */
    if ((NULL_PTR == chHandleInt)                   ||
        (chHandleInt->chInitDone != UDMA_INIT_DONE) ||
        (chStats == NULL))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
        {
            if((chHandleInt->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
            {
                chNum       = chHandleInt->txChNum;
                udmapChDir = CSL_UDMAP_CHAN_DIR_TX;
            }
            else if((chHandleInt->chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
            {
                chNum       = chHandleInt->extChNum + drvHandle->extChOffset;
                udmapChDir = CSL_UDMAP_CHAN_DIR_TX;
            }
            else
            {
                if((chHandleInt->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
                {
                    chNum       = chHandleInt->txChNum;
                    udmapChDir = CSL_UDMAP_CHAN_DIR_TX;
                }
                else
                {
                    chNum       = chHandleInt->rxChNum;
                    udmapChDir = CSL_UDMAP_CHAN_DIR_RX;
                }
            }
            CSL_udmapGetChanStats(&drvHandle->udmapRegs, chNum, udmapChDir, &udmapChanStats);
            (void)memcpy(chStats, &udmapChanStats, sizeof(Udma_ChStats));
        }
    }

    return (retVal);
}

int32_t Udma_getPeerData(Udma_ChHandle chHandle, uint32_t *peerData)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) ||
       (chHandleInt->chInitDone != UDMA_INIT_DONE) ||
       ((chHandleInt->chType & UDMA_CH_FLAG_PDMA) != UDMA_CH_FLAG_PDMA))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    return (retVal);
}

int32_t Udma_clearPeerData(Udma_ChHandle chHandle, uint32_t peerData)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_ChHandleInt    chHandleInt = (Udma_ChHandleInt) chHandle;

    /* Error check */
    if((NULL_PTR == chHandleInt) ||
       (chHandleInt->chInitDone != UDMA_INIT_DONE) ||
       ((chHandleInt->chType & UDMA_CH_FLAG_PDMA) != UDMA_CH_FLAG_PDMA))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    return (retVal);
}

static int32_t Udma_chCheckParams(Udma_DrvHandleInt drvHandle,
                                  uint32_t chType,
                                  const Udma_ChPrms *chPrms)
{
    int32_t     retVal = UDMA_SOK;

    if((chType & UDMA_CH_FLAG_PDMA) == UDMA_CH_FLAG_PDMA)
    {
        if((UDMA_DMA_CH_INVALID == chPrms->peerChNum) ||
           (UDMA_DMA_CH_NA == chPrms->peerChNum))
        {
            retVal = UDMA_EINVALID_PARAMS;
            DebugP_logError("[UDMA] Invalid Peer Channel Number!!!\r\n");
        }
    }
    if((chType & UDMA_CH_FLAG_MAPPED) == UDMA_CH_FLAG_MAPPED)
    {
        if(UDMA_MAPPED_GROUP_INVALID == chPrms->mappedChGrp)
        {
            retVal = UDMA_EINVALID_PARAMS;
            DebugP_logError("[UDMA] Invalid Mapped Channel Group!!!\r\n");
        }
    }

    return (retVal);
}

static int32_t Udma_chAllocResource(Udma_ChHandleInt chHandle)
{
    int32_t                 retVal = UDMA_SOK, tempRetVal;
    Udma_DrvHandleInt       drvHandle;
    uint16_t                ringNum = UDMA_RING_INVALID;

    drvHandle = chHandle->drvHandle;

    /* Allocate UDMAP channel */
    if((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
    {
        if((chHandle->chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
        {
            chHandle->txChNum =
                Udma_rmAllocBlkCopyHcCh(chHandle->chPrms.chNum, drvHandle);
        }
        else if((chHandle->chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
        {
            chHandle->txChNum =
                Udma_rmAllocBlkCopyUhcCh(chHandle->chPrms.chNum, drvHandle);
        }
        else
        {
            chHandle->txChNum =
                Udma_rmAllocBlkCopyCh(chHandle->chPrms.chNum, drvHandle);
        }
        if(UDMA_DMA_CH_INVALID == chHandle->txChNum)
        {
            retVal = UDMA_EALLOC;
            DebugP_logError("[UDMA] RM Alloc Blkcpy Ch failed!!!\r\n");
        }
        else
        {
            if (UDMA_INST_TYPE_LCDMA_BCDMA == chHandle->drvHandle->instType)
            {
                /* For BCDMA Block Copy, rxChNum is not used.*/
                chHandle->rxChNum     = UDMA_DMA_CH_INVALID;
                /* For BCDMA Block Copy, PeerThreadID is not applicable (no pairing required). */
                chHandle->peerThreadId = UDMA_THREAD_ID_INVALID;
            }
            else
            {
                /* RX channel same as TX channel for block copy */
                chHandle->rxChNum = chHandle->txChNum;
                /* Add thread offset and or RX relative channel as the thread
                 * already has bit info (CSL_PSILCFG_DEST_THREAD_OFFSET)
                 * for destination thread */
                chHandle->peerThreadId =
                    chHandle->rxChNum + drvHandle->udmapDestThreadOffset;
            }

        }
    }
    else
    {
        /* Allocate UDMAP for PDMA channels */
        if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            if((chHandle->chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
            {
                chHandle->txChNum =
                    Udma_rmAllocTxHcCh(chHandle->chPrms.chNum, drvHandle);
            }
            else if((chHandle->chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
            {
                chHandle->txChNum =
                    Udma_rmAllocTxUhcCh(chHandle->chPrms.chNum, drvHandle);
            }
            else
            {
                chHandle->txChNum =
                    Udma_rmAllocTxCh(chHandle->chPrms.chNum, drvHandle);
            }
            if(UDMA_DMA_CH_INVALID == chHandle->txChNum)
            {
                retVal = UDMA_EALLOC;
                DebugP_logError("[UDMA] RM Alloc TX Ch failed!!!\r\n");
            }
        }
        else
        {
            if((chHandle->chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
            {
                chHandle->rxChNum =
                    Udma_rmAllocRxHcCh(chHandle->chPrms.chNum, drvHandle);
            }
            else if((chHandle->chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
            {
                chHandle->rxChNum =
                    Udma_rmAllocRxUhcCh(chHandle->chPrms.chNum, drvHandle);
            }
            else
            {
                chHandle->rxChNum =
                    Udma_rmAllocRxCh(chHandle->chPrms.chNum, drvHandle);
            }
            if(UDMA_DMA_CH_INVALID == chHandle->rxChNum)
            {
                retVal = UDMA_EALLOC;
                DebugP_logError("[UDMA] RM Alloc RX Ch failed!!!\r\n");
            }
            else
            {
                /* Assign default flow */
                chHandle->defaultFlow               = &chHandle->defaultFlowObj;
                chHandle->defaultFlow->drvHandle    = drvHandle;
                chHandle->defaultFlow->flowStart    = chHandle->rxChNum;
                chHandle->defaultFlow->flowCnt      = 1U;
                chHandle->defaultFlow->flowInitDone = UDMA_INIT_DONE;
            }
        }

        if(UDMA_SOK == retVal)
        {
            /* Allocate peer channel for PDMA channels */
            if((chHandle->chType & UDMA_CH_FLAG_PDMA) == UDMA_CH_FLAG_PDMA)
            {
                /* PDMA peer channel assignment */
                chHandle->pdmaChNum = chHandle->chPrms.peerChNum;
                /* Thread ID already added while getting PDMA channel num */
                chHandle->peerThreadId = chHandle->pdmaChNum;
            }

            if((chHandle->chType & UDMA_CH_FLAG_PSIL) == UDMA_CH_FLAG_PSIL)
            {
                chHandle->peerThreadId = chHandle->chPrms.peerChNum;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Free queue ring number is same as UDMAP channel number */
        if(NULL_PTR != chHandle->chPrms.fqRingPrms.ringMem)
        {
            /* Allocate only when memory is provided */
            if((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) ==
                UDMA_CH_FLAG_BLK_COPY)
            {
                /* Same as TX channel incase of block copy */
                ringNum = (uint16_t)chHandle->txChNum;
            }
            else
            {
                if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
                {
                    /* For UDMAP, txChOffset is 0 */
                    ringNum = (uint16_t)(chHandle->txChNum + drvHandle->txChOffset);
                }
                else
                {
                    ringNum = (uint16_t)(chHandle->rxChNum + drvHandle->rxChOffset);
                }
            }

            chHandle->fqRing = &chHandle->fqRingObj;
            retVal = Udma_ringAlloc(
                         drvHandle,
                         chHandle->fqRing,
                         ringNum,
                         &chHandle->chPrms.fqRingPrms);
            if(UDMA_SOK != retVal)
            {
                chHandle->fqRing = (Udma_RingHandleInt) NULL_PTR;
                DebugP_logError("[UDMA] FQ ring alloc failed!!!\r\n");
            }
            else if(((chHandle->chType & UDMA_CH_FLAG_MAPPED) == UDMA_CH_FLAG_MAPPED) &&
                    ((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX))
            {
                /* Assign the default flow start id as the allocated default ring num(without offset) for mapped RX channels.
                 * This is because the default flow start idx is not equal to rxChNum,
                 * since there may be 1 to many mapping between RX Channels and dedicated flows */
                DebugP_assert(chHandle->fqRing->ringNum >= drvHandle->rxChOffset);
                chHandle->defaultFlow->flowStart    = chHandle->fqRing->ringNum - drvHandle->rxChOffset;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Allocate completion ring only when memory is provided */
        if(NULL_PTR != chHandle->chPrms.cqRingPrms.ringMem)
        {
            chHandle->cqRing = &chHandle->cqRingObj;
            retVal = Udma_ringAlloc(
                         drvHandle,
                         chHandle->cqRing,
                         UDMA_RING_ANY,
                         &chHandle->chPrms.cqRingPrms);
            if(UDMA_SOK != retVal)
            {
                chHandle->cqRing = (Udma_RingHandleInt) NULL_PTR;
                DebugP_logError("[UDMA] CQ ring alloc failed!!!\r\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Allocate teardown completion ring only when memory is provided */
        if(NULL_PTR != chHandle->chPrms.tdCqRingPrms.ringMem)
        {
            chHandle->tdCqRing = &chHandle->tdCqRingObj;
            retVal = Udma_ringAlloc(
                         drvHandle,
                         chHandle->tdCqRing,
                         UDMA_RING_ANY,
                         &chHandle->chPrms.tdCqRingPrms);
            if(UDMA_SOK != retVal)
            {
                chHandle->tdCqRing = (Udma_RingHandleInt) NULL_PTR;
                DebugP_logError("[UDMA] TD CQ ring alloc failed!!!\r\n");
            }
        }
    }

    if(UDMA_SOK != retVal)
    {
        tempRetVal = Udma_chFreeResource(chHandle);
        if(UDMA_SOK != tempRetVal)
        {
            DebugP_logError("[UDMA] Free resource failed!!!\r\n");
        }
    }
    else
    {
        /* Assign overlay pointers */
        Udma_chAssignRegOverlay(drvHandle, chHandle);
    }

    return (retVal);
}

static int32_t Udma_chFreeResource(Udma_ChHandleInt chHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;

    drvHandle = chHandle->drvHandle;
    if((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
    {
        if(UDMA_DMA_CH_INVALID != chHandle->txChNum)
        {
            /* TX channel free */
            if((chHandle->chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
            {
                Udma_rmFreeBlkCopyHcCh(chHandle->txChNum, drvHandle);
            }
            else if((chHandle->chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
            {
                Udma_rmFreeBlkCopyUhcCh(chHandle->txChNum, drvHandle);
            }
            else
            {
                Udma_rmFreeBlkCopyCh(chHandle->txChNum, drvHandle);
            }
            chHandle->txChNum = UDMA_DMA_CH_INVALID;
            chHandle->rxChNum = UDMA_DMA_CH_INVALID;
        }
    }
    else
    {
        if(UDMA_DMA_CH_INVALID != chHandle->txChNum)
        {
            /* TX channel free */
            if((chHandle->chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
            {
                Udma_rmFreeTxHcCh(chHandle->txChNum, drvHandle);
            }
            else if((chHandle->chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
            {
                Udma_rmFreeTxUhcCh(chHandle->txChNum, drvHandle);
            }
            else
            {
                Udma_rmFreeTxCh(chHandle->txChNum, drvHandle);
            }
            chHandle->txChNum = UDMA_DMA_CH_INVALID;
        }
        if(UDMA_DMA_CH_INVALID != chHandle->rxChNum)
        {
            /* RX channel free */
            if((chHandle->chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
            {
                Udma_rmFreeRxHcCh(chHandle->rxChNum, drvHandle);
            }
            else if((chHandle->chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
            {
                Udma_rmFreeRxUhcCh(chHandle->rxChNum, drvHandle);
            }
            else
            {
                Udma_rmFreeRxCh(chHandle->rxChNum, drvHandle);
            }
            chHandle->rxChNum = UDMA_DMA_CH_INVALID;
        }

        chHandle->defaultFlowObj.drvHandle    = (Udma_DrvHandleInt) NULL_PTR;
        chHandle->defaultFlowObj.flowStart    = UDMA_FLOW_INVALID;
        chHandle->defaultFlowObj.flowCnt      = 0U;
        chHandle->defaultFlowObj.flowInitDone = UDMA_DEINIT_DONE;
        chHandle->defaultFlow                 = (Udma_FlowHandleInt) NULL_PTR;
    }
    chHandle->pdmaChNum = UDMA_DMA_CH_INVALID;
    chHandle->peerThreadId = UDMA_THREAD_ID_INVALID;

    if(NULL_PTR != chHandle->fqRing)
    {
        retVal += Udma_ringFree(chHandle->fqRing);
        if(UDMA_SOK != retVal)
        {
            DebugP_logError("[UDMA] RM Free FQ ring failed!!!\r\n");
        }
        chHandle->fqRing = (Udma_RingHandleInt) NULL_PTR;
    }
    if(NULL_PTR != chHandle->cqRing)
    {
        retVal += Udma_ringFree(chHandle->cqRing);
        if(UDMA_SOK != retVal)
        {
            DebugP_logError("[UDMA] RM Free CQ ring failed!!!\r\n");
        }
        chHandle->cqRing = (Udma_RingHandleInt) NULL_PTR;
    }
    if(NULL_PTR != chHandle->tdCqRing)
    {
        retVal += Udma_ringFree(chHandle->tdCqRing);
        if(UDMA_SOK != retVal)
        {
            DebugP_logError("[UDMA] RM Free TDCQ ring failed!!!\r\n");
        }
        chHandle->tdCqRing = (Udma_RingHandleInt) NULL_PTR;
    }

    return (retVal);
}

static int32_t Udma_chPair(Udma_ChHandleInt chHandle)
{
    int32_t                 retVal = UDMA_SOK;
    Udma_DrvHandleInt       drvHandle;
    struct tisci_msg_rm_psil_pair_req rmPairReq;

    drvHandle = chHandle->drvHandle;

    if((UDMA_INST_TYPE_LCDMA_BCDMA                 == drvHandle->instType) &&
       ((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY))
    {
        /* For BCDMA Block Copy, pairing not required.*/
    }
    else
    {
        rmPairReq.nav_id = drvHandle->devIdPsil;
        /* Do TX check first so that TX becomes source thread for block copy */
        if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            DebugP_assert(chHandle->txChNum != UDMA_DMA_CH_INVALID);

            /* For TX, UDMAP channel is source */
            rmPairReq.src_thread = chHandle->txChNum + drvHandle->udmapSrcThreadOffset;
            rmPairReq.dst_thread = chHandle->peerThreadId;
        }
        else    /* RX channel */
        {
            DebugP_assert(chHandle->rxChNum != UDMA_DMA_CH_INVALID);

            /* For RX, UDMAP channel is destination */
            rmPairReq.src_thread = chHandle->peerThreadId;
            rmPairReq.dst_thread = chHandle->rxChNum + drvHandle->udmapDestThreadOffset;
        }

        /* Pair source thread with destination thread */
        retVal = Sciclient_rmPsilPair(&rmPairReq, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            DebugP_logError("[UDMA] RM PSI Pairing failed!!!\r\n");
        }
    }

    return (retVal);
}

static int32_t Udma_chUnpair(Udma_ChHandleInt chHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    struct tisci_msg_rm_psil_unpair_req rmUnpairReq;

    drvHandle = chHandle->drvHandle;

    if((UDMA_INST_TYPE_LCDMA_BCDMA                 == drvHandle->instType) &&
       ((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY))
    {
        /* For BCDMA Block Copy, un-pairing / thread disbale not required.*/
    }
    else
    {
        rmUnpairReq.nav_id = drvHandle->devIdPsil;
        /* Do TX check first so that TX becomes source thread for block copy */
        if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            DebugP_assert(chHandle->txChNum != UDMA_DMA_CH_INVALID);

            /* For TX, UDMAP channel is source */
            rmUnpairReq.src_thread = chHandle->txChNum + drvHandle->udmapSrcThreadOffset;
            rmUnpairReq.dst_thread = chHandle->peerThreadId;
        }
        else    /* RX channel */
        {
            DebugP_assert(chHandle->rxChNum != UDMA_DMA_CH_INVALID);

            /* For RX, UDMAP channel is destination */
            rmUnpairReq.src_thread = chHandle->peerThreadId;
            rmUnpairReq.dst_thread = chHandle->rxChNum + drvHandle->udmapDestThreadOffset;
        }

        /* Unpair source thread with destination thread */
        retVal = Sciclient_rmPsilUnpair(&rmUnpairReq, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            DebugP_logError("[UDMA] RM PSI Un-Pairing failed!!!\r\n");
        }
    }

    return (retVal);
}

static void Udma_chEnableLocal(Udma_ChHandleInt chHandle)
{
    uint32_t                regVal;
    Udma_DrvHandleInt       drvHandle;
    drvHandle = chHandle->drvHandle;
    CSL_UdmapRT             udmapRtEnable;

    if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
    {
       /* Set only enable and clear all other flags which might be set from
        * previous run */
        udmapRtEnable.enable         = TRUE;
        udmapRtEnable.teardown       = FALSE;
        udmapRtEnable.forcedTeardown = FALSE;
        udmapRtEnable.pause          = FALSE;
        udmapRtEnable.error          = FALSE;

        if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            Udma_assert(drvHandle, chHandle->pTxRtRegs != NULL_PTR);
            Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);

            regVal = CSL_REG32_RD(&chHandle->pTxRtRegs->PEER8);
            CSL_FINS(regVal, PSILCFG_REG_RT_ENABLE_ENABLE, (uint32_t) 1U);
            CSL_REG32_WR(&chHandle->pTxRtRegs->PEER8, regVal);

            (void) CSL_udmapSetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &udmapRtEnable);
        }

        if((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
        {
            Udma_assert(drvHandle, chHandle->pRxRtRegs != NULL_PTR);
            Udma_assert(drvHandle, chHandle->rxChNum != UDMA_DMA_CH_INVALID);

            /*
            * Note: UDMAP shoule be enabled first (receiver) before enabling
            *       PEER/PDMA through PSIL (source)
            *       This ensures UDMAP RX is ready to receive data
            */
            /* Note: Block copy uses same RX channel. So do for both TX/RX */
            (void) CSL_udmapSetRxRT(
                &drvHandle->udmapRegs, chHandle->rxChNum, &udmapRtEnable);

            regVal = CSL_REG32_RD(&chHandle->pRxRtRegs->PEER8);
            CSL_FINS(regVal, PSILCFG_REG_RT_ENABLE_ENABLE, (uint32_t) 1U);
            CSL_REG32_WR(&chHandle->pRxRtRegs->PEER8, regVal);
        }
    }

    return;
}

static int32_t Udma_chDisableBlkCpyChan(Udma_ChHandleInt chHandle, uint32_t timeout)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            currTimeout = 0U;
    Udma_DrvHandleInt   drvHandle;
    CSL_UdmapRT     udmapRtStatus;

    drvHandle = chHandle->drvHandle;
    DebugP_assert(chHandle->txChNum != UDMA_DMA_CH_INVALID);

    /* Initiate graceful teardown first - Source is udma thread for TX */
    if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
    {
        retVal = CSL_udmapTeardownTxChan(
                    &drvHandle->udmapRegs, chHandle->txChNum, (bool)false, (bool)false);
    }

    if(CSL_PASS != retVal)
    {
        DebugP_logError("[UDMA] Blkcpy teardown failed!!\r\n");
    }

    /* Check if graceful teardown is complete */
    while(UDMA_SOK == retVal)
    {
        if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
        {
            (void) CSL_udmapGetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &udmapRtStatus);
            if(FALSE == udmapRtStatus.enable)
            {
                /* Teardown complete */
                break;
            }
        }
        if(currTimeout > timeout)
        {
            retVal = UDMA_ETIMEOUT;
        }
        else
        {
            (void) ClockP_usleep(1000U);
            currTimeout++;
        }
    }

    if(UDMA_SOK != retVal)
    {
        /* Graceful teardown failed - initiate force teardown */
        if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
        {
            retVal = CSL_udmapTeardownTxChan(
                        &drvHandle->udmapRegs, chHandle->txChNum, (bool)true, (bool)false);
        }
        if(CSL_PASS != retVal)
        {
            DebugP_logError("[UDMA] Blkcpy force disable failed!!\r\n");
        }

        /* Wait for disable to complete */
        currTimeout = 0U;
        while(UDMA_SOK == retVal)
        {
            if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
            {
                (void) CSL_udmapGetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &udmapRtStatus);
                if(FALSE == udmapRtStatus.enable)
                {
                    /* Teardown complete */
                    break;
                }
            }
            if(currTimeout > timeout)
            {
                retVal = UDMA_ETIMEOUT;
                DebugP_logError("[UDMA] Blockcpy ch teardown timed out!!!\r\n");
            }
            else
            {
                (void) ClockP_usleep(1000U);
                currTimeout++;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
        {
            /* Clear teardown and enable bits in UDMAP */
            udmapRtStatus.enable   = FALSE;
            udmapRtStatus.teardown = FALSE;
            udmapRtStatus.forcedTeardown = FALSE;
            (void) CSL_udmapSetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &udmapRtStatus);
        }
    }

    return (retVal);
}

static int32_t Udma_chDisableTxChan(Udma_ChHandleInt chHandle, uint32_t timeout)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            peerRtEnable = 0U, currTimeout = 0U;
    Udma_DrvHandleInt   drvHandle;
    CSL_UdmapRT         udmapRtStatus;
    uint32_t            rtEnableRegOffset;

    drvHandle = chHandle->drvHandle;
    DebugP_assert(chHandle->txChNum != UDMA_DMA_CH_INVALID);
    Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);
    rtEnableRegOffset = CSL_PSILCFG_REG_RT_ENABLE - CSL_PSILCFG_REG_STATIC_TR;

    /* Initiate graceful teardown first - Source is udma thread for TX */
    if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
    {
        retVal = CSL_udmapTeardownTxChan(
                    &drvHandle->udmapRegs, chHandle->txChNum, (bool)false, (bool)false);
    }

    if(CSL_PASS != retVal)
    {
        DebugP_logError("[UDMA] TX teardown failed!!\r\n");
    }

    /* Check if graceful teardown is complete */
    while(UDMA_SOK == retVal)
    {
        if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
        {
            (void) CSL_udmapGetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &udmapRtStatus);
            if(FALSE == udmapRtStatus.enable)
            {
                /* Teardown complete */
                break;
            }
        }
        if(currTimeout > timeout)
        {
            retVal = UDMA_ETIMEOUT;
        }
        else
        {
            (void) ClockP_usleep(1000U);
            currTimeout++;
        }
    }

    if(UDMA_SOK != retVal)
    {
        /* Graceful teardown failed - initiate force teardown */
        if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
        {
            retVal = CSL_udmapTeardownTxChan(
                        &drvHandle->udmapRegs, chHandle->txChNum, (bool)true, (bool)false);
        }
        if(CSL_PASS != retVal)
        {
            DebugP_logError("[UDMA] TX force disable failed!!\r\n");
        }

        if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
        {
            /* Set flush in peer */
            (void) CSL_udmapGetChanPeerReg(
                &drvHandle->udmapRegs,
                chHandle->txChNum,
                CSL_UDMAP_CHAN_DIR_TX,
                rtEnableRegOffset,
                &peerRtEnable);
            CSL_FINS(peerRtEnable, PSILCFG_REG_RT_ENABLE_FLUSH, (uint32_t) 1U);
            (void) CSL_udmapSetChanPeerReg(
                &drvHandle->udmapRegs,
                chHandle->txChNum,
                CSL_UDMAP_CHAN_DIR_TX,
                rtEnableRegOffset,
                &peerRtEnable);
        }
        /* Wait for disable to complete */
        currTimeout = 0U;
        while(UDMA_SOK == retVal)
        {
            if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
            {
                (void) CSL_udmapGetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &udmapRtStatus);
                (void) CSL_udmapGetChanPeerReg(
                    &drvHandle->udmapRegs,
                    chHandle->txChNum,
                    CSL_UDMAP_CHAN_DIR_TX,
                    rtEnableRegOffset, &peerRtEnable);
                if((FALSE == udmapRtStatus.enable) &&
                (CSL_FEXT(peerRtEnable, PSILCFG_REG_RT_ENABLE_ENABLE) == FALSE))
                {
                    /* Teardown complete */
                    break;
                }
            }
            if(currTimeout > timeout)
            {
                retVal = UDMA_ETIMEOUT;
                DebugP_logError("[UDMA] TX ch teardown timed out!!!\r\n");
            }
            else
            {
                (void) ClockP_usleep(1000U);
                currTimeout++;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
        {
            /* Clear teardown and enable bits in both UDMAP and peer */
            udmapRtStatus.enable   = FALSE;
            udmapRtStatus.teardown = FALSE;
            udmapRtStatus.forcedTeardown = FALSE;
            CSL_FINS(peerRtEnable, PSILCFG_REG_RT_ENABLE_TDOWN, (uint32_t) 0U);
            (void) CSL_udmapSetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &udmapRtStatus);
            (void) CSL_udmapSetChanPeerReg(
                &drvHandle->udmapRegs,
                chHandle->txChNum,
                CSL_UDMAP_CHAN_DIR_TX,
                rtEnableRegOffset,
                &peerRtEnable);
        }
    }

    return (retVal);
}

static int32_t Udma_chDisableRxChan(Udma_ChHandleInt chHandle, uint32_t timeout)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            currTimeout = 0U, regVal;
    Udma_DrvHandleInt   drvHandle;
    CSL_UdmapRT         udmapRtStatus;
    uint32_t            peerRtEnable = 0U, peerRtEnableBit = 0U;
    uint32_t            rtEnableRegOffset;

    drvHandle = chHandle->drvHandle;
    DebugP_assert(chHandle->rxChNum != UDMA_DMA_CH_INVALID);
    DebugP_assert(chHandle->peerThreadId != UDMA_THREAD_ID_INVALID);
    rtEnableRegOffset = CSL_PSILCFG_REG_RT_ENABLE - CSL_PSILCFG_REG_STATIC_TR;

    /* Initiate graceful teardown first - Source is peer thread for RX */
    if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
    {
        regVal = CSL_REG32_RD(&chHandle->pRxRtRegs->PEER8);
        CSL_FINS(regVal, PSILCFG_REG_RT_ENABLE_TDOWN, (uint32_t) 1U);
        CSL_REG32_WR(&chHandle->pRxRtRegs->PEER8, regVal);
    }

    /* Check if graceful teardown is complete */
    while(UDMA_SOK == retVal)
    {
        if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
        {
            (void) CSL_udmapGetRxRT(&drvHandle->udmapRegs, chHandle->rxChNum, &udmapRtStatus);
            if(FALSE == udmapRtStatus.enable)
            {
                /* Teardown complete */
                break;
            }
        }
        if(currTimeout > timeout)
        {
            retVal = UDMA_ETIMEOUT;
        }
        else
        {
            (void) ClockP_usleep(1000U);
            currTimeout++;
        }
    }

    if(UDMA_SOK != retVal)
    {
        /* Graceful teardown failed - initiate force teardown */
        if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
        {
            retVal = CSL_udmapTeardownRxChan(
                        &drvHandle->udmapRegs, chHandle->rxChNum, (bool)true, (bool)false);
        }
        if(CSL_PASS != retVal)
        {
            DebugP_logError("[UDMA] RX force disable failed!!\r\n");
        }

        /* Wait for disable to complete - both locally and for peer thread */
        currTimeout = 0U;
        while(UDMA_SOK == retVal)
        {
            if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
            {
                (void) CSL_udmapGetRxRT(&drvHandle->udmapRegs, chHandle->rxChNum, &udmapRtStatus);
                (void) CSL_udmapGetChanPeerReg(
                    &drvHandle->udmapRegs,
                    chHandle->rxChNum,
                    CSL_UDMAP_CHAN_DIR_RX,
                    rtEnableRegOffset, &peerRtEnable);
                peerRtEnableBit = CSL_FEXT(peerRtEnable, PSILCFG_REG_RT_ENABLE_ENABLE);
               if((FALSE == udmapRtStatus.enable) && (FALSE == peerRtEnableBit))
                {
                    /* Teardown complete */
                    break;
                }
            }
            if(currTimeout > timeout)
            {
                retVal = UDMA_ETIMEOUT;
                DebugP_logError("[UDMA] RX ch teardown timed out!!!\r\n");
            }
            else
            {
                (void) ClockP_usleep(1000U);
                currTimeout++;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
        {
            /* Clear teardown bits in both the UDMAP and peer */
            udmapRtStatus.teardown = FALSE;   /* Note that other bits are cleared from previous call */
            CSL_FINS(peerRtEnable, PSILCFG_REG_RT_ENABLE_TDOWN, (uint32_t) FALSE);
            (void) CSL_udmapSetRxRT(
                &drvHandle->udmapRegs, chHandle->rxChNum, &udmapRtStatus);
            (void) CSL_udmapSetChanPeerReg(
                &drvHandle->udmapRegs,
                chHandle->rxChNum,
                CSL_UDMAP_CHAN_DIR_RX,
                rtEnableRegOffset,
                &peerRtEnable);
        }
    }

    return (retVal);
}

static void Udma_chAssignRegOverlay(Udma_DrvHandleInt drvHandle, Udma_ChHandleInt chHandle)
{
    if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
    {
        if(chHandle->txChNum != UDMA_DMA_CH_INVALID)
        {
            Udma_assert(drvHandle, drvHandle->udmapRegs.pTxChanCfgRegs != NULL_PTR);
            Udma_assert(drvHandle, drvHandle->udmapRegs.pTxChanRtRegs != NULL_PTR);
            Udma_assert(drvHandle,
                chHandle->txChNum <
                    (sizeof(CSL_udmap_txcrtRegs) /
                        sizeof(CSL_udmap_txcrtRegs_chan)));
            chHandle->pTxCfgRegs =
                &drvHandle->udmapRegs.pTxChanCfgRegs->CHAN[chHandle->txChNum];
            chHandle->pTxRtRegs  =
                &drvHandle->udmapRegs.pTxChanRtRegs->CHAN[chHandle->txChNum];
        }
        if(chHandle->rxChNum != UDMA_DMA_CH_INVALID)
        {
            Udma_assert(drvHandle, drvHandle->udmapRegs.pRxChanCfgRegs != NULL_PTR);
            Udma_assert(drvHandle, drvHandle->udmapRegs.pRxChanRtRegs != NULL_PTR);
            Udma_assert(drvHandle,
                chHandle->rxChNum <
                    (sizeof(CSL_udmap_rxcrtRegs) /
                        sizeof(CSL_udmap_rxcrtRegs_chan)));
            chHandle->pRxCfgRegs =
                &drvHandle->udmapRegs.pRxChanCfgRegs->CHAN[chHandle->rxChNum];
            chHandle->pRxRtRegs  =
                &drvHandle->udmapRegs.pRxChanRtRegs->CHAN[chHandle->rxChNum];
        }
        if(chHandle->extChNum != UDMA_DMA_CH_INVALID)
        {
            Udma_assert(drvHandle, drvHandle->udmapRegs.pTxChanCfgRegs != NULL_PTR);
            Udma_assert(drvHandle, drvHandle->udmapRegs.pTxChanRtRegs != NULL_PTR);
            Udma_assert(drvHandle,
                (chHandle->extChNum + drvHandle->extChOffset) <
                    (sizeof(CSL_udmap_txcrtRegs) /
                        sizeof(CSL_udmap_txcrtRegs_chan)));
            chHandle->pExtCfgRegs =
                &drvHandle->udmapRegs.pTxChanCfgRegs->CHAN[
                                chHandle->extChNum + drvHandle->extChOffset];
            chHandle->pExtRtRegs  =
                &drvHandle->udmapRegs.pTxChanRtRegs->CHAN[
                                chHandle->extChNum + drvHandle->extChOffset];
        }
    }
}

static void Udma_chInitRegs(Udma_ChHandleInt chHandle)
{
    chHandle->pTxCfgRegs        = (volatile CSL_udmap_txccfgRegs_chan *) NULL_PTR;
    chHandle->pTxRtRegs         = (volatile CSL_udmap_txcrtRegs_chan *) NULL_PTR;
    chHandle->pRxCfgRegs        = (volatile CSL_udmap_rxccfgRegs_chan *) NULL_PTR;
    chHandle->pRxRtRegs         = (volatile CSL_udmap_rxcrtRegs_chan *) NULL_PTR;
    chHandle->pExtCfgRegs       = (volatile CSL_udmap_txccfgRegs_chan *) NULL_PTR;
    chHandle->pExtRtRegs        = (volatile CSL_udmap_txcrtRegs_chan *) NULL_PTR;
}


static void Udma_chPauseTxLocal(Udma_DrvHandleInt drvHandle, uint32_t txChNum,uint32_t chType)
{
    if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
    {
        (void) CSL_udmapPauseTxChan(&drvHandle->udmapRegs, txChNum);
    }
}

static void Udma_chUnpauseTxLocal(Udma_DrvHandleInt drvHandle, uint32_t txChNum, uint32_t chType)
{
    if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
    {
        (void) CSL_udmapUnpauseTxChan(&drvHandle->udmapRegs, txChNum);
    }
}

static void Udma_chPauseRxLocal(Udma_DrvHandleInt drvHandle, uint32_t rxChNum)
{
    if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
    {
        (void) CSL_udmapPauseRxChan(&drvHandle->udmapRegs, rxChNum);
    }
}

static void Udma_chUnpauseRxLocal(Udma_DrvHandleInt drvHandle, uint32_t rxChNum)
{
    if(UDMA_INST_TYPE_NORMAL == drvHandle->instType)
    {
        (void) CSL_udmapUnpauseRxChan(&drvHandle->udmapRegs, rxChNum);
    }
}

static void Udma_chSetPeerReg(Udma_DrvHandleInt drvHandle,
                              const Udma_ChPdmaPrms *pdmaPrms,
                              volatile uint32_t *PEER8,
                              volatile uint32_t *PEER1,
                              volatile uint32_t *PEER0)
{
    uint32_t        regVal;

    DebugP_assert(PEER8 != NULL_PTR);
    regVal = CSL_REG32_RD(PEER8);
    CSL_FINS(regVal, PSILCFG_REG_RT_ENABLE_ENABLE, (uint32_t) 0U);
    CSL_REG32_WR(PEER8, regVal);

    DebugP_assert(PEER0 != NULL_PTR);
    regVal = CSL_FMK(PSILCFG_REG_STATIC_TR_X, pdmaPrms->elemSize) |
                CSL_FMK(PSILCFG_REG_STATIC_TR_Y, pdmaPrms->elemCnt);
    CSL_REG32_WR(PEER0, regVal);

    DebugP_assert(PEER1 != NULL_PTR);
    regVal = CSL_FMK(PSILCFG_REG_STATIC_TR_Z, pdmaPrms->fifoCnt);
    CSL_REG32_WR(PEER1, regVal);
}

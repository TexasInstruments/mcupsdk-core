/*
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
 */

#include <string.h>
#include <drivers/udma.h>
#include <drivers/mcspi/v0/dma/mcspi_dma.h>
#include <drivers/mcspi.h>
#include <kernel/dpl/CacheP.h>

/* Static Function Declarations */
static int32_t MCSPI_udmaOpen(void* mcspiDmaArgs);
static int32_t MCSPI_udmaChInit(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg,
                                const MCSPI_DmaChConfig *dmaChCfg);
static int32_t MCSPI_udmaClose(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg);
static int32_t MCSPI_udmaTransfer(MCSPI_Object *obj,
                                        MCSPI_ChObject *chObj,
                                        const MCSPI_Attrs *attrs,
                                        MCSPI_Transaction *transaction);
static void MCSPI_udmaHpdInit(Udma_ChHandle rxChHandle,
                              uint8_t *pHpdMem,
                              const void *destBuf,
                              uint32_t length);
static void MCSPI_udmaIsrTx(Udma_EventHandle eventHandle,
                                  uint32_t eventType,
                                  void *args);
static void MCSPI_udmaIsrRx(Udma_EventHandle eventHandle,
                                  uint32_t eventType,
                                  void *args);
static int32_t MCSPI_udmaStop(MCSPI_Object *obj, const MCSPI_Attrs *attrs,
                              MCSPI_ChObject *chObj, uint32_t chNum);
static int32_t MCSPI_udmaInitRxCh(MCSPI_Handle handle, MCSPI_ChObject *chObj);
static int32_t MCSPI_udmaInitTxCh(MCSPI_Handle handle, MCSPI_ChObject *chObj);
static int32_t MCSPI_udmaDeInitCh(Udma_ChHandle chHandle,
                                  Udma_EventHandle eventHandle);
static int32_t MCSPI_udmaConfigPdmaRx(MCSPI_Object *obj,
                                      MCSPI_ChObject *chObj,
                                      MCSPI_Transaction *transaction,
                                      uint32_t numWords,
                                      uint8_t *rxBufPtr);
static int32_t MCSPI_udmaConfigPdmaTx(MCSPI_Object *obj,
                                      MCSPI_ChObject *chObj,
                                      MCSPI_Transaction *transaction,
                                      uint32_t numWords,
                                      const uint8_t *txBufPtr);
static void MCSPI_udmaStart(MCSPI_ChObject *chObj, const MCSPI_Attrs *attrs,
                            uint32_t baseAddr);

/* UDMA Function Pointers */
MCSPI_DmaFxns gMcspiDmaUdmaFxns =
{
    .dmaOpenFxn = MCSPI_udmaOpen,
    .dmaCloseFxn = MCSPI_udmaClose,
    .dmaChInitFxn = MCSPI_udmaChInit,
    .dmaTransferMasterFxn = MCSPI_udmaTransfer,
    .dmaStopFxn = MCSPI_udmaStop,
};

static int32_t MCSPI_udmaOpen(void* mcspiDmaArgs)
{
    return SystemP_SUCCESS;
}

static int32_t MCSPI_udmaChInit(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg,
                                const MCSPI_DmaChConfig *dmaChCfg)
{
    int32_t status = SystemP_SUCCESS;
    MCSPI_Object   *obj;
    MCSPI_Config   *config;
    MCSPI_ChObject *chObj;

    config = (MCSPI_Config *) handle;
    obj = config->object;
    chObj = &obj->chObj[chCfg->chNum];
    memcpy(&chObj->dmaChCfg, dmaChCfg, sizeof(MCSPI_DmaChConfig));

    if(MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode)
    {
        status  = MCSPI_udmaInitRxCh(handle, chObj);
        status |= MCSPI_udmaInitTxCh(handle, chObj);
    }
    else if(MCSPI_TR_MODE_TX_ONLY == chObj->chCfg.trMode)
    {
        status = MCSPI_udmaInitTxCh(handle, chObj);
    }
    else if(MCSPI_TR_MODE_RX_ONLY == chObj->chCfg.trMode)
    {
        status = MCSPI_udmaInitRxCh(handle, chObj);
    }
    chObj->dmaChCfg.isOpen = TRUE;

    return status;
}

static int32_t MCSPI_udmaClose(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg)
{
    int32_t status = SystemP_SUCCESS;
    MCSPI_Config   *config;
    MCSPI_Object   *obj;
    MCSPI_ChObject *chObj;

    /* Check parameters */
    if((NULL == handle) ||
       (NULL == chCfg) ||
       (chCfg->chNum >= MCSPI_MAX_NUM_CHANNELS))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config = (MCSPI_Config *) handle;
        obj = config->object;
        chObj = &obj->chObj[chCfg->chNum];

        if (chObj->dmaChCfg.isOpen != FALSE)
        {
            if(MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode)
            {
                MCSPI_udmaDeInitCh(chObj->dmaChCfg.rxChHandle,
                                   chObj->dmaChCfg.cqRxEvtHandle);
                MCSPI_udmaDeInitCh(chObj->dmaChCfg.txChHandle,
                                   chObj->dmaChCfg.cqTxEvtHandle);
            }
            else if(MCSPI_TR_MODE_TX_ONLY == chObj->chCfg.trMode)
            {
                MCSPI_udmaDeInitCh(chObj->dmaChCfg.txChHandle,
                                   chObj->dmaChCfg.cqTxEvtHandle);
            }
            else if(MCSPI_TR_MODE_RX_ONLY == chObj->chCfg.trMode)
            {
                MCSPI_udmaDeInitCh(chObj->dmaChCfg.rxChHandle,
                                   chObj->dmaChCfg.cqRxEvtHandle);
            }
            chObj->dmaChCfg.isOpen = FALSE;
        }
    }

    return status;
}

static int32_t MCSPI_udmaTransfer(MCSPI_Object *obj,
                                MCSPI_ChObject *chObj,
                                const MCSPI_Attrs *attrs,
                                MCSPI_Transaction *transaction)
{
    int32_t status = SystemP_SUCCESS;

    if(MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode)
    {
        if ((chObj->curRxBufPtr != NULL) && (chObj->curTxBufPtr != NULL))
        {
            status  = MCSPI_udmaConfigPdmaRx(obj, chObj, transaction,
                                            transaction->count, chObj->curRxBufPtr);
            status |= MCSPI_udmaConfigPdmaTx(obj, chObj, transaction,
                                             transaction->count, chObj->curTxBufPtr);
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg.trMode)
    {
        if (chObj->curTxBufPtr != NULL)
        {
            status  = MCSPI_udmaConfigPdmaTx(obj, chObj, transaction,
                                         transaction->count, chObj->curTxBufPtr);
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else if (MCSPI_TR_MODE_RX_ONLY == chObj->chCfg.trMode)
    {
        if (chObj->curRxBufPtr != NULL)
        {
            status  = MCSPI_udmaConfigPdmaRx(obj, chObj, transaction,
                                        transaction->count, chObj->curRxBufPtr);
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    /* Initiate Transfer */
    MCSPI_udmaStart(chObj, attrs, obj->baseAddr);

    return (status);
}

static void MCSPI_udmaHpdInit(Udma_ChHandle chHandle,
                              uint8_t       *pHpdMem,
                              const void    *destBuf,
                              uint32_t      length)
{
    CSL_UdmapCppi5HMPD *pHpd = (CSL_UdmapCppi5HMPD *) pHpdMem;
    uint32_t descType = (uint32_t)CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST;

    /* Setup descriptor */
    CSL_udmapCppi5SetDescType(pHpd, descType);
    CSL_udmapCppi5SetEpiDataPresent(pHpd, FALSE);
    CSL_udmapCppi5SetPsDataLoc(pHpd, 0U);
    CSL_udmapCppi5SetPsDataLen(pHpd, 0U);
    CSL_udmapCppi5SetPktLen(pHpd, descType, length);
    CSL_udmapCppi5SetPsFlags(pHpd, 0U);
    CSL_udmapCppi5SetIds(pHpd, descType, 0x321, UDMA_DEFAULT_FLOW_ID);
    CSL_udmapCppi5SetSrcTag(pHpd, 0x0000);     /* Not used */
    CSL_udmapCppi5SetDstTag(pHpd, 0x0000);     /* Not used */
    /* Return Policy descriptors are reserved in case of AM243X/Am64X */
    CSL_udmapCppi5SetReturnPolicy(
        pHpd,
        descType,
        0U,
        0U,
        0U,
        0U);
    CSL_udmapCppi5LinkDesc(pHpd, 0U);
    CSL_udmapCppi5SetBufferAddr(pHpd, (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL));
    CSL_udmapCppi5SetBufferLen(pHpd, length);
    CSL_udmapCppi5SetOrgBufferAddr(pHpd, (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL));
    CSL_udmapCppi5SetOrgBufferLen(pHpd, length);

    /* Writeback cache */
    CacheP_wb(pHpdMem, sizeof(CSL_UdmapCppi5HMPD), CacheP_TYPE_ALLD);

    return;
}

static int32_t MCSPI_udmaInitRxCh(MCSPI_Handle handle, MCSPI_ChObject *chObj)
{
    int32_t             retVal;
    MCSPI_Config        *config;
    MCSPI_Object        *obj;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_DrvHandle      drvHandle;
    Udma_ChHandle       rxChHandle;
    MCSPI_DmaConfig     *dmaConfig;
    McspiDma_UdmaArgs   *udmaArgs;

    config      = (MCSPI_Config *) handle;
    obj         = config->object;
    dmaConfig   = (MCSPI_DmaConfig *)obj->mcspiDmaHandle;
    udmaArgs    = (McspiDma_UdmaArgs *)dmaConfig->mcspiDmaArgs;
    drvHandle   = udmaArgs->drvHandle;

    /* Init RX channel parameters */
    chType = UDMA_CH_TYPE_PDMA_RX;
    UdmaChPrms_init(&chPrms, chType);
    chPrms.peerChNum                = chObj->dmaChCfg.rxEvtNum;
    chPrms.fqRingPrms.ringMem       = chObj->dmaChCfg.rxRingMem;
    chPrms.fqRingPrms.ringMemSize   = chObj->dmaChCfg.ringMemSize;
    chPrms.fqRingPrms.elemCnt       = chObj->dmaChCfg.ringElemCnt;
    rxChHandle                      = chObj->dmaChCfg.rxChHandle;

    /* Open channel for block copy */
    retVal = Udma_chOpen(drvHandle, rxChHandle, chType, &chPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config RX channel */
    UdmaChRxPrms_init(&rxPrms, chType);
    retVal = Udma_chConfigRx(rxChHandle, &rxPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Register ring completion callback */
    eventHandle = chObj->dmaChCfg.cqRxEvtHandle;
    UdmaEventPrms_init(&eventPrms);
    eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    eventPrms.chHandle          = rxChHandle;
    eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
    eventPrms.eventCb           = &MCSPI_udmaIsrRx;
    eventPrms.appData           = (void *) config;
    retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);

    return retVal;
}

static int32_t MCSPI_udmaInitTxCh(MCSPI_Handle handle, MCSPI_ChObject *chObj)
{
    int32_t             retVal;
    MCSPI_Config        *config;
    MCSPI_Object        *obj;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChTxPrms       txPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_DrvHandle      drvHandle;
    Udma_ChHandle       txChHandle;
    MCSPI_DmaConfig     *dmaConfig;
    McspiDma_UdmaArgs   *udmaArgs;

    config      = (MCSPI_Config *) handle;
    obj         = config->object;
    dmaConfig   = (MCSPI_DmaConfig *)obj->mcspiDmaHandle;
    udmaArgs    = (McspiDma_UdmaArgs *)dmaConfig->mcspiDmaArgs;
    drvHandle   = udmaArgs->drvHandle;

    /* Init TX channel parameters */
    chType = UDMA_CH_TYPE_PDMA_TX;
    UdmaChPrms_init(&chPrms, chType);
    chPrms.peerChNum                = chObj->dmaChCfg.txEvtNum;
    chPrms.fqRingPrms.ringMem       = chObj->dmaChCfg.txRingMem;
    chPrms.fqRingPrms.ringMemSize   = chObj->dmaChCfg.ringMemSize;
    chPrms.fqRingPrms.elemCnt       = chObj->dmaChCfg.ringElemCnt;
    txChHandle                      = chObj->dmaChCfg.txChHandle;

    /* Open channel for block copy */
    retVal = Udma_chOpen(drvHandle, txChHandle, chType, &chPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config TX channel */
    UdmaChTxPrms_init(&txPrms, chType);
    retVal = Udma_chConfigTx(txChHandle, &txPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Register ring completion callback */
    eventHandle = chObj->dmaChCfg.cqTxEvtHandle;
    UdmaEventPrms_init(&eventPrms);
    eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    eventPrms.chHandle          = txChHandle;
    eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
    eventPrms.eventCb           = &MCSPI_udmaIsrTx;
    eventPrms.appData           = (void *) config;
    retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);

    return retVal;
}

static int32_t MCSPI_udmaDeInitCh(Udma_ChHandle chHandle,
                                  Udma_EventHandle eventHandle)
{
    int32_t status = UDMA_SOK;

    /* Disable Channel */
    status = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == status);

    /* UnRegister Event */
    status = Udma_eventUnRegister(eventHandle);
    DebugP_assert(UDMA_SOK == status);

    /* Flush any pending request from the free queue */
    while(1)
    {
        uint64_t pDesc;
        int32_t  tempRetVal;

        tempRetVal = Udma_ringFlushRaw(
                         Udma_chGetFqRingHandle(chHandle), &pDesc);
        if(UDMA_ETIMEOUT == tempRetVal)
        {
            break;
        }
    }

    /* Close channel */
    status = Udma_chClose(chHandle);
    DebugP_assert(UDMA_SOK == status);

    return status;
}

static int32_t MCSPI_udmaConfigPdmaTx(MCSPI_Object *obj,
                                      MCSPI_ChObject *chObj,
                                      MCSPI_Transaction *transaction,
                                      uint32_t numWords,
                                      const uint8_t *txBufPtr)
{
    int32_t             retVal;
    Udma_ChPdmaPrms     pdmaPrms;
    Udma_ChHandle       txChHandle;

    txChHandle  = chObj->dmaChCfg.txChHandle;

    /* Config PDMA channel */
    UdmaChPdmaPrms_init(&pdmaPrms);
    /* If chObj->bufWidthShift 0 - UDMA_PDMA_ES_8BITS
     *    chObj->bufWidthShift 1 - UDMA_PDMA_ES_16BITS
     *    chObj->bufWidthShift 2 - UDMA_PDMA_ES_32BITS */
    if (chObj->bufWidthShift < 2U)
    {
        pdmaPrms.elemSize = chObj->bufWidthShift;
    }
    else
    {
        pdmaPrms.elemSize = UDMA_PDMA_ES_32BITS;
    }

    /* Number of words received in each transfer */
    pdmaPrms.elemCnt = 1U;
    /* Dont care for Tx */
    pdmaPrms.fifoCnt    = 0U;

    retVal = Udma_chConfigPdma(txChHandle, &pdmaPrms);
    DebugP_assert(UDMA_SOK == retVal);

    retVal = Udma_chEnable(txChHandle);
    DebugP_assert(UDMA_SOK == retVal);

    /* Update host packet descriptor, length should be always in terms of total number of bytes */
    MCSPI_udmaHpdInit(txChHandle, (uint8_t *) chObj->dmaChCfg.txHpdMem, txBufPtr, (numWords << chObj->bufWidthShift));

    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(txChHandle),
                 (uint64_t) Udma_defaultVirtToPhyFxn(chObj->dmaChCfg.txHpdMem, 0U, NULL));
    DebugP_assert(UDMA_SOK == retVal);

    return (retVal);
}

static int32_t MCSPI_udmaConfigPdmaRx(MCSPI_Object *obj,
                                      MCSPI_ChObject *chObj,
                                      MCSPI_Transaction *transaction,
                                      uint32_t numWords,
                                      uint8_t *rxBufPtr)
{
    int32_t             retVal;
    Udma_ChPdmaPrms     pdmaPrms;
    Udma_ChHandle       rxChHandle;

    rxChHandle  = chObj->dmaChCfg.rxChHandle;

    /* Config PDMA channel */
    UdmaChPdmaPrms_init(&pdmaPrms);
    /* If chObj->bufWidthShift 0 - UDMA_PDMA_ES_8BITS
     *    chObj->bufWidthShift 1 - UDMA_PDMA_ES_16BITS
     *    chObj->bufWidthShift 2 - UDMA_PDMA_ES_32BITS */
    if (chObj->bufWidthShift < 2U)
    {
        pdmaPrms.elemSize = chObj->bufWidthShift;
    }
    else
    {
        pdmaPrms.elemSize = UDMA_PDMA_ES_32BITS;
    }

    /* Number of words received in each transfer */
    pdmaPrms.elemCnt = 1U;
    pdmaPrms.fifoCnt = numWords;

    retVal = Udma_chConfigPdma(rxChHandle, &pdmaPrms);
    DebugP_assert(UDMA_SOK == retVal);

    retVal = Udma_chEnable(rxChHandle);
    DebugP_assert(UDMA_SOK == retVal);

    /* Update host packet descriptor, length should be always in terms of total number of bytes */
    MCSPI_udmaHpdInit(rxChHandle, (uint8_t *) chObj->dmaChCfg.rxHpdMem, rxBufPtr, (numWords << chObj->bufWidthShift));

    /* Submit HPD to channel */
    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(rxChHandle),
                 (uint64_t) Udma_defaultVirtToPhyFxn(chObj->dmaChCfg.rxHpdMem, 0U, NULL));
    DebugP_assert(UDMA_SOK == retVal);

    return (retVal);
}

static int32_t MCSPI_udmaStop(MCSPI_Object *obj, const MCSPI_Attrs *attrs,
                              MCSPI_ChObject *chObj, uint32_t chNum)
{
    uint32_t baseAddr;
    int32_t status = SystemP_SUCCESS;

    baseAddr = obj->baseAddr;
    if(MCSPI_MS_MODE_MASTER == obj->openPrms.msMode)
    {
        /* Manual CS de-assert */
        if(MCSPI_CH_MODE_SINGLE == attrs->chMode)
        {
            if (chObj->csDisable == TRUE)
            {
                CSL_REG32_FINS(
                    baseAddr + MCSPI_CHCONF(chNum),
                    MCSPI_CH0CONF_FORCE,
                    CSL_MCSPI_CH0CONF_FORCE_DEASSERT);
                    chObj->csEnable = TRUE;
            }
        }
    }

    /* Disable channel */
    CSL_REG32_FINS(
        baseAddr + MCSPI_CHCTRL(chNum),
        MCSPI_CH0CTRL_EN,
        CSL_MCSPI_CH0CTRL_EN_NACT);

    if(MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode)
    {
        /* Disable DMA */
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_DISABLED);
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_DISABLED);
    }
    else if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg.trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_DISABLED);
    }
    else if (MCSPI_TR_MODE_RX_ONLY == chObj->chCfg.trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_DISABLED);
    }

    return status;
}

static void MCSPI_udmaStart(MCSPI_ChObject *chObj, const MCSPI_Attrs *attrs,
                           uint32_t baseAddr)
{

    uint32_t chNum = chObj->chCfg.chNum;

    /* Enable DMA */
    if (MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_ENABLED);
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_ENABLED);
    }
    else if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg.trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_ENABLED);
    }
    else if (MCSPI_TR_MODE_RX_ONLY == chObj->chCfg.trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_ENABLED);
    }

    /* Manual CS assert */
    if(MCSPI_CH_MODE_SINGLE == attrs->chMode)
    {
        if (chObj->csEnable == TRUE)
        {
            CSL_REG32_FINS(
                baseAddr + MCSPI_CHCONF(chNum),
                MCSPI_CH0CONF_FORCE,
                CSL_MCSPI_CH0CONF_FORCE_ASSERT);
            chObj->csEnable = FALSE;
        }
    }

    /* Enable channel */
    CSL_REG32_FINS(
        baseAddr + MCSPI_CHCTRL(chNum),
        MCSPI_CH0CTRL_EN,
        CSL_MCSPI_CH0CTRL_EN_ACT);

    /*
     * Note: Once the channel is enabled, DMA will trigger its transfer.
     */
}

static void MCSPI_udmaIsrTx(Udma_EventHandle eventHandle,
                                 uint32_t eventType,
                                 void *args)
{
    int32_t             retVal;
    uint64_t            pDesc;
    CSL_UdmapCppi5HMPD *pHpd;
    MCSPI_Config       *config;
    MCSPI_Object       *obj;
    MCSPI_ChObject     *chObj;
    MCSPI_Transaction  *transaction;
    uint32_t            chNum, effByteCnt, peerData;
    Udma_ChHandle       txChHandle;
    const MCSPI_Attrs  *attrs;

    /* Check parameters */
    if(NULL != args)
    {
        config = (MCSPI_Config *) args;
        obj = config->object;
        DebugP_assert(NULL != obj);
        DebugP_assert(NULL != config->attrs);
        attrs = config->attrs;

        transaction = obj->currTransaction;
        chNum = transaction->channel;
        chObj = &obj->chObj[chNum];
        txChHandle  = chObj->dmaChCfg.txChHandle;
        effByteCnt = transaction->count << chObj->bufWidthShift;

        if (eventType == UDMA_EVENT_TYPE_DMA_COMPLETION)
        {
            CacheP_inv(chObj->dmaChCfg.txHpdMem, chObj->dmaChCfg.hpdMemSize, CacheP_TYPE_ALLD);
            retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(txChHandle), &pDesc);
            pHpd = (CSL_UdmapCppi5HMPD *)(uintptr_t)pDesc;
            if ((retVal == UDMA_SOK) && (pHpd != NULL))
            {
                transaction->status = MCSPI_TRANSFER_COMPLETED;
            }
            else
            {
                transaction->status = MCSPI_TRANSFER_FAILED;
            }

            /* In case of TX only mode, stop channel and close it */
            if ((transaction->status == MCSPI_TRANSFER_COMPLETED) &&
                (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg.trMode))
            {
                retVal = Udma_getPeerData(txChHandle, &peerData);
                DebugP_assert(retVal == UDMA_SOK);
                while (effByteCnt != peerData)
                {
                    Udma_getPeerData(txChHandle, &peerData);
                };
                /* Clear data */
                Udma_clearPeerData(txChHandle, peerData);

                /* Get Byte count transmitted */
                effByteCnt = (pHpd->descInfo & CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_MASK) >> CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_SHIFT;
                obj->currTransaction->count = (effByteCnt >> chObj->bufWidthShift);

                /* Stop MCSPI Channel */
                MCSPI_udmaStop(obj, attrs, chObj, chNum);
                /* Update the driver internal status. */
                obj->currTransaction = NULL;
                /*
                * Post transfer Sem in case of blocking transfer.
                * Call the callback function in case of Callback mode.
                */
                if (obj->openPrms.transferMode == MCSPI_TRANSFER_MODE_BLOCKING)
                {
                    SemaphoreP_post(&obj->transferSemObj);
                }
                else
                {
                    obj->openPrms.transferCallbackFxn((MCSPI_Handle) config, transaction);
                }
            }
        }
        else
        {
            transaction->status = MCSPI_TRANSFER_FAILED;
        }
    }

    return;
}

static void MCSPI_udmaIsrRx(Udma_EventHandle eventHandle,
                                 uint32_t eventType,
                                 void *args)
{
    int32_t             retVal;
    uint64_t            pDesc;
    CSL_UdmapCppi5HMPD *pHpd;
    MCSPI_Config       *config;
    MCSPI_Object       *obj;
    MCSPI_ChObject     *chObj;
    MCSPI_Transaction  *transaction;
    uint32_t            chNum, effByteCnt, peerData;
    Udma_ChHandle       rxChHandle;
    const MCSPI_Attrs  *attrs;

    /* Check parameters */
    if(NULL != args)
    {
        config = (MCSPI_Config *) args;
        obj = config->object;
        DebugP_assert(NULL != obj);
        DebugP_assert(NULL != config->attrs);
        attrs = config->attrs;
        transaction = obj->currTransaction;
        chNum = transaction->channel;
        chObj = &obj->chObj[chNum];
        rxChHandle  = chObj->dmaChCfg.rxChHandle;
        effByteCnt = transaction->count << chObj->bufWidthShift;

        if (eventType == UDMA_EVENT_TYPE_DMA_COMPLETION)
        {
            CacheP_inv(chObj->dmaChCfg.rxHpdMem, chObj->dmaChCfg.hpdMemSize, CacheP_TYPE_ALLD);
            retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(rxChHandle), &pDesc);
            pHpd = (CSL_UdmapCppi5HMPD *)(uintptr_t)pDesc;
            if (retVal == UDMA_SOK)
            {
                transaction->status = MCSPI_TRANSFER_COMPLETED;
            }
            else
            {
                transaction->status = MCSPI_TRANSFER_FAILED;
            }

            if (MCSPI_TR_MODE_TX_ONLY != chObj->chCfg.trMode)
            {
                retVal = Udma_getPeerData(rxChHandle, &peerData);
                DebugP_assert(retVal == UDMA_SOK);
                while (effByteCnt != peerData)
                {
                    Udma_getPeerData(rxChHandle, &peerData);
                };
                /* Clear Data */
                Udma_clearPeerData(rxChHandle, peerData);

                /* Get Byte count received */
                effByteCnt = (pHpd->descInfo & CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_MASK) >> CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_SHIFT;
                obj->currTransaction->count = (effByteCnt >> chObj->bufWidthShift);

                /* Stop MCSPI Channel */
                MCSPI_udmaStop(obj, attrs, chObj, chNum);
                /* Update the driver internal status. */
                obj->currTransaction = NULL;
                /*
                * Post transfer Sem in case of blocking transfer.
                * Call the callback function in case of Callback mode.
                */
                if (obj->openPrms.transferMode == MCSPI_TRANSFER_MODE_BLOCKING)
                {
                    SemaphoreP_post(&obj->transferSemObj);
                }
                else
                {
                    obj->openPrms.transferCallbackFxn((MCSPI_Handle) config, transaction);
                }
            }
        }
        else
        {
            transaction->status = MCSPI_TRANSFER_FAILED;
        }
    }
    return;
}

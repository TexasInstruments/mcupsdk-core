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
#include <drivers/mcspi/v0/lld/mcspi_lld.h>
#include <drivers/mcspi/v0/lld/dma/udma/mcspi_dma_udma.h>
#include <drivers/mcspi/v0/lld/dma/mcspi_dma.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/udma.h>
#include <kernel/dpl/ClockP.h>

/* Static Function Declarations */
static void MCSPI_udmaHpdInit(uint8_t *pHpdMem,
                              const void *destBuf,
                              uint32_t length);
static void MCSPI_udmaIsrTx(Udma_EventHandle eventHandle,
                            uint32_t eventType,
                            void *args);
static void MCSPI_udmaIsrRx(Udma_EventHandle eventHandle,
                            uint32_t eventType,
                            void *args);
static int32_t MCSPI_udmaInitRxCh(MCSPILLD_Handle hMcspi, const MCSPI_ChObject *chObj);
static int32_t MCSPI_udmaInitTxCh(MCSPILLD_Handle hMcspi, const MCSPI_ChObject *chObj);
static int32_t MCSPI_udmaDeInitCh(Udma_ChHandle chHandle,
                                  Udma_EventHandle eventHandle,
                                  uint32_t  isChEnabled);
static int32_t MCSPI_udmaConfigPdmaRx(const MCSPI_ChObject *chObj,
                                      uint32_t numWords,
                                      const uint8_t *rxBufPtr);
static int32_t MCSPI_udmaConfigPdmaTx(const MCSPI_ChObject *chObj,
                                      uint32_t numWords,
                                      const uint8_t *txBufPtr);
static void MCSPI_udmaStart(MCSPILLD_Handle hMcspi, MCSPI_ChObject *chObj,
                            uint32_t baseAddr);

int32_t MCSPI_lld_dmaInit(MCSPI_DmaHandle mcspiDmaHandle)
{
    int32_t    status = MCSPI_STATUS_SUCCESS;

    if(mcspiDmaHandle == NULL)
    {
        status = MCSPI_STATUS_FAILURE;
    }

    return status;
}

int32_t MCSPI_lld_dmaChInit(MCSPILLD_Handle hMcspi, uint32_t chCnt)
{
    int32_t              status = MCSPI_STATUS_SUCCESS;
    MCSPI_ChObject      *chObj;
    MCSPI_UdmaChConfig  *dmaChConfig;

    chObj           = &hMcspi->hMcspiInit->chObj[chCnt];
    dmaChConfig     = (MCSPI_UdmaChConfig *)chObj->dmaChCfg;
    dmaChConfig     = &(dmaChConfig[chObj->dmaChConfigNum]);
    chObj->dmaChCfg = (MCSPI_DmaChConfig)dmaChConfig;

    if(MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode)
    {
    /*
     * Note: In TX RX mode, we expect TX Interrupt to be triggered first
     * and RX Interrupt next. To keep this flow, TX channel(shared event)
     * should be initialized first and then the RX channel(shared event).
     */
        status  = MCSPI_udmaInitTxCh(hMcspi, chObj);
        status += MCSPI_udmaInitRxCh(hMcspi, chObj);
    }
    else if(MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
    {
        status = MCSPI_udmaInitTxCh(hMcspi, chObj);
    }
    else
    {
        status = MCSPI_udmaInitRxCh(hMcspi, chObj);
    }

    if(status == MCSPI_STATUS_SUCCESS)
    {
        dmaChConfig->isOpen = TRUE;
    }

    return status;
}

int32_t MCSPI_lld_dmaDeInit(MCSPILLD_Handle hMcspi, const MCSPI_ChConfig *chCfg, uint32_t chCnt)
{
    int32_t              status = MCSPI_STATUS_SUCCESS;
    MCSPI_ChObject      *chObj;
    MCSPI_UdmaChConfig  *dmaChConfig;

    /* Check parameters */
    if((NULL == hMcspi) ||
       (NULL == chCfg) ||
       (chCfg->chNum >= MCSPI_MAX_NUM_CHANNELS))
    {
        status = MCSPI_STATUS_FAILURE;
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        chObj            = &hMcspi->hMcspiInit->chObj[chCnt];
        dmaChConfig      = (MCSPI_UdmaChConfig *)chObj->dmaChCfg;

        if (dmaChConfig->isOpen != FALSE)
        {
            if(MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode)
            {
                status += MCSPI_udmaDeInitCh(dmaChConfig->rxChHandle,
                                   dmaChConfig->cqRxEvtHandle,
                                   dmaChConfig->isChEnabled);
                status += MCSPI_udmaDeInitCh(dmaChConfig->txChHandle,
                                   dmaChConfig->cqTxEvtHandle,
                                   dmaChConfig->isChEnabled);
            }
            else if(MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
            {
                status += MCSPI_udmaDeInitCh(dmaChConfig->txChHandle,
                                   dmaChConfig->cqTxEvtHandle,
                                   dmaChConfig->isChEnabled);
            }
            else
            {
                status += MCSPI_udmaDeInitCh(dmaChConfig->rxChHandle,
                                   dmaChConfig->cqRxEvtHandle,
                                   dmaChConfig->isChEnabled);
            }
            dmaChConfig->isOpen = FALSE;
        }
    }

    return status;
}

int32_t MCSPI_lld_dmaTransfer(MCSPILLD_Handle hMcspi,
                              MCSPI_ChObject *chObj,
                              const MCSPI_Transaction *transaction)
{
    int32_t              status = MCSPI_STATUS_SUCCESS;
    MCSPI_UdmaChConfig  *dmaChConfig = (MCSPI_UdmaChConfig *)chObj->dmaChCfg;

    if(MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode)
    {
        if ((chObj->curRxBufPtr != NULL) && (chObj->curTxBufPtr != NULL))
        {
            status  = MCSPI_udmaConfigPdmaRx(chObj, transaction->count, chObj->curRxBufPtr);
            status += MCSPI_udmaConfigPdmaTx(chObj, transaction->count, chObj->curTxBufPtr);
        }
        else
        {
            status = MCSPI_STATUS_FAILURE;
        }
    }
    else if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
    {
        if (chObj->curTxBufPtr != NULL)
        {
            status  = MCSPI_udmaConfigPdmaTx(chObj, transaction->count, chObj->curTxBufPtr);
        }
        else
        {
            status = MCSPI_STATUS_FAILURE;
        }
    }
    else
    {
        if (chObj->curRxBufPtr != NULL)
        {
            status  = MCSPI_udmaConfigPdmaRx(chObj, transaction->count, chObj->curRxBufPtr);
        }
        else
        {
            status = MCSPI_STATUS_FAILURE;
        }
    }
    /* Enable the UDMA channel flag */
    dmaChConfig->isChEnabled = MCSPI_UDMA_CHANNEL_ENABLE;

    /* Initiate Transfer */
    MCSPI_udmaStart(hMcspi, chObj, hMcspi->baseAddr);

    return (status);
}

static void MCSPI_udmaHpdInit(uint8_t       *pHpdMem,
                              const void    *destBuf,
                              uint32_t      length)
{
    CSL_UdmapCppi5HMPD *pHpd = (CSL_UdmapCppi5HMPD *) pHpdMem;
    uint32_t descType = (uint32_t)CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST;

    /* Setup descriptor */
    CSL_udmapCppi5SetDescType(pHpd, descType);
    CSL_udmapCppi5SetEpiDataPresent(pHpd, (bool)FALSE);
    CSL_udmapCppi5SetPsDataLoc(pHpd, 0U);
    CSL_udmapCppi5SetPsDataLen(pHpd, 0U);
    CSL_udmapCppi5SetPktLen(pHpd, descType, length);
    CSL_udmapCppi5SetPsFlags(pHpd, 0U);
    CSL_udmapCppi5SetIds(pHpd, descType, 0x321, UDMA_DEFAULT_FLOW_ID);
    CSL_udmapCppi5SetSrcTag(pHpd, 0x0000);     /* Not used */
    CSL_udmapCppi5SetDstTag(pHpd, 0x0000);     /* Not used */
    /* Return Policy descriptors are reserved in case of AM243X/Am64X */
    CSL_udmapCppi5SetReturnPolicy(pHpd, descType, 0U, 0U, 0U, 0U);
    CSL_udmapCppi5LinkDesc(pHpd, 0U);
    CSL_udmapCppi5SetBufferAddr(pHpd, (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL));
    CSL_udmapCppi5SetBufferLen(pHpd, length);
    CSL_udmapCppi5SetOrgBufferAddr(pHpd, (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL));
    CSL_udmapCppi5SetOrgBufferLen(pHpd, length);

    /* Writeback cache */
    CacheP_wb(pHpdMem, sizeof(CSL_UdmapCppi5HMPD), CacheP_TYPE_ALLD);

    return;
}

static int32_t MCSPI_udmaInitRxCh(MCSPILLD_Handle hMcspi, const MCSPI_ChObject *chObj)
{
    int32_t             retVal, status = MCSPI_STATUS_SUCCESS;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;

    Udma_DrvHandle      mcspiUdmaHandle;
    Udma_ChHandle       rxChHandle;
    MCSPI_UdmaChConfig  *dmaChConfig;

    dmaChConfig = (MCSPI_UdmaChConfig *)chObj->dmaChCfg;
    mcspiUdmaHandle  = (Udma_DrvHandle) (hMcspi->hMcspiInit->mcspiDmaHandle);

    /* Init RX channel parameters */
    chType = UDMA_CH_TYPE_PDMA_RX;
    UdmaChPrms_init(&chPrms, chType);
    chPrms.peerChNum                = dmaChConfig->rxEvtNum;
    chPrms.fqRingPrms.ringMem       = dmaChConfig->rxRingMem;
    chPrms.fqRingPrms.ringMemSize   = dmaChConfig->ringMemSize;
    chPrms.fqRingPrms.elemCnt       = dmaChConfig->ringElemCnt;
    rxChHandle                      = dmaChConfig->rxChHandle;

    /* Open channel for block copy */
    retVal = Udma_chOpen(mcspiUdmaHandle, rxChHandle, chType, &chPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config RX channel */
    UdmaChRxPrms_init(&rxPrms, chType);
    retVal = Udma_chConfigRx(rxChHandle, &rxPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Register ring completion callback */
    eventHandle = dmaChConfig->cqRxEvtHandle;
    UdmaEventPrms_init(&eventPrms);
    eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    eventPrms.chHandle          = rxChHandle;
    eventPrms.controllerEventHandle = Udma_eventGetGlobalHandle(mcspiUdmaHandle);
    eventPrms.eventCb           = &MCSPI_udmaIsrRx;
    eventPrms.appData           = (void *) hMcspi;
    retVal = Udma_eventRegister(mcspiUdmaHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);

    if(retVal == UDMA_SOK)
    {
        status = MCSPI_STATUS_SUCCESS;
    }
    else
    {
        status = MCSPI_STATUS_FAILURE;
    }

    return status;
}

static int32_t MCSPI_udmaInitTxCh(MCSPILLD_Handle hMcspi, const MCSPI_ChObject *chObj)
{
    int32_t             retVal, status = MCSPI_STATUS_SUCCESS;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChTxPrms       txPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_DrvHandle      mcspiUdmaHandle;
    Udma_ChHandle       txChHandle;
    MCSPI_UdmaChConfig  *dmaChConfig;

    dmaChConfig = (MCSPI_UdmaChConfig *)chObj->dmaChCfg;
    mcspiUdmaHandle  = (Udma_DrvHandle) (hMcspi->hMcspiInit->mcspiDmaHandle);

    /* Init TX channel parameters */
    chType = UDMA_CH_TYPE_PDMA_TX;
    UdmaChPrms_init(&chPrms, chType);
    chPrms.peerChNum                = dmaChConfig->txEvtNum;
    chPrms.fqRingPrms.ringMem       = dmaChConfig->txRingMem;
    chPrms.fqRingPrms.ringMemSize   = dmaChConfig->ringMemSize;
    chPrms.fqRingPrms.elemCnt       = dmaChConfig->ringElemCnt;
    txChHandle                      = dmaChConfig->txChHandle;

    /* Open channel for block copy */
    retVal = Udma_chOpen(mcspiUdmaHandle, txChHandle, chType, &chPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config TX channel */
    UdmaChTxPrms_init(&txPrms, chType);
    retVal = Udma_chConfigTx(txChHandle, &txPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Register ring completion callback */
    eventHandle = dmaChConfig->cqTxEvtHandle;
    UdmaEventPrms_init(&eventPrms);
    eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    eventPrms.chHandle          = txChHandle;
    eventPrms.controllerEventHandle = Udma_eventGetGlobalHandle(mcspiUdmaHandle);
    eventPrms.eventCb           = &MCSPI_udmaIsrTx;
    eventPrms.appData           = (void *) hMcspi;
    retVal = Udma_eventRegister(mcspiUdmaHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);

    if(retVal == UDMA_SOK)
    {
        status = MCSPI_STATUS_SUCCESS;
    }
    else
    {
        status = MCSPI_STATUS_FAILURE;
    }

    return status;
}

static int32_t MCSPI_udmaDeInitCh(Udma_ChHandle chHandle,
                                  Udma_EventHandle eventHandle,
                                  uint32_t  isChEnabled)
{
    int32_t udmaStatus = UDMA_SOK;
    int32_t status = MCSPI_STATUS_SUCCESS;

    if(isChEnabled == MCSPI_UDMA_CHANNEL_ENABLE)
    {
        /* Disable Channel */
        udmaStatus = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
        DebugP_assert(UDMA_SOK == udmaStatus);
    }

    /* UnRegister Event */
    udmaStatus = Udma_eventUnRegister(eventHandle);
    DebugP_assert(UDMA_SOK == udmaStatus);

    if(isChEnabled == MCSPI_UDMA_CHANNEL_ENABLE)
    {
        /* Flush any pending request from the free queue */
        while((bool)TRUE)
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
    }

    /* Close channel */
    udmaStatus = Udma_chClose(chHandle);
    DebugP_assert(UDMA_SOK == udmaStatus);

    if(udmaStatus == UDMA_SOK)
    {
        status = MCSPI_STATUS_SUCCESS;
    }
    else
    {
        status = MCSPI_STATUS_FAILURE;
    }

    return status;
}

static int32_t MCSPI_udmaConfigPdmaTx(const MCSPI_ChObject *chObj,
                                      uint32_t numWords,
                                      const uint8_t *txBufPtr)
{
    int32_t             retVal, status = MCSPI_STATUS_SUCCESS;
    Udma_ChPdmaPrms     pdmaPrms;
    Udma_ChHandle       txChHandle;
    MCSPI_UdmaChConfig  *dmaChConfig;

    dmaChConfig = (MCSPI_UdmaChConfig *)chObj->dmaChCfg;
    txChHandle  = dmaChConfig->txChHandle;

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
    MCSPI_udmaHpdInit((uint8_t *) dmaChConfig->txHpdMem, txBufPtr, (numWords << chObj->bufWidthShift));

    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(txChHandle),
                 (uint64_t) Udma_defaultVirtToPhyFxn(dmaChConfig->txHpdMem, 0U, NULL));
    DebugP_assert(UDMA_SOK == retVal);

    if(retVal == UDMA_SOK)
    {
        status = MCSPI_STATUS_SUCCESS;
    }
    else
    {
        status = MCSPI_STATUS_FAILURE;
    }

    return (status);
}

static int32_t MCSPI_udmaConfigPdmaRx(const MCSPI_ChObject *chObj,
                                      uint32_t numWords,
                                      const uint8_t *rxBufPtr)
{
    int32_t             retVal, status = MCSPI_STATUS_SUCCESS;
    Udma_ChPdmaPrms     pdmaPrms;
    Udma_ChHandle       rxChHandle;
    MCSPI_UdmaChConfig  *dmaChConfig;

    dmaChConfig = (MCSPI_UdmaChConfig *)chObj->dmaChCfg;
    rxChHandle  = dmaChConfig->rxChHandle;

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
    MCSPI_udmaHpdInit((uint8_t *) dmaChConfig->rxHpdMem, rxBufPtr, (numWords << chObj->bufWidthShift));

    /* Submit HPD to channel */
    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(rxChHandle),
                 (uint64_t) Udma_defaultVirtToPhyFxn(dmaChConfig->rxHpdMem, 0U, NULL));
    DebugP_assert(UDMA_SOK == retVal);

    if(retVal == UDMA_SOK)
    {
        status = MCSPI_STATUS_SUCCESS;
    }
    else
    {
        status = MCSPI_STATUS_FAILURE;
    }


    return (status);
}

void MCSPI_lld_dmaStop(MCSPILLD_Handle hMcspi,
                              MCSPI_ChObject *chObj, uint32_t chNum)
{
    uint32_t baseAddr;
    MCSPILLD_InitHandle hMcspiInit = hMcspi->hMcspiInit;

    baseAddr = hMcspi->baseAddr;
    if(MCSPI_MS_MODE_CONTROLLER == hMcspiInit->msMode)
    {
        /* Manual CS de-assert */
        if(MCSPI_CH_MODE_SINGLE == hMcspiInit->chMode)
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

    if(MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode)
    {
        /* Disable DMA */
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_DISABLED);
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_DISABLED);
    }
    else if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_DISABLED);
    }
    else
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_DISABLED);
    }

    hMcspi->state = MCSPI_STATE_READY;

    return;
}

static void MCSPI_udmaStart(MCSPILLD_Handle hMcspi, MCSPI_ChObject *chObj,
                           uint32_t baseAddr)
{

    uint32_t chNum = chObj->chCfg->chNum;

    /* Enable DMA */
    if (MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_ENABLED);
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_ENABLED);
    }
    else if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_ENABLED);
    }
    else
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_ENABLED);
    }

    /* Manual CS assert */
    if(MCSPI_CH_MODE_SINGLE == hMcspi->hMcspiInit->chMode)
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
    MCSPI_ChObject     *chObj;
    MCSPI_Transaction  *transaction;
    uint32_t            chNum, effByteCnt, peerData;
    Udma_ChHandle       txChHandle;
    MCSPI_UdmaChConfig *dmaChConfig;
    MCSPILLD_Handle     hMcspi;
    uint32_t            baseAddr, irqStatus = 0U;
    volatile uint32_t   chStat;
    uint32_t            startTicks, elapsedTicks = 0;
    int32_t             status = MCSPI_STATUS_SUCCESS;
    MCSPILLD_InitHandle hMcspiInit;

    /* Check parameters */
    if((NULL != args) && (eventHandle != NULL))
    {
        hMcspi       = (MCSPILLD_Handle) args;
        hMcspiInit   = hMcspi->hMcspiInit;
        transaction  = &hMcspi->transaction;
        chNum        = transaction->channel;
        chObj        = &hMcspi->hMcspiInit->chObj[chNum];
        dmaChConfig  = (MCSPI_UdmaChConfig *)chObj->dmaChCfg;
        txChHandle   = dmaChConfig->txChHandle;
        effByteCnt   = transaction->count << chObj->bufWidthShift;
        baseAddr     = hMcspi->baseAddr;

        if (eventType == UDMA_EVENT_TYPE_DMA_COMPLETION)
        {
            CacheP_inv(dmaChConfig->txHpdMem, dmaChConfig->hpdMemSize, CacheP_TYPE_ALLD);
            retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(txChHandle), &pDesc);
            pHpd = (CSL_UdmapCppi5HMPD *)(uintptr_t)pDesc;
            if ((retVal == UDMA_SOK) && (pHpd != NULL))
            {
                status = MCSPI_TRANSFER_COMPLETED;
            }
            else
            {
                status = MCSPI_TRANSFER_FAILED;
                hMcspi->hMcspiInit->errorCallbackFxn(hMcspi, status);
            }

            /* In case of TX only mode, stop channel and close it */
            if ((status == MCSPI_TRANSFER_COMPLETED) &&
                (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode))
            {
                retVal = Udma_getPeerData(txChHandle, &peerData);
                DebugP_assert(retVal == UDMA_SOK);
                startTicks = hMcspiInit->clockP_get();
                while ((effByteCnt != peerData) && (elapsedTicks < transaction->timeout))
                {
                    retVal += Udma_getPeerData(txChHandle, &peerData);
                    elapsedTicks = hMcspiInit->clockP_get() - startTicks;
                };
                /* Clear data */
                retVal += Udma_clearPeerData(txChHandle, peerData);
                DebugP_assert(retVal == UDMA_SOK);

                /* Get Byte count transmitted */
                effByteCnt = (pHpd->descInfo & CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_MASK) >> CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_SHIFT;
                hMcspi->transaction.count = (effByteCnt >> chObj->bufWidthShift);

                do{
                    /* Wait for end of transfer. */
                    chStat = CSL_REG32_RD(baseAddr + MCSPI_CHSTAT(chNum));
                    elapsedTicks = hMcspiInit->clockP_get() - startTicks;
                }while (((chStat & CSL_MCSPI_CH0STAT_EOT_MASK) == 0U) && (elapsedTicks < transaction->timeout));

                /* Stop MCSPI Channel */
                MCSPI_lld_dmaStop(hMcspi, chObj, chNum);
                hMcspi->state = MCSPI_STATE_READY;

                irqStatus = CSL_REG32_RD(baseAddr + CSL_MCSPI_IRQSTATUS);
                if (((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_MASK << (4U * chNum))) != 0U) &&
                    (hMcspiInit->msMode == MCSPI_MS_MODE_PERIPHERAL))
                {
                    retVal  = MCSPI_TRANSFER_CANCELLED;
                    hMcspi->errorFlag |= MCSPI_ERROR_TX_UNDERFLOW;
                }

                if(hMcspi->errorFlag != 0U)
                {
                    hMcspi->state = MCSPI_STATE_READY;
                    hMcspi->hMcspiInit->errorCallbackFxn(hMcspi, retVal);
                }
                else
                {
                    hMcspi->hMcspiInit->transferCallbackFxn(hMcspi, MCSPI_TRANSFER_COMPLETED);
                }
            }
        }
    }

    return;
}

static void MCSPI_udmaIsrRx(Udma_EventHandle eventHandle,
                                 uint32_t eventType,
                                 void *args)
{
    int32_t                retVal;
    uint64_t               pDesc;
    CSL_UdmapCppi5HMPD    *pHpd;
    MCSPI_ChObject        *chObj;
    MCSPI_Transaction     *transaction;
    uint32_t               chNum, effByteCnt, peerData;
    Udma_ChHandle          rxChHandle;
    MCSPI_UdmaChConfig    *dmaChConfig;
    MCSPILLD_Handle        hMcspi;
    uint32_t               startTicks, elapsedTicks = 0;
    int32_t                status = MCSPI_STATUS_SUCCESS;
    MCSPILLD_InitHandle    hMcspiInit;
    uint32_t               baseAddr, irqStatus = 0U;

    /* Check parameters */
    if ((NULL != args) && (eventHandle != NULL))
    {
        hMcspi       = (MCSPILLD_Handle) args;
        hMcspiInit   = hMcspi->hMcspiInit;
        baseAddr     = hMcspi->baseAddr;
        transaction  = &hMcspi->transaction;
        chNum        = transaction->channel;
        chObj        = &hMcspi->hMcspiInit->chObj[chNum];
        dmaChConfig  = (MCSPI_UdmaChConfig *)chObj->dmaChCfg;
        rxChHandle   = dmaChConfig->rxChHandle;
        effByteCnt   = transaction->count << chObj->bufWidthShift;

        if (eventType == UDMA_EVENT_TYPE_DMA_COMPLETION)
        {
            CacheP_inv(dmaChConfig->rxHpdMem, dmaChConfig->hpdMemSize, CacheP_TYPE_ALLD);
            retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(rxChHandle), &pDesc);
            pHpd   = (CSL_UdmapCppi5HMPD *)(uintptr_t)pDesc;
            if (retVal == UDMA_SOK)
            {
                status = MCSPI_TRANSFER_COMPLETED;
            }
            else
            {
                status = MCSPI_TRANSFER_FAILED;
                hMcspi->hMcspiInit->errorCallbackFxn(hMcspi, status);
            }

            if ((status == MCSPI_TRANSFER_COMPLETED) &&
                (MCSPI_TR_MODE_TX_ONLY != chObj->chCfg->trMode))
            {
                retVal = Udma_getPeerData(rxChHandle, &peerData);
                DebugP_assert(retVal == UDMA_SOK);
                startTicks = hMcspiInit->clockP_get();
                while ((effByteCnt != peerData) && (elapsedTicks < transaction->timeout))
                {
                    retVal += Udma_getPeerData(rxChHandle, &peerData);
                    elapsedTicks = hMcspiInit->clockP_get() - startTicks;
                };
                /* Clear Data */
                retVal += Udma_clearPeerData(rxChHandle, peerData);
                DebugP_assert(retVal == UDMA_SOK);

                /* Stop MCSPI Channel */
                MCSPI_lld_dmaStop(hMcspi, chObj, chNum);
                hMcspi->state = MCSPI_STATE_READY;

                /* Get Byte count received */
                effByteCnt = (pHpd->descInfo & CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_MASK) >> CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_SHIFT;
                hMcspi->transaction.count = (effByteCnt >> chObj->bufWidthShift);

                irqStatus = CSL_REG32_RD(baseAddr + CSL_MCSPI_IRQSTATUS);
                if ((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_MASK)) != 0U)
                {
                    retVal = MCSPI_TRANSFER_CANCELLED;
                    hMcspi->errorFlag |= MCSPI_ERROR_RX_OVERFLOW;
                }

                if (((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_MASK << (4U * chNum))) != 0U) &&
                    ((hMcspiInit->msMode == MCSPI_MS_MODE_PERIPHERAL)))
                {
                    retVal = MCSPI_TRANSFER_CANCELLED;
                    hMcspi->errorFlag |= MCSPI_ERROR_TX_UNDERFLOW;
                }

                if(hMcspi->errorFlag != 0U)
                {
                    hMcspi->hMcspiInit->errorCallbackFxn(hMcspi, retVal);
                }
                else
                {
                    hMcspi->hMcspiInit->transferCallbackFxn(hMcspi, MCSPI_TRANSFER_COMPLETED);
                }
            }
        }
    }
    return;
}

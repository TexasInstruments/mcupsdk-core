/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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
#include <drivers/uart/v0/lld/dma/uart_dma.h>
#include <drivers/uart/v0/lld/dma/udma/uart_dma_udma.h>
#include <drivers/uart/v0/lld/uart_lld.h>
#include <drivers/uart/v0/lld/dma/soc/uart_dma_soc.h>
#include <kernel/dpl/CacheP.h>

/* Static Function Declarations */
static int32_t UART_udmaInitTxCh(UARTLLD_Handle hUart, const UART_UdmaChConfig *udmaArgs);
static int32_t UART_udmaInitRxCh(UARTLLD_Handle hUart, const UART_UdmaChConfig *udmaArgs);
static void UART_udmaHpdInit(Udma_ChHandle chHandle, uint8_t       *pHpdMem,
                              const void    *destBuf,
                              uint32_t      length);
static int32_t UART_udmaConfigPdmaTx(UARTLLD_Handle hUart, const UART_Transaction *transaction);
static int32_t UART_udmaConfigPdmaRx(UARTLLD_Handle hUart, const UART_Transaction *transaction);
static int32_t UART_udmaDeInitCh(Udma_ChHandle chHandle, Udma_EventHandle eventHandle);
static void UART_udmaIsrRx(Udma_EventHandle eventHandle, uint32_t eventType, void *args);
static void UART_udmaIsrTx(Udma_EventHandle eventHandle, uint32_t eventType, void *args);

int32_t UART_lld_dmaInit(UARTLLD_Handle hUart, UART_DmaChConfig dmaChCfg)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    UART_UdmaChConfig *udmaChCfg = (UART_UdmaChConfig *)dmaChCfg;

    status  = UART_udmaInitRxCh(hUart, udmaChCfg);
    status += UART_udmaInitTxCh(hUart, udmaChCfg);

    if (status == UDMA_SOK)
    {
        udmaChCfg->isOpen = TRUE;
        status = UART_TRANSFER_STATUS_SUCCESS;
    }
    else
    {
        status = UART_TRANSFER_STATUS_FAILURE;
    }

    return status;
}

static int32_t UART_udmaInitRxCh(UARTLLD_Handle hUart, const UART_UdmaChConfig *udmaChCfg)
{
    int32_t             retVal;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_DrvHandle         drvHandle;
    Udma_ChHandle       rxChHandle;

    /* Init RX channel parameters */
    chType = UDMA_CH_TYPE_PDMA_RX;
    UdmaChPrms_init(&chPrms, chType);
    chPrms.peerChNum                = hUart->hUartInit->rxEvtNum;
    chPrms.fqRingPrms.ringMem       = udmaChCfg->rxRingMem;
    chPrms.fqRingPrms.ringMemSize   = udmaChCfg->ringMemSize;
    chPrms.fqRingPrms.elemCnt       = udmaChCfg->ringElemCnt;
    if(udmaChCfg->isCqRingMem == UDMA_COMP_QUEUE_RING_MEM_ENABLED){
        chPrms.cqRingPrms.ringMem       = udmaChCfg->cqRxRingMem;
        chPrms.cqRingPrms.ringMemSize   = udmaChCfg->ringMemSize;
        chPrms.cqRingPrms.elemCnt       = udmaChCfg->ringElemCnt;
    }
    rxChHandle                      = udmaChCfg->rxChHandle;
    drvHandle                       = udmaChCfg->drvHandle;

    /* Open channel for block copy */
    retVal = Udma_chOpen(drvHandle, rxChHandle, chType, &chPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config RX channel */
    UdmaChRxPrms_init(&rxPrms, chType);
    retVal = Udma_chConfigRx(rxChHandle, &rxPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Register ring completion callback */
    eventHandle = udmaChCfg->cqRxEvtHandle;
    UdmaEventPrms_init(&eventPrms);
    eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    eventPrms.chHandle          = rxChHandle;
    eventPrms.controllerEventHandle = Udma_eventGetGlobalHandle(drvHandle);
    eventPrms.eventCb           = &UART_udmaIsrRx;
    eventPrms.appData           = (void *) hUart;
    retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);

    return retVal;
}

static int32_t UART_udmaInitTxCh(UARTLLD_Handle hUart, const UART_UdmaChConfig *udmaChCfg)
{
    int32_t             retVal;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChTxPrms       txPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_DrvHandle      drvHandle;
    Udma_ChHandle       txChHandle;

    /* Init TX channel parameters */
    chType = UDMA_CH_TYPE_PDMA_TX;
    UdmaChPrms_init(&chPrms, chType);
    chPrms.peerChNum                = hUart->hUartInit->txEvtNum;
    chPrms.fqRingPrms.ringMem       = udmaChCfg->txRingMem;
    chPrms.fqRingPrms.ringMemSize   = udmaChCfg->ringMemSize;
    chPrms.fqRingPrms.elemCnt       = udmaChCfg->ringElemCnt;
    if(udmaChCfg->isCqRingMem == UDMA_COMP_QUEUE_RING_MEM_ENABLED){
        chPrms.cqRingPrms.ringMem       = udmaChCfg->cqTxRingMem;
        chPrms.cqRingPrms.ringMemSize   = udmaChCfg->ringMemSize;
        chPrms.cqRingPrms.elemCnt       = udmaChCfg->ringElemCnt;
    }
    txChHandle                      = udmaChCfg->txChHandle;
    drvHandle                       = udmaChCfg->drvHandle;

    /* Open channel for block copy */
    retVal = Udma_chOpen(drvHandle, txChHandle, chType, &chPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config TX channel */
    UdmaChTxPrms_init(&txPrms, chType);
    retVal = Udma_chConfigTx(txChHandle, &txPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Register ring completion callback */
    eventHandle = udmaChCfg->cqTxEvtHandle;
    UdmaEventPrms_init(&eventPrms);
    eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    eventPrms.chHandle          = txChHandle;
    eventPrms.controllerEventHandle = Udma_eventGetGlobalHandle(drvHandle);
    eventPrms.eventCb           = &UART_udmaIsrTx;
    eventPrms.appData           = (void *) hUart;
    retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);

    return retVal;
}

int32_t UART_lld_dmaWrite(UARTLLD_Handle hUart, const UART_Transaction *transaction)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;

    status = UART_udmaConfigPdmaTx(hUart, transaction);
    if (status != UDMA_SOK)
    {
        status = UART_TRANSFER_STATUS_FAILURE;
    }

    return status;
}

int32_t UART_lld_dmaRead(UARTLLD_Handle hUart, const UART_Transaction *transaction)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;

    status = UART_udmaConfigPdmaRx(hUart, transaction);
    if (status != UDMA_SOK)
    {
        status = UART_TRANSFER_STATUS_FAILURE;
    }

    return status;
}

int32_t UART_lld_dmaDeInit(UARTLLD_Handle hUart)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    UART_UdmaChConfig *udmaChCfg;

    udmaChCfg    = (UART_UdmaChConfig *)hUart->hUartInit->dmaChCfg;

    if (udmaChCfg->isOpen != FALSE)
    {
        (void)UART_udmaDeInitCh(udmaChCfg->rxChHandle,
                          udmaChCfg->cqRxEvtHandle);
        (void)UART_udmaDeInitCh(udmaChCfg->txChHandle,
                          udmaChCfg->cqTxEvtHandle);
        udmaChCfg->isOpen = FALSE;
    }

    return status;
}

static int32_t UART_udmaConfigPdmaTx(UARTLLD_Handle hUart, const UART_Transaction *transaction)
{
    int32_t             retVal;
    Udma_ChPdmaPrms     pdmaPrms;
    Udma_ChHandle       txChHandle;
    UART_UdmaChConfig   *udmaChCfg;

    udmaChCfg    = (UART_UdmaChConfig *)hUart->hUartInit->dmaChCfg;
    txChHandle  = udmaChCfg->txChHandle;

    /* Config PDMA channel */
    UdmaChPdmaPrms_init(&pdmaPrms);
    pdmaPrms.elemSize = UDMA_PDMA_ES_8BITS;
    /* Number of words received in each transfer */
    pdmaPrms.elemCnt  = 1U;
    /* Dont care for Tx */
    pdmaPrms.fifoCnt  = 0U;

    retVal = Udma_chConfigPdma(txChHandle, &pdmaPrms);
    DebugP_assert(UDMA_SOK == retVal);

    retVal = Udma_chEnable(txChHandle);
    DebugP_assert(UDMA_SOK == retVal);

    /* Update host packet descriptor, length should be always in terms of total number of bytes */
    UART_udmaHpdInit(txChHandle, (uint8_t *) udmaChCfg->txHpdMem, hUart->writeBuf, transaction->count);

    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(txChHandle),
                 (uint64_t) Udma_defaultVirtToPhyFxn(udmaChCfg->txHpdMem, 0U, NULL));
    DebugP_assert(UDMA_SOK == retVal);

    return (retVal);
}

static int32_t UART_udmaConfigPdmaRx(UARTLLD_Handle hUart, const UART_Transaction *transaction)
{
    int32_t             retVal;
    Udma_ChPdmaPrms     pdmaPrms;
    Udma_ChHandle       rxChHandle;
    UART_UdmaChConfig   *udmaChCfg;

    udmaChCfg    = (UART_UdmaChConfig *)hUart->hUartInit->dmaChCfg;
    rxChHandle  = udmaChCfg->rxChHandle;

    /* Config PDMA channel */
    UdmaChPdmaPrms_init(&pdmaPrms);
    pdmaPrms.elemSize = UDMA_PDMA_ES_8BITS;

    /* Number of words received in each transfer */
    pdmaPrms.elemCnt = 1U;
    pdmaPrms.fifoCnt = transaction->count;

    retVal = Udma_chConfigPdma(rxChHandle, &pdmaPrms);
    DebugP_assert(UDMA_SOK == retVal);

    retVal = Udma_chEnable(rxChHandle);
    DebugP_assert(UDMA_SOK == retVal);

    /* Update host packet descriptor, length should be always in terms of total number of bytes */
    UART_udmaHpdInit(rxChHandle, (uint8_t *) udmaChCfg->rxHpdMem, hUart->readBuf, transaction->count);

    /* Submit HPD to channel */
    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(rxChHandle),
                 (uint64_t) Udma_defaultVirtToPhyFxn(udmaChCfg->rxHpdMem, 0U, NULL));
    DebugP_assert(UDMA_SOK == retVal);

    return (retVal);
}

static void UART_udmaHpdInit(Udma_ChHandle chHandle,
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

    /* Return Policy descriptors*/
    UART_udmapSetReturnPolicy(chHandle, pHpdMem);

    CSL_udmapCppi5LinkDesc(pHpd, 0U);
    CSL_udmapCppi5SetBufferAddr(pHpd, (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL));
    CSL_udmapCppi5SetBufferLen(pHpd, length);
    CSL_udmapCppi5SetOrgBufferAddr(pHpd, (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL));
    CSL_udmapCppi5SetOrgBufferLen(pHpd, length);

    /* Writeback cache */
    CacheP_wb(pHpdMem, sizeof(CSL_UdmapCppi5HMPD), CacheP_TYPE_ALLD);

    return;
}

static int32_t UART_udmaDeInitCh(Udma_ChHandle chHandle,
                                  Udma_EventHandle eventHandle)
{
    int32_t status = UDMA_SOK;
    uint32_t temp = TRUE;
    /* Disable Channel */
    status = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == status);

    /* UnRegister Event */
    status = Udma_eventUnRegister(eventHandle);
    DebugP_assert(UDMA_SOK == status);

    /* Flush any pending request from the free queue */
    while(temp == TRUE)
    {
        uint64_t pDesc;
        int32_t  tempRetVal;

        tempRetVal = Udma_ringFlushRaw(
                         Udma_chGetFqRingHandle(chHandle), &pDesc);
        if(UDMA_ETIMEOUT == tempRetVal)
        {
            temp = FALSE;
        }
    }

    /* Close channel */
    status = Udma_chClose(chHandle);
    DebugP_assert(UDMA_SOK == status);

    return status;
}

int32_t UART_lld_dmaDisableChannel(UARTLLD_Handle hUart,
                                       uint32_t isChannelTx)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    uint32_t temp = TRUE;
    UART_UdmaChConfig *udmaChCfg;
    Udma_ChHandle chHandle;

    udmaChCfg   = (UART_UdmaChConfig *)hUart->hUartInit->dmaChCfg;

    /* Disable Channel */
    if (isChannelTx == TRUE)
    {
        chHandle = udmaChCfg->txChHandle;
    }
    else
    {
        chHandle = udmaChCfg->rxChHandle;
    }

    status = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == status);

    /* Flush any pending request from the free queue */
    while(temp == TRUE)
    {
        uint64_t pDesc;
        int32_t  tempRetVal;

        tempRetVal = Udma_ringFlushRaw(
                         Udma_chGetFqRingHandle(chHandle), &pDesc);
        if(UDMA_ETIMEOUT == tempRetVal)
        {
            temp = FALSE;
        }
    }

    return status;
}
static void UART_udmaIsrTx(Udma_EventHandle eventHandle,
                                 uint32_t eventType,
                                 void *args)
{
    int32_t             retVal;
    uint64_t            pDesc;
    CSL_UdmapCppi5HMPD *pHpd;
    Udma_ChHandle      txChHandle;
    UART_UdmaChConfig   *udmaChCfg;
    UARTLLD_Handle        hUart;

    /* Check parameters */
    if(NULL != args)
    {
        hUart = (UARTLLD_Handle)args;
        udmaChCfg    = (UART_UdmaChConfig *)hUart->hUartInit->dmaChCfg;
        txChHandle  = udmaChCfg->txChHandle;

        if (eventType == UDMA_EVENT_TYPE_DMA_COMPLETION)
        {
            CacheP_inv(udmaChCfg->txHpdMem, udmaChCfg->hpdMemSize, CacheP_TYPE_ALLD);
            retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(txChHandle), &pDesc);
            if ((retVal == UDMA_SOK) && (pDesc != 0UL))
            {
                pHpd = (CSL_UdmapCppi5HMPD *)(uintptr_t)pDesc;
                hUart->writeTrans.status = UART_TRANSFER_STATUS_SUCCESS;
                /* Get Byte count transmitted */
                hUart->writeTrans.count = (pHpd->descInfo & CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_MASK) >> CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_SHIFT;
            }
            else
            {
                hUart->writeTrans.status = UART_TRANSFER_STATUS_ERROR_OTH;
            }

            hUart->hUartInit->writeCompleteCallbackFxn(hUart);
            UART_lld_Transaction_deInit(&hUart->writeTrans);
        }
        else
        {
            hUart->writeTrans.status = UART_TRANSFER_STATUS_ERROR_OTH;
        }
    }

    return;
}

static void UART_udmaIsrRx(Udma_EventHandle eventHandle,
                           uint32_t eventType,
                           void *args)
{
    int32_t             retVal;
    uint64_t            pDesc;
    CSL_UdmapCppi5HMPD *pHpd;
    Udma_ChHandle      rxChHandle;
    UART_UdmaChConfig   *udmaChCfg;
    UARTLLD_Handle        hUart;

    /* Check parameters */
    if(NULL != args)
    {
        hUart = (UARTLLD_Handle)args;
        udmaChCfg    = (UART_UdmaChConfig *)hUart->hUartInit->dmaChCfg;
        rxChHandle  = udmaChCfg->rxChHandle;

        if (eventType == UDMA_EVENT_TYPE_DMA_COMPLETION)
        {
            CacheP_inv(udmaChCfg->rxHpdMem, udmaChCfg->hpdMemSize, CacheP_TYPE_ALLD);
            retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(rxChHandle), &pDesc);
            if ((retVal == UDMA_SOK) && (pDesc != 0UL))
            {
                pHpd = (CSL_UdmapCppi5HMPD *)(uintptr_t)pDesc;
                /* Get Byte count received */
                hUart->readTrans.count = (pHpd->descInfo & CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_MASK) >> CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_SHIFT;
                hUart->readTrans.status = UART_TRANSFER_STATUS_SUCCESS;
            }
            else
            {
                hUart->readTrans.status = UART_TRANSFER_STATUS_ERROR_OTH;
            }

            hUart->hUartInit->readCompleteCallbackFxn(hUart);
            UART_lld_Transaction_deInit(&hUart->readTrans);
        }
        else
        {
            hUart->writeTrans.status = UART_TRANSFER_STATUS_ERROR_OTH;
        }
    }

    return;
}

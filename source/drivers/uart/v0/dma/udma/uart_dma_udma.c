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
#include <drivers/uart/v0/dma/uart_dma.h>
#include <drivers/uart/v0/dma/udma/uart_dma_udma.h>
#include <drivers/uart.h>
#include <kernel/dpl/CacheP.h>

/* Static Function Declarations */
static int32_t UART_udmaOpen(UART_Handle uartHandle, void* uartDmaArgs);
static int32_t UART_udmaInitTxCh(UART_Handle uartHandle, UartDma_UdmaArgs *udmaArgs);
static int32_t UART_udmaInitRxCh(UART_Handle uartHandle, UartDma_UdmaArgs *udmaArgs);
static int32_t UART_udmaTransferWrite(UART_Object *obj, const UART_Attrs *attrs,
                                      UART_Transaction *transaction);
static int32_t UART_udmaTransferRead(UART_Object *obj, const UART_Attrs *attrs,
                                     UART_Transaction *transaction);
static int32_t UART_udmaClose(UART_Handle handle);
static int32_t UART_udmaDisableChannel(UART_Handle handle,
                                       uint32_t isChannelTx);
static void UART_udmaHpdInit(Udma_ChHandle chHandle,
                              uint8_t       *pHpdMem,
                              const void    *destBuf,
                              uint32_t      length);
static int32_t UART_udmaConfigPdmaTx(UART_Object *obj,
                                     UART_Transaction *transaction);
static int32_t UART_udmaConfigPdmaRx(UART_Object *obj,
                                     UART_Transaction *transaction);
static int32_t UART_udmaDeInitCh(Udma_ChHandle chHandle,
                                 Udma_EventHandle eventHandle);
static void UART_udmaIsrRx(Udma_EventHandle eventHandle,
                           uint32_t eventType,
                           void *args);
static void UART_udmaIsrTx(Udma_EventHandle eventHandle,
                           uint32_t eventType,
                           void *args);

/* UDMA Function Pointers */
UART_DmaFxns gUartDmaUdmaFxns =
{
    .dmaOpenFxn               = UART_udmaOpen,
    .dmaTransferWriteFxn      = UART_udmaTransferWrite,
    .dmaTransferReadFxn       = UART_udmaTransferRead,
    .dmaCloseFxn              = UART_udmaClose,
    .dmaDisableChannelFxn     = UART_udmaDisableChannel,
};

static int32_t UART_udmaOpen(UART_Handle uartHandle, void* uartDmaArgs)
{
    int32_t status = SystemP_SUCCESS;
    UartDma_UdmaArgs *udmaArgs = (UartDma_UdmaArgs *)uartDmaArgs;

    status  = UART_udmaInitRxCh(uartHandle, udmaArgs);
    status |= UART_udmaInitTxCh(uartHandle, udmaArgs);
    if (status == UDMA_SOK)
    {
        udmaArgs->isOpen = TRUE;
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t UART_udmaInitRxCh(UART_Handle uartHandle, UartDma_UdmaArgs *udmaArgs)
{
    int32_t             retVal;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_DrvHandle      drvHandle;
    Udma_ChHandle       rxChHandle;
    UART_Config        *config;
    UART_Object        *object;
    UART_Params        *prms;

    config  = (UART_Config *) uartHandle;
    object  = config->object;
    DebugP_assert(NULL != object);
    prms    = &config->object->prms;

    /* Init RX channel parameters */
    chType = UDMA_CH_TYPE_PDMA_RX;
    UdmaChPrms_init(&chPrms, chType);
    chPrms.peerChNum                = prms->rxEvtNum;
    chPrms.fqRingPrms.ringMem       = udmaArgs->rxRingMem;
    chPrms.fqRingPrms.ringMemSize   = udmaArgs->ringMemSize;
    chPrms.fqRingPrms.elemCnt       = udmaArgs->ringElemCnt;
    rxChHandle                      = udmaArgs->rxChHandle;
    drvHandle                       = udmaArgs->drvHandle;

    /* Open channel for block copy */
    retVal = Udma_chOpen(drvHandle, rxChHandle, chType, &chPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config RX channel */
    UdmaChRxPrms_init(&rxPrms, chType);
    retVal = Udma_chConfigRx(rxChHandle, &rxPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Register ring completion callback */
    eventHandle = udmaArgs->cqRxEvtHandle;
    UdmaEventPrms_init(&eventPrms);
    eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    eventPrms.chHandle          = rxChHandle;
    eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
    eventPrms.eventCb           = &UART_udmaIsrRx;
    eventPrms.appData           = (void *) uartHandle;
    retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);

    return retVal;
}

static int32_t UART_udmaInitTxCh(UART_Handle uartHandle, UartDma_UdmaArgs *udmaArgs)
{
    int32_t             retVal;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChTxPrms       txPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_DrvHandle      drvHandle;
    Udma_ChHandle       txChHandle;
    UART_Config        *config;
    UART_Object        *object;
    UART_Params        *prms;

    config  = (UART_Config *) uartHandle;
    object  = config->object;
    DebugP_assert(NULL != object);
    prms    = &config->object->prms;

    /* Init TX channel parameters */
    chType = UDMA_CH_TYPE_PDMA_TX;
    UdmaChPrms_init(&chPrms, chType);
    chPrms.peerChNum                = prms->txEvtNum;
    chPrms.fqRingPrms.ringMem       = udmaArgs->txRingMem;
    chPrms.fqRingPrms.ringMemSize   = udmaArgs->ringMemSize;
    chPrms.fqRingPrms.elemCnt       = udmaArgs->ringElemCnt;
    txChHandle                      = udmaArgs->txChHandle;
    drvHandle                       = udmaArgs->drvHandle;

    /* Open channel for block copy */
    retVal = Udma_chOpen(drvHandle, txChHandle, chType, &chPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config TX channel */
    UdmaChTxPrms_init(&txPrms, chType);
    retVal = Udma_chConfigTx(txChHandle, &txPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Register ring completion callback */
    eventHandle = udmaArgs->cqTxEvtHandle;
    UdmaEventPrms_init(&eventPrms);
    eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    eventPrms.chHandle          = txChHandle;
    eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
    eventPrms.eventCb           = &UART_udmaIsrTx;
    eventPrms.appData           = (void *) uartHandle;
    retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);

    return retVal;
}

static int32_t UART_udmaTransferWrite(UART_Object *obj, const UART_Attrs *attrs,
                                      UART_Transaction *transaction)
{
    int32_t status = SystemP_SUCCESS;

    status = UART_udmaConfigPdmaTx(obj, transaction);
    if (status != UDMA_SOK)
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t UART_udmaTransferRead(UART_Object *obj, const UART_Attrs *attrs,
                                     UART_Transaction *transaction)
{
    int32_t status = SystemP_SUCCESS;

    status = UART_udmaConfigPdmaRx(obj, transaction);
    if (status != UDMA_SOK)
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t UART_udmaClose(UART_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    UART_Config   *config;
    UART_DmaHandle dmaHandle;
    UART_DmaConfig *dmaConfig;
    UartDma_UdmaArgs *udmaArgs;

    config = (UART_Config *) handle;
    dmaHandle = config->object->uartDmaHandle;
    dmaConfig = (UART_DmaConfig *)dmaHandle;
    udmaArgs = (UartDma_UdmaArgs *)dmaConfig->uartDmaArgs;

    if (udmaArgs->isOpen != FALSE)
    {
        UART_udmaDeInitCh(udmaArgs->rxChHandle,
                          udmaArgs->cqRxEvtHandle);
        UART_udmaDeInitCh(udmaArgs->txChHandle,
                          udmaArgs->cqTxEvtHandle);
        udmaArgs->isOpen = FALSE;
    }

    return status;
}

static int32_t UART_udmaConfigPdmaTx(UART_Object *obj,
                                     UART_Transaction *transaction)
{
    int32_t             retVal;
    Udma_ChPdmaPrms     pdmaPrms;
    Udma_ChHandle       txChHandle;
    UART_DmaConfig     *dmaConfig;
    UartDma_UdmaArgs   *udmaArgs;
    UART_DmaHandle dmaHandle;

    dmaHandle = obj->uartDmaHandle;
    dmaConfig = (UART_DmaConfig *)dmaHandle;
    udmaArgs = (UartDma_UdmaArgs *)dmaConfig->uartDmaArgs;
    txChHandle  = udmaArgs->txChHandle;

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
    UART_udmaHpdInit(txChHandle, (uint8_t *) udmaArgs->txHpdMem, obj->writeBuf, transaction->count);

    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(txChHandle),
                 (uint64_t) Udma_defaultVirtToPhyFxn(udmaArgs->txHpdMem, 0U, NULL));
    DebugP_assert(UDMA_SOK == retVal);

    return (retVal);
}

static int32_t UART_udmaConfigPdmaRx(UART_Object *obj,
                                     UART_Transaction *transaction)
{
    int32_t             retVal;
    Udma_ChPdmaPrms     pdmaPrms;
    Udma_ChHandle       rxChHandle;
    UART_DmaConfig     *dmaConfig;
    UartDma_UdmaArgs   *udmaArgs;
    UART_DmaHandle      dmaHandle;

    dmaHandle = obj->uartDmaHandle;
    dmaConfig = (UART_DmaConfig *)dmaHandle;
    udmaArgs = (UartDma_UdmaArgs *)dmaConfig->uartDmaArgs;
    rxChHandle  = udmaArgs->rxChHandle;

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
    UART_udmaHpdInit(rxChHandle, (uint8_t *) udmaArgs->rxHpdMem, obj->readBuf, transaction->count);

    /* Submit HPD to channel */
    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(rxChHandle),
                 (uint64_t) Udma_defaultVirtToPhyFxn(udmaArgs->rxHpdMem, 0U, NULL));
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

static int32_t UART_udmaDeInitCh(Udma_ChHandle chHandle,
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

static int32_t UART_udmaDisableChannel(UART_Handle handle,
                                       uint32_t isChannelTx)
{
    int32_t status = SystemP_SUCCESS;
    UART_Config   *config;
    UART_DmaHandle dmaHandle;
    UART_DmaConfig *dmaConfig;
    UartDma_UdmaArgs *udmaArgs;
    Udma_ChHandle chHandle;

    config = (UART_Config *) handle;
    dmaHandle = config->object->uartDmaHandle;
    dmaConfig = (UART_DmaConfig *)dmaHandle;
    udmaArgs = (UartDma_UdmaArgs *)dmaConfig->uartDmaArgs;

    /* Disable Channel */
    if (isChannelTx == TRUE)
    {
        chHandle = udmaArgs->txChHandle;
    }
    else
    {
        chHandle = udmaArgs->rxChHandle;
    }

    status = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
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

    return status;
}
static void UART_udmaIsrTx(Udma_EventHandle eventHandle,
                                 uint32_t eventType,
                                 void *args)
{
    int32_t             retVal;
    uint64_t            pDesc;
    CSL_UdmapCppi5HMPD *pHpd;
    UART_Config       *config;
    UART_Object       *obj;
    Udma_ChHandle      txChHandle;
    UART_DmaConfig     *dmaConfig;
    UartDma_UdmaArgs   *udmaArgs;
    UART_DmaHandle      dmaHandle;

    /* Check parameters */
    if(NULL != args)
    {
        config = (UART_Config *) args;
        obj = config->object;
        DebugP_assert(NULL != obj);

        dmaHandle = obj->uartDmaHandle;
        dmaConfig = (UART_DmaConfig *)dmaHandle;
        udmaArgs = (UartDma_UdmaArgs *)dmaConfig->uartDmaArgs;
        txChHandle  = udmaArgs->txChHandle;

        if (eventType == UDMA_EVENT_TYPE_DMA_COMPLETION)
        {
            CacheP_inv(udmaArgs->txHpdMem, udmaArgs->hpdMemSize, CacheP_TYPE_ALLD);
            retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(txChHandle), &pDesc);
            if ((retVal == UDMA_SOK) && (pDesc != 0UL))
            {
                pHpd = (CSL_UdmapCppi5HMPD *)(uintptr_t)pDesc;
                obj->writeTrans->status = UART_TRANSFER_STATUS_SUCCESS;
                /* Get Byte count transmitted */
                obj->writeTrans->count = (pHpd->descInfo & CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_MASK) >> CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_SHIFT;
            }
            else
            {
                obj->writeTrans->status = UART_TRANSFER_STATUS_ERROR_OTH;
            }

            /*
            * Post transfer Sem in case of bloacking transfer.
            * Call the callback function in case of Callback mode.
            */
            if (obj->prms.writeMode == UART_TRANSFER_MODE_CALLBACK)
            {
                 obj->prms.writeCallbackFxn((UART_Handle) config, obj->writeTrans);
            }
            else
            {
                (void)SemaphoreP_post(&obj->writeTransferSemObj);
            }
            obj->writeTrans = NULL;
        }
        else
        {
            obj->writeTrans->status = UART_TRANSFER_STATUS_ERROR_OTH;
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
    UART_Config       *config;
    UART_Object       *obj;
    Udma_ChHandle      rxChHandle;
    UART_DmaConfig     *dmaConfig;
    UartDma_UdmaArgs   *udmaArgs;
    UART_DmaHandle      dmaHandle;

    /* Check parameters */
    if(NULL != args)
    {
        config = (UART_Config *) args;
        obj = config->object;
        DebugP_assert(NULL != obj);

        dmaHandle = obj->uartDmaHandle;
        dmaConfig = (UART_DmaConfig *)dmaHandle;
        udmaArgs = (UartDma_UdmaArgs *)dmaConfig->uartDmaArgs;
        rxChHandle  = udmaArgs->rxChHandle;

        if (eventType == UDMA_EVENT_TYPE_DMA_COMPLETION)
        {
            CacheP_inv(udmaArgs->rxHpdMem, udmaArgs->hpdMemSize, CacheP_TYPE_ALLD);
            retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(rxChHandle), &pDesc);
            if ((retVal == UDMA_SOK) && (pDesc != 0UL))
            {
                pHpd = (CSL_UdmapCppi5HMPD *)(uintptr_t)pDesc;
                /* Get Byte count received */
                obj->readTrans->count = (pHpd->descInfo & CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_MASK) >> CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_SHIFT;
                obj->readTrans->status = UART_TRANSFER_STATUS_SUCCESS;
            }
            else
            {
                obj->readTrans->status = UART_TRANSFER_STATUS_ERROR_OTH;
            }

            /*
            * Post transfer Sem in case of bloacking transfer.
            * Call the callback function in case of Callback mode.
            */
            if (obj->prms.readMode == UART_TRANSFER_MODE_CALLBACK)
            {
                obj->prms.readCallbackFxn((UART_Handle) config, obj->readTrans);
            }
            else
            {
                (void)SemaphoreP_post(&obj->readTransferSemObj);
            }
            obj->readTrans = NULL;
        }
        else
        {
            obj->writeTrans->status = UART_TRANSFER_STATUS_ERROR_OTH;
        }
    }

    return;
}

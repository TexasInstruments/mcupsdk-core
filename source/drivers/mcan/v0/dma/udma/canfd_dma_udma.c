/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 *  \file canfd_dma_udma.c
 *
 *  \brief File containing EDMA Driver APIs implementation for MCAN.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <drivers/soc.h>
#include <drivers/udma.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/mcan/v0/canfd.h>
#include <drivers/mcan/v0/dma/udma/canfd_dma_udma.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void CANFD_dmaTxCallBack(CANFD_MessageObject* ptrCanMsgObj);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t CANFD_dmaOpen(CANFD_Handle canfdHandle, CANFD_DmaChConfig dmaChCfg)
{
    int32_t status = SystemP_SUCCESS;
    return status;
}

static void CANFD_udmaIsrTx(Udma_EventHandle eventHandle,
                                 uint32_t eventType,
                                 void *args)
{
    uint8_t *currentDataPtr;
    uint64_t pDesc;

    if(NULL_PTR != args)
    {
        CANFD_MessageObject* ptrCanMsgObj = (CANFD_MessageObject *)(args);
        CANFD_Object *ptrCanFdObj         = ptrCanMsgObj->canfdHandle->object;
        CANFD_UdmaChConfig *udmaChCfg     = (CANFD_UdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
        Udma_ChHandle txChHandle          =  udmaChCfg->txChHandle[ptrCanMsgObj->dmaEventNo];
        
        ptrCanMsgObj->dmaMsgConfig.currentMsgNum++;
        currentDataPtr = (uint8_t *)(ptrCanMsgObj->dmaMsgConfig.data);

        CacheP_inv(udmaChCfg->txHpdMem[ptrCanMsgObj->dmaEventNo], udmaChCfg->hpdMemSize, CacheP_TYPE_ALLD);
        (void)Udma_ringDequeueRaw(Udma_chGetCqRingHandle(txChHandle), &pDesc);

        CANFD_dmaTxCompletionCallback(ptrCanMsgObj, (void *)(currentDataPtr), CANFD_DMA_TX_COMPLETION_FINAL);
    }
}

int32_t CANFD_createDmaTxMsgObject(const CANFD_Object *ptrCanFdObj, CANFD_MessageObject* ptrCanMsgObj)
{
    uint32_t            i;
    int32_t             retVal = SystemP_FAILURE;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChTxPrms       txPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_ChHandle       txChHandle;
    Udma_DrvHandle      canfdUdmaHandle;
    CANFD_UdmaChConfig *udmaChCfg;
    
    if((NULL_PTR != ptrCanFdObj) && (NULL_PTR != ptrCanMsgObj))
    {
        udmaChCfg = (CANFD_UdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
        canfdUdmaHandle = (Udma_DrvHandle)ptrCanFdObj->canfdDmaHandle;

        /* Check the free Tx dma event to program */
        for (i = 0U; i < MCAN_MAX_TX_DMA_BUFFERS; i++)
        {
            if ((udmaChCfg->udmaTxChAlloc & ((uint32_t)1U << i)) == (uint32_t)0U)
            {
                udmaChCfg->udmaTxChAlloc |= ((uint32_t)1U << i);
                ptrCanMsgObj->dmaEventNo = i;
                break;
            }
        }
        if (i == MCAN_MAX_TX_DMA_BUFFERS)
        {
            /* Error: Unable to allocate the memory */
            retVal = SystemP_FAILURE;
        }
        else
        {
            /* Init TX channel parameters */
            chType = UDMA_CH_TYPE_PDMA_TX;
            UdmaChPrms_init(&chPrms, chType);
            chPrms.peerChNum                = udmaChCfg->txEvtNum[ptrCanMsgObj->dmaEventNo];
            chPrms.fqRingPrms.ringMem       = udmaChCfg->txRingMem[ptrCanMsgObj->dmaEventNo];
            chPrms.fqRingPrms.ringMemSize   = udmaChCfg->ringMemSize;
            chPrms.fqRingPrms.elemCnt       = udmaChCfg->ringElemCnt;
            txChHandle                      =  udmaChCfg->txChHandle[ptrCanMsgObj->dmaEventNo];

            /* Open channel for block copy */
            retVal = Udma_chOpen(canfdUdmaHandle, txChHandle, chType, &chPrms);
            DebugP_assert(UDMA_SOK == retVal);

            /* Config TX channel */
            UdmaChTxPrms_init(&txPrms, chType);
            retVal = Udma_chConfigTx(txChHandle, &txPrms);
            DebugP_assert(UDMA_SOK == retVal);

            /* Register ring completion callback */
            eventHandle =  udmaChCfg->cqTxEvtHandle[ptrCanMsgObj->dmaEventNo];
            UdmaEventPrms_init(&eventPrms);
            eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
            eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
            eventPrms.chHandle          = txChHandle;
            eventPrms.controllerEventHandle = Udma_eventGetGlobalHandle(canfdUdmaHandle);
            eventPrms.eventCb           = &CANFD_udmaIsrTx;
            eventPrms.appData           = (void *) ptrCanMsgObj;
            retVal = Udma_eventRegister(canfdUdmaHandle, eventHandle, &eventPrms);
            DebugP_assert(UDMA_SOK == retVal);
            retVal = SystemP_SUCCESS;
        }
    }

    return retVal;
}

int32_t CANFD_deleteDmaTxMsgObject(const CANFD_Object *ptrCanFdObj, const CANFD_MessageObject* ptrCanMsgObj)
{
    int32_t status = SystemP_FAILURE;
    CANFD_UdmaChConfig *udmaChCfg;
    Udma_ChHandle       txChHandle;
    Udma_EventHandle    eventHandle;

    if((NULL_PTR != ptrCanFdObj) && (NULL_PTR != ptrCanMsgObj))
    {
        udmaChCfg = (CANFD_UdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
        txChHandle                      =  udmaChCfg->txChHandle[ptrCanMsgObj->dmaEventNo];
        eventHandle =  udmaChCfg->cqTxEvtHandle[ptrCanMsgObj->dmaEventNo];

        /* Disable Channel */
        status = Udma_chDisable(txChHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
        DebugP_assert(UDMA_SOK == status);

        /* UnRegister Event */
        status = Udma_eventUnRegister(eventHandle);
        DebugP_assert(UDMA_SOK == status);

        /* Flush any pending request from the free queue */
        while(true)
        {
            uint64_t pDesc;
            int32_t  tempRetVal;

            tempRetVal = Udma_ringFlushRaw(
                            Udma_chGetFqRingHandle(txChHandle), &pDesc);
            if(UDMA_ETIMEOUT == tempRetVal)
            {
                break;
            }
        }

        /* Close channel */
        status = Udma_chClose(txChHandle);
        DebugP_assert(UDMA_SOK == status);
        status = SystemP_SUCCESS;
    }

    return status;
}

void CANFD_dmaTxCallBack(CANFD_MessageObject* ptrCanMsgObj)
{

    if (ptrCanMsgObj->dmaMsgConfig.currentMsgNum == (ptrCanMsgObj->dmaMsgConfig.numMsgs))
    {
        ptrCanMsgObj->dmaMsgConfig.currentMsgNum++;
    }
    else if (ptrCanMsgObj->dmaMsgConfig.currentMsgNum < (ptrCanMsgObj->dmaMsgConfig.numMsgs))
    {
        ptrCanMsgObj->dmaMsgConfig.currentMsgNum++;
    }
    else
    {
        /* Callback for all msgs are called. This should not be called. */
    }
}

static void CANFD_udmaHpdInit(Udma_ChHandle chHandle,
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

int32_t CANFD_configureDmaTx(const CANFD_Object *ptrCanFdObj, CANFD_MessageObject* ptrCanMsgObj, uint32_t dataLengthPerMsg, uint32_t numMsgs, const void* data)
{
    int32_t             status = SystemP_FAILURE;
    int32_t             retVal;
    Udma_ChPdmaPrms     pdmaPrms;
    Udma_ChHandle       txChHandle;
    CANFD_UdmaChConfig *udmaChCfg;

    if((NULL_PTR != ptrCanFdObj) && (NULL_PTR != ptrCanMsgObj))
    {
        udmaChCfg   = (CANFD_UdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
        txChHandle  =  udmaChCfg->txChHandle[ptrCanMsgObj->dmaEventNo];

        /* Store the current Tx msg. */
        ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg = dataLengthPerMsg;
        ptrCanMsgObj->dmaMsgConfig.numMsgs          = numMsgs;
        ptrCanMsgObj->dmaMsgConfig.data             = data;
        ptrCanMsgObj->dmaMsgConfig.currentMsgNum    = 0U;

        /* Config PDMA channel */
        UdmaChPdmaPrms_init(&pdmaPrms);

        pdmaPrms.elemSize = UDMA_PDMA_ES_8BITS;

        /* Number of words received in each transfer */
        pdmaPrms.elemCnt = dataLengthPerMsg + MCAN_MSG_HEADER_SIZE;
        /* Dont care for Tx */
        pdmaPrms.fifoCnt    = 0U;

        retVal = Udma_chConfigPdma(txChHandle, &pdmaPrms);
        DebugP_assert(UDMA_SOK == retVal);

        retVal = Udma_chEnable(txChHandle);
        DebugP_assert(UDMA_SOK == retVal);

        /* Update host packet descriptor, length should be always in terms of total number of bytes */
        CANFD_udmaHpdInit(txChHandle, (uint8_t *) udmaChCfg->txHpdMem[ptrCanMsgObj->dmaEventNo], data, MCAN_MSG_HEADER_SIZE+(dataLengthPerMsg*numMsgs));

        retVal = Udma_ringQueueRaw(
                    Udma_chGetFqRingHandle(txChHandle),
                    (uint64_t) Udma_defaultVirtToPhyFxn(udmaChCfg->txHpdMem[ptrCanMsgObj->dmaEventNo], 0U, NULL));
        DebugP_assert(UDMA_SOK == retVal);
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t CANFD_cancelDmaTx(const CANFD_Object *ptrCanFdObj, const CANFD_MessageObject* ptrCanMsgObj)
{
    int32_t            status = SystemP_SUCCESS;
    return status;
}

//TODO: udma should generate interrupts and call application callback for every message
static void CANFD_udmaIsrRx(Udma_EventHandle eventHandle,
                                 uint32_t eventType,
                                 void *args)
{
    if(NULL_PTR != args)
    {
        CANFD_MessageObject* ptrCanMsgObj = (CANFD_MessageObject *)(args);
        CANFD_Object *ptrCanFdObj         = ptrCanMsgObj->canfdHandle->object;
        CANFD_UdmaChConfig *udmaChCfg     = (CANFD_UdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
        Udma_ChHandle rxChHandle          =  udmaChCfg->rxChHandle[ptrCanMsgObj->dmaEventNo];
        uint8_t *currentDataPtr;
        uint64_t pDesc;

        ptrCanMsgObj->dmaMsgConfig.currentMsgNum++;
        currentDataPtr = (uint8_t *)(ptrCanMsgObj->dmaMsgConfig.data);

        CacheP_inv(udmaChCfg->rxHpdMem[ptrCanMsgObj->dmaEventNo], udmaChCfg->hpdMemSize, CacheP_TYPE_ALLD);
        (void)Udma_ringDequeueRaw(Udma_chGetCqRingHandle(rxChHandle), &pDesc);

        CANFD_dmaRxCompletionCallback(ptrCanMsgObj, (void *)(currentDataPtr), CANFD_DMA_RX_COMPLETION_FINAL);
    }
}

int32_t CANFD_createDmaRxMsgObject(const CANFD_Object *ptrCanFdObj, CANFD_MessageObject* ptrCanMsgObj)
{
    int32_t             retVal = SystemP_FAILURE;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_ChHandle       rxChHandle;
    Udma_DrvHandle      canfdUdmaHandle;
    CANFD_UdmaChConfig *udmaChCfg;

    if((NULL_PTR != ptrCanFdObj) && (NULL_PTR != ptrCanMsgObj))
    {
        canfdUdmaHandle = (Udma_DrvHandle)ptrCanFdObj->canfdDmaHandle;
        udmaChCfg = (CANFD_UdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
        ptrCanMsgObj->dmaEventNo = ptrCanMsgObj->rxElement;
        if(ptrCanMsgObj->dmaEventNo < MCAN_MAX_RX_DMA_BUFFERS)
        {
            /* Init TX channel parameters */
            chType = UDMA_CH_TYPE_PDMA_RX;
            UdmaChPrms_init(&chPrms, chType);
            chPrms.peerChNum                = udmaChCfg->rxEvtNum[ptrCanMsgObj->dmaEventNo];
            chPrms.fqRingPrms.ringMem       = udmaChCfg->rxRingMem[ptrCanMsgObj->dmaEventNo];
            chPrms.fqRingPrms.ringMemSize   = udmaChCfg->ringMemSize;
            chPrms.fqRingPrms.elemCnt       = udmaChCfg->ringElemCnt;
            rxChHandle                      =  udmaChCfg->rxChHandle[ptrCanMsgObj->dmaEventNo];

            /* Open channel for block copy */
            retVal = Udma_chOpen(canfdUdmaHandle, rxChHandle, chType, &chPrms);
            DebugP_assert(UDMA_SOK == retVal);

            /* Config TX channel */
            UdmaChRxPrms_init(&rxPrms, chType);
            retVal = Udma_chConfigRx(rxChHandle, &rxPrms);
            DebugP_assert(UDMA_SOK == retVal);

            /* Register ring completion callback */
            eventHandle =  udmaChCfg->cqRxEvtHandle[ptrCanMsgObj->dmaEventNo];
            UdmaEventPrms_init(&eventPrms);
            eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
            eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
            eventPrms.chHandle          = rxChHandle;
            eventPrms.controllerEventHandle = Udma_eventGetGlobalHandle(canfdUdmaHandle);
            eventPrms.eventCb           = &CANFD_udmaIsrRx;
            eventPrms.appData           = (void *) ptrCanMsgObj;
            retVal = Udma_eventRegister(canfdUdmaHandle, eventHandle, &eventPrms);
            DebugP_assert(UDMA_SOK == retVal);
            retVal = SystemP_SUCCESS;
        }
    }

    return retVal;
}

int32_t CANFD_deleteDmaRxMsgObject(const CANFD_Object *ptrCanFdObj, const CANFD_MessageObject* ptrCanMsgObj)
{
    int32_t status = SystemP_FAILURE;
    CANFD_UdmaChConfig *udmaChCfg;
    Udma_ChHandle       rxChHandle;
    Udma_EventHandle    eventHandle;

    if((NULL_PTR != ptrCanFdObj) && (NULL_PTR != ptrCanMsgObj))
    {
        udmaChCfg   = (CANFD_UdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
        rxChHandle  =  udmaChCfg->rxChHandle[ptrCanMsgObj->dmaEventNo];
        eventHandle =  udmaChCfg->cqRxEvtHandle[ptrCanMsgObj->dmaEventNo];

        /* Disable Channel */
        status = Udma_chDisable(rxChHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
        DebugP_assert(UDMA_SOK == status);

        /* UnRegister Event */
        status = Udma_eventUnRegister(eventHandle);
        DebugP_assert(UDMA_SOK == status);

        /* Flush any pending request from the free queue */
        while(true)
        {
            uint64_t pDesc;
            int32_t  tempRetVal;

            tempRetVal = Udma_ringFlushRaw(
                            Udma_chGetFqRingHandle(rxChHandle), &pDesc);
            if(UDMA_ETIMEOUT == tempRetVal)
            {
                break;
            }
        }

        /* Close channel */
        status = Udma_chClose(rxChHandle);
        DebugP_assert(UDMA_SOK == status);
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t CANFD_configureDmaRx(const CANFD_Object *ptrCanFdObj, CANFD_MessageObject* ptrCanMsgObj, uint32_t dataLengthPerMsg, uint32_t numMsgs, const void* data)
{
    int32_t             status = SystemP_FAILURE;
    int32_t             retVal;
    Udma_ChPdmaPrms     pdmaPrms;
    Udma_ChHandle       rxChHandle;
    CANFD_UdmaChConfig *udmaChCfg;

    if((NULL_PTR != ptrCanFdObj) && (NULL_PTR != ptrCanMsgObj))
    {
        udmaChCfg  = (CANFD_UdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;;
        rxChHandle =  udmaChCfg->rxChHandle[ptrCanMsgObj->dmaEventNo];

        /* Store the current Rx msg. */
        ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg = dataLengthPerMsg;
        ptrCanMsgObj->dmaMsgConfig.numMsgs          = numMsgs;
        ptrCanMsgObj->dmaMsgConfig.data             = data;
        ptrCanMsgObj->dmaMsgConfig.currentMsgNum    = 0U;

        /* Config PDMA channel */
        UdmaChPdmaPrms_init(&pdmaPrms);

        pdmaPrms.elemSize = UDMA_PDMA_ES_8BITS;

        /* Number of words received in each transfer */
        pdmaPrms.elemCnt = dataLengthPerMsg;
        /* Dont care for Tx */
        pdmaPrms.fifoCnt    = numMsgs;

        retVal = Udma_chConfigPdma(rxChHandle, &pdmaPrms);
        DebugP_assert(UDMA_SOK == retVal);

        retVal = Udma_chEnable(rxChHandle);
        DebugP_assert(UDMA_SOK == retVal);

        /* Update host packet descriptor, length should be always in terms of total number of bytes */
        CANFD_udmaHpdInit(rxChHandle, (uint8_t *) udmaChCfg->rxHpdMem[ptrCanMsgObj->dmaEventNo], data, dataLengthPerMsg*numMsgs);

        retVal = Udma_ringQueueRaw(
                    Udma_chGetFqRingHandle(rxChHandle),
                    (uint64_t) Udma_defaultVirtToPhyFxn(udmaChCfg->rxHpdMem[ptrCanMsgObj->dmaEventNo], 0U, NULL));
        DebugP_assert(UDMA_SOK == retVal);
        status = SystemP_SUCCESS;
    }

    return status;
}


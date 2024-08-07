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
 *  \file canfd_dma_edma.c
 *
 *  \brief File containing EDMA Driver APIs implementation for CANFD.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <drivers/soc.h>
#include <drivers/mcan/v0/dma/edma/canfd_dma_edma.h>
#include <drivers/mcan/v0/canfd.h>
#include <kernel/dpl/CacheP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** \brief Transmit EDMA channel event queue number                           */
#define EDMA_CANFD_TX_EVT_QUEUE_NO                  (0U)
/** \brief Receive EDMA channel event queue number                            */
#define EDMA_CANFD_RX_EVT_QUEUE_NO                  (1U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void CANFD_dmaTxCallBack(CANFD_MessageObject* ptrCanMsgObj);
static void CANFD_edmaIsrTx(Edma_IntrHandle intrHandle, void *args);
static void CANFD_edmaIsrRx(Edma_IntrHandle intrHandle, void *args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t CANFD_dmaOpen(CANFD_Handle canfdHandle, CANFD_DmaChConfig dmaChCfg)
{
    int32_t             status = SystemP_FAILURE;
    CANFD_EdmaChConfig *edmaChCfg = NULL;
    uint32_t            baseAddr, regionId;
    uint32_t            isEdmaInterruptEnabled;
    EDMA_Handle         canfdEdmaHandle;
    CANFD_Object       *object;

    if((NULL != canfdHandle) && (NULL != dmaChCfg))
    {
        edmaChCfg = (CANFD_EdmaChConfig *)dmaChCfg;
        object = (CANFD_Object *) canfdHandle;
        canfdEdmaHandle = (EDMA_Handle) object->canfdDmaHandle;

        if(canfdEdmaHandle != NULL)
        {
            /* Read base address of allocated EDMA instance */
            baseAddr = EDMA_getBaseAddr(canfdEdmaHandle);

            /* Read the region ID of the EDMA instance */
            regionId = EDMA_getRegionId(canfdEdmaHandle);

            /* Store the EDMA parameters */
            edmaChCfg->edmaBaseAddr = baseAddr;
            edmaChCfg->edmaRegionId = regionId;

            /* Check if interrupt is enabled */
            isEdmaInterruptEnabled = EDMA_isInterruptEnabled(canfdEdmaHandle);

            if((baseAddr != 0U) && (regionId < SOC_EDMA_NUM_REGIONS) && (isEdmaInterruptEnabled == TRUE))
            {
                /* Validate the EDMA parameters for MCAN */
                edmaChCfg->isOpen = TRUE;
                edmaChCfg->edmaTxChAlloc = 0U;
                edmaChCfg->edmaRxChAlloc = 0U;
                status = SystemP_SUCCESS;
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t CANFD_createDmaTxMsgObject(const CANFD_Object *ptrCanFdObj, CANFD_MessageObject* ptrCanMsgObj)
{
    uint32_t i, ret;
    int32_t status = SystemP_SUCCESS, chAllocStatus;
    EDMA_Handle canfdEdmaHandle = NULL;
    CANFD_EdmaChConfig *edmaChCfg = NULL;

    if((NULL != ptrCanFdObj) && (NULL != ptrCanMsgObj))
    {
        canfdEdmaHandle = (EDMA_Handle) ptrCanFdObj->canfdDmaHandle;
        edmaChCfg       = (CANFD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
        /* Check the free Tx dma event to program */
        for (i = 0U; i < MCAN_MAX_TX_DMA_BUFFERS; i++)
        {
            if ((edmaChCfg->edmaTxChAlloc && (uint32_t)(1U << i)) == 0U)
            {
                edmaChCfg->edmaTxChAlloc |= (1U << i);
                ptrCanMsgObj->dmaEventNo = i;
                break;
            }
        }
        if (i == MCAN_MAX_TX_DMA_BUFFERS)
        {
            /* Error: Unable to allocate the memory */
            status = MCAN_OUT_OF_RESOURCES;
        }
        if (status == SystemP_SUCCESS)
        {
            /* Allocate EDMA channel for CANFD TX transfer */
            chAllocStatus = EDMA_allocDmaChannel(canfdEdmaHandle, &(edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo]));
            status += chAllocStatus;

            /* Allocate EDMA TCC for CANFD TX transfer */
            edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocTcc(canfdEdmaHandle, &(edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo]));

            /* Allocate a Param ID for CANFD TX transfer */
            edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocParam(canfdEdmaHandle, &(edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo]));

            if(status == SystemP_SUCCESS)
            {
                ret = EDMA_configureChannelRegion(edmaChCfg->edmaBaseAddr, edmaChCfg->edmaRegionId, EDMA_CHANNEL_TYPE_DMA,
                        edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo], edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo],
                        edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo], EDMA_CANFD_TX_EVT_QUEUE_NO);
                if(ret == (uint32_t)TRUE)
                {
                    status = SystemP_SUCCESS;
                }
                else
                {
                    status = SystemP_FAILURE;
                }
                /* Register TX interrupt */
                edmaChCfg->edmaIntrObjTx[ptrCanMsgObj->dmaEventNo].tccNum = edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo];
                edmaChCfg->edmaIntrObjTx[ptrCanMsgObj->dmaEventNo].cbFxn  = &CANFD_edmaIsrTx;
                edmaChCfg->edmaIntrObjTx[ptrCanMsgObj->dmaEventNo].appData = (void *) ptrCanMsgObj;
                status += EDMA_registerIntr(canfdEdmaHandle, &(edmaChCfg->edmaIntrObjTx[ptrCanMsgObj->dmaEventNo]));
            }

            if (status != SystemP_SUCCESS)
            {
                /* Free all allocated resources of edma */
                if ((chAllocStatus == SystemP_SUCCESS) && (edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY))
                {
                    EDMA_freeDmaChannel(canfdEdmaHandle, &(edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo]));
                }
                if (edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
                {
                    EDMA_freeTcc(canfdEdmaHandle, &(edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo]));
                }
                if (edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
                {
                    EDMA_freeParam(canfdEdmaHandle, &(edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo]));
                }
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t CANFD_deleteDmaTxMsgObject(const CANFD_Object *ptrCanFdObj, const CANFD_MessageObject* ptrCanMsgObj)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Handle         canfdEdmaHandle;
    CANFD_EdmaChConfig *edmaChCfg;

    if ((ptrCanFdObj != NULL) || (ptrCanMsgObj != NULL))
    {
        canfdEdmaHandle = (EDMA_Handle) ptrCanFdObj->canfdDmaHandle;
        edmaChCfg = (CANFD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

        if ((canfdEdmaHandle == NULL) || (edmaChCfg == NULL))
        {
            status = SystemP_FAILURE;
        }
        else
        {
            /* Free all allocated resources of edma */
            if (edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeDmaChannel(canfdEdmaHandle, &(edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo]));
            }

            if (edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeTcc(canfdEdmaHandle, &(edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo]));
                edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
            }

            if (edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeParam(canfdEdmaHandle, &(edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo]));
                edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

void CANFD_dmaTxCallBack(CANFD_MessageObject* ptrCanMsgObj)
{
    uint8_t            *currentDataPtr;
    uint32_t            baseAddr, regionId, dmaTxCh;
    CANFD_Object       *ptrCanFdObj = NULL;
    CANFD_EdmaChConfig *edmaChCfg   = NULL;

    if(NULL != ptrCanMsgObj)
    {
        if (ptrCanMsgObj->dmaMsgConfig.currentMsgNum == (ptrCanMsgObj->dmaMsgConfig.numMsgs))
        {
            currentDataPtr = (uint8_t *)(ptrCanMsgObj->dmaMsgConfig.data);
            currentDataPtr = currentDataPtr + ((ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg * ptrCanMsgObj->dmaMsgConfig.currentMsgNum));

            ptrCanMsgObj->dmaMsgConfig.currentMsgNum++;

            /* All msgs are transmitted. */
            ptrCanFdObj = ptrCanMsgObj->canfdHandle->object;
            edmaChCfg   = (CANFD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
            baseAddr    = edmaChCfg->edmaBaseAddr;
            regionId    = edmaChCfg->edmaRegionId;
            dmaTxCh     = edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo];

            /* Disable event trigger transfer */
            EDMA_disableTransferRegion(baseAddr,
                                    regionId,
                                    dmaTxCh,
                                    EDMA_TRIG_MODE_EVENT);

            CANFD_dmaTxCompletionCallback(ptrCanMsgObj,
                                        (void *)(currentDataPtr),
                                        CANFD_DMA_TX_COMPLETION_FINAL);
        }
        else if (ptrCanMsgObj->dmaMsgConfig.currentMsgNum < (ptrCanMsgObj->dmaMsgConfig.numMsgs))
        {
            currentDataPtr = (uint8_t *)(ptrCanMsgObj->dmaMsgConfig.data);
            currentDataPtr = currentDataPtr + ((ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg * ptrCanMsgObj->dmaMsgConfig.currentMsgNum));

            ptrCanMsgObj->dmaMsgConfig.currentMsgNum++;
            CANFD_dmaTxCompletionCallback(ptrCanMsgObj,
                                        (void *)(currentDataPtr),
                                        CANFD_DMA_TX_COMPLETION_INTERMEDIATE);
        }
        else
        {
            /* Callback for all msgs are called. This should not be called. */
        }
    }

    return;
}

__attribute__((weak)) void CANFD_dmaTxCompletionCallback(CANFD_MessageObject* ptrCanMsgObj, void *data, uint32_t completionType)
{

}

int32_t CANFD_configureDmaTx(const CANFD_Object *ptrCanFdObj, CANFD_MessageObject* ptrCanMsgObj,
                             uint32_t dataLengthPerMsg, uint32_t numMsgs, const void* data)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaTxCh, tccTx, paramTx;
    EDMACCPaRAMEntry    edmaTxParam;
    CANFD_EdmaChConfig *edmaChCfg = NULL;
    uint32_t            srcAddr, dstAddr;

    if((NULL != ptrCanFdObj) && (NULL != ptrCanMsgObj) &&
       (dataLengthPerMsg != 0U) && (numMsgs != 0U) && (NULL != data))
    {
        edmaChCfg = (CANFD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
        /* Store the current Tx msg. */
        ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg = dataLengthPerMsg;
        /* numMsg should numMsg - 1 because 1st transmission is done by the CANFD IP itself */
        ptrCanMsgObj->dmaMsgConfig.numMsgs          = numMsgs - 1U;
        ptrCanMsgObj->dmaMsgConfig.data             = data;
        ptrCanMsgObj->dmaMsgConfig.currentMsgNum    = 1;

        /* Fetch the EDMA paramters for CANFD transfer */
        baseAddr   = edmaChCfg->edmaBaseAddr;
        regionId   = edmaChCfg->edmaRegionId;
        dmaTxCh    = edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo];
        tccTx      = edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo];
        paramTx    = edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo];

        /* First msg is already copied to msg ram. Program the dma to transfer from second msg. */
        srcAddr = (uint32_t) data + dataLengthPerMsg;
        /* Get the buffer address in message ram. */
        status = MCAN_getWriteMsgElemAddress(ptrCanFdObj->regBaseAddress, MCAN_MEM_TYPE_BUF,  ptrCanMsgObj->txElement, &dstAddr);

        if(status == SystemP_SUCCESS)
        {
            /* Add base address to the offset. */
            dstAddr += ptrCanFdObj->regBaseAddress;
            /* First 8 bytes are for header which will remain common for all msgs.
            Data should be updated from dstAddr + 8. */
            dstAddr += 8U;

            /* Transmit param set configuration */
            EDMA_ccPaRAMEntry_init(&edmaTxParam);
            edmaTxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void*) srcAddr);
            edmaTxParam.destAddr      = (uint32_t) SOC_virtToPhy((uint8_t *) dstAddr);
            edmaTxParam.aCnt          = (uint16_t) dataLengthPerMsg;
            edmaTxParam.bCnt          = (uint16_t) numMsgs;
            edmaTxParam.cCnt          = (uint16_t) 1;
            edmaTxParam.bCntReload    = (uint16_t) edmaTxParam.bCnt;
            edmaTxParam.srcBIdx       = (int16_t)  edmaTxParam.aCnt;
            edmaTxParam.destBIdx      = (int16_t) 0;
            edmaTxParam.srcCIdx       = (int16_t) 0;
            edmaTxParam.destCIdx      = (int16_t) 0;
            edmaTxParam.linkAddr      = 0xFFFFU;
            edmaTxParam.opt           = 0;
            edmaTxParam.opt          |=
                (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | ((tccTx << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

            /* Write Tx param set */
            EDMA_setPaRAM(baseAddr, paramTx, &edmaTxParam);

            /* Set event trigger to start CANFD TX transfer */
            EDMA_enableTransferRegion(baseAddr, regionId, dmaTxCh,
                EDMA_TRIG_MODE_EVENT);
        }
    }
    else
    {
        return SystemP_FAILURE;
    }

    return status;
}

int32_t CANFD_cancelDmaTx(const CANFD_Object *ptrCanFdObj, const CANFD_MessageObject* ptrCanMsgObj)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaTxCh;
    CANFD_EdmaChConfig *edmaChCfg = NULL;

    if((NULL != ptrCanFdObj) && (NULL != ptrCanMsgObj))
    {
        edmaChCfg = (CANFD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

        /* Fetch the EDMA paramters for CANFD transfer */
        baseAddr  = edmaChCfg->edmaBaseAddr;
        regionId  = edmaChCfg->edmaRegionId;
        dmaTxCh   = edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo];

        /* Set event trigger to start CANFD TX transfer */
        EDMA_disableTransferRegion(baseAddr, regionId, dmaTxCh,
                                   EDMA_TRIG_MODE_EVENT);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static void CANFD_edmaIsrTx(Edma_IntrHandle intrHandle, void *args)
{
    uint8_t *currentDataPtr;
    CANFD_MessageObject* ptrCanMsgObj = NULL;
    uint32_t baseAddr, regionId, dmaTxCh;
    CANFD_Object *ptrCanFdObj = NULL;
    CANFD_EdmaChConfig *edmaChCfg = NULL;

    if(NULL != args)
    {
        ptrCanMsgObj   = (CANFD_MessageObject *)(args);
        ptrCanMsgObj->dmaMsgConfig.currentMsgNum++;
        currentDataPtr = (uint8_t *)(ptrCanMsgObj->dmaMsgConfig.data);
        currentDataPtr = currentDataPtr + ((ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg * ptrCanMsgObj->dmaMsgConfig.currentMsgNum));

        if (ptrCanMsgObj->dmaMsgConfig.currentMsgNum == (ptrCanMsgObj->dmaMsgConfig.numMsgs))
        {
            /* All msgs are transmitted. */
            ptrCanFdObj = ptrCanMsgObj->canfdHandle->object;
            edmaChCfg   = (CANFD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

            baseAddr = edmaChCfg->edmaBaseAddr;
            regionId = edmaChCfg->edmaRegionId;
            dmaTxCh  = edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo];

            /* Disable event trigger transfer */
            EDMA_disableTransferRegion(baseAddr, regionId, dmaTxCh, EDMA_TRIG_MODE_EVENT);
            CANFD_dmaTxCompletionCallback(ptrCanMsgObj, (void *)(currentDataPtr), CANFD_DMA_TX_COMPLETION_FINAL);
        }
        else
        {
            CANFD_dmaTxCompletionCallback(ptrCanMsgObj, (void *)(currentDataPtr), CANFD_DMA_TX_COMPLETION_INTERMEDIATE);
        }
    }
}

static void CANFD_edmaIsrRx(Edma_IntrHandle intrHandle, void *args)
{
    uint8_t *currentDataPtr;
    CANFD_MessageObject* ptrCanMsgObj = NULL;

    if(NULL != args)
    {
        ptrCanMsgObj   = (CANFD_MessageObject *)(args);
        ptrCanMsgObj->dmaMsgConfig.currentMsgNum++;
        currentDataPtr = (uint8_t *)(ptrCanMsgObj->dmaMsgConfig.data);
        currentDataPtr = currentDataPtr + ((ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg * ptrCanMsgObj->dmaMsgConfig.currentMsgNum));

        if (ptrCanMsgObj->dmaMsgConfig.currentMsgNum == (ptrCanMsgObj->dmaMsgConfig.numMsgs))
        {
            /* All msgs are transmitted. */
            uint32_t baseAddr, regionId, dmaRxCh;
            CANFD_Object *ptrCanFdObj = ptrCanMsgObj->canfdHandle->object;
            CANFD_EdmaChConfig *edmaChCfg = (CANFD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

            baseAddr  = edmaChCfg->edmaBaseAddr;
            regionId  = edmaChCfg->edmaRegionId;
            dmaRxCh   = edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo];

            /* Disable event trigger transfer */
            EDMA_disableTransferRegion(baseAddr, regionId, dmaRxCh, EDMA_TRIG_MODE_EVENT);
            CANFD_dmaRxCompletionCallback(ptrCanMsgObj, (void *)(currentDataPtr), CANFD_DMA_RX_COMPLETION_FINAL);
        }
        else
        {
            CANFD_dmaRxCompletionCallback(ptrCanMsgObj, (void *)(currentDataPtr), CANFD_DMA_RX_COMPLETION_INTERMEDIATE);
        }
    }
}

__attribute__((weak)) void CANFD_dmaRxCompletionCallback(CANFD_MessageObject* ptrCanMsgObj, void *data, uint32_t completionType)
{

}

int32_t CANFD_createDmaRxMsgObject(const CANFD_Object *ptrCanFdObj, CANFD_MessageObject* ptrCanMsgObj)
{
    int32_t status = SystemP_SUCCESS;
    int32_t chAllocStatus = SystemP_SUCCESS;
    EDMA_Handle canfdEdmaHandle = NULL;
    CANFD_EdmaChConfig *edmaChCfg = NULL;
    bool ret;

    if((NULL != ptrCanFdObj) && (NULL != ptrCanMsgObj))
    {
        canfdEdmaHandle = (EDMA_Handle) ptrCanFdObj->canfdDmaHandle;
        edmaChCfg = (CANFD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

        if((NULL == canfdEdmaHandle) && (NULL != edmaChCfg))
        {
            status = SystemP_FAILURE;
        }
        else
        {
            /* First MCAN_MAX_RX_DMA_BUFFERS are reserved for the dma mode and one of them will be allocated.
                edmaRxChId[] array will have the corresponding dma channel as configured by crossbar. */
            if(ptrCanMsgObj->rxElement < MCAN_MAX_RX_DMA_BUFFERS)
            {
                ptrCanMsgObj->dmaEventNo = ptrCanMsgObj->rxElement;

                /* Allocate EDMA channel for CANFD RX transfer */
                chAllocStatus = EDMA_allocDmaChannel(canfdEdmaHandle, &(edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo]));
                status += chAllocStatus;

                /* Allocate EDMA TCC for CANFD RX transfer */
                edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocTcc(canfdEdmaHandle, &(edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo]));

                /* Allocate a Param ID for CANFD RX transfer */
                edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocParam(canfdEdmaHandle, &(edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo]));
            }
            else
            {
                status = SystemP_FAILURE;
            }

            if(status == SystemP_SUCCESS)
            {
                ret = EDMA_configureChannelRegion(edmaChCfg->edmaBaseAddr, edmaChCfg->edmaRegionId, EDMA_CHANNEL_TYPE_DMA,
                    edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo], edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo],
                    edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo], EDMA_CANFD_RX_EVT_QUEUE_NO);
                if(ret == (bool)TRUE)
                {
                    status = SystemP_SUCCESS;
                }
                else
                {
                    status = SystemP_SUCCESS;
                }

                /* Register RX interrupt */
                edmaChCfg->edmaIntrObjRx[ptrCanMsgObj->dmaEventNo].tccNum = edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo];
                edmaChCfg->edmaIntrObjRx[ptrCanMsgObj->dmaEventNo].cbFxn  = &CANFD_edmaIsrRx;
                edmaChCfg->edmaIntrObjRx[ptrCanMsgObj->dmaEventNo].appData = (void *) ptrCanMsgObj;
                status += EDMA_registerIntr(canfdEdmaHandle, &(edmaChCfg->edmaIntrObjRx[ptrCanMsgObj->dmaEventNo]));
            }

            if (status != SystemP_SUCCESS)
            {
                /* Free all allocated resources of edma */
                if ((chAllocStatus == SystemP_SUCCESS) && (edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY))
                {
                    EDMA_freeDmaChannel(canfdEdmaHandle, &(edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo]));
                }
                if (edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
                {
                    EDMA_freeTcc(canfdEdmaHandle, &(edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo]));
                }
                if (edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
                {
                    EDMA_freeParam(canfdEdmaHandle, &(edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo]));
                }
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t CANFD_deleteDmaRxMsgObject(const CANFD_Object *ptrCanFdObj,
                                   const CANFD_MessageObject* ptrCanMsgObj)
{
    int32_t status = SystemP_SUCCESS;
    EDMA_Handle canfdEdmaHandle;
    CANFD_EdmaChConfig *edmaChCfg;

    if((NULL != ptrCanFdObj) && (NULL != ptrCanMsgObj))
    {
        canfdEdmaHandle = (EDMA_Handle) ptrCanFdObj->canfdDmaHandle;
        edmaChCfg = (CANFD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

        if ((canfdEdmaHandle == NULL) || (edmaChCfg == NULL))
        {
            status = SystemP_FAILURE;
        }
        else
        {
            /* Free all allocated resources of edma */
            if (edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeDmaChannel(canfdEdmaHandle, &(edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo]));
            }
            if (edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeTcc(canfdEdmaHandle, &(edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo]));
                edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
            }
            if (edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeParam(canfdEdmaHandle, &(edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo]));
                edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t CANFD_configureDmaRx(const CANFD_Object *ptrCanFdObj,
                             CANFD_MessageObject* ptrCanMsgObj,
                             uint32_t dataLengthPerMsg,
                             uint32_t numMsgs,
                             const void* data)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh, tccRx, paramRx;
    EDMACCPaRAMEntry    edmaRxParam;
    CANFD_EdmaChConfig *edmaChCfg = NULL;
    uint32_t            srcAddr, dstAddr;

    if((NULL != ptrCanFdObj) && (NULL != ptrCanMsgObj))
    {
        edmaChCfg = (CANFD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
        /* Store the current Rx msg. */
        ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg = dataLengthPerMsg;
        /* numMsg should numMsg - 1 because 1st transmission is done by the CANFD IP itself */
        ptrCanMsgObj->dmaMsgConfig.numMsgs          = numMsgs - 1U;
        ptrCanMsgObj->dmaMsgConfig.data             = data;
        ptrCanMsgObj->dmaMsgConfig.currentMsgNum    = 0;

        if(edmaChCfg == NULL)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            /* Fetch the EDMA paramters for CANFD transfer */
            baseAddr   = edmaChCfg->edmaBaseAddr;
            regionId   = edmaChCfg->edmaRegionId;
            dmaRxCh    = edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo];
            tccRx      = edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo];
            paramRx    = edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo];

            /* Get the buffer address in message ram. */
            status = MCAN_getReadMsgElemAddress(ptrCanFdObj->regBaseAddress,
                                                MCAN_MEM_TYPE_BUF,
                                                ptrCanMsgObj->rxElement,
                                                0,
                                                &srcAddr);
        }

        if(status == SystemP_SUCCESS)
        {
            /* Add base address to the offset. */
            srcAddr += ptrCanFdObj->regBaseAddress;
            /* Program the dma to receive the msg. */
            dstAddr = (uint32_t) data;

            /* Transmit param set configuration */
            EDMA_ccPaRAMEntry_init(&edmaRxParam);
            edmaRxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void*) srcAddr);
            edmaRxParam.destAddr      = (uint32_t) SOC_virtToPhy((uint8_t *) dstAddr);
            edmaRxParam.aCnt          = (uint16_t) dataLengthPerMsg;
            edmaRxParam.bCnt          = (uint16_t) numMsgs;
            edmaRxParam.cCnt          = (uint16_t) 1;
            edmaRxParam.bCntReload    = (uint16_t) edmaRxParam.bCnt;
            edmaRxParam.srcBIdx       = (int16_t) 0;
            edmaRxParam.destBIdx      = (int16_t) edmaRxParam.aCnt;
            edmaRxParam.srcCIdx       = (int16_t) 0;
            edmaRxParam.destCIdx      = (int16_t) 0;
            edmaRxParam.linkAddr      = 0xFFFFU;
            edmaRxParam.opt           = 0;
            edmaRxParam.opt          |=
                (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | ((tccRx << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

            /* Write Rx param set */
            EDMA_setPaRAM(baseAddr, paramRx, &edmaRxParam);

            /* Set event trigger to start CANFD Rx transfer */
            EDMA_enableTransferRegion(baseAddr, regionId, dmaRxCh,
                EDMA_TRIG_MODE_EVENT);
        }
    }

    return status;
}

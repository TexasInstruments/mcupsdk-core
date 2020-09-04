/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  \file mcasp.c
 *
 *  \brief File containing MCASP Driver APIs implementation.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <kernel/dpl/DebugP.h>
#include <drivers/mcasp.h>
#include <drivers/mcasp/v0/mcasp_priv.h>
#include <drivers/edma.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Event queue to be used  */
#define MCASP_EDMA_EVT_QUEUE_NO      (0U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None. */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void MCASP_edmaIsrFxnTx(Edma_IntrHandle intrHandle, void *args);
static void MCASP_edmaIsrFxnRx(Edma_IntrHandle intrHandle, void *args);
static void MCASP_edmaLoadPramRx(EDMACCPaRAMEntry *edmaParam,
                            MCASP_Config *config, MCASP_Transaction *txn);
static void MCASP_edmaLoadPramTx(EDMACCPaRAMEntry *edmaParam,
                            MCASP_Config *config, MCASP_Transaction *txn);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None. */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void MCASP_openDma(MCASP_Config *config, uint32_t edmaInst)
{

    int32_t             status = SystemP_SUCCESS;
    EDMA_Handle         mcaspEdmaHandle = EDMA_getHandle(edmaInst);
    uint32_t            isEdmaInterruptEnabled;
    MCASP_Object        *object = config->object;
    const MCASP_Attrs   *attrs = config->attrs;

    DebugP_assert (mcaspEdmaHandle != NULL);

    object->xmtDmaObj.baseAddr = EDMA_getBaseAddr(mcaspEdmaHandle);
    DebugP_assert(object->xmtDmaObj.baseAddr != 0);

    object->xmtDmaObj.regionId = EDMA_getRegionId(mcaspEdmaHandle);
    DebugP_assert(object->xmtDmaObj.regionId < SOC_EDMA_NUM_REGIONS);

    object->xmtDmaObj.chId = attrs->edmaChTx;
    status = EDMA_allocDmaChannel(mcaspEdmaHandle, &(object->xmtDmaObj.chId));
    DebugP_assert(status == SystemP_SUCCESS);

    object->xmtDmaObj.tccId = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(mcaspEdmaHandle, &(object->xmtDmaObj.tccId));
    DebugP_assert(status == SystemP_SUCCESS);

    object->xmtDmaObj.paramId = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(mcaspEdmaHandle, &(object->xmtDmaObj.paramId));
    DebugP_assert(status == SystemP_SUCCESS);

    EDMA_configureChannelRegion(object->xmtDmaObj.baseAddr, object->xmtDmaObj.regionId, EDMA_CHANNEL_TYPE_DMA,
        object->xmtDmaObj.chId, object->xmtDmaObj.tccId, object->xmtDmaObj.paramId, MCASP_EDMA_EVT_QUEUE_NO);

    object->xmtDmaObj.linkParamId = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(mcaspEdmaHandle, &(object->xmtDmaObj.linkParamId));
    DebugP_assert(status == SystemP_SUCCESS);

    object->rcvDmaObj.baseAddr = EDMA_getBaseAddr(mcaspEdmaHandle);
    DebugP_assert(object->rcvDmaObj.baseAddr != 0);

    object->rcvDmaObj.regionId = EDMA_getRegionId(mcaspEdmaHandle);
    DebugP_assert(object->rcvDmaObj.regionId < SOC_EDMA_NUM_REGIONS);

    object->rcvDmaObj.chId = attrs->edmaChRx;
    status = EDMA_allocDmaChannel(mcaspEdmaHandle, &(object->rcvDmaObj.chId));
    DebugP_assert(status == SystemP_SUCCESS);

    object->rcvDmaObj.tccId = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(mcaspEdmaHandle, &(object->rcvDmaObj.tccId));
    DebugP_assert(status == SystemP_SUCCESS);

    object->rcvDmaObj.paramId = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(mcaspEdmaHandle, &(object->rcvDmaObj.paramId));
    DebugP_assert(status == SystemP_SUCCESS);

    EDMA_configureChannelRegion(object->rcvDmaObj.baseAddr, object->rcvDmaObj.regionId, EDMA_CHANNEL_TYPE_DMA,
        object->rcvDmaObj.chId, object->rcvDmaObj.tccId, object->rcvDmaObj.paramId, MCASP_EDMA_EVT_QUEUE_NO);

    object->rcvDmaObj.linkParamId = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(mcaspEdmaHandle, &(object->rcvDmaObj.linkParamId));
    DebugP_assert(status == SystemP_SUCCESS);

    /* Check if interrupt is enabled */
    isEdmaInterruptEnabled = EDMA_isInterruptEnabled(mcaspEdmaHandle);
    DebugP_assert(isEdmaInterruptEnabled == TRUE);

    /* Register interrupt */
    object->xmtDmaObj.intrObj.tccNum = object->xmtDmaObj.tccId;
    object->xmtDmaObj.intrObj.cbFxn  = &MCASP_edmaIsrFxnTx;
    object->xmtDmaObj.intrObj.appData = (void *) config;
    status = EDMA_registerIntr(mcaspEdmaHandle, &(object->xmtDmaObj.intrObj));
    DebugP_assert(status == SystemP_SUCCESS);

    object->rcvDmaObj.intrObj.tccNum = object->rcvDmaObj.tccId;
    object->rcvDmaObj.intrObj.cbFxn  = &MCASP_edmaIsrFxnRx;
    object->rcvDmaObj.intrObj.appData = (void *) config;
    status = EDMA_registerIntr(mcaspEdmaHandle, &(object->rcvDmaObj.intrObj));
    DebugP_assert(status == SystemP_SUCCESS);
}

void MCASP_closeDma(MCASP_Config *config, uint32_t edmaInst)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Handle         mcaspEdmaHandle = EDMA_getHandle(edmaInst);
    uint32_t            isEdmaInterruptEnabled;
    MCASP_Object        *object = config->object;

    DebugP_assert (mcaspEdmaHandle != NULL);

    EDMA_freeChannelRegion(object->xmtDmaObj.baseAddr, object->xmtDmaObj.regionId, EDMA_CHANNEL_TYPE_DMA,
        object->xmtDmaObj.chId, EDMA_TRIG_MODE_EVENT, object->xmtDmaObj.tccId, MCASP_EDMA_EVT_QUEUE_NO);

    status = EDMA_freeDmaChannel(mcaspEdmaHandle, &(object->xmtDmaObj.chId));
    DebugP_assert(status == SystemP_SUCCESS);

    status = EDMA_freeTcc(mcaspEdmaHandle, &(object->xmtDmaObj.tccId));
    DebugP_assert(status == SystemP_SUCCESS);

    status = EDMA_freeParam(mcaspEdmaHandle, &(object->xmtDmaObj.paramId));
    DebugP_assert(status == SystemP_SUCCESS);

    status = EDMA_freeParam(mcaspEdmaHandle, &(object->xmtDmaObj.linkParamId));
    DebugP_assert(status == SystemP_SUCCESS);

    EDMA_freeChannelRegion(object->rcvDmaObj.baseAddr, object->rcvDmaObj.regionId, EDMA_CHANNEL_TYPE_DMA,
        object->rcvDmaObj.chId, EDMA_TRIG_MODE_EVENT, object->rcvDmaObj.tccId, MCASP_EDMA_EVT_QUEUE_NO);

    status = EDMA_freeDmaChannel(mcaspEdmaHandle, &(object->rcvDmaObj.chId));
    DebugP_assert(status == SystemP_SUCCESS);

    status = EDMA_freeTcc(mcaspEdmaHandle, &(object->rcvDmaObj.tccId));
    DebugP_assert(status == SystemP_SUCCESS);

    status = EDMA_freeParam(mcaspEdmaHandle, &(object->rcvDmaObj.paramId));
    DebugP_assert(status == SystemP_SUCCESS);

    status = EDMA_freeParam(mcaspEdmaHandle, &(object->rcvDmaObj.linkParamId));
    DebugP_assert(status == SystemP_SUCCESS);

    /* Check if interrupt is enabled */
    isEdmaInterruptEnabled = EDMA_isInterruptEnabled(mcaspEdmaHandle);
    DebugP_assert(isEdmaInterruptEnabled == TRUE);

    /* Register interrupt */
    status = EDMA_unregisterIntr(mcaspEdmaHandle, &(object->xmtDmaObj.intrObj));
    DebugP_assert(status == SystemP_SUCCESS);

    status = EDMA_unregisterIntr(mcaspEdmaHandle, &(object->rcvDmaObj.intrObj));
    DebugP_assert(status == SystemP_SUCCESS);
}

static void MCASP_edmaIsrFxnTx(Edma_IntrHandle intrHandle, void *args)
{
    EDMACCPaRAMEntry   edmaParam;
    MCASP_Object *object = ((MCASP_Config *)args)->object;
    MCASP_TransferObj *xfrObj = &(object->XmtObj);

    MCASP_Transaction *txn = QueueP_get(object->curentQueueHandleTx);
    /* get next buffer and program. */
    MCASP_Transaction *nextTxn = QueueP_get(object->reqQueueHandleTx);

    if (object->XmtObj.loopjobEnable == true)
    {
        if ((txn != object->curentQueueHandleTx) &&
            (txn != &(object->XmtObj.txnLoopjob)))
        {
            /* This transaction is not from loopjob. */
            txn->status = SystemP_SUCCESS;
            xfrObj->cbFxn((MCASP_Handle *)args, txn);
        }
        if (nextTxn == object->reqQueueHandleTx)
        {
            /* No buffers are queued. */
            nextTxn = &object->XmtObj.txnLoopjob;
        }
    }
    else
    {
        if (nextTxn == object->reqQueueHandleTx)
        {
            /* No buffers are queued. program the same buffer. */
            if (txn != object->curentQueueHandleTx)
            {
                nextTxn = txn;
            }
            else
            {
                nextTxn = NULL;
            }
        }
        else
        {
            if (txn != object->curentQueueHandleTx)
            {
                /* Give callback for current txn */
                txn->status = SystemP_SUCCESS;
                xfrObj->cbFxn((MCASP_Handle *)args, txn);
            }
        }
    }
    if (nextTxn != NULL)
    {
        /* Program Param Set */
        EDMA_getPaRAM(object->xmtDmaObj.baseAddr, object->xmtDmaObj.linkParamId, &edmaParam);
        edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy(nextTxn->buf);
        edmaParam.cCnt          = (uint16_t) ((nextTxn->count > edmaParam.bCnt) ? (nextTxn->count / edmaParam.bCnt) : 1);
        EDMA_setPaRAM(object->xmtDmaObj.baseAddr, object->xmtDmaObj.linkParamId, &edmaParam);

        QueueP_put(object->curentQueueHandleTx, nextTxn);
    }
}

static void MCASP_edmaIsrFxnRx(Edma_IntrHandle intrHandle, void *args)
{
    EDMACCPaRAMEntry   edmaParam;
    MCASP_Object *object = ((MCASP_Config *)args)->object;
    MCASP_TransferObj *xfrObj = &(object->RcvObj);

    MCASP_Transaction *txn = QueueP_get(object->curentQueueHandleRx);
    /* get next buffer and program. */
    MCASP_Transaction *nextTxn = QueueP_get(object->reqQueueHandleRx);

    if (object->RcvObj.loopjobEnable == true)
    {
        if ((txn != object->curentQueueHandleRx) &&
            (txn != &(object->RcvObj.txnLoopjob)))
        {
            /* This transaction is not from loopjob. */
            txn->status = SystemP_SUCCESS;
            xfrObj->cbFxn((MCASP_Handle *)args, txn);
        }
        if (nextTxn == object->reqQueueHandleRx)
        {
            /* No buffers are queued. */
            nextTxn = &object->RcvObj.txnLoopjob;
        }
    }
    else
    {
        if (nextTxn == object->reqQueueHandleRx)
        {
            /* No buffers are queued. program the same buffer. */
            if (txn != object->curentQueueHandleRx)
            {
                nextTxn = txn;
            }
            else
            {
                nextTxn = NULL;
            }
        }
        else
        {
            if (txn != object->curentQueueHandleRx)
            {
                /* Give callback for current txn */
                txn->status = SystemP_SUCCESS;
                xfrObj->cbFxn((MCASP_Handle *)args, txn);
            }
        }
    }
    if (nextTxn != NULL)
    {
        /* Program Param Set */
        EDMA_getPaRAM(object->rcvDmaObj.baseAddr, object->rcvDmaObj.linkParamId, &edmaParam);
        edmaParam.destAddr      = (uint32_t) SOC_virtToPhy(nextTxn->buf);
        edmaParam.cCnt          = (uint16_t) ((nextTxn->count > edmaParam.bCnt) ? (nextTxn->count / edmaParam.bCnt) : 1);
        EDMA_setPaRAM(object->rcvDmaObj.baseAddr, object->rcvDmaObj.linkParamId, &edmaParam);

        QueueP_put(object->curentQueueHandleRx, nextTxn);
    }
}

int32_t MCASP_enableDmaTx(MCASP_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    EDMACCPaRAMEntry   edmaParam;
    MCASP_Object *object = config->object;
    MCASP_Transaction *txn = QueueP_get(object->reqQueueHandleTx);

    if (txn == object->reqQueueHandleTx)
    {
        /* No buffers are queued. */
        if (object->XmtObj.loopjobEnable == true)
        {
            txn = &object->XmtObj.txnLoopjob;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* Program Param Set */
        EDMA_ccPaRAMEntry_init(&edmaParam);
        MCASP_edmaLoadPramTx(&edmaParam, config, txn);
        EDMA_setPaRAM(object->xmtDmaObj.baseAddr, object->xmtDmaObj.paramId, &edmaParam);
        QueueP_put(object->curentQueueHandleTx, txn);
    }

    if (status == SystemP_SUCCESS)
    {
        /* get the second txn and program the link param */
        txn = QueueP_get(object->reqQueueHandleTx);
        if (txn == object->reqQueueHandleTx)
        {
            if (object->XmtObj.loopjobEnable == true)
            {
                txn = &object->XmtObj.txnLoopjob;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }
    if (status == SystemP_SUCCESS)
    {
        MCASP_edmaLoadPramTx(&edmaParam, config, txn);
        EDMA_setPaRAM(object->xmtDmaObj.baseAddr, object->xmtDmaObj.linkParamId, &edmaParam);
        QueueP_put(object->curentQueueHandleTx, txn);

        /* Link the 2 param ids */
        EDMA_linkChannel(object->xmtDmaObj.baseAddr, object->xmtDmaObj.paramId, object->xmtDmaObj.linkParamId);
        EDMA_linkChannel(object->xmtDmaObj.baseAddr, object->xmtDmaObj.linkParamId, object->xmtDmaObj.linkParamId);

        /* Enable the event triggered transfer on edma channel. */
        EDMA_enableTransferRegion(object->xmtDmaObj.baseAddr, object->xmtDmaObj.regionId, object->xmtDmaObj.chId, EDMA_TRIG_MODE_EVENT);
    }
    return status;
}

int32_t MCASP_enableDmaRx(MCASP_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    EDMACCPaRAMEntry   edmaParam;
    MCASP_Object *object = config->object;
    MCASP_Transaction *txn = QueueP_get(object->reqQueueHandleRx);

    if (txn == object->reqQueueHandleRx)
    {
        /* No buffers are queued. */
        if (object->RcvObj.loopjobEnable == true)
        {
            txn = &object->RcvObj.txnLoopjob;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* Program Param Set */
        EDMA_ccPaRAMEntry_init(&edmaParam);
        MCASP_edmaLoadPramRx(&edmaParam, config, txn);
        EDMA_setPaRAM(object->rcvDmaObj.baseAddr, object->rcvDmaObj.paramId, &edmaParam);
        QueueP_put(object->curentQueueHandleRx, txn);
    }

    if (status == SystemP_SUCCESS)
    {
        /* get the second txn and program the link param */
        txn = QueueP_get(object->reqQueueHandleRx);
        if (txn == object->reqQueueHandleRx)
        {
            if (object->RcvObj.loopjobEnable == true)
            {
                txn = &object->RcvObj.txnLoopjob;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }
    if (status == SystemP_SUCCESS)
    {
        MCASP_edmaLoadPramRx(&edmaParam, config, txn);
        EDMA_setPaRAM(object->rcvDmaObj.baseAddr, object->rcvDmaObj.linkParamId, &edmaParam);
        QueueP_put(object->curentQueueHandleRx, txn);

        /* Link the 2 param ids */
        EDMA_linkChannel(object->rcvDmaObj.baseAddr, object->rcvDmaObj.paramId, object->rcvDmaObj.linkParamId);
        EDMA_linkChannel(object->rcvDmaObj.baseAddr, object->rcvDmaObj.linkParamId, object->rcvDmaObj.linkParamId);

        /* Enable the event triggered transfer on edma channel. */
        EDMA_enableTransferRegion(object->rcvDmaObj.baseAddr, object->rcvDmaObj.regionId, object->rcvDmaObj.chId , EDMA_TRIG_MODE_EVENT);
    }
    return status;
}

void MCASP_disableDmaTx(MCASP_Config *config)
{
    MCASP_Object *object = config->object;
    /* End transfer */
    EDMA_disableTransferRegion(object->xmtDmaObj.baseAddr,
        object->xmtDmaObj.regionId,
        object->xmtDmaObj.chId,
        EDMA_TRIG_MODE_EVENT);
}

void MCASP_disableDmaRx(MCASP_Config *config)
{
    MCASP_Object *object = config->object;
    /* End transfer */
    EDMA_disableTransferRegion(object->rcvDmaObj.baseAddr,
        object->rcvDmaObj.regionId,
        object->rcvDmaObj.chId,
        EDMA_TRIG_MODE_EVENT);
}

static void MCASP_edmaLoadPramTx(EDMACCPaRAMEntry *edmaParam, MCASP_Config *config, MCASP_Transaction *txn)
{
    MCASP_Object *object = config->object;
    const MCASP_Attrs   *attrs = config->attrs;
    MCASP_TransferObj *xfrObj = &(object->XmtObj);
    uint32_t frameCount = txn->count / (xfrObj->slotCount * xfrObj->serCount);

    edmaParam->srcAddr       = (uint32_t) SOC_virtToPhy(txn->buf);
    edmaParam->destAddr      = (uint32_t) SOC_virtToPhy((void *)attrs->dataBaseAddr);

    switch(xfrObj->bufferFormat)
    {
        case MCASP_AUDBUFF_FORMAT_1SER_MULTISLOT_INTERLEAVED:
        case MCASP_AUDBUFF_FORMAT_MULTISER_MULTISLOT_SEMI_INTERLEAVED_1:
        {
            edmaParam->aCnt          = (uint16_t) ((attrs->txSlotSize / 8) + (((attrs->txSlotSize % 8) == 0) ? 0 : 1));
            edmaParam->bCnt          = (uint16_t) 32;
            edmaParam->cCnt          = (uint16_t) ((txn->count > edmaParam->bCnt) ? (txn->count / edmaParam->bCnt) : 1);
            edmaParam->bCntReload    = (uint16_t) edmaParam->bCnt;
            edmaParam->srcBIdx       = (int16_t) EDMA_PARAM_BIDX(edmaParam->aCnt);
            edmaParam->destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
            edmaParam->srcCIdx       = (int16_t) edmaParam->aCnt * edmaParam->bCnt;
            edmaParam->destCIdx      = (int16_t) 0;
            edmaParam->srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(edmaParam->aCnt);
            edmaParam->destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
            break;
        }
        case MCASP_AUDBUFF_FORMAT_1SER_MULTISLOT_NON_INTERLEAVED:
        {
            edmaParam->aCnt          = (uint16_t) ((attrs->txSlotSize / 8) + (((attrs->txSlotSize % 8) == 0) ? 0 : 1));
            edmaParam->bCnt          = (uint16_t) xfrObj->slotCount;
            edmaParam->cCnt          = (uint16_t) ((txn->count > edmaParam->bCnt) ? (txn->count / edmaParam->bCnt) : 1);
            edmaParam->bCntReload    = (uint16_t) edmaParam->bCnt;
            edmaParam->srcBIdx       = (int16_t) EDMA_PARAM_BIDX(frameCount * edmaParam->aCnt);
            edmaParam->destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
            edmaParam->srcCIdx       = (int16_t) edmaParam->aCnt;
            edmaParam->destCIdx      = (int16_t) 0;
            edmaParam->srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(frameCount * edmaParam->aCnt);
            edmaParam->destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
            break;
        }
        case MCASP_AUDBUFF_FORMAT_MULTISER_MULTISLOT_SEMI_INTERLEAVED_2:
        {
            edmaParam->aCnt          = (uint16_t) ((attrs->txSlotSize / 8) + (((attrs->txSlotSize % 8) == 0) ? 0 : 1));
            edmaParam->bCnt          = (uint16_t) xfrObj->serCount;
            edmaParam->cCnt          = (uint16_t) ((txn->count > edmaParam->bCnt) ? (txn->count / edmaParam->bCnt) : 1);
            edmaParam->bCntReload    = (uint16_t) edmaParam->bCnt;
            edmaParam->srcBIdx       = (int16_t) EDMA_PARAM_BIDX(frameCount * xfrObj->slotCount * edmaParam->aCnt);
            edmaParam->destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
            edmaParam->srcCIdx       = (int16_t) edmaParam->aCnt;
            edmaParam->destCIdx      = (int16_t) 0;
            edmaParam->srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(frameCount * xfrObj->slotCount * edmaParam->aCnt);
            edmaParam->destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
            break;
        }
        default:
        {
            DebugP_assert(0);
        }
    }

    edmaParam->linkAddr      = 0xFFFFU;
    edmaParam->opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
         ((((uint32_t)object->xmtDmaObj.tccId) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
}

static void MCASP_edmaLoadPramRx(EDMACCPaRAMEntry *edmaParam, MCASP_Config *config, MCASP_Transaction *txn)
{
    MCASP_Object *object = config->object;
    const MCASP_Attrs   *attrs = config->attrs;
    MCASP_TransferObj *xfrObj = &(object->RcvObj);
    uint32_t frameCount = txn->count / (xfrObj->slotCount * xfrObj->serCount);

    edmaParam->srcAddr       = (uint32_t) SOC_virtToPhy((void *)attrs->dataBaseAddr);
    edmaParam->destAddr      = (uint32_t) SOC_virtToPhy(txn->buf);

    switch(xfrObj->bufferFormat)
    {
        case MCASP_AUDBUFF_FORMAT_1SER_MULTISLOT_INTERLEAVED:
        case MCASP_AUDBUFF_FORMAT_MULTISER_MULTISLOT_SEMI_INTERLEAVED_1:
        {
            edmaParam->aCnt          = (uint16_t) ((attrs->txSlotSize / 8) + (((attrs->txSlotSize % 8) == 0) ? 0 : 1));
            edmaParam->bCnt          = (uint16_t) 32;
            edmaParam->cCnt          = (uint16_t) ((txn->count > edmaParam->bCnt) ? (txn->count / edmaParam->bCnt) : 1);
            edmaParam->bCntReload    = (uint16_t) edmaParam->bCnt;
            edmaParam->srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
            edmaParam->destBIdx      = (int16_t) EDMA_PARAM_BIDX(edmaParam->aCnt);
            edmaParam->srcCIdx       = (int16_t) 0;
            edmaParam->destCIdx      = (int16_t) edmaParam->aCnt * edmaParam->bCnt;
            edmaParam->srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
            edmaParam->destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(edmaParam->aCnt);
            break;
        }
        case MCASP_AUDBUFF_FORMAT_1SER_MULTISLOT_NON_INTERLEAVED:
        {
            edmaParam->aCnt          = (uint16_t) ((attrs->txSlotSize / 8) + (((attrs->txSlotSize % 8) == 0) ? 0 : 1));
            edmaParam->bCnt          = (uint16_t) xfrObj->slotCount;
            edmaParam->cCnt          = (uint16_t) ((txn->count > edmaParam->bCnt) ? (txn->count / edmaParam->bCnt) : 1);
            edmaParam->bCntReload    = (uint16_t) edmaParam->bCnt;
            edmaParam->srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
            edmaParam->destBIdx      = (int16_t) EDMA_PARAM_BIDX(frameCount * edmaParam->aCnt);
            edmaParam->srcCIdx       = (int16_t) 0;
            edmaParam->destCIdx      = (int16_t) edmaParam->aCnt;
            edmaParam->srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
            edmaParam->destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(frameCount * edmaParam->aCnt);
            break;
        }
        case MCASP_AUDBUFF_FORMAT_MULTISER_MULTISLOT_SEMI_INTERLEAVED_2:
        {
            edmaParam->aCnt          = (uint16_t) ((attrs->txSlotSize / 8) + (((attrs->txSlotSize % 8) == 0) ? 0 : 1));
            edmaParam->bCnt          = (uint16_t) xfrObj->serCount;
            edmaParam->cCnt          = (uint16_t) ((txn->count > edmaParam->bCnt) ? (txn->count / edmaParam->bCnt) : 1);
            edmaParam->bCntReload    = (uint16_t) edmaParam->bCnt;
            edmaParam->srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
            edmaParam->destBIdx      = (int16_t) EDMA_PARAM_BIDX(frameCount * xfrObj->slotCount * edmaParam->aCnt);
            edmaParam->srcCIdx       = (int16_t) 0;
            edmaParam->destCIdx      = (int16_t) edmaParam->aCnt;
            edmaParam->srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
            edmaParam->destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(frameCount * xfrObj->slotCount * edmaParam->aCnt);
            break;
        }
        default:
        {
            DebugP_assert(0);
        }
    }

    edmaParam->linkAddr      = 0xFFFFU;
    edmaParam->opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
         ((((uint32_t)object->rcvDmaObj.tccId) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
}

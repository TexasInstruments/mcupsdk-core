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
 *  \file fsi_dma_edma.c
 *
 *  \brief File containing EDMA Driver APIs implementation for FSI driver.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <drivers/soc.h>
#include <drivers/fsi/v1/dma/edma/fsi_dma_edma.h>
#include <drivers/fsi/v1/fsi_tx_hld.h>
#include <drivers/fsi/v1/fsi_rx_hld.h>
#include <kernel/dpl/CacheP.h>
#include "fsi_dma_edma.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** \brief Transmit EDMA channel event queue number                           */
#define EDMA_FSI_TX_EVT_QUEUE_NO                  (0U)
/** \brief Receive EDMA channel event queue number                            */
#define EDMA_FSI_RX_EVT_QUEUE_NO                  (1U)

/* Event queue to be used for EDMA transfer */
#define EDMA_TEST_EVT_QUEUE_NO          (0U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void fsi_edmaIsrTx(Edma_IntrHandle intrHandle, void *args);
static void fsi_edmaIsrRx(Edma_IntrHandle intrHandle, void *args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t FSI_Tx_dmaOpen(FSI_Tx_Handle fsiTxHandle, FSI_Tx_DmaChConfig dmaChCfg)
{
    int32_t             status = SystemP_FAILURE;
    FSI_Tx_EdmaChConfig *edmaChCfg = NULL;
    uint32_t            baseAddr, regionId;
    uint32_t            isEdmaInterruptEnabled;
    EDMA_Handle         fsiTxEdmaHandle;
    FSI_Tx_Object       *object;
    FSI_Tx_Config *config = NULL;

    if((NULL != fsiTxHandle) && (NULL != dmaChCfg))
    {
        edmaChCfg = (FSI_Tx_EdmaChConfig *)dmaChCfg;
        config =    (FSI_Tx_Config *)fsiTxHandle;
        object =    config->object;

        fsiTxEdmaHandle = (EDMA_Handle) object->fsiTxDmaHandle;

        if(fsiTxEdmaHandle != NULL)
        {
            /* Read base address of allocated EDMA instance */
            baseAddr = EDMA_getBaseAddr(fsiTxEdmaHandle);

            /* Read the region ID of the EDMA instance */
            regionId = EDMA_getRegionId(fsiTxEdmaHandle);

            /* Store the EDMA parameters */
            edmaChCfg->edmaBaseAddr = baseAddr;
            edmaChCfg->edmaRegionId = regionId;

            /* Check if interrupt is enabled */
            isEdmaInterruptEnabled = EDMA_isInterruptEnabled(fsiTxEdmaHandle);

            if((baseAddr != (uint32_t)0U) && (regionId < (uint32_t)SOC_EDMA_NUM_REGIONS) && (isEdmaInterruptEnabled == (uint32_t)TRUE))
            {
                /* Validate the EDMA parameters for FsiTx */
                edmaChCfg->isOpen = (uint32_t)TRUE;
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

int32_t FSI_Tx_edmaIntrInit(FSI_Tx_Object *FsiTxObj, uint32_t tccAlloc)
{
    int32_t     status;
    EDMA_Handle fsiTxEdmaHandle = NULL;
    FSI_Tx_EdmaChConfig *edmaChCfg = NULL;

    fsiTxEdmaHandle = (EDMA_Handle) FsiTxObj->fsiTxDmaHandle;
    edmaChCfg       = (FSI_Tx_EdmaChConfig *)FsiTxObj->fsiTxDmaChCfg;

    /* Register TX DMA Completion interrupt */
    edmaChCfg->edmaIntrObjTx.tccNum = tccAlloc;
    edmaChCfg->edmaIntrObjTx.cbFxn  = &fsi_edmaIsrTx;
    edmaChCfg->edmaIntrObjTx.appData = (void *) FsiTxObj;
    status = EDMA_registerIntr(fsiTxEdmaHandle, &edmaChCfg->edmaIntrObjTx);

    return status;
}

int32_t FSI_Tx_edmaChInit(const FSI_Tx_Object *FsiTxObj, uint32_t edmaEventNo,
                         uint32_t *edmaTxParam, uint32_t *edmaTccTx)
{
    int32_t status = SystemP_SUCCESS, chAllocStatus;
    EDMA_Handle fsiTxEdmaHandle = NULL;
    FSI_Tx_EdmaChConfig *edmaChCfg = NULL;

    if(NULL != FsiTxObj)
    {
        fsiTxEdmaHandle = (EDMA_Handle) FsiTxObj->fsiTxDmaHandle;
        edmaChCfg       = (FSI_Tx_EdmaChConfig *)FsiTxObj->fsiTxDmaChCfg;

        if (status == SystemP_SUCCESS)
        {
            /* Allocate EDMA channel for FSI TX transfer */
            chAllocStatus = EDMA_allocDmaChannel(fsiTxEdmaHandle, &edmaChCfg->edmaTxChId[edmaEventNo]);
            status += chAllocStatus;

            /* Allocate a Param ID for FSI TX transfer */
            status += EDMA_allocParam(fsiTxEdmaHandle, edmaTxParam);

            if(edmaTccTx != NULL)
            {
                /* Allocate EDMA TCC for FSI TX transfer */
                status += EDMA_allocTcc(fsiTxEdmaHandle, edmaTccTx);
            }

            if (status != SystemP_SUCCESS)
            {
                /* Free all allocated resources of edma */
                if ((chAllocStatus == SystemP_SUCCESS) && (edmaChCfg->edmaTxChId[edmaEventNo] != EDMA_RESOURCE_ALLOC_ANY))
                {
                    status += EDMA_freeDmaChannel(fsiTxEdmaHandle, &(edmaChCfg->edmaTxChId[edmaEventNo]));
                }
                if(edmaTccTx != NULL)
                {
                    if (*edmaTccTx != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        status += EDMA_freeTcc(fsiTxEdmaHandle, edmaTccTx);
                    }
                }
                if (*edmaTxParam != EDMA_RESOURCE_ALLOC_ANY)
                {
                    status += EDMA_freeParam(fsiTxEdmaHandle, edmaTxParam);
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

int32_t FSI_Tx_configureDma(const FSI_Tx_Object *fsiTxObj, uint32_t *dmaCh,
                void *src, void *dst, uint32_t *tcc, uint32_t *param, uint32_t regionId,
                uint32_t aCnt, uint32_t bCnt, uint32_t cCnt, uint32_t srcBIdx, uint32_t destBIdx,
                uint32_t srcCIdx, uint32_t destCIdx, uint32_t triggerMode)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr;
    EDMACCPaRAMEntry    edmaTxParam;
    FSI_Tx_EdmaChConfig *edmaChCfg = NULL;

    if(NULL != fsiTxObj)
    {
        edmaChCfg = (FSI_Tx_EdmaChConfig *)fsiTxObj->fsiTxDmaChCfg;

        /* Fetch the EDMA paramters for FSI-Tx transfer */
        baseAddr   = edmaChCfg->edmaBaseAddr;
        regionId   = edmaChCfg->edmaRegionId;

        if(tcc != NULL)
        {
            /* Request channel */
            EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            *dmaCh, *tcc, *param, EDMA_TEST_EVT_QUEUE_NO);
        }
        else
        {
            /* Request channel */
            EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            *dmaCh, 0, *param, EDMA_TEST_EVT_QUEUE_NO);
        }
        /* EDMA Params Description
        * tcc  - The tcc number on which the completion interrupt is generated
        * aCnt - Number of bytes in each word
        * bCnt - Number of words in each frame
        * cCnt - Number of frames
        * srcBIdx  - Index between consecutive words of a Source Buffer
        * destBIdx - Index between consecutive words of a Destination Buffer
        * srcCIdx  - Index between consecutive words of a Source Block
        * destCIdx - Index between consecutive words of a Destination Block
        */
        /* Transmit param set configuration */
        EDMA_ccPaRAMEntry_init(&edmaTxParam);
        edmaTxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)src);
        edmaTxParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)dst);
        edmaTxParam.aCnt          = (uint16_t) aCnt;
        edmaTxParam.bCnt          = (uint16_t) bCnt;
        edmaTxParam.cCnt          = (uint16_t) cCnt;
        edmaTxParam.bCntReload    = bCnt;
        edmaTxParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(srcBIdx);
        edmaTxParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(destBIdx);
        edmaTxParam.srcCIdx       = (int16_t) srcCIdx;
        edmaTxParam.destCIdx      = (int16_t) destCIdx;
        edmaTxParam.linkAddr      = 0xFFFFU;
        edmaTxParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(srcBIdx);
        edmaTxParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(destBIdx);
        /* Transfer Completion Interrupt is enabled only after Frame Tag And User
        * data is transmitted/received */
        if (tcc == NULL)
        {
            edmaTxParam.opt       = EDMA_OPT_SYNCDIM_MASK;
        }
        else
        {
            edmaTxParam.opt       = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
                                ((((uint32_t)*tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
        }

        EDMA_setPaRAM(baseAddr, *param, &edmaTxParam);
        EDMA_enableTransferRegion(baseAddr, regionId, *dmaCh,
                                triggerMode);
    }

    return status;

}

int32_t FSI_Tx_edmaChDeInit(const FSI_Tx_Object *fsiTxObj, uint32_t edmaEventNo)
{
    int32_t status;
    EDMA_Handle fsiTxEdmaHandle = NULL;
    FSI_Tx_EdmaChConfig *edmaChCfg = NULL;

    fsiTxEdmaHandle = (EDMA_Handle) fsiTxObj->fsiTxDmaHandle;
    edmaChCfg = (FSI_Tx_EdmaChConfig *)fsiTxObj->fsiTxDmaChCfg;

    //AD_review:
    //SemaphoreP_destruct(&gFsiDmaTxSemObject);
    status  = EDMA_unregisterIntr(fsiTxEdmaHandle, &(edmaChCfg->edmaIntrObjTx));
    return status;
}

static void fsi_edmaIsrTx(Edma_IntrHandle intrHandle, void *args)
{
    FSI_Tx_DmaCompletionCallback(args);
}

int32_t FSI_Tx_dmaClose(FSI_Tx_Handle fsiTxHandle, FSI_Tx_DmaChConfig dmaChCfg)
{
    int32_t             status = SystemP_FAILURE;
    FSI_Tx_EdmaChConfig *edmaChCfg = NULL;
    uint32_t            edmaBaseAddr, regionId, txBaseAddr;
    uint32_t            dmaCh0, dmaCh1, param0, param1, tccTx;
    EDMA_Handle         fsiTxEdmaHandle;
    FSI_Tx_Object       *object;
    FSI_Tx_Config *config = NULL;
    const FSI_Tx_Attrs *attrs;

    if((NULL != fsiTxHandle) && (NULL != dmaChCfg))
    {
        edmaChCfg = (FSI_Tx_EdmaChConfig *)dmaChCfg;
        config = (FSI_Tx_Config*)fsiTxHandle;
        object = config->object;
        attrs = config->attrs;
        txBaseAddr = attrs->baseAddr;

        fsiTxEdmaHandle = (EDMA_Handle) object->fsiTxDmaHandle;

        if(fsiTxEdmaHandle != NULL)
        {
            edmaBaseAddr = EDMA_getBaseAddr(fsiTxEdmaHandle);
            regionId = edmaChCfg->edmaRegionId;
            dmaCh0 = edmaChCfg->edmaTxChId[0];
            dmaCh1 = edmaChCfg->edmaTxChId[1];
            param0 = edmaChCfg->edmaTxParam[0];
            param1 = edmaChCfg->edmaTxParam[1];
            tccTx = edmaChCfg->edmaTccTx;

            FSI_disableTxDMAEvent(txBaseAddr);
            /* Free channel */
            EDMA_freeChannelRegion(edmaBaseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaCh0, EDMA_TRIG_MODE_EVENT, 0, EDMA_TEST_EVT_QUEUE_NO);

            EDMA_freeChannelRegion(edmaBaseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaCh1, EDMA_TRIG_MODE_EVENT, tccTx, EDMA_TEST_EVT_QUEUE_NO);

            status = EDMA_freeTcc(fsiTxEdmaHandle, &tccTx);
            DebugP_assert(status == SystemP_SUCCESS);

            /* Free the EDMA resources managed by driver. */
            status = EDMA_freeDmaChannel(fsiTxEdmaHandle, &dmaCh0);
            DebugP_assert(status == SystemP_SUCCESS);
            status = EDMA_freeParam(fsiTxEdmaHandle, &param0);
            DebugP_assert(status == SystemP_SUCCESS);
            status = EDMA_freeDmaChannel(fsiTxEdmaHandle, &dmaCh1);
            DebugP_assert(status == SystemP_SUCCESS);
            status = EDMA_freeParam(fsiTxEdmaHandle, &param1);
            DebugP_assert(status == SystemP_SUCCESS);

            EDMA_unregisterIntr(fsiTxEdmaHandle, &edmaChCfg->edmaIntrObjTx);
        }
    }

    return status;

}

/* for FSI-Rx configuration */

int32_t FSI_Rx_dmaOpen(FSI_Rx_Handle fsiRxHandle, FSI_Rx_DmaChConfig dmaChCfg)
{
    int32_t             status = SystemP_FAILURE;
    FSI_Rx_EdmaChConfig *edmaChCfg = NULL;
    uint32_t            baseAddr, regionId;
    uint32_t            isEdmaInterruptEnabled;
    EDMA_Handle         fsiRxEdmaHandle;
    FSI_Rx_Object       *object;
    FSI_Rx_Config *config = NULL;

    if((NULL != fsiRxHandle) && (NULL != dmaChCfg))
    {
        edmaChCfg = (FSI_Rx_EdmaChConfig *)dmaChCfg;
        config = (FSI_Rx_Config*)fsiRxHandle;
        object = config->object;

        fsiRxEdmaHandle = (EDMA_Handle) object->fsiRxDmaHandle;

        if(fsiRxEdmaHandle != NULL)
        {
            /* Read base address of allocated EDMA instance */
            baseAddr = EDMA_getBaseAddr(fsiRxEdmaHandle);

            /* Read the region ID of the EDMA instance */
            regionId = EDMA_getRegionId(fsiRxEdmaHandle);

            /* Store the EDMA parameters */
            edmaChCfg->edmaBaseAddr = baseAddr;
            edmaChCfg->edmaRegionId = regionId;

            /* Check if interrupt is enabled */
            isEdmaInterruptEnabled = EDMA_isInterruptEnabled(fsiRxEdmaHandle);

            if((baseAddr != (uint32_t)0U) && (regionId < (uint32_t)SOC_EDMA_NUM_REGIONS) && (isEdmaInterruptEnabled == (uint32_t)TRUE))
            {
                /* Validate the EDMA parameters for FsiRx */
                edmaChCfg->isOpen = (uint32_t)TRUE;
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

int32_t FSI_Rx_edmaChInit(const FSI_Rx_Object *FsiRxObj, uint32_t edmaEventNo,
                         uint32_t *edmaRxParam, uint32_t *edmaTccRx)
{
    int32_t status = SystemP_SUCCESS, chAllocStatus;
    EDMA_Handle fsiRxEdmaHandle = NULL;
    FSI_Rx_EdmaChConfig *edmaChCfg = NULL;

    if(NULL != FsiRxObj)
    {
        fsiRxEdmaHandle = (EDMA_Handle) FsiRxObj->fsiRxDmaHandle;
        edmaChCfg       = (FSI_Rx_EdmaChConfig *)FsiRxObj->fsiRxDmaChCfg;

        if (status == SystemP_SUCCESS)
        {
            /* Allocate EDMA channel for FSI TX transfer */
            chAllocStatus = EDMA_allocDmaChannel(fsiRxEdmaHandle, &edmaChCfg->edmaRxChId[edmaEventNo]);
            status += chAllocStatus;

            /* Allocate a Param ID for FSI TX transfer */
            status += EDMA_allocParam(fsiRxEdmaHandle, edmaRxParam);

            if(edmaTccRx != NULL)
            {
                /* Allocate EDMA TCC for FSI TX transfer */
                status += EDMA_allocTcc(fsiRxEdmaHandle, edmaTccRx);
            }

            if (status != SystemP_SUCCESS)
            {
                /* Free all allocated resources of edma */
                if ((chAllocStatus == SystemP_SUCCESS) && (edmaChCfg->edmaRxChId[edmaEventNo] != EDMA_RESOURCE_ALLOC_ANY))
                {
                    status += EDMA_freeDmaChannel(fsiRxEdmaHandle, &(edmaChCfg->edmaRxChId[edmaEventNo]));
                }
                if (edmaTccRx != NULL)
                {
                    if(*edmaTccRx != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        status += EDMA_freeTcc(fsiRxEdmaHandle, edmaTccRx);
                    }
                }
                if (*edmaRxParam != EDMA_RESOURCE_ALLOC_ANY)
                {
                    status += EDMA_freeParam(fsiRxEdmaHandle, edmaRxParam);
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

int32_t FSI_Rx_configureDma(const FSI_Rx_Object *fsiRxObj, uint32_t *dmaCh,
                void *src, void *dst, uint32_t *tcc, uint32_t *param, uint32_t regionId,
                uint32_t aCnt, uint32_t bCnt, uint32_t cCnt, uint32_t srcBIdx, uint32_t destBIdx,
                uint32_t srcCIdx, uint32_t destCIdx, uint32_t triggerMode)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr;
    EDMACCPaRAMEntry    edmaRxParam;
    FSI_Rx_EdmaChConfig *edmaChCfg = NULL;

    if(NULL != fsiRxObj)
    {
        edmaChCfg = (FSI_Rx_EdmaChConfig *)fsiRxObj->fsiRxDmaChCfg;

        /* Fetch the EDMA paramters for FSI-Rx transfer */
        baseAddr   = edmaChCfg->edmaBaseAddr;
        regionId   = edmaChCfg->edmaRegionId;

        if(tcc != NULL)
        {
            /* Request channel */
            EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            *dmaCh, *tcc, *param, EDMA_TEST_EVT_QUEUE_NO);
        }
        else
        {
            /* Request channel */
            EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            *dmaCh, 0, *param, EDMA_TEST_EVT_QUEUE_NO);
        }

        /* EDMA Params Description
        * tcc  - The tcc number on which the completion interrupt is generated
        * aCnt - Number of bytes in each word
        * bCnt - Number of words in each frame
        * cCnt - Number of frames
        * srcBIdx  - Index between consecutive words of a Source Buffer
        * destBIdx - Index between consecutive words of a Destination Buffer
        * srcCIdx  - Index between consecutive words of a Source Block
        * destCIdx - Index between consecutive words of a Destination Block
        */
        /* Transmit param set configuration */
        EDMA_ccPaRAMEntry_init(&edmaRxParam);
        edmaRxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)src);
        edmaRxParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)dst);
        edmaRxParam.aCnt          = (uint16_t) aCnt;
        edmaRxParam.bCnt          = (uint16_t) bCnt;
        edmaRxParam.cCnt          = (uint16_t) cCnt;
        edmaRxParam.bCntReload    = bCnt;
        edmaRxParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(srcBIdx);
        edmaRxParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(destBIdx);
        edmaRxParam.srcCIdx       = (int16_t) srcCIdx;
        edmaRxParam.destCIdx      = (int16_t) destCIdx;
        edmaRxParam.linkAddr      = 0xFFFFU;
        edmaRxParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(srcBIdx);
        edmaRxParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(destBIdx);
        /* Transfer Completion Interrupt is enabled only after Frame Tag And User
        * data is transmitted/received */
        if (tcc == NULL)
        {
            edmaRxParam.opt       = EDMA_OPT_SYNCDIM_MASK;
        }
        else
        {
            edmaRxParam.opt       = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
                                ((((uint32_t)*tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
        }

        EDMA_setPaRAM(baseAddr, *param, &edmaRxParam);
        EDMA_enableTransferRegion(baseAddr, regionId, *dmaCh,
                                triggerMode);
    }

    return status;

}

int32_t FSI_Rx_edmaChDeInit(const FSI_Rx_Object *fsiRxObj, uint32_t edmaEventNo)
{
    int32_t status;
    EDMA_Handle fsiRxEdmaHandle = NULL;
    FSI_Rx_EdmaChConfig *edmaChCfg = NULL;

    fsiRxEdmaHandle = (EDMA_Handle) fsiRxObj->fsiRxDmaHandle;
    edmaChCfg = (FSI_Rx_EdmaChConfig *)fsiRxObj->fsiRxDmaChCfg;

    //SemaphoreP_destruct(&gFsiDmaRxSemObject);
    status  = EDMA_unregisterIntr(fsiRxEdmaHandle, &(edmaChCfg->edmaIntrObjRx));
    return status;
}

int32_t FSI_Rx_edmaIntrInit(FSI_Rx_Object *FsiRxObj, uint32_t tccAlloc)
{
    int32_t     status;
    EDMA_Handle fsiRxEdmaHandle = NULL;
    FSI_Rx_EdmaChConfig *edmaChCfg = NULL;

    fsiRxEdmaHandle = (EDMA_Handle) FsiRxObj->fsiRxDmaHandle;
    edmaChCfg       = (FSI_Rx_EdmaChConfig *)FsiRxObj->fsiRxDmaChCfg;

    /* Register TX DMA Completion interrupt */
    edmaChCfg->edmaIntrObjRx.tccNum = tccAlloc;
    edmaChCfg->edmaIntrObjRx.cbFxn  = &fsi_edmaIsrRx;
    edmaChCfg->edmaIntrObjRx.appData = (void *) FsiRxObj;
    status = EDMA_registerIntr(fsiRxEdmaHandle, &edmaChCfg->edmaIntrObjRx);

    return status;
}

static void fsi_edmaIsrRx(Edma_IntrHandle intrHandle, void *args)
{
    FSI_Rx_DmaCompletionCallback(args);
}

int32_t FSI_Rx_dmaClose(FSI_Rx_Handle fsiRxHandle, FSI_Rx_DmaChConfig dmaChCfg)
{
    int32_t             status = SystemP_FAILURE;
    FSI_Rx_EdmaChConfig *edmaChCfg = NULL;
    uint32_t            edmaBaseAddr, regionId, rxBaseAddr;
    uint32_t            dmaCh0, dmaCh1, param0, param1, tccRx;
    EDMA_Handle         fsiRxEdmaHandle;
    FSI_Rx_Object       *object;
    FSI_Rx_Config *config = NULL;
    const FSI_Rx_Attrs *attrs;

    if((NULL != fsiRxHandle) && (NULL != dmaChCfg))
    {
        edmaChCfg = (FSI_Rx_EdmaChConfig *)dmaChCfg;
        config = (FSI_Rx_Config*)fsiRxHandle;
        object = config->object;
        attrs = config->attrs;
        rxBaseAddr = attrs->baseAddr;

        fsiRxEdmaHandle = (EDMA_Handle) object->fsiRxDmaHandle;

        if(fsiRxEdmaHandle != NULL)
        {
            edmaBaseAddr = EDMA_getBaseAddr(fsiRxEdmaHandle);
            regionId = edmaChCfg->edmaRegionId;
            dmaCh0 = edmaChCfg->edmaRxChId[0];
            dmaCh1 = edmaChCfg->edmaRxChId[1];
            param0 = edmaChCfg->edmaRxParam[0];
            param1 = edmaChCfg->edmaRxParam[1];
            tccRx = edmaChCfg->edmaTccRx;

            /* Free channel */
            FSI_disableRxDMAEvent(rxBaseAddr);

            EDMA_freeChannelRegion(edmaBaseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaCh0, EDMA_TRIG_MODE_EVENT, 0, EDMA_TEST_EVT_QUEUE_NO);

            EDMA_freeChannelRegion(edmaBaseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaCh1, EDMA_TRIG_MODE_EVENT, tccRx, EDMA_TEST_EVT_QUEUE_NO);

            status = EDMA_freeTcc(fsiRxEdmaHandle, &tccRx);
            DebugP_assert(status == SystemP_SUCCESS);

            status = EDMA_freeDmaChannel(fsiRxEdmaHandle, &dmaCh0);
            DebugP_assert(status == SystemP_SUCCESS);

            status = EDMA_freeParam(fsiRxEdmaHandle, &param0);
            DebugP_assert(status == SystemP_SUCCESS);

            status = EDMA_freeDmaChannel(fsiRxEdmaHandle, &dmaCh1);
            DebugP_assert(status == SystemP_SUCCESS);

            status = EDMA_freeParam(fsiRxEdmaHandle, &param1);
            DebugP_assert(status == SystemP_SUCCESS);

            EDMA_unregisterIntr(fsiRxEdmaHandle, &edmaChCfg->edmaIntrObjRx);
        }
    }

    return status;

}
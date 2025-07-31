/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
 *  \file mmcsd_edma.c
 *
 *  \brief File containing EDMA Driver APIs implementation for MMCSD.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/mmcsd/v1/lld/dma/edma/mmcsd_edma_lld.h>
#include <drivers/soc.h>
#include <kernel/dpl/CacheP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Value for A count                                                  */
#define EDMA_MMCSD_A_COUNT                          (4U)
/* Max Value for EDMA count - 31KB                                    */
#define MAX_EDMA_COUNT                              (31744U)
/* Transmit EDMA channel event queue number                           */
#define EDMA_MMCSD_TX_EVT_QUEUE_NO                  (0U)
/* Receive EDMA channel event queue number                            */
#define EDMA_MMCSD_RX_EVT_QUEUE_NO                  (1U)

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static void MMCSD_edmaIsrRx(Edma_IntrHandle intrHandle, void *args);
static void MMCSD_edmaIsrTx(Edma_IntrHandle intrHandle, void *args);
static void MMCSD_edmaChConfig_init(MMCSD_EdmaChConfig *edmaParams);
static void MMCSD_edmaDoNothing(Edma_IntrHandle intrHandle, void *args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void MMCSD_edmaChConfig_init(MMCSD_EdmaChConfig *edmaParams)
{
    if( edmaParams != NULL)
    {
        edmaParams->edmaTccTx = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaTccRx = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaTccDummy = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaTxParam = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaRxParam = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaDummyParam = EDMA_RESOURCE_ALLOC_ANY;  
        edmaParams->edmaRegionId = 0U;
        edmaParams->edmaBaseAddr = 0U;
        edmaParams->isIntEnabled = 0U;
    }
}

int32_t MMCSD_edmaChInit(MMCSDLLD_Handle hMmcsd)
{
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh,dmaTxCh, tccRx, tccTx, tccDummy, paramRx, paramTx, paramDummy;
    int32_t             status = MMCSD_STS_ERR;
    uint32_t            edmaStatus;
    uint32_t            isEdmaInterruptEnabled;
    MMCSDLLD_InitHandle  initHandle = hMmcsd->initHandle;
    MMCSD_EdmaChConfig     *edmaParams = (MMCSD_EdmaChConfig *)initHandle->mmcsdDmaChConfig;
    Edma_IntrObject     *edmaIntrObjectRx = &edmaParams->edmaIntrObjRx;
    Edma_IntrObject     *edmaIntrObjectTx = &edmaParams->edmaIntrObjTx;
    Edma_IntrObject     *edmaIntrObjectDummy = &edmaParams->edmaIntrObjDummy;
    EDMA_Handle         mmcsdEdmaHandle = (EDMA_Handle *)initHandle->mmcsdDmaHandle;

    if (mmcsdEdmaHandle != NULL)
    {
        MMCSD_edmaChConfig_init(edmaParams);
        /* Read base address of allocated EDMA instance */
        baseAddr = EDMA_getBaseAddr(mmcsdEdmaHandle);
        /* Check if interrupt is enabled */
        isEdmaInterruptEnabled = EDMA_isInterruptEnabled(mmcsdEdmaHandle);
        
        if(baseAddr != 0U && isEdmaInterruptEnabled)
        {
            status = MMCSD_STS_SUCCESS;

            /* Read the region ID of the EDMA instance */
            regionId = EDMA_getRegionId(mmcsdEdmaHandle);

            if(regionId < SOC_EDMA_NUM_REGIONS)
            {
                /* Allocate EDMA channel for MMCSD Tx transfer */
                dmaTxCh = edmaParams->edmaTxChId;
                status += EDMA_allocDmaChannel(mmcsdEdmaHandle, &dmaTxCh);

                /* Allocate EDMA TCC for MMCSD Tx transfer */
                tccTx = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocTcc(mmcsdEdmaHandle, &tccTx);

                /* Allocate a Param ID for MMCSD Tx transfer */
                paramTx = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocParam(mmcsdEdmaHandle, &paramTx);

                if(status == MMCSD_STS_SUCCESS)
                {
                    edmaStatus = EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                        dmaTxCh, tccTx, paramTx, EDMA_MMCSD_TX_EVT_QUEUE_NO);

                    if(edmaStatus != TRUE )
                    {
                        status = MMCSD_STS_ERR;
                    }

                    /* Register interrupt */
                    edmaIntrObjectTx->tccNum = tccTx;
                    edmaIntrObjectTx->cbFxn  = &MMCSD_edmaIsrTx;
                    edmaIntrObjectTx->appData = (void *) hMmcsd;
                    status += EDMA_registerIntr(mmcsdEdmaHandle, edmaIntrObjectTx);
                    /* Store the EDMA parameters and handle*/
                    edmaParams->edmaBaseAddr = baseAddr;
                    edmaParams->edmaRegionId = regionId;
                    edmaParams->edmaTxParam = paramTx;
                    edmaParams->edmaTxChId = dmaTxCh;
                    edmaParams->edmaTccTx = tccTx;
                    edmaParams->isIntEnabled = isEdmaInterruptEnabled;
                }

                if(status == MMCSD_STS_SUCCESS)
                {

                    /* Allocate EDMA TCC for MMCSD Dummy transfer */
                    tccDummy = EDMA_RESOURCE_ALLOC_ANY;
                    status += EDMA_allocTcc(mmcsdEdmaHandle, &tccDummy);

                    /* Allocate a Param ID for MMCSD Dummy transfer */
                    paramDummy = EDMA_RESOURCE_ALLOC_ANY;
                    status += EDMA_allocParam(mmcsdEdmaHandle, &paramDummy);

                    if(status == MMCSD_STS_SUCCESS)
                    {

                        /* Register Dummy interrupt */
                        edmaIntrObjectDummy->tccNum = tccDummy;
                        edmaIntrObjectDummy->cbFxn  = &MMCSD_edmaDoNothing;
                        edmaIntrObjectDummy->appData = (void *) hMmcsd;
                        status += EDMA_registerIntr(mmcsdEdmaHandle, edmaIntrObjectDummy);

                        if(status == MMCSD_STS_SUCCESS)
                        {
                            /* Store the EDMA parameters for McSPI Dummy*/
                            edmaParams->edmaDummyParam = paramDummy;
                            edmaParams->edmaTccDummy   = tccDummy;
                        }
                    }
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Allocate EDMA channel for MMCSD RX transfer */
                    dmaRxCh = edmaParams->edmaRxChId;
                    status += EDMA_allocDmaChannel(mmcsdEdmaHandle, &dmaRxCh);

                    /* Allocate EDMA TCC for MMCSD RX transfer */
                    tccRx = EDMA_RESOURCE_ALLOC_ANY;
                    status += EDMA_allocTcc(mmcsdEdmaHandle, &tccRx);

                    /* Allocate a Param ID for MMCSD RX transfer */
                    paramRx = EDMA_RESOURCE_ALLOC_ANY;
                    status += EDMA_allocParam(mmcsdEdmaHandle, &paramRx);

                    if(status == MMCSD_STS_SUCCESS)
                    {
                        edmaStatus = EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                            dmaRxCh, tccRx, paramRx, EDMA_MMCSD_RX_EVT_QUEUE_NO);

                        if(edmaStatus != TRUE )
                        {
                            status = MMCSD_STS_ERR;
                        }

                        /* Register RX interrupt */
                        edmaIntrObjectRx->tccNum = tccRx;
                        edmaIntrObjectRx->cbFxn  = &MMCSD_edmaIsrRx;
                        edmaIntrObjectRx->appData = (void *) hMmcsd;
                        status += EDMA_registerIntr(mmcsdEdmaHandle, edmaIntrObjectRx);

                        if(status == MMCSD_STS_SUCCESS)
                        {
                            /* Store the EDMA parameters for McSPI RX*/
                            edmaParams->edmaRxParam = paramRx;
                            edmaParams->edmaRxChId  = dmaRxCh;
                            edmaParams->edmaTccRx   = tccRx;
                            status = MMCSD_STS_SUCCESS;
                        }
                    }
                }
            }
            else
            {
                status = MMCSD_STS_ERR;
            }
        }
    }

    return status;
}

int32_t MMCSD_edmaTransfer(MMCSDLLD_Handle hMmcsd, MMCSDLLD_Transaction *trans)
{
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh,dmaTxCh, tccRx, tccTx, tccDummy, paramRx, paramTx, paramDummy;
    MMCSD_EdmaChConfig  *edmaParams = (MMCSD_EdmaChConfig *)hMmcsd->initHandle->mmcsdDmaChConfig;
    MMCSDLLD_Object     *mmcsdObj = (MMCSDLLD_Object *)hMmcsd;    
    EDMACCPaRAMEntry    edmaTxParam, edmaRxParam, edmaDummyParam;
    uint32_t            edmaStatus;
    int32_t             status = MMCSD_STS_ERR;
    int16_t             cCount = MAX_EDMA_COUNT  < trans->blockCount ? MAX_EDMA_COUNT : trans->blockCount;

    /* Fetch the EDMA paramters for MMCSD transfer */
    baseAddr      = edmaParams->edmaBaseAddr;
    regionId      = edmaParams->edmaRegionId;

    if(trans->flags & MMCSD_CMDRSP_WRITE)
    {
        /* EDMA Tx transaction */

        CacheP_wbInv(trans->dataBuf, trans->blockSize*cCount, CacheP_TYPE_ALL);
        
        dmaTxCh         = edmaParams->edmaTxChId;
        tccTx           = edmaParams->edmaTccTx;
        tccDummy        = edmaParams->edmaTccDummy;
        paramTx         = edmaParams->edmaTxParam;  
        paramDummy      = edmaParams->edmaDummyParam;

        /* Program Param Set */
        EDMA_ccPaRAMEntry_init(&edmaTxParam);

        edmaTxParam.srcAddr       = (uint32_t) SOC_virtToPhy(trans->dataBuf);
        edmaTxParam.destAddr      = (uint32_t) SOC_virtToPhy((uint8_t *)(mmcsdObj->initHandle->baseAddr + CSL_MMC_DATA));
        edmaTxParam.aCnt          = (uint16_t) EDMA_MMCSD_A_COUNT;
        edmaTxParam.bCnt          = (uint16_t) (trans->blockSize/EDMA_MMCSD_A_COUNT);
        edmaTxParam.cCnt          = (uint16_t) cCount;
        edmaTxParam.bCntReload    = (uint16_t) (trans->blockSize/EDMA_MMCSD_A_COUNT);
        edmaTxParam.srcBIdx       = (int16_t) edmaTxParam.aCnt;
        edmaTxParam.destBIdx      = (int16_t) 0;
        edmaTxParam.srcCIdx       = (int16_t) (trans->blockSize);
        edmaTxParam.destCIdx      = (int16_t) 0;
        edmaTxParam.linkAddr      = 0xFFFFU;
        edmaTxParam.opt           = (((tccTx) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
                                EDMA_OPT_TCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK;

        EDMA_setPaRAM(baseAddr, paramTx, &edmaTxParam);

        /* Initialize TX Param Set*/
        EDMA_ccPaRAMEntry_init(&edmaDummyParam);

        /* Dummy param set configuration */
        edmaDummyParam.aCnt          = (uint16_t) EDMA_MMCSD_A_COUNT;
        edmaDummyParam.linkAddr      = 0xFFFFU;
        edmaDummyParam.opt           = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_STATIC_MASK) |
                                    ((tccDummy << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK);

        /* Write Tx param set */
        EDMA_setPaRAM(baseAddr, paramDummy, &edmaDummyParam);

        /* Link  dummy param ID */
        EDMA_linkChannel(baseAddr, paramTx, paramDummy);
        
        /* Set event trigger to start MMCSD transfer */
        edmaStatus = EDMA_enableTransferRegion(baseAddr, regionId, dmaTxCh, EDMA_TRIG_MODE_EVENT);

        if (edmaStatus == TRUE)
        {
            status = MMCSD_STS_SUCCESS;
        }
        else
        {
            status = MMCSD_STS_ERR;
        }
    }
    else
    {
        /* EDMA Rx transfer */
        dmaRxCh    = edmaParams->edmaRxChId;
        tccRx      = edmaParams->edmaTccRx; 
        paramRx    = edmaParams->edmaRxParam;

        /* Program Param Set */
        EDMA_ccPaRAMEntry_init(&edmaRxParam);
        edmaRxParam.srcAddr       = (uint32_t) SOC_virtToPhy((uint8_t *)(mmcsdObj->initHandle->baseAddr + CSL_MMC_DATA));
        edmaRxParam.destAddr      = (uint32_t) SOC_virtToPhy(trans->dataBuf);
        edmaRxParam.aCnt          = (uint16_t) EDMA_MMCSD_A_COUNT;
        edmaRxParam.bCnt          = (uint16_t) (trans->blockSize/EDMA_MMCSD_A_COUNT);
        edmaRxParam.cCnt          = (uint16_t) cCount;
        edmaRxParam.bCntReload    = (uint16_t) (trans->blockSize/EDMA_MMCSD_A_COUNT);
        edmaRxParam.srcBIdx       = (int16_t) 0;
        edmaRxParam.destBIdx      = (int16_t) edmaRxParam.aCnt;
        edmaRxParam.srcCIdx       = (int16_t) 0;
        edmaRxParam.destCIdx      = (int16_t) (trans->blockSize);
        edmaRxParam.linkAddr      = 0xFFFFU;
        edmaRxParam.opt           = (((tccRx) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) | 
                                    EDMA_OPT_TCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK;

        EDMA_setPaRAM(baseAddr, paramRx, &edmaRxParam);
        
        /* Set event trigger to start MMCSD transfer */
        edmaStatus = EDMA_enableTransferRegion(baseAddr, regionId, dmaRxCh, EDMA_TRIG_MODE_EVENT);
        
        if (edmaStatus == TRUE)
        {
            CacheP_inv(trans->dataBuf, trans->blockSize*cCount, CacheP_TYPE_ALL);
            status = MMCSD_STS_SUCCESS;
        }
        else
        {
            status = MMCSD_STS_ERR;
        }
    }
    return status;
}

int32_t MMCSD_edmaChDeinit(MMCSDLLD_Handle hMmcsd)
{
    int32_t             status = MMCSD_STS_SUCCESS;
    uint32_t            edmaStatus;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh, dmaTxCh, tccRx, tccTx, tccDummy, paramRx, paramTx, paramDummy;
    MMCSDLLD_InitHandle  initHandle = hMmcsd->initHandle;
    MMCSD_EdmaChConfig     *edmaParams = (MMCSD_EdmaChConfig *)initHandle->mmcsdDmaChConfig;

    /* Fetch the EDMA paramters */
    baseAddr = edmaParams->edmaBaseAddr;
    regionId = edmaParams->edmaRegionId;
    dmaTxCh  = edmaParams->edmaTxChId;
    dmaRxCh  = edmaParams->edmaRxChId;
    tccTx    = edmaParams->edmaTccTx;
    tccRx    = edmaParams->edmaTccRx;
    paramRx  = edmaParams->edmaRxParam;
    paramTx  = edmaParams->edmaTxParam;
    tccDummy  = edmaParams->edmaTccDummy;
    paramDummy = edmaParams->edmaDummyParam;

    /* Free Tx channel */
    edmaStatus = EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaTxCh, EDMA_TRIG_MODE_EVENT, tccTx, EDMA_MMCSD_TX_EVT_QUEUE_NO);

    if(edmaStatus == TRUE)
    {
        status = MMCSD_STS_SUCCESS;
    }
    else
    {
        status = MMCSD_STS_ERR;
    }
    /* Unregister the EDMA interrupt */
    status += EDMA_unregisterIntr(initHandle->mmcsdDmaHandle, &edmaParams->edmaIntrObjTx);
    /* Free the EDMA resources managed by driver. */
    status += EDMA_freeDmaChannel(initHandle->mmcsdDmaHandle, &dmaTxCh);
    status += EDMA_freeTcc(initHandle->mmcsdDmaHandle, &tccTx);
    status += EDMA_freeParam(initHandle->mmcsdDmaHandle, &paramTx);

    status += EDMA_unregisterIntr(initHandle->mmcsdDmaHandle, &edmaParams->edmaIntrObjDummy);
    status += EDMA_freeTcc(initHandle->mmcsdDmaHandle, &tccDummy);
    status += EDMA_freeParam(initHandle->mmcsdDmaHandle, &paramDummy);


    /* Free Rx channel */
    edmaStatus = EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaRxCh, EDMA_TRIG_MODE_EVENT, tccRx, EDMA_MMCSD_RX_EVT_QUEUE_NO);

    if(edmaStatus == TRUE)
    {
        status = MMCSD_STS_SUCCESS;
    }
    else
    {
        status = MMCSD_STS_ERR;
    }
    /* Unregister the EDMA interrupt */
    status += EDMA_unregisterIntr(initHandle->mmcsdDmaHandle, &edmaParams->edmaIntrObjRx);
    /* Free the EDMA resources managed by driver. */
    status += EDMA_freeDmaChannel(initHandle->mmcsdDmaHandle, &dmaRxCh);
    status += EDMA_freeTcc(initHandle->mmcsdDmaHandle, &tccRx);
    status += EDMA_freeParam(initHandle->mmcsdDmaHandle, &paramRx);

    return status;
}

static void MMCSD_edmaIsrRx(Edma_IntrHandle intrHandle, void *args)
{
    MMCSDLLD_Handle handle  = (MMCSDLLD_Handle) args;
    
    if(handle->mmcsdTxn.blockCount > MAX_EDMA_COUNT)
    {
        handle->mmcsdTxn.blockCount -= MAX_EDMA_COUNT;
        handle->mmcsdTxn.dataBuf += (MAX_EDMA_COUNT * handle->mmcsdTxn.blockSize);
        
        MMCSD_edmaTransfer(handle, &handle->mmcsdTxn);
    }
    else
    {
        MMCSD_lld_completeCurrTransfer(handle,handle->xferState);
    }
}

static void MMCSD_edmaIsrTx(Edma_IntrHandle intrHandle, void *args)
{
    MMCSDLLD_Handle handle  = (MMCSDLLD_Handle) args;
  
    if(handle->mmcsdTxn.blockCount > MAX_EDMA_COUNT)
    {
        handle->mmcsdTxn.blockCount -= MAX_EDMA_COUNT;
        handle->mmcsdTxn.dataBuf += (MAX_EDMA_COUNT * handle->mmcsdTxn.blockSize);
        
        MMCSD_edmaTransfer(handle, &handle->mmcsdTxn);
    }
    else
    {
        MMCSD_lld_completeCurrTransfer(handle,handle->xferState);
    }
}

static void MMCSD_edmaDoNothing(Edma_IntrHandle intrHandle, void *args)
{
    /* This function is a placeholder for the dummy transfer interrupt handler.
     * It does not perform any action and is used to avoid unnecessary processing
     * during dummy transfers.
     */
    (void) intrHandle; // Suppress unused parameter warning
    (void) args;      // Suppress unused parameter warning
}
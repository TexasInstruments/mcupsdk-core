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

#include <drivers/gpmc/v0/dma/gpmc_dma.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/edma.h>
#include <drivers/gpmc.h>
#include <drivers/soc.h>

/* Value for A count*/
#define EDMA_GPMC_A_COUNT           (1U)
/* Max Value for EDMA count */
#define MAX_EDMA_COUNT              (31*1024)
/* Value for C count */
#define EDMA_GPMC_C_COUNT           (1U)
/* Event queue to be used  */
#define EDMA_GPMC_EVT_QUEUE_NO      (0U)


static void GPMC_edmaIsrFxn(Edma_IntrHandle intrHandle, void *args);

void GPMC_EdmaParams_init(void * pvEdmaParams)
{
    if( pvEdmaParams != NULL)
    {
        Gpmc_DmaArgs *edmaParams = (Gpmc_DmaArgs *)pvEdmaParams;
        edmaParams->edmaTcc = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaChId = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaParam = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaRegionId = 0U;
        edmaParams->edmaBaseAddr = 0U;
        edmaParams->isIntEnabled = 0U;
    }
}

int32_t GpmcDma_edmaOpen(Gpmc_DmaArgs* gpmcDmaArgs)
{

    uint32_t baseAddr, regionId, dmaCh, tcc, param;
    int32_t status = SystemP_FAILURE;
    uint32_t isEdmaInterruptEnabled;
    Gpmc_DmaArgs *edmaParams = gpmcDmaArgs;
    Edma_IntrObject *edmaIntrObject = &(edmaParams->edmaIntrObj);
    EDMA_Handle gpmcEdmaHandle = EDMA_getHandle(edmaParams->edmaInst);
    SemaphoreP_Object *gEdmaTransferDoneSem = &(edmaParams->gEdmaTransferDoneSem);

    if (gpmcEdmaHandle != NULL)
    {
        GPMC_EdmaParams_init(edmaParams);
        /* Read base address of allocated EDMA instance */
        baseAddr = EDMA_getBaseAddr(gpmcEdmaHandle);

        if(baseAddr != 0)
        {
            status = SystemP_SUCCESS;

            /* Check if interrupt is enabled */
            isEdmaInterruptEnabled = EDMA_isInterruptEnabled(gpmcEdmaHandle);
            /* Read the region ID of the EDMA instance */
            regionId = EDMA_getRegionId(gpmcEdmaHandle);

            if(regionId < SOC_EDMA_NUM_REGIONS)
            {
                /* Allocate EDMA channel for GPMC transfer */
                dmaCh = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocDmaChannel(gpmcEdmaHandle, &dmaCh);

                /* Allocate EDMA TCC for GPMC transfer */
                tcc = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocTcc(gpmcEdmaHandle, &tcc);

                /* Allocate a Param ID for GPMC transfer */
                param = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocParam(gpmcEdmaHandle, &param);
                if(status == SystemP_SUCCESS)
                {
                    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                        dmaCh, tcc, param, EDMA_GPMC_EVT_QUEUE_NO);

                    if(isEdmaInterruptEnabled == TRUE)
                    {
                        status = SemaphoreP_constructBinary(gEdmaTransferDoneSem, 0);
                        DebugP_assert(SystemP_SUCCESS == status);

                        /* Register interrupt */
                        edmaIntrObject->tccNum = tcc;
                        edmaIntrObject->cbFxn  = &GPMC_edmaIsrFxn;
                        edmaIntrObject->appData = (void *) gEdmaTransferDoneSem;
                        status = EDMA_registerIntr(gpmcEdmaHandle, edmaIntrObject);
                        DebugP_assert(status == SystemP_SUCCESS);
                    }

                    /* Store the EDMA paramters and handle*/
                    edmaParams->edmaBaseAddr = baseAddr;
                    edmaParams->edmaRegionId = regionId;
                    edmaParams->edmaParam = param;
                    edmaParams->edmaChId = dmaCh;
                    edmaParams->edmaTcc = tcc;
                    edmaParams->isIntEnabled = isEdmaInterruptEnabled;
                }
                if(status != SystemP_SUCCESS)
                {
                    if(dmaCh != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        EDMA_freeDmaChannel(gpmcEdmaHandle, &dmaCh);
                    }
                    if(tcc != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        EDMA_freeTcc(gpmcEdmaHandle, &tcc);
                    }
                    if(param != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        EDMA_freeParam(gpmcEdmaHandle, &param);
                    }
                }
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }

    return status;
}

int32_t GpmcDma_edmaClose(Gpmc_DmaArgs* gpmcDmaArgs)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId, dmaCh, tcc, param;
    Gpmc_DmaArgs *edmaParams = gpmcDmaArgs;
    EDMA_Handle gpmcEdmaHandle = EDMA_getHandle(edmaParams->edmaInst);

    /* Fetch the EDMA paramters */
    baseAddr = edmaParams->edmaBaseAddr;
    regionId = edmaParams->edmaRegionId;
    dmaCh    = edmaParams->edmaChId;
    tcc      = edmaParams->edmaTcc;
    param    = edmaParams->edmaParam;

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_GPMC_EVT_QUEUE_NO);

    if(edmaParams->isIntEnabled == TRUE)
    {
        status = EDMA_unregisterIntr(gpmcEdmaHandle, &edmaParams->edmaIntrObj);
        SemaphoreP_destruct(&edmaParams->gEdmaTransferDoneSem);
    }
    /* Free the EDMA resources managed by driver. */
    status += EDMA_freeDmaChannel(gpmcEdmaHandle, &dmaCh);
    status += EDMA_freeTcc(gpmcEdmaHandle, &tcc);
    status += EDMA_freeParam(gpmcEdmaHandle, &param);

    return status;
}

int32_t GpmcDma_edmaCopy(Gpmc_DmaArgs* gpmcDmaArgs, uint32_t *dst, uint32_t *src, uint32_t length, uint8_t fifoDrain)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId, dmaCh, tcc, param;
    Gpmc_DmaArgs *edmaParams = gpmcDmaArgs;
    EDMACCPaRAMEntry   edmaParam;

    /* Fetch the EDMA paramters for GPMC transfer */
    baseAddr = edmaParams->edmaBaseAddr;
    regionId = edmaParams->edmaRegionId;
    dmaCh    = edmaParams->edmaChId;
    tcc      = edmaParams->edmaTcc;
    param    = edmaParams->edmaParam;

    CacheP_wb(src, length*EDMA_GPMC_A_COUNT, CacheP_TYPE_ALL);
    CacheP_wb(dst, length*EDMA_GPMC_A_COUNT, CacheP_TYPE_ALL);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam);

    edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy(src);
    edmaParam.destAddr      = (uint32_t) SOC_virtToPhy(dst);
    if(length > MAX_EDMA_COUNT)
    {
        edmaParam.aCnt      = (uint16_t) MAX_EDMA_COUNT;
    }
    else
    {
        edmaParam.aCnt          = (uint16_t) EDMA_GPMC_A_COUNT;
    }
    edmaParam.bCnt          = (uint16_t) (length/edmaParam.aCnt);
    edmaParam.cCnt          = (uint16_t) EDMA_GPMC_C_COUNT;
    edmaParam.bCntReload    = (uint16_t) (length/edmaParam.aCnt);
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(edmaParam.aCnt);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(edmaParam.aCnt);
    edmaParam.srcCIdx       = (int16_t) EDMA_GPMC_A_COUNT;
    edmaParam.destCIdx      = (int16_t) EDMA_GPMC_A_COUNT;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(edmaParam.aCnt);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(edmaParam.aCnt);
    edmaParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
        (((tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(baseAddr, param, &edmaParam);

    /* Set manual trigger to start GPMC transfer */
    EDMA_enableTransferRegion(baseAddr, regionId, dmaCh,
            EDMA_TRIG_MODE_MANUAL);

    if(edmaParams->isIntEnabled == true)
    {
        SemaphoreP_pend(&edmaParams->gEdmaTransferDoneSem, SystemP_WAIT_FOREVER);
    }
    else
    {
        /* Poll for transfer completion */
        while(EDMA_readIntrStatusRegion(baseAddr, regionId, tcc) != 1);

        EDMA_clrIntrRegion(baseAddr, regionId, tcc);
    }

    if((length > MAX_EDMA_COUNT) && (length % MAX_EDMA_COUNT != 0))
    {
        edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy(src) + (MAX_EDMA_COUNT * (length / MAX_EDMA_COUNT));
        edmaParam.destAddr      = (uint32_t) SOC_virtToPhy(dst) + (MAX_EDMA_COUNT * (length / MAX_EDMA_COUNT));
        edmaParam.aCnt      = (uint16_t) EDMA_GPMC_A_COUNT;
        edmaParam.bCnt      = (uint16_t) (length % MAX_EDMA_COUNT);
        edmaParam.srcBIdx   = (int16_t) EDMA_PARAM_BIDX(EDMA_GPMC_A_COUNT);
        edmaParam.destBIdx  = (int16_t) EDMA_PARAM_BIDX(EDMA_GPMC_A_COUNT);
        edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_GPMC_A_COUNT);
        edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_GPMC_A_COUNT);

        EDMA_setPaRAM(baseAddr, param, &edmaParam);

        /* Set manual trigger to start GPMC transfer */
        EDMA_enableTransferRegion(baseAddr, regionId, dmaCh,
                EDMA_TRIG_MODE_MANUAL);

        if(edmaParams->isIntEnabled == TRUE)
        {
            SemaphoreP_pend(&edmaParams->gEdmaTransferDoneSem, SystemP_WAIT_FOREVER);
        }
        else
        {
            /* Poll for transfer completion */
            while(EDMA_readIntrStatusRegion(baseAddr, regionId, tcc) != 1);

            EDMA_clrIntrRegion(baseAddr, regionId, tcc);
        }
    }

    CacheP_inv(dst, length*EDMA_GPMC_A_COUNT, CacheP_TYPE_ALL);

    return status;
}

static void GPMC_edmaIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);
    SemaphoreP_post(semObjPtr);
}
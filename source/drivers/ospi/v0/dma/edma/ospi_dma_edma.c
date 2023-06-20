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

#include <drivers/edma.h>
#include <drivers/ospi.h>
#include <drivers/ospi/v0/dma/ospi_dma.h>
#include <drivers/ospi/v0/dma/edma/ospi_dma_edma.h>
#include <drivers/soc.h>
#include <kernel/dpl/CacheP.h>

/* Value for A count*/
#define EDMA_QSPI_A_COUNT           (1U)
/* Max Value for EDMA count */
#define MAX_EDMA_COUNT              (31*1024)
/* Value for C count */
#define EDMA_QSPI_C_COUNT           (1U)
/* Event queue to be used  */
#define EDMA_QSPI_EVT_QUEUE_NO      (0U)


static void OSPI_edmaIsrFxn(Edma_IntrHandle intrHandle, void *args);
static int32_t OspiDma_edmaOpen(void* ospiDmaArgs);
static int32_t OspiDma_edmaClose(OSPI_DmaHandle handle, void* ospiDmaArgs);
static int32_t OspiDma_edmaCopy(void* ospiDmaArgs, void* dst, void* src, uint32_t length);

OSPI_DmaFxns gOspiDmaEdmaFxns =
{
    .dmaOpenFxn = OspiDma_edmaOpen,
    .dmaCloseFxn = OspiDma_edmaClose,
    .dmaCopyFxn = OspiDma_edmaCopy,
};

void OSPI_EdmaParams_init(void * pvEdmaParams)
{
    if( pvEdmaParams != NULL)
    {
        OspiDma_EdmaArgs *edmaParams = (OspiDma_EdmaArgs *)pvEdmaParams;
        edmaParams->edmaTcc = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaChId = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaParam = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaRegionId = 0U;
        edmaParams->edmaBaseAddr = 0U;
        edmaParams->isIntEnabled = 0U;
    }
}

static int32_t OspiDma_edmaOpen(void* ospiDmaArgs)
{

    uint32_t baseAddr, regionId, dmaCh, tcc, param;
    int32_t status = SystemP_FAILURE;
    uint32_t isEdmaInterruptEnabled;
    OspiDma_EdmaArgs *edmaParams = (OspiDma_EdmaArgs*)ospiDmaArgs;
    Edma_IntrObject *edmaIntrObject = &(edmaParams->edmaIntrObj);
    EDMA_Handle qspiEdmaHandle = EDMA_getHandle(edmaParams->edmaInst);
    SemaphoreP_Object *gEdmaTransferDoneSem = &(edmaParams->gEdmaTransferDoneSem);

    if (qspiEdmaHandle != NULL)
    {
        OSPI_EdmaParams_init(edmaParams);
        /* Read base address of allocated EDMA instance */
        baseAddr = EDMA_getBaseAddr(qspiEdmaHandle);

        if(baseAddr != 0)
        {
            status = SystemP_SUCCESS;

            /* Check if interrupt is enabled */
            isEdmaInterruptEnabled = EDMA_isInterruptEnabled(qspiEdmaHandle);
            /* Read the region ID of the EDMA instance */
            regionId = EDMA_getRegionId(qspiEdmaHandle);

            if(regionId < SOC_EDMA_NUM_REGIONS)
            {
                /* Allocate EDMA channel for QSPI transfer */
                dmaCh = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocDmaChannel(qspiEdmaHandle, &dmaCh);

                /* Allocate EDMA TCC for QSPI transfer */
                tcc = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocTcc(qspiEdmaHandle, &tcc);

                /* Allocate a Param ID for QSPI transfer */
                param = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocParam(qspiEdmaHandle, &param);
                if(status == SystemP_SUCCESS)
                {
                    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                        dmaCh, tcc, param, EDMA_QSPI_EVT_QUEUE_NO);

                    if(isEdmaInterruptEnabled == TRUE)
                    {
                        status = SemaphoreP_constructBinary(gEdmaTransferDoneSem, 0);
                        DebugP_assert(SystemP_SUCCESS == status);

                        /* Register interrupt */
                        edmaIntrObject->tccNum = tcc;
                        edmaIntrObject->cbFxn  = &OSPI_edmaIsrFxn;
                        edmaIntrObject->appData = (void *) gEdmaTransferDoneSem;
                        status = EDMA_registerIntr(qspiEdmaHandle, edmaIntrObject);
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
                        EDMA_freeDmaChannel(qspiEdmaHandle, &dmaCh);
                    }
                    if(tcc != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        EDMA_freeTcc(qspiEdmaHandle, &tcc);
                    }
                    if(param != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        EDMA_freeParam(qspiEdmaHandle, &param);
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

static int32_t OspiDma_edmaClose(OSPI_DmaHandle handle, void* ospiDmaArgs)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId, dmaCh, tcc, param;
    OspiDma_EdmaArgs *edmaParams = (OspiDma_EdmaArgs*)ospiDmaArgs;
    EDMA_Handle qspiEdmaHandle = EDMA_getHandle(edmaParams->edmaInst);

    /* Fetch the EDMA paramters */
    baseAddr = edmaParams->edmaBaseAddr;
    regionId = edmaParams->edmaRegionId;
    dmaCh    = edmaParams->edmaChId;
    tcc      = edmaParams->edmaTcc;
    param    = edmaParams->edmaParam;

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_QSPI_EVT_QUEUE_NO);

    if(edmaParams->isIntEnabled == TRUE)
    {
        status = EDMA_unregisterIntr(qspiEdmaHandle, &edmaParams->edmaIntrObj);
        SemaphoreP_destruct(&edmaParams->gEdmaTransferDoneSem);
    }
    /* Free the EDMA resources managed by driver. */
    status += EDMA_freeDmaChannel(qspiEdmaHandle, &dmaCh);
    status += EDMA_freeTcc(qspiEdmaHandle, &tcc);
    status += EDMA_freeParam(qspiEdmaHandle, &param);

    return status;
}

static int32_t OspiDma_edmaCopy(void* ospiDmaArgs, void* dst, void* src, uint32_t length)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId, dmaCh, tcc, param;
    OspiDma_EdmaArgs *edmaParams = (OspiDma_EdmaArgs*)ospiDmaArgs;
    EDMACCPaRAMEntry   edmaParam;

    /* Fetch the EDMA paramters for QSPI transfer */
    baseAddr = edmaParams->edmaBaseAddr;
    regionId = edmaParams->edmaRegionId;
    dmaCh    = edmaParams->edmaChId;
    tcc      = edmaParams->edmaTcc;
    param    = edmaParams->edmaParam;

    CacheP_wb(src, length*EDMA_QSPI_A_COUNT, CacheP_TYPE_ALL);
    CacheP_wb(dst, length*EDMA_QSPI_A_COUNT, CacheP_TYPE_ALL);

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
        edmaParam.aCnt          = (uint16_t) EDMA_QSPI_A_COUNT;
    }
    edmaParam.bCnt          = (uint16_t) (length/edmaParam.aCnt);
    edmaParam.cCnt          = (uint16_t) EDMA_QSPI_C_COUNT;
    edmaParam.bCntReload    = (uint16_t) (length/edmaParam.aCnt);
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(edmaParam.aCnt);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(edmaParam.aCnt);
    edmaParam.srcCIdx       = (int16_t) EDMA_QSPI_A_COUNT;
    edmaParam.destCIdx      = (int16_t) EDMA_QSPI_A_COUNT;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(edmaParam.aCnt);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(edmaParam.aCnt);
    edmaParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
         (((tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(baseAddr, param, &edmaParam);

    /* Set manual trigger to start QSPI transfer */
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
        edmaParam.aCnt      = (uint16_t) EDMA_QSPI_A_COUNT;
        edmaParam.bCnt      = (uint16_t) (length % MAX_EDMA_COUNT);
        edmaParam.srcBIdx   = (int16_t) EDMA_PARAM_BIDX(EDMA_QSPI_A_COUNT);
        edmaParam.destBIdx  = (int16_t) EDMA_PARAM_BIDX(EDMA_QSPI_A_COUNT);
        edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_QSPI_A_COUNT);
        edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_QSPI_A_COUNT);

        EDMA_setPaRAM(baseAddr, param, &edmaParam);

        /* Set manual trigger to start QSPI transfer */
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

    CacheP_inv(dst, length*EDMA_QSPI_A_COUNT, CacheP_TYPE_ALL);

    return status;
}

static void OSPI_edmaIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);
    SemaphoreP_post(semObjPtr);
}
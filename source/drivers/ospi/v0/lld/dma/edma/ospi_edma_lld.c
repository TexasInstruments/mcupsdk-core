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

#include <drivers/edma.h>
#include <drivers/edma/v0/edma.h>
#include <drivers/ospi.h>
#include <drivers/ospi/v0/lld/dma/edma/ospi_edma_lld.h>
#include <drivers/ospi/v0/lld/dma/ospi_lld_dma.h>
#include <drivers/soc.h>
#include <kernel/dpl/CacheP.h>

/* Value for A count*/
#define EDMA_OSPI_A_COUNT           (1U)
/* Max Value for EDMA count */
#define MAX_EDMA_COUNT              (31*1024U)
/* Value for C count */
#define EDMA_OSPI_C_COUNT           (1U)
/* Event queue to be used  */
#define EDMA_OSPI_EVT_QUEUE_NO      (0U)


static void OSPI_edmaIsrFxn(Edma_IntrHandle intrHandle, void *args);
void OSPI_EdmaParams_init(void * pvEdmaParams);

void OSPI_EdmaParams_init(void * pvEdmaParams)
{
    if( pvEdmaParams != NULL)
    {
        OspiDma_EdmaArgs *edmaParams = (OspiDma_EdmaArgs *)pvEdmaParams;
        edmaParams->edmaTcc = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaChId = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaChainChId = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaParam = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaChainParam = EDMA_RESOURCE_ALLOC_ANY;
        edmaParams->edmaRegionId = 0U;
        edmaParams->edmaBaseAddr = 0U;
        edmaParams->isIntEnabled = 0U;
    }
}

int32_t OSPI_dmaOpen(OSPILLD_Handle hOspi)
{

    uint32_t baseAddr, regionId, dmaCh, tcc, param,dmaChainCh, chainParam;
    int32_t status = SystemP_FAILURE;
    uint32_t isEdmaInterruptEnabled;

    OspiDma_EdmaArgs *edmaParams = (OspiDma_EdmaArgs*)hOspi->hOspiInit->ospiDmaChConfig;
    Edma_IntrObject *edmaIntrObject = &(edmaParams->edmaIntrObj);
    EDMA_Handle ospiEdmaHandle = EDMA_getHandle(edmaParams->edmaInst);

    if (ospiEdmaHandle != NULL)
    {
        OSPI_EdmaParams_init(edmaParams);
        /* Read base address of allocated EDMA instance */
        baseAddr = EDMA_getBaseAddr(ospiEdmaHandle);

        if(baseAddr != 0U)
        {
            status = SystemP_SUCCESS;

            /* Check if interrupt is enabled */
            isEdmaInterruptEnabled = EDMA_isInterruptEnabled(ospiEdmaHandle);
            /* Read the region ID of the EDMA instance */
            regionId = EDMA_getRegionId(ospiEdmaHandle);

            if(regionId < SOC_EDMA_NUM_REGIONS)
            {
                /* Allocate EDMA channel for OSPI transfer */
                dmaCh = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocDmaChannel(ospiEdmaHandle, &dmaCh);

                /* Allocate EDMA chained channel for QSPI transfer */
                dmaChainCh = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocDmaChannel(ospiEdmaHandle, &dmaChainCh);

                /* Allocate EDMA TCC for OSPI transfer */
                tcc = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocTcc(ospiEdmaHandle, &tcc);

                /* Allocate a Param ID for OSPI transfer */
                param = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocParam(ospiEdmaHandle, &param);

                /* Allocate a Param ID for chained channel in OSPI transfer */
                chainParam = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocParam(ospiEdmaHandle, &chainParam);

                if(status == SystemP_SUCCESS)
                {
                    status += EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                        dmaCh, tcc, param, EDMA_OSPI_EVT_QUEUE_NO);

                    if(isEdmaInterruptEnabled == TRUE)
                    {
                        /* Register interrupt */
                        edmaIntrObject->tccNum = tcc;
                        edmaIntrObject->cbFxn  = &OSPI_edmaIsrFxn;
                        edmaIntrObject->appData = (void *) hOspi;
                        status = EDMA_registerIntr(ospiEdmaHandle, edmaIntrObject);
                    }

                    /* Store the EDMA paramters and handle*/
                    edmaParams->edmaBaseAddr = baseAddr;
                    edmaParams->edmaRegionId = regionId;
                    edmaParams->edmaParam = param;
                    edmaParams->edmaChainParam = chainParam;
                    edmaParams->edmaChId = dmaCh;
                    edmaParams->edmaChainChId = dmaChainCh;
                    edmaParams->edmaTcc = tcc;
                    edmaParams->isIntEnabled = isEdmaInterruptEnabled;
                }
                if(status != SystemP_SUCCESS)
                {
                    if(dmaCh != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        status += EDMA_freeDmaChannel(ospiEdmaHandle, &dmaCh);
                    }
                    if(dmaChainCh != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        status += EDMA_freeDmaChannel(ospiEdmaHandle, &dmaChainCh);
                    }
                    if(tcc != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        status += EDMA_freeTcc(ospiEdmaHandle, &tcc);
                    }
                    if(param != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        status += EDMA_freeParam(ospiEdmaHandle, &param);
                    }
                    if(chainParam != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        status += EDMA_freeParam(ospiEdmaHandle, &chainParam);
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

int32_t OSPI_dmaClose(OSPILLD_Handle hOspi)
{
    int32_t             status = SystemP_SUCCESS;
    int32_t             retVal;
    uint32_t            baseAddr, regionId, dmaCh, tcc, param,dmaChainCh, chainParam;

    OspiDma_EdmaArgs *edmaParams = (OspiDma_EdmaArgs*)hOspi->hOspiInit->ospiDmaChConfig;
    
    EDMA_Handle ospiEdmaHandle = EDMA_getHandle(edmaParams->edmaInst);

    /* Fetch the EDMA paramters */
    baseAddr    = edmaParams->edmaBaseAddr;
    regionId    = edmaParams->edmaRegionId;
    dmaCh       = edmaParams->edmaChId;
    dmaChainCh  = edmaParams->edmaChainChId;
    tcc         = edmaParams->edmaTcc;
    param       = edmaParams->edmaParam;
    chainParam  = edmaParams->edmaChainParam;

    /* Free channel */
    retVal = EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_OSPI_EVT_QUEUE_NO);

    status = (retVal == (uint32_t)TRUE) ? SystemP_SUCCESS : SystemP_FAILURE;
    
    if(edmaParams->isIntEnabled == TRUE)
    {
        status = EDMA_unregisterIntr(ospiEdmaHandle, &edmaParams->edmaIntrObj);
    }
    /* Free the EDMA resources managed by driver. */
    status += EDMA_freeDmaChannel(ospiEdmaHandle, &dmaCh);
    status += EDMA_freeDmaChannel(ospiEdmaHandle, &dmaChainCh);
    status += EDMA_freeTcc(ospiEdmaHandle, &tcc);
    status += EDMA_freeParam(ospiEdmaHandle, &param);
    status += EDMA_freeParam(ospiEdmaHandle, &chainParam);

    return status;
}

int32_t OSPI_dmaCopy(OSPILLD_Handle hOspi, void* dst, void* src, uint32_t length,uint32_t timeout)
{
    int32_t         status = SystemP_SUCCESS;
    uint32_t        baseAddr, regionId, dmaCh, tcc, param,dmaChainCh, chainParam, chainOptions;
    uint32_t        edmaStatus;
    uint32_t        startTicks, elapsedTicks = 0;
    OspiDma_EdmaArgs *edmaParams = (OspiDma_EdmaArgs*)hOspi->hOspiInit->ospiDmaChConfig;
    uint32_t transTimeout = hOspi->Clock_usecToTicks(timeout);

    EDMACCPaRAMEntry   edmaParam;
    /* Fetch the EDMA paramters for QSPI transfer */
    baseAddr     = edmaParams->edmaBaseAddr;
    regionId     = edmaParams->edmaRegionId;
    dmaCh        = edmaParams->edmaChId;
    dmaChainCh   = edmaParams->edmaChainChId;
    tcc          = edmaParams->edmaTcc;
    param        = edmaParams->edmaParam;
    chainParam   = edmaParams->edmaChainParam;

    CacheP_wb(src, length*EDMA_OSPI_A_COUNT, CacheP_TYPE_ALL);
    CacheP_wb(dst, length*EDMA_OSPI_A_COUNT, CacheP_TYPE_ALL);

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
        edmaParam.aCnt          = (uint16_t) EDMA_OSPI_A_COUNT;
    }
    edmaParam.bCnt          = (uint16_t) (length/edmaParam.aCnt);
    edmaParam.cCnt          = (uint16_t) EDMA_OSPI_C_COUNT;
    edmaParam.bCntReload    = (uint16_t) (length/edmaParam.aCnt);
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(edmaParam.aCnt);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(edmaParam.aCnt);
    edmaParam.srcCIdx       = (int16_t) EDMA_OSPI_A_COUNT;
    edmaParam.destCIdx      = (int16_t) EDMA_OSPI_A_COUNT;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(edmaParam.aCnt);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(edmaParam.aCnt);

    /*
     * Check if transfer length is less than maximum EDMA transfer count or if it's a
     * multiple of maximum transfer length in which case, chaining is not required and this
     * channel is enough to complete the transfer.
     * If not, chaining has to be enabled and interrupt is not enabled for this channel
     * Interrupt is enabled only for the chained channel to signal completion.
     */
    if(length <= MAX_EDMA_COUNT)
    {
        edmaParam.opt      |=  (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
            (((tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    }
    else
    {
        edmaParam.opt      |=
        (EDMA_OPT_SYNCDIM_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    }

    EDMA_setPaRAM(baseAddr, param, &edmaParam);

    hOspi->currTrans->dataLen = length;
    hOspi->currTrans->buf = dst;

    if (length <= MAX_EDMA_COUNT)
    {
        /* Set manual trigger to start QSPI transfer */
        edmaStatus = EDMA_enableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);

        if (edmaStatus == TRUE)
        {
            if (edmaParams->isIntEnabled != TRUE)
            {
                startTicks = hOspi->Clock_getTicks();
                /* Poll for transfer completion */
                while ((EDMA_readIntrStatusRegion(baseAddr, regionId, tcc) != 1U) && (elapsedTicks < transTimeout))
                {
                    elapsedTicks = hOspi->Clock_getTicks() - startTicks;
                }

                EDMA_clrIntrRegion(baseAddr, regionId, tcc);
                CacheP_inv(dst, length*EDMA_OSPI_A_COUNT, CacheP_TYPE_ALL);
            }
        }
    }
    else
    {
        edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy(src) + (MAX_EDMA_COUNT * (length / MAX_EDMA_COUNT));
        edmaParam.destAddr      = (uint32_t) SOC_virtToPhy(dst) + (MAX_EDMA_COUNT * (length / MAX_EDMA_COUNT));
        edmaParam.aCnt      = (uint16_t) EDMA_OSPI_A_COUNT;
        edmaParam.bCnt      = (uint16_t) (length % MAX_EDMA_COUNT);
        edmaParam.srcBIdx   = (int16_t) EDMA_OSPI_A_COUNT;
        edmaParam.destBIdx  = (int16_t) EDMA_OSPI_A_COUNT;
        edmaParam.opt      |=  (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
            (((tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));    // Working in polled

        EDMA_setPaRAM(baseAddr, chainParam, &edmaParam);

        chainOptions = (EDMA_OPT_TCCHEN_MASK |
                             EDMA_OPT_ITCCHEN_MASK);

        EDMA_chainChannel(baseAddr, param, dmaChainCh, chainOptions);

        /* Set manual trigger to start QSPI transfer */
        edmaStatus = EDMA_enableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);

        if (edmaStatus == TRUE)
        {
            if(edmaParams->isIntEnabled != TRUE)
            {
                EDMA_disableEvtIntrRegion(baseAddr, regionId, dmaCh);
                startTicks = hOspi->Clock_getTicks();
                /* Poll for transfer completion */
                while((EDMA_readIntrStatusRegion(baseAddr, regionId, tcc) != 1U) && (elapsedTicks < transTimeout))
                {
                    elapsedTicks =  hOspi->Clock_getTicks()  - startTicks;
                }

                EDMA_clrIntrRegion(baseAddr, regionId, tcc);
                CacheP_inv(dst, length*EDMA_OSPI_A_COUNT, CacheP_TYPE_ALL);
            }
        }
    }
    return status;
}



static void OSPI_edmaIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    
    OSPILLD_Handle ospiHdl  = (OSPILLD_Handle) args;

    CacheP_inv(ospiHdl->currTrans->buf, ospiHdl->currTrans->dataLen, CacheP_TYPE_ALL);

    /* Need to update the above in future */
    OSPI_lld_readCompleteCallback((OSPILLD_Handle)args);
}

int32_t OSPI_isDmaInterruptEnabled(OSPILLD_Handle hOspi)
{    
    OspiDma_EdmaArgs *edmaParams = (OspiDma_EdmaArgs*)hOspi->hOspiInit->ospiDmaChConfig;

    return (int32_t)edmaParams->isIntEnabled;
}
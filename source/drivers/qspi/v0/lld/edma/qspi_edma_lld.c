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
 *  \file qspi_edma.c
 *
 *  \brief File containing EDMA Driver APIs implementation for QSPI.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/qspi/v0/lld/edma/qspi_edma_lld.h>
#include <drivers/soc.h>
#include <kernel/dpl/CacheP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Value for A count*/
#define EDMA_QSPI_A_COUNT           (1U)
/* Max Value for EDMA count - 31KB */
#define MAX_EDMA_COUNT              (31744U)
/* Value for C count */
#define EDMA_QSPI_C_COUNT           (1U)
/* Event queue to be used  */
#define EDMA_QSPI_EVT_QUEUE_NO      (0U)

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static void QSPI_edmaIsrFxn(Edma_IntrHandle intrHandle, void *args);
static void QSPI_EdmaParams_init(QSPI_EdmaParams *edmaParams);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void QSPI_EdmaParams_init(QSPI_EdmaParams *edmaParams)
{
    if( edmaParams != NULL)
    {
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

int32_t QSPI_edmaChannelConfig(QSPILLD_Handle hQspi)
{
    uint32_t            baseAddr, regionId, dmaCh, dmaChainCh, tcc, param, chainParam;
    int32_t             status = QSPI_SYSTEM_FAILURE;
    uint32_t            edmaStatus;
    uint32_t            isEdmaInterruptEnabled;
    QSPILLD_InitHandle  hQspiInit = hQspi->hQspiInit;
    QSPI_EdmaParams     *edmaParams = hQspiInit->qspiDmaChConfig;
    Edma_IntrObject     *edmaIntrObject = &edmaParams->edmaIntrObj;
    EDMA_Handle         qspiEdmaHandle = hQspiInit->qspiDmaHandle;

    if (qspiEdmaHandle != NULL)
    {
        QSPI_EdmaParams_init(edmaParams);
        /* Read base address of allocated EDMA instance */
        baseAddr = EDMA_getBaseAddr(qspiEdmaHandle);

        if(baseAddr != 0U)
        {
            status = QSPI_SYSTEM_SUCCESS;

            /* Check if interrupt is enabled */
            isEdmaInterruptEnabled = EDMA_isInterruptEnabled(qspiEdmaHandle);
            /* Read the region ID of the EDMA instance */
            regionId = EDMA_getRegionId(qspiEdmaHandle);

            if(regionId < SOC_EDMA_NUM_REGIONS)
            {
                /* Allocate EDMA channel for QSPI transfer */
                dmaCh = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocDmaChannel(qspiEdmaHandle, &dmaCh);

                /* Allocate EDMA chained channel for QSPI transfer */
                dmaChainCh = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocDmaChannel(qspiEdmaHandle, &dmaChainCh);

                /* Allocate EDMA TCC for QSPI transfer */
                tcc = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocTcc(qspiEdmaHandle, &tcc);

                /* Allocate a Param ID for QSPI transfer */
                param = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocParam(qspiEdmaHandle, &param);

                /* Allocate a Param ID for chained channel in QSPI transfer */
                chainParam = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocParam(qspiEdmaHandle, &chainParam);

                if(status == QSPI_SYSTEM_SUCCESS)
                {
                    edmaStatus = EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                        dmaCh, tcc, param, EDMA_QSPI_EVT_QUEUE_NO);

                    if(edmaStatus == TRUE)
                    {
                        edmaStatus = EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                            dmaChainCh, tcc, chainParam, EDMA_QSPI_EVT_QUEUE_NO);
                    }
                    if(edmaStatus != TRUE )
                    {
                        status = QSPI_SYSTEM_FAILURE;
                    }

                    if(isEdmaInterruptEnabled == TRUE)
                    {

                        /* Register interrupt */
                        edmaIntrObject->tccNum = tcc;
                        edmaIntrObject->cbFxn  = &QSPI_edmaIsrFxn;
                        edmaIntrObject->appData = (void *) hQspi;
                        status += EDMA_registerIntr(qspiEdmaHandle, edmaIntrObject);
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
                if(status != QSPI_SYSTEM_SUCCESS)
                {
                    if(dmaCh != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        status += EDMA_freeDmaChannel(qspiEdmaHandle, &dmaCh);
                    }
                    if(dmaChainCh != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        status += EDMA_freeDmaChannel(qspiEdmaHandle, &dmaChainCh);
                    }
                    if(tcc != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        status += EDMA_freeTcc(qspiEdmaHandle, &tcc);
                    }
                    if(param != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        status += EDMA_freeParam(qspiEdmaHandle, &param);
                    }
                    if(param != EDMA_RESOURCE_ALLOC_ANY)
                    {
                        status += EDMA_freeParam(qspiEdmaHandle, &chainParam);
                    }
                }
            }
            else
            {
                status = QSPI_SYSTEM_FAILURE;
            }
        }
    }

    return status;
}

void QSPI_edmaTransfer(void* dst, void* src, uint32_t length,
                       QSPILLD_Handle hQspi, uint32_t timeout)
{
    uint32_t            baseAddr, regionId, dmaCh, tcc, param, chainParam, chainOptions,dmaChainCh;
    QSPI_EdmaParams     *edmaParams = hQspi->hQspiInit->qspiDmaChConfig;
    EDMACCPaRAMEntry   edmaParam;
    uint32_t            edmaStatus;
    uint32_t startTicks, elapsedTicks = 0;
    QSPILLD_InitHandle hQspiInit = hQspi->hQspiInit;

    /* Fetch the EDMA paramters for QSPI transfer */
    baseAddr      = edmaParams->edmaBaseAddr;
    regionId      = edmaParams->edmaRegionId;
    dmaCh         = edmaParams->edmaChId;
    dmaChainCh    = edmaParams->edmaChainChId;
    tcc           = edmaParams->edmaTcc;
    param         = edmaParams->edmaParam;
    chainParam    = edmaParams->edmaChainParam;

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
    edmaParam.srcBIdx       = (int16_t) edmaParam.aCnt;
    edmaParam.destBIdx      = (int16_t) edmaParam.aCnt;
    edmaParam.srcCIdx       = (int16_t) EDMA_QSPI_A_COUNT;
    edmaParam.destCIdx      = (int16_t) EDMA_QSPI_A_COUNT;
    edmaParam.linkAddr      = 0xFFFFU;

    hQspi->transaction = &(hQspi->trans);
    hQspi->transaction->dataLen = length;
    hQspi->transaction->buf = dst;
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

    if(length <= MAX_EDMA_COUNT)
    {
        /* Set manual trigger to start QSPI transfer */
        edmaStatus = EDMA_enableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);

        if (edmaStatus == TRUE)
        {
            if(edmaParams->isIntEnabled != TRUE)
            {
                startTicks = hQspiInit->Clock_getTicks();
                /* Poll for transfer completion */
                while((EDMA_readIntrStatusRegion(baseAddr, regionId, tcc) != 1U) && (elapsedTicks < timeout))
                {
                    elapsedTicks = hQspiInit->Clock_getTicks() - startTicks;
                }

                EDMA_clrIntrRegion(baseAddr, regionId, tcc);
                CacheP_inv(dst, length*EDMA_QSPI_A_COUNT, CacheP_TYPE_ALL);
            }
        }
    }
    else
    {
        edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy(src) + (MAX_EDMA_COUNT * (length / MAX_EDMA_COUNT));
        edmaParam.destAddr      = (uint32_t) SOC_virtToPhy(dst) + (MAX_EDMA_COUNT * (length / MAX_EDMA_COUNT));
        edmaParam.aCnt      = (uint16_t) EDMA_QSPI_A_COUNT;
        edmaParam.bCnt      = (uint16_t) (length % MAX_EDMA_COUNT);
        edmaParam.srcBIdx   = (int16_t) EDMA_QSPI_A_COUNT;
        edmaParam.destBIdx  = (int16_t) EDMA_QSPI_A_COUNT;
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
                startTicks = hQspiInit->Clock_getTicks();
                /* Poll for transfer completion */
                while((EDMA_readIntrStatusRegion(baseAddr, regionId, tcc) != 1U) && (elapsedTicks < timeout))
                {
                    elapsedTicks =  hQspiInit->Clock_getTicks()  - startTicks;
                }

                EDMA_clrIntrRegion(baseAddr, regionId, tcc);

                CacheP_inv(dst, length*EDMA_QSPI_A_COUNT, CacheP_TYPE_ALL);
            }
        }
    }
}

int32_t QSPI_edmaChannelFree(QSPILLD_Handle hQspi)
{
    int32_t             status = QSPI_SYSTEM_SUCCESS;
    uint32_t            edmaStatus;
    uint32_t            baseAddr, regionId, dmaCh, tcc, param;
    QSPILLD_InitHandle  hQspiInit = hQspi->hQspiInit;
    QSPI_EdmaParams     *edmaParams = hQspiInit->qspiDmaChConfig;

    /* Fetch the EDMA paramters */
    baseAddr = edmaParams->edmaBaseAddr;
    regionId = edmaParams->edmaRegionId;
    dmaCh    = edmaParams->edmaChId;
    tcc      = edmaParams->edmaTcc;
    param    = edmaParams->edmaParam;

    /* Free channel */
    edmaStatus = EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_QSPI_EVT_QUEUE_NO);

    if(edmaStatus == TRUE)
    {
        status = QSPI_SYSTEM_SUCCESS;
    }
    else
    {
        status = QSPI_SYSTEM_FAILURE;
    }

    if(edmaParams->isIntEnabled == TRUE)
    {
        status = EDMA_unregisterIntr(hQspiInit->qspiDmaHandle, &edmaParams->edmaIntrObj);
    }
    /* Free the EDMA resources managed by driver. */
    status += EDMA_freeDmaChannel(hQspiInit->qspiDmaHandle, &dmaCh);
    status += EDMA_freeTcc(hQspiInit->qspiDmaHandle, &tcc);
    status += EDMA_freeParam(hQspiInit->qspiDmaHandle, &param);

    return status;
}

static void QSPI_edmaIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    QSPILLD_Handle pQspiHdl  = (QSPILLD_Handle) args;

    CacheP_inv(pQspiHdl->transaction->buf, pQspiHdl->transaction->dataLen, CacheP_TYPE_ALL);
    /*  */
    QSPI_lld_readCompleteCallback((QSPILLD_Handle)args);
}
/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 *  \file mcspi_dma_edma.c
 *
 *  \brief File containing EDMA Driver APIs implementation for McSPI.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <drivers/soc.h>
#include <drivers/mcspi/v0/lld/mcspi_lld.h>
#include <drivers/mcspi/v0/lld/dma/edma/mcspi_dma_edma.h>
#include <drivers/mcspi/v0/lld/dma/mcspi_dma.h>
#include <kernel/dpl/CacheP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** \brief Transmit EDMA channel event queue number                           */
#define EDMA_MCSPI_TX_EVT_QUEUE_NO                  (0U)
/** \brief Receive EDMA channel event queue number                            */
#define EDMA_MCSPI_RX_EVT_QUEUE_NO                  (1U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void MCSPI_edmaIsrTx(Edma_IntrHandle intrHandle, void *args);
static void MCSPI_edmaIsrRx(Edma_IntrHandle intrHandle, void *args);
static void MCSPI_dmaStart(MCSPILLD_Handle hMcspi, MCSPI_ChObject *chObj,
                           uint32_t baseAddr);
static void MCSPI_EdmaChConfig_init(MCSPI_EdmaChConfig *dmaChConfig);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t MCSPI_lld_dmaInit(MCSPI_DmaHandle mcspiDmaHandle)
{
    int32_t    status = MCSPI_STATUS_SUCCESS;
    uint32_t   isInit;

    if(mcspiDmaHandle != NULL)
    {
        /* Verify that EDMA initialization is complete */
        isInit = EDMA_isInitialized(mcspiDmaHandle);
        if(isInit != TRUE)
        {
            status = MCSPI_STATUS_SUCCESS;
        }
    }
    else
    {
        status = MCSPI_STATUS_FAILURE;
    }

    return status;
}

static void MCSPI_EdmaChConfig_init(MCSPI_EdmaChConfig *dmaChConfig)
{
    if( dmaChConfig != NULL)
    {
        dmaChConfig->edmaRegionId  = 0U;
        dmaChConfig->edmaBaseAddr  = 0U;
        dmaChConfig->edmaTccRx     = EDMA_RESOURCE_ALLOC_ANY;
        dmaChConfig->edmaTccTx     = EDMA_RESOURCE_ALLOC_ANY;
        dmaChConfig->edmaRxParam   = EDMA_RESOURCE_ALLOC_ANY;
        dmaChConfig->edmaTxParam   = EDMA_RESOURCE_ALLOC_ANY;
    }
}

int32_t MCSPI_lld_dmaChInit(MCSPILLD_Handle hMcspi, uint32_t chCnt)
{
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh,dmaTxCh, tccRx, tccTx, paramDummy, paramRx, paramTx;
    int32_t             status = MCSPI_STATUS_FAILURE, edmaStatus = SystemP_SUCCESS;
    uint32_t            isEdmaInterruptEnabled;
    MCSPI_ChObject      *chObj;
    MCSPI_EdmaChConfig  *dmaChConfig;
    Edma_IntrObject     *edmaIntrObjectRx;
    Edma_IntrObject     *edmaIntrObjectTx;
    EDMA_Handle         mcspiEdmaHandle;
    MCSPILLD_InitHandle hMcspiInit = hMcspi->hMcspiInit;
    uint32_t            retVal = (uint32_t)FALSE;

    chObj             = &hMcspiInit->chObj[chCnt];
    dmaChConfig       = (MCSPI_EdmaChConfig *)chObj->dmaChCfg;
    dmaChConfig       = &(dmaChConfig[chObj->dmaChConfigNum]);
    mcspiEdmaHandle   = (EDMA_Handle) (hMcspiInit->mcspiDmaHandle);
    edmaIntrObjectRx  = &dmaChConfig->edmaIntrObjRx;
    edmaIntrObjectTx  = &dmaChConfig->edmaIntrObjTx;

    MCSPI_EdmaChConfig_init(dmaChConfig);

    if(mcspiEdmaHandle != NULL)
    {
        status = MCSPI_STATUS_SUCCESS;

        /* Read base address of allocated EDMA instance */
        baseAddr = EDMA_getBaseAddr(mcspiEdmaHandle);

        /* Read the region ID of the EDMA instance */
        regionId = EDMA_getRegionId(mcspiEdmaHandle);

        /* Store the EDMA parameters */
        dmaChConfig->edmaBaseAddr = baseAddr;
        dmaChConfig->edmaRegionId = regionId;

        /* Check if interrupt is enabled */
        isEdmaInterruptEnabled = EDMA_isInterruptEnabled(mcspiEdmaHandle);

        if((baseAddr != 0U) && (regionId < SOC_EDMA_NUM_REGIONS) && (isEdmaInterruptEnabled == TRUE))
        {
            /*
             * Note: In TX RX mode, we expect TX Interrupt to be triggered first
             * and RX Interrupt next. To keep this flow, TX channel(TCC) should
             * be initialized first and then the RX channel(TCC).
             */
            if((MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode) || (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode))
            {
                /* Allocate EDMA channel for MCSPI TX transfer */
                dmaTxCh = dmaChConfig->edmaTxChId;
                edmaStatus += EDMA_allocDmaChannel(mcspiEdmaHandle, &dmaTxCh);

                /* Allocate EDMA TCC for MCSPI TX transfer */
                tccTx = EDMA_RESOURCE_ALLOC_ANY;
                edmaStatus += EDMA_allocTcc(mcspiEdmaHandle, &tccTx);

                /* Allocate a Param ID for MCSPI TX transfer */
                paramTx = EDMA_RESOURCE_ALLOC_ANY;
                edmaStatus += EDMA_allocParam(mcspiEdmaHandle, &paramTx);

                /* Allocate a Param ID for McSPI TX Dummy transfer */
                paramDummy = EDMA_RESOURCE_ALLOC_ANY;
                edmaStatus += EDMA_allocParam(mcspiEdmaHandle, &paramDummy);

                if(edmaStatus == MCSPI_STATUS_SUCCESS)
                {
                    retVal = EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                                                                  dmaTxCh, tccTx, paramTx, EDMA_MCSPI_TX_EVT_QUEUE_NO);
                    if(retVal == (uint32_t)TRUE)
                    {
                        edmaStatus += MCSPI_STATUS_SUCCESS;
                    }
                    else
                    {
                        edmaStatus += MCSPI_STATUS_FAILURE;
                    }
                    /* Register TX interrupt */
                    edmaIntrObjectTx->tccNum  = tccTx;
                    edmaIntrObjectTx->cbFxn   = &MCSPI_edmaIsrTx;
                    edmaIntrObjectTx->appData = (void *) hMcspi;
                    edmaStatus += EDMA_registerIntr(mcspiEdmaHandle, edmaIntrObjectTx);
                }

                if(edmaStatus == SystemP_SUCCESS)
                {
                    /* Store the EDMA parameters for McSPI TX*/
                    dmaChConfig->edmaTxParam    = paramTx;
                    dmaChConfig->edmaDummyParam = paramDummy;
                    dmaChConfig->edmaTxChId  = dmaTxCh;
                    dmaChConfig->edmaTccTx   = tccTx;
                }
            }

            if((MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode) || (MCSPI_TR_MODE_RX_ONLY == chObj->chCfg->trMode))
            {
                /* Allocate EDMA channel for MCSPI RX transfer */
                dmaRxCh = dmaChConfig->edmaRxChId;
                edmaStatus += EDMA_allocDmaChannel(mcspiEdmaHandle, &dmaRxCh);

                /* Allocate EDMA TCC for MCSPI RX transfer */
                tccRx = EDMA_RESOURCE_ALLOC_ANY;
                edmaStatus += EDMA_allocTcc(mcspiEdmaHandle, &tccRx);

                /* Allocate a Param ID for MCSPI RX transfer */
                paramRx = EDMA_RESOURCE_ALLOC_ANY;
                edmaStatus += EDMA_allocParam(mcspiEdmaHandle, &paramRx);

                if(edmaStatus == SystemP_SUCCESS)
                {
                    retVal = EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                                                                  dmaRxCh, tccRx, paramRx, EDMA_MCSPI_RX_EVT_QUEUE_NO);
                    if(retVal == (uint32_t)TRUE)
                    {
                        edmaStatus += SystemP_SUCCESS;
                    }
                    else
                    {
                        edmaStatus += SystemP_FAILURE;
                    }

                    /* Register RX interrupt */
                    edmaIntrObjectRx->tccNum = tccRx;
                    edmaIntrObjectRx->cbFxn  = &MCSPI_edmaIsrRx;
                    edmaIntrObjectRx->appData = (void *) hMcspi;
                    edmaStatus += EDMA_registerIntr(mcspiEdmaHandle, edmaIntrObjectRx);
                }

                if(edmaStatus == SystemP_SUCCESS)
                {
                    /* Store the EDMA parameters for McSPI RX*/
                    dmaChConfig->edmaRxParam = paramRx;
                    dmaChConfig->edmaRxChId  = dmaRxCh;
                    dmaChConfig->edmaTccRx   = tccRx;
                }
            }

            if(edmaStatus == SystemP_SUCCESS)
            {
                dmaChConfig->isOpen = TRUE;
                status = MCSPI_STATUS_SUCCESS;
            }
        }
        else
        {
            status = MCSPI_STATUS_FAILURE;
        }
    }
    return status;
}

int32_t MCSPI_lld_dmaDeInit(MCSPILLD_Handle hMcspi,
                            const MCSPI_ChConfig *chCfg,
                            uint32_t chCnt)
{
    int32_t              status     = MCSPI_STATUS_SUCCESS;
    int32_t              edmaStatus = SystemP_SUCCESS;
    MCSPI_ChObject      *chObj;
    MCSPI_EdmaChConfig  *dmaChConfig;
    EDMA_Handle          mcspiEdmaHandle;
    Edma_IntrObject     *edmaIntrObjectRx;
    Edma_IntrObject     *edmaIntrObjectTx;
    uint32_t             baseAddr, regionId;
    uint32_t             dmaRxCh,dmaTxCh, tccRx, tccTx, paramRx, paramTx;
    MCSPILLD_InitHandle  hMcspiInit;
    uint32_t             retVal = (uint32_t)FALSE;

    /* Check parameters */
    if((NULL == hMcspi) ||
       (NULL == chCfg) ||
       (NULL == hMcspi->hMcspiInit) ||
       (chCfg->chNum >= MCSPI_MAX_NUM_CHANNELS))
    {
        status = MCSPI_STATUS_FAILURE;
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        hMcspiInit       = hMcspi->hMcspiInit;
        chObj            = &hMcspiInit->chObj[chCnt];
        dmaChConfig      = (MCSPI_EdmaChConfig *)chObj->dmaChCfg;
        mcspiEdmaHandle  = (EDMA_Handle) (hMcspiInit->mcspiDmaHandle);

        /* Fetch the EDMA paramters */
        baseAddr         = dmaChConfig->edmaBaseAddr;
        regionId         = dmaChConfig->edmaRegionId;

        if (dmaChConfig->isOpen != FALSE)
        {
            if((MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode) || (MCSPI_TR_MODE_RX_ONLY == chObj->chCfg->trMode))
            {
                /* Fetch the EDMA paramters */
                dmaRxCh    = dmaChConfig->edmaRxChId;
                tccRx      = dmaChConfig->edmaTccRx;
                paramRx    = dmaChConfig->edmaRxParam;
                edmaIntrObjectRx = &dmaChConfig->edmaIntrObjRx;

                /* unregister Rx interrupt */
                edmaStatus = EDMA_unregisterIntr(mcspiEdmaHandle, edmaIntrObjectRx);

                /* Free Rx channel */
                retVal = EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                                      dmaRxCh, EDMA_TRIG_MODE_MANUAL, tccRx, EDMA_MCSPI_RX_EVT_QUEUE_NO);
                if(retVal == (uint32_t)TRUE)
                {
                    edmaStatus += SystemP_SUCCESS;
                }
                else
                {
                    edmaStatus += SystemP_FAILURE;
                }
                if(dmaRxCh != EDMA_RESOURCE_ALLOC_ANY)
                {
                    edmaStatus += EDMA_freeDmaChannel(mcspiEdmaHandle, &dmaRxCh);
                }
                if(tccRx != EDMA_RESOURCE_ALLOC_ANY)
                {
                    edmaStatus += EDMA_freeTcc(mcspiEdmaHandle, &tccRx);
                }
                if(paramRx != EDMA_RESOURCE_ALLOC_ANY)
                {
                    edmaStatus += EDMA_freeParam(mcspiEdmaHandle, &paramRx);
                }

            }

            if((MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode) || (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode))
            {
                /* Fetch the EDMA paramters */
                dmaTxCh    = dmaChConfig->edmaTxChId;
                tccTx      = dmaChConfig->edmaTccTx;
                paramTx    = dmaChConfig->edmaTxParam;
                edmaIntrObjectTx = &dmaChConfig->edmaIntrObjTx;

                /* unregister Tx interrupt */
                edmaStatus += EDMA_unregisterIntr(mcspiEdmaHandle, edmaIntrObjectTx);

                /* Free Tx channel */
                retVal = EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                                 dmaTxCh, EDMA_TRIG_MODE_MANUAL, tccTx, EDMA_MCSPI_TX_EVT_QUEUE_NO);
                if(retVal == (uint32_t)TRUE)
                {
                    edmaStatus = SystemP_SUCCESS;
                }
                else
                {
                    edmaStatus = SystemP_FAILURE;
                }

                if(dmaTxCh != EDMA_RESOURCE_ALLOC_ANY)
                {
                    edmaStatus += EDMA_freeDmaChannel(mcspiEdmaHandle, &dmaTxCh);
                }
                if(tccTx != EDMA_RESOURCE_ALLOC_ANY)
                {
                    edmaStatus += EDMA_freeTcc(mcspiEdmaHandle, &tccTx);
                }
                if(paramTx != EDMA_RESOURCE_ALLOC_ANY)
                {
                    edmaStatus += EDMA_freeParam(mcspiEdmaHandle, &paramTx);
                }
            }

            dmaChConfig->isOpen = FALSE;
        }
        if(edmaStatus == SystemP_SUCCESS)
        {
            status = MCSPI_STATUS_SUCCESS;
        }
        else
        {
            status = MCSPI_STATUS_FAILURE;
        }
    }

    return status;
}

int32_t MCSPI_lld_dmaTransfer(MCSPILLD_Handle hMcspi,
                              MCSPI_ChObject *chObj,
                              const MCSPI_Transaction *transaction)
{
    int32_t             status = MCSPI_STATUS_SUCCESS;
    int32_t             edmaStatus = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh, dmaTxCh, tccRx, tccTx, paramRx, paramTx, paramDummy;
    EDMACCPaRAMEntry    edmaTxParam, edmaRxParam, edmaDummyParam;
    MCSPI_EdmaChConfig *dmaChConfig;
    uint32_t            retVal = (uint32_t)FALSE;

    dmaChConfig = (MCSPI_EdmaChConfig *)chObj->dmaChCfg;

    /* Fetch the EDMA paramters for McSPI transfer */
    baseAddr  = dmaChConfig->edmaBaseAddr;
    regionId  = dmaChConfig->edmaRegionId;

    if((MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode) || (MCSPI_TR_MODE_RX_ONLY == chObj->chCfg->trMode))
    {
        /* Fetch the EDMA paramters for McSPI RX transfer */
        dmaRxCh    = dmaChConfig->edmaRxChId;
        tccRx      = dmaChConfig->edmaTccRx;
        paramRx    = dmaChConfig->edmaRxParam;

        /* Initialize RX Param Set */
        EDMA_ccPaRAMEntry_init(&edmaRxParam);

        /* Receive param set configuration */
        edmaRxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(hMcspi->baseAddr +
                                                              ((uint32_t)MCSPI_CHRX(chObj->chCfg->chNum))));
        edmaRxParam.destAddr      = (uint32_t) SOC_virtToPhy((void*) chObj->curRxBufPtr);
        edmaRxParam.aCnt          = (uint16_t) ((uint16_t)1U << chObj->bufWidthShift);
        edmaRxParam.bCnt          = (uint16_t) (transaction->count);
        edmaRxParam.cCnt          = (uint16_t) 1;
        edmaRxParam.bCntReload    = (uint16_t) edmaRxParam.bCnt;
        edmaRxParam.srcBIdx       = (int16_t) 0;
        edmaRxParam.destBIdx      = (int16_t) edmaRxParam.aCnt;
        edmaRxParam.srcCIdx       = (int16_t) 0;
        edmaRxParam.destCIdx      = (int16_t) 0;
        edmaRxParam.linkAddr      = 0xFFFFU;
        edmaRxParam.opt           =
            (EDMA_OPT_TCINTEN_MASK | ((tccRx << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

        /* Write Rx param set */
        EDMA_setPaRAM(baseAddr, paramRx, &edmaRxParam);

        /* Set event trigger to start McSPI RX transfer */
        retVal = (int32_t)EDMA_enableTransferRegion(baseAddr, regionId, dmaRxCh,
                                           EDMA_TRIG_MODE_EVENT);
        if(retVal == TRUE)
        {
            edmaStatus = MCSPI_STATUS_SUCCESS;
        }
        else
        {
            edmaStatus = MCSPI_STATUS_FAILURE;
        }
    }

    if((MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode) || (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode))
    {
        /* Fetch the EDMA paramters for McSPI TX transfer */
        dmaTxCh    = dmaChConfig->edmaTxChId;
        tccTx      = dmaChConfig->edmaTccTx;
        paramTx    = dmaChConfig->edmaTxParam;
        paramDummy = dmaChConfig->edmaDummyParam;

        /* Initialize TX Param Set */
        EDMA_ccPaRAMEntry_init(&edmaTxParam);

        /* Transmit param set configuration */
        edmaTxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *) chObj->curTxBufPtr);
        edmaTxParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)(hMcspi->baseAddr +
                                                              MCSPI_CHTX(chObj->chCfg->chNum)));
        edmaTxParam.aCnt          = (uint16_t) ((uint16_t)1U << chObj->bufWidthShift);
        edmaTxParam.bCnt          = (uint16_t) (transaction->count);
        edmaTxParam.cCnt          = (uint16_t) 1;
        edmaTxParam.bCntReload    = (uint16_t) edmaTxParam.bCnt;
        edmaTxParam.srcBIdx       = (int16_t) edmaTxParam.aCnt;
        edmaTxParam.destBIdx      = (int16_t) 0;
        edmaTxParam.srcCIdx       = (int16_t) 0;
        edmaTxParam.destCIdx      = (int16_t) 0;
        edmaTxParam.linkAddr      = 0xFFFFU;
        edmaTxParam.opt           = 0;
        edmaTxParam.opt          |=
            (EDMA_OPT_TCINTEN_MASK | ((tccTx << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

        /* Write Tx param set */
        EDMA_setPaRAM(baseAddr, paramTx, &edmaTxParam);

        /* Initialize TX Param Set */
        EDMA_ccPaRAMEntry_init(&edmaDummyParam);

        /* Dummy param set configuration */
        edmaDummyParam.aCnt          = (uint16_t) 1;
        edmaDummyParam.linkAddr      = 0xFFFFU;

        /* Write Tx dummy param set */
        EDMA_setPaRAM(baseAddr, paramDummy, &edmaDummyParam);

        /* Link  dummy param ID */
        EDMA_linkChannel(baseAddr, paramTx, paramDummy);

        /* Set event trigger to start McSPI TX transfer */
        retVal = (int32_t)EDMA_enableTransferRegion(baseAddr, regionId, dmaTxCh,
                                            EDMA_TRIG_MODE_EVENT);
        if(retVal == TRUE)
        {
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    if(edmaStatus == SystemP_SUCCESS)
    {
        status = MCSPI_STATUS_SUCCESS;
    }
    else
    {
        status = MCSPI_STATUS_FAILURE;
    }

    /* Initiate Transfer */
    MCSPI_dmaStart(hMcspi, chObj, hMcspi->baseAddr);

    return status;
}

void MCSPI_lld_dmaStop(MCSPILLD_Handle hMcspi, MCSPI_ChObject *chObj, uint32_t chNum)
{
    uint32_t baseAddr;
    MCSPILLD_InitHandle hMcspiInit = hMcspi->hMcspiInit;

    baseAddr = hMcspi->baseAddr;
    if(MCSPI_MS_MODE_CONTROLLER == hMcspiInit->msMode)
    {
        /* Manual CS de-assert */
        if(MCSPI_CH_MODE_SINGLE == hMcspiInit->chMode)
        {
            if (chObj->csDisable == TRUE)
            {
                CSL_REG32_FINS(
                    baseAddr + MCSPI_CHCONF(chNum),
                    MCSPI_CH0CONF_FORCE,
                    CSL_MCSPI_CH0CONF_FORCE_DEASSERT);
                    chObj->csEnable = TRUE;
            }
        }
    }

    /* Disable channel */
    CSL_REG32_FINS(
        baseAddr + MCSPI_CHCTRL(chNum),
        MCSPI_CH0CTRL_EN,
        CSL_MCSPI_CH0CTRL_EN_NACT);

    if(MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode)
    {
        /* Disable DMA */
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_DISABLED);
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_DISABLED);
    }
    else if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
    {
        /* Disable DMA for TX only mode */
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_DISABLED);
    }
    else
    {
        /* Disable DMA for RX only mode */
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_DISABLED);
    }

    hMcspi->state = MCSPI_STATE_READY;

    return;
}

static void MCSPI_dmaStart(MCSPILLD_Handle hMcspi, MCSPI_ChObject *chObj, uint32_t baseAddr)
{

    uint32_t chNum = chObj->chCfg->chNum;

    /* Enable DMA */
    if (MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_ENABLED);
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_ENABLED);
    }
    else if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
    {
        /* Enable DMA for TX only mode */
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_ENABLED);
    }
    else
    {
        /* Enable DMA for RX only mode */
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_ENABLED);
    }

    /* Manual CS assert */
    if(MCSPI_CH_MODE_SINGLE == hMcspi->hMcspiInit->chMode)
    {
        if (chObj->csEnable == TRUE)
        {
            CSL_REG32_FINS(
                baseAddr + MCSPI_CHCONF(chNum),
                MCSPI_CH0CONF_FORCE,
                CSL_MCSPI_CH0CONF_FORCE_ASSERT);
            chObj->csEnable = FALSE;
        }
    }

    /* Enable channel */
    CSL_REG32_FINS(
        baseAddr + MCSPI_CHCTRL(chNum),
        MCSPI_CH0CTRL_EN,
        CSL_MCSPI_CH0CTRL_EN_ACT);

    /*
     * Note: Once the channel is enabled, DMA will trigger its transfer.
     */
}

static void MCSPI_edmaIsrTx(Edma_IntrHandle intrHandle, void *args)
{
    MCSPI_ChObject         *chObj;
    uint32_t                chNum;
    MCSPI_Transaction      *transaction;
    MCSPILLD_Handle         hMcspi;
    uint32_t                baseAddr;
    volatile uint32_t       chStat;
    MCSPILLD_InitHandle     hMcspiInit;
    uint32_t startTicks, elapsedTicks = 0, irqStatus = 0U;
    int32_t status  =       MCSPI_STATUS_SUCCESS;

    if((NULL != args) && (intrHandle != NULL))
    {
        hMcspi = (MCSPILLD_Handle)args;
        transaction = &hMcspi->transaction;
        chNum = transaction->channel;
        chObj = &hMcspi->hMcspiInit->chObj[chNum];
        baseAddr = hMcspi->baseAddr;
        hMcspiInit = hMcspi->hMcspiInit;
        startTicks = hMcspiInit->clockP_get();

        if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
        {
            do{
                    /* Wait for end of transfer. */
                    chStat = CSL_REG32_RD(baseAddr + MCSPI_CHSTAT(chNum));
                    elapsedTicks = hMcspiInit->clockP_get() - startTicks;
            }while (((chStat & CSL_MCSPI_CH0STAT_EOT_MASK) == 0U) && (elapsedTicks < transaction->timeout));

            /* Stop MCSPI Channel */
            MCSPI_lld_dmaStop(hMcspi, chObj, chNum);
            hMcspi->state = MCSPI_STATE_READY;

            irqStatus = CSL_REG32_RD(baseAddr + CSL_MCSPI_IRQSTATUS);
            if (((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_MASK << (4U * chNum))) != 0U) &&
                (hMcspiInit->msMode == MCSPI_MS_MODE_PERIPHERAL))
                {
                    status = MCSPI_TRANSFER_CANCELLED;
                    hMcspi->errorFlag |= MCSPI_ERROR_TX_UNDERFLOW;
                }

            if(hMcspi->errorFlag != 0U)
            {
                hMcspi->state = MCSPI_STATE_READY;
                hMcspi->hMcspiInit->errorCallbackFxn(hMcspi, status);
            }
            else
            {
                hMcspi->hMcspiInit->transferCallbackFxn(hMcspi, MCSPI_TRANSFER_COMPLETED);
            }
        }
    }
    return;
}

static void MCSPI_edmaIsrRx(Edma_IntrHandle intrHandle, void *args)
{
    MCSPI_ChObject       *chObj;
    uint32_t              chNum;
    MCSPI_Transaction    *transaction;
    MCSPILLD_Handle       hMcspi = NULL;
    int32_t               status = MCSPI_STATUS_SUCCESS;
    uint32_t              baseAddr, irqStatus = 0U;

    if((NULL != args) && (intrHandle != NULL))
    {
        hMcspi = (MCSPILLD_Handle)args;
        baseAddr = hMcspi->baseAddr;
        transaction = &hMcspi->transaction;
        chNum = transaction->channel;
        chObj = &hMcspi->hMcspiInit->chObj[chNum];

        if (MCSPI_TR_MODE_TX_ONLY != chObj->chCfg->trMode)
        {
            /* Stop MCSPI Channel */
            MCSPI_lld_dmaStop(hMcspi, chObj, chNum);
            hMcspi->state = MCSPI_STATE_READY;

            irqStatus = CSL_REG32_RD(baseAddr + CSL_MCSPI_IRQSTATUS);
            if ((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_MASK)) != 0U)
            {
                status = MCSPI_TRANSFER_CANCELLED;
                hMcspi->errorFlag |= MCSPI_ERROR_RX_OVERFLOW;
            }

            if (((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_MASK << (4U * chNum))) != 0U) &&
                ((hMcspi->hMcspiInit->msMode == MCSPI_MS_MODE_PERIPHERAL)))
            {
                status = MCSPI_TRANSFER_CANCELLED;
                hMcspi->errorFlag |= MCSPI_ERROR_TX_UNDERFLOW;
            }

            if(hMcspi->errorFlag != 0U)
            {
                hMcspi->hMcspiInit->errorCallbackFxn(hMcspi, status);
            }
            else
            {
                hMcspi->hMcspiInit->transferCallbackFxn(hMcspi, MCSPI_TRANSFER_COMPLETED);
            }
        }
    }
    return;
}

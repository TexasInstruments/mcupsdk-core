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
#include <drivers/edma.h>
#include <drivers/mcspi/v0/dma/mcspi_dma.h>
#include <drivers/mcspi.h>
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
static int32_t MCSPI_edmaOpen(void* mcspiDmaArgs);
static int32_t MCSPI_edmaChInit(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg,
                                const MCSPI_DmaChConfig *dmaChCfg);
static int32_t MCSPI_edmaClose(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg);
static int32_t MCSPI_edmaTransfer(MCSPI_Object *obj,
                                        MCSPI_ChObject *chObj,
                                        const MCSPI_Attrs *attrs,
                                        MCSPI_Transaction *transaction);
static void MCSPI_edmaIsrTx(Edma_IntrHandle intrHandle, void *args);
static void MCSPI_edmaIsrRx(Edma_IntrHandle intrHandle, void *args);
static int32_t MCSPI_edmaStop(MCSPI_Object *obj, const MCSPI_Attrs *attrs,
                              MCSPI_ChObject *chObj, uint32_t chNum);
static void MCSPI_edmaStart(MCSPI_ChObject *chObj, const MCSPI_Attrs *attrs,
                            uint32_t baseAddr);

/* EDMA Function Pointers */
MCSPI_DmaFxns gMcspiDmaEdmaFxns =
{
    .dmaOpenFxn = MCSPI_edmaOpen,
    .dmaCloseFxn = MCSPI_edmaClose,
    .dmaChInitFxn = MCSPI_edmaChInit,
    .dmaTransferMasterFxn = MCSPI_edmaTransfer,
    .dmaStopFxn = MCSPI_edmaStop,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static int32_t MCSPI_edmaOpen(void* mcspiDmaArgs)
{
    return SystemP_SUCCESS;
}

static void MCSPI_DmaChConfig_init(MCSPI_DmaChConfig *dmaChConfig)
{
    if( dmaChConfig != NULL)
    {
        dmaChConfig->edmaRegionId  = 0U;
        dmaChConfig->edmaBaseAddr  = 0U;
        dmaChConfig->edmaTccRx     = EDMA_RESOURCE_ALLOC_ANY;
        dmaChConfig->edmaTccTx     = EDMA_RESOURCE_ALLOC_ANY;
        dmaChConfig->edmaRxChId    = EDMA_RESOURCE_ALLOC_ANY;
        dmaChConfig->edmaTxChId    = EDMA_RESOURCE_ALLOC_ANY;
        dmaChConfig->edmaRxParam   = EDMA_RESOURCE_ALLOC_ANY;
        dmaChConfig->edmaTxParam   = EDMA_RESOURCE_ALLOC_ANY;
    }
}

static int32_t MCSPI_edmaChInit(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg,
                                const MCSPI_DmaChConfig *dmaChCfg)
{
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh,dmaTxCh, tccRx, tccTx, paramRx, paramTx;
    int32_t             status = SystemP_FAILURE;
    uint32_t            isEdmaInterruptEnabled;
    MCSPI_Config        *config;
    MCSPI_Object        *object;
    MCSPI_ChObject      *chObj;
    MCSPI_DmaConfig     *dmaConfig;
    MCSPI_DmaChConfig   *dmaChConfig;
    Edma_IntrObject     *edmaIntrObjectRx;
    Edma_IntrObject     *edmaIntrObjectTx;
    EDMA_Handle         mcspiEdmaHandle;
    McspiDma_EdmaArgs   *edmaArgs;

    config           = (MCSPI_Config *) handle;
    object           = config->object;
    chObj            = &object->chObj[chCfg->chNum];
    dmaChConfig      = &chObj->dmaChCfg;
    dmaConfig        = (MCSPI_DmaConfig *)object->mcspiDmaHandle;
    edmaArgs         = (McspiDma_EdmaArgs *)dmaConfig->mcspiDmaArgs;
    mcspiEdmaHandle  = (EDMA_Handle) (edmaArgs->drvHandle);
    edmaIntrObjectRx = &dmaChConfig->edmaIntrObjRx;
    edmaIntrObjectTx = &dmaChConfig->edmaIntrObjTx;

    MCSPI_DmaChConfig_init(dmaChConfig);

    if(mcspiEdmaHandle != NULL)
    {
        status = SystemP_SUCCESS;

        /* Read base address of allocated EDMA instance */
        baseAddr = EDMA_getBaseAddr(mcspiEdmaHandle);

        /* Read the region ID of the EDMA instance */
        regionId = EDMA_getRegionId(mcspiEdmaHandle);

        /* Store the EDMA parameters */
        dmaChConfig->edmaBaseAddr = baseAddr;
        dmaChConfig->edmaRegionId = regionId;

        /* Check if interrupt is enabled */
        isEdmaInterruptEnabled = EDMA_isInterruptEnabled(mcspiEdmaHandle);
        DebugP_assert(isEdmaInterruptEnabled == TRUE);

        if((baseAddr != 0) && (regionId < SOC_EDMA_NUM_REGIONS))
        {
            if((MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode) || (MCSPI_TR_MODE_RX_ONLY == chObj->chCfg.trMode))
            {
                /* Allocate EDMA channel for MCSPI RX transfer */
                dmaRxCh = dmaChCfg->edmaRxChId;
                status += EDMA_allocDmaChannel(mcspiEdmaHandle, &dmaRxCh);

                /* Allocate EDMA TCC for MCSPI RX transfer */
                tccRx = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocTcc(mcspiEdmaHandle, &tccRx);

                /* Allocate a Param ID for MCSPI RX transfer */
                paramRx = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocParam(mcspiEdmaHandle, &paramRx);

                if(status == SystemP_SUCCESS)
                {
                    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                    dmaRxCh, tccRx, paramRx, EDMA_MCSPI_RX_EVT_QUEUE_NO);

                    /* Register RX interrupt */
                    edmaIntrObjectRx->tccNum = tccRx;
                    edmaIntrObjectRx->cbFxn  = &MCSPI_edmaIsrRx;
                    edmaIntrObjectRx->appData = (void *) config;
                    status += EDMA_registerIntr(mcspiEdmaHandle, edmaIntrObjectRx);
                    DebugP_assert(status == SystemP_SUCCESS);
                }

                if(status == SystemP_SUCCESS)
                {
                    /* Store the EDMA parameters for McSPI RX*/
                    dmaChConfig->edmaRxParam = paramRx;
                    dmaChConfig->edmaRxChId  = dmaRxCh;
                    dmaChConfig->edmaTccRx   = tccRx;
                }
            }

            if((MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode) || (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg.trMode))
            {
                /* Allocate EDMA channel for MCSPI TX transfer */
                dmaTxCh = dmaChCfg->edmaTxChId;
                status += EDMA_allocDmaChannel(mcspiEdmaHandle, &dmaTxCh);

                /* Allocate EDMA TCC for MCSPI TX transfer */
                tccTx = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocTcc(mcspiEdmaHandle, &tccTx);

                /* Allocate a Param ID for MCSPI TX transfer */
                paramTx = EDMA_RESOURCE_ALLOC_ANY;
                status += EDMA_allocParam(mcspiEdmaHandle, &paramTx);

                if(status == SystemP_SUCCESS)
                {
                    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                    dmaTxCh, tccTx, paramTx, EDMA_MCSPI_TX_EVT_QUEUE_NO);

                    /* Register TX interrupt */
                    edmaIntrObjectTx->tccNum = tccTx;
                    edmaIntrObjectTx->cbFxn  = &MCSPI_edmaIsrTx;
                    edmaIntrObjectTx->appData = (void *) config;
                    status += EDMA_registerIntr(mcspiEdmaHandle, edmaIntrObjectTx);
                    DebugP_assert(status == SystemP_SUCCESS);
                }

                if(status == SystemP_SUCCESS)
                {
                    /* Store the EDMA parameters for McSPI TX*/
                    dmaChConfig->edmaTxParam = paramTx;
                    dmaChConfig->edmaTxChId  = dmaTxCh;
                    dmaChConfig->edmaTccTx   = tccTx;
                }
            }

            chObj->dmaChCfg.isOpen = TRUE;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    return status;
}

static int32_t MCSPI_edmaClose(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg)
{
    int32_t status = SystemP_SUCCESS;
    MCSPI_Config        *config;
    MCSPI_Object        *obj;
    MCSPI_ChObject      *chObj;
    MCSPI_DmaChConfig   *dmaChConfig;
    MCSPI_DmaConfig     *dmaConfig;
    McspiDma_EdmaArgs   *edmaArgs;
    EDMA_Handle         mcspiEdmaHandle;
    Edma_IntrObject     *edmaIntrObjectRx;
    Edma_IntrObject     *edmaIntrObjectTx;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh,dmaTxCh, tccRx, tccTx, paramRx, paramTx;

    /* Check parameters */
    if((NULL == handle) ||
       (NULL == chCfg) ||
       (chCfg->chNum >= MCSPI_MAX_NUM_CHANNELS))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config           = (MCSPI_Config *) handle;
        obj              = config->object;
        chObj            = &obj->chObj[chCfg->chNum];
        dmaChConfig      = &chObj->dmaChCfg;
        dmaConfig        = (MCSPI_DmaConfig *)obj->mcspiDmaHandle;
        edmaArgs         = (McspiDma_EdmaArgs *)dmaConfig->mcspiDmaArgs;
        mcspiEdmaHandle  = (EDMA_Handle) (edmaArgs->drvHandle);

        /* Fetch the EDMA paramters */
        baseAddr         = dmaChConfig->edmaBaseAddr;
        regionId         = dmaChConfig->edmaRegionId;

        if (chObj->dmaChCfg.isOpen != FALSE)
        {
            if((MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode) || (MCSPI_TR_MODE_RX_ONLY == chObj->chCfg.trMode))
            {
                /* Fetch the EDMA paramters */
                dmaRxCh    = dmaChConfig->edmaRxChId;
                tccRx      = dmaChConfig->edmaTccRx;
                paramRx    = dmaChConfig->edmaRxParam;
                edmaIntrObjectRx = &dmaChConfig->edmaIntrObjRx;

                /* unregister Rx interrupt */
                status += EDMA_unregisterIntr(mcspiEdmaHandle, edmaIntrObjectRx);

                /* Free Rx channel */
                EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                    dmaRxCh, EDMA_TRIG_MODE_MANUAL, tccRx, EDMA_MCSPI_RX_EVT_QUEUE_NO);

                if(dmaRxCh != EDMA_RESOURCE_ALLOC_ANY)
                {
                    EDMA_freeDmaChannel(mcspiEdmaHandle, &dmaRxCh);
                }
                if(tccRx != EDMA_RESOURCE_ALLOC_ANY)
                {
                    EDMA_freeTcc(mcspiEdmaHandle, &tccRx);
                }
                if(paramRx != EDMA_RESOURCE_ALLOC_ANY)
                {
                    EDMA_freeParam(mcspiEdmaHandle, &paramRx);
                }
            }

            if((MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode) || (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg.trMode))
            {
                /* Fetch the EDMA paramters */
                dmaTxCh    = dmaChConfig->edmaTxChId;
                tccTx      = dmaChConfig->edmaTccTx;
                paramTx    = dmaChConfig->edmaTxParam;
                edmaIntrObjectTx = &dmaChConfig->edmaIntrObjTx;

                /* unregister Tx interrupt */
                status += EDMA_unregisterIntr(mcspiEdmaHandle, edmaIntrObjectTx);

                /* Free Tx channel */
                EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                    dmaTxCh, EDMA_TRIG_MODE_MANUAL, tccTx, EDMA_MCSPI_TX_EVT_QUEUE_NO);

                if(dmaTxCh != EDMA_RESOURCE_ALLOC_ANY)
                {
                    EDMA_freeDmaChannel(mcspiEdmaHandle, &dmaTxCh);
                }
                if(tccTx != EDMA_RESOURCE_ALLOC_ANY)
                {
                    EDMA_freeTcc(mcspiEdmaHandle, &tccTx);
                }
                if(paramTx != EDMA_RESOURCE_ALLOC_ANY)
                {
                    EDMA_freeParam(mcspiEdmaHandle, &paramTx);
                }
            }

            chObj->dmaChCfg.isOpen = FALSE;
        }
    }

    return status;
}

static int32_t MCSPI_edmaTransfer(MCSPI_Object *obj,
                                MCSPI_ChObject *chObj,
                                const MCSPI_Attrs *attrs,
                                MCSPI_Transaction *transaction)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh, dmaTxCh, tccRx, tccTx, paramRx, paramTx;
    EDMACCPaRAMEntry    edmaTxParam, edmaRxParam;
    MCSPI_DmaChConfig   *dmaChConfig;

    dmaChConfig = &chObj->dmaChCfg;

    /* Fetch the EDMA paramters for McSPI transfer */
    baseAddr               = dmaChConfig->edmaBaseAddr;
    regionId               = dmaChConfig->edmaRegionId;

    if((MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode) || (MCSPI_TR_MODE_RX_ONLY == chObj->chCfg.trMode))
    {
        /* Fetch the EDMA paramters for McSPI RX transfer */
        dmaRxCh    = dmaChConfig->edmaRxChId;
        tccRx      = dmaChConfig->edmaTccRx;
        paramRx    = dmaChConfig->edmaRxParam;

        /* Initialize RX Param Set */
        EDMA_ccPaRAMEntry_init(&edmaRxParam);

        /* Receive param set configuration */
        edmaRxParam.srcAddr       = (uint32_t) SOC_virtToPhy((uint8_t *) attrs->baseAddr +
                                    MCSPI_CHRX((chObj->chCfg).chNum));
        edmaRxParam.destAddr      = (uint32_t) SOC_virtToPhy((void*) chObj->curRxBufPtr);
        edmaRxParam.aCnt          = (uint16_t) (1 << chObj->bufWidthShift);
        edmaRxParam.bCnt          = (uint16_t) (transaction->count);
        edmaRxParam.cCnt          = (uint16_t) 1;
        edmaRxParam.bCntReload    = (uint16_t) edmaRxParam.bCnt;
        edmaRxParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
        edmaRxParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(edmaRxParam.aCnt);
        edmaRxParam.srcCIdx       = (int16_t) 0;
        edmaRxParam.destCIdx      = (int16_t) 0;
        edmaRxParam.linkAddr      = 0xFFFFU;
        edmaRxParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
        edmaRxParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(edmaRxParam.aCnt);
        edmaRxParam.opt           =
            (EDMA_OPT_TCINTEN_MASK | ((tccRx << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

        /* Write Rx param set */
        EDMA_setPaRAM(baseAddr, paramRx, &edmaRxParam);

        /* Set event trigger to start McSPI RX transfer */
        EDMA_enableTransferRegion(baseAddr, regionId, dmaRxCh,
             EDMA_TRIG_MODE_EVENT);
    }

    if((MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode) || (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg.trMode))
    {
        /* Fetch the EDMA paramters for McSPI TX transfer */
        dmaTxCh    = dmaChConfig->edmaTxChId;
        tccTx      = dmaChConfig->edmaTccTx;
        paramTx    = dmaChConfig->edmaTxParam;

        /* Initialize TX Param Set */
        EDMA_ccPaRAMEntry_init(&edmaTxParam);

        /* Transmit param set configuration */
        edmaTxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void*) chObj->curTxBufPtr);
        edmaTxParam.destAddr      = (uint32_t) SOC_virtToPhy((uint8_t*) attrs->baseAddr +
                                    MCSPI_CHTX((chObj->chCfg).chNum));
        edmaTxParam.aCnt          = (uint16_t) (1 << chObj->bufWidthShift);
        edmaTxParam.bCnt          = (uint16_t) (transaction->count);
        edmaTxParam.cCnt          = (uint16_t) 1;
        edmaTxParam.bCntReload    = (uint16_t) edmaTxParam.bCnt;
        edmaTxParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(edmaTxParam.aCnt);
        edmaTxParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
        edmaTxParam.srcCIdx       = (int16_t) 0;
        edmaTxParam.destCIdx      = (int16_t) 0;
        edmaTxParam.linkAddr      = 0xFFFFU;
        edmaTxParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(edmaTxParam.aCnt);
        edmaTxParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
        edmaTxParam.opt           = 0;
        edmaTxParam.opt          |=
            (EDMA_OPT_TCINTEN_MASK | ((tccTx << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

        /* Write Tx param set */
        EDMA_setPaRAM(baseAddr, paramTx, &edmaTxParam);

        /* Set event trigger to start McSPI TX transfer */
        EDMA_enableTransferRegion(baseAddr, regionId, dmaTxCh,
             EDMA_TRIG_MODE_EVENT);
    }

    /* Initiate Transfer */
    MCSPI_edmaStart(chObj, attrs, obj->baseAddr);

    return status;
}

static int32_t MCSPI_edmaStop(MCSPI_Object *obj, const MCSPI_Attrs *attrs,
                              MCSPI_ChObject *chObj, uint32_t chNum)
{
    uint32_t baseAddr;
    int32_t status = SystemP_SUCCESS;

    baseAddr = obj->baseAddr;
    if(MCSPI_MS_MODE_MASTER == obj->openPrms.msMode)
    {
        /* Manual CS de-assert */
        if(MCSPI_CH_MODE_SINGLE == attrs->chMode)
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

    if(MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode)
    {
        /* Disable DMA */
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_DISABLED);
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_DISABLED);
    }
    else if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg.trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_DISABLED);
    }
    else if (MCSPI_TR_MODE_RX_ONLY == chObj->chCfg.trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_DISABLED);
    }

    return status;
}

static void MCSPI_edmaStart(MCSPI_ChObject *chObj, const MCSPI_Attrs *attrs,
                           uint32_t baseAddr)
{

    uint32_t chNum = chObj->chCfg.chNum;

    /* Enable DMA */
    if (MCSPI_TR_MODE_TX_RX == chObj->chCfg.trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_ENABLED);
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_ENABLED);
    }
    else if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg.trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAW, CSL_MCSPI_CH0CONF_DMAW_ENABLED);
    }
    else if (MCSPI_TR_MODE_RX_ONLY == chObj->chCfg.trMode)
    {
        CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_DMAR, CSL_MCSPI_CH0CONF_DMAR_ENABLED);
    }

    /* Manual CS assert */
    if(MCSPI_CH_MODE_SINGLE == attrs->chMode)
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
    MCSPI_Config       *config;
    MCSPI_Object       *obj;
    const MCSPI_Attrs  *attrs;
    MCSPI_ChObject     *chObj;
    uint32_t           chNum;
    MCSPI_Transaction  *transaction;

    if(NULL != args)
    {
        config = (MCSPI_Config *) args;
        obj = config->object;
        DebugP_assert(NULL != obj);
        DebugP_assert(NULL != config->attrs);
        attrs = config->attrs;
        transaction = obj->currTransaction;
        if (transaction != NULL)
        {
            chNum = transaction->channel;
            chObj = &obj->chObj[chNum];

            transaction->status = MCSPI_TRANSFER_COMPLETED;

            if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg.trMode)
            {
                /* Stop MCSPI Channel */
                MCSPI_edmaStop(obj, attrs, chObj, chNum);
                /* Update the driver internal status. */
                obj->currTransaction = NULL;
                /*
                * Post transfer Sem in case of blocking transfer.
                * Call the callback function in case of Callback mode.
                */
                if (obj->openPrms.transferMode == MCSPI_TRANSFER_MODE_BLOCKING)
                {
                    SemaphoreP_post(&obj->transferSemObj);
                }
                else
                {
                    obj->openPrms.transferCallbackFxn((MCSPI_Handle) config, transaction);
                }
            }
        }
    }
    return;
}

static void MCSPI_edmaIsrRx(Edma_IntrHandle intrHandle, void *args)
{
    MCSPI_Config       *config;
    MCSPI_Object       *obj;
    const MCSPI_Attrs  *attrs;
    MCSPI_ChObject     *chObj;
    uint32_t           chNum;
    MCSPI_Transaction  *transaction;

    if(NULL != args)
    {
        config = (MCSPI_Config *) args;
        obj = config->object;
        DebugP_assert(NULL != obj);
        DebugP_assert(NULL != config->attrs);
        attrs = config->attrs;
        transaction = obj->currTransaction;
        if (transaction != NULL)
        {
            chNum = transaction->channel;
            chObj = &obj->chObj[chNum];

            transaction->status = MCSPI_TRANSFER_COMPLETED;

            if (MCSPI_TR_MODE_TX_ONLY != chObj->chCfg.trMode)
            {
                /* Stop MCSPI Channel */
                MCSPI_edmaStop(obj, attrs, chObj, chNum);
                /* Update the driver internal status. */
                obj->currTransaction = NULL;
                /*
                * Post transfer Sem in case of blocking transfer.
                * Call the callback function in case of Callback mode.
                */
                if (obj->openPrms.transferMode == MCSPI_TRANSFER_MODE_BLOCKING)
                {
                    SemaphoreP_post(&obj->transferSemObj);
                }
                else
                {
                    obj->openPrms.transferCallbackFxn((MCSPI_Handle) config, transaction);
                }
            }
        }
    }
    return;
}

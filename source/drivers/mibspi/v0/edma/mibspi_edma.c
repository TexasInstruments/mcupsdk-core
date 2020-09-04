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
 *  \file mibspi_edma.c
 *
 *  \brief File containing EDMA Driver APIs implementation for MIBSPI.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <drivers/edma.h>
#include <drivers/mibspi/v0/edma/mibspi_edma.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** \brief Transmit EDMA channel event queue number                           */
#define MIBSPI_TXEVENTQUE                  (0U)
/** \brief Receive EDMA channel event queue number                            */
#define MIBSPI_RXEVENTQUE                  (1U)
/** \brief Invalide DMA reqline value                                         */
#define MIBSPI_INVALID_DMA_REQLINE          (uint8_t)0xFFU
/** \brief Macro to get the size of an array                                  */
#define MIBSPI_UTILS_ARRAYSIZE(x)  (sizeof(x) / sizeof(x[0]))

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void MIBSPI_edmaRegUpdateRxParams(const MIBSPI_Object            *ptrMibSpiDriver,
                                         const MIBSPI_DMAXferAddrInfo   *xferAddrInfo,
                                         const MIBSPI_DMAXferSizeInfo   *xferSizeInfo,
                                         uint32_t                        tcc,
                                         EDMACCPaRAMEntry              *rxParamSet);
static void MIBSPI_edmaRegUpdateTxParams(const MIBSPI_Object            *ptrMibSpiDriver,
                                         const MIBSPI_DMAXferAddrInfo   *xferAddrInfo,
                                         const MIBSPI_DMAXferSizeInfo   *xferSizeInfo,
                                         uint32_t                        tcc,
                                         EDMACCPaRAMEntry              *txParamSet);                                         
static void MIBSPI_edmaRamUpdateRxParams(const MIBSPI_Object            *ptrMibSpiDriver,
                                         const MIBSPI_DMAXferAddrInfo   *xferAddrInfo,
                                         const MIBSPI_DMAXferSizeInfo   *xferSizeInfo,
                                         uint32_t                       tcc,
                                         EDMACCPaRAMEntry              *rxParamSet);
static void MIBSPI_edmaRamUpdateTxParams(const MIBSPI_Object            *ptrMibSpiDriver,
                                         const MIBSPI_DMAXferAddrInfo   *xferAddrInfo,
                                         const MIBSPI_DMAXferSizeInfo   *xferSizeInfo,
                                         uint32_t                        tcc,
                                         EDMACCPaRAMEntry              *txParamSet);
static int32_t MIBSPI_edmaRegTransfer(MIBSPI_Object*    ptrMibSpiDriver,
                                      MIBSPI_DMAXferInfo *xferInfo);
static int32_t MIBSPI_edmaRamTransfer(MIBSPI_Object      *ptrMibSpiDriver,
                                      MIBSPI_DMAXferInfo *xferInfo);
static uint32_t MIBSPI_edmaMapTcc2DmaReqLine(const MIBSPI_Attrs *ptrHwCfg, 
                                             uint32_t tcc, 
                                             Bool isRxCh);
static void EDMA_txCompletionIsrFxn(Edma_IntrHandle intrHandle, void *args);
static void EDMA_rxCompletionIsrFxn(Edma_IntrHandle intrHandle, void *args);
static void SPIIntEnable(uint32_t baseAdd);
static void MIBSPI_edmaDoNothing(Edma_IntrHandle intrHandle, void *args);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t MIBSPI_edmaOpen(MIBSPI_Handle handle, uint32_t edmaInst)
{
    uint32_t               baseAddr, regionId;
    int32_t                spiStatus = SystemP_FAILURE;
    MIBSPI_Object         *object = ((MIBSPI_Config *)handle)->object;
    EDMA_Handle           *mibspiEdmaHandle = &object->params.dmaHandle;

    /* Open the EDMA channel */
    *mibspiEdmaHandle = EDMA_getHandle(edmaInst);

    /* Read base address of allocated EDMA instance */
    baseAddr = EDMA_getBaseAddr(*mibspiEdmaHandle);
    if(baseAddr != 0)
    {
        spiStatus = SystemP_SUCCESS;

        /* Read the region ID of the EDMA instance */
        regionId = EDMA_getRegionId(*mibspiEdmaHandle);

        if(regionId < SOC_EDMA_NUM_REGIONS)
        {
            /* Store the EDMA parameters */
            object->edmaBaseAddr = baseAddr;
            object->edmaRegionId = regionId;
        }
        else
        {
           spiStatus = SystemP_FAILURE;
        }
    }
    return spiStatus;
}

int32_t MIBSPI_edmaAllocChResource(MIBSPI_Handle handle, uint32_t dmaReqLine)
{
    uint32_t               baseAddr, regionId, dmaCh, tcc, param,dmaResourceRx,dmaResourceTx, edmaLinkParamId;
    int32_t                spiStatus = SystemP_SUCCESS;
    MIBSPI_Config         *ptrSPIConfig ;
    MIBSPI_Object         *object;
    const MIBSPI_Attrs    *hwAttrs ;
    MIBSPI_EDMAChParams   *edmaParamsTx, *edmaParamsRx ;
    EDMA_Handle            mibspiEdmaHandle;

    /* Get the SPI driver Configuration: */
    ptrSPIConfig = (MIBSPI_Config*)handle;

    object = (MIBSPI_Object*)ptrSPIConfig->object;

    mibspiEdmaHandle = object->params.dmaHandle;

    /* Get the pointer to the object and hwAttrs */
    hwAttrs = object->ptrHwCfg;

    edmaParamsTx = &object->edmaParamsTx[dmaReqLine];
    edmaParamsRx = &object->edmaParamsRx[dmaReqLine];

    /* Assign the dma channel resources */
    dmaResourceRx = hwAttrs->dmaReqlineCfg[dmaReqLine].rxDmaReqLine;
    dmaResourceTx = hwAttrs->dmaReqlineCfg[dmaReqLine].txDmaReqLine;

    /* Check EDMA handle is not NULL */
    if(mibspiEdmaHandle == NULL)
    {
        spiStatus = SystemP_FAILURE;
    }

    if (dmaReqLine >= hwAttrs->numDmaReqLines)
    {
        spiStatus = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == spiStatus)
    {
        /* Get the base address and region ID */
        baseAddr = object->edmaBaseAddr;
        regionId = object->edmaRegionId;

        if((baseAddr != 0) && (regionId < SOC_EDMA_NUM_REGIONS))
        {
            /* Allocate EDMA channel for MIBSPI TX  */
            dmaCh = dmaResourceTx;
            spiStatus += EDMA_allocDmaChannel(mibspiEdmaHandle, &dmaCh);

            /* Allocate EDMA TCC for MIBSPI TX */
            tcc =  dmaResourceTx;
            spiStatus += EDMA_allocTcc(mibspiEdmaHandle, &tcc);

            /* Allocate a Param ID for MIBSPI TX */
            param =  EDMA_RESOURCE_ALLOC_ANY;
            spiStatus += EDMA_allocParam(mibspiEdmaHandle, &param);

            if(spiStatus == SystemP_SUCCESS)
            {
                EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                     dmaCh, tcc, param, MIBSPI_TXEVENTQUE);

                /* Store the EDMA parameters and handle */
                edmaParamsTx->edmaParam = param;
                edmaParamsTx->edmaChId = dmaCh;
                edmaParamsTx->edmaTcc = tcc;
                
                /* Register transfer interrupt */
                object->intrTxObj.tccNum = tcc;
                object->intrTxObj.cbFxn  = &EDMA_txCompletionIsrFxn;
                object->intrTxObj.appData = (void *) object;
                spiStatus = EDMA_registerIntr(mibspiEdmaHandle, &object->intrTxObj);
                DebugP_assert(spiStatus == SystemP_SUCCESS);
            }

            if(spiStatus == SystemP_SUCCESS)
            {
                /* Allocate EDMA channel for MIBSPI RX  */
                dmaCh = dmaResourceRx;
                spiStatus += EDMA_allocDmaChannel(mibspiEdmaHandle, &dmaCh);

                /* Allocate EDMA TCC for MIBSPI RX */
                tcc =  dmaResourceRx;
                spiStatus += EDMA_allocTcc(mibspiEdmaHandle, &tcc);

                /* Allocate a Param ID for MIBSPI RX */
                param =  EDMA_RESOURCE_ALLOC_ANY;
                spiStatus += EDMA_allocParam(mibspiEdmaHandle, &param);

                if(spiStatus == SystemP_SUCCESS)
                {
                    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                         dmaCh, tcc, param, MIBSPI_RXEVENTQUE);

                    /* Store the EDMA parameters */
                    edmaParamsRx->edmaParam = param;
                    edmaParamsRx->edmaChId = dmaCh;
                    edmaParamsRx->edmaTcc = tcc;
                    
                    /* Register receive interrupt */
                    object->intrRxObj.tccNum = tcc;
                    object->intrRxObj.cbFxn  = &EDMA_rxCompletionIsrFxn;
                    object->intrRxObj.appData = (void *) object;
                    spiStatus = EDMA_registerIntr(mibspiEdmaHandle, &object->intrRxObj);
                    DebugP_assert(spiStatus == SystemP_SUCCESS);
                }
            }

            /* Allocate link param ID for SPI mode */
            if (object->params.compatibilityMode)
            {
                edmaLinkParamId = EDMA_RESOURCE_ALLOC_ANY;
                spiStatus = EDMA_allocParam(mibspiEdmaHandle, &edmaLinkParamId);
                object->params.edmaLinkParamId = edmaLinkParamId;
                
                /* Register dummy interrupt */
                object->intrDummyObj.tccNum = edmaLinkParamId;
                object->intrDummyObj.cbFxn  = &MIBSPI_edmaDoNothing;
                object->intrDummyObj.appData = (void *) NULL;
                spiStatus = EDMA_registerIntr(mibspiEdmaHandle, &object->intrDummyObj);
            }
        }
    }

    return(spiStatus);
}

int32_t MIBSPI_edmaTransfer(MIBSPI_Handle handle, MIBSPI_DMAXferInfo *xferInfo)
{
    MIBSPI_Object*      ptrMibSpiDriver = NULL;
    MIBSPI_Config*      ptrSPIConfig;
    int32_t             spiStatus = SystemP_SUCCESS;

    /* Get the SPI driver Configuration: */
    ptrSPIConfig = (MIBSPI_Config*)handle;

    ptrMibSpiDriver = (MIBSPI_Object*)ptrSPIConfig->object;

    DebugP_assert(ptrMibSpiDriver->params.dmaHandle != NULL);

    if (ptrMibSpiDriver->params.compatibilityMode)
    {
        spiStatus = MIBSPI_edmaRegTransfer(ptrMibSpiDriver, xferInfo);
    }
    else
    {
        spiStatus = MIBSPI_edmaRamTransfer(ptrMibSpiDriver, xferInfo);
    }
    return spiStatus;
}

int32_t MIBSPI_edmaFreeChResource(const MIBSPI_Handle handle, uint32_t dmaReqLine)
{
    uint32_t               baseAddr, regionId, dmaCh, tcc, param;
    int32_t                spiStatus = SystemP_SUCCESS;
    MIBSPI_Config         *ptrSPIConfig ;
    MIBSPI_Object         *ptrMibSpiDriver;
    MIBSPI_EDMAChParams   *edmaParamsTx, *edmaParamsRx ;
    EDMA_Handle            mibspiEdmaHandle;

    /* Get the SPI driver Configuration: */
    ptrSPIConfig = (MIBSPI_Config*)handle;

    ptrMibSpiDriver = (MIBSPI_Object*)ptrSPIConfig->object;

    mibspiEdmaHandle = ptrMibSpiDriver->params.dmaHandle;

    edmaParamsTx = &ptrMibSpiDriver->edmaParamsTx[dmaReqLine];
    edmaParamsRx = &ptrMibSpiDriver->edmaParamsRx[dmaReqLine];

    /* Fetch the EDMA paramters */
    baseAddr = ptrMibSpiDriver->edmaBaseAddr;
    regionId = ptrMibSpiDriver->edmaRegionId;
    dmaCh    = edmaParamsRx->edmaChId;
    tcc      = edmaParamsRx->edmaTcc;
    param    = edmaParamsRx->edmaParam;

    /* unregister Rx interrupt */
    spiStatus = EDMA_unregisterIntr(mibspiEdmaHandle, &ptrMibSpiDriver->intrRxObj);
    
    /* Free Rx channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, MIBSPI_RXEVENTQUE);

    /* Free the EDMA Rx channel resources managed by driver. */
    spiStatus += EDMA_freeDmaChannel(mibspiEdmaHandle, &dmaCh);
    spiStatus += EDMA_freeTcc(mibspiEdmaHandle, &tcc);
    spiStatus += EDMA_freeParam(mibspiEdmaHandle, &param);

    dmaCh    = edmaParamsTx->edmaChId;
    tcc      = edmaParamsTx->edmaTcc;
    param    = edmaParamsTx->edmaParam;

    /* unregister Tx interrupt */
    spiStatus += EDMA_unregisterIntr(mibspiEdmaHandle, &ptrMibSpiDriver->intrTxObj);
    
    /* Free Tx channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, MIBSPI_TXEVENTQUE);

    /* Free the EDMA Rx channel resources managed by driver. */
    spiStatus += EDMA_freeDmaChannel(mibspiEdmaHandle, &dmaCh);
    spiStatus += EDMA_freeTcc(mibspiEdmaHandle, &tcc);
    spiStatus += EDMA_freeParam(mibspiEdmaHandle, &param);

    /* De-allocate link param ID for SPI mode */
    if (ptrMibSpiDriver->params.compatibilityMode)
    {
        /* unregister dummy interrupt */
        spiStatus += EDMA_unregisterIntr(mibspiEdmaHandle, &ptrMibSpiDriver->intrDummyObj);
        ptrMibSpiDriver->params.compatibilityMode = FALSE;
        param = ptrMibSpiDriver->params.edmaLinkParamId;
        spiStatus += EDMA_freeParam(mibspiEdmaHandle, &param);
    }
    return spiStatus;
}

/* ========================================================================== */
/*                          Static Function Definitions                       */
/* ========================================================================== */
static void MIBSPI_edmaRegUpdateRxParams(const MIBSPI_Object            *ptrMibSpiDriver,
                                         const MIBSPI_DMAXferAddrInfo   *xferAddrInfo,
                                         const MIBSPI_DMAXferSizeInfo   *xferSizeInfo,
                                         uint32_t                        tcc,
                                         EDMACCPaRAMEntry              *rxParamSet)
{
    Bool dummyRxXfer;
    
    if (xferAddrInfo->daddr == NULL)
    {
        dummyRxXfer = TRUE;
    }
    else
    {
        dummyRxXfer = FALSE;
    }
    
    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(rxParamSet);;

    /* Source address */
    rxParamSet->srcAddr = xferAddrInfo->saddr;

    if (!dummyRxXfer)
    {
        /* destinationAddress is address of memory location named buffer.*/
        rxParamSet->destAddr = xferAddrInfo->daddr;
    }
    else
    {
         rxParamSet->destAddr = SOC_virtToPhy((void*)&ptrMibSpiDriver->rxScratchBuffer);
    }
    
    DebugP_assert(xferSizeInfo->elemSize <= 2);
    rxParamSet->aCnt = xferSizeInfo->elemSize;

    /* bCount holds the number of such arrays to be transferred.*/
    rxParamSet->bCnt = xferSizeInfo->elemCnt;

    /* cCount holds the number of frames of aCount*bCount bytes to be transferred.*/
    DebugP_assert(xferSizeInfo->frameCnt == 1);
    rxParamSet->cCnt = xferSizeInfo->frameCnt;

    /* The sourceBindex should not be incremented since it is a h/w register.*/
    rxParamSet->srcBIdx     = (int16_t) EDMA_PARAM_BIDX(0);
    rxParamSet->srcBIdxExt  = (int8_t) EDMA_PARAM_BIDX_EXT(0);

    if(!dummyRxXfer)
    {
        /* The destinationBindex should be incremented aCount number of bytes.*/
        rxParamSet->destBIdx     = (int16_t) EDMA_PARAM_BIDX(rxParamSet->aCnt);
        rxParamSet->destBIdxExt  = (int8_t) EDMA_PARAM_BIDX_EXT(rxParamSet->aCnt);
    }
    else
    {
        /* The destinationBindex should not be incremented.*/
        rxParamSet->destBIdx     = (int16_t) EDMA_PARAM_BIDX(0);
        rxParamSet->destBIdxExt  = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    }

    /* A sync Transfer Mode. */
    /* srCIdx and destinationCindex set to zero since ASYNC Mode is used.*/
    rxParamSet->srcCIdx = 0;
    rxParamSet->destCIdx = 0;

    rxParamSet->linkAddr = 0xFFFFU;

    rxParamSet->bCntReload = rxParamSet->bCnt;

    rxParamSet->opt |= (EDMA_OPT_TCINTEN_MASK | 
    ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    

}

static void MIBSPI_edmaRegUpdateTxParams(const MIBSPI_Object            *ptrMibSpiDriver,
                                         const MIBSPI_DMAXferAddrInfo   *xferAddrInfo,
                                         const MIBSPI_DMAXferSizeInfo   *xferSizeInfo,
                                         uint32_t                        tcc,
                                         EDMACCPaRAMEntry              *txParamSet)
{
    Bool dummyTxXfer;
    
    if(xferAddrInfo->saddr == NULL)
    {
        dummyTxXfer = TRUE;
    }
    else
    {
        dummyTxXfer = FALSE;
    }
    
    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(txParamSet);

    if(!dummyTxXfer)
    {
        /* sourceAddress holds address of memory location buffer. */
        txParamSet->srcAddr = xferAddrInfo->saddr;
    }
    else
    {
        /* sourceAddress holds address of memory location buffer. */
        txParamSet->srcAddr = SOC_virtToPhy((void*)&ptrMibSpiDriver->txScratchBuffer);
    }

    /* destinationAddress holds address of SPIDAT1 register. */
    txParamSet->destAddr = xferAddrInfo->daddr;

    /* 8-bit word length */
    txParamSet->aCnt = xferSizeInfo->elemSize;

    /* bCount holds the number of such arrays to be transferred. */
    txParamSet->bCnt = xferSizeInfo->elemCnt;

    /* cCount holds the number of frames of aCount*bCount bytes to be transferred. */
    txParamSet->cCnt = xferSizeInfo->frameCnt;

    if(!dummyTxXfer)
    {
        /*
         ** The sourceBindex should be incremented by aCount number of bytes since the
         ** source used here is memory.
        */
        txParamSet->srcBIdx     = (int16_t) EDMA_PARAM_BIDX(txParamSet->aCnt);
        txParamSet->srcBIdxExt  = (int8_t) EDMA_PARAM_BIDX_EXT(txParamSet->aCnt);
    }
    else
    {
        /* If Tx scratch buffer do not increment sourceBindex */
        txParamSet->srcBIdx     = (int16_t) EDMA_PARAM_BIDX(0);
        txParamSet->srcBIdxExt  = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    }

    txParamSet->destBIdx     = (int16_t) EDMA_PARAM_BIDX(0);
    txParamSet->destBIdxExt  = (int8_t) EDMA_PARAM_BIDX_EXT(0);

    /* Async Transfer Mode is set in OPT.*/
    /* srCIdx and destinationCindex set to zero since ASYNC Mode is used. */
    txParamSet->srcCIdx = 0;
    txParamSet->destCIdx = 0;
    txParamSet->linkAddr = 0xFFFFU;
    txParamSet->bCntReload = txParamSet->bCnt;

    txParamSet->opt |= (EDMA_OPT_TCINTEN_MASK | 
             ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
  
}

static void MIBSPI_edmaRamUpdateRxParams(const MIBSPI_Object            *ptrMibSpiDriver,
                                         const MIBSPI_DMAXferAddrInfo   *xferAddrInfo,
                                         const MIBSPI_DMAXferSizeInfo   *xferSizeInfo,
                                         uint32_t                       tcc,
                                         EDMACCPaRAMEntry              *rxParamSet)
{
    Bool dummyRxXfer;
    
    if(xferAddrInfo->daddr == NULL)
    {
        dummyRxXfer = TRUE;
    }
    else
    {
        dummyRxXfer = FALSE;
    }
    
    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(rxParamSet);

    /* sourceAddress holds address of memory location buffer. */
    rxParamSet->srcAddr = xferAddrInfo->saddr;

    if (!dummyRxXfer)
    {
        /* destinationAddress is address of memory location named buffer.*/
        rxParamSet->destAddr = xferAddrInfo->daddr;
    }
    else
    {
         rxParamSet->destAddr = SOC_virtToPhy((void*)&ptrMibSpiDriver->rxScratchBuffer);
    }

    /* aCount holds the number of bytes in an array.*/
    rxParamSet->aCnt = xferSizeInfo->elemSize ;

    /* bCount holds the number of such arrays to be transferred.*/
    rxParamSet->bCnt = xferSizeInfo->elemCnt;

    /* cCount holds the number of frames of aCount*bCount bytes to be transferred.*/
    rxParamSet->cCnt = xferSizeInfo->frameCnt;

    /* The sourceBindex is MIBSPI Ram and has index of 4.*/
    rxParamSet->srcBIdx     = (int16_t) EDMA_PARAM_BIDX(4);
    rxParamSet->srcBIdxExt  = (int8_t) EDMA_PARAM_BIDX_EXT(4);

    if(!dummyRxXfer)
    {
        /* The destinationBindex should be incremented aCount number of bytes.*/
        rxParamSet->destBIdx     = (int16_t) EDMA_PARAM_BIDX(rxParamSet->aCnt);
        rxParamSet->destBIdxExt  = (int8_t) EDMA_PARAM_BIDX_EXT(rxParamSet->aCnt);
    }
    else
    {
        /* The destinationBindex should not be incremented.*/
        rxParamSet->destBIdx     = (int16_t) EDMA_PARAM_BIDX(0);
        rxParamSet->destBIdxExt  = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    }

    /* AB sync Transfer Mode. */
    rxParamSet->srcCIdx = 0;
    rxParamSet->destCIdx = rxParamSet->bCnt * rxParamSet->aCnt;
    rxParamSet->linkAddr = 0xFFFFU;
    rxParamSet->bCntReload = rxParamSet->bCnt;
    rxParamSet->opt |=
     (EDMA_OPT_TCINTEN_MASK |  EDMA_OPT_SYNCDIM_MASK |
    ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
}

static void MIBSPI_edmaRamUpdateTxParams(const MIBSPI_Object            *ptrMibSpiDriver,
                                         const MIBSPI_DMAXferAddrInfo   *xferAddrInfo,
                                         const MIBSPI_DMAXferSizeInfo   *xferSizeInfo,
                                         uint32_t                        tcc,
                                         EDMACCPaRAMEntry              *txParamSet)
{
    Bool dummyTxXfer;
    

    
    if(xferAddrInfo->saddr == NULL)
    {
        dummyTxXfer = TRUE;
    }
    else
    {
        dummyTxXfer = FALSE;
    }

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(txParamSet);
    
    if(!dummyTxXfer)
    {
        /* sourceAddress holds address of memory location buffer. */
        txParamSet->srcAddr = xferAddrInfo->saddr;
    }
    else
    {
        /* sourceAddress holds address of memory location buffer. */
        txParamSet->srcAddr = SOC_virtToPhy((void*)&ptrMibSpiDriver->txScratchBuffer);
    }

    /* destinationAddress holds address of SPIDAT1 register. */
    txParamSet->destAddr = xferAddrInfo->daddr;

    /* aCount holds the number of bytes in an array. */
    txParamSet->aCnt =  xferSizeInfo->elemSize;

    /* bCount holds the number of such arrays to be transferred. */
    txParamSet->bCnt =  xferSizeInfo->elemCnt;

    /* cCount holds the number of frames of aCount*bCount bytes to be transferred. */
    txParamSet->cCnt = xferSizeInfo->frameCnt;

    /*
    ** The sourceBindex should be incremented by aCount number of bytes since the
    ** source used here is memory.
    */
    if(!dummyTxXfer)
    {
        txParamSet->srcBIdx     = (int16_t) EDMA_PARAM_BIDX(txParamSet->aCnt);
        txParamSet->srcBIdxExt  = (int8_t) EDMA_PARAM_BIDX_EXT(txParamSet->aCnt);
    }
    else
    {
        txParamSet->srcBIdx     = (int16_t) EDMA_PARAM_BIDX(0);
        txParamSet->srcBIdxExt  = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    }

    txParamSet->destBIdx     = (int16_t) EDMA_PARAM_BIDX(4);
    txParamSet->destBIdxExt  = (int8_t) EDMA_PARAM_BIDX_EXT(4);

    /* AB-sync Transfer Mode is set in OPT.*/
    txParamSet->srcCIdx = txParamSet->bCnt * txParamSet->aCnt;
    txParamSet->destCIdx = 0;
    txParamSet->linkAddr = 0xFFFFU;
    txParamSet->bCntReload = txParamSet->bCnt;
    txParamSet->opt |=
    ( EDMA_OPT_TCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
}

static int32_t MIBSPI_edmaRegTransfer(MIBSPI_Object*    ptrMibSpiDriver,
                                      MIBSPI_DMAXferInfo *xferInfo)
{
    uint32_t               baseAddr, regionId, dmaChTx, tccTx, paramTx, dmaChRx, tccRx, paramRx, edmaLinkParamId;
    int32_t                spiStatus = SystemP_SUCCESS;
    EDMACCPaRAMEntry      rxParamSet;
    EDMACCPaRAMEntry      txParamSet;
    EDMACCPaRAMEntry      dummyParamSet;
    const MIBSPI_Attrs    *hwAttrs ;
    MIBSPI_EDMAChParams   *edmaParamsTx, *edmaParamsRx;
    EDMA_Handle            mibspiEdmaHandle;

    mibspiEdmaHandle = ptrMibSpiDriver->params.dmaHandle;

     /* Get the pointer to the object and hwAttrs */
    hwAttrs = ptrMibSpiDriver->ptrHwCfg;

    edmaParamsTx = &ptrMibSpiDriver->edmaParamsTx[xferInfo->dmaReqLine];
    edmaParamsRx = &ptrMibSpiDriver->edmaParamsRx[xferInfo->dmaReqLine];

    /* Get the base address and region ID */
    baseAddr = ptrMibSpiDriver->edmaBaseAddr;
    regionId = ptrMibSpiDriver->edmaRegionId;

    /* Get transmit channel params */
    dmaChTx  = edmaParamsTx->edmaChId;
    tccTx    = edmaParamsTx->edmaTcc;
    paramTx  = edmaParamsTx->edmaParam;

    /* Get receive channel params */
    dmaChRx  = edmaParamsRx->edmaChId;
    tccRx    = edmaParamsRx->edmaTcc;
    paramRx  = edmaParamsRx->edmaParam;

    DebugP_assert(mibspiEdmaHandle != NULL);

    ptrMibSpiDriver->transactionState.edmaCbCheck = MIBSPI_NONE_EDMA_CALLBACK_OCCURED;

    /* Update param sets based on the transaction parameters */
    MIBSPI_edmaRegUpdateRxParams(ptrMibSpiDriver,&xferInfo->rx, &xferInfo->size,
                                 tccRx, &rxParamSet);

    MIBSPI_edmaRegUpdateTxParams(ptrMibSpiDriver,&xferInfo->tx, &xferInfo->size,
                                 tccTx, &txParamSet);

    /* Write Rx param set */
    EDMA_setPaRAM(baseAddr, paramRx, &rxParamSet);

    /* Write Tx param set */
    EDMA_setPaRAM(baseAddr, paramTx, &txParamSet);

    /* initalize dummy param set */
    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&dummyParamSet);
    edmaLinkParamId = ptrMibSpiDriver->params.edmaLinkParamId;

    dummyParamSet.aCnt     = 1;
    dummyParamSet.linkAddr = 0xFFFFU;
    dummyParamSet.opt |= ( EDMA_OPT_TCINTEN_MASK | EDMA_OPT_STATIC_MASK |
                            ((((uint32_t)edmaLinkParamId) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) );

    /* Write Tx param set */
    EDMA_setPaRAM(baseAddr, edmaLinkParamId, &dummyParamSet);

    /* Link  dummy param ID */
    EDMA_linkChannel(baseAddr, paramTx, edmaLinkParamId);
    
    /*
     * Transfer is done in A sync mode
     */
    EDMA_enableTransferRegion(baseAddr, regionId, dmaChRx,
            EDMA_TRIG_MODE_EVENT);
    EDMA_enableTransferRegion(baseAddr, regionId, dmaChTx,
            EDMA_TRIG_MODE_EVENT);

    /* Enable SPI DMA for transaction */
    SPIIntEnable((uintptr_t)hwAttrs->ptrSpiRegBase);

    return spiStatus;
}

static int32_t MIBSPI_edmaRamTransfer(MIBSPI_Object      *ptrMibSpiDriver,
                                      MIBSPI_DMAXferInfo *xferInfo)
{
    uint32_t               baseAddr, regionId, dmaChTx, tccTx, paramTx, dmaChRx, tccRx, paramRx;
    int32_t                spiStatus = SystemP_SUCCESS;
    EDMACCPaRAMEntry      rxParamSet;
    EDMACCPaRAMEntry      txParamSet;
    MIBSPI_EDMAChParams   *edmaParamsTx, *edmaParamsRx;
    EDMA_Handle            mibspiEdmaHandle;
    
    mibspiEdmaHandle = ptrMibSpiDriver->params.dmaHandle;

    edmaParamsTx = &ptrMibSpiDriver->edmaParamsTx[xferInfo->dmaReqLine];
    edmaParamsRx = &ptrMibSpiDriver->edmaParamsRx[xferInfo->dmaReqLine];

    /* Get the base address and region ID */
    baseAddr = ptrMibSpiDriver->edmaBaseAddr;
    regionId = ptrMibSpiDriver->edmaRegionId;

    /* Get transmit channel params */
    dmaChTx  = edmaParamsTx->edmaChId;
    tccTx    = edmaParamsTx->edmaTcc;
    paramTx  = edmaParamsTx->edmaParam;

    /* Get receive channel params */
    dmaChRx  = edmaParamsRx->edmaChId;
    tccRx    = edmaParamsRx->edmaTcc;
    paramRx  = edmaParamsRx->edmaParam;

    DebugP_assert(mibspiEdmaHandle != NULL);

    ptrMibSpiDriver->transactionState.edmaCbCheck = MIBSPI_NONE_EDMA_CALLBACK_OCCURED;

    /* Update param sets based on the transaction parameters */
    MIBSPI_edmaRamUpdateRxParams(ptrMibSpiDriver, &xferInfo->rx, &xferInfo->size,
                                 tccRx, &rxParamSet);

    MIBSPI_edmaRamUpdateTxParams(ptrMibSpiDriver, &xferInfo->tx, &xferInfo->size,
                                 tccTx, &txParamSet);

    /* Write Rx param set */
    EDMA_setPaRAM(baseAddr, paramRx, &rxParamSet);

    /* Write Tx param set */
    EDMA_setPaRAM(baseAddr, paramTx, &txParamSet);

    /*
     * Transfer is done in AB sync mode
     */
    EDMA_enableTransferRegion(baseAddr, regionId, dmaChRx,
            EDMA_TRIG_MODE_EVENT);
    EDMA_enableTransferRegion(baseAddr, regionId, dmaChTx,
            EDMA_TRIG_MODE_EVENT);

    return spiStatus;
}

static uint32_t MIBSPI_edmaMapTcc2DmaReqLine(const MIBSPI_Attrs *ptrHwCfg, 
                                             uint32_t tcc, 
                                             Bool isRxCh)
{
    uint32_t i;
    uint32_t dmaReqLine;

    for (i = 0; i < ptrHwCfg->numDmaReqLines; i++)
    {
        if (isRxCh)
        {
            if (ptrHwCfg->dmaReqlineCfg[i].rxDmaReqLine == tcc)
            {
                break;
            }
        }
        else
        {
            if (ptrHwCfg->dmaReqlineCfg[i].txDmaReqLine == tcc)
            {
                break;
            }
        }
    }
    if (i < ptrHwCfg->numDmaReqLines)
    {
        dmaReqLine = i;
    }
    else
    {
        dmaReqLine = MIBSPI_INVALID_DMA_REQLINE;
    }
    return dmaReqLine;
}

static void EDMA_txCompletionIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    MIBSPI_Object   *ptrMibSpiDriver = (MIBSPI_Object *)args;
    uint32_t         dmaReqLine;
    
    dmaReqLine = MIBSPI_edmaMapTcc2DmaReqLine(ptrMibSpiDriver->ptrHwCfg, ptrMibSpiDriver->intrTxObj.tccNum, FALSE);
    DebugP_assert((dmaReqLine < ptrMibSpiDriver->ptrHwCfg->numDmaReqLines)
                  &&
                 (dmaReqLine < MIBSPI_UTILS_ARRAYSIZE(ptrMibSpiDriver->ptrHwCfg->dmaReqlineCfg)));
    ptrMibSpiDriver->dmaInfo[dmaReqLine].txDmaIntCnt++; 
    EDMA_disableTransferRegion(ptrMibSpiDriver->edmaBaseAddr,ptrMibSpiDriver->edmaRegionId,
                               ptrMibSpiDriver->edmaParamsTx[dmaReqLine].edmaChId, EDMA_TRIG_MODE_EVENT);    

    if (MIBSPI_RX_EDMA_CALLBACK_OCCURED == ptrMibSpiDriver->transactionState.edmaCbCheck)
    {
        ptrMibSpiDriver->transactionState.edmaCbCheck = MIBSPI_NONE_EDMA_CALLBACK_OCCURED;

        /* Call the completion function */
        MIBSPI_dmaDoneCb(ptrMibSpiDriver->mibspiHandle);
    }
    else
    {
        ptrMibSpiDriver->transactionState.edmaCbCheck = MIBSPI_TX_EDMA_CALLBACK_OCCURED;
    }
}

static void EDMA_rxCompletionIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    MIBSPI_Object   *ptrMibSpiDriver = (MIBSPI_Object *)args;
    uint32_t         dmaReqLine;
    
    dmaReqLine = MIBSPI_edmaMapTcc2DmaReqLine(ptrMibSpiDriver->ptrHwCfg, ptrMibSpiDriver->intrRxObj.tccNum, TRUE);
    DebugP_assert((dmaReqLine < ptrMibSpiDriver->ptrHwCfg->numDmaReqLines)
                   &&
                  (dmaReqLine < MIBSPI_UTILS_ARRAYSIZE(ptrMibSpiDriver->ptrHwCfg->dmaReqlineCfg)));
    ptrMibSpiDriver->dmaInfo[dmaReqLine].rxDmaIntCnt++;  

    EDMA_disableTransferRegion(ptrMibSpiDriver->edmaBaseAddr,ptrMibSpiDriver->edmaRegionId,
                              ptrMibSpiDriver->edmaParamsRx[dmaReqLine].edmaChId, EDMA_TRIG_MODE_EVENT);     

    if (MIBSPI_TX_EDMA_CALLBACK_OCCURED == ptrMibSpiDriver->transactionState.edmaCbCheck)
    {
        ptrMibSpiDriver->transactionState.edmaCbCheck = MIBSPI_NONE_EDMA_CALLBACK_OCCURED;

        /* Call the completion function */
        MIBSPI_dmaDoneCb(ptrMibSpiDriver->mibspiHandle);
    }
    else
    {
        ptrMibSpiDriver->transactionState.edmaCbCheck = MIBSPI_RX_EDMA_CALLBACK_OCCURED;
    }
}

static void SPIIntEnable(uint32_t baseAdd)
{
    uint32_t regAddr = baseAdd + CSL_SPI_SPIINT0;
    /* Enable the Interrupts. */
    CSL_FINS(*(uint32_t*)regAddr,SPI_SPIINT0_DMAREQEN,1U);
}

static void MIBSPI_edmaDoNothing(Edma_IntrHandle intrHandle, void *args)
{
    /* No implementation required */
}

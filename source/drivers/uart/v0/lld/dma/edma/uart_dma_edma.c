/*
 *  Copyright (C) 2022-2023 Texas Instruments Incorporated
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
 *  \file uart_dma_edma.c
 *
 *  \brief File containing EDMA Driver APIs implementation for UART.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <drivers/soc.h>
#include <drivers/edma/v0/edma.h>
#include <drivers/uart/v0/lld/dma/uart_dma.h>
#include <drivers/uart/v0/lld/dma/edma/uart_dma_edma.h>
#include <drivers/uart/v0/lld/uart_lld.h>
#include <kernel/dpl/CacheP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** \brief Transmit EDMA channel event queue number                           */
#define EDMA_UART_TX_EVT_QUEUE_NO                  (0U)
/** \brief Receive EDMA channel event queue number                            */
#define EDMA_UART_RX_EVT_QUEUE_NO                  (1U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int32_t UART_edmaChInit(UARTLLD_Handle hUart, UART_EdmaChConfig *edmaChCfg);
static void UART_edmaIsrTx(Edma_IntrHandle intrHandle, void *args);
static void UART_edmaIsrRx(Edma_IntrHandle intrHandle, void *args);
static void UART_edmaDoNothing(Edma_IntrHandle intrHandle, void *args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t UART_lld_dmaInit(UARTLLD_Handle hUart, UART_DmaChConfig dmaChCfg)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    UART_EdmaChConfig *edmaChCfg = (UART_EdmaChConfig *)dmaChCfg;

    status = UART_edmaChInit(hUart, edmaChCfg);

    if (status == UART_TRANSFER_STATUS_SUCCESS)
    {
        edmaChCfg->isOpen = TRUE;
    }

    return status;
}

static int32_t UART_edmaChInit(UARTLLD_Handle hUart, UART_EdmaChConfig *edmaChCfg)
{
    int32_t             status = UART_TRANSFER_STATUS_FAILURE;
    uint32_t            retVal = TRUE;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh,dmaTxCh, tccRx, tccTx, tccDummy, paramRx, paramTx, paramDummy;
    uint32_t            isEdmaInterruptEnabled;
    Edma_IntrObject     *edmaIntrObjectRx;
    Edma_IntrObject     *edmaIntrObjectTx;
    Edma_IntrObject     *edmaIntrObjectDummy;
    EDMA_Handle         uartEdmaHandle;

    uartEdmaHandle = (EDMA_Handle)hUart->hUartInit->uartDmaHandle;
    edmaIntrObjectRx = &edmaChCfg->edmaIntrObjRx;
    edmaIntrObjectTx = &edmaChCfg->edmaIntrObjTx;
    edmaIntrObjectDummy = &edmaChCfg->edmaIntrObjDummy;

    if(uartEdmaHandle != NULL_PTR)
    {
        status = UART_TRANSFER_STATUS_SUCCESS;

        /* Read base address of allocated EDMA instance */
        baseAddr = EDMA_getBaseAddr(uartEdmaHandle);

        /* Read the region ID of the EDMA instance */
        regionId = EDMA_getRegionId(uartEdmaHandle);

        /* Store the EDMA parameters */
        edmaChCfg->edmaBaseAddr = baseAddr;
        edmaChCfg->edmaRegionId = regionId;

        /* Check if interrupt is enabled */
        isEdmaInterruptEnabled = EDMA_isInterruptEnabled(uartEdmaHandle);

        if((baseAddr != 0U) && (regionId < SOC_EDMA_NUM_REGIONS) && (isEdmaInterruptEnabled == TRUE))
        {
            /* Allocate EDMA channel for UART RX transfer */
            dmaRxCh = edmaChCfg->edmaRxChId;
            status += EDMA_allocDmaChannel(uartEdmaHandle, &dmaRxCh);

            /* Allocate EDMA TCC for UART RX transfer */
            tccRx = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocTcc(uartEdmaHandle, &tccRx);

            /* Allocate a Param ID for UART RX transfer */
            paramRx = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocParam(uartEdmaHandle, &paramRx);

            if(status == UART_TRANSFER_STATUS_SUCCESS)
            {
                retVal = EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaRxCh, tccRx, paramRx, EDMA_UART_RX_EVT_QUEUE_NO);
                if(retVal == FALSE)
                {
                    status += UART_TRANSFER_STATUS_FAILURE;
                }

                /* Register RX interrupt */
                edmaIntrObjectRx->tccNum = tccRx;
                edmaIntrObjectRx->cbFxn  = &UART_edmaIsrRx;
                edmaIntrObjectRx->appData = (void *) hUart;
                status += EDMA_registerIntr(uartEdmaHandle, edmaIntrObjectRx);
            }

            if(status == UART_TRANSFER_STATUS_SUCCESS)
            {
                /* Store the EDMA parameters for UART RX*/
                edmaChCfg->edmaRxParam = paramRx;
                edmaChCfg->edmaRxChId  = dmaRxCh;
                edmaChCfg->edmaTccRx   = tccRx;
            }

            /* Allocate EDMA channel for UART TX transfer */
            dmaTxCh = edmaChCfg->edmaTxChId;
            status += EDMA_allocDmaChannel(uartEdmaHandle, &dmaTxCh);

            /* Allocate EDMA TCC for UART TX transfer */
            tccTx = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocTcc(uartEdmaHandle, &tccTx);

            /* Allocate a Param ID for UART TX transfer */
            paramTx = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocParam(uartEdmaHandle, &paramTx);

            if(status == UART_TRANSFER_STATUS_SUCCESS)
            {
                retVal = EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaTxCh, tccTx, paramTx, EDMA_UART_TX_EVT_QUEUE_NO);

                if(retVal == FALSE)
                {
                    status += UART_TRANSFER_STATUS_FAILURE;
                }
                /* Register TX interrupt */
                edmaIntrObjectTx->tccNum = tccTx;
                edmaIntrObjectTx->cbFxn  = &UART_edmaIsrTx;
                edmaIntrObjectTx->appData = (void *) hUart;
                status += EDMA_registerIntr(uartEdmaHandle, edmaIntrObjectTx);
            }

            if(status == UART_TRANSFER_STATUS_SUCCESS)
            {
                /* Store the EDMA parameters for UART TX*/
                edmaChCfg->edmaTxParam = paramTx;
                edmaChCfg->edmaTxChId  = dmaTxCh;
                edmaChCfg->edmaTccTx   = tccTx;
            }

            /* Allocate EDMA TCC for Dummy param set */
            tccDummy = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocTcc(uartEdmaHandle, &tccDummy);

            /* Allocate a Param ID for UART TX transfer */
            paramDummy = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocParam(uartEdmaHandle, &paramDummy);

            if(status == UART_TRANSFER_STATUS_SUCCESS)
            {

                /* Register Dummy interrupt */
                edmaIntrObjectDummy->tccNum = tccDummy;
                edmaIntrObjectDummy->cbFxn  = &UART_edmaDoNothing;
                edmaIntrObjectDummy->appData = (void *) hUart;
                status += EDMA_registerIntr(uartEdmaHandle, edmaIntrObjectDummy);
            }

            if(status == UART_TRANSFER_STATUS_SUCCESS)
            {
                /* Store the EDMA parameters for UART TX*/
                edmaChCfg->edmaDummyParam = paramDummy;
                edmaChCfg->edmaTccDummy   = tccDummy;
                edmaChCfg->isOpen = TRUE;
            }
        }
        else
        {
            status = UART_TRANSFER_STATUS_FAILURE;
        }
    }
    return status;
}

int32_t UART_lld_dmaWrite(UARTLLD_Handle hUart, const UART_Transaction *transaction)
{
    int32_t             status = UART_TRANSFER_STATUS_SUCCESS;
    uint32_t            retVal = FALSE;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaTxCh, tccTx, tccDummy, paramTx, paramDummy;
    uint32_t            isTxFifoEmpty, isEdmaEventPending;
    EDMACCPaRAMEntry    edmaTxParam, edmaDummyParam;
    UART_EdmaChConfig   *edmaChCfg;

    isTxFifoEmpty = FALSE;
    isEdmaEventPending = FALSE;

    edmaChCfg = (UART_EdmaChConfig *)hUart->hUartInit->dmaChCfg;

    /* Fetch the EDMA paramters for UART transfer */
    baseAddr               = edmaChCfg->edmaBaseAddr;
    regionId               = edmaChCfg->edmaRegionId;

    /* Fetch the EDMA paramters for UART TX transfer */
    dmaTxCh    = edmaChCfg->edmaTxChId;
    tccTx      = edmaChCfg->edmaTccTx;
    tccDummy   = edmaChCfg->edmaTccDummy;
    paramTx    = edmaChCfg->edmaTxParam;
    paramDummy = edmaChCfg->edmaDummyParam;

    /* Initialize TX Param Set */
    EDMA_ccPaRAMEntry_init(&edmaTxParam);

    /* Transmit param set configuration */
    edmaTxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *) transaction->buf);
    edmaTxParam.destAddr      = (uint32_t) SOC_virtToPhy((void *) (hUart->baseAddr + (uint32_t)UART_THR));
    edmaTxParam.aCnt          = (uint16_t) 1;
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
    edmaDummyParam.opt           = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_STATIC_MASK |
                                   ((tccDummy << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    /* Write Tx param set */
    EDMA_setPaRAM(baseAddr, paramDummy, &edmaDummyParam);

    /* Link  dummy param ID */
    EDMA_linkChannel(baseAddr, paramTx, paramDummy);

    if(FALSE == UART_checkCharsAvailInFifo(hUart->baseAddr))
    {
        isTxFifoEmpty = TRUE;
    }

    if(TRUE == EDMA_readEventStatusRegion(baseAddr, dmaTxCh))
    {
        isEdmaEventPending = TRUE;
    }

    /* Set event trigger to start UART TX transfer */
    retVal = (uint32_t)EDMA_enableTransferRegion(baseAddr, regionId, dmaTxCh,
         EDMA_TRIG_MODE_EVENT);

    if(retVal == TRUE)
    {
        if((isTxFifoEmpty!=0U) && (isEdmaEventPending == 0U))
        {
            /* Set manual trigger to start UART TX transfer */
            retVal = (uint32_t)EDMA_enableTransferRegion(baseAddr, regionId, dmaTxCh,
                EDMA_TRIG_MODE_MANUAL);
        }
    }

    if(retVal == TRUE)
    {
        status = UART_TRANSFER_STATUS_SUCCESS;
    }
    return status;
}

int32_t UART_lld_dmaRead(UARTLLD_Handle hUart, const UART_Transaction *transaction)
{
    uint32_t            retVal = TRUE;
    int32_t             status = UART_STATUS_SUCCESS;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh, tccRx, paramRx;
    EDMACCPaRAMEntry    edmaRxParam;
    UART_EdmaChConfig   *edmaChCfg;

    edmaChCfg = (UART_EdmaChConfig *)hUart->hUartInit->dmaChCfg;

    /* Fetch the EDMA paramters for UART transfer */
    baseAddr               = edmaChCfg->edmaBaseAddr;
    regionId               = edmaChCfg->edmaRegionId;

    /* Fetch the EDMA paramters for UART RX transfer */
    dmaRxCh    = edmaChCfg->edmaRxChId;
    tccRx      = edmaChCfg->edmaTccRx;
    paramRx    = edmaChCfg->edmaRxParam;

    /* Initialize RX Param Set */
    EDMA_ccPaRAMEntry_init(&edmaRxParam);

    /* Receive param set configuration */
    edmaRxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *) (hUart->baseAddr + UART_RHR));
    edmaRxParam.destAddr      = (uint32_t) SOC_virtToPhy((void *) transaction->buf);
    edmaRxParam.aCnt          = (uint16_t) 1;
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

    /* Set event trigger to start UART RX transfer */
    retVal = EDMA_enableTransferRegion(baseAddr, regionId, dmaRxCh,
         EDMA_TRIG_MODE_EVENT);
    if(retVal == TRUE)
    {
        status = UART_TRANSFER_STATUS_SUCCESS;
    }
    else
    {
        status = UART_TRANSFER_STATUS_FAILURE;
    }

    return status;
}

int32_t UART_lld_dmaDeInit(UARTLLD_Handle hUart)
{
    int32_t            status = UART_TRANSFER_STATUS_SUCCESS;
    uint32_t           retVal = TRUE;
    uint32_t           baseAddr, regionId;
    UART_EdmaChConfig   *edmaChCfg;
    EDMA_Handle         uartEdmaHandle;
    Edma_IntrObject     *edmaIntrObjectRx;
    Edma_IntrObject     *edmaIntrObjectTx;
    Edma_IntrObject     *edmaIntrObjectDummy;
    uint32_t            dmaRxCh,dmaTxCh, tccRx, tccTx, tccDummy, paramRx, paramTx, paramDummy;
    UARTLLD_InitHandle     hUartInit;

    /* Check parameters */
    if(NULL_PTR == hUart)
    {
        status = UART_TRANSFER_STATUS_FAILURE;
    }

    if(UART_TRANSFER_STATUS_SUCCESS == status)
    {
        hUartInit = hUart->hUartInit;
        edmaChCfg    = (UART_EdmaChConfig *)hUartInit->dmaChCfg;
        uartEdmaHandle = (EDMA_Handle)hUartInit->uartDmaHandle;

        /* Fetch the EDMA paramters */
        baseAddr         = edmaChCfg->edmaBaseAddr;
        regionId         = edmaChCfg->edmaRegionId;

        if (edmaChCfg->isOpen != FALSE)
        {
            /* Fetch the EDMA paramters */
            dmaRxCh    = edmaChCfg->edmaRxChId;
            tccRx      = edmaChCfg->edmaTccRx;
            paramRx    = edmaChCfg->edmaRxParam;
            edmaIntrObjectRx = &edmaChCfg->edmaIntrObjRx;

            /* unregister Rx interrupt */
            status += EDMA_unregisterIntr(uartEdmaHandle, edmaIntrObjectRx);

            /* Free Rx channel */
            retVal = EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaRxCh, EDMA_TRIG_MODE_EVENT, tccRx, EDMA_UART_RX_EVT_QUEUE_NO);
            if(retVal == FALSE)
            {
                status += UART_TRANSFER_STATUS_FAILURE;
            }

            if(dmaRxCh != EDMA_RESOURCE_ALLOC_ANY)
            {
                status += EDMA_freeDmaChannel(uartEdmaHandle, &dmaRxCh);
            }
            if(tccRx != EDMA_RESOURCE_ALLOC_ANY)
            {
                status += EDMA_freeTcc(uartEdmaHandle, &tccRx);
            }
            if(paramRx != EDMA_RESOURCE_ALLOC_ANY)
            {
                status += EDMA_freeParam(uartEdmaHandle, &paramRx);
            }

            /* Fetch the EDMA paramters */
            dmaTxCh    = edmaChCfg->edmaTxChId;
            tccTx      = edmaChCfg->edmaTccTx;
            paramTx    = edmaChCfg->edmaTxParam;
            edmaIntrObjectTx = &edmaChCfg->edmaIntrObjTx;

            /* unregister Tx interrupt */
            status += EDMA_unregisterIntr(uartEdmaHandle, edmaIntrObjectTx);

            /* Free Tx channel */
            retVal = EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaTxCh, EDMA_TRIG_MODE_EVENT, tccTx, EDMA_UART_TX_EVT_QUEUE_NO);
            if(retVal == FALSE)
            {
                status += UART_TRANSFER_STATUS_FAILURE;
            }

            if(dmaTxCh != EDMA_RESOURCE_ALLOC_ANY)
            {
               status += EDMA_freeDmaChannel(uartEdmaHandle, &dmaTxCh);
            }
            if(tccTx != EDMA_RESOURCE_ALLOC_ANY)
            {
               status += EDMA_freeTcc(uartEdmaHandle, &tccTx);
            }
            if(paramTx != EDMA_RESOURCE_ALLOC_ANY)
            {
                status += EDMA_freeParam(uartEdmaHandle, &paramTx);
            }

            tccDummy   = edmaChCfg->edmaTccDummy;
            paramDummy    = edmaChCfg->edmaDummyParam;
            edmaIntrObjectDummy = &edmaChCfg->edmaIntrObjDummy;

            /* unregister Dummy interrupt */
            status += EDMA_unregisterIntr(uartEdmaHandle, edmaIntrObjectDummy);

            if(tccDummy != EDMA_RESOURCE_ALLOC_ANY)
            {
                status += EDMA_freeTcc(uartEdmaHandle, &tccDummy);
            }
            if(paramDummy != EDMA_RESOURCE_ALLOC_ANY)
            {
               status += EDMA_freeParam(uartEdmaHandle, &paramDummy);
            }

            edmaChCfg->isOpen = FALSE;
        }
    }

    return status;
}

int32_t UART_lld_dmaDisableChannel(UARTLLD_Handle hUart,
                                       uint32_t isChannelTx)
{
    int32_t       status = UART_TRANSFER_STATUS_SUCCESS;
    uint32_t      baseAddr, regionId, dmaCh;
    UART_EdmaChConfig *edmaChCfg;

    if(NULL_PTR != hUart)
    {
        edmaChCfg = (UART_EdmaChConfig *)hUart->hUartInit->dmaChCfg;

        baseAddr               = edmaChCfg->edmaBaseAddr;
        regionId               = edmaChCfg->edmaRegionId;

        /* Disable Channel */
        if (isChannelTx == TRUE)
        {
            dmaCh    = edmaChCfg->edmaTxChId;
        }
        else
        {
            dmaCh    = edmaChCfg->edmaRxChId;
        }

        /* Stop the UART event transfer */
        (void)EDMA_disableTransferRegion(baseAddr, regionId, dmaCh,
            EDMA_TRIG_MODE_EVENT);
    }
    else
    {
        status = UART_TRANSFER_STATUS_FAILURE;
    }

    return status;
}

static void UART_edmaIsrTx(Edma_IntrHandle intrHandle, void *args)
{
    UARTLLD_Handle hUart;

    /* Check parameters */
    if(NULL != args)
    {
        hUart = (UARTLLD_Handle)args;
        hUart->writeTrans.status = UART_TRANSFER_STATUS_SUCCESS;
        hUart->hUartInit->writeCompleteCallbackFxn(hUart);
        UART_lld_Transaction_deInit(&hUart->writeTrans);
    }
    return;
}

static void UART_edmaIsrRx(Edma_IntrHandle intrHandle, void *args)
{
    UARTLLD_Handle hUart;

    /* Check parameters */
    if(NULL != args)
    {
        hUart = (UARTLLD_Handle)args;
        hUart->readTrans.status = UART_TRANSFER_STATUS_SUCCESS;
        hUart->hUartInit->readCompleteCallbackFxn(hUart);
        UART_lld_Transaction_deInit(&hUart->readTrans);
    }

    return;
}

static void UART_edmaDoNothing(Edma_IntrHandle intrHandle, void *args)
{
    /* No implementation required */
}

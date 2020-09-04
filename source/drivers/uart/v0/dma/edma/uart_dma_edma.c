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
#include <drivers/edma.h>
#include <drivers/uart/v0/dma/uart_dma.h>
#include <drivers/uart/v0/dma/edma/uart_dma_edma.h>
#include <drivers/uart.h>
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
static int32_t UART_edmaOpen(UART_Handle uartHandle, void* uartDmaArgs);
static int32_t UART_edmaChInit(UART_Handle handle, UartDma_EdmaArgs *edmaArgs);
static int32_t UART_edmaClose(UART_Handle handle);
static int32_t UART_edmaDisableChannel(UART_Handle handle,
                                       uint32_t isChannelTx);
static int32_t UART_edmaTransferWrite(UART_Object *obj, const UART_Attrs *attrs,
                                      UART_Transaction *transaction);
static int32_t UART_edmaTransferRead(UART_Object *obj, const UART_Attrs *attrs,
                                      UART_Transaction *transaction);
static void UART_edmaIsrTx(Edma_IntrHandle intrHandle, void *args);
static void UART_edmaIsrRx(Edma_IntrHandle intrHandle, void *args);
static void UART_edmaDoNothing(Edma_IntrHandle intrHandle, void *args);

/* EDMA Function Pointers */
UART_DmaFxns gUartDmaEdmaFxns =
{
    .dmaOpenFxn               = UART_edmaOpen,
    .dmaTransferWriteFxn      = UART_edmaTransferWrite,
    .dmaTransferReadFxn       = UART_edmaTransferRead,
    .dmaCloseFxn              = UART_edmaClose,
    .dmaDisableChannelFxn     = UART_edmaDisableChannel,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t UART_edmaOpen(UART_Handle uartHandle, void* uartDmaArgs)
{
    int32_t status = SystemP_SUCCESS;
    UartDma_EdmaArgs *edmaArgs = (UartDma_EdmaArgs *)uartDmaArgs;

    status = UART_edmaChInit(uartHandle, edmaArgs);

    if (status == SystemP_SUCCESS)
    {
        edmaArgs->isOpen = TRUE;
    }

    return status;
}

static int32_t UART_edmaChInit(UART_Handle handle, UartDma_EdmaArgs *edmaArgs)
{
    int32_t             status = SystemP_FAILURE;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh,dmaTxCh, tccRx, tccTx, tccDummy, paramRx, paramTx, paramDummy;
    uint32_t            isEdmaInterruptEnabled;
    Edma_IntrObject     *edmaIntrObjectRx;
    Edma_IntrObject     *edmaIntrObjectTx;
    Edma_IntrObject     *edmaIntrObjectDummy;
    EDMA_Handle         uartEdmaHandle;

    uartEdmaHandle = edmaArgs->drvHandle;
    edmaIntrObjectRx = &edmaArgs->edmaIntrObjRx;
    edmaIntrObjectTx = &edmaArgs->edmaIntrObjTx;
    edmaIntrObjectDummy = &edmaArgs->edmaIntrObjDummy;

    if(uartEdmaHandle != NULL)
    {
        status = SystemP_SUCCESS;

        /* Read base address of allocated EDMA instance */
        baseAddr = EDMA_getBaseAddr(uartEdmaHandle);

        /* Read the region ID of the EDMA instance */
        regionId = EDMA_getRegionId(uartEdmaHandle);

        /* Store the EDMA parameters */
        edmaArgs->edmaBaseAddr = baseAddr;
        edmaArgs->edmaRegionId = regionId;

        /* Check if interrupt is enabled */
        isEdmaInterruptEnabled = EDMA_isInterruptEnabled(uartEdmaHandle);
        DebugP_assert(isEdmaInterruptEnabled == TRUE);

        if((baseAddr != 0) && (regionId < SOC_EDMA_NUM_REGIONS))
        {
            /* Allocate EDMA channel for UART RX transfer */
            dmaRxCh = edmaArgs->edmaRxChId;
            status += EDMA_allocDmaChannel(uartEdmaHandle, &dmaRxCh);

            /* Allocate EDMA TCC for UART RX transfer */
            tccRx = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocTcc(uartEdmaHandle, &tccRx);

            /* Allocate a Param ID for UART RX transfer */
            paramRx = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocParam(uartEdmaHandle, &paramRx);

            if(status == SystemP_SUCCESS)
            {
                EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaRxCh, tccRx, paramRx, EDMA_UART_RX_EVT_QUEUE_NO);

                /* Register RX interrupt */
                edmaIntrObjectRx->tccNum = tccRx;
                edmaIntrObjectRx->cbFxn  = &UART_edmaIsrRx;
                edmaIntrObjectRx->appData = (void *) handle;
                status += EDMA_registerIntr(uartEdmaHandle, edmaIntrObjectRx);
                DebugP_assert(status == SystemP_SUCCESS);
            }

            if(status == SystemP_SUCCESS)
            {
                /* Store the EDMA parameters for UART RX*/
                edmaArgs->edmaRxParam = paramRx;
                edmaArgs->edmaRxChId  = dmaRxCh;
                edmaArgs->edmaTccRx   = tccRx;
            }

            /* Allocate EDMA channel for UART TX transfer */
            dmaTxCh = edmaArgs->edmaTxChId;
            status += EDMA_allocDmaChannel(uartEdmaHandle, &dmaTxCh);

            /* Allocate EDMA TCC for UART TX transfer */
            tccTx = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocTcc(uartEdmaHandle, &tccTx);

            /* Allocate a Param ID for UART TX transfer */
            paramTx = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocParam(uartEdmaHandle, &paramTx);

            if(status == SystemP_SUCCESS)
            {
                EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaTxCh, tccTx, paramTx, EDMA_UART_TX_EVT_QUEUE_NO);

                /* Register TX interrupt */
                edmaIntrObjectTx->tccNum = tccTx;
                edmaIntrObjectTx->cbFxn  = &UART_edmaIsrTx;
                edmaIntrObjectTx->appData = (void *) handle;
                status += EDMA_registerIntr(uartEdmaHandle, edmaIntrObjectTx);
                DebugP_assert(status == SystemP_SUCCESS);
            }

            if(status == SystemP_SUCCESS)
            {
                /* Store the EDMA parameters for UART TX*/
                edmaArgs->edmaTxParam = paramTx;
                edmaArgs->edmaTxChId  = dmaTxCh;
                edmaArgs->edmaTccTx   = tccTx;
            }

            /* Allocate EDMA TCC for Dummy param set */
            tccDummy = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocTcc(uartEdmaHandle, &tccDummy);

            /* Allocate a Param ID for UART TX transfer */
            paramDummy = EDMA_RESOURCE_ALLOC_ANY;
            status += EDMA_allocParam(uartEdmaHandle, &paramDummy);

            if(status == SystemP_SUCCESS)
            {

                /* Register Dummy interrupt */
                edmaIntrObjectDummy->tccNum = tccDummy;
                edmaIntrObjectDummy->cbFxn  = &UART_edmaDoNothing;
                edmaIntrObjectDummy->appData = (void *) handle;
                status += EDMA_registerIntr(uartEdmaHandle, edmaIntrObjectDummy);
                DebugP_assert(status == SystemP_SUCCESS);
            }

            if(status == SystemP_SUCCESS)
            {
                /* Store the EDMA parameters for UART TX*/
                edmaArgs->edmaDummyParam = paramDummy;
                edmaArgs->edmaTccDummy   = tccDummy;
            }

            edmaArgs->isOpen = TRUE;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    return status;
}

static int32_t UART_edmaTransferWrite(UART_Object *obj, const UART_Attrs *attrs,
                                      UART_Transaction *transaction)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaTxCh, tccTx, tccDummy, paramTx, paramDummy;
    uint32_t            isTxFifoEmpty, isEdmaEventPending;
    EDMACCPaRAMEntry    edmaTxParam, edmaDummyParam;
    UART_DmaConfig     *dmaConfig;
    UartDma_EdmaArgs   *edmaArgs;
    UART_DmaHandle      dmaHandle;

    isTxFifoEmpty = FALSE;
    isEdmaEventPending = FALSE;

    dmaHandle = obj->uartDmaHandle;
    dmaConfig = (UART_DmaConfig *)dmaHandle;
    edmaArgs = (UartDma_EdmaArgs *)dmaConfig->uartDmaArgs;

    /* Fetch the EDMA paramters for UART transfer */
    baseAddr               = edmaArgs->edmaBaseAddr;
    regionId               = edmaArgs->edmaRegionId;

    /* Fetch the EDMA paramters for UART TX transfer */
    dmaTxCh    = edmaArgs->edmaTxChId;
    tccTx      = edmaArgs->edmaTccTx;
    tccDummy   = edmaArgs->edmaTccDummy;
    paramTx    = edmaArgs->edmaTxParam;
    paramDummy = edmaArgs->edmaDummyParam;

    /* Initialize TX Param Set */
    EDMA_ccPaRAMEntry_init(&edmaTxParam);

    /* Transmit param set configuration */
    edmaTxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void*) transaction->buf);
    edmaTxParam.destAddr      = (uint32_t) SOC_virtToPhy((uint8_t *) attrs->baseAddr + UART_THR);
    edmaTxParam.aCnt          = (uint16_t) 1;
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

    /* Initialize TX Param Set */
    EDMA_ccPaRAMEntry_init(&edmaDummyParam);

    /* Dummy param set configuration */
    edmaDummyParam.aCnt          = (uint16_t) 1;
    edmaDummyParam.linkAddr      = 0xFFFFU;
    edmaDummyParam.opt          |= (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_STATIC_MASK |
                                   ((tccDummy << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    /* Write Tx param set */
    EDMA_setPaRAM(baseAddr, paramDummy, &edmaDummyParam);

    /* Link  dummy param ID */
    EDMA_linkChannel(baseAddr, paramTx, paramDummy);

    if(!UART_checkCharsAvailInFifo(attrs->baseAddr))
    {
        isTxFifoEmpty = TRUE;
    }

    if(EDMA_readEventStatusRegion(baseAddr, dmaTxCh))
    {
        isEdmaEventPending = TRUE;
    }

    /* Set event trigger to start UART TX transfer */
    EDMA_enableTransferRegion(baseAddr, regionId, dmaTxCh,
         EDMA_TRIG_MODE_EVENT);

    if (isTxFifoEmpty && !isEdmaEventPending)
    {
        /* Set manual trigger to start UART TX transfer */
        EDMA_enableTransferRegion(baseAddr, regionId, dmaTxCh,
         EDMA_TRIG_MODE_MANUAL);
    }

    return status;
}

static int32_t UART_edmaTransferRead(UART_Object *obj, const UART_Attrs *attrs,
                                     UART_Transaction *transaction)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh, tccRx, paramRx;
    EDMACCPaRAMEntry    edmaRxParam;
    UART_DmaConfig     *dmaConfig;
    UartDma_EdmaArgs   *edmaArgs;
    UART_DmaHandle      dmaHandle;

    dmaHandle = obj->uartDmaHandle;
    dmaConfig = (UART_DmaConfig *)dmaHandle;
    edmaArgs = (UartDma_EdmaArgs *)dmaConfig->uartDmaArgs;

    /* Fetch the EDMA paramters for UART transfer */
    baseAddr               = edmaArgs->edmaBaseAddr;
    regionId               = edmaArgs->edmaRegionId;

    /* Fetch the EDMA paramters for UART RX transfer */
    dmaRxCh    = edmaArgs->edmaRxChId;
    tccRx      = edmaArgs->edmaTccRx;
    paramRx    = edmaArgs->edmaRxParam;

    /* Initialize RX Param Set */
    EDMA_ccPaRAMEntry_init(&edmaRxParam);

    /* Receive param set configuration */
    edmaRxParam.srcAddr       = (uint32_t) SOC_virtToPhy((uint8_t *) attrs->baseAddr + UART_RHR);
    edmaRxParam.destAddr      = (uint32_t) SOC_virtToPhy((void*) transaction->buf);
    edmaRxParam.aCnt          = (uint16_t) 1;
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
    edmaRxParam.opt          |=
    (EDMA_OPT_TCINTEN_MASK | ((tccRx << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    /* Write Rx param set */
    EDMA_setPaRAM(baseAddr, paramRx, &edmaRxParam);

    /* Set event trigger to start UART RX transfer */
    EDMA_enableTransferRegion(baseAddr, regionId, dmaRxCh,
         EDMA_TRIG_MODE_EVENT);

    return status;
}

static int32_t UART_edmaClose(UART_Handle handle)
{
    int32_t            status = SystemP_SUCCESS;
    uint32_t           baseAddr, regionId;
    UART_Config        *config;
    UART_DmaHandle     dmaHandle;
    UART_DmaConfig     *dmaConfig;
    UartDma_EdmaArgs   *edmaArgs;
    EDMA_Handle         uartEdmaHandle;
    Edma_IntrObject     *edmaIntrObjectRx;
    Edma_IntrObject     *edmaIntrObjectTx;
    Edma_IntrObject     *edmaIntrObjectDummy;
    uint32_t            dmaRxCh,dmaTxCh, tccRx, tccTx, tccDummy, paramRx, paramTx, paramDummy;

    /* Check parameters */
    if(NULL == handle)
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config      = (UART_Config *) handle;
        dmaHandle   = config->object->uartDmaHandle;
        dmaConfig   = (UART_DmaConfig *)dmaHandle;
        edmaArgs    = (UartDma_EdmaArgs *)dmaConfig->uartDmaArgs;
        uartEdmaHandle = edmaArgs->drvHandle;

        /* Fetch the EDMA paramters */
        baseAddr         = edmaArgs->edmaBaseAddr;
        regionId         = edmaArgs->edmaRegionId;

        if (edmaArgs->isOpen != FALSE)
        {
            /* Fetch the EDMA paramters */
            dmaRxCh    = edmaArgs->edmaRxChId;
            tccRx      = edmaArgs->edmaTccRx;
            paramRx    = edmaArgs->edmaRxParam;
            edmaIntrObjectRx = &edmaArgs->edmaIntrObjRx;

            /* unregister Rx interrupt */
            status += EDMA_unregisterIntr(uartEdmaHandle, edmaIntrObjectRx);

            /* Free Rx channel */
            EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaRxCh, EDMA_TRIG_MODE_EVENT, tccRx, EDMA_UART_RX_EVT_QUEUE_NO);

            if(dmaRxCh != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeDmaChannel(uartEdmaHandle, &dmaRxCh);
            }
            if(tccRx != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeTcc(uartEdmaHandle, &tccRx);
            }
            if(paramRx != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeParam(uartEdmaHandle, &paramRx);
            }

            /* Fetch the EDMA paramters */
            dmaTxCh    = edmaArgs->edmaTxChId;
            tccTx      = edmaArgs->edmaTccTx;
            paramTx    = edmaArgs->edmaTxParam;
            edmaIntrObjectTx = &edmaArgs->edmaIntrObjTx;

            /* unregister Tx interrupt */
            status += EDMA_unregisterIntr(uartEdmaHandle, edmaIntrObjectTx);

            /* Free Tx channel */
            EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaTxCh, EDMA_TRIG_MODE_EVENT, tccTx, EDMA_UART_TX_EVT_QUEUE_NO);

            if(dmaTxCh != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeDmaChannel(uartEdmaHandle, &dmaTxCh);
            }
            if(tccTx != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeTcc(uartEdmaHandle, &tccTx);
            }
            if(paramTx != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeParam(uartEdmaHandle, &paramTx);
            }

            tccDummy   = edmaArgs->edmaTccDummy;
            paramDummy    = edmaArgs->edmaDummyParam;
            edmaIntrObjectDummy = &edmaArgs->edmaIntrObjDummy;

            /* unregister Dummy interrupt */
            status += EDMA_unregisterIntr(uartEdmaHandle, edmaIntrObjectDummy);

            if(tccDummy != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeTcc(uartEdmaHandle, &tccDummy);
            }
            if(paramDummy != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeParam(uartEdmaHandle, &paramDummy);
            }

            edmaArgs->isOpen = FALSE;
        }
    }

    return status;
}

static int32_t UART_edmaDisableChannel(UART_Handle handle,
                                       uint32_t isChannelTx)
{
    int32_t       status = SystemP_SUCCESS;
    uint32_t      baseAddr, regionId, dmaCh;
    UART_Config   *config;
    UART_DmaHandle dmaHandle;
    UART_DmaConfig *dmaConfig;
    UartDma_EdmaArgs *edmaArgs;

    if(NULL != handle)
    {
        config = (UART_Config *) handle;
        dmaHandle = config->object->uartDmaHandle;
        dmaConfig = (UART_DmaConfig *)dmaHandle;
        edmaArgs = (UartDma_EdmaArgs *)dmaConfig->uartDmaArgs;

        baseAddr               = edmaArgs->edmaBaseAddr;
        regionId               = edmaArgs->edmaRegionId;

        /* Disable Channel */
        if (isChannelTx == TRUE)
        {
            dmaCh    = edmaArgs->edmaTxChId;
        }
        else
        {
            dmaCh    = edmaArgs->edmaRxChId;
        }

        /* Stop the UART event transfer */
        EDMA_disableTransferRegion(baseAddr, regionId, dmaCh,
            EDMA_TRIG_MODE_EVENT);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static void UART_edmaIsrTx(Edma_IntrHandle intrHandle, void *args)
{
    UART_Config       *config;
    UART_Object       *obj;

    /* Check parameters */
    if(NULL != args)
    {
        config = (UART_Config *) args;
        obj = config->object;
        DebugP_assert(NULL != obj);

        obj->writeTrans->status = UART_TRANSFER_STATUS_SUCCESS;

        /*
        * Post transfer Sem in case of bloacking transfer.
        * Call the callback function in case of Callback mode.
        */
        if (obj->prms.writeMode == UART_TRANSFER_MODE_CALLBACK)
        {
            obj->prms.writeCallbackFxn((UART_Handle) config, obj->writeTrans);
        }
        else
        {
            (void)SemaphoreP_post(&obj->writeTransferSemObj);
        }
        obj->writeTrans = NULL;
    }
    return;
}

static void UART_edmaIsrRx(Edma_IntrHandle intrHandle, void *args)
{
    UART_Config       *config;
    UART_Object       *obj;

    /* Check parameters */
    if(NULL != args)
    {
        config = (UART_Config *) args;
        obj = config->object;
        DebugP_assert(NULL != obj);

        obj->readTrans->status = UART_TRANSFER_STATUS_SUCCESS;

        /*
        * Post transfer Sem in case of bloacking transfer.
        * Call the callback function in case of Callback mode.
        */
        if (obj->prms.readMode == UART_TRANSFER_MODE_CALLBACK)
        {
            obj->prms.readCallbackFxn((UART_Handle) config, obj->readTrans);
        }
        else
        {
            (void)SemaphoreP_post(&obj->readTransferSemObj);
        }
        obj->readTrans = NULL;
    }

    return;
}

static void UART_edmaDoNothing(Edma_IntrHandle intrHandle, void *args)
{
    /* No implementation required */
}

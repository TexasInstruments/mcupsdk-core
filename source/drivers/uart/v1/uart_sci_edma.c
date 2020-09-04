/*
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file   uart_sci_edma.c
 *
 *  \brief  This file contains the implementation of UART Dma mode
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/soc.h>
#include <drivers/uart.h>
#include <drivers/uart/v1/uart_sci_edma.h>
#include <drivers/edma.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Event queue to be used  */
#define EDMA_UART_EVT_QUEUE_NO      (0U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
static inline void UART_disableRxDma(CSL_sciRegs *pSCIRegs);
static inline void UART_enableRxDma(CSL_sciRegs *pSCIRegs);
static inline void UART_disableTxDma(CSL_sciRegs *pSCIRegs);
static inline void UART_enableTxDma(CSL_sciRegs *pSCIRegs);
static void UART_rxEdmaIsrFxn(Edma_IntrHandle intrHandle, void *args);
static void UART_txEdmaIsrFxn(Edma_IntrHandle intrHandle, void *args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


int32_t UART_edmaChannelConfig(UART_Handle uartHandle, uint32_t edmaInst)
{
    uint32_t            baseAddr, regionId, dmaCh, tcc, param;
    int32_t             status = SystemP_SUCCESS;
    UART_Object         *object = ((UART_Config *)uartHandle)->object;

    UART_EdmaParams     *edmaParams;
    EDMA_Handle         uartEdmaHandle = EDMA_getHandle(edmaInst);
    Edma_IntrObject     *edmaIntrObject;

    /* Read base address of allocated EDMA instance */
    baseAddr = EDMA_getBaseAddr(uartEdmaHandle);
    if(baseAddr == 0)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        /* Read the region ID of the EDMA instance */
        regionId = EDMA_getRegionId(uartEdmaHandle);

        if(regionId > SOC_EDMA_NUM_REGIONS)
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Allocate EDMA channel for UART Rx Channel */
        edmaParams = &object->rxEdmaParams;
        dmaCh = object->prms.rxEvtNum;
        status += EDMA_allocDmaChannel(uartEdmaHandle, &dmaCh);
        if(status != SystemP_SUCCESS)
        {
            dmaCh = EDMA_RESOURCE_ALLOC_ANY;
        }

        /* Allocate EDMA TCC for UART transfer */
        tcc = EDMA_RESOURCE_ALLOC_ANY;
        status += EDMA_allocTcc(uartEdmaHandle, &tcc);

        /* Allocate a Param ID for UART transfer */
        param = EDMA_RESOURCE_ALLOC_ANY;
        status += EDMA_allocParam(uartEdmaHandle, &param);
        if(status == SystemP_SUCCESS)
        {
            EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaCh, tcc, param, EDMA_UART_EVT_QUEUE_NO);

            /* Store the EDMA paramters and handle*/
            object->uartEdmaHandle  = uartEdmaHandle;
            edmaParams->edmaBaseAddr = baseAddr;
            edmaParams->edmaRegionId = regionId;
            edmaParams->edmaParam = param;
            edmaParams->edmaChId = dmaCh;
            edmaParams->edmaTcc = tcc;
            edmaParams->isIntEnabled = EDMA_isInterruptEnabled(uartEdmaHandle);
            if (edmaParams->isIntEnabled == TRUE)
            {
                edmaIntrObject = &edmaParams->edmaIntrObj;
                edmaIntrObject->tccNum = tcc;
                edmaIntrObject->cbFxn = &UART_rxEdmaIsrFxn;
                edmaIntrObject->appData = (void *)uartHandle;
                status = EDMA_registerIntr(uartEdmaHandle, edmaIntrObject);
            }
        }

        if(status != SystemP_SUCCESS)
        {
            if(dmaCh != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeDmaChannel(uartEdmaHandle, &dmaCh);
            }
            if(tcc != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeTcc(uartEdmaHandle, &tcc);
            }
            if(param != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeParam(uartEdmaHandle, &param);
            }
        }
    }
    if(status == SystemP_SUCCESS)
    {
        /* Allocate EDMA channel for UART Tx Channel */
        edmaParams = &object->txEdmaParams;
        dmaCh = object->prms.txEvtNum;
        status += EDMA_allocDmaChannel(uartEdmaHandle, &dmaCh);
        if(status != SystemP_SUCCESS)
        {
            dmaCh = EDMA_RESOURCE_ALLOC_ANY;
        }

        /* Allocate EDMA TCC for UART transfer */
        tcc = EDMA_RESOURCE_ALLOC_ANY;
        status += EDMA_allocTcc(uartEdmaHandle, &tcc);

        /* Allocate a Param ID for UART transfer */
        param = EDMA_RESOURCE_ALLOC_ANY;
        status += EDMA_allocParam(uartEdmaHandle, &param);
        if(status == SystemP_SUCCESS)
        {
            EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaCh, tcc, param, EDMA_UART_EVT_QUEUE_NO);

            /* Store the EDMA paramters and handle*/
            object->uartEdmaHandle  = uartEdmaHandle;
            edmaParams->edmaBaseAddr = baseAddr;
            edmaParams->edmaRegionId = regionId;
            edmaParams->edmaParam = param;
            edmaParams->edmaChId = dmaCh;
            edmaParams->edmaTcc = tcc;
            edmaParams->isIntEnabled = EDMA_isInterruptEnabled(uartEdmaHandle);
            if (edmaParams->isIntEnabled == TRUE)
            {
                edmaIntrObject = &edmaParams->edmaIntrObj;
                edmaIntrObject->tccNum = tcc;
                edmaIntrObject->cbFxn = &UART_txEdmaIsrFxn;
                edmaIntrObject->appData = (void *)uartHandle;
                status = EDMA_registerIntr(uartEdmaHandle, edmaIntrObject);
            }
        }

        if(status != SystemP_SUCCESS)
        {
            if(dmaCh != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeDmaChannel(uartEdmaHandle, &dmaCh);
            }
            if(tcc != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeTcc(uartEdmaHandle, &tcc);
            }
            if(param != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeParam(uartEdmaHandle, &param);
            }
        }
    }
    return status;
}

static void UART_rxEdmaIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    UART_Config        *config;
    UART_Object        *object;

    if(NULL != args)
    {
        config = (UART_Config *) args;
        object = (UART_Object *) config->object;
        DebugP_assert(NULL != object);
        DebugP_assert(NULL != object->pSCIRegs);
        /*
            * Post transfer Sem in case of bloacking transfer.
            * Call the callback function in case of Callback mode.
            */
        if(object->prms.readMode == UART_TRANSFER_MODE_CALLBACK)
        {
            object->readTrans->status = UART_TRANSFER_STATUS_SUCCESS;
            object->prms.readCallbackFxn((UART_Handle) config, object->readTrans);
        }
        else
        {
            (void)SemaphoreP_post(&object->readTransferSemObj);
        }
        object->readTrans = NULL;
    }
}

int32_t UART_readDma(UART_Config    *config,
                    UART_Object      *object,
                    UART_Attrs const *attrs,
                    UART_Transaction *trans)
{
    int32_t             status = SystemP_SUCCESS, semStatus;
    UART_EdmaParams     *edmaParams = &object->rxEdmaParams;
    EDMACCPaRAMEntry   edmaParam;

    /* Disable the UART Receive DMA mode */
    UART_disableRxDma(object->pSCIRegs);

    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) (&object->pSCIRegs->SCIRD);
    edmaParam.destAddr      = (uint32_t) SOC_virtToPhy(trans->buf);
    edmaParam.aCnt          = (uint16_t) 1;
    edmaParam.bCnt          = (uint16_t) trans->count;
    edmaParam.cCnt          = (uint16_t) 1;
    edmaParam.bCntReload    = (uint16_t) 0;
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(1);
    edmaParam.srcCIdx       = (int16_t) 0;
    edmaParam.destCIdx      = (int16_t) 0;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(1);
    edmaParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK |
         (((edmaParams->edmaTcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(edmaParams->edmaBaseAddr, edmaParams->edmaParam, &edmaParam);

    /* Set event trigger to start UART transfer */
    EDMA_enableTransferRegion(edmaParams->edmaBaseAddr, edmaParams->edmaRegionId, edmaParams->edmaChId,
             EDMA_TRIG_MODE_EVENT);

    /* Enable the UART to operate in receive DMA mode */
    UART_enableRxDma(object->pSCIRegs);

    if (edmaParams->isIntEnabled == TRUE)
    {
        if(object->prms.readMode == UART_TRANSFER_MODE_BLOCKING)
        {
            /* Pend on lock and wait for Hwi to finish. */
            semStatus = SemaphoreP_pend(&object->readTransferSemObj, trans->timeout);
            if(semStatus == SystemP_SUCCESS)
            {
                if(trans->status == UART_TRANSFER_STATUS_SUCCESS)
                {
                    status = SystemP_SUCCESS;
                }
                else
                {
                    status = SystemP_FAILURE;
                }
            }
            else
            {
                /* Cancel the DMA transfer and return error. */
            }
        }
        else
        {
            /*
            * for callback mode, immediately return SUCCESS,
            * once the transaction is done, callback function
            * will return the transaction status and actual
            * read count
            */
            trans->count = 0U;
            status = SystemP_SUCCESS;
        }
    }
    else
    {
        /* Poll for transfer completion */
        while(EDMA_readIntrStatusRegion(edmaParams->edmaBaseAddr, edmaParams->edmaRegionId, edmaParams->edmaTcc) != 1);
        EDMA_clrIntrRegion(edmaParams->edmaBaseAddr, edmaParams->edmaRegionId, edmaParams->edmaTcc);

        trans->status = UART_TRANSFER_STATUS_SUCCESS;
        object->readTrans = NULL;
    }

    return status;
}

static void UART_txEdmaIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    UART_Config        *config;
    UART_Object        *object;

    if(NULL != args)
    {
        config = (UART_Config *) args;
        object = (UART_Object *) config->object;
        DebugP_assert(NULL != object);
        DebugP_assert(NULL != object->pSCIRegs);
        /*
            * Post transfer Sem in case of bloacking transfer.
            * Call the callback function in case of Callback mode.
            */
        if(object->prms.writeMode == UART_TRANSFER_MODE_CALLBACK)
        {
            object->writeTrans->status = UART_TRANSFER_STATUS_SUCCESS;
            object->prms.writeCallbackFxn((UART_Handle) config, object->writeTrans);
        }
        else
        {
            (void)SemaphoreP_post(&object->writeTransferSemObj);
        }
        object->writeTrans = NULL;
    }
}

int32_t UART_writeDma(UART_Config    *config,
                    UART_Object      *object,
                    UART_Attrs const *attrs,
                    UART_Transaction *trans)
{
    int32_t             status = SystemP_SUCCESS, semStatus;
    UART_EdmaParams     *edmaParams = &object->txEdmaParams;
    EDMACCPaRAMEntry   edmaParam;

    /* Disable the UART Receive DMA mode */
    UART_disableTxDma(object->pSCIRegs);

    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy(trans->buf);
    edmaParam.destAddr      = (uint32_t) (&object->pSCIRegs->SCITD);
    edmaParam.aCnt          = (uint16_t) 1;
    edmaParam.bCnt          = (uint16_t) trans->count;
    edmaParam.cCnt          = (uint16_t) 1;
    edmaParam.bCntReload    = (uint16_t) 0;
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(1);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
    edmaParam.srcCIdx       = (int16_t) 0;
    edmaParam.destCIdx      = (int16_t) 0;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(1);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK |
         (((edmaParams->edmaTcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(edmaParams->edmaBaseAddr, edmaParams->edmaParam, &edmaParam);

    /* Set event trigger to start UART transfer */
    EDMA_enableTransferRegion(edmaParams->edmaBaseAddr, edmaParams->edmaRegionId, edmaParams->edmaChId,
             EDMA_TRIG_MODE_EVENT);

    /* Enable the UART to operate in receive DMA mode */
    UART_enableTxDma(object->pSCIRegs);
    if (edmaParams->isIntEnabled == TRUE)
    {
        if(object->prms.readMode == UART_TRANSFER_MODE_BLOCKING)
        {
            /* Pend on lock and wait for Hwi to finish. */
            semStatus = SemaphoreP_pend(&object->writeTransferSemObj, trans->timeout);
            if(semStatus == SystemP_SUCCESS)
            {
                if(trans->status == UART_TRANSFER_STATUS_SUCCESS)
                {
                    status = SystemP_SUCCESS;
                }
                else
                {
                    status = SystemP_FAILURE;
                }
            }
            else
            {
                /* Cancel the DMA transfer and return error. */
            }
        }
        else
        {
            /*
            * for callback mode, immediately return SUCCESS,
            * once the transaction is done, callback function
            * will return the transaction status and actual
            * read count
            */
            trans->count = 0U;
            status = SystemP_SUCCESS;
        }
    }
    else
    {
        /* Poll for transfer completion */
        while(EDMA_readIntrStatusRegion(edmaParams->edmaBaseAddr, edmaParams->edmaRegionId, edmaParams->edmaTcc) != 1);
        EDMA_clrIntrRegion(edmaParams->edmaBaseAddr, edmaParams->edmaRegionId, edmaParams->edmaTcc);

        trans->status = UART_TRANSFER_STATUS_SUCCESS;
        object->writeTrans = NULL;
    }

    return status;
}

static inline void UART_disableRxDma(CSL_sciRegs *pSCIRegs)
{
    /* Disable the Rx DMA All and Rx DMA */
    CSL_REG_WR(&pSCIRegs->SCICLEARINT, CSL_FMK(SCI_SCICLEARINT_CLR_RX_DMA_ALL, 1U));
    CSL_REG_WR(&pSCIRegs->SCICLEARINT, CSL_FMK(SCI_SCICLEARINT_CLR_RX_DMA, 1U));
}

static inline void UART_enableRxDma(CSL_sciRegs *pSCIRegs)
{
    /* Enable the Rx DMA All and Rx DMA */
    CSL_REG_WR(&pSCIRegs->SCISETINT, CSL_FMK(SCI_SCISETINT_SET_RX_DMA_ALL, 1U));
    CSL_REG_WR(&pSCIRegs->SCISETINT, CSL_FMK(SCI_SCISETINT_SET_RX_DMA, 1U));
}

static inline void UART_disableTxDma(CSL_sciRegs *pSCIRegs)
{
    /* Disable the Transmit DMA */
    CSL_REG_WR(&pSCIRegs->SCICLEARINT, CSL_FMK(SCI_SCICLEARINT_CLR_TX_DMA, 1U));
}

static inline void UART_enableTxDma(CSL_sciRegs *pSCIRegs)
{
    /* Enable Transmit DMA */
    CSL_REG_WR(&pSCIRegs->SCISETINT, CSL_FMK(SCI_SCISETINT_SET_TX_DMA, 1U));
}

/*
 *  Copyright (C) 2023-24 Texas Instruments Incorporated
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
 *  \file mcspi_v0_lld.c
 *
 *  \brief File containing MCSPI Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <string.h>
#include <drivers/mcspi/v0/lld/mcspi_lld.h>
#include <drivers/mcspi/v0/lld/dma/mcspi_dma.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Driver internal functions */
static void MCSPI_initiateLastChunkTransfer(MCSPILLD_Handle hMcspi,
                                            MCSPI_ChObject *chObj,
                                            const MCSPI_Transaction *transaction);

static inline void MCSPI_fifoWrite(uint32_t baseAddr,
                                   MCSPI_ChObject *chObj,
                                   uint32_t transferLength);

static inline void MCSPI_fifoRead(uint32_t baseAddr,
                                  MCSPI_ChObject *chObj,
                                  uint32_t transferLength);

static inline void MCSPI_fifoReadMultiWord(uint32_t baseAddr,
                                           MCSPI_ChObject *chObj,
                                           uint32_t transferLength);

static inline void MCSPI_fifoWriteMultiWord(uint32_t baseAddr,
                                           MCSPI_ChObject *chObj,
                                           uint32_t transferLength);

static uint32_t MCSPI_getDataWidthBitMask(uint32_t dataWidth);
static uint32_t Spi_mcspiGetRxMask(uint32_t csNum);
static uint32_t Spi_mcspiGetTxMask(uint32_t csNum);
static void MCSPI_setClkConfig(uint32_t baseAddr,
                               uint32_t chNum,
                               uint32_t inputClkFreq,
                               uint32_t bitRate);

static inline uint8_t *MCSPI_fifoWrite8(uint32_t  baseAddr,
                                        uint32_t  chNum,
                                        uint8_t  *bufPtr,
                                        uint32_t  transferLength);

static inline uint16_t *MCSPI_fifoWrite16(uint32_t  baseAddr,
                                          uint32_t  chNum,
                                          uint16_t *bufPtr,
                                          uint32_t  transferLength);

static inline uint32_t *MCSPI_fifoWrite32(uint32_t  baseAddr,
                                          uint32_t  chNum,
                                          uint32_t *bufPtr,
                                          uint32_t  transferLength);

static inline uint8_t *MCSPI_fifoRead8(uint32_t  baseAddr,
                                       uint32_t  chNum,
                                       uint8_t  *bufPtr,
                                       uint32_t  transferLength,
                                       uint32_t  dataWidthBitMask);

static inline uint16_t *MCSPI_fifoRead16(uint32_t  baseAddr,
                                         uint32_t  chNum,
                                         uint16_t  *bufPtr,
                                         uint32_t  transferLength,
                                         uint32_t  dataWidthBitMask);

static inline uint32_t *MCSPI_fifoRead32(uint32_t  baseAddr,
                                         uint32_t  chNum,
                                         uint32_t  *bufPtr,
                                         uint32_t  transferLength,
                                         uint32_t  dataWidthBitMask);

static inline void MCSPI_fifoWriteDefault(uint32_t baseAddr,
                                          uint32_t chNum,
                                          uint32_t defaultTxData,
                                          uint32_t transferLength);

static inline void MCSPI_fifoReadDiscard(uint32_t baseAddr,
                                         uint32_t chNum,
                                         uint32_t transferLength);

static inline int32_t MCSPI_lld_isOperModeValid(uint32_t operMode);
static inline int32_t MCSPI_lld_isChModeValid(uint32_t chMode);
static inline int32_t MCSPI_lld_isPinModeValid(uint32_t pinMode);
static inline int32_t MCSPI_lld_isInitDelayValid(uint32_t initDelay);
static inline int32_t MCSPI_lld_isMsModeValid(uint32_t msMode);
static inline int32_t MCSPI_lld_isDataSizeValid(uint32_t dataSize);
static inline int32_t MCSPI_lld_isHandleValid(MCSPI_DmaHandle handle);
static inline int32_t MCSPI_lld_isParameterValid(uint32_t handleParameters);
static inline int32_t MCSPI_lld_isChannelValid(uint32_t channel);
static inline int32_t MCSPI_lld_isChCfgValid(const MCSPI_ChConfig *chCfg);
static int32_t MCSPI_lld_chConfig(MCSPILLD_Handle hMcspi,
                                   const MCSPI_ChConfig *chCfg,
                                   uint32_t chCnt);

static uint32_t MCSPI_continueTxRx(MCSPILLD_Handle hMcspi,
                                   MCSPI_ChObject *chObj,
                                   const MCSPI_Transaction *transaction);

static int32_t MCSPI_transferControllerPoll(MCSPILLD_Handle hMcspi,
                                   MCSPI_ChObject *chObj,
                                   const MCSPI_Transaction *transaction);

static void MCSPI_transferControllerIntr(MCSPILLD_Handle hMcspi,
                                         MCSPI_ChObject *chObj);

static uint32_t MCSPI_continuePeripheralTxRx(MCSPILLD_Handle hMcspi,
                                   MCSPI_ChObject *chObj,
                                   const MCSPI_Transaction *transaction);

static int32_t MCSPI_transferPeripheralPoll(MCSPILLD_Handle hMcspi,
                                   MCSPI_ChObject *chObj,
                                   const MCSPI_Transaction *transaction);

static void MCSPI_transferPeripheralIntr(MCSPILLD_Handle hMcspi,
                                         MCSPI_ChObject *chObj);

static void MCSPI_configInstance(MCSPILLD_Handle hMcspi);
static void MCSPI_setChConfig(MCSPILLD_Handle hMcspi, MCSPI_ChObject *chObj);
static int32_t MCSPI_checkChConfig(MCSPILLD_Object *obj,
                                   const MCSPI_ChConfig *chCfg);

static void MCSPI_setFifoConfig(MCSPILLD_Handle hMcspi,
                                MCSPI_ChObject *chObj,
                                uint32_t baseAddr,
                                uint32_t numWordsTxRx);

static void MCSPI_setPeripheralFifoConfig(MCSPI_ChObject *chObj,
                                          uint32_t baseAddr,
                                          uint32_t numWordsTxRx);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t MCSPI_lld_init(MCSPILLD_Handle hMcspi)
{
    int32_t                status = MCSPI_STATUS_SUCCESS;
    uint32_t               chCnt;
    MCSPILLD_InitHandle    hMcspiInit;
    const MCSPI_ChConfig   *chCfg;

    if((hMcspi != NULL) && (hMcspi->hMcspiInit != NULL))
    {
        if(hMcspi->state != MCSPI_STATE_RESET)
        {
            status = MCSPI_INVALID_STATE;
        }
    }
    else
    {
        status = MCSPI_INVALID_PARAM;
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        hMcspi->state = MCSPI_STATE_BUSY;
        hMcspiInit = hMcspi->hMcspiInit;

        /* Check the MCSPI Input parameters */
        status =  MCSPI_lld_isBaseAddrValid(hMcspi->baseAddr);
        status += MCSPI_lld_isParameterValid(hMcspiInit->inputClkFreq);
        status += MCSPI_lld_isOperModeValid(hMcspiInit->operMode);
        status += MCSPI_lld_isChModeValid(hMcspiInit->chMode);
        status += MCSPI_lld_isPinModeValid(hMcspiInit->pinMode);
        status += MCSPI_lld_isInitDelayValid(hMcspiInit->initDelay);
        status += MCSPI_lld_isMsModeValid(hMcspiInit->msMode);

        if(status != MCSPI_STATUS_SUCCESS)
        {
            status = MCSPI_INVALID_PARAM;
        }
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        /* Configure the MCSPI instance parameters */
        MCSPI_configInstance(hMcspi);

        /* Channel configuration */
        for(chCnt = 0U; chCnt < MCSPI_MAX_NUM_CHANNELS; chCnt++)
        {
            if(hMcspiInit->chEnabled[chCnt] == TRUE)
            {
                chCfg = hMcspiInit->chObj[chCnt].chCfg;

                status =  MCSPI_lld_isChCfgValid(chCfg);
                status += MCSPI_lld_isChannelValid(chCfg->chNum);

                if(MCSPI_STATUS_SUCCESS == status)
                {
                    status += MCSPI_lld_chConfig(hMcspi,
                                hMcspiInit->chObj[chCnt].chCfg,
                                chCnt);
                }
            }
        }
        if(status != MCSPI_STATUS_SUCCESS)
        {
            status = MCSPI_INVALID_PARAM;
        }
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        hMcspi->state = MCSPI_STATE_READY;
    }
    else
    {
        /* Free-up resources in case of error */
        (void)MCSPI_lld_deInit(hMcspi);
    }

    return status;
}

int32_t MCSPI_lld_initDma(MCSPILLD_Handle hMcspi)
{
    uint32_t                   chCnt;
    MCSPILLD_InitHandle        hMcspiInit;
    const MCSPI_ChConfig      *chCfg;
    int32_t  status =          MCSPI_STATUS_SUCCESS;

    if((hMcspi != NULL) && (hMcspi->hMcspiInit != NULL))
    {
        if(hMcspi->state != MCSPI_STATE_RESET)
        {
            status = MCSPI_INVALID_STATE;
        }
    }
    else
    {
        status = MCSPI_INVALID_PARAM;
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        hMcspi->state = MCSPI_STATE_BUSY;
        hMcspiInit = hMcspi->hMcspiInit;

        /* Check the MCSPI Input parameters */
        status =  MCSPI_lld_isBaseAddrValid(hMcspi->baseAddr);
        status += MCSPI_lld_isParameterValid(hMcspiInit->inputClkFreq);
        status += MCSPI_lld_isHandleValid(hMcspiInit->mcspiDmaHandle);
        status += MCSPI_lld_isOperModeValid(hMcspiInit->operMode);
        status += MCSPI_lld_isChModeValid(hMcspiInit->chMode);
        status += MCSPI_lld_isPinModeValid(hMcspiInit->pinMode);
        status += MCSPI_lld_isInitDelayValid(hMcspiInit->initDelay);
        status += MCSPI_lld_isMsModeValid(hMcspiInit->msMode);

        if(status != MCSPI_STATUS_SUCCESS)
        {
            status = MCSPI_INVALID_PARAM;
        }
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        status = MCSPI_lld_dmaInit(hMcspiInit->mcspiDmaHandle);
        /* Configure the MCSPI instance parameters */
        MCSPI_configInstance(hMcspi);

        /* Channel configuration */
        for(chCnt = 0U; chCnt < MCSPI_MAX_NUM_CHANNELS; chCnt++)
        {
            if(hMcspiInit->chEnabled[chCnt] == TRUE)
            {
                chCfg = hMcspiInit->chObj[chCnt].chCfg;

                status =  MCSPI_lld_isChCfgValid(chCfg);
                status += MCSPI_lld_isChannelValid(chCfg->chNum);

                if(MCSPI_STATUS_SUCCESS == status)
                {
                    status += MCSPI_lld_chConfig(hMcspi, chCfg, chCnt);
                    status += MCSPI_lld_dmaChInit(hMcspi, chCnt);
                }
            }
        }

        if(status == MCSPI_STATUS_SUCCESS)
        {
            hMcspi->state = MCSPI_STATE_READY;
        }
        if((status == MCSPI_STATUS_FAILURE) || (status == MCSPI_INVALID_PARAM))
        {
            /* Free-up resources in case of error */
            (void)MCSPI_lld_deInitDma(hMcspi);
        }
    }

    return status;
}

int32_t MCSPI_lld_deInit(MCSPILLD_Handle hMcspi)
{
    int32_t             status = MCSPI_STATUS_SUCCESS;
    uint32_t            baseAddr;

    if(NULL != hMcspi)
    {
        baseAddr      = hMcspi->baseAddr;
        hMcspi->state = MCSPI_STATE_BUSY;

        /* Reset MCSPI */
        MCSPI_reset(baseAddr);

        hMcspi->state = MCSPI_STATE_RESET;
    }
    else
    {
        status = MCSPI_INVALID_PARAM;
    }

    return status;
}

int32_t MCSPI_lld_deInitDma(MCSPILLD_Handle hMcspi)
{
    int32_t                status = MCSPI_STATUS_SUCCESS;
    uint32_t               baseAddr;
    uint32_t               chCnt;
    MCSPILLD_InitHandle    hMcspiInit;

    if(NULL != hMcspi)
    {
        baseAddr      = hMcspi->baseAddr;
        hMcspi->state = MCSPI_STATE_BUSY;
        hMcspiInit    = hMcspi->hMcspiInit;

        /* Reset MCSPI */
        MCSPI_reset(baseAddr);

        for(chCnt = 0U; chCnt < MCSPI_MAX_NUM_CHANNELS; chCnt++)
        {
            if(hMcspiInit->chEnabled[chCnt] == TRUE)
            {
                status += MCSPI_lld_dmaDeInit(hMcspi,
                                            hMcspiInit->chObj[chCnt].chCfg,
                                            chCnt);
            }
        }

        hMcspi->state = MCSPI_STATE_RESET;
    }
    else
    {
        status = MCSPI_INVALID_PARAM;
    }

    return status;
}

int32_t MCSPI_lld_write(MCSPILLD_Handle hMcspi, void *txBuf, uint32_t count,
                        uint32_t timeout, const MCSPI_ExtendedParams *extendedParams)
{
    int32_t                status = MCSPI_STATUS_SUCCESS;
    MCSPI_Transaction     *transaction;
    uint32_t               baseAddr, chNum;
    MCSPILLD_InitHandle    hMcspiInit;
    MCSPI_ChObject        *chObj;

    /* Check parameters */
    if((NULL == hMcspi) || (NULL == hMcspi->hMcspiInit))
    {
        status = MCSPI_INVALID_PARAM;
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        transaction = &hMcspi->transaction;
        /* Check if any transaction is in progress */
        if(hMcspi->state == MCSPI_STATE_READY)
        {
            hMcspi->state = MCSPI_STATE_BUSY;
            /* Initialize transaction with default parameters */
            MCSPI_lld_Transaction_init(transaction);

            if(extendedParams != NULL)
            {
                transaction->channel   = extendedParams->channel;
                transaction->dataSize  = extendedParams->dataSize;
                transaction->csDisable = extendedParams->csDisable;
                transaction->args      = extendedParams->args;
            }

            baseAddr             = hMcspi->baseAddr;
            hMcspiInit           = hMcspi->hMcspiInit;
            transaction->count   = count;
            transaction->timeout = timeout;
            transaction->txBuf   = txBuf;

            status =  MCSPI_lld_isParameterValid(count);
            status += MCSPI_lld_isParameterValid(timeout);
            status += MCSPI_lld_isChannelValid(transaction->channel);
            status += MCSPI_lld_isDataSizeValid(transaction->dataSize);
            if(status != MCSPI_STATUS_SUCCESS)
            {
                status = MCSPI_INVALID_PARAM;
            }

            /* Check if the channel is configured */
            if(TRUE != hMcspiInit->chObj[transaction->channel].isOpen)
            {
                /* Channel not configured */
                status = MCSPI_TRANSFER_FAILED;
            }
        }
        else
        {
            status = MCSPI_STATUS_BUSY;
        }
    }

    if (MCSPI_STATUS_SUCCESS == status)
    {
        /* Reset counter and other params */
        chNum = transaction->channel;
        chObj = &hMcspiInit->chObj[chNum];
        chObj->curTxBufPtr = (uint8_t *) transaction->txBuf;
        chObj->curRxBufPtr = NULL;
        chObj->curTxWords  = 0U;
        chObj->curRxWords  = 0U;

        /* Initialize channel dataSize */
        MCSPI_setChDataSize(baseAddr, chObj, transaction->dataSize,
                            transaction->csDisable);

        if(MCSPI_MS_MODE_CONTROLLER == hMcspiInit->msMode)
        {
            /* Enable FIFO*/
            MCSPI_setFifoConfig(hMcspi, chObj, baseAddr, transaction->count);
            MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);

            status = MCSPI_transferControllerPoll(hMcspi, chObj, transaction);
        }
        else
        {
            /* Enable FIFO*/
            MCSPI_setPeripheralFifoConfig(chObj, baseAddr, count);
            MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);

            status = MCSPI_transferPeripheralPoll(hMcspi, chObj, transaction);
        }

        if(status == MCSPI_TIMEOUT)
        {
            status = MCSPI_TRANSFER_TIMEOUT;
        }
        else if ((status == MCSPI_TRANSFER_CANCELLED) && (hMcspi->errorFlag != 0U))
        {
            status = MCSPI_TRANSFER_FAILED;
        }
        else
        {
            /* success case */
            status  = MCSPI_TRANSFER_COMPLETED;
        }

        hMcspi->state = MCSPI_STATE_READY;
    }

    return (status);
}

int32_t MCSPI_lld_writeIntr(MCSPILLD_Handle hMcspi, void *txBuf, uint32_t count, uint32_t timeout,
                            const MCSPI_ExtendedParams *extendedParams)
{
    int32_t                status = MCSPI_STATUS_SUCCESS;
    MCSPI_Transaction     *transaction;
    uint32_t               baseAddr, chNum;
    MCSPILLD_InitHandle    hMcspiInit;
    MCSPI_ChObject        *chObj;

    /* Check parameters */
    if((NULL == hMcspi) || (NULL == hMcspi->hMcspiInit))
    {
        status = MCSPI_INVALID_PARAM;
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        transaction = &hMcspi->transaction;
        /* Check if any transaction is in progress */
        if(hMcspi->state == MCSPI_STATE_READY)
        {
            hMcspi->state = MCSPI_STATE_BUSY;
            /* Initialize transaction with default parameters */
            MCSPI_lld_Transaction_init(transaction);
            if(extendedParams != NULL)
            {
                transaction->channel   = extendedParams->channel;
                transaction->dataSize  = extendedParams->dataSize;
                transaction->csDisable = extendedParams->csDisable;
                transaction->args      = extendedParams->args;
            }

            baseAddr             = hMcspi->baseAddr;
            hMcspiInit           = hMcspi->hMcspiInit;
            transaction->count   = count;
            transaction->timeout = timeout;
            transaction->txBuf   = txBuf;

            status =  MCSPI_lld_isParameterValid(count);
            status += MCSPI_lld_isParameterValid(timeout);
            status += MCSPI_lld_isChannelValid(transaction->channel);
            status += MCSPI_lld_isDataSizeValid(transaction->dataSize);
            if(status != MCSPI_STATUS_SUCCESS)
            {
                status = MCSPI_INVALID_PARAM;
            }

            /* Check if the channel is configured */
            if(TRUE != hMcspiInit->chObj[transaction->channel].isOpen)
            {
                /* Channel not configured */
                status = MCSPI_TRANSFER_FAILED;
            }
        }
        else
        {
            status = MCSPI_STATUS_BUSY;
        }
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        /* Reset counter and other params */
        chNum = transaction->channel;
        chObj = &hMcspiInit->chObj[chNum];
        chObj->curTxBufPtr = (uint8_t *) transaction->txBuf;
        chObj->curRxBufPtr = NULL;
        chObj->curTxWords  = 0U;
        chObj->curRxWords  = 0U;

        /* Initialize channel dataSize */
        MCSPI_setChDataSize(baseAddr, chObj, transaction->dataSize,
                            transaction->csDisable);

        if(MCSPI_MS_MODE_CONTROLLER == hMcspiInit->msMode)
        {
            /* Enable FIFO*/
            MCSPI_setFifoConfig(hMcspi,chObj, baseAddr, transaction->count);
            MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);
            MCSPI_transferControllerIntr(hMcspi, chObj);
        }
        else
        {
            /* Enable FIFO*/
            MCSPI_setPeripheralFifoConfig(chObj, baseAddr, count);
            MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);
            MCSPI_transferPeripheralIntr(hMcspi, chObj);
        }
    }

    return (status);
}

int32_t MCSPI_lld_writeDma(MCSPILLD_Handle hMcspi, void *txBuf, uint32_t count, uint32_t timeout,
                           const MCSPI_ExtendedParams *extendedParams)
{
    int32_t status =       MCSPI_STATUS_SUCCESS;
    MCSPI_Transaction     *transaction;
    uint32_t               baseAddr, chNum;
    MCSPI_ChObject        *chObj;
    MCSPILLD_InitHandle    hMcspiInit;

    /* Check parameters */
    if((NULL == hMcspi) || (NULL == hMcspi->hMcspiInit))
    {
        status = MCSPI_INVALID_PARAM;
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        transaction = &hMcspi->transaction;
        /* Check if any transaction is in progress */
        if(hMcspi->state == MCSPI_STATE_READY)
        {
            hMcspi->state = MCSPI_STATE_BUSY;
            /* Initialize transaction with default parameters */
            MCSPI_lld_Transaction_init(transaction);

            if(extendedParams != NULL)
            {
                transaction->channel   = extendedParams->channel;
                transaction->dataSize  = extendedParams->dataSize;
                transaction->csDisable = extendedParams->csDisable;
                transaction->args      = extendedParams->args;
            }

            baseAddr             = hMcspi->baseAddr;
            hMcspiInit           = hMcspi->hMcspiInit;
            transaction->count   = count;
            transaction->timeout = timeout;
            transaction->txBuf   = txBuf;

            status =  MCSPI_lld_isParameterValid(count);
            status += MCSPI_lld_isParameterValid(timeout);
            status += MCSPI_lld_isChannelValid(transaction->channel);
            status += MCSPI_lld_isDataSizeValid(transaction->dataSize);
            if(status != MCSPI_STATUS_SUCCESS)
            {
                status = MCSPI_INVALID_PARAM;
            }

            /* Check if the channel is configured */
            if(TRUE != hMcspiInit->chObj[transaction->channel].isOpen)
            {
                /* Channel not configured */
                status = MCSPI_TRANSFER_FAILED;
            }
        }
        else
        {
            status = MCSPI_STATUS_BUSY;
        }
    }

    if (MCSPI_STATUS_SUCCESS == status)
    {
        /* Reset counter and other params */
        chNum = transaction->channel;
        chObj = &hMcspiInit->chObj[chNum];
        chObj->curTxBufPtr = (uint8_t *) transaction->txBuf;
        chObj->curRxBufPtr = NULL;
        chObj->curTxWords  = 0U;
        chObj->curRxWords  = 0U;

        /* Initialize channel dataSize */
        MCSPI_setChDataSize(baseAddr, chObj, transaction->dataSize,
                            transaction->csDisable);

        status = MCSPI_lld_dmaTransfer(hMcspi, chObj, transaction);
    }

    return (status);
}

int32_t MCSPI_lld_read(MCSPILLD_Handle hMcspi, void *rxBuf, uint32_t count,
                       uint32_t timeout, const MCSPI_ExtendedParams *extendedParams)
{
    int32_t                status = MCSPI_STATUS_SUCCESS;
    MCSPI_Transaction     *transaction;
    uint32_t               baseAddr, chNum;
    MCSPILLD_InitHandle    hMcspiInit;
    MCSPI_ChObject        *chObj;

    /* Check parameters */
    if((NULL == hMcspi) || (NULL == hMcspi->hMcspiInit))
    {
        status = MCSPI_INVALID_PARAM;
    }
    if(MCSPI_STATUS_SUCCESS == status)
    {
        transaction = &hMcspi->transaction;
        /* Check if any transaction is in progress */
        if(hMcspi->state == MCSPI_STATE_READY)
        {
            hMcspi->state = MCSPI_STATE_BUSY;
            /* Initialize transaction with default parameters */
            MCSPI_lld_Transaction_init(transaction);

            if(extendedParams != NULL)
            {
                transaction->channel   = extendedParams->channel;
                transaction->dataSize  = extendedParams->dataSize;
                transaction->csDisable = extendedParams->csDisable;
                transaction->args      = extendedParams->args;
            }

            baseAddr             = hMcspi->baseAddr;
            hMcspiInit           = hMcspi->hMcspiInit;
            transaction->count   = count;
            transaction->timeout = timeout;
            transaction->rxBuf   = rxBuf;

            status  = MCSPI_lld_isParameterValid(count);
            status += MCSPI_lld_isParameterValid(timeout);
            status += MCSPI_lld_isChannelValid(transaction->channel);
            status += MCSPI_lld_isDataSizeValid(transaction->dataSize);
            /* Check if the channel is configured */
            if(TRUE != hMcspiInit->chObj[transaction->channel].isOpen)
            {
                /* Channel not configured */
                status = MCSPI_TRANSFER_FAILED;
            }
        }
        else
        {
            status = MCSPI_STATUS_BUSY;
        }
    }

    if (MCSPI_STATUS_SUCCESS == status)
    {
        /* Reset counter and other params */
        chNum = transaction->channel;
        chObj = &hMcspiInit->chObj[chNum];
        chObj->curTxBufPtr = NULL;
        chObj->curRxBufPtr = (uint8_t *) transaction->rxBuf;
        chObj->curTxWords  = 0U;
        chObj->curRxWords  = 0U;

        /* Initialize channel dataSize */
        MCSPI_setChDataSize(baseAddr, chObj, transaction->dataSize,
                            transaction->csDisable);

        if(MCSPI_MS_MODE_CONTROLLER == hMcspiInit->msMode)
        {
            /* Enable FIFO*/
            MCSPI_setFifoConfig(hMcspi,chObj, baseAddr, transaction->count);
            MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);

            status = MCSPI_transferControllerPoll(hMcspi, chObj, transaction);
        }
        else
        {
            /* Enable FIFO*/
            MCSPI_setPeripheralFifoConfig(chObj, baseAddr, count);
            MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);

            status = MCSPI_transferPeripheralPoll(hMcspi, chObj, transaction);
        }

        if(status == MCSPI_TIMEOUT)
        {
            status = MCSPI_TRANSFER_TIMEOUT;
        }
        else if ((status == MCSPI_TRANSFER_CANCELLED) && (hMcspi->errorFlag != 0U))
        {
            status = MCSPI_TRANSFER_FAILED;
        }
        else
        {
            /* success case */
            status  = MCSPI_TRANSFER_COMPLETED;
        }

        hMcspi->state = MCSPI_STATE_READY;
    }

    return (status);
}

int32_t MCSPI_lld_readIntr(MCSPILLD_Handle hMcspi, void *rxBuf, uint32_t count,
                           uint32_t timeout, const MCSPI_ExtendedParams *extendedParams)
{
    int32_t                status = MCSPI_STATUS_SUCCESS;
    MCSPI_Transaction     *transaction;
    uint32_t               baseAddr, chNum;
    MCSPILLD_InitHandle    hMcspiInit;
    MCSPI_ChObject        *chObj;

    /* Check parameters */
    if((NULL == hMcspi) || (NULL == hMcspi->hMcspiInit))
    {
        status = MCSPI_INVALID_PARAM;
    }
    if(MCSPI_STATUS_SUCCESS == status)
    {
        transaction = &hMcspi->transaction;
        /* Check if any transaction is in progress */
        if(hMcspi->state == MCSPI_STATE_READY)
        {
            hMcspi->state = MCSPI_STATE_BUSY;
            /* Initialize transaction with default parameters */
            MCSPI_lld_Transaction_init(transaction);

            if(extendedParams != NULL)
            {
                transaction->channel   = extendedParams->channel;
                transaction->dataSize  = extendedParams->dataSize;
                transaction->csDisable = extendedParams->csDisable;
                transaction->args      = extendedParams->args;
            }

            baseAddr             = hMcspi->baseAddr;
            hMcspiInit           = hMcspi->hMcspiInit;
            transaction->count   = count;
            transaction->timeout = timeout;
            transaction->rxBuf   = rxBuf;

            status  = MCSPI_lld_isParameterValid(count);
            status += MCSPI_lld_isParameterValid(timeout);
            status += MCSPI_lld_isChannelValid(transaction->channel);
            status += MCSPI_lld_isDataSizeValid(transaction->dataSize);
            if(status != MCSPI_STATUS_SUCCESS)
            {
                status = MCSPI_INVALID_PARAM;
            }

            /* Check if the channel is configured */
            if(TRUE != hMcspiInit->chObj[transaction->channel].isOpen)
            {
                /* Channel not configured */
                status = MCSPI_TRANSFER_FAILED;
            }
        }
        else
        {
            status = MCSPI_STATUS_BUSY;
        }
    }

    if (MCSPI_STATUS_SUCCESS == status)
    {
        /* Reset counter and other params */
        chNum = transaction->channel;
        chObj = &hMcspiInit->chObj[chNum];
        chObj->curTxBufPtr = NULL;
        chObj->curRxBufPtr = (uint8_t *) transaction->rxBuf;
        chObj->curTxWords  = 0U;
        chObj->curRxWords  = 0U;

        /* Initialize channel dataSize */
        MCSPI_setChDataSize(baseAddr, chObj, transaction->dataSize,
                            transaction->csDisable);

        if(MCSPI_MS_MODE_CONTROLLER == hMcspiInit->msMode)
        {
            /* Enable FIFO*/
            MCSPI_setFifoConfig(hMcspi,chObj, baseAddr, transaction->count);
            MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);
            MCSPI_transferControllerIntr(hMcspi, chObj);
        }
        else
        {
            /* Enable FIFO*/
            MCSPI_setPeripheralFifoConfig(chObj, baseAddr, count);
            MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);
            MCSPI_transferPeripheralIntr(hMcspi, chObj);
        }
    }

    return (status);
}

int32_t MCSPI_lld_readDma(MCSPILLD_Handle hMcspi, void *rxBuf, uint32_t count,
                          uint32_t timeout, const MCSPI_ExtendedParams *extendedParams)
{
    int32_t status =       MCSPI_STATUS_SUCCESS;
    MCSPI_Transaction     *transaction;
    uint32_t               baseAddr, chNum;
    MCSPI_ChObject        *chObj;
    MCSPILLD_InitHandle    hMcspiInit;

    /* Check parameters */
    if((NULL == hMcspi) || (NULL == hMcspi->hMcspiInit))
    {
        status = MCSPI_INVALID_PARAM;
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        transaction = &hMcspi->transaction;
        /* Check if any transaction is in progress */
        if(hMcspi->state == MCSPI_STATE_READY)
        {
            hMcspi->state = MCSPI_STATE_BUSY;
            /* Initialize transaction with default parameters */
            MCSPI_lld_Transaction_init(transaction);

            if(extendedParams != NULL)
            {
                transaction->channel   = extendedParams->channel;
                transaction->dataSize  = extendedParams->dataSize;
                transaction->csDisable = extendedParams->csDisable;
                transaction->args      = extendedParams->args;
            }

            baseAddr             = hMcspi->baseAddr;
            hMcspiInit           = hMcspi->hMcspiInit;
            transaction->count   = count;
            transaction->timeout = timeout;
            transaction->rxBuf   = rxBuf;

            status = MCSPI_lld_isParameterValid(count);
            status += MCSPI_lld_isParameterValid(timeout);
            status += MCSPI_lld_isChannelValid(transaction->channel);
            status += MCSPI_lld_isDataSizeValid(transaction->dataSize);
            if(status != MCSPI_STATUS_SUCCESS)
            {
                status = MCSPI_INVALID_PARAM;
            }

            /* Check if the channel is configured */
            if(TRUE != hMcspiInit->chObj[transaction->channel].isOpen)
            {
                /* Channel not configured */
                status = MCSPI_TRANSFER_FAILED;
            }
        }
        else
        {
            status = MCSPI_STATUS_BUSY;
        }
    }

    if (MCSPI_STATUS_SUCCESS == status)
    {
        /* Reset counter and other params */
        chNum = transaction->channel;
        chObj = &hMcspiInit->chObj[chNum];
        chObj->curTxBufPtr = NULL;
        chObj->curRxBufPtr = (uint8_t *) transaction->rxBuf;
        chObj->curTxWords  = 0U;
        chObj->curRxWords  = 0U;

        /* Initialize channel dataSize */
        MCSPI_setChDataSize(baseAddr, chObj, transaction->dataSize,
                            transaction->csDisable);

        status = MCSPI_lld_dmaTransfer(hMcspi, chObj, transaction);
    }

    return (status);
}

int32_t MCSPI_lld_readWrite(MCSPILLD_Handle hMcspi, void *txBuf, void *rxBuf, uint32_t count,
                            uint32_t timeout, const MCSPI_ExtendedParams *extendedParams)
{
    int32_t status =       MCSPI_STATUS_SUCCESS;
    MCSPI_Transaction     *transaction;
    uint32_t               baseAddr, chNum;
    MCSPILLD_InitHandle    hMcspiInit;
    MCSPI_ChObject        *chObj;

    /* Check parameters */
    if((NULL == hMcspi) || (NULL == hMcspi->hMcspiInit))
    {
        status = MCSPI_INVALID_PARAM;
    }
    if(MCSPI_STATUS_SUCCESS == status)
    {
        transaction = &hMcspi->transaction;
        /* Check if any transaction is in progress */
        if(hMcspi->state == MCSPI_STATE_READY)
        {
            hMcspi->state = MCSPI_STATE_BUSY;
            /* Initialize transaction with default parameters */
            MCSPI_lld_Transaction_init(transaction);

            if(extendedParams != NULL)
            {
                transaction->channel   = extendedParams->channel;
                transaction->dataSize  = extendedParams->dataSize;
                transaction->csDisable = extendedParams->csDisable;
                transaction->args      = extendedParams->args;
            }

            baseAddr             = hMcspi->baseAddr;
            hMcspiInit           = hMcspi->hMcspiInit;
            transaction->timeout = timeout;
            transaction->count   = count;
            transaction->txBuf   = txBuf;
            transaction->rxBuf   = rxBuf;

            status = MCSPI_lld_isParameterValid(timeout);
            status += MCSPI_lld_isChannelValid(transaction->channel);
            status += MCSPI_lld_isDataSizeValid(transaction->dataSize);
            if(status != MCSPI_STATUS_SUCCESS)
            {
                status = MCSPI_INVALID_PARAM;
            }

            /* Check if the channel is configured */
            if(TRUE != hMcspiInit->chObj[transaction->channel].isOpen)
            {
                /* Channel not configured */
                status = MCSPI_TRANSFER_FAILED;
            }
        }
        else
        {
            status = MCSPI_STATUS_BUSY;
        }
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        /* Reset counter and other params */
        chNum = transaction->channel;
        chObj = &hMcspiInit->chObj[chNum];
        chObj->curTxBufPtr = (uint8_t *) transaction->txBuf;
        chObj->curRxBufPtr = (uint8_t *) transaction->rxBuf;
        chObj->curTxWords  = 0U;
        chObj->curRxWords  = 0U;

        /* Initialize channel dataSize */
        MCSPI_setChDataSize(baseAddr, chObj, transaction->dataSize,
                            transaction->csDisable);

        if(MCSPI_MS_MODE_CONTROLLER == hMcspiInit->msMode)
        {
            /* Enable FIFO*/
            MCSPI_setFifoConfig(hMcspi,chObj, baseAddr, transaction->count);
            MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);

            status = MCSPI_transferControllerPoll(hMcspi, chObj, transaction);
        }
        else
        {
            /* Enable FIFO*/
            MCSPI_setPeripheralFifoConfig(chObj, baseAddr, count);
            MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);

            status = MCSPI_transferPeripheralPoll(hMcspi, chObj, transaction);
        }

        if(status == MCSPI_TIMEOUT)
        {
            status = MCSPI_TRANSFER_TIMEOUT;
        }
        else if ((status == MCSPI_TRANSFER_CANCELLED) && (hMcspi->errorFlag != 0U))
        {
            status = MCSPI_TRANSFER_FAILED;
        }
        else
        {
            /* success case */
            status  = MCSPI_TRANSFER_COMPLETED;
        }

        hMcspi->state = MCSPI_STATE_READY;
    }

    return (status);
}

int32_t MCSPI_lld_readWriteIntr(MCSPILLD_Handle hMcspi, void *txBuf, void *rxBuf, uint32_t count,
                                uint32_t timeout, const MCSPI_ExtendedParams *extendedParams)
{
    int32_t                status = MCSPI_STATUS_SUCCESS;
    MCSPI_Transaction     *transaction;
    uint32_t               baseAddr, chNum;
    MCSPILLD_InitHandle    hMcspiInit;
    MCSPI_ChObject        *chObj;

    /* Check parameters */
    if((NULL == hMcspi) || (NULL == hMcspi->hMcspiInit))
    {
        status = MCSPI_INVALID_PARAM;
    }
    if(MCSPI_STATUS_SUCCESS == status)
    {
        transaction = &hMcspi->transaction;
        /* Check if any transaction is in progress */
        if(hMcspi->state == MCSPI_STATE_READY)
        {
            hMcspi->state = MCSPI_STATE_BUSY;
            /* Initialize transaction with default parameters */
            MCSPI_lld_Transaction_init(transaction);
            if(extendedParams != NULL)
            {
                transaction->channel   = extendedParams->channel;
                transaction->dataSize  = extendedParams->dataSize;
                transaction->csDisable = extendedParams->csDisable;
                transaction->args      = extendedParams->args;
            }

            baseAddr             = hMcspi->baseAddr;
            hMcspiInit           = hMcspi->hMcspiInit;
            transaction->timeout = timeout;
            transaction->count   = count;
            transaction->txBuf   = txBuf;
            transaction->rxBuf   = rxBuf;

            status = MCSPI_lld_isParameterValid(count);
            status += MCSPI_lld_isParameterValid(timeout);
            status += MCSPI_lld_isChannelValid(transaction->channel);
            status += MCSPI_lld_isDataSizeValid(transaction->dataSize);
            if(status != MCSPI_STATUS_SUCCESS)
            {
                status = MCSPI_INVALID_PARAM;
            }

            /* Check if the channel is configured */
            if(TRUE != hMcspiInit->chObj[transaction->channel].isOpen)
            {
                /* Channel not configured */
                status = MCSPI_TRANSFER_FAILED;
            }
        }
        else
        {
            status = MCSPI_STATUS_BUSY;
        }
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        /* Reset counter and other params */
        chNum = transaction->channel;
        chObj = &hMcspiInit->chObj[chNum];
        chObj->curTxBufPtr = (uint8_t *) transaction->txBuf;
        chObj->curRxBufPtr = (uint8_t *) transaction->rxBuf;
        chObj->curTxWords  = 0U;
        chObj->curRxWords  = 0U;

        /* Initialize channel dataSize */
        MCSPI_setChDataSize(baseAddr, chObj, transaction->dataSize,
                            transaction->csDisable);

        if(MCSPI_MS_MODE_CONTROLLER == hMcspiInit->msMode)
        {
            /* Enable FIFO */
            MCSPI_setFifoConfig(hMcspi,chObj, baseAddr, transaction->count);
            MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);
            MCSPI_transferControllerIntr(hMcspi, chObj);
        }
        else
        {
            /* Enable FIFO */
            MCSPI_setPeripheralFifoConfig(chObj, baseAddr, count);
            MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);
            MCSPI_transferPeripheralIntr(hMcspi, chObj);
        }
    }

    return (status);
}

int32_t MCSPI_lld_readWriteDma(MCSPILLD_Handle hMcspi, void *txBuf, void *rxBuf, uint32_t count,
                               uint32_t timeout, const MCSPI_ExtendedParams *extendedParams)
{
    int32_t                status = MCSPI_STATUS_SUCCESS;
    MCSPI_Transaction     *transaction;
    uint32_t               baseAddr, chNum;
    MCSPILLD_InitHandle    hMcspiInit;
    MCSPI_ChObject        *chObj;

    /* Check parameters */
    if((NULL == hMcspi) || (NULL == hMcspi->hMcspiInit))
    {
        status = MCSPI_INVALID_PARAM;
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        transaction = &hMcspi->transaction;

        /* Check if any transaction is in progress */
        if(hMcspi->state == MCSPI_STATE_READY)
        {
            hMcspi->state = MCSPI_STATE_BUSY;
            /* Initialize transaction with default parameters */
            MCSPI_lld_Transaction_init(transaction);

            if(extendedParams != NULL)
            {
                transaction->channel   = extendedParams->channel;
                transaction->dataSize  = extendedParams->dataSize;
                transaction->csDisable = extendedParams->csDisable;
                transaction->args      = extendedParams->args;
            }

            baseAddr             = hMcspi->baseAddr;
            hMcspiInit           = hMcspi->hMcspiInit;
            transaction->timeout = timeout;
            transaction->count   = count;
            transaction->txBuf   = txBuf;
            transaction->rxBuf   = rxBuf;

            status = MCSPI_lld_isParameterValid(count);
            status += MCSPI_lld_isParameterValid(timeout);
            status += MCSPI_lld_isChannelValid(transaction->channel);
            status += MCSPI_lld_isDataSizeValid(transaction->dataSize);
            if(status != MCSPI_STATUS_SUCCESS)
            {
                status = MCSPI_INVALID_PARAM;
            }

            /* Check if the channel is configured */
            if(TRUE != hMcspiInit->chObj[transaction->channel].isOpen)
            {
                /* Channel not configured */
                status += MCSPI_TRANSFER_FAILED;
            }
        }
        else
        {
            status = MCSPI_STATUS_BUSY;
        }
    }

    if (MCSPI_STATUS_SUCCESS == status)
    {
        /* Reset counter and other params */
        chNum = transaction->channel;
        chObj = &hMcspiInit->chObj[chNum];
        chObj->curTxBufPtr = (uint8_t *) transaction->txBuf;
        chObj->curRxBufPtr = (uint8_t *) transaction->rxBuf;
        chObj->curTxWords  = 0U;
        chObj->curRxWords  = 0U;

        /* Initialize channel dataSize */
        MCSPI_setChDataSize(baseAddr, chObj, transaction->dataSize,
                            transaction->csDisable);

        status = MCSPI_lld_dmaTransfer(hMcspi, chObj, transaction);
    }

    return (status);
}

int32_t MCSPI_lld_readWriteCancel(MCSPILLD_Handle hMcspi)
{
    int32_t             status = MCSPI_STATUS_SUCCESS;
    uint32_t            chNum;
    MCSPI_ChObject     *chObj;

    /* Check parameters */
    if (NULL == hMcspi)
    {
        status = MCSPI_INVALID_PARAM;
    }
    else
    {
        /* Check if any transaction is in progress */
        if(MCSPI_STATE_BUSY == hMcspi->state)
        {
            chNum = hMcspi->transaction.channel;
            chObj = &hMcspi->hMcspiInit->chObj[chNum];

            /* Stop MCSPI Channel */
            MCSPI_stop(hMcspi, chObj, chNum);

            status  = MCSPI_TRANSFER_CANCELLED;
            /* Return the actual number of words transferred */
            hMcspi->transaction.count = chObj->curRxWords;
            if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
            {
                hMcspi->transaction.count = chObj->curTxWords;
            }

            hMcspi->state = MCSPI_STATE_READY;
            if(hMcspi->errorFlag != 0U)
            {
                hMcspi->hMcspiInit->errorCallbackFxn(hMcspi, status);
            }
            else
            {
                hMcspi->hMcspiInit->transferCallbackFxn(hMcspi, status);
            }
        }
        else
        {
            /* No transaction to cancel. */
            status = MCSPI_STATUS_FAILURE;
        }
    }

    return (status);
}

int32_t MCSPI_lld_readWriteDmaCancel(MCSPILLD_Handle hMcspi)
{
    int32_t             status = MCSPI_STATUS_SUCCESS;
    uint32_t            chNum;
    MCSPI_ChObject     *chObj;

    /* Check parameters */
    if (NULL == hMcspi)
    {
        status = MCSPI_INVALID_PARAM;
    }
    else
    {
        /* Check if any transaction is in progress */
        if(MCSPI_STATE_BUSY == hMcspi->state)
        {
            chNum = hMcspi->transaction.channel;
            chObj = &hMcspi->hMcspiInit->chObj[chNum];

            /* Stop MCSPI Channel */
            MCSPI_lld_dmaStop(hMcspi, chObj, chNum);

            status  = MCSPI_TRANSFER_CANCELLED;
            /* Return the actual number of words transferred */
            hMcspi->transaction.count = chObj->curRxWords;
            if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
            {
                hMcspi->transaction.count = chObj->curTxWords;
            }

            hMcspi->state = MCSPI_STATE_READY;
            if(hMcspi->errorFlag != 0U)
            {
                hMcspi->hMcspiInit->errorCallbackFxn(hMcspi, status);

            }
            else
            {
                hMcspi->hMcspiInit->transferCallbackFxn(hMcspi, status);
            }
        }
        else
        {
            /* No transaction to cancel. */
            status = MCSPI_STATUS_FAILURE;
        }
    }
    return (status);
}

void MCSPI_lld_controllerIsr(void* args)
{
    uint32_t               transferStatus;
    MCSPI_ChObject        *chObj;
    MCSPI_Transaction     *transaction;
    uint32_t               chNum;
    MCSPILLD_Handle        hMcspi = NULL;

    /* Check parameters */
    if(NULL != args)
    {
        hMcspi = (MCSPILLD_Handle)args;
        transaction = &hMcspi->transaction;
        chNum = transaction->channel;
        chObj = &hMcspi->hMcspiInit->chObj[chNum];
        transferStatus = MCSPI_continueTxRx(hMcspi, chObj, transaction);
        if (MCSPI_TRANSFER_COMPLETED == (int32_t)transferStatus)
        {
            /* Process the transfer completion. */
            /* Stop MCSPI Channel */
            MCSPI_stop(hMcspi, chObj, chNum);
            /* Return the actual number of words transferred */
            hMcspi->transaction.count = chObj->curRxWords;
            if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
            {
                hMcspi->transaction.count = chObj->curTxWords;
            }
            hMcspi->state = MCSPI_STATE_READY;
            hMcspi->hMcspiInit->transferCallbackFxn(hMcspi, transferStatus);
        }

        if((hMcspi->errorFlag != 0U) && (transferStatus == MCSPI_TRANSFER_CANCELLED))
        {
            hMcspi->state = MCSPI_STATE_READY;
            hMcspi->hMcspiInit->errorCallbackFxn(hMcspi, transferStatus);
        }
        /*
        * Else the transfer is still pending.
        * Do nothing, wait for next interrupt.
        */
    }

    return;
}

void MCSPI_lld_peripheralIsr(void* args)
{
    uint32_t            transferStatus;
    MCSPI_ChObject     *chObj;
    MCSPI_Transaction  *transaction;
    uint32_t            baseAddr, chNum;
    MCSPILLD_Handle     hMcspi = NULL;

    /* Check parameters */
    if(NULL != args)
    {
        hMcspi = (MCSPILLD_Handle)args;
        transaction = &hMcspi->transaction;
        baseAddr = hMcspi->baseAddr;

        chNum = transaction->channel;
        chObj = &hMcspi->hMcspiInit->chObj[chNum];
        transferStatus = MCSPI_continuePeripheralTxRx(hMcspi, chObj, transaction);

        if (MCSPI_TRANSFER_COMPLETED == (int32_t)transferStatus)
        {
            /* Process the transfer completion. */
            /* Stop MCSPI Channel */
            MCSPI_stop(hMcspi, chObj, chNum);

            /* Disable TX and RX FIFO */
            chObj->chConfRegVal &= ~(CSL_MCSPI_CH0CONF_FFEW_MASK | CSL_MCSPI_CH0CONF_FFER_MASK);
            CSL_REG32_WR(baseAddr + MCSPI_CHCONF(chObj->chCfg->chNum), chObj->chConfRegVal);

            /* Return the actual number of words transferred */
            hMcspi->transaction.count = chObj->curRxWords;
            if (MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
            {
                hMcspi->transaction.count = chObj->curTxWords;
            }
            hMcspi->state = MCSPI_STATE_READY;
            if(MCSPI_TRANSFER_COMPLETED == (int32_t)transferStatus)
            {
                hMcspi->hMcspiInit->transferCallbackFxn(hMcspi, transferStatus);
            }
        }

        if((hMcspi->errorFlag != 0U) && (transferStatus == MCSPI_TRANSFER_CANCELLED))
        {
            hMcspi->hMcspiInit->errorCallbackFxn(hMcspi, transferStatus);
        }
        /*
        * Else the transfer is still pending.
        * Do nothing, wait for next interrupt.
        */
    }

    return;
}

int32_t MCSPI_lld_getState(MCSPILLD_Handle hMcspi)
{
    return (int32_t)(hMcspi->state);
}

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static int32_t MCSPI_lld_chConfig(MCSPILLD_Handle hMcspi,
                       const MCSPI_ChConfig *chCfg, uint32_t chCnt)
{
    int32_t         status = MCSPI_STATUS_SUCCESS;
    MCSPI_ChObject  *chObj;

    /* Check parameters */
    if((NULL == chCfg) || (chCfg->chNum >= MCSPI_MAX_NUM_CHANNELS))
    {
        status = MCSPI_INVALID_PARAM;
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        /* Check channel parameters */
        status = MCSPI_checkChConfig(hMcspi, chCfg);
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        chObj = &hMcspi->hMcspiInit->chObj[chCnt];

        /* Configure the MCSPI channel */
        MCSPI_setChConfig(hMcspi, chObj);

        chObj->isOpen = TRUE;
        chObj->csEnable = TRUE;
    }

    return (status);
}

static void MCSPI_initiateLastChunkTransfer(MCSPILLD_Handle hMcspi,
                                            MCSPI_ChObject *chObj,
                                            const MCSPI_Transaction *transaction)
{
    uint32_t        baseAddr, chNum;
    uint32_t        reminder;
    uint32_t        regVal;

    baseAddr = hMcspi->baseAddr;
    chNum = chObj->chCfg->chNum;

    /* Disable channel so that new settings takes effect */
    chObj->chCtrlRegVal &= (~CSL_MCSPI_CH0CTRL_EN_MASK);
    CSL_REG32_WR(baseAddr + MCSPI_CHCTRL(chNum), chObj->chCtrlRegVal);

    /* Start transferring only multiple of FIFO trigger level */
    if(MCSPI_TR_MODE_RX_ONLY != chObj->chCfg->trMode)
    {
        reminder = (transaction->count & (chObj->effTxFifoDepth - 1U));
    }
    else
    {
        reminder = (transaction->count & (chObj->effRxFifoDepth - 1U));
    }

    /* Set FIFO trigger level and word count */
    regVal  = 0;
    if (chObj->chCfg->rxFifoTrigLvl != 1U)
    {
        regVal |= ((((reminder << chObj->bufWidthShift) - 1U) << CSL_MCSPI_XFERLEVEL_AFL_SHIFT) &
                CSL_MCSPI_XFERLEVEL_AFL_MASK);
    }
    if (chObj->chCfg->txFifoTrigLvl != 1U)
    {
        regVal |= ((((reminder << chObj->bufWidthShift) - 1U) << CSL_MCSPI_XFERLEVEL_AEL_SHIFT) &
                CSL_MCSPI_XFERLEVEL_AEL_MASK);
    }
    regVal |= ((reminder << CSL_MCSPI_XFERLEVEL_WCNT_SHIFT) &
               CSL_MCSPI_XFERLEVEL_WCNT_MASK);

    CSL_REG32_WR(baseAddr + CSL_MCSPI_XFERLEVEL, regVal);

    /* Enable channel */
    chObj->chCtrlRegVal |= CSL_MCSPI_CH0CTRL_EN_MASK;
    CSL_REG32_WR(baseAddr + MCSPI_CHCTRL(chNum), chObj->chCtrlRegVal);
}

static uint32_t MCSPI_continueTxRx(MCSPILLD_Handle hMcspi,
                                   MCSPI_ChObject *chObj,
                                   const MCSPI_Transaction *transaction)
{
    uint32_t                 baseAddr, chNum, txEmptyMask, rxFullMask;
    uint32_t                 retVal = MCSPI_TRANSFER_STARTED;
    volatile uint32_t        irqStatus, chStat;

    baseAddr = hMcspi->baseAddr;
    chNum    = chObj->chCfg->chNum;
    txEmptyMask = Spi_mcspiGetTxMask(chNum);
    rxFullMask  = Spi_mcspiGetRxMask(chNum);

    irqStatus = CSL_REG32_RD(baseAddr + CSL_MCSPI_IRQSTATUS);

    if ((irqStatus & chObj->intrMask) != 0U)
    {
        /* Clear the interrupts being serviced. */
        CSL_REG32_WR(baseAddr + CSL_MCSPI_IRQSTATUS, (irqStatus & chObj->intrMask));

        /* First read the data from the Rx FIFO. */
        if ((irqStatus & rxFullMask) == rxFullMask)
        {
            /* Perform RX only when enabled */
            if(MCSPI_TR_MODE_TX_ONLY != chObj->chCfg->trMode)
            {
                uint32_t numWordsToRead = transaction->count - chObj->curRxWords;
                if (numWordsToRead > chObj->effRxFifoDepth)
                {
                    numWordsToRead = chObj->effRxFifoDepth;
                }
                if(hMcspi->hMcspiInit->multiWordAccess == TRUE)
                {
                    /* In case of Multi word Access, always read 32 bits of data from RX FIFO. */
                    MCSPI_fifoReadMultiWord(baseAddr, chObj, numWordsToRead);
                }
                else
                {
                    /* Read data from RX FIFO. */
                    MCSPI_fifoRead(baseAddr, chObj, numWordsToRead);
                }
            }
        }
        if ((irqStatus & txEmptyMask) == txEmptyMask)
        {
            uint32_t numWordsToWrite = transaction->count - chObj->curTxWords;
            if (numWordsToWrite > chObj->effTxFifoDepth)
            {
                numWordsToWrite = chObj->effTxFifoDepth;
            }
            if(hMcspi->hMcspiInit->multiWordAccess == TRUE)
            {
                /* Write the data in TX FIFO.Even in RX only mode, dummy data has to
                be written to receive data from Peripheral */
                MCSPI_fifoWriteMultiWord(baseAddr, chObj, numWordsToWrite);
            }
            else
            {
                MCSPI_fifoWrite(baseAddr, chObj, numWordsToWrite);
            }
        }
        if ((irqStatus & CSL_MCSPI_IRQSTATUS_EOW_MASK) == CSL_MCSPI_IRQSTATUS_EOW_MASK)
        {
            if (MCSPI_TR_MODE_RX_ONLY != chObj->chCfg->trMode)
            {
                if (transaction->count == chObj->curTxWords)
                {
                    do
                    {
                        /* Wait for end of transfer. */
                        chStat = CSL_REG32_RD(baseAddr + MCSPI_CHSTAT(chNum));
                    }while ((chStat & CSL_MCSPI_CH0STAT_EOT_MASK) == 0U);

                    /* read the last data if any from Rx FIFO. */
                    if ((MCSPI_TR_MODE_TX_ONLY != chObj->chCfg->trMode) &&
                        (transaction->count != chObj->curRxWords))
                    {
                        /* This is a corner case. EOW is set at the end of transmission.
                            * the reception is not complete by the time we are processing EOW.
                            * Read the remaining bytes.
                            */
                        MCSPI_fifoRead(baseAddr, chObj, (transaction->count - chObj->curRxWords));
                    }
                    /* Clear all interrupts. */
                    MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);

                    retVal = MCSPI_TRANSFER_COMPLETED;
                }
                else
                {
                    MCSPI_initiateLastChunkTransfer(hMcspi, chObj, transaction);
                }
            }
            else
            {
                if (transaction->count == chObj->curRxWords)
                {
                    do{
                        /* Wait for end of transfer. */
                        chStat = CSL_REG32_RD(baseAddr + MCSPI_CHSTAT(chNum));
                    }while ((chStat & CSL_MCSPI_CH0STAT_EOT_MASK) == 0u);
                    /* Clear all interrupts. */
                    MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);
                    retVal = MCSPI_TRANSFER_COMPLETED;
                }
                else
                {
                    MCSPI_initiateLastChunkTransfer(hMcspi, chObj, transaction);
                }
            }
        }

        /* Check for Rx overflow or Tx underflow.
         * Cancel the current transfer and return error. */
        if ((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_MASK)) != 0U)
        {
            retVal = MCSPI_TRANSFER_CANCELLED;
            hMcspi->errorFlag |= MCSPI_ERROR_RX_OVERFLOW;
        }

        if (((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_MASK << (4U * chNum))) != 0U) &&
            (hMcspi->hMcspiInit->msMode == MCSPI_MS_MODE_PERIPHERAL))
        {
            retVal  = MCSPI_TRANSFER_CANCELLED;
            hMcspi->errorFlag |= MCSPI_ERROR_TX_UNDERFLOW;
        }
    }

    return retVal;
}

static int32_t MCSPI_transferControllerPoll(MCSPILLD_Handle hMcspi,
                                        MCSPI_ChObject *chObj,
                                        const MCSPI_Transaction *transaction)
{
    int32_t                  status = MCSPI_STATUS_SUCCESS;
    uint32_t                 baseAddr, chNum;
    uint32_t                 numWordsToWrite, txEmptyMask, irqStatus = 0U;
    uint32_t                 timeout, startTicks, elapsedTicks;
    uint32_t                 timeoutElapsed  = FALSE;
    MCSPILLD_InitHandle      hMcspiInit = hMcspi->hMcspiInit;

    baseAddr = hMcspi->baseAddr;
    chNum    = chObj->chCfg->chNum;
    timeout  = transaction->timeout;

    /* Manual CS assert */
    if(MCSPI_CH_MODE_SINGLE == hMcspi->hMcspiInit->chMode)
    {
        if (chObj->csEnable ==  TRUE)
        {
            chObj->chConfRegVal |= CSL_MCSPI_CH0CONF_FORCE_MASK;
            CSL_REG32_WR(baseAddr + MCSPI_CHCONF(chObj->chCfg->chNum), chObj->chConfRegVal);
            chObj->csEnable = FALSE;
        }
    }

    /* Enable channel */
    chObj->chCtrlRegVal |= CSL_MCSPI_CH0CTRL_EN_MASK;
    CSL_REG32_WR(baseAddr + MCSPI_CHCTRL(chNum), chObj->chCtrlRegVal);

    startTicks = hMcspiInit->clockP_get();

    /* wait for the Tx Empty bit to be set. */
    while (((MCSPI_readChStatusReg(baseAddr, chNum) & CSL_MCSPI_CH0STAT_TXS_MASK) == 0U)
            && (timeoutElapsed != TRUE))
    {
        elapsedTicks = hMcspiInit->clockP_get() - startTicks;
        if (elapsedTicks >= timeout)
        {
            /* timeout occured */
            timeoutElapsed = TRUE;
        }
        /* wait for the Tx Empty Event. */
    }
    /* Initially write the full Tx FIFO */
    if (transaction->count > chObj->effTxFifoDepth)
    {
        numWordsToWrite = chObj->effTxFifoDepth;
    }
    else
    {
        numWordsToWrite = transaction->count;
    }
    /* Write the data in TX FIFO.Even in RX only mode, dummy data has to
        be written to receive data from Peripheral */
    MCSPI_fifoWrite(baseAddr, chObj, numWordsToWrite);

    if(MCSPI_TR_MODE_TX_ONLY != chObj->chCfg->trMode)
    {
        while ((((transaction->count - chObj->curTxWords) != 0U) ||
                ((transaction->count - chObj->curRxWords) != 0U)) &&
               (timeoutElapsed != TRUE))
        {
            irqStatus = CSL_REG32_RD(baseAddr + CSL_MCSPI_IRQSTATUS);
            /* Now keep polling the CH_STAT register, if RXs bit is set, at least 1 word is available.
            Read the data from Rx register, also write the same number of bytes in Tx register.
            In case of Controller mode only when 1 word is sent out, 1 word will be received. */
            if ((MCSPI_readChStatusReg(baseAddr, chNum) & CSL_MCSPI_CH0STAT_RXS_MASK) != 0U)
            {
                MCSPI_fifoRead(baseAddr, chObj, 1);
                if (transaction->count > chObj->curTxWords)
                {
                    MCSPI_fifoWrite(baseAddr, chObj, 1);
                }
            }
            elapsedTicks = hMcspiInit->clockP_get() - startTicks;
            if (elapsedTicks >= timeout)
            {
                /* timeout occured */
                timeoutElapsed = TRUE;
            }
        }
    }
    else
    {
        txEmptyMask = Spi_mcspiGetTxMask(chNum);
        while (((transaction->count - chObj->curTxWords) != 0U) && (timeoutElapsed != (uint32_t)TRUE))
        {
            irqStatus = CSL_REG32_RD(baseAddr + CSL_MCSPI_IRQSTATUS);

            if ((irqStatus & chObj->intrMask) != 0U)
            {
                /* Clear the interrupts being serviced. */
                CSL_REG32_WR(baseAddr + CSL_MCSPI_IRQSTATUS, (irqStatus & chObj->intrMask));
                if ((irqStatus & txEmptyMask) == txEmptyMask)
                {
                    numWordsToWrite = transaction->count - chObj->curTxWords;
                    if (numWordsToWrite > chObj->effTxFifoDepth)
                    {
                        numWordsToWrite = chObj->effTxFifoDepth;
                    }

                    /* Write the data in TX FIFO.Even in RX only mode, dummy data has to
                    be written to receive data from Peripheral */
                    MCSPI_fifoWrite(baseAddr, chObj, numWordsToWrite);
                }
            }
            elapsedTicks = hMcspiInit->clockP_get() - startTicks;
            if (elapsedTicks >= timeout)
            {
                /* timeout occured */
                timeoutElapsed = TRUE;
            }
        }
        /* Wait for the last byte to be sent out. */
        while ((0U == (MCSPI_readChStatusReg(baseAddr, chNum) &
                        CSL_MCSPI_CH0STAT_TXFFE_MASK)) && (timeoutElapsed != TRUE))
        {
            /* Wait fot Tx FIFO to be empty for the last set of data. */
            elapsedTicks = hMcspiInit->clockP_get() - startTicks;
            if (elapsedTicks >= timeout)
            {
                /* timeout occured */
                timeoutElapsed = TRUE;
            }
        }
        while ((0U == (MCSPI_readChStatusReg(baseAddr, chNum) &
                        CSL_MCSPI_CH0STAT_EOT_MASK)) && (timeoutElapsed != TRUE))
        {
            /* Tx FIFO Empty is triggered when last word from FIFO is written to
            internal shift register. SO wait for the end of transfer of last word.
            The EOT gets set after every word when the transfer from shift
            register is complete and is reset when the transmission starts.
            So FIFO empty check is required to make sure the data in FIFO is
            sent out then wait for EOT for the last word. */
            elapsedTicks = hMcspiInit->clockP_get() - startTicks;
            if (elapsedTicks >= timeout)
            {
                /* timeout occured */
                timeoutElapsed = TRUE;
            }
        }
    }

    /* Check for Rx overflow or Tx underflow.
     * Cancel the current transfer and return error.
     */
    if ((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_MASK)) != 0U)
    {
        status = MCSPI_TRANSFER_CANCELLED;
        hMcspi->errorFlag |= MCSPI_ERROR_RX_OVERFLOW;
    }

    if(timeoutElapsed == TRUE)
    {
        status = MCSPI_TIMEOUT;
    }

    /* Stop MCSPI Channel */
    MCSPI_stop(hMcspi, chObj, chNum);

    return (status);
}

static void MCSPI_transferControllerIntr(MCSPILLD_Handle hMcspi,
                                         MCSPI_ChObject *chObj)
{
    uint32_t  baseAddr, chNum;

    baseAddr = hMcspi->baseAddr;
    chNum    = chObj->chCfg->chNum;

    CSL_REG32_WR(baseAddr + CSL_MCSPI_IRQENABLE, chObj->intrMask);

    /* Manual CS assert */
    if(MCSPI_CH_MODE_SINGLE == hMcspi->hMcspiInit->chMode)
    {
        if (chObj->csEnable == TRUE)
        {
            chObj->chConfRegVal |= CSL_MCSPI_CH0CONF_FORCE_MASK;
            CSL_REG32_WR(baseAddr + MCSPI_CHCONF(chObj->chCfg->chNum), chObj->chConfRegVal);
            chObj->csEnable = FALSE;
        }
    }

    /* Enable channel */
    chObj->chCtrlRegVal |= CSL_MCSPI_CH0CTRL_EN_MASK;
    CSL_REG32_WR(baseAddr + MCSPI_CHCTRL(chNum), chObj->chCtrlRegVal);

    /*
     * Note: Once the channel is enabled, we will get the TX almost empty
     *       interrupt. No data transfer is required here!!
     */
    return;
}

static uint32_t MCSPI_continuePeripheralTxRx(MCSPILLD_Handle hMcspi,
                                             MCSPI_ChObject *chObj,
                                             const MCSPI_Transaction *transaction)
{
    uint32_t            baseAddr, chNum, chStat;
    uint32_t            retVal = MCSPI_TRANSFER_STARTED;
    volatile uint32_t   irqStatus;

    baseAddr = hMcspi->baseAddr;
    chNum    = chObj->chCfg->chNum;
    irqStatus = CSL_REG32_RD(baseAddr + CSL_MCSPI_IRQSTATUS);
    if ((irqStatus & chObj->intrMask) != 0U)
    {
        /* Clear the interrupts being serviced. */
        CSL_REG32_WR(baseAddr + CSL_MCSPI_IRQSTATUS, (irqStatus & chObj->intrMask));
        /* First read the data from the Rx FIFO. */
        if ((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_RX0_FULL_MASK << (4U * chNum))) != 0U)
        {
            /* Perform RX only when enabled */
            if(MCSPI_TR_MODE_TX_ONLY != chObj->chCfg->trMode)
            {
                uint32_t numWordsToRead = transaction->count - chObj->curRxWords;
                if (numWordsToRead > chObj->effRxFifoDepth)
                {
                    numWordsToRead = chObj->effRxFifoDepth;
                }
                if(hMcspi->hMcspiInit->multiWordAccess == TRUE)
                {
                    /* In case of Multi word Access, always read 32 bits of data from RX FIFO. */
                    MCSPI_fifoReadMultiWord(baseAddr, chObj, numWordsToRead);
                }
                else
                {
                    /* Read data from RX FIFO. */
                    MCSPI_fifoRead(baseAddr, chObj, numWordsToRead);
                }

                /* Check if transfer is completed for current transaction. */
                if (transaction->count == chObj->curRxWords)
                {
                    retVal = MCSPI_TRANSFER_COMPLETED;
                }
            }
        }
        if ((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_EMPTY_MASK << (4U * chNum))) != 0U)
        {
            /* Perform TX only when enabled */
            if(MCSPI_TR_MODE_RX_ONLY != chObj->chCfg->trMode)
            {
                uint32_t numWordsToWrite = transaction->count - chObj->curTxWords;
                if (numWordsToWrite > chObj->effTxFifoDepth)
                {
                    numWordsToWrite = chObj->effTxFifoDepth;
                }
                if(hMcspi->hMcspiInit->multiWordAccess == TRUE)
                {
                    MCSPI_fifoWriteMultiWord(baseAddr, chObj, numWordsToWrite);
                }
                else
                {
                    /* Write the data in TX FIFO. */
                    MCSPI_fifoWrite(baseAddr, chObj, numWordsToWrite);
                }

                if(MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
                {
                    /* Check if transfer is completed for current transaction. */
                    if (transaction->count == chObj->curTxWords)
                    {
                        do{
                            /* Wait for TX FIFO Empty. */
                            chStat = CSL_REG32_RD(baseAddr + MCSPI_CHSTAT(chNum));
                        }while ((chStat & CSL_MCSPI_CH0STAT_TXFFE_MASK) == 0U);
                        do{
                            /* Wait for end of transfer. */
                            chStat = CSL_REG32_RD(baseAddr + MCSPI_CHSTAT(chNum));
                        }while ((chStat & CSL_MCSPI_CH0STAT_EOT_MASK) == 0U);
                        retVal = MCSPI_TRANSFER_COMPLETED;
                    }
                }
            }
        }

        /* Check for Rx overflow or Tx underflow.
         * Cancel the current transfer and return error. */
        if ((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_MASK)) != 0U)
        {
            retVal = MCSPI_TRANSFER_CANCELLED;
            hMcspi->errorFlag |= MCSPI_ERROR_RX_OVERFLOW;
        }

        if ((irqStatus & ((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_MASK << (4U * chNum))) != 0U)
        {
            retVal = MCSPI_TRANSFER_CANCELLED;
            hMcspi->errorFlag |= MCSPI_ERROR_TX_UNDERFLOW;
        }
    }
    return retVal;
}

static int32_t MCSPI_transferPeripheralPoll(MCSPILLD_Handle hMcspi,
                                        MCSPI_ChObject *chObj,
                                        const MCSPI_Transaction *transaction)
{
    int32_t                 status = MCSPI_STATUS_SUCCESS;
    uint32_t                baseAddr, chNum;
    uint32_t                transferStatus = MCSPI_TRANSFER_STARTED;
    uint32_t                timeout, startTicks, elapsedTicks;
    uint32_t                timeoutElapsed  = FALSE;
    MCSPILLD_InitHandle     hMcspiInit = hMcspi->hMcspiInit;

    baseAddr = hMcspi->baseAddr;
    chNum    = chObj->chCfg->chNum;
    timeout  = transaction->timeout;

    /* Enable channel */
    chObj->chCtrlRegVal |= CSL_MCSPI_CH0CTRL_EN_MASK;
    CSL_REG32_WR(baseAddr + MCSPI_CHCTRL(chNum), chObj->chCtrlRegVal);

    startTicks = hMcspiInit->clockP_get();
    /* Busy loop till channel transfer is completed */
    do
    {
        transferStatus = MCSPI_continuePeripheralTxRx(hMcspi, chObj, transaction);
        elapsedTicks = hMcspiInit->clockP_get() - startTicks;
        if (elapsedTicks >= timeout)
        {
            /* timeout occured */
            timeoutElapsed = TRUE;
        }

        if(transferStatus == MCSPI_TRANSFER_CANCELLED)
        {
            status = MCSPI_TRANSFER_CANCELLED;
            break;
        }

    } while ((transferStatus != (uint32_t)MCSPI_TRANSFER_COMPLETED) && (timeoutElapsed != TRUE));

    /* Stop MCSPI Channel */
    MCSPI_stop(hMcspi, chObj, chNum);

    if(timeoutElapsed == TRUE)
    {
        status = MCSPI_TIMEOUT;
    }

    return (status);
}

static void MCSPI_transferPeripheralIntr(MCSPILLD_Handle hMcspi,
                                        MCSPI_ChObject *chObj)
{
    uint32_t        baseAddr, chNum;

    baseAddr = hMcspi->baseAddr;
    chNum    = chObj->chCfg->chNum;

    CSL_REG32_WR(baseAddr + CSL_MCSPI_IRQENABLE, chObj->intrMask);

    /* Enable channel */
    chObj->chCtrlRegVal |= CSL_MCSPI_CH0CTRL_EN_MASK;
    CSL_REG32_WR(baseAddr + MCSPI_CHCTRL(chNum), chObj->chCtrlRegVal);

    /*
     * Note: Once the channel is enabled, we will get the TX almost empty
     *       interrupt. No data transfer is required here!!
     */

    return;
}

static inline void MCSPI_fifoWrite(uint32_t baseAddr, MCSPI_ChObject *chObj, uint32_t transferLength)
{
    uint32_t        chNum;

    chNum = chObj->chCfg->chNum;
    if(NULL != chObj->curTxBufPtr)
    {
        if(0U == chObj->bufWidthShift)
        {
            chObj->curTxBufPtr = MCSPI_fifoWrite8(
                                     baseAddr,
                                     chNum,
                                     chObj->curTxBufPtr,
                                     transferLength);
        }
        else if(1U == chObj->bufWidthShift)
        {
            chObj->curTxBufPtr = (uint8_t *) MCSPI_fifoWrite16(
                                     baseAddr,
                                     chNum,
                                     (uint16_t *) chObj->curTxBufPtr,
                                     transferLength);
        }
        else
        {
            chObj->curTxBufPtr = (uint8_t *) MCSPI_fifoWrite32(
                                     baseAddr,
                                     chNum,
                                     (uint32_t *) chObj->curTxBufPtr,
                                     transferLength);
        }
    }
    else
    {
        /* NULL TX pointer provided. Use default data */
        MCSPI_fifoWriteDefault(
            baseAddr, chNum, chObj->chCfg->defaultTxData, transferLength);
    }
    chObj->curTxWords += transferLength;

    return;
}

static inline void MCSPI_fifoWriteMultiWord(uint32_t baseAddr,
                                           MCSPI_ChObject *chObj,
                                           uint32_t transferLength)
{
    uint32_t    chNum = chObj->chCfg->chNum;

    /* Check if cuRxBufPtr is NULL */
    if(NULL != chObj->curRxBufPtr)
    {
        /* Read all the bytes that are multiple of 4 */
        if((transferLength % 4U) == 0U)
        {
            chObj->curTxBufPtr = (uint8_t *) MCSPI_fifoWrite32(
                                     baseAddr,
                                     chNum,
                                     (uint32_t *) chObj->curTxBufPtr,
                                     (transferLength / 4U));

            chObj->curTxWords += transferLength;
        }
    }

    return;
}

static inline void MCSPI_fifoRead(uint32_t baseAddr, MCSPI_ChObject *chObj, uint32_t transferLength)
{
    uint32_t        chNum;

    chNum = chObj->chCfg->chNum;
    /* Check if cuRxBufPtr is NULL */
    if(NULL != chObj->curRxBufPtr)
    {
        if(0U == chObj->bufWidthShift)
        {
            chObj->curRxBufPtr = MCSPI_fifoRead8(
                                     baseAddr,
                                     chNum,
                                     chObj->curRxBufPtr,
                                     transferLength,
                                     chObj->dataWidthBitMask);
        }
        else if(1U == chObj->bufWidthShift)
        {
            chObj->curRxBufPtr = (uint8_t *) MCSPI_fifoRead16(
                                     baseAddr,
                                     chNum,
                                     (uint16_t *) chObj->curRxBufPtr,
                                     transferLength,
                                     chObj->dataWidthBitMask);
        }
        else
        {
            chObj->curRxBufPtr = (uint8_t *) MCSPI_fifoRead32(
                                     baseAddr,
                                     chNum,
                                     (uint32_t *) chObj->curRxBufPtr,
                                     transferLength,
                                     chObj->dataWidthBitMask);
        }
    }
    else
    {
        /* NULL RX pointer provided. Read and discard data */
        MCSPI_fifoReadDiscard(baseAddr, chNum, transferLength);
    }
    chObj->curRxWords += transferLength;

    return;
}

static inline void MCSPI_fifoReadMultiWord(uint32_t baseAddr,
                                           MCSPI_ChObject *chObj,
                                           uint32_t transferLength)
{
    uint32_t    chNum = chObj->chCfg->chNum;
    uint32_t    dataWidthBitMask = 0xFFFFFFFFU;

    /* Check if cuRxBufPtr is NULL */
    if(NULL != chObj->curRxBufPtr)
    {
        /* Read all the bytes that are multiple of 4 */
        if((transferLength / 4U) > 0U)
        {
            chObj->curRxBufPtr = (uint8_t *) MCSPI_fifoRead32(
                                            baseAddr,
                                            chNum,
                                            (uint32_t *) chObj->curRxBufPtr,
                                            (transferLength / 4U),
                                            dataWidthBitMask);

            chObj->curRxWords += transferLength;
        }
    }

    return;
}

static void MCSPI_configInstance(MCSPILLD_Handle hMcspi)
{
    uint32_t                regVal;
    uint32_t                baseAddr;
    MCSPILLD_InitHandle     hMcspiInit = hMcspi->hMcspiInit;

    baseAddr = hMcspi->baseAddr;

    /* Reset MCSPI */
    MCSPI_reset(baseAddr);

    /* Set sysconfig */
    regVal = (((uint32_t)CSL_MCSPI_SYSCONFIG_CLOCKACTIVITY_BOTH <<
                         CSL_MCSPI_SYSCONFIG_CLOCKACTIVITY_SHIFT) |
              ((uint32_t)CSL_MCSPI_SYSCONFIG_SIDLEMODE_NO <<
                       CSL_MCSPI_SYSCONFIG_SIDLEMODE_SHIFT) |
              ((uint32_t)CSL_MCSPI_SYSCONFIG_ENAWAKEUP_NOWAKEUP <<
                       CSL_MCSPI_SYSCONFIG_ENAWAKEUP_SHIFT) |
              ((uint32_t)CSL_MCSPI_SYSCONFIG_AUTOIDLE_OFF <<
                       CSL_MCSPI_SYSCONFIG_AUTOIDLE_SHIFT));
    CSL_REG32_WR(baseAddr + CSL_MCSPI_SYSCONFIG, regVal);

    /* Set module control */
    regVal = (hMcspiInit->msMode << CSL_MCSPI_MODULCTRL_MS_SHIFT);
    /* Set multi word acces enable/disable */
    regVal |= (hMcspiInit->multiWordAccess << CSL_MCSPI_MODULCTRL_MOA_SHIFT);
    /* Configure Single/Multi Channel in Controller mode only */
    if(MCSPI_MS_MODE_CONTROLLER == hMcspiInit->msMode)
    {
        regVal |= (hMcspiInit->chMode << CSL_MCSPI_MODULCTRL_SINGLE_SHIFT);
    }
    if(MCSPI_CH_MODE_SINGLE == hMcspiInit->chMode)
    {
        /* 3/4 pin mode applicable only in single channel mode.
        * For  multi-ch mode, CS is always controlled by HW during transfer */
        regVal |= (hMcspiInit->pinMode << CSL_MCSPI_MODULCTRL_PIN34_SHIFT);
        if(MCSPI_MS_MODE_CONTROLLER == hMcspiInit->msMode)
        {
            /* Init delay applicable only for single Controller mode */
            regVal |= (hMcspiInit->initDelay << CSL_MCSPI_MODULCTRL_INITDLY_SHIFT);
        }
    }
    CSL_REG32_WR(baseAddr + CSL_MCSPI_MODULCTRL, regVal);

    return;
}

static void MCSPI_setChConfig(MCSPILLD_Handle hMcspi,
                              MCSPI_ChObject *chObj)
{
    uint32_t                 regVal;
    uint32_t                 baseAddr, chNum;
    const MCSPI_ChConfig    *chCfg;

    baseAddr = hMcspi->baseAddr;
    chNum    = chObj->chCfg->chNum;
    chCfg    = chObj->chCfg;

    regVal = CSL_REG32_RD(baseAddr + MCSPI_CHCONF(chNum));

    /* Clear PHA, POL, DPE0, DPE1 and set PHA, POL fields */
    regVal &= ~((uint32_t) CSL_MCSPI_CH0CONF_PHA_MASK  |
                (uint32_t) CSL_MCSPI_CH3CONF_POL_MASK  |
                (uint32_t) CSL_MCSPI_CH0CONF_DPE0_MASK |
                (uint32_t) CSL_MCSPI_CH0CONF_DPE1_MASK);
    regVal |= (chCfg->frameFormat & ((uint32_t) CSL_MCSPI_CH0CONF_PHA_MASK |
                                    (uint32_t) CSL_MCSPI_CH3CONF_POL_MASK));
    CSL_FINS(regVal, MCSPI_CH0CONF_EPOL, chCfg->csPolarity);
    CSL_FINS(regVal, MCSPI_CH0CONF_TRM, chCfg->trMode);
    CSL_FINS(regVal, MCSPI_CH0CONF_IS, chCfg->inputSelect);
    CSL_FINS(regVal, MCSPI_CH0CONF_DPE0, chCfg->dpe0);
    CSL_FINS(regVal, MCSPI_CH0CONF_DPE1, chCfg->dpe1);
    CSL_FINS(regVal, MCSPI_CH0CONF_SPIENSLV, chCfg->slvCsSelect);
    CSL_FINS(regVal, MCSPI_CH0CONF_SBE, chCfg->startBitEnable);
    CSL_FINS(regVal, MCSPI_CH0CONF_SBPOL, chCfg->startBitPolarity);
    CSL_FINS(regVal, MCSPI_CH0CONF_TURBO, chCfg->turboEnable);
    CSL_FINS(regVal, MCSPI_CH0CONF_TCS0, chCfg->csIdleTime);

    CSL_REG32_WR(baseAddr + MCSPI_CHCONF(chNum), regVal);

    /* Set clock dividers */
    MCSPI_setClkConfig(baseAddr, chNum, hMcspi->hMcspiInit->inputClkFreq, chCfg->bitRate);

    if (hMcspi->hMcspiInit->msMode == MCSPI_MS_MODE_PERIPHERAL)
    {
        if(MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode)
        {
            chObj->intrMask = (((uint32_t)CSL_MCSPI_IRQSTATUS_RX0_FULL_MASK      << (4U * chNum)) |
                               ((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_EMPTY_MASK     << (4U * chNum)) |
                               ((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_MASK << (4U * chNum)) |
                               ((uint32_t)CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_MASK));
        }
        else if(MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
        {
            chObj->intrMask = (((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_EMPTY_MASK     << (4U * chNum)) |
                               ((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_MASK << (4U * chNum)));
        }
        else
        {
            chObj->intrMask = (((uint32_t)CSL_MCSPI_IRQSTATUS_RX0_FULL_MASK     << (4U * chNum)) |
                               ((uint32_t)CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_MASK));
        }
    }
    else
    {
        if(MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode)
        {
            chObj->intrMask =  (((uint32_t)CSL_MCSPI_IRQSTATUS_RX0_FULL_MASK  << (4U * chNum)) |
                                ((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_EMPTY_MASK << (4U * chNum)) |
                                (uint32_t)CSL_MCSPI_IRQSTATUS_EOW_MASK);
        }
        else if(MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
        {
            chObj->intrMask =  (((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_EMPTY_MASK << (4U * chNum)) |
                                ((uint32_t)CSL_MCSPI_IRQSTATUS_EOW_MASK));
        }
        else
        {
            chObj->intrMask = (((uint32_t)CSL_MCSPI_IRQSTATUS_RX0_FULL_MASK << (4U * chNum)) |
                               ((uint32_t)CSL_MCSPI_IRQSTATUS_TX0_EMPTY_MASK << (4U * chNum)) |
                               ((uint32_t)CSL_MCSPI_IRQSTATUS_EOW_MASK));
        }
    }

    /* Store Ch Conf, Ch Ctrl, Syst register values in channel object. */
    chObj->chConfRegVal = CSL_REG32_RD(baseAddr + MCSPI_CHCONF(chNum));
    chObj->chCtrlRegVal = CSL_REG32_RD(baseAddr + MCSPI_CHCTRL(chNum));
    chObj->systRegVal = CSL_REG32_RD(baseAddr + CSL_MCSPI_SYST) & (~(CSL_MCSPI_SYST_SSB_MASK));

    return;
}

static int32_t MCSPI_checkChConfig(MCSPILLD_Handle hMcspi, const MCSPI_ChConfig *chCfg)
{
    int32_t     status = MCSPI_STATUS_SUCCESS;

    if((hMcspi->hMcspiInit->msMode == MCSPI_MS_MODE_PERIPHERAL) && (chCfg->chNum != 0U))
    {
        DebugP_logError("[MCSPI] Only channel 0 supported in peripheral mode !!!\r\n");
        status = MCSPI_INVALID_PARAM;
    }

    return (status);
}

static uint32_t MCSPI_getDataWidthBitMask(uint32_t dataWidth)
{
    uint32_t    i, fifoBitMask = 0x0U;
    uint32_t    tmpVar = 0U;

    for (i = 0U; i < dataWidth; i++)
    {
        tmpVar = ((uint32_t)1U << i);
        fifoBitMask |= tmpVar;
    }

    return (fifoBitMask);
}

void MCSPI_reset(uint32_t baseAddr)
{
    uint32_t    regVal;

    /* Set the SOFTRESET field of MCSPI_SYSCONFIG register. */
    CSL_REG32_FINS(
        baseAddr + CSL_MCSPI_SYSCONFIG,
        MCSPI_SYSCONFIG_SOFTRESET,
        CSL_MCSPI_SYSCONFIG_SOFTRESET_ON);

    /* Stay in the loop until reset is done. */
    while((bool)TRUE)
    {
        regVal = CSL_REG32_RD(baseAddr + CSL_MCSPI_SYSSTATUS);
        if((regVal & CSL_MCSPI_SYSSTATUS_RESETDONE_MASK) ==
            CSL_MCSPI_SYSSTATUS_RESETDONE_MASK)
        {
            break;
        }
        /* Busy wait */
    }
}

static void MCSPI_setClkConfig(uint32_t baseAddr,
                               uint32_t chNum,
                               uint32_t inputClkFreq,
                               uint32_t bitRate)
{
    uint32_t fRatio;
    uint32_t clkD;
    uint32_t extClk;

    /* Calculate the value of fRatio. */
    fRatio = inputClkFreq / bitRate;
    if(((inputClkFreq % bitRate) != 0U) && (fRatio < MCSPI_MAX_CLK_DIVIDER_SUPPORTED))
    {
        /* use a higher divider value in case the ratio
         * is fractional so that we get a lower SPI clock
         * than requested. This ensures we don't go beyond
         * recommended clock speed for the SPI peripheral */
        fRatio++;
    }

    /* If fRatio is not a power of 2, set granularity of 1 clock cycle */
    if((uint32_t) 0U != (fRatio & (fRatio - 1U)))
    {
        /* Set the clock granularity to 1 clock cycle */
        CSL_REG32_FINS(
            baseAddr + MCSPI_CHCONF(chNum),
            MCSPI_CH0CONF_CLKG,
            CSL_MCSPI_CH0CONF_CLKG_ONECYCLE);

        /* Calculate the ratios clkD and extClk based on fClk */
        extClk = (fRatio - 1U) >> 4U;
        clkD   = (fRatio - 1U) & (uint32_t) MCSPI_CLKD_MASK;

        /* Set the extClk field */
        CSL_REG32_FINS(
            baseAddr + MCSPI_CHCTRL(chNum),
            MCSPI_CH0CTRL_EXTCLK,
            extClk);
    }
    else
    {
        /* Clock granularity of power of 2 */
        CSL_REG32_FINS(
            baseAddr + MCSPI_CHCONF(chNum),
            MCSPI_CH0CONF_CLKG,
            CSL_MCSPI_CH0CONF_CLKG_POWERTWO);

        clkD = 0U;
        while (1U != fRatio)
        {
            fRatio >>= 1U;
            clkD++;
        }
    }

    /* Configure the clkD field */
    CSL_REG32_FINS(baseAddr + MCSPI_CHCONF(chNum), MCSPI_CH0CONF_CLKD, clkD);

    return;
}

static void MCSPI_setFifoConfig(MCSPILLD_Handle hMcspi,
                                MCSPI_ChObject *chObj,
                                uint32_t baseAddr,
                                uint32_t numWordsTxRx)
{
    uint32_t txFifoTrigLvl, rxFifoTrigLvl;
    uint32_t regVal;
    uint32_t reminder = 0, effNumWordsTxRx;

    if (MCSPI_OPER_MODE_INTERRUPT == hMcspi->hMcspiInit->operMode)
    {
        /* Start transferring only multiple of FIFO trigger level */
        if(MCSPI_TR_MODE_RX_ONLY != chObj->chCfg->trMode)
        {
            reminder = (numWordsTxRx & (chObj->effTxFifoDepth - 1U));
        }
        else
        {
            reminder = (numWordsTxRx & (chObj->effRxFifoDepth - 1U));
        }

        effNumWordsTxRx = numWordsTxRx - reminder;
    }
    else
    {
        effNumWordsTxRx = numWordsTxRx;
    }

    rxFifoTrigLvl = chObj->chCfg->rxFifoTrigLvl;
    txFifoTrigLvl = chObj->chCfg->txFifoTrigLvl;

    /* Handle transfers with less than FIFO level.
     * Set FIFO trigger level and word count to be equal to the
     * reminder word. Otherwise the HW doesn't generating TX
     * empty interrupt if WCNT is less than FIFO trigger level */
    if(effNumWordsTxRx == 0U)
    {
        effNumWordsTxRx = reminder;
        if(txFifoTrigLvl != 1U)
        {
            /* Set TX level only when TX FIFO is enabled */
            txFifoTrigLvl = reminder << chObj->bufWidthShift;
        }
        if(rxFifoTrigLvl != 1U)
        {
            /* Set RX level only when RX FIFO is enabled */
            rxFifoTrigLvl = reminder << chObj->bufWidthShift;
        }
    }

    /* Set FIFO trigger level and word count should be set to 0.
       Setting all fields in register below, so read modify write not required. */
    regVal = 0;
    regVal |= (((rxFifoTrigLvl - 1U) << CSL_MCSPI_XFERLEVEL_AFL_SHIFT) &
               CSL_MCSPI_XFERLEVEL_AFL_MASK);
    regVal |= (((txFifoTrigLvl - 1U) << CSL_MCSPI_XFERLEVEL_AEL_SHIFT) &
               CSL_MCSPI_XFERLEVEL_AEL_MASK);
    regVal |= ((effNumWordsTxRx << CSL_MCSPI_XFERLEVEL_WCNT_SHIFT) &
               CSL_MCSPI_XFERLEVEL_WCNT_MASK);
    CSL_REG32_WR(baseAddr + CSL_MCSPI_XFERLEVEL, regVal);

    /* Enable TX and RX FIFO */
    chObj->chConfRegVal &= ~(CSL_MCSPI_CH0CONF_FFEW_MASK | CSL_MCSPI_CH0CONF_FFER_MASK);
    if(MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode)
    {
        chObj->chConfRegVal |= ((uint32_t)CSL_MCSPI_CH0CONF_FFEW_FFENABLED << CSL_MCSPI_CH0CONF_FFEW_SHIFT);
        chObj->chConfRegVal |= ((uint32_t)CSL_MCSPI_CH0CONF_FFER_FFENABLED << CSL_MCSPI_CH0CONF_FFER_SHIFT);
    }
    else if(MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
    {
        chObj->chConfRegVal |= ((uint32_t)CSL_MCSPI_CH0CONF_FFEW_FFENABLED << CSL_MCSPI_CH0CONF_FFEW_SHIFT);
    }
    else
    {
        chObj->chConfRegVal |= ((uint32_t)CSL_MCSPI_CH0CONF_FFER_FFENABLED << CSL_MCSPI_CH0CONF_FFER_SHIFT);
    }
    CSL_REG32_WR(baseAddr + MCSPI_CHCONF(chObj->chCfg->chNum), chObj->chConfRegVal);

    return;
}

static uint32_t MCSPI_getFifoTrigLvl(uint32_t numWords, uint32_t fifoDepth)
{
    uint32_t fifoTrigLvl = 1U;
    if (numWords > fifoDepth)
    {
        uint32_t i;
        for (i=fifoDepth; i>0U; i--)
        {
            if ((numWords%i) == 0U )
            {
                fifoTrigLvl = i;
                break;
            }
        }
    }
    else
    {
        fifoTrigLvl = numWords;
    }
    return fifoTrigLvl;
}

static void MCSPI_setPeripheralFifoConfig(MCSPI_ChObject *chObj,
                                     uint32_t baseAddr,
                                     uint32_t numWordsTxRx)
{
    uint32_t txFifoTrigLvl, rxFifoTrigLvl;
    uint32_t regVal;

    /* Find Fifo depth to configure to be multiple of number of words to transfer. */
    chObj->effTxFifoDepth = MCSPI_getFifoTrigLvl(numWordsTxRx, chObj->chCfg->txFifoTrigLvl >> chObj->bufWidthShift);
    chObj->effRxFifoDepth = MCSPI_getFifoTrigLvl(numWordsTxRx, chObj->chCfg->rxFifoTrigLvl >> chObj->bufWidthShift);


    txFifoTrigLvl = chObj->effTxFifoDepth << chObj->bufWidthShift;
    rxFifoTrigLvl = chObj->effRxFifoDepth << chObj->bufWidthShift;

    /* Set FIFO trigger level. word count set to 0 for slve mode.
       Setting all fields in register below, so read modify write not required. */
    regVal  = 0U;
    regVal |= (((rxFifoTrigLvl - 1U) << CSL_MCSPI_XFERLEVEL_AFL_SHIFT) &
               CSL_MCSPI_XFERLEVEL_AFL_MASK);
    regVal |= (((txFifoTrigLvl - 1U) << CSL_MCSPI_XFERLEVEL_AEL_SHIFT) &
               CSL_MCSPI_XFERLEVEL_AEL_MASK);
    regVal &= ~CSL_MCSPI_XFERLEVEL_WCNT_MASK;
    CSL_REG32_WR(baseAddr + CSL_MCSPI_XFERLEVEL, regVal);

    /* Enable TX and RX FIFO */
    chObj->chConfRegVal &= ~(CSL_MCSPI_CH0CONF_FFEW_MASK | CSL_MCSPI_CH0CONF_FFER_MASK);
    if(MCSPI_TR_MODE_TX_RX == chObj->chCfg->trMode)
    {
        chObj->chConfRegVal |= ((uint32_t)CSL_MCSPI_CH0CONF_FFEW_FFENABLED << CSL_MCSPI_CH0CONF_FFEW_SHIFT);
        chObj->chConfRegVal |= ((uint32_t)CSL_MCSPI_CH0CONF_FFER_FFENABLED << CSL_MCSPI_CH0CONF_FFER_SHIFT);
    }
    else if(MCSPI_TR_MODE_TX_ONLY == chObj->chCfg->trMode)
    {
        chObj->chConfRegVal |= ((uint32_t)CSL_MCSPI_CH0CONF_FFEW_FFENABLED << CSL_MCSPI_CH0CONF_FFEW_SHIFT);
    }
    else
    {
        chObj->chConfRegVal |= ((uint32_t)CSL_MCSPI_CH0CONF_FFER_FFENABLED << CSL_MCSPI_CH0CONF_FFER_SHIFT);
    }
    CSL_REG32_WR(baseAddr + MCSPI_CHCONF(chObj->chCfg->chNum), chObj->chConfRegVal);

    return;
}

static inline uint8_t *MCSPI_fifoWrite8(uint32_t baseAddr,
                                        uint32_t chNum,
                                        uint8_t  *bufPtr,
                                        uint32_t transferLength)
{
    uint32_t    i, txData;
    uint8_t    *bufferPtr = bufPtr;

    /* Write the data in TX FIFO for 8-bit transfer */
    for(i = 0U; i < transferLength; i++)
    {
        txData = *bufferPtr;
        bufferPtr++;
        CSL_REG32_WR(baseAddr + MCSPI_CHTX(chNum), txData);
    }

    return (bufferPtr);
}

static inline uint16_t *MCSPI_fifoWrite16(uint32_t baseAddr,
                                          uint32_t chNum,
                                          uint16_t *bufPtr,
                                          uint32_t transferLength)
{
    uint32_t     i, txData;
    uint16_t    *bufferPtr = bufPtr;

    /* Write the data in TX FIFO for 16-bit transfer */
    for(i = 0U; i < transferLength; i++)
    {
        txData = *bufferPtr;
        bufferPtr++;
        CSL_REG32_WR(baseAddr + MCSPI_CHTX(chNum), txData);
    }

    return (bufferPtr);
}

static inline uint32_t *MCSPI_fifoWrite32(uint32_t baseAddr,
                                          uint32_t chNum,
                                          uint32_t *bufPtr,
                                          uint32_t transferLength)
{
    uint32_t        i, txData;
    uint32_t    *bufferPtr = bufPtr;

    /* Write the data in TX FIFO for 32-bit transfer */
    for(i = 0U; i < transferLength; i++)
    {
        txData = *bufferPtr;
        bufferPtr++;
        CSL_REG32_WR(baseAddr + MCSPI_CHTX(chNum), txData);
    }

    return (bufferPtr);
}

static inline uint8_t *MCSPI_fifoRead8(uint32_t  baseAddr,
                                       uint32_t  chNum,
                                       uint8_t  *bufPtr,
                                       uint32_t  transferLength,
                                       uint32_t  dataWidthBitMask)
{
    uint32_t     i, rxData;
    uint8_t     *bufferPtr = bufPtr;

    /* Read the data from RX FIFO for 8-bit transfer */
    for(i = 0U; i < transferLength; i++)
    {
        rxData = MCSPI_readRxDataReg(baseAddr, chNum);
        rxData &= dataWidthBitMask;         /* Clear unused bits */
       *bufferPtr = (uint8_t) rxData;
        bufferPtr++;
    }

    return (bufferPtr);
}

static inline uint16_t *MCSPI_fifoRead16(uint32_t  baseAddr,
                                         uint32_t  chNum,
                                         uint16_t  *bufPtr,
                                         uint32_t  transferLength,
                                         uint32_t  dataWidthBitMask)
{
    uint32_t     i, rxData;
    uint16_t     *bufferPtr = bufPtr;

    /* Read the data from RX FIFO for 16-bit transfer */
    for(i = 0U; i < transferLength; i++)
    {
        rxData = MCSPI_readRxDataReg(baseAddr, chNum);
        rxData &= dataWidthBitMask;         /* Clear unused bits */
       *bufferPtr = (uint16_t) rxData;
        bufferPtr++;
    }

    return (bufferPtr);
}

static inline uint32_t *MCSPI_fifoRead32(uint32_t  baseAddr,
                                         uint32_t  chNum,
                                         uint32_t  *bufPtr,
                                         uint32_t  transferLength,
                                         uint32_t  dataWidthBitMask)
{
    uint32_t     i, rxData;
    uint32_t     *bufferPtr = bufPtr;

    /* Read the data from RX FIFO for 32-bit transfer */
    for(i = 0U; i < transferLength; i++)
    {
        rxData = MCSPI_readRxDataReg(baseAddr, chNum);
        rxData &= dataWidthBitMask;         /* Clear unused bits */
       *bufferPtr = (uint32_t) rxData;
        bufferPtr++;
    }

    return (bufferPtr);
}

static inline void MCSPI_fifoWriteDefault(uint32_t baseAddr,
                                          uint32_t chNum,
                                          uint32_t defaultTxData,
                                          uint32_t transferLength)
{
    uint32_t        i;

    /* Write default data to TX FIFO */
    for(i = 0U; i < transferLength; i++)
    {
        CSL_REG32_WR(baseAddr + MCSPI_CHTX(chNum), defaultTxData);
    }

    return;
}

static inline void MCSPI_fifoReadDiscard(uint32_t baseAddr,
                                         uint32_t chNum,
                                         uint32_t transferLength)
{
    uint32_t            i;
    volatile uint32_t   rxData;

    /* Read the data from RX FIFO and discard it */
    for(i = 0U; i < transferLength; i++)
    {
        rxData = MCSPI_readRxDataReg(baseAddr, chNum);
        (void) rxData;
    }

    return;
}

void MCSPI_clearAllIrqStatus(uint32_t baseAddr)
{
    /* Clear all previous interrupt status */
    CSL_REG32_FINS(baseAddr + CSL_MCSPI_SYST, MCSPI_SYST_SSB, CSL_MCSPI_SYST_SSB_OFF);
    CSL_REG32_WR(baseAddr + CSL_MCSPI_IRQSTATUS, MCSPI_IRQSTATUS_CLEAR_ALL);

    return;
}

static uint32_t Spi_mcspiGetTxMask(uint32_t csNum)
{
    uint32_t txEmptyMask;

    if ((uint32_t)MCSPI_CHANNEL_0 == csNum)
    {
        txEmptyMask = (uint32_t) CSL_MCSPI_IRQSTATUS_TX0_EMPTY_MASK;
    }
    else if ((uint32_t)MCSPI_CHANNEL_1 == csNum)
    {
        txEmptyMask = (uint32_t) CSL_MCSPI_IRQSTATUS_TX1_EMPTY_MASK;
    }
    else if ((uint32_t)MCSPI_CHANNEL_2 == csNum)
    {
        txEmptyMask = (uint32_t) CSL_MCSPI_IRQSTATUS_TX2_EMPTY_MASK;
    }
    else
    {
        txEmptyMask = (uint32_t) CSL_MCSPI_IRQSTATUS_TX3_EMPTY_MASK;
    }

    return (txEmptyMask);
}

static uint32_t Spi_mcspiGetRxMask(uint32_t csNum)
{
    uint32_t rxFullMask;

    if ((uint32_t)MCSPI_CHANNEL_0 == csNum)
    {
        rxFullMask = (uint32_t) CSL_MCSPI_IRQSTATUS_RX0_FULL_MASK;
    }
    else if ((uint32_t)MCSPI_CHANNEL_1 == csNum)
    {
        rxFullMask = (uint32_t) CSL_MCSPI_IRQSTATUS_RX1_FULL_MASK;
    }
    else if ((uint32_t)MCSPI_CHANNEL_2 == csNum)
    {
        rxFullMask = (uint32_t) CSL_MCSPI_IRQSTATUS_RX2_FULL_MASK;
    }
    else
    {
        rxFullMask = (uint32_t) CSL_MCSPI_IRQSTATUS_RX3_FULL_MASK;
    }

    return (rxFullMask);
}

void MCSPI_stop(MCSPILLD_Handle hMcspi, MCSPI_ChObject *chObj, uint32_t chNum)
{
    uint32_t regVal, baseAddr;
    MCSPILLD_InitHandle hMcspiInit = hMcspi->hMcspiInit;

    baseAddr = hMcspi->baseAddr;
    if (MCSPI_OPER_MODE_INTERRUPT == hMcspiInit->operMode)
    {
        /* Disable channel interrupts. */
        regVal = CSL_REG32_RD(baseAddr + CSL_MCSPI_IRQENABLE);
        regVal &= ~(chObj->intrMask);
        CSL_REG32_WR(baseAddr + CSL_MCSPI_IRQENABLE, regVal);
    }
    MCSPI_intrStatusClear(chObj, baseAddr, chObj->intrMask);

    if(MCSPI_MS_MODE_CONTROLLER == hMcspiInit->msMode)
    {
        /* Manual CS de-assert */
        if(MCSPI_CH_MODE_SINGLE == hMcspiInit->chMode)
        {
            if (chObj->csDisable ==  TRUE)
            {
                chObj->chConfRegVal &= (~CSL_MCSPI_CH0CONF_FORCE_MASK);
                CSL_REG32_WR(baseAddr + MCSPI_CHCONF(chObj->chCfg->chNum), chObj->chConfRegVal);
                chObj->csEnable = TRUE;
            }
        }
    }

    /* Disable channel */
    CSL_REG32_FINS(
        baseAddr + MCSPI_CHCTRL(chNum),
        MCSPI_CH0CTRL_EN,
        CSL_MCSPI_CH0CTRL_EN_NACT);

    /* Disable TX and RX FIFO This is required so that next CS can
     * use the FIFO */
    chObj->chConfRegVal &= ~(CSL_MCSPI_CH0CONF_FFEW_MASK | CSL_MCSPI_CH0CONF_FFER_MASK);
    CSL_REG32_WR(baseAddr + MCSPI_CHCONF(chNum), chObj->chConfRegVal);

}

void MCSPI_setChDataSize(uint32_t baseAddr, MCSPI_ChObject *chObj,
                                uint32_t dataSize, uint32_t csDisable)
{
    uint32_t chNum;

    chNum = chObj->chCfg->chNum;
    CSL_FINS(chObj->chConfRegVal, MCSPI_CH0CONF_WL, (dataSize - 1U));
    CSL_REG32_WR(baseAddr + MCSPI_CHCONF(chNum), chObj->chConfRegVal);

    chObj->csDisable = csDisable;
    /* Calculate buffer access width */
    chObj->bufWidthShift = MCSPI_getBufWidthShift(dataSize);

    /* Calculate data width mask depending on SPI word size */
    chObj->dataWidthBitMask = MCSPI_getDataWidthBitMask(dataSize);

    chObj->effTxFifoDepth = chObj->chCfg->txFifoTrigLvl >> chObj->bufWidthShift;
    chObj->effRxFifoDepth = chObj->chCfg->rxFifoTrigLvl >> chObj->bufWidthShift;

    return;
}

uint32_t MCSPI_lld_getBaseAddr(MCSPILLD_Handle hMcspi)
{
    uint32_t            baseAddr;

    /* Check parameters */
    if (NULL == hMcspi)
    {
        baseAddr = 0U;
    }
    else
    {
        baseAddr = hMcspi->baseAddr;
    }

    return baseAddr;
}

int32_t MCSPI_lld_reConfigFifo(MCSPILLD_Handle hMcspi,
                           uint32_t chNum,
                           uint32_t numWordsRxTx)
{
    uint32_t               baseAddr;
    MCSPI_ChObject        *chObj;
    int32_t                status = MCSPI_STATUS_SUCCESS;
    MCSPILLD_InitHandle    hMcspiInit;

    /* Check parameters */
    if((NULL == hMcspi) ||
       (NULL == hMcspi->hMcspiInit) ||
       (chNum >= MCSPI_MAX_NUM_CHANNELS))
    {
        status = MCSPI_INVALID_PARAM;
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        baseAddr = hMcspi->baseAddr;
        hMcspiInit = hMcspi->hMcspiInit;
        chObj = &hMcspiInit->chObj[chNum];

        /* Re-Configure word count */
        if(MCSPI_OPER_MODE_DMA != hMcspiInit->operMode)
        {
            MCSPI_setFifoConfig(hMcspi, chObj, baseAddr, numWordsRxTx);
        }
    }

    return status;
}

static inline int32_t MCSPI_lld_isOperModeValid(uint32_t operMode)
{
    int32_t status = MCSPI_INVALID_PARAM;

    if((operMode == MCSPI_OPER_MODE_POLLED) ||
       (operMode == MCSPI_OPER_MODE_INTERRUPT) ||
       (operMode == MCSPI_OPER_MODE_DMA))
    {
        status = MCSPI_STATUS_SUCCESS;
    }

    return status;
}

static inline int32_t MCSPI_lld_isChModeValid(uint32_t chMode)
{
    int32_t status = MCSPI_INVALID_PARAM;

    if((chMode == MCSPI_CH_MODE_SINGLE) || (chMode == MCSPI_CH_MODE_MULTI))
    {
        status = MCSPI_STATUS_SUCCESS;
    }

    return status;
}

static inline int32_t MCSPI_lld_isPinModeValid(uint32_t pinMode)
{
    int32_t status = MCSPI_INVALID_PARAM;

    if((pinMode == MCSPI_PINMODE_3PIN) ||
       (pinMode == MCSPI_PINMODE_4PIN))

    {
        status = MCSPI_STATUS_SUCCESS;
    }

    return status;
}

static inline int32_t MCSPI_lld_isInitDelayValid(uint32_t initDelay)
{
    int32_t status = MCSPI_INVALID_PARAM;

    if((initDelay == MCSPI_INITDLY_0) ||
       (initDelay == MCSPI_INITDLY_4) ||
       (initDelay == MCSPI_INITDLY_8) ||
       (initDelay == MCSPI_INITDLY_16) ||
       (initDelay == MCSPI_INITDLY_32))
    {
        status = MCSPI_STATUS_SUCCESS;
    }

    return status;
}

static inline int32_t MCSPI_lld_isMsModeValid(uint32_t msMode)
{
    int32_t status = MCSPI_INVALID_PARAM;

    if((msMode == MCSPI_MS_MODE_CONTROLLER) ||
       (msMode == MCSPI_MS_MODE_PERIPHERAL))
    {
        status = MCSPI_STATUS_SUCCESS;
    }

    return status;
}

static inline int32_t MCSPI_lld_isDataSizeValid(uint32_t dataSize)
{
    int32_t status = MCSPI_INVALID_PARAM;

    if((dataSize >= 4U) && (dataSize <= 32U))
    {
        status = MCSPI_STATUS_SUCCESS;
    }

    return status;
}

static inline int32_t MCSPI_lld_isHandleValid(MCSPI_DmaHandle handle)
{
    int32_t status = MCSPI_INVALID_PARAM;

    if(handle != NULL_PTR)
    {
        status = MCSPI_STATUS_SUCCESS;
    }

    return status;
}
static inline int32_t MCSPI_lld_isParameterValid(uint32_t handleParameters)
{
    int32_t status = MCSPI_INVALID_PARAM;

    if(handleParameters != 0U)
    {
        status = MCSPI_STATUS_SUCCESS;
    }

    return status;
}

static inline int32_t MCSPI_lld_isChannelValid(uint32_t channel)
{
    int32_t status = MCSPI_INVALID_PARAM;

    if(channel < MCSPI_MAX_NUM_CHANNELS)
    {
        status = MCSPI_STATUS_SUCCESS;
    }

    return status;
}

static inline int32_t MCSPI_lld_isChCfgValid(const MCSPI_ChConfig *chCfg)
{
    int32_t status = MCSPI_INVALID_PARAM;

    if((void *)chCfg != NULL_PTR)
    {
        status = MCSPI_STATUS_SUCCESS;
    }

    return status;
}
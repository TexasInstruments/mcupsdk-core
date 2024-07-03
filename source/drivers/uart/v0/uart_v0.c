/*
 * Copyright (C) 2021-2023 Texas Instruments Incorporated
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
 *  \file   UART.c
 *
 *  \brief  This file contains the implementation of UART driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <string.h>
#include <stdint.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TaskP.h>
#include <drivers/uart/v0/lld/uart_lld.h>
#include <drivers/uart/v0/lld/dma/uart_dma.h>

/* UART Config,DMA structure handles */
extern UART_DmaHandle       gUartDmaHandle[];
extern UART_DmaChConfig     gUartDmaChConfig[];

/**
 *  \brief  This API is the callback that gets after UART write completion.
 *
 *  \param  hUart           Handle to the UART instance used
 *  \param  transaction      Structure pointing to the current transaction
 *
 */
static void UART_lld_writeCompleteCallback(void *args);

/**
 *  \brief  This API is the callback that gets after UART read completion.
 *
 *  \param  hUart           Handle to the UART instance used
 *  \param  transaction     Structure pointing to the current transaction
 *
 */
static void UART_lld_readCompleteCallback(void *args);


/**
 *  \brief  This API is the callback that gets called when a UART error occurs.
 *
 *  \param  hUart           Handle to the UART instance used
 *  \param  transaction     Structure pointing to the current transaction. The transaction
 *                          status holds the error flag.
 *
 */
static void UART_lld_errorCallback(void *args);

/**
 *  \brief  This function checks the openParameters for UART
 *
 *  \param  prms        Pointer to open parameters. If NULL is passed, then
 *                      default values will be used
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 */
static int32_t UART_checkOpenParams(const UART_Params *prms);

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define UART_FIFO_CONFIG(txGra, rxGra, txTrig, rxTrig, txClr, rxClr, dmaEnPath, \
                         dmaMode)                                               \
    (((uint32_t) (txGra & 0xFU) << (uint32_t)26U)    |                       \
     ((uint32_t) (rxGra & 0xFU) << (uint32_t)22U)    |                       \
     ((uint32_t) (txTrig & 0xFFU) << (uint32_t)14U)  |                       \
     ((uint32_t) (rxTrig & 0xFFU) << (uint32_t)6U)   |                       \
     ((uint32_t) (txClr & 0x1U) << (uint32_t)5U)     |                       \
     ((uint32_t) (rxClr & 0x1U) << (uint32_t)4U)     |                       \
     ((uint32_t) (dmaEnPath & 0x1U) << (uint32_t)3U) |                       \
     (uint32_t) (dmaMode & 0x7U))

#define UART_FIFO_CONFIG_TXGRA          ((uint32_t) 0xFU << 26)
#define UART_FIFO_CONFIG_RXGRA          ((uint32_t) 0xFU << 22)
#define UART_FIFO_CONFIG_TXTRIG         ((uint32_t) 0xFFU << 14)
#define UART_FIFO_CONFIG_RXTRIG         ((uint32_t) 0xFFU << 6)
#define UART_FIFO_CONFIG_TXCLR          ((uint32_t) 0x1U << 5)
#define UART_FIFO_CONFIG_RXCLR          ((uint32_t) 0x1U << 4)
#define UART_FIFO_CONFIG_DMAENPATH      ((uint32_t) 0x1U << 3)
#define UART_FIFO_CONFIG_DMAMODE        ((uint32_t) 0x7U << 0)

#define UART_TRIG_LVL_GRANULARITY_4     ((uint32_t) 0x0000U)
#define UART_TRIG_LVL_GRANULARITY_1     ((uint32_t) 0x0001U)

#define UART_DMA_EN_PATH_FCR            (UART_SCR_DMA_MODE_CTL_DMA_MODE_CTL_VALUE_0)
#define UART_DMA_EN_PATH_SCR            (UART_SCR_DMA_MODE_CTL_DMA_MODE_CTL_VALUE_1)

#define UART_INT2_RX_EMPTY              (UART_IER2_EN_RXFIFO_EMPTY_MASK)
#define UART_INT2_TX_EMPTY              (UART_IER2_EN_TXFIFO_EMPTY_MASK)

#define UART_DMA_MODE_0_ENABLE          (UART_SCR_DMA_MODE_2_DMA_MODE_2_VALUE_0)
#define UART_DMA_MODE_1_ENABLE          (UART_SCR_DMA_MODE_2_DMA_MODE_2_VALUE_1)
#define UART_DMA_MODE_2_ENABLE          (UART_SCR_DMA_MODE_2_DMA_MODE_2_VALUE_2)
#define UART_DMA_MODE_3_ENABLE          (UART_SCR_DMA_MODE_2_DMA_MODE_2_VALUE_3)

#define UART_MIR_OVERSAMPLING_RATE_41   ((uint32_t) 41U)
#define UART_MIR_OVERSAMPLING_RATE_42   ((uint32_t) 42U)

#define UART_BREAK_COND_DISABLE         (UART_LCR_BREAK_EN_BREAK_EN_VALUE_0 \
                                            << UART_LCR_BREAK_EN_SHIFT)
#define UART_BREAK_COND_ENABLE          (UART_LCR_BREAK_EN_BREAK_EN_VALUE_1 \
                                            << UART_LCR_BREAK_EN_SHIFT)
#define UART_NO_HARDWARE_FLOW_CONTROL    (UART_EFR_HW_NO_FLOW_CONTROL_VALUE)
#define UART_RTS_ENABLE                  (UART_EFR_HW_ENABLE_RTS_VALUE)
#define UART_CTS_ENABLE                  (UART_EFR_HW_ENALE_CTS_VALUE)
#define UART_RTS_CTS_ENABLE              (UART_EFR_HW_ENABLE_RTS_CTS_FLOW_CONTROL_VALUE)

#define UART_TIMEOUTL                       (0x98U)
#define UART_TIMEOUTH                       (0x9CU)

#define UART_EFR2                             (0x8CU)
#define UART_EFR2_TIMEOUT_BEHAVE_SHIFT        (0x6U)
#define UART_EFR2_TIMEOUT_BEHAVE_MASK         (0x6U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct
{
    void                   *lock;
    /**< Driver lock - to protect across open/close */
    SemaphoreP_Object       lockObj;
    /**< Driver lock object */
} UART_DrvObj;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Driver object */
static UART_DrvObj     gUartDrvObj =
{
    .lock           = NULL,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void UART_init(void)
{
    int32_t         status;
    uint32_t        cnt;
    UART_Object    *object;

    /* Init each driver instance object */
    for (cnt = 0U; cnt < gUartConfigNum; cnt++)
    {
        /* initialize object varibles */
        object = gUartConfig[cnt].object;
        DebugP_assert(NULL_PTR != object);
        (void)memset(object, 0, sizeof(UART_Object));
        gUartConfig[cnt].attrs->baseAddr = (uint32_t) AddrTranslateP_getLocalAddr((uint64_t)gUartConfig[cnt].attrs->baseAddr);
    }

    /* Create driver lock */
    status = SemaphoreP_constructMutex(&gUartDrvObj.lockObj);
    if(SystemP_SUCCESS == status)
    {
        gUartDrvObj.lock = &gUartDrvObj.lockObj;
    }

    return;
}

void UART_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gUartDrvObj.lock)
    {
        SemaphoreP_destruct(&gUartDrvObj.lockObj);
        gUartDrvObj.lock = NULL;
    }

    return;
}

static int32_t UART_checkOpenParams(const UART_Params *prms)
{
    int32_t     status = SystemP_SUCCESS;

    if((UART_TRANSFER_MODE_CALLBACK == prms->readMode) &&
       (NULL_PTR == prms->readCallbackFxn))
    {
        status = SystemP_FAILURE;
    }
    if((UART_TRANSFER_MODE_CALLBACK == prms->writeMode) &&
       (NULL_PTR == prms->writeCallbackFxn))
    {
        status = SystemP_FAILURE;
    }

    return (status);
}

UART_Handle UART_open(uint32_t index, const UART_Params *prms)
{
    int32_t             status = SystemP_SUCCESS;
    UART_Handle         handle = NULL;
    UART_Config        *config = NULL;
    UART_Object        *object    = NULL;
    const UART_Attrs   *attrs;
    HwiP_Params         hwiPrms;
    UARTLLD_Handle      uartLld_handle;
    UARTLLD_InitHandle  uartLldInit_handle;

    /* Check index */
    if(index >= gUartConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gUartConfig[index];
    }

    DebugP_assert(NULL_PTR != gUartDrvObj.lock);
    (void)SemaphoreP_pend(&gUartDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(SystemP_SUCCESS  == status)
    {
        object = config->object;
        attrs  = config->attrs;
        DebugP_assert(NULL_PTR != object);
        if(TRUE == object->isOpen)
        {
            /* Handle is already opened */
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        /* Init state */
        object->handle = (UART_Handle) config;
        if(NULL != prms)
        {
            ( void )memcpy(&object->prms, prms, sizeof(UART_Params));
        }
        else
        {
            /* Init with default if NULL is passed */
            UART_Params_init(&object->prms);

        }

         /*  Mapping HLD parameter with LLD. */
        object->uartLld_handle             = &object->uartLld_object;
        uartLld_handle                     = object->uartLld_handle;

        object->uartLld_initHandle         = &object->uartLld_initObject;
        uartLldInit_handle                 = object->uartLld_initHandle;

        uartLld_handle->hUartInit          = uartLldInit_handle;
        uartLld_handle->baseAddr           = attrs->baseAddr;
        uartLld_handle->args               = (void *)object->handle;
        uartLld_handle->writeBuf           = object->writeBuf;
        uartLld_handle->writeCount         = object->writeCount;
        uartLld_handle->writeSizeRemaining = object->writeSizeRemaining;
        uartLld_handle->readBuf            = object->readBuf;
        uartLld_handle->readCount          = object->readCount;
        uartLld_handle->readSizeRemaining  = object->readSizeRemaining;
        uartLld_handle->rxTimeoutCnt       = object->rxTimeoutCnt;
        uartLld_handle->readErrorCnt       = object->readErrorCnt;
        uartLld_handle->state              = UART_STATE_RESET;

        uartLldInit_handle->inputClkFreq      = attrs->inputClkFreq;
        uartLldInit_handle->baudRate          = object->prms.baudRate;
        uartLldInit_handle->baudRate          = object->prms.baudRate;
        uartLldInit_handle->dataLength        = object->prms.dataLength;
        uartLldInit_handle->stopBits          = object->prms.stopBits;
        uartLldInit_handle->parityType        = object->prms.parityType;
        uartLldInit_handle->readReturnMode    = object->prms.readReturnMode;
        uartLldInit_handle->hwFlowControl     = object->prms.hwFlowControl;
        uartLldInit_handle->hwFlowControlThr  = object->prms.hwFlowControlThr;
        uartLldInit_handle->intrNum           = object->prms.intrNum;
        uartLldInit_handle->transferMode      = object->prms.transferMode;
        uartLldInit_handle->intrPriority      = object->prms.intrPriority;
        uartLldInit_handle->operMode          = object->prms.operMode;
        uartLldInit_handle->rxTrigLvl         = object->prms.rxTrigLvl;
        uartLldInit_handle->txTrigLvl         = object->prms.txTrigLvl;
        uartLldInit_handle->uartDmaHandle     = NULL;
        uartLldInit_handle->dmaChCfg          = NULL;
        uartLldInit_handle->rxEvtNum          = object->prms.rxEvtNum;
        uartLldInit_handle->txEvtNum          = object->prms.txEvtNum;
        uartLldInit_handle->writeMode         = object->prms.writeMode;
        uartLldInit_handle->readMode          = object->prms.readMode;
        uartLldInit_handle->timeGuardVal      = object->prms.timeGuardVal;
        uartLldInit_handle->clockP_get        = ClockP_getTicks;
        uartLldInit_handle->clockP_usecToTick = ClockP_usecToTicks;
        /* Read, Write & Error callback Functions */
        uartLldInit_handle->readCompleteCallbackFxn =  UART_lld_readCompleteCallback;
        uartLldInit_handle->writeCompleteCallbackFxn = UART_lld_writeCompleteCallback;
        uartLldInit_handle->errorCallbackFxn =         UART_lld_errorCallback;

        /* Check open parameters */
        status = UART_checkOpenParams(&object->prms);
    }

    if(SystemP_SUCCESS == status)
    {
        uartLld_handle->state = UART_STATE_RESET;

        /* If DMA is enabled, program DMA */
        if(UART_CONFIG_MODE_DMA == object->prms.transferMode)
        {
            uartLldInit_handle->uartDmaHandle = (UART_DmaHandle) gUartDmaHandle[index];
            uartLldInit_handle->dmaChCfg      = gUartDmaChConfig[index];
            status = UART_lld_initDma(uartLld_handle);
        }
        else
        {
            status = UART_lld_init(uartLld_handle);
            object->uartDmaHandle = NULL;
        }

        if(SystemP_SUCCESS == status)
        {
            /* Create instance lock */
            status = SemaphoreP_constructMutex(&object->lockObj);
            if(SystemP_SUCCESS == status)
            {
                object->lock = &object->lockObj;
            }
            /* Create transfer sync semaphore */
            status += SemaphoreP_constructBinary(&object->readTransferSemObj, 0U);
            if(SystemP_SUCCESS == status)
            {
                object->readTransferSem = &object->readTransferSemObj;
                uartLld_handle->readTransferMutex = object->readTransferSem;
            }
            status += SemaphoreP_constructBinary(&object->writeTransferSemObj, 0U);
            if(SystemP_SUCCESS == status)
            {
                object->writeTransferSem = &object->writeTransferSemObj;
                uartLld_handle->writeTransferMutex = object->writeTransferSem;
            }

            /* Register interrupt */
            if((UART_CONFIG_MODE_INTERRUPT == object->prms.transferMode) && (TRUE != object->prms.skipIntrReg))
            {
                DebugP_assert(object->prms.intrNum != 0xFFFFU);
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum      = object->prms.intrNum;
                hwiPrms.callback    = &UART_lld_controllerIsr;
                hwiPrms.priority    = object->prms.intrPriority;
                hwiPrms.args        = (void *) uartLld_handle;
                status += HwiP_construct(&object->hwiObj, &hwiPrms);
                if(SystemP_SUCCESS == status)
                {
                    object->hwiHandle = &object->hwiObj;
                }
            }
        }
    }

    if(SystemP_SUCCESS == status)
    {
        object->isOpen = TRUE;
        handle = (UART_Handle) config;
    }

    SemaphoreP_post(&gUartDrvObj.lockObj);

    /* Free-up resources in case of error */
    if(SystemP_SUCCESS != status)
    {
        if(NULL != config)
        {
            UART_close((UART_Handle) config);
        }
    }

    return (handle);
}

UART_Handle UART_getHandle(uint32_t index)
{
    UART_Handle         handle = NULL;

    /* Check index */
    if(index < gUartConfigNum)
    {
        UART_Object *object;

        object = gUartConfig[index].object;

        if(object && (TRUE == object->isOpen))
        {
            /* valid handle */
            handle = object->handle;
        }
    }

    return handle;
}

void UART_close(UART_Handle handle)
{
    UART_Config        *config;
    UART_Object        *object;
    const UART_Attrs   *attrs;

    config = (UART_Config *) handle;
    UARTLLD_Handle      uartLld_handle;

    if ((NULL != config) && (config->object != NULL) && (config->object->isOpen != FALSE))
    {
        object = config->object;
        attrs = config->attrs;
        object->uartLld_handle = &object->uartLld_object;
        uartLld_handle = object->uartLld_handle;
        DebugP_assert(NULL_PTR != object);
        DebugP_assert(NULL_PTR != attrs);

        DebugP_assert(NULL_PTR != gUartDrvObj.lock);
        (void)SemaphoreP_pend(&gUartDrvObj.lockObj, SystemP_WAIT_FOREVER);

        /* Flush TX FIFO */
        UART_flushTxFifo(handle);

        /* Disable UART and interrupts. */
        UART_intrDisable(attrs->baseAddr,
                       UART_INTR_RHR_CTI | UART_INTR_THR | UART_INTR_LINE_STAT);
        UART_intr2Disable(attrs->baseAddr, UART_INT2_TX_EMPTY);
        (void)UART_operatingModeSelect(attrs->baseAddr, UART_OPER_MODE_DISABLED);

        if(UART_CONFIG_MODE_DMA == object->prms.transferMode)
        {
            (void)UART_lld_deInitDma(uartLld_handle);
        }
        else
        {
            (void)UART_lld_deInit(uartLld_handle);
        }
        if(NULL != object->lock)
        {
            SemaphoreP_destruct(&object->lockObj);
            object->lock = NULL;
        }
        if(NULL != object->readTransferSem)
        {
            SemaphoreP_destruct(&object->readTransferSemObj);
            object->readTransferSem = NULL;
            uartLld_handle->readTransferMutex = NULL;
        }
        if(NULL != object->writeTransferSem)
        {
            SemaphoreP_destruct(&object->writeTransferSemObj);
            object->writeTransferSem = NULL;
            uartLld_handle->writeTransferMutex = NULL;
        }
        if(NULL != object->hwiHandle)
        {
            HwiP_destruct(&object->hwiObj);
            object->hwiHandle = NULL;
        }

        object->isOpen = FALSE;
        SemaphoreP_post(&gUartDrvObj.lockObj);
    }

    return;
}

int32_t UART_write(UART_Handle handle, UART_Transaction *trans)
{
    int32_t             status = SystemP_SUCCESS, semStatus = SystemP_SUCCESS;
    UART_Config        *config;
    UART_Object        *object;
    const UART_Attrs   *attrs;
    UART_Params        *prms;
    uintptr_t           key;
    UARTLLD_Handle      uartLld_handle;
    UART_ExtendedParams extendedParams;

    /* Check parameters */
    if ((NULL_PTR == handle) || (NULL_PTR == trans))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config  = (UART_Config *) handle;
        object  = config->object;
        attrs   = config->attrs;
        prms    = &config->object->prms;
        uartLld_handle = object->uartLld_handle;
        object->writeTrans = trans;

        DebugP_assert(NULL_PTR != object);
        DebugP_assert(NULL_PTR != attrs);

        /* Assinging the Extended Parameters */
        extendedParams.args = trans->args;

    }

    if(SystemP_SUCCESS == status)
    {
        /* When User Managed Interrupt is set, user need to manage the read/write operation */
        if (TRUE == prms->skipIntrReg)
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        key = HwiP_disable();

        HwiP_restore(key);

        uartLld_handle->state = UART_STATE_READY;

        /* Interrupt mode */
        if ((UART_CONFIG_MODE_INTERRUPT == prms->transferMode) ||
            (UART_CONFIG_MODE_DMA == prms->transferMode))
        {
            if (UART_CONFIG_MODE_INTERRUPT == prms->transferMode)
            {
                status = UART_lld_writeIntr(uartLld_handle, trans->buf, trans->count, &extendedParams);
            }
            else
            {
                status = UART_lld_writeDma(uartLld_handle, trans->buf, trans->count, &extendedParams);
            }
            if (SystemP_SUCCESS == status)
            {
                if(object->prms.writeMode == UART_TRANSFER_MODE_BLOCKING)
                {
                    /* Pend on lock and wait for Hwi to finish. */
                    semStatus = SemaphoreP_pend(&object->writeTransferSemObj, trans->timeout);
                    if (semStatus == SystemP_SUCCESS)
                    {
                        if (trans->status == (uint32_t)UART_STATUS_SUCCESS)
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
                        trans->status = UART_TRANSFER_TIMEOUT;
                        /* Cancel the DMA without posting the semaphore */
                        (void)UART_writeCancelNoCB(uartLld_handle);
                        status = SystemP_FAILURE;
                    }
                }
                else
                {
                    /*
                    * for callback mode, immediately return SUCCESS,
                    * once the transaction is done, callback function
                    * will return the transaction status and actual
                    * write count
                    */
                    status = SystemP_SUCCESS;
                }
            }
        }
        else
        {
            /* Polled mode */
            status = UART_lld_write(uartLld_handle, trans->buf, trans->count, trans->timeout, &extendedParams);
        }
    }

    return (status);
}

int32_t UART_read(UART_Handle handle, UART_Transaction *trans)
{
    int32_t             status = SystemP_SUCCESS, semStatus = SystemP_SUCCESS;
    UART_Config        *config;
    UART_Object        *object;
    const UART_Attrs   *attrs;
    UART_Params        *prms;
    uintptr_t           key;
    UARTLLD_Handle      uartLld_handle;
    UART_ExtendedParams extendedParams;

    /* Check parameters */
    if ((NULL_PTR == handle) || (NULL_PTR == trans))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config  = (UART_Config *) handle;
        object  = config->object;
        attrs   = config->attrs;
        prms    = &config->object->prms;
        uartLld_handle = object->uartLld_handle;
        object->readTrans = trans;

        DebugP_assert(NULL_PTR != object);
        DebugP_assert(NULL_PTR != attrs);

        /* Assinging the Extended Parameters */
        extendedParams.args = trans->args;
    }

    if(SystemP_SUCCESS == status)
    {
        /* When User Managed Interrupt is set, user need to manage the read/write operation */
        if (TRUE == prms->skipIntrReg)
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        key = HwiP_disable();

        HwiP_restore(key);

        /* Interrupt mode */
        if ((UART_CONFIG_MODE_INTERRUPT == prms->transferMode) ||
            (UART_CONFIG_MODE_DMA == prms->transferMode))
        {
            if (UART_CONFIG_MODE_INTERRUPT == prms->transferMode)
            {
                status = UART_lld_readIntr(uartLld_handle, trans->buf, trans->count, &extendedParams);
            }
            else
            {
                status = UART_lld_readDma(uartLld_handle, trans->buf, trans->count, &extendedParams);
            }
            if (SystemP_SUCCESS == status)
            {
                if(object->prms.readMode == UART_TRANSFER_MODE_BLOCKING)
                {
                    /* Pend on lock and wait for Hwi to finish. */
                    semStatus = SemaphoreP_pend(&object->readTransferSemObj, trans->timeout);
                    if (semStatus == SystemP_SUCCESS)
                    {
                        if (trans->status == (uint32_t)UART_STATUS_SUCCESS)
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
                        trans->status = UART_TRANSFER_TIMEOUT;
                        /* Cancel the DMA without posting the semaphore */
                        (void)UART_readCancelNoCB(uartLld_handle);
                        status = SystemP_FAILURE;
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
                    status = SystemP_SUCCESS;
                }
            }
        }
        else
        {
            /* Polled mode */
            if (UART_CONFIG_MODE_POLLED == prms->transferMode)
                status = UART_lld_read(uartLld_handle, trans->buf, trans->count, trans->timeout, &extendedParams);
            else
                status = UART_lld_readWithCounter(uartLld_handle, trans->buf, trans->count, trans->timeout, &extendedParams);
        }
    }

    return (status);
}

int32_t UART_writeCancel(UART_Handle handle, UART_Transaction *trans)
{
    int32_t             status = SystemP_SUCCESS;
    UART_Config        *config;
    UART_Object        *object;
    UART_Params        *prms;
    UARTLLD_Handle      uartLld_handle;

    /* Check parameters */
    if((NULL == handle) || (NULL == trans))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config = (UART_Config *) handle;
        object = config->object;
        prms    = &config->object->prms;
        DebugP_assert(NULL != object);
        uartLld_handle = object->uartLld_handle;

        /* When User Managed Interrupt is set, user need to manage the read/write operation */
        if (TRUE == prms->skipIntrReg)
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        status = UART_lld_writeCancel(uartLld_handle, trans);
        if(SystemP_SUCCESS == status)
        {
            object->writeTrans->status = UART_TRANSFER_STATUS_CANCELLED;
            /*
            * Post transfer Sem in case of bloacking transfer.
            * Call the callback function in case of Callback mode.
            */
            if (object->prms.writeMode == UART_TRANSFER_MODE_CALLBACK)
            {
                 object->prms.writeCallbackFxn((UART_Handle) config, object->writeTrans);
            }
            else
            {
                (void)SemaphoreP_post(&object->writeTransferSemObj);
            }
            object->writeTrans = NULL;
        }
        else
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_OTH;
            status = SystemP_FAILURE;
        }
    }

    return (status);
}

int32_t UART_readCancel(UART_Handle handle, UART_Transaction *trans)
{
    int32_t             status = SystemP_SUCCESS;
    UART_Config        *config;
    UART_Object        *object;
    UART_Params        *prms;
    UARTLLD_Handle      uartLld_handle;

    /* Check parameters */
    if((NULL == handle) || (NULL == trans))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config = (UART_Config *) handle;
        object = config->object;
        prms    = &config->object->prms;
        uartLld_handle = object->uartLld_handle;
        DebugP_assert(NULL != object);

        /* When User Managed Interrupt is set, user need to manage the read/write operation */
        if (TRUE == prms->skipIntrReg)
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        status = UART_lld_readCancel(uartLld_handle, trans);
        if (status == SystemP_SUCCESS)
        {
            object->readTrans->status = UART_TRANSFER_STATUS_CANCELLED;
            if (object->prms.readMode == UART_TRANSFER_MODE_CALLBACK)
            {
                object->prms.readCallbackFxn((UART_Handle) config, object->readTrans);
            }
            else
            {
                (void)SemaphoreP_post(&object->readTransferSemObj);
            }
            object->readTrans = NULL;
        }
        else
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_OTH;
            status = SystemP_FAILURE;
        }
    }

    return (status);
}

void UART_flushTxFifo(UART_Handle handle)
{
    UART_Config        *config;
    const UART_Attrs   *attrs;
    uint32_t            isTxFifoEmpty, startTicks, elapsedTicks;
    uint32_t            timeout = UART_TRANSMITEMPTY_TRIALCOUNT;
    uint32_t            timeoutElapsed  = FALSE;

    config = (UART_Config *) handle;

    if (NULL != config)
    {
        attrs = config->attrs;
        DebugP_assert(NULL_PTR != attrs);

        /* Update current tick value to perform timeout operation */
        startTicks = ClockP_getTicks();
        while (FALSE == timeoutElapsed)
        {
            /* Get TX FIFO status */
            isTxFifoEmpty = UART_spaceAvail(attrs->baseAddr);
            if (TRUE == isTxFifoEmpty)
            {
                /* FIFO and Shift register is empty */
                break;
            }

            /* Check whether timeout happened or not */
            elapsedTicks = ClockP_getTicks() - startTicks;
            if (elapsedTicks >= timeout)
            {
                /* timeout occured */
                timeoutElapsed = TRUE;
            }
            else
            {
                TaskP_yield();
            }
        }
        DebugP_assert(FALSE == timeoutElapsed);
    }

    return;
}

static void UART_lld_writeCompleteCallback(void *args)
{
    UART_Config   *config;
    UART_Object   *obj;

    UARTLLD_Handle hUart = (UARTLLD_Handle)args;

    if(NULL_PTR != hUart)
    {
        UART_Handle handle = (UART_Handle)hUart->args;
        if(NULL_PTR != handle)
        {
            config = (UART_Config *) handle;
            obj = config->object;
            obj->writeTrans->count = hUart->writeTrans.count;
            if (obj->prms.writeMode == UART_TRANSFER_MODE_CALLBACK)
            {
                obj->prms.writeCallbackFxn(hUart, &hUart->writeTrans);
            }
            else
            {
                SemaphoreP_post((SemaphoreP_Object *)hUart->writeTransferMutex);
            }
        }
    }

}

static void UART_lld_readCompleteCallback(void *args)
{
    UART_Config   *config;
    UART_Object   *obj;

    UARTLLD_Handle hUart = (UARTLLD_Handle)args;

    if(NULL_PTR != hUart)
    {
        UART_Handle handle = (UART_Handle)hUart->args;
        if(NULL_PTR != handle)
        {
            config = (UART_Config *) handle;
            obj = config->object;
            obj->readTrans->count = hUart->readTrans.count;
            if (obj->prms.readMode == UART_TRANSFER_MODE_CALLBACK)
            {
                obj->prms.readCallbackFxn(hUart, &hUart->readTrans);
            }
            else
            {
                SemaphoreP_post((SemaphoreP_Object *)hUart->readTransferMutex);
            }
        }
    }
}

static void UART_lld_errorCallback(void *args)
{
    UART_NOT_IN_USE(args);
}

/**
 *  \brief  Function to initialize the #UART_Params struct to its defaults
 *
 *  \param  trans       Pointer to #UART_Params structure for
 *                      initialization
 */
void UART_Params_init(UART_Params *prms)
{
    if(prms != NULL)
    {
        prms->baudRate           = 115200U;
        prms->dataLength         = UART_LEN_8;
        prms->stopBits           = UART_STOPBITS_1;
        prms->parityType         = UART_PARITY_NONE;
        prms->readMode           = UART_TRANSFER_MODE_BLOCKING;
        prms->readReturnMode     = UART_READ_RETURN_MODE_FULL;
        prms->writeMode          = UART_TRANSFER_MODE_BLOCKING;
        prms->readCallbackFxn    = NULL;
        prms->writeCallbackFxn   = NULL;
        prms->hwFlowControl      = FALSE;
        prms->hwFlowControlThr   = UART_RXTRIGLVL_16;
        prms->intrNum            = 0xFFFF;
        prms->transferMode       = UART_CONFIG_MODE_INTERRUPT;
        prms->intrPriority       = 4U;
        prms->skipIntrReg        = FALSE;
        prms->uartDmaIndex       = -1;
        prms->operMode           = UART_OPER_MODE_16X;
        prms->rxTrigLvl          = UART_RXTRIGLVL_8;
        prms->txTrigLvl          = UART_TXTRIGLVL_32;
    }
}

/**
 *  \brief  Function to initialize the #UART_Transaction struct to its defaults
 *
 *  \param  trans       Pointer to #UART_Transaction structure for
 *                      initialization
 */
void UART_Transaction_init(UART_Transaction *trans)
{
    if(trans != NULL)
    {
        trans->buf              = NULL;
        trans->count            = 0U;
        trans->timeout          = SystemP_WAIT_FOREVER;
        trans->status           = UART_STATUS_SUCCESS;
        trans->args             = NULL;
    }
}

/* ========================================================================== */
/*                       Advanced Function Definitions                        */
/* ========================================================================== */

uint32_t UART_getBaseAddr(UART_Handle handle)
{
    UART_Config       *config;
    UART_Attrs        *attrs;
    uint32_t           baseAddr;

    /* Check parameters */
    if (NULL_PTR == handle)
    {
        baseAddr = 0U;
    }
    else
    {
        config = (UART_Config *) handle;
        attrs = config->attrs;
        baseAddr = attrs->baseAddr;
    }

    return baseAddr;
}

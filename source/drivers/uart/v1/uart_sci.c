/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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
#include <drivers/uart.h>
#include <drivers/uart/v1/uart_sci_edma.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TaskP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief Timeout in ms used for TX FIFO empty at the time of delete. Three
 *  seconds is more than sufficient to transfer 64 bytes (FIFO size) at the
 *  lowest baud rate of 2400.
 */
#define UART_TRANSMITEMPTY_TRIALCOUNT   (3000U)

#define UARTSCI_TIMING_MODE_SYNC        ((uint32_t) 0U)
#define UARTSCI_TIMING_MODE_ASYNC       ((uint32_t) 1U)

#define UARTSCI_CLOCK_MODE_EXTERNAL     ((uint32_t) 0U)
#define UARTSCI_CLOCK_MODE_INTERNAL     ((uint32_t) 1U)

#define UARTSCI_COMM_MODE_IDLE          ((uint32_t) 0U)
#define UARTSCI_COMM_MODE_ADDRESS       ((uint32_t) 1U)

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
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Driver internal functions */
static void UART_masterIsr(void *arg);

static Bool UART_writeCancelNoCB(UART_Object *object, UART_Attrs const *attrs);
static Bool UART_readCancelNoCB(UART_Object *object, UART_Attrs const *attrs);

static int32_t UART_writePolling(UART_Object *object,
                                 UART_Attrs const *attrs,
                                 UART_Transaction *trans);
static int32_t UART_writeInterrupt(UART_Object *object,
                                   UART_Attrs const *attrs,
                                   UART_Transaction *trans);

static int32_t UART_readPolling(UART_Config      *config,
                                UART_Object      *object,
                                UART_Attrs const *attrs,
                                UART_Transaction *trans);
static int32_t UART_readInterrupt(UART_Config      *config,
                                UART_Object      *object,
                                UART_Attrs const *attrs,
                                UART_Transaction *trans);

static void UART_configInstance(UART_Config *config);
static int32_t UART_checkOpenParams(const UART_Params *prms);
static int32_t UART_checkTransaction(const UART_Object *object,
                                     UART_Transaction *trans);

/* Low level HW functions */
static inline void UART_txChar(UART_Object *object);
static inline void UART_rxChar(UART_Object *object);
static inline uint32_t UART_isTxRdy(const CSL_sciRegs *pSCIRegs);
static inline uint32_t UART_isRxRdy(const CSL_sciRegs *pSCIRegs);
static inline void UART_txIntrEnable(CSL_sciRegs *pSCIRegs);
static inline void UART_rxIntrEnable(CSL_sciRegs *pSCIRegs);
static inline void UART_txIntrDisable(CSL_sciRegs *pSCIRegs);
static inline void UART_rxIntrDisable(CSL_sciRegs *pSCIRegs);
static inline uint32_t UART_isTxIntrEnabled(const CSL_sciRegs* pSCIRegs);
static inline uint32_t UART_isRxIntrEnabled(const CSL_sciRegs* pSCIRegs);

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
        DebugP_assert(NULL != object);
        memset(object, 0, sizeof(UART_Object));
        gUartConfig[cnt].attrs->baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(gUartConfig[cnt].attrs->baseAddr);
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

UART_Handle UART_open(uint32_t index, const UART_Params *prms)
{
    int32_t             status = SystemP_SUCCESS;
    UART_Handle         handle = NULL;
    UART_Config        *config = NULL;
    UART_Object        *object    = NULL;
    HwiP_Params         hwiPrms;

    /* Check index */
    if(index >= gUartConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gUartConfig[index];
    }

    DebugP_assert(NULL != gUartDrvObj.lock);
    SemaphoreP_pend(&gUartDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(SystemP_SUCCESS == status)
    {
        DebugP_assert(NULL != config->object);
        DebugP_assert(NULL != config->attrs);
        object = config->object;
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
            memcpy(&object->prms, prms, sizeof(UART_Params));
        }
        else
        {
            /* Init with default if NULL is passed */
            UART_Params_init(&object->prms);
        }

        /* Check open parameters */
        status = UART_checkOpenParams(&object->prms);
    }

    if(SystemP_SUCCESS == status)
    {
        /* Configure the UART instance parameters */
        UART_configInstance(config);

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
        }
        status += SemaphoreP_constructBinary(&object->writeTransferSemObj, 0U);
        if(SystemP_SUCCESS == status)
        {
            object->writeTransferSem = &object->writeTransferSemObj;
        }

        /* Register interrupt */
        if(UART_CONFIG_MODE_INTERRUPT == object->prms.transferMode)
        {
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = object->prms.intrNum;
            hwiPrms.callback    = &UART_masterIsr;
            hwiPrms.priority    = object->prms.intrPriority;
            hwiPrms.args        = (void *) config;
            status += HwiP_construct(&object->hwiObj, &hwiPrms);
            if(SystemP_SUCCESS == status)
            {
                object->hwiHandle = &object->hwiObj;
            }
        }

        /* Initialize the DMA configuration */
        if(UART_CONFIG_MODE_DMA == object->prms.transferMode)
        {
            UART_edmaChannelConfig((UART_Handle) config, object->prms.edmaInst);
        }

        /* Start the SCI */
        DebugP_assert(NULL != object->pSCIRegs);
        CSL_FINS(object->pSCIRegs->SCIGCR1, SCI_SCIGCR1_SW_NRESET, 1U);
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

    if((NULL != config) &&
       (config->object != NULL) &&
       (config->object->isOpen != (uint32_t)FALSE))
    {
        object = config->object;
        attrs = config->attrs;

        DebugP_assert(NULL != object);
        DebugP_assert(NULL != attrs);

        DebugP_assert(NULL != gUartDrvObj.lock);
        SemaphoreP_pend(&gUartDrvObj.lockObj, SystemP_WAIT_FOREVER);

        /* Update current tick value to perform timeout operation */
        DebugP_assert(NULL != object->pSCIRegs);

        /* Flush TX FIFO */
        UART_flushTxFifo(handle);

        /* Disable UART and interrupts. */
        DebugP_assert(NULL != object->pSCIRegs);
        CSL_REG_WR(&object->pSCIRegs->SCICLEARINT, 0xFFFFFFFFU);

        if(NULL != object->lock)
        {
            SemaphoreP_destruct(&object->lockObj);
            object->lock = NULL;
        }
        if(NULL != object->readTransferSem)
        {
            SemaphoreP_destruct(&object->readTransferSemObj);
            object->readTransferSem = NULL;
        }
        if(NULL != object->writeTransferSem)
        {
            SemaphoreP_destruct(&object->writeTransferSemObj);
            object->writeTransferSem = NULL;
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
    int32_t             status = SystemP_SUCCESS;
    UART_Config        *config;
    UART_Object        *object;
    const UART_Attrs   *attrs;
    uintptr_t           key;

    /* Check parameters */
    if((NULL == handle) || (NULL == trans))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config  = (UART_Config *) handle;
        object  = config->object;
        attrs   = config->attrs;

        DebugP_assert(NULL != object);
        DebugP_assert(NULL != attrs);

        if(object->isOpen == TRUE)
        {
            status = UART_checkTransaction(object, trans);
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        /* When User Managed Interrupt is set, user need to manage the read/write operation */
        if(UART_CONFIG_MODE_USER_INTR == object->prms.transferMode)
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {

        key = HwiP_disable();

        /* Check if any transaction is in progress */
        if(NULL != object->writeTrans)
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_INUSE;
            status = SystemP_FAILURE;
        }
        else
        {
            /* Initialize transaction params */
            object->writeTrans              = trans;
            object->writeBuf                = trans->buf;
            object->writeTrans->timeout     = trans->timeout;
            object->writeCount              = 0U;
            object->writeSizeRemaining      = trans->count;
        }

        HwiP_restore(key);

        if(SystemP_SUCCESS == status)
        {
            /* Interrupt mode */
            if(UART_CONFIG_MODE_INTERRUPT == object->prms.transferMode)
            {
                status = UART_writeInterrupt(object, attrs, trans);
            }
            else if(UART_CONFIG_MODE_POLLED == object->prms.transferMode)
            {
                /* Polled mode */
                status = UART_writePolling(object, attrs, trans);
            }
            else if(UART_CONFIG_MODE_DMA == object->prms.transferMode)
            {
                /* DMA mode */
                status = UART_writeDma(config, object, attrs, trans);
            }
        }
    }

    return (status);
}

int32_t UART_read(UART_Handle handle, UART_Transaction *trans)
{
    int32_t             status = SystemP_SUCCESS;
    UART_Config        *config;
    UART_Object        *object;
    const UART_Attrs   *attrs;
    uintptr_t           key;

    /* Check parameters */
    if((NULL == handle) || (NULL == trans))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config  = (UART_Config *) handle;
        object     = config->object;
        attrs   = config->attrs;
        DebugP_assert(NULL != object);

        if(object->isOpen == TRUE)
        {
            status = UART_checkTransaction(object, trans);
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        /* When User Managed Interrupt is set, user need to manage the read/write operation */
        if(UART_CONFIG_MODE_USER_INTR == object->prms.transferMode)
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {

        key = HwiP_disable();

        /* Check if any transaction is in progress */
        if(NULL != object->readTrans)
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_INUSE;
            status = SystemP_FAILURE;
        }
        else
        {
            /* Initialize transaction params */
            object->readTrans           = trans;
            object->readBuf             = trans->buf;
            object->readSizeRemaining   = trans->count;
            object->readTrans->timeout  = trans->timeout;
            object->readCount           = 0U;
            object->rxTimeoutCnt        = 0U;
            object->readErrorCnt        = 0U;
        }

        HwiP_restore(key);

        if(SystemP_SUCCESS == status)
        {
            /* Interrupt mode */
            if(UART_CONFIG_MODE_INTERRUPT == object->prms.transferMode)
            {
                status = UART_readInterrupt(config, object, attrs, trans);
            }
            else if(UART_CONFIG_MODE_POLLED == object->prms.transferMode)
            {
                /* Polled mode */
                status = UART_readPolling(config, object, attrs, trans);
            }
            else if(UART_CONFIG_MODE_DMA == object->prms.transferMode)
            {
                /* DMA mode */
                status = UART_readDma(config, object, attrs, trans);
            }
        }
    }

    return (status);
}

int32_t UART_writeCancel(UART_Handle handle, UART_Transaction *trans)
{
    int32_t             status = SystemP_SUCCESS;
    UART_Config        *config;
    UART_Object        *object;
    const UART_Attrs   *attrs;

    /* Check parameters */
    if((NULL == handle) || (NULL == trans))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config = (UART_Config *) handle;
        object = config->object;
        attrs = config->attrs;
        DebugP_assert(NULL != object);

        if(object->isOpen == TRUE)
        {
            status = UART_checkTransaction(object, trans);
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        /* When User Managed Interrupt is set, user need to manage the read/write operation */
        if(UART_CONFIG_MODE_USER_INTR == object->prms.transferMode)
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        if(UART_writeCancelNoCB(object, attrs) == (Bool)TRUE)
        {
            object->writeTrans->status = UART_TRANSFER_STATUS_CANCELLED;
            /*
            * Post transfer Sem in case of bloacking transfer.
            * Call the callback function in case of Callback mode.
            */
            if(object->prms.writeMode == UART_TRANSFER_MODE_CALLBACK)
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
    const UART_Attrs   *attrs;

    /* Check parameters */
    if((NULL == handle) || (NULL == trans))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config = (UART_Config *) handle;
        object = config->object;
        attrs = config->attrs;
        DebugP_assert(NULL != object);

        if(object->isOpen == TRUE)
        {
            status = UART_checkTransaction(object, trans);
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        /* When User Managed Interrupt is set, user need to manage the read/write operation */
        if(UART_CONFIG_MODE_USER_INTR == object->prms.transferMode)
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        if(UART_readCancelNoCB(object, attrs) == (Bool)TRUE)
        {
            object->readTrans->status = UART_TRANSFER_STATUS_CANCELLED;
            if(object->prms.readMode == UART_TRANSFER_MODE_CALLBACK)
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
    UART_Object        *object;
    uint32_t            isTxEmpty, startTicks, elapsedTicks;
    uint32_t            timeout = UART_TRANSMITEMPTY_TRIALCOUNT;
    uint32_t            timeoutElapsed  = FALSE;

    config = (UART_Config *) handle;

    if ((NULL != config) &&
       (config->object != NULL) &&
       (config->object->isOpen != (uint32_t)FALSE))
    {
        object = config->object;
        DebugP_assert(NULL != object);

        /* Update current tick value to perform timeout operation */
        startTicks = ClockP_getTicks();
        while(FALSE == timeoutElapsed)
        {
            /* Get TX status */
            isTxEmpty = CSL_FEXT(object->pSCIRegs->SCIFLR, SCI_SCIFLR_TX_EMPTY);
            if((uint32_t) TRUE == isTxEmpty)
            {
                /* TX is empty */
                break;
            }

            /* Check whether timeout happened or not */
            elapsedTicks = ClockP_getTicks() - startTicks;
            if(elapsedTicks >= timeout)
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

static void UART_masterIsr(void *arg)
{
    UART_Config        *config;
    UART_Object        *object;
    uint32_t            intrStatus, regVal;

    if(NULL != arg)
    {
        config = (UART_Config *) arg;
        object = (UART_Object *) config->object;
        DebugP_assert(NULL != object);
        DebugP_assert(NULL != object->pSCIRegs);

        intrStatus = CSL_REG_RD(&object->pSCIRegs->SCIFLR);
        /* Handle only when Rx interrupts are enabled */
        if(UART_isRxIntrEnabled(object->pSCIRegs))
        {
            /* Is there a Rx Interrupt? */
            if(intrStatus & CSL_SCI_SCIFLR_RXRDY_MASK)
            {
                /* Do we have valid data buffer to place the data? */
                if(object->readSizeRemaining > 0U)
                {
                    UART_rxChar(object);

                    /* Are we done with the read buffer? */
                    if(object->readSizeRemaining == 0U)
                    {
                        /* Disable RX interrupt until we do a new read */
                        UART_rxIntrDisable(object->pSCIRegs);

                        /* Update the driver internal status */
                        /* Reset the read buffer so we can pass it back */
                        object->readBuf =
                            (uint8_t *)object->readBuf - object->readCount;
                        if(object->readTrans != NULL)
                        {
                            object->readTrans->count = (uint32_t)(object->readCount);
                            object->readTrans->status = UART_TRANSFER_STATUS_SUCCESS;
                        }

                        /*
                         * Post transfer Sem in case of bloacking transfer.
                         * Call the callback function in case of Callback mode.
                         */
                        if(object->prms.readMode == UART_TRANSFER_MODE_CALLBACK)
                        {
                            object->prms.readCallbackFxn((UART_Handle) config, object->readTrans);
                        }
                        else
                        {
                            (void)SemaphoreP_post(&object->readTransferSemObj);
                        }
                        object->readTrans = NULL;
                    }
                }
                else
                {
                    /* Dummy read and drop the received character */
                    CSL_FEXT(object->pSCIRegs->SCIRD, SCI_SCIRD_RD);
                }
            }
        }

        /* Handle only when TX interrupts are enabled */
        if(UART_isTxIntrEnabled(object->pSCIRegs))
        {
            /* Is there a Tx Interrupt? */
            if(intrStatus & CSL_SCI_SCIFLR_TXRDY_MASK)
            {
                /*Is there any data which needs to be written? */
                if(object->writeSizeRemaining > 0U)
                {
                    UART_txChar(object);

                    /* Are we done with the write buffer? */
                    if(object->writeSizeRemaining == 0U)
                    {
                        /* Disable TX interrupt until we do a new write */
                        UART_txIntrDisable(object->pSCIRegs);

                        /* Reset the write buffer so we can pass it back */
                        object->writeBuf = (const uint8_t *)object->writeBuf - object->writeCount;
                        if(object->writeTrans != NULL)
                        {
                            object->writeTrans->count = (uint32_t)(object->writeCount);
                            object->writeTrans->status = UART_TRANSFER_STATUS_SUCCESS;
                        }

                        /*
                        * Post transfer Sem in case of bloacking transfer.
                        * Call the callback function in case of Callback mode.
                        */
                        if(object->prms.writeMode == UART_TRANSFER_MODE_CALLBACK)
                        {
                             object->prms.writeCallbackFxn((UART_Handle) config, object->writeTrans);
                        }
                        else
                        {
                            (void)SemaphoreP_post(&object->writeTransferSemObj);
                        }
                        object->writeTrans = NULL;
                    }
                }
                else
                {
                    /* Disable TX interrupt */
                    UART_txIntrDisable(object->pSCIRegs);
                }
            }
        }

        /* Check for errors */
        if((intrStatus & CSL_SCI_SCIFLR_OE_MASK) ||
           (intrStatus & CSL_SCI_SCIFLR_FE_MASK) ||
           (intrStatus & CSL_SCI_SCIFLR_PE_MASK))
        {
            if(object->readTrans != NULL)
            {
                object->readErrorCnt++;
                if(intrStatus & CSL_SCI_SCIFLR_OE_MASK)
                {
                    object->readTrans->status = UART_TRANSFER_STATUS_ERROR_OE;
                }
                if(intrStatus & CSL_SCI_SCIFLR_FE_MASK)
                {
                    object->readTrans->status = UART_TRANSFER_STATUS_ERROR_FE;
                }
                if(intrStatus & CSL_SCI_SCIFLR_PE_MASK)
                {
                    object->readTrans->status = UART_TRANSFER_STATUS_ERROR_PE;
                }
            }

            /* Clear only those errors that are set */
            regVal = intrStatus & (CSL_SCI_SCIFLR_OE_MASK | CSL_SCI_SCIFLR_FE_MASK | CSL_SCI_SCIFLR_PE_MASK);
            CSL_REG_WR(&object->pSCIRegs->SCIFLR, regVal);
        }
    }

    return;
}

static Bool UART_writeCancelNoCB(UART_Object *object, UART_Attrs const *attrs)
{
    uintptr_t           key;
    Bool                retVal = (Bool)TRUE;

    /* Disable Tx interrupt */
    DebugP_assert(NULL != object->pSCIRegs);
    UART_txIntrDisable(object->pSCIRegs);

    /* Disable interrupts to avoid writing data while changing state. */
    key = HwiP_disable();

    /* Return if there is no write. */
    if((object->writeSizeRemaining) == 0U)
    {
        retVal = (Bool)FALSE;
    }
    else
    {
        /* Reset the write buffer so we can pass it back */
        object->writeBuf = (const uint8_t *)object->writeBuf - object->writeCount;
        if(object->writeTrans != NULL)
        {
            object->writeTrans->count = (uint32_t)(object->writeCount);
        }

        /* Set size = 0 to prevent writing and restore interrupts. */
        object->writeSizeRemaining = 0;
    }

    HwiP_restore(key);

    return (retVal);
}

static Bool UART_readCancelNoCB(UART_Object *object, UART_Attrs const *attrs)
{
    uintptr_t           key;
    Bool                retVal = (Bool)TRUE;

    /* Disable Rx interrupt */
    DebugP_assert(NULL != object->pSCIRegs);
    UART_rxIntrDisable(object->pSCIRegs);

    /* Disable interrupts to avoid reading data while changing state. */
    key = HwiP_disable();

    if(object->readSizeRemaining == 0U)
    {
        retVal = (Bool)FALSE;
    }
    else
    {
        /* Reset the read buffer so we can pass it back */
        object->readBuf = (uint8_t *)object->readBuf - object->readCount;
        if(object->readTrans != NULL)
        {
            object->readTrans->count = object->readCount;
        }

        /* Set size = 0 to prevent reading and restore interrupts. */
        object->readSizeRemaining = 0;
    }

    HwiP_restore(key);

    return (retVal);
}

static int32_t UART_writePolling(UART_Object *object,
                                 UART_Attrs const *attrs,
                                 UART_Transaction *trans)
{
    uint32_t            timeout, startTicks, elapsedTicks;
    int32_t             retVal          = SystemP_SUCCESS;
    uint32_t            timeoutElapsed  = FALSE;

    DebugP_assert(NULL != object->pSCIRegs);

    timeout = trans->timeout;
    /* Update current tick value to perform timeout operation */
    startTicks = ClockP_getTicks();
    while((FALSE == timeoutElapsed) &&
          (0U != object->writeSizeRemaining))
    {
        /* Transfer data */
        if(UART_isTxRdy(object->pSCIRegs))
        {
            UART_txChar(object);
        }

        /* Check whether timeout happened or not */
        elapsedTicks = ClockP_getTicks() - startTicks;
        if(elapsedTicks >= timeout)
        {
            /* timeout occured */
            timeoutElapsed = TRUE;
        }
    }

    if(0U == object->writeSizeRemaining)
    {
        retVal             = SystemP_SUCCESS;
        trans->status      = UART_TRANSFER_STATUS_SUCCESS;
        object->writeTrans = NULL;
    }
    else
    {
        /* Return SystemP_TIMEOUT so that application gets whatever bytes are
         * transmitted. Set the trans status to timeout so that
         * application can handle the timeout. */
        retVal             = SystemP_TIMEOUT;
        trans->status      = UART_TRANSFER_STATUS_TIMEOUT;
        trans->count       = object->writeCount;
        object->writeTrans = NULL;
    }

    return (retVal);
}

static int32_t UART_writeInterrupt(UART_Object *object,
                                   UART_Attrs const *attrs,
                                   UART_Transaction *trans)
{
    int32_t     status = SystemP_SUCCESS, semStatus;

    DebugP_assert(NULL != object->pSCIRegs);

    /****************************************************************
    * Normal Mode: We always need to send out the first character
    * because the Transmit interrupt is only generated after the
    * first transfer from the TD to the TXSHF
    ****************************************************************/
    /* Wait for Tx Ready */
    while(UART_isTxRdy(object->pSCIRegs) == 0U)
    {
        TaskP_yield();
    }

    /* Send the first char */
    UART_txChar(object);

    /* Was it just one data to send? */
    if(object->writeSizeRemaining == 0U)
    {
        status             = SystemP_SUCCESS;
        trans->status      = UART_TRANSFER_STATUS_SUCCESS;
        object->writeTrans = NULL;
    }
    else
    {
        /* Enable Tx interrupt */
        UART_txIntrEnable(object->pSCIRegs);

        if(object->prms.writeMode == UART_TRANSFER_MODE_BLOCKING)
        {
             /* Block on transferSemObj until the transfer completion. */
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
                 /* Cancel the write without posting the semaphore */
                 trans->status = UART_TRANSFER_STATUS_TIMEOUT;
                 UART_writeCancelNoCB(object, attrs);
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
             trans->count = 0U;
             status = SystemP_SUCCESS;
         }
    }

    return status;
}

static int32_t UART_readPolling(UART_Config      *config,
                                UART_Object      *object,
                                UART_Attrs const *attrs,
                                UART_Transaction *trans)
{
    uint32_t            timeout, startTicks, elapsedTicks;
    int32_t             retVal          = SystemP_SUCCESS;
    uint32_t            timeoutElapsed  = FALSE;

    DebugP_assert(NULL != object->pSCIRegs);

    timeout = trans->timeout;
    /* Update current tick value to perform timeout operation */
    startTicks = ClockP_getTicks();
    while((FALSE == timeoutElapsed) &&
          (0U != object->readSizeRemaining))
    {
        /* Read data when ready */
        if(UART_isRxRdy(object->pSCIRegs))
        {
            UART_rxChar(object);
        }

        /* Check whether timeout happened or not */
        elapsedTicks = ClockP_getTicks() - startTicks;
        if(elapsedTicks >= timeout)
        {
            /* timeout occured */
            timeoutElapsed = TRUE;
        }
    }

    if((object->readSizeRemaining == 0U) &&
       (object->readErrorCnt == 0U) &&
       (object->rxTimeoutCnt == 0U))
    {
        retVal             = SystemP_SUCCESS;
        trans->status      = UART_TRANSFER_STATUS_SUCCESS;
        object->readTrans  = NULL;
    }
    else
    {
        /* Return SystemP_TIMEOUT so that application gets whatever bytes are
         * transmitted. Set the trans status to timeout so that
         * application can handle the timeout. */
        retVal             = SystemP_TIMEOUT;
        trans->status      = UART_TRANSFER_STATUS_TIMEOUT;
        trans->count       = object->readCount;
        object->readTrans  = NULL;
    }

    return (retVal);
}

static int32_t UART_readInterrupt(UART_Config    *config,
                                UART_Object      *object,
                                UART_Attrs const *attrs,
                                UART_Transaction *trans)
{
    int32_t             status = SystemP_SUCCESS, semStatus;

    /* Enable Rx interrupt */
    DebugP_assert(NULL != object->pSCIRegs);
    UART_rxIntrEnable(object->pSCIRegs);

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
            /* Cancel the read without posting the semaphore */
            trans->status = UART_TRANSFER_STATUS_TIMEOUT;
            UART_readCancelNoCB(object, attrs);
            status = SystemP_FAILURE;
        }
        trans->count = (uint32_t)(object->readCount);
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

    return status;
}

static void UART_configInstance(UART_Config *config)
{
    CSL_sciRegs            *pSCIRegs;
    const UART_Attrs       *attrs;
    UART_Params            *prms;
    UART_Object            *object;
    uint32_t                regVal;
    uint32_t                divisionFactor = 16U;

    DebugP_assert(NULL != config->attrs);
    DebugP_assert(NULL != config->object);

    attrs = config->attrs;
    object = config->object;
    pSCIRegs = (CSL_sciRegs *) config->attrs->baseAddr;
    prms = &config->object->prms;
    object->pSCIRegs = pSCIRegs;

    /* Enable SCI by setting the RESET bit to 1 */
    CSL_FINS(pSCIRegs->SCIGCR0, SCI_SCIGCR0_RESET, 1U);
    /* Clear the SWnRST bit to 0 before SCI is configured */
    CSL_FINS(pSCIRegs->SCIGCR1, SCI_SCIGCR1_SW_NRESET, 0U);

    /* Disable and clear interrupts */
    CSL_REG_WR(&pSCIRegs->SCICLEARINT, 0xFFFFFFFFU);
    CSL_REG_WR(&pSCIRegs->SCICLEARINTLVL, 0xFFFFFFFFU);

    /*
     * Configure SCI
     */
    /* TX enable */
    regVal = 0U;
    regVal |= CSL_FMK(SCI_SCIGCR1_TXENA, 1U);
    CSL_FINS(pSCIRegs->SCIPIO0, SCI_SCIPIO0_TX_FUNC, 1U);
    /* RX enable */
    regVal |= CSL_FMK(SCI_SCIGCR1_RXENA, 1U);
    CSL_FINS(pSCIRegs->SCIPIO0, SCI_SCIPIO0_RX_FUNC, 1U);
    /* Clock settings */
    regVal |= CSL_FMK(SCI_SCIGCR1_CLOCK, UARTSCI_CLOCK_MODE_INTERNAL);
    CSL_FINS(pSCIRegs->SCIPIO0, SCI_SCIPIO0_CLK_FUNC, UARTSCI_CLOCK_MODE_INTERNAL);
    /* Misc modes */
    regVal |= CSL_FMK(SCI_SCIGCR1_TIMING_MODE, UARTSCI_TIMING_MODE_ASYNC);
    regVal |= CSL_FMK(SCI_SCIGCR1_COMM_MODE, UARTSCI_COMM_MODE_IDLE);
    /* Stop bit */
    regVal |= CSL_FMK(SCI_SCIGCR1_STOP, prms->stopBits);
    /* Parity */
    if(prms->parityType == UART_PARITY_EVEN)
    {
        regVal |= CSL_FMK(SCI_SCIGCR1_PARITY_ENA, 1U);
        regVal |= CSL_FMK(SCI_SCIGCR1_PARITY, 1U);
    }
    else if(prms->parityType == UART_PARITY_ODD)
    {
        regVal |= CSL_FMK(SCI_SCIGCR1_PARITY_ENA, 1U);
        regVal |= CSL_FMK(SCI_SCIGCR1_PARITY, 0U);
    }
    else
    {
        regVal |= CSL_FMK(SCI_SCIGCR1_PARITY_ENA, 0U);
    }
    CSL_REG_WR(&pSCIRegs->SCIGCR1, regVal);
    /* Data length */
    CSL_REG_WR(&pSCIRegs->SCICHAR, CSL_FMK(SCI_SCICHAR_CHAR, prms->dataLength));
    object->shiftJustification = UART_LEN_8 - prms->dataLength; /* 8-bit: 0 shift, 7-bit: 1 shift etc... */
    /* Baud rate */
    regVal = (attrs->inputClkFreq / (prms->baudRate * divisionFactor)) - 1U;
    CSL_REG_WR(&pSCIRegs->SCIBAUD, regVal);

    /* Enable Error Interrupts: Framing Error, Overrun Error, Parity Error */
    regVal = 0U;
    regVal |= CSL_FMK(SCI_SCISETINT_SET_PE_INT, 1U);
    regVal |= CSL_FMK(SCI_SCISETINT_SET_OE_INT, 1U);
    regVal |= CSL_FMK(SCI_SCISETINT_SET_FE_INT, 1U);
    CSL_REG_WR(&pSCIRegs->SCISETINT, regVal);

    return;
}

static int32_t UART_checkOpenParams(const UART_Params *prms)
{
    int32_t     status = SystemP_SUCCESS;

    /* only NONE, ODD and EVEN parity are supported */
    if(prms->parityType > UART_PARITY_EVEN)
    {
        status = SystemP_FAILURE;
    }
    /* only 5/6/7/8-bit data length are supported  */
    if(prms->dataLength > UART_LEN_8)
    {
        status = SystemP_FAILURE;
    }
    /* Sanity Check: Ensure that the clock frequency is NON-Zero */
    if(prms->baudRate == 0U)
    {
        status = SystemP_FAILURE;
    }
    if((UART_TRANSFER_MODE_CALLBACK == prms->readMode) &&
       (NULL == prms->readCallbackFxn))
    {
        status = SystemP_FAILURE;
    }
    if((UART_TRANSFER_MODE_CALLBACK == prms->writeMode) &&
       (NULL == prms->writeCallbackFxn))
    {
        status = SystemP_FAILURE;
    }

    return (status);
}

static int32_t UART_checkTransaction(const UART_Object *object,
                                     UART_Transaction *trans)
{
    int32_t     status = SystemP_SUCCESS;

    if(0U == trans->count)
    {
        /* Transfer count should be positive */
        trans->status = UART_TRANSFER_STATUS_ERROR_OTH;
        status = SystemP_FAILURE;
    }
    if(NULL == trans->buf)
    {
        status = SystemP_FAILURE;
    }

    return (status);
}

static inline void UART_txChar(UART_Object *object)
{
    uint8_t txCh;

    txCh = *((uint8_t *) object->writeBuf);
    CSL_FINS(object->pSCIRegs->SCITD, SCI_SCITD_TD, txCh);
    object->writeSizeRemaining--;
    object->writeBuf = (uint8_t *) object->writeBuf + 1;
    object->writeCount++;

    return;
}

static inline void UART_rxChar(UART_Object *object)
{
    uint8_t rxCh;

    rxCh = (uint8_t) CSL_FEXT(object->pSCIRegs->SCIRD, SCI_SCIRD_RD);
    rxCh >>= object->shiftJustification;
    *((uint8_t *) object->readBuf) = rxCh;
    object->readSizeRemaining--;
    object->readBuf = (uint8_t *) object->readBuf + 1;
    object->readCount++;

    return;
}

static inline uint32_t UART_isTxRdy(const CSL_sciRegs *pSCIRegs)
{
    return CSL_FEXT(pSCIRegs->SCIFLR, SCI_SCIFLR_TXRDY);
}

static inline uint32_t UART_isRxRdy(const CSL_sciRegs *pSCIRegs)
{
    return CSL_FEXT(pSCIRegs->SCIFLR, SCI_SCIFLR_RXRDY);
}

static inline void UART_txIntrEnable(CSL_sciRegs *pSCIRegs)
{
    CSL_FINS(pSCIRegs->SCISETINT, SCI_SCISETINT_SET_TX_INT, 1U);
}

static inline void UART_rxIntrEnable(CSL_sciRegs *pSCIRegs)
{
    CSL_FINS(pSCIRegs->SCISETINT, SCI_SCISETINT_SET_RX_INT, 1U);
}

static inline void UART_txIntrDisable(CSL_sciRegs *pSCIRegs)
{
    CSL_REG_WR(&pSCIRegs->SCICLEARINT, CSL_FMK(SCI_SCICLEARINT_CLR_TX_INT, 1U));
}

static inline void UART_rxIntrDisable(CSL_sciRegs *pSCIRegs)
{
    CSL_REG_WR(&pSCIRegs->SCICLEARINT, CSL_FMK(SCI_SCICLEARINT_CLR_RX_INT, 1U));
}

static inline uint32_t UART_isTxIntrEnabled(const CSL_sciRegs* pSCIRegs)
{
    return CSL_FEXT(pSCIRegs->SCISETINT, SCI_SCISETINT_SET_TX_INT);
}

static inline uint32_t UART_isRxIntrEnabled(const CSL_sciRegs* pSCIRegs)
{
    return CSL_FEXT(pSCIRegs->SCISETINT, SCI_SCISETINT_SET_RX_INT);
}

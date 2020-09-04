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
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TaskP.h>
#include <drivers/uart/v0/dma/uart_dma.h>
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
static void UART_configInstance(UART_Config *config);
static void UART_resetModule(uint32_t baseAddr);
static int32_t UART_checkOpenParams(const UART_Params *prms);
static int32_t UART_checkTransaction(const UART_Object *object,
                                     UART_Transaction *trans);
static void UART_masterIsr(void *arg);
static inline void UART_procLineStatusErr(UART_Config *config);
static inline uint32_t UART_writeData(UART_Object *object,
                                      UART_Attrs const *attrs,
                                      uint32_t size);
static Bool UART_writeCancelNoCB(UART_Handle *handle,
                                 UART_Object *object, UART_Attrs const *attrs);
static int32_t UART_readInterrupt(UART_Config      *config,
                                UART_Object      *object,
                                UART_Attrs const *attrs,
                                UART_Transaction *trans);
static int32_t UART_writePolling(UART_Object *object,
                                 UART_Attrs const *attrs,
                                 UART_Transaction *trans);
static int32_t UART_writeInterrupt(UART_Object *object,
                                   UART_Attrs const *attrs,
                                   UART_Transaction *trans);
static void UART_writeDataPolling(UART_Object *object,
                                      UART_Attrs const *attrs);
static uint32_t UART_fifoWrite(const UART_Attrs *attrs,
                               const uint8_t    *buffer,
                               uint32_t          writeSizeRemaining);
static inline uint32_t UART_readData(UART_Object *object,
                                     UART_Attrs const *attrs,
                                     uint32_t size);
static int32_t UART_readPolling(UART_Config      *config,
                                UART_Object      *object,
                                UART_Attrs const *attrs,
                                UART_Transaction *trans);
static void UART_readDataPolling(UART_Config      *config,
                                 UART_Object      *object,
                                 UART_Attrs const *attrs);
static uint32_t UART_fifoRead(UART_Config      *config,
                              const UART_Attrs *attrs,
                              uint8_t    *buffer,
                              uint32_t          readSizeRemaining);
static inline uint8_t UART_readByte(UART_Config *config, const UART_Attrs *attrs);
static Bool UART_statusIsDataReady(UART_Config       *config,
                                  const UART_Attrs  *attrs);
static Bool UART_readCancelNoCB(UART_Handle *handle, UART_Object *object, UART_Attrs const *attrs);
/* Low level HW functions */
static uint32_t UART_enhanFuncEnable(uint32_t baseAddr);
static void UART_regConfModeRestore(uint32_t baseAddr, uint32_t lcrRegValue);
static void UART_modemControlReset(uint32_t baseAddr);
static uint32_t UART_operatingModeSelect(uint32_t baseAddr, uint32_t modeFlag);
static void UART_moduleReset(uint32_t baseAddr);
static uint32_t UART_subConfigTCRTLRModeEn(uint32_t baseAddr);
static void UART_enhanFuncBitValRestore(uint32_t baseAddr, uint32_t enhanFnBitVal);
static uint32_t UART_divisorLatchWrite(uint32_t baseAddr, uint32_t divisorValue);
static void UART_fifoRegisterWrite(uint32_t baseAddr, uint32_t fcrValue);
static void UART_tcrTlrBitValRestore(uint32_t baseAddr, uint32_t tcrTlrBitVal);
static uint32_t UART_fifoConfig(uint32_t baseAddr, uint32_t fifoConfig);
static inline uint32_t UART_divideRoundCloset(uint32_t divident, uint32_t divisor);
static uint32_t UART_divisorValCompute(uint32_t moduleClk,
                                      uint32_t baudRate,
                                      uint32_t modeFlag,
                                      uint32_t mirOverSampRate);
static void UART_lineCharConfig(uint32_t baseAddr,
                                 uint32_t wLenStbFlag,
                                 uint32_t parityFlag);
static void UART_divisorLatchDisable(uint32_t baseAddr);
static void UART_breakCtl(uint32_t baseAddr, uint32_t breakState);
static uint8_t UART_fifoCharGet(uint32_t baseAddr);
static void UART_hardwareFlowCtrlOptSet(uint32_t baseAddr, uint32_t hwFlowCtrl);
static void UART_flowCtrlTrigLvlConfig(uint32_t baseAddr,
                               uint32_t rtsHaltFlag,
                               uint32_t rtsStartFlag);
static uint32_t UART_spaceAvail(uint32_t baseAddr);
static uint32_t UART_getRxError(uint32_t baseAddr);
static uint32_t UART_regConfigModeEnable(uint32_t baseAddr, uint32_t modeFlag);
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
        object = config->object;
        DebugP_assert(NULL != object);
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

        /* If DMA is enabled, program DMA */
        if(UART_CONFIG_MODE_DMA == object->prms.transferMode)
        {
            object->uartDmaHandle = UART_dmaOpen((UART_Handle) config,
                                                 object->prms.uartDmaIndex);
        }
        else
        {
            object->uartDmaHandle = NULL;
        }

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
        if((UART_CONFIG_MODE_INTERRUPT == object->prms.transferMode) && (TRUE != object->prms.skipIntrReg))
        {
            DebugP_assert(object->prms.intrNum != 0xFFFF);
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

    if ((NULL != config) && (config->object != NULL) && (config->object->isOpen != (uint32_t)FALSE))
    {
        object = config->object;
        attrs = config->attrs;

        DebugP_assert(NULL != object);
        DebugP_assert(NULL != attrs);

        DebugP_assert(NULL != gUartDrvObj.lock);
        SemaphoreP_pend(&gUartDrvObj.lockObj, SystemP_WAIT_FOREVER);

        /* Flush TX FIFO */
        UART_flushTxFifo(handle);

        /* Disable UART and interrupts. */
        UART_intrDisable(attrs->baseAddr,
                       UART_INTR_RHR_CTI | UART_INTR_THR | UART_INTR_LINE_STAT);
        UART_intr2Disable(attrs->baseAddr, UART_INT2_TX_EMPTY);
        (void)UART_operatingModeSelect(attrs->baseAddr, UART_OPER_MODE_DISABLED);

        if(UART_CONFIG_MODE_DMA == object->prms.transferMode)
        {
            UART_dmaClose(handle);
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
    int32_t             status = SystemP_SUCCESS, semStatus = SystemP_SUCCESS;
    UART_Config        *config;
    UART_Object        *object;
    const UART_Attrs   *attrs;
    UART_Params        *prms;
    uintptr_t           key;

    /* Check parameters */
    if ((NULL == handle) || (NULL == trans))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config  = (UART_Config *) handle;
        object  = config->object;
        attrs   = config->attrs;
        prms    = &config->object->prms;

        DebugP_assert(NULL != object);
        DebugP_assert(NULL != attrs);

        if (object->isOpen == TRUE)
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
        if (TRUE == prms->skipIntrReg)
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
            if ((UART_CONFIG_MODE_INTERRUPT == prms->transferMode) ||
                (UART_CONFIG_MODE_DMA == prms->transferMode))
            {
                if (UART_CONFIG_MODE_INTERRUPT == prms->transferMode)
                {
                    status = UART_writeInterrupt(object, attrs, trans);
                }
                else
                {
                    status = UART_writeInterruptDma(object, attrs, trans);
                }
                if ((SystemP_SUCCESS == status) &&
                    (object->prms.writeMode == UART_TRANSFER_MODE_BLOCKING))
                {
                     /* Block on transferSemObj until the transfer completion. */
                     semStatus = SemaphoreP_pend(&object->writeTransferSemObj, trans->timeout);
                     if (semStatus == SystemP_SUCCESS)
                     {
                         if (trans->status == UART_TRANSFER_STATUS_SUCCESS)
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
                         trans->status = UART_TRANSFER_STATUS_TIMEOUT;
                         /* Cancel the DMA without posting the semaphore */
                         UART_writeCancelNoCB(&handle, object, attrs);
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
            else
            {
                /* Polled mode */
                status = UART_writePolling(object, attrs, trans);
            }
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
        prms    = &config->object->prms;
        DebugP_assert(NULL != object);

        if (object->isOpen == TRUE)
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
        if (TRUE == prms->skipIntrReg)
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
            object->readTrans->timeout  = trans->timeout;
            object->readCount           = 0U;
            object->rxTimeoutCnt        = 0U;
            object->readErrorCnt        = 0U;
        }

        HwiP_restore(key);

        if(SystemP_SUCCESS == status)
        {
            /* Interrupt mode */
            if ((UART_CONFIG_MODE_INTERRUPT == prms->transferMode) ||
                (UART_CONFIG_MODE_DMA == prms->transferMode))
            {
                if (UART_CONFIG_MODE_INTERRUPT == prms->transferMode)
                {
                    status = UART_readInterrupt(config, object, attrs, trans);
                }
                else
                {
                    status = UART_readInterruptDma(object, attrs, trans);
                }
                if ((SystemP_SUCCESS == status) &&
                    (object->prms.readMode == UART_TRANSFER_MODE_BLOCKING))
                {
                    /* Pend on lock and wait for Hwi to finish. */
                    semStatus = SemaphoreP_pend(&object->readTransferSemObj, trans->timeout);
                    if (semStatus == SystemP_SUCCESS)
                    {
                        if (trans->status == UART_TRANSFER_STATUS_SUCCESS)
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
                        trans->status = UART_TRANSFER_STATUS_TIMEOUT;
                        /* Cancel the DMA without posting the semaphore */
                        UART_readCancelNoCB(&handle, object, attrs);
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
            }
            else
            {
                /* Polled mode */
                status = UART_readPolling(config, object, attrs, trans);
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
    UART_Params        *prms;

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
        prms    = &config->object->prms;
        DebugP_assert(NULL != object);

        /* When User Managed Interrupt is set, user need to manage the read/write operation */
        if (TRUE == prms->skipIntrReg)
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        if (UART_writeCancelNoCB(&handle, object, attrs) == (Bool)TRUE)
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
    const UART_Attrs   *attrs;
    UART_Params        *prms;

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
        prms    = &config->object->prms;
        DebugP_assert(NULL != object);

        /* When User Managed Interrupt is set, user need to manage the read/write operation */
        if (TRUE == prms->skipIntrReg)
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        if (UART_readCancelNoCB(&handle, object, attrs) == (Bool)TRUE)
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
        DebugP_assert(NULL != attrs);

        /* Update current tick value to perform timeout operation */
        startTicks = ClockP_getTicks();
        while (FALSE == timeoutElapsed)
        {
            /* Get TX FIFO status */
            isTxFifoEmpty = UART_spaceAvail(attrs->baseAddr);
            if ((uint32_t) TRUE == isTxFifoEmpty)
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

static int32_t UART_readInterrupt(UART_Config    *config,
                                UART_Object      *object,
                                UART_Attrs const *attrs,
                                UART_Transaction *trans)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            baseAddr;

    baseAddr = attrs->baseAddr;

    /* Disable RX threshold and line status error interrupt */
    UART_intrDisable(baseAddr, UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);

    object->readSizeRemaining = UART_readData(object, attrs, trans->count);
    if ((object->readSizeRemaining) == 0U)
    {
        /* Update the actual read count */
        trans->count = (uint32_t)(object->readCount);
        trans->status = UART_TRANSFER_STATUS_SUCCESS;

        if (object->prms.readMode == UART_TRANSFER_MODE_CALLBACK)
        {
            object->prms.readCallbackFxn((UART_Handle) config, object->readTrans);
        }
        status = SystemP_SUCCESS;
        object->readTrans = NULL;
    }
    else
    {
        /* Enable Rx, read time-out and RX Line Error interrupt */
        UART_intrEnable(baseAddr, (uint32_t) UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);
    }

    return status;
}

static Bool UART_writeCancelNoCB(UART_Handle *handle, UART_Object *object, UART_Attrs const *attrs)
{
    uintptr_t           key;
    Bool                retVal = (Bool)TRUE;

    UART_intrDisable(attrs->baseAddr, UART_INTR_THR);

    /* Disable interrupts to avoid writing data while changing state. */
    key = HwiP_disable();

    /* Return if there is no write. */
    if ((object->writeSizeRemaining) == 0U)
    {
        retVal = (Bool)FALSE;
    }
    else
    {
        if (object->prms.transferMode == UART_CONFIG_MODE_DMA)
        {
            /* Disable DMA TX channel */
            UART_dmaDisableChannel(handle, (Bool)TRUE);
            if (object->writeTrans != NULL)
            {
                object->writeTrans->count = 0;
            }
            else
            {
                object->writeCount = 0;
            }
        }
        else
        {
            /* Reset the write buffer so we can pass it back */
            object->writeBuf = (const uint8_t *)object->writeBuf - object->writeCount;
            if (object->writeTrans != NULL)
            {
                object->writeTrans->count = (uint32_t)(object->writeCount);
            }

            /* Set size = 0 to prevent writing and restore interrupts. */
            object->writeSizeRemaining = 0;
        }
    }

    HwiP_restore(key);

    return (retVal);
}

static Bool UART_readCancelNoCB(UART_Handle *handle, UART_Object *object, UART_Attrs const *attrs)
{
    uintptr_t           key;
    Bool                retVal = (Bool)TRUE;
    uint8_t             rdData;
    uint32_t            flag;

    UART_intrDisable(attrs->baseAddr,
                   UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);

    /* Disable interrupts to avoid reading data while changing state. */
    key = HwiP_disable();
    if (object->readSizeRemaining == 0U)
    {
        retVal = (Bool)FALSE;
    }
    else
    {
        if (object->prms.transferMode == UART_CONFIG_MODE_DMA)
        {
            /* Disable DMA TX channel */
            UART_dmaDisableChannel(handle, (Bool)FALSE);
            if (object->readTrans != NULL)
            {
                object->readTrans->count = 0;
            }
            else
            {
                object->readCount = 0;
            }
        }
        else
        {
            /* Reset the read buffer so we can pass it back */
            object->readBuf = (uint8_t *)object->readBuf - object->readCount;
            if (object->readTrans != NULL)
            {
                object->readTrans->count = object->readCount;
            }

            /* Set size = 0 to prevent reading and restore interrupts. */
            object->readSizeRemaining = 0;

            /* Flush the RX FIFO */
            do
            {
                flag = UART_getChar(attrs->baseAddr, &rdData);
            }
            while (flag != FALSE);
        }
    }

    HwiP_restore(key);
    return (retVal);
}

static void UART_configInstance(UART_Config *config)
{
    uint32_t                baseAddr;
    const UART_Attrs       *attrs;
    UART_Params            *prms;
    UART_Object            *object;
    uint32_t                regVal, divisorVal, wLenStbFlag, parityFlag;

    DebugP_assert(NULL != config->attrs);
    DebugP_assert(NULL != config->object);

    attrs = config->attrs;
    object = config->object;
    baseAddr = config->attrs->baseAddr;
    prms = &config->object->prms;

    /* Reset module */
    UART_resetModule(baseAddr);

    /* Set up the TX and RX FIFO Trigger levels. */
    if(UART_CONFIG_MODE_DMA == prms->transferMode)
    {
        regVal = UART_FIFO_CONFIG(UART_TRIG_LVL_GRANULARITY_1,
                                  UART_TRIG_LVL_GRANULARITY_1,
                                  prms->txTrigLvl,
                                  prms->rxTrigLvl,
                                  1U,
                                  1U,
                                  UART_DMA_EN_PATH_FCR,
                                  UART_DMA_MODE_1_ENABLE);
        /* Configuring the FIFO settings. */
        UART_fifoConfig(baseAddr, regVal);
    }
    else
    {
        regVal = UART_FIFO_CONFIG(UART_TRIG_LVL_GRANULARITY_1,
                                  UART_TRIG_LVL_GRANULARITY_1,
                                  prms->txTrigLvl,
                                  prms->rxTrigLvl,
                                  1U,
                                  1U,
                                  UART_DMA_EN_PATH_FCR,
                                  UART_DMA_MODE_0_ENABLE);

        /* Configuring the FIFO settings. */
        UART_fifoConfig(baseAddr, regVal);
    }

    /* Computing the Divisor Value for params.baudRate */
    divisorVal = UART_divisorValCompute(attrs->inputClkFreq,
                                       prms->baudRate,
                                       prms->operMode,
                                       UART_MIR_OVERSAMPLING_RATE_42);
    /* Configuring the Baud Rate settings. */
    UART_divisorLatchWrite(baseAddr, divisorVal);

    /* Switching to Configuration Mode B. */
    UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Programming the Line Characteristics */
    wLenStbFlag = (prms->dataLength << UART_LCR_CHAR_LENGTH_SHIFT);
    wLenStbFlag |= (prms->stopBits << UART_LCR_NB_STOP_SHIFT);
    parityFlag = (prms->parityType << UART_LCR_PARITY_EN_SHIFT);
    UART_lineCharConfig(baseAddr, wLenStbFlag, parityFlag);

    /* Disable write access to Divisor Latches. */
    UART_divisorLatchDisable(baseAddr);

    /* Disabling Break Control. */
    UART_breakCtl(baseAddr, UART_BREAK_COND_DISABLE);

    /* Set UART operating mode */
    UART_operatingModeSelect(baseAddr, prms->operMode);

    if (object->prms.hwFlowControl == (uint32_t)TRUE)
    {

        UART_hardwareFlowCtrlOptSet(baseAddr, UART_RTS_CTS_ENABLE);
        /* In case of HW flow control, the programmer must ensure that the
        trigger level to halt transmission is greater than or equal to the
        RX FIFO trigger level */
        if (object->prms.hwFlowControlThr >= object->prms.rxTrigLvl)
        {
            UART_flowCtrlTrigLvlConfig(baseAddr,
                                      object->prms.hwFlowControlThr,
                                      object->prms.rxTrigLvl);
        }
    }
    else
    {
        UART_hardwareFlowCtrlOptSet(baseAddr, UART_NO_HARDWARE_FLOW_CONTROL);
    }

    return;
}

static void UART_resetModule(uint32_t baseAddr)
{
    /* Switch to mode B to access EFR */
    /* Set the ENHANCEDEN Bit Field to Enable access to the MCR & IER reg
     * Setting the EFR[4] bit to 1 */
    UART_enhanFuncEnable(baseAddr);
    /* Force LCR[6] to zero, to avoid UART breaks and LCR[7] to zero to access
     * MCR reg */
    UART_regConfModeRestore(baseAddr, 0x00U);
    /* RESET MCR Reg */
    UART_modemControlReset(baseAddr);

    /* Disable all interrupts */
    UART_intrDisable(baseAddr, 0xFFU);
    UART_intr2Disable(baseAddr, UART_INT2_TX_EMPTY);

    /* Put the module in Disable State */
    UART_operatingModeSelect(baseAddr, UART_OPER_MODE_DISABLED);

    /* Reset Uart and setup hardware params */
    UART_moduleReset(baseAddr);

    return;
}

static int32_t UART_checkOpenParams(const UART_Params *prms)
{
    int32_t     status = SystemP_SUCCESS;

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

static uint32_t UART_enhanFuncEnable(uint32_t baseAddr)
{
    uint32_t enhanFnBitVal;
    uint32_t lcrRegValue;

    /* Enabling Configuration Mode B of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Collecting the current value of ENHANCEDEN bit of EFR. */
    enhanFnBitVal = HW_RD_REG32(baseAddr + UART_EFR) & UART_EFR_ENHANCED_EN_MASK;

    /* Setting the ENHANCEDEN bit in EFR register. */
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                  UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

    /* Programming LCR with the collected value. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return enhanFnBitVal;
}

static void UART_regConfModeRestore(uint32_t baseAddr, uint32_t lcrRegValue)
{
    /* Programming the Line Control Register(LCR). */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
}

static void UART_modemControlReset(uint32_t baseAddr)
{
    uint32_t mcrResetVal = 0U;
    /* Resetting bits of MCR. */
    HW_WR_REG32(baseAddr + UART_MCR, mcrResetVal);
}

static uint32_t UART_operatingModeSelect(uint32_t baseAddr, uint32_t modeFlag)
{
    uint32_t operMode;

    operMode = HW_RD_REG32(baseAddr + UART_MDR1) & UART_MDR1_MODE_SELECT_MASK;

    /* Programming the MODESELECT field in MDR1. */
    HW_WR_FIELD32(baseAddr + UART_MDR1, UART_MDR1_MODE_SELECT,
                  modeFlag >> UART_MDR1_MODE_SELECT_SHIFT);

    return operMode;
}

static void UART_moduleReset(uint32_t baseAddr)
{
    /* Performing Software Reset of the module. */
    HW_WR_FIELD32(baseAddr + UART_SYSC, UART_SYSC_SOFTRESET,
                  UART_SYSC_SOFTRESET_SOFTRESET_VALUE_1);

    /* Wait until the process of Module Reset is complete. */
    while (0U == HW_RD_FIELD32(baseAddr + UART_SYSS, UART_SYSS_RESETDONE))
    {
        /* Do nothing - Busy wait */
    }
}

static uint32_t UART_subConfigTCRTLRModeEn(uint32_t baseAddr)
{
    uint32_t enhanFnBitVal;
    uint32_t tcrTlrValue;
    uint32_t lcrRegValue;

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Collecting the current value of EFR[4] and later setting it. */
    enhanFnBitVal = HW_RD_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN);

    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                  UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Configuration Mode A. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_A);

    /* Collecting the bit value of MCR[6]. */
    tcrTlrValue = HW_RD_REG32(baseAddr + UART_MCR) & UART_MCR_TCR_TLR_MASK;

    /* Setting the TCRTLR bit in Modem Control Register(MCR). */
    HW_WR_FIELD32(baseAddr + UART_MCR, UART_MCR_TCR_TLR,
                  UART_MCR_TCR_TLR_TCR_TLR_VALUE_1);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Restoring the value of EFR[4] to its original value. */
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN, enhanFnBitVal);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return tcrTlrValue;
}

static void UART_enhanFuncBitValRestore(uint32_t baseAddr, uint32_t enhanFnBitVal)
{
    uint32_t lcrRegValue;

    /* Enabling Configuration Mode B of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Restoring the value of EFR[4]. */
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                  enhanFnBitVal >> UART_EFR_ENHANCED_EN_SHIFT);

    /* Programming LCR with the collected value. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
}

static uint32_t UART_divisorLatchWrite(uint32_t baseAddr, uint32_t divisorValue)
{
    volatile uint32_t enhanFnBitVal;
    volatile uint32_t sleepMdBitVal;
    volatile uint32_t lcrRegValue;
    volatile uint32_t operMode;
    uint32_t          divRegVal;

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Collecting the current value of EFR[4] and later setting it. */
    enhanFnBitVal = HW_RD_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN);
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                  UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Operational Mode. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_OPERATIONAL_MODE);

    /*
    ** Collecting the current value of IER[4](SLEEPMODE bit) and later
    ** clearing it.
    */
    sleepMdBitVal = HW_RD_FIELD32(baseAddr + UART_IER, UART_IER_SLEEP_MODE);

    HW_WR_FIELD32(baseAddr + UART_IER, UART_IER_SLEEP_MODE, 0U);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Collecting the current value of Divisor Latch Registers. */
    divRegVal  = HW_RD_REG32(baseAddr + UART_DLL) & 0xFFU;
    divRegVal |= (HW_RD_REG32(baseAddr + UART_DLH) & 0x3FU) << 8;

    /* Switch the UART instance to Disabled state. */
    operMode = UART_operatingModeSelect(baseAddr,
                                       (uint32_t) UART_MDR1_MODE_SELECT_MASK);

    /* Writing to Divisor Latch Low(DLL) register. */
    HW_WR_REG32(baseAddr + UART_DLL, divisorValue & 0x00FFU);

    /* Writing to Divisor Latch High(DLH) register. */
    HW_WR_REG32(baseAddr + UART_DLH, (divisorValue & 0x3F00U) >> 8);

    /* Restoring the Operating Mode of UART. */
    (void) UART_operatingModeSelect(baseAddr, operMode);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Operational Mode. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_OPERATIONAL_MODE);

    /* Restoring the value of IER[4] to its original value. */
    HW_WR_FIELD32(baseAddr + UART_IER, UART_IER_SLEEP_MODE, sleepMdBitVal);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Restoring the value of EFR[4] to its original value. */
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN, enhanFnBitVal);

    /* Restoring the value of LCR Register. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return divRegVal;
}

static void UART_fifoRegisterWrite(uint32_t baseAddr, uint32_t fcrValue)
{
    uint32_t divLatchRegVal;
    uint32_t enhanFnBitVal;
    uint32_t lcrRegValue;

    /* Switching to Register Configuration Mode A of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_A);

    /* Clearing the contents of Divisor Latch Registers. */
    divLatchRegVal = UART_divisorLatchWrite(baseAddr, 0x0000U);

    /* Set the EFR[4] bit to 1. */
    enhanFnBitVal = UART_enhanFuncEnable(baseAddr);

    /* Writing the 'fcrValue' to the FCR register. */
    HW_WR_REG32(baseAddr + UART_FCR, fcrValue);

    /* Restoring the value of EFR[4] to its original value. */
    UART_enhanFuncBitValRestore(baseAddr, enhanFnBitVal);

    /* Programming the Divisor Latch Registers with the collected value. */
    (void) UART_divisorLatchWrite(baseAddr, divLatchRegVal);

    /* Reinstating LCR with its original value. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
}

static void UART_tcrTlrBitValRestore(uint32_t baseAddr, uint32_t tcrTlrBitVal)
{
    uint32_t enhanFnBitVal;
    uint32_t lcrRegValue;

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Collecting the current value of EFR[4] and later setting it. */
    enhanFnBitVal = HW_RD_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN);

    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                  UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Configuration Mode A of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_A);

    /* Programming MCR[6] with the corresponding bit value in 'tcrTlrBitVal'. */
    HW_WR_FIELD32(baseAddr + UART_MCR, UART_MCR_TCR_TLR, tcrTlrBitVal);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Restoring the value of EFR[4] to its original value. */
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN, enhanFnBitVal);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
}

static uint32_t UART_fifoConfig(uint32_t baseAddr, uint32_t fifoConfig)
{
    uint32_t enhanFnBitVal;
    uint32_t tcrTlrBitVal;
    uint32_t tlrValue;
    uint32_t fcrValue = 0U;
    uint32_t txGra = (fifoConfig & UART_FIFO_CONFIG_TXGRA) >> 26;
    uint32_t rxGra = (fifoConfig & UART_FIFO_CONFIG_RXGRA) >> 22;
    uint32_t txTrig = (fifoConfig & UART_FIFO_CONFIG_TXTRIG) >> 14;
    uint32_t rxTrig = (fifoConfig & UART_FIFO_CONFIG_RXTRIG) >> 6;
    uint32_t txClr = (fifoConfig & UART_FIFO_CONFIG_TXCLR) >> 5;
    uint32_t rxClr = (fifoConfig & UART_FIFO_CONFIG_RXCLR) >> 4;

    uint32_t dmaEnPath = (fifoConfig & UART_FIFO_CONFIG_DMAENPATH) >> 3;
    uint32_t dmaMode   = (fifoConfig & UART_FIFO_CONFIG_DMAMODE);

    /* Setting the EFR[4] bit to 1. */
    enhanFnBitVal = UART_enhanFuncEnable(baseAddr);

    tcrTlrBitVal = UART_subConfigTCRTLRModeEn(baseAddr);

    /* Enable FIFO */
    fcrValue |= UART_FCR_FIFO_EN_MASK;

    /* Setting the Receiver FIFO trigger level. */
    if(UART_TRIG_LVL_GRANULARITY_1 != rxGra)
    {
        /* Clearing the RXTRIGGRANU1 bit in SCR. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_RX_TRIG_GRANU1,
                      UART_SCR_RX_TRIG_GRANU1_RX_TRIG_GRANU1_VALUE_0);

        /* Clearing the RX_FIFO_TRIG_DMA field of TLR register. */
        HW_WR_FIELD32(baseAddr + UART_TLR, UART_TLR_RX_FIFO_TRIG_DMA,
                      0U);

        fcrValue &= ~((uint32_t) UART_FCR_RX_FIFO_TRIG_MASK);

        /*
        ** Checking if 'rxTrig' matches with the RX Trigger level values
        ** in FCR.
        */
        if((UART_RXTRIGLVL_8 == rxTrig) ||
           (UART_RXTRIGLVL_16 == rxTrig) ||
           (UART_RXTRIGLVL_56 == rxTrig) ||
           (UART_RXTRIGLVL_60 == rxTrig))
        {
            fcrValue |= rxTrig & UART_FCR_RX_FIFO_TRIG_MASK;
        }
        else
        {
            /* RX Trigger level will be a multiple of 4. */
            /* Programming the RX_FIFO_TRIG_DMA field of TLR register. */
            HW_WR_FIELD32(baseAddr + UART_TLR, UART_TLR_RX_FIFO_TRIG_DMA,
                          rxTrig);
        }
    }
    else
    {
        /* 'rxTrig' now has the 6-bit RX Trigger level value. */

        rxTrig &= 0x003FU;

        /* Collecting the bits rxTrig[5:2]. */
        tlrValue = (rxTrig & 0x003CU) >> 2;

        /* Collecting the bits rxTrig[1:0] and writing to 'fcrValue'. */
        fcrValue |= (rxTrig & 0x0003U) << UART_FCR_RX_FIFO_TRIG_SHIFT;

        /* Setting the RXTRIGGRANU1 bit of SCR register. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_RX_TRIG_GRANU1,
                      UART_SCR_RX_TRIG_GRANU1_RX_TRIG_GRANU1_VALUE_1);

        /* Programming the RX_FIFO_TRIG_DMA field of TLR register. */
        HW_WR_FIELD32(baseAddr + UART_TLR, UART_TLR_RX_FIFO_TRIG_DMA, tlrValue);
    }

    /* Setting the Transmitter FIFO trigger level. */
    if(UART_TRIG_LVL_GRANULARITY_1 != txGra)
    {
        /* Clearing the TXTRIGGRANU1 bit in SCR. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_TX_TRIG_GRANU1,
                      UART_SCR_TX_TRIG_GRANU1_TX_TRIG_GRANU1_VALUE_0);

        /* Clearing the TX_FIFO_TRIG_DMA field of TLR register. */
        HW_WR_FIELD32(baseAddr + UART_TLR, UART_TLR_TX_FIFO_TRIG_DMA,
                      0U);

        fcrValue &= ~((uint32_t) UART_FCR_TX_FIFO_TRIG_MASK);

        /*
        ** Checking if 'txTrig' matches with the TX Trigger level values
        ** in FCR.
        */
        if((UART_TXTRIGLVL_8 == (txTrig)) ||
           (UART_TXTRIGLVL_16 == (txTrig)) ||
           (UART_TXTRIGLVL_32 == (txTrig)) ||
           (UART_TXTRIGLVL_56 == (txTrig)))
        {
            fcrValue |= txTrig & UART_FCR_TX_FIFO_TRIG_MASK;
        }
        else
        {
            /* TX Trigger level will be a multiple of 4. */
            /* Programming the TX_FIFO_TRIG_DMA field of TLR register. */
            HW_WR_FIELD32(baseAddr + UART_TLR, UART_TLR_TX_FIFO_TRIG_DMA,
                          txTrig);
        }
    }
    else
    {
        /* 'txTrig' now has the 6-bit TX Trigger level value. */

        txTrig &= 0x003FU;

        /* Collecting the bits txTrig[5:2]. */
        tlrValue = (txTrig & 0x003CU) >> 2;

        /* Collecting the bits txTrig[1:0] and writing to 'fcrValue'. */
        fcrValue |= (txTrig & 0x0003U) << UART_FCR_TX_FIFO_TRIG_SHIFT;

        /* Setting the TXTRIGGRANU1 bit of SCR register. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_TX_TRIG_GRANU1,
                      UART_SCR_TX_TRIG_GRANU1_TX_TRIG_GRANU1_VALUE_1);

        /* Programming the TX_FIFO_TRIG_DMA field of TLR register. */
        HW_WR_FIELD32(baseAddr + UART_TLR, UART_TLR_TX_FIFO_TRIG_DMA, tlrValue);
    }

    if(UART_DMA_EN_PATH_FCR == dmaEnPath)
    {
        /* Configuring the UART DMA Mode through FCR register. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_DMA_MODE_CTL,
                      UART_SCR_DMA_MODE_CTL_DMA_MODE_CTL_VALUE_0);

        dmaMode &= 0x1U;

        /* Clearing the bit corresponding to the DMA_MODE in 'fcrValue'. */
        fcrValue &= ~((uint32_t) UART_FCR_DMA_MODE_MASK);

        /* Setting the DMA Mode of operation. */
        fcrValue |= dmaMode << UART_FCR_DMA_MODE_SHIFT;
    }
    else
    {
        dmaMode &= 0x3U;

        /* Configuring the UART DMA Mode through SCR register. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_DMA_MODE_CTL,
                      UART_SCR_DMA_MODE_CTL_DMA_MODE_CTL_VALUE_1);

        /* Programming the DMAMODE2 field in SCR. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_DMA_MODE_2, dmaMode);
    }

    /* Programming the bits which clear the RX and TX FIFOs. */
    fcrValue |= rxClr << UART_FCR_RX_FIFO_CLEAR_SHIFT;
    fcrValue |= txClr << UART_FCR_TX_FIFO_CLEAR_SHIFT;

    /* Writing 'fcrValue' to the FIFO Control Register(FCR). */
    UART_fifoRegisterWrite(baseAddr, fcrValue);

    /* Restoring the value of TCRTLR bit in MCR. */
    UART_tcrTlrBitValRestore(baseAddr, tcrTlrBitVal);

    /* Restoring the value of EFR[4] to the original value. */
    UART_enhanFuncBitValRestore(baseAddr, enhanFnBitVal);

    return fcrValue;
}

static inline uint32_t UART_divideRoundCloset(uint32_t divident, uint32_t divisor)
{
    return ((divident + (divisor/2U))/divisor);
}

static uint32_t UART_divisorValCompute(uint32_t moduleClk,
                                      uint32_t baudRate,
                                      uint32_t modeFlag,
                                      uint32_t mirOverSampRate)
{
    uint32_t divisorValue = 0U;
    uint32_t tempModeFlag = modeFlag & UART_MDR1_MODE_SELECT_MASK;

    switch (tempModeFlag)
    {
        case UART_OPER_MODE_16X:
        case UART_OPER_MODE_SIR:
            divisorValue = UART_divideRoundCloset(moduleClk, 16U * baudRate);
            break;

        case UART_OPER_MODE_13X:
            divisorValue = UART_divideRoundCloset(moduleClk, 13U * baudRate);
            break;

        case UART_OPER_MODE_MIR:
            divisorValue = UART_divideRoundCloset(moduleClk, mirOverSampRate * baudRate);
            break;

        case UART_OPER_MODE_FIR:
            divisorValue = 0U;
            break;

        default:
            break;
    }

    return divisorValue;
}

static void UART_lineCharConfig(uint32_t baseAddr,
                                 uint32_t wLenStbFlag,
                                 uint32_t parityFlag)
{
    uint32_t lcrRegValue;

    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
    /* Clearing the CHAR_LENGTH and NB_STOP fields in LCR.*/
    lcrRegValue &= ~((uint32_t) UART_LCR_NB_STOP_MASK | (uint32_t) UART_LCR_CHAR_LENGTH_MASK);

    /* Programming the CHAR_LENGTH and NB_STOP fields in LCR. */
    lcrRegValue |= wLenStbFlag & (UART_LCR_NB_STOP_MASK |
                                  UART_LCR_CHAR_LENGTH_MASK);

    /* Clearing the PARITY_EN, PARITY_TYPE1 and PARITY_TYPE2 fields in LCR. */
    lcrRegValue &= ~((uint32_t) UART_LCR_PARITY_TYPE2_MASK |
                     (uint32_t) UART_LCR_PARITY_TYPE1_MASK |
                     (uint32_t) UART_LCR_PARITY_EN_MASK);

    /* Programming the PARITY_EN, PARITY_TYPE1 and PARITY_TYPE2 fields in LCR.*/
    lcrRegValue |= parityFlag & (UART_LCR_PARITY_TYPE2_MASK |
                                 UART_LCR_PARITY_TYPE1_MASK |
                                 UART_LCR_PARITY_EN_MASK);
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
}

static void UART_divisorLatchDisable(uint32_t baseAddr)
{
    /* Disabling access to Divisor Latch registers by clearing LCR[7] bit. */
    HW_WR_FIELD32(baseAddr + UART_LCR, UART_LCR_DIV_EN,
                  UART_LCR_DIV_EN_DIV_EN_VALUE_0);
}

static void UART_breakCtl(uint32_t baseAddr, uint32_t breakState)
{
    /* Programming the BREAK_EN bit in LCR. */
    HW_WR_FIELD32(baseAddr + UART_LCR, UART_LCR_BREAK_EN,
                  breakState >> UART_LCR_BREAK_EN_SHIFT);
}

static uint8_t UART_fifoCharGet(uint32_t baseAddr)
{
    uint32_t tempRetVal = 0U;
    tempRetVal = HW_RD_REG32(baseAddr + UART_RHR);
    return ((uint8_t) tempRetVal);
}

static void UART_hardwareFlowCtrlOptSet(uint32_t baseAddr, uint32_t hwFlowCtrl)
{
    uint32_t lcrRegValue = 0;

    /* Switching to Configuration Mode B of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Configuring the HWFLOWCONTROL field in EFR. */
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_HW_FLOW_CONTROL, hwFlowCtrl);

    /* Restoring LCR with the collected value. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
}

static void UART_flowCtrlTrigLvlConfig(uint32_t baseAddr,
                               uint32_t rtsHaltFlag,
                               uint32_t rtsStartFlag)
{
    uint32_t tcrValue = 0;

    tcrValue = rtsHaltFlag & UART_TCR_RX_FIFO_TRIG_HALT_MASK;

    tcrValue |= (rtsStartFlag <<
                 UART_TCR_RX_FIFO_TRIG_START_SHIFT) &
                UART_TCR_RX_FIFO_TRIG_START_MASK;

    /* Writing to TCR register. */
    HW_WR_REG32(baseAddr + UART_TCR, tcrValue);
}

static uint32_t UART_spaceAvail(uint32_t baseAddr)
{
    uint32_t lcrRegValue = 0;
    uint32_t retVal      = FALSE;

    /* Switching to Register Operational Mode of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_OPERATIONAL_MODE);

    /*
    ** Checking if either TXFIFOE or TXSRE bits of Line Status Register(LSR)
    ** are set. TXFIFOE bit is set if TX FIFO(or THR in non-FIFO mode) is
    ** empty. TXSRE is set if both the TX FIFO(or THR in non-FIFO mode) and
    ** the transmitter shift register are empty.
    */
    if ((UART_LSR_TX_SR_E_MASK | UART_LSR_TX_FIFO_E_MASK) ==
        (HW_RD_REG32(baseAddr + UART_LSR) &
            (UART_LSR_TX_SR_E_MASK | UART_LSR_TX_FIFO_E_MASK)))
    {
        retVal = (uint32_t) TRUE;
    }

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

static uint32_t UART_getRxError(uint32_t baseAddr)
{
    uint32_t lcrRegValue = 0;
    uint32_t retVal      = 0;

    /* Switching to Register Operational Mode of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_OPERATIONAL_MODE);

    retVal = HW_RD_REG32(baseAddr + UART_LSR) &
             (UART_LSR_RX_FIFO_STS_MASK |
              UART_LSR_RX_BI_MASK |
              UART_LSR_RX_FE_MASK |
              UART_LSR_RX_PE_MASK |
              UART_LSR_RX_OE_MASK);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

static uint32_t UART_regConfigModeEnable(uint32_t baseAddr, uint32_t modeFlag)
{
    uint32_t lcrRegValue;

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    switch (modeFlag)
    {
        case UART_REG_CONFIG_MODE_A:
        case UART_REG_CONFIG_MODE_B:
            HW_WR_REG32(baseAddr + UART_LCR, modeFlag & 0xFFU);
            break;

        case UART_REG_OPERATIONAL_MODE:
            HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                        & 0x7FU);
            break;

        default:
            break;
    }

    return lcrRegValue;
}

/*
 *  ======== UART_masterIsr ========
 *  Hwi function that processes UART interrupts.
 *
 *  In non-DMA mode, three UART interrupts are enabled:
 *    1. transmit FIFO is below the TX FIFO trigger level (THR)
 *    2. receive FIFO is above RX FIFO trigger level (RHR)
 *    3. line status rx error
 *
 *  ISR checks the three interrupts, to ensure that all
 *  the pending interrupts are handled.
 *
 *  If line status rx error is detected, ISR clears the last read error, update
 *  the actual number of bytes transferred and transfer status in readTrans and
 *  calls back to application in the callback mode or post the read semaphone
 *  in the blocking mode. ISR
 *
 *  If RHR interrupt is received, ISR calls the in-lined function readData to
 *  read the data out from the RX FIFO. When all the data are received, ISR updates
 *  the actual number of bytes transferred and transfer status in readTrans and
 *  calls back to the application in the callback mode or post the read semaphone
 *  in the blocking mode. if RX timeout is detected, ISR will log the timeout
 *  count.
 *
 *  If THR interrupt is received, ISR calls the in-lined function writeData,
 *  to write the data to the TX FIFO. After all the data are sent, TX FIFO empty
 *  interrupt is enabled, ISR will update the actual number of bytes transferred
 *  and transfer status in writeTrans and calls back to application in the
 *  callback mode or post the read semaphone in the blocking mode.
 *
 *
 *  @param(arg)         The UART_Handle for this Hwi.
 */
static void UART_masterIsr(void *arg)
{
    UART_Config        *config;
    UART_Object        *object;
    UART_Attrs const   *attrs;
    uint32_t            intType;
    uint8_t             rdData;
    int32_t             status = SystemP_SUCCESS;

    /* Check parameters */
    if(NULL == arg)
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config = (UART_Config*)arg;
        object = (UART_Object*)config->object;
        attrs  = (UART_Attrs const *)config->attrs;
        DebugP_assert(NULL != object);
        DebugP_assert(NULL != attrs);

        while ((Bool)TRUE)
        {
            intType = UART_getIntrIdentityStatus(attrs->baseAddr);

            if ((intType & UART_INTID_RX_THRES_REACH) == UART_INTID_RX_THRES_REACH)
            {
                if ((intType & UART_INTID_RX_LINE_STAT_ERROR) ==
                    UART_INTID_RX_LINE_STAT_ERROR)
                {
                    /* RX line status error */
                    UART_procLineStatusErr(config);
                }
                else
                {
                    if ((intType & UART_INTID_CHAR_TIMEOUT) == UART_INTID_CHAR_TIMEOUT)
                    {
                        /* Disable Interrupt first, to avoid further RX timeout */
                        UART_intrDisable(attrs->baseAddr, UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);
                        /* RX timeout, log the RX timeout errors */
                        object->rxTimeoutCnt++;
                    }
                    /* RX FIFO threshold reached */
                    if (object->readSizeRemaining > 0U)
                    {
                        object->readSizeRemaining = UART_readData(object, attrs, object->readSizeRemaining);
                        if ((object->readSizeRemaining == 0U) ||
                            (object->prms.readReturnMode == UART_READ_RETURN_MODE_PARTIAL))
                        {
                            /* transfer completed */
                            UART_intrDisable(attrs->baseAddr, UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);

                            /* Update the driver internal status. */
                            /* Reset the read buffer so we can pass it back */
                            object->readBuf =
                                (uint8_t *)object->readBuf - object->readCount;
                            if (object->readTrans != NULL)
                            {
                                object->readTrans->count = (uint32_t)(object->readCount);
                                object->readTrans->status = UART_TRANSFER_STATUS_SUCCESS;
                            }

                            /*
                            * Post transfer Sem in case of bloacking transfer.
                            * Call the callback function in case of Callback mode.
                            */
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
                            /* Enable Rx interrupt again in case of UART_READ_RETURN_MODE_FULL */
                            UART_intrEnable(attrs->baseAddr, (uint32_t) UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);
                        }
                    }
                    else
                    {
                        /* Disable the receive interrupt again. */
                        (void)UART_getChar(attrs->baseAddr, &rdData);
                        UART_intrDisable(attrs->baseAddr, (uint32_t) UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);
                    }
                }
            }
            else if ((intType & UART_INTID_TX_THRES_REACH) == UART_INTID_TX_THRES_REACH)
            {
                /* TX FIFO threshold reached */
                if (object->writeSizeRemaining > 0U)
                {
                    object->writeSizeRemaining = (size_t)UART_writeData(object, attrs, (object->writeSizeRemaining));
                    if ((object->writeSizeRemaining) == 0U)
                    {
                        UART_intrDisable(attrs->baseAddr, UART_INTR_THR);

                        /* Reset the write buffer so we can pass it back */
                        object->writeBuf = (const uint8_t *)object->writeBuf - object->writeCount;
                        if (object->writeTrans != NULL)
                        {
                            object->writeTrans->count = (uint32_t)(object->writeCount);
                            object->writeTrans->status = UART_TRANSFER_STATUS_SUCCESS;
                        }

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
                }
                else
                {
                    UART_intrDisable(attrs->baseAddr, UART_INTR_THR);
                }
            }
            else
            {
                break;
            }
        } /* while(TRUE) */
    }
}

static inline uint32_t UART_writeData(UART_Object *object,
                                      UART_Attrs const *attrs,
                                      uint32_t writeSizeRemaining)
{
    uint32_t numBytesToTransfer, numBytesToTransferred;

    /* In interrupt mode write only threshold level of data with FIFO enabled */
    numBytesToTransfer = writeSizeRemaining;
    if (numBytesToTransfer >= object->prms.txTrigLvl)
    {
        numBytesToTransfer = object->prms.txTrigLvl;
    }

    numBytesToTransferred = numBytesToTransfer;
    /* Send characters until FIFO threshold level or done. */
    while (numBytesToTransfer != 0U)
    {
        UART_putChar(attrs->baseAddr, *(const uint8_t *)object->writeBuf);
        object->writeBuf = (const uint8_t *)object->writeBuf + 1U;

        numBytesToTransfer--;
        object->writeCount++;
    }

    return (writeSizeRemaining - numBytesToTransferred);
}

static int32_t UART_writeInterrupt(UART_Object *object,
                                   UART_Attrs const *attrs,
                                   UART_Transaction *trans)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    baseAddr;

    baseAddr = attrs->baseAddr;

    /* Enable the transmit interrupt. */
    UART_intrEnable(baseAddr, UART_INTR_THR);

    return status;
}

static int32_t UART_writePolling(UART_Object *object,
                                 UART_Attrs const *attrs,
                                 UART_Transaction *trans)
{
    uint32_t            timeout, startTicks, elapsedTicks;
    int32_t             retVal          = SystemP_SUCCESS;
    uint32_t            timeoutElapsed  = FALSE;
    uint32_t            baseAddr        = attrs->baseAddr;
    uint32_t            lineStatus      = 0U;

    timeout = trans->timeout;
    object->writeSizeRemaining = trans->count;
    /* Update current tick value to perform timeout operation */
    startTicks = ClockP_getTicks();
    while ((FALSE == timeoutElapsed)
           && (0U != object->writeSizeRemaining))
    {
        /* Transfer DATA */
        UART_writeDataPolling(object, attrs);
        /* Check whether timeout happened or not */
        elapsedTicks = ClockP_getTicks() - startTicks;
        if (elapsedTicks >= timeout)
        {
            /* timeout occured */
            timeoutElapsed = TRUE;
        }
    }

    if (0U == object->writeSizeRemaining)
    {
        do
        {
            lineStatus = UART_readLineStatus(baseAddr);
        }
        while ((uint32_t) (UART_LSR_TX_FIFO_E_MASK |
                         UART_LSR_TX_SR_E_MASK) !=
               (lineStatus & (uint32_t) (UART_LSR_TX_FIFO_E_MASK |
                                       UART_LSR_TX_SR_E_MASK)));

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

static void UART_writeDataPolling(UART_Object *object, UART_Attrs const *attrs)
{
    uint32_t numBytesWritten = 0U;

    numBytesWritten = UART_fifoWrite(attrs,
                                     (const uint8_t *) object->writeBuf,
                                     object->writeSizeRemaining);

    object->writeSizeRemaining -= numBytesWritten;
    object->writeBuf           = (const uint8_t *) object->writeBuf + numBytesWritten;
    object->writeCount         += numBytesWritten;

    return;
}

static uint32_t UART_fifoWrite(const UART_Attrs *attrs,
                               const uint8_t    *buffer,
                               uint32_t          writeSizeRemaining)
{
    uint32_t size                  = writeSizeRemaining;
    uint32_t lineStatus            = 0U;
    uint32_t tempChunksize         = 0U;
    int32_t  maxTrialCount         = (int32_t) UART_TRANSMITEMPTY_TRIALCOUNT;

    /* Load the fifo size  */
    tempChunksize = UART_FIFO_SIZE;

    /* Before we could write no of bytes, we should have
     * no of free buffers. Hence, we check for shiftregister
     * empty (ensure the FIFO is empty) to write num of bytes */
    do
    {
        lineStatus = (uint32_t) UART_readLineStatus(attrs->baseAddr);
        maxTrialCount--;
    }
    while (((uint32_t) (UART_LSR_TX_SR_E_MASK | UART_LSR_TX_FIFO_E_MASK) !=
            ((uint32_t) (UART_LSR_TX_SR_E_MASK |
                       UART_LSR_TX_FIFO_E_MASK) & lineStatus))
           && (0U < maxTrialCount));

    if (maxTrialCount > 0U)
    {
        while ((tempChunksize > 0U) && (writeSizeRemaining > 0U))
        {
            /* Writing to the H/w */
            UART_putChar(attrs->baseAddr, (*buffer));
            buffer++;
            writeSizeRemaining--;
            tempChunksize--;
        }
    }

    /* Returns the size actually written */
    return (size - writeSizeRemaining);
}

static inline uint32_t UART_readData(UART_Object *object,
                                     UART_Attrs const *attrs,
                                     uint32_t size)
{
    uint8_t             readIn = 0;
    uint32_t            readSuccess;
    uint32_t             rdSize = size;

    readSuccess = UART_getChar(attrs->baseAddr, &readIn);

    /* Receive chars until empty or done. */
    while ((rdSize != 0U) && (readSuccess != FALSE))
    {
        *(uint8_t *)object->readBuf = readIn;
        object->readBuf = (uint8_t *)object->readBuf + 1U;
        object->readCount++;
        rdSize--;

        /* If read returnMode is UART_RETURN_FULL, avoids missing input character
         * of next read
         */
        if (rdSize != 0U)
        {
            readSuccess = UART_getChar(attrs->baseAddr, &readIn);
        }
    }

    return (rdSize);
}

static int32_t UART_readPolling(UART_Config      *config,
                                UART_Object      *object,
                                UART_Attrs const *attrs,
                                UART_Transaction *trans)
{
    uint32_t            timeout, startTicks, elapsedTicks;
    int32_t             retVal          = SystemP_SUCCESS;
    uint32_t            timeoutElapsed  = FALSE;

    timeout = trans->timeout;
    object->readSizeRemaining = trans->count;
    /* Update current tick value to perform timeout operation */
    startTicks = ClockP_getTicks();
    while ((FALSE == timeoutElapsed)
           && (0U != object->readSizeRemaining))
    {
        /* Transfer DATA */
        UART_readDataPolling(config, object, attrs);
        /* Check whether timeout happened or not */
        elapsedTicks = ClockP_getTicks() - startTicks;
        if (elapsedTicks >= timeout)
        {
            /* timeout occured */
            timeoutElapsed = TRUE;
        }
    }

    if ((object->readSizeRemaining == 0U) &&
        (object->readErrorCnt == 0U) && (object->rxTimeoutCnt == 0U))
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

static void UART_readDataPolling(UART_Config      *config,
                                 UART_Object      *object,
                                 UART_Attrs const *attrs)
{
    uint32_t numBytesRead = 0U;

    numBytesRead = UART_fifoRead(config, attrs,
                                 (uint8_t *) object->readBuf,
                                 object->readSizeRemaining);

    object->readSizeRemaining -= numBytesRead;
    object->readBuf           = (uint8_t *) object->readBuf + numBytesRead;
    object->readCount         += numBytesRead;

    return;
}

static uint32_t UART_fifoRead(UART_Config      *config,
                              const UART_Attrs *attrs,
                              uint8_t          *buffer,
                              uint32_t          readSizeRemaining)
{
    uint32_t size    = readSizeRemaining;
    Bool isRxReady = FALSE;

    isRxReady = UART_statusIsDataReady(config, attrs);

    while (((Bool) TRUE == isRxReady) && (0U != readSizeRemaining))
    {
        /* once the H/w is ready  reading from the H/w                        */
        *buffer = (UInt8) UART_readByte(config, attrs);
        buffer++;
        --readSizeRemaining;

        isRxReady = UART_statusIsDataReady(config, attrs);
    }

    return (size - readSizeRemaining);
}

static inline uint8_t UART_readByte(UART_Config *config,
                                    const UART_Attrs *attrs)
{
    uint8_t           readByte = 0;
    volatile uint32_t waitCount = UART_ERROR_COUNT;
    uint32_t          errorVal;
    UART_Object      *object = (UART_Object*)config->object;

    errorVal = UART_getRxError(attrs->baseAddr);
    /* Read and throw Erroneous bytes from RxFIFO */
    while ((UART_LSR_RX_FIFO_STS_MASK |
            UART_LSR_RX_BI_MASK |
            UART_LSR_RX_FE_MASK |
            UART_LSR_RX_PE_MASK |
            UART_LSR_RX_OE_MASK) == errorVal)
    {
        readByte = UART_fifoCharGet(attrs->baseAddr);
        object->readErrorCnt++;
        waitCount--;

        errorVal = UART_getRxError(attrs->baseAddr);
        if (0U == waitCount)
        {
            break;
        }
    }
    /* Read non-erroneous byte from RxFIFO */
    readByte = UART_fifoCharGet(attrs->baseAddr);

    return readByte;
}

static Bool UART_statusIsDataReady(UART_Config       *config,
                                  const UART_Attrs  *attrs)
{
    uint32_t status = 0;
    Bool retVal   = FALSE;

    status = (UInt32) UART_readLineStatus(attrs->baseAddr);

    /* Added for error checks */
    if ((UInt32) UART_LSR_RX_FIFO_STS_MASK ==
        (status & (UInt32) UART_LSR_RX_FIFO_STS_MASK))
    {
        UART_procLineStatusErr(config);
    }
    /* Caution: This should be under if else of error check since
     * the RX error handler clears the FIFO. Hence the status we have read
     * before this call will become stale. Hence the data will not be
     * ready in FIFO. Otherwise we will read the FIFO register which has
     * a infinite loop for data ready and the code hangs there till user
     * gives any character!! */
    else if ((UInt32) UART_LSR_RX_FIFO_E_MASK ==
             (status & (UInt32) UART_LSR_RX_FIFO_E_MASK))
    {
        retVal = (Bool) TRUE;
    }
    else
    {
        /* Do nothing */
    }

    return retVal;
}

static inline void UART_procLineStatusErr(UART_Config *config)
{
    UART_Object      *object = (UART_Object*)config->object;
    UART_Attrs const *attrs  = (UART_Attrs const *)config->attrs;
    uint32_t          lineStatus, iteration = 0U;

    lineStatus = UART_readLineStatus(attrs->baseAddr);

    if (((lineStatus & UART_FIFO_PE_FE_BI_DETECTED) == UART_FIFO_PE_FE_BI_DETECTED)
            || ((lineStatus & UART_OVERRUN_ERROR) == UART_OVERRUN_ERROR))
    {
        /* empty the RX FIFO which contains data with errors */
        if (object->readTrans != NULL)
        {
            object->readTrans->count = (uint32_t)(object->readCount);
        }

        /* Clearing Receive Errors(FE,BI,PE)by reading erroneous data from RX FIFO */
        /* Iteration count: Worst case = FIFO size */
        iteration = UART_FIFO_SIZE;
        do
        {
            /* Read and throw error byte */
            /* Till Line status int is pending */
            (void)UART_fifoCharGet(attrs->baseAddr);

            iteration--;

            lineStatus = (uint32_t) UART_readLineStatus(attrs->baseAddr);
            lineStatus &= (UART_LSR_RX_FIFO_STS_MASK |
                       UART_LSR_RX_BI_MASK |
                       UART_LSR_RX_FE_MASK |
                       UART_LSR_RX_PE_MASK |
                       UART_LSR_RX_OE_MASK |
                       UART_LSR_RX_FIFO_E_MASK);
        }
        while ((lineStatus != 0U) && (iteration != 0U));

        UART_intrDisable(attrs->baseAddr, UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);

        /* Reset the read buffer and read count so we can pass it back */
        object->readBuf = (uint8_t *)object->readBuf - object->readCount;

        if (object->readTrans != NULL)
        {
            if ((lineStatus & UART_BREAK_DETECTED_ERROR) != 0U)
            {
                object->readTrans->status = UART_TRANSFER_STATUS_ERROR_BI;
                object->readErrorCnt++;
            }
            else if ((lineStatus & UART_FRAMING_ERROR) != 0U)
            {
                object->readTrans->status = UART_TRANSFER_STATUS_ERROR_FE;
                object->readErrorCnt++;
            }
            else if ((lineStatus & UART_PARITY_ERROR) != 0U)
            {
                object->readTrans->status = UART_TRANSFER_STATUS_ERROR_PE;
                object->readErrorCnt++;
            }
            else
            {
                object->readTrans->status = UART_TRANSFER_STATUS_ERROR_OE;
                object->readErrorCnt++;
            }
        }

        /*
        * Post transfer Sem in case of bloacking transfer.
        * Call the callback function in case of Callback mode.
        */
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

    return;
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
    if (NULL == handle)
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

void UART_enableLoopbackMode(uint32_t baseAddr)
{
    uint32_t lcrRegValue;

    /* Switching to Register Configuration Mode A. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_A);

    /* Enable Loopback Mode. */
    HW_WR_FIELD32(baseAddr + UART_MCR, UART_MCR_LOOPBACK_EN,
                  UART_MCR_LOOPBACK_EN_LOOPBACK_EN_VALUE_1);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return;
}

void UART_disableLoopbackMode(uint32_t baseAddr)
{
    uint32_t lcrRegValue;

    /* Switching to Register Configuration Mode A. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_A);

    /* Enable Loopback Mode. */
    HW_WR_FIELD32(baseAddr + UART_MCR, UART_MCR_LOOPBACK_EN,
                  UART_MCR_LOOPBACK_EN_LOOPBACK_EN_VALUE_0);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return;
}

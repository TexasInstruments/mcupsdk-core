/*
 * Copyright (C) 2024 Texas Instruments Incorporated
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
 *  \file   i2c_v0_lld.c
 *
 *  \brief  File containing I2C LLD Driver APIs implementation for V0.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/i2c/v0/lld/i2c_lld.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**
 *  \anchor I2CBufferStatus
 *  \name I2CBufferStatus API Values
 *  @{
 */
#define I2C_TX_BUFFER_STATUS        ((uint32_t) 0U)
#define I2C_RX_BUFFER_STATUS        ((uint32_t) 1U)
/** \brief  Internal FIFO depth flag */
#define I2C_FIFO_DEPTH              ((uint32_t) 2U)
/** @} */

/**
 *  \anchor I2CFIFOThresholdConfig/I2CFIFOClear
 *  \name I2CFIFOThresholdConfig/I2CFIFOClear API Values
 *  @{
 */
#define I2C_TX_MODE                 ((uint32_t) 1U)
#define I2C_RX_MODE                 ((uint32_t) 0U)
/** @} */

/**
 *  \anchor I2CControllerIntEnableEx
 *  \name I2CControllerIntEnableEx API Values
 *  @{
 */
/** \brief Arbitration lost interrupt */
#define I2C_INT_ARBITRATION_LOST    ((uint32_t) I2C_IRQSTATUS_AL_MASK)
/** \brief No acknowledgement interrupt */
#define I2C_INT_NO_ACK              ((uint32_t) I2C_IRQSTATUS_NACK_MASK)
/** \brief Register access ready interrupt */
#define I2C_INT_ADRR_READY_ACESS    ((uint32_t) I2C_IRQSTATUS_ARDY_MASK)
/** \brief Receive data ready interrupt */
#define I2C_INT_RECV_READY          ((uint32_t) I2C_IRQSTATUS_RRDY_MASK)
/** \brief Transmit data ready interrupt */
#define I2C_INT_TRANSMIT_READY      ((uint32_t) I2C_IRQSTATUS_XRDY_MASK)
/** \brief General call Interrupt */
#define I2C_INT_GENERAL_CALL        ((uint32_t) I2C_IRQSTATUS_GC_MASK)
/** \brief Start Condition interrupt */
#define I2C_INT_START               ((uint32_t) I2C_IRQSTATUS_STC_MASK)
/** \brief Access Error interrupt */
#define I2C_INT_ACCESS_ERROR        ((uint32_t) I2C_IRQSTATUS_AERR_MASK)
/** \brief Bus Free interrupt */
#define I2C_INT_STOP_CONDITION      ((uint32_t) I2C_IRQSTATUS_BF_MASK)
/** \brief Addressed as Target interrupt */
#define I2C_INT_ADRR_TARGET         ((uint32_t) I2C_IRQSTATUS_AAS_MASK)
/** \brief Transmit underflow interrupt */
#define I2C_INT_TRANSMIT_UNDER_FLOW ((uint32_t) I2C_IRQSTATUS_XUDF_MASK)
/** \brief Receive overrun interrupt */
#define I2C_INT_RECV_OVER_RUN       ((uint32_t) I2C_IRQSTATUS_ROVR_MASK)
/** \brief Receive Draining interrupt */
#define I2C_INT_RECV_DRAIN          ((uint32_t) I2C_IRQSTATUS_RDR_MASK)
/** \brief Transmit Draining interrupt */
#define I2C_INT_TRANSMIT_DRAIN      ((uint32_t) I2C_IRQSTATUS_XDR_MASK)
/** \brief Bus busy interrupt raw status */
#define I2C_INT_BUS_BUSY            ((uint32_t) I2C_IRQSTATUS_RAW_BB_MASK)
/** \brief Bus free interrupt raw status */
#define I2C_INT_BUS_FREE            ((uint32_t) I2C_IRQSTATUS_RAW_BF_MASK)
/** \brief Enable all interrupt */
#define I2C_INT_ALL                 ((uint32_t) 0x7FFFU)
/** @} */


/**
 *  \anchor I2CControllerControl
 *  \name I2CControllerControl API Values
 *  @{
 */
/** \brief Controller transmit mode. */
#define I2C_CFG_MST_TX              (((uint32_t) I2C_CON_TRX_MASK) | \
                                             (uint32_t) (I2C_CON_MST_MASK))
/** \brief Controller receive mode. */
#define I2C_CFG_MST_RX              ((uint32_t) I2C_CON_MST_MASK)
/** \brief Stop condition. */
#define I2C_CFG_STOP                ((uint32_t) I2C_CON_STP_MASK)
/** \brief Normal mode. */
#define I2C_CFG_N0RMAL_MODE         ((uint32_t) 0 << I2C_CON_STB_SHIFT)
/** \brief Start byte mode. */
#define I2C_CFG_SRT_BYTE_MODE       ((uint32_t) I2C_CON_STB_MASK)
/** \brief 7 bit target address. */
#define I2C_CFG_7BIT_TARGET_ADDR    ((uint32_t) 0 << I2C_CON_XSA_SHIFT)
/** \brief 10 bit target address. */
#define I2C_CFG_10BIT_TARGET_ADDR   ((uint32_t) I2C_CON_XSA_MASK)
/** \brief Controller mode 10 bit own address 0 */
#define I2C_CFG_10BIT_OWN_ADDR_0    ((uint32_t) I2C_CON_XOA0_MASK)
/** \brief Controller mode 10 bit own address 1 */
#define I2C_CFG_10BIT_OWN_ADDR_1    ((uint32_t) I2C_CON_XOA1_MASK)
/** \brief Controller mode 10 bit own address 2 */
#define I2C_CFG_10BIT_OWN_ADDR_2    ((uint32_t) I2C_CON_XOA2_MASK)
/** \brief Controller mode 10 bit own address 3 */
#define I2C_CFG_10BIT_OWN_ADDR_3    ((uint32_t) I2C_CON_XOA3_MASK)
/** \brief Controller mode 7 bit own address 0 */
#define I2C_CFG_7BIT_OWN_ADDR_0     ((uint32_t) 0 << I2C_CON_XOA0_SHIFT)
/** \brief Controller mode 7 bit own address 1 */
#define I2C_CFG_7BIT_OWN_ADDR_1     ((uint32_t) 0 << I2C_CON_XOA1_SHIFT)
/** \brief Controller mode 7 bit own address 2 */
#define I2C_CFG_7BIT_OWN_ADDR_2     ((uint32_t) 0 << I2C_CON_XOA2_SHIFT)
/** \brief Controller mode 7 bit own address 3 */
#define I2C_CFG_7BIT_OWN_ADDR_3     ((uint32_t) 0 << I2C_CON_XOA3_SHIFT)
/** \brief I2C module enable */
#define I2C_CFG_MST_ENABLE          ((uint32_t) I2C_CON_I2C_EN_MASK)
/** \brief Start condition, initiate I2C transfer */
#define I2C_CFG_START               ((uint32_t) I2C_CON_STT_MASK)
/** \brief I2C configure controller mode. */
#define I2C_CFG_MST                 ((uint32_t) I2C_CON_MST_MASK)
/** \brief High speed operation mode */
#define I2C_CFG_HS_MOD              ((uint32_t) CSL_I2C_CON_OPMODE_HSI2C << CSL_I2C_CON_OPMODE_SHIFT)
/** @} */

/**
 *  \anchor ownAddressSet
 *  \name ownAddressSet API Values
 *  @{
 */
#define I2C_OWN_ADDR_0              ((uint32_t) 0U)
#define I2C_OWN_ADDR_1              ((uint32_t) 1U)
#define I2C_OWN_ADDR_2              ((uint32_t) 2U)
#define I2C_OWN_ADDR_3              ((uint32_t) 3U)
/** @} */

/**
 *  \anchor I2C_Prescaler
 *  \name MACROS used to in Prescaling I2C Clock.
 *  @{
 */
#define I2C_MAX_CLK_PRESCALAR       ((uint32_t) 255U)
#define I2C_INTERNAL_CLK_STEP       ((uint32_t) 1000000U)
/** @} */

#define I2C_DELAY_MED               ((uint32_t) 10000U)
#define I2C_DELAY_BIG               ((uint32_t) 30000U)
#define I2C_DELAY_SMALL             ((uint32_t) 5000U)

#define I2C_BUS_BUSY_TIMEOUT_IN_US          ((uint32_t) 500U)

#define I2C_MODULE_INTERNAL_CLK_4MHZ        ((uint32_t) 4000000U)
#define I2C_MODULE_INTERNAL_CLK_12MHZ       ((uint32_t) 12000000U)

#define I2C_MAX_CONSECUTIVE_ISRS            ((uint32_t) 1U)


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* Default I2C transaction parameters structure */
const I2CLLD_Transaction I2C_lld_defaultTransaction = {

    NULL,                                       /* writeBuf */
    0,                                          /* writeCount */
    NULL,                                       /* readBuf */
    0,                                          /* readCount */
};

/* Default I2C Message parameters structure */
const I2CLLD_Message I2C_lld_defaultMessage = {
    NULL,                                       /* txnArray */
    0,                                          /* txnCount */
    0,                                          /* targetAddress */
    NULL,                                       /* arg */
    I2C_WAIT_FOREVER,                           /* timeout */
    true,                                       /* controllerMode */
    false                                       /* expandSA */
};

/* ========================================================================== */
/*                      Internal Function Declarations                        */
/* ========================================================================== */
static void I2CControllerInitExpClk(uint32_t baseAddr, uint32_t sysClk,
                                    uint32_t internalClk, uint32_t outputClk);
static void I2CControllerEnable(uint32_t baseAddr);
static void I2CControllerDisable(uint32_t baseAddr);
static void I2CControllerEnableFreeRun(uint32_t baseAddr);
static void I2CControllerSetSysTest(uint32_t baseAddr, uint32_t sysTest);
static void I2CControllerControl(uint32_t baseAddr, uint32_t cmd);
static void I2CControllerStart(uint32_t baseAddr);
static void I2CControllerStop(uint32_t baseAddr);
static void I2CControllerIntEnableEx(uint32_t baseAddr, uint32_t intFlag);
static void I2CTargetIntEnableEx(uint32_t baseAddr, uint32_t intFlag);
static void I2CControllerIntDisableEx(uint32_t baseAddr, uint32_t intFlag);
static void I2CTargetIntDisableEx(uint32_t baseAddr, uint32_t intFlag);
static void I2CControllerIntClearEx(uint32_t baseAddr, uint32_t intFlag);
static void I2CTargetIntClearEx(uint32_t baseAddr, uint32_t intFlag);
static void I2CControllerTargetAddrSet(uint32_t baseAddr, uint32_t targetAdd);
static void I2CSetDataCount(uint32_t baseAddr, uint32_t count);
static void I2CFIFOClear(uint32_t baseAddr, uint32_t flag);
static void I2COwnAddressSet(uint32_t baseAddr, uint32_t ownAdd, uint32_t flag);
static void I2CSoftReset(uint32_t baseAddr);
static void I2CAutoIdleDisable(uint32_t baseAddr);
static void I2CControllerDataPut(uint32_t baseAddr, uint8_t data);
static void I2CSyscInit(uint32_t baseAddr, uint32_t syscFlag);
static void I2CConfig(uint32_t baseAddr, uint32_t conParams);

static uint32_t I2CControllerGetSysTest(uint32_t baseAddr);
static uint32_t I2CControllerIntStatus(uint32_t baseAddr);
static uint32_t I2CControllerIntRawStatus(uint32_t baseAddr);
static uint32_t I2CTargetIntRawStatus(uint32_t baseAddr);
static uint32_t I2CControllerIntRawStatusEx(uint32_t baseAddr,
                                            uint32_t intFlag);
static uint32_t I2CGetEnabledIntStatus(uint32_t baseAddr, uint32_t intFlag);
static uint32_t I2CDataCountGet(uint32_t baseAddr);
static uint32_t I2CBufferStatus(uint32_t baseAddr, uint32_t flag);
static uint32_t I2CSystemStatusGet(uint32_t baseAddr);

static uint8_t I2CControllerDataGet(uint32_t baseAddr);
static uint8_t I2CTargetDataGet(uint32_t baseAddr);

static int32_t I2CControllerBusBusy(uint32_t baseAddr);


static int32_t I2C_waitForBb(   I2CLLD_Handle handle, uint32_t timeout);
static int32_t I2C_waitForPin(  I2CLLD_Handle handle, uint32_t flag,
                                uint32_t *pTimeout);

static int32_t I2C_lld_primeTransferPoll(   I2CLLD_Handle handle);
static int32_t I2C_lld_primeTransferIntr(   I2CLLD_Handle handle);
static void I2C_lld_completeCurrTransfer(   I2CLLD_Handle handle,
                                            int32_t xferStatus);
static void I2C_lld_completeCurrTargetTransfer( I2CLLD_Handle handle,
                                                int32_t xferStatus);
static int32_t I2C_lld_transferInit(I2CLLD_Handle handle,
                                    I2CLLD_Message *msg);

static int32_t I2C_lld_resetCtrl(I2CLLD_Handle handle);
static int32_t I2C_lld_ctrlInit(I2CLLD_Handle handle);

/* ========================================================================== */
/*                       API Function Definitions                             */
/* ========================================================================== */

int32_t I2C_lld_init(I2CLLD_Handle handle)
{
    int32_t             status = I2C_STS_SUCCESS;
    I2CLLD_Object       *object = NULL;
    uint32_t            outputClk;
    uint32_t            internalClk;

    if(handle != NULL)
    {
        /* Get pointer to driver Object */
        object = (I2CLLD_Object *) handle;
        /* State should be reset before initialization */
        if(object->state != I2C_STATE_RESET)
        {
            status = I2C_STS_ERR;
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    if(status == I2C_STS_SUCCESS)
    {
        /* check if the I2C base address is valid or not */
        status = I2C_lld_isBaseAddrValid(object->baseAddr);
    }

    if(status == I2C_STS_SUCCESS)
    {
        /* Specify the busy state for this I2C peripheral */
        object->state = I2C_STATE_BUSY;
        /* Clear the current message pointer */
        object->currentMsg = NULL;
        /* Clear the current target transaction pointer */
        object->currentTargetTransaction = NULL;
        /* Set args as null */
        object->args = NULL_PTR;

        /* Put i2c in reset/disabled state */
        I2CControllerDisable(object->baseAddr);
        /* Disable Auto Idle functionality */
        I2CAutoIdleDisable(object->baseAddr);
        /* Extract bit rate from the input parameter */
        switch(object->bitRate)
        {
            case I2C_100KHZ:
            {
                outputClk = 100000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_4MHZ;
                break;
            }

            case I2C_400KHZ:
            {
                outputClk = 400000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_12MHZ;
                break;
            }

            case I2C_1P0MHZ:
            {
                outputClk = 1000000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_12MHZ;
                break;
            }

            case I2C_3P4MHZ:
            {
                outputClk = 3400000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_12MHZ;
                break;
            }

            default:
            {
                /* Default case force it to 100 KHZ bit rate */
                outputClk = 100000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_4MHZ;
                break;
            }
        }

        /* Set the I2C configuration */
        I2CControllerInitExpClk(object->baseAddr, object->funcClk,
                                internalClk, outputClk);
        /* Clear any pending interrupts */
        I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);
        /* Mask off all interrupts */
        I2CControllerIntDisableEx(object->baseAddr, I2C_INT_ALL);
        /* Enable the I2C Controller for operation */
        I2CControllerEnable(object->baseAddr);
        /* Enable free run mode */
        I2CControllerEnableFreeRun(object->baseAddr);
        /* Specify the idle state for this I2C peripheral */
        object->state = I2C_STATE_IDLE;
    }

    return status;
}

int32_t I2C_lld_deInit(I2CLLD_Handle handle)
{
    int32_t             status = I2C_STS_SUCCESS;
    I2CLLD_Object       *object = NULL;

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;

        /* Check to see if a I2C transaction is in progress */
        if (object->currentMsg == NULL)
        {
            /* Mask I2C interrupts */
            I2CControllerIntDisableEx(object->baseAddr, I2C_INT_ALL);
            /* Disable the I2C Controller */
            I2CControllerDisable(object->baseAddr);
            /* Change driver state back to RESET */
            object->state = I2C_STATE_RESET;
        }
        else
        {
            status = I2C_STS_ERR;
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_Transaction_init(I2CLLD_Transaction *transaction)
{
    int32_t status = I2C_STS_SUCCESS;

    if(transaction != NULL)
    {
        *transaction = I2C_lld_defaultTransaction;
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_Message_init(I2CLLD_Message *msg)
{
    int32_t status = I2C_STS_SUCCESS;

    if(msg != NULL)
    {
        *msg = I2C_lld_defaultMessage;
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_write(  I2CLLD_Handle handle,
                        I2C_ExtendedParams *extendedParams,
                        uint32_t timeout)
{
    int32_t                 status = I2C_STS_SUCCESS;
    I2CLLD_Object           *object = NULL;

    if ((handle != NULL) && (extendedParams != NULL))
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;

        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* TODO: This should happen automically. */
            object->state = I2C_STATE_BUSY;

            (void)I2C_lld_Transaction_init(&object->i2ctxn);
            object->i2ctxn.writeBuf      = extendedParams->buffer;
            object->i2ctxn.writeCount    = extendedParams->size;

            (void)I2C_lld_Message_init(&object->i2cMsg);
            object->i2cMsg.txn              = &object->i2ctxn;
            object->i2cMsg.txnCount         = 1U;
            object->i2cMsg.targetAddress    = extendedParams->deviceAddress;
            object->i2cMsg.expandSA         = extendedParams->expandSA;
            object->i2cMsg.timeout          = timeout;

            object->currentMsg = &object->i2cMsg;
            object->currentTxnCount = 0U;

            status = I2C_lld_primeTransferPoll(handle);
        }
        else
        {
            status = I2C_STS_ERR_BUS_BUSY;
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_writeIntr(  I2CLLD_Handle handle,
                            I2C_ExtendedParams *extendedParams)
{
    int32_t                 status = I2C_STS_SUCCESS;
    I2CLLD_Object           *object = NULL;

    if ((handle != NULL) && (extendedParams != NULL))
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;

        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* TODO: This should happen automically. */
            object->state = I2C_STATE_BUSY;

            (void)I2C_lld_Transaction_init(&object->i2ctxn);
            object->i2ctxn.writeBuf      = extendedParams->buffer;
            object->i2ctxn.writeCount    = extendedParams->size;

            (void)I2C_lld_Message_init(&object->i2cMsg);
            object->i2cMsg.txn              = &object->i2ctxn;
            object->i2cMsg.txnCount         = 1U;
            object->i2cMsg.targetAddress    = extendedParams->deviceAddress;
            object->i2cMsg.expandSA         = extendedParams->expandSA;

            object->currentMsg = &object->i2cMsg;
            object->currentTxnCount = 0U;

            status = I2C_lld_primeTransferIntr(handle);
        }
        else
        {
            status = I2C_STS_ERR_BUS_BUSY;
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_read(   I2CLLD_Handle handle,
                        I2C_ExtendedParams *extendedParams,
                        uint32_t timeout)
{
    int32_t                 status = I2C_STS_SUCCESS;
    I2CLLD_Object           *object = NULL;

    if ((handle != NULL) && (extendedParams != NULL))
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;

        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* TODO: This should happen automically. */
            object->state = I2C_STATE_BUSY;

            (void)I2C_lld_Transaction_init(&object->i2ctxn);
            object->i2ctxn.readBuf      = extendedParams->buffer;
            object->i2ctxn.readCount    = extendedParams->size;

            (void)I2C_lld_Message_init(&object->i2cMsg);
            object->i2cMsg.txn              = &object->i2ctxn;
            object->i2cMsg.txnCount         = 1U;
            object->i2cMsg.targetAddress    = extendedParams->deviceAddress;
            object->i2cMsg.expandSA         = extendedParams->expandSA;
            object->i2cMsg.timeout          = timeout;

            object->currentMsg = &object->i2cMsg;
            object->currentTxnCount = 0U;

            status = I2C_lld_primeTransferPoll(handle);
        }
        else
        {
            status = I2C_STS_ERR_BUS_BUSY;
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_readIntr(   I2CLLD_Handle handle,
                            I2C_ExtendedParams *extendedParams)
{
    int32_t                 status = I2C_STS_SUCCESS;
    I2CLLD_Object           *object = NULL;

    if ((handle != NULL) && (extendedParams != NULL))
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;

        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* TODO: This should happen automically. */
            object->state = I2C_STATE_BUSY;

            (void)I2C_lld_Transaction_init(&object->i2ctxn);
            object->i2ctxn.readBuf      = extendedParams->buffer;
            object->i2ctxn.readCount    = extendedParams->size;

            (void)I2C_lld_Message_init(&object->i2cMsg);
            object->i2cMsg.txn              = &object->i2ctxn;
            object->i2cMsg.txnCount         = 1U;
            object->i2cMsg.targetAddress    = extendedParams->deviceAddress;
            object->i2cMsg.expandSA         = extendedParams->expandSA;

            object->currentMsg = &object->i2cMsg;
            object->currentTxnCount = 0U;

            status = I2C_lld_primeTransferIntr(handle);
        }
        else
        {
            status = I2C_STS_ERR_BUS_BUSY;
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_mem_write(  I2CLLD_Handle handle,
                            I2C_Memory_ExtendedParams *mem_extendedParams,
                            uint32_t timeout)
{
    int32_t                 status = I2C_STS_SUCCESS;
    I2CLLD_Object           *object = NULL;
    uint32_t                buffer_len = 0U;

    if ((handle != NULL) && (mem_extendedParams != NULL))
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;

        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* TODO: This should happen automically. */
            object->state = I2C_STATE_BUSY;

            buffer_len += mem_extendedParams->memAddrSize;
            buffer_len += mem_extendedParams->extendedParams.size;

            object->memTxnActive = true;
            object->memAddrSize = mem_extendedParams->memAddrSize;
            object->dataArray = &(mem_extendedParams->extendedParams.buffer[0]);

            if(mem_extendedParams->memAddrSize == I2C_MEM_ADDR_SIZE_8_BITS)
            {
                object->addBuff[0] = (uint8_t)(mem_extendedParams->memAddr &
                                                (uint32_t)0x00FF);
            }
            else if(mem_extendedParams->memAddrSize==I2C_MEM_ADDR_SIZE_16_BITS)
            {
                object->addBuff[0] = (uint8_t)((mem_extendedParams->memAddr &
                                                (uint32_t)0xFF00) >> 8U);
                object->addBuff[1] = (uint8_t)(mem_extendedParams->memAddr &
                                                (uint32_t)0x00FF);
            }
            else
            {
                status = I2C_STS_ERR_INVALID_PARAM;
            }

            if(status == I2C_STS_SUCCESS)
            {
                (void)I2C_lld_Transaction_init(&object->i2ctxn);
                object->i2ctxn.writeBuf = &object->addBuff[0];
                object->i2ctxn.writeCount = buffer_len;

                (void)I2C_lld_Message_init(&object->i2cMsg);
                object->i2cMsg.txn = &object->i2ctxn;
                object->i2cMsg.txnCount = 1U;
                object->i2cMsg.targetAddress =
                            mem_extendedParams->extendedParams.deviceAddress;
                object->i2cMsg.expandSA =
                            mem_extendedParams->extendedParams.expandSA;
                object->i2cMsg.timeout = timeout;

                object->currentMsg = &object->i2cMsg;
                object->currentTxnCount = 0U;

                status = I2C_lld_primeTransferPoll(handle);
            }

            object->memTxnActive = false;
        }
        else
        {
            status = I2C_STS_ERR_BUS_BUSY;
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_mem_writeIntr(  I2CLLD_Handle handle,
                                I2C_Memory_ExtendedParams *mem_extendedParams)
{
    int32_t                 status = I2C_STS_SUCCESS;
    I2CLLD_Object           *object = NULL;
    uint32_t                buffer_len = 0U;

    if ((handle != NULL) && (mem_extendedParams != NULL))
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;

        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* TODO: This should happen automically. */
            object->state = I2C_STATE_BUSY;

            buffer_len += mem_extendedParams->memAddrSize;
            buffer_len += mem_extendedParams->extendedParams.size;

            object->memTxnActive = true;
            object->memAddrSize = mem_extendedParams->memAddrSize;
            object->dataArray = &(mem_extendedParams->extendedParams.buffer[0]);

            if(mem_extendedParams->memAddrSize == I2C_MEM_ADDR_SIZE_8_BITS)
            {
                object->addBuff[0] = (uint8_t)(mem_extendedParams->memAddr &
                                                (uint32_t)0x00FF);
            }
            else if(mem_extendedParams->memAddrSize==I2C_MEM_ADDR_SIZE_16_BITS)
            {
                object->addBuff[0] = (uint8_t)((mem_extendedParams->memAddr &
                                                (uint32_t)0xFF00) >> 8U);
                object->addBuff[1] = (uint8_t)(mem_extendedParams->memAddr &
                                                (uint32_t)0x00FF);
            }
            else
            {
                status = I2C_STS_ERR_INVALID_PARAM;
            }

            if(status == I2C_STS_SUCCESS)
            {
                (void)I2C_lld_Transaction_init(&object->i2ctxn);
                object->i2ctxn.writeBuf = &object->addBuff[0];
                object->i2ctxn.writeCount = buffer_len;

                (void)I2C_lld_Message_init(&object->i2cMsg);
                object->i2cMsg.txn = &object->i2ctxn;
                object->i2cMsg.txnCount = 1U;
                object->i2cMsg.targetAddress =
                            mem_extendedParams->extendedParams.deviceAddress;
                object->i2cMsg.expandSA =
                            mem_extendedParams->extendedParams.expandSA;


                object->currentMsg = &object->i2cMsg;
                object->currentTxnCount = 0U;

                status = I2C_lld_primeTransferIntr(handle);
            }
        }
        else
        {
            status = I2C_STS_ERR_BUS_BUSY;
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_mem_read(   I2CLLD_Handle handle,
                            I2C_Memory_ExtendedParams *mem_extendedParams,
                            uint32_t timeout)
{
    int32_t                 status = I2C_STS_SUCCESS;
    I2CLLD_Object           *object = NULL;
    uint32_t                buffer_len = 0U;

    if ((handle != NULL) && (mem_extendedParams != NULL))
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;

        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* TODO: This should happen automically. */
            object->state = I2C_STATE_BUSY;

            buffer_len += mem_extendedParams->memAddrSize;

            object->memTxnActive = true;

            if(mem_extendedParams->memAddrSize == I2C_MEM_ADDR_SIZE_8_BITS)
            {
                object->addBuff[0] = (uint8_t)(mem_extendedParams->memAddr &
                                                (uint32_t)0x00FF);
            }
            else if(mem_extendedParams->memAddrSize==I2C_MEM_ADDR_SIZE_16_BITS)
            {
                object->addBuff[0] = (uint8_t)((mem_extendedParams->memAddr &
                                                (uint32_t)0xFF00) >> 8U);
                object->addBuff[1] = (uint8_t)(mem_extendedParams->memAddr &
                                                (uint32_t)0x00FF);
            }
            else
            {
                status = I2C_STS_ERR_INVALID_PARAM;
            }

            if(status == I2C_STS_SUCCESS)
            {
                (void)I2C_lld_Transaction_init(&object->i2ctxn);
                object->i2ctxn.writeBuf = &object->addBuff[0];
                object->i2ctxn.writeCount = buffer_len;
                object->i2ctxn.readBuf = mem_extendedParams->extendedParams.buffer;
                object->i2ctxn.readCount = mem_extendedParams->extendedParams.size;

                (void)I2C_lld_Message_init(&object->i2cMsg);
                object->i2cMsg.txn = &object->i2ctxn;
                object->i2cMsg.txnCount = 1U;
                object->i2cMsg.targetAddress =
                            mem_extendedParams->extendedParams.deviceAddress;
                object->i2cMsg.expandSA =
                            mem_extendedParams->extendedParams.expandSA;
                object->i2cMsg.timeout = timeout;

                object->currentMsg = &object->i2cMsg;
                object->currentTxnCount = 0U;

                status = I2C_lld_primeTransferPoll(handle);
            }
            object->memTxnActive = false;
        }
        else
        {
            status = I2C_STS_ERR_BUS_BUSY;
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_mem_readIntr(   I2CLLD_Handle handle,
                                I2C_Memory_ExtendedParams *mem_extendedParams)
{
    int32_t                 status = I2C_STS_SUCCESS;
    I2CLLD_Object           *object = NULL;

    if ((handle != NULL) && (mem_extendedParams != NULL))
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;

        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* TODO: This should happen automically. */
            object->state = I2C_STATE_BUSY;

            object->memTxnActive = true;
            object->memAddrSize = mem_extendedParams->memAddrSize;

            if(mem_extendedParams->memAddrSize == I2C_MEM_ADDR_SIZE_8_BITS)
            {
                object->addBuff[0] = (uint8_t)(mem_extendedParams->memAddr &
                                                (uint32_t)0x00FF);
            }
            else if(mem_extendedParams->memAddrSize==I2C_MEM_ADDR_SIZE_16_BITS)
            {
                object->addBuff[0] = (uint8_t)((mem_extendedParams->memAddr &
                                                (uint32_t)0xFF00) >> 8U);
                object->addBuff[1] = (uint8_t)(mem_extendedParams->memAddr &
                                                (uint32_t)0x00FF);
            }
            else
            {
                status = I2C_STS_ERR_INVALID_PARAM;
            }

            if(status == I2C_STS_SUCCESS)
            {
                (void)I2C_lld_Transaction_init(&object->i2ctxn);
                object->i2ctxn.writeBuf = &object->addBuff[0];
                object->i2ctxn.writeCount = object->memAddrSize;
                object->i2ctxn.readBuf = mem_extendedParams->extendedParams.buffer;
                object->i2ctxn.readCount = mem_extendedParams->extendedParams.size;

                (void)I2C_lld_Message_init(&object->i2cMsg);
                object->i2cMsg.txn = &object->i2ctxn;
                object->i2cMsg.txnCount = 1U;
                object->i2cMsg.targetAddress =
                            mem_extendedParams->extendedParams.deviceAddress;
                object->i2cMsg.expandSA =
                            mem_extendedParams->extendedParams.expandSA;

                object->currentMsg = &object->i2cMsg;
                object->currentTxnCount = 0U;

                status = I2C_lld_primeTransferIntr(handle);
            }
        }
        else
        {
            status = I2C_STS_ERR_BUS_BUSY;
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_transferPoll(I2CLLD_Handle handle, I2CLLD_Message *msg)
{
    int32_t status;

    if((handle != NULL) && (msg != NULL))
    {
        status = I2C_lld_transferInit(handle, msg);
        if (status == I2C_STS_SUCCESS)
        {
            /* I2C_lld_transferInit returns I2C_STS_SUCCESS */
            status = I2C_lld_primeTransferPoll(handle);
        }
        else
        {
            /*  If status is not I2C_STS_SUCCESS, Can happen in casess
                I2C_lld_transferInit returns I2C_STS_ERR_INVALID_PARAM or
                I2C_STS_ERR_BUS_BUSY.

                In Such case return Genaric Error.
            */
            status = I2C_STS_ERR;
        }
    }
    else
    {
        /* In case of invalid Handle or Message passing */
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_transferIntr(I2CLLD_Handle handle, I2CLLD_Message *msg)
{
    int32_t status;

    if((handle != NULL) && (msg != NULL))
    {
        status = I2C_lld_transferInit(handle, msg);
        if (status == I2C_STS_SUCCESS)
        {
            status = I2C_lld_primeTransferIntr(handle);
        }
        else
        {
            /*  If status is not I2C_STS_SUCCESS; Can happen in case
                controllerMode member of given msg is false
                In Such case return Genaric Error.
            */
            status = I2C_STS_ERR;
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_targetTransferIntr( I2CLLD_Handle handle,
                                    I2CLLD_targetTransaction *txn)
{
    int32_t                 retVal = I2C_STS_SUCCESS;
    I2CLLD_Object           *object = NULL;
    uint32_t                xsa;
    uint32_t                regVal = 0U;

    if ((handle != NULL) && (txn != NULL))
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;
        if (txn->timeout == 0U)
        {
            /* Timeout cannot be NO_WAIT, set it to Maximum Possible Value */
            txn->timeout = I2C_WAIT_FOREVER;
        }
    }
    else
    {
        retVal = I2C_STS_ERR_INVALID_PARAM;
    }

    if (retVal == I2C_STS_SUCCESS)
    {
        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* Set driver object state to Busy */
            object->state = I2C_STATE_BUSY;
            /* Save current transaction in driver Object */
            object->currentTargetTransaction = txn;
        }
        else
        {
            /* Some transfer is going on. return with system Busy. */
            retVal = I2C_STS_ERR_BUS_BUSY;
        }
    }

    if (retVal == I2C_STS_SUCCESS)
    {
        /* Store the current target transaction */
        object->currentTargetTransaction = txn;

        object->writeBufIdx = (uint8_t *)txn->writeBuf;
        object->writeCountIdx = (uint32_t)txn->writeCount;

        object->readBufIdx = (uint8_t *)txn->readBuf;
        object->readCountIdx = (uint32_t)txn->readCount;

        if (object->currentTargetTransaction->expandSA == (bool)true)
        {
            /* Enable the 10-bit address mode for Own Address 0 */
            xsa = I2C_CFG_10BIT_OWN_ADDR_0;
        }
        else
        {
            /* Enable the 7-bit address mode for Own Address 0 */
            xsa = I2C_CFG_7BIT_OWN_ADDR_0;
        }

        /* In target mode, set the I2C own address */
        I2COwnAddressSet(   object->baseAddr,
                            object->ownTargetAddr[0],
                            I2C_OWN_ADDR_0);

        /* Configure data buffer length to 0 as the actual number of bytes to
           transmit/receive is dependant on external controller. */
        I2CSetDataCount(object->baseAddr, 0U);
        /* Enable interrupts for target mode */
        I2CTargetIntEnableEx(object->baseAddr,
                            I2C_INT_TRANSMIT_READY | I2C_INT_RECV_READY |
                            I2C_INT_ADRR_READY_ACESS | I2C_INT_ADRR_TARGET);
        /* Start the I2C transfer in target mode */
        regVal = I2C_CFG_MST_ENABLE | xsa;

        if ((object->bitRate == I2C_1P0MHZ) ||
            (object->bitRate == I2C_3P4MHZ))
        {
            regVal |= I2C_CFG_HS_MOD;
        }
        I2CControllerControl(object->baseAddr, regVal);
    }

    return retVal;
}

int32_t I2C_lld_probe(I2CLLD_Handle handle, uint32_t targetAddr)
{
    int32_t             retVal = I2C_STS_ERR;
    uint32_t            regVal;
    I2CLLD_Object       *object = NULL;

    /* Input parameter validation */
    if (handle != NULL)
    {
        object = (I2CLLD_Object *)handle;

        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer should be going on */
            /* Change the current state to BUSY */
            object->state = I2C_STATE_BUSY;

            /* Get All the enabled Interrupts */
            regVal = I2CGetEnabledIntStatus(object->baseAddr, I2C_INT_ALL);
            /* Disable All the interrupts */
            I2CControllerIntDisableEx(object->baseAddr, I2C_INT_ALL);
            /* Wait until bus not busy */
            if (I2C_waitForBb(handle, I2C_DELAY_MED) != I2C_STS_SUCCESS)
            {
                retVal = I2C_STS_ERR;
            }
            else
            {
                /* Set target address */
                I2CControllerTargetAddrSet(object->baseAddr, (uint32_t)targetAddr);
                /* Try to write one byte */
                I2CControllerDataPut(object->baseAddr, (uint8_t) 0U);
                /* Set Data Count to one */
                I2CSetDataCount(object->baseAddr, (uint32_t) 1U);
                /* Stop bit needed here; Enable Stop condition */
                I2CConfig(object->baseAddr,
                            (I2C_CFG_MST_ENABLE | I2C_CFG_MST_TX |
                            I2C_CFG_START | I2C_CFG_STOP));

                /* Enough delay for the NACK bit set */
                object->Clock_uSleep(I2C_DELAY_BIG);

                /* Check if NACK bit is set */
                if (0U == I2CControllerIntRawStatusEx(object->baseAddr, I2C_INT_NO_ACK))
                {
                    /* NACK bit not set */
                    retVal = I2C_STS_SUCCESS;
                    /* Clear all interrupts */
                    I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);
                    /* Wait for stop to complete and bus to become free */
                    (void)I2C_waitForBb(handle, I2C_DELAY_MED);
                }
                else
                {
                    /* NACK bit set */
                    /* Clear all interrupts */
                    I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);
                    /* Generate Stop */
                    I2CControllerStop(object->baseAddr);
                    /* Wait for stop to complete and bus to become free */
                    (void)I2C_waitForBb(handle, I2C_DELAY_MED);
                    /* NACK bit was set; Device not Available */
                    retVal = I2C_STS_ERR;
                }

                /* Clear TX FIFO */
                I2CFIFOClear(object->baseAddr, I2C_TX_MODE);
                /* Clear RX FIFO */
                I2CFIFOClear(object->baseAddr, I2C_RX_MODE);
                /* Set Data Count to Zero */
                I2CSetDataCount(object->baseAddr, 0U);
            }

            /* Re-Enable the Interrupts that were disabled */
            I2CControllerIntEnableEx(object->baseAddr, regVal);
            /* Change the driver state back to IDLE */
            object->state = I2C_STATE_IDLE;
        }
        else
        {
            /* Some transfer is going on. return with system Busy. */
            retVal = I2C_STS_ERR_BUS_BUSY;
        }
    }

    return (retVal);
}

int32_t I2C_lld_setBusFrequency(I2CLLD_Handle handle, uint32_t busFrequency)
{
    int32_t             retVal = I2C_STS_SUCCESS;
    I2CLLD_Object       *object = NULL;
    uint32_t            outputClk = 0;
    uint32_t            internalClk = 0;

    /* Input parameter validation */
    if(NULL != handle)
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;

        if (object->state == I2C_STATE_IDLE)
        {
            /* Change driver state to Busy */
            object->state = I2C_STATE_BUSY;
            /* Put i2c in reset/disabled state */
            I2CControllerDisable(object->baseAddr);
            /* Extract bit rate from the input parameter */
            switch(busFrequency)
            {
                case I2C_100KHZ:
                {
                    outputClk = 100000U;
                    internalClk = I2C_MODULE_INTERNAL_CLK_4MHZ;
                    break;
                }

                case I2C_400KHZ:
                {
                    outputClk = 400000U;
                    internalClk = I2C_MODULE_INTERNAL_CLK_12MHZ;
                    break;
                }

                case I2C_1P0MHZ:
                {
                    outputClk = 1000000U;
                    internalClk = I2C_MODULE_INTERNAL_CLK_12MHZ;
                    break;
                }

                case I2C_3P4MHZ:
                {
                    outputClk = 3400000U;
                    internalClk = I2C_MODULE_INTERNAL_CLK_12MHZ;
                    break;
                }

                default:
                {
                    /* Default case force it to 100 KHZ bit rate */
                    outputClk = 100000U;
                    internalClk = I2C_MODULE_INTERNAL_CLK_4MHZ;
                    break;
                }
            }

            /* Set the I2C configuration */
            I2CControllerInitExpClk(object->baseAddr, object->funcClk,
                                    internalClk, outputClk);
            /* Clear any pending interrupts */
            I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);
            /* Mask off all interrupts */
            I2CControllerIntDisableEx(object->baseAddr, I2C_INT_ALL);
            /* Enable the I2C Controller for operation */
            I2CControllerEnable(object->baseAddr);
            /* Enable free run mode */
            I2CControllerEnableFreeRun(object->baseAddr);
            /* Change Driver state back to IDLE */
            object->state = I2C_STATE_IDLE;
        }
        else
        {
            /* Some transfer is going on. return with system Busy. */
            retVal = I2C_STS_ERR_BUS_BUSY;
        }
    }
    else
    {
        retVal = I2C_STS_ERR_INVALID_PARAM;
    }

    return retVal;
}

int32_t I2C_lld_recoverBus(I2CLLD_Handle handle, uint32_t i2cDelay)
{

    int32_t             retVal = I2C_STS_SUCCESS;
    I2CLLD_Object       *object = NULL;
    uint32_t            sysTest, i;

    if(handle != NULL)
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;
    }
    else
    {
        retVal = I2C_STS_ERR_INVALID_PARAM;
    }


    if (retVal == I2C_STS_SUCCESS)
    {
        /* Check if SDA or SCL is stuck low based on the SYSTEST.
        * If SCL is stuck low we reset the IP.
        * If SDA is stuck low drive 9 clock pulses on SCL and check if the
        * target has released the SDA. If not we reset the I2C controller.
        */

        sysTest = I2CControllerGetSysTest(object->baseAddr);
        if ((sysTest & I2C_SYSTEST_SCL_I_FUNC_MASK) == 0U)
        {
            /* SCL is stuck low reset the I2C IP */
            retVal = I2C_lld_resetCtrl(object);
        }
        else if ((sysTest & I2C_SYSTEST_SDA_I_FUNC_MASK) == 0U)
        {
            /* SDA is stuck low; generate 9 clk pulses on SCL */
            /* Switch to system test mode */
            sysTest = (((uint32_t)I2C_SYSTEST_ST_EN_MASK) |
                ((uint32_t)CSL_I2C_SYSTEST_TMODE_LOOPBACK << (uint32_t)CSL_I2C_SYSTEST_TMODE_SHIFT));

            I2CControllerSetSysTest(object->baseAddr, sysTest);

            for (i = 0; i < 9U; i++)
            {
                sysTest = (I2C_SYSTEST_SCL_O_SCLOH << I2C_SYSTEST_SCL_O_SHIFT);
                I2CControllerSetSysTest(object->baseAddr, sysTest);
                object->Clock_uSleep(i2cDelay);

                sysTest = (I2C_SYSTEST_SCL_O_SCLOL << I2C_SYSTEST_SCL_O_SHIFT);
                I2CControllerSetSysTest(object->baseAddr, sysTest);
                object->Clock_uSleep(i2cDelay);
            }

            /* Switch back to functional mode */
            sysTest = (I2C_SYSTEST_ST_EN_DISABLE << I2C_SYSTEST_ST_EN_SHIFT);
            sysTest |= (I2C_SYSTEST_TMODE_FUNCTIONAL << CSL_I2C_SYSTEST_TMODE_SHIFT);
            I2CControllerSetSysTest(object->baseAddr, sysTest);

            /* Now check if the SDA is releases. If its still stuck low,
             * There is nothing that can be done. We still try to reset our IP.
             */
            sysTest = I2CControllerGetSysTest(object->baseAddr);

            if ((sysTest & I2C_SYSTEST_SDA_I_FUNC_MASK) == 0U)
            {
                retVal = I2C_lld_resetCtrl(handle);
            }
        }
        else
        {
            /* Nothing to be done. SCA and SDA both are not stuck to low */
        }
    }

    return retVal;
}


/* ========================================================================== */
/*                     API ISR Function Definitions                           */
/* ========================================================================== */

void I2C_lld_controllerIsr(void *args)
{
    I2CLLD_Object       *object = NULL;
    int32_t             xferStatus = I2C_STS_SUCCESS;
    uint32_t            isrLoopCount = 0U;
    uint32_t            w;
    uint32_t            stat;
    uint32_t            rawStat;
    uint32_t            intErr = 0U;
    uint32_t            fatalError = 0U;
    uint32_t            xsa;
    uint32_t            regVal;
    uint32_t            loopFlag = TRUE;

    /* Get the pointer to the object */
    object = (I2CLLD_Object*)args;

    /*
     * Ack the stat in one go, but [R/X]RDY should be
     * acked after the data operation is complete.
     */

    /*
     * The Controller ISR handling is based on AM5 TRM HS I2C Programming Guide
     * Section 24.1.5. The driver only enables [R/X]RDY and ARDY interrupts, and
     * ISR checks the raw interrupt status register to handle any fatal errors
     * (No Ack, arbitration lost, etc.). It also logs non fatal errors (TX underflow
     * RX overflow, etc.) internally. The ISR also handles restart condition (write
     * followed by a restart restart)
     *
     *
     * Keep looping till there are no pending interrupts.
     * This allows I2C_INT_ADRR_READY_ACESS to be processed in same ISR
     * as [R/X]RDY and reduce interrupt count.
     *
     */

    while (loopFlag == TRUE)
    {
        /* Store current interrupt status */
        stat = I2CControllerIntStatus(object->baseAddr);
        if ((0U == stat) || (isrLoopCount > I2C_MAX_CONSECUTIVE_ISRS))
        {
            /* No interrupt remaining or loop count went over 1 */
            loopFlag = FALSE;
        }
        else
        {
            /* Loop Count under max intr and intrs available */
            /* Increment loop count */
            isrLoopCount++;
            /* Check Raw Interrupt Status */
            rawStat = I2CControllerIntRawStatus(object->baseAddr);

            if ((rawStat & I2C_INT_NO_ACK) != 0U)
            {
                intErr |= I2C_INT_NO_ACK;
                xferStatus = I2C_STS_ERR_NO_ACK;
                I2CControllerIntClearEx(object->baseAddr, I2C_INT_NO_ACK);
                fatalError = 1U;
            }

            if ((rawStat & I2C_INT_ARBITRATION_LOST) != 0U)
            {
                intErr |= I2C_INT_ARBITRATION_LOST;
                xferStatus = I2C_STS_ERR_ARBITRATION_LOST;
                I2CControllerIntClearEx(object->baseAddr,I2C_INT_ARBITRATION_LOST);
                fatalError = 1U;
            }

            if ((rawStat & I2C_INT_ACCESS_ERROR) != 0U)
            {
                intErr |= I2C_INT_ACCESS_ERROR;
                xferStatus = I2C_STS_ERR_ACCESS_ERROR;
                I2CControllerIntClearEx(object->baseAddr,I2C_INT_ACCESS_ERROR);
                fatalError = 1U;
            }

            if(fatalError != 0U)
            {
                /* Some Error Happened */
                /* Issue the stop condition */
                I2CControllerStop(object->baseAddr);
                /* Disable all interrupts */
                I2CControllerIntDisableEx(object->baseAddr, I2C_INT_ALL);
                /* Clear all interrupts */
                I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);

                I2CControllerIntEnableEx(object->baseAddr, I2C_INT_STOP_CONDITION);
                /* Clear TX FIFO */
                I2CFIFOClear(object->baseAddr, I2C_TX_MODE);
                /* Clear RX FIFO */
                I2CFIFOClear(object->baseAddr, I2C_RX_MODE);
                /* Set data count to zero */
                I2CSetDataCount(object->baseAddr, 0);
                /* Store interrupt status in driver object */
                object->intStatusErr |= intErr;
                /* Breat out of the interrpt Loop */
                break;
            }

            if(rawStat & I2C_INT_STOP_CONDITION)
            {
                if ((object->intStatusErr & I2C_INT_NO_ACK) != 0U)
                {
                    xferStatus = I2C_STS_ERR_NO_ACK;
                }

                if ((object->intStatusErr & I2C_INT_ARBITRATION_LOST) != 0U)
                {
                    xferStatus = I2C_STS_ERR_ARBITRATION_LOST;
                }

                if ((object->intStatusErr & I2C_INT_ACCESS_ERROR) != 0U)
                {
                    xferStatus = I2C_STS_ERR_ACCESS_ERROR;
                }

                /* Finish current transfer */
                I2C_lld_completeCurrTransfer(object, xferStatus);

                if(object->intStatusErr != 0)
                {
                    /* Disable all interrupts */
                    I2CControllerIntDisableEx(object->baseAddr, I2C_INT_ALL);
                    /* Clear all interrupts */
                    I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);
                    /* Put i2c in reset/disabled state */
                    I2CControllerDisable(object->baseAddr);
                    /* Enable i2c module */
                    I2CControllerEnable(object->baseAddr);
                    break;
                }
            }

            /*  Store all interrupts except receive ready and transmit ready
             *  in seperate variable & clear all those interrupts
             */
            w = stat & ~(I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY);
            I2CControllerIntClearEx(object->baseAddr, w);

            if (((stat & I2C_INT_ADRR_READY_ACESS) == I2C_INT_ADRR_READY_ACESS) ||
                ((stat & I2C_INT_BUS_FREE) == I2C_INT_BUS_FREE))
            {
                /* Bus free or register access ready interrupt fired */
                /* Clear Receive ready and transmit ready Interrupt */
                w = stat & (I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY);
                I2CControllerIntClearEx(object->baseAddr, w);

                if (((object->writeCountIdx) == 0U) && ((object->readCountIdx) != 0U))
                {
                    /* Write Completed; Read count is not zero */
                    /* Set read count */
                    I2CSetDataCount(object->baseAddr, object->readCountIdx);
                    /*
                     * Configure peripheral for I2C Receive mode,
                     * do not send stop when sending restart
                     */
                    if (object->currentMsg->expandSA == (bool)true)
                    {
                        /* Enable the 10-bit address mode */
                        xsa = I2C_CFG_10BIT_TARGET_ADDR;
                    }
                    else
                    {
                        /* Enable the 7-bit address mode */
                        xsa = I2C_CFG_7BIT_TARGET_ADDR;
                    }

                    regVal = I2C_CFG_MST_RX | xsa;
                    if ((object->bitRate == I2C_1P0MHZ) || (object->bitRate == I2C_3P4MHZ))
                    {
                        regVal |= I2C_CFG_HS_MOD;
                    }
                    I2CControllerControl(object->baseAddr, regVal);
                    /* Enable RX interrupt to handle data received */
                    I2CControllerIntEnableEx(object->baseAddr, I2C_INT_RECV_READY);
                    /* Start I2C peripheral in RX mode */
                    I2CControllerStart(object->baseAddr);
                }
                else
                {
                    /* Write completed; Read count is zero (Nothing to read) */
                    /* No further looping requried */
                    if ((rawStat & I2C_INT_BUS_BUSY) != 0U)
                    {
                        /* Bus Busy state */
                        /* Generate Stop */
                        I2CControllerStop(object->baseAddr);
                        /* Enable bus free interrupt to wait for bus release */
                        I2CControllerIntEnableEx(object->baseAddr, I2C_INT_BUS_FREE);
                    }
                    else
                    {
                        /* Disable all interrupts */
                        I2CControllerIntDisableEx(object->baseAddr, I2C_INT_ALL);
                        /* Complete current Transfer */
                        I2C_lld_completeCurrTransfer(object,xferStatus);
                    }
                    /* Set loopFlag to false; don't execute anything from below
                     * Transaction Finished.
                     */
                    loopFlag = FALSE;
                }
            }

            if ((loopFlag == TRUE) && (0U == intErr))
            {
                /* Further interrupts available but no error happend */
                if ((stat & I2C_INT_RECV_READY) != 0U)
                {
                    /* Receive ready interrupt happened */
                    /* Save the received data */
                    if (object->readBufIdx != NULL)
                    {
                        *(object->readBufIdx) = I2CControllerDataGet(object->baseAddr);
                        object->readBufIdx++;
                        object->readCountIdx--;
                    }
                    /* Clear receive ready interrpt */
                    I2CControllerIntClearEx((object->baseAddr),
                                            (stat & I2C_INT_RECV_READY));
                }

                if ((stat & I2C_INT_TRANSMIT_READY) != 0U)
                {
                    /* Transmit ready interrupt happened */
                    if (object->writeCountIdx != 0U)
                    {
                        /* Write count index is not Zero */
                        /*
                         * Write data until FIFO is full or all data written.
                         * The math below is: (DCOUNT - TXBUFSTAT) % 64 < TXBUFSIZE.
                         */
                        while ( ((object->writeCountIdx) != 0U) &&
                                /*    Pending number of bytes to be transmitted - number of bytes yet to be transmitted from fifo = actual pending bytes */
                                (   ((I2CDataCountGet(object->baseAddr) - I2CBufferStatus(object->baseAddr, I2C_TX_BUFFER_STATUS)) % 64U) <
                                    (((uint32_t)8U) << I2CBufferStatus(object->baseAddr,I2C_FIFO_DEPTH))))
                                    /* FIFO depth in Bytes */
                        {

                            /* Write data into transmit FIFO */
                            I2CControllerDataPut(   object->baseAddr,
                                                    *(object->writeBufIdx));
                            (object->writeBufIdx)++;
                            object->writeCountIdx--;

                            /* Update Write buffer index in case of a memory Transaction */
                            if(object->memTxnActive)
                            {
                                if( (object->i2ctxn.writeCount - object->memAddrSize) ==
                                    (object->writeCountIdx))
                                {
                                    object->writeBufIdx = object->dataArray;
                                }
                            }
                        }
                    }
                    else
                    {
                        /* Write count index is Zero */
                        /* Disable Transmit ready interrupt */
                        I2CControllerIntDisableEx(object->baseAddr, I2C_INT_TRANSMIT_READY);
                    }
                    /* Clear The Transmit Ready interrupt */
                    I2CControllerIntClearEx(object->baseAddr,
                                        (stat & I2C_INT_TRANSMIT_READY));
                }
            }

            if ((loopFlag == TRUE) && (0U == intErr))
            {
                /* Loop flag true with no error */
                /* Check weather Receive overrun happened */
                if ((stat & I2C_INT_RECV_OVER_RUN) != 0U)
                {
                    intErr |= I2C_INT_RECV_OVER_RUN;
                }
                /* Check weather Transmit underflow happened */
                if ((stat & I2C_INT_TRANSMIT_UNDER_FLOW) != 0U)
                {
                     intErr |= I2C_INT_TRANSMIT_UNDER_FLOW;
                }
            }
        }
    }

    /* Save the interrupt status errors */
    object->intStatusErr |= intErr;
}

void I2C_lld_targetIsr(void *args)
{
    I2CLLD_Object       *object = NULL;
    uint32_t            rawStat;

    /* Get the pointer to the object */
    object = (I2CLLD_Object*)args;

    /*
     *  ISR Handling of different events is as below in same order:
     *  1.  RRDY: Handle receive ready interrupt before AAS(Address recognized as target) or ARDY(Register access ready IRQ enabled status) interrupt,
     *      as If ISR delayed and restart or stop condition received then
     *      the data should be read first.
     *  2.  ARDY: In case of stop condition bus busy will be reset. If it is a
     *      restart condition bus busy will still be set.
     *  3.  AAS: In case of addressed as target after transfer is initiated, it
     *      is a restart condition. Give the restart callback.
     *  4.  XRDY: Read application tx buffer and send data.
     *
     *  In case of restart hold the bus low till application gives next buffer.
     *  If the Buffers are overflowing or underflowing return error code
     *  accordingly. Application should call the abort transfer explicitly.
     */
    rawStat = I2CTargetIntRawStatus(object->baseAddr);

    if ((rawStat & I2C_INT_RECV_READY) != 0U)
    {
        /* Receive Ready interrupt received */
        if (object->readCountIdx != 0U)
        {
            /* Read count is not zero i.e. Bytes yet to be read */
            /* Read from RX register only when current transaction is ongoing */
            *(object->readBufIdx) = I2CControllerDataGet(object->baseAddr);
            object->readBufIdx++;
            object->readCountIdx--;
            /* Clear receive ready interrupt */
            I2CControllerIntClearEx(object->baseAddr, I2C_INT_RECV_READY);
        }
        else
        {
            /* Clear the RX data FIFO */
            (void)I2CTargetDataGet(object->baseAddr);
            /* Clear all Interrupts */
            I2CTargetIntClearEx(object->baseAddr, I2C_INT_ALL);
            /* Disable STOP condition Interrupt */
            I2CTargetIntDisableEx(object->baseAddr, I2C_INT_ALL);
            /* Finish the current transfer */
            I2C_lld_completeCurrTargetTransfer(object, I2C_STS_ERR);
        }
    }

    if ((rawStat & I2C_INT_ADRR_READY_ACESS) != 0U)
    {
        /* ARDY(Register access ready interrupt) */
        if ((rawStat & I2C_INT_BUS_BUSY) != 0U)
        {
            /* Bus Still busy means stop condition did not happen */
            /* Clear Interrupt, Callback will be handled in ADDR_TARGET */
            I2CTargetIntClearEx(object->baseAddr, I2C_INT_ADRR_READY_ACESS);
        }
        else
        {
            /* Bus busy was reset i.e. stop condition happened; End of transfer */
            /* Clear all Interrupts */
            I2CTargetIntClearEx(object->baseAddr, I2C_INT_ALL);
            /* Finish the current transfer */
            I2C_lld_completeCurrTargetTransfer(object, I2C_STS_SUCCESS);
        }
    }

    if ((rawStat & I2C_INT_ADRR_TARGET) != 0U)
    {
        /* AAS(Address recognized as target) interrupt happened */
        if (object->state == I2C_STATE_IDLE)
        {
            /* This is the first transfer initiation from controller */
            /* Update the state to transfer started */
            object->state = I2C_TARGET_XFER_STATE;
            /* Clear the Interrupt */
            I2CTargetIntClearEx(object->baseAddr, I2C_INT_ADRR_TARGET);
        }
        else if (object->state == I2C_TARGET_XFER_STATE)
        {
            if ((object->writeCountIdx == 0U) && ((rawStat & I2C_INT_TRANSMIT_UNDER_FLOW) != 0U))
            {
                /*
                 * This is a restart condition, target write count should be provided by
                 * application in the callback function
                 */

                /* Callback to application to restart read/write */
                object->targetTransferCompleteCallback( (void *)object,
                                                object->currentTargetTransaction,
                                                I2C_STS_RESTART);

                object->writeBufIdx = (uint8_t*)(object->currentTargetTransaction->writeBuf);
                object->writeCountIdx = (uint32_t)(object->currentTargetTransaction->writeCount);

                object->readBufIdx = (uint8_t*)(object->currentTargetTransaction->readBuf);
                object->readCountIdx = (uint32_t)(object->currentTargetTransaction->readCount);
            }
            else
            {
                /* No Code */
            }

            object->state = I2C_TARGET_RESTART_STATE;
            I2CTargetIntClearEx(object->baseAddr, I2C_INT_ADRR_TARGET);
        }
        else
        {
            /* Control should not come here. Sphurious interrupt clear it. */
            I2CTargetIntClearEx(object->baseAddr, I2C_INT_ADRR_TARGET);
        }
    }

    if ((rawStat & I2C_INT_TRANSMIT_READY) != 0U)
    {
        /* XRDY(Transmit Ready Interrupt happened) */
        if (object->state == I2C_TARGET_XFER_STATE)
        {
            /* I2C in Target Transfer state */
            if (object->writeCountIdx != 0U)
            {
                /* Write count not zero */
                I2CControllerDataPut(object->baseAddr, *(object->writeBufIdx));
                object->writeCountIdx--;
                object->writeBufIdx++;
                I2CTargetIntClearEx(object->baseAddr, I2C_INT_TRANSMIT_READY);
            }
            else
            {
                /* Write count zero; write Completed */
                /* Clear all interrupts */
                I2CTargetIntClearEx(object->baseAddr, I2C_INT_ALL);
                /* Disable STOP condition interrupt */
                I2CTargetIntDisableEx(object->baseAddr, I2C_INT_ALL);
                /* Update Read buffer and count */
                object->readBufIdx = (uint8_t*)(object->currentTargetTransaction->readBuf);
                object->readCountIdx = (uint32_t)(object->currentTargetTransaction->readCount);
                /* Post Semaphore to unblock transfer fxn */
                object->targetTransferCompleteCallback( (void*)object,
                                                object->currentTargetTransaction,
                                                I2C_STS_ERR);
                /* No other transactions need to occur */
                object->currentTargetTransaction = NULL;
            }
        }
        else
        {
            /* I2C is not in target Transfer State; may be restart state */
            if (object->writeCountIdx != 0U)
            {
                I2CControllerDataPut(object->baseAddr, *(object->writeBufIdx));
                object->writeCountIdx--;
                object->writeBufIdx++;
                I2CTargetIntClearEx(object->baseAddr, I2C_INT_TRANSMIT_READY);
            }
            else
            {
                /* Clear all interrupts */
                I2CTargetIntClearEx(object->baseAddr, I2C_INT_ALL);
                /* Disable STOP condition interrupt */
                I2CTargetIntDisableEx(object->baseAddr, I2C_INT_ALL);
                /* Update Read buffer and count */
                object->readBufIdx = (uint8_t*)(object->currentTargetTransaction->readBuf);
                object->readCountIdx = (uint32_t)object->currentTargetTransaction->readCount;
                /* Post Semaphore to unblock transfer fxn */
                object->targetTransferCompleteCallback( (void*)object,
                                                object->currentTargetTransaction,
                                                I2C_STS_ERR);
                /* No other transactions need to occur */
                object->currentTargetTransaction = NULL;

            }
        }
    }
    return;
}

/* ========================================================================== */
/*                      Internal Function Definitions                         */
/* ========================================================================== */

static void I2CControllerInitExpClk(uint32_t baseAddr,
                                    uint32_t sysClk,
                                    uint32_t internalClk,
                                    uint32_t outputClk)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    uint32_t prescalar;
    uint32_t divisor;
    uint32_t div_h, div_l;
    uint32_t actIntClk = 0U;

    /* Iterate through the valid pre-scalar values until one is found that is
     * the closest match to the desired internal clock speed
     */
    for (prescalar = 0U; prescalar < I2C_MAX_CLK_PRESCALAR; prescalar++)
    {
        /* Calculate the actual speed that would be used for the current
         * pre-scalar value, and if it is within 1 MHz of the desired value then
         * we have a match
         */
        actIntClk = sysClk / (prescalar + 1U);

        if (actIntClk <= (internalClk + I2C_INTERNAL_CLK_STEP))
        {
            break;
        }
    }

    if (outputClk > 400000U)
    {
        /* Prescalar bypassed in high speed mode */
        prescalar = 0;
        actIntClk = sysClk;
    }
    i2cRegs->PSC = prescalar;

    /* Calculate the divisor to be used based on actual internal clock speed
     * based on pre-scalar value used
     */
    divisor = actIntClk / outputClk;
    if ((outputClk * divisor) != actIntClk)
    {
        /* Round up the divisor so that output clock never exceeds the
         * requested value if it is not exact
         */
        divisor += 1U;
    }

    /* Split into SCK HIGH and LOW time to take odd numbered divisors
     * into account and avoid reducing the output clock frequency
     */
    div_h = divisor / 2U;
    div_l = divisor - div_h;

    if (outputClk > 400000U)
    {
        i2cRegs->SCLL = (uint32_t)((div_l - 7U) << 8U);
        i2cRegs->SCLH = (uint32_t)((div_h - 5U) << 8U);
    }
    else
    {
        i2cRegs->SCLL = (uint32_t)(div_l - 7U);
        i2cRegs->SCLH = (uint32_t)(div_h - 5U);
    }
}



static void I2CControllerEnable(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->CON |= I2C_CON_I2C_EN_MASK;
}

static void I2CControllerEnableFreeRun(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->SYSTEST |= I2C_SYSTEST_FREE_MASK;
}

static void I2CControllerSetSysTest(uint32_t baseAddr, uint32_t sysTest)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->SYSTEST = sysTest;
}

static uint32_t I2CControllerGetSysTest(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    uint32_t sysTest = 0U;

    sysTest = i2cRegs->SYSTEST;

    return (sysTest);
}

static void I2CControllerDisable(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->CON &= ~I2C_CON_I2C_EN_MASK;
}

static int32_t I2CControllerBusBusy(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    int32_t status = (int32_t)0;

    if((i2cRegs->IRQSTATUS_RAW & CSL_I2C_IRQSTATUS_RAW_BB_MASK) != 0U)
    {
        status = (int32_t)1;
    }

    return status;
}

static void I2CControllerControl(uint32_t baseAddr, uint32_t cmd)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->CON = cmd | I2C_CON_I2C_EN_MASK;
}

static void I2CControllerStart(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->CON |= I2C_CON_STT_MASK;
}

static void I2CControllerStop(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->CON |= I2C_CON_STP_MASK;
}

static void I2CControllerIntEnableEx(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    uint32_t i2cRegValue = i2cRegs->IRQENABLE_SET;
    i2cRegValue |= intFlag;
    i2cRegs->IRQENABLE_SET = i2cRegValue;
}

static void I2CTargetIntEnableEx(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    uint32_t i2cRegValue = i2cRegs->IRQENABLE_SET;
    i2cRegValue |= intFlag;
    i2cRegs->IRQENABLE_SET = i2cRegValue;
}

static void I2CControllerIntDisableEx(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->IRQENABLE_CLR = intFlag;
}

static void I2CTargetIntDisableEx(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->IRQENABLE_CLR = intFlag;
}

static uint32_t I2CControllerIntStatus(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    return((uint32_t)(i2cRegs->IRQSTATUS));
}

static uint32_t I2CControllerIntRawStatus(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    return((uint32_t)(i2cRegs->IRQSTATUS_RAW));
}

static uint32_t I2CTargetIntRawStatus(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    return((uint32_t)(i2cRegs->IRQSTATUS_RAW));
}

static uint32_t I2CControllerIntRawStatusEx(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    return(((uint32_t)(i2cRegs->IRQSTATUS_RAW)) & intFlag);
}

static void I2CControllerIntClearEx(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->IRQSTATUS = intFlag;
}

static void I2CTargetIntClearEx(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->IRQSTATUS = intFlag;
}

static uint32_t I2CGetEnabledIntStatus(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    return (uint32_t)((i2cRegs->IRQENABLE_SET) & intFlag);
}

static void I2CControllerTargetAddrSet(uint32_t baseAddr, uint32_t targetAdd)
{

    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->SA = targetAdd;
}

static void I2CSetDataCount(uint32_t baseAddr, uint32_t count)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->CNT = count;
}

static uint32_t I2CDataCountGet(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    return (i2cRegs->CNT);
}

static void I2CFIFOClear(uint32_t baseAddr, uint32_t flag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    if (I2C_TX_MODE == flag)
    {
        i2cRegs->BUF |= I2C_BUF_TXFIFO_CLR_MASK;
    }
    else
    {
        i2cRegs->BUF |= I2C_BUF_RXFIFO_CLR_MASK;
    }
}

static uint32_t I2CBufferStatus(uint32_t baseAddr, uint32_t flag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    uint32_t status;

    switch (flag)
    {
        case I2C_TX_BUFFER_STATUS:
            status = ((i2cRegs->BUFSTAT) & (CSL_I2C_BUFSTAT_TXSTAT_MASK));
            break;

        case I2C_RX_BUFFER_STATUS:
            status = ((i2cRegs->BUFSTAT) & (CSL_I2C_BUFSTAT_RXSTAT_MASK));
            break;

        case I2C_FIFO_DEPTH:
            status = ((i2cRegs->BUFSTAT) & (CSL_I2C_BUFSTAT_FIFODEPTH_MASK));
            break;

        default:
            /* Invalid input */
            status = 0U;
            break;
    }

    return status;
}

static void I2COwnAddressSet(uint32_t baseAddr, uint32_t ownAdd, uint32_t flag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;

    switch (flag)
    {
        case I2C_OWN_ADDR_0:
            i2cRegs->OA = ownAdd;
            break;

        case I2C_OWN_ADDR_1:
            i2cRegs->OA1 = ownAdd;
            break;

        case I2C_OWN_ADDR_2:
            i2cRegs->OA2 = ownAdd;
            break;

        case I2C_OWN_ADDR_3:
            i2cRegs->OA3 = ownAdd;
            break;

        default:
            /* Invalid input */
            break;
    }
}

static void I2CSoftReset(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->SYSC |= I2C_SYSC_SRST_MASK;
}

static void I2CAutoIdleDisable(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->SYSC &= ~I2C_SYSC_AUTOIDLE_MASK;
}

static uint32_t I2CSystemStatusGet(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    return (i2cRegs->SYSS & I2C_SYSS_RDONE_MASK);
}

static void I2CControllerDataPut(uint32_t baseAddr, uint8_t data)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->DATA = (uint32_t)data;
    /*write data to be transmitted to Data transmit register */
}

static uint8_t I2CControllerDataGet(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    uint8_t rData;

    rData = (uint8_t)(i2cRegs->DATA);

    return rData;
}

static uint8_t I2CTargetDataGet(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    uint8_t rData;

    rData = (uint8_t)(i2cRegs->DATA);

    return rData;
}

static void I2CSyscInit(uint32_t baseAddr, uint32_t syscFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->SYSC = syscFlag;
}

static void I2CConfig(uint32_t baseAddr, uint32_t conParams)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->CON = conParams;
}

/** ============================================================================
* Description:      Waits for bus to become free; program will wait for the bus
*                   to become free for timeout number of ticks
*                   keeping track of time.
*                   In case value of provided timeout is zero; program will
*                   wait for 500 micro Seconds.
*
* Input Parameters: handle(I2CLLD_Handle)
*                   flag(uint32_t)
*                   pTimeout(uint32_t *)
* Return Values:    I2C_STS_SUCCESS,
*                   I2C_STS_ERR_TIMEOUT,
** ========================================================================== */

static int32_t I2C_waitForBb(I2CLLD_Handle handle, uint32_t timeout)
{
    uint32_t            stat;
    int32_t             retVal = I2C_STS_SUCCESS;
    uint32_t            bbtimeout = timeout;
    uint32_t            startTicks, elapsedTicks = 0;
    I2CLLD_Object       *object = NULL;

    /* Get Pointer to the driver object */
    object = (I2CLLD_Object *)handle;

    if(bbtimeout > 0U)
    {
        /* BUS BUS timeout greater than 0 (Valid Value) */
        /* Clear current interrupts...*/
        I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);

        while (bbtimeout > 0U)
        {
            /* Poll Bus Busy bit */
            stat = I2CControllerIntRawStatusEx(object->baseAddr, I2C_INT_BUS_BUSY);
            if (stat == 0U)
            {
                /* Bus Free Case, Break out of loop */
                break;
            }
            /* Decrement bus busy timeout */
            bbtimeout = bbtimeout - 1U;
        }

        if (timeout > 0U)
        {
            if (bbtimeout == 0U)
            {
                /* Timeout happened */
                retVal = I2C_STS_ERR_TIMEOUT;
            }
            else
            {
                /* Bus is free before timeout could happen */
                /* No Code */
            }
        }

        /* Clear all interrupts (delayed stuff) */
        I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);
    }
    else
    {
        /* If provided timeout value is zero */
        startTicks = object->Clock_getTicks();

        while((I2CControllerBusBusy(object->baseAddr) == 1)  &&
        (elapsedTicks < object->Clock_usecToTicks(I2C_BUS_BUSY_TIMEOUT_IN_US)))
        {
            elapsedTicks = object->Clock_getTicks() - startTicks;
            /* Set timeout error if timeout occurred */
            if(elapsedTicks >=
                    object->Clock_usecToTicks(I2C_BUS_BUSY_TIMEOUT_IN_US))
            {
                retVal = I2C_STS_ERR_TIMEOUT;
                break;
            }
        }
    }

    return retVal;
}

/** ============================================================================
* Description:      Internal function, waits for flag bits to be set while
*                   keeping track of time.
*
* Input Parameters: handle(I2CLLD_Handle)
*                   flag(uint32_t)
*                   pTimeout(uint32_t *)
* Return Values:    I2C_STS_SUCCESS,
*                   I2C_STS_ERR_TIMEOUT,
** ========================================================================== */

static int32_t I2C_waitForPin( I2CLLD_Handle handle,
                                uint32_t flag,
                                uint32_t *pTimeout)
{
    int32_t             status = I2C_STS_SUCCESS;
    uint32_t            intRawStat = 0U;
    uint32_t            timeout = *pTimeout;
    I2CLLD_Object       *object = NULL;

    /* Get pointer to the driver Object */
    object = (I2CLLD_Object *)handle;

    if(timeout > 0U)
    {
        /* Get Current Interrupt Status */
        intRawStat = I2CControllerIntRawStatus(object->baseAddr);
        /* Loop until the flag condition is met or timeout happens */
        while ( ((uint32_t) 0U == (intRawStat & flag)) &&
                (status == I2C_STS_SUCCESS))
        {
            if (    ((object->Clock_getTicks()) - (object->startTicks)) >
                    (object->Clock_usecToTicks((uint64_t)(timeout))))
            {
                status = I2C_STS_ERR_TIMEOUT;
            }
            /* Update Interrupt Status */
            intRawStat = I2CControllerIntRawStatus(object->baseAddr);
        }

        if (status == I2C_STS_ERR_TIMEOUT)
        {
            /* Clear all Interrupts in case of timeout */
            I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);
        }
    }

    return status;
}

/** ============================================================================
* Description:      Internal function to Initialize I2C Transaction
* Input Parameters: I2CLLD_Handle, I2CLLD_Message
* Return Values:    I2C_STS_SUCCESS,
*                   I2C_STS_ERR_INVALID_PARAM,
*                   I2C_STS_ERR_BUS_BUSY
** ========================================================================== */

static int32_t I2C_lld_transferInit(I2CLLD_Handle handle, I2CLLD_Message *msg)
{
    int32_t             status = I2C_STS_SUCCESS;
    I2CLLD_Object       *object = NULL;

    /* Get the pointer to the object */
    object = (I2CLLD_Object*)handle;

    if (msg->timeout == 0U)
    {
        /* Timeout cannot be I2C_NO_WAIT, set it to I2C_WAIT_FOREVER */
        msg->timeout = I2C_WAIT_FOREVER;
    }

    if((msg->txn) == (I2CLLD_Transaction *)NULL)
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    if (    (status == I2C_STS_SUCCESS) &&
            ((msg->txn->writeCount != 0U) || (msg->txn->readCount != 0U)))
    {
        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* Set state to busy */
            object->state = I2C_STATE_BUSY;
            /* Set current message */
            object->currentMsg = msg;
            /* Set transaction count to zero */
            object->currentTxnCount = 0U;
        }
        else
        {
            /* Some transfer is going on. return with system busy. */
            status = I2C_STS_ERR_BUS_BUSY;
        }
    }

    return status;
}

/** ============================================================================
* Description:      Polling mode prime transfer function. Carries out all
*                   all transactions within the given message
*                   I2C_lld_transferInit was called with before calling this.
*
* Input Parameters: I2CLLD_Handle
* Return Values:    I2C_STS_SUCCESS,
*                   I2C_STS_ERR_TIMEOUT,
*                   I2C_STS_ERR_ARBITRATION_LOST,
*                   I2C_STS_ERR_NO_ACK,
*                   I2C_STS_ERR_ACCESS_ERROR,
*                   I2C_STS_ERR
** ========================================================================== */

static int32_t I2C_lld_primeTransferPoll(I2CLLD_Handle handle)
{
    I2CLLD_Object       *object = NULL;
    int32_t             status = I2C_STS_SUCCESS;
    uint32_t            xsa, regVal;
    uint32_t            errStat = 0, fatalError = 0;

    /* Get the pointer to the object */
    object = (I2CLLD_Object*)handle;
    /* Store current time */
    object->startTicks = object->Clock_getTicks();

    while((object->currentTxnCount) < (object->currentMsg->txnCount))
    {
        /* Store Write and Read details in the driver object */
        object->writeBufIdx = (uint8_t*)object->currentMsg->txn[object->currentTxnCount].writeBuf;
        object->writeCountIdx = (uint32_t)object->currentMsg->txn[object->currentTxnCount].writeCount;

        object->readBufIdx = (uint8_t*)object->currentMsg->txn[object->currentTxnCount].readBuf;
        object->readCountIdx = (uint32_t)object->currentMsg->txn[object->currentTxnCount].readCount;

        /* Clear All interrupts */
        I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);

        if (object->currentMsg->expandSA == true)
        {
            /* Enable the 10-bit address mode */
            xsa = I2C_CFG_10BIT_TARGET_ADDR;
        }
        else
        {
            /* Enable the 7-bit address mode */
            xsa = I2C_CFG_7BIT_TARGET_ADDR;
        }

        /* Wait for Bus to be free */
        while(  (I2CControllerBusBusy(object->baseAddr) == (int32_t)1) &&
                (status == I2C_STS_SUCCESS))
        {
            if (    ((object->Clock_getTicks()) - (object->startTicks)) >
                    (object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout))))
            {
                status = I2C_STS_ERR_TIMEOUT;
            }
        }

        /* In case previous transaction failed and there is still data in FIFO */
        /* Clear TX FIFO */
        I2CFIFOClear(object->baseAddr, I2C_TX_MODE);
        /* Clear RX FIFO */
        I2CFIFOClear(object->baseAddr, I2C_RX_MODE);

        /* Controller Mode */
        if(object->currentMsg->controllerMode)
        {
            /* In controller mode, set the I2C target address */
            I2CControllerTargetAddrSet( object->baseAddr,
                                        object->currentMsg->targetAddress);

            if((object->writeCountIdx != 0U) && (status == I2C_STS_SUCCESS))
            {
                /* Set number of bytes to Write */
                I2CSetDataCount(object->baseAddr, object->writeCountIdx);
                /* Put controller in transmitter mode */
                regVal = I2C_CFG_MST_TX | xsa;
                /* Check if I2C is in HS-Mode or not */
                if (    (object->bitRate == I2C_1P0MHZ) ||
                        (object->bitRate == I2C_3P4MHZ))
                {
                    regVal |= I2C_CFG_HS_MOD;
                }
                /* Write configuration to control Register */
                I2CControllerControl(object->baseAddr, regVal);
                /* Generate start */
                I2CControllerStart(object->baseAddr);
                /* Loop Until Write is complete or timeout happens or some error happens */
                while ((object->writeCountIdx != 0U) && (status == I2C_STS_SUCCESS))
                {
                    /* Wait for transmit ready or error or timeout */
                    while(  (
                            (I2CControllerIntRawStatusEx(object->baseAddr,  I2C_INT_TRANSMIT_READY) == 0U) &&
                            (I2CControllerIntRawStatusEx(object->baseAddr, (I2C_INT_ARBITRATION_LOST    |
                                                                            I2C_INT_NO_ACK              |
                                                                            I2C_INT_ACCESS_ERROR        |
                                                                            I2C_INT_STOP_CONDITION) ) == 0U)) &&
                            (status == I2C_STS_SUCCESS))
                    {
                        if (((object->Clock_getTicks()) - (object->startTicks)) >
                            (object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout))))
                        {
                            status = I2C_STS_ERR_TIMEOUT;
                        }
                    }
                    /* Get raw interrupt status */
                    errStat = I2CControllerIntRawStatusEx(object->baseAddr, I2C_INT_ARBITRATION_LOST    | \
                                                                            I2C_INT_NO_ACK              | \
                                                                            I2C_INT_ACCESS_ERROR);

                    /* if we get an error or timeout happens, break out of loop */
                    if ((errStat != 0U) || (status != I2C_STS_SUCCESS))
                    {
                        if(errStat != 0U)
                        {
                            fatalError = 1U;
                        }
                        break;
                    }

                    /* Write byte and increase data pointer to next byte */
                    I2CControllerDataPut(object->baseAddr, *(object->writeBufIdx));
                    /* Increment write buffer index */
                    (object->writeBufIdx)++;
                    /* Clear transmit ready interrupt */
                    I2CControllerIntClearEx(object->baseAddr, I2C_INT_TRANSMIT_READY);
                    /* Update number of bytes to be written */
                    object->writeCountIdx--;
                    /* In case of memory transaction, change buffer index */
                    if(handle->memTxnActive)
                    {
                        if( (handle->i2ctxn.writeCount - handle->memAddrSize) ==
                            (handle->writeCountIdx))
                        {
                            handle->writeBufIdx = handle->dataArray;
                        }
                    }
                }

                if ((fatalError == 0U) && (status == I2C_STS_SUCCESS))
                {
                    /* No Error happened, write completed successfully */
                    /* Wait for register access ready */
                    status = I2C_waitForPin(    object,
                                                I2C_INT_ADRR_READY_ACESS,
                                                &(object->currentMsg->timeout));
                }

                if(fatalError == 1U)
                {
                    /* Some kind of Error Happened */
                    if ((errStat & I2C_INT_ARBITRATION_LOST) == I2C_INT_ARBITRATION_LOST)
                    {
                        status = I2C_STS_ERR_ARBITRATION_LOST;
                    }
                    else if ((errStat & I2C_INT_NO_ACK) == I2C_INT_NO_ACK)
                    {
                        status = I2C_STS_ERR_NO_ACK;
                    }
                    else if ((errStat & I2C_INT_ACCESS_ERROR) == I2C_INT_ACCESS_ERROR)
                    {
                        status = I2C_STS_ERR_ACCESS_ERROR;
                    }
                    else
                    {
                        /* No Code */
                    }
                    /* Store error in error status */
                    object->intStatusErr = errStat;
                }
                else
                {
                    /* No Code */
                }

                if (object->readCountIdx == 0U)
                {
                    /* There is nothing to read as part of this transaction */
                    /* Generate stop when there is no read followed by write */
                    I2CControllerStop(object->baseAddr);

                    /* Check if any Error occured during write */
                    if ((fatalError == 0U) && (status == I2C_STS_SUCCESS))
                    {
                        /* No Error happened; Write executed successfully */
                        /* Wait for stop to happen */
                        status = I2C_waitForPin(handle,
                                                I2C_INT_STOP_CONDITION,
                                                &(object->currentMsg->timeout));


                        /* Wait for register access ready */
                        status = I2C_waitForPin(handle,
                                                I2C_INT_ADRR_READY_ACESS,
                                                &(object->currentMsg->timeout));
                    }
                    else
                    {
                        /* Some Error happened */
                        /* Wait for stop to Happen; ignore return value */
                        (void)I2C_waitForPin(   handle,
                                                I2C_INT_STOP_CONDITION,
                                                &(object->currentMsg->timeout));
                        /* Wait for register access ready */
                        (void)I2C_waitForPin(   handle,
                                                I2C_INT_ADRR_READY_ACESS,
                                                &(object->currentMsg->timeout));
                        /* Return value ignored as status should hold the error
                        */
                    }
                }
                else
                {
                    /* There is some read to be executed
                    as part of this transaction */

                    /*
                        Do nothing
                        1. if some error happened during write status will not
                            be I2C_STS_SUCCESS so read will not happen.
                        2. if write operation happened successfully and read
                            count is not zero then driver will wait for
                            I2C_INT_ADRR_READY_ACESS and go for read operation
                            without stop condition.
                    */
                }
            }

            if ((object->readCountIdx != 0U) && (status == I2C_STS_SUCCESS))
            {
                /* Clear all interrupts */
                I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);
                /* Set number of bytes to read */
                I2CSetDataCount(object->baseAddr, object->readCountIdx);
                /* Set to controller receiver mode */
                regVal = I2C_CFG_MST_RX | xsa;
                /* Check if I2C is in HS-Mode or not */
                if ((object->bitRate == I2C_1P0MHZ) ||
                    (object->bitRate == I2C_3P4MHZ))
                {
                    regVal |= I2C_CFG_HS_MOD;
                }
                /* Write configuration to control Register */
                I2CControllerControl(object->baseAddr, regVal);
                /* Generate start */
                I2CControllerStart(object->baseAddr);
                /* Loop until read completes or some error happens */
                while ((object->readCountIdx != 0U) && (status == I2C_STS_SUCCESS))
                {
                    /* Wait for receive ready or error */
                    while(( (I2CControllerIntRawStatusEx(object->baseAddr,  I2C_INT_RECV_READY) == 0U) &&
                            (I2CControllerIntRawStatusEx(object->baseAddr,  I2C_INT_ARBITRATION_LOST    |
                                                                            I2C_INT_NO_ACK              |
                                                                            I2C_INT_ACCESS_ERROR ) == 0U)) &&
                            (status == I2C_STS_SUCCESS))
                    {
                        if (((object->Clock_getTicks()) - (object->startTicks)) >
                            (object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout))))
                        {
                            status = I2C_STS_ERR_TIMEOUT;
                        }
                    }

                    errStat = I2CControllerIntRawStatusEx(object->baseAddr, I2C_INT_ARBITRATION_LOST    |
                                                                            I2C_INT_NO_ACK              |
                                                                            I2C_INT_ACCESS_ERROR);

                    /* if we get an error or timeout happens, break out of loop */
                    if ((errStat != 0U) || (status != I2C_STS_SUCCESS))
                    {
                        if(errStat != 0U)
                        {
                            fatalError = 1U;
                        }
                        break;
                    }

                    /* Read byte */
                    *(object->readBufIdx) =
                        (uint8_t)I2CControllerDataGet(object->baseAddr);

                    /* Clear receive ready interrupt */
                    I2CControllerIntClearEx(object->baseAddr, I2C_INT_RECV_READY);
                    /* Increment read buffer index */
                    object->readBufIdx++;
                    /* Update number of bytes to be read */
                    object->readCountIdx--;
                }

                if ((fatalError == 0U) && (status == I2C_STS_SUCCESS))
                {
                    /* Wait for register access ready */
                    status = I2C_waitForPin(    handle,
                                                I2C_INT_ADRR_READY_ACESS,
                                                &(object->currentMsg->timeout));
                }

                if(fatalError == 1U)
                {
                    /* Some kind of Error Happened */
                    if ((errStat & I2C_INT_ARBITRATION_LOST) == I2C_INT_ARBITRATION_LOST)
                    {
                        status = I2C_STS_ERR_ARBITRATION_LOST;
                    }
                    else if ((errStat & I2C_INT_NO_ACK) == I2C_INT_NO_ACK)
                    {
                        status = I2C_STS_ERR_NO_ACK;
                    }
                    else if ((errStat & I2C_INT_ACCESS_ERROR) == I2C_INT_ACCESS_ERROR)
                    {
                        status = I2C_STS_ERR_ACCESS_ERROR;
                    }
                    else
                    {
                        /* No Code */
                     }
                    /* Store error in error status */
                    object->intStatusErr = errStat;
                }
                else
                {
                    /* No Error Happened */
                    /* No Code */
                }

                /* Generate stop */
                I2CControllerStop(object->baseAddr);

                if ((fatalError == 0U) && (status == I2C_STS_SUCCESS))
                {
                    /* No Error happened; Read executed successfully */
                    /* Wait for stop to happen */
                    status = I2C_waitForPin(    handle,
                                                I2C_INT_STOP_CONDITION,
                                                &(object->currentMsg->timeout));

                    /* Wait for register access ready */
                    status = I2C_waitForPin(    handle,
                                                I2C_INT_ADRR_READY_ACESS,
                                                &(object->currentMsg->timeout));
                }
                else
                {
                    /* Some Error happened */
                    /* Wait for stop to happen */
                    (void)I2C_waitForPin(   handle,
                                            I2C_INT_STOP_CONDITION,
                                            &(object->currentMsg->timeout));

                    /* Wait for register access ready */
                    (void)I2C_waitForPin(   handle,
                                            I2C_INT_ADRR_READY_ACESS,
                                            &(object->currentMsg->timeout));
                    /* Return value ignored as status should hold the error
                    */
                }
            }
        }
        /* Target Mode */
        else
        {
            /* Polling mode not available in target Mode */
            /* Control should not enter here */
            status = I2C_STS_ERR;
        }

        if(status == I2C_STS_SUCCESS)
        {
            /* If Last transaction completed successfully */
            /* Increment Transaction count and go to next Transaction */
            object->currentTxnCount++;
        }
        else
        {
            /* Last transaction failed */
            /* Break out of loop of transactions */
            break;
        }
    }

    /* No further Transactions or some existing transaction Failed */
    object->currentMsg = NULL;
    /* Change the driver State back to idle */
    object->state = I2C_STATE_IDLE;

    return status;
}

/** ============================================================================
* Description:      Interrupt mode prime transfer function. Carries out all
*                   all transactions within the given message
*                   I2C_lld_transferInit was called with before calling this.
*
* Input Parameters: I2CLLD_Handle
* Return Values:    I2C_STS_SUCCESS,
*                   I2C_STS_ERR
** ========================================================================== */

static int32_t I2C_lld_primeTransferIntr(I2CLLD_Handle handle)
{
    I2CLLD_Object       *object = NULL;
    int32_t             status = I2C_STS_SUCCESS;
    uint32_t            xsa, regVal;

    /* Get the pointer to the object */
    object = (I2CLLD_Object *)handle;

    /* Store Write and Read details in the driver object */
    object->writeBufIdx = (uint8_t*)object->currentMsg->txn[object->currentTxnCount].writeBuf;
    object->writeCountIdx = (uint32_t)object->currentMsg->txn[object->currentTxnCount].writeCount;

    object->readBufIdx = (uint8_t*)object->currentMsg->txn[object->currentTxnCount].readBuf;
    object->readCountIdx = (uint32_t)object->currentMsg->txn[object->currentTxnCount].readCount;

    if(object->intStatusErr != 0)
    {
        (void)I2C_lld_recoverBus(object, 10);
    }

    object->intStatusErr = 0;

    if (object->currentMsg->expandSA == true)
    {
        /* Enable the 10-bit address mode */
        xsa = I2C_CFG_10BIT_TARGET_ADDR;
    }
    else
    {
        /* Enable the 7-bit address mode */
        xsa = I2C_CFG_7BIT_TARGET_ADDR;
    }
    /* Clear all interrupts */
    I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);
    /* Clear TX FIFO */
    I2CFIFOClear(object->baseAddr, I2C_TX_MODE);
    /* Clear RX FIFO */
    I2CFIFOClear(object->baseAddr, I2C_RX_MODE);

    /* Controller Mode */
    if (object->currentMsg->controllerMode)
    {
        /* In controller mode, set the I2C target address */
        I2CControllerTargetAddrSet( object->baseAddr,
                                    object->currentMsg->targetAddress);
        /* Start transfer in Transmit mode */
        if (object->writeCountIdx != 0U)
        {
            /* Write Count not zero */
            /* Set number of bytes to be transmitting */
            I2CSetDataCount(object->baseAddr, object->writeCountIdx);
            /*
             * Configure the I2C transfer to be in controller transmitter mode
             */
            regVal = I2C_CFG_MST_TX | xsa;
            if ((object->readCountIdx) == 0U)
            {
                /*
                 * if there is no read data, automatically send stop when write is complete
                 * otherwise (restart), do not send stop
                 */
                regVal |= I2C_CFG_STOP;
            }

            if ((object->bitRate == I2C_1P0MHZ) || (object->bitRate == I2C_3P4MHZ))
            {
                regVal |= I2C_CFG_HS_MOD;
            }
            I2CControllerControl(object->baseAddr, regVal);

            regVal =    I2C_INT_TRANSMIT_READY | I2C_INT_ADRR_READY_ACESS |
                        I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST;
            /* Enable Requried Interrupts */
            I2CControllerIntEnableEx(object->baseAddr, regVal);
            /* Start the I2C transfer in controller transmit mode */
            I2CControllerStart(object->baseAddr);

        }
        else
        {
            /* Write count zero */
            /* Specify the number of bytes to read */
            I2CSetDataCount(object->baseAddr, object->readCountIdx);

            /*
             * Start the I2C transfer in controller receive mode,
             * and automatically send stop when done
             */
            regVal = I2C_CFG_MST_RX | I2C_CFG_STOP | xsa;
            if (    (object->bitRate == I2C_1P0MHZ) ||
                    (object->bitRate == I2C_3P4MHZ))
            {
                regVal |= I2C_CFG_HS_MOD;
            }
            I2CControllerControl(object->baseAddr, regVal);

            /* Enable RX interrupts */
            I2CControllerIntEnableEx(   object->baseAddr,
                                        I2C_INT_RECV_READY          |
                                        I2C_INT_ADRR_READY_ACESS    |
                                        I2C_INT_NO_ACK              |
                                        I2C_INT_ARBITRATION_LOST);
            /* Send start bit */
            I2CControllerStart(object->baseAddr);
        }
    }
    /* Target Mode */
    else
    {
        status = I2C_STS_ERR;
    }

    return status;
}

/** ============================================================================
* Description:      Completes current transaction; increments current
*                   transaction count if more transactions available in current
*                   message or finishes the transfer process.
*
* Input Parameters: handle(I2CLLD_Handle), xferStatus(int32_t)
* Return Values:    NONE
** ========================================================================== */

static void I2C_lld_completeCurrTransfer(   I2CLLD_Handle handle,
                                            int32_t xferStatus)
{
    I2CLLD_Object *object = (I2CLLD_Object*)handle;
    /* Input parameter validation */
    if (handle != NULL)
    {
        if(object->currentMsg != NULL)
        {
            if( (xferStatus == I2C_STS_SUCCESS) &&
                (object->currentTxnCount < (object->currentMsg->txnCount - 1U)))
            {
                /* Last transaction completed successfully,
                 * Other thansactions available */
                /* Increment Current Transaction Count */
                object->currentTxnCount++;
                /* Initiate the next transaction */
                (void)I2C_lld_primeTransferIntr(handle);
            }
            else
            {
                /* No other thansactions available */
                /* Clear the data Array in case transaction was of type mem */
                object->dataArray = (uint8_t*)NULL;
                /* Call Transfer Complete Callback */
                object->transferCompleteCallback(handle, object->currentMsg, xferStatus);
                /* No other transactions need to occur */
                object->currentMsg = NULL;
                /* Change I2C state back to Idle */
                object->state = I2C_STATE_IDLE;
            }
        }
    }
}

/** ============================================================================
* Description:      Completes current target transaction. Clears stored current
*                   Target Transaction.
*
* Input Parameters: handle(I2CLLD_Handle), xferStatus(int32_t)
* Return Values:    NONE
** ========================================================================== */

static void I2C_lld_completeCurrTargetTransfer( I2CLLD_Handle handle,
                                                int32_t xferStatus)
{
    if(handle != NULL)
    {
        /* Get Pointer to driver Object */
        I2CLLD_Object *object = (I2CLLD_Object*)handle;
        /* Callback to Application */
        object->targetTransferCompleteCallback( (void *)object,
                                                object->currentTargetTransaction,
                                                xferStatus);
        /* No other transactions need to occur */
        object->currentTargetTransaction = NULL;
        /* Change driver state back to IDLE */
        object->state = I2C_STATE_IDLE;
    }
}

static int32_t I2C_lld_resetCtrl(I2CLLD_Handle handle)
{
    int32_t status = I2C_STS_SUCCESS;

    if(handle != NULL)
    {
        status = I2C_lld_ctrlInit(handle);
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

static int32_t I2C_lld_ctrlInit(I2CLLD_Handle handle)
{
    I2CLLD_Object       *object = NULL;
    uint32_t            delay = 50U;
    uint32_t            outputClk;
    uint32_t            internalClk;
    uint32_t            regVal;
    int32_t             retVal = I2C_STS_SUCCESS;

    /* Get the pointer to the object */
    object = (I2CLLD_Object *)handle;

    /* Put i2c in reset/disabled state */
    I2CControllerDisable(object->baseAddr);
    /* Do a software reset */
    I2CSoftReset(object->baseAddr);
    /* Enable i2c module */
    I2CControllerEnable(object->baseAddr);

    /* Wait for the reset to get complete  -- constant delay - 50ms */
    while ((I2CSystemStatusGet(object->baseAddr) == 0U) && (delay != 0U))
    {
        delay--;
        object->Clock_uSleep(1000);
    }

    if (delay == 0U)
    {
        /* Reset has failed, return!!! */
        retVal = I2C_STS_ERR;
    }
    else
    {
        /* Put i2c in reset/disabled state */
        I2CControllerDisable(object->baseAddr);
        /* Configure i2c bus speed */
        switch(object->bitRate)
        {
            case I2C_100KHZ:
            {
                outputClk = 100000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_4MHZ;
                break;
            }
            case I2C_400KHZ:
            {
                outputClk = 400000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_12MHZ;
                break;
            }
            case I2C_1P0MHZ:
            {
                outputClk = 1000000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_12MHZ;
                break;
            }
           case I2C_3P4MHZ:
           {
                outputClk = 3400000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_12MHZ;
                break;
           }

           default:
           {
                /* Default case force it to 100 KHZ bit rate */
                outputClk = 100000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_4MHZ;
                break;
           }

        }

        /* Set the I2C configuration */
        I2CControllerInitExpClk(    object->baseAddr, object->funcClk,
                                    internalClk, outputClk);
        /**
         * Configure I2C_SYSC params
         * Disable auto idle mode
         * Both OCP and systen clock cut off
         * Wake up mechanism disabled
         * No idle mode selected
         */
        regVal = (CSL_I2C_SYSC_AUTOIDLE_MASK) &
                (CSL_I2C_SYSC_AUTOIDLE_DISABLE << CSL_I2C_SYSC_AUTOIDLE_SHIFT);
        regVal |= (CSL_I2C_SYSC_IDLEMODE_MASK) &
                (CSL_I2C_SYSC_IDLEMODE_NOIDLE << CSL_I2C_SYSC_IDLEMODE_SHIFT);
        regVal |= (CSL_I2C_SYSC_ENAWAKEUP_MASK) &
                (CSL_I2C_SYSC_ENAWAKEUP_DISABLE << CSL_I2C_SYSC_ENAWAKEUP_SHIFT);
        regVal |= (CSL_I2C_SYSC_CLKACTIVITY_MASK) &
                ((uint32_t)CSL_I2C_SYSC_CLKACTIVITY_BOOTHOFF << (uint32_t)CSL_I2C_SYSC_CLKACTIVITY_SHIFT);

        I2CSyscInit(object->baseAddr, regVal);

        /* Configure I2C_CON params */
        regVal =    (I2C_CON_OPMODE_MASK) &
                    (I2C_CON_OPMODE_FSI2C << I2C_CON_OPMODE_SHIFT);
        regVal |=   (I2C_CON_STB_MASK) &
                    (I2C_CON_STB_NORMAL << I2C_CON_STB_SHIFT);

        I2CConfig(object->baseAddr, regVal);

        /* Take the I2C module out of reset */
        I2CControllerEnable(object->baseAddr);
        /* Enable free run mode */
        I2CControllerEnableFreeRun(object->baseAddr);
    }

    /* Clear interrupt status register */
    I2CControllerIntClearEx(object->baseAddr, I2C_INT_ALL);

    return retVal;
}

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
 *  \file   i2c_v1_lld.c
 *
 *  \brief  File containing I2C LLD Driver APIs implementation for V1.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/i2c/v1/lld/i2c_lld.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \anchor   I2C_ControllerControl
 * \name MACROS that can be passed to I2CControllerControl API as cmd to
 * configure mode of operation of I2C
 * @{
 */

#define I2C_CFG_MASK_TX                      (CSL_I2C_ICMDR_TRX_MASK)
#define I2C_CFG_MASK_RX                      (CSL_I2C_ICMDR_TRX_MASK)
#define I2C_CFG_MASK_STOP                    (CSL_I2C_ICMDR_STP_MASK)
#define I2C_CFG_MASK_START                   (CSL_I2C_ICMDR_STT_MASK)
#define I2C_CFG_MASK_RUN_FREE                (CSL_I2C_ICMDR_FREE_MASK)
#define I2C_CFG_MASK_REPEAT_MODE             (CSL_I2C_ICMDR_RM_MASK)
#define I2C_CFG_MASK_LOOP_BACK               (CSL_I2C_ICMDR_DLB_MASK)
#define I2C_CFG_MASK_XA                      (CSL_I2C_ICMDR_XA_MASK)


#define I2C_CFG_CMD_TX                       (CSL_I2C_ICMDR_TRX_MASK)
#define I2C_CFG_CMD_RX                       (0U)
#define I2C_CFG_CMD_STOP                     (CSL_I2C_ICMDR_STP_MASK)
#define I2C_CFG_CMD_START                    (CSL_I2C_ICMDR_STT_MASK)
#define I2C_CFG_CMD_RUN_FREE_ON              (CSL_I2C_ICMDR_FREE_MASK)
#define I2C_CFG_CMD_RUN_FREE_OFF             (0U)
#define I2C_CFG_CMD_REPEAT_MODE_ON           (CSL_I2C_ICMDR_RM_MASK)
#define I2C_CFG_CMD_REPEAT_MODE_OFF          (0U)
#define I2C_CFG_CMD_LOOP_BACK_ON             (CSL_I2C_ICMDR_DLB_MASK)
#define I2C_CFG_CMD_LOOP_BACK_OFF            (0U)
#define I2C_CFG_CMD_10BIT_ADDRESS            (I2C_CFG_MASK_XA)
#define I2C_CFG_CMD_7BIT_ADDRESS             (0U)

/** @} */


/**
 * \anchor   I2C_ControllerIntEnableEx
 * \name MACROS that can be passed to I2CControllerIntStatusEx and I2CControllerIntClearEx
 * APIs as int status flag to check and clear interrupt status
 * @{
 */

#define I2C_INT_ARBITRATION_LOST            (CSL_I2C_ICSTR_AL_MASK)
#define I2C_INT_NO_ACK                      (CSL_I2C_ICSTR_NACK_MASK)
#define I2C_INT_ADRR_READY_ACESS            (CSL_I2C_ICSTR_ARDY_MASK)
#define I2C_INT_RECV_READY                  (CSL_I2C_ICSTR_ICRRDY_MASK)
#define I2C_INT_TRANSMIT_READY              (CSL_I2C_ICSTR_ICXRDY_MASK)
#define I2C_INT_STOP_CONDITION              (CSL_I2C_ICSTR_SCD_MASK)
#define I2C_INT_ADRR_ZERO                   (CSL_I2C_ICSTR_AD0_MASK)
#define I2C_INT_ADRR_TARGET                 (CSL_I2C_ICSTR_AAS_MASK)
#define I2C_INT_TRANSMIT_UNDER_FLOW         (CSL_I2C_ICSTR_XSMT_MASK)
#define I2C_INT_RECV_OVER_RUN               (CSL_I2C_ICSTR_RSFULL_MASK)
#define I2C_INT_BUS_BUSY                    (CSL_I2C_ICSTR_BB_MASK)
#define I2C_INT_NO_ACK_SENT                 (CSL_I2C_ICSTR_NACKSNT_MASK)
#define I2C_INT_TARGET_DIRECTION            (CSL_I2C_ICSTR_SDIR_MASK)

#define I2C_ALL_INTS                        (   I2C_INT_ARBITRATION_LOST        |   \
                                                I2C_INT_NO_ACK                  |   \
                                                I2C_INT_ADRR_READY_ACESS        |   \
                                                I2C_INT_RECV_READY              |   \
                                                I2C_INT_TRANSMIT_READY          |   \
                                                I2C_INT_STOP_CONDITION          |   \
                                                I2C_INT_ADRR_ZERO               |   \
                                                I2C_INT_ADRR_TARGET             |   \
                                                I2C_INT_TRANSMIT_UNDER_FLOW     |   \
                                                I2C_INT_RECV_OVER_RUN           |   \
                                                I2C_INT_BUS_BUSY                |   \
                                                I2C_INT_NO_ACK_SENT             |   \
                                                I2C_INT_TARGET_DIRECTION            \
                                            )

/** @} */


/**
 * \anchor   I2C_ControllerInterruptFlag
 * \name MACROS that can be passed to I2CControllerIntEnableEx and I2CControllerIntDisableEx
 * APIs as intFlag to enable or disable interrupts
 * @{
 */

#define I2C_INT_MASK_ARBITRATION_LOST       (CSL_I2C_ICIMR_AL_MASK)
#define I2C_INT_MASK_NO_ACK                 (CSL_I2C_ICIMR_NACK_MASK)
#define I2C_INT_MASK_ADRR_READY_ACESS       (CSL_I2C_ICIMR_ARDY_MASK)
#define I2C_INT_MASK_RECV_READY             (CSL_I2C_ICIMR_ICRRDY_MASK)
#define I2C_INT_MASK_TRANSMIT_READY         (CSL_I2C_ICIMR_ICXRDY_MASK)
#define I2C_INT_MASK_STOP_CONDITION         (CSL_I2C_ICIMR_SCD_MASK)
#define I2C_INT_MASK_ADRR_TARGET            (CSL_I2C_ICIMR_AAS_MASK)

#define I2C_ALL_INTS_MASK                   (   I2C_INT_MASK_ARBITRATION_LOST   |   \
                                                I2C_INT_MASK_NO_ACK             |   \
                                                I2C_INT_MASK_ADRR_READY_ACESS   |   \
                                                I2C_INT_MASK_RECV_READY         |   \
                                                I2C_INT_MASK_TRANSMIT_READY     |   \
                                                I2C_INT_MASK_STOP_CONDITION     |   \
                                                I2C_INT_MASK_ADRR_TARGET            \
                                            )

/** @} */


/**
 * \anchor   I2C_interruptVector
 * \name MACROS used to configure the I2C interrupt vector Code
 * @{
 */

#define I2C_IVR_INTCODE_MASK                (CSL_I2C_ICIVR_INTCODE_MASK)
#define I2C_IVR_INTCODE_AL                  (CSL_I2C_ICIVR_INTCODE_AL)
#define I2C_IVR_INTCODE_NACK                (CSL_I2C_ICIVR_INTCODE_NACK)
#define I2C_IVR_INTCODE_ARDY                (CSL_I2C_ICIVR_INTCODE_RAR)
#define I2C_IVR_INTCODE_RRDY                (CSL_I2C_ICIVR_INTCODE_RDR)
#define I2C_IVR_INTCODE_XRDY                (CSL_I2C_ICIVR_INTCODE_TDR)
#define I2C_IVR_INTCODE_SCD                 (CSL_I2C_ICIVR_INTCODE_SCD)
#define I2C_IVR_INTCODE_AAS                 (CSL_I2C_ICIVR_INTCODE_AAS)

/** @} */

#define I2C_MODULE_INTERNAL_CLK_8MHZ        (8000000U)
#define I2C_MODULE_INTERNAL_CLK_12MHZ       (12000000U)

#define I2C_DELAY_MED                       ((uint32_t) 10000U)
#define I2C_DELAY_BIG                       ((uint32_t) 30000U)

#define I2C_CONTROLLER_ERR_NONE             (0)

#define I2C_BUS_BUSY_TIMEOUT_IN_US          (500U)

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
static void I2CTargetEnable(uint32_t baseAddr);
static void I2CControllerDisable(uint32_t baseAddr);
static void I2CControllerControl(uint32_t baseAddr, uint32_t ctrlMask,
                                 uint32_t ctrlCmds);
static void I2CModeControl(uint32_t baseAddr, uint32_t ctrlMask,
                           uint32_t ctrlCmds);
static void I2CControllerStart(uint32_t baseAddr);
static void I2CControllerStop(uint32_t baseAddr);
static void I2CControllerIntEnableEx(uint32_t baseAddr, uint32_t intFlag);
static void I2CControllerIntDisableEx(uint32_t baseAddr, uint32_t intFlag);
static void I2CControllerIntClearEx(uint32_t baseAddr, uint32_t intFlag);
static void I2CControllerTargetAddrSet(uint32_t baseAddr, uint32_t targetAddr);
static void I2CSetDataCount(uint32_t baseAddr, uint32_t count);
static void I2CControllerDataPut(uint32_t baseAddr, uint8_t data);
static void I2COwnAddressSet(uint32_t baseAddr, uint32_t ownAddr);
static uint32_t I2CControllerIntStatusEx(uint32_t baseAddr, uint32_t intFlag);
static uint32_t I2CControllerErr(uint32_t baseAddr);
static uint32_t I2CIntVectGet(uint32_t baseAddr);
static int32_t I2CControllerBusBusy(uint32_t baseAddr);
static uint8_t I2CControllerDataGet(uint32_t baseAddr);
static int32_t I2C_lld_waitForBb(I2CLLD_Handle handle, uint32_t timeout);
static void I2C_lld_completeCurrTransfer(I2CLLD_Handle handle,
                                         int32_t xferStatus);
static int32_t I2C_lld_primeTransferIntr(I2CLLD_Handle handle,
                                        I2CLLD_Message *msg);
static int32_t I2C_lld_primeTransferPoll(I2CLLD_Handle handle,
                                        I2CLLD_Message *msg);
static int32_t I2C_lld_transferInit(I2CLLD_Handle handle, I2CLLD_Message *msg);

static int32_t lld_check_param(bool expression);

/* ========================================================================== */
/*                       API Function Definitions                             */
/* ========================================================================== */

int32_t I2C_lld_init(I2CLLD_Handle handle)
{
    int32_t                 status = I2C_STS_SUCCESS;
    I2CLLD_Object           *obj = NULL;
    uint32_t                outputClk;
    uint32_t                internalClk;

    obj = (I2CLLD_Object *) handle;

    if(obj != NULL)
    {
        if(obj->state != I2C_STATE_RESET)
        {
            status = I2C_STS_ERR;
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    if(I2C_STS_SUCCESS == status)
    {
        /* check if the I2C base address is valid or not */
        status = lld_check_param((bool)IS_I2C_BASE_ADDR_VALID(obj->baseAddr));
    }

    if(I2C_STS_SUCCESS == status)
    {
        /* Specify the busy state for this I2C peripheral */
        obj->state = I2C_STATE_BUSY;

        /* Clear the head pointer */
        obj->currentMsg = NULL;
        obj->args = NULL_PTR;

        /* Put i2c in reset/disabled state */
        I2CControllerDisable(obj->baseAddr);

        /* Extract bit rate from the input parameter */
        switch(obj->bitRate)
        {
            case I2C_100KHZ:
            {
                outputClk = 100000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_8MHZ;
                break;
            }

            case I2C_400KHZ:
            {
                outputClk = (uint32_t)400000U;
                internalClk = (uint32_t)I2C_MODULE_INTERNAL_CLK_12MHZ;
                break;
            }

            default:
            {
                /* Default case force it to 100 KHZ bit rate */
                outputClk = 100000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_8MHZ;
                break;
            }
        }

        /* Set the I2C configuration */
        I2CControllerInitExpClk(obj->baseAddr,
                          obj->funcClk,
                          internalClk,
                          outputClk);

        /* Clear any pending interrupts */
        I2CControllerIntClearEx(obj->baseAddr, I2C_ALL_INTS);

        /* Mask off all interrupts */
        I2CControllerIntDisableEx(obj->baseAddr, I2C_ALL_INTS);

        /* Enable the I2C Controller for operation */
        I2CControllerEnable(obj->baseAddr);

        /* Set own Address */
        I2COwnAddressSet(obj->baseAddr, obj->ownTargetAddr);

        /* Specify the idle state for this I2C peripheral */
        obj->state = I2C_STATE_IDLE;

    }

    return status;
}

int32_t I2C_lld_deInit(I2CLLD_Handle handle)
{
    int32_t                 status = I2C_STS_SUCCESS;
    I2CLLD_Object           *object = NULL;

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;

        /* Check to see if a I2C transaction is in progress */
        if (object->currentMsg == NULL)
        {
            /* Mask I2C interrupts */
            I2CControllerIntDisableEx(object->baseAddr, I2C_ALL_INTS_MASK);

            /* Disable the I2C Controller */
            I2CControllerDisable(object->baseAddr);

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
    int32_t                 status = I2C_STS_SUCCESS;

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

            status = I2C_lld_primeTransferIntr(handle, &object->i2cMsg);
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

int32_t I2C_lld_target_writeIntr(I2CLLD_Handle handle,
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
            object->state = I2C_STATE_BUSY;

            (void)I2C_lld_Transaction_init(&object->i2ctxn);
            object->i2ctxn.writeBuf      = extendedParams->buffer;
            object->i2ctxn.writeCount    = extendedParams->size;

            (void)I2C_lld_Message_init(&object->i2cMsg);
            object->i2cMsg.txn              = &object->i2ctxn;
            object->i2cMsg.txnCount         = 1U;
            object->i2cMsg.expandSA         = extendedParams->expandSA;
            object->i2cMsg.controllerMode   = false;

            object->currentMsg = &object->i2cMsg;
            object->currentTxnCount = 0U;

            status = I2C_lld_primeTransferIntr(handle, &object->i2cMsg);
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

            status = I2C_lld_primeTransferPoll(handle, &object->i2cMsg);
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

int32_t I2C_lld_target_write(I2CLLD_Handle handle,
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
            object->i2cMsg.expandSA         = extendedParams->expandSA;
            object->i2cMsg.timeout          = timeout;
            object->i2cMsg.controllerMode   = false;

            object->currentMsg = &object->i2cMsg;
            object->currentTxnCount = 0U;

            status = I2C_lld_primeTransferPoll(handle, &object->i2cMsg);
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

            status = I2C_lld_primeTransferIntr(handle, &object->i2cMsg);
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

int32_t I2C_lld_target_readIntr(I2CLLD_Handle handle,
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
            object->i2cMsg.expandSA         = extendedParams->expandSA;
            object->i2cMsg.controllerMode   = false;

            object->currentMsg = &object->i2cMsg;
            object->currentTxnCount = 0U;

            status = I2C_lld_primeTransferIntr(handle, &object->i2cMsg);
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

            status = I2C_lld_primeTransferPoll(handle, &object->i2cMsg);
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

int32_t I2C_lld_target_read(I2CLLD_Handle handle,
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
            object->i2cMsg.expandSA         = extendedParams->expandSA;
            object->i2cMsg.timeout          = timeout;
            object->i2cMsg.controllerMode   = false;

            object->currentMsg = &object->i2cMsg;
            object->currentTxnCount = 0U;

            status = I2C_lld_primeTransferPoll(handle, &object->i2cMsg);
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

int32_t I2C_lld_mem_writeIntr(I2CLLD_Handle handle,
                              I2C_Memory_ExtendedParams * mem_extendedParams)
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

                status = I2C_lld_primeTransferIntr(handle, &object->i2cMsg);
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

int32_t I2C_lld_mem_write(  I2CLLD_Handle handle,
                            I2C_Memory_ExtendedParams * mem_extendedParams,
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

                status = I2C_lld_primeTransferPoll(handle, &object->i2cMsg);
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

int32_t I2C_lld_mem_readIntr(I2CLLD_Handle handle,
                              I2C_Memory_ExtendedParams * mem_extendedParams)
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

                status = I2C_lld_primeTransferIntr(handle, &object->i2cMsg);
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
                            I2C_Memory_ExtendedParams * mem_extendedParams,
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

                status = I2C_lld_primeTransferPoll(handle, &object->i2cMsg);
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

int32_t I2C_lld_transferIntr(I2CLLD_Handle handle, I2CLLD_Message * msg)
{
    int32_t status;

    if((handle != NULL) && (msg != NULL))
    {
        status = I2C_lld_transferInit(handle, msg);
        if (status == I2C_STS_SUCCESS)
        {
            status = I2C_lld_primeTransferIntr(handle, msg);
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
            status = I2C_lld_primeTransferPoll(handle, msg);
        }
    }
    else
    {
        status = I2C_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t I2C_lld_targetTransferIntr( I2CLLD_Handle handle,
                                    I2CLLD_targetTransaction * txn)
{
    int32_t                 retVal = I2C_STS_SUCCESS;
    I2CLLD_Object           *object = NULL;
    uint32_t                xsa;

    if ((handle != NULL) && (txn != NULL))
    {
        /* Get the pointer to the object */
        object = (I2CLLD_Object*)handle;
        if (txn->timeout == 0U)
        {
            /* timeout cannot be NO_WAIT, set it to default value */
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
            /* TODO: This should happen automically. */
            object->state = I2C_STATE_BUSY;

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
        /* Store the new internal counters and pointers */
        object->currentTargetTransaction = txn;

        object->writeBufIdx = (uint8_t*)txn->writeBuf;
        object->writeCountIdx = (uint32_t)txn->writeCount;

        object->readBufIdx = txn->readBuf;
        object->readCountIdx = (uint32_t)txn->readCount;

        if (object->currentTargetTransaction->expandSA == true)
        {
            /* enable the 10-bit address mode */
            xsa = I2C_CFG_CMD_10BIT_ADDRESS;
        }
        else
        {
            /* enable the 7-bit address mode */
            xsa = I2C_CFG_CMD_7BIT_ADDRESS;
        }

        /* clear all interrupts */
        I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

        /* Currently target mode is supported only when interrupt is enabled */
        /*
            * Enable Address as Target interrupt which is the first interrupt
            * received in target mode
            */
        I2CControllerIntEnableEx(object->baseAddr, I2C_INT_MASK_ADRR_TARGET);
        /* Configure data buffer length to 0 as the actual number of bytes to
            transmit/receive is dependant on external controller. */
        I2CSetDataCount(object->baseAddr, 0U);

        /* Start the I2C transfer in target mode */
        I2CTargetEnable(object->baseAddr);

        /* set to controller receiver mode */
        I2CModeControl(object->baseAddr, I2C_CFG_MASK_XA, xsa);
    }
    return (retVal);
}

int32_t I2C_lld_probe(I2CLLD_Handle handle, uint32_t targetAddr)
{
    int32_t                 retVal = I2C_STS_ERR;
    uint32_t                regVal;
    I2CLLD_Object           *object  = NULL;

    /* Input parameter validation */
    if (handle != NULL)
    {
        object  = (I2CLLD_Object*)handle;

        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* TODO: This should happen automically. */
            object->state = I2C_STATE_BUSY;

            /* Disable interrupts first */
            regVal = I2CControllerIntStatusEx(object->baseAddr, I2C_ALL_INTS);

            I2CControllerIntDisableEx(object->baseAddr, I2C_ALL_INTS);

            /* wait until bus not busy */
            if (I2C_lld_waitForBb(handle, I2C_DELAY_MED) != I2C_STS_SUCCESS)
            {
                retVal = I2C_STS_ERR;
            }
            else
            {
                /* set target address */
                I2CControllerTargetAddrSet( object->baseAddr,
                                            (uint32_t)targetAddr);
                /* try to write one byte */
                I2CControllerDataPut(object->baseAddr, (uint8_t) 0U);
                I2CSetDataCount(object->baseAddr, (uint32_t) 1U);

                /* stop bit needed here */
                I2CControllerControl(object->baseAddr,I2C_CFG_MASK_STOP,(
                        I2C_CFG_CMD_TX | I2C_CFG_CMD_START | I2C_CFG_CMD_STOP));

                /* enough delay for the NACK bit set */
                object->Clock_uSleep(I2C_DELAY_BIG);

                if (0U == I2CControllerIntStatusEx( object->baseAddr,
                                                    I2C_INT_NO_ACK))
                {
                    /* success case */
                    retVal = I2C_STS_SUCCESS;
                }
                else
                {
                    /* Clear sources*/
                    I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);
                    /* finish up xfer */
                    I2CControllerStop(object->baseAddr);
                    (void)I2C_lld_waitForBb(handle, I2C_DELAY_MED);
                    /* Error case */
                    retVal = I2C_STS_ERR;
                }
                I2CSetDataCount(object->baseAddr, 0U);
                I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);
            }
            /* wait for bus free */
            if (I2C_lld_waitForBb(handle, I2C_DELAY_MED) != I2C_STS_SUCCESS)
            {
                retVal = I2C_STS_ERR;
            }
            /* Enable interrupts now */
            I2CControllerIntEnableEx(object->baseAddr, regVal);
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

    return (retVal);
}

int32_t I2C_lld_setBusFrequency(I2CLLD_Handle handle, uint32_t busFrequency)
{
    int32_t                 retVal      = I2C_STS_SUCCESS;
    I2CLLD_Object           *object     = NULL;
    uint32_t                outputClk   = 0U;
    uint32_t                internalClk = 0U;

    /* Input parameter validation */
    if(NULL != handle)
    {
        /* Get the pointer to the object */
        object  = (I2CLLD_Object*)handle;

        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* TODO: This should happen automically. */
            object->state = I2C_STATE_BUSY;
            /* Put i2c in reset/disabled state */
            I2CControllerDisable(object->baseAddr);
            /* Extract bit rate from the input parameter */
            switch(busFrequency)
            {
                case (uint32_t)I2C_100KHZ:
                {
                    outputClk = 100000U;
                    internalClk = I2C_MODULE_INTERNAL_CLK_8MHZ;
                    break;
                }
                case (uint32_t)I2C_400KHZ:
                {
                    outputClk = 400000U;
                    internalClk = I2C_MODULE_INTERNAL_CLK_12MHZ;
                    break;
                }
                default:
                {
                    outputClk = 100000U;
                    internalClk = I2C_MODULE_INTERNAL_CLK_8MHZ;
                    break;
                }
            }

            /* Set the I2C configuration */
            I2CControllerInitExpClk(object->baseAddr, object->funcClk,
                                    internalClk, outputClk);
            /* Clear any pending interrupts */
            I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);
            /* Mask off all interrupts */
            I2CControllerIntDisableEx(object->baseAddr, I2C_ALL_INTS);
            I2COwnAddressSet(object->baseAddr, object->ownTargetAddr);
            /* Enable the I2C Controller for operation */

            I2CControllerEnable(object->baseAddr);

            retVal = I2C_STS_SUCCESS;
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

    return (retVal);
}

/* ========================================================================== */
/*                     API ISR Function Definitions                           */
/* ========================================================================== */

/*
 *  Hwi interrupt handler to service the I2C peripheral in controller mode
 */
void I2C_lld_controllerIsr(void* args)
{
    I2CLLD_Handle           handle = (I2CLLD_Object *)NULL_PTR;
    int32_t                 xferStatus = I2C_STS_SUCCESS;
    uint32_t                errStatus;
    uint32_t                stopCondition;
    uint32_t                xsa;

    handle = (I2CLLD_Handle)args;

    /* Input parameter validation */
    if (args != NULL_PTR)
    {
        /* Check for I2C Errors */
        errStatus = I2CControllerErr(handle->baseAddr);
        if ((errStatus & I2C_INT_STOP_CONDITION) != (uint32_t)0U)
        {
            if ((handle->writeCountIdx == 0U) && (handle->readCountIdx == 0U))
            {
                /* End of transfer stop condition, not an error */
                stopCondition = I2C_INT_STOP_CONDITION;
                errStatus &= (uint32_t)(~stopCondition);
            }
        }

        if ((errStatus == (uint32_t)I2C_CONTROLLER_ERR_NONE) ||
            (handle->state == I2C_STATE_ERROR))
        {

            /* No errors, now check what we need to do next */
            switch (handle->state)
            {
                /*
                * ERROR case is OK because if an Error is detected, a STOP bit is
                * sent; which in turn will call another interrupt. This interrupt
                * call will then post the transferComplete semaphore to unblock the
                * I2C_transfer function
                */
                case I2C_STATE_ERROR:
                case I2C_STATE_BUSY:
                    if(handle->state == I2C_STATE_ERROR)
                    {
                        if (handle->intStatusErr & I2C_INT_NO_ACK)
                        {
                            xferStatus = I2C_STS_ERR_NO_ACK;
                        }
                        else if (handle->intStatusErr & I2C_INT_ARBITRATION_LOST)
                        {
                            xferStatus = I2C_STS_ERR_ARBITRATION_LOST;
                        }
                        /* Mask I2C interrupts */
                        I2CControllerIntDisableEx(handle->baseAddr, I2C_ALL_INTS_MASK);
                        /* Disable the I2C Controller */
                        I2CControllerDisable(handle->baseAddr);
                        /* Enable the I2C Controller for operation */
                        I2CControllerEnable(handle->baseAddr);
                    }
                    else
                    {
                        xferStatus = I2C_STS_SUCCESS;
                    }

                    /* Clear STOP condition interrupt */
                    I2CControllerIntClearEx(handle->baseAddr,
                                        I2C_INT_STOP_CONDITION);

                    /* Disable STOP condition interrupt */
                    I2CControllerIntDisableEx(handle->baseAddr,
                                        I2C_INT_MASK_STOP_CONDITION);

                    /* callback to application or post Semaphore to unblock transfer fxn */
                    I2C_lld_completeCurrTransfer(handle,xferStatus);
                break;

                case I2C_WRITE_STATE:
                    /* Check if data needs to be sent */
                    if (handle->writeCountIdx != 0U)
                    {
                        /* Write data into transmit FIFO */
                        I2CControllerDataPut(handle->baseAddr,
                                         *(handle->writeBufIdx));
                        (handle->writeBufIdx)++;
                        handle->writeCountIdx--;

                        if(handle->memTxnActive)
                        {
                            if((handle->i2ctxn.writeCount - handle->memAddrSize) == (handle->writeCountIdx))
                            {
                                handle->writeBufIdx = handle->dataArray;
                            }
                        }

                        if (handle->writeCountIdx == 0U)
                        {
                            /* End of write, disable TX ready interrupt */
                            I2CControllerIntDisableEx(handle->baseAddr,
                                                I2C_INT_MASK_TRANSMIT_READY);
                            if (handle->readCountIdx != 0U)
                            {
                                /* Get register access interrupt to start reading */
                                I2CControllerIntEnableEx(handle->baseAddr,
                                                    I2C_INT_MASK_ADRR_READY_ACESS);
                            }
                            else
                            {
                                /* Done with all transmissions, wait for stop condition */
                                handle->state = I2C_STATE_BUSY;
                            }
                        }
                    }

                    else
                    {
                        if ((handle->readCountIdx) != 0U)
                        {
                            /* Next state: Receive mode */
                            handle->state = I2C_READ_STATE;
                            I2CControllerIntDisableEx(handle->baseAddr,
                                                I2C_INT_MASK_ADRR_READY_ACESS);

                            /* Set number of bytes to receive */
                            I2CSetDataCount(handle->baseAddr, handle->readCountIdx);

                            /* Configure peripheral for I2C Receive mode with stop */
                            if (handle->currentMsg->expandSA == true)
                            {
                                /* enable the 10-bit address mode */
                                xsa = I2C_CFG_CMD_10BIT_ADDRESS;
                            }
                            else
                            {
                                /* enable the 7-bit address mode */
                                xsa = I2C_CFG_CMD_7BIT_ADDRESS;
                            }
                            I2CControllerControl(handle->baseAddr,
                                            I2C_CFG_MASK_RX | I2C_CFG_MASK_REPEAT_MODE | I2C_CFG_MASK_XA,
                                            I2C_CFG_CMD_RX | I2C_CFG_CMD_REPEAT_MODE_OFF | xsa);

                            /* Enable RX interrupt to handle data received */
                            I2CControllerIntEnableEx(handle->baseAddr, I2C_INT_MASK_RECV_READY);

                            /* Start I2C peripheral in RX mode */
                            I2CControllerStart(handle->baseAddr);
                        }

                        else
                        {

                        }
                    }
                    break;

                case I2C_READ_STATE:
                    /* Save the received data */
                    *(handle->readBufIdx) = I2CControllerDataGet(handle->baseAddr);

                    (handle->readBufIdx)++;
                    (handle->readCountIdx)--;

                    if (handle->readCountIdx == 0U) {
                        /* No more data to receive, Next state: Idle mode */
                        handle->state = I2C_STATE_BUSY;

                        /* Disable RX interrupt, next interrupt will be from STOP */
                        I2CControllerIntDisableEx(handle->baseAddr,
                                            I2C_INT_MASK_RECV_READY);
                        /* Send stop */
                        I2CControllerStop(handle->baseAddr);
                    }
                    break;

                default:
                    handle->state = I2C_STATE_ERROR;
                    break;
            }
        }
        else
        {
            /* Some sort of error happened! */
            handle->state = I2C_STATE_ERROR;
            /* Store the interrupt status */
            handle->intStatusErr = errStatus;

            if ((errStatus & I2C_INT_NO_ACK) != (uint32_t)0U)
            {
                xferStatus = I2C_STS_ERR_NO_ACK;
                I2CControllerIntClearEx(handle->baseAddr, I2C_INT_NO_ACK);
            }
            else if ((errStatus & I2C_INT_ARBITRATION_LOST) != (uint32_t)0U)
            {
                xferStatus = I2C_STS_ERR_ARBITRATION_LOST;
                I2CControllerIntClearEx(handle->baseAddr,I2C_INT_ARBITRATION_LOST);
            }
            else
            {
                xferStatus = I2C_STS_ERR;
            }

            /* Disable all interrupts */
            I2CControllerIntDisableEx(handle->baseAddr, I2C_ALL_INTS);
            /* Do not clear the status register */
            I2CControllerIntClearEx(handle->baseAddr, I2C_ALL_INTS);
            /* Only enable stop condition interrupt */
            I2CControllerIntEnableEx(handle->baseAddr, I2C_INT_MASK_STOP_CONDITION);
            /* Generate stop */
            I2CControllerStop(handle->baseAddr);
        }
    }
}

/*
 *  Hwi interrupt handler to service the I2C peripheral in target mode
 */
void I2C_lld_targetIsr(void* args)
{
    I2CLLD_Object           *object = NULL;
    I2CLLD_Handle           handle = NULL;
    uint32_t                intCode;
    uint32_t                intStat;

    /* Get the pointer to the object */
    handle = (I2CLLD_Handle)args;
    /* Get the pointer to the object */
    object = (I2CLLD_Object*)handle;

    if(args != NULL)
    {
        intCode = I2CIntVectGet(handle->baseAddr);
        intStat = I2CControllerIntStatusEx(handle->baseAddr, I2C_ALL_INTS);

        switch (intCode)
        {
            case I2C_IVR_INTCODE_AAS:
                if (handle->state == I2C_STATE_BUSY)
                {
                    /*
                    * This is the first transfer initiation from controller
                    * Update the state to transfer started and enable
                    * all the salve interrupts
                    */
                    handle->state = I2C_TARGET_XFER_STATE;
                    I2CControllerIntClearEx(handle->baseAddr, I2C_ALL_INTS);
                    I2CControllerIntEnableEx(handle->baseAddr,
                    I2C_INT_MASK_TRANSMIT_READY | I2C_INT_MASK_RECV_READY |
                    I2C_INT_MASK_ADRR_READY_ACESS | I2C_INT_MASK_ADRR_TARGET |
                    I2C_INT_MASK_NO_ACK | I2C_INT_MASK_STOP_CONDITION);

                    if (((handle->writeCountIdx) != 0U) &&
                        ((intStat & I2C_INT_TRANSMIT_READY) == I2C_INT_TRANSMIT_READY))
                    {
                        /* Target transmit mode, send data and clear the interrupt */
                        I2CControllerDataPut(handle->baseAddr, *(handle->writeBufIdx));
                        handle->writeCountIdx--;
                        handle->writeBufIdx++;
                    }

                }
                else if (object->state == I2C_TARGET_XFER_STATE)
                {
                    /*
                    * This is a restart condition, callback to application
                    * to restart read/write
                    */
                    object->currentMsg->txn[object->currentTxnCount].readCount -= object->readCountIdx;
                    object->currentMsg->txn[object->currentTxnCount].writeCount -= object->writeCountIdx;

                    handle->targetTransferCompleteCallback(handle,
                                                        &(handle->currentMsg->txn[object->currentTxnCount]),
                                                        I2C_STS_RESTART);

                    object->writeBufIdx = object->currentMsg->txn[object->currentTxnCount].writeBuf;
                    object->writeCountIdx = object->currentMsg->txn[object->currentTxnCount].writeCount;

                    object->readBufIdx = object->currentMsg->txn[object->currentTxnCount].readBuf;
                    object->readCountIdx = object->currentMsg->txn[object->currentTxnCount].readCount;

                    handle->state = I2C_TARGET_RESTART_STATE;

                    if (((handle->writeCountIdx) != 0U) &&
                        ((intStat & I2C_INT_TRANSMIT_READY) == I2C_INT_TRANSMIT_READY))
                    {
                        /* Target transmit mode with restart, send data and clear the interrupt */

                        handle->writeCountIdx--;
                        handle->writeBufIdx++;
                        I2CControllerDataPut(handle->baseAddr, *(handle->writeBufIdx));
                        handle->writeCountIdx--;
                        handle->writeBufIdx++;
                    }
                }
                else
                {
                    /* Control should not come here. Sphurious interrupt clear it. */
                }
                I2CControllerIntClearEx(handle->baseAddr, I2C_INT_ADRR_TARGET);
                break;

            case I2C_IVR_INTCODE_NACK:
                    object->currentMsg->txn[object->currentTxnCount].readCount -= object->readCountIdx;
                    object->currentMsg->txn[object->currentTxnCount].writeCount -= object->writeCountIdx;

                    handle->targetTransferCompleteCallback(handle,
                                                        &(handle->currentMsg->txn[object->currentTxnCount]),
                                                        I2C_STS_ERR_NO_ACK);

                I2CControllerIntDisableEx(handle->baseAddr, I2C_ALL_INTS_MASK);
                I2CControllerIntClearEx(handle->baseAddr, I2C_ALL_INTS);
                break;

            case I2C_IVR_INTCODE_ARDY:
                I2CControllerIntClearEx(handle->baseAddr, I2C_INT_ADRR_READY_ACESS);
                break;

            case I2C_IVR_INTCODE_RRDY:
                /* Read from Rx register only when current transaction is ongoing */
                if ((handle->readCountIdx) != (uint32_t)0U)
                {
                    *(handle->readBufIdx) = I2CControllerDataGet(handle->baseAddr);
                    handle->readBufIdx++;
                    handle->readCountIdx--;
                }
                else
                {
                    /* RX buffer full, drop the data received */
                    (void)I2CControllerDataGet(handle->baseAddr);
                }
                I2CControllerIntClearEx(handle->baseAddr, I2C_INT_RECV_READY);
                break;

            case I2C_IVR_INTCODE_XRDY:
                if ((handle->state == I2C_TARGET_XFER_STATE) || (handle->state == I2C_TARGET_RESTART_STATE))
                {
                    if (handle->writeCountIdx != 0U)
                    {
                        I2CControllerDataPut(handle->baseAddr, *(handle->writeBufIdx));
                        handle->writeCountIdx--;
                        handle->writeBufIdx++;
                    }
                    else
                    {
                        if ((object->currentMsg->txn[object->currentTxnCount].writeCount) != (uint32_t)0U)
                        {
                            /* TX buffer empty, send 0 */
                            I2CControllerDataPut(handle->baseAddr, 0U);
                        }
                    }
                }
                break;

            case I2C_IVR_INTCODE_SCD:
                    I2CControllerIntDisableEx(handle->baseAddr, I2C_ALL_INTS_MASK);
                    I2CControllerIntClearEx(handle->baseAddr, I2C_ALL_INTS);
                    object->currentMsg->txn[object->currentTxnCount].readCount -= object->readCountIdx;
                    object->currentMsg->txn[object->currentTxnCount].writeCount -= object->writeCountIdx;

                    if(object->currentTxnCount < (object->currentMsg->txnCount - 1U))
                    {
                        object->currentTxnCount++;

                        (void)I2C_lld_primeTransferIntr(handle, object->currentMsg);

                        object->state = I2C_STATE_BUSY;
                    }
                    else
                    {
                        handle->targetTransferCompleteCallback(handle,
                                                            &(handle->currentMsg->txn[object->currentTxnCount]),
                                                            I2C_STS_SUCCESS);
                        object->currentMsg = NULL;
                        handle->state = I2C_STATE_IDLE;
                    }
                break;

            default:
                break;
        }
    }
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
    uint32_t divider;
    uint32_t diff;

    /* Set the prescalar value */
    prescalar = (sysClk / internalClk) - (uint32_t)1U;
    i2cRegs->ICPSC = prescalar;

    if (prescalar == 0U)
    {
        diff = 7U;
    }
    else if (prescalar == 1U)
    {
        diff = 6U;
    }
    else
    {
        diff = 5U;
    }

    /* Set the CLKL and CLKH values */
    divider = internalClk / outputClk;
    if((divider % 2U) == 0U)
    {
        divider = divider / 2U;
        i2cRegs->ICCLKL = divider - diff;
        i2cRegs->ICCLKH = divider - diff;
    }
    else
    {
        i2cRegs->ICCLKL = ((divider / 2U) - diff);
        i2cRegs->ICCLKH = ((divider / 2U) - diff) + 1U;
    }

}

static void I2CControllerEnable(uint32_t baseAddr)
{
    /* Bring the I2C module out of reset */
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    /* Enable controller mode */
    i2cRegs->ICMDR = CSL_I2C_ICMDR_MST_MASK | CSL_I2C_ICMDR_FREE_MASK;
    /* Bring I2C out of reset */
    i2cRegs->ICMDR |= CSL_I2C_ICMDR_IRS_MASK;
}

static void I2CTargetEnable(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    /* Enable controller mode */
    i2cRegs->ICMDR = CSL_I2C_ICMDR_FREE_MASK;
    /* Bring I2C out of reset */
    i2cRegs->ICMDR |= CSL_I2C_ICMDR_IRS_MASK;
}

static void I2CControllerDisable(uint32_t baseAddr)
{
    /* Put I2C module in reset */
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->ICMDR &= ~CSL_I2C_ICMDR_IRS_MASK;
}

static void I2CControllerControl(   uint32_t baseAddr, uint32_t ctrlMask,
                                    uint32_t ctrlCmds)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    uint32_t i2cMdr = i2cRegs->ICMDR;
    i2cMdr &= ~ctrlMask;
    i2cMdr |= (ctrlCmds | CSL_I2C_ICMDR_MST_MASK);
    i2cRegs->ICMDR = i2cMdr;
}

static void I2CModeControl( uint32_t baseAddr, uint32_t ctrlMask,
                            uint32_t ctrlCmds)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    uint32_t i2cMdr = i2cRegs->ICMDR;
    i2cMdr &= ~ctrlMask;
    i2cMdr |= ctrlCmds;
    i2cRegs->ICMDR = i2cMdr;
}

static void I2CControllerStart(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->ICMDR |= (CSL_I2C_ICMDR_MST_MASK | CSL_I2C_ICMDR_STT_MASK);
}

static void I2CControllerStop(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->ICMDR |= (CSL_I2C_ICMDR_MST_MASK | CSL_I2C_ICMDR_STP_MASK);
}

static void I2CControllerIntEnableEx(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->ICIMR |= intFlag;
}

static void I2CControllerIntDisableEx(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->ICIMR &= ~intFlag;
}

static void I2CControllerIntClearEx(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->ICSTR = intFlag;
}

static void I2CControllerTargetAddrSet(uint32_t baseAddr, uint32_t targetAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->ICSAR = targetAddr;
}

static void I2CSetDataCount(uint32_t baseAddr, uint32_t count)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->ICCNT = count;
}

static void I2CControllerDataPut(uint32_t baseAddr, uint8_t data)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->ICDXR = data;
}

static void I2COwnAddressSet(uint32_t baseAddr, uint32_t ownAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    i2cRegs->ICOAR = ownAddr;
}

static uint32_t I2CControllerIntStatusEx(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    return (i2cRegs->ICSTR & intFlag);
}

static uint32_t I2CControllerErr(uint32_t baseAddr)
{
    uint32_t errMask;
    errMask = I2C_INT_ARBITRATION_LOST    |
              I2C_INT_NO_ACK              |
              I2C_INT_STOP_CONDITION      |
              I2C_INT_ADRR_ZERO           |
              I2C_INT_RECV_OVER_RUN;

    return (I2CControllerIntStatusEx(baseAddr, errMask));
}

static uint32_t I2CIntVectGet(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    return ((i2cRegs->ICIVR) & I2C_IVR_INTCODE_MASK);
}

static int32_t I2CControllerBusBusy(uint32_t baseAddr)
{
    int32_t ret_val = (int32_t)0;
    if(I2CControllerIntStatusEx(baseAddr, I2C_INT_BUS_BUSY) != (uint32_t)0U)
    {
        ret_val = (int32_t)1;
    }
    return ret_val;
}

static uint8_t I2CControllerDataGet(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    return (uint8_t)(i2cRegs->ICDRR);
}

static int32_t I2C_lld_waitForBb(I2CLLD_Handle handle, uint32_t timeout)
{
    uint32_t                stat;
    int32_t                 retVal = I2C_STS_SUCCESS;
    volatile uint32_t       bbtimeout = timeout;
    uint32_t                startTicks, elapsedTicks = 0;

    I2CLLD_Object           *object  = NULL;

    object  = (I2CLLD_Object*)handle;

    if(bbtimeout > 0U)
    {
        /* Clear current interrupts...*/
        I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

        while (bbtimeout > 0U)
        {
            stat = I2CControllerIntStatusEx(object->baseAddr, I2C_INT_BUS_BUSY);
            if (stat == 0U)
            {
                break;
            }
            bbtimeout = bbtimeout - 1U;
            I2CControllerIntClearEx(object->baseAddr, stat);
        }

        if (timeout > 0U)
        {
            if (bbtimeout == 0U)
            {
                retVal = I2C_STS_ERR;
            }
        }
        /* clear delayed stuff*/
        I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);
    }
    else
    {
        startTicks = object->Clock_getTicks();

        while((I2CControllerBusBusy(object->baseAddr) == 1) &&
        (elapsedTicks < object->Clock_usecToTicks(I2C_BUS_BUSY_TIMEOUT_IN_US)))
        {
            elapsedTicks = object->Clock_getTicks() - startTicks;
        }
        /* Set timeout error if timeout occurred */
        if(elapsedTicks >= object->Clock_usecToTicks(I2C_BUS_BUSY_TIMEOUT_IN_US))
        {
            retVal = I2C_STS_ERR_TIMEOUT;
        }
    }

    return retVal;
}

static void I2C_lld_completeCurrTransfer(I2CLLD_Handle handle,
                                         int32_t xferStatus)
{
    I2CLLD_Object *object = (I2CLLD_Object*)handle;
    /* Input parameter validation */
    if (handle != NULL)
    {
        if(object->currentMsg != NULL)
        {
            /* Other transactions available */
            if( (xferStatus == I2C_STS_SUCCESS) &&
                (object->currentTxnCount < (object->currentMsg->txnCount - 1U)))
            {
                object->currentTxnCount++;

                (void)I2C_lld_primeTransferIntr(handle, object->currentMsg);

                object->state = I2C_STATE_BUSY;
            }

            /* No other transactions available */
            else
            {
                object->dataArray = (uint8_t*)NULL;
                /* Call Transfer Complete Callback */
                object->transferCompleteCallback(handle, object->currentMsg, xferStatus);
                /* No other transactions need to occur */
                object->currentMsg = NULL;

                object->state = I2C_STATE_IDLE;
            }
        }
    }
}

static int32_t I2C_lld_primeTransferIntr(I2CLLD_Handle handle,
                                         I2CLLD_Message *msg)
{
    I2CLLD_Object           *object = NULL;
    int32_t                 status = I2C_STS_SUCCESS;
    uint32_t                xsa;

    /* Get the pointer to the object */
    object = (I2CLLD_Object*)handle;

    object->writeBufIdx = (uint8_t*)msg->txn[object->currentTxnCount].writeBuf;
    object->writeCountIdx = (uint32_t)msg->txn[object->currentTxnCount].writeCount;

    object->readBufIdx = (uint8_t*)msg->txn[object->currentTxnCount].readBuf;
    object->readCountIdx = (uint32_t)msg->txn[object->currentTxnCount].readCount;

    object->intStatusErr = 0U;

    if (object->currentMsg->expandSA == true)
    {
        /* enable the 10-bit address mode */
        xsa = I2C_CFG_CMD_10BIT_ADDRESS;
    }
    else
    {
        /* enable the 7-bit address mode */
        xsa = I2C_CFG_CMD_7BIT_ADDRESS;
    }

    /* clear all interrupts */
    I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

    if (object->currentMsg->controllerMode)
    {
        /* In controller mode, set the I2C target address */
        I2CControllerTargetAddrSet(object->baseAddr,
                              object->currentMsg->targetAddress);

        /* Enable interrupts on stop and error bits */
        I2CControllerIntEnableEx(object->baseAddr,
                                I2C_INT_MASK_ARBITRATION_LOST |
                                I2C_INT_MASK_NO_ACK           |
                                I2C_INT_MASK_STOP_CONDITION);

        if (object->writeCountIdx != 0U)
        {
            /* Set number of bytes to be transmitting */
            I2CSetDataCount(object->baseAddr, object->writeCountIdx);

            /*
             * Configure the I2C transfer to be in controller transmitter mode
             */
            I2CControllerControl(object->baseAddr,
                I2C_CFG_MASK_TX | I2C_CFG_MASK_REPEAT_MODE | I2C_CFG_MASK_XA,
                I2C_CFG_CMD_TX | I2C_CFG_CMD_REPEAT_MODE_OFF | xsa);

            /* set the transfer state to write mode */
            object->state = I2C_WRITE_STATE;

            /*
             * enable transmit interrupt.
             */
            I2CControllerIntEnableEx(object->baseAddr,
                                    I2C_INT_MASK_TRANSMIT_READY     |
                                    I2C_INT_NO_ACK                  |
                                    I2C_INT_ARBITRATION_LOST);

            if (object->readCountIdx == 0U)
            {
                /* If no data read, send stop at the end of write */
                I2CControllerControl(object->baseAddr,
                                    I2C_CFG_MASK_STOP | I2C_CFG_MASK_XA,
                                    I2C_CFG_CMD_STOP | xsa);
            }

            /* Start the I2C transfer in controller transmit mode */
            I2CControllerStart(object->baseAddr);
        }
        /* Start transfer in Receive mode */
        else
        {
            object->state = I2C_READ_STATE;

            /* set number of bytes to read */
            I2CSetDataCount(object->baseAddr, object->readCountIdx);

            /* set to controller receiver mode */
            I2CControllerControl(object->baseAddr,
                                I2C_CFG_MASK_RX | I2C_CFG_MASK_REPEAT_MODE  | I2C_CFG_MASK_XA,
                                I2C_CFG_CMD_RX | I2C_CFG_CMD_REPEAT_MODE_OFF | xsa);

            /* Enable RX interrupts */
            I2CControllerIntEnableEx(object->baseAddr, I2C_INT_MASK_RECV_READY  | I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST);

            /* Send start bit */
            I2CControllerStart(object->baseAddr);

        }
    }

    /* In target mode */
    else
    {
        /*
            * Enable Address as Target interrupt which is the first interrupt
            * received in target mode
            */
        I2CControllerIntEnableEx(object->baseAddr, I2C_INT_MASK_ADRR_TARGET);
        /* Configure data buffer length to 0 as the actual number of bytes to
            transmit/receive is dependant on external controller. */
        I2CSetDataCount(object->baseAddr, 0U);

        /* Start the I2C transfer in target mode */
        I2CTargetEnable(object->baseAddr);

        /* set to controller receiver mode */
        I2CModeControl(object->baseAddr, I2C_CFG_MASK_XA, xsa);
    }

    return status;
}

/* This function will return I2C_StatusCodes based on execution */
static int32_t I2C_lld_primeTransferPoll(   I2CLLD_Handle handle,
                                            I2CLLD_Message *msg)
{
    I2CLLD_Object           *object = NULL;
    int32_t                 status = I2C_STS_SUCCESS;
    uint32_t                errStat = 0;
    uint32_t                xsa;
    uint32_t                intStat;

    /* Get the pointer to the object */
    object = (I2CLLD_Object*)handle;

    object->startTicks = object->Clock_getTicks();

    while((object->currentTxnCount) < (object->currentMsg->txnCount))
    {

        object->writeBufIdx = (uint8_t*)object->currentMsg->txn[object->currentTxnCount].writeBuf;
        object->writeCountIdx = (uint32_t)object->currentMsg->txn[object->currentTxnCount].writeCount;

        object->readBufIdx = (uint8_t*)object->currentMsg->txn[object->currentTxnCount].readBuf;
        object->readCountIdx = (uint32_t)object->currentMsg->txn[object->currentTxnCount].readCount;

        if (object->currentMsg->expandSA == true)
        {
            /* enable the 10-bit address mode */
            xsa = I2C_CFG_CMD_10BIT_ADDRESS;
        }
        else
        {
            /* enable the 7-bit address mode */
            xsa = I2C_CFG_CMD_7BIT_ADDRESS;
        }

        while(  (I2CControllerBusBusy(object->baseAddr) == (int32_t)1) &&
                (status == I2C_STS_SUCCESS))
        {
            if (    ((object->Clock_getTicks()) - (object->startTicks)) >
                    (object->Clock_usecToTicks((uint64_t)(msg->timeout))))
            {
                status = I2C_STS_ERR_TIMEOUT;
            }
        }

        if (object->currentMsg->controllerMode)
        {
            /* In controller mode, set the I2C target address */
            I2CControllerTargetAddrSet(object->baseAddr,
                                object->currentMsg->targetAddress);


            if((object->writeCountIdx != 0U) && (status == I2C_STS_SUCCESS))
            {
                /* clear all interrupts */
                I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

                /* set number of bytes to write */
                I2CSetDataCount(object->baseAddr, object->writeCountIdx);

                /* set to controller transmitter mode */
                I2CControllerControl(object->baseAddr,
                                    I2C_CFG_MASK_TX | I2C_CFG_MASK_REPEAT_MODE | I2C_CFG_MASK_XA,
                                    I2C_CFG_CMD_TX | I2C_CFG_CMD_REPEAT_MODE_OFF | xsa);

                /* Write byte and increase data pointer to next byte */
                I2CControllerDataPut(object->baseAddr, *(object->writeBufIdx));
                (object->writeBufIdx)++;

                /* Update number of bytes written */
                (object->writeCountIdx)--;

                if(handle->memTxnActive)
                {
                    if((handle->i2ctxn.writeCount - handle->memAddrSize) == (handle->writeCountIdx))
                    {
                        handle->writeBufIdx = handle->dataArray;
                    }
                }

                /* generate start */
                I2CControllerStart(object->baseAddr);

                /* wait for bus busy */
                while((status == I2C_STS_SUCCESS) &&
                (I2CControllerBusBusy(object->baseAddr) == 0))
                {
                    if ((   object->Clock_getTicks() - object->startTicks) >
                            object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout)))
                    {
                        status = I2C_STS_ERR_TIMEOUT;
                    }
                }
                while ((status == I2C_STS_SUCCESS) && (object->writeCountIdx != 0U))
                {
                    if ((   object->Clock_getTicks() - object->startTicks) >
                            object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout)))
                    {
                        status = I2C_STS_ERR_TIMEOUT;
                    }
                    while((I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_TRANSMIT_READY) == 0U) &&
                        (I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_ARBITRATION_LOST | I2C_INT_NO_ACK |
                        I2C_INT_STOP_CONDITION ) == 0U))
                    {
                        if ((   object->Clock_getTicks() - object->startTicks) >
                                object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout)))
                        {
                            status = I2C_STS_ERR_TIMEOUT;
                            break;
                        }
                    }
                    errStat = I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_ARBITRATION_LOST | I2C_INT_NO_ACK |
                        I2C_INT_STOP_CONDITION);
                    /* wait for transmit ready or error */
                    if (errStat != 0U)
                    {
                        if ((errStat & (I2C_INT_ARBITRATION_LOST    |
                                        I2C_INT_NO_ACK              |
                                        I2C_INT_STOP_CONDITION )) != (uint32_t)0U)
                        {
                            /* if we get an error, do a stop and return failure */
                            status = I2C_STS_ERR;
                            break;
                        }
                    }
                    /* write byte and increase data pointer to next byte */
                    I2CControllerDataPut(object->baseAddr, *(object->writeBufIdx));
                    (object->writeBufIdx)++;

                    /* update number of bytes written */
                    object->writeCountIdx--;

                    if(handle->memTxnActive)
                    {
                        if((handle->i2ctxn.writeCount - handle->memAddrSize) == (handle->writeCountIdx))
                        {
                            handle->writeBufIdx = handle->dataArray;
                        }
                    }
                }

                /* wait for register access ready */
                while((status == I2C_STS_SUCCESS) &&
                        (I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_ADRR_READY_ACESS) == 0U))
                {
                    if  ((object->Clock_getTicks() - object->startTicks) >
                        object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout)))
                    {
                        status = I2C_STS_ERR_TIMEOUT;
                    }
                }

                if ((errStat & I2C_INT_ARBITRATION_LOST) == I2C_INT_ARBITRATION_LOST)
                {
                    status = I2C_STS_ERR_ARBITRATION_LOST;
                }
                else if ((errStat & I2C_INT_NO_ACK) == I2C_INT_NO_ACK)
                {
                    status = I2C_STS_ERR_NO_ACK;
                }
                else
                {
                    status = I2C_STS_SUCCESS;
                }

                /* Store the error in intStatusErr */
                handle->intStatusErr = errStat;

                if((object->readCountIdx == 0U) && (status == I2C_STS_SUCCESS))
                {
                    /* generate stop when no data read */
                    I2CControllerStop(object->baseAddr);

                    /* wait for stop to happen */
                    while((status == I2C_STS_SUCCESS) &&
                            (I2CControllerIntStatusEx(object->baseAddr,
                            I2C_INT_STOP_CONDITION) == 0U))
                    {
                        if ((   object->Clock_getTicks() - object->startTicks) >
                            object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout)))
                        {
                            status = I2C_STS_ERR_TIMEOUT;
                        }
                    }
                }
            }

            if((object->readCountIdx != 0U) && (status == I2C_STS_SUCCESS))
            {
                /* clear all interrupts */
                I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

                /* set number of bytes to read */
                I2CSetDataCount(object->baseAddr, object->readCountIdx);

                /* set to controller receiver mode */
                I2CControllerControl(object->baseAddr,
                    I2C_CFG_MASK_RX | I2C_CFG_MASK_REPEAT_MODE | I2C_CFG_MASK_XA,
                    I2C_CFG_CMD_RX | I2C_CFG_CMD_REPEAT_MODE_OFF | xsa);

                /* generate start */
                I2CControllerStart(object->baseAddr);

                /* wait for bus not busy */
                while(  (status == I2C_STS_SUCCESS) &&
                        (I2CControllerBusBusy(object->baseAddr) == 0))
                {
                    if  ((object->Clock_getTicks() - object->startTicks) >
                        object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout)))
                    {
                        status = I2C_STS_ERR_TIMEOUT;
                    }
                }
                while(object->readCountIdx != 0U)
                {
                    /* wait for receive ready or error */
                    while((status == I2C_STS_SUCCESS) &&
                        (I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_RECV_READY) == 0U) &&
                        (I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_ARBITRATION_LOST | I2C_INT_NO_ACK |
                        I2C_INT_STOP_CONDITION) == 0U))
                    {
                        if  ((object->Clock_getTicks() - object->startTicks) >
                            object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout)))
                        {
                            status = I2C_STS_ERR_TIMEOUT;
                            break;
                        }
                    }

                    /* if we get an error, do a stop and return failure */
                    errStat=I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_ARBITRATION_LOST | I2C_INT_NO_ACK |
                        I2C_INT_STOP_CONDITION);
                    if(errStat != 0U) {
                        break;
                    }
                    /* read byte and increase data pointer to next byte */
                    *(object->readBufIdx) =
                        (uint8_t)I2CControllerDataGet(object->baseAddr);

                    (object->readBufIdx)++;
                    (object->readCountIdx)--;   /* update number of bytes read */
                }

                if((errStat & I2C_INT_ARBITRATION_LOST) == I2C_INT_ARBITRATION_LOST)
                {
                    status = I2C_STS_ERR_ARBITRATION_LOST;
                }
                else if((errStat & I2C_INT_NO_ACK) == I2C_INT_NO_ACK)
                {
                    status = I2C_STS_ERR_NO_ACK;
                }
                else
                {
                    status = I2C_STS_SUCCESS;
                }

                /* Store the error in intStatusErr */
                handle->intStatusErr = errStat;
                /* generate stop when requested */
                I2CControllerStop(object->baseAddr);

                /* wait for stop to happen */
                while(  (status == I2C_STS_SUCCESS) &&
                        (I2CControllerIntStatusEx(object->baseAddr,
                            I2C_INT_STOP_CONDITION) == 0U))
                {
                    if  ((object->Clock_getTicks() - object->startTicks) >
                        object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout)))
                    {
                        status = I2C_STS_ERR_TIMEOUT;
                    }
                }
            }
        }
        else
        {
            /* Clear all interrupts */
            I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);
            /*
            * Enable Address as Target interrupt which is the first interrupt
            * received in target mode
            */
            I2CControllerIntEnableEx(object->baseAddr, I2C_INT_MASK_ADRR_TARGET);
            /* Configure data buffer length to 0 as the actual number of bytes to
                transmit/receive is dependant on external controller. */
            I2CSetDataCount(object->baseAddr, 0U);

            /* Start the I2C transfer in target mode */
            I2CTargetEnable(object->baseAddr);

            /* set to controller receiver mode */
            I2CModeControl(object->baseAddr, I2C_CFG_MASK_XA, xsa);

            /* Wait for address match */
            while ( (status == I2C_STS_SUCCESS) &&
                    (I2CControllerIntStatusEx(object->baseAddr, I2C_INT_ADRR_TARGET) == 0U))
            {
                if ((   object->Clock_getTicks() - object->startTicks) >
                        object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout)))
                {
                    status = I2C_STS_ERR_TIMEOUT;
                }
            }

            while (status == I2C_STS_SUCCESS)
            {
                intStat = I2CControllerIntStatusEx(handle->baseAddr, I2C_ALL_INTS);

                if((intStat & I2C_INT_ADRR_READY_ACESS) != 0)
                {
                    I2CControllerIntClearEx(handle->baseAddr, I2C_INT_ADRR_READY_ACESS);
                }

                if((intStat & I2C_INT_RECV_READY) != 0)
                {
                    /* Read from Rx register only when read count is not exhausted */
                    if ((handle->readCountIdx) != (uint32_t)0U)
                    {
                        *(handle->readBufIdx) = I2CControllerDataGet(handle->baseAddr);
                        handle->readBufIdx++;
                        handle->readCountIdx--;
                    }
                    else
                    {
                        /* RX buffer full, drop the data Received */
                        (void)I2CControllerDataGet(handle->baseAddr);
                    }

                    I2CControllerIntClearEx(handle->baseAddr, I2C_INT_RECV_READY);
                }

                if((intStat & I2C_INT_TRANSMIT_READY) != 0)
                {
                    if (handle->writeCountIdx != 0U)
                    {
                        I2CControllerDataPut(handle->baseAddr, *(handle->writeBufIdx));
                        handle->writeCountIdx--;
                        handle->writeBufIdx++;
                    }
                    else
                    {
                        if ((object->currentMsg->txn[object->currentTxnCount].writeCount) != (uint32_t)0U)
                        {
                            /* TX buffer empty, send 0 */
                            I2CControllerDataPut(handle->baseAddr, 0U);
                        }
                    }
                }

                if((intStat & I2C_INT_STOP_CONDITION) != 0)
                {
                    I2CControllerIntDisableEx(handle->baseAddr, I2C_ALL_INTS_MASK);
                    I2CControllerIntClearEx(handle->baseAddr, I2C_ALL_INTS);
                    break;
                }

                if ((   object->Clock_getTicks() - object->startTicks) >
                        object->Clock_usecToTicks((uint64_t)(object->currentMsg->timeout)))
                {
                    status = I2C_STS_ERR_TIMEOUT;
                }
            }
        }

        if(status == I2C_STS_SUCCESS)
        {
            object->currentTxnCount++;
        }
        else{
            break;
        }
    }

    /* No further Transactions */
    object->currentMsg = NULL;
    object->state = I2C_STATE_IDLE;

    return status;
}

static int32_t I2C_lld_transferInit(I2CLLD_Handle handle, I2CLLD_Message *msg)
{
    int32_t                 retVal = I2C_STS_SUCCESS;
    I2CLLD_Object           *object = NULL;

    /* Get the pointer to the object */
    object = (I2CLLD_Object*)handle;

    if (msg->timeout == 0U)
    {
        /* timeout cannot be NO_WAIT, set it to default value */
        msg->timeout = I2C_WAIT_FOREVER;
    }

    if((msg->txn) == (I2CLLD_Transaction*)NULL)
    {
        retVal = I2C_STS_ERR;
    }

    if (    (retVal == I2C_STS_SUCCESS) &&
            ((msg->txn->writeCount != 0U) || (msg->txn->readCount != 0U)))
    {
        if (object->state == I2C_STATE_IDLE)
        {
            /* No current transfer is going on. */
            /* TODO: This should happen automically. */
            object->state = I2C_STATE_BUSY;
            object->currentMsg = msg;
            object->currentTxnCount = 0U;
        }
        else
        {
            /* Some transfer is going on. return with system Busy. */
            retVal = I2C_STS_ERR_BUS_BUSY;
        }
    }

    return (retVal);
}

static int32_t lld_check_param(bool expression)
{
    int32_t retVal = I2C_STS_ERR_INVALID_PARAM;
    if(expression)
    {
        retVal = I2C_STS_SUCCESS;
    }
    return retVal;
}

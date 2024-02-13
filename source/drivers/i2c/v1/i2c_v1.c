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
 *  \file   i2c_v1.c
 *
 *  \brief  File containing I2C Driver APIs implementation for V1.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/i2c.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_i2c.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define I2C_MODULE_INTERNAL_CLK_8MHZ    (8000000U)
#define I2C_MODULE_INTERNAL_CLK_12P5MHZ (12500000U)
#define I2C_MODULE_FUNC_CLK_200MHZ      (200000000U)
#define I2C_MODULE_FUNC_CLK_150MHZ      (150000000U)
#define I2C_MODULE_FUNC_CLK_96MHZ       (96000000U)
#define I2C_CONTROLLER_ERR_NONE             (0)
#define I2C_DELAY_MED                   ((uint32_t) 10000U)
#define I2C_DELAY_BIG                   ((uint32_t) 30000U)
#define I2C_DELAY_SMALL                 ((uint32_t) 5000U)
#define I2C_DELAY_USEC                  ((uint32_t) 250U)
#define I2C_OWN_ADDR                    (0x10U)

/*
 * Maximum number of loop count to handle in the same ISR, which is
 * to process multiple interrupts (ARDY, RRDY, XRDY) in the ISR to
 * reduce interrupt count
 *
 * Keep at least 3 to support optimal RX followed by TX scenario
 * Keep at least 2 to support optimal RX scenario
 */
#define I2C_MAX_CONSECUTIVE_ISRS      (1U)

/**
 *  \anchor I2C_State
 *  \name MACROS used to define the state of the I2C Driver State Machine
 *  @{
 */
#define I2C_IDLE_STATE                  ((uint8_t) 0U)
#define I2C_WRITE_STATE                 ((uint8_t) 1U)
#define I2C_READ_STATE                  ((uint8_t) 2U)
/** \brief  I2C is trasferring in target mode */
#define I2C_TARGET_XFER_STATE            ((uint8_t) 3U)
/** \brief  I2C is restarting trasfer in target mode */
#define I2C_TARGET_RESTART_STATE         ((uint8_t) 4U)
#define I2C_ERROR                       ((uint8_t) 255U)
/** @} */

/**
 * \anchor   I2C_ControllerControl
 * \name MACROS that can be passed to I2CControllerControl API as cmd to configure
 * mode of operation of I2C
 * @{
 */
#define I2C_CFG_MASK_TX             (CSL_I2C_ICMDR_TRX_MASK)
#define I2C_CFG_MASK_RX             (CSL_I2C_ICMDR_TRX_MASK)
#define I2C_CFG_MASK_STOP           (CSL_I2C_ICMDR_STP_MASK)
#define I2C_CFG_MASK_START          (CSL_I2C_ICMDR_STT_MASK)
#define I2C_CFG_MASK_RUN_FREE       (CSL_I2C_ICMDR_FREE_MASK)
#define I2C_CFG_MASK_REPEAT_MODE    (CSL_I2C_ICMDR_RM_MASK)
#define I2C_CFG_MASK_LOOP_BACK      (CSL_I2C_ICMDR_DLB_MASK)
#define I2C_CFG_MASK_XA             (CSL_I2C_ICMDR_XA_MASK)


#define I2C_CFG_CMD_TX              (CSL_I2C_ICMDR_TRX_MASK)
#define I2C_CFG_CMD_RX              (0U)
#define I2C_CFG_CMD_STOP            (CSL_I2C_ICMDR_STP_MASK)
#define I2C_CFG_CMD_START           (CSL_I2C_ICMDR_STT_MASK)
#define I2C_CFG_CMD_RUN_FREE_ON     (CSL_I2C_ICMDR_FREE_MASK)
#define I2C_CFG_CMD_RUN_FREE_OFF    (0U)
#define I2C_CFG_CMD_REPEAT_MODE_ON  (CSL_I2C_ICMDR_RM_MASK)
#define I2C_CFG_CMD_REPEAT_MODE_OFF (0U)
#define I2C_CFG_CMD_LOOP_BACK_ON    (CSL_I2C_ICMDR_DLB_MASK)
#define I2C_CFG_CMD_LOOP_BACK_OFF   (0U)
#define I2C_CFG_CMD_10BIT_ADDRESS   (I2C_CFG_MASK_XA)
#define I2C_CFG_CMD_7BIT_ADDRESS    (0U)
/** @} */

/**
 * \anchor   I2C_ControllerIntEnableEx
 * \name MACROS that can be passed to I2CControllerIntStatusEx and I2CControllerIntClearEx
 * APIs as int status flag to check and clear interrupt status
 * @{
 */
#define I2C_INT_ARBITRATION_LOST     (CSL_I2C_ICSTR_AL_MASK)
#define I2C_INT_NO_ACK               (CSL_I2C_ICSTR_NACK_MASK)
#define I2C_INT_ADRR_READY_ACESS     (CSL_I2C_ICSTR_ARDY_MASK)
#define I2C_INT_RECV_READY           (CSL_I2C_ICSTR_ICRRDY_MASK)
#define I2C_INT_TRANSMIT_READY       (CSL_I2C_ICSTR_ICXRDY_MASK)
#define I2C_INT_STOP_CONDITION       (CSL_I2C_ICSTR_SCD_MASK)
#define I2C_INT_ADRR_ZERO            (CSL_I2C_ICSTR_AD0_MASK)
#define I2C_INT_ADRR_TARGET           (CSL_I2C_ICSTR_AAS_MASK)
#define I2C_INT_TRANSMIT_UNDER_FLOW  (CSL_I2C_ICSTR_XSMT_MASK)
#define I2C_INT_RECV_OVER_RUN        (CSL_I2C_ICSTR_RSFULL_MASK)
#define I2C_INT_BUS_BUSY             (CSL_I2C_ICSTR_BB_MASK)
#define I2C_INT_NO_ACK_SENT          (CSL_I2C_ICSTR_NACKSNT_MASK)
#define I2C_INT_TARGET_DIRECTION      (CSL_I2C_ICSTR_SDIR_MASK)

#define I2C_ALL_INTS                (I2C_INT_ARBITRATION_LOST     | \
                                     I2C_INT_NO_ACK               | \
                                     I2C_INT_ADRR_READY_ACESS     | \
                                     I2C_INT_RECV_READY           | \
                                     I2C_INT_TRANSMIT_READY       | \
                                     I2C_INT_STOP_CONDITION       | \
                                     I2C_INT_ADRR_ZERO            | \
                                     I2C_INT_ADRR_TARGET           | \
                                     I2C_INT_TRANSMIT_UNDER_FLOW  | \
                                     I2C_INT_RECV_OVER_RUN        | \
                                     I2C_INT_BUS_BUSY             | \
                                     I2C_INT_NO_ACK_SENT          | \
                                     I2C_INT_TARGET_DIRECTION)
/** @} */

/**
 * \anchor   I2C_ControllerInterruptFlag
 * \name MACROS that can be passed to I2CControllerIntEnableEx and I2CControllerIntDisableEx
 * APIs as intFlag to enable or disable interrupts
 * @{
 */
#define I2C_INT_MASK_ARBITRATION_LOST     (CSL_I2C_ICIMR_AL_MASK)
#define I2C_INT_MASK_NO_ACK               (CSL_I2C_ICIMR_NACK_MASK)
#define I2C_INT_MASK_ADRR_READY_ACESS     (CSL_I2C_ICIMR_ARDY_MASK)
#define I2C_INT_MASK_RECV_READY           (CSL_I2C_ICIMR_ICRRDY_MASK)
#define I2C_INT_MASK_TRANSMIT_READY       (CSL_I2C_ICIMR_ICXRDY_MASK)
#define I2C_INT_MASK_STOP_CONDITION       (CSL_I2C_ICIMR_SCD_MASK)
#define I2C_INT_MASK_ADRR_TARGET           (CSL_I2C_ICIMR_AAS_MASK)

#define I2C_ALL_INTS_MASK           (I2C_INT_MASK_ARBITRATION_LOST     | \
                                     I2C_INT_MASK_NO_ACK               | \
                                     I2C_INT_MASK_ADRR_READY_ACESS     | \
                                     I2C_INT_MASK_RECV_READY           | \
                                     I2C_INT_MASK_TRANSMIT_READY       | \
                                     I2C_INT_MASK_STOP_CONDITION       | \
                                     I2C_INT_MASK_ADRR_TARGET)
/** @} */

/**
 * \anchor   I2C_interruptVector
 * \name MACROS used to configure the I2C interrupt vector Code
 * @{
 */

#define I2C_IVR_INTCODE_MASK             (CSL_I2C_ICIVR_INTCODE_MASK)
#define I2C_IVR_INTCODE_AL               (CSL_I2C_ICIVR_INTCODE_AL)
#define I2C_IVR_INTCODE_NACK             (CSL_I2C_ICIVR_INTCODE_NACK)
#define I2C_IVR_INTCODE_ARDY             (CSL_I2C_ICIVR_INTCODE_RAR)
#define I2C_IVR_INTCODE_RRDY             (CSL_I2C_ICIVR_INTCODE_RDR)
#define I2C_IVR_INTCODE_XRDY             (CSL_I2C_ICIVR_INTCODE_TDR)
#define I2C_IVR_INTCODE_SCD              (CSL_I2C_ICIVR_INTCODE_SCD)
#define I2C_IVR_INTCODE_AAS              (CSL_I2C_ICIVR_INTCODE_AAS)
/** @} */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    void                   *lock;
    /**< Driver lock - to protect across open/close */
    SemaphoreP_Object       lockObj;
    /**< Driver lock object */
} I2C_DrvObj;

/* Default I2C parameters structure */
const I2C_Params I2C_defaultParams = {
    I2C_MODE_BLOCKING,  /* transferMode */
    NULL,               /* transferCallbackFxn */
    I2C_100KHZ,         /* bitRate */
};

/* Default I2C transaction parameters structure */
const I2C_Transaction I2C_defaultTransaction = {

    NULL,                         /* writeBuf */
    0,                            /* writeCount */
    NULL,                         /* readBuf */
    0,                            /* readCount */
    0,                            /* targetAddress */
    NULL,                         /* nextPtr */
    NULL,                         /* arg */
    SystemP_WAIT_FOREVER,         /* timeout */
    (bool)true,                   /* controllerMode */
    (bool)false                   /* expandSA */
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void        I2C_hwiFxn(void* arg);
static void        I2C_hwiFxnTarget(I2C_Handle handle);
static void        I2C_hwiFxnController(I2C_Handle handle);
static void        I2C_completeCurrTransfer(I2C_Handle handle, int32_t xferStatus);
static int32_t     I2C_primeTransfer(I2C_Handle handle,
                              I2C_Transaction *transaction);
static void        I2C_transferCallback(I2C_Handle handle,
                                  I2C_Transaction *msg,
                                  int32_t transferStatus);
static int32_t     I2C_waitForBb(uint32_t baseAddr, uint32_t timeout);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Externs */
extern I2C_Config gI2cConfig[];
extern uint32_t gI2cConfigNum;

/** \brief Driver object */
static I2C_DrvObj gI2cDrvObj =
{
    .lock = NULL,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*TI_INSPECTED 8 D : MISRAC_2012_R_2.2
 *"No problem in redefining variable without being referenced after it's
 * definition" */
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
    prescalar = sysClk / internalClk - 1U;
    i2cRegs->ICPSC = prescalar;

    if (prescalar == 0)
    {
        diff = 7;
    }
    else if (prescalar == 1)
    {
        diff = 6;
    }
    else
    {
        diff = 5;
    }
    /* Set the CLKL and CLKH values */
    divider = internalClk / outputClk;
    divider = divider / 2U;
    i2cRegs->ICCLKL = divider - diff;
    i2cRegs->ICCLKH = divider - diff;
}

static void I2CControllerEnable(uint32_t baseAddr)
{
    /* Bring the I2C module out of reset */
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;

    /* Set Own Address */
    i2cRegs->ICOAR = I2C_OWN_ADDR;

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

static uint32_t I2CControllerIntStatusEx(uint32_t baseAddr, uint32_t intFlag)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    return (i2cRegs->ICSTR & intFlag);
}

static int32_t I2CControllerBusBusy(uint32_t baseAddr)
{
    bool ret_val = false;
    if(I2CControllerIntStatusEx(baseAddr, I2C_INT_BUS_BUSY))
    {
        ret_val = (bool)true;
    }
    else
    {
        ret_val = (bool)false;
    }
    return ret_val;
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

static void I2CControllerControl(uint32_t baseAddr, uint32_t ctrlMask, uint32_t ctrlCmds)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;
    uint32_t i2cMdr = i2cRegs->ICMDR;

    i2cMdr &= ~ctrlMask;
    i2cMdr |= (ctrlCmds | CSL_I2C_ICMDR_MST_MASK);
    i2cRegs->ICMDR = i2cMdr;
}

static void I2CModeControl(uint32_t baseAddr, uint32_t ctrlMask, uint32_t ctrlCmds)
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

static uint8_t I2CControllerDataGet(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;

    return (uint8_t)(i2cRegs->ICDRR);
}

static void I2COwnAddressSet(uint32_t baseAddr, uint32_t ownAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;

    i2cRegs->ICOAR = ownAddr;
}

static uint32_t I2CIntVectGet(uint32_t baseAddr)
{
    CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)baseAddr;

    return ((i2cRegs->ICIVR) & I2C_IVR_INTCODE_MASK);
}

void I2C_Params_init(I2C_Params *params)
{
    /* Input parameter validation */
    if (params != NULL)
    {
        *params = I2C_defaultParams;
    }
}

void I2C_Transaction_init(I2C_Transaction *transaction)
{
    *transaction = I2C_defaultTransaction;
}

/*
 *  ======== I2C_close ========
 */
void I2C_close(I2C_Handle handle)
{
    I2C_Object         *object = NULL;
    I2C_HwAttrs const  *hwAttrs = NULL;

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object */
        object = (I2C_Object*)handle->object;
        hwAttrs = (I2C_HwAttrs const *)handle->hwAttrs;

        DebugP_assert(NULL != gI2cDrvObj.lock);
        SemaphoreP_pend(&gI2cDrvObj.lockObj, SystemP_WAIT_FOREVER);

        /* Check to see if a I2C transaction is in progress */
        if (object->headPtr == NULL)
        {
            /* Mask I2C interrupts */
            I2CControllerIntDisableEx(object->baseAddr, I2C_ALL_INTS_MASK);

            /* Disable the I2C Controller */
            I2CControllerDisable(object->baseAddr);

            if (TRUE == hwAttrs->enableIntr)
            {
                /* Destruct the Hwi */
                (void)HwiP_destruct(&object->hwiObj);
            }

            /* Destruct the instance lock */
            (void)SemaphoreP_destruct(&object->mutex);

            if (I2C_MODE_BLOCKING == object->i2cParams.transferMode)
            {
                /* Destruct the transfer completion lock */
                (void)SemaphoreP_destruct(&object->transferComplete);
            }


            object->isOpen = (bool)false;
        }
        SemaphoreP_post(&gI2cDrvObj.lockObj);
    }
    return;
}

I2C_Handle I2C_getHandle(uint32_t index)
{
    I2C_Handle         handle = NULL;
    /* Check index */
    if(index < gI2cConfigNum)
    {
        I2C_Object *object;

        object = gI2cConfig[index].object;
        if((object != NULL) && ((bool)true == object->isOpen))
        {
            /* valid handle */
            handle = (I2C_Handle)&(gI2cConfig[index]);
        }
    }

    return handle;
}

static void I2C_completeCurrTransfer(I2C_Handle handle, int32_t xferStatus)
{
    I2C_Object     *object = (I2C_Object*)handle->object;

    object->state = I2C_IDLE_STATE;
    if(object->currentTransaction != NULL)
    {
          object->currentTransaction->readCount -= object->readCountIdx;
          object->currentTransaction->writeCount -= object->writeCountIdx;

          /* Callback to application or post semaphore */
          object->i2cParams.transferCallbackFxn(handle,
                                                object->currentTransaction,
                                                xferStatus);

          /* See if we need to process any other transactions */
          if (object->headPtr == object->tailPtr)
          {
              /* No other transactions need to occur */
              object->currentTransaction = NULL;
              object->headPtr = NULL;

          }
          else
          {
            /* Another transfer needs to take place */
            object->headPtr = (I2C_Transaction*)(object->headPtr->nextPtr);

             /* Start new transfer */
            I2C_primeTransfer(handle, object->headPtr);
          }
    }
}

/*
 *  Hwi interrupt handler to service the I2C peripheral in controller mode
 */
static void I2C_hwiFxnController(I2C_Handle handle)
{
    I2C_Object        *object = NULL;
    int32_t            xferStatus = I2C_STS_SUCCESS;
    uint32_t           errStatus;
    uint32_t           stopCondition;
    uint32_t           xsa;

    /* Get the pointer to the object */
    object = (I2C_Object*)handle->object;

    /* Check for I2C Errors */
    errStatus = I2CControllerErr(object->baseAddr);
    if (errStatus & I2C_INT_STOP_CONDITION)
    {
        if ((object->writeCountIdx == 0U) && (object->readCountIdx == 0U))
        {
            /* End of transfer stop condition, not an error */
            stopCondition = I2C_INT_STOP_CONDITION;
            errStatus &= (uint32_t)(~stopCondition);
        }
    }

    if ((errStatus == I2C_CONTROLLER_ERR_NONE) || (object->state == I2C_ERROR))
    {

        /* No errors, now check what we need to do next */
        switch (object->state) {
            /*
             * ERROR case is OK because if an Error is detected, a STOP bit is
             * sent; which in turn will call another interrupt. This interrupt
             * call will then post the transferComplete semaphore to unblock the
             * I2C_transfer function
             */
            case I2C_ERROR:
            case I2C_IDLE_STATE:
                if(object->state == I2C_ERROR) {

                    if (object->intStatusErr & I2C_INT_NO_ACK)
                    {
                        xferStatus = I2C_STS_ERR_NO_ACK;
                    }
                    else if (object->intStatusErr & I2C_INT_ARBITRATION_LOST)
                    {
                        xferStatus = I2C_STS_ERR_ARBITRATION_LOST;
                    }
                    /* Mask I2C interrupts */
                    I2CControllerIntDisableEx(object->baseAddr, I2C_ALL_INTS_MASK);
                    /* Disable the I2C Controller */
                    I2CControllerDisable(object->baseAddr);
                    /* Enable the I2C Controller for operation */
                    I2CControllerEnable(object->baseAddr);
                }
                else
                {
                    xferStatus = I2C_STS_SUCCESS;
                }

                /* Clear STOP condition interrupt */
                I2CControllerIntClearEx(object->baseAddr,
                                    I2C_INT_STOP_CONDITION);
                /* Disable STOP condition interrupt */
                I2CControllerIntDisableEx(object->baseAddr,
                                      I2C_INT_MASK_STOP_CONDITION);
                /* callback to application or post Semaphore to unblock transfer fxn */
                I2C_completeCurrTransfer(handle,xferStatus);

  	        break;

            case I2C_WRITE_STATE:
                /* Check if data needs to be sent */
                if (object->writeCountIdx != 0U) {
                    /* Write data into transmit FIFO */
                    I2CControllerDataPut(object->baseAddr,
                                     *(object->writeBufIdx));
                    (object->writeBufIdx)++;
                    object->writeCountIdx--;

                    if (object->writeCountIdx == 0U)
                    {
                        /* End of write, disable TX ready interrupt */
                        I2CControllerIntDisableEx(object->baseAddr,
                                              I2C_INT_MASK_TRANSMIT_READY);
                        if (object->readCountIdx)
                        {
                            /* Get register access interrupt to start reading */
                            I2CControllerIntEnableEx(object->baseAddr,
                            	                 I2C_INT_MASK_ADRR_READY_ACESS);
                        }
                        else
                        {
                            /* Done with all transmissions, wait for stop condition */
                            object->state = I2C_IDLE_STATE;
                        }
                    }
                }
                else {
                    if (object->readCountIdx) {
                        /* Next state: Receive mode */
                        object->state = I2C_READ_STATE;
                        I2CControllerIntDisableEx(object->baseAddr,
                                              I2C_INT_MASK_ADRR_READY_ACESS);

                        /* Set number of bytes to receive */
                        I2CSetDataCount(object->baseAddr, object->readCountIdx);

                        /* Configure peripheral for I2C Receive mode with stop */
                        if (object->currentTransaction->expandSA == true)
                        {
                            /* enable the 10-bit address mode */
                            xsa = I2C_CFG_CMD_10BIT_ADDRESS;
                        }
                        else
                        {
                            /* enable the 7-bit address mode */
                            xsa = I2C_CFG_CMD_7BIT_ADDRESS;
                        }
                        I2CControllerControl(object->baseAddr,
                                         I2C_CFG_MASK_RX | I2C_CFG_MASK_REPEAT_MODE | I2C_CFG_MASK_XA,
                                         I2C_CFG_CMD_RX | I2C_CFG_CMD_REPEAT_MODE_OFF | xsa);

                        /* Enable RX interrupt to handle data received */
                        I2CControllerIntEnableEx(object->baseAddr, I2C_INT_MASK_RECV_READY);

                        /* Start I2C peripheral in RX mode */
                        I2CControllerStart(object->baseAddr);
                    }
                }
                break;

            case I2C_READ_STATE:
                /* Save the received data */
                *(object->readBufIdx) = I2CControllerDataGet(object->baseAddr);

                object->readBufIdx++;
                object->readCountIdx--;

                if (object->readCountIdx == 0U) {
                    /* No more data to receive, Next state: Idle mode */
                    object->state = I2C_IDLE_STATE;

                    /* Disable RX interrupt, next interrupt will be from STOP */
                    I2CControllerIntDisableEx(object->baseAddr,
                                          I2C_INT_MASK_RECV_READY);
                    /* Send stop */
                    I2CControllerStop(object->baseAddr);
                }
                break;

            default:
                object->state = I2C_ERROR;
                break;
        }
    }
    else {
        /* Some sort of error happened! */
        object->state = I2C_ERROR;
        object->intStatusErr = errStatus;

	    if (errStatus & I2C_INT_NO_ACK)
        {
           xferStatus = I2C_STS_ERR_NO_ACK;
           I2CControllerIntClearEx(object->baseAddr, I2C_INT_NO_ACK);
		} else if (errStatus & I2C_INT_ARBITRATION_LOST)
        {
           xferStatus = I2C_STS_ERR_ARBITRATION_LOST;
 		   I2CControllerIntClearEx(object->baseAddr,I2C_INT_ARBITRATION_LOST);
        } else {
           xferStatus = I2C_STS_ERR;
		}

        /* Disable all interrupts */
        I2CControllerIntDisableEx(object->baseAddr, I2C_ALL_INTS);
        /* Do not clear the status register */
        I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);
        /* Only enable stop condition interrupt */
        I2CControllerIntEnableEx(object->baseAddr, I2C_INT_MASK_STOP_CONDITION);
        /* Generate stop */
        I2CControllerStop(object->baseAddr);
    }
}

/*
 *  Hwi interrupt handler to service the I2C peripheral in target mode
 */
static void I2C_hwiFxnTarget(I2C_Handle handle)
{
    I2C_Object     *object = NULL;
    uint32_t        intCode;
    uint32_t        intStat;

    /* Get the pointer to the object */
    object = (I2C_Object*)handle->object;

    intCode = I2CIntVectGet(object->baseAddr);
    intStat = I2CControllerIntStatusEx(object->baseAddr, I2C_ALL_INTS);

    switch (intCode)
    {
        case I2C_IVR_INTCODE_AAS:
            if (object->state == I2C_IDLE_STATE)
            {
                /*
                 * This is the first transfer initiation from controller
                 * Update the state to transfer started and enable
                 * all the target interrupts
                 */
                object->state = I2C_TARGET_XFER_STATE;
                I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);
                I2CControllerIntEnableEx(object->baseAddr,
                                     I2C_INT_MASK_TRANSMIT_READY | I2C_INT_MASK_RECV_READY |
                                     I2C_INT_MASK_ADRR_READY_ACESS | I2C_INT_MASK_ADRR_TARGET |
                                     I2C_INT_MASK_NO_ACK | I2C_INT_MASK_STOP_CONDITION);

                if (((object->writeCountIdx) != 0) &&
                    ((intStat & I2C_INT_TRANSMIT_READY) == I2C_INT_TRANSMIT_READY))
                {
                    /* Target transmit mode, send data and clear the interrupt */
                    I2CControllerDataPut(object->baseAddr, *(object->writeBufIdx));
                    object->writeCountIdx--;
                    object->writeBufIdx++;

                }
            }
            else if (object->state == I2C_TARGET_XFER_STATE)
            {
                /*
                 * This is a restart condition, callback to application
                 * to restart read/write
                 */
                object->currentTransaction->readCount -= object->readCountIdx;
                object->currentTransaction->writeCount -= object->writeCountIdx;
                object->i2cParams.transferCallbackFxn(handle,
                                                      object->currentTransaction,
                                                      I2C_STS_RESTART);
                object->writeBufIdx = (uint8_t*)(object->currentTransaction->writeBuf);
                object->writeCountIdx = object->currentTransaction->writeCount;

                object->readBufIdx = (uint8_t*)(object->currentTransaction->readBuf);
                object->readCountIdx = object->currentTransaction->readCount;

                object->state = I2C_TARGET_RESTART_STATE;
                if (((object->writeCountIdx) != 0U) &&
                    ((intStat & I2C_INT_TRANSMIT_READY) == I2C_INT_TRANSMIT_READY))
                {
                    /* Target transmit mode with restart, send data and clear the interrupt */
                    I2CControllerDataPut(object->baseAddr, *(object->writeBufIdx));
                    object->writeCountIdx--;
                    object->writeBufIdx++;
                }
            }
            else
            {
                /* Control should not come here. spurious interrupt clear it. */
            }
            I2CControllerIntClearEx(object->baseAddr, I2C_INT_ADRR_TARGET);
            break;

        case I2C_IVR_INTCODE_NACK:
            /* Get a NACK from controller, stop the transfer and callback */
            object->currentTransaction->readCount -= object->readCountIdx;
            object->currentTransaction->writeCount -= object->writeCountIdx;
            object->i2cParams.transferCallbackFxn(handle,
                                                  object->currentTransaction,
                                                  I2C_STS_ERR_NO_ACK);
            I2CControllerIntDisableEx(object->baseAddr, I2C_ALL_INTS_MASK);
            I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);
            break;

        case I2C_IVR_INTCODE_ARDY:
            I2CControllerIntClearEx(object->baseAddr, I2C_INT_ADRR_READY_ACESS);
            break;

        case I2C_IVR_INTCODE_RRDY:
            /* Read from Rx register only when current transaction is ongoing */
            if (object->readCountIdx)
            {
                *(object->readBufIdx) = I2CControllerDataGet(object->baseAddr);
                object->readBufIdx++;
                object->readCountIdx--;
                I2CControllerIntClearEx(object->baseAddr, I2C_INT_RECV_READY);
            }
            else
            {
                /* RX buffer full, drop the data received */
                I2CControllerDataGet(object->baseAddr);
            }
            I2CControllerIntClearEx(object->baseAddr, I2C_INT_RECV_READY);
            break;

        case I2C_IVR_INTCODE_XRDY:
            if ((object->state == I2C_TARGET_XFER_STATE) || (object->state == I2C_TARGET_RESTART_STATE))
            {
                if (object->writeCountIdx != 0U)
                {
                    I2CControllerDataPut(object->baseAddr, *(object->writeBufIdx));
                    object->writeCountIdx--;
                    object->writeBufIdx++;
                }
                else
                {
                    if (object->currentTransaction->writeCount)
                    {
                        /* TX buffer empty, send 0 */
                        I2CControllerDataPut(object->baseAddr, 0);
                    }
                }
            }
            break;

        case I2C_IVR_INTCODE_SCD:
            /* stop condition detected, end of current transfer */
            I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);
            object->state = I2C_IDLE_STATE;
            object->currentTransaction->readCount -= object->readCountIdx;
            object->currentTransaction->writeCount -= object->writeCountIdx;

            /* Callback to application or post semaphore */
            object->i2cParams.transferCallbackFxn(handle,
                                                  object->currentTransaction,
                                                  I2C_STS_SUCCESS);

            /* See if we need to process any other transactions */
            if (object->headPtr == object->tailPtr)
            {
                /* No other transactions need to occur */
                object->currentTransaction = NULL;
                object->headPtr = NULL;
                I2CControllerIntDisableEx(object->baseAddr, I2C_ALL_INTS_MASK);
            }
            else
            {
                /* Another transfer needs to take place */
                object->headPtr = (I2C_Transaction*)(object->headPtr->nextPtr);

                /* Start new transfer */
                I2C_primeTransfer(handle, object->headPtr);
            }
            break;

        default:
            break;
    }
}

/*
 *  Hwi interrupt handler to service the I2C peripheral
 *
 *  The handler is a generic handler for a I2C object.
 */
static void I2C_hwiFxn(void* arg)
{
    I2C_Handle          handle = (I2C_Handle)arg;
    I2C_Object      *object = NULL;

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object */
        object = (I2C_Object *)handle->object;

        if (object->currentTransaction->controllerMode)
        {
            I2C_hwiFxnController(handle);
        }
        else
        {
            I2C_hwiFxnTarget(handle);
        }
    }
    return;
}

void I2C_init(void)
{
    I2C_Handle handle;
    uint32_t i;
    int32_t  status = SystemP_SUCCESS;

    /* Call init function for each config*/
    for (i = 0; i < gI2cConfigNum; i++) {

        handle = &gI2cConfig[i];
        /* Input parameter validation */
        if (handle->object != NULL)
        {
            /* Mark the object as available */
            handle->object->isOpen = (bool)false;
        }
    }

    /* Create driver lock */
    status = SemaphoreP_constructMutex(&gI2cDrvObj.lockObj);
    if(SystemP_SUCCESS == status)
    {
        gI2cDrvObj.lock = &gI2cDrvObj.lockObj;
    }

}

void I2C_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gI2cDrvObj.lock)
    {
        SemaphoreP_destruct(&gI2cDrvObj.lockObj);
        gI2cDrvObj.lock = NULL;
    }
}

I2C_Handle I2C_open(uint32_t idx, const I2C_Params *params)
{
    I2C_Handle          handle = NULL;
    uint32_t            outputClk;
    I2C_Object         *object = NULL;
    I2C_HwAttrs const  *hwAttrs = NULL;
    uint32_t            internalClk;
    int32_t             status = SystemP_SUCCESS;

    /* Check index */
    if(idx >= gI2cConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        handle = (I2C_Handle)&(gI2cConfig[idx]);
    }

    DebugP_assert(NULL != gI2cDrvObj.lock);
    SemaphoreP_pend(&gI2cDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(SystemP_SUCCESS == status)
    {
        object = (I2C_Object*)handle->object;
        DebugP_assert(NULL != object);
        DebugP_assert(NULL != handle->hwAttrs);
        hwAttrs = (I2C_HwAttrs const *)handle->hwAttrs;
        if(TRUE == object->isOpen)
        {
            /* Handle already opended */
            status = SystemP_FAILURE;
            handle = NULL;
        }
    }

    if (SystemP_SUCCESS == status)
    {
        /* Mark the handle as being used */
        object->isOpen = (bool)true;
        object->baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(hwAttrs->baseAddr);
        /* Store the I2C parameters */
        if (params == NULL) {
            /* No params passed in, so use the defaults */
            I2C_Params_init(&(object->i2cParams));
        }
        else {
            /* Copy the params contents */
            object->i2cParams = *params;
        }

        if (TRUE == hwAttrs->enableIntr)
        {
            HwiP_Params hwiPrms;

            /* Initialize with defaults */
            HwiP_Params_init(&hwiPrms);

            /* Populate the interrupt parameters */
            hwiPrms.args = (void *)handle;
            hwiPrms.callback = &I2C_hwiFxn;
            hwiPrms.eventId = hwAttrs->eventId; /* Event going in to CPU */
            hwiPrms.intNum = hwAttrs->intNum; /* Host Interrupt vector */

            /* Register interrupts */
            status = HwiP_construct(&object->hwiObj,&hwiPrms);
            DebugP_assert(status==SystemP_SUCCESS);
        }

        /*
        * Construct thread safe handles for this I2C peripheral
        * Semaphore to provide exclusive access to the I2C peripheral
        */
        status = SemaphoreP_constructMutex(&object->mutex);
        DebugP_assert(status==SystemP_SUCCESS);

        /*
        * Store a callback function that posts the transfer complete
        * semaphore for synchronous mode
        */
        if (object->i2cParams.transferMode == I2C_MODE_BLOCKING)
        {
            /*
            * Semaphore to cause the waiting task to block for the I2C
            * to finish
            */
            status = SemaphoreP_constructBinary(&object->transferComplete, 0);
            DebugP_assert(status==SystemP_SUCCESS);

            /* Store internal callback function */
            object->i2cParams.transferCallbackFxn = &I2C_transferCallback;
        }
        if(object->i2cParams.transferMode == I2C_MODE_CALLBACK)
        {
            if (params != NULL)
            {
            /* Save the callback function pointer */
            object->i2cParams.transferCallbackFxn = params->transferCallbackFxn;
            }
        }

        /* Specify the idle state for this I2C peripheral */
        object->state = I2C_IDLE_STATE;

        /* Clear the head pointer */
        object->headPtr = NULL;
        object->tailPtr = NULL;

        /* Put i2c in reset/disabled state */
        I2CControllerDisable(object->baseAddr);

        /* Internal clock should be between 6.7MHz and 13.3MHz.
           Try to get the integer divider for the PSC and ICCLK.
           Deriving the output clock for 100KHz and 400KHz -
            If input clock is 200MHz or 96MHz  Configure internal clock for 8MHz.
            If input clock is 150MHz/others Configure internal clock for 12.5Mhz. */
        if ((hwAttrs->funcClk == I2C_MODULE_FUNC_CLK_200MHZ) ||
            (hwAttrs->funcClk == I2C_MODULE_FUNC_CLK_96MHZ))
        {
            internalClk = I2C_MODULE_INTERNAL_CLK_8MHZ;
        }
        else
        {
            internalClk = I2C_MODULE_INTERNAL_CLK_12P5MHZ;
        }

        /* Extract bit rate from the input parameter */
        switch(object->i2cParams.bitRate)
        {
            case I2C_100KHZ:
            {
                outputClk = 100000U;
                break;
            }

            case I2C_400KHZ:
            {
                outputClk = 400000U;
                break;
            }

            case I2C_1P0MHZ:
            {
                outputClk = 1000000U;
                break;
            }

            case I2C_3P4MHZ:
            {
                /* Set internal clock to 12.5 MHz to generate 3.4MHz output clock. */
                outputClk = 3400000U;
                internalClk = I2C_MODULE_INTERNAL_CLK_12P5MHZ;
                break;
            }

            default:
            {
                /* Default case force it to 100 KHZ bit rate */
                outputClk = 100000U;
                break;
            }
        }

        /* Set the I2C configuration */
        I2CControllerInitExpClk(object->baseAddr,
                          hwAttrs->funcClk,
                          internalClk,
                          outputClk);

        /* Clear any pending interrupts */
        I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

        /* Mask off all interrupts */
        I2CControllerIntDisableEx(object->baseAddr, I2C_ALL_INTS);

        /* Enable the I2C Controller for operation */
        I2CControllerEnable(object->baseAddr);
        I2COwnAddressSet(object->baseAddr, hwAttrs->ownTargetAddr);

    }

    SemaphoreP_post(&gI2cDrvObj.lockObj);
    return (handle);
}

static int32_t I2C_primeTransfer(I2C_Handle handle,
                                 I2C_Transaction *transaction)
{
    I2C_Object  *object = NULL;
    I2C_HwAttrs const *hwAttrs = NULL;
    int32_t status = I2C_STS_SUCCESS;
    uint32_t errStat=0,fatalError=0;
    uint32_t xsa;

    /* Get the pointer to the object and hwAttrs */
    object = (I2C_Object*)handle->object;
    hwAttrs = (I2C_HwAttrs const *)handle->hwAttrs;

    /* Store the new internal counters and pointers */
    object->currentTransaction = transaction;

    object->writeBufIdx = (uint8_t*)transaction->writeBuf;
    object->writeCountIdx = (uint32_t)transaction->writeCount;

    object->readBufIdx = (uint8_t*)transaction->readBuf;
    object->readCountIdx = (uint32_t)transaction->readCount;

    object->intStatusErr = 0U;

    if (object->currentTransaction->expandSA == true)
    {
        /* enable the 10-bit address mode */
        xsa = I2C_CFG_CMD_10BIT_ADDRESS;
    }
    else
    {
        /* enable the 7-bit address mode */
        xsa = I2C_CFG_CMD_7BIT_ADDRESS;
    }

    if (transaction->controllerMode)
    {
        /* In controller mode, set the I2C target address */
        I2CControllerTargetAddrSet(object->baseAddr,
                              object->currentTransaction->targetAddress);

        if (TRUE == hwAttrs->enableIntr)
        {
            /* clear all interrupts */
            I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

            /* Enable interrupts on stop and error bits */
            I2CControllerIntEnableEx(object->baseAddr,
                                 I2C_INT_MASK_ARBITRATION_LOST |
                                 I2C_INT_MASK_NO_ACK           |
                                 I2C_INT_MASK_STOP_CONDITION);

            /* Start transfer in Transmit mode */
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
                                     I2C_INT_MASK_TRANSMIT_READY | I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST);

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
        else  /* POLLING MODE */
        {
            if(object->writeCountIdx != 0U)
            {
                /* wait for bus free */
                while(I2CControllerBusBusy(object->baseAddr))
                {}

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
                object->writeCountIdx--;

                /* generate start */
                I2CControllerStart(object->baseAddr);

                /* wait for bus busy */
                while(I2CControllerBusBusy(object->baseAddr) == false)
                {}
                while(object->writeCountIdx != 0U)
                {
                    /* wait for transmit ready or error */
                    while((I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_TRANSMIT_READY) == 0U) &&
                        (I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_ARBITRATION_LOST | I2C_INT_NO_ACK |
                        I2C_INT_STOP_CONDITION ) == 0U))
                    {}

                    /* if we get an error, do a stop */
                    errStat=I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_ARBITRATION_LOST | I2C_INT_NO_ACK |
                        I2C_INT_STOP_CONDITION );
                    if(errStat)
                    /* if we get an error, do a stop and return failure */
                    {
                       fatalError=1U;
                       break;
                    }
                    /* write byte and increase data pointer to next byte */
                    I2CControllerDataPut(object->baseAddr, *(object->writeBufIdx));
                    (object->writeBufIdx)++;

                    /* update number of bytes written */
                    object->writeCountIdx--;
                }

                if(fatalError == 0U) {
                /* wait for register access ready */
                while(I2CControllerIntStatusEx(object->baseAddr,
                    I2C_INT_ADRR_READY_ACESS) == 0U) {
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

                if(object->readCountIdx == 0U)
                {
                    /* generate stop when no data read */
                    I2CControllerStop(object->baseAddr);

                    /* wait for stop to happen */
                    while(I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_STOP_CONDITION) == 0U)
                    {}
                }
            }

            if((object->readCountIdx != 0U) && (status==I2C_STS_SUCCESS))
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
                while(I2CControllerBusBusy(object->baseAddr) == false)
                {}
                while(object->readCountIdx != 0U)
                {
                    /* wait for receive ready or error */
                    while((I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_RECV_READY) == 0U) &&
                        (I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_ARBITRATION_LOST | I2C_INT_NO_ACK |
                        I2C_INT_STOP_CONDITION) == 0U))
                    {}

                    /* if we get an error, do a stop and return failure */
                    errStat=I2CControllerIntStatusEx(object->baseAddr,
                        I2C_INT_ARBITRATION_LOST | I2C_INT_NO_ACK |
                        I2C_INT_STOP_CONDITION);
                    if(errStat) {
                       break;

                    }
                    /* read byte and increase data pointer to next byte */
                    *(object->readBufIdx) =
                        (uint8_t)I2CControllerDataGet(object->baseAddr);

                    object->readBufIdx++;
                    object->readCountIdx--;   /* update number of bytes read */
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
                /* generate stop when requested */
                I2CControllerStop(object->baseAddr);

                /* wait for stop to happen */
                while(I2CControllerIntStatusEx(object->baseAddr,
                    I2C_INT_STOP_CONDITION) == 0U)
                {}
            }
        }
    }
    /* In target mode */
    else
    {
        /* clear all interrupts */
        I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

        /* Currently target mode is supported only when interrupt is enabled */
        if (TRUE == hwAttrs->enableIntr)
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
    }

    return status;
}

int32_t I2C_transfer(I2C_Handle handle,
                            I2C_Transaction *transaction)
{
    int32_t             retVal = I2C_STS_ERR;
    uint8_t             ret_flag = 0U;
    uintptr_t           key;
    I2C_Object      *object = NULL;
    I2C_HwAttrs const  *hwAttrs = NULL;

    if ((handle != NULL) && (transaction != NULL))
    {
        /* Get the pointer to the object and hwAttrs */
        object = (I2C_Object*)handle->object;
        hwAttrs = (I2C_HwAttrs const *)handle->hwAttrs;

        if (transaction->timeout == 0U)
        {
            /* timeout cannot be NO_WAIT, set it to default value */
            transaction->timeout = SystemP_WAIT_FOREVER;
        }
    }
    else
    {
        ret_flag = 1U;
    }

    if ((ret_flag == 0U) &&
        ((transaction->writeCount != 0U) ||
         (transaction->readCount != 0U)))
    {
        if (object->i2cParams.transferMode == I2C_MODE_CALLBACK)
        {
            /* Check if a transfer is in progress */
            key = HwiP_disable();
            if (object->headPtr != NULL)
            {
                /* Transfer in progress. Update the message pointed by the tailPtr
                 * to point to the next message in the queue
                 */
                object->tailPtr->nextPtr = transaction;

                /* Update the tailPtr to point to the last message */
                object->tailPtr = transaction;

                /* I2C is still being used */
                HwiP_restore(key);
                retVal = I2C_STS_SUCCESS;
                ret_flag = 1U;
            }
            else
            {
                /* Store the headPtr indicating I2C is in use */
                object->headPtr = transaction;
                object->tailPtr = transaction;
                HwiP_restore(key);
            }
        }

        if(ret_flag == 0U)
        {
            /* Acquire the lock for this particular I2C handle */
            (void)SemaphoreP_pend(&object->mutex, SystemP_WAIT_FOREVER);

            /*
             * I2CSubArtic_primeTransfer is a longer process and
             * protection is needed from the I2C interrupt
             */
            if (TRUE == hwAttrs->enableIntr)
            {
                HwiP_disableInt((uint32_t)hwAttrs->intNum);
            }

            retVal = I2C_primeTransfer(handle, transaction);

            if (TRUE == hwAttrs->enableIntr)
            {
                HwiP_enableInt((uint32_t)hwAttrs->intNum);
            }
            if (object->i2cParams.transferMode == I2C_MODE_BLOCKING &&
                (TRUE == hwAttrs->enableIntr))
            {
                /*
                  * Wait for the transfer to complete here.
                  * It's OK to block from here because the I2C's Hwi will unblock
                  * upon errors
                  */
                retVal = SemaphoreP_pend(&object->transferComplete, transaction->timeout);

                if ( retVal != SystemP_SUCCESS)
                {
                    /* Transaction timed out or had some error in semaphore pend */
                    retVal = I2C_STS_ERR_TIMEOUT;
                }
                else
                {
                    /* Hwi handle has posted a 'transferComplete' check for Errors */
                    if (object->state == I2C_IDLE_STATE)
                    {

                    }
                    if ((object->intStatusErr & I2C_INT_ARBITRATION_LOST) != 0U)
                    {
                        retVal = I2C_STS_ERR_ARBITRATION_LOST;
                    }
                    else if ((object->intStatusErr & I2C_INT_NO_ACK) != 0U)
                    {
                        retVal = I2C_STS_ERR_NO_ACK;
                    }
                    else
                    {
                        retVal = I2C_STS_SUCCESS;
                    }
                }
            }
            /* Release the lock for this particular I2C handle */
            (void)SemaphoreP_post(&object->mutex);
        }
    }

    return (retVal);
}

static void I2C_transferCallback(I2C_Handle handle,
                          I2C_Transaction *msg,
                          int32_t transferStatus)
{
    I2C_Object   *object;

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object */
        object = (I2C_Object *)handle->object;

        /* Indicate transfer complete */
        SemaphoreP_post(&object->transferComplete);
    }
}

int32_t I2C_probe(I2C_Handle handle, uint32_t targetAddr)
{
    int32_t             retVal  = I2C_STS_ERR;
    uint32_t            regVal;
    I2C_Object         *object  = NULL;

    /* Input parameter validation */
    if (handle != NULL)
    {
        object  = (I2C_Object*)handle->object;

        /* Disable interrupts first */
        regVal = I2CControllerIntStatusEx(object->baseAddr, I2C_ALL_INTS);

        I2CControllerIntDisableEx(object->baseAddr, I2C_ALL_INTS);

        /* wait until bus not busy */
        if (I2C_waitForBb(object->baseAddr, I2C_DELAY_MED) != I2C_STS_SUCCESS)
        {
            retVal = I2C_STS_ERR;
        }
        else
        {
            /* set target address */
            I2CControllerTargetAddrSet(object->baseAddr, (uint32_t) targetAddr);

            /* try to write one byte */
            I2CControllerDataPut(object->baseAddr, (uint8_t) 0U);
            I2CSetDataCount(object->baseAddr, (uint32_t) 1U);

            /* stop bit needed here */
            I2CControllerControl(object->baseAddr,I2C_CFG_MASK_STOP,(
                             I2C_CFG_CMD_TX | I2C_CFG_CMD_START | I2C_CFG_CMD_STOP));

            /* enough delay for the NACK bit set */
            ClockP_usleep(I2C_DELAY_BIG);

            if (0U == I2CControllerIntStatusEx(object->baseAddr, I2C_INT_NO_ACK))
            {
                retVal = I2C_STS_SUCCESS;        /* success case */
            }
            else
            {
                /* Clear sources*/
                I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

                /* finish up xfer */
                I2CControllerStop(object->baseAddr);
                (void)I2C_waitForBb(object->baseAddr, I2C_DELAY_MED);

                retVal = I2C_STS_ERR;         /* Error case */
            }

            I2CSetDataCount(object->baseAddr, 0);
            I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);
        }

        /* wait for bus free */
        if (I2C_waitForBb(object->baseAddr, I2C_DELAY_MED) != I2C_STS_SUCCESS)
        {
            retVal = I2C_STS_ERR;
        }

        /* Enable interrupts now */
        I2CControllerIntEnableEx(object->baseAddr, regVal);
    }
    return (retVal);
}

static int32_t I2C_waitForBb(uint32_t baseAddr, uint32_t timeout)
{
    uint32_t            stat;
    int32_t             retVal = I2C_STS_SUCCESS;
    volatile uint32_t   bbtimeout = timeout;

    if(bbtimeout > 0U)
    {
        /* Clear current interrupts...*/
        I2CControllerIntClearEx(baseAddr, I2C_ALL_INTS);

    while (bbtimeout > 0U)
    {
        stat = I2CControllerIntStatusEx(baseAddr, I2C_INT_BUS_BUSY);
        if (stat == 0U)
        {
            break;
        }
        bbtimeout = bbtimeout - 1U;
        I2CControllerIntClearEx(baseAddr, stat);
    }

    if (timeout > 0U)
    {
        if (bbtimeout == 0U)
        {
            retVal = I2C_STS_ERR;
        }
    }

        /* clear delayed stuff*/
        I2CControllerIntClearEx(baseAddr, I2C_ALL_INTS);
    }
    else
    {
        while(I2CControllerBusBusy(baseAddr) == 1)
        {
        }
    }

    return retVal;
}

int32_t I2C_setBusFrequency(I2C_Handle handle, uint32_t busFrequency)
{
    int32_t             retVal      = I2C_STS_SUCCESS;
    I2C_HwAttrs const  *hwAttrs     = NULL;
    I2C_Object         *object      = NULL;
    uint32_t            outputClk   = 0;
    uint32_t            internalClk = 0;

    /* Input parameter validation */
    if(NULL != handle)
    {
        /* Get the pointer to the object and hwAttrs */
        hwAttrs = (I2C_HwAttrs const *)handle->hwAttrs;
        object  = (I2C_Object*)handle->object;

        /* Acquire the lock for this particular I2C handle */
        (void)SemaphoreP_pend(&object->mutex, SystemP_WAIT_FOREVER);

        /* Put i2c in reset/disabled state */
        I2CControllerDisable(object->baseAddr);

        /* Internal clock should be between 6.7MHz and 13.3MHz.
           Try to get the integer divider for the PSC and ICCLK.
           Deriving the output clock for 100KHz and 400KHz -
            If input clock is 200MHz or 96MHz  Configure internal clock for 8MHz.
            If input clock is 150MHz/others Configure internal clock for 12.5Mhz. */
        if ((hwAttrs->funcClk == I2C_MODULE_FUNC_CLK_200MHZ) ||
            (hwAttrs->funcClk == I2C_MODULE_FUNC_CLK_96MHZ))
        {
            internalClk = I2C_MODULE_INTERNAL_CLK_8MHZ;
        }
        else
        {
            internalClk = I2C_MODULE_INTERNAL_CLK_12P5MHZ;
        }

        /* Extract bit rate from the input parameter */
        switch(busFrequency)
        {
           case (uint32_t)I2C_100KHZ:
           {
               outputClk = 100000U;
               break;
           }

           case (uint32_t)I2C_400KHZ:
           {
               outputClk = 400000U;
               break;
           }
           default:
           {
               outputClk = 100000U;
               break;
           }
        }

        /* Set the I2C configuration */
        I2CControllerInitExpClk(object->baseAddr, hwAttrs->funcClk, internalClk,
            outputClk);

        /* Clear any pending interrupts */
        I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

        /* Mask off all interrupts */
        I2CControllerIntDisableEx(object->baseAddr, I2C_ALL_INTS);

        /* Enable the I2C Controller for operation */
        I2CControllerEnable(object->baseAddr);

        I2COwnAddressSet(object->baseAddr, hwAttrs->ownTargetAddr);

        retVal = I2C_STS_SUCCESS;

        /* Release the lock for this particular I2C handle */
        (void)SemaphoreP_post(&object->mutex);
    }
    else
    {
        retVal = I2C_STS_ERR;
    }

    return retVal;
}

/*
 *  Copyright (C)2018-2024 Texas Instruments Incorporated
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
 *  \file   i2c_v0.c
 *
 *  \brief  File containing I2C Driver APIs implementation for V0.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/i2c.h>
#include <kernel/dpl/AddrTranslateP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define I2C_DELAY_MED                   ((uint32_t) 10000U)
#define I2C_DELAY_BIG                   ((uint32_t) 30000U)
#define I2C_DELAY_SMALL                 ((uint32_t) 5000U)
#define I2C_DELAY_USEC                  ((uint32_t) 250U)

/*
 * Maximum number of loop count to handle in the same ISR, which is
 * to process multiple interrupts (ARDY, RRDY, XRDY) in the ISR to
 * reduce interrupt count
 *
 * Keep at least 3 to support optimal RX followed by TX scenario
 * Keep at least 2 to support optimal RX scenario
 */
#define I2C_MAX_CONSECUTIVE_ISRS      (1U)

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

    I2C_MODE_BLOCKING,              /* transferMode */
    NULL,                           /* transferCallbackFxn */
    I2C_100KHZ,                     /* bitRate */
};

/* Default I2C Memory Transaction Structure */
const I2C_Mem_Transaction I2C_defaultMemTransaction = {

    0,                              /* memAddr */
    I2C_MEM_ADDR_SIZE_8_BITS,       /* memAddrSize */
    NULL,                           /* buffer */
    0,                              /* size */
    I2C_MEM_TXN_DIR_INVALID         /* memDataDir */
};

/* Default I2C transaction parameters structure */
const I2C_Transaction I2C_defaultTransaction = {

    NULL,                           /* writeBuf */
    0,                              /* writeCount */
    NULL,                           /* readBuf */
    0,                              /* readCount */
    0,                              /* targetAddress */
    NULL,                           /* nextPtr */
    NULL,                           /* arg */
    SystemP_WAIT_FOREVER,           /* timeout */
    (bool)true,                     /* controllerMode */
    (bool)false,                    /* expandSA */
    (bool)false,                    /* memTxnEnable */
    NULL,                           /* memTransaction */
    I2C_STS_SUCCESS                 /* status */
};

/* ========================================================================== */
/*                     Internal Function Declarations                         */
/* ========================================================================== */

static inline void I2C_transferCallback(I2C_Handle handle,
                                        I2C_Transaction *msg,
                                        int32_t transferStatus);

static void I2C_completeCurrTransfer(I2C_Handle handle, int32_t xferStatus);

/* Transfer complete callback functions */
static void I2C_LLD_transferCompleteCallback(void *args,
                                             const I2CLLD_Message * msg,
                                             int32_t transferStatus);

static void I2C_LLD_targetTransferCompleteCallback(void *args,
                                    const I2CLLD_targetTransaction * targetTxn,
                                    int32_t transferStatus);

/* HWI Function */
static void I2C_hwiFxn(void* arg);

/* Internal Transfer Functions */
static int32_t I2C_primeTransfer(I2C_Handle handle,
                                 I2C_Transaction *transaction);

static int32_t I2C_mem_primeTransfer(I2C_Handle handle,
                                     I2C_Transaction *transaction);
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
/*                       API Function Definitions                             */
/* ========================================================================== */

void I2C_init(void)
{
    I2C_Handle handle;
    uint32_t i;

    /* Call init function for each config */
    for (i = 0; i < gI2cConfigNum; i++)
    {
        handle = &gI2cConfig[i];
        /* Input parameter validation */
        if (handle->object != NULL)
        {
            /* Mark the object as available */
            handle->object->isOpen = (bool)false;
        }
    }
    /* Create driver lock */
    (void)SemaphoreP_constructMutex(&gI2cDrvObj.lockObj);
    gI2cDrvObj.lock = &gI2cDrvObj.lockObj;
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

void I2C_Params_init(I2C_Params *params)
{
    /* Input parameter validation */
    if (params != NULL)
    {
        *params = I2C_defaultParams;
    }
}

I2C_Handle I2C_open(uint32_t idx, const I2C_Params *params)
{
    I2C_Handle          handle = NULL;
    I2C_Object          *object = NULL;
    I2C_HwAttrs const   *hwAttrs = NULL;
    I2CLLD_Handle       i2cLldHandle;
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i;

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
    (void)SemaphoreP_pend(&gI2cDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(SystemP_SUCCESS == status)
    {
        object = (I2C_Object*)handle->object;
        DebugP_assert(NULL != object);
        DebugP_assert(NULL != handle->hwAttrs);
        hwAttrs = (I2C_HwAttrs const *)handle->hwAttrs;
        if(true == object->isOpen)
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
        /* Store the I2C parameters */
        if (params == NULL) {
            /* No params passed in, so use the defaults */
            I2C_Params_init(&(object->i2cParams));
        }
        else {
            /* Copy the params contents */
            object->i2cParams = *params;
        }

        object->i2cLldHandle = &object->i2cLldObject;
        i2cLldHandle = object->i2cLldHandle;

        i2cLldHandle->baseAddr = handle->hwAttrs->baseAddr;
        i2cLldHandle->intrNum = handle->hwAttrs->intNum;
        i2cLldHandle->bitRate = object->i2cParams.bitRate;
        i2cLldHandle->funcClk = handle->hwAttrs->funcClk;
        for(i=0; i<I2C_MAX_NUM_OWN_TARGET_ADDR; i++)
        {
            i2cLldHandle->ownTargetAddr[i] = handle->hwAttrs->ownTargetAddr[i];
        }
        i2cLldHandle->state = I2C_STATE_RESET;

        i2cLldHandle->Clock_getTicks = ClockP_getTicks;
        i2cLldHandle->Clock_usecToTicks = ClockP_usecToTicks;
        i2cLldHandle->Clock_uSleep = ClockP_usleep;

        i2cLldHandle->targetTransferCompleteCallback = I2C_LLD_targetTransferCompleteCallback;

        if (true == hwAttrs->enableIntr)
        {
            i2cLldHandle->transferCompleteCallback = I2C_LLD_transferCompleteCallback;

            HwiP_Params hwiPrms;

            /* Initialize with defaults */
            HwiP_Params_init(&hwiPrms);

            /* Populate the interrupt parameters */
            hwiPrms.args = (void *)handle;
            hwiPrms.callback = &I2C_hwiFxn;
            /* Event going in to CPU */
            hwiPrms.eventId = (uint16_t)hwAttrs->eventId;
            hwiPrms.intNum = hwAttrs->intNum;
            hwiPrms.isPulse = 1;
            hwiPrms.priority = 4U;
            hwiPrms.isFIQ = 0;

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

        /* Clear the head pointer */
        object->headPtr = NULL;
        object->tailPtr = NULL;

        /* Initialize LLD driver */
        if(I2C_lld_init(i2cLldHandle) == I2C_STS_SUCCESS)
        {
            status = SystemP_SUCCESS;
            /* Store HLD handle insinde LLD Object */
            i2cLldHandle->args = handle;
            /* Specify the idle state for this I2C peripheral */
            object->state = I2C_STATE_IDLE;
        }
        else
        {
            status = SystemP_FAILURE;
        }

        DebugP_assert(status==SystemP_SUCCESS);
    }

    SemaphoreP_post(&gI2cDrvObj.lockObj);
    return (handle);
}

void I2C_close(I2C_Handle handle)
{
    I2C_Object          *object = NULL;
    I2C_HwAttrs const   *hwAttrs = NULL;
    int32_t             status = SystemP_SUCCESS;

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object */
        object = (I2C_Object*)handle->object;
        hwAttrs = (I2C_HwAttrs const *)handle->hwAttrs;

        DebugP_assert(NULL != gI2cDrvObj.lock);
        (void)SemaphoreP_pend(&gI2cDrvObj.lockObj, SystemP_WAIT_FOREVER);

        /* Check to see if a I2C transaction is in progress */
        if (object->headPtr == NULL)
        {
            if(I2C_lld_deInit(object->i2cLldHandle) == I2C_STS_SUCCESS)
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
            DebugP_assert(status == SystemP_SUCCESS);

            if (true == hwAttrs->enableIntr)
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

void I2C_Memory_Transaction_init(I2C_Mem_Transaction *memTransaction)
{
    *memTransaction = I2C_defaultMemTransaction;
}

void I2C_Transaction_init(I2C_Transaction *transaction)
{
    *transaction = I2C_defaultTransaction;
}

int32_t I2C_transfer(I2C_Handle handle,
                            I2C_Transaction *transaction)
{
    int32_t             retVal = I2C_STS_ERR;
    uint8_t             ret_flag = 0U;
    uintptr_t           key;
    I2C_Object          *object = NULL;
    I2C_HwAttrs const   *hwAttrs = NULL;

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

    if ((ret_flag == 0U) && (   (transaction->writeCount != 0U)    ||
                                (transaction->readCount != 0U)     ||
                                (transaction->memTxnEnable == true)))
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
            if (true == hwAttrs->enableIntr)
            {
                (void)HwiP_disableInt((uint32_t)hwAttrs->intNum);
            }

            /* Conditional statement for memory transaction and normal transaction */
            if(transaction->memTxnEnable)
            {
                retVal = I2C_mem_primeTransfer(handle, transaction);
            }
            else
            {
                retVal = I2C_primeTransfer(handle, transaction);
            }

            if (true == hwAttrs->enableIntr)
            {
                (void)HwiP_enableInt((uint32_t)hwAttrs->intNum);
            }

            if ((retVal == I2C_STS_SUCCESS) &&
                (object->i2cParams.transferMode == I2C_MODE_BLOCKING) &&
                (true == hwAttrs->enableIntr))
            {
                /*
                  * Wait for the transfer to complete here.
                  * It's OK to block from here because the I2C's Hwi will unblock
                  * upon errors
                  */
                retVal = SemaphoreP_pend(   &object->transferComplete,
                                            transaction->timeout);

                if ( retVal != SystemP_SUCCESS)
                {
                    /* Transaction timed out or had some error in semaphore pend */
                    retVal = SystemP_TIMEOUT;
                    (void)I2C_recoverBus(handle, I2C_DELAY_SMALL);
                }
            }

            /* Polling mode Case */
            if ((false == hwAttrs->enableIntr))
            {
                transaction->status = retVal;

                if(retVal == I2C_STS_SUCCESS)
                {
                    retVal = SystemP_SUCCESS;
                }
                else if(retVal == I2C_STS_ERR_TIMEOUT)
                {
                    retVal = SystemP_TIMEOUT;
                }
                else
                {
                    retVal = SystemP_FAILURE;
                }
            }
            /* Release the lock for this particular I2C handle */
            (void)SemaphoreP_post(&object->mutex);
        }
    }

    return (retVal);
}

int32_t I2C_probe(I2C_Handle handle, uint32_t targetAddr)
{
    int32_t retVal = SystemP_FAILURE;
    I2C_Object *object = NULL;
    I2CLLD_Handle i2cLldHandle = NULL;

    if(NULL != handle)
    {
        object = (I2C_Object*)handle->object;
        i2cLldHandle = object->i2cLldHandle;
        /* Acquire the lock for this particular I2C handle */
        (void)SemaphoreP_pend(&object->mutex, SystemP_WAIT_FOREVER);
        /* Invoke LLD API */
        if(I2C_lld_probe(i2cLldHandle, targetAddr) == I2C_STS_SUCCESS)
        {
            retVal = SystemP_SUCCESS;
        }
        /* Release the lock for this particular I2C handle */
        (void)SemaphoreP_post(&object->mutex);
    }
    return (retVal);
}

int32_t I2C_setBusFrequency(I2C_Handle handle, uint32_t busFrequency)
{
    int32_t retVal = SystemP_FAILURE;
    I2C_Object *object = NULL;
    I2CLLD_Handle i2cLldHandle = NULL;

    if(NULL != handle)
    {
        object = (I2C_Object*)handle->object;
        i2cLldHandle = object->i2cLldHandle;
        /* Acquire the lock for this particular I2C handle */
        (void)SemaphoreP_pend(&object->mutex, SystemP_WAIT_FOREVER);
        /* Invoke LLD API*/
        if(I2C_lld_setBusFrequency(i2cLldHandle, busFrequency) == I2C_STS_SUCCESS)
        {
            retVal = SystemP_SUCCESS;
        }
        /* Release the lock for this particular I2C handle */
        (void)SemaphoreP_post(&object->mutex);
    }
    return retVal;
}

int32_t I2C_recoverBus(I2C_Handle handle, uint32_t i2cDelay)
{
    I2C_Object   *object = NULL;
    int32_t status = I2C_STS_SUCCESS;

    if(handle == NULL)
    {
        status = I2C_STS_ERR;
    }

    if(I2C_STS_SUCCESS == status)
    {
        object = (I2C_Object*)handle->object;
        if(object == NULL)
        {
            status = I2C_STS_ERR;
        }
    }

    if (I2C_STS_SUCCESS == status)
    {
        /* Acquire the lock for this particular I2C handle */
        (void)SemaphoreP_pend(&object->mutex, SystemP_WAIT_FOREVER);

        status = I2C_lld_recoverBus(object->i2cLldHandle, i2cDelay);

        /* Release the lock for this particular I2C handle */
        (void)SemaphoreP_post(&object->mutex);
    }

    if (status == I2C_STS_SUCCESS)
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
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

/* ========================================================================== */
/*                      Internal Function Definitions                         */
/* ========================================================================== */

static int32_t I2C_primeTransfer(   I2C_Handle handle,
                                    I2C_Transaction *transaction)
{
    I2C_Object *object = NULL;
    I2C_HwAttrs const *hwAttrs = NULL;

    /* Get the pointer to the object and hwAttrs */
    object = (I2C_Object*)handle->object;
    hwAttrs = (I2C_HwAttrs const *)handle->hwAttrs;

    I2CLLD_Handle i2cLldHandle;
    i2cLldHandle = object->i2cLldHandle;

    int32_t status = I2C_STS_SUCCESS;

    /* Store the new internal counters and pointers */
    object->currentTransaction = transaction;

    /* Controller Mode */
    if(object->currentTransaction->controllerMode)
    {
        (void)I2C_lld_Transaction_init(&i2cLldHandle->i2ctxn);
        i2cLldHandle->i2ctxn.writeBuf = (uint8_t*)object->currentTransaction->writeBuf;
        i2cLldHandle->i2ctxn.writeCount = (uint32_t)object->currentTransaction->writeCount;
        i2cLldHandle->i2ctxn.readBuf = (uint8_t*)object->currentTransaction->readBuf;
        i2cLldHandle->i2ctxn.readCount = (uint32_t)object->currentTransaction->readCount;

        (void)I2C_lld_Message_init(&i2cLldHandle->i2cMsg);
        i2cLldHandle->i2cMsg.txn = &i2cLldHandle->i2ctxn;
        i2cLldHandle->i2cMsg.txnCount = 1U;
        i2cLldHandle->i2cMsg.targetAddress = object->currentTransaction->targetAddress;
        i2cLldHandle->i2cMsg.timeout = object->currentTransaction->timeout;
        i2cLldHandle->i2cMsg.expandSA = object->currentTransaction->expandSA;

        if (true == hwAttrs->enableIntr)
        {
            status = I2C_lld_transferIntr(i2cLldHandle, &(i2cLldHandle->i2cMsg));
        }
        else
        {
            status = I2C_lld_transferPoll(i2cLldHandle, &(i2cLldHandle->i2cMsg));
            object->currentTransaction->status = status;
        }
    }

    /* Target Mode */
    else
    {
        i2cLldHandle->i2cTargetTransaction.writeBuf = (uint8_t*)transaction->writeBuf;
        i2cLldHandle->i2cTargetTransaction.writeCount = (uint32_t)transaction->writeCount;

        i2cLldHandle->i2cTargetTransaction.readBuf = (uint8_t*)transaction->readBuf;
        i2cLldHandle->i2cTargetTransaction.readCount = (uint32_t)transaction->readCount;

        i2cLldHandle->i2cTargetTransaction.timeout = transaction->timeout;
        i2cLldHandle->i2cTargetTransaction.expandSA = transaction->expandSA;

        status = I2C_lld_targetTransferIntr(i2cLldHandle, &(i2cLldHandle->i2cTargetTransaction));
    }

    return status;
}

static int32_t I2C_mem_primeTransfer(   I2C_Handle handle,
                                        I2C_Transaction *transaction)
{
    I2C_Object *object = NULL;
    I2C_HwAttrs const *hwAttrs = NULL;

    /* Get the pointer to the object and hwAttrs */
    object = (I2C_Object*)handle->object;
    hwAttrs = (I2C_HwAttrs const *)handle->hwAttrs;

    I2CLLD_Handle i2cLldHandle;
    i2cLldHandle = object->i2cLldHandle;

    int32_t status = I2C_STS_SUCCESS;

    object->currentTransaction = transaction;

    /* Store the new internal counters and pointers */
    /* Controller Mode */
    if(object->currentTransaction->controllerMode)
    {

        I2C_Memory_ExtendedParams mem_extendedParams;

        mem_extendedParams.memAddr =
                        object->currentTransaction->memTransaction->memAddr;
        mem_extendedParams.memAddrSize =
                        object->currentTransaction->memTransaction->memAddrSize;



        mem_extendedParams.extendedParams.deviceAddress =
                        object->currentTransaction->targetAddress;
        mem_extendedParams.extendedParams.buffer =
                        object->currentTransaction->memTransaction->buffer;
        mem_extendedParams.extendedParams.size =
                        object->currentTransaction->memTransaction->size;
        mem_extendedParams.extendedParams.expandSA =
                        object->currentTransaction->expandSA;

        /* INTERRUPT MODE */
        if (true == hwAttrs->enableIntr)
        {
            if(object->currentTransaction->memTransaction->memDataDir ==
                                                            I2C_MEM_TXN_DIR_TX)
            {
                status = I2C_lld_mem_writeIntr(i2cLldHandle,
                                                        &mem_extendedParams);
            }
            else if(object->currentTransaction->memTransaction->memDataDir ==
                                                            I2C_MEM_TXN_DIR_RX)
            {
                status = I2C_lld_mem_readIntr(i2cLldHandle,
                                                        &mem_extendedParams);
            }
            else
            {
                status = I2C_STS_ERR_INVALID_PARAM;
            }
        }
        /* POLLING MODE */
        else
        {
            if(object->currentTransaction->memTransaction->memDataDir ==
                                                            I2C_MEM_TXN_DIR_TX)
            {
                status = I2C_lld_mem_write(i2cLldHandle, &mem_extendedParams,
                                           object->currentTransaction->timeout);
            }
            else if(object->currentTransaction->memTransaction->memDataDir ==
                                                            I2C_MEM_TXN_DIR_RX)
            {
                status = I2C_lld_mem_read(i2cLldHandle, &mem_extendedParams,
                                          object->currentTransaction->timeout);
            }
            else
            {
                status = I2C_STS_ERR_INVALID_PARAM;
            }
        }
    }

    /* Target Mode */
    else
    {
        status = I2C_STS_ERR;
    }

    return status;
}

static inline void I2C_transferCallback(I2C_Handle handle,
                                        I2C_Transaction *msg,
                                        int32_t transferStatus)
{
    I2C_Object *object;

    /* Input parameter validation */
    if ((handle != NULL) && (msg != NULL))
    {
        /* Get the pointer to the object */
        object = (I2C_Object *)handle->object;
        /* Indicate transfer complete */
        SemaphoreP_post(&object->transferComplete);
    }
}

static void I2C_completeCurrTransfer(I2C_Handle handle, int32_t xferStatus)
{
    I2C_Object     *object = (I2C_Object*)handle->object;

    object->state = I2C_STATE_IDLE;

    if(object->currentTransaction != NULL)
    {
        object->currentTransaction->status = xferStatus;
        /* Callback to application or post semaphore */
        object->i2cParams.transferCallbackFxn(  handle,
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
        (void)I2C_primeTransfer(handle, object->headPtr);
        }
    }
}

/* LLD transfer complete callback called from LLD in Blocking Mode */
static void I2C_LLD_transferCompleteCallback (void * args,
                                              const I2CLLD_Message * msg,
                                              int32_t transferStatus)
{
    I2CLLD_Handle i2cLldHandle = (I2CLLD_Handle)args;

    if((NULL_PTR != i2cLldHandle) && (NULL_PTR != msg))
    {
        I2C_Handle i2cHldhandle = (I2C_Handle)i2cLldHandle->args;

        if(NULL != i2cHldhandle)
        {
            I2C_completeCurrTransfer(i2cHldhandle, transferStatus);
        }
    }
}

static void I2C_LLD_targetTransferCompleteCallback (void * args,
                                const I2CLLD_targetTransaction * targetTxn,
                                int32_t transferStatus)
{
    I2CLLD_Handle   lldhandle = (I2CLLD_Handle)args;
    I2C_Config      *config;
    I2C_Object      *object;

    if((NULL_PTR != lldhandle) && (NULL_PTR != targetTxn))
    {
        I2C_Handle hldhandle = (I2C_Handle)lldhandle->args;

        config = (I2C_Config *) hldhandle;
        object = config->object;

        if(NULL != hldhandle)
        {
            if(transferStatus != I2C_STS_RESTART)
            {
                I2C_completeCurrTransfer(hldhandle, transferStatus);
            }
            else
            {
                if((object->i2cParams.transferMode) == I2C_MODE_CALLBACK)
                {
                    object->i2cParams.transferCallbackFxn(hldhandle,
                                                        object->currentTransaction,
                                                        transferStatus);
                }
                else
                {
                    /* No Code */
                }
            }
        }
    }
}

static void I2C_hwiFxn(void* arg)
{
    I2C_Handle      handle = (I2C_Handle)arg;
    I2C_Object      *object = NULL;

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object */
        object = (I2C_Object *)handle->object;

        if (object->currentTransaction->controllerMode)
        {
            I2C_lld_controllerIsr(object->i2cLldHandle);
        }
        else
        {
            I2C_lld_targetIsr(object->i2cLldHandle);
        }
    }
    return;
}


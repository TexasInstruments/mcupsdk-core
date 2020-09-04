/*
 *  Copyright (C)2018-2021 Texas Instruments Incorporated
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
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_i2c.h>
#include <kernel/dpl/AddrTranslateP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define I2C_MODULE_INTERNAL_CLK_4MHZ    (4000000U)
#define I2C_MODULE_INTERNAL_CLK_12MHZ   (12000000U)
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
    0,                            /* slaveAddress */
    NULL,                         /* nextPtr */
    NULL,                         /* arg */
    SystemP_WAIT_FOREVER,         /* timeout */
    (bool)true,                   /* masterMode */
    (bool)false                   /* expandSA */
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

bool        I2C_checkTimeout(uint32_t *pUsecCnt);
void        I2C_hwiFxn(void* arg);
void        I2C_hwiFxnSlave(I2C_Handle handle);
void        I2C_hwiFxnMaster(I2C_Handle handle);
void        I2C_completeCurrTransfer(I2C_Handle handle, int32_t xferStatus);
int32_t     I2C_primeTransfer(I2C_Handle handle,
                              I2C_Transaction *transaction);
void        I2C_transferCallback(I2C_Handle handle,
                                  I2C_Transaction *msg,
                                  int32_t transferStatus);
int32_t     I2C_waitForBb(uint32_t baseAddr, uint32_t timeout);
void        I2C_udelay(uint32_t delay);
int32_t     I2C_resetCtrl(I2C_Handle handle);
int32_t     I2C_ctrlInit(I2C_Handle handle);
uint32_t    I2C_waitForPin(I2C_Handle  handle,
                           uint32_t    flag,
                           uint32_t   *pTimeout);

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

void I2CMasterInitExpClk(uint32_t baseAddr,
                         uint32_t sysClk,
                         uint32_t internalClk,
                         uint32_t outputClk)
{
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
    HW_WR_REG32(baseAddr + I2C_PSC, prescalar);

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
        HW_WR_REG32(baseAddr + I2C_SCLL, (div_l - 7U) << 8U);
        HW_WR_REG32(baseAddr + I2C_SCLH, (div_h - 5U) << 8U);
    }
    else
    {
        HW_WR_REG32(baseAddr + I2C_SCLL, div_l - 7U);
        HW_WR_REG32(baseAddr + I2C_SCLH, div_h - 5U);
    }
}

void I2CMasterEnable(uint32_t baseAddr)
{
    /* Bring the I2C module out of reset */
    HW_WR_FIELD32(baseAddr + I2C_CON, I2C_CON_I2C_EN,
                  I2C_CON_I2C_EN_ENABLE);
}

void I2CMasterEnableFreeRun(uint32_t baseAddr)
{
    /* Set the I2C module in free running mode */
    HW_WR_FIELD32(baseAddr + I2C_SYSTEST, I2C_SYSTEST_FREE,
                  I2C_SYSTEST_FREE_FREE);
}

void I2CMasterSetSysTest(uint32_t baseAddr, uint32_t sysTest)
{
    HW_WR_REG32(baseAddr + I2C_SYSTEST, sysTest);
}

uint32_t I2CMasterGetSysTest(uint32_t baseAddr)
{
    uint32_t sysTest;

    sysTest = HW_RD_REG32(baseAddr + I2C_SYSTEST);

    return (sysTest);
}

void I2CMasterDisable(uint32_t baseAddr)
{
    /* Put I2C module in reset */
    HW_WR_FIELD32(baseAddr + I2C_CON, I2C_CON_I2C_EN,
                  I2C_CON_I2C_EN_DISABLE);
}

int32_t I2CMasterBusBusy(uint32_t baseAddr)
{
    uint32_t status;

    if (HW_RD_FIELD32(baseAddr + I2C_IRQSTATUS_RAW,
                      I2C_IRQSTATUS_RAW_BB) == I2C_IRQSTATUS_RAW_BB_SET)
    {
        status = 1U;
    }
    else
    {
        status = 0U;
    }

    return (int32_t) status;
}

void I2CMasterControl(uint32_t baseAddr, uint32_t cmd)
{
    HW_WR_REG32(baseAddr + I2C_CON, cmd | I2C_CON_I2C_EN_MASK);
}

void I2CMasterStart(uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + I2C_CON, I2C_CON_STT, I2C_CON_STT_STT);
}

void I2CMasterStop(uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + I2C_CON, I2C_CON_STP, I2C_CON_STP_STP);
}

void I2CMasterIntEnableEx(uint32_t baseAddr, uint32_t intFlag)
{
    uint32_t i2cRegValue = HW_RD_REG32(baseAddr + I2C_IRQENABLE_SET);
    i2cRegValue |= intFlag;
    HW_WR_REG32(baseAddr + I2C_IRQENABLE_SET, i2cRegValue);
}

void I2CSlaveIntEnableEx(uint32_t baseAddr, uint32_t intFlag)
{
    uint32_t i2cRegValue = HW_RD_REG32(baseAddr + I2C_IRQENABLE_SET);
    i2cRegValue |= intFlag;
    HW_WR_REG32(baseAddr + I2C_IRQENABLE_SET, i2cRegValue);
}

void I2CMasterIntDisableEx(uint32_t baseAddr, uint32_t intFlag)
{
    HW_WR_REG32(baseAddr + I2C_IRQENABLE_CLR, intFlag);
}

void I2CSlaveIntDisableEx(uint32_t baseAddr, uint32_t intFlag)
{
    HW_WR_REG32(baseAddr + I2C_IRQENABLE_CLR, intFlag);
}

uint32_t I2CMasterIntStatus(uint32_t baseAddr)
{
    return HW_RD_REG32(baseAddr + I2C_IRQSTATUS);
}

uint32_t I2CMasterIntRawStatus(uint32_t baseAddr)
{
    return HW_RD_REG32(baseAddr + I2C_IRQSTATUS_RAW);
}

uint32_t I2CSlaveIntRawStatus(uint32_t baseAddr)
{
    return HW_RD_REG32(baseAddr + I2C_IRQSTATUS_RAW);
}

uint32_t I2CMasterIntRawStatusEx(uint32_t baseAddr, uint32_t intFlag)
{
    return (HW_RD_REG32(baseAddr + I2C_IRQSTATUS_RAW) & intFlag);
}

void I2CMasterIntClearEx(uint32_t baseAddr, uint32_t intFlag)
{
    HW_WR_REG32(baseAddr + I2C_IRQSTATUS, intFlag);
}

void I2CSlaveIntClearEx(uint32_t baseAddr, uint32_t intFlag)
{
    HW_WR_REG32(baseAddr + I2C_IRQSTATUS, intFlag);
}

uint32_t I2CGetEnabledIntStatus(uint32_t baseAddr, uint32_t intFlag)
{
    return (HW_RD_REG32(baseAddr + I2C_IRQENABLE_SET) & intFlag);
}

void I2CMasterSlaveAddrSet(uint32_t baseAddr, uint32_t slaveAdd)
{
    /*Set the address of the slave with which the master will communicate.*/
    HW_WR_REG32(baseAddr + I2C_SA, slaveAdd);
}

void I2CSetDataCount(uint32_t baseAddr, uint32_t count)
{
    HW_WR_REG32(baseAddr + I2C_CNT, count);
}

uint32_t I2CDataCountGet(uint32_t baseAddr)
{
    return HW_RD_REG32(baseAddr + I2C_CNT);
}

void I2CFIFOClear(uint32_t baseAddr, uint32_t flag)
{
    if (I2C_TX_MODE == flag)
    {
        HW_WR_FIELD32(baseAddr + I2C_BUF, I2C_BUF_TXFIFO_CLR,
                      I2C_BUF_TXFIFO_CLR_RSTMODE);
    }
    else
    {
        HW_WR_FIELD32(baseAddr + I2C_BUF, I2C_BUF_RXFIFO_CLR,
                      I2C_BUF_RXFIFO_CLR_RSTMODE);
    }
}

uint32_t I2CBufferStatus(uint32_t baseAddr, uint32_t flag)
{
    uint32_t status;

    switch (flag)
    {
        case I2C_TX_BUFFER_STATUS:
            status = HW_RD_FIELD32(baseAddr + I2C_BUFSTAT, I2C_BUFSTAT_TXSTAT);
            break;

        case I2C_RX_BUFFER_STATUS:
            status = HW_RD_FIELD32(baseAddr + I2C_BUFSTAT, I2C_BUFSTAT_RXSTAT);
            break;

        case I2C_FIFO_DEPTH:
            status = HW_RD_FIELD32(baseAddr + I2C_BUFSTAT, I2C_BUFSTAT_FIFODEPTH);
            break;

        default:
            /* Invalid input */
            status = 0U;
            break;
    }

    return status;
}

void I2COwnAddressSet(uint32_t baseAddr, uint32_t ownAdd, uint32_t flag)
{
    switch (flag)
    {
        case I2C_OWN_ADDR_0:
            HW_WR_REG32(baseAddr + I2C_OA, ownAdd);
            break;

        case I2C_OWN_ADDR_1:
            HW_WR_REG32(baseAddr + I2C_OA1, ownAdd);
            break;

        case I2C_OWN_ADDR_2:
            HW_WR_REG32(baseAddr + I2C_OA2, ownAdd);
            break;

        case I2C_OWN_ADDR_3:
            HW_WR_REG32(baseAddr + I2C_OA3, ownAdd);
            break;

        default:
            /* Invalid input */
            break;
    }
}

void I2CSoftReset(uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + I2C_SYSC, I2C_SYSC_SRST, I2C_SYSC_SRST_RSTMODE);
}

void I2CAutoIdleEnable(uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + I2C_SYSC, I2C_SYSC_AUTOIDLE,
                  I2C_SYSC_AUTOIDLE_ENABLE);
}

void I2CAutoIdleDisable(uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + I2C_SYSC, I2C_SYSC_AUTOIDLE, I2C_SYSC_AUTOIDLE_DISABLE);
}

uint32_t I2CSystemStatusGet(uint32_t baseAddr)
{
    return (HW_RD_REG32(baseAddr + I2C_SYSS) & I2C_SYSS_RDONE_MASK);
}

void I2CMasterDataPut(uint32_t baseAddr, uint8_t data)
{
    /*write data to be transmitted to Data transmit register */
    HW_WR_REG32(baseAddr + I2C_DATA, (uint32_t) data);
}

uint8_t I2CMasterDataGet(uint32_t baseAddr)
{
    uint8_t rData;

    rData = (uint8_t) HW_RD_REG32(baseAddr + I2C_DATA);
    return rData;
}

uint8_t I2CSlaveDataGet(uint32_t baseAddr)
{
    uint8_t rData;

    rData = (uint8_t) HW_RD_REG32(baseAddr + I2C_DATA);

    return rData;
}

void I2CSyscInit(uint32_t baseAddr, uint32_t syscFlag)
{
    HW_WR_REG32(baseAddr + I2C_SYSC, syscFlag);
}

void I2CConfig(uint32_t baseAddr, uint32_t conParams)
{
    HW_WR_REG32(baseAddr + I2C_CON, conParams);
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
            I2CMasterIntDisableEx(object->baseAddr, I2C_INT_ALL);

            /* Disable the I2C Master */
            I2CMasterDisable(object->baseAddr);

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

void I2C_completeCurrTransfer(I2C_Handle handle, int32_t xferStatus)
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
            (void)I2C_primeTransfer(handle, object->headPtr);
          }
    }
}

/*
 *  Hwi interrupt handler to service the I2C peripheral in master mode
 */
void I2C_hwiFxnMaster(I2C_Handle handle)
{
    I2C_Object        *object = NULL;
    int32_t            xferStatus = I2C_STS_SUCCESS;
    uint32_t           isrLoopCount = 0;
    uint32_t           w;
    uint32_t           stat;
    uint32_t           rawStat;
    uint32_t           intErr = 0;
    uint32_t           fatalError = 0;
    uint32_t           xsa;
    uint32_t           regVal;
    uint32_t           loopFlag = TRUE;

    /* Get the pointer to the object */
    object = (I2C_Object*)handle->object;

    /*
     * Ack the stat in one go, but [R/X]RDY should be
     * acked after the data operation is complete.
     */

    /*
     * The Master ISR handling is based on AM5 TRM HS I2C Programming Guide
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
        stat = I2CMasterIntStatus(object->baseAddr);
        if ((0U == stat) || (isrLoopCount > I2C_MAX_CONSECUTIVE_ISRS))
        {
            loopFlag = FALSE;
        }
        else
        {
            isrLoopCount++;

            rawStat = I2CMasterIntRawStatus(object->baseAddr);

            if ((rawStat & I2C_INT_NO_ACK) != 0U)
            {
                intErr   |= I2C_INT_NO_ACK;
                xferStatus = I2C_STS_ERR_NO_ACK;
                I2CMasterIntClearEx(object->baseAddr, I2C_INT_NO_ACK);
                fatalError=1U;
            }

            if ((rawStat & I2C_INT_ARBITRATION_LOST) != 0U)
            {
                intErr   |= I2C_INT_ARBITRATION_LOST;
                xferStatus = I2C_STS_ERR_ARBITRATION_LOST;
                I2CMasterIntClearEx(object->baseAddr,I2C_INT_ARBITRATION_LOST);
                fatalError=1U;
            }

            if ((rawStat & I2C_INT_ACCESS_ERROR) != 0U)
            {
                intErr   |= I2C_INT_ACCESS_ERROR;
                xferStatus = I2C_STS_ERR_ACCESS_ERROR;
                I2CMasterIntClearEx(object->baseAddr,I2C_INT_ACCESS_ERROR);
                fatalError=1U;
            }

            if(fatalError != 0U)
            {
               /* ISsue the stop condition*/
               I2CMasterStop(object->baseAddr);
               I2CMasterIntDisableEx(object->baseAddr, I2C_INT_ALL);
               I2CMasterIntClearEx(object->baseAddr, I2C_INT_ALL);
               I2CFIFOClear(object->baseAddr, I2C_TX_MODE);
               I2CFIFOClear(object->baseAddr, I2C_RX_MODE);
               I2CSetDataCount(object->baseAddr, 0);
               object->intStatusErr |= intErr;
               I2C_completeCurrTransfer(handle,xferStatus);
               break;
            }

            w = stat & ~(I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY);
            I2CMasterIntClearEx(object->baseAddr, w);

            if (((stat & I2C_INT_ADRR_READY_ACESS) == I2C_INT_ADRR_READY_ACESS) ||
                ((stat & I2C_INT_BUS_FREE) == I2C_INT_BUS_FREE))
            {
                w = stat & (I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY);
                I2CMasterIntClearEx(object->baseAddr, w);

                if (((object->writeCountIdx) == 0U) && ((object->readCountIdx) != 0U))
                {
                    /* Start write completed, restart read, set read count */
                    I2CSetDataCount(object->baseAddr, object->readCountIdx);

                    /*
                     * Configure peripheral for I2C Receive mode,
                     * do not send stop when sending restart
                     */
                    if (object->currentTransaction->expandSA == (bool)true)
                    {
                        /* enable the 10-bit address mode */
                        xsa = I2C_CFG_10BIT_SLAVE_ADDR;
                    }
                    else
                    {
                        /* enable the 7-bit address mode */
                        xsa = I2C_CFG_7BIT_SLAVE_ADDR;
                    }
                    regVal = I2C_CFG_MST_RX | xsa;
                    if ((object->i2cParams.bitRate == I2C_1P0MHZ) || (object->i2cParams.bitRate == I2C_3P4MHZ))
                    {
                        regVal |= I2C_CFG_HS_MOD;
                    }
                    I2CMasterControl(object->baseAddr, regVal);

                    /* Enable RX interrupt to handle data received */
                    I2CMasterIntEnableEx(object->baseAddr, I2C_INT_RECV_READY);

                    /* Start I2C peripheral in RX mode */
                    I2CMasterStart(object->baseAddr);
                }
                else
                {
                    if ((rawStat & I2C_INT_BUS_BUSY) != 0U)
                    {
                        I2CMasterStop(object->baseAddr);
                        /* if bus still busy, enable bus free interrupt to wait for bus released */
                        I2CMasterIntEnableEx(object->baseAddr, I2C_INT_BUS_FREE);
                    }
                    else
                    {
                        I2CMasterIntDisableEx(object->baseAddr, I2C_INT_ALL);
                        I2C_completeCurrTransfer(handle,xferStatus);
                    }
                    loopFlag = FALSE;
                }
            }

            if ((loopFlag == TRUE) && (0U == intErr))
            {
                if ((stat & I2C_INT_RECV_READY) != 0U)
                {
                    /* Save the received data */
                    if (object->readBufIdx != NULL)
                    {
                        *(object->readBufIdx) = I2CMasterDataGet(object->baseAddr);
                        object->readBufIdx++;
                        object->readCountIdx--;
                    }

                    I2CMasterIntClearEx(object->baseAddr,
                                        (stat & I2C_INT_RECV_READY));
                }

                if ((stat & I2C_INT_TRANSMIT_READY) != 0U)
                {
                    if (object->writeCountIdx != 0U)
                    {
                        /*
                         * Write data until FIFO is full or all data written.
                         * The math below is: (DCOUNT - TXBUFSTAT) % 64 < TXBUFSIZE.
                         */
                        while (((object->writeCountIdx) != 0U) &&
                            (((I2CDataCountGet(object->baseAddr)
                            - I2CBufferStatus(object->baseAddr,
                            I2C_TX_BUFFER_STATUS)) % 64U) < (((uint32_t)8U) <<
                            I2CBufferStatus(object->baseAddr,I2C_FIFO_DEPTH)))) {

                            /* Write data into transmit FIFO */
                            I2CMasterDataPut(object->baseAddr,
                                             *(object->writeBufIdx));
                            (object->writeBufIdx)++;
                            object->writeCountIdx--;
                        }
                    }
                    else
                    {
                        I2CMasterIntDisableEx(object->baseAddr, I2C_INT_TRANSMIT_READY);
                    }

                    I2CMasterIntClearEx(object->baseAddr,
                                        (stat & I2C_INT_TRANSMIT_READY));
                }
            }

            if ((loopFlag == TRUE) && (0U == intErr))
            {
                if ((stat & I2C_INT_RECV_OVER_RUN) != 0U)
                {
                    intErr |= I2C_INT_RECV_OVER_RUN;
                }
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

/*
 *  Hwi interrupt handler to service the I2C peripheral in slave mode
 */
void I2C_hwiFxnSlave(I2C_Handle handle)
{
    I2C_Object     *object = NULL;
    uint32_t        rawStat;

    /* Get the pointer to the object */
    object = (I2C_Object*)handle->object;

    /*
     *  ISR Handling of different events is as below in same order:
     *  1.  RRDY: Handle receive ready interrupt before AAS or ARDY interrupt,
     *      as If ISR delayed and restart or stop condition received then
     *      the data should be read first.
     *  2.  ARDY: In case of stop condition bus busy will be reset. If it is a
     *      restart condition bus busy will still be set.
     *  3.  AAS: In case of addressed as slave after transfer is initiated, it
     *      is a restart condition. Give the restart callback.
     *  4.  XRDY: Read application tx buffer and send data.
     *
     *  In case of restart hold the bus low till application gives next buffer.
     *  If the Buffers are overflowing or underflowing return error code
     *  accordingly. Application should call the abort transfer explicitly.
     */
    rawStat = I2CSlaveIntRawStatus(object->baseAddr);

    if ((rawStat & I2C_INT_RECV_READY) != 0U)
    {
        /* Read from Rx register only when current transaction is ongoing */
        if (object->readCountIdx != 0U)
        {
            *(object->readBufIdx) = I2CMasterDataGet(object->baseAddr);
            object->readBufIdx++;
            object->readCountIdx--;
            I2CMasterIntClearEx(object->baseAddr, I2C_INT_RECV_READY);
        }
        else
        {
            /* Clear the RX data fifo */
            (void)I2CSlaveDataGet(object->baseAddr);

            /* Clear all interrupts */
            I2CSlaveIntClearEx(object->baseAddr, I2C_INT_ALL);

            /* Disable STOP condition interrupt */
            I2CSlaveIntDisableEx(object->baseAddr, I2C_INT_ALL);

            /* Finish the current transfer */
            I2C_completeCurrTransfer(handle,I2C_STS_ERR);
        }
    }

    if ((rawStat & I2C_INT_ADRR_READY_ACESS) != 0U)
    {
        if ((rawStat & I2C_INT_BUS_BUSY) != 0U)
        {
            /* Clear Interrupt, Callback will be handled in ADDR_SLAVE */
            I2CSlaveIntClearEx(object->baseAddr, I2C_INT_ADRR_READY_ACESS);
        }
        else
        {
            /* This is end of current transfer */
            I2CSlaveIntClearEx(object->baseAddr, I2C_INT_ALL);
            object->state = I2C_IDLE_STATE;

            /* Finish the current transfer */
            I2C_completeCurrTransfer(handle,I2C_STS_SUCCESS);
        }
    }

    if ((rawStat & I2C_INT_ADRR_SLAVE) != 0U)
    {
        if (object->state == I2C_IDLE_STATE)
        {
            /* This is the first transfer initiation from master */
            /* Update the state to transfer started and Clear the Interrupt */
            object->state = I2C_SLAVE_XFER_STATE;
            I2CSlaveIntClearEx(object->baseAddr, I2C_INT_ADRR_SLAVE);
        }
        else if (object->state == I2C_SLAVE_XFER_STATE)
        {
            if ((object->writeCountIdx == 0U) && ((rawStat & I2C_INT_TRANSMIT_UNDER_FLOW) != 0U))
            {
                /*
                 * This is a restart condition, slave write count should be provided by
                 * application in the callback function
                 */

                /* Callback to application to restart read/write */
                object->i2cParams.transferCallbackFxn(handle,
                                                      object->currentTransaction,
                                                      I2C_STS_RESTART);
                object->writeBufIdx = (uint8_t*)(object->currentTransaction->writeBuf);
                object->writeCountIdx = (uint32_t)object->currentTransaction->writeCount;

                object->readBufIdx = (uint8_t*)(object->currentTransaction->readBuf);
                object->readCountIdx = (uint32_t)object->currentTransaction->readCount;
            }
            else
            {
            }

            object->state = I2C_SLAVE_RESTART_STATE;
            I2CSlaveIntClearEx(object->baseAddr, I2C_INT_ADRR_SLAVE);
        }
        else
        {
            /* Control should not come here. Sphurious interrupt clear it. */
            I2CSlaveIntClearEx(object->baseAddr, I2C_INT_ADRR_SLAVE);
        }
    }

    if ((rawStat & I2C_INT_TRANSMIT_READY) != 0U)
    {
        if (object->state == I2C_SLAVE_XFER_STATE)
        {
            if (object->writeCountIdx != 0U)
            {
                I2CMasterDataPut(object->baseAddr, *(object->writeBufIdx));
                object->writeCountIdx--;
                object->writeBufIdx++;
                I2CSlaveIntClearEx(object->baseAddr, I2C_INT_TRANSMIT_READY);
            }
            else
            {
                /* Clear all interrupts */
                I2CSlaveIntClearEx(object->baseAddr, I2C_INT_ALL);

                /* Disable STOP condition interrupt */
                I2CSlaveIntDisableEx(object->baseAddr, I2C_INT_ALL);



                object->readBufIdx = (uint8_t*)(object->currentTransaction->readBuf);
                object->readCountIdx = (uint32_t)object->currentTransaction->readCount;



                /* Post Semaphore to unblock transfer fxn */
                object->i2cParams.transferCallbackFxn(handle,
                                                      object->currentTransaction,
                                                     I2C_STS_ERR);

                /* See if we need to process any other transactions */
                if (object->headPtr == object->tailPtr) {
                    /* No other transactions need to occur */
                    object->currentTransaction = NULL;
                    object->headPtr = NULL;

                }
                else {
                    /* Another transfer needs to take place */
                    object->headPtr = (I2C_Transaction*)(object->headPtr->nextPtr);

                    /* Start new transfer */
                    (void)I2C_primeTransfer(handle, object->headPtr);
                }
            }
        }
        else
        {
            if (object->writeCountIdx != 0U)
            {
                I2CMasterDataPut(object->baseAddr, *(object->writeBufIdx));
                object->writeCountIdx--;
                object->writeBufIdx++;
                I2CSlaveIntClearEx(object->baseAddr, I2C_INT_TRANSMIT_READY);
            }
            else
            {
                /* Clear all interrupts */
                I2CSlaveIntClearEx(object->baseAddr, I2C_INT_ALL);

                /* Disable STOP condition interrupt */
                I2CSlaveIntDisableEx(object->baseAddr, I2C_INT_ALL);

                object->readBufIdx = (uint8_t*)(object->currentTransaction->readBuf);
                object->readCountIdx = (uint32_t)object->currentTransaction->readCount;

                /* Post Semaphore to unblock transfer fxn */
                object->i2cParams.transferCallbackFxn(handle,
                                                      object->currentTransaction,
                                                      I2C_STS_ERR);

                /* See if we need to process any other transactions */
                if (object->headPtr == object->tailPtr) {
                    /* No other transactions need to occur */
                    object->currentTransaction = NULL;
                    object->headPtr = NULL;

                }
                else {
                    /* Another transfer needs to take place */
                    object->headPtr = (I2C_Transaction*)(object->headPtr->nextPtr);

                    /* Start new transfer */
                    (void)I2C_primeTransfer(handle, object->headPtr);
                }
            }
        }
    }

    return;
}

/*
 *  Hwi interrupt handler to service the I2C peripheral
 *
 *  The handler is a generic handler for a I2C object.
 */
void I2C_hwiFxn(void* arg)
{
    I2C_Handle          handle = (I2C_Handle)arg;
    I2C_Object      *object = NULL;

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object */
        object = (I2C_Object *)handle->object;

        if (object->currentTransaction->masterMode)
        {
            I2C_hwiFxnMaster(handle);
        }
        else
        {
            I2C_hwiFxnSlave(handle);
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

        if(status == SystemP_SUCCESS)
        {
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
            I2CMasterDisable(object->baseAddr);

            /* Disable Auto Idle functionality */
            I2CAutoIdleDisable(object->baseAddr);

            /* Extract bit rate from the input parameter */
            switch(object->i2cParams.bitRate)
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
            I2CMasterInitExpClk(object->baseAddr,
                              hwAttrs->funcClk,
                              internalClk,
                              outputClk);

            /* Clear any pending interrupts */
            I2CMasterIntClearEx(object->baseAddr, I2C_INT_ALL);

            /* Mask off all interrupts */
            I2CMasterIntDisableEx(object->baseAddr, I2C_INT_ALL);

            /* Enable the I2C Master for operation */
            I2CMasterEnable(object->baseAddr);

            /* Enable free run mode */
            I2CMasterEnableFreeRun(object->baseAddr);

            /* Return the address of the i2cObjectArray[i] configuration struct */
        }
    }

    SemaphoreP_post(&gI2cDrvObj.lockObj);
    return (handle);
}

bool I2C_checkTimeout(uint32_t *pUsecCnt)
{
    bool timeout = (bool)false;

    *pUsecCnt = *pUsecCnt + 1U;
    if (*pUsecCnt == 1000U)
    {
        *pUsecCnt = 0U;
        timeout = (bool)true;
    }

    return (timeout);
}

int32_t I2C_primeTransfer(I2C_Handle handle,
                                 I2C_Transaction *transaction)
{
    I2C_Object  *object = NULL;
    I2C_HwAttrs const *hwAttrs = NULL;
    int32_t status = I2C_STS_SUCCESS;
    uint32_t errStat=0,fatalError=0;
    uint32_t regVal;
    uint32_t xsa;
    uint32_t timeout = transaction->timeout;
    uint32_t uSecTimeout = 0U;

    /* Get the pointer to the object and hwAttrs */
    object = (I2C_Object*)handle->object;
    hwAttrs = (I2C_HwAttrs const *)handle->hwAttrs;

    /* Store the new internal counters and pointers */
    object->currentTransaction = transaction;

    object->writeBufIdx = (uint8_t*)transaction->writeBuf;
    object->writeCountIdx = (uint32_t)transaction->writeCount;

    object->readBufIdx = (uint8_t*)transaction->readBuf;
    object->readCountIdx = (uint32_t)transaction->readCount;

    object->intStatusErr = 0;

    object->state = I2C_IDLE_STATE;

    /* clear all interrupts */
    I2CMasterIntClearEx(object->baseAddr, I2C_INT_ALL);

    if (transaction->masterMode)
    {
        if (object->currentTransaction->expandSA == (bool)true)
        {
            /* enable the 10-bit address mode */
            xsa = I2C_CFG_10BIT_SLAVE_ADDR;
        }
        else
        {
            /* enable the 7-bit address mode */
            xsa = I2C_CFG_7BIT_SLAVE_ADDR;
        }

        /* In master mode, set the I2C slave address */
        I2CMasterSlaveAddrSet(object->baseAddr,
                              object->currentTransaction->slaveAddress);

        if (TRUE == hwAttrs->enableIntr)
        {
            /* Start transfer in Transmit mode */
            if (object->writeCountIdx != 0U)
            {
                /* Set number of bytes to be transmitting */
                I2CSetDataCount(object->baseAddr, object->writeCountIdx);

                /*
                 * Configure the I2C transfer to be in master transmitter mode
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

                if ((object->i2cParams.bitRate == I2C_1P0MHZ) || (object->i2cParams.bitRate == I2C_3P4MHZ))
                {
                    regVal |= I2C_CFG_HS_MOD;
                }
                I2CMasterControl(object->baseAddr, regVal);

                regVal = I2C_INT_TRANSMIT_READY | I2C_INT_ADRR_READY_ACESS | I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST;
                I2CMasterIntEnableEx(object->baseAddr, regVal);

                /* Start the I2C transfer in master transmit mode */
                I2CMasterStart(object->baseAddr);

            }
            else
            {
                /* Specify the number of bytes to read */
                I2CSetDataCount(object->baseAddr, object->readCountIdx);

                /*
                 * Start the I2C transfer in master receive mode,
                 * and automatically send stop when done
                 */
                regVal = I2C_CFG_MST_RX | I2C_CFG_STOP | xsa;
                if ((object->i2cParams.bitRate == I2C_1P0MHZ) || (object->i2cParams.bitRate == I2C_3P4MHZ))
                {
                    regVal |= I2C_CFG_HS_MOD;
                }
                I2CMasterControl(object->baseAddr, regVal);

                /* Enable RX interrupts */
                I2CMasterIntEnableEx(object->baseAddr,
                                     I2C_INT_RECV_READY | I2C_INT_ADRR_READY_ACESS | I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST);

                /* Send start bit */
                I2CMasterStart(object->baseAddr);

            }
        }
        else  /* POLLING MODE */
        {
            if(object->writeCountIdx != 0U)
            {
                /* set number of bytes to write */
                I2CSetDataCount(object->baseAddr, object->writeCountIdx);

                /* set to master transmitter mode */
                regVal = I2C_CFG_MST_TX | xsa;
                if ((object->i2cParams.bitRate == I2C_1P0MHZ) || (object->i2cParams.bitRate == I2C_3P4MHZ))
                {
                    regVal |= I2C_CFG_HS_MOD;
                }

                /* wait for bus busy */
                while ((I2CMasterBusBusy(object->baseAddr) == 1) && (timeout != 0))
                {
                    I2C_udelay(I2C_DELAY_USEC);
                    if (I2C_checkTimeout(&uSecTimeout))
                    {
                        timeout--;
                    }
                }

                I2CMasterControl(object->baseAddr, regVal);

                /* generate start */
                I2CMasterStart(object->baseAddr);

                while ((object->writeCountIdx != 0U) && (timeout != 0))
                {
                    /* wait for transmit ready or error */
                    while(((I2CMasterIntRawStatusEx(object->baseAddr, I2C_INT_TRANSMIT_READY) == 0U) && \
                           (I2CMasterIntRawStatusEx(object->baseAddr, I2C_INT_ARBITRATION_LOST | \
                                                                       I2C_INT_NO_ACK | \
                                                                       I2C_INT_ACCESS_ERROR | \
                                                                       I2C_INT_STOP_CONDITION ) == 0U)) && \
                          (timeout != 0))
                    {
                        I2C_udelay(I2C_DELAY_USEC);
                        if (I2C_checkTimeout(&uSecTimeout))
                        {
                            timeout--;
                        }
                    }

                    errStat = I2CMasterIntRawStatusEx(object->baseAddr, I2C_INT_ARBITRATION_LOST | \
                                                                         I2C_INT_NO_ACK | \
                                                                         I2C_INT_ACCESS_ERROR);

                    /* if we get an error, do a stop and return failure */
                    if (errStat != 0U)
                    /* if we get an error, do a stop and return failure */
                    {
                       fatalError = 1U;
                       break;
                    }
                    /* write byte and increase data pointer to next byte */
                    I2CMasterDataPut(object->baseAddr, *(object->writeBufIdx));
                    (object->writeBufIdx)++;

                    /* clear transmit ready interrupt */
                    I2CMasterIntClearEx(object->baseAddr, I2C_INT_TRANSMIT_READY);

                    /* update number of bytes written */
                    object->writeCountIdx--;
                }

                if ((fatalError == 0U) && (timeout != 0U))
                {
                    /* wait for register access ready */
                    timeout = I2C_waitForPin(handle,
                                                I2C_INT_ADRR_READY_ACESS,
                                                &timeout);
                }

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
                else if (timeout == 0U)
                {
                    status = I2C_STS_ERR_TIMEOUT;
                }
                else
                {
                    status = I2C_STS_SUCCESS;
                }

                if (object->readCountIdx == 0U)
                {
                    /* generate stop when there is no read following by write */
                    I2CMasterStop(object->baseAddr);

                    if ((fatalError == 0U) && (timeout != 0U))
                    {
                        /* wait for stop to happen */
                        timeout = I2C_waitForPin(handle,
                                                    I2C_INT_STOP_CONDITION,
                                                    &timeout);


                        /* wait for register access ready */
                        timeout = I2C_waitForPin(handle,
                                                    I2C_INT_ADRR_READY_ACESS,
                                                    &timeout);

                        if (timeout == 0U)
                        {
                            status = I2C_STS_ERR_TIMEOUT;
                        }
                    }
                }
            }

            if ((object->readCountIdx != 0U) && (status == I2C_STS_SUCCESS))
            {
                /* clear all interrupts */
                I2CMasterIntClearEx(object->baseAddr, I2C_INT_ALL);

                /* set number of bytes to read */
                I2CSetDataCount(object->baseAddr, object->readCountIdx);

                /* set to master receiver mode */
                regVal = I2C_CFG_MST_RX | xsa;
                if ((object->i2cParams.bitRate == I2C_1P0MHZ) || (object->i2cParams.bitRate == I2C_3P4MHZ))
                {
                    regVal |= I2C_CFG_HS_MOD;
                }

                /* wait for bus not busy.
                 * Check bus busy for read-only transfers to support
                 * repeat start condition during write address and read data.
                 */
                if((object->writeBufIdx) == NULL)
                {
                    while ((I2CMasterBusBusy(object->baseAddr) == 1) && (timeout != 0U))
                    {
                        I2C_udelay(I2C_DELAY_USEC);
                        if (I2C_checkTimeout(&uSecTimeout))
                        {
                            timeout--;
                        }
                    }
                }

                I2CMasterControl(object->baseAddr, regVal);

                /* generate start */
                I2CMasterStart(object->baseAddr);

                while ((object->readCountIdx != 0U) && (timeout != 0U))
                {
                    /* wait for receive ready or error */
                    while(((I2CMasterIntRawStatusEx(object->baseAddr, I2C_INT_RECV_READY) == 0U) && \
                           (I2CMasterIntRawStatusEx(object->baseAddr, I2C_INT_ARBITRATION_LOST | \
                                                                       I2C_INT_NO_ACK | \
                                                                       I2C_INT_ACCESS_ERROR ) == 0U)) && \
                          (timeout != 0U))
                    {
                        I2C_udelay(I2C_DELAY_USEC);
                        if (I2C_checkTimeout(&uSecTimeout))
                        {
                            timeout--;
                        }
                    }

                    errStat = I2CMasterIntRawStatusEx(object->baseAddr, I2C_INT_ARBITRATION_LOST | \
                                                                         I2C_INT_NO_ACK | \
                                                                         I2C_INT_ACCESS_ERROR);

                    /* if we get an error, do a stop and return failure */
                    if (errStat != 0U)
                    {
                       fatalError = 1U;
                       break;
                    }

                    /* read byte and increase data pointer to next byte */
                    *(object->readBufIdx) =
                        (uint8_t)I2CMasterDataGet(object->baseAddr);

                    /* clear receive ready interrupt */
                    I2CMasterIntClearEx(object->baseAddr, I2C_INT_RECV_READY);

                    object->readBufIdx++;
                    object->readCountIdx--;   /* update number of bytes read */
                }

                if ((fatalError == 0U) && (timeout != 0U))
                {
                    /* wait for register access ready */
                    timeout = I2C_waitForPin(handle,
                                                I2C_INT_ADRR_READY_ACESS,
                                                &timeout);
                }

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
                else if (timeout == 0U)
                {
                    status = I2C_STS_ERR_TIMEOUT;
                }
                else
                {
                    status = I2C_STS_SUCCESS;
                }

                /* generate stop when requested */
                I2CMasterStop(object->baseAddr);

                if ((fatalError == 0U) && (timeout != 0U))
                {
                    /* wait for stop to happen */
                    timeout = I2C_waitForPin(handle,
                                                I2C_INT_STOP_CONDITION,
                                                &timeout);

                    /* wait for register access ready */
                    timeout = I2C_waitForPin(handle,
                                                I2C_INT_ADRR_READY_ACESS,
                                                &timeout);

                    if (timeout == 0U)
                    {
                        status = I2C_STS_ERR_TIMEOUT;
                    }
                }
            }
        }
    }
    /* In slave mode */
    else
    {
        if (object->currentTransaction->expandSA == (bool)true)
        {
            /* enable the 10-bit address mode */
            xsa = I2C_CFG_10BIT_OWN_ADDR_0;
        }
        else
        {
            /* enable the 7-bit address mode */
            xsa = I2C_CFG_7BIT_OWN_ADDR_0;
        }

        /* Currently slave mode is supported only when interrupt is enabled */
        if (TRUE == hwAttrs->enableIntr)
        {
            /* In slave mode, set the I2C own address */
            I2COwnAddressSet(object->baseAddr,
                             hwAttrs->ownSlaveAddr[0],
                             I2C_OWN_ADDR_0);

            /* Configure data buffer length to 0 as the actual number of bytes to
               transmit/receive is dependant on external master. */
            I2CSetDataCount(object->baseAddr, 0U);

            /* Enable interrupts in slave mode */
            I2CSlaveIntEnableEx(object->baseAddr,
                                I2C_INT_TRANSMIT_READY | I2C_INT_RECV_READY |
                                I2C_INT_ADRR_READY_ACESS | I2C_INT_ADRR_SLAVE);

            /* Start the I2C transfer in slave mode */
            regVal = I2C_CFG_MST_ENABLE | xsa;
            if ((object->i2cParams.bitRate == I2C_1P0MHZ) || (object->i2cParams.bitRate == I2C_3P4MHZ))
            {
                regVal |= I2C_CFG_HS_MOD;
            }
            I2CMasterControl(object->baseAddr, regVal);
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
             * Clear the RX + TX FIFOs. If the previous transfer failed due to an error,
             * there's a possibility data could still be in the FIFO.
             */
            I2CFIFOClear(object->baseAddr, I2C_TX_MODE);
            I2CFIFOClear(object->baseAddr, I2C_RX_MODE);
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
                    (void)I2C_recoverBus(handle, I2C_DELAY_SMALL);
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
                    else if ((object->intStatusErr & I2C_INT_ACCESS_ERROR) != 0U)
                    {
                        retVal = I2C_STS_ERR_ACCESS_ERROR;
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

void I2C_transferCallback(I2C_Handle handle,
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

int32_t I2C_probe(I2C_Handle handle, uint32_t slaveAddr)
{
    int32_t             retVal  = I2C_STS_ERR;
    uint32_t            regVal;
    I2C_Object         *object  = NULL;

    /* Input parameter validation */
    if (handle != NULL)
    {
        object  = (I2C_Object*)handle->object;

        /* Disable interrupts first */
        regVal = I2CGetEnabledIntStatus(object->baseAddr, I2C_INT_ALL);

        I2CMasterIntDisableEx(object->baseAddr, I2C_INT_ALL);

        /* wait until bus not busy */
        if (I2C_waitForBb(object->baseAddr, I2C_DELAY_MED) != I2C_STS_SUCCESS)
        {
            retVal = I2C_STS_ERR;
        }
        else
        {
            /* set slave address */
            I2CMasterSlaveAddrSet(object->baseAddr, (uint32_t) slaveAddr);

            /* try to write one byte */
            I2CMasterDataPut(object->baseAddr, (uint8_t) 0U);
            I2CSetDataCount(object->baseAddr, (uint32_t) 1U);

            /* stop bit needed here */
            I2CConfig(object->baseAddr,
                        (I2C_CFG_MST_ENABLE | I2C_CFG_MST_TX | I2C_CFG_START |
                        I2C_CFG_STOP));

            /* enough delay for the NACK bit set */
            I2C_udelay(I2C_DELAY_BIG);

            if (0U == I2CMasterIntRawStatusEx(object->baseAddr, I2C_INT_NO_ACK))
            {
                retVal = I2C_STS_SUCCESS;        /* success case */
            }
            else
            {
                /* Clear sources*/
                I2CMasterIntClearEx(object->baseAddr, I2C_INT_ALL);

                /* finish up xfer */
                I2CMasterStop(object->baseAddr);
                (void)I2C_waitForBb(object->baseAddr, I2C_DELAY_MED);

                retVal = I2C_STS_ERR;         /* Error case */
            }

            I2CFIFOClear(object->baseAddr, I2C_TX_MODE);
            I2CFIFOClear(object->baseAddr, I2C_RX_MODE);
            I2CSetDataCount(object->baseAddr, 0);
            I2CMasterIntClearEx(object->baseAddr, I2C_INT_ALL);
        }

        /* wait for bus free */
        if (I2C_waitForBb(object->baseAddr, I2C_DELAY_MED) != I2C_STS_SUCCESS)
        {
            retVal = I2C_STS_ERR;
        }

        /* Enable interrupts now */
        I2CMasterIntEnableEx(object->baseAddr, regVal);
    }
    return (retVal);
}

int32_t I2C_waitForBb(uint32_t baseAddr, uint32_t timeout)
{
    uint32_t            stat;
    int32_t             retVal = I2C_STS_SUCCESS;
    volatile uint32_t   bbtimeout = timeout;

    if(bbtimeout > 0U)
    {
        /* Clear current interrupts...*/
        I2CMasterIntClearEx(baseAddr, I2C_INT_ALL);

    while (bbtimeout > 0U)
    {
        stat = I2CMasterIntRawStatusEx(baseAddr, I2C_INT_BUS_BUSY);
        if (stat == 0U)
        {
            break;
        }
        bbtimeout = bbtimeout - 1U;
        I2CMasterIntClearEx(baseAddr, stat);
    }

    if (timeout > 0U)
    {
        if (bbtimeout == 0U)
        {
            retVal = I2C_STS_ERR;
        }
    }

        /* clear delayed stuff*/
        I2CMasterIntClearEx(baseAddr, I2C_INT_ALL);
    }
    else
    {
        while(I2CMasterBusBusy(baseAddr) == 1)
        {
        }
    }

    return retVal;
}

void I2C_udelay(uint32_t delay)
{
    volatile uint32_t del = delay;

    while (del != 0U)
    {
        del = del - 1U;
    }
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
        I2CMasterDisable(object->baseAddr);

        /* Extract bit rate from the input parameter */
        switch(busFrequency)
        {
           case (uint32_t)I2C_100KHZ:
           {
               outputClk = 100000U;
               internalClk = I2C_MODULE_INTERNAL_CLK_4MHZ;
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
               internalClk = I2C_MODULE_INTERNAL_CLK_4MHZ;
               break;
           }
        }

        /* Set the I2C configuration */
        I2CMasterInitExpClk(object->baseAddr, hwAttrs->funcClk, internalClk,
            outputClk);

        /* Clear any pending interrupts */
        I2CMasterIntClearEx(object->baseAddr, I2C_INT_ALL);

        /* Mask off all interrupts */
        I2CMasterIntDisableEx(object->baseAddr, I2C_INT_ALL);

        /* Enable the I2C Master for operation */
        I2CMasterEnable(object->baseAddr);

        /* Enable free run mode */
        I2CMasterEnableFreeRun(object->baseAddr);

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

int32_t I2C_recoverBus(I2C_Handle handle, uint32_t i2cDelay)
{
    I2C_Object   *object = NULL;
    int32_t status = I2C_STS_SUCCESS;
    uint32_t sysTest, i;

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

        /* Check if SDA or SCL is stuck low based on the SYSTEST.
         * If SCL is stuck low we reset the IP.
         * If SDA is stuck low drive 9 clock pulses on SCL and check if the
         * slave has released the SDA. If not we reset the I2C controller.
         */

        sysTest = I2CMasterGetSysTest(object->baseAddr);
        if ((sysTest & I2C_SYSTEST_SCL_I_FUNC_MASK) == 0U)
        {
            /* SCL is stuck low reset the I2C IP */
            status = I2C_resetCtrl(handle);
        }
        else if ((sysTest & I2C_SYSTEST_SDA_I_FUNC_MASK) == 0U)
        {
            /* SDA is stuck low generate 9 clk pulses on SCL */
            /* switch to system test mode */
            HW_SET_FIELD32(sysTest, I2C_SYSTEST_ST_EN, I2C_SYSTEST_ST_EN_ENABLE);
            HW_SET_FIELD32(sysTest, I2C_SYSTEST_TMODE,
                           I2C_SYSTEST_TMODE_LOOPBACK);
            I2CMasterSetSysTest(object->baseAddr, sysTest);
            for (i = 0; i < 9U; i++)
            {
                HW_SET_FIELD32(sysTest, I2C_SYSTEST_SCL_O,
                               I2C_SYSTEST_SCL_O_SCLOH);
                I2CMasterSetSysTest(object->baseAddr, sysTest);
                I2C_udelay(i2cDelay);
                HW_SET_FIELD32(sysTest, I2C_SYSTEST_SCL_O,
                               I2C_SYSTEST_SCL_O_SCLOL);
                I2CMasterSetSysTest(object->baseAddr, sysTest);
                I2C_udelay(i2cDelay);
            }
            /* Switch back to functional mode */
            HW_SET_FIELD32(sysTest, I2C_SYSTEST_ST_EN,
                           I2C_SYSTEST_ST_EN_DISABLE);
            HW_SET_FIELD32(sysTest, I2C_SYSTEST_TMODE,
                           I2C_SYSTEST_TMODE_FUNCTIONAL);
            I2CMasterSetSysTest(object->baseAddr, sysTest);
            /* Now check if the SDA is releases. If its still stuck low,
             * There is nothing that can be done. We still try to reset our IP.
             */
            sysTest = I2CMasterGetSysTest(object->baseAddr);
            if ((sysTest & I2C_SYSTEST_SDA_I_FUNC_MASK) == 0U)
            {
                status = I2C_resetCtrl(handle);
            }
        }
        else
        {
            /* Nothing to be done. SCA and SDA both are not stuck to low */
        }

        /* Release the lock for this particular I2C handle */
        (void)SemaphoreP_post(&object->mutex);
    }
    return status;
}

int32_t I2C_resetCtrl(I2C_Handle handle)
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
        status = I2C_ctrlInit(handle);
    }

    return status;
}

int32_t I2C_ctrlInit(I2C_Handle handle)
{
    I2C_HwAttrs const  *hwAttrs = NULL;
    I2C_Object      *object = NULL;
    uint32_t             delay = 50U;
    uint32_t             outputClk;
    uint32_t             internalClk;
    uint32_t             regVal;
    int32_t retVal = I2C_STS_SUCCESS;

    /* Get the pointer to hwAttrs */
    object = (I2C_Object*)handle->object;
    hwAttrs = (I2C_HwAttrs const *)handle->hwAttrs;

    /* Put i2c in reset/disabled state */
    I2CMasterDisable(object->baseAddr);

    /* Do a software reset */
    I2CSoftReset(object->baseAddr);

    /* Enable i2c module */
    I2CMasterEnable(object->baseAddr);

    /* Wait for the reset to get complete  -- constant delay - 50ms */
    while ((I2CSystemStatusGet(object->baseAddr) == 0U) && (delay != 0U))
    {
        delay--;
        I2C_udelay(I2C_DELAY_SMALL);
    }

    if (delay == 0U)
    {
        /* reset has failed, return!!! */
        retVal = I2C_STS_ERR;
    }
    else
    {
        /* Put i2c in reset/disabled state */
        I2CMasterDisable(object->baseAddr);

        /* Configure i2c bus speed*/
        switch(object->i2cParams.bitRate)
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
               outputClk = 3400000U;
               internalClk = I2C_MODULE_INTERNAL_CLK_12MHZ;
               break;
           }

           default:
           {
               /* Default case force it to 100 KHZ bit rate */
               outputClk = 100000U;
               internalClk = I2C_MODULE_INTERNAL_CLK_4MHZ;
           }
           break;
        }

        /* Set the I2C configuration */
        I2CMasterInitExpClk(object->baseAddr,
                  hwAttrs->funcClk,
                  internalClk,
                  outputClk);


        /**
         * Configure I2C_SYSC params
         * Disable auto idle mode
         * Both OCP and systen clock cut off
         * Wake up mechanism disabled
         * No idle mode selected
         */
        regVal = I2C_AUTOIDLE_DISABLE | I2C_CUT_OFF_BOTH_CLK |
                 I2C_ENAWAKEUP_DISABLE | I2C_NO_IDLE_MODE;
        I2CSyscInit(object->baseAddr, regVal);

        /* Configure I2C_CON params */
        regVal = I2C_OPMODE_FAST_STAND_MODE | I2C_NORMAL_MODE;
        I2CConfig(object->baseAddr, regVal);

        /* Take the I2C module out of reset: */
        I2CMasterEnable(object->baseAddr);

        /* Enable free run mode */
        I2CMasterEnableFreeRun(object->baseAddr);
    }

    /*Clear status register */
    I2CMasterIntClearEx(object->baseAddr, I2C_INT_ALL);

    return retVal;
}

uint32_t I2C_waitForPin(I2C_Handle  handle,
                        uint32_t    flag,
                        uint32_t   *pTimeout)
{
    uint32_t           status;
    I2C_Object        *object = NULL;
    uint32_t           timeout = *pTimeout;
    uint32_t           uSecTimeout = 0U;

    object = (I2C_Object*)handle->object;
    if(timeout > 0U)
    {
        status = I2CMasterIntRawStatus(object->baseAddr);
        while ((uint32_t) 0U == (status & flag))
        {
            if ((uint32_t) 0U != timeout)
            {
                I2C_udelay(I2C_DELAY_USEC);
                if (I2C_checkTimeout(&uSecTimeout))
                {
                    timeout--;
                }
                status = I2CMasterIntRawStatus(object->baseAddr);
            }
            else
            {
                break;
            }
        }

        if (timeout == 0U)
        {
            I2CMasterIntClearEx(object->baseAddr, I2C_INT_ALL);
        }
    }

    return (timeout);
}

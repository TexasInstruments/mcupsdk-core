/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/soc_config.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

/* Temperature result registers */
#define TMP10X_RESULT_REG               (0x0000U)

/* I2C Interrupt Priority */
#define I2C_INTERRUPT_PRIORITY          (4U)

/*
 * I2C
 */

I2CLLD_Handle gI2cLldHandle0;
uint32_t gI2cVimStsAddr, intrNum, gI2cVimStsClrMask, intcBaseAddr;
uint32_t gI2CTransferMutex = MUTEX_ARM_UNLOCKED;

extern HwiP_Config gHwiConfig;
extern uint8_t Board_getSocTemperatureSensorAddr(void);

/* Transfer Complete Callback Function Declaration */
void I2C_lld_transferCompleteCallback_implementation (void * args, const I2CLLD_Message * msg, int32_t transferStatus);

static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_I2C_ISR(void);

void i2c_temperature_main(void *arg)
{
    uint16_t sample;
    int16_t temperature;
    uint8_t txBuffer[1];
    uint8_t rxBuffer[2];
    int32_t status;
    uint8_t deviceAddress;
    I2CLLD_Transaction i2cTransaction;
    I2CLLD_Message i2cMsg;

    Drivers_open();
    Board_driversOpen();

    gI2cLldHandle0 = (I2CLLD_Handle)(gI2cLldHandle[0]);
    intrNum = gI2cLldHandle0->intrNum;
    intcBaseAddr = gHwiConfig.intcBaseAddr;
    gI2cVimStsAddr = intcBaseAddr + (0x404u + (((intrNum)>> 5) & 0xFu) * 0x20u);
    gI2cVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);

    /* Assign Transfer Complete Callback Function */
    gI2cLldHandle0->transferCompleteCallback = I2C_lld_transferCompleteCallback_implementation;

    /* Register Interrupt */
    HwiP_setVecAddr(intrNum, (uintptr_t)&App_I2C_ISR);
    HwiP_setPri(intrNum, I2C_INTERRUPT_PRIORITY);
    HwiP_enableInt(intrNum);

    /* Determine if I2C sensor is present */
    deviceAddress = Board_getSocTemperatureSensorAddr();
    DebugP_log("[I2C] I2C Temperature Read test Started in interrupt mode ... !!!\r\n");

    status = I2C_lld_probe(gI2cLldHandle0, deviceAddress);

    if(status == I2C_STS_SUCCESS)
    {
        /* found temperature sensor */
        DebugP_log("[I2C] Temperature sensor found at device address 0x%02x \r\n", deviceAddress);
    }
    else
    {
        DebugP_logError("[I2C] Temperature sensor not found at device address 0x%02x \r\n", deviceAddress);
    }

    if(status == I2C_STS_SUCCESS)
    {
        /* Initialize transaction object */
        (void)I2C_lld_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf = txBuffer;
        i2cTransaction.writeCount = 1U;
        /* Initialize message object */
        (void)I2C_lld_Message_init(&i2cMsg);
        i2cMsg.txn = &i2cTransaction;
        i2cMsg.txnCount = 1U;
        i2cMsg.targetAddress = deviceAddress;
        txBuffer[0] = TMP10X_RESULT_REG;

        /* Lock Mutex */
        gI2CTransferMutex = MUTEX_ARM_LOCKED;
        /* Start Transfer in interrupt Mode */
        status = I2C_lld_transferIntr(gI2cLldHandle0, &i2cMsg);
        /* Wait for the Mutex to Unlock */
        while(try_lock_mutex(&gI2CTransferMutex) == MUTEX_ARM_LOCKED);

        /* Sleep for 10us */
        gI2cLldHandle[0]->Clock_uSleep(10U);

        if(status == I2C_STS_SUCCESS)
        {
            /* read the results */
            /* Initialize transaction object */
            (void)I2C_lld_Transaction_init(&i2cTransaction);
            i2cTransaction.readBuf = rxBuffer;
            i2cTransaction.readCount = 2U;
            /* Initialize message object */
            (void)I2C_lld_Message_init(&i2cMsg);
            i2cMsg.txn = &i2cTransaction;
            i2cMsg.txnCount = 1U;
            i2cMsg.targetAddress = deviceAddress;

            /* Take 20 samples and print them out onto the console */
            for(sample = 0; sample < 20; sample++)
            {
                gI2CTransferMutex = MUTEX_ARM_LOCKED;
                /* clear RX buffer every time we read, to make sure it does not have stale data */
                rxBuffer[0] = rxBuffer[1] = 0U;
                status = I2C_lld_transferIntr(gI2cLldHandle0, &i2cMsg);
                while(try_lock_mutex(&gI2CTransferMutex) == MUTEX_ARM_LOCKED);
                if(status == I2C_STS_SUCCESS)
                {
                    /* Create 16 bit temperature */
                    temperature = ((uint16_t)rxBuffer[0] << 8) | (rxBuffer[1]);
                    /*
                     * 4 LSBs of temperature are 0 according to datasheet
                     * since temperature is stored in 12 bits. Therefore,
                     * right shift by 4 places
                     */
                    temperature = temperature >> 4;
                    /*
                     * If the 12th bit of temperature is set '1' (equivalent to 8th bit of the first byte read),
                     * then we have a 2's complement negative value which needs to be sign extended
                     */
                    if(rxBuffer[0] & 0x80)
                    {
                        temperature |= 0xF000;
                    }
                    /* Of the 12 bits of temperature, 4 LSBs are for decimal point according to datasheet so divide by 16 */
                    DebugP_log("[I2C] Sample %u: %f (celcius)\r\n", sample, temperature/16.0);
                }
                else
                {
                    DebugP_logError("[I2C] Sample %u read failed\r\n", sample);
                    break;
                }
            }
        }
    }

    /* De-Register Interrupt */
    HwiP_setVecAddr(intrNum, 0U);
    HwiP_disableInt(intrNum);

    if(status == I2C_STS_SUCCESS)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}

static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_I2C_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(I2C_lld_controllerIsr, \
                                    gI2cLldHandle0, \
                                    intrNum, \
                                    gI2cVimStsAddr, \
                                    gI2cVimStsClrMask,
                                    intcBaseAddr);
}

void I2C_lld_transferCompleteCallback_implementation (void * args, const I2CLLD_Message * msg, int32_t transferStatus)
{
    /* Unlock Mutex */
    unlock_mutex(&gI2CTransferMutex);
}

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

/* Sub address command as per TPIC2810 datasheet */
#define TPIC2810_CMD_RD                     (0x11U)
#define TPIC2810_CMD_WR_SHIFT_REG           (0x11U)
#define TPIC2810_CMD_LOAD_OUTPUT            (0x22U)
#define TPIC2810_CMD_WR_IMMEDIATE           (0x44U)

/* I2C Interrupt Priority */
#define I2C_INTERRUPT_PRIORITY              (4U)

/* LED States */
#define LED_SET_ALL                         (0xFFU)
#define LED_RESET_ALL                       (0x00U)

I2CLLD_Handle gI2cLldHandle0;
uint32_t gI2cVimStsAddr, intrNum, gI2cVimStsClrMask, intcBaseAddr;
uint32_t gI2CTransferMutex = MUTEX_ARM_UNLOCKED;
I2C_ExtendedParams extendedParamsTx, extendedParamsRx;

extern uint8_t Board_getSocLedDeviceAddr(void);

/* Transfer Complete Callback Function Declaration */
void I2C_lld_transferCompleteCallback_implementation (void * args,
                                            const I2CLLD_Message * msg,
                                            int32_t transferStatus);

static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_I2C_ISR(void);

void i2c_led_read_write_main(void *arg)
{
    int32_t status = 0U;
    uint8_t deviceAddress;
    uint8_t txBuf[2U];
    uint8_t rxBuf[1U];
    uint32_t loopCount = 3U, delayMsec = 100U;
    uint8_t ledMask = 0U;
    uint8_t ledCount = 8U;

    Drivers_open();
    Board_driversOpen();

    gI2cLldHandle0 = (I2CLLD_Handle)(gI2cLldHandle[0]);
    intrNum = gI2cLldHandle0->intrNum;
    intcBaseAddr = gHwiConfig.intcBaseAddr;
    gI2cVimStsAddr = intcBaseAddr + (0x404u + (((intrNum)>> 5) & 0xFu) * 0x20u);
    gI2cVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);

    DebugP_log("[I2C] LED Read Write Test Started ...\r\n");
    DebugP_log("LED will Blink for %d loop in polling Mode ...\r\n", loopCount);

    /* Get the Device Address */
    deviceAddress = Board_getSocLedDeviceAddr();

    /* Initialize I2C_ExtendedParams objects*/
    extendedParamsTx.deviceAddress = deviceAddress;
    extendedParamsTx.buffer = txBuf;
    extendedParamsTx.size = 0U;
    extendedParamsTx.expandSA = false;

    extendedParamsRx.deviceAddress = deviceAddress;
    extendedParamsRx.buffer = rxBuf;
    extendedParamsRx.size = 0U;
    extendedParamsRx.expandSA = false;

    while(loopCount-- != 0){

        ledMask = (uint8_t)(LED_RESET_ALL);

        for(uint8_t i = 0; i < ledCount; i++){

            ledMask |= (uint8_t)(1 << i);
            txBuf[0U] = (uint8_t)TPIC2810_CMD_WR_SHIFT_REG;
            txBuf[1U] = ledMask;
            rxBuf[0U] = (uint8_t)(0U);

            extendedParamsTx.size = 2U;
            status = I2C_lld_write(gI2cLldHandle0, &extendedParamsTx, I2C_WAIT_FOREVER);
            if (status != I2C_STS_SUCCESS) {
                break;
            }

            extendedParamsRx.size = 1U;
            status = I2C_lld_read(gI2cLldHandle0, &extendedParamsRx, I2C_WAIT_FOREVER);
            if (status != I2C_STS_SUCCESS) {
                break;
            }

            txBuf[0U] = (uint8_t)TPIC2810_CMD_LOAD_OUTPUT;
            extendedParamsTx.size = 1U;
            status = I2C_lld_write(gI2cLldHandle0, &extendedParamsTx, I2C_WAIT_FOREVER);
            if (status != I2C_STS_SUCCESS) {
                break;
            }

            if(txBuf[1U] != rxBuf[0]){
                break;
            }

            /* Wait 100ms */
            gI2cLldHandle[0]->Clock_uSleep(delayMsec * 1000U);
        }

        if (status != I2C_STS_SUCCESS) {
            break;
        }
    }

    /* Assign Transfer Complete Callback Function */
    gI2cLldHandle0->transferCompleteCallback = I2C_lld_transferCompleteCallback_implementation;

    /* Register Interrupt */
    HwiP_setVecAddr(intrNum, (uintptr_t)&App_I2C_ISR);
    HwiP_setPri(intrNum, I2C_INTERRUPT_PRIORITY);
    HwiP_enableInt(intrNum);

    loopCount = 3U;

    DebugP_log("LED will Blink for %d loop in interrupt Mode ...\r\n", loopCount);

    while(loopCount-- != 0){

        ledMask = (uint8_t)(LED_RESET_ALL);

        for(uint8_t i = 0; i < ledCount; i++)
        {
            ledMask |= (uint8_t)(1 << i);
            txBuf[0U] = (uint8_t)TPIC2810_CMD_WR_SHIFT_REG;
            txBuf[1U] = ledMask;
            rxBuf[0U] = (uint8_t)(0U);

            extendedParamsTx.size = 2U;
            /* Lock Mutex */
            gI2CTransferMutex = MUTEX_ARM_LOCKED;
            /* Start Transfer in interrupt Mode */
            status = I2C_lld_writeIntr(gI2cLldHandle[0], &extendedParamsTx);
            /* Wait for the Mutex to Unlock */
            while(try_lock_mutex(&gI2CTransferMutex) == MUTEX_ARM_LOCKED);

            if (status != I2C_STS_SUCCESS)
            {
                break;
            }

            extendedParamsRx.size = 1U;
            /* Lock Mutex */
            gI2CTransferMutex = MUTEX_ARM_LOCKED;
            /* Start Transfer in interrupt Mode */
            status = I2C_lld_readIntr(gI2cLldHandle0, &extendedParamsRx);
            /* Wait for the Mutex to Unlock */
            while(try_lock_mutex(&gI2CTransferMutex) == MUTEX_ARM_LOCKED);

            if (status != I2C_STS_SUCCESS)
            {
                break;
            }

            txBuf[0U] = (uint8_t)TPIC2810_CMD_LOAD_OUTPUT;
            extendedParamsTx.size = 1U;
            /* Lock Mutex */
            gI2CTransferMutex = MUTEX_ARM_LOCKED;
            /* Start Transfer in interrupt Mode */
            status = I2C_lld_writeIntr(gI2cLldHandle0, &extendedParamsTx);
            /* Wait for the Mutex to Unlock */
            while(try_lock_mutex(&gI2CTransferMutex) == MUTEX_ARM_LOCKED);

            if (status != I2C_STS_SUCCESS)
            {
                break;
            }

            if(txBuf[1U] != rxBuf[0])
            {
                break;
            }

            /* Wait 100ms */
            gI2cLldHandle[0]->Clock_uSleep(delayMsec * 1000U);

        }

        if (status != I2C_STS_SUCCESS) {
            break;
        }
    }

    /* De-Register Interrupt */
    HwiP_setVecAddr(intrNum, 0);
    HwiP_disableInt(intrNum);

    /* Turn off all LEDs */
    if (status == I2C_STS_SUCCESS) {

        ledMask = (uint8_t)(LED_RESET_ALL);
        txBuf[0U] = TPIC2810_CMD_WR_IMMEDIATE;
        txBuf[1U] = ledMask;
        extendedParamsTx.size = 2U;
        status = I2C_lld_write(gI2cLldHandle0, &extendedParamsTx, I2C_WAIT_FOREVER);
    }


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
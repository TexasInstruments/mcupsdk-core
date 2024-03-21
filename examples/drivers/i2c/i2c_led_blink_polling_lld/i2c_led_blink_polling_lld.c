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

/* Sub address command as per TPIC2810 datasheet */
#define TPIC2810_CMD_RD                 (0x11U)
#define TPIC2810_CMD_WR_SHIFT_REG       (0x11U)
#define TPIC2810_CMD_LOAD_OUTPUT        (0x22U)
#define TPIC2810_CMD_WR_IMMEDIATE       (0x44U)

/* LED States */
#define LED_SET_ALL                     (0xFFU)
#define LED_RESET_ALL                   (0x00U)

extern uint8_t Board_getSocLedDeviceAddr(void);

void i2c_led_blink_main(void *arg)
{
    int32_t status;
    uint8_t deviceAddress;
    I2CLLD_Transaction i2cTransaction;
    I2CLLD_Message i2cMsg;
    uint32_t loopcnt = 10U, delayMsec = 100U;
    uint8_t wrData[2U];

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[I2C] LLD LED Blink Test Started in Polling Mode ...\r\n");
    DebugP_log("LED will Blink for %d loop ...\r\n", loopcnt);

    /* Get the Device Address */
    deviceAddress = Board_getSocLedDeviceAddr();

    while (loopcnt-- != 0)
    {
        /* Set mask */
        wrData[0U] = TPIC2810_CMD_WR_IMMEDIATE;
        wrData[1U] = (uint8_t) (LED_SET_ALL);
        /* Initialize transaction object */
        (void)I2C_lld_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf = &wrData[0U];
        i2cTransaction.writeCount = 2U;
        /* Initialize message object */
        (void)I2C_lld_Message_init(&i2cMsg);
        i2cMsg.txn = &i2cTransaction;
        i2cMsg.txnCount = 1U;
        i2cMsg.targetAddress = deviceAddress;
        i2cMsg.timeout = I2C_WAIT_FOREVER;

        /* Start Transfer in polling Mode */
        status = I2C_lld_transferPoll(gI2cLldHandle[0], &i2cMsg);
        /* Sleep for 100Ms */
        gI2cLldHandle[0]->Clock_uSleep(delayMsec * 1000U);

        if (status != I2C_STS_SUCCESS)
        {
            break;
        }

        /* Set mask */
        wrData[0U] = TPIC2810_CMD_WR_IMMEDIATE;
        wrData[1U] = (uint8_t) (LED_RESET_ALL);
        /* Initialize transaction object */
        (void)I2C_lld_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf = &wrData[0U];
        i2cTransaction.writeCount = 2U;
        /* Initialize message object */
        (void)I2C_lld_Message_init(&i2cMsg);
        i2cMsg.txn = &i2cTransaction;
        i2cMsg.txnCount = 1U;
        i2cMsg.targetAddress = deviceAddress;
        i2cMsg.timeout = I2C_WAIT_FOREVER;

        /* Start Transfer in polling Mode */
        status = I2C_lld_transferPoll(gI2cLldHandle[0], &i2cMsg);
        /* Sleep for 100Ms */
        gI2cLldHandle[0]->Clock_uSleep(delayMsec * 1000U);

        if (status != I2C_STS_SUCCESS)
        {
            break;
        }
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

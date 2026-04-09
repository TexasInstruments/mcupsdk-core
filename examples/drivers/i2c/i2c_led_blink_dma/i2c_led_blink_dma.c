/*
 *  Copyright (C) 2018-2025 Texas Instruments Incorporated
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
 *
 * ============================================================================
 *
 * This example demonstrates I2C communication with a TPIC2810 LED controller
 * using DMA for efficient data transfer. It shows a blinking pattern by
 * alternately setting all LEDs on and off through I2C write operations
 * driven by EDMA, repeating for 10 loops.
 *
 * ============================================================================
 */

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/hw_include/soc_config.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/cslr_soc.h>

/* Sub address command as per TPIC2810 datasheet */
#define TPIC2810_CMD_WR_IMMEDIATE           (0x44U)

/* LED States */
#define LED_SET_ALL                         (0xFFU)
#define LED_RESET_ALL                       (0x00U)

I2CLLD_Handle gI2cLldHandle0;
I2C_ExtendedParams extendedParamsTx;

uint8_t txBuf[2U] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/* Declared in i2c_dma_driver.c */
extern uint8_t Board_getSocLedDeviceAddr(void);
extern void    I2C_dmaSetup(void);
extern void    I2C_dmaCleanup(void);
extern int32_t I2C_lld_writeDMA(I2CLLD_Handle handle, I2C_ExtendedParams *extendedParams);

void i2c_led_blink_dma_main(void *arg)
{
    int32_t  status     = I2C_STS_SUCCESS;
    uint8_t  deviceAddress;
    uint32_t loopcnt    = 10U;
    uint32_t delayMsec  = 100U;

    Drivers_open();
    Board_driversOpen();

    gI2cLldHandle0 = (I2CLLD_Handle)(gI2cLldHandle[0]);

    DebugP_log("[I2C] LED Blink DMA Test Started ...\r\n");
    DebugP_log("LED will Blink for %d loop in DMA Mode ...\r\n", loopcnt);

    deviceAddress = Board_getSocLedDeviceAddr();

    extendedParamsTx.deviceAddress = deviceAddress;
    extendedParamsTx.buffer        = txBuf;
    extendedParamsTx.size          = 2U;
    extendedParamsTx.expandSA      = false;

    /* Allocate EDMA channels */
    I2C_dmaSetup();

    while (loopcnt-- != 0U)
    {
        /* Turn all LEDs on */
        txBuf[0U] = (uint8_t)TPIC2810_CMD_WR_IMMEDIATE;
        txBuf[1U] = (uint8_t)LED_SET_ALL;
        status = I2C_lld_writeDMA(gI2cLldHandle0, &extendedParamsTx);
        if (status != I2C_STS_SUCCESS)
        {
            break;
        }
        gI2cLldHandle0->Clock_uSleep(delayMsec * 1000U);

        /* Turn all LEDs off */
        txBuf[0U] = (uint8_t)TPIC2810_CMD_WR_IMMEDIATE;
        txBuf[1U] = (uint8_t)LED_RESET_ALL;
        status = I2C_lld_writeDMA(gI2cLldHandle0, &extendedParamsTx);
        if (status != I2C_STS_SUCCESS)
        {
            break;
        }
        gI2cLldHandle0->Clock_uSleep(delayMsec * 1000U);
    }

    /* Free EDMA channels */
    I2C_dmaCleanup();

    if (status == I2C_STS_SUCCESS)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

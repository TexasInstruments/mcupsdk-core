/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#include <drivers/gpio.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define LED_ON           (0x01U)
#define LED_OFF          (0x00U)
#define LED_BLINK_COUNT  (10U)


/*
 * This example configures a GPIO pin connected to an LED on the EVM in
 * output mode.
 * The application toggles the LED on/off using RTI timer.
 */

volatile uint32_t gLedState, gBlinkCount;
uint32_t gpioBaseAddr, pinNum;

void rti_led_blink(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[RTI LED Blink Test] Starting ...\r\n");

    /* Get address after translation translate */
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_LED_BASE_ADDR);
    pinNum       = GPIO_LED_PIN;
    gLedState = LED_ON;
    gBlinkCount = 0;

    /* Set LED GPIO pin in output mode */
    GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_LED_DIR);
    /* Set LED GPIO pin HIGH */
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);

    /* Start the RTI counter */
    (void)RTI_counterEnable(CONFIG_RTI0_BASE_ADDR, RTI_TMR_CNT_BLK_INDEX_0);

    DebugP_log("[RTI LED Blink Test] Timer Started...\r\n");

    /* Wait until the LED is blinked specified number of times */
    while(gBlinkCount < LED_BLINK_COUNT);

    /* Stop the RTI counter */
    (void)RTI_counterDisable(CONFIG_RTI0_BASE_ADDR, RTI_TMR_CNT_BLK_INDEX_0);

    DebugP_log("[RTI LED Blink Test] Timer Stopped...\r\n");

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

void rtiEvent0(void)
{
    if(gLedState == LED_ON)
    {
        GPIO_pinWriteLow(gpioBaseAddr, pinNum);
        gLedState = LED_OFF;
    }
    else{
        GPIO_pinWriteHigh(gpioBaseAddr, pinNum);
        gLedState = LED_ON;
    }
    gBlinkCount++;
}
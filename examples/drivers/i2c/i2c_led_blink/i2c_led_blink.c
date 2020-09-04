/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

extern uint32_t Board_getnumLedPerGroup(void);

void i2c_led_blink_main(void *args)
{
    int32_t             status;
    uint32_t            loopcnt = 10U, ledCnt, delayMsec = 100U, numLedPerGroup;
    LED_Handle          ledHandle;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("I2C LED Blink Test Started ...\r\n");
    DebugP_log("LED will Blink for %d loop ...\r\n", loopcnt);

    ledHandle = gLedHandle[CONFIG_LED0];
    DebugP_assert(NULL != ledHandle);

    numLedPerGroup = Board_getnumLedPerGroup();

    /* ON all LED when more than one LED is present - only then mask API is supported */
    if(numLedPerGroup > 1U)
    {
        status = LED_setMask(ledHandle, 0xFFU);
        DebugP_assert(SystemP_SUCCESS == status);
        ClockP_usleep(delayMsec * 1000U);
    }

    while(loopcnt > 0U)
    {
        for(ledCnt = 0U; ledCnt < numLedPerGroup; ledCnt++)
        {
            status = LED_off(ledHandle, ledCnt);
            DebugP_assert(SystemP_SUCCESS == status);
            ClockP_usleep(delayMsec * 1000U);
        }

        for(ledCnt = 0U; ledCnt < numLedPerGroup; ledCnt++)
        {
            status = LED_on(ledHandle, ledCnt);
            DebugP_assert(SystemP_SUCCESS == status);
            ClockP_usleep(delayMsec * 1000U);
        }

        loopcnt--;
    }

    if(numLedPerGroup > 1U)
    {
        /* OFF all LED when more than one LED is present - only then mask API is supported */
        status = LED_setMask(ledHandle, 0x00U);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    DebugP_log("I2C LED Blink Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

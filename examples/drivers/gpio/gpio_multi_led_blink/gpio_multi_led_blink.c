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

/*
 * This example configures multiple GPIO pins connected to LEDs on the EVM in
 * output mode.
 * The application toggles the each LED on/off for 10 seconds and exits.
 */

void gpio_multi_led_blink_main(void *args)
{
    int32_t     status;
    uint32_t    delaySec = 1, ledCnt, loopcnt;
    LED_Handle  ledHandle;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("GPIO Multi LED Blink Test Started ...\r\n");
    DebugP_log("Each LED will Blink for %d seconds ...\r\n", (delaySec * 10));

    for(ledCnt = 0U; ledCnt < CONFIG_LED_NUM_INSTANCES; ledCnt++)
    {
        ledHandle = gLedHandle[ledCnt];
        DebugP_assert(NULL != ledHandle);
        loopcnt = 5U;

        while(loopcnt > 0)
        {
            status = LED_on(ledHandle, 0U);
            DebugP_assert(SystemP_SUCCESS == status);
            ClockP_sleep(delaySec);

            status = LED_off(ledHandle, 0U);
            DebugP_assert(SystemP_SUCCESS == status);
            ClockP_sleep(delaySec);

            loopcnt--;
        }
    }
    DebugP_log("GPIO Multi LED Blink Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

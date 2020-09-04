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
#include <kernel/dpl/HwiP.h>
#include <drivers/watchdog.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define NUM_OF_ITERATIONS    (3U)

/*
 * This example uses the WDT module in reset mode to trigger warm reset.
 *
 * In this example WDT0 is used, the user can also select a
 * different one through syscfg.
 *
 * User can specify paramters like window size, and expiry time to trigger
 * the warm reset.
 *
 */

void watchdog_reset_mode_main(void *args)
{
    /* Watchdog timer expiry time in millisecond */
    uint32_t wdtExpiryTimeinMs = gWatchdogParams[CONFIG_WDT0].expirationTime;
    uint32_t index = 0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("Watchdog reset Mode Test Started ...\r\n");

    DebugP_log("Servicing WDT for few iterations \r\n");

    /* Service the WDT for more than expiry time */
    for(index = 0; index < NUM_OF_ITERATIONS; index++)
    {
        /* Need to service the WDT in open window */
        while(Watchdog_isClosedWindow(gWatchdogHandle[CONFIG_WDT0]) == true);
        /* Clear Watchdog timer */
        Watchdog_clear(gWatchdogHandle[CONFIG_WDT0]);
        /* wait for (expiry time/2 ) time */
        ClockP_usleep((wdtExpiryTimeinMs*100)/2);
    }

    DebugP_log("Watchdog triggers warm reset in %d (ms)\r\n", wdtExpiryTimeinMs);
    DebugP_log("All tests have passed!!\r\n");

    /* Wait till WDT triggers the reset */
    while(1)
    {
        DebugP_log("Prints will be stopped after reset \r\n");
    }

}

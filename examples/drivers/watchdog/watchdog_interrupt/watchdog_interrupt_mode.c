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
#include <drivers/soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

volatile uint32_t   gWatchdogInt = 0;

void   watchdogCallback(void *arg);
/*
 * This example uses the WDT module in non reset mode to generate NMI Interrupt.
 * 
 * The Watchdog interrupt is configured as a non-maskable interrupt and the user-defined
 * callback function is registered. ESM module is configured with ESM Group 2 number and
 * ESM NMI number to generate a non-maskable interrupt to the CPU.
 *
 * The callback function in the application handles the watchdog interrupt
 */

void watchdogCallback(void *arg)
{
    gWatchdogInt++;
    if (gWatchdogInt < 10)
    {
        Watchdog_clear(gWatchdogHandle[CONFIG_WDT0]);
    }
    return;
}

void watchdog_interrupt_main(void *args)
{

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
#if defined(SOC_AM64X) || defined(SOC_AM243X)
    HwiP_Params             hwiPrms;
    int32_t                 status = SystemP_SUCCESS;
    static HwiP_Object       gRtiHwiObject;
    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_WDT0_INTR;
    hwiPrms.callback    = &watchdogCallback;
    status              = HwiP_construct(&gRtiHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
    DebugP_log("Watchdog interrupt Mode Test Started ...\r\n");

    while (gWatchdogInt == 0);
  
    DebugP_log("Watchdog Driver NMI received\r\n");
    
    DebugP_log("All tests have passed!!\r\n");
    
    Board_driversClose();
    Drivers_close();

}

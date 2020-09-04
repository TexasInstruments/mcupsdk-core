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

//
// Included Files
//
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
This example configures ePWMx as follows
 - ePWMx with DCAEVT1 forcing the ePWM output LOW
 - GPIO48 is used as the input to the INPUT XBAR INPUT1
 - INPUT1 (from INPUT XBAR) is used as the source for DCAEVT1
 - GPIO122 is set to output and toggled in the main loop to trip the PWM

 - ePWMx with DCBEVT1 forcing the ePWM output LOW
 - GPIO48 is used as the input to the INPUT XBAR INPUT1
 - INPUT1 (from INPUT XBAR) is used as the source for DCBEVT1
 - GPIO122 is set to output and toggled in the main loop to trip the PWM
 - DCBEVT1 uses the filtered version of DCBEVT1
 - The DCFILT signal uses the valley switching module to delay the
 - DCFILT signal by a software defined DELAY value.

*/

#define EPWM_TBCTR          (8U)

/* Global variables and objects */
static HwiP_Object  gEpwmHwiObject;

/* Function Prototypes */
static void App_epwmIntrISR(void *handle);

/* Get Address of ePWM */
uint32_t gEpwmBaseAddr;


void epwm_valley_switching_main(void *args)
{
    int32_t  status;
    HwiP_Params  hwiPrms;
    uint32_t count = 0;

    GPIO_setDirMode(CSL_GPIO0_U_BASE, GPIO_OUTPUT_PIN, 0);
    GPIO_pinWriteHigh(CSL_GPIO0_U_BASE, GPIO_OUTPUT_PIN);

    GPIO_setDirMode(GPIO_INPUT_BASE_ADDR, GPIO_INPUT_PIN, GPIO_INPUT_DIR);

    Drivers_open();
    Board_driversOpen();

    /* Get Address of ePWM */
    gEpwmBaseAddr = CONFIG_EPWMx_BASE_ADDR;

    DebugP_log("EPWM Valley Switching Test Started ...\r\n");

    /* For EPWM 0 */

    /* Register & enable interrupt */
     HwiP_Params_init(&hwiPrms);
    /* Integrate with Syscfg */
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_epwmIntrISR;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    EPWM_clearTripZoneFlag(gEpwmBaseAddr, EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCBEVT1 | EPWM_TZ_INTERRUPT);
    EPWM_clearOneShotTripZoneFlag(gEpwmBaseAddr, EPWM_TZ_OST_FLAG_DCBEVT1);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);

    volatile uint16_t *reg = (uint16_t *)(gEpwmBaseAddr + EPWM_TBCTR);

    for(;count<10;count++)
    {
        //
        // PULL Trip GPIO LOW from CTR=4000 to 11000
        // PULL Trip GPIO HIGH at CTR=11000
        //
        while(*reg < 4000);
        GPIO_pinWriteLow(CSL_GPIO0_U_BASE, GPIO_OUTPUT_PIN);
        while(*reg < 11000);
        GPIO_pinWriteHigh(CSL_GPIO0_U_BASE, GPIO_OUTPUT_PIN);
        while(*reg > 3000);
        GPIO_pinWriteHigh(CSL_GPIO0_U_BASE, GPIO_OUTPUT_PIN);
    }

    DebugP_log("EPWM Valley Switching Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    EPWM_disableInterrupt(gEpwmBaseAddr);
    HwiP_destruct(&gEpwmHwiObject);
    Board_driversClose();
    Drivers_close();
}


static void App_epwmIntrISR(void *handle)
{
    EPWM_clearTripZoneFlag(gEpwmBaseAddr, EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCBEVT1 | EPWM_TZ_INTERRUPT);
    EPWM_clearOneShotTripZoneFlag(gEpwmBaseAddr, EPWM_TZ_OST_FLAG_DCBEVT1);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);
    return;
}
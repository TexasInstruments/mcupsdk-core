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
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
This example modifies the MEP control registers to show edge displacement
for high-resolution period with ePWM in Up-Down count mode
due to the HRPWM control extension of the respective ePWM module.

External Connections
Monitor ePWM0/1/2/3/4 A/B pins on an oscilloscope.
 */

//
// Defines
//
#define EPWM_TIMER_TBPRD    20UL
#define LAST_EPWM_INDEX_FOR_EXAMPLE    5
#define MIN_HRPWM_PRD_PERCENT   0.2

//
// Globals
//

float periodFine = MIN_HRPWM_PRD_PERCENT;
uint16_t status;
uint16_t i=0;
uint16_t MEP_ScaleFactor = 33; // TBCLK/MEP_step_size_am263x = 5ns/150ps

/* variable to hold base address of EPWM that is used */
uint32_t gEpwmBaseAddr1, gEpwmBaseAddr2, gEpwmBaseAddr3, gEpwmBaseAddr4, gEpwmBaseAddr5;

volatile uint32_t ePWM[] =
   {CONFIG_EPWM0_BASE_ADDR, CONFIG_EPWM1_BASE_ADDR, CONFIG_EPWM2_BASE_ADDR, CONFIG_EPWM3_BASE_ADDR, CONFIG_EPWM4_BASE_ADDR};

// Function Prototypes
//
void initHRPWM(uint32_t period);

void epwm_hr_updown_main(void *args)
{
    uint16_t i = 0;
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("EPWM High Resolution Up Down Test Started ...\r\n");

     /* Get Address of ePWM */
    gEpwmBaseAddr1 = CONFIG_EPWM0_BASE_ADDR;
    gEpwmBaseAddr2 = CONFIG_EPWM1_BASE_ADDR;
    gEpwmBaseAddr3 = CONFIG_EPWM2_BASE_ADDR;
    gEpwmBaseAddr4 = CONFIG_EPWM3_BASE_ADDR;
    gEpwmBaseAddr5 = CONFIG_EPWM4_BASE_ADDR;

    initHRPWM(EPWM_TIMER_TBPRD);

    for(;i<10;i++)
    {
         //
         // Sweep DutyFine
         //
         for(periodFine = MIN_HRPWM_PRD_PERCENT; periodFine < 0.9; periodFine += 0.01)
         {
             ClockP_sleep(10);
             for(i=0; i<LAST_EPWM_INDEX_FOR_EXAMPLE; i++)
             {
                 float count = ((EPWM_TIMER_TBPRD-1) << 8UL) + (float)(periodFine * 256);
                 uint32_t compCount = count;
                 HRPWM_setHiResTimeBasePeriod(ePWM[i], compCount);
             }
         }
     }

    DebugP_log("EPWM High Resolution Up Down Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

void initHRPWM(uint32_t period)
{
    HRPWM_setMEPStep(CSL_CONTROLSS_OTTOCAL0_U_BASE, MEP_ScaleFactor);

    uint16_t j;

    //
    // ePWM channel register configuration with HRPWM
    //
    for (j=0;j<LAST_EPWM_INDEX_FOR_EXAMPLE;j++)
    {
        //
        // set duty 50% initially
        //
        HRPWM_setCounterCompareValue(ePWM[j], HRPWM_COUNTER_COMPARE_A, (period/2 << 8));
        HRPWM_setCounterCompareValue(ePWM[j], HRPWM_COUNTER_COMPARE_B, (period/2 << 8));

        HRPWM_setMEPEdgeSelect(ePWM[j], HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE);
        HRPWM_setMEPControlMode(ePWM[j], HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
        HRPWM_setCounterCompareShadowLoadEvent(ePWM[j], HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO_PERIOD);

        HRPWM_setMEPEdgeSelect(ePWM[j], HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE);
        HRPWM_setMEPControlMode(ePWM[j], HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
        HRPWM_setCounterCompareShadowLoadEvent(ePWM[j], HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO_PERIOD);
        HRPWM_enableAutoConversion(ePWM[j]);

        //
        // Turn on high-resolution period control.
        //

        HRPWM_enablePeriodControl(ePWM[j]);
        HRPWM_enablePhaseShiftLoad(ePWM[j]);
    }
}
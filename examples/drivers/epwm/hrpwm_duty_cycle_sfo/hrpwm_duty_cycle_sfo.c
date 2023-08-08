/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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

#include <calibration/sfo.h>

#define PWM_PERIOD_CYCLES               100UL
#define PWM_DUTY_PERCENT_INITIAL        50
#define MIN_HRPWM_DUTY_PERCENT          4.0/((float)PWM_PERIOD_CYCLES)*100.0

uint16_t MEP_ScaleFactor;
uint32_t gOttoCal_base = CSL_CONTROLSS_OTTOCAL0_U_BASE;


extern int SFO_CAL;
extern int MEP_SF[(PWM_CH_MAX + 1)];

void initHRPWM_duty(uint32_t base, uint32_t period, uint32_t duty_percent)
{
    //
    // ePWM channel register configuration with HRPWM
    //

    EPWM_setEmulationMode(base, EPWM_EMULATION_FREE_RUN);

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(base, period-1);
    EPWM_setPhaseShift(base, 0U);
    EPWM_setTimeBaseCounter(base, 0U);

    //
    // set duty 50% initially
    //
    HRPWM_setCounterCompareValue(base, HRPWM_COUNTER_COMPARE_A, (((period*duty_percent)/100) << 8));
    HRPWM_setCounterCompareValue(base, HRPWM_COUNTER_COMPARE_B, (((period*duty_percent)/100) << 8));


    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setClockPrescaler(base,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(base,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(base,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set actions
    //

    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);


    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);


    HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_FALLING_EDGE);
    HRPWM_setMEPControlMode(base, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setCounterCompareShadowLoadEvent(base, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);

    HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_FALLING_EDGE);
    HRPWM_setMEPControlMode(base, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setCounterCompareShadowLoadEvent(base, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);

    HRPWM_enableAutoConversion(base);


    //
    // Turn on high-resolution period control.
    //

    HRPWM_disablePeriodControl(base);
    HRPWM_disablePhaseShiftLoad(base);

}


/*
    This example modifies the MEP control registers to show edge displacement for high-resolution duty cycle with ePWM in Up count mode due to the HRPWM control extension of the respective ePWM module.

    External Connections: Monitor ePWM0 A/B pins on an oscilloscope.
*/

void hrpwm_duty_cycle_sfo(void *args)
{
    int32_t  status = SFO_INCOMPLETE;
    float dutyFine = MIN_HRPWM_DUTY_PERCENT;
    uint32_t base = CSL_CONTROLSS_G0_EPWM0_U_BASE;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("EPWM High Resolution Duty Cycle Test Started ...\r\n");
    DebugP_log("Please observe pins HSEC 49 and HSEC 51.");

    /*
    *  Calling SFO() updates the HRMSTEP register with calibrated MEP_ScaleFactor.
    *  HRMSTEP must be populated with a scale factor value prior to enabling
    *  high resolution period control.
    */
    while(status == SFO_INCOMPLETE)
    {
        status = SFO();
        if(status == SFO_ERROR)
        {
            DebugP_log("SFO status=%u , ", status);
        }
    }

   initHRPWM_duty(CSL_CONTROLSS_G0_EPWM0_U_BASE, PWM_PERIOD_CYCLES, PWM_DUTY_PERCENT_INITIAL);


    for(dutyFine = MIN_HRPWM_DUTY_PERCENT; dutyFine < 99.9; dutyFine += 0.15)
    {
        ClockP_sleep(1);

        float count = (dutyFine * (float)(PWM_PERIOD_CYCLES << 8))/100;
        uint32_t compCount = (count);
        HRPWM_setCounterCompareValue(base, HRPWM_COUNTER_COMPARE_A, compCount);
        HRPWM_setCounterCompareValue(base, HRPWM_COUNTER_COMPARE_B, compCount);

        DebugP_log("\r\nHigh Resolution duty cycle %d.%02d percent. ", (uint8_t)dutyFine, (uint8_t)((dutyFine - (uint8_t)dutyFine)*100));
        float pos_width = (dutyFine * (float)(PWM_PERIOD_CYCLES) * 5)/100;
        DebugP_log("(positive width = %d.%03d nanoseconds)", (uint8_t)pos_width, (uint8_t)((pos_width - (uint8_t)pos_width)*1000));
        DebugP_log("\r\nEPWM0 CMP = %d, CMPHR = %d", ((compCount>>8)&0xFFFF), (compCount&0xFF));


        DebugP_log("\r\n\r\nCalibration... ");

        //
        // Call the scale factor optimizer lib function SFO()
        // periodically to track for any change due to temp/voltage.
        // This function generates MEP_ScaleFactor by running the
        // MEP calibration module in the HRPWM logic. This scale
        // factor can be used for all HRPWM channels. The SFO()
        // function also updates the HRMSTEP register with the
        // scale factor value.
        //
        status = SFO(); // in background, MEP calibration module continuously updates MEP_ScaleFactor

        if (status == SFO_ERROR)
        {
        // SFO function returns 2 if an error occurs & #of MEP steps/coarse step exceeds maximum of 255.
        DebugP_log("SFO status=%u , ", status);
        }

        if(status == 0)
        {
        DebugP_log("Running...\r\n");
        }

        if(status == 1)
        {
        DebugP_log("Complete\r\n");

        DebugP_log("SFO status=%u , ", status);
        DebugP_log("MEP_ScaleFactor=%u", MEP_ScaleFactor);
        DebugP_log("\r\n");
        }


    }

    DebugP_log("EPWM Duty Cycle Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}








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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/dac.h>
#include "ti_drivers_config.h"
#include <math.h>
#include "ti_dpl_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example uses the DAC module to generate a ramp signal on
 * the DAC output
 *
 * The voltage can be specified by the user. The max value can be 4095.
 *
 * User can also configure the run time by changing the APP_NUM_ITER macro.
 * Frequency can also be changed by changing the time period macro.
 *
 * This example also showcases how to initialize and configure the
 * DAC module.
 *
 * The ramp wave produced can be viewed on the DAC output pin using
 * an oscilloscope,
 */

#define PI 2*asin(1.0)
/* Frequency of Timer ISR */
#define TIMER_ISR_FREQUENCY 20000

/* Frequency of output ramp wave */
#define OUTPUT_FREQUENCY 50

/* Time duration in seconds for which ramp waves are produced */
#define SINCE_OUTPUT_TIME 10

/* Global variables and objects */
/*
 * Variable to store value which is gradually incremented in the ISR
 * to create the ramp wave.
 */
float gAlpha = 0;
/*
 * Value by which the value is incremented in the Timer ISR. This determines
 * the frequency of the ramp wave
 */
float gStepValue = 0;



/* Base address of DAC instance */
uint32_t gDacBaseAddr = CONFIG_DAC0_BASE_ADDR;
/* Final value that gets written into DAC shadow register */
uint16_t gDacOutput = 0;

void dac_ramp_wave_main(void *args)
{
    /* Calculate the step value to obtain ramp wave of required frquency */
    gStepValue =  (((float)OUTPUT_FREQUENCY)*2*PI/((float)TIMER_ISR_FREQUENCY));

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("DAC Ramp Wave Test Started ...\r\n");

    /* Start the timer to get interrupt at a rate of 20KHz*/
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

    /*
     * Wait while TIMER causes interrupts. DAC is updated until
     * required number of ramp waves are produced.
     */
    ClockP_sleep(SINCE_OUTPUT_TIME);

    /* Stop the timer to halt generation of ramp waves */
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

    DebugP_log("DAC Ramp Wave Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

void timerISR(void)
{
    /* Calculate the ramp value. Then scale and align it to the DAC output */
    gDacOutput =((uint16_t)((0x10000 *(gAlpha/(2*PI))))) >> 4;

    /* Update value to be used in the next cycle */
    gAlpha = gAlpha + gStepValue;

    /* Reset value to 0 once a cyce is complete */
    if(gAlpha >= 2*PI)
    {
        gAlpha = 0;
    }

    /* Write current ramp wave value to buffered DAC */
    DAC_setShadowValue(gDacBaseAddr, gDacOutput);
}
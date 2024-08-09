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
#include <drivers/dac.h>
#include <math.h>
#include "ti_drivers_config.h"
#include "ti_dpl_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example uses the DAC module to generate a sine wave on
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
 * The sine wave produced can be viewed on the DAC output pin using
 * an oscilloscope,
 */

/* Frequency of Timer ISR */
#define TIMER_ISR_FREQUENCY 20000
/* Frequency of output since wave */
#define OUTPUT_FREQUENCY 50
/* Value of pi calculated from inverse sine operation*/
#define PI (2 * asin(1.0))
/* Time duration in seconds for which sine waves are produced */
#define SINCE_OUTPUT_TIME 10

/* Global variables and objects */
/*
 * Variable to store angle which is gradually incremented in the ISR
 * to create the sine wave.
 */
float gAlpha = 0;
/*
 * Value by which the angle is incremented in the Timer ISR. This determines
 * the frequency of the sine wave
 */
float gStepValue = 0;
/* Base address of DAC instance */
uint32_t gDacBaseAddr = CONFIG_DAC0_BASE_ADDR;
/* Final value that gets written into DAC shadow register */
uint16_t gDacOutput = 0;

void dac_sine_wave_main(void *args)
{
    /* Calculate the step value to obtain sine wave of required frquency */
    gStepValue = (OUTPUT_FREQUENCY * 2 * PI)/TIMER_ISR_FREQUENCY;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    

    DebugP_log("DAC Sine Wave Test Started ...\r\n");

    /* Start the timer to get interrupt at a rate of 20KHz*/
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

    /*
     * Wait while TIMER causes interrupts. DAC is updated until
     * required number of sine waves are produced.
     */
    ClockP_sleep(SINCE_OUTPUT_TIME);

    /* Stop the timer to halt generation of sine waves */
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

    DebugP_log("DAC Sine Wave Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

void timerISR(void)
{
    /* Calculate the singe of angle. Then scale and align it to the DAC output */
    gDacOutput =((uint16_t)((0x8000 * sin(gAlpha))+0x8000)) >> 4;

    /* Update angle to be used in the next cycle */
    gAlpha = gAlpha + gStepValue;

    /* Reset angle to 0 once a cyce is complete */
    if(gAlpha >= 2*PI)
    {
        gAlpha = 0;
    }

    /* Write current sine value to buffered DAC */
    DAC_setShadowValue(gDacBaseAddr, gDacOutput);
}
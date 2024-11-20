/*
 *  Copyright (C) 2023-2024 Texas Instruments Incorporated
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
#include <drivers/eqep.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 *  Speed and Direction Measurement Using eQEP
 * 
 * This example is used to measure the frequency of an input signal using
 * the eQEP module. ePWM1A is configured to generate this input signal with
 * a frequency of 5KHz. EQEP unit timeout is set which will generate
 * an interrupt every UNIT_PERIOD microsecond for frequency measurement
 *
 * The configuration for this example is as follows
 * - PWM frequency is specified as 5000Hz
 * - UNIT_PERIOD is specified as 10000 us
 * - Min frequency is (1/(2*10ms)) i.e 50Hz
 * - Highest frequency can be (2^32)/(2*10ms)
 * - Resolution of frequency measurement is 50hz
 *
 * freq : Simulated frequency is measured by counting the external
 * input pulses for UNIT_PERIOD (unit timer set to 10 ms).
 * 
 * Internal Connections \n
 * - ePWM1A -> GPIO45 -> INPUTXBAR0 -> PWMXBAR0 -> eQEP0A
 * - ePWM1B -> GPIO46 -> INPUTXBAR1 -> PWMXBAR1 -> eQEP0B
 * 
 */ 

/* Defines */
/* Sysclk frequency */
#define DEVICE_SYSCLK_FREQ  (200000000U)
/* Macro for interrupt pulse */
#define APP_INT_IS_PULSE  (1U)
/* PWM output at 5 kHz */
#define PWM_CLK   5000U
/* Specify the period in microseconds */
#define UNIT_PERIOD  10000U

/* Global variables and objects */
uint32_t gEqepBaseAddr;
uint32_t gEpwmBaseAddr;
static HwiP_Object gEqepHwiObject;

/* Counter to check measurement gets saturated */
uint32_t gCount = 0;
/* Stores the previous position counter value */
uint32_t gOldcount = 0;
/* Measured quadrature signal frequency of motor using eQEP */
int32_t gFreq = 0;
/* Pass or fail indicator */
uint32_t gPass = 0, gFail = 0;

volatile uint32_t gNewcount = 0 ;

/* Function Prototypes */
static void App_eqepIntrISR(void *handle);

void eqep_freq_cal_interrupt_main(void *args)
{
    int32_t status;
    HwiP_Params hwiPrms;

    /* Open drivers
		- Open the UART driver for console
		- Open EPWM and EQEP driver for frequency measurement
		- Open XBAR driver for EQEP interrupt routing to R5F
	*/
    Drivers_open();
    Board_driversOpen();
	
    DebugP_log("EQEP Frequency Measurement Test Started ...\r\n");
	DebugP_log("Please wait few seconds ...\r\n");
	
    gEqepBaseAddr = CONFIG_EQEP0_BASE_ADDR;
    gEpwmBaseAddr = CONFIG_EPWM0_BASE_ADDR;

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    /* Integrate with Syscfg */
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_eqepIntrISR;
    /* Integrate with Syscfg */
    hwiPrms.isPulse     = APP_INT_IS_PULSE;
    status              = HwiP_construct(&gEqepHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Clear eqep interrupt */
   EQEP_clearInterruptStatus(gEqepBaseAddr,EQEP_INT_UNIT_TIME_OUT|EQEP_INT_GLOBAL);

    while( *(volatile uint32_t *) ((uintptr_t) &gCount) < 10)
    {
        //Wait for frequency measurement
    }
	
    if((gPass == 1) && (gFail == 0))
    {
		/* Expected 5000 Hz based on programmed EPWM frequency */
    DebugP_log("Expected frequency = %d Hz, Measured frequency = %d Hz \r\n", PWM_CLK, gFreq);	
	
    DebugP_log("EQEP Frequency Measurement Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Fail\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

/* eqep0 ISR--interrupts once every 4 QCLK counts (one period) */
static void App_eqepIntrISR(void *handle)
{
    /* Increment count value */
    gCount++;

    /* New position counter value */
    gNewcount = EQEP_getPositionLatch(gEqepBaseAddr);

   /* Calculates the number of position in unit time */

   /* Stores the current position count value to oldcount variable */
   gOldcount = gNewcount;

   /* Simulated Frequency calculation 
      gFreq = ((position counts) * 1000000)/ (2 / * Unit_time (in microsecond))
   */
   gFreq = (gNewcount * (uint32_t)1000000U)/(2 * (uint32_t)UNIT_PERIOD);

   //
   // Compares the measured frequency with input frequency
   //
   if (gCount >= 2){
        if(((gFreq - PWM_CLK) <50) && ((PWM_CLK - gFreq ) < 50)){
            gPass = 1; 
            gFail = 0;
        }else {
            gPass = 0;  
            gFail = 1;
        }
    }

   /* Clear interrupt flag */
   EQEP_clearInterruptStatus(gEqepBaseAddr,EQEP_INT_UNIT_TIME_OUT|EQEP_INT_GLOBAL);
}

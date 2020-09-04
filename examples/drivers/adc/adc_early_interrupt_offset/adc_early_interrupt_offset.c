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
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include <drivers/adc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/*
 * Introduction :
 *      The early interrupt feature allows generation (or setting) of INT flag
 * right before the Conversion starts, but after the sample and hold (acquisition)
 * window. This is to facilitate the early updation to the system that the ADC
 * result could be expected. The configurability of offset for the early interrupt
 * allows even more flexibility as of after how many sysclk cycles from the
 * Sample and hold windw should the interrupt be generated.
 *
 * Example Description :
 *       This example demonstrates the early interrupt feature and its offset
 * configurability for the ADC. SOC 1 is set to be triggered when ADC1_INT1 is set,
 * which in turn is set for EOC/SOC0. Hence, SOC1 are supposed to happen at EOC0.
 * The ADC1_INT1 is set as an early interrupt and the offset is varied. which delays
 * the triggering of the SOC1. Although triggered, the SOC1 should wait until the
 * conversion of SOC0 is complete. hence a delay from trigger to acquisition is
 * present in SOC1. This can be obtained by setting the PPB (Post Processing Block)
 * to the SOC1.
 *        So, by reading the varied delay against offset value, the example records
 * the offset configurability of the early interrupt.
 *
 * Note :
 * - The PPB Delay reads minimum of 2 sysclk cycles even for immediate conversion.
 * - ADC Clock is set to prescale factore of 3.
 *
 * SOC Configurations:
 * - SOC 0 :
 * 1. Trigger is set to software.
 * 2. Sample window : 16
 * 3. Samples on Channel 0 (arbitrary)
 *
 * - SOC 1 :
 * 1. Trigger is set to software, ADC1_INT1.
 * 2. Sample window : 16
 * 3. Samples on Channel 0 (arbitrary)
 *
 * PPB Configurations :
 * 1. PPB 1 is set for SOC 1
 *
 * Interrupt Configurations :
 * INT xbar0 is set for ADC 1 INT 2.
 *
 * ADC 1 INT 1 :
 * 1. triggered by EOC/SOC0
 *
 * ADC 1 INT 2 :
 * 1. triggered by EOC/SOC1
 * 2. triggers ISR App_adcISR()
 *
 * ISR configurations :
 * App_adcISR0 :
 * - reads the delays in SOC1.
 * - clears the INT 1, 2 flags
 *
 * External Connections :
 * - Am263x-CC : Feed analog inputs to channels
 *          ADC1_AIN0 - HSEC Pin 18
 * - Am263x-LP : Feed analog inputs to channels
 *          ADC1_AIN0 - J1/3 Pin 24
 *
 * Watch Variables :
 *          gAdc1soc1Delay[] - holds the delay values for ADC1_SOC1
 */


/* Number of sysclk cycles for SOC conversion "after Sample and hold" */
#define SOC_CONVERSION_CYCLES     32

/* Global variables and objects */
volatile uint32_t gAdc1soc1Delay[SOC_CONVERSION_CYCLES];



uint32_t gAdc1baseAddr = CONFIG_ADC1_BASE_ADDR;

/* Variable to check ISR Completion */
volatile uint8_t gISRcomplete = 0;

volatile uint32_t gIntOffset = 0;

/* Variable for ISR registring */
static HwiP_Object  gAdcHwiObject;


void App_adcISR(void *args);


void adc_early_interrupt_offset_main(void *args)
{
    /* Refer to Syscfg for SOC, PPB, Interrupt and INT xbar Configurations.*/
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC Early Interrupt and Configurable Offset Test Started\r\n");

    /* Registring ISR*/
    int32_t  status;
    HwiP_Params  hwiPrms;

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_adcISR;
    status              = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Clear ADC 1 INT 1 flag*/
    ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
    /* Clear ADC 1 INT 2 flag*/
    ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER2);

    /* wait until required conversions are completed */
    while(gIntOffset < SOC_CONVERSION_CYCLES)
    {
        /* Setting the offset value. This is offset between the negative edge
           of the S&H window to the early interrupt.*/
        ADC_setInterruptCycleOffset(gAdc1baseAddr,gIntOffset);

        /* Triggering the SOC0 to generate INT1. this subsequently triggers the
           SOC1, which generates the INT2 and triggers the App_adcISR.*/
        ADC_forceSOC(gAdc1baseAddr, ADC_SOC_NUMBER0);

        /* wait unitl the ISR is complete */
        while(gISRcomplete == 0);
        /* resetting the ISR Completion variable*/
        gISRcomplete = 0;
        /* incrementing the gIntOffset*/
        gIntOffset++;
    }

    /* Printing the Delay stamp arrays.*/
    DebugP_log("\tOffset\tDelay\r\n");
    for(int index=0; index<SOC_CONVERSION_CYCLES; index++)
    {
        DebugP_log("\t%d\t%d\r\n",index, gAdc1soc1Delay[index]);
    }
    DebugP_log("ADC Early Interrupt and Configurable Offset Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}
/* ISR for ADC 1 INT 2*/
void App_adcISR(void *args)
{
    /* Reading the PPB delay assosciated with the SOC1*/
    gAdc1soc1Delay[gIntOffset] = ADC_getPPBDelayTimeStamp(gAdc1baseAddr, ADC_PPB_NUMBER1);
    /* Updating the ISR completion variable*/
    gISRcomplete = 1;

    /* Clearing INT flags*/
    ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER2);
    if(true == ADC_getInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
    }
    if(true == ADC_getInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER2))
    {
        ADC_clearInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER2);
    }
}


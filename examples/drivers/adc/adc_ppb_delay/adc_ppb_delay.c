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
 * Example Description :
 *      This example demonstrates the PPB delay time stamp feature of the
 * ADC. The PPBs (Post Processing Blocks) in ADC offer timestamping the delay
 * between the conversion of given SOC and the trigger(associated) occurrence.
 *
 * Note :
 * - if a trigger occurred and the conversion occurred immediately, i.e., without
 * any delay, the PPB captures a value '2'. and contains '0' as reset value.
 * Hence, 0 in Delay represents SOC hasn't converted or there is an overflow of
 * Delay counter.
 *
 * SOC Configurations:
 * - SOC 0 :
 * 1. Trigger is set to EPWM0SOCA.
 * 2. Sample window : 17
 * 3. Samples on Channel 0 (arbitrary)
 *
 * - SOC 1 :
 * 1. Trigger is set to EPWM1SOCA.
 * 2. Sample window : 17
 * 3. Samples on Channel 1 (arbitrary)
 *
 * PPB Configurations :
 * 1. PPB 1 is set for SOC 0
 * 2. PPB 2 is set for SOC 1
 *
 * Interrupt Configurations :
 * INT xbar0, INT xbar1 are set for ADC 1 INT 1 and ADC 1 INT 2 respectively.
 *
 * ADC 1 INT 1 :
 * 1. triggered by EOC/SOC0
 * 2. triggers ISR App_adcISR0()
 *
 * ADC 1 INT 2 :
 * 1. triggered by EOC/SOC1
 * 2. triggers ISR App_adcISR1()
 *
 * ISR configurations :
 * App_adcISR0 :
 * - reads the delays in SOC0 and SOC1.
 * - clears the INT 1 flag
 * App_adcISR1 :
 * - clears the INT 1 flag
 *
 * EPWM Configurations:
 * EPWM 0 :
 * 1. TimeBase Period is set to 2048.
 * 2. Generates EPWM0SOCA when the TimeBase counter equals TimeBase Period.
 * 3. Counter Mode will be set to Up-count mode in the example.
 *
 * EPWM 1 :
 * 1. TimeBase Period is set to 9999.
 * 2. Generates EPWM1SOCA when the TimeBase Counter equals TimeBase Period.
 * 3. Counter Mode will be set to Up-count mode in the example.
 *
 * External Connections :
 * - Am263x-CC : Feed analog inputs to channels
 *          ADC1_AIN0 - HSEC Pin 18
 *          ADC1_AIN1 - HSEC Pin 20
 * - Am263x-LP : Feed analog inputs to channels
 *          ADC1_AIN0 - J1/3 Pin 24
 *          ADC1_AIN1 - J1/3 Pin 29
 *
 * Watch Variables :
 *          gAdc1soc0Delay[] - holds the delay values for ADC1_SOC0
 *          gAdc1soc1Delay[] - holds the delay values for ADC1_SOC1
 */


/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT     256

/* Global variables and objects */
volatile uint32_t gAdc1soc0Delay[ADC_CONVERSION_COUNT];
volatile uint32_t gAdc1soc1Delay[ADC_CONVERSION_COUNT];

volatile uint32_t gIndex = 0;


uint32_t gAdc1baseAddr = CONFIG_ADC1_BASE_ADDR;
uint32_t gEpwm0baseAddr = CONFIG_EPWM0_BASE_ADDR;
uint32_t gEpwm1baseAddr = CONFIG_EPWM1_BASE_ADDR;


/* Variable to store the count of completed conversions
Initialising conversion count to 0 */
volatile uint32_t gAdcConversionCount = 0;
static HwiP_Object  gAdcHwiObject1;
static HwiP_Object  gAdcHwiObject2;


void App_adcISR0(void *args);
void App_adcISR1(void *args);


void adc_ppb_delay_main(void *args)
{
    /* Refer to Syscfg for SOC, PPB, EPWM, Interrupt and INT xbar Configurations.*/
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();



    DebugP_log("ADC PPB Delay Test Started\r\n");

    /* Registring ISRs*/
    int32_t  status;
    HwiP_Params  hwiPrms;

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_adcISR0;
    status              = HwiP_construct(&gAdcHwiObject1, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms.callback    = &App_adcISR1;
    status              = HwiP_construct(&gAdcHwiObject2, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    gIndex = 0;
    /* Clear ADC 1 INT 1 flag*/
    ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
    /* Clear ADC 1 INT 2 flag*/
    ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER2);

    /* Run the EPWM counters to initiate generations of EPWMxSOCA to start conversions*/
	EPWM_setTimeBaseCounterMode(gEpwm0baseAddr, EPWM_COUNTER_MODE_UP);
    EPWM_setTimeBaseCounterMode(gEpwm1baseAddr, EPWM_COUNTER_MODE_UP);




    /* wait until required conversions are completed */
    while(gAdcConversionCount < ADC_CONVERSION_COUNT);

    /* Printing the Delay stamp arrays.*/
    DebugP_log("Delays associated with\r\n");
    DebugP_log("conversion\tSOC 0\tSOC 1\r\n");
    for(gIndex = 0; gIndex < ADC_CONVERSION_COUNT; gIndex++)
    {
        if((gAdc1soc0Delay[gIndex] > 2) || (gAdc1soc1Delay[gIndex] > 2))
        {
            DebugP_log("\t%d\t%d\t%d\r\n",gIndex, gAdc1soc0Delay[gIndex], gAdc1soc1Delay[gIndex]);
        }
        else
        {
            continue;
        }
    }
    DebugP_log("ADC PPB Delay Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}
/* ISR for ADC 1 INT 1*/
void App_adcISR0(void *args)
{
    /* Clear ADC 1 INT 1 flag*/
    ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
    /* Clear ADC 1 INT 1 overflow flag if any*/
    if(true == ADC_getInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
    }
    /* Read the Delays Associated with SOC0 and SOC1*/
    if(gIndex < ADC_CONVERSION_COUNT)
    {
        gAdc1soc0Delay[gIndex] = ADC_getPPBDelayTimeStamp(gAdc1baseAddr, ADC_PPB_NUMBER1);
        gAdc1soc1Delay[gIndex] = ADC_getPPBDelayTimeStamp(gAdc1baseAddr, ADC_PPB_NUMBER2);
        gIndex++;

    }

    if(gAdcConversionCount < ADC_CONVERSION_COUNT)
    {
        gAdcConversionCount++;
    }
    else
    {
        ADC_disableInterrupt(gAdc1baseAddr, ADC_INT_NUMBER1);
        ADC_disableInterrupt(gAdc1baseAddr, ADC_INT_NUMBER2);

        EPWM_setTimeBaseCounterMode(gEpwm0baseAddr, EPWM_COUNTER_MODE_STOP_FREEZE);
        EPWM_setTimeBaseCounterMode(gEpwm1baseAddr, EPWM_COUNTER_MODE_STOP_FREEZE);
    }
}
/* ISR for ADC 1 INT 2*/
void App_adcISR1(void *args)
{
    /* Clear ADC 1 INT 2 flag*/
    ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER2);
    /* Clear ADC 1 INT 2 overflow flag if any*/
    if(true == ADC_getInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER2))
    {
        ADC_clearInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER2);
    }
}

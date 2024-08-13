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
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include <drivers/adc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example sets up ePWM0 to periodically trigger a set of conversions
 * (SOC0,1,2) on ADC2 and ADC1. This demonstrates multiple ADCs working
 * together to process a batch of conversions using the available parallelism
 * across multiple ADCs.
 * ADC2 Interrupt ISR is used to read results of both ADC2 and ADC1.
 * The below watch variables can be used to view ADC conversion results.
 *
 * External Connections
 * ADC2_AIN0, ADC2_AIN1, ADC2_AIN2 and ADC1_AIN0, ADC1_AIN1, ADC1_AIN2 pins
 * (if AM261X-LP is used, then : ADC2_AIN0, ADC2_AIN2, ADC2_AIN3 and ADC1_AIN0, ADC1_AIN1, ADC1_AIN2 pins)
 * should be connected to signals to be converted.
 *
 * Watch Variables
 * gAdc2Result0 - Digital representation of the voltage on pin ADC2_AIN0
 * gAdc2result1 - Digital representation of the voltage on pin ADC2_AIN1
 * adc2Result2 - Digital representation of the voltage on pin ADC2_AIN2
 * adc1Result0 - Digital representation of the voltage on pin ADC1_AIN0
 * adc1Result1 - Digital representation of the voltage on pin ADC1_AIN1
 * adc1Result2 - Digital representation of the voltage on pin ADC1_AIN2
 */

/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT     1024

/* Global variables and objects */
/* Variable to store conversion results from all 6 pins  */
uint16_t gAdc2Result0[ADC_CONVERSION_COUNT];
uint16_t gAdc2Result1[ADC_CONVERSION_COUNT];
uint16_t gAdc2Result2[ADC_CONVERSION_COUNT];
uint16_t gAdc1Result0[ADC_CONVERSION_COUNT];
uint16_t gAdc1Result1[ADC_CONVERSION_COUNT];
uint16_t gAdc1Result2[ADC_CONVERSION_COUNT];

uint32_t gAdc2baseAddr = CONFIG_ADC2_BASE_ADDR;
static HwiP_Object  gAdcHwiObject;
/* Variable to store the count of completed conversions */
volatile uint32_t gAdcConversionCount = 0;
/* Function Prototypes */
static void App_adcISR(void *args);

void adc_multiple_soc_epwm_main(void *args)
{
    int32_t  status;
    HwiP_Params  hwiPrms;
    uint32_t     loopCnt = 0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /*
     * In Syscfg, the compare A value is set to 1000 and the period to 1999.
     * EPWM ADC trigger source is Time-base counter equal to CMPA when the
     * timer is incrementing.
     * Since ePWM clock is 50MHz(using divider of 4), this would give 25kHz
     * sampling rate. The sample rate can also be modulated by changing the
     * ePWM period directly (ensure that the compare A value is less than
     * the period).
     */

    DebugP_log("ADC EPWM Triggered Conversion Test Started ...\r\n");

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    /* Integrate with Syscfg */
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_adcISR;
    status              = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_UP);

    /* Wait until required number of ADC conversions are completed */
    while( gAdcConversionCount < ADC_CONVERSION_COUNT );

    /* Remove ePWM trigger to stop ADC conversions */
    EPWM_disableADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A);

    DebugP_log("ADC2 SOC0 : SOC1 : SOC2 : ADC1 SOC0 : SOC1: SOC2 Result register value :\r\n");

    loopCnt = 0;
    /* Print few elements from the result buffer */
    while(loopCnt < ADC_CONVERSION_COUNT)
    {
        DebugP_log("%d : %d : %d : %d : %d : %d\r\n", gAdc2Result0[loopCnt],
         gAdc2Result1[loopCnt], gAdc2Result2[loopCnt], gAdc1Result0[loopCnt],
         gAdc1Result1[loopCnt], gAdc1Result2[loopCnt]);
        loopCnt += 100;
    }

    DebugP_log("ADC EPWM Triggered Conversion Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_adcISR(void *args)
{
    /* Store results */
    gAdc2Result0[gAdcConversionCount] = ADC_readResult(CONFIG_ADC2_RESULT_BASE_ADDR, ADC_SOC_NUMBER0);
    gAdc1Result0[gAdcConversionCount] = ADC_readResult(CONFIG_ADC1_RESULT_BASE_ADDR, ADC_SOC_NUMBER0);
    gAdc2Result1[gAdcConversionCount] = ADC_readResult(CONFIG_ADC2_RESULT_BASE_ADDR, ADC_SOC_NUMBER1);
    gAdc1Result1[gAdcConversionCount] = ADC_readResult(CONFIG_ADC1_RESULT_BASE_ADDR, ADC_SOC_NUMBER1);
    gAdc2Result2[gAdcConversionCount] = ADC_readResult(CONFIG_ADC2_RESULT_BASE_ADDR, ADC_SOC_NUMBER2);
    gAdc1Result2[gAdcConversionCount] = ADC_readResult(CONFIG_ADC1_RESULT_BASE_ADDR, ADC_SOC_NUMBER2);

    /* Update the conversion count */
    gAdcConversionCount++;

    /* Clear the interrupt flag */
    ADC_clearInterruptStatus(gAdc2baseAddr, ADC_INT_NUMBER1);

    /* Check if overflow has occurred */
    if(true == ADC_getInterruptOverflowStatus(gAdc2baseAddr, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(gAdc2baseAddr, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(gAdc2baseAddr, ADC_INT_NUMBER1);
    }
}
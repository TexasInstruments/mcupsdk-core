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
 * This example sets up ePWM0 to periodically trigger a set of conversions
 * (SOC0,1,12,13,14,15) on ADC0 for conversion of inputs on ADC_AIN0, ADC_AIN1 and
 * burst mode conversion on ADC_AIN3.
 *
 * This demonstrates a batch of conversion on ADC0 (inputs on ADC_AIN0 and ADC_AIN1)
 * and burst mode conversion on ADC0 (input ADC_AIN3)
 *
 * ADC0 Interrupt ISR is used to read results of ADC0 (i.e. digital representations
 * of inputs ADC_AIN0, ADC_AIN1 and average of oversampled ADC_AIN3)
 *
 *
 *
 * The below watch variables can be used to view ADC conversion results.
 *
 * Watch Variables
 * gAdc0Result0 - Digital representation of the voltage on pin ADC0_AIN0
 * gAdc0result1 - Digital representation of the voltage on pin ADC0_AIN1
 * gadc0Result2 - Digital representation of the voltage on pin ADC0_AIN3
 *
 * External Connections
 * ADC0_AIN0, ADC0_AIN1, ADC0_AIN3 pins
 * should be connected to signals to be converted.
 *
 * Check example.syscfg
 *
 */


/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT     1024

/* Global variables and objects */
/* Variable to store conversion results from all 3 pins  */
uint16_t gAdc0Result0[ADC_CONVERSION_COUNT];
uint16_t gAdc0Result1[ADC_CONVERSION_COUNT];
uint16_t gAdc0Result2[ADC_CONVERSION_COUNT];



uint32_t gAdc0baseAddr = CONFIG_ADC0_BASE_ADDR;


static HwiP_Object  gAdcHwiObject;


/* Variable to store the count of completed conversions
Initialising conversion count to 0 */
volatile uint32_t gAdcConversionCount = 0;


/* Function Prototypes */
static void App_adcISR(void *args);

void adc_burst_mode_oversampling_main(void *args)
{
    int32_t  status;

    /* Initialising a Interrupt parameter */
    HwiP_Params  hwiPrms;

    /* variable to iterate ADC_CONVERSION_COUNT */
    uint32_t     loopCnt = 0;


    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

        /* Integrate with Syscfg
        * Set up the SOC configurations as follows
        *      SOC0 - input : ADCIN0, trigger : EPWM0 ADCSOCA
        *      SOC1 - input : ADCIN1, trigger : EPWM0 ADCSOCA
        *      SOC 12,13,14,15 : input : ADCIN3, trigger : EPWM0 ADCSOCA

        * In Syscfg, the compare A value is set to 1000 and the period to 1999.
        * EPWM ADC trigger source is Time-base counter equal to CMPA when the
        * timer is incrementing.
        * Since ePWM clock is 50MHz(using divider of 4), this would give 25kHz
        * sampling rate. The sample rate can also be modulated by changing the
        * ePWM period directly (ensure that the compare A value is less than
        * the period).
        */

    DebugP_log("ADC EPWM Triggered Burst Mode Conversions Test Started ...\r\n");

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);


    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.priority    = 0;                        /* setting high priority. optional */
    hwiPrms.callback    = &App_adcISR;
    status              = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Setting the EPWM TB counter mode to up count mode*/
    EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_UP);

    /* Wait until required number of ADC conversions are completed */
    while( gAdcConversionCount < ADC_CONVERSION_COUNT );

    /* Remove ePWM trigger to stop ADC conversions */
    EPWM_disableADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A);


    loopCnt = 0;


    /* Print few elements from the result buffer */
        DebugP_log("ADC0\r\nSOC0    :   SOC1    :   SOC12    :   Result register value :\r\n");

        while(loopCnt < ADC_CONVERSION_COUNT)
        {
            DebugP_log("%d  :   %d  :   %d\r\n",
                        gAdc0Result0[loopCnt],
                        gAdc0Result1[loopCnt],
                        gAdc0Result2[loopCnt]);

            loopCnt += 100;
        }

    DebugP_log("ADC Burst Mode Oversampling Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_adcISR(void *args)
{
    /* Store results */

        /* storing the results of ADC_AIN0 and ADC_AIN1 */
        gAdc0Result0[gAdcConversionCount] = ADC_readResult(CONFIG_ADC0_RESULT_BASE_ADDR, ADC_SOC_NUMBER0);
        gAdc0Result1[gAdcConversionCount] = ADC_readResult(CONFIG_ADC0_RESULT_BASE_ADDR, ADC_SOC_NUMBER1);

        /* Averaging the results of four SOCs (12-15) since burst size is configured for 4 SOCs */
        gAdc0Result2[gAdcConversionCount] =(
            ADC_readResult(CONFIG_ADC0_RESULT_BASE_ADDR, ADC_SOC_NUMBER12)+
            ADC_readResult(CONFIG_ADC0_RESULT_BASE_ADDR, ADC_SOC_NUMBER13)+
            ADC_readResult(CONFIG_ADC0_RESULT_BASE_ADDR, ADC_SOC_NUMBER14)+
            ADC_readResult(CONFIG_ADC0_RESULT_BASE_ADDR, ADC_SOC_NUMBER15)
            )>>2;

    /* Update the conversion count */
    gAdcConversionCount++;

    /* Clear the interrupt flag */
    ADC_clearInterruptStatus(gAdc0baseAddr, ADC_INT_NUMBER1);

    /* Check if overflow has occurred */
    if(true == ADC_getInterruptOverflowStatus(gAdc0baseAddr, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(gAdc0baseAddr, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(gAdc0baseAddr, ADC_INT_NUMBER1);
    }
}

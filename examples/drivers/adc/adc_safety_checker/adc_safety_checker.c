/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
#include <drivers/adc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * Safety Checker module
 *      The safety checker module provides the ability to compare ADC conversion
 * results for safety critical application. It is divided into two parts:
 * 1) Safety checker tiles and 2) Safety checker aggregator. Safety checker tile
 * captures conversion results from multiple ADc modules and compares the
 * absolute value of the difference to the configured tolerance. While Safety
 * checker aggregator takes the computed data and genreate a trip event signal
 * if the result is out of range. The trip event can be sent to INT XBAR and
 * an ISR can be triggered.
 *
 * Example Description:
 * This example compares the absolute value of the two ADC conversion results
 * with the set tolerance value using safety checker module.
 *
 * Channel 0 and 1 of ADC1 are used to compare the two ADC conversions. If
 * the difference between two conversion results exceeds the value configured
 * as tolerance then the ADC safety checker tile generates an out-of-tolerance
 * event. This event will be used by ADC safety checker aggregator to generate
 * an interrupt.

 * External Connections
 *      ADC1-SOC0 Samples on Channel 0 and ADC1-SOC1 Samples on Channel 1.
 *  - on AM263Px CC E1, with HSEC Dock
 *      - Feed Analog input to ADC1_AIN0 - HSEC PIN 12
 *      - Feed Analog input to ADC1_AIN1 - HSEC PIN 14
 *  - on AM263Px LP,
 *      - Feed Analog input to ADC1_AIN0 - J1/3 PIN  24
 *      - Feed Analog input to ADC1_AIN1 - J1/3 PIN  29
 *
 * Watch Variables
 *  gAdc1Result0[] - array of digital represented voltages on ADC1 channel 0
 *  gAdc1Result1[] - array of digital represented voltages on ADC1 channel 1
 *  count - number of times the OOT flag is generated
*/

/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT     10

/* Global variables and objects */
volatile uint32_t gAdc1Result0[ADC_CONVERSION_COUNT];
volatile uint32_t gAdc1Result1[ADC_CONVERSION_COUNT];
volatile uint32_t getsafecheckStatus[ADC_CONVERSION_COUNT];
uint32_t count = 0;

static HwiP_Object  gAdcHwiObject;

/* Initialising conversion count to 0 */
volatile uint32_t gAdcConversionCount = 0;

/* Function Prototypes */
static void App_adcISR(void *args);

void adc_safety_checker_main(void *args)
{
    uint32_t baseAddr = CONFIG_ADC0_BASE_ADDR;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    int32_t  status;
    /* Initialising a Interrupt parameter */
    HwiP_Params  hwiPrms;
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.priority    = 0;                        /* setting high priority. optional */
    hwiPrms.callback    = &App_adcISR;
    status              = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    DebugP_log("ADC Safety Checker Test Started ...\r\n");

    DebugP_log("ADC 1 Channel 0\t\tADC 1 Channel 1\t\tSafety Check OOT Flag\r\n");

    while(gAdcConversionCount < ADC_CONVERSION_COUNT)
    {

        ADC_forceMultipleSOC(baseAddr, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1));
        while(ADC_getInterruptStatus(baseAddr, ADC_INT_NUMBER1) == false)
        {
            /* Wait for the SOC conversion to complete */
        }

        /* Clear any pending interrupts */
        ADC_clearInterruptStatus(baseAddr, ADC_INT_NUMBER1);

        gAdc1Result0[gAdcConversionCount] = ADC_readResult(CONFIG_ADC0_RESULT_BASE_ADDR, ADC_SOC_NUMBER0);
        gAdc1Result1[gAdcConversionCount] = ADC_readResult(CONFIG_ADC0_RESULT_BASE_ADDR, ADC_SOC_NUMBER1);

        /* Printing results */
            DebugP_log("\t%d\t\t\t%d\t\t\t%d\r\n",
                        gAdc1Result0[gAdcConversionCount],
                        gAdc1Result1[gAdcConversionCount],
                        getsafecheckStatus[gAdcConversionCount]);

            gAdcConversionCount++;
    }

    DebugP_log("ADC OOT Flag Count %d\r\n", count);

    /* Clear and disable interrupt */
    ADC_disableInterrupt(baseAddr, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(baseAddr, ADC_INT_NUMBER1);
    /* Power down the ADC */
    ADC_disableConverter(baseAddr);

    DebugP_log("ADC Safety Checker Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_adcISR(void *args)
{
        if(gAdcConversionCount < ADC_CONVERSION_COUNT)
        {
            /* get the OOT flag status */
            getsafecheckStatus[gAdcConversionCount] = ADC_getSafetyCheckStatus(CSL_CONTROLSS_ADCSAFE_EVENT_AGG_U_BASE, ADC_SAFETY_CHECKER1, ADC_SAFETY_CHECK_OOT_FLG);

            if(getsafecheckStatus[gAdcConversionCount])
            {
                //
                // Indicates that the OOT flag is generated. The tasks can be performed based on this OOT event.
                //
                count++;

            }
        }

        /* Clear pending ADC and safety checker interrupts */
        ADC_clearSafetyCheckStatus(CSL_CONTROLSS_ADCSAFE_EVENT_AGG_U_BASE, ADC_SAFETY_CHECKER1, ADC_SAFETY_CHECK_OOT_FLG);
        ADC_clearSafetyCheckIntStatus(CSL_CONTROLSS_ADCSAFE_EVENT_AGG_U_BASE);

}

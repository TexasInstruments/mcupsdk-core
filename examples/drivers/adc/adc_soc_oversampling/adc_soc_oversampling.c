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
#include <drivers/adc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/*
 * Example Description :
 *      This example shows oversamping on a given ADC channel, periodically
 * triggered by EPWM. ADC1 Channel 2 is sampled by SOC (2-5) and are triggered
 * by EPWMSOCA along with SOC 0 (sampling channel 0) and SOC 1 (sampling Channel
 * 1).
 *
 * SOC Configurations :
 * - SOC(0-5) are triggered by EPWMSOCA. Sample window is set at 16.
 * - SOC 0 samples on channel 0
 * - SOC 1 samples on channel 1
 * - SOC (2-5) sample on channel 2
 *
 * INT Configurations
 * - ADC1INT1 is set for EOC/SOC5
 *
 * ISR Configurations
 * - INTXbar0 is set for ADC1INT1.
 * - App_adcISR services this interrupt.
 *      - reads the results for the SOC(0-5),
 *      - freezes EPWM counter if required conversions are complete.
 *
 * External Connections
 * - AM263X-CC
 *      - Feed Analog Voltages on ADC 1 Channel 0 - HSEC PIN 18
 *      - Feed Analog Voltages on ADC 1 Channel 1 - HSEC PIN 20
 *      - Feed Analog Voltages on ADC 1 Channel 2 - HSEC PIN 21
 *  - AM263X-LP
 *      - Feed Analog Voltages on ADC 1 Channel 0 - J1/3 PIN 24
 *      - Feed Analog Voltages on ADC 1 Channel 1 - J1/3 PIN 29
 *      - Feed Analog Voltages on ADC 1 Channel 2 - J5/7 PIN 67
 *
 * Watch Variables
 * gAdc1Channel0Result - array of digital represented voltages on ADC1 channel 0
 * gAdc1Channel1Result - array of digital represented voltages on ADC1 channel 1
 * gAdc1Channel2Result - array of digital represented voltages on ADC1 channel 2
 *
 */

uint32_t gAdc1BaseAddr = CONFIG_ADC1_BASE_ADDR;

uint32_t gAdc1ResultBaseAddr = CONFIG_ADC1_RESULT_BASE_ADDR;

#define ADC_CONVERSION_COUNT (1024)

volatile uint16_t gAdcConversionCount = 0;
volatile uint16_t gAdcResultIndex = 0;

volatile uint16_t gAdc1Channel0Result[ADC_CONVERSION_COUNT];
volatile uint16_t gAdc1Channel1Result[ADC_CONVERSION_COUNT];
volatile uint16_t gAdc1Channel2Result[ADC_CONVERSION_COUNT];


static HwiP_Object gAdcHwiObject;

static void App_adcISR(void *args);

void adc_soc_oversampling_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC SOC Oversampling Test Started ...\r\n");

    /* registering ISRs */
    int32_t     status;
    HwiP_Params hwiPrms;

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_adcISR;
    status              = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Clearing Interrupt flags if any*/
    if(true == ADC_getInterruptStatus(gAdc1BaseAddr, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptStatus(gAdc1BaseAddr, ADC_INT_NUMBER1);
    }

    EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_UP);

    while(gAdcConversionCount < ADC_CONVERSION_COUNT);

    /* Printing few results */
    DebugP_log("ADC 1 Channel 0\tChannel 1\tChannel 2\r\n");
    for(int iter = 0; iter < ADC_CONVERSION_COUNT; iter+= 100)
    {
        DebugP_log("\t%d\t\t%d\t\t%d\r\n",
                    gAdc1Channel0Result[iter],
                    gAdc1Channel1Result[iter],
                    gAdc1Channel2Result[iter]);
    }

    DebugP_log("ADC SOC Oversampling Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_adcISR(void *args)
{
    if(gAdcConversionCount < ADC_CONVERSION_COUNT)
    {
    /* Read Results*/
    gAdc1Channel0Result[gAdcResultIndex] = ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER0);
    gAdc1Channel1Result[gAdcResultIndex] = ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER1);

    gAdc1Channel2Result[gAdcResultIndex] = ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER2);
    gAdc1Channel2Result[gAdcResultIndex] += ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER3);
    gAdc1Channel2Result[gAdcResultIndex] += ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER4);
    gAdc1Channel2Result[gAdcResultIndex] += ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER5);

    /* averaging the results on SOC (2-5)*/
    gAdc1Channel2Result[gAdcResultIndex] = (gAdc1Channel2Result[gAdcResultIndex])>>2;

        gAdcConversionCount++;
        gAdcResultIndex++;
    }
    else
    {
        /* Stopping time base counter of EPWM*/
        EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_STOP_FREEZE);
    }

    /* Clearing Interrupt Status*/
    ADC_clearInterruptStatus(gAdc1BaseAddr, ADC_INT_NUMBER1);
    if(true == ADC_getInterruptOverflowStatus(gAdc1BaseAddr, ADC_INT_NUMBER1))
    {
        // ADC_clearInterruptStatus(gAdc1BaseAddr, ADC_INT_NUMBER1);
        ADC_clearInterruptOverflowStatus(gAdc1BaseAddr, ADC_INT_NUMBER1);
    }
}

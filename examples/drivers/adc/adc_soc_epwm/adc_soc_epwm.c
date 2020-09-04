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
 *      This examples demonstrates periodical triggering of conversion on ADC1
 * by EPWM0
 *
 * SOC Configurations :
 * - SOC 0 trigger is set by EPWM0SOCA
 * - Samples on ADC1 Channel 0
 *
 * Interrupt Configurations :
 * - ADC1INT1 source is set to EOC/SOC0
 * - INTXBAR0 is set to ADC1INT1.
 *
 * ISR App_adcISR
 * - reads the SOC0 result.
 *
 * External Connections :
 * - CC:
 *     feed analog input on ADC 1 Channel 0 - HSEC connecter pin - 18
 * - LP:
 *     feed analog input on ADC 1 Channel 0 - J1/J3 Pin - 24
 *
 * Watch Variables :
 * - gAdc1Result0 : the array holds the sampled values of the ADC 1 Channel 0
 */


/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT     1024

/* Global variables and objects */
volatile uint32_t gAdc1Result0[ADC_CONVERSION_COUNT];
volatile uint32_t gIndex = 0;

uint32_t gAdc1baseAddr = CONFIG_ADC1_BASE_ADDR;
uint32_t gAdc1resultBaseAddr = CONFIG_ADC1_RESULT_BASE_ADDR;

static HwiP_Object  gAdcHwiObject;

/* Variable to store the count of completed conversions
Initialising conversion count to 0 */
volatile uint32_t gAdcConversionCount = 0;

static void App_adcISR(void *args);

void adc_soc_epwm_main(void *args)
{


    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC Soc EPWM Test Started\r\n");

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

    /* Starting the EPWM-TB Counter*/
    EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_UP);

    /* Wait until all the conversions are done.*/
    while (gAdcConversionCount < ADC_CONVERSION_COUNT);

    DebugP_log("ADC1 SOC0 results\r\n");
    for(int iter = 0; iter < gIndex; iter+= 100)
    {
        DebugP_log("\t%d\r\n",gAdc1Result0[iter]);
    }

    DebugP_log("ADC Soc EPWM Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_adcISR(void *args)
{


    if (gAdcConversionCount < ADC_CONVERSION_COUNT)
    {
        gAdc1Result0[gIndex] = ADC_readResult(gAdc1resultBaseAddr, ADC_SOC_NUMBER0);
        gIndex++;
        gAdcConversionCount++;
    }
    else
    {
        /* stopping the TBCounter to halt PWM*/
        EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_STOP_FREEZE);
    }
    ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
    if(true == ADC_getInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
        ADC_clearInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
    }
}


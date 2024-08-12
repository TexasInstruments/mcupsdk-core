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
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include <drivers/adc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/*
 * Trigger repeater module
 *      The trigger repeater module allows to achieve an oversampling,
 * undersampling or to apply a trigger delay. The repeater modules can select
 * any of the regular ADC triggers that are selectable by ADCSOCxCTL.TRIGGER
 * register and generate a number of repeated pulses as configured in repeater
 * module. The repeater module can apply four types of trigger modifications:
 * - Oversample mode - allows multiple back-to-back samples from a single
 *  trigger pulse.
 * - Undersample mode - scale down the trigger frequency for one or more SOCs.
 * - Phase delay - delay the initial trigger by a specified number of SYSCLK.
 * - Re-trigger Spread - the additional time between two samples.
 *
 * Example Description :
 * This example configures ADC for oversampling using trigger repeater block.
 * The ePWM0 is configured to periodically trigger the trigger repeater module
 * on ADC1 for conversion of inputs on ADC1_AIN0. The trigger repeater module
 * is configured to generate 4 repeated pulses. Post-processing block will
 * take the repeated pulses, increment PPB conversion count and generates
 * an oversampling interrupt (OSINT1). The accumulated samples will be stored
 * in PPB SUM register.
 *
 * SOC Configurations :
 * - Trigger repeater module on ADC1 is triggered by EPWMSOCA.
 * - SOC0 will be triggered by trigger repeater and Sample window is set at 16.
 * - SOC0 samples on channel 0 and trigger source is set as repeated trigger.
 *
 * INT Configurations
 * - EPWM will generate an interrupt to read the ADC OSINT flag
 *
 * ISR Configurations
 * - INTXbar0 is set for EPWM0INT1.
 * - App_epwmISR services this interrupt.
 *      - reads ADC OSINT flag to check if oversampling is done.
 *
 * External Connections
 * - AM263PX-CC E1
 *      - Feed Analog Voltages on ADC 1 Channel 0 - HSEC PIN 12
 *  - AM263PX-LP or AM261X-LP
 *      - Feed Analog Voltages on ADC 1 Channel 0 - J1/3 PIN 24
 *
 */

/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT     10

/* Global variables and objects */
volatile uint32_t gAdc1Result0[ADC_CONVERSION_COUNT];
volatile uint32_t gIndex = 0;

uint32_t gAdc1baseAddr = CONFIG_ADC1_BASE_ADDR;

static HwiP_Object  gAdcHwiObject;

/* Variable to store the epwm ISR count
Initialising epwm ISR count and ppb count to 0 */

volatile uint32_t epwmISRCount = 0;
volatile uint32_t ppbCount = 0;

/* Function Prototypes */
static void App_epwmISR(void *args);

void adc_trigger_repeater_oversampling_main(void *args)
{

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC Trigger Repeater Oversampling Test Started...\r\n");

    int32_t  status;
    /* Initialising a Interrupt parameter */
    HwiP_Params  hwiPrms;
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.priority    = 0;                        /* setting high priority. optional */
    hwiPrms.callback    = &App_epwmISR;
    status              = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Starting the EPWM-TB Counter*/
    EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_UP);
    /* Clear any pending interrupts */
    ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
    EPWM_clearEventTriggerInterruptFlag(CONFIG_EPWM0_BASE_ADDR);

    while (epwmISRCount < ADC_CONVERSION_COUNT);

    DebugP_log("ADC Trigger Repeater Oversampling results:\r\n");
    if(epwmISRCount == ppbCount)
    {
        DebugP_log("Trigger Repeater Oversampling Test Passed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");

    }
    else
    {
        DebugP_log("Trigger Repeater Oversampling Test Failed!!\r\n");
        DebugP_log("Some tests have Failed!!\r\n");

    }

    Board_driversClose();
    Drivers_close();
}

static void App_epwmISR(void *args)
{

    if (epwmISRCount < ADC_CONVERSION_COUNT)
    {
        /* Check if the ADC OSINT flag is high */
        if(ADC_getIntResultStatus(gAdc1baseAddr, ADC_INT_NUMBER1))
        {
            ppbCount++;
        }

        epwmISRCount++;
    }
    /* Clear any pending interrupts */
    EPWM_clearEventTriggerInterruptFlag(CONFIG_EPWM0_BASE_ADDR);
    ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
    if(true == ADC_getInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
        ADC_clearInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
    }

}

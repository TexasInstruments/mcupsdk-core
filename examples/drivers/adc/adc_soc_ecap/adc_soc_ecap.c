/*
 *  Copyright (C) 2022-2023 Texas Instruments Incorporated
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
 * Example Description
 *      This example demostrates how the ECAP and ADC to be configured so that the
 * ADC SOC can be triggered from the ECAP Events. The ECAP can trigger the ADC
 * SOC Conversions in its both Capture mode and APWM mode.
 *      - In ECAP Capture Mode, the CAPEVTx [x= 1 to 4] can be used and
 *      - In ECAP APWM Mode, The counter match to Compare or Period can be used.
 *      The example configures an ECAP (APWM_ECAP) to be in the APWM mode and generate an PWM
 * waveform, and another ECAP (CAPTURE_ECAP) to be in Capture mode to recieve the PWM generated
 * from the APWM_ECAP, via GPIO and InputXbar. Also, 2 ADCs ADC_APWM_TRIG, ADC_CAP_TRIG
 * are configured to recieve triggers from the APWM_ECAP and CAPTURE_ECAP respectively.
 * Both ADCs generate Interrupt at the EOCs and trigger respective App_adcISR.
 *      The ISRs are used for result reading and validation purpose of the example via
 * incrementing a ISR count variable.
 *
 * ECAP Configurations:
 * 1. APWM_ECAP :
 *      - generate a PWM wave of 5 KHz with 50% duty cycle.
 *      - generate an ADC SOC trigger at its compare match (i.e, 5KHz triggers)
 *
 * 2. CAPTURE_ECAP :
 *      - recieve 2 edges, 1 rising then 1 falling (CAPEVT1,2)
 *      - generate an ADC SOC trigger at CAPEVT2
 *
 * ADC Configurations :
 *  - ADC_APWM_TRIG (or ADC_CAP_TRIG) :
 *      - SOC 0 is configured to recieve trigger from the APWM_ECAP (or CAPTURE_ECAP)
 *      - INT 1 is generated at the EOC/SOC0
 *      - Prescaled at 3, with Sample and Hold window of 18, configuring for 4MSPS on ADC.
 *
 * ISRs (App_capTrigAdcISR and App_apwmTrigAdcISR)
 * - ADC INT 1 from ADC_APWM_TRIG (or ADC_CAP_TRIG) is configured via INTxBar to trigger
 *      App_apwmTrigAdcISR (or App_capTrigAdcISR).
 * - Each ISR reads the respective SOC result, clears the ADC INT flags and Increments a counter
 *      for validation.
 *
 * The below watch variables can be used to view ADC conversion results.
 *
 * Watch Variables
 * - gApwmTrigAdcResult[] : Digital representation of the voltage sample on pin AIN2 of ADC_APWM_TRIG
 * - gCapTrigAdcResult[] : Digital representation of the voltage sample on pin AIN0 of ADC_CAP_TRIG
 *
 * External Connections :
 * AM263x-CC E2 or AM263Px-CC E2 :
 *      Feed Analog voltages on ADC_APWM_TRIG (ADC0)
 *      - ADC0_AIN2, i.e., HSEC PIN  15
 *      Feed Analog voltages on ADC_CAP_TRIG (ADC1)
 *      - ADC1_AIN0, i.e., HSEC PIN  12
 * AM263x-LP :
 *      Feed Analog voltages on ADC_APWM_TRIG (ADC0)
 *      - ADC0_AIN2, i.e., J5/7 PIN  66
 *      Feed Analog voltages on ADC_CAP_TRIG (ADC1)
 *      - ADC1_AIN0, i.e., J1/3 PIN  24
 */


/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT     (256U)

/* Global variables and objects */
/* Variable to store conversion results from all 4 pins  */
uint16_t gApwmTrigAdcResult[ADC_CONVERSION_COUNT];
uint16_t gCapTrigAdcResult[ADC_CONVERSION_COUNT];

/* Variable to count ISR and index result to be stored to. */
volatile uint32_t gApwmTrigAdcIsrCount = 0;
volatile uint32_t gCapTrigAdcIsrCount = 0;

/* Variables to store the base addresses */
uint32_t gApwmTrigAdcBaseAddr = ADC_APWM_TRIG_BASE_ADDR;
uint32_t gCapTrigAdcBaseAddr = ADC_CAP_TRIG_BASE_ADDR;
uint32_t gApwmTrigAdcResultBaseAddr = ADC_APWM_TRIG_RESULT_BASE_ADDR;
uint32_t gCapTrigAdcResultBaseAddr = ADC_CAP_TRIG_RESULT_BASE_ADDR;

uint32_t gApwmEcapBaseAddr = APWM_ECAP_BASE_ADDR;
uint32_t gCaptureEcapBaseAddr = CAPTURE_ECAP_BASE_ADDR;


/* HwiP Objects for ISR registration */
static HwiP_Object  gApwmTrigAdcHwiObject;
static HwiP_Object  gCapTrigAdcHwiObject;


/* Function Prototypes */
static void App_apwmTrigAdcISR(void *args);
static void App_capTrigAdcISR(void *args);

void adc_soc_ecap_main(void *args)
{

    /* variable to iterate ADC_CONVERSION_COUNT */

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();


    DebugP_log("ADC Triggered by ECAP Test Started ...\r\n");
    int32_t  status;
    /* Initialising a Interrupt parameter */
    HwiP_Params  hwiPrms;
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.priority    = 0;                        /* setting high priority. optional */
    hwiPrms.callback    = &App_apwmTrigAdcISR;
    status              = HwiP_construct(&gApwmTrigAdcHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms.priority    = 0;                        /* setting high priority. optional */
    hwiPrms.callback    = &App_capTrigAdcISR;
    status              = HwiP_construct(&gCapTrigAdcHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    ADC_clearInterruptStatus(gApwmTrigAdcBaseAddr, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(gCapTrigAdcBaseAddr, ADC_INT_NUMBER1);

    /* Wait until required number of ADC conversions are completed */
    while( gCapTrigAdcIsrCount < ADC_CONVERSION_COUNT );

    /* all the conversions triggered from the CAPTURE ECAP are complete. */
    if((gApwmTrigAdcIsrCount < gCapTrigAdcIsrCount) && ((gApwmTrigAdcIsrCount - gCapTrigAdcIsrCount) <= 1))
    {
        /* the ISR counts for the ADC_APWM_TRIG should be same or 1 more than the ADC_CAPTURE_TRIG considering the timing*/
        DebugP_log("ADC Triggered by ECAP test failed. One or more APWM Triggered conversions are missed.\r\n");
        DebugP_log("Some tests have Failed!!\r\n");
    }
    else
    {
        DebugP_log("ADC Triggered by ECAP Test Passed\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

static void App_apwmTrigAdcISR(void *args)
{

    ADC_clearInterruptStatus(gApwmTrigAdcBaseAddr, ADC_INT_NUMBER1);
    if(true == ADC_getInterruptOverflowStatus(gApwmTrigAdcBaseAddr, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(gApwmTrigAdcBaseAddr, ADC_INT_NUMBER1);
    }
    if (gApwmTrigAdcIsrCount >= ADC_CONVERSION_COUNT)
    {
        /* stopping the APWM_ECAP Counter to halt PWM*/
        ECAP_stopCounter(gApwmEcapBaseAddr);
    }
    else
    {
        gApwmTrigAdcResult[gApwmTrigAdcIsrCount] = ADC_readResult(gApwmTrigAdcResultBaseAddr, 0);
    }
    gApwmTrigAdcIsrCount++;
}

static void App_capTrigAdcISR(void *args)
{

    ADC_clearInterruptStatus(gCapTrigAdcBaseAddr, ADC_INT_NUMBER1);
    if(true == ADC_getInterruptOverflowStatus(gCapTrigAdcBaseAddr, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(gCapTrigAdcBaseAddr, ADC_INT_NUMBER1);
    }
    if (gCapTrigAdcIsrCount >= ADC_CONVERSION_COUNT)
    {
        /* stopping the CAPTURE_ECAP counter*/
        ECAP_stopCounter(gCaptureEcapBaseAddr);
    }
    else
    {
        gCapTrigAdcResult[gCapTrigAdcIsrCount] = ADC_readResult(gCapTrigAdcResultBaseAddr, 0);
    }
    gCapTrigAdcIsrCount++;

}

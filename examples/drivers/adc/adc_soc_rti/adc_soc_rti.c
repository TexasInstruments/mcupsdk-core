/*
 *  Copyright (C) 2022-2024 Texas Instruments Incorporated
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
 *      This example showcases the RTI triggering ADC Start of Conversions (SOC).
 * RTI in the example is configured for the 5uS period. The ADC SOC is configured 
 * to triggered from the RTI, trigger an ADC Interrupt. 
 * 
 * External Connections
 *      External Connections are arbitrary, for conversion and the SOC configuration,
 * the SOC is configured for the ADC 0 Channel 2.
 *  - on AM263x CC E2, AM263Px CC E2, with HSEC Dock 
 *      - Feed Analog input to ADC0_AIN2 - HSEC PIN 15  
 *  - on AM263x LP E2, AM263Px LP
 *      - Feed Analog Input to the ADC0_AIN2 - J7 Pin 66
 * 
 * Watch Variables 
 *  gAdcIsrCount - number of ADC conversion Interrupts.
 *  gRtiIsrCount - number of RTI triggered Interrupts.
 *  gAdcResults[] - array holding the results converted by RTI Triggers
 */


/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT     (256U)
#define RTI_ISR_COUNT            (256U)

/* Variables to keep track of ISR counts */
volatile uint32_t gAdcIsrCount = 0;
volatile uint32_t gRtiIsrCount = 0;

uint32_t gAdcBaseAddr = CONFIG_ADC0_BASE_ADDR;
uint32_t gAdcResultBaseAddr = CONFIG_ADC0_RESULT_BASE_ADDR;

/* Array to store RTI triggered ADC Conversions */
uint16_t gAdcResults[ADC_CONVERSION_COUNT] = {0};

/* HwiP Objects for ISR registration */
static HwiP_Object  gAdcHwiObject;


/* Function Prototypes */
static void App_adcISR(void *args);

void adc_soc_rti_main(void *args)
{

    /* variable to iterate ADC_CONVERSION_COUNT */

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();


    DebugP_log("ADC Triggered by RTI Test Started ...\r\n");
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

    ADC_clearInterruptStatus(gAdcBaseAddr, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(gAdcBaseAddr, ADC_INT_NUMBER1);

    RTI_counterEnable(CONFIG_RTI0_BASE_ADDR, CONFIG_RTI0_COMP0_SRC);
    

    /* waiting till all the RTI Interrupts are Serviced */
    while(gRtiIsrCount < RTI_ISR_COUNT);
    

    /* number of RTI ISR should be equal to, or one more than, the number of ADC Interrupts, considering the */
    if((gRtiIsrCount < gAdcIsrCount) || ((gRtiIsrCount - gAdcIsrCount) > 1))
    {
        /* the ISR counts for the RTI should be same or 1 more than those of ADC considering the timing*/
        DebugP_log("ADC Triggered by RTI test failed. One or more RTI Triggered conversions are missed.\r\n");
        DebugP_log("Some tests have Failed!!\r\n");
    }
    else
    {
        DebugP_log("ADC Triggered by RTI Test Passed\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

static void App_adcISR(void *args)
{   
    gAdcResults[gAdcIsrCount++] = ADC_readResult(gAdcResultBaseAddr, ADC_SOC_NUMBER0);

    if (gAdcIsrCount >= ADC_CONVERSION_COUNT)
    {
        /* Disabling the ADC Interrupt  */
        ADC_disableInterrupt(gAdcBaseAddr, ADC_INT_NUMBER1);
    }

    /* Clearing Interrupt and Overflow Interrupt flag if any. */
    ADC_clearInterruptStatus(gAdcBaseAddr, ADC_INT_NUMBER1);
    if(true == ADC_getInterruptOverflowStatus(gAdcBaseAddr, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(gAdcBaseAddr, ADC_INT_NUMBER1);
    }
}

/* The App_rtiISR is defined in the Syscfg */
void App_rtiISR(void *args)
{

    gRtiIsrCount++;
    if(gRtiIsrCount >= RTI_ISR_COUNT)
    {   
        /* Disabling the RTI counter. 
        Essentially freezing the RTI not to generate more interrupts */
        RTI_counterDisable(CONFIG_RTI0_BASE_ADDR, CONFIG_RTI0_COMP0_SRC);
    }

}

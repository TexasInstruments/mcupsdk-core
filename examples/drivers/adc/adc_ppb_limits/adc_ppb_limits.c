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
 *      Post Processing Blocks of the ADC are featured with limit checking
 * on the PPBResult registers. the following are limits that each PPB can
 * check,
 *      1. High Limit
 *      2. Low Limit
 *      3. Zero Crossing.
 *      PPB can generate Event flags, for selected events. Also, each PPB
 * can generate an EVTINT based on which event has occured. ADCxEVTINT is
 * a logical OR of all such PPBxEVTINT in given ADC.
 *
 *      This example is a demonstration of such feature and uses the loopback
 * from DAC to generate the required analog wave form (ramp wave is choosen for
 * this example). Once the High and Low values are touched, the PPBs generate
 * the EVTINT and ISR will check the events of PPB0, PPB1 (both configured for
 * SOC0) and determine which limit has occured.
 *
 * Configurations :
 * 1. SOC configuration :
 *      - SOC0 is configured to sample on AIN0
 *      - Sample window of 20 is selected (arbitrary)
 *      - Software only trigger is selected.
 * 2. PPB Configurations :
 *      - SOC0 is selected for PPB0 and PPB1
 *      - PPB1 is set for Higher Limit Checking i.e., PPB Event is set for TripHigh
 *      - PPB1EVTINT is set for TripHigh
 *      - PPB2 is set for Lower Limit Checking i.e., PPB Event is set for TripLow
 *      - PPB2EVTINT is set for TripLow
 *      Note : PPBEVTINT is a logical OR for all the PPBxEVTINT, and is source for ISR
 * ISR Configuration
 * - App_adc_PPB_ISR is set for INTXbar0, selected for ADC1EVTINT
 * - ISR checks which event has occured and load the corresponding PPB result to respective arrays.
 * - ISR Clears the events for further generated event identification
 *
 * External Connections
 *
 * 1. Am263x-CC
 *      Loop DAC output back to ADC1 Channel 0
 *      Connect HSEC Pin 9 to HSEC Pin 18
 *
 * 2. Am263x-LP
 *      Loop DAC output back to ADC1 Channel 0
 *      Connect J1/J3 Pin 30 to J1/J3 Pin 24
 *
 * Watch Variables
 * - gAdcHighLimitOutputs[] - holds the High limit trip values
 * - gAdcLowLimitOutputs[] - holds the Low limit trip values
 */



/* Number of ADC conversions required */
#define NUMBER_OF_PPB_EVENTS     256

volatile uint32_t gAdcHighLimitOutputs[NUMBER_OF_PPB_EVENTS];
volatile uint32_t gAdcLowLimitOutputs[NUMBER_OF_PPB_EVENTS];

uint32_t gAdc1baseAddr = CONFIG_ADC1_BASE_ADDR;
uint32_t gAdc1ResultBaseAddr = CONFIG_ADC1_RESULT_BASE_ADDR;
uint32_t gDacbaseAddr  = CONFIG_DAC_BASE_ADDR;

volatile uint32_t gPpbEventsCount = 0;
volatile uint32_t gHighLimitIndex = 0;
volatile uint32_t gLowLimitIndex = 0;


static HwiP_Object  gAdcHwiObject1;

void App_adc_PPB_ISR(void *args);

void adc_ppb_limits_main(void *args)
{
    /* Refer to Syscfg for SOC, PPB, EPWM, Interrupt and INT xbar Configurations.*/
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC PPB Limits Test Started\r\n");

    /* Registring ISRs for PPB EVT INT.*/
    int32_t  status;
    HwiP_Params  hwiPrms;

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_adc_PPB_ISR;
    status              = HwiP_construct(&gAdcHwiObject1, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Setting the limits. this can also be done through Sysconfig.*/
    int32_t tripHighLimit = 3000;
    int32_t tripLowLimit = 1000;
    /* 3000 in ADC result corresponds to analog voltage of 2.34 V */
    /* 1000 in ADC result corresponds to analog voltage of 0.78 V */

    int32_t tripLowLimit_dummy = 0;
    int32_t tripHighLimit_dummy = 0;

    ADC_setPPBTripLimits(gAdc1baseAddr, ADC_PPB_NUMBER1, tripHighLimit, tripLowLimit_dummy);
    ADC_setPPBTripLimits(gAdc1baseAddr, ADC_PPB_NUMBER2, tripHighLimit_dummy, tripLowLimit);

    /* Clearing events if any*/
    ADC_clearPPBEventStatus(gAdc1baseAddr, ADC_PPB_NUMBER1, ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO);
    ADC_clearPPBEventStatus(gAdc1baseAddr, ADC_PPB_NUMBER2, ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO);

    volatile uint16_t DacShadowValue = 2045 ;
    /* Setting the DAC output to be in between 3.3V and 0V*/
    DAC_setShadowValue(gDacbaseAddr, DacShadowValue);

    int stepValue = 100;
    while (gPpbEventsCount < NUMBER_OF_PPB_EVENTS)
    {
        if(DacShadowValue >= 3095)
        {
            /* Reaching max limit*/
            DacShadowValue = 100;
        }
        else
        {
            /* Ramp wave is in between*/
            DacShadowValue += stepValue;
        }
        DAC_setShadowValue(gDacbaseAddr, DacShadowValue);
        ClockP_usleep(1);

        ADC_forceSOC(gAdc1baseAddr, ADC_SOC_NUMBER0);
    }

    DebugP_log("ADC trip high events occurred for Conversion Values : \r\n");
    for(int iter = 0; iter <= gHighLimitIndex; iter += 10)
    {
        DebugP_log("\t%d\r\n",gAdcHighLimitOutputs[iter]);
    }

    DebugP_log("ADC trip low events occurred for Conversion Values : \r\n");
    for(int iter = 0; iter <= gLowLimitIndex; iter += 10)
    {
        DebugP_log("\t%d\r\n",gAdcLowLimitOutputs[iter]);
    }

    DebugP_log("ADC PPB Limits Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}
/* ISR for ADC EVTINT */
void App_adc_PPB_ISR(void *args)
{
    /* ISR is invoked when there is a PPB event */

    /* Check if PPB1 Trip High Event triggered Interrupt */
    if((ADC_getPPBEventStatus(gAdc1baseAddr, ADC_PPB_NUMBER1)&ADC_EVT_TRIPHI) == ADC_EVT_TRIPHI)
    {
        /* Reading the PPB1 result */
        gAdcHighLimitOutputs[gHighLimitIndex] = ADC_readPPBResult(gAdc1ResultBaseAddr, ADC_PPB_NUMBER1);
        gHighLimitIndex++;
        /* Clearing the PPB1 Event status */
        ADC_clearPPBEventStatus(gAdc1baseAddr, ADC_PPB_NUMBER1, ADC_EVT_TRIPHI);
    }
    /* Check if PPB2 Trip Low Event triggered Interrupt */
    if((ADC_getPPBEventStatus(gAdc1baseAddr, ADC_PPB_NUMBER2)&ADC_EVT_TRIPLO) == ADC_EVT_TRIPLO)
    {
        /* Reading the PPB2 result */
        gAdcLowLimitOutputs[gLowLimitIndex] = ADC_readPPBResult(gAdc1ResultBaseAddr, ADC_PPB_NUMBER2);
        gLowLimitIndex++;
        /* Clearing the PPB2 Event status */
        ADC_clearPPBEventStatus(gAdc1baseAddr, ADC_PPB_NUMBER2, ADC_EVT_TRIPLO);
    }
    gPpbEventsCount++;
}

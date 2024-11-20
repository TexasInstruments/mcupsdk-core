/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example uses the ADC Post Processing Block to trip a ePWM in case of
 * out of bound input.
 *
 * ADCINT1 is configured to periodically trigger the ADC. Initially ADC is
 * triggered through software.
 *
 * Limit detection Post processing block is configured and if ADC results are
 * outside the defined range, the PPB will generate an ADCxEVTy event. This
 * signal is configured as trip source for EPWM0, 1 and 2.
 *
 * This example showcases one-shot, cycle-by-cycle and direct tripping of PWMs.
 *  - EPWM0 is configured for One Shot trip.
 *  - EPWM1 is configured for Cycle by Cycle trip.
 *  - EPWM2 is configured for Direct trip via Digital compare submodule.
 *
 * External Connections
 * ADC0_AIN2 pin should be connected to DAC output pin.
 * EPWM0A, EPWM0B, EPWM1A, EPWM1B, EPWM2A, EPWM2B can be connected to an
 * oscilloscope to validate tripping.
 *
 * Watch Variables
 * adc0Result2 - Digital representation of the voltage on pin ADC0_AIN2
 *
 */

/* Defines */
#define RESULTS_BUFFER_SIZE    (512U)

/* Global variables and objects */
volatile uint16_t indexA = 0;     /* Index into result buffer */
volatile uint16_t bufferFull;    /* Flag to indicate buffer is full */
uint16_t adcResults[RESULTS_BUFFER_SIZE];    /* ADC result buffer */
uint32_t gAdcBaseAddr;
uint32_t gAdcResultBaseAddr;
static HwiP_Object gAdcHwiObject0, gAdcHwiObject1;
volatile uint32_t adcOstEvtFlag = 0, adcCbcEvtFlag = 0, adcDirectEvtFlag = 0;
/* Function prototypes */
static void App_adcEocIntrISR(void *handle);
static void App_adcEvtIntrISR(void *handle);

void adc_ppb_epwm_trip_main(void *args)
{
    gAdcBaseAddr = CONFIG_ADC0_BASE_ADDR;
    gAdcResultBaseAddr = CONFIG_ADC0_RESULT_BASE_ADDR;
    HwiP_Params hwiPrms0, hwiPrms1;
    int32_t status;
    uint32_t dacVal;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC PPB ePWM Trip Test Started ...\r\n");

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms0);
    hwiPrms0.intNum = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms0.callback = &App_adcEocIntrISR;
    hwiPrms0.priority = 1U;
    status = HwiP_construct(&gAdcHwiObject0, &hwiPrms0);
    DebugP_assert(SystemP_SUCCESS == status);

    HwiP_Params_init(&hwiPrms1);
    hwiPrms1.intNum = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms1.callback = &App_adcEvtIntrISR;
    hwiPrms1.priority = 2U;
    status = HwiP_construct(&gAdcHwiObject1, &hwiPrms1);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enabling the EPWM Trip Out which may be routed to GPIO via Output xbar */
    EPWM_enableTripZoneOutput(CONFIG_EPWM2_BASE_ADDR,EPWM_TZ_SELECT_TRIPOUT_DCAEVT1);

    /* Clear any spurious flags */
    EPWM_clearTripZoneFlag(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_FLAG_DCAEVT1);
    EPWM_clearOneShotTripZoneFlag(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_OST_FLAG_DCAEVT1);
    EPWM_clearTripZoneFlag(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_FLAG_DCAEVT2);
    EPWM_clearCycleByCycleTripZoneFlag(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_CBC_FLAG_DCAEVT2);
    EPWM_clearTripZoneFlag(CONFIG_EPWM2_BASE_ADDR, (EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCBEVT2));
    ADC_clearInterruptStatus(gAdcBaseAddr, ADC_INT_NUMBER1);
    ADC_clearPPBEventStatus(CONFIG_ADC0_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO));
    /* Trigger ADC once through software */
    ADC_forceSOC(gAdcBaseAddr, ADC_SOC_NUMBER0);
    dacVal = 0;
    while(true)
    {
        /* Set DAC Output */
        DAC_setShadowValue(CONFIG_DAC0_BASE_ADDR, dacVal);
        /* Wait for a few us */
        ClockP_usleep(10);
        /* Increment the DAC value */
        dacVal += 10;
        /* If DAC output reaches maximum value, bring it to 0 */
        if(dacVal > 4095)
        {
            dacVal = 0;
        }
        /* Exit the loop once all trips occur */
        if(adcOstEvtFlag == 1 && adcCbcEvtFlag == 1 && adcDirectEvtFlag == 1)
        {
            break;
        }

    }

    DebugP_log("ADC PPB ePWM Trip Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_adcEocIntrISR(void *handle)
{
    /* Add the latest result to the buffer */
    adcResults[indexA++] = ADC_readResult(gAdcResultBaseAddr, ADC_SOC_NUMBER0);

    /* Set the bufferFull flag if the buffer is full */
    if(RESULTS_BUFFER_SIZE <= indexA)
    {
        indexA = 0;
    }

    return;
}

static void App_adcEvtIntrISR(void *handle)
{
    uint32_t ostFlag = 0, cbcFlag = 0, directFlag = 0;

    ostFlag = EPWM_getOneShotTripZoneFlagStatus(CONFIG_EPWM0_BASE_ADDR);
    cbcFlag = EPWM_getCycleByCycleTripZoneFlagStatus(CONFIG_EPWM1_BASE_ADDR);
    directFlag = EPWM_getTripZoneFlagStatus(CONFIG_EPWM2_BASE_ADDR);

    if( (ostFlag & EPWM_TZ_OST_FLAG_DCAEVT1) != 0)
    {
        adcOstEvtFlag = 1;
    }
    if( (cbcFlag & EPWM_TZ_CBC_FLAG_DCAEVT2) != 0)
    {
        adcCbcEvtFlag = 1;
    }
    if( (directFlag & (EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCBEVT2)) == (EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCBEVT2))
    {
        adcDirectEvtFlag = 1;
    }

    ADC_clearPPBEventStatus(CONFIG_ADC0_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO));
    return;
}

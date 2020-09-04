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
#include <drivers/adc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * Example Description:
 *
 * This example showcases the feature "High Priority SOC configuration"
 * High Priority SOC Configuration :
 *          when the SOC are configured in high priority (SOC0 through
 *      SOCx) the other SOC are configured in the Round Robin mode. the high
 *      priority SOC, when triggered, pre-empts the Round robin priority
 *
 * Configurations :
 *      Triggers from EPWM0 are configured through Syscfg. the counter is
 *  configured in Stop and Freeze mode. the example has to change the Counter
 *  mode to get the counter running and EPWM0 can generate triggers to ADC.
 *  NOTE :
 *          PPB Delay Time Stamp cannot be used if the SOC trigger is software. hence, the example uses EPWM0 to trigger
 *      SOC conversions.
 *          This example intends to showcase the Priority modes in the SOC. the input channels can be configured through
 *      Syscfg as per need.
 *
 * ADC0 :   (done through Syscfg)
 *          SOC 0-3 are in Round Robin (no high priority selected)
 *          all SOC are triggered by EPWM0SOCA signal.
 *          Conversion sequence : SOC 0 --> SOC 1 --> SOC 2 --> SOC 3
 *
 * ADC1 :   (done through Syscfg)
 *          SOC 0-3 are in Round Robin (no high priority selected)
 *          SOC 1,2 are triggered by EPWM0SOCA signal.
 *          SOC 0,3 are triggered by EPWM0SOCB signal.
 *          Conversion sequence : SOC 1 --> SOC 2 --> SOC 3 --> SOC 0
 *
 * ADC2 :   (done through Syscfg)
 *          SOC 0-3 are in High Priority (SOC 4-15 are in low priority mode by default)
 *          all SOC are triggered by EPWM0SOCA signal.
 *          Conversion sequence : SOC 0 --> SOC 1 --> SOC 2 --> SOC 3
 *
 * ADC3 :   (done through Syscfg)
 *          SOC 0-3 are in High Priority (SOC 4-15 are in low priority mode by default)
 *          SOC 1,2 are triggered by EPWM0SOCA signal.
 *          SOC 0,3 are triggered by EPWM0SOCB signal.
 *          Conversion sequence : SOC 1 --> SOC 2 --> SOC 0 --> SOC 3
 *
 * ADC4 :   (done through Syscfg)
 *          SOC 0 are in High Priority (SOC 1-15 are in low priority mode by default)
 *          SOC 0-3 are triggered by EPWM0SOCA signal.
 *          Conversion sequence : SOC 0 --> SOC 1 --> SOC 2 --> SOC 3
 *
 * ADC4 :   (reset and done through the code for reference. Can also be set from syscfg)
 *          SOC 0 are in High Priority (SOC 1-15 are in low priority mode by default)
 *          SOC 1,2 are triggered by EPWM0SOCA signal.
 *          SOC 0,3 are triggered by EPWM0SOCB signal.
 *          Conversion sequence : SOC 1 --> SOC 2 --> SOC 0 --> SOC 3
 *
 * PPB Configurations :
 *      (done through Syscfg for ADC 0-3. done through Code below for ADC 4 for reference)
 *      All PPBx in ADCy are configured for respective SOCx for delay in trigger to signal capture.
 */

#define ADC_CONVERSION_COUNT        2
#define SOC_DELAY_ARRAY_LENGTH      4 * ADC_CONVERSION_COUNT

/* ADC base addresses */
uint32_t gAdc0BaseAddr = CONFIG_ADC0_BASE_ADDR;
uint32_t gAdc1BaseAddr = CONFIG_ADC1_BASE_ADDR;
uint32_t gAdc2BaseAddr = CONFIG_ADC2_BASE_ADDR;
uint32_t gAdc3BaseAddr = CONFIG_ADC3_BASE_ADDR;
uint32_t gAdc4BaseAddr = CONFIG_ADC4_BASE_ADDR;

/* EPWM0 base address*/
uint32_t gEpwm0BaseAddr = CONFIG_EPWM0_BASE_ADDR;

volatile uint16_t gAdc0PpbDelay[SOC_DELAY_ARRAY_LENGTH];
volatile uint16_t gAdc1PpbDelay[SOC_DELAY_ARRAY_LENGTH];
volatile uint16_t gAdc2PpbDelay[SOC_DELAY_ARRAY_LENGTH];
volatile uint16_t gAdc3PpbDelay[SOC_DELAY_ARRAY_LENGTH];
volatile uint16_t gAdc4PpbDelay[SOC_DELAY_ARRAY_LENGTH];
volatile uint16_t gAdc4PpbDelay_case2[SOC_DELAY_ARRAY_LENGTH];


volatile    uint32_t    gIndex;
volatile    uint8_t     gAdc4Case;
volatile    uint32_t    gAdcConversionCount;

static      HwiP_Object gAdcHwiObject;

static void App_adcISR(void *args);

void adc_high_priority_soc_main(void *args)
{

    gAdcConversionCount = ADC_CONVERSION_COUNT;
    gIndex = 0;
    gAdc4Case = 1;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC High Priority SOC Test Started ...\r\n");

    /* Register & enable interrupt */
    int32_t     status;
    HwiP_Params hwiPrms;
    HwiP_Params_init(&hwiPrms);

    /* Integrate with Syscfg : XBarOut0 is configured for ADC4 INT1 */
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_adcISR;
    status              = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Since Continuous Interrrupt Mode is not enabled, INT flag needs to be cleared to service next routine*/
    ADC_clearInterruptStatus(gAdc4BaseAddr, ADC_INT_NUMBER2);
    if(ADC_getInterruptOverflowStatus(gAdc4BaseAddr, ADC_INT_NUMBER2))
    {
        ADC_clearInterruptOverflowStatus(gAdc4BaseAddr, ADC_INT_NUMBER2);
    }

    /* Enabling the EPWM0 TB counter*/
    EPWM_setTimeBaseCounter(gEpwm0BaseAddr, 0);
    EPWM_setTimeBaseCounterMode(gEpwm0BaseAddr, EPWM_COUNTER_MODE_UP_DOWN);

    while(gAdcConversionCount > 0)
    {
        /* wait for all the ADC conversions complete*/
    }

    /* ADC4 reset*/
    SOC_generateAdcReset(4);
    /* enable ADC reference*/
    SOC_enableAdcReference(4);
    /* 500 us delay for ADC to power up*/
    // ClockP_usleep(500);
    /* Configures the analog-to-digital converter module prescaler. */
	ADC_setPrescaler(gAdc4BaseAddr, ADC_CLK_DIV_4_0);
	/* Configures the analog-to-digital converter resolution and signal mode. */
	ADC_setMode(gAdc4BaseAddr, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    /* setting up SOC 0 as high priority*/
    ADC_setSOCPriority(gAdc4BaseAddr, ADC_PRI_SOC0_HIPRI);
    /* setting up SOC 0,1,2,3 for their respective triggers, channels, SampleWindow*/
    ADC_setupSOC(gAdc4BaseAddr, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM0_SOCB, ADC_CH_ADCIN0, 16);
    ADC_setupSOC(gAdc4BaseAddr, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN0, 16);
    ADC_setupSOC(gAdc4BaseAddr, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN0, 16);
    ADC_setupSOC(gAdc4BaseAddr, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM0_SOCB, ADC_CH_ADCIN0, 16);
    /* Enabling and Setting up Interrupt source for INT*/
    ADC_enableInterrupt(gAdc4BaseAddr, ADC_INT_NUMBER1);
    ADC_enableContinuousMode(gAdc4BaseAddr, ADC_INT_NUMBER1);
    ADC_setInterruptSource(gAdc4BaseAddr, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);
    /* Setting up PPBs for respective SOCs*/
    ADC_setupPPB(gAdc4BaseAddr, ADC_PPB_NUMBER1, ADC_SOC_NUMBER0);
    ADC_setupPPB(gAdc4BaseAddr, ADC_PPB_NUMBER2, ADC_SOC_NUMBER1);
    ADC_setupPPB(gAdc4BaseAddr, ADC_PPB_NUMBER3, ADC_SOC_NUMBER2);
    ADC_setupPPB(gAdc4BaseAddr, ADC_PPB_NUMBER4, ADC_SOC_NUMBER3);
    /* ADC Setup complete*/

    gIndex = 0;
    gAdc4Case = 2;
    gAdcConversionCount = ADC_CONVERSION_COUNT;
    /* Enabling the EPWM0 Counter again */
    EPWM_setTimeBaseCounter(gEpwm0BaseAddr, 0);
    EPWM_setTimeBaseCounterMode(gEpwm0BaseAddr, EPWM_COUNTER_MODE_UP_DOWN);
    while(gAdcConversionCount > 0)
    {
        /* wait for all the ADC conversions complete*/
    }


    uint8_t skipInterations = 4;

    DebugP_log("\r\nADC0: All SOC in round robin. All SOC triggered by EPWM0SOCA\r\n");
    DebugP_log("Expected: delay(SOC0)<delay(SOC1)<delay(SOC2)<delay(SOC3)");
    DebugP_log("\r\ndelay(SOC0)\tdelay(SOC1)\tdelay(SOC2)\tdelay(SOC3)\r\n");
    for(gIndex = 0; gIndex < SOC_DELAY_ARRAY_LENGTH; gIndex += skipInterations)
    {
        DebugP_log("%d\t\t%d\t\t%d\t\t%d\r\n", gAdc0PpbDelay[gIndex+0], gAdc0PpbDelay[gIndex+1], gAdc0PpbDelay[gIndex+2], gAdc0PpbDelay[gIndex+3]);
    }

    DebugP_log("\r\nADC1: All SOC in round robin. SOC1,2 triggered by EPWM0SOCA.SOC0,3 triggered by EPWM0SOCB \r\n");
    DebugP_log("Expected: delay(SOC1)<delay(SOC2) delay(SOC3)<delay(SOC0)");
    DebugP_log("\r\ndelay(SOC0)\tdelay(SOC1)\tdelay(SOC2)\tdelay(SOC3)\r\n");
    for(gIndex = 0; gIndex < SOC_DELAY_ARRAY_LENGTH; gIndex += skipInterations)
    {
        DebugP_log("%d\t\t%d\t\t%d\t\t%d\r\n", gAdc1PpbDelay[gIndex+0], gAdc1PpbDelay[gIndex+1], gAdc1PpbDelay[gIndex+2], gAdc1PpbDelay[gIndex+3]);
    }

    DebugP_log("\r\nADC2: All SOC in high Priority. All SOC triggered by EPWM0SOCA\r\n");
    DebugP_log("Expected: delay(SOC0)<delay(SOC1)<delay(SOC2)<delay(SOC3)");
    DebugP_log("\r\ndelay(SOC0)\tdelay(SOC1)\tdelay(SOC2)\tdelay(SOC3)\r\n");
    for(gIndex = 0; gIndex < SOC_DELAY_ARRAY_LENGTH; gIndex += skipInterations)
    {
        DebugP_log("%d\t\t%d\t\t%d\t\t%d\r\n", gAdc2PpbDelay[gIndex+0], gAdc2PpbDelay[gIndex+1], gAdc2PpbDelay[gIndex+2], gAdc2PpbDelay[gIndex+3]);
    }

    DebugP_log("\r\nADC3: All SOC in High Priority. SOC1,2 triggered by EPWM0SOCA.SOC0,3 triggered by EPWM0SOCB \r\n");
    DebugP_log("Expected: delay(SOC1)<delay(SOC2) delay(SOC0)<delay(SOC3)");
    DebugP_log("\r\ndelay(SOC0)\tdelay(SOC1)\tdelay(SOC2)\tdelay(SOC3)\r\n");
    for(gIndex = 0; gIndex < SOC_DELAY_ARRAY_LENGTH; gIndex += skipInterations)
    {
        DebugP_log("%d\t\t%d\t\t%d\t\t%d\r\n", gAdc3PpbDelay[gIndex+0], gAdc3PpbDelay[gIndex+1], gAdc3PpbDelay[gIndex+2], gAdc3PpbDelay[gIndex+3]);
    }

    DebugP_log("\r\nADC4: SOC0 in high Priority. All SOC triggered by EPWM0SOCA\r\n");
    DebugP_log("Expected: delay(SOC0)<delay(SOC1)<delay(SOC2)<delay(SOC3)");
    DebugP_log("\r\ndelay(SOC0)\tdelay(SOC1)\tdelay(SOC2)\tdelay(SOC3)\r\n");
    for(gIndex = 0; gIndex < SOC_DELAY_ARRAY_LENGTH; gIndex += skipInterations)
    {
        DebugP_log("%d\t\t%d\t\t%d\t\t%d\r\n", gAdc4PpbDelay[gIndex+0],gAdc4PpbDelay[gIndex+1],gAdc4PpbDelay[gIndex+2],gAdc4PpbDelay[gIndex+3]);
    }

    DebugP_log("\r\nADC4: SOC0 in round robin. SOC1,2 triggered by EPWM0SOCA.SOC0,3 triggered by EPWM0SOCB \r\n");
    DebugP_log("Expected: delay(SOC0) should be between 0-1 conversions\r\n");
    DebugP_log("\tdelay(SOC1) : 0 conversions, SOC2 : 2 Conversions, SOC3: 2 conversions\r\n");
    DebugP_log("\r\ndelay(SOC0)\tdelay(SOC1)\tdelay(SOC2)\tdelay(SOC3)\r\n");
    for(gIndex = 0; gIndex < SOC_DELAY_ARRAY_LENGTH; gIndex += skipInterations)
    {
        DebugP_log("%d\t\t%d\t\t%d\t\t%d\r\n", gAdc4PpbDelay_case2[gIndex+0], gAdc4PpbDelay_case2[gIndex+1], gAdc4PpbDelay_case2[gIndex+2], gAdc4PpbDelay_case2[gIndex+3]);
    }
    /* Clear and disable interrupt */
    ADC_disableInterrupt(gAdc4BaseAddr, ADC_INT_NUMBER2);
    ADC_clearInterruptStatus(gAdc4BaseAddr, ADC_INT_NUMBER2);

    /* Power down the ADC */
    ADC_disableConverter(gAdc0BaseAddr);
    ADC_disableConverter(gAdc1BaseAddr);
    ADC_disableConverter(gAdc2BaseAddr);
    ADC_disableConverter(gAdc3BaseAddr);
    ADC_disableConverter(gAdc4BaseAddr);

    DebugP_log("\r\nADC High Priority Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_adcISR(void *args)
{   EPWM_setTimeBaseCounterMode(gEpwm0BaseAddr, EPWM_COUNTER_MODE_STOP_FREEZE);
    if(gAdc4Case == 1)
    {
        for(int iter = 0; iter < 4; iter++)
        {
            gAdc0PpbDelay[gIndex+iter] = ADC_getPPBDelayTimeStamp(gAdc0BaseAddr, iter);
            gAdc1PpbDelay[gIndex+iter] = ADC_getPPBDelayTimeStamp(gAdc1BaseAddr, iter);
            gAdc2PpbDelay[gIndex+iter] = ADC_getPPBDelayTimeStamp(gAdc2BaseAddr, iter);
            gAdc3PpbDelay[gIndex+iter] = ADC_getPPBDelayTimeStamp(gAdc3BaseAddr, iter);
            gAdc4PpbDelay[gIndex+iter] = ADC_getPPBDelayTimeStamp(gAdc4BaseAddr, iter);
        }
    }
    else
    {
        for(int iter = 0; iter < 4; iter++)
        {
            gAdc4PpbDelay_case2[gIndex+iter] = ADC_getPPBDelayTimeStamp(gAdc4BaseAddr, iter);
        }
    }
    gIndex += 4;
    gAdcConversionCount--;


    /* Since Continuous Interrrupt Mode is not enabled, INT flag needs to be cleared to service next routine*/
    ADC_clearInterruptStatus(gAdc4BaseAddr, ADC_INT_NUMBER2);
    if(ADC_getInterruptOverflowStatus(gAdc4BaseAddr, ADC_INT_NUMBER2))
    {
        ADC_clearInterruptOverflowStatus(gAdc4BaseAddr, ADC_INT_NUMBER2);
    }
    if(gAdcConversionCount <= 0)
    {
        /* Change EPWM0 counter mode to stop and freeze*/
        EPWM_setTimeBaseCounterMode(gEpwm0BaseAddr, EPWM_COUNTER_MODE_STOP_FREEZE);
    }
    else
    {
        /* Change EPWM0 counter mode to stop and freeze*/
        EPWM_setTimeBaseCounter(gEpwm0BaseAddr,0);
        EPWM_setTimeBaseCounterMode(gEpwm0BaseAddr, EPWM_COUNTER_MODE_UP_DOWN);
    }

}







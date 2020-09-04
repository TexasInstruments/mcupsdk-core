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
 * Example Description
 *      The example demonstrates Burst mode of ADC periodically triggered by
 * trigger from EPWM0.
 *
 * SOC Configurations
 * - SOC0 - SOC6 are configured High priority
 * - SOC7 - SOC15 are configured for Burst mode, with Burst size of 3.
 *   i.e., SOC(7,8,9), SOC(10,11,12), SOC(13,14,15) act as bursts and
 * - EPWM0SOCA is selected as trigger for Burst mode.
 * - SOC(7,10,13, 14) sample on ADC 1 Channel 0
 * - SOC(8,11)        sample on ADC 1 Channel 1
 * - SOC(9,12)        sample on ADC 1 Channel 2
 * - SOC(15)          sample on ADC 1 Channel 3
 *
 * Interrupt Configurations
 * - ADC1 INT 1 is generated at the EOC/SOC9
 * - ADC1 INT 2 is generated at the EOC/SOC12
 * - ADC1 INT 3 is generated at the EOC/SOC15
 *
 * ISR
 * - ADC INT 1,2,3 are configured to same ISR, App_adcISR.
 * - reads which EOC generated Interrupt and reads the respective
 *   result registers.
 *
 * The below watch variables can be used to view ADC conversion results.
 *
 * Watch Variables
 * - gAdc1Result0[] : Digital representation of the voltage, averaged on
                      burst sample on pin ADC0_AIN0
 * - gAdc1Result1[] : Digital representation of the voltage, averaged on
                      burst sample on pin ADC0_AIN1
 * - gAdc1Result2[] : Digital representation of the voltage sampled on pin ADC0_AIN2
 * - gAdc1result3[] : Digital representation of the voltage sampled on pin ADC0_AIN3
 *
 * External Connections :
 * AM263x-CC :
 *      Feed analog voltages on
 *      - ADC1_AIN0, i.e., HSEC PIN  18
 *      - ADC1_AIN1, i.e., HSEC PIN  20
 *      - ADC1_AIN2, i.e., HSEC PIN  21
 *      - ADC1_AIN3, i.e., HSEC PIN  23
 * AM263x-LP :
 *      Feed analog voltages on
 *      - ADC1_AIN0, i.e., J1/3 PIN  24
 *      - ADC1_AIN1, i.e., J1/3 PIN  29
 *      - ADC1_AIN2, i.e., J5/7 PIN  67
 *      - ADC1_AIN3, i.e., J1/3 PIN  6
 *
 */


/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT     1024

/* Global variables and objects */
/* Variable to store conversion results from all 4 pins  */
uint16_t gAdc1Result0[ADC_CONVERSION_COUNT];
uint16_t gAdc1Result1[ADC_CONVERSION_COUNT];
uint16_t gAdc1Result2[ADC_CONVERSION_COUNT];
uint16_t gAdc1Result3[ADC_CONVERSION_COUNT];




uint32_t gAdc1baseAddr = CONFIG_ADC1_BASE_ADDR;
uint32_t gAdc1ResultBaseAddr = CONFIG_ADC1_RESULT_BASE_ADDR;

static HwiP_Object  gAdcHwiObject;


/* Variable to store the count of completed conversions
Initialising conversion count to 0 */
volatile uint32_t gAdcConversionCount = 0;
volatile uint16_t gResultIndex = 0;

/* Function Prototypes */
static void App_adcISR(void *args);

void adc_burst_mode_epwm_main(void *args)
{

    /* variable to iterate ADC_CONVERSION_COUNT */

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();


    DebugP_log("ADC Burst Mode EPWM Test Started ...\r\n");

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

    /* Setting the EPWM TB counter mode to up count mode*/
    EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_UP);

    /* Wait until required number of ADC conversions are completed */
    while( gAdcConversionCount < ADC_CONVERSION_COUNT );

    /* Remove EPWMSOCA trigger*/
    EPWM_disableADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A);

    gResultIndex = 0;

    /* Print few elements from the result buffer */
        DebugP_log("ADC1Digital Converted Values\r\nChannel0    :   Channel1    :   Channel2   : Channel3 \r\n");

        while(gResultIndex < ADC_CONVERSION_COUNT)
        {
            DebugP_log("    %d  :   %d  :   %d  :   %d\r\n",
                        gAdc1Result0[gResultIndex],
                        gAdc1Result1[gResultIndex],
                        gAdc1Result2[gResultIndex],
                        gAdc1Result3[gResultIndex]);

            gResultIndex += 100;
        }

    DebugP_log("ADC Burst Mode EPWM Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_adcISR(void *args)
{
    uint16_t rrPointer;
    ADC_IntNumber burstInterruptSource;
    /* check last conversion and read those results. */
    rrPointer = (HW_RD_REG16(gAdc1baseAddr + CSL_ADC_ADCSOCPRICTL) & 0x03E0) >> 5;
    switch(rrPointer)
    {
        case 9:
            gAdc1Result0[gResultIndex] = ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER7);
            gAdc1Result1[gResultIndex] = ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER8);
            gAdc1Result2[gResultIndex] = ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER9);

            burstInterruptSource = ADC_INT_NUMBER1;

            break;
        case 12:
            gAdc1Result0[gResultIndex] += ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER10);
            gAdc1Result1[gResultIndex] += ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER11);
            gAdc1Result2[gResultIndex] += ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER12);

            burstInterruptSource = ADC_INT_NUMBER2;

            break;
        case 15:
            gAdc1Result0[gResultIndex] += ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER13);
            gAdc1Result0[gResultIndex] += ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER14);
            gAdc1Result3[gResultIndex] += ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER15);

            gAdc1Result0[gResultIndex] = (gAdc1Result0[gResultIndex])>>2;   /*Dividing by 4*/
            gAdc1Result1[gResultIndex] = (gAdc1Result1[gResultIndex])>>1;   /*Dividing by 2*/
            gAdc1Result2[gResultIndex] = (gAdc1Result2[gResultIndex])>>1;   /*Dividing by 2*/

            gResultIndex++;
            gAdcConversionCount++;

            burstInterruptSource = ADC_INT_NUMBER3;

            break;
        default:
            burstInterruptSource = ADC_INT_NUMBER4;
            break;
    }
    ADC_clearInterruptStatus(gAdc1baseAddr, burstInterruptSource);
    if(true == ADC_getInterruptOverflowStatus(gAdc1baseAddr, burstInterruptSource))
    {
        ADC_clearInterruptOverflowStatus(gAdc1baseAddr, burstInterruptSource);
    }
    if (gAdcConversionCount >= ADC_CONVERSION_COUNT)
    {
        /* stopping the TBCounter to halt PWM*/
        EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_STOP_FREEZE);
    }

}

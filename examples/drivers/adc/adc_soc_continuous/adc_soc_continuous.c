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
 *      This example converts ADC 1 Channel 0 on all its SOC configurations,
 * essentially achieving full sampling rate on the input signal on the
 * given channel. The ADC clock is prescaled to a factor of 3.
 *
 * SOC Configurations :
 * 1. ADC_INT1 is set to trigger SOC(0-15).
 * 2. All SOC are set to be triggered only throguh software
 * 3. Sampling window is 17 system clock cycles, the conversion happens for 31
 *    system clock cycles. (each SOC is 49 system clock cycles long).
 *    therefore each sample conversion happens in approcimately 4MSPS
 * 4. EOC/SOC7 provides INT1.
 * 5. EOC/SOC15 provides INT2.
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
#define SAMPLES_COUNT            16*ADC_CONVERSION_COUNT
/* Global variables and objects */
volatile uint32_t gAdc1Result0[SAMPLES_COUNT];
volatile uint32_t gIndex = 0;
volatile uint32_t ResultAdc1Ch0 = 0;

uint32_t gAdc1baseAddr = CONFIG_ADC1_BASE_ADDR;
uint32_t gAdc1resultBaseAddr = CONFIG_ADC1_RESULT_BASE_ADDR;

/* Variable to store the count of completed conversions
Initialising conversion count to 0 */
volatile uint32_t gAdcConversionCount = 0;


void adc_soc_continuous_main(void *args)
{


    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC Soc Continuous Test Started\r\n");

    gIndex = 0;
    ADC_forceMultipleSOC(gAdc1baseAddr, 0xFFFF);
    while(gAdcConversionCount < ADC_CONVERSION_COUNT)
    {
        while(false == ADC_getInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER2));
        ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER2);
        ResultAdc1Ch0 = 0;
        for(ADC_SOCNumber soc_number = ADC_SOC_NUMBER0; soc_number < ADC_SOC_NUMBER8; soc_number++)
        {
            gAdc1Result0[gIndex++] = ADC_readResult(gAdc1resultBaseAddr,soc_number);
        }

        while(false == ADC_getInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1));
        ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
        for(ADC_SOCNumber soc_number = ADC_SOC_NUMBER8; soc_number < ADC_SOC_NUMBER15; soc_number++)
        {
            gAdc1Result0[gIndex++] = ADC_readResult(gAdc1resultBaseAddr, soc_number);
        }
        gAdcConversionCount++;
    }

    uint32_t gSkipIterations = 1600;
    DebugP_log("ADC 1 channel 1 output:\r\n");
    for(gIndex = 0; gIndex < SAMPLES_COUNT; gIndex += gSkipIterations)
    {
        DebugP_log("\t%d\r\n",gAdc1Result0[gIndex]);
    }
    DebugP_log("ADC Soc Continuous Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}


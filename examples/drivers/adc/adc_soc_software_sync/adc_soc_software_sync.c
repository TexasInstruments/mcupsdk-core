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
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * Example Description
 *      This example shows synchronous operation on ADC1 and ADC2 triggered
 * by a software forced by toggling a GPIO. The example uses a GPIO loopback
 * into the INPUTBAR[5] as trigger for the SOC in the given ADC and uses the
 * software to toggle to the loopback GPIO trigger the conversions.
 *
 * SOC Configurations
 * - ADC 1
 *      - SOC 0,1 triggers are set to InputXbar[5]
 *      - SOC 0 samples Channel 0
 *      - SOC 1 samples Channel 1
 *
 * - ADC 2
 *      - SOC 0,1 triggers are set to InputXbar[5]
 *      - SOC 0 samples Channel 0
 *      - SOC 1 samples Channel 2
 *
 * Interrupt Configurations
 * - ADC1INT1 source is set to EOC/SOC1
 * - INTXbar[0] is configured for ADC1INT1
 *
 * ISR
 * - App_adcISR read the results stored by SOC0,1 in ADC1,2.
 *
 * External Connections
 * - AM263x-CC
 *      - Connect loopback on GPIO 24, GPIO 23, i.e., HSEC PINS 87, 85.
 *      - Feed Analog voltage on
 *          - ADC 1 Channel 0 : HSEC PIN 18
 *          - ADC 1 Channel 1 : HSEC PIN 20
 *          - ADC 2 Channel 0 : HSEC PIN 24
 *          - ADC 2 Channel 1 : HSEC PIN 26
 * - AM263x-LP
 *      - Connect loopback on GPIO 24, GPIO 23, i.e., J5/7 PINS 49,50.
 *      - Feed Analog voltage on
 *          - ADC 1 Channel 0 : J1/3 PIN 24
 *          - ADC 1 Channel 1 : J1/3 PIN 29
 *          - ADC 2 Channel 0 : J1/3 PIN 25
 *          - ADC 2 Channel 1 : J5/7 PIN 63
 *
 * Watch Variables
 * - gAdc1Result0 : Digital representation of Voltages on ADC 1 Channel 0
 * - gAdc1Result1 : Digital representation of Voltages on ADC 1 Channel 1
 * - gAdc2Result0 : Digital representation of Voltages on ADC 2 Channel 0
 * - gAdc2Result1 : Digital representation of Voltages on ADC 2 Channel 1
 */

#define ADC_CONVERSION_COUNT 1024

uint32_t gAdc1Result0[ADC_CONVERSION_COUNT];
uint32_t gAdc1Result1[ADC_CONVERSION_COUNT];
uint32_t gAdc2Result0[ADC_CONVERSION_COUNT];
uint32_t gAdc2Result1[ADC_CONVERSION_COUNT];


uint32_t gAdc1baseAddr = CONFIG_ADC1_BASE_ADDR;
uint32_t gAdc2baseAddr = CONFIG_ADC2_BASE_ADDR;

uint32_t gAdc1ResultBaseAddr = CONFIG_ADC1_RESULT_BASE_ADDR;
uint32_t gAdc2ResultBaseAddr = CONFIG_ADC2_RESULT_BASE_ADDR;

static HwiP_Object  gAdcHwiObject;

/* Variable to store the conversion count and a variable to iterate over results*/
volatile uint32_t gAdcConversionCount = 0;
volatile uint16_t gResultIndex = 0;

void App_adcISR(void *args);

void adc_soc_software_sync_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC Synchronous Software Triggered Conversion Test Started ...\r\n");

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

    /* Setting the GPIO pins direction*/
    GPIO_setDirMode(GPIO_23_INPUT_BASE_ADDR, GPIO_23_INPUT_PIN, GPIO_23_INPUT_DIR);
    GPIO_setDirMode(GPIO_24_OUTPUT_BASE_ADDR, GPIO_24_OUTPUT_PIN, GPIO_24_OUTPUT_DIR);

    while(gAdcConversionCount < ADC_CONVERSION_COUNT)
    {
        /* toggling the GPIO to trigger the Conversions */
        GPIO_pinWriteHigh(GPIO_24_OUTPUT_BASE_ADDR, GPIO_24_OUTPUT_PIN);
        ClockP_usleep(50);                      /* delay of 50 uSec (arbitrary)*/
        GPIO_pinWriteLow(GPIO_24_OUTPUT_BASE_ADDR, GPIO_24_OUTPUT_PIN);

        /* wait until conversion is complete*/
        while(false == ADC_getInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1));

        /* clear Interrupt flag.*/
        ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
        if(true == ADC_getInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER1))
        {
            ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
            ADC_clearInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
        }
    }

    /* Printing some of the conversion values*/
    DebugP_log("\tADC1\t\tADC1\r\n");
    DebugP_log("\tSOC0  SOC1\tSOC0  SOC1\r\n");
    int skipCount = 100;
    for(int iter = 0; iter < ADC_CONVERSION_COUNT; iter+= skipCount)
    {
        DebugP_log("\t%d \t%d \t%d \t%d\r\n",
                gAdc1Result0[iter],
                gAdc1Result1[iter],
                gAdc1Result0[iter],
                gAdc1Result1[iter]);
    }

    DebugP_log("ADC Synchronous Software Triggered Conversion Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}


void App_adcISR(void *args)
{
    /* Reading the Result Registers of SOC 0,1 in ADC 1,2 */
    gAdc1Result0[gResultIndex] = ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER0);
    gAdc1Result1[gResultIndex] = ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER1);
    gAdc2Result0[gResultIndex] = ADC_readResult(gAdc2ResultBaseAddr, ADC_SOC_NUMBER0);
    gAdc2Result1[gResultIndex] = ADC_readResult(gAdc2ResultBaseAddr, ADC_SOC_NUMBER1);

    if(gAdcConversionCount < ADC_CONVERSION_COUNT)
    {
        gResultIndex++;
        gAdcConversionCount++;
    }
    else
    {
        /* Disabling interrupt for ADC */
        ADC_disableInterrupt(gAdc1baseAddr, ADC_INT_NUMBER1);
    }
    /* Clearing interrupt flag is handled in the main routine.*/
}


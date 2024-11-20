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
 * Example Describption
 * In AM263Px DAC can be loopedback to ADC CAL1, and in AM261x the DAC can be loopedback to ADC CAL0
 * This example demostrates the ADC-DAC loopback feature. Each ADC has two CAL channels viz. CAL Channel 0 and CAL Channel 1. these CAL channels are common to all the ADCs. While using the ADC-DAC loopback, the following needs to be taken care of,
 * 1. ADC cal channel should be of high impedence, i.e., no source driving it.
 * 2. DAC output should be driven only by DAC and there should not be any other source driving it.
 * 3. ADC sampling times must be increased in this mode. the example uses the 256 Sample and Hold window configuration.  
 * 4. Sampling the DAC voltage with multiple ADCs at the same time will produce inconsistent results. Due to sampling kickback and charge injection. Hence, only one ADC may be used with the given CAL channel at any point of using the DAC-ADC loopback.
 * 
 * Configurations
 * - DAC is configured using syscfg
 * - ADC SOC0 is configured CAL0 Channel, with sample and hold window of 256 and generate interrupt at EOC0
 * 
 * External Connections
 * - No external connections required
 * 
 * Watch Variables
 * The below watch variables can be used to view ADC conversion results.
 * - gAdcResults[]        : Digital representation of the voltage sample on pin AIN2 of ADC0, triggered by RTI
 * - gDac_shadowValues[]  : DAC digital values corresponding to the ADC conversions. 
 *
 */


/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT     (40U)

/* Max code error defined in the Expected ADC-DAC loopback converted to voltage */
#define MAX_ADC_DAC_LOOPBACK_ERROR_VOLTAGE (0.02)

uint32_t gConversionCount = 0;
uint32_t gDacBaseAddr     = CONFIG_DAC0_BASE_ADDR;
uint32_t gAdcBaseAddr     = CONFIG_ADC0_BASE_ADDR;
uint32_t gAdcResBaseAddr  = CONFIG_ADC0_RESULT_BASE_ADDR;

uint16_t gDac_shadowValues [ADC_CONVERSION_COUNT] = {0};
uint16_t gAdcResults[ADC_CONVERSION_COUNT] = {0};
void adc_dac_loopback_main(void *args)
{
    /* variable to iterate ADC_CONVERSION_COUNT */

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC DAC Loopback Test Started ...\r\n");
    
    /* populating the DAC values */
    for(uint8_t iter = 0; iter < ADC_CONVERSION_COUNT; iter++)
    {
        gDac_shadowValues[iter] = iter*100; 
    }
    
    /* Interrupts are enabled for the EOC-0, clearing the Interrupt flags if any */
    ADC_clearInterruptStatus(gAdcBaseAddr, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(gAdcBaseAddr, ADC_INT_NUMBER1);

    /* enabling the ADC-DAC loopback */
    SOC_enableAdcDacLoopback(TRUE);

    /* changing the DAC value and reading the ADC result to an array */
    for(uint8_t iter = 1; iter< ADC_CONVERSION_COUNT; iter++)
    {   
        /* setting the shadow value for the DAC */
        DAC_setShadowValue(gDacBaseAddr, gDac_shadowValues[iter]);
        
        ClockP_usleep(10);
        
        /* Software forcing the ADC SOC */
        ADC_forceSOC(gAdcBaseAddr, ADC_SOC_NUMBER0);

        /* waiting on the ADC interrupt */
        while(true != ADC_getInterruptStatus(gAdcBaseAddr, ADC_INT_NUMBER1));

        /* reading the result */
        gAdcResults[iter] = ADC_readResult(gAdcResBaseAddr, ADC_SOC_NUMBER0);

        /* clearing the interrupt flags */
        ADC_clearInterruptStatus(gAdcBaseAddr, ADC_INT_NUMBER1);
        ADC_clearInterruptOverflowStatus(gAdcBaseAddr, ADC_INT_NUMBER1);

    }
    uint8_t errors = 0;
    DebugP_log("DAC code\t|\tADC Result\r\n");
    for(uint8_t iter = 1; iter < ADC_CONVERSION_COUNT; iter++)
    {
        DebugP_log("\t%d\t|\t%d\r\n", gDac_shadowValues[iter], gAdcResults[iter]);

        float adc_voltage = (gAdcResults[iter])*(3.2/4096);
        float dac_voltage = (gDac_shadowValues[iter])*(3.3/4096);

        DebugP_log("--->%.5fV\t|\t%.5fV\r\n", dac_voltage, adc_voltage);

        /* codes should not be way off */
        float errorVoltage = (dac_voltage - adc_voltage);
        if((errorVoltage > MAX_ADC_DAC_LOOPBACK_ERROR_VOLTAGE) || (errorVoltage < -MAX_ADC_DAC_LOOPBACK_ERROR_VOLTAGE))
        {
            errors++;
        }
    }
    if(errors > 0)
    {
        DebugP_log("Some ADC Converted voltages are very different than DAC generated ones.\r\n");
        DebugP_log("Some tests have Failed!!\r\n");
    }
    else
    {
        DebugP_log("ADC DAC loopback Test Passed\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    /* Removing the ADC-DAC loopback */
    SOC_enableAdcDacLoopback(FALSE);

    Board_driversClose();
    Drivers_close();
}

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
 *      This example showcases the global force feature. The ADC SOCs can be 
 * triggered via software. these software triggers can also be a synchronous 
 * across multiple ADCs and multiple SOCs. In the following example, 
 * the ADC0-4 are selected for the global force and the SOC 0 on each of 
 * them is triggered at once. 
 * 
 * External Connections
 *      External Connections are arbitrary, for conversion and the SOC configuration,
 * one of the SOC is configured for the ADC 0 Channel 2.
 *  - on AM263x CC E2, AM263Px CC E2, with HSEC Dock 
 *      - Feed Analog input to ADC0_AIN2 - HSEC PIN 15  
 *  - on AM263x LP E2, AM263Px LP
 *      - Feed Analog Input to the ADC0_AIN2 - J7 Pin 66
 * 
 * Watch Variables 
 *  gAdc0Results[] - array holding the results converted by ADC0 on Global Force Triggers
 *  gAdc1Results[] - array holding the results converted by ADC1 on Global Force Triggers
 *  gAdc2Results[] - array holding the results converted by ADC2 on Global Force Triggers
 *  gAdc3Results[] - array holding the results converted by ADC3 on Global Force Triggers
 *  gAdc4Results[] - array holding the results converted by ADC4 on Global Force Triggers
 */


/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT     (256U)

/* definitions in main.c */
extern uint32_t gAdcBaseAddr[CONFIG_ADC_NUM_INSTANCES];

extern uint32_t gAdcResultBaseAddr[CONFIG_ADC_NUM_INSTANCES];

extern uint16_t* gAdcResults[CONFIG_ADC_NUM_INSTANCES];

uint32_t globalForceSocNumber = (uint32_t)ADC_SOC_NUMBER0;

uint32_t gConversionCount = 0;

void adc_global_force_main(void *args)
{
    /* variable to iterate ADC_CONVERSION_COUNT */

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();


    DebugP_log("ADC Triggered by Global Software Force Test Started ...\r\n");


    for(uint8_t adcInst = 0; adcInst < CONFIG_ADC_NUM_INSTANCES; adcInst++)
    {
        /* enabling the ADCs for the global force */
        SOC_enableAdcGlobalForce(adcInst, TRUE);

        /* Clearing Interrupt Status for all ADCs involved */
        ADC_clearInterruptStatus(gAdcBaseAddr[adcInst], ADC_INT_NUMBER1);
        ADC_clearInterruptOverflowStatus(gAdcBaseAddr[adcInst], ADC_INT_NUMBER1);    
    }

    uint32_t status = 0;
    while(gConversionCount < ADC_CONVERSION_COUNT)
    {

        /* triggering the ADC SOC 0 on all the selected ADCs */
        SOC_adcSocGlobalForce(globalForceSocNumber);
        
        /* wait on the ADC0, INT0, generated at EOC0 */
        while(true != ADC_getInterruptStatus(gAdcBaseAddr[0], ADC_INT_NUMBER1));
        
        for(uint8_t adcInst = 0; adcInst < CONFIG_ADC_NUM_INSTANCES; adcInst++)
        {   
            
            /* The other ADC INT0 should have been set too, as all the SOCs are 
            triggered at once and have same conversion time. */
            status = ADC_getInterruptStatus(gAdcBaseAddr[adcInst], ADC_INT_NUMBER1);

            if(status == 0)
            {   
                DebugP_log("Interrupt not Set on the ADC instance : %d, base : 0x%x\r\n", adcInst, gAdcBaseAddr[adcInst]);
                break;
            }else
            {
                /* Conversions happened, reading result */
                gAdcResults[adcInst][gConversionCount] = ADC_readResult(gAdcResultBaseAddr[adcInst], ADC_SOC_NUMBER0);
            }

            /* Clearing Interrupt Status for all ADCs involved */
            ADC_clearInterruptStatus(gAdcBaseAddr[adcInst], ADC_INT_NUMBER1);
            ADC_clearInterruptOverflowStatus(gAdcBaseAddr[adcInst], ADC_INT_NUMBER1);    
        }
        if(status == 0)
        {
            break;
        }
        gConversionCount++;

    }
    if(status == 0)
    {
        /* the ISR counts for the RTI should be same or 1 more than those of ADC considering the timing*/
        DebugP_log("ADC Triggered by Global Software Force test failed. One or more Global Software Forced conversions are missed.\r\n");
        DebugP_log("Some tests have Failed!!\r\n");
    }
    else
    {
        DebugP_log("ADC Triggered by Global Software Force Test Passed\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    /* Removing the ADCs from the Global Force */
    for(uint8_t adcInst = 0; adcInst < CONFIG_ADC_NUM_INSTANCES; adcInst++)
    {
        SOC_enableAdcGlobalForce(adcInst, FALSE);
    }

    Board_driversClose();
    Drivers_close();
}

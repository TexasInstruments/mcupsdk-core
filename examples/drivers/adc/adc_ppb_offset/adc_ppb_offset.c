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
 *      This example demonstrates the PPB offset features of Post Processing
 * Blocks of ADC. The PPB offset features are the following,
 *      1. PPB Calibration Offset :
 *              This is used for correcting the calibration offset of the
 *          selected SOC result. for example, when set the ADC result corresponding
 *          to the set SOC ADC_result = ADC_result - PPB Calibration value.
 *              If more than one PPB are set for the given SOC, then the result with
 *          higher value after calculation will be implemented.
 *      2. PPB Reference Offset :
 *              This can be used to calculate an offset from a given reference. the
 *          result will be specific to the PPB registers and is stored seperately
 *          from the SOC result. for example, when set, the PPB result corresponding
 *          to the set SOC ADC_result, PPB_result = ADC_result - PPB Reference value.
 *              If "Twos complement mode" is selected, then,
 *           PPB_result = PPB Reference value - ADC_result
 *
 * SOC Configurations :
 *      ADC SOC0 and SOC1 are set for Channel 0, with sample window 17 each and
 * are triggered by only software.
 *
 * PPB Configurations :
 *              | PPB1 | PPB2 | PPB3
 *          ----|------|------|-----
 *          SOC | SOC1 | SOC1 | SOC1
 *    Cal offset| -100 | 50   | 100
 *    Ref offset| 0    | 50   | 50
 *    Twos Compl| no   | no   | yes
 *
 * Interrupt Configurations:
 *      ADC1_INT1 is set to EOC/SOC1.
 *
 * External Connections :
 * AM263X-CC    :   Feed Analog Input to ADC1_AIN0, HSEC Pin 18
 * AM263X-LP    :   Feed Analog Input to ADC1_AIN0, J1/3 Pin 24
 *
 * Watch Variables :
 * - gAdc1Soc0Result[] holds the digital representation of the Analog signal on ADC1_AIN0
 * - gAdc1Soc1Result[] holds the digital representation of the Analog signal on ADC1_AIN0
 *                     with Calibration offset
 * - gAdc1PPB1Result[] holds the digital representation of the Analog signal on ADC1_AIN0
 *                     with Calibration offset with Reference Offset of 0
 * - gAdc1PPB1Result[] holds the digital representation of the Analog signal on ADC1_AIN0
 *                     with Calibration offset with Reference Offset of 50
 * - gAdc1PPB1Result[] holds the digital representation of the Analog signal on ADC1_AIN0
 *                     with Calibration offset with Reference Offset of 50
 *                     and Twos complement enabled.
 */


/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT     256

volatile uint32_t gAdc1BaseAddr = CONFIG_ADC1_BASE_ADDR;
volatile uint32_t gAdc1ResultBaseAddr = CONFIG_ADC1_RESULT_BASE_ADDR;
/* ADC SOC results array*/
volatile uint32_t gAdc1Soc0Result[ADC_CONVERSION_COUNT];
volatile uint32_t gAdc1Soc1Result[ADC_CONVERSION_COUNT];

/* PPB results*/
volatile int32_t gAdc1PPB1Result[ADC_CONVERSION_COUNT];
volatile int32_t gAdc1PPB2Result[ADC_CONVERSION_COUNT];
volatile int32_t gAdc1PPB3Result[ADC_CONVERSION_COUNT];

volatile uint32_t gAdcConversionCount = 0;

void adc_ppb_offset_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC PPB Offset Test Started\r\n");


    ADC_clearInterruptStatus(gAdc1BaseAddr, ADC_INT_NUMBER1);
    while(gAdcConversionCount<ADC_CONVERSION_COUNT)
    {
        /* forcing the SOC covnersion*/
        ADC_forceSOC(gAdc1BaseAddr, ADC_SOC_NUMBER0);
        ADC_forceSOC(gAdc1BaseAddr, ADC_SOC_NUMBER1);

        /* waiting for the Interrupt flag, generates at the end of conversion*/
        while(false == ADC_getInterruptStatus(gAdc1BaseAddr, ADC_INT_NUMBER1));
        /* Clearing the interrupt flag*/
        ADC_clearInterruptStatus(gAdc1BaseAddr, ADC_INT_NUMBER1);

        /* Reading the SOC result*/
        gAdc1Soc0Result[gAdcConversionCount] = ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER0);
        gAdc1Soc1Result[gAdcConversionCount] = ADC_readResult(gAdc1ResultBaseAddr, ADC_SOC_NUMBER1);


        /* Reading the PPB results*/
        gAdc1PPB1Result[gAdcConversionCount] = ADC_readPPBResult(gAdc1ResultBaseAddr, ADC_PPB_NUMBER1);
        gAdc1PPB2Result[gAdcConversionCount] = ADC_readPPBResult(gAdc1ResultBaseAddr, ADC_PPB_NUMBER2);
        gAdc1PPB3Result[gAdcConversionCount] = ADC_readPPBResult(gAdc1ResultBaseAddr, ADC_PPB_NUMBER3);
        gAdcConversionCount++;
    }

    /*
     * SOC result holds the calibration corrected value from PPB1 and PPB2, of which,
     * (SOC_value - PPB calibration value) is largest.
     *
     * PPB 1 result holds the SOC calibration corrected value offset with reference offset value of 100.
     * PPB 2 result holds the SOC calibration corrected value offset with reference offset value of 50.
     */
    DebugP_log("SOC0\tSOC1(SOC0 result + calibration offset)\tPPB1(SOC1 result)\tPPB2(SOC1 result + ref offset)\tPPB3(PPB2 result in 2's complement)\r\n");
    int skipIterations = 32;
    for(int index=0; index < ADC_CONVERSION_COUNT; index += skipIterations)
    {
        DebugP_log("\t%d\t\t%d\t\t%d\t\t%d\t\t%d\r\n",gAdc1Soc0Result[index], gAdc1Soc1Result[index], gAdc1PPB1Result[index], gAdc1PPB2Result[index], gAdc1PPB3Result[index]);
    }

    DebugP_log("ADC PPB Offset Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

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
 *      This example demonstrates the Open Shorts Detection circuit on the ADC channels
 * for detecting the pin faults in the system. the example enables the OSD circuit along
 * with the mandatory ADC configurations (see Assumptions below) and diagnoses ADC0-Channel 0
 * input pin.
 *
 *      The open short detection circuit connects a resistor between an analog input pad
 * and either supply or ground. These resistors can be used to estimate the input
 * impedance of the circuit driving the analog input pad thereby detecting if the pad
 * is shorted or open.
 *
 * Assumptions
 *  1. ADC is in Single Ended Mode
 *  2. Sampling time is increased well above the minimum value
 *  3. Atleast 1 uS delay after configuring the OSD circuit to ADC samples
 *  4. OSD is not implemented on the CAL 1,2 inputs.
 *  5. Divider Resistence tolerances vary widely and are not used for accuracy checks
 *
 * OSD Circuit configurations
 * config   | function      | Impedance | Voltage on 5K | Voltage on 7K
 * ---------|---------------|-----------|---------------|-----------
 * 0        | Zero Scale    | 5K // 7K  | VSSA          | VSSA
 * ---------|---------------|-----------|---------------|-----------
 * 1        | Zero Scale    | 5K        | VSSA          | OPEN
 * ---------|---------------|-----------|---------------|-----------
 * 2        | Zero Scale    | 7K        | OPEN          | VSSA
 * ---------|---------------|-----------|---------------|-----------
 * 3        | Full Scale    | 5K // 7K  | VDD           | VDD
 * ---------|---------------|-----------|---------------|-----------
 * 4        | Full Scale    | 5K        | VDD           | OPEN
 * ---------|---------------|-----------|---------------|-----------
 * 5        | Full Scale    | 7K        | OPEN          | VDD
 * ---------|---------------|-----------|---------------|-----------
 * 6        | 5/12 Scale    | 5K // 7K  | VSSA          | VDD
 * ---------|---------------|-----------|---------------|-----------
 * 7        | 5/12 Scale    | 5K // 7K  | VDD           | VSSA
 * ---------|---------------|-----------|---------------|-----------
 *
 * Qualification process
 * In the example, ADCA A0 channel is configured and following algorithm is
 * used to check the A0 pin status:
 * Step 1: Configure full scale OSDETECT mode & capture ADC results(resultHi)
 * Step 2: Configure zero scale OSDETECT mode & capture ADC results(resultLo)
 * Step 3: Disable OSDETECT mode and capture ADC results(resultNormal)
 * Step 4: Determine the state of the ADC pin
 *      a. If the pin is open, resultLo would be equal to Vreflo and resultHi
 *         would be equal to Vrefhi
 *      b. If the pin is shorted to Vrefhi, resultLo should be approximately
 *         equal to Vrefhi and resultHi should be equal to Vrefhi
 *      c. If the pin is shorted to Vreflo, resultLo should be equal to
 *         Vreflo and resultHi should be approximately equal to Vreflo
 *      d. If the pin is connected to a valid signal, resultLo should be
 *         greater than osdLoLimit but less than resultNormal while resultHi
 *         should be less than osdHiLimit but greater than resultNormal
 * ----------------------------------------------------------------------------
 * Input  |  Full-Scale output     |  Zero-scale Output     | Pin Status
 * ----------------------------------------------------------------------------
 * Unknown| VREFHI                 | VREFLO                 | Open
 * VREFHI | VREFHI                 | approx. VREFHI         | Shorted to VREFHI
 * VREFLO | approx. VREFLO         | VREFLO                 | Shorted to VREFLO
 *    Vn  | Vn < resultHi < VREFHI | VREFLO < resultLo < Vn | Good
 * ----------------------------------------------------------------------------
 * Step 5: osDetectStatusVal of value greater than 4 would mean that there is
 * no pin fault.
 *      a. If osDetectStatusVal == 1, means pin A0 is OPEN
 *      b. If osDetectStatusVal == 2, means pin A0 is shorted to VREFLO
 *      c. If osDetectStatusVal == 4, means pin A0 is shorted to VREFHI
 *      d. If osDetectStatusVal == 8, means pin A0 is in GOOD/VALID state
 *      e. Any value of osDetectStatusVal > 4, means pin A0 is in VALID state
 *
 * Configurations
 *      1. The ADC0 SOC0 is configured sample on the Channel 2.
 *      2. The SOC0 Sample and hold window is set to 256 for OSD detection usecase
 *      3. ADC0 INT1 is configured for the EOC0.
 *      4. SOC0 is triggered by Software.
 *      5. OSD circuit is enabeld for ADC0, Channel 2 (see below execution)
 *
 * External Connections
 *      ADC0-SOC0 Samples on Channel 2
 *  - on AM263Px CC E2, with HSEC Dock
 *      - Feed Analog input to ADC0_AIN2 - HSEC PIN 15
 *  - on AM263Px LP
 *      - Feed Analog Input to the ADC0_AIN2 - J7 Pin 66
 *
 * Watch Variables
 *  osDetectStatusVal - OS detection status of voltage on ADC Channel.
 *  adcResult         - a digital representation of the voltage on ADC Channel.
 */



uint16_t i = 0U;
volatile uint16_t adcAResult0;
volatile uint16_t resultHi;
volatile uint16_t resultLo;
volatile uint16_t resultNormal;

/*
 Application specific limits from Vreflo to Vrefhi.
 This can change due to various factors like source
 voltage offset etc.
*/
uint16_t osdHiLimit = 3900;    // upper limit
uint16_t osdLoLimit = 50;      // lower limit


/*
 Macros for open/shorts detection circuit status. If the osDetect status is
 other than open, shorted to Vreflo and shorted to Vrefhi values (i.e > 4),
 it can be considered as a valid signal
 */
#define APP_ADC_OSDETECT_STATUS_OPEN                  1
#define APP_ADC_OSDETECT_STATUS_SHORTED_TO_VREFLO     2
#define APP_ADC_OSDETECT_STATUS_SHORTED_TO_VREFHI     4
#define APP_ADC_OSDETECT_STATUS_GOOD                  8
#define APP_ADC_OSDETECT_STATUS_UNDETERMINED          16

/* Macro for number of OSD Configurations */
#define NUM_OSD_MODES 6

uint16_t osDetectStatusVal = 0;

uint32_t gAdc0BaseAddr = CONFIG_ADC0_BASE_ADDR;
uint32_t gAdcInst = 0;
uint32_t gAdcCh = (uint32_t) ADC_CH_ADCIN2;

uint32_t gAdc0ResultBaseAddr = CONFIG_ADC0_RESULT_BASE_ADDR;

volatile uint16_t gOsdResults[NUM_OSD_MODES] = {0};  // first 3 are of zeroScale, and the next 3 are of FullScale
volatile uint16_t gAdcResult = 0;
volatile uint16_t gAverageResult = 0;

/* for OSD detection */
uint16_t performOSDetection(
    uint32_t adcBase,
    uint32_t adcResultBase,
    uint16_t channel);

void adc_open_shorts_detection_main(void *args)
{

    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC Open Shorts Detection Test Started ...\r\n");

    /* Running the OSD detection */
    osDetectStatusVal = performOSDetection(
        gAdc0BaseAddr,
        gAdc0ResultBaseAddr,
        gAdcCh
        );

    /* Printing Results */
    DebugP_log("config\t|\tResult\t|\tMean Result\t|Result without OSD on\r\n");
    DebugP_log("---------------------------------------------------------\r\n");
    for(uint32_t config = 0; config < NUM_OSD_MODES; config++)
    {
        DebugP_log("%d\t|\t%d\t|\t%d\t\t|%d\r\n", config, gOsdResults[config], gAverageResult, gAdcResult);
    }

    DebugP_log("status : ");

    if(osDetectStatusVal == APP_ADC_OSDETECT_STATUS_OPEN )
    {
        DebugP_log("APP_ADC_OSDETECT_STATUS_OPEN\r\n");
    }
    if(osDetectStatusVal == APP_ADC_OSDETECT_STATUS_SHORTED_TO_VREFLO)
    {
        DebugP_log("APP_ADC_OSDETECT_STATUS_SHORTED_TO_VREFLO \r\n");
    }
    if(osDetectStatusVal == APP_ADC_OSDETECT_STATUS_SHORTED_TO_VREFHI)
    {
        DebugP_log("APP_ADC_OSDETECT_STATUS_SHORTED_TO_VREFHI \r\n");
    }
    if(osDetectStatusVal == APP_ADC_OSDETECT_STATUS_GOOD)
    {
        DebugP_log("APP_ADC_OSDETECT_STATUS_GOOD\r\n");
    }

    /* Disabling and Clear Interrupt */
    ADC_disableInterrupt(gAdc0BaseAddr, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(gAdc0BaseAddr, ADC_INT_NUMBER1);

    DebugP_log("ADC Open Shorts Detection Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

uint16_t performOSDetection(
    uint32_t adcBase,
    uint32_t adcResultBase,
    uint16_t channel)
{
    uint32_t adcInstance = 0;
    bool shortFlag   =  true;
    bool openFlag    =  true;

    /* without open short circuit */
    SOC_enableAdcOsdChannel(gAdcInst, gAdcCh, FALSE);
    ClockP_usleep(1);

    /* Clear Interrupt */
    ADC_clearInterruptStatus(adcBase, ADC_INT_NUMBER1);

    /* Force SOC */
    ADC_forceSOC(adcBase, ADC_SOC_NUMBER0);

    /* wait for interrupt */
    while(true != ADC_getInterruptStatus(adcBase, ADC_INT_NUMBER1))
    {
        /* Do Nothing */
    }

    /* read Result */
    gAdcResult = ADC_readResult(adcResultBase, ADC_SOC_NUMBER0);

    /* Enabling the OSD Circuit for the ADC Channel */
    SOC_enableAdcOsdChannel(gAdcInst, gAdcCh, TRUE);

    for(uint32_t config = 0; config < NUM_OSD_MODES; config++)
    {
        /* Configure OSD mode */
        SOC_setAdcOsdConfig(adcInstance, config);

        /* wait for 1uS */
        ClockP_usleep(1);

        /* Clear Interrupt */
        ADC_clearInterruptStatus(adcBase, ADC_INT_NUMBER1);

        /* Force SOC */
        ADC_forceSOC(adcBase, ADC_SOC_NUMBER0);

        /* wait for interrupt */
        while(true != ADC_getInterruptStatus(adcBase, ADC_INT_NUMBER1))
        {
            /* Do Nothing */
        }

        /* read Result */
        gOsdResults[config] = ADC_readResult(adcResultBase, ADC_SOC_NUMBER0);

        /* Calculating the Average, may be used for further manual checks */
        gAverageResult += gOsdResults[config]/NUM_OSD_MODES;

    }

    /* validating */
    for(uint32_t config = 0; config < NUM_OSD_MODES; config++)
    {
        if(openFlag == true)
        {
            /* if open, on zero scale the values are closer to lo limit,
                and on full scale the values are closer to the hi limit */
            if(((config < 3) && (gOsdResults[config] <= osdLoLimit))
                ||
              ((config >= 3) && (gOsdResults[config] >= osdHiLimit)))
            {
                openFlag = true;
            }
            else
            {
                openFlag = false;
            }
        }
        if(shortFlag == true)
        {
            /* if short, on zero scale or full scale, the results are
            very close to shorted value. i.e., either all are towards lo limit or
            all are towards the hi limit */
            if (((gOsdResults[0] <= osdLoLimit) && (gOsdResults[config] <= osdLoLimit))
                ||
                ((gOsdResults[0] >= osdHiLimit) && (gOsdResults[config] >= osdHiLimit)))
            {
                shortFlag = true;
            }
            else
            {
                shortFlag = false;
            }
        }
    }

    /* Disabling the OSD Circuit for the ADC Channel */
    SOC_enableAdcOsdChannel(gAdcInst, gAdcCh, FALSE);


    if(openFlag == true)
    {
        return APP_ADC_OSDETECT_STATUS_OPEN;
    }
    if(shortFlag == true)
    {
        if((gOsdResults[0] <= osdLoLimit))
        {
            return APP_ADC_OSDETECT_STATUS_SHORTED_TO_VREFLO;
        }
        return APP_ADC_OSDETECT_STATUS_SHORTED_TO_VREFHI;
    }
    return APP_ADC_OSDETECT_STATUS_GOOD;

}


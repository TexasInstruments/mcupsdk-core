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
#include <drivers/cmpss.h>
#include <drivers/epwm.h>
#include <drivers/soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * A CMPSS example that shows the calibration of CMPSS when the negative
 * input of the comparator is driven from the compdac.
 * 
 * This example enables the CMPSSA1 COMPH comparator for calibration.
 * CMPIN1P is used to give positive input coming from the DAC and Compdac
 * is configured to provide the negative input. DAC is used to provide
 * different voltage values to CMPIN1P. Compdac is configured to provide
 * signal of 4095 and will decrement till CMPSS latch is set for 
 * that specific voltage value at positive input. Output will be the 
 * measured voltages at which CMPSS latch is set when compared with 
 * different DAC value including the compdac offset error.
 *
 * External Connections \n
 *  - Connect DAC output to CMPIN1P
 * 
 * AM263X-CC or AM263PX-CC
 *  - Connect HSEC Pin 9 to HSEC Pin 15
 * 
 * AM263X-LP or AM263PX-LP or AM261X-LP
 *  - Connect J1/J3 Pin 30 to J1/3 Pin 66
 *
 * AM261X-LP
 *  - Connect J1/J3 Pin 30 to J1/3 Pin 63
 *
 */

/* Global variables */
uint16_t gCmpssDacValue = 0;
volatile uint16_t gCmpssHiCmpStatus = 0;
/* DAC Output Voltage */
volatile uint16_t gDACValue[] = {4000,3000,2000,1000};

void cmpss_calibration(void *args)
{
    /* Set up COMP1H and DAC */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("CMPSS Calibration Test Started ...\r\n");
    DebugP_log("DAC value\tMeasured calibration voltage\r\n");

    for(int i = 0; i < 4; i++)
    {   
        gCmpssDacValue = 4095;
        
        /* Set compdac and dac value */
        CMPSS_setDACValueHigh(CONFIG_CMPSS0_BASE_ADDR, gCmpssDacValue);
        DAC_setShadowValue(CONFIG_DAC0_BASE_ADDR, gDACValue[i]);

        gCmpssHiCmpStatus = CMPSS_getStatus(CONFIG_CMPSS0_BASE_ADDR) & CMPSS_STS_HI_FILTOUT;

        /* Decrement compdac value till the COMPHSTS is set*/
        while(gCmpssHiCmpStatus == 0U)
        {
            CMPSS_setDACValueHigh(CONFIG_CMPSS0_BASE_ADDR, gCmpssDacValue);

            /*Check if COMPHSTS is set*/
            gCmpssHiCmpStatus = CMPSS_getStatus(CONFIG_CMPSS0_BASE_ADDR) & CMPSS_STS_HI_FILTOUT;
            if(gCmpssHiCmpStatus == 0)
            {
                gCmpssDacValue--;
            }
        }

        DebugP_log("%dV:\t\t%d \r\n", gDACValue[i], gCmpssDacValue);
    }

    DebugP_log("CMPSS Calibration Test Passed!!!");
    DebugP_log("All tests have Passed!!!");
}
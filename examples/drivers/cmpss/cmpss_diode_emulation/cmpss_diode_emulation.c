
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
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * Example Description
 * This example configures EPWM0,EPWM1 and EPWM2 as follows
 * All ePWM's are 25KHz, 50% duty cycle
 *
 * DAC0 is used as positive input to CMPSSA1 High and Low:
 * Reference voltage source = external
 *
 * CMPSSA1 configuration for TRIPH/TRIPL signal inputs to enter DE mode:
 *  - Positive input = DAC_OUT for both CMPSSA1 High, Low
 *  - Negative input = 1.65v from internal DAC for CMPSSA1 High
 *                     3.3v from internal DAC for CMPSSA1 Low
 * This means only CMPSSTRIPH1 is triggered when DACA_OUT > 1.65v and
 * CMPSSTRIPL1 is always Low
 *
 * Diode emulation configurations:
 *  - For all ePWM's, TripH = CMPSSTRIPH1 and TRIPL = CMPSSTRIPL1
      Since a TRIPH_OR_TRIPL triggers DE entry, only TripH is used for demo
    - EPWM0_A is set to active high and EPWM0_B set to low during DE
    - EPWM1_A is configured same as EPWM0 with re-entry delay
        - Re-entry delay = 10 EPWMSYNCPER cycles
    - EPWM2 is confiugred same as EPWM0 with DE mode disabled for the purpose
      of comparison
 *
 * External Connections \n
 *  - Connect DAC output to CMPIN1P
 *  - Connect GND to CMPIN1N
 *  - Probe EPWM2A for reference waveform
    - Probe EPWM 0A and 1A for the diode emulation enabled waveforms
 * 
 * AM263PX-CC
 *  - Connect HSEC Pin 9 to HSEC Pin 15
 *  - Connect GND to HSEC Pin 17
 *  - Probe the following on the HSEC pins
 *      - EPWM 0A : 49
 *      - EPWM 1A : 53
 *      - EPWM 2A : 50
 * 
 * AM263PX-LP
 *  - Connect J1/J3 Pin 30 to J1/3 Pin 66
 *  - Connect GND to J1/3 Pin 2
 *  - Probe the following on boosterpack
 *      - EPWM 0A : J2/4 11
 *      - EPWM 1A : J2/4 37
 *      - EPWM 2A : J2/4 39
 *
 * AM261X-LP
 *  - Connect J1/J3 Pin 30 to J1/3 Pin 63
 *  - Connect GND to J1/3 Pin 2
 *  - Probe the following on boosterpack
 *      - EPWM 0A : J7 70
 *      - EPWM 1A : J7 69
 *      - EPWM 2A : J4 40
 *
*/

#define APP_INT_IS_PULSE    (1U)

/* Global variables */
uint16_t gDacVal= 0U;


/* Get Address of DAC */
uint32_t gDacBaseAddr = CONFIG_DAC0_BASE_ADDR;


void cmpss_diode_emulation_main(void *args)
{

    /* Open drivers to open the UART driver for console 
       Set up COMP1H with diode emulation feature enabled */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("CMPSS Diode Emulation Logic Test Started ...\r\n");

    uint16_t i = 0;

    while(i<20000)
    {
        if(gDacVal >= 4095U)
        {
            gDacVal = 0U;
            i++;
        }
        DAC_setShadowValue(gDacBaseAddr,gDacVal);
        gDacVal+=50;
    }

    DebugP_log("CMPSS Diode Emulation Logic Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

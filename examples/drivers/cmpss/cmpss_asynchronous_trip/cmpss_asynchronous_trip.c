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
 * A CMPSS example that enables the CMPSS High comparator and feeds the
 * asynchronous output to GPIO and EPWM

 * This example enables the CMPSSA0 COMPH comparator and feeds the asynchronous
 * CTRIPOUTH signal to the XBAROUT0 pin and CTRIPH to EPWM0B.
 *
 * CMPSS is configured to generate trip signals to trip the EPWM signals.
 * CMPIN1P is used to give positive input and internal DAC is configured
 * to provide the negative input. Internal DAC is configured to provide a
 * signal at VDD/2. An EPWM signal is generated at EPWM0B and is configured
 * to be tripped by CTRIPOUTH.
 *
 * When a low input(VSS) is provided to CMPIN1P,
 *     - Trip signal(XBAROUT0) output is low
 *     - EPWM0B gives a PWM signal
 *
 * When a high input(higher than VDD/2) is provided to CMPIN1P,
 *     - Trip signal(XBAROUT0) output turns high
 *     - EPWM0B gets tripped and outputs as high
 *
 * External Connections \n
 *  - Give input on CMPIN1P (ControlCard HSEC Pin 12)
 *  - Outputs can be observed on
 *    - XBAROUT0 (SOC pin QSPI0_CSN1. USER_LED1 on ControlCard)
 *    - and EPWM0B (ControlCard HSEC pin 51) using an oscilloscope
 *
 */

void cmpss_asynchronous_trip(void *args)
{
    /* Set up COMP1H
       Set up ePWM0 to take CTRIPH as TRIP4 for its DC trip input
       Configure XBAROUT0 to output CTRIPOUT1H (routed through OUTPUTXBAR0) and
       EPWM0B to output CTRIPH (routed through ePWM TRIP4 and ePWM0)*/
    Drivers_open();
    Board_driversOpen();

    DebugP_log("CMPSS asynchronous trip Test Started ...\r\n");

    EPWM_disableTripZoneAdvAction(CONFIG_EPWM0_BASE_ADDR);
    /* Clear trip flags */
    EPWM_clearTripZoneFlag(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);
    int loopCount = 15;
    /* Loop indefinitely*/
    while(loopCount > 0)
    {
        /* Trip flag is set when CTRIP signal is asserted*/
        if((EPWM_getTripZoneFlagStatus(CONFIG_EPWM0_BASE_ADDR) &
            EPWM_TZ_FLAG_OST) != 0U)
        {
            loopCount--;
            DebugP_log("CTRIP signal is asserted \r\n");

            /* Wait for comparator CTRIP to de-assert*/
            while((CMPSS_getStatus(CONFIG_CMPSS0_BASE_ADDR) & CMPSS_STS_HI_FILTOUT) != 0U);

            /* Clear trip flags*/
            EPWM_clearTripZoneFlag(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_INTERRUPT |
                                   EPWM_TZ_FLAG_OST);
        }
    }
    DebugP_log("CMPSS asynchronous trip Test Passed!!!");
    DebugP_log("All tests have Passed!!!");
}


/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
#include <kernel/dpl/CycleCounterP.h>
#include <drivers/epwm.h>
#include <drivers/gpio.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * An EPWM example to detect occurrence of a trip event in a configured time window.
 * The window is configured by MIN and MAX values configured in MINMAX register set.
 *
 * Purpose of this window is to detect the occurrence of such edge. If no such edge occurs, this module will generate a trip event as well as
 * interrupt configurable by user.
 *
 * In this example EPWM1_A is used for Digital compare input on which logic is performed.
 * EPWM0_A is used as the gating signal to Min/max logic. (Routed to PWM XBar from GPIO)
 *
 * If an edge is not detected on EPWM0 in the MINMAX window,
 *     - EPWM1 is tripped
 *
 * External Connections \n
 *
 *  ##AM263x-cc
 *  - Connect EPWM0_A(HSEC pin 49) to GPIO15(HSEC pin 81)
 *  - Output can be observed from EPWM1_A(HSEC pin 51)
 *  - CAPEVT TripOut can be observed from XBAROUT3(HSEC pin 75)
 *
 */

/* Globals */
uint32_t gEpwm1BaseAddr;
uint32_t gEpwm2BaseAddr;

static bool isCAPEVTSet(uint32_t base);

void epwm_dccap_main(void *args)
{
    DebugP_log("\r\nEPWM Capture logic test\r\n");

    Drivers_open();
    Board_driversOpen();

    /* Get Address of ePWM */
    uint32_t epwmInstance1 = 1;
    uint32_t epwmInstance2 = 0;
    gEpwm1BaseAddr = CSL_CONTROLSS_G0_EPWM0_U_BASE + epwmInstance1*(0x1000);
    gEpwm2BaseAddr = CSL_CONTROLSS_G0_EPWM0_U_BASE + epwmInstance2*(0x1000);

    //Gate the EPWM clock
    SOC_setMultipleEpwmTbClk((1U << epwmInstance1) | (1U << epwmInstance2), FALSE);

    //Start TimeBase counters
    EPWM_setTimeBaseCounterMode(gEpwm2BaseAddr, EPWM_COUNTER_MODE_UP);
    EPWM_setTimeBaseCounterMode(gEpwm1BaseAddr, EPWM_COUNTER_MODE_UP);

    //Disable Capture Trip inputs
    EPWM_disableCaptureTripCombinationInput(gEpwm1BaseAddr, EPWM_DC_COMBINATIONAL_TRIPIN1, EPWM_CAPTURE_GATE);
    EPWM_disableCaptureTripCombinationInput(gEpwm1BaseAddr, EPWM_DC_COMBINATIONAL_TRIPIN1, EPWM_CAPTURE_INPUT);

    //Enable the Capture event
    EPWM_enableCaptureInEvent(gEpwm1BaseAddr);

    //Select Polarity for Capture Input
    EPWM_invertCaptureInputPolarity(gEpwm1BaseAddr, 0);

    //Select Polarity for Capture Gate Input
    EPWM_configCaptureGateInputPolarity(gEpwm1BaseAddr, 0);

    // Disable all Trip Zone signals
    EPWM_disableTripZoneSignals(gEpwm1BaseAddr, EPWM_TZ_SIGNAL_CBC1 |
                                 EPWM_TZ_SIGNAL_CBC2 |
                                 EPWM_TZ_SIGNAL_CBC3 |
                                 EPWM_TZ_SIGNAL_CBC4 |
                                 EPWM_TZ_SIGNAL_CBC5 |
                                 EPWM_TZ_SIGNAL_CBC6 |
                                 EPWM_TZ_SIGNAL_DCAEVT2 |
                                 EPWM_TZ_SIGNAL_DCBEVT2 |
                                 EPWM_TZ_SIGNAL_OSHT1 |
                                 EPWM_TZ_SIGNAL_OSHT2 |
                                 EPWM_TZ_SIGNAL_OSHT3 |
                                 EPWM_TZ_SIGNAL_OSHT4 |
                                 EPWM_TZ_SIGNAL_OSHT5 |
                                 EPWM_TZ_SIGNAL_OSHT6 |
                                 EPWM_TZ_SIGNAL_DCAEVT1 |
                                 EPWM_TZ_SIGNAL_DCBEVT1);

    //Configure CAPEVT as one-shot trip source for this ePWM module
    EPWM_disableTripZone2Signals(gEpwm1BaseAddr, EPWM_TZ2_SIGNAL_CAPEVT_CBC);
    EPWM_enableTripZone2Signals(gEpwm1BaseAddr, EPWM_TZ2_SIGNAL_CAPEVT_OST);

    //Get CAPEVT trip out
    // App_pinmuxConfig();
    EPWM_enableTripZoneOutput(gEpwm1BaseAddr, EPWM_TZ_SELECT_TRIPOUT_CAPEVT);

    //Set both EPWM counters to 0
    EPWM_setTimeBaseCounter(gEpwm1BaseAddr, 0);
    EPWM_setTimeBaseCounter(gEpwm2BaseAddr, 0);

    //Ungate the EPWM clock
    SOC_setMultipleEpwmTbClk((1U << epwmInstance1) | (1U << epwmInstance2), TRUE);

    uint32_t iterations = 15;
    while(iterations)
    {
        DebugP_log("\r\nIteration: %u\r\n", 16 - iterations);

        //TEST1 - Set minmax with edge in it
        if(iterations == 12)
        {
            EPWM_setXMINMAXRegValue(gEpwm1BaseAddr, EPWM_XMIN_ACTIVE, 0);
            EPWM_setXMINMAXRegValue(gEpwm1BaseAddr, EPWM_XMAX_ACTIVE, 400);

            if(isCAPEVTSet(gEpwm1BaseAddr)){
                //tripped
                DebugP_assert(FALSE);
            }
        }
        //TEST2 - change minmax to exclude edge
        if(iterations == 8)
        {
            //Clear TZFLG[CAPEVT], needed to end the trip zone
            EPWM_clearTripZoneFlag(gEpwm1BaseAddr, EPWM_TZ_FLAG_CAPEVT);
            //Clear trip zone flag for one shot trip (don't need to do this for CBC)
            EPWM_clearTripZoneFlag(gEpwm1BaseAddr, EPWM_TZ_FLAG_OST);

            //Change MINMAX window to get no edge in it (to start the trip)
            EPWM_setXMINMAXRegValue(gEpwm1BaseAddr, EPWM_XMIN_ACTIVE, 500);
            EPWM_setXMINMAXRegValue(gEpwm1BaseAddr, EPWM_XMAX_ACTIVE, 900);

            ClockP_usleep(2000);
            if(isCAPEVTSet(gEpwm1BaseAddr) == 0){
                //no trip
                DebugP_assert(FALSE);
            }
        }
        //TEST3 - change input to get edge in minmax
        if(iterations == 4)
        {
            EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP1_ACTIVE, 700); //Set output pins to high at TBCTR
            EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP2_ACTIVE, 1100);//Set output pins to low at TBCTR

            // Clear TZFLG[CAPEVT], needed to end the trip zone
            EPWM_clearTripZoneFlag(gEpwm1BaseAddr, EPWM_TZ_FLAG_CAPEVT);
            // Clear trip zone flag for one shot trip (don't need to do this for CBC)
            EPWM_clearTripZoneFlag(gEpwm1BaseAddr, EPWM_TZ_FLAG_OST);

            EPWM_setXMINMAXRegValue(gEpwm1BaseAddr, EPWM_XMIN_ACTIVE, 500);
            EPWM_setXMINMAXRegValue(gEpwm1BaseAddr, EPWM_XMAX_ACTIVE, 900);

            ClockP_usleep(2000);
            if(isCAPEVTSet(gEpwm1BaseAddr)){
                //tripped
                DebugP_assert(FALSE);
            }
        }

        DebugP_log("PWMXBAR_STATUS = 0x%08x\r\n", SOC_xbarGetPWMXBarOutputSignalStatus(CSL_CONTROLSS_PWMXBAR_U_BASE));
        DebugP_log("DCCAP = %d\r\n", EPWM_getDigitalCompareCaptureCount(gEpwm1BaseAddr));
        DebugP_log("DCCAP Status = %d\r\n", EPWM_getDigitalCompareCaptureStatus(gEpwm1BaseAddr));
        DebugP_log("DCCAPCTL = 0x%04x\r\n", HW_RD_REG16(gEpwm1BaseAddr + CSL_EPWM_DCCAPCTL));
        iterations--;
    }

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static bool isCAPEVTSet(uint32_t base)
{
    return (EPWM_getTripZoneFlagStatus(base) & EPWM_TZ_FLAG_CAPEVT) != 0;
}
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
 * Introduction:
 *      The EPWM Edge Detection in DCCAP is a feature to monitor the trip input
 * to be present in a given window. A typical usecase would when the EPWMs are
 * used for controlling a motor, then the expected high current pushes in the
 * initial switching time is expected and if they are not present, then,
 * there is some issue with system and the EPWMs need to to be tripped.
 *
 * Example Description
 *      This example showcases the configurations needed to use the Edge
 * Detection feature in the DCCAP to detect occurrence of a trip event in
 * a configured time window. The window is configured by MIN and MAX values
 * configured in MINMAX register set. Purpose of this window is to detect
 * the occurrence of such edge. If no such edge occurs, this module will
 * generate a trip event as well as interrupt configurable by user.
 *
 * EPMW0_A to detect the trip within a MINMAX window and trip its EPWMxA output
 * EPMW1_A waveform is used as a trip input.
 *
 * Configurations
 * 1. EPWM0
 *      - DCCAP is enabled, trip inputs are configured for Trip1.
 *      - waveform A/B are configured (in the example) to match the min-max window
 *        for observing.
 *      - IN Trip zone, CAPEVT is configured for CBC source and Trip action on output
 *        A to low Action when trip occured.
 *      - XCMP is enbaled for this feature usage.
 *
 * 2. EPWM1
 *      - XCMP is enabled (not a necessary requirement)
 *      - generates a waveform, that goes high on counter = 100, goes low on counter = 1000,
 *        for a period of 2000
 *
 * 3. PWMXbar 0 to take GPIO45 via Inputxbar (a common pin that has EPWM 1A output)
 *
 * Note
 *      the EPWM1A is routed internally via GPIO. if wish to use a different/external
 * trip input, then,
 *      1. remove EPWM 1A configuration (either pinmux or all configurations from syscfg)
 *      2. Add GPIO45 as a input pin.
 *      3. connect external trip input to GPIO45
 *
 * External Connections
 * 1. AM263Px-cc with HSEC dock connected
 *      - EPWM0A can be observed on HSEC PIN 49
 *      - EPWM0B can be observed on HSEC PIN 51
 *      - EPWM1A (Trip Input) can be observed on HSEC PIN 53.
 *      - CAPEVT TripOut can be observed from XBAROUT8(HSEC pin 75)
 *
 */

/* Globals */
uint32_t gEpwm0BaseAddr;
uint32_t gEpwm1BaseAddr;

uint32_t minValue = 0;
uint32_t maxValue = 0;

int32_t status = SystemP_SUCCESS;

#define INPUT_WAVEFORM_RISING_EDGE_COUNTER_VALUE (100)  /* as defined in the syscfg for EPWM1 A */

static bool App_isCAPEVTSet(uint32_t base);




void epwm_dccap_main(void *args)
{
    DebugP_log("EPWM Edge Detection Test Started ...\r\n");

    Drivers_open();
    Board_driversOpen();

    /* Get Address of ePWM */
    gEpwm0BaseAddr = CONFIG_EPWM0_BASE_ADDR;
    gEpwm1BaseAddr = CONFIG_EPWM1_BASE_ADDR;

    /*
    Making the EPWM synchronous.
    Note:
        the type 4 sync in feature (SYNCIN, SYNCOUT from TB Submodule) is not
        supported when the xcmp mode is enabled.
    */

    /* Gate the EPWM clock for configurations */
    SOC_setMultipleEpwmTbClk(gEpwmTbClkSyncEnableMask, FALSE);

    /* Set both EPWM counters to 0 */
    EPWM_setTimeBaseCounter(gEpwm0BaseAddr, 0);
    EPWM_setTimeBaseCounter(gEpwm1BaseAddr, 0);

    /* Start TimeBase counters */
    EPWM_setTimeBaseCounterMode(gEpwm0BaseAddr, EPWM_COUNTER_MODE_UP);
    EPWM_setTimeBaseCounterMode(gEpwm1BaseAddr, EPWM_COUNTER_MODE_UP);

    /*
    enabling the CAPEVT in trip out for observing on the pin.
    Note:
        for a usecase, if this event is need to route to external devices, then can be used.
        Otherwise can be ignored
    */
    EPWM_enableTripZoneOutput(gEpwm0BaseAddr, EPWM_TZ_SELECT_TRIPOUT_CAPEVT);

    /* Ungate the EPWM clock after the configurations are done */
    SOC_setMultipleEpwmTbClk(gEpwmTbClkSyncEnableMask, TRUE);

    /* wait for sometime to observe the EPWM Waveforms */
    ClockP_sleep(2);

    /* Enabling the DCCAP Capture */
    EPWM_enableDigitalCompareCounterCapture(gEpwm0BaseAddr);

    for(minValue = 10; minValue < 200; minValue +=50)
    {
        for(maxValue = minValue+50; maxValue < 250; maxValue +=100)
        {
            volatile bool CapevtStatus;

            DebugP_log("minValue : %d\tmaxValue : %d\r\n", minValue, maxValue);
            DebugP_log("INPUT_WAVEFORM_RISING_EDGE_COUNTER_VALUE : %d\r\n", INPUT_WAVEFORM_RISING_EDGE_COUNTER_VALUE);

            /* Clearing any CAPEVT set before */
            EPWM_clearTripZoneFlag(gEpwm0BaseAddr, EPWM_TZ_FLAG_CAPEVT);
            /*
            if One-shot trip source is tripping the EPWM wave then the following should be used
            EPWM_clearTripZoneFlag(gEpwm0BaseAddr, EPWM_TZ_FLAG_OST);
            */

            /*
            updating the EPWM A, B waveforms to match the MINMAX window for observability.
            This can be skipped for a normal usecase
            */
            EPWM_setXCMPRegValue(gEpwm0BaseAddr, EPWM_XCMP1_ACTIVE, minValue);
            EPWM_setXCMPRegValue(gEpwm0BaseAddr, EPWM_XCMP2_ACTIVE, maxValue);
            EPWM_setXCMPRegValue(gEpwm0BaseAddr, EPWM_XCMP5_ACTIVE, minValue);
            EPWM_setXCMPRegValue(gEpwm0BaseAddr, EPWM_XCMP6_ACTIVE, maxValue);

            /* Updating the MINMAX register */
            EPWM_setXMINMAXRegValue(gEpwm0BaseAddr, EPWM_XMIN_ACTIVE, minValue);
            EPWM_setXMINMAXRegValue(gEpwm0BaseAddr, EPWM_XMAX_ACTIVE, maxValue);

            // EPWM_forceXLoad(gEpwm0BaseAddr);

            EPWM_clearTripZoneFlag(gEpwm0BaseAddr, EPWM_TZ_FLAG_CAPEVT);
            /*
            if One-shot trip source is tripping the EPWM wave then the following should be used
            EPWM_clearTripZoneFlag(gEpwm0BaseAddr, EPWM_TZ_FLAG_OST);
            */

            ClockP_usleep(25);
            CapevtStatus = App_isCAPEVTSet(gEpwm0BaseAddr);

            DebugP_log("CAPEVT Status = %d\r\n", CapevtStatus);
            DebugP_log("DCCAP Status = %d\r\n\r\n", EPWM_getDigitalCompareCaptureStatus(gEpwm0BaseAddr));

            /* validation */
            if((minValue >= INPUT_WAVEFORM_RISING_EDGE_COUNTER_VALUE) ||
                (maxValue <= INPUT_WAVEFORM_RISING_EDGE_COUNTER_VALUE))
            {
                /*
                this is the case where the CAPEVT should have set.
                Because, the edge is outside the MINMAX window */

                if(CapevtStatus != true)
                {
                    status = SystemP_FAILURE;
                    break;
                }
            }
            else
            {
                /* the edge is in the MINMAX window. this is where the CAPEVT should not have set */
                if(CapevtStatus == true)
                {
                    status = SystemP_FAILURE;
                    break;
                }
            }
        }
        if(status == SystemP_FAILURE)
        {
            break;
        }
    }

    /* exiting */
    EPWM_disableDigitalCompareCounterCapture(gEpwm0BaseAddr);
    EPWM_disableTripZoneOutput(gEpwm0BaseAddr, EPWM_TZ_SELECT_TRIPOUT_CAPEVT);


    if(status == SystemP_SUCCESS)
    {
        DebugP_log("EPWM Edge Detection Test Passed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
        DebugP_log("EPWM Edge Detection Test Failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

static bool App_isCAPEVTSet(uint32_t base)
{
    return ((EPWM_getTripZoneFlagStatus(base) & EPWM_TZ_FLAG_CAPEVT) != 0U);
}

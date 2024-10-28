
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

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * Example Description
 *  This example configures EPWM0, EPWM1, epEPWM2 to produce signal of desired
 * frequency and duty. It also configures phase between the configured
 * modules.
 *
 * Signal of 10kHz with duty of 0.5 is configured on ePWMxA & ePWMxB
 * with ePWMxB inverted. Also, phase of 120 degree is configured between
 * EPWM0 to EPWM2 (EPWM2 to EPWM4 in case of AM261x-LP) signals.
 *
 * During the test, monitor EPWM0, EPWM1, and/or EPWM2 (EPWM2, EPWM3 and/or
 * EPWM4 in case of AM261x-LP) outputs on an oscilloscope.
 *
 * On AM263x CC/ AM263Px CC with HSEC Dock,
 * Probe the following on the HSEC pins
 *  - EPWM 0A/0B : 49 / 51
 *  - EPWM 1A/1B : 53 / 55
 *  - EPWM 2A/2B : 50 / 52
 *
 * On AM263x LP/ AM263Px LP,
 * Probe the following on boosterpack
 *  - EPWM 0A/0B : J4 11 / J8 59
 *  - EPWM 1A/1B : J2 37 / J2 38
 *  - EPWM 2A/2B : J2 39 / J2 40
 * 
 * On AM261x LP,
 * Probe the following on boosterpack
 *  - EPWM 2A/2B : J2 40 / J8 39
 *  - EPWM 3A/3B : J2 38 / J2 37
 *  - EPWM 4A/4B : J2 36 / J2 35
 */

#define SYSCLK_FREQ (200*1000*1000U) // 200 MHz

EPWM_SignalParams pwmSignal ={
        10000,                          // Desired Signal Frequency(in Hz)
        0.5f,                           // Desired ePWMxA Signal Duty
        0.5f,                           // Desired ePWMxB Signal Duty
        true,                           // Invert ePWMxB Signal if true
        SYSCLK_FREQ,             // SYSCLK Frequency(in Hz)
        EPWM_COUNTER_MODE_UP_DOWN,      // Time Base Counter Mode
        EPWM_CLOCK_DIVIDER_1,           // Time Base Counter Clock Divider
        EPWM_HSCLOCK_DIVIDER_1          // Time Base Counter HS Clock Divider
    };

uint32_t gEpwm0Base = CONFIG_EPWM0_BASE_ADDR;
uint32_t gEpwm1Base = CONFIG_EPWM1_BASE_ADDR;
uint32_t gEpwm2Base = CONFIG_EPWM2_BASE_ADDR;




void configurePhase(uint32_t base, uint32_t mainBase, uint16_t phaseVal);

void epwm_configure_signal_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Check the syscfg for configurations */

    /* Note that the Sync Mechanism is added for the waveform viewability.
    it is not a pre-requisite for the Chopper functionality.*/

    DebugP_log("EPWM Configure Signal Test Started ...\r\n");
    DebugP_log("EPWM Configure Signal Example runs for 5 Secs \r\n");

    /* Disabling tbclk sync for EPWMs 0-2 for configurations */
    SOC_setMultipleEpwmTbClk(gEpwmTbClkSyncEnableMask, FALSE);

    /* Note that syscfg has these modules added for pinmux and
    other SOC Configurations but not for the EPWM signal configurations */

    /* Configuring the EPWM signals as per EPWM_SignalParams */
    EPWM_configureSignal(gEpwm0Base, &pwmSignal);
    EPWM_configureSignal(gEpwm1Base, &pwmSignal);
    EPWM_configureSignal(gEpwm2Base, &pwmSignal);

    /* Disabling the Phase shift for the EPWM 0.
    and setting it as main PWM to sync others */
    EPWM_disablePhaseShiftLoad(gEpwm0Base);
    EPWM_setPhaseShift(gEpwm0Base, 0U);
    EPWM_enableSyncOutPulseSource(gEpwm0Base, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    /* Configure phase shift for EPWM1 & 2 */
    configurePhase(gEpwm1Base, gEpwm0Base, 120);
    configurePhase(gEpwm2Base, gEpwm0Base, 240);

    EPWM_enablePhaseShiftLoad(gEpwm1Base);
    EPWM_enablePhaseShiftLoad(gEpwm2Base);

    /* Enabling tbclk sync for EPWMs 0-2 after configurations */
    SOC_setMultipleEpwmTbClk(gEpwmTbClkSyncEnableMask, TRUE);

    ClockP_sleep(5);

    DebugP_log("EPWM Configure Signal Test Passed!!\r\n");
    DebugP_log("All Tests have Passed!!");

    Board_driversClose();
    Drivers_close();
}


/* configurePhase - Configure ePWMx Phase */
void configurePhase(uint32_t base, uint32_t mainBase, uint16_t phaseVal)
{
    uint32_t readPrdVal, phaseRegVal;

    /* Read Period value to calculate value for Phase Register */
    readPrdVal = EPWM_getTimeBasePeriod(mainBase);

    /*
    Phase calculation for the UP_DOWN_COUNT Mode.
    this can differ based on the counter mode set by user
    E.g. for Counter modes up or down,
        phaseRegVal = (readPrdVal * phaseVal) / 360U;
    */
    phaseRegVal = (2U * readPrdVal * phaseVal) / 360U;

    EPWM_selectPeriodLoadEvent(base, EPWM_SHADOW_LOAD_MODE_SYNC);
    EPWM_setPhaseShift(base, phaseRegVal);
    EPWM_setTimeBaseCounter(base, phaseRegVal);
}


/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

/* Description:
 * This example demonstrates how to measure the DSP clock frequency on AM273x devices.
 * It configures the OBSCLKOUT_CLK_SRC_SEL to DPLL_DSP_HSDIV0_CLKOUT1 
 * Prints the configured clock frequency of the sample to the console.
 * It generates the DSP clock with the frequency 550/(OBSCLKOUT_DIV_VAL) and can be viewed on oscilloscope by probing R299 resistor.
 */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

void clock_measurement_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

#if !defined(_TMS320C6X)
    uint32_t r5ClkFreq;
    /* Get DSP Clock and output to the console */
    r5ClkFreq = SOC_rcmGetR5Clock();
    DebugP_log("MSS Frequency is %d Hz, %d MHz\r\n", r5ClkFreq, SOC_RCM_FREQ_HZ2MHZ(r5ClkFreq));
#elif !defined(CPU_DSS_CM4)
    uint32_t dspClkFreq;
    /* Get DSP Clock and output to the console */
    dspClkFreq = SOC_rcmGetDspClock();
    DebugP_log("DSP Frequency is %d Hz, %d MHz\r\n", dspClkFreq, SOC_RCM_FREQ_HZ2MHZ(dspClkFreq));
#endif
    ClockP_sleep(2);

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MSS_TOP_RCM, 0);
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_RCM, 0);

    /* pointer to the base address of the MSS TOP RCM region */
    CSL_mss_toprcmRegs* ptrTopRcm = (CSL_mss_toprcmRegs*)(0x2140000U);

    CSL_REG32_FINS_RAW(&ptrTopRcm->OBSCLKOUT_CLK_SRC_SEL, CSL_MSS_TOPRCM_OBSCLKOUT_CLK_SRC_SEL_OBSCLKOUT_CLK_SRC_SEL_CLKSRCSEL_MASK, 0, 0x222); // src= DPLL_DSP_HSDIV0_CLKOUT1
    CSL_REG32_FINS_RAW(&ptrTopRcm->OBSCLKOUT_DIV_VAL, CSL_MSS_TOPRCM_OBSCLKOUT_DIV_VAL_OBSCLKOUT_DIV_VAL_CLKDIV_MASK, 0, 0x444); // 550/5 = 110 MHz

    Board_driversClose();
    Drivers_close();
    while(1);
}

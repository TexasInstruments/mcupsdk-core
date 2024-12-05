/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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

//
// Included Files
//
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * ECAP APWM Phase Shift Feature.
 * 
 * This example showcases the phase shift feature of the ECAP.
 *
 * Configurations,
 * 1. EPWM and 2 ECAPs (in APWM mode) are configured for period of 5000 SYSCLKs
 * 2. ECAPs are configured for Duty of 50% Active High PWM.
 * 3. ECAP 0 has phase shift value of 0, where are the ECAP1 has phase shift value of 2500.
 * 4. Output Xbars are used for routing the ECAP APWM output
 * 
 * External connections 
 * --------------------
 * Observe ECAP outputs on the Outputxbar instances
 * - AM263x
 *      - CC E2, outputxbar 7,8 showcase ecap0,1 apwm outs respectively
 *        on HSEC pins 87 and 89
 *      - LP, outputxbar 7,8 showcase ecap0,1 apwm outs respectively
 * - AM263Px
 *      - CC E2, outputxbar 7,8 showcase ecap0,1 apwm outs respectively
 *        on HSEC pins 87 and 89
 *      - LP, outputxbar 7,8 showcase ecap0,1 apwm outs respectively
 *        on J5.49, J5.50
 * - AM261x
 *      - LP, outputxbar 1,4, showcase ecap 0,1 apwm outs respectively
 *        on J4.17, J4.12
 * 
*/


void ecap_apwm_phase_shift_main(void *args)
{

    Drivers_open();
    Board_driversOpen();

    DebugP_log("ECAP APWM Phase Shift Test Started ...\r\n");
    ECAP_startCounter(CONFIG_ECAP0_BASE_ADDR);
    ECAP_startCounter(CONFIG_ECAP1_BASE_ADDR);

    ClockP_sleep(3);

    DebugP_log("ECAP APWM Phase Shift Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}
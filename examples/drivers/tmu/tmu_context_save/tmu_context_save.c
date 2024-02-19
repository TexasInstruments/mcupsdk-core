
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



#include <stdint.h>
#include <mathlib/trig/ti_tmu_trig.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/am263px/cslr_mss_ctrl.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/* # Example Description

 The example uses demonstartes the Context Save and Restore feature of TMU. In this example, the
 background/main task makes use of TMU sin operation, while that operation is happening an interrupt
 gets registered and enabled. The ISR for this interrupt also makes use of TMU operations, however
 using the context save feature we can store the result value obtained from the main function and once
 the ISR routine is completed, it can restore the inital result value.

*/

static HwiP_Object gTMUHwiObject;
volatile uint32_t gTMUOperationsCount = 0;
volatile uint32_t loopCount = 0;
volatile float trigVal2;
static void App_tmuISR(void *args);

#ifndef PI
#define PI                  3.14159265358979323846f
#endif

#ifndef ReciprocalOf2PI
#define ReciprocalOf2PI                0.159154943091895335768f
#endif

#ifndef TMUComputations
#define TMUComputations               10u
#endif

#ifndef TestValue
#define TestValue                6.195221f
#endif

#define HWREG(x)                                                            \
        (*((volatile uint32_t *)(x)))
#define HWREGH(x)                                                           \
        (*((volatile uint16_t *)(x)))

typedef union {
    float float_value;
    uint32_t int_value;
} float_reg;

void tmu_context_save_main()
{

    Drivers_open();
    Board_driversOpen();

    DebugP_log("\nTMU Context Save and Restore Test Started ...\r\n");
    DebugP_log("TMU Context Save and Restore Example runs for 10 Secs \r\n");

    int32_t  status, i;
    HwiP_Params  hwiPrms;
    float_reg in_angle, result;
    float trigVal, trigVal2;

    /* Compute sin value of input TestValue and store it in trigVal */
    in_angle.float_value = TestValue * ReciprocalOf2PI;

    HWREG(CSL_MSS_TMU_BASE  + CSL_TMU_SINPUF32_R0) = in_angle.int_value;
    asm("dmb");
    result.int_value = (float)HWREG(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R0);
    trigVal = result.float_value;

    /* Register software interrupt and it's parameters */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_INTR_SW_IRQ;
    hwiPrms.priority    = 0;
    hwiPrms.callback    = &App_tmuISR;
    status              = HwiP_construct(&gTMUHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Enable the software interrupt number of times */
    for(i = 0; i<TMUComputations; i++)
    {
        HWREG(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_R5SS0_CORE0_SW_INT) = 1;
        ClockP_sleep(1);
    }

    result.int_value = (float)HWREG(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R0);
    trigVal2 = result.float_value;

    /* Print result value that was obtained before interrupt occured and result value that is restored after interrupt occurs */
    DebugP_log("Value in result register before interrupts occur : %f \r\n", trigVal);
    DebugP_log("Value in result register after interrupts occur : %f \r\n", trigVal2);

    if(trigVal == trigVal2)
    {
        DebugP_log("\r\nTMU Context Save and Restore Test Passed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
       DebugP_log("TMU Context Save and Restore Test Failed!!");
    }

    Board_driversClose();
    Drivers_close();

}

static void App_tmuISR(void *args)
{
    float input_value;

    /* Writes a 1 to the context save register which will initiate context save of all result registers and store their values in the corresponding context save registers */
    ISR_TMU_CONTEXT_SAVE;

    float testStep = 2 * PI / (float) TMUComputations;
    if(loopCount < TMUComputations)
    {
        input_value = (float) loopCount * testStep;
        trigVal2 = ti_tmu_cos_pu(input_value);
        loopCount++;
    }
    else
    {
       HWREG(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_R5SS0_CORE0_SW_INT) = 0;
    }

    /* Writes a 1 to the context restore register which will initiate context restore of all result registers */
    ISR_TMU_CONTEXT_RESTORE;

}

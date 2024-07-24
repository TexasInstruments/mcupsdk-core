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
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/SemaphoreP.h>

SemaphoreP_Object gOverflowSemObj;

void OverFlowCallbackUser(GPTIMER_Handle handle)
{
    /* Post Semaphore */
    SemaphoreP_post(&gOverflowSemObj);
}

void gp_timer_overflow_callback_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    DebugP_log("GP Timer Overflow Callback Test Started ...\r\n");
    SemaphoreP_constructBinary(&gOverflowSemObj, 0);

    /* Set Tiemr Count to a high value so that overflow can happen sooner */
    GPTIMER_setCount(gGpTimerHandle[0], 0xFF000000U);
    /* Start the Timer */
    GPTIMER_start(gGpTimerHandle[0]);
    /* Pend Semaphore, Wait for Post in Callback */
    SemaphoreP_pend(&gOverflowSemObj, SystemP_WAIT_FOREVER);
    DebugP_log("Overflow Interrupt triggered !!\r\n");

    /* Stop the Timer */
    GPTIMER_stop(gGpTimerHandle[0]);

    /* Clean Up */
    SemaphoreP_destruct(&gOverflowSemObj);
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

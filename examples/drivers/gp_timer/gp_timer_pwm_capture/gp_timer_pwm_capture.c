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

uint32_t ticks = 0U;
uint32_t ticksUpdateFlag = 0U;
SemaphoreP_Object gCaptureSemObj;

void CaptureCallbackUser(GPTIMER_Handle handle)
{
    uint32_t captureVal1, captureVal2;
    captureVal1 = GPTIMER_getTimerCaptureVal1(handle);
    captureVal2 = GPTIMER_getTimerCaptureVal2(handle);
    ticks = captureVal2-captureVal1;
    SemaphoreP_post(&gCaptureSemObj);
}

void gp_timer_pwm_capture_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    DebugP_log("GP Timer PWM Capture Test Started ...\r\n");
    SemaphoreP_constructBinary(&gCaptureSemObj, 0);

    /* To obtain the desired wave form, start the counter at 0xFFFFFFFE value */
    GPTIMER_setCount(gGpTimerHandle[0], 0xFFFFFFFEU);
    /* Start the timer which is responsible for captureing the Pulse */
    GPTIMER_start(gGpTimerHandle[1]);
    /* Start the Timer which is responsible for generating PWM */
    GPTIMER_start(gGpTimerHandle[0]);
    /* Wait until the flag is set */
    /* Pend Semaphore, Wait for Post in Callback */
    SemaphoreP_pend(&gCaptureSemObj, SystemP_WAIT_FOREVER);
    /* Stop The Counter */
    GPTIMER_stop(gGpTimerHandle[0]);
    GPTIMER_stop(gGpTimerHandle[1]);

    DebugP_log("Ticks between Rising and falling edge : %d \r\n", ticks);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

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
 *  "AS IS" AND ANY EXPgResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

#include "benchmarkdemo.h"
#include "ti_drivers_open_close.h"


App_TimerStats gAppTimerStats = {0L, 0L, 0, 0, 0L, 10U };

/* Timer tick function called at user defined frequency */
void App_timerCallback(void *args)
{
    uint32_t latency = 0;
    uint32_t reloadVal;
    uint32_t curVal;

    /* compute the timer interrupt latency */
    curVal = TimerP_getCount(gTimerBaseAddr[CONFIG_TIMER_BENCHMARKDEMO]);
    reloadVal = TimerP_getReloadCount(gTimerBaseAddr[CONFIG_TIMER_BENCHMARKDEMO]);

    if(gAppTimerStats.isrCount >= gAppTimerStats.initSkipCount)
    {
        if (curVal>=reloadVal)
        {
            latency = (curVal-reloadVal)*(1000000000/CONFIG_TIMER_BENCHMARKDEMO_INPUT_CLK_HZ);
        }
        else
        {
            latency = gAppTimerStats.avg;
        }
        if (latency > gAppTimerStats.max)
        {
            gAppTimerStats.max = latency;
        }
    }
    gAppTimerStats.isrCount++;

    /* compute average timer interrupt latency */
    gAppTimerStats.total += latency;
    gAppTimerStats.avg = (uint32_t)(gAppTimerStats.total/gAppTimerStats.isrCount);
}

void App_timerResetStats()
{
    gAppTimerStats.isrCount = 0L;
    gAppTimerStats.isrCountOld = 0L;
    gAppTimerStats.max = 0;
    gAppTimerStats.avg = 0;
    gAppTimerStats.total = 0L;
}

void App_timerSetHz(uint32_t freq)
{
    TimerP_Params timerParams;

    TimerP_Params_init(&timerParams);
    timerParams.inputPreScaler = CONFIG_TIMER_BENCHMARKDEMO_INPUT_PRE_SCALER;
    timerParams.inputClkHz     = CONFIG_TIMER_BENCHMARKDEMO_INPUT_CLK_HZ;
    timerParams.periodInUsec   = (1000000/freq);
    timerParams.oneshotMode    = 0;
    timerParams.enableOverflowInt = 1;
    TimerP_setup(gTimerBaseAddr[CONFIG_TIMER_BENCHMARKDEMO], &timerParams);
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER_BENCHMARKDEMO]);
    App_timerResetStats();
}

uint32_t App_timerHasExpired()
{
    uint32_t hasExpired = 0;

    if (gAppTimerStats.isrCount>gAppTimerStats.isrCountOld)
    {
        gAppTimerStats.isrCountOld++;
        hasExpired = 1;
    }
    return hasExpired;
}


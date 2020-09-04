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

#include <string.h>
#include <drivers/soc.h>
#include "benchmarkdemo.h"
#include "ti_drivers_open_close.h"

/* max number of options across all apps */
#define APP_MAX_OPTIONS     (4U)

typedef struct
{
    /* Input: application number (0-5: 0: none, 1:cfft, 2:fir, 3:foc, 4:pid, 5:ADC/ePWM) */
    int32_t app;
    /* Input: application running frequency (0-4: 0: none, 1:8Khz, 2:16Khz, 3:32Khz, 4:50Khz) */
    int32_t freq;
    /* Input: modification flag (0/1) */
    int32_t mod_flag;
} App_Input;

typedef struct
{
    /* Output: current CPU load (0-100) */
    int32_t cur;
    /* Output: average CPU laod (0-100) */
    int32_t ave;
    /* Output: max CPU laod (0-100) */
    int32_t max;
} App_CpuLoad;

typedef struct
{
    /* Output: average timer interrupt latency (0-10000 us) */
    int32_t ave;
    /* Output: max timer interrupt latency (0-10000 us) */
    int32_t max;
} App_IntLatency;

typedef struct
{
    /* Output: average cycle count per loop (0-2^32) */
    int32_t ave;
    /* Output: max cycle count per loop (0-2^32) */
    int32_t max;
} App_CycleCountPerLoop;

typedef struct
{
    /* Output: CPU load struct */
    App_CpuLoad cload;
    /* Output: INT latency struct */
    App_IntLatency ilate;
    /* Output: circle count per loop struct */
    App_CycleCountPerLoop ccploop;
    /* Output: counter for average period  */
    int64_t ave_count;
    /* Output: SRAM usage in percent (0-100) */
    int32_t sram_pcnt;
    /* Output: current core ID (0-3) */
    int32_t core_num;
    /* Output: current app (0-3) */
    int32_t app;
    /* Output: current freq (0-3) */
    int32_t freq;
} App_Output;

typedef struct
{
    /* core input parameters */
    App_Input input;
    /* core input parameters */
    App_Output output;
} App_Stats;

App_Stats gAppStats;    /* information sent out to visualize */
App_Input gAppInput;    /* information received as input from UI */

uint32_t gAppId;                        /* ID of app that is selected to run */
uint32_t gAppOptionSelect;              /* current selected option within the app */
uint32_t gAppOptions[APP_MAX_OPTIONS+1];/* list of options for this app */
uint32_t gAppRunFreq;                   /* frequency at which to run the app */
uint64_t gAppCpuHz;                     /* CPU clock in Hz for the current running CPU */
uint32_t gAppCountPerLoopMax;           /* Cycles taken for one run of benchmark, max */
uint32_t gAppCountPerLoopAvg;           /* Cycles taken for all runs of benchmark, avg */
uint64_t gAppCountPerLoopTotal;         /* Cycles taken for all runs of benchmark, total */
uint64_t gAppCountPerLoopIterations;    /* Number of benchmark iterations */
uint32_t gAppProfileOverheadCycles;     /* Overhead to measure the cycles, this is subtracted form measured value */

extern App_TimerStats gAppTimerStats;   /* Timer statistics */

void App_statsReset()
{
    uint32_t start, end;

    CycleCounterP_reset();
    start = CycleCounterP_getCount32();
    end = CycleCounterP_getCount32();

    if (end >= start)
    {
        gAppProfileOverheadCycles = end - start;
    }
    else
    {
        gAppProfileOverheadCycles = start - end;
    }

    gAppCountPerLoopMax = 0;
    gAppCountPerLoopAvg = 0;
    gAppCountPerLoopTotal = 0;
    gAppCountPerLoopIterations = 0;

    memset(&gAppStats, 0, sizeof(gAppStats));
    memset(&gAppInput, 0, sizeof(gAppInput));

    gAppStats.input.app = gAppId;
    gAppStats.input.freq = gAppOptionSelect;
    gAppStats.output.app = gAppId;
    gAppStats.output.freq = gAppOptionSelect;
}

void App_statsInit(uint32_t appId)
{
    gAppId = appId;
    gAppCpuHz = SOC_getSelfCpuClk();
    gAppOptionSelect = 1;

    /* we get input from 1 .. 4 from the UI, to avoid +1/-1 in logic,
     * putting a valid value at index 0 and then putting the options
     * so that the logic is simplified
     */
    switch(appId)
    {
        case APP_ID_CFFT:
            /* CFFT size */
            gAppOptions[0] = 128;
            gAppOptions[1] = 128;
            gAppOptions[2] = 256;
            gAppOptions[3] = 512;
            gAppOptions[4] = 1024;
            gAppRunFreq = 1000;
            break;
        case APP_ID_FIR:
            /* option selects the run frequency */
            gAppOptions[0] = 1000;
            gAppOptions[1] = 1000;
            gAppOptions[2] = 2000;
            gAppOptions[3] = 4000;
            gAppOptions[4] = 8000;
            gAppRunFreq = gAppOptions[gAppOptionSelect];
            break;
        case APP_ID_FOC:
            /* option selects the run frequency */
            gAppOptions[0] = 16*1000;
            gAppOptions[1] = 16*1000;
            gAppOptions[2] = 32*1000;
            gAppOptions[3] = 100*1000;
            gAppOptions[4] = 250*1000;
            gAppRunFreq = gAppOptions[gAppOptionSelect];
            break;
        case APP_ID_ADC:
            /* option selects the run frequency */
            gAppOptions[0] = 1000;
            gAppOptions[1] = 1000;
            gAppOptions[2] = 2000;
            gAppOptions[3] = 4000;
            gAppOptions[4] = 8000;
            gAppRunFreq = gAppOptions[gAppOptionSelect];
            break;
    }

    App_statsReset();
    App_timerSetHz(gAppRunFreq);

    #ifdef ENABLE_IPC_RPMSG_CHAR
    App_ipcInit();
    #endif
}

void App_statsUpdate(uint32_t start, uint32_t end)
{
    uint32_t total;

    if (end >= start)
    {
        total = end - start;
    }
    else
    {
        total = start - end;
    }
    if(total >= gAppProfileOverheadCycles)
    {
        total -= gAppProfileOverheadCycles;
    }

    /* Compute the average and max of count per loop */
    if (total>gAppCountPerLoopMax)
    {
        /* Count per loop max */
        gAppCountPerLoopMax = total;
    }

    gAppCountPerLoopIterations++;
    gAppCountPerLoopTotal += total;
    gAppCountPerLoopAvg    = gAppCountPerLoopTotal/gAppCountPerLoopIterations;

    /* populate the core stat */
    gAppStats.output.ave_count      = gAppTimerStats.isrCount;
    gAppStats.output.core_num       = IpcNotify_getSelfCoreId();
    gAppStats.output.app            = gAppId;
    gAppStats.output.freq           = gAppOptionSelect;
    gAppStats.output.ccploop.ave    = gAppCountPerLoopAvg;
    gAppStats.output.ccploop.max    = 0;
    gAppStats.output.cload.cur      = ((uint64_t)total*gAppRunFreq*100)/gAppCpuHz;
    gAppStats.output.cload.ave      = ((uint64_t)gAppCountPerLoopAvg*gAppRunFreq*100)/gAppCpuHz;
    gAppStats.output.cload.max      = 0;
    gAppStats.output.ilate.max      = 0;
    gAppStats.output.ilate.ave      = gAppTimerStats.avg;
}

void App_statsUpdateUI()
{
    #ifdef ENABLE_IPC_RPMSG_CHAR
    uint16_t recvMsgSize;

    /* Check for new message */
    App_ipcRecv((char *)&gAppInput, &recvMsgSize, sizeof(gAppInput));
    if (recvMsgSize>0)
    {
        /* make sure the app ID matches */
        if (gAppInput.app==gAppId)
        {
            if(gAppId == APP_ID_CFFT)
            {
                /* check for CFFT size selection change */
                if( gAppOptionSelect != gAppInput.freq )
                {
                    App_timerSetHz(gAppRunFreq);
                    gAppOptionSelect = gAppInput.freq;
                    if(gAppOptionSelect>APP_MAX_OPTIONS)
                        gAppOptionSelect = APP_MAX_OPTIONS;
                    App_statsReset();
                }
            }
            else
            {
                gAppOptionSelect = gAppInput.freq;
                if(gAppOptionSelect>APP_MAX_OPTIONS)
                    gAppOptionSelect = APP_MAX_OPTIONS;

                /* set the running frequency to the selected one */
                if (gAppRunFreq!=gAppOptions[gAppOptionSelect])
                {
                    /* set to selected frequency */
                    gAppRunFreq = gAppOptions[gAppOptionSelect];
                    App_timerSetHz(gAppRunFreq);
                    App_statsReset();
                }
            }
        }
        /* Send the gAppStats to the A53 */
        App_ipcSend((char *)&gAppStats, sizeof(gAppStats));
    }
    #endif
}

uint32_t App_statsGetOptionValue()
{
    return gAppOptions[gAppOptionSelect];
}
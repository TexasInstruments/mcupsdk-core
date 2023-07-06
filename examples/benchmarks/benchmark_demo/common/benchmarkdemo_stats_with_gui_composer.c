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
#include <stdio.h>
#include <float.h>
#include <drivers/soc.h>
#include "benchmarkdemo.h"
#include "ti_drivers_open_close.h"

/* max number of options across all apps */
#define APP_MAX_OPTIONS (5U)

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
    App_Output output;
} App_Stats;

volatile App_Stats gAppStats[5] __attribute__((aligned(128), section(".bss.user_shared_mem"))) = {0}; /* information sent out to GUI */

uint32_t gAppId;                       /* ID of app that is selected to run */
uint32_t gAppOptionSelect;             /* current selected option within the app */
uint32_t gAppOptions[APP_MAX_OPTIONS]; /* list of options for this app */
uint32_t gAppRunFreq;                  /* frequency at which to run the app */
uint64_t gAppCpuHz;                    /* CPU clock in Hz for the current running CPU */
uint32_t gAppCountPerLoopMax;          /* Cycles taken for one run of benchmark, max */
uint32_t gAppCountPerLoopAvg;          /* Cycles taken for all runs of benchmark, avg */
uint64_t gAppCountPerLoopTotal;        /* Cycles taken for all runs of benchmark, total */
uint64_t gAppCountPerLoopIterations;   /* Number of benchmark iterations */
uint32_t gAppProfileOverheadCycles;    /* Overhead to measure the cycles, this is subtracted form measured value */
uint32_t gAppCoreNum;

#ifdef APP_MAIN_CORE

#include "ina_device_current_monitor.h"
#include "stream_benchmark.h"

#define APP_UART_TX_BUF_SIZE (64U)
uint8_t gUartTxBuffer[APP_UART_TX_BUF_SIZE] = {0};

#define APP_UART_RX_BUF_SIZE (16U)
uint8_t gUartRxBuffer[APP_UART_RX_BUF_SIZE] = {0};

double gAppStreamMax[4];
double gAppStreamAvg[4];


#endif

/* from benchmarkdemo_timer.c */
extern App_TimerStats gAppTimerStats;

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

    gAppStats[gAppCoreNum].output.app = gAppId;
    gAppStats[gAppCoreNum].output.freq = gAppOptionSelect;
    gAppStats[gAppCoreNum].output.core_num = gAppCoreNum;


    App_timerResetStats();

#ifdef APP_MAIN_CORE
    for (uint32_t i = 0; i < 4; i++)
    {
        mintime[i] = FLT_MAX;
        avgtime[i] = 0;
        gAppStreamMax[i] = 0;
        gAppStreamAvg[i] = 0;
    }
#endif
}


void App_statsInit(uint32_t appId)
{
    gAppId = appId;
    gAppCpuHz = SOC_getSelfCpuClk();
    gAppOptionSelect = 0;
    gAppCoreNum = IpcNotify_getSelfCoreId();

    switch (appId)
    {
    case APP_ID_CFFT:
        /* CFFT size */
        gAppOptions[0] = 128;
        gAppOptions[1] = 256;
        gAppOptions[2] = 512;
        gAppOptions[3] = 1024;
        gAppOptions[4] = 2048;
        gAppRunFreq = 2000;
        break;
    case APP_ID_FIR:
        /* option selects the run frequency */
        gAppOptions[0] = 2000;
        gAppOptions[1] = 4000;
        gAppOptions[2] = 8000;
        gAppOptions[3] = 16000;
        gAppOptions[4] = 28000;
        gAppRunFreq = gAppOptions[gAppOptionSelect];
        break;
    case APP_ID_FOC:
        /* option selects the run frequency */
        gAppOptions[0] = 32 * 1000;
        gAppOptions[1] = 64 * 1000;
        gAppOptions[2] = 100 * 1000;
        gAppOptions[3] = 200 * 1000;
        gAppOptions[4] = 400 * 1000;
        gAppRunFreq = gAppOptions[gAppOptionSelect];
        break;
    case APP_ID_STREAM:
        /* option selects the run frequency */
        gAppOptions[0] = 1;
        gAppOptions[1] = 2;
        gAppOptions[2] = 10;
        gAppOptions[3] = 20;
        gAppOptions[4] = 30;
        gAppRunFreq = gAppOptions[gAppOptionSelect];
        break;
    }

    App_statsReset();
    App_timerSetHz(gAppRunFreq);
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

    if (total >= gAppProfileOverheadCycles)
    {
        total -= gAppProfileOverheadCycles;
    }

    /* Compute the average and max of count per loop */
    if (total > gAppCountPerLoopMax)
    {
        /* Count per loop max */
        gAppCountPerLoopMax = total;
    }

    gAppCountPerLoopIterations++;
    gAppCountPerLoopTotal += total;
    gAppCountPerLoopAvg = gAppCountPerLoopTotal / gAppCountPerLoopIterations;

    /* populate the core stat */
    gAppStats[gAppCoreNum].output.ave_count = gAppTimerStats.isrCount;
    gAppStats[gAppCoreNum].output.app = gAppId;
    gAppStats[gAppCoreNum].output.ccploop.ave = gAppCountPerLoopAvg;
    gAppStats[gAppCoreNum].output.ccploop.max = gAppCountPerLoopMax;
    gAppStats[gAppCoreNum].output.cload.cur = ((uint64_t)total * gAppRunFreq * 100) / gAppCpuHz;
    gAppStats[gAppCoreNum].output.cload.ave = ((uint64_t)gAppCountPerLoopAvg * gAppRunFreq * 100) / gAppCpuHz;
    gAppStats[gAppCoreNum].output.cload.max = ((uint64_t)gAppCountPerLoopMax * gAppRunFreq * 100) / gAppCpuHz;
    gAppStats[gAppCoreNum].output.ilate.max = gAppTimerStats.max;
    gAppStats[gAppCoreNum].output.ilate.ave = gAppTimerStats.avg;

    if (gAppOptionSelect != gAppStats[gAppCoreNum].output.freq)
    {
        gAppOptionSelect = gAppStats[gAppCoreNum].output.freq;

        if (gAppId != APP_ID_CFFT)
        {
            /* set to selected frequency */
            gAppRunFreq = gAppOptions[gAppOptionSelect];
        }
        App_statsReset();
        App_timerSetHz(gAppRunFreq);
    }
}

uint32_t App_statsGetOptionValue()
{
    return gAppOptions[gAppOptionSelect];
}

#ifndef APP_MAIN_CORE
void App_statsUpdateUI()
{
    /* nothing to do on non-main core's */
}
#endif

#ifdef APP_MAIN_CORE
void App_statsUpdateUI()
{
    UART_Transaction trans;
    uint32_t appTotalPower = 0;

    UART_Transaction_init(&trans);

    /* Check for GUI Input: App Option */
    trans.buf = (void *)gUartRxBuffer;
    trans.count = APP_UART_RX_BUF_SIZE;
    UART_read(gUartHandle[CONFIG_UART0], &trans);

    /* GUI expects:
        CSL_CORE_ID_R5FSS0_0 == 1U
        CSL_CORE_ID_R5FSS0_1 == 2U
        CSL_CORE_ID_R5FSS1_0 == 3U
        CSL_CORE_ID_R5FSS1_1 == 4U
    */
    for (uint32_t i = CSL_CORE_ID_R5FSS0_0; i <= CSL_CORE_ID_R5FSS1_1; i++)
    {

        /*Update GUI Output: CPU Loading */
        trans.buf = (void *)gUartTxBuffer;
        sprintf(trans.buf,
                "{\"cload_core%d\":{\"cur\":%d,\"ave\":%d,\"max\":%d}}\r\n",
                gAppStats[i].output.core_num, gAppStats[i].output.cload.cur,
                gAppStats[i].output.cload.ave, gAppStats[i].output.cload.max);
        trans.count = strlen(trans.buf);
        UART_write(gUartHandle[CONFIG_UART0], &trans);

        /* Update GUI Output: Interrupt Latency */
        trans.buf = (void *)gUartTxBuffer;
        sprintf(trans.buf, "{\"ilate_core%d\":{\"ave\":%d,\"max\":%d}}\r\n",
                gAppStats[i].output.core_num, gAppStats[i].output.ilate.ave,
                gAppStats[i].output.ilate.max);
        trans.count = strlen(trans.buf);
        UART_write(gUartHandle[CONFIG_UART0], &trans);

        /* Update GUI Output: Cycle Counter Per Loop */
        trans.buf = (void *)gUartTxBuffer;
        sprintf(trans.buf, "{\"ccploop_core%d\":{\"ave\":%d,\"max\":%d}}\r\n",
                gAppStats[i].output.core_num, gAppStats[i].output.ccploop.ave,
                gAppStats[i].output.ccploop.max);
        trans.count = strlen(trans.buf);
        UART_write(gUartHandle[CONFIG_UART0], &trans);
    }

    for (uint32_t i = 0; i < NUM_OF_INA_DEVICES; i++)
    {
        trans.buf = (void *)gUartTxBuffer;
        sprintf(trans.buf, "{\"bvolt\":{\"%d\":%.2f}}\r\n", i,
                (float)gAppBusVoltageStats[i] / 1000);
        trans.count = strlen(trans.buf);
        UART_write(gUartHandle[CONFIG_UART0], &trans);

        trans.buf = (void *)gUartTxBuffer;
        sprintf(trans.buf, "{\"cur\":{\"%d\":%d}}\r\n", i, gAppCurrentStats[i]);
        trans.count = strlen(trans.buf);
        UART_write(gUartHandle[CONFIG_UART0], &trans);

        appTotalPower += gAppPowerStats[i];
    }

    trans.buf = (void *)gUartTxBuffer;
    sprintf(trans.buf, "{\"power\":{\"6\":%d}}\r\n", appTotalPower);
    trans.count = strlen(trans.buf);
    UART_write(gUartHandle[CONFIG_UART0], &trans);

    /* STREAM stats */
    for (uint32_t i = 0; i < 4; i++)
    {
        if (avgtime[i] != 0)
        {
            gAppStreamAvg[i] = 1.0E-06 * bytes[i] / avgtime[i];
        }
        trans.buf = (void *)gUartTxBuffer;
        sprintf(trans.buf, "{\"streamavg\":{\"%d\":%12.1f}}\r\n", i, gAppStreamAvg[i]);
        trans.count = strlen(trans.buf);
        UART_write(gUartHandle[CONFIG_UART0], &trans);

        if (mintime[i] != 0)
        {
            gAppStreamMax[i] = 1.0E-06 * bytes[i] / mintime[i];
        }
        trans.buf = (void *)gUartTxBuffer;
        sprintf(trans.buf, "{\"streambest\":{\"%d\":%12.1f}}\r\n", i, gAppStreamMax[i]);
        trans.count = strlen(trans.buf);
        UART_write(gUartHandle[CONFIG_UART0], &trans);
    }
}

void uart_read_callback(UART_Handle handle, UART_Transaction *trans)
{
    uint32_t core;

    /* expecting "{"opt_cX":Y}" from GUI, where X is core num and Y is index of app option */
    for (uint32_t i = 0; i < trans->count; i++)
    {
        /* get the core num of sent app option*/
        if ('c' == gUartRxBuffer[i])
        {
            core = gUartRxBuffer[i + 1] - '0'; // subtracting '0' to convert from ascii
        }
        /* get index of new app option */
        if (':' == gUartRxBuffer[i])
        {
            gAppStats[core].output.freq = (uint32_t)gUartRxBuffer[i + 1] - '0'; // subtracting '0' to convert from ascii
        }
    }
}
#endif /* APP_MAIN_CORE */



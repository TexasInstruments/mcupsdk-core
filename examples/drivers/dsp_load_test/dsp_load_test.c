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

/*
 * DSP Load Test - AM273x C66x
 *
 * Keeps the C66x DSP fully loaded by running DSP_fft32x32 (1024-pt) in a
 * tight loop.  A high-priority monitor task wakes every DSP_LOAD_REPORT_PERIOD_MS
 * to print CPU load via TaskP_loadGetTotalCpuLoad() (which reads the C66x cycle
 * counter), then goes back to sleep - giving the FFT task full CPU in between.
 *
 * DSP_LOAD_NUM_REPORTS controls how long the test runs:
 *   0  - run forever (FFT task never stops)
 *   N  - print N windows then stop and exit cleanly
 *
 * Expected output (every DSP_LOAD_REPORT_PERIOD_MS):
 *   [DSP_LOAD]  [1] CPU: 100.00%  FFT: 99.99%  iters/window: 6061
 */

#include <stdint.h>
#include <math.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <ti/dsplib/dsplib.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "gen_twiddle_fft32x32.h"

/* -------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------
 * FFT_SIZE   : 1024-pt complex FFT.  Larger -> more cycles per call.
 *              Must be a power of 2 and >= 8.
 * PERIOD_MS  : Reporting interval in milliseconds.
 * NUM_REPORTS: 0 = run forever; N = stop after N windows.
 * ------------------------------------------------------------------------- */
#define DSP_LOAD_FFT_SIZE           (1024U)
#define DSP_LOAD_REPORT_PERIOD_MS   (2000U)
#define DSP_LOAD_NUM_REPORTS        (10U)    /* 0 = run forever */

/* -------------------------------------------------------------------------
 * Task priorities
 *   FFT task     : low  - fills every idle CPU cycle
 *   Monitor task : high - briefly preempts FFT to read and print load
 * ------------------------------------------------------------------------- */
#define FFT_TASK_PRI            (2U)
#define FFT_TASK_STACK_SIZE     (8U * 1024U)

#define MONITOR_TASK_PRI        (configMAX_PRIORITIES - 2U)
#define MONITOR_TASK_STACK_SIZE (4U * 1024U)

/* -------------------------------------------------------------------------
 * FFT buffers - placed in DSS L3 to avoid filling the smaller L2
 * ------------------------------------------------------------------------- */
#pragma DATA_SECTION(gFftInput,   ".bss.dss_l3")
#pragma DATA_SECTION(gFftOutput,  ".bss.dss_l3")
#pragma DATA_SECTION(gFftTwiddle, ".bss.dss_l3")

#pragma DATA_ALIGN(gFftInput,   8)
int32_t gFftInput  [2U * DSP_LOAD_FFT_SIZE];

#pragma DATA_ALIGN(gFftOutput,  8)
int32_t gFftOutput [2U * DSP_LOAD_FFT_SIZE];

#pragma DATA_ALIGN(gFftTwiddle, 8)
int32_t gFftTwiddle[2U * DSP_LOAD_FFT_SIZE];

/* -------------------------------------------------------------------------
 * Task objects (global - required for FreeRTOS static allocation)
 * ------------------------------------------------------------------------- */
static uint8_t      gFftTaskStack    [FFT_TASK_STACK_SIZE]     __attribute__((aligned(32)));
static TaskP_Object gFftTaskObj;

static uint8_t      gMonitorTaskStack[MONITOR_TASK_STACK_SIZE] __attribute__((aligned(32)));
static TaskP_Object gMonitorTaskObj;

/* Written by FFT task, read by monitor task */
static volatile uint32_t gFftIterCount = 0U;
/* Set to 1 by monitor task when NUM_REPORTS is reached */
static volatile uint32_t gStopFftTask  = 0U;

/* -------------------------------------------------------------------------
 * Helper: initialise FFT input with a sine wave
 * ------------------------------------------------------------------------- */
static void dsp_load_initInput(void)
{
    uint32_t i;
    for (i = 0U; i < DSP_LOAD_FFT_SIZE; i++)
    {
        gFftInput[2U * i]      = (int32_t)(0x7FFFFFFFL *
                                   sinf(2.0f * 3.14159265f * (float)i /
                                        (float)DSP_LOAD_FFT_SIZE));
        gFftInput[2U * i + 1U] = 0;
    }
}

/* -------------------------------------------------------------------------
 * FFT load task (low priority)
 *   Runs DSP_fft32x32 in a tight loop - no sleep, no yield.
 *   The monitor task's periodic sleep is what gives it CPU time to report;
 *   this task claims all remaining cycles to keep the DSP fully loaded.
 * ------------------------------------------------------------------------- */
static void dsp_load_fftTask(void *args)
{
    while (gStopFftTask == 0U)
    {
        DSP_fft32x32(gFftTwiddle, DSP_LOAD_FFT_SIZE, gFftInput, gFftOutput);
        gFftIterCount++;
    }

    TaskP_exit();
}

/* -------------------------------------------------------------------------
 * Monitor task (high priority)
 *   Sleeps for REPORT_PERIOD_MS (FFT task runs freely during this time),
 *   then wakes briefly to read and print the C66x CPU load.
 *   Runs forever when DSP_LOAD_NUM_REPORTS == 0.
 * ------------------------------------------------------------------------- */
static void dsp_load_monitorTask(void *args)
{
    uint32_t   report   = 0U;
    uint32_t   cpuLoad;
    TaskP_Load fftLoad;
    uint32_t   prevIter;

    DebugP_log("\r\n[DSP_LOAD] =============================================\r\n");
    DebugP_log("[DSP_LOAD]  AM273x C66x DSP Load Test\r\n");
    DebugP_log("[DSP_LOAD]  FFT: %u pts | Window: %u ms | Reports: ",
               DSP_LOAD_FFT_SIZE, DSP_LOAD_REPORT_PERIOD_MS);
    if (DSP_LOAD_NUM_REPORTS == 0U)
        DebugP_log("forever\r\n");
    else
        DebugP_log("%u\r\n", DSP_LOAD_NUM_REPORTS);
    DebugP_log("[DSP_LOAD] =============================================\r\n");

    TaskP_loadResetAll();

    while (1)
    {
        prevIter = gFftIterCount;

        /* Sleep: FFT task runs at full speed during this period */
        ClockP_usleep(DSP_LOAD_REPORT_PERIOD_MS * 1000U);

        cpuLoad = TaskP_loadGetTotalCpuLoad();
        TaskP_loadGet(&gFftTaskObj, &fftLoad);
        report++;

        if (DSP_LOAD_NUM_REPORTS == 0U)
        {
            DebugP_log("[DSP_LOAD]  [%u] CPU: %3u.%02u%%  FFT: %3u.%02u%%  iters/window: %u\r\n",
                       report,
                       cpuLoad / 100U, cpuLoad % 100U,
                       fftLoad.cpuLoad / 100U, fftLoad.cpuLoad % 100U,
                       gFftIterCount - prevIter);
        }
        else
        {
            DebugP_log("[DSP_LOAD]  [%u/%u] CPU: %3u.%02u%%  FFT: %3u.%02u%%  iters/window: %u\r\n",
                       report, DSP_LOAD_NUM_REPORTS,
                       cpuLoad / 100U, cpuLoad % 100U,
                       fftLoad.cpuLoad / 100U, fftLoad.cpuLoad % 100U,
                       gFftIterCount - prevIter);
        }

        TaskP_loadResetAll();

        if ((DSP_LOAD_NUM_REPORTS != 0U) && (report >= DSP_LOAD_NUM_REPORTS))
            break;
    }

    /* Only reached when DSP_LOAD_NUM_REPORTS > 0 */
    gStopFftTask = 1U;
    ClockP_usleep(100000U);     /* allow FFT task to observe the flag and exit */

    DebugP_log("[DSP_LOAD] =============================================\r\n");
    DebugP_log("[DSP_LOAD]  Test Completed!\r\n");
    DebugP_log("[DSP_LOAD] =============================================\r\n");

    Board_driversClose();
    Drivers_close();

    TaskP_exit();
}

/* -------------------------------------------------------------------------
 * Entry point called from main.c
 * ------------------------------------------------------------------------- */
void dsp_load_test_main(void *args)
{
    int32_t      status;
    TaskP_Params taskParams;

    Drivers_open();
    Board_driversOpen();

    /* Pre-compute twiddle factors and input data before tasks start */
    gen_twiddle_fft32x32((int *)gFftTwiddle, DSP_LOAD_FFT_SIZE, 2147483647.5);
    dsp_load_initInput();

    /* FFT load task - runs continuously at low priority */
    TaskP_Params_init(&taskParams);
    taskParams.name      = "fft_load";
    taskParams.stackSize = FFT_TASK_STACK_SIZE;
    taskParams.stack     = gFftTaskStack;
    taskParams.priority  = FFT_TASK_PRI;
    taskParams.args      = NULL;
    taskParams.taskMain  = dsp_load_fftTask;
    status = TaskP_construct(&gFftTaskObj, &taskParams);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Monitor task - wakes periodically to report load */
    TaskP_Params_init(&taskParams);
    taskParams.name      = "dsp_monitor";
    taskParams.stackSize = MONITOR_TASK_STACK_SIZE;
    taskParams.stack     = gMonitorTaskStack;
    taskParams.priority  = MONITOR_TASK_PRI;
    taskParams.args      = NULL;
    taskParams.taskMain  = dsp_load_monitorTask;
    status = TaskP_construct(&gMonitorTaskObj, &taskParams);
    DebugP_assert(status == SystemP_SUCCESS);

    TaskP_exit();
}

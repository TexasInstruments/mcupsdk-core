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

#include <dhry.h>
#include <FreeRTOS.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Number of Dhrystone iterations */
#define DHRY_ITERATIONS     (30000000U)

/* Disable logging in dhrystone */
int dhryLogEnable = 0;

void dhrystone_benchmark_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    uint32_t dhryPerSec = 0;
    int status = SystemP_SUCCESS;

    /* Run Dhrystone demo with 1 thread */
    status = dhryCreateThreads(1, DHRY_ITERATIONS, &dhryPerSec);
    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[DHRYSTONE BENCHMARKING] %-32s : %d\r\n", "Iterations", DHRY_ITERATIONS);
        DebugP_log("[DHRYSTONE BENCHMARKING] %-32s : %d\r\n", "Threads", 1);
        DebugP_log("[DHRYSTONE BENCHMARKING] %-32s : %d\r\n", "Dhrystones per second", dhryPerSec);
        DebugP_log("\r\n");
    }

    /* Run Dhrystone demo with 2 threads */
    if(status == SystemP_SUCCESS)
    {
        status = dhryCreateThreads(2, DHRY_ITERATIONS, &dhryPerSec);
        if(status == SystemP_SUCCESS)
        {
            DebugP_log("[DHRYSTONE BENCHMARKING] %-32s : %d\r\n", "Iterations", DHRY_ITERATIONS);
            DebugP_log("[DHRYSTONE BENCHMARKING] %-32s : %d\r\n", "Threads", 2);
            DebugP_log("[DHRYSTONE BENCHMARKING] %-32s : %d\r\n", "Dhrystones per second", dhryPerSec);
            DebugP_log("\r\n");
        }
    }

    /* Run Dhrystone demo with 5 threads */
    if(status == SystemP_SUCCESS)
    {
        status = dhryCreateThreads(5, DHRY_ITERATIONS, &dhryPerSec);
        if(status == SystemP_SUCCESS)
        {
            DebugP_log("[DHRYSTONE BENCHMARKING] %-32s : %d\r\n", "Iterations", DHRY_ITERATIONS);
            DebugP_log("[DHRYSTONE BENCHMARKING] %-32s : %d\r\n", "Threads", 5);
            DebugP_log("[DHRYSTONE BENCHMARKING] %-32s : %d\r\n", "Dhrystones per second", dhryPerSec);
            DebugP_log("\r\n");
        }
    }

    /* Run Dhrystone demo with 10 threads */
    if(status == SystemP_SUCCESS)
    {
        status = dhryCreateThreads(10, DHRY_ITERATIONS, &dhryPerSec);
        if(status == SystemP_SUCCESS)
        {
            DebugP_log("[DHRYSTONE BENCHMARKING] %-32s : %d\r\n", "Iterations", DHRY_ITERATIONS);
            DebugP_log("[DHRYSTONE BENCHMARKING] %-32s : %d\r\n", "Threads", 10);
            DebugP_log("[DHRYSTONE BENCHMARKING] %-32s : %d\r\n", "Dhrystones per second", dhryPerSec);
            DebugP_log("\r\n");
        }
    }

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}
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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>
#include "tx_api.h"
#include "ti_eclipse_threadx_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * IMPORTANT NOTES:
 *
 * Alignment of stack is not strictly needed for R5F but debug is easier if stack is nicely
 * aligned.
 *
 * Task priority, 0 is lowest priority, TX_MAX_PRIORITIES-1 is highest
 * For this example any valid task priority can be set.
 *
 * See tx_port.h for TX_MAX_PRIORITIES.
 * tx_port.h can be found under eclipse_threadx/threadx/threadx_src/ports_module/${cpu}/${toolchain}/inc
 *
 */

#define NUM_TASK_SWITCHES      (1000000u)

#define PING_INT_NUM           (20u)
#define PONG_INT_NUM           (21u)

#define PING_TASK_PRI  (2u)
#define PONG_TASK_PRI  (3u)

#define PING_TASK_SIZE (8192u)
uint8_t gPingTaskStack[PING_TASK_SIZE] __attribute__((aligned(32)));

#define PONG_TASK_SIZE (8192u)
uint8_t gPongTaskStack[PONG_TASK_SIZE] __attribute__((aligned(32)));

TX_THREAD gPingThread;
TX_SEMAPHORE gPingSem;
HwiP_Object gPingHwiObj;

TX_THREAD gPongThread;
TX_SEMAPHORE gPongSem;
HwiP_Object gPongHwiObj;

static void ping_isr(void *arg)
{
    UINT status;

    status = tx_semaphore_put(&gPongSem);
    DebugP_assertNoLog(status == TX_SUCCESS);
}

static void pong_isr(void *arg)
{
    UINT status;

    status = tx_semaphore_put(&gPingSem);
    DebugP_assertNoLog(status == TX_SUCCESS);
}

void ping_main(ULONG args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;

    DebugP_log("\r\n");
    DebugP_log("[THREADX TASK SWITCH] ping task ... start !!!\r\n");
    { /* switch between ping and pong tasks using semaphores */
        count = NUM_TASK_SWITCHES;
        curTime = ClockP_getTimeUsec();
        while(count--)
        {
            tx_semaphore_put(&gPongSem); /* wake up pong task */
            tx_semaphore_get(&gPingSem, TX_WAIT_FOREVER); /* wait for pong to signal */
        }
        curTime = ClockP_getTimeUsec() - curTime;

        DebugP_log("\r\n");
        DebugP_log("execution time for task switches = %" PRId64 " us\r\n", curTime);
        DebugP_log("number of task switches = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES*2);
        DebugP_log("time per task switch (semaphore give/take) = %" PRId32 " ns\r\n", (uint32_t)(curTime*1000/(NUM_TASK_SWITCHES*2)));
    }
    { /* switch from ping task to ISR to pong task and back to ping task using semaphores, here there is a task switch */
        HwiP_Params hwiParams;

        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = PING_INT_NUM;
        hwiParams.callback = ping_isr;
        HwiP_construct(&gPingHwiObj, &hwiParams);

        count = NUM_TASK_SWITCHES;
        curTime = ClockP_getTimeUsec();
        while(count--)
        {
            HwiP_post(PING_INT_NUM);
            tx_semaphore_get(&gPingSem, TX_WAIT_FOREVER); /* wait for ISR to signal */
        }
        curTime = ClockP_getTimeUsec() - curTime;

        HwiP_destruct(&gPingHwiObj);

        DebugP_log("\r\n");
        DebugP_log("execution time for task - ISR - task - task switches = %" PRId64 " us\r\n", curTime);
        DebugP_log("number of ISRs = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES*2);
        DebugP_log("time per task - ISR - task switch (semaphore give/take) = %" PRId32 " ns\r\n", (uint32_t)(curTime*1000/(2*NUM_TASK_SWITCHES)));
    }

    /* delay some time, just to show delay works */
    tx_thread_sleep( ClockP_usecToTicks(100*1000) );
    tx_thread_sleep( ClockP_usecToTicks(101*1000) );

    DebugP_log("\r\n");
    DebugP_log("[THREADX TASK SWITCH] ping task ... done !!!\r\n");
    DebugP_log("\r\n");
    DebugP_log("All tests have passed!!\r\n");

    tx_thread_delete(tx_thread_identify());
}

void pong_main(ULONG args)
{
    uint32_t count; /* loop `count` times */

    count = NUM_TASK_SWITCHES;
    while(count--)
    {
        tx_semaphore_get(&gPongSem, TX_WAIT_FOREVER); /* wait for ping to signal */
        tx_semaphore_put(&gPingSem); /* wakeup ping task */
    }
    {
        HwiP_Params hwiParams;

        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = PONG_INT_NUM;
        hwiParams.callback = pong_isr;
        HwiP_construct(&gPongHwiObj, &hwiParams);

        count = NUM_TASK_SWITCHES;
        while(count--)
        {
            tx_semaphore_get(&gPongSem, TX_WAIT_FOREVER); /* wait for ISR to signal */
            HwiP_post(PONG_INT_NUM);
        }
        HwiP_destruct(&gPongHwiObj);
    }

    tx_thread_delete(tx_thread_identify());
}

void threadx_task_switch_main(ULONG args)
{
    int32_t res;
    UINT status;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    res = EclipseThreadx_open();
    DebugP_assert(res == SystemP_SUCCESS);

    /* first create the semaphores */
    status = tx_semaphore_create(&gPingSem, "ping_sem", 0);
    DebugP_assert(status == TX_SUCCESS);

    status = tx_semaphore_create(&gPongSem, "pong_sem", 0);
    DebugP_assert(status == TX_SUCCESS);

    status = tx_thread_create(&gPongThread, "pong", pong_main, 0, gPongTaskStack,
                              PONG_TASK_SIZE, PONG_TASK_PRI, PONG_TASK_PRI, TX_NO_TIME_SLICE, TX_AUTO_START);
    DebugP_assert(status == TX_SUCCESS);


    status = tx_thread_create(&gPingThread, "ping", ping_main, 0, gPingTaskStack,
                              PING_TASK_SIZE, PING_TASK_PRI, PING_TASK_PRI, TX_NO_TIME_SLICE, TX_AUTO_START);
    DebugP_assert(status == TX_SUCCESS);


    res = EclipseThreadx_close();
    DebugP_assert(res == SystemP_SUCCESS);

    Board_driversClose();
    /* Dont close drivers to keep the UART driver open for console */
    /* Drivers_close(); */
}

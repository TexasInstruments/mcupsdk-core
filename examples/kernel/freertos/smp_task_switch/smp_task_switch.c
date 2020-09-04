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
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * IMPORTANT NOTES:
 *
 * Alignment of stack is not strictly needed for R5F but debug is easier if stack is nicely
 * aligned.
 *
 * Task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest
 * For this example any valid task priority can be set.
 *
 * See FreeRTOSConfig.h for configMAX_PRIORITIES and StackType_t type.
 * FreeRTOSConfig.h can be found under kernel/freertos/config/${device}/${cpu}/
 *
 * In this example,
 * We create task's, semaphore's, ISR's and stack for the tasks using static allocation.
 * We dont need to delete these semaphore's since static allocation is used.
 *
 * One MUST not return out of a FreeRTOS task instead one MUST call vTaskDelete instead.
 */

#define NUM_TASK_SWITCHES      (1000000u)

#define PING_INT_NUM           (20u)
#define PONG_INT_NUM           (21u)

#define PING_TASK_PRI  (2u)
#define PONG_TASK_PRI  (3u)

#define PING_TASK_SIZE (1024u)
StackType_t gPingTaskStack[PING_TASK_SIZE] __attribute__((aligned(32)));

#define PONG_TASK_SIZE (1024u)
StackType_t gPongTaskStack[PONG_TASK_SIZE] __attribute__((aligned(32)));

StaticTask_t gPingTaskObj;
TaskHandle_t gPingTask;
StaticSemaphore_t gPingSemObj;
SemaphoreHandle_t gPingSem;
HwiP_Object gPingHwiObj;

StaticTask_t gPongTaskObj;
TaskHandle_t gPongTask;
StaticSemaphore_t gPongSemObj;
SemaphoreHandle_t gPongSem;
HwiP_Object gPongHwiObj;

static void ping_isr(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    xSemaphoreGiveFromISR( gPongSem, &doTaskSwitch); /* wake up pong task */
    portYIELD_FROM_ISR( doTaskSwitch );
}

static void pong_isr(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    xSemaphoreGiveFromISR( gPingSem, &doTaskSwitch); /* wake up ping task */
    portYIELD_FROM_ISR( doTaskSwitch );
}

void ping_main(void *args)
{
    uint64_t curTime;
    uint32_t count; /* loop `count` times */

    DebugP_log("\r\n");
    DebugP_log("[FreeRTOS] ping task ... start !!!\r\n");
    { /* switch between ping and pong tasks using semaphores */
        count = NUM_TASK_SWITCHES;
        curTime = ClockP_getTimeUsec();
        while(count--)
        {
            xSemaphoreGive( gPongSem); /* wake up pong task */
            xSemaphoreTake( gPingSem, portMAX_DELAY); /* wait for pong to signal */
        }
        curTime = ClockP_getTimeUsec() - curTime;

        DebugP_log("\r\n");
        DebugP_log("execution time for task switches = %" PRId64 " us\r\n", curTime);
        DebugP_log("number of task switches = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES*2);
        DebugP_log("time per task switch (semaphore give/take) = %" PRId32 " ns\r\n", (uint32_t)(curTime*1000/(NUM_TASK_SWITCHES*2)));
    }
    { /* switch between ping and pong tasks using direct-to-task notifications */
        count = NUM_TASK_SWITCHES;
        curTime = ClockP_getTimeUsec();
        while(count--)
        {
            xTaskNotifyGive( gPongTask); /* wake up pong task */
            ulTaskNotifyTake( pdTRUE, portMAX_DELAY); /* wait for pong to signal */
        }
        curTime = ClockP_getTimeUsec() - curTime;

        DebugP_log("\r\n");
        DebugP_log("execution time for task switches = %" PRId64 " us\r\n", curTime);
        DebugP_log("number of task switches = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES*2);
        DebugP_log("time per task switch (direct-to-task notification give/take) = %" PRId32 " ns\r\n", (uint32_t)(curTime*1000/(NUM_TASK_SWITCHES*2)));
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
            xSemaphoreTake( gPingSem, portMAX_DELAY); /* wait for ISR to signal */
        }
        curTime = ClockP_getTimeUsec() - curTime;

        HwiP_destruct(&gPingHwiObj);

        DebugP_log("\r\n");
        DebugP_log("execution time for task - ISR - task - task switches = %" PRId64 " us\r\n", curTime);
        DebugP_log("number of ISRs = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES*2);
        DebugP_log("time per task - ISR - task switch (semaphore give/take) = %" PRId32 " ns\r\n", (uint32_t)(curTime*1000/(2*NUM_TASK_SWITCHES)));
    }

    /* delay some time, just to show delay works */
    vTaskDelay( ClockP_usecToTicks(100*1000) );
    vTaskDelay( ClockP_usecToTicks(101*1000) );

    DebugP_log("\r\n");
    DebugP_log("[FreeRTOS] ping task ... done !!!\r\n");
    DebugP_log("\r\n");
    DebugP_log("All tests have passed!!\r\n");

    /* One MUST not return out of a FreeRTOS task instead one MUST call vTaskDelete */
    vTaskDelete(NULL);
}

void pong_main(void *args)
{
    uint32_t count; /* loop `count` times */

    count = NUM_TASK_SWITCHES;
    while(count--)
    {
        xSemaphoreTake( gPongSem, portMAX_DELAY); /* wait for ping to signal */
        xSemaphoreGive( gPingSem); /* wakeup ping task */
    }
    count = NUM_TASK_SWITCHES;
    while(count--)
    {
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY); /* wait for ping to signal */
        xTaskNotifyGive( gPingTask); /* wake up ping task */
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
            xSemaphoreTake( gPongSem, portMAX_DELAY); /* wait for ISR to signal */
            HwiP_post(PONG_INT_NUM);
        }
        HwiP_destruct(&gPongHwiObj);
    }
    /* One MUST not return out of a FreeRTOS task instead one MUST call vTaskDelete */
    vTaskDelete(NULL);
}

void smp_task_switch_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* first create the semaphores */
    gPingSem = xSemaphoreCreateBinaryStatic(&gPingSemObj);
    configASSERT(gPingSem != NULL);
    vQueueAddToRegistry(gPingSem, "Ping Sem"); /* This makes the semaphore visible in ROV within CCS IDE */

    gPongSem = xSemaphoreCreateBinaryStatic(&gPongSemObj);
    configASSERT(gPongSem != NULL);
    vQueueAddToRegistry(gPongSem, "Pong Sem"); /* This makes the semaphore visible in ROV within CCS IDE */

    /* then create the tasks, order of task creation does not matter for this example */
    gPongTask = xTaskCreateStatic( pong_main,      /* Pointer to the function that implements the task. */
                                  "pong",          /* Text name for the task.  This is to facilitate debugging only. */
                                  PONG_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,            /* We are not using the task parameter. */
                                  PONG_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gPongTaskStack,  /* pointer to stack base */
                                  &gPongTaskObj ); /* pointer to statically allocated task object memory */
    configASSERT(gPongTask != NULL);

    /* Set Core affinity so "pong_main" task runs on Core 1 */
    /* Core affinity needs to be set as the interrupt used is an PPI (core specific) */
    /* So the Interrupt construct and post needs to be done from the same core */
    vTaskCoreAffinitySet(gPongTask, 2);

    gPingTask = xTaskCreateStatic( ping_main,      /* Pointer to the function that implements the task. */
                                  "ping",          /* Text name for the task.  This is to facilitate debugging only. */
                                  PING_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,            /* We are not using the task parameter. */
                                  PING_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gPingTaskStack,  /* pointer to stack base */
                                  &gPingTaskObj ); /* pointer to statically allocated task object memory */
    configASSERT(gPingTask != NULL);

    /* Set Core affinity so "ping_main" task runs on Core 0 */
    /* Core affinity needs to be set as the interrupt used is an PPI (core specific) */
    /* So the Interrupt construct and post needs to be done from the same core */
    vTaskCoreAffinitySet(gPingTask, 1);

    Board_driversClose();
    /* Dont close drivers to keep the UART driver open for console */
    /* Drivers_close(); */
}

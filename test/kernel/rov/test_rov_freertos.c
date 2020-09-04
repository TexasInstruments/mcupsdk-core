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
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This test is mainly to test FreeRTOS ROV features, the test application itself does not do any exciting
 * and there is no pass / fail other than the test exits gracefully.
 *
 * Please load the application and run via a CCS project.
 * Please refer ROV developer guide for more details.
 *
 * This is a manual test, after loading the binary put a break point in ping task
 * and single step and at each step see if the info as shown in ROV is as expected.
 *
 * Test on R5F as well as M4F, the ROV info should display correctly on both.
 *
 * ROV is right now support with FreeRTOS only.
 * With NORTOS, the ROV windows in CCS should gracefully exit with error.
 */

#define PING_INT_NUM           (20u)
#define PONG_INT_NUM           (21u)

#define PING_TASK_PRI  (2u)
#define PONG_TASK_PRI  (3u)

#define PING_TASK_SIZE (256*4u)
StackType_t gPingTaskStack[PING_TASK_SIZE] __attribute__((aligned(32)));

#define PONG_TASK_SIZE (256*4u)
StackType_t gPongTaskStack[PONG_TASK_SIZE] __attribute__((aligned(32)));

#define TASK_DPL_STACK_SIZE (256*2u)
uint8_t gDplTaskStack[TASK_DPL_STACK_SIZE] __attribute__((aligned(32)));

StaticTask_t gPingTaskObj;
TaskHandle_t gPingTask;
StaticTask_t gPongTaskObj;
TaskHandle_t gPongTask;

HwiP_Object gPingHwiObj;
HwiP_Object gPongHwiObj;

StaticSemaphore_t gPingSemObj;
SemaphoreHandle_t gPingSem;
StaticSemaphore_t gPongSemObj;
SemaphoreHandle_t gPongSem;
StaticSemaphore_t gCountingSemObj;
SemaphoreHandle_t gCountingSem;
StaticSemaphore_t gMutexSemObj;
SemaphoreHandle_t gMutexSem;
StaticSemaphore_t gMutexRecursiveSemObj;
SemaphoreHandle_t gMutexRecursiveSem;

#define QUEUE_ITEM_SIZE (1u)
#define QUEUE_ITEMS     (1u)
StaticQueue_t gQueueObj;
QueueHandle_t gQueue;
uint8_t gQueueBuf[QUEUE_ITEMS*QUEUE_ITEM_SIZE];

StaticTimer_t gTimerObj1;
TimerHandle_t gTimer1;
StaticTimer_t gTimerObj2;
TimerHandle_t gTimer2;

SemaphoreP_Object gDplBinarySem;
SemaphoreP_Object gDplCountingSem;
SemaphoreP_Object gDplMutex;

TaskP_Object gDplTask;
ClockP_Object gDplClock;

void timer_callback1( TimerHandle_t xTimer )
{
    /* we dont need to do anything here */
}

void timer_callback2( TimerHandle_t xTimer )
{
    /* we dont need to do anything here */
}

void clock_callback(ClockP_Object *obj, void *args)
{
    /* we dont need to do anything here */
}

static void ping_isr(void *arg)
{
    /* we dont need to do anything here */
}

static void pong_isr(void *arg)
{
    /* we dont need to do anything here */
}

void ping_main(void *args)
{
    uint8_t queueItem;
    uint32_t i;

    DebugP_log("Starting timers ...\r\n");

    /* start the timer, now it should be visible in ROV, ROV only shows timers which are started */
    xTimerStart(gTimer1, portMAX_DELAY);
    xTimerStart(gTimer2, portMAX_DELAY);

    DebugP_log("Giving semaphores ...\r\n");

    /* put a break point here, we should see receiver blocked in ROV */
    xSemaphoreGive( gPingSem);
    /* put a break point here, we should see receiver blocked in ROV */
    xSemaphoreGive( gPongSem);

    DebugP_log("Send messages to queues ...\r\n");

    /* put a break point here, we should see receiver blocked in ROV */
    xQueueSend( gQueue, &queueItem, portMAX_DELAY);
    xQueueReceive( gQueue, &queueItem, portMAX_DELAY);
    /* put a break point here, we should see sender blocked in ROV */
    xQueueReceive( gQueue, &queueItem, portMAX_DELAY);

    DebugP_log("Take mutexes ...\r\n");

    xSemaphoreTake( gMutexSem, portMAX_DELAY);
    xSemaphoreTakeRecursive( gMutexRecursiveSem, portMAX_DELAY);
    xSemaphoreTakeRecursive( gMutexRecursiveSem, portMAX_DELAY);

    DebugP_log("Give mutexes ...\r\n");

    /* put a break point here, we should see mutex holder and recursive call count in ROV */
    xSemaphoreGiveRecursive( gMutexRecursiveSem );
    xSemaphoreGiveRecursive( gMutexRecursiveSem );
    xSemaphoreGive( gMutexSem );

    DebugP_log("Wait for some time ...\r\n");

    vTaskDelay( 100 );

    DebugP_log("Stop timers ...\r\n");

    xTimerStop( gTimer1, portMAX_DELAY );
    /* put a break point here, we should see a timer removed from ROV */

    vTaskDelay( 10 );

    /* put a break point here, we should see logs printed so far in ROV */
    for(i=0; i<500; i++)
    {
        DebugP_log("This is line %3d\r\n", i);
    }
    /* put a break point here, we should see logs printed so far in ROV, but now the log should have wrapped around */
    for(; i<1000; i++)
    {
        DebugP_log("This is line %3d\r\n", i);
    }
    /* put a break point here, we should see logs printed so far in ROV, but now the log should have wrapped around again */

    DebugP_log("All tests have passed!!\r\n");

    /* One MUST not return out of a FreeRTOS task instead one MUST call vTaskDelete */
    vTaskDelete(NULL);
}

void pong_main(void *args)
{
    uint8_t queueItem;

    xSemaphoreTake( gPingSem, portMAX_DELAY);
    xSemaphoreTake( gPongSem, portMAX_DELAY);

    xQueueReceive( gQueue, &queueItem, portMAX_DELAY);
    xQueueSend( gQueue, &queueItem, portMAX_DELAY);
    xQueueSend( gQueue, &queueItem, portMAX_DELAY);

    /* One MUST not return out of a FreeRTOS task instead one MUST call vTaskDelete */
    vTaskDelete(NULL);
}

void task_dpl_main(void *args)
{
    /* sleep for a long time */
    ClockP_sleep(60*60*24);

    TaskP_exit();
}

void test_rov_main(void *args)
{
    HwiP_Params hwiParams;
    ClockP_Params clockParams;
    TaskP_Params taskParams;
    int32_t status;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("\r\n");

    /* single step in this function, as objects are created they will appear in ROV */
    DebugP_log("Creating semaphores ...\r\n");

    gPingSem = xSemaphoreCreateBinaryStatic(&gPingSemObj);
    configASSERT(gPingSem != NULL);
    /* we need to do below to make a semaphore, mutex, queue to be visible in ROV */
    vQueueAddToRegistry(gPingSem, "My Ping Sem");

    gPongSem = xSemaphoreCreateBinaryStatic(&gPongSemObj);
    configASSERT(gPongSem != NULL);
    vQueueAddToRegistry(gPongSem, "My Pong Sem");

    gCountingSem = xSemaphoreCreateCountingStatic(4, 1, &gCountingSemObj);
    configASSERT(gCountingSem != NULL);
    vQueueAddToRegistry(gCountingSem, "My Counting Sem");

    DebugP_log("Creating mutexes ...\r\n");

    gMutexSem = xSemaphoreCreateMutexStatic(&gMutexSemObj);
    configASSERT(gMutexSem != NULL);
    vQueueAddToRegistry(gMutexSem, "My Mutex");

    gMutexRecursiveSem = xSemaphoreCreateRecursiveMutexStatic(&gMutexRecursiveSemObj);
    configASSERT(gMutexRecursiveSem != NULL);
    vQueueAddToRegistry(gMutexRecursiveSem, "My Recursive Mutex");

    DebugP_log("Creating queues ...\r\n");

    gQueue = xQueueCreateStatic(QUEUE_ITEMS, QUEUE_ITEM_SIZE, gQueueBuf, &gQueueObj);
    configASSERT(gQueue != NULL);
    vQueueAddToRegistry(gQueue, "My Queue");

    DebugP_log("Creating timers ...\r\n");

    gTimer1 = xTimerCreateStatic("My Timer 1", 10, pdTRUE, (void*)0, timer_callback1, &gTimerObj1);
    configASSERT(gTimer1 != NULL);

    gTimer2 = xTimerCreateStatic("My Timer 2", 100, pdTRUE, (void*)1, timer_callback2, &gTimerObj2);
    configASSERT(gTimer2 != NULL);

    DebugP_log("Creating interrupts ...\r\n");

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = PING_INT_NUM;
    hwiParams.callback = ping_isr;
    status = HwiP_construct(&gPingHwiObj, &hwiParams);
    DebugP_assert(status==SystemP_SUCCESS);

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = PONG_INT_NUM;
    hwiParams.callback = pong_isr;
    status = HwiP_construct(&gPongHwiObj, &hwiParams);
    DebugP_assert(status==SystemP_SUCCESS);

    DebugP_log("Creating DPL semaphores and mutexs ...\r\n");

    status = SemaphoreP_constructBinary(&gDplBinarySem, 0);
    DebugP_assert(status==SystemP_SUCCESS);
    status = SemaphoreP_constructCounting(&gDplCountingSem, 0, 2);
    DebugP_assert(status==SystemP_SUCCESS);
    status = SemaphoreP_constructMutex(&gDplMutex);
    DebugP_assert(status==SystemP_SUCCESS);

    DebugP_log("Creating DPL clocks ...\r\n");

    ClockP_Params_init(&clockParams);
    clockParams.start = 1;
    clockParams.timeout = 20;
    clockParams.period = 20;
    clockParams.callback = clock_callback;
    clockParams.args = (void*)0x3;
    status = ClockP_construct(&gDplClock, &clockParams);
    DebugP_assert(status==SystemP_SUCCESS);

    DebugP_log("Creating DPL tasks ...\r\n");

    TaskP_Params_init(&taskParams);
    taskParams.stackSize = TASK_DPL_STACK_SIZE;
    taskParams.stack = gDplTaskStack;
    taskParams.taskMain = task_dpl_main;
    status = TaskP_construct(&gDplTask, &taskParams);
    DebugP_assert(status==SystemP_SUCCESS);

    DebugP_log("Creating tasks ...\r\n");

    /* then create the tasks, order of task creation does not matter for this example */
    gPongTask = xTaskCreateStatic( pong_main,      /* Pointer to the function that implements the task. */
                                  "pong",          /* Text name for the task.  This is to facilitate debugging only. */
                                  PONG_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,            /* We are not using the task parameter. */
                                  PONG_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gPongTaskStack,  /* pointer to stack base */
                                  &gPongTaskObj ); /* pointer to statically allocated task object memory */
    configASSERT(gPongTask != NULL);

    gPingTask = xTaskCreateStatic( ping_main,      /* Pointer to the function that implements the task. */
                                  "ping",          /* Text name for the task.  This is to facilitate debugging only. */
                                  PING_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,            /* We are not using the task parameter. */
                                  PING_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gPingTaskStack,  /* pointer to stack base */
                                  &gPingTaskObj ); /* pointer to statically allocated task object memory */
    configASSERT(gPingTask != NULL);

    /* Dont close drivers to keep the UART driver open for console */
    /* Board_driversClose(); */
    /* Drivers_close(); */
}

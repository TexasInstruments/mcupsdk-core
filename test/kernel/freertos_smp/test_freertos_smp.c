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
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/TimerP.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <event_groups.h>
#include <unity.h>
#include "ti_drivers_open_close.h"

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

#define NUM_TASK_SWITCHES (100000u)

#define PING_INT_NUM (16u)
#define PONG_INT_NUM (17u)
#define SPI_INT_NUM0 (34u)
#define SPI_INT_NUM1 (35u)

#define PING_TASK_PRI (2u)
#define PONG_TASK_PRI (3u)

/* bit mask upto 24 bits */
#define EVENT_PING    (0x000001u)
#define EVENT_PONG    (0x000002u)

#define PING_INT_PRIORITY   (3u)
#define PONG_INT_PRIORITY   (4u)
#define SPI_INT_PRIORITY    (4u)

#define PING_TASK_SIZE (1024*4u)
uint8_t gPingTaskStack[PING_TASK_SIZE] __attribute__((aligned(32)));

#define PONG_TASK_SIZE (1024*4u)
uint8_t gPongTaskStack[PONG_TASK_SIZE] __attribute__((aligned(32)));

StaticEventGroup_t gEventObj;
EventGroupHandle_t gEvent;

TaskP_Object gPingTaskObj;
StaticSemaphore_t gPingSemObj;
SemaphoreHandle_t gPingSem;

TaskP_Object gPongTaskObj;
StaticSemaphore_t gPongSemObj;
SemaphoreHandle_t gPongSem;

StaticSemaphore_t gISRSemObj0;
SemaphoreHandle_t gISRSem0;

StaticSemaphore_t gISRSemObj1;
SemaphoreHandle_t gISRSem1;

StaticSemaphore_t gSpiIsrCompletionSemObj;
SemaphoreHandle_t gSpiIsrCompletionSem;

HwiP_Object gPingHwiObj;
HwiP_Object gPongHwiObj;
HwiP_Object gSpiIsrHwiObj0;
HwiP_Object gSpiIsrHwiObj1;

static uint32_t gCore1SpiIsrExecution0 = 0;
static uint32_t gCore1SpiIsrExecution1 = 0;

float floatLoadAndMultiply(float f1, float f2);
float floatMultiply();

int32_t mq_init( void );
void mq_dispatcher( void );
void mq_receiver( void );

double gFloat = 10.0;

extern uint32_t mq_receiveError;

static void ping_isr_1(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    xSemaphoreGiveFromISR(gPingSem, &doTaskSwitch); /* wake up ping task */
    portYIELD_FROM_ISR(doTaskSwitch);
}

static void ping_isr_2(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    xSemaphoreGiveFromISR(gPongSem, &doTaskSwitch); /* wake up pong task */
    portYIELD_FROM_ISR(doTaskSwitch);
}

static void ping_isr_3(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    vTaskNotifyGiveFromISR( TaskP_getHndl(&gPingTaskObj), &doTaskSwitch); /* wake up ping task */
    portYIELD_FROM_ISR(doTaskSwitch);
}

static void ping_isr_4(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    vTaskNotifyGiveFromISR( TaskP_getHndl(&gPongTaskObj), &doTaskSwitch); /* wake up pong task */
    portYIELD_FROM_ISR(doTaskSwitch);
}

static void ping_isr_5(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    xEventGroupSetBitsFromISR(gEvent, EVENT_PING, &doTaskSwitch); /* wake up ping task */
    portYIELD_FROM_ISR(doTaskSwitch);
}

static void pong_isr_2(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    xSemaphoreGiveFromISR(gPingSem, &doTaskSwitch); /* wake up ping task */
    portYIELD_FROM_ISR(doTaskSwitch);
}

static void pong_isr_4(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    vTaskNotifyGiveFromISR( TaskP_getHndl(&gPingTaskObj), &doTaskSwitch); /* wake up ping task */
    portYIELD_FROM_ISR(doTaskSwitch);
}

static void sgi_isr0(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    if(portGET_CORE_ID() != 0)
    {
        gCore1SpiIsrExecution0++;
    }

    xSemaphoreGiveFromISR(gISRSem0, &doTaskSwitch);
}

static void sgi_isr1(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    if(portGET_CORE_ID() != 0)
    {
        gCore1SpiIsrExecution1++;
    }

    xSemaphoreGiveFromISR(gISRSem1, &doTaskSwitch);
}

/* switch between ping and pong tasks using semaphores */
void test_taskSwitchWithSemaphore(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    uint32_t cycles;

    CycleCounterP_reset();

    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    cycles = CycleCounterP_getCount32();
    while (count--)
    {
        xSemaphoreGive(gPongSem);                /* wake up pong task */
        xSemaphoreTake(gPingSem, portMAX_DELAY); /* wait for pong to signal */
    }
    cycles = CycleCounterP_getCount32() - cycles;
    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_log("\r\n");
    DebugP_log("execution time for task switches = %" PRId64 " us, %d cycles\r\n", curTime, cycles);
    DebugP_log("number of task switches = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES * 2);
    DebugP_log("time per task switch (semaphore give/take) = %" PRId32 " ns, %d cycles\r\n",
        (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES * 2)),
        cycles/(NUM_TASK_SWITCHES * 2)
        );
}

/* switch between ping and pong tasks using direct-to-task notifications */
void test_taskSwitchWithTaskNotify(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;

    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        xTaskNotifyGive( TaskP_getHndl(&gPongTaskObj) );              /* wake up pong task */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* wait for pong to signal */
    }
    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_log("\r\n");
    DebugP_log("execution time for task switches = %" PRId64 " us\r\n", curTime);
    DebugP_log("number of task switches = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES * 2);
    DebugP_log("time per task switch (direct-to-task notification give/take) = %" PRId32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES * 2)));
}

/* switch between ping and pong tasks using event groups */
void test_taskSwitchWithEventGroups(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;

    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        xEventGroupSetBits(gEvent, EVENT_PONG); /* wake up pong task */
        xEventGroupWaitBits(gEvent, EVENT_PING, pdTRUE, pdTRUE, portMAX_DELAY); /* wait for pong to signal */
    }
    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_log("\r\n");
    DebugP_log("execution time for task switches = %" PRId64 " us\r\n", curTime);
    DebugP_log("number of task switches = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES * 2);
    DebugP_log("time per task switch (event group set/wait) = %" PRId32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES * 2)));
}

/* switch from ping to ISR and back to the same task using event group */
void test_taskToIsrUsingEventGroups(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    HwiP_Params hwiParams;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = PING_INT_NUM;
    hwiParams.callback = ping_isr_5;
    hwiParams.priority = PING_INT_PRIORITY;
    HwiP_construct(&gPingHwiObj, &hwiParams);

    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        HwiP_post(PING_INT_NUM);
        xEventGroupWaitBits(gEvent, EVENT_PING, pdTRUE, pdTRUE, portMAX_DELAY); /* wait for ISR to signal */
    }
    curTime = ClockP_getTimeUsec() - curTime;

    HwiP_destruct(&gPingHwiObj);

    DebugP_log("\r\n");
    DebugP_log("execution time for task - ISR - task switches = %" PRId64 " us\r\n", curTime);
    DebugP_log("number of task switches = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES);
    DebugP_log("time per task - ISR - task switch (event group set/wait) = %" PRId32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES)));
}

/* just invoke the task switch logic without any semaphores or direct-to-task notifications */
void test_taskYeild(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;

    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        taskYIELD();
    }
    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_log("\r\n");
    DebugP_log("execution time for task yields = %" PRId64 " us\r\n", curTime);
    DebugP_log("number of task yields = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES);
    DebugP_log("time per task yield = %" PRId32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES)));
}

/* switch from ping to ISR and back to the same task using semaphores, here there is no task switch */
void test_taskToIsrUsingSemaphoreAndNoTaskSwitch(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    HwiP_Params hwiParams;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = PING_INT_NUM;
    hwiParams.callback = ping_isr_1;
    hwiParams.priority = PING_INT_PRIORITY;
    HwiP_construct(&gPingHwiObj, &hwiParams);

    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        HwiP_post(PING_INT_NUM);
        xSemaphoreTake(gPingSem, portMAX_DELAY); /* wait for ISR to signal */
    }
    curTime = ClockP_getTimeUsec() - curTime;

    HwiP_destruct(&gPingHwiObj);

    DebugP_log("\r\n");
    DebugP_log("execution time for task - ISR - task switches = %" PRId64 " us\r\n", curTime);
    DebugP_log("number of task switches = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES);
    DebugP_log("time per task - ISR - task switch (semaphore give/take) = %" PRId32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES)));
}

/* switch from ping to ISR and back to the same task using direct-to-task notify, here there is no task switch */
void test_taskToIsrUsingTaskNotifyAndNoTaskSwitch(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;

    HwiP_Params hwiParams;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = PING_INT_NUM;
    hwiParams.callback = ping_isr_3;
    hwiParams.priority = PING_INT_PRIORITY;
    HwiP_construct(&gPingHwiObj, &hwiParams);

    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        HwiP_post(PING_INT_NUM);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* wait for ISR to signal */
    }
    curTime = ClockP_getTimeUsec() - curTime;

    HwiP_destruct(&gPingHwiObj);

    DebugP_log("\r\n");
    DebugP_log("execution time for task - ISR - task switches = %" PRId64 " us\r\n", curTime);
    DebugP_log("number of task switches = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES);
    DebugP_log("time per task - ISR - task switch (direct-to-task notification give/take) = %" PRId32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES)));
}

/* switch from ping task to ISR to pong task and back to ping task using semaphores, here there is a task switch */
void test_taskToIsrUsingSemaphoreAndWithTaskSwitch(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    HwiP_Params hwiParams;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = PING_INT_NUM;
    hwiParams.callback = ping_isr_2;
    hwiParams.priority = PING_INT_PRIORITY;
    HwiP_construct(&gPingHwiObj, &hwiParams);

    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        HwiP_post(PING_INT_NUM);
        xSemaphoreTake(gPingSem, portMAX_DELAY); /* wait for ISR to signal */
    }
    curTime = ClockP_getTimeUsec() - curTime;

    HwiP_destruct(&gPingHwiObj);

    DebugP_log("\r\n");
    DebugP_log("execution time for task - ISR - task - task switches = %" PRId64 " us\r\n", curTime);
    DebugP_log("number of ISRs = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES * 2);
    DebugP_log("time per task - ISR - task switch (semaphore give/take) = %" PRId32 " ns\r\n", (uint32_t)(curTime * 1000 / (2 * NUM_TASK_SWITCHES)));
}

/* switch from ping task to ISR to pong task and back to ping task using direct-to-task notify, here there is task switch */
void test_taskToIsrUsingTaskNotifyAndWithTaskSwitch(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    HwiP_Params hwiParams;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = PING_INT_NUM;
    hwiParams.callback = ping_isr_4;
    hwiParams.priority = PING_INT_PRIORITY;
    HwiP_construct(&gPingHwiObj, &hwiParams);

    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        HwiP_post(PING_INT_NUM);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* wait for ISR to signal */
    }
    curTime = ClockP_getTimeUsec() - curTime;

    HwiP_destruct(&gPingHwiObj);

    DebugP_log("\r\n");
    DebugP_log("execution time for task - ISR - task switches = %" PRId64 " us\r\n", curTime);
    DebugP_log("number of task switches = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES * 2);
    DebugP_log("time per task - ISR - task switch (direct-to-task notification give/take) = %" PRId32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES * 2)));
}

/* switch between ping and pong tasks and do float operations in between */
void test_taskSwitchWithFloatOperations(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    double f;

    f = 0.0;
    floatLoadAndMultiply(0.1, 0.1);
    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        f = f + floatMultiply();
        xSemaphoreGive(gPongSem);                /* wake up pong task */
        xSemaphoreTake(gPingSem, portMAX_DELAY); /* wait for pong to signal */
    }
    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_log("\r\n");
    DebugP_log("execution time for task switches = %" PRId64 " us\r\n", curTime);
    DebugP_log("number of task switches = %" PRId32 " \r\n", (uint32_t)NUM_TASK_SWITCHES * 2);
    DebugP_log("time per task switch (semaphore give/take) = %" PRId32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES * 2)));
    TEST_ASSERT_UINT32_WITHIN(1, (NUM_TASK_SWITCHES / 100), (uint32_t)f);
}

/* wait some msecs, this is just to show how delay API can be used,
* there is no need to delay before deleting the task
*/
void test_taskDelay(void *args)
{
    uint64_t curTime;
    uint32_t delay1 = 100*1000, delay2 = 110*1000; /* in usecs */

    curTime = ClockP_getTimeUsec();
    /* convert to ticks before pass to vTaskDelay */
    vTaskDelay( ClockP_usecToTicks(delay1) );
    vTaskDelay( ClockP_usecToTicks(delay2) );
    curTime = ClockP_getTimeUsec() - curTime;
    TEST_ASSERT_UINT32_WITHIN( 1000, (delay1 + delay2), (uint32_t)curTime);
}

void test_timerIsr(void *args)
{
    volatile uint32_t *pTimerIsrCount = (uint32_t *)args;
    *pTimerIsrCount = *pTimerIsrCount + 1;
}

void test_timer(void *args)
{
    volatile uint32_t timerIsrCount = 0;
    uint32_t delayInMs = 1000;

    /* pass timer ISR count value to ISR as argument */
    HwiP_setArgs(&gTimerHwiObj[CONFIG_TIMER0], (void*)&timerIsrCount);

    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

    ClockP_usleep( delayInMs * 1000 );

    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

    DebugP_log("Timer ISR count = %d\r\n", timerIsrCount);
    TEST_ASSERT_UINT32_WITHIN( 1000, ( 1000000u * delayInMs) / CONFIG_TIMER0_NSEC_PER_TICK_ACTUAL, timerIsrCount);
}

void test_taskLoad(void *args)
{
    TaskP_Load taskLoad;
    uint32_t cpuLoad;
    uint32_t minExpectedCpuLoad = 3000;

    /* We expect CPU load to be > 10% atleast */
    cpuLoad = TaskP_loadGetTotalCpuLoad();
    DebugP_log(" LOAD: CPU  = %2d.%2d %%\r\n", cpuLoad/100, cpuLoad%100 );

    TEST_ASSERT_GREATER_THAN_UINT32(minExpectedCpuLoad*2, cpuLoad);

    TaskP_loadGet(&gPingTaskObj, &taskLoad);
    DebugP_log(" LOAD: %s = %2d.%2d %%\r\n", taskLoad.name, taskLoad.cpuLoad/100, taskLoad.cpuLoad%100 );

    TEST_ASSERT_GREATER_THAN_UINT32(minExpectedCpuLoad, taskLoad.cpuLoad);

    TaskP_loadGet(&gPongTaskObj, &taskLoad);
    DebugP_log(" LOAD: %s = %2d.%2d %%\r\n", taskLoad.name, taskLoad.cpuLoad/100, taskLoad.cpuLoad%100 );

    TEST_ASSERT_GREATER_THAN_UINT32(minExpectedCpuLoad, taskLoad.cpuLoad);

    /* Reset all load statistics, CPU load statistics should be < 1% now */
    DebugP_log(" LOAD: reset load statistics !!!\r\n");
    TaskP_loadResetAll();

    vTaskDelay(pdMS_TO_TICKS(1000));

    cpuLoad = TaskP_loadGetTotalCpuLoad();
    DebugP_log(" LOAD: CPU  = %2d.%2d %%\r\n", cpuLoad/100, cpuLoad%100 );

    TEST_ASSERT_UINT32_WITHIN( 100, 100, cpuLoad);

    TaskP_loadGet(&gPingTaskObj, &taskLoad);
    DebugP_log(" LOAD: %s = %2d.%2d %%\r\n", taskLoad.name, taskLoad.cpuLoad/100, taskLoad.cpuLoad%100 );

    TEST_ASSERT_UINT32_WITHIN( 100, 100, taskLoad.cpuLoad);

    TaskP_loadGet(&gPongTaskObj, &taskLoad);
    DebugP_log(" LOAD: %s = %2d.%2d %%\r\n", taskLoad.name, taskLoad.cpuLoad/100, taskLoad.cpuLoad%100 );

    TEST_ASSERT_UINT32_WITHIN( 100, 100, taskLoad.cpuLoad);
}

/* Check if the task runs on the core to which affinity is set */
void test_coreAffinity(void *args)
{
    uint32_t coreId = (uint32_t)args;

    TEST_ASSERT_EQUAL_UINT32(coreId, portGET_CORE_ID());
}

/* Check if ISR for shared peripheral interrupts (SPI) runs only on Core 0 */
void test_spiIntrCoreAffiintiy(void *args)
{
    uint32_t count; /* loop `count` times */

    {
        HwiP_Params hwiParams;

        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = SPI_INT_NUM1;
        hwiParams.callback = sgi_isr1;
        hwiParams.priority = SPI_INT_PRIORITY;
        HwiP_construct(&gSpiIsrHwiObj1, &hwiParams);

        count = NUM_TASK_SWITCHES;
        while (count--)
        {
            HwiP_post(SPI_INT_NUM1);
            xSemaphoreTake(gISRSem1, portMAX_DELAY); /* wait for ISR to signal */
        }
        HwiP_destruct(&gSpiIsrHwiObj1);
    }

    xSemaphoreTake(gSpiIsrCompletionSem, portMAX_DELAY); /* wait for SGI ISR to signal completion */

    TEST_ASSERT_EQUAL_UINT32(gCore1SpiIsrExecution0, 0);
    TEST_ASSERT_EQUAL_UINT32(gCore1SpiIsrExecution1, 0);
}

/* Message queue transaction between tasks running on two cores */
void test_mqTransfer( void *args )
{
    mq_receiver();

    TEST_ASSERT_EQUAL_UINT32(mq_receiveError, 0);
}

void ping_main(void *args)
{
    /* Any task that uses the floating point unit MUST call portTASK_USES_FLOATING_POINT()
        * before any floating point instructions are executed.
        */
    portTASK_USES_FLOATING_POINT();

    UNITY_BEGIN();

    RUN_TEST(test_taskSwitchWithSemaphore, 2442, NULL);
    RUN_TEST(test_taskSwitchWithTaskNotify, 2441, NULL);
    RUN_TEST(test_taskSwitchWithEventGroups, 2444, NULL);
    RUN_TEST(test_taskToIsrUsingEventGroups, 2445, NULL);
    RUN_TEST(test_taskYeild, 2440, NULL);
    RUN_TEST(test_taskToIsrUsingSemaphoreAndNoTaskSwitch, 2439, NULL);
    RUN_TEST(test_taskToIsrUsingTaskNotifyAndNoTaskSwitch, 2438, NULL);
    RUN_TEST(test_taskToIsrUsingSemaphoreAndWithTaskSwitch, 2437, NULL);
    RUN_TEST(test_taskToIsrUsingTaskNotifyAndWithTaskSwitch, 2436, NULL);
    RUN_TEST(test_taskSwitchWithFloatOperations, 2435, NULL);

    RUN_TEST(test_taskDelay, 2434, NULL);
    RUN_TEST(test_timer, 2433, NULL);
    RUN_TEST(test_taskLoad, 2443, NULL);

    RUN_TEST(test_coreAffinity, 2446, (void *)1);
    RUN_TEST(test_spiIntrCoreAffiintiy, 2447, NULL);
    RUN_TEST(test_mqTransfer, 2448, NULL);

    UNITY_END();

    /* One MUST not return out of a FreeRTOS task instead one MUST call vTaskDelete */
    vTaskDelete(NULL);
}

void pong_main(void *args)
{
    uint32_t count; /* loop `count` times */

    /* Any task that uses the floating point unit MUST call portTASK_USES_FLOATING_POINT()
        * before any floating point instructions are executed.
        */
    portTASK_USES_FLOATING_POINT();

    count = NUM_TASK_SWITCHES;
    while (count--)
    {
        xSemaphoreTake(gPongSem, portMAX_DELAY); /* wait for ping to signal */
        xSemaphoreGive(gPingSem);                /* wakeup ping task */
    }

    count = NUM_TASK_SWITCHES;
    while (count--)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* wait for ping to signal */
        xTaskNotifyGive(TaskP_getHndl(&gPingTaskObj));              /* wake up ping task */
    }
    count = NUM_TASK_SWITCHES;
    while (count--)
    {
        xEventGroupWaitBits(gEvent, EVENT_PONG, pdTRUE, pdTRUE, portMAX_DELAY); /* wait for ping to signal */
        xEventGroupSetBits(gEvent, EVENT_PING); /* wake up ping task */
    }
    {
        HwiP_Params hwiParams;

        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = PONG_INT_NUM;
        hwiParams.callback = pong_isr_2;
        hwiParams.priority = PONG_INT_PRIORITY;
        HwiP_construct(&gPongHwiObj, &hwiParams);

        count = NUM_TASK_SWITCHES;
        while (count--)
        {
            xSemaphoreTake(gPongSem, portMAX_DELAY); /* wait for ISR to signal */
            HwiP_post(PONG_INT_NUM);
        }
        HwiP_destruct(&gPongHwiObj);
    }
    {
        HwiP_Params hwiParams;

        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = PONG_INT_NUM;
        hwiParams.callback = pong_isr_4;
        hwiParams.priority = PONG_INT_PRIORITY;
        HwiP_construct(&gPongHwiObj, &hwiParams);

        count = NUM_TASK_SWITCHES;
        while (count--)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* wait for ISR to signal */
            HwiP_post(PONG_INT_NUM);
        }
        HwiP_destruct(&gPongHwiObj);
    }
    { /* switch between ping and pong tasks and do float operations in between */
        double f;

        f = 10.0;

        count = NUM_TASK_SWITCHES;
        while (count--)
        {
            f = f + floatLoadAndMultiply(0.1, 1.0);
            xSemaphoreTake(gPongSem, portMAX_DELAY); /* wait for ping to signal */
            xSemaphoreGive(gPingSem);                /* wakeup ping task */
        }
    }
    {
        HwiP_Params hwiParams;

        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = SPI_INT_NUM0;
        hwiParams.callback = sgi_isr0;
        hwiParams.priority = SPI_INT_PRIORITY;
        HwiP_construct(&gSpiIsrHwiObj0, &hwiParams);

        count = NUM_TASK_SWITCHES;
        while (count--)
        {
            HwiP_post(SPI_INT_NUM0);
            xSemaphoreTake(gISRSem0, portMAX_DELAY); /* wait for ISR to signal */
        }
        HwiP_destruct(&gSpiIsrHwiObj0);

        /* Give semaphore when SGI interrupt execution is complete  */
        xSemaphoreGive(gSpiIsrCompletionSem);
    }

    {
        /* Call function to send messages to mqueue */
        mq_dispatcher();
    }

    /* One MUST not return out of a FreeRTOS task instead one MUST call vTaskDelete */
    vTaskSuspend(NULL);
}

void setUp(void)
{
}

void tearDown(void)
{
}

void test_freertos_smp_main(void *args)
{
    int32_t status;
    TaskP_Params taskParams;

    /* Open drivers to open the UART driver for console */
    Drivers_open();

    /* first create the semaphores */
    gPingSem = xSemaphoreCreateBinaryStatic(&gPingSemObj);
    configASSERT(gPingSem != NULL);

    gPongSem = xSemaphoreCreateBinaryStatic(&gPongSemObj);
    configASSERT(gPongSem != NULL);

    gISRSem0 = xSemaphoreCreateBinaryStatic(&gISRSemObj0);
    configASSERT(gISRSem0 != NULL);

    gISRSem1 = xSemaphoreCreateBinaryStatic(&gISRSemObj1);
    configASSERT(gISRSem1 != NULL);

    gSpiIsrCompletionSem = xSemaphoreCreateBinaryStatic(&gSpiIsrCompletionSemObj);
    configASSERT(gSpiIsrCompletionSem != NULL);

    gEvent = xEventGroupCreateStatic(&gEventObj);
    configASSERT(gEvent != NULL);

    status = mq_init();
    DebugP_assert(status==SystemP_SUCCESS);

    /* then create the tasks, order of task creation does not matter for this example */

    TaskP_Params_init(&taskParams);
    taskParams.name = "pong";
    taskParams.stackSize = PONG_TASK_SIZE;
    taskParams.stack = gPongTaskStack;
    taskParams.priority = PONG_TASK_PRI;
    taskParams.args = NULL;
    taskParams.taskMain = pong_main;
    /* Set core Affinity to run pong task on Core 0 only */
    taskParams.coreAffinity = 1;
    status = TaskP_construct(&gPongTaskObj, &taskParams);
    DebugP_assert(status==SystemP_SUCCESS);



    TaskP_Params_init(&taskParams);
    taskParams.name = "ping";
    taskParams.stackSize = PING_TASK_SIZE;
    taskParams.stack = gPingTaskStack;
    taskParams.priority = PING_TASK_PRI;
    taskParams.args = NULL;
    taskParams.taskMain = ping_main;
    /* Set core Affinity to run ping task on Core 1 only */
    taskParams.coreAffinity = 2;
    status = TaskP_construct(&gPingTaskObj, &taskParams);
    DebugP_assert(status==SystemP_SUCCESS);

    /* Dont close drivers to keep the UART driver open for console */
    /* Drivers_close(); */
}

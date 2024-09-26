/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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
#include <tx_api.h>
#include <unity.h>
#include "ti_drivers_open_close.h"
#if defined(__ARM_ARCH_7R__)
#include <stdatomic.h>
#endif

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

#define NUM_TASK_SWITCHES (100000u)

#define PING_INT_NUM (16u)
#define PONG_INT_NUM (17u)

#define PING_TASK_PRI (2u)
#define PONG_TASK_PRI (3u)

#define HIGH_PRI_INT_NUM    (20u)
#define HIGH_INT_PRI        (3u)

#define LOW_PRI_INT_NUM    (21u)
#define LOW_INT_PRI        (10u)

/* bit mask upto 24 bits */
#define EVENT_PING    (0x000001u)
#define EVENT_PONG    (0x000002u)

#define PING_INT_PRIORITY   (3u)
#define PONG_INT_PRIORITY   (4u)

#define PING_TASK_SIZE (1024*4u)
uint8_t gPingTaskStack[PING_TASK_SIZE] __attribute__((aligned(32)));

#define PONG_TASK_SIZE (1024*4u)
uint8_t gPongTaskStack[PONG_TASK_SIZE] __attribute__((aligned(32)));


TX_THREAD gPongThread;
TX_THREAD pPingThread;

TX_SEMAPHORE gPingSem;
TX_SEMAPHORE gPongSem;

TX_EVENT_FLAGS_GROUP gEventFlags;

HwiP_Object gPingHwiObj;
HwiP_Object gPongHwiObj;
HwiP_Object gHighPriHwiObj;
HwiP_Object gLowPriHwiObj;

/* Delay in seconds, to allow execution of ISRs */
#define APP_DELAY_SEC           (1)

/* Counter to simulate delay inside critical section */
#define LOOP_COUNT          (1000 * 1000)


/* Loop variables used in critical section to simulate delay */
volatile int32_t gDelayCnt = 0;

/* Variables used to track isr trigger count*/
volatile int32_t gHighPriTriggerCnt = 0;
volatile int32_t gLowPriTriggerCnt = 0;


float floatLoadAndMultiply(float f1, float f2);
float floatMultiply();

double gFloat = 10.0;

atomic_uint gAtomicIntCounter;
uint32_t gNonAtomicIntCounter;



#if defined(__ARM_ARCH_7R__)
#define ATOMIC_TEST_LOOP_COUNT  (1000000u)
#define ATOMIC_TEST_NUM_TASKS   (4u)
#define ATOMIC_TEST_TASK_PRI    (2u)
#define ATOMIC_TEST_TASK_SIZE   (128*4u)
uint8_t gAtomicTestTaskStack[ATOMIC_TEST_NUM_TASKS][ATOMIC_TEST_TASK_SIZE] __attribute__((aligned(32)));


TaskP_Object gAtomicTestTaskObj[ATOMIC_TEST_NUM_TASKS];

TX_SEMAPHORE g_atomic_test_sem[ATOMIC_TEST_NUM_TASKS];

#endif









static void ping_isr_1(void *arg)
{
    UINT status;

    status = tx_semaphore_put(&gPingSem);
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
}

static void ping_isr_2(void *arg)
{
    UINT status;

    status = tx_semaphore_put(&gPongSem);
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
}

static void ping_isr_5(void *arg)
{
    UINT status;

    status = tx_event_flags_set(&gEventFlags, EVENT_PING, TX_OR);
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
}

static void ping_isr_6(void *arg)
{
    UINT status;

    gFloat = gFloat + floatLoadAndMultiply(0.1, 1.0);

    status = tx_semaphore_put(&gPingSem);
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
}


static void pong_isr_2(void *arg)
{
    UINT status;

    status = tx_semaphore_put(&gPingSem); /* wake up ping task */
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
}


/* High Priority ISR */
static void high_pri_isr(void *arg)
{
    /* Increment the count variable */
    gHighPriTriggerCnt++;
}

/* Low Priority ISR */
static void low_pri_isr(void *arg)
{
    /* Increment the count variable */
    gLowPriTriggerCnt++;
}


/* Enable & Trigger high priority interrupts during critical section
 *  Interrupts with priority greater than configMAX_SYSCALL_INTERRUPT_PRIORITY are enabled during critical section
 */
void test_isrEnableWithCriticalSection (void *args)
{
    TX_INTERRUPT_SAVE_AREA;
    int32_t  status = SystemP_SUCCESS;
    HwiP_Params hwiParams;

    int32_t oldHighPriTriggerCnt    = 0;
    int32_t oldLowPriTriggerCnt     = 0;

    /* Configure high priority interrupt, enabled during threadx critical section */
    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = HIGH_PRI_INT_NUM;
    hwiParams.callback = high_pri_isr;
    hwiParams.priority = HIGH_INT_PRI;
    HwiP_construct(&gHighPriHwiObj, &hwiParams);

    /* Configure low priority interrupt, disabled during threadx critical section */
    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = LOW_PRI_INT_NUM;
    hwiParams.callback = low_pri_isr;
    hwiParams.priority = LOW_INT_PRI;
    HwiP_construct(&gLowPriHwiObj, &hwiParams);

    DebugP_log("Entering critical section.\r\n");
    /* Enter threadx critical section */
    TX_DISABLE;

    HwiP_post(HIGH_PRI_INT_NUM);
    HwiP_post(LOW_PRI_INT_NUM);

    /* Wait for isr execution */
    while (gDelayCnt < LOOP_COUNT)
    {
        gDelayCnt++;
    }

    /* Check execution counts for both interrupts */
    if(gHighPriTriggerCnt  != 1)
    {
        status = SystemP_FAILURE;
    }

    if(gLowPriTriggerCnt != 0)
    {
        status = SystemP_FAILURE;
    }

    oldHighPriTriggerCnt = gHighPriTriggerCnt;
    oldLowPriTriggerCnt = gLowPriTriggerCnt;

    /* Exit threadx critical section */
    TX_RESTORE;

    DebugP_log("Exited critical section.\r\n");
    DebugP_log("High priority interrupt count inside critical section: %d\r\n",oldHighPriTriggerCnt);
    DebugP_log("Low priority interrupt count inside critical section:  %d\r\n",oldLowPriTriggerCnt);

    if(status == SystemP_SUCCESS)
    {
        HwiP_post(HIGH_PRI_INT_NUM);
        HwiP_post(LOW_PRI_INT_NUM);

        /* Wait for isr execution */
        ClockP_sleep(APP_DELAY_SEC);

        /* Check execution counts for both interrupts */
        if(gHighPriTriggerCnt  != 2)
        {
            status = SystemP_FAILURE;
        }

        if(gLowPriTriggerCnt != 2)
        {
            status = SystemP_FAILURE;
        }
    }

    DebugP_log("\r\nHigh priority interrupt count outside critical section: %d\r\n",gHighPriTriggerCnt);
    DebugP_log("Low priority interrupt count outside critical section:  %d\r\n",gLowPriTriggerCnt);

    HwiP_destruct(&gHighPriHwiObj);
    HwiP_destruct(&gLowPriHwiObj);

    TEST_ASSERT_INT32_WITHIN(0,0,status);
}


/* switch between ping and pong tasks using semaphores */
void test_taskSwitchWithSemaphore(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    uint32_t cycles;
    UINT status;

    CycleCounterP_reset();

    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    cycles = CycleCounterP_getCount32();
    while (count--)
    {
        status = tx_semaphore_put(&gPongSem);                /* wake up pong task */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

        status = tx_semaphore_get(&gPingSem, TX_WAIT_FOREVER); /* wait for pong to signal */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
    }
    cycles = CycleCounterP_getCount32() - cycles;
    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_log("\r\n");
    DebugP_log("execution time for task switches = %" PRIu64 " us, %"PRIu32" cycles\r\n", curTime, cycles);
    DebugP_log("number of task switches = %" PRIu32 " \r\n", (uint32_t)NUM_TASK_SWITCHES * 2);
    DebugP_log("time per task switch (semaphore give/take) = %" PRIu32 " ns, %"PRIu32" cycles\r\n",
        (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES * 2)),
        cycles/(NUM_TASK_SWITCHES * 2)
        );
}


/* switch between ping and pong tasks using event groups */
void test_taskSwitchWithEventGroups(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    ULONG flags;
    UINT status;

    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        status = tx_event_flags_set(&gEventFlags, EVENT_PONG, TX_OR); /* wake up pong task */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

        status = tx_event_flags_get(&gEventFlags, EVENT_PING, TX_AND_CLEAR, &flags, TX_WAIT_FOREVER); /* wait for pong to signal */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
    }
    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_log("\r\n");
    DebugP_log("execution time for task switches = %" PRIu64 " us\r\n", curTime);
    DebugP_log("number of task switches = %" PRIu32 " \r\n", (uint32_t)NUM_TASK_SWITCHES * 2);
    DebugP_log("time per task switch (event group set/wait) = %" PRIu32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES * 2)));
}

/* switch from ping to ISR and back to the same task using event group */
void test_taskToIsrUsingEventGroups(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    HwiP_Params hwiParams;
    ULONG flags;
    UINT status;

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
        status = tx_event_flags_get(&gEventFlags, EVENT_PING, TX_AND_CLEAR, &flags, TX_WAIT_FOREVER);
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
    }
    curTime = ClockP_getTimeUsec() - curTime;

    HwiP_destruct(&gPingHwiObj);

    DebugP_log("\r\n");
    DebugP_log("execution time for task - ISR - task switches = %" PRIu64 " us\r\n", curTime);
    DebugP_log("number of task switches = %" PRIu32 " \r\n", (uint32_t)NUM_TASK_SWITCHES);
    DebugP_log("time per task - ISR - task switch (event group set/wait) = %" PRIu32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES)));
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
        tx_thread_relinquish();
    }
    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_log("\r\n");
    DebugP_log("execution time for task yields = %" PRIu64 " us\r\n", curTime);
    DebugP_log("number of task yields = %" PRIu32 " \r\n", (uint32_t)NUM_TASK_SWITCHES);
    DebugP_log("time per task yield = %" PRIu32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES)));
}

/* switch from ping to ISR and back to the same task using semaphores, here there is no task switch */
void test_taskToIsrUsingSemaphoreAndNoTaskSwitch(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    HwiP_Params hwiParams;
    UINT status;

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
        status = tx_semaphore_get(&gPingSem, TX_WAIT_FOREVER); /* wait for ISR to signal */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
    }
    curTime = ClockP_getTimeUsec() - curTime;

    HwiP_destruct(&gPingHwiObj);

    DebugP_log("\r\n");
    DebugP_log("execution time for task - ISR - task switches = %" PRIu64 " us\r\n", curTime);
    DebugP_log("number of task switches = %" PRIu32 " \r\n", (uint32_t)NUM_TASK_SWITCHES);
    DebugP_log("time per task - ISR - task switch (semaphore give/take) = %" PRIu32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES)));
}


/* switch from ping task to ISR to pong task and back to ping task using semaphores, here there is a task switch */
void test_taskToIsrUsingSemaphoreAndWithTaskSwitch(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    HwiP_Params hwiParams;
    UINT status;

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
        status = tx_semaphore_get(&gPingSem, TX_WAIT_FOREVER); /* wait for ISR to signal */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
    }
    curTime = ClockP_getTimeUsec() - curTime;

    HwiP_destruct(&gPingHwiObj);

    DebugP_log("\r\n");
    DebugP_log("execution time for task - ISR - task - task switches = %" PRIu64 " us\r\n", curTime);
    DebugP_log("number of ISRs = %" PRIu32 " \r\n", (uint32_t)NUM_TASK_SWITCHES * 2);
    DebugP_log("time per task - ISR - task switch (semaphore give/take) = %" PRIu32 " ns\r\n", (uint32_t)(curTime * 1000 / (2 * NUM_TASK_SWITCHES)));
}


/* switch between ping and pong tasks and do float operations in between */
void test_taskSwitchWithFloatOperations(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    double f;
    UINT status;

    f = 0.0;
    floatLoadAndMultiply(0.1, 0.1);
    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        f = f + floatMultiply();
        status = tx_semaphore_put(&gPongSem);                /* wake up pong task */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

        status = tx_semaphore_get(&gPingSem, TX_WAIT_FOREVER); /* wait for pong to signal */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
    }
    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_log("\r\n");
    DebugP_log("execution time for task switches = %" PRIu64 " us\r\n", curTime);
    DebugP_log("number of task switches = %" PRIu32 " \r\n", (uint32_t)NUM_TASK_SWITCHES * 2);
    DebugP_log("time per task switch (semaphore give/take) = %" PRIu32 " ns\r\n", (uint32_t)(curTime * 1000 / (NUM_TASK_SWITCHES * 2)));
    TEST_ASSERT_UINT32_WITHIN(1, (NUM_TASK_SWITCHES / 100), (uint32_t)f);
}

/* switch between ping and pong tasks and do float operations in between */
void test_taskToIsrWithFloatOperations(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    double f;
    HwiP_Params hwiParams;
    UINT status;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = PING_INT_NUM;
    hwiParams.callback = ping_isr_6;
    hwiParams.priority = PING_INT_PRIORITY;
    HwiP_construct(&gPingHwiObj, &hwiParams);



    gFloat = 10.0;
    f = 0.0;
    floatLoadAndMultiply(0.1, 0.1);
    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        f = f + floatMultiply();
        HwiP_post(PING_INT_NUM);
        status = tx_semaphore_get(&gPingSem, TX_WAIT_FOREVER); /* wait for pong to signal */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
    }
    curTime = ClockP_getTimeUsec() - curTime;

    HwiP_destruct(&gPingHwiObj);

    DebugP_log("\r\n");
    DebugP_log("execution time for task - ISR - task switches = %" PRIu64 " us\r\n", curTime);
    DebugP_log("number of ISRs = %" PRIu32 " \r\n", (uint32_t)NUM_TASK_SWITCHES * 2);
    DebugP_log("time per task - ISR - task switch (semaphore give/take) = %" PRIu32 " ns\r\n", (uint32_t)(curTime * 1000 / (2 * NUM_TASK_SWITCHES)));
    TEST_ASSERT_UINT32_WITHIN(1, (NUM_TASK_SWITCHES / 100), (uint32_t)f);
    TEST_ASSERT_UINT32_WITHIN(1, 10 + (NUM_TASK_SWITCHES / 10), (uint32_t)gFloat);
}

/* switch between ping task and fiq isr and do float operations in both */
void test_taskToFiqIsrWithFloatOperations(void *args)
{
    uint32_t count; /* loop `count` times */
    uint64_t curTime;
    double f;
    HwiP_Params hwiParams;
    UINT status;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = PING_INT_NUM;
    hwiParams.callback = ping_isr_6;
    hwiParams.isFIQ = 1;
    hwiParams.priority = PING_INT_PRIORITY;
    HwiP_construct(&gPingHwiObj, &hwiParams);



    gFloat = 10.0;
    f = 0.0;
    floatLoadAndMultiply(0.1, 0.1);
    count = NUM_TASK_SWITCHES;
    curTime = ClockP_getTimeUsec();
    while (count--)
    {
        f = f + floatMultiply();
        HwiP_post(PING_INT_NUM);
        status = tx_semaphore_get(&gPingSem, TX_WAIT_FOREVER); /* wait for fiq isr to signal */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
    }
    curTime = ClockP_getTimeUsec() - curTime;

    HwiP_destruct(&gPingHwiObj);

    DebugP_log("\r\n");
    DebugP_log("execution time for task - ISR - task switches = %" PRIu64 " us\r\n", curTime);
    DebugP_log("number of ISRs = %" PRIu32 " \r\n", (uint32_t)NUM_TASK_SWITCHES * 2);
    DebugP_log("time per task - ISR - task switch (semaphore give/take) = %" PRIu32 " ns\r\n", (uint32_t)(curTime * 1000 / (2 * NUM_TASK_SWITCHES)));
    TEST_ASSERT_UINT32_WITHIN(1, (NUM_TASK_SWITCHES / 100), (uint32_t)f);
    TEST_ASSERT_UINT32_WITHIN(1, 10 + (NUM_TASK_SWITCHES / 10), (uint32_t)gFloat);
}

/* wait some msecs, this is just to show how delay API can be used,
* there is no need to delay before deleting the task
*/
void test_taskDelay(void *args)
{
    uint64_t curTime;
    uint64_t delay1 = 100*1000, delay2 = 110*1000; /* in usecs */
    UINT status;

    curTime = ClockP_getTimeUsec();
    /* convert to ticks before pass to vTaskDelay */
    status = tx_thread_sleep( delay1 * TX_TIMER_TICKS_PER_SECOND / 1000000ul);
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

    status = tx_thread_sleep( delay2 * TX_TIMER_TICKS_PER_SECOND / 1000000ul);
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

    curTime = ClockP_getTimeUsec() - curTime;
    TEST_ASSERT_UINT32_WITHIN( 1000, (delay1 + delay2), (uint32_t)curTime);
}


void test_timerIsr(void *args)
{
    volatile uint32_t *pTimerIsrCount = (uint32_t *)args;
    *pTimerIsrCount = *pTimerIsrCount + 1;

#if defined(__ARM_ARCH_7R__)
    gAtomicIntCounter++;
    gNonAtomicIntCounter++;
#endif
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

    DebugP_log("Timer ISR count = %"PRIu32"\r\n", timerIsrCount);
    TEST_ASSERT_UINT32_WITHIN( 1000, ( 1000000u * delayInMs) / CONFIG_TIMER0_NSEC_PER_TICK_ACTUAL, timerIsrCount);
}


#if defined(__ARM_ARCH_7R__)
void test_atomicTaskMain(void *args)
{
    uint32_t taskId = (uint32_t)args;
    uint32_t i, loopCnt = ATOMIC_TEST_LOOP_COUNT;
    UINT status;

    for(i=0; i<loopCnt; i++)
    {
        gAtomicIntCounter++;
        gNonAtomicIntCounter++;
    }

    status = tx_semaphore_put(&g_atomic_test_sem[taskId]);
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

    status = tx_thread_suspend(tx_thread_identify());
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
}

/*
 * This test will check operation from stdatomics.h
 * This checks atomic increments between the current task and ISR.
 * And then checks atomic increments between multiple tasks as well as ISR
 * For the task to switch asychronoulsy, we should ideally make
 * configUSE_TIME_SLICING as 1, since will cause a task switch while the variable is being incremented
 * However even if not 1, the test will still pass.
 */
void test_atomics(void *args)
{
    volatile uint32_t timerIsrCount;
    uint32_t i, loopCnt = ATOMIC_TEST_LOOP_COUNT;
    UINT status;

    /* pass timer ISR count value to ISR as argument */
    HwiP_setArgs(&gTimerHwiObj[CONFIG_TIMER0], (void*)&timerIsrCount);

    /* test atomics with current task and ISR */
    timerIsrCount = 0;
    gAtomicIntCounter = 0;
    gNonAtomicIntCounter = 0;

    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

    for(i=0; i<loopCnt; i++)
    {
        gAtomicIntCounter++;
        gNonAtomicIntCounter++;
    }

    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

    TEST_ASSERT_EQUAL_UINT32( timerIsrCount + loopCnt, gAtomicIntCounter );
    TEST_ASSERT_NOT_EQUAL_UINT32( timerIsrCount + loopCnt, gNonAtomicIntCounter );

    /* test atomics with multiple task's and ISR */
    for(i=0; i<ATOMIC_TEST_NUM_TASKS; i++)
    {
        status = tx_semaphore_create(&g_atomic_test_sem[i], "atomic_test_sem", 0);
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
    }

    timerIsrCount = 0;
    gAtomicIntCounter = 0;
    gNonAtomicIntCounter = 0;

    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

    for(i=0; i<ATOMIC_TEST_NUM_TASKS; i++)
    {
        int32_t status;
        TaskP_Params taskParams;

        TaskP_Params_init(&taskParams);
        taskParams.name = "ATOMIC TASK";
        taskParams.stackSize = ATOMIC_TEST_TASK_SIZE;
        taskParams.stack = &gAtomicTestTaskStack[i][0];
        taskParams.priority = ATOMIC_TEST_TASK_PRI;
        taskParams.args = (void*)i;
        taskParams.taskMain = test_atomicTaskMain;
        status = TaskP_construct(&gAtomicTestTaskObj[i], &taskParams);
        DebugP_assert(status == SystemP_SUCCESS);
    }
    for(i=0; i<ATOMIC_TEST_NUM_TASKS; i++)
    {
        tx_semaphore_get(&g_atomic_test_sem[i], TX_WAIT_FOREVER);
        tx_semaphore_delete(&g_atomic_test_sem[i]);
    }

    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

    for(i=0; i<ATOMIC_TEST_NUM_TASKS; i++)
    {
        TaskP_destruct(&gAtomicTestTaskObj[i]);
    }

    TEST_ASSERT_EQUAL_UINT32( timerIsrCount + ATOMIC_TEST_NUM_TASKS*ATOMIC_TEST_LOOP_COUNT, gAtomicIntCounter );
    TEST_ASSERT_NOT_EQUAL_UINT32( timerIsrCount + ATOMIC_TEST_NUM_TASKS*ATOMIC_TEST_LOOP_COUNT, gNonAtomicIntCounter );
}
#endif


void ping_main(ULONG args)
{
    UNITY_BEGIN();


    RUN_TEST(test_taskSwitchWithSemaphore, 20100, NULL);
    RUN_TEST(test_taskSwitchWithEventGroups, 20101, NULL);
    RUN_TEST(test_taskToIsrUsingEventGroups, 20102, NULL);

    RUN_TEST(test_taskYeild, 20103, NULL);
    RUN_TEST(test_taskToIsrUsingSemaphoreAndNoTaskSwitch, 20104, NULL);
    RUN_TEST(test_taskToIsrUsingSemaphoreAndWithTaskSwitch, 20105, NULL);

    RUN_TEST(test_taskSwitchWithFloatOperations, 20106, NULL);

#if defined(__ARM_ARCH_7R__)
    /* floating point operations in ISR supported in R5F only */
    RUN_TEST(test_taskToIsrWithFloatOperations, 20107, NULL);
#endif


    RUN_TEST(test_taskDelay, 20108, NULL);
    RUN_TEST(test_timer, 20109, NULL);


#if defined(__ARM_ARCH_7R__)
    /* atomics not tested with other architectures */
    RUN_TEST(test_atomics, 20110, NULL);
#endif


#if defined(__ARM_ARCH_7R__)

    #ifdef EN_MAX_SYSCALL_INTR_PRI_CRIT_SECTION
    /** Interrupts inside critical section supported in R5F only. Not supported in am273x.
     *  EN_MAX_SYSCALL_INTR_PRI_CRIT_SECTION  defined in source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F/portmacro.h
     * MUST be uncommented for this test to pass.
    */
    RUN_TEST(test_isrEnableWithCriticalSection, 20111, NULL);
    #endif


    #ifdef EN_SAVE_RESTORE_FPU_CONTEXT
    /** floating point operations in FIQ ISR supported in R5F only.
     *  Make sure the macro EN_SAVE_RESTORE_FPU_CONTEXT is uncommented in source/kernel/dpl/HwiP.h, otherwise the test will fail.
    */
    RUN_TEST(test_taskToFiqIsrWithFloatOperations, 20112, NULL);
    #endif
#endif


    UNITY_END();
}

void pong_main(ULONG args)
{
    ULONG flags;
    uint32_t count; /* loop `count` times */
    UINT status;

    count = NUM_TASK_SWITCHES;
    while (count--)
    {
        status = tx_semaphore_get(&gPongSem, TX_WAIT_FOREVER); /* wait for ping to signal */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

        status = tx_semaphore_put(&gPingSem);                /* wakeup ping task */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
    }
    count = NUM_TASK_SWITCHES;
    while (count--)
    {
        status = tx_event_flags_get(&gEventFlags, EVENT_PONG, TX_AND_CLEAR, &flags, TX_WAIT_FOREVER); /* wait for ping to signal */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

        status = tx_event_flags_set(&gEventFlags, EVENT_PING, TX_OR); /* wake up ping task */
        TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
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
            status = tx_semaphore_get(&gPongSem, TX_WAIT_FOREVER); /* wait for ISR to signal */
            TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

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

            status = tx_semaphore_get(&gPongSem, TX_WAIT_FOREVER); /* wait for ping to signal */
            TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

            tx_semaphore_put(&gPingSem);                /* wakeup ping task */
            TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);
        }
    }
}


void setUp(void)
{
}

void tearDown(void)
{
}



void test_main(ULONG args)
{
    UINT status;

    /* Open drivers to open the UART driver for console */
    Drivers_open();

    status = tx_semaphore_create(&gPingSem, "ping_sem", 0);
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

    status = tx_semaphore_create(&gPongSem, "pong_sem", 0);
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

    status = tx_event_flags_create(&gEventFlags, "event_flags");
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

    status = tx_thread_create(&gPongThread, "pong", pong_main, 0, gPongTaskStack,
                              PONG_TASK_SIZE, PONG_TASK_PRI, PONG_TASK_PRI, TX_NO_TIME_SLICE, TX_AUTO_START);
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

    status = tx_thread_create(&pPingThread, "ping", ping_main, 0, gPingTaskStack,
                              PING_TASK_SIZE, PING_TASK_PRI, PING_TASK_PRI, TX_NO_TIME_SLICE, TX_AUTO_START);
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

    status = tx_thread_terminate(tx_thread_identify());
    TEST_ASSERT_EQUAL_UINT32(status, TX_SUCCESS);

    /* Dont close drivers to keep the UART driver open for console */
    /* Drivers_close(); */
}

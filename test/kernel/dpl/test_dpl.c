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
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HeapP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/EventP.h>
#include <kernel/dpl/QueueP.h>
#include <kernel/dpl/MailboxP.h>
#include <drivers/soc.h>
#include <unity.h>
#include "ti_drivers_open_close.h"

#define TEST_INT_NUM     (20U)

/* Event bit mask upto 24 bits */
#define EVENT_BIT_FROM_PING             (0x000001u)
#define EVENT_BIT_FROM_PONG             (0x000002u)
#define EVENT_BIT_FROM_ISR              (0x100000u)
#define EVENT_BIT2_FROM_ISR             (0x200000u)

#define EVENT_TASK_PRI             (14U)    /* One less than highest priority kernel timer task */
#define EVENT_TASK_STACK_SIZE      (4*1024U)
static uint8_t gTaskStack[EVENT_TASK_STACK_SIZE] __attribute__((aligned(32)));
static TaskP_Object gEventTask;
static EventP_Object gMyEvent;
int32_t gEventSetStatusFromISR;
int32_t gEventSet2StatusFromISR;
int32_t gEventClearStatusFromISR;
int32_t gEventGetBitsStatusFromISR;
uint32_t gEventGetBitsFromISR;

/* MailboxP Test Object and Definitions */
#define TEST_MBOX_TASK_STACK_SIZE      (4*1024U)
#define TEST_MBOX_TASK1_PRIO             (14U)
#define TEST_MBOX_TASK2_PRIO             (14U)
#define TEST_MBOX_MSG_SIZE               (10U)
#define TEST_MBOX_BUFF_COUNT             (3U)

struct test_mboxTaskTestParam
{
    MailboxP_Handle hMboxClientRx;
    MailboxP_Handle hMboxClientTx;
};

struct test_mboxIsrTestParam
{
    MailboxP_Handle hMboxClientRx;
    MailboxP_Handle hMboxClientTx;
    int32_t numMsgs1;
    int32_t numMsgs2;
    int32_t status1AtIsr;
    int32_t status2AtIsr;
    uint8_t msgAtIsr[TEST_MBOX_MSG_SIZE];
};

static uint8_t gTestMboxTask1Stack[EVENT_TASK_STACK_SIZE] __attribute__((aligned(32)));
static MailboxP_Object gMyMboxClientTx;
static MailboxP_Object gMyMboxClientRx;
static TaskP_Object gTestMboxTaskObj;
static uint8_t gMailBoxBuff[TEST_MBOX_MSG_SIZE*TEST_MBOX_BUFF_COUNT];


/* User defined heap memory and handle */
#define MY_HEAP_MEM_SIZE (2 * 1024u)
static uint8_t gMyHeapMem[MY_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

volatile uint32_t gMyISRCount;
volatile uint32_t gInISR;

static SemaphoreP_Object gMyDoneSem;
static ClockP_Object gMyClock;
static HeapP_Object gMyHeap;

#define MY_TASK_PRI         (14U)   /* One less than highest priority kernel timer task */
#define MY_TASK_STACK_SIZE  (4*1024U)
static uint8_t gMyTaskStack[MY_TASK_STACK_SIZE] __attribute__((aligned(32)));
static TaskP_Object gMyTask;

#if defined(_TMS320C6X)
void test_c66x(void);
#endif

static void myISR1(void *args)
{
    gMyISRCount++;

    gInISR += HwiP_inISR();
}

static void myISR2(void *args)
{
    SemaphoreP_Object *pSemObj = (SemaphoreP_Object *)args;

    SemaphoreP_post(pSemObj);
}

static void myISR3(void *args)
{
    gInISR = 1;
}

void test_hwiProfile(void *args)
{
    HwiP_Params hwiParams;
    HwiP_Object hwiObj;
    int32_t status;
    uint32_t i, cycles, maxCount = 10000;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = TEST_INT_NUM;
    hwiParams.callback = myISR3;
    status = HwiP_construct(&hwiObj, &hwiParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    CycleCounterP_reset();
    cycles = CycleCounterP_getCount32();
    for (i = 0; i < maxCount; i++)
    {
        gInISR = 0;
        HwiP_post(hwiParams.intNum);
        while (gInISR == 0)
            ;
    }
    cycles = CycleCounterP_getCount32() - cycles;

    HwiP_destruct(&hwiObj);

    DebugP_log("\r\n");
    DebugP_log(" Without SemaphoreP,\r\n");
    DebugP_log("  number of interrupts = %d\r\n", maxCount);
    DebugP_log("  total cycles = %d\r\n", cycles);
    DebugP_log("  cycles per interrupt = %d\r\n", cycles/maxCount);

    status = SemaphoreP_constructBinary(&gMyDoneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = TEST_INT_NUM;
    hwiParams.callback = myISR2;
    hwiParams.args = &gMyDoneSem;
    status = HwiP_construct(&hwiObj, &hwiParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    CycleCounterP_reset();
    cycles = CycleCounterP_getCount32();
    for (i = 0; i < maxCount; i++)
    {
        gInISR = 0;
        HwiP_post(hwiParams.intNum);
        SemaphoreP_pend(&gMyDoneSem, SystemP_WAIT_FOREVER);
    }
    cycles = CycleCounterP_getCount32() - cycles;

    HwiP_destruct(&hwiObj);
    SemaphoreP_destruct(&gMyDoneSem);

    DebugP_log("\r\n");
    DebugP_log(" With SemaphoreP,\r\n");
    DebugP_log("  number of interrupts = %d\r\n", maxCount);
    DebugP_log("  total cycles = %d\r\n", cycles);
    DebugP_log("  cycles per interrupt = %d\r\n", cycles/maxCount);
    DebugP_log("\r\n");
}

void test_hwi(void *args)
{
    HwiP_Params hwiParams;
    HwiP_Object hwiObj;
    int32_t status;
    uint32_t i, maxCount = 10;

    gMyISRCount = 0;
    gInISR = 0;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = TEST_INT_NUM;
    hwiParams.callback = myISR1;
    status = HwiP_construct(&hwiObj, &hwiParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    for (i = 0; i < maxCount; i++)
    {
        HwiP_post(hwiParams.intNum);
    }

    while (gMyISRCount < maxCount)
        ;

    TEST_ASSERT_EQUAL_INT32(maxCount, gMyISRCount);
    TEST_ASSERT_EQUAL_INT32(maxCount, gInISR);
    TEST_ASSERT_EQUAL_UINT32( 0, HwiP_inISR());

    HwiP_destruct(&hwiObj);
}

volatile uint32_t gMyNestedISRCount1;
volatile uint32_t gMyNestedISRCount2;
volatile uint32_t gNestedISRViolation;

#define MY_NESTED_ISR1_NUM  (TEST_INT_NUM)
#define MY_NESTED_ISR2_NUM  (TEST_INT_NUM+1)

static void myNestedISR1(void *args)
{
    gMyNestedISRCount1++;

    HwiP_post(MY_NESTED_ISR2_NUM);

    if(gMyNestedISRCount1 != gMyNestedISRCount2)
    {
        gNestedISRViolation++;
    }
}

static void myNestedISR2(void *args)
{
    gMyNestedISRCount2++;
}

void test_hwiNested(void *args)
{
    HwiP_Params hwiParams1;
    HwiP_Params hwiParams2;
    HwiP_Object hwiObj1;
    HwiP_Object hwiObj2;
    int32_t status;
    uint32_t i, maxCount = 10;

    gMyNestedISRCount1 = 0;
    gMyNestedISRCount2 = 0;
    gNestedISRViolation = 0;

    HwiP_Params_init(&hwiParams1);
    hwiParams1.intNum = MY_NESTED_ISR1_NUM;
    hwiParams1.callback = myNestedISR1;
    hwiParams1.priority = 4;
    status = HwiP_construct(&hwiObj1, &hwiParams1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_Params_init(&hwiParams2);
    hwiParams2.intNum = MY_NESTED_ISR2_NUM;
    hwiParams2.callback = myNestedISR2;
    hwiParams2.priority = 3;
    status = HwiP_construct(&hwiObj2, &hwiParams2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    for (i = 0; i < maxCount; i++)
    {
        HwiP_post(MY_NESTED_ISR1_NUM);
    }

    while (gMyNestedISRCount1 < maxCount)
        ;

    while (gMyNestedISRCount2 < maxCount)
        ;

    HwiP_destruct(&hwiObj1);
    HwiP_destruct(&hwiObj2);

    TEST_ASSERT_EQUAL_INT32(maxCount, gMyNestedISRCount1);
    TEST_ASSERT_EQUAL_INT32(maxCount, gMyNestedISRCount2);
    TEST_ASSERT_EQUAL_UINT32(0, gNestedISRViolation);
}

void test_semaphoreBinary(void *args)
{
    HwiP_Params hwiParams;
    HwiP_Object hwiObj;
    int32_t status;

    status = SemaphoreP_constructBinary(&gMyDoneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = TEST_INT_NUM;
    hwiParams.callback = myISR2;
    hwiParams.args = &gMyDoneSem;
    status = HwiP_construct(&hwiObj, &hwiParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_post(hwiParams.intNum);

    status = SemaphoreP_pend(&gMyDoneSem, 10);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_destruct(&hwiObj);
    SemaphoreP_destruct(&gMyDoneSem);
}

void test_semaphoreCounting(void *args)
{
    HwiP_Params hwiParams;
    HwiP_Object hwiObj;
    int32_t status;
    uint32_t i, maxCount = 10;

    status = SemaphoreP_constructCounting(&gMyDoneSem, 0, maxCount);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = TEST_INT_NUM;
    hwiParams.callback = myISR2;
    hwiParams.args = &gMyDoneSem;
    HwiP_construct(&hwiObj, &hwiParams);

    for (i = 0; i < maxCount; i++)
    {
        HwiP_post(hwiParams.intNum);
    }
    for (i = 0; i < maxCount; i++)
    {
        status = SemaphoreP_pend(&gMyDoneSem, 10);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    }

    HwiP_destruct(&hwiObj);
    SemaphoreP_destruct(&gMyDoneSem);
}

void test_semaphoreTimeout(void *args)
{
    int32_t status;
    uint32_t curTicks, timeout;

    status = SemaphoreP_constructBinary(&gMyDoneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    timeout = 10;

    curTicks = ClockP_getTicks();
    status = SemaphoreP_pend(&gMyDoneSem, timeout);
    curTicks = ClockP_getTicks() - curTicks;

    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);
    TEST_ASSERT_UINT32_WITHIN(5, timeout, curTicks);

    timeout = 0;

    curTicks = ClockP_getTicks();
    status = SemaphoreP_pend(&gMyDoneSem, timeout);
    curTicks = ClockP_getTicks() - curTicks;

    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);
    TEST_ASSERT_UINT32_WITHIN(5, timeout, curTicks);

    SemaphoreP_destruct(&gMyDoneSem);
}

void test_semaphoreMutex(void *args)
{
    int32_t status;
    uint32_t i, nesting = 10, iterations = 10;

    status = SemaphoreP_constructMutex(&gMyDoneSem);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    while (iterations--)
    {
        for (i = 0; i < nesting; i++)
        {
            status = SemaphoreP_pend(&gMyDoneSem, SystemP_WAIT_FOREVER);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        }
        for (i = 0; i < nesting; i++)
        {
            SemaphoreP_post(&gMyDoneSem);
        }
    }
    SemaphoreP_destruct(&gMyDoneSem);
}

static void myClockCallback(ClockP_Object *obj, void *args)
{
    SemaphoreP_Object *pSemObj = (SemaphoreP_Object *)args;

    SemaphoreP_post(pSemObj);
}

void test_clockMode(void *args)
{
    ClockP_Params clockParams;
    int32_t status;
    uint32_t curTicks, i, elaspedTicks, iteratioms = 10;
    uint32_t oneShotMode = (uint32_t)args;

    status = SemaphoreP_constructBinary(&gMyDoneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    ClockP_Params_init(&clockParams);
    clockParams.timeout = 10;
    if (oneShotMode)
    {
        clockParams.period = 0;
    }
    else
    {
        clockParams.period = clockParams.timeout;
    }
    clockParams.callback = myClockCallback;
    clockParams.args = &gMyDoneSem;
    status = ClockP_construct(&gMyClock, &clockParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    curTicks = ClockP_getTicks();


    ClockP_start(&gMyClock);

    if (oneShotMode)
    {
        iteratioms = 1;
    }
    else
    {
        iteratioms = 10;
    }

    for (i = 0; i < iteratioms; i++)
    {
        status = SemaphoreP_pend(&gMyDoneSem, clockParams.timeout * 2);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

        elaspedTicks = ClockP_getTicks() - curTicks;
        TEST_ASSERT_UINT32_WITHIN(5, clockParams.timeout * (i + 1), elaspedTicks);
    }

    if (!oneShotMode)
    {
        ClockP_stop(&gMyClock);
    }

    /* check if no more clock callbacks */
    status = SemaphoreP_pend(&gMyDoneSem, clockParams.timeout * 2);
    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);

    ClockP_destruct(&gMyClock);
    SemaphoreP_destruct(&gMyDoneSem);
}

static void test_clockWithHwiIsr(void *args)
{
    ClockP_start((ClockP_Object *)args);
}

void test_clock(void *args)
{
    uint32_t curTicks, sleepTimeInSecs;
    uint64_t sleepTimeInUsecs, curTime;
    uint32_t delayErrorInUsecs = ClockP_ticksToUsec(1); /* error in ClockP_sleep and ClockP_usleep can be upto 1 tick */

    /* test ticks to usec conversions */
    curTicks = ClockP_getTicks();
    curTime = ClockP_ticksToUsec(curTicks);
    TEST_ASSERT_EQUAL_UINT32(ClockP_usecToTicks(curTime), curTicks);

    /* test sleep in units of seconds */
    sleepTimeInSecs = 1;
    curTicks = ClockP_getTicks();
    ClockP_sleep(sleepTimeInSecs);
    curTicks = ClockP_getTicks() - curTicks;
    TEST_ASSERT_UINT32_WITHIN(delayErrorInUsecs, ClockP_usecToTicks(sleepTimeInSecs * 1000000), curTicks);

    /* test sleep in units of usecs and < 1 tick */
    sleepTimeInUsecs = 500;
    curTime = ClockP_getTimeUsec();
    ClockP_usleep(sleepTimeInUsecs);
    curTime = ClockP_getTimeUsec() - curTime;
    TEST_ASSERT_UINT32_WITHIN(delayErrorInUsecs, sleepTimeInUsecs, curTime);

    /* test sleep in units of usecs and > 1 tick */
    sleepTimeInUsecs = ClockP_ticksToUsec(1) + ClockP_ticksToUsec(1) / 3;
    curTime = ClockP_getTimeUsec();
    ClockP_usleep(sleepTimeInUsecs);
    curTime = ClockP_getTimeUsec() - curTime;
    TEST_ASSERT_UINT32_WITHIN(delayErrorInUsecs, sleepTimeInUsecs, curTime);

    /* test start of ClockP from ISR */
    {
        ClockP_Params clockParams;
        int32_t status;
        HwiP_Params hwiParams;
        HwiP_Object hwiObj;

        status = SemaphoreP_constructBinary(&gMyDoneSem, 0);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

        ClockP_Params_init(&clockParams);
        clockParams.timeout = 10;
        clockParams.period = clockParams.timeout;
        clockParams.callback = myClockCallback;
        clockParams.args = &gMyDoneSem;
        status = ClockP_construct(&gMyClock, &clockParams);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        status = ClockP_isActive(&gMyClock);
        TEST_ASSERT_EQUAL_INT32(status, SystemP_SUCCESS);

        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = TEST_INT_NUM;
        hwiParams.callback = test_clockWithHwiIsr;
        hwiParams.args = &gMyClock;
        status = HwiP_construct(&hwiObj, &hwiParams);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

        HwiP_post(hwiParams.intNum);

        status = SemaphoreP_pend(&gMyDoneSem, clockParams.timeout * 2);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

        ClockP_stop(&gMyClock);

        /* check if no more clock callbacks */
        status = SemaphoreP_pend(&gMyDoneSem, clockParams.timeout * 2);
        TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);

        HwiP_destruct(&hwiObj);
        ClockP_destruct(&gMyClock);
        SemaphoreP_destruct(&gMyDoneSem);
    }
}



#define NUM_ALLOCS (10u)
void test_heap(void *args)
{
    void *ptr[NUM_ALLOCS];
    uint32_t size[NUM_ALLOCS] = {255, 232, 255, 127, 63, 31, 15, 7, 3, 1};
    uint32_t freeSize;
    int32_t i;

    /* create heap */
    HeapP_construct(&gMyHeap, gMyHeapMem, MY_HEAP_MEM_SIZE);

    freeSize = HeapP_getFreeHeapSize(&gMyHeap);

    /* allocate memory from heap */
    for (i = 0; i < NUM_ALLOCS; i++)
    {
        ptr[i] = HeapP_alloc(&gMyHeap, size[i]);
        TEST_ASSERT_NOT_NULL(ptr[i]);
    }

    /* this one should fail */
    TEST_ASSERT_NULL(HeapP_alloc(&gMyHeap, freeSize / 2));

    /* use the memory */
    for (i = 0; i < NUM_ALLOCS; i++)
    {
        memset(ptr[i], i, size[i]);
    }

    /* free half the pointers */
    for (i = 0; i < NUM_ALLOCS / 2; i++)
    {
        HeapP_free(&gMyHeap, ptr[i]);
    }

    /* alloc again but change the order */
    for (i = (NUM_ALLOCS / 2) - 1; i >= 0; i--)
    {
        ptr[i] = HeapP_alloc(&gMyHeap, size[i]);
        TEST_ASSERT_NOT_NULL(ptr[i]);
    }

    /* free all the pointers */
    for (i = 0; i < NUM_ALLOCS; i++)
    {
        HeapP_free(&gMyHeap, ptr[i]);
    }

    TEST_ASSERT_EQUAL_UINT32(freeSize, HeapP_getFreeHeapSize(&gMyHeap));

    HeapP_destruct(&gMyHeap);
}

void test_cycleCounter(void *args)
{
    uint32_t cycleCount1, cycleCount10, cycleCount100;

    CycleCounterP_reset();

    cycleCount1 = CycleCounterP_getCount32();

    ClockP_usleep(1*1000);

    cycleCount1 = CycleCounterP_getCount32() - cycleCount1;

    cycleCount10 = CycleCounterP_getCount32();

    ClockP_usleep(10*1000);

    cycleCount10 = CycleCounterP_getCount32() - cycleCount10;

    cycleCount100 = CycleCounterP_getCount32();

    ClockP_usleep(100*1000);

    cycleCount100 = CycleCounterP_getCount32() - cycleCount100;

    /* cycle count value depends on CPU clock, since we dont know CPU clock in the program
     * we check if the cycle count value that is measured increases
     * propotionately to the increase in delay
     */

    /* make sure value is non-zero */
    TEST_ASSERT_GREATER_THAN_UINT32(0, cycleCount1);
    TEST_ASSERT_GREATER_THAN_UINT32(0, cycleCount10);
    TEST_ASSERT_GREATER_THAN_UINT32(0, cycleCount100);

    /* in FreeRTOS, we do WFI in idle task. In this case the cycle counter also stops counting
     * So this test will fail in FreeRTOS, but this is expected behaviour
     */
    #if defined (OS_NORTOS)
    TEST_ASSERT_UINT32_WITHIN( 1000000, cycleCount1*100, cycleCount10*10);
    TEST_ASSERT_UINT32_WITHIN( 1000000, cycleCount10*10, cycleCount100);
    #endif
}

void test_debugLog(void *args)
{
    uint32_t oldDebugLogZone, i;

    oldDebugLogZone = DebugP_logZoneEnable(DebugP_LOG_ZONE_INFO);

    DebugP_logInfo("We should see this.\r\n");

    DebugP_logZoneRestore(oldDebugLogZone);

    oldDebugLogZone = DebugP_logZoneDisable(DebugP_LOG_ZONE_INFO);

    DebugP_logInfo("We should not see this.\r\n");

    DebugP_logZoneRestore(oldDebugLogZone);

    /* By default via syscfg ERROR and WARNING zones are enabled and Info is disabled */
    DebugP_logError("We should see this.\r\n");
    DebugP_logWarn("We should see this.\r\n");
    DebugP_logInfo("We should not see this.\r\n");

    oldDebugLogZone = DebugP_logZoneDisable(DebugP_LOG_ZONE_ERROR | DebugP_LOG_ZONE_WARN);

    DebugP_logError("We should not see this.\r\n");
    DebugP_logWarn("We should not see this.\r\n");

    DebugP_logZoneRestore(oldDebugLogZone);

    /* This test print to the same line with \r instead of \r\n */
    for(i=0; i<3;i++)
    {
        DebugP_log("| %6d.%d \r", (uint32_t)(ClockP_getTimeUsec()/1000000), (uint32_t)(ClockP_getTimeUsec()%1000000) );
        ClockP_usleep(100*1000);
        DebugP_log("/ %6d.%d \r", (uint32_t)(ClockP_getTimeUsec()/1000000), (uint32_t)(ClockP_getTimeUsec()%1000000) );
        ClockP_usleep(100*1000);
        DebugP_log("- %6d.%d \r", (uint32_t)(ClockP_getTimeUsec()/1000000), (uint32_t)(ClockP_getTimeUsec()%1000000) );
        ClockP_usleep(100*1000);
        DebugP_log("\\ %6d.%d \r", (uint32_t)(ClockP_getTimeUsec()/1000000), (uint32_t)(ClockP_getTimeUsec()%1000000) );
        ClockP_usleep(100*1000);
    }

    /* enable this to manually check assert, once enabled, program wont go beyond this line */
    /* DebugP_assert(2==3); */
}

void myTaskMain(void *args)
{
    uint32_t iterations = 5;

    SemaphoreP_Object *pSem = (SemaphoreP_Object *)args;

    TaskP_yield();

    while(iterations)
    {
        DebugP_log("My task iteration #%d\r\n", iterations);
        TaskP_yield();
        iterations--;
    }

    SemaphoreP_post(pSem);

    TaskP_exit();
}

void test_MailboxTask1(void *args)
{
    struct test_mboxTaskTestParam* pTaskArgs = (struct test_mboxTaskTestParam*) args;
    uint8_t msgBuff[TEST_MBOX_MSG_SIZE];

    int32_t status = MailboxP_post(pTaskArgs->hMboxClientTx, "PINGT", SystemP_NO_WAIT);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    status = MailboxP_pend(pTaskArgs->hMboxClientRx, msgBuff, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* fill the \0 char to avoid mem overflows in error cases */
    TEST_ASSERT_EQUAL_STRING("PONGM", msgBuff);
    TaskP_exit();
}

static void testMboxEchoIsr(void *args)
{
    struct test_mboxIsrTestParam *pTestParms = (struct test_mboxIsrTestParam*) args;
    pTestParms->numMsgs1 = MailboxP_getNumPendingMsgs(pTestParms->hMboxClientRx);
    pTestParms->status1AtIsr = MailboxP_pend(pTestParms->hMboxClientRx, pTestParms->msgAtIsr, SystemP_NO_WAIT);
    pTestParms->status2AtIsr = MailboxP_post(pTestParms->hMboxClientTx, pTestParms->msgAtIsr, SystemP_NO_WAIT);
    pTestParms->numMsgs2 = MailboxP_getNumPendingMsgs(pTestParms->hMboxClientTx);
}

void test_mailbox(void *args)
{
    int32_t status;
    MailboxP_Params mboxParams;
    TaskP_Params    taskParams;
    MailboxP_Handle mboxClientTxHandle;
    MailboxP_Handle mboxClientRxHandle;
    uint8_t msgBuff[TEST_MBOX_MSG_SIZE];
    struct test_mboxTaskTestParam taskArgs;

    MailboxP_Params_init(&mboxParams);
    mboxParams.name = (uint8_t *)"testMbox";
    mboxParams.buf  = (void *)gMailBoxBuff;
    mboxParams.size =  TEST_MBOX_MSG_SIZE;
    mboxParams.count = TEST_MBOX_BUFF_COUNT;
    mboxParams.bufsize = TEST_MBOX_MSG_SIZE * TEST_MBOX_BUFF_COUNT;

    mboxClientTxHandle = MailboxP_create(&gMyMboxClientTx, &mboxParams);
    TEST_ASSERT_NOT_NULL(mboxClientTxHandle);

    mboxClientRxHandle = MailboxP_create(&gMyMboxClientRx, &mboxParams);
    TEST_ASSERT_NOT_NULL(mboxClientRxHandle);

    taskArgs.hMboxClientTx = mboxClientTxHandle;
    taskArgs.hMboxClientRx = mboxClientRxHandle;

    TaskP_Params_init(&taskParams);
    taskParams.name = "MAILBOX_TASK1";
    taskParams.stackSize = TEST_MBOX_TASK_STACK_SIZE;
    taskParams.stack = gTestMboxTask1Stack;
    taskParams.priority = TEST_MBOX_TASK1_PRIO;
    taskParams.args = &taskArgs;
    taskParams.taskMain = test_MailboxTask1;


    //############
    TEST_ASSERT_EQUAL_INT32(0x0, MailboxP_getNumPendingMsgs(mboxClientTxHandle));
    TEST_ASSERT_EQUAL_INT32(0x0, MailboxP_getNumPendingMsgs(mboxClientRxHandle));

    status = MailboxP_pend(mboxClientTxHandle, msgBuff, SystemP_NO_WAIT);
    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);

    status = TaskP_construct(&gTestMboxTaskObj, &taskParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status = MailboxP_pend(mboxClientTxHandle, msgBuff, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    TEST_ASSERT_EQUAL_STRING("PINGT", msgBuff);

    status = MailboxP_post(mboxClientRxHandle, "PONGM", SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    ClockP_sleep(1); // allow for test_MailboxTask1 to run
    TaskP_destruct(&gTestMboxTaskObj);

    //############
    TEST_ASSERT_EQUAL_INT32(0x0, MailboxP_getNumPendingMsgs(mboxClientTxHandle));
    TEST_ASSERT_EQUAL_INT32(0x0, MailboxP_getNumPendingMsgs(mboxClientRxHandle));
    status = TaskP_construct(&gTestMboxTaskObj, &taskParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    ClockP_sleep(1); // allow for test_MailboxTask1 to run

    status = MailboxP_pend(mboxClientTxHandle, msgBuff, SystemP_NO_WAIT);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    TEST_ASSERT_EQUAL_STRING("PINGT", msgBuff);

    status = MailboxP_post(mboxClientRxHandle, "PONGM", SystemP_NO_WAIT);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    TEST_ASSERT_EQUAL_INT32(0x1, MailboxP_getNumPendingMsgs(mboxClientRxHandle));
    ClockP_sleep(1); // allow for test_MailboxTask1 to run
    TEST_ASSERT_EQUAL_INT32(0x0, MailboxP_getNumPendingMsgs(mboxClientRxHandle));
    TEST_ASSERT_EQUAL_INT32(0x0, MailboxP_getNumPendingMsgs(mboxClientTxHandle));
    TaskP_destruct(&gTestMboxTaskObj);

    //############
    HwiP_Params hwiParams;
    HwiP_Object hwiObj;

    struct test_mboxIsrTestParam isrParams;
    isrParams.hMboxClientTx = mboxClientTxHandle;
    isrParams.hMboxClientRx = mboxClientRxHandle;
    strcpy((char*)isrParams.msgAtIsr, "TEST");
    isrParams.numMsgs1 = -1;
    isrParams.numMsgs2 = -1;
    isrParams.status1AtIsr = SystemP_FAILURE;
    isrParams.status2AtIsr = SystemP_FAILURE;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = TEST_INT_NUM;
    hwiParams.callback = testMboxEchoIsr;
    hwiParams.args = &isrParams;
    status = HwiP_construct(&hwiObj, &hwiParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    TEST_ASSERT_EQUAL_INT32(0x0, MailboxP_getNumPendingMsgs(mboxClientTxHandle));
    TEST_ASSERT_EQUAL_INT32(0x0, MailboxP_getNumPendingMsgs(mboxClientRxHandle));

    status = MailboxP_post(mboxClientRxHandle, "PONGM", SystemP_NO_WAIT);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_post(hwiParams.intNum);
    status = MailboxP_pend(mboxClientTxHandle, msgBuff, SystemP_NO_WAIT);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    TEST_ASSERT_EQUAL_STRING("PONGM", msgBuff);
    HwiP_destruct(&hwiObj);

    TEST_ASSERT_EQUAL_INT32(isrParams.numMsgs1, 1);
    TEST_ASSERT_EQUAL_INT32(isrParams.numMsgs2, 1);
    TEST_ASSERT_EQUAL_INT32(isrParams.status1AtIsr, SystemP_SUCCESS);
    TEST_ASSERT_EQUAL_INT32(isrParams.status2AtIsr, SystemP_SUCCESS);
    TEST_ASSERT_EQUAL_STRING("PONGM", isrParams.msgAtIsr);

    TEST_ASSERT_EQUAL_INT32(0x0, MailboxP_getNumPendingMsgs(mboxClientTxHandle));
    TEST_ASSERT_EQUAL_INT32(0x0, MailboxP_getNumPendingMsgs(mboxClientRxHandle));

    //############
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, MailboxP_delete(mboxClientTxHandle));
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, MailboxP_delete(mboxClientRxHandle));
}

void test_task(void *args)
{
    int32_t status;
    TaskP_Params myTaskParams;
    uint32_t iterations = 5;

    status = SemaphoreP_constructBinary(&gMyDoneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    TaskP_Params_init(&myTaskParams);
    myTaskParams.name = "MY_TASK";
    myTaskParams.stackSize = MY_TASK_STACK_SIZE;
    myTaskParams.stack = gMyTaskStack;
    myTaskParams.priority = MY_TASK_PRI;
    myTaskParams.args = &gMyDoneSem;
    myTaskParams.taskMain = myTaskMain;

    status = TaskP_construct(&gMyTask, &myTaskParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    while(iterations)
    {
        DebugP_log("Test task iteration #%d\r\n", iterations);
        TaskP_yield();
        iterations--;
    }

    status = SemaphoreP_pend(&gMyDoneSem, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* Dont delete task since the TaskP_exit() in the task deletes itself */
    /* TaskP_destruct(&gMyTask); */
    SemaphoreP_destruct(&gMyDoneSem);
}

static void eventIsr(void *args)
{
    gEventGetBitsStatusFromISR = EventP_getBits(&gMyEvent, &gEventGetBitsFromISR);
    gEventSetStatusFromISR = EventP_setBits(&gMyEvent, EVENT_BIT_FROM_ISR);
    gEventSet2StatusFromISR = EventP_setBits(&gMyEvent, EVENT_BIT2_FROM_ISR);
    gEventClearStatusFromISR = EventP_clearBits(&gMyEvent, EVENT_BIT2_FROM_ISR);
}

void eventTaskMain(void *args)
{
    uint32_t eventBits;
    HwiP_Params hwiParams;
    HwiP_Object hwiObj;
    int32_t status;

    status = EventP_waitBits(&gMyEvent, EVENT_BIT_FROM_PING, 0, 1, SystemP_WAIT_FOREVER, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING, eventBits);

    status = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING, eventBits);

    status = EventP_setBits(&gMyEvent, EVENT_BIT_FROM_PONG);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    status = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32((EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG), eventBits);

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = TEST_INT_NUM;
    hwiParams.callback = eventIsr;
    hwiParams.args = NULL;
    status = HwiP_construct(&hwiObj, &hwiParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_post(hwiParams.intNum);

    HwiP_destruct(&hwiObj);

    TaskP_exit();
}


void test_event(void *args)
{
    int32_t status;
    TaskP_Params taskParams;
    uint32_t eventBits;

    status = EventP_construct(&gMyEvent);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    TaskP_Params_init(&taskParams);
    taskParams.name = "EVENT_TASK";
    taskParams.stackSize = EVENT_TASK_STACK_SIZE;
    taskParams.stack = gTaskStack;
    taskParams.priority = EVENT_TASK_PRI;
    taskParams.args = NULL;
    taskParams.taskMain = eventTaskMain;

    status = TaskP_construct(&gEventTask, &taskParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    gEventSetStatusFromISR = SystemP_FAILURE;
    gEventSet2StatusFromISR = SystemP_FAILURE;
    gEventClearStatusFromISR = SystemP_FAILURE;
    gEventGetBitsStatusFromISR = SystemP_FAILURE;
    gEventGetBitsFromISR = 0x0;

    status = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    TEST_ASSERT_EQUAL_INT32(0x0, eventBits);

    status = EventP_setBits(&gMyEvent, EVENT_BIT_FROM_PING);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    status = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING, eventBits);

    status = EventP_waitBits(&gMyEvent, (EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG | EVENT_BIT_FROM_ISR), 0, 1, SystemP_WAIT_FOREVER, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    TEST_ASSERT_EQUAL_INT32((EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG | EVENT_BIT_FROM_ISR), eventBits);

    status = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    TEST_ASSERT_EQUAL_INT32((EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG | EVENT_BIT_FROM_ISR), eventBits);

    status = EventP_clearBits(&gMyEvent, (EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG | EVENT_BIT_FROM_ISR));
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    status = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    TEST_ASSERT_EQUAL_INT32(0x0, eventBits);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, gEventSetStatusFromISR);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, gEventSet2StatusFromISR);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, gEventClearStatusFromISR);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, gEventGetBitsStatusFromISR);
    TEST_ASSERT_EQUAL_INT32((EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG), gEventGetBitsFromISR);

    /* Dont delete task since the TaskP_exit() in the task deletes itself */
    /* TaskP_destruct(&gMyTask); */
    EventP_destruct(&gMyEvent);
}

/* Test to check different clock configurations */
void test_clockConfig(void *args)
{
    uint32_t start, end, overhead;
    /* Calculate overhead */
    CycleCounterP_reset();
    start = CycleCounterP_getCount32();
    end = CycleCounterP_getCount32();
    DebugP_log("Start: %d\r\n", start);
    DebugP_log("End: %d\r\n", end);
    overhead = end - start;
    DebugP_log("Total Overhead: %d Cycles\r\n", overhead);
    CycleCounterP_reset();

    start = CycleCounterP_getCount32();
    ClockP_usleep(1);
    end = CycleCounterP_getCount32();
    overhead = end - start;
    DebugP_log("Start: %d\r\n", start);
    DebugP_log("End: %d\r\n", end);
    DebugP_log("Total Overhead: %d Cycles\r\n", overhead);
    DebugP_log("Total: %d Cycles = %d microseconds @ 400MHz\r\n", overhead, overhead / 400);
}

void test_debugScanf(void *args)
{
    int32_t status;

    uint32_t value32[2];
    char valueStr[80];
    char valueChar;
    float valueFloat;

    DebugP_log("Enter a 32b number\r\n");
    value32[0] = 0;
    status = DebugP_scanf("%d", &value32[0]);
    DebugP_log("32b value = %d\r\n", value32[0]);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    DebugP_log("Enter two 32b numbers\r\n");
    value32[0] = value32[1] = 0;
    status = DebugP_scanf("%d%d", &value32[0], &value32[1]);
    DebugP_log("32b values = %d %d\r\n", value32[0], value32[1]);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    DebugP_log("Enter a float\r\n");
    valueFloat = 0;
    status = DebugP_scanf("%f", &valueFloat);
    DebugP_log("Float value = %f\r\n", valueFloat);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    DebugP_log("Enter a char\r\n");
    valueChar = ' ';
    status = DebugP_scanf("%c", &valueChar);
    DebugP_log("Char value = %c\r\n", valueChar);

    DebugP_log("Enter a string\r\n");
    valueStr[0] = 0;
    status = DebugP_scanf("%s", valueStr);
    DebugP_log("String value = %s\r\n", valueStr);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

/*
 * This test does a RD, WR access time test with data in TCMA and MSRAM
 * The TCMA and MSRAM address are hardcoded and set according to AM64x SOC
 *
 * Due to this this test is kept disabled by default and can be enabled
 * and adjusted manually when running on AM64x or other SOCs
 */
void test_memRdWr(void *args)
{
    uint8_t *tcmAddr = (uint8_t*)0x1000;
    uint8_t *msmcAddr = (uint8_t*)0x70000000;
    uint32_t *curAddr;
    uint32_t i, j, len = ((uint32_t)16*1024)/sizeof(uint32_t), loop = 1;
    uint32_t oldIntState;
    uint32_t tcmWrCycles;
    uint32_t msmcWrCycles;
    uint32_t tcmRdCycles;
    uint32_t msmcRdCycles;
    uint32_t overheadCycles;
    uint32_t tcmVal = 0;
    uint32_t msmcVal = 0;

    /* disable interrupts so that tick timer cannot come in between */
    oldIntState = HwiP_disable();

    CycleCounterP_reset();

    overheadCycles = CycleCounterP_getCount32();
    overheadCycles = CycleCounterP_getCount32() - overheadCycles;

    tcmWrCycles = CycleCounterP_getCount32();
    for(j=0; j<loop; j++)
    {
        curAddr = (uint32_t*)tcmAddr;
        for(i=0; i<len; i++)
        {
            *curAddr++ = 1;
        }
    }
    tcmWrCycles = CycleCounterP_getCount32() - tcmWrCycles;

    msmcWrCycles = CycleCounterP_getCount32();
    for(j=0; j<loop; j++)
    {
        curAddr = (uint32_t*)msmcAddr;
        for(i=0; i<len; i++)
        {
            *curAddr++ = 1;
        }
    }
    msmcWrCycles = CycleCounterP_getCount32() - msmcWrCycles;

    tcmRdCycles = CycleCounterP_getCount32();
    for(j=0; j<loop; j++)
    {
        curAddr = (uint32_t*)tcmAddr;
        for(i=0; i<len; i++)
        {
            tcmVal += *curAddr++;
        }
    }
    tcmRdCycles = CycleCounterP_getCount32() - tcmRdCycles;

    msmcRdCycles = CycleCounterP_getCount32();
    for(j=0; j<loop; j++)
    {
        curAddr = (uint32_t*)msmcAddr;
        for(i=0; i<len; i++)
        {
            msmcVal += *curAddr++;
        }
    }
    msmcRdCycles = CycleCounterP_getCount32() - msmcRdCycles;

    HwiP_restore(oldIntState);

    DebugP_log(" Loop count = %d, Size = %d B, Overhead cycles = %d\r\n", loop, len*sizeof(uint32_t), overheadCycles);
    DebugP_log(" TCM  WR cycles = %d\r\n", tcmWrCycles/loop);
    DebugP_log(" TCM  RD cycles = %d (value = %d)\r\n", tcmRdCycles/loop, tcmVal/loop);
    DebugP_log(" MSMC WR cycles = %d\r\n", msmcWrCycles/loop);
    DebugP_log(" MSMC RD cycles = %d (value = %d)\r\n", msmcRdCycles/loop, msmcVal/loop);
}

float floatLoadAndMultiply(float f1, float f2);
float floatMultiply();

double gFloat = 10.0;

static void test_floatOperationsIsr(void *arg)
{
    gFloat = gFloat + floatLoadAndMultiply(0.1, 1.0);

    gMyISRCount = 1;
}

/* switch between ping and pong tasks and do float operations in between */
void test_mainToIsrWithFloatOperations(void *args)
{
    uint32_t count; /* loop `count` times */
    uint32_t numInterrupts = 100000;
    double f;
    HwiP_Params hwiParams;
    HwiP_Object hwiObj;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = TEST_INT_NUM;
    hwiParams.callback = test_floatOperationsIsr;
    hwiParams.priority = 8;
    HwiP_construct(&hwiObj, &hwiParams);

    gFloat = 10.0;
    f = 0.0;
    floatLoadAndMultiply(0.1, 0.1);
    count = numInterrupts;
    while (count--)
    {
        gMyISRCount = 0;
        f = f + floatMultiply();
        HwiP_post(TEST_INT_NUM);
        while(gMyISRCount == 0)
            ;
    }

    HwiP_destruct(&hwiObj);

    TEST_ASSERT_UINT32_WITHIN(1, (numInterrupts / 100), (uint32_t)f);
    TEST_ASSERT_UINT32_WITHIN(1, 10 + (numInterrupts / 10), (uint32_t)gFloat);
}

static void test_floatOperationsFiqIsr(void *arg)
{
    gFloat = gFloat + floatLoadAndMultiply(0.1, 1.0);

    gMyISRCount = 1;
}

/* switch between main and fiq isr and do float operations in both */
void test_mainToFiqWithFloatOperations(void *args)
{
    uint32_t count; /* loop `count` times */
    uint32_t numInterrupts = 100000;
    double f;
    HwiP_Params hwiParams;
    HwiP_Object hwiObj;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = TEST_INT_NUM;
    hwiParams.callback = test_floatOperationsFiqIsr;
    hwiParams.priority = 8;
    hwiParams.isFIQ = 1;
    HwiP_construct(&hwiObj, &hwiParams);

    gFloat = 10.0;
    f = 0.0;
    floatLoadAndMultiply(0.1, 0.1);
    count = numInterrupts;
    while (count--)
    {
        gMyISRCount = 0;
        f = f + floatMultiply();
        HwiP_post(TEST_INT_NUM);
        while(gMyISRCount == 0)
            ;
    }

    HwiP_destruct(&hwiObj);

    TEST_ASSERT_UINT32_WITHIN(1, (numInterrupts / 100), (uint32_t)f);
    TEST_ASSERT_UINT32_WITHIN(1, 10 + (numInterrupts / 10), (uint32_t)gFloat);
}

void test_addrconversion(void *args)
{
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
    uint64_t    phyAddr;
    void       *virtAddr;
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')
    /*
     * Virtual to Physical test
     */
    phyAddr = SOC_virtToPhy((void *) CSL_MSS_TCMA_RAM_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_TCMA_CR5A_U_BASE, (uint32_t) phyAddr);
    phyAddr = SOC_virtToPhy((void *) (CSL_MSS_TCMA_RAM_BASE  + CSL_MSS_TCMA_RAM_SIZE - 1));
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_TCMA_CR5A_U_BASE + CSL_MSS_TCMA_RAM_SIZE - 1, (uint32_t) phyAddr);

    phyAddr = SOC_virtToPhy((void *) CSL_MSS_TCMB_RAM_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_TCMB_CR5A_U_BASE, (uint32_t) phyAddr);
    phyAddr = SOC_virtToPhy((void *) (CSL_MSS_TCMB_RAM_BASE + CSL_MSS_TCMB_RAM_SIZE - 1));
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_TCMB_CR5A_U_BASE + CSL_MSS_TCMB_RAM_SIZE - 1, (uint32_t) phyAddr);

    phyAddr = SOC_virtToPhy((void *) CSL_MSS_L2_RAM_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_GLOB_MSS_L2_RAM_BASE, (uint32_t) phyAddr);
    phyAddr = SOC_virtToPhy((void *) (CSL_MSS_L2_RAM_BASE + CSL_MSS_L2_RAM_SIZE - 1));
    TEST_ASSERT_EQUAL_UINT32(CSL_GLOB_MSS_L2_RAM_BASE + CSL_MSS_L2_RAM_SIZE - 1, (uint32_t) phyAddr);

    phyAddr = SOC_virtToPhy((void *) CSL_MSS_L3_RAM_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_L3_RAM_BASE, (uint32_t) phyAddr);
    phyAddr = SOC_virtToPhy((void *) (CSL_MSS_L3_RAM_BASE + CSL_MSS_L3_RAM_SIZE - 1));
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_L3_RAM_BASE + CSL_MSS_L3_RAM_SIZE - 1, (uint32_t) phyAddr);

    /*
     * Physical to Virtual test
     */
    virtAddr = SOC_phyToVirt(CSL_MSS_TCMA_CR5A_U_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_TCMA_RAM_BASE, (uint32_t) virtAddr);
    virtAddr = SOC_phyToVirt(CSL_MSS_TCMA_CR5A_U_BASE + CSL_MSS_TCMA_RAM_SIZE - 1);
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_TCMA_RAM_BASE + CSL_MSS_TCMA_RAM_SIZE - 1, (uint32_t) virtAddr);

    virtAddr = SOC_phyToVirt(CSL_MSS_TCMB_CR5A_U_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_TCMB_RAM_BASE, (uint32_t) virtAddr);
    virtAddr = SOC_phyToVirt(CSL_MSS_TCMB_CR5A_U_BASE + CSL_MSS_TCMB_RAM_SIZE - 1);
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_TCMB_RAM_BASE + CSL_MSS_TCMB_RAM_SIZE - 1, (uint32_t) virtAddr);

    virtAddr = SOC_phyToVirt(CSL_GLOB_MSS_L2_RAM_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_L2_RAM_BASE, (uint32_t) virtAddr);
    virtAddr = SOC_phyToVirt(CSL_GLOB_MSS_L2_RAM_BASE + CSL_MSS_L2_RAM_SIZE - 1);
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_L2_RAM_BASE + CSL_MSS_L2_RAM_SIZE - 1, (uint32_t) virtAddr);

    virtAddr = SOC_phyToVirt(CSL_MSS_L3_RAM_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_L3_RAM_BASE, (uint32_t) virtAddr);
    virtAddr = SOC_phyToVirt(CSL_MSS_L3_RAM_BASE + CSL_MSS_L3_RAM_SIZE - 1);
    TEST_ASSERT_EQUAL_UINT32(CSL_MSS_L3_RAM_BASE + CSL_MSS_L3_RAM_SIZE - 1, (uint32_t) virtAddr);
#endif  /* (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R') */

#if defined(_TMS320C6X)
    /*
     * Virtual to Physical test
     */
    phyAddr = SOC_virtToPhy((void *) CSL_DSP_L1P_U_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_GLOB_DSP_L1P_U_BASE, (uint32_t) phyAddr);
    phyAddr = SOC_virtToPhy((void *) (CSL_DSP_L1P_U_BASE  + CSL_DSP_L1P_SIZE - 1));
    TEST_ASSERT_EQUAL_UINT32(CSL_GLOB_DSP_L1P_U_BASE + CSL_DSP_L1P_SIZE - 1, (uint32_t) phyAddr);

    phyAddr = SOC_virtToPhy((void *) CSL_DSP_L1D_U_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_GLOB_DSP_L1D_U_BASE, (uint32_t) phyAddr);
    phyAddr = SOC_virtToPhy((void *) (CSL_DSP_L1D_U_BASE + CSL_DSP_L1D_SIZE - 1));
    TEST_ASSERT_EQUAL_UINT32(CSL_GLOB_DSP_L1D_U_BASE + CSL_DSP_L1D_SIZE - 1, (uint32_t) phyAddr);

    phyAddr = SOC_virtToPhy((void *) CSL_DSP_L2_U_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_GLOB_DSP_L2_U_BASE, (uint32_t) phyAddr);
    phyAddr = SOC_virtToPhy((void *) (CSL_DSP_L2_U_BASE + CSL_DSP_L2_RAM_SIZE - 1));
    TEST_ASSERT_EQUAL_UINT32(CSL_GLOB_DSP_L2_U_BASE + CSL_DSP_L2_RAM_SIZE - 1, (uint32_t) phyAddr);

    phyAddr = SOC_virtToPhy((void *) CSL_DSP_L3_U_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_DSP_L3_U_BASE, (uint32_t) phyAddr);
    phyAddr = SOC_virtToPhy((void *) (CSL_DSP_L3_U_BASE + CSL_DSP_L3_RAM_SIZE - 1));
    TEST_ASSERT_EQUAL_UINT32(CSL_DSP_L3_U_BASE + CSL_DSP_L3_RAM_SIZE - 1, (uint32_t) phyAddr);

    /*
     * Physical to Virtual test
     */
    virtAddr = SOC_phyToVirt(CSL_GLOB_DSP_L1P_U_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_DSP_L1P_U_BASE, (uint32_t) virtAddr);
    virtAddr = SOC_phyToVirt(CSL_GLOB_DSP_L1P_U_BASE  + CSL_DSP_L1P_SIZE - 1);
    TEST_ASSERT_EQUAL_UINT32(CSL_DSP_L1P_U_BASE + CSL_DSP_L1P_SIZE - 1, (uint32_t) virtAddr);

    virtAddr = SOC_phyToVirt(CSL_GLOB_DSP_L1D_U_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_DSP_L1D_U_BASE, (uint32_t) virtAddr);
    virtAddr = SOC_phyToVirt(CSL_GLOB_DSP_L1D_U_BASE + CSL_DSP_L1D_SIZE - 1);
    TEST_ASSERT_EQUAL_UINT32(CSL_DSP_L1D_U_BASE + CSL_DSP_L1D_SIZE - 1, (uint32_t) virtAddr);

    virtAddr = SOC_phyToVirt(CSL_GLOB_DSP_L2_U_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_DSP_L2_U_BASE, (uint32_t) virtAddr);
    virtAddr = SOC_phyToVirt(CSL_GLOB_DSP_L2_U_BASE + CSL_DSP_L2_RAM_SIZE - 1);
    TEST_ASSERT_EQUAL_UINT32(CSL_DSP_L2_U_BASE + CSL_DSP_L2_RAM_SIZE - 1, (uint32_t) virtAddr);

    virtAddr = SOC_phyToVirt(CSL_DSP_L3_U_BASE);
    TEST_ASSERT_EQUAL_UINT32(CSL_DSP_L3_U_BASE, (uint32_t) virtAddr);
    virtAddr = SOC_phyToVirt(CSL_DSP_L3_U_BASE + CSL_DSP_L3_RAM_SIZE - 1);
    TEST_ASSERT_EQUAL_UINT32(CSL_DSP_L3_U_BASE + CSL_DSP_L3_RAM_SIZE - 1, (uint32_t) virtAddr);
#endif  /* _TMS320C6X */
#endif  /* SOC_AM273X || SOC_AWR294X */

    return;
}

typedef struct Test_Queue_Buf_s
{
    QueueP_Elem lnk;
    uint32_t    index;
} Test_Queue_Buf;

void test_queue(void *args)
{
    QueueP_Handle   handle;
    QueueP_Object   qObj;
    Test_Queue_Buf  buf[10], *pBuf;
    int32_t         status;
    int             i;

    handle = QueueP_create(&qObj);
    TEST_ASSERT_NOT_NULL(handle);

    for (i = 0; i < 10; i++) {
        buf[i].index = (uint32_t)i;
    }
    /* Test 1: queue push/pop test */
    for (i = 0; i < 10; i++) {
        status = QueueP_put(handle, (QueueP_Elem *)&buf[i]);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    }
    for (i = 0; i < 10; i++) {
        pBuf = (Test_Queue_Buf *)QueueP_get(handle);
        TEST_ASSERT_NOT_NULL(pBuf);
        TEST_ASSERT_EQUAL_UINT32(pBuf->index, i);
    }

    /* Test 2: queue empty test */
    /* After poping all the pushed elements, queue should be empty */
    TEST_ASSERT_EQUAL_UINT32(QueueP_EMPTY, QueueP_isEmpty(handle));

    /* When called with an empty queue, QueueP_get should return a pointer to the queue itself */
    pBuf = QueueP_get(handle);

    status = QueueP_delete(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

void setUp(void)
{
}

void tearDown(void)
{
}

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();

    UNITY_BEGIN();

    RUN_TEST(test_hwi, 282, NULL);
    RUN_TEST(test_semaphoreMutex, 283, NULL);
    RUN_TEST(test_semaphoreBinary, 284, NULL);
    RUN_TEST(test_semaphoreCounting, 285, NULL);
    RUN_TEST(test_semaphoreTimeout, 286, NULL);
    RUN_TEST(test_clockMode, 287, (void *)0);
    RUN_TEST(test_clockMode, 288, (void *)1);
    RUN_TEST(test_clock, 289, NULL);
    RUN_TEST(test_heap, 290, NULL);
    RUN_TEST(test_cycleCounter, 291, NULL);
    RUN_TEST(test_debugLog, 292, NULL);
    RUN_TEST(test_hwiProfile, 293, NULL);
    RUN_TEST(test_queue, 3808, NULL);
    RUN_TEST(test_clockConfig, 10782, NULL);

    /* tasks are not supported in nortos */
    #if defined (OS_FREERTOS) || defined (OS_SAFERTOS)
    RUN_TEST(test_task, 294, NULL);
    RUN_TEST(test_event, 805, NULL);
    #endif

    #if defined (OS_FREERTOS)
    RUN_TEST(test_mailbox, 13390, NULL);
    #endif

    #if defined(__ARM_ARCH_7R__) && defined(OS_FREERTOS)
    /* nested ISR not supported for now */
    #elif defined(_TMS320C6X)
    /* nested ISR not supported in C66x */
    #else
    RUN_TEST(test_hwiNested, 295, NULL);
    #endif

    #if defined(__ARM_ARCH_7R__)
    /* floating point operations in ISR supported in R5F only */
    RUN_TEST(test_mainToIsrWithFloatOperations, 1571, NULL);

    #ifdef EN_SAVE_RESTORE_FPU_CONTEXT
    /** floating point operations in FIQ supported in R5F only
     *  Make sure the macro EN_SAVE_RESTORE_FPU_CONTEXT is uncommented in source/kernel/dpl/HwiP.h, otherwise the test will fail.
    */
    RUN_TEST(test_mainToFiqWithFloatOperations, 12213, NULL);
    #endif
    #endif

    /* disabled by default since otherwise the application wait for user input */
    #if 0
    RUN_TEST(test_debugScanf, 14, NULL);
    #endif

    /* Perform c66x specific tests */
#if defined(_TMS320C6X)
    test_c66x();
#endif

    RUN_TEST(test_addrconversion, 898, NULL);

    UNITY_END();

    Drivers_close();
}

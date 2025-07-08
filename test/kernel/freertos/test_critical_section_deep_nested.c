/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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

#include "test_critical_section.h"

#if (configUSE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS==1)

#define ISR_START_IDX     (100U)
#define ISR_COUNT         (HwiP_MAX_PRIORITY)
#define ISR_MAX_IDX       (ISR_START_IDX + ISR_COUNT - 1U)

#define CRITICAL_ISR_HIGHEST_PRI_INT_NUM  (ISR_START_IDX)
#define CRITICAL_ISR_LOWEST_PRI_INT_NUM   (ISR_START_IDX + configMAX_SYSCALL_INTERRUPT_PRIORITY - 1U)
#define CRITICAL_ISR_START_IDX            (CRITICAL_ISR_HIGHEST_PRI_INT_NUM)
#define CRITICAL_ISR_MAX_IDX              (CRITICAL_ISR_LOWEST_PRI_INT_NUM)
#define CRITICAL_ISR_COUNT                (CRITICAL_ISR_MAX_IDX - CRITICAL_ISR_START_IDX + 1U)

#define NORMAL_ISR_HIGHEST_PRI_INT_NUM    (CRITICAL_ISR_LOWEST_PRI_INT_NUM + 1U)
#define NORMAL_ISR_LOWEST_PRI_INT_NUM     (ISR_MAX_IDX)
#define NORMAL_ISR_START_IDX              (NORMAL_ISR_HIGHEST_PRI_INT_NUM)
#define NORMAL_ISR_MAX_IDX                (NORMAL_ISR_LOWEST_PRI_INT_NUM)
#define NORMAL_ISR_COUNT                  (NORMAL_ISR_MAX_IDX - NORMAL_ISR_START_IDX + 1U)

static volatile uint32_t gIsrEntryTracker[ISR_COUNT] = { 0U };
static volatile uint32_t gIsrExitTracker[ISR_COUNT]  = { 0U };

static uint32_t gIsrEntrySeqExp[ISR_COUNT] = { 0U };
static uint32_t gIsrExitSeqExp[ISR_COUNT]  = { 0U };

static volatile uint32_t gCriticalIsrTracker[CRITICAL_ISR_COUNT] = { 0U };
static volatile uint32_t gNormalIsrTracker[NORMAL_ISR_COUNT]     = { 0U };

static volatile uint32_t gIsrEntryCnt       = 0U;
static volatile uint32_t gIsrExitCnt        = 0U;
static volatile uint32_t gNormalIsrErrorCnt = 0U;
                                                                                                                                   
static HwiP_Object gHwiObj[ISR_COUNT];

/* Critical ISR - This will run even when in a critical section.
 *
 * Each critical ISR will trigger the next higher priority critical interrupt as well as normal ISR.
 * It will immediately switch to the high priority critical ISR.
 * And all normal ISR will be executed only after exit from the task critical section based on pending interrupts priority.
 */ 
static void critical_isr(void *arg)
{
    uint32_t intNum = (uint32_t)arg;
    uint32_t criticalIntNumOffset, nextNormalIntNum, nextCriticalIntNum;

    /* Store the interrupt number in entry sequence tracker */
    gIsrEntryTracker[gIsrEntryCnt] = intNum;
    gIsrEntryCnt++;

    /* Mark the ISR as executed */
    gCriticalIsrTracker[intNum - CRITICAL_ISR_START_IDX]++;

    /* Trigger the next higher priority normal interrupt.
     * This shouldn't fire until exit from critical section in task as well as all other high priority pending interrupts are serviced. */
    criticalIntNumOffset = CRITICAL_ISR_MAX_IDX - intNum;
    nextNormalIntNum     = NORMAL_ISR_LOWEST_PRI_INT_NUM - criticalIntNumOffset - 1U;
    if(nextNormalIntNum >= NORMAL_ISR_START_IDX)  /* Check if valid normal interrupt */
    {
        HwiP_post(nextNormalIntNum);
    }

    /* Trigger the next higher priority critical interrupt.
     * This should fire immediately. */
    nextCriticalIntNum = intNum - 1U;
    if(nextCriticalIntNum >= CRITICAL_ISR_START_IDX)  /* Check if valid critical interrupt */
    {
        HwiP_post(nextCriticalIntNum);
    }

    /* Store the interrupt number in exit sequence tracker. 
     * Highest priority critical interrupt should be the first to exit, 
     * and Lowest priority critical interrupt should be the last to exit here. */
    gIsrExitTracker[gIsrExitCnt] = intNum;
    gIsrExitCnt++;
}

/* Normal ISR - This won't run when in a critical section.
 *
 * Each normal ISR will trigger the next higher priority normal interrupt(if not already executed) from an ISR critical section.
 * It will immediately switch to the high priority normal ISR after exit from ISR critical section.
 */ 
static void normal_isr(void *arg)
{
    uint32_t intNum = (uint32_t)arg;
    uint32_t key;
    uint32_t nextNormalIntNum;

    /* Store the interrupt number in entry sequence tracker */
    gIsrEntryTracker[gIsrEntryCnt] = intNum;
    gIsrEntryCnt++;

    /* Mark the ISR as executed */
    gNormalIsrTracker[intNum - NORMAL_ISR_START_IDX]++;

    key = taskENTER_CRITICAL_FROM_ISR(); /* Enter Critical Section */

    /* Trigger the next higher priority normal interrupt(if not already executed).
     * This should fire soon after the critical section exit from ISR below. */
    nextNormalIntNum = intNum - 1U;
    if(nextNormalIntNum >= NORMAL_ISR_START_IDX) /* Check if valid normal interrupt */
    {
        if(gNormalIsrTracker[nextNormalIntNum - NORMAL_ISR_START_IDX] == 0U) /* Check if already executed or not */
        {
            HwiP_post(nextNormalIntNum);

            /* Make sure the higher priority normal interrupt is not yet executed since in critical section */
            if(gNormalIsrTracker[nextNormalIntNum - NORMAL_ISR_START_IDX] == 1U)
            {
                gNormalIsrErrorCnt++;
            }
        }
    }

    taskEXIT_CRITICAL_FROM_ISR(key); /* Exit Critical Section */

    /* Store the interrupt number in exit sequence tracker. 
     * Highest priority normal interrupt should be the first to exit here, 
     * and Lowest priority normal interrupt should be the last to exit. */
    gIsrExitTracker[gIsrExitCnt] = intNum;
    gIsrExitCnt++;
}

static void test_reset_variables(void)
{
    gIsrEntryCnt       = 0U;
    gIsrExitCnt        = 0U;
    gNormalIsrErrorCnt = 0U;

    memset((void*)gIsrEntryTracker,    0U, sizeof(gIsrEntryTracker));
    memset((void*)gIsrExitTracker,     0U, sizeof(gIsrExitTracker));
    memset((void*)gCriticalIsrTracker, 0U, sizeof(gCriticalIsrTracker));
    memset((void*)gNormalIsrTracker,   0U, sizeof(gNormalIsrTracker));
}

static void test_register_interrupts(bool isPulse)
{
    uint32_t priority = 0U; /* Start with highest priority */

    for(uint32_t intNum = ISR_START_IDX; intNum <= ISR_MAX_IDX; intNum++)
    {
        HwiP_Params hwiParams;

        HwiP_Params_init(&hwiParams);
        hwiParams.intNum   = intNum;
        hwiParams.callback = (intNum <= CRITICAL_ISR_MAX_IDX) ? critical_isr : normal_isr;
        hwiParams.priority = priority;
        hwiParams.isPulse  = isPulse;
        hwiParams.args     = (void*)intNum;
        HwiP_construct(&gHwiObj[priority], &hwiParams);

        priority++;
    }
}

static uint32_t* test_get_expected_isr_entry_sequence()
{
    uint32_t idx = 0U;

    /* Execution starts with lowest priority critical ISR */
    uint32_t intNum = CRITICAL_ISR_LOWEST_PRI_INT_NUM;
    /* And goes to each "higher" priority critical ISR one by one (triggered by each critical ISR) */
    while(intNum >= CRITICAL_ISR_HIGHEST_PRI_INT_NUM)
    {
        gIsrEntrySeqExp[idx++] = intNum;
        intNum--; /* "Decrement" since the interrupt numbers are such that higher the priority lower the interrupt number */
    }
    /* Then, it should enter the highest priority pending normal ISR triggered by the critical interrupt */
    intNum = NORMAL_ISR_LOWEST_PRI_INT_NUM - CRITICAL_ISR_COUNT;
    /* And then to each "higher" priority normal ISR one by one (triggered by each normal ISR) */
    while(intNum >= NORMAL_ISR_HIGHEST_PRI_INT_NUM)
    {
        gIsrEntrySeqExp[idx++] = intNum;
        intNum--; /* "Decrement" since the interrupt numbers are such that higher the priority lower the interrupt number */
    }
    /* Then, it should enter the 2nd highest priority pending normal ISR triggered by the critical interrupt */
    intNum = NORMAL_ISR_LOWEST_PRI_INT_NUM - CRITICAL_ISR_COUNT + 1U;
    /* And then to each "lower" priority pending normal ISR one by one (which was triggered by critical interrupt) */
    while(intNum <= NORMAL_ISR_LOWEST_PRI_INT_NUM)
    {
        gIsrEntrySeqExp[idx++] = intNum;
        intNum++; /* "Increment" since the interrupt numbers are such that higher the priority lower the interrupt number */
    }

    return gIsrEntrySeqExp;
}

static uint32_t* test_get_expected_isr_exit_sequence()
{
    uint32_t idx = 0U;

    /* ISRs exits in order from the highest priority interrupt to lowest priority interrupt. */
    /* Increment interrupt number since the interrupt numbers are such that higher the priority lower the interrupt number */
    for(uint32_t intNum = CRITICAL_ISR_HIGHEST_PRI_INT_NUM; intNum <= NORMAL_ISR_LOWEST_PRI_INT_NUM; intNum++)
    {
        gIsrExitSeqExp[idx++] = intNum;
    }

    return gIsrExitSeqExp;
}

static void test_unregister_interrupts(void)
{
    for(uint32_t idx = 0U; idx < ISR_COUNT; idx++)
    {
        HwiP_destruct(&gHwiObj[idx]);
    }
}

/** Test for critical sections which sweeps through all priority levels of interrupts
 * 
 * ---------------------------------------------------------------------------------------------------------------
 * | [Task] --> Enter Critical Section --> Trigger Lowest Pri Normal Int --> Trigger Lowest Pri Critical Int --> |
 * |                                       (max normal idx)                  (max critical idx)                  |
 * |                                                                                                             |
 * | [Lowest Pri Critical ISR] --> Trigger Higher Pri Normal Int --> Trigger Higher Pri Critical Int -->         |
 * | (max critical idx)            (max normal idx - 1)              (max critical idx - 1)                      |
 * |                                                                                                             |
 * | [Higher Pri Critical ISR] --> Trigger Higher Pri Normal Int --> Trigger Higher Pri Critical Int -->         |
 * | (max critical idx - 1)        (max normal idx - 2)              (max critical idx - 2)                      |
 * |                                                                                                             |
 * | ...                                                                                                         |
 * |                                                                                                             |
 * | [Highest Pri Critical ISR] --> Trigger Higher Pri Normal Int        -->                                     |
 * | (min critical idx)             (max normal idx - critical int count)                                        |
 * |                                                                                                             |
 * | [Lower Pri Critical ISR*]     --> [Lower Pri Critical ISR*]      --> ... --> [Lowest Pri Critical ISR*] --> |
 * | (min critical idx + 1)(*return)   (min critical idx + 2)(*return)            (max critical idx)(*return)    |
 * |                                                                                                             |
 * | [Task Critical Section] --> Exit Critical Section -->                                                       |
 * |                                                                                                             |
 * | [Highest Pri Pending Normal ISR triggered by Critical ISR] --> Trigger Higher Pri Normal ISR -->            |
 * | (max normal idx - critical int count)                                                                       |
 * |                                                                                                             |
 * | [Higher Pri Normal ISR]                  --> Trigger Higher Pri Normal ISR -->                              |
 * | (max normal idx - critical int count - 1)                                                                   |
 * |                                                                                                             |
 * | ...                                                                                                         |
 * |                                                                                                             |
 * | [Highest Pri Normal ISR] --> [Lower Pri Critical ISR*]    --> [Lower Pri Critical ISR*]    --> ... -->      |
 * | (min normal idx)             (min normal idx + 1)(*return)    (min normal idx + 2)(*return)                 |
 * |                                                                                                             |
 * | [Highest Pri Pending Normal ISR triggered by Critical ISR*] -->                                             |
 * | (max normal idx - critical int count)(*return)                                                              |
 * |                                                                                                             |
 * | [Next Highest Pri Pending Normal ISR (lower priority) triggered by Critical ISR]                            |
 * | (max normal idx - critical int count + 1)                                                                   |
 * |                                                                                                             |
 * | ...                                                                                                         |
 * |                                                                                                             |
 * | [Lowest Pri Normal ISR]            --> [Task]                                                               |
 * | (max normal idx)(triggered by task)                                                                         |
 * ---------------------------------------------------------------------------------------------------------------
 * 
 * For example, with 16 priority levels and critical section interrupt priority mask set to 4 
 * (level 0 - 3 allowed in critical section),
 * - Task triggers normal interrupt of priority 15 and critical interrupt of priority 3.
 * - Critical ISR for priority 3 will be executed first, 
 *   which will trigger normal interrupt of priority 14 and critical interrupt of priority 2.
 * - Critical ISR for priority 2 will be executed next, 
 *   which will trigger normal interrupt of priority 13 and critical interrupt of priority 1.
 * - Critical ISR for priority 1 will be executed next,
 *   which will trigger normal interrupt of priority 12 and critical interrupt of priority 0.
 * - Critical ISR for priority 0 will be executed next, 
 *   which will trigger normal interrupt of priority 11.
 * - Critical ISR for priority 0 will return, and then critical ISR for priority 1 will return, and so on 
 *   until all critical ISRs are returned.
 * - Then after exiting the task critical section, it switches to the highest priority pending normal ISR 
 *   (priority 11), which was triggered by the critical ISR for priority 0.
 * - The normal ISR for priority 11 will trigger the next higher priority normal ISR (priority 10).
 * - The normal ISR for priority 10 will trigger the next higher priority normal ISR (priority 9).
 * - This will continue until the highest priority normal ISR (priority 4).
 * - Then the normal ISR for priority 4 will return, and then normal ISR for priority 5 will return, and so on
 *   until all normal ISRs upto priority 11 are returned.
 * - Then it executes normal ISR for priority 12, which was triggered by the critical ISR for priority 1.
 * - Normal ISR for priority 13 will be executed next, which was triggered by the critical ISR for priority 2.
 * - Normal ISR for priority 14 will be executed next, which was triggered by the critical ISR for priority 3.
 * - Finally, the normal ISR for priority 15 will be executed, which was triggered by the task itself. 
 */
void test_criticalSectionFromIsrDeepNested(void *args)
{
    bool isPulse = (bool)args;

    DebugP_log("Testing with %s interrupts\r\n", isPulse ? "pulse" : "level");

    test_register_interrupts(isPulse);

    for(uint32_t loopCnt = 0U; loopCnt < TEST_LOOP_CNT; loopCnt++)
    {
        test_reset_variables();

        taskENTER_CRITICAL();

        /* Trigger lowest priority normal interrupt. 
        * This shouldn't fire until exit from critical section as well as all other high priority pending interrupts are serviced. */
        HwiP_post(NORMAL_ISR_LOWEST_PRI_INT_NUM);
        /* Trigger the lowest priority critical interrupt (priority still higher than normal interrupts)
        * This should fire immediately */
        HwiP_post(CRITICAL_ISR_LOWEST_PRI_INT_NUM);

        /* Each critical ISR triggers the next higher critical interrupt as well as normal interrupts.
        * It should switch to higher priority critical ISR immediately.
        * All normal ISR should be executed only after exit from the task critical section. */

        TEST_ASSERT_EACH_EQUAL_UINT32(1U, gCriticalIsrTracker, CRITICAL_ISR_COUNT);   /* Make sure all critical ISRs fired */
        TEST_ASSERT_EACH_EQUAL_UINT32(0U, gNormalIsrTracker  , NORMAL_ISR_COUNT);     /* Make sure none of the normal ISRs are fired yet */

        taskEXIT_CRITICAL();

        /* All normal ISRs should get executed now */
        /* Each normal ISR triggers the next higher normal interrupt (if not already executed). */

        TEST_ASSERT_EACH_EQUAL_UINT32(1U, gCriticalIsrTracker, CRITICAL_ISR_COUNT);   /* Make sure all critical ISRs count remains same */
        TEST_ASSERT_EACH_EQUAL_UINT32(1U, gNormalIsrTracker, NORMAL_ISR_COUNT);       /* Make sure all normal ISRs fired */

        /* Validate the order of ISR entry sequence */
        TEST_ASSERT_EQUAL_UINT32_ARRAY(test_get_expected_isr_entry_sequence(), gIsrEntryTracker, ISR_COUNT); 
        /* Validate the order of ISR exit sequence */
        TEST_ASSERT_EQUAL_UINT32_ARRAY(test_get_expected_isr_exit_sequence() , gIsrExitTracker,  ISR_COUNT);
        /* Make sure no error in execution flow from normal ISRs */
        TEST_ASSERT_EQUAL_UINT32(0U, gNormalIsrErrorCnt);
    }

    test_unregister_interrupts();
}

#endif /* configUSE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS==1 */

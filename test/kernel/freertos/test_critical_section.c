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

/** 
 * Defines
 */

 /* Interrupt Numbers */
#define HI_PRI_INT_NUM    (20u)
#define LO_PRI_INT_NUM    (21u)
#define BASE_PRI_INT_NUM  (22u)

/* Interrupt Priorities */
#define HI_PRI_INT_PRI    (configMAX_SYSCALL_INTERRUPT_PRIORITY - 1U)  /* High priority, enabled in critical section */
#define LO_PRI_INT_PRI    (configMAX_SYSCALL_INTERRUPT_PRIORITY)       /* Low priority, disabled in critical section */
#define BASE_PRI_INT_PRI  (HwiP_MAX_PRIORITY - 1U)                     /* Lowest priority, disabled in critical section */

#define CHECK_UPDATE_ERR_CNT(exp, actual, errCnt)   do { if((exp) != (actual)) { (errCnt)++; } } while(0)

/** 
 * Global Variables
 */

/* Variables used to track ISR counts */
static volatile uint32_t gHiPriIsrCnt        = 0U;
static volatile uint32_t gLoPriIsrCnt        = 0U;
static volatile uint32_t gBasePriIsrCnt      = 0U;
/* Variable used to track errors in base priority ISR */
static volatile uint32_t gBasePriIsrErrorCnt = 0U;
/* Variables used to control interrupt retrigger from ISRs */
static volatile uint32_t gHiPriReTrigEnable  = 0U;
static volatile uint32_t gLoPriReTrigEnable  = 0U;

static HwiP_Object gHiPriHwiObj;
static HwiP_Object gLoPriHwiObj;
static HwiP_Object gBasePriHwiObj;

/** 
 * ISR Functions
 */

/* High Priority ISR - This will run even when in a critical section */
static void hi_pri_isr(void *arg)
{
    gHiPriIsrCnt++;                 /* Increment the count variable */

    if(gHiPriReTrigEnable == 1U)    /* Re-trigger the interrupts if requested */
    {
        HwiP_post(HI_PRI_INT_NUM);  /* This should result in entering this ISR once again after completion (for pulse interrupts)*/
        HwiP_post(LO_PRI_INT_NUM);  /* This should fire only after exiting from critical section, if any active */
        gHiPriReTrigEnable = 0U;    /* Reset the retrigger variable, so that retrigger only happens once */
    }
}

/* Low Priority ISR - This won't run when in a critical section */
static void lo_pri_isr(void *arg)
{
    gLoPriIsrCnt++;                 /* Increment the count variable */

    if(gLoPriReTrigEnable == 1U)    /* Re-trigger the interrupts if requested */
    {
        HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */
        HwiP_post(LO_PRI_INT_NUM);  /* This should result in entering this ISR once again after completion (for pulse interrupts) */
        gLoPriReTrigEnable = 0U;    /* Reset the retrigger variable, so that retrigger only happens once */
    }
}

#ifdef HWIP_NESTED_INTERRUPTS_IRQ_ENABLE
/* Base Priority ISR - This won't run when in a critical section 
 *
 * ----------------------------------------------------------------------------------------------------------
 * | [Base ISR] --> Enter Critical Section #1 --> Trig Hi Pri Int --> [Hi Pri ISR] -->                      |
 * |                                                                  (H1)                                  |
 * |                                                                                                        |
 * | [ISR Critical Section #1] --> Trig Lo Pri Int --> Enter Critical Section #2 --> Trig Hi Pri Int        |
 * |                                                                                                        |
 * | [Hi Pri ISR] --> [ISR Critical Section #2] --> Trig Lo Pri Int                  -->                    |
 * | (H2)                                           (discarded since already pending)                       |
 * |                                                                                                        |
 * | Exit Critical Section #2 --> [ISR Critical Section #1] --> Trig Hi Pri Int --> [Hi Pri ISR] -->        |
 * |                                                                                (H3)                    |
 * |                                                                                                        |
 * | Trig Lo Pri Int                  --> Exit Critical Section #1 --> [Lo Pri ISR] --> [Base ISR] -->      |
 * | (discarded since already pending)                                 (L1)                                 |
 * |                                                                                                        |
 * | Trig Hi Pri Int --> [Hi Pri ISR] --> [Base ISR] --> Trig Lo Pri Int --> [Lo Pri ISR] --> [Base ISR]    |
 * |                     (H4)                                                    (L2)                       |
 * ----------------------------------------------------------------------------------------------------------
 * 
*/
static void base_pri_isr(void *arg)
{
    uint32_t key1, key2;
    
    uint32_t hiPriIsrCntExp = gBasePriIsrCnt * 4U; /* Each time this ISR runs, it will fire high priority ISR 4x */
    uint32_t loPriIsrCntExp = gBasePriIsrCnt * 2U; /* Each time this ISR runs, it will fire low priority  ISR 2x */

    gBasePriIsrCnt++;               /* Increment the count variable */

    key1 = taskENTER_CRITICAL_FROM_ISR();          /* Enter critical section in ISR - Nesting count = 1 */

    HwiP_post(HI_PRI_INT_NUM);      /* This should fire immediately resulting in switch to high priority ISR */
    HwiP_post(LO_PRI_INT_NUM);      /* This should fire only after exiting from critical section */

    CHECK_UPDATE_ERR_CNT(++hiPriIsrCntExp, gHiPriIsrCnt, gBasePriIsrErrorCnt); /* Make sure the high priority ISR was fired */
    CHECK_UPDATE_ERR_CNT(  loPriIsrCntExp, gLoPriIsrCnt, gBasePriIsrErrorCnt); /* Make sure the low priority ISR was not fired since in critical section */

    key2 = taskENTER_CRITICAL_FROM_ISR();          /* Enter critical section in ISR again - Nesting count = 2 */

    HwiP_post(HI_PRI_INT_NUM);      /* This should fire immediately resulting in switch to high priority ISR */
    HwiP_post(LO_PRI_INT_NUM);      /* This won't have any effect since already pending  */

    CHECK_UPDATE_ERR_CNT(++hiPriIsrCntExp, gHiPriIsrCnt, gBasePriIsrErrorCnt); /* Make sure the high priority ISR was fired */
    CHECK_UPDATE_ERR_CNT(  loPriIsrCntExp, gLoPriIsrCnt, gBasePriIsrErrorCnt); /* Make sure the low priority ISR was not fired yet since in critical section */

    taskEXIT_CRITICAL_FROM_ISR(key2);           /* Exit critical section in ISR - Nesting count = 1 */

    /* Low priority ISR shouldn't fire now since still its in a critical section with Nesting Count = 1 */

    HwiP_post(HI_PRI_INT_NUM);      /* This should fire immediately resulting in switch to high priority ISR */
    HwiP_post(LO_PRI_INT_NUM);      /* This won't have any effect since already pending  */

    CHECK_UPDATE_ERR_CNT(++hiPriIsrCntExp, gHiPriIsrCnt, gBasePriIsrErrorCnt); /* Make sure the high priority ISR was fired */
    CHECK_UPDATE_ERR_CNT(  loPriIsrCntExp, gLoPriIsrCnt, gBasePriIsrErrorCnt); /* Make sure the low priority ISR was not fired yet since in critical section */

    taskEXIT_CRITICAL_FROM_ISR(key1);           /* Exit critical section in ISR - Nesting count = 0 */
    
    /* Low priority ISR should fire now since it exited from critical section */
    
    /* Now trigger again from outside of critical section */
    
    HwiP_post(HI_PRI_INT_NUM);      /* This should fire immediately resulting in switch to high priority ISR */
    HwiP_post(LO_PRI_INT_NUM);      /* This should fire immediately resulting in switch to low priority  ISR */

    /* Final ISR count will be tested from test task */
}
#endif /* HWIP_NESTED_INTERRUPTS_IRQ_ENABLE */

/** 
 * Utility functions
 */

static void test_register_isr(uint32_t intNum, HwiP_FxnCallback callback, uint32_t priority, bool isPulse, 
                              HwiP_Object *hwiObj, void *arg)
{
    HwiP_Params hwiParams;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum   = intNum;
    hwiParams.callback = callback;
    hwiParams.priority = priority;
    hwiParams.isPulse  = isPulse;
    hwiParams.args     = arg;
    HwiP_construct(hwiObj, &hwiParams);
}

/** 
 * Unity test functions
 * 
 * - These will be run by the ping task
 */

/** Test for critical sections from task context 
 * 
 * ----------------------------------------------------------------------------------------------------------
 * | [Task] --> Enter Critical Section --> Trig Hi Pri Int --> [Hi Pri ISR] --> [Task Critical Section] --> |
 * |                                                           (H1)                                         |
 * |                                                                                                        |
 * | Trig Lo Pri Int --> Exit Critical Section --> [Lo Pri ISR] --> [Task] -->  Trig Hi Pri Int -->         |
 * |                                               (L1)                                                     |
 * |                                                                                                        |
 * | [Hi Pri ISR] --> [Task] -->  Trig Lo Pri Int --> [Lo Pri ISR] --> [Task]                               |
 * | (H2)                                             (L2)                                                  |
 * ----------------------------------------------------------------------------------------------------------
 */
void test_criticalSectionFromTask(void *args)
{
    bool isPulse = (bool)args;

    DebugP_log("Testing with %s interrupts\r\n", isPulse ? "pulse" : "level");

    volatile uint32_t oldHiPriIsrCnt = 0U; 
    volatile uint32_t oldLoPriIsrCnt = 0U;

    uint32_t hiPriIsrCntExp = 0U;
    uint32_t loPriIsrCntExp = 0U;

    gHiPriIsrCnt = 0U;
    gLoPriIsrCnt = 0U;

    /* Register high priority interrupt, enabled during freertos critical section */
    test_register_isr(HI_PRI_INT_NUM, hi_pri_isr, HI_PRI_INT_PRI, isPulse, &gHiPriHwiObj, NULL);
    /* Register low priority interrupt, disabled during freertos critical section */
    test_register_isr(LO_PRI_INT_NUM, lo_pri_isr, LO_PRI_INT_PRI, isPulse, &gLoPriHwiObj, NULL);

    for(uint32_t loopCnt = 0U; loopCnt < TEST_LOOP_CNT; loopCnt++)
    {
        taskENTER_CRITICAL();

        HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */
        HwiP_post(LO_PRI_INT_NUM);  /* This should fire only after exiting from critical section */

        /* Test asserts can't be used in critical section since this relies on logging
        * and logging may use delays. Delay won't work since ticker timer interrupt(low priority) is masked.
        * Hence store the ISR count values to check after exiting from critical section. */
        oldHiPriIsrCnt = gHiPriIsrCnt;
        oldLoPriIsrCnt = gLoPriIsrCnt;

        taskEXIT_CRITICAL();

        /* Low priority ISR should fire now since it exited from critical section */

        /* Check ISR counts in critical sections */
        TEST_ASSERT_EQUAL_UINT32(++hiPriIsrCntExp, oldHiPriIsrCnt);   /* Make sure the high priority ISR was fired */
        TEST_ASSERT_EQUAL_UINT32(  loPriIsrCntExp, oldLoPriIsrCnt);   /* Make sure the low priority  ISR was not fired since in critical section */

        /* Check ISR counts after critical sections */
        TEST_ASSERT_EQUAL_UINT32(  hiPriIsrCntExp, gHiPriIsrCnt);     /* Make sure the high priority ISR count remain same */
        TEST_ASSERT_EQUAL_UINT32(++loPriIsrCntExp, gLoPriIsrCnt);     /* Make sure the low priority  ISR was fired after critical section */

        /* Now repeat same from outside of critical section */

        HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */          
        HwiP_post(LO_PRI_INT_NUM);  /* This should fire immediately resulting in switch to low priority  ISR */

        TEST_ASSERT_EQUAL_UINT32(++hiPriIsrCntExp, gHiPriIsrCnt);     /* Make sure the high priority ISR was fired */
        TEST_ASSERT_EQUAL_UINT32(++loPriIsrCntExp, gLoPriIsrCnt);     /* Make sure the low priority  ISR was fired */
    }

    HwiP_destruct(&gHiPriHwiObj);
    HwiP_destruct(&gLoPriHwiObj);
}

/** Test for critical sections from task context with Hi Pri ISR retriggering Hi & Lo Int once (Pulse) 
 * 
 * ---------------------------------------------------------------------------------------------------------
 * | [Task] --> Enter Critical Section --> Trig Hi Pri Int --> [Hi Pri ISR] --> Trig Hi Pri Int -->        |
 * |                                                           (H1)                                        |
 * |                                                                                                       |
 * | Trig Lo Pri Int --> [Hi Pri ISR] --> [Task Critical Section] --> Trig Lo Pri Int                  --> |
 * |                     (H2)                                         (discarded since already pending)    |
 * |                                                                                                       |
 * | Exit Critical Section --> [Lo Pri ISR] --> [Task] --> Trig Hi Pri Int --> [Hi Pri ISR] -->            |
 * |                           (L1)                                            (H3)                        |
 * |                                                                                                       |
 * | Trig Hi Pri Int --> Trig Lo Pri Int --> [Hi Pri ISR] --> [Lo Pri ISR] --> [Task] -->                  |
 * |                                         (H4)             (L2)                                         |
 * |                                                                                                       |
 * | Trig Lo Pri Int --> [Lo Pri ISR] --> [Task]                                                           |
 * |                     (L3)                                                                              |
 * ---------------------------------------------------------------------------------------------------------
 */
void test_criticalSectionFromTaskWithHiPriReTrigPulse(void *args)
{
    volatile uint32_t oldHiPriIsrCnt = 0U;
    volatile uint32_t oldLoPriIsrCnt = 0U;

    uint32_t hiPriIsrCntExp = 0U;
    uint32_t loPriIsrCntExp = 0U;

    gHiPriIsrCnt = 0U;
    gLoPriIsrCnt = 0U;

    test_register_isr(HI_PRI_INT_NUM, hi_pri_isr, HI_PRI_INT_PRI, true, &gHiPriHwiObj, NULL);
    test_register_isr(LO_PRI_INT_NUM, lo_pri_isr, LO_PRI_INT_PRI, true, &gLoPriHwiObj, NULL);

    for(uint32_t loopCnt = 0U; loopCnt < TEST_LOOP_CNT; loopCnt++)
    {
        taskENTER_CRITICAL();

        gHiPriReTrigEnable = 1U;    /* Enable retrigger of hi and lo pri interrupts from high priority ISR */
        HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */
                                    /* High priority interrupt will be fired again due to re-trigger. */
                                    /* Low priority interrupt stays pending due to critical section */
        HwiP_post(LO_PRI_INT_NUM);  /* This won't have any effect since already pending  */

        oldHiPriIsrCnt = gHiPriIsrCnt;
        oldLoPriIsrCnt = gLoPriIsrCnt;

        taskEXIT_CRITICAL();

        /* Low priority ISR should fire once now since it exited from critical section */

        /* Check ISR counts in critical sections */
        hiPriIsrCntExp += 2U;                                      /* Make sure the high priority ISR was fired twice (1 from above and 1 due to retrigger)*/
        TEST_ASSERT_EQUAL_UINT32(hiPriIsrCntExp, oldHiPriIsrCnt);
        TEST_ASSERT_EQUAL_UINT32(loPriIsrCntExp, oldLoPriIsrCnt);  /* Make sure the low priority ISR was not fired since in critical section */

        /* Check ISR counts after critical sections */
        TEST_ASSERT_EQUAL_UINT32(  hiPriIsrCntExp, gHiPriIsrCnt);    /* Make sure the high priority ISR count remain same */
        TEST_ASSERT_EQUAL_UINT32(++loPriIsrCntExp, gLoPriIsrCnt);    /* Make sure the low priority ISR was fired once after critical section */

        /* Now repeat same from outside of critical section */

        gHiPriReTrigEnable = 1U;    /* Enable retrigger of hi and lo pri interrupts from high priority ISR */
        HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */

        hiPriIsrCntExp += 2U;                                      /* Make sure the high priority ISR was fired twice (1 from above and 1 due to retrigger)*/
        TEST_ASSERT_EQUAL_UINT32(  hiPriIsrCntExp, gHiPriIsrCnt);
        TEST_ASSERT_EQUAL_UINT32(++loPriIsrCntExp, gLoPriIsrCnt);  /* Make sure the low priority ISR  was fired due to retrigger */

        HwiP_post(LO_PRI_INT_NUM);  /* This should fire immediately resulting in switch to low priority ISR */

        TEST_ASSERT_EQUAL_UINT32(  hiPriIsrCntExp, gHiPriIsrCnt);  /* Make sure the high priority ISR count remain same */
        TEST_ASSERT_EQUAL_UINT32(++loPriIsrCntExp, gLoPriIsrCnt);  /* Make sure the low priority ISR was fired */
    }

    HwiP_destruct(&gHiPriHwiObj);
    HwiP_destruct(&gLoPriHwiObj);
}

/** Test for critical sections from task context with Hi Pri ISR retriggering Hi & Lo Int once (Level) 
 * 
 * ---------------------------------------------------------------------------------------------------------
 * | [Task] --> Enter Critical Section --> Trig Hi Pri Int --> [Hi Pri ISR] -->                            |
 * |                                                           (H1)                                        |
 * |                                                                                                       |
 * | Trig Hi Pri Int                       --> Trig Lo Pri Int --> [Task Critical Section] -->             |
 * | (discarded since level already active)                                                                |
 * |                                                                                                       |
 * | Trig Lo Pri Int                  --> Exit Critical Section --> [Lo Pri ISR] --> [Task] -->            |
 * | (discarded since already pending)                              (L1)                                   |
 * |                                                                                                       |
 * | Trig Hi Pri Int --> [Hi Pri ISR] --> Trig Hi Pri Int                       --> Trig Lo Pri Int -->    |
 * |                     (H2)             (discarded since level already active)                           |
 * |                                                                                                       |
 * | [Lo Pri ISR] --> [Task] --> Trig Lo Pri Int --> [Lo Pri ISR] --> [Task]                               |
 * | (L2)                                            (L3)                                                  |
 * ---------------------------------------------------------------------------------------------------------
 */
void test_criticalSectionFromTaskWithHiPriReTrigLevel(void *args)
{
    volatile uint32_t oldHiPriIsrCnt = 0U;
    volatile uint32_t oldLoPriIsrCnt = 0U;

    uint32_t hiPriIsrCntExp = 0U;
    uint32_t loPriIsrCntExp = 0U;

    gHiPriIsrCnt = 0U;
    gLoPriIsrCnt = 0U;

    test_register_isr(HI_PRI_INT_NUM, hi_pri_isr, HI_PRI_INT_PRI, false, &gHiPriHwiObj, NULL);
    test_register_isr(LO_PRI_INT_NUM, lo_pri_isr, LO_PRI_INT_PRI, false, &gLoPriHwiObj, NULL);

    for(uint32_t loopCnt = 0U; loopCnt < TEST_LOOP_CNT; loopCnt++)
    {
        taskENTER_CRITICAL();

        gHiPriReTrigEnable = 1U;    /* Enable retrigger of hi and lo pri interrupts from high priority ISR */
        HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */
                                    /* High priority interrupt won't be fired again since its level interrupt and was already active. */
                                    /* Low priority interrupt stays pending due to critical section */
        HwiP_post(LO_PRI_INT_NUM);  /* This won't have any effect since already pending  */

        oldHiPriIsrCnt = gHiPriIsrCnt;
        oldLoPriIsrCnt = gLoPriIsrCnt;

        taskEXIT_CRITICAL();

        /* Low priority ISR should fire once now since it exited from critical section */

        /* Check ISR counts in critical sections */
        TEST_ASSERT_EQUAL_UINT32(++hiPriIsrCntExp, oldHiPriIsrCnt);   /* Make sure the high priority ISR was fired once */
        TEST_ASSERT_EQUAL_UINT32(  loPriIsrCntExp, oldLoPriIsrCnt);   /* Make sure the low priority  ISR was not fired since in critical section */

        /* Check ISR counts after critical sections */
        TEST_ASSERT_EQUAL_UINT32(  hiPriIsrCntExp, gHiPriIsrCnt);     /* Make sure the high priority ISR count remain same */
        TEST_ASSERT_EQUAL_UINT32(++loPriIsrCntExp, gLoPriIsrCnt);     /* Make sure the low priority  ISR was fired once after critical section */

        /* Now repeat same from outside of critical section */

        gHiPriReTrigEnable = 1U;    /* Enable retrigger of hi and lo pri interrupts from high priority ISR */ 
        HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */
                                    /* High priority interrupt won't be fired again since its level interrupt and was already active. */
                                    /* Low priority interrupt should be fired soon after exiting from high priority ISR */
                                    
        TEST_ASSERT_EQUAL_UINT32(++hiPriIsrCntExp, gHiPriIsrCnt);     /* Make sure the high priority ISR was fired once */
        TEST_ASSERT_EQUAL_UINT32(++loPriIsrCntExp, gLoPriIsrCnt);     /* Make sure the low priority  ISR was fired due to retrigger */

        HwiP_post(LO_PRI_INT_NUM);  /* This should fire immediately resulting in switch to low priority ISR */

        TEST_ASSERT_EQUAL_UINT32(  hiPriIsrCntExp, gHiPriIsrCnt);     /* Make sure the high priority ISR count remain same */
        TEST_ASSERT_EQUAL_UINT32(++loPriIsrCntExp, gLoPriIsrCnt);     /* Make sure the low priority  ISR was fired */
    }

    HwiP_destruct(&gHiPriHwiObj);
    HwiP_destruct(&gLoPriHwiObj);
}

/** Test for critical sections from task context with Lo Pri ISR retriggering Hi & Lo Int once (Pulse) 
 * 
 * -----------------------------------------------------------------------------------------------------------
 * | [Task] --> Enter Critical Section --> Trig Hi Pri Int --> [Hi Pri ISR] -->  [Task Critical Section] --> |
 * |                                                           (H1)                                          |
 * |                                                                                                         |
 * | Trig Lo Pri Int --> Exit Critical Section --> [Lo Pri ISR] -->  Trig Hi Pri Int --> [Hi Pri ISR] -->    |
 * |                                               (L1)                                  (H2)                |
 * |                                                                                                         |
 * | [Lo Pri ISR*] --> Trig Lo Pri Int --> [Lo Pri ISR] --> [Task] --> Trig Hi Pri Int --> [Hi Pri ISR] -->  |
 * | (*return)                             (L2)                                            (H3)              |
 * |                                                                                                         |
 * | [Task] --> Trig Lo Pri Int --> [Lo Pri ISR] --> Trig Hi Pri Int --> [Hi Pri ISR] --> [Lo Pri ISR*] -->  |
 * |                                (L3)                                 (H4)             (*return)          |
 * |                                                                                                         |
 * | Trig Lo Pri Int --> [Lo Pri ISR] --> [Task]                                                             |
 * |                     (L4)                                                                                |
 * -----------------------------------------------------------------------------------------------------------
 */
void test_criticalSectionFromTaskWithLoPriReTrigPulse(void *args)
{
    volatile uint32_t oldHiPriIsrCnt = 0U;
    volatile uint32_t oldLoPriIsrCnt = 0U;

    uint32_t hiPriIsrCntExp = 0U;
    uint32_t loPriIsrCntExp = 0U;

    gHiPriIsrCnt = 0U;
    gLoPriIsrCnt = 0U;

    test_register_isr(HI_PRI_INT_NUM, hi_pri_isr, HI_PRI_INT_PRI, true, &gHiPriHwiObj, NULL);
    test_register_isr(LO_PRI_INT_NUM, lo_pri_isr, LO_PRI_INT_PRI, true, &gLoPriHwiObj, NULL);

    for(uint32_t loopCnt = 0U; loopCnt < TEST_LOOP_CNT; loopCnt++)
    {
        taskENTER_CRITICAL();

        gLoPriReTrigEnable = 1U;    /* Enable retrigger of hi and lo pri interrupts from low priority ISR */
        HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */
        HwiP_post(LO_PRI_INT_NUM);  /* This should fire only after exiting from critical section */

        oldHiPriIsrCnt = gHiPriIsrCnt;
        oldLoPriIsrCnt = gLoPriIsrCnt;

        taskEXIT_CRITICAL();

        /* Low priority ISR should fire now since it exited from critical section */
        /* High and Low priority interrupts will be fired again due to re-trigger. */

        /* Check ISR counts in critical sections */
        TEST_ASSERT_EQUAL_UINT32(++hiPriIsrCntExp, oldHiPriIsrCnt);   /* Make sure the high priority ISR was fired */
        TEST_ASSERT_EQUAL_UINT32(  loPriIsrCntExp, oldLoPriIsrCnt);   /* Make sure the low priority  ISR was not fired since in critical section */

        /* Check ISR counts after critical sections */
        TEST_ASSERT_EQUAL_UINT32(++hiPriIsrCntExp, gHiPriIsrCnt);     /* Make sure the high priority ISR was fired due to retrigger */
        loPriIsrCntExp += 2U;                                         /* Make sure the low priority  ISR was fired twice (1 from above and 1 due to retrigger) */
        TEST_ASSERT_EQUAL_UINT32(  loPriIsrCntExp, gLoPriIsrCnt);

        /* Now repeat same from outside of critical section */

        HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */
        
        TEST_ASSERT_EQUAL_UINT32(++hiPriIsrCntExp, gHiPriIsrCnt);     /* Make sure the high priority ISR was fired */    
        TEST_ASSERT_EQUAL_UINT32(  loPriIsrCntExp, gLoPriIsrCnt);     /* Make sure the low priority  ISR count remain same */

        gLoPriReTrigEnable = 1U;    /* Enable retrigger of hi and lo pri interrupts from low priority ISR */
        HwiP_post(LO_PRI_INT_NUM);  /* This should fire immediately resulting in switch to low priority ISR */

        TEST_ASSERT_EQUAL_UINT32(++hiPriIsrCntExp, gHiPriIsrCnt);     /* Make sure the high priority ISR was fired due to retrigger */
        loPriIsrCntExp += 2U;                                         /* Make sure the low priority  ISR was fired twice (1 from above and 1 due to retrigger) */
        TEST_ASSERT_EQUAL_UINT32(  loPriIsrCntExp, gLoPriIsrCnt);
    }

    HwiP_destruct(&gHiPriHwiObj);
    HwiP_destruct(&gLoPriHwiObj);
}

/** Test for critical sections from task context with Lo Pri ISR retriggering Hi & Lo Int once (Level) 
 * 
 * -----------------------------------------------------------------------------------------------------------
 * | [Task] --> Enter Critical Section --> Trig Hi Pri Int --> [Hi Pri ISR] -->  [Task Critical Section] --> |
 * |                                                           (H1)                                          |
 * |                                                                                                         |
 * | Trig Lo Pri Int --> Exit Critical Section --> [Lo Pri ISR] -->  Trig Hi Pri Int --> [Hi Pri ISR] -->    |
 * |                                               (L1)                                  (H2)                |
 * |                                                                                                         |
 * | [Lo Pri ISR*] --> Trig Lo Pri Int                       --> [Task] --> Trig Hi Pri Int -->              |
 * | (*return)         (discarded since level already active)                                                |
 * |                                                                                                         |
 * |                                                                                                         |
 * | [Hi Pri ISR] --> [Task] --> Trig Lo Pri Int --> [Lo Pri ISR] --> Trig Hi Pri Int --> [Hi Pri ISR] -->   |
 * | (H3)                                            (L2)                                 (H4)               |
 * |                                                                                                         |
 * | [Lo Pri ISR*] --> Trig Lo Pri Int                       --> [Task]                                      |
 * | (*return)         (discarded since level already active)                                                |
 * -----------------------------------------------------------------------------------------------------------
 */
void test_criticalSectionFromTaskWithLoPriReTrigLevel(void *args)
{
    volatile uint32_t oldHiPriIsrCnt = 0U;
    volatile uint32_t oldLoPriIsrCnt = 0U;

    uint32_t hiPriIsrCntExp = 0U;
    uint32_t loPriIsrCntExp = 0U;

    gHiPriIsrCnt = 0U;
    gLoPriIsrCnt = 0U;

    test_register_isr(HI_PRI_INT_NUM, hi_pri_isr, HI_PRI_INT_PRI, false, &gHiPriHwiObj, NULL);
    test_register_isr(LO_PRI_INT_NUM, lo_pri_isr, LO_PRI_INT_PRI, false, &gLoPriHwiObj, NULL);

    for(uint32_t loopCnt = 0U; loopCnt < TEST_LOOP_CNT; loopCnt++)
    {
        taskENTER_CRITICAL();

        gLoPriReTrigEnable = 1U;    /* Enable retrigger of hi and lo pri interrupts from low priority ISR */
        HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */
        HwiP_post(LO_PRI_INT_NUM);  /* This should fire only after exiting from critical section */

        oldHiPriIsrCnt = gHiPriIsrCnt;
        oldLoPriIsrCnt = gLoPriIsrCnt;

        taskEXIT_CRITICAL();

        /* Low priority ISR should fire now since it exited from critical section */
        /* High priority interrupt will be fired again due to re-trigger. */
        /* Low priority interrupt won't be fired again since its level interrupt and is already active. */

        /* Check ISR counts in critical sections */
        TEST_ASSERT_EQUAL_UINT32(++hiPriIsrCntExp, oldHiPriIsrCnt);   /* Make sure the high priority ISR was fired */
        TEST_ASSERT_EQUAL_UINT32(  loPriIsrCntExp, oldLoPriIsrCnt);   /* Make sure the low priority  ISR was not fired since in critical section */

        /* Check ISR counts after critical sections */
        TEST_ASSERT_EQUAL_UINT32(++hiPriIsrCntExp, gHiPriIsrCnt);     /* Make sure the high priority ISR was fired due to retrigger */
        TEST_ASSERT_EQUAL_UINT32(++loPriIsrCntExp, gLoPriIsrCnt);     /* Make sure the low priority  ISR was fired once after critical section */

        /* Now repeat same from outside of critical section */

        HwiP_post(HI_PRI_INT_NUM);   /* This should fire immediately resulting in switch to high priority ISR */

        TEST_ASSERT_EQUAL_UINT32(++hiPriIsrCntExp, gHiPriIsrCnt);     /* Make sure the high priority ISR was fired */
        TEST_ASSERT_EQUAL_UINT32(  loPriIsrCntExp, gLoPriIsrCnt);     /* Make sure the low priority  ISR count remain same */

        gLoPriReTrigEnable = 1U;    /* Enable retrigger of hi and lo pri interrupts from low priority ISR */
        HwiP_post(LO_PRI_INT_NUM);  /* This should fire immediately resulting in switch to low priority ISR */
                                    /* High priority interrupt will be fired again due to re-trigger. */
                                    /* Low priority interrupt won't be fired again since its level interrupt and is already active. */
        
        TEST_ASSERT_EQUAL_UINT32(++hiPriIsrCntExp, gHiPriIsrCnt);     /* Make sure the high priority ISR was fired due to retrigger */
        TEST_ASSERT_EQUAL_UINT32(++loPriIsrCntExp, gLoPriIsrCnt);     /* Make sure the low priority  ISR was fired once */
    }

    HwiP_destruct(&gHiPriHwiObj);
    HwiP_destruct(&gLoPriHwiObj);
}

#ifdef HWIP_NESTED_INTERRUPTS_IRQ_ENABLE
/** Test for critical sections from ISR context  
 * 
 * ----------------------------------------------------------------------------------------------------------
 * | [Task] --> Trig Base Pri Int --> [Base Pri ISR] -->                                                    |
 * |                                  (B1)                                                                  |
 * | In Base Priority ISR,                                                                                  |
 * | - it enter critical section (nesting count = 1) and triggers Hi & Lo Pri Int                           |
 * | - Hi Pri ISR will run immediately since it is enabled in critical section (H1)                         |
 * | - it enters critical section again (nesting count = 2) and triggers Hi & Lo Pri Int                    |
 * | - Hi Pri ISR will run immediately (H2)                                                                 |
 * | - it exits critical section (nesting count = 1) and triggers Hi & Lo Pri Int                           |
 * | - Hi Pri ISR will run immediately (H3)                                                                 |
 * | - it exits critical section again (nesting count = 0)                                                  |
 * | - Lo Pri ISR will run once after exit from critical section (L1)                                       |
 * | - it triggers Hi & Lo Pri Int again                                                                    |
 * | - Hi Pri ISR will run immediately followed by Lo Pri ISR (H4, L2)                                      |
 * |                                                                                                        |
 * | Refer to the diagram in `base_pri_isr` for the flow.                                                   |
 * |                                                                                                        |
 * | [Base Pri ISR*] --> [Task]                                                                             |
 * | (*return)                                                                                              |
 * ----------------------------------------------------------------------------------------------------------
 */
void test_criticalSectionFromIsr(void *args)
{
    bool isPulse = (bool)args;

    DebugP_log("Testing with %s interrupts\r\n", isPulse ? "pulse" : "level");

    gHiPriIsrCnt        = 0U;
    gLoPriIsrCnt        = 0U;
    gBasePriIsrCnt      = 0U;
    gBasePriIsrErrorCnt = 0U;

    /* Register high priority interrupt, enabled even during ISR critical section */
    test_register_isr(HI_PRI_INT_NUM,   hi_pri_isr,   HI_PRI_INT_PRI,   isPulse, &gHiPriHwiObj,   NULL);
    /* Register low priority interrupt, disabled during ISR critical section */
    test_register_isr(LO_PRI_INT_NUM,   lo_pri_isr,   LO_PRI_INT_PRI,   isPulse, &gLoPriHwiObj,   NULL);
    /* Register base(lowest) priority interrupt, this enters to ISR critical section and runs the tests */
    test_register_isr(BASE_PRI_INT_NUM, base_pri_isr, BASE_PRI_INT_PRI, isPulse, &gBasePriHwiObj, NULL);

    for(uint32_t loopCnt = 1U; loopCnt <= TEST_LOOP_CNT; loopCnt++)
    {
        HwiP_post(BASE_PRI_INT_NUM);  /* This should fire immediately resulting in switch to base priority ISR */

        TEST_ASSERT_EQUAL_UINT32(loopCnt * 1U, gBasePriIsrCnt);       /* Make sure base priority ISR fired once */
        TEST_ASSERT_EQUAL_UINT32(loopCnt * 4U, gHiPriIsrCnt);         /* Make sure high priority ISR fired 4 times (3x - critical section, 1x - outside)*/
        TEST_ASSERT_EQUAL_UINT32(loopCnt * 2U, gLoPriIsrCnt);         /* Make sure low priority  ISR fired 2 times (1x - exit critical section, 1x - outside)*/
        TEST_ASSERT_EQUAL_UINT32(0U,           gBasePriIsrErrorCnt);  /* Make sure no error in execution flow from base priority ISR */
    }

    HwiP_destruct(&gHiPriHwiObj);
    HwiP_destruct(&gLoPriHwiObj);
    HwiP_destruct(&gBasePriHwiObj);
}
#endif /* HWIP_NESTED_INTERRUPTS_IRQ_ENABLE */

void test_freertos_critical_section_run(void)
{
    bool pulse = true, level = false;

    RUN_TEST(test_criticalSectionFromTask, 14776, (void*)pulse);
    RUN_TEST(test_criticalSectionFromTask, 14777, (void*)level);
    RUN_TEST(test_criticalSectionFromTaskWithHiPriReTrigPulse, 14778, NULL);
    RUN_TEST(test_criticalSectionFromTaskWithHiPriReTrigLevel, 14779, NULL);
    RUN_TEST(test_criticalSectionFromTaskWithLoPriReTrigPulse, 14780, NULL);
    RUN_TEST(test_criticalSectionFromTaskWithLoPriReTrigLevel, 14781, NULL);
#ifdef HWIP_NESTED_INTERRUPTS_IRQ_ENABLE
    /* nested interrupts should be enabled to test critical sections from ISR */
    RUN_TEST(test_criticalSectionFromIsr, 14782, (void*)pulse);
    RUN_TEST(test_criticalSectionFromIsr, 14783, (void*)level);
    RUN_TEST(test_criticalSectionFromIsrDeepNested, 14784, (void*)pulse);
    RUN_TEST(test_criticalSectionFromIsrDeepNested, 14785, (void*)level);
#endif /* HWIP_NESTED_INTERRUPTS_IRQ_ENABLE */
}

#endif /* configUSE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS==1 */
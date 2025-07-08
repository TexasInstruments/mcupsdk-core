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

#include <stdbool.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>
#include <unity.h>

#ifdef HWIP_USE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS
/** NOTE: This tests are enabled only for NORTOS.
 * 
 * For FreeRTOS, refer `test/kernel/freertos/test_critical_section.c` for extensive tests of critical sections
 * from tasks and ISRs.
*/

/** 
 * Defines
 */

 /* Interrupt Numbers */
#define HI_PRI_INT_NUM    (20u)
#define LO_PRI_INT_NUM    (21u)
#define BASE_PRI_INT_NUM  (22u)

/* Interrupt Priorities */
#define HI_PRI_INT_PRI    (HWIP_CRITICAL_SECTION_INTERRUPT_PRIORITY_THRESHOLD - 1U)  /* High priority,    enabled in critical section */
#define LO_PRI_INT_PRI    (HWIP_CRITICAL_SECTION_INTERRUPT_PRIORITY_THRESHOLD)       /* Low priority,    disabled in critical section */
#define BASE_PRI_INT_PRI  (HwiP_MAX_PRIORITY - 1U)                                   /* Lowest priority, disabled in critical section */

#define TEST_LOOP_CNT   (100U)

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

static HwiP_Object gHiPriHwiObj;
static HwiP_Object gLoPriHwiObj;
static HwiP_Object gBasePriHwiObj;

/* Private Functions Declarations */
static uint32_t test_critical_section(uint32_t loopCnt);

/** 
 * ISR Functions
 */

/* High Priority ISR - This will run even when in a critical section */
static void hi_pri_isr(void *arg)
{
    gHiPriIsrCnt++; /* Increment the count variable */
}

/* Low Priority ISR - This won't run when in a critical section */
static void lo_pri_isr(void *arg)
{
    gLoPriIsrCnt++; /* Increment the count variable */
}

/* Base(Lowest) Priority ISR - This won't run when in a critical section */
static void base_pri_isr(void *arg)
{    
    gBasePriIsrErrorCnt = test_critical_section(gBasePriIsrCnt);

    gBasePriIsrCnt++;   /* Increment the count variable */
}

/** Critical section test function which will run from Main or Base Priority ISR
 * 
 * This will perform the following:-
 * - enter critical section (nesting count = 1) and triggers Hi & Lo Pri Int
 * - Hi Pri ISR will run immediately since it is enabled in critical section (H1)
 * - enters critical section again (nesting count = 2) and triggers Hi & Lo Pri In
 * - Hi Pri ISR will run immediately (H2)
 * - exits critical section (nesting count = 1) and triggers Hi & Lo Pri Int
 * - Hi Pri ISR will run immediately (H3)
 * - exits critical section again (nesting count = 0)
 * - Lo Pri ISR will run once after exit from critical section (L1)
 * - triggers Hi & Lo Pri Int again
 * - Hi Pri ISR will run immediately followed by Lo Pri ISR (H4, L2)
 * 
 * --------------------------------------------------------------------------------------------------
 * | [Main / Base Pri ISR] --> Enter Critical Section #1 --> Trig Hi Pri Int --> [Hi Pri ISR] -->   |
 * |                                                                             (H1)               |
 * |                                                                                                |
 * | [Critical Section #1] --> Trig Lo Pri Int --> Enter Critical Section #2 --> Trig Hi Pri Int    |
 * |                                                                                                |
 * | [Hi Pri ISR] --> [Critical Section #2] --> Trig Lo Pri Int                  -->                |
 * | (H2)                                       (discarded since already pending)                   |
 * |                                                                                                |
 * | Exit Critical Section #2 --> [Critical Section #1] --> Trig Hi Pri Int --> [Hi Pri ISR] -->    |
 * |                                                                            (H3)                |
 * |                                                                                                |
 * | Trig Lo Pri Int                  --> Exit Critical Section #1 --> [Lo Pri ISR] -->             |
 * | (discarded since already pending)                                 (L1)                         |
 * |                                                                                                |
 * | [Main / Base Pri ISR] --> Trig Hi Pri Int --> [Hi Pri ISR] --> [Main / Base PriISR] -->        |
 * |                                               (H4)                                             |
 * |                                                                                                |
 * | Trig Lo Pri Int --> [Lo Pri ISR] --> [Main / Base Pri ISR]                                     |
 * |                     (L2)                                                                       |
 * --------------------------------------------------------------------------------------------------
 */

static uint32_t test_critical_section(uint32_t loopCnt)
{
    uint32_t errCnt = 0U;
    uint32_t key1, key2;
    
    uint32_t hiPriIsrCntExp = loopCnt * 4U; /* Each time this runs, it will fire high priority ISR 4x */
    uint32_t loPriIsrCntExp = loopCnt * 2U; /* Each time this runs, it will fire low priority  ISR 2x */

    key1 = HWIP_ENTER_CRITICAL_SECTION();   /* Enter critical section - Nesting count = 1 */

    HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */
    HwiP_post(LO_PRI_INT_NUM);  /* This should fire only after exiting from critical section */

    CHECK_UPDATE_ERR_CNT(++hiPriIsrCntExp, gHiPriIsrCnt, errCnt); /* Make sure the high priority ISR was fired */
    CHECK_UPDATE_ERR_CNT(  loPriIsrCntExp, gLoPriIsrCnt, errCnt); /* Make sure the low priority  ISR was not fired since in critical section */

    key2 = HWIP_ENTER_CRITICAL_SECTION();   /* Enter critical section in ISR again - Nesting count = 2 */

    HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */
    HwiP_post(LO_PRI_INT_NUM);  /* This won't have any effect since already pending  */

    CHECK_UPDATE_ERR_CNT(++hiPriIsrCntExp, gHiPriIsrCnt, errCnt); /* Make sure the high priority ISR was fired */
    CHECK_UPDATE_ERR_CNT(  loPriIsrCntExp, gLoPriIsrCnt, errCnt); /* Make sure the low priority  ISR was not fired since in critical section */

    HWIP_EXIT_CRITICAL_SECTION(key2);       /* Exit critical section in ISR - Nesting count = 1 */

    /* Low priority ISR shouldn't fire now since still its in a critical section with Nesting Count = 1 */

    HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */
    HwiP_post(LO_PRI_INT_NUM);  /* This won't have any effect since already pending  */

    CHECK_UPDATE_ERR_CNT(++hiPriIsrCntExp, gHiPriIsrCnt, errCnt); /* Make sure the high priority ISR was fired */
    CHECK_UPDATE_ERR_CNT(  loPriIsrCntExp, gLoPriIsrCnt, errCnt); /* Make sure the low priority  ISR was not fired since in critical section */

    HWIP_EXIT_CRITICAL_SECTION(key1);       /* Exit critical section in ISR - Nesting count = 0 */
    
    /* Low priority ISR should fire now since it exited from critical section */
    
    /* Now trigger again from outside of critical section */
    
    HwiP_post(HI_PRI_INT_NUM);  /* This should fire immediately resulting in switch to high priority ISR */
    HwiP_post(LO_PRI_INT_NUM);  /* This should fire immediately resulting in switch to low priority  ISR */

    return errCnt;
}

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

void test_criticalSectionFromMain(void *args)
{
    bool isPulse = (bool)args;

    DebugP_log("Testing with %s interrupts\r\n", isPulse ? "pulse" : "level");

    gHiPriIsrCnt = 0U;
    gLoPriIsrCnt = 0U;

    /* Register high priority interrupt, enabled during freertos critical section */
    test_register_isr(HI_PRI_INT_NUM, hi_pri_isr, HI_PRI_INT_PRI, isPulse, &gHiPriHwiObj, NULL);
    /* Register low priority interrupt, disabled during freertos critical section */
    test_register_isr(LO_PRI_INT_NUM, lo_pri_isr, LO_PRI_INT_PRI, isPulse, &gLoPriHwiObj, NULL);

    for(uint32_t loopCnt = 0U; loopCnt < TEST_LOOP_CNT; loopCnt++)
    {
        TEST_ASSERT_EQUAL(0U, test_critical_section(loopCnt));  /* Make sure no errors in execution flow */
        TEST_ASSERT_EQUAL((loopCnt + 1U) * 4U, gHiPriIsrCnt);   /* Make sure high priority ISR fired 4 times (3x - critical section, 1x - outside)*/
        TEST_ASSERT_EQUAL((loopCnt + 1U) * 2U, gLoPriIsrCnt);   /* Make sure low priority  ISR fired 2 times (1x - exit critical section, 1x - outside)*/
    }

    HwiP_destruct(&gHiPriHwiObj);
    HwiP_destruct(&gLoPriHwiObj);
}

#ifdef HWIP_NESTED_INTERRUPTS_IRQ_ENABLE
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

        TEST_ASSERT_EQUAL(loopCnt * 1U, gBasePriIsrCnt);       /* Make sure base priority ISR fired once */
        TEST_ASSERT_EQUAL(loopCnt * 4U, gHiPriIsrCnt);         /* Make sure high priority ISR fired 4 times (3x - critical section, 1x - outside)*/
        TEST_ASSERT_EQUAL(loopCnt * 2U, gLoPriIsrCnt);         /* Make sure low priority  ISR fired 2 times (1x - exit critical section, 1x - outside)*/
        TEST_ASSERT_EQUAL(0U,           gBasePriIsrErrorCnt);  /* Make sure no error in execution flow from base priority ISR */
    }

    HwiP_destruct(&gHiPriHwiObj);
    HwiP_destruct(&gLoPriHwiObj);
    HwiP_destruct(&gBasePriHwiObj);
}
#endif /* HWIP_NESTED_INTERRUPTS_IRQ_ENABLE */

void test_r5f_critical_section(void)
{
    bool pulse = true, level = false;

    RUN_TEST(test_criticalSectionFromMain, 14788, (void*)pulse);
    RUN_TEST(test_criticalSectionFromMain, 14789, (void*)level);
#ifdef HWIP_NESTED_INTERRUPTS_IRQ_ENABLE
    /* nested interrupts should be enabled to test critical sections from ISR */
    RUN_TEST(test_criticalSectionFromIsr, 14790, (void*)pulse);
    RUN_TEST(test_criticalSectionFromIsr, 14791, (void*)level);
#endif /* HWIP_NESTED_INTERRUPTS_IRQ_ENABLE */
}

#endif /* HWIP_USE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS */
/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <drivers/gtc.h>
#include <drivers/soc.h>
#include <drivers/hw_include/cslr.h>
#include <kernel/nortos/dpl/m4/SysTickTimerP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/nortos/dpl/common/ClockP_nortos_priv.h>
#include <kernel/dpl/AddrTranslateP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define SYST_BASE               (0xE000E010u)
#define SYST_RVR                (volatile uint32_t *)((SYST_BASE) + 0x04u)
#define SYST_MAX_COUNT_VALUE    (0xFFFFFFFFU)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Testcases */
static void test_gtc_output(void *args);
static void test_systick_output(void *args);
static uint64_t test_calc_time(uint32_t timerCount);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern ClockP_Control gClockCtrl;
extern ClockP_Config gClockConfig;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(test_gtc_output,     0, NULL);
    RUN_TEST(test_systick_output, 0, NULL);

    UNITY_END();
    Drivers_close();

    return;
}

/*
 * Unity framework required functions
 */
void setUp(void)
{
}

void tearDown(void)
{
}

/*
 * Testcases
 */
static void test_gtc_output(void *args)
{
    int32_t retVal = SystemP_SUCCESS;
    uint64_t gtcCount1 = 0, gtcCount2 = 0;
    uint64_t clkRate = 0;

    DebugP_log("\ntest_gtc_output started...\r\n");

    retVal = GTC_init();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_moduleGetClockFrequency(TISCI_DEV_GTC0, TISCI_DEV_GTC0_GTC_CLK, &clkRate);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    gtcCount1 = GTC_getCount64();
    ClockP_sleep(1);
    gtcCount2 = GTC_getCount64();

    TEST_ASSERT_UINT32_WITHIN(1000000, clkRate, (gtcCount2 - gtcCount1));

    DebugP_log("%lf time taken in seconds for test_gtc_output!!!\r\n",((Float64)((Float64)gtcCount2 - (Float64)gtcCount1) / (Float64)clkRate));
    DebugP_log("test_gtc_output end!!!\r\n");

    return;
}

static void test_systick_output(void *args)
{
    uint32_t sysTickCount1 = 0, sysTickCount2 = 0;

    DebugP_log("\ntest_systick_output started...\r\n");

    sysTickCount1 = (uint32_t)test_calc_time(ClockP_getTimerCount(gClockCtrl.timerBaseAddr));
    ClockP_sleep(1);
    sysTickCount2 = (uint32_t)test_calc_time(ClockP_getTimerCount(gClockCtrl.timerBaseAddr));

    TEST_ASSERT_UINT32_WITHIN(400000000, gClockConfig.timerInputClkHz, (sysTickCount2 - sysTickCount1));

    DebugP_log("%lf time taken in seconds for test_systick_output!!!\r\n",((Float64)((sysTickCount2 - sysTickCount1)) / (Float64)1000000));
    DebugP_log("test_systick_output end!!!\r\n");

    return;
}

static uint64_t test_calc_time(uint32_t timerCount)
{
    volatile uint32_t *addrRVR = SYST_RVR;

    /* Calculation needed as systick interrupt will be generated every 1000us/1ms and reload register is configured accordingly*/
    return ((uint64_t)((gClockCtrl.ticks * 1000)) + (uint64_t)((((timerCount - (*addrRVR)) * 1000) / (SYST_MAX_COUNT_VALUE - (*addrRVR)))));

}

/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void test_led_on_off(void *args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    int32_t status = SystemP_SUCCESS;

    UNITY_BEGIN();

    /* Open I2C and other drivers */
    Drivers_open();

    /* Open drivers */
    status = Board_driversOpen();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    RUN_TEST(test_led_on_off, 1964, NULL);

    UNITY_END();

    /* Close I2C and other drivers */
    Board_driversClose();
    Drivers_close();

    return;
}

/* Unity framework required information */
void setUp(void)
{
}

void tearDown(void)
{
}

/* Testcases */
static void test_led_on_off(void *args)
{
    int32_t             status;
    uint32_t            ledInst, loopcnt, ledCnt, numLedPerGroup, delayMsec = 100U;
    LED_Handle          ledHandle;
    const LED_Attrs    *attrs;

    for(ledInst = CONFIG_LED_GPIO; ledInst < (CONFIG_LED_GPIO + CONFIG_LED_NUM_INSTANCES); ledInst++)
    {
        loopcnt = 10U;

        ledHandle = gLedHandle[ledInst];
        TEST_ASSERT_NOT_NULL(ledHandle);
		attrs = LED_getAttrs(ledInst);
        TEST_ASSERT_NOT_NULL(attrs);
        numLedPerGroup = attrs->numLedPerGroup;
		
        /* ON all LED when more than one LED is present - only then mask API is supported */
        if(numLedPerGroup > 1U)
        {
            status = LED_setMask(ledHandle, 0xFFU);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
            ClockP_usleep(delayMsec * 1000U);
        }

        DebugP_log("LED Blink Test Started for LED Instance %d ...\r\n", ledInst);
        DebugP_log("LED will Blink for %d loop ...\r\n", loopcnt);
        while(loopcnt > 0U)
        {
            for(ledCnt = 0U; ledCnt < numLedPerGroup; ledCnt++)
            {
                status = LED_off(ledHandle, ledCnt);
                TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
                ClockP_usleep(delayMsec * 1000U);
            }

            for(ledCnt = 0U; ledCnt < numLedPerGroup; ledCnt++)
            {
                status = LED_on(ledHandle, ledCnt);
                TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
                ClockP_usleep(delayMsec * 1000U);
            }

            loopcnt--;
        }

        if(numLedPerGroup > 1U)
        {
            /* OFF all LED when more than one LED is present - only then mask API is supported */
            status = LED_setMask(ledHandle, 0x00U);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        }
        else
        {
            status = LED_off(ledHandle, 0U);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        }
        DebugP_log("LED Blink Test Passed for LED Instance %d!!\r\n", ledInst);
    }

    return;
}

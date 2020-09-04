/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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
#include <drivers/gpio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/** \brief GPIO test params */
typedef struct Test_GpioPrms
{
    uint32_t            baseAddr;
    uint32_t            pinNum;
    uint32_t            trigType;
    uint32_t            levelType;
    uint32_t            loopcnt;
    uint32_t            delayms;
    volatile uint32_t   intrcnt;
} Test_GpioPrms_t;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Testcases */
static void test_gpio_output(void *args);
static void test_gpio_trigger(void *args);
static void test_gpio_macros(void *args);

/* Helper functions */
static void test_gpio_toggle_loop(Test_GpioPrms_t *testPrms);
static void test_gpio_trigger_loop(Test_GpioPrms_t *testPrms);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(test_gpio_output,  1987, NULL);
    RUN_TEST(test_gpio_macros,  1988, NULL);
    RUN_TEST(test_gpio_trigger, 1989, NULL);

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
static void test_gpio_output(void *args)
{
    Test_GpioPrms_t testPrms;

    DebugP_log("\ntest_gpio_output started...\r\n");

    testPrms.baseAddr = GPIO_LED_BASE_ADDR;
    testPrms.baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(testPrms.baseAddr);
    testPrms.pinNum   = GPIO_LED_PIN;
    testPrms.loopcnt  = 5;
    testPrms.delayms  = 100;
    test_gpio_toggle_loop(&testPrms);

    DebugP_log("test_gpio_output end!!!\r\n");

    return;
}

static void test_gpio_trigger(void *args)
{
    Test_GpioPrms_t testPrms;

    DebugP_log("\ntest_gpio_trigger started...\r\n");

    testPrms.baseAddr = GPIO_PUSH_BUTTON_BASE_ADDR;
    testPrms.baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(testPrms.baseAddr);
    testPrms.pinNum   = GPIO_PUSH_BUTTON_PIN;
    testPrms.loopcnt  = 5;
    testPrms.delayms  = 100;
    testPrms.trigType = GPIO_TRIG_TYPE_RISE_EDGE;
    test_gpio_trigger_loop(&testPrms);
    testPrms.trigType = GPIO_TRIG_TYPE_FALL_EDGE;
    test_gpio_trigger_loop(&testPrms);
    testPrms.trigType = GPIO_TRIG_TYPE_BOTH_EDGE;
    test_gpio_trigger_loop(&testPrms);

    DebugP_log("test_gpio_trigger end!!!\r\n");

    return;
}

static void test_gpio_macros(void *args)
{
    uint32_t index, port, offset;

    DebugP_log("\ntest_gpio_macros started...\r\n");

    /* Macro checks */
    TEST_ASSERT_EQUAL_UINT32(8, GPIO_MAX_PORT);
    TEST_ASSERT_EQUAL_UINT32(4, GPIO_MAX_INTERRUPT_PORT);
    TEST_ASSERT_EQUAL_UINT32(8, GPIO_MAX_PINS_PER_PORT);
	TEST_ASSERT_EQUAL_UINT32(64, GPIO_MAX_PINS);

	 /* Pin Offset test */
    offset = GPIO_GET_PIN_OFFSET(0);
    TEST_ASSERT_EQUAL_UINT32(0, offset);
    offset = GPIO_GET_PIN_OFFSET(8);
    TEST_ASSERT_EQUAL_UINT32(8, offset);
    offset = GPIO_GET_PIN_OFFSET(15);
    TEST_ASSERT_EQUAL_UINT32(15, offset);
    offset = GPIO_GET_PIN_OFFSET(16);
    TEST_ASSERT_EQUAL_UINT32(16, offset);
    offset = GPIO_GET_PIN_OFFSET(24);
    TEST_ASSERT_EQUAL_UINT32(24, offset);
    offset = GPIO_GET_PIN_OFFSET(31);
    TEST_ASSERT_EQUAL_UINT32(31, offset);
    offset = GPIO_GET_PIN_OFFSET(32);
    TEST_ASSERT_EQUAL_UINT32(32, offset);
    offset = GPIO_GET_PIN_OFFSET(45);
    TEST_ASSERT_EQUAL_UINT32(45, offset);
    offset = GPIO_GET_PIN_OFFSET(63);
    TEST_ASSERT_EQUAL_UINT32(63, offset);
	
    /* Port Number test */
    port = GPIO_GET_PORT_NUM(0);
    TEST_ASSERT_EQUAL_UINT32(0, port);
    port = GPIO_GET_PORT_NUM(8);
    TEST_ASSERT_EQUAL_UINT32(1, port);
    port = GPIO_GET_PORT_NUM(15);
    TEST_ASSERT_EQUAL_UINT32(1, port);
    port = GPIO_GET_PORT_NUM(16);
    TEST_ASSERT_EQUAL_UINT32(2, port);
    port = GPIO_GET_PORT_NUM(24);
    TEST_ASSERT_EQUAL_UINT32(3, port);
    port = GPIO_GET_PORT_NUM(31);
    TEST_ASSERT_EQUAL_UINT32(3, port);
    port = GPIO_GET_PORT_NUM(32);
    TEST_ASSERT_EQUAL_UINT32(4, port);
	port = GPIO_GET_PORT_NUM(45);
    TEST_ASSERT_EQUAL_UINT32(5, port);
    port = GPIO_GET_PORT_NUM(63);
    TEST_ASSERT_EQUAL_UINT32(7, port);

    /* Pin index test */
    index = GPIO_GET_PIN_INDEX(0);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_PIN_INDEX(8);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_PIN_INDEX(15);
    TEST_ASSERT_EQUAL_UINT32(7, index);
    index = GPIO_GET_PIN_INDEX(16);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_PIN_INDEX(24);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_PIN_INDEX(31);
    TEST_ASSERT_EQUAL_UINT32(7, index);
    index = GPIO_GET_PIN_INDEX(32);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_PIN_INDEX(45);
    TEST_ASSERT_EQUAL_UINT32(5, index);
    index = GPIO_GET_PIN_INDEX(63);
    TEST_ASSERT_EQUAL_UINT32(7, index);
  
    DebugP_log("test_gpio_macros end!!!\r\n");

    return;
}

/*
 * Other functions
 */
static void test_gpio_toggle_loop(Test_GpioPrms_t *testPrms)
{
    uint32_t    pinValue, loopcnt;

    GPIO_setDirMode(testPrms->baseAddr, testPrms->pinNum, GPIO_DIRECTION_OUTPUT);
    loopcnt = 0;
    while(loopcnt < testPrms->loopcnt)
    {
        GPIO_pinWriteHigh(testPrms->baseAddr, testPrms->pinNum);
        /* Readback and check */
        pinValue = GPIO_pinRead(testPrms->baseAddr, testPrms->pinNum);
        TEST_ASSERT_EQUAL_UINT32(GPIO_PIN_HIGH, pinValue);

        ClockP_usleep(testPrms->delayms * 1000);

        GPIO_pinWriteLow(testPrms->baseAddr, testPrms->pinNum);
        /* Readback and check */
        pinValue = GPIO_pinRead(testPrms->baseAddr, testPrms->pinNum);
        TEST_ASSERT_EQUAL_UINT32(GPIO_PIN_LOW, pinValue);

        ClockP_usleep(testPrms->delayms * 1000);
        loopcnt++;
    }

    return;
}

static void test_gpio_trigger_loop(Test_GpioPrms_t *testPrms)
{
    uint32_t        loopcnt;
    uint32_t        pendingInterrupt =0;
    int32_t         status = SystemP_SUCCESS;

    testPrms->intrcnt = 0;
    GPIO_setDirMode(testPrms->baseAddr, testPrms->pinNum, GPIO_DIRECTION_OUTPUT);

    /* Set init value based on trigger required */
    if((GPIO_TRIG_TYPE_RISE_EDGE == testPrms->trigType) ||
       (GPIO_TRIG_TYPE_BOTH_EDGE == testPrms->trigType))
    {
        GPIO_pinWriteLow(testPrms->baseAddr, testPrms->pinNum);
        status = GPIO_setTrigType(testPrms->baseAddr, testPrms->pinNum, testPrms->trigType);
        if(status != SystemP_SUCCESS)
        {
             DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",testPrms->pinNum);
             DebugP_assert(status == SystemP_SUCCESS);
        }
    }
    else
    {
        GPIO_pinWriteHigh(testPrms->baseAddr, testPrms->pinNum);
    }
    ClockP_usleep(testPrms->delayms * 1000);

    /* Configure trigger */
    status = GPIO_ignoreOrHonorPolarity(testPrms->baseAddr, testPrms->pinNum, testPrms->trigType);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",testPrms->pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }

    status = GPIO_markHighLowLevelInterrupt(testPrms->baseAddr,  testPrms->pinNum, testPrms->levelType);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",testPrms->pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }

    status = GPIO_enableInterrupt(testPrms->baseAddr, testPrms->pinNum);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",testPrms->pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }

    loopcnt = 0;
    while(loopcnt < testPrms->loopcnt)
    {
        if(GPIO_TRIG_TYPE_BOTH_EDGE == testPrms->trigType)
        {
            /* Since both edge, trigger alternatively */
            if((loopcnt & 0x01) == 0)
            {
                GPIO_pinWriteHigh(testPrms->baseAddr, testPrms->pinNum);
                ClockP_usleep(testPrms->delayms * 1000);
            }
            else
            {
                GPIO_pinWriteLow(testPrms->baseAddr, testPrms->pinNum);
                ClockP_usleep(testPrms->delayms * 1000);
            }

            /* Check for interrupt status */
            pendingInterrupt = GPIO_getHighLowLevelPendingInterrupt(testPrms->baseAddr, testPrms->levelType);
            status =GPIO_clearInterrupt(testPrms->baseAddr, testPrms->pinNum);
            if(status != SystemP_SUCCESS)
            {
                 DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",testPrms->pinNum);
                 DebugP_assert(status == SystemP_SUCCESS);
            }

            if(pendingInterrupt)
            {
                testPrms->intrcnt++;
            }
        }

        if(GPIO_TRIG_TYPE_RISE_EDGE == testPrms->trigType)
        {
            GPIO_pinWriteHigh(testPrms->baseAddr, testPrms->pinNum);
            ClockP_usleep(testPrms->delayms * 1000);

            /* Check for interrupt status */
            pendingInterrupt = GPIO_getHighLowLevelPendingInterrupt(testPrms->baseAddr, testPrms->levelType);
            GPIO_clearInterrupt(testPrms->baseAddr, testPrms->pinNum);
            if(pendingInterrupt)
            {
                testPrms->intrcnt++;
            }

            GPIO_pinWriteLow(testPrms->baseAddr, testPrms->pinNum);
            ClockP_usleep(testPrms->delayms * 1000);

            /* Check for interrupt status - it should not occur */
            pendingInterrupt = GPIO_getHighLowLevelPendingInterrupt(testPrms->baseAddr, testPrms->levelType);
            TEST_ASSERT_EQUAL_UINT32(0, (pendingInterrupt));
        }

        if(GPIO_TRIG_TYPE_FALL_EDGE == testPrms->trigType)
        {
            GPIO_pinWriteLow(testPrms->baseAddr, testPrms->pinNum);
            ClockP_usleep(testPrms->delayms * 1000);

            /* Check for interrupt status */
            pendingInterrupt = GPIO_getHighLowLevelPendingInterrupt(testPrms->baseAddr, testPrms->levelType);
            GPIO_clearInterrupt(testPrms->baseAddr, testPrms->pinNum);
            if(pendingInterrupt)
            {
                testPrms->intrcnt++;
            }

            GPIO_pinWriteHigh(testPrms->baseAddr, testPrms->pinNum);
            ClockP_usleep(testPrms->delayms * 1000);

            /* Check for interrupt status - it should not occur */
            pendingInterrupt = GPIO_getHighLowLevelPendingInterrupt(testPrms->baseAddr, testPrms->levelType);
            TEST_ASSERT_EQUAL_UINT32(0, (pendingInterrupt));
        }

        loopcnt++;
    }

    TEST_ASSERT_EQUAL_INT32(testPrms->loopcnt, testPrms->intrcnt);

    /* Unregister interrupt */
    status = GPIO_disableInterrupt(testPrms->baseAddr, testPrms->pinNum);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",testPrms->pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }

    status = GPIO_setTrigType(testPrms->baseAddr, testPrms->pinNum, GPIO_TRIG_TYPE_NONE);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",testPrms->pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }

    status = GPIO_clearInterrupt(testPrms->baseAddr, testPrms->pinNum);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",testPrms->pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }

    /* Reset to default value */
    GPIO_pinWriteLow(testPrms->baseAddr, testPrms->pinNum);

    return;
}

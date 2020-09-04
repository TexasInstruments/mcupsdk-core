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
    uint32_t            loopcnt;
    uint32_t            delayms;
    volatile uint32_t   intrcnt;
} Test_GpioPrms_t;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Testcases */
static void test_gpio_output(void *args);
static void test_gpio_read(void *args);
static void test_gpio_trigger(void *args);
static void test_gpio_macros(void *args);

/* Helper functions */
static void test_gpio_toggle_loop(Test_GpioPrms_t *testPrms);
static void test_gpio_read_loop(Test_GpioPrms_t *testPrms);
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

    RUN_TEST(test_gpio_output,  166, NULL);
    RUN_TEST(test_gpio_read,    167, NULL);
    RUN_TEST(test_gpio_trigger, 168, NULL);
    RUN_TEST(test_gpio_macros,  169, NULL);

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

static void test_gpio_read(void *args)
{
    Test_GpioPrms_t testPrms;

    DebugP_log("\ntest_gpio_read started...\r\n");

    testPrms.baseAddr = GPIO_PUSH_BUTTON_BASE_ADDR;
    testPrms.baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(testPrms.baseAddr);
    testPrms.pinNum   = GPIO_PUSH_BUTTON_PIN;
    testPrms.loopcnt  = 5;
    testPrms.delayms  = 100;
    test_gpio_read_loop(&testPrms);

    DebugP_log("test_gpio_read end!!!\r\n");

    return;
}

static void test_gpio_trigger(void *args)
{
    Test_GpioPrms_t testPrms;

    DebugP_log("\ntest_gpio_trigger started...\r\n");

    testPrms.baseAddr = GPIO_LED_BASE_ADDR;
    testPrms.baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(testPrms.baseAddr);
    testPrms.pinNum   = GPIO_LED_PIN;
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
    uint32_t index, pos, mask;

    DebugP_log("\ntest_gpio_macros started...\r\n");

    /* Macro checks */
    TEST_ASSERT_EQUAL_UINT32(16, GPIO_MAX_PIN_PER_BANK);
    TEST_ASSERT_EQUAL_UINT32(2,  GPIO_BANKS_PER_REG);
    TEST_ASSERT_EQUAL_UINT32(32, GPIO_PINS_PER_REG);

    /* Bank index test */
    index = GPIO_GET_BANK_INDEX(0);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_BANK_INDEX(8);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_BANK_INDEX(15);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_BANK_INDEX(16);
    TEST_ASSERT_EQUAL_UINT32(1, index);
    index = GPIO_GET_BANK_INDEX(24);
    TEST_ASSERT_EQUAL_UINT32(1, index);
    index = GPIO_GET_BANK_INDEX(31);
    TEST_ASSERT_EQUAL_UINT32(1, index);
    index = GPIO_GET_BANK_INDEX(32);
    TEST_ASSERT_EQUAL_UINT32(2, index);
    index = GPIO_GET_BANK_INDEX(100);
    TEST_ASSERT_EQUAL_UINT32(6, index);

    /* Register index test */
    index = GPIO_GET_REG_INDEX(0);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_REG_INDEX(8);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_REG_INDEX(15);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_REG_INDEX(16);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_REG_INDEX(24);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_REG_INDEX(31);
    TEST_ASSERT_EQUAL_UINT32(0, index);
    index = GPIO_GET_REG_INDEX(32);
    TEST_ASSERT_EQUAL_UINT32(1, index);
    index = GPIO_GET_REG_INDEX(63);
    TEST_ASSERT_EQUAL_UINT32(1, index);
    index = GPIO_GET_REG_INDEX(64);
    TEST_ASSERT_EQUAL_UINT32(2, index);
    index = GPIO_GET_REG_INDEX(100);
    TEST_ASSERT_EQUAL_UINT32(3, index);

    /* Register bit position test */
    pos = GPIO_GET_BIT_POS(0);
    TEST_ASSERT_EQUAL_UINT32(0, pos);
    pos = GPIO_GET_BIT_POS(8);
    TEST_ASSERT_EQUAL_UINT32(8, pos);
    pos = GPIO_GET_BIT_POS(15);
    TEST_ASSERT_EQUAL_UINT32(15, pos);
    pos = GPIO_GET_BIT_POS(16);
    TEST_ASSERT_EQUAL_UINT32(16, pos);
    pos = GPIO_GET_BIT_POS(24);
    TEST_ASSERT_EQUAL_UINT32(24, pos);
    pos = GPIO_GET_BIT_POS(31);
    TEST_ASSERT_EQUAL_UINT32(31, pos);
    pos = GPIO_GET_BIT_POS(32);
    TEST_ASSERT_EQUAL_UINT32(0, pos);
    pos = GPIO_GET_BIT_POS(63);
    TEST_ASSERT_EQUAL_UINT32(31, pos);
    pos = GPIO_GET_BIT_POS(64);
    TEST_ASSERT_EQUAL_UINT32(0, pos);
    pos = GPIO_GET_BIT_POS(100);
    TEST_ASSERT_EQUAL_UINT32(4, pos);

    /* Bank bit position test */
    pos = GPIO_GET_BANK_BIT_POS(0);
    TEST_ASSERT_EQUAL_UINT32(0, pos);
    pos = GPIO_GET_BANK_BIT_POS(8);
    TEST_ASSERT_EQUAL_UINT32(8, pos);
    pos = GPIO_GET_BANK_BIT_POS(15);
    TEST_ASSERT_EQUAL_UINT32(15, pos);
    pos = GPIO_GET_BANK_BIT_POS(16);
    TEST_ASSERT_EQUAL_UINT32(0, pos);
    pos = GPIO_GET_BANK_BIT_POS(24);
    TEST_ASSERT_EQUAL_UINT32(8, pos);
    pos = GPIO_GET_BANK_BIT_POS(31);
    TEST_ASSERT_EQUAL_UINT32(15, pos);
    pos = GPIO_GET_BANK_BIT_POS(32);
    TEST_ASSERT_EQUAL_UINT32(0, pos);
    pos = GPIO_GET_BANK_BIT_POS(63);
    TEST_ASSERT_EQUAL_UINT32(15, pos);
    pos = GPIO_GET_BANK_BIT_POS(64);
    TEST_ASSERT_EQUAL_UINT32(0, pos);
    pos = GPIO_GET_BANK_BIT_POS(100);
    TEST_ASSERT_EQUAL_UINT32(4, pos);

    /* Register bit mask test */
    mask = GPIO_GET_BIT_MASK(0);
    TEST_ASSERT_EQUAL_UINT32(0x00000001, mask);
    mask = GPIO_GET_BIT_MASK(8);
    TEST_ASSERT_EQUAL_UINT32(0x00000100, mask);
    mask = GPIO_GET_BIT_MASK(15);
    TEST_ASSERT_EQUAL_UINT32(0x00008000, mask);
    mask = GPIO_GET_BIT_MASK(16);
    TEST_ASSERT_EQUAL_UINT32(0x00010000, mask);
    mask = GPIO_GET_BIT_MASK(24);
    TEST_ASSERT_EQUAL_UINT32(0x01000000, mask);
    mask = GPIO_GET_BIT_MASK(31);
    TEST_ASSERT_EQUAL_UINT32(0x80000000, mask);
    mask = GPIO_GET_BIT_MASK(32);
    TEST_ASSERT_EQUAL_UINT32(0x00000001, mask);
    mask = GPIO_GET_BIT_MASK(63);
    TEST_ASSERT_EQUAL_UINT32(0x080000000, mask);
    mask = GPIO_GET_BIT_MASK(64);
    TEST_ASSERT_EQUAL_UINT32(0x00000001, mask);
    mask = GPIO_GET_BIT_MASK(100);
    TEST_ASSERT_EQUAL_UINT32(0x00000010, mask);

    /* Bank bit mask test */
    mask = GPIO_GET_BANK_BIT_MASK(0);
    TEST_ASSERT_EQUAL_UINT32(0x0001, mask);
    mask = GPIO_GET_BANK_BIT_MASK(8);
    TEST_ASSERT_EQUAL_UINT32(0x0100, mask);
    mask = GPIO_GET_BANK_BIT_MASK(15);
    TEST_ASSERT_EQUAL_UINT32(0x8000, mask);
    mask = GPIO_GET_BANK_BIT_MASK(16);
    TEST_ASSERT_EQUAL_UINT32(0x0001, mask);
    mask = GPIO_GET_BANK_BIT_MASK(24);
    TEST_ASSERT_EQUAL_UINT32(0x0100, mask);
    mask = GPIO_GET_BANK_BIT_MASK(31);
    TEST_ASSERT_EQUAL_UINT32(0x8000, mask);
    mask = GPIO_GET_BANK_BIT_MASK(32);
    TEST_ASSERT_EQUAL_UINT32(0x0001, mask);
    mask = GPIO_GET_BANK_BIT_MASK(63);
    TEST_ASSERT_EQUAL_UINT32(0x8000, mask);
    mask = GPIO_GET_BANK_BIT_MASK(64);
    TEST_ASSERT_EQUAL_UINT32(0x0001, mask);
    mask = GPIO_GET_BANK_BIT_MASK(100);
    TEST_ASSERT_EQUAL_UINT32(0x0010, mask);

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
        pinValue = GPIO_pinOutValueRead(testPrms->baseAddr, testPrms->pinNum);
        TEST_ASSERT_EQUAL_UINT32(GPIO_PIN_HIGH, pinValue);

        ClockP_usleep(testPrms->delayms * 1000);

        GPIO_pinWriteLow(testPrms->baseAddr, testPrms->pinNum);
        /* Readback and check */
        pinValue = GPIO_pinOutValueRead(testPrms->baseAddr, testPrms->pinNum);
        TEST_ASSERT_EQUAL_UINT32(GPIO_PIN_LOW, pinValue);

        ClockP_usleep(testPrms->delayms * 1000);
        loopcnt++;
    }

    return;
}

static void test_gpio_read_loop(Test_GpioPrms_t *testPrms)
{
    uint32_t        pinValue, loopcnt;

    GPIO_setDirMode(testPrms->baseAddr, testPrms->pinNum, GPIO_DIRECTION_INPUT);
    loopcnt = 0;
    while(loopcnt < testPrms->loopcnt)
    {
        pinValue = GPIO_pinRead(testPrms->baseAddr, testPrms->pinNum);
        DebugP_log("Current GPIO pin value: %d\r\n", pinValue);
        ClockP_usleep(testPrms->delayms * 1000);
        loopcnt++;
    }

    return;
}

static void test_gpio_trigger_loop(Test_GpioPrms_t *testPrms)
{
    uint32_t        bankNum, loopcnt;
    uint32_t        intrStatus, pinMask;

    testPrms->intrcnt = 0;
    bankNum = GPIO_GET_BANK_INDEX(testPrms->pinNum);
    pinMask = GPIO_GET_BANK_BIT_MASK(testPrms->pinNum);
    GPIO_setDirMode(testPrms->baseAddr, testPrms->pinNum, GPIO_DIRECTION_OUTPUT);

    /* Set init value based on trigger required */
    if((GPIO_TRIG_TYPE_RISE_EDGE == testPrms->trigType) ||
       (GPIO_TRIG_TYPE_BOTH_EDGE == testPrms->trigType))
    {
        GPIO_pinWriteLow(testPrms->baseAddr, testPrms->pinNum);
    }
    else
    {
        GPIO_pinWriteHigh(testPrms->baseAddr, testPrms->pinNum);
    }
    ClockP_usleep(testPrms->delayms * 1000);

    /* Configure trigger */
    GPIO_setTrigType(testPrms->baseAddr, testPrms->pinNum, testPrms->trigType);
    GPIO_bankIntrEnable(testPrms->baseAddr, bankNum);

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
            intrStatus = GPIO_getBankIntrStatus(testPrms->baseAddr, bankNum);
            GPIO_clearBankIntrStatus(testPrms->baseAddr, bankNum, intrStatus);
            if(intrStatus & pinMask)
            {
                testPrms->intrcnt++;
            }
        }

        if(GPIO_TRIG_TYPE_RISE_EDGE == testPrms->trigType)
        {
            GPIO_pinWriteHigh(testPrms->baseAddr, testPrms->pinNum);
            ClockP_usleep(testPrms->delayms * 1000);

            /* Check for interrupt status */
            intrStatus = GPIO_getBankIntrStatus(testPrms->baseAddr, bankNum);
            GPIO_clearBankIntrStatus(testPrms->baseAddr, bankNum, intrStatus);
            if(intrStatus & pinMask)
            {
                testPrms->intrcnt++;
            }

            GPIO_pinWriteLow(testPrms->baseAddr, testPrms->pinNum);
            ClockP_usleep(testPrms->delayms * 1000);

            /* Check for interrupt status - it should not occur */
            intrStatus = GPIO_getBankIntrStatus(testPrms->baseAddr, bankNum);
            TEST_ASSERT_EQUAL_UINT32(0, (intrStatus & pinMask));
        }

        if(GPIO_TRIG_TYPE_FALL_EDGE == testPrms->trigType)
        {
            GPIO_pinWriteLow(testPrms->baseAddr, testPrms->pinNum);
            ClockP_usleep(testPrms->delayms * 1000);

            /* Check for interrupt status */
            intrStatus = GPIO_getBankIntrStatus(testPrms->baseAddr, bankNum);
            GPIO_clearBankIntrStatus(testPrms->baseAddr, bankNum, intrStatus);
            if(intrStatus & pinMask)
            {
                testPrms->intrcnt++;
            }

            GPIO_pinWriteHigh(testPrms->baseAddr, testPrms->pinNum);
            ClockP_usleep(testPrms->delayms * 1000);

            /* Check for interrupt status - it should not occur */
            intrStatus = GPIO_getBankIntrStatus(testPrms->baseAddr, bankNum);
            TEST_ASSERT_EQUAL_UINT32(0, (intrStatus & pinMask));
        }

        loopcnt++;
    }

    TEST_ASSERT_EQUAL_INT32(testPrms->loopcnt, testPrms->intrcnt);

    /* Unregister interrupt */
    GPIO_bankIntrDisable(testPrms->baseAddr, bankNum);
    GPIO_setTrigType(testPrms->baseAddr, testPrms->pinNum, GPIO_TRIG_TYPE_NONE);
    GPIO_clearIntrStatus(testPrms->baseAddr, testPrms->pinNum);

    /* Reset to default value */
    GPIO_pinWriteLow(testPrms->baseAddr, testPrms->pinNum);

    return;
}

/*
 * Copyright (C) 2024 Texas Instruments Incorporated
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
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include "ti_drivers_open_close.h"
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct MMCSD_TestParams_s {

    bool                    dmaEnable;
    uint32_t                busWidth;
    uint32_t                tuningType;
    uint32_t                uaBusSpeed;
    bool                    callbackMode;

} MMCSD_TestParams;

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TEST_MMCSD_EMMC_START_BLK        (0x300000U) /* 1.5GB */
#define TEST_MMCSD_DATA_SIZE             (1024U * 1) /* has to be 256 B aligned */
#define TEST_MMCSD_FILE_LINE_CNT         (100U)
#define TEST_MMCSD_FAT_PARTITION_SIZE    (128U * 1024U * 1024U) /* 128 MB */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Test Cases */
static void test_no_DMA_Interrupt_HS200_Blocking(void *args);
static void test_no_DMA_Interrupt_SDR50_Blocking(void *args);
static void test_no_DMA_Interrupt_SDR25_Blocking(void *args);

static void test_DMA_Interrupt_HS200_Blocking(void *args);
static void test_DMA_Interrupt_SDR50_Blocking(void *args);
static void test_DMA_Interrupt_SDR25_Blocking(void *args);

static void test_no_DMA_Interrupt_HS200_Callback(void *args);
static void test_no_DMA_Interrupt_SDR50_Callback(void *args);
static void test_no_DMA_Interrupt_SDR25_Callback(void *args);

static void test_DMA_Interrupt_HS200_Callback(void *args);
static void test_DMA_Interrupt_SDR50_Callback(void *args);
static void test_DMA_Interrupt_SDR25_Callback(void *args);

/* Test Case Sub Functions */
static void test_no_DMA_Interrupt_HS200_Auto_8Bit(void *args);
static void test_no_DMA_Interrupt_HS200_Auto_4Bit(void *args);
static void test_no_DMA_Interrupt_HS200_Manual_8Bit(void *args);
static void test_no_DMA_Interrupt_HS200_Manual_4Bit(void *args);

static void test_no_DMA_Interrupt_SDR50_NA_8Bit(void *args);
static void test_no_DMA_Interrupt_SDR50_NA_4Bit(void *args);
static void test_no_DMA_Interrupt_SDR50_NA_1Bit(void *args);

static void test_no_DMA_Interrupt_SDR25_NA_8Bit(void *args);
static void test_no_DMA_Interrupt_SDR25_NA_4Bit(void *args);
static void test_no_DMA_Interrupt_SDR25_NA_1Bit(void *args);

static void test_DMA_Interrupt_HS200_Auto_8Bit(void *args);
static void test_DMA_Interrupt_HS200_Auto_4Bit(void *args);
static void test_DMA_Interrupt_HS200_Manual_8Bit(void *args);
static void test_DMA_Interrupt_HS200_Manual_4Bit(void *args);

static void test_DMA_Interrupt_SDR50_NA_8Bit(void *args);
static void test_DMA_Interrupt_SDR50_NA_4Bit(void *args);
static void test_DMA_Interrupt_SDR50_NA_1Bit(void *args);

static void test_DMA_Interrupt_SDR25_NA_8Bit(void *args);
static void test_DMA_Interrupt_SDR25_NA_4Bit(void *args);
static void test_DMA_Interrupt_SDR25_NA_1Bit(void *args);

/* Helper Functions */
static void test_mmcsd_fill_buffers(void);
static void test_mmcsd_set_test_params(MMCSD_TestParams *testParams,
                                       uint32_t setting_id);
static int32_t test_mmcsd_raw_io(MMCSD_Handle handle,
                                 MMCSD_TestParams* tParams);
static void test_mmcsd_enable_callback(MMCSD_Handle handle,
                                       MMCSD_txnCallbackFxn callbackFxn,
                                       MMCSD_TestParams *testParams);
static void test_mmcsd_disable_callback(MMCSD_Handle handle,
                                       MMCSD_txnCallbackFxn callbackFxn,
                                       MMCSD_TestParams *testParams);

void MMCSD_enableDma(MMCSD_Handle handle);
void MMCSD_disableDma(MMCSD_Handle handle);

static void test_mmcsd_callback(MMCSD_Handle mmcsdHnd, int32_t transferStatus);

static void wait_transfer_complete(void);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t gMmcsdTestTxBuf[TEST_MMCSD_DATA_SIZE] __attribute__((aligned(128U)));
uint8_t gMmcsdTestRxBuf[TEST_MMCSD_DATA_SIZE] __attribute__((aligned(128U)));

SemaphoreP_Object       xferComplete;

MMCSD_TestParams        testParams;
uint32_t                flag = 0U;

extern MMCSD_Config gMmcsdConfig[];
extern MMCSD_Params gMmcsdParams[];
extern MMCSD_Handle gMmcsdHandle[];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();

    UNITY_BEGIN();

    /* Run Test Cases */

    RUN_TEST(test_no_DMA_Interrupt_HS200_Blocking, 1234, (void *)&testParams);
    RUN_TEST(test_no_DMA_Interrupt_SDR50_Blocking, 1234, (void *)&testParams);
    RUN_TEST(test_no_DMA_Interrupt_SDR25_Blocking, 1234, (void *)&testParams);

    RUN_TEST(test_DMA_Interrupt_HS200_Blocking, 1234, (void *)&testParams);
    RUN_TEST(test_DMA_Interrupt_SDR50_Blocking, 1234, (void *)&testParams);
    RUN_TEST(test_DMA_Interrupt_SDR25_Blocking, 1234, (void *)&testParams);

    RUN_TEST(test_no_DMA_Interrupt_HS200_Callback, 1234, (void *)&testParams);
    RUN_TEST(test_no_DMA_Interrupt_SDR50_Callback, 1234, (void *)&testParams);
    RUN_TEST(test_no_DMA_Interrupt_SDR25_Callback, 1234, (void *)&testParams);

    RUN_TEST(test_DMA_Interrupt_HS200_Callback, 1234, (void *)&testParams);
    RUN_TEST(test_DMA_Interrupt_SDR50_Callback, 1234, (void *)&testParams);
    RUN_TEST(test_DMA_Interrupt_SDR25_Callback, 1234, (void *)&testParams);

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

static void test_no_DMA_Interrupt_HS200_Blocking(void *args)
{
    test_no_DMA_Interrupt_HS200_Auto_8Bit(args);
    test_no_DMA_Interrupt_HS200_Auto_4Bit(args);
    test_no_DMA_Interrupt_HS200_Manual_8Bit(args);
    test_no_DMA_Interrupt_HS200_Manual_4Bit(args);
}

static void test_no_DMA_Interrupt_SDR50_Blocking(void *args)
{
    test_no_DMA_Interrupt_SDR50_NA_8Bit(args);
    test_no_DMA_Interrupt_SDR50_NA_4Bit(args);
    test_no_DMA_Interrupt_SDR50_NA_1Bit(args);
}

static void test_no_DMA_Interrupt_SDR25_Blocking(void *args)
{
    test_no_DMA_Interrupt_SDR25_NA_8Bit(args);
    test_no_DMA_Interrupt_SDR25_NA_4Bit(args);
    test_no_DMA_Interrupt_SDR25_NA_1Bit(args);
}

static void test_DMA_Interrupt_HS200_Blocking(void *args)
{
    test_DMA_Interrupt_HS200_Auto_8Bit(args);
    test_DMA_Interrupt_HS200_Auto_4Bit(args);
    test_DMA_Interrupt_HS200_Manual_8Bit(args);
    test_DMA_Interrupt_HS200_Manual_4Bit(args);
}

static void test_DMA_Interrupt_SDR50_Blocking(void *args)
{
    test_DMA_Interrupt_SDR50_NA_8Bit(args);
    test_DMA_Interrupt_SDR50_NA_4Bit(args);
    test_DMA_Interrupt_SDR50_NA_1Bit(args);
}

static void test_DMA_Interrupt_SDR25_Blocking(void *args)
{
    test_DMA_Interrupt_SDR25_NA_8Bit(args);
    test_DMA_Interrupt_SDR25_NA_4Bit(args);
    test_DMA_Interrupt_SDR25_NA_1Bit(args);
}

static void test_no_DMA_Interrupt_HS200_Callback(void *args)
{
    test_mmcsd_enable_callback(gMmcsdHandle[CONFIG_MMCSD0],
                               &test_mmcsd_callback, &testParams);

    test_no_DMA_Interrupt_HS200_Auto_8Bit(args);
    test_no_DMA_Interrupt_HS200_Auto_4Bit(args);
    test_no_DMA_Interrupt_HS200_Manual_8Bit(args);
    test_no_DMA_Interrupt_HS200_Manual_4Bit(args);

    test_mmcsd_disable_callback(gMmcsdHandle[CONFIG_MMCSD0],
                                &test_mmcsd_callback, &testParams);
}

static void test_no_DMA_Interrupt_SDR50_Callback(void *args)
{
    test_mmcsd_enable_callback(gMmcsdHandle[CONFIG_MMCSD0],
                               &test_mmcsd_callback, &testParams);

    test_no_DMA_Interrupt_SDR50_NA_8Bit(args);
    test_no_DMA_Interrupt_SDR50_NA_4Bit(args);
    test_no_DMA_Interrupt_SDR50_NA_1Bit(args);

    test_mmcsd_disable_callback(gMmcsdHandle[CONFIG_MMCSD0],
                                &test_mmcsd_callback, &testParams);
}

static void test_no_DMA_Interrupt_SDR25_Callback(void *args)
{
    test_mmcsd_enable_callback(gMmcsdHandle[CONFIG_MMCSD0],
                               &test_mmcsd_callback, &testParams);

    test_no_DMA_Interrupt_SDR25_NA_8Bit(args);
    test_no_DMA_Interrupt_SDR25_NA_4Bit(args);
    test_no_DMA_Interrupt_SDR25_NA_1Bit(args);

    test_mmcsd_disable_callback(gMmcsdHandle[CONFIG_MMCSD0],
                                &test_mmcsd_callback, &testParams);
}

static void test_DMA_Interrupt_HS200_Callback(void *args)
{
    test_mmcsd_enable_callback(gMmcsdHandle[CONFIG_MMCSD0],
                               &test_mmcsd_callback, &testParams);

    test_DMA_Interrupt_HS200_Auto_8Bit(args);
    test_DMA_Interrupt_HS200_Auto_4Bit(args);
    test_DMA_Interrupt_HS200_Manual_8Bit(args);
    test_DMA_Interrupt_HS200_Manual_4Bit(args);

    test_mmcsd_disable_callback(gMmcsdHandle[CONFIG_MMCSD0],
                                &test_mmcsd_callback, &testParams);
}

static void test_DMA_Interrupt_SDR50_Callback(void *args)
{
    test_mmcsd_enable_callback(gMmcsdHandle[CONFIG_MMCSD0],
                               &test_mmcsd_callback, &testParams);

    test_DMA_Interrupt_SDR50_NA_8Bit(args);
    test_DMA_Interrupt_SDR50_NA_4Bit(args);
    test_DMA_Interrupt_SDR50_NA_1Bit(args);

    test_mmcsd_disable_callback(gMmcsdHandle[CONFIG_MMCSD0],
                                &test_mmcsd_callback, &testParams);
}

static void test_DMA_Interrupt_SDR25_Callback(void *args)
{
    test_mmcsd_enable_callback(gMmcsdHandle[CONFIG_MMCSD0],
                               &test_mmcsd_callback, &testParams);

    test_DMA_Interrupt_SDR25_NA_8Bit(args);
    test_DMA_Interrupt_SDR25_NA_4Bit(args);
    test_DMA_Interrupt_SDR25_NA_1Bit(args);

    test_mmcsd_disable_callback(gMmcsdHandle[CONFIG_MMCSD0],
                                &test_mmcsd_callback, &testParams);
}


/* Test Case Sub Functions */
static void test_no_DMA_Interrupt_HS200_Auto_8Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 1U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_no_DMA_Interrupt_HS200_Auto_4Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 2U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_no_DMA_Interrupt_HS200_Manual_8Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 3U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_no_DMA_Interrupt_HS200_Manual_4Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 4U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_no_DMA_Interrupt_SDR50_NA_8Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 5U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_no_DMA_Interrupt_SDR50_NA_4Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 6U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_no_DMA_Interrupt_SDR50_NA_1Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 7U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_no_DMA_Interrupt_SDR25_NA_8Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 8U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_no_DMA_Interrupt_SDR25_NA_4Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 9U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_no_DMA_Interrupt_SDR25_NA_1Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 10U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_DMA_Interrupt_HS200_Auto_8Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 11U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_DMA_Interrupt_HS200_Auto_4Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams,12U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_DMA_Interrupt_HS200_Manual_8Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 13U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_DMA_Interrupt_HS200_Manual_4Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 14U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_DMA_Interrupt_SDR50_NA_8Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 15U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_DMA_Interrupt_SDR50_NA_4Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 16U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_DMA_Interrupt_SDR50_NA_1Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 17U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_DMA_Interrupt_SDR25_NA_8Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 18U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_DMA_Interrupt_SDR25_NA_4Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 19U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_DMA_Interrupt_SDR25_NA_1Bit(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_TestParams *testParams = (MMCSD_TestParams*)args;
    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD0];

    test_mmcsd_set_test_params(testParams, 20U);
    retVal = test_mmcsd_raw_io(handle, testParams);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

/* Helper Functions */

static void test_mmcsd_fill_buffers(void)
{
    uint32_t i;

    for(i = 0U; i < TEST_MMCSD_DATA_SIZE; i++)
    {
        gMmcsdTestTxBuf[i] = i % 256;
        gMmcsdTestRxBuf[i] = 0U;
    }
}

static void test_mmcsd_set_test_params(MMCSD_TestParams *testParams,
                                       uint32_t setting_id)
{
    switch(setting_id)
    {
        case 1:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_8BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_HS200;
            testParams->dmaEnable = false;
        }
        break;

        case 2:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_4BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_HS200;
            testParams->dmaEnable = false;
        }
        break;

        case 3:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_8BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_MANUAL;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_HS200;
            testParams->dmaEnable = false;
        }
        break;

        case 4:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_4BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_MANUAL;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_HS200;
            testParams->dmaEnable = false;
        }
        break;

        case 5:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_8BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_SDR50;
            testParams->dmaEnable = false;
        }
        break;

        case 6:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_4BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_SDR50;
            testParams->dmaEnable = false;
        }
        break;

        case 7:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_1BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_SDR50;
            testParams->dmaEnable = false;
        }
        break;

        case 8:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_8BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_SDR25;
            testParams->dmaEnable = false;
        }
        break;

        case 9:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_4BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_SDR25;
            testParams->dmaEnable = false;
        }
        break;

        case 10:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_1BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_SDR25;
            testParams->dmaEnable = false;
        }
        break;

        case 11:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_8BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_HS200;
            testParams->dmaEnable = true;
        }
        break;

        case 12:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_4BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_HS200;
            testParams->dmaEnable = true;
        }
        break;

        case 13:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_8BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_MANUAL;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_HS200;
            testParams->dmaEnable = true;
        }
        break;

        case 14:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_4BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_MANUAL;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_HS200;
            testParams->dmaEnable = true;
        }
        break;

        case 15:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_8BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_SDR50;
            testParams->dmaEnable = true;
        }
        break;

        case 16:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_4BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_SDR50;
            testParams->dmaEnable = true;
        }
        break;

        case 17:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_1BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_SDR50;
            testParams->dmaEnable = true;
        }
        break;

        case 18:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_8BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_SDR25;
            testParams->dmaEnable = true;
        }
        break;

        case 19:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_4BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_SDR25;
            testParams->dmaEnable = true;
        }
        break;

        case 20:
        {
            testParams->busWidth = MMCSD_BUS_WIDTH_1BIT;
            testParams->tuningType = MMCSD_PHY_TUNING_TYPE_AUTO;
            testParams->uaBusSpeed = MMCSD_MMC_MODE_SDR25;
            testParams->dmaEnable = true;
        }
        break;

        default:
        break;
    }
}

void MMCSD_enableDma(MMCSD_Handle handle)
{
    MMCSD_Config *config = (MMCSD_Config *)handle;
    MMCSD_Object *object = (MMCSD_Object *)config->object;
    object->mmcsdLldInitObject.enableDma = true;
}

void MMCSD_disableDma(MMCSD_Handle handle)
{
    MMCSD_Config *config = (MMCSD_Config *)handle;
    MMCSD_Object *object = (MMCSD_Object *)config->object;
    object->mmcsdLldInitObject.enableDma = false;
}

static int32_t test_mmcsd_raw_io(MMCSD_Handle handle, MMCSD_TestParams* tParams)
{
    int32_t status = SystemP_SUCCESS;

    uint32_t blockSize = MMCSD_getBlockSize(handle);
    uint32_t numBlocks = TEST_MMCSD_DATA_SIZE / blockSize;

    MMCSD_change_Tuning_Type(handle, tParams->tuningType);
    MMCSD_change_Bus_Config(handle, tParams->uaBusSpeed, tParams->busWidth);
    if(tParams->dmaEnable)
    {
        MMCSD_enableDma(handle);
    }
    else
    {
        MMCSD_disableDma(handle);
    }

    test_mmcsd_fill_buffers();

    /* Write */
    if(tParams->callbackMode)
    {
        status = MMCSD_write(handle, gMmcsdTestTxBuf,
                         TEST_MMCSD_EMMC_START_BLK, numBlocks);
        wait_transfer_complete();

    }
    else
    {
        status = MMCSD_write(handle, gMmcsdTestTxBuf,
                         TEST_MMCSD_EMMC_START_BLK, numBlocks);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Read */
        if(tParams->callbackMode)
        {
            status = MMCSD_read(handle, gMmcsdTestRxBuf,
                            TEST_MMCSD_EMMC_START_BLK, numBlocks);
            wait_transfer_complete();
        }
        else
        {
            status = MMCSD_read(handle, gMmcsdTestRxBuf,
                            TEST_MMCSD_EMMC_START_BLK, numBlocks);
        }
    }
    if(status == SystemP_SUCCESS)
    {
        /* Compare */
        status = memcmp(gMmcsdTestRxBuf, gMmcsdTestTxBuf, TEST_MMCSD_DATA_SIZE);
    }

    return status;
}

static void test_mmcsd_enable_callback(MMCSD_Handle handle,
                                       MMCSD_txnCallbackFxn callbackFxn,
                                       MMCSD_TestParams *testParams)
{
    MMCSD_Config *config = (MMCSD_Config *)handle;
    MMCSD_Object *object = (MMCSD_Object *)config->object;
    object->transferMode = MMCSD_MODE_CALLBACK;
    object->txnCallbackFxn = callbackFxn;
    testParams->callbackMode = true;
    SemaphoreP_constructBinary(&xferComplete, 0U);
}

static void test_mmcsd_disable_callback(MMCSD_Handle handle,
                                       MMCSD_txnCallbackFxn callbackFxn,
                                       MMCSD_TestParams *testParams)
{
    MMCSD_Config *config = (MMCSD_Config *)handle;
    MMCSD_Object *object = (MMCSD_Object *)config->object;
    object->transferMode = MMCSD_MODE_BLOCKING;
    object->txnCallbackFxn = NULL;
    testParams->callbackMode = false;
    SemaphoreP_destruct(&xferComplete);
}

static void test_mmcsd_callback(MMCSD_Handle mmcsdHnd, int32_t transferStatus)
{
    if(mmcsdHnd != NULL)
    {
        SemaphoreP_post(&xferComplete);
    }
}

static void wait_transfer_complete(void)
{
    SemaphoreP_pend(&xferComplete, SystemP_WAIT_FOREVER);
}
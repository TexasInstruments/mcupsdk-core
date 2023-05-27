/* Copyright (c) 2023 Texas Instruments Incorporated
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
 *
 */

 /**
 *  \file     HwiP_c66_test_neg.c
 *
 *  \brief    This file contains Nortos R5 API unit test code.
 *
 *  \details  nortos unit tests
 **/

#include <kernel/nortos/dpl/c66/HwiP_c66.h>
#include <drivers/hw_include/csl_types.h>
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
#include <drivers/soc.h>
#include <unity.h>
#include "ti_drivers_open_close.h"

int32_t HwiP_registerNmiHandler(HwiP_FxnCallback nmiHandler, void *args);
int32_t HwiP_unregisterNmiHandler(void);

	HwiP_Object handle;
	HwiP_Params params;
	uint32_t intNum = 0U;
	uint32_t oldIntState = 0U;

/* Negative test of HwiP_init API */
void HwiP_init_negTest(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
		HwiP_init();
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test of HwiP_disable API */
void HwiP_disable_negTest(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
		if (HwiP_disable() == NULL)
		{
			testStatus = SystemP_SUCCESS;
		}
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
		DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	 TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test of HwiP_enable API */
void HwiP_enable_negTest(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
		HwiP_enable();
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
     TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test of HwiP_restore API */
void HwiP_restore_negTest(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
		HwiP_restore(NULL);
    }

	if (testStatus != SystemP_SUCCESS)
    {

        testStatus = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test of HwiP_Params_init API */
void HwiP_Params_init_negTest(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
		HwiP_Params_init(NULL);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
		DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test of HwiP_registerNmiHandler API */
void HwiP_HwiP_registerNmiHandler_negTest(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;
    HwiP_FxnCallback nmiHandler = NULL;

    if (testStatus == SystemP_SUCCESS)
    {
		HwiP_registerNmiHandler(nmiHandler, args);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
		DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test of HwiP_registerNmiHandler API */
void HwiP_HwiP_registerNmiHandler_negTestOne(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;
    HwiP_FxnCallback nmiHandler = NULL;

    if (testStatus == SystemP_SUCCESS)
    {
        gHwiCtrl.nmiHandler = nmiHandler;
		HwiP_registerNmiHandler(nmiHandler, args);
        HwiP_unregisterNmiHandler();
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
		DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_neg_main(void *args)
{
    RUN_TEST(HwiP_init_negTest,9994,NULL);
	RUN_TEST(HwiP_disable_negTest, 9995, NULL);
	RUN_TEST(HwiP_enable_negTest, 9996, NULL);
	RUN_TEST(HwiP_restore_negTest, 9997, NULL);
	RUN_TEST(HwiP_Params_init_negTest, 9998, NULL);
    RUN_TEST(HwiP_HwiP_registerNmiHandler_negTest, 10955, NULL);
    RUN_TEST(HwiP_HwiP_registerNmiHandler_negTestOne, 10956, NULL);
}

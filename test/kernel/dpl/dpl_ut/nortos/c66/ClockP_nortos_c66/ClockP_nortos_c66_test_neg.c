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
 *  \file     nortos_c66_test_neg.c
 *
 *  \brief    This file contains Nortos c66 API unit test code.
 *
 *  \details  nortos unit tests
 **/
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
#include <kernel/nortos/dpl/common/ClockP_nortos_priv.h>
#include <unity.h>
#include "ti_drivers_open_close.h"

/* negative test of ClockP_timerClearOverflowInt API */
void test_negClockP_timerClearOverflowInt(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
        ClockP_timerClearOverflowInt(0);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* negative test of ClockP_getTimerCount API */
void test_negClockP_getTimerCount(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
		if(ClockP_getTimerCount(0) == NULL)
		{
			testStatus = SystemP_FAILURE;
        }
    }
    if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* negative test of ClockP_init API */
void test_negClockP_init(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
		ClockP_init();

    }
    if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_neg_main(void *args)
{
	RUN_TEST(test_negClockP_timerClearOverflowInt, 9978, NULL);
	RUN_TEST(test_negClockP_getTimerCount, 9979,NULL);
	RUN_TEST(test_negClockP_init, 9980,NULL);
}

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
 *  \file    SemaphoreP_nortos_test_pos.c
 *
 *  \brief    This file contains SemaphoreP_nortos API unit test code.
 *
 *  \details  SemaphoreP_nortos unit tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

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

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

    int32_t  testStatus = SystemP_SUCCESS;
    int32_t  testStatus1 = SystemP_SUCCESS;
    int32_t  testStatus2 = SystemP_SUCCESS;
    int32_t  testStatus3 = SystemP_SUCCESS;
    int32_t  testStatus4 = SystemP_SUCCESS;
	SemaphoreP_Object obj;
	uint32_t initCount = 0;
	uint32_t maxCount = 0;
	uint32_t timeout = 0;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


void posTest_SemaphoreP_constructBinary(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        if( SemaphoreP_constructBinary(&obj, initCount) != SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
		}
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void posTest_SemaphoreP_constructCounting(void * args)
{
    if (testStatus1 == SystemP_SUCCESS)
    {
		if(SemaphoreP_constructCounting(&obj,initCount,maxCount) != SystemP_SUCCESS)
		{
			testStatus1 = SystemP_FAILURE;
		}
    }

	if (testStatus1 != SystemP_SUCCESS)
    {
        testStatus1 = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }

	if (testStatus2 == SystemP_SUCCESS)
    {
		if(SemaphoreP_constructCounting(&obj,initCount,0U) != SystemP_SUCCESS)
		{
			testStatus2 = SystemP_FAILURE;
		}
    }

	if (testStatus2 != SystemP_SUCCESS)
    {
        testStatus2 = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }

	if (testStatus3 == SystemP_SUCCESS)
    {
		if(SemaphoreP_constructCounting(&obj,2U,1U) != SystemP_SUCCESS)
		{
			testStatus3 = SystemP_FAILURE;
		}
    }

	if (testStatus3 != SystemP_SUCCESS)
    {
        testStatus3 = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
    if ((testStatus1 != SystemP_FAILURE) && (testStatus2 != SystemP_FAILURE) && (testStatus3 != SystemP_FAILURE))
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void posTest_SemaphoreP_constructMutex(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
		if(SemaphoreP_constructMutex(&obj) != SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
		}
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void posTest_SemaphoreP_destruct(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
		SemaphoreP_destruct(&obj);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void posTest_SemaphoreP_pend(void * args)
{
    if (testStatus1 == SystemP_SUCCESS)
    {
		if(SemaphoreP_pend(&obj, SystemP_NO_WAIT) != SystemP_SUCCESS)
		{
			testStatus1 = SystemP_FAILURE;
		}
    }

	if (testStatus1 != SystemP_SUCCESS)
    {
        testStatus1 = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }

	if (testStatus2 == SystemP_SUCCESS)
    {
		uint32_t timeout=10U;
		if(SemaphoreP_pend(&obj, timeout) != SystemP_SUCCESS)
		{
			testStatus2 = SystemP_FAILURE;
		}
    }

    if (testStatus2 != SystemP_SUCCESS)
    {
        testStatus2 = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }

	if (testStatus3 == SystemP_SUCCESS)
    {
		uint32_t timeout= 0x0F;
		if(SemaphoreP_pend(&obj, timeout) != SystemP_SUCCESS)
		{
			testStatus3 = SystemP_FAILURE;
		}
    }

	if (testStatus3 != SystemP_SUCCESS)
    {
        testStatus3 = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }

	if (testStatus4 == SystemP_SUCCESS)
    {
		if(SemaphoreP_pend(&obj, SystemP_WAIT_FOREVER) != SystemP_SUCCESS)
		{
			testStatus4 = SystemP_FAILURE;
		}
    }

	if (testStatus4 != SystemP_SUCCESS)
    {
        testStatus4 = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
	if ((testStatus1 != SystemP_FAILURE) && (testStatus2 != SystemP_FAILURE) && (testStatus3 != SystemP_FAILURE) && (testStatus4 != SystemP_FAILURE))
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void pos_SemaphoreP_pend(void * args)
{
     if (testStatus1 == SystemP_SUCCESS)
    {

		if(SemaphoreP_pend(&obj, -2U) != SystemP_SUCCESS)
		{
			testStatus1 = SystemP_FAILURE;
		}
    }

	if (testStatus1 != SystemP_SUCCESS)
    {
        testStatus1 = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
}
void posTest_SemaphoreP_post(void * args)
{
    if (testStatus1 == SystemP_SUCCESS)
    {
		SemaphoreP_post(&obj);
    }

	if (testStatus1 != SystemP_SUCCESS)
    {
        testStatus1 = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }

	if (testStatus2 == SystemP_SUCCESS)
    {
		SemaphoreP_post(&obj);
    }

	if (testStatus2 != SystemP_SUCCESS)
    {
        testStatus2 = SystemP_FAILURE;
        DebugP_log("SemaphoreP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
    if ((testStatus1 != SystemP_FAILURE) && (testStatus2 != SystemP_FAILURE))
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_pos_main(void * args)
{
    RUN_TEST(posTest_SemaphoreP_constructBinary,9360,NULL);
    RUN_TEST(posTest_SemaphoreP_constructCounting,9355,NULL);
    RUN_TEST(posTest_SemaphoreP_constructMutex,9354,NULL);
    RUN_TEST(posTest_SemaphoreP_destruct, 9356,NULL);
    RUN_TEST(posTest_SemaphoreP_pend, 9358,NULL);
    RUN_TEST(posTest_SemaphoreP_post, 9357,NULL);
    RUN_TEST(pos_SemaphoreP_pend, 9359, NULL);
}

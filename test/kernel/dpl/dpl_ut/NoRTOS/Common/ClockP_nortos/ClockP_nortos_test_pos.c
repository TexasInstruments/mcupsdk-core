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
 */

/**
 *  \file    ClockP_nortos_test_pos.c
 *
 *  \brief    This file contains ClockP positive unit test code.
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
#include <unity.h>
#include "ti_drivers_open_close.h"
#include <kernel/nortos/dpl/common/ClockP_nortos_priv.h>


	void *args = NULL;
	ClockP_Object handle;
	ClockP_Params params;
	uint64_t usecs = 10000U;
	uint32_t ticks = 1U;
	uint32_t timeout = 0;
	uint32_t sec = 0;
	uint32_t usec = 0;
    ClockP_Struct Struct;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

 void test_timerTickIsrOne(void * args)
 {
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {

        ClockP_timerTickIsr(args);
	}
	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
 }

 void test_timerTickIsrTwo(void * args)
 {
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        int *ptr=NULL;
		args=ptr;

        ClockP_timerTickIsr(args);

    }

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
 }

void test_construct(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        params.start = 1U;
        ClockP_construct(&handle, &params);
    }

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_destruct(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {

        ClockP_destruct(&handle);
    }

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_usecToTicks(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {

        if(ClockP_usecToTicks(usecs)==0U)
        {
            testStatus = SystemP_FAILURE;
        }
    }
	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_ticksToUsec(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {

        if(ClockP_ticksToUsec(ticks) == 0U)
        {
            testStatus = SystemP_FAILURE;
        }
    }
	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_getTicks(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {

        if(ClockP_getTicks() == 0U)
        {
            testStatus = SystemP_FAILURE;
        }
    }
	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
 }

void test_getTimeout(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        (ClockP_getTimeout(&handle));
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

 void test_isActive(void *args)
 {
     int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        if (ClockP_isActive(&handle) == 0U)
        {
            testStatus = SystemP_FAILURE;
        }
    }
	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_Params_init(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {

        ClockP_Params_init(&params);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

 void test_setTimeout(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
       ClockP_setTimeout(&handle, timeout);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_start(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        ClockP_start(&handle);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_stop(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        ClockP_stop(&handle);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
       testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_sleep(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
       ClockP_sleep(sec);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

 void test_usleep(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        ClockP_usleep(usec);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
       testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_usleepTwo(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        uint32_t usec = 100*1000U;

        ClockP_usleep(usec);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_getTimeUsec(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        if (ClockP_getTimeUsec() == 0)
        {
            testStatus = SystemP_FAILURE;
        }
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}


void test_pos_main(void *args)
{
RUN_TEST(test_timerTickIsrOne, 5727,NULL);
RUN_TEST(test_timerTickIsrTwo, 5728,NULL);
RUN_TEST(test_construct,5657,NULL);
RUN_TEST(test_destruct,5731,NULL);
RUN_TEST(test_usecToTicks,5733,NULL);
RUN_TEST(test_ticksToUsec,5735,NULL);
RUN_TEST(test_getTicks,5737,NULL);
RUN_TEST(test_getTimeout, 5739,NULL);
RUN_TEST(test_isActive, 5743,NULL);
RUN_TEST(test_Params_init, 5748,NULL);
RUN_TEST(test_setTimeout, 5754,NULL);
RUN_TEST(test_start, 5765,NULL);
RUN_TEST(test_stop, 5771,NULL);
RUN_TEST(test_sleep, 5773,NULL);
RUN_TEST(test_usleep, 5779 ,NULL);
RUN_TEST(test_usleepTwo, 5777,NULL);
RUN_TEST(test_getTimeUsec, 5780,NULL);
}

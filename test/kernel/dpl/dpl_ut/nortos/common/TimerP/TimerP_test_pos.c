/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 *  \file    TimerP_rti_test_neg.c
 *
 *  \brief    This file contains TimerP negative unit test code.
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
#include <kernel/dpl/TimerP.h>

#define TEST_INT_NUM     (20U)

/* Event bit mask upto 24 bits */
#define EVENT_BIT_FROM_PING             (0x000001u)
#define EVENT_BIT_FROM_PONG             (0x000002u)
#define EVENT_BIT_FROM_ISR              (0x100000u)
#define EVENT_BIT2_FROM_ISR             (0x200000u)

#define EVENT_TASK_PRI             (14U)    /* One less than highest priority kernel timer task */
#define EVENT_TASK_STACK_SIZE      (4*1024U)


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

void test_Params_init        (void *args);
void test_setupTwo(void *args);
void test_setupThree(void *args);
void test_setupFour(void *args);
void test_start(void *args);
void test_stop(void *args);
void test_getCount(void *args);
void test_getReloadCount(void *args);
void test_clearOverflowInt(void *args);
void test_isOverflowed(void *args);



/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
Positive test of TimerP_Params_init API
*/
void test_Params_init(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
        TimerP_Params params;
        TimerP_Params_init(&params);
    }

	if (testStatus != SystemP_SUCCESS)
    {

        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
       testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Positive test of TimerP_setup API by setting values of periodInUsec,periodInNsec,inputPreScaler to 1U
*/
void test_setupOne(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;

	uint32_t baseAddr = 0X01U;

    if (testStatus == SystemP_SUCCESS)
    {
       TimerP_Params params;
		params.periodInUsec = 1U;
		params.periodInNsec = 1U;
		params.inputPreScaler=1U;
        TimerP_setup(baseAddr, &params);

    }

	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
       testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Positive test of TimerP_setup API by setting values of periodInUsec to 1U,periodInNsec to 0U,inputPreScaler to 1U
*/
void test_setupTwo(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
	uint32_t baseAddr = 0X01U;

    if (testStatus == SystemP_SUCCESS)
    {
       TimerP_Params params;
		params.periodInUsec = 1U;
		params.periodInNsec = 0U;
		params.inputPreScaler=1U;
        TimerP_setup(baseAddr, &params);

    }

	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
       testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Positive test of TimerP_setup API by setting values of periodInUsec to 1U,periodInNsec to 0U,inputPreScaler to 1U,
inputClkHz to 25*1000000, enableOverflowInt to 0U
*/
void test_setupThree(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
	uint32_t baseAddr = 0X01U;

    if (testStatus == SystemP_SUCCESS)
    {
       TimerP_Params params;
        params.inputClkHz = 25*1000000;
		params.periodInUsec = 0U;
		params.periodInNsec = 1U;
		params.inputPreScaler=2U;
        params.enableOverflowInt=0U;
        TimerP_setup(baseAddr, &params);

    }

	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
       testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Positive test of TimerP_start API
*/
void test_start(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
	uint32_t baseAddr = 0X01U;

    if (testStatus == SystemP_SUCCESS)
    {
        (TimerP_start(baseAddr));
    }

	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
       testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Positive test of TimerP_stop API
*/
void test_stop(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
	uint32_t baseAddr = 0X01U;

    if (testStatus == SystemP_SUCCESS)
    {

        TimerP_stop(baseAddr);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
       testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Positive test of TimerP_getCount API
*/
 void test_getCount(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
	uint32_t baseAddr = 0X01U;

    if (testStatus == SystemP_SUCCESS)
    {

        if (TimerP_getCount(baseAddr)!= FALSE)
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

/*
Positive test of TimerP_getReloadCount API
*/
void test_getReloadCount(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
	uint32_t baseAddr = 0X01U;

    if (testStatus == SystemP_SUCCESS)
    {

        if (TimerP_getReloadCount(baseAddr)!= FALSE)
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

/*
Positive test of TimerP_clearOverflowInt API
*/
void test_clearOverflowInt(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
	uint32_t baseAddr = 0X01U;

    if (testStatus == SystemP_SUCCESS)
    {

        (TimerP_clearOverflowInt(baseAddr));
    }

	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
       testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Positive test of TimerP_isOverflowed API
*/
void test_isOverflowed(void * args)
{
    int32_t  testStatus = SystemP_SUCCESS;
	uint32_t baseAddr = 0X01U;

    if (testStatus == SystemP_SUCCESS)
    {

        (TimerP_isOverflowed(baseAddr));
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
    RUN_TEST(test_Params_init,11550,NULL);
    RUN_TEST(test_setupOne,11551,NULL);
    RUN_TEST(test_setupTwo,11552,NULL);
    RUN_TEST(test_setupThree, 11553, NULL);
    RUN_TEST(test_start,11554,NULL);
    RUN_TEST(test_stop,11555,NULL);
    RUN_TEST(test_getCount,11556,NULL);
    RUN_TEST(test_getReloadCount,11557,NULL);
    RUN_TEST(test_clearOverflowInt,11558,NULL);
    RUN_TEST(test_isOverflowed,11559,NULL);
}

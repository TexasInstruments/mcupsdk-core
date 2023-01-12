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
 *  \file     HeapP_nortos_posTest.c
 *
 *  \brief    This file contains HeapP No RTOS unit test code.
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
#include <kernel/nortos/dpl/common/HeapP_internal.h>
#include <kernel/dpl/DebugP.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#define MY_HEAP_MEM_SIZE            (2 * 1024U)

HeapP_Object heap;
HeapP_MemStats pHeapStats;
uint8_t gMyHeapMem[MY_HEAP_MEM_SIZE];
void * ptr=NULL;
size_t allocSize=40U;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


/* positive test of HeapP_construct API */
void posTest_HeapP_construct(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
        HeapP_construct(&heap,gMyHeapMem,MY_HEAP_MEM_SIZE);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
		DebugP_log("nortos_common_pos_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* positive test of HeapP_destruct API */
void posTest_HeapP_destruct(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
        HeapP_destruct(&heap);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
		DebugP_log("nortos_common_pos_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* positive test of HeapP_alloc API */
void posTest_HeapP_alloc(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
        if(HeapP_alloc(&heap,allocSize)==NULL)
		{
			testStatus = SystemP_FAILURE;
		}
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
		DebugP_log("nortos_common_pos_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* positive test of HeapP_free API */
void posTest_HeapP_free(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
         HeapP_free(&heap,ptr);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
		DebugP_log("nortos_common_pos_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* positive test of HeapP_getFreeHeapSize API */
void posTest_HeapP_getFreeHeapSize(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
        if(HeapP_getFreeHeapSize(&heap)==0)
		{
			testStatus = SystemP_FAILURE;
		}
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
		DebugP_log("nortos_common_pos_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* positive test of HeapP_getMinimumEverFreeHeapSize API */
void posTest_HeapP_getMinimumEverFreeHeapSize(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
        if(HeapP_getMinimumEverFreeHeapSize(&heap)==0)
		{
			testStatus = SystemP_FAILURE;
		}
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
		DebugP_log("nortos_common_pos_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* positive test of HeapP_getHeapStats API */
void posTest_HeapP_getHeapStats(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
        HeapP_getHeapStats(&heap,&pHeapStats);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
		DebugP_log("nortos_common_pos_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}


void test_pos_main(void *args)
 {
    RUN_TEST(posTest_HeapP_construct, 9074, NULL);
	RUN_TEST(posTest_HeapP_destruct, 9075, NULL);
	RUN_TEST(posTest_HeapP_alloc, 9076, NULL);
	RUN_TEST(posTest_HeapP_free, 9077, NULL);
	RUN_TEST(posTest_HeapP_getFreeHeapSize, 9078, NULL);
	RUN_TEST(posTest_HeapP_getMinimumEverFreeHeapSize, 9079, NULL);
	RUN_TEST(posTest_HeapP_getHeapStats, 9080, NULL);

 }

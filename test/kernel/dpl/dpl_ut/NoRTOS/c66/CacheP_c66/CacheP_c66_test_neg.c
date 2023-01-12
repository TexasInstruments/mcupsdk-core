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
 *  \file     CacheP_c66_test_neg.c
 *
 *  \brief    This file contains Nortos R5 API unit test code.
 *
 *  \details  nortos unit tests
 **/

#include <kernel/dpl/AddrTranslateP.h>
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

void *addr;
void *baseAddr;

void test_negCacheP_init(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
    /* Negative test of CacheP_init API */
    if (testStatus == SystemP_SUCCESS)
    {
		CacheP_init();
    }

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);

    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_negCacheP_enable(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	 /* Negative test of CacheP_enable API */
	if (testStatus == SystemP_SUCCESS)
    {
		CacheP_enable(0U);
    }

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_negCacheP_disable(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	/* Negative test of CacheP_disable API */
	if (testStatus == SystemP_SUCCESS)
    {
		CacheP_disable(0U);
    }

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}
void test_negCacheP_getEnabled(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	/* Negative test of CacheP_getEnabled API */
	if (testStatus == SystemP_SUCCESS)
    {
		if(CacheP_getEnabled () == 0U)
		{
			testStatus = SystemP_FAILURE;
		}
    }
	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_negCacheP_wbAll(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	/* Negative test of CacheP_wbAll API */
	if (testStatus == SystemP_SUCCESS)
    {
		CacheP_wbAll(0U);
    }
	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);

    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_negCacheP_wbInvAll(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	/* Negative test of CacheP_wbInvAll API */
	if (testStatus == SystemP_SUCCESS)
    {
		CacheP_wbInvAll(0U);
    }
	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_negCacheP_wb(void *args)
{
	 uint32_t type = 0;
	 uint32_t size = 0;
	 int32_t  testStatusOne = SystemP_SUCCESS;
	 int32_t  testStatusTwo = SystemP_SUCCESS;
	 int32_t  testStatusThree = SystemP_SUCCESS;
	 int32_t  testStatus = SystemP_SUCCESS;
	/* Negative test of CacheP_wb API */
	if (testStatusOne == SystemP_SUCCESS)
    {
		CacheP_wb(NULL, size, type);
    }
	if (testStatusOne != SystemP_SUCCESS)
    {
		testStatusOne = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	if (testStatusTwo == SystemP_SUCCESS)
    {
		CacheP_wb(&addr, 0U, type);
    }
	if (testStatusTwo != SystemP_SUCCESS)
    {
		testStatusTwo = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);

    }
	if (testStatusThree == SystemP_SUCCESS)
    {
		CacheP_wb(&addr, size, 0U);
    }
	if (testStatusThree != SystemP_SUCCESS)
    {
		testStatusThree = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }

	if ((testStatusOne == SystemP_SUCCESS) && (testStatusTwo == SystemP_SUCCESS) && (testStatusThree == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		testStatus = SystemP_FAILURE;
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_negCacheP_inv(void *args)
{
	int32_t  testStatusOne = SystemP_SUCCESS;
	int32_t  testStatusTwo = SystemP_SUCCESS;
	int32_t  testStatusThree = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t type = 0;
	uint32_t size = 0;
	/* Negative test of CacheP_inv API */
	if (testStatusOne == SystemP_SUCCESS)
    {
		CacheP_inv(NULL, size, type);
    }
	if (testStatusOne != SystemP_SUCCESS)
    {
		testStatusOne = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	if (testStatusTwo == SystemP_SUCCESS)
    {
		CacheP_inv(&addr, 0U, type);
    }
	if (testStatusTwo != SystemP_SUCCESS)
    {
		testStatusOne = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	if (testStatusThree == SystemP_SUCCESS)
    {
		CacheP_inv(&addr, size, 0U);
    }
	if (testStatus != SystemP_SUCCESS)
    {
		testStatusOne = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	if ((testStatusOne == SystemP_SUCCESS) && (testStatusTwo == SystemP_SUCCESS) && (testStatusThree == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		testStatus = SystemP_FAILURE;
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_negCacheP_wbInv(void *args)
{
	int32_t  testStatusOne = SystemP_SUCCESS;
	int32_t  testStatusTwo = SystemP_SUCCESS;
	int32_t  testStatusThree = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t type = 0;
	uint32_t size = 0;
	/* Negative test of CacheP_wbInv API */
	if (testStatusOne == SystemP_SUCCESS)
    {
		CacheP_wbInv(NULL, size, type);
    }
	if (testStatusOne != SystemP_SUCCESS)
    {
		testStatusOne = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	if (testStatusTwo == SystemP_SUCCESS)
    {
		CacheP_wbInv(&addr, 0U, type);
    }
	if (testStatusTwo != SystemP_SUCCESS)
    {
		testStatusTwo = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	if (testStatusThree == SystemP_SUCCESS)
    {
		CacheP_wbInv(&addr, size, 0U);
    }
	if (testStatusThree != SystemP_SUCCESS)
    {
		testStatusThree = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_negCacheP_getSize(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	/* Negative test of CacheP_getSize API */
	if (testStatus == SystemP_SUCCESS)
    {
		CacheP_getSize(NULL);
    }
	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_negCacheP_setMar(void *args)
{
	int32_t  testStatusOne = SystemP_SUCCESS;
	int32_t  testStatusTwo = SystemP_SUCCESS;
	int32_t  testStatusThree = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t size = 0;
	uint32_t value = 0;
	/* Negative test of CacheP_setMar API */
	if (testStatusOne == SystemP_SUCCESS)
    {
		CacheP_setMar(NULL,size,value);
    }
	if (testStatusOne != SystemP_SUCCESS)
    {
		testStatusOne = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);

    }
	if (testStatusTwo == SystemP_SUCCESS)
    {
		CacheP_setMar(&baseAddr,0U,value);
    }
	if (testStatusTwo != SystemP_SUCCESS)
    {
		testStatusTwo = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);
    }
	if (testStatusThree == SystemP_SUCCESS)
    {
		CacheP_setMar(&baseAddr,size,0U);
    }
	if (testStatusThree != SystemP_SUCCESS)
    {
		testStatusThree = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);

    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_negCacheP_getMar(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
    {
		if(CacheP_getMar(NULL) == 0)
		{
			testStatus = SystemP_FAILURE;
		}
    }
	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("nortos_c66_neg_Test: failure on line no. %d \n", __LINE__);

    }
}

void test_neg_main(void *args)
 {
    RUN_TEST(test_negCacheP_init, 282, NULL);
	RUN_TEST(test_negCacheP_enable, 282, NULL);
	RUN_TEST(test_negCacheP_disable, 282, NULL);
	RUN_TEST(test_negCacheP_wbAll, 282, NULL);
	RUN_TEST(test_negCacheP_wb, 282, NULL);
	RUN_TEST(test_negCacheP_inv, 282, NULL);
	RUN_TEST(test_negCacheP_setMar, 282, NULL);
	RUN_TEST(test_negCacheP_getMar, 282, NULL);
	RUN_TEST(test_negCacheP_wbInvAll, 282, NULL);
	RUN_TEST(test_negCacheP_wbInv, 282, NULL);
	RUN_TEST(test_negCacheP_getEnabled, 282,NULL);
	RUN_TEST(test_negCacheP_getSize, 282, NULL);

 }

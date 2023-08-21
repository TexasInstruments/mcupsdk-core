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
 **
 *  \file    CacheP_armv7r_testneg.c
 *
 *  \brief    This file contains CacheP_armv7r API unit test code.
 *
 *  \details  CacheP_armv7r unit tests
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
#include <kernel/dpl/CacheP.h>


void  CacheP_init_neg(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;
	/*  Error/Fault test of CacheP_init API*/
    if (testStatus == SystemP_SUCCESS)
    {
        CacheP_init();
    }

    if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("CacheP_armv7r_neg_test: falure on line no %d \n", __LINE__);

    }

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void CacheP_disable_neg(void *args)
{
    /*  Error/Fault test of disableInt API*/
	int32_t    testStatus = SystemP_SUCCESS;
	uint32_t type =0;
    if (testStatus == SystemP_SUCCESS)
    {
        CacheP_disable(type);

    }

    if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("CacheP_armv7r_neg_test: falure on line no %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);

}

void CacheP_enable_neg(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;
	uint32_t type =0;
    /*  Error/Fault test of CacheP_enable API*/
    if (testStatus == SystemP_SUCCESS)
    {
        CacheP_enable(type);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("CacheP_armv7r_neg_test: falure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void  CacheP_inv_neg(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;
	uint32_t type = 1U;
	void *blockPtr=NULL;
	uint32_t byteCnt=0;
    /*  Error/Fault test of CacheP_inv API*/
    if (testStatus == SystemP_SUCCESS)
    {
        CacheP_inv(blockPtr, byteCnt, type);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("CacheP_armv7r_neg_test: falure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void  CacheP_inv_neg_one(void *args)
{
	int32_t    testStatus = SystemP_SUCCESS;
	uint32_t type = 2U;
	void *blockPtr=NULL;
	uint32_t byteCnt=0;
    /*  Error/Fault test of CacheP_inv API*/
    if (testStatus == SystemP_SUCCESS)
    {
        CacheP_inv(blockPtr, byteCnt, type);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("CacheP_armv7r_neg_test: falure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_neg_main(void *args)
{

    RUN_TEST(CacheP_init_neg, 11433, NULL);
    RUN_TEST(CacheP_disable_neg, 11434, NULL);
    RUN_TEST(CacheP_enable_neg, 11435, NULL);
    RUN_TEST(CacheP_inv_neg, 11436, NULL);
    RUN_TEST(CacheP_inv_neg_one, 11543, NULL);

}

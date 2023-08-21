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
 *  \file     MpuP_armv7r_test_pos.c
 *
 *  \brief    This file contains MpuP_armv7r API unit test code.
 *
 *  \details  MpuP_armv7r unit tests
 **/

#include "test_dpl.h"
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
#include <kernel/dpl/MpuP_armv7.h>
#include <kernel/dpl/DebugP.h>


int32_t testStatus = SystemP_SUCCESS;
MpuP_RegionAttrs region, attrs;
uint32_t regionNum=1;
void *addr=NULL;
uint32_t size=0x4U;


/*  Positive test of MpuP_RegionAttrs_init API*/
void posTest_MpuP_RegionAttrs_init(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        MpuP_RegionAttrs_init(&region);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("MpuP_armv7r_pos_test: failure on line no %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of MpuP_setRegion API*/
void posTest_MpuP_setRegion(void * args)
{

    if (testStatus == SystemP_SUCCESS)
    {
        MpuP_RegionAttrs_init(&region);
        region.isEnable       = 1;
        MpuP_setRegion(regionNum,addr,size,&attrs);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("MpuP_armv7r_pos_test: failure on line no %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of MpuP_enable API*/
void posTest_MpuP_enable(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        MpuP_enable();
        CacheP_enable(CacheP_TYPE_ALL);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("MpuP_armv7r_pos_test: failure on line no %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of MpuP_disable API*/
void posTest_MpuP_disable(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        CacheP_disable(CacheP_TYPE_ALL);
        MpuP_disable();
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("MpuP_armv7r_pos_test: failure on line no %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of MpuP_isEnable API*/
void posTest_MpuP_isEnable(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        if(MpuP_isEnable()==1)
        {
            testStatus=SystemP_FAILURE;
		}
    }

   if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("MpuP_armv7r_pos_test: failure on line no %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_pos_main(void * args)
{
    RUN_TEST(posTest_MpuP_enable,11820,NULL);
    RUN_TEST(posTest_MpuP_disable,11819,NULL);
    RUN_TEST(posTest_MpuP_isEnable,11818,NULL);
    RUN_TEST(posTest_MpuP_setRegion,11817,NULL);
    RUN_TEST(posTest_MpuP_RegionAttrs_init,11816,NULL);
}

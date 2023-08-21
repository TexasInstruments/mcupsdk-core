/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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
 *  \file     HwiP_armv7r_vim_testpos.c
 *
 *  \brief    This file contains HwiP_armv7r_vim API unit test code.
 *
 *  \details  HwiP_armv7r_vim unit tests
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
#include <kernel/dpl/CpuIdP.h>
#include <kernel/dpl/DebugP.h>
#include <stdint.h>
#include <kernel/nortos/dpl/r5/CpuId_armv7r.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/MpuP_armv7.h>
#include <drivers/hw_include/soc_config.h>

#define TEST_INT_NUM     (20U)
int32_t    testStatus = SystemP_SUCCESS;
uint32_t intNum=1;
uint32_t oldIntState;
uint32_t oldEnableState=1U;
HwiP_Params params;
HwiP_Object handle;
volatile uint32_t gMyISRCount;
volatile uint32_t gInISR;
static SemaphoreP_Object gMyDoneSem;

static void myISR4(void *args)
{
    gMyISRCount++;

    gInISR += HwiP_inISR();
}

static void myISR6(void *args)
{
    gInISR = 1;
}

static void myISR3(void *args)
{
    gInISR = 1;
}

static void myISR2(void *args)
{
    SemaphoreP_Object *pSemObj = (SemaphoreP_Object *)args;

    SemaphoreP_post(pSemObj);
}

/*  Positive test of enableInt API*/
void posTest_HwiP_enableInt(void * args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        HwiP_enableInt(TEST_INT_NUM);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("HwiP_armv7r_vim_pos_test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of disableInt API*/
void posTest_HwiP_disableInt(void * args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        if (HwiP_disableInt(TEST_INT_NUM) != 1)
        {
            testStatus = SystemP_FAILURE;
        }
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("HwiP_armv7r_vim_pos_test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of clearInt API*/
void posTest_HwiP_clearInt(void * args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        HwiP_clearInt(TEST_INT_NUM);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("HwiP_armv7r_vim_pos_test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of Params_init API*/
void posTest_HwiP_Params_init(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        HwiP_Params hwiParams;
        HwiP_Object hwiObj;
        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = TEST_INT_NUM;
        hwiParams.callback = myISR3;
        testStatus = HwiP_construct(&hwiObj, &hwiParams);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("HwiP_armv7r_vim_pos_test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of HwiP_disable API*/
void posTest_HwiP_disable(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        oldIntState = HwiP_disable();
        HwiP_restore(oldIntState);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("HwiP_armv7r_vim_pos_test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}


/*  Positive test of destruct API*/
void posTest_HwiP_destruct(void * args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        HwiP_Params hwiParams;
        HwiP_Object hwiObj;
        uint32_t i, maxCount = 10;

        gMyISRCount = 0;
        gInISR = 0;

        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = TEST_INT_NUM;
        hwiParams.callback = myISR4;
        testStatus = HwiP_construct(&hwiObj, &hwiParams);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);

    for (i = 0; i < maxCount; i++)
    {
        HwiP_post(hwiParams.intNum);
    }

    while (gMyISRCount < maxCount)
        ;

    TEST_ASSERT_EQUAL_INT32(maxCount, gMyISRCount);
    TEST_ASSERT_EQUAL_INT32(maxCount, gInISR);
    TEST_ASSERT_EQUAL_UINT32( 0, HwiP_inISR());

    HwiP_destruct(&hwiObj);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("HwiP_armv7r_vim_pos_test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of destruct API by setting isFIQ as 1*/
void posTest_HwiP_destructOne(void * args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        HwiP_Params hwiParams;
        HwiP_Object hwiObj;
        uint32_t i, maxCount = 10;

        gMyISRCount = 0;
        gInISR = 0;

        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = TEST_INT_NUM;
        hwiParams.callback = myISR4;
        hwiParams.isFIQ = 1U;
        testStatus = HwiP_construct(&hwiObj, &hwiParams);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);

    for (i = 0; i < maxCount; i++)
    {
        HwiP_post(hwiParams.intNum);
    }

    while (gMyISRCount < maxCount)
        ;

    TEST_ASSERT_EQUAL_INT32(maxCount, gMyISRCount);
    TEST_ASSERT_EQUAL_INT32(maxCount, gInISR);
    TEST_ASSERT_EQUAL_UINT32( 0, HwiP_inISR());

    HwiP_destruct(&hwiObj);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("HwiP_armv7r_vim_pos_test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of restoreInt API*/
void posTest_HwiP_restoreInt(void * args)
{
    intNum = TEST_INT_NUM;
    oldEnableState = 1;
    int32_t    testStatus = SystemP_SUCCESS;
    int32_t    testStatus1 = SystemP_SUCCESS;
    int32_t    testStatus2 = SystemP_SUCCESS;
    if (testStatus1 == SystemP_SUCCESS)
    {
        HwiP_restoreInt(intNum, oldEnableState);
    }

    if (testStatus1 != SystemP_SUCCESS)
    {
        testStatus1 = SystemP_FAILURE;
        DebugP_log("HwiP_armv7r_vim_pos_test: failure on line no %d \n", __LINE__);
    }

    if (testStatus2 == SystemP_SUCCESS)
    {
        oldEnableState = 0U;
        HwiP_restoreInt(intNum, oldEnableState);
    }

    if (testStatus2 != SystemP_SUCCESS)
    {
        testStatus2 = SystemP_FAILURE;
        DebugP_log("HwiP_armv7r_vim_pos_test: failure on line no %d \n", __LINE__);
    }

    if((testStatus1 == SystemP_SUCCESS) && (testStatus2 == SystemP_SUCCESS))
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of init API with multiple callbacks*/
void posTest_HwiP_Params_initOne(void * args)
{
    if(testStatus == SystemP_SUCCESS)
    {
        HwiP_Params hwiParams1,hwiParams2;
        HwiP_Object hwiObj1,hwiObj2;

        HwiP_Params_init(&hwiParams1);
        hwiParams1.intNum = TEST_INT_NUM;
        hwiParams1.callback = myISR6;
        testStatus = HwiP_construct(&hwiObj1, &hwiParams1);

        HwiP_Params_init(&hwiParams2);
        hwiParams2.intNum = TEST_INT_NUM;
        hwiParams2.callback = myISR2;
        hwiParams2.args = &gMyDoneSem;
        HwiP_construct(&hwiObj2, &hwiParams2);
        HwiP_destruct(&hwiObj2);
        HwiP_destruct(&hwiObj1);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("HwiP_armv7r_vim_pos_test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of setArgs API*/
void posTest_HwiP_setArgs(void * args)
{
    void * arguments=NULL;
    int32_t    testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        if (HwiP_setArgs(&handle,arguments) == 1)
        {
            testStatus = SystemP_FAILURE;
        }
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("HwiP_armv7m_pos_test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_pos_main(void * args)
{
    RUN_TEST(posTest_HwiP_destruct,11621,NULL);
    RUN_TEST(posTest_HwiP_enableInt,11623,NULL);
    RUN_TEST(posTest_HwiP_disableInt,11624,NULL);
    RUN_TEST(posTest_HwiP_clearInt,11625,NULL);
    RUN_TEST(posTest_HwiP_restoreInt,11626,NULL);
    RUN_TEST(posTest_HwiP_Params_initOne,11628,NULL);
    RUN_TEST(posTest_HwiP_Params_init,11627,NULL);
    RUN_TEST(posTest_HwiP_destructOne,11622,NULL);
    RUN_TEST(posTest_HwiP_setArgs,11629,NULL);
}

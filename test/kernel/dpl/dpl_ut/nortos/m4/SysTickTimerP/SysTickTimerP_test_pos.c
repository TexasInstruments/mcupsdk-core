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
 *  \file     SysTickTimerP_test_pos.c
 *
 *  \brief    This file contains SysTickTimerP API unit test code.
 *
 *  \details  SysTickTimerP_test_pos unit tests
 **/


#include <kernel/dpl/HwiP.h>
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
#include <kernel/nortos/dpl/common/ClockP_nortos_priv.h>
#include <kernel/nortos/dpl/m4/SysTickTimerP.h>
#include <drivers/hw_include/cslr.h>

int32_t  testStatus = SystemP_SUCCESS;
TimerP_Params *params;

/* Positive test for SysTickTimerP_Params_init API*/
void posTest_SysTickTimerP_Params_init(void * args)
{
    if(testStatus == SystemP_SUCCESS)
    {
       SysTickTimerP_Params_init(params);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("SysTickTimerP_pos_Test: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test for SysTickTimerP_setup API for Branch Coverage*/
void posTest_SysTickTimerP_setupOne(void * args)
{
    if(testStatus == SystemP_SUCCESS)
    {
        params->inputClkHz = 200*1000000;
        params->periodInUsec = 1000;
        params->periodInNsec = 10U;
        params->oneshotMode = 1;
        params->enableOverflowInt = 0;
       SysTickTimerP_setup(params);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("SysTickTimerP_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test for SysTickTimerP_setup API*/
void posTest_SysTickTimerP_setup(void * args)
{
    if(testStatus == SystemP_SUCCESS)
    {
        TimerP_Params param;
        param.periodInNsec = 0;
        param.oneshotMode = 1;
        param.enableOverflowInt = 0;
       SysTickTimerP_setup(params);
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("SysTickTimerP_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test for SysTickTimerP_start API*/
void posTest_SysTickTimerP_start(void * args)
{
    if(testStatus == SystemP_SUCCESS)
    {
       SysTickTimerP_start();
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("SysTickTimerP_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test for SysTickTimerP_stop API*/
void posTest_SysTickTimerP_stop(void * args)
{
    if(testStatus == SystemP_SUCCESS)
    {
       SysTickTimerP_stop();
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("SysTickTimerP_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test for SysTickTimerP_getCount API*/
void posTest_SysTickTimerP_getCount(void * args)
{
    if(testStatus == SystemP_SUCCESS)
    {
       if(SysTickTimerP_getCount()==1U)
       {
           testStatus = SystemP_FAILURE;
       }
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("SysTickTimerP_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test for SysTickTimerP_getReloadCount API*/
void posTest_SysTickTimerP_getReloadCount(void * args)
{
    if(testStatus == SystemP_SUCCESS)
    {
       if(SysTickTimerP_getReloadCount()==1U)
       {
           testStatus = SystemP_FAILURE;
       }
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("SysTickTimerP_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test for SysTickTimerP_isOverflowed API*/
void posTest_SysTickTimerP_isOverflowed(void * args)
{
    if(testStatus == SystemP_SUCCESS)
    {
       if(SysTickTimerP_isOverflowed()==1U)
       {
           testStatus = SystemP_FAILURE;
       }
    }

    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("SysTickTimerP_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_pos_main(void * args)
{
    RUN_TEST(posTest_SysTickTimerP_setupOne,11133,NULL);
    RUN_TEST(posTest_SysTickTimerP_Params_init,11131,NULL);
    RUN_TEST(posTest_SysTickTimerP_setup,11132,NULL);
    RUN_TEST(posTest_SysTickTimerP_start,11134,NULL);
    RUN_TEST(posTest_SysTickTimerP_stop,11135,NULL);
    RUN_TEST(posTest_SysTickTimerP_getCount,11136,NULL);
    RUN_TEST(posTest_SysTickTimerP_isOverflowed,11138,NULL);
    RUN_TEST(posTest_SysTickTimerP_getReloadCount,11137,NULL);
}

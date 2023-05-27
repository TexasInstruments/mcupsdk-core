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
 *  \file    DebugP_log_test_pos.c
 *
 *  \brief    This file contains Nortos common API unit test code.
 *
 *  \details  nortos unit tests
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
#include <kernel/dpl/DebugP.h>

int32_t  testStatus = SystemP_SUCCESS;
uint32_t logZoneMask = 0;
int expression = 1;
const char file[]="debugP";
const char function[]="debugassert";
int line = 20;
const char expressionString;

/* positive test of DebugP_logZoneEnable API */
void posTest_DebugP_logZoneEnable(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        if(DebugP_logZoneEnable(DebugP_LOG_ZONE_INFO) == 0)
		{
			testStatus = SystemP_SUCCESS;
		}
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_log_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* positive test of DebugP_logZoneDisable API */
void posTest_DebugP_logZoneDisable(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        if(DebugP_logZoneDisable(DebugP_LOG_ZONE_INFO) == 0)
		{
			testStatus = SystemP_FAILURE;
		}
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_log_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* positive test of DebugP_logZoneRestore API */
void posTest_DebugP_logZoneRestore(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
		uint32_t oldDebugLogZone;
		oldDebugLogZone = DebugP_logZoneDisable(logZoneMask);
        DebugP_logZoneRestore(oldDebugLogZone);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_log_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* positive test of DebugP_logChar API */
void posTest_DebugP_logChar(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
		char a = 'A';
        DebugP_logChar(a);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_log_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_pos_main(void * args)
{
    RUN_TEST(posTest_DebugP_logZoneEnable,11437,NULL);
    RUN_TEST(posTest_DebugP_logZoneDisable,11438,NULL);
    RUN_TEST(posTest_DebugP_logZoneRestore,11439,NULL);
    RUN_TEST(posTest_DebugP_logChar,11440,NULL);
}

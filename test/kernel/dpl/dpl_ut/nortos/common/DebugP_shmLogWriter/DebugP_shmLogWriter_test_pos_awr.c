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
 *  \file     DebugP_shmLogWriter_test_pos.c
 *
 *  \brief    This file contains DebugP_shmLogWriter API unit test code.
 *
 *  \details  DebugP_shmLogWriter unit tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/soc.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/HeapP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/EventP.h>
#include <kernel/dpl/QueueP.h>
#include <drivers/soc.h>
#include <unity.h>
#include "ti_drivers_open_close.h"
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/nortos/dpl/common/printf.h>

#define DebugP_SHM_LOG_SIZE_ONE  ((2*1024U) - 15U)
#define DebugP_SHM_LOG_SIZE_TWO  ((2*1024U) + 2U)
#define DebugP_SHM_LOG_SIZE_FOUR  ((2*1024U) + 4U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

	int32_t testStatus=SystemP_SUCCESS;
    int32_t         testStatus1 = SystemP_SUCCESS;
    int32_t         testStatus2 = SystemP_SUCCESS;
    int32_t         testStatus3 = SystemP_SUCCESS;
    int32_t         testStatus4 = SystemP_SUCCESS;
    int32_t         testStatus5 = SystemP_SUCCESS;
    DebugP_ShmLog shmLog, shmlog1;
    uint16_t selfCoreId=1;
    uint8_t *buf=NULL;
    uint16_t num_bytes=1;
    char character= '\0';

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


/*
 * Test to verify DebugP_shmLogWriterPutChar API with valid parameters for MCDC Coverage
 */
void posTest_DebugP_shmLogWriterPutCharThree(void *args)
{
    DebugP_shmLogWriterPutChar(character);
    DebugP_shmLogWriterInit(&shmlog1,selfCoreId);
    if(testStatus2==SystemP_SUCCESS)
	{
        char character= 'a';
		shmlog1.wrIndex = DebugP_SHM_LOG_SIZE_TWO;
        shmlog1.rdIndex = DebugP_SHM_LOG_SIZE_TWO;
        DebugP_shmLogWriterPutChar(character);
    }
	if (testStatus2 != SystemP_SUCCESS)
    {
        testStatus2 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogWriter_nortos_posTest: failure on line no. %d \n", __LINE__);
    }
}

/*
 * Test to verify DebugP_shmLogWriterPutChar API with valid parameters for Branch and MC/DC.
 */
void posTest_DebugP_shmLogWriterPutCharFour(void *args)
{
    DebugP_shmLogWriterPutChar(character);
    DebugP_shmLogWriterInit(&shmlog1,selfCoreId);
    if(testStatus2==SystemP_SUCCESS)
	{
        char character= 'a';
		shmlog1.wrIndex = 2;
        shmlog1.rdIndex = 4;
        DebugP_shmLogWriterPutChar(character);
    }
	if (testStatus2 != SystemP_SUCCESS)
    {
        testStatus2 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogWriter_nortos_posTest: failure on line no. %d \n", __LINE__);
    }
}

/*
 * Test to verify DebugP_shmLogWriterInit API with valid parameters.
 */
void posTest_DebugP_shmLogWriterInit(void * args)
{
    if(testStatus==SystemP_SUCCESS)
	{
        DebugP_shmLogWriterInit(&shmLog,selfCoreId);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogWriter_nortos_posTest: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify DebugP_shmLogWriterPutChar API.
 */
void posTest_DebugP_shmLogWriterPutChar(void * args)
{
    if(testStatus1==SystemP_SUCCESS)
	{
        DebugP_shmLogWriterPutChar(character);
    }
	if (testStatus1 != SystemP_SUCCESS)
    {
        testStatus1 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogWriter_nortos_posTest: failure on line no. %d \n", __LINE__);
    }

	if(testStatus2==SystemP_SUCCESS)
	{
		char character= '\n';
        DebugP_shmLogWriterPutChar(character);
    }
	if (testStatus2 != SystemP_SUCCESS)
    {
        testStatus2 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogWriter_nortos_posTest: failure on line no. %d \n", __LINE__);
    }
    if(testStatus3==SystemP_SUCCESS)
	{
		char character= 'z';
        DebugP_shmLogWriterPutChar(character);
    }
	if (testStatus3 != SystemP_SUCCESS)
    {
        testStatus3 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogWriter_nortos_posTest: failure on line no. %d \n", __LINE__);
    }

    if(testStatus4==SystemP_SUCCESS)
	{
        int character = 0;
        while(character<2049)
        {
            DebugP_shmLogWriterPutChar(character);
            character++;
        }
    }
	if (testStatus4 != SystemP_SUCCESS)
    {
        testStatus4 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogWriter_nortos_posTest: failure on line no. %d \n", __LINE__);
    }
    if(testStatus5==SystemP_SUCCESS)
	{
        int character = 0;
        while(character<3072)
        {
            DebugP_shmLogWriterPutChar(character);
            character++;
        }
    }
	if (testStatus5 != SystemP_SUCCESS)
    {
        testStatus5 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogWriter_nortos_posTest: failure on line no. %d \n", __LINE__);
    }
    if ((testStatus1 != SystemP_FAILURE) && (testStatus2 != SystemP_FAILURE) && (testStatus3 != SystemP_FAILURE) && (testStatus4 != SystemP_FAILURE) && (testStatus5 != SystemP_FAILURE))
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_pos_main(void *args)
{
    RUN_TEST(posTest_DebugP_shmLogWriterPutCharThree, 11738, NULL);
    RUN_TEST(posTest_DebugP_shmLogWriterPutCharFour, 11739, NULL);
    RUN_TEST(posTest_DebugP_shmLogWriterPutChar, 5978,NULL);
    RUN_TEST(posTest_DebugP_shmLogWriterInit, 5976, NULL);
    RUN_TEST(posTest_DebugP_shmLogWriterPutChar, 5978,NULL);
}

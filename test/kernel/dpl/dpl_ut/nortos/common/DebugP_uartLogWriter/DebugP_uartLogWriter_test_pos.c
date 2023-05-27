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
 *  \file    DebugP_uartLogWriter_test_pos.c
 *
 *  \brief    This file contains DebugP_uartLogWriter API unit test code.
 *
 *  \details  DebugP_uartLogWriter unit tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

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
#include <drivers/uart.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/nortos/dpl/common/printf.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

    int32_t  testStatus = SystemP_SUCCESS;
    int32_t testStatus1 = SystemP_SUCCESS;
    int32_t testStatus2 = SystemP_SUCCESS;
	uint8_t buf=0xFF;
	char character=0xFF;
	uint16_t num_bytes=0;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

void DebugP_uartLogWriterPutCharBuffered(char character);
void DebugP_uartLogWriterPutLine(uint8_t *buf, uint16_t num_bytes);

/*
 * Test to verify DebugP_uartSetDrvIndex API with valid parameters.
 */
void posTest_DebugP_uartSetDrvIndex(void * args)
{
    uint32_t gDebugP_uartDrvIndex = 0xFFFFFFFF;
    if (testStatus == SystemP_SUCCESS)
    {
       DebugP_uartSetDrvIndex(gDebugP_uartDrvIndex);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_uartLogWriter_posTest: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify DebugP_uartLogWriterPutChar API with valid parameters.
 */
void posTest_DebugP_uartLogWriterPutChar(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
       DebugP_uartLogWriterPutChar(character);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_uartLogWriter_posTest: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify DebugP_uartLogWriterPutLine API with valid parameters.
 */
void posTest_DebugP_uartLogWriterPutLine(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
       DebugP_uartLogWriterPutLine(&buf,num_bytes);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_uartLogWriter_posTest: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify DebugP_uartLogWriterPutCharBuffered API with valid & invalid parameters to acheive Branch or MC/DC.
 */
void posTest_DebugP_uartLogWriterPutCharBuffered(void * args)
{
    if (testStatus1 == SystemP_SUCCESS)
    {
       DebugP_uartLogWriterPutCharBuffered(character);
    }

	if (testStatus1 != SystemP_SUCCESS)
    {
        testStatus1 = SystemP_FAILURE;
        DebugP_log("DebugP_uartLogWriter_posTest: failure on line no. %d \n", __LINE__);
    }

	 if (testStatus2 == SystemP_SUCCESS)
    {
		char character= '\n';
       DebugP_uartLogWriterPutCharBuffered(character);
    }

	if (testStatus2 != SystemP_SUCCESS)
    {
        testStatus2 = SystemP_FAILURE;
        DebugP_log("DebugP_uartLogWriter_posTest: failure on line no. %d \n", __LINE__);
    }

    if ((testStatus1 != SystemP_FAILURE) && (testStatus2 != SystemP_FAILURE))
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);

}

/*
 * Test to verify DebugP_uartLogWriterPutCharBuffered API by passing the character values greater than 128.
 */
void posTest_DebugP_uartLogWriterPutCharBufferedOne(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        char abc[140] = "Hydrophilic signal molecules, such as growth factors, adrenaline, and insulin,it cannot pass the cell membrane because as the cell.\n";
        int32_t len = strlen(abc);
        int32_t i = 0;
        for(i=0; i<len; i++)
        {
            DebugP_uartLogWriterPutCharBuffered(character);
        }
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_uartLogWriter_posTest: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify DebugP_uartLogWriterPutCharBuffered API with '\n' as a parameter.
 */
void posTest_DebugP_uartLogWriterPutCharBufferedTwo(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        char character= '\n';
       DebugP_uartLogWriterPutCharBuffered(character);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_uartLogWriter_posTest: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_pos_main(void *args)
{
    RUN_TEST(posTest_DebugP_uartLogWriterPutLine, 10794,NULL);
    RUN_TEST(posTest_DebugP_uartSetDrvIndex, 5982, NULL);
    RUN_TEST(posTest_DebugP_uartLogWriterPutChar, 5984,NULL);
    RUN_TEST(posTest_DebugP_uartLogWriterPutCharBufferedTwo,10792,NULL);
    RUN_TEST(posTest_DebugP_uartLogWriterPutCharBuffered,10791,NULL);
    RUN_TEST(posTest_DebugP_uartLogWriterPutCharBufferedOne,10793,NULL);
}

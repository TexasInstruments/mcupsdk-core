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
 *  \file     DebugP_uartScanf_test_pos.c
 *
 *  \brief    This file contains DebugP_uartScanf API unit test code.
 *
 *  \details  DebugP_uartScanf unit tests
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
#include <stdarg.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/uart.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

	extern uint32_t gDebugP_uartDrvIndex;
	UART_Handle uartHandle;
	int32_t testStatus=SystemP_SUCCESS;
    char lineBuf;
    uint32_t bufSize = 0xFFFFF;
    uint32_t value32 = 0;
    UART_Transaction trans;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * Test to verify DebugP_readLine API with valid parameters.
 */
void posTest_DebugP_readLine(void * args)
{
	UART_Handle handle = NULL;
    UART_close(handle);
	if(testStatus==SystemP_SUCCESS)
	{
        trans.status=1;
		if(DebugP_readLine(&lineBuf,bufSize)== SystemP_SUCCESS)
		{
			testStatus=SystemP_FAILURE;
		}
	}

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
    	DebugP_log("DebugP_uartScanf_nortos_posTest: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify DebugP_scanf API with valid parameters.
 */
void posTest_DebugP_scanf(void * args)
{
	UART_Handle handle = NULL;
	if(testStatus==SystemP_SUCCESS)
	{
    	value32 = 0;
		DebugP_log("Enter a 32b number\r\n");
    	DebugP_scanf("%d", &value32);
    	DebugP_log("32b value = %d\r\n", value32);
	}

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_uartScanf_nortos_posTest: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
    UART_close(handle);
}

/*
 * Test to verify DebugP_scanf API with different invalid parameters to cover Branch .
 */
void posTest_DebugP_scanfOne(void * args)
{
    UART_Handle handle = NULL;
    UART_close(handle);

	if(testStatus==SystemP_SUCCESS)
	{
    	value32 = 0xFF;
		DebugP_log("Enter a 32b number\r\n");
    	DebugP_scanf("%d", &value32);
    	DebugP_log("32b value = %d\r\n", value32);
	}

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_uartScanf_nortos_posTest: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify DebugP_readLine API with valid parameters and uartDrvIndex set with 'CONFIG_UART0'.
 */
 void posTest_DebugP_readLineOne(void * args)
 {
     if(testStatus==SystemP_SUCCESS)
	{
        DebugP_uartSetDrvIndex(0);
        if(DebugP_readLine(&lineBuf,bufSize)== SystemP_SUCCESS)
		{
			testStatus=SystemP_FAILURE;
		}
	}

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
    	DebugP_log("DebugP_uartScanf_nortos_posTest: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
 }

/*
 * Test to verify DebugP_readLine API with valid parameters.
 */
 void posTest_DebugP_readLineTwo(void * args)
 {
    if(testStatus==SystemP_SUCCESS)
	{
        trans.status=1;
        if(DebugP_readLine(&lineBuf,bufSize)== SystemP_SUCCESS)
		{
			testStatus=SystemP_FAILURE;
		}
	}

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
    	DebugP_log("DebugP_uartScanf_nortos_posTest: failure on line no. %d \n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
 }

void test_pos_main(void *args)
{
	RUN_TEST(posTest_DebugP_scanf,5991, NULL);
    RUN_TEST(posTest_DebugP_scanfOne,10796, NULL);
    RUN_TEST(posTest_DebugP_readLine,5990, NULL);
	RUN_TEST(posTest_DebugP_readLineTwo,10795, NULL);
	RUN_TEST(posTest_DebugP_readLineOne,5992, NULL);
}

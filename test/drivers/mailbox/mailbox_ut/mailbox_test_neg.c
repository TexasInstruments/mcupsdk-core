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
 *  \file     mailbox_test_neg.c
 *
 *  \brief    This file contains mailbox API unit test code.
 *
 *  \details  mailbox unit tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <unity.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include <stdint.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/mailbox/v0/mailbox_priv.h>
#include <drivers/ipc_notify.h>
#include <drivers/mailbox/v0/mailbox.h>
#include <drivers/hw_include/awr294x/cslr_soc_defines.h>

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */


static const uint16_t gMessage_r5fss0_0[11] =
{
    0x1234, 0x4321, 0x0448, 0x0012,
    0x0000, 0x0000, 0x0001, 0xFBA4,
    0x0220, 0x0004, 0xBA39
};

static const uint16_t gMessage_c66ss0[11] =
{
    0x1234, 0x4321, 0x044A, 0x0012,
    0x0000, 0x0000, 0x0001, 0xFBA2,
    0x0220, 0x0004, 0x77B1
};

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

	uint32_t    remoteCoreId = 0;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void negTest_readCallback(void * args)
{
    int32_t testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
	{
	  Mailbox_readCallback(0U);
    }

    if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("mailbox_neg_Test: failure on line no. %d  \r\n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void negTest_isCoreEnabled(void * args)
{
    int32_t testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        if (Mailbox_isCoreEnabled(5U) != 0)
        {
            testStatus = SystemP_FAILURE;
			DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
        }
    }

    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void negTest_Params_init(void * args)
{
    int32_t testStatus = SystemP_SUCCESS;
    if(testStatus == SystemP_SUCCESS)
    {
        Mailbox_Params_init(NULL);
    }

    if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
    }

    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void negTest_setReadCallback(void * args)
{
    int32_t testStatus = SystemP_SUCCESS;
	int32_t testStatus1=SystemP_SUCCESS;
    int32_t testStatus2=SystemP_SUCCESS;

    if(testStatus1==SystemP_SUCCESS)
    {
    	void *readCallbackArgs;
      	Mailbox_setReadCallback(NULL,&readCallbackArgs);
    }

    if (testStatus1 != SystemP_SUCCESS)
    {
		testStatus1 = SystemP_FAILURE;
        DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
    }

    if(testStatus2==SystemP_SUCCESS)
    {
        Mailbox_ReadCallback readCallback = NULL;
        Mailbox_setReadCallback(readCallback,NULL);
    }

    if (testStatus2 != SystemP_SUCCESS)
    {
		testStatus2 = SystemP_FAILURE;
        DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
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

void negTest_getRemoteCoreObj(void * args)
{
     int32_t         testStatus = SystemP_SUCCESS;
	int32_t         testStatus1 = SystemP_SUCCESS;
    int32_t         testStatus2 = SystemP_SUCCESS;

    if (testStatus1 == SystemP_SUCCESS)
    {
        uint32_t selfCoreId = CSL_CORE_ID_R5FSS0_0;
		uint32_t remoteCoreId = CSL_CORE_ID_MAX;
		if (Mailbox_getRemoteCoreObj(selfCoreId, remoteCoreId)!= NULL)
		{
			testStatus1 = SystemP_FAILURE;
			DebugP_log("mailbox_pos_Test: failure on line no. %d \r\n", __LINE__);
		}
	}

    if (testStatus2 == SystemP_SUCCESS)
	{
		uint32_t selfCoreId = CSL_CORE_ID_C66SS0;
		uint32_t remoteCoreId = CSL_CORE_ID_MAX;
		if (Mailbox_getRemoteCoreObj(selfCoreId, remoteCoreId)!= NULL)
		{
			testStatus2 = SystemP_FAILURE;
			DebugP_log("mailbox_pos_Test: failure on line no. %d \r\n", __LINE__);
		}
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

void negTest_readDone(void * args)
{
    int32_t testStatus = SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
    {
        if (Mailbox_readDone(0U)!= SystemP_FAILURE)
        {
            testStatus = SystemP_FAILURE;
			DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
        }
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void negTest_ReadwriteSystemNoWait(void * args)
{
    int32_t testStatus = SystemP_SUCCESS;
    int32_t testStatus1=SystemP_SUCCESS;
    int32_t testStatus2=SystemP_SUCCESS;
	uint32_t    size = 0U;
	uint32_t    timeToWaitInTicks = SystemP_NO_WAIT;
	uint32_t 	remoteCoreId = 1U;
	if (testStatus1 == SystemP_SUCCESS)
	{
		if (Mailbox_write(remoteCoreId,NULL,size,timeToWaitInTicks)!= SystemP_FAILURE)
		{
			testStatus1 = SystemP_FAILURE;
			DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
		}
	}
	if (testStatus2 == SystemP_SUCCESS)
	{
		if (Mailbox_read(remoteCoreId,NULL,size,timeToWaitInTicks)!= SystemP_FAILURE)
		{
			testStatus2 = SystemP_FAILURE;
			DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
		}
	}
	if ((testStatus1 == SystemP_SUCCESS) && (testStatus2 == SystemP_SUCCESS))
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

static void negTest_Write(void * args)
{
    int32_t testStatus = SystemP_SUCCESS;
	int32_t     testStatus1 = SystemP_SUCCESS;
    int32_t     testStatus2 = SystemP_SUCCESS;
    int32_t     testStatus3 = SystemP_SUCCESS;
    int32_t     testStatus4 = SystemP_SUCCESS;
	uint32_t    size = 0;
	uint8_t     *buffer = NULL ;
	uint32_t iterations = 10, i;
	uint32_t    timeToWaitInTicks = 0;

	if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_R5FSS0_0)
    {
        buffer = (uint8_t*)gMessage_r5fss0_0;
        size = sizeof(gMessage_r5fss0_0);
    }

    if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_C66SS0)
    {
        buffer = (uint8_t*)gMessage_c66ss0;
        size = sizeof(gMessage_c66ss0);
    }

	for(i=0; i<iterations; i++)
    {
        if (testStatus1 == SystemP_SUCCESS)
        {
            remoteCoreId = 10U;
            timeToWaitInTicks = SystemP_NO_WAIT;
            if (Mailbox_write(remoteCoreId,buffer,size,timeToWaitInTicks)!= SystemP_FAILURE)
            {
                testStatus1 = SystemP_FAILURE;
                DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
		    }
	    }

	    if (testStatus2 == SystemP_SUCCESS)
	    {
		    remoteCoreId = 1U;
		    timeToWaitInTicks = 0U;
		    size=100U;
		    if (Mailbox_write(remoteCoreId,buffer,size,timeToWaitInTicks)!= SystemP_FAILURE)
		    {
			    testStatus = SystemP_FAILURE;
			    DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
		    }
	    }

        if (testStatus3 == SystemP_SUCCESS)
	    {
            if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_R5FSS0_0)
            {
                buffer = (uint8_t*)gMessage_r5fss0_0;
                size = 0;
            }

            if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_C66SS0)
            {
            buffer = (uint8_t*)gMessage_c66ss0;
            size = 0;
            }
		    remoteCoreId = 3U;
		    timeToWaitInTicks = 0U;
		    if (Mailbox_write(remoteCoreId,buffer,size,timeToWaitInTicks)!= SystemP_FAILURE)
		    {
			    testStatus3 = SystemP_FAILURE;
			    DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
		    }
	    }

        if (testStatus4 == SystemP_SUCCESS)
	    {
            if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_R5FSS0_0)
            {
                buffer = (uint8_t*)gMessage_r5fss0_0;
                size = 0;
            }

            if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_C66SS0)
            {
            buffer = (uint8_t*)gMessage_c66ss0;
            size = 0;
            }
		    remoteCoreId = 10U;
		    timeToWaitInTicks = 0U;
		    if (Mailbox_write(remoteCoreId,buffer,size,timeToWaitInTicks)!= SystemP_FAILURE)
		    {
			    testStatus4 = SystemP_FAILURE;
			    DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
		    }
	    }
        if ((testStatus1 == SystemP_SUCCESS) && (testStatus2 == SystemP_SUCCESS) && (testStatus3 == SystemP_SUCCESS) && (testStatus4 == SystemP_SUCCESS))
        {
            testStatus = SystemP_SUCCESS;
        }
        else
        {
            testStatus = SystemP_FAILURE;
        }

        TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
    }
}

static void negTest_Read(void * args)
{
    int32_t testStatus = SystemP_SUCCESS;
	int32_t testStatus1 = SystemP_SUCCESS;
    int32_t testStatus2 = SystemP_SUCCESS;
    int32_t testStatus3 = SystemP_SUCCESS;
    int32_t testStatus4 = SystemP_SUCCESS;
	uint32_t    size = 0;
	uint8_t     *buffer = NULL ;
	uint32_t iterations = 10, i;
	uint32_t    timeToWaitInTicks = 0;

	if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_R5FSS0_0)
    {
        buffer = (uint8_t*)gMessage_r5fss0_0;
        size = sizeof(gMessage_r5fss0_0);
    }

    if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_C66SS0)
    {
        buffer = (uint8_t*)gMessage_c66ss0;
        size = sizeof(gMessage_c66ss0);
    }

	for(i=0; i<iterations; i++)
    {
	   if (testStatus1 == SystemP_SUCCESS)
	   {
		 remoteCoreId = 10U;
		 timeToWaitInTicks = SystemP_NO_WAIT;
		 if (Mailbox_read(remoteCoreId,buffer,size,timeToWaitInTicks)!= SystemP_FAILURE)
		 {
			testStatus1 = SystemP_FAILURE;
			DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
		 }
	   }

	   if (testStatus2 == SystemP_SUCCESS)
	   {
		 remoteCoreId = 1U;
		 timeToWaitInTicks = SystemP_NO_WAIT;
		 if (Mailbox_read(remoteCoreId,buffer,size,timeToWaitInTicks)!= SystemP_FAILURE)
		 {
			testStatus2 = SystemP_FAILURE;
			DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
		 }
	   }

       if (testStatus3 == SystemP_SUCCESS)
	    {
            if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_R5FSS0_0)
            {
                buffer = (uint8_t*)gMessage_r5fss0_0;
                size = 0;
            }

            if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_C66SS0)
            {
            buffer = (uint8_t*)gMessage_c66ss0;
            size = 0;
            }
		    remoteCoreId = 3U;
		    timeToWaitInTicks = 0U;
		    if (Mailbox_read(remoteCoreId,buffer,size,timeToWaitInTicks)!= SystemP_FAILURE)
		    {
			    testStatus3 = SystemP_FAILURE;
			    DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
		    }
	    }

        if (testStatus4 == SystemP_SUCCESS)
	    {
            if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_R5FSS0_0)
            {
                buffer = (uint8_t*)gMessage_r5fss0_0;
                size = 0;
            }

            if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_C66SS0)
            {
            buffer = (uint8_t*)gMessage_c66ss0;
            size = 0;
            }
		    remoteCoreId = 10U;
		    timeToWaitInTicks = 0U;
		    if (Mailbox_read(remoteCoreId,buffer,size,timeToWaitInTicks)!= SystemP_FAILURE)
		    {
			    testStatus4 = SystemP_FAILURE;
			    DebugP_log("mailbox_neg_Test: failure on line no. %d \r\n", __LINE__);
		    }
        }

	if ((testStatus1 == SystemP_SUCCESS) && (testStatus2 == SystemP_SUCCESS) && (testStatus3 == SystemP_SUCCESS) && (testStatus4 == SystemP_SUCCESS))
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
    }
}

void test_neg_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();

    UNITY_BEGIN();

    RUN_TEST(negTest_isCoreEnabled,5630,NULL);
    RUN_TEST(negTest_Params_init,5621,NULL);
    RUN_TEST(negTest_setReadCallback,5625,NULL);
    RUN_TEST(negTest_getRemoteCoreObj,6182,NULL);
    RUN_TEST(negTest_readCallback, 5619, NULL);
	RUN_TEST(negTest_readDone,5643,NULL);
	RUN_TEST(negTest_ReadwriteSystemNoWait,9028,NULL);
	RUN_TEST(negTest_Write,5636,NULL);
	RUN_TEST(negTest_Read,5642,NULL);
    UNITY_END();

    Drivers_close();
}

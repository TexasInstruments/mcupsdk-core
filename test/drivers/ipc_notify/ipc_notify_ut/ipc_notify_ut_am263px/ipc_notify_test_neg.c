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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <inttypes.h>
#include <drivers/soc.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/ipc_notify.h>
#include <drivers/ipc_notify/v1/ipc_notify_v1.h>
#include <unity.h>
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

void IpcNotify_syncCallback(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, void *args);

/* Negative test for IpcNotify_unregisterClient API */
void negTest_ipcNotifyUnregisterclient(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    uint16_t localClientId = 17U;

	if (testStatus == SystemP_SUCCESS)
    {
        if(IpcNotify_unregisterClient(localClientId) != SystemP_SUCCESS)
        {
            DebugP_log("ipc_notify_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test for IpcNotify_isCoreEnabled API */
void negTest_ipcNotifyisCoreEnabled(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    uint32_t coreId;
    if (testStatus == SystemP_SUCCESS)
    {
        coreId = 5U;
        if(IpcNotify_isCoreEnabled(coreId) != 0)
        {
            testStatus = SystemP_FAILURE;
             DebugP_log("ipc_notify_neg_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test for IpcNotify_waitSync API */
void negTest_ipcNotifywaitSync(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    int32_t teststatus1 = SystemP_SUCCESS;
    int32_t teststatus2 = SystemP_SUCCESS;
    uint32_t remoteCoreId;
    uint32_t timeout;

    if (teststatus1 == SystemP_SUCCESS)
    {
        remoteCoreId = 5U;
        timeout = 1000;
        if(IpcNotify_waitSync (remoteCoreId,timeout) != SystemP_FAILURE)

        {
            teststatus1 = SystemP_FAILURE;
            DebugP_log("ipc_notify_neg_Test: failure on line no. %d \n", __LINE__);
        }
    }
    if (teststatus2 == SystemP_SUCCESS)
    {
        remoteCoreId = 0U;
        timeout = 1000;
        if(IpcNotify_waitSync (remoteCoreId,timeout) != SystemP_FAILURE)
        {
            teststatus2 = SystemP_FAILURE;
            DebugP_log("ipc_notify_neg_Test: failure on line no. %d \n", __LINE__);
        }
    }
    if ((teststatus1 == SystemP_SUCCESS) && (teststatus2 == SystemP_SUCCESS))
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test for IpcNotify_sendMsg API with invalid remotecoreID */
void negTest_ipcNotifysendMsg(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    uint32_t remoteCoreId;
    uint32_t gServerClientId;
    uint32_t msgValue;
    uint32_t waitForFifoNotFull;

    if (testStatus == SystemP_SUCCESS)
    {
        remoteCoreId = 5U;
        gServerClientId = 4u;
        msgValue = 0;
        waitForFifoNotFull = 1U;
        if(IpcNotify_sendMsg (remoteCoreId,gServerClientId,msgValue,waitForFifoNotFull) != SystemP_FAILURE)

        {
            testStatus = SystemP_FAILURE;
             DebugP_log("ipc_notify_neg_Test: failure on line no. %d \n", __LINE__);
        }
    }
TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test for IpcNotify_sendMsg API with valid remotecoreID */
void negTest_ipcNotifysendMsgOne(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    uint32_t remoteCoreId = 1U;
    uint32_t gServerClientId = 4u;
    uint32_t msgValue = 0;
    uint32_t waitForFifoNotFull = 1U;
    if (testStatus == SystemP_SUCCESS)
    {
        if(IpcNotify_sendMsg(remoteCoreId,gServerClientId,msgValue,waitForFifoNotFull) != SystemP_FAILURE)
        {
            testStatus = SystemP_FAILURE;
             DebugP_log("ipc_notify_neg_Test: failure on line no. %d \n", __LINE__);
        }
    }
TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test for IpcNotify_sendMsg API with waitForFifo as zero */
void negTest_ipcNotifysendMsgWaitFifo(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    uint32_t remoteCoreId = (uint32_t)args;
    uint32_t gMsgEchoCount = 10000u;
    uint32_t i;
    uint32_t gNullClientId = 5u;
    for(i=0 ; i<gMsgEchoCount; i++)
    {
        /* send message to remote core which does have any registered handler,
           DO NOT wait for message to be put in HW FIFO */
        testStatus = IpcNotify_sendMsg(remoteCoreId, gNullClientId, 0, 0);
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, testStatus);
}

/* Negative test for IpcNotify_syncCallback API with invalid remotecoreID */
void negTest_ipcNotifysyncCallback(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    uint32_t remoteCoreId = (uint32_t)args;
    if (testStatus == SystemP_SUCCESS)
    {
        IpcNotify_syncCallback(remoteCoreId,20,0,NULL);
    }
    else
    {
            testStatus = SystemP_FAILURE;
             DebugP_log("ipc_notify_neg_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}


void test_neg_main(void *args)
{
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(negTest_ipcNotifyUnregisterclient, 9861, NULL);
    RUN_TEST(negTest_ipcNotifyisCoreEnabled, 9863, NULL);
    RUN_TEST(negTest_ipcNotifywaitSync, 9864, NULL);
    RUN_TEST(negTest_ipcNotifysendMsg, 9866, NULL);
    RUN_TEST(negTest_ipcNotifysendMsgOne, 9865, NULL);
    RUN_TEST(negTest_ipcNotifysendMsgWaitFifo, 10567, (void*)CSL_CORE_ID_R5FSS0_1);
    RUN_TEST(negTest_ipcNotifysyncCallback, 10568, (void*)CSL_CORE_ID_MAX);

    UNITY_END();
    Drivers_close();

}

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

/* Positive test case for IpcNotify_sendMsg API */
void posTest_ipcNotifysendMsg(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    uint32_t remoteCoreId = 1U;
    uint32_t gServerClientId = 4u;
    uint32_t msgValue = 0;
    uint32_t waitForFifoNotFull = 1U;

	if (testStatus == SystemP_SUCCESS)
    {
        if(IpcNotify_sendMsg(remoteCoreId,gServerClientId,msgValue,waitForFifoNotFull) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_notify_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test case for IpcNotify_waitSync API */
void posTest_ipcNotifywaitSync(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    uint32_t remoteCoreId = 1U;
    uint32_t timeout = 500;

   if (testStatus == SystemP_SUCCESS)
    {
        if(IpcNotify_waitSync(remoteCoreId,timeout) != SystemP_TIMEOUT)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_notify_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test case for IpcNotify_registerNonNotifyCallback API */
void posTest_ipcNotifyregisterNonNotifyCallback(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
       IpcNotify_NonNotifyCallback callback = NULL;
       IpcNotify_registerNonNotifyCallback(callback);
    }
    if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("ipc_notify_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test case for IpcNotify_init API */
void posTest_ipcNotifyinit(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    IpcNotify_Params notifyParams;
    IpcNotify_Params_init(&notifyParams);
    notifyParams.selfCoreId = 0;

    if (testStatus == SystemP_SUCCESS)
    {
        if(IpcNotify_init(&notifyParams) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("nortos_common_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test case for IpcNotify_sendMsg API */
void posTest_ipcNotifysendMsgOne(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    uint32_t remoteCoreId = 1U;
    uint32_t gServerClientId = 4u;
    uint32_t waitForFifoNotFull = 1;
    uint32_t i;
    uint32_t gmsgEchoCount = 4u;
    uint32_t msgValue;

    if (testStatus == SystemP_SUCCESS)
    {
        for(i=0 ; i<gmsgEchoCount; i++)
        {
            msgValue = i;
            if(IpcNotify_sendMsg(remoteCoreId,gServerClientId,msgValue,waitForFifoNotFull) != SystemP_SUCCESS)
            {
                testStatus = SystemP_FAILURE;
                DebugP_log("ipc_notify_pos_Test: failure on line no. %d \n", __LINE__);
            }
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test case for IpcNotify_syncAll API */
void posTest_IpcNotify_syncAll(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
    uint32_t timeout = 0;

   if (testStatus == SystemP_SUCCESS)
    {
        if(IpcNotify_syncAll(timeout) != SystemP_TIMEOUT)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_notify_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_pos_main(void *args)
{
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(posTest_ipcNotifysendMsg, 9855, NULL);
    RUN_TEST(posTest_ipcNotifywaitSync, 9856, NULL);
    RUN_TEST(posTest_ipcNotifyregisterNonNotifyCallback, 9857, NULL);
    RUN_TEST(posTest_ipcNotifysendMsgOne, 9858, NULL);
    RUN_TEST(posTest_IpcNotify_syncAll, 9859, NULL);
    RUN_TEST(posTest_ipcNotifyinit, 9860, NULL);

    UNITY_END();
    Drivers_close();
}


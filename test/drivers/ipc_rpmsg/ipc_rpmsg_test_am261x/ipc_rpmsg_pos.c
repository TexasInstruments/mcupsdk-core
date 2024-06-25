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
 *  \file     ipc_rpmsg_pos.c
 *
 *  \brief    This file contains IPC RPMsg API unit test code.
 *
 *  \details  IPC RP MSG unit tests
 **/
 #include <stdio.h>
 #include <string.h>
 #include <inttypes.h>
 #include <kernel/dpl/ClockP.h>
 #include <kernel/dpl/SemaphoreP.h>
 #include <drivers/soc.h>
 #include <drivers/ipc_notify.h>
 #include <drivers/ipc_rpmsg.h>
 #include <unity.h>
 #include "ti_drivers_open_close.h"
 #include <drivers/ipc_rpmsg/ipc_rpmsg_priv.h>

/* max size of message that will be ever sent */
#define MAX_MSG_SIZE    (128u)
uint16_t GServerEndPt = 10;
/* RPMessage objects to receive ack messages */
RPMessage_Object gClientMsgObject;
/* Temporary define for RPMsg send timeout(2 seconds) to fix code hang issue */
#define RPMSG_SEND_TIMEOUT (2 * 1000000)
/* All cores that participate in the IPC */
uint32_t GRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_0,
    CSL_CORE_ID_R5FSS0_1,
    2U,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};

typedef struct{

    uint32_t curCount;
    uint32_t maxCount;

}Msg_Back2Back;

RPMessage_Core *Core;
IpcRpmsg_Ctrl *msg_ctrl;
RPMessage_Struct *structT;
RPMessage_Object handle;

/*Test for dynamic coverage of RPMessage_destruct API */
void posTest_RPMessage_destruct(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    structT->localEndPt = 65U;

    if (testStatus == SystemP_SUCCESS)
    {
        RPMessage_destruct(&handle);
    }
    else{
        testStatus = SystemP_FAILURE;
        DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test for dynamic coverage of RPMessage_send API */
void posTest_RPMessage_send(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    Msg_Back2Back *data;
    uint16_t dataLen = 120;
    uint16_t remoteCoreId = 1U;
    uint16_t remoteEndPt = 10;
    uint16_t localEndPt = 12;

   if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_send(&data, dataLen, remoteCoreId, remoteEndPt, localEndPt, SystemP_WAIT_FOREVER) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test for dynamic coverage of RPMessage_send with remoteEndPt as RPMESSAGE_MAX_LOCAL_ENDPT */
void posTest_RPMessage_send_remoteEndpt(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    Msg_Back2Back *data;
    uint16_t dataLen = 120;
    uint16_t remoteCoreId = 1U;
    uint16_t remoteEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
    uint16_t localEndPt = 12;

   if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_send(&data, dataLen, remoteCoreId, remoteEndPt, localEndPt, SystemP_WAIT_FOREVER) == SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test for MCDC coverage of RPMessage_send with valid remoteCoreId */
void posTest_RPMessage_send_mcdcOne(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    RPMessage_Params rpmsgParams;
    RPMessage_Params_init(&rpmsgParams);
    Msg_Back2Back *data = NULL;
    uint16_t dataLen = 0;
    uint16_t remoteCoreId = 1U;
    uint16_t remoteEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
    uint16_t localEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;

    rpmsgParams.vringTxBaseAddr[remoteCoreId] = RPMESSAGE_VRING_ADDR_INVALID;
    RPMessage_init(&rpmsgParams);
    gIpcRpmsgCtrl.isCoreEnable[remoteCoreId] = 0;

   if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_send(&data, dataLen, remoteCoreId, remoteEndPt, localEndPt, SystemP_WAIT_FOREVER) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_FAILURE);
}

/*Test for MCDC coverage of RPMessage_send with valid remoteCoreId and isCoreEnable*/
void posTest_RPMessage_send_mcdcTwo(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    Msg_Back2Back *data = NULL;
    uint16_t dataLen = 0;
    uint16_t remoteCoreId = 1U;
    uint16_t remoteEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
    uint16_t localEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
    gIpcRpmsgCtrl.isCoreEnable[remoteCoreId] = 1;

   if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_send(&data, dataLen, remoteCoreId, remoteEndPt, localEndPt, SystemP_WAIT_FOREVER) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_FAILURE);
}

/*Test for MCDC coverage of RPMessage_send with valid remoteCoreId, data and isCoreEnable*/
void posTest_RPMessage_send_mcdcThree(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    Msg_Back2Back *data;
    uint16_t dataLen = 0;
    uint16_t remoteCoreId = 1U;
    uint16_t remoteEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
    uint16_t localEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
    gIpcRpmsgCtrl.isCoreEnable[remoteCoreId] = 1;

   if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_send(&data, dataLen, remoteCoreId, remoteEndPt, localEndPt, SystemP_WAIT_FOREVER) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_FAILURE);
}

/*Test for MCDC coverage of RPMessage_send with valid remoteCoreId, data, dataLen and isCoreEnable*/
void posTest_RPMessage_send_mcdcFour(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    Msg_Back2Back *data;
    uint16_t dataLen = 120;
    uint16_t remoteCoreId = 1U;
    uint16_t remoteEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
    uint16_t localEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
    gIpcRpmsgCtrl.isCoreEnable[remoteCoreId] = 1;

   if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_send(&data, dataLen, remoteCoreId, remoteEndPt, localEndPt, SystemP_WAIT_FOREVER) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_FAILURE);
}

/*Test for MCDC coverage of RPMessage_send with valid remoteCoreId, data, dataLen,remoteEndPt and isCoreEnable*/
void posTest_RPMessage_send_mcdcFive(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    Msg_Back2Back *data;
    uint16_t dataLen = 120;
    uint16_t remoteCoreId = 1U;
    uint16_t remoteEndPt = 11;
    uint16_t localEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
    gIpcRpmsgCtrl.isCoreEnable[remoteCoreId] = 1;

   if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_send(&data, dataLen, remoteCoreId, remoteEndPt, localEndPt, SystemP_WAIT_FOREVER) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_FAILURE);
}

/*Test for MCDC coverage of RPMessage_send with invalid remoteCoreId */
void posTest_RPMessage_send_mcdcSix(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;
    Msg_Back2Back *data;
    uint16_t dataLen = 120;
    uint16_t remoteCoreId = 5U;
    uint16_t remoteEndPt = 11;
    uint16_t localEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
    gIpcRpmsgCtrl.isCoreEnable[remoteCoreId] = 1;

   if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_send(&data, dataLen, remoteCoreId, remoteEndPt, localEndPt, SystemP_WAIT_FOREVER) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_FAILURE);
}

void test_pos_main(void *args)
{
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(posTest_RPMessage_send, 10008, NULL);
    RUN_TEST(posTest_RPMessage_send_remoteEndpt, 10617, NULL);
    RUN_TEST(posTest_RPMessage_destruct, 10009, NULL);
    RUN_TEST(posTest_RPMessage_send_mcdcOne, 10618, NULL);
    RUN_TEST(posTest_RPMessage_send_mcdcTwo, 10619, NULL);
    RUN_TEST(posTest_RPMessage_send_mcdcThree, 10620, NULL);
    RUN_TEST(posTest_RPMessage_send_mcdcFour, 10621, NULL);
    RUN_TEST(posTest_RPMessage_send_mcdcFive, 10622, NULL);
    RUN_TEST(posTest_RPMessage_send_mcdcSix, 10623, NULL);

    UNITY_END();
    Drivers_close();
}

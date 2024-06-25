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
 *  \file     ipc_rpmsg_neg.c
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

void RPMessage_forceRecvMsgHandlers(void);
void RPMessage_notifyCallback(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args);
void RPMessage_controlEndPtHandler(RPMessage_Object *obj, void *arg, void *data, uint16_t dataLen, int32_t crcStatus, uint16_t remoteCoreId, uint16_t remoteEndPt);
void RPMessage_recvHandler(uint32_t remoteCoreId) ;

typedef struct{

    uint32_t curCount;
    uint32_t maxCount;

}Msg_Back2Back;

Msg_Back2Back data;

RPMessage_Core *Core;

extern IpcRpmsg_Ctrl gIpcRpmsgCtrl;

void ControlEndPtCallback(void * arg, uint16_t remoteCoreId, uint16_t remoteEndPt , const char * remoteServiceName )
{
    RPMessage_send(&data , 4 , 1 , 14 , 15 , SystemP_WAIT_FOREVER);
}

/*negative test case for RPMessage_recv API*/
void negTest_RPMessage_recv(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    RPMessage_Object *obj = NULL;
    Msg_Back2Back *data;
    uint16_t dataLen = 1U;
    uint16_t remoteCoreId = 1U;
    uint16_t remoteEndPt = 10;

   if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_recv(obj, &data, &dataLen, &remoteCoreId, &remoteEndPt, SystemP_WAIT_FOREVER) == SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*negative test case for RPMessage_recv API with invalid datalength*/
 void negTest_RPMessage_recv_dataLen(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    RPMessage_Object *obj = NULL;
    Msg_Back2Back *data;
    uint16_t dataLen = 0;
    uint16_t remoteCoreId = 1U;
    uint16_t remoteEndPt = 10;

   if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_recv(obj, &data, &dataLen, &remoteCoreId, &remoteEndPt, 0) == SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*negative test case for RPMessage_recv API with invalid remoteEndPt*/
 void negTest_RPMessage_recv_remoteEndPt(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    RPMessage_Object *obj = NULL;
    Msg_Back2Back *data;
    uint16_t dataLen = 120;
    uint16_t remoteCoreId = 1U;
    uint16_t remoteEndPt = 0U;

   if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_recv(obj, &data, &dataLen, &remoteCoreId, &remoteEndPt, 0) == SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*negative test case for RPMessage_send API */
void negTest_RPMessage_send(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    static char msgBuf[128u];
    uint16_t remoteCoreId,msgSize;
    msgSize=0U;
    remoteCoreId=0U;
    gIpcRpmsgCtrl.isCoreEnable[remoteCoreId] = 0;

    if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_send(
                    msgBuf, msgSize,
                    remoteCoreId, 10,
                    12,
                    SystemP_WAIT_FOREVER) == SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*negative test case to set controlEndPtCallback as null for RPMessage_controlEndPtCallback*/
void negTest_RPMessage_controlEndPtCallback(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
        RPMessage_controlEndPtCallback(NULL, NULL);
    }
    else
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*negative test case for RPMessage_controlEndPtHandler with data length as 0*/
void negTest_RPMessage_controlEndPtHandler_dataLen(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    Msg_Back2Back *data;
    uint16_t remoteCoreId;
    RPMessage_Object obj;
    remoteCoreId=1U;

    if (testStatus == SystemP_SUCCESS)
    {
        RPMessage_controlEndPtHandler(&obj, NULL,
        &data, 0, SystemP_SUCCESS,
        remoteCoreId, 10);
    }
    else
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*negative test case to set controlEndPtCallback as null in RPMessage_controlEndPtHandler API */
void negTest_RPMessage_controlEndPtHandler(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    Msg_Back2Back *data;
    uint16_t remoteCoreId;
    RPMessage_Object obj;
    remoteCoreId=1U;
    gIpcRpmsgCtrl.controlEndPtCallback = 0;

    if (testStatus == SystemP_SUCCESS)
    {
        RPMessage_controlEndPtHandler(&obj, NULL,
        &data, 120, SystemP_SUCCESS,
        remoteCoreId, 10);
    }
    else
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*negative test case to set data length less the size of RPMessage_AnnounceMsg in RPMessage_controlEndPtHandler API */
void negTest_RPMessage_controlEndPtHandler_dataLen_Announce(void * args )
{
    int32_t testStatus = SystemP_SUCCESS ;
    Msg_Back2Back *data ;
    uint16_t remoteCoreId , dataLen;
    RPMessage_Object obj ;
    remoteCoreId = 1U ;
    gIpcRpmsgCtrl.controlEndPtCallback = ControlEndPtCallback ;
    dataLen = sizeof ( RPMessage_AnnounceMsg ) - 1U ;
    if(testStatus == SystemP_SUCCESS)
    {
        RPMessage_controlEndPtHandler ( & obj , NULL , &data, dataLen , SystemP_SUCCESS, remoteCoreId , 10);
    }
    else
    {
        testStatus = SystemP_FAILURE ;
        DebugP_log ( "ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32 ( testStatus , SystemP_SUCCESS ) ;
}

/*negative test case for RPMessage_send API with local and remote end point set to max*/
void negTest_RPMessage_send_localendpt(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    char msgBuf[64U];
    uint16_t remoteCoreId, msgSize;
    msgSize = strlen(msgBuf) + 1;
    remoteCoreId=1U;

    if (testStatus == SystemP_SUCCESS)
    {
        if(RPMessage_send(
                    msgBuf, msgSize,
                    remoteCoreId, RPMESSAGE_MAX_LOCAL_ENDPT,
                    RPMESSAGE_MAX_LOCAL_ENDPT,
                    SystemP_WAIT_FOREVER) == SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* negative test to cover the MCDC condition of RPMessage_notifyCallback with isCoreEnable as true
and isCoreInitialized as false*/
void negTest_RPMessage_notifyCallback(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    uint16_t remoteCoreId;
    remoteCoreId=0U;
    gIpcRpmsgCtrl.isCoreEnable[remoteCoreId] = 1;
    gIpcRpmsgCtrl.isCoreInitialized[remoteCoreId] = 0;

    if (testStatus == SystemP_SUCCESS)
    {
        RPMessage_notifyCallback(remoteCoreId, IPC_NOTIFY_CLIENT_ID_RPMSG, RPMESSAGE_MSG_VRING_NEW_FULL, SystemP_SUCCESS, NULL);
    }
    else
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* negative test to cover the MCDC condition of RPMessage_notifyCallback with remote core ID
set to self core ID*/
void negTest_RPMessage_notifyCallback_mcdcOne(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    uint16_t remoteCoreId;
    remoteCoreId = gIpcRpmsgCtrl.selfCoreId;
    RPMessage_deInit();

    if (testStatus == SystemP_SUCCESS)
    {
        RPMessage_notifyCallback(remoteCoreId, 10, 4, SystemP_SUCCESS, NULL);
    }
    else
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* negative test to cover the MCDC condition of RPMessage_notifyCallback with isCoreEnable as false
and isCoreInitialized as true*/
void negTest_RPMessage_notifyCallback_mcdcTwo(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    uint16_t remoteCoreId;
    remoteCoreId=1U;
    RPMessage_deInit();
    gIpcRpmsgCtrl.isCoreEnable[remoteCoreId] = 0;
    gIpcRpmsgCtrl.isCoreInitialized[remoteCoreId] = 1;

    if (testStatus == SystemP_SUCCESS)
    {
        RPMessage_notifyCallback(remoteCoreId, 11, 4, SystemP_SUCCESS, NULL);
    }
    else
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}


/*Negative test case for dynamic coverage of vringTxBaseAddr as invalid */
void negTest_RPMessage_initOne(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    RPMessage_Params rpmsgParams;
    RPMessage_Params_init(&rpmsgParams);
    rpmsgParams.vringRxBaseAddr[CSL_CORE_ID_R5FSS0_1] = 0xFFFFFFFFU;
    rpmsgParams.vringTxBaseAddr[CSL_CORE_ID_R5FSS0_0] = 0xFFFFFFFAU;

    if (testStatus == SystemP_SUCCESS)
    {
        if( RPMessage_init(&rpmsgParams) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
            DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Negative test case for RPMessage_forceRecvMsgHandlers */
void negTest_RPMessage_forceRecvMsgHandlers(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
        RPMessage_forceRecvMsgHandlers();
    }
    else
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Negative test case for RPMessage_destruct */
void negTest_RPMessage_destruct(void *args)
{
    int32_t testStatus = SystemP_SUCCESS;
    gIpcRpmsgCtrl.localEndPtObj[0U] = (RPMessage_Struct *)64U;
    if (testStatus == SystemP_SUCCESS)
    {
        RPMessage_destruct(&gIpcRpmsgCtrl.controlEndPtObj);
    }
    else
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Negative test case for RPMessage_allocEndPtMsg to set pMsg as null */
void negTest_RPMessage_allocEndPtMsg(void * args)
{
    int32_t testStatus = SystemP_SUCCESS ;
    uint32_t remotecoreID = 1U ;
    RPMessage_deInit () ;
    if(testStatus == SystemP_SUCCESS)
    {
        RPMessage_recvHandler(remotecoreID) ;
    }
    else
    {
        testStatus = SystemP_FAILURE ;
        DebugP_log ( "ipc_rpmsg_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32 ( testStatus , SystemP_SUCCESS ) ;
}

void test_neg_main(void *args)
{
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(negTest_RPMessage_recv, 10010, NULL);
    RUN_TEST(negTest_RPMessage_recv_remoteEndPt, 10655, NULL);
    RUN_TEST(negTest_RPMessage_recv_dataLen, 10654, NULL);
    RUN_TEST(negTest_RPMessage_send, 10656, NULL);
    RUN_TEST(negTest_RPMessage_controlEndPtHandler_dataLen_Announce, 10664, NULL);
    RUN_TEST(negTest_RPMessage_controlEndPtCallback, 10657, NULL);
    RUN_TEST(negTest_RPMessage_controlEndPtHandler_dataLen, 10665, NULL);
    RUN_TEST(negTest_RPMessage_controlEndPtHandler, 10666, NULL);
    RUN_TEST(negTest_RPMessage_notifyCallback, 10667, NULL);
    RUN_TEST(negTest_RPMessage_initOne, 10668, NULL);
    RUN_TEST(negTest_RPMessage_forceRecvMsgHandlers, 10669, NULL);
    RUN_TEST(negTest_RPMessage_send_localendpt, 10670, NULL);
    RUN_TEST(negTest_RPMessage_destruct, 10671, NULL);
    RUN_TEST(negTest_RPMessage_allocEndPtMsg, 10672, NULL);
    RUN_TEST(negTest_RPMessage_notifyCallback_mcdcOne, 10673, NULL);
    RUN_TEST(negTest_RPMessage_notifyCallback_mcdcTwo, 10674, NULL);

    UNITY_END();
    Drivers_close();
}

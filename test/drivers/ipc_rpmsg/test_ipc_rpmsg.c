/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/soc.h>
#include <drivers/ipc_notify.h>
#include <drivers/ipc_rpmsg.h>
#include <FreeRTOS.h>
#include <task.h>
#include <unity.h>
#include "ti_drivers_open_close.h"

/* number of iterations of message exchange to do, this is only used by some tests */
uint32_t gMsgEchoCount = 10000;

#if defined(SOC_AM64X)
/* main core that checks the test pass/fail */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* All cores that participate in the IPC */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_0,
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_R5FSS1_0,
    CSL_CORE_ID_R5FSS1_1,
    CSL_CORE_ID_M4FSS0_0,
    CSL_CORE_ID_A53SS0_0,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#if defined(SOC_AM243X)
/* main core that checks the test pass/fail */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* All cores that participate in the IPC */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_0,
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_R5FSS1_0,
    CSL_CORE_ID_R5FSS1_1,
    CSL_CORE_ID_M4FSS0_0,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#if defined(SOC_AM263X)
/* main core that checks the test pass/fail */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* All cores that participate in the IPC */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_0,
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_R5FSS1_0,
    CSL_CORE_ID_R5FSS1_1,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#if defined(SOC_AM273X) || defined(SOC_AWR294X)
/* main core that checks the test pass/fail */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* All cores that participate in the IPC */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_0,
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_C66SS0,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

/* max size of message that will be ever sent */
#define MAX_MSG_SIZE    (128u)

/* name of server that is annoucned */
#define SERVER_NAME "rpmsg.server"

/* Temporary define for RPMsg send timeout(2 seconds) to fix code hang issue */
#define RPMSG_SEND_TIMEOUT (2 * 1000000)

/* semaphore that is set from callback handler when all sent messages in back to back mode are ack'ed */
SemaphoreP_Object gAckDoneSem;
/* semaphore that is set from callback handler when all sent messages in rx notify callback mode are ack'ed */
SemaphoreP_Object gRxNotifyAckDoneSem;

/* server task related properties, like priority, stack size, stack memory, task object handles */
#define SERVER_TASK_PRI (2u)
#define SERVER_TASK_SIZE (16*1024/sizeof(StackType_t))
StackType_t  gServerTaskStack[SERVER_TASK_SIZE] __attribute__((aligned(32)));
StaticTask_t gServerTaskObj;
TaskHandle_t gServerTask;

/* RPMessage objects to receive messages */
RPMessage_Object gServerMsgObject;
/* RPMessage objects to receive ack messages */
RPMessage_Object gClientMsgObject;
/* RPMessage objects to receive ack messages in back to back mode */
RPMessage_Object gAckMsgObject;
/* No messages are sent or received to this object, its used for error check tests */
RPMessage_Object gNullRpmsgObj;
/* RPMessage objects to receive ack messages in rx notify callback mode */
RPMessage_Object gRxNotifyAckMsgObject;

/* RPMessage end points for server, server acks, server acks in back to back mode */
uint16_t gServerEndPt = 10;
uint16_t gClientEndPt = 11;
uint16_t gAckEndPt    = 12;
uint16_t gNullEndPt   = 13; /* this end point is not created is used for error tests */
uint16_t gRxNotifyAckEndPt = 14;

/* one to one test args */
typedef struct {
    uint16_t remoteCoreId; /* core to test message exchange with */
    uint16_t msgSize;   /* size of message to exchange */
    uint32_t echoMsgCount; /* number of messages to exchange */
} Test_Args;

/* message that is sent in back to back test */
typedef struct {

    uint32_t curCount;
    uint32_t maxCount;

} Msg_BackToBack;

typedef struct {

    uint16_t remoteEndPt;
    char remoteServiceName[32];

} ControlEndPt_Info;

ControlEndPt_Info gControlEndPt_info[CSL_CORE_ID_MAX];

/* Performance is calculated for 4 msg lengths (4,32,64,112) and from all remote cores */
#define MAX_IPC_RPMSG_PERF_CNT      ((CSL_CORE_ID_MAX-1) * 4)
typedef struct ipcPerfObj_s {
uint32_t remoteCoreId;
uint32_t msgSize;
uint64_t msgLatency;
} ipcPerfObj_t;
ipcPerfObj_t gIpcPerfObj[MAX_IPC_RPMSG_PERF_CNT] = {0};
uint32_t     gIpcPerfCnt = 0;

void test_rpmsgRxNotifyHandler(RPMessage_Object *obj, void *arg);

/* handle announcement messages and store in a global, these are checked later on */
void test_rpmsgControlEndPtCallback(void *arg,
    uint16_t remoteCoreId, uint16_t remoteEndPt, const char *remoteServiceName)
{
    ControlEndPt_Info *obj = (ControlEndPt_Info *)arg;

    if(remoteCoreId < CSL_CORE_ID_MAX)
    {
        obj[remoteCoreId].remoteEndPt = remoteEndPt;
        strncpy(obj[remoteCoreId].remoteServiceName, remoteServiceName, 32);
    }
}

/* Ack message handler when messages are sent back to back, here after required messages are received semaphore is posted */
void test_rpmsgAckHandler(RPMessage_Object *obj, void *arg, void *data, uint16_t dataLen, uint16_t remoteCoreId, uint16_t remoteEndPt)
{
    Msg_BackToBack *pMsg = (Msg_BackToBack*)data;

    if(pMsg->curCount == (pMsg->maxCount-1) )
    {
        SemaphoreP_Object *pDoneSem = (SemaphoreP_Object *)arg;

        SemaphoreP_post(pDoneSem);
    }
}


/* server task which simply echos the receive message back to the sender */
void test_rpmsgServerMain(void *args)
{
    int32_t status;
    static char recvMsg[MAX_MSG_SIZE];
    uint16_t recvMsgSize, remoteCoreId, remoteCoreEndPt;

    /* wait for messages forever in a loop */
    while(1)
    {
        /* set 'recvMsgSize' to size of recv buffer,
        * after return `recvMsgSize` contains actual size of valid data in recv buffer
        */
        recvMsgSize = sizeof(recvMsg);
        status = RPMessage_recv(&gServerMsgObject,
            recvMsg, &recvMsgSize,
            &remoteCoreId, &remoteCoreEndPt,
            SystemP_WAIT_FOREVER);
        DebugP_assert(status==SystemP_SUCCESS);

        /* echo the same message as reply */

        /* send ack to sender CPU at the sender end point */
        status = RPMessage_send(
            recvMsg, recvMsgSize,
            remoteCoreId, remoteCoreEndPt,
            RPMessage_getLocalEndPt(&gServerMsgObject),
            SystemP_WAIT_FOREVER);
        DebugP_assert(status==SystemP_SUCCESS);
    }
    /* This loop will never exit */
}

/* create semaphores, rpmessage objects and tasks as needed */
void test_rpmsgCreateObjects()
{
    int32_t status;
    RPMessage_CreateParams createParams;

    status = SemaphoreP_constructBinary(&gAckDoneSem, 0);
    DebugP_assert(status==SystemP_SUCCESS);

    status = SemaphoreP_constructBinary(&gRxNotifyAckDoneSem, 0);
    DebugP_assert(status==SystemP_SUCCESS);

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = gServerEndPt;
    status = RPMessage_construct(&gServerMsgObject, &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = gAckEndPt;
    createParams.recvCallback = test_rpmsgAckHandler;
    createParams.recvCallbackArgs = &gAckDoneSem;
    status = RPMessage_construct(&gAckMsgObject, &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = gRxNotifyAckEndPt;
    createParams.recvNotifyCallback = test_rpmsgRxNotifyHandler;
    createParams.recvNotifyCallbackArgs = &gRxNotifyAckDoneSem;
    status = RPMessage_construct(&gRxNotifyAckMsgObject, &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = gClientEndPt;
    status = RPMessage_construct(&gClientMsgObject, &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    memset(&gControlEndPt_info[0], 0, sizeof(ControlEndPt_Info));
    RPMessage_controlEndPtCallback(test_rpmsgControlEndPtCallback, &gControlEndPt_info[0]);

    gServerTask = xTaskCreateStatic(test_rpmsgServerMain,
                                  "test_rpmsgServerMain",
                                  SERVER_TASK_SIZE,
                                  NULL,
                                  SERVER_TASK_PRI,
                                  gServerTaskStack,
                                  &gServerTaskObj);
    configASSERT(gServerTask != NULL);

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("[TEST IPC RPMSG] Ready for IPC !!!\r\n");
}

/* delete all objects created in test_rpmsgCreateObjects
 * Note, on core other than main cores the server task never exists so objects are never deleted
 */
void test_rpmsgDestructObjects()
{
    vTaskDelete(gServerTask);
    RPMessage_destruct(&gServerMsgObject);
    RPMessage_destruct(&gAckMsgObject);
    RPMessage_destruct(&gRxNotifyAckMsgObject);
    RPMessage_destruct(&gClientMsgObject);
    SemaphoreP_destruct(&gAckDoneSem);
    SemaphoreP_destruct(&gRxNotifyAckDoneSem);
}

void test_rpmsgControlEndPt(void *args)
{
    uint32_t remoteCoreId, i;

    /* wait a while we should definitely get annoucments within this time */
    ClockP_usleep(10*1000);

    /* validate server end pt annoucements */
    for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++ )
    {
        remoteCoreId = gRemoteCoreId[i];
        if(remoteCoreId != IpcNotify_getSelfCoreId())
        {
            TEST_ASSERT_EQUAL_UINT16(gServerEndPt, gControlEndPt_info[remoteCoreId].remoteEndPt);
            TEST_ASSERT_EQUAL_UINT32( 0, strcmp(gControlEndPt_info[remoteCoreId].remoteServiceName, SERVER_NAME) );
        }
    }
}

void test_rpmsgAnyToAny(void *args)
{
    uint32_t msg, i;
    static char msgBuf[MAX_MSG_SIZE];
    static char ackMsgBuf[MAX_MSG_SIZE];
    int32_t status;
    uint16_t remoteCoreId, remoteCoreEndPt, msgSize, ackMsgSize;

    for(msg=0; msg<gMsgEchoCount; msg++)
    {
        snprintf(msgBuf, MAX_MSG_SIZE-1, "%d", msg);
        msgBuf[MAX_MSG_SIZE-1] = 0;
        msgSize = strlen(msgBuf) + 1; /* count the terminating char as well */

        /* send the same message to all cores */
        for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++ )
        {
            if(gRemoteCoreId[i] != IpcNotify_getSelfCoreId())
            {
                /*
                 * Temporary fix for code hang issue.
                 * Replaced "SystemP_WAIT_FOREVER" with 2 second timeout.
                 */
                status = RPMessage_send(
                    msgBuf, msgSize,
                    gRemoteCoreId[i], gServerEndPt,
                    RPMessage_getLocalEndPt(&gClientMsgObject),
                    ClockP_usecToTicks(RPMSG_SEND_TIMEOUT));
                TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
            }
        }
        /* wait for response from all cores */
        for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++ )
        {
            if(gRemoteCoreId[i] != IpcNotify_getSelfCoreId())
            {
                /* set 'ackMsgSize' to size of recv buffer,
                * after return `msgSize` contains actual size of valid data in recv buffer
                */
                ackMsgSize = sizeof(ackMsgBuf);
                ackMsgBuf[0] = 0;
                status = RPMessage_recv(&gClientMsgObject,
                    ackMsgBuf, &ackMsgSize,
                    &remoteCoreId, &remoteCoreEndPt,
                    SystemP_WAIT_FOREVER);
                TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
                TEST_ASSERT_EQUAL_UINT16(gServerEndPt, remoteCoreEndPt);
                TEST_ASSERT_EQUAL_UINT16(msgSize, ackMsgSize);
                TEST_ASSERT_EQUAL_INT32( 0, strcmp(ackMsgBuf, msgBuf));
            }
        }
    }

    /* wait for all cores to have executed upto this point
     * if we dont get sync from cores then message exchange between some core's has failed
     */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);
}

/* message exchange with one core, this is used to measure message exchange latency */
void test_rpmsgOneToOne(void *args)
{
    Test_Args *pTestArgs = (Test_Args*)args;
    uint16_t remoteCoreId = pTestArgs->remoteCoreId;
    uint16_t msgSize = pTestArgs->msgSize;
    uint32_t echoMsgCount = pTestArgs->echoMsgCount;
    uint64_t curTime;
    uint32_t msg;
    static char msgBuf[MAX_MSG_SIZE];
    static char ackMsgBuf[MAX_MSG_SIZE];
    int32_t status;
    uint16_t remoteCoreEndPt, ackMsgSize;

    TEST_ASSERT_LESS_OR_EQUAL_UINT16(MAX_MSG_SIZE, msgSize);

    /* fill with known data, we dont check data integrity in this test since
     * we want to focus on performance here
     *
     * Data integrity is tested in test_rpmsgAnyToAny
     */
    memset(msgBuf, 0xAA, MAX_MSG_SIZE);

    curTime = ClockP_getTimeUsec();

    for(msg=0; msg<echoMsgCount; msg++)
    {
        status = RPMessage_send(
            msgBuf, msgSize,
            remoteCoreId, gServerEndPt,
            RPMessage_getLocalEndPt(&gClientMsgObject),
            SystemP_WAIT_FOREVER);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

        ackMsgSize = sizeof(ackMsgBuf);
        status = RPMessage_recv(&gClientMsgObject,
            ackMsgBuf, &ackMsgSize,
            &remoteCoreId, &remoteCoreEndPt,
            SystemP_WAIT_FOREVER);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        TEST_ASSERT_EQUAL_UINT16(msgSize, ackMsgSize);
    }

    curTime = ClockP_getTimeUsec() - curTime;

    gIpcPerfObj[gIpcPerfCnt].remoteCoreId = remoteCoreId;
    gIpcPerfObj[gIpcPerfCnt].msgSize = msgSize;
    gIpcPerfObj[gIpcPerfCnt].msgLatency = curTime;
    gIpcPerfCnt++;
    DebugP_assert(gIpcPerfCnt < MAX_IPC_RPMSG_PERF_CNT);

}

/* In this test
    - we do message exchange with one core,
    - we send message back to back without waiting for ack
    - ack received in the callback mode
    - this tests fifo full and wait conditions
    - and test also callback mode of IPC
 */
void test_rpmsgOneToOneBackToBack(void *args)
{
    Test_Args *pTestArgs = (Test_Args*)args;
    uint16_t remoteCoreId = pTestArgs->remoteCoreId;
    uint32_t echoMsgCount = pTestArgs->echoMsgCount;
    uint64_t curTime;
    uint32_t msg, oldDebugLogZone;
    int32_t status;

    /* disable warning logs since we are testing for those, so it will clutter the output */
    oldDebugLogZone = DebugP_logZoneDisable(DebugP_LOG_ZONE_WARN);

    curTime = ClockP_getTimeUsec();

    for(msg=0; msg<echoMsgCount; msg++)
    {
        Msg_BackToBack msgObj;

        msgObj.curCount = msg;
        msgObj.maxCount = echoMsgCount;
        status = RPMessage_send(
            &msgObj, sizeof(msgObj),
            remoteCoreId, gServerEndPt,
            RPMessage_getLocalEndPt(&gAckMsgObject),
            SystemP_WAIT_FOREVER);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    }
    status = SemaphoreP_pend(&gAckDoneSem, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_logZoneRestore(oldDebugLogZone);

    DebugP_log("[TEST IPC RPMSG] Messages sent = %d, remote core = %s \r\n",
                    echoMsgCount, SOC_getCoreName(remoteCoreId));
    DebugP_log("[TEST IPC RPMSG] Total execution time = %" PRId64 " usecs\r\n", curTime);
    DebugP_log("[TEST IPC RPMSG] Avg one-way message latency = %" PRId32 " nsec\r\n",
        (uint32_t)(curTime*1000u/(echoMsgCount*2)));
}

/* Ack message handler when messages are sent with rx notify handler registered, here after required messages are received semaphore is posted */
void test_rpmsgRxNotifyHandler(RPMessage_Object *obj, void *arg)
{
    Msg_BackToBack msg;
    uint16_t dataLen = sizeof(msg);
    uint16_t remoteCoreId, remoteEndPt;
    int32_t status;

    status = RPMessage_recv(obj, &msg, &dataLen, &remoteCoreId, &remoteEndPt, 0);
    DebugP_assertNoLog(status == SystemP_SUCCESS);
    DebugP_assertNoLog(dataLen == sizeof(Msg_BackToBack));
    DebugP_assertNoLog(remoteEndPt == gServerEndPt);

    if(msg.curCount == (msg.maxCount-1) )
    {
        SemaphoreP_Object *pDoneSem = (SemaphoreP_Object *)arg;

        SemaphoreP_post(pDoneSem);
    }
}

/* In this test
    - we do message exchange with one core,
    - we use rx notify callback and see if it is getting invoked
    - we also test calling RPMessage_recv within the callback itself
 */
void test_rpmsgRxNotifyCallback(void *args)
{
    Test_Args *pTestArgs = (Test_Args*)args;
    uint16_t remoteCoreId = pTestArgs->remoteCoreId;
    uint32_t echoMsgCount = pTestArgs->echoMsgCount;
    uint32_t msg;
    int32_t status;

    for(msg=0; msg<echoMsgCount; msg++)
    {
        Msg_BackToBack msgObj;

        msgObj.curCount = msg;
        msgObj.maxCount = echoMsgCount;
        status = RPMessage_send(
            &msgObj, sizeof(msgObj),
            remoteCoreId, gServerEndPt,
            RPMessage_getLocalEndPt(&gRxNotifyAckMsgObject),
            SystemP_WAIT_FOREVER);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    }

    status = SemaphoreP_pend(&gRxNotifyAckDoneSem, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

}

void test_rpmsgErrorChecks(void *args)
{
    int32_t status;
    uint32_t msg, fifoFullCount, msgCount, oldDebugLogZone;
    uint16_t msgSize, remoteCoreId, remoteEndPt;
    RPMessage_CreateParams rpmsgPrm;
    Msg_BackToBack msgObj;

    /* disable error and warning logs since we are testing for those, so it will clutter the output */
    oldDebugLogZone = DebugP_logZoneDisable(DebugP_LOG_ZONE_WARN | DebugP_LOG_ZONE_ERROR);

    /* message to send, set values such ack callback wont post a semaphore */
    msgObj.curCount = 0;
    msgObj.maxCount = 10;

    /* invalid local end point */
    RPMessage_CreateParams_init(&rpmsgPrm);
    rpmsgPrm.localEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
    status = RPMessage_construct(&gNullRpmsgObj, &rpmsgPrm);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    /* already created end point */
    RPMessage_CreateParams_init(&rpmsgPrm);
    rpmsgPrm.localEndPt = gServerEndPt;
    status = RPMessage_construct(&gNullRpmsgObj, &rpmsgPrm);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    /* annouce to invalid core ID */
    status = RPMessage_announce(CSL_CORE_ID_MAX, 0, "test");
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    /* send to invalid core ID */
    status = RPMessage_send(&msgObj, sizeof(msgObj), CSL_CORE_ID_MAX,
                    gServerEndPt, gAckEndPt, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    /* send with NULL data */
    status = RPMessage_send(NULL, sizeof(msgObj), CSL_CORE_ID_R5FSS0_1,
                    gServerEndPt, gAckEndPt, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    /* send with zero size data  */
    status = RPMessage_send(&msgObj, 0, CSL_CORE_ID_R5FSS0_1,
                    gServerEndPt, gAckEndPt, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    /* send to end pt that is not created on remote,
       this is allowed, recevied message is dropped */
    status = RPMessage_send(&msgObj, sizeof(msgObj), CSL_CORE_ID_R5FSS0_1,
                    gNullEndPt, gAckEndPt, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    /* set reply to end pt that is not created on local,
       this is allowed, ack message is dropped */
    status = RPMessage_send(&msgObj, sizeof(msgObj), CSL_CORE_ID_R5FSS0_1,
                    gServerEndPt, gNullEndPt, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* send back to back messages without waiting, there should be timeout conditon
       due to queue full many times
     */
    fifoFullCount = 0;
    msgCount = 1000;
    for(msg=0; msg<msgCount; msg++)
    {
        status = RPMessage_send(&msgObj, sizeof(msgObj), CSL_CORE_ID_R5FSS0_1,
                        gServerEndPt, gAckEndPt, SystemP_NO_WAIT);
        if(status == SystemP_TIMEOUT)
        {
            fifoFullCount++;
        }
    }
    /* send one message waiting for space in FIFO just to check a success case after failure */
    status = RPMessage_send(&msgObj, sizeof(msgObj), CSL_CORE_ID_R5FSS0_1,
                        gServerEndPt, gAckEndPt, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    DebugP_log("[TEST IPC RPMSG] Messages sent = %d \r\n", msgCount);
    DebugP_log("[TEST IPC RPMSG] FIFO Full count = %d \r\n", fifoFullCount);

    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(0, fifoFullCount);

    /* use recv API on a end point which is created in handler mode */
    status = RPMessage_recv(&gAckMsgObject, &msgObj, &msgSize, &remoteCoreId, &remoteEndPt, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    /* recv error checks, first create rpmsg object */
    RPMessage_CreateParams_init(&rpmsgPrm);
    rpmsgPrm.localEndPt = gNullEndPt;
    status = RPMessage_construct(&gNullRpmsgObj, &rpmsgPrm);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* parameter checks */
    status = RPMessage_recv(&gNullRpmsgObj, NULL, &msgSize, &remoteCoreId, &remoteEndPt, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);
    status = RPMessage_recv(&gNullRpmsgObj, &msgObj, NULL, &remoteCoreId, &remoteEndPt, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);
    status = RPMessage_recv(&gNullRpmsgObj, &msgObj, &msgSize, NULL, &remoteEndPt, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);
    status = RPMessage_recv(&gNullRpmsgObj, &msgObj, &msgSize, &remoteCoreId, NULL, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);
    /* timeout check */
    status = RPMessage_recv(&gNullRpmsgObj, &msgObj, &msgSize, &remoteCoreId, &remoteEndPt, 10);
    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);
    status = RPMessage_recv(&gNullRpmsgObj, &msgObj, &msgSize, &remoteCoreId, &remoteEndPt, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);

    /* unblock test, post unblock and then waitforever, it should return with a timeout status */
    RPMessage_unblock(&gNullRpmsgObj);
    status = RPMessage_recv(&gNullRpmsgObj, &msgObj, &msgSize, &remoteCoreId, &remoteEndPt, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);

    /* cleanup */
    RPMessage_destruct(&gNullRpmsgObj);

    DebugP_logZoneRestore(oldDebugLogZone);
}

/* This code executes on remote core, i.e not on main core */
void test_ipc_remote_core_start()
{
    UNITY_BEGIN();

    /* announce endpt to main core, this is need for annouce test */
    RPMessage_announce(gMainCoreId, RPMessage_getLocalEndPt(&gServerMsgObject), SERVER_NAME);

    /* Run any to any test */
    test_rpmsgAnyToAny(NULL);

    /* wait for ever, now onwards main core will initaite all requests
     * to the server task, which will simply echo the messages
     */
    ClockP_sleep(SystemP_WAIT_FOREVER);

    UNITY_END();
}

/* This code executes on main core, i.e not on remote core */
void test_ipc_main_core_start()
{
    uint32_t i;

    Test_Args testArgs;

    UNITY_BEGIN();

    /* These MUST be the first tests to run */
    RUN_TEST(test_rpmsgControlEndPt, 296, NULL);
    RUN_TEST(test_rpmsgAnyToAny, 297, NULL);

    /* now you can comment tests if needed to debug a specific test */
    testArgs.echoMsgCount = 1000; /* this value is used by all later tests */
    /* performance test with minimum payload size */
    testArgs.msgSize = 4;
    testArgs.remoteCoreId = CSL_CORE_ID_R5FSS0_1;
    RUN_TEST(test_rpmsgOneToOne, 298, &testArgs);

    #if defined(SOC_AM64X) || defined(SOC_AM243X) || defined(SOC_AM263X)
    testArgs.remoteCoreId = CSL_CORE_ID_R5FSS1_0;
    RUN_TEST(test_rpmsgOneToOne, 300, &testArgs);
    testArgs.remoteCoreId = CSL_CORE_ID_R5FSS1_1;
    RUN_TEST(test_rpmsgOneToOne, 301, &testArgs);
    #endif

    #if defined(SOC_AM64X) || defined(SOC_AM243X)
    testArgs.remoteCoreId = CSL_CORE_ID_M4FSS0_0;
    RUN_TEST(test_rpmsgOneToOne, 1819, &testArgs);
    #endif

    #if defined(SOC_AM64X)
    testArgs.remoteCoreId = CSL_CORE_ID_A53SS0_0;
    RUN_TEST(test_rpmsgOneToOne, 1870, &testArgs);
    #endif

    #if defined(SOC_AM273X) || defined(SOC_AWR294X)
    testArgs.remoteCoreId = CSL_CORE_ID_C66SS0;
    RUN_TEST(test_rpmsgOneToOne, 300, &testArgs);
    #endif

    /* performance test with varying payload size */
    testArgs.remoteCoreId = CSL_CORE_ID_R5FSS0_1;
    testArgs.msgSize = 32;
    RUN_TEST(test_rpmsgOneToOne, 302, &testArgs);
    testArgs.remoteCoreId = CSL_CORE_ID_R5FSS0_1;
    testArgs.msgSize = 64;
    RUN_TEST(test_rpmsgOneToOne, 303, &testArgs);
    testArgs.remoteCoreId = CSL_CORE_ID_R5FSS0_1;
    testArgs.msgSize = 112;
    RUN_TEST(test_rpmsgOneToOne, 304, &testArgs);

    #if defined(SOC_AM64X) || defined(SOC_AM243X)
    /* performance test with varying payload size */
    testArgs.remoteCoreId = CSL_CORE_ID_M4FSS0_0;
    testArgs.msgSize = 32;
    RUN_TEST(test_rpmsgOneToOne, 1820, &testArgs);
    testArgs.remoteCoreId = CSL_CORE_ID_M4FSS0_0;
    testArgs.msgSize = 64;
    RUN_TEST(test_rpmsgOneToOne, 1821, &testArgs);
    testArgs.remoteCoreId = CSL_CORE_ID_M4FSS0_0;
    testArgs.msgSize = 112;
    RUN_TEST(test_rpmsgOneToOne, 1822, &testArgs);
    #endif

    #if defined(SOC_AM64X)
    /* performance test with varying payload size */
    testArgs.remoteCoreId = CSL_CORE_ID_A53SS0_0;
    testArgs.msgSize = 32;
    RUN_TEST(test_rpmsgOneToOne, 1871, &testArgs);
    testArgs.remoteCoreId = CSL_CORE_ID_A53SS0_0;
    testArgs.msgSize = 64;
    RUN_TEST(test_rpmsgOneToOne, 1872, &testArgs);
    testArgs.remoteCoreId = CSL_CORE_ID_A53SS0_0;
    testArgs.msgSize = 112;
    RUN_TEST(test_rpmsgOneToOne, 1873, &testArgs);
    #endif

    /* back to back message send and handler mode rx tests */
    testArgs.remoteCoreId = CSL_CORE_ID_R5FSS0_1;
    testArgs.msgSize = 4;
    RUN_TEST(test_rpmsgOneToOneBackToBack, 305, &testArgs);

    #if defined(SOC_AM64X) || defined(SOC_AM243X)
    /* back to back message send and handler mode rx tests */
    testArgs.remoteCoreId = CSL_CORE_ID_M4FSS0_0;
    testArgs.msgSize = 4;
    RUN_TEST(test_rpmsgOneToOneBackToBack, 1823, &testArgs);
    #endif

    #if defined(SOC_AM64X)
    /* back to back message send and handler mode rx tests */
    testArgs.remoteCoreId = CSL_CORE_ID_A53SS0_0;
    testArgs.msgSize = 4;
    RUN_TEST(test_rpmsgOneToOneBackToBack, 1874, &testArgs);
    #endif

    /* rx notify callback tests */
    testArgs.remoteCoreId = CSL_CORE_ID_R5FSS0_1;
    testArgs.msgSize = 4;
    testArgs.echoMsgCount = 1000;
    RUN_TEST(test_rpmsgRxNotifyCallback, 909, &testArgs);

    /* error condition checks */
    RUN_TEST(test_rpmsgErrorChecks, 306, NULL);

    /* Print performance numbers. */
    DebugP_log("\n[TEST IPC RPMSG] Performance Numbers Print Start\r\n\n");
    DebugP_log("- %u messages are sent and average one way message latency is measured\r\n\n", gMsgEchoCount);
    DebugP_log("Local Core  | Remote Core | Message Size | Average Message Latency (us)\r\n");
    DebugP_log("------------|-------------|--------------|------------------------------\r\n");
    for (i=0; i<gIpcPerfCnt; i++) {
        DebugP_log(" %s\t| %s\t| %d\t| %5.3f\r\n", SOC_getCoreName(gMainCoreId), SOC_getCoreName(gIpcPerfObj[i].remoteCoreId),
            gIpcPerfObj[i].msgSize,
            ((float)(gIpcPerfObj[i].msgLatency*1000/(gMsgEchoCount*2))/1000));
    }
    DebugP_log("\n[TEST IPC RPMSG] Performance Numbers Print End\r\n\n");

    /* delete objects test, this MUST be the last test */
    test_rpmsgDestructObjects();

    UNITY_END();
}

void test_main(void *args)
{
    Drivers_open();

    test_rpmsgCreateObjects();
    if(IpcNotify_getSelfCoreId()==gMainCoreId)
    {
        test_ipc_main_core_start();
    }
    else
    {
        test_ipc_remote_core_start();
    }

    Drivers_close();
}

void setUp(void)
{
}

void tearDown(void)
{
}

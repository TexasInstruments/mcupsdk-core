/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
#include <kernel/dpl/DebugP.h>
#include <drivers/ipc_notify.h>
#include <drivers/ipc_rpmsg.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/ipc_notify/v1/ipc_notify_v1.h>
/* This example shows message exchange between two cores.
 *
 * One of the core is designated as the 'main' core
 * and other cores are designated as `remote` cores.
 *
 * 
 * This is a stress test involving two cores, main core and remote core.
 * Can use one of the remote cores, then enable ipc in SysConfig for the remnote core
 * 
 * In this test both the main core and remote core send messages simultaneously and
 * receive the messages synchronously (callback mode)
 *
 * The main core repeats this for gMsgEchoCount iterations.
 *
 * In each iteration of message exchange, the message value is incremented.
 *
 * When iteration count reaches gMsgEchoCount, the example is completed.
 *
 */

 /* maximum size that message can have in this example */
#define MAX_MSG_SIZE        (64u)

/* Main core end point
 *
 * pick any unique value on that core between 0..RPMESSAGE_MAX_LOCAL_ENDPT-1
 * the value need not be unique across cores
 */
#define MAIN_CORE_END_PT  (12U)
/* number of iterations of message exchange to do */

uint32_t gMsgEchoCount = 1000000U;
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that also send messages to main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId[] = {
    // CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_C66SS0,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};

/*
 * Remote core  end point
 *
 * pick any unique value on that core between 0..RPMESSAGE_MAX_LOCAL_ENDPT-1
 * the value need not be unique across cores
 */
uint16_t gRemoteCoreEndPt = 13u;

/* RPMessage_Object MUST be global or static */
RPMessage_Object gAckReplyMsgObject;
SemaphoreP_Object gSrcDoneSem;
SemaphoreP_Object gDestDoneSem;

char     gRecvMsgBuffDsp[100U];
char     gRecvMsgBuffMainCore[100U];
uint16_t gSrcRecvMsgSize, gSrcRemoteCoreId, gSrcRemoteCoreEndPt ;
uint32_t gMainCoreRcvMsgCount = 0U;
uint32_t gRemoteCoreRcvMsgCount = 0U;
uint32_t gTransmitIndex = 0U;
uint32_t gRecvIndex = 0U;
uint32_t gNoOfMessagesMainCoreSent = 0U;
uint32_t gNoOfMessagesRemoteCoreSent = 0U;

 void gRemoteCoreEndPtCallback(RPMessage_Object *obj, void *arg, void *data, uint16_t dataLen, int32_t crcStatus, uint16_t remoteCoreId, uint16_t remoteEndPt)
{
    memcpy( &gRecvMsgBuffDsp[gTransmitIndex % 100], data ,dataLen);
    gTransmitIndex++;
    gSrcRecvMsgSize = dataLen;
    gSrcRemoteCoreId = remoteCoreId;
    gSrcRemoteCoreEndPt = remoteEndPt;
    if(++gRemoteCoreRcvMsgCount == gMsgEchoCount)
    {
        SemaphoreP_post(&gDestDoneSem);
    }
}

void gMainCoreEndPtCallback(RPMessage_Object *obj, void *arg, void *data, uint16_t dataLen, int32_t crcStatus, uint16_t remoteCoreId, uint16_t remoteEndPt)
{
    memcpy(&gRecvMsgBuffMainCore[gRecvIndex % 100], data, dataLen);
    gRecvIndex++;
    (void) remoteCoreId;
    (void) remoteEndPt;
    if(++gMainCoreRcvMsgCount == gMsgEchoCount)
    {
        SemaphoreP_post(&gSrcDoneSem);
    }
}

void ipc_rpmsg_echo_main_core_start(void)
{
    RPMessage_CreateParams createParams;
    uint32_t msg, numRemoteCores, i;
    uint64_t curTime;
    char msgBuf[MAX_MSG_SIZE];
    int32_t status;
    uint16_t msgSize;

    status = SemaphoreP_constructBinary(&gSrcDoneSem, 0);
    DebugP_assert(status==SystemP_SUCCESS);

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = MAIN_CORE_END_PT;
    createParams.recvCallback = gMainCoreEndPtCallback;
    status = RPMessage_construct(&gAckReplyMsgObject, &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    numRemoteCores = 0;
    for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++)
    {
        numRemoteCores++;
    }

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    ClockP_usleep(500*1000); /* wait for log messages from remote cores to be flushed, otherwise this delay is not needed */

    DebugP_log("[IPC RPMSG ECHO] Message exchange started by main core !!!\r\n");

    curTime = ClockP_getTimeUsec();

    for(msg = 0; msg < gMsgEchoCount; msg++)
    {
        snprintf(msgBuf, MAX_MSG_SIZE-1, "%" PRIu32, msg);
        msgBuf[MAX_MSG_SIZE-1] = 0;
        msgSize = strlen(msgBuf) + 1; /* count the terminating char as well */
        /* send the same messages to all cores */
            status = RPMessage_send(msgBuf, msgSize, gRemoteCoreId[0], gRemoteCoreEndPt, RPMessage_getLocalEndPt(&gAckReplyMsgObject),SystemP_WAIT_FOREVER);
            DebugP_assert(status==SystemP_SUCCESS);
            gNoOfMessagesMainCoreSent++;
    }

    SemaphoreP_pend(&gSrcDoneSem, SystemP_WAIT_FOREVER);
    curTime = ClockP_getTimeUsec() - curTime;
    if(gMainCoreRcvMsgCount == gMsgEchoCount)
    {
        DebugP_log("[IPC RPMSG ECHO] Messages Received from Remote core = %d \r\n", gMainCoreRcvMsgCount);
        curTime = ClockP_getTimeUsec() - curTime;

        DebugP_log("[IPC RPMSG ECHO] All echoed messages received by main core from %d remote cores !!!\r\n", numRemoteCores);
        DebugP_log("[IPC RPMSG ECHO] Messages sent to each core = %d \r\n", gNoOfMessagesMainCoreSent);
        DebugP_log("[IPC RPMSG ECHO] Number of remote cores = %d \r\n", numRemoteCores);
        DebugP_log("[IPC RPMSG ECHO] Total execution time = %" PRId64 " usecs\r\n", curTime);
        DebugP_log("[IPC RPMSG ECHO] One way message latency = %" PRId32 " nsec\r\n",
            (uint32_t)(curTime*1000u/(gMsgEchoCount*numRemoteCores*2)));

        RPMessage_destruct(&gAckReplyMsgObject);

        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log(" IPC RPMessage send failed!!\r\n");
    }

}

/* RPMessage_Object MUST be global or static */
static RPMessage_Object gRecvMsgObject;

void ipc_rpmsg_echo_remote_core_start(void)
{
    int32_t status;
    uint32_t msg;
    char msgBuf[MAX_MSG_SIZE];
    uint16_t msgSize;
    RPMessage_CreateParams createParams;

    status = SemaphoreP_constructBinary(&gDestDoneSem, 0);
    DebugP_assert(status==SystemP_SUCCESS);

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt   = gRemoteCoreEndPt;
    createParams.recvCallback = gRemoteCoreEndPtCallback;
    status = RPMessage_construct(&gRecvMsgObject, &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("[IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!\r\n");

    for(msg = 0; msg < gMsgEchoCount; msg++)
    {
        snprintf(msgBuf, MAX_MSG_SIZE-1, "%" PRIu32, msg);
        msgBuf[MAX_MSG_SIZE-1] = 0;
        msgSize = strlen(msgBuf) + 1; /* count the terminating char as well */
        /* send the same messages to all cores */

        status = RPMessage_send(msgBuf, msgSize, gMainCoreId, MAIN_CORE_END_PT, RPMessage_getLocalEndPt(&gRecvMsgObject),SystemP_WAIT_FOREVER);
        DebugP_assert(status==SystemP_SUCCESS);
        gNoOfMessagesRemoteCoreSent++;
        
    }
    SemaphoreP_pend(&gDestDoneSem, SystemP_WAIT_FOREVER);
    
    DebugP_log("[IPC RPMSG ECHO] Messages Sent to Remote core = %d \r\n", gNoOfMessagesRemoteCoreSent);
    DebugP_log("[IPC RPMSG ECHO] Messages Received from Remote core = %d \r\n", gRemoteCoreRcvMsgCount);

}

void ipc_rpmsg_callback_simultaneous_send_dualcore_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    if(IpcNotify_getSelfCoreId()==gMainCoreId)
    {
        ipc_rpmsg_echo_main_core_start();
    }
    else
    {
        ipc_rpmsg_echo_remote_core_start();
    }

    Board_driversClose();
    /* We dont close drivers to let the UART driver remain open and flush any pending messages to console */
    /* Drivers_close(); */
}


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

/* This example shows message exchange between multiple cores.
 *
 * One of the core is designated as the 'main' core
 * and other cores are designated as `remote` cores.
 *
 * The main core initiates IPC with remote core's by sending it a message.
 * The remote cores echo the same message to the main core.
 *
 * The main core repeats this for gMsgEchoCount iterations.
 *
 * In each iteration of message exchange, the message value is incremented.
 *
 * When iteration count reaches gMsgEchoCount, the example is completed.
 *
 */

/* number of iterations of message exchange to do */
uint32_t gMsgEchoCount = 100000u;

typedef struct
{
    uint32_t data;
    uint16_t dataLen;
    uint16_t remoteCoreId;
    uint16_t remoteEndPt;
} Ipc_RecvData;

Ipc_RecvData gRecvDataMain[CSL_CORE_ID_MAX], gRecvDataRemote;

#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_R5FSS1_0,
    CSL_CORE_ID_R5FSS1_1,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#if defined(SOC_AM261X)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

/*
 * Remote core service end point
 *
 * pick any unique value on that core between 0..RPMESSAGE_MAX_LOCAL_ENDPT-1
 * the value need not be unique across cores
 */
uint16_t gRemoteServiceEndPt = 13u;

/* maximum size that message can have in this example */
#define MAX_MSG_SIZE        (64u)

/* Main core ack reply end point
 *
 * pick any unique value on that core between 0..RPMESSAGE_MAX_LOCAL_ENDPT-1
 * the value need not be unique across cores
 */
#define MAIN_CORE_ACK_REPLY_END_PT  (12U)

/* RPMessage_Object MUST be global or static */
RPMessage_Object gAckReplyMsgObject;

/* semaphore's used to indicate a main core has finished all message exchanges */
SemaphoreP_Object gMainDoneSem[CSL_CORE_ID_MAX];

/* semaphore used to indicate a remote core has finished all message xchange */
SemaphoreP_Object gRemoteDoneSem;

void ipc_rpmsg_msg_handler_main_core(RPMessage_Object *obj, void *arg, void *data, uint16_t dataLen, int32_t crcStatus, uint16_t remoteCoreId, uint16_t remoteEndPt)
{
    uint32_t msgValue = *((uint32_t *)data);

    gRecvDataMain[remoteCoreId].data = msgValue + 1U;
    gRecvDataMain[remoteCoreId].dataLen = dataLen;
    gRecvDataMain[remoteCoreId].remoteCoreId = remoteCoreId;
    gRecvDataMain[remoteCoreId].remoteEndPt = remoteEndPt;

    /* there is one semaphore for each core ID, so post the semaphore for the remote core that
        * has finished all message exchange iterations
        */
    SemaphoreP_post(&gMainDoneSem[remoteCoreId]);
}

void ipc_rpmsg_msg_handler_remote_core(RPMessage_Object *obj, void *arg, void *data, uint16_t dataLen, int32_t crcStatus, uint16_t remoteCoreId, uint16_t remoteEndPt)
{
    uint32_t msgValue = *((uint32_t *)data);

    gRecvDataRemote.data = msgValue;
    gRecvDataRemote.dataLen = dataLen;
    gRecvDataRemote.remoteCoreId = remoteCoreId;
    gRecvDataRemote.remoteEndPt = remoteEndPt;

    SemaphoreP_post(&gRemoteDoneSem);
}

void ipc_rpmsg_echo_main_core_start(void)
{
    RPMessage_CreateParams createParams;
    uint32_t msg, i, numRemoteCores;
    uint64_t curTime;
    int32_t status;

    /* create completion semaphores for all cores */
    for(i=0; i < CSL_CORE_ID_MAX; i++)
    {
        SemaphoreP_constructBinary(&gMainDoneSem[i], 0);
    }

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = MAIN_CORE_ACK_REPLY_END_PT;
    createParams.recvCallback = ipc_rpmsg_msg_handler_main_core;
    createParams.recvCallbackArgs = &gMainDoneSem;
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
        /* send the same messages to all cores */
        for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++ )
        {
            status = RPMessage_send(
                &gRecvDataMain[gRemoteCoreId[i]].data, sizeof(gRecvDataMain[gRemoteCoreId[i]].data),
                gRemoteCoreId[i], gRemoteServiceEndPt,
                RPMessage_getLocalEndPt(&gAckReplyMsgObject),
                SystemP_WAIT_FOREVER);
            DebugP_assert(status==SystemP_SUCCESS);
        }
        /* wait for response from all cores */
        for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++ )
        {
            SemaphoreP_pend(&gMainDoneSem[gRemoteCoreId[i]], SystemP_WAIT_FOREVER);
        }
    }

    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_log("[IPC RPMSG ECHO] All echoed messages received by main core from %d remote cores !!!\r\n", numRemoteCores);
    DebugP_log("[IPC RPMSG ECHO] Messages sent to each core = %d \r\n", gMsgEchoCount);
    DebugP_log("[IPC RPMSG ECHO] Number of remote cores = %d \r\n", numRemoteCores);
    DebugP_log("[IPC RPMSG ECHO] Total execution time = %" PRId64 " usecs\r\n", curTime);
    DebugP_log("[IPC RPMSG ECHO] One way message latency = %" PRId32 " nsec\r\n",
        (uint32_t)(curTime*1000u/(gMsgEchoCount*numRemoteCores*2)));

    RPMessage_destruct(&gAckReplyMsgObject);

    DebugP_log("All tests have passed!!\r\n");
}

/* RPMessage_Object MUST be global or static */
static RPMessage_Object gRecvMsgObject;

void ipc_rpmsg_echo_remote_core_start(void)
{
    int32_t status;
    RPMessage_CreateParams createParams;

    SemaphoreP_constructBinary(&gRemoteDoneSem, 0);

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = gRemoteServiceEndPt;
    createParams.recvCallback = ipc_rpmsg_msg_handler_remote_core;
    createParams.recvCallbackArgs = &gRemoteDoneSem;
    status = RPMessage_construct(&gRecvMsgObject, &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("[IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!\r\n");

    /* wait for messages forever in a loop */
    while(1)
    {
        /* wait for message */
        SemaphoreP_pend(&gRemoteDoneSem, SystemP_WAIT_FOREVER);

        /* echo the same message as reply */
        /* send ack to sender CPU at the sender end point */
        status = RPMessage_send(
            &gRecvDataRemote.data, gRecvDataRemote.dataLen,
            gRecvDataRemote.remoteCoreId, gRecvDataRemote.remoteEndPt,
            RPMessage_getLocalEndPt(&gRecvMsgObject),
            SystemP_WAIT_FOREVER);
        DebugP_assert(status==SystemP_SUCCESS);
    }
    /* This loop will never exit */
}

void ipc_rpmsg_echo_callback_main(void *args)
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

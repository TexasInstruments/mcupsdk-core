/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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
#include <kernel/dpl/TaskP.h>
#include <drivers/ipc_notify.h>
#include <drivers/ipc_rpmsg.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "FreeRTOS.h"
#include "task.h"


/* This example shows message exchange between Linux and RTOS/NORTOS cores.
 * This example also does message exchange between the RTOS/NORTOS cores themselves.
 *
 * The Linux core initiates IPC with other core's by sending it a message.
 * The other cores echo the same message to the Linux core.
 *
 * At the same time all RTOS/NORTOS cores, also send messages to each others
 * and reply back to each other. i.e all CPUs send and recevive messages from each other
 *
 * This example can very well have been NORTOS based, however for convinience
 * of creating two tasks to talk to two clients on linux side, we use FreeRTOS
 * for the same.
 */

/*
 * Remote core service end point
 *
 * Pick any unique value on that core between 0..RPMESSAGE_MAX_LOCAL_ENDPT-1.
 * The value need not be unique across cores.
 *
 * The service names MUST match what linux is expecting
 */
/* This is used to run the echo test with linux kernel */
#define IPC_RPMESSAGE_SERVICE_PING        "ti.ipc4.ping-pong"
#define IPC_RPMESSAGE_ENDPT_PING          (13U)

/* This is used to run the echo test with user space kernel */
#define IPC_RPMESSAGE_SERVICE_CHRDEV      "rpmsg_chrdev"
#define IPC_RPMESSAGE_ENDPT_CHRDEV_PING   (14U)

/* Use by this to receive ACK messages that it sends to other RTOS cores */
#define IPC_RPMESSAGE_RNDPT_ACK_REPLY     (11U)

/* Maximum size that message can have in this example
 * RPMsg maximum size is 512 bytes in linux including the header of 16 bytes.
 * Message payload size without the header is 512 - 16 = 496
 */
#define IPC_RPMESSAGE_MAX_MSG_SIZE        (496u)

/*
 * Number of RP Message ping "servers" we will start,
 * - one for ping messages for linux kernel "sample ping" client
 * - and another for ping messages from linux "user space" client using "rpmsg char"
 */
#define IPC_RPMESSAGE_NUM_RECV_TASKS         (2u)

/**
 * \brief Client ID used for remoteproc (RP_MBOX) related messages,  this client ID should not be used by other users
 */
#define IPC_NOTIFY_CLIENT_ID_RP_MBOX  (15U)

/**
 * \brief Various messages sent by remote proc kernel driver.
 *
 * Client ID for this will always be 15 \ref IPC_NOTIFY_CLIENT_ID_RP_MBOX
 */
#define IPC_NOTIFY_RP_MBOX_READY		    (0x0FFFFF00)
#define IPC_NOTIFY_RP_MBOX_PENDING_MSG	    (0x0FFFFF01)
#define IPC_NOTIFY_RP_MBOX_CRASH		    (0x0FFFFF02)
#define IPC_NOTIFY_RP_MBOX_ECHO_REQUEST	    (0x0FFFFF03)
#define IPC_NOTIFY_RP_MBOX_ECHO_REPLY	    (0x0FFFFF04)
#define IPC_NOTIFY_RP_MBOX_ABORT_REQUEST	(0x0FFFFF05)
#define IPC_NOTIFY_RP_MBOX_SUSPEND_AUTO	    (0x0FFFFF10)
#define IPC_NOTIFY_RP_MBOX_SUSPEND_SYSTEM	(0x0FFFFF11)
#define IPC_NOTIFY_RP_MBOX_SUSPEND_ACK	    (0x0FFFFF12)
#define IPC_NOTIFY_RP_MBOX_SUSPEND_CANCEL	(0x0FFFFF13)
#define IPC_NOTIFY_RP_MBOX_SHUTDOWN     	(0x0FFFFF14)
#define IPC_NOTIFY_RP_MBOX_SHUTDOWN_ACK     (0x0FFFFF15)
#define IPC_NOTIFY_RP_MBOX_END_MSG		    (0x0FFFFF16)

/* RPMessage object used to recvice messages */
RPMessage_Object gIpcRecvMsgObject[IPC_RPMESSAGE_NUM_RECV_TASKS];

/* RPMessage object used to send messages to other non-Linux remote cores */
RPMessage_Object gIpcAckReplyMsgObject;

/* Task priority, stack, stack size and task objects, these MUST be global's */
#define IPC_RPMESSAGE_TASK_PRI         (8U)
#define IPC_RPMESSAGE_TASK_STACK_SIZE  (8*1024U)
uint8_t gIpcTaskStack[IPC_RPMESSAGE_NUM_RECV_TASKS][IPC_RPMESSAGE_TASK_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gIpcTask[IPC_RPMESSAGE_NUM_RECV_TASKS];

/* number of iterations of message exchange to do */
uint32_t gMsgEchoCount = 100000u;
/* non-Linux cores that exchange messages among each other */
#if defined (SOC_AM64X)
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_0,
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_R5FSS1_0,
    CSL_CORE_ID_R5FSS1_1,
    CSL_CORE_ID_M4FSS0_0,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#if defined (SOC_AM65X)
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_0,
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#if defined (SOC_AM62X)
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_M4FSS0_0,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

volatile uint8_t gShutdown = 0u;
volatile uint8_t gShutdownRemotecoreID = 0u;
volatile uint8_t gIpcAckReplyMsgObjectPending = 0u;

void ipc_recv_task_main(void *args)
{
    int32_t status;
    char recvMsg[IPC_RPMESSAGE_MAX_MSG_SIZE+1]; /* +1 for NULL char in worst case */
    uint16_t recvMsgSize, remoteCoreId, remoteCoreEndPt;
    RPMessage_Object *pRpmsgObj = (RPMessage_Object *)args;

    DebugP_log("[IPC RPMSG ECHO] Remote Core waiting for messages at end point %d ... !!!\r\n",
        RPMessage_getLocalEndPt(pRpmsgObj)
        );

    /* wait for messages forever in a loop */
    while(1)
    {
        /* set 'recvMsgSize' to size of recv buffer,
        * after return `recvMsgSize` contains actual size of valid data in recv buffer
        */
        recvMsgSize = IPC_RPMESSAGE_MAX_MSG_SIZE;
        status = RPMessage_recv(pRpmsgObj,
            recvMsg, &recvMsgSize,
            &remoteCoreId, &remoteCoreEndPt,
            SystemP_WAIT_FOREVER);

        if (gShutdown == 1u)
        {
            break;
        }

        DebugP_assert(status==SystemP_SUCCESS);

        /* echo the same message string as reply */
        #if 0 /* not logging this so that this does not add to the latency of message exchange */
        recvMsg[recvMsgSize] = 0; /* add a NULL char at the end of message */
        DebugP_log("%s\r\n", recvMsg);
        #endif

        /* send ack to sender CPU at the sender end point */
        status = RPMessage_send(
            recvMsg, recvMsgSize,
            remoteCoreId, remoteCoreEndPt,
            RPMessage_getLocalEndPt(pRpmsgObj),
            SystemP_WAIT_FOREVER);
        DebugP_assert(status==SystemP_SUCCESS);
    }

    DebugP_log("[IPC RPMSG ECHO] Closing all drivers and going to WFI ... !!!\r\n");

    /* Close the drivers */
    Drivers_close();

    /* ACK the suspend message */
    IpcNotify_sendMsg(gShutdownRemotecoreID, IPC_NOTIFY_CLIENT_ID_RP_MBOX, IPC_NOTIFY_RP_MBOX_SHUTDOWN_ACK, 1u);

    /* Disable interrupts */
    HwiP_disable();
#if (__ARM_ARCH_PROFILE == 'R') ||  (__ARM_ARCH_PROFILE == 'M')
    /* For ARM R and M cores*/
    __asm__ __volatile__ ("wfi"   "\n\t": : : "memory");
#endif
#if defined(BUILD_C7X)
    asm("    IDLE");
#endif
    vTaskDelete(NULL);
}

void ipc_rpmsg_send_messages(void)
{
    RPMessage_CreateParams createParams;
    uint32_t msg, i, numRemoteCores;
    uint64_t curTime;
    char msgBuf[IPC_RPMESSAGE_MAX_MSG_SIZE];
    int32_t status;
    uint16_t remoteCoreId, remoteCoreEndPt, msgSize;

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = IPC_RPMESSAGE_RNDPT_ACK_REPLY;
    status = RPMessage_construct(&gIpcAckReplyMsgObject, &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    numRemoteCores = 0;
    for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++)
    {
        if(gRemoteCoreId[i] != IpcNotify_getSelfCoreId()) /* dont count self */
        {
            numRemoteCores++;
        }
    }

    DebugP_log("[IPC RPMSG ECHO] Message exchange started with RTOS cores !!!\r\n");

    curTime = ClockP_getTimeUsec();

    for(msg=0; msg<gMsgEchoCount; msg++)
    {
        snprintf(msgBuf, IPC_RPMESSAGE_MAX_MSG_SIZE-1, "%d", msg);
        msgBuf[IPC_RPMESSAGE_MAX_MSG_SIZE-1] = 0;
        msgSize = strlen(msgBuf) + 1; /* count the terminating char as well */

        /* send the same messages to all cores */
        for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++ )
        {
            if(gRemoteCoreId[i] != IpcNotify_getSelfCoreId()) /* dont send message to self */
            {
                status = RPMessage_send(
                    msgBuf, msgSize,
                    gRemoteCoreId[i], IPC_RPMESSAGE_ENDPT_CHRDEV_PING,
                    RPMessage_getLocalEndPt(&gIpcAckReplyMsgObject),
                    SystemP_WAIT_FOREVER);
                DebugP_assert(status==SystemP_SUCCESS);
            }
        }
        /* wait for response from all cores */
        for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++ )
        {
            if(gRemoteCoreId[i] != IpcNotify_getSelfCoreId()) /* dont send message to self */
            {
                /* set 'msgSize' to size of recv buffer,
                * after return `msgSize` contains actual size of valid data in recv buffer
                */
                msgSize = sizeof(msgBuf);

                gIpcAckReplyMsgObjectPending = 1;
                status = RPMessage_recv(&gIpcAckReplyMsgObject,
                    msgBuf, &msgSize,
                    &remoteCoreId, &remoteCoreEndPt,
                    SystemP_WAIT_FOREVER);
                if (gShutdown == 1u)
                {
                    break;
                }
                DebugP_assert(status==SystemP_SUCCESS);
                gIpcAckReplyMsgObjectPending = 0;

            }
        }
        if (gShutdown == 1u)
        {
            break;
        }
    }
    gIpcAckReplyMsgObjectPending = 0;

    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_log("[IPC RPMSG ECHO] All echoed messages received by main core from %d remote cores !!!\r\n", numRemoteCores);
    DebugP_log("[IPC RPMSG ECHO] Messages sent to each core = %d \r\n", gMsgEchoCount);
    DebugP_log("[IPC RPMSG ECHO] Number of remote cores = %d \r\n", numRemoteCores);
    DebugP_log("[IPC RPMSG ECHO] Total execution time = %" PRId64 " usecs\r\n", curTime);
    DebugP_log("[IPC RPMSG ECHO] One way message latency = %" PRId32 " nsec\r\n",
        (uint32_t)(curTime*1000u/(gMsgEchoCount*numRemoteCores*2)));

    RPMessage_destruct(&gIpcAckReplyMsgObject);
}

void ipc_rpmsg_create_recv_tasks(void)
{
    int32_t status;
    RPMessage_CreateParams createParams;
    TaskP_Params taskParams;

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = IPC_RPMESSAGE_ENDPT_PING;
    status = RPMessage_construct(&gIpcRecvMsgObject[0], &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = IPC_RPMESSAGE_ENDPT_CHRDEV_PING;
    status = RPMessage_construct(&gIpcRecvMsgObject[1], &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    /* We need to "announce" to Linux client else Linux does not know a service exists on this CPU
     * This is not mandatory to do for RTOS clients
     */
    status = RPMessage_announce(CSL_CORE_ID_A53SS0_0, IPC_RPMESSAGE_ENDPT_PING, IPC_RPMESSAGE_SERVICE_PING);
    DebugP_assert(status==SystemP_SUCCESS);

    status = RPMessage_announce(CSL_CORE_ID_A53SS0_0, IPC_RPMESSAGE_ENDPT_CHRDEV_PING, IPC_RPMESSAGE_SERVICE_CHRDEV);
    DebugP_assert(status==SystemP_SUCCESS);

    /* Create the tasks which will handle the ping service */
    TaskP_Params_init(&taskParams);
    taskParams.name = "RPMESSAGE_PING";
    taskParams.stackSize = IPC_RPMESSAGE_TASK_STACK_SIZE;
    taskParams.stack = gIpcTaskStack[0];
    taskParams.priority = IPC_RPMESSAGE_TASK_PRI;
    /* we use the same task function for echo but pass the appropiate rpmsg handle to it, to echo messages */
    taskParams.args = &gIpcRecvMsgObject[0];
    taskParams.taskMain = ipc_recv_task_main;

    status = TaskP_construct(&gIpcTask[0], &taskParams);
    DebugP_assert(status == SystemP_SUCCESS);

    TaskP_Params_init(&taskParams);
    taskParams.name = "RPMESSAGE_CHAR_PING";
    taskParams.stackSize = IPC_RPMESSAGE_TASK_STACK_SIZE;
    taskParams.stack = gIpcTaskStack[1];
    taskParams.priority = IPC_RPMESSAGE_TASK_PRI;
    /* we use the same task function for echo but pass the appropiate rpmsg handle to it, to echo messages */
    taskParams.args = &gIpcRecvMsgObject[1];
    taskParams.taskMain = ipc_recv_task_main;

    status = TaskP_construct(&gIpcTask[1], &taskParams);
    DebugP_assert(status == SystemP_SUCCESS);
}

void ipc_rp_mbox_callback(uint32_t remoteCoreId, uint16_t clientId, uint32_t msgValue, int32_t crcStatus, void *args)
{
    if (clientId == IPC_NOTIFY_CLIENT_ID_RP_MBOX)
    {
        if (msgValue == IPC_NOTIFY_RP_MBOX_SHUTDOWN)
        {
            /* Suspend request from the remotecore */
            gShutdown = 1u;
            gShutdownRemotecoreID = remoteCoreId;
            RPMessage_unblock(&gIpcRecvMsgObject[0]);
            RPMessage_unblock(&gIpcRecvMsgObject[1]);

            if (gIpcAckReplyMsgObjectPending == 1u)
                RPMessage_unblock(&gIpcAckReplyMsgObject);
        }
    }
}

void ipc_rpmsg_echo_main(void *args)
{
    int32_t status;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[IPC RPMSG ECHO] %s %s\r\n", __DATE__, __TIME__);

    /* This API MUST be called by applications when its ready to talk to Linux */
    status = RPMessage_waitForLinuxReady(SystemP_WAIT_FOREVER);
    DebugP_assert(status==SystemP_SUCCESS);

    /* Register a callback for the RP_MBOX messages from the Linux remoteproc driver*/
    IpcNotify_registerClient(IPC_NOTIFY_CLIENT_ID_RP_MBOX, &ipc_rp_mbox_callback, NULL);

    /* create message receive tasks, these tasks always run and never exit */
    ipc_rpmsg_create_recv_tasks();

    /* wait for all non-Linux cores to be ready, this ensure that when we send messages below
     * they wont be lost due to rpmsg end point not created at remote core
     */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Due to below "if" condition only one non-Linux core sends messages to all other non-Linux Cores
     * This is done mainly to show deterministic latency measurement
     */
    if( IpcNotify_getSelfCoreId() == CSL_CORE_ID_R5FSS0_0 )
    {
        ipc_rpmsg_send_messages();
    }
    /* exit from this task, vTaskDelete() is called outside this function, so simply return */

    Board_driversClose();
    /* We dont close drivers since threads are running in background */
    /* Drivers_close(); */
}

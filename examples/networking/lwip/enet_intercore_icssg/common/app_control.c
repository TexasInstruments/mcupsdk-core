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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
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

#include "app_control.h"
#include "ti_ic_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*
 * Remote core service end point
 *
 * Pick any unique value on that core between 0..RPMESSAGE_MAX_LOCAL_ENDPT-1.
 * The value need not be unique across cores.
 *
 * The service names MUST match what linux is expecting
 */
/* This is used to run the echo test with linux kernel */
#define IPC_RPMESSAGE_SERVICE_PING        "ti.icve"
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
#define IPC_RPMESSAGE_NUM_RECV_TASKS      (2U)

/**
 * \brief Client ID used for remoteproc (RP_MBOX) related messages,  this client ID should not be used by other users
 */
#define IPC_NOTIFY_CLIENT_ID_RP_MBOX      (15U)

/* Task priority, stack, stack size and task objects, these MUST be global's */
#define IPC_RPMESSAGE_TASK_PRI            (8U)
#define IPC_RPMESSAGE_TASK_STACK_SIZE     (8*1024U)

/**
 * \brief Various messages sent by remote proc kernel driver.
 *
 * Client ID for this will always be 15 \ref IPC_NOTIFY_CLIENT_ID_RP_MBOX
 */
#define IPC_NOTIFY_RP_MBOX_READY            (0x0FFFFF00)
#define IPC_NOTIFY_RP_MBOX_PENDING_MSG      (0x0FFFFF01)
#define IPC_NOTIFY_RP_MBOX_CRASH            (0x0FFFFF02)
#define IPC_NOTIFY_RP_MBOX_ECHO_REQUEST     (0x0FFFFF03)
#define IPC_NOTIFY_RP_MBOX_ECHO_REPLY       (0x0FFFFF04)
#define IPC_NOTIFY_RP_MBOX_ABORT_REQUEST    (0x0FFFFF05)
#define IPC_NOTIFY_RP_MBOX_SUSPEND_AUTO     (0x0FFFFF10)
#define IPC_NOTIFY_RP_MBOX_SUSPEND_SYSTEM   (0x0FFFFF11)
#define IPC_NOTIFY_RP_MBOX_SUSPEND_ACK      (0x0FFFFF12)
#define IPC_NOTIFY_RP_MBOX_SUSPEND_CANCEL   (0x0FFFFF13)
#define IPC_NOTIFY_RP_MBOX_SHUTDOWN         (0x0FFFFF14)
#define IPC_NOTIFY_RP_MBOX_SHUTDOWN_ACK     (0x0FFFFF15)
#define IPC_NOTIFY_RP_MBOX_END_MSG          (0x0FFFFF16)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* RPMessage object used to recvice messages */
RPMessage_Object gIpcRecvMsgObject[IPC_RPMESSAGE_NUM_RECV_TASKS];

uint8_t gIpcTaskStack[IPC_RPMESSAGE_NUM_RECV_TASKS][IPC_RPMESSAGE_TASK_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gIpcTask[IPC_RPMESSAGE_NUM_RECV_TASKS];

volatile uint8_t gShutdown = 0u;
volatile uint8_t gShutdownRemotecoreID = 0u;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void AppCtrl_recvTask(void *args);

static void AppCtrl_sendTask(void* args);

static void AppCtrl_handleControlMsg(void* pMsg, uint16_t remoteCoreId, uint16_t remoteCoreEndPt);

static int32_t AppCtrl_handleReq(Icve_reqMsg reqMsg, uint16_t remoteCoreId, uint16_t remoteCoreEndPt);

static int32_t AppCtrl_handleNotify(Icve_notifyMsg notifyMsg, uint16_t remoteCoreId, uint16_t remoteCoreEndPt);

static int32_t AppCtrl_handleResponse(Icve_respMsg respMsg, uint16_t remoteCoreId, uint16_t remoteCoreEndPt);

static int32_t AppCtrl_sendRespMsg(Icve_message* pMsg, uint16_t remoteCoreId, uint16_t remoteCoreEndPt);

static int32_t AppCtrl_respShrMemRegion(uint16_t remoteCoreId, uint16_t remoteCoreEndPt);

extern bool App_IsLinuxPresent();

extern uint32_t App_getSelfCoreId();

extern int32_t AppCtrl_addMacAddr2fbd(Icve_macAddr mac);

extern int32_t AppCtrl_addMcastAddr(Icve_macAddr mac);

extern int32_t AppCtrl_delMcastAddr(Icve_macAddr mac);

extern void AppTcp_startClient(ip_addr_t ipAddr);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void AppCtrl_mbox_callback(uint32_t remoteCoreId, uint16_t clientId, uint32_t msgValue, int32_t crcStatus, void *args)
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
        }
    }
}

/* ========================================================================== */
/*                        Control Message Handling                            */
/* ========================================================================== */

void AppCtrl_createRecvTask()
{
    int32_t status;
    TaskP_Params taskParams;
    RPMessage_CreateParams createParams;

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = IPC_RPMESSAGE_ENDPT_PING;
    status = RPMessage_construct(&gIpcRecvMsgObject[0], &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    /* Create the tasks which will handle the ping service */
    TaskP_Params_init(&taskParams);
    taskParams.name      = "RPMESSAGE_CTRL_RECV";
    taskParams.stackSize = IPC_RPMESSAGE_TASK_STACK_SIZE;
    taskParams.stack     = gIpcTaskStack[0];
    taskParams.priority  = (IPC_RPMESSAGE_TASK_PRI-2U);
    /* we use the same task function for echo but pass the appropiate rpmsg handle to it, to echo messages */
    taskParams.args      = &gIpcRecvMsgObject[0];
    taskParams.taskMain  = AppCtrl_recvTask;

    status = TaskP_construct(&gIpcTask[0], &taskParams);
    DebugP_assert(status == SystemP_SUCCESS);
}

static void AppCtrl_recvTask(void *args)
{
    char msgBuf[IPC_RPMESSAGE_MAX_MSG_SIZE];
    int32_t status;
    uint16_t remoteCoreId, remoteCoreEndPt, msgSize;
    RPMessage_Object *pRpmsgObj = (RPMessage_Object *)args;

    if(App_IsLinuxPresent())
    {
        /* This API MUST be called by applications when its ready to talk to Linux
         * Enable when Linux is enabled*/
        status = RPMessage_waitForLinuxReady(SystemP_WAIT_FOREVER);
        DebugP_assert(status==SystemP_SUCCESS);

        /* Register a callback for the RP_MBOX messages from the Linux remoteproc driver*/
        IpcNotify_registerClient(IPC_NOTIFY_CLIENT_ID_RP_MBOX, &AppCtrl_mbox_callback, NULL);

        /* We need to "announce" to Linux client else Linux does not know a service exists on this CPU
         * This is not mandatory to do for RTOS clients
         */
        status = RPMessage_announce(CSL_CORE_ID_A53SS0_0, IPC_RPMESSAGE_ENDPT_PING, IPC_RPMESSAGE_SERVICE_PING);
        DebugP_assert(status==SystemP_SUCCESS);
    }
    else
    {
        IpcNotify_syncAll(SystemP_WAIT_FOREVER);
    }

    while(1)
    {
        status = RPMessage_recv(pRpmsgObj,
                                msgBuf, &msgSize,
                                &remoteCoreId, &remoteCoreEndPt,
                                SystemP_WAIT_FOREVER);
        if(status == SystemP_SUCCESS)
        {
            DebugP_log("Received Test Pkt\r\n");
            DebugP_log("[IPC RPMSG ECHO] Remote Core ID    = %d \r\n", remoteCoreId);
            DebugP_log("[IPC RPMSG ECHO] Remote Core EndPt = %d \r\n", remoteCoreEndPt);
            AppCtrl_handleControlMsg(msgBuf, remoteCoreId, remoteCoreEndPt);
        }
    }
}

static void AppCtrl_handleControlMsg(void* pMsg, uint16_t remoteCoreId, uint16_t remoteCoreEndPt)
{
    int32_t status = ICVE_FAIL;
    Icve_message* msgBuf = (Icve_message*) pMsg;

    switch(msgBuf->msg_hdr.msg_type)
    {
        case ICVE_REQUEST_MSG:
            status = AppCtrl_handleReq(msgBuf->req_msg, remoteCoreId, remoteCoreEndPt);
            break;

        case ICVE_NOTIFY_MSG:
            status = AppCtrl_handleNotify(msgBuf->notify_msg, remoteCoreId, remoteCoreEndPt);
            break;

        case ICVE_RESPONSE_MSG:
            status = AppCtrl_handleResponse(msgBuf->resp_msg, remoteCoreId, remoteCoreEndPt);
            break;
    }
    if(status != ICVE_OK)
    {
        DebugP_log("Control Message failed: %d\r\n", status);
    }
}

static int32_t AppCtrl_handleReq(Icve_reqMsg reqMsg, uint16_t remoteCoreId, uint16_t remoteCoreEndPt)
{
    int32_t status = ICVE_OK;
    Icve_message msg;

    switch(reqMsg.type)
    {
        case ICVE_REQ_SHM_INFO:
            status = AppCtrl_respShrMemRegion(remoteCoreId, remoteCoreEndPt);
            break;

        case ICVE_REQ_SET_MAC_ADDR:
            status = AppCtrl_addMacAddr2fbd(reqMsg.mac_addr);

            if(status == ICVE_OK)
            {
                msg.resp_msg.type = ICVE_RESP_SET_MAC_ADDR;
                AppCtrl_sendRespMsg(&msg, remoteCoreId, remoteCoreEndPt);
            }
            break;

        case ICVE_REQ_ADD_MC_ADDR:
            status = AppCtrl_addMcastAddr(reqMsg.mac_addr);
            if(status == ICVE_OK)
            {
                msg.resp_msg.type = ICVE_RESP_ADD_MC_ADDR;
                AppCtrl_sendRespMsg(&msg, remoteCoreId, remoteCoreEndPt);
            }
            break;

        case ICVE_REQ_DEL_MC_ADDR:
            status = AppCtrl_delMcastAddr(reqMsg.mac_addr);

            if(status == ICVE_OK)
            {
                msg.resp_msg.type = ICVE_RESP_DEL_MC_ADDR;
                AppCtrl_sendRespMsg(&msg, remoteCoreId, remoteCoreEndPt);
            }
            break;

        default:
            break;
    }

    return status;
}

static int32_t AppCtrl_handleNotify(Icve_notifyMsg notifyMsg, uint16_t remoteCoreId, uint16_t remoteCoreEndPt)
{
    int32_t status = ICVE_OK;
    DebugP_log("Notify Message Handling is not available\r\n");
    //netif_set_link_down for IF down case
    return status;
}

static int32_t AppCtrl_handleResponse(Icve_respMsg respMsg, uint16_t remoteCoreId, uint16_t remoteCoreEndPt)
{
    int32_t status = ICVE_OK;
    DebugP_log("Response Messages Handling is not available for this core\r\n");
    return status;
}

static int32_t AppCtrl_respShrMemRegion(uint16_t remoteCoreId, uint16_t remoteCoreEndPt)
{
    int32_t status = ICVE_OK;
    Icve_respMsg respMsg;
    Icve_message msg;

    status = App_getSharedMemInfo(&respMsg, remoteCoreId);

    if(status == ICVE_OK)
    {
        respMsg.type = ICVE_RESP_SHM_INFO;
        msg.resp_msg = respMsg;
        AppCtrl_sendRespMsg(&msg, remoteCoreId, remoteCoreEndPt);
    }
    return status;
}

static int32_t AppCtrl_sendRespMsg(Icve_message* pMsg, uint16_t remoteCoreId, uint16_t remoteCoreEndPt)
{
    int32_t status;
    pMsg->msg_hdr.msg_type = ICVE_RESPONSE_MSG;
    pMsg->msg_hdr.src_id   = App_getSelfCoreId();


    status = RPMessage_send(pMsg, sizeof(Icve_message),
                            remoteCoreId, remoteCoreEndPt,
                            RPMessage_getLocalEndPt(&gIpcRecvMsgObject[0]),
                            SystemP_WAIT_FOREVER);

    DebugP_log("Response message sent\r\n");
    return status;
}

/*! Test code. Not used in the example. Used for Debug purposes only */
void AppCtrl_createSendTask()
{
    int32_t status;
    TaskP_Params taskParams;
    RPMessage_CreateParams createParams;

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = IPC_RPMESSAGE_ENDPT_CHRDEV_PING;
    status = RPMessage_construct(&gIpcRecvMsgObject[1], &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    /* Create the tasks which will handle the ping service */
    TaskP_Params_init(&taskParams);
    taskParams.name      = "RPMESSAGE_CTRL_SEND";
    taskParams.stackSize = IPC_RPMESSAGE_TASK_STACK_SIZE;
    taskParams.stack     = gIpcTaskStack[1];
    taskParams.priority  = (IPC_RPMESSAGE_TASK_PRI-2U);
    /* we use the same task function for echo but pass the appropiate rpmsg handle to it, to echo messages */
    taskParams.args      = &gIpcRecvMsgObject[1];
    taskParams.taskMain  = AppCtrl_sendTask;

    status = TaskP_construct(&gIpcTask[1], &taskParams);
    DebugP_assert(status == SystemP_SUCCESS);
}

/*! Test code. Not used in the example. Used for Debug purposes only */
static void AppCtrl_sendTask(void* args)
{
    int32_t status;
    char msgBuf[IPC_RPMESSAGE_MAX_MSG_SIZE];
    uint16_t msgSize, remoteCoreId, remoteCoreEndPt;

    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    Icve_macAddr macAddr;
    macAddr.macAddr[0] = 0x00;
    macAddr.macAddr[1] = 0x01;
    macAddr.macAddr[2] = 0x02;
    macAddr.macAddr[3] = 0x04;
    macAddr.macAddr[4] = 0x05;
    macAddr.macAddr[5] = 0x06;
    AppCtrl_sendAddMacAddrReq(macAddr, ICVE_REQ_SET_MAC_ADDR);

    status = RPMessage_recv(&gIpcRecvMsgObject[1],
                            msgBuf, &msgSize,
                            &remoteCoreId, &remoteCoreEndPt,
                            SystemP_WAIT_FOREVER);
    if(status == SystemP_SUCCESS)
    {
        DebugP_log("Received Response Pkt\r\n");
        DebugP_log("[IPC RPMSG ECHO] Remote Core ID    = %d \r\n", remoteCoreId);
        DebugP_log("[IPC RPMSG ECHO] Remote Core EndPt = %d \r\n", remoteCoreEndPt);
    }

    Icve_message* mBuf = (Icve_message *)(msgBuf);
    status = RPMessage_recv(&gIpcRecvMsgObject[1],
                            msgBuf, &msgSize,
                            &remoteCoreId, &remoteCoreEndPt,
                            SystemP_WAIT_FOREVER);

    if(mBuf->msg_hdr.msg_type == ICVE_NOTIFY_MSG)
    {
        if(mBuf->notify_msg.type == ICVE_NOTIFY_REMOTE_IP)
        {
            AppTcp_startClient(mBuf->notify_msg.ipAddr);
        }
    }

    vTaskDelete(NULL);
}

void AppCtrl_sendAddMacAddrReq(Icve_macAddr args, uint32_t type)
{
    Icve_macAddr macAddr = args;
    int32_t status;
    Icve_message pMsg;

    pMsg.msg_hdr.msg_type = ICVE_REQUEST_MSG;
    pMsg.msg_hdr.src_id   = App_getSelfCoreId();
    pMsg.req_msg.type     = type;
    pMsg.req_msg.mac_addr = macAddr;

    status = RPMessage_send(&pMsg, sizeof(Icve_message),
                            CSL_CORE_ID_R5FSS0_0, IPC_RPMESSAGE_ENDPT_PING,
                            RPMessage_getLocalEndPt(&gIpcRecvMsgObject[1]),
                            SystemP_WAIT_FOREVER);
    DebugP_assert(status==SystemP_SUCCESS);
}

void AppCtrl_sendIPNotify()
{
    int32_t status;
    Icve_message pMsg;

    pMsg.msg_hdr.msg_type  = ICVE_NOTIFY_MSG;
    pMsg.msg_hdr.src_id    = App_getSelfCoreId();
    pMsg.notify_msg.type   = ICVE_NOTIFY_REMOTE_IP;
    pMsg.notify_msg.ipAddr = netif_default->ip_addr;

    status = RPMessage_send(&pMsg, sizeof(Icve_message),
                            CSL_CORE_ID_R5FSS0_1, IPC_RPMESSAGE_ENDPT_CHRDEV_PING,
                            RPMessage_getLocalEndPt(&gIpcRecvMsgObject[1]),
                            SystemP_WAIT_FOREVER);
    DebugP_assert(status==SystemP_SUCCESS);
}

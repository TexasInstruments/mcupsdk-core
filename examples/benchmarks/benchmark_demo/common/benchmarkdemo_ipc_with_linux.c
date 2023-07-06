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
 *  "AS IS" AND ANY EXPgResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

#include "benchmarkdemo.h"
#include "ti_drivers_open_close.h"

/* This is used to talk to A53 user space. */
#define APP_IPC_RPMESSAGE_SERVICE_CHRDEV               "rpmsg_chrdev"
#define APP_IPC_RPMESSAGE_ENDPT_CHRDEV_BENCHMARKDEMO   (14U)

/* RPMessage object used to receive messages */
static RPMessage_Object gAppIpcRecvMsgObject;

/* global used to hold the CPU ID and endpoint to which we need to send message to 
 * when we get a message we, know the sender CPU ID and end point and we use this
 * to reply back 
 */
static uint16_t gAppIpcRemoteCoreId, gAppIpcRemoteCoreEndPt;

void App_ipcInit()
{
    int32_t status;
    RPMessage_CreateParams createParams;

    /* This API MUST be called by applications when its ready to talk to Linux */
    status = RPMessage_waitForLinuxReady(SystemP_WAIT_FOREVER);
    DebugP_assert(status==SystemP_SUCCESS);

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = APP_IPC_RPMESSAGE_ENDPT_CHRDEV_BENCHMARKDEMO;
    status = RPMessage_construct(&gAppIpcRecvMsgObject, &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    status = RPMessage_announce(CSL_CORE_ID_A53SS0_0, APP_IPC_RPMESSAGE_ENDPT_CHRDEV_BENCHMARKDEMO, APP_IPC_RPMESSAGE_SERVICE_CHRDEV);
    DebugP_assert(status==SystemP_SUCCESS);
}

void App_ipcRecv(void *buf, uint16_t *bufSize, uint16_t maxSize)
{
    int32_t status;
    uint16_t recvMsgSize = maxSize;

    *bufSize = 0;

    status = RPMessage_recv(&gAppIpcRecvMsgObject,
                buf, &recvMsgSize,
                &gAppIpcRemoteCoreId, &gAppIpcRemoteCoreEndPt,
                SystemP_NO_WAIT);
    if(status == SystemP_SUCCESS)
    {
        *bufSize=recvMsgSize;
    }
}

void App_ipcSend(void *buf, uint16_t bufSize)
{
    int32_t status;

    status = RPMessage_send(
                buf, bufSize,
                gAppIpcRemoteCoreId, gAppIpcRemoteCoreEndPt,
                RPMessage_getLocalEndPt(&gAppIpcRecvMsgObject),
                SystemP_WAIT_FOREVER);
    DebugP_assert(status==SystemP_SUCCESS);
}

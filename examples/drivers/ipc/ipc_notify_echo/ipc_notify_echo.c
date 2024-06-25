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
#include <inttypes.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/ipc_notify.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* This example shows message exchange between multiple cores.
 *
 * One of the core is designated as the 'main' core
 * and other cores are desginated as `remote` cores.
 *
 * The main core initiates IPC with remote core's by sending it a message.
 * The remote cores echo the same message to the main core.
 *
 * The main core repeats this for gMsgEchoCount iterations.
 *
 * In each iteration of message exchange, the message value is incremented.
 *
 * When iteration count reaches gMsgEchoCount, a semaphore is posted and
 * the pending thread/task on that core is unblocked.
 *
 * When a message or its echo is received, a user registered callback is invoked.
 * The message is echoed from within the user callback itself.
 *
 * This is a example message exchange, in final systems, user can do more
 * sophisticated message exchanges as needed for their applications.
 */

/* number of iterations of message exchange to do */
uint32_t gMsgEchoCount = 1000000u;
/* client ID that is used to send and receive messages */
uint32_t gClientId = 4u;

#if defined(SOC_AM64X)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_R5FSS1_0,
    CSL_CORE_ID_R5FSS1_1,
    CSL_CORE_ID_M4FSS0_0,
    CSL_CORE_ID_A53SS0_0,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#if defined(SOC_AM243X)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_R5FSS1_0,
    CSL_CORE_ID_R5FSS1_1,
    CSL_CORE_ID_M4FSS0_0,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
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

#if defined(SOC_AM273X) || defined(SOC_AWR294X)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_C66SS0,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

/* semaphore's used to indicate a main core has finished all message exchanges */
SemaphoreP_Object gMainDoneSem[CSL_CORE_ID_MAX];

/* semaphore used to indicate a remote core has finished all message xchange */
SemaphoreP_Object gRemoteDoneSem;

void ipc_notify_msg_handler_main_core(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args)
{
    /* increment msgValue and send it back until gMsgEchoCount iterations are done */
    if(msgValue != (gMsgEchoCount-1))
    {
        /* send new message to remote core, that echod our message */
        msgValue++;
        IpcNotify_sendMsg(remoteCoreId, gClientId, msgValue, 1);
    }
    else
    {
        /* there is one semaphore for each core ID, so post the semaphore for the remote core that
         * has finished all message exchange iterations
         */
        SemaphoreP_post(&gMainDoneSem[remoteCoreId]);
    }
}

void ipc_notify_echo_main_core_start(void)
{
    int32_t status;
    uint32_t i, numRemoteCores;

    /* create completion semaphores for all cores */
    for(i=0; i < CSL_CORE_ID_MAX; i++)
    {
        SemaphoreP_constructBinary(&gMainDoneSem[i], 0);
    }

    /* register a handler to receive messages */
    status = IpcNotify_registerClient(gClientId, ipc_notify_msg_handler_main_core, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("[IPC NOTIFY ECHO] Message exchange started by main core !!!\r\n");

    for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++)
    {
        uint32_t msgValue = 0;
        /* send message's to all participating core's, wait for message to be put in HW FIFO */
        status = IpcNotify_sendMsg(gRemoteCoreId[i], gClientId, msgValue, 1);
        DebugP_assert(status==SystemP_SUCCESS);
    }

    /* wait for all messages to be echo'ed back */
    numRemoteCores = 0;
    for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++)
    {
        SemaphoreP_pend(&gMainDoneSem[ gRemoteCoreId[i] ], SystemP_WAIT_FOREVER);
        numRemoteCores++;
    }

    DebugP_log("[IPC NOTIFY ECHO] All echoed messages received by main core from %d remote cores !!!\r\n", numRemoteCores);
    DebugP_log("[IPC NOTIFY ECHO] Messages sent to each core = %d \r\n", gMsgEchoCount);
    DebugP_log("[IPC NOTIFY ECHO] Number of remote cores = %d \r\n", numRemoteCores);
    DebugP_log("All tests have passed!!\r\n");
}

void ipc_notify_msg_handler_remote_core(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args)
{
    /* on remote core, we have registered handler on the same client ID and current core client ID */
    IpcNotify_sendMsg(remoteCoreId, localClientId, msgValue, 1);

    /* if all messages received then post semaphore to exit */
    if(msgValue == (gMsgEchoCount-1))
    {
        SemaphoreP_post(&gRemoteDoneSem);
    }
}

void ipc_notify_echo_remote_core_start(void)
{
    int32_t status;

    SemaphoreP_constructBinary(&gRemoteDoneSem, 0);

    /* register a handler to receive messages */
    status = IpcNotify_registerClient(gClientId, ipc_notify_msg_handler_remote_core, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("[IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!\r\n");

    /* wait for all messages to be echo'ed back */
    SemaphoreP_pend(&gRemoteDoneSem, SystemP_WAIT_FOREVER);

    DebugP_log("[IPC NOTIFY ECHO] Remote core has echoed all messages !!!\r\n");
}

void ipc_notify_echo_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    if(IpcNotify_getSelfCoreId()==gMainCoreId)
    {
        ipc_notify_echo_main_core_start();
    }
    else
    {
        ipc_notify_echo_remote_core_start();
    }

    Board_driversClose();
    /* We dont close drivers to let the UART driver remain open and flush any pending messages to console */
    /* Drivers_close(); */
}

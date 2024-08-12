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
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <security/security_common/drivers/hsmclient/hsmclient.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/ipc_notify.h>
#include <drivers/ipc_rpmsg.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <unity.h>

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
volatile uint32_t gMsgEchoCount = 100000u;

#if defined(SOC_AM243X)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#if defined(SOC_AM64X)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#if defined(SOC_AM273X) || defined(SOC_AWR294X)
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

/* Flag that is used to check data abort exception*/
uint32_t gDataAbortReceived = 0;

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

extern FirewallRegionReq_t gMpuFirewallRegionConfig_0[FIREWALL_ARRAY0_NUM_REGIONS] ;

#define APP_CLIENT_ID                  (0x02)

/* Strong declaration of user defined data abort exception */
extern void HwiP_user_data_abort_handler_c(DFSR dfsr,ADFSR adfsr,volatile uint32_t dfar,volatile uint32_t address,volatile uint32_t spsr);

void ipc_safeipc_test_main_core()
{
    RPMessage_CreateParams createParams;
    uint32_t msg, i, numRemoteCores;
    uint64_t curTime;
    char msgBuf[MAX_MSG_SIZE];
    int32_t status;
    uint16_t remoteCoreId, remoteCoreEndPt, msgSize;

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = MAIN_CORE_ACK_REPLY_END_PT;
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

    for(msg=0; msg<gMsgEchoCount; msg++)
    {
        snprintf(msgBuf, MAX_MSG_SIZE-1, "%d", msg);
        msgBuf[MAX_MSG_SIZE-1] = 0;
        msgSize = strlen(msgBuf) + 1; /* count the terminating char as well */

        /* send the same messages to all cores */
        for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++ )
        {
            status = RPMessage_send(
                msgBuf, msgSize,
                gRemoteCoreId[i], gRemoteServiceEndPt,
                RPMessage_getLocalEndPt(&gAckReplyMsgObject),
                SystemP_WAIT_FOREVER);
            DebugP_assert(status==SystemP_SUCCESS);
        }
        /* wait for response from all cores */
        for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++ )
        {
            /* set 'msgSize' to size of recv buffer,
            * after return `msgSize` contains actual size of valid data in recv buffer
            */
            msgSize = sizeof(msgBuf);
            status = RPMessage_recv(&gAckReplyMsgObject,
                msgBuf, &msgSize,
                &remoteCoreId, &remoteCoreEndPt,
                SystemP_WAIT_FOREVER);
            DebugP_assert(status==SystemP_SUCCESS);
        }
    }

    curTime = ClockP_getTimeUsec() - curTime;

    RPMessage_destruct(&gAckReplyMsgObject);

    DebugP_log("[IPC RPMSG ECHO] All echoed messages received by main core from %d remote cores !!!\r\n", numRemoteCores);
    DebugP_log("[IPC RPMSG ECHO] Messages sent to each core = %d \r\n", gMsgEchoCount);
    DebugP_log("[IPC RPMSG ECHO] Number of remote cores = %d \r\n", numRemoteCores);
    DebugP_log("[IPC RPMSG ECHO] Total execution time = %" PRId64 " usecs\r\n", curTime);
    DebugP_log("[IPC RPMSG ECHO] One way message latency = %" PRId32 " nsec\r\n",
        (uint32_t)(curTime*1000u/(gMsgEchoCount*numRemoteCores*2)));

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

void ipc_safeipc_test_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
    HsmClient_t client ;
    FirewallReq_t FirewallReqObj;
    uint8_t *uid = malloc(HSM_UID_SIZE) ;

    Drivers_open();
    Board_driversOpen();

    status = HsmClient_register(&client,APP_CLIENT_ID);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Send Request for UID to HSM Server */
    status = HsmClient_getUID(&client, (uint8_t *)uid, SystemP_WAIT_FOREVER);

    /* Configure firewalls */
    FirewallReqObj.regionCount = FIREWALL_ARRAY0_NUM_REGIONS;
    FirewallReqObj.FirewallRegionArr = gMpuFirewallRegionConfig_0;

    status = HsmClient_setFirewall(&client,&FirewallReqObj,SystemP_WAIT_FOREVER);
    DebugP_assert(status==SystemP_SUCCESS);

    UNITY_BEGIN();

    RUN_TEST(ipc_safeipc_test_main_core, 12267, NULL);

    UNITY_END();

    Board_driversClose();
    /* We dont close drivers to let the UART driver remain open and flush any pending messages to console */
    /* Drivers_close(); */
}

/* Strong definition of user defined data abort exception. This function will be called incase of any data abort */
void HwiP_user_data_abort_handler_c(DFSR dfsr,ADFSR adfsr,volatile uint32_t dfar,volatile uint32_t address,volatile uint32_t spsr){
    gDataAbortReceived = 1;
}

void setUp(void)
{
}

void tearDown(void)
{
}
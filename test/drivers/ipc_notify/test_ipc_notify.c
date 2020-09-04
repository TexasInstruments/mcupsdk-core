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
#include <drivers/soc.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/ipc_notify.h>
#include <unity.h>
#include "ti_drivers_open_close.h"

/* number of iterations of message exchange to do */
uint32_t gMsgEchoCount = 10000u;
/* client ID that is used to receive messages in Any to Any test */
uint32_t gRxClientId = 2u;
/* client ID that is used to receive ACK messages in Any to Any test */
uint32_t gAckClientId = 3u;

/* client ID used on remote core that is used to receive and echo messages to main more */
uint32_t gServerClientId = 4u;
/* client ID on main core that is used to receive ACK messages from remote core server */
uint32_t gClientId = 2u;
/* client ID on which NO handler is registered */
uint32_t gNullClientId = 5u;

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

/* semaphore's used to indicate a core has recevied all ACK messages exchanges from each core in Any to Any test */
SemaphoreP_Object gAckDoneSem[CSL_CORE_ID_MAX];

/* semaphore used to indicate a core has recevied all messages in Any to Any test, and client=server tests */
SemaphoreP_Object gRxDoneSem;

uint64_t gOnewayMsgLatency[CSL_CORE_ID_MAX] = {0};

/* message handler to receive ack's in any to any test */
void test_ipc_notify_ack_msg_handler(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, void *args)
{
    /* increment msgValue and send it back until gMsgEchoCount iterations are done */
    if(msgValue != (gMsgEchoCount-1))
    {
        /* send new message to remote core, that echod our message */
        msgValue++;
        IpcNotify_sendMsg(remoteCoreId, gRxClientId, msgValue, 1);
    }
    else
    {
        /* there is one semaphore for each core ID, so post the semaphore for the remote core that
         * has finished all message exchange iterations
         */
        SemaphoreP_post(&gAckDoneSem[remoteCoreId]);
    }
}

/* message handler to receive messages in any to any test */
void test_ipc_notify_rx_msg_handler(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, void *args)
{
    /* on remote core, we have registered handler on the same client ID and current core client ID */
    IpcNotify_sendMsg(remoteCoreId, gAckClientId, msgValue, 1);

    /* if all messages received then post semaphore to exit */
    if(msgValue == (gMsgEchoCount-1))
    {
        SemaphoreP_post(&gRxDoneSem);
    }
}

/*
 * In this test,
 * - All cores send messages to all other cores
 * - On getting message each core ACKs the message back to the sender core
 *
 * Here in total
 * - gMsgEchoCount * (NUM_CORE-1) messages are sent from a core
 * - gMsgEchoCount * (NUM_CORE-1) messages are received by a core
 * - gMsgEchoCount * (NUM_CORE-1) messages are ACK'ed by core
 * - gMsgEchoCount * (NUM_CORE-1) messages are ACK's are recevied by core
 * i.e in total gMsgEchoCount * (NUM_CORE-1) * 2 messages are sent from a core
 *     and gMsgEchoCount * (NUM_CORE-1) * 2 messages are recevied by a core
 */
void test_notifyAnyToAny(void *args)
{
    int32_t status;
    uint32_t i, numRemoteCores;

    /* create completion semaphores */
    for(i=0; i < CSL_CORE_ID_MAX; i++)
    {
        SemaphoreP_constructBinary(&gAckDoneSem[i], 0);
    }
    SemaphoreP_constructBinary(&gRxDoneSem, 0);

    /* register a handler to receive ACK messages */
    status = IpcNotify_registerClient(gAckClientId, test_ipc_notify_ack_msg_handler, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    /* register a handler to receive messages */
    status = IpcNotify_registerClient(gRxClientId, test_ipc_notify_rx_msg_handler, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    DebugP_log("[TEST IPC NOTIFY] Ready for IPC !!!\r\n");

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* send 1st message, subseqent messages are sent in the handler itself */
    for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++)
    {
        if(gRemoteCoreId[i] != IpcNotify_getSelfCoreId())
        {
            uint32_t msgValue = 0;
            /* send message's to all participating core's, wait for message to be put in HW FIFO */
            status = IpcNotify_sendMsg(gRemoteCoreId[i], gRxClientId, msgValue, 1);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        }
    }

    /* wait for all messages to be echo'ed back */
    numRemoteCores = 0;
    for(i=0; gRemoteCoreId[i]!=CSL_CORE_ID_MAX; i++)
    {
        if(gRemoteCoreId[i] != IpcNotify_getSelfCoreId())
        {
            status = SemaphoreP_pend(&gAckDoneSem[ gRemoteCoreId[i] ], SystemP_WAIT_FOREVER);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
            numRemoteCores++;
        }
    }

    /* wait for all messages to be recevied */
    status = SemaphoreP_pend(&gRxDoneSem, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* wait for all cores to have executed upto this point
     * if we dont get sync from cores then message exchange between some core's has failed
     */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* unregister handlers */
    IpcNotify_unregisterClient(gAckClientId);
    IpcNotify_unregisterClient(gRxClientId);

    /* delete semaphores */
    for(i=0; i < CSL_CORE_ID_MAX; i++)
    {
        SemaphoreP_destruct(&gAckDoneSem[i]);
    }
    SemaphoreP_destruct(&gRxDoneSem);
}

/* server handler on remote core, it simply echos the message to gClientId */
void test_ipc_notify_server_msg_handler(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, void *args)
{
    /* send ACK to sender */
    IpcNotify_sendMsg(remoteCoreId, gClientId, msgValue, 1);
}

/* client handler on main core core, it sneds a message back to server untll gMsgEchoCount
 * messages have been exchanged and then posts a semaphore to indicate done to main core
 */
void test_ipc_notify_client_msg_handler(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, void *args)
{
    /* increment msgValue and send it back until gMsgEchoCount iterations are done */
    if(msgValue != (gMsgEchoCount-1))
    {
        /* send new message to remote core, that echod our message */
        msgValue++;
        IpcNotify_sendMsg(remoteCoreId, gServerClientId, msgValue, 1);
    }
    else
    {
        SemaphoreP_Object *pDoneSem = (SemaphoreP_Object *)args;

        SemaphoreP_post(pDoneSem);
    }
}

/*
 *   In this test
 *   - We exchange messages between main core and one remote core
 *   - The remote core simply echo's back the message
 *   - We use this to measure IPC performance between a pair of cores,
 *     here one core is always the main core, but that should not affect
 *     performance numbers
 */
void test_notifyOneToOne(void *args)
{
    uint32_t remoteCoreId = (uint32_t)args;
    int32_t status;
    uint64_t curTime;

    /* create done semaphore */
    SemaphoreP_constructBinary(&gRxDoneSem, 0);

    /* register a handler to receive ACK messages, also pass semaphore handle as a arg */
    status = IpcNotify_registerClient(gClientId, test_ipc_notify_client_msg_handler, &gRxDoneSem);
    DebugP_assert(status==SystemP_SUCCESS);

    curTime = ClockP_getTimeUsec();

    /* send 1st message, subseqent messages are sent in the handler itself */
    {
        uint32_t msgValue = 0;
        /* send message to server remote core, wait for message to be put in HW FIFO */
        status = IpcNotify_sendMsg(remoteCoreId, gServerClientId, msgValue, 1);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    }

    /* wait for all messages to be echo'ed back */
    status = SemaphoreP_pend(&gRxDoneSem, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    curTime = ClockP_getTimeUsec() - curTime;

    /* unregister handler */
    IpcNotify_unregisterClient(gClientId);

    /* delete semaphores */
    SemaphoreP_destruct(&gRxDoneSem);

    gOnewayMsgLatency[remoteCoreId] = curTime;
}

/* client handler on main core core, when gMsgEchoCount
 * messages have been exchanged it posts a semaphore to indicate done to main core
 */
void test_ipc_notify_client_back_to_back_msg_handler(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, void *args)
{
    if(msgValue != (gMsgEchoCount-1))
    {
        /* no nothing, until gMsgEchoCount iterations are done */
    }
    else
    {
        SemaphoreP_Object *pDoneSem = (SemaphoreP_Object *)args;

        SemaphoreP_post(pDoneSem);
    }
}

/*
 * In this test,
 * - We dont ack in handler
 * - Rather the main thread sends messages back to back in a tight loop,
 * - Here the HW fifo becomes full and API waits for FIFO to get empty
 */
void test_notifyOneToOneBackToBack(void *args)
{
    uint32_t remoteCoreId = (uint32_t)args;
    int32_t status;
    uint32_t i;
    uint64_t curTime;
    uint32_t oldDebugLogZone;

    /* create Ack done */
    SemaphoreP_constructBinary(&gRxDoneSem, 0);

    /* register a handler to receive ACK messages, use args as well here */
    status = IpcNotify_registerClient(gClientId, test_ipc_notify_client_back_to_back_msg_handler,
                        &gRxDoneSem);
    DebugP_assert(status==SystemP_SUCCESS);

    DebugP_log("[TEST IPC NOTIFY] Message exchange started with remote core %s ... !!!\r\n", SOC_getCoreName(remoteCoreId));

    /* disable warning logs since we are testing for those, so it will clutter the output */
    oldDebugLogZone = DebugP_logZoneDisable(DebugP_LOG_ZONE_WARN);

    curTime = ClockP_getTimeUsec();

    for(i=0 ; i<gMsgEchoCount; i++)
    {
        uint32_t msgValue = i;
        /* send message to server remote core, wait for message to be put in HW FIFO */
        status = IpcNotify_sendMsg(remoteCoreId, gServerClientId, msgValue, 1);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    }

    /* wait for all messages to be echo'ed back */
    status = SemaphoreP_pend(&gRxDoneSem, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    curTime = ClockP_getTimeUsec() - curTime;

    DebugP_logZoneRestore(oldDebugLogZone);

    /* unregister handlers */
    IpcNotify_unregisterClient(gClientId);

    /* delete semaphores */
    SemaphoreP_destruct(&gRxDoneSem);

    DebugP_log("[TEST IPC NOTIFY] Messages sent = %d \r\n", gMsgEchoCount);
    DebugP_log("[TEST IPC NOTIFY] Total execution time = %" PRId64 " usecs\r\n", curTime);
    DebugP_log("[TEST IPC NOTIFY] Avg one-way message latency = %" PRId32 " nsec\r\n",
        (uint32_t)(curTime*1000u/(gMsgEchoCount*2)));
}

/*
 * In this test
 * - We check various error conditions for the APIs
 */
void test_notifyErrorChecks(void *args)
{
    uint32_t remoteCoreId = (uint32_t)args;
    int32_t status;
    uint32_t i;
    uint32_t hwFifoFullCount, oldDebugLogZone;

    /* disable error and warning logs since we are testing for those, so it will clutter the output */
    oldDebugLogZone = DebugP_logZoneDisable(DebugP_LOG_ZONE_WARN | DebugP_LOG_ZONE_ERROR);

    /* send message to self */
    status = IpcNotify_sendMsg(IpcNotify_getSelfCoreId(), gServerClientId, 0, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    /* send message to invalid core */
    status = IpcNotify_sendMsg(CSL_CORE_ID_MAX, gServerClientId, 0, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    /* register to invalid client ID */
    status = IpcNotify_registerClient(IPC_NOTIFY_CLIENT_ID_MAX,
                    test_ipc_notify_client_back_to_back_msg_handler, NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    /* register to same client ID twice */
    status = IpcNotify_registerClient(gClientId,
                    test_ipc_notify_client_back_to_back_msg_handler, NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status = IpcNotify_registerClient(gClientId,
                    test_ipc_notify_client_back_to_back_msg_handler, NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    /* unregister multiple time is ok */
    status = IpcNotify_unregisterClient(gClientId);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status = IpcNotify_unregisterClient(gClientId);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* send back to back message with, waitForFifoNotFull as 0
     * at many points HW fifo MUST become full
     */
    hwFifoFullCount = 0;
    for(i=0 ; i<gMsgEchoCount; i++)
    {
        /* send message to remote core which does have any registered handler,
           DO NOT wait for message to be put in HW FIFO */
        status = IpcNotify_sendMsg(remoteCoreId, gNullClientId, 0, 0);
        if(status==SystemP_TIMEOUT)
        {
            hwFifoFullCount++;
        }
    }
    /* send one message waiting for space in HW FIFO just to check a success case after failure */
    status = IpcNotify_sendMsg(remoteCoreId, gNullClientId, 0, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    DebugP_log("[TEST IPC NOTIFY] Messages sent = %d \r\n", gMsgEchoCount);
    DebugP_log("[TEST IPC NOTIFY] HW FIFO Full count = %d \r\n", hwFifoFullCount);

    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(0, hwFifoFullCount);

    DebugP_logZoneRestore(oldDebugLogZone);
}

/* This code executes on all remote core, i.e not on main core */
void test_ipc_remote_core_start()
{
    int32_t status;

    UNITY_BEGIN();

    /* register a handler which acts a server to echo messages from main core */
    status = IpcNotify_registerClient(gServerClientId, test_ipc_notify_server_msg_handler, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    test_notifyAnyToAny(NULL);

    /* wait for ever at remote core, now onwards main core will initaite all requests
     * to the server handler
     */
    ClockP_sleep(SystemP_WAIT_FOREVER);

    UNITY_END();
}

/* This code executes on main core, i.e not on remote core */
void test_ipc_main_core_start()
{
    uint32_t i;

    UNITY_BEGIN();

    /* This MUST be the first test to run */
    RUN_TEST(test_notifyAnyToAny, 307, NULL);
    #if defined(SOC_AM64X) || defined(SOC_AM243X)
    RUN_TEST(test_notifyOneToOne, 308, (void*)CSL_CORE_ID_R5FSS0_1);
    RUN_TEST(test_notifyOneToOne, 311, (void*)CSL_CORE_ID_M4FSS0_0);
    RUN_TEST(test_notifyOneToOne, 309, (void*)CSL_CORE_ID_R5FSS1_0);
    RUN_TEST(test_notifyOneToOne, 310, (void*)CSL_CORE_ID_R5FSS1_1);
    #if defined(SOC_AM64X)
    RUN_TEST(test_notifyOneToOne, 1650, (void*)CSL_CORE_ID_A53SS0_0);
    RUN_TEST(test_notifyOneToOneBackToBack, 1649, (void*)CSL_CORE_ID_A53SS0_0);
    #endif
    RUN_TEST(test_notifyOneToOneBackToBack, 312, (void*)CSL_CORE_ID_R5FSS0_1);
    RUN_TEST(test_notifyOneToOneBackToBack, 313, (void*)CSL_CORE_ID_M4FSS0_0);
    RUN_TEST(test_notifyErrorChecks, 314, (void*)CSL_CORE_ID_R5FSS0_1);
    #endif
    #if defined(SOC_AM263X)
    RUN_TEST(test_notifyOneToOne, 308, (void*)CSL_CORE_ID_R5FSS0_1);
    RUN_TEST(test_notifyOneToOne, 309, (void*)CSL_CORE_ID_R5FSS1_0);
    RUN_TEST(test_notifyOneToOne, 310, (void*)CSL_CORE_ID_R5FSS1_1);
    RUN_TEST(test_notifyOneToOneBackToBack, 312, (void*)CSL_CORE_ID_R5FSS0_1);
    RUN_TEST(test_notifyErrorChecks, 314, (void*)CSL_CORE_ID_R5FSS0_1);
    #endif
    #if defined(SOC_AM273X) || defined(SOC_AWR294X)
    RUN_TEST(test_notifyOneToOne, 308, (void*)CSL_CORE_ID_R5FSS0_1);
    RUN_TEST(test_notifyOneToOne, 1868, (void*)CSL_CORE_ID_C66SS0);
    RUN_TEST(test_notifyOneToOneBackToBack, 312, (void*)CSL_CORE_ID_R5FSS0_1);
    RUN_TEST(test_notifyOneToOneBackToBack, 1869, (void*)CSL_CORE_ID_C66SS0);
    RUN_TEST(test_notifyErrorChecks, 314, (void*)CSL_CORE_ID_R5FSS0_1);
    #endif

    DebugP_log("\n[TEST IPC NOTIFY] Performance Numbers Print Start\r\n\n");
    DebugP_log("- %u messages are sent and average one way message latency is measured\r\n\n", gMsgEchoCount);
    DebugP_log("Local Core  | Remote Core | Average Message Latency (us)\r\n");
    DebugP_log("------------|-------------|------------------------------\r\n");
    for (i=0; i<CSL_CORE_ID_MAX; i++) {
        if (i != gMainCoreId) {
            DebugP_log(" %s\t| %s\t| %5.2f\r\n", SOC_getCoreName(gMainCoreId), SOC_getCoreName(i),
                ((float)(gOnewayMsgLatency[i]*1000/(gMsgEchoCount*2))/1000));
        }
    }
    DebugP_log("\n[TEST IPC NOTIFY] Performance Numbers Print End\r\n");

    UNITY_END();
}

void test_main(void *args)
{
    Drivers_open();

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

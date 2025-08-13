/*
 *  Copyright (C) 2024-25 Texas Instruments Incorporated
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
 *  \file test_mcan_canfd.c
 *
 *  \brief This file contains mcan test code.
 * 
 * \details  This UT demonstrates the MCAN operation configured in different 
 *           configurations and all possible MCAN instances that can be 
 *           configured. This example sends a known data of length 
 *           APP_MCAN_DATASIZE and then receives the same in RX mode. 
 *           Internal pad level loopback mode is enabled to receive data.
 *           When transfer is completed, TX and RX buffer data are compared.
 *           If data is matched, test result is passed otherwise failed.
 */

#include "string.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TaskP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <unity.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/mcan/v0/canfd.h>
#include "test_mcan_canfd.h"
#include <drivers/mcan.h>
#include "FreeRTOS.h"
#include "task.h"

/* Task Macros */
#define CANFD_TASK_PRIORITY   (8U)
#define CANFD_TASK_STACK_SIZE (4U * 1024U)

uint8_t rxData[MCAN_APP_TEST_DATA_SIZE] = {0};
uint8_t txData[MCAN_APP_TEST_DATA_SIZE];
uint8_t rxData1[MCAN_APP_TEST_DATA_SIZE] = {0};
uint8_t txData1[MCAN_APP_TEST_DATA_SIZE];

uint32_t  gFlagInstance1 = 0U;
int32_t   gCancelTestStatus = SystemP_SUCCESS;
CANFD_Config *config;
CANFD_Attrs  *attrs;   
CANFD_Handle  canfdHandle = NULL;
CANFD_Handle  canfdHandle1 = NULL;
CANFD_MsgObjHandle  gTxMsgObjHandle, gRxMsgObjHandle;

static SemaphoreP_Object gMcanTxDoneSem, gMcanRxDoneSem;
static SemaphoreP_Object gMcanTxDoneSem1, gMcanRxDoneSem1;

/* Semaphore to track end of rx_task and tx_task */
SemaphoreP_Object gCanfdWriteTaskDoneSemaphoreObj;
SemaphoreP_Object gCanfdWriteTaskCancelDoneSemaphoreObj;

uint8_t gCanfdWriteTaskStack[CANFD_TASK_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gCanfdWriteTaskObject;

uint8_t gCanfdWriteCancelTaskStack[CANFD_TASK_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gCanfdWriteCancelTaskObject;

static void test_canfd_loopback(void *args);
static void test_canfd_loopback_msg_lost(void *args);
static void test_canfd_loopback_polled(void *args);
static void test_canfd_loopback_dma(void *args);
static void test_canfd_loopback_perf(void *args);
static void test_canfd_loopback_cancel(void *args);
static void test_canfd_loopback_cancel_transfer(void *args);
static void test_canfd_loopback_multi_instances(void *args);
static void test_canfd_loopback_cancel_cancel(void *args);
static void App_CANFDPrintRevID(MCAN_RevisionId *revId);
static void test_canfd_set_params(CANFD_TestParams *testParams, uint32_t tcId);
void App_CANFD_TransferCallback(void *args, CANFD_Reason reason);
void App_CANFD_ErrorCallback(void *args, CANFD_Reason reason, CANFD_ErrStatusResp* errStatusResp);
static void test_canfd_getBitRate(void *args);
static void test_canfd_get_revId(void *args);
static void test_canfd_endianness(void *args);
static void test_canfd_TxRx_pin_state(void *args);
static void test_canfd_Clk_Stop_Req_Test(void *args);
static void test_canfd_TS_Reset(void *args);
void App_delayFunc(uint32_t timeout);

void test_main(void *args)
{
    CANFD_TestParams  testParams;

    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    test_canfd_set_params(&testParams, 13920);
    RUN_TEST(test_canfd_loopback, 13920, (void*)&testParams);
    test_canfd_set_params(&testParams, 13921);
    RUN_TEST(test_canfd_loopback, 13921, (void*)&testParams);
    test_canfd_set_params(&testParams, 13922);
    RUN_TEST(test_canfd_loopback, 13922, (void*)&testParams);
    test_canfd_set_params(&testParams, 13923);
    RUN_TEST(test_canfd_loopback, 13923, (void*)&testParams);
    test_canfd_set_params(&testParams, 13924);
    RUN_TEST(test_canfd_loopback, 13924, (void*)&testParams);
    test_canfd_set_params(&testParams, 13925);
    RUN_TEST(test_canfd_loopback, 13925, (void*)&testParams);
    test_canfd_set_params(&testParams, 13926);
    RUN_TEST(test_canfd_loopback, 13926, (void*)&testParams);
    test_canfd_set_params(&testParams, 13927);
    //RUN_TEST(test_canfd_loopback, 13927, (void*)&testParams);
    test_canfd_set_params(&testParams, 13928);
    //RUN_TEST(test_canfd_loopback, 13928, (void*)&testParams);
    test_canfd_set_params(&testParams, 13929);
    RUN_TEST(test_canfd_loopback, 13929, (void*)&testParams);
    test_canfd_set_params(&testParams, 13930);
   // RUN_TEST(test_canfd_loopback,  13930, (void*)&testParams);
    test_canfd_set_params(&testParams, 13931);
    RUN_TEST(test_canfd_loopback, 13931, (void*)&testParams);
    test_canfd_set_params(&testParams, 13933);
    RUN_TEST(test_canfd_loopback, 13933, (void*)&testParams);
    test_canfd_set_params(&testParams, 13934);
    RUN_TEST(test_canfd_loopback_msg_lost, 13934, (void*)&testParams);
    test_canfd_set_params(&testParams, 13935);
    RUN_TEST(test_canfd_loopback_msg_lost, 13935, (void*)&testParams);
    test_canfd_set_params(&testParams, 13936);
    RUN_TEST(test_canfd_loopback, 13936, (void*)&testParams);
    test_canfd_set_params(&testParams, 13937);
    RUN_TEST(test_canfd_loopback_perf, 13937, (void*)&testParams);
    test_canfd_set_params(&testParams, 13938);
    RUN_TEST(test_canfd_loopback_perf, 13938, (void*)&testParams);
    test_canfd_set_params(&testParams, 13939);
    RUN_TEST(test_canfd_loopback, 13939, (void*)&testParams);
    test_canfd_set_params(&testParams, 13940);
    RUN_TEST(test_canfd_loopback_cancel, 13940, (void*)&testParams);
    test_canfd_set_params(&testParams, 13941);
    RUN_TEST(test_canfd_loopback, 13941, (void*)&testParams);
    test_canfd_set_params(&testParams, 13942);
    RUN_TEST(test_canfd_loopback, 13942, (void*)&testParams);
    test_canfd_set_params(&testParams, 13943);
  // RUN_TEST(test_canfd_loopback, 13943, (void*)&testParams);
    test_canfd_set_params(&testParams, 13944);
    RUN_TEST(test_canfd_get_revId, 13944, (void*)&testParams);
    test_canfd_set_params(&testParams, 13945);
    RUN_TEST(test_canfd_TxRx_pin_state, 13945, (void*)&testParams);
    test_canfd_set_params(&testParams, 13946);
    RUN_TEST(test_canfd_endianness, 13946, (void*)&testParams);
    test_canfd_set_params(&testParams, 13947);
    RUN_TEST(test_canfd_loopback, 13947, (void*)&testParams);
    test_canfd_set_params(&testParams, 13948);
    RUN_TEST(test_canfd_loopback, 13948, (void*)&testParams);
    test_canfd_set_params(&testParams, 13949);
    RUN_TEST(test_canfd_loopback, 13949, (void*)&testParams);
    test_canfd_set_params(&testParams, 13950);
    RUN_TEST(test_canfd_loopback, 13950, (void*)&testParams);
    test_canfd_set_params(&testParams, 13951);
    RUN_TEST(test_canfd_loopback, 13951, (void*)&testParams);
    test_canfd_set_params(&testParams, 13952);
    RUN_TEST(test_canfd_TS_Reset, 13952, (void*)&testParams);
    test_canfd_set_params(&testParams, 13953);
    RUN_TEST(test_canfd_loopback_polled, 13953, (void*)&testParams);
    test_canfd_set_params(&testParams, 13954);
    RUN_TEST(test_canfd_loopback_multi_instances, 13954, (void*)&testParams);
    test_canfd_set_params(&testParams, 13955);
    RUN_TEST(test_canfd_Clk_Stop_Req_Test, 13955, (void*)&testParams);
    test_canfd_set_params(&testParams, 13956);
    RUN_TEST(test_canfd_getBitRate, 13956, (void*)&testParams);
    test_canfd_set_params(&testParams, 13957);
    RUN_TEST(test_canfd_loopback, 13957, (void*)&testParams);
    test_canfd_set_params(&testParams, 13958);
    RUN_TEST(test_canfd_loopback_polled, 13958, (void*)&testParams);

    UNITY_END();

    Board_driversClose();
    Drivers_close();
}

static void test_canfd_loopback(void *args)
{
    int32_t          status = SystemP_SUCCESS;
    CANFD_Handle     canfdHandle;
    CANFD_TestParams      *testParams = (CANFD_TestParams *)args;
    App_CANFD_TxMsgParams *txMsgParams = testParams->txMsgParams;
    CANFD_MsgObjHandle     txMsgObjHandle = &(testParams->txMsgObject);
    CANFD_MsgObjHandle     rxMsgObjHandle = &(testParams->rxMsgObject);
    CANFD_OpenParams      *canfdOpenParams = &(testParams->openParams);
    CANFD_MessageObject   *txMsgObject = &(testParams->txMsgObject);
    CANFD_MessageObject   *rxMsgObject = &(testParams->rxMsgObject);

    /* Memset Buffers */
    memset(&txData[0U], 0, txMsgParams->dataLength * sizeof(txData[0U]));
    memset(&rxData[0U], 0, txMsgParams->dataLength * sizeof(rxData[0U]));

    CANFD_close(gCanfdHandle[testParams->canfdInstance]);

    for(int8_t i = 0; i < txMsgParams->dataLength; i++)
    {
        txData[i] = txMsgParams->data[i];
    }

    canfdHandle = CANFD_open(testParams->canfdInstance, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);
    status += SemaphoreP_constructBinary(&gMcanTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status += SemaphoreP_constructBinary(&gMcanRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Setup the transmit message object */
    txMsgObject->direction = CANFD_Direction_TX;
    txMsgObject->msgIdType = txMsgParams->msgIdType;
    txMsgObject->txMemType  = txMsgParams->txMemType;
    txMsgObject->dataLength = txMsgParams->dataLength;
    txMsgObject->args       = NULL;

    if(testParams->txMsgParams->msgIdType == CANFD_MCANXidType_11_BIT)
    {
        txMsgObject->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid1;
        txMsgObject->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid2;
        rxMsgObject->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid1;
        rxMsgObject->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid2;
    }
    else
    {
        txMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
        txMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
        rxMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
        rxMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
    }

    /* Setup the receive message object */
    rxMsgObject->direction = CANFD_Direction_RX;
    rxMsgObject->msgIdType = txMsgParams->msgIdType;;
    rxMsgObject->args       = (uint8_t*) rxData;
    rxMsgObject->rxMemType  = txMsgParams->rxMemType;
    rxMsgObject->dataLength = txMsgParams->dataLength;
    rxMsgObject->rxFifoNum  = testParams->rxFifoNum;

    status += CANFD_createMsgObject (canfdHandle, txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Tx message object failed\r\n");
        return;
    }

    status += CANFD_createMsgObject (canfdHandle, rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Rx message object failed\r\n");
        return;
    }

    status += CANFD_read(rxMsgObjHandle, MCAN_APP_TEST_MESSAGE_COUNT, &rxData[0]);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD read in interrupt mode failed\r\n");
        status = SystemP_FAILURE;
    }

    /* Send data over Tx message object */
    status += CANFD_write (txMsgObjHandle,
                        testParams->txMsgObject.startMsgId,
                        CANFD_MCANFrameType_FD,
                        0,
                        &txData[0]);

    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD write in interrupt mode failed\r\n");
        status = SystemP_FAILURE;
    }

    /* Wait for Tx completion */
    SemaphoreP_pend(&gMcanTxDoneSem, SystemP_WAIT_FOREVER);
    /* Wait for Rx completion */
    SemaphoreP_pend(&gMcanRxDoneSem, SystemP_WAIT_FOREVER);

    /* Compare data */
    for(int32_t i = 0U; i < txMsgParams->dataLength; i++)
    {
        if(txData[i] != rxData[i])
        {
            status += SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    SemaphoreP_destruct(&gMcanTxDoneSem);
    SemaphoreP_destruct(&gMcanRxDoneSem);

    status += CANFD_deleteMsgObject(txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_deleteMsgObject(rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Rx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    CANFD_close(canfdHandle);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

/* 
 * The FIFO is configured in Blocking mode. Once the FIFO becomes full, 
 * any subsequent incoming messages will be discarded. Additionally, 
 * the corresponding interrupt flags (Rx FIFO 0 Message Lost or Rx FIFO 1 Message Lost) 
 * will be set to indicate the loss of messages in the respective FIFO.
 * On msg lost ErrorCallback funtion will be called which will post semaphore.
 */
static void test_canfd_loopback_msg_lost(void *args)
{
    int32_t          status = SystemP_SUCCESS;
    CANFD_Handle     canfdHandle;
    CANFD_TestParams      *testParams = (CANFD_TestParams *)args;
    App_CANFD_TxMsgParams *txMsgParams = testParams->txMsgParams;
    CANFD_MsgObjHandle     txMsgObjHandle = &(testParams->txMsgObject);
    CANFD_MsgObjHandle     rxMsgObjHandle = &(testParams->rxMsgObject);
    CANFD_OpenParams      *canfdOpenParams = &(testParams->openParams);
    CANFD_MessageObject   *txMsgObject = &(testParams->txMsgObject);
    CANFD_MessageObject   *rxMsgObject = &(testParams->rxMsgObject);

    /* Memset Buffers */
    memset(&txData[0U], 0, txMsgParams->dataLength * sizeof(txData[0U]));
    memset(&rxData[0U], 0, txMsgParams->dataLength * sizeof(rxData[0U]));

    CANFD_close(gCanfdHandle[testParams->canfdInstance]);

    for(int8_t i = 0; i < txMsgParams->dataLength; i++)
    {
        txData[i] = txMsgParams->data[i];
    }

    canfdHandle = CANFD_open(testParams->canfdInstance, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);
    status += SemaphoreP_constructBinary(&gMcanTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status += SemaphoreP_constructBinary(&gMcanRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Setup the transmit message object */
    txMsgObject->direction = CANFD_Direction_TX;
    txMsgObject->msgIdType = txMsgParams->msgIdType;
    txMsgObject->txMemType  = txMsgParams->txMemType;
    txMsgObject->dataLength = txMsgParams->dataLength;
    txMsgObject->args       = NULL;

    if(testParams->txMsgParams->msgIdType == CANFD_MCANXidType_11_BIT)
    {
        txMsgObject->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid1;
        txMsgObject->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid2;
        rxMsgObject->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid1;
        rxMsgObject->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid2;
    }
    else
    {
        txMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
        txMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
        rxMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
        rxMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
    }

    /* Setup the receive message object */
    rxMsgObject->direction = CANFD_Direction_RX;
    rxMsgObject->msgIdType = txMsgParams->msgIdType;;
    rxMsgObject->args       = (uint8_t*) rxData;
    rxMsgObject->rxMemType  = MCAN_MEM_TYPE_FIFO;
    rxMsgObject->dataLength = txMsgParams->dataLength;
    rxMsgObject->rxFifoNum  = testParams->rxFifoNum;

    /* FIFO Block mode Test - Start */
    DebugP_log("\nFIFO 0/1 Message Lost Test:\r\n");
    /* Accept non-matching messages into FIFO */
    /* Send messages until FIFO condition is reached */
    for(uint32_t iterationCount = 0U; iterationCount < canfdOpenParams->msgRAMConfig.rxFIFO0size; iterationCount++)
    {
        status += CANFD_createMsgObject (canfdHandle, txMsgObjHandle);
        if (status != SystemP_SUCCESS)
        {
            DebugP_log ("Error: CANFD create Tx message object failed\r\n");
            return;
        }

        status += CANFD_createMsgObject (canfdHandle, rxMsgObjHandle);
        if (status != SystemP_SUCCESS)
        {
            DebugP_log ("Error: CANFD create Rx message object failed\r\n");
            return;
        }

        /* Send data over Tx message object */
        status += CANFD_write (txMsgObjHandle,
                                testParams->txMsgObject.startMsgId,
                                CANFD_MCANFrameType_FD,
                                0,
                                &txData[0]);
                                /* Wait for Tx completion */
        SemaphoreP_pend(&gMcanTxDoneSem, SystemP_WAIT_FOREVER);  
    }

    /* Send another message to cause overflow */
    status += CANFD_write (txMsgObjHandle,
                            testParams->txMsgObject.startMsgId,
                            CANFD_MCANFrameType_FD,
                            0,
                            &txData[0]);
    /* Semaphore post will be done from errorCallback function */
    SemaphoreP_pend(&gMcanTxDoneSem, SystemP_WAIT_FOREVER);

    if ((testParams->reason == MCAN_INTR_SRC_RX_FIFO0_MSG_LOST) &&
        (rxMsgObject->rxFifoNum == MCAN_RX_FIFO_NUM_0))
    {
        DebugP_log ("RX FIFO0 message lost test pass");
        status += SystemP_SUCCESS;
    }
    else if ((testParams->reason == MCAN_INTR_SRC_RX_FIFO1_MSG_LOST) &&
             (rxMsgObject->rxFifoNum == MCAN_RX_FIFO_NUM_1))
    {
        DebugP_log ("RX FIFO1 message lost test pass");
        status += SystemP_SUCCESS;
    }
    else
    {
        status += SystemP_FAILURE;
    }
    
    /*  For the last iteration i.e. once you get the RX full interrupt and after that if you want to write any 
        msg in msg RAM with rxFIFO0OpMode is configured to blocking mode, Rx FIFO 0/1 Message Lost interrupt 
        will be triggered.    
    */
    SemaphoreP_destruct(&gMcanTxDoneSem);
    SemaphoreP_destruct(&gMcanRxDoneSem);

    status += CANFD_deleteMsgObject(txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_deleteMsgObject(rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Rx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    CANFD_close(canfdHandle);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

static void test_canfd_loopback_polled(void *args)
{
    int32_t          status = SystemP_SUCCESS;
    CANFD_Handle     canfdHandle;
    CANFD_TestParams      *testParams = (CANFD_TestParams *)args;
    App_CANFD_TxMsgParams *txMsgParams = testParams->txMsgParams;
    CANFD_MsgObjHandle     txMsgObjHandle = &(testParams->txMsgObject);
    CANFD_MsgObjHandle     rxMsgObjHandle = &(testParams->rxMsgObject);
    CANFD_OpenParams      *canfdOpenParams = &(testParams->openParams);
    CANFD_MessageObject   *txMsgObject = &(testParams->txMsgObject);
    CANFD_MessageObject   *rxMsgObject = &(testParams->rxMsgObject);

    /* Memset Buffers */
    memset(&txData[0U], 0, txMsgParams->dataLength * sizeof(txData[0U]));
    memset(&rxData[0U], 0, txMsgParams->dataLength * sizeof(rxData[0U]));

    CANFD_close(gCanfdHandle[testParams->canfdInstance]);

    for(int8_t i = 0; i < txMsgParams->dataLength; i++)
    {
        txData[i] = txMsgParams->data[i];
    }

    canfdHandle = CANFD_open(testParams->canfdInstance, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);

    /* Setup the transmit message object */
    txMsgObject->direction = CANFD_Direction_TX;
    txMsgObject->msgIdType = txMsgParams->msgIdType;
    txMsgObject->txMemType  = txMsgParams->txMemType;
    txMsgObject->dataLength = txMsgParams->dataLength;
    txMsgObject->args       = NULL;

    if(testParams->txMsgParams->msgIdType == CANFD_MCANXidType_11_BIT)
    {
        txMsgObject->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid1;
        txMsgObject->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid2;
        rxMsgObject->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid1;
        rxMsgObject->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid2;
    }
    else
    {
        txMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
        txMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
        rxMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
        rxMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
    }

    /* Setup the receive message object */
    rxMsgObject->direction = CANFD_Direction_RX;
    rxMsgObject->msgIdType = txMsgParams->msgIdType;;
    rxMsgObject->args       = (uint8_t*) rxData;
    rxMsgObject->rxMemType  = txMsgParams->rxMemType;
    rxMsgObject->dataLength = txMsgParams->dataLength;
    rxMsgObject->rxFifoNum  = testParams->rxFifoNum;

    status += CANFD_createMsgObject (canfdHandle, txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Tx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_createMsgObject (canfdHandle, rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Rx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    /* Send data over Tx message object */
    status += CANFD_write (txMsgObjHandle,
                        testParams->txMsgObject.startMsgId,
                        CANFD_MCANFrameType_FD,
                        0,
                        &txData[0]);

    status += CANFD_read(rxMsgObjHandle, MCAN_APP_TEST_MESSAGE_COUNT, &rxData[0]);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD read in interrupt mode failed\r\n");
        status = SystemP_FAILURE;
    }

    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD write in interrupt mode failed\r\n");
        status = SystemP_FAILURE;
    }

    /* Compare data */
    for(int32_t i = 0U; i < txMsgParams->dataLength; i++)
    {
        if(txData[i] != rxData[i])
        {
            status += SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    status += CANFD_deleteMsgObject(txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_deleteMsgObject(rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Rx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    CANFD_close(canfdHandle);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

static void test_canfd_loopback_dma(void *args)
{
    int32_t          status = SystemP_SUCCESS;
    CANFD_Handle     canfdHandle;
    CANFD_TestParams      *testParams = (CANFD_TestParams *)args;
    App_CANFD_TxMsgParams *txMsgParams = testParams->txMsgParams;
    CANFD_MsgObjHandle     txMsgObjHandle = &(testParams->txMsgObject);
    CANFD_MsgObjHandle     rxMsgObjHandle = &(testParams->rxMsgObject);
    CANFD_OpenParams      *canfdOpenParams = &(testParams->openParams);
    CANFD_MessageObject   *txMsgObject = &(testParams->txMsgObject);
    CANFD_MessageObject   *rxMsgObject = &(testParams->rxMsgObject);

    /* Memset Buffers */
    memset(&txData[0U], 0, txMsgParams->dataLength * sizeof(txData[0U]));
    memset(&rxData[0U], 0, txMsgParams->dataLength * sizeof(rxData[0U]));

    CANFD_close(gCanfdHandle[testParams->canfdInstance]);

    for(int8_t i = 0; i < txMsgParams->dataLength; i++)
    {
        txData[i] = txMsgParams->data[i];
    }

    /* Writeback buffer */
    CacheP_wb(&txData[0U], sizeof(txData), CacheP_TYPE_ALLD);
    CacheP_wb(&rxData[0U], sizeof(rxData), CacheP_TYPE_ALLD);

    canfdHandle = CANFD_open(testParams->canfdInstance, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);
    status += SemaphoreP_constructBinary(&gMcanTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status += SemaphoreP_constructBinary(&gMcanRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Setup the transmit message object */
    txMsgObject->direction = CANFD_Direction_TX;
    txMsgObject->msgIdType = txMsgParams->msgIdType;
    txMsgObject->txMemType  = txMsgParams->txMemType;
    txMsgObject->dataLength = txMsgParams->dataLength;
    txMsgObject->args       = NULL;

    if(testParams->txMsgParams->msgIdType == CANFD_MCANXidType_11_BIT)
    {
        txMsgObject->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid1;
        txMsgObject->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid2;
        rxMsgObject->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid1;
        rxMsgObject->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid2;
    }
    else
    {
        txMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
        txMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
        rxMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
        rxMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
    }

    /* Setup the receive message object */
    rxMsgObject->direction = CANFD_Direction_RX;
    rxMsgObject->msgIdType = txMsgParams->msgIdType;;
    rxMsgObject->args       = (uint8_t*) rxData;
    rxMsgObject->rxMemType  = txMsgParams->rxMemType;
    rxMsgObject->dataLength = txMsgParams->dataLength;
    rxMsgObject->rxFifoNum  = testParams->rxFifoNum;

    status += CANFD_createMsgObject (canfdHandle, txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Tx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_createMsgObject (canfdHandle, rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Rx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_read(rxMsgObjHandle, 1, &rxData[0]);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD read in DMA mode failed\r\n");
        status = SystemP_FAILURE;
    }

    /* Send data over Tx message object */
    status += CANFD_write (txMsgObjHandle,
                        testParams->txMsgObject.startMsgId,
                        CANFD_MCANFrameType_FD,
                        1,
                        &txData[0]);

    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD write in DMA mode failed\r\n");
        status = SystemP_FAILURE;
    }

    /* Wait for Tx completion */
    SemaphoreP_pend(&gMcanTxDoneSem, SystemP_WAIT_FOREVER);
    /* Wait for Rx completion */
    SemaphoreP_pend(&gMcanRxDoneSem, SystemP_WAIT_FOREVER);

    /* Invalidate cache */
    CacheP_inv(&rxData, sizeof(rxData), CacheP_TYPE_ALLD);

    /* Compare data */
    for(int32_t i = 0U; i < txMsgParams->dataLength; i++)
    {
        if(txData[i] != rxData[i])
        {
            status += SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    SemaphoreP_destruct(&gMcanTxDoneSem);
    SemaphoreP_destruct(&gMcanRxDoneSem);

    status += CANFD_deleteMsgObject(txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_deleteMsgObject(rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Rx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    CANFD_close(canfdHandle);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

static int32_t App_compareData(uint8_t *txData, uint8_t *rxData)
{
    int32_t status = SystemP_SUCCESS;

    /* Compare data */
    for(int32_t i = 0U; i < MCAN_APP_TEST_DATA_SIZE; i++)
    {
        if(txData[i] != rxData[i])
        {
            status = SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    return status;
}

/* This API provides a performance report for both Standard CAN 
 * and Extended CAN.
 */
static void test_canfd_loopback_perf(void *args)
{
    int32_t                status = SystemP_SUCCESS;
    CANFD_Handle           canfdHandle;
    CANFD_TestParams      *testParams = (CANFD_TestParams *)args;
    App_CANFD_TxMsgParams *txMsgParams = testParams->txMsgParams;
    CANFD_MsgObjHandle     txMsgObjHandle = &(testParams->txMsgObject);
    CANFD_MsgObjHandle     rxMsgObjHandle = &(testParams->rxMsgObject);
    CANFD_OpenParams      *canfdOpenParams = &(testParams->openParams);
    CANFD_MessageObject   *txMsgObject = &(testParams->txMsgObject);
    CANFD_MessageObject   *rxMsgObject = &(testParams->rxMsgObject);
    uint64_t               tsDiff, hwUtiln, tsFreq;
    uint64_t               numOfMsgPerSec;
    uint32_t               startTicks, stopTicks, maxMsgCnt, frameType;
    uint32_t               ticksDelay = CycleCounterP_getCount32();

    /* Memset Buffers */
    memset(&txData[0U], 0, txMsgParams->dataLength * sizeof(txData[0U]));
    memset(&rxData[0U], 0, txMsgParams->dataLength * sizeof(rxData[0U]));

    CANFD_close(gCanfdHandle[testParams->canfdInstance]);

    for(int8_t i = 0; i < txMsgParams->dataLength; i++)
    {
        txData[i] = txMsgParams->data[i];
    }

    canfdHandle = CANFD_open(testParams->canfdInstance, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);

    status += SemaphoreP_constructBinary(&gMcanTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status += SemaphoreP_constructBinary(&gMcanRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Setup the transmit message object */
    txMsgObject->direction  = CANFD_Direction_TX;
    txMsgObject->msgIdType  = txMsgParams->msgIdType;
    txMsgObject->txMemType  = txMsgParams->rxMemType;
    txMsgObject->args       = NULL;

    if(testParams->txMsgParams->msgIdType == CANFD_MCANXidType_11_BIT)
    {
        txMsgObject->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid1;
        txMsgObject->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid2;
        rxMsgObject->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid1;
        rxMsgObject->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid2;
        txMsgObject->dataLength = 8U;
        rxMsgObject->dataLength = 8U;
        frameType = CANFD_MCANFrameType_CLASSIC;
    }
    else
    {
        txMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
        txMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
        rxMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
        rxMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
        txMsgObject->dataLength = MCAN_APP_TEST_DATA_SIZE;
        rxMsgObject->dataLength = MCAN_APP_TEST_DATA_SIZE;
        frameType = CANFD_MCANFrameType_FD;
    }

    /* Setup the receive message object */
    rxMsgObject->direction  = CANFD_Direction_RX;
    rxMsgObject->msgIdType  = txMsgParams->msgIdType;
    rxMsgObject->args       = (uint8_t*) rxData;
    rxMsgObject->rxMemType  = txMsgParams->rxMemType;
    rxMsgObject->rxFifoNum  = testParams->rxFifoNum;

    status += CANFD_createMsgObject (canfdHandle, txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Tx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_createMsgObject (canfdHandle, rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Rx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    /* Check for STD vs EXT ID and FD vs Classic messages for CAN message */
    if (testParams->txMsgParams->msgIdType == CANFD_MCANXidType_29_BIT)
    {
        maxMsgCnt = MCAN_THEOROTICAL_MAX_EXT_1_5_MBPS;
    }
    else
    {
        maxMsgCnt = MCAN_THEOROTICAL_MAX_STD_1_5_MBPS;
    }

    CycleCounterP_reset();

    /* Get ticks delay */
    startTicks = CycleCounterP_getCount32();
    ticksDelay = CycleCounterP_getCount32();
    ticksDelay = CycleCounterP_getCount32() - ticksDelay;

    for(uint32_t iterationCnt = 0U; iterationCnt < MCAN_APP_TEST_PERFORMANCE_MESSAGE_COUNT; iterationCnt++)
    {
        status += CANFD_read(rxMsgObjHandle, 0U, &rxData[0]);
        if (status != SystemP_SUCCESS)
        {
            DebugP_log ("Error: CANFD read in interrupt mode failed\r\n");
            status = SystemP_FAILURE;
        }

        /* Send data over Tx message object */
        status += CANFD_write (txMsgObjHandle,
                            testParams->txMsgObject.startMsgId,
                            frameType,
                            0,
                            &txData[0]);

        if (status != SystemP_SUCCESS)
        {
            DebugP_log ("Error: CANFD write in interrupt mode failed\r\n");
            status = SystemP_FAILURE;
        }

        /* Wait for Tx completion */
        SemaphoreP_pend(&gMcanTxDoneSem, SystemP_WAIT_FOREVER);
        /* Wait for Rx completion */
        SemaphoreP_pend(&gMcanRxDoneSem, SystemP_WAIT_FOREVER);
        
        /* Compare data */
        for(int32_t i = 0U; i < txMsgObject->dataLength; i++)
        {
            if(txData[i] != rxData[i])
            {
                status += SystemP_FAILURE;   /* Data mismatch */
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
        }
    }
    
    /* capture time stamp After transfer completion */
    stopTicks = CycleCounterP_getCount32();

    if(stopTicks > startTicks)
    {
        tsDiff = (uint64_t)(stopTicks - startTicks - ticksDelay);
    }
    else
    {
        /* Counter overflow, assume only one overflow has happened */
        tsDiff = (uint64_t)((0xFFFFFFFFU - startTicks) + stopTicks - ticksDelay);
    }
    
    tsFreq = SOC_getSelfCpuClk();
    numOfMsgPerSec = (MCAN_APP_TEST_PERFORMANCE_MESSAGE_COUNT * tsFreq) / tsDiff;

    /* If internal loopback then no need of x2 as Tx and Rx nodes are same */
    if (!(attrs->CANFDMcanloopbackParams.mode == CANFD_MCANLoopBackMode_INTERNAL))
    {
        numOfMsgPerSec *= 2U;
    }
    hwUtiln = ((numOfMsgPerSec * 100) / (maxMsgCnt));

    DebugP_log("\n\n===========================================================\r");
    DebugP_log("TxRx:: Iteration Count:%d \r\n", MCAN_APP_TEST_PERFORMANCE_MESSAGE_COUNT);
    DebugP_log("TxRx:: ThroughPut: %lld Msg/sec\r\n", numOfMsgPerSec);
    DebugP_log("TxRx:: HW Utilization: %lld%%\r\n", hwUtiln);
    DebugP_log("===========================================================\r");
        
    if(hwUtiln < 85U)
    {
        testParams->testResult += SystemP_FAILURE;
        status += SystemP_FAILURE;
    }

    status += CANFD_deleteMsgObject(txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_deleteMsgObject(rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Rx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    SemaphoreP_destruct(&gMcanTxDoneSem);
    SemaphoreP_destruct(&gMcanRxDoneSem);
    
    CANFD_close(canfdHandle);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}


/*
 * This API is designed to cancel a running task. In this implementation, 
 * two tasks are created, where one task is responsible for canceling the other. 
 * This mechanism ensures controlled termination of the designated task.
 */
static void test_canfd_loopback_cancel(void *args)
{
    int32_t                status          = SystemP_SUCCESS;
    CANFD_TestParams      *testParams      = (CANFD_TestParams *)args;
    App_CANFD_TxMsgParams *txMsgParams     = testParams->txMsgParams;
    CANFD_OpenParams      *canfdOpenParams = &(testParams->openParams);

    TaskP_Params transferTaskParms, transferCancelTaskParms;

    /* Memset Buffers */
    memset(&txData[0U], 0, txMsgParams->dataLength * sizeof(txData[0U]));
    memset(&rxData[0U], 0, txMsgParams->dataLength * sizeof(rxData[0U]));

    CANFD_close(gCanfdHandle[testParams->canfdInstance]);

    for(int8_t i = 0; i < txMsgParams->dataLength; i++)
    {
        txData[i] = txMsgParams->data[i];
    }

    canfdHandle = CANFD_open(testParams->canfdInstance, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);

    status += SemaphoreP_constructCounting(&gCanfdWriteTaskDoneSemaphoreObj, 0, 2);
    DebugP_assert(SystemP_SUCCESS == status);
    status += SemaphoreP_constructCounting(&gCanfdWriteTaskCancelDoneSemaphoreObj, 0, 2);
    DebugP_assert(SystemP_SUCCESS == status);

    TaskP_Params_init(&transferTaskParms);
    transferTaskParms.name = "MCAN CANFD Write Task";
    transferTaskParms.stackSize = CANFD_TASK_STACK_SIZE;
    transferTaskParms.stack = gCanfdWriteTaskStack;
    transferTaskParms.priority = CANFD_TASK_PRIORITY;
    transferTaskParms.args = testParams;
    transferTaskParms.taskMain = test_canfd_loopback_cancel_transfer;
    status = TaskP_construct(&gCanfdWriteTaskObject, &transferTaskParms);
    DebugP_assert(status == SystemP_SUCCESS);

    ClockP_usleep(300);

    TaskP_Params_init(&transferCancelTaskParms);
    transferCancelTaskParms.name = "MCAN CANFD Write Cancel Task";
    transferCancelTaskParms.stackSize = CANFD_TASK_STACK_SIZE;
    transferCancelTaskParms.stack = gCanfdWriteCancelTaskStack;
    transferCancelTaskParms.priority = CANFD_TASK_PRIORITY;
    transferCancelTaskParms.args = &gTxMsgObjHandle;
    transferCancelTaskParms.taskMain = test_canfd_loopback_cancel_cancel;
    status = TaskP_construct(&gCanfdWriteCancelTaskObject, &transferCancelTaskParms);
    DebugP_assert(status == SystemP_SUCCESS);

    if(status != SystemP_SUCCESS)
    {
        gCancelTestStatus = SystemP_FAILURE;
    }

    SemaphoreP_pend(&gCanfdWriteTaskDoneSemaphoreObj, SystemP_WAIT_FOREVER);
    SemaphoreP_pend(&gCanfdWriteTaskCancelDoneSemaphoreObj, SystemP_WAIT_FOREVER);

    TaskP_destruct(&gCanfdWriteTaskObject);
    TaskP_destruct(&gCanfdWriteCancelTaskObject);
    ClockP_usleep(1000);

    CANFD_close(canfdHandle);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, gCancelTestStatus);

    return;
}

static void test_canfd_loopback_cancel_transfer(void *args)
{
    int32_t                status = SystemP_SUCCESS;
    CANFD_TestParams      *testParams  = (CANFD_TestParams *)args;
    App_CANFD_TxMsgParams *txMsgParams = testParams->txMsgParams;
    CANFD_MessageObject   *txMsgObject = &(testParams->txMsgObject);
    CANFD_MessageObject   *rxMsgObject = &(testParams->rxMsgObject);

    gTxMsgObjHandle  = &(testParams->txMsgObject);
    gRxMsgObjHandle  = &(testParams->rxMsgObject);

    /* Memset Buffers */
    memset(&txData[0U], 0, txMsgParams->dataLength * sizeof(txData[0U]));
    memset(&rxData[0U], 0, txMsgParams->dataLength * sizeof(rxData[0U]));

    for(int8_t i = 0; i < txMsgParams->dataLength; i++)
    {
        txData[i] = txMsgParams->data[i];
    }
    
    status += SemaphoreP_constructBinary(&gMcanTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status += SemaphoreP_constructBinary(&gMcanRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Setup the transmit message object */
    txMsgObject->direction = CANFD_Direction_TX;
    txMsgObject->msgIdType = txMsgParams->msgIdType;
    txMsgObject->txMemType  = txMsgParams->txMemType;
    txMsgObject->dataLength = txMsgParams->dataLength;
    txMsgObject->args       = NULL;

    if(testParams->txMsgParams->msgIdType == CANFD_MCANXidType_11_BIT)
    {
        txMsgObject->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid1;
        txMsgObject->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid2;
        rxMsgObject->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid1;
        rxMsgObject->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs->filterConfig)->sfid2;
    }
    else
    {
        txMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
        txMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
        rxMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
        rxMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
    }

    /* Setup the receive message object */
    rxMsgObject->direction = CANFD_Direction_RX;
    rxMsgObject->msgIdType = txMsgParams->msgIdType;;
    rxMsgObject->args       = (uint8_t*) rxData;
    rxMsgObject->rxMemType  = txMsgParams->rxMemType;
    rxMsgObject->dataLength = txMsgParams->dataLength;
    rxMsgObject->rxFifoNum  = testParams->rxFifoNum;

    status += CANFD_createMsgObject (canfdHandle, gTxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Tx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_createMsgObject (canfdHandle, gRxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Rx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_read(gRxMsgObjHandle, MCAN_APP_TEST_MESSAGE_COUNT, &rxData[0]);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD read in interrupt mode failed\r\n");
        status = SystemP_FAILURE;
    }

    /* Send data over Tx message object */
    status += CANFD_write (gTxMsgObjHandle,
                          testParams->txMsgObject.startMsgId,
                          CANFD_MCANFrameType_FD,
                          0,
                          &txData[0]);

    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD write in interrupt mode failed\r\n");
        status = SystemP_FAILURE;
    }

    /* Wait for Tx completion */
    SemaphoreP_pend(&gMcanTxDoneSem, SystemP_WAIT_FOREVER);
    /* Wait for Rx completion */
    SemaphoreP_pend(&gMcanRxDoneSem, SystemP_WAIT_FOREVER);

    /* Compare data */
    for(int32_t i = 0U; i < txMsgParams->dataLength; i++)
    {
        if(txData[i] != rxData[i])
        {
            gCancelTestStatus = SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    ClockP_usleep(100);

    status += CANFD_deleteMsgObject(gTxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_deleteMsgObject(gRxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Rx message object failed\r\n");
        status = SystemP_FAILURE;
    }

    if(status != SystemP_SUCCESS)
    {
        gCancelTestStatus = SystemP_FAILURE;
    }

    SemaphoreP_post(&gCanfdWriteTaskDoneSemaphoreObj);
    while(1)
    {
        /* Yield to the main task which deletes this task. */
        TaskP_yield();
    };
}

static void test_canfd_loopback_cancel_cancel(void *args)
{
    int32_t transferOK;
    CANFD_MsgObjHandle* temp =args;
    transferOK = CANFD_writeCancel(*temp);
    DebugP_assert(transferOK == SystemP_SUCCESS);

    SemaphoreP_post(&gCanfdWriteTaskCancelDoneSemaphoreObj);
    vTaskSuspend(NULL);
    while(1)
    {
        /* Yield to the main task which deletes this task. */
        TaskP_yield();
    };
}

/*
 * This API is responsible for configuring and validating multiple instances 
 * running simultaneously. It ensures proper setup and checks the functionality 
 * of each instance to maintain smooth operation and avoid conflicts.
 */
static void test_canfd_loopback_multi_instances(void *args)
{
    int32_t                status = SystemP_SUCCESS;
    int32_t                testStatus = SystemP_SUCCESS;
    CANFD_Config          *config0 = &gCanfdConfig[CONFIG_MCAN0];
    CANFD_Attrs           *attrs0  = (CANFD_Attrs *)config0->attrs;
    CANFD_TestParams      *testParams0 = (CANFD_TestParams *)args;
    App_CANFD_TxMsgParams *txMsgParams0 = testParams0->txMsgParams;
    CANFD_MsgObjHandle     txMsgObjHandle0 = &(testParams0->txMsgObject);
    CANFD_MsgObjHandle     rxMsgObjHandle0 = &(testParams0->rxMsgObject);
    CANFD_OpenParams      *canfdOpenParams0 = &(testParams0->openParams);
    CANFD_MessageObject   *txMsgObject0 = &(testParams0->txMsgObject);
    CANFD_MessageObject   *rxMsgObject0 = &(testParams0->rxMsgObject);

    CANFD_Config          *config1 = &gCanfdConfig[CONFIG_MCAN1];
    CANFD_Attrs           *attrs1  = (CANFD_Attrs *)config1->attrs;
    CANFD_TestParams       testParams1;
    CANFD_TestParams      *testParams1ptr = &testParams1;

    /* Create a copy of args to that other instance can use it */
    memcpy(&testParams1, (CANFD_TestParams *)args, sizeof(CANFD_TestParams));

    App_CANFD_TxMsgParams *txMsgParams1 = testParams1ptr->txMsgParams;
    CANFD_MsgObjHandle     txMsgObjHandle1 = &(testParams1ptr->txMsgObject);
    CANFD_MsgObjHandle     rxMsgObjHandle1 = &(testParams1ptr->rxMsgObject);
    CANFD_OpenParams      *canfdOpenParams1 = &(testParams1ptr->openParams);
    CANFD_MessageObject   *txMsgObject1 = &(testParams1ptr->txMsgObject);
    CANFD_MessageObject   *rxMsgObject1 = &(testParams1ptr->rxMsgObject);

    /* Memset Buffers */
    memset(&txData[0U],  0, txMsgParams0->dataLength * sizeof(txData[0U]));
    memset(&rxData[0U],  0, txMsgParams0->dataLength * sizeof(rxData[0U]));
    /* Memset Buffers */
    memset(&txData1[0U],  0, txMsgParams0->dataLength * sizeof(txData1[0U]));
    memset(&rxData1[0U],  0, txMsgParams0->dataLength * sizeof(rxData1[0U]));

    CANFD_close(gCanfdHandle[CONFIG_MCAN0]);
    CANFD_close(gCanfdHandle[CONFIG_MCAN1]);

    for(int8_t i = 0; i < txMsgParams0->dataLength; i++)
    {
        txData[i]  = txMsgParams0->data[i];
        txData1[i]  = txMsgParams1->data[i] + 1U;
    }
    attrs1->baseAddr = CONFIG_MCAN1_BASE_ADDR;
    attrs1->operMode = CANFD_OPER_MODE_INTERRUPT;
#if defined (SOC_AM273X) 
    attrs1->intrNum0 = CSL_MSS_INTR_MSS_MCANB_INT0;
    attrs1->intrNum1 = CSL_MSS_INTR_MSS_MCANB_INT1;
#elif defined (SOC_AM64X) || defined (SOC_AM243X)
    attrs1->intrNum0 = CSLR_GICSS0_SPI_MCAN1_MCANSS_MCAN_LVL_INT_0;
    attrs1->intrNum1 = CSLR_GICSS0_SPI_MCAN1_MCANSS_MCAN_LVL_INT_1;
#else
    attrs1->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN1_MCAN_LVL_INT_0;
    attrs1->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN1_MCAN_LVL_INT_1;
#endif
    canfdHandle = CANFD_open(CONFIG_MCAN0, canfdOpenParams0);
    TEST_ASSERT_NOT_NULL(canfdHandle);
    canfdHandle1 = CANFD_open(CONFIG_MCAN1, canfdOpenParams1);
    TEST_ASSERT_NOT_NULL(canfdHandle1);

    status += SemaphoreP_constructBinary(&gMcanTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status += SemaphoreP_constructBinary(&gMcanRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status += SemaphoreP_constructBinary(&gMcanTxDoneSem1, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status += SemaphoreP_constructBinary(&gMcanRxDoneSem1, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Setup the transmit message object */
    txMsgObject0->direction  = CANFD_Direction_TX;
    txMsgObject0->msgIdType  = txMsgParams0->msgIdType;
    txMsgObject0->txMemType  = txMsgParams0->txMemType;
    txMsgObject0->dataLength = txMsgParams0->dataLength;
    txMsgObject0->args       = NULL;

    if(testParams0->txMsgParams->msgIdType == CANFD_MCANXidType_11_BIT)
    {
        txMsgObject0->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs0->filterConfig)->sfid1;
        txMsgObject0->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs0->filterConfig)->sfid2;
        rxMsgObject0->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs0->filterConfig)->sfid1;
        rxMsgObject0->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs0->filterConfig)->sfid2;
    }
    else
    {
        txMsgObject0->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs0->filterConfig)->efid1;
        txMsgObject0->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs0->filterConfig)->efid2;
        rxMsgObject0->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs0->filterConfig)->efid1;
        rxMsgObject0->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs0->filterConfig)->efid2;
    }

    /* Setup the receive message object */
    rxMsgObject0->direction = CANFD_Direction_RX;
    rxMsgObject0->msgIdType = txMsgParams0->msgIdType;
    rxMsgObject0->args       = (uint8_t*) rxData;
    rxMsgObject0->rxMemType  = txMsgParams0->rxMemType;
    rxMsgObject0->dataLength = txMsgParams0->dataLength;

    /* Setup the transmit message object */
    txMsgObject1->direction  = CANFD_Direction_TX;
    txMsgObject1->msgIdType  = txMsgParams1->msgIdType;
    txMsgObject1->txMemType  = txMsgParams1->txMemType;
    txMsgObject1->dataLength = txMsgParams1->dataLength;
    txMsgObject1->args       = NULL;

    if(testParams1ptr->txMsgParams->msgIdType == CANFD_MCANXidType_11_BIT)
    {
        txMsgObject1->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs1->filterConfig)->sfid1;
        txMsgObject1->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs1->filterConfig)->sfid2;
        rxMsgObject1->startMsgId = ((MCAN_StdMsgIDFilterElement*)attrs1->filterConfig)->sfid1;
        rxMsgObject1->endMsgId   = ((MCAN_StdMsgIDFilterElement*)attrs1->filterConfig)->sfid2;
    }
    else
    {
        txMsgObject1->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs1->filterConfig)->efid1;
        txMsgObject1->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs1->filterConfig)->efid2;
        rxMsgObject1->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs1->filterConfig)->efid1;
        rxMsgObject1->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs1->filterConfig)->efid2;
    }

    /* Setup the receive message object */
    rxMsgObject1->direction  = CANFD_Direction_RX;
    rxMsgObject1->msgIdType  = txMsgParams1->msgIdType;
    rxMsgObject1->args       = (uint8_t*) rxData1;
    rxMsgObject1->rxMemType  = txMsgParams1->rxMemType;
    rxMsgObject1->dataLength = txMsgParams1->dataLength;
    rxMsgObject1->rxFifoNum  = testParams1.rxFifoNum;

    status += CANFD_createMsgObject (canfdHandle, txMsgObjHandle0);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Tx message object failed for instance 0.\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_createMsgObject (canfdHandle, rxMsgObjHandle0);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Rx message object failed for instance 0.\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_createMsgObject (canfdHandle1, txMsgObjHandle1);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Tx message object failed for instance 1.\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_createMsgObject (canfdHandle1, rxMsgObjHandle1);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Rx message object failed for instance 1.\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_read(rxMsgObjHandle0, MCAN_APP_TEST_MESSAGE_COUNT, &rxData[0]);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD read in interrupt mode failed for instance 0.\r\n");
        status = SystemP_FAILURE;
    }

    /* Send data over Tx message object */
    status += CANFD_write (txMsgObjHandle0,
                          testParams0->txMsgObject.startMsgId,
                          CANFD_MCANFrameType_FD,
                          0,
                          &txData[0]);

    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD write in interrupt mode failed for instance 0.\r\n");
        status = SystemP_FAILURE;
    }

    /* Wait for Tx completion */
    SemaphoreP_pend(&gMcanTxDoneSem, SystemP_WAIT_FOREVER);
    /* Wait for Rx completion */
    SemaphoreP_pend(&gMcanRxDoneSem, SystemP_WAIT_FOREVER);

    /* Set the flag to indicate other instance is being used. */
    gFlagInstance1 = 1U;

    status += CANFD_read(rxMsgObjHandle1, MCAN_APP_TEST_MESSAGE_COUNT, &rxData1[0]);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD read in interrupt mode failed for instance 1.\r\n");
        status = SystemP_FAILURE;
    }

    /* Send data over Tx message object */
    status += CANFD_write (txMsgObjHandle1,
                          testParams1ptr->txMsgObject.startMsgId,
                          CANFD_MCANFrameType_FD,
                          MCAN_APP_TEST_MESSAGE_COUNT,
                          &txData1[0]);

    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD write in interrupt mode failed for instance 1.\r\n");
        status = SystemP_FAILURE;
    }

    /* Wait for Tx completion */
    SemaphoreP_pend(&gMcanTxDoneSem1, SystemP_WAIT_FOREVER);
    /* Wait for Rx completion */
    SemaphoreP_pend(&gMcanRxDoneSem1, SystemP_WAIT_FOREVER);
    
    /* Compare data */
    for(int32_t i = 0U; i < txMsgParams0->dataLength; i++)
    {
        if(txData[i] != rxData[i])
        {
            status += SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d for instance 0.\r\n", i);
            break;
        }
    }

    /* Compare data */
    for(int32_t i = 0U; i < txMsgParams1->dataLength; i++)
    {
        if(txData1[i] != rxData1[i])
        {
            status += SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d for instance 1.\r\n", i);
            break;
        }
    }

    SemaphoreP_destruct(&gMcanTxDoneSem);
    SemaphoreP_destruct(&gMcanRxDoneSem);
    SemaphoreP_destruct(&gMcanTxDoneSem1);
    SemaphoreP_destruct(&gMcanRxDoneSem1);

    status += CANFD_deleteMsgObject(txMsgObjHandle0);
    status += CANFD_deleteMsgObject(txMsgObjHandle1);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed.\r\n");
        status = SystemP_FAILURE;
    }

    status += CANFD_deleteMsgObject(rxMsgObjHandle0);
    status += CANFD_deleteMsgObject(rxMsgObjHandle1);
    if (status != SystemP_SUCCESS)
    {
        status = SystemP_FAILURE;
        DebugP_log ("Error: CANFD delete Rx message object failed\r\n");
    }

    /* Reset the flag. */
    gFlagInstance1 = 0U;

    /* Memset Buffers */
    memset(&txData[0U],  0, txMsgParams0->dataLength * sizeof(txData[0U]));
    memset(&rxData[0U],  0, txMsgParams0->dataLength * sizeof(rxData[0U]));
    /* Memset Buffers */
    memset(&txData1[0U],  0, txMsgParams0->dataLength * sizeof(txData1[0U]));
    memset(&rxData1[0U],  0, txMsgParams0->dataLength * sizeof(rxData1[0U]));

    CANFD_close(canfdHandle);
    CANFD_close(canfdHandle1);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    return;
}

static void App_CANFDPrintRevID(MCAN_RevisionId *revId)
{
    if(revId != NULL)
    {
        DebugP_log("\n\n----------------------------------\n\r");
        DebugP_log("|  Minor Revision = %d \t\t |\n\r", revId->minor);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  Major Revision = %d \t\t |\n\r", revId->major);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  RTL Revision = %d \t\t |\n\r",revId->rtlRev);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  Custom = %d\t\t\t |\n\r", revId->custom);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  PID register scheme = %d \t |\n\r", revId->scheme);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  RTL Revision = %d \t         |\n\r",revId->rtlRev);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  Module ID = %d \t         |\n\r", revId->modId);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  Business Unit: = %d \t         |\n\r", revId->bu);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  Time Stamp Day = %d \t         |\n\r",revId->day);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  Time Stamp Months = %d \t |\n\r",revId->mon);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  Time Stamp Years = %d \t |\n\r",revId->year);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  Substep of Core release = %d \t |\n\r",revId->subStep);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  Step of Core release = %d \t |\n\r",revId->step);
        DebugP_log("----------------------------------\n\r");
        DebugP_log("|  Core Release = %d \t\t |\n\r",revId->rel);
        DebugP_log("----------------------------------\n\n\r");
    }
}

/*
 * This API retrieves and prints the revision details along with the basic 
 * CAN controller information.
 */
static void test_canfd_get_revId(void *args)
{
    int32_t           testStatus = SystemP_SUCCESS;
    CANFD_Handle      canfdHandle = NULL;
    CANFD_Config     *config = NULL;
    CANFD_Object     *ptrCanFdObj = NULL;
    CANFD_TestParams *testParams = (CANFD_TestParams *)args;
    CANFD_OpenParams *canfdOpenParams = &(testParams->openParams);
    MCAN_RevisionId  *revId;

    CANFD_close(gCanfdHandle[testParams->canfdInstance]);

    canfdHandle = CANFD_open(testParams->canfdInstance, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);
    config      = (CANFD_Config*) canfdHandle;
    ptrCanFdObj = (CANFD_Object*) config->object;
    DebugP_assert(NULL_PTR != ptrCanFdObj);
    revId = &ptrCanFdObj->revId;

    CANFD_getRevisionId(canfdHandle);
    /* Get MCANSS Revision ID */
    App_CANFDPrintRevID(revId);

    CANFD_close(canfdHandle);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    return;
}

/*
 * This API retrieves the current state of the TX and RX pins of the CAN controller. 
 * It provides a detailed status report, enabling monitoring and diagnostics 
 * of communication functionality.
 */
static void test_canfd_TxRx_pin_state(void *args)
{
    int32_t           testStatus = SystemP_SUCCESS;
    uint32_t          pinState = 0U;
    CANFD_Handle      canfdHandle = NULL;
    CANFD_Config     *config = NULL;
    CANFD_Object     *ptrCanFdObj = NULL;
    CANFD_TestParams *testParams = (CANFD_TestParams *)args;
    CANFD_OpenParams *canfdOpenParams = &(testParams->openParams);

    CANFD_close(gCanfdHandle[testParams->canfdInstance]);

    canfdHandle = CANFD_open(testParams->canfdInstance, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);
    config      = (CANFD_Config*) canfdHandle;
    ptrCanFdObj = (CANFD_Object*) config->object;
    DebugP_assert(NULL_PTR != ptrCanFdObj);

    /* Monitors the actual value of the MCAN RX pin
       Rx pin may be Dominant/Recessive. Not correct to return error */
    pinState = CANFD_getRxPinState(canfdHandle);
    if (pinState == 0U)
    {
        DebugP_log( "Rx Pin Mode: Dominant\r\n");
    }
    else
    {
        DebugP_log( "Rx Pin Mode: Recessive\r\n");
    }

    DebugP_log( "Setting Tx PAD into Dominant mode.\r\n");
    CANFD_setTxPinState(canfdHandle, 0x2);
    pinState = CANFD_getTxPinState(canfdHandle);
    if(pinState == 0x2U)
    {
        DebugP_log( "Tx Pin Mode: Dominant\r\n");
    }
    else
    {
        testStatus += SystemP_FAILURE;
        DebugP_log( "Tx Pin Mode: Recessive\r\n");
    }

    DebugP_log( "Setting Tx PAD into Recessive mode.\r\n");
    CANFD_setTxPinState(canfdHandle, 0x3U);
    pinState = CANFD_getTxPinState(canfdHandle);
    if (pinState == 0x3U)
    {
        DebugP_log( "Tx Pin Mode: Recessive\r\n\n");
    }
    else
    {
        testStatus += SystemP_FAILURE;
        DebugP_log( "Tx Pin Mode: Dominant\r\n\n");
    }

    DebugP_log( "Setting Tx PAD into Dominant mode.\r\n");
    CANFD_setTxPinState(canfdHandle, 0x0);
    pinState = CANFD_getTxPinState(canfdHandle);
    if(pinState == 0x0U)
    {
        DebugP_log( "Resetting Tx Pin \r\n");
    }
    else
    {
        testStatus += SystemP_FAILURE;
        DebugP_log( "Tx Pin Set Fail \r\n");
    }

    CANFD_close(canfdHandle);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    return;
}

/* This API return endianness value of MCAN module. */
static void test_canfd_endianness(void *args)
{
    uint32_t           endianessVal = 0U;
    int32_t           testStatus = SystemP_SUCCESS;
    CANFD_Handle      canfdHandle = NULL;
    CANFD_Config     *config = NULL;
    CANFD_Object     *ptrCanFdObj = NULL;
    CANFD_TestParams *testParams = (CANFD_TestParams *)args;
    CANFD_OpenParams *canfdOpenParams = &(testParams->openParams);

    CANFD_close(gCanfdHandle[testParams->canfdInstance]);

    canfdHandle = CANFD_open(testParams->canfdInstance, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);
    config      = (CANFD_Config*) canfdHandle;
    ptrCanFdObj = (CANFD_Object*) config->object;
    DebugP_assert(NULL_PTR != ptrCanFdObj);

    endianessVal = CANFD_getEndianVal(canfdHandle);
    DebugP_log("Endianess Value: %x\n\r", endianessVal);
    if(endianessVal == (uint32_t)0x87654321)
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }

    CANFD_close(canfdHandle);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    return;
}

/* This API get the Bit-rate of MCAN module. */
static void test_canfd_getBitRate(void *args)
{
    int32_t          testStatus = SystemP_SUCCESS;
    CANFD_Handle     canfdHandle;
    CANFD_TestParams      *testParams = (CANFD_TestParams *)args;
    CANFD_OpenParams      *canfdOpenParams = &(testParams->openParams);
    uint32_t canfdNomBitrate, canfdDataBitrate;
    CANFD_MCANBitTimingParams  *bitTimingParams;

    CANFD_close(gCanfdHandle[testParams->canfdInstance]);

    canfdHandle = CANFD_open(testParams->canfdInstance, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);

    config      = (CANFD_Config*) canfdHandle;
    bitTimingParams = &config->attrs->CANFDMcanBitTimingParams;

    CANFD_getBitTime(canfdHandle);
    /**
     * bit rate(bits per second) = (CAN clock in Hz) / BRP / (1 + TSEG1 + TSEG2)
     * CAN clock is functional clock of CAN module (80MHz by default in TDA4)
     * BRP : Bit rate pre-scalar value
     * TSEG1: TSEG2: Time segments used to define sampling point for the bit.
     * TSEG1: Time before the sampling point = Prop_Seg + Phase_Seg1
     * TSEG2: Time after the sampling point = Phase_Seg2
     */

    canfdNomBitrate = ((APP_MCAN_FUNCTIONAL_CLK / bitTimingParams->nomBrp) / 
                   (1 + (bitTimingParams->nomPropSeg + bitTimingParams->nomPseg1) + bitTimingParams->nomPseg2));

    canfdDataBitrate = ((APP_MCAN_FUNCTIONAL_CLK / bitTimingParams->dataBrp) / 
                   (1 + (bitTimingParams->dataPropSeg + bitTimingParams->dataPseg1) + bitTimingParams->dataPseg2));
    /* Norminal data bitrate is 1000 and 5000 kbps respectively  */
    if((canfdNomBitrate == 1000U) && (canfdDataBitrate == 5000))
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }

    CANFD_close(canfdHandle);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    return;
}

/*  This API add clock stop request for MCAN module to put it
 *  in power down mode. 
 */
static void test_canfd_Clk_Stop_Req_Test(void *args)
{
    int32_t          testStatus = SystemP_SUCCESS;
    CANFD_Handle     canfdHandle;
    CANFD_TestParams *testParams = (CANFD_TestParams *)args;
    CANFD_OpenParams *canfdOpenParams = &(testParams->openParams);

    CANFD_close(gCanfdHandle[testParams->canfdInstance]);

    canfdHandle = CANFD_open(testParams->canfdInstance, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);

    config      = (CANFD_Config*) canfdHandle;
    DebugP_log("Asserting Clock Stop Request..\r\n");
    CANFD_addClockStopRequest(canfdHandle, TRUE);
    while (CANFD_clockStopReq(canfdHandle) != 1U)
    {
    }
    DebugP_log("Clock Stop Request ACKed!!\r\n");
    
    testStatus = CANFD_getClkStopAck(canfdHandle);
    if(testStatus == 1U)
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }

    CANFD_addClockStopRequest(canfdHandle, FALSE);
    CANFD_close(canfdHandle);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    return;
}

/* This API will reset timestamp counter value. */
static void test_canfd_TS_Reset(void *args)
{
    int32_t          testStatus = SystemP_SUCCESS;
    CANFD_Handle     canfdHandle;
    uint32_t oldTimeStamp, timeStamp;
    CANFD_TestParams *testParams = (CANFD_TestParams *)args;
    CANFD_OpenParams *canfdOpenParams = &(testParams->openParams);

    CANFD_close(gCanfdHandle[testParams->canfdInstance]);

    canfdHandle = CANFD_open(testParams->canfdInstance, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);

    config = (CANFD_Config*) canfdHandle;
    timeStamp = CANFD_getTSCounterVal(canfdHandle);
    DebugP_log("TS at (t) ms: %d\r\n", timeStamp);
    /* Delay added for getting new timeStamp- so that there will difference between two timeStamps */
    App_delayFunc(10);
    oldTimeStamp = timeStamp;
    timeStamp = CANFD_getTSCounterVal(canfdHandle);
    DebugP_log("TS at (t + 10) ms: %d\r\n", timeStamp);

    /* reset TS counter */
    CANFD_resetTSCounter(canfdHandle);
    oldTimeStamp = timeStamp;
    timeStamp = CANFD_getTSCounterVal(canfdHandle);
    /* wrap around condition is not taken care */
    if (timeStamp < oldTimeStamp)
    {
        DebugP_log("TS Counter Reset done!!\r\n");
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    return;
}

void App_delayFunc(uint32_t timeout)
{
}

static void test_canfd_set_params(CANFD_TestParams *testParams, uint32_t tcId)
{
    config = &gCanfdConfig[CONFIG_MCAN0];
    attrs  = (CANFD_Attrs *)config->attrs;
    CANFD_OpenParams *openParams = &(testParams->openParams);
    CANFD_MCANBitTimingParams *bitTimingParams = &(attrs->CANFDMcanBitTimingParams);
    MCAN_StdMsgIDFilterElement  stdMsgIdFilterEle;

    /* Default Attribute Parameters */
    attrs->baseAddr                       = CONFIG_MCAN0_BASE_ADDR,
#if defined (SOC_AM273X)
    attrs->intrNum0 = CSL_MSS_INTR_MSS_MCANA_INT0;
    attrs->intrNum1 = CSL_MSS_INTR_MSS_MCANA_INT1;
#elif defined (SOC_AM64X) || defined (SOC_AM243X)
    attrs->intrNum0 = CSLR_GICSS0_SPI_MCAN0_MCANSS_MCAN_LVL_INT_0;
    attrs->intrNum1 = CSLR_GICSS0_SPI_MCAN0_MCANSS_MCAN_LVL_INT_1;
#else
    attrs->intrNum0                       = CSLR_R5FSS0_CORE0_INTR_MCAN0_MCAN_LVL_INT_0,
    attrs->intrNum1                       = CSLR_R5FSS0_CORE0_INTR_MCAN0_MCAN_LVL_INT_1,
#endif  
    attrs->intrPriority                   = 4U,
    attrs->operMode                       = CANFD_OPER_MODE_INTERRUPT,
    attrs->OptionTLVtype                  = CANFD_Option_MCAN_LOOPBACK,
    attrs->CANFDMcanloopbackParams.enable = true,
    attrs->CANFDMcanloopbackParams.mode   = CANFD_MCANLoopBackMode_INTERNAL,
    /* 
     *  Provide actual Bit-timing parameters as these values will be directly written
     *  to MCANn_CFG_NBTP and MCANn_CFG_DBTP registers. These values can't be zero.
     */
    /* Default Bit-Rate 1000kbps and 5000 kbps*/
    bitTimingParams->nomBrp          = 2U,
    bitTimingParams->nomPropSeg      = 11U,
    bitTimingParams->nomPseg1        = 22U,
    bitTimingParams->nomPseg2        = 6U,
    bitTimingParams->nomSjw          = 1U,
    bitTimingParams->dataBrp         = 1U,
    bitTimingParams->dataPropSeg     = 2U,
    bitTimingParams->dataPseg1       = 4U,
    bitTimingParams->dataPseg2       = 1U,
    bitTimingParams->dataSjw         = 1U,

    /* Default Open Parameters */
    openParams->transferMode              = CANFD_TRANSFER_MODE_CALLBACK,
    openParams->transferCallbackFxn       = App_CANFD_TransferCallback,
    openParams->errorCallbackFxn          = App_CANFD_ErrorCallback,
    openParams->fdMode                    = true,
    openParams->brsEnable                 = true,
    openParams->txpEnable                 = false,
    openParams->efbi                      = false,
    openParams->pxhddisable               = false,
    openParams->darEnable                 = false,
    openParams->wkupReqEnable             = false,
    openParams->autoWkupEnable            = false,
    openParams->emulationEnable           = false,
    openParams->emulationFAck             = false,
    openParams->clkStopFAck               = false,
    openParams->wdcPreload                = MCAN_RWD_WDC_MAX,
    openParams->tdcEnable                 = false,
    /* Transmitter Delay Compensation parameters. */
    openParams->tdcConfig.tdcf            = 0x0AU,
    openParams->tdcConfig.tdco            = 0x06U,
    /* Initialize MCAN Config parameters.  */
    openParams->monEnable                 = false,
    openParams->asmEnable                 = false,
    openParams->tsPrescalar               = 0xFU,
    openParams->tsSelect                  = 0U,
    openParams->timeoutSelect             = MCAN_TIMEOUT_SELECT_CONT,
    openParams->timeoutPreload            = 0xFFFFU,
    openParams->timeoutCntEnable          = false,
    /* Global Filter Configuration parameters. */
    openParams->filterConfig.rrfe         = false,
    openParams->filterConfig.rrfs         = false,
    openParams->filterConfig.anfe         = false,
    openParams->filterConfig.anfs         = false,
    /* Message RAM Configuration parameters. */
    openParams->msgRAMConfig.lss          = APP_MCAN_STD_ID_FILTER_NUM,
    openParams->msgRAMConfig.lse          = APP_MCAN_EXT_ID_FILTER_NUM,
    openParams->msgRAMConfig.txBufNum     = APP_MCAN_TX_BUFF_SIZE,
    openParams->msgRAMConfig.txFIFOSize              = 0U,
    openParams->msgRAMConfig.txBufMode               = 0U, /* Tx FIFO/QUEUE operation */
    openParams->msgRAMConfig.txEventFIFOSize         = APP_MCAN_TX_BUFF_SIZE,
    openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
    openParams->msgRAMConfig.rxFIFO0size             = APP_MCAN_FIFO_0_NUM,
    openParams->msgRAMConfig.rxFIFO0waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
    openParams->msgRAMConfig.rxFIFO0OpMode           = 0U,
    openParams->msgRAMConfig.rxFIFO1size             = APP_MCAN_FIFO_1_NUM,
    openParams->msgRAMConfig.rxFIFO1waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
    openParams->msgRAMConfig.rxFIFO1OpMode           = 0U,
    /* ECC Configuration parameters. */
    openParams->eccConfig.enable           = true,
    openParams->eccConfig.enableChk        = true,
    openParams->eccConfig.enableRdModWr    = true,
    openParams->errInterruptEnable         = true,
    openParams->dataInterruptEnable        = true,

    testParams->testCaseId   = tcId;
    testParams->canfdInstance = CONFIG_MCAN0;
    testParams->rxFifoNum   = canTxMsg[0U].rxFifoNum;

    if(openParams->fdMode == true)
    {
        attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*)(&canExtIdFilter[0U]);
    }
    else
    {
        attrs->filterConfig = (MCAN_StdMsgIDFilterElement*)(&stdMsgIdFilterEle);
        stdMsgIdFilterEle.sfid1 = 0x29E;
        stdMsgIdFilterEle.sfec  = 0x7U;
        stdMsgIdFilterEle.sfid2 = 0x29E;
        stdMsgIdFilterEle.sft   = 0x0U;
    }

    switch (tcId)
    {
        case 13920:
            /*
            * MCAN CANFD Loopback Bitrate Test configured for 1Mbps/5Mbps.
            * MCAN: CAN FD Mode with Bit Rate Switching enabled and bitrates set to 1Mbps and 5Mbps.
            * In this test, transmitted messages are stored in the buffer, while received messages 
            * are stored in the FIFO.
            */

            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum                = 0U,
            openParams->msgRAMConfig.txFIFOSize              = 16U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10U,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
            /* mcan module configuration parameters */
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = &canExtIdFilter[1U];
            canExtIdFilter[1U].efec = canTxMsg[2U].filterElement;
            canExtIdFilter[1U].eft  = canTxMsg[2U].rxfilterType;
            testParams->rxFifoNum   = canTxMsg[2U].rxFifoNum;
            break;
        case 13921:
            /* Internal Loopback, High Priority And Bitrate 1Mbps/2.5Mbps test.
             * MCAN: CAN FD Mode with Bit Rate Switching ON And Bitrate 1MBps/5MBps
             * Transmitted and received msg will be stored in buffer and FIFO respectively.
             */
#if defined (SOC_AM261X)
            /* AM261x has only two MCAN instances */
            testParams->canfdInstance = CONFIG_MCAN0;
            config = &gCanfdConfig[CONFIG_MCAN0];
            attrs  = (CANFD_Attrs *)config->attrs;
            attrs->baseAddr = CONFIG_MCAN0_BASE_ADDR,
            attrs->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN0_MCAN_LVL_INT_0,
            attrs->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN0_MCAN_LVL_INT_1,
#elif defined (SOC_AM273X)
            testParams->canfdInstance = CONFIG_MCAN1;
            config = &gCanfdConfig[CONFIG_MCAN1];
            attrs  = (CANFD_Attrs *)config->attrs;
            attrs->operMode = CANFD_OPER_MODE_INTERRUPT;
            attrs->baseAddr = CONFIG_MCAN1_BASE_ADDR,
            attrs->intrNum0 = CSL_MSS_INTR_MSS_MCANA_INT0,
            attrs->intrNum1 = CSL_MSS_INTR_MSS_MCANA_INT1,
#elif defined (SOC_AM64X) || defined (SOC_AM243X)
            testParams->canfdInstance = CONFIG_MCAN1;
            config = &gCanfdConfig[CONFIG_MCAN1];
            attrs  = (CANFD_Attrs *)config->attrs;
            attrs->operMode = CANFD_OPER_MODE_INTERRUPT;
            attrs->baseAddr = CONFIG_MCAN1_BASE_ADDR,
            attrs->intrNum0 = CSLR_GICSS0_SPI_MCAN1_MCANSS_MCAN_LVL_INT_0;
            attrs->intrNum1 = CSLR_GICSS0_SPI_MCAN1_MCANSS_MCAN_LVL_INT_1;
#else
            testParams->canfdInstance = CONFIG_MCAN2;
            config = &gCanfdConfig[CONFIG_MCAN2];
            attrs  = (CANFD_Attrs *)config->attrs;
            attrs->operMode = CANFD_OPER_MODE_INTERRUPT;
            attrs->baseAddr = CONFIG_MCAN2_BASE_ADDR,
            attrs->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN2_MCAN_LVL_INT_0,
            attrs->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN2_MCAN_LVL_INT_1,
#endif
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum                = 0U,
            openParams->msgRAMConfig.txFIFOSize              = 10U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10U,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL;
            /* mcan module bit timing parameters. 1000kbps and 2500 kbps */
            bitTimingParams->nomBrp          = 2U,
            bitTimingParams->nomPropSeg      = 11U,
            bitTimingParams->nomPseg1        = 22U,
            bitTimingParams->nomPseg2        = 6U,
            bitTimingParams->nomSjw          = 1U,
            bitTimingParams->dataBrp         = 1U,
            bitTimingParams->dataPropSeg     = 13U,
            bitTimingParams->dataPseg1       = 14U,
            bitTimingParams->dataPseg2       = 4U,
            bitTimingParams->dataSjw         = 1U,
            /* mcan module configuration parameters */
            testParams->txMsgParams = &canTxMsg[7U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            testParams->rxFifoNum   = canTxMsg[7U].rxFifoNum;
            canExtIdFilter[0U].efec = canTxMsg[7U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[7U].rxfilterType;
            break;
        case 13922:
            /*
            * MCAN: CAN FD Mode with Bit Rate Switching enabled and additional bitrate 
            * configuration of 250kbps/2.5Mbp.
            * In this mode, both transmitted and received messages are stored in the buffer.
            */

            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum                = 10U,
            openParams->msgRAMConfig.txFIFOSize              = 0U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 3U,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL;
            /* mcan module bit timing parameters. 250kbps and 5000 kbps */
            bitTimingParams->nomBrp          = 10U,
            bitTimingParams->nomPropSeg      = 13U,
            bitTimingParams->nomPseg1        = 14U,
            bitTimingParams->nomPseg2        = 4U,
            bitTimingParams->nomSjw          = 1U,
            bitTimingParams->dataBrp         = 1U,
            bitTimingParams->dataPropSeg     = 2U,
            bitTimingParams->dataPseg1       = 4U,
            bitTimingParams->dataPseg2       = 1U,
            bitTimingParams->dataSjw         = 1U,
            /* mcan module configuration parameters */
            testParams->txMsgParams = &canTxMsg[0U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            testParams->rxFifoNum = canTxMsg[0U].rxFifoNum;
            canExtIdFilter[0U].efec = canTxMsg[0U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[0U].rxfilterType;
            break;
        case 13923:
            /*
            * MCAN: CAN FD Mode Test with Bit Rate Switching enabled and configured 
            * for a bitrate of 125kbps/5Mbps in internal loopback mode.
            * In this mode, both transmitted and received messages are stored in the buffer
            */

#if defined (SOC_AM26PX)
            /* AM263px has 8 mcan instances, instance 4 is being tested */
            testParams->canfdInstance = CONFIG_MCAN4;
            config = &gCanfdConfig[CONFIG_MCAN4];
            attrs  = (CANFD_Attrs *)config->attrs;
            attrs->operMode = CANFD_OPER_MODE_INTERRUPT;
            attrs->baseAddr = CONFIG_MCAN4_BASE_ADDR,
            attrs->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN4_MCAN_LVL_INT_0,
            attrs->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN4_MCAN_LVL_INT_1,
#endif
            /* mcan module bit timing parameters. 125kbps and 5000 kbps */
            bitTimingParams->nomBrp          = 20U,
            bitTimingParams->nomPropSeg      = 13U,
            bitTimingParams->nomPseg1        = 14U,
            bitTimingParams->nomPseg2        = 4U,
            bitTimingParams->nomSjw          = 1U,
            bitTimingParams->dataBrp         = 1U,
            bitTimingParams->dataPropSeg     = 2U,
            bitTimingParams->dataPseg1       = 4U,
            bitTimingParams->dataPseg2       = 1U,
            bitTimingParams->dataSjw         = 1U,
            /* mcan module configuration parameters */
            testParams->txMsgParams = &canTxMsg[0U];
            testParams->rxFifoNum = canTxMsg[0U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[0U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[0U].rxfilterType;
            break;
        case 13924:
            /*
             * CAN FD Mode Test with Bit Rate Switching enabled and Extended ID in internal loopback mode.
             * In this mode, both transmitted and received messages are stored in the buffer.
             */

#if defined (SOC_AM26PX)
            /* AM263px has 8 mcan instances, instance 5 is being tested */
            testParams->canfdInstance = CONFIG_MCAN5;
            config = &gCanfdConfig[CONFIG_MCAN5];
            attrs  = (CANFD_Attrs *)config->attrs;
            attrs->operMode = CANFD_OPER_MODE_INTERRUPT;
            attrs->baseAddr = CONFIG_MCAN5_BASE_ADDR,
            attrs->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN5_MCAN_LVL_INT_0,
            attrs->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN5_MCAN_LVL_INT_1,
#endif
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[0U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            testParams->rxFifoNum = canTxMsg[0U].rxFifoNum;
            canExtIdFilter[0U].efec = canTxMsg[0U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[0U].rxfilterType;
            break;
        case 13925:
            /*
             * MCAN CANFD Loopback Classic CAN Test in internal loopback mode.
             * In this mode, both transmitted and received messages are stored 
             * in the FIFO.
             */

#if defined (SOC_AM26PX)
            /* AM263px has 8 mcan instances, instance 5 is being tested */
            testParams->canfdInstance = CONFIG_MCAN5;
            config = &gCanfdConfig[CONFIG_MCAN5];
            attrs  = (CANFD_Attrs *)config->attrs;
            attrs->operMode = CANFD_OPER_MODE_INTERRUPT;
            attrs->baseAddr = CONFIG_MCAN5_BASE_ADDR,
            attrs->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN5_MCAN_LVL_INT_0,
            attrs->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN5_MCAN_LVL_INT_1,
#endif
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum                = 0U,
            openParams->msgRAMConfig.txFIFOSize              = 10U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10U,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL;
            testParams->txMsgParams = &canTxMsg[21U];
            attrs->filterConfig = (MCAN_StdMsgIDFilterElement*) &canStdIDFilter[0U];  /* Standard message ID filters */
            testParams->rxFifoNum = canTxMsg[21U].rxFifoNum;
            canStdIDFilter[0U].sfec = canTxMsg[21U].filterElement;
            canStdIDFilter[0U].sft  = canTxMsg[21U].rxfilterType;
            break;
        case 13926:
            /*
             * MCAN CANFD LOOPBACK Transmit FIFO Test. In this test, transmitted
             * messages are stored in the FIFO and received back into the FIFO 
             * if they match the configured filter criteria. 
             */

#if defined (SOC_AM26PX)
            /* AM263px has 8 mcan instances, instance 6 is being tested */
            testParams->canfdInstance = CONFIG_MCAN6;
            config = &gCanfdConfig[CONFIG_MCAN6];
            attrs  = (CANFD_Attrs *)config->attrs;
            attrs->operMode = CANFD_OPER_MODE_INTERRUPT;
            attrs->baseAddr = CONFIG_MCAN6_BASE_ADDR,
            attrs->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN6_MCAN_LVL_INT_0,
            attrs->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN6_MCAN_LVL_INT_1,
#endif
            /*  Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum                = 0U,
            openParams->msgRAMConfig.txFIFOSize              = 10U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10U,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL;
            testParams->txMsgParams = &canTxMsg[3U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            testParams->rxFifoNum = canTxMsg[3U].rxFifoNum;
            canExtIdFilter[0U].efec = canTxMsg[3U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[3U].rxfilterType;
            break;
        case 13927:
            /* MCAN CANFD LOOPBACK Tx Mixed Config. With Buffer and Queue*/
#if defined (SOC_AM26PX)
            /* AM263px has 8 mcan instances, instance 7 is being tested */
            testParams->canfdInstance = CONFIG_MCAN7;
            config = &gCanfdConfig[CONFIG_MCAN7];
            attrs  = (CANFD_Attrs *)config->attrs;
            attrs->baseAddr = CONFIG_MCAN7_BASE_ADDR,
            attrs->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN7_MCAN_LVL_INT_0,
            attrs->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN7_MCAN_LVL_INT_1,
#endif
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.lss          = APP_MCAN_STD_ID_FILTER_NUM,
            openParams->msgRAMConfig.lse          = APP_MCAN_EXT_ID_FILTER_NUM,
            openParams->msgRAMConfig.txBufNum     = 0,
            openParams->msgRAMConfig.txFIFOSize              = 10U,
            openParams->msgRAMConfig.txBufMode               = 1U, /* Tx FIFO/QUEUE operation */
            openParams->msgRAMConfig.txEventFIFOSize         = 10,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0size             = APP_MCAN_FIFO_0_NUM,
            openParams->msgRAMConfig.rxFIFO0waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0OpMode           = 0U,
            openParams->msgRAMConfig.rxFIFO1size             = APP_MCAN_FIFO_1_NUM,
            openParams->msgRAMConfig.rxFIFO1waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO1OpMode           = 0U,
            testParams->txMsgParams = &canTxMsg[8U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            testParams->rxFifoNum = canTxMsg[8U].rxFifoNum;
            canExtIdFilter[0U].efec = canTxMsg[8U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[8U].rxfilterType;
            break;
        case 13928: 
            /* MCAN CANFD LOOPBACK Mixed Tx Buffers and FIFO Test
             * This test case utilizes both the buffer and FIFO on the TX side for transmission, 
             * while the RX side exclusively uses FIFO for receiving messages. 
             */
            testParams->txMsgParams = &canTxMsg[8U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            testParams->rxFifoNum = canTxMsg[8U].rxFifoNum;
            canExtIdFilter[0U].efec = canTxMsg[8U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[8U].rxfilterType;
            break;
        case 13929:
             /*
              * MCAN CANFD Loopback Test for Maximum TX Buffer Size Configuration. 
              * In this test, both the TX and RX sides use buffers for message storage, 
              * ensuring validation of maximum buffer utilization during transmission and reception.
              */
            openParams->tsSelect                  = 1U;
            openParams->msgRAMConfig.txBufNum     = APP_MCAN_TX_BUFF_MAX_SIZE,
            openParams->msgRAMConfig.txFIFOSize   = 0U,
            testParams->txMsgParams = &canTxMsg[1U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            testParams->rxFifoNum = canTxMsg[1U].rxFifoNum;
            canExtIdFilter[0U].efec = canTxMsg[1U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[1U].rxfilterType;
            break;
        case 13930:
            /*
            * MCAN CANFD LOOPBACK Mixed Rx Buffers and FIFO Test
            * MCAN CANFD Loopback Test with Mixed RX Buffers and FIFO Configuration.
            * In this test case, the TX side exclusively uses the buffer for transmission, 
            * while the RX side leverages both the buffer and FIFO for receiving messages. 
            * This configuration facilitates a thorough validation of mixed storage setups, 
            * ensuring comprehensive testing of the system's capabilities.
            */
            openParams->tsSelect    = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            testParams->rxFifoNum = canTxMsg[2U].rxFifoNum;
            canExtIdFilter[0U].efec = canTxMsg[2U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13931:
            /* 
             * MCAN CANFD LOOPBACK Receive FIFO 0 Test. In this mode, 
             * both transmitted and received messages are stored 
             * in the buffer and FIFO0 respectively.
             */
            openParams->msgRAMConfig.txBufNum                = 0,
            openParams->msgRAMConfig.txFIFOSize              = 10U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10U,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL;
            testParams->txMsgParams = &canTxMsg[5U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            testParams->rxFifoNum = canTxMsg[5U].rxFifoNum;
            canExtIdFilter[0U].efec = canTxMsg[5U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[5U].rxfilterType;
            break;
        case 13933:
            /* 
             * MCAN CANFD LOOPBACK Receive FIFO 1 Test. In this mode, 
             * both transmitted and received messages are stored 
             * in the buffer and FIFO 1 respectively.
             */
            openParams->tsSelect = 1U;
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufMode     = 1U;
            openParams->msgRAMConfig.rxFIFO0size   = 5U;
            openParams->msgRAMConfig.rxFIFO0OpMode = 1U;
            openParams->msgRAMConfig.rxFIFO1size   = 5U;
            openParams->msgRAMConfig.rxFIFO1OpMode = 1U;
            testParams->txMsgParams  = &canTxMsg[15U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            testParams->rxFifoNum = canTxMsg[15U].rxFifoNum;
            canExtIdFilter[0U].efec = canTxMsg[15U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[15U].rxfilterType;
            break;
        case 13934:
            /*
            * MCAN CANFD Loopback Test for Receive FIFO 0 Message Lost Condition.
            * This test configures the RX FIFO in blocking mode. Once the buffer is full, 
            * any incoming messages will be rejected or lost, triggering the corresponding 
            * interrupt to indicate the loss. This setup validates the behavior of the 
            * system under FIFO overflow conditions.
            */
            openParams->tsSelect    = 1U;
            openParams->filterConfig.anfe = true,
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum          = 0U,
            openParams->msgRAMConfig.txFIFOSize        = 2U,
            openParams->msgRAMConfig.txBufMode         = 0U, /* Tx FIFO/QUEUE operation */
            openParams->msgRAMConfig.txEventFIFOSize   = 2U,
            openParams->msgRAMConfig.txEventFIFOWaterMark = 2,
            openParams->msgRAMConfig.rxFIFO0size   = 2U;
            openParams->msgRAMConfig.rxFIFO0OpMode = 0U;
            openParams->msgRAMConfig.rxFIFO1size   = 2U;
            openParams->msgRAMConfig.rxFIFO1OpMode = 0U;
            openParams->msgRAMConfig.rxFIFO0waterMark  = 2,
            openParams->msgRAMConfig.rxFIFO1waterMark  = 2,
            testParams->txMsgParams  = &canTxMsg[22U];
            testParams->rxFifoNum = MCAN_RX_FIFO_NUM_0;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[22U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[22U].rxfilterType;
            break;
        case 13935:
            /*
            * MCAN CANFD Loopback Test for Receive FIFO 1 Message Lost Condition.
            * This test configures the RX FIFO in blocking mode. Once the buffer is full, 
            * any incoming messages will be rejected or lost, triggering the corresponding 
            * interrupt to indicate the loss. This setup validates the behavior of the 
            * system under FIFO overflow conditions.
            */
            openParams->tsSelect = 1U;
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum      = 0U,
            openParams->msgRAMConfig.txFIFOSize    = 2U,
            openParams->msgRAMConfig.txBufMode     = 0U;
            openParams->msgRAMConfig.txEventFIFOSize      = 2U,
            openParams->msgRAMConfig.txEventFIFOWaterMark = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0size   = 2U;
            openParams->msgRAMConfig.rxFIFO0OpMode = 0U;
            openParams->msgRAMConfig.rxFIFO1size   = 2U;
            openParams->msgRAMConfig.rxFIFO1OpMode = 0U;
            openParams->msgRAMConfig.rxFIFO0waterMark  = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO1waterMark  = App_MCAN_FIFO_WATERMARK_LEVEL,
            testParams->txMsgParams  = &canTxMsg[22U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            testParams->rxFifoNum = canTxMsg[22U].rxFifoNum;
            canExtIdFilter[0U].efec = canTxMsg[22U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[22U].rxfilterType;
            break;
        case 13936:
            /*
            * This test case configures the RX buffer or FIFO to its maximum size.
            * It performs a Maximum RX Buffer/FIFO Size Configuration Test. 
            */
            openParams->msgRAMConfig.txBufNum                = 0U,
            openParams->msgRAMConfig.txFIFOSize              = 32U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0size             = APP_MCAN_FIFO_0_NUM,
            openParams->msgRAMConfig.rxFIFO0waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0OpMode           = 0U,
            openParams->msgRAMConfig.rxFIFO1size             = APP_MCAN_FIFO_1_NUM,
            openParams->msgRAMConfig.rxFIFO1waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO1OpMode           = 0U,
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            testParams->rxFifoNum = canTxMsg[2U].rxFifoNum;
            canExtIdFilter[0U].efec = canTxMsg[2U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13937:
            /*
            * This test is designed for performance analysis of CAN Standard IDs, 
            * with a focus on TX and RX throughput. It evaluates the efficiency and 
            * reliability of data transmission and reception under varying conditions. 
            * Both TX and RX storage types are configured as buffers for this test.
            */
            openParams->fdMode = false;
            testParams->txMsgParams = &canTxMsg[18U];
            testParams->rxFifoNum = canTxMsg[18U].rxFifoNum;
            attrs->filterConfig = (MCAN_StdMsgIDFilterElement*) &canStdIDFilter[0U];
            canStdIDFilter[0U].sfec = MCAN_EXT_FILT_ELEM_BUFFER;
            canStdIDFilter[0U].sft  = MCAN_EXT_FILT_TYPE_DISABLE;
            break;
        case 13938:
            /*
            * This test is designed for performance analysis of CAN Extended IDs, 
            * with a focus on TX and RX throughput. It evaluates the efficiency and 
            * reliability of data transmission and reception under varying conditions. 
            * Both TX and RX storage types are configured as buffers for this test.
            */
            openParams->tsSelect = 0U;
             /* Default Bit-Rate 1000kbps and 5000 kbps*/
            bitTimingParams->nomBrp          = 3U,
            bitTimingParams->nomPropSeg      = 7U,
            bitTimingParams->nomPseg1        = 6U,
            bitTimingParams->nomPseg2        = 4U,
            bitTimingParams->nomSjw          = 0U,
            bitTimingParams->dataBrp         = 1U,
            bitTimingParams->dataPropSeg     = 2U,
            bitTimingParams->dataPseg1       = 3U,
            bitTimingParams->dataPseg2       = 0U,
            bitTimingParams->dataSjw         = 0U,

            /* MCAN CANFD LOOPBACK Maximum Rx Buffer/FIFO Size Configuration Test */
            openParams->msgRAMConfig.lss          = 1U,
            openParams->msgRAMConfig.lse          = 1U,
            openParams->msgRAMConfig.txBufNum                = 10U,
            openParams->msgRAMConfig.txFIFOSize              = 10U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10U,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0size             = 10U,
            openParams->msgRAMConfig.rxFIFO0waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0OpMode           = 0U,
            openParams->msgRAMConfig.rxFIFO1size             = 10U,
            openParams->msgRAMConfig.rxFIFO1waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO1OpMode           = 0U,

            testParams->txMsgParams = &canTxMsg[0U];
            testParams->rxFifoNum = canTxMsg[4U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[4U];
            canExtIdFilter[3U].efec = MCAN_EXT_FILT_ELEM_BUFFER;
            canExtIdFilter[3U].eft  = MCAN_EXT_FILT_TYPE_RANGE;
            break;
        case 13939:
            /*
            * MCAN CANFD Loopback High Priority Message Notification Test.
            * This test is designed to validate the handling of high-priority messages 
            * in loopback mode, ensuring accurate notification and processing of such 
            * messages within the system.
            */
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum     = 0,
            openParams->msgRAMConfig.txFIFOSize              = 32U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0size             = APP_MCAN_FIFO_0_NUM,
            openParams->msgRAMConfig.rxFIFO0waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0OpMode           = 0U,
            openParams->msgRAMConfig.rxFIFO1size             = APP_MCAN_FIFO_1_NUM,
            openParams->msgRAMConfig.rxFIFO1waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO1OpMode           = 0U,
            testParams->txMsgParams = &canTxMsg[7U];
            testParams->rxFifoNum = canTxMsg[7U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[7U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[7U].rxfilterType;
            break;
        case 13940:
            /*
            * This test is designed to demonstrate task cancellation functionality. 
            * In this implementation, two tasks are created, where one task is responsible 
            * for canceling the other. This setup ensures controlled and reliable 
            * termination of the designated task.
            */
            testParams->txMsgParams = &canTxMsg[0U];
            testParams->rxFifoNum = canTxMsg[0U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[0U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[0U].rxfilterType;
            break;
        case 13941:
            /*
            * This test is designed to demonstrate the Automatic Retransmission feature 
            * of the CAN module.
            */
            openParams->darEnable   = false,
            openParams->tsSelect    = 1U;
            testParams->txMsgParams = &canTxMsg[1U];
            testParams->rxFifoNum = canTxMsg[1U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[1U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[1U].rxfilterType;
            break;
        case 13942:
            /*
            * This test is designed to demonstrate the Transmitter Delay Compensation feature 
            * of the CAN module. It validates the module's ability to account for transmission 
            * delays, ensuring accurate and synchronized communication across the network.
            */
            openParams->tdcEnable   = true,
            openParams->tsSelect    = 1U;
            testParams->txMsgParams = &canTxMsg[1U];
            testParams->rxFifoNum = canTxMsg[1U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[1U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[1U].rxfilterType;
            break;
        case 13943:
            /*
            * This test configures Interrupt Line 1 and validates the interrupt handling 
            * functionality on that line. It is designed as part of the CAN CANFD Loopback 
            * Interrupt Line 1 Test, ensuring proper configuration and accurate response to 
            * interrupts triggered on Line 1.
            */
            testParams->txMsgParams  = &canTxMsg[1U];
            testParams->rxFifoNum = canTxMsg[1U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[1U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[1U].rxfilterType;
            break;
        case 13944:
            /* This test retrieves and prints the revision details along with the basic 
             * CAN controller information.
             */
            testParams->txMsgParams  = &canTxMsg[1U];
            testParams->rxFifoNum = canTxMsg[1U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[1U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[1U].rxfilterType;
            break;
        case 13945:
            /*
            * This test retrieves the current state of the TX and RX pins of the CAN controller. 
            * It provides a detailed status report, enabling monitoring and diagnostics 
            * of communication functionality.
            */
            testParams->txMsgParams = &canTxMsg[1U];
            testParams->rxFifoNum = canTxMsg[1U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[1U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[1U].rxfilterType;
            break;
        case 13946:
            /*
            * This test retrieves and validates the endianness value of the MCAN module. 
            * It ensures accurate detection and confirmation of the module's byte order 
            * configuration, supporting reliable data processing and communication.
            */
            openParams->tsSelect    = 1U;
            testParams->txMsgParams = &canTxMsg[1U];
            testParams->rxFifoNum = canTxMsg[1U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[1U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[1U].rxfilterType;
            break;
        case 13947:
            /*
            * MCAN CANFD Loopback Global Filter Test.
            * This test validates the functionality of the global filter in loopback mode, 
            * ensuring accurate processing and handling of messages based on global filtering criteria.
            */
            openParams->msgRAMConfig.txBufNum                = 0U,
            openParams->msgRAMConfig.txFIFOSize              = 10U,
            openParams->msgRAMConfig.txBufMode               = 0U, 
            openParams->msgRAMConfig.rxFIFO0OpMode           = 10U,
            openParams->msgRAMConfig.rxFIFO0OpMode           = 1U,
            openParams->msgRAMConfig.rxFIFO1OpMode           = 1U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10U,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->tsSelect = 1U;
            /* Global Filter Configuration parameters. */
            openParams->filterConfig.rrfs = true,
            testParams->txMsgParams  = &canTxMsg[17U];
            testParams->rxFifoNum = MCAN_RX_FIFO_NUM_0;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = 0U;
            canExtIdFilter[0U].eft  = 1U;
            break;
        case 13948:
            /*
            * MCAN CANFD Loopback Acceptance Filter Range Filter Test.
            * This test validates the functionality of the acceptance range filter in loopback mode, 
            * ensuring accurate processing of messages that fall within the specified range criteria.
            */
            openParams->tsSelect = 1U;
            openParams->msgRAMConfig.txBufNum                = 0U,
            openParams->msgRAMConfig.txFIFOSize              = 32U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0size             = APP_MCAN_FIFO_0_NUM,
            openParams->msgRAMConfig.rxFIFO0waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0OpMode           = 0U,
            openParams->msgRAMConfig.rxFIFO1size             = APP_MCAN_FIFO_1_NUM,
            openParams->msgRAMConfig.rxFIFO1waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO1OpMode           = 0U,
            testParams->txMsgParams = &canTxMsg[15U];
            testParams->rxFifoNum = MCAN_RX_FIFO_NUM_0;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[15U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[15U].rxfilterType;
            break;
        case 13949:
            /*
            * MCAN CANFD Loopback Acceptance Filter Specific ID Filter Test.
            * This test validates the functionality of the acceptance filter by using 
            * specific ID criteria. It ensures accurate filtering and processing of 
            * messages that match the predefined IDs.
            */
            openParams->tsSelect = 1U;
            openParams->msgRAMConfig.txBufNum                = 0,
            openParams->msgRAMConfig.txFIFOSize              = 32U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0size             = APP_MCAN_FIFO_0_NUM,
            openParams->msgRAMConfig.rxFIFO0waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO0OpMode           = 0U,
            openParams->msgRAMConfig.rxFIFO1size             = APP_MCAN_FIFO_1_NUM,
            openParams->msgRAMConfig.rxFIFO1waterMark        = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->msgRAMConfig.rxFIFO1OpMode           = 0U,
            testParams->txMsgParams = &canTxMsg[18U];
            testParams->rxFifoNum = canTxMsg[18U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[4U];
            canExtIdFilter[0U].efec = canTxMsg[18U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[18U].rxfilterType;
            break;
        case 13950:
            /*
            * MCAN CANFD Loopback Acceptance Filter Classic BitMask Filter Test.
            * This test validates the functionality of the acceptance filter using the 
            * classic bitmask filtering approach. It ensures accurate message filtering 
            * and processing based on the specified bitmask criteria.
            */
            /* mcan module bit timing parameters. 1000kbps and 2500 kbps */
            bitTimingParams->nomBrp          = 2U,
            bitTimingParams->nomPropSeg      = 11U,
            bitTimingParams->nomPseg1        = 22U,
            bitTimingParams->nomPseg2        = 6U,
            bitTimingParams->nomSjw          = 1U,
            bitTimingParams->dataBrp         = 1U,
            bitTimingParams->dataPropSeg     = 13U,
            bitTimingParams->dataPseg1       = 14U,
            bitTimingParams->dataPseg2       = 4U,
            bitTimingParams->dataSjw         = 1U,
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum                = 0U,
            openParams->msgRAMConfig.txFIFOSize              = 32U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10U,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
            testParams->txMsgParams = &canTxMsg[7U];
            testParams->rxFifoNum = canTxMsg[7U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[7U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[7U].rxfilterType;
            break;
        case 13951:
            /*
            * MCAN CANFD Loopback External Time-Stamp Code Coverage Improvement Test.
            * This test validates the functionality of the external time-stamp feature 
            * and focuses on enhancing code coverage through detailed testing scenarios.
            */
            openParams->tsSelect = 2U;
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum                = 0U,
            openParams->msgRAMConfig.txFIFOSize              = 32U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
            testParams->txMsgParams = &canTxMsg[2U];
            testParams->rxFifoNum = canTxMsg[2U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[2U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13952:
            /*
            * MCAN CANFD Loopback Time Stamp Counter Reset and Code Coverage Improvement Test.
            * This test validates the Time Stamp Counter Reset functionality and focuses on 
            * enhancing code coverage through comprehensive testing scenarios.
            */
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum                = 0U,
            openParams->msgRAMConfig.txFIFOSize              = 32U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            testParams->rxFifoNum = canTxMsg[2U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[2U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13953:
            /*
            * CANFD Loopback Test in Polled Mode.
            * This test is performed in polled mode with both TX and RX memory storage 
            * types configured as Buffer. It validates the loopback functionality. 
            */
            attrs->operMode    = CANFD_OPER_MODE_POLLED,
            /* mcan module bit timing parameters. 250 kbps and 5000 kbps */
            bitTimingParams->nomBrp          = 10U,
            bitTimingParams->nomPropSeg      = 13U,
            bitTimingParams->nomPseg1        = 14U,
            bitTimingParams->nomPseg2        = 4U,
            bitTimingParams->nomSjw          = 1U,
            bitTimingParams->dataBrp         = 1U,
            bitTimingParams->dataPropSeg     = 2U,
            bitTimingParams->dataPseg1       = 4U,
            bitTimingParams->dataPseg2       = 1U,
            bitTimingParams->dataSjw         = 1U,
            testParams->txMsgParams = &canTxMsg[0U];
            testParams->rxFifoNum = canTxMsg[0U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[0U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[0U].rxfilterType;
            break;
        case 13954:
            /*
            * This test is responsible for configuring and validating multiple instances 
            * running simultaneously. It ensures proper setup and checks the functionality 
            * of each instance to maintain smooth operation and avoid conflicts.
            */
            openParams->tsSelect = 1U;
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum                = 10U,
            openParams->msgRAMConfig.txFIFOSize              = 0U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL,
            testParams->txMsgParams = &canTxMsg[0U];
            testParams->rxFifoNum = canTxMsg[0U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[0U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[0U].rxfilterType;
            break;
        case 13955:
            /*
            * MCAN CANFD Clock Stop Request Test.
            * This test validates the functionality of the clock stop request feature 
            * in the MCAN CANFD module, ensuring proper handling and system response 
            * during clock stop scenarios.
            */
            testParams->txMsgParams = &canTxMsg[2U];
            testParams->rxFifoNum = canTxMsg[2U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[2U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13956:
            /*
            * MCAN module bit timing parameters: 1000kbps and 5000kbps.
            * This test retrieves and validates the configured bit rates, 
            * ensuring correct setup and performance of the communication module.
            */
            bitTimingParams->nomBrp          = 2U,
            bitTimingParams->nomPropSeg      = 11U,
            bitTimingParams->nomPseg1        = 22U,
            bitTimingParams->nomPseg2        = 6U,
            bitTimingParams->nomSjw          = 1U,
            bitTimingParams->dataBrp         = 1U,
            bitTimingParams->dataPropSeg     = 10U,
            bitTimingParams->dataPseg1       = 5U,
            bitTimingParams->dataPseg2       = 0U,
            bitTimingParams->dataSjw         = 1U,
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum          = 0U,
            openParams->msgRAMConfig.txFIFOSize        = 32U;
            openParams->msgRAMConfig.txBufMode         = 1U;
            openParams->msgRAMConfig.txEventFIFOSize   = APP_MCAN_TX_BUFF_SIZE;
            openParams->msgRAMConfig.rxFIFO0size       = 5U;
            openParams->msgRAMConfig.rxFIFO0OpMode     = 1U;
            openParams->msgRAMConfig.rxFIFO1size       = 5U;
            testParams->txMsgParams = &canTxMsg[2U];
            testParams->rxFifoNum = canTxMsg[2U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[2U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13957:
            /*
            * MCAN CANFD Loopback Acceptance Filter Range Filter Test 2.
            * This test validates the functionality of the acceptance range filter 
            * in loopback mode, ensuring accurate filtering and processing of messages 
            * that fall within the specified range criteria.
            */
            testParams->txMsgParams = &canTxMsg[15U];
            testParams->rxFifoNum = canTxMsg[15U].rxFifoNum;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[3U];
            canExtIdFilter[0U].efec = canTxMsg[15U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[15U].rxfilterType;
            break;
        case 13958:
            /*
            * CANFD Loopback Test in Polled Mode.
            * This test is performed in polled mode with both TX and RX memory storage 
            * types configured as FIFO. It validates the loopback functionality. 
            */
            attrs->operMode    = CANFD_OPER_MODE_POLLED;
            testParams->rxFifoNum = MCAN_RX_FIFO_NUM_0;
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum                = 0U,
            openParams->msgRAMConfig.txFIFOSize              = 10U,
            openParams->msgRAMConfig.txBufMode               = 0U,
            openParams->msgRAMConfig.txEventFIFOSize         = 10U,
            openParams->msgRAMConfig.txEventFIFOWaterMark    = App_MCAN_FIFO_WATERMARK_LEVEL;
            openParams->msgRAMConfig.rxFIFO0size       = 5U;
            openParams->msgRAMConfig.rxFIFO0OpMode     = 0U;
            openParams->msgRAMConfig.rxFIFO1size       = 5U;
            testParams->txMsgParams = &canTxMsg[3U];
            testParams->rxFifoNum = canTxMsg[3U].rxFifoNum;
            testParams->txMsgParams->rxMemType = MCAN_MEM_TYPE_FIFO;
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];
            canExtIdFilter[0U].efec = canTxMsg[3U].filterElement;
            canExtIdFilter[0U].eft  = canTxMsg[3U].rxfilterType;
            break;
        default:
            break;
    }

    return;
}

void setUp(void)
{
}

void tearDown(void)
{
}

void App_CANFD_TransferCallback(void *args, CANFD_Reason reason)
{
    if ((reason == CANFD_Reason_TX_COMPLETION) || (reason == CANFD_Reason_TX_CANCELED))
    {
        /* condition to post semaphore for MCAN instance 0 */
        if(gFlagInstance1 == 1U)
        {
            /* condition to post semaphore for MCAN instance 1 */
            SemaphoreP_post((SemaphoreP_Object *)&gMcanTxDoneSem1);
        }
        else
        {
            SemaphoreP_post((SemaphoreP_Object *)&gMcanTxDoneSem);
        }
    }
    if (reason == CANFD_Reason_RX)
    {
        /* condition to post semaphore for MCAN instance 1 */
        if(gFlagInstance1 == 1U)
        {
            SemaphoreP_post((SemaphoreP_Object *)&gMcanRxDoneSem1);
        }
        else
        {
            SemaphoreP_post((SemaphoreP_Object *)&gMcanRxDoneSem);
        }
    }
}

void App_CANFD_ErrorCallback(void *args, CANFD_Reason reason, CANFD_ErrStatusResp* errStatusResp)
{
    CANFD_TestParams *testParams = (CANFD_TestParams *)args;

    if(args != NULL)
    {
        testParams->reason = reason;
        if ((reason == CANFD_Reason_SRC_RX_FIFO0_MSG_LOST) || (reason == CANFD_Reason_SRC_RX_FIFO1_MSG_LOST))
        {
            SemaphoreP_post((SemaphoreP_Object *)&gMcanTxDoneSem);
        }
    }
}
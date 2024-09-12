/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

/* This example demonstrates the CAN message transmission and reception in
 * digital loop back mode with the following configuration.
 *
 * CAN FD Message Format.
 * Message ID Type is Standard, Msg Id 0xC0.
 * MCAN is configured in Interrupt Mode.
 * MCAN Interrupt Line Number 0.
 * Arbitration Bit Rate 1Mbps.
 * Data Bit Rate 5Mbps.
 * Buffer mode is used for Tx and RX to store message in message RAM.
 *
 * Message is transmitted and received back internally using internal loopback
 * mode. When the received message id and the data matches with the transmitted
 * one, then the example is completed.
 *
 */

#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/mcan/v0/canfd.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/** \brief Number of messages sent */
#define MCAN_APP_TEST_MESSAGE_COUNT         10U
/** \brief Number of messages sent */
#define MCAN_APP_TEST_DATA_SIZE             64U

CANFD_MessageObject         txMsgObject;
CANFD_MessageObject         rxMsgObject;
CANFD_Object               *canfdObject;
static SemaphoreP_Object gMcanTxDoneSem, gMcanRxDoneSem;
uint8_t  txData[MCAN_APP_TEST_MESSAGE_COUNT][MCAN_APP_TEST_DATA_SIZE] = {{0}};
uint8_t  rxData[MCAN_APP_TEST_MESSAGE_COUNT][MCAN_APP_TEST_DATA_SIZE] = {{0}};

void canfd_loopback_dma_main(void *args)
{
    CANFD_MsgObjHandle          txMsgObjHandle;
    CANFD_MsgObjHandle          rxMsgObjHandle;
    int32_t                     status = 0;
    int32_t                     errDataMismatch = SystemP_SUCCESS;
    uint32_t                    i,j;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    status = SemaphoreP_constructBinary(&gMcanTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status = SemaphoreP_constructBinary(&gMcanRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* initialize the data buffers */
    for (i = 0; i < MCAN_APP_TEST_MESSAGE_COUNT; i++)
    {
        for (j = 0; j < MCAN_APP_TEST_DATA_SIZE; j++)
        {
            txData[i][j] = i + j;
            rxData[i][j] = 0U;
        }
    }
    /* Writeback buffer */
    CacheP_wb(&txData[0U], sizeof(txData), CacheP_TYPE_ALLD);
    CacheP_wb(&rxData[0U], sizeof(rxData), CacheP_TYPE_ALLD);

    DebugP_log("\n\n[MCAN] CANFD Loopback DMA mode, application started ...\r\n");

    /* Setup the receive message object */
    rxMsgObject.direction  = CANFD_Direction_RX;
    rxMsgObject.msgIdType  = CANFD_MCANXidType_29_BIT;
    rxMsgObject.startMsgId = 0x29E;
    rxMsgObject.endMsgId   = 0x29E;
    rxMsgObject.args       = (uint8_t*) rxData;
    rxMsgObject.rxMemType  = MCAN_MEM_TYPE_BUF;
    rxMsgObject.dataLength = MCAN_APP_TEST_DATA_SIZE;

    status = CANFD_createMsgObject (gCanfdHandle[CONFIG_MCAN0], &rxMsgObject);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Rx message object failed\r\n");
        return;
    }
    rxMsgObjHandle = &rxMsgObject;

    /* Setup the transmit message object */
    txMsgObject.direction     = CANFD_Direction_TX;
    txMsgObject.msgIdType     = CANFD_MCANXidType_29_BIT;
    txMsgObject.startMsgId    = 0x29E;
    txMsgObject.endMsgId      = 0x29E;
    txMsgObject.txMemType     = MCAN_MEM_TYPE_BUF;
    txMsgObject.dataLength    = MCAN_APP_TEST_DATA_SIZE;
    txMsgObject.args          = NULL;

    status = CANFD_createMsgObject (gCanfdHandle[CONFIG_MCAN0], &txMsgObject);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Tx message object failed\r\n");
        return;
    }
    txMsgObjHandle = &txMsgObject;

    status = CANFD_read(rxMsgObjHandle, MCAN_APP_TEST_MESSAGE_COUNT, &rxData[0][0]);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD read dma config failed\r\n");
        return;
    }

    status = CANFD_write(txMsgObjHandle, txMsgObject.startMsgId, CANFD_MCANFrameType_FD, MCAN_APP_TEST_MESSAGE_COUNT, &txData[0][0]);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD write dma config failed\r\n");
        return;
    }

    /* Wait for Tx completion */
    SemaphoreP_pend(&gMcanTxDoneSem, SystemP_WAIT_FOREVER);
    /* Wait for Rx completion */
    SemaphoreP_pend(&gMcanRxDoneSem, SystemP_WAIT_FOREVER);
    /* Invalidate cache */
    CacheP_inv(&rxData, sizeof(rxData), CacheP_TYPE_ALLD);

    /* Compare data */
    for(int32_t i = 0U; i < MCAN_APP_TEST_MESSAGE_COUNT; i++)
    {
        for(int32_t j = 0U; j < MCAN_APP_TEST_DATA_SIZE; j++)
        {
            if(txData[i][j] != rxData[i][j])
            {
                errDataMismatch = SystemP_FAILURE;   /* Data mismatch */
                DebugP_log("Data Mismatch for message count %d at offset %d\r\n", i, j);
                break;
            }
        }
    }

    status = CANFD_deleteMsgObject(txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed \r\n");
        return;
    }

    status = CANFD_deleteMsgObject(rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Rx message object failed \r\n");
        return;
    }

    if ((status == SystemP_SUCCESS) && (errDataMismatch == SystemP_SUCCESS))
    {
        DebugP_log("[MCAN] CANFD Internal loopback testing for %d iterations Passed\r\n", MCAN_APP_TEST_MESSAGE_COUNT);
        DebugP_log(" All tests have passed.\r\n");
    }
    else
    {
        DebugP_log("[MCAN] CANFD Internal loopback testing for %d iterations Failed\r\n", MCAN_APP_TEST_MESSAGE_COUNT);
        DebugP_log(" Some tests have failed.\r\n");
    }

    /* Open drivers to open the UART driver for console */
    Drivers_close();
    Board_driversClose();

    return;
}

void App_CANFDTransferCallback(void *args, CANFD_Reason reason)
{
    if (reason == CANFD_Reason_TX_COMPLETION)
    {
        SemaphoreP_post((SemaphoreP_Object *)&gMcanTxDoneSem);
    }
    if (reason == CANFD_Reason_RX)
    {
        SemaphoreP_post((SemaphoreP_Object *)&gMcanRxDoneSem);
    }
}

void App_CANFDErrorCallback(void *args, CANFD_Reason reason)
{
    /* Do nothing. */
}

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

#include <string.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

/** \brief Number of messages sent */
#define MCAN_APP_TEST_MESSAGE_COUNT         100U

/** \brief Number of messages sent */
#define MCAN_APP_TEST_DATA_SIZE             64U

uint32_t gMcanVimStsAddr, intrNum;
uint32_t gMcanVimStsClrMask;
uint32_t intcBaseAddr;

CANFD_MessageObject         txMsgObject;
CANFD_MessageObject         rxMsgObject;
uint32_t                    intrNum;
volatile uint32_t           iterationCount = 0U;
uint8_t                     rxData[MCAN_APP_TEST_DATA_SIZE] = {0};
uint8_t                     txData[128U] =
                               {0xA1, 0x1A, 0xFF, 0xFF, 0xC1, 0x1C, 0xB1, 0x1B,
                                0xA2, 0x2A, 0xFF, 0xFF, 0xC2, 0x2C, 0xB2, 0x2B,
                                0xA3, 0x3A, 0xFF, 0xFF, 0xC3, 0x3C, 0xB3, 0x3B,
                                0xA4, 0x4A, 0xFF, 0xFF, 0xC4, 0x4C, 0xB4, 0x4B,
                                0xA5, 0x5A, 0xFF, 0xFF, 0xC5, 0x5C, 0xB5, 0x5B,
                                0xA6, 0x6A, 0xFF, 0xFF, 0xC6, 0x6C, 0xB6, 0x6B,
                                0xA7, 0x7A, 0xFF, 0xFF, 0xC7, 0x7C, 0xB7, 0x7B,
                                0xA8, 0x8A, 0xFF, 0xFF, 0xC8, 0x8C, 0xB8, 0x8B,
                                0xA1, 0x1A, 0xFF, 0xFF, 0xC1, 0x1C, 0xB1, 0x1B,
                                0xA2, 0x2A, 0xFF, 0xFF, 0xC2, 0x2C, 0xB2, 0x2B,
                                0xA3, 0x3A, 0xFF, 0xFF, 0xC3, 0x3C, 0xB3, 0x3B,
                                0xA4, 0x4A, 0xFF, 0xFF, 0xC4, 0x4C, 0xB4, 0x4B,
                                0xA5, 0x5A, 0xFF, 0xFF, 0xC5, 0x5C, 0xB5, 0x5B,
                                0xA6, 0x6A, 0xFF, 0xFF, 0xC6, 0x6C, 0xB6, 0x6B,
                                0xA7, 0x7A, 0xFF, 0xFF, 0xC7, 0x7C, 0xB7, 0x7B,
                                0xA8, 0x8A, 0xFF, 0xFF, 0xC8, 0x8C, 0xB8, 0x8B
                                };

void canfd_loopback_polling_main(void *args)
{
    CANFD_MsgObjHandle          txMsgObjHandle;
    CANFD_MsgObjHandle          rxMsgObjHandle;
    int32_t                     retVal = 0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[MCAN] Loopback Polling mode, application started ...\r\n");

    /* Setup the transmit message object */
    txMsgObject.direction  = CANFD_Direction_TX;
    txMsgObject.msgIdType  = CANFD_MCANXidType_29_BIT;
    txMsgObject.startMsgId = 0x29E;
    txMsgObject.endMsgId   = 0x29E;
    txMsgObject.txMemType  = MCAN_MEM_TYPE_BUF;
    txMsgObject.dataLength = MCAN_APP_TEST_DATA_SIZE;
    txMsgObject.args       = NULL;
    retVal = CANFD_createMsgObject (gCanfdHandle[CONFIG_MCAN0], &txMsgObject);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Tx message object failed\r\n");
        return;
    }
    txMsgObjHandle = &txMsgObject;

    /* Setup the receive message object */
    rxMsgObject.direction  = CANFD_Direction_RX;
    rxMsgObject.msgIdType  = CANFD_MCANXidType_29_BIT;
    rxMsgObject.startMsgId = 0x29E;
    rxMsgObject.endMsgId   = 0x29E;
    rxMsgObject.args       = (uint8_t*) rxData;
    rxMsgObject.rxMemType  = MCAN_MEM_TYPE_BUF;
    rxMsgObject.dataLength = MCAN_APP_TEST_DATA_SIZE;
    retVal = CANFD_createMsgObject (gCanfdHandle[CONFIG_MCAN0], &rxMsgObject);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Rx message object failed\r\n");
        return;
    }
    rxMsgObjHandle = &rxMsgObject;

    while (iterationCount != MCAN_APP_TEST_MESSAGE_COUNT)
    {
        /* Send data over Tx message object */
        retVal += CANFD_write (txMsgObjHandle,
                               txMsgObject.startMsgId,
                               CANFD_MCANFrameType_FD,
                               10,
                               &txData[0]);

        /* Send data over Tx message object */
        retVal += CANFD_read (rxMsgObjHandle, 0, &rxData[0]);

        /* Compare data */
        for(int i = 0U; i < MCAN_APP_TEST_DATA_SIZE; i++)
        {
            if(txData[i] != rxData[i])
            {
                retVal = SystemP_FAILURE;   /* Data mismatch */
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
        }

        if (retVal != SystemP_SUCCESS)
        {
            DebugP_log ("Error: CANFD transmit data for iteration %d failed\r\n", iterationCount);
            return;
        }

        iterationCount++;
    }

    retVal = CANFD_deleteMsgObject(txMsgObjHandle);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed\r\n");
        return;
    }

    retVal = CANFD_deleteMsgObject(rxMsgObjHandle);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Rx message object failed\r\n");
        return;
    }

    if (retVal == SystemP_SUCCESS)
    {
        DebugP_log("[MCAN] Internal loopback testing for %d iterations Passed\r\n", iterationCount);
        DebugP_log("All tests have passed\r\n");
    }
    else
    {
        DebugP_log("[MCAN] Internal loopback testing for %d iterations Failed\r\n", iterationCount);
        DebugP_log("Some tests have Failed\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}
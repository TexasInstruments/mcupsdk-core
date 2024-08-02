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
#include <drivers/mcan/v0/canfd.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

/** \brief Number of messages sent */
#define MCAN_APP_TEST_MESSAGE_COUNT         10U
/** \brief Number of messages sent */
#define MCAN_APP_TEST_DATA_SIZE             64U

volatile uint32_t           gTxDoneFlag = 0U;
volatile uint32_t           iterationCount = 0U;
volatile uint32_t           gRxDoneFlag = 0U;
CANFD_MessageObject         txMsgObject;
CANFD_MessageObject         rxMsgObject;
CANFD_Object               *canfdObject;
uint8_t                     txData[MCAN_APP_TEST_MESSAGE_COUNT][MCAN_APP_TEST_DATA_SIZE] = {{0}};
CANFD_DmaRxBuf              rxData[MCAN_APP_TEST_MESSAGE_COUNT] = {{0}};

void canfd_loopback_dma_main(void *args)
{
    CANFD_MsgObjHandle          txMsgObjHandle;
    CANFD_MsgObjHandle          rxMsgObjHandle;
    int32_t                     status = 0;
    CANFD_OptionTLV             optionTLV;
    CANFD_MCANBitTimingParams  *bitTimingParams;
    CANFD_MCANLoopbackCfgParams mcanloopbackParams;
    uint32_t                    i,j;
    CANFD_Config               *config;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    config          = (CANFD_Config*) gCanfdHandle[CONFIG_MCAN0];
    canfdObject     = config->object;
    bitTimingParams = (&config->attrs->CANFDMcanBitTimingParams);

    /* initialize the data buffers */
    for (i = 0; i < MCAN_APP_TEST_MESSAGE_COUNT; i++)
    {
        for (j = 0; j < MCAN_APP_TEST_DATA_SIZE; j++)
        {
            txData[i][j] = i + j;
            rxData[i].data[j] = 0U;
        }
    }
    /* Writeback buffer */
    CacheP_wb(&txData[0U], sizeof(txData), CacheP_TYPE_ALLD);
    CacheP_wb(&rxData[0U], sizeof(rxData), CacheP_TYPE_ALLD);

    DebugP_log("\n\n[MCAN] Loopback DMA mode, application started ...\r\n");

    /* Configure the CAN driver */
    status = CANFD_configBitTime (gCanfdHandle[CONFIG_MCAN0], bitTimingParams);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD Module configure bit time failed \n");
        return;
    }

    optionTLV.type   = CANFD_Option_MCAN_LOOPBACK;
    optionTLV.length = sizeof(CANFD_MCANLoopbackCfgParams);
    optionTLV.value  = (void*) &mcanloopbackParams;

    status = CANFD_setOptions(gCanfdHandle[CONFIG_MCAN0], &optionTLV);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD set option Loopback failed \n");
        return;
    }

    gTxDoneFlag = 0;
    gRxDoneFlag = 0;

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
        DebugP_log ("Error: CANFD create Rx message object failed\n");
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
        DebugP_log ("Error: CANFD create Tx message object failed\n");
        return;
    }
    txMsgObjHandle = &txMsgObject;

    status = CANFD_readDmaConfig(rxMsgObjHandle, rxData, MCAN_APP_TEST_MESSAGE_COUNT);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD read dma config failed\n");
        return;
    }

    status = CANFD_write(txMsgObjHandle, txMsgObject.startMsgId, CANFD_MCANFrameType_FD, MCAN_APP_TEST_MESSAGE_COUNT, &txData[0]);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD write dma config failed\n");
        return;
    }

    while (iterationCount != MCAN_APP_TEST_MESSAGE_COUNT)
    {
        /* wait for last transaction to complete. */
        while (gTxDoneFlag == 0U);
        gTxDoneFlag = 0U;

        while (gRxDoneFlag == 0U);
        gRxDoneFlag = 0U;

        /* Send next data over Tx message object */
        status = CANFD_writeDmaTriggerNext(txMsgObjHandle);
        if (status != SystemP_SUCCESS)
        {
            DebugP_log ("Error: CANFD transmit next data for iteration %d failed \n", iterationCount);
            return;
        }

        iterationCount++;
    }

    status = CANFD_deleteMsgObject(txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed \n");
        return;
    }

    status = CANFD_deleteMsgObject(rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Rx message object failed \n");
        return;
    }

    if (status == SystemP_SUCCESS)
    {
        DebugP_log("[MCAN] Internal loopback testing for %d iterations Passed\n", iterationCount);
        DebugP_log(" All tests have passed.\n");
    }
    else
    {
        DebugP_log("[MCAN] Internal loopback testing for %d iterations Failed\n", iterationCount);
        DebugP_log(" Some tests have failed.\n");
    }

    /* Open drivers to open the UART driver for console */
    Drivers_close();
    Board_driversClose();

    return;
}

void CANFD_dmaTxCompletionCallback(CANFD_MessageObject* ptrCanMsgObj, void *data, uint32_t completionType)
{
    if ((ptrCanMsgObj == &txMsgObject) && (completionType == CANFD_DMA_TX_COMPLETION_FINAL))
    {
        SemaphoreP_post(&ptrCanMsgObj->canfdHandle->object->writeTransferSemObj);
    }

    if ((ptrCanMsgObj == &txMsgObject) && (completionType == CANFD_DMA_TX_COMPLETION_INTERMEDIATE))
    {
        gTxDoneFlag = 1U;
    }

    return;
}

void CANFD_dmaRxCompletionCallback(CANFD_MessageObject* ptrCanMsgObj, void *data, uint32_t completionType)
{
    if ((ptrCanMsgObj == &txMsgObject) && (completionType == CANFD_DMA_RX_COMPLETION_FINAL))
    {
        SemaphoreP_post(&ptrCanMsgObj->canfdHandle->object->readTransferSemObj);
    }

    if ((ptrCanMsgObj == &txMsgObject) && (completionType == CANFD_DMA_RX_COMPLETION_INTERMEDIATE))
    {
        gRxDoneFlag = 1U;
    }

    return;
}
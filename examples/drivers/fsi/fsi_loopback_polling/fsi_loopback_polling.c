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

#include <string.h>
#include <stdint.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/fsi.h>
#include <drivers/pinmux.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example performs FSI TX to FSI RX internal loopback in polled mode.
 * The application configures an instance of FSI TX and FSI RX module with below configuration
 *
 * - Single lane
 * - TX clock at 50 MHz
 * - 16 words per frame (transfer)
 *
 * With above configuration, the application transfers 100 frames of data from FSI TX,
 * waits for data to be received by FSI RX and then compares the data.
 *
 * Once the transfer it completes, it compares the source and destination buffers for any data mismatch.
 */

/* FSI TXCLK - 50 MHz */
#define FSI_APP_TXCLK_FREQ              (50 * 1000 * 1000)
/* FSI module input clock - 500 MHz */
#define FSI_APP_CLK_FREQ                (CONFIG_FSI_TX0_CLK)
/* FSI TX prescaler value for TXCLKIN of 100 MHz. / 2 is provided as TXCLK = TXCLKIN/2 */
#define FSI_APP_TX_PRESCALER_VAL        (FSI_APP_CLK_FREQ / FSI_APP_TXCLK_FREQ / 2U)

#define FSI_APP_LOOP_COUNT              (100U)
/* User data to be sent with Data frame */
#define FSI_APP_TX_USER_DATA            (0x07U)
/* Configuring Frame - can be between 1-16U */
#define FSI_APP_FRAME_DATA_WORD_SIZE    (16U)
/* 0x0U for 1 lane and 0x1U for two lane */
#define FSI_APP_N_LANES                 (0x0U)
#define FSI_APP_TX_DATA_FRAME_TAG       (0x1U)

/* Index of FSI TX/RX buffer, gBudIdx + FSI_APP_FRAME_DATA_WORD_SIZE should be <= 16 */
uint16_t gRxBufData[FSI_MAX_VALUE_BUF_PTR_OFF + 1U];
uint16_t gTxBufData[FSI_MAX_VALUE_BUF_PTR_OFF + 1U];

static int32_t Fsi_appTxConfig(uint32_t txBaseAddr);
static int32_t Fsi_appRxConfig(uint32_t rxBaseAddr);

static int32_t Fsi_appCompareData(uint16_t *txBufPtr, uint16_t *rxBufPtr);

void *fsi_loopback_main(void *args)
{
    int32_t     status;
    uint32_t    rxBaseAddr, txBaseAddr;
    uint16_t    dataSize;
    uint32_t    loopCnt;
    uint16_t    bufIdx;
    uint16_t    txEvtSts, rxEvtSts;

    /* Test parameters */
    rxBaseAddr = CONFIG_FSI_RX0_BASE_ADDR;
    txBaseAddr = CONFIG_FSI_TX0_BASE_ADDR;
    dataSize   = FSI_APP_FRAME_DATA_WORD_SIZE;
    loopCnt    = FSI_APP_LOOP_COUNT;
    bufIdx     = 0U;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[FSI] Loopback Polling application started ...\r\n");

    /* FSI configuration */
    status  = Fsi_appTxConfig(txBaseAddr);
    status += Fsi_appRxConfig(rxBaseAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Enable loopback */
    status = FSI_enableRxInternalLoopback(rxBaseAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Send Flush Sequence to sync, after every rx soft reset */
    status = FSI_executeTxFlushSequence(txBaseAddr, FSI_APP_TX_PRESCALER_VAL);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Start transfer */
    while(loopCnt--)
    {
        /* Memset TX buffer with new data for every loop */
        for(uint32_t i = 0; i < dataSize; i++)
        {
            gTxBufData[i] = loopCnt + i;
            gRxBufData[i] = 0U;
        }

        /* Transmit data */
        status = FSI_setTxBufferPtr(txBaseAddr, bufIdx);
        status += FSI_writeTxBuffer(txBaseAddr, gTxBufData, dataSize, bufIdx);
        status += FSI_startTxTransmit(txBaseAddr);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Wait for TX completion */
        while(1)
        {
            FSI_getTxEventStatus(txBaseAddr, &txEvtSts);
            if(txEvtSts & FSI_TX_EVT_FRAME_DONE)
            {
                FSI_clearTxEvents(txBaseAddr, FSI_TX_EVT_FRAME_DONE);
                break;
            }
        }
        /* Wait for RX completion */
        while(1)
        {
            FSI_getRxEventStatus(rxBaseAddr, &rxEvtSts);
            if(rxEvtSts & FSI_RX_EVT_FRAME_DONE)
            {
                FSI_clearRxEvents(rxBaseAddr, FSI_RX_EVT_FRAME_DONE);
                break;
            }
        }

        /* Read data */
        status = FSI_readRxBuffer(rxBaseAddr, gRxBufData, dataSize, bufIdx);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Compare data */
        status = Fsi_appCompareData(gTxBufData, gRxBufData);
        DebugP_assert(status == SystemP_SUCCESS);
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("[FSI] %d frames successfully received!!!\r\n", FSI_APP_LOOP_COUNT);
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return NULL;
}

static int32_t Fsi_appTxConfig(uint32_t txBaseAddr)
{
    int32_t     status;

    /* TX init and reset */
    status = FSI_performTxInitialization(txBaseAddr, FSI_APP_TX_PRESCALER_VAL);
    status += FSI_resetTxModule(txBaseAddr, FSI_TX_MASTER_CORE_RESET);
    FSI_clearTxModuleReset(txBaseAddr, FSI_TX_MASTER_CORE_RESET);

    /* Setting for requested transfer params */
    status += FSI_setTxSoftwareFrameSize(txBaseAddr, FSI_APP_FRAME_DATA_WORD_SIZE);
    status += FSI_setTxDataWidth(txBaseAddr, FSI_APP_N_LANES);

    /* Setting frame config */
    status += FSI_setTxUserDefinedData(txBaseAddr, FSI_APP_TX_USER_DATA);
    status += FSI_setTxFrameTag(txBaseAddr, FSI_APP_TX_DATA_FRAME_TAG);
    status += FSI_setTxFrameType(txBaseAddr, FSI_FRAME_TYPE_NWORD_DATA);

    return status;
}

static int32_t Fsi_appRxConfig(uint32_t rxBaseAddr)
{
    int32_t     status;

    /* RX init and reset */
    status  = FSI_performRxInitialization(rxBaseAddr);
    status += FSI_resetRxModule(rxBaseAddr, FSI_RX_MASTER_CORE_RESET);
    FSI_clearRxModuleReset(rxBaseAddr, FSI_RX_MASTER_CORE_RESET);

    /* Setting for requested transfer params */
    status += FSI_setRxSoftwareFrameSize(rxBaseAddr, FSI_APP_FRAME_DATA_WORD_SIZE);
    status += FSI_setRxDataWidth(rxBaseAddr, FSI_APP_N_LANES);
    status += FSI_setRxBufferPtr(rxBaseAddr, 0U);

    return status;
}

static int32_t Fsi_appCompareData(uint16_t *txBufPtr, uint16_t *rxBufPtr)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    i;

    for(i = 0; i < FSI_APP_FRAME_DATA_WORD_SIZE; i++)
    {
        if(*rxBufPtr++ != *txBufPtr++)
        {
            status = SystemP_FAILURE;
            break;
        }
    }

    return status;
}

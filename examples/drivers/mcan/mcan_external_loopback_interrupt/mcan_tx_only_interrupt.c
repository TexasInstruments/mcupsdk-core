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

/* This example is a multi core application which demonstrates the CAN message
 * communication between the MCAN instances.
 *
 * Instance MCAN1 is designated as 'TX'
 * and the other instance MCAN0 is designated as 'RX'.
 * These 2 instances are connected externally.
 *
 * The MCAN1 instance initiates the transmission by sending a message.
 * The MCAN0 instance receives the same message.
 *
 * Message is transmitted with the following configuration.
 *
 * CAN FD Message Format.
 * Message ID Type is Standard, Message Id is defined as APP_MCAN_STD_ID.
 * MCAN is configured in Interrupt Mode.
 * MCAN Interrupt Line Number 0.
 * Arbitration Bit Rate 1Mbps.
 * Data Bit Rate 5Mbps.
 * Buffer mode is used for both TX/RX to send/receive message in message RAM.
 *
 * This example runs for APP_MCAN_MSG_LOOP_COUNT iterations and in each 
 * iteration the received message id and the data is compared with the transmitted
 * one. After APP_MCAN_MSG_LOOP_COUNT iterations the example is completed.
 *
 * This is a example CAN communication, user can configure different message
 * formats as needed for their applications.
 *
 */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/mcan.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define APP_MCAN_BASE_ADDR                       (CONFIG_MCAN0_BASE_ADDR)
#define APP_MCAN_INTR_NUM                        (CONFIG_MCAN0_INTR)
#define APP_MCAN_MSG_LOOP_COUNT                  (10U)

/* Allocate Message RAM memory section to filter elements, buffers, FIFO */
/* Maximum STD Filter Element can be configured is 128 */
#define APP_MCAN_STD_ID_FILTER_CNT               (0U)
/* Maximum EXT Filter Element can be configured is 64 */
#define APP_MCAN_EXT_ID_FILTER_CNT               (0U)
/* Maximum TX Buffer + TX FIFO, combined can be configured is 32 */
#define APP_MCAN_TX_BUFF_CNT                     (1U)
#define APP_MCAN_TX_FIFO_CNT                     (0U)
/* Maximum TX Event FIFO can be configured is 32 */
#define APP_MCAN_TX_EVENT_FIFO_CNT               (0U)
/* Maximum RX FIFO 0 can be configured is 64 */
#define APP_MCAN_FIFO_0_CNT                      (0U)
/* Maximum RX FIFO 1 can be configured is 64 and
 * rest of the memory is allocated to RX buffer which is again of max size 64 */
#define APP_MCAN_FIFO_1_CNT                      (0U)

/* Standard Id configured in this app */
#define APP_MCAN_STD_ID                          (0xC0U)
#define APP_MCAN_STD_ID_MASK                     (0x7FFU)
#define APP_MCAN_STD_ID_SHIFT                    (18U)

#define APP_MCAN_EXT_ID_MASK                     (0x1FFFFFFFU)

/* In the CAN FD format, the Data length coding differs from the standard CAN.
 * In case of standard CAN it is 8 bytes */
static const uint8_t gMcanTxDataSize[16U] = {0U,  1U,  2U,  3U,
                                           4U,  5U,  6U,  7U,
                                           8U,  12U, 16U, 20U,
                                           24U, 32U, 48U, 64U};

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gMcanTxDoneSem;
static HwiP_Object       gMcanTxHwiObject;
static uint32_t          gMcanRxBaseAddr;

/* Static Function Declarations */
static void    App_mcanIntrISR(void *arg);
static void    App_mcanConfig(void);
static void    App_mcanInitMsgRamConfigParams(
               MCAN_MsgRAMConfigParams *msgRAMConfigParams);
static void    App_mcanEnableIntr(void);
static void    App_mcanConfigTxMsg(MCAN_TxBufElement *txMsg);

void mcan_loopback_tx_interrupt_main(void *args)
{
    int32_t                 status = SystemP_SUCCESS;
    HwiP_Params             hwiPrms;
    MCAN_TxBufElement       txMsg;
    MCAN_ProtocolStatus     protStatus;
    uint32_t                i, bufNum = 0U;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Wait for mcan rx application to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("[MCAN] TX Application, Interrupt mode started ...\r\n");

    /* Construct Tx Semaphore objects */
    status = SemaphoreP_constructBinary(&gMcanTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = APP_MCAN_INTR_NUM;
    hwiPrms.callback    = &App_mcanIntrISR;
    status              = HwiP_construct(&gMcanTxHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Assign MCAN instance address */
    gMcanRxBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(APP_MCAN_BASE_ADDR);

    /* Configure MCAN module, Enable LoopBack Mode */
    App_mcanConfig();

    /* Enable Interrupts */
    App_mcanEnableIntr();

    /* Transmit And Receive Message */
    for (i = 0U; i < APP_MCAN_MSG_LOOP_COUNT; i++)
    {

        /* Wait for mcan rx application to be ready */
        IpcNotify_syncAll(SystemP_WAIT_FOREVER);

        /* Configure Tx Msg to transmit */
        App_mcanConfigTxMsg(&txMsg);

        /* Select buffer number, 32 buffers available */
        bufNum = 0U;
        /* Enable Transmission interrupt for the selected buf num,
         * If FIFO is used, then need to send FIFO start index until FIFO count */
        status = MCAN_txBufTransIntrEnable(gMcanRxBaseAddr, bufNum, (uint32_t)TRUE);
        DebugP_assert(status == CSL_PASS);

        /* Write message to Msg RAM */
        MCAN_writeMsgRam(gMcanRxBaseAddr, MCAN_MEM_TYPE_BUF, bufNum, &txMsg);

        /* Add request for transmission, This function will trigger transmission */
        status = MCAN_txBufAddReq(gMcanRxBaseAddr, bufNum);
        DebugP_assert(status == CSL_PASS);

        /* Wait for Tx completion */
        SemaphoreP_pend(&gMcanTxDoneSem, SystemP_WAIT_FOREVER);

        MCAN_getProtocolStatus(gMcanRxBaseAddr, &protStatus);
        /* Checking for Tx Errors */
        if (((MCAN_ERR_CODE_NO_ERROR != protStatus.lastErrCode) ||
             (MCAN_ERR_CODE_NO_CHANGE != protStatus.lastErrCode)) &&
            ((MCAN_ERR_CODE_NO_ERROR != protStatus.dlec) ||
             (MCAN_ERR_CODE_NO_CHANGE != protStatus.dlec)) &&
            (0U != protStatus.pxe))
        {
             DebugP_assert(FALSE);
        }
    }

    /* De-Construct Tx Semaphore objects */
    HwiP_destruct(&gMcanTxHwiObject);
    SemaphoreP_destruct(&gMcanTxDoneSem);

    /* Wait for mcan tx application to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("[MCAN]TX test passed!!\r\n");

    Board_driversClose();
    Drivers_close();

    return;
}

static void App_mcanConfig(void)
{
    MCAN_InitParams            initParams = {0U};
    MCAN_ConfigParams          configParams = {0U};
    MCAN_MsgRAMConfigParams    msgRAMConfigParams = {0U};
    MCAN_BitTimingParams       bitTimes = {0U};

    /* Initialize MCAN module initParams */
    MCAN_initOperModeParams(&initParams);
    /* CAN FD Mode and Bit Rate Switch Enabled */
    initParams.fdMode          = TRUE;
    initParams.brsEnable       = TRUE;

    /* Initialize MCAN module Global Filter Params */
    MCAN_initGlobalFilterConfigParams(&configParams);

    /* Initialize MCAN module Bit Time Params */
    /* Configuring default 1Mbps and 5Mbps as nominal and data bit-rate resp */
    MCAN_initSetBitTimeParams(&bitTimes);

    /* Initialize MCAN module Message Ram Params */
    App_mcanInitMsgRamConfigParams(&msgRAMConfigParams);

    /* wait for memory initialization to happen */
    while (FALSE == MCAN_isMemInitDone(gMcanRxBaseAddr))
    {}

    /* Put MCAN in SW initialization mode */
    MCAN_setOpMode(gMcanRxBaseAddr, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(gMcanRxBaseAddr))
    {}

    /* Initialize MCAN module */
    MCAN_init(gMcanRxBaseAddr, &initParams);
    /* Configure MCAN module Gloabal Filter */
    MCAN_config(gMcanRxBaseAddr, &configParams);
    /* Configure Bit timings */
    MCAN_setBitTime(gMcanRxBaseAddr, &bitTimes);
    /* Configure Message RAM Sections */
    MCAN_msgRAMConfig(gMcanRxBaseAddr, &msgRAMConfigParams);
    /* Set Extended ID Mask */
    MCAN_setExtIDAndMask(gMcanRxBaseAddr, APP_MCAN_EXT_ID_MASK);

    /* Take MCAN out of the SW initialization mode */
    MCAN_setOpMode(gMcanRxBaseAddr, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanRxBaseAddr))
    {}

    return;
}

static void App_mcanConfigTxMsg(MCAN_TxBufElement *txMsg)
{
    uint32_t i;

    /* Initialize message to transmit */
    MCAN_initTxBufElement(txMsg);
    /* Standard message identifier 11 bit, stored into ID[28-18] */
    txMsg->id  = ((APP_MCAN_STD_ID & MCAN_STD_ID_MASK) << MCAN_STD_ID_SHIFT);
    txMsg->dlc = MCAN_DATA_SIZE_64BYTES; /* Payload size is 64 bytes */
    txMsg->fdf = TRUE; /* CAN FD Frame Format */
    txMsg->xtd = FALSE; /* Extended id not configured */
    for (i = 0U; i < gMcanTxDataSize[MCAN_DATA_SIZE_64BYTES]; i++)
    {
        txMsg->data[i] = i;
    }

    return;
}

static void App_mcanEnableIntr(void)
{
    MCAN_enableIntr(gMcanRxBaseAddr, MCAN_INTR_MASK_ALL, (uint32_t)TRUE);
    MCAN_enableIntr(gMcanRxBaseAddr,
                    MCAN_INTR_SRC_RES_ADDR_ACCESS, (uint32_t)FALSE);
    /* Select Interrupt Line 0 */
    MCAN_selectIntrLine(gMcanRxBaseAddr, MCAN_INTR_MASK_ALL, MCAN_INTR_LINE_NUM_0);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanRxBaseAddr, MCAN_INTR_LINE_NUM_0, (uint32_t)TRUE);

    return;
}

static void App_mcanInitMsgRamConfigParams(MCAN_MsgRAMConfigParams
                                           *msgRAMConfigParams)
{
    int32_t status;

    MCAN_initMsgRamConfigParams(msgRAMConfigParams);

    /* Configure the user required msg ram params */
    msgRAMConfigParams->lss = APP_MCAN_STD_ID_FILTER_CNT;
    msgRAMConfigParams->lse = APP_MCAN_EXT_ID_FILTER_CNT;
    msgRAMConfigParams->txBufCnt = APP_MCAN_TX_BUFF_CNT;
    msgRAMConfigParams->txFIFOCnt = APP_MCAN_TX_FIFO_CNT;
    /* Buffer/FIFO mode is selected */
    msgRAMConfigParams->txBufMode = MCAN_TX_MEM_TYPE_BUF;
    msgRAMConfigParams->txEventFIFOCnt = APP_MCAN_TX_EVENT_FIFO_CNT;
    msgRAMConfigParams->rxFIFO0Cnt = APP_MCAN_FIFO_0_CNT;
    msgRAMConfigParams->rxFIFO1Cnt = APP_MCAN_FIFO_1_CNT;
    /* FIFO blocking mode is selected */
    msgRAMConfigParams->rxFIFO0OpMode = MCAN_RX_FIFO_OPERATION_MODE_BLOCKING;
    msgRAMConfigParams->rxFIFO1OpMode = MCAN_RX_FIFO_OPERATION_MODE_BLOCKING;

    status = MCAN_calcMsgRamParamsStartAddr(msgRAMConfigParams);
    DebugP_assert(status == CSL_PASS);

    return;
}

static void App_mcanIntrISR(void *arg)
{
    uint32_t intrStatus;

    intrStatus = MCAN_getIntrStatus(gMcanRxBaseAddr);
    MCAN_clearIntrStatus(gMcanRxBaseAddr, intrStatus);

    if (MCAN_INTR_SRC_TRANS_COMPLETE ==
        (intrStatus & MCAN_INTR_SRC_TRANS_COMPLETE))
    {
        SemaphoreP_post(&gMcanTxDoneSem);
    }

    return;
}

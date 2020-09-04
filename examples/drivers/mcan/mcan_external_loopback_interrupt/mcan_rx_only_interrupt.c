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
#include <drivers/mcan.h>
#include "ti_dpl_config.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define APP_MCAN_BASE_ADDR                       (CONFIG_MCAN0_BASE_ADDR)
#define APP_MCAN_INTR_NUM                        (CONFIG_MCAN0_INTR)
#define APP_MCAN_MSG_LOOP_COUNT                  (10U)

/* Allocate Message RAM memory section to filter elements, buffers, FIFO */
/* Maximum STD Filter Element can be configured is 128 */
#define APP_MCAN_STD_ID_FILTER_CNT               (1U)
/* Maximum EXT Filter Element can be configured is 64 */
#define APP_MCAN_EXT_ID_FILTER_CNT               (0U)
/* Maximum TX Buffer + TX FIFO, combined can be configured is 32 */
#define APP_MCAN_TX_BUFF_CNT                     (0U)
#define APP_MCAN_TX_FIFO_CNT                     (0U)
/* Maximum TX Event FIFO can be configured is 32 */
#define APP_MCAN_TX_EVENT_FIFO_CNT               (0U)
/* Maximum RX FIFO 0 can be configured is 64 */
#define APP_MCAN_FIFO_0_CNT                      (0U)
/* Maximum RX FIFO 1 can be configured is 64 and
 * rest of the memory is allocated to RX buffer which is again of max size 64 */
#define APP_MCAN_FIFO_1_CNT                      (0U)

/* Standard Id configured in TX application.
 * This macro defined to compare with the received message id in this application */
#define APP_MCAN_STD_ID                          (0xC0U)
#define APP_MCAN_STD_ID_MASK                     (0x7FFU)
#define APP_MCAN_STD_ID_SHIFT                    (18U)
#define APP_MCAN_EXT_ID_MASK                     (0x1FFFFFFFU)

/* In the CAN FD format, the Data length coding differs from the standard CAN.
 * In case of standard CAN it is 8 bytes */
static const uint8_t gMcanRxDataSize[16U] = {0U,  1U,  2U,  3U,
                                           4U,  5U,  6U,  7U,
                                           8U,  12U, 16U, 20U,
                                           24U, 32U, 48U, 64U};
/* CAN FD Data */
uint8_t gMcanRxRecvdData[MCAN_MAX_PAYLOAD_BYTES];

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gMcanRxDoneSem;
static HwiP_Object       gMcanRxHwiObject;
static uint32_t          gMcanRxBaseAddr;

/* Extern Function Declarations */
void mcan_enableTransceiver(void);

/* Static Function Declarations */
static void    App_mcanIntrISR(void *arg);
static void    App_mcanConfig(void);
static void    App_mcanInitMsgRamConfigParams(
               MCAN_MsgRAMConfigParams *msgRAMConfigParams);
static void    App_mcanEnableIntr(void);
static void    App_mcanCompareMsg(MCAN_RxBufElement *rxMsg);
static void    App_mcanInitStdFilterElemParams(
                                  MCAN_StdMsgIDFilterElement *stdFiltElem,
                                  uint32_t bufNum);
static void    App_mcanInitExpectedRxData(void);

void mcan_loopback_rx_interrupt_main(void *args)
{
    int32_t                 status = SystemP_SUCCESS;
    HwiP_Params             hwiPrms;
    MCAN_RxBufElement       rxMsg;
    MCAN_RxNewDataStatus    newDataStatus;
    MCAN_ErrCntStatus       errCounter;
    uint32_t                i, bufNum, fifoNum, bitPos = 0U;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Enabled transceiver in RX application. No need to do in TX application.
     * If user wants to use only TX application then this function need to be
     * called in TX application also. */
    mcan_enableTransceiver();

    /* Wait for mcan tx application to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("[MCAN] RX Application, Interrupt mode started ...\r\n");

    /* Construct Rx Semaphore objects */
    status = SemaphoreP_constructBinary(&gMcanRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = APP_MCAN_INTR_NUM;
    hwiPrms.callback    = &App_mcanIntrISR;
    status              = HwiP_construct(&gMcanRxHwiObject, &hwiPrms);
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

        /* Wait for mcan tx application to be ready */
        IpcNotify_syncAll(SystemP_WAIT_FOREVER);

        /* Wait for Rx completion */
        SemaphoreP_pend(&gMcanRxDoneSem, SystemP_WAIT_FOREVER);

        /* Checking for Rx Errors */
        MCAN_getErrCounters(gMcanRxBaseAddr, &errCounter);
        DebugP_assert((0U == errCounter.recErrCnt) &&
                      (0U == errCounter.canErrLogCnt));

        /* Get the new data staus, indicates buffer num which received message */
        MCAN_getNewDataStatus(gMcanRxBaseAddr, &newDataStatus);
        MCAN_clearNewDataStatus(gMcanRxBaseAddr, &newDataStatus);

        /* Select buffer and fifo number, Buffer is used in this app */
        bufNum = 0U;
        fifoNum = MCAN_RX_FIFO_NUM_0;

        bitPos = (1U << bufNum);
        if (bitPos == (newDataStatus.statusLow & bitPos))
        {
            MCAN_readMsgRam(gMcanRxBaseAddr, MCAN_MEM_TYPE_BUF, bufNum, fifoNum, &rxMsg);
        }
        else
        {
            DebugP_assert(FALSE);
        }

        /* Compare Tx/Rx data */
        App_mcanCompareMsg(&rxMsg);
    }

    /* De-Construct Rx Semaphore objects */
    HwiP_destruct(&gMcanRxHwiObject);
    SemaphoreP_destruct(&gMcanRxDoneSem);

    /* Wait for mcan tx application to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    /* We don't close drivers so that the UART driver remains open and flush any
     * pending messages to console */
    /* Drivers_close(); */

    return;
}

static void App_mcanConfig(void)
{
    MCAN_StdMsgIDFilterElement stdFiltElem[APP_MCAN_STD_ID_FILTER_CNT] = {0U};
    MCAN_InitParams            initParams = {0U};
    MCAN_ConfigParams          configParams = {0U};
    MCAN_MsgRAMConfigParams    msgRAMConfigParams = {0U};
    MCAN_BitTimingParams       bitTimes = {0U};
    uint32_t                   i;

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

    /* This data initialized here should be same as what TX application sends */
    App_mcanInitExpectedRxData();

    /* Initialize Filter element to receive msg, should be same as tx msg id */
    for (i = 0U; i < APP_MCAN_STD_ID_FILTER_CNT; i++)
    {
        App_mcanInitStdFilterElemParams(&stdFiltElem[i], i);
    }

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

    /* Configure Standard ID filter element */
    for (i = 0U; i < APP_MCAN_STD_ID_FILTER_CNT; i++)
    {
        MCAN_addStdMsgIDFilter(gMcanRxBaseAddr, i, &stdFiltElem[i]);
    }

    /* Take MCAN out of the SW initialization mode */
    MCAN_setOpMode(gMcanRxBaseAddr, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanRxBaseAddr))
    {}

    return;
}

static void App_mcanInitStdFilterElemParams(MCAN_StdMsgIDFilterElement *stdFiltElem,
                                            uint32_t bufNum)
{
    /* sfid1 defines the ID of the standard message to be stored. */
    stdFiltElem->sfid1 = APP_MCAN_STD_ID;
    /* As buffer mode is selected, sfid2 should be bufNum[0 - 63] */
    stdFiltElem->sfid2 = bufNum;
    /* Store message in buffer */
    stdFiltElem->sfec  = MCAN_STD_FILT_ELEM_BUFFER;
    /* Below configuration is ignored if message is stored in buffer */
    stdFiltElem->sft   = MCAN_STD_FILT_TYPE_RANGE;

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

static void App_mcanCompareMsg(MCAN_RxBufElement *rxMsg)
{
    uint32_t i;

    if (APP_MCAN_STD_ID == ((rxMsg->id >> APP_MCAN_STD_ID_SHIFT) & APP_MCAN_STD_ID_MASK))
    {
        for (i = 0U; i < gMcanRxDataSize[MCAN_DATA_SIZE_64BYTES]; i++)
        {
            if (gMcanRxRecvdData[i] != rxMsg->data[i])
            {
                DebugP_logError("Data mismatch !!!\r\n");
                DebugP_assert(FALSE);
            }
        }
    }
    else
    {
        DebugP_logError("Message ID mismatch !!!\r\n");
        DebugP_assert(FALSE);
    }

    return;
}

/* Data initializated here should be same as TX application */
static void App_mcanInitExpectedRxData(void)
{
    uint32_t i;

    for (i = 0U; i < gMcanRxDataSize[MCAN_DATA_SIZE_64BYTES]; i++)
    {
        gMcanRxRecvdData[i] = i;
    }
}

static void App_mcanIntrISR(void *arg)
{
    uint32_t intrStatus;

    intrStatus = MCAN_getIntrStatus(gMcanRxBaseAddr);
    MCAN_clearIntrStatus(gMcanRxBaseAddr, intrStatus);

    /* If FIFO0/FIFO1 is used, then MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG macro
     * needs to be replaced by MCAN_INTR_SRC_RX_FIFO0_NEW_MSG/
     * MCAN_INTR_SRC_RX_FIFO1_NEW_MSG respectively */
    if (MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG ==
        (intrStatus & MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG))
    {
        SemaphoreP_post(&gMcanRxDoneSem);
    }

    return;
}

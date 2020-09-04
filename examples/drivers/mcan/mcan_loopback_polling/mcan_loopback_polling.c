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

/* This example demonstrates the CAN message transmission and reception in
 * digital loop back mode with the following configuration.
 *
 * Classic CAN Message Format.
 * Message ID Type is Extended, Msg Id 0xD0, 0xD1, 0xD2, 0xD3, 0xD4.
 * MCAN is configured in Polling Mode.
 * Arbitration Bit Rate 1Mbps.
 * Data Bit Rate 5Mbps.
 * Classic Bit Mask Acceptance Filtering type is used.
 * FIFO mode is used for Tx and RX to store message in message RAM.
 *
 * 5 Messages are transmitted and received back internally using internal loopback
 * mode. When all the 5 messages received with the id and the data matches
 * with the transmitted one, then the example is completed.
 */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
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
#define APP_MCAN_EXT_ID_FILTER_CNT               (5U)
/* Maximum TX Buffer + TX FIFO, combined can be configured is 32 */
#define APP_MCAN_TX_BUFF_CNT                     (0U)
#define APP_MCAN_TX_FIFO_CNT                     (5U)
/* Maximum TX Event FIFO can be configured is 32 */
#define APP_MCAN_TX_EVENT_FIFO_CNT               (0U)
/* Maximum RX FIFO 0 can be configured is 64 */
#define APP_MCAN_FIFO_0_CNT                      (5U)
/* Maximum RX FIFO 1 can be configured is 64 and
 * rest of the memory is allocated to RX buffer which is again of max size 64 */
#define APP_MCAN_FIFO_1_CNT                      (0U)

/* Extended Id configured in this app */
#define APP_MCAN_EXT_ID                          (0xD0U)
#define APP_MCAN_EXT_ID_MASK                     (0x1FFFFFFFU)

/* Classic Bit Mask Filter */
#define APP_MCAN_CLASSIC_BIT_MASK                (0xFFFFFFFFU)



/* In the CAN FD format, the Data length coding differs from the standard CAN.
 * In case of standard CAN it is 8 bytes */
static const uint8_t gMcanDataSize[16U] = {0U,  1U,  2U,  3U,
                                           4U,  5U,  6U,  7U,
                                           8U,  12U, 16U, 20U,
                                           24U, 32U, 48U, 64U};

static uint32_t      gMcanBaseAddr;

/* Static Function Declarations */
static void    App_mcanConfig(Bool enableInternalLpbk);
static void    App_mcanInitMsgRamConfigParams(
               MCAN_MsgRAMConfigParams *msgRAMConfigParams);
static void    App_mcanConfigTxMsg(MCAN_TxBufElement *txMsg, uint32_t idx);
static void    App_mcanCompareMsg(MCAN_TxBufElement *txMsg,
                                  MCAN_RxBufElement *rxMsg);
static void    App_mcanInitExtFilterElemParams(
               MCAN_ExtMsgIDFilterElement *extFiltElem, uint32_t bufNum);;

void mcan_loopback_polling_main(void *args)
{
    int32_t                 status = SystemP_SUCCESS;
    MCAN_TxBufElement       txMsg[APP_MCAN_TX_FIFO_CNT];
    MCAN_ProtocolStatus     protStatus;
    MCAN_RxBufElement       rxMsg;
    MCAN_ErrCntStatus       errCounter;
    uint32_t                i, j, fifoStartIdx, bitPos = 0U;
    uint32_t                txStatus, loopCnt, fifoFillLvl;
    MCAN_RxFIFOStatus       fifoStatus;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[MCAN] Loopback Polling mode, application started ...\r\n");

    /* Assign MCAN instance address */
    gMcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(APP_MCAN_BASE_ADDR);

    /* Configure MCAN module, Enable LoopBack Mode */
    App_mcanConfig(TRUE);

    /* Transmit And Receive Message */
    for (i = 0U; i < APP_MCAN_MSG_LOOP_COUNT; i++)
    {
        /* Transmit Message from FIFO */
        for(j = 0U; j < APP_MCAN_TX_FIFO_CNT; j++)
        {
            /* Configure Tx Msg to transmit */
            App_mcanConfigTxMsg(&txMsg[j], j);

            /* Select buffer/FIFO number, 32 available together combined */
            fifoStartIdx = j;
            /* Write message to Msg RAM */
            MCAN_writeMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO, fifoStartIdx, &txMsg[j]);

            /* Add request for transmission, This function will trigger transmission */
            status = MCAN_txBufAddReq(gMcanBaseAddr, fifoStartIdx);
            DebugP_assert(status == CSL_PASS);

            bitPos = (1U << fifoStartIdx);
            /* Poll for Tx completion */
            do
            {
                txStatus = MCAN_getTxBufTransmissionStatus(gMcanBaseAddr);
            }while((txStatus & bitPos) != bitPos);

            MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
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

        /* Poll for Rx completion */
        fifoStatus.num = MCAN_RX_FIFO_NUM_0;
        do
        {
            MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
            fifoFillLvl = fifoStatus.fillLvl;
        }while(fifoFillLvl != APP_MCAN_FIFO_0_CNT);
        for(loopCnt = 0U ; loopCnt < fifoFillLvl ; loopCnt++)
        {
            /* Checking for Rx Errors */
            MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
            DebugP_assert((0U == errCounter.recErrCnt) &&
                          (0U == errCounter.canErrLogCnt));

            MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
            MCAN_readMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO, fifoStatus.getIdx,
                            fifoStatus.num, &rxMsg);
           (void) MCAN_writeRxFIFOAck(gMcanBaseAddr, fifoStatus.num,
                                      fifoStatus.getIdx);
            /* Compare Tx/Rx data */
            App_mcanCompareMsg(&txMsg[loopCnt], &rxMsg);
        }
    }
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();

    return;
}

static void App_mcanConfig(Bool enableInternalLpbk)
{
    MCAN_ExtMsgIDFilterElement extFiltElem[APP_MCAN_EXT_ID_FILTER_CNT] = {0U};
    MCAN_InitParams            initParams = {0U};
    MCAN_ConfigParams          configParams = {0U};
    MCAN_MsgRAMConfigParams    msgRAMConfigParams = {0U};
    MCAN_BitTimingParams       bitTimes = {0U};
    uint32_t                   i;

    /* Initialize MCAN module initParams */
    MCAN_initOperModeParams(&initParams);
    /* CAN FD Mode and Bit Rate Switch Disabled */
    initParams.fdMode          = FALSE;
    initParams.brsEnable       = FALSE;

    /* Initialize MCAN module Global Filter Params */
    MCAN_initGlobalFilterConfigParams(&configParams);

    /* Initialize MCAN module Bit Time Params */
    /* Configuring default 1Mbps and 5Mbps as nominal and data bit-rate resp */
    MCAN_initSetBitTimeParams(&bitTimes);

    /* Initialize MCAN module Message Ram Params */
    App_mcanInitMsgRamConfigParams(&msgRAMConfigParams);

    /* Initialize Filter element to receive msg, should be same as tx msg id */
    for (i = 0U; i < APP_MCAN_EXT_ID_FILTER_CNT; i++)
    {
        App_mcanInitExtFilterElemParams(&extFiltElem[i], i);
    }
    /* wait for memory initialization to happen */
    while (FALSE == MCAN_isMemInitDone(gMcanBaseAddr))
    {}

    /* Put MCAN in SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(gMcanBaseAddr))
    {}

    /* Initialize MCAN module */
    MCAN_init(gMcanBaseAddr, &initParams);
    /* Configure MCAN module Gloabal Filter */
    MCAN_config(gMcanBaseAddr, &configParams);
    /* Configure Bit timings */
    MCAN_setBitTime(gMcanBaseAddr, &bitTimes);
    /* Configure Message RAM Sections */
    MCAN_msgRAMConfig(gMcanBaseAddr, &msgRAMConfigParams);
    /* Set Extended ID Mask */
    MCAN_setExtIDAndMask(gMcanBaseAddr, APP_MCAN_EXT_ID_MASK);

    /* Configure Standard ID filter element */
    for (i = 0U; i < APP_MCAN_EXT_ID_FILTER_CNT; i++)
    {
        MCAN_addExtMsgIDFilter(gMcanBaseAddr, i, &extFiltElem[i]);
    }
    if (TRUE == enableInternalLpbk)
    {
        MCAN_lpbkModeEnable(gMcanBaseAddr, MCAN_LPBK_MODE_INTERNAL, TRUE);
    }

    /* Take MCAN out of the SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanBaseAddr))
    {}

    return;
}

static void App_mcanConfigTxMsg(MCAN_TxBufElement *txMsg, uint32_t idx)
{
    uint32_t i;

    /* Initialize message to transmit */
    MCAN_initTxBufElement(txMsg);
    /* Standard message identifier 11 bit, stored into ID[28-18] */
    txMsg->id  = ((APP_MCAN_EXT_ID + idx) & APP_MCAN_EXT_ID_MASK);
    txMsg->dlc = MCAN_DATA_SIZE_8BYTES; /* Payload size is 64 bytes */
    txMsg->fdf = FALSE; /* It's a Classic CAN Frame Format,  */
    txMsg->xtd = TRUE; /* Extended id configured */
    /* Data Payload is 8bytes */
    for (i = 0U; i < gMcanDataSize[MCAN_DATA_SIZE_8BYTES]; i++)
    {
        txMsg->data[i] = i;
    }

    return;
}

static void App_mcanInitExtFilterElemParams(MCAN_ExtMsgIDFilterElement *extFiltElem,
                                            uint32_t bufNum)
{
    /* efid1 defines the ID of the standard message to be stored.
     * Message id configured is 0xD0 to 0xD4 */
    extFiltElem->efid1 = APP_MCAN_EXT_ID + bufNum;
    /* As fifo mode is selected, efid2 should be mask */
    extFiltElem->efid2 = APP_MCAN_CLASSIC_BIT_MASK;
    /* Store message in buffer */
    extFiltElem->efec  = MCAN_STD_FILT_ELEM_FIFO0;
    /* Below configuration is ignored if message is stored in buffer */
    extFiltElem->eft   = MCAN_STD_FILT_TYPE_CLASSIC;

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

static void App_mcanCompareMsg(MCAN_TxBufElement *txMsg,
                               MCAN_RxBufElement *rxMsg)
{
    uint32_t i;

    if ((txMsg->id & APP_MCAN_EXT_ID_MASK) ==
            (rxMsg->id & APP_MCAN_EXT_ID_MASK))
    {
        for (i = 0U; i < gMcanDataSize[MCAN_DATA_SIZE_8BYTES]; i++)
        {
            if (txMsg->data[i] != rxMsg->data[i])
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

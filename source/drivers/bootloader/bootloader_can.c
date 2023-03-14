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

#include <drivers/bootloader/bootloader_can.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <stdio.h>
#include <string.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/mcan.h>
#include <drivers/soc.h>

uint32_t gMcanBaseAddr;

MCAN_TxBufElement gtxMsg;
MCAN_RxBufElement grxMsg;

#define ENABLE_CANFD_SUPPORT /* Macro for CAN FD Support, undefine this to use as Std CAN */

void mcanConfigTxMsg(MCAN_TxBufElement *txMsg, uint32_t idx)
{
    /* Initialize message to transmit */
    MCAN_initTxBufElement(txMsg);
    /* Standard message identifier 11 bit, stored into ID[28-18] */
    txMsg->id  = ((APP_MCAN_EXT_ID + idx) & APP_MCAN_EXT_ID_MASK);
    txMsg->dlc = APP_MCAN_BUFFER_SIZE;
    #ifdef ENABLE_CANFD_SUPPORT
    txMsg->fdf = TRUE; /* It's a CANFD Frame Format  */
    #else
    txMsg->fdf = FALSE; /* It's not a CANFD Frame Format  */
    #endif
    txMsg->xtd = TRUE; /* Extended id configured */

    return;
}

char gCANMsgsList[][9U] = {
    "PING",
    "PONG",
    "MSGACK",
    "LSTMSG",
    "LSTMSGAK",
    "LSTMSGCF",
    "RUN",
    "RESET"
};

int32_t Bootloader_CANTransmitResp(uint8_t *src)
{
    int32_t status = SystemP_SUCCESS;
    MCAN_ProtocolStatus     protStatus;
    uint32_t                txStatus;

    /* Delay of 5 ms to sync-up with the device */
    ClockP_usleep(5000);

    /* Configure Tx Msg to transmit */
    mcanConfigTxMsg(&gtxMsg, 0U);

    /* Data Payload is used as 16 bytes for the response */
    memcpy(&gtxMsg.data[0], src, 16U);

    /* Write message to Msg RAM */
    MCAN_writeMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO, 0U, &gtxMsg);

    /* Add request for transmission, This function will trigger transmission */
    status = MCAN_txBufAddReq(gMcanBaseAddr, 0U);

    /* Poll for Tx completion */
    do
    {
        txStatus = MCAN_getTxBufTransmissionStatus(gMcanBaseAddr);
    }while((txStatus & (1U << 0U)) != (1U << 0U));

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

    return status;
}

int32_t Bootloader_CANReceiveFile(uint32_t *fileSize, uint8_t *dstBuf, uint32_t *run)
{
    int32_t status = SystemP_SUCCESS;
    bool done = FALSE;
    uint32_t                txStatus, fifoFillLvl;
    MCAN_RxFIFOStatus       rxFifoStatus;
    MCAN_ErrCntStatus       errCounter;
    MCAN_ProtocolStatus     protStatus;
    uint8_t seq_No = 0;

        /**********************  Ping from Script to am263x-cc Board  ***************************/
    /* Poll for Rx completion */
    rxFifoStatus.num = MCAN_RX_FIFO_NUM_0;
    do
    {
        MCAN_getRxFIFOStatus(gMcanBaseAddr, &rxFifoStatus);
        fifoFillLvl = rxFifoStatus.fillLvl;
    }while(fifoFillLvl != APP_MCAN_FIFO_0_CNT);

    /* Checking for Rx Errors */
    MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
    DebugP_assert((0U == errCounter.recErrCnt) &&
                    (0U == errCounter.canErrLogCnt));

    MCAN_getRxFIFOStatus(gMcanBaseAddr, &rxFifoStatus);

    MCAN_readMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO, rxFifoStatus.getIdx,
                    rxFifoStatus.num, &grxMsg);

    (void) MCAN_writeRxFIFOAck(gMcanBaseAddr, rxFifoStatus.num,
                                rxFifoStatus.getIdx);

    DebugP_assert(!strcmp((const char *)&grxMsg.data[0], &gCANMsgsList[0][0]));

    /**********************  Pong from am263x-cc Board to Script ***************************/
    /* Configure Tx Msg to transmit */
    mcanConfigTxMsg(&gtxMsg, 0U);

    /* Data Payload */
    memcpy(&gtxMsg.data[0], &gCANMsgsList[1][0], sizeof(gCANMsgsList[1]));

    /* Write message to Msg RAM */
    MCAN_writeMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO, 0U, &gtxMsg);

    /* Add request for transmission, This function will trigger transmission */
    status = MCAN_txBufAddReq(gMcanBaseAddr, 0U);

    /* Poll for Tx completion */
    do
    {
        txStatus = MCAN_getTxBufTransmissionStatus(gMcanBaseAddr);
    }while((txStatus & (1U << 0U)) != (1U << 0U));

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

    /* Data Payload */
    memcpy(&gtxMsg.data[0], &gCANMsgsList[2][0], sizeof(gCANMsgsList[2]));

    /* Write message to Msg RAM */
    MCAN_writeMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO, 0U, &gtxMsg);

    while(done == FALSE)
    {
        /* Poll for Rx completion */
        rxFifoStatus.num = MCAN_RX_FIFO_NUM_0;

        do
        {
            MCAN_getRxFIFOStatus(gMcanBaseAddr, &rxFifoStatus);
            fifoFillLvl = rxFifoStatus.fillLvl;
        }while(fifoFillLvl != APP_MCAN_FIFO_0_CNT);

        /* Checking for Rx Errors */
        MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
        DebugP_assert((0U == errCounter.recErrCnt) &&
                        (0U == errCounter.canErrLogCnt));

        MCAN_getRxFIFOStatus(gMcanBaseAddr, &rxFifoStatus);

        MCAN_readMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO, rxFifoStatus.getIdx,
                        rxFifoStatus.num, &grxMsg);

        (void) MCAN_writeRxFIFOAck(gMcanBaseAddr, rxFifoStatus.num,
                                    rxFifoStatus.getIdx);

        if(!strcmp((const char *)&grxMsg.data[0], &gCANMsgsList[3][0]))
        {
            done = TRUE;

            /* Data Payload */
            memcpy(&gtxMsg.data[0], &gCANMsgsList[4][0], sizeof(gCANMsgsList[4]));

            /* Write message to Msg RAM */
            MCAN_writeMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO, 0U, &gtxMsg);
        }

        if((grxMsg.data[0] == seq_No)||(done == TRUE))
        {
            memcpy(&dstBuf[*fileSize], &grxMsg.data[1], FILE_PKT_SIZE);
            *fileSize += FILE_PKT_SIZE;

            /* Add request for transmission, This function will trigger transmission */
            status = MCAN_txBufAddReq(gMcanBaseAddr, 0U);

            /* Poll for Tx completion */
            do
            {
                txStatus = MCAN_getTxBufTransmissionStatus(gMcanBaseAddr);
            }while((txStatus & (1U << 0U)) != (1U << 0U));

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

            seq_No++;
        }

        if (done == TRUE)
        {
            /* Poll for Rx completion */
            rxFifoStatus.num = MCAN_RX_FIFO_NUM_0;

            do
            {
                MCAN_getRxFIFOStatus(gMcanBaseAddr, &rxFifoStatus);
                fifoFillLvl = rxFifoStatus.fillLvl;
            }while(fifoFillLvl != APP_MCAN_FIFO_0_CNT);

            /* Checking for Rx Errors */
            MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
            DebugP_assert((0U == errCounter.recErrCnt) &&
                            (0U == errCounter.canErrLogCnt));

            MCAN_getRxFIFOStatus(gMcanBaseAddr, &rxFifoStatus);

            MCAN_readMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO, rxFifoStatus.getIdx,
                            rxFifoStatus.num, &grxMsg);

            (void) MCAN_writeRxFIFOAck(gMcanBaseAddr, rxFifoStatus.num,
                                        rxFifoStatus.getIdx);

            if(!strcmp((const char *)&grxMsg.data[0], &gCANMsgsList[5][0]))
            {
                *fileSize -= FILE_PKT_SIZE;
                status = SystemP_SUCCESS;

                /* Data Payload */
                memcpy(&gtxMsg.data[0], &gCANMsgsList[2][0], sizeof(gCANMsgsList[2]));

                /* Write message to Msg RAM */
                MCAN_writeMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO, 0U, &gtxMsg);

                /* Add request for transmission, This function will trigger transmission */
                status = MCAN_txBufAddReq(gMcanBaseAddr, 0U);

                /* Poll for Tx completion */
                do
                {
                    txStatus = MCAN_getTxBufTransmissionStatus(gMcanBaseAddr);
                }while((txStatus & (1U << 0U)) != (1U << 0U));

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
            else
            {
                done = FALSE;
            }
        }
    }

    /* Poll for RESET or RUN Command */
    rxFifoStatus.num = MCAN_RX_FIFO_NUM_0;
    do
    {
        MCAN_getRxFIFOStatus(gMcanBaseAddr, &rxFifoStatus);
        fifoFillLvl = rxFifoStatus.fillLvl;
    }while(fifoFillLvl != APP_MCAN_FIFO_0_CNT);

    /* Checking for Rx Errors */
    MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
    DebugP_assert((0U == errCounter.recErrCnt) &&
                    (0U == errCounter.canErrLogCnt));

    MCAN_getRxFIFOStatus(gMcanBaseAddr, &rxFifoStatus);

    MCAN_readMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO, rxFifoStatus.getIdx,
                    rxFifoStatus.num, &grxMsg);

    (void) MCAN_writeRxFIFOAck(gMcanBaseAddr, rxFifoStatus.num,
                                rxFifoStatus.getIdx);

    if(!strcmp((const char *)&grxMsg.data[0], &gCANMsgsList[6][0]))
    {
        (*run) = CSL_TRUE;
    }

    /**********************  ACK from am263x-cc Board to Script ***************************/
    /* Configure Tx Msg to transmit */
    mcanConfigTxMsg(&gtxMsg, 0U);

    /* Data Payload */
    memcpy(&gtxMsg.data[0], &gCANMsgsList[2][0], sizeof(gCANMsgsList[2]));

    /* Write message to Msg RAM */
    MCAN_writeMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO, 0U, &gtxMsg);

    /* Add request for transmission, This function will trigger transmission */
    status = MCAN_txBufAddReq(gMcanBaseAddr, 0U);

    /* Poll for Tx completion */
    do
    {
        txStatus = MCAN_getTxBufTransmissionStatus(gMcanBaseAddr);
    }while((txStatus & (1U << 0U)) != (1U << 0U));

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

    return status;
}

void Bootloader_CANInit(uint64_t systemAddr)
{

    gMcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(systemAddr);

    MCAN_InitParams            initParams = {0U};
    MCAN_ConfigParams          configParams = {0U};
    MCAN_MsgRAMConfigParams    msgRAMConfigParams = {0U};
    MCAN_BitTimingParams       bitTimes = {0U};
    MCAN_ExtMsgIDFilterElement extFiltElem = {0U};

    /* Initialize MCAN module initParams */
    MCAN_initOperModeParams(&initParams);

    #ifdef ENABLE_CANFD_SUPPORT
    /* CAN FD Mode and Bit Rate Switch Disabled */
    initParams.fdMode          = TRUE;
    initParams.brsEnable       = TRUE;
    #else
    /* CAN FD Mode and Bit Rate Switch Disabled */
    initParams.fdMode          = FALSE;
    initParams.brsEnable       = FALSE;
    #endif

    /* Initialize MCAN module Global Filter Params */
    MCAN_initGlobalFilterConfigParams(&configParams);

    /* Initialize MCAN module Bit Time Params */
    /* Configuring default 1Mbps and 5Mbps as nominal and data bit-rate resp */
    MCAN_initSetBitTimeParams(&bitTimes);

    /* Initialize MCAN module Message Ram Params */
    MCAN_initMsgRamConfigParams(&msgRAMConfigParams);

    /* Configure the user required msg ram params */
    msgRAMConfigParams.lss = APP_MCAN_STD_ID_FILTER_CNT;
    msgRAMConfigParams.lse = APP_MCAN_EXT_ID_FILTER_CNT;
    msgRAMConfigParams.txBufCnt = APP_MCAN_TX_BUFF_CNT;
    msgRAMConfigParams.txFIFOCnt = APP_MCAN_TX_FIFO_CNT;
    /* Buffer/FIFO mode is selected */
    msgRAMConfigParams.txBufMode = MCAN_TX_MEM_TYPE_BUF;
    msgRAMConfigParams.txEventFIFOCnt = APP_MCAN_TX_EVENT_FIFO_CNT;
    msgRAMConfigParams.rxFIFO0Cnt = APP_MCAN_FIFO_0_CNT;
    msgRAMConfigParams.rxFIFO1Cnt = APP_MCAN_FIFO_1_CNT;
    /* FIFO blocking mode is selected */
    msgRAMConfigParams.rxFIFO0OpMode = MCAN_RX_FIFO_OPERATION_MODE_BLOCKING;
    msgRAMConfigParams.rxFIFO1OpMode = MCAN_RX_FIFO_OPERATION_MODE_BLOCKING;

    MCAN_calcMsgRamParamsStartAddr(&msgRAMConfigParams);

    /* Initialize Filter element to receive msg, should be same as tx msg id */
    /* efid1 defines the ID of the standard message to be stored.
     * Message id configured is 0xC0 */
    extFiltElem.efid1 = APP_MCAN_EXT_ID;
    /* As fifo mode is selected, efid2 should be mask */
    extFiltElem.efid2 = APP_MCAN_CLASSIC_BIT_MASK;
    /* Store message in buffer */
    extFiltElem.efec  = MCAN_STD_FILT_ELEM_FIFO0;
    /* Below configuration is ignored if message is stored in buffer */
    extFiltElem.eft   = MCAN_STD_FILT_TYPE_CLASSIC;

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
    MCAN_addExtMsgIDFilter(gMcanBaseAddr, 0U, &extFiltElem);

    /* Take MCAN out of the SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanBaseAddr))
    {}

    /* Initialize message to transmit */
    MCAN_initTxBufElement(&gtxMsg);
}
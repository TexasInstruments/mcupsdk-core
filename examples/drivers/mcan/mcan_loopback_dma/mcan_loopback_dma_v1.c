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

#include <stdio.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/mcan/v0/mcan.h>
#include <drivers/mcan/v0/dma/edma/canfd_dma_edma.c>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/** \brief Number of messages sent */
#define APP_MCAN_TEST_MESSAGE_COUNT              10U
/* Maximum STD Filter Element can be configured is 128 */
#define APP_MCAN_STD_ID_FILTER_CNT               (1U)
/* Number of messages sent */
#define APP_MCAN_TEST_DATA_SIZE                  (64U)
/* Padding required to align the buffers to cacheline */
#define CACHELINE_PADDING                        (24U)
/* Maximum TX Buffer + TX FIFO, combined can be configured is 32 */
#define APP_MCAN_TX_BUFF_CNT                     (1U)
/* Maximum RX FIFO 0 can be configured is 64 */
#define APP_MCAN_FIFO_0_CNT                      (0U)
/* Maximum RX FIFO 1 can be configured is 64 */
#define APP_MCAN_FIFO_1_CNT                      (0U)
/* Maximum TX Event FIFO can be configured is 32 */
#define APP_MCAN_TX_EVENT_FIFO_CNT               (0U)
#define APP_MCAN_TX_FIFO_CNT                     (0U)
/* Maximum EXT Filter Element can be configured is 64 */
#define APP_MCAN_EXT_ID_FILTER_CNT               (0U)

/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO                   (0U)
/* Standard Id configured in this app */
#define APP_MCAN_STD_ID                          (0xC0U)
#define APP_MCAN_STD_ID_MASK                     (0x7FFU)
#define APP_MCAN_STD_ID_SHIFT                    (18U)

/** \brief Number of messages sent */
#define APP_MCAN_EXT_ID_MASK               (0x1FFFFFFFU)

static uint32_t          gMcanBaseAddr;
static SemaphoreP_Object gMcanTxDoneSem, gMcanRxDoneSem;
uint32_t    dmaTxCh, txTcc, rxTcc, paramTx, dmaRxCh, paramRx;
uint32_t    gEdmaBaseAddr, txRegionId, rxRegionId;
Edma_IntrObject     intrObjTx;
Edma_IntrObject     intrObjRx;
uint32_t gRxNumMsgCount = 0U;
uint32_t gTxNumMsgCount = 0U;

static void App_deleteTxMsgConfig(void);
static void App_deleteRxMsgConfig(void);
static void App_mcanConfig(Bool enableInternalLpbk);
static void App_mcanConfigTxMsg(uint32_t dataLengthPerMsg, uint32_t numMsgs, void* data);
static void App_mcanConfigRxMsg(uint32_t dataLengthPerMsg, uint32_t numMsgs, void* data);
static int32_t App_mcanCompareMsg(uint8_t *txMsg, uint8_t *rxMsg);
static void App_mcanInitMsgRamConfigParams(MCAN_MsgRAMConfigParams *msgRAMConfigParams);
static void App_mcanInitStdFilterElemParams(MCAN_StdMsgIDFilterElement *stdFiltElem, uint32_t bufNum);

static void App_mcanIsrTxFxn(Edma_IntrHandle intrHandle, void *args);
static void App_mcanIsrRxFxn(Edma_IntrHandle intrHandle, void *args);

/* Local Buffers uses for CAN data transmission and reception */
static uint8_t App_txData[MCAN_HEADER_SIZE_BYTES + (APP_MCAN_TEST_MESSAGE_COUNT * APP_MCAN_TEST_DATA_SIZE) + CACHELINE_PADDING] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
static uint8_t App_rxData[MCAN_HEADER_SIZE_BYTES + (APP_MCAN_TEST_MESSAGE_COUNT * APP_MCAN_TEST_DATA_SIZE) + CACHELINE_PADDING] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

void mcan_loopback_dma_main(void *args)
{
    int32_t  status = SystemP_SUCCESS;
    MCAN_ProtocolStatus     protStatus;
    MCAN_ErrCntStatus       errCounter;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("\n[MCAN] Loopback DMA mode, application started ...\r\n");

    /* Assign MCAN instance address */
    gMcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_MCAN0_BASE_ADDR);

    status = SemaphoreP_constructBinary(&gMcanTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status = SemaphoreP_constructBinary(&gMcanRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Initialize the data buffers */
    for (uint32_t i = 0U; i < (APP_MCAN_TEST_MESSAGE_COUNT * APP_MCAN_TEST_DATA_SIZE); i++)
    {
        App_txData[MCAN_HEADER_SIZE_BYTES+i] = i % 256;
        App_rxData[MCAN_HEADER_SIZE_BYTES+i] = 0;
    }

    /* Writeback buffer */
    CacheP_wb(&App_txData[0], sizeof(App_txData), CacheP_TYPE_ALLD);
    CacheP_wb(&App_rxData[0], sizeof(App_rxData), CacheP_TYPE_ALLD);

    /* Configure MCAN module, Enable LoopBack Mode */
    App_mcanConfig(TRUE);

    /* Configure and Enable MCAN DMA Reception */
    App_mcanConfigRxMsg((APP_MCAN_TEST_DATA_SIZE), APP_MCAN_TEST_MESSAGE_COUNT, App_rxData);

    /* Configure and Enable MCAN DMA Transmission */
    App_mcanConfigTxMsg(APP_MCAN_TEST_DATA_SIZE, APP_MCAN_TEST_MESSAGE_COUNT, App_txData);  

    /* Wait for Tx completion */
    SemaphoreP_pend(&gMcanTxDoneSem, SystemP_WAIT_FOREVER);
    /* Wait for Rx completion */
    SemaphoreP_pend(&gMcanRxDoneSem, SystemP_WAIT_FOREVER);

    /* Perform Cache Invalidate on data receive buffer */
    CacheP_inv(App_rxData, sizeof(App_rxData), CacheP_TYPE_ALLD);

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

    /* Checking for Rx Errors */
    MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
    DebugP_assert((0U == errCounter.recErrCnt) &&
                    (0U == errCounter.canErrLogCnt));

    /* Compare Tx/Rx data */
    status = App_mcanCompareMsg(App_txData, App_rxData);

    App_deleteTxMsgConfig();
    App_deleteRxMsgConfig();

    SemaphoreP_destruct(&gMcanTxDoneSem);
    SemaphoreP_destruct(&gMcanRxDoneSem);
    
    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[MCAN] Internal loopback testing for %d iterations Passed\n", APP_MCAN_TEST_MESSAGE_COUNT);
        DebugP_log(" All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("[MCAN] Internal loopback testing for %d iterations Failed\n", APP_MCAN_TEST_MESSAGE_COUNT);
        DebugP_log(" Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

static void App_mcanConfig(Bool enableInternalLpbk)
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

    /* Initialize Filter element to receive msg, should be same as tx msg id */
    for (i = 0U; i < APP_MCAN_STD_ID_FILTER_CNT; i++)
    {
        App_mcanInitStdFilterElemParams(&stdFiltElem[i], i);
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
    for (i = 0U; i < APP_MCAN_STD_ID_FILTER_CNT; i++)
    {
        MCAN_addStdMsgIDFilter(gMcanBaseAddr, i, &stdFiltElem[i]);
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

static void App_mcanConfigRxMsg(uint32_t dataLengthPerMsg, uint32_t numMsgs, void* data)
{
    uint32_t    srcAddr, dstAddr;
    int32_t     status = SystemP_SUCCESS;
    EDMACCPaRAMEntry    edmaRxParam;
    int32_t chAllocStatus = SystemP_SUCCESS;
    bool ret;
  
    gEdmaBaseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(gEdmaBaseAddr != 0);

    /* Allocate EDMA channel for CANFD RX transfer */
    dmaRxCh = EDMA_RESOURCE_ALLOC_ANY;
    chAllocStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaRxCh);
    status += chAllocStatus;

    /* Allocate EDMA TCC for CANFD RX transfer */
    rxTcc = EDMA_RESOURCE_ALLOC_ANY;
    status += EDMA_allocTcc(gEdmaHandle[0], &rxTcc);

    /* Allocate a Param ID for CANFD RX transfer */
    paramRx = EDMA_RESOURCE_ALLOC_ANY;
    status += EDMA_allocParam(gEdmaHandle[0], &paramRx);

    if(status == SystemP_SUCCESS)
    {
        ret = EDMA_configureChannelRegion(gEdmaBaseAddr, rxRegionId, EDMA_CHANNEL_TYPE_DMA,
                                          dmaRxCh, rxTcc, paramRx, EDMA_CANFD_RX_EVT_QUEUE_NO);
        if(ret == (bool)TRUE)
        {
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_SUCCESS;
        }

        /* Register interrupt */
        intrObjRx.tccNum = rxTcc;
        intrObjRx.cbFxn  = &App_mcanIsrRxFxn;
        intrObjRx.appData = (void *) &gMcanRxDoneSem;
        status = EDMA_registerIntr(gEdmaHandle[0], &intrObjRx);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Get the buffer address in message ram. */
        status = MCAN_getReadMsgElemAddress(gMcanBaseAddr,
                                            MCAN_MEM_TYPE_BUF,
                                            rxTcc,
                                            0,
                                            &srcAddr);
        /* Add base address to the offset. */
        srcAddr += gMcanBaseAddr + 8U;
        /* Program the dma to receive the msg. */
        dstAddr = (uint32_t)data;

        /* Transmit param set configuration */
        EDMA_ccPaRAMEntry_init(&edmaRxParam);
        edmaRxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void*) srcAddr);
        edmaRxParam.destAddr      = (uint32_t) SOC_virtToPhy((uint8_t *) dstAddr);
        edmaRxParam.aCnt          = (uint16_t) dataLengthPerMsg;
        edmaRxParam.bCnt          = (uint16_t) numMsgs;
        edmaRxParam.cCnt          = (uint16_t) 1;
        edmaRxParam.bCntReload    = (uint16_t) edmaRxParam.bCnt;
        edmaRxParam.srcBIdx       = (int16_t) 0;
        edmaRxParam.destBIdx      = (int16_t) edmaRxParam.aCnt;
        edmaRxParam.srcCIdx       = (int16_t) 0;
        edmaRxParam.destCIdx      = (int16_t) 0;
        edmaRxParam.linkAddr      = 0xFFFFU;
        edmaRxParam.opt           = 0;
        edmaRxParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | ((rxTcc<< EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

        /* Write Rx param set */
        EDMA_setPaRAM(gEdmaBaseAddr, paramRx, &edmaRxParam);

        /* Set event trigger to start CANFD Rx transfer */
        status = EDMA_enableTransferRegion(gEdmaBaseAddr, rxRegionId, dmaRxCh, EDMA_TRIG_MODE_EVENT);
        DebugP_assert(status == (uint32_t)TRUE);
    }
    else
    { 
        App_deleteRxMsgConfig();
    }
    
    return;
}

static void App_mcanConfigTxMsg(uint32_t dataLengthPerMsg, uint32_t numMsgs, void* data)
{
    int32_t     status = SystemP_SUCCESS;
    EDMACCPaRAMEntry    edmaTxParam;
    uint32_t            srcAddr, dstAddr;
    uint32_t            txElement = 0;
    bool ret;
    MCAN_TxBufElement   txBuffElem;

    /* Initialize message to transmit */
    MCAN_initTxBufElement(&txBuffElem);

    /*  populate the Tx buffer message element  */
    txBuffElem.fdf = 1U;
    txBuffElem.xtd = 0U;
    txBuffElem.dlc = MCAN_DATA_SIZE_64BYTES;
    /* Populate the Id */
    txBuffElem.id  = (APP_MCAN_STD_ID & MCAN_STD_ID_MASK) << MCAN_STD_ID_SHIFT;

    /* Copy the data of first message */
    memcpy ((void*)&txBuffElem.data, data+8, APP_MCAN_TEST_DATA_SIZE);

    MCAN_writeDmaHeader(data, &txBuffElem);
    CacheP_wb(data, MCAN_HEADER_SIZE_BYTES, CacheP_TYPE_ALLD);

    gEdmaBaseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(gEdmaBaseAddr != 0);

    txRegionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(txRegionId < SOC_EDMA_NUM_REGIONS);

    dmaTxCh = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaTxCh);
    DebugP_assert(status == SystemP_SUCCESS);

    txTcc = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(gEdmaHandle[0], &txTcc);
    DebugP_assert(status == SystemP_SUCCESS);

    paramTx = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &paramTx);
    DebugP_assert(status == SystemP_SUCCESS);

    if(status == SystemP_SUCCESS)
    {
        /* Request channel */
        ret = EDMA_configureChannelRegion(gEdmaBaseAddr, txRegionId, EDMA_CHANNEL_TYPE_DMA,
                        dmaTxCh, txTcc, paramTx, EDMA_TEST_EVT_QUEUE_NO);

        if(ret == (bool)TRUE)
        {
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_SUCCESS;
        }
    
        /* Register interrupt */
        intrObjTx.tccNum = txTcc;
        intrObjTx.cbFxn  = &App_mcanIsrTxFxn;
        intrObjTx.appData = (void *) &gMcanTxDoneSem;
        status = EDMA_registerIntr(gEdmaHandle[0], &intrObjTx);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Copy the Header in msg ram */
        MCAN_writeHeaderToMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_BUF, txElement, &txBuffElem);

        srcAddr = (uint32_t)data + 8U;
        /* Get the buffer address in message ram. */
        status = MCAN_getWriteMsgElemAddress(gMcanBaseAddr, MCAN_MEM_TYPE_BUF, txElement, &dstAddr);
        /* Add base address to the offset. */
        dstAddr += gMcanBaseAddr+8;

        /* Program param Set */
        EDMA_ccPaRAMEntry_init(&edmaTxParam);
        edmaTxParam.srcAddr       = SOC_virtToPhy((uint8_t *)srcAddr);
        edmaTxParam.destAddr      = SOC_virtToPhy((uint8_t *)dstAddr);
        edmaTxParam.aCnt          = (uint16_t) dataLengthPerMsg;
        edmaTxParam.bCnt          = (uint16_t) numMsgs;
        edmaTxParam.cCnt          = (uint16_t) 1;
        edmaTxParam.bCntReload    = (uint16_t) edmaTxParam.bCnt;
        edmaTxParam.srcBIdx       = (uint16_t) edmaTxParam.aCnt;
        edmaTxParam.destBIdx      = 0U;
        edmaTxParam.srcCIdx       = 0U;
        edmaTxParam.destCIdx      = 0U;
        edmaTxParam.linkAddr      = 0xFFFFU;
        edmaTxParam.opt           = 0U;
        edmaTxParam.opt          |=
            (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | ((((uint32_t)txTcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

        /* Write Tx param set */
        EDMA_setPaRAM(gEdmaBaseAddr, paramTx, &edmaTxParam);

        /* Set event trigger to start CANFD Tx transfer */
        status = EDMA_enableTransferRegion(gEdmaBaseAddr, txRegionId, dmaTxCh, EDMA_TRIG_MODE_EVENT);
        DebugP_assert(status == (uint32_t)TRUE);
        /* Set event trigger to start CANFD Tx transfer */
        status = EDMA_enableTransferRegion(gEdmaBaseAddr, txRegionId, dmaTxCh, EDMA_TRIG_MODE_MANUAL);
        DebugP_assert(status == (uint32_t)TRUE);        
    }
    else
    {
        App_deleteTxMsgConfig();
    }

    return;
}

static void App_deleteTxMsgConfig(void)
{
    int32_t  status = SystemP_SUCCESS;

    /* Free channel */
    EDMA_freeChannelRegion(gEdmaBaseAddr, txRegionId, EDMA_CHANNEL_TYPE_DMA,
        dmaTxCh, EDMA_TRIG_MODE_MANUAL, txTcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaTxCh);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeTcc(gEdmaHandle[0], &txTcc);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeParam(gEdmaHandle[0], &paramTx);
    DebugP_assert(status == SystemP_SUCCESS);
}

static void App_deleteRxMsgConfig(void)
{
    int32_t  status = SystemP_SUCCESS;

    /* Free channel */
    EDMA_freeChannelRegion(gEdmaBaseAddr, txRegionId, EDMA_CHANNEL_TYPE_DMA,
        dmaTxCh, EDMA_TRIG_MODE_MANUAL, rxTcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaRxCh);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeTcc(gEdmaHandle[0], &rxTcc);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeParam(gEdmaHandle[0], &paramRx);
    DebugP_assert(status == SystemP_SUCCESS);
}

static int32_t App_mcanCompareMsg(uint8_t *txMsg, uint8_t *rxMsg)
{
    int32_t  status = SystemP_SUCCESS;

    for (uint32_t i = 0U; i < APP_MCAN_TEST_MESSAGE_COUNT * APP_MCAN_TEST_DATA_SIZE; i++)
    {
        if (txMsg[i+8] != rxMsg[i])
        {
            /* Data mismatch */
            DebugP_log(" Data mismatch at offset %d!!!\r\n", i+8U);
            status = SystemP_FAILURE;
            break;
        }
    }

    return status;
}

static void App_mcanInitStdFilterElemParams(MCAN_StdMsgIDFilterElement *stdFiltElem,
                                            uint32_t bufNum)
{
    /* sfid1 defines the ID of the standard message to be stored. */
    stdFiltElem->sfid1 = APP_MCAN_STD_ID;
    /* As buffer mode is selected, sfid2 should be bufNum[0 - 63] */
    stdFiltElem->sfid2 = bufNum;
    /* Enable filter event for RX */
    stdFiltElem->sfid2 |= 0x40; 
    /* Store message in buffer */
    stdFiltElem->sfec  = MCAN_STD_FILT_ELEM_BUFFER;
    /* Below configuration is ignored if message is stored in buffer */
    stdFiltElem->sft   = MCAN_STD_FILT_TYPE_RANGE;

    return;
}

static void App_mcanInitMsgRamConfigParams(MCAN_MsgRAMConfigParams
                                           *msgRAMConfigParams)
{
    uint32_t startAddr = 0;

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

    /* Tx buffer configuration */
    msgRAMConfigParams->txStartAddr = startAddr;
    msgRAMConfigParams->txBufMode = 1U;
    msgRAMConfigParams->txBufElemSize = MCAN_ELEM_SIZE_64BYTES;

    /* Tx Event FIFO configuration */
    msgRAMConfigParams->txEventFIFOStartAddr = 0;
    msgRAMConfigParams->txEventFIFOCnt = 0;
    msgRAMConfigParams->txEventFIFOWaterMark = 0;

    /* Rx Buffer configuration */
    startAddr += (msgRAMConfigParams->txBufCnt * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
    startAddr += (msgRAMConfigParams->txFIFOCnt * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
    msgRAMConfigParams->rxBufStartAddr = startAddr;
    msgRAMConfigParams->rxBufElemSize = MCAN_ELEM_SIZE_64BYTES;

    /* 11-bit filter configuration */
    startAddr += (64 * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
    msgRAMConfigParams->flssa = startAddr;
    msgRAMConfigParams->lss = msgRAMConfigParams->lss;

    /* 29-bit filter configuration */
    startAddr += ((msgRAMConfigParams->lss + 1U) * MCAN_MSG_RAM_STD_ELEM_SIZE * 4U);
    msgRAMConfigParams->flesa = startAddr;
    msgRAMConfigParams->lse = msgRAMConfigParams->lse;

    /* Rx FIFO 0 configuration */
    startAddr += ((msgRAMConfigParams->lse + 1U) * MCAN_MSG_RAM_EXT_ELEM_SIZE * 4U);
    msgRAMConfigParams->rxFIFO0StartAddr = startAddr;
    msgRAMConfigParams->rxFIFO0WaterMark = 0U;
    msgRAMConfigParams->rxFIFO0ElemSize = MCAN_ELEM_SIZE_64BYTES;

    /* Rx FIFO 1 configuration */
    startAddr += ((msgRAMConfigParams->rxFIFO0Cnt + 1U) * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
    msgRAMConfigParams->rxFIFO1StartAddr = startAddr;
    msgRAMConfigParams->rxFIFO1WaterMark = 0U;
    msgRAMConfigParams->rxFIFO1ElemSize = MCAN_ELEM_SIZE_64BYTES;
}

static void App_mcanIsrTxFxn(Edma_IntrHandle intrHandle, void *args)
{
    int32_t  status = SystemP_FAILURE;

    if(NULL != args)
    {
        gTxNumMsgCount++;
        /* Add request for transmission, This function will trigger transmission */
        status = MCAN_txBufAddReq(gMcanBaseAddr, 0U);
        DebugP_assert(status == CSL_PASS);
        
        if(gTxNumMsgCount == APP_MCAN_TEST_MESSAGE_COUNT)
        {
            SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
            DebugP_assert(semObjPtr != NULL);
            SemaphoreP_post(semObjPtr);
        }
    }
}

static void App_mcanIsrRxFxn(Edma_IntrHandle intrHandle, void *args)
{
    MCAN_RxNewDataStatus newDataStatus = {0};

    if(NULL != args)
    {
        /* Get the new data status */
        MCAN_getNewDataStatus(gMcanBaseAddr, &newDataStatus);
        /* Clear NewData status to accept new messages */
        MCAN_clearNewDataStatus(gMcanBaseAddr, &newDataStatus);

        gRxNumMsgCount++;
        if(gRxNumMsgCount == APP_MCAN_TEST_MESSAGE_COUNT)
        {
            SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
            DebugP_assert(semObjPtr != NULL);
            SemaphoreP_post(semObjPtr);
        }
    }
}

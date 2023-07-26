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

/* This example demonstrates the CAN message transmission and reception with DMA in
 * digital loop back mode with the following configuration.
 *
 * CAN FD Message Format.
 * Message ID Type is Standard, Msg Id 0xC0.
 * MCAN is configured in DMA Mode.
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
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/mcan.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Base address of MCAN module used */
#define APP_MCAN_BASE_ADDR                       (CONFIG_MCAN0_BASE_ADDR)

/*
 * Allocate Message RAM memory section to filter elements, buffers, FIFO.
 * The PDMA channels in AM64x/AM243x are hardcoded to read MCAN buffers at
 * specific addresses. Thus, the Message RAM must be configured so that the
 * RX and TX buffers are at that memory address. Once Data is copied to
 * Message RAM buffer, PDMA also sets the Add request bit in TXBAR register
 * to start the message transfer.
 *
 * Below is the Buffer Mapping for TX and RX channel -
 *          PDMA Channel                  Buffer Address
 * UDMA_PDMA_CH_MAIN1_MCAN0_CH0_TX          0x20708000
 * UDMA_PDMA_CH_MAIN1_MCAN0_CH1_TX          0x20708048
 * UDMA_PDMA_CH_MAIN1_MCAN0_CH2_TX          0x20708090
 * UDMA_PDMA_CH_MAIN1_MCAN1_CH0_TX          0x20718000
 * UDMA_PDMA_CH_MAIN1_MCAN1_CH1_TX          0x20718048
 * UDMA_PDMA_CH_MAIN1_MCAN1_CH2_TX          0x20718090
 * UDMA_PDMA_CH_MAIN1_MCAN0_CH0_RX          0x20708900
 * UDMA_PDMA_CH_MAIN1_MCAN0_CH1_RX          0x20708990
 * UDMA_PDMA_CH_MAIN1_MCAN0_CH2_RX          0x20708A20
 * UDMA_PDMA_CH_MAIN1_MCAN1_CH0_RX          0x20718900
 * UDMA_PDMA_CH_MAIN1_MCAN1_CH1_RX          0x20718990
 * UDMA_PDMA_CH_MAIN1_MCAN1_CH2_RX          0x20718A20
 *
 * Three Filter event bits can be used to select the CAN events to be tiggered
 * upon filter match.
 *
 * Below is the Filter configuration to MCAN Channel mapping -
 * CANFE2  CANFE1  CANFE0    Description
 *   0       0        0        Not Used
 *   0       0        1        Not Used
 *   0       1        0        PDMA CAN Channel offset 0, RX Buffer 0
 *   0       1        1        PDMA CAN Channel offset 0, RX Buffer 1
 *   1       0        0        PDMA CAN Channel offset 1, RX Buffer 2
 *   1       0        1        PDMA CAN Channel offset 1, RX Buffer 3
 *   1       1        0        PDMA CAN Channel offset 2, RX Buffer 4
 *   1       1        1        PDMA CAN Channel offset 2, RX Buffer 5
 */

/* Filter Event config used for MCAN UDMA reception (from above table) */
#define APP_MCAN_FILTER_EVENT                    (0x2U)
/* MCAN RX Buffer used for data reception */
#define APP_MCAN_RX_BUFF                         (0x0U)
/* MCAN Filter used for data reception */
#define APP_MCAN_RX_FILTER                       (0x0U)
/* Maximum STD Filter Element can be configured is 128 */
#define APP_MCAN_STD_ID_FILTER_CNT               (127U)
/* Maximum EXT Filter Element can be configured is 64 */
#define APP_MCAN_EXT_ID_FILTER_CNT               (64U)
/* Maximum TX Buffer + TX FIFO, combined can be configured is 32 */
#define APP_MCAN_TX_BUFF_CNT                     (32U)
#define APP_MCAN_TX_FIFO_CNT                     (0U)
/* Maximum TX Event FIFO can be configured is 32 */
#define APP_MCAN_TX_EVENT_FIFO_CNT               (0U)
/* Maximum RX FIFO 0 can be configured is 64 */
#define APP_MCAN_FIFO_0_CNT                      (64U)
/* Maximum RX FIFO 1 can be configured is 64 */
#define APP_MCAN_FIFO_1_CNT                      (64U)
/* Standard Id configured in this app */
#define APP_MCAN_STD_ID                          (0xC0U)
#define APP_MCAN_EXT_ID_MASK                     (0x1FFFFFFFU)
/* Number of messages sent */
#define APP_MCAN_TEST_MESSAGE_COUNT              (10U)
/* Number of messages sent */
#define APP_MCAN_TEST_DATA_SIZE                  (64U)
/* Padding required to align the buffers to cacheline */
#define CACHELINE_PADDING                        (24U)
/* PFilter event shift in SFID */
#define APP_MCAN_SFID2_FILTER_EVENT_SHIFT        (6U)

/*
 * UDMA Ring parameters
 */
/** \brief Number of ring entries - we can prime this much ADC operations */
#define APP_MCAN_UDMA_TEST_RING_ENTRIES          (1U)
/** \brief Size (in bytes) of each ring entry (Size of pointer - 64-bit) */
#define APP_MCAN_UDMA_TEST_RING_ENTRY_SIZE       (sizeof(uint64_t))
/** \brief Total ring memory */
#define APP_MCAN_UDMA_TEST_RING_MEM_SIZE         (APP_MCAN_UDMA_TEST_RING_ENTRIES * APP_MCAN_UDMA_TEST_RING_ENTRY_SIZE)
/** \brief UDMA host mode buffer descriptor memory size. */
#define APP_MCAN_UDMA_TEST_DESC_SIZE             (sizeof(CSL_UdmapCppi5HMPD))

/* MCAN UDMA Channel Configurations */
/* MCAN UDMA TX and RX Channel Objects */
static Udma_ChObject App_mcanUdmaTxChObj;
static Udma_ChObject App_mcanUdmaRxChObj;
/**< UDMA TX and RX completion queue object */
static Udma_EventObject        App_mcanUdmaCqTxEventObjCh;
static Udma_EventObject        App_mcanUdmaCqRxEventObjCh;
/* MCAN UDMA Channel Ring Mem */
static uint8_t App_mcanUdmaRxRingMemCh[APP_MCAN_UDMA_TEST_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t App_mcanUdmaTxRingMemCh[APP_MCAN_UDMA_TEST_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
/* MCAN UDMA Channel HPD Mem */
static uint8_t App_mcanUdmaTxHpdMemCh[APP_MCAN_UDMA_TEST_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t App_mcanUdmaRxHpdMemCh[APP_MCAN_UDMA_TEST_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
/* Local Buffers uses for CAN data transmission and reception */
static uint8_t App_txData[MCAN_HEADER_SIZE_BYTES + (APP_MCAN_TEST_MESSAGE_COUNT * APP_MCAN_TEST_DATA_SIZE) + CACHELINE_PADDING] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t App_rxData[MCAN_HEADER_SIZE_BYTES + (APP_MCAN_TEST_MESSAGE_COUNT * APP_MCAN_TEST_DATA_SIZE) + CACHELINE_PADDING] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Semaphore to indicate TX and RX transfer completion */
static SemaphoreP_Object App_mcanTxDoneSem, App_mcanRxDoneSem;
/* Variable to store the MCAN module base address */
static uint32_t          App_mcanBaseAddr;
/* Variable to store UDMA handle */
static Udma_DrvHandle    App_udmaDrvHandle;

/* Static Function Declarations */
static void App_mcanIntrISR(void *arg);
static void App_mcanConfig(Bool enableInternalLpbk);
static void App_mcanInitMsgRamConfigParams(MCAN_MsgRAMConfigParams *msgRAMConfigParams);
static void App_mcanConfigTxMsg();
static void App_mcanConfigRxMsg();
static void App_mcanCompareMsg(uint8_t *txMsg, uint8_t *rxMsg);
static void App_mcanInitStdFilterElemParams(MCAN_StdMsgIDFilterElement *stdFiltElem, uint32_t bufNum);
static void App_udmaCallbackTx(Udma_EventHandle eventHandle, uint32_t eventType, void *args);
static void App_udmaCallbackRx(Udma_EventHandle eventHandle, uint32_t eventType, void *args);
static void App_mcanStartTx(uint32_t dataLengthPerMsg, uint32_t numMsgs, void* data);
static void App_mcanStartRx(uint32_t dataLengthPerMsg, uint32_t numMsgs, void* data);
static void App_mcanDmaDeinit();

void mcan_loopback_dma_main(void *args)
{
    int32_t                 status = SystemP_SUCCESS;
    uint32_t                i;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[MCAN] Loopback DMA mode, application started ...\r\n");

    /* Construct Tx/Rx Semaphore objects */
    status = SemaphoreP_constructBinary(&App_mcanTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status = SemaphoreP_constructBinary(&App_mcanRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Get the UDMA driver instance handle */
    App_udmaDrvHandle = &gUdmaDrvObj[CONFIG_UDMA0];

    /* Initialize the data buffers */
    for (i=0; i<APP_MCAN_TEST_MESSAGE_COUNT*APP_MCAN_TEST_DATA_SIZE; i++)
    {
        App_txData[MCAN_HEADER_SIZE_BYTES+i] = i%256;
        App_rxData[MCAN_HEADER_SIZE_BYTES+i] = 0;
    }

    /* Writeback buffer */
    CacheP_wb(&App_txData[0U], sizeof(App_txData), CacheP_TYPE_ALLD);
    CacheP_wb(&App_rxData[0U], sizeof(App_rxData), CacheP_TYPE_ALLD);

    /* Assign MCAN instance address */
    App_mcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(APP_MCAN_BASE_ADDR);

    /* Configure MCAN module, Enable LoopBack Mode */
    App_mcanConfig(TRUE);

    /* Configure Tx Msg to transmit */
    App_mcanConfigTxMsg();

    /* Configure Rx Msg to receive */
    App_mcanConfigRxMsg();

    /* Enable MCAN DMA Reception */
    App_mcanStartRx(APP_MCAN_TEST_DATA_SIZE + MCAN_HEADER_SIZE_BYTES, APP_MCAN_TEST_MESSAGE_COUNT, App_rxData);

    /* Enable MCAN DMA Transmission */
    App_mcanStartTx(APP_MCAN_TEST_DATA_SIZE, APP_MCAN_TEST_MESSAGE_COUNT, App_txData);

    /* Wait for Tx completion */
    SemaphoreP_pend(&App_mcanTxDoneSem, SystemP_WAIT_FOREVER);

    /* Wait for Rx completion */
    SemaphoreP_pend(&App_mcanRxDoneSem, SystemP_WAIT_FOREVER);

    /* Perform Cache Invalidate on data receive buffer */
    CacheP_inv((void*)App_rxData, sizeof(App_rxData), CacheP_TYPE_ALLD);

    /* Compare Tx/Rx data */
    App_mcanCompareMsg(App_txData, App_rxData);

    /* De-Construct Tx/Rx Semaphore objects */
    SemaphoreP_destruct(&App_mcanTxDoneSem);
    SemaphoreP_destruct(&App_mcanRxDoneSem);

    /* Deinitialize UDMA parameters */
    App_mcanDmaDeinit();

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();

    return;
}

static void App_mcanConfig(Bool enableInternalLpbk)
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
    while (FALSE == MCAN_isMemInitDone(App_mcanBaseAddr))
    {}

    /* Put MCAN in SW initialization mode */
    MCAN_setOpMode(App_mcanBaseAddr, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(App_mcanBaseAddr))
    {}

    /* Initialize MCAN module */
    MCAN_init(App_mcanBaseAddr, &initParams);
    /* Configure MCAN module Gloabal Filter */
    MCAN_config(App_mcanBaseAddr, &configParams);
    /* Configure Bit timings */
    MCAN_setBitTime(App_mcanBaseAddr, &bitTimes);
    /* Configure Message RAM Sections */
    MCAN_msgRAMConfig(App_mcanBaseAddr, &msgRAMConfigParams);
    /* Set Extended ID Mask */
    MCAN_setExtIDAndMask(App_mcanBaseAddr, APP_MCAN_EXT_ID_MASK);

    if (TRUE == enableInternalLpbk)
    {
        MCAN_lpbkModeEnable(App_mcanBaseAddr, MCAN_LPBK_MODE_INTERNAL, TRUE);
    }

    /* Take MCAN out of the SW initialization mode */
    MCAN_setOpMode(App_mcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(App_mcanBaseAddr))
    {}

    return;
}

static void App_mcanConfigTxMsg()
{
    uint32_t            chType;
    int32_t             retVal = SystemP_SUCCESS;
    Udma_ChPrms         chPrms;
    Udma_ChTxPrms       txPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_ChHandle       txChHandle;

    /* Init TX channel parameters */
    chType = UDMA_CH_TYPE_PDMA_TX;
    UdmaChPrms_init(&chPrms, chType);
    chPrms.peerChNum                = UDMA_PDMA_CH_MAIN1_MCAN0_CH0_TX;
    chPrms.fqRingPrms.ringMem       = &App_mcanUdmaTxRingMemCh;
    chPrms.fqRingPrms.ringMemSize   = APP_MCAN_UDMA_TEST_RING_MEM_SIZE;
    chPrms.fqRingPrms.elemCnt       = APP_MCAN_UDMA_TEST_RING_ENTRIES;
    txChHandle                      = &App_mcanUdmaTxChObj;

    /* Open channel for block copy */
    retVal = Udma_chOpen(App_udmaDrvHandle, txChHandle, chType, &chPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config TX channel */
    UdmaChTxPrms_init(&txPrms, chType);
    retVal = Udma_chConfigTx(txChHandle, &txPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Register ring completion callback */
    eventHandle = &App_mcanUdmaCqTxEventObjCh;
    UdmaEventPrms_init(&eventPrms);
    eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    eventPrms.chHandle          = txChHandle;
    eventPrms.controllerEventHandle = Udma_eventGetGlobalHandle(App_udmaDrvHandle);
    eventPrms.eventCb           = &App_udmaCallbackTx;
    eventPrms.appData           = NULL;
    retVal = Udma_eventRegister(App_udmaDrvHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);
    return;
}

static void App_mcanDmaDeinit()
{
    int32_t  status = UDMA_SOK;
    uint64_t pDesc;
    int32_t  tempRetVal;

    /* Disable Channel */
    status = Udma_chDisable(&App_mcanUdmaTxChObj, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == status);
    status = Udma_chDisable(&App_mcanUdmaRxChObj, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == status);

    /* UnRegister Event */
    status = Udma_eventUnRegister(&App_mcanUdmaCqTxEventObjCh);
    DebugP_assert(UDMA_SOK == status);
    status = Udma_eventUnRegister(&App_mcanUdmaCqRxEventObjCh);
    DebugP_assert(UDMA_SOK == status);

    /* Flush any pending request from the free queue */
    while(1)
    {
        tempRetVal = Udma_ringFlushRaw(
                         Udma_chGetFqRingHandle(&App_mcanUdmaTxChObj), &pDesc);
        if(UDMA_ETIMEOUT == tempRetVal)
        {
            break;
        }
    }
    while(1)
    {
        tempRetVal = Udma_ringFlushRaw(
                         Udma_chGetFqRingHandle(&App_mcanUdmaRxChObj), &pDesc);
        if(UDMA_ETIMEOUT == tempRetVal)
        {
            break;
        }
    }

    /* Close channel */
    status = Udma_chClose(&App_mcanUdmaTxChObj);
    DebugP_assert(UDMA_SOK == status);
    status = Udma_chClose(&App_mcanUdmaRxChObj);
    DebugP_assert(UDMA_SOK == status);
}

static void App_mcanConfigRxMsg()
{
    uint32_t            chType;
    int32_t             retVal = SystemP_SUCCESS;
    Udma_ChPrms         chPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_ChHandle       rxChHandle;
    MCAN_StdMsgIDFilterElement stdFiltElem;

    /* Initialize Filter element to receive msg, should be same as tx msg id */
    App_mcanInitStdFilterElemParams(&stdFiltElem, APP_MCAN_RX_BUFF);
    /* Configure Standard ID filter element */
    MCAN_addStdMsgIDFilter(App_mcanBaseAddr, APP_MCAN_RX_FILTER, &stdFiltElem);

    /* Init RX channel parameters */
    chType = UDMA_CH_TYPE_PDMA_RX;
    UdmaChPrms_init(&chPrms, chType);
    chPrms.peerChNum                = UDMA_PDMA_CH_MAIN1_MCAN0_CH0_RX;
    chPrms.fqRingPrms.ringMem       = &App_mcanUdmaRxRingMemCh;
    chPrms.fqRingPrms.ringMemSize   = APP_MCAN_UDMA_TEST_RING_MEM_SIZE;
    chPrms.fqRingPrms.elemCnt       = APP_MCAN_UDMA_TEST_RING_ENTRIES;
    rxChHandle                      = &App_mcanUdmaRxChObj;

    /* Open channel for block copy */
    retVal = Udma_chOpen(App_udmaDrvHandle, rxChHandle, chType, &chPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config RX channel */
    UdmaChRxPrms_init(&rxPrms, chType);
    retVal = Udma_chConfigRx(rxChHandle, &rxPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Register ring completion callback */
    eventHandle = &App_mcanUdmaCqRxEventObjCh;
    UdmaEventPrms_init(&eventPrms);
    eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    eventPrms.chHandle          = rxChHandle;
    eventPrms.controllerEventHandle = Udma_eventGetGlobalHandle(App_udmaDrvHandle);
    eventPrms.eventCb           = &App_udmaCallbackRx;
    eventPrms.appData           = NULL;
    retVal = Udma_eventRegister(App_udmaDrvHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);
    return;
}

static void CANFD_udmaHpdInit(Udma_ChHandle chHandle,
                              uint8_t       *pHpdMem,
                              const void    *destBuf,
                              uint32_t      length)
{
    CSL_UdmapCppi5HMPD *pHpd = (CSL_UdmapCppi5HMPD *) pHpdMem;
    uint32_t descType = (uint32_t)CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST;

    /* Setup descriptor */
    CSL_udmapCppi5SetDescType(pHpd, descType);
    CSL_udmapCppi5SetEpiDataPresent(pHpd, FALSE);
    CSL_udmapCppi5SetPsDataLoc(pHpd, 0U);
    CSL_udmapCppi5SetPsDataLen(pHpd, 0U);
    CSL_udmapCppi5SetPktLen(pHpd, descType, length);
    CSL_udmapCppi5SetPsFlags(pHpd, 0U);
    CSL_udmapCppi5SetIds(pHpd, descType, 0x321, UDMA_DEFAULT_FLOW_ID);
    CSL_udmapCppi5SetSrcTag(pHpd, 0x0000);     /* Not used */
    CSL_udmapCppi5SetDstTag(pHpd, 0x0000);     /* Not used */
    /* Return Policy descriptors are reserved in case of AM243X/Am64X */
    CSL_udmapCppi5SetReturnPolicy(
        pHpd,
        descType,
        0U,
        0U,
        0U,
        0U);
    CSL_udmapCppi5LinkDesc(pHpd, 0U);
    CSL_udmapCppi5SetBufferAddr(pHpd, (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL));
    CSL_udmapCppi5SetBufferLen(pHpd, length);
    CSL_udmapCppi5SetOrgBufferAddr(pHpd, (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL));
    CSL_udmapCppi5SetOrgBufferLen(pHpd, length);

    /* Writeback cache */
    CacheP_wb(pHpdMem, sizeof(CSL_UdmapCppi5HMPD), CacheP_TYPE_ALLD);

    return;
}

static void App_mcanStartTx(uint32_t dataLengthPerMsg, uint32_t numMsgs, void* data)
{
    int32_t             retVal;
    Udma_ChPdmaPrms     pdmaPrms;
    Udma_ChHandle       txChHandle;
    MCAN_TxBufElement   txBuffElem;

    txChHandle = &App_mcanUdmaTxChObj;

     /* populate the Tx buffer message element */
    txBuffElem.rtr = 0;
    txBuffElem.esi = 0;
    txBuffElem.efc = 0;
    txBuffElem.mm = 0;
    txBuffElem.brs = 1U;
    txBuffElem.fdf = 1U;

    /* Populate the Id */
    txBuffElem.xtd = 0;
    txBuffElem.id = (APP_MCAN_STD_ID & MCAN_STD_ID_MASK) << MCAN_STD_ID_SHIFT;

    /* Copy the data of first message */
    memcpy ((void*)&txBuffElem.data, data, dataLengthPerMsg);

    txBuffElem.dlc = MCAN_DATA_SIZE_64BYTES;

    MCAN_writeDmaHeader(data, &txBuffElem);
    CacheP_wb(data, MCAN_HEADER_SIZE_BYTES, CacheP_TYPE_ALLD);

    /* Config PDMA channel */
    UdmaChPdmaPrms_init(&pdmaPrms);

    pdmaPrms.elemSize = UDMA_PDMA_ES_8BITS;

    /* Number of words received in each transfer */
    pdmaPrms.elemCnt = dataLengthPerMsg + MCAN_HEADER_SIZE_BYTES;
    /* Dont care for Tx */
    pdmaPrms.fifoCnt    = 0U;

    retVal = Udma_chConfigPdma(txChHandle, &pdmaPrms);
    DebugP_assert(UDMA_SOK == retVal);

    retVal = Udma_chEnable(txChHandle);
    DebugP_assert(UDMA_SOK == retVal);

    /* Update host packet descriptor, length should be always in terms of total number of bytes */
    CANFD_udmaHpdInit(txChHandle, (uint8_t *) &App_mcanUdmaTxHpdMemCh, data, MCAN_HEADER_SIZE_BYTES+(dataLengthPerMsg*numMsgs));

    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(txChHandle),
                 (uint64_t) Udma_defaultVirtToPhyFxn(&App_mcanUdmaTxHpdMemCh, 0U, NULL));
    DebugP_assert(UDMA_SOK == retVal);
}

static void App_mcanStartRx(uint32_t dataLengthPerMsg, uint32_t numMsgs, void* data)
{
    int32_t             retVal;
    Udma_ChPdmaPrms     pdmaPrms;
    Udma_ChHandle       rxChHandle;

    rxChHandle = &App_mcanUdmaRxChObj;

    /* Config PDMA channel */
    UdmaChPdmaPrms_init(&pdmaPrms);

    pdmaPrms.elemSize = UDMA_PDMA_ES_8BITS;

    /* Number of words received in each transfer */
    pdmaPrms.elemCnt = dataLengthPerMsg;
    /* Dont care for Tx */
    pdmaPrms.fifoCnt    = numMsgs;

    retVal = Udma_chConfigPdma(rxChHandle, &pdmaPrms);
    DebugP_assert(UDMA_SOK == retVal);

    retVal = Udma_chEnable(rxChHandle);
    DebugP_assert(UDMA_SOK == retVal);

    /* Update host packet descriptor, length should be always in terms of total number of bytes */
    CANFD_udmaHpdInit(rxChHandle, (uint8_t *) &App_mcanUdmaRxHpdMemCh, data, dataLengthPerMsg*numMsgs);

    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(rxChHandle),
                 (uint64_t) Udma_defaultVirtToPhyFxn(&App_mcanUdmaRxHpdMemCh, 0U, NULL));
    DebugP_assert(UDMA_SOK == retVal);
}

static void App_mcanInitStdFilterElemParams(MCAN_StdMsgIDFilterElement *stdFiltElem,
                                            uint32_t bufNum)
{
    /* sfid1 defines the ID of the standard message to be stored. */
    stdFiltElem->sfid1 = APP_MCAN_STD_ID;
    /* As buffer mode is selected, sfid2 should be bufNum[0 - 63] */
    stdFiltElem->sfid2 = bufNum;
    stdFiltElem->sfid2 |=(APP_MCAN_FILTER_EVENT << APP_MCAN_SFID2_FILTER_EVENT_SHIFT);
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
    msgRAMConfigParams->rxFIFO0WaterMark = 0;
    msgRAMConfigParams->rxFIFO0ElemSize = MCAN_ELEM_SIZE_64BYTES;

    /* Rx FIFO 1 configuration */
    startAddr += ((msgRAMConfigParams->rxFIFO0Cnt + 1U) * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
    msgRAMConfigParams->rxFIFO1StartAddr = startAddr;
    msgRAMConfigParams->rxFIFO1WaterMark = 0;
    msgRAMConfigParams->rxFIFO1ElemSize = MCAN_ELEM_SIZE_64BYTES;
}

static void App_mcanCompareMsg(uint8_t *txMsg, uint8_t *rxMsg)
{
    uint32_t i, txMsgId, rxMsgId;

    /* Extract MessageID from first 4 bytes of CAN header */
    txMsgId = (((*((uint32_t*)txMsg))>> MCAN_STD_ID_SHIFT) & MCAN_STD_ID_MASK);
    rxMsgId = (((*((uint32_t*)rxMsg))>> MCAN_STD_ID_SHIFT) & MCAN_STD_ID_MASK);

    if (txMsgId == rxMsgId)
    {
        for (i = 0U; i < APP_MCAN_TEST_MESSAGE_COUNT*APP_MCAN_TEST_DATA_SIZE; i++)
        {
            if (txMsg[i+8] != rxMsg[i+8])
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

static void App_udmaCallbackTx(Udma_EventHandle eventHandle,
                                 uint32_t eventType,
                                 void *args)
{
    uint64_t pDesc;

    CacheP_inv(&App_mcanUdmaTxHpdMemCh, APP_MCAN_UDMA_TEST_DESC_SIZE, CacheP_TYPE_ALLD);
    Udma_ringDequeueRaw(Udma_chGetCqRingHandle(&App_mcanUdmaTxChObj), &pDesc);

    SemaphoreP_post(&App_mcanTxDoneSem);
}

static void App_udmaCallbackRx(Udma_EventHandle eventHandle,
                                 uint32_t eventType,
                                 void *args)
{
    uint64_t pDesc;

    CacheP_inv(&App_mcanUdmaRxHpdMemCh, APP_MCAN_UDMA_TEST_DESC_SIZE, CacheP_TYPE_ALLD);
    Udma_ringDequeueRaw(Udma_chGetCqRingHandle(&App_mcanUdmaRxChObj), &pDesc);

    SemaphoreP_post(&App_mcanRxDoneSem);
}
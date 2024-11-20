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
 *
 */

/**
 *  \file test_mcan_canfd.c
 *
 *  \brief This file contains mcan test code.
 * 
 * \details  This UT demonstrates the MCAN operation configured in different 
 *           configurations and all possible MCAN instances that can be 
 *           configured. This example sends a known data of length 
 *           APP_MCAN_DATASIZE and then receives the same in RX mode. 
 *           Internal pad level loopback mode is enabled to receive data.
 *           When transfer is completed, TX and RX buffer data are compared.
 *           If data is matched, test result is passed otherwise failed.
 */

#include "string.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <unity.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/mcan/v0/canfd.h>
#include "test_mcan_canfd.h"
#include <drivers/mcan.h>

uint8_t rxData[MCAN_APP_TEST_DATA_SIZE] = {0};
uint8_t txData[MCAN_APP_TEST_DATA_SIZE];
static SemaphoreP_Object    gMcanTxDoneSem, gMcanRxDoneSem;

static void test_canfd_loopback(void *args);
static void test_canfd_set_params(CANFD_TestParams *testParams, uint32_t tcId);
static void App_CANFD_TransferCallback(void *args, CANFD_Reason reason);
static void App_CANFD_ErrorCallback(void *args, CANFD_Reason reason, CANFD_ErrStatusResp* errStatusResp);

void test_main(void *args)
{
    CANFD_TestParams  testParams;

    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    test_canfd_set_params(&testParams, 13920);
    RUN_TEST(test_canfd_loopback,  13920, (void*)&testParams);
    test_canfd_set_params(&testParams, 13921);
    RUN_TEST(test_canfd_loopback,  13921, (void*)&testParams);
    test_canfd_set_params(&testParams, 13922);
    RUN_TEST(test_canfd_loopback,  13922, (void*)&testParams);
    test_canfd_set_params(&testParams, 13923);
    RUN_TEST(test_canfd_loopback,  13923, (void*)&testParams);
    test_canfd_set_params(&testParams, 13924);
    RUN_TEST(test_canfd_loopback,  13924, (void*)&testParams);
    test_canfd_set_params(&testParams, 13925);
    RUN_TEST(test_canfd_loopback,  13925, (void*)&testParams);
    test_canfd_set_params(&testParams, 13926);
    RUN_TEST(test_canfd_loopback,  13926, (void*)&testParams);
    test_canfd_set_params(&testParams, 13927);
    RUN_TEST(test_canfd_loopback,  13927, (void*)&testParams);
    test_canfd_set_params(&testParams, 13928);
    RUN_TEST(test_canfd_loopback,  13928, (void*)&testParams);
    test_canfd_set_params(&testParams, 13929);
    RUN_TEST(test_canfd_loopback,  13929, (void*)&testParams);
    test_canfd_set_params(&testParams, 13930);
    RUN_TEST(test_canfd_loopback,  13930, (void*)&testParams);
    test_canfd_set_params(&testParams, 13931);
    RUN_TEST(test_canfd_loopback,  13931, (void*)&testParams);
    test_canfd_set_params(&testParams, 13933);
    RUN_TEST(test_canfd_loopback,  13933, (void*)&testParams);
    test_canfd_set_params(&testParams, 13934);
    RUN_TEST(test_canfd_loopback,  13934, (void*)&testParams);
    test_canfd_set_params(&testParams, 13935);
    RUN_TEST(test_canfd_loopback,  13935, (void*)&testParams);
    test_canfd_set_params(&testParams, 13936);
    RUN_TEST(test_canfd_loopback,  13936, (void*)&testParams);
    test_canfd_set_params(&testParams, 13937);
    RUN_TEST(test_canfd_loopback,  13937, (void*)&testParams);
    test_canfd_set_params(&testParams, 13938);
    RUN_TEST(test_canfd_loopback,  13938, (void*)&testParams);
    test_canfd_set_params(&testParams, 13939);
    RUN_TEST(test_canfd_loopback,  13939, (void*)&testParams);
    test_canfd_set_params(&testParams, 13940);
    RUN_TEST(test_canfd_loopback,  13940, (void*)&testParams);
    test_canfd_set_params(&testParams, 13941);
    RUN_TEST(test_canfd_loopback,  13941, (void*)&testParams);
    test_canfd_set_params(&testParams, 13942);
    RUN_TEST(test_canfd_loopback,  13942, (void*)&testParams);
    test_canfd_set_params(&testParams, 13943);
    RUN_TEST(test_canfd_loopback,  13943, (void*)&testParams);
    test_canfd_set_params(&testParams, 13944);
    RUN_TEST(test_canfd_loopback,  13944, (void*)&testParams);
    test_canfd_set_params(&testParams, 13945);
    RUN_TEST(test_canfd_loopback,  13945, (void*)&testParams);
    test_canfd_set_params(&testParams, 13946);
    RUN_TEST(test_canfd_loopback,  13946, (void*)&testParams);
    test_canfd_set_params(&testParams, 13947);
    RUN_TEST(test_canfd_loopback,  13947, (void*)&testParams);
    test_canfd_set_params(&testParams, 13948);
    RUN_TEST(test_canfd_loopback,  13948, (void*)&testParams);
    test_canfd_set_params(&testParams, 13949);
    RUN_TEST(test_canfd_loopback,  13949, (void*)&testParams);
    test_canfd_set_params(&testParams, 13950);
    RUN_TEST(test_canfd_loopback,  13950, (void*)&testParams);
    test_canfd_set_params(&testParams, 13951);
    RUN_TEST(test_canfd_loopback,  13951, (void*)&testParams);
    test_canfd_set_params(&testParams, 13952);
    RUN_TEST(test_canfd_loopback,  13952, (void*)&testParams);
    // test_canfd_set_params(&testParams, 13953);
    // RUN_TEST(test_canfd_loopback,  13953, (void*)&testParams);
    test_canfd_set_params(&testParams, 13954);
    RUN_TEST(test_canfd_loopback,  13954, (void*)&testParams);
    test_canfd_set_params(&testParams, 13955);
    RUN_TEST(test_canfd_loopback,  13955, (void*)&testParams);
    // test_canfd_set_params(&testParams, 13956);
    // RUN_TEST(test_canfd_loopback,  13956, (void*)&testParams);
    test_canfd_set_params(&testParams, 13957);
    RUN_TEST(test_canfd_loopback,  13957, (void*)&testParams);
    test_canfd_set_params(&testParams, 13958);
    RUN_TEST(test_canfd_loopback,  13958, (void*)&testParams);
    test_canfd_set_params(&testParams, 13959);
    RUN_TEST(test_canfd_loopback,  13959, (void*)&testParams);
#if (MCAN_MANUAL_TEST_ENABLE == 1U)
    test_canfd_set_params(&testParams, 13960);
    RUN_TEST(test_canfd_loopback,  13960, (void*)&testParams);
    test_canfd_set_params(&testParams, 13958);
    RUN_TEST(test_canfd_loopback,  13958, (void*)&testParams);
    test_canfd_set_params(&testParams, 13959);
    RUN_TEST(test_canfd_loopback,  13959, (void*)&testParams);
#endif

    UNITY_END();

    Board_driversClose();
    Drivers_close();
}

static void test_canfd_loopback(void *args)
{
    int32_t          status = SystemP_SUCCESS;
    int32_t          testStatus = SystemP_SUCCESS;
    CANFD_Handle     canfdHandle;
    CANFD_Config          *config = &gCanfdConfig[CONFIG_MCAN0];
    CANFD_Attrs           *attrs  = (CANFD_Attrs *)config->attrs;
    CANFD_TestParams      *testParams = (CANFD_TestParams *)args;
    App_CANFD_TxMsgParams *txMsgParams = testParams->txMsgParams;
    CANFD_MsgObjHandle     txMsgObjHandle = &(testParams->txMsgObject);
    CANFD_MsgObjHandle     rxMsgObjHandle = &(testParams->rxMsgObject);
    CANFD_OpenParams      *canfdOpenParams = &(testParams->openParams);
    CANFD_MessageObject   *txMsgObject = &(testParams->txMsgObject);
    CANFD_MessageObject   *rxMsgObject = &(testParams->rxMsgObject); 

    /* Memset Buffers */
    memset(&txData[0U], 0, txMsgParams->dataLength * sizeof(txData[0U]));
    memset(&rxData[0U], 0, txMsgParams->dataLength * sizeof(rxData[0U]));

    CANFD_close(gCanfdHandle[CONFIG_MCAN0]);

    for(int8_t i = 0; i < txMsgParams->dataLength; i++)
    {
        txData[i] = txMsgParams->data[i];
    }

    canfdHandle = CANFD_open(CONFIG_MCAN0, canfdOpenParams);
    TEST_ASSERT_NOT_NULL(canfdHandle);

    status = SemaphoreP_constructBinary(&gMcanTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status = SemaphoreP_constructBinary(&gMcanRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Setup the transmit message object */
    txMsgObject->direction = CANFD_Direction_TX;
    txMsgObject->msgIdType = txMsgParams->msgIdType;
    txMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
    txMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
    txMsgObject->txMemType  = txMsgParams->txMemType;
    txMsgObject->dataLength = txMsgParams->dataLength;
    txMsgObject->args       = NULL;

    /* Setup the receive message object */
    rxMsgObject->direction = CANFD_Direction_RX;
    rxMsgObject->msgIdType = txMsgParams->msgIdType;;
    rxMsgObject->startMsgId = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid1;
    rxMsgObject->endMsgId   = ((MCAN_ExtMsgIDFilterElement*)attrs->filterConfig)->efid2;
    rxMsgObject->args       = (uint8_t*) rxData;
    rxMsgObject->rxMemType  = txMsgParams->rxMemType;
    rxMsgObject->dataLength = txMsgParams->dataLength;

    status = CANFD_createMsgObject (canfdHandle, txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Tx message object failed\r\n");
        return;
    }

    status = CANFD_createMsgObject (canfdHandle, rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Rx message object failed\r\n");
        return;
    }

    status += CANFD_read(rxMsgObjHandle, MCAN_APP_TEST_MESSAGE_COUNT, &rxData[0]);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD read in interrupt mode failed\r\n");
        return;
    }

    /* Send data over Tx message object */
    status = CANFD_write (txMsgObjHandle,
                          testParams->txMsgObject.startMsgId,
                          CANFD_MCANFrameType_FD,
                          0,
                          &txData[0]);

    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD write in interrupt mode failed\r\n");
        return;
    }

    /* Wait for Tx completion */
    SemaphoreP_pend(&gMcanTxDoneSem, SystemP_WAIT_FOREVER);
    /* Wait for Rx completion */
    SemaphoreP_pend(&gMcanRxDoneSem, SystemP_WAIT_FOREVER);

    /* Compare data */
    for(int32_t i = 0U; i < txMsgParams->dataLength; i++)
    {
        if(txData[i] != rxData[i])
        {
            testStatus = SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    CANFD_close(canfdHandle);

    status = CANFD_deleteMsgObject(txMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed\r\n");
        return;
    }

    status = CANFD_deleteMsgObject(rxMsgObjHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Rx message object failed\r\n");
        return;
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    return;
}

static int32_t App_compareData(uint8_t *txData, uint8_t *rxData)
{
    int32_t status = SystemP_SUCCESS;

    /* Compare data */
    for(int32_t i = 0U; i < MCAN_APP_TEST_DATA_SIZE; i++)
    {
        if(txData[i] != rxData[i])
        {
            status = SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    return status;
}

static void test_canfd_set_params(CANFD_TestParams *testParams, uint32_t tcId)
{
    CANFD_Config *config = &gCanfdConfig[CONFIG_MCAN0];
    CANFD_Attrs  *attrs  = (CANFD_Attrs *)config->attrs;
    CANFD_OpenParams *openParams = &(testParams->openParams);
    CANFD_MCANBitTimingParams *bitTimingParams = &(attrs->CANFDMcanBitTimingParams);
    MCAN_StdMsgIDFilterElement  stdMsgIdFilterEle; 

    /* Default Attribute Parameters */
    attrs->baseAddr                       = CONFIG_MCAN0_BASE_ADDR,
    attrs->intrNum0                       = CSLR_R5FSS0_CORE0_INTR_MCAN0_MCAN_LVL_INT_0,
    attrs->intrNum1                       = CSLR_R5FSS0_CORE0_INTR_MCAN0_MCAN_LVL_INT_1,
    attrs->intrPriority                   = 4U,
    attrs->operMode                       = CANFD_OPER_MODE_INTERRUPT,
    attrs->OptionTLVtype                  = CANFD_Option_MCAN_LOOPBACK,
    attrs->CANFDMcanloopbackParams.enable = true,
    attrs->CANFDMcanloopbackParams.mode   = CANFD_MCANLoopBackMode_INTERNAL,
    bitTimingParams->nomBrp          = 0x1U,
    bitTimingParams->nomPropSeg      = 0x1EU,
    bitTimingParams->nomPseg1        = 0x25U,
    bitTimingParams->nomPseg2        = 0xCU,
    bitTimingParams->nomSjw          = 0x1U,
    bitTimingParams->dataBrp         = 0x2U,
    bitTimingParams->dataPropSeg     = 0x3U,
    bitTimingParams->dataPseg1       = 0x3U,
    bitTimingParams->dataPseg2       = 0x1U,
    bitTimingParams->dataSjw         = 0x1U,

    /* Default Open Parameters */
    openParams->transferMode              = CANFD_TRANSFER_MODE_CALLBACK,
    openParams->transferCallbackFxn       = App_CANFD_TransferCallback,
    openParams->errorCallbackFxn          = App_CANFD_ErrorCallback,
    openParams->fdMode                    = true,
    openParams->brsEnable                 = true,
    openParams->txpEnable                 = false,
    openParams->efbi                      = false,
    openParams->pxhddisable               = false,
    openParams->darEnable                 = true,
    openParams->wkupReqEnable             = true,
    openParams->autoWkupEnable            = true,
    openParams->emulationEnable           = false,
    openParams->emulationFAck             = false,
    openParams->clkStopFAck               = false,
    openParams->wdcPreload                = MCAN_RWD_WDC_MAX,
    openParams->tdcEnable                 = true,
    /* Transmitter Delay Compensation parameters. */
    openParams->tdcConfig.tdcf            = 0x0AU,
    openParams->tdcConfig.tdco            = 0x06U,
    /* Initialize MCAN Config parameters.  */
    openParams->monEnable                 = false,
    openParams->asmEnable                 = false,
    openParams->tsPrescalar               = 0xFU,
    openParams->tsSelect                  = 0,
    openParams->timeoutSelect             = MCAN_TIMEOUT_SELECT_CONT,
    openParams->timeoutPreload            = 0xFFFFU,
    openParams->timeoutCntEnable          = false,
    /* Global Filter Configuration parameters. */
    openParams->filterConfig.rrfe         = true,
    openParams->filterConfig.rrfs         = true,
    openParams->filterConfig.anfe         = true,
    openParams->filterConfig.anfs         = true,
    /* Message RAM Configuration parameters. */
    openParams->msgRAMConfig.lss          = APP_MCAN_STD_ID_FILTER_NUM,
    openParams->msgRAMConfig.lse          = APP_MCAN_EXT_ID_FILTER_NUM,
    openParams->msgRAMConfig.txBufNum     = APP_MCAN_TX_BUFF_SIZE,
    openParams->msgRAMConfig.txFIFOSize              = 0U,
    openParams->msgRAMConfig.txBufMode               = 0U,
    openParams->msgRAMConfig.txEventFIFOSize         = APP_MCAN_TX_BUFF_SIZE,
    openParams->msgRAMConfig.txEventFIFOWaterMark    = 3U,
    openParams->msgRAMConfig.rxFIFO0size             = APP_MCAN_FIFO_0_NUM,
    openParams->msgRAMConfig.rxFIFO0waterMark        = 3U,
    openParams->msgRAMConfig.rxFIFO0OpMode           = 0U,
    openParams->msgRAMConfig.rxFIFO1size             = APP_MCAN_FIFO_1_NUM,
    openParams->msgRAMConfig.rxFIFO1waterMark        = 3U,
    openParams->msgRAMConfig.rxFIFO1OpMode           = 0U,
    /* ECC Configuration parameters. */
    openParams->eccConfig.enable           = true,
    openParams->eccConfig.enableChk        = true,
    openParams->eccConfig.enableRdModWr    = true,
    openParams->errInterruptEnable         = true,
    openParams->dataInterruptEnable        = true,

    testParams->txMsgNum     = 1U; /* tx message number */
    testParams->stdIdFiltNum = 2U; /* standard ID message filter number */
    testParams->extIdFiltNum = 2U; /* extended ID message filter number */

    if(openParams->fdMode == true)
    {
        attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*)(&canExtIdFilter[0]);
    }
    else
    {
        attrs->filterConfig = (MCAN_StdMsgIDFilterElement*)(&stdMsgIdFilterEle);
        stdMsgIdFilterEle.sfid1 = 0x29E;
        stdMsgIdFilterEle.sfec  = 0x7;
        stdMsgIdFilterEle.sfid2 = 0x29E;
        stdMsgIdFilterEle.sft   = 0x0;
    }

    switch (tcId)
    {
        case 13920:
            /* 1000kbps and 5000 kbps*/
            bitTimingParams->nomBrp          = 0x1U,
            bitTimingParams->nomPropSeg      = 0x1EU,
            bitTimingParams->nomPseg1        = 0x25U,
            bitTimingParams->nomPseg2        = 0xCU,
            bitTimingParams->nomSjw          = 0x1U,
            bitTimingParams->dataBrp         = 0x2U,
            bitTimingParams->dataPropSeg     = 0x3U,
            bitTimingParams->dataPseg1       = 0x3U,
            bitTimingParams->dataPseg2       = 0x1U,
            bitTimingParams->dataSjw         = 0x1U,
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = &canExtIdFilter[1U];  /* extended message ID filters */
            canExtIdFilter[1U].efec = canTxMsg[2].rxBuffNum;
            canExtIdFilter[1U].eft  = canTxMsg[2].rxfilterType;
            break;
        case 13921:
#if defined (SOC_AM261X)
            attrs->baseAddr = CONFIG_MCAN1_BASE_ADDR,
            attrs->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN1_MCAN_LVL_INT_0,
            attrs->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN1_MCAN_LVL_INT_1,
#else
            attrs->baseAddr = CONFIG_MCAN2_BASE_ADDR,
            attrs->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN2_MCAN_LVL_INT_0,
            attrs->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN2_MCAN_LVL_INT_1,
#endif

            /* mcan module bit timing parameters. 1000kbps and 2500 kbps */ //1
            bitTimingParams->nomBrp          = 0x1U,
            bitTimingParams->nomPropSeg      = 0x1EU,
            bitTimingParams->nomPseg1        = 0x25U,
            bitTimingParams->nomPseg2        = 0xCU,
            bitTimingParams->nomSjw          = 0x1U,
            bitTimingParams->dataBrp         = 0x2U,
            bitTimingParams->dataPropSeg     = 0x7U,
            bitTimingParams->dataPseg1       = 0x6U,
            bitTimingParams->dataPseg2       = 0x2U,
            bitTimingParams->dataSjw         = 0x1U,
            /* mcan module configuration parameters */ //2 done //same as default
            testParams->txMsgParams = &canTxMsg[7U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[7U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[7U].rxfilterType;
            break;
        case 13922:
#if defined (SOC_AM261X)
            attrs->baseAddr = CONFIG_MCAN1_BASE_ADDR,
            attrs->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN1_MCAN_LVL_INT_0,
            attrs->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN1_MCAN_LVL_INT_1,
#else
            attrs->baseAddr = CONFIG_MCAN3_BASE_ADDR,
            attrs->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN3_MCAN_LVL_INT_0,
            attrs->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN3_MCAN_LVL_INT_1,
#endif        
            /* mcan module bit timing parameters. 250kbps and 5000 kbps */ //2
            bitTimingParams->nomBrp          = 0x8U,
            bitTimingParams->nomPropSeg      = 0x11U,
            bitTimingParams->nomPseg1        = 0x11U,
            bitTimingParams->nomPseg2        = 0x5U,
            bitTimingParams->nomSjw          = 0x1U,
            bitTimingParams->dataBrp         = 0x2U,
            bitTimingParams->dataPropSeg     = 0x3U,
            bitTimingParams->dataPseg1       = 0x3U,
            bitTimingParams->dataPseg2       = 0x1U,
            bitTimingParams->dataSjw         = 0x1U,
            /* mcan module configuration parameters */ //2 done //same as default
            testParams->txMsgParams = &canTxMsg[7U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[7U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[7U].rxfilterType;
            break;
        case 13923:
            /* mcan module bit timing parameters. 125kbps and 5000 kbps */ //3
            bitTimingParams->nomBrp          = 0x8U,
            bitTimingParams->nomPropSeg      = 0x1EU,
            bitTimingParams->nomPseg1        = 0x27U,
            bitTimingParams->nomPseg2        = 0xAU,
            bitTimingParams->nomSjw          = 0x1U,
            bitTimingParams->dataBrp         = 0x2U,
            bitTimingParams->dataPropSeg     = 0x3U,
            bitTimingParams->dataPseg1       = 0x3U,
            bitTimingParams->dataPseg2       = 0x1U,
            bitTimingParams->dataSjw         = 0x1U,
            /* mcan module configuration parameters */ //2 done //same as default
            testParams->txMsgParams = &canTxMsg[7U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[7U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[7U].rxfilterType;
            break;
        case 13924:
            /* MCAN Loopback Extended Id Test */
            /* mcan module configuration parameters */ //1 done 
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[1U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[1U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[1U].rxfilterType;
            break;
        case 13925:
            testParams->txMsgNum     = 2U; /* tx message number */
            testParams->stdIdFiltNum = 1U; /* standard ID message filter number */
            testParams->extIdFiltNum = 1U; /* extended ID message filter number */
            testParams->txMsgParams = &canTxMsg[3U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[3U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[3U].rxfilterType;
            break;
        case 13926:
            testParams->txMsgNum     = 2U; /* tx message number */
            testParams->stdIdFiltNum = 1U; /* standard ID message filter number */
            testParams->extIdFiltNum = 1U; /* extended ID message filter number */
            testParams->txMsgParams = &canTxMsg[3U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[3U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[3U].rxfilterType;
            break;
        case 13927:
            /* mcan module configuration parameters */ //2 done //same as default
            /* mcan module MSG RAM configuration parameters */ //1 done
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufMode = 1U;

            testParams->txMsgNum     = 2U; /* tx message number */
            testParams->stdIdFiltNum = 2U; /* standard ID message filter number */
            testParams->extIdFiltNum = 2U; /* extended ID message filter number */
            testParams->txMsgParams = &canTxMsg[8U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[8U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[8U].rxfilterType;
            break;
        case 13928:
            /* mcan module configuration parameters */ //2 done //same as default
            testParams->txMsgNum     = 2U; /* tx message number */
            testParams->stdIdFiltNum = 2U; /* standard ID message filter number */
            testParams->extIdFiltNum = 2U; /* extended ID message filter number */
            testParams->txMsgParams = &canTxMsg[8U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[8U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[8U].rxfilterType;
            break;
        case 13929:
         /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13930:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13931:
            testParams->txMsgNum     = 3U; /* tx message number */
            testParams->stdIdFiltNum = 1U; /* standard ID message filter number */
            testParams->extIdFiltNum = 1U; /* extended ID message filter number */
            testParams->txMsgParams = &canTxMsg[5U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[5U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[5U].rxfilterType;
            break;
        case 13933:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            /* mcan module MSG RAM configuration parameters */ //2 done
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufMode = 1U;
            openParams->msgRAMConfig.rxFIFO0size   = 5U;
            openParams->msgRAMConfig.rxFIFO0OpMode = 1U;
            openParams->msgRAMConfig.rxFIFO1size   = 5U;
            openParams->msgRAMConfig.rxFIFO1OpMode = 1U;

            testParams->txMsgNum     = 1U; /* tx message number */
            testParams->stdIdFiltNum = 0U; /* standard ID message filter number */
            testParams->extIdFiltNum = 0U; /* extended ID message filter number */
            testParams->txMsgParams  = &canTxMsg[15U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[15U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[15U].rxfilterType;
            break;
        case 13934:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            /* mcan module MSG RAM configuration parameters */ //2 done
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufMode     = 1U;
            openParams->msgRAMConfig.rxFIFO0size   = 5U;
            openParams->msgRAMConfig.rxFIFO0OpMode = 1U;
            openParams->msgRAMConfig.rxFIFO1size   = 5U;
            openParams->msgRAMConfig.rxFIFO1OpMode = 1U;

            testParams->txMsgNum     = 1U; /* tx message number */
            testParams->stdIdFiltNum = 0U; /* standard ID message filter number */
            testParams->extIdFiltNum = 0U; /* extended ID message filter number */
            testParams->txMsgParams  = &canTxMsg[17U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[17U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[17U].rxfilterType;
            break;
        case 13935:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            /* mcan module MSG RAM configuration parameters */ //2 done
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufMode     = 1U;
            openParams->msgRAMConfig.rxFIFO0size   = 5U;
            openParams->msgRAMConfig.rxFIFO0OpMode = 1U;
            openParams->msgRAMConfig.rxFIFO1size   = 5U;
            openParams->msgRAMConfig.rxFIFO1OpMode = 1U;

            testParams->txMsgNum     = 1U; /* tx message number */
            testParams->stdIdFiltNum = 0U; /* standard ID message filter number */
            testParams->extIdFiltNum = 0U; /* extended ID message filter number */
            testParams->txMsgParams  = &canTxMsg[15U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[15U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[15U].rxfilterType;
            break;
        case 13936:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            /* mcan module MSG RAM configuration parameters */ //0 done
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13937:
            /* mcan module configuration parameters */ //2 done //same as default
            testParams->txMsgParams = &canTxMsg[18U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[18U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[18U].rxfilterType;
            break;
        case 13938:
            /* mcan module configuration parameters */ //2 done //same as default
            testParams->txMsgParams = &canTxMsg[19U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[3U];  /* extended message ID filters */
            canExtIdFilter[3U].efec = canTxMsg[19U].rxBuffNum;
            canExtIdFilter[3U].eft  = canTxMsg[19U].rxfilterType;
            break;
        case 13939:
            /* mcan module configuration parameters */ //2 done //same as default
            testParams->txMsgParams = &canTxMsg[7U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[7U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[7U].rxfilterType;
            break;
        case 13940:
            /* mcan module configuration parameters */ //2 done //same as default
            testParams->txMsgParams = &canTxMsg[0U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[0U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[0U].rxfilterType;
            break;
        case 13941:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect    = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13942:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect    = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13943:
            testParams->txMsgNum     = 2U; /* tx message number */
            testParams->stdIdFiltNum = 1U; /* standard ID message filter number */
            testParams->extIdFiltNum = 1U; /* extended ID message filter number */
            testParams->txMsgParams  = &canTxMsg[3U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[3U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[3U].rxfilterType;
            break;
        case 13944:
            testParams->txMsgNum     = 2U; /* tx message number */
            testParams->stdIdFiltNum = 1U; /* standard ID message filter number */
            testParams->extIdFiltNum = 1U; /* extended ID message filter number */
            testParams->txMsgParams  = &canTxMsg[3U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[3U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[3U].rxfilterType;
            break;
        case 13945:
            /* mcan module configuration parameters */ //2 done //same as default
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13946:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect    = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13947:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            /* mcan module MSG RAM configuration parameters */ //2 done
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufMode     = 1U;
            openParams->msgRAMConfig.rxFIFO0size   = 5U;
            openParams->msgRAMConfig.rxFIFO0OpMode = 1U;
            openParams->msgRAMConfig.rxFIFO1size   = 5U;
            openParams->msgRAMConfig.rxFIFO1OpMode = 1U;

            testParams->txMsgNum     = 1U; /* tx message number */
            testParams->stdIdFiltNum = 0U; /* standard ID message filter number */
            testParams->extIdFiltNum = 0U; /* extended ID message filter number */
            testParams->txMsgParams  = &canTxMsg[15U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[15U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[15U].rxfilterType;
            break;
        case 13948:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13949:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13950:
            /* mcan module bit timing parameters. 1000kbps and 2500 kbps */ //1
            bitTimingParams->nomBrp          = 0x1U,
            bitTimingParams->nomPropSeg      = 0x1EU,
            bitTimingParams->nomPseg1        = 0x25U,
            bitTimingParams->nomPseg2        = 0xCU,
            bitTimingParams->nomSjw          = 0x1U,
            bitTimingParams->dataBrp         = 0x2U,
            bitTimingParams->dataPropSeg     = 0x7U,
            bitTimingParams->dataPseg1       = 0x6U,
            bitTimingParams->dataPseg2       = 0x2U,
            bitTimingParams->dataSjw         = 0x1U,
            /* mcan module configuration parameters */ //2 done //same as default
            testParams->txMsgParams = &canTxMsg[7U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[7U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[7U].rxfilterType;
            break;
        case 13951:
            /* mcan module configuration parameters */ //3 done
            openParams->tsSelect = 2U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13952:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13953:
            /* mcan module bit timing parameters. */ //4
            /* mcan module bit timing parameters. 250 kbps and 5000 kbps */
            bitTimingParams->nomBrp          = 0x4U,
            bitTimingParams->nomPropSeg      = 0x22U,
            bitTimingParams->nomPseg1        = 0x22U,
            bitTimingParams->nomPseg2        = 0xBU,
            bitTimingParams->nomSjw          = 0x1U,
            bitTimingParams->dataBrp         = 0x2U,
            bitTimingParams->dataPropSeg     = 0x3U,
            bitTimingParams->dataPseg1       = 0x3U,
            bitTimingParams->dataPseg2       = 0x1U,
            bitTimingParams->dataSjw         = 0x1U,
            /* mcan module initialization parameters */ //1 done
            /* Transmitter Delay Compensation parameters. */
            openParams->tdcConfig.tdcf = MCAN_TDCR_TDCF_MAX + 1;
            openParams->tdcConfig.tdco = MCAN_TDCR_TDCO_MAX;
            /* mcan module configuration parameters */ //5 done
            openParams->monEnable = true;

            /* mcan module MSG RAM configuration parameters */ //3 done
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum          = 64,
            openParams->msgRAMConfig.txFIFOSize        = 64U;
            openParams->msgRAMConfig.txBufMode         = 1U;
            openParams->msgRAMConfig.txEventFIFOSize   = APP_MCAN_TX_BUFF_SIZE;
            openParams->msgRAMConfig.rxFIFO0size       = 5U;
            openParams->msgRAMConfig.rxFIFO0OpMode     = 1U;
            openParams->msgRAMConfig.rxFIFO1size       = 5U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13954:
            attrs->baseAddr = CONFIG_MCAN1_BASE_ADDR,
            attrs->intrNum0 = CSLR_R5FSS0_CORE0_INTR_MCAN1_MCAN_LVL_INT_0,
            attrs->intrNum1 = CSLR_R5FSS0_CORE0_INTR_MCAN1_MCAN_LVL_INT_1,
    
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13955:
            /* mcan module configuration parameters */ //2 done //same as default
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13956:
            /* mcan module bit timing parameters. */ //5
            /* mcan module bit timing parameters. 250kbps and 5000 kbps */ //5
            bitTimingParams->nomBrp          = 0x2U,
            bitTimingParams->nomPropSeg      = 0x46U,
            bitTimingParams->nomPseg1        = 0x49U,
            bitTimingParams->nomPseg2        = 0x10U,
            bitTimingParams->nomSjw          = 0x1U,
            bitTimingParams->dataBrp         = 0x2U,
            bitTimingParams->dataPropSeg     = 0x3U,
            bitTimingParams->dataPseg1       = 0x3U,
            bitTimingParams->dataPseg2       = 0x1U,
            bitTimingParams->dataSjw         = 0x1U,
            /* mcan module initialization parameters */ //1 done
            /* Transmitter Delay Compensation parameters. */
            openParams->tdcConfig.tdcf  = MCAN_TDCR_TDCF_MAX + 1;
            openParams->tdcConfig.tdco  = MCAN_TDCR_TDCO_MAX;
            /* mcan module configuration parameters */ //5 done
            openParams->monEnable                 = true;
            /* mcan module MSG RAM configuration parameters */ //3 done
            /* Message RAM Configuration parameters. */
            openParams->msgRAMConfig.txBufNum          = 64,
            openParams->msgRAMConfig.txFIFOSize        = 64U;
            openParams->msgRAMConfig.txBufMode         = 1U;
            openParams->msgRAMConfig.txEventFIFOSize   = APP_MCAN_TX_BUFF_SIZE;
            openParams->msgRAMConfig.rxFIFO0size       = 5U;
            openParams->msgRAMConfig.rxFIFO0OpMode     = 1U;
            openParams->msgRAMConfig.rxFIFO1size       = 5U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13957:
            /* mcan module configuration parameters */ //2 done //same as default
            testParams->txMsgParams = &canTxMsg[20U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13958:
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13959:
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
#if (MCAN_MANUAL_TEST_ENABLE == 1U)
        case 13960:
            /* mcan module configuration parameters */ //7
            openParams->monEnable         = true;
            openParams->tsSelect          = 0x1U,
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13958:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
        case 13959:
            /* mcan module configuration parameters */ //1 done
            openParams->tsSelect = 1U;
            testParams->txMsgParams = &canTxMsg[2U];
            attrs->filterConfig = (MCAN_ExtMsgIDFilterElement*) &canExtIdFilter[0U];  /* extended message ID filters */
            canExtIdFilter[0U].efec = canTxMsg[2U].rxBuffNum;
            canExtIdFilter[0U].eft  = canTxMsg[2U].rxfilterType;
            break;
#endif
        default:
            break;
    }

    return;
}

void setUp(void)
{
}

void tearDown(void)
{
}

static void App_CANFD_TransferCallback(void *args, CANFD_Reason reason)
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

static void App_CANFD_ErrorCallback(void *args, CANFD_Reason reason, CANFD_ErrStatusResp* errStatusResp)
{
    /* Do nothing. */
}
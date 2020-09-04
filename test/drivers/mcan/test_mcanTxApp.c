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
 *
 */
/**
 *  \file     test_mcanTxApp.c
 *
 *  \brief    This file contains mcan test code.
 *
 *  \details  mcan operational mode is set to normal mode for group 1.
 *            EVE reset interrupt is enabled from group 1 and
 *            it's priority is set as high level interrupt.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include "test_mcan.h"

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define APP_MCAN_STD_ID_SHIFT             (18U)
#define APP_MCAN_EXT_ID_SHIFT             (0U)
#define APP_MCAN_STD_ID_MASK              (0x7FFU << APP_MCAN_STD_ID_SHIFT)
#define APP_MCAN_EXT_ID_MASK              (0x1FFFFFFFU)

/**
 * \brief  Mask and shift for Tx Buffers elements.
 */
#define MCANSS_TX_BUFFER_ELEM_ID_SHIFT                           (0U)
#define MCANSS_TX_BUFFER_ELEM_ID_MASK                            (0x1FFFFFFFU)
#define MCANSS_TX_BUFFER_ELEM_RTR_SHIFT                          (29U)
#define MCANSS_TX_BUFFER_ELEM_RTR_MASK                           (0x20000000U)
#define MCANSS_TX_BUFFER_ELEM_XTD_SHIFT                          (30U)
#define MCANSS_TX_BUFFER_ELEM_XTD_MASK                           (0x40000000U)
#define MCANSS_TX_BUFFER_ELEM_ESI_SHIFT                          (31U)
#define MCANSS_TX_BUFFER_ELEM_ESI_MASK                           (0x80000000U)
#define MCANSS_TX_BUFFER_ELEM_DLC_SHIFT                          (16U)
#define MCANSS_TX_BUFFER_ELEM_DLC_MASK                           (0x000F0000U)
#define MCANSS_TX_BUFFER_ELEM_BRS_SHIFT                          (20U)
#define MCANSS_TX_BUFFER_ELEM_BRS_MASK                           (0x00100000U)
#define MCANSS_TX_BUFFER_ELEM_FDF_SHIFT                          (21U)
#define MCANSS_TX_BUFFER_ELEM_FDF_MASK                           (0x00200000U)
#define MCANSS_TX_BUFFER_ELEM_EFC_SHIFT                          (23U)
#define MCANSS_TX_BUFFER_ELEM_EFC_MASK                           (0x00800000U)
#define MCANSS_TX_BUFFER_ELEM_MM_SHIFT                           (24U)
#define MCANSS_TX_BUFFER_ELEM_MM_MASK                            (0xFF000000U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern uint32_t          gMcanBaseAddr;
extern uint32_t          gMcanAppdataSize[16];
extern volatile uint32_t gMcanECCIntrFlag;
extern MCAN_ECCErrStatus gMcaneccErr;
extern volatile uint32_t gMcanIsrIntr0Status;
extern volatile uint32_t gMcanIsrIntr1Status;
extern volatile uint32_t gMcanExtTSIntrFlag;
extern MCAN_BitTimingParams canFDBitTimings[];
extern MCAN_InitParams canFDInitParams[];
extern MCAN_ConfigParams canFDConfigParams[];
extern MCAN_MsgRAMConfigParams canFDRAMConfigParams[];
volatile uint32_t    rxBuffNum;
volatile uint32_t isrPrintEnable = (uint32_t)FALSE;
uint32_t objSize[8] = {4, 5, 6, 7, 8, 10, 14, 18};
uint32_t dataSize[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
/* 1 full CAN FD message = 18 words and put this in .data section */
uint32_t txBuffer[500U * 18U] __attribute__(( aligned(128), section(".data") )) = {1U};
uint32_t rxBuffer[500U * 18U] __attribute__(( aligned(128), section(".data") )) = {1U};
SemaphoreP_Object gTxDoneSem, gRxDoneSem;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
/**
 * \brief   This function will configure MCAN module
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
static int32_t App_mcanConfig(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function contains MCAN Tx Test
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
static int32_t App_mcanTxTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function contains MCAN Tx Test for performance test
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
int32_t App_mcanPerfTxTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function will check TC pass/fail criteria apart from Tx/Rx
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      Execution status.
 */
static int32_t App_mcanCheckTCResultsMisc(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function will check received message against transferred.
 *
 * \param   txMsg       Tx Message.
 * \param   rxMsg       Rx Message.
 *
 * \retval  status      Check status.
 */
static int32_t App_mcanTxRxMessageCheck(MCAN_TxBufElement txMsg,
                                        MCAN_RxBufElement rxMsg);

/**
 * \brief   This function will check received message against transferred.
 *
 * \param   rxMsg       Read Rx Message.
 * \param   status      Interrupt Status.
 *
 * \retval  status      Check status.
 */
static int32_t App_mcanReadRxMSG(MCAN_RxBufElement *rxMsg,
                                 uint32_t status);

/**
 * \brief   This function will configure receiver or other things depending on
 *          TC parameters.
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      Execution status.
 */
static int32_t App_mcanTCEntrySetup(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function will configure receiver or other things for next TC.
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      Execution status.
 */
static int32_t App_mcanTCExitSetup(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function will send a message to change the receiver side
 *          application baud-rate.
 *
 * \param   idx         'canFDBitTimings' array index.
 *
 * \retval  status      Execution status.
 */
static int32_t App_mcanChangeBaudrateMSG(uint32_t idx);

/**
 * \brief   This function will check transmitted message against Tx Event Entry.
 *
 * \param   txMsg       Tx Message.
 * \param   rxMsg       Tx Event Message.
 *
 * \retval  status      Check status.
 */
static int32_t App_mcanTxEventMessageCheck(MCAN_TxBufElement txMsg,
                                           MCAN_TxEventFIFOElement txEventMsg);

/**
 * \brief   This function contains MCAN ECC Test
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
static int32_t App_mcanECCTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function contains code for MCAN ECC Self Test/Diagnostic mode
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
static int32_t App_mcanECCSelfTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function used CAN message to DDR. CAN be used for any memory.
 *
 * \param   addr            Address where message to be written.
 * \param   elem            Tx Element.
 *
 * \retval  none.
 */
static void App_mcanWriteMem(uint32_t cnt, uint32_t addr, const MCAN_TxBufElement *elem);

/**
 * \brief   This function contains test code for MCAN State transition
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
static int32_t App_mcanStateTransnTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function contains test code for MCAN Ext Time Stamp Interrupt
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
static int32_t App_mcanExtTSIntrTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function contains test code for MCAN Rx/Tx state
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
static int32_t App_mcanTxRxPinStateTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function contains test code for MCAN Rx/Tx state
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
static int32_t App_mcanClkStpReqTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function contains test code for MCAN Rx/Tx state
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
static int32_t App_mcanTSRstTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function contains test code for MCAN Rx/Tx state
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
static int32_t App_mcanBusMonTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function contains test code for MCAN Latency measurement
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
static int32_t App_mcanLatencyTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function contains MCAN Tx Test and then Rx Test for performance
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
int32_t App_mcanPerfTxRxTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function contains MCAN Message Arbitration test
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */
int32_t App_mcanMsgArbTest(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function contains MCAN Rx FIFO Mode Tests
 *
 * \param   testParams  Test case parameters.
 *
 * \retval  status      configuration status.
 */

static int32_t App_mcanMsgCancelTest(st_mcanTestcaseParams_t *testParams);
int32_t App_mcanRxFIFOModes(st_mcanTestcaseParams_t *testParams);

extern int32_t App_mcanRegisterInterrupt();
extern int32_t App_mcanUnRegisterInterrupt();

extern void App_mcanIntr0ISR(void *handle);

extern void App_mcanIntr1ISR(void *handle);

extern void App_mcanECCIntrISR(void *handle);

extern void App_mcanTSIntrISR(void *handle);

extern uint32_t App_getBitConfigParamId(const MCAN_BitTimingParams *bitTimings);

extern void App_delayFunc(uint32_t timeout);

extern void App_mcanGetIntStatus(uint32_t baseAddr);

extern void App_mcanWriteMsg(uint32_t                 baseAddr,
                      uint32_t                 elemAddr,
                      const MCAN_TxBufElement *elem);

extern void App_mcanReadMsg(uint32_t           baseAddr,
                     uint32_t           elemAddr,
                     MCAN_RxBufElement *elem);
static void App_mcanInitTxElem(MCAN_TxBufElementNoCpy *txMsg);
static void App_mcanCompareBitTimeParams(MCAN_BitTimingParams *setPrms,
                MCAN_BitTimingParams *dstPrms);
static int32_t App_mcanTxTestBusMonitor(st_mcanTestcaseParams_t *testParams);
static int32_t App_mcanTxTestBusOff(st_mcanTestcaseParams_t *testParams);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t st_mcanTxApp_main(st_mcanTestcaseParams_t *testParams)
{
    int32_t                    configStatus = CSL_PASS;

    if ((testParams->testcaseId == 1635U) || (testParams->testcaseId == 1636U))
    {
        App_mcanNegativeTest(testParams);
    }
    else
    {
        configStatus += App_mcanRegisterInterrupt();
        if (CSL_PASS != configStatus)
        {
            DebugP_log("\nError in X-Bar Configuration...");
        }

        if (testParams->testcaseId == 1269)
        {
            gMcanBaseAddr = CONFIG_MCAN1_BASE_ADDR;
        }
        else
        {
            gMcanBaseAddr = DEF_MCAN_MODULE;
        }

#if defined (SOC_AM263X)
        if (testParams->testcaseId == 1235)
        {
            gMcanBaseAddr = CONFIG_MCAN2_BASE_ADDR;
        }
        else if (testParams->testcaseId == 1236)
        {
            gMcanBaseAddr = CONFIG_MCAN3_BASE_ADDR;
        }
        else
        {
            gMcanBaseAddr = DEF_MCAN_MODULE;
        }
#endif

        /* Reset MCAN Module */
        MCAN_reset(gMcanBaseAddr);
        MCAN_lpbkModeEnable(gMcanBaseAddr, MCAN_LPBK_MODE_INTERNAL, FALSE);
        while (MCAN_isInReset(gMcanBaseAddr) == (uint32_t)TRUE)
        {
        }
        configStatus += App_mcanConfig(testParams);
        if (CSL_PASS != configStatus)
        {
            DebugP_log("\nError in MCAN Configuration...");
        }

        configStatus += App_mcanTCEntrySetup(testParams);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nError in MCAN TC Entry Configuration...TC cannot be Run...");
        }
        else
        {
            switch (testParams->testcaseId)
            {
                case 10:
                    testParams->testResult = App_mcanECCTest(testParams);
                break;
                case 11:
                    testParams->testResult = App_mcanECCSelfTest(testParams);
                break;
                case 12:
                    testParams->testResult = App_mcanECCSelfTest(testParams);
                break;
                case 15:
                    testParams->testResult = App_mcanStateTransnTest(testParams);
                break;
                case 1633:
                    testParams->testResult = App_mcanExtTSIntrTest(testParams);
                break;
                case 1263:
                    testParams->testResult = App_mcanTxRxPinStateTest(testParams);
                break;
                case 1261:
                    testParams->testResult = App_mcanClkStpReqTest(testParams);
                break;
                case 1634:
                    testParams->testResult = App_mcanTSRstTest(testParams);
                break;
                case 20:
                    testParams->testResult = App_mcanBusMonTest(testParams);
                break;
                case 21:
                    testParams->testResult = App_mcanLatencyTest(testParams);
                break;
                case 22:
                    testParams->testResult = App_mcanPerfTxTest(testParams);
                break;
                case 2022:
                    testParams->testResult = App_mcanPerfTxRxTest(testParams);
                break;
                case 24:
                    testParams->testResult = App_mcanMsgArbTest(testParams);
                break;
                case 1248:
                case 1249:
                case 1250:
                case 25:
                    testParams->testResult = App_mcanRxFIFOModes(testParams);
                break;
                case 2023:
                    testParams->testResult = App_mcanPerfTxRxTest(testParams);
                break;
                case 1254:
                    testParams->testResult = App_mcanMsgCancelTest(testParams);
                break;
                case 1256:
                case 1257:
                    testParams->testResult = App_mcanTxTestBusOff(testParams);
                break;
                case 1259:
                    testParams->testResult = App_mcanTxTestBusMonitor(testParams);
                break;
                default:
                    testParams->testResult = App_mcanTxTest(testParams);
                break;
            }
            testParams->testResult += App_mcanCheckTCResultsMisc(testParams);
            configStatus += App_mcanTCExitSetup(testParams);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nError in MCAN TC Exit Configuration...");
            }
            configStatus += App_mcanUnRegisterInterrupt();
        }
    }
    return 0;
}

int32_t App_mcanNegativeTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t           configStatus = CSL_PASS;
    MCAN_RxFIFOStatus fifoStatus;
    MCAN_TxBufElement txMsg;
    MCAN_InitParams   initParams;
    MCAN_ConfigParams configParams;
    MCAN_BitTimingParams bitTimes;
    MCAN_ECCErrStatus eccErr;
    MCAN_ECCAggrRevisionId eccAggrRevId;
    MCAN_ECCWrapRevisionId eccWrapRevId;

    /* Reset MCAN Module */
    MCAN_reset(gMcanBaseAddr);
    while (MCAN_isInReset(gMcanBaseAddr) == (uint32_t)TRUE)
    {
    }
    /* wait for memory initialization to happen */
    while (FALSE == MCAN_isMemInitDone(gMcanBaseAddr))
    {}

    /* Put MCAN in SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(gMcanBaseAddr))
    {}
    /* Initialize MCAN module */
    configStatus = MCAN_init(gMcanBaseAddr, testParams->mcanConfigParams.initParams);
    DebugP_assert(configStatus == CSL_EFAIL);
    configStatus = MCAN_init(gMcanBaseAddr, &canFDInitParams[2U]);
    DebugP_assert(configStatus == CSL_EFAIL);
    configStatus = MCAN_init(gMcanBaseAddr, &canFDInitParams[3U]);
    DebugP_assert(configStatus == CSL_EFAIL);

    /* Configure MCAN module */
    configStatus = MCAN_config(gMcanBaseAddr, testParams->mcanConfigParams.configParams);
    DebugP_assert(configStatus == CSL_EFAIL);
    configStatus = MCAN_config(gMcanBaseAddr, &canFDConfigParams[6U]);
    DebugP_assert(configStatus == CSL_EFAIL);

    /* Configure Bit timings */
    configStatus = MCAN_setBitTime(gMcanBaseAddr, testParams->mcanConfigParams.bitTimings);
    DebugP_assert(configStatus == CSL_EFAIL);
    /* Configure Bit timings */
    configStatus = MCAN_setBitTime(gMcanBaseAddr, &canFDBitTimings[6U]);
    DebugP_assert(configStatus == CSL_EFAIL);
    configStatus = MCAN_setBitTime(gMcanBaseAddr, &canFDBitTimings[7U]);
    DebugP_assert(configStatus == CSL_EFAIL);
    configStatus = MCAN_setBitTime(gMcanBaseAddr, &canFDBitTimings[8U]);
    DebugP_assert(configStatus == CSL_EFAIL);
    configStatus = MCAN_setBitTime(gMcanBaseAddr, &canFDBitTimings[9U]);
    DebugP_assert(configStatus == CSL_EFAIL);
    configStatus = MCAN_setBitTime(gMcanBaseAddr, &canFDBitTimings[10U]);
    DebugP_assert(configStatus == CSL_EFAIL);
    configStatus = MCAN_setBitTime(gMcanBaseAddr, &canFDBitTimings[11U]);
    DebugP_assert(configStatus == CSL_EFAIL);

    configStatus = MCAN_msgRAMConfig(gMcanBaseAddr, &canFDRAMConfigParams[4U]);
    DebugP_assert(configStatus == CSL_EFAIL);

    /* Set Extended ID Mask */
    configStatus = MCAN_setExtIDAndMask(gMcanBaseAddr, MCAN_XIDAM_EIDM_MAX + 1U);
    DebugP_assert(configStatus == CSL_EFAIL);

    MCAN_writeMsgRam(gMcanBaseAddr,
                     MCAN_MEM_TYPE_FIFO + 1,
                     0,
                     NULL);
    MCAN_writeMsgRamNoCpy(gMcanBaseAddr,
                     MCAN_MEM_TYPE_FIFO + 1,
                     0,
                     NULL);
    MCAN_readMsgRam(gMcanBaseAddr,
                    MCAN_MEM_TYPE_BUF,
                    0U,
                    MCAN_RX_FIFO_NUM_1 + 1,
                    NULL);
    MCAN_readMsgRam(gMcanBaseAddr,
                    MCAN_MEM_TYPE_FIFO,
                    0U,
                    MCAN_RX_FIFO_NUM_1 + 1,
                    NULL);
    MCAN_readMsgRamNoCpy(gMcanBaseAddr,
                    MCAN_MEM_TYPE_BUF,
                    0U,
                    MCAN_RX_FIFO_NUM_1 + 1,
                    NULL);
    MCAN_readMsgRamNoCpy(gMcanBaseAddr,
                    MCAN_MEM_TYPE_FIFO,
                    0U,
                    MCAN_RX_FIFO_NUM_1 + 1,
                    NULL);

    MCAN_txBufAddReq(gMcanBaseAddr,
                     MCAN_TX_BUFFER_MAX_NUM + 1);
    fifoStatus.num = (uint32_t)MCAN_RX_FIFO_NUM_1 + 1;
    MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
    MCAN_writeRxFIFOAck(gMcanBaseAddr, MCAN_RX_FIFO_NUM_1 + 1, 0U);
    MCAN_writeRxFIFOAck(gMcanBaseAddr, MCAN_RX_FIFO_NUM_0, MCAN_RX_FIFO_0_MAX_NUM + 1);
    MCAN_writeRxFIFOAck(gMcanBaseAddr, MCAN_RX_FIFO_NUM_1, MCAN_RX_FIFO_1_MAX_NUM + 1);
    MCAN_txBufCancellationReq(gMcanBaseAddr, MCAN_TX_BUFFER_MAX_NUM + 1);
    MCAN_txBufTransIntrEnable(gMcanBaseAddr, MCAN_TX_BUFFER_MAX_NUM + 1, 0U);
    MCAN_txBufCancellationIntrEnable(gMcanBaseAddr, MCAN_TX_BUFFER_MAX_NUM + 1, TRUE);
    MCAN_txBufCancellationIntrEnable(gMcanBaseAddr, 0, FALSE);
    MCAN_writeTxEventFIFOAck(gMcanBaseAddr, MCAN_TX_BUFFER_MAX_NUM + 1);
    MCAN_extTSSetRawStatus(gMcanBaseAddr);

    MCAN_lpbkModeEnable(gMcanBaseAddr, MCAN_LPBK_MODE_EXTERNAL, FALSE);
    MCAN_lpbkModeEnable(gMcanBaseAddr, MCAN_LPBK_MODE_EXTERNAL, TRUE);
    MCAN_lpbkModeEnable(gMcanBaseAddr, MCAN_LPBK_MODE_INTERNAL, FALSE);
    MCAN_initTxBufElement(NULL);
    MCAN_initTxBufElement(&txMsg);
    MCAN_initOperModeParams(NULL);
    MCAN_initOperModeParams(&initParams);
    MCAN_initGlobalFilterConfigParams(NULL);
    MCAN_initGlobalFilterConfigParams(&configParams);
    MCAN_initSetBitTimeParams(NULL);
    MCAN_initSetBitTimeParams(&bitTimes);
    MCAN_initMsgRamConfigParams(NULL);
    MCAN_initMsgRamConfigParams(testParams->mcanConfigParams.ramConfig);
    MCAN_calcMsgRamParamsStartAddr(NULL);
    MCAN_calcMsgRamParamsStartAddr(&canFDRAMConfigParams[5U]);

    /* Enable ECC Interrupts */
    MCAN_eccAggrGetRevisionId(gMcanBaseAddr, &eccAggrRevId);
    MCAN_eccWrapGetRevisionId(gMcanBaseAddr, &eccWrapRevId);
    MCAN_eccEnableIntr(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_SEC, TRUE);
    MCAN_eccEnableIntr(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED, TRUE);
    MCAN_eccEnableIntr(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED + 1, TRUE);
    MCAN_eccConfig(gMcanBaseAddr, testParams->mcanConfigParams.eccConfigParams);
    MCAN_eccForceError(gMcanBaseAddr, testParams->mcanConfigParams.eccFrcParams);
    testParams->mcanConfigParams.eccFrcParams->errType = MCAN_ECC_ERR_TYPE_DED;
    MCAN_eccForceError(gMcanBaseAddr, testParams->mcanConfigParams.eccFrcParams);
    MCAN_eccWriteEOI(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_SEC);
    MCAN_eccWriteEOI(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED);
    MCAN_eccWriteEOI(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED + 1);
    MCAN_eccClearErrorStatus(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_SEC);
    MCAN_eccClearErrorStatus(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED);
    MCAN_eccClearErrorStatus(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED + 1);
    MCAN_eccGetErrorStatus(gMcanBaseAddr, &eccErr);
    MCAN_eccEnableIntr(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_SEC, FALSE);
    MCAN_eccEnableIntr(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED, FALSE);
    MCAN_eccEnableIntr(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED + 1, FALSE);

    return configStatus;
}

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

void App_mcanWriteMem(uint32_t cnt, uint32_t addr, const MCAN_TxBufElement *elem)
{
    uint32_t regVal;

    uint32_t loopCnt = 0U;
    loopCnt = 0U;
    HW_WR_REG32(addr, cnt);
    addr += 4U;
    loopCnt += 4U;
    while(loopCnt < dataSize[elem->dlc])
    {
        regVal  = 0U;
        regVal |= elem->data[(loopCnt / 4U)];
        regVal |= (elem->data[(loopCnt / 4U) + 1U] << 8U);
        regVal |= (elem->data[(loopCnt / 4U) + 2U] << 16U);
        regVal |= (elem->data[(loopCnt / 4U) + 3U] << 24U);
        HW_WR_REG32(addr, regVal);
        addr += 4U;
        loopCnt += 4U;
    }
}

static int32_t App_mcanConfig(st_mcanTestcaseParams_t *testParams)
{
    uint32_t                   fdoe, loopCnt, extMask = 0U;
    int32_t                    configStatus = CSL_PASS;
    MCAN_RevisionId            revId;
    MCAN_BitTimingParams bitTimes;

    /* Reset MCAN SS */
    /* Get MCANSS Revision ID */
    MCAN_getRevisionId(gMcanBaseAddr, &revId);
    /* Enable Auto wakeup */
    fdoe = MCAN_isFDOpEnable(gMcanBaseAddr);
    (void)fdoe; /* Kill warning. Presently set but not used */
    /* wait for memory initialization to happen */
    while (FALSE == MCAN_isMemInitDone(gMcanBaseAddr))
    {}
    /* Get endianess value */
    DebugP_assert(MCAN_getEndianVal(gMcanBaseAddr) == 0x87654321U);
    /* Put MCAN in SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(gMcanBaseAddr))
    {}
    /* Initialize MCAN module */
    configStatus += MCAN_init(gMcanBaseAddr, testParams->mcanConfigParams.initParams);
    if(configStatus != CSL_PASS)
    {
        DebugP_log("\nMCAN Initialization FAILED...\n", -1);
    }
    /* Configure MCAN module */
    configStatus += MCAN_config(gMcanBaseAddr, testParams->mcanConfigParams.configParams);
    if(configStatus != CSL_PASS)
    {
        DebugP_log("\nMCAN Configuration FAILED...\n", -1);
    }
    /* Configure Bit timings */
    configStatus += MCAN_setBitTime(gMcanBaseAddr, testParams->mcanConfigParams.bitTimings);
    if(configStatus != CSL_PASS)
    {
        DebugP_log("\nMCAN Bit Time Configuration FAILED...\n", -1);
    }
    MCAN_getBitTime(gMcanBaseAddr, &bitTimes);

    App_mcanCompareBitTimeParams(testParams->mcanConfigParams.bitTimings, &bitTimes);

    /* Set Extended ID Mask */
    configStatus += MCAN_setExtIDAndMask(gMcanBaseAddr, 0x1FFFFFFFU);
    if(configStatus != CSL_PASS)
    {
        DebugP_log("\nMCAN Bit Time Configuration FAILED...\n", -1);
    }
    extMask = MCAN_getExtIDANDMassk(gMcanBaseAddr);
    DebugP_assert(extMask == 0x1FFFFFFFU);

    /* Configure Message RAM Sections */
    configStatus += MCAN_calcMsgRamParamsStartAddr(testParams->mcanConfigParams.ramConfig);
    if(configStatus != CSL_PASS)
    {
        DebugP_log("\nMCAN Message RAM Addr Calculation FAILED...\n", -1);
    }

    configStatus += MCAN_msgRAMConfig(gMcanBaseAddr, testParams->mcanConfigParams.ramConfig);
    if(configStatus != CSL_PASS)
    {
        DebugP_log("\nMCAN Message RAM Configuration FAILED...\n", -1);
    }

    for(loopCnt = 0U ; loopCnt < testParams->mcanConfigParams.stdIdFiltNum; loopCnt++)
    {
        /* Configure Standard ID filter element */
        MCAN_addStdMsgIDFilter(gMcanBaseAddr, loopCnt, &testParams->mcanConfigParams.stdIDFilter[loopCnt]);
    }
    for(loopCnt = 0U ; loopCnt < testParams->mcanConfigParams.extIdFiltNum; loopCnt++)
    {
        /* Configure Standard ID filter element */
        MCAN_addExtMsgIDFilter(gMcanBaseAddr, loopCnt, &testParams->mcanConfigParams.extIDFilter[loopCnt]);
    }
    /* Configure ECC */
    MCAN_eccConfig(gMcanBaseAddr, testParams->mcanConfigParams.eccConfigParams);
    /* Enable loopback if autoRunEnable is set/TRUE */
    /* Enable loopback only if internal loopback is set as test mode */
    if (testParams->mcanConfigParams.mcanTestType == MCAN_TEST_TYPE_INTERNAL_LOOBACK)
    {
        MCAN_lpbkModeEnable(gMcanBaseAddr, MCAN_LPBK_MODE_INTERNAL, TRUE);
    }
    /* Take MCAN out of the SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanBaseAddr))
    {}
    return configStatus;
}

static int32_t App_mcanTxTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t  configStatus = CSL_PASS, testStatus = CSL_PASS;
    uint32_t loopCnt      = 1U, iterationCnt = 0U, intrMask, txBufCnt;
    uint32_t bitPos = 0U, txStatus = 0U;
    MCAN_ProtocolStatus protStatus;
    MCAN_ErrCntStatus    errCounter;
    MCAN_RxBufElement rxMsg;
    MCAN_TxFIFOStatus txFIFOStatus;
    MCAN_HighPriorityMsgInfo highPriorityMsgStatus;

    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, testParams->mcanConfigParams.intrEnable, (uint32_t)TRUE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLineSelectMask,
                        testParams->mcanConfigParams.intrLine);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLine,
                        1U);
    /* Enable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)TRUE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n", -1);
            }
        }
    }
    /* Enable interrupts for Tx FIFO/Queue */
    for (loopCnt = testParams->mcanConfigParams.ramConfig->txBufCnt ;
         loopCnt < (testParams->mcanConfigParams.ramConfig->txFIFOCnt +
                    testParams->mcanConfigParams.ramConfig->txBufCnt);
         loopCnt++)
    {
        /* Enable Transmission interrupt */
        configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                loopCnt,
                (uint32_t)TRUE);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nMCAN Tx FIFO Interrupt Enable FAILED...\n", -1);
        }
    }

    for (iterationCnt = 0U ;
         iterationCnt < testParams->mcanConfigParams.txMSGInterationCnt ;
         iterationCnt++)
    {
        for (loopCnt = 0U ;
             loopCnt < testParams->mcanConfigParams.txMsgNum ;
             loopCnt++)
        {
            MCAN_getTxFIFOQueStatus(gMcanBaseAddr, &txFIFOStatus);
            if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
            {
                txBufCnt = testParams->mcanConfigParams.txMsg[loopCnt].bufferNum;
            }
            else
            {
                txBufCnt = txFIFOStatus.putIdx;
            }
            /* Calculate Rx Interrupt Mask */
            if(testParams->mcanConfigParams.txMsg[loopCnt].rxMSGStorageId == MCAN_MEM_TYPE_BUF)
            {
                intrMask = MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG;
            }
            else
            {
                if(testParams->mcanConfigParams.txMsg[loopCnt].rxBuffNum == MCAN_RX_FIFO_NUM_0)
                {
                    intrMask = MCAN_INTR_SRC_RX_FIFO0_NEW_MSG;
                }
                else
                {
                    intrMask = MCAN_INTR_SRC_RX_FIFO1_NEW_MSG;
                }

            }
            switch (testParams->testcaseId)
            {
                case 1252:
                    intrMask = MCAN_INTR_SRC_HIGH_PRIO_MSG;
                break;
                default:
                break;
            }
            // App_mcanPrintTxMsg(&testParams->mcanConfigParams.txMsg[loopCnt].txElem);
            /* Write message to Msg RAM */
            MCAN_writeMsgRam(gMcanBaseAddr,
                             testParams->mcanConfigParams.txMsg[loopCnt].storageId,
                             txBufCnt,
                             &testParams->mcanConfigParams.txMsg[loopCnt].txElem);
            /* Add request for transmission */
            configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
            if (CSL_PASS != configStatus)
            {
                DebugP_log("\nError in Adding Transmission Request...\n", -1);
            }
            while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_TRANS_COMPLETE) ==
                                MCAN_INTR_SRC_TRANS_COMPLETE))
            {
            }
            while (!((gMcanIsrIntr0Status & intrMask) == intrMask))
            {
            }
            bitPos = (1U << txBufCnt);
            /* Poll for Tx completion */
            do
            {
                txStatus = MCAN_getTxBufTransmissionStatus(gMcanBaseAddr);
            }while((txStatus & bitPos) != bitPos);
            /* Checking for Errors */
            MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
            if ((0U == errCounter.recErrCnt) &&
                (0U == errCounter.canErrLogCnt))
            {
                MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
                /* Checking for Errors */
                if (((MCAN_ERR_CODE_NO_ERROR == protStatus.lastErrCode) ||
                     (MCAN_ERR_CODE_NO_CHANGE == protStatus.lastErrCode)) &&
                    ((MCAN_ERR_CODE_NO_ERROR == protStatus.dlec) ||
                     (MCAN_ERR_CODE_NO_CHANGE == protStatus.dlec)) &&
                    (0U == protStatus.pxe))
                {
                    configStatus += App_mcanReadRxMSG(&rxMsg, gMcanIsrIntr0Status);
                    if(configStatus != CSL_PASS)
                    {
                        DebugP_log("\nUnable to read received message(Iteration Count:Message Number): (%d,%d).\n", (iterationCnt + 1U), (loopCnt + 1U));
                    }
                    if (testParams->testcaseId == 1252)
                    {
                        MCAN_getHighPriorityMsgStatus(gMcanBaseAddr, &highPriorityMsgStatus);
                        /* Message should be stored in FIFO 0 according to the configuration */
                        DebugP_assert(highPriorityMsgStatus.msi == 2U);
                    }
                    configStatus += App_mcanTxRxMessageCheck(
                                    testParams->mcanConfigParams.txMsg[loopCnt].txElem,
                                    rxMsg);
                    /* Check if Message is stored into appropriate Rx Buffer */
                    if(testParams->mcanConfigParams.txMsg[loopCnt].rxMSGStorageId == MCAN_MEM_TYPE_BUF)
                    {
                        if(rxBuffNum != testParams->mcanConfigParams.txMsg[loopCnt].rxBuffNum)
                        {
                           testStatus += CSL_EFAIL;
                        }
                    }
                    if(configStatus != CSL_PASS)
                    {
                        testStatus += CSL_EFAIL;
                        DebugP_log("\nTransmitted and received message does not match(Iteration Count:Message Number): (%d,%d).\n", (iterationCnt + 1U), (loopCnt + 1U));
                    }
                    else
                    {
                        testStatus += CSL_PASS;
                        DebugP_log(
                            "\nMessage successfully transferred/received(Iteration Count:Message Number): (%d,%d).\n",
                            (iterationCnt + 1U), (loopCnt + 1U));
                    }
                }
                else
                {
                    testStatus += CSL_EFAIL;
                    DebugP_log("\nError in transmission/reception(Iteration Count:Message Number): (%d,%d).\n",
                               (iterationCnt + 1U), (loopCnt + 1U));
                }
            }
            else
            {
                testStatus += CSL_EFAIL;
                DebugP_log("\nError Counters: Error in transmission/reception(Iteration Count:Message Number): (%d,%d).\n",
                           (iterationCnt + 1U), (loopCnt + 1U));
            }
            gMcanIsrIntr0Status = 0U;
        }
    }
    /* Disable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)FALSE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n", -1);
            }
        }
    }

    return testStatus;
}

static int32_t App_mcanReadRxMSG(MCAN_RxBufElement *rxMsg,
                                 uint32_t status)
{
    uint32_t    readBuffNum, bitPos;
    MCAN_RxFIFOStatus fifoStatus;
    MCAN_RxNewDataStatus newDataStatus;
    int32_t retStatus = CSL_EFAIL;

    if((status & MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG) ==
                                    MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG)
    {
        MCAN_getNewDataStatus(gMcanBaseAddr, &newDataStatus);
        for(readBuffNum = 0U ; readBuffNum < 64U ; readBuffNum++)
        {
            if(readBuffNum < 32U)
            {
                status = newDataStatus.statusLow;
                bitPos = 1U << readBuffNum;
                newDataStatus.statusLow = (0U | bitPos);
            }
            else
            {
                status = newDataStatus.statusHigh;
                bitPos = 1U << (readBuffNum - 32U);
                newDataStatus.statusHigh = (0U | bitPos);
            }
            MCAN_clearNewDataStatus(gMcanBaseAddr, &newDataStatus);

            rxBuffNum = readBuffNum;
            if(bitPos == (status & bitPos))
            {
                MCAN_readMsgRam(gMcanBaseAddr,
                                MCAN_MEM_TYPE_BUF,
                                readBuffNum,
                                MCAN_RX_FIFO_NUM_0,
                                rxMsg);
                if (isrPrintEnable == (uint32_t)TRUE)
                {
                    DebugP_log( "\nRx Buffer: Received message with following details:");
                }
                retStatus = CSL_PASS;
                break;
            }
        }
    }
    else if(((status & MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ==
                                    MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ||
        ((status & MCAN_INTR_SRC_RX_FIFO1_NEW_MSG) ==
                                    MCAN_INTR_SRC_RX_FIFO1_NEW_MSG))
    {
        retStatus = CSL_PASS;
        readBuffNum = 0U;
        if(((status & MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ==
                                    MCAN_INTR_SRC_RX_FIFO0_NEW_MSG))
        {
            fifoStatus.num = (uint32_t)MCAN_RX_FIFO_NUM_0;
            if (isrPrintEnable == (uint32_t)TRUE)
            {
                DebugP_log( "\nRx FIFO 0: Received message with following details:");
            }
        }
        else
        {
            fifoStatus.num = (uint32_t)MCAN_RX_FIFO_NUM_1;
            if (isrPrintEnable == (uint32_t)TRUE)
            {
                DebugP_log( "\nRx FIFO 1: Received message with following details:");
            }
        }
        MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
        MCAN_readMsgRam(gMcanBaseAddr,
                        MCAN_MEM_TYPE_FIFO,
                        fifoStatus.getIdx,
                        (uint32_t)fifoStatus.num,
                        rxMsg);
        (void) MCAN_writeRxFIFOAck(gMcanBaseAddr,
                                   (uint32_t)fifoStatus.num,
                                   fifoStatus.getIdx);
    }

    return retStatus;
}

static int32_t App_mcanTxRxMessageCheck(MCAN_TxBufElement txMsg,
                                        MCAN_RxBufElement rxMsg)
{
    uint32_t msgMask, loopCnt;
    int32_t retStatus = CSL_PASS;

    /* Check/Compare Rx message with Tx message */
    if(txMsg.xtd == 1U)
    {
        msgMask = APP_MCAN_EXT_ID_MASK;
    }
    else
    {
        msgMask = APP_MCAN_STD_ID_MASK;
    }
    if (((txMsg.id & msgMask) == (rxMsg.id & msgMask)) &&
        (txMsg.rtr == rxMsg.rtr) &&
        (txMsg.xtd == rxMsg.xtd) &&
        (txMsg.esi == rxMsg.esi) &&
        (txMsg.dlc == rxMsg.dlc) &&
        (txMsg.brs == rxMsg.brs) &&
        (txMsg.fdf == rxMsg.fdf))
    {
        for (loopCnt = 0U;
             loopCnt < gMcanAppdataSize[rxMsg.dlc];
             loopCnt++)
        {
            if (txMsg.data[loopCnt] != rxMsg.data[loopCnt])
            {
                break;
            }
        }
        if(loopCnt == gMcanAppdataSize[rxMsg.dlc])
        {
            retStatus = CSL_PASS;
        }
        else
        {
            retStatus = CSL_EFAIL;
        }
    }
    else
    {
        retStatus = CSL_EFAIL;
    }

    return retStatus;
}

static int32_t App_mcanTCEntrySetup(st_mcanTestcaseParams_t *testParams)
{
    int32_t status = CSL_PASS, configStatus = CSL_PASS;
    uint32_t    configId;

    configId = App_getBitConfigParamId(testParams->mcanConfigParams.bitTimings);
    /* send a message to change baud-rate of receiver and this is valid only for
       B2B and not for loop back */
    if((configId != 0U) && (configId != 1U) && (configId != 2U) &&
        (configId != 3U) && (configId != 0xFFFFFFFFU))
    {
        /* change Tx baud-rate to default */
        /* Put MCAN in SW initialization mode */
        MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_SW_INIT);
        while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(gMcanBaseAddr))
        {}
        /* Configure Bit timings */
        configStatus += MCAN_setBitTime(gMcanBaseAddr, &canFDBitTimings[0U]);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nDEFAULT MCAN Bit Time Configuration FAILED...\n", -1);
        }
        /* Take MCAN out of the SW initialization mode */
        MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
        while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanBaseAddr))
        {}
        status += App_mcanChangeBaudrateMSG(configId);
        /* change Tx baud-rate to configured by TC */
        /* Put MCAN in SW initialization mode */
        MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_SW_INIT);
        while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(gMcanBaseAddr))
        {}
        /* Configure Bit timings */
        configStatus += MCAN_setBitTime(gMcanBaseAddr, &canFDBitTimings[configId]);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nCONFIGURRED MCAN Bit Time Configuration FAILED...\n", -1);
        }
        /* Take MCAN out of the SW initialization mode */
        MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
        while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanBaseAddr))
        {}
    }
    switch (testParams->testcaseId)
    {
        case 1633:
            MCAN_extTSCounterConfig(gMcanBaseAddr, 0xFF);
            MCAN_extTSCounterEnable(gMcanBaseAddr, (uint32_t)TRUE);
        break;
        case 16:
            MCAN_extTSCounterConfig(gMcanBaseAddr, 0x3FFF);
            MCAN_extTSCounterEnable(gMcanBaseAddr, (uint32_t)TRUE);
        break;
        case 1003:
            MCAN_extTSCounterConfig(gMcanBaseAddr, 0xFFFF);
            MCAN_extTSCounterEnable(gMcanBaseAddr, (uint32_t)TRUE);
        break;
        default:
        break;
    }

    return status;
}

static int32_t App_mcanTCExitSetup(st_mcanTestcaseParams_t *testParams)
{
    int32_t status = CSL_PASS;
    uint32_t    configId;

    configId = App_getBitConfigParamId(testParams->mcanConfigParams.bitTimings);
    /* send a message to change baud-rate of receiver and this is valid only for
       B2B and not for loop back */
    if((configId != 0U) && (configId != 1U) && (configId != 2U) &&
        (configId != 3U) && (configId != 0xFFFFFFFFU))
    {
        status += App_mcanChangeBaudrateMSG(0U);

    }

    return status;
}

static int32_t App_mcanChangeBaudrateMSG(uint32_t idx)
{
    int32_t configStatus = CSL_PASS;
    MCAN_RxBufElement    rxMsg = {0};
    MCAN_TxBufElement    txMsg;
    MCAN_RxNewDataStatus newDataStatus;
    MCAN_ErrCntStatus    errCounter;
    MCAN_ProtocolStatus protStatus;
    uint32_t    readBuffNum, bitPos, status;
    MCAN_RxFIFOStatus fifoStatus;

    DebugP_log( "\n===============Sending Change Baud-rate message to Receiver===============\n");
    /* Message will be sent using Tx buffer 1 */
    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, MCAN_INTR_MASK_ALL, (uint32_t)TRUE);
    MCAN_enableIntr(gMcanBaseAddr,
                    MCAN_INTR_SRC_RES_ADDR_ACCESS, (uint32_t)FALSE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        MCAN_INTR_MASK_ALL,
                        MCAN_INTR_LINE_NUM_1);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        MCAN_INTR_LINE_NUM_1,
                        1U);
    if(idx == 0U)
    {
        txMsg.id =  (0x16 << 18);
    }
    else
    {
        txMsg.id =  (0x44 << 18);
    }
    txMsg.rtr = 0U;
    txMsg.xtd = 0U;
    txMsg.esi = 0U;
    txMsg.dlc = 1U;
    txMsg.brs = 1U;
    txMsg.fdf = 1U;
    txMsg.efc = 0U;
    txMsg.mm = 0xAAU;
    txMsg.data[0U] = (uint8_t)(idx & 0xFFU);
    /* Enable Transmission interrupt */
    configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                                             1U,
                                             (uint32_t)TRUE);
    if(configStatus != CSL_PASS)
    {
         DebugP_log("\nError in enabling Transmission Interrupt...\n", -1);
    }
    /* Always writing to Tx Buffer number 1 for echoing back */
    MCAN_writeMsgRam(gMcanBaseAddr,
                     MCAN_MEM_TYPE_BUF,
                     1U,
                     (const MCAN_TxBufElement*) &txMsg);
    /* Add request for transmission */
    configStatus += MCAN_txBufAddReq(gMcanBaseAddr, 1U);
    if (CSL_PASS != configStatus)
    {
        DebugP_log("\nError in Adding Transmission Request...\n", -1);
    }
    while (!(MCAN_INTR_SRC_TRANS_COMPLETE ==
            (gMcanIsrIntr1Status & MCAN_INTR_SRC_TRANS_COMPLETE)))
    {
    }
    while (!((((gMcanIsrIntr1Status & MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG) ==
                        MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG) ||
             ((gMcanIsrIntr1Status & MCAN_INTR_SRC_RX_FIFO1_NEW_MSG) ==
                        MCAN_INTR_SRC_RX_FIFO1_NEW_MSG) ||
             ((gMcanIsrIntr1Status & MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ==
                        MCAN_INTR_SRC_RX_FIFO0_NEW_MSG))))
    {
    }
    /* Checking for Errors */
    MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
    if ((0U != errCounter.recErrCnt) &&
        (0U != errCounter.canErrLogCnt))
    {
        configStatus += CSL_EFAIL;
        DebugP_log("\nError Counters: Error in transmission/reception\n");
    }
    MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
    /* Checking for Errors */
    if (((MCAN_ERR_CODE_NO_ERROR == protStatus.lastErrCode) ||
         (MCAN_ERR_CODE_NO_CHANGE == protStatus.lastErrCode)) &&
        ((MCAN_ERR_CODE_NO_ERROR == protStatus.dlec) ||
         (MCAN_ERR_CODE_NO_CHANGE == protStatus.dlec)) &&
        (0U == protStatus.pxe))
    {
        DebugP_log("\nMessage successfully transferred.\n");
    }
    else
    {
        configStatus += CSL_EFAIL;
        DebugP_log("\nError in transmission message.\n");
    }
    if((gMcanIsrIntr1Status & MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG) ==
                                MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG)
    {
        MCAN_getNewDataStatus(gMcanBaseAddr, &newDataStatus);
        MCAN_clearNewDataStatus(gMcanBaseAddr, &newDataStatus);
        for(readBuffNum = 0U ; readBuffNum < 64U ; readBuffNum++)
        {
            if(readBuffNum < 32U)
            {
                status = newDataStatus.statusLow;
                bitPos = 1U << readBuffNum;
            }
            else
            {
                status = newDataStatus.statusHigh;
                bitPos = 1U << (readBuffNum - 32U);
            }

            if(bitPos == (status & bitPos))
            {
                MCAN_readMsgRam(gMcanBaseAddr,
                                MCAN_MEM_TYPE_BUF,
                                readBuffNum,
                                MCAN_RX_FIFO_NUM_0,
                                &rxMsg);
                DebugP_log( "\nRx Buffer: Received message.");
            }
        }
    }
    else if(((gMcanIsrIntr1Status & MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ==
                                    MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ||
        ((gMcanIsrIntr1Status & MCAN_INTR_SRC_RX_FIFO1_NEW_MSG) ==
                                    MCAN_INTR_SRC_RX_FIFO1_NEW_MSG))
    {
        readBuffNum = 0U;
        if(((gMcanIsrIntr1Status & MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ==
                                    MCAN_INTR_SRC_RX_FIFO0_NEW_MSG))
        {
            fifoStatus.num = (uint32_t)MCAN_RX_FIFO_NUM_0;
            DebugP_log( "\nRx FIFO 0: Received message.");

        }
        else
        {
            fifoStatus.num = (uint32_t)MCAN_RX_FIFO_NUM_1;
            DebugP_log( "\nRx FIFO 1: Received message.");
        }
        MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
        MCAN_readMsgRam(gMcanBaseAddr,
                        MCAN_MEM_TYPE_FIFO,
                        fifoStatus.getIdx,
                        (uint32_t)fifoStatus.num,
                        &rxMsg);
        (void) MCAN_writeRxFIFOAck(gMcanBaseAddr,
                                   (uint32_t)fifoStatus.num,
                                   fifoStatus.getIdx);

    }
    configStatus += App_mcanTxRxMessageCheck(txMsg, rxMsg);

    /* Revert configuration done by this  function to send the baud-rate change message */
    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, MCAN_INTR_MASK_ALL, (uint32_t)FALSE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        MCAN_INTR_MASK_ALL,
                        MCAN_INTR_LINE_NUM_0);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        MCAN_INTR_LINE_NUM_1,
                        0U);
    /* Enable Transmission interrupt */
    configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                                             1U,
                                             (uint32_t)FALSE);
    if(configStatus != CSL_PASS)
    {
         DebugP_log("\nRerverting Config: Error in disabling Transmission Interrupt...\n", -1);
    }
    if(configStatus != CSL_PASS)
    {
        DebugP_log("\n===============Change in Baud-rate for Receiver FAILED===============\n");
    }
    else
    {
        DebugP_log("\n===============Change in Baud-rate for Receiver is SUCCESSFUL===============\n");
    }
    /* Wait for sometime, this could be needed by receiver to change the baud rate */
    /* Added Delay - This is needed by Rx node to change the baudrate */
    App_delayFunc(50U);

    return configStatus;
}

static int32_t App_mcanCheckTCResultsMisc(st_mcanTestcaseParams_t *testParams)
{
    int32_t status = CSL_PASS;
    uint32_t iterationCnt, loopCnt, oldTimeStamp = 0U;
    MCAN_TxEventFIFOElement txEventElem;
    MCAN_TxEventFIFOStatus txEventFIFOStatus;

    switch (testParams->testcaseId)
    {
        case 1:
            for (iterationCnt = 0U ;
             iterationCnt < testParams->mcanConfigParams.txMSGInterationCnt ;
             iterationCnt++)
            {
                for (loopCnt = 0U ;
                     loopCnt < testParams->mcanConfigParams.txMsgNum ;
                     loopCnt++)
                {
                    MCAN_getTxEventFIFOStatus(gMcanBaseAddr, &txEventFIFOStatus);
                    MCAN_readTxEventFIFO(gMcanBaseAddr, &txEventElem);
                    MCAN_writeTxEventFIFOAck(gMcanBaseAddr, txEventFIFOStatus.getIdx);

                    if(App_mcanTxEventMessageCheck(
                        testParams->mcanConfigParams.txMsg[loopCnt].txElem,
                        txEventElem) == CSL_PASS)
                    {
                        oldTimeStamp = txEventElem.txts;
                        status += CSL_PASS;
                        DebugP_log("Tx Event FIFO element matches with transmitted message.\r\n", -1);
                    }
                    else
                    {
                        DebugP_log("Tx Event FIFO element does not matche with transmitted message.\r\n", -1);
                        status += CSL_EFAIL;
                    }
                }
            }
        break;
        case 1633:
            for (iterationCnt = 0U ;
             iterationCnt < testParams->mcanConfigParams.txMSGInterationCnt ;
             iterationCnt++)
            {
                for (loopCnt = 0U ;
                     loopCnt < testParams->mcanConfigParams.txMsgNum ;
                     loopCnt++)
                {
                    MCAN_getTxEventFIFOStatus(gMcanBaseAddr, &txEventFIFOStatus);
                    MCAN_readTxEventFIFO(gMcanBaseAddr, &txEventElem);
                    MCAN_writeTxEventFIFOAck(gMcanBaseAddr, txEventFIFOStatus.getIdx);

                    if ((App_mcanTxEventMessageCheck(
                        testParams->mcanConfigParams.txMsg[loopCnt].txElem,
                        txEventElem) == CSL_PASS)  &&
                        (oldTimeStamp != txEventElem.txts))
                    {
                        oldTimeStamp = txEventElem.txts;
                        status += CSL_PASS;
                        DebugP_log("Tx Event FIFO element matches with transmitted message.\r\n", -1);
                    }
                    else
                    {
                        DebugP_log("Tx Event FIFO element does not matche with transmitted message.\r\n", -1);
                        status += CSL_EFAIL;
                    }
                }
            }
        break;
        case 1000:
            for (iterationCnt = 0U ;
             iterationCnt < testParams->mcanConfigParams.txMSGInterationCnt ;
             iterationCnt++)
            {
                for (loopCnt = 0U ;
                     loopCnt < testParams->mcanConfigParams.txMsgNum ;
                     loopCnt++)
                {
                    MCAN_getTxEventFIFOStatus(gMcanBaseAddr, &txEventFIFOStatus);
                    MCAN_readTxEventFIFO(gMcanBaseAddr, &txEventElem);
                    MCAN_writeTxEventFIFOAck(gMcanBaseAddr, txEventFIFOStatus.getIdx);

                    if(App_mcanTxEventMessageCheck(
                        testParams->mcanConfigParams.txMsg[loopCnt].txElem,
                        txEventElem) == CSL_PASS)
                    {
                        oldTimeStamp = txEventElem.txts;
                        status += CSL_PASS;
                        DebugP_log("Tx Event FIFO element matches with transmitted message.\r\n", -1);
                    }
                    else
                    {
                        DebugP_log("Tx Event FIFO element does not matche with transmitted message.\r\n", -1);
                        status += CSL_EFAIL;
                    }
                }
            }
        break;
        case 1003:
            for (iterationCnt = 0U ;
             iterationCnt < testParams->mcanConfigParams.txMSGInterationCnt ;
             iterationCnt++)
            {
                for (loopCnt = 0U ;
                     loopCnt < testParams->mcanConfigParams.txMsgNum ;
                     loopCnt++)
                {
                    MCAN_getTxEventFIFOStatus(gMcanBaseAddr, &txEventFIFOStatus);
                    MCAN_readTxEventFIFO(gMcanBaseAddr, &txEventElem);
                    MCAN_writeTxEventFIFOAck(gMcanBaseAddr, txEventFIFOStatus.getIdx);

                    if ((App_mcanTxEventMessageCheck(
                        testParams->mcanConfigParams.txMsg[loopCnt].txElem,
                        txEventElem) == CSL_PASS)  &&
                        (oldTimeStamp != txEventElem.txts))
                    {
                        oldTimeStamp = txEventElem.txts;
                        status += CSL_PASS;
                        DebugP_log("Tx Event FIFO element matches with transmitted message.\r\n", -1);
                    }
                    else
                    {
                        DebugP_log("Tx Event FIFO element does not match with transmitted message.\r\n", -1);
                        status += CSL_EFAIL;
                    }
                }
            }
        break;
        default:
        break;
    }

    return status;
}

static int32_t App_mcanTxEventMessageCheck(MCAN_TxBufElement txMsg,
                                           MCAN_TxEventFIFOElement txEventMsg)
{
    int32_t retStatus = CSL_PASS;

    if ((txMsg.id == txEventMsg.id) &&
        (txMsg.rtr == txEventMsg.rtr) &&
        (txMsg.xtd == txEventMsg.xtd) &&
        (txMsg.esi == txEventMsg.esi) &&
        (txMsg.dlc == txEventMsg.dlc) &&
        (txMsg.brs == txEventMsg.brs) &&
        (txMsg.fdf == txEventMsg.fdf) &&
        (txMsg.mm == txEventMsg.mm))
    {
        retStatus += CSL_PASS;
    }
    else
    {
        retStatus += CSL_EFAIL;
    }

    return retStatus;
}

int32_t App_mcanPerfTxTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t  configStatus = CSL_PASS, testStatus = CSL_PASS, status;
    uint32_t loopCnt      = 1U, iterationCnt = 0U, txBufCnt, maxMsgCnt;
    // uint32_t ;
    MCAN_ProtocolStatus protStatus;
    MCAN_ErrCntStatus    errCounter;
    // MCAN_RxBufElement rxMsg;
    MCAN_TxFIFOStatus txFIFOStatus;
    uint64_t tsDiff, hwUtiln, tsFreq;
    uint64_t numOfMsgPerSec;

    status = SemaphoreP_constructBinary(&gTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status = SemaphoreP_constructBinary(&gRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    configStatus += App_mcanRegisterInterrupt();
    if (CSL_PASS != configStatus)
    {
        DebugP_log("\nError in X-Bar Configuration...");
    }

    /* Reset MCAN Module */
    MCAN_reset(gMcanBaseAddr);
    while (MCAN_isInReset(gMcanBaseAddr) == (uint32_t)TRUE)
    {
    }
    configStatus += App_mcanConfig(testParams);
    if (CSL_PASS != configStatus)
    {
        DebugP_log("\nError in MCAN Configuration...");
    }

    configStatus += App_mcanTCEntrySetup(testParams);
    if(configStatus != CSL_PASS)
    {
        DebugP_log("\nError in MCAN TC Entry Configuration...TC cannot be Run...");
    }
    else
    {
    }
    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, testParams->mcanConfigParams.intrEnable, (uint32_t)TRUE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLineSelectMask,
                        testParams->mcanConfigParams.intrLine);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLine,
                        1U);
    /* Enable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)TRUE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n", -1);
            }
        }
    }
    /* Enable interrupts for Tx FIFO/Queue */
    for (loopCnt = testParams->mcanConfigParams.ramConfig->txBufCnt ;
         loopCnt < (testParams->mcanConfigParams.ramConfig->txFIFOCnt +
                    testParams->mcanConfigParams.ramConfig->txBufCnt);
         loopCnt++)
    {
        /* Enable Transmission interrupt */
        configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                loopCnt,
                (uint32_t)TRUE);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nMCAN Tx FIFO Interrupt Enable FAILED...\n", -1);
        }
    }

    /* Check for STD vs EXT ID and FD vs Classic messages for CAN message */
    if ((testParams->mcanConfigParams.txMsg[0U].txElem.xtd == 1U) && (testParams->mcanConfigParams.txMsg[0U].txElem.fdf == 1U))
    {
        maxMsgCnt = MCAN_THEOROTICAL_MAX_EXT_1_5_MBPS;
    }
    else if ((testParams->mcanConfigParams.txMsg[0U].txElem.xtd == 1U) && (testParams->mcanConfigParams.txMsg[0U].txElem.fdf == 0U))
    {
        maxMsgCnt = MCAN_CLASSIC_CAN_THEOROTICAL_MAX_EXT_1_MBPS;
    }
    else if ((testParams->mcanConfigParams.txMsg[0U].txElem.xtd == 0U) && (testParams->mcanConfigParams.txMsg[0U].txElem.fdf == 0U))
    {
        maxMsgCnt = MCAN_CLASSIC_CAN_THEOROTICAL_MAX_STD_1_MBPS;
    }
    else
    {
        maxMsgCnt = MCAN_THEOROTICAL_MAX_STD_1_5_MBPS;
    }
    /* capture time stamp before triggering Tx */
    for (iterationCnt = 0U ;
         iterationCnt < testParams->mcanConfigParams.txMSGInterationCnt ;
         iterationCnt++)
    {
        for (loopCnt = 0U ;
             loopCnt < testParams->mcanConfigParams.txMsgNum ;
             loopCnt++)
        {
            MCAN_getTxFIFOQueStatus(gMcanBaseAddr, &txFIFOStatus);
            if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
            {
                txBufCnt = testParams->mcanConfigParams.txMsg[loopCnt].bufferNum;
            }
            else
            {
                txBufCnt = txFIFOStatus.putIdx;
            }
            /* Calculate Rx Interrupt Mask */
            // if(testParams->mcanConfigParams.txMsg[loopCnt].rxMSGStorageId == MCAN_MEM_TYPE_BUF)
            // {
                // intrMask = MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG;
            // }
            // else
            // {
                // if(testParams->mcanConfigParams.txMsg[loopCnt].rxBuffNum == MCAN_RX_FIFO_NUM_0)
                // {
                    // intrMask = MCAN_INTR_SRC_RX_FIFO0_NEW_MSG;
                // }
                // else
                // {
                    // intrMask = MCAN_INTR_SRC_RX_FIFO1_NEW_MSG;
                // }

            // }
            // switch (testParams->testcaseId)
            // {
                // case 5:
                    // intrMask = MCAN_INTR_SRC_HIGH_PRIO_MSG;
                // break;
                // case 1004:
                    // intrMask = MCAN_INTR_SRC_HIGH_PRIO_MSG;
                // break;
                // default:
                // break;
            // }
            /* Write message to Msg RAM */
            MCAN_writeMsgRam(gMcanBaseAddr,
                             testParams->mcanConfigParams.txMsg[loopCnt].storageId,
                             txBufCnt,
                             &testParams->mcanConfigParams.txMsg[loopCnt].txElem);
            /* Add request for transmission */
            configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
            if (CSL_PASS != configStatus)
            {
                DebugP_log("\nError in Adding Transmission Request...\n", -1);
            }

            SemaphoreP_pend(&gTxDoneSem, SystemP_WAIT_FOREVER);
            // if (gRxDoneSem == NULL)
            // {
                // while (!((gMcanIsrIntr0Status & intrMask) == intrMask))
                // {
                // }
            // }
            // else
            // {
                // SemaphoreP_pend(gRxDoneSem, SystemP_WAIT_FOREVER);
            // }
            /* Checking for Errors */
            MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
            if ((0U == errCounter.recErrCnt) &&
                (0U == errCounter.canErrLogCnt))
            {
                MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
                /* Checking for Errors */
                if (((MCAN_ERR_CODE_NO_ERROR == protStatus.lastErrCode) ||
                     (MCAN_ERR_CODE_NO_CHANGE == protStatus.lastErrCode)) &&
                    ((MCAN_ERR_CODE_NO_ERROR == protStatus.dlec) ||
                     (MCAN_ERR_CODE_NO_CHANGE == protStatus.dlec)) &&
                    (0U == protStatus.pxe))
                {
                    // configStatus += App_mcanReadRxMSG(&rxMsg, gMcanIsrIntr0Status);
                    // if(configStatus != CSL_PASS)
                    // {
                        // DebugP_log("\nUnable to read received message(Iteration Count:Message Number): (%d,%d).\n", (iterationCnt + 1U), (loopCnt + 1U));
                    // }
                    // configStatus += App_mcanTxRxMessageCheck(
                                    // testParams->mcanConfigParams.txMsg[loopCnt].txElem,
                                    // rxMsg);
                    // /* Check if Message is stored into appropriate Rx Buffer */
                    // if(testParams->mcanConfigParams.txMsg[loopCnt].rxMSGStorageId == MCAN_MEM_TYPE_BUF)
                    // {
                        // if(rxBuffNum != testParams->mcanConfigParams.txMsg[loopCnt].rxBuffNum)
                        // {
                           // testStatus += CSL_EFAIL;
                        // }
                    // }
                    // if(configStatus != CSL_PASS)
                    // {
                        // testStatus += CSL_EFAIL;
                        // DebugP_log("\nTransmitted and received message does not match(Iteration Count:Message Number): (%d,%d).\n", (iterationCnt + 1U), (loopCnt + 1U));
                    // }
                    // else
                    // {
                        // testStatus += CSL_PASS;
                    // }
                }
                else
                {
                    testStatus += CSL_EFAIL;
                    DebugP_log("\nError in transmission/reception(Iteration Count:Message Number): (%d,%d).\n",
                               (iterationCnt + 1U), (loopCnt + 1U));
                }
            }
            else
            {
                testStatus += CSL_EFAIL;
                DebugP_log("\nError Counters: Error in transmission/reception(Iteration Count:Message Number): (%d,%d).\n",
                           (iterationCnt + 1U), (loopCnt + 1U));
            }
            gMcanIsrIntr0Status = 0U;
        }
    }
    /* capture time stamp After Tx completion */
    tsDiff = 1;
    tsFreq = MCAN_APP_CNT_FREQ_KHZ;

    numOfMsgPerSec = (testParams->mcanConfigParams.txMSGInterationCnt *
                      testParams->mcanConfigParams.txMsgNum * tsFreq) / tsDiff;
    /* If internal loopback then no need of x2 as Tx and Rx nodes are same */
    if (testParams->mcanConfigParams.mcanTestType == MCAN_TEST_TYPE_INTERNAL_LOOBACK)
    {
        numOfMsgPerSec *= 2U;
    }
    hwUtiln = ((numOfMsgPerSec * 100) / maxMsgCnt);
    printf("\nThroughPut: %lld Msg/sec\n", numOfMsgPerSec);
    printf("HW Utilization: %lld", hwUtiln);

    testParams->testResult += App_mcanCheckTCResultsMisc(testParams);
    configStatus += App_mcanTCExitSetup(testParams);
    if(configStatus != CSL_PASS)
    {
        DebugP_log("\nError in MCAN TC Exit Configuration...");
    }
    if(hwUtiln < 70U)
    {
        testParams->testResult += CSL_EFAIL;
        testStatus += CSL_EFAIL;
    }
    testParams->isRun      = CSL_PASS;
    SemaphoreP_destruct(&gTxDoneSem);
    SemaphoreP_destruct(&gRxDoneSem);

    DebugP_log("\nPer TX Test: %d", testStatus);

    return testStatus;
}

static int32_t App_mcanECCTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t  configStatus = CSL_PASS, testStatus = CSL_PASS;
    uint32_t loopCnt      = 1U, iterationCnt = 0U, intrMask, txBufCnt;
    MCAN_ProtocolStatus protStatus;
    MCAN_ErrCntStatus    errCounter;
    MCAN_RxBufElement rxMsg;
    MCAN_TxFIFOStatus txFIFOStatus;
    uint32_t startAddr, elemSize, elemAddr;

    gMcanECCIntrFlag = 1U;
    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, testParams->mcanConfigParams.intrEnable, (uint32_t)TRUE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLineSelectMask,
                        testParams->mcanConfigParams.intrLine);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLine,
                        1U);
    /* Enable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)TRUE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n", -1);
            }
            /* Enable Transmission Cancellation interrupt */
            configStatus += MCAN_txBufCancellationIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)TRUE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Cancellation Interrupt Enable FAILED...\n", -1);
            }
        }
    }
    /* Enable interrupts for Tx FIFO/Queue */
    for (loopCnt = testParams->mcanConfigParams.ramConfig->txBufCnt ;
         loopCnt < (testParams->mcanConfigParams.ramConfig->txFIFOCnt +
                    testParams->mcanConfigParams.ramConfig->txBufCnt);
         loopCnt++)
    {
        /* Enable Transmission interrupt */
        configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                loopCnt,
                (uint32_t)TRUE);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nMCAN Tx FIFO Interrupt Enable FAILED...\n", -1);
        }
        /* Enable Transmission Cancellation interrupt */
        configStatus += MCAN_txBufCancellationIntrEnable(gMcanBaseAddr,
                loopCnt,
                (uint32_t)TRUE);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nMCAN Tx Cancellation Interrupt Enable FAILED...\n", -1);
        }
    }
    /* Enable ECC Interrupts */
    MCAN_eccEnableIntr(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_SEC, TRUE);
    MCAN_eccEnableIntr(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED, TRUE);
    if (testParams->mcanConfigParams.txMsg[0U].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
    {
        loopCnt = 0U;
        startAddr = testParams->mcanConfigParams.ramConfig->txStartAddr;
        elemSize  = objSize[testParams->mcanConfigParams.ramConfig->txBufElemSize];
        elemSize *= 4U;
        elemAddr  = startAddr + (elemSize * testParams->mcanConfigParams.txMsg[0U].bufferNum);
        /* Access data bytes of the message */
        elemAddr += 12U;
        elemAddr += MCAN_MCAN_MSG_MEM;
        elemAddr += gMcanBaseAddr;

        MCAN_getTxFIFOQueStatus(gMcanBaseAddr, &txFIFOStatus);
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                    MCAN_MEM_TYPE_BUF)
        {
            txBufCnt = testParams->mcanConfigParams.txMsg[loopCnt].bufferNum;
        }
        else
        {
            txBufCnt = txFIFOStatus.putIdx;
        }
        /* Calculate Rx Interrupt Mask */
        if(testParams->mcanConfigParams.txMsg[loopCnt].rxMSGStorageId == MCAN_MEM_TYPE_BUF)
        {
            intrMask = MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG;
        }
        else
        {
            if(testParams->mcanConfigParams.txMsg[loopCnt].rxBuffNum == MCAN_RX_FIFO_NUM_0)
            {
                intrMask = MCAN_INTR_SRC_RX_FIFO0_NEW_MSG;
            }
            else
            {
                intrMask = MCAN_INTR_SRC_RX_FIFO1_NEW_MSG;
            }

        }
        switch (testParams->testcaseId)
        {
            case 1252:
                intrMask = MCAN_INTR_SRC_HIGH_PRIO_MSG;
            break;
            default:
            break;
        }
        /* SEC error test */
        {
            // App_mcanPrintTxMsg(&testParams->mcanConfigParams.txMsg[loopCnt].txElem);
            /* Write message to Msg RAM */
            MCAN_writeMsgRam(gMcanBaseAddr,
                             testParams->mcanConfigParams.txMsg[loopCnt].storageId,
                             txBufCnt,
                             &testParams->mcanConfigParams.txMsg[loopCnt].txElem);
            /* Disable ECC */
            testParams->mcanConfigParams.eccConfigParams->enable = 0U;
            testParams->mcanConfigParams.eccConfigParams->enableChk = 0U;
            MCAN_eccConfig(gMcanBaseAddr, testParams->mcanConfigParams.eccConfigParams);
            DebugP_log("Before single Bit Error/Bit Flip:0x%x\n", HW_RD_REG32(elemAddr));
            /* Introduce single bit error */
            HW_WR_REG32(elemAddr, HW_RD_REG32(elemAddr)^0x8U);
            DebugP_log("After single Bit Error/Bit Flip:0x%x\n", HW_RD_REG32(elemAddr));
            /* Enable ECC */
            testParams->mcanConfigParams.eccConfigParams->enable = 1U;
            testParams->mcanConfigParams.eccConfigParams->enableChk = 1U;
            MCAN_eccConfig(gMcanBaseAddr, testParams->mcanConfigParams.eccConfigParams);
            /* Add request for transmission */
            configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
            if (CSL_PASS != configStatus)
            {
                DebugP_log("\nError in Adding Transmission Request...\n", -1);
            }
            while (gMcanECCIntrFlag)
            {
            }
            gMcanECCIntrFlag = 1U;
            while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_TRANS_COMPLETE) ==
                                MCAN_INTR_SRC_TRANS_COMPLETE))
            {}
            while (!((gMcanIsrIntr0Status & intrMask) == intrMask))
            {
            }
            /* Check for ECC Error */
            if ((gMcaneccErr.secErr == 1U) &&
                (gMcaneccErr.dedErr != 1U) &&
                (gMcaneccErr.row == ((elemAddr - (MCAN_MCAN_MSG_MEM + gMcanBaseAddr))/4)) &&
                (gMcaneccErr.bit1 == 3U))
            {
                DebugP_log("\nECC SEC Error location matched.\n");
            }
            else
            {
                testStatus += CSL_EFAIL;
                DebugP_log("\nECC SEC Error location does not match.\n");
            }
            /* Checking for Errors */
            MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
            if ((0U == errCounter.recErrCnt) &&
                (0U == errCounter.canErrLogCnt))
            {
                MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
                /* Checking for Errors */
                if (((MCAN_ERR_CODE_NO_ERROR == protStatus.lastErrCode) ||
                     (MCAN_ERR_CODE_NO_CHANGE == protStatus.lastErrCode)) &&
                    ((MCAN_ERR_CODE_NO_ERROR == protStatus.dlec) ||
                     (MCAN_ERR_CODE_NO_CHANGE == protStatus.dlec)) &&
                    (0U == protStatus.pxe))
                {
                    configStatus += App_mcanReadRxMSG(&rxMsg, gMcanIsrIntr0Status);
                    if(configStatus != CSL_PASS)
                    {
                        DebugP_log("\nUnable to read received message(Iteration Count:Message Number): (%d,%d).\n", (iterationCnt + 1U), (loopCnt + 1U));
                    }
                    configStatus += App_mcanTxRxMessageCheck(
                                    testParams->mcanConfigParams.txMsg[loopCnt].txElem,
                                    rxMsg);
                    /* Check if Message is stored into appropriate Rx Buffer */
                    if(testParams->mcanConfigParams.txMsg[loopCnt].rxMSGStorageId == MCAN_MEM_TYPE_BUF)
                    {
                        if(rxBuffNum != testParams->mcanConfigParams.txMsg[loopCnt].rxBuffNum)
                        {
                           testStatus += CSL_EFAIL;
                        }
                    }
                    if(configStatus != CSL_PASS)
                    {
                        testStatus += CSL_EFAIL;
                        DebugP_log("\nTransmitted and received message does not match(Iteration Count:Message Number): (%d,%d).\n", (iterationCnt + 1U), (loopCnt + 1U));
                    }
                    else
                    {
                        testStatus += CSL_PASS;
                        DebugP_log(
                            "\nMessage successfully transferred/received(Iteration Count:Message Number): (%d,%d).\n",
                            (iterationCnt + 1U), (loopCnt + 1U));
                    }
                }
                else
                {
                    testStatus += CSL_EFAIL;
                    DebugP_log("\nError in transmission/reception(Iteration Count:Message Number): (%d,%d).\n",
                               (iterationCnt + 1U), (loopCnt + 1U));
                }
            }
            else
            {
                testStatus += CSL_EFAIL;
                DebugP_log("\nError Counters: Error in transmission/reception(Iteration Count:Message Number): (%d,%d).\n",
                           (iterationCnt + 1U), (loopCnt + 1U));
            }
            gMcanIsrIntr0Status = 0U;
        }
        /* DED error test */
        {
            // App_mcanPrintTxMsg(&testParams->mcanConfigParams.txMsg[loopCnt].txElem);
            /* Write message to Msg RAM */
            MCAN_writeMsgRam(gMcanBaseAddr,
                             testParams->mcanConfigParams.txMsg[loopCnt].storageId,
                             txBufCnt,
                             &testParams->mcanConfigParams.txMsg[loopCnt].txElem);
            /* Disable ECC */
            testParams->mcanConfigParams.eccConfigParams->enable = 0U;
            testParams->mcanConfigParams.eccConfigParams->enableChk = 0U;
            MCAN_eccConfig(gMcanBaseAddr, testParams->mcanConfigParams.eccConfigParams);
            DebugP_log("Before Double Bit Error/Bit Flip:0x%x\n", HW_RD_REG32(elemAddr));
            /* Introduce Double bit error */
            HW_WR_REG32(elemAddr, HW_RD_REG32(elemAddr)^0xAU);
            DebugP_log("After Double Bit Error/Bit Flip:0x%x\n", HW_RD_REG32(elemAddr));
            /* Enable ECC */
            testParams->mcanConfigParams.eccConfigParams->enable = 1U;
            testParams->mcanConfigParams.eccConfigParams->enableChk = 1U;
            MCAN_eccConfig(gMcanBaseAddr, testParams->mcanConfigParams.eccConfigParams);
            /* Add request for transmission */
            configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
            if (CSL_PASS != configStatus)
            {
                DebugP_log("\nError in Adding Transmission Request...\n", -1);
            }
            while (gMcanECCIntrFlag)
            {
            }
            gMcanECCIntrFlag = 1U;
            if (MCAN_OPERATION_MODE_SW_INIT == MCAN_getOpMode(gMcanBaseAddr))
            {
                DebugP_log("\nTransmission of corrupted message stopped.\n");
            }
            else
            {
                testStatus += CSL_EFAIL;
                DebugP_log("\nSent out Corrupted Message.\n");
            }
            /* cancel transmission of corrupted message */
            MCAN_txBufCancellationReq(gMcanBaseAddr, txBufCnt);
            while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_TRANS_CANCEL_FINISH) ==
                                MCAN_INTR_SRC_TRANS_CANCEL_FINISH))
            {}
            DebugP_log( "\nTx Buffer Transmission Cancellation Interrupt Happened.\n");
            /* Take MCAN out of the SW initialization mode */
            MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
            while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanBaseAddr))
            {}
            /* Check for ECC Error */
            if ((gMcaneccErr.secErr != 1U) &&
                (gMcaneccErr.dedErr == 1U) &&
                (gMcaneccErr.row == ((elemAddr - (MCAN_MCAN_MSG_MEM + gMcanBaseAddr))/4)))
            {
                DebugP_log("\nECC DED Error location matched.\n");
            }
            else
            {
                testStatus += CSL_EFAIL;
                DebugP_log("\nECC DED Error location does not match.\n");
            }
            gMcanIsrIntr0Status = 0U;
        }
    }
    return testStatus;
}

static int32_t App_mcanECCSelfTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t  testStatus = CSL_PASS;
    uint32_t accessAddr, secErrFlag = 0U, dedErrFlag = 0U;

    gMcanECCIntrFlag = 1U;
    /* Enable ECC Interrupts */
    MCAN_eccEnableIntr(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_SEC, TRUE);
    MCAN_eccEnableIntr(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED, TRUE);
    MCAN_eccForceError(gMcanBaseAddr, testParams->mcanConfigParams.eccFrcParams);
    if (testParams->mcanConfigParams.eccFrcParams->errType == MCAN_ECC_ERR_TYPE_SEC)
    {
        secErrFlag = 1U;
        DebugP_log("SEC Error Test:\n");
    }
    if (testParams->mcanConfigParams.eccFrcParams->errType == MCAN_ECC_ERR_TYPE_DED)
    {
        dedErrFlag = 1U;
        DebugP_log("DED Error Test:\n");
    }
    accessAddr = gMcanBaseAddr;
    accessAddr += MCAN_MCAN_MSG_MEM;
    accessAddr += (testParams->mcanConfigParams.eccFrcParams->rowNum * 4U);
    HW_RD_REG32(accessAddr);
    while (gMcanECCIntrFlag)
    {
    }
    gMcanECCIntrFlag = 1U;
    /* Check for ECC Error */
    if ((gMcaneccErr.secErr == secErrFlag) &&
        (gMcaneccErr.dedErr == dedErrFlag) &&
        (gMcaneccErr.row == testParams->mcanConfigParams.eccFrcParams->rowNum))
    {
        if (secErrFlag == 1U)
        {
            if (gMcaneccErr.bit1 != testParams->mcanConfigParams.eccFrcParams->bit1)
            {
                testStatus += CSL_EFAIL;
                DebugP_log("\nECC SEC Error bit location match failed.\n");
            }
        }
        DebugP_log("\nECC Error location matched.\n");
    }
    else
    {
        testStatus += CSL_EFAIL;
        DebugP_log("\nECC Error location does not match.\n");
    }

    return testStatus;
}

static int32_t App_mcanMsgCancelTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t  configStatus = CSL_PASS, testStatus = CSL_PASS;
    uint32_t loopCnt      = 1U, txBufCnt;

    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, testParams->mcanConfigParams.intrEnable, (uint32_t)TRUE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLineSelectMask,
                        testParams->mcanConfigParams.intrLine);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLine,
                        1U);
    /* Enable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)TRUE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n", -1);
                testStatus += CSL_EFAIL;
            }
        }
    }
    /* Enable interrupts for Tx FIFO/Queue */
    for (loopCnt = testParams->mcanConfigParams.ramConfig->txBufCnt ;
         loopCnt < (testParams->mcanConfigParams.ramConfig->txFIFOCnt +
                    testParams->mcanConfigParams.ramConfig->txBufCnt);
         loopCnt++)
    {
        /* Enable Transmission interrupt */
        configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                loopCnt,
                (uint32_t)TRUE);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nMCAN Tx FIFO Interrupt Enable FAILED...\n", -1);
            testStatus += CSL_EFAIL;
        }
    }

    loopCnt = 0U;
    txBufCnt = testParams->mcanConfigParams.txMsg[loopCnt].bufferNum;
    // App_mcanPrintTxMsg(&testParams->mcanConfigParams.txMsg[loopCnt].txElem);

    /* Write message to Msg RAM */
    MCAN_writeMsgRam(gMcanBaseAddr,
                     testParams->mcanConfigParams.txMsg[0U].storageId,
                     txBufCnt,
                     &testParams->mcanConfigParams.txMsg[0U].txElem);
    /* add transmission request only if previous transmission is completed */
    if ((MCAN_getTxBufReqPend(gMcanBaseAddr) & (1U << txBufCnt)) != (1U << txBufCnt))
    {
        /* Clear pending Tx request */
        MCAN_txBufCancellationReq(gMcanBaseAddr, txBufCnt);
        while ((MCAN_txBufCancellationStatus(gMcanBaseAddr)  & (0x1 << txBufCnt)) != (0x1 << txBufCnt))
        {
        }
        MCAN_txBufCancellationIntrEnable(gMcanBaseAddr, txBufCnt, TRUE);
        /* Take MCAN out of the SW initialization mode */
        MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
        while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanBaseAddr))
        {}
    }

    return testStatus;
}

static int32_t App_mcanStateTransnTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t  configStatus = CSL_PASS, testStatus = CSL_PASS;
    uint32_t loopCnt      = 1U, txBufCnt;
    MCAN_ProtocolStatus protStatus;
    MCAN_ErrCntStatus errCnt;

    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, testParams->mcanConfigParams.intrEnable, (uint32_t)TRUE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLineSelectMask,
                        testParams->mcanConfigParams.intrLine);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLine,
                        1U);
    /* Enable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)TRUE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n", -1);
                testStatus += CSL_EFAIL;
            }
        }
    }
    /* Enable interrupts for Tx FIFO/Queue */
    for (loopCnt = testParams->mcanConfigParams.ramConfig->txBufCnt ;
         loopCnt < (testParams->mcanConfigParams.ramConfig->txFIFOCnt +
                    testParams->mcanConfigParams.ramConfig->txBufCnt);
         loopCnt++)
    {
        /* Enable Transmission interrupt */
        configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                loopCnt,
                (uint32_t)TRUE);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nMCAN Tx FIFO Interrupt Enable FAILED...\n", -1);
            testStatus += CSL_EFAIL;
        }
    }

    loopCnt = 0U;
    txBufCnt = testParams->mcanConfigParams.txMsg[loopCnt].bufferNum;
    // App_mcanPrintTxMsg(&testParams->mcanConfigParams.txMsg[loopCnt].txElem);

    /* Write message to Msg RAM */
    MCAN_writeMsgRam(gMcanBaseAddr,
                     testParams->mcanConfigParams.txMsg[0U].storageId,
                     txBufCnt,
                     &testParams->mcanConfigParams.txMsg[0U].txElem);
    DebugP_log("\nCurrent State: Error Active\n");
    while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_ERR_PASSIVE) ==
                                MCAN_INTR_SRC_ERR_PASSIVE))
    {
        /* add transmission request only if previous transmission is completed */
        if ((MCAN_getTxBufReqPend(gMcanBaseAddr) & (1U << txBufCnt)) != (1U << txBufCnt))
        {
            loopCnt++;
            /* Add request for transmission */
            configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
            if (CSL_PASS != configStatus)
            {
                DebugP_log("\nError in Adding Transmission Request...\n", -1);
                testStatus += CSL_EFAIL;
            }
        }
    }
    gMcanIsrIntr0Status = 0U;
    MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
    if ((protStatus.errPassive == 1U) && (protStatus.busOffStatus != 1U))
    {
        DebugP_log("\nCurrent State: Error Passive\n");
        DebugP_log( "Message transmitted unsuccessfully: %d\n", loopCnt);

    }

    while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_BUS_OFF_STATUS) ==
                                MCAN_INTR_SRC_BUS_OFF_STATUS))
    {
        /* add transmission request only if previous transmission is completed */
        if ((MCAN_getTxBufReqPend(gMcanBaseAddr) & (1U << txBufCnt)) != (1U << txBufCnt))
        {
            loopCnt++;
            /* Add request for transmission */
            configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
            if (CSL_PASS != configStatus)
            {
                DebugP_log("\nError in Adding Transmission Request...\n", -1);
            }
        }
    }
    gMcanIsrIntr0Status = 0U;
    MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
    if ((protStatus.errPassive == 1U) &&
        (protStatus.busOffStatus == 1U) &&
        (MCAN_OPERATION_MODE_SW_INIT == MCAN_getOpMode(gMcanBaseAddr)))
    {
        DebugP_log("\nCurrent State: Bus Off\n");
        DebugP_log("Message transmitted unsuccessfully: %d\n", loopCnt);

    }
    loopCnt = 0U;

    /* Clear pending Tx request */
    MCAN_txBufCancellationReq(gMcanBaseAddr, 0U);
    while ((MCAN_txBufCancellationStatus(gMcanBaseAddr)  & (0x1 << 0)) != (0x1 << 0))
    {
    }
    /* Take MCAN out of the SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanBaseAddr))
    {}
    /* wait for 129 occurrences of Bus Idle (129 × 11 consecutive recessive bits)
       before resuming normal operation. */
    /* The MCAN_ECR[14:8] REC field is used to count these sequences */
    /* Wait till first 11 consecutive recessive bits are detected */
    /* After resetting CCCR.INIT bit, MCAN writes to '5'(MCAN_ERR_CODE_BIT0_ERROR) to PSR.LEC */
    MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
    while (protStatus.lastErrCode != MCAN_ERR_CODE_BIT0_ERROR)
    {
        MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
    }
    /* This is 7 bit field(max value 127), so wait till it becomes none zero */
    /* wait for rest 128 occurrences */
    MCAN_getErrCounters(gMcanBaseAddr, &errCnt);
    while (errCnt.recErrCnt != 0U)
    {
        MCAN_getErrCounters(gMcanBaseAddr, &errCnt);
    }
    MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
    if ((protStatus.errPassive == 0U) &&
        (protStatus.busOffStatus == 0U) &&
        (protStatus.warningStatus == 0U))
    {
        DebugP_log("\nCurrent State: Error Active\n");
        DebugP_log("Message transmitted Successfully: %d\n", loopCnt);
    }
    else
    {
        DebugP_log("\nCurrent State: Error Warning/Passive or Bus Off\n");
        testStatus += CSL_EFAIL;
    }

    return testStatus;
}

static int32_t App_mcanExtTSIntrTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t testStatus = CSL_PASS;

    gMcanExtTSIntrFlag = 1U;
    /* Enable External Time Stamp Interrupt */
    MCAN_extTSEnableIntr(gMcanBaseAddr, TRUE);

    isrPrintEnable = TRUE;

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
    /* MCAN External TS Interrupt is connected to ESM Module
     * so ESM produces only one time interrupt
     */
    DebugP_log( "Waiting for Interrupt...\n");
    while (gMcanExtTSIntrFlag)
    {
    }
#else
    DebugP_log( "Waiting for 1st Interrupt...\n");
    while (gMcanExtTSIntrFlag)
    {
    }
    gMcanExtTSIntrFlag = 1U;
    DebugP_log( "Waiting for 2nd Interrupt...\n");
    while (gMcanExtTSIntrFlag)
    {
    }
    gMcanExtTSIntrFlag = 1U;
    DebugP_log( "Waiting for 3rd Interrupt...\n");
    while (gMcanExtTSIntrFlag)
    {
    }
    gMcanExtTSIntrFlag = 1U;
#endif
    /* Disable External Time Stamp Interrupt */
    MCAN_extTSEnableIntr(gMcanBaseAddr, FALSE);

    return testStatus;
}

static int32_t App_mcanTxRxPinStateTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t testStatus = CSL_PASS;

    /* Put MCAN in SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(gMcanBaseAddr))
    {}

    /* Monitors the actual value of the MCAN RX pin
       Rx pin may be Dominant/Recessive. Not correct to return error */
    if (MCAN_getRxPinState(gMcanBaseAddr) == 0U)
    {
        DebugP_log( "Rx Pin Mode: Dominant\r\n");
    }
    else
    {
        DebugP_log( "Rx Pin Mode: Recessive\r\n");
    }
    if (MCAN_getRxPinState(gMcanBaseAddr) == 1U)
    {
        DebugP_log( "Rx Pin Mode: Recessive\r\n");
    }
    else
    {
        DebugP_log( "Rx Pin Mode: Dominant\r\n");
    }
    DebugP_log( "Setting Tx PAD into Dominant mode.\r\n");
    MCAN_setTxPinState(gMcanBaseAddr, 0x2);
    if (MCAN_getTxPinState(gMcanBaseAddr) == 0x2U)
    {
        DebugP_log( "Tx Pin Mode: Dominant\r\n");
    }
    else
    {
        testStatus += CSL_EFAIL;
        DebugP_log( "Rx Pin Mode: Recessive\r\n");
    }
    DebugP_log( "Setting Tx PAD into Recessive mode.\r\n");
    MCAN_setTxPinState(gMcanBaseAddr, 0x3);
    if (MCAN_getTxPinState(gMcanBaseAddr) == 0x3U)
    {
        DebugP_log( "Tx Pin Mode: Recessive\r\n");
    }
    else
    {
        testStatus += CSL_EFAIL;
        DebugP_log( "Tx Pin Mode: Dominant\r\n");
    }

    DebugP_log( "Setting Tx PAD into Dominant mode.\r\n");
    MCAN_setTxPinState(gMcanBaseAddr, 0x0);
    if (MCAN_getTxPinState(gMcanBaseAddr) == 0x0)
    {
        DebugP_log( "Resetting Tx Pin \r\n");
    }
    else
    {
        testStatus += CSL_EFAIL;
        DebugP_log( "Tx Pin Set Fail \r\n");
    }

    /* Take MCAN out of the SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanBaseAddr))
    {}

    return testStatus;
}

static int32_t App_mcanClkStpReqTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t testStatus = CSL_PASS;

    DebugP_log("Asserting Clock Stop Request..\r\n");
    MCAN_addClockStopRequest(gMcanBaseAddr, TRUE);
    while (MCAN_getClkStopAck(gMcanBaseAddr) != 1U)
    {
    }
    DebugP_log("Clock Stop Request ACKed!!\r\n");
    if (MCAN_OPERATION_MODE_SW_INIT == MCAN_getOpMode(gMcanBaseAddr))
    {
        DebugP_log("MCAN is in SW Init Mode.\r\n");
    }
    else
    {
        DebugP_log("MCAN is in not SW Init Mode.\r\n");
        testStatus += CSL_EFAIL;
    }
    MCAN_getClockStopAck(gMcanBaseAddr);
    MCAN_addClockStopRequest(gMcanBaseAddr, FALSE);

    return testStatus;
}

static int32_t App_mcanTSRstTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t testStatus = CSL_PASS;
    uint32_t oldTimeStamp, timeStamp;

    timeStamp = MCAN_getTSCounterVal(gMcanBaseAddr);
    DebugP_log("TS at (t) ms: %d\r\n", timeStamp);
    /* Delay added for getting new timeStamp- so that there will difference between two timeStamps */
    App_delayFunc(10);
    oldTimeStamp = timeStamp;
    timeStamp = MCAN_getTSCounterVal(gMcanBaseAddr);
    DebugP_log("TS at (t + 10) ms: %d\r\n", timeStamp);

    /* reset TS counter */
    MCAN_resetTSCounter(gMcanBaseAddr);
    oldTimeStamp = timeStamp;
    timeStamp = MCAN_getTSCounterVal(gMcanBaseAddr);
    /* wrap around condition is not taken care */
    if (timeStamp < oldTimeStamp)
    {
        DebugP_log("TS Counter Reset done!!\r\n");
    }
    else
    {
        testStatus += CSL_EFAIL;
    }

    return testStatus;
}

static int32_t App_mcanBusMonTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t configStatus = CSL_PASS, testStatus = CSL_PASS;
    MCAN_RxBufElement rxMsg;

    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, testParams->mcanConfigParams.intrEnable, (uint32_t)TRUE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLineSelectMask,
                        testParams->mcanConfigParams.intrLine);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLine,
                        1U);
    gMcanIsrIntr0Status = 0U;
    DebugP_log( "Send message over the CAN network(TDA3xx MCAN and only one another node).\n");
    DebugP_log( "Same message shall be received by MCAN but transmitter node shall get ACK Error.\n");
    while  (!(((gMcanIsrIntr0Status & MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG) ==
                                    MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG) ||
            ((gMcanIsrIntr0Status & MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ==
                                    MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ||
            ((gMcanIsrIntr0Status & MCAN_INTR_SRC_RX_FIFO1_NEW_MSG) ==
                                    MCAN_INTR_SRC_RX_FIFO1_NEW_MSG)))
    {
    }
    /* Disable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLine,
                        0U);
    DebugP_log( "Message received:\n");
    configStatus += App_mcanReadRxMSG(&rxMsg, gMcanIsrIntr0Status);
    if(configStatus != CSL_PASS)
    {
        testStatus += CSL_EFAIL;
        DebugP_log("\nUnable to read received message.\n");
    }
    gMcanIsrIntr0Status = 0U;

    return testStatus;
}

static int32_t App_mcanLatencyTest(st_mcanTestcaseParams_t *testParams)
{
    return (CSL_PASS);
}

int32_t App_mcanPerfTxRxTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t  configStatus = CSL_PASS, testStatus = CSL_PASS;
    int32_t                 status = SystemP_SUCCESS;
    MCAN_RxFIFOStatus fifoStatus;
    MCAN_RxBufElement rxMsg;
    uint32_t loopCnt      = 1U, iterationCnt = 0U, txBufNum, maxMsgCnt;
    MCAN_ErrCntStatus    errCounter;
    MCAN_TxFIFOStatus txFIFOStatus;
    uint64_t tsDiff, hwUtiln, tsFreq;
    uint64_t numOfMsgPerSec;
    /* Get ticks delay */
    CycleCounterP_reset();
    uint32_t ticksDelay = CycleCounterP_getCount32();
    uint32_t startTicks, stopTicks;

    ticksDelay = CycleCounterP_getCount32() - ticksDelay;

    status = SemaphoreP_constructBinary(&gTxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status = SemaphoreP_constructBinary(&gRxDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    configStatus += App_mcanRegisterInterrupt();
    if (CSL_PASS != configStatus)
    {
        DebugP_log("\nError in Interrupt Configuration...");
    }

    /* Reset MCAN Module */
    MCAN_reset(gMcanBaseAddr);
    while (MCAN_isInReset(gMcanBaseAddr) == (uint32_t)TRUE)
    {
    }
    configStatus += App_mcanConfig(testParams);
    if (CSL_PASS != configStatus)
    {
        DebugP_log("\nError in MCAN Configuration...");
    }

    configStatus += App_mcanTCEntrySetup(testParams);
    if(configStatus != CSL_PASS)
    {
        DebugP_log("\nError in MCAN TC Entry Configuration...TC cannot be Run...");
    }
    else
    {
    }
    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, testParams->mcanConfigParams.intrEnable, (uint32_t)TRUE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLineSelectMask,
                        testParams->mcanConfigParams.intrLine);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLine,
                        1U);
    /* Enable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)TRUE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n");
            }
        }
    }
    /* Enable interrupts for Tx FIFO/Queue */
    for (loopCnt = testParams->mcanConfigParams.ramConfig->txBufCnt ;
         loopCnt < (testParams->mcanConfigParams.ramConfig->txFIFOCnt +
                    testParams->mcanConfigParams.ramConfig->txBufCnt);
         loopCnt++)
    {
        /* Enable Transmission interrupt */
        configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                loopCnt,
                (uint32_t)TRUE);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nMCAN Tx FIFO Interrupt Enable FAILED...\n");
        }
    }

    /* Check for STD vs EXT ID and FD vs Classic messages for CAN message */
    if ((testParams->mcanConfigParams.txMsg[0U].txElem.xtd == 1U) && (testParams->mcanConfigParams.txMsg[0U].txElem.fdf == 1U))
    {
        maxMsgCnt = MCAN_THEOROTICAL_MAX_EXT_1_5_MBPS;
    }
    else if ((testParams->mcanConfigParams.txMsg[0U].txElem.xtd == 1U) && (testParams->mcanConfigParams.txMsg[0U].txElem.fdf == 0U))
    {
        maxMsgCnt = MCAN_CLASSIC_CAN_THEOROTICAL_MAX_EXT_1_MBPS;
    }
    else if ((testParams->mcanConfigParams.txMsg[0U].txElem.xtd == 0U) && (testParams->mcanConfigParams.txMsg[0U].txElem.fdf == 0U))
    {
        maxMsgCnt = MCAN_CLASSIC_CAN_THEOROTICAL_MAX_STD_1_MBPS;
    }
    else
    {
        maxMsgCnt = MCAN_THEOROTICAL_MAX_STD_1_5_MBPS;
    }
    /* Throughput measurement for Tx only */
    /* capture time stamp before triggering Tx */
    MCAN_getTxFIFOQueStatus(gMcanBaseAddr, &txFIFOStatus);
    if (testParams->mcanConfigParams.txMsg[0U].storageId ==
                                                MCAN_MEM_TYPE_BUF)
    {
        txBufNum = testParams->mcanConfigParams.txMsg[0U].bufferNum;
    }
    else
    {
        txBufNum = txFIFOStatus.putIdx;
    }
    CycleCounterP_reset();
    startTicks = CycleCounterP_getCount32();
    for (iterationCnt = 0U ;
         iterationCnt < testParams->mcanConfigParams.txMSGInterationCnt ;
         iterationCnt++)
    {
        for (loopCnt = 0U ;
             loopCnt < testParams->mcanConfigParams.txMsgNum ;
             loopCnt++)
        {
            /* Write message to Msg RAM */
            MCAN_writeMsgRam(gMcanBaseAddr,
                             testParams->mcanConfigParams.txMsg[loopCnt].storageId,
                             txBufNum,
                             &testParams->mcanConfigParams.txMsg[loopCnt].txElem);
            /* Add request for transmission */
            configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufNum);
            if (CSL_PASS != configStatus)
            {
                DebugP_log("\nError in Adding Transmission Request...\n");
            }
            /* Wait for Tx completion */
            SemaphoreP_pend(&gTxDoneSem, SystemP_WAIT_FOREVER);

            /* Wait for Rx completion */
            SemaphoreP_pend(&gRxDoneSem, SystemP_WAIT_FOREVER);

            /* Checking for Errors */
            MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
            if ((0U != errCounter.recErrCnt) &&
                (0U != errCounter.transErrLogCnt))
            {
                /* Error occurred in transmission*/
                testStatus += CSL_EFAIL;
                DebugP_log("\nError Counters: Error in transmission/reception(Iteration Count:Message Number): (%d,%d).\n",
                           (iterationCnt + 1U), (loopCnt + 1U));
                break;
            }

            /* Messages will be stored in FIFO1 as these will fail filters and non-matching messages are stored into FIFO 1 */
            fifoStatus.num = (uint32_t)MCAN_RX_FIFO_NUM_1;
            MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
            MCAN_readMsgRam(gMcanBaseAddr,
                            MCAN_MEM_TYPE_FIFO,
                            fifoStatus.getIdx,
                            (uint32_t)fifoStatus.num,
                            &rxMsg);
            (void) MCAN_writeRxFIFOAck(gMcanBaseAddr,
                                       (uint32_t)fifoStatus.num,
                                       fifoStatus.getIdx);
            configStatus += App_mcanTxRxMessageCheck(
                                testParams->mcanConfigParams.txMsg[loopCnt].txElem,
                                rxMsg);
            if(configStatus != CSL_PASS)
            {
                testStatus += CSL_EFAIL;
                DebugP_log("\nTransmitted and received message does not match(Iteration Count:Message Number): (%d,%d).\n", (iterationCnt + 1U), (loopCnt + 1U));
                break;
            }
        }
    }
    /* capture time stamp After Tx completion */
    stopTicks = CycleCounterP_getCount32();

    if(stopTicks > startTicks)
    {
        tsDiff = (uint64_t)(stopTicks - startTicks - ticksDelay);
    }
    else
    {
        /* Counter overflow, assume only one overflow has happened */
        tsDiff = (uint64_t)((0xFFFFFFFFU - startTicks) + stopTicks - ticksDelay);
    }

    tsFreq = SOC_getSelfCpuClk();
    numOfMsgPerSec = ((uint64_t)testParams->mcanConfigParams.txMSGInterationCnt *
                      (uint64_t)testParams->mcanConfigParams.txMsgNum * tsFreq) / tsDiff;

    /* If internal loopback then no need of x2 as Tx and Rx nodes are same */
    if (!(testParams->mcanConfigParams.mcanTestType == MCAN_TEST_TYPE_INTERNAL_LOOBACK))
    {
        numOfMsgPerSec *= 2U;
    }
    hwUtiln = ((numOfMsgPerSec * 100) / maxMsgCnt);
    DebugP_log("\nTxRx:: Iteration Count:%d\tNumber of messages:%d\r\n", testParams->mcanConfigParams.txMSGInterationCnt, testParams->mcanConfigParams.txMsgNum);
    DebugP_log("\nTxRx:: ThroughPut: %lld Msg/sec\r\n", numOfMsgPerSec);
    DebugP_log("\nTxRx:: HW Utilization: %lld%%\r\n", hwUtiln);
    if(hwUtiln < 85U)
    {
        testParams->testResult += CSL_EFAIL;
        testStatus += CSL_EFAIL;
    }
    testParams->isRun      = CSL_PASS;
    return testStatus;
}

int32_t App_mcanMsgArbTest(st_mcanTestcaseParams_t *testParams)
{
    int32_t  configStatus = CSL_PASS, testStatus = CSL_PASS;
    uint32_t loopCnt      = 1U, iterationCnt = 0U, txBufCnt, lastMsgId, idMask;
    uint32_t cnt = 0U, idShift;
    MCAN_ProtocolStatus protStatus;
    MCAN_ErrCntStatus    errCounter;
    MCAN_RxBufElement rxMsg;
    MCAN_TxFIFOStatus txFIFOStatus;
    MCAN_RxFIFOStatus fifoStatus;

    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, testParams->mcanConfigParams.intrEnable, (uint32_t)TRUE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLineSelectMask,
                        testParams->mcanConfigParams.intrLine);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLine,
                        1U);
    /* Enable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[0].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[0].bufferNum,
                    (uint32_t)TRUE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n", -1);
            }
        }
    }
    /* Enable interrupts for Tx FIFO/Queue */
    for (loopCnt = testParams->mcanConfigParams.ramConfig->txBufCnt ;
         loopCnt < (testParams->mcanConfigParams.ramConfig->txFIFOCnt +
                    testParams->mcanConfigParams.ramConfig->txBufCnt);
         loopCnt++)
    {
        /* Enable Transmission interrupt */
        configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                loopCnt,
                (uint32_t)TRUE);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nMCAN Tx FIFO Interrupt Enable FAILED...\n", -1);
        }
    }
    for (iterationCnt = 0U ;
         iterationCnt < testParams->mcanConfigParams.txMSGInterationCnt ;
         iterationCnt++)
    {
        /* Add transmission request for the message in single go.
         * This is not possible with current API implementation, so put MCAN into SW Init mode
         * then add transmission request for buffers and then put MCAN into Normal operational mode */
        /* Put MCAN in SW initialization mode */
        /* First write all messages to message RAM and then trigger their transmission in single go */
        for (loopCnt = 0U ;
             loopCnt < testParams->mcanConfigParams.txMsgNum ;
             loopCnt++)
        {
            MCAN_getTxFIFOQueStatus(gMcanBaseAddr, &txFIFOStatus);
            if (testParams->mcanConfigParams.txMsg[0].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
            {
                txBufCnt = testParams->mcanConfigParams.txMsg[0].bufferNum;
            }
            else
            {
                txBufCnt = txFIFOStatus.putIdx;
            }
            /* Write message to Msg RAM */
            MCAN_writeMsgRam(gMcanBaseAddr,
                             testParams->mcanConfigParams.txMsg[0].storageId,
                             txBufCnt,
                             &testParams->mcanConfigParams.txMsg[0].txElem);
            /* Add request for transmission */
            configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
            if (CSL_PASS != configStatus)
            {
                DebugP_log("\nError in Adding Transmission Request...\n", -1);
            }
            while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_TRANS_COMPLETE) ==
                                    MCAN_INTR_SRC_TRANS_COMPLETE))
            {
            }
            gMcanIsrIntr0Status = 0U;
        }

        /* wait till all messages has been received - not checking for each message individually */
        fifoStatus.num = (uint32_t)testParams->mcanConfigParams.txMsg[0U].rxBuffNum;
        MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
        while (fifoStatus.fillLvl < testParams->mcanConfigParams.txMsgNum)
        {
            MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
        }
        /* All the messages has been received */
        /* Checking for Errors */
        MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
        if ((0U == errCounter.recErrCnt) &&
            (0U == errCounter.canErrLogCnt))
        {
            MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
            /* Checking for Errors */
            if (((MCAN_ERR_CODE_NO_ERROR == protStatus.lastErrCode) ||
                 (MCAN_ERR_CODE_NO_CHANGE == protStatus.lastErrCode)) &&
                ((MCAN_ERR_CODE_NO_ERROR == protStatus.dlec) ||
                 (MCAN_ERR_CODE_NO_CHANGE == protStatus.dlec)) &&
                (0U == protStatus.pxe))
            {
                lastMsgId = 0U;
                /* Read messages from Rx FIFO */
                fifoStatus.num = (uint32_t)testParams->mcanConfigParams.txMsg[0U].rxBuffNum;
                for (loopCnt = 0U ;
                     loopCnt < testParams->mcanConfigParams.txMsgNum ;
                     loopCnt++)
                {
                    MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
                    MCAN_readMsgRam(gMcanBaseAddr,
                                    (uint32_t)MCAN_MEM_TYPE_FIFO,
                                    fifoStatus.getIdx,
                                    (uint32_t)fifoStatus.num,
                                    &rxMsg);
                    (void) MCAN_writeRxFIFOAck(gMcanBaseAddr,
                                               (uint32_t)fifoStatus.num,
                                               fifoStatus.getIdx);
                    if (rxMsg.xtd == 1U)
                    {
                        idMask = APP_MCAN_EXT_ID_MASK;
                        idShift = APP_MCAN_EXT_ID_SHIFT;
                    }
                    else
                    {
                        idMask = APP_MCAN_STD_ID_MASK;
                        idShift = APP_MCAN_STD_ID_SHIFT;
                    }
                    if ((lastMsgId & idMask) > (rxMsg.id & idMask))
                    {
                        testStatus += CSL_EFAIL;
                        DebugP_log("\nError: Messages did not receive in expected order.\n");
                        break;
                    }
                    else
                    {
                        lastMsgId = rxMsg.id;
                        DebugP_log("(%d)th message received: (ID: 0x%x)(TS: %d)\n", (loopCnt + 1U), ((rxMsg.id & idMask) >> idShift), rxMsg.rxts);
                        /* check received message against sent message */
                        for (cnt = 0U ;
                             cnt < testParams->mcanConfigParams.txMsgNum ;
                             cnt++)
                        {
                            if ((testParams->mcanConfigParams.txMsg[cnt].txElem.id & idMask) ==
                                (rxMsg.id & idMask))
                            {
                                break;
                            }
                        }
                        configStatus += App_mcanTxRxMessageCheck(
                                        testParams->mcanConfigParams.txMsg[cnt].txElem,
                                        rxMsg);
                        if(configStatus != CSL_PASS)
                        {
                            testStatus += CSL_EFAIL;
                            DebugP_log("\nTransmitted and received message does not match.\n");
                        }
                    }
                }
            }
            else
            {
                testStatus += CSL_EFAIL;
                DebugP_log("\nError in transmission/reception(Iteration Count): (%d).\n",
                           (iterationCnt + 1U));
            }
        }
        else
        {
            testStatus += CSL_EFAIL;
            DebugP_log("\nError Counters: Error in transmission/reception(Iteration Count): (%d).\n",
                       (iterationCnt + 1U));
        }
    }

    return testStatus;
}

int32_t App_mcanRxFIFOModes(st_mcanTestcaseParams_t *testParams)
{
    int32_t  configStatus = CSL_PASS, testStatus = CSL_PASS;
    uint32_t loopCnt      = 1U, loopBreakFlag = 1U, txBufCnt;
    uint32_t getIdx, putIdx;
    MCAN_ProtocolStatus protStatus;
    MCAN_ErrCntStatus    errCounter;
    MCAN_RxBufElementNoCpy rxMsg;
    MCAN_TxFIFOStatus txFIFOStatus;
    MCAN_RxFIFOStatus fifoStatus;
    MCAN_ConfigParams configParams;
    MCAN_TxBufElementNoCpy txElem = {0U};
    uint8_t txData[MCAN_MAX_PAYLOAD_BYTES];

    /* Initialize TX Data */
    for (loopCnt = 0U; loopCnt < MCAN_MAX_PAYLOAD_BYTES; loopCnt++)
    {
        txData[loopCnt] = loopCnt;
    }

    App_mcanInitTxElem(&txElem);
    txElem.data = &txData[0U];

    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, testParams->mcanConfigParams.intrEnable, (uint32_t)TRUE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLineSelectMask,
                        testParams->mcanConfigParams.intrLine);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLine,
                        1U);
    /* Enable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)TRUE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n", -1);
            }
        }
    }
    /* Enable interrupts for Tx FIFO/Queue */
    for (loopCnt = testParams->mcanConfigParams.ramConfig->txBufCnt ;
         loopCnt < (testParams->mcanConfigParams.ramConfig->txFIFOCnt +
                    testParams->mcanConfigParams.ramConfig->txBufCnt);
         loopCnt++)
    {
        /* Enable Transmission interrupt */
        configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                loopCnt,
                (uint32_t)TRUE);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nMCAN Tx FIFO Interrupt Enable FAILED...\n", -1);
        }
    }
    /* FIFO Block mode Test - Start */
    DebugP_log("FIFO 1 Blocking Mode Test:\r\n", -1);
    /* Accept non-matching messages into FIFO1 */
    /* Send messages until FIFO1 condition is reached */
    loopCnt = 0U;
    while (loopBreakFlag == 1U)
    {
        loopCnt++;
        MCAN_getTxFIFOQueStatus(gMcanBaseAddr, &txFIFOStatus);
        if (testParams->mcanConfigParams.txMsg[0U].storageId ==
                                                    MCAN_MEM_TYPE_BUF)
        {
            txBufCnt = testParams->mcanConfigParams.txMsg[0U].bufferNum;
        }
        else
        {
            txBufCnt = txFIFOStatus.putIdx;
        }
        /* Write message to Msg RAM-sending message with extended ID only */
        testParams->mcanConfigParams.txMsg[0U].txElem.id = loopCnt;
        testParams->mcanConfigParams.txMsg[0U].txElem.xtd = 1U;
        MCAN_writeMsgRamNoCpy(gMcanBaseAddr,
                         testParams->mcanConfigParams.txMsg[0U].storageId,
                         txBufCnt,
                         &txElem);
        /* Add request for transmission */
        configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
        if (CSL_PASS != configStatus)
        {
            DebugP_log("\nError in Adding Transmission Request...\n", -1);
        }
        while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_TRANS_COMPLETE) ==
                                MCAN_INTR_SRC_TRANS_COMPLETE))
        {
        }
        while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_RX_FIFO1_NEW_MSG) ==
                                MCAN_INTR_SRC_RX_FIFO1_NEW_MSG))
        {
        }
        if ((gMcanIsrIntr0Status & MCAN_INTR_SRC_RX_FIFO1_FULL) ==
                                MCAN_INTR_SRC_RX_FIFO1_FULL)
        {
            loopBreakFlag = 0U;
        }
        gMcanIsrIntr0Status = 0U;
        /* Checking for Errors */
        MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
        if ((0U == errCounter.recErrCnt) &&
            (0U == errCounter.canErrLogCnt))
        {
            MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
            /* Checking for Errors */
            if (((MCAN_ERR_CODE_NO_ERROR == protStatus.lastErrCode) ||
                 (MCAN_ERR_CODE_NO_CHANGE == protStatus.lastErrCode)) &&
                ((MCAN_ERR_CODE_NO_ERROR == protStatus.dlec) ||
                 (MCAN_ERR_CODE_NO_CHANGE == protStatus.dlec)) &&
                (0U == protStatus.pxe))
            {
            }
            else
            {
                testStatus += CSL_EFAIL;
                DebugP_log("\nError in transmission/reception(Iteration Count).\n");
                loopBreakFlag = 0U;
            }
        }
        else
        {
            testStatus += CSL_EFAIL;
            DebugP_log("\nError Counters: Error in transmission/reception.\n");
            loopBreakFlag = 0U;
        }
    }
    fifoStatus.num = MCAN_RX_FIFO_NUM_1;
    MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
    if (fifoStatus.fifoFull == 1U)
    {
        getIdx = fifoStatus.getIdx;
        putIdx = fifoStatus.putIdx;
        loopCnt++;
        /* Send another message to cause overflow */
        /* Write message to Msg RAM-sending message with extended ID only */
        testParams->mcanConfigParams.txMsg[0U].txElem.id = loopCnt;
        testParams->mcanConfigParams.txMsg[0U].txElem.xtd = 1U;
        MCAN_writeMsgRamNoCpy(gMcanBaseAddr,
                         testParams->mcanConfigParams.txMsg[0U].storageId,
                         txBufCnt,
                         &txElem);
        /* Add request for transmission */
        configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
        if (CSL_PASS != configStatus)
        {
            DebugP_log("\nError in Adding Transmission Request...\n", -1);
        }
        while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_TRANS_COMPLETE) ==
                                MCAN_INTR_SRC_TRANS_COMPLETE))
        {
        }
        while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_RX_FIFO1_MSG_LOST) ==
                                MCAN_INTR_SRC_RX_FIFO1_MSG_LOST))
        {
        }
        /* get FIFO status, since FIFO is in Blocking mode, latest message will
         * lost and get/put index will not be updated */
        MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
        if ((getIdx == fifoStatus.getIdx) && (putIdx == fifoStatus.putIdx))
        {
            DebugP_log("Get and Put Indices are not updated after sending message after FIFO Full Condition.\r\n", -1);
        }
        /* Read first message in FIFO - this should be oldest message(with EXT ID: 1) */
        MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
        MCAN_readMsgRamNoCpy(gMcanBaseAddr,
                        MCAN_MEM_TYPE_FIFO,
                        fifoStatus.getIdx,
                        (uint32_t)fifoStatus.num,
                        &rxMsg);
        (void) MCAN_writeRxFIFOAck(gMcanBaseAddr,
                                   (uint32_t)fifoStatus.num,
                                   fifoStatus.getIdx);
        if ((rxMsg.id >> MCAN_STD_ID_SHIFT) == (txElem.id >> MCAN_STD_ID_SHIFT))
        {
            DebugP_log("Oldest message is not overwritten, FIFO is in Blocking Mode.\r\n", -1);
        }
        else
        {
            testStatus += CSL_EFAIL;
            DebugP_log("Oldest message is overwritten, FIFO is not in Blocking Mode.\r\n", -1);
        }
    }
    else
    {
        testStatus += CSL_EFAIL;
    }
    /* FIFO Block mode Test - End */
    /* FIFO Overwrite mode Test - Start */
    DebugP_log("\nFIFO 0 Overwrite Mode Test:\r\n", -1);
    /* Accept non-matching messages into FIFO0(previously was FIFO1) */
    configParams.monEnable = testParams->mcanConfigParams.configParams->monEnable;
    configParams.asmEnable = testParams->mcanConfigParams.configParams->asmEnable;
    configParams.tsPrescalar = testParams->mcanConfigParams.configParams->tsPrescalar;
    configParams.tsSelect = testParams->mcanConfigParams.configParams->tsSelect;
    configParams.timeoutPreload = testParams->mcanConfigParams.configParams->timeoutPreload;
    configParams.timeoutCntEnable = testParams->mcanConfigParams.configParams->timeoutCntEnable;
    configParams.filterConfig.rrfe = testParams->mcanConfigParams.configParams->filterConfig.rrfe;
    configParams.filterConfig.rrfs = testParams->mcanConfigParams.configParams->filterConfig.rrfs;
    configParams.filterConfig.anfe = 0U;
    configParams.filterConfig.anfs = 0U;
    /* Put MCAN in SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(gMcanBaseAddr))
    {}
    MCAN_config(gMcanBaseAddr, &configParams);
    /* Take MCAN out of the SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanBaseAddr))
    {}
    /* Send messages until FIFO1 condition is reached */
    loopCnt = 0U;
    gMcanIsrIntr0Status = 0U;
    loopBreakFlag = 1U;
    while (loopBreakFlag == 1U)
    {
        loopCnt++;
        MCAN_getTxFIFOQueStatus(gMcanBaseAddr, &txFIFOStatus);
        if (testParams->mcanConfigParams.txMsg[0U].storageId ==
                                                    MCAN_MEM_TYPE_BUF)
        {
            txBufCnt = testParams->mcanConfigParams.txMsg[0U].bufferNum;
        }
        else
        {
            txBufCnt = txFIFOStatus.putIdx;
        }
        /* Write message to Msg RAM-sending message with extended ID only */
        testParams->mcanConfigParams.txMsg[0U].txElem.id = loopCnt;
        testParams->mcanConfigParams.txMsg[0U].txElem.xtd = 1U;
        MCAN_writeMsgRamNoCpy(gMcanBaseAddr,
                         testParams->mcanConfigParams.txMsg[0U].storageId,
                         txBufCnt,
                         &txElem);
        /* Add request for transmission */
        configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
        if (CSL_PASS != configStatus)
        {
            DebugP_log("\nError in Adding Transmission Request...\n", -1);
        }
        while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_TRANS_COMPLETE) ==
                                MCAN_INTR_SRC_TRANS_COMPLETE))
        {
        }
        while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ==
                                MCAN_INTR_SRC_RX_FIFO0_NEW_MSG))
        {
        }
        if ((gMcanIsrIntr0Status & MCAN_INTR_SRC_RX_FIFO0_FULL) ==
                                MCAN_INTR_SRC_RX_FIFO0_FULL)
        {
            loopBreakFlag = 0U;
        }
        gMcanIsrIntr0Status = 0U;
        /* Checking for Errors */
        MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
        if ((0U == errCounter.recErrCnt) &&
            (0U == errCounter.canErrLogCnt))
        {
            MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
            /* Checking for Errors */
            if (((MCAN_ERR_CODE_NO_ERROR == protStatus.lastErrCode) ||
                 (MCAN_ERR_CODE_NO_CHANGE == protStatus.lastErrCode)) &&
                ((MCAN_ERR_CODE_NO_ERROR == protStatus.dlec) ||
                 (MCAN_ERR_CODE_NO_CHANGE == protStatus.dlec)) &&
                (0U == protStatus.pxe))
            {
            }
            else
            {
                testStatus += CSL_EFAIL;
                DebugP_log("\nError in transmission/reception(Iteration Count).\n");
                loopBreakFlag = 0U;
            }
        }
        else
        {
            testStatus += CSL_EFAIL;
            DebugP_log("\nError Counters: Error in transmission/reception.\n");
            loopBreakFlag = 0U;
        }
    }
    fifoStatus.num = MCAN_RX_FIFO_NUM_0;
    MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
    if (fifoStatus.fifoFull == 1U)
    {
        getIdx = fifoStatus.getIdx;
        putIdx = fifoStatus.putIdx;
        loopCnt++;
        /* Send another message to cause overflow */
        /* Write message to Msg RAM-sending message with extended ID only */
        testParams->mcanConfigParams.txMsg[0U].txElem.id = loopCnt;
        testParams->mcanConfigParams.txMsg[0U].txElem.xtd = 1U;
        MCAN_writeMsgRamNoCpy(gMcanBaseAddr,
                         testParams->mcanConfigParams.txMsg[0U].storageId,
                         txBufCnt,
                         &txElem);
        /* Add request for transmission */
        configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
        if (CSL_PASS != configStatus)
        {
            DebugP_log("\nError in Adding Transmission Request...\n", -1);
        }
        while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_TRANS_COMPLETE) ==
                                MCAN_INTR_SRC_TRANS_COMPLETE))
        {
        }
        while (!((gMcanIsrIntr0Status & MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ==
                                MCAN_INTR_SRC_RX_FIFO0_NEW_MSG))
        {
        }
        gMcanIsrIntr0Status = 0U;
        /* get FIFO status, since FIFO is in Blocking mode, latest message will
         * lost and get/put index will not be updated */
        MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
        if ((getIdx == (fifoStatus.getIdx - 1U)) && (putIdx == (fifoStatus.putIdx - 1U)))
        {
            DebugP_log("Get and Put Indices are  updated after sending message after FIFO Full Condition.\r\n", -1);
        }
        /* Read first message in FIFO - this should be second oldest message(with EXT ID: 2) */
        MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
        MCAN_readMsgRamNoCpy(gMcanBaseAddr,
                        MCAN_MEM_TYPE_FIFO,
                        fifoStatus.getIdx,
                        (uint32_t)fifoStatus.num,
                        &rxMsg);
        (void) MCAN_writeRxFIFOAck(gMcanBaseAddr,
                                   (uint32_t)fifoStatus.num,
                                   fifoStatus.getIdx);
        if ((rxMsg.id >> MCAN_STD_ID_SHIFT) == (txElem.id >> MCAN_STD_ID_SHIFT))
        {
            DebugP_log("First message in the FIFO is second oldest message.\r\n", -1);
        }
        else
        {
            testStatus += CSL_EFAIL;
            DebugP_log("First message in the FIFO is not second oldest message.\r\n", -1);
        }
        MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
        while (fifoStatus.fillLvl > 0U)
        {
            MCAN_readMsgRamNoCpy(gMcanBaseAddr,
                            MCAN_MEM_TYPE_FIFO,
                            fifoStatus.getIdx,
                            (uint32_t)fifoStatus.num,
                            &rxMsg);
            (void) MCAN_writeRxFIFOAck(gMcanBaseAddr,
                                       (uint32_t)fifoStatus.num,
                                       fifoStatus.getIdx);
            MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
        }
        if ((rxMsg.id >> MCAN_STD_ID_SHIFT) == (txElem.id >> MCAN_STD_ID_SHIFT))
        {
            DebugP_log("Oldest message is overwritten, FIFO is in Overwriting Mode.\r\n", -1);
        }
        else
        {
            testStatus += CSL_EFAIL;
            DebugP_log("Oldest message is not overwritten, FIFO is not in Overwriting Mode.\r\n", -1);
        }
    }
    else
    {
        testStatus += CSL_EFAIL;
    }
    /* FIFO Overwrite mode Test - End */

    /* Disable Interrupt */
    MCAN_enableIntr(gMcanBaseAddr, MCAN_INTR_MASK_ALL, (uint32_t)FALSE);

    return testStatus;
}

static int32_t App_mcanTxTestBusMonitor(st_mcanTestcaseParams_t *testParams)
{
    int32_t  configStatus = CSL_PASS, testStatus = CSL_PASS;
    uint32_t loopCnt      = 1U, iterationCnt = 0U, intrMask, txBufCnt;
    uint32_t bitPos = 0U;
    MCAN_RxBufElement rxMsg;
    MCAN_TxFIFOStatus txFIFOStatus;

    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, testParams->mcanConfigParams.intrEnable, (uint32_t)TRUE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLineSelectMask,
                        testParams->mcanConfigParams.intrLine);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLine,
                        1U);
    /* Enable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)TRUE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n", -1);
            }
        }
    }
    /* Enable interrupts for Tx FIFO/Queue */
    for (loopCnt = testParams->mcanConfigParams.ramConfig->txBufCnt ;
         loopCnt < (testParams->mcanConfigParams.ramConfig->txFIFOCnt +
                    testParams->mcanConfigParams.ramConfig->txBufCnt);
         loopCnt++)
    {
        /* Enable Transmission interrupt */
        configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                loopCnt,
                (uint32_t)TRUE);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nMCAN Tx FIFO Interrupt Enable FAILED...\n", -1);
        }
    }

    for (iterationCnt = 0U ;
         iterationCnt < testParams->mcanConfigParams.txMSGInterationCnt ;
         iterationCnt++)
    {
        for (loopCnt = 0U ;
             loopCnt < testParams->mcanConfigParams.txMsgNum ;
             loopCnt++)
        {
            MCAN_getTxFIFOQueStatus(gMcanBaseAddr, &txFIFOStatus);
            if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
            {
                txBufCnt = testParams->mcanConfigParams.txMsg[loopCnt].bufferNum;
            }
            else
            {
                txBufCnt = txFIFOStatus.putIdx;
            }
            /* Calculate Rx Interrupt Mask */
            if(testParams->mcanConfigParams.txMsg[loopCnt].rxMSGStorageId == MCAN_MEM_TYPE_BUF)
            {
                intrMask = MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG;
            }
            else
            {
                if(testParams->mcanConfigParams.txMsg[loopCnt].rxBuffNum == MCAN_RX_FIFO_NUM_0)
                {
                    intrMask = MCAN_INTR_SRC_RX_FIFO0_NEW_MSG;
                }
                else
                {
                    intrMask = MCAN_INTR_SRC_RX_FIFO1_NEW_MSG;
                }

            }
            (void)intrMask; /* Presently set but not used. Suppress warning */
            gMcanIsrIntr0Status = 0U;
            // App_mcanPrintTxMsg(&testParams->mcanConfigParams.txMsg[loopCnt].txElem);
            /* Write message to Msg RAM */
            MCAN_writeMsgRam(gMcanBaseAddr,
                             testParams->mcanConfigParams.txMsg[loopCnt].storageId,
                             txBufCnt,
                             &testParams->mcanConfigParams.txMsg[loopCnt].txElem);
            /* Add request for transmission */
            configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
            if (CSL_PASS != configStatus)
            {
                DebugP_log("\nError in Adding Transmission Request...\n", -1);
            }
            bitPos = (1U << txBufCnt);
            if ((MCAN_getTxBufReqPend(gMcanBaseAddr) & (bitPos)) != (bitPos))
            {
                /* Send CAN Msg with ID as 0x4U from any node or CAN tool */
                while  (!(((gMcanIsrIntr0Status & MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG) ==
                                                MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG) ||
                        ((gMcanIsrIntr0Status & MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ==
                                                MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) ||
                        ((gMcanIsrIntr0Status & MCAN_INTR_SRC_RX_FIFO1_NEW_MSG) ==
                                                MCAN_INTR_SRC_RX_FIFO1_NEW_MSG)))
                {
                }
                configStatus = App_mcanReadRxMSG(&rxMsg, gMcanIsrIntr0Status);
                if((configStatus != CSL_PASS) && (((rxMsg.id >> MCAN_STD_ID_SHIFT) & MCAN_STD_ID_MASK) != 0x4U))
                {
                    DebugP_log("\nUnable to read received message(Iteration Count:Message Number): (%d,%d).\n", (iterationCnt + 1U), (loopCnt + 1U));
                    testStatus = CSL_EFAIL;
                }
                gMcanIsrIntr0Status = 0U;
            }
        }
    }
    /* Disable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)FALSE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n", -1);
            }
        }
    }

    return testStatus;
}

/* Test Setup: Short MCAN_HIGH and MCAN_LOW Pins */
static int32_t App_mcanTxTestBusOff(st_mcanTestcaseParams_t *testParams)
{
    int32_t  configStatus = CSL_PASS, testStatus = CSL_PASS;
    uint32_t loopCnt      = 1U, iterationCnt = 0U, intrMask, txBufCnt;
    MCAN_TxFIFOStatus txFIFOStatus;

    /* Enable Interrupts */
    MCAN_enableIntr(gMcanBaseAddr, testParams->mcanConfigParams.intrEnable, (uint32_t)TRUE);
    /* Select Interrupt Line */
    MCAN_selectIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLineSelectMask,
                        testParams->mcanConfigParams.intrLine);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr,
                        testParams->mcanConfigParams.intrLine,
                        1U);
    /* Enable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)TRUE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n", -1);
            }
        }
    }
    /* Enable interrupts for Tx FIFO/Queue */
    for (loopCnt = testParams->mcanConfigParams.ramConfig->txBufCnt ;
         loopCnt < (testParams->mcanConfigParams.ramConfig->txFIFOCnt +
                    testParams->mcanConfigParams.ramConfig->txBufCnt);
         loopCnt++)
    {
        /* Enable Transmission interrupt */
        configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                loopCnt,
                (uint32_t)TRUE);
        if(configStatus != CSL_PASS)
        {
            DebugP_log("\nMCAN Tx FIFO Interrupt Enable FAILED...\n", -1);
        }
    }

    for (iterationCnt = 0U ;
         iterationCnt < testParams->mcanConfigParams.txMSGInterationCnt ;
         iterationCnt++)
    {
        for (loopCnt = 0U ;
             loopCnt < testParams->mcanConfigParams.txMsgNum ;
             loopCnt++)
        {
            MCAN_getTxFIFOQueStatus(gMcanBaseAddr, &txFIFOStatus);
            if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
            {
                txBufCnt = testParams->mcanConfigParams.txMsg[loopCnt].bufferNum;
            }
            else
            {
                txBufCnt = txFIFOStatus.putIdx;
            }
            /* Calculate Rx Interrupt Mask */
            if(testParams->mcanConfigParams.txMsg[loopCnt].rxMSGStorageId == MCAN_MEM_TYPE_BUF)
            {
                intrMask = MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG;
            }
            else
            {
                if(testParams->mcanConfigParams.txMsg[loopCnt].rxBuffNum == MCAN_RX_FIFO_NUM_0)
                {
                    intrMask = MCAN_INTR_SRC_RX_FIFO0_NEW_MSG;
                }
                else
                {
                    intrMask = MCAN_INTR_SRC_RX_FIFO1_NEW_MSG;
                }

            }
            (void)intrMask; /* Kill warning as presently varaible set but not used */
            gMcanIsrIntr0Status = 0U;
            // App_mcanPrintTxMsg(&testParams->mcanConfigParams.txMsg[loopCnt].txElem);
            /* Write message to Msg RAM */
            MCAN_writeMsgRam(gMcanBaseAddr,
                             testParams->mcanConfigParams.txMsg[loopCnt].storageId,
                             txBufCnt,
                             &testParams->mcanConfigParams.txMsg[loopCnt].txElem);
            /* Add request for transmission */
            volatile uint32_t myTemp = 1U;
            MCAN_ProtocolStatus protStatus;
            while(myTemp)
            {
                configStatus += MCAN_txBufAddReq(gMcanBaseAddr, txBufCnt);
                MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
                /* Checking for Errors */
                if ((1U == protStatus.busOffStatus) &&
                     (1U == protStatus.errPassive))
                {
                    testStatus = CSL_PASS;
                    break;
                }
            }
        }
    }
    /* Disable interrupts for Tx Buffers */
    for (loopCnt = 0U ;
         loopCnt < testParams->mcanConfigParams.txMsgNum ;
         loopCnt++)
    {
        if (testParams->mcanConfigParams.txMsg[loopCnt].storageId ==
                                                        MCAN_MEM_TYPE_BUF)
        {
            /* Enable Transmission interrupt */
            configStatus += MCAN_txBufTransIntrEnable(gMcanBaseAddr,
                    testParams->mcanConfigParams.txMsg[loopCnt].bufferNum,
                    (uint32_t)FALSE);
            if(configStatus != CSL_PASS)
            {
                DebugP_log("\nMCAN Tx Buffer Interrupt Enable FAILED...\n", -1);
            }
        }
    }

    return testStatus;
}

static void App_mcanInitTxElem(MCAN_TxBufElementNoCpy *txMsg)
{
    /* Standard message identifier 11 bit, stored into ID[28-18] */
    txMsg->id  = ((uint32_t)0x3U << 18U), /* Identifier */
    /* Transmit data frame */
    txMsg->rtr = FALSE;
    /* Standard message id */
    txMsg->xtd = FALSE;
    txMsg->esi = FALSE;
    /* Payload size is 64 bytes */
    txMsg->dlc = 0xFU;
    /* Bit Rate Switch */
    txMsg->brs = TRUE;
    /* CAN FD Frame Format */
    txMsg->fdf = TRUE;
    txMsg->efc = TRUE;
    txMsg->mm  = 0xAAU;
    txMsg->data = NULL;
}

static void App_mcanCompareBitTimeParams(MCAN_BitTimingParams *setPrms,
                MCAN_BitTimingParams *dstPrms)
{
    if ((setPrms->nomRatePrescalar != dstPrms->nomRatePrescalar) ||
        (setPrms->nomTimeSeg1 != dstPrms->nomTimeSeg1) ||
        (setPrms->nomTimeSeg2 != dstPrms->nomTimeSeg2) ||
        (setPrms->nomSynchJumpWidth != dstPrms->nomSynchJumpWidth) ||
        (setPrms->dataRatePrescalar != dstPrms->dataRatePrescalar) ||
        (setPrms->dataTimeSeg1 != dstPrms->dataTimeSeg1) ||
        (setPrms->dataTimeSeg2 != dstPrms->dataTimeSeg2) ||
        (setPrms->dataSynchJumpWidth != dstPrms->dataSynchJumpWidth))
   {
        DebugP_assert(TRUE);
   }
}

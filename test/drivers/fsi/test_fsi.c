/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TaskP.h>
#include <drivers/fsi.h>
#include <drivers/pinmux.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "test_fsi_common.h"
#include "test_fsi_rx_task.h"
#include "test_fsi_tx_task.h"

/* ========================================================================== */
/*                         Global Varialbe Declarations                       */
/* ========================================================================== */
/* Semaphore to track end of rx_task and tx_task */
SemaphoreP_Object gFsiTaskDoneSemaphoreObj;
SemaphoreP_Object gFsiTxRx_SyncSemaphoreObj;

uint8_t gFsiRxTaskStack[FSI_TASK_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gFsiRxTaskObject;

uint8_t gFsiTxTaskStack[FSI_TASK_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gFsiTxTaskObject;

/* +1 to allocate full 16 elements FSI_MAX_VALUE_BUF_PTR_OFF is 15 */
extern uint16_t gRxBufData[FSI_MAX_VALUE_BUF_PTR_OFF + 1U];
extern uint16_t gTxBufData[FSI_MAX_VALUE_BUF_PTR_OFF + 1U];
uint32_t        gDataWordArray[FSI_MAX_VALUE_BUF_PTR_OFF + 1U] =
                {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U,
                 9U, 10U, 11U, 12U, 13U, 14U, 15U, 16U};
/* TX Functional Clock should be less than SYSCLK / 2 */
uint32_t        gPrescalValArray[FSI_MAX_VALUE_BUF_PTR_OFF + 1U] =
                {2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U,
                 16U, 32U, 64U, 96U, 128U, 160U, 200U, 255U};

FSI_RxTestParams gFsiRxTestParams = {0U};
FSI_TxTestParams gFsiTxTestParams = {0U};

static int32_t Fsi_appCompareData(uint16_t *txBufPtr, uint16_t *rxBufPtr);
static void Fsi_appTxRxTestParamsInit(FSI_RxTestParams *rxTestParms,
                                      FSI_TxTestParams *txTestParms);
void test_fsi_txrx(void *args);
static void test_fsi_set_params(FSI_MainTestParams *testParams, uint32_t testCaseNo);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    FSI_MainTestParams  testParams;

    Drivers_open();

    UNITY_BEGIN();

    test_fsi_set_params(&testParams, 1514);
    RUN_TEST(test_fsi_txrx, 1514, (void*)&testParams);
    test_fsi_set_params(&testParams, 1515);
    RUN_TEST(test_fsi_txrx, 1515, (void*)&testParams);
    test_fsi_set_params(&testParams, 1516);
    RUN_TEST(test_fsi_txrx, 1516, (void*)&testParams);
    test_fsi_set_params(&testParams, 1517);
    /* PreScaler and DataWord RampUp Test*/
    for (uint32_t idx = 0U; idx < (FSI_MAX_VALUE_BUF_PTR_OFF + 1U); idx++)
    {
        gFsiRxTestParams.frameDataSize = gDataWordArray[idx];
        gFsiTxTestParams.frameDataSize = gDataWordArray[idx];
        gFsiRxTestParams.prescaleVal = gPrescalValArray[idx];
        gFsiTxTestParams.prescaleVal = gPrescalValArray[idx];
        RUN_TEST(test_fsi_txrx, 1517, (void*)&testParams);
    }
    test_fsi_set_params(&testParams, 1518);
    RUN_TEST(test_fsi_txrx, 1518, (void*)&testParams);
    test_fsi_set_params(&testParams, 1519);
    RUN_TEST(test_fsi_txrx, 1519, (void*)&testParams);
    test_fsi_set_params(&testParams, 1520);
    RUN_TEST(test_fsi_txrx, 1520, (void*)&testParams);
    test_fsi_set_params(&testParams, 1523);
    RUN_TEST(test_fsi_txrx, 1523, (void*)&testParams);
    test_fsi_set_params(&testParams, 1525);
    RUN_TEST(test_fsi_txrx, 1525, (void*)&testParams);
    test_fsi_set_params(&testParams, 1527);
    RUN_TEST(test_fsi_txrx, 1527, (void*)&testParams);
    test_fsi_set_params(&testParams, 1528);
    RUN_TEST(test_fsi_txrx, 1528, (void*)&testParams);
    test_fsi_set_params(&testParams, 1529);
    RUN_TEST(test_fsi_txrx, 1529, (void*)&testParams);
    /* FSI All Instances Test, FSI TX0 Connected to FSI RX0, FSI RX1, FSI RX2
       FSI TX1 Connected to FSI RX3, FSI RX4, FSI RX5 */
    test_fsi_set_params(&testParams, 1526);
    gFsiRxTestParams.baseAddr = CONFIG_FSI_RX0_BASE_ADDR;
    gFsiRxTestParams.intrLine = CONFIG_FSI_RX0_INTR1;
    gFsiTxTestParams.baseAddr = CONFIG_FSI_TX0_BASE_ADDR;
    gFsiTxTestParams.intrLine = CONFIG_FSI_TX0_INTR1;
    RUN_TEST(test_fsi_txrx, 1526, (void*)&testParams);
#if defined (SOC_AM64X) || defined (SOC_AM243X)
    gFsiRxTestParams.baseAddr = CONFIG_FSI_RX1_BASE_ADDR;
    gFsiRxTestParams.intrLine = CONFIG_FSI_RX1_INTR1;
    RUN_TEST(test_fsi_txrx, 1526, (void*)&testParams);
    gFsiRxTestParams.baseAddr = CONFIG_FSI_RX2_BASE_ADDR;
    gFsiRxTestParams.intrLine = CONFIG_FSI_RX2_INTR1;
    RUN_TEST(test_fsi_txrx, 1526, (void*)&testParams);
    test_fsi_set_params(&testParams, 1530);
    /* Added below check to test all instance in AM243/AM64X EVM.
       In case of AM243X-LP only 1 FSI TX and 3 FSI RX instances available */
#if (CONFIG_FSI_TX_NUM_INSTANCES > 1)
    gFsiTxTestParams.baseAddr = CONFIG_FSI_TX1_BASE_ADDR;
    gFsiTxTestParams.intrLine = CONFIG_FSI_TX1_INTR1;
    gFsiRxTestParams.baseAddr = CONFIG_FSI_RX3_BASE_ADDR;
    gFsiRxTestParams.intrLine = CONFIG_FSI_RX3_INTR1;
    RUN_TEST(test_fsi_txrx, 1530, (void*)&testParams);
    gFsiRxTestParams.baseAddr = CONFIG_FSI_RX4_BASE_ADDR;
    gFsiRxTestParams.intrLine = CONFIG_FSI_RX4_INTR1;
    RUN_TEST(test_fsi_txrx, 1530, (void*)&testParams);
    gFsiRxTestParams.baseAddr = CONFIG_FSI_RX5_BASE_ADDR;
    gFsiRxTestParams.intrLine = CONFIG_FSI_RX5_INTR1;
    RUN_TEST(test_fsi_txrx, 1530, (void*)&testParams);
#else
    gFsiRxTestParams.baseAddr = CONFIG_FSI_RX0_BASE_ADDR;
    gFsiRxTestParams.intrLine = CONFIG_FSI_RX0_INTR1;
    gFsiTxTestParams.baseAddr = CONFIG_FSI_TX0_BASE_ADDR;
    gFsiTxTestParams.intrLine = CONFIG_FSI_TX0_INTR1;
    RUN_TEST(test_fsi_txrx, 1530, (void*)&testParams);
#endif
#else
    gFsiTxTestParams.baseAddr = CONFIG_FSI_TX1_BASE_ADDR;
    gFsiTxTestParams.intrLine = CONFIG_FSI_TX1_INTR1;
    gFsiRxTestParams.baseAddr = CONFIG_FSI_RX1_BASE_ADDR;
    gFsiRxTestParams.intrLine = CONFIG_FSI_RX1_INTR1;
    RUN_TEST(test_fsi_txrx, 1530, (void*)&testParams);
    gFsiTxTestParams.baseAddr = CONFIG_FSI_TX2_BASE_ADDR;
    gFsiTxTestParams.intrLine = CONFIG_FSI_TX2_INTR1;
    gFsiRxTestParams.baseAddr = CONFIG_FSI_RX2_BASE_ADDR;
    gFsiRxTestParams.intrLine = CONFIG_FSI_RX2_INTR1;
    RUN_TEST(test_fsi_txrx, 1530, (void*)&testParams);
    gFsiTxTestParams.baseAddr = CONFIG_FSI_TX3_BASE_ADDR;
    gFsiTxTestParams.intrLine = CONFIG_FSI_TX3_INTR1;
    gFsiRxTestParams.baseAddr = CONFIG_FSI_RX3_BASE_ADDR;
    gFsiRxTestParams.intrLine = CONFIG_FSI_RX3_INTR1;
    RUN_TEST(test_fsi_txrx, 1530, (void*)&testParams);
#endif
    test_fsi_set_params(&testParams, 1647);
    RUN_TEST(test_fsi_txrx, 1647, (void*)&testParams);
    test_fsi_set_params(&testParams, 1521);
    RUN_TEST(test_fsi_txrx, 1521, (void*)&testParams);
    test_fsi_set_params(&testParams, 1522);
    RUN_TEST(test_fsi_txrx, 1522, (void*)&testParams);
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    test_fsi_set_params(&testParams, 4081);
    RUN_TEST(test_fsi_txrx, 4081, (void*)&testParams);
#endif

    UNITY_END();

    Drivers_close();
}

void test_fsi_txrx(void *args)
{
    int32_t         status;
    TaskP_Params    txTaskParms, rxTaskParms;
    FSI_MainTestParams *testParams = (FSI_MainTestParams*)args;
    FSI_TxTestParams *txTestParams = testParams->txTestParams;
    FSI_RxTestParams *rxTestParams = testParams->rxTestParams;

    status = SemaphoreP_constructBinary(&gFsiTxRx_SyncSemaphoreObj, 0);
    DebugP_assert(status == SystemP_SUCCESS);
    status = SemaphoreP_constructCounting(&txTestParams->taskDoneSemaphoreObj, 0, 2);
    DebugP_assert(status == SystemP_SUCCESS);
    status = SemaphoreP_constructCounting(&rxTestParams->taskDoneSemaphoreObj, 0, 2);
    DebugP_assert(status == SystemP_SUCCESS);

    TaskP_Params_init(&rxTaskParms);
    rxTaskParms.name = "FSI-RX Task";
    rxTaskParms.stackSize = FSI_TASK_STACK_SIZE;
    rxTaskParms.stack = gFsiRxTaskStack;
    rxTaskParms.priority = FSI_TASK_PRIORITY;
    rxTaskParms.args = rxTestParams;
    rxTaskParms.taskMain = testParams->testCaseRxFxnPtr;
    status = TaskP_construct(&gFsiRxTaskObject, &rxTaskParms);
    DebugP_assert(status == SystemP_SUCCESS);

    TaskP_Params_init(&txTaskParms);
    txTaskParms.name = "FSI-TX Task";
    txTaskParms.stackSize = FSI_TASK_STACK_SIZE;
    txTaskParms.stack = gFsiTxTaskStack;
    txTaskParms.priority = FSI_TASK_PRIORITY;
    txTaskParms.args = txTestParams;
    txTaskParms.taskMain = testParams->testCaseTxFxnPtr;;
    status = TaskP_construct(&gFsiTxTaskObject, &txTaskParms);
    DebugP_assert(status == SystemP_SUCCESS);

    SemaphoreP_pend(&txTestParams->taskDoneSemaphoreObj, SystemP_WAIT_FOREVER);
    SemaphoreP_pend(&rxTestParams->taskDoneSemaphoreObj, SystemP_WAIT_FOREVER);

    /* Compare data */
    if ((rxTestParams->rxFrameWDTest != TRUE) && (rxTestParams->rxPingWDTest != TRUE) )
    {
        status = Fsi_appCompareData(gTxBufData, gRxBufData);
        DebugP_assert(status == SystemP_SUCCESS);
    }

    SemaphoreP_destruct(&gFsiTxRx_SyncSemaphoreObj);
    SemaphoreP_destruct(&txTestParams->taskDoneSemaphoreObj);
    SemaphoreP_destruct(&rxTestParams->taskDoneSemaphoreObj);
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

/*
 * Unity framework required functions
 */
void setUp(void)
{
}

void tearDown(void)
{
}

/*
 * Testcases
 */
static void test_fsi_set_params(FSI_MainTestParams *testParams, uint32_t testCaseNo)
{
    testParams->testCaseTxFxnPtr = fsi_tx_main;
    testParams->testCaseRxFxnPtr = fsi_rx_main;
    testParams->rxTestParams = &gFsiRxTestParams;
    testParams->txTestParams = &gFsiTxTestParams;

    /* memset both TX/RX buffers to 0 for each testcase */
    memset(gTxBufData, 0U, sizeof(gTxBufData));
    memset(gRxBufData, 0U, sizeof(gRxBufData));
    switch (testCaseNo)
    {
        case 1514:
        case 1525:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            break;
        case 1515:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            gFsiRxTestParams.intrLine = CONFIG_FSI_RX0_INTR2;
            gFsiRxTestParams.intrNum  = FSI_INT2;
            gFsiTxTestParams.intrLine = CONFIG_FSI_TX0_INTR2;
            gFsiTxTestParams.intrNum  = FSI_INT2;
            break;
        case 1516:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            gFsiRxTestParams.numLane  = FSI_DATA_WIDTH_2_LANE;
            gFsiTxTestParams.numLane  = FSI_DATA_WIDTH_2_LANE;
            break;
        case 1517:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            break;
        case 1518:
        case 1527:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            gFsiRxTestParams.userDefinedCrc  = TRUE;
            gFsiTxTestParams.userDefinedCrc  = TRUE;
            break;
        case 1519:
        case 1528:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            gFsiRxTestParams.testEccFlag  = TRUE;
            gFsiTxTestParams.testEccFlag  = TRUE;
            gFsiRxTestParams.frameDataSize = 2U;
            gFsiTxTestParams.frameDataSize = 2U;
            break;
        case 1520:
        case 1529:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            gFsiTxTestParams.intrEvt = FSI_TX_EVT_PING_HW_TRIG;
            gFsiRxTestParams.intrEvt = FSI_RX_EVT_PING_FRAME;
            testParams->testCaseTxFxnPtr = fsi_tx_hwPingTest;
            testParams->testCaseRxFxnPtr = fsi_rx_hwPingTest;
            break;
        case 1523:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            gFsiRxTestParams.numLane  = FSI_DATA_WIDTH_2_LANE;
            gFsiTxTestParams.numLane  = FSI_DATA_WIDTH_2_LANE;
            gFsiRxTestParams.delayLineCtrlTest  = TRUE;
            /* TX Delay Test is applicable for AM263X only */
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            gFsiTxTestParams.delayLineCtrlTest  = TRUE;
#endif
            break;
        case 1526:
        case 1530:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            break;
        case 1647:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            testParams->testCaseTxFxnPtr = fsi_tx_negativeTest;
            testParams->testCaseRxFxnPtr = fsi_rx_negativeTest;
            break;
        case 1521:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            gFsiTxTestParams.intrEvt = FSI_TX_EVT_PING_HW_TRIG;
            gFsiRxTestParams.intrEvt = FSI_RX_EVT_PING_WD_TIMEOUT;
            gFsiRxTestParams.rxPingWDTest = TRUE;
            gFsiTxTestParams.rxPingWDTest = TRUE;
            testParams->testCaseTxFxnPtr = fsi_tx_hwPingTest;
            testParams->testCaseRxFxnPtr = fsi_rx_hwPingTest;
            break;
        case 1522:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            gFsiRxTestParams.intrEvt = FSI_RX_EVT_FRAME_WD_TIMEOUT;
            gFsiRxTestParams.rxFrameWDTest = TRUE;
            gFsiTxTestParams.rxFrameWDTest = TRUE;
            break;
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
        case 4081:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            gFsiRxTestParams.udataFilterTest = TRUE;
            gFsiTxTestParams.udataFilterTest = TRUE;
            gFsiTxTestParams.userData  = FSI_APP_TX_USER_DATA_FILTER_VALUE;
            gFsiRxTestParams.userData  = FSI_APP_RX_USER_DATA_FILTER_VALUE;
            break;
        case 4233:
            Fsi_appTxRxTestParamsInit(&gFsiRxTestParams, &gFsiTxTestParams);
            gFsiRxTestParams.rxTriggerTest = TRUE;
            gFsiTxTestParams.rxTriggerTest = TRUE;
            break;
#endif
    }

    return;
}

static void Fsi_appTxRxTestParamsInit(FSI_RxTestParams *rxTestParms,
                                      FSI_TxTestParams *txTestParms)
{
    rxTestParms->baseAddr = CONFIG_FSI_RX0_BASE_ADDR;
    rxTestParms->frameDataSize = FSI_APP_FRAME_DATA_WORD_SIZE;
    rxTestParms->numLane = FSI_APP_N_LANES;
    rxTestParms->intrLine = CONFIG_FSI_RX0_INTR1;
    rxTestParms->intrNum  = FSI_INT1;
    rxTestParms->intrEvt  = FSI_RX_EVT_DATA_FRAME;
    rxTestParms->testEccFlag  = FALSE;
    rxTestParms->userDefinedCrc = FALSE;
    rxTestParms->delayLineCtrlTest = FALSE;
    rxTestParms->rxFrameWDTest = FALSE;
    rxTestParms->rxPingWDTest = FALSE;
    rxTestParms->udataFilterTest = FALSE;
    rxTestParms->userData  = FSI_APP_TX_USER_DATA;
    rxTestParms->rxTriggerTest = FALSE;
    txTestParms->baseAddr = CONFIG_FSI_TX0_BASE_ADDR;
    txTestParms->frameDataSize = FSI_APP_FRAME_DATA_WORD_SIZE;
    txTestParms->prescaleVal = FSI_APP_TX_PRESCALER_VAL;
    txTestParms->intrLine = CONFIG_FSI_TX0_INTR1;
    txTestParms->intrNum  = FSI_INT1;
    txTestParms->intrEvt  = FSI_TX_EVT_FRAME_DONE;
    txTestParms->userData  = FSI_APP_TX_USER_DATA;
    txTestParms->numLane  = FSI_APP_N_LANES;
    txTestParms->frameDataTag  = FSI_APP_TX_DATA_FRAME_TAG;
    txTestParms->frameType  = FSI_FRAME_TYPE_NWORD_DATA;
    txTestParms->userDefinedCrc = FALSE;
    txTestParms->testEccFlag  = FALSE;
    txTestParms->delayLineCtrlTest = FALSE;
    txTestParms->rxFrameWDTest = FALSE;
    txTestParms->rxPingWDTest = FALSE;
    txTestParms->udataFilterTest = FALSE;
    txTestParms->rxTriggerTest = FALSE;
    return;
}

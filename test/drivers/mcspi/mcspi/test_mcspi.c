/*
 *  Copyright (C) 2021-22 Texas Instruments Incorporated
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

/* This UT demonstrates the McSPI RX and TX operation configured
 * in different configurations and all possible MCSPI instances that can be
 * configured. MCSPI2 instance is muxed with UART, so it is not tested.
 * In case of AM243 LP we, have only 3 instances available.
 *
 * This example sends a known data in the TX mode of length APP_MCSPI_MSGSIZE
 * and then receives the same in RX mode. Internal pad level loopback mode
 * is enabled to receive data.
 *
 * When transfer is completed, TX and RX buffer data are compared.
 * If data is matched, test result is passed otherwise failed.
 */
#include "string.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <unity.h>

#if (CONFIG_MCSPI_NUM_INSTANCES > 2)
#include <drivers/mcspi/v0/dma/mcspi_dma.h>
#endif

/* Task Macros */
#define MCSPI_TASK_PRIORITY   (8U)
#define MCSPI_TASK_STACK_SIZE (4U * 1024U)

/* Number of Word count */
#define APP_MCSPI_MSGSIZE                   (100U)
#define APP_MCSPI_TXONLYMSGSIZE             (5U)
#define APP_MCSPI_TRANSFER_LOOPCOUNT        (5U)
#define APP_MCSPI_PERF_LOOP_ITER_CNT        (1000U)

#if defined(SOC_AM263X)

#define MCSPI0_BASE_ADDRESS             (CSL_MCSPI0_U_BASE)
#define MCSPI1_BASE_ADDRESS             (CSL_MCSPI1_U_BASE)
#define MCSPI2_BASE_ADDRESS             (CSL_MCSPI2_U_BASE)
#define MCSPI3_BASE_ADDRESS             (CSL_MCSPI3_U_BASE)
#define MCSPI4_BASE_ADDRESS             (CSL_MCSPI4_U_BASE)

#define MCSPI0_INT_NUM                  (CSLR_R5FSS0_CORE0_INTR_MCSPI0_INTR)
#define MCSPI1_INT_NUM                  (CSLR_R5FSS0_CORE0_INTR_MCSPI1_INTR)
#define MCSPI2_INT_NUM                  (CSLR_R5FSS0_CORE0_INTR_MCSPI2_INTR)
#define MCSPI3_INT_NUM                  (CSLR_R5FSS0_CORE0_INTR_MCSPI3_INTR)
#define MCSPI4_INT_NUM                  (CSLR_R5FSS0_CORE0_INTR_MCSPI4_INTR)

#else

#define MCSPI0_BASE_ADDRESS             (CSL_MCSPI0_CFG_BASE)
#define MCSPI1_BASE_ADDRESS             (CSL_MCSPI1_CFG_BASE)
#define MCSPI2_BASE_ADDRESS             (CSL_MCSPI2_CFG_BASE)
#define MCSPI3_BASE_ADDRESS             (CSL_MCSPI3_CFG_BASE)
#define MCSPI4_BASE_ADDRESS             (CSL_MCSPI4_CFG_BASE)

#define MCSPI0_INT_NUM                  (204U)
#define MCSPI1_INT_NUM                  (205U)
#define MCSPI2_INT_NUM                  (206U)
#define MCSPI3_INT_NUM                  (63U)
#define MCSPI4_INT_NUM                  (207U)
#endif

static uint32_t   gClkDividerTestListRampUp[] =
{
    0U,   1U,   2U,   3U,   4U,   5U,   6U,    7U,    8U,    9U,    10U,
    99U,  15U,  31U,  63U,  127U,  199U,  255U,  299U, 399U, 499U, 511U, 599U,
    699U, 799U, 899U, 999U, 1023U, 2047U, 3000U, 4095U
};

static uint32_t   gClkDividerTestListRampDown[] =
{
    4095U, 3000U, 2047U, 1023U, 999U, 899U, 799U,
    699U, 599U, 511U, 499U, 399U, 299U, 255U,
    199U, 127U, 99U, 63U, 31U, 15U, 10U, 9U,
    8U, 7U, 6U, 5U, 4U, 3U, 2U, 1U, 0U
};

#define SPI_TEST_NUM_CLK_LIST            (sizeof (gClkDividerTestListRampUp) / \
                                          sizeof (gClkDividerTestListRampUp[0U]))

typedef struct MCSPI_TestParams_s {
    MCSPI_ChConfig      mcspiChConfigParams;
    MCSPI_OpenParams    mcspiOpenParams;
    uint32_t            transferLength;
    uint32_t            dataSize;
} MCSPI_TestParams;


/* Semaphore to track end of rx_task and tx_task */
SemaphoreP_Object gMcspiTransferTaskDoneSemaphoreObj;
SemaphoreP_Object gMcspiTransferCancelTaskDoneSemaphoreObj;

uint8_t gMcspiTransferTaskStack[MCSPI_TASK_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gMcspiTransferTaskObject;

uint8_t gMcspiTransferCancelTaskStack[MCSPI_TASK_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gMcspiTransferCancelTaskObject;

uint32_t     gMcspiTxBuffer[APP_MCSPI_MSGSIZE];
uint32_t     gMcspiRxBuffer[APP_MCSPI_MSGSIZE];
uint32_t     gMcspiTxBuffer1[APP_MCSPI_MSGSIZE];
uint32_t     gMcspiRxBuffer1[APP_MCSPI_MSGSIZE];
uint16_t     gMcspiPerfTxBuffer[APP_MCSPI_TXONLYMSGSIZE];
uint32_t     gChEnableRegVal, gChDisableRegVal;
uint32_t     gCsAssertRegVal, gCsDeAssertRegVal;

#if (CONFIG_MCSPI_NUM_INSTANCES > 2)
uint32_t     gMcspiTxBufferDma[APP_MCSPI_MSGSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint32_t     gMcspiRxBufferDma[APP_MCSPI_MSGSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
#endif

typedef struct mcspiUtPref_ {
uint64_t polled;
uint64_t interrupt;
uint64_t dma;
} mcspiUtPref;

mcspiUtPref gUtPerf[3];

/* Semaphore to indicate Tx/Rx completion used in callback api's */
static SemaphoreP_Object gMcspiTransferDoneSem;

static void test_mcspi_set_params(MCSPI_TestParams *testParams, uint32_t testCaseId);
void test_mcspi_loopback(void *args);
void test_mcspi_loopback_simultaneous(void *args);
void test_mcspi_loopback_back2back(void *args);
void test_mcspi_loopback_multimaster(void *args);
void test_mcspi_performance_16bit(void *args);
void test_mcspi_loopback_timeout(void *args);
void test_mcspi_transfer_cancel(void *args);
void test_mcspi_transfer_cancel_transfer(void *args);
void test_mcspi_transfer_cancel_cancel(void *args);
void test_mcspi_loopback_performance(void *args);
void test_mcspi_callback(MCSPI_Handle handle, MCSPI_Transaction *trans);
static void mcspi_low_latency_transfer_16bit(uint32_t baseAddr,
                                            uint32_t chNum,
                                            uint16_t *txBuff,
                                            uint32_t length,
                                            uint32_t bufWidthShift);

#if (CONFIG_MCSPI_NUM_INSTANCES > 2)
void test_mcspi_loopback_dma(void *args);
void test_mcspi_loopback_multimaster_dma(void *args);
#endif

#define TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || \
                ((MCSPI_TRANSFER_COMPLETED != transaction.status) && \
                (MCSPI_TRANSFER_STARTED != transaction.status))) \
        { \
            DebugP_assert(FALSE); /* MCSPI TX/RX failed!! */ \
        } \
    } while(0) \

void test_main(void *args)
{
    MCSPI_TestParams  testParams;
    uint32_t          clkList;
    MCSPI_ChConfig   *chConfigParams;
    MCSPI_Config     *config;
    MCSPI_Attrs      *attrParams;

    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    test_mcspi_set_params(&testParams, 335);
    RUN_TEST(test_mcspi_loopback,  335, (void*)&testParams);
    test_mcspi_set_params(&testParams, 336);
    RUN_TEST(test_mcspi_loopback, 336, (void*)&testParams);
/* AM263X does not support MCU_SPI instance */
#if !defined(SOC_AM263X)
/* AM243 LP we, have only 2 instances available */
#if (CONFIG_MCSPI_NUM_INSTANCES > 2)
    test_mcspi_set_params(&testParams, 970);
    RUN_TEST(test_mcspi_loopback,  970, (void*)&testParams);
    test_mcspi_set_params(&testParams, 971);
    RUN_TEST(test_mcspi_loopback,  971, (void*)&testParams);
#endif
#endif
#if (CONFIG_MCSPI_NUM_INSTANCES > 2)
    test_mcspi_set_params(&testParams, 972);
    RUN_TEST(test_mcspi_loopback,  972, (void*)&testParams);
#endif
#if !defined(SOC_AM64X) && !defined(SOC_AM243X) && !defined(SOC_AM263X)
    test_mcspi_set_params(&testParams, 973);
    RUN_TEST(test_mcspi_loopback,  973, (void*)&testParams);
#endif
    test_mcspi_set_params(&testParams, 974);
    RUN_TEST(test_mcspi_loopback,  974, (void*)&testParams);
#if (CONFIG_MCSPI_NUM_INSTANCES > 2)
    test_mcspi_set_params(&testParams, 975);
    RUN_TEST(test_mcspi_loopback,  975, (void*)&testParams);
#endif
    test_mcspi_set_params(&testParams, 980);
    RUN_TEST(test_mcspi_loopback_performance,  980, (void*)&testParams);
    test_mcspi_set_params(&testParams, 7351);
    RUN_TEST(test_mcspi_loopback_performance,  7351, (void*)&testParams);
    test_mcspi_set_params(&testParams, 7352);
    RUN_TEST(test_mcspi_loopback_performance,  7352, (void*)&testParams);
    test_mcspi_set_params(&testParams, 7353);
    RUN_TEST(test_mcspi_loopback_performance,  7353, (void*)&testParams);
    test_mcspi_set_params(&testParams, 7354);
    RUN_TEST(test_mcspi_loopback_performance,  7354, (void*)&testParams);
    test_mcspi_set_params(&testParams, 7355);
    RUN_TEST(test_mcspi_loopback_performance,  7355, (void*)&testParams);

    test_mcspi_set_params(&testParams, 985);
    RUN_TEST(test_mcspi_loopback_back2back,  985, (void*)&testParams);
    test_mcspi_set_params(&testParams, 991);
    RUN_TEST(test_mcspi_loopback,  991, (void*)&testParams);
    test_mcspi_set_params(&testParams, 992);
    /* Change clock divider as per test list */
    chConfigParams = &(testParams.mcspiChConfigParams);
    config = &gMcspiConfig[CONFIG_MCSPI0];
    attrParams = (MCSPI_Attrs *)config->attrs;
    for (clkList = 0U; clkList < SPI_TEST_NUM_CLK_LIST; clkList++)
    {
        chConfigParams->bitRate = (attrParams->inputClkFreq / (gClkDividerTestListRampUp[clkList] + 1));
        RUN_TEST(test_mcspi_loopback,  992, (void*)&testParams);
    }
    test_mcspi_set_params(&testParams, 993);
    chConfigParams = &(testParams.mcspiChConfigParams);
    config = &gMcspiConfig[CONFIG_MCSPI0];
    attrParams = (MCSPI_Attrs *)config->attrs;
    for (clkList = 0U; clkList < SPI_TEST_NUM_CLK_LIST; clkList++)
    {
        chConfigParams->bitRate = (attrParams->inputClkFreq / (gClkDividerTestListRampDown[clkList] + 1));
        RUN_TEST(test_mcspi_loopback,  993, (void*)&testParams);
    }
    test_mcspi_set_params(&testParams, 994);
    RUN_TEST(test_mcspi_loopback,  994, (void*)&testParams);
    test_mcspi_set_params(&testParams, 995);
    RUN_TEST(test_mcspi_loopback,  995, (void*)&testParams);
    test_mcspi_set_params(&testParams, 996);
    RUN_TEST(test_mcspi_loopback,  996, (void*)&testParams);
    test_mcspi_set_params(&testParams, 997);
    RUN_TEST(test_mcspi_loopback,  997, (void*)&testParams);
    test_mcspi_set_params(&testParams, 998);
    RUN_TEST(test_mcspi_loopback,  998, (void*)&testParams);
    test_mcspi_set_params(&testParams, 999);
    RUN_TEST(test_mcspi_loopback,  999, (void*)&testParams);
    test_mcspi_set_params(&testParams, 1000);
    RUN_TEST(test_mcspi_loopback,  1000, (void*)&testParams);
    test_mcspi_set_params(&testParams, 1001);
    RUN_TEST(test_mcspi_loopback,  1001, (void*)&testParams);
    test_mcspi_set_params(&testParams, 1002);
    RUN_TEST(test_mcspi_loopback,  1002, (void*)&testParams);
    test_mcspi_set_params(&testParams, 1003);
    RUN_TEST(test_mcspi_loopback,  1003, (void*)&testParams);
    test_mcspi_set_params(&testParams, 1004);
    RUN_TEST(test_mcspi_loopback,  1004, (void*)&testParams);
    test_mcspi_set_params(&testParams, 1005);
    RUN_TEST(test_mcspi_loopback,  1005, (void*)&testParams);
    test_mcspi_set_params(&testParams, 1006);
    RUN_TEST(test_mcspi_loopback_multimaster,  1006, (void*)&testParams);
#if (CONFIG_MCSPI_NUM_INSTANCES > 2)
    test_mcspi_set_params(&testParams, 1007);
    /* Change clock divider as per test list */
    chConfigParams = &(testParams.mcspiChConfigParams);
    config = &gMcspiConfig[CONFIG_MCSPI0];
    attrParams = (MCSPI_Attrs *)config->attrs;
    for (clkList = 0U; clkList < SPI_TEST_NUM_CLK_LIST; clkList++)
    {
        chConfigParams->bitRate = (attrParams->inputClkFreq / (gClkDividerTestListRampUp[clkList] + 1));
        RUN_TEST(test_mcspi_loopback,  1007, (void*)&testParams);
    }
#endif
    test_mcspi_set_params(&testParams, 1008);
    chConfigParams = &(testParams.mcspiChConfigParams);
    config = &gMcspiConfig[CONFIG_MCSPI0];
    attrParams = (MCSPI_Attrs *)config->attrs;
    for (clkList = 0U; clkList < SPI_TEST_NUM_CLK_LIST; clkList++)
    {
        chConfigParams->bitRate = (attrParams->inputClkFreq / (gClkDividerTestListRampDown[clkList] + 1));
        RUN_TEST(test_mcspi_loopback,  1008, (void*)&testParams);
    }
    test_mcspi_set_params(&testParams, 1009);
    RUN_TEST(test_mcspi_loopback_simultaneous, 1009, (void*)&testParams);
    test_mcspi_set_params(&testParams, 1011);
    RUN_TEST(test_mcspi_transfer_cancel, 1011, (void*)&testParams);
    test_mcspi_set_params(&testParams, 1012);
    RUN_TEST(test_mcspi_transfer_cancel, 1012, (void*)&testParams);
    test_mcspi_set_params(&testParams, 1014);
    RUN_TEST(test_mcspi_loopback_timeout, 1014, (void*)&testParams);
    test_mcspi_set_params(&testParams, 1565);
    RUN_TEST(test_mcspi_performance_16bit, 1565, (void*)&testParams);
#if (CONFIG_MCSPI_NUM_INSTANCES > 2)
    test_mcspi_set_params(&testParams, 2397);
    RUN_TEST(test_mcspi_loopback_multimaster_dma,  2397, (void*)&testParams);
    test_mcspi_set_params(&testParams, 2394);
    RUN_TEST(test_mcspi_loopback_dma,  2394, (void*)&testParams);
    test_mcspi_set_params(&testParams, 7356);
    RUN_TEST(test_mcspi_loopback_dma,  7356, (void*)&testParams);
    test_mcspi_set_params(&testParams, 7357);
    RUN_TEST(test_mcspi_loopback_dma,  7357, (void*)&testParams);
#endif

    /* Print Performance Numbers. */
    DebugP_log("\nMCSPI Performance Numbers Print Start\r\n\n");
    DebugP_log("Number of Words | Word Width (Bits)     | Polled mode Throughput / Transfer time  | Interrupt mode (Mbps) Throughput / Transfer time | Dma mode (Mbps) Throughput / Transfer time\r\n");
    DebugP_log("----------------|-----------------------|-------------------------------|-------------------------------|-------------------------------\r\n");
    for (uint32_t i =0; i<3; i++)
    {
        uint32_t dataWidth = 8*(1<<i);
        uint32_t dataLength = 400/(1<<i);
        DebugP_log(" %u\t\t| %02u\t\t\t| %5.2f Mbps / %5.2f us \t| %5.2f Mbps / %5.2f us \t| %5.2f Mbps / %5.2f us\r\n", dataLength, dataWidth,
                            (float)(dataLength * dataWidth)/((float)gUtPerf[i].polled / APP_MCSPI_PERF_LOOP_ITER_CNT),
                            ((float)gUtPerf[i].polled / APP_MCSPI_PERF_LOOP_ITER_CNT),
                            (float)(dataLength * dataWidth)/((float)gUtPerf[i].interrupt / APP_MCSPI_PERF_LOOP_ITER_CNT),
                            ((float)gUtPerf[i].interrupt / APP_MCSPI_PERF_LOOP_ITER_CNT),
                            (float)(dataLength * dataWidth)/((float)gUtPerf[i].dma / APP_MCSPI_PERF_LOOP_ITER_CNT),
                            ((float)gUtPerf[i].dma / APP_MCSPI_PERF_LOOP_ITER_CNT));
    }
    DebugP_log("\nMCSPI Performance Numbers Print End\r\n");

    UNITY_END();

    Board_driversClose();
    Drivers_close();

    MCSPI_deinit();

    return;
}

void test_mcspi_loopback(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    MCSPI_Handle        mcspiHandle;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    uint8_t            *tempRxPtr8 = NULL, *tempTxPtr8 = NULL;
    uint16_t           *tempRxPtr16 = NULL, *tempTxPtr16 = NULL;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);
    MCSPI_ChConfig     *mcspiChConfigParams = &(testParams->mcspiChConfigParams);

    /* Memset Buffers */
    memset(&gMcspiTxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiTxBuffer[0U]));
    memset(&gMcspiRxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiRxBuffer[0U]));

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);

    status = MCSPI_chConfig(
                 gMcspiHandle[CONFIG_MCSPI0],
                 mcspiChConfigParams);
    DebugP_assert(status == SystemP_SUCCESS);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    dataWidth = testParams->dataSize;
    if (dataWidth < 9U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr8 = (uint8_t *) &gMcspiTxBuffer[0U];
        tempRxPtr8 = (uint8_t *) &gMcspiRxBuffer[0U];
    }
    else if (dataWidth < 17U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr16 = (uint16_t *) &gMcspiTxBuffer[0U];
        tempRxPtr16 = (uint16_t *) &gMcspiRxBuffer[0U];
    }
    else
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr32 = (uint32_t *) &gMcspiTxBuffer[0U];
        tempRxPtr32 = (uint32_t *) &gMcspiRxBuffer[0U];
    }
    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers */
    for (i = 0U; i < testParams->transferLength; i++)
    {
        tempTxData = 0xDEADBABE;
        tempTxData &= (fifoBitMask);
        if (dataWidth < 9U)
        {
            *tempTxPtr8++ = (uint8_t) (tempTxData);
            *tempRxPtr8++ = 0U;
        }
        else if (dataWidth < 17U)
        {
            *tempTxPtr16++ = (uint16_t) (tempTxData);
            *tempRxPtr16++ = 0U;
        }
        else
        {
            *tempTxPtr32++ = (uint32_t) (tempTxData);
            *tempRxPtr32++ = 0U;
        }
    }

    /* Initiate transfer */
    spiTransaction.channel  = testParams->mcspiChConfigParams.chNum;
    spiTransaction.count    = testParams->transferLength;
    spiTransaction.dataSize  = dataWidth;
    spiTransaction.csDisable = TRUE;
    spiTransaction.txBuf    = (void *)gMcspiTxBuffer;
    spiTransaction.rxBuf    = (void *)gMcspiRxBuffer;
    spiTransaction.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for transfer completion */
        SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Compare data */
    uint8_t *tempTxPtr, *tempRxPtr;
    tempTxPtr = (uint8_t *) &gMcspiTxBuffer[0U];
    tempRxPtr = (uint8_t *) &gMcspiRxBuffer[0U];
    for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
    {
        if(*tempTxPtr++ != *tempRxPtr++)
        {
            status = SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gMcspiTransferDoneSem);
    }

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

void test_mcspi_loopback_performance(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i,j, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    MCSPI_Handle        mcspiHandle;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    uint8_t            *tempRxPtr8 = NULL, *tempTxPtr8 = NULL;
    uint16_t           *tempRxPtr16 = NULL, *tempTxPtr16 = NULL;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);
    MCSPI_ChConfig     *mcspiChConfigParams = &(testParams->mcspiChConfigParams);
    uint64_t            startTimeInUSec, elapsedTimeInUsecs, totalTimeInUsecs = 0U;
    uint32_t            perf_offset;
    MCSPI_Config       *config = &gMcspiConfig[CONFIG_MCSPI0];
    MCSPI_Attrs        *attrParams = (MCSPI_Attrs *)config->attrs;

    /* Memset Buffers */
    memset(&gMcspiTxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiTxBuffer[0U]));
    memset(&gMcspiRxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiRxBuffer[0U]));

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);

    status = MCSPI_chConfig(
                 gMcspiHandle[CONFIG_MCSPI0],
                 mcspiChConfigParams);
    DebugP_assert(status == SystemP_SUCCESS);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    dataWidth = testParams->dataSize;
    if (dataWidth < 9U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr8 = (uint8_t *) &gMcspiTxBuffer[0U];
        tempRxPtr8 = (uint8_t *) &gMcspiRxBuffer[0U];
    }
    else if (dataWidth < 17U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr16 = (uint16_t *) &gMcspiTxBuffer[0U];
        tempRxPtr16 = (uint16_t *) &gMcspiRxBuffer[0U];
    }
    else
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr32 = (uint32_t *) &gMcspiTxBuffer[0U];
        tempRxPtr32 = (uint32_t *) &gMcspiRxBuffer[0U];
    }
    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers */
    for (i = 0U; i < testParams->transferLength; i++)
    {
        tempTxData = 0xDEADBABE;
        tempTxData &= (fifoBitMask);
        if (dataWidth < 9U)
        {
            *tempTxPtr8++ = (uint8_t) (tempTxData);
            *tempRxPtr8++ = 0U;
        }
        else if (dataWidth < 17U)
        {
            *tempTxPtr16++ = (uint16_t) (tempTxData);
            *tempRxPtr16++ = 0U;
        }
        else
        {
            *tempTxPtr32++ = (uint32_t) (tempTxData);
            *tempRxPtr32++ = 0U;
        }
    }

    for(j = 0U; j < APP_MCSPI_PERF_LOOP_ITER_CNT; j++)
    {
        /* Initiate transfer */
        spiTransaction.channel  = testParams->mcspiChConfigParams.chNum;
        spiTransaction.count    = testParams->transferLength;
        spiTransaction.dataSize  = dataWidth;
        spiTransaction.csDisable = TRUE;
        spiTransaction.txBuf    = (void *)gMcspiTxBuffer;
        spiTransaction.rxBuf    = (void *)gMcspiRxBuffer;
        spiTransaction.args     = NULL;
        startTimeInUSec = ClockP_getTimeUsec();
        transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
        TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction);
        if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
        {
            /* Wait for transfer completion */
            SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
        }
        elapsedTimeInUsecs = ClockP_getTimeUsec() - startTimeInUSec;
        totalTimeInUsecs += elapsedTimeInUsecs;

        /* Compare data */
        uint8_t *tempTxPtr, *tempRxPtr;
        tempTxPtr = (uint8_t *) &gMcspiTxBuffer[0U];
        tempRxPtr = (uint8_t *) &gMcspiRxBuffer[0U];
        for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
        {
            if(*tempTxPtr++ != *tempRxPtr++)
            {
                status = SystemP_FAILURE;   /* Data mismatch */
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
        }
    }

    /* Store Performance value in global var. Performance numbers are printed at the end of UT. */
    if (dataWidth == 8) {
        perf_offset = 0;
    } else if (dataWidth == 16) {
        perf_offset = 1;
    } else if (dataWidth == 32) {
        perf_offset = 2;
    }
    if (attrParams->operMode == MCSPI_OPER_MODE_POLLED)
    {
        gUtPerf[perf_offset].polled = totalTimeInUsecs;
    }
    if (attrParams->operMode == MCSPI_OPER_MODE_INTERRUPT)
    {
        gUtPerf[perf_offset].interrupt = totalTimeInUsecs;
    }

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gMcspiTransferDoneSem);
    }

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

void test_mcspi_loopback_back2back(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    MCSPI_Handle        mcspiHandle;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    uint8_t            *tempRxPtr8 = NULL, *tempTxPtr8 = NULL;
    uint16_t           *tempRxPtr16 = NULL, *tempTxPtr16 = NULL;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);
    MCSPI_ChConfig     *mcspiChConfigParams = &(testParams->mcspiChConfigParams);

    /* Memset Buffers */
    memset(&gMcspiTxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiTxBuffer[0U]));
    memset(&gMcspiRxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiRxBuffer[0U]));

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);

    status = MCSPI_chConfig(
                 gMcspiHandle[CONFIG_MCSPI0],
                 mcspiChConfigParams);
    DebugP_assert(status == SystemP_SUCCESS);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    dataWidth = testParams->dataSize;
    if (dataWidth < 9U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr8 = (uint8_t *) &gMcspiTxBuffer[0U];
        tempRxPtr8 = (uint8_t *) &gMcspiRxBuffer[0U];
    }
    else if (dataWidth < 17U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr16 = (uint16_t *) &gMcspiTxBuffer[0U];
        tempRxPtr16 = (uint16_t *) &gMcspiRxBuffer[0U];
    }
    else
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr32 = (uint32_t *) &gMcspiTxBuffer[0U];
        tempRxPtr32 = (uint32_t *) &gMcspiRxBuffer[0U];
    }
    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers */
    for (i = 0U; i < testParams->transferLength; i++)
    {
        tempTxData = 0xDEADBABE;
        tempTxData &= (fifoBitMask);
        if (dataWidth < 9U)
        {
            *tempTxPtr8++ = (uint8_t) (tempTxData);
            *tempRxPtr8++ = 0U;
        }
        else if (dataWidth < 17U)
        {
            *tempTxPtr16++ = (uint16_t) (tempTxData);
            *tempRxPtr16++ = 0U;
        }
        else
        {
            *tempTxPtr32++ = (uint32_t) (tempTxData);
            *tempRxPtr32++ = 0U;
        }
    }

    /* Initiate transfer */
    spiTransaction.channel  = testParams->mcspiChConfigParams.chNum;
    spiTransaction.count    = testParams->transferLength;
    spiTransaction.dataSize  = dataWidth;
    spiTransaction.csDisable = TRUE;
    spiTransaction.txBuf    = (void *)gMcspiTxBuffer;
    spiTransaction.rxBuf    = (void *)gMcspiRxBuffer;
    spiTransaction.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    DebugP_assert(transferOK != SystemP_SUCCESS);
    DebugP_assert(spiTransaction.status == MCSPI_TRANSFER_CANCELLED);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for transfer completion */
        SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Compare data */
    uint8_t *tempTxPtr, *tempRxPtr;
    tempTxPtr = (uint8_t *) &gMcspiTxBuffer[0U];
    tempRxPtr = (uint8_t *) &gMcspiRxBuffer[0U];
    for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
    {
        if(*tempTxPtr++ != *tempRxPtr++)
        {
            status = SystemP_SUCCESS;   /* Data Match */
            DebugP_log("Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gMcspiTransferDoneSem);
    }

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

void test_mcspi_loopback_multimaster(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    int32_t             transferOK, chCnt;
    MCSPI_Transaction   spiTransaction;
    MCSPI_Handle        mcspiHandle;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    uint8_t            *tempRxPtr8 = NULL, *tempTxPtr8 = NULL;
    uint16_t           *tempRxPtr16 = NULL, *tempTxPtr16 = NULL;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);
    uint32_t            mcspiChDataSize[4] = { 4, 8, 8, 8};
    MCSPI_Config       *config;
    MCSPI_Attrs        *attrParams;

    /* Memset Buffers */
    memset(&gMcspiTxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiTxBuffer[0U]));
    memset(&gMcspiRxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiRxBuffer[0U]));

    config = &gMcspiConfig[CONFIG_MCSPI1];
    attrParams = (MCSPI_Attrs *)config->attrs;

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI1]);

    attrParams->operMode = MCSPI_OPER_MODE_INTERRUPT;
    mcspiHandle = MCSPI_open(CONFIG_MCSPI1, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);

    for(chCnt = 0U; chCnt < CONFIG_MCSPI1_NUM_CH; chCnt++)
    {
        if ((chCnt % 2) == 0U)
        {
            gConfigMcspi1ChCfg[chCnt].inputSelect = MCSPI_IS_D1;
            gConfigMcspi1ChCfg[chCnt].dpe0        = MCSPI_DPE_DISABLE;
            gConfigMcspi1ChCfg[chCnt].dpe1        = MCSPI_DPE_ENABLE;
        }
        else
        {
            gConfigMcspi1ChCfg[chCnt].inputSelect = MCSPI_IS_D0;
            gConfigMcspi1ChCfg[chCnt].dpe0        = MCSPI_DPE_ENABLE;
            gConfigMcspi1ChCfg[chCnt].dpe1        = MCSPI_DPE_DISABLE;
        }

        status = MCSPI_chConfig(
                     gMcspiHandle[CONFIG_MCSPI1],
                     &gConfigMcspi1ChCfg[chCnt]);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CONFIG_MCSPI1 channel %d config failed !!!\r\n", chCnt);
            break;
        }

        if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
        {
            status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        dataWidth = mcspiChDataSize[chCnt];
        if (dataWidth < 9U)
        {
            /* Init TX buffer with known data and memset RX buffer */
            tempTxPtr8 = (uint8_t *) &gMcspiTxBuffer[0U];
            tempRxPtr8 = (uint8_t *) &gMcspiRxBuffer[0U];
        }
        else if (dataWidth < 17U)
        {
            /* Init TX buffer with known data and memset RX buffer */
            tempTxPtr16 = (uint16_t *) &gMcspiTxBuffer[0U];
            tempRxPtr16 = (uint16_t *) &gMcspiRxBuffer[0U];
        }
        else
        {
            /* Init TX buffer with known data and memset RX buffer */
            tempTxPtr32 = (uint32_t *) &gMcspiTxBuffer[0U];
            tempRxPtr32 = (uint32_t *) &gMcspiRxBuffer[0U];
        }
        fifoBitMask = 0x0U;
        for (dataWidthIdx = 0U;
             dataWidthIdx < dataWidth; dataWidthIdx++)
        {
            fifoBitMask |= (1U << dataWidthIdx);
        }

        /* Memfill buffers */
        for (i = 0U; i < testParams->transferLength; i++)
        {
            tempTxData = 0xDEADBABE;
            tempTxData &= (fifoBitMask);
            if (dataWidth < 9U)
            {
                *tempTxPtr8++ = (uint8_t) (tempTxData);
                *tempRxPtr8++ = 0U;
            }
            else if (dataWidth < 17U)
            {
                *tempTxPtr16++ = (uint16_t) (tempTxData);
                *tempRxPtr16++ = 0U;
            }
            else
            {
                *tempTxPtr32++ = (uint32_t) (tempTxData);
                *tempRxPtr32++ = 0U;
            }
        }

        /* Initiate transfer */
        spiTransaction.channel  = gConfigMcspi1ChCfg[chCnt].chNum;
        spiTransaction.dataSize  = dataWidth;
        spiTransaction.csDisable = TRUE;
        spiTransaction.count    = testParams->transferLength;
        spiTransaction.txBuf    = (void *)gMcspiTxBuffer;
        spiTransaction.rxBuf    = (void *)gMcspiRxBuffer;
        spiTransaction.args     = NULL;
        transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI1], &spiTransaction);
        TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction);

        if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
        {
            /* Wait for transfer completion */
            SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
        }

        /* Compare data */
        uint8_t *tempTxPtr, *tempRxPtr;
        tempTxPtr = (uint8_t *) &gMcspiTxBuffer[0U];
        tempRxPtr = (uint8_t *) &gMcspiRxBuffer[0U];
        for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
        {
            if(*tempTxPtr++ != *tempRxPtr++)
            {
                status = SystemP_FAILURE;   /* Data mismatch */
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
        }

        if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
        {
            SemaphoreP_destruct(&gMcspiTransferDoneSem);
        }

    }

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI1]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

#if (CONFIG_MCSPI_NUM_INSTANCES > 2)
void test_mcspi_loopback_multimaster_dma(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    int32_t             transferOK, chCnt;
    MCSPI_Transaction   spiTransaction;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    uint8_t            *tempRxPtr8 = NULL, *tempTxPtr8 = NULL;
    uint16_t           *tempRxPtr16 = NULL, *tempTxPtr16 = NULL;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);

    /* Memset Buffers */
    memset(&gMcspiTxBufferDma[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiTxBufferDma[0U]));
    memset(&gMcspiRxBufferDma[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiRxBufferDma[0U]));

    for(chCnt = 0U; chCnt < CONFIG_MCSPI3_NUM_CH; chCnt++)
    {
        if ((chCnt % 2) == 0U)
        {
            gConfigMcspi3ChCfg[chCnt].inputSelect = MCSPI_IS_D1;
            gConfigMcspi3ChCfg[chCnt].dpe0        = MCSPI_DPE_DISABLE;
            gConfigMcspi3ChCfg[chCnt].dpe1        = MCSPI_DPE_ENABLE;
        }
        else
        {
            gConfigMcspi3ChCfg[chCnt].inputSelect = MCSPI_IS_D0;
            gConfigMcspi3ChCfg[chCnt].dpe0        = MCSPI_DPE_ENABLE;
            gConfigMcspi3ChCfg[chCnt].dpe1        = MCSPI_DPE_DISABLE;
        }

        status = MCSPI_chConfig(
                     gMcspiHandle[CONFIG_MCSPI3],
                     &gConfigMcspi3ChCfg[chCnt]);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CONFIG_MCSPI3 channel %d config failed !!!\r\n", chCnt);
            break;
        }

        if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
        {
            status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        dataWidth = 8U;
        if (dataWidth < 9U)
        {
            /* Init TX buffer with known data and memset RX buffer */
            tempTxPtr8 = (uint8_t *) &gMcspiTxBufferDma[0U];
            tempRxPtr8 = (uint8_t *) &gMcspiRxBufferDma[0U];
        }
        else if (dataWidth < 17U)
        {
            /* Init TX buffer with known data and memset RX buffer */
            tempTxPtr16 = (uint16_t *) &gMcspiTxBufferDma[0U];
            tempRxPtr16 = (uint16_t *) &gMcspiRxBufferDma[0U];
        }
        else
        {
            /* Init TX buffer with known data and memset RX buffer */
            tempTxPtr32 = (uint32_t *) &gMcspiTxBufferDma[0U];
            tempRxPtr32 = (uint32_t *) &gMcspiRxBufferDma[0U];
        }
        fifoBitMask = 0x0U;
        for (dataWidthIdx = 0U;
             dataWidthIdx < dataWidth; dataWidthIdx++)
        {
            fifoBitMask |= (1U << dataWidthIdx);
        }

        /* Memfill buffers */
        for (i = 0U; i < testParams->transferLength; i++)
        {
            tempTxData = 0xDEADBABE;
            tempTxData &= (fifoBitMask);
            if (dataWidth < 9U)
            {
                *tempTxPtr8++ = (uint8_t) (tempTxData);
                *tempRxPtr8++ = 0U;
            }
            else if (dataWidth < 17U)
            {
                *tempTxPtr16++ = (uint16_t) (tempTxData);
                *tempRxPtr16++ = 0U;
            }
            else
            {
                *tempTxPtr32++ = (uint32_t) (tempTxData);
                *tempRxPtr32++ = 0U;
            }
        }

        /* Writeback buffer */
        CacheP_wb(&gMcspiTxBufferDma[0U], sizeof(gMcspiTxBufferDma), CacheP_TYPE_ALLD);
        CacheP_wb(&gMcspiRxBufferDma[0U], sizeof(gMcspiRxBufferDma), CacheP_TYPE_ALLD);

        /* Initiate transfer */
        spiTransaction.channel  = gConfigMcspi3ChCfg[chCnt].chNum;
        spiTransaction.dataSize  = dataWidth;
        spiTransaction.csDisable = TRUE;
        spiTransaction.count    = APP_MCSPI_MSGSIZE / (dataWidth / 8);
        spiTransaction.txBuf    = (void *)gMcspiTxBufferDma;
        spiTransaction.rxBuf    = (void *)gMcspiRxBufferDma;
        spiTransaction.args     = NULL;
        transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI3], &spiTransaction);
        TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction);

        if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
        {
            /* Wait for transfer completion */
            SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
        }

        /* Invalidate cache */
        CacheP_inv(&gMcspiRxBufferDma[0U], sizeof(gMcspiRxBufferDma), CacheP_TYPE_ALLD);
        /* Compare data */
        uint8_t *tempTxPtr, *tempRxPtr;
        tempTxPtr = (uint8_t *) &gMcspiTxBufferDma[0U];
        tempRxPtr = (uint8_t *) &gMcspiRxBufferDma[0U];
        for(i = 0U; i < (APP_MCSPI_MSGSIZE); i++)
        {
            if(*tempTxPtr++ != *tempRxPtr++)
            {
                status = SystemP_FAILURE;   /* Data mismatch */
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
        }

        if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
        {
            SemaphoreP_destruct(&gMcspiTransferDoneSem);
        }

    }

    for(chCnt = 0U; chCnt < CONFIG_MCSPI3_NUM_CH; chCnt++)
    {
        status = MCSPI_dmaClose(gMcspiHandle[CONFIG_MCSPI3],
                                &gConfigMcspi3ChCfg[chCnt]);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CONFIG_MCSPI3 DMA close %d failed !!!\r\n", chCnt);
            break;
        }
    }

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI3]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

void test_mcspi_loopback_dma(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, j, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    uint8_t            *tempRxPtr8 = NULL, *tempTxPtr8 = NULL;
    uint16_t           *tempRxPtr16 = NULL, *tempTxPtr16 = NULL;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);
    MCSPI_Config       *config;
    MCSPI_Attrs        *attrParams;
    MCSPI_Handle        mcspiHandle;
    uint64_t            startTimeInUSec, elapsedTimeInUsecs, totalTimeInUsecs = 0U;
    uint32_t            perf_offset;

    /* Memset Buffers */
    memset(&gMcspiTxBufferDma[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiTxBufferDma[0U]));
    memset(&gMcspiRxBufferDma[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiRxBufferDma[0U]));

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI3]);

    config = &gMcspiConfig[CONFIG_MCSPI3];
    attrParams = (MCSPI_Attrs *)config->attrs;
    attrParams->operMode               = MCSPI_OPER_MODE_DMA;
    mcspiOpenParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
    mcspiOpenParams->transferCallbackFxn    = test_mcspi_callback;
    mcspiOpenParams->mcspiDmaIndex          = 0;
    mcspiHandle = MCSPI_open(CONFIG_MCSPI3, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    status = MCSPI_chConfig(
                 gMcspiHandle[CONFIG_MCSPI3],
                 &gConfigMcspi3ChCfg[0U]);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("CONFIG_MCSPI3 channel %d config failed !!!\r\n", 0);
    }

    status = MCSPI_dmaChConfig(
                 gMcspiHandle[CONFIG_MCSPI3],
                 &gConfigMcspi3ChCfg[0U],
                 &gConfigMcspi3DmaChCfg[0U]);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("CONFIG_MCSPI3 channel %d config failed !!!\r\n", 0);
    }

    dataWidth = testParams->dataSize;
    if (dataWidth < 9U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr8 = (uint8_t *) &gMcspiTxBufferDma[0U];
        tempRxPtr8 = (uint8_t *) &gMcspiRxBufferDma[0U];
    }
    else if (dataWidth < 17U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr16 = (uint16_t *) &gMcspiTxBufferDma[0U];
        tempRxPtr16 = (uint16_t *) &gMcspiRxBufferDma[0U];
    }
    else
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr32 = (uint32_t *) &gMcspiTxBufferDma[0U];
        tempRxPtr32 = (uint32_t *) &gMcspiRxBufferDma[0U];
    }
    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers */
    for (i = 0U; i < testParams->transferLength; i++)
    {
        tempTxData = 0xDEADBABE;
        tempTxData &= (fifoBitMask);
        if (dataWidth < 9U)
        {
            *tempTxPtr8++ = (uint8_t) (tempTxData);
            *tempRxPtr8++ = 0U;
        }
        else if (dataWidth < 17U)
        {
            *tempTxPtr16++ = (uint16_t) (tempTxData);
            *tempRxPtr16++ = 0U;
        }
        else
        {
            *tempTxPtr32++ = (uint32_t) (tempTxData);
            *tempRxPtr32++ = 0U;
        }
    }

    /* Writeback buffer */
    CacheP_wb(&gMcspiTxBufferDma[0U], sizeof(gMcspiTxBufferDma), CacheP_TYPE_ALLD);
    CacheP_wb(&gMcspiRxBufferDma[0U], sizeof(gMcspiRxBufferDma), CacheP_TYPE_ALLD);
    for(j = 0U; j < APP_MCSPI_PERF_LOOP_ITER_CNT; j++)
    {
        /* Initiate transfer */
        spiTransaction.channel  = gConfigMcspi3ChCfg[0U].chNum;
        spiTransaction.count    = testParams->transferLength;
        spiTransaction.dataSize  = dataWidth;
        spiTransaction.csDisable = FALSE;
        spiTransaction.txBuf    = (void *)gMcspiTxBufferDma;
        spiTransaction.rxBuf    = (void *)gMcspiRxBufferDma;
        spiTransaction.args     = NULL;
        startTimeInUSec = ClockP_getTimeUsec();
        transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI3], &spiTransaction);
        TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction);

        if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
        {
            /* Wait for transfer completion */
            SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
        }
        elapsedTimeInUsecs = ClockP_getTimeUsec() - startTimeInUSec;
        totalTimeInUsecs += elapsedTimeInUsecs;

        /* Invalidate cache */
        CacheP_inv(&gMcspiRxBufferDma[0U], sizeof(gMcspiRxBufferDma), CacheP_TYPE_ALLD);

        /* Compare data */
        uint8_t *tempTxPtr, *tempRxPtr;
        tempTxPtr = (uint8_t *) &gMcspiTxBufferDma[0U];
        tempRxPtr = (uint8_t *) &gMcspiRxBufferDma[0U];
        for(i = 0U; i < (APP_MCSPI_MSGSIZE); i++)
        {
            if(*tempTxPtr++ != *tempRxPtr++)
            {
                status = SystemP_FAILURE;   /* Data mismatch */
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
        }
    }

    /* Store Performance value in global var. Performance numbers are printed at the end of UT. */
    if (dataWidth == 8) {
        perf_offset = 0;
    } else if (dataWidth == 16) {
        perf_offset = 1;
    } else{
        perf_offset = 2;
    }
    gUtPerf[perf_offset].dma = totalTimeInUsecs;

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gMcspiTransferDoneSem);
    }

    status = MCSPI_dmaClose(gMcspiHandle[CONFIG_MCSPI3],
                            &gConfigMcspi3ChCfg[0U]);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI3]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

#endif

void test_mcspi_loopback_timeout(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    MCSPI_Handle        mcspiHandle;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    uint8_t            *tempRxPtr8 = NULL, *tempTxPtr8 = NULL;
    uint16_t           *tempRxPtr16 = NULL, *tempTxPtr16 = NULL;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);
    MCSPI_ChConfig     *mcspiChConfigParams = &(testParams->mcspiChConfigParams);

    /* Memset Buffers */
    memset(&gMcspiTxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiTxBuffer[0U]));
    memset(&gMcspiRxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiRxBuffer[0U]));

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);

    status = MCSPI_chConfig(
                 gMcspiHandle[CONFIG_MCSPI0],
                 mcspiChConfigParams);
    DebugP_assert(status == SystemP_SUCCESS);

    dataWidth = testParams->dataSize;
    if (dataWidth < 9U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr8 = (uint8_t *) &gMcspiTxBuffer[0U];
        tempRxPtr8 = (uint8_t *) &gMcspiRxBuffer[0U];
    }
    else if (dataWidth < 17U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr16 = (uint16_t *) &gMcspiTxBuffer[0U];
        tempRxPtr16 = (uint16_t *) &gMcspiRxBuffer[0U];
    }
    else
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr32 = (uint32_t *) &gMcspiTxBuffer[0U];
        tempRxPtr32 = (uint32_t *) &gMcspiRxBuffer[0U];
    }
    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers */
    for (i = 0U; i < testParams->transferLength; i++)
    {
        tempTxData = 0xDEADBABE;
        tempTxData &= (fifoBitMask);
        if (dataWidth < 9U)
        {
            *tempTxPtr8++ = (uint8_t) (tempTxData);
            *tempRxPtr8++ = 0U;
        }
        else if (dataWidth < 17U)
        {
            *tempTxPtr16++ = (uint16_t) (tempTxData);
            *tempRxPtr16++ = 0U;
        }
        else
        {
            *tempTxPtr32++ = (uint32_t) (tempTxData);
            *tempRxPtr32++ = 0U;
        }
    }

    /* Initiate transfer */
    spiTransaction.channel  = testParams->mcspiChConfigParams.chNum;
    spiTransaction.dataSize  = dataWidth;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = testParams->transferLength;
    spiTransaction.txBuf    = (void *)gMcspiTxBuffer;
    spiTransaction.rxBuf    = (void *)gMcspiRxBuffer;
    spiTransaction.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    DebugP_assert(spiTransaction.status == MCSPI_TRANSFER_TIMEOUT);
    DebugP_assert(transferOK == SystemP_FAILURE);

    /* Compare data and should mismatch as timeout occurred */
    uint8_t *tempTxPtr, *tempRxPtr;
    tempTxPtr = (uint8_t *) &gMcspiTxBuffer[0U];
    tempRxPtr = (uint8_t *) &gMcspiRxBuffer[0U];
    for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
    {
        if(*tempTxPtr++ != *tempRxPtr++)
        {
            status = SystemP_SUCCESS;
            break;
        }
    }

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

void test_mcspi_performance_16bit(void *args)
{
    uint32_t            i, j;
    uint32_t            dataLength, dataWidth, bitRate, bufWidthShift;
    uint32_t            baseAddr, chNum, dataSize;
    uint32_t            chCtrlRegVal, chConfRegVal;
    uint64_t            startTimeInUSec, elapsedTimeInUsecs;
    int32_t             status = SystemP_SUCCESS;
    MCSPI_Handle        mcspiHandle;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);

    /* update data size to 16 and bitrate to 12MHZ */
    dataSize = 16U;
    gConfigMcspi0ChCfg[0U].bitRate  = 12500000U;
    gConfigMcspi0ChCfg[0U].trMode   = MCSPI_TR_MODE_TX_ONLY;

    status = MCSPI_chConfig(gMcspiHandle[CONFIG_MCSPI0], &gConfigMcspi0ChCfg[0U]);
    DebugP_assert(SystemP_SUCCESS == status);

    DebugP_log("[MCSPI] Performance 16bit Example Started...\r\n\n");

    /* Memfill buffers */
    for(i = 0U; i < APP_MCSPI_TXONLYMSGSIZE; i++)
    {
        gMcspiPerfTxBuffer[i] = i;
    }

    /* Initialize SPI Channel Number */
    chNum  = gConfigMcspi0ChCfg[0U].chNum;

    /* Get SPI Channel Info */
    baseAddr = MCSPI_getBaseAddr(gMcspiHandle[CONFIG_MCSPI0]);
    DebugP_assert(baseAddr != 0U); /* MCSPI baseAddr Invalid!! */

    /* Set dataWidth */
    MCSPI_setDataWidth(baseAddr, chNum, dataSize);
    /* Enable the transmitter FIFO of McSPI peripheral. */
    MCSPI_enableTxFIFO(baseAddr, chNum, MCSPI_TX_FIFO_ENABLE);

    /* Disable the receiver FIFO of McSPI peripheral for Tx only mode. */
    MCSPI_enableRxFIFO(baseAddr, chNum, MCSPI_RX_FIFO_DISABLE);

    /*
     * Channel Control and config registers are updated after Open/Reconfigure.
     * Channel enable/disable and CS assert/deassert require updation of bits in
     * control and config registers. Also these registers will not be updated
     * during data transfer. So reg read modify write operations can be updated
     * to write only operations.
     * Store ch enable/disable reg val and cs assert/deassert reg vals.
     */
    chCtrlRegVal     = MCSPI_readChCtrlReg(baseAddr, chNum);
    gChEnableRegVal  = chCtrlRegVal | CSL_MCSPI_CH0CTRL_EN_MASK;
    gChDisableRegVal = chCtrlRegVal & (~CSL_MCSPI_CH0CTRL_EN_MASK);

    chConfRegVal      = MCSPI_readChConf(baseAddr, chNum);
    gCsAssertRegVal   = chConfRegVal | CSL_MCSPI_CH0CONF_FORCE_MASK;
    gCsDeAssertRegVal = chConfRegVal & (~CSL_MCSPI_CH0CONF_FORCE_MASK);

    /*  Calculate buffer width shift value.
     *  When dataWidth <= 8,           bufWidth = uint8_t  (1 byte - 0 shift)
     *  When dataWidth > 8  && <= 16,  bufWidth = uint16_t (2 bytes - 1 shift)
     *  When dataWidth > 16 && <= 32,  bufWidth = uint32_t (4 bytes - 2 shift)
     */
    dataWidth  = dataSize;
    bufWidthShift = MCSPI_getBufWidthShift(dataWidth);
    dataLength = APP_MCSPI_TXONLYMSGSIZE;
    bitRate    = gConfigMcspi0ChCfg[0U].bitRate;

    /* Initiate transfer */
    startTimeInUSec = ClockP_getTimeUsec();
    for(j = 0U; j < APP_MCSPI_TRANSFER_LOOPCOUNT; j++)
    {
        mcspi_low_latency_transfer_16bit(baseAddr, chNum,
                                   &gMcspiPerfTxBuffer[0], dataLength, bufWidthShift);
    }
    elapsedTimeInUsecs = ClockP_getTimeUsec() - startTimeInUSec;

    DebugP_log("----------------------------------------------------------\r\n");
    DebugP_log("McSPI Clock %d Hz\r\n", bitRate);
    DebugP_log("----------------------------------------------------------\r\n");
    DebugP_log("Data Width \tData Length \tTransfer Time (micro sec)\r\n");
    DebugP_log("%u\t\t%u\t\t%5.2f\r\n", dataWidth, dataLength,
                        (float)elapsedTimeInUsecs / APP_MCSPI_TRANSFER_LOOPCOUNT);
    DebugP_log("----------------------------------------------------------\r\n\n");

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    return;
}

void test_mcspi_loopback_simultaneous(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction, spiTransaction1;
    MCSPI_Handle        mcspiHandle, mcspiHandle1;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    uint16_t           *tempRxPtr16 = NULL, *tempTxPtr16 = NULL;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL, dataWidth1, bufWidthShift;
    MCSPI_TestParams    testParams1;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);
    MCSPI_ChConfig     *mcspiChConfigParams = &(testParams->mcspiChConfigParams);
    MCSPI_OpenParams   *mcspiOpenParams1 = &(testParams1.mcspiOpenParams);
    MCSPI_ChConfig     *mcspiChConfigParams1 = &(testParams1.mcspiChConfigParams);
    MCSPI_Config     *config = &gMcspiConfig[CONFIG_MCSPI1];
    MCSPI_Attrs      *attrParams = (MCSPI_Attrs *)config->attrs;

    mcspiOpenParams->transferMode = MCSPI_TRANSFER_MODE_BLOCKING;
    /* Instance 1 Init params */
    test_mcspi_set_params(&testParams1, 1009);
#if (CONFIG_MCSPI_NUM_INSTANCES > 2U)
    attrParams->baseAddr           = MCSPI1_BASE_ADDRESS;
    attrParams->intrNum            = MCSPI1_INT_NUM;
#else /* LP Case */
    attrParams->baseAddr           = MCSPI3_BASE_ADDRESS;
    attrParams->intrNum            = MCSPI3_INT_NUM;
#endif
    attrParams->operMode         = MCSPI_OPER_MODE_INTERRUPT;
    attrParams->intrPriority       = 4U;
    mcspiChConfigParams1->bitRate            = 12500000;
    mcspiChConfigParams1->csPolarity         = MCSPI_CS_POL_HIGH;
    testParams1.dataSize           = 16;

    /* Memset Buffers */
    memset(&gMcspiTxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiTxBuffer[0U]));
    memset(&gMcspiRxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiRxBuffer[0U]));
    /* Memset Buffers */
    memset(&gMcspiTxBuffer1[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiTxBuffer1[0U]));
    memset(&gMcspiRxBuffer1[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiRxBuffer1[0U]));

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);
    MCSPI_close(gMcspiHandle[CONFIG_MCSPI1]);

    mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);
    mcspiHandle1 = MCSPI_open(CONFIG_MCSPI1, mcspiOpenParams1);
    TEST_ASSERT_NOT_NULL(mcspiHandle1);

    status = MCSPI_chConfig(
                 gMcspiHandle[CONFIG_MCSPI0],
                 mcspiChConfigParams);
    DebugP_assert(status == SystemP_SUCCESS);
    status = MCSPI_chConfig(
                 gMcspiHandle[CONFIG_MCSPI1],
                 mcspiChConfigParams1);
    DebugP_assert(status == SystemP_SUCCESS);

    if(mcspiOpenParams1->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    dataWidth = testParams->dataSize;
    bufWidthShift = MCSPI_getBufWidthShift(dataWidth);
    if (dataWidth <= 32U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr32 = (uint32_t *) &gMcspiTxBuffer[0U];
        tempRxPtr32 = (uint32_t *) &gMcspiRxBuffer[0U];
    }

    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers */
    for (i = 0U; i < testParams->transferLength; i++)
    {
        tempTxData = 0xDEADBABE;
        tempTxData &= (fifoBitMask);
        *tempTxPtr32++ = (uint32_t) (tempTxData);
        *tempRxPtr32++ = 0U;
    }

    dataWidth1 = testParams1.dataSize;
    bufWidthShift = MCSPI_getBufWidthShift(dataWidth1);
    if (dataWidth1 <= 16U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr16 = (uint16_t *) &gMcspiTxBuffer1[0U];
        tempRxPtr16 = (uint16_t *) &gMcspiRxBuffer1[0U];
    }

    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth1; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers */
    for (i = 0U; i < (APP_MCSPI_MSGSIZE * (sizeof(gMcspiTxBuffer1[0U]) / (1 << bufWidthShift))); i++)
    {
        tempTxData = 0xBEDEEFAD;
        tempTxData &= (fifoBitMask);
        *tempTxPtr16++ = (uint16_t) (tempTxData);
        *tempRxPtr16++ = 0U;
    }

    /* Initiate transfer */
    spiTransaction1.channel  = testParams1.mcspiChConfigParams.chNum;
    spiTransaction1.dataSize  = testParams1.dataSize;
    spiTransaction1.csDisable = TRUE;
    spiTransaction1.count    = (APP_MCSPI_MSGSIZE * (sizeof(gMcspiTxBuffer1[0U]) / (1 << bufWidthShift)));
    spiTransaction1.txBuf    = (void *)gMcspiTxBuffer1;
    spiTransaction1.rxBuf    = (void *)gMcspiRxBuffer1;
    spiTransaction1.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI1], &spiTransaction1);
    TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction1);

    /* Initiate transfer */
    spiTransaction.channel  = testParams->mcspiChConfigParams.chNum;
    spiTransaction.count    = testParams->transferLength;
    spiTransaction.dataSize  = dataWidth;
    spiTransaction.csDisable = TRUE;
    spiTransaction.txBuf    = (void *)gMcspiTxBuffer;
    spiTransaction.rxBuf    = (void *)gMcspiRxBuffer;
    spiTransaction.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction);

    if(mcspiOpenParams1->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for transfer completion */
        SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Compare data */
    uint8_t *tempTxPtr, *tempRxPtr;
    tempTxPtr = (uint8_t *) &gMcspiTxBuffer[0U];
    tempRxPtr = (uint8_t *) &gMcspiRxBuffer[0U];
    for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
    {
        if(*tempTxPtr++ != *tempRxPtr++)
        {
            status = SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    /* Compare data */
    tempTxPtr = (uint8_t *) &gMcspiTxBuffer1[0U];
    tempRxPtr = (uint8_t *) &gMcspiRxBuffer1[0U];
    for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
    {
        if(*tempTxPtr++ != *tempRxPtr++)
        {
            status = SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    if(mcspiOpenParams1->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gMcspiTransferDoneSem);
    }

    /* Memset Buffers */
    memset(&gMcspiTxBuffer1[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiTxBuffer1[0U]));
    memset(&gMcspiRxBuffer1[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiRxBuffer1[0U]));

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);
    MCSPI_close(gMcspiHandle[CONFIG_MCSPI1]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

void test_mcspi_transfer_cancel(void *args)
{
    int32_t status = SystemP_SUCCESS;
    MCSPI_Handle        mcspiHandle;
    TaskP_Params transferTaskParms, transferCancelTaskParms;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    MCSPI_ChConfig     *mcspiChConfigParams = &(testParams->mcspiChConfigParams);
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);

    status = MCSPI_chConfig(
                 gMcspiHandle[CONFIG_MCSPI0],
                 mcspiChConfigParams);
    DebugP_assert(status == SystemP_SUCCESS);

    status = SemaphoreP_constructCounting(&gMcspiTransferTaskDoneSemaphoreObj, 0, 2);
    DebugP_assert(status == SystemP_SUCCESS);
    status = SemaphoreP_constructCounting(&gMcspiTransferCancelTaskDoneSemaphoreObj, 0, 2);
    DebugP_assert(status == SystemP_SUCCESS);

    TaskP_Params_init(&transferTaskParms);
    transferTaskParms.name = "MCSPI Transfer Task";
    transferTaskParms.stackSize = MCSPI_TASK_STACK_SIZE;
    transferTaskParms.stack = gMcspiTransferTaskStack;
    transferTaskParms.priority = MCSPI_TASK_PRIORITY;
    transferTaskParms.args = testParams;
    transferTaskParms.taskMain = test_mcspi_transfer_cancel_transfer;
    status = TaskP_construct(&gMcspiTransferTaskObject, &transferTaskParms);
    DebugP_assert(status == SystemP_SUCCESS);

    ClockP_usleep(1000);

    TaskP_Params_init(&transferCancelTaskParms);
    transferCancelTaskParms.name = "MCSPI Transfer Cancel Task";
    transferCancelTaskParms.stackSize = MCSPI_TASK_STACK_SIZE;
    transferCancelTaskParms.stack = gMcspiTransferCancelTaskStack;
    transferCancelTaskParms.priority = MCSPI_TASK_PRIORITY;
    transferCancelTaskParms.args = testParams;
    transferCancelTaskParms.taskMain = test_mcspi_transfer_cancel_cancel;
    status = TaskP_construct(&gMcspiTransferCancelTaskObject, &transferCancelTaskParms);
    DebugP_assert(status == SystemP_SUCCESS);

    SemaphoreP_pend(&gMcspiTransferTaskDoneSemaphoreObj, SystemP_WAIT_FOREVER);
    SemaphoreP_pend(&gMcspiTransferCancelTaskDoneSemaphoreObj, SystemP_WAIT_FOREVER);

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

}

void test_mcspi_transfer_cancel_transfer(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    uint8_t            *tempRxPtr8 = NULL, *tempTxPtr8 = NULL;
    uint16_t           *tempRxPtr16 = NULL, *tempTxPtr16 = NULL;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);

    /* Memset Buffers */
    memset(&gMcspiTxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiTxBuffer[0U]));
    memset(&gMcspiRxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(gMcspiRxBuffer[0U]));

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    dataWidth = testParams->dataSize;
    if (dataWidth < 9U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr8 = (uint8_t *) &gMcspiTxBuffer[0U];
        tempRxPtr8 = (uint8_t *) &gMcspiRxBuffer[0U];
    }
    else if (dataWidth < 17U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr16 = (uint16_t *) &gMcspiTxBuffer[0U];
        tempRxPtr16 = (uint16_t *) &gMcspiRxBuffer[0U];
    }
    else
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr32 = (uint32_t *) &gMcspiTxBuffer[0U];
        tempRxPtr32 = (uint32_t *) &gMcspiRxBuffer[0U];
    }
    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers */
    for (i = 0U; i < testParams->transferLength; i++)
    {
        tempTxData = 0xABCDBADC;
        tempTxData &= (fifoBitMask);
        if (dataWidth < 9U)
        {
            *tempTxPtr8++ = (uint8_t) (tempTxData);
            *tempRxPtr8++ = 0U;
        }
        else if (dataWidth < 17U)
        {
            *tempTxPtr16++ = (uint16_t) (tempTxData);
            *tempRxPtr16++ = 0U;
        }
        else
        {
            *tempTxPtr32++ = (uint32_t) (tempTxData);
            *tempRxPtr32++ = 0U;
        }
    }

    /* Initiate transfer */
    spiTransaction.channel  = testParams->mcspiChConfigParams.chNum;
    spiTransaction.dataSize  = dataWidth;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = testParams->transferLength;
    spiTransaction.txBuf    = (void *)gMcspiTxBuffer;
    spiTransaction.rxBuf    = (void *)gMcspiRxBuffer;
    spiTransaction.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_BLOCKING)
    {
        DebugP_assert(spiTransaction.status == MCSPI_TRANSFER_CANCELLED);
    }
    DebugP_assert(transferOK == SystemP_SUCCESS);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for transfer completion */
        SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(spiTransaction.status == MCSPI_TRANSFER_CANCELLED);
    }

    /* Compare data and should be mismatch as it is cancelled in another task */
    uint8_t *tempTxPtr, *tempRxPtr;
    tempTxPtr = (uint8_t *) &gMcspiTxBuffer[0U];
    tempRxPtr = (uint8_t *) &gMcspiRxBuffer[0U];
    for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
    {
        if(*tempTxPtr++ != *tempRxPtr++)
        {
            status = SystemP_SUCCESS;   /* Data mismatch */
            break;
        }
    }

    /* Initiate transfer */
    spiTransaction.channel   = testParams->mcspiChConfigParams.chNum;
    spiTransaction.dataSize  = dataWidth;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count     = testParams->transferLength;
    spiTransaction.txBuf     = (void *)gMcspiTxBuffer;
    spiTransaction.rxBuf     = (void *)gMcspiRxBuffer;
    spiTransaction.args      = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_BLOCKING)
    {
        DebugP_assert(spiTransaction.status == MCSPI_TRANSFER_COMPLETED);
    }
    DebugP_assert(transferOK == SystemP_SUCCESS);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for transfer completion */
        SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(spiTransaction.status == MCSPI_TRANSFER_COMPLETED);
    }

    /* Compare data and should be mismatch as it is cancelled in another task */
    tempTxPtr = (uint8_t *) &gMcspiTxBuffer[0U];
    tempRxPtr = (uint8_t *) &gMcspiRxBuffer[0U];
    for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
    {
        if(*tempTxPtr++ != *tempRxPtr++)
        {
            status = SystemP_FAILURE;   /* Data mismatch */
            DebugP_assert(FALSE);
            break;
        }
    }

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gMcspiTransferDoneSem);
    }

    SemaphoreP_post(&gMcspiTransferTaskDoneSemaphoreObj);
    TaskP_exit();

    return;
}

void test_mcspi_transfer_cancel_cancel(void *args)
{
    int32_t transferOK;

    transferOK = MCSPI_transferCancel(gMcspiHandle[CONFIG_MCSPI0]);
    DebugP_assert(transferOK == SystemP_SUCCESS);

    SemaphoreP_post(&gMcspiTransferCancelTaskDoneSemaphoreObj);
    TaskP_exit();
}

void test_mcspi_callback(MCSPI_Handle handle, MCSPI_Transaction *trans)
{
    DebugP_assertNoLog(MCSPI_TRANSFER_COMPLETED == trans->status);
    SemaphoreP_post(&gMcspiTransferDoneSem);

    return;
}

void test_mcspi_callback_cancel(MCSPI_Handle handle, MCSPI_Transaction *trans)
{
    SemaphoreP_post(&gMcspiTransferDoneSem);
    return;
}

void setUp(void)
{
}

void tearDown(void)
{
}

static void mcspi_low_latency_transfer_16bit(uint32_t baseAddr,
                                            uint32_t chNum,
                                            uint16_t *txBuff,
                                            uint32_t length,
                                            uint32_t bufWidthShift)
{
    /* Effective FIFO depth in bytes(64/32/16) depending on datawidth */
    uint32_t effTxFifoDepth = MCSPI_FIFO_LENGTH >> bufWidthShift;
    uint32_t i, numWordsWritten = 0U, transferLength = length;

    /* Enable the McSPI channel for communication.*/
    /* Updated for write only operation. */
    MCSPI_writeChCtrlReg(baseAddr, chNum, gChEnableRegVal);

    /* SPIEN line is forced to low state.*/
    /* Updated for write only operation. */
    MCSPI_writeChConfReg(baseAddr, chNum, gCsAssertRegVal);

    while (transferLength != 0)
    {
        /* Write Effective TX FIFO depth */
        if (transferLength >= effTxFifoDepth)
        {
            transferLength = effTxFifoDepth;
        }
        while (0 == (MCSPI_readChStatusReg(baseAddr, chNum) &
                        CSL_MCSPI_CH0STAT_TXFFE_MASK))
        {
            /* Wait fot Tx FIFO to be empty before writing the data. */
        }
        /* Write the data in Tx FIFO. */
        for (i = 0; i < transferLength; i++)
        {
            MCSPI_writeTxDataReg(baseAddr, (uint16_t) (*txBuff++), chNum);
        }
        numWordsWritten  += transferLength;
        transferLength    = length - numWordsWritten;
    }

    while (0 == (MCSPI_readChStatusReg(baseAddr, chNum) &
                    CSL_MCSPI_CH0STAT_TXFFE_MASK))
    {
        /* Wait fot Tx FIFO to be empty for the last set of data. */
    }
    while (0 == (MCSPI_readChStatusReg(baseAddr, chNum) &
                    CSL_MCSPI_CH0STAT_EOT_MASK))
    {
        /* Tx FIFO Empty is triggered when last word from FIFO is written to
           internal shift register. SO wait for the end of transfer of last word.
           The EOT gets set after every word when the transfer from shift
           register is complete and is reset when the transmission starts.
           So FIFO empty check is required to make sure the data in FIFO is
           sent out then wait for EOT for the last word. */
    }

    /* Force SPIEN line to the inactive state.*/
    /* Updated for write only operation. */
    MCSPI_writeChConfReg(baseAddr, chNum, gCsDeAssertRegVal);

    /* Disable the McSPI channel.*/
    /* Updated for write only operation. */
    MCSPI_writeChCtrlReg(baseAddr, chNum, gChDisableRegVal);
}

static void test_mcspi_set_params(MCSPI_TestParams *testParams, uint32_t tcId)
{
    uint32_t bufWidthShift;
    MCSPI_Config     *config = &gMcspiConfig[CONFIG_MCSPI0];
    MCSPI_Attrs      *attrParams = (MCSPI_Attrs *)config->attrs;
    MCSPI_OpenParams *openParams = &(testParams->mcspiOpenParams);
    MCSPI_ChConfig   *chConfigParams = &(testParams->mcspiChConfigParams);

    /* Default Attribute Parameters */
    attrParams->baseAddr           = MCSPI0_BASE_ADDRESS;
    attrParams->inputClkFreq       = 50000000U;
    attrParams->intrNum            = MCSPI0_INT_NUM;
    attrParams->operMode           = MCSPI_OPER_MODE_INTERRUPT;
    attrParams->intrPriority       = 4U;
    attrParams->chMode             = MCSPI_CH_MODE_SINGLE;
    attrParams->pinMode            = MCSPI_PINMODE_4PIN;
    attrParams->initDelay          = MCSPI_INITDLY_0;

    /* Default Open Parameters */
    openParams->transferMode           = MCSPI_TRANSFER_MODE_BLOCKING;
    openParams->transferTimeout        = SystemP_WAIT_FOREVER;
    openParams->transferCallbackFxn    = NULL;
    openParams->msMode                 = MCSPI_MS_MODE_MASTER;
    openParams->mcspiDmaIndex          = -1;

    /* Default Channel Config Parameters */
    chConfigParams->chNum              = MCSPI_CHANNEL_0;
    chConfigParams->frameFormat        = MCSPI_FF_POL0_PHA0;
    chConfigParams->bitRate            = 50000000;
    chConfigParams->csPolarity         = MCSPI_CS_POL_LOW;
    testParams->dataSize               = 32;
    chConfigParams->trMode             = MCSPI_TR_MODE_TX_RX;
    chConfigParams->inputSelect        = MCSPI_IS_D0;
    chConfigParams->dpe0               = MCSPI_DPE_ENABLE;
    chConfigParams->dpe1               = MCSPI_DPE_DISABLE;
    chConfigParams->slvCsSelect        = MCSPI_SLV_CS_SELECT_0;
    chConfigParams->startBitEnable     = FALSE;
    chConfigParams->startBitPolarity   = MCSPI_SB_POL_LOW;
    chConfigParams->csIdleTime         = MCSPI_TCS0_0_CLK;
    chConfigParams->defaultTxData      = 0x0U;
    chConfigParams->txFifoTrigLvl      = 16U;
    chConfigParams->rxFifoTrigLvl      = 16U;
    switch (tcId)
    {
        case 335:
            attrParams->operMode         = MCSPI_OPER_MODE_POLLED;
            break;
        case 336:
            openParams->transferMode = MCSPI_TRANSFER_MODE_CALLBACK;
            openParams->transferCallbackFxn    = test_mcspi_callback;
            break;
/* AM263X does not support MCU_SPI instance */
#if !defined(SOC_AM263X)
#if (CONFIG_MCSPI_NUM_INSTANCES > 2)
        case 970:
            attrParams->baseAddr           = CSL_MCU_MCSPI0_CFG_BASE;
            attrParams->intrNum            = 208U;
            break;
        case 971:
            attrParams->baseAddr           = CSL_MCU_MCSPI1_CFG_BASE;
            attrParams->intrNum            = 209U;
            testParams->dataSize           = 8;
            break;
#endif
#endif
        case 972:
            attrParams->baseAddr           = MCSPI4_BASE_ADDRESS;
            attrParams->operMode           = MCSPI_OPER_MODE_POLLED;
            break;
        case 973:
            attrParams->baseAddr           = MCSPI2_BASE_ADDRESS;
            attrParams->intrNum            = MCSPI2_INT_NUM;
            testParams->dataSize           = 16;
            break;
        case 974:
            attrParams->baseAddr           = MCSPI0_BASE_ADDRESS;
            attrParams->intrNum            = MCSPI0_INT_NUM;
            break;
        case 975:
            attrParams->baseAddr           = MCSPI4_BASE_ADDRESS;
            attrParams->intrNum            = MCSPI4_INT_NUM;
            break;
        case 980:
            testParams->dataSize           = 8;
            break;
        case 7351:
            testParams->dataSize           = 16;
            break;
        case 7352:
            testParams->dataSize           = 32;
            break;
        case 7353:
            testParams->dataSize           = 8;
            attrParams->operMode = MCSPI_OPER_MODE_POLLED;
            break;
        case 7354:
            testParams->dataSize           = 16;
            attrParams->operMode = MCSPI_OPER_MODE_POLLED;
            break;
        case 7355:
            testParams->dataSize           = 32;
            attrParams->operMode = MCSPI_OPER_MODE_POLLED;
            break;
        case 985:
            openParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
            openParams->transferCallbackFxn    = test_mcspi_callback;
            chConfigParams->bitRate            = 12207;
            break;
        case 991:
        case 992:
        case 993:
            attrParams->operMode = MCSPI_OPER_MODE_POLLED;
            break;
        case 995:
            testParams->dataSize = 16;
            break;
        case 996:
            testParams->dataSize = 8;
            break;
        case 997:
            attrParams->pinMode = MCSPI_PINMODE_3PIN;
            break;
        case 998:
            attrParams->initDelay = MCSPI_INITDLY_8;
            break;
        case 999:
            chConfigParams->csPolarity = MCSPI_CS_POL_HIGH;
            break;
        case 1000:
            chConfigParams->startBitEnable = TRUE;
            break;
        case 1001:
            chConfigParams->csIdleTime = MCSPI_TCS0_3_CLK;
            break;
        case 1002:
            chConfigParams->inputSelect = MCSPI_IS_D1;
            chConfigParams->dpe0        = MCSPI_DPE_DISABLE;
            chConfigParams->dpe1        = MCSPI_DPE_ENABLE;
            break;
        case 1003:
            openParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
            openParams->transferCallbackFxn    = test_mcspi_callback;
            break;
        case 1005:
            openParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
            openParams->transferCallbackFxn    = test_mcspi_callback;
            break;
        case 1006:
            attrParams->operMode               = MCSPI_OPER_MODE_INTERRUPT;
            break;
        case 1007:
            attrParams->baseAddr               = MCSPI1_BASE_ADDRESS;
            attrParams->intrNum                = MCSPI1_INT_NUM;
            break;
        case 1009:
            openParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
            openParams->transferCallbackFxn    = test_mcspi_callback;
            break;
        case 1011:
            testParams->dataSize               = 8;
            chConfigParams->bitRate            = 1000000;
            break;
        case 1012:
            testParams->dataSize               = 16;
            chConfigParams->bitRate            = 1000000;
            openParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
            openParams->transferCallbackFxn    = test_mcspi_callback_cancel;
            break;
        case 1014:
            openParams->transferTimeout        = 1U;
            chConfigParams->bitRate            = 50000;
            break;
        case 1565:
            attrParams->operMode               = MCSPI_OPER_MODE_POLLED;
            break;
#if (CONFIG_MCSPI_NUM_INSTANCES > 2)
        case 2394:
            attrParams->baseAddr               = MCSPI3_BASE_ADDRESS;
            testParams->dataSize               = 8;
            attrParams->operMode               = MCSPI_OPER_MODE_DMA;
            openParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
            openParams->transferCallbackFxn    = test_mcspi_callback;
            openParams->mcspiDmaIndex          = 0;
            break;
        case 7356:
            attrParams->baseAddr               = MCSPI3_BASE_ADDRESS;
            testParams->dataSize               = 16;
            attrParams->operMode               = MCSPI_OPER_MODE_DMA;
            openParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
            openParams->transferCallbackFxn    = test_mcspi_callback;
            openParams->mcspiDmaIndex          = 0;
            break;
        case 7357:
            attrParams->baseAddr               = MCSPI3_BASE_ADDRESS;
            testParams->dataSize               = 32;
            attrParams->operMode               = MCSPI_OPER_MODE_DMA;
            openParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
            openParams->transferCallbackFxn    = test_mcspi_callback;
            openParams->mcspiDmaIndex          = 0;
            break;
        case 2397:
            attrParams->baseAddr               = MCSPI3_BASE_ADDRESS;
            testParams->dataSize               = 16;
            attrParams->operMode               = MCSPI_OPER_MODE_DMA;
            openParams->mcspiDmaIndex          = 0;
            break;
#endif
    }

    bufWidthShift = MCSPI_getBufWidthShift(testParams->dataSize);
    /* If Count is less than FIFO trigger level */
    if ((tcId == 971U) || (tcId == 991U))
    {
        testParams->transferLength = (2U * (sizeof(gMcspiTxBuffer[0U]) / (1 << bufWidthShift)));
    }
    else
    {
        testParams->transferLength = (APP_MCSPI_MSGSIZE * (sizeof(gMcspiTxBuffer[0U]) / (1 << bufWidthShift)));
    }

    return;
}

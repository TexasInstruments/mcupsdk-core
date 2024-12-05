/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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

/*
 *  The application demonstrates MCSPI Controller operation by sending
 *  a known data from the Controller and then receiving the same from slave
 *  and finally compare the results.Ipc sync is used for syncronization
 *  between the cores.
 *
 *  Please connect pins as described below on AM64X/AM243X EVM.
 *  All the pins are available at SAFETY CONNECTOR J1 near power supply.
 *  MCU SPI1 pins are muxed with GPIO.
 *  MCU_SPI0_CS0(Pin 6)   ------------->   MCU_SPI1_CS0(Pin 7)
 *  MCU_SPI0_CLK(Pin 16)  ------------->   MCU_SPI1_CLK(Pin 9)
 *  MCU_SPI0_D0(Pin 4)    ------------->   MCU_SPI1_D1(Pin 12)
 *  MCU_SPI0_D1(Pin 2)    ------------->   MCU_SPI1_D0(Pin 5)
 *
 *  Please connect pins as described below on AM263x LP.
 *  MCU_SPI0_CS0(Pin 18)   ------------->   MCU_SPI1_CS0(Pin 58)
 *  MCU_SPI0_CLK(Pin 7)    ------------->   MCU_SPI1_CLK(Pin 47)
 *  MCU_SPI0_D0(Pin 55)    ------------->   MCU_SPI1_D1(Pin 14)
 *  MCU_SPI0_D1(Pin 54)    ------------->   MCU_SPI1_D0(Pin 15)
 *  
 *  Please connect pins as described below on AM263px LP.
 *  MCU_SPI0_CS0(Pin 8) (C11)   ------------->   MCU_SPI1_CS0(Pin 58) (C9)
 *  MCU_SPI0_CLK(Pin 7) (A11)   ------------->   MCU_SPI1_CLK(Pin 47) (A10)
 *  MCU_SPI0_D0(Pin 15) (C10)   ------------->   MCU_SPI1_D1(Pin 54)  (D9)
 *  MCU_SPI0_D1(Pin 14) (B11)   ------------->   MCU_SPI1_D0(Pin 55)  (B10)
 * 
 *
 *  Please connect pins as described below on AM261x LP.
 *  MCU_SPI0_CS0(Pin 19) (B13)  ------------>  MCU_SPI2_CS1(Pin 59) A18  
 *  MCU_SPI0_CLK(Pin 7) (A13)   ------------>  MCU_SPI2_CLK(Pin 47) D17 
 *  MCU_SPI0_D0(Pin 15) (B12)   ------------>  MCU_SPI2_D1(Pin 54)  B18
 *  MCU_SPI0_D1(Pin 14) (C12)   ------------>  MCU_SPI2_D0(Pin 55)  A16
 * 
 *  Please connect pins as described below on AM261x SOM on HSEC Board.
 *  MCU_SPI0_CS0 (C11) -> HSEC_SPI1_CS0(J20-16)  ------------>  MCU_SPI3_CS0 (D7) -> HSEC_SPI1_CS0(J20-12)
 *  MCU_SPI0_CLK (A11) -> HSEC_SPI1_CLK(J20-15)  ------------>  MCU_SPI3_CLK (C8) -> HSEC_SPI1_CLK(J20-11)
 *  MCU_SPI0_D0  (C10) -> HSEC_SPI1_CS0(J20-14)  ------------>  MCU_SPI3_D0 (C7) -> HSEC_SPI1_CS0(J20-10)
 *  MCU_SPI0_D1  (B11) -> HSEC_SPI1_CS0(J20-13)  ------------>  MCU_SPI3_D1 (B7) -> HSEC_SPI1_CS0(J20-9)
 * 
 */

#include "string.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <unity.h>

#define APP_MCSPI_MSGSIZE       (128U)
#define APP_PERF_LOOP_ITER_CNT  (10U)
#define TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || \
                ((MCSPI_TRANSFER_COMPLETED != transaction.status) && \
                (MCSPI_TRANSFER_STARTED != transaction.status))) \
        { \
            DebugP_assert(FALSE); /* MCSPI TX/RX failed!! */ \
        } \
    } while(0) \

typedef struct MCSPI_TestParams_s {
    MCSPI_ChConfig     *mcspiChConfigParams;
    MCSPI_OpenParams    mcspiOpenParams;
    uint32_t            testcaseId;
    uint32_t            dataSize;
} MCSPI_ControllerTestParams;

static uint32_t   gClkDividerTestListRampUp[] =
{
    1U,   2U,   3U,   4U,   5U,   6U,    7U,    8U,    9U,    10U,
    99U,  15U,  31U,  63U,  127U,  199U,  255U,  299U, 399U, 499U, 511U, 599U,
    699U, 799U, 899U, 999U, 1023U, 2047U, 3000U
};

static uint32_t   gClkDividerTestListRampDown[] =
{
    3000U, 2047U, 1023U, 999U, 899U, 799U,
    699U, 599U, 511U, 499U, 399U, 299U, 255U,
    199U, 127U, 99U, 63U, 31U, 15U, 10U, 9U,
    8U, 7U, 6U, 5U, 4U, 3U, 2U, 1U
};

#define SPI_TEST_NUM_CLK_LIST            (sizeof (gClkDividerTestListRampUp) / \
                                          sizeof (gClkDividerTestListRampUp[0U]))

uint8_t  gMcspiTxBuffer[APP_MCSPI_MSGSIZE];
uint8_t  gMcspiRxBuffer[APP_MCSPI_MSGSIZE];
uint32_t gMcspiControllerTxBuffer[APP_MCSPI_MSGSIZE];
uint32_t gMcspiControllerRxBuffer[APP_MCSPI_MSGSIZE];

extern MCSPI_Handle     gMcspiHandle[];
extern MCSPI_Config     gMcspiConfig[];
extern MCSPI_ChConfig  *gConfigMcspiChCfg[];

/* Semaphore to indicate Tx/Rx completion used in callback api's */
static SemaphoreP_Object gMcspiTransferDoneSem;

static int32_t mcspi_controller_transfer(uint32_t size);
void mcspi_controller_main(void *args);
void test_mcspi_callback(MCSPI_Handle handle, MCSPI_Transaction *trans);
static void test_mcspi_set_controller_params(MCSPI_ControllerTestParams *testParams, uint32_t tcId);
static void test_mcspi_controller_transfer(void *args);
static void test_mcspi_controller_transfer_performance(void *args);
static void test_mcspi_controller_transfer_8_16_32bit(void *args);

void test_mcspi_controller_main(void *args)
{
    MCSPI_ControllerTestParams  testParams;
    uint32_t          clkList;
    MCSPI_ChConfig   *chConfigParams;
    MCSPI_Config     *config;
    MCSPI_Attrs      *attrParams;

    testParams.mcspiChConfigParams = gConfigMcspiChCfg[MCSPI_CHANNEL_0];

    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    test_mcspi_set_controller_params(&testParams, 260);
    RUN_TEST(mcspi_controller_main,  260, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 943);
    RUN_TEST(test_mcspi_controller_transfer,  943, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 944);
    RUN_TEST(test_mcspi_controller_transfer,  944, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 945);
    RUN_TEST(test_mcspi_controller_transfer,  945, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 946);
    RUN_TEST(test_mcspi_controller_transfer,  946, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 947);
    RUN_TEST(test_mcspi_controller_transfer,  947, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 948);
    RUN_TEST(test_mcspi_controller_transfer,  948, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 949);
    RUN_TEST(test_mcspi_controller_transfer,  949, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 950);
    RUN_TEST(test_mcspi_controller_transfer,  950, (void*)&testParams);
#if !defined (SOC_AM261X)
    test_mcspi_set_controller_params(&testParams, 951);
    RUN_TEST(test_mcspi_controller_transfer,  951, (void*)&testParams);
#endif
    test_mcspi_set_controller_params(&testParams, 952);
    RUN_TEST(test_mcspi_controller_transfer,  952, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 953);
    RUN_TEST(test_mcspi_controller_transfer,  953, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 954);
    RUN_TEST(test_mcspi_controller_transfer,  954, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 955);
    /* Change clock divider as per test list */
    chConfigParams = testParams.mcspiChConfigParams;
    config = &gMcspiConfig[CONFIG_MCSPI0];
    attrParams = (MCSPI_Attrs *)config->attrs;
    for (clkList = 0U; clkList < SPI_TEST_NUM_CLK_LIST; clkList++)
    {
        chConfigParams->bitRate = (attrParams->inputClkFreq / (gClkDividerTestListRampUp[clkList] + 1));
        RUN_TEST(test_mcspi_controller_transfer,  955, (void*)&testParams);
    }
    test_mcspi_set_controller_params(&testParams, 956);
    chConfigParams = testParams.mcspiChConfigParams;
    config = &gMcspiConfig[CONFIG_MCSPI0];
    attrParams = (MCSPI_Attrs *)config->attrs;
    for (clkList = 0U; clkList < SPI_TEST_NUM_CLK_LIST; clkList++)
    {
        chConfigParams->bitRate = (attrParams->inputClkFreq / (gClkDividerTestListRampDown[clkList] + 1));
        RUN_TEST(test_mcspi_controller_transfer,  956, (void*)&testParams);
    }
    test_mcspi_set_controller_params(&testParams, 957);
    RUN_TEST(test_mcspi_controller_transfer,  957, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 958);
    RUN_TEST(test_mcspi_controller_transfer,  958, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 959);
    RUN_TEST(test_mcspi_controller_transfer,  959, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 960);
    RUN_TEST(test_mcspi_controller_transfer,  960, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 961);
    RUN_TEST(test_mcspi_controller_transfer,  961, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 962);
    RUN_TEST(test_mcspi_controller_transfer,  962, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 963);
    RUN_TEST(test_mcspi_controller_transfer,  963, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 964);
    RUN_TEST(test_mcspi_controller_transfer,  964, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 965);
    RUN_TEST(test_mcspi_controller_transfer,  965, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 966);
    RUN_TEST(test_mcspi_controller_transfer,  966, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 967);
    RUN_TEST(test_mcspi_controller_transfer,  967, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 968);
    /* Change clock divider as per test list */
    chConfigParams = testParams.mcspiChConfigParams;
    config = &gMcspiConfig[CONFIG_MCSPI0];
    attrParams = (MCSPI_Attrs *)config->attrs;
    for (clkList = 0U; clkList < SPI_TEST_NUM_CLK_LIST; clkList++)
    {
        chConfigParams->bitRate = (attrParams->inputClkFreq / (gClkDividerTestListRampUp[clkList] + 1));
        ClockP_usleep(500000);
        RUN_TEST(test_mcspi_controller_transfer,  968, (void*)&testParams);
    }
    test_mcspi_set_controller_params(&testParams, 969);
    chConfigParams = testParams.mcspiChConfigParams;
    config = &gMcspiConfig[CONFIG_MCSPI0];
    attrParams = (MCSPI_Attrs *)config->attrs;
    for (clkList = 0U; clkList < SPI_TEST_NUM_CLK_LIST; clkList++)
    {
        chConfigParams->bitRate = (attrParams->inputClkFreq / (gClkDividerTestListRampDown[clkList] + 1));
        DebugP_log("\n %d %d\n\r",clkList,chConfigParams->bitRate);
        ClockP_usleep(500000);
        RUN_TEST(test_mcspi_controller_transfer,  969, (void*)&testParams);
    }
    ClockP_sleep(1);
    test_mcspi_set_controller_params(&testParams, 976);
    RUN_TEST(test_mcspi_controller_transfer_performance,  976, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 1013);
    RUN_TEST(test_mcspi_controller_transfer,  1013, (void*)&testParams);
    test_mcspi_set_controller_params(&testParams, 2302);
    RUN_TEST(test_mcspi_controller_transfer_8_16_32bit,  2302, (void*)&testParams);

    UNITY_END();

    Board_driversClose();
    Drivers_close();

    return;
}

void mcspi_controller_main(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    int32_t             statusAll = SystemP_SUCCESS;
    uint32_t            size;

    DebugP_log("[MCSPI Controller] example started ...\r\n");

    size = APP_MCSPI_MSGSIZE/4;
    status = mcspi_controller_transfer(size);
    if (status != SystemP_SUCCESS)
    {
        statusAll += status;
        DebugP_log("[MCSPI Controller] test failed for size: %d!!\r\n", size);
    }

    size = APP_MCSPI_MSGSIZE/2;
    status = mcspi_controller_transfer(size);
    if (status != SystemP_SUCCESS)
    {
        statusAll += status;
        DebugP_log("[MCSPI Controller] test failed for size: %d!!\r\n", size);
    }

    size = APP_MCSPI_MSGSIZE;
    status = mcspi_controller_transfer(size);
    if (status != SystemP_SUCCESS)
    {
        statusAll += status;
        DebugP_log("[MCSPI controller] test failed for size: %d!!\r\n", size);
    }

    /* wait for mcspi slave to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, statusAll);

    return;
}

static int32_t mcspi_controller_transfer(uint32_t size)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;

    DebugP_log("[MCSPI Controller] transfer test with size:%d ...\r\n", size);

    /* wait for mcspi slave to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Memfill buffers */
    for(i = 0U; i < size; i++)
    {
        gMcspiTxBuffer[i] = i + 1U;
        gMcspiRxBuffer[i] = 0U;
    }

    /*
     * MCSPI transfer is a blocking call, so cant add sync after calling the transfer function.
     * added a small delay in controller side so that the slave side will call the transfer function.
     */
    ClockP_usleep(5000);

    /* Initiate transfer */
    spiTransaction.channel  = gConfigMcspi0ChCfg[0].chNum;
    spiTransaction.dataSize  = 8;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = size / (spiTransaction.dataSize/8);
    spiTransaction.txBuf    = (void *)gMcspiTxBuffer;
    spiTransaction.rxBuf    = (void *)gMcspiRxBuffer;
    spiTransaction.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction);

    /* Compare data */
    for(i = 0U; i < size; i++)
    {
        if(gMcspiTxBuffer[i] != gMcspiRxBuffer[i])
        {
            status = SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("[MCSPI Slave] Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    return status;
}

static void test_mcspi_controller_transfer(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    uint32_t            bufWidthShift;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    MCSPI_Handle        mcspiHandle;
    MCSPI_ControllerTestParams   *testParams = (MCSPI_ControllerTestParams *)args;
    uint8_t            *tempRxPtr8 = NULL, *tempTxPtr8 = NULL;
    uint16_t           *tempRxPtr16 = NULL, *tempTxPtr16 = NULL;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);

    /* Memset Buffers */
    memset(&gMcspiControllerTxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(uint32_t));
    memset(&gMcspiControllerRxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(uint32_t));

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    /* wait for mcspi slave to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    dataWidth = testParams->dataSize;
    bufWidthShift = MCSPI_getBufWidthShift(dataWidth);
    if (dataWidth < 9U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr8 = (uint8_t *) &gMcspiControllerTxBuffer[0U];
        tempRxPtr8 = (uint8_t *) &gMcspiControllerRxBuffer[0U];
    }
    else if (dataWidth < 17U)
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr16 = (uint16_t *) &gMcspiControllerTxBuffer[0U];
        tempRxPtr16 = (uint16_t *) &gMcspiControllerRxBuffer[0U];
    }
    else
    {
        /* Init TX buffer with known data and memset RX buffer */
        tempTxPtr32 = (uint32_t *) &gMcspiControllerTxBuffer[0U];
        tempRxPtr32 = (uint32_t *) &gMcspiControllerRxBuffer[0U];
    }
    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers */
    for (i = 0U; i < (APP_MCSPI_MSGSIZE  * (sizeof(uint32_t) / (1 << bufWidthShift))); i++)
    {
        tempTxData = 0xDEADBABE + i;
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

    /*
     * MCSPI transfer is a blocking call, so cant add sync after calling the transfer function.
     * added a small delay in controller side so that the slave side will call the transfer function.
     */
    ClockP_usleep(5000);

    /* Initiate transfer */
    spiTransaction.channel  = testParams->mcspiChConfigParams->chNum;
    spiTransaction.dataSize  = testParams->dataSize;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = (APP_MCSPI_MSGSIZE  * (sizeof(uint32_t) / (1 << bufWidthShift)));
    if(testParams->testcaseId == 1013U)
    {
        spiTransaction.txBuf = NULL;
    }
    else
    {
        spiTransaction.txBuf = (void *)gMcspiControllerTxBuffer;
    }
    spiTransaction.rxBuf    = (void *)gMcspiControllerRxBuffer;
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
    tempTxPtr = (uint8_t *) &gMcspiControllerTxBuffer[0U];
    tempRxPtr = (uint8_t *) &gMcspiControllerRxBuffer[0U];
    for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
    {
        if(*tempTxPtr++ != *tempRxPtr++)
        {
        /*  By default MCSPI is configured as TX only and data should not be received
            in RX buffer. */
            if ((testParams->testcaseId == 954U) || (testParams->testcaseId == 967U))
            {
                status = SystemP_FAILURE;
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
            else
            {
                status = SystemP_SUCCESS;
            }
            break;
        }
    }

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gMcspiTransferDoneSem);
    }

    /* wait for mcspi slave to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    return;
}

static void test_mcspi_controller_transfer_8_16_32bit(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    uint32_t            bufWidthShift;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    MCSPI_Handle        mcspiHandle;
    MCSPI_ControllerTestParams   *testParams = (MCSPI_ControllerTestParams *)args;
    uint8_t            *tempRxPtr8 = NULL, *tempTxPtr8 = NULL;
    uint16_t           *tempRxPtr16 = NULL, *tempTxPtr16 = NULL;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);
    uint8_t *tempTxPtr, *tempRxPtr;

    /* Memset Buffers */
    memset(&gMcspiControllerTxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(uint32_t));
    memset(&gMcspiControllerRxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(uint32_t));

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    /* wait for mcspi slave to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    dataWidth = 8U;
    bufWidthShift = MCSPI_getBufWidthShift(dataWidth);

    /* Init TX buffer with known data and memset RX buffer */
    tempTxPtr8 = (uint8_t *) &gMcspiControllerTxBuffer[0U];
    tempRxPtr8 = (uint8_t *) &gMcspiControllerRxBuffer[0U];
    /* Init TX buffer with known data and memset RX buffer */
    tempTxPtr16 = (uint16_t *) &gMcspiControllerTxBuffer[0U];
    tempRxPtr16 = (uint16_t *) &gMcspiControllerRxBuffer[0U];
    /* Init TX buffer with known data and memset RX buffer */
    tempTxPtr32 = (uint32_t *) &gMcspiControllerTxBuffer[0U];
    tempRxPtr32 = (uint32_t *) &gMcspiControllerRxBuffer[0U];

    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers */
    for (i = 0U; i < (APP_MCSPI_MSGSIZE  * (sizeof(uint32_t) / (1 << bufWidthShift))); i++)
    {
        tempTxData = 0xDEADBABE + i;
        tempTxData &= (fifoBitMask);
        *tempTxPtr8++ = (uint8_t) (tempTxData);
        *tempRxPtr8++ = 0U;
    }

    /*
     * MCSPI transfer is a blocking call, so cant add sync after calling the transfer function.
     * added a small delay in controller side so that the slave side will call the transfer function.
     */
    ClockP_usleep(5000);

    /* Initiate transfer */
    spiTransaction.channel  = testParams->mcspiChConfigParams->chNum;
    spiTransaction.dataSize  = dataWidth;
    spiTransaction.csDisable = FALSE;
    spiTransaction.count    = (APP_MCSPI_MSGSIZE  * (sizeof(uint32_t) / (1 << bufWidthShift)));
    if(testParams->testcaseId == 1013U)
    {
        spiTransaction.txBuf = NULL;
    }
    else
    {
        spiTransaction.txBuf = (void *)gMcspiControllerTxBuffer;
    }
    spiTransaction.rxBuf    = (void *)gMcspiControllerRxBuffer;
    spiTransaction.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for transfer completion */
        SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Compare data */
    tempTxPtr = (uint8_t *) &gMcspiControllerTxBuffer[0U];
    tempRxPtr = (uint8_t *) &gMcspiControllerRxBuffer[0U];
    for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
    {
        if(*tempTxPtr++ != *tempRxPtr++)
        {
        /*  By default MCSPI is configured as TX only and data should not be received
            in RX buffer. */
            if ((testParams->testcaseId == 954U) || (testParams->testcaseId == 967U))
            {
                status = SystemP_FAILURE;
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
            else
            {
                status = SystemP_SUCCESS;
            }
            break;
        }
    }

    /*
     * MCSPI transfer is a blocking call, so cant add sync after calling the transfer function.
     * added a small delay in controller side so that the slave side will call the transfer function.
     */
    ClockP_usleep(5000);
    /* Memset Buffers */
    memset(&gMcspiControllerTxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(uint32_t));
    memset(&gMcspiControllerRxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(uint32_t));
    dataWidth = 16U;
    bufWidthShift = MCSPI_getBufWidthShift(dataWidth);

    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers */
    for (i = 0U; i < (APP_MCSPI_MSGSIZE  * (sizeof(uint32_t) / (1 << bufWidthShift))); i++)
    {
        tempTxData = 0xDEADBABE + i;
        tempTxData &= (fifoBitMask);
        *tempTxPtr16++ = (uint16_t) (tempTxData);
        *tempRxPtr16++ = 0U;
    }

    /* Initiate transfer */
    spiTransaction.channel  = testParams->mcspiChConfigParams->chNum;
    spiTransaction.dataSize  = dataWidth;
    spiTransaction.csDisable = FALSE;
    spiTransaction.count    = (APP_MCSPI_MSGSIZE  * (sizeof(uint32_t) / (1 << bufWidthShift)));
    if(testParams->testcaseId == 1013U)
    {
        spiTransaction.txBuf = NULL;
    }
    else
    {
        spiTransaction.txBuf = (void *)gMcspiControllerTxBuffer;
    }
    spiTransaction.rxBuf    = (void *)gMcspiControllerRxBuffer;
    spiTransaction.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for transfer completion */
        SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Compare data */
    tempTxPtr = (uint8_t *) &gMcspiControllerTxBuffer[0U];
    tempRxPtr = (uint8_t *) &gMcspiControllerRxBuffer[0U];
    for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
    {
        if(*tempTxPtr++ != *tempRxPtr++)
        {
        /*  By default MCSPI is configured as TX only and data should not be received
            in RX buffer. */
            if ((testParams->testcaseId == 954U) || (testParams->testcaseId == 967U))
            {
                status = SystemP_FAILURE;
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
            else
            {
                status = SystemP_SUCCESS;
            }
            break;
        }
    }

    /*
     * MCSPI transfer is a blocking call, so cant add sync after calling the transfer function.
     * added a small delay in controller side so that the slave side will call the transfer function.
     */
    ClockP_usleep(5000);
    /* Memset Buffers */
    memset(&gMcspiControllerTxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(uint32_t));
    memset(&gMcspiControllerRxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(uint32_t));
    dataWidth = 32U;
    bufWidthShift = MCSPI_getBufWidthShift(dataWidth);

    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers */
    for (i = 0U; i < (APP_MCSPI_MSGSIZE  * (sizeof(uint32_t) / (1 << bufWidthShift))); i++)
    {
        tempTxData = 0xDEADBABE + i;
        tempTxData &= (fifoBitMask);
        *tempTxPtr32++ = (uint32_t) (tempTxData);
        *tempRxPtr32++ = 0U;
    }

    /* Initiate transfer */
    spiTransaction.channel  = testParams->mcspiChConfigParams->chNum;
    spiTransaction.dataSize  = dataWidth;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = (APP_MCSPI_MSGSIZE  * (sizeof(uint32_t) / (1 << bufWidthShift)));
    if(testParams->testcaseId == 1013U)
    {
        spiTransaction.txBuf = NULL;
    }
    else
    {
        spiTransaction.txBuf = (void *)gMcspiControllerTxBuffer;
    }
    spiTransaction.rxBuf    = (void *)gMcspiControllerRxBuffer;
    spiTransaction.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for transfer completion */
        SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Compare data */
    tempTxPtr = (uint8_t *) &gMcspiControllerTxBuffer[0U];
    tempRxPtr = (uint8_t *) &gMcspiControllerRxBuffer[0U];
    for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
    {
        if(*tempTxPtr++ != *tempRxPtr++)
        {
        /*  By default MCSPI is configured as TX only and data should not be received
            in RX buffer. */
            if ((testParams->testcaseId == 954U) || (testParams->testcaseId == 967U))
            {
                status = SystemP_FAILURE;
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
            else
            {
                status = SystemP_SUCCESS;
            }
            break;
        }
    }

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gMcspiTransferDoneSem);
    }

    /* wait for mcspi slave to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    return;
}

void test_mcspi_callback(MCSPI_Handle handle, MCSPI_Transaction *trans)
{
    DebugP_assertNoLog(MCSPI_TRANSFER_COMPLETED == trans->status);
    SemaphoreP_post(&gMcspiTransferDoneSem);

    return;
}

static void test_mcspi_controller_transfer_performance(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i,j, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    uint32_t            bufWidthShift, dataLength;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    MCSPI_Handle        mcspiHandle;
    MCSPI_ControllerTestParams   *testParams = (MCSPI_ControllerTestParams *)args;
    uint8_t            *tempRxPtr8 = NULL, *tempTxPtr8 = NULL;
    uint16_t           *tempRxPtr16 = NULL, *tempTxPtr16 = NULL;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);
    MCSPI_ChConfig     *mcspiChConfigParams = testParams->mcspiChConfigParams;
    uint64_t            startTimeInUSec, elapsedTimeInUsecs, totalTimeInUsecs = 0U;

    /* Memset Buffers */
    /* Initiate transfer */
    for(j = 0U; j < APP_PERF_LOOP_ITER_CNT; j++)
    {
        memset(&gMcspiControllerTxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(uint32_t));
        memset(&gMcspiControllerRxBuffer[0U], 0, APP_MCSPI_MSGSIZE * sizeof(uint32_t));

        MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

        mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
        TEST_ASSERT_NOT_NULL(mcspiHandle);

        if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
        {
            status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        /* wait for mcspi slave to be ready */
        IpcNotify_syncAll(SystemP_WAIT_FOREVER);

        dataWidth = testParams->dataSize;
        bufWidthShift = MCSPI_getBufWidthShift(dataWidth);
        if (dataWidth < 9U)
        {
            /* Init TX buffer with known data and memset RX buffer */
            tempTxPtr8 = (uint8_t *) &gMcspiControllerTxBuffer[0U];
            tempRxPtr8 = (uint8_t *) &gMcspiControllerRxBuffer[0U];
        }
        else if (dataWidth < 17U)
        {
            /* Init TX buffer with known data and memset RX buffer */
            tempTxPtr16 = (uint16_t *) &gMcspiControllerTxBuffer[0U];
            tempRxPtr16 = (uint16_t *) &gMcspiControllerRxBuffer[0U];
        }
        else
        {
            /* Init TX buffer with known data and memset RX buffer */
            tempTxPtr32 = (uint32_t *) &gMcspiControllerTxBuffer[0U];
            tempRxPtr32 = (uint32_t *) &gMcspiControllerRxBuffer[0U];
        }
        fifoBitMask = 0x0U;
        for (dataWidthIdx = 0U;
             dataWidthIdx < dataWidth; dataWidthIdx++)
        {
            fifoBitMask |= (1U << dataWidthIdx);
        }

        /* Memfill buffers */
        for (i = 0U; i < (APP_MCSPI_MSGSIZE  * (sizeof(uint32_t) / (1 << bufWidthShift))); i++)
        {
            tempTxData = 0xDEADBABE + i;
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

        /*
         * MCSPI transfer is a blocking call, so cant add sync after calling the transfer function.
         * added a small delay in controller side so that the slave side will call the transfer function.
         */
        ClockP_usleep(5000);

        spiTransaction.channel  = testParams->mcspiChConfigParams->chNum;
        spiTransaction.dataSize  = testParams->dataSize;
        spiTransaction.csDisable = TRUE;
        spiTransaction.count    = (APP_MCSPI_MSGSIZE  * (sizeof(uint32_t) / (1 << bufWidthShift)));
        if(testParams->testcaseId == 1013U)
        {
            spiTransaction.txBuf = NULL;
        }
        else
        {
            spiTransaction.txBuf = (void *)gMcspiControllerTxBuffer;
        }
        spiTransaction.rxBuf    = (void *)gMcspiControllerRxBuffer;
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
        tempTxPtr = (uint8_t *) &gMcspiControllerTxBuffer[0U];
        tempRxPtr = (uint8_t *) &gMcspiControllerRxBuffer[0U];
        for(i = 0U; i < (APP_MCSPI_MSGSIZE * 4); i++)
        {
            if(*tempTxPtr++ != *tempRxPtr++)
            {
            /*  By default MCSPI is configured as TX only and data should not be received
                in RX buffer. */
                if ((testParams->testcaseId == 954U) || (testParams->testcaseId == 967U))
                {
                    status = SystemP_FAILURE;
                    DebugP_log("Data Mismatch at offset %d\r\n", i);
                    break;
                }
                else
                {
                    status = SystemP_SUCCESS;
                }
                break;
            }
        }
    }

    dataLength = (APP_MCSPI_MSGSIZE  * (sizeof(uint32_t) / (1 << bufWidthShift)));
    DebugP_log("----------------------------------------------------------\r\n");
    DebugP_log("McSPI Clock %d Hz\r\n", mcspiChConfigParams->bitRate);
    DebugP_log("----------------------------------------------------------\r\n");
    DebugP_log("Data Width \tData Length \tTransfer Time (micro sec)\r\n");
    DebugP_log("%u\t\t%u\t\t%5.2f\r\n", testParams->dataSize, dataLength,
                        (float)totalTimeInUsecs / APP_PERF_LOOP_ITER_CNT);
    DebugP_log("----------------------------------------------------------\r\n\n");

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gMcspiTransferDoneSem);
    }

    /* wait for mcspi slave to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    return;
}

void setUp(void)
{
}

void tearDown(void)
{
}

static void test_mcspi_set_controller_params(MCSPI_ControllerTestParams *testParams, uint32_t tcId)
{
    MCSPI_Config     *config = &gMcspiConfig[CONFIG_MCSPI0];
    MCSPI_Attrs      *attrParams = (MCSPI_Attrs *)config->attrs;
    MCSPI_OpenParams *openParams = &(testParams->mcspiOpenParams);
    MCSPI_ChConfig   *chConfigParams = testParams->mcspiChConfigParams;
    testParams->testcaseId             = 0U;

    /* Default Attribute Parameters */
    attrParams->inputClkFreq       = 50000000U;
    attrParams->operMode           = MCSPI_OPER_MODE_INTERRUPT;
    attrParams->intrPriority       = 4U;
    attrParams->chMode             = MCSPI_CH_MODE_SINGLE;
    attrParams->pinMode            = MCSPI_PINMODE_4PIN;
    attrParams->initDelay          = MCSPI_INITDLY_0;

    /* Default Open Parameters */
    openParams->transferMode           = MCSPI_TRANSFER_MODE_BLOCKING;
    openParams->transferTimeout        = SystemP_WAIT_FOREVER;
    openParams->transferCallbackFxn    = NULL;
    openParams->msMode                 = MCSPI_MS_MODE_CONTROLLER;

    /* Default Channel Config Parameters */
    /* Adding extra dummy MCSPI instance for using
       chNum-1 for AM243X-LP */
#if (CONFIG_MCSPI_NUM_INSTANCES > 1)
    chConfigParams->chNum              = MCSPI_CHANNEL_1;
#else
    chConfigParams->chNum              = MCSPI_CHANNEL_0;
#endif
    chConfigParams->frameFormat        = MCSPI_FF_POL0_PHA0;
    chConfigParams->bitRate            = 1000000;
    chConfigParams->csPolarity         = MCSPI_CS_POL_LOW;
    testParams->dataSize               = 8;
    chConfigParams->trMode             = MCSPI_TR_MODE_TX_RX;
    chConfigParams->inputSelect        = MCSPI_IS_D1;
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
        case 943:
        attrParams->operMode           = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize           = 32;
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
        break;
        case 944:
        attrParams->operMode           = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize           = 16;
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
        break;
        case 945:
        attrParams->operMode           = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize           = 8;
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
        break;
        case 946:
        attrParams->operMode           = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize           = 32;
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
        chConfigParams->frameFormat    = MCSPI_FF_POL0_PHA1;
        break;
        case 947:
        attrParams->operMode           = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize           = 32;
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
        chConfigParams->frameFormat    = MCSPI_FF_POL1_PHA0;
        break;
        case 948:
        attrParams->operMode           = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize           = 32;
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
        chConfigParams->frameFormat    = MCSPI_FF_POL1_PHA1;
        break;
        case 949:
        attrParams->operMode           = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize           = 32;
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
        attrParams->pinMode            = MCSPI_PINMODE_3PIN;
        /* Adding chNum - 0 for am243x-lp */
#if (CONFIG_MCSPI_NUM_INSTANCES > 1)
        chConfigParams->chNum          = MCSPI_CHANNEL_0;
#endif
        break;
        case 950:
        attrParams->operMode           = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize           = 32;
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
        attrParams->initDelay          = MCSPI_INITDLY_32;
        break;
        case 951:
        attrParams->operMode           = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize           = 32;
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
        chConfigParams->csPolarity     = MCSPI_CS_POL_HIGH;
        break;
        case 952:
        attrParams->operMode               = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize               = 32;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        chConfigParams->startBitEnable     = TRUE;
        chConfigParams->startBitPolarity   = MCSPI_SB_POL_HIGH;
        break;
        case 953:
        attrParams->operMode               = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize               = 32;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        chConfigParams->csIdleTime         = MCSPI_TCS0_1_CLK;
        break;
        case 954:
        attrParams->operMode               = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize               = 8;
        chConfigParams->trMode             = MCSPI_TR_MODE_RX_ONLY;
        testParams->testcaseId             = 954U;
        break;
        case 955:
        case 956:
        attrParams->operMode               = MCSPI_OPER_MODE_POLLED;
        testParams->dataSize               = 32;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        break;
        case 957:
        attrParams->operMode               = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize               = 32;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        break;
        case 958:
        attrParams->operMode               = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize               = 15;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        break;
        case 959:
        attrParams->operMode               = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize               = 6;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        break;
        case 960:
        attrParams->operMode           = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize           = 32;
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
        attrParams->pinMode            = MCSPI_PINMODE_3PIN;
        /* Adding chNum - 0 for am243x-lp */
#if (CONFIG_MCSPI_NUM_INSTANCES > 1)
        chConfigParams->chNum          = MCSPI_CHANNEL_0;
#endif
        break;
        case 961:
        attrParams->operMode           = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize           = 32;
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
        attrParams->initDelay          = MCSPI_INITDLY_4;
        break;
        case 962:
        attrParams->operMode           = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize           = 32;
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
        chConfigParams->csPolarity     = MCSPI_CS_POL_LOW;
        break;
        case 963:
        attrParams->operMode               = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize               = 32;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        chConfigParams->startBitEnable     = TRUE;
        chConfigParams->startBitPolarity   = MCSPI_SB_POL_LOW;
        break;
        case 964:
        attrParams->operMode               = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize               = 32;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        chConfigParams->csIdleTime         = MCSPI_TCS0_2_CLK;
        break;
        case 965:
        attrParams->operMode               = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize               = 32;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        chConfigParams->inputSelect        = MCSPI_IS_D0;
        chConfigParams->dpe0               = MCSPI_DPE_DISABLE;
        chConfigParams->dpe1               = MCSPI_DPE_ENABLE;
        break;
        case 966:
        attrParams->operMode               = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize               = 32;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        openParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
        openParams->transferCallbackFxn    = test_mcspi_callback;
        break;
        case 967:
        attrParams->operMode               = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize               = 8;
        chConfigParams->trMode             = MCSPI_TR_MODE_RX_ONLY;
        testParams->testcaseId             = 967U;
        break;
        case 968:
        attrParams->operMode               = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize               = 8;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        break;
        case 969:
        attrParams->operMode               = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize               = 16;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        openParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
        openParams->transferCallbackFxn    = test_mcspi_callback;
        break;
        case 976:
        attrParams->operMode               = MCSPI_OPER_MODE_INTERRUPT;
        testParams->dataSize               = 8;
        chConfigParams->trMode             = MCSPI_TR_MODE_TX_ONLY;
        chConfigParams->bitRate            = 25000000;
        break;
        case 1013:
        chConfigParams->defaultTxData      = 0xCCU;
        testParams->testcaseId             = 1013U;
        break;
        case 2302:
        testParams->testcaseId             = 2302U;
    }

    return;
}

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

/*
 *  The application demonstrates MCSPI master DMA operation by sending
 *  a known data to the slave and then sending the same
 *  and finally compare the results.Ipc sync is used for syncronization
 *  between the cores.
 *  Please connect pins as described below on AM243X LP.
 *  Master SPI Pins                                 Slave SPI Pins
 *  -----------------                            ---------------------
 *  SPI0_CS1[Pin J6.6]     <----------------->     SPI3_CS0[Pin J2.6]
 *  SPI0_D0 [Pin J6.12]    <----------------->     SPI3_D1[Pin J2.14]
 *  SPI0_D1 [Pin J6.14]    <----------------->     SPI3_D0[Pin J2.12]
 *  SPI0_CLK [Pin J5.13]   <----------------->     SPI3_CLK[Pin J1.13]
 */

#include "string.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <drivers/mcspi/v0/dma/mcspi_dma.h>
#include <unity.h>

#define APP_MCSPI_MSGSIZE       (128U)
#define TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || \
                ((MCSPI_TRANSFER_COMPLETED != transaction.status) && \
                (MCSPI_TRANSFER_STARTED != transaction.status))) \
        { \
            DebugP_assert(FALSE); /* MCSPI TX/RX failed!! */ \
        } \
    } while(0) \

typedef struct MCSPI_MasterTestParams_s {
    MCSPI_ChConfig      mcspiChConfigParams;
    MCSPI_OpenParams    mcspiOpenParams;
    uint32_t            testcaseId;
    uint32_t            dataSize;
} MCSPI_MasterTestParams;

uint8_t  gMcspiTxBuffer[APP_MCSPI_MSGSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint8_t  gMcspiRxBuffer[APP_MCSPI_MSGSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint32_t gMcspiTxBuffer32[APP_MCSPI_MSGSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint32_t gMcspiRxBuffer32[APP_MCSPI_MSGSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/* Semaphore to indicate Tx/Rx completion used in callback api's */
static SemaphoreP_Object gMcspiTransferDoneSem;

static void mcspi_master_main(void *args);
static void test_mcspi_callback(MCSPI_Handle handle, MCSPI_Transaction *trans);
static int32_t mcspi_master_transfer(uint32_t size);
static void test_mcspi_set_master_params(MCSPI_MasterTestParams *testParams, uint32_t tcId);
static void test_mcspi_master_transfer(void *args);

void test_mcspi_master_dma_main(void *args)
{
    MCSPI_MasterTestParams  testParams;

    Drivers_open();

    UNITY_BEGIN();

    test_mcspi_set_master_params(&testParams, 2405);
    RUN_TEST(mcspi_master_main,  2405, (void*)&testParams);
    test_mcspi_set_master_params(&testParams, 2411);
    RUN_TEST(test_mcspi_master_transfer,  2411, (void*)&testParams);

    UNITY_END();

    Drivers_close();

    return;
}

static void mcspi_master_main(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    int32_t             statusAll = SystemP_SUCCESS;
    uint32_t            size;

    DebugP_log("[MCSPI Master] example started ...\r\n");

    size = APP_MCSPI_MSGSIZE/4;
    status = mcspi_master_transfer(size);
    if (status != SystemP_SUCCESS)
    {
        statusAll += status;
        DebugP_log("[MCSPI Master] test failed for size: %d!!\r\n", size);
    }

    size = APP_MCSPI_MSGSIZE/2;
    status = mcspi_master_transfer(size);
    if (status != SystemP_SUCCESS)
    {
        statusAll += status;
        DebugP_log("[MCSPI Master] test failed for size: %d!!\r\n", size);
    }

    size = APP_MCSPI_MSGSIZE;
    status = mcspi_master_transfer(size);
    if (status != SystemP_SUCCESS)
    {
        statusAll += status;
        DebugP_log("[MCSPI Master] test failed for size: %d!!\r\n", size);
    }

    /* wait for mcspi slave to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, statusAll);

    return;
}

static int32_t mcspi_master_transfer(uint32_t size)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;

    DebugP_log("[MCSPI Master] transfer test with size:%d ...\r\n", size);

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
     * added a small delay in master side so that the slave side will call the transfer function.
     */
    ClockP_usleep(5000);

    /* Writeback buffer */
    CacheP_wb(&gMcspiTxBuffer[0U], sizeof(gMcspiTxBuffer), CacheP_TYPE_ALLD);
    CacheP_wb(&gMcspiRxBuffer[0U], sizeof(gMcspiRxBuffer), CacheP_TYPE_ALLD);

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

    /* Invalidate cache */
    CacheP_inv(&gMcspiRxBuffer[0U], sizeof(gMcspiRxBuffer), CacheP_TYPE_ALLD);
    /* Compare data */
    for(i = 0U; i < size; i++)
    {
        if(gMcspiTxBuffer[i] != gMcspiRxBuffer[i])
        {
            status = SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("[MCSPI Master] Data Mismatch at offset %d\r\n", i);
            break;
        }
    }

    return status;
}

static void test_mcspi_master_transfer(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, dataWidth, fifoBitMask, tempTxData, dataWidthIdx;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    MCSPI_Handle        mcspiHandle;
    MCSPI_MasterTestParams   *testParams = (MCSPI_MasterTestParams *)args;
    uint32_t           *tempRxPtr32 = NULL, *tempTxPtr32 = NULL;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);
    MCSPI_ChConfig     *mcspiChConfigParams = &(testParams->mcspiChConfigParams);

    /* Memset Buffers */
    memset(&gMcspiTxBuffer32[0U], 0, APP_MCSPI_MSGSIZE);
    memset(&gMcspiRxBuffer32[0U], 0, APP_MCSPI_MSGSIZE);

    status = MCSPI_dmaClose(gMcspiHandle[CONFIG_MCSPI0],
                                &gConfigMcspi0ChCfg[0]);
    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    mcspiOpenParams->mcspiDmaIndex = 0;
    mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);

    status = MCSPI_chConfig(
                 gMcspiHandle[CONFIG_MCSPI0],
                 mcspiChConfigParams);
    DebugP_assert(status == SystemP_SUCCESS);

    status = MCSPI_dmaChConfig(
                 gMcspiHandle[CONFIG_MCSPI0],
                 mcspiChConfigParams,
                 &gConfigMcspi0DmaChCfg[0U]);
    DebugP_assert(status == SystemP_SUCCESS);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    /* wait for mcspi slave to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    dataWidth = testParams->dataSize;
    /* Init TX buffer with known data and memset RX buffer */
    tempTxPtr32 = (uint32_t *) &gMcspiTxBuffer32[0U];
    tempRxPtr32 = (uint32_t *) &gMcspiRxBuffer32[0U];

    fifoBitMask = 0x0U;
    for (dataWidthIdx = 0U;
         dataWidthIdx < dataWidth; dataWidthIdx++)
    {
        fifoBitMask |= (1U << dataWidthIdx);
    }

    /* Memfill buffers, TX data should be same as what done in Master application */
    for (i = 0U; i < APP_MCSPI_MSGSIZE; i++)
    {
        tempTxData = 0xDEADBABE + i;
        tempTxData &= (fifoBitMask);
        *tempTxPtr32++ = (uint32_t) (tempTxData);
        *tempRxPtr32++ = 0U;
    }

    /*
     * MCSPI transfer is a blocking call, so cant add sync after calling the transfer function.
     * added a small delay in master side so that the slave side will call the transfer function.
     */
    ClockP_usleep(5000);

    /* Writeback buffer */
    CacheP_wb(&gMcspiTxBuffer32[0U], sizeof(gMcspiTxBuffer32), CacheP_TYPE_ALLD);
    CacheP_wb(&gMcspiRxBuffer32[0U], sizeof(gMcspiRxBuffer32), CacheP_TYPE_ALLD);

    /* Initiate transfer */
    spiTransaction.channel   = testParams->mcspiChConfigParams.chNum;
    spiTransaction.dataSize  = testParams->dataSize;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = APP_MCSPI_MSGSIZE;
    spiTransaction.txBuf    = (void *)gMcspiTxBuffer32;
    spiTransaction.rxBuf    = (void *)gMcspiRxBuffer32;
    spiTransaction.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for transfer completion */
        SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Invalidate cache */
    CacheP_inv(&gMcspiRxBuffer32[0U], sizeof(gMcspiRxBuffer32), CacheP_TYPE_ALLD);

    /* Compare data */
    /* This parameter value should be same as master */
    for(i = 0U; i < (APP_MCSPI_MSGSIZE); i++)
    {
        if (gMcspiTxBuffer32[i] != gMcspiRxBuffer32[i])
        {
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_FAILURE;
            DebugP_log("Data Unexpectedly matched at offset %d\r\n", i);
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

void setUp(void)
{
}

void tearDown(void)
{
}

static void test_mcspi_callback(MCSPI_Handle handle, MCSPI_Transaction *trans)
{
    DebugP_assertNoLog(MCSPI_TRANSFER_COMPLETED == trans->status);
    SemaphoreP_post(&gMcspiTransferDoneSem);

    return;
}

static void test_mcspi_set_master_params(MCSPI_MasterTestParams *testParams, uint32_t tcId)
{
    MCSPI_Config     *config = &gMcspiConfig[CONFIG_MCSPI0];
    MCSPI_Attrs      *attrParams = (MCSPI_Attrs *)config->attrs;
    MCSPI_OpenParams *openParams = &(testParams->mcspiOpenParams);
    MCSPI_ChConfig   *chConfigParams = &(testParams->mcspiChConfigParams);
    testParams->testcaseId             = 0U;

    /* Default Attribute Parameters */
    attrParams->inputClkFreq       = 50000000U;
    attrParams->operMode           = MCSPI_OPER_MODE_DMA;
    attrParams->intrPriority       = 4U;
    attrParams->chMode             = MCSPI_CH_MODE_SINGLE;
    attrParams->pinMode            = MCSPI_PINMODE_4PIN;
    attrParams->initDelay          = MCSPI_INITDLY_0;

    /* Default Open Parameters */
    openParams->transferMode           = MCSPI_TRANSFER_MODE_BLOCKING;
    openParams->transferTimeout        = SystemP_WAIT_FOREVER;
    openParams->transferCallbackFxn    = NULL;
    openParams->msMode                 = MCSPI_MS_MODE_MASTER;

#if defined (SOC_AM263X)
    chConfigParams->chNum              = MCSPI_CHANNEL_0;
#else
    /* Default Channel Config Parameters */
    /* Adding extra dummy MCSPI instance for using
       chNum-1 for AM243X-LP */
    chConfigParams->chNum              = MCSPI_CHANNEL_1;
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

    switch (tcId)
    {
        case 2411:
        testParams->dataSize           = 32;
#if defined (SOC_AM263X)
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_ONLY;
#else
        chConfigParams->trMode         = MCSPI_TR_MODE_TX_RX;
#endif
        break;
    }

    return;
}

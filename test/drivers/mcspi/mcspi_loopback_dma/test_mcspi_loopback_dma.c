/*
 *  Copyright (C) 2021-25 Texas Instruments Incorporated
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

 #include "string.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <unity.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/mcspi/v0/lld/mcspi_lld.h>
#include <drivers/mcspi/v0/lld/dma/mcspi_dma.h>

/* Test Macros */
#define APP_MCSPI_MSGSIZE (1024U)

/* Test Parameters Structure */
typedef struct MCSPI_TestParams_s {
    MCSPI_ChConfig     *mcspiChConfigParams;
    MCSPI_OpenParams    mcspiOpenParams;
    uint32_t            transferLength;
    uint32_t            dataSize;
} MCSPI_TestParams;

/* External declarations from ti_drivers_config.h */
extern MCSPI_Handle         gMcspiHandle[];
extern MCSPI_Config         gMcspiConfig[];
extern MCSPI_ChConfig      *gConfigMcspiChCfg[];

/* Semaphore for transfer completion in callback mode */
SemaphoreP_Object gMcspiTransferDoneSem;

/* Test assertion macro */
#define TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || \
                ((MCSPI_TRANSFER_COMPLETED != transaction.status) && \
                (MCSPI_TRANSFER_STARTED != transaction.status))) \
        { \
            DebugP_assert(FALSE); \
        } \
    } while(0)

/* Forward declarations */
void test_mcspi_dma_loopback_transfer_4095bytes_positive(void *args);
void test_mcspi_dma_loopback_transfer_4096bytes_negative(void *args);
void test_mcspi_callback(MCSPI_Handle handle, MCSPI_Transaction *trans);
static void test_mcspi_set_params(MCSPI_TestParams *testParams, uint32_t testCaseId);

void test_mcspi_loopback_dma(void *args)
{
    MCSPI_TestParams  testParams;

    testParams.mcspiChConfigParams = gConfigMcspiChCfg[MCSPI_CHANNEL_0];

    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    test_mcspi_set_params(&testParams, 6432);
    RUN_TEST(test_mcspi_dma_loopback_transfer_4095bytes_positive, 15698, (void*)&testParams);
    test_mcspi_set_params(&testParams, 6432);
    RUN_TEST(test_mcspi_dma_loopback_transfer_4096bytes_negative, 15699, (void*)&testParams);

    UNITY_END();

    Drivers_close();
    Board_driversClose();
}

/* Unity framework setUp and tearDown functions */
void setUp(void)
{
    /* Setup code if needed */
}

void tearDown(void)
{
    /* Teardown code if needed */
}

/* Callback function for DMA transfer completion */
void test_mcspi_callback(MCSPI_Handle handle, MCSPI_Transaction *trans)
{
    SemaphoreP_post(&gMcspiTransferDoneSem);
}

/* Helper function to set test parameters based on test case ID */
static void test_mcspi_set_params(MCSPI_TestParams *testParams, uint32_t testCaseId)
{
    MCSPI_OpenParams *mcspiOpenParams = &(testParams->mcspiOpenParams);

    /* Set default parameters */
    testParams->transferLength = APP_MCSPI_MSGSIZE;
    testParams->dataSize = 8U;
    mcspiOpenParams->transferMode = MCSPI_TRANSFER_MODE_BLOCKING;
    mcspiOpenParams->transferTimeout = SystemP_WAIT_FOREVER;
    mcspiOpenParams->transferCallbackFxn = NULL;
    mcspiOpenParams->msMode = MCSPI_MS_MODE_CONTROLLER;
    mcspiOpenParams->mcspiDmaIndex = -1;
}

void test_mcspi_dma_loopback_transfer_4095bytes_positive(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);
    MCSPI_Config       *config;
    MCSPI_Attrs        *attrParams;
    MCSPI_Handle        mcspiHandle;
    uint8_t            *tempTxPtr8 = NULL, *tempRxPtr8 = NULL;

    /* 4095-byte DMA Loopback Test Buffers (MCUSDK-15698) */
    uint8_t      gMcspiTxBuffer4095[4095] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
    uint8_t      gMcspiRxBuffer4095[4095] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

    /* Memset Buffers */
    memset(&gMcspiTxBuffer4095[0U], 0, 4095);
    memset(&gMcspiRxBuffer4095[0U], 0, 4095);

    /* Close existing handle and reconfigure for DMA */
    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    config = &gMcspiConfig[CONFIG_MCSPI0];
    attrParams = (MCSPI_Attrs *)config->attrs;
    attrParams->operMode                    = MCSPI_OPER_MODE_DMA;
    mcspiOpenParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
    mcspiOpenParams->transferCallbackFxn    = test_mcspi_callback;
    mcspiOpenParams->mcspiDmaIndex          = 0;
    mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);
    gMcspiHandle[CONFIG_MCSPI0] = mcspiHandle;  /* Update global handle */

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    /* Initialize TX buffer with known data (8-bit) */
    tempTxPtr8 = (uint8_t *) &gMcspiTxBuffer4095[0U];
    tempRxPtr8 = (uint8_t *) &gMcspiRxBuffer4095[0U];

    for (i = 0U; i < 4095; i++)
    {
        *tempTxPtr8++ = (i + 1U) & 0xFF;
        *tempRxPtr8++ = 0U;
    }

    /* Writeback buffer */
    CacheP_wb(&gMcspiTxBuffer4095[0U], 4095, CacheP_TYPE_ALLD);
    CacheP_wb(&gMcspiRxBuffer4095[0U], 4095, CacheP_TYPE_ALLD);

    /* Initiate transfer */
    spiTransaction.channel  = gConfigMcspiChCfg[0U]->chNum;
    spiTransaction.dataSize = 8U;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = 4095 / (spiTransaction.dataSize / 8);
    spiTransaction.txBuf    = (void *)gMcspiTxBuffer4095;
    spiTransaction.rxBuf    = (void *)gMcspiRxBuffer4095;
    spiTransaction.args     = NULL;

    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    TEST_APP_MCSPI_ASSERT_ON_FAILURE(transferOK, spiTransaction);

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for transfer completion */
        SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Invalidate cache */
    CacheP_inv(&gMcspiRxBuffer4095[0U], 4095, CacheP_TYPE_ALLD);

    /* Compare data */
    tempTxPtr8 = (uint8_t *) &gMcspiTxBuffer4095[0U];
    tempRxPtr8 = (uint8_t *) &gMcspiRxBuffer4095[0U];
    for(i = 0U; i < 4095; i++)
    {
        if(*tempTxPtr8++ != *tempRxPtr8++)
        {
            status = SystemP_FAILURE;   /* Data mismatch */
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


void test_mcspi_dma_loopback_transfer_4096bytes_negative(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i;
    int32_t             transferOK;
    uint32_t            transferRejected = 0U;
    MCSPI_Transaction   spiTransaction;
    MCSPI_TestParams   *testParams = (MCSPI_TestParams *)args;
    MCSPI_OpenParams   *mcspiOpenParams = &(testParams->mcspiOpenParams);
    MCSPI_Config       *config;
    MCSPI_Attrs        *attrParams;
    MCSPI_Handle        mcspiHandle;
    uint8_t            *tempTxPtr8 = NULL, *tempRxPtr8 = NULL;
    uint8_t            gMcspiTxBuffer4096[5000] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
    uint8_t            gMcspiRxBuffer4096[5000] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

    /* Memset Buffers */
    memset(&gMcspiTxBuffer4096[0U], 0, 4096);
    memset(&gMcspiRxBuffer4096[0U], 0, 4096);

    /* Close existing handle and reconfigure for DMA */
    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    config = &gMcspiConfig[CONFIG_MCSPI0];
    attrParams = (MCSPI_Attrs *)config->attrs;
    attrParams->operMode                    = MCSPI_OPER_MODE_DMA;
    mcspiOpenParams->transferMode           = MCSPI_TRANSFER_MODE_CALLBACK;
    mcspiOpenParams->transferCallbackFxn    = test_mcspi_callback;
    mcspiOpenParams->mcspiDmaIndex          = 0;
    mcspiHandle = MCSPI_open(CONFIG_MCSPI0, mcspiOpenParams);
    TEST_ASSERT_NOT_NULL(mcspiHandle);
    gMcspiHandle[CONFIG_MCSPI0] = mcspiHandle;  /* Update global handle */

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    /* Initialize TX buffer with known data (8-bit) */
    tempTxPtr8 = (uint8_t *) &gMcspiTxBuffer4096[0U];
    tempRxPtr8 = (uint8_t *) &gMcspiRxBuffer4096[0U];

    for (i = 0U; i < 4096; i++)
    {
        *tempTxPtr8++ = i & 0xFF;
        *tempRxPtr8++ = 0U;
    }

    /* Writeback buffer */
    CacheP_wb(&gMcspiTxBuffer4096[0U], 4096, CacheP_TYPE_ALLD);
    CacheP_wb(&gMcspiRxBuffer4096[0U], 4096, CacheP_TYPE_ALLD);

    /* Initiate transfer with 4096 bytes - should be rejected */
    spiTransaction.channel  = gConfigMcspiChCfg[0U]->chNum;
    spiTransaction.dataSize = 8U;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = 4096 / (spiTransaction.dataSize / 8);  /* 4096 bytes */
    spiTransaction.txBuf    = (void *)gMcspiTxBuffer4096;
    spiTransaction.rxBuf    = (void *)gMcspiRxBuffer4096;
    spiTransaction.args     = NULL;

    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);

    /* Check if transfer was rejected (negative value indicates error) */
    if (transferOK < 0)
    {
        /* Transfer correctly rejected for exceeding 4095-byte limit */
        status = SystemP_SUCCESS;
        transferRejected = 1U;  /* Mark that transfer was rejected early */
    }
    else if (transferOK == SystemP_SUCCESS)
    {
        /* If transfer initiated, wait for completion or timeout */
        if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
        {
            /* Wait with timeout to avoid hang */
            status = SemaphoreP_pend(&gMcspiTransferDoneSem, 1000U); /* 1 second timeout */
            if (status != SystemP_SUCCESS)
            {
                status = SystemP_FAILURE;
            }
            else
            {
                /* Unexpected: transfer completed for 4096 bytes */
                status = SystemP_FAILURE;
            }
        }
    }

    if(mcspiOpenParams->transferMode == MCSPI_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gMcspiTransferDoneSem);
    }

    /* Close and reset to polled mode to avoid DMA cleanup errors when transfer was rejected */
    if (transferRejected == 1U)
    {
        /* Transfer was rejected: reset to polled mode before close to avoid DMA deinit errors */
        attrParams->operMode = MCSPI_OPER_MODE_POLLED;
        mcspiOpenParams->transferMode = MCSPI_TRANSFER_MODE_BLOCKING;
    }

    MCSPI_close(gMcspiHandle[CONFIG_MCSPI0]);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}


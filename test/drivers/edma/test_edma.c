/*
 *  Copyright (C) 2022-2023 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Value for A count*/
#define EDMA_TEST_A_COUNT           (16U)
/* Value for B count */
#define EDMA_TEST_B_COUNT           (4U)
/* Value for C count */
#define EDMA_TEST_C_COUNT           (2U)
/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO      (0U)
/**
 * Buffers (src and dest) are needed for mem-2-mem data transfers.
 * This define is for the MAXIMUM size and hence the maximum data
 * which could be transferred using the sample test cases below.
 */
#define EDMA_TEST_BUFFER_SIZE             (EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT)

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static void test_edmaInterrupt(void *args);
static void test_edmaATransfer(void *args);
static void test_edmaABTransfer(void *args);
static void test_edmaChainTransfer(void *args);
static void test_edmaLinkTransfer(void *args);
static void test_qdmaTransfer(void *args);
static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t            *srcBuffPtr, *dstBuffPtr;
EDMACCPaRAMEntry   edmaParam1, edmaParam2;

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gEdmaTestDoneSem;

/* The source buffer used for transfer */
static uint8_t gEdmaTestSrcBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
/* The destination buffer used for transfer */
static uint8_t gEdmaTestDstBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{

    uint32_t            baseAddr;

    Drivers_open();
    Board_driversOpen();

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    UNITY_BEGIN();

    RUN_TEST(test_qdmaTransfer, 1672, NULL);
    RUN_TEST(test_edmaATransfer, 1676, NULL);
    RUN_TEST(test_edmaABTransfer, 1675, NULL);
    RUN_TEST(test_edmaChainTransfer, 1674, NULL);
    RUN_TEST(test_edmaLinkTransfer, 1673, NULL);
    RUN_TEST(test_edmaInterrupt, 1677, NULL);

    UNITY_END();

    Board_driversClose();
    Drivers_close();
}

/*
 *   In this test, EDMA transfer in interrupt mode is verified.
 */
static void test_edmaInterrupt(void *args)
{
    Edma_IntrObject     intrObj;
    uint32_t            loopCnt = 0;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaCh, tcc, param;
    int32_t             status = SystemP_SUCCESS;
    int32_t             testStatus = SystemP_SUCCESS;

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaCh = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(status == SystemP_SUCCESS);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(status == SystemP_SUCCESS);

    param = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &param);
    DebugP_assert(status == SystemP_SUCCESS);

    /*
     * Initialize the source address with a pattern
     * initialize dst address with zero/another pattern (optional)
     */
    srcBuffPtr = (uint8_t *) gEdmaTestSrcBuff;
    dstBuffPtr = (uint8_t *) gEdmaTestDstBuff;

    for(loopCnt = 0U; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        srcBuffPtr[loopCnt] = (uint8_t)loopCnt;
        dstBuffPtr[loopCnt] = 0;
    }

    CacheP_wb((void *)srcBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    CacheP_wb((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, tcc, param, EDMA_TEST_EVT_QUEUE_NO);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam1);
    edmaParam1.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam1.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam1.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam1.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam1.cCnt          = (uint16_t) EDMA_TEST_C_COUNT;
    edmaParam1.bCntReload    = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam1.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam1.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam1.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam1.destCIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam1.linkAddr      = 0xFFFFU;
    edmaParam1.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam1.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam1.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(baseAddr, param, &edmaParam1);

    status = SemaphoreP_constructBinary(&gEdmaTestDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register interrupt */
    intrObj.tccNum = tcc;
    intrObj.cbFxn  = &EDMA_regionIsrFxn;
    intrObj.appData = (void *) &gEdmaTestDoneSem;
    status = EDMA_registerIntr(gEdmaHandle[0], &intrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    /*
     * Transfer is done in A sync mode
     * Number of triggers required are B_COUNT * C_COUNT
     */
    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMA_enableTransferRegion(
            baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);

        SemaphoreP_pend(&gEdmaTestDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Invalidate destination buffer and compare with src buffer */
    CacheP_inv((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    for(loopCnt = 0; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        if(srcBuffPtr[loopCnt] != dstBuffPtr[loopCnt])
        {
            testStatus = SystemP_FAILURE;
            break;
        }
    }

    status = EDMA_unregisterIntr(gEdmaHandle[0], &intrObj);
    SemaphoreP_destruct(&gEdmaTestDoneSem);

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeParam(gEdmaHandle[0], &param);
    DebugP_assert(status == SystemP_SUCCESS);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
}

static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);
    SemaphoreP_post(semObjPtr);
}

/*
 *   In this test, EDMA transfer in A synchronized transfer is verified
 *   in polling mode.
 */
static void test_edmaATransfer(void *args)
{
    uint32_t            loopCnt = 0;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaCh, tcc, param;
    int32_t             testStatus = SystemP_SUCCESS;

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaCh = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    param = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /*
     * Initialize the source address with a pattern
     * initialize dst address with zero/another pattern (optional)
     */
    srcBuffPtr = (uint8_t *) gEdmaTestSrcBuff;
    dstBuffPtr = (uint8_t *) gEdmaTestDstBuff;

    for(loopCnt = 0U; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        srcBuffPtr[loopCnt] = (uint8_t)loopCnt;
        dstBuffPtr[loopCnt] = 0;
    }
    CacheP_wb((void *)srcBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    CacheP_wb((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, tcc, param, EDMA_TEST_EVT_QUEUE_NO);

    /* Disable the interrupt for the channel to transfer in polled mode */
    EDMA_disableEvtIntrRegion(baseAddr, regionId, dmaCh);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam1);
    edmaParam1.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam1.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam1.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam1.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam1.cCnt          = (uint16_t) EDMA_TEST_C_COUNT;
    edmaParam1.bCntReload    = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam1.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam1.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam1.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam1.destCIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam1.linkAddr      = 0xFFFFU;
    edmaParam1.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam1.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam1.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(baseAddr, param, &edmaParam1);

    /*
     * Transfer is done in A sync mode
     * Number of triggers required are B_COUNT * C_COUNT
     */
    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMA_enableTransferRegion(baseAddr, regionId, dmaCh,
             EDMA_TRIG_MODE_MANUAL);

        while(EDMA_readIntrStatusRegion(baseAddr, regionId, tcc) != 1);

        EDMA_clrIntrRegion(baseAddr, regionId, tcc);
    }

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeParam(gEdmaHandle[0], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /* Invalidate destination buffer and compare with src buffer */
    CacheP_inv((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    for(loopCnt = 0; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        if(srcBuffPtr[loopCnt] != dstBuffPtr[loopCnt])
        {
            testStatus = SystemP_FAILURE;
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
}

/*
 *   In this test, EDMA transfer in AB synchronized mode is verified.
 */
static void test_edmaABTransfer(void *args)
{
    uint32_t            loopCnt = 0;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaCh, tcc, param;
    int32_t             testStatus = SystemP_SUCCESS;

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaCh = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    param = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /*
     * Initialize the source address with a pattern
     * initialize dst address with zero/another pattern (optional)
     */
    srcBuffPtr = (uint8_t *) gEdmaTestSrcBuff;
    dstBuffPtr = (uint8_t *) gEdmaTestDstBuff;
    for(loopCnt = 0U; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        srcBuffPtr[loopCnt] = (uint8_t)loopCnt;
        dstBuffPtr[loopCnt] = 0;
    }
    CacheP_wb((void *)srcBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    CacheP_wb((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, tcc, param, EDMA_TEST_EVT_QUEUE_NO);

    /* Disable the interrupt for the channel to transfer in polled mode */
    EDMA_disableEvtIntrRegion(baseAddr, regionId, dmaCh);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam1);
    edmaParam1.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam1.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam1.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam1.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam1.cCnt          = (uint16_t) EDMA_TEST_C_COUNT;
    edmaParam1.bCntReload    = 0;
    edmaParam1.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam1.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam1.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam1.destCIdx      = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam1.linkAddr      = 0xFFFFU;
    edmaParam1.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam1.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam1.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(baseAddr, param, &edmaParam1);

    /*
     * Transfer is done in AB sync mode
     * Number of triggers required is C_COUNT
     */
    for(loopCnt = 0; loopCnt < (EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMA_enableTransferRegion(baseAddr, regionId, dmaCh,
             EDMA_TRIG_MODE_MANUAL);

        while(EDMA_readIntrStatusRegion(baseAddr, regionId, tcc) != 1);

        EDMA_clrIntrRegion(baseAddr, regionId, tcc);
    }

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeParam(gEdmaHandle[0], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /* Invalidate destination buffer and compare with src buffer */
    CacheP_inv((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    for(loopCnt = 0; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        if(srcBuffPtr[loopCnt] != dstBuffPtr[loopCnt])
        {
            testStatus = SystemP_FAILURE;
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
}

/*
 *   In this test, EDMA chaining functionality is tested.
 */
static void test_edmaChainTransfer(void *args)
{
    uint32_t            loopCnt = 0;
    uint32_t            baseAddr, regionId;
    int32_t             testStatus = SystemP_SUCCESS;
    uint32_t            dmaCh0, tcc0, param0, dmaCh1, tcc1, param1;

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaCh0 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh0);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc0 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc0);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    param0 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param0);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    dmaCh1 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc1 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    param1 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /*
     * Initialize the source address with a pattern
     * initialize dst address with zero/another pattern (optional)
     */
    srcBuffPtr = (uint8_t *) gEdmaTestSrcBuff;
    dstBuffPtr = (uint8_t *) gEdmaTestDstBuff;

    uint32_t chainOptions = (EDMA_OPT_TCCHEN_MASK |
                             EDMA_OPT_ITCCHEN_MASK |
                             EDMA_OPT_TCINTEN_MASK |
                             EDMA_OPT_ITCINTEN_MASK);

    for(loopCnt = 0U; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        srcBuffPtr[loopCnt] = (uint8_t)loopCnt;
        dstBuffPtr[loopCnt] = 0;
    }
    CacheP_wb((void *)srcBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    CacheP_wb((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh0, tcc0, param0, EDMA_TEST_EVT_QUEUE_NO);

    /* Disable the interrupt for the channel to transfer in polled mode */
    EDMA_disableEvtIntrRegion(baseAddr, regionId, dmaCh0);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam1);
    edmaParam1.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam1.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam1.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam1.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam1.cCnt          = (uint16_t) EDMA_TEST_C_COUNT / 2;
    edmaParam1.bCntReload    = (uint16_t) 0;
    edmaParam1.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam1.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam1.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam1.destCIdx      = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam1.linkAddr      = 0xFFFFU;
    edmaParam1.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam1.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam1.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
         ((((uint32_t)tcc0) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(baseAddr, param0, &edmaParam1);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh1, tcc1, param1, EDMA_TEST_EVT_QUEUE_NO);

    /* Disable the interrupt for the channel to transfer in polled mode */
    EDMA_disableEvtIntrRegion(baseAddr, regionId, dmaCh1);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam2);
    edmaParam2.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr)+(EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT);
    edmaParam2.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr)+(EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT);
    edmaParam2.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam2.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam2.cCnt          = (uint16_t) EDMA_TEST_C_COUNT / 2;
    edmaParam2.bCntReload    = (uint16_t) 0;
    edmaParam2.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam2.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam2.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam2.destCIdx      = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam2.linkAddr      = 0xFFFFU;
    edmaParam2.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam2.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam2.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
         ((((uint32_t)tcc1) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(baseAddr, param1, &edmaParam2);

    /*Set params for chain transfer. Chain two channels*/
    EDMA_chainChannel(baseAddr, param0, dmaCh1, chainOptions);

    /*
     * Transfer is done in AB sync mode
     */
    EDMA_enableTransferRegion(baseAddr, regionId, dmaCh0,
            EDMA_TRIG_MODE_MANUAL);

    while(EDMA_readIntrStatusRegion(baseAddr, regionId, tcc1) != 1);

    EDMA_clrIntrRegion(baseAddr, regionId, tcc1);

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh0, EDMA_TRIG_MODE_MANUAL, tcc0, EDMA_TEST_EVT_QUEUE_NO);

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh1, EDMA_TRIG_MODE_EVENT, tcc1, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh0);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc0);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeParam(gEdmaHandle[0], &param0);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh1);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc1);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeParam(gEdmaHandle[0], &param1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /* Invalidate destination buffer and compare with src buffer */
    CacheP_inv((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    for(loopCnt = 0; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        if(srcBuffPtr[loopCnt] != dstBuffPtr[loopCnt])
        {
            testStatus = SystemP_FAILURE;
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
}

/*
 *   In this test, EDMA linking functionality is tested.
 */
static void test_edmaLinkTransfer(void *args)
{
    uint32_t            loopCnt = 0;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaCh, tcc, param0, param1;
    int32_t             testStatus = SystemP_SUCCESS;

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaCh = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    param0 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param0);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    param1 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /*
     * Initialize the source address with a pattern
     * initialize dst address with zero/another pattern (optional)
     */
    srcBuffPtr = (uint8_t *) gEdmaTestSrcBuff;
    dstBuffPtr = (uint8_t *) gEdmaTestDstBuff;
    for(loopCnt = 0U; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        srcBuffPtr[loopCnt] = (uint8_t)loopCnt;
        dstBuffPtr[loopCnt] = 0;
    }
    CacheP_wb((void *)srcBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    CacheP_wb((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, tcc, param0, EDMA_TEST_EVT_QUEUE_NO);

    /* Disable the interrupt for the channel to transfer in polled mode */
    EDMA_disableEvtIntrRegion(baseAddr, regionId, dmaCh);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam1);
    edmaParam1.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam1.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam1.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam1.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam1.cCnt          = (uint16_t) EDMA_TEST_C_COUNT/2;
    edmaParam1.bCntReload    = 0;
    edmaParam1.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam1.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam1.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam1.destCIdx      = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam1.linkAddr      = 0xFFFFU;
    edmaParam1.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam1.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam1.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |  EDMA_OPT_SYNCDIM_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_ccPaRAMEntry_init(&edmaParam2);
    edmaParam2.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr) + ( EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT );
    edmaParam2.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr) + ( EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT );
    edmaParam2.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam2.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam2.cCnt          = (uint16_t) EDMA_TEST_C_COUNT/2;
    edmaParam2.bCntReload    = 0;
    edmaParam2.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam2.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam2.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam2.destCIdx      = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam2.linkAddr      = 0xFFFFU;
    edmaParam2.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam2.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam2.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |  EDMA_OPT_SYNCDIM_MASK);

    EDMA_setPaRAM(baseAddr, param0, &edmaParam1);
    EDMA_setPaRAM(baseAddr, param1, &edmaParam2);

    EDMA_linkChannel(baseAddr, param0, param1);

    /*
     * Transfer is done in AB sync mode
     * Number of triggers required is C_COUNT
     */
    for(loopCnt = 0; loopCnt < (EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMA_enableTransferRegion(baseAddr, regionId, dmaCh,
             EDMA_TRIG_MODE_MANUAL);

        while(EDMA_readIntrStatusRegion(baseAddr, regionId, tcc) != 1);

        EDMA_clrIntrRegion(baseAddr, regionId, tcc);
    }

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeParam(gEdmaHandle[0], &param0);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeParam(gEdmaHandle[0], &param1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /* Invalidate destination buffer and compare with src buffer */
    CacheP_inv((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    for(loopCnt = 0; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        if(srcBuffPtr[loopCnt] != dstBuffPtr[loopCnt])
        {
            testStatus = SystemP_FAILURE;
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
}

/*
 *   In this test, QDMA transfer is verified in interrupt mode.
 */
static void test_qdmaTransfer(void *args)
{
    uint32_t            baseAddr, regionId;
    uint32_t            qdmaCh, tcc, param;
    int32_t             status = SystemP_SUCCESS;
    int32_t             testStatus = SystemP_SUCCESS;
    uint32_t            loopCnt = 0;
    Edma_IntrObject     intrObj;

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    qdmaCh = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocQdmaChannel(gEdmaHandle[0], &qdmaCh);
    DebugP_assert(status == SystemP_SUCCESS);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(status == SystemP_SUCCESS);

    param = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &param);
    DebugP_assert(status == SystemP_SUCCESS);

    /*
     * Initialize the source address with a pattern
     * initialize dst address with zero/another pattern (optional)
     */
    srcBuffPtr = (uint8_t *) gEdmaTestSrcBuff;
    dstBuffPtr = (uint8_t *) gEdmaTestDstBuff;
    for(loopCnt = 0U; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        srcBuffPtr[loopCnt] = (uint8_t)loopCnt;
        dstBuffPtr[loopCnt] = 0;
    }
    CacheP_wb((void *)srcBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    CacheP_wb((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_QDMA,
         qdmaCh, tcc, param, EDMA_TEST_EVT_QUEUE_NO);

    EDMA_mapQdmaChToPaRAM(baseAddr, qdmaCh, &param);

    EDMA_enableQdmaEvtRegion(baseAddr, regionId, qdmaCh);

    /* Set trig word for ccnt*/
    EDMA_setQdmaTrigWord(baseAddr, qdmaCh, EDMACC_PARAM_ENTRY_CCNT);

    status = SemaphoreP_constructBinary(&gEdmaTestDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register interrupt */
    intrObj.tccNum = tcc;
    intrObj.cbFxn  = &EDMA_regionIsrFxn;
    intrObj.appData = (void *) &gEdmaTestDoneSem;
    status = EDMA_registerIntr(gEdmaHandle[0], &intrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam1);
    edmaParam1.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam1.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam1.aCnt          = (uint16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT;
    edmaParam1.bCnt          = (uint16_t) 1;
    edmaParam1.cCnt          = (uint16_t) 1;
    edmaParam1.bCntReload    = (uint16_t) 1;
    edmaParam1.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT);
    edmaParam1.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT);
    edmaParam1.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT;
    edmaParam1.destCIdx      = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT;
    edmaParam1.linkAddr      = 0xFFFFU;
    edmaParam1.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT);
    edmaParam1.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT);
    edmaParam1.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_TPCC_OPT_STATIC_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_qdmaSetPaRAM(baseAddr, param, &edmaParam1);

    SemaphoreP_pend(&gEdmaTestDoneSem, SystemP_WAIT_FOREVER);

    /* Invalidate destination buffer and compare with src buffer */
    CacheP_inv((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    for(loopCnt = 0; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        if(srcBuffPtr[loopCnt] != dstBuffPtr[loopCnt])
        {
            testStatus = SystemP_FAILURE;
            break;
        }
    }

    status = EDMA_unregisterIntr(gEdmaHandle[0], &intrObj);
    SemaphoreP_destruct(&gEdmaTestDoneSem);

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_QDMA,
         qdmaCh, EDMA_TRIG_MODE_QDMA, tcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeDmaChannel(gEdmaHandle[0], &qdmaCh);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeParam(gEdmaHandle[0], &param);
    DebugP_assert(status == SystemP_SUCCESS);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
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

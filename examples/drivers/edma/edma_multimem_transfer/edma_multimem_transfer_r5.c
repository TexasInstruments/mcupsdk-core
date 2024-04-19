/*
 *  Copyright (C) 2023-24 Texas Instruments Incorporated
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

/**
 *  This example performs EDMA transfer test using interrupt mode.
 *
 * The source memory is initialized with sample data and destination memory
 * is initialized with zeroes for validation. Cache write back is done to
 * ensure that initialized data from cache is written to memory.
 *
 * The PaRAM set is initialized with proper configuration and assigned to a
 * channel for transfer. The transfer done is A Synchronized and so, BCNT*CCNT
 * triggers are needed for complete transfer.
 *
 * Intermediate and final transfer interrupts are enabled and the transfer
 * completion interrupt status is polled to be set before giving next trigger.
 *
 * After the transfer is completed, cache is invalidated to update cache with
 * the transferred data. Data validation is performed by comparing
 * source and destination memory. If the equality test is successful, the test
 * was successful.
 *
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/edma.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Value for A count*/
#define EDMA_TEST_A_COUNT           (256U)
/* Value for B count */
#define EDMA_TEST_B_COUNT           (4U)
/* Value for C count */
#define EDMA_TEST_C_COUNT           (1U)
/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO      (0U)
/**
 * Buffers (src and dest) are needed for mem-2-mem data transfers.
 * This define is for the MAXIMUM size and hence the maximum data
 * which could be transferred using the sample test cases below.
 */
#define EDMA_TEST_BUFFER_SIZE             (EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT)

/* The source buffer used for transfer */
static uint8_t gEdmaOcramSrcBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
/* The destination buffer used for transfer */
static uint8_t gEdmaOcramDstBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/* The source buffer used for transfer */
static uint8_t gEdmaTCMASrcBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT), section(".bss.tcm_a_mem")));
/* The destination buffer used for transfer */
static uint8_t gEdmaTCMADstBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT), section(".bss.tcm_a_mem")));

/* The source buffer used for transfer */
static uint8_t gEdmaTCMBSrcBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT), section(".bss.tcm_b_mem")));
/* The destination buffer used for transfer */
static uint8_t gEdmaTCMBDstBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT), section(".bss.tcm_b_mem")));

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gEdmaTestDoneSem;

uint32_t gEdmaCh, gEdmaTcc, gEdmaParam, gEdmaBaseAddr, gEdmaRegionId;
Edma_IntrObject     gEdmaIntrObj;

static void App_edmaConfig();
static void App_edmaDeConfig();
static void App_initializeBuffers(uint8_t *srcBuffPtr, uint8_t *dstBuffPtr);
static int32_t App_compareBuffers(uint8_t *srcBuffPtr, uint8_t *dstBuffPtr);
static void App_edmaTransfer(uint8_t *srcBuffPtr, uint8_t *dstBuffPtr);
static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args);

void edma_multimem_transfer(void *args)
{
    int32_t  status = SystemP_SUCCESS;
    uint64_t curTime;
    uint64_t ocmToOcm, tcmaToTcmA, tcmbToTcmB, ocmToTcma, tcmaToOcm;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[EDMA] Interrupt Transfer Test Started...\r\n");

    App_edmaConfig();

    curTime = ClockP_getTimeUsec();
    /* OCRAM to OCRAM transfer */
    App_edmaTransfer(gEdmaOcramSrcBuff, gEdmaOcramDstBuff);
    ocmToOcm = ClockP_getTimeUsec() - curTime;

    curTime = ClockP_getTimeUsec();
    /* TCMA to TCMA transfer */
    App_edmaTransfer(gEdmaTCMASrcBuff, gEdmaTCMADstBuff);
    tcmaToTcmA = ClockP_getTimeUsec() - curTime;

    curTime = ClockP_getTimeUsec();
    /* TCMB to TCMB transfer */
    App_edmaTransfer(gEdmaTCMBSrcBuff, gEdmaTCMBDstBuff);
    tcmbToTcmB = ClockP_getTimeUsec() - curTime;

    curTime = ClockP_getTimeUsec();
    /* OCRAM to TCMA transfer */
    App_edmaTransfer(gEdmaOcramSrcBuff, gEdmaTCMADstBuff);
    ocmToTcma = ClockP_getTimeUsec() - curTime;

    curTime = ClockP_getTimeUsec();
    /* TCMA to OCRAM transfer */
    App_edmaTransfer(gEdmaTCMADstBuff, gEdmaOcramSrcBuff);
    tcmaToOcm = ClockP_getTimeUsec() - curTime;

    DebugP_log("BENCHMARK START - EDMA - EDMA Memory Copy BENCHMARK \r\n");
    DebugP_log("\nEDMA Memory Copy Benchmark Numbers\r\n\n");
    DebugP_log("Size in Bytes | Source Memory | Destination Memory | Transfer time(us)\r\n");
    DebugP_log("--------------|---------------|--------------------|------------------\r\n");
    DebugP_log("    1024      |      OCRAM    |     OCRAM          |    %" PRId64 "   \r\n", ocmToOcm);
    DebugP_log("    1024      |      TCMA     |     TCMA           |    %" PRId64 "   \r\n", tcmaToTcmA);
    DebugP_log("    1024      |      TCMB     |     TCMB           |    %" PRId64 "   \r\n", tcmbToTcmB);
    DebugP_log("    1024      |      OCRAM    |     TCMA           |    %" PRId64 "   \r\n", ocmToTcma);
    DebugP_log("    1024      |      TCMA     |     OCRAM          |    %" PRId64 "   \r\n", tcmaToOcm);

    App_edmaDeConfig();

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("\nBENCHMARK END \r\n");
        DebugP_log("[EDMA] Interrupt Transfer Test Completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
    return;
}

static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);
    SemaphoreP_post(semObjPtr);
}

static void App_edmaConfig()
{
    int32_t    status = SystemP_SUCCESS;

    gEdmaBaseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(gEdmaBaseAddr != 0);

    gEdmaRegionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(gEdmaRegionId < SOC_EDMA_NUM_REGIONS);

    gEdmaCh = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &gEdmaCh);
    DebugP_assert(status == SystemP_SUCCESS);

    gEdmaTcc = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(gEdmaHandle[0], &gEdmaTcc);
    DebugP_assert(status == SystemP_SUCCESS);

    gEdmaParam = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &gEdmaParam);
    DebugP_assert(status == SystemP_SUCCESS);

    status = SemaphoreP_constructBinary(&gEdmaTestDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Request channel */
    EDMA_configureChannelRegion(gEdmaBaseAddr, gEdmaRegionId, EDMA_CHANNEL_TYPE_DMA,
         gEdmaCh, gEdmaTcc, gEdmaParam, EDMA_TEST_EVT_QUEUE_NO);

    /* Register interrupt */
    gEdmaIntrObj.tccNum = gEdmaTcc;
    gEdmaIntrObj.cbFxn  = &EDMA_regionIsrFxn;
    gEdmaIntrObj.appData = (void *) &gEdmaTestDoneSem;
    status = EDMA_registerIntr(gEdmaHandle[0], &gEdmaIntrObj);
    DebugP_assert(status == SystemP_SUCCESS);
}

static void App_edmaDeConfig()
{
    int32_t    status = SystemP_SUCCESS;

    status = EDMA_unregisterIntr(gEdmaHandle[0], &gEdmaIntrObj);
    SemaphoreP_destruct(&gEdmaTestDoneSem);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Free channel */
    EDMA_freeChannelRegion(gEdmaBaseAddr, gEdmaRegionId, EDMA_CHANNEL_TYPE_DMA,
        gEdmaCh, EDMA_TRIG_MODE_MANUAL, gEdmaTcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeDmaChannel(gEdmaHandle[0], &gEdmaCh);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeTcc(gEdmaHandle[0], &gEdmaTcc);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeParam(gEdmaHandle[0], &gEdmaParam);
    DebugP_assert(status == SystemP_SUCCESS);
}

static void App_edmaTransfer(uint8_t *srcBuffPtr, uint8_t *dstBuffPtr)
{
    EDMACCPaRAMEntry   edmaParam;
    int32_t    status = SystemP_SUCCESS;

    App_initializeBuffers(srcBuffPtr, dstBuffPtr);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam.cCnt          = (uint16_t) EDMA_TEST_C_COUNT;
    edmaParam.bCntReload    = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.destCIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
         ((((uint32_t)gEdmaTcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(gEdmaBaseAddr, gEdmaParam, &edmaParam);

    EDMA_enableTransferRegion(
        gEdmaBaseAddr, gEdmaRegionId, gEdmaCh, EDMA_TRIG_MODE_MANUAL);

    SemaphoreP_pend(&gEdmaTestDoneSem, SystemP_WAIT_FOREVER);

    /* Invalidate destination buffer and compare with src buffer */
    CacheP_inv((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    status = App_compareBuffers(srcBuffPtr, dstBuffPtr);
    DebugP_assert(status == SystemP_SUCCESS);
}

static int32_t App_compareBuffers(uint8_t *srcBuffPtr, uint8_t *dstBuffPtr)
{
    int32_t    status = SystemP_SUCCESS;
    uint32_t   loopCnt = 0;

    for(loopCnt = 0; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        if(srcBuffPtr[loopCnt] != dstBuffPtr[loopCnt])
        {
            DebugP_log("Error matching value at src and dst address %d\r\n", loopCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    return status;
}

static void App_initializeBuffers(uint8_t *srcBuffPtr, uint8_t *dstBuffPtr)
{
    uint32_t   loopCnt = 0;

    /*
     * Initialize the source address with a pattern
     * initialize dst address with zero/another pattern (optional)
     */
    for(loopCnt = 0U; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        srcBuffPtr[loopCnt] = (uint8_t)loopCnt;
        dstBuffPtr[loopCnt] = 0;
    }
    CacheP_wb((void *)srcBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    CacheP_wb((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
}

/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  This example performs EDMA transfer test using link mode.
 *
 * The link mode allows the entire PaRAM set to be reloaded from a location within the PaRAM memory map.
 * The PaRAM set is initialized with two channels and sets the Link field of
 * the PaRAM set associated with first channel to point  to the PaRAM set
 * associated with second channel.The transfer done in AB Synchronized and
 * so,CCNT triggers are needed to complete the transfer.
 *
 * The source memory is initialized with sample data and destination memory
 * is initialized with zeroes for validation. Cache write back is done to
 * ensure that initialized data from cache is written to memory.
 *
 * Upon completion of a transfer, the current transfer parameters are reloaded with the parameter set pointed that the 16-bit link address
 * field of the current parameter.
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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/edma.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Value for A count*/
#define EDMA_TEST_A_COUNT           (16U)
/* Value for B count */
#define EDMA_TEST_B_COUNT           (4U)
/* Value for C count */
#define EDMA_TEST_C_COUNT           (4U)
/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO      (0U)
/**
 * Buffers (src and dest) are needed for mem-2-mem data transfers.
 * This define is for the MAXIMUM size and hence the maximum data
 * which could be transferred using the sample test cases below.
 */
#define EDMA_TEST_BUFFER_SIZE             (EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT)

/* The source buffer used for transfer */
static uint8_t gEdmaTestSrcBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
/* The destination buffer used for transfer */
static uint8_t gEdmaTestDstBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

void edma_link_transfer(void *args)
{
    uint32_t            loopCnt = 0;
    uint32_t            baseAddr, regionId;
	 uint8_t            *srcBuffPtr, *dstBuffPtr;
    EDMACCPaRAMEntry   edmaParam1,edmaParam2;
    uint32_t            dmaCh, tcc, param0, param1;
    int32_t             testStatus = SystemP_SUCCESS;

	/* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[EDMA] Link Transfer Test Started...\r\n");

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
    edmaParam2.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr) + ( EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT / 2);
    edmaParam2.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr) + ( EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT / 2);
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
        /* Transfer is done in AB sync mode. Every enable transfer API call
         * will transfer aCnt * bCnt bymber of bytes and required cCnt (EDMA_TEST_C_COUNT/2)
         * number of triggers for param0.
         * At the end of transfer param1 will be loaded in param0 because of linking.
         * param1 is also configured in AB Sync mode and require EDMA_TEST_C_COUNT/2
         * number of triggers to complete transfer.
         */
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
            DebugP_log("Error matching value at src and dst address %d\r\n", loopCnt);
            testStatus = SystemP_FAILURE;
            break;
        }
    }

	if(testStatus == SystemP_SUCCESS)
    {
        DebugP_log("[EDMA] Link Transfer Test Completed!!\r\n");
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

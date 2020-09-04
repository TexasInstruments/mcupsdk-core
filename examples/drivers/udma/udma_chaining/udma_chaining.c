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
 */

/**
 *
 * UDMA chaining sample application performs a chain of block copy transfer
 * using channel global trigger.
 * Channel 0 completion triggers Channel 1 transfer: CH0 -> CH1.
 *
 * The application opens and configures two BCDMA channel using SysConfig.
 *
 * The first channel doesn't user a global trigger and each channel triggers
 * the next channel's global trigger through the channel's TR event register.
 *
 * A channel's source buffer is previous channel's destination buffer. This
 * ensures that chaining trigger works in a synchronized manner when the
 * memory compare matches.
 *
 * The transfer completion is based on last channel's DMA completion event.
 * Once the transfer it completes, it does cache operation for data coherency
 * and compares the source and destination buffers for any data mismatch.
 *
 */

#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Number of bytes to do memcpy */
#define UDMA_TEST_NUM_BYTES             (1024U)
/* UDMA TR packet descriptor memory size - with one TR */
#define UDMA_TEST_TRPD_SIZE             (UDMA_GET_TRPD_TR15_SIZE(1U))
/** \brief Number of channels */
#define UDMA_TEST_NUM_CH                (2U)

/* UDMA TRPD Memory */
uint8_t gUdmaTestTrpdMem[UDMA_TEST_NUM_CH][UDMA_TEST_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Application Buffers */
uint8_t gUdmaTestSrcBuf[UDMA_ALIGN_SIZE(UDMA_TEST_NUM_BYTES)] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
uint8_t gUdmaTestDestBuf[UDMA_TEST_NUM_CH][UDMA_ALIGN_SIZE(UDMA_TEST_NUM_BYTES)] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gUdmaTestDoneSem;

void App_udmaEventCb(Udma_EventHandle eventHandle, uint32_t eventType, void *appData);
static void App_udmaTrpdInit(Udma_ChHandle chHandle,
                             uint32_t chIdx,
                             uint8_t *trpdMem,
                             const void *destBuf,
                             const void *srcBuf,
                             uint32_t length);
static void App_udmaInitSrcBuf(uint8_t *srcBuf, uint32_t length);
static void App_udmaInitDestBuf(uint8_t *destBuf, uint32_t length);
static void App_udmaCompareBuf(uint8_t *srcBuf, uint8_t *destBuf, uint32_t length);

void *udma_chaining_main(void *args)
{
    int32_t         retVal = UDMA_SOK, status;
    Udma_ChHandle   chHandle0, chHandle1;
    uint32_t        length = UDMA_TEST_NUM_BYTES;
    uint64_t        pDesc;
    uint32_t        trRespStatus;
    uint8_t        *trpdMem;
    uint64_t        trpdMemPhy;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    chHandle0 = gConfigUdma0BlkCopyChHandle[0];  /* Has to be done after driver open */
    chHandle1 = gConfigUdma0BlkCopyChHandle[1];  /* Has to be done after driver open */
    DebugP_log("[UDMA] Chaining application started ...\r\n");

    status = SemaphoreP_constructBinary(&gUdmaTestDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Channel enable */
    retVal = Udma_chEnable(chHandle0);
    DebugP_assert(UDMA_SOK == retVal);
    retVal = Udma_chEnable(chHandle1);
    DebugP_assert(UDMA_SOK == retVal);

    /* Chain channels CH0 -> CH1 */
    retVal = Udma_chSetChaining(chHandle0, chHandle1, CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0);
    DebugP_assert(UDMA_SOK == retVal);

    /* Init buffers */
    App_udmaInitSrcBuf(&gUdmaTestSrcBuf[0U], length);
    App_udmaInitDestBuf(&gUdmaTestDestBuf[0U][0U], length);
    App_udmaInitDestBuf(&gUdmaTestDestBuf[1U][0U], length);

    /* Init TR packet descriptor */
    App_udmaTrpdInit(chHandle0, 0U, &gUdmaTestTrpdMem[0U][0U], &gUdmaTestDestBuf[0U][0U], &gUdmaTestSrcBuf[0U], length);
    /* Channel 1 source buffer is same as channel 0 destination buffer */
    App_udmaTrpdInit(chHandle1, 1U, &gUdmaTestTrpdMem[1U][0U], &gUdmaTestDestBuf[1U][0U], &gUdmaTestDestBuf[0U][0U], length);

    /* Submit TRPD to channel 0 */
    trpdMem = &gUdmaTestTrpdMem[0U][0U];
    trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(trpdMem, 0U, NULL);
    retVal = Udma_ringQueueRaw(Udma_chGetFqRingHandle(chHandle0), trpdMemPhy);
    DebugP_assert(UDMA_SOK == retVal);

    /* Submit TRPD to channel 1 */
    trpdMem = &gUdmaTestTrpdMem[1U][0U];
    trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(trpdMem, 0U, NULL);
    retVal = Udma_ringQueueRaw(Udma_chGetFqRingHandle(chHandle1), trpdMemPhy);
    DebugP_assert(UDMA_SOK == retVal);

    /* Wait for return descriptor in completion ring - this marks transfer completion */
    SemaphoreP_pend(&gUdmaTestDoneSem, SystemP_WAIT_FOREVER);

    /* Dequeue TRPD from channels */
    retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chHandle0), &pDesc);
    DebugP_assert(UDMA_SOK == retVal);
    retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chHandle1), &pDesc);
    DebugP_assert(UDMA_SOK == retVal);

    /* Check TR response status */
    trpdMem = &gUdmaTestTrpdMem[0U][0U];
    CacheP_inv(trpdMem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);
    trRespStatus = UdmaUtils_getTrpdTr15Response(trpdMem, 1U, 0U);
    DebugP_assert(CSL_UDMAP_TR_RESPONSE_STATUS_COMPLETE == trRespStatus);
    trpdMem = &gUdmaTestTrpdMem[1U][0U];
    CacheP_inv(trpdMem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);
    trRespStatus = UdmaUtils_getTrpdTr15Response(trpdMem, 1U, 0U);
    DebugP_assert(CSL_UDMAP_TR_RESPONSE_STATUS_COMPLETE == trRespStatus);

    /* Compare data */
    App_udmaCompareBuf(&gUdmaTestSrcBuf[0U], &gUdmaTestDestBuf[0U][0U], length);
    App_udmaCompareBuf(&gUdmaTestSrcBuf[0U], &gUdmaTestDestBuf[1U][0U], length);

    /* Channel disable */
    retVal = Udma_chDisable(chHandle0, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == retVal);
    retVal = Udma_chDisable(chHandle1, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == retVal);

    /* Break chain */
    retVal = Udma_chBreakChaining(chHandle0, chHandle1);
    DebugP_assert(UDMA_SOK == retVal);

    SemaphoreP_destruct(&gUdmaTestDoneSem);
    DebugP_log("All tests have passed!!\r\n");
    Board_driversClose();
    Drivers_close();

    return NULL;
}

void App_udmaEventCb(Udma_EventHandle eventHandle, uint32_t eventType, void *appData)
{
    if(UDMA_EVENT_TYPE_DMA_COMPLETION == eventType)
    {
        SemaphoreP_post(&gUdmaTestDoneSem);
    }
}

static void App_udmaTrpdInit(Udma_ChHandle chHandle,
                             uint32_t chIdx,
                             uint8_t *trpdMem,
                             const void *destBuf,
                             const void *srcBuf,
                             uint32_t length)
{
    CSL_UdmapTR15  *pTr;
    uint32_t        cqRingNum = Udma_chGetCqRingNum(chHandle);

    /* Make TRPD with TR15 TR type */
    UdmaUtils_makeTrpdTr15(trpdMem, 1U, cqRingNum);

    /* Setup TR */
    pTr = UdmaUtils_getTrpdTr15Pointer(trpdMem, 0U);
    pTr->flags    = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOL, CSL_UDMAP_TR_FLAGS_EOL_MATCH_SOL_EOL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION);
    if(0U == chIdx)
    {
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    }
    else
    {
        /* Set global trigger for channel 1 */
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0);
    }
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U);  /* This will come back in TR response */
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);
    pTr->icnt0    = length;
    pTr->icnt1    = 1U;
    pTr->icnt2    = 1U;
    pTr->icnt3    = 1U;
    pTr->dim1     = pTr->icnt0;
    pTr->dim2     = (pTr->icnt0 * pTr->icnt1);
    pTr->dim3     = (pTr->icnt0 * pTr->icnt1 * pTr->icnt2);
    pTr->addr     = (uint64_t) Udma_defaultVirtToPhyFxn(srcBuf, 0U, NULL);
    pTr->fmtflags = 0x00000000U;    /* Linear addressing, 1 byte per elem */
    pTr->dicnt0   = length;
    pTr->dicnt1   = 1U;
    pTr->dicnt2   = 1U;
    pTr->dicnt3   = 1U;
    pTr->ddim1    = pTr->dicnt0;
    pTr->ddim2    = (pTr->dicnt0 * pTr->dicnt1);
    pTr->ddim3    = (pTr->dicnt0 * pTr->dicnt1 * pTr->dicnt2);
    pTr->daddr    = (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL);

    /* Perform cache writeback */
    CacheP_wb(trpdMem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);

    return;
}

static void App_udmaInitSrcBuf(uint8_t *srcBuf, uint32_t length)
{
    uint32_t        i;

    for(i = 0U; i < length; i++)
    {
        srcBuf[i] = i;
    }
    /* Writeback buffer */
    CacheP_wb(srcBuf, length, CacheP_TYPE_ALLD);

    return;
}

static void App_udmaInitDestBuf(uint8_t *destBuf, uint32_t length)
{
    uint32_t        i;

    for(i = 0U; i < length; i++)
    {
        destBuf[i] = 0xA5U;
    }
    /* Writeback buffer */
    CacheP_wb(destBuf, length, CacheP_TYPE_ALLD);

    return;
}

static void App_udmaCompareBuf(uint8_t *srcBuf, uint8_t *destBuf, uint32_t length)
{
    uint32_t        i;

    /* Invalidate destination buffer */
    CacheP_inv(destBuf, length, CacheP_TYPE_ALLD);
    for(i = 0U; i < length; i++)
    {
        if(srcBuf[i] != destBuf[i])
        {
            DebugP_logError("Data mismatch !!!\r\n");
            DebugP_assert(FALSE);
        }
    }

    return;
}

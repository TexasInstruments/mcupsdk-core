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
 *  \brief UDMA SW trigger sample application performs 3D transfer using
 *  SW trigger method as below
 *
 *  Loop N times (icnt2)
 *      - SW trigger CH 0 -> Triggers MSMC to Intermediate buffer
 *      - Wait for CH 0 icnt0 x icnt1 to complete
 *      - SW trigger Channel 1 -> Triggers Intermediate buffer to MSMC
 *      - Wait for CH 1 icnt0 x icnt1 to complete
 *
 *  Each loop transfers M (icnt0 x icnt1) bytes of data
 *  MSMC size is M x N and Intermediate buffer size is just M bytes.
 *  Intermediate buffer memory set to wrap around after M bytes of transfer
 *
 *  Where,
 *      - M is icnt0 x icnt1 (UDMA_TEST_1D_SIZE x UDMA_TEST_2D_SIZE)
 *      - N is icnt2 = UDMA_TEST_3D_SIZE
 *
 * Once the transfer it completes, it does cache operation for data coherency
 * and compares the source and destination buffers for any data mismatch.
 */

#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/** \brief 1D Size */
#define UDMA_TEST_1D_SIZE               (32U)
/** \brief 2D Size */
#define UDMA_TEST_2D_SIZE               (4U)
/** \brief 3D Size */
#define UDMA_TEST_3D_SIZE               (10U)

/* Number of bytes to do memcpy */
#define UDMA_TEST_NUM_BYTES             (UDMA_TEST_1D_SIZE * UDMA_TEST_2D_SIZE * UDMA_TEST_3D_SIZE)
/* Number of bytes to do memcpy */
#define UDMA_TEST_NUM_BYTES_IND_BUF     (UDMA_TEST_1D_SIZE * UDMA_TEST_2D_SIZE)
/* UDMA TR packet descriptor memory size - with one TR */
#define UDMA_TEST_TRPD_SIZE             (UDMA_GET_TRPD_TR15_SIZE(1U))
/** \brief Number of channels */
#define UDMA_TEST_NUM_CH                (2U)

/* UDMA TRPD Memory */
uint8_t gUdmaTestTrpdMem[UDMA_TEST_NUM_CH][UDMA_TEST_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Application Buffers */
uint8_t gUdmaTestSrcBuf[UDMA_ALIGN_SIZE(UDMA_TEST_NUM_BYTES)] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
uint8_t gUdmaTestDestBuf[UDMA_ALIGN_SIZE(UDMA_TEST_NUM_BYTES)] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
uint8_t gUdmaTestIndBuf[UDMA_ALIGN_SIZE(UDMA_TEST_NUM_BYTES_IND_BUF)] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* TR Event parameters */
Udma_EventObject    gCh0TrEventObj, gCh1TrEventObj;
Udma_EventHandle    gCh0TrEventHandle, gCh1TrEventHandle;
Udma_EventPrms      gCh0TrEventPrms, gCh1TrEventPrms;

static void App_udmaTriggerInit(Udma_ChHandle ch0Handle, Udma_ChHandle ch1Handle);
static void App_udmaTriggerDeInit(Udma_ChHandle ch0Handle, Udma_ChHandle ch1Handle);
static void App_udmaTrpdInit(Udma_ChHandle chHandle,
                             uint32_t chIdx,
                             uint8_t *trpdMem,
                             const void *destBuf,
                             const void *srcBuf);
static void App_udmaInitSrcBuf(uint8_t *srcBuf, uint32_t length);
static void App_udmaInitDestBuf(uint8_t *destBuf, uint32_t length);
static void App_udmaCompareBuf(uint8_t *srcBuf, uint8_t *destBuf, uint32_t length);

void *udma_sw_trigger_main(void *args)
{
    int32_t         retVal = UDMA_SOK;
    Udma_ChHandle   ch0Handle, ch1Handle;
    uint64_t        pDesc;
    uint32_t        trRespStatus;
    uint8_t        *trpdMem;
    uint64_t        trpdMemPhy;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    ch0Handle = gConfigUdma0BlkCopyChHandle[0];  /* Has to be done after driver open */
    ch1Handle = gConfigUdma0BlkCopyChHandle[1];  /* Has to be done after driver open */
    DebugP_log("[UDMA] SW Trigger application started ...\r\n");

    App_udmaTriggerInit(ch0Handle, ch1Handle);

    /*
     * Processing loop
     */
    {
        volatile uint32_t  *ch0SwTriggerReg, *ch1SwTriggerReg;
        uint32_t            triggerMask, triggerCnt, tCnt;

        /* Set trigger parameters */
        triggerMask = ((uint32_t)1U << (CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0 - 1U));
        ch0SwTriggerReg = (volatile uint32_t *) Udma_chGetSwTriggerRegister(ch0Handle);
        ch1SwTriggerReg = (volatile uint32_t *) Udma_chGetSwTriggerRegister(ch1Handle);

        /* Submit TRPD to channel 0 */
        trpdMem = &gUdmaTestTrpdMem[0U][0U];
        trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(trpdMem, 0U, NULL);
        retVal = Udma_ringQueueRaw(Udma_chGetFqRingHandle(ch0Handle), trpdMemPhy);
        DebugP_assert(UDMA_SOK == retVal);

        /* Submit TRPD to channel 1 */
        trpdMem = &gUdmaTestTrpdMem[1U][0U];
        trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(trpdMem, 0U, NULL);
        retVal = Udma_ringQueueRaw(Udma_chGetFqRingHandle(ch1Handle), trpdMemPhy);
        DebugP_assert(UDMA_SOK == retVal);

        /* Set number of times to trigger based on event size */
        triggerCnt = UDMA_TEST_3D_SIZE;
        for(tCnt = 0U; tCnt < triggerCnt; tCnt++)
        {
            /* Set channel 0 trigger and wait for completion */
            CSL_REG32_WR(ch0SwTriggerReg, triggerMask);

            while(1U)
            {
                volatile uint64_t   intrStatusReg;
                intrStatusReg = CSL_REG64_RD(gCh0TrEventPrms.intrStatusReg);
                if(intrStatusReg & gCh0TrEventPrms.intrMask)
                {
                    /* Clear interrupt */
                    CSL_REG64_WR(gCh0TrEventPrms.intrClearReg, gCh0TrEventPrms.intrMask);
                    break;
                }
                TaskP_yield();
            }

            /* Set channel 1 trigger and wait for completion */
            CSL_REG32_WR(ch1SwTriggerReg, triggerMask);
            while(1U)
            {
                volatile uint64_t   intrStatusReg;
                intrStatusReg = CSL_REG64_RD(gCh1TrEventPrms.intrStatusReg);
                if(intrStatusReg & gCh1TrEventPrms.intrMask)
                {
                    /* Clear interrupt */
                    CSL_REG64_WR(gCh1TrEventPrms.intrClearReg, gCh1TrEventPrms.intrMask);
                    break;
                }
                TaskP_yield();
            }
        }

        /* Dequeue TRPD from channels */
        retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(ch0Handle), &pDesc);
        DebugP_assert(UDMA_SOK == retVal);
        retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(ch1Handle), &pDesc);
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
    }

    /* Compare data */
    App_udmaCompareBuf(&gUdmaTestSrcBuf[0U], &gUdmaTestDestBuf[0U], UDMA_TEST_NUM_BYTES);

    App_udmaTriggerDeInit(ch0Handle, ch1Handle);

    DebugP_log("All tests have passed!!\r\n");
    Board_driversClose();
    Drivers_close();

    return NULL;
}

static void App_udmaTriggerInit(Udma_ChHandle ch0Handle, Udma_ChHandle ch1Handle)
{
    int32_t         retVal;
    Udma_DrvHandle  drvHandle = &gUdmaDrvObj[CONFIG_UDMA0];

    /* Init buffers */
    App_udmaInitSrcBuf(&gUdmaTestSrcBuf[0U], UDMA_TEST_NUM_BYTES);
    App_udmaInitDestBuf(&gUdmaTestDestBuf[0U], UDMA_TEST_NUM_BYTES);
    App_udmaInitDestBuf(&gUdmaTestIndBuf[0U], UDMA_TEST_NUM_BYTES_IND_BUF);

    /* Init TR packet descriptor */
    App_udmaTrpdInit(ch0Handle, 0U, &gUdmaTestTrpdMem[0U][0U], &gUdmaTestIndBuf[0U], &gUdmaTestSrcBuf[0U]);
    /* Channel 1 source buffer is same as channel 0 destination buffer */
    App_udmaTrpdInit(ch1Handle, 1U, &gUdmaTestTrpdMem[1U][0U], &gUdmaTestDestBuf[0U], &gUdmaTestIndBuf[0U]);

    /* Register TR event - CH 0 */
    gCh0TrEventHandle = &gCh0TrEventObj;
    UdmaEventPrms_init(&gCh0TrEventPrms);
    gCh0TrEventPrms.eventType         = UDMA_EVENT_TYPE_TR;
    gCh0TrEventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    gCh0TrEventPrms.chHandle          = ch0Handle;
    /* For polling mode we can't use the existing master event as that is meant only for interrupt event -
     * we can't mix interrupt and poll mode in same master handle. Set the parameter to NULL
     * so that the driver creates a new master event. */
    gCh0TrEventPrms.masterEventHandle = NULL;
    gCh0TrEventPrms.eventCb           = NULL;
    gCh0TrEventPrms.appData           = NULL;
    retVal = Udma_eventRegister(drvHandle, gCh0TrEventHandle, &gCh0TrEventPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Register TR event - CH 1 */
    gCh1TrEventHandle = &gCh1TrEventObj;
    UdmaEventPrms_init(&gCh1TrEventPrms);
    gCh1TrEventPrms.eventType         = UDMA_EVENT_TYPE_TR;
    gCh1TrEventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    gCh1TrEventPrms.chHandle          = ch1Handle;
    /* Reuse previous event as master handle so that we share the same IA */
    gCh1TrEventPrms.masterEventHandle = gCh0TrEventHandle;
    gCh1TrEventPrms.eventCb           = NULL;
    gCh1TrEventPrms.appData           = NULL;
    retVal = Udma_eventRegister(drvHandle, gCh1TrEventHandle, &gCh1TrEventPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Channel enable */
    retVal = Udma_chEnable(ch0Handle);
    DebugP_assert(UDMA_SOK == retVal);
    retVal = Udma_chEnable(ch1Handle);
    DebugP_assert(UDMA_SOK == retVal);

    return;
}

static void App_udmaTriggerDeInit(Udma_ChHandle ch0Handle, Udma_ChHandle ch1Handle)
{
    int32_t         retVal;

    /* Channel disable */
    retVal = Udma_chDisable(ch0Handle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == retVal);
    retVal = Udma_chDisable(ch1Handle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == retVal);

    /* Unregister TR event - unregister master event at the end */
    retVal = Udma_eventUnRegister(gCh1TrEventHandle);
    DebugP_assert(UDMA_SOK == retVal);
    retVal = Udma_eventUnRegister(gCh0TrEventHandle);
    DebugP_assert(UDMA_SOK == retVal);

    return;
}

static void App_udmaTrpdInit(Udma_ChHandle chHandle,
                             uint32_t chIdx,
                             uint8_t *trpdMem,
                             const void *destBuf,
                             const void *srcBuf)
{
    CSL_UdmapTR15  *pTr;
    uint32_t        cqRingNum = Udma_chGetCqRingNum(chHandle);

    /* Make TRPD with TR15 TR type */
    UdmaUtils_makeTrpdTr15(trpdMem, 1U, cqRingNum);

    /* Setup TR */
    pTr = UdmaUtils_getTrpdTr15Pointer(trpdMem, 0U);
    pTr->flags    = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOL, CSL_UDMAP_TR_FLAGS_EOL_ICNT0_ICNT1);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT2_DEC);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT2_DEC);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U);  /* This will come back in TR response */
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);

    pTr->icnt0    = UDMA_TEST_1D_SIZE;
    pTr->icnt1    = UDMA_TEST_2D_SIZE;
    pTr->icnt2    = UDMA_TEST_3D_SIZE;
    pTr->icnt3    = 1U;
    pTr->addr     = (uint64_t) Udma_defaultVirtToPhyFxn(srcBuf, 0U, NULL);
    pTr->fmtflags = 0x00000000U;    /* Linear addressing, 1 byte per elem */

    pTr->dicnt0   = UDMA_TEST_1D_SIZE;
    pTr->dicnt1   = UDMA_TEST_2D_SIZE;
    pTr->dicnt2   = UDMA_TEST_3D_SIZE;
    pTr->dicnt3   = 1U;
    pTr->daddr    = (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL);

    if(0U == chIdx)
    {
        pTr->dim1     = pTr->icnt0;
        pTr->dim2     = (pTr->icnt0 * pTr->icnt1);
        pTr->dim3     = (pTr->icnt0 * pTr->icnt1 * pTr->icnt2);
        pTr->ddim1    = pTr->dicnt0;
        pTr->ddim2    = 0U;
        pTr->ddim3    = 0U;
    }
    else
    {
        pTr->dim1     = pTr->icnt0;
        pTr->dim2     = 0U;
        pTr->dim3     = 0U;
        pTr->ddim1    = pTr->dicnt0;
        pTr->ddim2    = (pTr->dicnt0 * pTr->dicnt1);
        pTr->ddim3    = (pTr->dicnt0 * pTr->dicnt1 * pTr->dicnt2);
    }

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

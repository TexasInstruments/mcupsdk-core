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

#include <string.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <drivers/soc.h>
#include <drivers/pcie.h>
#include <drivers/udma.h>

/******** Buffer *******/
#define BUF_SIZE 0x4000000u

uint8_t src_buf[BUF_SIZE]  __attribute__ ((section (".buffer"), aligned (0x4000000)));

static SemaphoreP_Object gBufTransDoneSem;

/* UDMA TR packet descriptor memory size - with one TR */
#define UDMA_TEST_TRPD_SIZE             (UDMA_GET_TRPD_TR15_SIZE(1U))

/* UDMA TRPD Memory */
uint8_t gUdmaTestTrpdMem[UDMA_TEST_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* NUmber of Iterations */
#define NUM_ITER  10

static void pcieLegacyISR (void *args)
{
    SemaphoreP_post(&gBufTransDoneSem);

    Pcie_rcLegacyIrqEoi (gPcieHandle[CONFIG_PCIE0]);
}

/* Initialize UDMA Trpd */
static void App_udmaTrpdInit(Udma_ChHandle chHandle,
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
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOL, CSL_UDMAP_TR_FLAGS_EOL_MATCH_SOL_EOL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U);  /* This will come back in TR response */
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);
    pTr->icnt0    = 8192;
    pTr->icnt1    = 8192;
    pTr->icnt2    = 1;
    pTr->icnt3    = 1;
    pTr->dim1     = 0;
    pTr->dim2     = 0;
    pTr->dim3     = 0;
    pTr->addr     = (uint64_t) Udma_defaultVirtToPhyFxn(srcBuf, 0U, NULL);
    pTr->fmtflags = 0x00000000U;    /* Linear addressing, 1 byte per elem */
    pTr->dicnt0   = 8192;
    pTr->dicnt1   = 8192;
    pTr->dicnt2   = 1;
    pTr->dicnt3   = 1;
    pTr->ddim1    = pTr->dicnt0;
    pTr->ddim2    = (pTr->dicnt0 * pTr->dicnt1);
    pTr->ddim3    = (pTr->dicnt0 * pTr->dicnt1 * pTr->dicnt2);
    pTr->daddr    = (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL);

    /* Perform cache writeback */
    CacheP_wb(trpdMem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);

    return;
}

static void Pcie_cpuBufTransfer (void)
{
    int32_t status = SystemP_SUCCESS;
    uint64_t timeDiff = 0U;
    uint32_t iter = 0;
    uint64_t cycleCount = 0;
    float result;

    timeDiff = 0;

    DebugP_log ("\nStarting PCIe Buffer transfer using CPU \r\n");

    for (iter = 0; iter < NUM_ITER; iter++)
    {
        cycleCount = ClockP_getTimeUsec();

        /* Initiate buffer transfer */
        {

            void *transBufAddr = (void *)(CONFIG_PCIE0_OB_REGION0_LOWER);

            memset(transBufAddr, iter, BUF_SIZE);

            CacheP_wbInv(transBufAddr, sizeof(uint8_t) * BUF_SIZE, CacheP_TYPE_ALL);

        }

        /* Wait for EP to interrupt RC using legacy interrupt */
        status = SemaphoreP_pend (&gBufTransDoneSem, SystemP_WAIT_FOREVER);

        cycleCount = ClockP_getTimeUsec() - cycleCount;

        timeDiff += cycleCount;
    }

    if (status != SystemP_SUCCESS)
    {
        DebugP_log("cpuBufTransfer fail\r\n");
    }

    DebugP_log ("Time taken for transfer of 640MB buffer using CPU copy --> %dus\r\n", timeDiff);

    result = (((float)NUM_ITER*64)/((float)timeDiff/1000000));

    DebugP_log ("Speed of PCIe transfer in MB/s using CPU copy --> %.3f MB/s\r\n", result);

    return;
}

static void Pcie_udmaBufTransfer (void)
{
    int32_t status = SystemP_SUCCESS;
    uint64_t timeDiff = 0U;
    uint32_t iter = 0;
    uint64_t cycleCount = 0;
    float result;

    Udma_ChHandle   chHandle;
    uint32_t        trRespStatus;
    uint64_t        pDesc;
    uint8_t        *trpdMem = &gUdmaTestTrpdMem[0U];
    uint64_t        trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(trpdMem, 0U, NULL);

    DebugP_log ("\nStarting PCIe Buffer transfer using UDMA \r\n");

    chHandle = gConfigUdma0BlkCopyChHandle[0];

    /* Channel enable */
    status = Udma_chEnable(chHandle);
    DebugP_assert(UDMA_SOK == status);

    App_udmaTrpdInit(chHandle, trpdMem, (void *)(CONFIG_PCIE0_OB_REGION0_LOWER), &src_buf);

    timeDiff = 0;

    for (iter = 0; iter < NUM_ITER; iter++)
    {
        memset (src_buf, iter, sizeof(src_buf));

        CacheP_wb (&src_buf, sizeof(src_buf), CacheP_TYPE_ALLD);

        cycleCount = ClockP_getTimeUsec();

        /* Submit TRPD to channel */
        status = Udma_ringQueueRaw(Udma_chGetFqRingHandle(chHandle), trpdMemPhy);
        DebugP_assert(UDMA_SOK == status);

        /* Wait for return descriptor in completion ring - this marks transfer completion */
        while(1)
        {
            status = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chHandle), &pDesc);
            if(UDMA_SOK == status)
            {
                /* Check TR response status */
                CacheP_inv(trpdMem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);
                trRespStatus = UdmaUtils_getTrpdTr15Response(trpdMem, 1U, 0U);
                DebugP_assert(CSL_UDMAP_TR_RESPONSE_STATUS_COMPLETE == trRespStatus);
                break;
            }
        }

        /* Wait for EP to interrupt RC using legacy interrupt */
        status = SemaphoreP_pend (&gBufTransDoneSem, SystemP_WAIT_FOREVER);

        cycleCount = ClockP_getTimeUsec() - cycleCount;

        timeDiff += cycleCount;
    }

    DebugP_log ("Time taken for transfer of 640MB buffer using UDMA --> %dus\r\n", timeDiff);

    result = (((float)NUM_ITER*64)/((float)timeDiff/1000000));

    DebugP_log ("Speed of PCIe transfer in MB/s using UDMA --> %.3f MB/s\r\n", result);

    /* Channel disable */
    status = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == status);

    return;
}

void pcie_benchmark_rc_main (void *args)
{
    int32_t status = SystemP_SUCCESS;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("Device in RC mode\r\n");

    /* Initialize Semaphore */
    status = SemaphoreP_constructBinary(&gBufTransDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable legacy interrupt and register ISR for legacy IR */
    {
        Pcie_legacyIrqRegisterParam irqParams;

        /* Enable legacy IRQ */
        status =  Pcie_rcLegacyIrqEnable (gPcieHandle[CONFIG_PCIE0], 1);

        if (status == SystemP_SUCCESS)
        {
            /* Register ISR for legacy interrupt */
            irqParams.intNum = 1;
            irqParams.funcPtr = pcieLegacyISR;
            status =  Pcie_rclegacyIrqIsrRegister (gPcieHandle[CONFIG_PCIE0], irqParams);
        }
    }

    /* Pcie buffer transfer using CPU */
    Pcie_cpuBufTransfer();

    /* Pcie buffer transfer using UDMA */
    Pcie_udmaBufTransfer();

    DebugP_log("\nPCIE benchmark example completed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();

    return;
}

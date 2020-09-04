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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <drivers/soc.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <string.h>

#define APP_OSPI_FLASH_DMA_REPEAT_CNT             (10U)
#define APP_OSPI_FLASH_DMA_TEST_SIZE_BYTES        (32U)
#define APP_OSPI_FLASH_DMA_BUF_SIZE               (APP_OSPI_FLASH_DMA_TEST_SIZE_BYTES)
#define APP_OSPI_FLASH_DMA_TRPD_SIZE              (UDMA_GET_TRPD_TR15_SIZE(1U))
#define APP_OSPI_FLASH_DMA_TR_INFINITE_RELOAD_CNT (0x1FFU)

#define APP_OSPI_FLASH_DMA_OFFSET (0x200000) /* 2MB offset in flash so that bootloader and any app flashed is not overwritten */

/* App Object */
typedef struct
{
    /* TR Event parameters */
    Udma_EventObject trEventObj;
    Udma_EventHandle trEventHandle;
    Udma_EventPrms trEventParams;

    /* Profiling parameters */
    uint64_t ticksDelay;
    uint64_t txTotalTicks;
    uint64_t rxTotalTicks[APP_OSPI_FLASH_DMA_REPEAT_CNT];

    /* Driver handles */
    Flash_Handle   flashHandle;
    OSPI_Handle    ospiHandle;
    Udma_DrvHandle udmaDrvHandle;
    Udma_ChHandle  udmaChHandle;

} App_OspiFlashDmaObj;

App_OspiFlashDmaObj gAppObj;

/* TRPD memory */
uint8_t gOspiFlashDmaTrpdMem[APP_OSPI_FLASH_DMA_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* App test buffers */
uint32_t gOspiFlashDmaTxBuf[APP_OSPI_FLASH_DMA_BUF_SIZE/sizeof(uint32_t)] __attribute__((aligned(128U)));
uint32_t gOspiFlashDmaRxBuf[APP_OSPI_FLASH_DMA_BUF_SIZE/sizeof(uint32_t)] __attribute__((aligned(128U)));

int32_t App_OspiFlashDmaInit(App_OspiFlashDmaObj *appObj);
int32_t App_OspiFlashDmaDeinit(App_OspiFlashDmaObj *appObj);
static inline uint64_t App_OspiFlashDmaGetTicksDiff(uint32_t stop, uint32_t start, uint32_t delay);
int32_t App_OspiFlashDmaTrEventRegister(App_OspiFlashDmaObj *appObj);
int32_t App_OspiFlashDmaTrEventUnregister(App_OspiFlashDmaObj *appObj);
void App_OspiFlashDmaTrpdInit(Udma_ChHandle chHandle, uint8_t* trpdMem, void *src, void *dst, uint32_t length);
void App_OspiFlashDmaFillBuffers(uint32_t lenBytes);
int32_t App_OspiFlashDmaCompareBuffers(uint32_t lenBytes);
void App_printPerformanceResults(App_OspiFlashDmaObj *appObj, uint32_t numBytes);

void ospi_flash_dma_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
    App_OspiFlashDmaObj *pAppObj = &gAppObj;
    uint32_t tCnt, triggerMask;
    volatile uint32_t * pSwTriggerReg;
    volatile uint64_t *intrStatusReg;
    volatile uint64_t *intrClearReg;
    volatile uint64_t intrStatusRegVal;
    volatile uint64_t intrMask;
    void* rxBuf = &gOspiFlashDmaRxBuf[0];

    /* Open OSPI and UDMA Drivers, among others */
    Drivers_open();

    /* Open Flash driver with OSPI instance as input */
    status = Board_driversOpen();
    DebugP_assert(status==SystemP_SUCCESS);

    DebugP_log("[OSPI] DMA low latency example started...\r\n");

    /* Initialize app object */
    App_OspiFlashDmaInit(pAppObj);

    /* Processing loop */
    pSwTriggerReg = (volatile uint32_t *) Udma_chGetSwTriggerRegister(pAppObj->udmaChHandle);
    triggerMask = ((uint32_t)1U << (CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0 - 1U));
    intrMask = pAppObj->trEventParams.intrMask;
    intrClearReg = pAppObj->trEventParams.intrClearReg;
    intrStatusReg = pAppObj->trEventParams.intrStatusReg;

    for(tCnt = 0; tCnt < APP_OSPI_FLASH_DMA_REPEAT_CNT; tCnt++)
    {
        uint32_t rxStartTicks, rxStopTicks;
        CycleCounterP_reset();
        rxStartTicks = CycleCounterP_getCount32();

        /* Set channel trigger and wait for completion */
        CSL_REG32_WR(pSwTriggerReg, triggerMask);

        while(1U)
        {
            intrStatusRegVal = CSL_REG64_RD(intrStatusReg);
            /* Check whether the interrupt status Reg is set - which indicates the
            * transfer completion of appTestObj->numBytes */
            if(intrStatusRegVal & intrMask)
            {
                /* Clear interrupt */
                CSL_REG64_WR(intrClearReg, intrMask);
                break;
            }
        }
        CacheP_inv(rxBuf, APP_OSPI_FLASH_DMA_TEST_SIZE_BYTES, CacheP_TYPE_ALLD);
        rxStopTicks = CycleCounterP_getCount32();

        pAppObj->rxTotalTicks[tCnt] = App_OspiFlashDmaGetTicksDiff(rxStopTicks, rxStartTicks, pAppObj->ticksDelay);

        /* Compare buffers */
        status = App_OspiFlashDmaCompareBuffers(APP_OSPI_FLASH_DMA_TEST_SIZE_BYTES);

        if(status != SystemP_SUCCESS)
        {
            break;
        }
    }

    /* Print performance stats */
    App_printPerformanceResults(pAppObj, APP_OSPI_FLASH_DMA_TEST_SIZE_BYTES);

    status += App_OspiFlashDmaDeinit(pAppObj);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("All tests have passed !!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed !!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

int32_t App_OspiFlashDmaInit(App_OspiFlashDmaObj *appObj)
{
    int32_t status = SystemP_SUCCESS;
    int32_t udmaStatus = UDMA_SOK;
    uint32_t offset, blk, page, ticksDelay;
    uint64_t trpdMemPhy;
    uint32_t flashSrcAddr;

    appObj->flashHandle         = gFlashHandle[CONFIG_FLASH0];
    appObj->ospiHandle          = OSPI_getHandle(CONFIG_OSPI0);
    appObj->udmaDrvHandle       = &gUdmaDrvObj[CONFIG_UDMA0];
    appObj->udmaChHandle        = gConfigUdma0BlkCopyChHandle[CONFIG_UDMA0];
    appObj->txTotalTicks        = 0U;
    memset(&appObj->rxTotalTicks, 0, APP_OSPI_FLASH_DMA_REPEAT_CNT * sizeof(uint32_t));

    /* Get ticks delay */
    CycleCounterP_reset();
    ticksDelay = CycleCounterP_getCount32();
    ticksDelay = CycleCounterP_getCount32() - ticksDelay;
    appObj->ticksDelay = ticksDelay;

    /* Register the TR Event */
    status = App_OspiFlashDmaTrEventRegister(appObj);
    DebugP_assert(status == SystemP_SUCCESS);

    /* UDMA Channel enable */
    udmaStatus = Udma_chEnable(appObj->udmaChHandle);
    DebugP_assert(UDMA_SOK == udmaStatus);

    /* Fill buffers with known data */
    App_OspiFlashDmaFillBuffers(APP_OSPI_FLASH_DMA_TEST_SIZE_BYTES);

    /* Doing the flash write in the init since this does not involve DMA.
       This part should be moved to the main loop when the flash write is
       also done via DMA
    */
    /* Block erase flash */
    offset = APP_OSPI_FLASH_DMA_OFFSET;
    Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], offset, &blk, &page);
    Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);

    uint32_t txStartTicks, txStopTicks;
    CycleCounterP_reset();
    txStartTicks = CycleCounterP_getCount32();
    Flash_write(gFlashHandle[CONFIG_FLASH0], offset, (void *)gOspiFlashDmaTxBuf, APP_OSPI_FLASH_DMA_TEST_SIZE_BYTES);
    txStopTicks = CycleCounterP_getCount32();

    appObj->txTotalTicks = App_OspiFlashDmaGetTicksDiff(txStopTicks, txStartTicks, appObj->ticksDelay);

    /* Enable the DAC mode so that the DMA can read directly from the flash */
    status = OSPI_enableDacMode(appObj->ospiHandle);

    /* Enable PHY */
    if((OSPI_isPhyEnable(appObj->ospiHandle) == TRUE) && (OSPI_getPhyEnableSuccess(appObj->ospiHandle) == TRUE))
    {
        OSPI_enablePhy(appObj->ospiHandle);
        /* We are using DMA separately, so no need to check if OSPI DMA mode is enabled */
        OSPI_enablePhyPipeline(appObj->ospiHandle);
    }

    /* Submit TRPD to channel */
    trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(gOspiFlashDmaTrpdMem, 0U, NULL);
    flashSrcAddr = OSPI_getFlashDataBaseAddr(appObj->ospiHandle) + APP_OSPI_FLASH_DMA_OFFSET;
    App_OspiFlashDmaTrpdInit(appObj->udmaChHandle,
                             &gOspiFlashDmaTrpdMem[0],
                             (void *)flashSrcAddr,
                             &gOspiFlashDmaRxBuf[0],
                             APP_OSPI_FLASH_DMA_TEST_SIZE_BYTES);

    udmaStatus = Udma_ringQueueRaw(Udma_chGetFqRingHandle(appObj->udmaChHandle), trpdMemPhy);
    DebugP_assert(udmaStatus==UDMA_SOK);

    if(udmaStatus == UDMA_SOK)
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t App_OspiFlashDmaDeinit(App_OspiFlashDmaObj *appObj)
{
    int32_t status = SystemP_SUCCESS;
    int32_t udmaStatus = UDMA_SOK;

    /* Disable PHY */
    if((OSPI_isPhyEnable(appObj->ospiHandle) == TRUE) && (OSPI_getPhyEnableSuccess(appObj->ospiHandle) == TRUE))
    {
        OSPI_disablePhy(appObj->ospiHandle);
        OSPI_disablePhyPipeline(appObj->ospiHandle);
    }

    /* Since TR Reload Count Set for perpetual loop, TRPD never completes and comes back to CQ.
     * To exit, teardown the channel using Udma_chDisable */
    udmaStatus = Udma_chDisable(appObj->udmaChHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(udmaStatus == UDMA_SOK);

    /* During channel forced teardown to break from the TR Reload Perpetual loop,
     * DMA will complete the already reloaded TR. This results in setting the
     * interrupt status register after this transfer completion.
     * Hence clear the interrupt */
    CSL_REG64_WR(appObj->trEventParams.intrClearReg, appObj->trEventParams.intrMask);

    /* Unregister the event */
    status = App_OspiFlashDmaTrEventUnregister(appObj);
    DebugP_assert(status == SystemP_SUCCESS);

    return status;
}

static inline uint64_t App_OspiFlashDmaGetTicksDiff(uint32_t stop, uint32_t start, uint32_t delay)
{
    uint32_t ticksDiff;

    if(stop > start)
    {
        ticksDiff = (uint64_t)(stop - start - delay);
    }
    else
    {
        /* Counter overflow, assume only one overflow has happened */
        ticksDiff = (uint64_t)((0xFFFFFFFFU - start) + stop - delay);
    }

    return ticksDiff;
}

int32_t App_OspiFlashDmaTrEventRegister(App_OspiFlashDmaObj *appObj)
{
    int32_t status = SystemP_SUCCESS;
    int32_t udmaStatus = UDMA_SOK;

    if(NULL != appObj)
    {
        Udma_EventHandle eventHandle = &appObj->trEventObj;
        UdmaEventPrms_init(&appObj->trEventParams);
        appObj->trEventParams.eventType = UDMA_EVENT_TYPE_TR;
        appObj->trEventParams.eventMode = UDMA_EVENT_MODE_SHARED;
        appObj->trEventParams.chHandle  = appObj->udmaChHandle;
        /* For polling mode we can't use the existing master event as that is meant only for interrupt event -
         *  we can't mix interrupt and poll mode in same master handle. Set the parameter to NULL
         *  so that the driver creates a new master event. */
        appObj->trEventParams.masterEventHandle = NULL;
        appObj->trEventParams.eventCb           = NULL;
        appObj->trEventParams.appData           = NULL; /* No callback */
        udmaStatus = Udma_eventRegister(appObj->udmaDrvHandle, eventHandle, &appObj->trEventParams);
        DebugP_assert(udmaStatus == UDMA_SOK);
        appObj->trEventHandle = eventHandle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t App_OspiFlashDmaTrEventUnregister(App_OspiFlashDmaObj *appObj)
{
    int32_t status = SystemP_SUCCESS;

    if(NULL != appObj)
    {
        int32_t udmaStatus = Udma_eventUnRegister(appObj->trEventHandle);
        DebugP_assert(udmaStatus == UDMA_SOK);
        appObj->trEventHandle = NULL;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

void App_OspiFlashDmaTrpdInit(Udma_ChHandle chHandle, uint8_t* trpdMem, void *src, void *dst, uint32_t length)
{
    CSL_UdmapCppi5TRPD *pTrpd = (CSL_UdmapCppi5TRPD *)trpdMem;
    CSL_UdmapTR15 *pTr;
    uint32_t cqRingNum = Udma_chGetCqRingNum(chHandle);

    /* Make TRPD */
    UdmaUtils_makeTrpdTr15(trpdMem, 1U, cqRingNum);
    CSL_udmapCppi5TrSetReload((CSL_UdmapCppi5TRPD*)pTrpd, APP_OSPI_FLASH_DMA_TR_INFINITE_RELOAD_CNT, 0U);

    /* Setup TR */
    pTr = UdmaUtils_getTrpdTr15Pointer(trpdMem, 0U);
    pTr->flags  = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_EOL, CSL_UDMAP_TR_FLAGS_EOL_ICNT0_ICNT1_ICNT2_ICNT3);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);

    pTr->icnt0    = length;
    pTr->icnt1    = 1;
    pTr->icnt2    = 1;
    pTr->icnt3    = 1;
    pTr->addr     = (uint64_t) Udma_defaultVirtToPhyFxn(src, 0U, NULL);
    pTr->fmtflags = 0x00000000U;        /* Linear addressing, 1 byte per elem. */

    pTr->dicnt0   = length;
    pTr->dicnt1   = 1;
    pTr->dicnt2   = 1;
    pTr->dicnt3   = 1;
    pTr->daddr    = (uint64_t) Udma_defaultVirtToPhyFxn(dst, 0U, NULL);

    pTr->dim1     = pTr->icnt0;
    pTr->dim2     = (pTr->icnt0 * pTr->icnt1);
    pTr->dim3     = (pTr->icnt0 * pTr->icnt1 * pTr->icnt2);
    pTr->ddim1    = pTr->dicnt0;
    pTr->ddim2    = (pTr->dicnt0 * pTr->dicnt1);
    pTr->ddim3    = (pTr->dicnt0 * pTr->dicnt1 * pTr->dicnt2);

    /* Perform cache writeback */
    CacheP_wb(trpdMem, APP_OSPI_FLASH_DMA_TRPD_SIZE, CacheP_TYPE_ALLD);
}

void App_OspiFlashDmaFillBuffers(uint32_t lenBytes)
{
    uint32_t i;
    uint32_t wordLength = lenBytes/sizeof(uint32_t);

    if(lenBytes % sizeof(uint32_t) != 0)
    {
        wordLength += 1;
    }

    for(i = 0U; i < wordLength; i++)
    {
        gOspiFlashDmaTxBuf[i] = i;
        gOspiFlashDmaRxBuf[i] = 0U;
    }
}

int32_t App_OspiFlashDmaCompareBuffers(uint32_t lenBytes)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t i;
    uint32_t wordLength = lenBytes/sizeof(uint32_t);

    if(lenBytes % sizeof(uint32_t) != 0)
    {
        wordLength += 1;
    }

    for(i = 0U; i < wordLength; i++)
    {
        if(gOspiFlashDmaTxBuf[i] != gOspiFlashDmaRxBuf[i])
        {
            status = SystemP_FAILURE;
            DebugP_logError("OSPI read data mismatch at index %d !!!\r\n", i);
            break;
        }
    }

    return status;
}

void App_printPerformanceResults(App_OspiFlashDmaObj *appObj, uint32_t numBytes)
{
    uint64_t cpuClockRate = 0U;
    uint32_t clkRateMHz = 0U;
    uint32_t txTicks, rxTicks, rxAvgTicks;

    cpuClockRate = SOC_getSelfCpuClk();
    clkRateMHz = cpuClockRate/1000000;

    txTicks = appObj->txTotalTicks;
    DebugP_log("OSPI Write %d bytes in %u ns\r\n", numBytes, txTicks*1000U/clkRateMHz);

    uint32_t i;
    rxAvgTicks = 0U;

    for(i = 0; i < APP_OSPI_FLASH_DMA_REPEAT_CNT; i++)
    {
        rxTicks = appObj->rxTotalTicks[i];
        DebugP_log("OSPI Read %d bytes in %u ns\r\n", numBytes, rxTicks*1000U/clkRateMHz);
        rxAvgTicks += rxTicks;
    }

    rxAvgTicks /= APP_OSPI_FLASH_DMA_REPEAT_CNT;

    DebugP_log("Average time for OSPI Read %d bytes in %u ns\r\n", numBytes, rxAvgTicks*1000U/clkRateMHz);
}

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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/edma.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example configures ePWMx and DMA as follows:
 *  - ePWMx is set up to generate PWM waveforms
 *  - DMA channel 0 is set up to update the CMPAHR, CMPA, CMPBHR and CMPB every
 *    period with the next value in the configuration array. This allows the
 *    user to create a DMA enabled fifo for all the CMPx and CMPxHR registers
 *    to generate unconventional PWM waveforms.
 *  - DMA channel 1 is set up to update the TBPHSHR, TBPHS, TBPRDHR and TBPRD
 *    every period with the next value in the configuration array.
 *  - Other registers such as AQCTL can be controlled through the DMA as well
 *    by following the same procedure. (Not used in this example)
 */

/* DMA channel number to transfer ADC0 and ADC1 conversion results*/
#define EPWM_EDMA_CHANNEL_A       (DMA_TRIG_XBAR_EDMA_MODULE_0)
#define EPWM_EDMA_CHANNEL_B       (DMA_TRIG_XBAR_EDMA_MODULE_1)

/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO      (0U)

/* 4 words per transfer */
#define BURST_SIZE       4
/* 4 transfers (different configs) */
#define TRANSFER_SIZE    4

/* Global variables and objects */
static HwiP_Object  gEpwmHwiObject;
uint32_t gEpwmBaseAddr;

uint16_t gPhasePeriodConfigs[TRANSFER_SIZE*BURST_SIZE] = {
/*  TBPHSHR ,   TBPHS   ,  TBPRDHR ,   TBPRD */
    9  << 8 ,    17U    ,  13 << 8 ,   2000U,
    10 << 8 ,    18U    ,  14 << 8 ,   4000U,
    11 << 8 ,    19U    ,  15 << 8 ,   6000U,
    12 << 8 ,    20U    ,  16 << 8 ,   8000U,
};

uint16_t gCompareConfigs[TRANSFER_SIZE*BURST_SIZE] = {
/*  CMPAHR  ,   CMPA   ,   CMPBHR  ,   CMPB  */
    1 << 8  ,  1001U   ,   5 << 8  ,   1000U,
    2 << 8  ,  2001U   ,   6 << 8  ,   2000U,
    3 << 8  ,  3001U   ,   7 << 8  ,   3000U,
    4 << 8  ,  4001U   ,   8 << 8  ,   4000U,
};

uint16_t App_dmaConfigure(
        const uint16_t *table, uint16_t table_size,
        EDMA_Handle dma_handle, uint32_t dma_ch,
        uint32_t epwm_transfer_addr);

static void App_epwmIntrISR(void *handle);

void epwm_dma_main(void *args)
{
    int32_t  status;
    HwiP_Params  hwiPrms;
    uint16_t i=10;

    gEpwmBaseAddr = CONFIG_EPWMx_BASE_ADDR;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    EPWM_clearADCTriggerFlag(gEpwmBaseAddr, EPWM_SOC_A);
    DebugP_log("EPWM DMA Test Started ...\r\n");

    /* For EPWM 1 */

    /* Register & enable interrupt */
     HwiP_Params_init(&hwiPrms);
    /* Integrate with Syscfg */
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    //hwiPrms.isPulse     = APP_INT_IS_PULSE;
    hwiPrms.callback    = &App_epwmIntrISR;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);

    /* Configure DMA channels to transfer both tables */
    App_dmaConfigure(gPhasePeriodConfigs, TRANSFER_SIZE, gEdmaHandle[0],
                EPWM_EDMA_CHANNEL_A, gEpwmBaseAddr + CSL_EPWM_TBPHS);
    App_dmaConfigure(gCompareConfigs, TRANSFER_SIZE, gEdmaHandle[0],
                EPWM_EDMA_CHANNEL_B, gEpwmBaseAddr + CSL_EPWM_CMPA);

    while(i--);

    DebugP_log("EPWM DMA Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}


uint16_t App_dmaConfigure(
        const uint16_t *table, uint16_t table_size,
        EDMA_Handle dma_handle, uint32_t dma_ch,
        uint32_t epwm_transfer_addr)
{

    uint32_t            baseAddr, regionId;
    EDMACCPaRAMEntry   edmaParam1,edmaParam2;
    uint32_t            dmaCh, tcc, param0, param1;
    int32_t             testStatus = SystemP_SUCCESS;

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaCh = dma_ch;
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

    CacheP_wb((void *)table, table_size*2, CacheP_TYPE_ALL);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, tcc, param0, EDMA_TEST_EVT_QUEUE_NO);

    /* Disable the interrupt for the channel to transfer in polled mode */
    EDMA_disableEvtIntrRegion(baseAddr, regionId, dmaCh);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam1);
    edmaParam1.srcAddr       = (uint32_t) SOC_virtToPhy((void *)table);
    edmaParam1.destAddr      = (uint32_t) SOC_virtToPhy((void *)(epwm_transfer_addr));
    edmaParam1.aCnt          = (uint16_t) BURST_SIZE * sizeof(uint16_t);
    edmaParam1.bCnt          = (uint16_t) TRANSFER_SIZE;
    edmaParam1.cCnt          = (uint16_t) 1;
    edmaParam1.bCntReload    = 0;
    edmaParam1.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(BURST_SIZE * sizeof(uint16_t));
    edmaParam1.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
    edmaParam1.srcCIdx       = (int16_t) 0;
    edmaParam1.destCIdx      = (int16_t) 0;
    edmaParam1.linkAddr      = 0xFFFFU;
    edmaParam1.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(BURST_SIZE * sizeof(uint16_t));
    edmaParam1.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaParam1.opt           = (((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_ccPaRAMEntry_init(&edmaParam2);
    edmaParam2.srcAddr       = (uint32_t) SOC_virtToPhy((void *)table);
    edmaParam2.destAddr      = (uint32_t) SOC_virtToPhy((void *)(epwm_transfer_addr));
    edmaParam2.aCnt          = (uint16_t) BURST_SIZE * sizeof(uint16_t);
    edmaParam2.bCnt          = (uint16_t) TRANSFER_SIZE;
    edmaParam2.cCnt          = (uint16_t) 1;
    edmaParam2.bCntReload    = 0;
    edmaParam2.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(BURST_SIZE * sizeof(uint16_t));
    edmaParam2.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
    edmaParam2.srcCIdx       = (int16_t) 0;
    edmaParam2.destCIdx      = (int16_t) 0;
    edmaParam2.linkAddr      = 0xFFFFU;
    edmaParam2.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(BURST_SIZE * sizeof(uint16_t));
    edmaParam2.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaParam2.opt           = (((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(baseAddr, param0, &edmaParam1);
    EDMA_setPaRAM(baseAddr, param1, &edmaParam2);

    EDMA_linkChannel(baseAddr, param0, param1);
    EDMA_linkChannel(baseAddr, param1, param1);

    EDMA_enableTransferRegion(baseAddr, regionId, dmaCh,
                              EDMA_TRIG_MODE_EVENT);

    return testStatus;
}

static void App_epwmIntrISR(void *handle)
{
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);
    return;
}
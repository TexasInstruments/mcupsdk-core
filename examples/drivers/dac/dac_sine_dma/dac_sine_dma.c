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
#include <kernel/dpl/ClockP.h>
#include <drivers/edma.h>
#include <drivers/dac.h>
#include <math.h>
#include "ti_drivers_config.h"
#include "ti_dpl_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example uses the DAC module to generate a sine wave on
 * the DAC output using DMA.
 *
 * Sine values for 0 to 360 degree are stored in a buffer of 360 elements.
 * These values are copied to DAC continuously using DMA with Timer as
 * trigger. This creates continuous sine waves as DAC output.
 *
 * The frequency of sine wave depends on the Timer frequency.
 *
 * The sine wave produced can be viewed on the DAC output pin using
 * an oscilloscope,
 */

/* Number of elements in the buffer */
#define SINE_TABLE_SIZE  360
/* Value of PI */
#define PI               3.14159265
/* Duration for which sine wave is produced through DAC */
#define SINE_OUTPUT_TIME         10
/* EDMA event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO  (0U)

/* Global variables and objects */
/* Buffer to store DAC values */
uint16_t gSineTable[SINE_TABLE_SIZE];
/* Used to adjust the magnitude of the waveform. (Range 0.0 -> 1.0) */
float gWaveformGain = 0.8003;
/* Used to adjust the offset of the waveform. (Range -1.0 -> 1.0) */
float gWaveformOffset = 0;

uint16_t App_dmaConfigure( const uint16_t *table, uint16_t table_num,
        EDMA_Handle dma_handle, uint32_t dma_ch,
        uint32_t dac_base);
void App_configureWaveform(void);

void dac_sine_dma_main(void *args)
{

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    

    DebugP_log("DAC Sine DMA  Test Started ...\r\n");

    App_configureWaveform();

    App_dmaConfigure(gSineTable,SINE_TABLE_SIZE,gEdmaHandle[0],
                    DMA_TRIG_XBAR_EDMA_MODULE_0,CONFIG_DAC0_BASE_ADDR);

    /* Start the timer to get interrupt at a rate of 20KHz*/
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

    /*
     * Wait while TIMER causes DMA triggers. DAC is updated and sine waves are
     * produced until timeout occurs.
     */
    ClockP_sleep(SINE_OUTPUT_TIME);

    /* Stop the timer to halt generation of sine waves */
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

    DebugP_log("DAC Sine DMA Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

uint16_t App_dmaConfigure(
        const uint16_t *table, uint16_t table_size,
        EDMA_Handle dma_handle, uint32_t dma_ch,
        uint32_t dac_base)
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
    edmaParam1.destAddr      = (uint32_t) SOC_virtToPhy((void *)(dac_base+CSL_DAC_DACVALS));
    edmaParam1.aCnt          = (uint16_t) 2;
    edmaParam1.bCnt          = (uint16_t) table_size;
    edmaParam1.cCnt          = (uint16_t) 1;
    edmaParam1.bCntReload    = 0;
    edmaParam1.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(2);
    edmaParam1.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
    edmaParam1.srcCIdx       = (int16_t) 0;
    edmaParam1.destCIdx      = (int16_t) 0;
    edmaParam1.linkAddr      = 0xFFFFU;
    edmaParam1.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(2);
    edmaParam1.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaParam1.opt           = (((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_ccPaRAMEntry_init(&edmaParam2);
    edmaParam2.srcAddr       = (uint32_t) SOC_virtToPhy((void *)table);
    edmaParam2.destAddr      = (uint32_t) SOC_virtToPhy((void *)(dac_base+CSL_DAC_DACVALS));
    edmaParam2.aCnt          = (uint16_t) 2;
    edmaParam2.bCnt          = (uint16_t) table_size;
    edmaParam2.cCnt          = (uint16_t) 1;
    edmaParam2.bCntReload    = 0;
    edmaParam2.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(2);
    edmaParam2.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
    edmaParam2.srcCIdx       = (int16_t) 0;
    edmaParam2.destCIdx      = (int16_t) 0;
    edmaParam2.linkAddr      = 0xFFFFU;
    edmaParam2.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(2);
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

void App_configureWaveform(void)
{
    uint16_t j;
    float offset;
    float waveformValue;

    /* Fill Sine Table */
    for(j=0;j<SINE_TABLE_SIZE;j++)
    {
        gSineTable[j] = (sin(j*PI/180.0)+1.0) * 2047.5;
    }

    /* Adjust for Gain and Offset */
    offset = (gSineTable[0] - (gSineTable[0]*gWaveformGain)) + (gSineTable[0]*gWaveformOffset);

    for(j=0;j<SINE_TABLE_SIZE;j++)
    {
        waveformValue = (gSineTable[j]*gWaveformGain)+offset;

        /* Validate that the value is within the DAC input range  */
        gSineTable[j] = waveformValue < 0 ? 0 : waveformValue > 4095 ? 4095 : waveformValue;
    }
}
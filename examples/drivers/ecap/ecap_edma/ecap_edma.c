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

//
// Included Files
//
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/**
 * Example Description:
 *  This example demostrates the ECAP triggerring EDMA for transfer of the ECAP
 * timestamp registers to the TCM location without R5 intervention.
 *
 *  ECAP can generate EDMA triggers for Data transfer in both CAPTURE mode and
 * APWM mode
 * - In ECAP Capture Mode, the CAPEVTx [x= 1 to 4] can be used and
 * - In ECAP APWM Mode, The counter match to Compare or Period can be used.
 *
 *  An ECAP (APWM_ECAP) is configured in APWM mode to generate trigger at
 * its counter matching period event to transfer timestamp data from "CAPTURE_ECAP2".
 * 2 ECAPs (CAPTURE_ECAP1, CAPTURE_ECAP2) are configured to capture the APMW_ECAP PWM
 * wave. CAPTURE_ECAP1 also triggers EDMA on one of its Capture events.
 *
 * ECAP Configurations:
 * 1. APWM_ECAP:
 * - APWM mode, 5KHz and 25% duty cycle
 * - DMA trigger at Counter = Period event for channel 0
 * - Output routed via Outputxbar13 (GPIO119)
 *
 * 2. CAPTURE_ECAP1:
 * - CAPTURE MODE, capturing Falling, Rising, Falling, Rising edges on its four events,
 * - counter resets at CAPEVT1,2,3,4. Thereby, recording
 *      - ON time at CAP1, CAP3,
 *      - OFF times at CAP2,CAP4,
 * - CAPEVT4 is the source for the DMA channel 1 trigger for the transfer of its timestamps in CAPx
 *
 * 3. CAPTURE_ECAP2:
 * - CAPTURE mode, Capturing Falling, Rising edges on its 2 events,
 * - counter resets on CAPEVT1,2. thereby recording,
 *      - ON time on CAP1
 *      - OFF time on CAP2
 *
 *  A total of 256 transfers on Channel 0 and 128 transfers on Channel 1 are triggered,
 * and the buffers "capTrigTimeStamps" and "pwmTrigTimeStamps" are compared.
 *
 * Watch Variables :
 * 1. capTrigTimeStamps [ ]: array with CAPTURE_ECAP1 Captured timestamps, triggered by Capture mode.
 * 2. pwmTrigTimeStamps [ ]: array with CAPTURE_ECAP2 Captured timestamps, triggered by APWM mode.
 *
 * External Connections : No external connections are required.
 *
 */

#define APP_NUM_EDMA_PARAMS (2)
#define APP_NUM_EDMA_CHANNELS (2)

#define APP_NUM_APWM_TRIG_TRANSFERS (256)   // Each Transfer is of 2 regs
#define APP_NUM_APWM_TRIG_REG_TRANSFER (2)

#define APP_NUM_CAP_TRIG_TRANSFERS  (128)   // Each Transfer is of 4 regs
#define APP_NUM_CAP_TRIG_REG_TRANSFER (4)

#define APP_EDMA_A_COUNT (4)    // 4 bytes, 1 in each register

#define EDMA_TEST_EVT_QUEUE_NO_0 (0)    // tptc 0

uint32_t gEdmaPaRams[APP_NUM_EDMA_PARAMS];
uint32_t gEdmaChannels[APP_NUM_EDMA_CHANNELS] = {
    DMA_TRIG_XBAR_EDMA_MODULE_0,
    DMA_TRIG_XBAR_EDMA_MODULE_1,
};

uint32_t gEdmaBase, gEdmaRegId, gEdmaTcc[APP_NUM_EDMA_CHANNELS];

uint32_t gApwmEcapBase = APWM_ECAP0_BASE_ADDR;
uint32_t gCap1EcapBase = CAPTURE_ECAP1_BASE_ADDR;
uint32_t gCap2EcapBase = CAPTURE_ECAP2_BASE_ADDR;

/* (APP_NUM_CAP_TRIG_TRANSFERS * APP_NUM_CAP_TRIG_REG_TRANSFER)
    or
   (APP_NUM_APWM_TRIG_TRANSFERS * APP_NUM_APWM_TRIG_REG_TRANSFER) */
#define ARRAY_SIZE (APP_NUM_CAP_TRIG_TRANSFERS * APP_NUM_CAP_TRIG_REG_TRANSFER)

volatile uint32_t capTrigTimeStamps[ARRAY_SIZE] = {0};
volatile uint32_t pwmTrigTimeStamps[ARRAY_SIZE] = {0};

void App_edmaConfig(void);

void App_edmaDeConfig(void);

bool App_compareInRange(uint32_t input1, uint32_t input2, uint32_t error_range);

void ecap_edma_main(void *args)
{

    bool status = true;
    Drivers_open();
    Board_driversOpen();


    DebugP_log("ECAP triggered EDMA transfers Test Started ...\r\n");

    /* stopping counters for the other configurations */
    ECAP_stopCounter(gApwmEcapBase);
    ECAP_stopCounter(gCap1EcapBase);
    ECAP_stopCounter(gCap2EcapBase);

    /* configuring the edma transfers */
    App_edmaConfig();
    /* clearing the interrupts for the edma channel transfer */
    EDMA_clrIntrRegion(gEdmaBase, gEdmaRegId, gEdmaTcc[0]);
    EDMA_clrIntrRegion(gEdmaBase, gEdmaRegId, gEdmaTcc[1]);

    /* starting the ECAP counters for the APWM generation and the Captures */
    ECAP_startCounter(gApwmEcapBase);
    ECAP_startCounter(gCap1EcapBase);
    ECAP_startCounter(gCap2EcapBase);

    /* enabling the edma transfers */
    EDMA_enableTransferRegion(gEdmaBase, gEdmaRegId, gEdmaChannels[0], EDMA_TRIG_MODE_EVENT);
    EDMA_enableTransferRegion(gEdmaBase, gEdmaRegId, gEdmaChannels[1], EDMA_TRIG_MODE_EVENT);


    /* wait while the transfers are complete */
    for(;;)
    {
        if(
            (EDMA_readIntrStatusRegion(gEdmaBase, gEdmaRegId, gEdmaTcc[0]) == 1) &&
            (EDMA_readIntrStatusRegion(gEdmaBase, gEdmaRegId, gEdmaTcc[1]) == 1)
        )
        {
            break;
        }
    }
    /* transfers complete. stopping the counters */
    ECAP_stopCounter(gApwmEcapBase);
    ECAP_stopCounter(gCap1EcapBase);
    ECAP_stopCounter(gCap2EcapBase);

    /* clearing the EDMA interrupts */
    EDMA_clrIntrRegion(gEdmaBase, gEdmaRegId, gEdmaTcc[0]);
    EDMA_clrIntrRegion(gEdmaBase, gEdmaRegId, gEdmaTcc[1]);

    DebugP_log("EDMA transfers complete.\r\n");

    /* comparing the copied Timestamp registers */
    for(int iter = 0; iter <ARRAY_SIZE; iter++)
    {
        uint32_t error_range = 5;
        status = App_compareInRange(capTrigTimeStamps[iter], pwmTrigTimeStamps[iter], error_range);
        if(status == false)
        {
            DebugP_log("Fail : Copied registers are not in sync | index : %d : cap1 : %d\t cap2 : %d\r\n", iter, capTrigTimeStamps[iter], pwmTrigTimeStamps[iter]);
            status = false;
            break;
        }
        if((capTrigTimeStamps[iter] == 0) || (pwmTrigTimeStamps[iter] == 0))
        {
            DebugP_log("DMA transfer failed\r\n");
            status = false;
            break;
        }
    }
    if(status == true)
    {
        DebugP_log("ECAP triggered EDMA transfers Passed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("ECAP triggered EDMA transfers Failed!!\r\n");
        DebugP_log("Some tests have Failed!!\r\n");
    }

    /* deconfiguring the EDMA */
    App_edmaDeConfig();

    Board_driversClose();
    Drivers_close();
}

void App_edmaConfig(void)
{
    int32_t status = SystemP_SUCCESS;
    EDMACCPaRAMEntry edma_paramEntry;

    gEdmaBase = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert( gEdmaBase != 0);

    gEdmaRegId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(gEdmaRegId < SOC_EDMA_NUM_REGIONS);

    for(int channel = 0; channel < APP_NUM_EDMA_CHANNELS; channel++)
    {
        status = EDMA_allocDmaChannel(gEdmaHandle[0], &(gEdmaChannels[channel]));
        DebugP_assert(status == SystemP_SUCCESS);
    }

    for(int param = 0; param < APP_NUM_EDMA_PARAMS; param++)
    {
        gEdmaPaRams[param] = EDMA_RESOURCE_ALLOC_ANY;
        status = EDMA_allocParam(gEdmaHandle[0], &(gEdmaPaRams[param]));
        DebugP_assert(status == SystemP_SUCCESS);
    }

    for(int tcc = 0; tcc < APP_NUM_EDMA_PARAMS; tcc++)
    {
        gEdmaTcc[tcc] = EDMA_RESOURCE_ALLOC_ANY;
        status = EDMA_allocTcc(gEdmaHandle[0], &gEdmaTcc[tcc]);
        DebugP_assert(status == SystemP_SUCCESS);
    }
    /* param set population for the cap triggered transfers */
    EDMA_ccPaRAMEntry_init(&edma_paramEntry);
    edma_paramEntry.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(gCap2EcapBase + CSL_ECAP_CAP1));
    edma_paramEntry.destAddr      = (uint32_t) SOC_virtToPhy((void *) pwmTrigTimeStamps);
    edma_paramEntry.aCnt          = (uint16_t) (APP_EDMA_A_COUNT);
    edma_paramEntry.bCnt          = (uint16_t) (APP_NUM_APWM_TRIG_REG_TRANSFER);
    edma_paramEntry.cCnt          = (uint16_t) (APP_NUM_APWM_TRIG_TRANSFERS);
    edma_paramEntry.bCntReload    = 0;
    edma_paramEntry.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(APP_EDMA_A_COUNT);
    edma_paramEntry.destBIdx      = (int16_t) EDMA_PARAM_BIDX(APP_EDMA_A_COUNT);
    edma_paramEntry.srcCIdx       = (int16_t) 0;
    edma_paramEntry.destCIdx      = (int16_t) APP_EDMA_A_COUNT*APP_NUM_APWM_TRIG_REG_TRANSFER;
    edma_paramEntry.linkAddr      = 0xFFFFU;
    edma_paramEntry.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edma_paramEntry.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edma_paramEntry.opt           = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
                              ((((uint32_t)(gEdmaTcc[0])) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(gEdmaBase, gEdmaPaRams[0], &edma_paramEntry);

    EDMA_ccPaRAMEntry_init(&edma_paramEntry);
    edma_paramEntry.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(gCap1EcapBase + CSL_ECAP_CAP1));
    edma_paramEntry.destAddr      = (uint32_t) SOC_virtToPhy((void *) capTrigTimeStamps);
    edma_paramEntry.aCnt          = (uint16_t) (APP_EDMA_A_COUNT);
    edma_paramEntry.bCnt          = (uint16_t) (APP_NUM_CAP_TRIG_REG_TRANSFER);
    edma_paramEntry.cCnt          = (uint16_t) (APP_NUM_CAP_TRIG_TRANSFERS);
    edma_paramEntry.bCntReload    = 0;
    edma_paramEntry.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(APP_EDMA_A_COUNT);
    edma_paramEntry.destBIdx      = (int16_t) EDMA_PARAM_BIDX(APP_EDMA_A_COUNT);
    edma_paramEntry.srcCIdx       = (int16_t) 0;
    edma_paramEntry.destCIdx      = (int16_t) APP_EDMA_A_COUNT * APP_NUM_CAP_TRIG_REG_TRANSFER;
    edma_paramEntry.linkAddr      = 0xFFFFU;
    edma_paramEntry.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edma_paramEntry.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edma_paramEntry.opt           = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
                              ((((uint32_t)(gEdmaTcc[1])) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(gEdmaBase, gEdmaPaRams[1], &edma_paramEntry);

    for(int iter = 0; iter < APP_NUM_EDMA_CHANNELS; iter++)
    {
        EDMA_configureChannelRegion(gEdmaBase, gEdmaRegId, EDMA_CHANNEL_TYPE_DMA,
            gEdmaChannels[iter], gEdmaTcc[iter], gEdmaPaRams[iter], EDMA_TEST_EVT_QUEUE_NO_0);
    }

}

void App_edmaDeConfig(void)
{
    int32_t status = SystemP_SUCCESS;

    for(int iter = 0; iter < APP_NUM_EDMA_CHANNELS; iter++)
    {
        EDMA_freeChannelRegion(gEdmaBase, gEdmaRegId, EDMA_CHANNEL_TYPE_DMA,
            gEdmaChannels[iter], gEdmaTcc[iter], gEdmaPaRams[iter], EDMA_TEST_EVT_QUEUE_NO_0);

        status = EDMA_freeDmaChannel(gEdmaHandle[0], &(gEdmaChannels[iter]));
        DebugP_assert(status == SystemP_SUCCESS);
        status = EDMA_freeTcc(gEdmaHandle[0], &(gEdmaTcc[iter]));
        DebugP_assert(status == SystemP_SUCCESS);
        status = EDMA_freeParam(gEdmaHandle[0], &gEdmaPaRams[iter]);
        DebugP_assert(status == SystemP_SUCCESS);
    }
}

bool App_compareInRange(uint32_t input1, uint32_t input2, uint32_t error_range)
{
    uint32_t absDiff = (input1 > input2)? (input1 - input2): (input2 - input1);
    if(absDiff <= error_range)
    {
        return true;
    }
    return false;
}


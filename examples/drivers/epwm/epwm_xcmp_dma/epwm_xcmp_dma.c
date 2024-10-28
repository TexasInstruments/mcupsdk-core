
/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <linker_defines.h>

/*
 * Example Description :
 *  The Example demonstrates xcmp shadow set registers of EPWMs to be loaded
 * with EDMA post period / duty calculation.
 *
 * xcmp shadow values are calculated and populated in an array in TCM, whilst
 * EDMA channels are configured for the transfer of these set of values to the
 * Shadow sets of the EPWMs, xlinked with xload register.
 *
 * EDMA Configurations :
 * 1. Each EPWM xcmp shadow set has 1 edma channel configured.
 * 2. All the EDMA channels except for the last are chained to their previous
 *    channel transfers so that, 1 manual trigger can trigger all the params
 * 3. Last EDMA channel has interrupt enabled for the intermediate and final
 *    transfers
 *
 * Shadow sets 2,3 are set to repeat 5 times each.
 *
 */

#define NUM_TEST_PWM (5)
#define MAX_SHADOW_LEVEL (3)

#define ISR_FREQUENCY_IN_HZ (20000U) // 50 uSec
#define EPWM_TBCLK_FREQUENCY_IN_HZ (200000000U)

#define EPWM_ISR_PERIOD ((EPWM_TBCLK_FREQUENCY_IN_HZ / ISR_FREQUENCY_IN_HZ))

#define TEST_DURATION_IN_SEC (1)

#define APP_EDMA_CHANNELS (15)
#define APP_EDMA_PARAMS (15)                                            // 3 epwm shadow sets to be transfered for each EPWM

#define SHADOW_REGS (9) /* 8 xcmp regs and 1 xtbprd regs */
#define EDMA_TEST_A_COUNT (4)                                           // 32 bit transfer for each xcmp register
#define EDMA_TEST_B_COUNT (SHADOW_REGS)                                           // 9 regs per shadow set.
#define EDMA_TEST_C_COUNT (TEST_DURATION_IN_SEC * ISR_FREQUENCY_IN_HZ)  // C_COUNT is a 16 bit register. max value should be < 65536

#define EDMA_TEST_EVT_QUEUE_NO (0)

extern EPWM_CurrentLink App_linkPwm;

/* base address varaible for EDMA */
uint32_t gEdmaBaseAddr;

/* Get Address of ePWM */
uint32_t gEpwmIsrBaseAddr = ISR_SOURCE_EPWM_BASE_ADDR;

uint32_t gEpwmBaseAddr[NUM_TEST_PWM] = {
    CONFIG_EPWM0_BASE_ADDR,
    CONFIG_EPWM1_BASE_ADDR,
    CONFIG_EPWM2_BASE_ADDR,
    CONFIG_EPWM3_BASE_ADDR,
    CONFIG_EPWM4_BASE_ADDR,
};

uint32_t shadowSet_offsets[MAX_SHADOW_LEVEL] = {
    CSL_EPWM_XCMP1_SHDW1 + 2U,      // using only xcmp, not xcmp_hr.
    CSL_EPWM_XCMP1_SHDW2 + 2U,      // using only xcmp, not xcmp_hr.
    CSL_EPWM_XCMP1_SHDW3 + 2U,      // using only xcmp, not xcmp_hr.
};


/* param set array */
uint32_t gEdmaPaRams[APP_EDMA_PARAMS];
/* channel set array */
uint32_t gEdmaChannels[APP_EDMA_CHANNELS];

uint32_t gEdmaRegionId, gEdmaTcc;

/* Placing these in the TCM memory for faster access. Note this Memory is not cached.
if these arrays are placed in any memory where cache is enabeld, one might have to use the CacheP_wb to reflect the data in the memories */

uint32_t gEpwm0ShadowSet1[SHADOW_REGS] TCM_A_MEM_SECTION = {0};
uint32_t gEpwm0ShadowSet2[SHADOW_REGS] TCM_A_MEM_SECTION = {0};
uint32_t gEpwm0ShadowSet3[SHADOW_REGS] TCM_A_MEM_SECTION = {0};

uint32_t gEpwm1ShadowSet1[SHADOW_REGS] TCM_A_MEM_SECTION = {0};
uint32_t gEpwm1ShadowSet2[SHADOW_REGS] TCM_A_MEM_SECTION = {0};
uint32_t gEpwm1ShadowSet3[SHADOW_REGS] TCM_A_MEM_SECTION = {0};

uint32_t gEpwm2ShadowSet1[SHADOW_REGS] TCM_A_MEM_SECTION = {0};
uint32_t gEpwm2ShadowSet2[SHADOW_REGS] TCM_A_MEM_SECTION = {0};
uint32_t gEpwm2ShadowSet3[SHADOW_REGS] TCM_A_MEM_SECTION = {0};

uint32_t gEpwm3ShadowSet1[SHADOW_REGS] TCM_A_MEM_SECTION = {0};
uint32_t gEpwm3ShadowSet2[SHADOW_REGS] TCM_A_MEM_SECTION = {0};
uint32_t gEpwm3ShadowSet3[SHADOW_REGS] TCM_A_MEM_SECTION = {0};

uint32_t gEpwm4ShadowSet1[SHADOW_REGS] TCM_A_MEM_SECTION = {0};
uint32_t gEpwm4ShadowSet2[SHADOW_REGS] TCM_A_MEM_SECTION = {0};
uint32_t gEpwm4ShadowSet3[SHADOW_REGS] TCM_A_MEM_SECTION = {0};

uint32_t* gShadowSets[NUM_TEST_PWM][MAX_SHADOW_LEVEL] = {
    { gEpwm0ShadowSet1,    gEpwm0ShadowSet2,    gEpwm0ShadowSet3},
    { gEpwm1ShadowSet1,    gEpwm1ShadowSet2,    gEpwm1ShadowSet3},
    { gEpwm2ShadowSet1,    gEpwm2ShadowSet2,    gEpwm2ShadowSet3},
    { gEpwm3ShadowSet1,    gEpwm3ShadowSet2,    gEpwm3ShadowSet3},
    { gEpwm4ShadowSet1,    gEpwm4ShadowSet2,    gEpwm4ShadowSet3},
};

uint32_t set_1Mhz[SHADOW_REGS] = {
        10,             /* xcmp1_shadow */
        100,            /* xcmp2_shadow */
        0,              /* xcmp3_shadow */
        0,              /* xcmp4_shadow */
        15,             /* xcmp5_shadow */
        105,            /* xcmp6_shadow */
        0,              /* xcmp7_shadow */
        0,              /* xcmp8_shadow */
        200,            /* xtbprd_shadow */
};

static HwiP_Object gEpwmUpdateHwiObject;
volatile uint32_t gTransferCount = 0;

void App_epwmUpdateISR(void *args);

static void App_edmaConfig(void);
static void App_edmaDeConfig(void);




void epwm_xcmp_dma_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("EPWM XCMP EDMA Test Started ...\r\n");

    /* disable TBSYNC for all EPWM */
    SOC_setMultipleEpwmTbClk(gEpwmTbClkSyncEnableMask, false);

    /* xlinking xload register */
    for(int base = 0; base <= NUM_TEST_PWM; base++)
    {
        EPWM_setupEPWMLinks(gEpwmBaseAddr[base], App_linkPwm, EPWM_LINK_XLOAD);
    }

    /* register the epwm update ISR */
    int32_t status;
    /* Initialising a Interrupt parameter */
    HwiP_Params hwiPrms;
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.priority = 0; /* setting high priority. optional */
    hwiPrms.callback = &App_epwmUpdateISR;
    status = HwiP_construct(&gEpwmUpdateHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* DMA channel Configurations */
    App_edmaConfig();

    /* enable the ISR source */
    EPWM_setTimeBasePeriod(gEpwmIsrBaseAddr, EPWM_ISR_PERIOD);
    EPWM_clearEventTriggerInterruptFlag(gEpwmIsrBaseAddr);
    EPWM_setTimeBaseCounterMode(gEpwmIsrBaseAddr, EPWM_COUNTER_MODE_UP);

    /* Setting EPWM TB Counter mode to up count mode */
    for(int base = 0; base <= NUM_TEST_PWM; base++)
    {
        EPWM_setTimeBaseCounterMode(gEpwmBaseAddr[base], EPWM_COUNTER_MODE_UP);
    }

    SOC_setMultipleEpwmTbClk(gEpwmTbClkSyncEnableMask, true);

    while(gTransferCount < EDMA_TEST_C_COUNT);

    /* pausing the EPWMs */
    SOC_setMultipleEpwmTbClk(gEpwmTbClkSyncEnableMask, false);

    /* freezing the EPWM Counters */
    EPWM_setTimeBaseCounterMode(gEpwmIsrBaseAddr, EPWM_COUNTER_MODE_STOP_FREEZE);
    for(int base = 0; base <= NUM_TEST_PWM; base++)
    {
        EPWM_setTimeBaseCounterMode(gEpwmBaseAddr[base], EPWM_COUNTER_MODE_STOP_FREEZE);
    }

    /* end example by manual validation */

    /* close emda configurations */
    App_edmaDeConfig();

    DebugP_log("EPWM XCMP EDMA Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

void App_epwmUpdateISR(void *args)
{
    /* compute the xcmp registers values */
    for(uint32_t epwm_index = 0; epwm_index < NUM_TEST_PWM; epwm_index++)
    {
        for(uint32_t shadow_index = 0; shadow_index < MAX_SHADOW_LEVEL; shadow_index++)
        {
            for(uint32_t xcmp_index = 0; xcmp_index < SHADOW_REGS; xcmp_index++)
            {
                uint32_t scale_value = 1;
                switch (shadow_index)
                {
                case 1:
                    scale_value = 4;        // shadow set 2 with 250 Khz
                    break;
                case 2:
                    scale_value = 2;        // shadow set 3 with 500 Khz
                    break;
                default:
                    /* case 0 */
                    scale_value = 1;        // shadow set 1 with 1 Mhz
                    break;
                }
                gShadowSets[epwm_index][shadow_index][xcmp_index] = scale_value*set_1Mhz[xcmp_index];

                /*
                if the  gShadowSets[epwm_index][shadow_index] are not in the TCM memory,
                and Cache is enabled for the memory where these are positioned,
                then the writes to these arrays might not reflect back on the memories, rather cached.
                In such a case, please use Cache write back to reflect these data in memories,
                so the EDMA (another Bus Master) can access these data. the following can be uncommented for the same purpose.
                */
                //CacheP_wb((void *)gShadowSets[epwm_index][shadow_index], SHADOW_REGS*4, CacheP_TYPE_ALL);
            }
        }
    }

    /* clear interrupt flag */
    EDMA_clrIntrRegion(gEdmaBaseAddr, gEdmaRegionId, gEdmaTcc);

    if(gTransferCount < EDMA_TEST_C_COUNT)
    {
        /* trigger edma transfers */
        EDMA_enableTransferRegion(gEdmaBaseAddr, gEdmaRegionId, gEdmaChannels[0], EDMA_TRIG_MODE_MANUAL);

        /* wait on all 15 dma transfers complete */
        while (EDMA_readIntrStatusRegion(gEdmaBaseAddr, gEdmaRegionId, gEdmaTcc) != 1)
        {
            /* do nothing */
        }
        /* clear interrupt flag */
        EDMA_clrIntrRegion(gEdmaBaseAddr, gEdmaRegionId, gEdmaTcc);

        /* provide xload start */
        EPWM_enableXLoad(gEpwmBaseAddr[0]);
        gTransferCount++;
    }
    else
    {
        /* stopping the ISR generating EPWM counter */
        EPWM_setTimeBaseCounterMode(gEpwmIsrBaseAddr, EPWM_COUNTER_MODE_STOP_FREEZE);
    }

    /* clear interrupt flag for next interrupt */
    EPWM_clearEventTriggerInterruptFlag(gEpwmIsrBaseAddr);

}

static void App_edmaConfig(void)
{
    int32_t status = SystemP_SUCCESS;

    gEdmaBaseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(gEdmaBaseAddr != 0);

    gEdmaRegionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(gEdmaRegionId < SOC_EDMA_NUM_REGIONS);

    /* resource allocation for all the channels */
    for (uint16_t channel = 0; channel < APP_EDMA_CHANNELS; channel++)
    {
        gEdmaChannels[channel] = EDMA_RESOURCE_ALLOC_ANY;
        status = EDMA_allocDmaChannel(gEdmaHandle[0], &(gEdmaChannels[channel]));
        DebugP_assert(status == SystemP_SUCCESS);
    }

    /* no rep required. do for only ch14 */
    gEdmaTcc = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(gEdmaHandle[0], &gEdmaTcc);
    DebugP_assert(status == SystemP_SUCCESS);

    /* PaRam allocation for all the 15 Param sets */
    for (uint16_t param = 0; param < APP_EDMA_PARAMS; param++)
    {
        gEdmaPaRams[param] = EDMA_RESOURCE_ALLOC_ANY;
        status = EDMA_allocParam(gEdmaHandle[0], &(gEdmaPaRams[param]));
        DebugP_assert(status == SystemP_SUCCESS);
    }

    DebugP_assert(SystemP_SUCCESS == status);

    /* Request 15 channels */
    for (uint16_t channel_index = 0; channel_index < APP_EDMA_CHANNELS; channel_index++)
    {
        uint16_t param_index = channel_index;
        EDMA_configureChannelRegion(gEdmaBaseAddr, gEdmaRegionId, EDMA_CHANNEL_TYPE_DMA,
                                    gEdmaChannels[channel_index], gEdmaTcc, gEdmaPaRams[param_index], EDMA_TEST_EVT_QUEUE_NO);
    }

    EDMACCPaRAMEntry edmaParam[APP_EDMA_PARAMS];
    /* common configurations for params */

    uint32_t param_index = 0;
    for(uint32_t epwm_index = 0; epwm_index < NUM_TEST_PWM; epwm_index++)
    {
        for(uint32_t shadow_index = 0; shadow_index < MAX_SHADOW_LEVEL; shadow_index++)
        {
            EDMA_ccPaRAMEntry_init(&(edmaParam[param_index]));
            edmaParam[param_index].srcAddr  = (uint32_t)SOC_virtToPhy((void *)(gShadowSets[epwm_index][shadow_index]));
            edmaParam[param_index].destAddr = (uint32_t)SOC_virtToPhy((void *)(gEpwmBaseAddr[epwm_index] + shadowSet_offsets[shadow_index]));
            edmaParam[param_index].aCnt             = (uint16_t) EDMA_TEST_A_COUNT;
            edmaParam[param_index].bCnt             = (uint16_t) EDMA_TEST_B_COUNT;
            edmaParam[param_index].cCnt             = (uint16_t) EDMA_TEST_C_COUNT;
            edmaParam[param_index].bCntReload       = (uint16_t) 0;
            edmaParam[param_index].srcBIdx          = (int16_t)EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
            edmaParam[param_index].destBIdx         = (int16_t)EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
            edmaParam[param_index].srcCIdx          = (int16_t)0;
            edmaParam[param_index].destCIdx         = (int16_t)0;
            edmaParam[param_index].linkAddr         = 0xFFFFU;
            edmaParam[param_index].srcBIdxExt       = (int8_t)0;
            edmaParam[param_index].destBIdxExt      = (int8_t)0;
            if (param_index != (APP_EDMA_CHANNELS - 1))
            {
                edmaParam[param_index].opt |=
                    (EDMA_OPT_SYNCDIM_MASK |
                    ((((uint32_t)gEdmaTcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
            }
            else
            {
                /* last param set to trigger interrupts */
                edmaParam[param_index].opt |=
                    (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
                    ((((uint32_t)gEdmaTcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
            }

            EDMA_setPaRAM(gEdmaBaseAddr, gEdmaPaRams[param_index], &(edmaParam[param_index]));


            param_index++;
        }
    }
    for(param_index = 0; param_index < APP_EDMA_PARAMS - 1; param_index++)
    {
        /* Chaining options for the current param to trigger next channel, except for the last channel */
        uint32_t chainOptions = (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK);
        uint16_t channel_index = param_index + 1;
        EDMA_chainChannel(gEdmaBaseAddr, gEdmaPaRams[param_index], gEdmaChannels[channel_index], chainOptions);
    }
}

static void App_edmaDeConfig()
{
    int32_t status = SystemP_SUCCESS;

    // status = EDMA_unregisterIntr(gEdmaHandle[0], &gEdmaIntrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    for (uint16_t channel_index = 0; channel_index < APP_EDMA_CHANNELS; channel_index++)
    {
        uint16_t param_index = channel_index;

        EDMA_freeChannelRegion(gEdmaBaseAddr, gEdmaRegionId, EDMA_CHANNEL_TYPE_DMA,
                               gEdmaChannels[channel_index], EDMA_TRIG_MODE_MANUAL, gEdmaTcc, EDMA_TEST_EVT_QUEUE_NO);

        /* Free the EDMA resources managed by driver. */
        status = EDMA_freeDmaChannel(gEdmaHandle[0], &(gEdmaChannels[channel_index]));
        DebugP_assert(status == SystemP_SUCCESS);
        status = EDMA_freeParam(gEdmaHandle[0], &(gEdmaPaRams[param_index]));
        DebugP_assert(status == SystemP_SUCCESS);
    }
    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeTcc(gEdmaHandle[0], &gEdmaTcc);
    DebugP_assert(status == SystemP_SUCCESS);
}

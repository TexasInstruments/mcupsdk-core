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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <stdlib.h>
#include <unity.h>
#include <drivers/resolver.h>
#include <drivers/soc.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/ClockP.h>

#include "menu.h"


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Testcases */


/* Non-JIRA Test case*/
static void RDC_apiCheck(void *args);

/*Utility/helper functions*/
int32_t test_rdc_cases(uint8_t in);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


#define TOTAL_TEST_CASES (1)
void test_main(void *args)
{
    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    tester_init();
    UNITY_BEGIN();
    char test_title[] = "----------------------RDC-TEST-CASES----------------------";



    menu_input test_list[TOTAL_TEST_CASES] =
    {
        {0, 1,     RDC_apiCheck,                                           "RDC_apiCheck"},
    };

    menu(TOTAL_TEST_CASES, test_list, test_title);

    if(enableLog)
    {
        DebugP_logZoneEnable(DebugP_LOG_ZONE_ERROR);
        DebugP_logZoneEnable(DebugP_LOG_ZONE_INFO);
    }
    UNITY_END();

    /* Close drivers */
    Board_driversClose();
    Drivers_close();

    return;
}

/* Unity framework required information */
void setUp(void)
{
}

void tearDown(void)
{
}

/* Test cases are numbered from 1 */
static void RDC_apiCheck(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_rdc_cases(1), 0);
}

uint32_t util_RDC_getInstanceFromBase(uint32_t base)
{
    switch(base)
    {
    case CSL_CONTROLSS_HW_RESOLVER_U_BASE:
            return 0;
    default:
            return 0xFFFFFFFF;
    }
}


#define NUM_RDC_INSTANCES (1)
uint32_t rdc_base_addr[NUM_RDC_INSTANCES] =
{
    CSL_CONTROLSS_HW_RESOLVER_U_BASE,
};

int32_t rdc_inst_tc_RDC_apiCheck(uint32_t base)
{
    /* TEST_ASSERT_BITS(mask, expected, actual) */

    int32_t error=0;
    uint32_t val = 2;

    /* Call the RDC_setAdcSequencerOperationalMode API */
    RDC_setAdcSequencerOperationalMode(base, val);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_UINT32(val, RDC_getAdcSequencerOperationalMode(base));
    TEST_ASSERT_TRUE(val == CSL_FEXT(*((uint32_t*) (base + CSL_RESOLVER_REGS_GLOBAL_CFG)), RESOLVER_REGS_GLOBAL_CFG_MODE));

    /* API checks for All write APIs in global and excitation configurations */
    for(uint16_t socWidth = 0; socWidth <= 0xFF; socWidth++)
    {
        RDC_setAdcSocWidth(base, (uint8_t) socWidth);
        /* Check if the value was written correctly */
        TEST_ASSERT_TRUE(((uint8_t)socWidth) == (uint8_t)CSL_FEXT(*((uint32_t*) (base + CSL_RESOLVER_REGS_GLOBAL_CFG)), RESOLVER_REGS_GLOBAL_CFG_SOC_WIDTH));
    }
    for(int shift = 0; shift <= 5; shift++)
    {
        uint8_t burstCount = (1U << shift);
        RDC_setAdcBurstCount(base, (uint8_t) burstCount);
        /* Check if the value was written correctly */
        TEST_ASSERT_TRUE(((uint8_t)burstCount) == (uint8_t)CSL_FEXT(*((uint32_t*)(base + CSL_RESOLVER_REGS_GLOBAL_CFG)), RESOLVER_REGS_GLOBAL_CFG_BURST_CNT));
    }

    RDC_enableAdcSingleEndedMode(base);
    /* Check if the value was written correctly */
    TEST_ASSERT_TRUE((1U) == CSL_FEXT(*((uint32_t*)(base + CSL_RESOLVER_REGS_GLOBAL_CFG)), RESOLVER_REGS_GLOBAL_CFG_SINGLE_EN));

    RDC_disableAdcSingleEndedMode( base);
    /* Check if the value was written correctly */
    TEST_ASSERT_TRUE((0U) == CSL_FEXT(*((uint32_t*)(base + CSL_RESOLVER_REGS_GLOBAL_CFG)), RESOLVER_REGS_GLOBAL_CFG_SINGLE_EN));

    uint8_t operational_modes [6] = {
        RDC_SEQUENCER_MODE_0,
        RDC_SEQUENCER_MODE_1,
        RDC_SEQUENCER_MODE_2,
        RDC_SEQUENCER_MODE_3,
        RDC_SEQUENCER_MODE_4,
        RDC_SEQUENCER_MODE_5,
    };
    for(int iter = 0; iter < 6; iter ++)
    {
        uint8_t operational_mode = operational_modes[iter];
        RDC_setAdcSequencerOperationalMode(base, operational_mode);
        /* Check if the value was written correctly */
        TEST_ASSERT_EQUAL_UINT32((uint32_t) operational_mode, RDC_getAdcSequencerOperationalMode(base));
        TEST_ASSERT_TRUE((uint8_t)operational_mode == (uint8_t)CSL_FEXT(*((uint32_t*)(base + CSL_RESOLVER_REGS_GLOBAL_CFG)), RESOLVER_REGS_GLOBAL_CFG_MODE));
    }

    RDC_enableResolver(base);
    /* Check if the value was written correctly */
    TEST_ASSERT_TRUE((1U) == CSL_FEXT(*((uint32_t*)(base + CSL_RESOLVER_REGS_GLOBAL_CFG)), RESOLVER_REGS_GLOBAL_CFG_MASTER_EN));
    RDC_disableResolver(base);
    /* Check if the value was written correctly */
    TEST_ASSERT_TRUE((0U) == CSL_FEXT(*((uint32_t*)(base + CSL_RESOLVER_REGS_GLOBAL_CFG)), RESOLVER_REGS_GLOBAL_CFG_MASTER_EN));

    for(uint16_t phase = RDC_EXCITATION_FREQUENCY_MIN_PHASE; phase <= RDC_EXCITATION_FREQUENCY_MAX_PHASE; phase++)
    {
        RDC_setExcitationSignalPhase(base, phase);
        /* Check if the value was written correctly */
        TEST_ASSERT_EQUAL_UINT32((uint32_t) phase, RDC_getExcitationSignalPhase(base));
        TEST_ASSERT_TRUE(((uint32_t) phase) == CSL_FEXT(*((uint32_t*)(base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1)), RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_PHASE_CFG));
    }

    uint8_t frequency_select_values[3] = {
        RDC_EXCITATION_FREQUENCY_5K,
        RDC_EXCITATION_FREQUENCY_10K,
        RDC_EXCITATION_FREQUENCY_20K,
    };
    for(int iter = 0; iter < 3; iter++)
    {
        uint8_t frequency_select_value = frequency_select_values[iter];
        RDC_setExcitationSignalFrequencySelect( base, frequency_select_value);
        TEST_ASSERT_EQUAL_UINT32((uint32_t) frequency_select_value, RDC_getExcitationSignalFrequencySelect(base));
        TEST_ASSERT_TRUE(((uint32_t) frequency_select_value) == CSL_FEXT(*((uint32_t*)(base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1)), RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_SEL));
    }

    RDC_enableExcitationSignalSyncIn(base);
    TEST_ASSERT_TRUE(((uint32_t) 1U) == CSL_FEXT(*((uint32_t*)(base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2)), RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EN));

    RDC_disableExcitationSignalSyncIn(base);
    TEST_ASSERT_TRUE(((uint32_t) 0U) == CSL_FEXT(*((uint32_t*)(base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2)), RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EN));

    /* RDC_clearExcitationSignalEventStatus(base); //cannot be checked as this needs an action from the IP. this API requires functional test */

    for(uint32_t socDelay = 0; socDelay <= CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_TO_SOC_DLY_START_MAX; socDelay++)
    {
        RDC_setExcitationSignalSocDelay(base, socDelay);
        TEST_ASSERT_TRUE(((uint32_t) socDelay) == CSL_FEXT(*((uint32_t*)(base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2)), RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_TO_SOC_DLY_START));
    }
    for(uint8_t amplitude = 0; amplitude <= 249; amplitude++)
    {
        RDC_setExcitationSignalAmplitudeControl(base, amplitude);
        TEST_ASSERT_TRUE(((uint32_t) amplitude) == CSL_FEXT(*((uint32_t*)(base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3)), RESOLVER_REGS_EXCIT_SAMPLE_CFG3_EXC_AMP_CTRL));
        TEST_ASSERT_TRUE( amplitude == RDC_getExcitationSignalAmplitudeControl(base));
    }

    RDC_enableSequencerInterrupt(base);
    TEST_ASSERT_TRUE(((uint32_t) 1U) == CSL_FEXT(*((uint32_t*) (base + CSL_RESOLVER_REGS_IRQSTATUS_SYS)), RESOLVER_REGS_IRQSTATUS_SYS_SEQ_ERR));

    RDC_disableSequencerInterrupt(base);
    TEST_ASSERT_TRUE(((uint32_t) 0U) == CSL_FEXT(*((uint32_t*) (base + CSL_RESOLVER_REGS_IRQSTATUS_SYS)), RESOLVER_REGS_IRQSTATUS_SYS_SEQ_ERR));

    /* RDC_clearSequencerInterrupt(uint32_t base); //cannot be checked as this needs an action from the IP. this API requires functional test */
    /* RDC_forceSequencerInterrupt(uint32_t base); //cannot be checked as this needs an action from the IP. this API requires functional test */

    /* API checks for Calibration module */
    /* RDC_clearCalibrationStatus(base); //cannot be checked as this needs an action from the IP. this API requires functional test  */
    uint8_t cal_channels [4] = {
        RDC_ADC_CAL_CHANNEL0,
        RDC_ADC_CAL_CHANNEL1,
        RDC_ADC_CAL_CHANNEL2,
        RDC_ADC_CAL_CHANNEL3,
    };
    for(uint8_t channel_index = 0; channel_index < 4; channel_index++)
    {
        uint8_t cal_channel = cal_channels[channel_index];
        RDC_selectCalibrationChannel(base, cal_channel);
        TEST_ASSERT_TRUE(((uint32_t) cal_channel) == CSL_FEXT(*((uint32_t*) (base + CSL_RESOLVER_REGS_CAL_CFG)), RESOLVER_REGS_CAL_CFG_CAL_CHSEL));

    }

    RDC_enableCalibration( base);
    TEST_ASSERT_TRUE(((uint32_t) 1U) == CSL_FEXT(*((uint32_t*) (base + CSL_RESOLVER_REGS_CAL_CFG)), RESOLVER_REGS_CAL_CFG_CAL_EN));

    /*API Checks for All write APIs in each core */
    for(uint32_t core = RDC_RESOLVER_CORE0; core <= RDC_RESOLVER_CORE1; core ++)
    {
        /* API Checks for Interrupt module */
        uint32_t offset = base + (core*RDC_CORE_OFFSET);

        uint32_t interrupt_sources[25]={
            RDC_INTERRUPT_SOURCE_LOWAMPLITUDE_ERR,
            RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_COS_FAULT_ERR,
            RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_SIN_FAULT_ERR,
            RDC_INTERRUPT_SOURCE_SINSQCOSSQ_LO_ERR,
            RDC_INTERRUPT_SOURCE_SINSQCOSSQ_HI_ERR,
            RDC_INTERRUPT_SOURCE_COS_MULTI_ZC_ERROR_ERR,
            RDC_INTERRUPT_SOURCE_SIN_MULTI_ZC_ERROR_ERR,
            RDC_INTERRUPT_SOURCE_COS_NEG_ZC_PEAK_MISMATCH_ERR,
            RDC_INTERRUPT_SOURCE_COS_POS_ZC_PEAK_MISMATCH_ERR,
            RDC_INTERRUPT_SOURCE_SIN_NEG_ZC_PEAK_MISMATCH_ERR,
            RDC_INTERRUPT_SOURCE_SIN_POS_ZC_PEAK_MISMATCH_ERR,
            RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_SIN_LO_ERR,
            RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_COS_LO_ERR,
            RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_HI_ERR,
            RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_LO_ERR,
            RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_HI_ERR,
            RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_LO_ERR,
            RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_HI_ERR,
            RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_LO_ERR,
            RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_HI_ERR,
            RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_LO_ERR,
            RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_HI_ERR,
            RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_LO_ERR,
            RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_HI_ERR,
            RDC_INTERRUPT_SOURCE_TRACK_LOCK_ERR,
        };
        for(uint32_t source_index = 0; source_index < 25; source_index++)
        {
            uint32_t interrupt_source = interrupt_sources[source_index];
            RDC_enableCoreInterrupt(base, core, interrupt_source);
            TEST_ASSERT_TRUE(((uint32_t) interrupt_source) == HW_RD_REG32(((offset + CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0))));
            TEST_ASSERT_EQUAL_UINT32((uint32_t) interrupt_source, RDC_getCoreEnabledInterruptSources(base, core));

            RDC_disableCoreInterrupt(base, core, interrupt_source);
            TEST_ASSERT_TRUE(((uint32_t) 0U) == HW_RD_REG32(((offset + CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0))));
            /* RDC_clearCoreInterrupt(base, core, interrupt_source); //cannot be checked as this needs an action from the IP. this API requires functional test */
            /* RDC_forceCoreInterrupt(base, core, interrupt_source); //cannot be checked as this needs an action from the IP. this API requires functional test */
        }

        /* API Check for DC and BPF configurations */
        for(uint8_t coef1=0; coef1 < 16; coef1++)
        {
            for(uint8_t coef2=0; coef2 <16; coef2++)
            {
                RDC_setDcOffsetCalCoef(base, core, coef1, coef2);
                TEST_ASSERT_TRUE(((uint32_t) coef1) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_DC_OFF_CFG1_0)), RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF1));
                TEST_ASSERT_TRUE(((uint32_t) coef2) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_DC_OFF_CFG1_0)), RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF2));
            }
        }

        RDC_enableBPF(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 1U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_DC_OFF_CFG1_0)), RESOLVER_REGS_DC_OFF_CFG1_0_BANDPASSFILTER_ON));
        RDC_disableBPF(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 0U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_DC_OFF_CFG1_0)), RESOLVER_REGS_DC_OFF_CFG1_0_BANDPASSFILTER_ON));

        RDC_disableDcOffsetAutoCorrection(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 1U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_DC_OFF_CFG1_0)), RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_CORR_ON));
        RDC_enableDcOffsetAutoCorrection(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 0U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_DC_OFF_CFG1_0)), RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_CORR_ON));

        int32_t step = 1001;    /* reason for step usage : the test takes quite too long for (2^32) * (2^32) runs */
        for(int32_t sin_32i = -32768;  sin_32i < 32768; sin_32i+=step)
        {
            int16_t sin = (int16_t) sin_32i;
            for(int32_t cos_32i = -32768;  cos_32i < 32768; cos_32i++)
            {
                int16_t cos = (int16_t) cos_32i;
                RDC_setDcOffsetManualCorrectionValue(base, core, sin, cos);
                TEST_ASSERT_TRUE(((int16_t) sin) == ((int16_t)CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_DC_OFF_CFG2_0)), RESOLVER_REGS_DC_OFF_CFG2_0_SIN_MAN_OFF_ADJ)));
                TEST_ASSERT_TRUE(((int16_t) cos) == ((int16_t)CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_DC_OFF_CFG2_0)), RESOLVER_REGS_DC_OFF_CFG2_0_COS_MAN_OFF_ADJ)));
            }
        }
        /* API check for the ideal sample configurations */
        for(uint8_t overrideValue = 0; overrideValue < 20; overrideValue++)
        {
            RDC_overrideIdealSampleTime(base, core, overrideValue);
            TEST_ASSERT_TRUE(((uint32_t) overrideValue) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_SAMPLE_CFG1_0)), RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_OVR));

            RDC_setIdealSampleBpfAdjust(base, core, overrideValue);    //sampleAdjustCount is similar to the overrideValue in range
            TEST_ASSERT_TRUE(((uint32_t) overrideValue) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_SAMPLE_CFG2_0)), RESOLVER_REGS_SAMPLE_CFG2_0_BANDPASSFILTERSAMPLEADJUST));
        }
        for(uint16_t absThresholdValue =0; absThresholdValue <= 0xFFFF; absThresholdValue++)
        {
            RDC_setIdealSampleDetectionThreshold(base, core, absThresholdValue);
            TEST_ASSERT_TRUE(((uint32_t) absThresholdValue) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_SAMPLE_CFG2_0)), RESOLVER_REGS_SAMPLE_CFG2_0_SAMPLE_DET_THRESHOLD));
            if(absThresholdValue == 0xFFFF)
            {
                break;
            }
        }
        uint8_t modes[4] = {
            RDC_IDEAL_SAMPLE_TIME_MODE_0_AUTO_DETECT,
            RDC_IDEAL_SAMPLE_TIME_MODE_1_AUTO_DETECT_ON_SIN,
            RDC_IDEAL_SAMPLE_TIME_MODE_2_AUTO_DETECT_ON_COS,
            RDC_IDEAL_SAMPLE_TIME_MODE_3_AUTO_DETECT_OFF,
        };
        for(uint8_t mode_index = 0; mode_index < 4; mode_index++ )
        {
            uint8_t mode = modes[mode_index];
            RDC_setIdealSampleMode(base, core, mode);
            TEST_ASSERT_TRUE(((uint32_t) mode) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_DEC_GF_CFG0)), RESOLVER_REGS_DEC_GF_CFG0_IDEAL_SAMPLE_MODE));
        }
        RDC_enableIdealSampleBottomSampling(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 1U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_DEC_GF_CFG0)), RESOLVER_REGS_DEC_GF_CFG0_ENABLE_BOTTOM));
        RDC_disableIdealSampleBottomSampling(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 0U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_DEC_GF_CFG0)), RESOLVER_REGS_DEC_GF_CFG0_ENABLE_BOTTOM));

        /* API check for phase gain configurations */
        for(uint8_t pgEstimationLimit = 0; pgEstimationLimit < 16; pgEstimationLimit++)
        {
            RDC_setPhaseGainEstimationTrainLimit(base, core, pgEstimationLimit);
            TEST_ASSERT_TRUE(((uint32_t) pgEstimationLimit) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_PG_EST_CFG1_0)), RESOLVER_REGS_PG_EST_CFG1_0_PG_TRAIN_LIMIT));
        }
        for(uint16_t cosPhaseBypass = 0; cosPhaseBypass <= 0xFFFF; cosPhaseBypass++)
        {
            RDC_setCosPhaseBypass(base, core, cosPhaseBypass);
            TEST_ASSERT_TRUE(((uint32_t) cosPhaseBypass) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_PG_EST_CFG2_0)), RESOLVER_REGS_PG_EST_CFG2_0_PHASECOSBYP0));
            if(cosPhaseBypass == 0xFFFF)
            {
                break;
            }
        }
        RDC_enablePhaseGainEstimation(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 0U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_PG_EST_CFG2_0)), RESOLVER_REGS_PG_EST_CFG2_0_BYPASSPHASEGAINCORR0));
        RDC_disablePhaseGainEstimation(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 1U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_PG_EST_CFG2_0)), RESOLVER_REGS_PG_EST_CFG2_0_BYPASSPHASEGAINCORR0));

        RDC_enablePhaseAutoCorrection(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 1U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_PG_EST_CFG2_0)), RESOLVER_REGS_PG_EST_CFG2_0_AUTOPHASECONTROL0));
        RDC_disablePhaseAutoCorrection(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 0U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_PG_EST_CFG2_0)), RESOLVER_REGS_PG_EST_CFG2_0_AUTOPHASECONTROL0));
        RDC_enableGainAutoCorrection(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 1U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_PG_EST_CFG2_0)), RESOLVER_REGS_PG_EST_CFG2_0_AUTOGAINCONTROL0));
        RDC_disableGainAutoCorrection(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 0U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_PG_EST_CFG2_0)), RESOLVER_REGS_PG_EST_CFG2_0_AUTOGAINCONTROL0));
        for(int32_t sinGainBypass_32i = 1; sinGainBypass_32i < 32768; sinGainBypass_32i += step)
        {
            int16_t sinGainBypass = (int16_t) sinGainBypass_32i;
            for(int32_t cosGainBypass_32i = -32768; cosGainBypass_32i < 32768; cosGainBypass_32i ++)
            {
                int16_t cosGainBypass = (int16_t) cosGainBypass_32i;
                RDC_setGainBypassValue(base, core, sinGainBypass, cosGainBypass);
                TEST_ASSERT_TRUE((sinGainBypass) == (int16_t)CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_PG_EST_CFG3_0)), RESOLVER_REGS_PG_EST_CFG3_0_GAINSINBYP0));
                TEST_ASSERT_TRUE((cosGainBypass) == (int16_t)CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_PG_EST_CFG3_0)), RESOLVER_REGS_PG_EST_CFG3_0_GAINCOSBYP0));
            }
        }
        /* API Check for track2 configurations */
        for(uint8_t kvelfilt = 0; kvelfilt <= 10; kvelfilt++)
        {
            Track2Constants_t track2Constants;
            track2Constants.kvelfilt = kvelfilt;
            RDC_setTrack2Constants(base, core, &track2Constants);
            TEST_ASSERT_TRUE(((uint32_t)kvelfilt) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_TRACK2_CFG1_0)), RESOLVER_REGS_TRACK2_CFG1_0_KVELFILT));
        }
        RDC_enableTrack2Boost(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 1U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_TRACK2_CFG2_0)), RESOLVER_REGS_TRACK2_CFG2_0_BOOST));
        RDC_disableTrack2Boost(base, core);
        TEST_ASSERT_TRUE(((uint32_t) 0U) == CSL_FEXT(*((uint32_t*) (offset + CSL_RESOLVER_REGS_TRACK2_CFG2_0)), RESOLVER_REGS_TRACK2_CFG2_0_BOOST));
        /* API Check for Diagnostic data */
        // RDC_setDiagnosticsSinCosOffsetDriftData(base, uint8_t resolverCore, Diag_Mon_SinCos_Offset_drift_data *monitorData);
        // RDC_setDiagnosticsSinCosGainDriftData(base, uint8_t resolverCore, Diag_Mon_SinCos_Gain_drift_data *monitorData);
        // RDC_setDiagnosticsCosPhaseDriftData(base, uint8_t resolverCore, Diag_Mon_Cos_Phase_drift_data *monitorData);
        // RDC_setDiagnosticsExcFreqDegradationData(base, uint8_t resolverCore, Diag_Mon_ExcFreq_Degradataion_data *monitorData);
        // RDC_setDiagnosticsRotationalSignalIntegrityData(base, uint8_t resolverCore, Diag_Mon_Rotational_Signal_Integrity_data *monitorData);
        // RDC_setDiagnosticsSignalIntegritySquareSumData(base,uint32_t resolverCore,Diag_Mon_Signal_Integrity_SinSq_CosSq *monitorData);
        // RDC_setDiagnosticsHighAmplitudeData( base, uint8_t resolverCore, Diag_Mon_Sin_Cos_High_Amplitude* monitorData);
        // RDC_setDiagnosticsWeakAmplitudeData( base, uint8_t resolverCore, Diag_Mon_Sin_Cos_Weak_Amplitude * monitorData);
    }

    return error;
}

uint8_t isTestSupported[NUM_RDC_INSTANCES][TOTAL_TEST_CASES] =
{
    {1},  //Supported tests for RDC0
};

int32_t test_rdc_cases(uint8_t in)
{
    int32_t failcount=0;

    uint32_t base;
    uint32_t rdc_instance = 0;
    for(rdc_instance = 0; rdc_instance < NUM_RDC_INSTANCES; rdc_instance++)
    {
        base = rdc_base_addr[rdc_instance];
        if(enableLog)
        {
            DebugP_log("\r\n\nbase=0x%08x, instance=%d\r\n", base, util_RDC_getInstanceFromBase(base));
        }

        if(isTestSupported[util_RDC_getInstanceFromBase(base)][in-1])
        {
            switch(in)
            {
                case 1:
                    failcount += rdc_inst_tc_RDC_apiCheck(base);
                    break;
            }
            if(enableLog)
            {
                DebugP_log("\r\nfailcount=%d", failcount);
            }

        }
        else
        {
            if(enableLog)
            {
                DebugP_log("\r\nRDC %u, Test %u not supported", util_RDC_getInstanceFromBase(base), in-1);
            }

        }

    }


    if(failcount!=0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nFAIL %d", failcount);
        }

        return 1;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }

        return 0;
    }

}

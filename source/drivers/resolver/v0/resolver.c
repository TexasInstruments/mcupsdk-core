/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <drivers/resolver.h>
#include <stdint.h>


// RESET PARAM VALUE
#define RDC_BPFDC_BPFENABLE_RESET_PARAM_VALUE                      (TRUE)
#define RDC_BPFDC_OFFSETCORRECTIONENABLE_RESET_PARAM_VALUE         (TRUE)
#define RDC_BPFDC_AUTOOFFSETCORRECTIONENABLE_RESET_PARAM_VALUE     (FALSE)
#define RDC_BPFDC_DCOFFCAL1_RESET_PARAM_VALUE                      (8U)
#define RDC_BPFDC_DCOFFCAL2_RESET_PARAM_VALUE                      (9U)
#define RDC_BPFDC_MANUALSIN_RESET_PARAM_VALUE                      (0U)
#define RDC_BPFDC_MANUALCOS_RESET_PARAM_VALUE                      (0U)
#define RDC_IDEALSAMPLE_OVERRIDEVALUE_RESET_PARAM_VALUE            (0U)
// #define RDC_IDEALSAMPLE_PEAKAVGLIMIT_RESET_PARAM_VALUE             (7U)
#define RDC_IDEALSAMPLE_ABSTHRESHOLDVALUE_RESET_PARAM_VALUE        (2500U)
#define RDC_IDEALSAMPLE_SAMPLEADJUSTCOUNT_RESET_PARAM_VALUE        (0U)
#define RDC_IDEALSAMPLE_MODE_RESET_PARAM_VALUE                     (0U)
#define RDC_IDEALSAMPLE_BOTTOMSAMPLEENABLE_RESET_PARAM_VALUE       (FALSE)
#define RDC_PG_ESTIMATIONENABLE_RESET_PARAM_VALUE                  (FALSE)
#define RDC_PG_GLITCHTHRESHOLD_RESET_PARAM_VALUE                   (16384U)
#define RDC_PG_ESTIMATIONLIMIT_RESET_PARAM_VALUE                   (8U)
#define RDC_PG_CORRECTIONENABLE_RESET_PARAM_VALUE                  (TRUE)
#define RDC_PG_AUTOCORRECTIONENABLE_RESET_PARAM_VALUE              (FALSE)
#define RDC_PG_SINGAINBYPASSVALUE_RESET_PARAM_VALUE                (16384U)
#define RDC_PG_COSGAINBYPASSVALUE_RESET_PARAM_VALUE                (16384U)
#define RDC_PG_COSPHASEBYPASSVALUE_RESET_PARAM_VALUE               (0U)
#define RDC_T2_KVELFILT_RESET_PARAM_VALUE                          (8U)

void RDC_coreParamsInit(Core_config_t* coreParams)
{
    coreParams->BpfDc_bpfEnable                     = RDC_BPFDC_BPFENABLE_RESET_PARAM_VALUE;
    coreParams->BpfDc_offsetCorrectionEnable        = RDC_BPFDC_OFFSETCORRECTIONENABLE_RESET_PARAM_VALUE;
    // coreParams->BpfDc_autoOffsetCorrectionEnable    = RDC_BPFDC_AUTOOFFSETCORRECTIONENABLE_RESET_PARAM_VALUE;
    coreParams->BpfDc_dcOffCal1                     = RDC_BPFDC_DCOFFCAL1_RESET_PARAM_VALUE;
    coreParams->BpfDc_dcOffCal2                     = RDC_BPFDC_DCOFFCAL2_RESET_PARAM_VALUE;
    coreParams->BpfDc_manualSin                     = RDC_BPFDC_MANUALSIN_RESET_PARAM_VALUE;
    coreParams->BpfDc_manualCos                     = RDC_BPFDC_MANUALCOS_RESET_PARAM_VALUE;

    coreParams->IdealSample_overrideValue           = RDC_IDEALSAMPLE_OVERRIDEVALUE_RESET_PARAM_VALUE;
    // coreParams->IdealSample_peakAvgLimit            = RDC_IDEALSAMPLE_PEAKAVGLIMIT_RESET_PARAM_VALUE;
    coreParams->IdealSample_absThresholdValue       = RDC_IDEALSAMPLE_ABSTHRESHOLDVALUE_RESET_PARAM_VALUE;
    coreParams->IdealSample_sampleAdjustCount       = RDC_IDEALSAMPLE_SAMPLEADJUSTCOUNT_RESET_PARAM_VALUE;
    coreParams->IdealSample_mode                    = RDC_IDEALSAMPLE_MODE_RESET_PARAM_VALUE;
    coreParams->IdealSample_bottomSampleEnable      = RDC_IDEALSAMPLE_BOTTOMSAMPLEENABLE_RESET_PARAM_VALUE;

    coreParams->Pg_estimationEnable                 = RDC_PG_ESTIMATIONENABLE_RESET_PARAM_VALUE;
    // coreParams->Pg_glitchThreshold                  = RDC_PG_GLITCHTHRESHOLD_RESET_PARAM_VALUE;
    coreParams->Pg_estimationLimit                  = RDC_PG_ESTIMATIONLIMIT_RESET_PARAM_VALUE;

    coreParams->Pg_correctionEnable                 = RDC_PG_CORRECTIONENABLE_RESET_PARAM_VALUE;
    coreParams->Pg_autoCorrectionEnable             = RDC_PG_AUTOCORRECTIONENABLE_RESET_PARAM_VALUE;
    coreParams->Pg_sinGainBypassValue               = RDC_PG_SINGAINBYPASSVALUE_RESET_PARAM_VALUE;
    coreParams->Pg_cosGainBypassValue               = RDC_PG_COSGAINBYPASSVALUE_RESET_PARAM_VALUE;
    coreParams->Pg_cosPhaseBypassValue              = RDC_PG_COSPHASEBYPASSVALUE_RESET_PARAM_VALUE;

    (coreParams->track2Constants).kvelfilt        = RDC_T2_KVELFILT_RESET_PARAM_VALUE;
}

#define RDC_ADC_PARAM1_RESET_PARAM_VALUE            (OVERSAMPLING_RATIO_20)
#define RDC_IDEALSAMPLE_PARAM2_RESET_PARAM_VALUE    (7U)
#define RDC_DC_PARAM3_RESET_PARAM_VALUE             (2U)
#define RDC_PG_PARAM4_RESET_PARAM_VALUE             (16384U)
#define RDC_T2_PARAM5_RESET_PARAM_VALUE             (6U)    // kffw         cfg2
#define RDC_T2_PARAM6_RESET_PARAM_VALUE             (64U)   // ki           cfg1
#define RDC_T2_PARAM7_RESET_PARAM_VALUE             (10U)   // kpdiv        cfg3
#define RDC_T2_PARAM8_RESET_PARAM_VALUE             (7U)    // vboost coef  cfg3
#define RDC_T2_PARAM9_RESET_PARAM_VALUE             (FALSE) // boost vel    cfg2

void RDC_BaselineParametersInit(uint32_t base)
{
    baselineParameters params;
    params.adcParam1   = RDC_ADC_PARAM1_RESET_PARAM_VALUE;
    params.IdealParam2 = RDC_IDEALSAMPLE_PARAM2_RESET_PARAM_VALUE;
    params.DcParam3    = RDC_DC_PARAM3_RESET_PARAM_VALUE;
    params.PgParam4    = RDC_PG_PARAM4_RESET_PARAM_VALUE;
    params.t2Param5    = RDC_T2_PARAM5_RESET_PARAM_VALUE;  // kffw         cfg2
    params.t2Param6    = RDC_T2_PARAM6_RESET_PARAM_VALUE;  // ki           cfg1
    params.t2Param7    = RDC_T2_PARAM7_RESET_PARAM_VALUE;  // kpdiv        cfg3
    params.t2Param8    = RDC_T2_PARAM8_RESET_PARAM_VALUE;  // vboost coef  cfg3
    params.t2Param9    = RDC_T2_PARAM9_RESET_PARAM_VALUE;  // boost vel    cfg2

    /* the APIs are removed. so need to write the reg level values */
    /* TODO: need to add the T2 coefs for the SW track2 as well */
    HW_WR_REG32(
            base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1) &
             ~CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_ADC_SAMPLE_RATE_MASK) |
                (((uint32_t)(params.adcParam1)) << CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_ADC_SAMPLE_RATE_SHIFT));
    for(int core = 0; core <= 1; core++)
    {
        HW_WR_REG32(
                base + CSL_RESOLVER_REGS_DC_OFF_CFG1_0 + (core * RDC_CORE_OFFSET),
                ((HW_RD_REG32(
                    base + CSL_RESOLVER_REGS_DC_OFF_CFG1_0 + (core * RDC_CORE_OFFSET)) & ~CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFFSET_HYSTERESIS_MASK) |
                    (((uint32_t) (params.DcParam3)) <<  CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFFSET_HYSTERESIS_SHIFT)));

        HW_WR_REG32(
                base + CSL_RESOLVER_REGS_SAMPLE_CFG1_0 + (core * RDC_CORE_OFFSET),
                ((HW_RD_REG32(
                    base + CSL_RESOLVER_REGS_SAMPLE_CFG1_0 + (core * RDC_CORE_OFFSET)) &
                ~CSL_RESOLVER_REGS_SAMPLE_CFG1_0_PEAK_AVG_LIMIT_MASK) |
                    (((uint32_t) (params.IdealParam2)) << CSL_RESOLVER_REGS_SAMPLE_CFG1_0_PEAK_AVG_LIMIT_SHIFT)));

        HW_WR_REG32(
                base + CSL_RESOLVER_REGS_PG_EST_CFG4_0 + (core * RDC_CORE_OFFSET),
                ((HW_RD_REG32(
                    base + CSL_RESOLVER_REGS_PG_EST_CFG4_0 + (core * RDC_CORE_OFFSET)) &
                ~CSL_RESOLVER_REGS_PG_EST_CFG4_0_PG_GLITCHTHRESHOLD_MASK) |
                    ((params.PgParam4) << CSL_RESOLVER_REGS_PG_EST_CFG4_0_PG_GLITCHTHRESHOLD_SHIFT)));

        HW_WR_REG32(
                base + CSL_RESOLVER_REGS_TRACK2_CFG1_0 + (core * RDC_CORE_OFFSET),
                ((HW_RD_REG32(
                    base + CSL_RESOLVER_REGS_TRACK2_CFG1_0 + (core * RDC_CORE_OFFSET)) &
                ~CSL_RESOLVER_REGS_TRACK2_CFG1_0_KI_MASK) |
                    ((params.t2Param6) << CSL_RESOLVER_REGS_TRACK2_CFG1_0_KI_SHIFT)));

        HW_WR_REG32(
                base + CSL_RESOLVER_REGS_TRACK2_CFG2_0 + (core * RDC_CORE_OFFSET),
                ((HW_RD_REG32(
                    base + CSL_RESOLVER_REGS_TRACK2_CFG2_0 + (core * RDC_CORE_OFFSET)) &
                ~(CSL_RESOLVER_REGS_TRACK2_CFG2_0_BOOST_MASK | CSL_RESOLVER_REGS_TRACK2_CFG2_0_KFFW_MASK)) |
                    ((params.t2Param5) << CSL_RESOLVER_REGS_TRACK2_CFG2_0_KFFW_SHIFT)));

        HW_WR_REG32(
                base + CSL_RESOLVER_REGS_TRACK2_CFG3_0 + (core * RDC_CORE_OFFSET),
                (HW_RD_REG32(
                    base + CSL_RESOLVER_REGS_TRACK2_CFG3_0 + (core * RDC_CORE_OFFSET)) &
                ~(CSL_RESOLVER_REGS_TRACK2_CFG3_0_KPDIV_MASK | CSL_RESOLVER_REGS_TRACK2_CFG3_0_VBOOSTCOEFF_MASK)) |
                    (((params.t2Param7) << CSL_RESOLVER_REGS_TRACK2_CFG3_0_KPDIV_SHIFT) |
                    ((params.t2Param8) << CSL_RESOLVER_REGS_TRACK2_CFG3_0_VBOOSTCOEFF_SHIFT)));
    }
}

void RDC_paramsInit(RDC_configParams* params)
{
        params->adv_config          = FALSE;
        params->Input_signalMode    = RDC_SINGALMODE_SINGLE_ENDED;
        params->Input_socWidth      = 0;
        params->Input_adcBurstCount = RDC_ADC_BURST_COUNT_DISABLE;
        params->Input_resolverSequencerMode = RDC_SEQUENCER_MODE_0;

        params->ExcFrq_freqSel      = RDC_EXCITATION_FREQUENCY_20K;
        params->ExcFrq_phase        = 0;
        params->ExcFrq_amplitude    = 0XEFU;
        params->ExcFrq_enableSyncIn = FALSE;
        params->ExcFrq_socDelay     = 0x190U;
        // uint8_t     ExcFrq_overSamplingRatio;

        RDC_coreParamsInit(&(params->core0));
        RDC_coreParamsInit(&(params->core1));

        params->Int_seqEnable       = FALSE;
        params->Int_core0Interrupts = 0;
        params->Int_core1Interrupts = 0;
}
void RDC_init(uint32_t base, RDC_configParams* params)
{
    RDC_disableResolver(base);
    RDC_BaselineParametersInit(base);

    /* INPUT CONFIGURATIONS */
    if(params->Input_signalMode == RDC_SINGALMODE_SINGLE_ENDED)
    {
        RDC_enableAdcSingleEndedMode(base);
    }
    else
    {
        RDC_disableAdcSingleEndedMode(base);
    }
    RDC_setAdcSocWidth(base, (uint8_t) (params->Input_socWidth));
    RDC_setAdcBurstCount(base, (uint8_t) (params->Input_adcBurstCount));
    RDC_setAdcSequencerOperationalMode(base, (uint8_t) (params->Input_resolverSequencerMode));

    /* EXCITATION FREQUENCY CONFIGURATIONS */
    RDC_setExcitationSignalFrequencySelect(base, (uint8_t) (params->ExcFrq_freqSel));
    RDC_setExcitationSignalPhase(base, (uint16_t) (params->ExcFrq_phase));
    RDC_setExcitationSignalAmplitudeControl(base, (uint8_t) (params->ExcFrq_amplitude));
    if(params->ExcFrq_enableSyncIn)
    {
        RDC_enableExcitationSignalSyncIn(base);
    }
    else
    {
        RDC_disableExcitationSignalSyncIn(base);
    }
    RDC_setExcitationSignalSocDelay(base, (uint16_t) (params->ExcFrq_socDelay));

    /*  RESOVLER resolverCore CONFIGURATIONS */
    int resolverCore = RDC_RESOLVER_CORE0;
    do{
        Core_config_t coreParams;

        switch(resolverCore)
        {
            case 0 : {
                coreParams = params->core0;
                break;
            }
            case 1 : {
                coreParams = params->core1;
                break;
            }
            default : {
                coreParams = params->core0;
                break;
            }
        }

        /* Band Pass or DC Offset Correction*/
        if(coreParams.BpfDc_bpfEnable == true)
        {
            RDC_enableBPF(base, resolverCore);
        }
        else
        {
            RDC_disableBPF(base, resolverCore);
            RDC_setDcOffsetCalCoef(
                base,
                resolverCore,
                (uint8_t) (coreParams.BpfDc_dcOffCal1),
                (uint8_t) (coreParams.BpfDc_dcOffCal2));

            if(coreParams.BpfDc_offsetCorrectionEnable)
            {

                RDC_enableDcOffsetCorrection(base, resolverCore);
                RDC_setDcOffsetManualCorrectionValue(
                    base,
                    resolverCore,
                    (int16_t)(coreParams.BpfDc_manualSin),
                    (int16_t)(coreParams.BpfDc_manualCos));
            }
            else
            {
                RDC_disableDcOffsetCorrection(base, resolverCore);
            }
        }
        /* Ideal Sample Configurations */
        RDC_setIdealSampleMode(base, resolverCore, (uint8_t) (coreParams.IdealSample_mode));
        RDC_overrideIdealSampleTime(base, resolverCore, (uint8_t) (coreParams.IdealSample_overrideValue));
        RDC_setIdealSampleDetectionThreshold(base, resolverCore, (uint16_t) (coreParams.IdealSample_absThresholdValue));
        RDC_setIdealSampleBpfAdjust(base, resolverCore, (uint8_t) (coreParams.IdealSample_sampleAdjustCount));
        if(coreParams.IdealSample_bottomSampleEnable == true)
        {
            RDC_enableIdealSampleBottomSampling(base, resolverCore);
        }
        else
        {
            RDC_disableIdealSampleBottomSampling(base, resolverCore);
        }

        /* Phase Gain configurations */
        if(coreParams.Pg_estimationEnable == true)
        {
            RDC_setPhaseGainEstimationTrainLimit(base, resolverCore, (uint8_t) (coreParams.Pg_estimationLimit));
            RDC_enablePhaseGainEstimation(base, resolverCore);
        }
        else
        {
            RDC_disablePhaseGainEstimation(base, resolverCore);
        }

        if((coreParams.Pg_autoCorrectionEnable == true) &&
           (coreParams.Pg_estimationEnable == true))
        {
            RDC_enablePhaseAutoCorrection(base, resolverCore);
            RDC_enableGainAutoCorrection(base, resolverCore);
        }
        else
        {
            RDC_disablePhaseAutoCorrection(base, resolverCore);
            RDC_disableGainAutoCorrection(base, resolverCore);

            RDC_setGainBypassValue(
                base,
                resolverCore,
                (int16_t) coreParams.Pg_sinGainBypassValue,
                (int16_t) coreParams.Pg_cosGainBypassValue);
            RDC_setCosPhaseBypass(
                base,
                resolverCore,
                (int16_t) coreParams.Pg_cosPhaseBypassValue);
        }

        /* track2 configurations */
        RDC_setTrack2Constants(base, resolverCore, &(coreParams.track2Constants));

        /* update or exit the loop */
        if(resolverCore == RDC_RESOLVER_CORE0)    // and resolverCore = any sequencer mode that involves two cores
        {
            resolverCore = RDC_RESOLVER_CORE1;
        }
        else
        {
            break;
        }
    }while ((resolverCore == RDC_RESOLVER_CORE0) || (resolverCore == RDC_RESOLVER_CORE1));

    if(params->Int_seqEnable)
    {
        RDC_enableSequencerInterrupt(base);
    }
    else
    {
        RDC_disableSequencerInterrupt(base);
    }
    RDC_enableCoreInterrupt(base, RDC_RESOLVER_CORE0, (params->Int_core0Interrupts));
    RDC_enableCoreInterrupt(base, RDC_RESOLVER_CORE1, (params->Int_core1Interrupts));
}
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

/**
 *  \defgroup DRV_RESOLVER_MODULE APIs for RESOLVER
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the RESOLVER module.
 *
 *  @{
 */

#ifndef RESOLVER_V1_H_
#define RESOLVER_V1_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! Header Files
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/cslr_resolver.h>
#include <kernel/dpl/SystemP.h>

//*****************************************************************************
//
// Defines for the API.
//
//*****************************************************************************
//*****************************************************************************
//
//! Defines to be used by the driver
//
//*****************************************************************************
#define RDC_CORE_OFFSET (CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1 - CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0)

/**
 * @brief Minimum Phase value that can be programmed for the Excitation signal */
#define RDC_EXCITATION_FREQUENCY_MIN_PHASE (0U)
/**
 * @brief Maximum Phase value that can be programmed for the Excitation signal */
#define RDC_EXCITATION_FREQUENCY_MAX_PHASE (7999U)

/**
 * @brief Maximum Excitation Signal Amplitude */
#define RDC_MAX_EXCITATION_AMPLITUDE (249U)

/**
 * @brief Macro used to specify resolver core 0*/
#define RDC_RESOLVER_CORE0 (0U)
/**
 * @brief Macro used to specify resolver core 1*/
#define RDC_RESOLVER_CORE1 (1U)

/**
 * @brief Macros used to sepcify Resovler Channel for the Calibration in the 
 * RDC_selectCalibrationChannel() as \e calChannel parameter */
#define RDC_ADC_CAL_CHANNEL0 (0U)
#define RDC_ADC_CAL_CHANNEL1 (1U)
#define RDC_ADC_CAL_CHANNEL2 (2U)
#define RDC_ADC_CAL_CHANNEL3 (3U)
#define RDC_ADC_CAL_CHANNEL_CAL2 (4U)
#define RDC_ADC_CAL_CHANNEL_CAL3 (5U)
#define RDC_ADC_CAL_CHANNEL_CAL0 (6U)
#define RDC_ADC_CAL_CHANNEL_CAL1 (7U)


#define RDC_DC_OFFSET_SIN_ESTIMATION (0U)
#define RDC_DC_OFFSET_COS_ESTIMATION (1U)


#define RDC_MIN_IDEAL_SAMPLE_PEAK_AVG_LIMIT (0U)
#define RDC_MAX_IDEAL_SAMPLE_PEAK_AVG_LIMIT (7U)

#define RDC_MAX_IDEAL_SAMPLE_BPF_ADJUST (0x0000001FU)

#define RDC_SINGALMODE_SINGLE_ENDED (0U)
#define RDC_SINGALMODE_DIFFERENTIAL_ENDED (1U)


//*****************************************************************************
//
//! Values that can be passed to
//! RDC_setAdcBurstCount() as
//! \e burstCount parameter
//
//*****************************************************************************
/**
 * @brief  Burst Count is disabled */
#define RDC_ADC_BURST_COUNT_DISABLE (1U)
/**
 * @brief  2    ADC Samples to be averaged */
#define RDC_ADC_BURST_COUNT_2 (2U)
/**
 * @brief  4    ADC Samples to be averaged */
#define RDC_ADC_BURST_COUNT_4 (4U)
/**
 * @brief  8    ADC Samples to be averaged */
#define RDC_ADC_BURST_COUNT_8 (8U)
/**
 * @brief  16   ADC Samples to be averaged */
#define RDC_ADC_BURST_COUNT_16 (16U)
/**
 * @brief  32   ADC Samples to be averaged */
#define RDC_ADC_BURST_COUNT_32 (32U)


//*****************************************************************************
//
//! Values that can be passed to
//! RDC_setAdcSequencerOperationalMode() as
//! \e operationalMode parameter
//
//! or returned by RDC_getAdcSequencerOperationalMode()
//
//*****************************************************************************
/**
 * @brief
 * ADC0 Samples Sin And Adc1 Samples Cos Parallelly For Core 0  */
#define RDC_SEQUENCER_MODE_0 (0U)
/**
 * @brief
 * ADC0 Samples Sin And Cos Sequentially For Core 0  */
#define RDC_SEQUENCER_MODE_1 (1U)
/**
 * @brief
 * ADC0 Samples Sin0, Cos0, Cos1, Sin1 Sequentially, To Be Averaged For Core 0
 */
#define RDC_SEQUENCER_MODE_2 (2U)
/**
 * @brief
 * ADC0 Samples Sin And ADC1 Samples Cos Parallelly For Core 0
 * Followed By
 * ADC0 Samples Sin And ADC1 Samples Cos Parallelly For Core 1
 *
 */
#define RDC_SEQUENCER_MODE_3 (3U)
/**
 * @brief
 * ADC0 Samples Sin For Core 0 And ADC1 Samples Sin For Core 1 Parallelly
 * Followed By
 * ADC0 Samples Cos For Core 0 And ADC1 Samples Cos For Core 2
 *
 */
#define RDC_SEQUENCER_MODE_4 (4U)
/**
 * @brief
 * ADC0/1 Parallelly Sample Sin0, Cos0, Cos1, Sin1 Samples Sequentially
 * For Core0/1. Both Sin(Cos) Samples For Each Core Will Be Averaged.
 *
 */
#define RDC_SEQUENCER_MODE_5 (5U)


//*****************************************************************************
//
//! Values that can be passed to
//! RDC_setExcitationSignalFrequencySelect() as
//! \e FrequencySel parameter
//
//! or returned by RDC_getExcitationSignalFrequencySelect()
//
//*****************************************************************************
/**
 * @brief select 5KHz Excitation Sine Frequency */
#define RDC_EXCITATION_FREQUENCY_5K (50)
/**
 * @brief select 10KHz Excitation Sine Frequency */
#define RDC_EXCITATION_FREQUENCY_10K (100)
/**
 * @brief select 20KHz Excitation Sine Frequency */
#define RDC_EXCITATION_FREQUENCY_20K (200)


//*****************************************************************************
//
//! Values that can be passed to
//! RDC_setExcitationSignalFrequencySelect() as
//! \e FrequencySel parameter
//
//! Values that can be returned by RDC_getAdcSampleRate()
//
//*****************************************************************************
/**
 * @brief ADC Sample rate selection value
 * oversampling 16 samples to 1
 */
#define OVERSAMPLING_RATIO_16 (8)
/**
 * @brief ADC Sample rate selection value
 * oversampling 20 samples to 1
 */
#define OVERSAMPLING_RATIO_20 (10)

//*****************************************************************************
//
//! Values that can be passed to
//! RDC_enableCoreInterrupt(), RDC_disableCoreInterrupt(),
//! RDC_clearCoreInterrupt(), RDC_forceCoreInterrupt(), as
//! \e FrequencySel parameter
//
//! Values that can be returned by
//! RDC_getCoreInterruptStatus(), RDC_getCoreEnabledInterruptSources()
//
//*****************************************************************************
/**
 * @brief Interrupt Sources Macros
 *
 */
#define RDC_INTERRUPT_SOURCE_LOWAMPLITUDE_ERR                   (0x00000001U)
#define RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_COS_FAULT_ERR        (0x00000002U)
#define RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_SIN_FAULT_ERR        (0x00000004U)
#define RDC_INTERRUPT_SOURCE_SINSQCOSSQ_LO_ERR                  (0x00000008U)
#define RDC_INTERRUPT_SOURCE_SINSQCOSSQ_HI_ERR                  (0x00000010U)
#define RDC_INTERRUPT_SOURCE_COS_MULTI_ZC_ERROR_ERR             (0x00000020U)
#define RDC_INTERRUPT_SOURCE_SIN_MULTI_ZC_ERROR_ERR             (0x00000040U)
#define RDC_INTERRUPT_SOURCE_COS_NEG_ZC_PEAK_MISMATCH_ERR       (0x00000080U)
#define RDC_INTERRUPT_SOURCE_COS_POS_ZC_PEAK_MISMATCH_ERR       (0x00000100U)
#define RDC_INTERRUPT_SOURCE_SIN_NEG_ZC_PEAK_MISMATCH_ERR       (0x00000200U)
#define RDC_INTERRUPT_SOURCE_SIN_POS_ZC_PEAK_MISMATCH_ERR       (0x00000400U)
#define RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_SIN_LO_ERR            (0x00000800U)
#define RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_COS_LO_ERR            (0x00001000U)
#define RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_HI_ERR                (0x00002000U)
#define RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_LO_ERR              (0x00004000U)
#define RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_HI_ERR              (0x00008000U)
#define RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_LO_ERR               (0x00010000U)
#define RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_HI_ERR               (0x00020000U)
#define RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_LO_ERR               (0x00040000U)
#define RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_HI_ERR               (0x00080000U)
#define RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_LO_ERR             (0x00100000U)
#define RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_HI_ERR             (0x00200000U)
#define RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_LO_ERR             (0x00400000U)
#define RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_HI_ERR             (0x00800000U)
#define RDC_INTERRUPT_SOURCE_TRACK_LOCK_ERR                     (0x01000000U)

#define RDC_INTERRUPT_SOURCE_ALL (RDC_INTERRUPT_SOURCE_LOWAMPLITUDE_ERR |             \
                                  RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_COS_FAULT_ERR |  \
                                  RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_SIN_FAULT_ERR |  \
                                  RDC_INTERRUPT_SOURCE_SINSQCOSSQ_LO_ERR |            \
                                  RDC_INTERRUPT_SOURCE_SINSQCOSSQ_HI_ERR |            \
                                  RDC_INTERRUPT_SOURCE_COS_MULTI_ZC_ERROR_ERR |       \
                                  RDC_INTERRUPT_SOURCE_SIN_MULTI_ZC_ERROR_ERR |       \
                                  RDC_INTERRUPT_SOURCE_COS_NEG_ZC_PEAK_MISMATCH_ERR | \
                                  RDC_INTERRUPT_SOURCE_COS_POS_ZC_PEAK_MISMATCH_ERR | \
                                  RDC_INTERRUPT_SOURCE_SIN_NEG_ZC_PEAK_MISMATCH_ERR | \
                                  RDC_INTERRUPT_SOURCE_SIN_POS_ZC_PEAK_MISMATCH_ERR | \
                                  RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_SIN_LO_ERR |      \
                                  RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_COS_LO_ERR |      \
                                  RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_HI_ERR |          \
                                  RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_LO_ERR |        \
                                  RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_HI_ERR |        \
                                  RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_LO_ERR |         \
                                  RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_HI_ERR |         \
                                  RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_LO_ERR |         \
                                  RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_HI_ERR |         \
                                  RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_LO_ERR |       \
                                  RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_HI_ERR |       \
                                  RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_LO_ERR |       \
                                  RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_HI_ERR |       \
                                  RDC_INTERRUPT_SOURCE_TRACK_LOCK_ERR)

//*****************************************************************************
//
//! Values that can be passed to
//! RDC_getCalibrationData() as
//! \e CalAdc parameter
//
//*****************************************************************************
/**
 * @brief Macro used to specify Calibration data for ADC 0 */
#define RDC_CAL_ADC0 (0U)
/**
 * @brief Macro used to specify Calibration data for ADC 1 */
#define RDC_CAL_ADC1 (1U)

//*****************************************************************************
//
//! Values that can be passed to
//! RDC_setIdealSampleMode() as
//! \e mode parameter
//
//*****************************************************************************
/**
 * @brief This mode runs Ideal Sample Time computation on both Sin and Cos
 */
#define RDC_IDEAL_SAMPLE_TIME_MODE_0_AUTO_DETECT (0U)
/**
 * @brief This mode runs Ideal Sample Time Computation only on Sin
 */
#define RDC_IDEAL_SAMPLE_TIME_MODE_1_AUTO_DETECT_ON_SIN (1U)
/**
 * @brief This mode runs Ideal Sample Time Computation only on COS
 */
#define RDC_IDEAL_SAMPLE_TIME_MODE_2_AUTO_DETECT_ON_COS (2U)
/**
 * @brief This mode enables Manual override of Ideal Sample Time selection
 */
#define RDC_IDEAL_SAMPLE_TIME_MODE_3_AUTO_DETECT_OFF (3U)

//*****************************************************************************
//
//! Values that can be passed to
//! RDC_setTrack2Constants() as
//! \e track2Constants parameter
//
//*****************************************************************************

/**
 * @brief struct to hold the track2 constant data
 *
 */
typedef struct
{
    uint8_t kvelfilt;
} Track2Constants_t;

//*****************************************************************************
//
//! Values that can be passed to
//! RDC_getDiagnosticsSinCosOffsetDriftData(),
//! RDC_setDiagnosticsSinCosOffsetDriftData() as
//! \e Diag_Mon_SinCos_Offset_drift_data parameter
//
//*****************************************************************************
/**
 * @brief Structure to hold the control/status data for Diagnostics mentioned under
 * Monitor Sin or Cos offset drift (DOS)
 */
typedef struct
{
    int16_t offset_drift_threshold_hi;
    int16_t offset_drift_threshold_lo;
    bool offset_drift_cos_hi;
    bool offset_drift_cos_lo;
    bool offset_drift_sin_hi;
    bool offset_drift_sin_lo;
    bool offset_drift_en;
} Diag_Mon_SinCos_Offset_drift_data;

//*****************************************************************************
//
//! Values that can be passed to
//! RDC_getDiagnosticsSinCosGainDriftData(),
//! RDC_setDiagnosticsSinCosGainDriftData() as
//! \e Diag_Mon_SinCos_Gain_drift_data parameter
//
//*****************************************************************************
/**
 * @brief Structure to hold the control/status data for Diagnostics mentioned under
 * Monitor Sin or Cos Gain drift (DOS)
 */
typedef struct
{
    uint16_t gain_drift_threshold_hi;
    uint16_t gain_drift_threshold_lo;
    uint8_t gain_drift_glitch_count;
    bool gain_drift_cos_hi;
    bool gain_drift_cos_lo;
    bool gain_drift_sin_hi;
    bool gain_drift_sin_lo;
    bool gain_drift_en;
} Diag_Mon_SinCos_Gain_drift_data;

//*****************************************************************************
//
//! Values that can be passed to
//! RDC_getDiagnosticsCosPhaseDriftData(),
//! RDC_setDiagnosticsCosPhaseDriftData() as
//! \e Diag_Mon_Cos_Phase_drift_data parameter
//
//*****************************************************************************
/**
 * @brief Structure to hold the control/status data for Diagnostics mentioned under
 * Monitor Cos Phase drift (DOS)
 */
typedef struct
{
    int16_t phase_drift_threshold_hi;
    int16_t phase_drift_threshold_lo;
    uint8_t phase_drift_glitch_count;
    bool phase_drift_cos_hi;
    bool phase_drift_cos_lo;
    bool phase_drift_en;
} Diag_Mon_Cos_Phase_drift_data;

//*****************************************************************************
//
//! Values that can be passed to
//! RDC_getDiagnosticsExcFreqDegradationData(),
//! RDC_setDiagnosticsExcFreqDegradationData() as
//! \e Diag_Mon_ExcFreq_Degradataion_data parameter
//
//*****************************************************************************
/**
 * @brief Structure to hold the control/status data for Diagnostics mentioned under
 * Monitor excitation frequency degradation or loss (DOS)
 */
typedef struct
{
    uint16_t excfreqdetected_sin;
    uint16_t excfreqdetected_cos;
    uint16_t excfreqdrift_threshold_hi;
    uint16_t excfreqdrift_threshold_lo;
    uint16_t excfreq_level;
    uint8_t excfreqdrift_glitchcount;
    bool excfreqdrift_hi;
    bool excfreqdrift_cos_lo;
    bool excfreqdrift_sin_lo;
    bool excfreqdrift_en;
} Diag_Mon_ExcFreq_Degradataion_data;

//*****************************************************************************
//
//! Values that can be passed to
//! RDC_getDiagnosticsRotationalSignalIntegrityData(),
//! RDC_setDiagnosticsRotationalSignalIntegrityData() as
//! \e Diag_Mon_Rotational_Signal_Integrity_data parameter
//
//*****************************************************************************
/**
 * @brief Structure to hold the control/status data for Diagnostics mentioned under
 * Monitor rotational signal integrity (DOS)
 */
typedef struct
{
    bool cos_neg_zc_peak_mismatch_err;
    bool cos_pos_zc_peak_mismatch_err;
    bool sin_neg_zc_peak_mismatch_err;
    bool sin_pos_zc_peak_mismatch_err;
    bool sin_multi_zc_error_err;
    bool cos_multi_zc_error_err;
    uint8_t cos_multi_zc_error_count;
    uint8_t sin_multi_zc_error_count;
    uint16_t rotpeak_level;
    uint16_t rotfreq_level;
    bool zero_cross_rot_en;
} Diag_Mon_Rotational_Signal_Integrity_data;


//*****************************************************************************
//
//! Values that can be passed to
//! RDC_getDiagnosticsSignalIntegritySquareSumData(),
//! RDC_setDiagnosticsSignalIntegritySquareSumData() as
//! \e Diag_Mon_Signal_Integrity_SinSq_CosSq parameter
//
//*****************************************************************************
/**
 * @brief Structure to hold the control/status data for Diagnostics mentioned under
 * Monitor signal integrity by checking Sin2+Cos2=Constant (DOS)
 */
typedef struct
{
    uint16_t sinsqcossq_threshold_hi;
    uint16_t sinsqcossq_threshold_lo;
    uint8_t sinsqcossq_glitchcount;
    uint16_t sinsqcossq_cossq;
    uint16_t sinsqcossq_sinsq;
    bool sinsqcossq_hi;
    bool sinsqcossq_lo;
} Diag_Mon_Signal_Integrity_SinSq_CosSq;

//*****************************************************************************
//
//! Values that can be passed to
//! RDC_getDiagnosticsHighAmplitudeData(),
//! RDC_setDiagnosticsHighAmplitudeData() as
//! \e Diag_Mon_Sin_Cos_High_Amplitude parameter
//
//*****************************************************************************
/**
 * @brief Structure to hold the control/status data for Diagnostics mentioned under
 * Monitor Sin or Cos saturation or very high amplitude (DOS)
 */
typedef struct
{
    uint16_t highAmplitude_threshold;
    uint8_t highAmplitude_glitchcount;
    int16_t highAmplitude_sin_value;
    int16_t highAmplitude_cos_value;
    bool highAmplitude_sin_error;
    bool highAmplitude_cos_error;
} Diag_Mon_Sin_Cos_High_Amplitude;

//*****************************************************************************
//
//! Values that can be passed to
//! RDC_getDiagnosticsWeakAmplitudeData(),
//! RDC_setDiagnosticsWeakAmplitudeData() as
//! \e Diag_Mon_Sin_Cos_Weak_Amplitude parameter
//
//*****************************************************************************
/**
 * @brief Structure to hold the control/status data for Diagnostics mentioned under
 * Monitor weak Sin or Cos signal below a threshold (LOS)
 */
typedef struct
{
    uint16_t lowAmplitude_threshold;
    uint8_t lowAmplitude_glitchcount;
    bool lowAmplitude_error;
    int16_t lowAmplitude_sin_value;
    int16_t lowAmplitude_cos_value;
} Diag_Mon_Sin_Cos_Weak_Amplitude;


//*****************************************************************************
//
//! Values that can be passed to
//! RDC_getAdcObservationalData(),
//! RDC_setDiagnosticsWeakAmplitudeData() as
//! \e ADC_observationalData parameter
//
//*****************************************************************************
/**
 * @brief Structure to hold the Observational Data reads
 * \e int16_t \e cos_adc    - SW Observational ADC data post latch and Averaged if enabled
 * \e int16_t \e sin_adc    - SW Observational ADC data post latch and Averaged if enabled
 * \e int16_t \e cos_rec    - SW Observational ADC data post recovered
 * \e int16_t \e sin_rec    - SW Observational ADC data post recovered
 * \e int16_t \e cos_dc     - SW Observational ADC data post DC offset Correction
 * \e int16_t \e sin_dc     - SW Observational ADC data post DC offset Correction
 * \e int16_t \e cos_pgc    - SW Observational ADC data post Phase Gain Correction
 * \e int16_t \e sin_pgc    - SW Observational ADC data post Phase Gain Correction
 */
typedef struct
{
    int16_t cos_adc;
    int16_t sin_adc;
    int16_t cos_rec;
    int16_t sin_rec;
    int16_t cos_dc;
    int16_t sin_dc;
    int16_t cos_pgc;
    int16_t sin_pgc;
} ADC_observationalData;


/**
 * @brief Struct holds the Resolver Core Configurations
 * Can be passed to RDC_coreParamsInit(Core_config_t* coreParams);
 * Note : this is also called in the RDC_paramsInit(RDC_configParams* params);
 */
typedef struct
{
    bool        BpfDc_bpfEnable;
    bool        BpfDc_offsetCorrectionEnable;
    uint8_t     BpfDc_dcOffCal1;
    uint8_t     BpfDc_dcOffCal2;
    int16_t     BpfDc_manualSin;
    int16_t     BpfDc_manualCos;

    uint8_t     IdealSample_overrideValue;
    uint16_t    IdealSample_absThresholdValue;
    uint8_t     IdealSample_sampleAdjustCount;
    uint8_t     IdealSample_mode;
    bool        IdealSample_bottomSampleEnable;

    bool        Pg_estimationEnable;
    uint8_t     Pg_estimationLimit;

    bool        Pg_correctionEnable;
    bool        Pg_autoCorrectionEnable;
    uint16_t     Pg_sinGainBypassValue;
    uint16_t     Pg_cosGainBypassValue;
    int16_t     Pg_cosPhaseBypassValue;

    Track2Constants_t   track2Constants;
}Core_config_t;

/**
 * @brief Struct holds the RDC configurations
 * Can be passed to
 * RDC_paramsInit(RDC_configParams* params);
 * RDC_init(uint32_t base, RDC_configParams* params);
 *
 */
typedef struct
{
    bool adv_config;
    uint8_t Input_signalMode;
    uint8_t Input_socWidth;
    uint8_t Input_adcBurstCount;
    uint8_t Input_resolverSequencerMode;

    uint8_t     ExcFrq_freqSel;
    uint16_t    ExcFrq_phase;
    uint8_t     ExcFrq_amplitude;
    bool        ExcFrq_enableSyncIn;
    uint16_t    ExcFrq_socDelay;

    Core_config_t core0;
    Core_config_t core1;

    bool        Int_seqEnable;
    uint32_t    Int_core0Interrupts;
    uint32_t    Int_core1Interrupts;

}RDC_configParams;

/**
 * @brief Struct holds the Baseline Parameter values
 * Can be passed to RDC_BaselineParametersInit(uint32_t base);
 *
 */
typedef struct
{
    uint8_t adcParam1;
    uint8_t IdealParam2;
    uint8_t DcParam3;
    uint16_t PgParam4;
    uint8_t t2Param5;
    uint8_t t2Param6;
    uint8_t t2Param7;
    uint8_t t2Param8;
    bool t2Param9;
}baselineParameters;

//*****************************************************************************
//
//! Values that can be passed to
//! RDC_getPeakHistogramObservationalData() as,
//! \e PeakHistogram_observationalData parameter
//
//*****************************************************************************
/**
 * @brief Struct to hold the peakHistogram Buckets for Ideal Sample Calculation by SW.
 * once the auto ideal sample detection has ran for the given train limit, these histograms may
 * be read the bucket with highest value may be selected as the ideal sample bucket.
 *
 */
typedef struct
{
    uint8_t peakHistgoramBucket[20];
}PeakHistogram_observationalData;

//*****************************************************************************
// GLOBAL CONFIGURATIONS
//*****************************************************************************
    /**
     * @brief
     * sets the Start of Conversion Width for the ADC conversion
     * @param base RDC Base Address
     * @param socWidth the SoC Width for the ADC Conversion
     */
    static inline void
    RDC_setAdcSocWidth(uint32_t base, uint8_t socWidth)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_GLOBAL_CFG,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_GLOBAL_CFG) &
             ~CSL_RESOLVER_REGS_GLOBAL_CFG_SOC_WIDTH_MASK) |
                (((uint8_t)socWidth) << CSL_RESOLVER_REGS_GLOBAL_CFG_SOC_WIDTH_SHIFT));
    }
    
    /**
     * @brief Returns the configured ADC SOC Width value
     * 
     * @param base RDC Base Address
     * @return uint8_t the configured ADC SOC Width Value
     */
    static inline uint8_t
    RDC_getConfiguredAdcSocWidth(uint32_t base)
    {
        return (
            (uint8_t) ((HW_RD_REG32(base + CSL_RESOLVER_REGS_GLOBAL_CFG) & 
            CSL_RESOLVER_REGS_GLOBAL_CFG_SOC_WIDTH_MASK) >> CSL_RESOLVER_REGS_GLOBAL_CFG_SOC_WIDTH_SHIFT));
    }

    /**
     * @brief
     * sets the ADC Burst count, samples to be averaged.
     * @param base RDC Base Address
     * @param burstCount the number of Burst per Sample to be averaged.
     * Valid Burst count values are
     *          \e RDC_ADC_BURST_COUNT_DISABLE     // Burst Count is disabled
     *          \e RDC_ADC_BURST_COUNT_2           // 2    samples to be averaged
     *          \e RDC_ADC_BURST_COUNT_4           // 4    samples to be averaged
     *          \e RDC_ADC_BURST_COUNT_8           // 8    samples to be averaged
     *          \e RDC_ADC_BURST_COUNT_16          // 16   samples to be averaged
     *          \e RDC_ADC_BURST_COUNT_32          // 32   samples to be averaged
     */
    static inline void
    RDC_setAdcBurstCount(uint32_t base, uint8_t burstCount)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_GLOBAL_CFG,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_GLOBAL_CFG) &
            ~CSL_RESOLVER_REGS_GLOBAL_CFG_BURST_CNT_MASK) |
                ((uint32_t)(((uint8_t)burstCount) << CSL_RESOLVER_REGS_GLOBAL_CFG_BURST_CNT_SHIFT)));
    }

    /**
     * @brief Returns the configured Burst count value. \note this is only applicable in the Sequencer Mode 0 
     * 
     * @param base RDC Base Address
     * @return uint8_t Configured Burst Count Value. Valid return values are...
     *          \e RDC_ADC_BURST_COUNT_DISABLE     // Burst Count is disabled
     *          \e RDC_ADC_BURST_COUNT_2           // 2    samples to be averaged
     *          \e RDC_ADC_BURST_COUNT_4           // 4    samples to be averaged
     *          \e RDC_ADC_BURST_COUNT_8           // 8    samples to be averaged
     *          \e RDC_ADC_BURST_COUNT_16          // 16   samples to be averaged
     *          \e RDC_ADC_BURST_COUNT_32          // 32   samples to be averaged
     */
    static inline uint8_t
    RDC_getConfiguredAdcBurstCount(uint32_t base)
    {
        return(
            (uint8_t) ((HW_RD_REG32(base + CSL_RESOLVER_REGS_GLOBAL_CFG) & 
            CSL_RESOLVER_REGS_GLOBAL_CFG_BURST_CNT_MASK) >> CSL_RESOLVER_REGS_GLOBAL_CFG_BURST_CNT_SHIFT)
        );
    }

    /**
     * @brief Enable Single Ended Mode of operation
     * - default is Differential Mode of operation
     *
     * @param base RDC Base Address
     */
    static inline void
    RDC_enableAdcSingleEndedMode(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_GLOBAL_CFG,
            (HW_RD_REG32(
                base + CSL_RESOLVER_REGS_GLOBAL_CFG) &
            ~CSL_RESOLVER_REGS_GLOBAL_CFG_SINGLE_EN_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_GLOBAL_CFG_SINGLE_EN_SHIFT)));
    }

    /**
     * @brief Disable Single Ended Mode of operation
     * - default is Differential Mode of operation
     *
     * @param base RDC Base Address
     */
    static inline void
    RDC_disableAdcSingleEndedMode(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_GLOBAL_CFG,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_GLOBAL_CFG) &
             ~CSL_RESOLVER_REGS_GLOBAL_CFG_SINGLE_EN_MASK));
    }

    /**
     * @brief Returns if Single ended mode of sampling is enabled for the ADCs
     * 
     * @param base 
     * @return true Single ended mode of sampling is enabled and default mode of Sifferential ended sampling is disabled
     * @return false Single ended mode of sampling is disbaled and default mode of Differential ended sampling is enabled
     */
    static inline bool
    RDC_isAdcSingleEndedModeEnabled(uint32_t base)
    {
        return (
            (HW_RD_REG32(base + CSL_RESOLVER_REGS_GLOBAL_CFG) & CSL_RESOLVER_REGS_GLOBAL_CFG_SINGLE_EN_MASK) == CSL_RESOLVER_REGS_GLOBAL_CFG_SINGLE_EN_MASK
        );
    }


    /**
     * @brief returns Sequencer Operational Mode
     *          Valid values are
     *   \e RDC_SEQUENCER_MODE_0    -   ADC0 SAMPLES SIN AND ADC1 SAMPLES COSINE PARALLELLY FOR CORE 0
     *   \e RDC_SEQUENCER_MODE_1    -   ADC0 SAMPLES SIN AND COSINE SEQUENTIALLY FOR CORE 0
     *   \e RDC_SEQUENCER_MODE_2    -   ADC0 SAMPLES SIN0, COS0, COS1, SIN1 SEQUENTIALLY, TO BE AVERAGED FOR CORE 0
     *   \e RDC_SEQUENCER_MODE_3    -   ADC0 SAMPLES SIN AND ADC1 SAMPLES COS PARALLELLY FOR CORE 0
     *                                  FOLLOWED BY
     *                                  ADC0 SAMPLES SIN AND ADC1 SAMPLES COS PARALLELLY FOR CORE 1
     *   \e RDC_SEQUENCER_MODE_4    -   ADC0 SAMPLES SIN FOR CORE 0 AND ADC1 SAMPLES SIN FOR CORE 1 PARALLELLY
     *                                  FOLLOWED BY
     *                                  ADC0 SAMPLES COS FOR CORE 0 AND ADC1 SAMPLES COS FOR CORE 2
     *   \e RDC_SEQUENCER_MODE_5    -   ADC0/1 PARALLELLY SAMPLE
     *                                  SIN0, COS0, COS1, SIN1 SAMPLES SEQUENTIALLY
     *                                  FOR CORE0/1. BOTH SIN(COS) SAMPLES FOR EACH CORE WILL BE AVERAGED.
     * @param base RDC Base Address
     * @return uint8_t 4-bit mode value
     */
    static inline uint32_t
    RDC_getAdcSequencerOperationalMode(uint32_t base)
    {
        return ((HW_RD_REG32(
                     base + CSL_RESOLVER_REGS_GLOBAL_CFG) &
                 CSL_RESOLVER_REGS_GLOBAL_CFG_MODE_MASK) >>
                CSL_RESOLVER_REGS_GLOBAL_CFG_MODE_SHIFT);
    }

    /**
     * @brief
     * sets Sequencer Operational Mode
     * Valid values are
     *   \e RDC_SEQUENCER_MODE_0    -   ADC0 SAMPLES SIN AND ADC1 SAMPLES COSINE PARALLELLY FOR CORE 0
     *   \e RDC_SEQUENCER_MODE_1    -   ADC0 SAMPLES SIN AND COSINE SEQUENTIALLY FOR CORE 0
     *   \e RDC_SEQUENCER_MODE_2    -   ADC0 SAMPLES SIN0, COS0, COS1, SIN1 SEQUENTIALLY, TO BE AVERAGED FOR CORE 0
     *   \e RDC_SEQUENCER_MODE_3    -   ADC0 SAMPLES SIN AND ADC1 SAMPLES COS PARALLELLY FOR CORE 0
     *                                  FOLLOWED BY
     *                                  ADC0 SAMPLES SIN AND ADC1 SAMPLES COS PARALLELLY FOR CORE 1
     *   \e RDC_SEQUENCER_MODE_4    -   ADC0 SAMPLES SIN FOR CORE 0 AND ADC1 SAMPLES SIN FOR CORE 1 PARALLELLY
     *                                  FOLLOWED BY
     *                                  ADC0 SAMPLES COS FOR CORE 0 AND ADC1 SAMPLES COS FOR CORE 2
     *   \e RDC_SEQUENCER_MODE_5    -   ADC0/1 PARALLELLY SAMPLE
     *                                  SIN0, COS0, COS1, SIN1 SAMPLES SEQUENTIALLY
     *                                  FOR CORE0/1. BOTH SIN(COS) SAMPLES FOR EACH CORE WILL BE AVERAGED.
     * @param base RDC Base Address
     * @param operationalMode RDC sequencer operational mode
     */
    static inline void
    RDC_setAdcSequencerOperationalMode(uint32_t base, uint8_t operationalMode)
    {
        DebugP_assert(
            (operationalMode >= RDC_SEQUENCER_MODE_0) &&
            (operationalMode <= RDC_SEQUENCER_MODE_5));

        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_GLOBAL_CFG,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_GLOBAL_CFG) &
             ~CSL_RESOLVER_REGS_GLOBAL_CFG_MODE_MASK) |
                ((uint32_t)((operationalMode) << CSL_RESOLVER_REGS_GLOBAL_CFG_MODE_SHIFT)));
    }
    /**
     * @brief
     * Enables the Resolver Operation
     * \note this API has to be called only after all the configurations are complete.
     * @param base RDC Base Address
     */
    static inline void
    RDC_enableResolver(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_GLOBAL_CFG,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_GLOBAL_CFG) &
             ~CSL_RESOLVER_REGS_GLOBAL_CFG_MASTER_EN_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_GLOBAL_CFG_MASTER_EN_SHIFT)));
    }
    /**
     * @brief
     * Disables the Resolver Operation
     * \note this API has to be called before any of the configurations are done.
     * @param base RDC Base Address
     */
    static inline void
    RDC_disableResolver(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_GLOBAL_CFG,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_GLOBAL_CFG) |
             CSL_RESOLVER_REGS_GLOBAL_CFG_MASTER_EN_MASK) &
                ~((uint32_t)((1U) << CSL_RESOLVER_REGS_GLOBAL_CFG_MASTER_EN_SHIFT)));
    }

    /**
     * @brief 
     * Returns if Resolver is enabled
     * @param base 
     * @return true Resolver is enabled
     * @return false Resolver is disabled
     */
    static inline bool
    RDC_isResolverEnabled(uint32_t base)
    {
        return (
            (HW_RD_REG32(base + CSL_RESOLVER_REGS_GLOBAL_CFG) & CSL_RESOLVER_REGS_GLOBAL_CFG_MASTER_EN_MASK) == CSL_RESOLVER_REGS_GLOBAL_CFG_MASTER_EN_MASK
        );
    }

//*****************************************************************************
// EXCITATION FREQUENCY AND SAMPLING CONFIGURATIONS
//*****************************************************************************

    /**
     * @brief Sets the Phase value for the Excitation Signal.
     * Phase values in the range [RDC_EXCITATION_FREQUENCY_MIN_PHASE, RDC_EXCITATION_FREQUENCY_MAX_PHASE] can be mapped to 0-360 deg
     *
     * @param base RDC Base Address
     * @param phase Phase values can be from the range [RDC_EXCITATION_FREQUENCY_MIN_PHASE, RDC_EXCITATION_FREQUENCY_MAX_PHASE]
     */
    static inline void
    RDC_setExcitationSignalPhase(uint32_t base, uint16_t phase)
    {
        DebugP_assert(
            (phase >= RDC_EXCITATION_FREQUENCY_MIN_PHASE) && (phase <= RDC_EXCITATION_FREQUENCY_MAX_PHASE));
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1) &
             ~CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_PHASE_CFG_MASK) |
                ((uint32_t)(phase << CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_PHASE_CFG_SHIFT)));
    }

    /**
     * @brief Returns the Phase Value programmed for Excitation signal
     *
     * @param base RDC Base Address
     * @return uint32_t Phase Value programmed for Excitation signal
     * ranges in [RDC_EXCITATION_FREQUENCY_MIN_PHASE, RDC_EXCITATION_FREQUENCY_MAX_PHASE], can be mapped to 0-360 deg
     */
    static inline uint32_t
    RDC_getExcitationSignalPhase(uint32_t base)
    {
        return (
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1) &
             CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_PHASE_CFG_MASK) >>
            CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_PHASE_CFG_SHIFT);
    }

    /**
     * @brief Sets the Excitation frequency value from the selected values.
     *
     * @param base RDC Base Address
     * @param FrequencySel Valid inputs are
     *  \e RDC_EXCITATION_FREQUENCY_5K    - selects for 5KHz Excitation Sine Frequency
     *  \e RDC_EXCITATION_FREQUENCY_10K   - selects for 10KHz Excitation Sine Frequency
     *  \e RDC_EXCITATION_FREQUENCY_20K   - selects for 20KHz Excitation Sine Frequency
     */
    static inline void
    RDC_setExcitationSignalFrequencySelect(uint32_t base, uint8_t FrequencySel)
    {
        DebugP_assert(
            (FrequencySel == RDC_EXCITATION_FREQUENCY_5K) ||
            (FrequencySel == RDC_EXCITATION_FREQUENCY_10K) ||
            (FrequencySel == RDC_EXCITATION_FREQUENCY_20K));
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1) &
             ~CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_SEL_MASK) |
                ((uint32_t)(FrequencySel << CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_SEL_SHIFT)));
    }

    /**
     * @brief Returns the selected Excitation Signal Frequency select
     *
     * @param base RDC Base Address
     * @return uint8_t Excitation Frequency selected
     *  \e RDC_EXCITATION_FREQUENCY_5K    - selected for 5KHz Excitation Sine Frequency
     *  \e RDC_EXCITATION_FREQUENCY_10K   - selected for 10KHz Excitation Sine Frequency
     *  \e RDC_EXCITATION_FREQUENCY_20K   - selected for 20KHz Excitation Sine Frequency.
     */
    static inline uint32_t
    RDC_getExcitationSignalFrequencySelect(uint32_t base)
    {
        return (
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1) &
             CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_SEL_MASK) >>
            CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_SEL_SHIFT);
    }

    /**
     * @brief Gets the ADC Sampling Ratio
     *
     * @param base RDC Base Address
     * @return uint32_t Valid values are \e OVERSAMPLING_RATIO_16
     */
    static inline uint32_t
    RDC_getAdcSampleRate(uint32_t base)
    {
        return (
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1) &
             CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_ADC_SAMPLE_RATE_MASK) >>
            CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_ADC_SAMPLE_RATE_SHIFT);
    }

    /**
     * @brief Enables the Excitation Signal Sync In.
     *
     * @param base RDC Base Address
     */
    static inline void
    RDC_enableExcitationSignalSyncIn(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2) &
             ~CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EN_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EN_SHIFT)));
    }

    /**
     * @brief Disables the Excitation Signal Sync In.
     *
     * @param base RDC Base Address
     */
    static inline void
    RDC_disableExcitationSignalSyncIn(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2) |
             CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EN_MASK) &
                ~((uint32_t)((1U) << CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EN_SHIFT)));
    }

    /**
     * @brief Returns if the Excitation Sync-in Signal is enabled
     * 
     * @param base RDC Base Address
     * @return true     Sync-in is enabled 
     * @return false    Sync-in is disabled
     */
    static inline bool
    RDC_isExcitationSignalSyncInEnabled(uint32_t base)
    {   
        return((HW_RD_REG32(base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2)  & 
        CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EN_MASK) == CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EN_MASK);
    }

    /**
     * @brief returns if there is a sync in event after the RDC_enableResolver() has been called
     * once this returns non-zero,
     * RDC_clearExcitationSignalEventStatus(),
     * RDC_disableResolver(),
     * RDC_enableResolver() needs to be called to rearm.
     *
     * @param base RDC Base Address
     * @return uint32_t sync in event status after resolver enable.
     */
    static inline uint32_t
    RDC_getExcitationSignalEventStatus(uint32_t base)
    {
        return (
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2) &
             CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EVENT_MASK) >>
            CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EVENT_SHIFT);
    }

    /**
     * @brief Clears SyncIn event status
     *
     * @param base RDC Base Address
     */
    static inline void
    RDC_clearExcitationSignalEventStatus(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2) &
             ~CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EVENT_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EVENT_SHIFT)));
    }

    /**
     * @brief Returns the latched value of the last pwm_sync_in rise event of the pwm
     * phase. this is updated on every pwm_sync_in event
     *
     * @param base RDC Base Address
     * @return uint32_t Excitation phase info.
     */
    static inline uint32_t
    RDC_getExcitationSignalPhaseInfo(uint32_t base)
    {
        return (
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2) &
             CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_PHASE_INFO_MASK) >>
            CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_PHASE_INFO_SHIFT);
    }

    /**
     * @brief Sets the SOC Delay from the PWM Exciation Signal.
     *
     * @param base RDC Base Address
     * @param socDelay in steps of 800KHz
     */
    static inline void
    RDC_setExcitationSignalSocDelay(uint32_t base, uint16_t socDelay)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2) &
             ~CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_TO_SOC_DLY_START_MASK) |
                ((uint32_t)(socDelay << CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_TO_SOC_DLY_START_SHIFT)));
    }

    /**
     * @brief Returns Configured Excitation Signal SOC Delay
     * 
     * @param base RDC base address
     * @return uint16_t 
     */
    static inline uint16_t
    RDC_getConfiguredExcitationSignalSocDelay(uint32_t base)
    {
        return(
            (uint16_t) ((HW_RD_REG32(base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2) &
             CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_TO_SOC_DLY_START_MASK) >> CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_TO_SOC_DLY_START_SHIFT)
        );
    }



    /**
     * @brief set the Amplitude control for the excitation signal
     *
     * @param base RDC Base Address
     * @param amplitude 0-249 mapping 0-100%
     */
    static inline void
    RDC_setExcitationSignalAmplitudeControl(uint32_t base, uint8_t amplitude)
    {
        DebugP_assert(amplitude <= RDC_MAX_EXCITATION_AMPLITUDE);
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3) &
             ~CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3_EXC_AMP_CTRL_MASK) |
                ((uint32_t)(amplitude << CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3_EXC_AMP_CTRL_SHIFT)));
    }

    /**
     * @brief returns the Amplitude control for the excitation signal
     *
     * @param base RDC Base Address
     * @return 0-249 mapping 0-100%
     */
    static inline uint32_t
    RDC_getExcitationSignalAmplitudeControl(uint32_t base)
    {
        return (
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3) &
             CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3_EXC_AMP_CTRL_MASK) >>
            CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3_EXC_AMP_CTRL_SHIFT);
    }

    //*****************************************************************************
    // INTERRUPT CONFIGURATIONS
    //*****************************************************************************

    /**
     * @brief Enable Sequencer Error Interrupt
     *
     * @param base RDC Base Address
     */
    static inline void
    RDC_enableSequencerInterrupt(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_IRQSTATUS_SYS,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_IRQSTATUS_SYS) &
             ~CSL_RESOLVER_REGS_IRQSTATUS_SYS_SEQ_ERR_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_IRQSTATUS_SYS_SEQ_ERR_SHIFT)));
    }

    /**
     * @brief Disable Sequencer Error Interrupt
     *
     * @param base RDC Base Address
     */
    static inline void
    RDC_disableSequencerInterrupt(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS) &
             ~CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_SEQ_ERR_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_SEQ_ERR_SHIFT)));
    }

    /**
     * @brief Returns if the sequencer error interrupt is enabled
     * 
     * @return true Sequencer error interrupt is enabled
     * @return false Sequencer error interrutp is disabled
     */
    static inline bool
    RDC_isSequencerInterruptEnabled(uint32_t base)
    {
        return((HW_RD_REG32(base + CSL_RESOLVER_REGS_IRQSTATUS_SYS) & CSL_RESOLVER_REGS_IRQSTATUS_SYS_SEQ_ERR_MASK) == CSL_RESOLVER_REGS_IRQSTATUS_SYS_SEQ_ERR_MASK);
    }

    /**
     * @brief Clear the Sequencer Error Interrupt status
     *
     * @param base RDC Base Address
     */
    static inline void
    RDC_clearSequencerInterrupt(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_IRQSTATUS_SYS,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_IRQSTATUS_SYS) &
             ~CSL_RESOLVER_REGS_IRQSTATUS_SYS_SEQ_ERR_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_IRQSTATUS_SYS_SEQ_ERR_SHIFT)));
    }

    /**
     * @brief Force the Sequencer Error Interrupt
     *
     * @param base RDC Base Address
     */
    static inline void
    RDC_forceSequencerInterrupt(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS) &
             ~CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_SEQ_ERR_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_SEQ_ERR_SHIFT)));
    }

    /**
     * @brief Returns Sequencer interrupt Status
     *
     * @param base RDC Base Address
     */
    static inline uint32_t
    RDC_getSequencerInterruptStatus(uint32_t base)
    {
        return (
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS) &
             ~CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_SEQ_ERR_MASK) |
            ((uint32_t)((1U) << CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_SEQ_ERR_SHIFT)));
    }

    /**
     * @brief enable Core Interrupt
     *
     * @param base RDC Base Address
     * @param ResolverCore Resolver Core
     * @param interruptSource Resolver Core Interrupt
     */
    static inline void
    RDC_enableCoreInterrupt(uint32_t base, uint32_t ResolverCore, uint32_t interruptSource)
    {
        DebugP_assert((ResolverCore == RDC_RESOLVER_CORE0) || (ResolverCore == RDC_RESOLVER_CORE1));
        DebugP_assert((interruptSource & (~((uint32_t)RDC_INTERRUPT_SOURCE_ALL))) == 0);

        uint32_t regOffset = CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0 + (ResolverCore * RDC_CORE_OFFSET);

        HW_WR_REG32(
            base + regOffset,
            HW_RD_REG32(
                base + regOffset) |
                ((uint32_t)interruptSource));
    }

    /**
     * @brief returns enabled Interrupt Sources
     *
     * @param base  RDC Base Address
     * @param ResolverCore Resolver Core
     * @return uint32_t
     */
    static inline uint32_t
    RDC_getCoreEnabledInterruptSources(uint32_t base, uint8_t ResolverCore)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0 + (ResolverCore * RDC_CORE_OFFSET);
        return (
            HW_RD_REG32(
                base + regOffset));
    }

    /**
     * @brief Disable Core Interrupt
     *
     * @param base RDC Base Address
     * @param ResolverCore Resolver Core
     * @param interruptSource Resolver Core Interrupt
     */
    static inline void
    RDC_disableCoreInterrupt(uint32_t base, uint32_t ResolverCore, uint32_t interruptSource)
    {
        DebugP_assert((ResolverCore == RDC_RESOLVER_CORE0) || (ResolverCore == RDC_RESOLVER_CORE1));
        DebugP_assert((interruptSource & (~((uint32_t)RDC_INTERRUPT_SOURCE_ALL))) == 0);

        uint32_t regOffset = CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0 + (ResolverCore * RDC_CORE_OFFSET);

        HW_WR_REG32(
            base + regOffset,
            HW_RD_REG32(
                base + regOffset) |
                ((uint32_t)interruptSource));
    }

    /**
     * @brief Clear the Core Interrupt status
     *
     * @param base RDC Base Address
     * @param ResolverCore Resolver Core
     * @param interruptSource Resolver Core Interrupt
     */
    static inline void
    RDC_clearCoreInterrupt(uint32_t base, uint32_t ResolverCore, uint32_t interruptSource)
    {
        DebugP_assert((ResolverCore == RDC_RESOLVER_CORE0) || (ResolverCore == RDC_RESOLVER_CORE1));
        DebugP_assert((interruptSource & (~((uint32_t)RDC_INTERRUPT_SOURCE_ALL))) == 0);

        uint32_t regOffset = CSL_RESOLVER_REGS_IRQSTATUS_SYS_0 + (ResolverCore * RDC_CORE_OFFSET);

        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~((uint32_t)RDC_INTERRUPT_SOURCE_ALL)) |
                interruptSource);
    }

    /**
     * @brief Force the Core Interrupt
     *
     * @param base RDC Base Address
     * @param ResolverCore Resolver Core
     * @param interruptSource Resolver Core Interrupt
     */
    static inline void
    RDC_forceCoreInterrupt(uint32_t base, uint32_t ResolverCore, uint32_t interruptSource)
    {
        DebugP_assert((ResolverCore == RDC_RESOLVER_CORE0) || (ResolverCore == RDC_RESOLVER_CORE1));
        DebugP_assert((interruptSource & (~((uint32_t)RDC_INTERRUPT_SOURCE_ALL))) == 0);

        uint32_t regOffset = CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0 + (ResolverCore * RDC_CORE_OFFSET);

        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~((uint32_t)RDC_INTERRUPT_SOURCE_ALL)) |
                interruptSource);
    }

    /**
     * @brief Returns Core interrupt Status
     *
     * @param base RDC Base Address
     * @param ResolverCore Resovler Core
     */
    static inline uint32_t
    RDC_getCoreInterruptStatus(uint32_t base, uint32_t ResolverCore)
    {
        DebugP_assert((ResolverCore == RDC_RESOLVER_CORE0) || (ResolverCore == RDC_RESOLVER_CORE1));

        uint32_t regOffset = CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0 + (ResolverCore * RDC_CORE_OFFSET);

        return (
            (HW_RD_REG32(
                 base + regOffset) &
             ((uint32_t)RDC_INTERRUPT_SOURCE_ALL)));
    }
    
    /**
     * @brief Returns Core interrupt sources
     *
     * @param base RDC Base Address
     * @param ResolverCore Resovler Core
     */
    static inline uint32_t
    RDC_getCoreInterruptSources(uint32_t base, uint32_t ResolverCore)
    {
        DebugP_assert((ResolverCore == RDC_RESOLVER_CORE0) || (ResolverCore == RDC_RESOLVER_CORE1));

        uint32_t regOffset = CSL_RESOLVER_REGS_IRQSTATUS_SYS_0 + (ResolverCore * RDC_CORE_OFFSET);

        return (
            (HW_RD_REG32(
                 base + regOffset) &
             ((uint32_t)RDC_INTERRUPT_SOURCE_ALL)));
    }

    //*****************************************************************************
    // CALIBRATION CONFIGURATIONS
    //*****************************************************************************

    /**
     * @brief Returns the Calibration Status.
     *
     * @param base RDC Base Address
     * @return bool Calibration Status
     */
    static inline bool
    RDC_getCalibrationStatus(uint32_t base)
    {
        return (
            (((HW_RD_REG32(
                   base + CSL_RESOLVER_REGS_CAL_CFG) &
               CSL_RESOLVER_REGS_CAL_CFG_CAL_DONE_MASK) >>
              CSL_RESOLVER_REGS_CAL_CFG_CAL_DONE_SHIFT) &
             (1U)) != 0U);
    }

    /**
     * @brief Clears Calibration Status for re-enabling Calibration Sequence
     *
     * @param base RDC Base Address
     */
    static inline void
    RDC_clearCalibrationStatus(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_CAL_CFG,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_CAL_CFG) &
             ~CSL_RESOLVER_REGS_CAL_CFG_CAL_DONE_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_CAL_CFG_CAL_DONE_SHIFT)));
    }

    /**
     * @brief Selects Calibration Channel for Cal sequence
     *
     * @param base RDC Base Address
     * @param calChannel channel to read from. valid values are the following
     *      -   \e RDC_ADC_CAL_CHANNEL0         - ADC_RxAIN0
     *      -   \e RDC_ADC_CAL_CHANNEL1         - ADC_RxAIN1
     *      -   \e RDC_ADC_CAL_CHANNEL2         - ADC_RxAIN2
     *      -   \e RDC_ADC_CAL_CHANNEL3         - ADC_RxAIN3
     *      -   \e RDC_ADC_CAL_CHANNEL_CAL0     - ADC_CAL0
     *      -   \e RDC_ADC_CAL_CHANNEL_CAL1     - ADC_CAL1
     *      -   \e RDC_ADC_CAL_CHANNEL_CAL2     - ADC_CAL2
     *      -   \e RDC_ADC_CAL_CHANNEL_CAL3     - ADC_CAL3
     */
    static inline void
    RDC_selectCalibrationChannel(uint32_t base, uint8_t calChannel)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_CAL_CFG,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_CAL_CFG) &
             ~CSL_RESOLVER_REGS_CAL_CFG_CAL_CHSEL_MASK) |
                ((uint32_t)(calChannel << CSL_RESOLVER_REGS_CAL_CFG_CAL_CHSEL_SHIFT)));
    }

    /**
     * @brief Enables ADC Calibration
     *
     * @param base RDC Base Address
     */
    static inline void
    RDC_enableCalibration(uint32_t base)
    {
        HW_WR_REG32(
            base + CSL_RESOLVER_REGS_CAL_CFG,
            (HW_RD_REG32(
                 base + CSL_RESOLVER_REGS_CAL_CFG) &
             ~CSL_RESOLVER_REGS_CAL_CFG_CAL_EN_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_CAL_CFG_CAL_EN_SHIFT)));
    }

    /**
     * @brief Returns the CAL ADC data for given ADC, if the mode permits.
     *
     * @param base RDC Base Address
     * @param CalAdc valid values are \e RDC_CAL_ADC0, \e RDC_CAL_ADC1
     * @return uint32_t CAL ADC DATA
     */
    static inline uint16_t
    RDC_getCalibrationData(uint32_t base, uint8_t CalAdc)
    {
        uint32_t regData = HW_RD_REG32(
                               base + CSL_RESOLVER_REGS_CAL_OBS) &
                           (CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC0_DATA_MASK |
                            CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC1_DATA_MASK);
        if (CalAdc == RDC_CAL_ADC0)
        {
            return ((uint16_t)((regData & CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC0_DATA_MASK) >>
                    CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC0_DATA_SHIFT));
        }
        return ((uint16_t)((regData & CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC1_DATA_MASK) >>
                CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC1_DATA_SHIFT));
    }

    //*****************************************************************************
    // DC OFFSET AND BAND PASS FILTER CONFIGURATIONS
    //*****************************************************************************

    /**
     * @brief Sets the DC Offset Coefficients coef1, coef2
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param coef1 4 bit unsigned value.
     * @param coef2 4 bit unsigned value.
     */
    static inline void
    RDC_setDcOffsetCalCoef(uint32_t base, uint8_t core, uint8_t coef1, uint8_t coef2)
    {
        DebugP_assert((core == RDC_RESOLVER_CORE0) || (core == RDC_RESOLVER_CORE1));
        uint32_t regOffset = CSL_RESOLVER_REGS_DC_OFF_CFG1_0 + (core * RDC_CORE_OFFSET);
        uint32_t mask = (CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF1_MASK |
                         CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF2_MASK);

        uint32_t value = (coef1 << CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF1_SHIFT) |
                         (coef2 << CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF2_SHIFT);
        value &= mask;

        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             (~mask)) |
                value);
    }

    /**
     * @brief Returns the Configured Coefficient Values for DC Offset Estimation and correction logic
     * 
     * @param base RDC base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param coef1 Configured Coef values.
     * @param coef2 Configured Coef values.
     */
    static inline void
    RDC_getConfiguredDcOffsetCalCoef(uint32_t base, uint8_t core, uint8_t* coef1, uint8_t* coef2)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DC_OFF_CFG1_0 + (core * RDC_CORE_OFFSET);
        uint32_t value = HW_RD_REG32(base+regOffset);

        *coef1  = (uint8_t)((value & CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF1_MASK) >> CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF1_SHIFT);
        *coef2  = (uint8_t)((value & CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF2_MASK) >> CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF2_SHIFT);
    }

    /**
     * @brief enables Band Pass Filter before DC Offset logic.
     *
     * \note the BPF is only supported for oversampling ratio of 20
     * \note The DC Offset logic can be bypassed if BPF is enabled
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_enableBPF(uint32_t base, uint8_t core)
    {
        DebugP_assert((core == RDC_RESOLVER_CORE0) || (core == RDC_RESOLVER_CORE1));
        uint32_t regOffset = CSL_RESOLVER_REGS_DC_OFF_CFG1_0 + (core * RDC_CORE_OFFSET);

        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_DC_OFF_CFG1_0_BANDPASSFILTER_ON_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_DC_OFF_CFG1_0_BANDPASSFILTER_ON_SHIFT)));
    }

    /**
     * @brief Disables Band Pass Filter Logic before DC Offset logic.
     *
     * @param base RDC Base Address
     * @param core valid values are RDC_RESOLVER_CORE0, RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_disableBPF(uint32_t base, uint8_t core)
    {
        DebugP_assert((core == RDC_RESOLVER_CORE0) || (core == RDC_RESOLVER_CORE1));
        uint32_t regOffset = CSL_RESOLVER_REGS_DC_OFF_CFG1_0 + (core * RDC_CORE_OFFSET);

        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) |
             CSL_RESOLVER_REGS_DC_OFF_CFG1_0_BANDPASSFILTER_ON_MASK) &
                ~((uint32_t)((1U) << CSL_RESOLVER_REGS_DC_OFF_CFG1_0_BANDPASSFILTER_ON_SHIFT)));
    }

    /**
     * @brief Returns if the BPF is enabled for given RDC Core
     * 
     * @param base RDC Base Address
     * @param core valid values are RDC_RESOLVER_CORE0, RDC_RESOLVER_CORE1
     * @return true BPF is enabled
     * @return false BPF is disabled
     */
    static inline bool
    RDC_isBPFEnabled(uint32_t base, uint8_t core)
    {
        DebugP_assert((core == RDC_RESOLVER_CORE0) || (core == RDC_RESOLVER_CORE1));
        uint32_t regOffset = CSL_RESOLVER_REGS_DC_OFF_CFG1_0 + (core * RDC_CORE_OFFSET);

        return((HW_RD_REG32(base + regOffset) & CSL_RESOLVER_REGS_DC_OFF_CFG1_0_BANDPASSFILTER_ON_MASK) == CSL_RESOLVER_REGS_DC_OFF_CFG1_0_BANDPASSFILTER_ON_MASK);
    }

    /**
     * @brief Disbales Auto Offset correction from the estimated values
     * Enables DC Offset Manual Correction logic
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_disableDcOffsetAutoCorrection(uint32_t base, uint8_t core)
    {
        DebugP_assert((core == RDC_RESOLVER_CORE0) || (core == RDC_RESOLVER_CORE1));
        uint32_t regOffset = CSL_RESOLVER_REGS_DC_OFF_CFG1_0 + (core * RDC_CORE_OFFSET);

        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_CORR_ON_MASK));
    }
    
    /**
     * @brief Returns if DC Auto Offset Correction is enabled for give RDC Core
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return true Auto Offset Correction is enabled
     * @return false Auto Offset Correction is disabled
     */
    static inline bool
    RDC_isDcOffsetAutoCorrectionEnabled(uint32_t base, uint8_t core)
    {
        DebugP_assert((core == RDC_RESOLVER_CORE0) || (core == RDC_RESOLVER_CORE1));
        uint32_t regOffset = CSL_RESOLVER_REGS_DC_OFF_CFG1_0 + (core * RDC_CORE_OFFSET);

        return(
            (HW_RD_REG32(base + regOffset) &
             CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_CORR_ON_MASK) == CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_CORR_ON_MASK
        );
    }

    /**
     * @brief Enables Auto DC Offset Correction from the estimated values
     * Disables DC Offset Manual Correction logic and enabled the Auto correction logic
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_enableDcOffsetAutoCorrection(uint32_t base, uint8_t core)
    {
        DebugP_assert((core == RDC_RESOLVER_CORE0) || (core == RDC_RESOLVER_CORE1));
        uint32_t regOffset = CSL_RESOLVER_REGS_DC_OFF_CFG1_0 + (core * RDC_CORE_OFFSET);

        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) |
             CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_CORR_ON_MASK));
    }

    /**
     * @brief Sets the Sin, Cosine Manual Correction values for the Dc Offset block
     * in the given resolver core.
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param sin 16 bit signed int value for sine manual offset
     * @param cos 16 bit signed int value for cosine manual offset
     */
    static inline void
    RDC_setDcOffsetManualCorrectionValue(uint32_t base, uint8_t core, int16_t sin, int16_t cos)
    {
        DebugP_assert((core == RDC_RESOLVER_CORE0) || (core == RDC_RESOLVER_CORE1));
        uint32_t regOffset = CSL_RESOLVER_REGS_DC_OFF_CFG2_0 + (core * RDC_CORE_OFFSET);
        uint32_t mask = (CSL_RESOLVER_REGS_DC_OFF_CFG2_0_SIN_MAN_OFF_ADJ_MASK |
                         CSL_RESOLVER_REGS_DC_OFF_CFG2_0_COS_MAN_OFF_ADJ_MASK);

        uint32_t value = (((uint32_t)((uint16_t)sin)) << CSL_RESOLVER_REGS_DC_OFF_CFG2_0_SIN_MAN_OFF_ADJ_SHIFT) |
                         (((uint32_t)((uint16_t)cos)) << CSL_RESOLVER_REGS_DC_OFF_CFG2_0_COS_MAN_OFF_ADJ_SHIFT);
        value &= mask;

        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             (~mask)) |
                value);
    }

    /**
     * @brief Gets the Sin, Cosine Manual Correction values for the Dc Offset block
     * in the given resolver core.
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * 
     * @param sin pointer to Return Sin Manual Correction Value Configured 
     * @param cos pointer to Return Cos Manual Correction Value Configured 
     */
    static inline void
    RDC_getDcOffsetManualCorrectionValue(uint32_t base, uint8_t core, int16_t* sin, int16_t* cos)
    {
        DebugP_assert((core == RDC_RESOLVER_CORE0) || (core == RDC_RESOLVER_CORE1));
        uint32_t regOffset = CSL_RESOLVER_REGS_DC_OFF_CFG2_0 + (core * RDC_CORE_OFFSET);
        uint32_t mask = (CSL_RESOLVER_REGS_DC_OFF_CFG2_0_SIN_MAN_OFF_ADJ_MASK |
                         CSL_RESOLVER_REGS_DC_OFF_CFG2_0_COS_MAN_OFF_ADJ_MASK);

        uint32_t value = HW_RD_REG32(base + regOffset) & mask;

        *sin = (int16_t) (value >> CSL_RESOLVER_REGS_DC_OFF_CFG2_0_SIN_MAN_OFF_ADJ_SHIFT);
        *cos = (int16_t) (value >> CSL_RESOLVER_REGS_DC_OFF_CFG2_0_COS_MAN_OFF_ADJ_SHIFT);
    }

    /**
     * @brief returns DC OFFSET estimation values
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param sinCosValue Valid inputs are
     *                  - \e RDC_DC_OFFSET_SIN_ESTIMATION
     *                  - \e RDC_DC_OFFSET_COS_ESTIMATION
     * @return int16_t
     */
    static inline int16_t
    RDC_getDcOffsetEstimatedValues(uint32_t base, uint8_t core, uint8_t sinCosValue)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DC_OFF0 + (core * RDC_CORE_OFFSET);
        uint32_t mask = CSL_RESOLVER_REGS_DC_OFF0_SIN_OFFSET_MASK | CSL_RESOLVER_REGS_DC_OFF0_COS_OFFSET_MASK;
        DebugP_assert((sinCosValue == RDC_DC_OFFSET_SIN_ESTIMATION) || (sinCosValue == RDC_DC_OFFSET_COS_ESTIMATION));
        return ((int16_t)(HW_RD_REG32(
                              base + regOffset) &
                          mask) >>
                (sinCosValue * CSL_RESOLVER_REGS_DC_OFF0_COS_OFFSET_SHIFT));
    }

    //*****************************************************************************
    // Ideal Sample Time Configurations
    //*****************************************************************************

    /**
     * @brief sets the Override value for the Ideal Sample Time selection.
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param overrideValue uint8_t value
     */
    static inline void
    RDC_overrideIdealSampleTime(uint32_t base, uint8_t core, uint8_t overrideValue)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_SAMPLE_CFG1_0 + (core * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_OVR_MASK) |
                ((uint32_t)(overrideValue << CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_OVR_SHIFT)));
    }

    /**
     * @brief Returns the Configured Override value for the Ideal Sample Time
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return uint8_t Returns the Configured Override value for the Ideal Sample Time
     */
    static inline uint8_t
    RDC_getConfiguredOverrideIdealSampleTime(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_SAMPLE_CFG1_0 + (core * RDC_CORE_OFFSET);
        uint32_t value = HW_RD_REG32(base + regOffset) & CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_OVR_MASK;
        return((uint8_t) (value >> CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_OVR_SHIFT));
    }

    /**
     * @brief Returns the Ideal Sample Time Esitimated by the resolver core.
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return uint8_t
     */
    static inline uint8_t
    RDC_getIdealSampleTime(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_SAMPLE_CFG1_0 + (core * RDC_CORE_OFFSET);
        return (
            (HW_RD_REG32(
                 base + regOffset) &
             CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_MASK) >>
            CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_SHIFT);
    }

    /**
     * @brief sets Ideal Sample Detetction Threshold. validates the sample for the
     *  Ideal Sample time detection computation
     *
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param absThresholdValue the value needs to be higher than the trigger peak
     * detect value.
     */
    static inline void
    RDC_setIdealSampleDetectionThreshold(uint32_t base, uint8_t core, uint16_t absThresholdValue)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_SAMPLE_CFG2_0 + (core * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_SAMPLE_CFG2_0_SAMPLE_DET_THRESHOLD_MASK) |
                ((uint32_t)(absThresholdValue << CSL_RESOLVER_REGS_SAMPLE_CFG2_0_SAMPLE_DET_THRESHOLD_SHIFT)));
    }

    /**
     * @brief Returns the confiugured Ideal Sample Detection Threshold
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1     
     * @return uint16_t Absolute Threshold value configured for the ideal sample detection
     */
    static inline uint16_t
    RDC_getConfiguredIdealSampleDetectionThreshold(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_SAMPLE_CFG2_0 + (core * RDC_CORE_OFFSET);
        return ((uint16_t) (
            (HW_RD_REG32 (base + regOffset) & CSL_RESOLVER_REGS_SAMPLE_CFG2_0_SAMPLE_DET_THRESHOLD_MASK)
             >> CSL_RESOLVER_REGS_SAMPLE_CFG2_0_SAMPLE_DET_THRESHOLD_SHIFT));
    }

    /**
     * @brief the BPF sample adjust when the BPF is turned on. This configuration takes effect
     * only on the auto mode, and doesn't take effect on the manual mode
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param sampleAdjustCount this is a 5 bit value.
     */
    static inline void
    RDC_setIdealSampleBpfAdjust(uint32_t base, uint8_t core, uint8_t sampleAdjustCount)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_SAMPLE_CFG2_0 + (core * RDC_CORE_OFFSET);
        DebugP_assert(sampleAdjustCount <= RDC_MAX_IDEAL_SAMPLE_BPF_ADJUST);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_SAMPLE_CFG2_0_BANDPASSFILTERSAMPLEADJUST_MASK) |
                ((uint32_t)(sampleAdjustCount << CSL_RESOLVER_REGS_SAMPLE_CFG2_0_BANDPASSFILTERSAMPLEADJUST_SHIFT)));
    }

    static inline uint8_t
    RDC_getConfiguredIdealSampleBpfAdjust(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_SAMPLE_CFG2_0 + (core * RDC_CORE_OFFSET);
        return ((uint8_t) ((HW_RD_REG32(base + regOffset) &
             ~CSL_RESOLVER_REGS_SAMPLE_CFG2_0_BANDPASSFILTERSAMPLEADJUST_MASK) 
             >> CSL_RESOLVER_REGS_SAMPLE_CFG2_0_BANDPASSFILTERSAMPLEADJUST_SHIFT));
    }

    /**
     * @brief Gets the status if the Peak Averaging Limit is reached.
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return true
     * @return false
     */
    static inline bool
    RDC_getIdealSamplePeakAvgLimitStatus(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_SAMPLE_CFG2_0 + (core * RDC_CORE_OFFSET);
        return (
            ((HW_RD_REG32(
                  base + regOffset) &
              CSL_RESOLVER_REGS_SAMPLE_CFG2_0_PEAK_AVG_LIMIT_DONE_MASK) &
             ((uint32_t)((1U) << CSL_RESOLVER_REGS_SAMPLE_CFG2_0_PEAK_AVG_LIMIT_DONE_SHIFT))) != 0U);
    }
    //note add clearIdealSamplePeakAvgLimitStatus API.

    /**
     * @brief Ideal Sample Time Computation Mode selection.
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param mode uint8_t input. the following are the Valid arguments
     * - \e RDC_IDEAL_SAMPLE_TIME_MODE_0_AUTO_DETECT           - Computation on sin and cos
     * - \e RDC_IDEAL_SAMPLE_TIME_MODE_1_AUTO_DETECT_ON_SIN    - Computation on sin only
     * - \e RDC_IDEAL_SAMPLE_TIME_MODE_2_AUTO_DETECT_ON_COS    - Computation on cos only
     * - \e RDC_IDEAL_SAMPLE_TIME_MODE_3_AUTO_DETECT_OFF       - Manual Override of Ideal Sample Time
     */
    static inline void
    RDC_setIdealSampleMode(uint32_t base, uint8_t core, uint8_t mode)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DEC_GF_CFG0 + (core * RDC_CORE_OFFSET);
        DebugP_assert(
            (mode & ~(CSL_RESOLVER_REGS_DEC_GF_CFG0_IDEAL_SAMPLE_MODE_MAX)) == 0U);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_DEC_GF_CFG0_IDEAL_SAMPLE_MODE_MASK) |
                ((uint32_t)(mode << CSL_RESOLVER_REGS_DEC_GF_CFG0_IDEAL_SAMPLE_MODE_SHIFT)));
    }

    /**
     * @brief Returns the configured Ideal Sample Mode
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1     
     * @return uint8_t 
     * the following are the Valid outputs
     * - \e RDC_IDEAL_SAMPLE_TIME_MODE_0_AUTO_DETECT           - Computation on sin and cos
     * - \e RDC_IDEAL_SAMPLE_TIME_MODE_1_AUTO_DETECT_ON_SIN    - Computation on sin only
     * - \e RDC_IDEAL_SAMPLE_TIME_MODE_2_AUTO_DETECT_ON_COS    - Computation on cos only
     * - \e RDC_IDEAL_SAMPLE_TIME_MODE_3_AUTO_DETECT_OFF       - Manual Override of Ideal Sample Time
     */
    static inline uint8_t 
    RDC_getConfiguredIdealSampleMode(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DEC_GF_CFG0 + (core * RDC_CORE_OFFSET);
        uint32_t value = HW_RD_REG32(base + regOffset) & CSL_RESOLVER_REGS_DEC_GF_CFG0_IDEAL_SAMPLE_MODE_MASK;
        return ((uint8_t) (value >> CSL_RESOLVER_REGS_DEC_GF_CFG0_IDEAL_SAMPLE_MODE_SHIFT));
    }

    /**
     * @brief Enables bottom Sampling. twice the sampling rate than disabled. the track2
     * loop runs twice the speed so, the velocity output needs to be converted to respective
     * rotations per second accordingly.
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_enableIdealSampleBottomSampling(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DEC_GF_CFG0 + (core * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_DEC_GF_CFG0_ENABLE_BOTTOM_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_DEC_GF_CFG0_ENABLE_BOTTOM_SHIFT)));
    }

    /**
     * @brief Returns if the Bottom Sampling is enabled in the sample selection. 
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return true     Bottom Sampling is enabled
     * @return false    Bottom Sampling is disabled
     */
    static inline bool
    RDC_isIdealSampleBottomSamplingEnabled(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DEC_GF_CFG0 + (core * RDC_CORE_OFFSET);
        return ((HW_RD_REG32(base + regOffset) & 
            CSL_RESOLVER_REGS_DEC_GF_CFG0_ENABLE_BOTTOM_MASK) == CSL_RESOLVER_REGS_DEC_GF_CFG0_ENABLE_BOTTOM_MASK);
    }

    /**
     * @brief Disables Bottom Sampling.
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_disableIdealSampleBottomSampling(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DEC_GF_CFG0 + (core * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) |
             CSL_RESOLVER_REGS_DEC_GF_CFG0_ENABLE_BOTTOM_MASK) &
                ~((uint32_t)((1U) << CSL_RESOLVER_REGS_DEC_GF_CFG0_ENABLE_BOTTOM_SHIFT)));
    }

    //*****************************************************************************
    // PHASE GAIN ESTIMATION AND CORRECTION CONFIGURAITONS
    //*****************************************************************************

    /**
     * @brief Gets status if the Phase Gain Estimation is complete
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return true if the phase gain estimation is complete
     * @return false if the phase gain estimation is not complete
     */
    static inline bool
    RDC_getPhaseGainEstimationStatus(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG1_0 + (core * RDC_CORE_OFFSET);
        return (
            ((HW_RD_REG32(
                  base + regOffset) &
              CSL_RESOLVER_REGS_PG_EST_CFG1_0_AUTOPHASEGAINREADYDONE_MASK) &
             ((uint32_t)((1U) << CSL_RESOLVER_REGS_PG_EST_CFG1_0_AUTOPHASEGAINREADYDONE_SHIFT))) != 0U);
    }

    /**
     * @brief Clears the Phase Gain Estimation status. 
     * This can be used for any udpates to the thresholds or the train limits'
     * and can be polled on for updated values. 
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_clearPhaseGainEstimationStatus(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG1_0 + (core * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset, 
            HW_RD_REG32(base + regOffset) | CSL_RESOLVER_REGS_PG_EST_CFG1_0_AUTOPHASEGAINREADYDONE_MASK);
    }

    /**
     * @brief Sets the Phase Gain Estimation train limit.
     * if the programmed value is x, 2^x rotations are considered for the train limit.
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param pgEstimationLimit
     */
    static inline void
    RDC_setPhaseGainEstimationTrainLimit(uint32_t base, uint8_t core, uint8_t pgEstimationLimit)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG1_0 + (core * RDC_CORE_OFFSET);
        DebugP_assert(pgEstimationLimit <= CSL_RESOLVER_REGS_PG_EST_CFG1_0_PG_TRAIN_LIMIT_MAX);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_PG_EST_CFG1_0_PG_TRAIN_LIMIT_MASK) |
                ((uint32_t)(pgEstimationLimit << CSL_RESOLVER_REGS_PG_EST_CFG1_0_PG_TRAIN_LIMIT_SHIFT)));
    }

    /**
     * @brief Returns the configured Phase gain Estimation Train Limit
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core Within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return uint8_t Configured Train limit for Phase Gain Estimation
     */
    static inline uint8_t
    RDC_getConfiguredPhaseGainEstimationTrainLimit(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG1_0 + (core * RDC_CORE_OFFSET);
        return ((uint8_t) ((HW_RD_REG32(base + regOffset) & 
        CSL_RESOLVER_REGS_PG_EST_CFG1_0_PG_TRAIN_LIMIT_MASK) >> CSL_RESOLVER_REGS_PG_EST_CFG1_0_PG_TRAIN_LIMIT_SHIFT));
    }

    /**
     * @brief sets the Cos Phase Manual Bypass Value
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param cosPhaseBypass
     * the values range [-2^15, 2^15], mapping to -90 to 90 deg correction for the phase
     */
    static inline void
    RDC_setCosPhaseBypass(uint32_t base, uint8_t core, int16_t cosPhaseBypass)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG2_0 + (core * RDC_CORE_OFFSET);
        DebugP_assert((cosPhaseBypass & (~CSL_RESOLVER_REGS_PG_EST_CFG2_0_PHASECOSBYP0_MAX)) == 0U);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_PG_EST_CFG2_0_PHASECOSBYP0_MASK) |
                ((uint32_t)(cosPhaseBypass << CSL_RESOLVER_REGS_PG_EST_CFG2_0_PHASECOSBYP0_SHIFT)));
    }

    /**
     * @brief Returns the configrued Cosine Phase Bypass value
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return int16_t Configured Phase bypass value
     * \note the values range [-2^15, 2^15], map to -90 to 90 deg correction for the phase
     */
    static inline int16_t 
    RDC_getConfiguredCosPhaseBypass(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG2_0 + (core * RDC_CORE_OFFSET);
        return ((int16_t) ((HW_RD_REG32(base + regOffset) &
             CSL_RESOLVER_REGS_PG_EST_CFG2_0_PHASECOSBYP0_MASK) >> CSL_RESOLVER_REGS_PG_EST_CFG2_0_PHASECOSBYP0_SHIFT));
    }

    /**
     * @brief Enables Phase Gain Estimation in the background
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_enablePhaseGainEstimation(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG2_0 + (core * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) |
             CSL_RESOLVER_REGS_PG_EST_CFG2_0_BYPASSPHASEGAINCORR0_MASK) &
                ~((1U) << CSL_RESOLVER_REGS_PG_EST_CFG2_0_BYPASSPHASEGAINCORR0_SHIFT));

    }

    /**
     * @brief Disbales Phase Gain Estimation in the background
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_disablePhaseGainEstimation(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG2_0 + (core * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_PG_EST_CFG2_0_BYPASSPHASEGAINCORR0_MASK) |
                ((1U) << CSL_RESOLVER_REGS_PG_EST_CFG2_0_BYPASSPHASEGAINCORR0_SHIFT));
    }

    /**
     * @brief Returns if the Phase Gain Estimation logic is enabled
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return true Phase Gain Estimation Logic is enabled
     * @return false Phase Gain Estimation Logic is enabled
     */
    static inline bool
    RDC_isPhaseGainEstimationEnabled(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG2_0 + (core * RDC_CORE_OFFSET);
        return (
            (HW_RD_REG32(base + regOffset) &
             CSL_RESOLVER_REGS_PG_EST_CFG2_0_BYPASSPHASEGAINCORR0_MASK) != CSL_RESOLVER_REGS_PG_EST_CFG2_0_BYPASSPHASEGAINCORR0_MASK
        );
    }

    /**
     * @brief Enables Phase Auto Correction.
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_enablePhaseAutoCorrection(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG2_0 + (core * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOPHASECONTROL0_MASK) |
                ((1U) << CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOPHASECONTROL0_SHIFT));
    }

    /**
     * @brief Disables Phase Auto Correction.
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_disablePhaseAutoCorrection(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG2_0 + (core * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) |
             CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOPHASECONTROL0_MASK) &
                ~((1U) << CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOPHASECONTROL0_SHIFT));
    }

    /**
     * @brief Returns if the Phase Auto Correction is enabled.
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return true Phase Auto Correction is enabled
     * @return false Phase Auto Correction is disabled
     */
    static inline bool
    RDC_isPhaseAutoCorrectionEnabled(uint32_t base, uint16_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG2_0 + (core * RDC_CORE_OFFSET);
        return(
            ((HW_RD_REG32(base + regOffset) & CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOPHASECONTROL0_MASK) == CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOPHASECONTROL0_MASK)
        );
    }

    /**
     * @brief Enable Gain Auto correction
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_enableGainAutoCorrection(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG2_0 + (core * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOGAINCONTROL0_MASK) |
                ((1U) << CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOGAINCONTROL0_SHIFT));
    }

    /**
     * @brief Disable Gain Auto Correction
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_disableGainAutoCorrection(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG2_0 + (core * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) |
             CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOGAINCONTROL0_MASK) &
                ~((1U) << CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOGAINCONTROL0_SHIFT));
    }

    /**
     * @brief Returns if the Gain Auto Correction is
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return true Gain Auto correction is enabled
     * @return false Gain Auto correction is disabled
     */
    static inline bool
    RDC_isGainAutoCorrectionEnabled(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG2_0 + (core * RDC_CORE_OFFSET);
        return(
            (HW_RD_REG32(base + regOffset) & CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOGAINCONTROL0_MASK) == CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOGAINCONTROL0_MASK
        );
    }

    /**
     * @brief Sets the Manual Gain Correction values for Sin and Cos
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param sinGainBypass uint16_t always positive value (gain >= 2^14)
     * @param cosGainBypass uint16_t type
     */
    static inline void
    RDC_setGainBypassValue(uint32_t base, uint8_t core, uint16_t sinGainBypass, uint16_t cosGainBypass)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG3_0 + (core * RDC_CORE_OFFSET);
        DebugP_assert(sinGainBypass >= 16384U);
        uint32_t value = ((uint32_t)(((uint32_t)cosGainBypass) << 16)) | ((uint32_t)((uint16_t)sinGainBypass));

        HW_WR_REG32(
            base + regOffset, value);
    }

    /**
     * @brief Returns the sine and cosine gain bypass values configured.
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param sinGainBypass Configured Sine Gain Bypass value
     * @param cosGainBypass Configured Cosine Gain Bypass value
     */
    static inline void 
    RDC_getConfiguredGainBypassValue(uint32_t base, uint8_t core, uint16_t* sinGainBypass, uint16_t* cosGainBypass)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG3_0 + (core * RDC_CORE_OFFSET);
        uint32_t value = HW_RD_REG32(base+regOffset);

        *sinGainBypass = (uint16_t) ((value & CSL_RESOLVER_REGS_PG_EST_CFG3_0_GAINSINBYP0_MASK) >> CSL_RESOLVER_REGS_PG_EST_CFG3_0_GAINSINBYP0_SHIFT);
        *cosGainBypass = (uint16_t) ((value & CSL_RESOLVER_REGS_PG_EST_CFG3_0_GAINCOSBYP0_MASK) >> CSL_RESOLVER_REGS_PG_EST_CFG3_0_GAINCOSBYP0_SHIFT);
    }

    /**
     * @brief returns the Cos Phase Offset Estimation
     * this can be used only if the RDC_getPhaseGainEstimationStatus() returns true
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return int16_t  -2^15 corresponds to -90 deg and 2^15 corresponds to +90 deg
     */
    static inline int16_t 
    RDC_getPhaseEstimation(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG4_0 + (core * RDC_CORE_OFFSET);
        return ((int16_t)
            (HW_RD_REG32(
                 base + regOffset) &
             CSL_RESOLVER_REGS_PG_EST_CFG4_0_PHASEESTIMATEFINAL_MASK) >>
            CSL_RESOLVER_REGS_PG_EST_CFG4_0_PHASEESTIMATEFINAL_SHIFT);
    }

    /**
     * @brief returns the Gain Squared estimates for sin and cosine gain values
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     *
     * \note this is gain squared value. Perform a sqrt() over to get the actual gain value. 
     * the calculated gain value can be used in the RDC_setGainBypassValue() as the gain_calc*(2^14)
     * being the \b sinGainBypass or \b cosGainBypass respectively for sine and cosine gain bypass 
     * values
     * 
     *  valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param cosGainEstimateSq float pass by reference
     * @param sinGainEstimateSq float pass by reference
     */
    static inline void
    RDC_getGainEstimation(uint32_t base, uint8_t core, float *sinGainEstimateSq, float *cosGainEstimateSq)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_PG_EST_CFG5_0 + (core * RDC_CORE_OFFSET);
        uint32_t cosSqAccValue = HW_RD_REG32(base + regOffset);
        
        *cosGainEstimateSq = ((uint32_t) 1  << 29)/cosSqAccValue;

        regOffset = CSL_RESOLVER_REGS_PG_EST_CFG6_0 + (core * RDC_CORE_OFFSET);
        
        uint32_t sinSqAccValue = HW_RD_REG32(base + regOffset);
        *sinGainEstimateSq = ((uint32_t) 1  << 29)/sinSqAccValue;
    }

    //*****************************************************************************
    // TRACK2 CONFIGURATIONS
    //*****************************************************************************


    /**
     * @brief sets up the Track2 loop constants
     * the following are the constants that can be setup using this API
     * - kvelfilt
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param track2Constants
     */
    static inline void
    RDC_setTrack2Constants(uint32_t base, uint8_t core, Track2Constants_t *track2Constants)
    {
        uint8_t kvelfilt = track2Constants->kvelfilt;
        uint32_t cfg1 = (kvelfilt << CSL_RESOLVER_REGS_TRACK2_CFG1_0_KVELFILT_SHIFT);

        uint32_t mask_cfg1 = (CSL_RESOLVER_REGS_TRACK2_CFG1_0_KVELFILT_MASK);

        uint32_t regOffset_cfg1 = CSL_RESOLVER_REGS_TRACK2_CFG1_0 + (core * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset_cfg1,
            (HW_RD_REG32(
                 base + regOffset_cfg1) &
             (~mask_cfg1)) |
                cfg1);
    }

    /**
     * @brief Returns the configured Track2 Constants
     * 
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @param track2Constants returns the configured into the struct members
     */
    static inline void
    RDC_getConfiguredTrack2Constants(uint32_t base, uint8_t core, Track2Constants_t* track2Constants)
    {
        uint32_t regOffset_cfg1 = CSL_RESOLVER_REGS_TRACK2_CFG1_0 + (core * RDC_CORE_OFFSET);
        track2Constants->kvelfilt = (uint8_t) (HW_RD_REG32(base + regOffset_cfg1) >> CSL_RESOLVER_REGS_TRACK2_CFG1_0_KVELFILT_SHIFT);
    }

    /**
     * @brief enables the track2 Boost
     *
     * @param base RDC base address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_enableTrack2Boost(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_TRACK2_CFG2_0 + (core * RDC_CORE_OFFSET);

        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) &
             ~CSL_RESOLVER_REGS_TRACK2_CFG2_0_BOOST_MASK) |
                ((uint32_t)((1U) << CSL_RESOLVER_REGS_TRACK2_CFG2_0_BOOST_SHIFT)));
    }
    /**
     * @brief disables the track2 Boost
     *
     * @param base RDC base address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     */
    static inline void
    RDC_disableTrack2Boost(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_TRACK2_CFG2_0 + (core * RDC_CORE_OFFSET);

        HW_WR_REG32(
            base + regOffset,
            (HW_RD_REG32(
                 base + regOffset) |
             CSL_RESOLVER_REGS_TRACK2_CFG2_0_BOOST_MASK) &
                ~((uint32_t)((1U) << CSL_RESOLVER_REGS_TRACK2_CFG2_0_BOOST_SHIFT)));
    }
    // Add is boost enabled API.
    
    /**
     * @brief Returns signed 16bit angle data from ArcTan. the data corresponds to
     * -180 to +180 degrees
     *      angle in degrees : ((16b signed int)) * 360 / 2^16
     *
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return in16_t angle value
     */
    static inline int16_t
    RDC_getArcTanAngle(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_ANGLE_ARCTAN_0 + (core * RDC_CORE_OFFSET);
        return (
            (int16_t)HW_RD_REG16(
                base + regOffset));
    }

    /**
     * @brief Returns Signed 16 bit angle data from Track2 Loop. the data corresponds to
     * -180 to 180 degrees
     *      angle in degrees : ((16b signed int)) * 360 / 2^16
     * @param base RDC Base Address
     * @param core denotes Resolver Core within RDC
     * valid values are \e RDC_RESOLVER_CORE0, \e RDC_RESOLVER_CORE1
     * @return int16_t angle value
     */
    static inline int16_t
    RDC_getTrack2Angle(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_ANGLE_TRACK2_0 + (core * RDC_CORE_OFFSET);
        return (
            (int16_t)HW_RD_REG16(
                base + regOffset));
    }

    /**
     * @brief Returns Signed 32 bit Velocity data from Track2 Loop.
     *
     *  Velocity in RPS : ((32b singed int)* frequency)/(2^32)  [if RDC_disableIdealSampleBottomSampling() is called]
     *  Velocity in RPS : ((32b singed int)* frequency)/((2^32) * 2)  [if RDC_enableIdealSampleBottomSampling() is called]
     *
     * @param base
     * @param core
     * @return int32_t
     */
    static inline int32_t
    RDC_getTrack2Velocity(uint32_t base, uint8_t core)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_VELOCITY_TRACK2_0 + (core * RDC_CORE_OFFSET);
        return (
            (int32_t)HW_RD_REG32(
                base + regOffset));
    }

    //*****************************************************************************
    // DIAGNOSTIC RELATED APIS
    //*****************************************************************************

    /**
     * @brief Returns the Monitor Sin or Cos Offset Drift (DOS) diagnostics data
     * \e int16_t \e offset_drift_threshold_hi - the configured offset drift threshold hi value
     * \e int16_t \e offset_drift_threshold_lo - the configured offset drift threshold lo value
     * \e bool    \e offset_drift_cos_hi       - the status of the flag in the Interrupt status
     * \e bool    \e offset_drift_cos_lo       - the status of the flag in the Interrupt status
     * \e bool    \e offset_drift_sin_hi       - the status of the flag in the Interrupt status
     * \e bool    \e offset_drift_sin_lo       - the status of the flag in the Interrupt status
     * \e bool    \e offset_drift_en           - if the interrupt is enabled for any of the error status
     *
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_SinCos_Offset_drift_data struct as input
     */
    static inline void
    RDC_getDiagnosticsSinCosOffsetDriftData(uint32_t base,
                                            uint8_t resolverCore,
                                            Diag_Mon_SinCos_Offset_drift_data *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG1_0 + (RDC_CORE_OFFSET * resolverCore);
        uint32_t value = HW_RD_REG32(base + regOffset);
        uint32_t interruptStatus = RDC_getCoreInterruptStatus(base, resolverCore);
        uint32_t enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);

        monitorData->offset_drift_threshold_hi = (int16_t)((uint16_t)((value &
                                                                       CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_HI_MASK) >>
                                                                      CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_HI_SHIFT));
        monitorData->offset_drift_threshold_lo = (int16_t)((uint16_t)((value &
                                                                       CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_LO_MASK) >>
                                                                      CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_LO_SHIFT));

        monitorData->offset_drift_cos_hi = ((interruptStatus & RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_HI_ERR) != 0);
        monitorData->offset_drift_sin_hi = ((interruptStatus & RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_HI_ERR) != 0);
        monitorData->offset_drift_cos_lo = ((interruptStatus & RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_LO_ERR) != 0);
        monitorData->offset_drift_sin_lo = ((interruptStatus & RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_LO_ERR) != 0);
        monitorData->offset_drift_en = ((enabledInterruptSources &
                                         (RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_HI_ERR |
                                          RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_HI_ERR |
                                          RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_LO_ERR |
                                          RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_LO_ERR)) != 0);
    }

    /**
     * @brief Sets the Monitor Sin or Cos Offset Drift (DOS) diagnostics controls
     * \e int16_t \e offset_drift_threshold_hi - the offset drift threshold hi value to be configured
     * \e int16_t \e offset_drift_threshold_lo - the offset drift threshold lo value to be configured
     * \e bool    \e offset_drift_cos_hi       - enables interrupt on this flag if set and offset_drift_en is set
     * \e bool    \e offset_drift_cos_lo       - enables interrupt on this flag if set and offset_drift_en is set
     * \e bool    \e offset_drift_sin_hi       - enables interrupt on this flag if set and offset_drift_en is set
     * \e bool    \e offset_drift_sin_lo       - enables interrupt on this flag if set and offset_drift_en is set
     * \e bool    \e offset_drift_en           - enables interrupt on the errors
     *
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_SinCos_Offset_drift_data struct as input
     */
    static inline void
    RDC_setDiagnosticsSinCosOffsetDriftData(uint32_t base,
                                            uint8_t resolverCore,
                                            Diag_Mon_SinCos_Offset_drift_data *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG1_0 + (RDC_CORE_OFFSET * resolverCore);
        uint32_t value = (((uint32_t)((uint16_t)(monitorData->offset_drift_threshold_hi))) << CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_HI_SHIFT) |
                         (((uint32_t)((uint16_t)(monitorData->offset_drift_threshold_lo))) << CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_LO_SHIFT);
        uint32_t interruptSource = 0;
        /* if interrupt needs to be enabled */
        RDC_disableCoreInterrupt(base, resolverCore,
                                 (RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_HI_ERR |
                                  RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_LO_ERR |
                                  RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_HI_ERR |
                                  RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_LO_ERR));

        /* writing the "value" */
        HW_WR_REG32(
            base + regOffset, value);

        uint32_t enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);

        if (monitorData->offset_drift_en)
        {
            if (monitorData->offset_drift_cos_hi)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_HI_ERR;
            }

            if (monitorData->offset_drift_sin_hi)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_HI_ERR;
            }

            if (monitorData->offset_drift_cos_lo)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_LO_ERR;
            }

            if (monitorData->offset_drift_sin_lo)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_LO_ERR;
            }

            RDC_enableCoreInterrupt(base, resolverCore, (interruptSource | enabledInterruptSources));
        }
    }

    /**
     * @brief Returns the Monitor Sin or Cos Gain drift (DOS) diagnostics data
     * \e uint16_t \e gain_drift_threshold_hi - the configured gain drift threshold hi value
     * \e uint16_t \e gain_drift_threshold_lo - the configured gain drift threshold lo value
     * \e uint8_t \e gain_drift_glitch_count - the configured gain drift glitch count value
     * \e bool    \e gain_drift_cos_hi       - the status of the flag in the Interrupt status
     * \e bool    \e gain_drift_cos_lo       - the status of the flag in the Interrupt status
     * \e bool    \e gain_drift_sin_hi       - the status of the flag in the Interrupt status
     * \e bool    \e gain_drift_sin_lo       - the status of the flag in the Interrupt status
     * \e bool    \e gain_drift_en           - if the interrupt is enabled for any of the error status
     *
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_SinCos_Gain_drift_data struct as input
     */
    static inline void
    RDC_getDiagnosticsSinCosGainDriftData(uint32_t base,
                                          uint8_t resolverCore,
                                          Diag_Mon_SinCos_Gain_drift_data *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG14_0 + (RDC_CORE_OFFSET * resolverCore);

        uint32_t value = HW_RD_REG32(base + regOffset);

        uint32_t interruptStatus = RDC_getCoreInterruptStatus(base, resolverCore);
        uint32_t enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);

        monitorData->gain_drift_threshold_hi = ((uint16_t)((value &
                                                                     CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_HI_MASK) >>
                                                                    CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_HI_SHIFT));
        monitorData->gain_drift_threshold_lo = ((uint16_t)((value &
                                                                     CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_LO_MASK) >>
                                                                    CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_LO_SHIFT));

        monitorData->gain_drift_cos_hi = ((interruptStatus & RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_HI_ERR) != 0);
        monitorData->gain_drift_cos_lo = ((interruptStatus & RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_LO_ERR) != 0);
        monitorData->gain_drift_sin_hi = ((interruptStatus & RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_HI_ERR) != 0);
        monitorData->gain_drift_sin_lo = ((interruptStatus & RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_LO_ERR) != 0);

        /* getting the glitch count value*/
        regOffset = CSL_RESOLVER_REGS_DIAG15_0 + (RDC_CORE_OFFSET * resolverCore);
        monitorData->gain_drift_glitch_count = (uint8_t)HW_RD_REG32(base + regOffset);

        monitorData->gain_drift_en = ((enabledInterruptSources &
                                      (RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_HI_ERR |
                                       RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_LO_ERR |
                                       RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_HI_ERR |
                                       RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_LO_ERR)) != 0);
    }

    /**
     * @brief Sets the Monitor Sin or Cos Gain drift (DOS) diagnostics Controls
     * \e uint16_t \e gain_drift_threshold_hi - the gain drift threshold hi value to be configured
     * \e uint16_t \e gain_drift_threshold_lo - the gain drift threshold lo value to be configured
     * \e uint8_t \e gain_drift_glitch_count - the gain drift glitch count value to be configured
     * \e bool    \e gain_drift_cos_hi       - enables interrupt on this flag if set and gain_drift_en is set
     * \e bool    \e gain_drift_cos_lo       - enables interrupt on this flag if set and gain_drift_en is set
     * \e bool    \e gain_drift_sin_hi       - enables interrupt on this flag if set and gain_drift_en is set
     * \e bool    \e gain_drift_sin_lo       - enables interrupt on this flag if set and gain_drift_en is set
     * \e bool    \e gain_drift_en           - enables interrupt on the errors
     *
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_SinCos_Gain_drift_data struct as input
     */
    static inline void
    RDC_setDiagnosticsSinCosGainDriftData(uint32_t base,
                                          uint8_t resolverCore,
                                          Diag_Mon_SinCos_Gain_drift_data *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG14_0 + (RDC_CORE_OFFSET * resolverCore);
        uint32_t value = (((uint32_t)((uint16_t)(monitorData->gain_drift_threshold_hi))) << CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_HI_SHIFT) |
                         (((uint32_t)((uint16_t)(monitorData->gain_drift_threshold_lo))) << CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_LO_SHIFT);

        uint32_t interruptSource = 0;

        /* if interrupt needs to be enabled */
        RDC_disableCoreInterrupt(base, resolverCore,
                                 (RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_LO_ERR |
                                  RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_HI_ERR |
                                  RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_LO_ERR |
                                  RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_HI_ERR));

        /* setting the glitch count value */
        regOffset = CSL_RESOLVER_REGS_DIAG15_0 + (RDC_CORE_OFFSET * resolverCore);
        HW_WR_REG32(
            base + regOffset, monitorData->gain_drift_glitch_count);

        /* writing the "value" */
        HW_WR_REG32(
            base + regOffset, value);

        uint32_t enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);

        if (monitorData->gain_drift_en)
        {
            if (monitorData->gain_drift_cos_hi)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_HI_ERR;
            }

            if (monitorData->gain_drift_sin_hi)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_HI_ERR;
            }

            if (monitorData->gain_drift_cos_lo)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_LO_ERR;
            }

            if (monitorData->gain_drift_sin_lo)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_LO_ERR;
            }

            RDC_enableCoreInterrupt(base, resolverCore, (interruptSource | enabledInterruptSources));
        }
    }


    /**
     * @brief Returns the Monitor Cos Phase drift (DOS) diagnostics data
     * \e int16_t \e phase_drift_threshold_hi - the configured phase drift threshold hi value
     * \e int16_t \e phase_drift_threshold_lo - the configured phase drift threshold lo value
     * \e uint8_t \e phase_drift_glitch_count - the configured phase drift glitch count value
     * \e bool    \e phase_drift_cos_hi       - the status of the flag in the Interrupt status
     * \e bool    \e phase_drift_cos_lo       - the status of the flag in the Interrupt status
     * \e bool    \e phase_drift_en           - if the interrupt is enabled for any of the error status
     *
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_Cos_Phase_drift_data struct as input
     */
    static inline void
    RDC_getDiagnosticsCosPhaseDriftData(uint32_t base,
                                        uint8_t resolverCore,
                                        Diag_Mon_Cos_Phase_drift_data *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG16_0 + (RDC_CORE_OFFSET * resolverCore);

        uint32_t value = HW_RD_REG32(base + regOffset);

        uint32_t interruptStatus = RDC_getCoreInterruptStatus(base, resolverCore);
        uint32_t enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);

        monitorData->phase_drift_threshold_hi = (int16_t)((uint16_t)((value &
                                                                      CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_HI_MASK) >>
                                                                     CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_HI_SHIFT));
        monitorData->phase_drift_threshold_lo = (int16_t)((uint16_t)((value &
                                                                      CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_LO_MASK) >>
                                                                     CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_LO_SHIFT));

        monitorData->phase_drift_cos_hi = ((interruptStatus & RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_HI_ERR) != 0);
        monitorData->phase_drift_cos_lo = ((interruptStatus & RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_LO_ERR) != 0);

        /* getting the glitch count value*/
        regOffset = CSL_RESOLVER_REGS_DIAG17_0 + (RDC_CORE_OFFSET * resolverCore);
        monitorData->phase_drift_glitch_count = (uint8_t)HW_RD_REG32(base + regOffset);

        monitorData->phase_drift_en = ((enabledInterruptSources &
                                       (RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_LO_ERR |
                                        RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_HI_ERR)) != 0);
    }

    /**
     * @brief Sets the Monitor Cos Phase drift (DOS) diagnostics Controls
     * \e int16_t \e phase_drift_threshold_hi - the phase drift threshold hi value to be configured
     * \e int16_t \e phase_drift_threshold_lo - the phase drift threshold lo value to be configured
     * \e uint8_t \e phase_drift_glitch_count - the phase drift glitch count value to be configured
     * \e bool    \e phase_drift_cos_hi       - enables interrupt on this flag if set and phase_drift_en is set
     * \e bool    \e phase_drift_cos_lo       - enables interrupt on this flag if set and phase_drift_en is set
     * \e bool    \e phase_drift_en           - enables interrupt on the errors
     *
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_Cos_Phase_drift_data struct as input
     */
    static inline void
    RDC_setDiagnosticsCosPhaseDriftData(uint32_t base,
                                        uint8_t resolverCore,
                                        Diag_Mon_Cos_Phase_drift_data *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG16_0 + (RDC_CORE_OFFSET * resolverCore);
        uint32_t value = (((uint32_t)((uint16_t)(monitorData->phase_drift_threshold_hi))) << CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_HI_SHIFT) |
                         (((uint32_t)((uint16_t)(monitorData->phase_drift_threshold_lo))) << CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_LO_SHIFT);

        uint32_t interruptSource = 0;
        /* if interrupt needs to be enabled */
        RDC_disableCoreInterrupt(base, resolverCore,
                                 (RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_LO_ERR |
                                  RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_HI_ERR));
        /* writing the glitch count value */
        regOffset = CSL_RESOLVER_REGS_DIAG17_0 + (RDC_CORE_OFFSET * resolverCore);
        HW_WR_REG32(
            base + regOffset, monitorData->phase_drift_glitch_count);

        /* writing the "value" */
        HW_WR_REG32(
            base + regOffset, value);

        uint32_t enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);

        if (monitorData->phase_drift_en)
        {
            if (monitorData->phase_drift_cos_hi)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_HI_ERR;
            }

            if (monitorData->phase_drift_cos_lo)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_LO_ERR;
            }

            RDC_enableCoreInterrupt(base, resolverCore, (interruptSource | enabledInterruptSources));
        }
    }



    /**
     * @brief Returns the Monitor excitation frequency degradation or loss (DOS) diagnostics data
     * \e uint16_t \e excfreqdetected_sin       - the counts detected between ZCS for sin excitation frequency
     * \e uint16_t \e excfreqdetected_cos       - the counts detected between ZCS for cos excitation frequency
     * \e uint16_t \e excfreqdrift_threshold_hi - the configured excfreqdrift threshold hi value
     * \e uint16_t \e excfreqdrift_threshold_lo - the configured excfreqdrift threshold lo value
     * \e uint16_t \e excfreq_level             - the configured threshold on ADC to detect sin or cos zero crossings
     * \e uint8_t \e excfreqdrift_glitchcount   - the configured excfreqdrift glitch count value
     * \e bool    \e excfreqdrift_hi            - the status of the flag in the Interrupt status
     * \e bool    \e excfreqdrift_cos_lo        - the status of the flag in the Interrupt status
     * \e bool    \e excfreqdrift_sin_lo        - the status of the flag in the Interrupt status
     * \e bool    \e excfreqdrift_en            - if the interrupt is enabled for any of the error status
     *
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_ExcFreq_Degradataion_data struct as input
     */
    static inline void
    RDC_getDiagnosticsExcFreqDegradationData(uint32_t base,
                                             uint8_t resolverCore,
                                             Diag_Mon_ExcFreq_Degradataion_data *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG2_0 + (RDC_CORE_OFFSET * resolverCore);

        uint32_t value = HW_RD_REG32(base + regOffset);

        uint32_t interruptStatus = RDC_getCoreInterruptStatus(base, resolverCore);
        uint32_t enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);

        monitorData->excfreqdetected_sin = ((uint16_t)((value &
                                                        CSL_RESOLVER_REGS_DIAG2_0_EXCFREQDETECTED_SIN_MASK) >>
                                                       CSL_RESOLVER_REGS_DIAG2_0_EXCFREQDETECTED_SIN_SHIFT));
        monitorData->excfreqdetected_cos = ((uint16_t)((value &
                                                        CSL_RESOLVER_REGS_DIAG2_0_EXCFREQDETECTED_COS_MASK) >>
                                                       CSL_RESOLVER_REGS_DIAG2_0_EXCFREQDETECTED_COS_SHIFT));

        regOffset = CSL_RESOLVER_REGS_DIAG3_0 + (RDC_CORE_OFFSET * resolverCore);
        value = HW_RD_REG32(base + regOffset);

        monitorData->excfreqdrift_threshold_hi = ((uint16_t)((value &
                                                              CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_HI_MASK) >>
                                                             CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_HI_SHIFT));
        monitorData->excfreqdrift_threshold_lo = ((uint16_t)((value &
                                                              CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_LO_MASK) >>
                                                             CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_LO_SHIFT));

        regOffset = CSL_RESOLVER_REGS_DIAG4_0 + (RDC_CORE_OFFSET * resolverCore);
        value = HW_RD_REG32(base + regOffset);

        monitorData->excfreq_level = ((uint16_t)((value &
                                                  CSL_RESOLVER_REGS_DIAG4_0_EXCFREQ_LEVEL_MASK) >>
                                                 CSL_RESOLVER_REGS_DIAG4_0_EXCFREQ_LEVEL_SHIFT));
        monitorData->excfreqdrift_glitchcount = ((uint8_t)((value &
                                                            CSL_RESOLVER_REGS_DIAG4_0_EXCFREQDRIFT_GLITCHCOUNT_MASK) >>
                                                           CSL_RESOLVER_REGS_DIAG4_0_EXCFREQDRIFT_GLITCHCOUNT_SHIFT));

        monitorData->excfreqdrift_hi = ((interruptStatus & RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_HI_ERR) != 0);
        monitorData->excfreqdrift_cos_lo = ((interruptStatus & RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_COS_LO_ERR) != 0);
        monitorData->excfreqdrift_sin_lo = ((interruptStatus & RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_SIN_LO_ERR) != 0);

        monitorData->excfreqdrift_en = ((enabledInterruptSources &
                                         (RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_HI_ERR |
                                          RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_COS_LO_ERR |
                                          RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_SIN_LO_ERR)) != 0);
    }

    /**
     * @brief Sets the Monitor excitation frequency degradation or loss (DOS) diagnostics Controls
     * \e uint16_t \e excfreqdetected_sin       - no action
     * \e uint16_t \e excfreqdetected_cos       - no action
     * \e uint16_t \e excfreqdrift_threshold_hi - the excfreqdrift threshold hi value to be configured
     * \e uint16_t \e excfreqdrift_threshold_lo - the excfreqdrift threshold lo value to be configured
     * \e uint16_t \e excfreq_level             - the threshold on ADC to detect sin or cos zero crossings to be configured
     * \e uint8_t \e excfreqdrift_glitchcount   - the excfreqdrift glitch count value to be configured
     * \e bool    \e excfreqdrift_hi            - enables interrupt on this flag if set and excfreqdrift_en is set
     * \e bool    \e excfreqdrift_cos_lo        - enables interrupt on this flag if set and excfreqdrift_en is set
     * \e bool    \e excfreqdrift_sin_lo        - enables interrupt on this flag if set and excfreqdrift_en is set
     * \e bool    \e excfreqdrift_en            - enables interrupt on the errors
     *
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_ExcFreq_Degradataion_data struct as input
     */
    static inline void
    RDC_setDiagnosticsExcFreqDegradationData(uint32_t base,
                                             uint8_t resolverCore,
                                             Diag_Mon_ExcFreq_Degradataion_data *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG3_0 + (RDC_CORE_OFFSET * resolverCore);
        uint32_t value = (((uint32_t)((uint16_t)(monitorData->excfreqdrift_threshold_hi))) << CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_HI_SHIFT) |
                         (((uint32_t)((uint16_t)(monitorData->excfreqdrift_threshold_lo))) << CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_LO_SHIFT);
        uint32_t interruptSource = 0;

        RDC_disableCoreInterrupt(base, resolverCore,
                                 (RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_HI_ERR |
                                  RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_COS_LO_ERR |
                                  RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_SIN_LO_ERR));

        uint32_t enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);

        HW_WR_REG32(
            base + regOffset, value);

        regOffset = CSL_RESOLVER_REGS_DIAG4_0 + (RDC_CORE_OFFSET * resolverCore);
        value = (((uint32_t)((uint16_t)(monitorData->excfreq_level))) << CSL_RESOLVER_REGS_DIAG4_0_EXCFREQ_LEVEL_SHIFT) |
                (((uint32_t)((uint16_t)(monitorData->excfreqdrift_glitchcount))) << CSL_RESOLVER_REGS_DIAG4_0_EXCFREQDRIFT_GLITCHCOUNT_SHIFT);

        HW_WR_REG32(
            base + regOffset, value);

        if (monitorData->excfreqdrift_en)
        {
            if (monitorData->excfreqdrift_hi)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_HI_ERR;
            }

            if (monitorData->excfreqdrift_cos_lo)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_COS_LO_ERR;
            }

            if (monitorData->excfreqdrift_sin_lo)
            {
                interruptSource |= RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_SIN_LO_ERR;
            }
            RDC_enableCoreInterrupt(base, resolverCore, (interruptSource | enabledInterruptSources));
        }
    }

     /**
     * @brief Returns the Monitor rotational signal integrity (DOS) diagnostics data
     * \e bool      \e cos_neg_zc_peak_mismatch_err - the status of the flag in the Interrupt status
     * \e bool      \e cos_pos_zc_peak_mismatch_err - the status of the flag in the Interrupt status
     * \e bool      \e sin_neg_zc_peak_mismatch_err - the status of the flag in the Interrupt status
     * \e bool      \e sin_pos_zc_peak_mismatch_err - the status of the flag in the Interrupt status
     * \e bool      \e sin_multi_zc_error_err       - the status of the flag in the Interrupt status
     * \e bool      \e cos_multi_zc_error_err       - the status of the flag in the Interrupt status
     * \e uint8_t   \e cos_multi_zc_error_count     - status of cos capture number of faulty ZCS
     * \e uint8_t   \e sin_multi_zc_error_count     - status of sin capture number of faulty ZCS
     * \e uint16_t  \e rotpeak_level                - the configured "peak level" of rotational Sin/Cos when other signal is zero crossing
     * \e uint16_t  \e rotfreq_level                - the configured "zero crossing detection level" of rotational sin/cos
     * \e bool      \e zero_cross_rot_en            - if the interrupt is enabled for any of the error status
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_Rotational_Signal_Integrity_data struct as input
     */
    static inline void
    RDC_getDiagnosticsRotationalSignalIntegrityData(uint32_t base,
                                                    uint8_t resolverCore,
                                                    Diag_Mon_Rotational_Signal_Integrity_data *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG13_0 + (RDC_CORE_OFFSET * resolverCore);
        uint32_t value = HW_RD_REG32(
            base + regOffset);

        monitorData->sin_multi_zc_error_count = (uint8_t)((value & CSL_RESOLVER_REGS_DIAG13_0_SIN_MULTI_ZC_ERROR_COUNT_MASK) >> CSL_RESOLVER_REGS_DIAG13_0_SIN_MULTI_ZC_ERROR_COUNT_SHIFT);

        monitorData->cos_multi_zc_error_count = (uint8_t)((value & CSL_RESOLVER_REGS_DIAG13_0_COS_MULTI_ZC_ERROR_COUNT_MASK) >> CSL_RESOLVER_REGS_DIAG13_0_COS_MULTI_ZC_ERROR_COUNT_SHIFT);

        regOffset = CSL_RESOLVER_REGS_DIAG12_0 + (RDC_CORE_OFFSET * resolverCore);
        value = HW_RD_REG32(
            base + regOffset);
        monitorData->rotpeak_level = (value & CSL_RESOLVER_REGS_DIAG12_0_ROTPEAK_LEVEL_MASK) >> CSL_RESOLVER_REGS_DIAG12_0_ROTPEAK_LEVEL_SHIFT;
        monitorData->rotpeak_level = (value & CSL_RESOLVER_REGS_DIAG12_0_ROTFREQ_LEVEL_MASK) >> CSL_RESOLVER_REGS_DIAG12_0_ROTFREQ_LEVEL_SHIFT;

        uint32_t interruptStatus =  RDC_getCoreInterruptStatus(base, resolverCore);
        monitorData->cos_neg_zc_peak_mismatch_err = ((interruptStatus & RDC_INTERRUPT_SOURCE_COS_NEG_ZC_PEAK_MISMATCH_ERR) != 0);
        monitorData->cos_pos_zc_peak_mismatch_err = ((interruptStatus & RDC_INTERRUPT_SOURCE_COS_POS_ZC_PEAK_MISMATCH_ERR) != 0);
        monitorData->sin_neg_zc_peak_mismatch_err = ((interruptStatus & RDC_INTERRUPT_SOURCE_SIN_NEG_ZC_PEAK_MISMATCH_ERR) != 0);
        monitorData->sin_pos_zc_peak_mismatch_err = ((interruptStatus & RDC_INTERRUPT_SOURCE_SIN_POS_ZC_PEAK_MISMATCH_ERR) != 0);


        uint32_t enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);
        monitorData->zero_cross_rot_en = (enabledInterruptSources & (RDC_INTERRUPT_SOURCE_COS_NEG_ZC_PEAK_MISMATCH_ERR |
                                                                     RDC_INTERRUPT_SOURCE_COS_POS_ZC_PEAK_MISMATCH_ERR |
                                                                     RDC_INTERRUPT_SOURCE_SIN_NEG_ZC_PEAK_MISMATCH_ERR |
                                                                     RDC_INTERRUPT_SOURCE_SIN_POS_ZC_PEAK_MISMATCH_ERR)) != 0;
    }

     /**
     * @brief Sets the Monitor rotational signal integrity (DOS) diagnostics Controls
     * \e bool      \e cos_neg_zc_peak_mismatch_err - enables interrupt on this flag if set and zero_cross_rot_en is set
     * \e bool      \e cos_pos_zc_peak_mismatch_err - enables interrupt on this flag if set and zero_cross_rot_en is set
     * \e bool      \e sin_neg_zc_peak_mismatch_err - enables interrupt on this flag if set and zero_cross_rot_en is set
     * \e bool      \e sin_pos_zc_peak_mismatch_err - enables interrupt on this flag if set and zero_cross_rot_en is set
     * \e bool      \e sin_multi_zc_error_err       - enables interrupt on this flag if set and zero_cross_rot_en is set
     * \e bool      \e cos_multi_zc_error_err       - enables interrupt on this flag if set and zero_cross_rot_en is set
     * \e uint8_t   \e cos_multi_zc_error_count     - no action
     * \e uint8_t   \e sin_multi_zc_error_count     - no action
     * \e uint16_t  \e rotpeak_level                - the "peak level" of rotational Sin/Cos when other signal is zero crossing to be configured
     * \e uint16_t  \e rotfreq_level                - the "zero crossing detection level" of rotational sin/cos to be configured
     * \e bool      \e zero_cross_rot_en            - enables interrupt on the errors
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_Rotational_Signal_Integrity_data struct as input
     */
    static inline void
    RDC_setDiagnosticsRotationalSignalIntegrityData(uint32_t base,
                                                    uint8_t resolverCore,
                                                    Diag_Mon_Rotational_Signal_Integrity_data *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG12_0 + (RDC_CORE_OFFSET * resolverCore);
        uint32_t value = (((uint32_t)((uint16_t)(monitorData->rotpeak_level))) << CSL_RESOLVER_REGS_DIAG12_0_ROTPEAK_LEVEL_SHIFT) |
                         (((uint32_t)((uint16_t)(monitorData->rotfreq_level))) << CSL_RESOLVER_REGS_DIAG12_0_ROTFREQ_LEVEL_SHIFT);
        HW_WR_REG32(
            base + regOffset,
            value);
        RDC_disableCoreInterrupt(base, resolverCore, (RDC_INTERRUPT_SOURCE_COS_NEG_ZC_PEAK_MISMATCH_ERR | RDC_INTERRUPT_SOURCE_COS_POS_ZC_PEAK_MISMATCH_ERR | RDC_INTERRUPT_SOURCE_SIN_NEG_ZC_PEAK_MISMATCH_ERR | RDC_INTERRUPT_SOURCE_SIN_POS_ZC_PEAK_MISMATCH_ERR));
        uint32_t enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);
        uint32_t interruptSources = 0;

        if (monitorData->zero_cross_rot_en)
        {
            if (monitorData->cos_neg_zc_peak_mismatch_err)
            {
                interruptSources |= RDC_INTERRUPT_SOURCE_COS_NEG_ZC_PEAK_MISMATCH_ERR;
            }

            if (monitorData->cos_pos_zc_peak_mismatch_err)
            {
                interruptSources |= RDC_INTERRUPT_SOURCE_COS_POS_ZC_PEAK_MISMATCH_ERR;
            }

            if (monitorData->sin_neg_zc_peak_mismatch_err)
            {
                interruptSources |= RDC_INTERRUPT_SOURCE_SIN_NEG_ZC_PEAK_MISMATCH_ERR;
            }

            if (monitorData->sin_pos_zc_peak_mismatch_err)
            {
                interruptSources |= RDC_INTERRUPT_SOURCE_SIN_POS_ZC_PEAK_MISMATCH_ERR;
            }

            RDC_enableCoreInterrupt(base, resolverCore, (interruptSources | enabledInterruptSources));
        }
    }

    /**
     * @brief Returns the Monitor signal integrity by checking Sin2+Cos2=Constant (DOS) diagnostics data
     * \e uint16_t \e sinsqcossq_threshold_hi - the configured threshold hi for sinsq + cossq
     * \e uint16_t \e sinsqcossq_threshold_lo - the configured threshold lo for sinsq + cossq
     * \e uint8_t  \e sinsqcossq_glitchcount  - the configured glitchcount for sinsq + cossq error
     * \e uint16_t \e sinsqcossq_cossq        - actual cossq value on error trigger either lo of hi, first event
     * \e uint16_t \e sinsqcossq_sinsq        - actual sinsq value on error trigger either lo of hi, first event
     * \e bool     \e sinsqcossq_hi           - the status of the flag in the Interrupt status
     * \e bool     \e sinsqcossq_lo           - the status of the flag in the Interrupt status
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_Signal_Integrity_SinSq_CosSq struct as input
     */
    static inline void
    RDC_getDiagnosticsSignalIntegritySquareSumData(
        uint32_t base,
        uint32_t resolverCore,
        Diag_Mon_Signal_Integrity_SinSq_CosSq *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG9_0 + (resolverCore * RDC_CORE_OFFSET);
        uint32_t value = HW_RD_REG32(
            base + regOffset);

        monitorData->sinsqcossq_threshold_hi = (value & CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_HI_MASK) >>
                                               CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_HI_SHIFT;
        monitorData->sinsqcossq_threshold_lo = (value & CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_LO_MASK) >>
                                               CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_LO_SHIFT;

        regOffset = CSL_RESOLVER_REGS_DIAG10_0 + (resolverCore * RDC_CORE_OFFSET);
        value = HW_RD_REG32(
            base + regOffset);

        monitorData->sinsqcossq_cossq = (value & CSL_RESOLVER_REGS_DIAG10_0_SINSQCOSSQ_COSSQ_MASK) >>
                                        CSL_RESOLVER_REGS_DIAG10_0_SINSQCOSSQ_COSSQ_SHIFT;
        monitorData->sinsqcossq_sinsq = (value & CSL_RESOLVER_REGS_DIAG10_0_SINSQCOSSQ_SINSQ_MASK) >>
                                        CSL_RESOLVER_REGS_DIAG10_0_SINSQCOSSQ_SINSQ_SHIFT;

        uint32_t interruptStatus = RDC_getCoreInterruptStatus(base, resolverCore);

        regOffset = CSL_RESOLVER_REGS_DIAG11_0 + (resolverCore * RDC_CORE_OFFSET);
        monitorData->sinsqcossq_glitchcount = (uint8_t)((HW_RD_REG32(
                                                             base + regOffset) &
                                                         CSL_RESOLVER_REGS_DIAG11_0_SINSQCOSSQ_GLITCHCOUNT_MASK) >>
                                                        CSL_RESOLVER_REGS_DIAG11_0_SINSQCOSSQ_GLITCHCOUNT_SHIFT);

        monitorData->sinsqcossq_hi = ((interruptStatus & RDC_INTERRUPT_SOURCE_SINSQCOSSQ_HI_ERR) != 0);
        monitorData->sinsqcossq_lo = ((interruptStatus & RDC_INTERRUPT_SOURCE_SINSQCOSSQ_LO_ERR) != 0);
    }

    /**
     * @brief Sets the Monitor signal integrity by checking Sin2+Cos2=Constant (DOS) diagnostics Controls
     * \e uint16_t \e sinsqcossq_threshold_hi - the threshold hi for sinsq + cossq to be configured
     * \e uint16_t \e sinsqcossq_threshold_lo - the threshold lo for sinsq + cossq to be configured
     * \e uint8_t  \e sinsqcossq_glitchcount  - the glitchcount for sinsq + cossq error to be configured
     * \e uint16_t \e sinsqcossq_cossq        - no action
     * \e uint16_t \e sinsqcossq_sinsq        - no action
     * \e bool     \e sinsqcossq_hi           - enable interrupt on this error if set
     * \e bool     \e sinsqcossq_lo           - enable interrupt on this error if set
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_Signal_Integrity_SinSq_CosSq struct as input
     */
    static inline void
    RDC_setDiagnosticsSignalIntegritySquareSumData(
        uint32_t base,
        uint32_t resolverCore,
        Diag_Mon_Signal_Integrity_SinSq_CosSq *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG9_0 + (resolverCore * RDC_CORE_OFFSET);
        uint32_t value = (((uint32_t)((uint16_t)(monitorData->sinsqcossq_threshold_hi))) << CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_HI_SHIFT) |
                         (((uint32_t)((uint16_t)(monitorData->sinsqcossq_threshold_lo))) << CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_LO_SHIFT);
        HW_WR_REG32(
            base + regOffset, value);

        regOffset = CSL_RESOLVER_REGS_DIAG11_0 + (resolverCore * RDC_CORE_OFFSET);
        HW_WR_REG32(
            base + regOffset, (uint32_t)(monitorData->sinsqcossq_glitchcount));

        RDC_disableCoreInterrupt(base, resolverCore,
                                 (RDC_INTERRUPT_SOURCE_SINSQCOSSQ_HI_ERR | RDC_INTERRUPT_SOURCE_SINSQCOSSQ_LO_ERR));
        uint32_t enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);
        uint32_t interruptSources = 0;

        if (monitorData->sinsqcossq_hi)
        {
            interruptSources |= RDC_INTERRUPT_SOURCE_SINSQCOSSQ_HI_ERR;
        }
        if (monitorData->sinsqcossq_lo)
        {
            interruptSources |= RDC_INTERRUPT_SOURCE_SINSQCOSSQ_LO_ERR;
        }
        RDC_enableCoreInterrupt(base, resolverCore, (enabledInterruptSources | interruptSources));
    }



    /**
     * @brief Returns the Monitor Sin or Cos saturation or very high amplitude (DOS) diagnostics data
     * \e uint16_t \e highAmplitude_threshold     - the configured threshold for highAmplitude error
     * \e uint8_t  \e highAmplitude_glitchcount   - the configured glitch count for highAmplitude error
     * \e int16_t  \e highAmplitude_sin_value     - value of sin channel at the error trigger instant
     * \e int16_t  \e highAmplitude_cos_value     - value of cos channel at the error trigger instant
     * \e bool     \e highAmplitude_sin_error     - status of the error flag in the interrupt register
     * \e bool     \e highAmplitude_cos_error     - status of the error flag in the interrupt register
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_Sin_Cos_High_Amplitude type struct as input
     */
    static inline void
    RDC_getDiagnosticsHighAmplitudeData(
        uint32_t base,
        uint8_t resolverCore,
        Diag_Mon_Sin_Cos_High_Amplitude *monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG7_0 + (resolverCore * RDC_CORE_OFFSET);
        uint32_t value = HW_RD_REG32(
            base + regOffset);
        uint32_t interruptStatus = RDC_getCoreInterruptStatus(base, resolverCore);

        monitorData->highAmplitude_glitchcount = (value & CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_GLITCHCOUNT_MASK) >>
                                                 CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_GLITCHCOUNT_SHIFT;

        monitorData->highAmplitude_threshold = (value & CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_THRESHOLD_MASK) >>
                                               CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_THRESHOLD_SHIFT;
        regOffset = CSL_RESOLVER_REGS_DIAG8_0 + (resolverCore * RDC_CORE_OFFSET);
        value = HW_RD_REG32(
            base + regOffset);

        monitorData->highAmplitude_sin_value = (int16_t) ((value & CSL_RESOLVER_REGS_DIAG8_0_HIGHAMPLITUDE_SIN_MASK) >> \
                                                 CSL_RESOLVER_REGS_DIAG8_0_HIGHAMPLITUDE_SIN_SHIFT);

        monitorData->highAmplitude_cos_value = (int16_t) ((value & CSL_RESOLVER_REGS_DIAG8_0_HIGHAMPLITUDE_COS_MASK) >> \
                                                 CSL_RESOLVER_REGS_DIAG8_0_HIGHAMPLITUDE_COS_SHIFT);
        monitorData->highAmplitude_cos_error = ((interruptStatus & RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_COS_FAULT_ERR) != 0);
        monitorData->highAmplitude_sin_error = ((interruptStatus & RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_SIN_FAULT_ERR) != 0);
    }

    /**
     * @brief Sets the Monitor Sin or Cos saturation or very high amplitude (DOS) diagnostics Controls
     * \e uint16_t \e highAmplitude_threshold     - the threshold for highAmplitude error to be configured
     * \e uint8_t  \e highAmplitude_glitchcount   - the glitch count for highAmplitude error to be configured
     * \e int16_t  \e highAmplitude_sin_value     - no action
     * \e int16_t  \e highAmplitude_cos_value     - no action
     * \e bool     \e highAmplitude_sin_error     - enabled interrupt on this error if set
     * \e bool     \e highAmplitude_cos_error     - enabled interrupt on this error if set
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_Sin_Cos_High_Amplitude type struct as input
     */
    static inline void
    RDC_setDiagnosticsHighAmplitudeData(
        uint32_t base,
        uint8_t resolverCore,
        Diag_Mon_Sin_Cos_High_Amplitude* monitorData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG7_0 + (resolverCore * RDC_CORE_OFFSET);
        uint32_t value = (((uint32_t)((uint16_t)(monitorData->highAmplitude_glitchcount))) << CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_GLITCHCOUNT_SHIFT) |
                         (((uint32_t)((uint16_t)(monitorData->highAmplitude_threshold))) << CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_THRESHOLD_SHIFT);
        uint32_t enabledInterruptSources = 0;
        uint32_t interruptSources = 0;
        HW_WR_REG32(
            base + regOffset,
            value);
        RDC_disableCoreInterrupt(base, resolverCore, (RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_COS_FAULT_ERR | RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_SIN_FAULT_ERR));
        enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);

        if(monitorData->highAmplitude_cos_error)
        {
            interruptSources |= RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_COS_FAULT_ERR;
        }
        if(monitorData->highAmplitude_sin_error)
        {
            interruptSources |= RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_SIN_FAULT_ERR;
        }
        RDC_enableCoreInterrupt(base, resolverCore, (interruptSources | enabledInterruptSources));
    }

    /**
     * @brief Returns the Monitor weak Sin or Cos signal below a threshold (LOS) diagnostics data
     * \e uint16_t \e lowAmplitude_threshold      - the configured threshold for the lowAmplitude error
     * \e uint8_t  \e lowAmplitude_glitchcount    - the configured glitch count for the lowAmplitude error
     * \e bool     \e lowAmplitude_error          - status of the error flag in the interrupt register
     * \e int16_t  \e lowAmplitude_sin_value      - value of sin channel at the trigger instant
     * \e int16_t  \e lowAmplitude_cos_value      - value of cos channel at the trigger instant
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_Sin_Cos_Weak_Amplitude type struct as input
     */
    static inline void
    RDC_getDiagnosticsWeakAmplitudeData(
        uint32_t base,
        uint8_t resolverCore,
        Diag_Mon_Sin_Cos_Weak_Amplitude * monitorData
    )
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG5_0 + (resolverCore * RDC_CORE_OFFSET);
        uint32_t interruptSources = RDC_getCoreInterruptStatus(base, resolverCore);
        uint32_t value = HW_RD_REG32(
            base + regOffset);
        monitorData->lowAmplitude_threshold = (value & CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_THRESHOLD_MASK) >>
                                              CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_THRESHOLD_SHIFT;
        monitorData->lowAmplitude_glitchcount = (value & CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_GLITCHCOUNT_MASK) >>
                                                CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_GLITCHCOUNT_SHIFT;

        monitorData->lowAmplitude_error = ((interruptSources & RDC_INTERRUPT_SOURCE_LOWAMPLITUDE_ERR) != 0U);

        regOffset = CSL_RESOLVER_REGS_DIAG6_0 + (resolverCore * RDC_CORE_OFFSET);
        value = HW_RD_REG32(
            base + regOffset);
        monitorData->lowAmplitude_cos_value = (int16_t)((value & CSL_RESOLVER_REGS_DIAG6_0_LOWAMPLITUDE_COS_MASK) >>\
        CSL_RESOLVER_REGS_DIAG6_0_LOWAMPLITUDE_COS_SHIFT);

        monitorData->lowAmplitude_sin_value = (int16_t)((value & CSL_RESOLVER_REGS_DIAG6_0_LOWAMPLITUDE_SIN_MASK) >>\
        CSL_RESOLVER_REGS_DIAG6_0_LOWAMPLITUDE_SIN_SHIFT);
    }

    /**
     * @brief Sets the Monitor weak Sin or Cos signal below a threshold (LOS) diagnostics Controls
     * \e uint16_t \e lowAmplitude_threshold      - the threshold for the lowAmplitude error to be configured
     * \e uint8_t  \e lowAmplitude_glitchcount    - the glitch count for the lowAmplitude error to be configured
     * \e bool     \e lowAmplitude_error          - enabled interrupt on the error if set
     * \e int16_t  \e lowAmplitude_sin_value      - no action
     * \e int16_t  \e lowAmplitude_cos_value      - no action
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param monitorData Diag_Mon_Sin_Cos_Weak_Amplitude type struct as input
     */
    static inline void
    RDC_setDiagnosticsWeakAmplitudeData(
        uint32_t base,
        uint8_t resolverCore,
        Diag_Mon_Sin_Cos_Weak_Amplitude * monitorData
    )
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_DIAG5_0 + (resolverCore * RDC_CORE_OFFSET);
        uint32_t value = ((uint32_t)(monitorData->lowAmplitude_glitchcount << CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_GLITCHCOUNT_SHIFT)) |
                            ((uint32_t) (monitorData->lowAmplitude_threshold << CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_THRESHOLD_SHIFT));
        uint32_t enabledInterruptSources = RDC_getCoreEnabledInterruptSources(base, resolverCore);

        HW_WR_REG32(
            base + regOffset, value);

        if(monitorData->lowAmplitude_error)
        {
            RDC_enableCoreInterrupt(base, resolverCore, (enabledInterruptSources | RDC_INTERRUPT_SOURCE_LOWAMPLITUDE_ERR));
        }
        else
        {
            RDC_disableCoreInterrupt(base, resolverCore, (RDC_INTERRUPT_SOURCE_LOWAMPLITUDE_ERR));
        }
    }

    //*****************************************************************************
    // Observational Data
    //*****************************************************************************

    /**
     * @brief Returns the Observational ADC data to struct type ADC_observationalData
     * \e int16_t \e cos_adc    - SW Observational ADC data post latch and Averaged if enabled
     * \e int16_t \e sin_adc    - SW Observational ADC data post latch and Averaged if enabled
     * \e int16_t \e cos_rec    - SW Observational ADC data post recovered
     * \e int16_t \e sin_rec    - SW Observational ADC data post recovered
     * \e int16_t \e cos_dc     - SW Observational ADC data post DC offset Correction
     * \e int16_t \e sin_dc     - SW Observational ADC data post DC offset Correction
     * \e int16_t \e cos_pgc    - SW Observational ADC data post Phase Gain Correction
     * \e int16_t \e sin_pgc    - SW Observational ADC data post Phase Gain Correction
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param AdcData ADC_observationalData type struct as input
     */
    static inline void
    RDC_getAdcObservationalData(
        uint32_t base,
        uint8_t resolverCore,
        ADC_observationalData * AdcData)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_OBS_ADC_0 + (resolverCore * RDC_CORE_OFFSET);
        uint32_t value = HW_RD_REG32(
            base + regOffset);
        AdcData->cos_adc = (int16_t)((value & CSL_RESOLVER_REGS_OBS_ADC_1_COS_ADC_MASK)>>CSL_RESOLVER_REGS_OBS_ADC_1_COS_ADC_SHIFT);
        AdcData->sin_adc = (int16_t)((value & CSL_RESOLVER_REGS_OBS_ADC_1_SIN_ADC_MASK)>>CSL_RESOLVER_REGS_OBS_ADC_1_SIN_ADC_SHIFT);

        regOffset = CSL_RESOLVER_REGS_OBS_ADC_REC_0 + (resolverCore * RDC_CORE_OFFSET);
        value = HW_RD_REG32(
            base + regOffset);
        AdcData->cos_rec = (int16_t)((value & CSL_RESOLVER_REGS_OBS_ADC_REC_1_COS_REC_MASK)>>CSL_RESOLVER_REGS_OBS_ADC_REC_1_COS_REC_SHIFT);
        AdcData->sin_rec = (int16_t)((value & CSL_RESOLVER_REGS_OBS_ADC_REC_1_SIN_REC_MASK)>>CSL_RESOLVER_REGS_OBS_ADC_REC_1_SIN_REC_SHIFT);

        regOffset = CSL_RESOLVER_REGS_OBS_ADC_DC_0 + (resolverCore * RDC_CORE_OFFSET);
        value = HW_RD_REG32(
            base + regOffset);
        AdcData->cos_dc = (int16_t)((value & CSL_RESOLVER_REGS_OBS_ADC_DC_1_COS_DC_MASK)>>CSL_RESOLVER_REGS_OBS_ADC_DC_1_COS_DC_SHIFT);
        AdcData->sin_dc = (int16_t)((value & CSL_RESOLVER_REGS_OBS_ADC_DC_1_SIN_DC_MASK)>>CSL_RESOLVER_REGS_OBS_ADC_DC_1_SIN_DC_SHIFT);

        regOffset = CSL_RESOLVER_REGS_OBS_ADC_PGC_0 + (resolverCore * RDC_CORE_OFFSET);
        value = HW_RD_REG32(
            base + regOffset);
        AdcData->cos_pgc = (int16_t)((value & CSL_RESOLVER_REGS_OBS_ADC_PGC_1_COS_PGC_MASK)>>CSL_RESOLVER_REGS_OBS_ADC_PGC_1_COS_PGC_SHIFT);
        AdcData->sin_pgc = (int16_t)((value & CSL_RESOLVER_REGS_OBS_ADC_PGC_1_SIN_PGC_MASK)>>CSL_RESOLVER_REGS_OBS_ADC_PGC_1_SIN_PGC_SHIFT);
    }

    /**
     * @brief Returns the Peak Histogram Bucket data.
     *
     * @param base RDC Base Address
     * @param resolverCore Denotes the Resolver Core within the RDC
     * @param histogram PeakHistogram_observationalData type struct as input
     */
    static inline void
    RDC_getPeakHistogramObservationalData(uint32_t base, uint8_t resolverCore, PeakHistogram_observationalData* histogram)
    {
        uint32_t regOffset = CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0 + (resolverCore * RDC_CORE_OFFSET);
        uint32_t value = HW_RD_REG32(
            base + regOffset);
        histogram->peakHistgoramBucket[0] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM0_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM0_0_SHIFT);
        histogram->peakHistgoramBucket[1] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM1_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM1_0_SHIFT);
        histogram->peakHistgoramBucket[2] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM2_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM2_0_SHIFT);
        histogram->peakHistgoramBucket[3] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM3_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM3_0_SHIFT);

        regOffset = CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0 + (resolverCore * RDC_CORE_OFFSET);
        value = HW_RD_REG32(
            base + regOffset);
        histogram->peakHistgoramBucket[4] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM4_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM4_0_SHIFT);
        histogram->peakHistgoramBucket[5] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM5_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM5_0_SHIFT);
        histogram->peakHistgoramBucket[6] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM6_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM6_0_SHIFT);
        histogram->peakHistgoramBucket[7] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM7_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM7_0_SHIFT);

        regOffset = CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0 + (resolverCore * RDC_CORE_OFFSET);
        value = HW_RD_REG32(
            base + regOffset);
        histogram->peakHistgoramBucket[8] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM8_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM8_0_SHIFT);
        histogram->peakHistgoramBucket[9] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM9_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM9_0_SHIFT);
        histogram->peakHistgoramBucket[10] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM10_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM10_0_SHIFT);
        histogram->peakHistgoramBucket[11] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM11_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM11_0_SHIFT);

        regOffset = CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0 + (resolverCore * RDC_CORE_OFFSET);
        value = HW_RD_REG32(
            base + regOffset);
        histogram->peakHistgoramBucket[12] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM12_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM12_0_SHIFT);
        histogram->peakHistgoramBucket[13] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM13_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM13_0_SHIFT);
        histogram->peakHistgoramBucket[14] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM14_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM14_0_SHIFT);
        histogram->peakHistgoramBucket[15] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM15_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM15_0_SHIFT);

        regOffset = CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0 + (resolverCore * RDC_CORE_OFFSET);
        value = HW_RD_REG32(
            base + regOffset);
        histogram->peakHistgoramBucket[16] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM16_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM16_0_SHIFT);
        histogram->peakHistgoramBucket[17] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM17_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM17_0_SHIFT);
        histogram->peakHistgoramBucket[18] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM18_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM18_0_SHIFT);
        histogram->peakHistgoramBucket[19] = ((value & CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM19_0_MASK) >> CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM19_0_SHIFT);
    }

    /**
     * @brief Inits the Core Parameters for the resolver core
     *
     * @param coreParams
     */
    extern void
    RDC_coreParamsInit(Core_config_t* coreParams);
    /**
     * @brief Inits the resolver Configuration parameters
     *
     * @param params
     */
    extern void
    RDC_paramsInit(RDC_configParams* params);
    /**
     * @brief Configures the RDC based on the parameter values
     * \note this does not include the diagnostics configurations
     *
     * @param base
     * @param params
     */
    extern void
    RDC_init(uint32_t base, RDC_configParams* params);  

    /**
     * @brief Returns the Static Configurations 
     * \note this does not include the diagnostics configurations
     * 
     * @param base 
     * @param params 
     */
    extern void 
    RDC_getStaticConfigurations(uint32_t base, RDC_configParams* params);

    /**
     * @brief Returns if the configurations in paramsInit to params
     * \note this verification does not include the diagnostics configurations
     * 
     * @param base RDC Base Address
     * @param paramsInit 
     * @param params
     * @return int32_t SystemP_SUCCESS is verification of configurations is success 
     * @return int32_t SystemP_FAILURE is verification of configurations is failure 
     */
    extern int32_t
    RDC_verifyStaticConfigurations(uint32_t base, RDC_configParams* paramsInit, RDC_configParams* params);

    /**
     * @brief Inits Baseline Parameter configurations
     *
     * @param base
     */
    extern void
    RDC_BaselineParametersInit(uint32_t base);
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // RESOLVER_V1_H_

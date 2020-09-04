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
 *  \defgroup DRV_ECAP_MODULE APIs for ECAP
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the ECAP module.
 *
 *  @{
 */

#ifndef ECAP_V1_H_
#define ECAP_V1_H_

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
#include <drivers/hw_include/cslr_ecap.h>

//*****************************************************************************
//
// eCAP minimum and maximum values
//
//*****************************************************************************
#define ECAP_MAX_PRESCALER_VALUE    (32U)  // Maximum Pre-scaler value

//*****************************************************************************
//
// Values that can be passed to ECAP_enableInterrupt(),
// ECAP_disableInterrupt(), ECAP_clearInterrupt() and ECAP_forceInterrupt() as
// the intFlags parameter and returned by ECAP_getInterruptSource().
//
//*****************************************************************************
//! Event 1 ISR source
#define ECAP_ISR_SOURCE_CAPTURE_EVENT_1    (0x2U)
//! Event 2 ISR source
#define ECAP_ISR_SOURCE_CAPTURE_EVENT_2    (0x4U)
//! Event 3 ISR source
#define ECAP_ISR_SOURCE_CAPTURE_EVENT_3    (0x8U)
//! Event 4 ISR source
#define ECAP_ISR_SOURCE_CAPTURE_EVENT_4    (0x10U)
//! Counter overflow ISR source
#define ECAP_ISR_SOURCE_COUNTER_OVERFLOW    (0x20U)
//! Counter equals period ISR source
#define ECAP_ISR_SOURCE_COUNTER_PERIOD    (0x40U)
//! Counter equals compare ISR source
#define ECAP_ISR_SOURCE_COUNTER_COMPARE    (0x80U)
//! High resolution error ISR source
#define ECAP_ISR_SOURCE_HR_ERROR    (0x100U)
//! All ISR source
#define ECAP_ISR_SOURCE_ALL    (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |\
                                ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |\
                                ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |\
                                ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |\
                                ECAP_ISR_SOURCE_COUNTER_OVERFLOW |\
                                ECAP_ISR_SOURCE_COUNTER_PERIOD   |\
                                ECAP_ISR_SOURCE_COUNTER_COMPARE  |\
                                ECAP_ISR_SOURCE_HR_ERROR)

//*****************************************************************************
//
// Values that can be passed to HRCAP_enableCalibrationInterrupt(),
// HRCAP_disableCalibrationInterrupt() as the intFlags parameter and
// HRCAP_clearCalibrationFlags() and HRCAP_forceCalibrationFlags() as the flags
// parameter and returned by HRCAP_getCalibrationFlags().
//
//*****************************************************************************
//! Global calibration interrupt flag
#define HRCAP_GLOBAL_CALIBRATION_INTERRUPT    (0x1U)
//! Calibration done flag
#define HRCAP_CALIBRATION_DONE    (0x2U)
//! Calibration period overflow flag
#define HRCAP_CALIBRATION_PERIOD_OVERFLOW    (0x4U)

//*****************************************************************************
//
//! Values that can be passed to ECAP_setEmulationMode() as the
//! \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    //! TSCTR is stopped on emulation suspension
    ECAP_EMULATION_STOP = 0x0U,
    //! TSCTR runs until 0 before stopping on emulation suspension
    ECAP_EMULATION_RUN_TO_ZERO = 0x1U,
    //! TSCTR is not affected by emulation suspension
    ECAP_EMULATION_FREE_RUN = 0x2U
}ECAP_EmulationMode;

//*****************************************************************************
//
//! Values that can be passed to ECAP_setCaptureMode() as the
//! \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    //! eCAP operates in continuous capture mode
    ECAP_CONTINUOUS_CAPTURE_MODE = 0U,
    //! eCAP operates in one shot capture mode
    ECAP_ONE_SHOT_CAPTURE_MODE = 1U
}ECAP_CaptureMode;

//*****************************************************************************
//
//! Values that can be passed to ECAP_setEventPolarity(),ECAP_setCaptureMode(),
//! ECAP_enableCounterResetOnEvent(),ECAP_disableCounterResetOnEvent(),
//! ECAP_getEventTimeStamp(),ECAP_setDMASource() as the \e event parameter.
//
//*****************************************************************************
typedef enum
{
    ECAP_EVENT_1 = 0U,   //!< eCAP event 1
    ECAP_EVENT_2 = 1U,   //!< eCAP event 2
    ECAP_EVENT_3 = 2U,   //!< eCAP event 3
    ECAP_EVENT_4 = 3U    //!< eCAP event 4
}ECAP_Events;

//*****************************************************************************
//
//! Values that can be passed to ECAP_setSyncOutMode() as the \e mode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! sync out on the sync in signal and software force
    ECAP_SYNC_OUT_SYNCI = 0x00,
    //! sync out on counter equals period
    ECAP_SYNC_OUT_COUNTER_PRD = 0x40,
    //! Disable sync out signal
    ECAP_SYNC_OUT_DISABLED = 0x80
}ECAP_SyncOutMode;

//*****************************************************************************
//
//! Values that can be passed to ECAP_setAPWMPolarity() as the \e polarity
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ECAP_APWM_ACTIVE_HIGH = 0x000, //!< APWM is active high
    ECAP_APWM_ACTIVE_LOW = 0x400  //!< APWM is active low
}ECAP_APWMPolarity;

//*****************************************************************************
//
//! Values that can be passed to ECAP_setEventPolarity() as the \e polarity
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ECAP_EVNT_RISING_EDGE = 0U, //!< Rising edge polarity
    ECAP_EVNT_FALLING_EDGE = 1U  //!< Falling edge polarity
}ECAP_EventPolarity;

//*****************************************************************************
//
//! Values that can be passed to ECAP_selectECAPInput() as the \e input
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! Capture input is FSI_RX0 Trigger 0
    ECAP_INPUT_FSI_RX0_TRIG_0 = 0,
    //! Capture input is FSI_RX0 Trigger 1
    ECAP_INPUT_FSI_RX0_TRIG_1 = 1,
    //! Capture input is FSI_RX0 Trigger 2
    ECAP_INPUT_FSI_RX0_TRIG_2 = 2,
    //! Capture input is FSI_RX0 Trigger 3
    ECAP_INPUT_FSI_RX0_TRIG_3 = 3,
    //! Capture input is FSI_RX1 Trigger 0
    ECAP_INPUT_FSI_RX1_TRIG_0 = 4,
    //! Capture input is FSI_RX1 Trigger 1
    ECAP_INPUT_FSI_RX1_TRIG_1 = 5,
    //! Capture input is FSI_RX1 Trigger 2
    ECAP_INPUT_FSI_RX1_TRIG_2 = 6,
    //! Capture input is FSI_RX1 Trigger 3
    ECAP_INPUT_FSI_RX1_TRIG_3 = 7,
    //! Capture input is FSI_RX2 Trigger 0
    ECAP_INPUT_FSI_RX2_TRIG_0 = 8,
    //! Capture input is FSI_RX2 Trigger 1
    ECAP_INPUT_FSI_RX2_TRIG_1 = 9,
    //! Capture input is FSI_RX2 Trigger 2
    ECAP_INPUT_FSI_RX2_TRIG_2 = 10,
    //! Capture input is FSI_RX2 Trigger 3
    ECAP_INPUT_FSI_RX2_TRIG_3 = 11,
    //! Capture input is FSI_RX3 Trigger 0
    ECAP_INPUT_FSI_RX3_TRIG_0 = 12,
    //! Capture input is FSI_RX3 Trigger 1
    ECAP_INPUT_FSI_RX3_TRIG_1 = 13,
    //! Capture input is FSI_RX3 Trigger 2
    ECAP_INPUT_FSI_RX3_TRIG_2 = 14,
    //! Capture input is FSI_RX3 Trigger 3
    ECAP_INPUT_FSI_RX3_TRIG_3 = 15,
    //! Capture input is EQEP0 QI Signal
    ECAP_INPUT_EQEP0_QI = 16,
    //! Capture input is EQEP0 QS Signal
    ECAP_INPUT_EQEP0_QS = 17,
    //! Capture input is EQEP1 QI Signal
    ECAP_INPUT_EQEP1_QI = 18,
    //! Capture input is EQEP1 QS Signal
    ECAP_INPUT_EQEP1_QS = 19,
    //! Capture input is EQEP2 QI Signal
    ECAP_INPUT_EQEP2_QI = 20,
    //! Capture input is EQEP2 QS Signal
    ECAP_INPUT_EQEP2_QS = 21,
    //! Capture input is EPWM0 SOC-A Signal
    ECAP_INPUT_EPWM0_SOCA = 54,
    //! Capture input is EPWM1 SOC-A Signal
    ECAP_INPUT_EPWM1_SOCA = 55,
    //! Capture input is EPWM2 SOC-A Signal
    ECAP_INPUT_EPWM2_SOCA = 56,
    //! Capture input is EPWM3 SOC-A Signal
    ECAP_INPUT_EPWM3_SOCA = 57,
    //! Capture input is EPWM4 SOC-A Signal
    ECAP_INPUT_EPWM4_SOCA = 58,
    //! Capture input is EPWM5 SOC-A Signal
    ECAP_INPUT_EPWM5_SOCA = 59,
    //! Capture input is EPWM6 SOC-A Signal
    ECAP_INPUT_EPWM6_SOCA = 60,
    //! Capture input is EPWM7 SOC-A Signal
    ECAP_INPUT_EPWM7_SOCA = 61,
    //! Capture input is EPWM8 SOC-A Signal
    ECAP_INPUT_EPWM8_SOCA = 62,
    //! Capture input is EPWM9 SOC-A Signal
    ECAP_INPUT_EPWM9_SOCA = 63,
    //! Capture input is EPWM10 SOC-A Signal
    ECAP_INPUT_EPWM10_SOCA = 64,
    //! Capture input is EPWM11 SOC-A Signal
    ECAP_INPUT_EPWM11_SOCA = 65,
    //! Capture input is EPWM12 SOC-A Signal
    ECAP_INPUT_EPWM12_SOCA = 66,
    //! Capture input is EPWM13 SOC-A Signal
    ECAP_INPUT_EPWM13_SOCA = 67,
    //! Capture input is EPWM14 SOC-A Signal
    ECAP_INPUT_EPWM14_SOCA = 68,
    //! Capture input is EPWM15 SOC-A Signal
    ECAP_INPUT_EPWM15_SOCA = 69,
    //! Capture input is EPWM16 SOC-A Signal
    ECAP_INPUT_EPWM16_SOCA = 70,
    //! Capture input is EPWM17 SOC-A Signal
    ECAP_INPUT_EPWM17_SOCA = 71,
    //! Capture input is EPWM18 SOC-A Signal
    ECAP_INPUT_EPWM18_SOCA = 72,
    //! Capture input is EPWM19 SOC-A Signal
    ECAP_INPUT_EPWM19_SOCA = 73,
    //! Capture input is EPWM20 SOC-A Signal
    ECAP_INPUT_EPWM20_SOCA = 74,
    //! Capture input is EPWM21 SOC-A Signal
    ECAP_INPUT_EPWM21_SOCA = 75,
    //! Capture input is EPWM22 SOC-A Signal
    ECAP_INPUT_EPWM22_SOCA = 76,
    //! Capture input is EPWM23 SOC-A Signal
    ECAP_INPUT_EPWM23_SOCA = 77,
    //! Capture input is EPWM24 SOC-A Signal
    ECAP_INPUT_EPWM24_SOCA = 78,
    //! Capture input is EPWM25 SOC-A Signal
    ECAP_INPUT_EPWM25_SOCA = 79,
    //! Capture input is EPWM26 SOC-A Signal
    ECAP_INPUT_EPWM26_SOCA = 80,
    //! Capture input is EPWM27 SOC-A Signal
    ECAP_INPUT_EPWM27_SOCA = 81,
    //! Capture input is EPWM28 SOC-A Signal
    ECAP_INPUT_EPWM28_SOCA = 82,
    //! Capture input is EPWM29 SOC-A Signal
    ECAP_INPUT_EPWM29_SOCA = 83,
    //! Capture input is EPWM30 SOC-A Signal
    ECAP_INPUT_EPWM30_SOCA = 84,
    //! Capture input is EPWM31 SOC-A Signal
    ECAP_INPUT_EPWM31_SOCA = 85,
    //! Capture input is EPWM0 SOC-B Signal
    ECAP_INPUT_EPWM0_SOCB = 86,
    //! Capture input is EPWM1 SOC-B Signal
    ECAP_INPUT_EPWM1_SOCB = 87,
    //! Capture input is EPWM2 SOC-B Signal
    ECAP_INPUT_EPWM2_SOCB = 88,
    //! Capture input is EPWM3 SOC-B Signal
    ECAP_INPUT_EPWM3_SOCB = 89,
    //! Capture input is EPWM4 SOC-B Signal
    ECAP_INPUT_EPWM4_SOCB = 90,
    //! Capture input is EPWM5 SOC-B Signal
    ECAP_INPUT_EPWM5_SOCB = 91,
    //! Capture input is EPWM6 SOC-B Signal
    ECAP_INPUT_EPWM6_SOCB = 92,
    //! Capture input is EPWM7 SOC-B Signal
    ECAP_INPUT_EPWM7_SOCB = 93,
    //! Capture input is EPWM8 SOC-B Signal
    ECAP_INPUT_EPWM8_SOCB = 94,
    //! Capture input is EPWM9 SOC-B Signal
    ECAP_INPUT_EPWM9_SOCB = 95,
    //! Capture input is EPWM10 SOC-B Signal
    ECAP_INPUT_EPWM10_SOCB = 96,
    //! Capture input is EPWM11 SOC-B Signal
    ECAP_INPUT_EPWM11_SOCB = 97,
    //! Capture input is EPWM12 SOC-B Signal
    ECAP_INPUT_EPWM12_SOCB = 98,
    //! Capture input is EPWM13 SOC-B Signal
    ECAP_INPUT_EPWM13_SOCB = 99,
    //! Capture input is EPWM14 SOC-B Signal
    ECAP_INPUT_EPWM14_SOCB = 100,
    //! Capture input is EPWM15 SOC-B Signal
    ECAP_INPUT_EPWM15_SOCB = 101,
    //! Capture input is EPWM16 SOC-B Signal
    ECAP_INPUT_EPWM16_SOCB = 102,
    //! Capture input is EPWM17 SOC-B Signal
    ECAP_INPUT_EPWM17_SOCB = 103,
    //! Capture input is EPWM18 SOC-B Signal
    ECAP_INPUT_EPWM18_SOCB = 104,
    //! Capture input is EPWM19 SOC-B Signal
    ECAP_INPUT_EPWM19_SOCB = 105,
    //! Capture input is EPWM20 SOC-B Signal
    ECAP_INPUT_EPWM20_SOCB = 106,
    //! Capture input is EPWM21 SOC-B Signal
    ECAP_INPUT_EPWM21_SOCB = 107,
    //! Capture input is EPWM22 SOC-B Signal
    ECAP_INPUT_EPWM22_SOCB = 108,
    //! Capture input is EPWM23 SOC-B Signal
    ECAP_INPUT_EPWM23_SOCB = 109,
    //! Capture input is EPWM24 SOC-B Signal
    ECAP_INPUT_EPWM24_SOCB = 110,
    //! Capture input is EPWM25 SOC-B Signal
    ECAP_INPUT_EPWM25_SOCB = 111,
    //! Capture input is EPWM26 SOC-B Signal
    ECAP_INPUT_EPWM26_SOCB = 112,
    //! Capture input is EPWM27 SOC-B Signal
    ECAP_INPUT_EPWM27_SOCB = 113,
    //! Capture input is EPWM28 SOC-B Signal
    ECAP_INPUT_EPWM28_SOCB = 114,
    //! Capture input is EPWM29 SOC-B Signal
    ECAP_INPUT_EPWM29_SOCB = 115,
    //! Capture input is EPWM30 SOC-B Signal
    ECAP_INPUT_EPWM30_SOCB = 116,
    //! Capture input is EPWM31 SOC-B Signal
    ECAP_INPUT_EPWM31_SOCB = 117,
    //! Capture input is SDFM0 Compare1 High
    ECAP_INPUT_SDFM0_COMPARE1_HIGH = 118,
    //! Capture input is SDFM0 Compare1 Low
    ECAP_INPUT_SDFM0_COMPARE1_LOW = 119,
    //! Capture input is SDFM0 Compare Z1
    ECAP_INPUT_SDFM0_COMPARE_Z1 = 120,
    //! Capture input is SDFM0 Compare2 High
    ECAP_INPUT_SDFM0_COMPARE2_HIGH = 121,
    //! Capture input is SDFM0 Compare2 Low
    ECAP_INPUT_SDFM0_COMPARE2_LOW = 122,
    //! Capture input is SDFM0 Compare Z2
    ECAP_INPUT_SDFM0_COMPARE_Z2 = 123,
    //! Capture input is SDFM0 Compare3 High
    ECAP_INPUT_SDFM0_COMPARE3_HIGH = 124,
    //! Capture input is SDFM0 Compare3 Low
    ECAP_INPUT_SDFM0_COMPARE3_LOW = 125,
    //! Capture input is SDFM0 Compare Z3
    ECAP_INPUT_SDFM0_COMPARE_Z3 = 126,
    //! Capture input is SDFM0 Compare4 High
    ECAP_INPUT_SDFM0_COMPARE4_HIGH = 127,
    //! Capture input is SDFM0 Compare4 Low
    ECAP_INPUT_SDFM0_COMPARE4_LOW = 128,
    //! Capture input is SDFM0 Compare Z4
    ECAP_INPUT_SDFM0_COMPARE_Z4 = 129,
    //! Capture input is SDFM1 Compare1 High
    ECAP_INPUT_SDFM1_COMPARE1_HIGH = 130,
    //! Capture input is SDFM1 Compare1 Low
    ECAP_INPUT_SDFM1_COMPARE1_LOW = 131,
    //! Capture input is SDFM1 Compare Z1
    ECAP_INPUT_SDFM1_COMPARE_Z1 = 132,
    //! Capture input is SDFM1 Compare2 High
    ECAP_INPUT_SDFM1_COMPARE2_HIGH = 133,
    //! Capture input is SDFM1 Compare2 Low
    ECAP_INPUT_SDFM1_COMPARE2_LOW = 134,
    //! Capture input is SDFM1 Compare Z2
    ECAP_INPUT_SDFM1_COMPARE_Z2 = 135,
    //! Capture input is SDFM1 Compare3 High
    ECAP_INPUT_SDFM1_COMPARE3_HIGH = 136,
    //! Capture input is SDFM1 Compare3 Low
    ECAP_INPUT_SDFM1_COMPARE3_LOW = 137,
    //! Capture input is SDFM1 Compare Z3
    ECAP_INPUT_SDFM1_COMPARE_Z3 = 138,
    //! Capture input is SDFM1 Compare4 High
    ECAP_INPUT_SDFM1_COMPARE4_HIGH = 139,
    //! Capture input is SDFM1 Compare4 Low
    ECAP_INPUT_SDFM1_COMPARE4_LOW = 140,
    //! Capture input is SDFM1 Compare Z4
    ECAP_INPUT_SDFM1_COMPARE_Z4 = 141,
    //! Capture input is CMPSSA0 CTRIP_LOW
    ECAP_INPUT_CMPSSA0_CTRIP_LOW = 142,
    //! Capture input is CMPSSA0 CTRIP_HIGH
    ECAP_INPUT_CMPSSA0_CTRIP_HIGH = 143,
    //! Capture input is CMPSSA1 CTRIP_LOW
    ECAP_INPUT_CMPSSA1_CTRIP_LOW = 144,
    //! Capture input is CMPSSA1 CTRIP_HIGH
    ECAP_INPUT_CMPSSA1_CTRIP_HIGH = 145,
    //! Capture input is CMPSSA2 CTRIP_LOW
    ECAP_INPUT_CMPSSA2_CTRIP_LOW = 146,
    //! Capture input is CMPSSA2 CTRIP_HIGH
    ECAP_INPUT_CMPSSA2_CTRIP_HIGH = 147,
    //! Capture input is CMPSSA3 CTRIP_LOW
    ECAP_INPUT_CMPSSA3_CTRIP_LOW = 148,
    //! Capture input is CMPSSA3 CTRIP_HIGH
    ECAP_INPUT_CMPSSA3_CTRIP_HIGH = 149,
    //! Capture input is CMPSSA4 CTRIP_LOW
    ECAP_INPUT_CMPSSA4_CTRIP_LOW = 150,
    //! Capture input is CMPSSA4 CTRIP_HIGH
    ECAP_INPUT_CMPSSA4_CTRIP_HIGH = 151,
    //! Capture input is CMPSSA5 CTRIP_LOW
    ECAP_INPUT_CMPSSA5_CTRIP_LOW = 152,
    //! Capture input is CMPSSA5 CTRIP_HIGH
    ECAP_INPUT_CMPSSA5_CTRIP_HIGH = 153,
    //! Capture input is CMPSSA6 CTRIP_LOW
    ECAP_INPUT_CMPSSA6_CTRIP_LOW = 154,
    //! Capture input is CMPSSA6 CTRIP_HIGH
    ECAP_INPUT_CMPSSA6_CTRIP_HIGH = 155,
    //! Capture input is CMPSSA7 CTRIP_LOW
    ECAP_INPUT_CMPSSA7_CTRIP_LOW = 156,
    //! Capture input is CMPSSA7 CTRIP_HIGH
    ECAP_INPUT_CMPSSA7_CTRIP_HIGH = 157,
    //! Capture input is CMPSSA8 CTRIP_LOW
    ECAP_INPUT_CMPSSA8_CTRIP_LOW = 158,
    //! Capture input is CMPSSA8 CTRIP_HIGH
    ECAP_INPUT_CMPSSA8_CTRIP_HIGH = 159,
    //! Capture input is CMPSSA9 CTRIP_LOW
    ECAP_INPUT_CMPSSA9_CTRIP_LOW = 160,
    //! Capture input is CMPSSA9 CTRIP_HIGH
    ECAP_INPUT_CMPSSA9_CTRIP_HIGH = 161,
    //! Capture input is CMPSSB0 CTRIP_LOW
    ECAP_INPUT_CMPSSB0_CTRIP_LOW = 162,
    //! Capture input is CMPSSB0 CTRIP_HIGH
    ECAP_INPUT_CMPSSB0_CTRIP_HIGH = 163,
    //! Capture input is CMPSSB1 CTRIP_LOW
    ECAP_INPUT_CMPSSB1_CTRIP_LOW = 164,
    //! Capture input is CMPSSB1 CTRIP_HIGH
    ECAP_INPUT_CMPSSB1_CTRIP_HIGH = 165,
    //! Capture input is CMPSSB2 CTRIP_LOW
    ECAP_INPUT_CMPSSB2_CTRIP_LOW = 166,
    //! Capture input is CMPSSB2 CTRIP_HIGH
    ECAP_INPUT_CMPSSB2_CTRIP_HIGH = 167,
    //! Capture input is CMPSSB3 CTRIP_LOW
    ECAP_INPUT_CMPSSB3_CTRIP_LOW = 168,
    //! Capture input is CMPSSB3 CTRIP_HIGH
    ECAP_INPUT_CMPSSB3_CTRIP_HIGH = 169,
    //! Capture input is CMPSSB4 CTRIP_LOW
    ECAP_INPUT_CMPSSB4_CTRIP_LOW = 170,
    //! Capture input is CMPSSB4 CTRIP_HIGH
    ECAP_INPUT_CMPSSB4_CTRIP_HIGH = 171,
    //! Capture input is CMPSSB5 CTRIP_LOW
    ECAP_INPUT_CMPSSB5_CTRIP_LOW = 172,
    //! Capture input is CMPSSB5 CTRIP_HIGH
    ECAP_INPUT_CMPSSB5_CTRIP_HIGH = 173,
    //! Capture input is CMPSSB6 CTRIP_LOW
    ECAP_INPUT_CMPSSB6_CTRIP_LOW = 174,
    //! Capture input is CMPSSB6 CTRIP_HIGH
    ECAP_INPUT_CMPSSB6_CTRIP_HIGH = 175,
    //! Capture input is CMPSSB7 CTRIP_LOW
    ECAP_INPUT_CMPSSB7_CTRIP_LOW = 176,
    //! Capture input is CMPSSB7 CTRIP_HIGH
    ECAP_INPUT_CMPSSB7_CTRIP_HIGH = 177,
    //! Capture input is CMPSSB8 CTRIP_LOW
    ECAP_INPUT_CMPSSB8_CTRIP_LOW = 178,
    //! Capture input is CMPSSB8 CTRIP_HIGH
    ECAP_INPUT_CMPSSB8_CTRIP_HIGH = 179,
    //! Capture input is CMPSSB9 CTRIP_LOW
    ECAP_INPUT_CMPSSB9_CTRIP_LOW = 180,
    //! Capture input is CMPSSB9 CTRIP_HIGH
    ECAP_INPUT_CMPSSB9_CTRIP_HIGH = 181,
    //! Capture input is ADC0 Event 0
    ECAP_INPUT_ADC0_EVT0 = 182,
    //! Capture input is ADC0 Event 1
    ECAP_INPUT_ADC0_EVT1 = 183,
    //! Capture input is ADC0 Event 2
    ECAP_INPUT_ADC0_EVT2 = 184,
    //! Capture input is ADC0 Event 3
    ECAP_INPUT_ADC0_EVT3 = 185,
    //! Capture input is ADC1 Event 0
    ECAP_INPUT_ADC1_EVT0 = 186,
    //! Capture input is ADC1 Event 1
    ECAP_INPUT_ADC1_EVT1 = 187,
    //! Capture input is ADC1 Event 2
    ECAP_INPUT_ADC1_EVT2 = 188,
    //! Capture input is ADC1 Event 3
    ECAP_INPUT_ADC1_EVT3 = 189,
    //! Capture input is ADC2 Event 0
    ECAP_INPUT_ADC2_EVT0 = 190,
    //! Capture input is ADC2 Event 1
    ECAP_INPUT_ADC2_EVT1 = 191,
    //! Capture input is ADC2 Event 2
    ECAP_INPUT_ADC2_EVT2 = 192,
    //! Capture input is ADC2 Event 3
    ECAP_INPUT_ADC2_EVT3 = 193,
    //! Capture input is ADC3 Event 0
    ECAP_INPUT_ADC3_EVT0 = 194,
    //! Capture input is ADC3 Event 1
    ECAP_INPUT_ADC3_EVT1 = 195,
    //! Capture input is ADC3 Event 2
    ECAP_INPUT_ADC3_EVT2 = 196,
    //! Capture input is ADC3 Event 3
    ECAP_INPUT_ADC3_EVT3 = 197,
    //! Capture input is ADC4 Event 0
    ECAP_INPUT_ADC4_EVT0 = 198,
    //! Capture input is ADC4 Event 1
    ECAP_INPUT_ADC4_EVT1 = 199,
    //! Capture input is ADC4 Event 2
    ECAP_INPUT_ADC4_EVT2 = 200,
    //! Capture input is ADC4 Event 3
    ECAP_INPUT_ADC4_EVT3 = 201,
    //! Capture input is InputXBar Output 0
    ECAP_INPUT_INPUTXBAR0 = 202,
    //! Capture input is InputXBar Output 1
    ECAP_INPUT_INPUTXBAR1 = 203,
    //! Capture input is InputXBar Output 2
    ECAP_INPUT_INPUTXBAR2 = 204,
    //! Capture input is InputXBar Output 3
    ECAP_INPUT_INPUTXBAR3 = 205,
    //! Capture input is InputXBar Output 4
    ECAP_INPUT_INPUTXBAR4 = 206,
    //! Capture input is InputXBar Output 5
    ECAP_INPUT_INPUTXBAR5 = 207,
    //! Capture input is InputXBar Output 6
    ECAP_INPUT_INPUTXBAR6 = 208,
    //! Capture input is InputXBar Output 7
    ECAP_INPUT_INPUTXBAR7 = 209,
    //! Capture input is InputXBar Output 8
    ECAP_INPUT_INPUTXBAR8 = 210,
    //! Capture input is InputXBar Output 9
    ECAP_INPUT_INPUTXBAR9 = 211,
    //! Capture input is InputXBar Output 10
    ECAP_INPUT_INPUTXBAR10 = 212,
    //! Capture input is InputXBar Output 11
    ECAP_INPUT_INPUTXBAR11 = 213,
    //! Capture input is InputXBar Output 12
    ECAP_INPUT_INPUTXBAR12 = 214,
    //! Capture input is InputXBar Output 13
    ECAP_INPUT_INPUTXBAR13 = 215,
    //! Capture input is InputXBar Output 14
    ECAP_INPUT_INPUTXBAR14 = 216,
    //! Capture input is InputXBar Output 15
    ECAP_INPUT_INPUTXBAR15 = 217,
    //! Capture input is InputXBar Output 16
    ECAP_INPUT_INPUTXBAR16 = 218,
    //! Capture input is InputXBar Output 17
    ECAP_INPUT_INPUTXBAR17 = 219,
    //! Capture input is InputXBar Output 18
    ECAP_INPUT_INPUTXBAR18 = 220,
    //! Capture input is InputXBar Output 19
    ECAP_INPUT_INPUTXBAR19 = 221,
    //! Capture input is InputXBar Output 20
    ECAP_INPUT_INPUTXBAR20 = 222,
    //! Capture input is InputXBar Output 21
    ECAP_INPUT_INPUTXBAR21 = 223,
    //! Capture input is InputXBar Output 22
    ECAP_INPUT_INPUTXBAR22 = 224,
    //! Capture input is InputXBar Output 23
    ECAP_INPUT_INPUTXBAR23 = 225,
    //! Capture input is InputXBar Output 24
    ECAP_INPUT_INPUTXBAR24 = 226,
    //! Capture input is InputXBar Output 25
    ECAP_INPUT_INPUTXBAR25 = 227,
    //! Capture input is InputXBar Output 26
    ECAP_INPUT_INPUTXBAR26 = 228,
    //! Capture input is InputXBar Output 27
    ECAP_INPUT_INPUTXBAR27 = 229,
    //! Capture input is InputXBar Output 28
    ECAP_INPUT_INPUTXBAR28 = 230,
    //! Capture input is InputXBar Output 29
    ECAP_INPUT_INPUTXBAR29 = 231,
    //! Capture input is InputXBar Output 30
    ECAP_INPUT_INPUTXBAR30 = 232,
    //! Capture input is InputXBar Output 31
    ECAP_INPUT_INPUTXBAR31 = 233
}ECAP_InputCaptureSignals;

//*****************************************************************************
//
//! Values that can be passed to ECAP_selectSocTriggerSource() as the \e triggersource
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT1 = 0,
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT2 = 1,
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT3 = 2,
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT4 = 3,
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD = 4,
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_CMP = 5,
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD_CMP = 6,
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_DISABLED = 7
}ECAP_SocTriggerSource;
//*****************************************************************************
//
//! Values that can be passed to ECAP_setSyncInPulseSource() as the \e mode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! Disable Sync-in
    ECAP_SYNC_IN_PULSE_SRC_DISABLE = 0x0,
    //! Sync-in source is EPWM0 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0 = 0x1,
    //! Sync-in source is EPWM1 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1 = 0x2,
    //! Sync-in source is EPWM2 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM2 = 0x3,
    //! Sync-in source is EPWM3 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM3 = 0x4,
    //! Sync-in source is EPWM4 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM4 = 0x5,
    //! Sync-in source is EPWM5 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM5 = 0x6,
    //! Sync-in source is EPWM6 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM6 = 0x7,
    //! Sync-in source is EPWM7 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM7 = 0x8,
    //! Sync-in source is EPWM8 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM8 = 0x9,
    //! Sync-in source is EPWM9 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM9 = 0xA,
    //! Sync-in source is EPWM10 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM10 = 0xB,
    //! Sync-in source is EPWM11 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM11 = 0xC,
    //! Sync-in source is EPWM12 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM12 = 0xD,
    //! Sync-in source is EPWM13 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM13 = 0xE,
    //! Sync-in source is EPWM14 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM14 = 0xF,
    //! Sync-in source is EPWM15 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM15 = 0x10,
    //! Sync-in source is EPWM16 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM16 = 0x11,
    //! Sync-in source is EPWM17 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM17 = 0x12,
    //! Sync-in source is EPWM18 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM18 = 0x13,
    //! Sync-in source is EPWM19 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM19 = 0x14,
    //! Sync-in source is EPWM20 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM20 = 0x15,
    //! Sync-in source is EPWM21 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM21 = 0x16,
    //! Sync-in source is EPWM22 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM22 = 0x17,
    //! Sync-in source is EPWM23 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM23 = 0x18,
    //! Sync-in source is EPWM24 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM24 = 0x19,
    //! Sync-in source is EPWM25 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM25 = 0x1A,
    //! Sync-in source is EPWM26 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM26 = 0x1B,
    //! Sync-in source is EPWM27 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM27 = 0x1C,
    //! Sync-in source is EPWM28 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM28 = 0x1D,
    //! Sync-in source is EPWM29 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM29 = 0x1E,
    //! Sync-in source is EPWM30 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM30 = 0x1F,
    //! Sync-in source is EPWM31 sync-out signal
    ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM31 = 0x20,
}ECAP_SyncInPulseSource;

//*****************************************************************************
//
//! Values that can be passed to HRCAP_getCalibrationClockPeriod() as the
//! \e clockSource parameter.
//
//*****************************************************************************
typedef enum
{
    HRCAP_CALIBRATION_CLOCK_SYSCLK = 0x0,  //!< Use SYSCLK for period match.
    HRCAP_CALIBRATION_CLOCK_HRCLK = 0x4  //!< Use HRCLK for period match.
}HRCAP_CalibrationClockSource;

//*****************************************************************************
//
//! Values that can be passed to HRCAP_setCalibrationMode(),
//! as the \e continuousMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Continuous calibration disabled.
    HRCAP_CONTINUOUS_CALIBRATION_DISABLED = 0x00,
    //! Continuous calibration enabled.
    HRCAP_CONTINUOUS_CALIBRATION_ENABLED = 0x20
}HRCAP_ContinuousCalibrationMode;

//*****************************************************************************
//
//! Sets the input prescaler.
//!
//! \param base is the base address of the ECAP module.
//! \param preScalerValue is the pre scaler value for ECAP input
//!
//! This function divides the ECAP input scaler. The pre scale value is
//! doubled inside the module. For example a preScalerValue of 5 will divide
//! the scaler by 10. Use a value of 0 to divide the pre scaler by 1.
//! The value of preScalerValue should be less than
//! \b ECAP_MAX_PRESCALER_VALUE.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setEventPrescaler(uint32_t base,
                                          uint16_t preScalerValue)
{
    DebugP_assert(preScalerValue < ECAP_MAX_PRESCALER_VALUE);

    //
    // Write to PRESCALE bit
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL1,
        ((HW_RD_REG16(base + CSL_ECAP_ECCTL1) &
        (~CSL_ECAP_ECCTL1_PRESCALE_MASK)) |
        (preScalerValue << CSL_ECAP_ECCTL1_PRESCALE_SHIFT)));
}

//*****************************************************************************
//
//! Sets the Capture event polarity.
//!
//! \param base is the base address of the ECAP module.
//! \param event is the event number.
//! \param polarity is the polarity of the event.
//!
//! This function sets the polarity of a given event. The value of event
//! is between \b ECAP_EVENT_1 and \b ECAP_EVENT_4 inclusive corresponding to
//! the four available events.For each event the polarity value determines the
//! edge on which the capture is activated. For a rising edge use a polarity
//! value of \b ECAP_EVNT_RISING_EDGE and for a falling edge use a polarity of
//! \b ECAP_EVNT_FALLING_EDGE.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setEventPolarity(uint32_t base,
                                         ECAP_Events event,
                                         ECAP_EventPolarity polarity)
{

    uint16_t shift;

    shift = ((uint16_t)event) << 1U;

    //
    // Write to CAP1POL, CAP2POL, CAP3POL or CAP4POL
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL1,
        ((HW_RD_REG16(base + CSL_ECAP_ECCTL1) & ~(1U << shift)) |
        ((uint16_t)polarity << shift)));
}

//*****************************************************************************
//
//! Sets the capture mode.
//!
//! \param base is the base address of the ECAP module.
//! \param mode is the capture mode.
//! \param event is the event number at which the counter stops or wraps.
//!
//! This function sets the eCAP module to a continuous or one-shot mode.
//! The value of mode should be either \b ECAP_CONTINUOUS_CAPTURE_MODE or
//! \b ECAP_ONE_SHOT_CAPTURE_MODE corresponding to continuous or one-shot mode
//! respectively.
//!
//! The value of event determines the event number at which the counter stops
//! (in one-shot mode) or the counter wraps (in continuous mode). The value of
//! event should be between \b ECAP_EVENT_1 and \b ECAP_EVENT_4 corresponding
//! to the valid event numbers.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setCaptureMode(uint32_t base,
                                       ECAP_CaptureMode mode,
                                       ECAP_Events event)
{
    //
    // Write to CONT/ONESHT
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        ((HW_RD_REG16(base + CSL_ECAP_ECCTL2) &
        (~CSL_ECAP_ECCTL2_CONT_ONESHT_MASK)) | (uint16_t)mode));

    //
    // Write to STOP_WRAP
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        ((HW_RD_REG16(base + CSL_ECAP_ECCTL2) &
        (~CSL_ECAP_ECCTL2_STOP_WRAP_MASK)) |
        (((uint16_t)event) << CSL_ECAP_ECCTL2_STOP_WRAP_SHIFT )));
}

//*****************************************************************************
//
//! Re-arms the eCAP module.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function re-arms the eCAP module.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_reArm(uint32_t base)
{
    //
    // Write to RE-ARM bit
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL2) | CSL_ECAP_ECCTL2_REARM_MASK));
}

//*****************************************************************************
//
//! Enables interrupt source.
//!
//! \param base is the base address of the ECAP module.
//! \param intFlags is the interrupt source to be enabled.
//!
//! This function sets and enables eCAP interrupt source. The following are
//! valid interrupt sources.
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_1 - Event 1 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_2 - Event 2 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_3 - Event 3 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_4 - Event 4 generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_OVERFLOW - Counter overflow generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_PERIOD   - Counter equal period generates
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE  - Counter equal compare generates
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_HR_ERROR - High resolution error generates interrupt
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_enableInterrupt(uint32_t base,
                                        uint16_t intFlags)
{
    DebugP_assert((intFlags & ~(ECAP_ISR_SOURCE_CAPTURE_EVENT_1 |
                   ECAP_ISR_SOURCE_CAPTURE_EVENT_2 |
                   ECAP_ISR_SOURCE_CAPTURE_EVENT_3 |
                   ECAP_ISR_SOURCE_CAPTURE_EVENT_4 |
                   ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                   ECAP_ISR_SOURCE_COUNTER_PERIOD |
                   ECAP_ISR_SOURCE_COUNTER_COMPARE |
                   ECAP_ISR_SOURCE_HR_ERROR)) == 0U);

    //
    // Set bits in ECEINT register
    //
    HW_WR_REG16(base + CSL_ECAP_ECEINT,
        (HW_RD_REG16(base + CSL_ECAP_ECEINT) | intFlags));
}

//*****************************************************************************
//
//! Disables interrupt source.
//!
//! \param base is the base address of the ECAP module.
//! \param intFlags is the interrupt source to be disabled.
//!
//! This function clears and disables eCAP interrupt source. The following are
//! valid interrupt sources.
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_1   - Event 1 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_2   - Event 2 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_3   - Event 3 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_4   - Event 4 generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_OVERFLOW  - Counter overflow generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_PERIOD    - Counter equal period generates
//!                                        interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE   - Counter equal compare generates
//!                                        interrupt
//!  - ECAP_ISR_SOURCE_HR_ERROR - High resolution error generates interrupt
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_disableInterrupt(uint32_t base,
                                         uint16_t intFlags)
{
    DebugP_assert((intFlags & ~(ECAP_ISR_SOURCE_CAPTURE_EVENT_1 |
                   ECAP_ISR_SOURCE_CAPTURE_EVENT_2 |
                   ECAP_ISR_SOURCE_CAPTURE_EVENT_3 |
                   ECAP_ISR_SOURCE_CAPTURE_EVENT_4 |
                   ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                   ECAP_ISR_SOURCE_COUNTER_PERIOD |
                   ECAP_ISR_SOURCE_COUNTER_COMPARE |
                   ECAP_ISR_SOURCE_HR_ERROR)) == 0U);

    //
    // Clear bits in ECEINT register
    //
    HW_WR_REG16(base + CSL_ECAP_ECEINT,
        (HW_RD_REG16(base + CSL_ECAP_ECEINT) & ~intFlags));
}

//*****************************************************************************
//
//! Returns the interrupt flag.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function returns the eCAP interrupt flag. The following are valid
//! interrupt sources corresponding to the eCAP interrupt flag.
//!
//! \return Returns the eCAP interrupt that has occurred. The following are
//!  valid return values.
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_1   - Event 1 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_2   - Event 2 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_3   - Event 3 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_4   - Event 4 generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_OVERFLOW  - Counter overflow generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_PERIOD    - Counter equal period generates
//!                                        interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE   - Counter equal compare generates
//!                                        interrupt
//!  - ECAP_ISR_SOURCE_HR_ERROR - High resolution error generates interrupt
//!
//! \note - User can check if a combination of various interrupts have occurred
//!         by ORing the above return values.
//
//*****************************************************************************
static inline uint16_t ECAP_getInterruptSource(uint32_t base)
{
    //
    // Return contents of ECFLG register
    //
    return(HW_RD_REG16(base + CSL_ECAP_ECFLG) & 0xFEU);
}

//*****************************************************************************
//
//! Returns the Global interrupt flag.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function returns the eCAP Global interrupt flag.
//!
//! \return Returns true if there is a global eCAP interrupt, false otherwise.
//
//*****************************************************************************
static inline bool ECAP_getGlobalInterruptStatus(uint32_t base)
{
    //
    // Return contents of Global interrupt bit
    //
    return((HW_RD_REG16(base + CSL_ECAP_ECFLG) & 0x1U) == 0x1U);
}

//*****************************************************************************
//
//! Clears interrupt flag.
//!
//! \param base is the base address of the ECAP module.
//! \param intFlags is the interrupt source.
//!
//! This function clears eCAP interrupt flags. The following are valid
//! interrupt sources.
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_1 - Event 1 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_2 - Event 2 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_3 - Event 3 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_4 - Event 4 generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_OVERFLOW - Counter overflow generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_PERIOD   - Counter equal period generates
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE  - Counter equal compare generates
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_HR_ERROR - High resolution error generates interrupt
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_clearInterrupt(uint32_t base,
                                       uint16_t intFlags)
{
    DebugP_assert((intFlags & ~(ECAP_ISR_SOURCE_CAPTURE_EVENT_1 |
                   ECAP_ISR_SOURCE_CAPTURE_EVENT_2 |
                   ECAP_ISR_SOURCE_CAPTURE_EVENT_3 |
                   ECAP_ISR_SOURCE_CAPTURE_EVENT_4 |
                   ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                   ECAP_ISR_SOURCE_COUNTER_PERIOD |
                   ECAP_ISR_SOURCE_COUNTER_COMPARE |
                   ECAP_ISR_SOURCE_HR_ERROR)) == 0U);

    //
    // Write to ECCLR register
    //
    HW_WR_REG16(base + CSL_ECAP_ECCLR,
        (HW_RD_REG16(base + CSL_ECAP_ECCLR) |
        (intFlags | CSL_ECAP_ECCLR_INT_MASK)));
}

//*****************************************************************************
//
//! Clears global interrupt flag
//!
//! \param base is the base address of the ECAP module.
//!
//! This function clears the global interrupt bit.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_clearGlobalInterrupt(uint32_t base)
{
    //
    // Write to INT bit
    //
    HW_WR_REG16(base + CSL_ECAP_ECCLR,
        (HW_RD_REG16(base + CSL_ECAP_ECCLR) | CSL_ECAP_ECCLR_INT_MASK));
}

//*****************************************************************************
//
//! Forces interrupt source.
//!
//! \param base is the base address of the ECAP module.
//! \param intFlags is the interrupt source.
//!
//! This function forces and enables eCAP interrupt source. The following are
//! valid interrupt sources.
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_1 - Event 1 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_2 - Event 2 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_3 - Event 3 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_4 - Event 4 generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_OVERFLOW - Counter overflow generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_PERIOD   - Counter equal period generates
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE  - Counter equal compare generates
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_HR_ERROR - High resolution error generates interrupt
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_forceInterrupt(uint32_t base,
                                       uint16_t intFlags)
{
    DebugP_assert((intFlags & ~(ECAP_ISR_SOURCE_CAPTURE_EVENT_1 |
                   ECAP_ISR_SOURCE_CAPTURE_EVENT_2 |
                   ECAP_ISR_SOURCE_CAPTURE_EVENT_3 |
                   ECAP_ISR_SOURCE_CAPTURE_EVENT_4 |
                   ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                   ECAP_ISR_SOURCE_COUNTER_PERIOD |
                   ECAP_ISR_SOURCE_COUNTER_COMPARE |
                   ECAP_ISR_SOURCE_HR_ERROR)) == 0U);

    //
    // Write to ECFRC register
    //
    HW_WR_REG16(base + CSL_ECAP_ECFRC,
        (HW_RD_REG16(base + CSL_ECAP_ECFRC) | intFlags));
}

//*****************************************************************************
//
//! Sets eCAP in Capture mode.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function sets the eCAP module to operate in Capture mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_enableCaptureMode(uint32_t base)
{
    //
    // Clear CAP/APWM bit
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL2) &
        ~CSL_ECAP_ECCTL2_CAP_APWM_MASK));
}

//*****************************************************************************
//
//! Sets eCAP in APWM mode.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function sets the eCAP module to operate in APWM mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_enableAPWMMode(uint32_t base)
{
    //
    // Set CAP/APWM bit
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL2) | CSL_ECAP_ECCTL2_CAP_APWM_MASK));
}

//*****************************************************************************
//
//! Enables counter reset on an event.
//!
//! \param base is the base address of the ECAP module.
//! \param event is the event number the time base gets reset.
//!
//! This function enables the base timer, TSCTR, to be reset on capture
//! event provided by the variable event. Valid inputs for event are
//! \b ECAP_EVENT_1 to \b ECAP_EVENT_4.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_enableCounterResetOnEvent(uint32_t base,
                                                  ECAP_Events event)
{
    //
    // Set CTRRST1,CTRRST2,CTRRST3 or CTRRST4 bits
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL1,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL1) |
        (1U << ((2U * (uint16_t)event) + 1U))));
}

//*****************************************************************************
//
//! Disables counter reset on events.
//!
//! \param base is the base address of the ECAP module.
//! \param event is the event number the time base gets reset.
//!
//! This function disables the base timer, TSCTR, from being reset on capture
//! event provided by the variable event. Valid inputs for event are
//! \b ECAP_EVENT_1 to \b ECAP_EVENT_4.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_disableCounterResetOnEvent(uint32_t base,
                                                   ECAP_Events event)
{
    DebugP_assert(((uint32_t) event >= 1U) || ((uint32_t) event <= 4U));

    //
    // Clear CTRRST1,CTRRST2,CTRRST3 or CTRRST4 bits
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL1,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL1) &
        ~(1U << ((2U * (uint16_t)event) + 1U))));
}

//*****************************************************************************
//
//! Enables time stamp capture.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function enables time stamp count to be captured
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_enableTimeStampCapture(uint32_t base)
{
    //
    // Set CAPLDEN bit
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL1,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL1) | CSL_ECAP_ECCTL1_CAPLDEN_MASK));
}

//*****************************************************************************
//
//! Disables time stamp capture.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function disables time stamp count to be captured
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_disableTimeStampCapture(uint32_t base)
{
    //
    // Clear CAPLDEN bit
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL1,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL1) & ~CSL_ECAP_ECCTL1_CAPLDEN_MASK));
}

//*****************************************************************************
//
//! Sets a phase shift value count.
//!
//! \param base is the base address of the ECAP module.
//! \param shiftCount is the phase shift value.
//!
//! This function writes a phase shift value to be loaded into the main time
//! stamp counter.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setPhaseShiftCount(uint32_t base, uint32_t shiftCount)
{
    //
    // Write to CTRPHS
    //
    HW_WR_REG32(base + CSL_ECAP_CTRPHS, shiftCount);
}

//*****************************************************************************
//
//! Set up the source for sync-in pulse.
//!
//! \param base is the base address of the ECAP module.
//! \param source is the sync-in pulse source.
//!
//! This function set the sync out pulse mode.
//! Valid values for mode are:
//!  - ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0-31 - sync-in pulse source can be
//!                                              any of the EPWM0-31 sync-out
//!                                              signal
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_setSyncInPulseSource(uint32_t base, ECAP_SyncInPulseSource source)
{
    //
    // Set ECAP Sync-In Source Mode.
    //
    HW_WR_REG16(base + CSL_ECAP_ECAPSYNCINSEL,
        ((HW_RD_REG16(base + CSL_ECAP_ECAPSYNCINSEL) &
        (~CSL_ECAP_ECAPSYNCINSEL_SEL_MASK)) |
        ((uint16_t)source & CSL_ECAP_ECAPSYNCINSEL_SEL_MASK)));
}

//*****************************************************************************
//
//! Enable counter loading with phase shift value.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function enables loading of the counter with the value present in the
//! phase shift counter as defined by the ECAP_setPhaseShiftCount() function.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_enableLoadCounter(uint32_t base)
{
    //
    // Write to SYNCI_EN
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL2) | CSL_ECAP_ECCTL2_SYNCI_EN_MASK));
}

//*****************************************************************************
//
//! Disable counter loading with phase shift value.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function disables loading of the counter with the value present in the
//! phase shift counter as defined by the ECAP_setPhaseShiftCount() function.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_disableLoadCounter(uint32_t base)
{
    //
    // Write to SYNCI_EN
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL2) &
        ~CSL_ECAP_ECCTL2_SYNCI_EN_MASK));
}

//*****************************************************************************
//
//! Load time stamp counter
//!
//! \param base is the base address of the ECAP module.
//!
//! This function forces the value in the phase shift counter register to be
//! loaded into Time stamp counter register.
//! Make sure to enable loading of Time stamp counter by calling
//! ECAP_enableLoadCounter() function before calling this function.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_loadCounter(uint32_t base)
{
    //
    // Write to SWSYNC
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL2) | CSL_ECAP_ECCTL2_SWSYNC_MASK));
}

//*****************************************************************************
//
//! Configures Sync out signal mode.
//!
//! \param base is the base address of the ECAP module.
//! \param mode is the sync out mode.
//!
//! This function sets the sync out mode. Valid parameters for mode are:
//! - ECAP_SYNC_OUT_SYNCI - Trigger sync out on sync-in event.
//! - ECAP_SYNC_OUT_COUNTER_PRD - Trigger sync out when counter equals period.
//! - ECAP_SYNC_OUT_DISABLED - Disable sync out.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setSyncOutMode(uint32_t base,
                                       ECAP_SyncOutMode mode)
{
    //
    // Write to SYNCO_SEL
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        ((HW_RD_REG16(base + CSL_ECAP_ECCTL2) &
        (~CSL_ECAP_ECCTL2_SYNCO_SEL_MASK)) | (uint16_t)mode));
}

//*****************************************************************************
//
//! Stops Time stamp counter.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function stops the time stamp counter.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_stopCounter(uint32_t base)
{
    //
    // Clear TSCTR
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL2) &
        ~CSL_ECAP_ECCTL2_TSCTRSTOP_MASK));
}

//*****************************************************************************
//
//! Starts Time stamp counter.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function starts the time stamp counter.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_startCounter(uint32_t base)
{
    //
    // Set TSCTR
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL2) |
        CSL_ECAP_ECCTL2_TSCTRSTOP_MASK));
}

//*****************************************************************************
//
//! Set eCAP APWM polarity.
//!
//! \param base is the base address of the ECAP module.
//! \param polarity is the polarity of APWM
//!
//! This function sets the polarity of the eCAP in APWM mode. Valid inputs for
//! polarity are:
//!  - ECAP_APWM_ACTIVE_HIGH - For active high.
//!  - ECAP_APWM_ACTIVE_LOW - For active low.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setAPWMPolarity(uint32_t base,
                                        ECAP_APWMPolarity polarity)
{
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        ((HW_RD_REG16(base + CSL_ECAP_ECCTL2) &
        ~CSL_ECAP_ECCTL2_APWMPOL_MASK) | (uint16_t)polarity));
}

//*****************************************************************************
//
//! Set eCAP APWM period.
//!
//! \param base is the base address of the ECAP module.
//! \param periodCount is the period count for APWM.
//!
//! This function sets the period count of the APWM waveform.
//! periodCount takes the actual count which is written to the register. The
//! user is responsible for converting the desired frequency or time into
//! the period count.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setAPWMPeriod(uint32_t base, uint32_t periodCount)
{
    //
    // Write to CAP1
    //
    HW_WR_REG32(base + CSL_ECAP_CAP1, periodCount);
}

//*****************************************************************************
//
//! Set eCAP APWM on or off time count.
//!
//! \param base is the base address of the ECAP module.
//! \param compareCount is the on or off count for APWM.
//!
//! This function sets the on or off time count of the APWM waveform depending
//! on the polarity of the output. If the output , as set by
//! ECAP_setAPWMPolarity(), is active high then compareCount determines the on
//! time. If the output is active low then compareCount determines the off
//! time. compareCount takes the actual count which is written to the register.
//! The user is responsible for converting the desired frequency or time into
//! the appropriate count value.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setAPWMCompare(uint32_t base, uint32_t compareCount)
{
    //
    // Write to CAP2
    //
    HW_WR_REG32(base + CSL_ECAP_CAP2, compareCount);
}

//*****************************************************************************
//
//! Load eCAP APWM shadow period.
//!
//! \param base is the base address of the ECAP module.
//! \param periodCount is the shadow period count for APWM.
//!
//! This function sets the shadow period count of the APWM waveform.
//! periodCount takes the actual count which is written to the register. The
//! user is responsible for converting the desired frequency or time into
//! the period count.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setAPWMShadowPeriod(uint32_t base,
                                            uint32_t periodCount)
{
    //
    // Write to CAP3
    //
    HW_WR_REG32(base + CSL_ECAP_CAP3, periodCount);
}

//*****************************************************************************
//
//! Set eCAP APWM shadow on or off time count.
//!
//! \param base is the base address of the ECAP module.
//! \param compareCount is the on or off count for APWM.
//!
//! This function sets the shadow on or off time count of the APWM waveform
//! depending on the polarity of the output. If the output , as set by
//! ECAP_setAPWMPolarity() , is active high then compareCount determines the
//! on time. If the output is active low then compareCount determines the off
//! time. compareCount takes the actual count which is written to the register.
//! The user is responsible for converting the desired frequency or time into
//! the appropriate count value.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setAPWMShadowCompare(uint32_t base,
                                             uint32_t compareCount)
{
    //
    // Write to CAP4
    //
    HW_WR_REG32(base + CSL_ECAP_CAP4, compareCount);
}

//*****************************************************************************
//
//! Returns the time base counter value.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function returns the time base counter value.
//!
//! \return Returns the time base counter value.
//
//*****************************************************************************
static inline uint32_t ECAP_getTimeBaseCounter(uint32_t base)
{
    //
    // Read the Time base counter value
    //
    return(HW_RD_REG32(base + CSL_ECAP_TSCTR));
}

//*****************************************************************************
//
//! Returns event time stamp.
//!
//! \param base is the base address of the ECAP module.
//! \param event is the event number.
//!
//! This function returns the current time stamp count of the given event.
//! Valid values for event are \b ECAP_EVENT_1 to \b ECAP_EVENT_4.
//!
//! \return Event time stamp value or 0 if \e event is invalid.
//
//*****************************************************************************
static inline uint32_t ECAP_getEventTimeStamp(uint32_t base, ECAP_Events event)
{
    uint32_t count;

    switch(event)
    {
        case ECAP_EVENT_1:

            //
            // Read CAP1 register
            //
            count = HW_RD_REG32(base + CSL_ECAP_CAP1);
        break;

        case ECAP_EVENT_2:
            //
            // Read CAP2 register
            //
            count = HW_RD_REG32(base + CSL_ECAP_CAP2);
        break;

        case ECAP_EVENT_3:

            //
            // Read CAP3 register
            //
            count = HW_RD_REG32(base + CSL_ECAP_CAP3);
        break;

        case ECAP_EVENT_4:

            //
            // Read CAP4 register
            //
            count = HW_RD_REG32(base + CSL_ECAP_CAP4);
        break;

        default:

            //
            // Invalid event parameter
            //
            count = 0U;
        break;
    }

    return(count);
}

//*****************************************************************************
//
//! Select eCAP input.
//!
//! \param base is the base address of the ECAP module.
//! \param input is the eCAP input signal.
//!
//! This function selects the eCAP input signal.
//!
//! Please refer to the ::ECAP_InputCaptureSignals Enum for the valid values
//! to be passed to \e input parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_selectECAPInput(uint32_t base,
                                        ECAP_InputCaptureSignals input)
{
    //
    // Write to ECCTL0
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL0,
        ((HW_RD_REG16(base + CSL_ECAP_ECCTL0) &
        ~CSL_ECAP_ECCTL0_INPUTSEL_MASK) | (uint16_t)input));
}
//*****************************************************************************
//
//! Select ADC SOC event
//!
//! \param base is the base address of the ECAP module.
//! \param triggersource is the SOC Trigger source.
//!
//! This function selects the ecap trigger source for ADC SOC event.
//!
//! Please refer to the ::ECAP_SocTriggerSource Enum for the valid values
//! to be passed to \e triggersource parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_selectSocTriggerSource(uint32_t base,
                                        ECAP_SocTriggerSource triggersource)
{
    uint16_t source = (uint16_t)triggersource;
    //
    // Write to ECCTL0
    //
    if(source >= ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD)
        source-= (uint16_t)ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD;

    HW_WR_REG32(base + CSL_ECAP_ECCTL0,
        ((HW_RD_REG32(base + CSL_ECAP_ECCTL0) &
        ~CSL_ECAP_ECCTL0_SOCEVTSEL_MASK) | (source << CSL_ECAP_ECCTL0_SOCEVTSEL_SHIFT)));
}

//*****************************************************************************
//
//! Resets eCAP counters and flags.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function resets the main counter (TSCTR register), event filter,
//! modulo counter, capture events and counter overflow flags
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_resetCounters(uint32_t base)
{
    //
    // Write to ECCTL2
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        (HW_RD_REG16(base + CSL_ECAP_ECCTL2) |
        CSL_ECAP_ECCTL2_CTRFILTRESET_MASK));
}

//*****************************************************************************
//
//! Sets the eCAP DMA source.
//!
//! \param base is the base address of the ECAP module.
//! \param event is the eCAP event for the DMA
//!
//! This function sets the eCAP event source for the DMA trigger.
//! Valid values for \e event are \b ECAP_EVENT_1 to \b ECAP_EVENT_4.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setDMASource(uint32_t base, ECAP_Events event)
{
    //
    // Write to ECCTL2
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        ((HW_RD_REG16(base + CSL_ECAP_ECCTL2) &
        ~CSL_ECAP_ECCTL2_DMAEVTSEL_MASK) |
        ((uint16_t)event << CSL_ECAP_ECCTL2_DMAEVTSEL_SHIFT)));
}

//*****************************************************************************
//
//! Return the Modulo counter status.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function returns the modulo counter status, indicating which register
//! gets loaded on the next capture event.
//!
//! \return Returns an \b ECAP_EVENT_n value indicating that CAPn is the
//! register to be loaded on the next event.
//
//*****************************************************************************
static inline ECAP_Events ECAP_getModuloCounterStatus(uint32_t base)
{
    uint16_t counterStatusValue;

    counterStatusValue = (((HW_RD_REG32(base + CSL_ECAP_ECCTL2) &
                          CSL_ECAP_ECCTL2_MODCNTRSTS_MASK) >>
                          CSL_ECAP_ECCTL2_MODCNTRSTS_SHIFT));

    //
    // Read MODCNTRSTS bit
    //
    return((ECAP_Events)(counterStatusValue));
}

//*****************************************************************************
//
//! Enables HRCAP.
//!
//! \param base is the base address of the ECAP instance used.
//!
//! This function enables High Resolution Capture module.
//!
//! \note High resolution clock must be enabled before High Resolution Module
//!       is enabled.
//!
//! \return None.
//
//*****************************************************************************
static inline void HRCAP_enableHighResolution(uint32_t base)
{
    //
    // Set HRE bit.
    //
    HW_WR_REG16(base + CSL_ECAP_HRCTL,
        (HW_RD_REG16(base + CSL_ECAP_HRCTL) | CSL_ECAP_HRCTL_HRE_MASK));
}

//*****************************************************************************
//
//! Disables HRCAP.
//!
//! \param base is the base address of the HRCAP instance used.
//!
//! This function disable High Resolution Capture module.
//!
//!
//! \return None.
//
//*****************************************************************************
static inline void HRCAP_disableHighResolution(uint32_t base)
{
    //
    // Set HRE bit.
    //
    HW_WR_REG16(base + CSL_ECAP_HRCTL,
        (HW_RD_REG16(base + CSL_ECAP_HRCTL) & ~CSL_ECAP_HRCTL_HRE_MASK));
}

//*****************************************************************************
//
//! Enables high resolution clock.
//!
//! \param base is the base address of the HRCAP instance used.
//!
//! This function enables High Resolution clock.
//!
//! \return None.
//
//*****************************************************************************
static inline void HRCAP_enableHighResolutionClock(uint32_t base)
{
    //
    // Set HRCLKE bit.
    //
    HW_WR_REG16(base + CSL_ECAP_HRCTL,
        (HW_RD_REG16(base + CSL_ECAP_HRCTL) | CSL_ECAP_HRCTL_HRCLKE_MASK));
}

//*****************************************************************************
//
//! Disables High resolution clock.
//!
//! \param base is the base address of the HRCAP instance used.
//!
//! This function disables High Resolution clock.
//!
//! \return None.
//
//*****************************************************************************
static inline void HRCAP_disbleHighResolutionClock(uint32_t base)
{
    //
    // Clear HRCLKE bit.
    //
    HW_WR_REG16(base + CSL_ECAP_HRCTL,
        (HW_RD_REG16(base + CSL_ECAP_HRCTL) & ~CSL_ECAP_HRCTL_HRCLKE_MASK));
}

//*****************************************************************************
//
//! Starts calibration.
//!
//! \param base is the base address of the HRCAP instance used.
//!
//! This function starts calibration.
//!
//! \return None.
//
//*****************************************************************************
static inline void HRCAP_startCalibration(uint32_t base)
{
    //
    // Set CALIBSTART bit.
    //
    HW_WR_REG16(base + CSL_ECAP_HRCTL,
        (HW_RD_REG16(base + CSL_ECAP_HRCTL) | CSL_ECAP_HRCTL_CALIBSTART_MASK));
}

//*****************************************************************************
//
//! Sets the calibration mode.
//!
//! \param base is the base address of the HRCAP instance used.
//!
//! This function sets the the calibration mode by turning on continuous
//! calibration.
//!
//! \return None.
//
//*****************************************************************************
static inline void HRCAP_setCalibrationMode(uint32_t base)
{
    //
    // Write to CALIBSTS and CALIBCONT bits.
    //
    HW_WR_REG16(base + CSL_ECAP_HRCTL,
        (HW_RD_REG16(base + CSL_ECAP_HRCTL) | CSL_ECAP_HRCTL_CALIBCONT_MASK));
}

//*****************************************************************************
//
//! Enables calibration interrupt.
//!
//! \param base is the base address of the HRCAP module.
//! \param intFlags is the calibration interrupt flags to be enabled.
//!
//! This function enables HRCAP calibration interrupt flags.
//! Valid values for intFlags are:
//!     - HRCAP_CALIBRATION_DONE   - Calibration done interrupt.
//!     - HRCAP_CALIBRATION_PERIOD_OVERFLOW - Calibration period overflow
//!                                                check interrupt.
//! \return None.
//
//*****************************************************************************
static inline void
HRCAP_enableCalibrationInterrupt(uint32_t base, uint16_t intFlags)
{
    DebugP_assert(intFlags & (HRCAP_CALIBRATION_DONE |
                  HRCAP_CALIBRATION_PERIOD_OVERFLOW));

    //
    // Set CALIBDONE or CALPRDCHKSTS.
    //
    HW_WR_REG16(base + CSL_ECAP_HRINTEN,
        (HW_RD_REG16(base + CSL_ECAP_HRINTEN) | intFlags));
}

//*****************************************************************************
//
//! Disables calibration interrupt source.
//!
//! \param base is the base address of the HRCAP module.
//! \param intFlags is the calibration interrupt flags to be disabled.
//!
//! This function disables HRCAP calibration interrupt flags.
//! Valid values for intFlags are:
//!     - HRCAP_CALIBRATION_DONE   - Calibration done interrupt.
//!     - HRCAP_CALIBRATION_PERIOD_OVERFLOW - Calibration period check
//!                                                   interrupt.
//! \return None.
//
//*****************************************************************************
static inline void
HRCAP_disableCalibrationInterrupt(uint32_t base, uint16_t intFlags)
{
    DebugP_assert(intFlags & (HRCAP_CALIBRATION_DONE |
                  HRCAP_CALIBRATION_PERIOD_OVERFLOW));

    //
    // Clear CALIBDONE or CALPRDCHKSTS.
    //
    HW_WR_REG16(base + CSL_ECAP_HRINTEN,
        (HW_RD_REG16(base + CSL_ECAP_HRINTEN) & ~intFlags));
}

//*****************************************************************************
//
//! Returns the calibration interrupt source.
//!
//! \param base is the base address of the HRCAP module.
//!
//! This function returns the HRCAP calibration interrupt source.
//!
//! \return Returns the HRCAP interrupt that has occurred. The following are
//!         valid return values.
//!          - HRCAP_GLOBAL_CALIBRATION_INTERRUPT - Global calibration
//!                                                 interrupt.
//!          - HRCAP_CALIBRATION_DONE   - Calibration done interrupt.
//!          - HRCAP_CALIBRATION_PERIOD_OVERFLOW - Calibration period overflow
//!                                                interrupt.
//!
//! \note - User can check if a combination of the interrupts have occurred
//!         by ORing the above return values.
//
//*****************************************************************************
static inline uint16_t HRCAP_getCalibrationFlags(uint32_t base)
{
    //
    // Return contents of HRFLG register.
    //
    return((uint16_t)(HW_RD_REG16(base + CSL_ECAP_HRFLG) & 0x7U));
}

//*****************************************************************************
//
//! Clears calibration flags.
//!
//! \param base is the base address of the HRCAP module.
//! \param flags is the calibration flags to be cleared.
//!
//! This function clears HRCAP calibration flags.
//! The following are valid values for flags.
//!     - HRCAP_GLOBAL_CALIBRATION_INTERRUPT - Global calibration interrupt.
//!     - HRCAP_CALIBRATION_DONE   - Calibration done flag.
//!     - HRCAP_CALIBRATION_PERIOD_OVERFLOW - Calibration period overflow flag.
//!
//! \return None.
//
//*****************************************************************************
static inline void HRCAP_clearCalibrationFlags(uint32_t base, uint16_t flags)
{
    DebugP_assert((flags == (HRCAP_CALIBRATION_DONE |
                 HRCAP_GLOBAL_CALIBRATION_INTERRUPT)) ||
                 (flags == (HRCAP_CALIBRATION_PERIOD_OVERFLOW |
                 HRCAP_GLOBAL_CALIBRATION_INTERRUPT)) ||
                 (flags == (HRCAP_CALIBRATION_DONE |
                 HRCAP_GLOBAL_CALIBRATION_INTERRUPT |
                 HRCAP_CALIBRATION_PERIOD_OVERFLOW)));

    //
    // Write to HRCLR register.
    //
    HW_WR_REG16(base + CSL_ECAP_HRCLR,
        (HW_RD_REG16(base + CSL_ECAP_HRCLR) | flags));
}

//*****************************************************************************
//
//! Return the Calibration status
//!
//! \param base is the base address of the HRCAP instance used.
//!
//! This function returns the calibration status.
//!
//! \return This functions returns true if the calibration is in process,false
//!         if there is no active calibration.
//
//*****************************************************************************
static inline bool HRCAP_isCalibrationBusy(uint32_t base)
{
    //
    // Read CALIBSTS bit.
    //
    return((HW_RD_REG16(base + CSL_ECAP_HRCTL)
            & CSL_ECAP_HRCTL_CALIBSTS_MASK) == CSL_ECAP_HRCTL_CALIBSTS_MASK);
}

//*****************************************************************************
//
//! Force a software based calibration
//!
//! \param base is the base address of the HRCAP instance used.
//! \param flag is the calibration flag source.
//!
//! This function forces a software based calibration done flag.
//! The following are valid values for flag.
//!     - HRCAP_CALIBRATION_DONE - Calibration done flag.
//!     - HRCAP_CALIBRATION_PERIOD_OVERFLOW - Calibration period overflow flag.
//!
//! \return None.
//
//*****************************************************************************
static inline void HRCAP_forceCalibrationFlags(uint32_t base, uint16_t flag)
{
    DebugP_assert((flag & (HRCAP_CALIBRATION_DONE |
                   HRCAP_CALIBRATION_PERIOD_OVERFLOW)));

    //
    // Write to CALIBDONE or CALPRDCHKSTS bit.
    //
    HW_WR_REG16(base + CSL_ECAP_HRFRC,
        (HW_RD_REG16(base + CSL_ECAP_HRFRC) | flag));
}

//*****************************************************************************
//
//! Sets the calibration period count
//!
//! \param base is the base address of the HRCAP instance used.
//! \param sysclkHz is the rate of the SYSCLK in Hz.
//!
//! This function sets the calibration period count value to achieve a period
//! of 1.6 milliseconds given the SYSCLK frequency in Hz (the \e sysclkHz
//! parameter).
//!
//! \return None.
//
//*****************************************************************************
static inline void HRCAP_setCalibrationPeriod(uint32_t base, uint32_t sysclkHz)
{
    //
    // Write to HRCALPRD register
    //
    HW_WR_REG16(base + CSL_ECAP_HRCALPRD,
        (sysclkHz * 16U) / 10000U);
}

//*****************************************************************************
//
//! Returns the calibration clock period
//!
//! \param base is the base address of the HRCAP instance used.
//! \param clockSource is the calibration clock source
//! (\b HRCAP_CALIBRATION_CLOCK_SYSCLK or \b HRCAP_CALIBRATION_CLOCK_HRCLK).
//!
//! This function returns the period match value of the calibration clock. The
//! return value has a valid count when a period match occurs.
//!
//! \return This function returns the captured value of the clock counter
//!         specified by clockSource.
//
//*****************************************************************************
static inline uint32_t
HRCAP_getCalibrationClockPeriod(uint32_t base,
                               HRCAP_CalibrationClockSource clockSource)
{
    //
    // Return HRCAP_O_HRSYSCLKCAP or HRCAP_O_HRCLKCAP.
    //
    return(HW_RD_REG16(base + CSL_ECAP_HRSYSCLKCAP + (uint32_t)clockSource));
}

//*****************************************************************************
//
//! Calculates the scale factor
//!
//! \param base is the base address of the HRCAP instance used.
//!
//! This function reads the SYSCLK and HRCLK calibration periods and then
//! uses them to calculate the scale factor.
//!
//! \return This function returns the calculated scale factor.
//
//*****************************************************************************
static inline Float32 HRCAP_getScaleFactor(uint32_t base)
{
    //
    // Calculate and return the scale factor.
    //
    return((Float32)HRCAP_getCalibrationClockPeriod(base,
                                            HRCAP_CALIBRATION_CLOCK_SYSCLK) /
           (Float32)HRCAP_getCalibrationClockPeriod(base,
                                            HRCAP_CALIBRATION_CLOCK_HRCLK));
}

//*****************************************************************************
//
//! Returns event time stamp in nanoseconds
//!
//! \param timeStamp is a raw time stamp count returned by
//! ECAP_getEventTimeStamp().
//! \param scaleFactor is the calculated scale factor returned by
//! HRCAP_getScaleFactor().
//!
//! This function converts a raw CAP time stamp (the \e timeStamp parameter) to
//! nanoseconds using the provided scale factor (the \e scaleFactor parameter).
//!
//! \return Returns the converted time stamp in nanoseconds.
//
//*****************************************************************************
static inline Float32
HRCAP_convertEventTimeStampNanoseconds(uint32_t timeStamp,
                                       Float32 scaleFactor)
{
    //
    // Convert the raw count value to nanoseconds using the given scale factor.
    //
    return((Float32)timeStamp * scaleFactor * ((Float32)5.0 /
                                                 (Float32)128.0));
}

//*****************************************************************************
//
//! Configures emulation mode.
//!
//! \param base is the base address of the ECAP module.
//! \param mode is the emulation mode.
//!
//! This function configures the eCAP counter, TSCTR,  to the desired emulation
//! mode when emulation suspension occurs. Valid inputs for mode are:
//! - ECAP_EMULATION_STOP  - Counter is stopped immediately.
//! - ECAP_EMULATION_RUN_TO_ZERO - Counter runs till it reaches 0.
//! - ECAP_EMULATION_FREE_RUN - Counter is not affected.
//!
//! \return None.
//
//*****************************************************************************
extern void ECAP_setEmulationMode(uint32_t base, ECAP_EmulationMode mode);

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

#endif // ECAP_V1_H_

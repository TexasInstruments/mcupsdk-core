/*
 * Copyright (C) 2021-2023 Texas Instruments Incorporated
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
// Define to mask out the bits in the signal monitoring unit.
//
//*****************************************************************************
#define ECAP_MUNIT_STEP            (CSL_ECAP_MUNIT_2_CTL - CSL_ECAP_MUNIT_1_CTL)

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
//! Monitoring unit 1 error event 1 ISR source
#define ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 (0x200U)
//! Monitoring unit 1 error event 2 ISR source
#define ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 (0x400U)
//! Monitoring unit 2 error event 1 ISR source
#define ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1 (0x800U)
//! Monitoring unit 2 error event 2 ISR source
#define ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2 (0x1000U)
//! All ISR source
#define ECAP_ISR_SOURCE_ALL    (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |\
                                ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |\
                                ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |\
                                ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |\
                                ECAP_ISR_SOURCE_COUNTER_OVERFLOW |\
                                ECAP_ISR_SOURCE_COUNTER_PERIOD   |\
                                ECAP_ISR_SOURCE_COUNTER_COMPARE  |\
                                ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 |\
                                ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 |\
                                ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1 |\
                                ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2)


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
//! ECAP_getEventTimeStamp() as the \e event parameter.
//
//*****************************************************************************
typedef enum
{
    ECAP_EVENT_1 = 0U,     //!< eCAP event 1 in capture mode
    ECAP_EVENT_2 = 1U,     //!< eCAP event 2 in capture mode
    ECAP_EVENT_3 = 2U,     //!< eCAP event 3 in capture mode
    ECAP_EVENT_4 = 3U,      //!< eCAP event 4
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
    //! Capture input is EPWM0 DEL ACTIVE Signal
    ECAP_INPUT_EPWM0_DELACTIVE = 22,
    //! Capture input is EPWM1 DEL ACTIVE Signal
    ECAP_INPUT_EPWM1_DELACTIVE = 23,
    //! Capture input is EPWM2 DEL ACTIVE Signal
    ECAP_INPUT_EPWM2_DELACTIVE = 24,
    //! Capture input is EPWM3 DEL ACTIVE Signal
    ECAP_INPUT_EPWM3_DELACTIVE = 25,
    //! Capture input is EPWM4 DEL ACTIVE Signal
    ECAP_INPUT_EPWM4_DELACTIVE = 26,
    //! Capture input is EPWM5 DEL ACTIVE Signal
    ECAP_INPUT_EPWM5_DELACTIVE = 27,
    //! Capture input is EPWM6 DEL ACTIVE Signal
    ECAP_INPUT_EPWM6_DELACTIVE = 28,
    //! Capture input is EPWM7 DEL ACTIVE Signal
    ECAP_INPUT_EPWM7_DELACTIVE = 29,
    //! Capture input is EPWM8 DEL ACTIVE Signal
    ECAP_INPUT_EPWM8_DELACTIVE = 30,
    //! Capture input is EPWM9 DEL ACTIVE Signal
    ECAP_INPUT_EPWM9_DELACTIVE = 31,
    //! Capture input is EPWM10 DEL ACTIVE Signal
    ECAP_INPUT_EPWM10_DELACTIVE = 32,
    //! Capture input is EPWM11 DEL ACTIVE Signal
    ECAP_INPUT_EPWM11_DELACTIVE = 33,
    //! Capture input is EPWM12 DEL ACTIVE Signal
    ECAP_INPUT_EPWM12_DELACTIVE = 34,
    //! Capture input is EPWM13 DEL ACTIVE Signal
    ECAP_INPUT_EPWM13_DELACTIVE = 35,
    //! Capture input is EPWM14 DEL ACTIVE Signal
    ECAP_INPUT_EPWM14_DELACTIVE = 36,
    //! Capture input is EPWM15 DEL ACTIVE Signal
    ECAP_INPUT_EPWM15_DELACTIVE = 37,
    //! Capture input is EPWM16 DEL ACTIVE Signal
    ECAP_INPUT_EPWM16_DELACTIVE = 38,
    //! Capture input is EPWM17 DEL ACTIVE Signal
    ECAP_INPUT_EPWM17_DELACTIVE = 39,
    //! Capture input is EPWM18 DEL ACTIVE Signal
    ECAP_INPUT_EPWM18_DELACTIVE = 40,
    //! Capture input is EPWM19 DEL ACTIVE Signal
    ECAP_INPUT_EPWM19_DELACTIVE = 41,
    //! Capture input is EPWM20 DEL ACTIVE Signal
    ECAP_INPUT_EPWM20_DELACTIVE = 42,
    //! Capture input is EPWM21 DEL ACTIVE Signal
    ECAP_INPUT_EPWM21_DELACTIVE = 43,
    //! Capture input is EPWM22 DEL ACTIVE Signal
    ECAP_INPUT_EPWM22_DELACTIVE = 44,
    //! Capture input is EPWM23 DEL ACTIVE Signal
    ECAP_INPUT_EPWM23_DELACTIVE = 45,
    //! Capture input is EPWM24 DEL ACTIVE Signal
    ECAP_INPUT_EPWM24_DELACTIVE = 46,
    //! Capture input is EPWM25 DEL ACTIVE Signal
    ECAP_INPUT_EPWM25_DELACTIVE = 47,
    //! Capture input is EPWM26 DEL ACTIVE Signal
    ECAP_INPUT_EPWM26_DELACTIVE = 48,
    //! Capture input is EPWM27 DEL ACTIVE Signal
    ECAP_INPUT_EPWM27_DELACTIVE = 49,
    //! Capture input is EPWM28 DEL ACTIVE Signal
    ECAP_INPUT_EPWM28_DELACTIVE = 50,
    //! Capture input is EPWM29 DEL ACTIVE Signal
    ECAP_INPUT_EPWM29_DELACTIVE = 51,
    //! Capture input is EPWM30 DEL ACTIVE Signal
    ECAP_INPUT_EPWM30_DELACTIVE = 52,
    //! Capture input is EPWM31 DEL ACTIVE Signal
    ECAP_INPUT_EPWM31_DELACTIVE = 53,
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
//! Values that can be passed to ECAP_selectQualPeriod() as the \e mode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! Bypass
    ECAP_PULSE_WIDTH_FILTER_BYPASS         = 0x0,
    //! Pulse width less than 1 cycle
    ECAP_PULSE_WIDTH_FILTER_CYCLE1         = 0x1,
    //! Pulse width less than 2 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE2         = 0x2,
    //! Pulse width less than 3 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE3         = 0x3,
    //! Pulse width less than 4 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE4         = 0x4,
    //! Pulse width less than 5 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE5         = 0x5,
    //! Pulse width less than 6 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE6         = 0x6,
    //! Pulse width less than 7 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE7         = 0x7,
    //! Pulse width less than 8 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE8         = 0x8,
    //! Pulse width less than 9 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE9         = 0x9,
    //! Pulse width less than 10 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE10        = 0xA,
    //! Pulse width less than 11 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE11        = 0xB,
    //! Pulse width less than 12 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE12        = 0xC,
    //! Pulse width less than 13 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE13        = 0xD,
    //! Pulse width less than 14 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE14        = 0xE,
    //! Pulse width less than 15 cycles
    ECAP_PULSE_WIDTH_FILTER_CYCLE15        = 0xF,
}ECAP_QualPeriodSelect;

//*****************************************************************************
//
//! Values that can be passed to ECAP_setDMASource() as the \e triggersource
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ECAP_CAP_MODE_DMA_TRIGGER_SRC_CEVT1 = 0,  //!< eCAP event 1 as DMA trigger source in capture mode
    ECAP_CAP_MODE_DMA_TRIGGER_SRC_CEVT2 = 1,  //!< eCAP event 2 as DMA trigger source in capture mode
    ECAP_CAP_MODE_DMA_TRIGGER_SRC_CEVT3 = 2,  //!< eCAP event 3 as DMA trigger source in capture mode
    ECAP_CAP_MODE_DMA_TRIGGER_SRC_CEVT4 = 3,  //!< eCAP event 4 as DMA trigger source in capture mode
    ECAP_APWM_MODE_DMA_TRIGGER_SRC_PRD = ECAP_CAP_MODE_DMA_TRIGGER_SRC_CEVT1, //!< eCAP PRD match as DMA trigger source in APWM mode
    ECAP_APWM_MODE_DMA_TRIGGER_SRC_CMP = ECAP_CAP_MODE_DMA_TRIGGER_SRC_CEVT2, //!< eCAP CMP match as DMA trigger source in APWM mode
    ECAP_APWM_MODE_DMA_TRIGGER_SRC_PRD_CMP = ECAP_CAP_MODE_DMA_TRIGGER_SRC_CEVT3, //!< eCAP PRD or CMP match as DMA trigger source in APWM mode
    ECAP_APWM_MODE_DMA_TRIGGER_SRC_DISABLED = ECAP_CAP_MODE_DMA_TRIGGER_SRC_CEVT4 //!< eCAP DMA trigger source disabled in APWM mode
}ECAP_DmaTriggerSource;
//*****************************************************************************
//
//! Values that can be passed to ECAP_selectSocTriggerSource() as the \e triggersource
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT1 = 0,  //!< eCAP event 1 as ADC SOC trigger source in capture mode
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT2 = 1,  //!< eCAP event 2 as ADC SOC trigger source in capture mode
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT3 = 2,  //!< eCAP event 3 as ADC SOC trigger source in capture mode
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT4 = 3,  //!< eCAP event 4 as ADC SOC trigger source in capture mode
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD = ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT1, //!< eCAP PRD match as ADC SOC trigger source in APWM mode
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_CMP = ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT2, //!< eCAP CMP match as ADC SOC trigger source in APWM mode
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD_CMP = ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT3, //!< eCAP PRD or CMP match as ADC SOC trigger source in APWM mode
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_DISABLED = ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT4 //!< eCAP ADC SOC trigger source disabled in APWM mode
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
// Values that can be passed to ECAP Signal Monitoring APIs
//
//*****************************************************************************
//! ECAP Monitoring Unit 1
#define ECAP_MONITORING_UNIT_1       0U
//! ECAP Monitoring Unit 2
#define ECAP_MONITORING_UNIT_2       1U

//*****************************************************************************
//
//! Values that can be passed to ECAP_selectShadowLoadMode() as the
//! \e loadMode parameter.
//
//*****************************************************************************
//! Load on next sync event
#define ECAP_ACTIVE_LOAD_SYNC_EVT            0U
//! Load on EPWM GLDLCSTRB event
#define ECAP_ACTIVE_LOAD_GLDLCSTRB_EVT       1U

//*****************************************************************************
//
//! Values that can be passed to ECAP_selectMonitoringType() as the
//! \e monSel parameter.
//
//*****************************************************************************
typedef enum
{
    //! High Pulse Width
    ECAP_MUNIT_HIGH_PULSE_WIDTH = 0U,
    //! Low Pulse Width
    ECAP_MUNIT_LOW_PULSE_WIDTH = 1U,
    //! Period width from rise to rise
    ECAP_MUNIT_PERIOD_WIDTH_RISE_RISE = 2U,
    //! Period width from fall to fall
    ECAP_MUNIT_PERIOD_WIDTH_FALL_FALL = 3U,
    //! Monitor rise edge
    ECAP_MUNIT_MONITOR_RISE_EDGE = 4U,
    //! Monitor fall edge
    ECAP_MUNIT_MONITOR_FALL_EDGE = 5U,
}ECAP_MonitoringTypeSelect;

//*****************************************************************************
//
//! Values that can be passed to ECAP_selectTripSignal() as the \e tripSel.
//
//*****************************************************************************
typedef enum
{
    //! Disabled
    ECAP_MUNIT_TRIP_DISABLED = 0U,
    //! MUNIT trip source is PWMXBAR output 0
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT0 = 1U,
    //! MUNIT trip source is PWMXBAR output 1
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT1 = 2U,
    //! MUNIT trip source is PWMXBAR output 2
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT2 = 3U,
    //! MUNIT trip source is PWMXBAR output 3
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT3 = 4U,
    //! MUNIT trip source is PWMXBAR output 4
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT4 = 5U,
    //! MUNIT trip source is PWMXBAR output 5
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT5 = 6U,
    //! MUNIT trip source is PWMXBAR output 6
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT6 = 7U,
    //! MUNIT trip source is PWMXBAR output 7
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT7 = 8U,
    //! MUNIT trip source is PWMXBAR output 8
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT8 = 9U,
    //! MUNIT trip source is PWMXBAR output 9
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT9 = 10U,
    //! MUNIT trip source is PWMXBAR output 10
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT10 = 11U,
    //! MUNIT trip source is PWMXBAR output 11
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT11 = 12U,
    //! MUNIT trip source is PWMXBAR output 12
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT12 = 13U,
    //! MUNIT trip source is PWMXBAR output 13
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT13 = 14U,
    //! MUNIT trip source is PWMXBAR output 14
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT14 = 15U,
    //! MUNIT trip source is PWMXBAR output 15
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT15 = 16U,
    //! MUNIT trip source is PWMXBAR output 16
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT16 = 17U,
    //! MUNIT trip source is PWMXBAR output 17
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT17 = 18U,
    //! MUNIT trip source is PWMXBAR output 18
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT18 = 19U,
    //! MUNIT trip source is PWMXBAR output 19
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT19 = 20U,
    //! MUNIT trip source is PWMXBAR output 20
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT20 = 21U,
    //! MUNIT trip source is PWMXBAR output 21
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT21 = 22U,
    //! MUNIT trip source is PWMXBAR output 22
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT22 = 23U,
    //! MUNIT trip source is PWMXBAR output 23
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT23 = 24U,
    //! MUNIT trip source is PWMXBAR output 24
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT24 = 25U,
    //! MUNIT trip source is PWMXBAR output 25
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT25 = 26U,
    //! MUNIT trip source is PWMXBAR output 26
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT26 = 27U,
    //! MUNIT trip source is PWMXBAR output 27
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT27 = 28U,
    //! MUNIT trip source is PWMXBAR output 28
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT28 = 29U,
    //! MUNIT trip source is PWMXBAR output 29
    ECAP_MUNIT_TRIP_EPWM_XBAR_OUT29 = 30U,
    //! MUNIT trip source is EPWM0 trip out signal
    ECAP_MUNIT_TRIP_EPWM0_TRIPOUT = 32U,
    //! MUNIT trip source is EPWM1 trip out signal
    ECAP_MUNIT_TRIP_EPWM1_TRIPOUT = 33U,
    //! MUNIT trip source is EPWM2 trip out signal
    ECAP_MUNIT_TRIP_EPWM2_TRIPOUT = 34U,
    //! MUNIT trip source is EPWM3 trip out signal
    ECAP_MUNIT_TRIP_EPWM3_TRIPOUT = 35U,
    //! MUNIT trip source is EPWM4 trip out signal
    ECAP_MUNIT_TRIP_EPWM4_TRIPOUT = 36U,
    //! MUNIT trip source is EPWM5 trip out signal
    ECAP_MUNIT_TRIP_EPWM5_TRIPOUT = 37U,
    //! MUNIT trip source is EPWM6 trip out signal
    ECAP_MUNIT_TRIP_EPWM6_TRIPOUT = 38U,
    //! MUNIT trip source is EPWM7 trip out signal
    ECAP_MUNIT_TRIP_EPWM7_TRIPOUT = 39U,
    //! MUNIT trip source is EPWM8 trip out signal
    ECAP_MUNIT_TRIP_EPWM8_TRIPOUT = 40U,
    //! MUNIT trip source is EPWM9 trip out signal
    ECAP_MUNIT_TRIP_EPWM9_TRIPOUT = 41U,
    //! MUNIT trip source is EPWM10 trip out signal
    ECAP_MUNIT_TRIP_EPWM10_TRIPOUT = 42U,
    //! MUNIT trip source is EPWM11 trip out signal
    ECAP_MUNIT_TRIP_EPWM11_TRIPOUT = 43U,
    //! MUNIT trip source is EPWM12 trip out signal
    ECAP_MUNIT_TRIP_EPWM12_TRIPOUT = 44U,
    //! MUNIT trip source is EPWM13 trip out signal
    ECAP_MUNIT_TRIP_EPWM13_TRIPOUT = 45U,
    //! MUNIT trip source is EPWM14 trip out signal
    ECAP_MUNIT_TRIP_EPWM14_TRIPOUT = 46U,
    //! MUNIT trip source is EPWM15 trip out signal
    ECAP_MUNIT_TRIP_EPWM15_TRIPOUT = 47U,
    //! MUNIT trip source is EPWM16 trip out signal
    ECAP_MUNIT_TRIP_EPWM16_TRIPOUT = 48U,
    //! MUNIT trip source is EPWM17 trip out signal
    ECAP_MUNIT_TRIP_EPWM17_TRIPOUT = 49U,
    //! MUNIT trip source is EPWM18 trip out signal
    ECAP_MUNIT_TRIP_EPWM18_TRIPOUT = 50U,
    //! MUNIT trip source is EPWM19 trip out signal
    ECAP_MUNIT_TRIP_EPWM19_TRIPOUT = 51U,
    //! MUNIT trip source is EPWM20 trip out signal
    ECAP_MUNIT_TRIP_EPWM20_TRIPOUT = 52U,
    //! MUNIT trip source is EPWM21 trip out signal
    ECAP_MUNIT_TRIP_EPWM21_TRIPOUT = 53U,
    //! MUNIT trip source is EPWM22 trip out signal
    ECAP_MUNIT_TRIP_EPWM22_TRIPOUT = 54U,
    //! MUNIT trip source is EPWM23 trip out signal
    ECAP_MUNIT_TRIP_EPWM23_TRIPOUT = 55U,
    //! MUNIT trip source is EPWM24 trip out signal
    ECAP_MUNIT_TRIP_EPWM24_TRIPOUT = 56U,
    //! MUNIT trip source is EPWM25 trip out signal
    ECAP_MUNIT_TRIP_EPWM25_TRIPOUT = 57U,
    //! MUNIT trip source is EPWM26 trip out signal
    ECAP_MUNIT_TRIP_EPWM26_TRIPOUT = 58U,
    //! MUNIT trip source is EPWM27 trip out signal
    ECAP_MUNIT_TRIP_EPWM27_TRIPOUT = 59U,
    //! MUNIT trip source is EPWM28 trip out signal
    ECAP_MUNIT_TRIP_EPWM28_TRIPOUT = 60U,
    //! MUNIT trip source is EPWM29 trip out signal
    ECAP_MUNIT_TRIP_EPWM29_TRIPOUT = 61U,
    //! MUNIT trip source is EPWM30 trip out signal
    ECAP_MUNIT_TRIP_EPWM30_TRIPOUT = 62U,
    //! MUNIT trip source is EPWM31 trip out signal
    ECAP_MUNIT_TRIP_EPWM31_TRIPOUT = 63U,
}ECAP_MunitTripInputSelect;

//*****************************************************************************
//
//! Values that can be passed to ECAP_selectGlobalLoadStrobe() as the \e strobe.
//
//*****************************************************************************
typedef enum
{
    //! Disabled
    ECAP_MUNIT_GLDSTRB_DISABLED = 0U,
    //! MUNIT global load strobe is EPWM0's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM0 = 1U,
    //! MUNIT global load strobe is EPWM1's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM1 = 2U,
    //! MUNIT global load strobe is EPWM2's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM2 = 3U,
    //! MUNIT global load strobe is EPWM3's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM3 = 4U,
    //! MUNIT global load strobe is EPWM4's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM4 = 5U,
    //! MUNIT global load strobe is EPWM5's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM5 = 6U,
    //! MUNIT global load strobe is EPWM6's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM6 = 7U,
    //! MUNIT global load strobe is EPWM7's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM7 = 8U,
    //! MUNIT global load strobe is EPWM8's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM8 = 9U,
    //! MUNIT global load strobe is EPWM9's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM9 = 10U,
    //! MUNIT global load strobe is EPWM10's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM10 = 11U,
    //! MUNIT global load strobe is EPWM11's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM11 = 12U,
    //! MUNIT global load strobe is EPWM12's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM12 = 13U,
    //! MUNIT global load strobe is EPWM13's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM13 = 14U,
    //! MUNIT global load strobe is EPWM14's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM14 = 15U,
    //! MUNIT global load strobe is EPWM15's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM15 = 16U,
    //! MUNIT global load strobe is EPWM16's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM16 = 17U,
    //! MUNIT global load strobe is EPWM17's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM17 = 18U,
    //! MUNIT global load strobe is EPWM18's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM18 = 19U,
    //! MUNIT global load strobe is EPWM19's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM19 = 20U,
    //! MUNIT global load strobe is EPWM20's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM20 = 21U,
    //! MUNIT global load strobe is EPWM21's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM21 = 22U,
    //! MUNIT global load strobe is EPWM22's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM22 = 23U,
    //! MUNIT global load strobe is EPWM23's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM23 = 24U,
    //! MUNIT global load strobe is EPWM24's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM24 = 25U,
    //! MUNIT global load strobe is EPWM25's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM25 = 26U,
    //! MUNIT global load strobe is EPWM26's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM26 = 27U,
    //! MUNIT global load strobe is EPWM27's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM27 = 28U,
    //! MUNIT global load strobe is EPWM28's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM28 = 29U,
    //! MUNIT global load strobe is EPWM29's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM29 = 30U,
    //! MUNIT global load strobe is EPWM30's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM30 = 31U,
    //! MUNIT global load strobe is EPWM31's global load strobe
    ECAP_MUNIT_GLDSTRB_EPWM31 = 32U,
}ECAP_MunitGlobalStrobeSelect;

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
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_1 - Enables Event 1 generating interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_2 - Enables Event 2 generating interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_3 - Enables Event 3 generating interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_4 - Enables Event 4 generating interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_OVERFLOW - Enables Counter overflow generating interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_PERIOD   - Enables Counter equal period generating
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE  - Enables Counter equal compare generating
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 - Enables Monitoring unit 1 error event 1
//!                                         generating interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 - Enables Monitoring unit 1 error event 2
//!                                         generating interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1 - Enables Monitoring unit 2 error event 1
//!                                         generating interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2 - Enables Monitoring unit 2 error event 2
//!                                         generating interrupt
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
                   ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 |
                   ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 |
                   ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1 |
                   ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2)) == 0U);

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
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_1 - Disables Event 1 generating interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_2 - Disables Event 2 generating interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_3 - Disables Event 3 generating interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_4 - Disables Event 4 generating interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_OVERFLOW - Disables Counter overflow generating interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_PERIOD   - Disables Counter equal period generating
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE  - Disables Counter equal compare generating
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 - Disables Monitoring unit 1 error event 1
//!                                         generating interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 - Disables Monitoring unit 1 error event 2
//!                                         generating interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1 - Disables Monitoring unit 2 error event 1
//!                                         generating interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2 - Disables Monitoring unit 2 error event 2
//!                                         generating interrupt
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
                   ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 |
                   ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 |
                   ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1 |
                   ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2)) == 0U);

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
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE  - Counter equal compare generates
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 - Monitoring unit 1 error event 1
//!                                         generates interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 - Monitoring unit 1 error event 2
//!                                         generates interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1 - Monitoring unit 2 error event 1
//!                                         generates interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2 - Monitoring unit 2 error event 2
//!                                         generates interrupt
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
    return(HW_RD_REG16(base + CSL_ECAP_ECFLG) & 0x1EFEU);
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
//!  - ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 - Monitoring unit 1 error event 1
//!                                         generates interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 - Monitoring unit 1 error event 2
//!                                         generates interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1 - Monitoring unit 2 error event 1
//!                                         generates interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2 - Monitoring unit 2 error event 2
//!                                         generates interrupt
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
                   ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 |
                   ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 |
                   ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1 |
                   ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2)) == 0U);

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
//!  - ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 - Monitoring unit 1 error event 1
//!                                         generates interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 - Monitoring unit 1 error event 2
//!                                         generates interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1 - Monitoring unit 2 error event 1
//!                                         generates interrupt
//!  - ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2 - Monitoring unit 2 error event 2
//!                                         generates interrupt
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
                   ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 |
                   ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 |
                   ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1 |
                   ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2
                   )) == 0U);

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
//! This function returns the current time stamp count of the given event in capture and apwm mode.
//! Valid values for \e event are \b ECAP_EVENT_1 to \b ECAP_EVENT_4.
//!
//! In capture mode of operation -
//! \b ECAP_EVENT_1 - Read CAP1 register
//! \b ECAP_EVENT_2 - Read CAP2 register
//! \b ECAP_EVENT_3 - Read CAP3 register
//! \b ECAP_EVENT_4 - Read CAP4 register
//!
//! In APWM mode of operation -
//! \b ECAP_EVENT_1 - Read PRD ACTIVE register
//! \b ECAP_EVENT_2 - Read CMP ACTIVE register
//! \b ECAP_EVENT_3 - Read PRD SHADOW register
//! \b ECAP_EVENT_4 - Read CMP SHADOW register
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
//! Select qualification period to filter out noise
//!
//! \param base is the base address of the ECAP module.
//! \param width is the pulse width below which the pulse
//!              will be filtered out.
//!
//! This function selects the qualification period to filter out pulses with
//! width less than the mentioned number of cycles.
//!
//! Please refer to the ::ECAP_QualPeriodSelect Enum for the valid values
//! to be passed to \e width parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_selectQualPeriod(uint32_t base,
                                         ECAP_QualPeriodSelect width)
{
    //
    // Write to ECCTL0 - Qualification Period
    //
    HW_WR_REG32(base + CSL_ECAP_ECCTL0,
               ((HW_RD_REG32(base + CSL_ECAP_ECCTL0) & ~CSL_ECAP_ECCTL0_QUALPRD_MASK) |
                ((uint32_t)width << CSL_ECAP_ECCTL0_QUALPRD_SHIFT)));
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
    HW_WR_REG32(base + CSL_ECAP_ECCTL0,
        ((HW_RD_REG32(base + CSL_ECAP_ECCTL0) &
        ~CSL_ECAP_ECCTL0_SOCEVTSEL_MASK) | (triggersource << CSL_ECAP_ECCTL0_SOCEVTSEL_SHIFT)));
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
//! \param triggersource is the eCAP event for the DMA
//!
//! Please refer to the ::ECAP_DmaTriggerSource Enum for the valid values
//! to be passed to \e triggersource parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setDMASource(uint32_t base, ECAP_DmaTriggerSource triggersource)
{
    //
    // Write to ECCTL2
    //
    HW_WR_REG16(base + CSL_ECAP_ECCTL2,
        ((HW_RD_REG16(base + CSL_ECAP_ECCTL2) &
        ~CSL_ECAP_ECCTL2_DMAEVTSEL_MASK) |
        ((uint16_t)triggersource << CSL_ECAP_ECCTL2_DMAEVTSEL_SHIFT)));
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

    counterStatusValue = (((HW_RD_REG16(base + CSL_ECAP_ECCTL2) &
                          CSL_ECAP_ECCTL2_MODCNTRSTS_MASK) >>
                          CSL_ECAP_ECCTL2_MODCNTRSTS_SHIFT));

    //
    // Read MODCNTRSTS bit
    //
    return((ECAP_Events)(counterStatusValue));
}

//
// Signal Monitoring related APIs
//
//*****************************************************************************
//
//! Enable eCAP monitoring unit.
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//!
//! This function enables the eCAP signal monitoring unit.
//! Valid values for the input variables are:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_enableSignalMonitoringUnit(uint32_t base, uint32_t munit)
{
    uint32_t munitOffset;
    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_CTL;

    //
    // Enable MUNIT
    //
    HW_WR_REG32(base + munitOffset,
        (HW_RD_REG32(base + munitOffset) |
        CSL_ECAP_MUNIT_1_CTL_EN_MASK));
}

//*****************************************************************************
//
//! Disable eCAP monitoring unit.
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//!
//! This function disables the eCAP signal monitoring unit.
//! Valid values for the input variables are:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_disableSignalMonitoringUnit(uint32_t base, uint32_t munit)
{
    uint32_t munitOffset;
    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_CTL;

    //
    // Disable MUNIT
    //
    HW_WR_REG32(base + munitOffset,
        (HW_RD_REG32(base + munitOffset) &
        ~CSL_ECAP_MUNIT_1_CTL_EN_MASK));
}

//*****************************************************************************
//
//! Enables debug mode to capture range from min to max
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//!
//! This function enables the eCAP debug mode for signal monitoring.
//! Range is captured in DEBUG_RANGE_MAX and DEBUG_RANGE_MIN registers.
//!
//! Valid values for the input variables are:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_enableDebugRange(uint32_t base, uint32_t munit)
{
    uint32_t munitOffset;

    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_CTL;

    //
    // Enable debug mode for MUNIT 1
    //
    HW_WR_REG32(base + munitOffset,
        (HW_RD_REG32(base + munitOffset) |
        CSL_ECAP_MUNIT_1_CTL_DEBUG_RANGE_EN_MASK));
}

//*****************************************************************************
//
//! Disables debug mode to capture range from min to max
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//!
//! This function disables the eCAP debug mode for signal monitoring.
//!
//! Valid values for the input variables are:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_disableDebugRange(uint32_t base, uint32_t munit)
{
    uint32_t munitOffset;
    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_CTL;

    //
    // Disable debug mode for MUNIT
    //
    HW_WR_REG32(base + munitOffset,
        (HW_RD_REG32(base + munitOffset) &
        ~CSL_ECAP_MUNIT_1_CTL_DEBUG_RANGE_EN_MASK));
}

//*****************************************************************************
//
//! Selects the type of monitoring
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//! \param monSel is the type of monitoring to be selected.
//!
//! Valid values for the input variables are:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//! - monSel:
//!    - ECAP_MUNIT_HIGH_PULSE_WIDTH       - High Pulse Width
//!    - ECAP_MUNIT_LOW_PULSE_WIDTH        - Low Pulse Width
//!    - ECAP_MUNIT_PERIOD_WIDTH_RISE_RISE - Period width from rise to rise
//!    - ECAP_MUNIT_PERIOD_WIDTH_FALL_FALL - Period width from fall to fall
//!    - ECAP_MUNIT_MONITOR_RISE_EDGE      - Monitor rise edge
//!    - ECAP_MUNIT_MONITOR_FALL_EDGE      - Monitor fall edge
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_selectMonitoringType(uint32_t base, uint32_t munit,
                          ECAP_MonitoringTypeSelect monSel)
{
    uint32_t munitOffset;
    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_CTL;

    //
    // Select Monitoring Type for MUNIT
    //
    HW_WR_REG32(base + munitOffset,
            (HW_RD_REG32(base + munitOffset) &
            ~CSL_ECAP_MUNIT_1_CTL_MON_SEL_MASK) |
            ((uint32_t)monSel << CSL_ECAP_MUNIT_1_CTL_MON_SEL_SHIFT));
}

//*****************************************************************************
//
//! Selects the trip signal to disable and enable monitoring automatically.
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param tripSel is the trip signal.
//!
//! Valid values for the input variables are:
//! - tripSel:
//!     - DISABLED
//!     - ECAP_MUNIT_TRIP_EPWM_XBAR_OUTn - n is 0-29
//!     - ECAP_MUNIT_TRIP_EPWMn_TRIPOUT - n is 0-31
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_selectTripSignal(uint32_t base, ECAP_MunitTripInputSelect tripSel)
{
    //
    // Select Monitoring Type
    //
    HW_WR_REG32(base + CSL_ECAP_MUNIT_COMMON_CTL,
            (HW_RD_REG32(base + CSL_ECAP_MUNIT_COMMON_CTL) &
             ~CSL_ECAP_MUNIT_COMMON_CTL_TRIPSEL_MASK) |
            ((uint32_t)tripSel << CSL_ECAP_MUNIT_COMMON_CTL_TRIPSEL_SHIFT));
}

//*****************************************************************************
//
//! Selects the global load strobe to enable shadow to active loading
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param strobe is the type of monitoring to be selected.
//!
//! Valid values for the input variables are:
//! - strobe:
//!     - DISABLED
//!     - ECAP_MUNIT_GLDSTRB_EPWMn - n is 0-31
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_selectGlobalLoadStrobe(uint32_t base, ECAP_MunitGlobalStrobeSelect strobe)
{
    //
    // Select the global load strobe
    //
    HW_WR_REG32(base + CSL_ECAP_MUNIT_COMMON_CTL,
            (HW_RD_REG32(base + CSL_ECAP_MUNIT_COMMON_CTL) &
             ~CSL_ECAP_MUNIT_COMMON_CTL_GLDSTRBSEL_MASK) |
            ((uint32_t)strobe << CSL_ECAP_MUNIT_COMMON_CTL_GLDSTRBSEL_SHIFT));
}

//*****************************************************************************
//
//! Enables shadowing for min and max registers
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//!
//! This function enables the shadowing feature for min and max registers.
//!
//! Valid values for the input variable:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_enableShadowMinMaxRegisters(uint32_t base, uint32_t munit)
{
    uint32_t munitOffset;
    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_SHADOW_CTL;

    //
    // Enable shadowing for MUNIT 1
    //
    HW_WR_REG32(base + munitOffset,
        (HW_RD_REG32(base + munitOffset) |
        CSL_ECAP_MUNIT_1_SHADOW_CTL_SYNCI_EN_MASK));
}

//*****************************************************************************
//
//! Disables shadowing for min and max registers
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//!
//! This function disables the shadowing feature for min and max registers.
//!
//! Valid values for the input variable:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_disableShadowMinMaxRegisters(uint32_t base, uint32_t munit)
{
    uint32_t munitOffset;

    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_SHADOW_CTL;

    //
    // Disable shadowing for MUNIT 1
    //
    HW_WR_REG32(base + munitOffset,
        (HW_RD_REG32(base + munitOffset) &
        ~CSL_ECAP_MUNIT_1_SHADOW_CTL_SYNCI_EN_MASK));
}

//*****************************************************************************
//
//! Enables software sync operation
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//!
//! This function enables the SW Sync to copy min and max values from
//! shadow to active registers immediately if shadowing is enabled.
//!
//! Valid values for the input variable:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_enableSoftwareSync(uint32_t base, uint32_t munit)
{
    uint32_t munitOffset;

    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_SHADOW_CTL;

    //
    // Generates SW Sync for MUNIT
    //
    HW_WR_REG32(base + munitOffset,
        (HW_RD_REG32(base + munitOffset) |
        CSL_ECAP_MUNIT_1_SHADOW_CTL_SWSYNC_MASK));
}

//*****************************************************************************
//
//! Selects the shadow to active load mode
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//! \param loadMode is the shadow to active mode to be selected.
//!
//! Valid values for the input variables are:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//! - loadMode:
//!    - ECAP_ACTIVE_LOAD_SYNC_EVT       - Active loaded with shadow on
//!                                        next sync event
//!    - ECAP_ACTIVE_LOAD_GLDLCSTRB_EVT  - Active loaded with shadow on
//!                                        EPWM GLDLCSTRB event
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_selectShadowLoadMode(uint32_t base, uint32_t munit,
                          uint32_t loadMode)
{
    uint32_t munitOffset;

    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_SHADOW_CTL;

    //
    // Select shadow to active load mode for MUNIT
    //
    HW_WR_REG32(base + munitOffset,
            (HW_RD_REG32(base + munitOffset) &
             ~CSL_ECAP_MUNIT_1_SHADOW_CTL_LOADMODE_MASK) |
            (loadMode << CSL_ECAP_MUNIT_1_SHADOW_CTL_LOADMODE_SHIFT));
}

//*****************************************************************************
//
//! Configure minimum value for monitoring
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//! \param minValue is the minimum value for monitoring.
//! Valid values for the input variables are:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//! - minValue:
//!    - Range from 0x0 to 0xFFFFFFFF
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_configureMinValue(uint32_t base, uint32_t munit,
                       uint32_t minValue)
{
    uint32_t munitOffset;
    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_MIN;

    //
    // Load minimum value for monitoring
    //
    HW_WR_REG32(base + munitOffset, minValue);
}

//*****************************************************************************
//
//! Configure maximum value for monitoring
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//! \param maxValue is the maximum value for monitoring.
//!
//! Valid values for the input variables are:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//! - maxValue:
//!    - Range from 0x0 to 0xFFFFFFFF
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_configureMaxValue(uint32_t base, uint32_t munit,
                       uint32_t maxValue)
{
    uint32_t munitOffset;

    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_MAX;

    //
    // Load maximum value for monitoring
    //
    HW_WR_REG32(base + munitOffset, maxValue);
}

//*****************************************************************************
//
//! Configure minimum value for shadow register
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//! \param minValue is the minimum value for monitoring.
//! Valid values for the input variables are:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//! - minValue:
//!    - Range from 0x0 to 0xFFFFFFFF
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_configureShadowMinValue(uint32_t base, uint32_t munit,
                             uint32_t minValue)
{
    uint32_t munitOffset;

    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_MIN_SHADOW;
    //
    // Load minimum value for monitoring
    //
    HW_WR_REG32(base + munitOffset, minValue);
}

//*****************************************************************************
//
//! Configure maximum value for shadow register
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//! \param maxValue is the maximum value for monitoring.
//!
//! Valid values for the input variables are:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//! - maxValue:
//!    - Range from 0x0 to 0xFFFFFFFF
//!
//! \return None.
//
//*****************************************************************************
static inline void
ECAP_configureShadowMaxValue(uint32_t base, uint32_t munit,
                             uint32_t maxValue)
{
    uint32_t munitOffset;

    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_MAX_SHADOW;

    //
    // Load maximum value for monitoring
    //
    HW_WR_REG32(base + munitOffset, maxValue);
}

//*****************************************************************************
//
//! Returns observed minimum value
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//!
//! This function returns the observed minimum value when the DEBUG_RANGE_EN
//! bit is set to 1.
//!
//! Valid values for the input variables are:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//!
//! \return returns the observed minimum value.
//
//*****************************************************************************
static inline uint32_t
ECAP_observedMinValue(uint32_t base, uint32_t munit)
{
    uint32_t munitOffset;

    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_DEBUG_RANGE_MIN;

    //
    // Returns minimum value
    //
    return(HW_RD_REG32(base + munitOffset));
}

//*****************************************************************************
//
//! Returns observed maximum value
//!
//! \param base is the base address of the ECAP signal monitoring module.
//! \param munit is the monitoring unit, either 1 or 2.
//!
//! This function returns the observed maximum value when the DEBUG_RANGE_EN
//! bit is set to 1.
//!
//! Valid values for the input variables are:
//! - munit:
//!    - ECAP_MONITORING_UNIT_1 - ECAP Monitoring Unit 1
//!    - ECAP_MONITORING_UNIT_2 - ECAP Monitoring Unit 2
//!
//! \return returns the observed maximum value.
//
//*****************************************************************************
static inline uint32_t
ECAP_observedMaxValue(uint32_t base, uint32_t munit)
{
    uint32_t munitOffset;

    //
    // Get the offset to the appropriate MUNIT configuration register.
    //
    munitOffset = (ECAP_MUNIT_STEP * munit) + CSL_ECAP_MUNIT_1_DEBUG_RANGE_MAX;

    //
    // Returns maximum value
    //
    return(HW_RD_REG32(base + munitOffset));
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

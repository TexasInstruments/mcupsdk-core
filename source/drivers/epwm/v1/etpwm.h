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
 *  \defgroup DRV_EPWM_MODULE APIs for EPWM
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the EPWM module.
 *
 *  @{
 */

#ifndef EPWM_V1_H_
#define EPWM_V1_H_

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
// Header Files
//
//*****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/cslr_epwm.h>

//*****************************************************************************
//
// Defines for the API.
//
//*****************************************************************************
//*****************************************************************************
//
// Define to specify mask for source parameter for
// EPWM_enableSyncOutPulseSource() & EPWM_disableSyncOutPulseSource()
//
//*****************************************************************************
#define EPWM_SYNC_OUT_SOURCE_M ((uint16_t)CSL_EPWM_EPWMSYNCOUTEN_SWEN_MASK   |\
                             (uint16_t)CSL_EPWM_EPWMSYNCOUTEN_ZEROEN_MASK    |\
                             (uint16_t)CSL_EPWM_EPWMSYNCOUTEN_CMPBEN_MASK    |\
                             (uint16_t)CSL_EPWM_EPWMSYNCOUTEN_CMPCEN_MASK    |\
                             (uint16_t)CSL_EPWM_EPWMSYNCOUTEN_CMPDEN_MASK    |\
                             (uint16_t)CSL_EPWM_EPWMSYNCOUTEN_DCAEVT1EN_MASK |\
                             (uint16_t)CSL_EPWM_EPWMSYNCOUTEN_DCBEVT1EN_MASK)

//*****************************************************************************
//
// Values that can be passed to EPWM_enableSyncOutPulseSource() &
// EPWM_disableSyncOutPulseSource() as the \e mode parameter.
//
//*****************************************************************************
//! Software force generated EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_SOFTWARE  CSL_EPWM_EPWMSYNCOUTEN_SWEN_MASK
//! Counter zero event generates EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO  CSL_EPWM_EPWMSYNCOUTEN_ZEROEN_MASK
//! Counter equal to CMPB event generates EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_B  CSL_EPWM_EPWMSYNCOUTEN_CMPBEN_MASK
//! Counter equal to CMPC event generates EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_C  CSL_EPWM_EPWMSYNCOUTEN_CMPCEN_MASK
//! Counter equal to CMPD event generates EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_D  CSL_EPWM_EPWMSYNCOUTEN_CMPDEN_MASK
//! DCA Event 1 Sync signal generates EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_DCA_EVT1_SYNC  CSL_EPWM_EPWMSYNCOUTEN_DCAEVT1EN_MASK
//! DCB Event 1 Sync signal generates EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_DCB_EVT1_SYNC  CSL_EPWM_EPWMSYNCOUTEN_DCBEVT1EN_MASK
//! Enable all the above sources
#define EPWM_SYNC_OUT_PULSE_ON_ALL  EPWM_SYNC_OUT_SOURCE_M

//
// Time Base Module
//
//*****************************************************************************
//
//! Values that can be passed to EPWM_setEmulationMode() as the
//! \e emulationMode parameter.
//
//*****************************************************************************
typedef enum
{
   //! Stop after next Time Base counter increment or decrement.
   EPWM_EMULATION_STOP_AFTER_NEXT_TB = 0,
   //! Stop when counter completes whole cycle
   EPWM_EMULATION_STOP_AFTER_FULL_CYCLE = 1,
   //! Free run
   EPWM_EMULATION_FREE_RUN = 2
} EPWM_EmulationMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setCountModeAfterSync() as the
//! \e mode parameter.
//
//*****************************************************************************
typedef enum
{
   EPWM_COUNT_MODE_DOWN_AFTER_SYNC = 0,  //!< Count down after sync event
   EPWM_COUNT_MODE_UP_AFTER_SYNC = 1  //!< Count up after sync event
} EPWM_SyncCountMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setClockPrescaler() as the
//! \e prescaler parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_CLOCK_DIVIDER_1 = 0,  //!< Divide clock by 1
    EPWM_CLOCK_DIVIDER_2 = 1,  //!< Divide clock by 2
    EPWM_CLOCK_DIVIDER_4 = 2,  //!< Divide clock by 4
    EPWM_CLOCK_DIVIDER_8 = 3,  //!< Divide clock by 8
    EPWM_CLOCK_DIVIDER_16 = 4,  //!< Divide clock by 16
    EPWM_CLOCK_DIVIDER_32 = 5,  //!< Divide clock by 32
    EPWM_CLOCK_DIVIDER_64 = 6,  //!< Divide clock by 64
    EPWM_CLOCK_DIVIDER_128 = 7  //!< Divide clock by 128
} EPWM_ClockDivider;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setClockPrescaler() as the
//! \e highSpeedPrescaler parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_HSCLOCK_DIVIDER_1 = 0,  //!< Divide clock by 1
    EPWM_HSCLOCK_DIVIDER_2 = 1,  //!< Divide clock by 2
    EPWM_HSCLOCK_DIVIDER_4 = 2,  //!< Divide clock by 4
    EPWM_HSCLOCK_DIVIDER_6 = 3,  //!< Divide clock by 6
    EPWM_HSCLOCK_DIVIDER_8 = 4,  //!< Divide clock by 8
    EPWM_HSCLOCK_DIVIDER_10 = 5,  //!< Divide clock by 10
    EPWM_HSCLOCK_DIVIDER_12 = 6,  //!< Divide clock by 12
    EPWM_HSCLOCK_DIVIDER_14 = 7   //!< Divide clock by 14
} EPWM_HSClockDivider;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setSyncInPulseSource() as the \e mode
//! parameter.
//! Note : ECAPx_SYNCOUT for x in [10,15] are not applicable for AM263x.
//
//*****************************************************************************
typedef enum
{
    //! Disable Sync-in
    EPWM_SYNC_IN_PULSE_SRC_DISABLE = 0x0,
    //! Sync-in source is EPWM0 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0 = 0x1,
    //! Sync-in source is EPWM1 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1 = 0x2,
    //! Sync-in source is EPWM2 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM2 = 0x3,
    //! Sync-in source is EPWM3 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM3 = 0x4,
    //! Sync-in source is EPWM4 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM4 = 0x5,
    //! Sync-in source is EPWM5 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM5 = 0x6,
    //! Sync-in source is EPWM6 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM6 = 0x7,
    //! Sync-in source is EPWM7 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM7 = 0x8,
    //! Sync-in source is EPWM8 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM8 = 0x9,
    //! Sync-in source is EPWM9 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM9 = 0xA,
    //! Sync-in source is EPWM10 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM10 = 0xB,
    //! Sync-in source is EPWM11 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM11 = 0xC,
    //! Sync-in source is EPWM12 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM12 = 0xD,
    //! Sync-in source is EPWM13 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM13 = 0xE,
    //! Sync-in source is EPWM14 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM14 = 0xF,
    //! Sync-in source is EPWM15 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM15 = 0x10,
    //! Sync-in source is EPWM16 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM16 = 0x11,
    //! Sync-in source is EPWM17 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM17 = 0x12,
    //! Sync-in source is EPWM18 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM18 = 0x13,
    //! Sync-in source is EPWM19 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM19 = 0x14,
    //! Sync-in source is EPWM20 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM20 = 0x15,
    //! Sync-in source is EPWM21 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM21 = 0x16,
    //! Sync-in source is EPWM22 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM22 = 0x17,
    //! Sync-in source is EPWM23 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM23 = 0x18,
    //! Sync-in source is EPWM24 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM24 = 0x19,
    //! Sync-in source is EPWM25 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM25 = 0x1A,
    //! Sync-in source is EPWM26 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM26 = 0x1B,
    //! Sync-in source is EPWM27 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM27 = 0x1C,
    //! Sync-in source is EPWM28 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM28 = 0x1D,
    //! Sync-in source is EPWM29 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM29 = 0x1E,
    //! Sync-in source is EPWM30 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM30 = 0x1F,
    //! Sync-in source is EPWM31 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM31 = 0x20,
    //! Sync-in source is ECAP0 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP0 = 0x40,
    //! Sync-in source is ECAP1 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP1 = 0x41,
    //! Sync-in source is ECAP2 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP2 = 0x42,
    //! Sync-in source is ECAP3 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP3 = 0x43,
    //! Sync-in source is ECAP4 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP4 = 0x44,
    //! Sync-in source is ECAP5 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP5 = 0x45,
    //! Sync-in source is ECAP6 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP6 = 0x46,
    //! Sync-in source is ECAP7 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP7 = 0x47,
    //! Sync-in source is ECAP8 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP8 = 0x48,
    //! Sync-in source is ECAP9 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP9 = 0x49,
    //! Sync-in source is ECAP10 sync-out signal. Note : these are not applicable for AM263x.
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP10 = 0x4A,
    //! Sync-in source is ECAP11 sync-out signal. Note : these are not applicable for AM263x.
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP11 = 0x4B,
    //! Sync-in source is ECAP12 sync-out signal. Note : these are not applicable for AM263x.
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP12 = 0x4C,
    //! Sync-in source is ECAP13 sync-out signal. Note : these are not applicable for AM263x.
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP13 = 0x4D,
    //! Sync-in source is ECAP14 sync-out signal. Note : these are not applicable for AM263x.
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP14 = 0x4E,
    //! Sync-in source is ECAP15 sync-out signal. Note : these are not applicable for AM263x.
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP15 = 0x4F,
    //! Sync-in source is Input XBAR out4 signal
    EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT4 = 0x50,
    //! Sync-in source is Input XBAR out20 signal
    EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT20 = 0x51,
    //! Sync-in source is C2K Timesync xbar sync pwm out0 signal
    EPWM_SYNC_IN_PULSE_SRC_C2K_TIMESYNC_XBAR_PWM_OUT0 = 0x58,
    //! Sync-in source is C2K Timesync xbar sync pwm out1 signal
    EPWM_SYNC_IN_PULSE_SRC_C2K_TIMESYNC_XBAR_PWM_OUT1 = 0x59,
    //! Sync-in source is FSI-RX0 RX Trigger 0 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX0_RX_TRIG0 = 0x60,
    //! Sync-in source is FSI-RX0 RX Trigger 1 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX0_RX_TRIG1 = 0x61,
    //! Sync-in source is FSI-RX0 RX Trigger 2 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX0_RX_TRIG2 = 0x62,
    //! Sync-in source is FSI-RX0 RX Trigger 3 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX0_RX_TRIG3 = 0x63,
    //! Sync-in source is FSI-RX1 RX Trigger 0 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX1_RX_TRIG0 = 0x64,
    //! Sync-in source is FSI-RX1 RX Trigger 1 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX1_RX_TRIG1 = 0x65,
    //! Sync-in source is FSI-RX1 RX Trigger 2 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX1_RX_TRIG2 = 0x66,
    //! Sync-in source is FSI-RX1 RX Trigger 3 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX1_RX_TRIG3 = 0x67,
    //! Sync-in source is FSI-RX2 RX Trigger 0 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX2_RX_TRIG0 = 0x68,
    //! Sync-in source is FSI-RX2 RX Trigger 1 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX2_RX_TRIG1 = 0x69,
    //! Sync-in source is FSI-RX2 RX Trigger 2 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX2_RX_TRIG2 = 0x6A,
    //! Sync-in source is FSI-RX2 RX Trigger 3 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX2_RX_TRIG3 = 0x6B,
    //! Sync-in source is FSI-RX3 RX Trigger 0 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX3_RX_TRIG0 = 0x6C,
    //! Sync-in source is FSI-RX3 RX Trigger 1 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX3_RX_TRIG1 = 0x6D,
    //! Sync-in source is FSI-RX3 RX Trigger 2 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX3_RX_TRIG2 = 0x6E,
    //! Sync-in source is FSI-RX3 RX Trigger 3 signal
    EPWM_SYNC_IN_PULSE_SRC_FSIRX3_RX_TRIG3 = 0x6F
} EPWM_SyncInPulseSource;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setOneShotSyncOutTrigger() as the
//! \e trigger parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_OSHT_SYNC_OUT_TRIG_SYNC = 0x0, //!< Trigger is OSHT sync
    EPWM_OSHT_SYNC_OUT_TRIG_RELOAD = 0x1  //!< Trigger is OSHT reload
} EPWM_OneShotSyncOutTrigger;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setPeriodLoadMode()  as the
//! \e loadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! PWM Period register access is through shadow register
    EPWM_PERIOD_SHADOW_LOAD = 0,
    //! PWM Period register access is directly
    EPWM_PERIOD_DIRECT_LOAD = 1
} EPWM_PeriodLoadMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTimeBaseCounterMode() as the
//! \e counterMode parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_COUNTER_MODE_UP = 0,         //!< Up - count mode.
    EPWM_COUNTER_MODE_DOWN = 1,       //!< Down - count mode.
    EPWM_COUNTER_MODE_UP_DOWN = 2,    //!< Up - down - count mode.
    EPWM_COUNTER_MODE_STOP_FREEZE = 3 //!< Stop - Freeze counter.
} EPWM_TimeBaseCountMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_selectPeriodLoadEvent() as the
//! \e shadowLoadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! shadow to active load occurs when time base counter reaches 0.
    EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO = 0,
    //! shadow to active load occurs when time base counter reaches 0 and a
    //! SYNC occurs
    EPWM_SHADOW_LOAD_MODE_COUNTER_SYNC = 1,
    //! shadow to active load occurs only when a SYNC occurs
    EPWM_SHADOW_LOAD_MODE_SYNC = 2
} EPWM_PeriodShadowLoadMode;

//*****************************************************************************
//
// Values that can be returned by the EPWM_getTimeBaseCounterDirection()
//
//*****************************************************************************
//! Time base counter is counting down
#define EPWM_TIME_BASE_STATUS_COUNT_DOWN   (0U)
//! Time base counter is counting up
#define EPWM_TIME_BASE_STATUS_COUNT_UP     (1U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_setupEPWMLinks() as the \e epwmLink
//! parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_LINK_WITH_EPWM_0 = 0,   //!< link current ePWM with ePWM0
    EPWM_LINK_WITH_EPWM_1 = 1,   //!< link current ePWM with ePWM1
    EPWM_LINK_WITH_EPWM_2 = 2,   //!< link current ePWM with ePWM2
    EPWM_LINK_WITH_EPWM_3 = 3,   //!< link current ePWM with ePWM3
    EPWM_LINK_WITH_EPWM_4 = 4,   //!< link current ePWM with ePWM4
    EPWM_LINK_WITH_EPWM_5 = 5,   //!< link current ePWM with ePWM5
    EPWM_LINK_WITH_EPWM_6 = 6,   //!< link current ePWM with ePWM6
    EPWM_LINK_WITH_EPWM_7 = 7,   //!< link current ePWM with ePWM7
    EPWM_LINK_WITH_EPWM_8 = 8,   //!< link current ePWM with ePWM8
    EPWM_LINK_WITH_EPWM_9 = 9,   //!< link current ePWM with ePWM9
    EPWM_LINK_WITH_EPWM_10 = 10,   //!< link current ePWM with ePWM10
    EPWM_LINK_WITH_EPWM_11 = 11,   //!< link current ePWM with ePWM11
    EPWM_LINK_WITH_EPWM_12 = 12,   //!< link current ePWM with ePWM12
    EPWM_LINK_WITH_EPWM_13 = 13,   //!< link current ePWM with ePWM13
    EPWM_LINK_WITH_EPWM_14 = 14,   //!< link current ePWM with ePWM14
    EPWM_LINK_WITH_EPWM_15 = 15,   //!< link current ePWM with ePWM15
    EPWM_LINK_WITH_EPWM_16 = 16,   //!< link current ePWM with ePWM16
    EPWM_LINK_WITH_EPWM_17 = 17,   //!< link current ePWM with ePWM17
    EPWM_LINK_WITH_EPWM_18 = 18,   //!< link current ePWM with ePWM18
    EPWM_LINK_WITH_EPWM_19 = 19,   //!< link current ePWM with ePWM19
    EPWM_LINK_WITH_EPWM_20 = 20,   //!< link current ePWM with ePWM20
    EPWM_LINK_WITH_EPWM_21 = 21,   //!< link current ePWM with ePWM21
    EPWM_LINK_WITH_EPWM_22 = 22,   //!< link current ePWM with ePWM22
    EPWM_LINK_WITH_EPWM_23 = 23,   //!< link current ePWM with ePWM23
    EPWM_LINK_WITH_EPWM_24 = 24,   //!< link current ePWM with ePWM24
    EPWM_LINK_WITH_EPWM_25 = 25,   //!< link current ePWM with ePWM25
    EPWM_LINK_WITH_EPWM_26 = 26,   //!< link current ePWM with ePWM26
    EPWM_LINK_WITH_EPWM_27 = 27,   //!< link current ePWM with ePWM27
    EPWM_LINK_WITH_EPWM_28 = 28,   //!< link current ePWM with ePWM28
    EPWM_LINK_WITH_EPWM_29 = 29,   //!< link current ePWM with ePWM29
    EPWM_LINK_WITH_EPWM_30 = 30,   //!< link current ePWM with ePWM30
    EPWM_LINK_WITH_EPWM_31 = 31,   //!< link current ePWM with ePWM31
} EPWM_CurrentLink;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setupEPWMLinks() as the \e linkComp
//! parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_LINK_TBPRD = 0,  //!< link TBPRD:TBPRDHR registers
    EPWM_LINK_COMP_A = 5,  //!< link COMPA registers
    EPWM_LINK_COMP_B = 10,  //!< link COMPB registers
    EPWM_LINK_COMP_C = 16, //!< link COMPC registers
    EPWM_LINK_COMP_D = 21, //!< link COMPD registers
    EPWM_LINK_GLDCTL2 = 26,  //!< link GLDCTL2 registers
    EPWM_LINK_DBRED = 1,    //!< link DBRED registers
    EPWM_LINK_DBFED = 6,    //!< link DBFED registers
    EPWM_LINK_XLOAD = 2      //!< link XLOAD registers
} EPWM_LinkComponent;

//
// Counter Compare Module
//
//*****************************************************************************
//
//! Values that can be passed to the EPWM_getCounterCompareShadowStatus(),
//! EPWM_setCounterCompareValue(), EPWM_setCounterCompareShadowLoadMode(),
//! EPWM_disableCounterCompareShadowLoadMode(), EPWM_getCounterCompareValue()
//! as the \e compModule parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_COUNTER_COMPARE_A = 0, //!< counter compare A
    EPWM_COUNTER_COMPARE_B = 4, //!< counter compare B
    EPWM_COUNTER_COMPARE_C = 10, //!< counter compare C
    EPWM_COUNTER_COMPARE_D = 14  //!< counter compare D
} EPWM_CounterCompareModule;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setCounterCompareShadowLoadMode() as the
//! \e loadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! load when counter equals zero
    EPWM_COMP_LOAD_ON_CNTR_ZERO = 0,
    //! load when counter equals period
    EPWM_COMP_LOAD_ON_CNTR_PERIOD = 1,
    //! load when counter equals zero or period
    EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! Freeze shadow to active load
    EPWM_COMP_LOAD_FREEZE = 3,
    //! load when counter equals zero
    EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO = 4,
    //! load when counter equals period
    EPWM_COMP_LOAD_ON_SYNC_CNTR_PERIOD = 5,
    //! load when counter equals zero or period
    EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO_PERIOD = 6,
    //! load on sync only
    EPWM_COMP_LOAD_ON_SYNC_ONLY = 8
} EPWM_CounterCompareLoadMode;

//
// Action Qualifier Module
//
//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierShadowLoadMode() and
//! EPWM_disableActionQualifierShadowLoadMode() as the \e aqModule parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_ACTION_QUALIFIER_A = 0, //!< Action Qualifier A
    EPWM_ACTION_QUALIFIER_B = 2  //!< Action Qualifier B
} EPWM_ActionQualifierModule;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierShadowLoadMode() as the
//! \e loadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! load when counter equals zero
    EPWM_AQ_LOAD_ON_CNTR_ZERO = 0,
    //! load when counter equals period
    EPWM_AQ_LOAD_ON_CNTR_PERIOD = 1,
    //! load when counter equals zero or period
    EPWM_AQ_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! Freeze shadow to active load
    EPWM_AQ_LOAD_FREEZE = 3,
    //! load on sync or when counter equals zero
    EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO = 4,
    //! load on sync or when counter equals period
    EPWM_AQ_LOAD_ON_SYNC_CNTR_PERIOD = 5,
    //! load on sync or when counter equals zero or period
    EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO_PERIOD = 6,
    //! load on sync only
    EPWM_AQ_LOAD_ON_SYNC_ONLY = 8
} EPWM_ActionQualifierLoadMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierT1TriggerSource() and
//! EPWM_setActionQualifierT2TriggerSource() as the \e trigger parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1 = 0, //!< Digital compare event A 1
    EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_2 = 1, //!< Digital compare event A 2
    EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_1 = 2, //!< Digital compare event B 1
    EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_2 = 3, //!< Digital compare event B 2
    EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_1 = 4, //!< Trip zone 1
    EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_2 = 5, //!< Trip zone 2
    EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_3 = 6, //!< Trip zone 3
    EPWM_AQ_TRIGGER_EVENT_TRIG_EPWM_SYNCIN = 7, //!< ePWM sync
    EPWM_AQ_TRIGGER_EVENT_TRIG_DC_EVTFILT = 8 //!< Digital compare filter event
} EPWM_ActionQualifierTriggerSource;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierAction() as the \e
//! event parameter.
//
//*****************************************************************************
typedef enum
{
    //! Time base counter equals zero
    EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO = 0,
    //! Time base counter equals period
    EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD = 2,
    //! Time base counter up equals COMPA
    EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA = 4,
    //! Time base counter down equals COMPA
    EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA = 6,
    //! Time base counter up equals COMPB
    EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB = 8,
    //! Time base counter down equals COMPB
    EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB = 10,
    //! T1 event on count up
    EPWM_AQ_OUTPUT_ON_T1_COUNT_UP = 1,
    //! T1 event on count down
    EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN = 3,
    //! T2 event on count up
    EPWM_AQ_OUTPUT_ON_T2_COUNT_UP = 5,
    //! T2 event on count down
    EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN = 7
} EPWM_ActionQualifierOutputEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierSWAction(),
//! EPWM_setActionQualifierAction() as the \e outPut parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_AQ_OUTPUT_NO_CHANGE = 0,  //!< No change in the output pins
    EPWM_AQ_OUTPUT_LOW = 1,  //!< Set output pins to low
    EPWM_AQ_OUTPUT_HIGH = 2,  //!< Set output pins to High
    EPWM_AQ_OUTPUT_TOGGLE = 3   //!< Toggle the output pins
} EPWM_ActionQualifierOutput;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierContSWForceAction()
//! as the \e outPut parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_AQ_SW_DISABLED = 0,  //!< Software forcing disabled
    EPWM_AQ_SW_OUTPUT_LOW = 1,  //!< Set output pins to low
    EPWM_AQ_SW_OUTPUT_HIGH = 2  //!< Set output pins to High
} EPWM_ActionQualifierSWOutput;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierActionComplete()
//! as the \e action parameter.
//
//*****************************************************************************
typedef enum
{
    //! Time base counter equals zero and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_ZERO = 0x0,
    //! Time base counter equals zero and set output pins to low
    EPWM_AQ_OUTPUT_LOW_ZERO = 0x1,
    //! Time base counter equals zero and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_ZERO = 0x2,
    //! Time base counter equals zero and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_ZERO = 0x3,
    //! Time base counter equals period and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_PERIOD = 0x0,
    //! Time base counter equals period and set output pins to low
    EPWM_AQ_OUTPUT_LOW_PERIOD = 0x4,
    //! Time base counter equals period and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_PERIOD = 0x8,
    //! Time base counter equals period and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_PERIOD = 0xC,
    //! Time base counter up equals COMPA and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_UP_CMPA = 0x00,
    //! Time base counter up equals COMPA and set output pins to low
    EPWM_AQ_OUTPUT_LOW_UP_CMPA = 0x10,
    //! Time base counter up equals COMPA and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_UP_CMPA = 0x20,
    //! Time base counter up equals COMPA and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_UP_CMPA = 0x30,
    //! Time base counter down equals COMPA and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_CMPA = 0x00,
    //! Time base counter down equals COMPA and set output pins to low
    EPWM_AQ_OUTPUT_LOW_DOWN_CMPA = 0x40,
    //! Time base counter down equals COMPA and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_DOWN_CMPA = 0x80,
    //! Time base counter down equals COMPA and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_DOWN_CMPA = 0xC0,
    //! Time base counter up equals COMPB and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_UP_CMPB = 0x000,
    //! Time base counter up equals COMPB and set output pins to low
    EPWM_AQ_OUTPUT_LOW_UP_CMPB = 0x100,
    //! Time base counter up equals COMPB and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_UP_CMPB = 0x200,
    //! Time base counter up equals COMPB and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_UP_CMPB = 0x300,
    //! Time base counter down equals COMPB and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_CMPB = 0x000,
    //! Time base counter down equals COMPB and set output pins to low
    EPWM_AQ_OUTPUT_LOW_DOWN_CMPB = 0x400,
    //! Time base counter down equals COMPB and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_DOWN_CMPB = 0x800,
    //! Time base counter down equals COMPB and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_DOWN_CMPB = 0xC00
} EPWM_ActionQualifierEventAction;

//*****************************************************************************
//
//! Values that can be passed to
//! EPWM_setAdditionalActionQualifierActionComplete()  as the \e action
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! T1 event on count up and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_UP_T1 = 0x0,
    //! T1 event on count up and set output pins to low
    EPWM_AQ_OUTPUT_LOW_UP_T1 = 0x1,
    //! T1 event on count up and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_UP_T1 = 0x2,
    //! T1 event on count up and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_UP_T1 = 0x3,
    //! T1 event on count down and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_T1 = 0x0,
    //! T1 event on count down and set output pins to low
    EPWM_AQ_OUTPUT_LOW_DOWN_T1 = 0x4,
    //! T1 event on count down and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_DOWN_T1 = 0x8,
    //! T1 event on count down and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_DOWN_T1 = 0xC,
    //! T2 event on count up and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_UP_T2 = 0x00,
    //! T2 event on count up and set output pins to low
    EPWM_AQ_OUTPUT_LOW_UP_T2 = 0x10,
    //! T2 event on count up and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_UP_T2 = 0x20,
    //! T2 event on count up and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_UP_T2 = 0x30,
    //! T2 event on count down and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_T2 = 0x00,
    //! T2 event on count down and set output pins to low
    EPWM_AQ_OUTPUT_LOW_DOWN_T2 = 0x40,
    //! T2 event on count down and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_DOWN_T2 = 0x80,
    //! T2 event on count down and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_DOWN_T2 = 0xC0
} EPWM_AdditionalActionQualifierEventAction;

//*****************************************************************************
//
//! Values that can be passed to EPWM_forceActionQualifierSWAction(),
//! EPWM_setActionQualifierSWAction(), EPWM_setActionQualifierAction(),
//! EPWM_setActionQualifierActionComplete(),
//! EPWM_setAdditionalActionQualifierActionComplete()
//! EPWM_setActionQualifierContSWForceAction() as the \e epwmOutput parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_AQ_OUTPUT_A = 0, //!< ePWMxA output
    EPWM_AQ_OUTPUT_B = 4  //!< ePWMxB output
} EPWM_ActionQualifierOutputModule;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierContSWForceShadowMode()
//! as the \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    //! shadow mode load when counter equals zero
    EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO = 0,
    //! shadow mode load when counter equals period
    EPWM_AQ_SW_SH_LOAD_ON_CNTR_PERIOD = 1,
    //! shadow mode load when counter equals zero or period
    EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! No shadow load mode. Immediate mode only.
    EPWM_AQ_SW_IMMEDIATE_LOAD = 3
} EPWM_ActionQualifierContForce;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDeadBandOutputSwapMode()
//! as the \e output parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DB_OUTPUT_A = 1, //!< DB output is ePWMA
    EPWM_DB_OUTPUT_B = 0  //!< DB output is ePWMB
} EPWM_DeadBandOutput;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDeadBandDelayPolarity(),
//! EPWM_setDeadBandDelayMode() as the \e delayMode parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DB_RED = 1, //!< DB RED (Rising Edge Delay) mode
    EPWM_DB_FED = 0  //!< DB FED (Falling Edge Delay) mode
} EPWM_DeadBandDelayMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDeadBandDelayPolarity() as the
//! \e polarity parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DB_POLARITY_ACTIVE_HIGH = 0, //!< DB polarity is not inverted
    EPWM_DB_POLARITY_ACTIVE_LOW  = 1  //!< DB polarity is inverted
} EPWM_DeadBandPolarity;

//*****************************************************************************
//
// Values that can be passed to EPWM_setRisingEdgeDeadBandDelayInput(),
// EPWM_setFallingEdgeDeadBandDelayInput() as the input parameter.
//
//*****************************************************************************
//! Input signal is ePWMA
#define EPWM_DB_INPUT_EPWMA       (0U)
//! Input signal is ePWMB
#define EPWM_DB_INPUT_EPWMB       (1U)
//! Input signal is the output of Rising Edge delay
#define EPWM_DB_INPUT_DB_RED      (2U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDeadBandControlShadowLoadMode() as
//! the \e loadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! load when counter equals zero
    EPWM_DB_LOAD_ON_CNTR_ZERO = 0,
    //! load when counter equals period
    EPWM_DB_LOAD_ON_CNTR_PERIOD = 1,
    //! load when counter equals zero or period
    EPWM_DB_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! Freeze shadow to active load
    EPWM_DB_LOAD_FREEZE = 3
} EPWM_DeadBandControlLoadMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setRisingEdgeDelayCountShadowLoadMode()
//! as the \e loadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! load when counter equals zero
    EPWM_RED_LOAD_ON_CNTR_ZERO = 0,
    //! load when counter equals period
    EPWM_RED_LOAD_ON_CNTR_PERIOD = 1,
    //! load when counter equals zero or period
    EPWM_RED_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! Freeze shadow to active load
    EPWM_RED_LOAD_FREEZE = 3
} EPWM_RisingEdgeDelayLoadMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setFallingEdgeDelayCountShadowLoadMode()
//! as the \e loadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! load when counter equals zero
    EPWM_FED_LOAD_ON_CNTR_ZERO = 0,
    //! load when counter equals period
    EPWM_FED_LOAD_ON_CNTR_PERIOD = 1,
    //! load when counter equals zero or period
    EPWM_FED_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! Freeze shadow to active load
    EPWM_FED_LOAD_FREEZE = 3
} EPWM_FallingEdgeDelayLoadMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDeadBandCounterClock() as the
//! \e clockMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Dead band counter runs at TBCLK rate
    EPWM_DB_COUNTER_CLOCK_FULL_CYCLE = 0,
    //! Dead band counter runs at 2*TBCLK rate
    EPWM_DB_COUNTER_CLOCK_HALF_CYCLE = 1
} EPWM_DeadBandClockMode;

//
// Trip Zone
//
//*****************************************************************************
//
// Values that can be passed to EPWM_enableTripZoneSignals() and
// EPWM_disableTripZoneSignals() as the tzSignal parameter.
//
//*****************************************************************************
//! TZ1 Cycle By Cycle
#define EPWM_TZ_SIGNAL_CBC1          (0x1U)
//! TZ2 Cycle By Cycle
#define EPWM_TZ_SIGNAL_CBC2          (0x2U)
//! TZ3 Cycle By Cycle
#define EPWM_TZ_SIGNAL_CBC3          (0x4U)
//! TZ4 Cycle By Cycle
#define EPWM_TZ_SIGNAL_CBC4          (0x8U)
//! TZ5 Cycle By Cycle
#define EPWM_TZ_SIGNAL_CBC5          (0x10U)
//! TZ6 Cycle By Cycle
#define EPWM_TZ_SIGNAL_CBC6          (0x20U)
//! DCAEVT2 Cycle By Cycle
#define EPWM_TZ_SIGNAL_DCAEVT2       (0x40U)
//! DCBEVT2 Cycle By Cycle
#define EPWM_TZ_SIGNAL_DCBEVT2       (0x80U)
//! One-shot TZ1
#define EPWM_TZ_SIGNAL_OSHT1         (0x100U)
//! One-shot TZ2
#define EPWM_TZ_SIGNAL_OSHT2         (0x200U)
//! One-shot TZ3
#define EPWM_TZ_SIGNAL_OSHT3         (0x400U)
//! One-shot TZ4
#define EPWM_TZ_SIGNAL_OSHT4         (0x800U)
//! One-shot TZ5
#define EPWM_TZ_SIGNAL_OSHT5         (0x1000U)
//! One-shot TZ6
#define EPWM_TZ_SIGNAL_OSHT6         (0x2000U)
//! One-shot DCAEVT1
#define EPWM_TZ_SIGNAL_DCAEVT1       (0x4000U)
//! One-shot DCBEVT1
#define EPWM_TZ_SIGNAL_DCBEVT1       (0x8000U)
// //! One-shot Capture event
// #define EPWM_TZ_SIGNAL_CAPEVT_OST    (0x10000U)
// //! Cycle by cycle capture event
// #define EPWM_TZ_SIGNAL_CAPEVT_CBC    (0x1000000U)

//*****************************************************************************
//
// Values that can be passed to EPWM_enableTripZone2Signals() and
// EPWM_disableTripZone2Signals() as the tzSignal parameter.
//
//*****************************************************************************
//! Cycle by cycle capture event
#define EPWM_TZ_SIGNAL_CAPEVT_CBC    (0x1U)
//! One-shot Capture event
#define EPWM_TZ_SIGNAL_CAPEVT_OST    (0x100U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneDigitalCompareEventCondition()
//! as the \e dcType parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_TZ_DC_OUTPUT_A1 = 0, //!< Digital Compare output 1 A
    EPWM_TZ_DC_OUTPUT_A2 = 3, //!< Digital Compare output 2 A
    EPWM_TZ_DC_OUTPUT_B1 = 6, //!< Digital Compare output 1 B
    EPWM_TZ_DC_OUTPUT_B2 = 9  //!< Digital Compare output 2 B
} EPWM_TripZoneDigitalCompareOutput;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneDigitalCompareEventCondition()
//! as the \e dcEvent parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_TZ_EVENT_DC_DISABLED = 0,  //!< Event is disabled
    EPWM_TZ_EVENT_DCXH_LOW = 1,  //!< Event when DCxH low
    EPWM_TZ_EVENT_DCXH_HIGH = 2,  //!< Event when DCxH high
    EPWM_TZ_EVENT_DCXL_LOW = 3,  //!< Event when DCxL low
    EPWM_TZ_EVENT_DCXL_HIGH = 4,  //!< Event when DCxL high
    EPWM_TZ_EVENT_DCXL_HIGH_DCXH_LOW = 5  //!< Event when DCxL high DCxH low
} EPWM_TripZoneDigitalCompareOutputEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneAction() as the \e tzEvent
//! parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_TZ_ACTION_EVENT_TZA = 0,     //!< TZ1 - TZ6, DCAEVT2, DCAEVT1
    EPWM_TZ_ACTION_EVENT_TZB = 2,     //!< TZ1 - TZ6, DCBEVT2, DCBEVT1
    EPWM_TZ_ACTION_EVENT_DCAEVT1 = 4, //!< DCAEVT1 (Digital Compare A event 1)
    EPWM_TZ_ACTION_EVENT_DCAEVT2 = 6, //!< DCAEVT2 (Digital Compare A event 2)
    EPWM_TZ_ACTION_EVENT_DCBEVT1 = 8, //!< DCBEVT1 (Digital Compare B event 1)
    EPWM_TZ_ACTION_EVENT_DCBEVT2 = 10 //!< DCBEVT2 (Digital Compare B event 2)
} EPWM_TripZoneEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneAction() as the
//! \e tzAction parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_TZ_ACTION_HIGH_Z = 0,  //!< high impedance output
    EPWM_TZ_ACTION_HIGH = 1,  //!< high voltage state
    EPWM_TZ_ACTION_LOW = 2,  //!< low voltage state
    EPWM_TZ_ACTION_DISABLE = 3  //!< disable action
} EPWM_TripZoneAction;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneAdvAction() as the
//! \e tzAdvEvent parameter.
//
//*****************************************************************************
typedef enum
{
    //! TZ1 - TZ6, DCBEVT2, DCBEVT1 while counting down
    EPWM_TZ_ADV_ACTION_EVENT_TZB_D = 9,
    //! TZ1 - TZ6, DCBEVT2, DCBEVT1 while counting up
    EPWM_TZ_ADV_ACTION_EVENT_TZB_U = 6,
    //! TZ1 - TZ6, DCAEVT2, DCAEVT1 while counting down
    EPWM_TZ_ADV_ACTION_EVENT_TZA_D = 3,
    //! TZ1 - TZ6, DCAEVT2, DCAEVT1 while counting up
    EPWM_TZ_ADV_ACTION_EVENT_TZA_U = 0
} EPWM_TripZoneAdvancedEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneAdvDigitalCompareActionA(),
//! EPWM_setTripZoneAdvDigitalCompareActionB(), EPWM_setTripZoneAdvAction()
//! as the \e tzAdvDCAction parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_TZ_ADV_ACTION_HIGH_Z = 0,  //!< high impedance output
    EPWM_TZ_ADV_ACTION_HIGH = 1,  //!< high voltage state
    EPWM_TZ_ADV_ACTION_LOW = 2,  //!< low voltage state
    EPWM_TZ_ADV_ACTION_TOGGLE = 3,  //!< toggle the output
    EPWM_TZ_ADV_ACTION_DISABLE = 7  //!< disable action
} EPWM_TripZoneAdvancedAction;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneAdvDigitalCompareActionA() and
//! EPWM_setTripZoneAdvDigitalCompareActionB() as the \e tzAdvDCEvent
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! Digital Compare event A/B 1 while counting up
    EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U = 0,
    //! Digital Compare event A/B 1 while counting down
    EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D = 3,
    //! Digital Compare event A/B 2 while counting up
    EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U = 6,
    //! Digital Compare event A/B 2 while counting down
    EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D = 9
} EPWM_TripZoneAdvDigitalCompareEvent;

//*****************************************************************************
//
// Values that can be passed to EPWM_enableTripZoneInterrupt()and
// EPWM_disableTripZoneInterrupt() as the tzInterrupt parameter .
//
//*****************************************************************************
//! Trip Zones Cycle By Cycle interrupt
#define EPWM_TZ_INTERRUPT_CBC      (0x2U)
//! Trip Zones One Shot interrupt
#define EPWM_TZ_INTERRUPT_OST      (0x4U)
//! Digital Compare A Event 1 interrupt
#define EPWM_TZ_INTERRUPT_DCAEVT1  (0x8U)
//! Digital Compare A Event 2 interrupt
#define EPWM_TZ_INTERRUPT_DCAEVT2  (0x10U)
//! Digital Compare B Event 1 interrupt
#define EPWM_TZ_INTERRUPT_DCBEVT1  (0x20U)
//! Digital Compare B Event 2 interrupt
#define EPWM_TZ_INTERRUPT_DCBEVT2  (0x40U)
//! Capture Event interrupt
#define EPWM_TZ_INTERRUPT_CAPEVT   (0x80U)

//*****************************************************************************
//
// Values that can be returned by EPWM_getTripZoneFlagStatus() .
//
//*****************************************************************************
//! Trip Zones Cycle By Cycle flag
#define EPWM_TZ_FLAG_CBC      (0x2U)
//! Trip Zones One Shot flag
#define EPWM_TZ_FLAG_OST      (0x4U)
//! Digital Compare A Event 1 flag
#define EPWM_TZ_FLAG_DCAEVT1  (0x8U)
//! Digital Compare A Event 2 flag
#define EPWM_TZ_FLAG_DCAEVT2  (0x10U)
//! Digital Compare B Event 1 flag
#define EPWM_TZ_FLAG_DCBEVT1  (0x20U)
//! Digital Compare B Event 2 flag
#define EPWM_TZ_FLAG_DCBEVT2  (0x40U)
//! Capture Event flag
#define EPWM_TZ_FLAG_CAPEVT   (0x80U)

//*****************************************************************************
//
// Value can be passed to EPWM_clearTripZoneFlag() as the
// tzInterrupt parameter and returned by EPWM_getTripZoneFlagStatus().
//
//*****************************************************************************
//! Trip Zone interrupt
#define EPWM_TZ_INTERRUPT     (0x1U)

//*****************************************************************************
//
// Values that can be passed to EPWM_clearCycleByCycleTripZoneFlag()
// as the tzCbcFlag parameter and returned by
// EPWM_getCycleByCycleTripZoneFlagStatus().
//
//*****************************************************************************
//! CBC flag 1
#define EPWM_TZ_CBC_FLAG_1        (0x1U)
//! CBC flag 2
#define EPWM_TZ_CBC_FLAG_2        (0x2U)
//! CBC flag 3
#define EPWM_TZ_CBC_FLAG_3        (0x4U)
//! CBC flag 4
#define EPWM_TZ_CBC_FLAG_4        (0x8U)
//! CBC flag 5
#define EPWM_TZ_CBC_FLAG_5        (0x10U)
//! CBC flag 6
#define EPWM_TZ_CBC_FLAG_6        (0x20U)
//! CBC flag Digital compare event A2
#define EPWM_TZ_CBC_FLAG_DCAEVT2  (0x40U)
//! CBC flag Digital compare event B2
#define EPWM_TZ_CBC_FLAG_DCBEVT2  (0x80U)
//! CBC flag Capture event
#define EPWM_TZ_CBC_FLAG_CAPEVT   (0x100U)

//*****************************************************************************
//
// Values that can be passed to EPWM_clearOneShotTripZoneFlag() as
// the tzCbcFlag parameter and returned by the
// EPWM_getOneShotTripZoneFlagStatus() .
//
//*****************************************************************************
//! OST flag OST1
#define EPWM_TZ_OST_FLAG_OST1      (0x1U)
//! OST flag OST2
#define EPWM_TZ_OST_FLAG_OST2      (0x2U)
//! OST flag OST3
#define EPWM_TZ_OST_FLAG_OST3      (0x4U)
//! OST flag OST4
#define EPWM_TZ_OST_FLAG_OST4      (0x8U)
//! OST flag OST5
#define EPWM_TZ_OST_FLAG_OST5      (0x10U)
//! OST flag OST6
#define EPWM_TZ_OST_FLAG_OST6      (0x20U)
//! OST flag Digital compare event A1
#define EPWM_TZ_OST_FLAG_DCAEVT1   (0x40U)
//! OST flag Digital compare event B1
#define EPWM_TZ_OST_FLAG_DCBEVT1   (0x80U)
//! OST flag Capture Event
#define EPWM_TZ_OST_FLAG_CAPEVT    (0x100U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_selectCycleByCycleTripZoneClearEvent() as
//! the \e clearMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Clear CBC pulse when counter equals zero
    EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO = 0,
    //! Clear CBC pulse when counter equals period
    EPWM_TZ_CBC_PULSE_CLR_CNTR_PERIOD = 1,
    //! Clear CBC pulse when counter equals zero or period
    EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO_PERIOD = 2
} EPWM_CycleByCycleTripZoneClearMode;

//*****************************************************************************
//
// Values that can be passed to EPWM_forceTripZoneEvent() as the
// tzForceEvent parameter.
//
//*****************************************************************************
//! Force Cycle By Cycle trip event
#define EPWM_TZ_FORCE_EVENT_CBC      (0x2U)
//! Force a One-Shot Trip Event
#define EPWM_TZ_FORCE_EVENT_OST      (0x4U)
//! Force Digital Compare Output A Event 1
#define EPWM_TZ_FORCE_EVENT_DCAEVT1  (0x8U)
//! Force Digital Compare Output A Event 2
#define EPWM_TZ_FORCE_EVENT_DCAEVT2  (0x10U)
//! Force Digital Compare Output B Event 1
#define EPWM_TZ_FORCE_EVENT_DCBEVT1  (0x20U)
//! Force Digital Compare Output B Event 2
#define EPWM_TZ_FORCE_EVENT_DCBEVT2  (0x40U)
//! Force Capture event
#define EPWM_TZ_FORCE_EVENT_CAPEVT   (0x80U)

//*****************************************************************************
//
// Values that can be passed to EPWM_enableTripZoneOutput() and
// EPWM_disableTripZoneOutput as the tzOutput parameter.
//
//*****************************************************************************
//! One shot
#define EPWM_TZ_SELECT_TRIPOUT_OST        (0x1)
//! Cycle by cycle
#define EPWM_TZ_SELECT_TRIPOUT_CBC        (0x2)
//! Trip Zone 1
#define EPWM_TZ_SELECT_TRIPOUT_TZ1        (0x4)
//! Trip Zone 2
#define EPWM_TZ_SELECT_TRIPOUT_TZ2        (0x8)
//! Trip Zone 3
#define EPWM_TZ_SELECT_TRIPOUT_TZ3        (0x10)
//! Trip Zone 4
#define EPWM_TZ_SELECT_TRIPOUT_TZ4        (0x20)
//! Trip Zone 5
#define EPWM_TZ_SELECT_TRIPOUT_TZ5        (0x40)
//! Trip Zone 6
#define EPWM_TZ_SELECT_TRIPOUT_TZ6        (0x80)
//! Digital capture A Event 1
#define EPWM_TZ_SELECT_TRIPOUT_DCAEVT1    (0x100)
//! Digital capture A Event 2
#define EPWM_TZ_SELECT_TRIPOUT_DCAEVT2    (0x200)
//! Digital capture B Event 1
#define EPWM_TZ_SELECT_TRIPOUT_DCBEVT1    (0x400)
//! Digtial capture B Event 2
#define EPWM_TZ_SELECT_TRIPOUT_DCBEVT2    (0x800)
//! Capture Event
#define EPWM_TZ_SELECT_TRIPOUT_CAPEVT     (0x1000)

//*****************************************************************************
//
// Values that can be passed to EPWM_setInterruptSource() as the
// interruptSource parameter.
//
//*****************************************************************************
//! Time-base counter equal to zero
#define EPWM_INT_TBCTR_ZERO            (1U)
//! Time-base counter equal to period
#define EPWM_INT_TBCTR_PERIOD          (2U)
//! Time-base counter based on mix events
#define EPWM_INT_TBCTR_ETINTMIX        (3U)
//! time-base counter equal to CMPA when the timer is incrementing
#define EPWM_INT_TBCTR_U_CMPA          (4U)
//! time-base counter equal to CMPC when the timer is incrementing
#define EPWM_INT_TBCTR_U_CMPC          (8U)
//! time-base counter equal to CMPA when the timer is decrementing
#define EPWM_INT_TBCTR_D_CMPA          (5U)
//! time-base counter equal to CMPC when the timer is decrementing
#define EPWM_INT_TBCTR_D_CMPC          (10U)
//! time-base counter equal to CMPB when the timer is incrementing
#define EPWM_INT_TBCTR_U_CMPB          (6U)
//! time-base counter equal to CMPD when the timer is incrementing
#define EPWM_INT_TBCTR_U_CMPD          (12U)
//! time-base counter equal to CMPB when the timer is decrementing
#define EPWM_INT_TBCTR_D_CMPB          (7U)
//! time-base counter equal to CMPD when the timer is decrementing
#define EPWM_INT_TBCTR_D_CMPD          (14U)

//*****************************************************************************
//
// Values that can be passed to EPWM_setInterruptSource() and
// EPWM_setADCTriggerSource() as the mixedSource parameter.
//
//*****************************************************************************
//! Time-base counter equal to zero
#define EPWM_INT_MIX_TBCTR_ZERO            (0x1)
//! Time-base counter equal to period
#define EPWM_INT_MIX_TBCTR_PERIOD          (0x2)
//! time-base counter equal to CMPA when the timer is incrementing
#define EPWM_INT_MIX_TBCTR_U_CMPA          (0x4)
//! time-base counter equal to CMPA when the timer is decrementing
#define EPWM_INT_MIX_TBCTR_D_CMPA          (0x8)
//! time-base counter equal to CMPB when the timer is incrementing
#define EPWM_INT_MIX_TBCTR_U_CMPB          (0x10)
//! time-base counter equal to CMPB when the timer is decrementing
#define EPWM_INT_MIX_TBCTR_D_CMPB          (0x20)
//! time-base counter equal to CMPC when the timer is incrementing
#define EPWM_INT_MIX_TBCTR_U_CMPC          (0x40)
//! time-base counter equal to CMPC when the timer is decrementing
#define EPWM_INT_MIX_TBCTR_D_CMPC          (0x80)
//! time-base counter equal to CMPD when the timer is incrementing
#define EPWM_INT_MIX_TBCTR_U_CMPD          (0x100)
//! time-base counter equal to CMPD when the timer is decrementing
#define EPWM_INT_MIX_TBCTR_D_CMPD          (0x200)
//! DCAEVT1 interrupt
#define EPWM_INT_MIX_DCAEVT1               (0x400)


//*****************************************************************************
//
//! Values that can be passed to EPWM_enableADCTrigger(),
//! EPWM_disableADCTrigger(),EPWM_setADCTriggerSource(),
//! EPWM_setADCTriggerEventPrescale(),EPWM_getADCTriggerFlagStatus(),
//! EPWM_clearADCTriggerFlag(),EPWM_enableADCTriggerEventCountInit(),
//! EPWM_disableADCTriggerEventCountInit(),EPWM_forceADCTriggerEventCountInit()
//! EPWM_setADCTriggerEventCountInitValue(),EPWM_getADCTriggerEventCount(),
//! EPWM_forceADCTrigger() as the \e adcSOCType parameter
//
//*****************************************************************************
typedef enum
{
    EPWM_SOC_A = 0,  //!< SOC A
    EPWM_SOC_B = 1   //!< SOC B
} EPWM_ADCStartOfConversionType;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setADCTriggerSource() as the
//! \e socSource parameter.
//
//*****************************************************************************
typedef enum
{
    //! Event is based on DCxEVT1
    EPWM_SOC_DCxEVT1 = 0,
    //! Time-base counter equal to zero
    EPWM_SOC_TBCTR_ZERO = 1,
    //! Time-base counter equal to period
    EPWM_SOC_TBCTR_PERIOD = 2,
    //! Time-base counter equal to zero or period
    EPWM_SOC_TBCTR_MIXED_EVENT = 3,
    //! time-base counter equal to CMPA when the timer is incrementing
    EPWM_SOC_TBCTR_U_CMPA = 4,
    //! time-base counter equal to CMPC when the timer is incrementing
    EPWM_SOC_TBCTR_U_CMPC = 8,
    //! time-base counter equal to CMPA when the timer is decrementing
    EPWM_SOC_TBCTR_D_CMPA = 5,
    //! time-base counter equal to CMPC when the timer is decrementing
    EPWM_SOC_TBCTR_D_CMPC = 10,
    //! time-base counter equal to CMPB when the timer is incrementing
    EPWM_SOC_TBCTR_U_CMPB = 6,
    //! time-base counter equal to CMPD when the timer is incrementing
    EPWM_SOC_TBCTR_U_CMPD = 12,
    //! time-base counter equal to CMPB when the timer is decrementing
    EPWM_SOC_TBCTR_D_CMPB = 7,
    //! time-base counter equal to CMPD when the timer is decrementing
    EPWM_SOC_TBCTR_D_CMPD = 14
} EPWM_ADCStartOfConversionSource;

//
// Digital Compare Module
//
//*****************************************************************************
//
//! Values that can be passed to EPWM_selectDigitalCompareTripInput(),
//! EPWM_enableDigitalCompareTripCombinationInput(),
//! EPWM_disableDigitalCompareTripCombinationInput() as the \e dcType
//! parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DC_TYPE_DCAH = 0,  //!< Digital Compare A High
    EPWM_DC_TYPE_DCAL = 1,  //!< Digital Compare A Low
    EPWM_DC_TYPE_DCBH = 2,  //!< Digital Compare B High
    EPWM_DC_TYPE_DCBL = 3   //!< Digital Compare B Low
} EPWM_DigitalCompareType;

//*****************************************************************************
//
//! Values that can be passed to EPWM_selectDigitalCompareTripInput() and
//! and EPWM_selectCaptureTripInput()
//! as the \e tripSource parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DC_TRIP_TRIPIN1 = 0,  //!< Trip 1
    EPWM_DC_TRIP_TRIPIN2 = 1,  //!< Trip 2
    EPWM_DC_TRIP_TRIPIN3 = 2,  //!< Trip 3
    EPWM_DC_TRIP_TRIPIN4 = 3,  //!< Trip 4
    EPWM_DC_TRIP_TRIPIN5 = 4,  //!< Trip 5
    EPWM_DC_TRIP_TRIPIN6 = 5,  //!< Trip 6
    EPWM_DC_TRIP_TRIPIN7 = 6,  //!< Trip 7
    EPWM_DC_TRIP_TRIPIN8 = 7,  //!< Trip 8
    EPWM_DC_TRIP_TRIPIN9 = 8,  //!< Trip 9
    EPWM_DC_TRIP_TRIPIN10 = 9,  //!< Trip 10
    EPWM_DC_TRIP_TRIPIN11 = 10,  //!< Trip 11
    EPWM_DC_TRIP_TRIPIN12 = 11,  //!< Trip 12
    EPWM_DC_TRIP_TRIPIN13 = 12,  //!< Trip 13
    EPWM_DC_TRIP_TRIPIN14 = 13,  //!< Trip 14
    EPWM_DC_TRIP_TRIPIN15 = 14,  //!< Trip 15
    EPWM_DC_TRIP_COMBINATION = 15 //!< All Trips (Trip1 - Trip 15) are selected
} EPWM_DigitalCompareTripInput;

//*****************************************************************************
//
// Values that can be passed to EPWM_enableDigitalCompareTripCombinationInput()
// EPWM_enableCaptureTripCombinationInput(),
// EPWM_disableCaptureTripCombinationInput(),
// EPWM_disableDigitalCompareTripCombinationInput() as the tripInput
// parameter.
//
//*****************************************************************************
//! Combinational Trip 1 input
#define EPWM_DC_COMBINATIONAL_TRIPIN1   (0x1U)
//! Combinational Trip 2 input
#define EPWM_DC_COMBINATIONAL_TRIPIN2   (0x2U)
//! Combinational Trip 3 input
#define EPWM_DC_COMBINATIONAL_TRIPIN3   (0x4U)
//! Combinational Trip 4 input
#define EPWM_DC_COMBINATIONAL_TRIPIN4   (0x8U)
//! Combinational Trip 5 input
#define EPWM_DC_COMBINATIONAL_TRIPIN5   (0x10U)
//! Combinational Trip 6 input
#define EPWM_DC_COMBINATIONAL_TRIPIN6   (0x20U)
//! Combinational Trip 7 input
#define EPWM_DC_COMBINATIONAL_TRIPIN7   (0x40U)
//! Combinational Trip 8 input
#define EPWM_DC_COMBINATIONAL_TRIPIN8   (0x80U)
//! Combinational Trip 9 input
#define EPWM_DC_COMBINATIONAL_TRIPIN9   (0x100U)
//! Combinational Trip 10 input
#define EPWM_DC_COMBINATIONAL_TRIPIN10  (0x200U)
//! Combinational Trip 11 input
#define EPWM_DC_COMBINATIONAL_TRIPIN11  (0x400U)
//! Combinational Trip 12 input
#define EPWM_DC_COMBINATIONAL_TRIPIN12  (0x800U)
//! Combinational Trip 13 input
#define EPWM_DC_COMBINATIONAL_TRIPIN13  (0x1000U)
//! Combinational Trip 14 input
#define EPWM_DC_COMBINATIONAL_TRIPIN14  (0x2000U)
//! Combinational Trip 15 input
#define EPWM_DC_COMBINATIONAL_TRIPIN15  (0x4000U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareBlankingEvent() as the
//! the \e blankingPulse parameter.
//
//*****************************************************************************
typedef enum
{
    //! Time base counter equals period
    EPWM_DC_WINDOW_START_TBCTR_PERIOD = 0,
    //! Time base counter equals zero
    EPWM_DC_WINDOW_START_TBCTR_ZERO = 1,
    //! Time base counter equals zero or period
    EPWM_DC_WINDOW_START_TBCTR_ZERO_PERIOD = 2,
    //! Time base counter blank pulse mix
    EPWM_DC_WINDOW_START_TBCTR_BLANK_PULSE_MIX = 3
} EPWM_DigitalCompareBlankingPulse;

//*****************************************************************************
//
// Values that can be passed to EPWM_setDigitalCompareBlankingEvent()
// as the mixedSource parameter.
//
//*****************************************************************************
//! Time-base counter equal to zero
#define EPWM_DC_TBCTR_ZERO            (0x1)
//! Time-base counter equal to period
#define EPWM_DC_TBCTR_PERIOD          (0x2)
//! time-base counter equal to CMPA when the timer is incrementing
#define EPWM_DC_TBCTR_U_CMPA          (0x4)
//! time-base counter equal to CMPA when the timer is decrementing
#define EPWM_DC_TBCTR_D_CMPA          (0x8)
//! time-base counter equal to CMPB when the timer is incrementing
#define EPWM_DC_TBCTR_U_CMPB          (0x10)
//! time-base counter equal to CMPB when the timer is decrementing
#define EPWM_DC_TBCTR_D_CMPB          (0x20)
//! time-base counter equal to CMPC when the timer is incrementing
#define EPWM_DC_TBCTR_U_CMPC          (0x40)
//! time-base counter equal to CMPC when the timer is decrementing
#define EPWM_DC_TBCTR_D_CMPC          (0x80)
//! time-base counter equal to CMPD when the timer is incrementing
#define EPWM_DC_TBCTR_U_CMPD          (0x100)
//! time-base counter equal to CMPD when the timer is decrementing
#define EPWM_DC_TBCTR_D_CMPD          (0x200)

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareFilterInput()
//! as the \e filterInput parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DC_WINDOW_SOURCE_DCAEVT1 = 0, //!< DC filter signal source is DCAEVT1
    EPWM_DC_WINDOW_SOURCE_DCAEVT2 = 1, //!< DC filter signal source is DCAEVT2
    EPWM_DC_WINDOW_SOURCE_DCBEVT1 = 2, //!< DC filter signal source is DCBEVT1
    EPWM_DC_WINDOW_SOURCE_DCBEVT2 = 3  //!< DC filter signal source is DCBEVT2
} EPWM_DigitalCompareFilterInput;

//*****************************************************************************
//
//! Values that can be assigned to EPWM_setDigitalCompareEventSource(),
//! EPWM_setDigitalCompareEventSyncMode(),EPWM_enableDigitalCompareSyncEvent()
//! EPWM_enableDigitalCompareADCTrigger(),EPWM_disableDigitalCompareSyncEvent()
//! EPWM_setDigitalCompareCBCLatchMode(),EPWM_getDigitalCompareCBCLatchStatus()
//! EPWM_selectDigitalCompareCBCLatchClearEvent(),
//! EPWM_disableDigitalCompareADCTrigger() as the \e dcModule parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DC_MODULE_A = 0,  //!< Digital Compare Module A
    EPWM_DC_MODULE_B = 1   //!< Digital Compare Module B
} EPWM_DigitalCompareModule;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareEventSource(),
//! EPWM_setDigitalCompareEventSyncMode, EPWM_setDigitalCompareCBCLatchMode(),
//! EPWM_selectDigitalCompareCBCLatchClearEvent(),
//! EPWM_getDigitalCompareCBCLatchStatus()
//! as the \e dcEvent parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DC_EVENT_1 = 0,  //!< Digital Compare Event number 1
    EPWM_DC_EVENT_2 = 1   //!< Digital Compare Event number 2
} EPWM_DigitalCompareEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareEventSource() as the
//! \e dcEventSource parameter.
//
//*****************************************************************************
typedef enum
{
    //! signal source is unfiltered (DCAEVT1/2)
    EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL = 0,
    //! signal source is filtered (DCEVTFILT)
    EPWM_DC_EVENT_SOURCE_FILT_SIGNAL = 1
} EPWM_DigitalCompareEventSource;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareEventSyncMode() as the
//! \e syncMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! DC input signal is synced with TBCLK
    EPWM_DC_EVENT_INPUT_SYNCED = 0,
    //! DC input signal is not synced with TBCLK
    EPWM_DC_EVENT_INPUT_NOT_SYNCED = 1
} EPWM_DigitalCompareSyncMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareCBCLatchMode() as the
//! \e latchMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! DC cycle-by-cycle(CBC) latch is disabled
    EPWM_DC_CBC_LATCH_DISABLED = 0x0,
    //! DC cycle-by-cycle(CBC) latch is enabled
    EPWM_DC_CBC_LATCH_ENABLED = 0x1
} EPWM_DigitalCompareCBCLatchMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_selectDigitalCompareCBCLatchClearEvent()
//! as the \e latchMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Clear CBC latch when counter equals zero
    EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO = 0x0,
    //! Clear CBC latch when counter equals period
    EPWM_DC_CBC_LATCH_CLR_ON_CNTR_PERIOD = 0x1,
    //! Clear CBC latch when counter equals zero or period
    EPWM_DC_CBC_LATCH_CLR_ON_CNTR_ZERO_PERIOD = 0x2
} EPWM_DigitalCompareCBCLatchClearEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setGlobalLoadTrigger() as the
//! \e loadTrigger parameter.
//
//*****************************************************************************
typedef enum
{
    //! load when counter is equal to zero
    EPWM_GL_LOAD_PULSE_CNTR_ZERO = 0x0,
    //! load when counter is equal to period
    EPWM_GL_LOAD_PULSE_CNTR_PERIOD = 0x1,
    //! load when counter is equal to zero or period
    EPWM_GL_LOAD_PULSE_CNTR_ZERO_PERIOD = 0x2,
    //! load on sync event
    EPWM_GL_LOAD_PULSE_SYNC = 0x3,
    //! load on sync event or when counter  is equal to zero
    EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_ZERO = 0x4,
    //! load on sync event or when counter  is equal to period
    EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_PERIOD = 0x5,
    //! load on sync event or when counter is equal to period or zero
    EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD = 0x6,
    //! load when counter is equal to cmpc and cmpc is incrementing
    EPWM_GL_LOAD_PULSE_CNTR_CMPC_U = 0x8,
    //! load when counter is equal to cmpc and cmpc is decrementing
    EPWM_GL_LOAD_PULSE_CNTR_CMPC_D = 0x9,
    //! load when counter is equal to cmpd and cmpd is incrementing
    EPWM_GL_LOAD_PULSE_CNTR_CMPD_U = 0xA,
    //! load when counter is equal to cmpd and cmpd is decrementing
    EPWM_GL_LOAD_PULSE_CNTR_CMPD_D = 0xB,
    //! load on global force
    EPWM_GL_LOAD_PULSE_GLOBAL_FORCE = 0xF
} EPWM_GlobalLoadTrigger;

//*****************************************************************************
//
// Values that can be passed to EPWM_enableGlobalLoadRegisters(),
// EPWM_disableGlobalLoadRegisters() as theloadRegister parameter.
//
//*****************************************************************************
//! Global load TBPRD : TBPRDHR
#define EPWM_GL_REGISTER_TBPRD_TBPRDHR  (0x1U)
//! Global load CMPA : CMPAHR
#define EPWM_GL_REGISTER_CMPA_CMPAHR  (0x2U)
//! Global load CMPB : CMPBHR
#define EPWM_GL_REGISTER_CMPB_CMPBHR  (0x4U)
//! Global load CMPC
#define EPWM_GL_REGISTER_CMPC  (0x8U)
//! Global load CMPD
#define EPWM_GL_REGISTER_CMPD  (0x10U)
//! Global load DBRED : DBREDHR
#define EPWM_GL_REGISTER_DBRED_DBREDHR  (0x20U)
//! Global load DBFED : DBFEDHR
#define EPWM_GL_REGISTER_DBFED_DBFEDHR  (0x40U)
//! Global load DBCTL
#define EPWM_GL_REGISTER_DBCTL  (0x80U)
//! Global load AQCTLA/A2
#define EPWM_GL_REGISTER_AQCTLA_AQCTLA2  (0x100U)
//! Global load AQCTLB/B2
#define EPWM_GL_REGISTER_AQCTLB_AQCTLB2  (0x200U)
//! Global load AQCSFRC
#define EPWM_GL_REGISTER_AQCSFRC  (0x400U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_setValleyTriggerSource() as the \e
//! trigger parameter.
//
//*****************************************************************************
typedef enum
{
    //! Valley capture trigged by software.
    EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE = 0U,
    //! Valley capture trigged by when counter is equal to zero.
    EPWM_VALLEY_TRIGGER_EVENT_CNTR_ZERO = 1U,
    //! Valley capture trigged by when counter is equal period.
    EPWM_VALLEY_TRIGGER_EVENT_CNTR_PERIOD = 2U,
    //! Valley capture trigged when counter is equal to zero or period.
    EPWM_VALLEY_TRIGGER_EVENT_CNTR_ZERO_PERIOD = 3U,
    //! Valley capture trigged by DCAEVT1 (Digital Compare A event 1)
    EPWM_VALLEY_TRIGGER_EVENT_DCAEVT1 = 4U,
    //! Valley capture trigged by DCAEVT2 (Digital Compare A event 2)
    EPWM_VALLEY_TRIGGER_EVENT_DCAEVT2 = 5U,
    //! Valley capture trigged by DCBEVT1 (Digital Compare B event 1)
    EPWM_VALLEY_TRIGGER_EVENT_DCBEVT1 = 6U,
    //! Valley capture trigged by DCBEVT2 (Digital Compare B event 2)
    EPWM_VALLEY_TRIGGER_EVENT_DCBEVT2 = 7U
} EPWM_ValleyTriggerSource;

//*****************************************************************************
//
//! Values that can be passed to EPWM_getValleyEdgeStatus() as the \e edge
//! parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_VALLEY_COUNT_START_EDGE = 0,  //!< Valley count start edge
    EPWM_VALLEY_COUNT_STOP_EDGE = 1  //!< Valley count stop edge
} EPWM_ValleyCounterEdge;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setValleyDelayDivider() as the
//! \e delayMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Delay value equals the offset value defines by software
    EPWM_VALLEY_DELAY_MODE_SW_DELAY = 0U,
    //! Delay value equals the sum of the Hardware counter value and
    //! the offset value defines by software
    EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SW_DELAY = 1U,
    //! Delay value equals the the Hardware counter shifted by
    //! (1 + the offset value defines by software)
    EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SHIFT_1_SW_DELAY = 2U,
    //! Delay value equals the the Hardware counter shifted by
    //! (2 + the offset value defines by software)
    EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SHIFT_2_SW_DELAY = 3U,
    //! Delay value equals the the Hardware counter shifted by
    //! (4 + the offset value defines by software)
    EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SHIFT_4_SW_DELAY = 4U
} EPWM_ValleyDelayMode;

//
// DC Edge Filter
//
//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareEdgeFilterMode()
//! as the \e edgeMode parameter.
//
//*****************************************************************************
typedef enum
{
    //!< Digital Compare Edge filter low to high edge mode
    EPWM_DC_EDGEFILT_MODE_RISING = 0,
    //!< Digital Compare Edge filter high to low edge mode
    EPWM_DC_EDGEFILT_MODE_FALLING = 1,
    //!< Digital Compare Edge filter both edges mode
    EPWM_DC_EDGEFILT_MODE_BOTH = 2
} EPWM_DigitalCompareEdgeFilterMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareEdgeFilterEdgeCount()
//! as the \e edgeCount parameter.
//
//*****************************************************************************
typedef enum
{
    //!< Digital Compare Edge filter edge count = 0
    EPWM_DC_EDGEFILT_EDGECNT_0 = 0,
    //!< Digital Compare Edge filter edge count = 1
    EPWM_DC_EDGEFILT_EDGECNT_1 = 1,
    //!< Digital Compare Edge filter edge count = 2
    EPWM_DC_EDGEFILT_EDGECNT_2 = 2,
    //!< Digital Compare Edge filter edge count = 3
    EPWM_DC_EDGEFILT_EDGECNT_3 = 3,
    //!< Digital Compare Edge filter edge count = 4
    EPWM_DC_EDGEFILT_EDGECNT_4 = 4,
    //!< Digital Compare Edge filter edge count = 5
    EPWM_DC_EDGEFILT_EDGECNT_5 = 5,
    //!< Digital Compare Edge filter edge count = 6
    EPWM_DC_EDGEFILT_EDGECNT_6 = 6,
    //!< Digital Compare Edge filter edge count = 7
    EPWM_DC_EDGEFILT_EDGECNT_7 = 7
} EPWM_DigitalCompareEdgeFilterEdgeCount;

//*****************************************************************************
//
//! Values that can be passed to EPWM_lockRegisters() as the \e registerGroup
//! parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_REGISTER_GROUP_HR = 0x1,  //!< HRPWM register group
    EPWM_REGISTER_GROUP_GLOBAL_LOAD = 0x2,  //!< Global load register group
    EPWM_REGISTER_GROUP_TRIP_ZONE = 0x4,  //!< Trip zone register group
    EPWM_REGISTER_GROUP_TRIP_ZONE_CLEAR = 0x8,  //!< Trip zone clear group
    EPWM_REGISTER_GROUP_DIGITAL_COMPARE = 0x10  //!< Digital compare group
} EPWM_LockRegisterGroup;

//
// Minimum Dead Band
//
//*****************************************************************************
//
//! Values that can be passed to
//
//*****************************************************************************
//!< Minimum Dead Band Block A
#define EPWM_MINDB_BLOCK_A    (0x0U)
//!< Minimum Dead Band Block B
#define EPWM_MINDB_BLOCK_B    (0x1U)

//*****************************************************************************
//
//! Values that can be passed to
//
//*****************************************************************************
//!< Minimum Dead Band don't invert
#define EPWM_MINDB_NO_INVERT    (0x0)
//!< Minimum Dead invert reference signal
#define EPWM_MINDB_INVERT       (0x1)

//*****************************************************************************
//
//! Values that can be passed to
//
//*****************************************************************************
//!< Minimum Dead Band Invert and perform logical AND
#define EPWM_MINDB_INVERT_LOGICAL_AND    (0x0)
//!< Minimum Dead Band Block perform logical OR
#define EPWM_MINDB_LOGICAL_OR            (0x1)

//*****************************************************************************
//
//! Values that can be passed to
//
//*****************************************************************************
//!< Minimum Dead Band Reference signal
#define EPWM_MINDB_PWMB                 (0x0)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 1
#define EPWM_MINDB_PWM_OUTXBAR_OUT1     (0x1)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 2
#define EPWM_MINDB_PWM_OUTXBAR_OUT2     (0x2)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 3
#define EPWM_MINDB_PWM_OUTXBAR_OUT3     (0x3)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 4
#define EPWM_MINDB_PWM_OUTXBAR_OUT4     (0x4)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 5
#define EPWM_MINDB_PWM_OUTXBAR_OUT5     (0x5)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 6
#define EPWM_MINDB_PWM_OUTXBAR_OUT6     (0x6)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 7
#define EPWM_MINDB_PWM_OUTXBAR_OUT7     (0x7)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 8
#define EPWM_MINDB_PWM_OUTXBAR_OUT8     (0x8)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 9
#define EPWM_MINDB_PWM_OUTXBAR_OUT9     (0x9)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 10
#define EPWM_MINDB_PWM_OUTXBAR_OUT10    (0xA)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 11
#define EPWM_MINDB_PWM_OUTXBAR_OUT11    (0xB)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 12
#define EPWM_MINDB_PWM_OUTXBAR_OUT12    (0xC)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 13
#define EPWM_MINDB_PWM_OUTXBAR_OUT13    (0xD)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 14
#define EPWM_MINDB_PWM_OUTXBAR_OUT14    (0xE)
//!< Minimum Dead Band Reference Signal PWM Output Xbar Output 15
#define EPWM_MINDB_PWM_OUTXBAR_OUT15    (0xF)

//*****************************************************************************
//
//! Values that can be passed to
//
//*****************************************************************************
//!< Minimum Dead Band Invert signal
#define EPWM_MINDB_BLOCKING_SIGNAL_SAME    (0x0)
//!< Minimum Dead Band Block B
#define EPWM_MINDB_BLOCKING_SIGNAL_DIFF    (0x1)

//*****************************************************************************
//
//! Values that can be passed to
//
//*****************************************************************************
//!< ICSS Xbar Out 0
#define EPWM_MINDB_ICSS_XBAR_OUT0     (0x0)
//!< ICSS Xbar Out 1
#define EPWM_MINDB_ICSS_XBAR_OUT1     (0x1)
//!< ICSS Xbar Out 2
#define EPWM_MINDB_ICSS_XBAR_OUT2     (0x2)
//!< ICSS Xbar Out 3
#define EPWM_MINDB_ICSS_XBAR_OUT3     (0x3)
//!< ICSS Xbar Out 4
#define EPWM_MINDB_ICSS_XBAR_OUT4     (0x4)
//!< ICSS Xbar Out 5
#define EPWM_MINDB_ICSS_XBAR_OUT5     (0x5)
//!< ICSS Xbar Out 6
#define EPWM_MINDB_ICSS_XBAR_OUT6     (0x6)
//!< ICSS Xbar Out 7
#define EPWM_MINDB_ICSS_XBAR_OUT7     (0x7)
//!< ICSS Xbar Out 8
#define EPWM_MINDB_ICSS_XBAR_OUT8     (0x8)
//!< ICSS Xbar Out 9
#define EPWM_MINDB_ICSS_XBAR_OUT9     (0x9)
//!< ICSS Xbar Out 10
#define EPWM_MINDB_ICSS_XBAR_OUT10    (0xA)
//!< ICSS Xbar Out 11
#define EPWM_MINDB_ICSS_XBAR_OUT11    (0xB)
//!< ICSS Xbar Out 12
#define EPWM_MINDB_ICSS_XBAR_OUT12    (0xC)
//!< ICSS Xbar Out 13
#define EPWM_MINDB_ICSS_XBAR_OUT13    (0xD)
//!< ICSS Xbar Out 14
#define EPWM_MINDB_ICSS_XBAR_OUT14    (0xE)
//!< ICSS Xbar Out 15
#define EPWM_MINDB_ICSS_XBAR_OUT15    (0xF)

//*****************************************************************************
//
//! Values that can be passed to HRPWM_setMEPEdgeSelect(),
//! HRPWM_setMEPControlMode(), HRPWM_setCounterCompareShadowLoadEvent()
//! as the \e channel parameter.
//
//*****************************************************************************
typedef enum
{
    HRPWM_CHANNEL_A = 0, //!< HRPWM A
    HRPWM_CHANNEL_B = 8  //!< HRPWM B
} HRPWM_Channel;

//*****************************************************************************
//
//! Values that can be passed to HRPWM_setMEPEdgeSelect() as the \e mepEdgeMode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! HRPWM is disabled
    HRPWM_MEP_CTRL_DISABLE = 0,
    //! MEP controls rising edge
    HRPWM_MEP_CTRL_RISING_EDGE = 1,
    //! MEP controls falling edge
    HRPWM_MEP_CTRL_FALLING_EDGE = 2,
    //! MEP controls both rising and falling edge
    HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE = 3
} HRPWM_MEPEdgeMode;

//*****************************************************************************
//
//! Values that can be passed to HRPWM_setHRMEPCtrlMode() as the \e
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! CMPAHR/CMPBHR or TBPRDHR controls MEP edge
    HRPWM_MEP_DUTY_PERIOD_CTRL = 0,
    //! TBPHSHR controls MEP edge
    HRPWM_MEP_PHASE_CTRL = 1
} HRPWM_MEPCtrlMode;

//*****************************************************************************
//
//! Values that can be passed to HRPWM_setCounterCompareShadowLoadEvent(),
//! HRPWM_setRisingEdgeDelayLoadMode() and HRPWM_setFallingEdgeDelayLoadMode
//! as the \e loadEvent parameter.
//
//*****************************************************************************
typedef enum
{
    //! load when counter equals zero
    HRPWM_LOAD_ON_CNTR_ZERO = 0,
    //! load when counter equals period
    HRPWM_LOAD_ON_CNTR_PERIOD = 1,
    //! load when counter equals zero or period
    HRPWM_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! load on translater event CMPB-3
    HRPWM_LOAD_ON_CMPB_EQ = 3
} HRPWM_LoadMode;

//*****************************************************************************
//
//! Values that can be passed to HRPWM_setChannelBOutputPath() as the \e
//! outputOnB parameter.
//
//*****************************************************************************
typedef enum
{
    //!< ePWMxB output is normal.
    HRPWM_OUTPUT_ON_B_NORMAL = 0,
    //!< ePWMxB output is inverted version of ePWMxA signal
    HRPWM_OUTPUT_ON_B_INV_A  = 1
} HRPWM_ChannelBOutput;

//*****************************************************************************
//
//! Values that can be passed to HRPWM_setSyncPulseSource() as the \e
//! syncPulseSource parameter.
//
//*****************************************************************************
typedef enum
{
    //! Counter equals Period
    HRPWM_PWMSYNC_SOURCE_PERIOD = 0,
    //! Counter equals zero
    HRPWM_PWMSYNC_SOURCE_ZERO = 1,
    //! Counter equals COMPC when counting up
    HRPWM_PWMSYNC_SOURCE_COMPC_UP = 4,
    //! Counter equals COMPC when counting down
    HRPWM_PWMSYNC_SOURCE_COMPC_DOWN = 5,
    //! Counter equals COMPD when counting up
    HRPWM_PWMSYNC_SOURCE_COMPD_UP = 6,
    //! Counter equals COMPD when counting down
    HRPWM_PWMSYNC_SOURCE_COMPD_DOWN = 7
} HRPWM_SyncPulseSource;

//*****************************************************************************
//
//! Values that can be passed to HRPWM_setCounterCompareValue() as the \e
//! compModule parameter.
//
//*****************************************************************************
typedef enum
{
    HRPWM_COUNTER_COMPARE_A = 0, //!< counter compare A
    HRPWM_COUNTER_COMPARE_B = 4  //!< counter compare B
} HRPWM_CounterCompareModule;

//*****************************************************************************
//
//! Values that can be passed to HRPWM_setDeadbandMEPEdgeSelect() as the \e
//! mepDBEdge.
//
//*****************************************************************************
typedef enum
{
    //! HRPWM is disabled
    HRPWM_DB_MEP_CTRL_DISABLE = 0,
    //! MEP controls Rising Edge Delay
    HRPWM_DB_MEP_CTRL_RED = 1,
    //! MEP controls Falling Edge Delay
    HRPWM_DB_MEP_CTRL_FED = 2,
    //! MEP controls both Falling and Rising edge delay
    HRPWM_DB_MEP_CTRL_RED_FED = 3
} HRPWM_MEPDeadBandEdgeMode;
//*****************************************************************************
//
//! Values that can be passed to HRPWM_setXCMPRegValue() as the
//! \e xcmpReg parameter.
//
//*****************************************************************************
typedef enum
{
     //! XCMP1_ACTIVE
    HRPWM_XCMP1_ACTIVE = 0,
    //! XCMP2_ACTIVE
    HRPWM_XCMP2_ACTIVE = 4,
    //! XCMP3_ACTIVE
    HRPWM_XCMP3_ACTIVE = 8,
    //! XCMP4_ACTIVE
    HRPWM_XCMP4_ACTIVE = 12,
    //! XCMP5_ACTIVE
    HRPWM_XCMP5_ACTIVE = 16,
    //! XCMP6_ACTIVE
    HRPWM_XCMP6_ACTIVE = 20,
    //! XCMP7_ACTIVE
    HRPWM_XCMP7_ACTIVE = 24,
    //! XCMP8_ACTIVE
    HRPWM_XCMP8_ACTIVE = 28,
    //! XTBPRD_ACTIVE
    HRPWM_XTBPRD_ACTIVE = 32,

    //! XCMP1_SHADOW1
    HRPWM_XCMP1_SHADOW1 = 128,
    //! XCMP2_SHADOW1
    HRPWM_XCMP2_SHADOW1 = 132,
    //! XCMP3_SHADOW1
    HRPWM_XCMP3_SHADOW1 = 136,
    //! XCMP4_SHADOW1
    HRPWM_XCMP4_SHADOW1 = 140,
    //! XCMP5_SHADOW1
    HRPWM_XCMP5_SHADOW1 = 144,
    //! XCMP6_SHADOW1
    HRPWM_XCMP6_SHADOW1 = 148,
    //! XCMP7_SHADOW1
    HRPWM_XCMP7_SHADOW1 = 152,
    //! XCMP8_SHADOW1
    HRPWM_XCMP8_SHADOW1 = 156,
    //! XTBPRD_SHADOW1
    HRPWM_XTBPRD_SHADOW1 = 160,

    //! XCMP1_SHADOW2
    HRPWM_XCMP1_SHADOW2 = 256,
    //! XCMP2_SHADOW2
    HRPWM_XCMP2_SHADOW2 = 260,
    //! XCMP3_SHADOW2
    HRPWM_XCMP3_SHADOW2 = 264,
    //! XCMP4_SHADOW2
    HRPWM_XCMP4_SHADOW2 = 268,
    //! XCMP5_SHADOW2
    HRPWM_XCMP5_SHADOW2 = 272,
    //! XCMP6_SHADOW2
    HRPWM_XCMP6_SHADOW2 = 276,
    //! XCMP7_SHADOW2
    HRPWM_XCMP7_SHADOW2 = 280,
    //! XCMP8_SHADOW2
    HRPWM_XCMP8_SHADOW2 = 284,
    //! XTBPRD_SHADOW2
    HRPWM_XTBPRD_SHADOW2 = 288,

    //! XCMP1_SHADOW3
    HRPWM_XCMP1_SHADOW3 = 384,
    //! XCMP2_SHADOW3
    HRPWM_XCMP2_SHADOW3 = 388,
    //! XCMP3_SHADOW3
    HRPWM_XCMP3_SHADOW3 = 392,
    //! XCMP4_SHADOW3
    HRPWM_XCMP4_SHADOW3 = 396,
    //! XCMP5_SHADOW3
    HRPWM_XCMP5_SHADOW3 = 400,
    //! XCMP6_SHADOW3
    HRPWM_XCMP6_SHADOW3 = 404,
    //! XCMP7_SHADOW3
    HRPWM_XCMP7_SHADOW3 = 408,
    //! XCMP8_SHADOW3
    HRPWM_XCMP8_SHADOW3 = 412,
    //! XTBPRD_SHADOW3
    HRPWM_XTBPRD_SHADOW3 = 416,
}HRPWM_XCMPReg;
//
//! Values that can be passed to EPWM_setXCMPActionQualifierAction()
//! and EPWM_setXCMPShadowRepeatBufxCount()
//
//*****************************************************************************
//!< XCMP set = Active
#define EPWM_XCMP_ACTIVE   (0x0U)
//!< XCMP set = Shadow 1
#define EPWM_XCMP_SHADOW1    (0x1U)
//!< XCMP set = Shadow 2
#define EPWM_XCMP_SHADOW2    (0x2U)
//!< XCMP set = Shadow 3
#define EPWM_XCMP_SHADOW3    (0x3U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_setXCMPRegValue() as the
//! \e xcmpReg parameter.
//
//*****************************************************************************
typedef enum
{
    //! XCMP1_ACTIVE
    EPWM_XCMP1_ACTIVE = 0,
    //! XCMP2_ACTIVE
    EPWM_XCMP2_ACTIVE = 4,
    //! XCMP3_ACTIVE
    EPWM_XCMP3_ACTIVE = 8,
    //! XCMP4_ACTIVE
    EPWM_XCMP4_ACTIVE = 12,
    //! XCMP5_ACTIVE
    EPWM_XCMP5_ACTIVE = 16,
    //! XCMP6_ACTIVE
    EPWM_XCMP6_ACTIVE = 20,
    //! XCMP7_ACTIVE
    EPWM_XCMP7_ACTIVE = 24,
    //! XCMP8_ACTIVE
    EPWM_XCMP8_ACTIVE = 28,
    //! XTBPRD_ACTIVE
    EPWM_XTBPRD_ACTIVE = 32,

    //! XCMP1_SHADOW1
    EPWM_XCMP1_SHADOW1 = 128,
    //! XCMP2_SHADOW1
    EPWM_XCMP2_SHADOW1 = 132,
    //! XCMP3_SHADOW1
    EPWM_XCMP3_SHADOW1 = 136,
    //! XCMP4_SHADOW1
    EPWM_XCMP4_SHADOW1 = 140,
    //! XCMP5_SHADOW1
    EPWM_XCMP5_SHADOW1 = 144,
    //! XCMP6_SHADOW1
    EPWM_XCMP6_SHADOW1 = 148,
    //! XCMP7_SHADOW1
    EPWM_XCMP7_SHADOW1 = 152,
    //! XCMP8_SHADOW1
    EPWM_XCMP8_SHADOW1 = 156,
     //! XTBPRD_SHADOW1
    EPWM_XTBPRD_SHADOW1 = 160,

    //! XCMP1_SHADOW2
    EPWM_XCMP1_SHADOW2 = 256,
    //! XCMP2_SHADOW2
    EPWM_XCMP2_SHADOW2 = 260,
    //! XCMP3_SHADOW2
    EPWM_XCMP3_SHADOW2 = 264,
    //! XCMP4_SHADOW2
    EPWM_XCMP4_SHADOW2 = 268,
    //! XCMP5_SHADOW2
    EPWM_XCMP5_SHADOW2 = 272,
    //! XCMP6_SHADOW2
    EPWM_XCMP6_SHADOW2 = 276,
    //! XCMP7_SHADOW2
    EPWM_XCMP7_SHADOW2 = 280,
    //! XCMP8_SHADOW2
    EPWM_XCMP8_SHADOW2 = 284,
    //! XTBPRD_SHADOW2
    EPWM_XTBPRD_SHADOW2 = 288,

    //! XCMP1_SHADOW3
    EPWM_XCMP1_SHADOW3 = 384,
    //! XCMP2_SHADOW3
    EPWM_XCMP2_SHADOW3 = 388,
    //! XCMP3_SHADOW3
    EPWM_XCMP3_SHADOW3 = 392,
    //! XCMP4_SHADOW3
    EPWM_XCMP4_SHADOW3 = 396,
    //! XCMP5_SHADOW3
    EPWM_XCMP5_SHADOW3 = 400,
    //! XCMP6_SHADOW3
    EPWM_XCMP6_SHADOW3 = 404,
    //! XCMP7_SHADOW3
    EPWM_XCMP7_SHADOW3 = 408,
    //! XCMP8_SHADOW3
    EPWM_XCMP8_SHADOW3 = 412,
    //! XTBPRD_SHADOW3
    EPWM_XTBPRD_SHADOW3 = 416,

}EPWM_XCMPReg;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setCMPShadowRegValue() as the
//! \e cmpReg parameter.
//
//*****************************************************************************
typedef enum
{
    //! CMPC_SHADOW1
    EPWM_CMPC_SHADOW1 = 0x0U,
    //! CMPD_SHADOW1
    EPWM_CMPD_SHADOW1 = 0x4U,
    //! CMPC_SHADOW2
    EPWM_CMPC_SHADOW2 = 0x80U,
    //! CMPD_SHADOW2
    EPWM_CMPD_SHADOW2 = 0x84U,
    //! CMPC_SHADOW3
    EPWM_CMPC_SHADOW3 = 0x100U,
    //! CMPD_SHADOW3
    EPWM_CMPD_SHADOW3 = 0x104U

}EPWM_XCompareReg;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setXMINMAXRegValue() as the
//! \e xminmaxReg parameter.
//
//*****************************************************************************
typedef enum
{
    //! XMAX_ACTIVE
    EPWM_XMAX_ACTIVE = 0x0U,
    //! XMIN_ACTIVE
    EPWM_XMIN_ACTIVE = 0x2U,
      //! XMAX_SHADOW1
    EPWM_XMAX_SHADOW1 = 0x80U,
    //! XMIN_SHADOW1
    EPWM_XMIN_SHADOW1 = 0x82U,
    //! XMAX_SHADOW2
    EPWM_XMAX_SHADOW2 = 0x100U,
    //! XMIN_SHADOW2
    EPWM_XMIN_SHADOW2 = 0x102U,
    //! XMAX_SHADOW3
    EPWM_XMAX_SHADOW3 = 0x180U,
    //! XMIN_SHADOW3
    EPWM_XMIN_SHADOW3 = 0x182U,

}EPWM_XMinMaxReg;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setXCMPActionQualifierAction() as the \e event parameter.
//
//*****************************************************************************
typedef enum
{
    //! Time base counter equals XCMP1
    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1 = 0,
    //! Time base counter equals XCMP2
    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2 = 2,
    //! Time base counter equals XCMP3
    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3 = 4,
    //! Time base counter equals XCMP4
    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4 = 6,
    //! Time base counter equals XCMP5
    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5 = 8,
    //! Time base counter equals XCMP6
    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6 = 10,
    //! Time base counter equals XCMP7
    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7 = 12,
    //! Time base counter equals XCMP8
    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8 = 14
} EPWM_XCMPActionQualifierOutputEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_allocAXCMP() as the \e alloctype parameter.
//
//*****************************************************************************

typedef enum
{
    //! Allocate 0 XCMP registers to CMPA
    EPWM_XCMP_NONE_CMPA = 0,
    //! Allocate XCMP1 register to CMPA
    EPWM_XCMP_1_CMPA = 1,
    //! Allocate XCMP1 - XCMP2 registers to CMPA
    EPWM_XCMP_2_CMPA = 2,
    //! Allocate XCMP1 - XCMP3 registers to CMPA
    EPWM_XCMP_3_CMPA = 3,
    //! Allocate XCMP1 - XCMP4 registers to CMPA
    EPWM_XCMP_4_CMPA = 4,
    //! Allocate XCMP1 - XCMP5 registers to CMPA
    EPWM_XCMP_5_CMPA = 5,
    //! Allocate XCMP1 - XCMP6 registers to CMPA
    EPWM_XCMP_6_CMPA = 6,
    //! Allocate XCMP1 - XCMP7 registers to CMPA
    EPWM_XCMP_7_CMPA = 7,
    //! Allocate XCMP1 - XCMP8 registers to CMPA
    EPWM_XCMP_8_CMPA = 8
}EPWM_XCMP_ALLOC_CMPA;

//*****************************************************************************
//
//! Values that can be passed to EPWM_allocBXCMP() as the \e alloctype parameter.
//
//*****************************************************************************

typedef enum
{
    //! Allocate XCMP5 register to CMPB
    EPWM_XCMP_1_CMPB = 5,
    //! Allocate XCMP5 - XCMP6 registers to CMPB
    EPWM_XCMP_2_CMPB = 6,
    //! Allocate XCMP5 - XCMP7 registers to CMPB
    EPWM_XCMP_3_CMPB = 7,
    //! Allocate XCMP5 - XCMP8 registers to CMPB
    EPWM_XCMP_4_CMPB = 8
}EPWM_XCMP_ALLOC_CMPB;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setXCMPLoadMode() as the
//! \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Load mode is LOADONCE
    EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE = 0,
    //! Load mode is LOADMULTIPLE
    EPWM_XCMP_XLOADCTL_LOADMODE_LOADMULTIPLE = 1
}EPWM_XCMPXloadCtlLoadMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setXCMPShadowLevel() as the
//! \e level parameter.
//
//*****************************************************************************

typedef enum
{
    //!  Only Active register is available
    EPWM_XCMP_XLOADCTL_SHDWLEVEL_0 = 0,
    //! SHDW1 and Active registers are available
    EPWM_XCMP_XLOADCTL_SHDWLEVEL_1 = 1,
    //! SHDW2, SHDW1 and Active registers are available
    EPWM_XCMP_XLOADCTL_SHDWLEVEL_2 = 2,
    //! SHDW3, SHDW2, SHDW1 and Active registers are available
    EPWM_XCMP_XLOADCTL_SHDWLEVEL_3 = 3
}EPWM_XCMP_XLOADCTL_SHDWLEVEL;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setXCMPShadowBufPtrLoadOnce() as the
//! \e ptr parameter.
//
//*****************************************************************************

typedef enum
{
    //! No Shadow buffer is in use
    EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL = 0,
    //! Shadow buffer 1 is in use
    EPWM_XCMP_XLOADCTL_SHDWBUFPTR_ONE =  1,
    //! Shadow buffer 2 is in use
    EPWM_XCMP_XLOADCTL_SHDWBUFPTR_TWO =  2,
    //! Shadow buffer 3 is in use
    EPWM_XCMP_XLOADCTL_SHDWBUFPTR_THREE =3
}EPWM_XCMP_XLOADCTL_SHDWBUFPTR;

//
// Diode Emulation Logic
//
//*****************************************************************************
//
//! Values that can be passed to EPWM_setDiodeEmulationMode() as the
//! \e mode parameter.
//
//*****************************************************************************
typedef enum{
    //! Diode Emulation mode is Cycle by Cycle
    EPWM_DIODE_EMULATION_CBC = 0,
    //! Diode Emulation mode is One Shot
    EPWM_DIODE_EMULATION_OST = 1
}EPWM_DiodeEmulationMode;


//*****************************************************************************
//
//! Values that can be passed to EPWM_configureDiodeEmulationTripSources() as the
//! \e source parameter.
//
//*****************************************************************************
typedef enum{
    //! Trip source is INPUTXBAR out0 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0 = 0x01,
    //! Trip source is INPUTXBAR out1 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT1 = 0x02,
    //! Trip source is INPUTXBAR out2 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT2 = 0x03,
    //! Trip source is INPUTXBAR out3 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT3 = 0x04,
    //! Trip source is INPUTXBAR out4 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT4 = 0x05,
    //! Trip source is INPUTXBAR out5 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT5 = 0x06,
    //! Trip source is INPUTXBAR out6 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT6 = 0x07,
    //! Trip source is INPUTXBAR out7 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT7 = 0x08,
    //! Trip source is INPUTXBAR out8 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT8 = 0x09,
    //! Trip source is INPUTXBAR out9 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT9 = 0x0A,
    //! Trip source is INPUTXBAR out10 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT10 = 0x0B,
    //! Trip source is INPUTXBAR out11 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT11 = 0x0C,
    //! Trip source is INPUTXBAR out12 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT12 = 0x0D,
    //! Trip source is INPUTXBAR out13 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT13 = 0x0E,
    //! Trip source is INPUTXBAR out14 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT14 = 0x0F,
    //! Trip source is INPUTXBAR out15 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT15 = 0x10,
    //! Trip source is INPUTXBAR out16 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT16 = 0x11,
    //! Trip source is INPUTXBAR out17 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT17 = 0x12,
    //! Trip source is INPUTXBAR out18 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT18 = 0x13,
    //! Trip source is INPUTXBAR out19 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT19 = 0x14,
    //! Trip source is INPUTXBAR out20 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT20 = 0x15,
    //! Trip source is INPUTXBAR out21 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT21 = 0x16,
    //! Trip source is INPUTXBAR out22 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT22 = 0x17,
    //! Trip source is INPUTXBAR out23 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT23 = 0x18,
    //! Trip source is INPUTXBAR out24 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT24 = 0x19,
    //! Trip source is INPUTXBAR out25 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT25 = 0x1A,
    //! Trip source is INPUTXBAR out26 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT26 = 0x1B,
    //! Trip source is INPUTXBAR out27 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT27 = 0x1C,
    //! Trip source is INPUTXBAR out28 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT28 = 0x1D,
    //! Trip source is INPUTXBAR out29 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT29 = 0x1E,
    //! Trip source is INPUTXBAR out30 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT30 = 0x1F,
    //! Trip source is INPUTXBAR out31 signal
    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT31 = 0x20,
    //! Trip source is CMPSSA0 signal
    EPWM_DE_TRIP_SRC_CMPSSA0 = 0x21,
    //! Trip source is CMPSSA1 signal
    EPWM_DE_TRIP_SRC_CMPSSA1 = 0x22,
    //! Trip source is CMPSSA2 signal
    EPWM_DE_TRIP_SRC_CMPSSA2 = 0x23,
    //! Trip source is CMPSSA3 signal
    EPWM_DE_TRIP_SRC_CMPSSA3 = 0x24,
    //! Trip source is CMPSSA4 signal
    EPWM_DE_TRIP_SRC_CMPSSA4 = 0x25,
    //! Trip source is CMPSSA5 signal
    EPWM_DE_TRIP_SRC_CMPSSA5 = 0x26,
    //! Trip source is CMPSSA6 signal
    EPWM_DE_TRIP_SRC_CMPSSA6 = 0x27,
    //! Trip source is CMPSSA7 signal
    EPWM_DE_TRIP_SRC_CMPSSA7 = 0x28,
    //! Trip source is CMPSSA8 signal
    EPWM_DE_TRIP_SRC_CMPSSA8 = 0x29,
    //! Trip source is CMPSSA9 signal
    EPWM_DE_TRIP_SRC_CMPSSA9 = 0x2A,
    //! Trip source is CMPSSB0 signal
    EPWM_DE_TRIP_SRC_CMPSSB0 = 0x31,
    //! Trip source is CMPSSB1 signal
    EPWM_DE_TRIP_SRC_CMPSSB1 = 0x32,
    //! Trip source is CMPSSB2 signal
    EPWM_DE_TRIP_SRC_CMPSSB2 = 0x33,
    //! Trip source is CMPSSB3 signal
    EPWM_DE_TRIP_SRC_CMPSSB3 = 0x34,
    //! Trip source is CMPSSB4 signal
    EPWM_DE_TRIP_SRC_CMPSSB4 = 0x35,
    //! Trip source is CMPSSB5 signal
    EPWM_DE_TRIP_SRC_CMPSSB5 = 0x36,
    //! Trip source is CMPSSB6 signal
    EPWM_DE_TRIP_SRC_CMPSSB6 = 0x37,
    //! Trip source is CMPSSB7 signal
    EPWM_DE_TRIP_SRC_CMPSSB7 = 0x38,
    //! Trip source is CMPSSB8 signal
    EPWM_DE_TRIP_SRC_CMPSSB8 = 0x39,
    //! Trip source is CMPSSB9 signal
    EPWM_DE_TRIP_SRC_CMPSSB9 = 0x3A
}EPWM_DiodeEmulationTripSource;


typedef enum{

    //!synchronized version of TRIPH or TRIPL signal
    EPWM_DE_SYNC_TRIPHorL = 0x00,
    //!synchronized and inverted version of TRIPH or TRIPL signal
    EPWM_DE_SYNC_INV_TRIPHorL = 0x01,
    //!a constant low signal
    EPWM_DE_LOW = 0x10,
    //!a constant high signal
    EPWM_DE_HIGH = 0x11
}EPWM_DiodeEmulationSignal;
//*****************************************************************************
//
//! Values that can be passed to EPWM_selectDiodeEmulationPWMsignal()
//! and EPWM_selectDiodeEmulationTripSignal()
//
//*****************************************************************************
//!< Diode emulation channel A
#define EPWM_DE_CHANNEL_A    (0x0U)
//!< Diode emulation channel B
#define EPWM_DE_CHANNEL_B    (0x1U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDiodeEmulationMonitorModeStep()
//
//*****************************************************************************

//!< Diode emulation count up step size
#define EPWM_DE_COUNT_UP    (0x0U)
//!< Diode emulation count down step size
#define EPWM_DE_COUNT_DOWN    (0x1U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_configureDiodeEmulationTripSources()
//
//*****************************************************************************

//!< Diode emulation Trip L
#define EPWM_DE_TRIPL    (0x1U)
//!< Diode emulation Trip H
#define EPWM_DE_TRIPH    (0x0U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_configCaptureGateInputPolarity()
//! as \e polSel parameter.
//
//*****************************************************************************
//! Capture gate is always on
#define EPWM_CAPGATE_INPUT_ALWAYS_ON             (0U)
//! Capture gate is always off
#define EPWM_CAPGATE_INPUT_ALWAYS_OFF            (1U)
//! Capture gate input is CAPGATE.sync
#define EPWM_CAPGATE_INPUT_SYNC                  (2U)
//! Capture gate input is CAPGATE.sync inverted
#define EPWM_CAPGATE_INPUT_SYNC_INVERT           (3U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_invertCaptureInputPolarity()
//! as the \e polSel parameter.
//
//*****************************************************************************
//! Capture input is not inverted
#define EPWM_CAPTURE_INPUT_CAPIN_SYNC            (0U)
//! Capture input is inverted
#define EPWM_CAPTURE_INPUT_CAPIN_SYNC_INVERT     (1U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_invertCaptureInputPolarity()
//! EPWM_enableCaptureTripCombinationInput(),
//! EPWM_disableCaptureTripCombinationInput()
//! as the \e polSel parameter.
//
//*****************************************************************************
//! Capture Gate
#define EPWM_CAPTURE_GATE              (1U)
//! Capture Input
#define EPWM_CAPTURE_INPUT             (0U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierContSWForceAction_opt_outputs()
//! as the \e outputAB parameter.
//
//*****************************************************************************
//! software force disabled on A, B
#define EPWM_AQ_A_SW_DISABLED_B_SW_DISABLED        (0x0U)
//! software force Low Action on A, software force disabled B
#define EPWM_AQ_A_SW_OUTPUT_LOW_B_SW_DISABLED      (0x1U)
//! software force High Action on A, software force disabled B
#define EPWM_AQ_A_SW_OUTPUT_HIGH_B_SW_DISABLED     (0x2U)
//! software force disabled on A, software force Low Action on B
#define EPWM_AQ_A_SW_DISABLED_B_SW_OUTPUT_LOW      (0x4U)
//! software force Low Action on A, software force Low Action on B
#define EPWM_AQ_A_SW_OUTPUT_LOW_B_SW_OUTPUT_LOW    (0x5U)
//! software force High Action on A, software force Low Action on B
#define EPWM_AQ_A_SW_OUTPUT_HIGH_B_SW_OUTPUT_LOW   (0x6U)
//! software force disabled on A, software force High Action on B
#define EPWM_AQ_A_SW_DISABLED_B_SW_OUTPUT_HIGH     (0x8U)
//! software force Low Action on A, software force High Action on B
#define EPWM_AQ_A_SW_OUTPUT_LOW_B_SW_OUTPUT_HIGH   (0x9U)
//! software force High Action on A, software force High Action on B
#define EPWM_AQ_A_SW_OUTPUT_HIGH_B_SW_OUTPUT_HIGH  (0xAU)


//*****************************************************************************
//
//! Defines to be used by the driver
//
//*****************************************************************************
#define EPWM_DCxCTL_STEP (CSL_EPWM_DCBCTL - CSL_EPWM_DCACTL)
#define EPWM_DCxxTRIPSEL (CSL_EPWM_DCALTRIPSEL - CSL_EPWM_DCAHTRIPSEL)
#define EPWM_XREGSHDWxSTS_STEP (CSL_EPWM_XREGSHDW2STS-CSL_EPWM_XREGSHDW1STS)
#define EPWM_XCMPx_ACTIVE_STEP (CSL_EPWM_XCMP2_ACTIVE-CSL_EPWM_XCMP1_ACTIVE)
#define EPWM_XCMPx_STEP (CSL_EPWM_XCMP1_SHDW2-CSL_EPWM_XCMP1_SHDW1)
#define EPWM_XCMPx_SHDWx_STEP (CSL_EPWM_XCMP2_SHDW1-CSL_EPWM_XCMP1_SHDW1)
#define EPWM_LOCK_KEY (0xA5A50000U)

//*****************************************************************************
//
//! Values that can be passed to EPWM_configureSignal() as the
//! \e signalParams parameter.
//
//*****************************************************************************
typedef struct
{
    Float32                freqInHz;    //!< Desired Signal Frequency(in Hz)
    Float32                dutyValA;    //!< Desired ePWMxA Signal Duty
    Float32                dutyValB;    //!< Desired ePWMxB Signal Duty
    bool                   invertSignalB; //!< Invert ePWMxB Signal if true
    Float32                sysClkInHz;  //!< SYSCLK Frequency(in Hz)
    EPWM_TimeBaseCountMode tbCtrMode;   //!< Time Base Counter Mode
    EPWM_ClockDivider      tbClkDiv;    //!< Time Base Counter Clock Divider
    EPWM_HSClockDivider    tbHSClkDiv;  //!< Time Base Counter HS Clock Divider
} EPWM_SignalParams;

//
// Time Base Sub Module related APIs
//
//*****************************************************************************
//
//! Set the time base count
//!
//! \param base is the base address of the EPWM module.
//! \param count is the time base count value.
//!
//! This function sets the 16 bit counter value of the time base counter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTimeBaseCounter(uint32_t base, uint16_t count)
{
    //
    // Write to TBCTR register
    //
    HW_WR_REG16(base + CSL_EPWM_TBCTR, count);
}

//*****************************************************************************
//
//! Set count mode after phase shift sync
//!
//! \param base is the base address of the EPWM module.
//! \param mode is the count mode.
//!
//! This function sets the time base count to count up or down after a new
//! phase value set by the EPWM_setPhaseShift(). The count direction is
//! determined by the variable mode. Valid inputs for mode are:
//!  - EPWM_COUNT_MODE_UP_AFTER_SYNC      - Count up after sync
//!  - EPWM_COUNT_MODE_DOWN_AFTER_SYNC    - Count down after sync
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setCountModeAfterSync(uint32_t base, EPWM_SyncCountMode mode)
{
    if(mode == EPWM_COUNT_MODE_UP_AFTER_SYNC)
    {
        //
        // Set PHSDIR bit
        //
        HW_WR_REG16(base + CSL_EPWM_TBCTL,
            (HW_RD_REG16(base + CSL_EPWM_TBCTL) |
            CSL_EPWM_TBCTL_PHSDIR_MASK));
    }
    else
    {
        //
        // Clear PHSDIR bit
        //
        HW_WR_REG16(base + CSL_EPWM_TBCTL,
            (HW_RD_REG16(base + CSL_EPWM_TBCTL) &
            ~CSL_EPWM_TBCTL_PHSDIR_MASK));
    }
}

//*****************************************************************************
//
//! Set the time base clock and the high speed time base clock count pre-scaler
//!
//! \param base is the base address of the EPWM module.
//! \param prescaler is the time base count pre scale value.
//! \param highSpeedPrescaler is the high speed time base count pre scale
//!        value.
//!
//! This function sets the pre scaler(divider)value for the time base clock
//! counter and the high speed time base clock counter.
//! Valid values for pre-scaler and highSpeedPrescaler are EPWM_CLOCK_DIVIDER_X
//! where X is 1,2,4,8,16, 32,64 or 128.
//! The actual numerical values for these macros represent values 0,1...7.
//! The equation for the output clock is:
//!   TBCLK = EPWMCLK/(highSpeedPrescaler * pre-scaler)
//!
//! \b Note: EPWMCLK is a scaled version of SYSCLK. At reset EPWMCLK is half
//!          SYSCLK.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setClockPrescaler(uint32_t base, EPWM_ClockDivider prescaler,
                       EPWM_HSClockDivider highSpeedPrescaler)
{
    //
    // Write to CLKDIV and HSPCLKDIV bit
    //
    HW_WR_REG16(base + CSL_EPWM_TBCTL,
        ((HW_RD_REG16(base + CSL_EPWM_TBCTL) &
        ~(CSL_EPWM_TBCTL_CLKDIV_MASK | CSL_EPWM_TBCTL_HSPCLKDIV_MASK)) |
        (uint16_t)(((uint16_t)prescaler << CSL_EPWM_TBCTL_CLKDIV_SHIFT) |
        (uint16_t)((uint16_t)highSpeedPrescaler << CSL_EPWM_TBCTL_HSPCLKDIV_SHIFT))));
}

//*****************************************************************************
//
//! Force a software sync pulse
//!
//! \param base is the base address of the EPWM module.
//!
//! This function causes a single software initiated sync pulse. Make sure the
//! appropriate mode is selected using EPWM_setupSyncOutputMode() before using
//! this function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_forceSyncPulse(uint32_t base)
{
    //
    // Set SWFSYNC bit
    //
    HW_WR_REG16(base + CSL_EPWM_TBCTL,
        HW_RD_REG16(base + CSL_EPWM_TBCTL) | CSL_EPWM_TBCTL_SWFSYNC_MASK);
}

//*****************************************************************************
//
//! Set up the source for sync-in pulse.
//!
//! \param base is the base address of the EPWM module.
//! \param source is the sync-in pulse source.
//!
//! This function set the sync in pulse source.
//! Valid values for source are:
//!  - EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1-32 - sync-in pulse source can be
//!                                             any of the EPWM1-32 sync-out
//!                                             signal
//!  - EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP1-10 - sync-in pulse source can be
//!                                             selected as any of the ECAP1-10
//!                                             sync-out signal
//!  - EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT4, 20 - sync-in pulse source can be
//!                                             selected as any of the Input
//!                                             xbar out4 and 20 signal
//!  - EPWM_SYNC_IN_PULSE_SRC_C2K_TIMESYNC_XBAR_PWM_OUT0-1 - sync-in pulse
//!                                             source can be selected as any
//!                                             of the timesync xbar pwm out0-1
//!  - EPWM_SYNC_IN_PULSE_SRC_FSIRX[0-3]_RX_TRIG[0-3] - sync-in pulse source
//!                                             can be selected as any of
//!                                             fsirx[0-3] rx trig[0-3] signal
//!  - EPWM_SYNC_IN_PULSE_SRC_DISABLE - sync-in pulse is disabled for the
//!                                     EPWM module
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setSyncInPulseSource(uint32_t base, EPWM_SyncInPulseSource source)
{
    //
    // Set EPWM Sync-In Source Mode.
    //
    HW_WR_REG16(base + CSL_EPWM_EPWMSYNCINSEL,
        ((HW_RD_REG16(base + CSL_EPWM_EPWMSYNCINSEL) &
        (~(uint16_t)CSL_EPWM_EPWMSYNCINSEL_SEL_MASK)) |
        (uint16_t)((uint16_t)source & CSL_EPWM_EPWMSYNCINSEL_SEL_MASK)));
}

//*****************************************************************************
//
//! Enables sync-out pulse source.
//!
//! \param base is the base address of the EPWM module.
//! \param source is the sync-out pulse source.
//!
//! This function enables the sync-out pulse source.
//! Below valid values for param \b source can be OR'd together to enable
//! multiple sync-out sources:
//! - EPWM_SYNC_OUT_PULSE_ON_SOFTWARE - sync-out pulse is generated by
//!                                     software when EPWM_forceSyncPulse()
//!                                     function is called or by EPWMxSYNCI
//!                                     signal.
//! - EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO - sync-out pulse is generated when
//!                                      time base counter equals zero.
//! - EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_B - sync-out pulse is generated when
//!                                           time base counter equals compare
//!                                           B value.
//! - EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_C - sync-out pulse is generated when
//!                                           time base counter equals compare
//!                                           C value.
//! - EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_D - sync-out pulse is generated when
//!                                           time base counter equals compare
//!                                           D value.
//! - EPWM_SYNC_OUT_PULSE_ON_DCA_EVT1_SYNC - sync-out pulse is generated by DCA
//!                                          event1 sync signal
//! - EPWM_SYNC_OUT_PULSE_ON_DCB_EVT1_SYNC - sync-out pulse is generated by DCB
//!                                          event1 sync signal
//! - EPWM_SYNC_OUT_PULSE_ON_ALL  - sync-out pulse is generated by all
//!                                 the above sources
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableSyncOutPulseSource(uint32_t base, uint16_t source)
{
    //
    // Check the arguments
    //
    DebugP_assert(source <= EPWM_SYNC_OUT_SOURCE_M);

    //
    // Enable selected EPWM Sync-Out Sources.
    //
    HW_WR_REG16(base + CSL_EPWM_EPWMSYNCOUTEN,
        (HW_RD_REG16(base + CSL_EPWM_EPWMSYNCOUTEN) |
        (uint16_t)source));
}

//*****************************************************************************
//
//! Disables sync-out pulse source.
//!
//! \param base is the base address of the EPWM module.
//! \param source is the sync-out pulse source.
//!
//! This function disables the sync-out pulse source.
//! Below valid values for param \b source can be OR'd together to disable
//! multiple sync-out sources:
//!  - EPWM_SYNC_OUT_PULSE_ON_SOFTWARE - disables software as sync-out source
//!
//!  - EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO - disables counter equal to zero event
//!                                       as sync-out source
//!  - EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_B - disables counter equal to cmpb
//!                                            event as sync-out source
//!  - EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_C - disables counter equal to cmpc
//!                                            event as sync-out source
//!  - EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_D - disables counter equal to cmpd
//!                                            event as sync-out source
//!  - EPWM_SYNC_OUT_PULSE_ON_DCA_EVT1_SYNC - disables DCA event1 sync signal
//!                                           as sync-out source
//!  - EPWM_SYNC_OUT_PULSE_ON_DCB_EVT1_SYNC - disables DCB event1 sync signal
//!                                           as sync-out source
//!  - EPWM_SYNC_OUT_PULSE_ON_ALL - disables all the above sync-out sources
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableSyncOutPulseSource(uint32_t base, uint16_t source)
{
    //
    // Check the arguments
    //
    DebugP_assert(source <= EPWM_SYNC_OUT_SOURCE_M);

    //
    // Disable EPWM Sync-Out Sources.
    //
    HW_WR_REG16(base + CSL_EPWM_EPWMSYNCOUTEN,
        (HW_RD_REG16(base + CSL_EPWM_EPWMSYNCOUTEN) &
        ~((uint16_t)source)));
}

//*****************************************************************************
//
//! Set up the one-shot sync-out trigger source.
//!
//! \param base is the base address of the EPWM module.
//! \param trigger is the one-shot sync-out signal trigger source.
//!
//! This function sets the one-shot sync-out trigger source.
//! Valid values for param \b trigger are:
//!  - EPWM_OSHT_SYNC_OUT_TRIG_SYNC - Trigger for one-shot sync-out signal is
//!                                    one-shot sync event.
//!  - EPWM_OSHT_SYNC_OUT_TRIG_RELOAD - Trigger for one-shot sync-out signal is
//!                                     one-shot reload event.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setOneShotSyncOutTrigger(uint32_t base,
                              EPWM_OneShotSyncOutTrigger trigger)
{
    //
    // Set source for One-Shot Sync-Out Pulse.
    //
    HW_WR_REG16(base + CSL_EPWM_TBCTL3,
        ((HW_RD_REG16(base + CSL_EPWM_TBCTL3) &
        ~(CSL_EPWM_TBCTL3_OSSFRCEN_MASK)) |
        (uint16_t)trigger));
}

//*****************************************************************************
//
//! Set PWM period load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param loadMode is the PWM period load mode.
//!
//! This function sets the load mode for the PWM period. If loadMode is set to
//! EPWM_PERIOD_SHADOW_LOAD, a write or read to the TBPRD (PWM Period count
//! register) accesses the shadow register. If loadMode is set to
//! EPWM_PERIOD_DIRECT_LOAD, a write or read to the TBPRD register accesses the
//! register directly.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setPeriodLoadMode(uint32_t base, EPWM_PeriodLoadMode loadMode)
{
    if(loadMode == EPWM_PERIOD_SHADOW_LOAD)
    {
        //
        // Clear PRDLD
        //
        HW_WR_REG16(base + CSL_EPWM_TBCTL,
            (HW_RD_REG16(base + CSL_EPWM_TBCTL) & ~CSL_EPWM_TBCTL_PRDLD_MASK));
    }
    else
    {
        //
        // Set PRDLD
        //
        HW_WR_REG16(base + CSL_EPWM_TBCTL,
            (HW_RD_REG16(base + CSL_EPWM_TBCTL) | CSL_EPWM_TBCTL_PRDLD_MASK));
    }
}

//*****************************************************************************
//
//! Enable phase shift load
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables loading of phase shift when the appropriate sync
//! event occurs.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enablePhaseShiftLoad(uint32_t base)
{
    //
    // Set PHSEN bit
    //
    HW_WR_REG16(base + CSL_EPWM_TBCTL,
        (HW_RD_REG16(base + CSL_EPWM_TBCTL) | CSL_EPWM_TBCTL_PHSEN_MASK));
}

//*****************************************************************************
//
//! Disable phase shift load
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables loading of phase shift.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disablePhaseShiftLoad(uint32_t base)
{
    //
    // Clear PHSEN bit
    //
    HW_WR_REG16(base + CSL_EPWM_TBCTL,
        (HW_RD_REG16(base + CSL_EPWM_TBCTL) & ~CSL_EPWM_TBCTL_PHSEN_MASK));
}

//*****************************************************************************
//
//! Set time base counter mode
//!
//! \param base is the base address of the EPWM module.
//! \param counterMode is the time base counter mode.
//!
//! This function sets up the time base counter mode.
//! Valid values for counterMode are:
//!  - EPWM_COUNTER_MODE_UP          - Up - count mode.
//!  - EPWM_COUNTER_MODE_DOWN        - Down - count mode.
//!  - EPWM_COUNTER_MODE_UP_DOWN     - Up - down - count mode.
//!  - EPWM_COUNTER_MODE_STOP_FREEZE - Stop - Freeze counter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTimeBaseCounterMode(uint32_t base, EPWM_TimeBaseCountMode counterMode)
{
    //
    // Write to CTRMODE bit
    //
    HW_WR_REG16(base + CSL_EPWM_TBCTL,
        ((HW_RD_REG16(base + CSL_EPWM_TBCTL) &
        ~(CSL_EPWM_TBCTL_CTRMODE_MASK)) | ((uint16_t)counterMode)));
}

//*****************************************************************************
//
//! Set shadow to active period load on sync mode
//!
//! \param base is the base address of the EPWM module.
//! \param shadowLoadMode is the shadow to active load mode.
//!
//! This function sets up the shadow to active Period register load mode with
//! respect to a sync event. Valid values for shadowLoadMode are:
//!  - EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO - shadow to active load occurs when
//!                                         time base counter reaches 0.
//!  - EPWM_SHADOW_LOAD_MODE_COUNTER_SYNC - shadow to active load occurs when
//!                                         time base counter reaches 0 and a
//!                                         SYNC occurs.
//!  - EPWM_SHADOW_LOAD_MODE_SYNC         - shadow to active load occurs only
//!                                         when a SYNC occurs.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_selectPeriodLoadEvent(uint32_t base,
                           EPWM_PeriodShadowLoadMode shadowLoadMode)
{
    //
    // Write to PRDLDSYNC bit
    //
    HW_WR_REG16(base + CSL_EPWM_TBCTL2,
        ((HW_RD_REG16(base + CSL_EPWM_TBCTL2) &
        ~(uint16_t)(CSL_EPWM_TBCTL2_PRDLDSYNC_MASK)) |
        (uint16_t)((uint16_t)shadowLoadMode << CSL_EPWM_TBCTL2_PRDLDSYNC_SHIFT)));
}
//*****************************************************************************
//
//! Enable one shot sync mode
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables one shot sync mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableOneShotSync(uint32_t base)
{
    //
    // Set OSHTSYNCMODE bit
    //
    HW_WR_REG16(base + CSL_EPWM_TBCTL2,
        (HW_RD_REG16(base + CSL_EPWM_TBCTL2) |
        CSL_EPWM_TBCTL2_OSHTSYNCMODE_MASK));
}

//*****************************************************************************
//
//! Disable one shot sync mode
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables one shot sync mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableOneShotSync(uint32_t base)
{
    //
    // Clear OSHTSYNCMODE bit
    //
    HW_WR_REG16(base + CSL_EPWM_TBCTL2,
        (HW_RD_REG16(base + CSL_EPWM_TBCTL2) &
        ~CSL_EPWM_TBCTL2_OSHTSYNCMODE_MASK));
}

//*****************************************************************************
//
//! Start one shot sync mode
//!
//! \param base is the base address of the EPWM module.
//!
//! This function propagates a one shot sync pulse.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_startOneShotSync(uint32_t base)
{
    //
    // Set OSHTSYNC bit
    //
    HW_WR_REG16(base + CSL_EPWM_TBCTL2,
        (HW_RD_REG16(base + CSL_EPWM_TBCTL2) | CSL_EPWM_TBCTL2_OSHTSYNC_MASK));
}

//*****************************************************************************
//
//! Returns time base counter value.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the current value of the time base counter.
//!
//! \return returns time base counter value
//
//*****************************************************************************
static inline uint16_t
EPWM_getTimeBaseCounterValue(uint32_t base)
{
    //
    // Returns TBCTR value
    //
    return(HW_RD_REG16(base + CSL_EPWM_TBCTR));
}

//*****************************************************************************
//
//! Return time base counter maximum status.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the status of the time base max counter.
//!
//! \return Returns true if the counter has reached 0xFFFF.
//!         Returns false if the counter hasn't reached 0xFFFF.
//
//*****************************************************************************
static inline bool
EPWM_getTimeBaseCounterOverflowStatus(uint32_t base)
{
    //
    // Return true if CTRMAX bit is set, false otherwise
    //
    return(((HW_RD_REG16(base + CSL_EPWM_TBSTS) &
            CSL_EPWM_TBSTS_CTRMAX_MASK) ==
            CSL_EPWM_TBSTS_CTRMAX_MASK) ? true : false);
}

//*****************************************************************************
//
//! Clear max time base counter event.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function clears the max time base counter latch event. The latch event
//! occurs when the time base counter reaches its maximum value of 0xFFFF.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_clearTimeBaseCounterOverflowEvent(uint32_t base)
{
    //
    // Set CTRMAX bit
    //
    HW_WR_REG16(base + CSL_EPWM_TBSTS,
        (HW_RD_REG16(base + CSL_EPWM_TBSTS) | CSL_EPWM_TBSTS_CTRMAX_MASK));
}

//*****************************************************************************
//
//! Return external sync signal status.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the external sync signal status.
//!
//! \return Returns true if if an external sync signal event
//!         Returns false if there is no event.
//
//*****************************************************************************
static inline bool
EPWM_getSyncStatus(uint32_t base)
{
    //
    // Return true if SYNCI bit is set, false otherwise
    //
    return(((HW_RD_REG16(base + CSL_EPWM_TBSTS) & CSL_EPWM_TBSTS_SYNCI_MASK) ==
            CSL_EPWM_TBSTS_SYNCI_MASK) ? true : false);
}

//*****************************************************************************
//
//! Clear external sync signal event.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function clears the external sync signal latch event.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_clearSyncEvent(uint32_t base)
{
    //
    // Set SYNCI bit
    //
    HW_WR_REG16(base + CSL_EPWM_TBSTS,
        (HW_RD_REG16(base + CSL_EPWM_TBSTS) | CSL_EPWM_TBSTS_SYNCI_MASK));
}

//*****************************************************************************
//
//! Return time base counter direction.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the direction of the time base counter.
//!
//! \return returns EPWM_TIME_BASE_STATUS_COUNT_UP if the counter is counting
//!                 up or EPWM_TIME_BASE_STATUS_COUNT_DOWN if the counter is
//!                 counting down.
//
//*****************************************************************************
static inline uint16_t
EPWM_getTimeBaseCounterDirection(uint32_t base)
{
    //
    // Return CTRDIR bit
    //
    return(HW_RD_REG16(base + CSL_EPWM_TBSTS) & CSL_EPWM_TBSTS_CTRDIR_MASK);
}

//*****************************************************************************
//
//! Sets the phase shift offset counter value.
//!
//! \param base is the base address of the EPWM module.
//! \param phaseCount is the phase shift count value.
//!
//! This function sets the 16 bit time-base counter phase of the ePWM relative
//! to the time-base that is supplying the synchronization input signal. Call
//! the EPWM_enablePhaseShiftLoad() function to enable loading of the
//! phaseCount phase shift value when a sync event occurs.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setPhaseShift(uint32_t base, uint16_t phaseCount)
{
    //
    // Write to TBPHS bit
    //
    HW_WR_REG32(base + CSL_EPWM_TBPHS,
        ((HW_RD_REG32(base + CSL_EPWM_TBPHS) &
        ~((uint32_t)CSL_EPWM_TBPHS_TBPHS_MASK)) |
        ((uint32_t)phaseCount << CSL_EPWM_TBPHS_TBPHS_SHIFT)));
}

//*****************************************************************************
//
//! Sets the PWM period count.
//!
//! \param base is the base address of the EPWM module.
//! \param periodCount is period count value.
//!
//! This function sets the period of the PWM count. The value of periodCount is
//! the value written to the register. User should map the desired period or
//! frequency of the waveform into the correct periodCount.
//! Invoke the function EPWM_selectPeriodLoadEvent() with the appropriate
//! parameter to set the load mode of the Period count. periodCount has a
//! maximum valid value of 0xFFFF
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTimeBasePeriod(uint32_t base, uint16_t periodCount)
{
    //
    // Write to TBPRD bit
    //
    HW_WR_REG16(base + CSL_EPWM_TBPRD, periodCount);
}

//*****************************************************************************
//
//! Gets the PWM period count.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function gets the period of the PWM count.
//!
//! \return The period count value.
//
//*****************************************************************************
static inline uint16_t
EPWM_getTimeBasePeriod(uint32_t base)
{
    //
    // Read from TBPRD bit
    //
    return(HW_RD_REG16(base + CSL_EPWM_TBPRD));
}

//*****************************************************************************
//
//! Sets the EPWM links.
//!
//! \param base is the base address of the EPWM module.
//! \param epwmLink is the ePWM instance to link with.
//! \param linkComp is the ePWM component to link.
//!
//! This function links the component defined in linkComp in the current ePWM
//! instance with the linkComp component of the ePWM instance defined by
//! epwmLink. A change (a write) in the value of linkComp component of epwmLink
//! instance, causes a change in the current ePWM linkComp component.
//! For example if the current ePWM is ePWM3 and the values of epwmLink and
//! linkComp are EPWM_LINK_WITH_EPWM_1 and EPWM_LINK_COMP_C respectively,
//! then a write to COMPC register in ePWM1, will result in a simultaneous
//! write to COMPC register in ePWM3.
//! Valid values for epwmLink are:
//!   - EPWM_LINK_WITH_EPWM_0  - link current ePWM with ePWM0
//!   - EPWM_LINK_WITH_EPWM_1  - link current ePWM with ePWM1
//!   - EPWM_LINK_WITH_EPWM_2  - link current ePWM with ePWM2
//!   - EPWM_LINK_WITH_EPWM_3  - link current ePWM with ePWM3
//!   - EPWM_LINK_WITH_EPWM_4  - link current ePWM with ePWM4
//!   - EPWM_LINK_WITH_EPWM_5  - link current ePWM with ePWM5
//!   - EPWM_LINK_WITH_EPWM_6  - link current ePWM with ePWM6
//!   - EPWM_LINK_WITH_EPWM_7  - link current ePWM with ePWM7
//!   - EPWM_LINK_WITH_EPWM_8  - link current ePWM with ePWM8
//!   - EPWM_LINK_WITH_EPWM_9  - link current ePWM with ePWM9
//!   - EPWM_LINK_WITH_EPWM_10  - link current ePWM with ePWM10
//!   - EPWM_LINK_WITH_EPWM_11  - link current ePWM with ePWM11
//!   - EPWM_LINK_WITH_EPWM_12  - link current ePWM with ePWM12
//!   - EPWM_LINK_WITH_EPWM_13  - link current ePWM with ePWM13
//!   - EPWM_LINK_WITH_EPWM_14  - link current ePWM with ePWM14
//!   - EPWM_LINK_WITH_EPWM_15  - link current ePWM with ePWM15
//!   - EPWM_LINK_WITH_EPWM_16  - link current ePWM with ePWM16
//!   - EPWM_LINK_WITH_EPWM_17  - link current ePWM with ePWM17
//!   - EPWM_LINK_WITH_EPWM_18  - link current ePWM with ePWM18
//!   - EPWM_LINK_WITH_EPWM_19  - link current ePWM with ePWM19
//!   - EPWM_LINK_WITH_EPWM_20  - link current ePWM with ePWM20
//!   - EPWM_LINK_WITH_EPWM_21  - link current ePWM with ePWM21
//!   - EPWM_LINK_WITH_EPWM_22  - link current ePWM with ePWM22
//!   - EPWM_LINK_WITH_EPWM_23  - link current ePWM with ePWM23
//!   - EPWM_LINK_WITH_EPWM_24  - link current ePWM with ePWM24
//!   - EPWM_LINK_WITH_EPWM_25  - link current ePWM with ePWM25
//!   - EPWM_LINK_WITH_EPWM_26  - link current ePWM with ePWM26
//!   - EPWM_LINK_WITH_EPWM_27  - link current ePWM with ePWM27
//!   - EPWM_LINK_WITH_EPWM_28  - link current ePWM with ePWM28
//!   - EPWM_LINK_WITH_EPWM_29  - link current ePWM with ePWM29
//!   - EPWM_LINK_WITH_EPWM_30  - link current ePWM with ePWM30
//!   - EPWM_LINK_WITH_EPWM_31  - link current ePWM with ePWM31
//!
//! Valid values for linkComp are:
//!   - EPWM_LINK_TBPRD    - link TBPRD:TBPRDHR registers
//!   - EPWM_LINK_COMP_A   - link COMPA registers
//!   - EPWM_LINK_COMP_B   - link COMPB registers
//!   - EPWM_LINK_COMP_C   - link COMPC registers
//!   - EPWM_LINK_COMP_D   - link COMPD registers
//!   - EPWM_LINK_GLDCTL2  - link GLDCTL2 registers
//!   - EPWM_LINK_DBRED    - link DBRED registers
//!   - EPWM_LINK_DBFED    - link DBFED registers
//!   - EPWM_LINK_XLOAD    - link XLOAD registers
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setupEPWMLinks(uint32_t base, EPWM_CurrentLink epwmLink,
                    EPWM_LinkComponent linkComp)
{
    uint32_t registerOffset;
    uint32_t linkComponent = (uint32_t)linkComp;

    if((linkComponent == (uint32_t)EPWM_LINK_DBRED) || (linkComponent == (uint32_t)EPWM_LINK_DBFED))
    {
        registerOffset = base + CSL_EPWM_EPWMXLINK2;
        linkComponent = (uint32_t)linkComponent - 1U;
    }
    else if (linkComponent == (uint32_t)EPWM_LINK_XLOAD)
    {
         registerOffset = base + CSL_EPWM_EPWMXLINKXLOAD;
         linkComponent = (uint32_t)linkComponent - 2U;
    }
    else
    {
        registerOffset = base + CSL_EPWM_EPWMXLINK;
    }

    //
    // Configure EPWM links
    //
    HW_WR_REG32(registerOffset,
        ((uint32_t)(HW_RD_REG32(registerOffset) &
        ~((uint32_t)CSL_EPWM_EPWMXLINK_TBPRDLINK_MASK << linkComponent)) |
        ((uint32_t)epwmLink << linkComponent)));
}

//*****************************************************************************
//
//! Sets up the Counter Compare shadow load mode
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the counter compare module.
//! \param loadMode is the shadow to active load mode.
//!
//! This function enables and sets up the counter compare shadow load mode.
//! Valid values for the variables are:
//!  - compModule
//!      - EPWM_COUNTER_COMPARE_A - counter compare A.
//!      - EPWM_COUNTER_COMPARE_B - counter compare B.
//!      - EPWM_COUNTER_COMPARE_C - counter compare C.
//!      - EPWM_COUNTER_COMPARE_D - counter compare D.
//!  - loadMode
//!      - EPWM_COMP_LOAD_ON_CNTR_ZERO - load when counter equals zero
//!      - EPWM_COMP_LOAD_ON_CNTR_PERIOD - load when counter equals period
//!      - EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD - load when counter equals
//!                                             zero or period
//!      - EPWM_COMP_LOAD_FREEZE  - Freeze shadow to active load
//!      - EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO - load when counter equals zero
//!      - EPWM_COMP_LOAD_ON_SYNC_CNTR_PERIOD -load when counter equals period
//!      - EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO_PERIOD - load when counter equals
//!                                                  zero or period
//!      - EPWM_COMP_LOAD_ON_SYNC_ONLY - load on sync only
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setCounterCompareShadowLoadMode(uint32_t base,
                                     EPWM_CounterCompareModule compModule,
                                     EPWM_CounterCompareLoadMode loadMode)
{
    uint16_t syncModeOffset;
    uint16_t loadModeOffset;
    uint16_t shadowModeOffset;
    uint32_t registerOffset;

    if((compModule == EPWM_COUNTER_COMPARE_A) ||
       (compModule == EPWM_COUNTER_COMPARE_C))
    {
        syncModeOffset = CSL_EPWM_CMPCTL_LOADASYNC_SHIFT;
        loadModeOffset = CSL_EPWM_CMPCTL_LOADAMODE_SHIFT;
        shadowModeOffset = CSL_EPWM_CMPCTL_SHDWAMODE_SHIFT;
    }
    else
    {
        syncModeOffset = CSL_EPWM_CMPCTL_LOADBSYNC_SHIFT;
        loadModeOffset = CSL_EPWM_CMPCTL_LOADBMODE_SHIFT;
        shadowModeOffset = CSL_EPWM_CMPCTL_SHDWBMODE_SHIFT;
    }

    //
    // Get the register offset.  CSL_EPWM_CMPCTL for A&B or
    // CSL_EPWM_CMPCTL2 for C&D
    //
    if((compModule == EPWM_COUNTER_COMPARE_A) ||
       (compModule == EPWM_COUNTER_COMPARE_B))
    {
        registerOffset = base + CSL_EPWM_CMPCTL;
    }
    else
    {
        registerOffset = base + CSL_EPWM_CMPCTL2;
    }

    //
    // Set the appropriate sync and load mode bits and also enable shadow
    // load mode. Shadow to active load can also be frozen.
    //
    HW_WR_REG16(registerOffset,
        ((HW_RD_REG16(registerOffset) &
        ~((CSL_EPWM_CMPCTL_LOADASYNC_MAX << syncModeOffset) |
        (CSL_EPWM_CMPCTL_LOADAMODE_MAX << loadModeOffset) |
        (CSL_EPWM_CMPCTL_SHDWAMODE_MAX << shadowModeOffset))) |
        ((((uint16_t)loadMode >> 2U) << syncModeOffset) |
        (((uint16_t)loadMode & CSL_EPWM_CMPCTL_LOADASYNC_MAX) <<
        loadModeOffset))));
}

//*****************************************************************************
//
//! Disable Counter Compare shadow load mode
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the counter compare module.
//!
//! This function disables counter compare shadow load mode.
//! Valid values for the variables are:
//!  - compModule
//!      - EPWM_COUNTER_COMPARE_A - counter compare A.
//!      - EPWM_COUNTER_COMPARE_B - counter compare B.
//!      - EPWM_COUNTER_COMPARE_C - counter compare C.
//!      - EPWM_COUNTER_COMPARE_D - counter compare D.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableCounterCompareShadowLoadMode(uint32_t base,
                                         EPWM_CounterCompareModule compModule)
{
    uint16_t shadowModeOffset;
    uint32_t registerOffset;

    if((compModule == EPWM_COUNTER_COMPARE_A) ||
       (compModule == EPWM_COUNTER_COMPARE_C))
    {
        shadowModeOffset = CSL_EPWM_CMPCTL_SHDWAMODE_SHIFT;
    }
    else
    {
        shadowModeOffset = CSL_EPWM_CMPCTL_SHDWBMODE_SHIFT;
    }

    //
    // Get the register offset.  CSL_EPWM_CMPCTL for A&B or
    // CSL_EPWM_CMPCTL2 for C&D
    //
    if((compModule == EPWM_COUNTER_COMPARE_A) ||
       (compModule == EPWM_COUNTER_COMPARE_B))
    {
        registerOffset = base + CSL_EPWM_CMPCTL;
    }
    else
    {
        registerOffset = base + CSL_EPWM_CMPCTL2;
    }

    //
    // Disable shadow load mode.
    //
    HW_WR_REG16(registerOffset,
        (HW_RD_REG16(registerOffset) |
        ((uint32_t)CSL_EPWM_CMPCTL_SHDWAMODE_MAX << shadowModeOffset)));
}

//*****************************************************************************
//
//! Set counter compare values.
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the Counter Compare value module.
//! \param compCount is the counter compare count value.
//!
//! This function sets the counter compare value for counter compare registers.
//! The maximum value for compCount is 0xFFFF.
//! Valid values for compModule are:
//!   - EPWM_COUNTER_COMPARE_A - counter compare A.
//!   - EPWM_COUNTER_COMPARE_B - counter compare B.
//!   - EPWM_COUNTER_COMPARE_C - counter compare C.
//!   - EPWM_COUNTER_COMPARE_D - counter compare D.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setCounterCompareValue(uint32_t base, EPWM_CounterCompareModule compModule,
                            uint16_t compCount)
{
    uint32_t registerOffset;

    //
    // Get the register offset for the Counter compare
    //
    registerOffset = base + CSL_EPWM_CMPA + (uint16_t)compModule;

    //
    // Write to the counter compare registers.
    //
    if((compModule == EPWM_COUNTER_COMPARE_A) ||
        (compModule == EPWM_COUNTER_COMPARE_B))
    {
        //
        // Write to COMPA or COMPB bits
        //
        HW_WR_REG16(registerOffset + 0x2U, compCount);
    }
    else
    {
        //
        // Write to COMPC or COMPD bits
        //
        HW_WR_REG16(registerOffset, compCount);
    }
}
//*****************************************************************************
//
//! Set counter compare A value.
//!
//! \param base is the base address of the EPWM module.
//! \param compCount is the counter compare count value.
//!
//! This function sets the counter compare value for counter compare A register.
//! The maximum value for compCount is 0xFFFF.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setCounterCompareValue_opt_cmpA(uint32_t base, uint16_t compCount)
{
    HW_WR_REG16(base + CSL_EPWM_CMPA + 0x2U, compCount);
}

//*****************************************************************************
//
//! Set counter compare B value.
//!
//! \param base is the base address of the EPWM module.
//! \param compCount is the counter compare count value.
//!
//! This function sets the counter compare value for counter compare B register.
//! The maximum value for compCount is 0xFFFF.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setCounterCompareValue_opt_cmpB(uint32_t base, uint16_t compCount)
{
    HW_WR_REG16(base + CSL_EPWM_CMPB + 0x2U, compCount);
}

//*****************************************************************************
//
//! Set counter compare C value.
//!
//! \param base is the base address of the EPWM module.
//! \param compCount is the counter compare count value.
//!
//! This function sets the counter compare value for counter compare C register.
//! The maximum value for compCount is 0xFFFF.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setCounterCompareValue_opt_cmpC(uint32_t base, uint16_t compCount)
{
    HW_WR_REG16(base + CSL_EPWM_CMPC, compCount);
}

//*****************************************************************************
//
//! Set counter compare D value.
//!
//! \param base is the base address of the EPWM module.
//! \param compCount is the counter compare count value.
//!
//! This function sets the counter compare value for counter compare D register.
//! The maximum value for compCount is 0xFFFF.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setCounterCompareValue_opt_cmpD(uint32_t base, uint16_t compCount)
{
    HW_WR_REG16(base + CSL_EPWM_CMPD, compCount);
}

//*****************************************************************************
//
//! Get counter compare values.
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the Counter Compare value module.
//!
//! This function gets the counter compare value for counter compare registers.
//! Valid values for compModule are:
//!   - EPWM_COUNTER_COMPARE_A - counter compare A.
//!   - EPWM_COUNTER_COMPARE_B - counter compare B.
//!   - EPWM_COUNTER_COMPARE_C - counter compare C.
//!   - EPWM_COUNTER_COMPARE_D - counter compare D.
//!
//! \return The counter compare count value.
//
//*****************************************************************************
static inline uint16_t
EPWM_getCounterCompareValue(uint32_t base, EPWM_CounterCompareModule compModule)
{
    uint32_t registerOffset;
    uint16_t compCount;

    //
    // Get the register offset for the Counter compare
    //
    registerOffset = base + CSL_EPWM_CMPA + (uint16_t)compModule;

    //
    // Read from the counter compare registers.
    //
    if((compModule == EPWM_COUNTER_COMPARE_A) ||
        (compModule == EPWM_COUNTER_COMPARE_B))
    {
        //
        // Read COMPA or COMPB bits
        //
        compCount = (uint16_t)((HW_RD_REG32(registerOffset) &
                    (uint32_t)CSL_EPWM_CMPA_CMPA_MASK) >>
                    CSL_EPWM_CMPA_CMPA_SHIFT);
    }
    else
    {
        //
        // Read COMPC or COMPD bits
        //
        compCount = HW_RD_REG16(registerOffset);
    }
    return(compCount);
}

//*****************************************************************************
//
//! Return the counter compare shadow register full status.
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the Counter Compare value module.
//!
//! This function returns the counter Compare shadow register full status flag.
//! Valid values for compModule are:
//!   - EPWM_COUNTER_COMPARE_A - counter compare A.
//!   - EPWM_COUNTER_COMPARE_B - counter compare B.
//!
//! \return Returns true if the shadow register is full.
//!         Returns false if the shadow register is not full.
//
//*****************************************************************************
static inline bool
EPWM_getCounterCompareShadowStatus(uint32_t base,
                                   EPWM_CounterCompareModule compModule)
{
    //
    // Check the arguments
    //
    DebugP_assert((compModule == EPWM_COUNTER_COMPARE_A) ||
                  (compModule == EPWM_COUNTER_COMPARE_B));

    //
    // Read the value of SHDWAFULL or SHDWBFULL bit
    //
    return((((HW_RD_REG32(base + CSL_EPWM_CMPCTL) >>
            ((((uint16_t)compModule >> 1U) & 0x2U) +
            CSL_EPWM_CMPCTL_SHDWAFULL_SHIFT)) &
            0x1U) == 0x1U) ? true:false);
}

//
// Action Qualifier module related APIs
//
//*****************************************************************************
//
//! Sets the Action Qualifier shadow load mode
//!
//! \param base is the base address of the EPWM module.
//! \param aqModule is the Action Qualifier module value.
//! \param loadMode is the shadow to active load mode.
//!
//! This function enables and sets the Action Qualifier shadow load mode.
//! Valid values for the variables are:
//!  - aqModule
//!      - EPWM_ACTION_QUALIFIER_A - Action Qualifier A.
//!      - EPWM_ACTION_QUALIFIER_B - Action Qualifier B.
//!  - loadMode
//!      - EPWM_AQ_LOAD_ON_CNTR_ZERO - load when counter equals zero
//!      - EPWM_AQ_LOAD_ON_CNTR_PERIOD - load when counter equals period
//!      - EPWM_AQ_LOAD_ON_CNTR_ZERO_PERIOD - load when counter equals
//!                                               zero or period
//!      - EPWM_AQ_LOAD_FREEZE  - Freeze shadow to active load
//!      - EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO - load on sync or when counter
//!                                          equals zero
//!      - EPWM_AQ_LOAD_ON_SYNC_CNTR_PERIOD - load on sync or when counter
//!                                           equals period
//!      - EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO_PERIOD - load on sync or when
//!                                               counter equals zero or period
//!      - EPWM_AQ_LOAD_ON_SYNC_ONLY - load on sync only
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierShadowLoadMode(uint32_t base,
                                      EPWM_ActionQualifierModule aqModule,
                                      EPWM_ActionQualifierLoadMode loadMode)
{
    uint16_t syncModeOffset;
    uint16_t shadowModeOffset;

    syncModeOffset = CSL_EPWM_AQCTL_LDAQASYNC_SHIFT + (uint16_t)aqModule;
    shadowModeOffset = CSL_EPWM_AQCTL_SHDWAQAMODE_SHIFT + (uint16_t)aqModule;

    //
    // Set the appropriate sync and load mode bits and also enable shadow
    // load mode. Shadow to active load can also be frozen.
    //

    HW_WR_REG16((base + CSL_EPWM_AQCTL),
        ((HW_RD_REG16(base + CSL_EPWM_AQCTL) &
        (~((CSL_EPWM_AQCTL_LDAQAMODE_MASK << (uint16_t)aqModule) |
        (CSL_EPWM_AQCTL_LDAQASYNC_MAX << (uint16_t)syncModeOffset))) |
        (CSL_EPWM_AQCTL_SHDWAQAMODE_MAX << shadowModeOffset)) |
        ((((uint16_t)loadMode >> 2U) << syncModeOffset) |
        (((uint16_t)loadMode & CSL_EPWM_AQCTL_LDAQAMODE_MASK) <<
        (uint16_t)aqModule))));
}

//*****************************************************************************
//
//! Disable Action Qualifier shadow load mode
//!
//! \param base is the base address of the EPWM module.
//! \param aqModule is the Action Qualifier module value.
//!
//! This function disables the Action Qualifier  shadow load mode.
//! Valid values for the variables are:
//!  - aqModule
//!      - EPWM_ACTION_QUALIFIER_A - Action Qualifier A.
//!      - EPWM_ACTION_QUALIFIER_B - Action Qualifier B.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableActionQualifierShadowLoadMode(uint32_t base,
                                          EPWM_ActionQualifierModule aqModule)
{
    uint16_t shadowModeOffset;

    shadowModeOffset = CSL_EPWM_AQCTL_SHDWAQAMODE_SHIFT + (uint16_t)aqModule;

    //
    // Disable shadow load mode. Action qualifier is loaded on
    // immediate mode only.
    //
    HW_WR_REG16(base + CSL_EPWM_AQCTL,
        (HW_RD_REG16(base + CSL_EPWM_AQCTL) &
        ~(CSL_EPWM_AQCTL_SHDWAQAMODE_MAX << shadowModeOffset)));
}

//*****************************************************************************
//
//! Set up Action qualifier trigger source for event T1
//!
//! \param base is the base address of the EPWM module.
//! \param trigger sources for Action Qualifier triggers.
//!
//! This function sets up the sources for Action Qualifier event T1.
//! Valid values for trigger are:
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1       - Digital compare event A 1
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_2       - Digital compare event A 2
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_1       - Digital compare event B 1
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_2       - Digital compare event B 2
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_1        - Trip zone 1
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_2        - Trip zone 2
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_3        - Trip zone 3
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_EPWM_SYNCIN - EPWM sync
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DC_EVTFILT  - Digital compare filter event
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierT1TriggerSource(uint32_t base,
                                     EPWM_ActionQualifierTriggerSource trigger)
{
    //
    // Set T1 trigger source
    //
    HW_WR_REG16(base + CSL_EPWM_AQTSRCSEL,
        ((HW_RD_REG16(base + CSL_EPWM_AQTSRCSEL) &
        (~CSL_EPWM_AQTSRCSEL_T1SEL_MASK)) |
        ((uint16_t)trigger)));
}

//*****************************************************************************
//
//! Set up Action qualifier trigger source for event T2
//!
//! \param base is the base address of the EPWM module.
//! \param trigger sources for Action Qualifier triggers.
//!
//! This function sets up the sources for Action Qualifier event T2.
//! Valid values for trigger are:
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1       - Digital compare event A 1
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_2       - Digital compare event A 2
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_1       - Digital compare event B 1
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_2       - Digital compare event B 2
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_1        - Trip zone 1
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_2        - Trip zone 2
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_3        - Trip zone 3
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_EPWM_SYNCIN - ePWM sync
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DC_EVTFILT  - Digital compare filter event
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierT2TriggerSource(uint32_t base,
                                      EPWM_ActionQualifierTriggerSource trigger)
{
    //
    // Set T2 trigger source
    //
    HW_WR_REG16(base + CSL_EPWM_AQTSRCSEL,
        ((HW_RD_REG16(base + CSL_EPWM_AQTSRCSEL) &
        (~(uint16_t)CSL_EPWM_AQTSRCSEL_T2SEL_MASK)) |
        (uint16_t)((uint16_t)trigger << CSL_EPWM_AQTSRCSEL_T2SEL_SHIFT)));
}

//*****************************************************************************
//
//! Set up Action qualifier outputs
//!
//! \param base is the base address of the EPWM module.
//! \param epwmOutput is the ePWM pin type.
//! \param output is the Action Qualifier output.
//! \param event is the event that causes a change in output.
//!
//! This function sets up the Action Qualifier output on ePWM A or ePWMB,
//! depending on the value of epwmOutput, to a value specified by outPut based
//! on the input events - specified by event.
//! The following are valid values for the parameters.
//!   - epwmOutput
//!       - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!       - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!   - output
//!       - EPWM_AQ_OUTPUT_NO_CHANGE  - No change in the output pins
//!       - EPWM_AQ_OUTPUT_LOW        - Set output pins to low
//!       - EPWM_AQ_OUTPUT_HIGH       - Set output pins to High
//!       - EPWM_AQ_OUTPUT_TOGGLE     - Toggle the output pins
//!   - event
//!       - EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO       - Time base counter equals
//!                                                 zero
//!       - EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD     - Time base counter equals
//!                                                 period
//!       - EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA    - Time base counter up equals
//!                                                 COMPA
//!       - EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA  - Time base counter down
//!                                                 equals COMPA
//!       - EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB    - Time base counter up equals
//!                                                 COMPB
//!       - EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB  - Time base counter down
//!                                                 equals COMPB
//!       - EPWM_AQ_OUTPUT_ON_T1_COUNT_UP         - T1 event on count up
//!       - EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN       - T1 event on count down
//!       - EPWM_AQ_OUTPUT_ON_T2_COUNT_UP         - T2 event on count up
//!       - EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN       - T2 event on count down
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierAction(uint32_t base,
                              EPWM_ActionQualifierOutputModule epwmOutput,
                              EPWM_ActionQualifierOutput output,
                              EPWM_ActionQualifierOutputEvent event)
{
    uint32_t registerOffset;
    uint32_t registerTOffset;

    //
    // Get the register offset
    //
    registerOffset = (uint32_t)CSL_EPWM_AQCTLA + (uint16_t)epwmOutput;
    registerTOffset = (uint32_t)CSL_EPWM_AQCTLA2 + (uint16_t)epwmOutput;

    //
    // If the event occurs on T1 or T2 events
    //
    if(((uint16_t)event & 0x1U) == 1U)
    {
        //
        // Write to T1U,T1D,T2U or T2D of AQCTLA2 register
        //
        HW_WR_REG16(base + registerTOffset,
            ((HW_RD_REG16(base + registerTOffset) &
            ~(CSL_EPWM_AQCTLA_ZRO_MAX << ((uint16_t)event - 1U))) |
            ((uint16_t)output << ((uint16_t)event - 1U))));
    }
    else
    {
        //
        // Write to ZRO,PRD,CAU,CAD,CBU or CBD bits of AQCTLA register
        //
        HW_WR_REG16(base + registerOffset,
            ((HW_RD_REG16(base + registerOffset) &
            ~(CSL_EPWM_AQCTLA_ZRO_MAX << (uint16_t)event)) |
            ((uint16_t)output << (uint16_t)event)));
    }
}

//*****************************************************************************
//
//! Set up Action qualifier event outputs
//!
//! \param base is the base address of the EPWM module.
//! \param epwmOutput is the ePWM pin type.
//! \param action is the desired action when the specified event occurs
//!
//! This function sets up the Action Qualifier output on ePWMA or ePWMB,
//! depending on the value of epwmOutput, to a value specified by action.
//! Valid action param values from different time base counter scenarios
//! should be OR'd together to configure complete action for a pwm output.
//! The following are valid values for the parameters.
//!   - epwmOutput
//!       - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!       - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!
//!   - action
//!       - When time base counter equals zero
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_ZERO   - Time base counter equals zero
//!                                             and no change in output pins
//!         - EPWM_AQ_OUTPUT_LOW_ZERO         - Time base counter equals zero
//!                                             and set output pins to low
//!         - EPWM_AQ_OUTPUT_HIGH_ZERO        - Time base counter equals zero
//!                                             and set output pins to high
//!         - EPWM_AQ_OUTPUT_TOGGLE_ZERO      - Time base counter equals zero
//!                                             and toggle the output pins
//!       - When time base counter equals period
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_PERIOD - Time base counter equals period
//!                                             and no change in output pins
//!         - EPWM_AQ_OUTPUT_LOW_PERIOD       - Time base counter equals period
//!                                             and set output pins to low
//!         - EPWM_AQ_OUTPUT_HIGH_PERIOD      - Time base counter equals period
//!                                             and set output pins to high
//!         - EPWM_AQ_OUTPUT_TOGGLE_PERIOD    - Time base counter equals period
//!                                             and toggle the output pins
//!       - When time base counter equals CMPA during up-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_UP_CMPA  - Time base counter up equals
//!                                               COMPA and no change in the
//!                                               output pins
//!         - EPWM_AQ_OUTPUT_LOW_UP_CMPA        - Time base counter up equals
//!                                               COMPA and set output pins low
//!         - EPWM_AQ_OUTPUT_HIGH_UP_CMPA      - Time base counter up equals
//!                                              COMPA and set output pins high
//!         - EPWM_AQ_OUTPUT_TOGGLE_UP_CMPA     - Time base counter up equals
//!                                               COMPA and toggle output pins
//!       - When time base counter equals CMPA during down-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_CMPA - Timebase counter down equals
//!                                                COMPA and no change in the
//!                                                output pins
//!         - EPWM_AQ_OUTPUT_LOW_DOWN_CMPA      - Time base counter down equals
//!                                               COMPA and set output pins low
//!         - EPWM_AQ_OUTPUT_HIGH_DOWN_CMPA    - Time base counter down equals
//!                                              COMPA and set output pins high
//!         - EPWM_AQ_OUTPUT_TOGGLE_DOWN_CMPA   - Time base counter down equals
//!                                               COMPA and toggle output pins
//!       - When time base counter equals CMPB during up-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_UP_CMPB  - Time base counter up equals
//!                                               COMPB and no change in the
//!                                               output pins
//!         - EPWM_AQ_OUTPUT_LOW_UP_CMPB        - Time base counter up equals
//!                                               COMPB and set output pins low
//!         - EPWM_AQ_OUTPUT_HIGH_UP_CMPB      - Time base counter up equals
//!                                              COMPB and set output pins high
//!         - EPWM_AQ_OUTPUT_TOGGLE_UP_CMPB     - Time base counter up equals
//!                                               COMPB and toggle output pins
//!       - When time base counter equals CMPB during down-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_CMPB- Time base counter down equals
//!                                               COMPB and no change in the
//!                                               output pins
//!         - EPWM_AQ_OUTPUT_LOW_DOWN_CMPB      - Time base counter down equals
//!                                               COMPB and set output pins low
//!         - EPWM_AQ_OUTPUT_HIGH_DOWN_CMPB    - Time base counter down equals
//!                                              COMPB and set output pins high
//!         - EPWM_AQ_OUTPUT_TOGGLE_DOWN_CMPB   - Time base counter down equals
//!                                               COMPB and toggle output pins
//!
//! \b note:  A logical OR of the valid values should be passed as the action
//!           parameter. Single action should be configured for each time base
//!           counter scenario.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierActionComplete(uint32_t base,
                                EPWM_ActionQualifierOutputModule epwmOutput,
                                EPWM_ActionQualifierEventAction action)
{
    uint32_t registerOffset;

    //
    // Get the register offset
    //
    registerOffset = (uint32_t)CSL_EPWM_AQCTLA + (uint16_t)epwmOutput;

    //
    // Write to ZRO, PRD, CAU, CAD, CBU or CBD bits of AQCTLA register
    //
    HW_WR_REG16(base + registerOffset, action);
}

//*****************************************************************************
//
//! Set up Additional action qualifier event outputs
//!
//! \param base is the base address of the EPWM module.
//! \param epwmOutput is the ePWM pin type.
//! \param action is the desired action when the specified event occurs
//!
//! This function sets up the Additional Action Qualifier output on ePWMA or
//! ePWMB depending on the value of epwmOutput, to a value specified by action.
//! Valid action param values from different event scenarios should be OR'd
//! together to configure complete action for a pwm output.
//! The following are valid values for the parameters.
//!   - epwmOutput
//!       - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!       - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!   - action
//!       - When T1 event occurs during up-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_UP_T1  - T1 event on count up
//!                                             and no change in output pins
//!         - EPWM_AQ_OUTPUT_LOW_UP_T1        - T1 event on count up
//!                                           and set output pins to low
//!         - EPWM_AQ_OUTPUT_HIGH_UP_T1       - T1 event on count up
//!                                           and set output pins to high
//!         - EPWM_AQ_OUTPUT_TOGGLE_UP_T1     - T1 event on count up
//!                                           and toggle the output pins
//!       - When T1 event occurs during down-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_T1- T1 event on count down
//!                                           and no change in output pins
//!         - EPWM_AQ_OUTPUT_LOW_DOWN_T1      - T1 event on count down
//!                                           and set output pins to low
//!         - EPWM_AQ_OUTPUT_HIGH_DOWN_T1     - T1 event on count down
//!                                           and set output pins to high
//!         - EPWM_AQ_OUTPUT_TOGGLE_DOWN_T1   - T1 event on count down
//!                                           and toggle the output pins
//!       - When T2 event occurs during up-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_UP_T2  - T2 event on count up
//!                                             and no change in output pins
//!         - EPWM_AQ_OUTPUT_LOW_UP_T2        - T2 event on count up
//!                                             and set output pins to low
//!         - EPWM_AQ_OUTPUT_HIGH_UP_T2       - T2 event on count up
//!                                             and set output pins to high
//!         - EPWM_AQ_OUTPUT_TOGGLE_UP_T2     - T2 event on count up
//!                                             and toggle the output pins
//!       - When T2 event occurs during down-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_T2 - T2 event on count down
//!                                              and no change in output pins
//!         - EPWM_AQ_OUTPUT_LOW_DOWN_T2       - T2 event on count down
//!                                              and set output pins to low
//!         - EPWM_AQ_OUTPUT_HIGH_DOWN_T2      - T2 event on count down
//!                                              and set output pins to high
//!         - EPWM_AQ_OUTPUT_TOGGLE_DOWN_T2    - T2 event on count down
//!                                              and toggle the output pins
//!
//! \b note:  A logical OR of the valid values should be passed as the action
//!           parameter. Single action should be configured for each event
//!           scenario.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setAdditionalActionQualifierActionComplete(uint32_t base,
                            EPWM_ActionQualifierOutputModule epwmOutput,
                            EPWM_AdditionalActionQualifierEventAction action)
{
    uint32_t registerTOffset;

    //
    // Get the register offset
    //
    registerTOffset = (uint32_t)CSL_EPWM_AQCTLA2 + (uint16_t)epwmOutput;

    //
    // Write to T1U, T1D, T2U or T2D of AQCTLA2 register
    //
    HW_WR_REG16(base + registerTOffset, action);
}

//*****************************************************************************
//
//! Sets up Action qualifier continuous software load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param mode is the mode for shadow to active load mode.
//!
//! This function sets up the AQCFRSC register load mode for continuous
//! software force reload mode. The software force actions are determined by
//! the EPWM_setActionQualifierContSWForceAction() function.
//! Valid values for mode are:
//!   - EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO        - shadow mode load when counter
//!                                               equals zero
//!   - EPWM_AQ_SW_SH_LOAD_ON_CNTR_PERIOD      - shadow mode load when counter
//!                                               equals period
//!   - EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO_PERIOD - shadow mode load when counter
//!                                               equals zero or period
//!   - EPWM_AQ_SW_IMMEDIATE_LOAD               - immediate mode load only
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierContSWForceShadowMode(uint32_t base,
                                            EPWM_ActionQualifierContForce mode)
{
    //
    // Set the Action qualifier software action reload mode.
    // Write to RLDCSF bit
    //
    HW_WR_REG16(base + CSL_EPWM_AQSFRC,
        ((HW_RD_REG16(base + CSL_EPWM_AQSFRC) &
        ~(uint16_t)CSL_EPWM_AQSFRC_RLDCSF_MASK) |
        (uint16_t)((uint16_t)mode << CSL_EPWM_AQSFRC_RLDCSF_SHIFT)));
}

//*****************************************************************************
//
//! Triggers a continuous software forced event.
//!
//! \param base is the base address of the EPWM module.
//! \param epwmOutput is the ePWM pin type.
//! \param output is the Action Qualifier output.
//!
//! This function triggers a continuous software forced Action Qualifier output
//! on ePWM A or B based on the value of epwmOutput.
//! Valid values for the parameters are:
//!   - epwmOutput
//!       - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!       - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!   - output
//!       - EPWM_AQ_SW_DISABLED       - Software forcing disabled.
//!       - EPWM_AQ_SW_OUTPUT_LOW     - Set output pins to low
//!       - EPWM_AQ_SW_OUTPUT_HIGH    - Set output pins to High
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierContSWForceAction(uint32_t base,
                                EPWM_ActionQualifierOutputModule epwmOutput,
                                EPWM_ActionQualifierSWOutput output)
{
    //
    // Initiate a continuous software forced output
    //
    if(epwmOutput == EPWM_AQ_OUTPUT_A)
    {
        HW_WR_REG16(base + CSL_EPWM_AQCSFRC,
            ((HW_RD_REG16(base + CSL_EPWM_AQCSFRC) &
            ~CSL_EPWM_AQCSFRC_CSFA_MASK) |
            ((uint16_t)output)));
    }
    else
    {
        HW_WR_REG16(base + CSL_EPWM_AQCSFRC,
            ((HW_RD_REG16(base + CSL_EPWM_AQCSFRC) &
            ~(uint16_t)CSL_EPWM_AQCSFRC_CSFB_MASK) |
            (uint16_t)((uint16_t)output << CSL_EPWM_AQCSFRC_CSFB_SHIFT)));
    }
}

//*****************************************************************************
//
//! Triggers a continuous software forced event.
//!
//! \param base is the base address of the EPWM module.
//! \param outputAB is the Action Qualifier output.
//!
//! This function triggers a continuous software forced Action Qualifier output
//! on ePWM A and B based on the value of outputAB.
//! Valid values for the parameters are:
//!   - outputAB
//!     - EPWM_AQ_A_SW_DISABLED_B_SW_DISABLED        - software force disabled on Outputs A & B
//!     - EPWM_AQ_A_SW_OUTPUT_LOW_B_SW_DISABLED      - software force Low Action on Output A,
//!                                                     software force disabled Output B
//!     - EPWM_AQ_A_SW_OUTPUT_HIGH_B_SW_DISABLED     - software force High Action on Output A,
//!                                                     software force disabled Output B
//!     - EPWM_AQ_A_SW_DISABLED_B_SW_OUTPUT_LOW      - software force disabled on Output A,
//!                                                     software force Low Action on Output B
//!     - EPWM_AQ_A_SW_OUTPUT_LOW_B_SW_OUTPUT_LOW    - software force Low Action on Output A,
//!                                                     software force Low Action on Output B
//!     - EPWM_AQ_A_SW_OUTPUT_HIGH_B_SW_OUTPUT_LOW   - software force High Action on Output A,
//!                                                     software force Low Action on Output B
//!     - EPWM_AQ_A_SW_DISABLED_B_SW_OUTPUT_HIGH     - software force disabled on Output A,
//!                                                     software force High Action on Output B
//!     - EPWM_AQ_A_SW_OUTPUT_LOW_B_SW_OUTPUT_HIGH   - software force Low Action on Output A,
//!                                                     software force High Action on Output B
//!     - EPWM_AQ_A_SW_OUTPUT_HIGH_B_SW_OUTPUT_HIGH  - software force High Action on Output A,
//!                                                         software force High Action on Output B
//! \return None.
//! \note this API overwrites the last 8 bits of the register. make sure if any
//!       to the lower 8 bits of the CSL_EPWM_AQCSFRC register.
//*****************************************************************************
static inline void
EPWM_setActionQualifierContSWForceAction_opt_outputs(uint32_t base, uint8_t outputAB)
{
    HW_WR_REG16(base + CSL_EPWM_AQCSFRC, outputAB);
}

//*****************************************************************************
//
//! Set up one time software forced Action qualifier outputs
//!
//! \param base is the base address of the EPWM module.
//! \param epwmOutput is the ePWM pin type.
//! \param output is the Action Qualifier output.
//!
//! This function sets up the one time software forced Action Qualifier output
//! on ePWM A or ePWMB, depending on the value of epwmOutput to a value
//! specified by outPut.
//! The following are valid values for the parameters.
//!   - epwmOutput
//!       - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!       - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!   - output
//!       - EPWM_AQ_OUTPUT_NO_CHANGE  - No change in the output pins
//!       - EPWM_AQ_OUTPUT_LOW        - Set output pins to low
//!       - EPWM_AQ_OUTPUT_HIGH       - Set output pins to High
//!       - EPWM_AQ_OUTPUT_TOGGLE     - Toggle the output pins
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierSWAction(uint32_t base,
                                EPWM_ActionQualifierOutputModule epwmOutput,
                                EPWM_ActionQualifierOutput output)
{
    //
    // Set the one time software forced action
    //
    if(epwmOutput == EPWM_AQ_OUTPUT_A)
    {
        HW_WR_REG16(base + CSL_EPWM_AQSFRC,
            ((HW_RD_REG16(base + CSL_EPWM_AQSFRC) &
            ~CSL_EPWM_AQSFRC_ACTSFA_MASK) |
            ((uint16_t)output)));
    }
    else
    {
        HW_WR_REG16(base + CSL_EPWM_AQSFRC,
            ((HW_RD_REG16(base + CSL_EPWM_AQSFRC) &
            ~(uint16_t)CSL_EPWM_AQSFRC_ACTSFB_MASK) |
            (uint16_t)((uint16_t)output << CSL_EPWM_AQSFRC_ACTSFB_SHIFT)));
    }
}

//*****************************************************************************
//
//! Triggers a one time software forced event on Action qualifier
//!
//! \param base is the base address of the EPWM module.
//! \param epwmOutput is the ePWM pin type.
//!
//! This function triggers a one time software forced Action Qualifier event
//! on ePWM A or B based on the value of epwmOutput.
//! Valid values for epwmOutput are:
//!   - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!   - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_forceActionQualifierSWAction(uint32_t base,
                                  EPWM_ActionQualifierOutputModule epwmOutput)
{
    //
    // Initiate a software forced event
    //
    if(epwmOutput == EPWM_AQ_OUTPUT_A)
    {
        HW_WR_REG16(base + CSL_EPWM_AQSFRC,
            (HW_RD_REG16(base + CSL_EPWM_AQSFRC) |
            CSL_EPWM_AQSFRC_OTSFA_MASK));
    }
    else
    {
        HW_WR_REG16(base + CSL_EPWM_AQSFRC,
            (HW_RD_REG16(base + CSL_EPWM_AQSFRC) |
            CSL_EPWM_AQSFRC_OTSFB_MASK));
    }
}

//
// Dead Band Module related APIs
//
//*****************************************************************************
//
//! Sets Dead Band signal output swap mode.
//!
//! \param base is the base address of the EPWM module.
//! \param output is the ePWM Dead Band output.
//! \param enableSwapMode is the output swap mode.
//!
//! This function sets up the output signal swap mode. For example if the
//! output variable is set to EPWM_DB_OUTPUT_A and enableSwapMode is true, then
//! the ePWM A output gets its signal from the ePWM B signal path. Valid values
//! for the input variables are:
//!  - output
//!      - EPWM_DB_OUTPUT_A   - ePWM output A
//!      - EPWM_DB_OUTPUT_B   - ePWM output B
//!  - enableSwapMode
//!      - true     - the output is swapped
//!      - false    - the output and the signal path are the same.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setDeadBandOutputSwapMode(uint32_t base, EPWM_DeadBandOutput output,
                               bool enableSwapMode)
{
    uint16_t mask;

    mask = (uint16_t)1U << ((uint16_t)output + CSL_EPWM_DBCTL_OUTSWAP_SHIFT);

    if(enableSwapMode)
    {
        //
        // Set the appropriate outswap bit to swap output
        //
        HW_WR_REG16(base + CSL_EPWM_DBCTL,
            (HW_RD_REG16(base + CSL_EPWM_DBCTL) | mask));
    }
    else
    {
        //
        // Clear the appropriate outswap bit to disable output swap
        //
        HW_WR_REG16(base + CSL_EPWM_DBCTL,
            (HW_RD_REG16(base + CSL_EPWM_DBCTL) & ~mask));
    }
}

//*****************************************************************************
//
//! Sets Dead Band signal output mode.
//!
//! \param base is the base address of the EPWM module.
//! \param delayMode is the Dead Band delay type.
//! \param enableDelayMode is the dead band delay mode.
//!
//! This function sets up the dead band delay mode. The delayMode variable
//! determines if the applied delay is Rising Edge or Falling Edge. The
//! enableDelayMode determines if a dead band delay should be applied.
//! Valid values for the variables are:
//!  - delayMode
//!      - EPWM_DB_RED   - Rising Edge delay
//!      - EPWM_DB_FED   - Falling Edge delay
//!  - enableDelayMode
//!      - true     - Falling edge or Rising edge delay is applied.
//!      - false    - Dead Band delay is bypassed.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setDeadBandDelayMode(uint32_t base, EPWM_DeadBandDelayMode delayMode,
                          bool enableDelayMode)
{
    uint16_t mask;

    mask = (uint16_t)1U << ((uint16_t)((uint16_t)delayMode + (uint16_t)CSL_EPWM_DBCTL_OUT_MODE_SHIFT));

    if(enableDelayMode)
    {
        //
        // Set the appropriate outmode bit to enable Dead Band delay
        //
        HW_WR_REG16(base + CSL_EPWM_DBCTL,
            (HW_RD_REG16(base + CSL_EPWM_DBCTL) | mask));
    }
    else
    {
        //
        // Clear the appropriate outswap bit to disable output swap
        //
        HW_WR_REG16(base + CSL_EPWM_DBCTL,
            (HW_RD_REG16(base + CSL_EPWM_DBCTL) & ~mask));
    }
}

//*****************************************************************************
//
//! Sets Dead Band delay polarity.
//!
//! \param base is the base address of the EPWM module.
//! \param delayMode is the Dead Band delay type.
//! \param polarity is the polarity of the delayed signal.
//!
//! This function sets up the polarity as determined by the variable polarity
//! of the Falling Edge or Rising Edge delay depending on the value of
//! delayMode. Valid values for the variables are:
//!   - delayMode
//!       - EPWM_DB_RED   - Rising Edge delay
//!       - EPWM_DB_FED   - Falling Edge delay
//!   - polarity
//!       - EPWM_DB_POLARITY_ACTIVE_HIGH   - polarity is not inverted.
//!       - EPWM_DB_POLARITY_ACTIVE_LOW    - polarity is inverted.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setDeadBandDelayPolarity(uint32_t base,
                              EPWM_DeadBandDelayMode delayMode,
                              EPWM_DeadBandPolarity polarity)
{
    uint16_t shift;

    shift = (((uint16_t)delayMode ^ 0x1U) + CSL_EPWM_DBCTL_POLSEL_SHIFT);

    //
    // Set the appropriate polsel bits for dead band polarity
    //
    HW_WR_REG16(base + CSL_EPWM_DBCTL,
        ((HW_RD_REG16(base + CSL_EPWM_DBCTL) & ~ (1U << shift)) |
        ((uint16_t)polarity << shift)));
}

//*****************************************************************************
//
//! Sets Rising Edge Dead Band delay input.
//!
//! \param base is the base address of the EPWM module.
//! \param input is the input signal to the dead band.
//!
//! This function sets up the rising Edge delay input signal.
//! Valid values for input are:
//!     - EPWM_DB_INPUT_EPWMA   - Input signal is ePWMA( Valid for both Falling
//!                                  Edge and Rising Edge)
//!     - EPWM_DB_INPUT_EPWMB   - Input signal is ePWMB( Valid for both Falling
//!                                  Edge and Rising Edge)
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setRisingEdgeDeadBandDelayInput(uint32_t base, uint16_t input)
{
    //
    // Check the arguments
    //
    DebugP_assert((input == EPWM_DB_INPUT_EPWMA) ||
           (input == EPWM_DB_INPUT_EPWMB));

    //
    // Set the Rising Edge Delay input
    //
    HW_WR_REG16(base + CSL_EPWM_DBCTL,
        ((HW_RD_REG16(base + CSL_EPWM_DBCTL) &
        ~(1U << (CSL_EPWM_DBCTL_IN_MODE_SHIFT))) |
        ((uint32_t)input << CSL_EPWM_DBCTL_IN_MODE_SHIFT)));
}

//*****************************************************************************
//
//! Sets Dead Band delay input.
//!
//! \param base is the base address of the EPWM module.
//! \param input is the input signal to the dead band.
//!
//! This function sets up the rising Edge delay input signal.
//! Valid values for input are:
//!   - EPWM_DB_INPUT_EPWMA   - Input signal is ePWMA(Valid for both Falling
//!                                Edge and Rising Edge)
//!   - EPWM_DB_INPUT_EPWMB   - Input signal is ePWMB(Valid for both Falling
//!                                Edge and Rising Edge)
//!   - EPWM_DB_INPUT_DB_RED  - Input signal is the output of Rising
//!                                Edge delay.
//!                               (Valid only for Falling Edge delay)
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setFallingEdgeDeadBandDelayInput(uint32_t base, uint16_t input)
{
    //
    // Check the arguments
    //
    DebugP_assert((input == EPWM_DB_INPUT_EPWMA) ||
           (input == EPWM_DB_INPUT_EPWMB) ||
           (input == EPWM_DB_INPUT_DB_RED));

    if(input == EPWM_DB_INPUT_DB_RED)
    {
        //
        // Set the Falling Edge Delay input
        //
        HW_WR_REG16(base + CSL_EPWM_DBCTL,
            (HW_RD_REG16(base + CSL_EPWM_DBCTL) |
            CSL_EPWM_DBCTL_DEDB_MODE_MASK));
    }
    else
    {
        //
        // Set the Falling Edge Delay input
        //
        HW_WR_REG16(base + CSL_EPWM_DBCTL,
            (HW_RD_REG16(base + CSL_EPWM_DBCTL) &
            ~CSL_EPWM_DBCTL_DEDB_MODE_MASK));

        //
        // Set the Rising Edge Delay input
        //
        HW_WR_REG16(base + CSL_EPWM_DBCTL,
            ((HW_RD_REG16(base + CSL_EPWM_DBCTL) &
            ~(1U << (CSL_EPWM_DBCTL_IN_MODE_SHIFT + 1U))) |
            ((uint32_t)input << (CSL_EPWM_DBCTL_IN_MODE_SHIFT + 1U))));
    }
}

//*****************************************************************************
//
//! Set the Dead Band control shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param loadMode is the shadow to active load mode.
//!
//! This function enables and sets the Dead Band control register shadow
//! load mode.
//! Valid values for the \e loadMode parameter are:
//!     - EPWM_DB_LOAD_ON_CNTR_ZERO         - load when counter equals zero.
//!     - EPWM_DB_LOAD_ON_CNTR_PERIOD       - load when counter equals period.
//!     - EPWM_DB_LOAD_ON_CNTR_ZERO_PERIOD  - load when counter equals zero or
//!                                            period.
//!     - EPWM_DB_LOAD_FREEZE                - Freeze shadow to active load.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setDeadBandControlShadowLoadMode(uint32_t base,
                                      EPWM_DeadBandControlLoadMode loadMode)
{
    //
    // Enable the shadow mode and setup the load event
    //
    HW_WR_REG16(base + CSL_EPWM_DBCTL2,
        ((HW_RD_REG16(base + CSL_EPWM_DBCTL2) &
        ~(uint16_t)CSL_EPWM_DBCTL2_LOADDBCTLMODE_MASK) |
        (uint16_t)(CSL_EPWM_DBCTL2_LOADDBCTLMODE_MASK | (uint16_t)loadMode)));
}

//*****************************************************************************
//
//! Disable Dead Band control shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the Dead Band control register shadow
//! load mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableDeadBandControlShadowLoadMode(uint32_t base)
{
    //
    // Disable the shadow load mode. Only immediate load mode only.
    //
    HW_WR_REG16(base + CSL_EPWM_DBCTL2,
        (HW_RD_REG16(base + CSL_EPWM_DBCTL2) &
        ~CSL_EPWM_DBCTL2_SHDWDBCTLMODE_MASK));
}

//*****************************************************************************
//
//! Set the RED (Rising Edge Delay) shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param loadMode is the shadow to active load event.
//!
//! This function sets the Rising Edge Delay register shadow load mode.
//! Valid values for the \e loadMode parameter are:
//!     - EPWM_RED_LOAD_ON_CNTR_ZERO        - load when counter equals zero.
//!     - EPWM_RED_LOAD_ON_CNTR_PERIOD      - load when counter equals period.
//!     - EPWM_RED_LOAD_ON_CNTR_ZERO_PERIOD - load when counter equals zero or
//!                                           period.
//!     - EPWM_RED_LOAD_FREEZE               - Freeze shadow to active load.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setRisingEdgeDelayCountShadowLoadMode(uint32_t base,
                                         EPWM_RisingEdgeDelayLoadMode loadMode)
{
    //
    // Enable the shadow mode. Set-up the load mode
    //
    HW_WR_REG16(base + CSL_EPWM_DBCTL,
        ((HW_RD_REG16(base + CSL_EPWM_DBCTL) &
        ~(uint16_t)CSL_EPWM_DBCTL_LOADREDMODE_MASK) |
        (uint16_t)(CSL_EPWM_DBCTL_SHDWDBREDMODE_MASK |
        (uint16_t)((uint16_t)loadMode << CSL_EPWM_DBCTL_LOADREDMODE_SHIFT))));

}

//*****************************************************************************
//
//! Disable the RED (Rising Edge Delay) shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the Rising Edge Delay register shadow load mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableRisingEdgeDelayCountShadowLoadMode(uint32_t base)
{
    //
    // Disable the shadow mode.
    //
    HW_WR_REG16(base + CSL_EPWM_DBCTL,
        (HW_RD_REG16(base + CSL_EPWM_DBCTL) &
        ~CSL_EPWM_DBCTL_SHDWDBREDMODE_MASK));
}

//*****************************************************************************
//
//! Set the FED (Falling Edge Delay) shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param loadMode is the shadow to active load event.
//!
//! This function enables and sets the Falling Edge Delay register shadow load
//! mode. Valid values for the \e loadMode parameters are:
//!     - EPWM_FED_LOAD_ON_CNTR_ZERO        - load when counter equals zero.
//!     - EPWM_FED_LOAD_ON_CNTR_PERIOD      - load when counter equals period.
//!     - EPWM_FED_LOAD_ON_CNTR_ZERO_PERIOD - load when counter equals zero or
//!                                            period.
//!     - EPWM_FED_LOAD_FREEZE               - Freeze shadow to active load.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setFallingEdgeDelayCountShadowLoadMode(uint32_t base,
                                        EPWM_FallingEdgeDelayLoadMode loadMode)
{
    //
    // Enable the shadow mode. Setup the load mode.
    //
    HW_WR_REG16(base + CSL_EPWM_DBCTL,
        ((HW_RD_REG16(base + CSL_EPWM_DBCTL) &
        ~(uint16_t)CSL_EPWM_DBCTL_LOADFEDMODE_MASK) |
        (uint16_t)(CSL_EPWM_DBCTL_SHDWDBFEDMODE_MASK |
        (uint16_t)((uint16_t)loadMode << CSL_EPWM_DBCTL_LOADFEDMODE_SHIFT))));
}

//*****************************************************************************
//
//! Disables the FED (Falling Edge Delay) shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the Falling Edge Delay register shadow load mode.
//! Valid values for the parameters are:
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableFallingEdgeDelayCountShadowLoadMode(uint32_t base)
{
    //
    // Disable the shadow mode.
    //
    HW_WR_REG16(base + CSL_EPWM_DBCTL,
        (HW_RD_REG16(base + CSL_EPWM_DBCTL) &
        ~CSL_EPWM_DBCTL_SHDWDBFEDMODE_MASK));
}

//*****************************************************************************
//
//! Sets Dead Band Counter clock rate.
//!
//! \param base is the base address of the EPWM module.
//! \param clockMode is the Dead Band counter clock mode.
//!
//! This function sets up the Dead Band counter clock rate with respect to
//! TBCLK (ePWM time base counter).
//! Valid values for clockMode are:
//!   - EPWM_DB_COUNTER_CLOCK_FULL_CYCLE  -Dead band counter runs at TBCLK
//!                                           (ePWM Time Base Counter) rate.
//!   - EPWM_DB_COUNTER_CLOCK_HALF_CYCLE  -Dead band counter runs at 2*TBCLK
//!                                         (twice ePWM Time Base Counter)rate.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setDeadBandCounterClock(uint32_t base,
                             EPWM_DeadBandClockMode clockMode)
{
    //
    // Set the DB clock mode
    //
    HW_WR_REG16(base + CSL_EPWM_DBCTL,
        ((HW_RD_REG16(base + CSL_EPWM_DBCTL) &
        ~(uint16_t)CSL_EPWM_DBCTL_HALFCYCLE_MASK) |
        (uint16_t)((uint16_t)clockMode << CSL_EPWM_DBCTL_HALFCYCLE_SHIFT)));
}

//*****************************************************************************
//
//! Set ePWM RED count
//!
//! \param base is the base address of the EPWM module.
//! \param redCount is the RED(Rising Edge Delay) count.
//!
//! This function sets the RED (Rising Edge Delay) count value.
//! The value of redCount should be less than 0x4000U.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setRisingEdgeDelayCount(uint32_t base, uint16_t redCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(redCount <= CSL_EPWM_DBRED_DBRED_MAX);

    //
    // Set the RED (Rising Edge Delay) count
    //
    HW_WR_REG16(base + CSL_EPWM_DBRED, redCount);
}

//*****************************************************************************
//
//! Set ePWM FED count
//!
//! \param base is the base address of the EPWM module.
//! \param fedCount is the FED(Falling Edge Delay) count.
//!
//! This function sets the FED (Falling Edge Delay) count value.
//! The value of fedCount should be less than 0x4000U.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setFallingEdgeDelayCount(uint32_t base, uint16_t fedCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(fedCount <= CSL_EPWM_DBFED_DBFED_MAX);

    //
    // Set the FED (Falling Edge Delay) count
    //
    HW_WR_REG16(base + CSL_EPWM_DBFED, fedCount);
}

//
// Chopper module related APIs
//
//*****************************************************************************
//
//! Enable chopper mode
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables ePWM chopper module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableChopper(uint32_t base)
{
    //
    // Set CHPEN bit. Enable Chopper
    //
    HW_WR_REG16(base + CSL_EPWM_PCCTL,
        (HW_RD_REG16(base + CSL_EPWM_PCCTL) | CSL_EPWM_PCCTL_CHPEN_MASK));
}

//*****************************************************************************
//
//! Disable chopper mode
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables ePWM chopper module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableChopper(uint32_t base)
{
    //
    // Clear CHPEN bit. Disable Chopper
    //
    HW_WR_REG16(base + CSL_EPWM_PCCTL,
        (HW_RD_REG16(base + CSL_EPWM_PCCTL) & ~CSL_EPWM_PCCTL_CHPEN_MASK));
}

//*****************************************************************************
//
//! Set chopper duty cycle.
//!
//! \param base is the base address of the EPWM module.
//! \param dutyCycleCount is the chopping clock duty cycle count.
//!
//! This function sets the chopping clock duty cycle. The value of
//! dutyCycleCount should be less than 7. The dutyCycleCount value is converted
//! to the actual chopper duty cycle value base on the following equation:
//!   chopper duty cycle = (dutyCycleCount + 1) / 8
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setChopperDutyCycle(uint32_t base, uint16_t dutyCycleCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(dutyCycleCount < CSL_EPWM_PCCTL_CHPDUTY_MAX);

    //
    // Set the chopper duty cycle
    //
    HW_WR_REG16(base + CSL_EPWM_PCCTL,
        ((HW_RD_REG16(base + CSL_EPWM_PCCTL) & ~CSL_EPWM_PCCTL_CHPDUTY_MASK) |
        ((uint32_t)dutyCycleCount << CSL_EPWM_PCCTL_CHPDUTY_SHIFT)));

}

//*****************************************************************************
//
//! Set chopper clock frequency scaler.
//!
//! \param base is the base address of the EPWM module.
//! \param freqDiv is the chopping clock frequency divider.
//!
//! This function sets the scaler for the chopping clock frequency. The value
//! of freqDiv should be less than 8. The chopping clock frequency is altered
//! based on the following equation.
//!   chopper clock frequency = SYSCLKOUT / (1 + freqDiv)
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setChopperFreq(uint32_t base, uint16_t freqDiv)
{
    //
    // Check the arguments
    //
    DebugP_assert(freqDiv <= CSL_EPWM_PCCTL_CHPFREQ_MAX);

    //
    // Set the chopper clock
    //
    HW_WR_REG16(base + CSL_EPWM_PCCTL,
        ((HW_RD_REG16(base + CSL_EPWM_PCCTL) &
        ~(uint16_t)CSL_EPWM_PCCTL_CHPFREQ_MASK) |
        ((uint32_t)freqDiv << CSL_EPWM_PCCTL_CHPFREQ_SHIFT)));
}

//*****************************************************************************
//
//! Set chopper clock frequency scaler.
//!
//! \param base is the base address of the EPWM module.
//! \param firstPulseWidth is the width of the first pulse.
//!
//! This function sets the first pulse width of chopper output waveform. The
//! value of firstPulseWidth should be less than 0x10. The value of the first
//! pulse width in seconds is given using the following equation:
//!     first pulse width = 1 / (((firstPulseWidth + 1) * SYSCLKOUT)/8)
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setChopperFirstPulseWidth(uint32_t base, uint16_t firstPulseWidth)
{
    //
    // Check the arguments
    //
    DebugP_assert(firstPulseWidth <= CSL_EPWM_PCCTL_OSHTWTH_MAX);

    //
    // Set the chopper clock
    //
    HW_WR_REG16(base + CSL_EPWM_PCCTL,
        ((HW_RD_REG16(base + CSL_EPWM_PCCTL) &
        ~(uint16_t)CSL_EPWM_PCCTL_OSHTWTH_MASK) |
        ((uint32_t)firstPulseWidth << CSL_EPWM_PCCTL_OSHTWTH_SHIFT)));
}

//
// Trip Zone module related APIs
//
//*****************************************************************************
//
//! Enables Trip Zone signal.
//!
//! \param base is the base address of the EPWM module.
//! \param tzSignal is the Trip Zone signal.
//!
//! This function enables the Trip Zone signals specified by tzSignal as a
//! source for the Trip Zone module.
//! Valid values for tzSignal are:
//!   - EPWM_TZ_SIGNAL_CBC1       - TZ1 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC2       - TZ2 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC3       - TZ3 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC4       - TZ4 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC5       - TZ5 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC6       - TZ6 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_DCAEVT2    - DCAEVT2 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_DCBEVT2    - DCBEVT2 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_OSHT1      - One-shot TZ1
//!   - EPWM_TZ_SIGNAL_OSHT2      - One-shot TZ2
//!   - EPWM_TZ_SIGNAL_OSHT3      - One-shot TZ3
//!   - EPWM_TZ_SIGNAL_OSHT4      - One-shot TZ4
//!   - EPWM_TZ_SIGNAL_OSHT5      - One-shot TZ5
//!   - EPWM_TZ_SIGNAL_OSHT6      - One-shot TZ6
//!   - EPWM_TZ_SIGNAL_DCAEVT1    - One-shot DCAEVT1
//!   - EPWM_TZ_SIGNAL_DCBEVT1    - One-shot DCBEVT1
//!
//! \b note:  A logical OR of the valid values can be passed as the tzSignal
//!           parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableTripZoneSignals(uint32_t base, uint32_t tzSignal)
{
    //
    // Set the trip zone bits
    //
    HW_WR_REG32(base + CSL_EPWM_TZSEL,
        (HW_RD_REG32(base + CSL_EPWM_TZSEL) | tzSignal));
}

//*****************************************************************************
//
//! Disables Trip Zone signal.
//!
//! \param base is the base address of the EPWM module.
//! \param tzSignal is the Trip Zone signal.
//!
//! This function disables the Trip Zone signal specified by tzSignal as a
//! source for the Trip Zone module.
//! Valid values for tzSignal are:
//!   - EPWM_TZ_SIGNAL_CBC1       - TZ1 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC2       - TZ2 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC3       - TZ3 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC4       - TZ4 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC5       - TZ5 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC6       - TZ6 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_DCAEVT2    - DCAEVT2 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_DCBEVT2    - DCBEVT2 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_OSHT1      - One-shot TZ1
//!   - EPWM_TZ_SIGNAL_OSHT2      - One-shot TZ2
//!   - EPWM_TZ_SIGNAL_OSHT3      - One-shot TZ3
//!   - EPWM_TZ_SIGNAL_OSHT4      - One-shot TZ4
//!   - EPWM_TZ_SIGNAL_OSHT5      - One-shot TZ5
//!   - EPWM_TZ_SIGNAL_OSHT6      - One-shot TZ6
//!   - EPWM_TZ_SIGNAL_DCAEVT1    - One-shot DCAEVT1
//!   - EPWM_TZ_SIGNAL_DCBEVT1    - One-shot DCBEVT1
//!
//! \b note:  A logical OR of the valid values can be passed as the tzSignal
//!           parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableTripZoneSignals(uint32_t base, uint32_t tzSignal)
{
    //
    // Clear the trip zone bits
    //
    HW_WR_REG32(base + CSL_EPWM_TZSEL,
        (HW_RD_REG32(base + CSL_EPWM_TZSEL) & ~tzSignal));
}

//*****************************************************************************
//
//! Enables Trip Zone 2 signal (TZSEL2).
//!
//! \param base is the base address of the EPWM module.
//! \param tzSignal is the Trip Zone signal.
//!
//! This function enables the Trip Zone signals specified by tzSignal as a
//! source for the Trip Zone module.
//! Valid values for tzSignal are:
//!   - EPWM_TZ_SIGNAL_CAPEVT_OST    - One-shot CAPEVT
//!   - EPWM_TZ_SIGNAL_CAPEVT_CBC    - Cycle By Cycle CAPEVT
//!
//! \b note:  A logical OR of the valid values can be passed as the tzSignal
//!           parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableTripZone2Signals(uint32_t base, uint16_t tzSignal)
{
    //
    // Set the trip zone bits
    //
    HW_WR_REG16(base + CSL_EPWM_TZSEL2,
        (HW_RD_REG16(base + CSL_EPWM_TZSEL2) | tzSignal));
}

//*****************************************************************************
//
//! Disables Trip Zone 2 signal (TZSEL2).
//!
//! \param base is the base address of the EPWM module.
//! \param tzSignal is the Trip Zone signal.
//!
//! This function disables the Trip Zone signals specified by tzSignal as a
//! source for the Trip Zone module.
//! Valid values for tzSignal are:
//!   - EPWM_TZ_SIGNAL_CAPEVT_OST    - One-shot CAPEVT
//!   - EPWM_TZ_SIGNAL_CAPEVT_CBC    - Cycle By Cycle CAPEVT
//!
//! \b note:  A logical OR of the valid values can be passed as the tzSignal
//!           parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableTripZone2Signals(uint32_t base, uint16_t tzSignal)
{
    //
    // Clear the trip zone bits
    //
    HW_WR_REG16(base + CSL_EPWM_TZSEL2,
        (HW_RD_REG16(base + CSL_EPWM_TZSEL2) & ~tzSignal));
}

//*****************************************************************************
//
//! Set Digital compare conditions that cause Trip Zone event.
//!
//! \param base is the base address of the EPWM module.
//! \param dcType is the Digital compare output type.
//! \param dcEvent is the Digital Compare output event.
//!
//! This function sets up the Digital Compare output Trip Zone event sources.
//! The dcType variable specifies the event source to be whether Digital
//! Compare output A or Digital Compare output B. The dcEvent parameter
//! specifies the event that causes Trip Zone.
//! Valid values for the parameters are:
//!  - dcType
//!      - EPWM_TZ_DC_OUTPUT_A1     - Digital Compare output 1 A
//!      - EPWM_TZ_DC_OUTPUT_A2     - Digital Compare output 2 A
//!      - EPWM_TZ_DC_OUTPUT_B1     - Digital Compare output 1 B
//!      - EPWM_TZ_DC_OUTPUT_B2     - Digital Compare output 2 B
//!  - dcEvent
//!      - EPWM_TZ_EVENT_DC_DISABLED         - Event Trigger is disabled
//!      - EPWM_TZ_EVENT_DCXH_LOW            - Trigger event when DCxH low
//!      - EPWM_TZ_EVENT_DCXH_HIGH           - Trigger event when DCxH high
//!      - EPWM_TZ_EVENT_DCXL_LOW            - Trigger event when DCxL low
//!      - EPWM_TZ_EVENT_DCXL_HIGH           - Trigger event when DCxL high
//!      - EPWM_TZ_EVENT_DCXL_HIGH_DCXH_LOW  - Trigger event when DCxL high
//!                                            DCxH low
//!
//!  \note x in DCxH/DCxL represents DCAH/DCAL or DCBH/DCBL
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTripZoneDigitalCompareEventCondition(uint32_t base,
                                EPWM_TripZoneDigitalCompareOutput dcType,
                                EPWM_TripZoneDigitalCompareOutputEvent dcEvent)
{
    //
    // Set Digital Compare Events conditions that cause a Digital Compare trip
    //
    HW_WR_REG16(base + CSL_EPWM_TZDCSEL,
        ((HW_RD_REG16(base + CSL_EPWM_TZDCSEL) &
        ~(CSL_EPWM_TZDCSEL_DCAEVT1_MASK << (uint16_t)dcType)) |
        ((uint16_t)dcEvent << (uint16_t)dcType)));
}

//*****************************************************************************
//
//! Enable advanced Trip Zone event Action.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the advanced actions of the Trip Zone events. The
//! advanced features combine the trip zone events with the direction of the
//! counter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableTripZoneAdvAction(uint32_t base)
{
    //
    // Enable Advanced feature. Set ETZE bit
    //
    HW_WR_REG16(base + CSL_EPWM_TZCTL2,
        (HW_RD_REG16(base + CSL_EPWM_TZCTL2) | CSL_EPWM_TZCTL2_ETZE_MASK));
}

//*****************************************************************************
//
//! Disable advanced Trip Zone event Action.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the advanced actions of the Trip Zone events.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableTripZoneAdvAction(uint32_t base)
{
    //
    // Disable Advanced feature. clear ETZE bit
    //
    HW_WR_REG16(base + CSL_EPWM_TZCTL2,
        (HW_RD_REG16(base + CSL_EPWM_TZCTL2) & ~CSL_EPWM_TZCTL2_ETZE_MASK));
}

//*****************************************************************************
//
//! Set Trip Zone Action.
//!
//! \param base is the base address of the EPWM module.
//! \param tzEvent is the Trip Zone event type.
//! \param tzAction is the Trip zone Action.
//!
//! This function sets the Trip Zone Action to be taken when a Trip Zone event
//! occurs.
//! Valid values for the parameters are:
//!  - tzEvent
//!      - EPWM_TZ_ACTION_EVENT_DCBEVT2  - DCBEVT2 (Digital Compare B event 2)
//!      - EPWM_TZ_ACTION_EVENT_DCBEVT1  - DCBEVT1 (Digital Compare B event 1)
//!      - EPWM_TZ_ACTION_EVENT_DCAEVT2  - DCAEVT2 (Digital Compare A event 2)
//!      - EPWM_TZ_ACTION_EVENT_DCAEVT1  - DCAEVT1 (Digital Compare A event 1)
//!      - EPWM_TZ_ACTION_EVENT_TZB      - TZ1 - TZ6, DCBEVT2, DCBEVT1
//!      - EPWM_TZ_ACTION_EVENT_TZA      - TZ1 - TZ6, DCAEVT2, DCAEVT1
//!  - tzAction
//!      - EPWM_TZ_ACTION_HIGH_Z         - high impedance output
//!      - EPWM_TZ_ACTION_HIGH           - high output
//!      - EPWM_TZ_ACTION_LOW            - low low
//!      - EPWM_TZ_ACTION_DISABLE        - disable action
//!
//! \note Disable the advanced Trip Zone event using
//!       EPWM_disableTripZoneAdvAction()  before calling this function.
//! \note This function operates on both ePWMA and ePWMB depending on the
//!       tzEvent parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTripZoneAction(uint32_t base, EPWM_TripZoneEvent tzEvent,
                       EPWM_TripZoneAction tzAction)
{
    //
    // Set the Action for Trip Zone events
    //
    HW_WR_REG16(base + CSL_EPWM_TZCTL,
        ((HW_RD_REG16(base + CSL_EPWM_TZCTL) &
        ~(CSL_EPWM_TZCTL_TZA_MASK << (uint16_t)tzEvent)) |
        ((uint16_t)tzAction << (uint16_t)tzEvent)));
}

//*****************************************************************************
//
//! Set Advanced Trip Zone Action.
//!
//! \param base is the base address of the EPWM module.
//! \param tzAdvEvent is the Trip Zone event type.
//! \param tzAdvAction is the Trip zone Action.
//!
//! This function sets the Advanced Trip Zone Action to be taken when an
//! advanced Trip Zone event occurs.
//!
//! Valid values for the parameters are:
//!  - tzAdvEvent
//!      - EPWM_TZ_ADV_ACTION_EVENT_TZB_D  - TZ1 - TZ6, DCBEVT2, DCBEVT1 while
//!                                             counting down
//!      - EPWM_TZ_ADV_ACTION_EVENT_TZB_U  - TZ1 - TZ6, DCBEVT2, DCBEVT1 while
//!                                             counting up
//!      - EPWM_TZ_ADV_ACTION_EVENT_TZA_D  - TZ1 - TZ6, DCAEVT2, DCAEVT1 while
//!                                             counting down
//!      - EPWM_TZ_ADV_ACTION_EVENT_TZA_U  - TZ1 - TZ6, DCAEVT2, DCAEVT1 while
//!                                             counting up
//!  - tzAdvAction
//!      - EPWM_TZ_ADV_ACTION_HIGH_Z    - high impedance output
//!      - EPWM_TZ_ADV_ACTION_HIGH      - high voltage state
//!      - EPWM_TZ_ADV_ACTION_LOW       - low voltage state
//!      - EPWM_TZ_ADV_ACTION_TOGGLE    - Toggle output
//!      - EPWM_TZ_ADV_ACTION_DISABLE   - disable action
//!
//! \note This function enables the advanced Trip Zone event.
//!
//! \note This function operates on both ePWMA and ePWMB depending on the
//!       tzAdvEvent  parameter.
//! \note Advanced Trip Zone events take into consideration the direction of
//!       the counter in addition to Trip Zone events.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTripZoneAdvAction(uint32_t base, EPWM_TripZoneAdvancedEvent tzAdvEvent,
                          EPWM_TripZoneAdvancedAction tzAdvAction)
{
    //
    // Set the Advanced Action for Trip Zone events
    //
    HW_WR_REG16(base + CSL_EPWM_TZCTL2,
        ((HW_RD_REG16(base + CSL_EPWM_TZCTL2) &
        ~(CSL_EPWM_TZCTL2_TZAU_MASK << (uint16_t)tzAdvEvent)) |
        ((uint16_t)tzAdvAction << (uint16_t)tzAdvEvent)));

    HW_WR_REG16(base + CSL_EPWM_TZCTL2,
        (HW_RD_REG16(base + CSL_EPWM_TZCTL2) | CSL_EPWM_TZCTL2_ETZE_MASK));
}

//*****************************************************************************
//
//! Set Advanced Digital Compare Trip Zone Action on ePWMA.
//!
//! \param base is the base address of the EPWM module.
//! \param tzAdvDCEvent is the Digital Compare Trip Zone event type.
//! \param tzAdvDCAction is the Digital Compare Trip zone Action.
//!
//! This function sets the Digital Compare (DC) Advanced Trip Zone Action to be
//! taken on ePWMA when an advanced Digital Compare Trip Zone A event occurs.
//! Valid values for the parameters are:
//!  - tzAdvDCEvent
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D  - Digital Compare event A2 while
//!                                                 counting down
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U  - Digital Compare event A2 while
//!                                                 counting up
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D  - Digital Compare event A1 while
//!                                                 counting down
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U  - Digital Compare event A1 while
//!                                                 counting up
//!  - tzAdvDCAction
//!      - EPWM_TZ_ADV_ACTION_HIGH_Z    - high impedance output
//!      - EPWM_TZ_ADV_ACTION_HIGH      - high voltage state
//!      - EPWM_TZ_ADV_ACTION_LOW       - low voltage state
//!      - EPWM_TZ_ADV_ACTION_TOGGLE    - Toggle output
//!      - EPWM_TZ_ADV_ACTION_DISABLE   - disable action
//!
//! \note This function enables the advanced Trip Zone event.
//!
//! \note Advanced Trip Zone events take into consideration the direction of
//!       the counter in addition to Digital Compare Trip Zone events.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTripZoneAdvDigitalCompareActionA(uint32_t base,
                              EPWM_TripZoneAdvDigitalCompareEvent tzAdvDCEvent,
                              EPWM_TripZoneAdvancedAction tzAdvDCAction)
{
    //
    // Set the Advanced Action for Trip Zone events
    //
    HW_WR_REG16(base + CSL_EPWM_TZCTLDCA,
        ((HW_RD_REG16(base + CSL_EPWM_TZCTLDCA) &
        ~(CSL_EPWM_TZCTLDCA_DCAEVT1U_MASK << (uint16_t)tzAdvDCEvent)) |
        ((uint16_t)tzAdvDCAction << (uint16_t)tzAdvDCEvent)));

    HW_WR_REG16(base + CSL_EPWM_TZCTL2,
        (HW_RD_REG16(base + CSL_EPWM_TZCTL2) | CSL_EPWM_TZCTL2_ETZE_MASK));
}

//*****************************************************************************
//
//! Set Advanced Digital Compare Trip Zone Action on ePWMB.
//!
//! \param base is the base address of the EPWM module.
//! \param tzAdvDCEvent is the Digital Compare Trip Zone event type.
//! \param tzAdvDCAction is the Digital Compare Trip zone Action.
//!
//! This function sets the Digital Compare (DC) Advanced Trip Zone Action to be
//! taken on ePWMB when an advanced Digital Compare Trip Zone B event occurs.
//! Valid values for the parameters are:
//!  - tzAdvDCEvent
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D  - Digital Compare event B2 while
//!                                                 counting down
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U  - Digital Compare event B2 while
//!                                                 counting up
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D  - Digital Compare event B1 while
//!                                                 counting down
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U  - Digital Compare event B1 while
//!                                                 counting up
//!  - tzAdvDCAction
//!      - EPWM_TZ_ADV_ACTION_HIGH_Z    - high impedance output
//!      - EPWM_TZ_ADV_ACTION_HIGH      - high voltage state
//!      - EPWM_TZ_ADV_ACTION_LOW       - low voltage state
//!      - EPWM_TZ_ADV_ACTION_TOGGLE    - Toggle output
//!      - EPWM_TZ_ADV_ACTION_DISABLE   - disable action
//!
//! \note This function enables the advanced Trip Zone event.
//!
//! \note Advanced Trip Zone events take into consideration the direction of
//!       the counter in addition to Digital Compare Trip Zone events.
//!
//! \return None.
//
//*****************************************************************************
static inline void EPWM_setTripZoneAdvDigitalCompareActionB(uint32_t base,
                              EPWM_TripZoneAdvDigitalCompareEvent tzAdvDCEvent,
                              EPWM_TripZoneAdvancedAction tzAdvDCAction)
{
    //
    // Set the Advanced Action for Trip Zone events
    //
    HW_WR_REG16(base + CSL_EPWM_TZCTLDCB,
        ((HW_RD_REG16(base + CSL_EPWM_TZCTLDCB) &
        ~(CSL_EPWM_TZCTLDCB_DCBEVT1U_MASK << (uint16_t)tzAdvDCEvent)) |
        ((uint16_t)tzAdvDCAction << (uint16_t)tzAdvDCEvent)));

    HW_WR_REG16(base + CSL_EPWM_TZCTL2,
        (HW_RD_REG16(base + CSL_EPWM_TZCTL2) | CSL_EPWM_TZCTL2_ETZE_MASK));
}

//*****************************************************************************
//
//! Enable Trip Zone interrupts.
//!
//! \param base is the base address of the EPWM module.
//! \param tzInterrupt is the Trip Zone interrupt.
//!
//! This function enables the Trip Zone interrupts.
//! Valid values for tzInterrupt are:
//!   - EPWM_TZ_INTERRUPT_CBC     - Trip Zones Cycle By Cycle interrupt
//!   - EPWM_TZ_INTERRUPT_OST     - Trip Zones One Shot interrupt
//!   - EPWM_TZ_INTERRUPT_DCAEVT1 - Digital Compare A Event 1 interrupt
//!   - EPWM_TZ_INTERRUPT_DCAEVT2 - Digital Compare A Event 2 interrupt
//!   - EPWM_TZ_INTERRUPT_DCBEVT1 - Digital Compare B Event 1 interrupt
//!   - EPWM_TZ_INTERRUPT_DCBEVT2 - Digital Compare B Event 2 interrupt
//!   - EPWM_TZ_INTERRUPT_CAPEVT  - Digital Capture Event interrupt
//!
//! \b note:  A logical OR of the valid values can be passed as the tzInterrupt
//!           parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableTripZoneInterrupt(uint32_t base, uint16_t tzInterrupt)
{
    //
    // Check the arguments
    //
    DebugP_assert((tzInterrupt <= 0x80U));

    //
    // Enable Trip zone interrupts
    //
    HW_WR_REG16(base + CSL_EPWM_TZEINT,
        (HW_RD_REG16(base + CSL_EPWM_TZEINT) | tzInterrupt));
}

//*****************************************************************************
//
//! Disable Trip Zone interrupts.
//!
//! \param base is the base address of the EPWM module.
//! \param tzInterrupt is the Trip Zone interrupt.
//!
//! This function disables the Trip Zone interrupts.
//! Valid values for tzInterrupt are:
//!   - EPWM_TZ_INTERRUPT_CBC     - Trip Zones Cycle By Cycle interrupt
//!   - EPWM_TZ_INTERRUPT_OST    - Trip Zones One Shot interrupt
//!   - EPWM_TZ_INTERRUPT_DCAEVT1 - Digital Compare A Event 1 interrupt
//!   - EPWM_TZ_INTERRUPT_DCAEVT2 - Digital Compare A Event 2 interrupt
//!   - EPWM_TZ_INTERRUPT_DCBEVT1 - Digital Compare B Event 1 interrupt
//!   - EPWM_TZ_INTERRUPT_DCBEVT2 - Digital Compare B Event 2 interrupt
//!   - EPWM_TZ_INTERRUPT_CAPEVT  - Digital Capture Event interrupt
//!
//! \b note:  A logical OR of the valid values can be passed as the tzInterrupt
//!           parameter.
//!
//! \return None.
//
//***************************************************************************
static inline void
EPWM_disableTripZoneInterrupt(uint32_t base, uint16_t tzInterrupt)
{
    //
    // Check the arguments
    //
    DebugP_assert((tzInterrupt > 0U) && (tzInterrupt <= 0x80U));

    //
    // Disable Trip zone interrupts
    //
    HW_WR_REG16(base + CSL_EPWM_TZEINT,
        (HW_RD_REG16(base + CSL_EPWM_TZEINT) & ~tzInterrupt));
}

//*****************************************************************************
//
//! Gets the Trip Zone status flag
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the Trip Zone status flag.
//!
//! \return The function returns the following or the bitwise OR value
//!         of the following values.
//!         - EPWM_TZ_INTERRUPT    - Trip Zone interrupt was generated
//!                                  due to the following TZ events.
//!         - EPWM_TZ_FLAG_CBC     - Trip Zones Cycle By Cycle event status flag
//!         - EPWM_TZ_FLAG_OST     - Trip Zones One Shot event status flag
//!         - EPWM_TZ_FLAG_DCAEVT1 - Digital Compare A Event 1 status flag
//!         - EPWM_TZ_FLAG_DCAEVT2 - Digital Compare A Event 2 status flag
//!         - EPWM_TZ_FLAG_DCBEVT1 - Digital Compare B Event 1 status flag
//!         - EPWM_TZ_FLAG_DCBEVT2 - Digital Compare B Event 2 status flag
//!         - EPWM_TZ_FLAG_CAPEVT  - Digital Capture Event flag
//
//***************************************************************************
static inline uint16_t
EPWM_getTripZoneFlagStatus(uint32_t base)
{
    //
    // Return the Trip zone flag status
    //
    return(HW_RD_REG16(base + CSL_EPWM_TZFLG) & 0xFFU);
}

//*****************************************************************************
//
//! Gets the Trip Zone Cycle by Cycle flag status
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the specific Cycle by Cycle Trip Zone flag
//! status.
//!
//! \return The function returns the following values.
//!           - EPWM_TZ_CBC_FLAG_1     - CBC 1 status flag
//!           - EPWM_TZ_CBC_FLAG_2     - CBC 2 status flag
//!           - EPWM_TZ_CBC_FLAG_3     - CBC 3 status flag
//!           - EPWM_TZ_CBC_FLAG_4     - CBC 4 status flag
//!           - EPWM_TZ_CBC_FLAG_5     - CBC 5 status flag
//!           - EPWM_TZ_CBC_FLAG_6     - CBC 6 status flag
//!           - EPWM_TZ_CBC_FLAG_DCAEVT2  - CBC status flag for Digital compare
//!                                                event A2
//!           - EPWM_TZ_CBC_FLAG_DCBEVT2  - CBC status flag for Digital compare
//!                                                event B2
//!           - EPWM_TZ_CBC_FLAG_CAPEVT - CBC status flag for capture event
//
//***************************************************************************
static inline uint16_t
EPWM_getCycleByCycleTripZoneFlagStatus(uint32_t base)
{
    //
    // Return the Cycle By Cycle Trip zone flag status
    //
    return(HW_RD_REG16(base + CSL_EPWM_TZCBCFLG) & 0x1FFU);
}

//*****************************************************************************
//
//! Gets the Trip Zone One Shot flag status
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the specific One Shot Trip Zone flag status.
//!
//! \return The function returns the bitwise OR of the following flags.
//!           - EPWM_TZ_OST_FLAG_OST1     - OST status flag for OST1
//!           - EPWM_TZ_OST_FLAG_OST2     - OST status flag for OST2
//!           - EPWM_TZ_OST_FLAG_OST3     - OST status flag for OST3
//!           - EPWM_TZ_OST_FLAG_OST4     - OST status flag for OST4
//!           - EPWM_TZ_OST_FLAG_OST5     - OST status flag for OST5
//!           - EPWM_TZ_OST_FLAG_OST6     - OST status flag for OST6
//!           - EPWM_TZ_OST_FLAG_DCAEVT1  - OST status flag for Digital
//!                                                 compare event A1
//!           - EPWM_TZ_OST_FLAG_DCBEVT1  - OST status flag for Digital
//!                                                 compare event B1
//
//***************************************************************************
static inline uint16_t
EPWM_getOneShotTripZoneFlagStatus(uint32_t base)
{
    //
    // Return the One Shot Trip zone flag status
    //
    return(HW_RD_REG16(base + CSL_EPWM_TZOSTFLG) & 0x1FFU);
}

//*****************************************************************************
//
//! Set the Trip Zone CBC pulse clear event.
//!
//! \param base is the base address of the EPWM module.
//! \param clearEvent is the CBC trip zone clear event.
//!
//! This function set the event which automatically clears the
//! CBC (Cycle by Cycle) latch.
//! Valid values for clearEvent are:
//!   - EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO         - Clear CBC pulse when counter
//!                                                  equals zero
//!   - EPWM_TZ_CBC_PULSE_CLR_CNTR_PERIOD       - Clear CBC pulse when counter
//!                                                  equals period
//!   - EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO_PERIOD  - Clear CBC pulse when counter
//!                                                  equals zero or period
//!
//!  \return None.
//
//**************************************************************************
static inline void
EPWM_selectCycleByCycleTripZoneClearEvent(uint32_t base,
                                 EPWM_CycleByCycleTripZoneClearMode clearEvent)
{
    //
    // Set the Cycle by Cycle Trip Latch mode
    //
    HW_WR_REG16(base + CSL_EPWM_TZCLR,
        ((HW_RD_REG16(base + CSL_EPWM_TZCLR) &
        ~(uint16_t)CSL_EPWM_TZCLR_CBCPULSE_MASK) |
        (uint16_t)((uint16_t)clearEvent << CSL_EPWM_TZCLR_CBCPULSE_SHIFT)));
}

//*****************************************************************************
//
//! Clear Trip Zone flag
//!
//! \param base is the base address of the EPWM module.
//! \param tzFlags is the Trip Zone flags.
//!
//! This function clears the Trip Zone flags
//! Valid values for tzFlags are:
//!   - EPWM_TZ_INTERRUPT    - Global Trip Zone interrupt flag
//!   - EPWM_TZ_FLAG_CBC     - Trip Zones Cycle By Cycle flag
//!   - EPWM_TZ_FLAG_OST     - Trip Zones One Shot flag
//!   - EPWM_TZ_FLAG_DCAEVT1 - Digital Compare A Event 1 flag
//!   - EPWM_TZ_FLAG_DCAEVT2 - Digital Compare A Event 2 flag
//!   - EPWM_TZ_FLAG_DCBEVT1 - Digital Compare B Event 1 flag
//!   - EPWM_TZ_FLAG_DCBEVT2 - Digital Compare B Event 2 flag
//!   - EPWM_TZ_FLAG_CAPEVT  - Digital Capture Event flag
//!
//! \b note: A bitwise OR of the valid values can be passed as the tzFlags
//! parameter.
//!
//! \return None.
//
//***************************************************************************
static inline void
EPWM_clearTripZoneFlag(uint32_t base, uint16_t tzFlags)
{
    //
    // Check the arguments
    //
    DebugP_assert((tzFlags <= 0x80U) && (tzFlags >= 0x1U));

    //
    // Clear Trip zone event flag
    //
    HW_WR_REG16(base + CSL_EPWM_TZCLR,
        (HW_RD_REG16(base + CSL_EPWM_TZCLR) | tzFlags));
}

//*****************************************************************************
//
//! Clear the Trip Zone Cycle by Cycle flag.
//!
//! \param base is the base address of the EPWM module.
//! \param tzCBCFlags is the CBC flag to be cleared.
//!
//! This function clears the specific Cycle by Cycle Trip Zone flag.
//! The following are valid values for tzCBCFlags.
//!   - EPWM_TZ_CBC_FLAG_1     - CBC 1 flag
//!   - EPWM_TZ_CBC_FLAG_2     - CBC 2 flag
//!   - EPWM_TZ_CBC_FLAG_3     - CBC 3 flag
//!   - EPWM_TZ_CBC_FLAG_4     - CBC 4 flag
//!   - EPWM_TZ_CBC_FLAG_5     - CBC 5 flag
//!   - EPWM_TZ_CBC_FLAG_6     - CBC 6 flag
//!   - EPWM_TZ_CBC_FLAG_DCAEVT2  - CBC flag Digital compare
//!                                         event A2
//!   - EPWM_TZ_CBC_FLAG_DCBEVT2  - CBC flag Digital compare
//!                                         event B2
//!
//! \return None.
//
//***************************************************************************
static inline void
EPWM_clearCycleByCycleTripZoneFlag(uint32_t base, uint16_t tzCBCFlags)
{
    //
    // Check the arguments
    //
    DebugP_assert(tzCBCFlags < 0x200U);

    //
    // Clear the Cycle By Cycle Trip zone flag
    //
    HW_WR_REG16(base + CSL_EPWM_TZCBCCLR,
        (HW_RD_REG16(base + CSL_EPWM_TZCBCCLR) | tzCBCFlags));
}

//*****************************************************************************
//
//! Clear the Trip Zone One Shot flag.
//!
//! \param base is the base address of the EPWM module.
//! \param tzOSTFlags is the OST flags to be cleared.
//!
//! This function clears the specific One Shot (OST) Trip Zone flag.
//! The following are valid values for tzOSTFlags.
//!  - EPWM_TZ_OST_FLAG_OST1      - OST flag for OST1
//!  - EPWM_TZ_OST_FLAG_OST2      - OST flag for OST2
//!  - EPWM_TZ_OST_FLAG_OST3      - OST flag for OST3
//!  - EPWM_TZ_OST_FLAG_OST4      - OST flag for OST4
//!  - EPWM_TZ_OST_FLAG_OST5      - OST flag for OST5
//!  - EPWM_TZ_OST_FLAG_OST6      - OST flag for OST6
//!  - EPWM_TZ_OST_FLAG_DCAEVT1   - OST flag for Digital compare event A1
//!  - EPWM_TZ_OST_FLAG_DCBEVT1   - OST flag for Digital compare event B1
//!  - EPWM_TZ_OST_FLAG_CAPEVT    - CBC status flag for capture event
//!
//! \return None.
//
//***************************************************************************
static inline void
EPWM_clearOneShotTripZoneFlag(uint32_t base, uint16_t tzOSTFlags)
{
    //
    // Check the arguments
    //
    DebugP_assert(tzOSTFlags < 0x200U);

    //
    // Clear the One Shot Trip zone flag
    //
    HW_WR_REG16(base + CSL_EPWM_TZOSTCLR,
        (HW_RD_REG16(base + CSL_EPWM_TZOSTCLR) | tzOSTFlags));
}

//*****************************************************************************
//
//! Force Trip Zone events.
//!
//! \param base is the base address of the EPWM module.
//! \param tzForceEvent is the forced Trip Zone event.
//!
//! This function forces a Trip Zone event.
//! Valid values for tzForceEvent are:
//!   - EPWM_TZ_FORCE_EVENT_CBC     - Force Trip Zones Cycle By Cycle event
//!   - EPWM_TZ_FORCE_EVENT_OST     - Force Trip Zones One Shot Event
//!   - EPWM_TZ_FORCE_EVENT_DCAEVT1 - Force Digital Compare A Event 1
//!   - EPWM_TZ_FORCE_EVENT_DCAEVT2 - Force Digital Compare A Event 2
//!   - EPWM_TZ_FORCE_EVENT_DCBEVT1 - Force Digital Compare B Event 1
//!   - EPWM_TZ_FORCE_EVENT_DCBEVT2 - Force Digital Compare B Event 2
//!   - EPWM_TZ_FORCE_EVENT_CAPEVT  - Force Capture Event
//!
//! \return None.
//
//***************************************************************************
static inline void
EPWM_forceTripZoneEvent(uint32_t base, uint16_t tzForceEvent)
{
    //
    // Check the arguments
    //
    DebugP_assert((tzForceEvent & 0xFF01U)== 0U);

    //
    // Force a Trip Zone event
    //
    HW_WR_REG16(base + CSL_EPWM_TZFRC,
        (HW_RD_REG16(base + CSL_EPWM_TZFRC) | tzForceEvent));
}

//*****************************************************************************
//
//! Enable Trip Zone outputs.
//!
//! \param base is the base address of the EPWM module.
//! \param tzOutput is the mask of Trip Zone signals to be enabled.
//!
//! This function enables Trip Zone outputs. We can enable multiple signals by
//! passing a mask to the function. Mask can be created by doing logical OR
//! of the following macros:
//! - EPWM_TZ_SELECT_TRIPOUT_OST, EPWM_TZ_SELECT_TRIPOUT_CBC,
//! - EPWM_TZ_SELECT_TRIPOUT_TZ[1-6], EPWM_TZ_SELECT_TRIPOUT_CAPEVT
//! - EPWM_TZ_SELECT_TRIPOUT_DC[A-B]EVT[1-2].
//!
//! \return None.
//
//***************************************************************************
static inline void
EPWM_enableTripZoneOutput(uint32_t base, uint16_t tzOutput)
{
    //
    // Enable the Trip Zone signals as output
    //
    HW_WR_REG16(base + CSL_EPWM_TZTRIPOUTSEL,
        (HW_RD_REG16(base + CSL_EPWM_TZTRIPOUTSEL) | tzOutput));
}

//*****************************************************************************
//
//! Disable Trip Zone outputs.
//!
//! \param base is the base address of the EPWM module.
//! \param tzOutput is the mask of Trip Zone signals to be disabled.
//!
//! This function disables Trip Zone outputs. We can disable multiple signals
//! by passing a mask to the function. Mask can be created by doing logical OR
//! of the following macros:
//! - EPWM_TZ_SELECT_TRIPOUT_OST, EPWM_TZ_SELECT_TRIPOUT_CBC,
//! - EPWM_TZ_SELECT_TRIPOUT_TZ[1-6], EPWM_TZ_SELECT_TRIPOUT_CAPEVT
//! - EPWM_TZ_SELECT_TRIPOUT_DC[A-B]EVT[1-2].
//!
//! \return None.
//
//***************************************************************************
static inline void
EPWM_disableTripZoneOutput(uint32_t base, uint16_t tzOutput)
{
    //
    // Disable the Trip Zone signals as output
    //
    HW_WR_REG16(base + CSL_EPWM_TZTRIPOUTSEL,
        (HW_RD_REG16(base + CSL_EPWM_TZTRIPOUTSEL) & ~tzOutput));
}

//
// Event Trigger related APIs
//
//*****************************************************************************
//
//! Enable ePWM interrupt.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the ePWM interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableInterrupt(uint32_t base)
{
    //
    // Enable ePWM interrupt
    //
    HW_WR_REG16(base + CSL_EPWM_ETSEL,
        (HW_RD_REG16(base + CSL_EPWM_ETSEL) | CSL_EPWM_ETSEL_INTEN_MASK));
}

//*****************************************************************************
//
//! disable ePWM interrupt.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the ePWM interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableInterrupt(uint32_t base)
{
    //
    // Disable ePWM interrupt
    //
    HW_WR_REG16(base + CSL_EPWM_ETSEL,
        (HW_RD_REG16(base + CSL_EPWM_ETSEL) & ~CSL_EPWM_ETSEL_INTEN_MASK));
}

//*****************************************************************************
//
//! Sets the ePWM interrupt source.
//!
//! \param base is the base address of the EPWM module.
//! \param interruptSource is the ePWM interrupt source.
//! \param mixedSource is the set of mixed signals to be enabled.
//!
//! This function sets the ePWM interrupt source.
//! Valid values for interruptSource are:
//!   - EPWM_INT_TBCTR_ZERO           - Time-base counter equal to zero
//!   - EPWM_INT_TBCTR_PERIOD         - Time-base counter equal to period
//!   - EPWM_INT_TBCTR_ETINT_MIX      - Time-base counter event according
//!                                         to mix event
//!   - EPWM_INT_TBCTR_U_CMPx         - Where x is A,B,C or D
//!                                     Time-base counter equal to CMPA, CMPB,
//!                                     CMPC or CMPD (depending the value of x)
//!                                     when the timer is incrementing
//!   - EPWM_INT_TBCTR_D_CMPx         - Where x is A,B,C or D
//!                                     Time-base counter equal to CMPA, CMPB,
//!                                     CMPC or CMPD (depending the value of x)
//!                                     when the timer is decrementing
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setInterruptSource(uint32_t base, uint16_t interruptSource,
                        uint16_t mixedSource)
{
    uint16_t intSource;

    //
    // Check the arguments
    //
    DebugP_assert(((interruptSource > 0U) && (interruptSource < 9U)) ||
           (interruptSource == 10U) || (interruptSource == 12U) ||
           (interruptSource == 14U));

    if((interruptSource == EPWM_INT_TBCTR_U_CMPC) ||
       (interruptSource == EPWM_INT_TBCTR_U_CMPD) ||
       (interruptSource == EPWM_INT_TBCTR_D_CMPC) ||
       (interruptSource == EPWM_INT_TBCTR_D_CMPD))
    {
        //
        // Shift the interrupt source by 1
        //
        intSource = interruptSource >> 1U;

        //
        // Enable events based on comp C or comp D
        //
        HW_WR_REG16(base + CSL_EPWM_ETSEL,
            (HW_RD_REG16(base + CSL_EPWM_ETSEL) |
            CSL_EPWM_ETSEL_INTSELCMP_MASK));
    }
    else if((interruptSource == EPWM_INT_TBCTR_U_CMPA) ||
            (interruptSource == EPWM_INT_TBCTR_U_CMPB) ||
            (interruptSource == EPWM_INT_TBCTR_D_CMPA) ||
            (interruptSource == EPWM_INT_TBCTR_D_CMPB))
    {
        intSource = interruptSource;

        //
        // Enable events based on comp A or comp B
        //
        HW_WR_REG16(base + CSL_EPWM_ETSEL,
            (HW_RD_REG16(base + CSL_EPWM_ETSEL) &
            ~CSL_EPWM_ETSEL_INTSELCMP_MASK));
    }
    else if(interruptSource == EPWM_INT_TBCTR_ETINTMIX)
    {
        intSource = interruptSource;

        //
        // Enable mixed events
        //
        HW_WR_REG16(base + CSL_EPWM_ETINTMIXEN, mixedSource);
    }
    else
    {
        intSource = interruptSource;
    }

    //
    // Set the interrupt source
    //
    HW_WR_REG16(base + CSL_EPWM_ETSEL,
        ((HW_RD_REG16(base + CSL_EPWM_ETSEL) &
        ~CSL_EPWM_ETSEL_INTSEL_MASK) | intSource));
}

//*****************************************************************************
//
//! Sets the ePWM interrupt event counts.
//!
//! \param base is the base address of the EPWM module.
//! \param eventCount is the event count for interrupt scale
//!
//! This function sets the interrupt event count that determines the number of
//! events that have to occur before an interrupt is issued.
//! Maximum value for eventCount is 15.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setInterruptEventCount(uint32_t base, uint16_t eventCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(eventCount <= CSL_EPWM_ETINTPS_INTPRD2_MAX);

    //
    // Enable advanced feature of interrupt every up to 15 events
    //
    HW_WR_REG16(base + CSL_EPWM_ETPS,
        (HW_RD_REG16(base + CSL_EPWM_ETPS) | CSL_EPWM_ETPS_INTPSSEL_MASK));

    HW_WR_REG16(base + CSL_EPWM_ETINTPS,
        ((HW_RD_REG16(base + CSL_EPWM_ETINTPS) &
        ~CSL_EPWM_ETINTPS_INTPRD2_MASK) | eventCount));
}

//*****************************************************************************
//
//! Return the interrupt status.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the ePWM interrupt status.
//! \b Note This function doesn't return the Trip Zone status.
//!
//! \return Returns true if ePWM interrupt was generated.
//!         Returns false if no interrupt was generated
//
//*****************************************************************************
static inline bool
EPWM_getEventTriggerInterruptStatus(uint32_t base)
{
    //
    // Return INT bit of ETFLG register
    //
    return(((HW_RD_REG16(base + CSL_EPWM_ETFLG) & 0x1U) ==
            0x1U) ? true : false);
}

//*****************************************************************************
//
//! Clear interrupt flag.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function clears the ePWM interrupt flag.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_clearEventTriggerInterruptFlag(uint32_t base)
{
    //
    // Clear INT bit of ETCLR register
    //
    HW_WR_REG16(base + CSL_EPWM_ETCLR, (CSL_EPWM_ETCLR_INT_MASK));
}

//*****************************************************************************
//
//! Enable Pre-interrupt count load.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the ePWM interrupt counter to be pre-interrupt loaded
//! with a count value.
//!
//! \note This is valid only for advanced/expanded interrupt mode
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableInterruptEventCountInit(uint32_t base)
{
    //
    // Enable interrupt event count initializing/loading
    //
    HW_WR_REG16(base + CSL_EPWM_ETCNTINITCTL,
        (HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL) |
        CSL_EPWM_ETCNTINITCTL_INTINITEN_MASK));
}

//*****************************************************************************
//
//! Disable interrupt count load.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the ePWM interrupt counter from being loaded with
//! pre-interrupt count value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableInterruptEventCountInit(uint32_t base)
{
    //
    // Disable interrupt event count initializing/loading
    //
    HW_WR_REG16(base + CSL_EPWM_ETCNTINITCTL,
        (HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL) &
        ~CSL_EPWM_ETCNTINITCTL_INTINITEN_MASK));
}

//*****************************************************************************
//
//! Force a software pre interrupt event counter load.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function forces the ePWM interrupt counter to be loaded with the
//! contents set by EPWM_setPreInterruptEventCount().
//!
//! \note make sure the EPWM_enablePreInterruptEventCountLoad() function is
//!       is called before invoking this function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_forceInterruptEventCountInit(uint32_t base)
{
    //
    // Load the Interrupt Event counter value
    //
    HW_WR_REG16(base + CSL_EPWM_ETCNTINITCTL,
        (HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL) |
        CSL_EPWM_ETCNTINITCTL_INTINITFRC_MASK));
}

//*****************************************************************************
//
//! Set interrupt count.
//!
//! \param base is the base address of the EPWM module.
//! \param eventCount is the ePWM interrupt count value.
//!
//! This function sets the ePWM interrupt count. eventCount is the value of the
//! pre-interrupt value that is to be loaded. The maximum value of eventCount
//! is 15.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setInterruptEventCountInitValue(uint32_t base, uint16_t eventCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(eventCount <= CSL_EPWM_ETCNTINIT_INTINIT_MAX);

    //
    // Set the Pre-interrupt event count
    //
    HW_WR_REG16(base + CSL_EPWM_ETCNTINIT,
        ((HW_RD_REG16(base + CSL_EPWM_ETCNTINIT) &
        ~CSL_EPWM_ETCNTINIT_INTINIT_MASK) |
        (uint16_t)(eventCount & CSL_EPWM_ETCNTINIT_INTINIT_MASK)));
}

//*****************************************************************************
//
//! Get the interrupt count.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the ePWM interrupt event count.
//!
//! \return The interrupt event counts that have occurred.
//
//*****************************************************************************
static inline uint16_t
EPWM_getInterruptEventCount(uint32_t base)
{
    //
    // Return the interrupt event count
    //
    return(((HW_RD_REG16(base + CSL_EPWM_ETINTPS) &
            CSL_EPWM_ETINTPS_INTCNT2_MASK) >>
            CSL_EPWM_ETINTPS_INTCNT2_SHIFT));
}

//*****************************************************************************
//
//! Force ePWM interrupt.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function forces an ePWM interrupt.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_forceEventTriggerInterrupt(uint32_t base)
{
    //
    // Set INT bit of ETFRC register
    //
    HW_WR_REG16(base + CSL_EPWM_ETFRC,
        (HW_RD_REG16(base + CSL_EPWM_ETFRC) | CSL_EPWM_ETFRC_INT_MASK));
}

//
// ADC SOC configuration related APIs
//
//*****************************************************************************
//
//! Enable ADC SOC event.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function enables the ePWM module to trigger an ADC SOC event.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableADCTrigger(uint32_t base, EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Enable an SOC
    //
    if(adcSOCType == EPWM_SOC_A)
    {
        HW_WR_REG16(base + CSL_EPWM_ETSEL,
        (HW_RD_REG16(base + CSL_EPWM_ETSEL) | CSL_EPWM_ETSEL_SOCAEN_MASK));
    }
    else
    {
        HW_WR_REG16(base + CSL_EPWM_ETSEL,
        (HW_RD_REG16(base + CSL_EPWM_ETSEL) | CSL_EPWM_ETSEL_SOCBEN_MASK));
    }
}

//*****************************************************************************
//
//! Disable ADC SOC event.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function disables the ePWM module from triggering an ADC SOC event.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableADCTrigger(uint32_t base, EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Disable an SOC
    //
    if(adcSOCType == EPWM_SOC_A)
    {
        HW_WR_REG16(base + CSL_EPWM_ETSEL,
        (HW_RD_REG16(base + CSL_EPWM_ETSEL) & ~CSL_EPWM_ETSEL_SOCAEN_MASK));
    }
    else
    {
        HW_WR_REG16(base + CSL_EPWM_ETSEL,
        (HW_RD_REG16(base + CSL_EPWM_ETSEL) & ~CSL_EPWM_ETSEL_SOCBEN_MASK));
    }
}

//*****************************************************************************
//
//! Sets the ePWM SOC source.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//! \param socSource is the SOC source.
//! \param mixedSource is the set of mixed signals to be enabled.
//!
//! This function sets the ePWM ADC SOC source.
//! Valid values for socSource are:
//!  - adcSOCType
//!     - EPWM_SOC_A  - SOC A
//!     - EPWM_SOC_B  - SOC B
//!  - socSource
//!     - EPWM_SOC_DCxEVT1              - Event is based on DCxEVT1
//!     - EPWM_SOC_TBCTR_ZERO           - Time-base counter equal to zero
//!     - EPWM_SOC_TBCTR_PERIOD         - Time-base counter equal to period
//!     - EPWM_SOC_TBCTR_ZERO_OR_PERIOD - Time-base counter equal to zero or
//!                                       period
//!     - EPWM_SOC_TBCTR_U_CMPx        - Where x is A,B,C or D
//!                                      Time-base counter equal to CMPA, CMPB,
//!                                      CMPC or CMPD(depending the value of x)
//!                                      when the timer is incrementing
//!     - EPWM_SOC_TBCTR_D_CMPx        - Where x is A,B,C or D
//!                                      Time-base counter equal to CMPA, CMPB,
//!                                      CMPC or CMPD(depending the value of x)
//!                                      when the timer is decrementing
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setADCTriggerSource(uint32_t base,
                         EPWM_ADCStartOfConversionType adcSOCType,
                         EPWM_ADCStartOfConversionSource socSource,
                         uint16_t mixedSource)
{
    uint16_t source;

    if((socSource == EPWM_SOC_TBCTR_U_CMPC) ||
       (socSource == EPWM_SOC_TBCTR_U_CMPD) ||
       (socSource == EPWM_SOC_TBCTR_D_CMPC) ||
       (socSource == EPWM_SOC_TBCTR_D_CMPD))
    {
        source = (uint16_t)socSource >> 1U;
    }
    else
    {
        source = (uint16_t)socSource;
    }

    if(adcSOCType == EPWM_SOC_A)
    {
        //
        // Set the SOC source
        //
        HW_WR_REG16(base + CSL_EPWM_ETSEL,
            ((HW_RD_REG16(base + CSL_EPWM_ETSEL) &
            ~CSL_EPWM_ETSEL_SOCASEL_MASK) |
            ((uint32_t)source << CSL_EPWM_ETSEL_SOCASEL_SHIFT)));

        //
        // Enable the comparator selection
        //
        if((socSource == EPWM_SOC_TBCTR_U_CMPA) ||
           (socSource == EPWM_SOC_TBCTR_U_CMPB) ||
           (socSource == EPWM_SOC_TBCTR_D_CMPA) ||
           (socSource == EPWM_SOC_TBCTR_D_CMPB))
        {
            //
            // Enable events based on comp A or comp B
            //
            HW_WR_REG16(base + CSL_EPWM_ETSEL,
            (HW_RD_REG16(base + CSL_EPWM_ETSEL) &
            ~CSL_EPWM_ETSEL_SOCASELCMP_MASK));
        }
        else if((socSource == EPWM_SOC_TBCTR_U_CMPC) ||
                (socSource == EPWM_SOC_TBCTR_U_CMPD) ||
                (socSource == EPWM_SOC_TBCTR_D_CMPC) ||
                (socSource == EPWM_SOC_TBCTR_D_CMPD))
        {
            //
            // Enable events based on comp C or comp D
            //
            HW_WR_REG16(base + CSL_EPWM_ETSEL,
            (HW_RD_REG16(base + CSL_EPWM_ETSEL) |
            CSL_EPWM_ETSEL_SOCASELCMP_MASK));
        }
        else if(socSource == EPWM_SOC_TBCTR_MIXED_EVENT)
        {
            //
            // Enable mixed events
            //
            HW_WR_REG16(base + CSL_EPWM_ETSOCAMIXEN, mixedSource);
        }
        else
        {
            //
            // No action required for the other socSource options
            //
        }
    }
    else
    {
        //
        // Enable the comparator selection
        //
        HW_WR_REG16(base + CSL_EPWM_ETSEL,
            ((HW_RD_REG16(base + CSL_EPWM_ETSEL) &
            ~CSL_EPWM_ETSEL_SOCBSEL_MASK) |
            ((uint32_t)source << CSL_EPWM_ETSEL_SOCBSEL_SHIFT)));

        //
        // Enable the comparator selection
        //
        if((socSource == EPWM_SOC_TBCTR_U_CMPA) ||
           (socSource == EPWM_SOC_TBCTR_U_CMPB) ||
           (socSource == EPWM_SOC_TBCTR_D_CMPA) ||
           (socSource == EPWM_SOC_TBCTR_D_CMPB))
        {
            //
            // Enable events based on comp A or comp B
            //
            HW_WR_REG16(base + CSL_EPWM_ETSEL,
                (HW_RD_REG16(base + CSL_EPWM_ETSEL) &
                ~CSL_EPWM_ETSEL_SOCBSELCMP_MASK));
        }
        else if((socSource == EPWM_SOC_TBCTR_U_CMPC) ||
                (socSource == EPWM_SOC_TBCTR_U_CMPD) ||
                (socSource == EPWM_SOC_TBCTR_D_CMPC) ||
                (socSource == EPWM_SOC_TBCTR_D_CMPD))
        {
            //
            // Enable events based on comp C or comp D
            //
            HW_WR_REG16(base + CSL_EPWM_ETSEL,
                (HW_RD_REG16(base + CSL_EPWM_ETSEL) |
                CSL_EPWM_ETSEL_SOCBSELCMP_MASK));
        }
        else if(socSource == EPWM_SOC_TBCTR_MIXED_EVENT)
        {
            //
            // Enable mixed events
            //
            HW_WR_REG16(base + CSL_EPWM_ETSOCBMIXEN, mixedSource);
        }
        else
        {
            //
            // No action required for the other socSource options
            //
        }
    }
}

//*****************************************************************************
//
//! Sets the ePWM SOC event counts.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//! \param preScaleCount is the event count number.
//!
//! This function sets the SOC event count that determines the number of
//! events that have to occur before an SOC is issued.
//!  Valid values for the parameters are:
//!   - adcSOCType
//!       - EPWM_SOC_A  - SOC A
//!       - EPWM_SOC_B  - SOC B
//!   - preScaleCount
//!        - [1 - 15]  -  Generate SOC pulse every preScaleCount
//!                       up to 15 events.
//!
//! \note A preScaleCount value of 0 disables the prescale.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setADCTriggerEventPrescale(uint32_t base,
                                EPWM_ADCStartOfConversionType adcSOCType,
                                uint16_t preScaleCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(preScaleCount <= CSL_EPWM_ETSOCPS_SOCAPRD2_MAX);

    //
    // Enable advanced feature of SOC every up to 15 events
    //
    HW_WR_REG16(base + CSL_EPWM_ETPS,
        (HW_RD_REG16(base + CSL_EPWM_ETPS) |
        CSL_EPWM_ETPS_SOCPSSEL_MASK));

    if(adcSOCType == EPWM_SOC_A)
    {
        //
        // Set the count for SOC A
        //
        HW_WR_REG16(base + CSL_EPWM_ETSOCPS,
            ((HW_RD_REG16(base + CSL_EPWM_ETSOCPS) &
            ~CSL_EPWM_ETSOCPS_SOCAPRD2_MASK) |
            preScaleCount));
    }
    else
    {
        //
        // Set the count for SOC B
        //
        HW_WR_REG16(base + CSL_EPWM_ETSOCPS,
            ((HW_RD_REG16(base + CSL_EPWM_ETSOCPS) &
            ~CSL_EPWM_ETSOCPS_SOCBPRD2_MASK) |
            ((uint32_t)preScaleCount << CSL_EPWM_ETSOCPS_SOCBPRD2_SHIFT)));
    }
}

//*****************************************************************************
//
//! Return the SOC event status.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function returns the ePWM SOC status.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \return Returns true if the selected adcSOCType SOC was generated.
//!         Returns false if the selected adcSOCType SOC was not generated.
//
//*****************************************************************************
static inline bool
EPWM_getADCTriggerFlagStatus(uint32_t base,
                             EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Return the SOC A/ B status
    //
    return((((HW_RD_REG16(base + CSL_EPWM_ETFLG) >>
            ((uint16_t)adcSOCType + CSL_EPWM_ETFLG_SOCA_SHIFT)) &
            0x1U) == 0x1U) ? true : false);
}

//*****************************************************************************
//
//! Clear SOC flag.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function clears the ePWM SOC flag.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_clearADCTriggerFlag(uint32_t base,
                         EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Clear SOC A/B bit of ETCLR register
    //
    HW_WR_REG16(base + CSL_EPWM_ETCLR,
        (HW_RD_REG16(base + CSL_EPWM_ETCLR) |
        ((uint16_t)1U << ((uint16_t)adcSOCType + CSL_EPWM_ETCLR_SOCA_SHIFT))));
}

//*****************************************************************************
//
//! Enable Pre-SOC event count load.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function enables the ePWM SOC event counter which is set by the
//! EPWM_setADCTriggerEventCountInitValue() function to be loaded before
//! an SOC event.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \note This is valid only for advanced/expanded SOC mode
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableADCTriggerEventCountInit(uint32_t base,
                                    EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Enable SOC event count initializing/loading
    //
    HW_WR_REG16(base + CSL_EPWM_ETCNTINITCTL,
        (HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL) | ((uint16_t)1U <<
        ((uint16_t)adcSOCType + CSL_EPWM_ETCNTINITCTL_SOCAINITEN_SHIFT))));
}

//*****************************************************************************
//
//! Disable Pre-SOC event count load.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function disables the ePWM SOC event counter from being loaded before
//! an SOC event (only an SOC event causes an increment of the counter value).
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \note This is valid only for advanced/expanded SOC mode
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableADCTriggerEventCountInit(uint32_t base,
                                     EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Disable SOC event count initializing/loading
    //
    HW_WR_REG16(base + CSL_EPWM_ETCNTINITCTL,
        (HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL) &
        ~(1U << ((uint16_t)adcSOCType +
        CSL_EPWM_ETCNTINITCTL_SOCAINITEN_SHIFT))));
}

//*****************************************************************************
//
//! Force a software pre SOC event counter load.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type
//!
//! This function forces the ePWM SOC counter to be loaded with the
//! contents set by EPWM_setPreADCStartOfConversionEventCount().
//!
//! \note make sure the EPWM_enableADCTriggerEventCountInit()
//!       function is called before invoking this function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_forceADCTriggerEventCountInit(uint32_t base,
                                   EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Load the Interrupt Event counter value
    //
    HW_WR_REG16(base + CSL_EPWM_ETCNTINITCTL,
        (HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL) |
        ((uint16_t)1U << ((uint16_t)adcSOCType +
        CSL_EPWM_ETCNTINITCTL_SOCAINITFRC_SHIFT))));
}

//*****************************************************************************
//
//! Set ADC Trigger count values.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//! \param eventCount is the ePWM interrupt count value.
//!
//! This function sets the ePWM ADC Trigger count values.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//! The eventCount has a maximum value of 15.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setADCTriggerEventCountInitValue(uint32_t base,
                                      EPWM_ADCStartOfConversionType adcSOCType,
                                      uint16_t eventCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(eventCount <= CSL_EPWM_ETCNTINIT_SOCAINIT_MAX);

    //
    // Set the ADC Trigger event count
    //
    if(adcSOCType == EPWM_SOC_A)
    {
        HW_WR_REG16(base + CSL_EPWM_ETCNTINIT,
            ((HW_RD_REG16(base + CSL_EPWM_ETCNTINIT) &
            ~CSL_EPWM_ETCNTINIT_SOCAINIT_MASK) |
            (uint16_t)(eventCount << CSL_EPWM_ETCNTINIT_SOCAINIT_SHIFT)));
    }
    else
    {
        HW_WR_REG16(base + CSL_EPWM_ETCNTINIT,
            ((HW_RD_REG16(base + CSL_EPWM_ETCNTINIT) &
            ~CSL_EPWM_ETCNTINIT_SOCBINIT_MASK) |
            ((uint32_t)eventCount << CSL_EPWM_ETCNTINIT_SOCBINIT_SHIFT)));
    }
}

//*****************************************************************************
//
//! Get the SOC event count.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function returns the ePWM SOC event count.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \return The SOC event counts that have occurred.
//
//*****************************************************************************
static inline uint16_t
EPWM_getADCTriggerEventCount(uint32_t base,
                             EPWM_ADCStartOfConversionType adcSOCType)
{
    uint16_t eventCount;

    //
    // Return the SOC event count
    //
    if(adcSOCType == EPWM_SOC_A)
    {
        eventCount = (HW_RD_REG16(base + CSL_EPWM_ETSOCPS) >>
                     CSL_EPWM_ETSOCPS_SOCACNT2_SHIFT) &
                     CSL_EPWM_ETSOCPS_SOCACNT2_MAX;
    }
    else
    {
        eventCount = (HW_RD_REG16(base + CSL_EPWM_ETSOCPS) >>
                     CSL_EPWM_ETSOCPS_SOCBCNT2_SHIFT) &
                     CSL_EPWM_ETSOCPS_SOCBCNT2_MAX;
    }

    return(eventCount);
}

//*****************************************************************************
//
//! Force SOC event.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function forces an ePWM SOC event.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_forceADCTrigger(uint32_t base, EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Set SOC A/B bit of ETFRC register
    //
    HW_WR_REG16(base + CSL_EPWM_ETFRC,
        (HW_RD_REG16(base + CSL_EPWM_ETFRC) |
        ((uint16_t)1U << ((uint16_t)adcSOCType + CSL_EPWM_ETFRC_SOCA_SHIFT))));
}

//
// Digital Compare module related APIs
//
//*****************************************************************************
//
//! Set the DC trip input.
//!
//! \param base is the base address of the EPWM module.
//! \param tripSource is the tripSource.
//! \param dcType is the Digital Compare type.
//!
//! This function sets the trip input to the Digital Compare (DC). For a given
//! dcType the function sets the tripSource to be the input to the DC.
//! Valid values for the parameter are:
//!  - tripSource
//!     - EPWM_DC_TRIP_TRIPINx -Trip x,where x ranges from 1 to 15
//!     - EPWM_DC_TRIP_COMBINATION - selects all the Trip signals whose input
//!                                  is enabled by the following function
//!                             EPWM_enableDigitalCompareTripCombinationInput()
//!  - dcType
//!     - EPWM_DC_TYPE_DCAH  - Digital Compare A High
//!     - EPWM_DC_TYPE_DCAL  - Digital Compare A Low
//!     - EPWM_DC_TYPE_DCBH  - Digital Compare B High
//!     - EPWM_DC_TYPE_DCBL  - Digital Compare B Low
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_selectDigitalCompareTripInput(uint32_t base,
                                   EPWM_DigitalCompareTripInput tripSource,
                                   EPWM_DigitalCompareType dcType)
{
    //
    // Set the DC trip input
    //
    HW_WR_REG16(base + CSL_EPWM_DCTRIPSEL,
        ((HW_RD_REG16(base + CSL_EPWM_DCTRIPSEL) &
        ~(CSL_EPWM_DCTRIPSEL_DCAHCOMPSEL_MASK <<
        ((uint16_t)dcType << 2U))) |
        ((uint16_t)tripSource << ((uint16_t)dcType << 2U))));
}

//
// DCFILT
//
//*****************************************************************************
//
//! Enable DC filter blanking window.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the DC filter blanking window.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareBlankingWindow(uint32_t base)
{
    //
    // Enable DC filter blanking window
    //
    HW_WR_REG16(base + CSL_EPWM_DCFCTL,
        (HW_RD_REG16(base + CSL_EPWM_DCFCTL) | CSL_EPWM_DCFCTL_BLANKE_MASK));
}

//*****************************************************************************
//
//! Disable DC filter blanking window.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the DC filter blanking window.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareBlankingWindow(uint32_t base)
{
    //
    // Disable DC filter blanking window
    //
    HW_WR_REG16(base + CSL_EPWM_DCFCTL,
        (HW_RD_REG16(base + CSL_EPWM_DCFCTL) & ~CSL_EPWM_DCFCTL_BLANKE_MASK));
}

//*****************************************************************************
//
//! Enable Digital Compare Window inverse mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the Digital Compare Window inverse mode. This will
//! invert the blanking window.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareWindowInverseMode(uint32_t base)
{
    //
    // Enable DC window inverse mode.
    //
    HW_WR_REG16(base + CSL_EPWM_DCFCTL,
        (HW_RD_REG16(base + CSL_EPWM_DCFCTL) | CSL_EPWM_DCFCTL_BLANKINV_MASK));
}

//*****************************************************************************
//
//! Disable Digital Compare Window inverse mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the Digital Compare Window inverse mode.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareWindowInverseMode(uint32_t base)
{
    //
    // Disable DC window inverse mode.
    //
    HW_WR_REG16(base + CSL_EPWM_DCFCTL,
        (HW_RD_REG16(base + CSL_EPWM_DCFCTL) &
        ~CSL_EPWM_DCFCTL_BLANKINV_MASK));
}

//*****************************************************************************
//
//! Set the Digital Compare filter blanking pulse.
//!
//! \param base is the base address of the EPWM module.
//! \param blankingPulse is Pulse that starts blanking window.
//! \param mixedSource is the set of mixed signals to be enabled.
//!
//! This function sets the input pulse that starts the Digital Compare blanking
//! window.
//! Valid values for blankingPulse are:
//!   - EPWM_DC_WINDOW_START_TBCTR_PERIOD - Time base counter equals period
//!   - EPWM_DC_WINDOW_START_TBCTR_ZERO   - Time base counter equals zero
//!   - EPWM_DC_WINDOW_START_TBCTR_ZERO_PERIOD - Time base counter equals zero
//!                                              or period.
//!   - EPWM_DC_WINDOW_START_TBCTR_BLANK_PULSE_MIX - Time base counter blank pulse mix
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareBlankingEvent(uint32_t base,
                                EPWM_DigitalCompareBlankingPulse blankingPulse,
                                uint16_t mixedSource)
{
    if(blankingPulse == EPWM_DC_WINDOW_START_TBCTR_BLANK_PULSE_MIX)
    {
        //
        // Enable mixed events
        //
        HW_WR_REG16(base + CSL_EPWM_BLANKPULSEMIXSEL, mixedSource);
    }

    //
    // Set DC blanking event
    //
    HW_WR_REG16(base + CSL_EPWM_DCFCTL,
        ((HW_RD_REG16(base + CSL_EPWM_DCFCTL) &
        ~CSL_EPWM_DCFCTL_PULSESEL_MASK) |
        ((uint16_t)((uint32_t)blankingPulse <<
        CSL_EPWM_DCFCTL_PULSESEL_SHIFT))));
}

//*****************************************************************************
//
//! Set up the Digital Compare filter input.
//!
//! \param base is the base address of the EPWM module.
//! \param filterInput is Digital Compare signal source.
//!
//! This function sets the signal input source that will be filtered by the
//! Digital Compare module.
//! Valid values for filterInput are:
//!   - EPWM_DC_WINDOW_SOURCE_DCAEVT1  - DC filter signal source is DCAEVT1
//!   - EPWM_DC_WINDOW_SOURCE_DCAEVT2  - DC filter signal source is DCAEVT2
//!   - EPWM_DC_WINDOW_SOURCE_DCBEVT1  - DC filter signal source is DCBEVT1
//!   - EPWM_DC_WINDOW_SOURCE_DCBEVT2  - DC filter signal source is DCBEVT2
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareFilterInput(uint32_t base,
                                  EPWM_DigitalCompareFilterInput filterInput)
{
    //
    // Set the signal source that will be filtered
    //
    HW_WR_REG16(base + CSL_EPWM_DCFCTL,
        ((HW_RD_REG16(base + CSL_EPWM_DCFCTL) & ~CSL_EPWM_DCFCTL_SRCSEL_MASK) |
        ((uint16_t)filterInput)));
}

//
// DC Edge Filter
//
//*****************************************************************************
//
//! Enable Digital Compare Edge Filter.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the Digital Compare Edge filter to generate event
//! after configured number of edges.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareEdgeFilter(uint32_t base)
{
    //
    // Enable DC Edge Filter
    //
    HW_WR_REG16(base + CSL_EPWM_DCFCTL,
        (HW_RD_REG16(base + CSL_EPWM_DCFCTL) |
        CSL_EPWM_DCFCTL_EDGEFILTSEL_MASK));
}

//*****************************************************************************
//
//! Disable Digital Compare Edge Filter.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the Digital Compare Edge filter.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareEdgeFilter(uint32_t base)
{
    //
    // Disable DC Edge Filter
    //
    HW_WR_REG16(base + CSL_EPWM_DCFCTL,
        (HW_RD_REG16(base + CSL_EPWM_DCFCTL) &
        ~CSL_EPWM_DCFCTL_EDGEFILTSEL_MASK));
}

//*****************************************************************************
//
//! Set the Digital Compare Edge Filter Mode.
//!
//! \param base is the base address of the EPWM module.
//! \param edgeMode is Digital Compare Edge filter mode.
//!
//! This function sets the Digital Compare Event filter mode. Valid values
//! for edgeMode are:
//!   - EPWM_DC_EDGEFILT_MODE_RISING  - DC edge filter mode is rising edge
//!   - EPWM_DC_EDGEFILT_MODE_FALLING - DC edge filter mode is falling edge
//!   - EPWM_DC_EDGEFILT_MODE_BOTH    - DC edge filter mode is both edges
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareEdgeFilterMode(uint32_t base,
                                    EPWM_DigitalCompareEdgeFilterMode edgeMode)
{
    //
    // Set DC Edge filter mode
    //
    HW_WR_REG16(base + CSL_EPWM_DCFCTL,
        (HW_RD_REG16(base + CSL_EPWM_DCFCTL) &
        ~CSL_EPWM_DCFCTL_EDGEMODE_MASK) |
        ((uint32_t)edgeMode << CSL_EPWM_DCFCTL_EDGEMODE_SHIFT));
}

//*****************************************************************************
//
//! Set the Digital Compare Edge Filter Edge Count.
//!
//! \param base is the base address of the EPWM module.
//! \param edgeCount is Digital Compare event filter count
//!
//! This function sets the Digital Compare Event filter Edge Count to generate
//! events. Valid values for edgeCount can be:
//!  - EPWM_DC_EDGEFILT_EDGECNT_0 - No edge is required to generate event
//!  - EPWM_DC_EDGEFILT_EDGECNT_1 - 1 edge is required for event generation
//!  - EPWM_DC_EDGEFILT_EDGECNT_2 - 2 edges are required for event generation
//!  - EPWM_DC_EDGEFILT_EDGECNT_3 - 3 edges are required for event generation
//!  - EPWM_DC_EDGEFILT_EDGECNT_4 - 4 edges are required for event generation
//!  - EPWM_DC_EDGEFILT_EDGECNT_5 - 5 edges are required for event generation
//!  - EPWM_DC_EDGEFILT_EDGECNT_6 - 6 edges are required for event generation
//!  - EPWM_DC_EDGEFILT_EDGECNT_7 - 7 edges are required for event generation
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareEdgeFilterEdgeCount(uint32_t base,
                            EPWM_DigitalCompareEdgeFilterEdgeCount edgeCount)
{
    //
    // Set DC Edge filter edge count
    //
    HW_WR_REG16(base + CSL_EPWM_DCFCTL,
        (HW_RD_REG16(base + CSL_EPWM_DCFCTL) &
        ~CSL_EPWM_DCFCTL_EDGECOUNT_MASK) |
        ((uint32_t)edgeCount << CSL_EPWM_DCFCTL_EDGECOUNT_SHIFT));
}

//*****************************************************************************
//
//! Returns the Digital Compare Edge Filter Edge Count.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the configured Digital Compare Edge filter edge
//! count required to generate events. It can return values from 0-7.
//!
//! \return Returns the configured DigitalCompare Edge filter edge count.
//
//*****************************************************************************
static inline uint16_t
EPWM_getDigitalCompareEdgeFilterEdgeCount(uint32_t base)
{
    //
    // Return configured DC edge filter edge count
    //
    return((HW_RD_REG16(base + CSL_EPWM_DCFCTL) &
            CSL_EPWM_DCFCTL_EDGECOUNT_MASK) >>
            CSL_EPWM_DCFCTL_EDGECOUNT_SHIFT);
}

//*****************************************************************************
//
//! Returns the Digital Compare Edge filter captured edge count status.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the count of edges captured by Digital Compare Edge
//! filter. It can return values from 0-7.
//!
//! \return Returns the count of captured edges
//
//*****************************************************************************
static inline uint16_t
EPWM_getDigitalCompareEdgeFilterEdgeStatus(uint32_t base)
{
    //
    // Return captured edge count by DC Edge filter
    //
    return((HW_RD_REG16(base + CSL_EPWM_DCFCTL) &
            CSL_EPWM_DCFCTL_EDGESTATUS_MASK) >>
            CSL_EPWM_DCFCTL_EDGESTATUS_SHIFT);
}

//*****************************************************************************
//
//! Set up the Digital Compare filter window offset
//!
//! \param base is the base address of the EPWM module.
//! \param windowOffsetCount is blanking window offset length.
//!
//! This function sets the offset between window start pulse and blanking
//! window in TBCLK count.
//! The function take a 16bit count value for the offset value.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareWindowOffset(uint32_t base, uint16_t windowOffsetCount)
{
    //
    // Set the blanking window offset in TBCLK counts
    //
    HW_WR_REG16(base + CSL_EPWM_DCFOFFSET, windowOffsetCount);
}

//*****************************************************************************
//
//! Set up the Digital Compare filter window length
//!
//! \param base is the base address of the EPWM module.
//! \param windowLengthCount is blanking window length.
//!
//! This function sets up the Digital Compare filter blanking window length in
//! TBCLK count.The function takes a 16bit count value for the window length.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareWindowLength(uint32_t base, uint16_t windowLengthCount)
{
    //
    // Set the blanking window length in TBCLK counts
    //
    HW_WR_REG16(base + CSL_EPWM_DCFWINDOW, windowLengthCount);
}

//*****************************************************************************
//
//! Return DC filter blanking window offset count.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns DC filter blanking window offset count.
//!
//! \return None
//
//*****************************************************************************
static inline uint16_t
EPWM_getDigitalCompareBlankingWindowOffsetCount(uint32_t base)
{
    //
    // Return the Blanking Window Offset count
    //
    return(HW_RD_REG16(base + CSL_EPWM_DCFOFFSETCNT));
}

//*****************************************************************************
//
//! Return DC filter blanking window length count.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns DC filter blanking window length count.
//!
//! \return None
//
//*****************************************************************************
static inline uint16_t
EPWM_getDigitalCompareBlankingWindowLengthCount(uint32_t base)
{
    //
    // Return the Blanking Window Length count
    //
    return(HW_RD_REG16(base + CSL_EPWM_DCFWINDOWCNT));
}

//*****************************************************************************
//
//! Set up the Digital Compare Event source.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//! \param dcEvent is the Digital Compare Event number.
//! \param dcEventSource is the - Digital Compare Event source.
//!
//! This function sets up the Digital Compare module Event sources.
//! The following are valid values for the parameters.
//!  - dcModule
//!      - EPWM_DC_MODULE_A  - Digital Compare Module A
//!      - EPWM_DC_MODULE_B  - Digital Compare Module B
//!  - dcEvent
//!      - EPWM_DC_EVENT_1   - Digital Compare Event number 1
//!      - EPWM_DC_EVENT_2   - Digital Compare Event number 2
//!  - dcEventSource
//!      - EPWM_DC_EVENT_SOURCE_FILT_SIGNAL  - signal source is filtered
//!            \note The signal source for this option is DCxEVTy, where the
//!                  value of x is dependent on dcModule and the value of y is
//!                  dependent on dcEvent. Possible signal sources are DCAEVT1,
//!                  DCBEVT1, DCAEVT2 or DCBEVT2 depending on the value of both
//!                  dcModule and dcEvent.
//!      - EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL  - signal source is unfiltered
//!                   The signal source for this option is DCEVTFILT.
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareEventSource(uint32_t base,
                                  EPWM_DigitalCompareModule dcModule,
                                  EPWM_DigitalCompareEvent dcEvent,
                                  EPWM_DigitalCompareEventSource dcEventSource)
{
    uint32_t registerOffset;

    registerOffset = CSL_EPWM_DCACTL + (uint32_t)((uint16_t)dcModule * (uint32_t)EPWM_DCxCTL_STEP);

    //
    // Set the DC event 1 source source
    //
    if(dcEvent == EPWM_DC_EVENT_1)
    {
        HW_WR_REG16(base + registerOffset,
            ((HW_RD_REG16(base + registerOffset) &
            ~CSL_EPWM_DCACTL_EVT1SRCSEL_MASK) |
            (uint16_t)dcEventSource));
    }
    else
    {
        HW_WR_REG16(base + registerOffset,
            ((HW_RD_REG16(base + registerOffset) &
            ~(uint16_t)CSL_EPWM_DCACTL_EVT2SRCSEL_MASK) |
            (uint16_t)((uint16_t)dcEventSource << CSL_EPWM_DCACTL_EVT2SRCSEL_SHIFT)));
    }
}

//*****************************************************************************
//
//! Set up the Digital Compare input sync mode.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//! \param dcEvent is the Digital Compare Event number.
//! \param syncMode is the Digital Compare Event sync mode.
//!
//! This function sets up the Digital Compare module Event sources.
//! The following are valid values for the parameters.
//!  - dcModule
//!      - EPWM_DC_MODULE_A  - Digital Compare Module A
//!      - EPWM_DC_MODULE_B  - Digital Compare Module B
//!  - dcEvent
//!      - EPWM_DC_EVENT_1   - Digital Compare Event number 1
//!      - EPWM_DC_EVENT_2   - Digital Compare Event number 2
//!  - syncMode
//!      - EPWM_DC_EVENT_INPUT_SYNCED      - DC input signal is synced with
//!                                          TBCLK
//!      - EPWM_DC_EVENT_INPUT_NOT SYNCED  - DC input signal is not synced with
//!                                          TBCLK
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareEventSyncMode(uint32_t base,
                                    EPWM_DigitalCompareModule dcModule,
                                    EPWM_DigitalCompareEvent dcEvent,
                                    EPWM_DigitalCompareSyncMode syncMode)
{
    uint32_t registerOffset;

    registerOffset = CSL_EPWM_DCACTL + (uint32_t)((uint16_t)dcModule * (uint32_t)EPWM_DCxCTL_STEP);

    //
    // Set the DC event sync mode
    //
    if(dcEvent == EPWM_DC_EVENT_1)
    {
        HW_WR_REG16(base + registerOffset,
            ((HW_RD_REG16(base + registerOffset) &
            ~(uint16_t)CSL_EPWM_DCACTL_EVT1FRCSYNCSEL_MASK) |
            (uint16_t)((uint16_t)syncMode << CSL_EPWM_DCACTL_EVT1FRCSYNCSEL_SHIFT)));
    }
    else
    {
        HW_WR_REG16(base + registerOffset,
            ((HW_RD_REG16(base + registerOffset) &
            ~(uint16_t)CSL_EPWM_DCACTL_EVT2FRCSYNCSEL_MASK) |
            (uint16_t)((uint16_t)syncMode << CSL_EPWM_DCACTL_EVT2FRCSYNCSEL_SHIFT)));
    }
}

//*****************************************************************************
//
//! Enable Digital Compare to generate Start of Conversion.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//!
//! This function enables the Digital Compare Event 1 to generate Start of
//! Conversion.
//! The following are valid values for the \e dcModule parameter.
//!     - EPWM_DC_MODULE_A  - Digital Compare Module A
//!     - EPWM_DC_MODULE_B  - Digital Compare Module B
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareADCTrigger(uint32_t base,
                                    EPWM_DigitalCompareModule dcModule)
{
    uint32_t registerOffset;

    registerOffset = CSL_EPWM_DCACTL + (uint32_t)((uint16_t)dcModule * (uint32_t)EPWM_DCxCTL_STEP);

    //
    // Enable Digital Compare start of conversion generation
    //
    HW_WR_REG16(base + registerOffset,
        (HW_RD_REG16(base + registerOffset) | CSL_EPWM_DCACTL_EVT1SOCE_MASK));
}

//*****************************************************************************
//
//! Disable Digital Compare from generating Start of Conversion.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//!
//! This function disables the Digital Compare Event 1 from generating Start of
//! Conversion.
//! The following are valid values for the \e dcModule parameter.
//!     - EPWM_DC_MODULE_A  - Digital Compare Module A
//!     - EPWM_DC_MODULE_B  - Digital Compare Module B
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareADCTrigger(uint32_t base,
                                     EPWM_DigitalCompareModule dcModule)
{
    uint32_t registerOffset;

    registerOffset = CSL_EPWM_DCACTL + (uint32_t)((uint16_t)dcModule * (uint32_t)EPWM_DCxCTL_STEP);

    //
    // Disable Digital Compare start of conversion generation
    //
    HW_WR_REG16(base + registerOffset,
        (HW_RD_REG16(base + registerOffset) & ~CSL_EPWM_DCACTL_EVT1SOCE_MASK));
}

//*****************************************************************************
//
//! Enable Digital Compare to generate sync out pulse.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//!
//! This function enables the Digital Compare Event 1 to generate sync out
//! pulse
//! The following are valid values for the \e dcModule parameter.
//!     - EPWM_DC_MODULE_A  - Digital Compare Module A
//!     - EPWM_DC_MODULE_B  - Digital Compare Module B
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareSyncEvent(uint32_t base,
                                   EPWM_DigitalCompareModule dcModule)
{
    uint32_t registerOffset;

    registerOffset = CSL_EPWM_DCACTL + (uint32_t)((uint16_t)dcModule * (uint32_t)EPWM_DCxCTL_STEP);

    //
    // Enable Digital Compare sync out pulse generation
    //
    HW_WR_REG16(base + registerOffset,
        (HW_RD_REG16(base + registerOffset) | CSL_EPWM_DCACTL_EVT1SYNCE_MASK));
}

//*****************************************************************************
//
//! Disable Digital Compare from generating Start of Conversion.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//!
//! This function disables the Digital Compare Event 1 from generating synch
//! out pulse.
//! The following are valid values for the \e dcModule parameters.
//!     - EPWM_DC_MODULE_A  - Digital Compare Module A
//!     - EPWM_DC_MODULE_B  - Digital Compare Module B
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareSyncEvent(uint32_t base,
                                    EPWM_DigitalCompareModule dcModule)
{
    uint32_t registerOffset;

    registerOffset = CSL_EPWM_DCACTL + (uint32_t)((uint16_t)dcModule * (uint32_t)EPWM_DCxCTL_STEP);

    //
    // Disable Digital Compare sync out pulse generation
    //
    HW_WR_REG16(base + registerOffset,
        (HW_RD_REG16(base + registerOffset) &
        ~CSL_EPWM_DCACTL_EVT1SYNCE_MASK));
}

//*****************************************************************************
//
//! Set up the Digital Compare CBC latch mode.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//! \param dcEvent is the Digital Compare Event number.
//! \param latchMode is the Digital Compare CBC latch mode.
//!
//! This function sets up the Digital Compare CBC latch mode.
//! The following are valid values for the parameters.
//!  - dcModule
//!      - EPWM_DC_MODULE_A  - Digital Compare Module A
//!      - EPWM_DC_MODULE_B  - Digital Compare Module B
//!  - dcEvent
//!      - EPWM_DC_EVENT_1   - Digital Compare Event number 1
//!      - EPWM_DC_EVENT_2   - Digital Compare Event number 2
//!  - latchMode
//!      - EPWM_DC_CBC_LATCH_DISABLED -DC cycle-by-cycle(CBC) latch is disabled
//!      - EPWM_DC_CBC_LATCH_ENABLED  -DC cycle-by-cycle(CBC) latch is enabled
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareCBCLatchMode(uint32_t base,
                                   EPWM_DigitalCompareModule dcModule,
                                   EPWM_DigitalCompareEvent dcEvent,
                                   EPWM_DigitalCompareCBCLatchMode latchMode)
{
    uint32_t registerOffset;

    registerOffset = CSL_EPWM_DCACTL + (uint32_t)((uint16_t)dcModule * (uint32_t)EPWM_DCxCTL_STEP);

    //
    // Set the DC CBC Latch Mode
    //
    if(dcEvent == EPWM_DC_EVENT_1)
    {
        HW_WR_REG16(base + registerOffset,
            ((HW_RD_REG16(base + registerOffset) &
            ~(uint16_t)CSL_EPWM_DCACTL_EVT1LATSEL_MASK) |
           (uint16_t)((uint16_t)latchMode << CSL_EPWM_DCACTL_EVT1LATSEL_SHIFT)));
    }
    else
    {
        HW_WR_REG16(base + registerOffset,
            ((HW_RD_REG16(base + registerOffset) &
            ~(uint16_t)CSL_EPWM_DCACTL_EVT2LATSEL_MASK) |
            (uint16_t)((uint16_t)latchMode << CSL_EPWM_DCACTL_EVT2LATSEL_SHIFT)));
    }
}

//*****************************************************************************
//
//! Sets the Digital Compare CBC latch clear event.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//! \param dcEvent is the Digital Compare Event number.
//! \param clearEvent is the Digital Compare CBC latch clear event.
//!
//! This function sets the Digital Compare CBC latch clear event.
//! The following are valid values for the parameters.
//!  - dcModule
//!      - EPWM_DC_MODULE_A  - Digital Compare Module A
//!      - EPWM_DC_MODULE_B  - Digital Compare Module B
//!  - dcEvent
//!      - EPWM_DC_EVENT_1   - Digital Compare Event number 1
//!      - EPWM_DC_EVENT_2   - Digital Compare Event number 2
//!  - clearEvent
//!      - EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO - DC CBC latch is cleared when
//!                                          counter is zero
//!      - EPWM_DC_CBC_LATCH_CLR_ON_CNTR_PERIOD - DC CBC latch is cleared when
//!                                               counter is equal to period
//!      - EPWM_DC_CBC_LATCH_CLR_ON_CNTR_ZERO_PERIOD - DC CBC latch is cleared
//!                                                    when either counter is
//!                                                    zero or equal to period
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_selectDigitalCompareCBCLatchClearEvent(uint32_t base,
                              EPWM_DigitalCompareModule dcModule,
                              EPWM_DigitalCompareEvent dcEvent,
                              EPWM_DigitalCompareCBCLatchClearEvent clearEvent)
{
    uint32_t registerOffset;

    registerOffset = CSL_EPWM_DCACTL + (uint32_t)((uint16_t)dcModule * (uint32_t)EPWM_DCxCTL_STEP);

    //
    // Set the DC CBC Latch Clear Event
    //
    if(dcEvent == EPWM_DC_EVENT_1)
    {
        HW_WR_REG16(base + registerOffset,
            ((HW_RD_REG16(base + registerOffset) &
            ~(uint16_t)CSL_EPWM_DCACTL_EVT1LATCLRSEL_MASK) |
            (uint16_t)((uint16_t)clearEvent << CSL_EPWM_DCACTL_EVT1LATCLRSEL_SHIFT)));
    }
    else
    {
        HW_WR_REG16(base + registerOffset,
            ((HW_RD_REG16(base + registerOffset) &
            ~(uint16_t)CSL_EPWM_DCACTL_EVT2LATCLRSEL_MASK) |
            (uint16_t)((uint16_t)clearEvent << CSL_EPWM_DCACTL_EVT2LATCLRSEL_SHIFT)));
    }
}

//*****************************************************************************
//
//! Gets the Digital Compare CBC latch status
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//! \param dcEvent is the Digital Compare Event number.
//!
//! This function returns the Digital Compare module cycle-by-cycle(CBC) latch
//! status.
//! The following are valid values for the parameters.
//!  - dcModule
//!      - EPWM_DC_MODULE_A  - Digital Compare Module A
//!      - EPWM_DC_MODULE_B  - Digital Compare Module B
//!  - dcEvent
//!      - EPWM_DC_EVENT_1   - Digital Compare Event number 1
//!      - EPWM_DC_EVENT_2   - Digital Compare Event number 2
//!
//! \return Returns Digital Compare CBC latch status.
//! -\b true  - CBC latch is enabled
//! -\b false - CBC latch is disabled
//
//*****************************************************************************
static inline bool
EPWM_getDigitalCompareCBCLatchStatus(uint32_t base,
                                     EPWM_DigitalCompareModule dcModule,
                                     EPWM_DigitalCompareEvent dcEvent)
{
    uint32_t registerOffset;
    uint16_t status;

    registerOffset = CSL_EPWM_DCACTL + (uint32_t)((uint16_t)dcModule * (uint32_t)EPWM_DCxCTL_STEP);

    //
    // Get DC CBC Latch Clear Event
    //
    if(dcEvent == EPWM_DC_EVENT_1)
    {
        status = HW_RD_REG16(base + registerOffset) &
                 CSL_EPWM_DCACTL_EVT1LAT_MASK;
    }
    else
    {
        status = HW_RD_REG16(base + registerOffset) &
                 CSL_EPWM_DCACTL_EVT2LAT_MASK;
    }

    return(status != 0U);
}

//
// DC capture mode
//
//*****************************************************************************
//
//! Enables the Time Base Counter Capture controller.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the time Base Counter Capture.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareCounterCapture(uint32_t base)
{
    //
    // Enable Time base counter capture
    //
    HW_WR_REG16(base + CSL_EPWM_DCCAPCTL,
        (HW_RD_REG16(base + CSL_EPWM_DCCAPCTL) | CSL_EPWM_DCCAPCTL_CAPE_MASK));
}

//*****************************************************************************
//
//! Disables the Time Base Counter Capture controller.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disable the time Base Counter Capture.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareCounterCapture(uint32_t base)
{
    //
    // Disable Time base counter capture
    //
    HW_WR_REG16(base + CSL_EPWM_DCCAPCTL,
        (HW_RD_REG16(base + CSL_EPWM_DCCAPCTL) &
        ~CSL_EPWM_DCCAPCTL_CAPE_MASK));
}

//*****************************************************************************
//
//! Set the Time Base Counter Capture mode.
//!
//! \param base is the base address of the EPWM module.
//! \param enableShadowMode is the shadow read mode flag.
//!
//! This function sets the mode the Time Base Counter value is read from. If
//! enableShadowMode is true, CPU reads of the DCCAP register will return the
//! shadow register contents.If enableShadowMode is false, CPU reads of the
//! DCCAP register will return the active register contents.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareCounterShadowMode(uint32_t base, bool enableShadowMode)
{
    if(enableShadowMode)
    {
        //
        // Enable DC counter shadow mode
        //
        HW_WR_REG16(base + CSL_EPWM_DCCAPCTL,
            (HW_RD_REG16(base + CSL_EPWM_DCCAPCTL) &
            ~CSL_EPWM_DCCAPCTL_SHDWMODE_MASK));
    }
    else
    {
       //
       // Disable DC counter shadow mode
       //
       HW_WR_REG16(base + CSL_EPWM_DCCAPCTL,
           (HW_RD_REG16(base + CSL_EPWM_DCCAPCTL) |
           CSL_EPWM_DCCAPCTL_SHDWMODE_MASK));
    }
}

//*****************************************************************************
//
//! Return the DC Capture event status.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the DC capture event status.
//!
//! \return Returns true if a DC capture event has occurs.
//!         Returns false if no DC Capture event has occurred.
//!
//! \return None.
//
//*****************************************************************************
static inline bool
EPWM_getDigitalCompareCaptureStatus(uint32_t base)
{
    //
    // Return the DC compare status
    //
    return((HW_RD_REG16(base + CSL_EPWM_DCCAPCTL) &
            CSL_EPWM_DCCAPCTL_CAPSTS_MASK) == CSL_EPWM_DCCAPCTL_CAPSTS_MASK);
}

//*****************************************************************************
//
//! Return the DC Time Base Counter capture value.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the DC Time Base Counter capture value. The value
//! read is determined by the mode as set in the
//! EPWM_setTimeBaseCounterReadMode() function.
//!
//! \return Returns the DC Time Base Counter Capture count value.
//
//*****************************************************************************
static inline uint16_t
EPWM_getDigitalCompareCaptureCount(uint32_t base)
{
    //
    // Return the DC Time Base Counter Capture count value
    //
    return(HW_RD_REG16(base + CSL_EPWM_DCCAP));
}

//*****************************************************************************
//
//! Enable DC TRIP combinational input.
//!
//! \param base is the base address of the EPWM module.
//! \param tripInput is the Trip number.
//! \param dcType is the Digital Compare module.
//!
//! This function enables the specified Trip input.
//! Valid values for the parameters are:
//!  - tripInput
//!      - EPWM_DC_COMBINATIONAL_TRIPINx, where x is 1,2,...12,13,14,15
//!  - dcType
//!      - EPWM_DC_TYPE_DCAH  - Digital Compare A High
//!      - EPWM_DC_TYPE_DCAL  - Digital Compare A Low
//!      - EPWM_DC_TYPE_DCBH  - Digital Compare B High
//!      - EPWM_DC_TYPE_DCBL  - Digital Compare B Low
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareTripCombinationInput(uint32_t base,
                                              uint16_t tripInput,
                                              EPWM_DigitalCompareType dcType)
{
    uint32_t registerOffset;

    //
    // Get the DCAHTRIPSEL, DCALTRIPSEL, DCBHTRIPSEL, DCBLTRIPSEL register
    // offset with respect to DCAHTRIPSEL
    //
    registerOffset = CSL_EPWM_DCAHTRIPSEL +
                     (uint32_t)((uint16_t)dcType * (uint32_t)EPWM_DCxxTRIPSEL);

    //
    // Set the DC trip input
    //
    HW_WR_REG16(base + registerOffset,
        (HW_RD_REG16(base + registerOffset) | tripInput));

    //
    // Enable the combination input
    //
    HW_WR_REG16(base + CSL_EPWM_DCTRIPSEL,
        (HW_RD_REG16(base + CSL_EPWM_DCTRIPSEL) |
        ((uint16_t)CSL_EPWM_DCTRIPSEL_DCAHCOMPSEL_MASK << ((uint16_t)dcType << 2U))));
}

//*****************************************************************************
//
//! Disable DC TRIP combinational input.
//!
//! \param base is the base address of the EPWM module.
//! \param tripInput is the Trip number.
//! \param dcType is the Digital Compare module.
//!
//! This function disables the specified Trip input.
//! Valid values for the parameters are:
//!  - tripInput
//!      - EPWM_DC_COMBINATIONAL_TRIPINx, where x is 1,2,...12,13,14,15
//!  - dcType
//!      - EPWM_DC_TYPE_DCAH  - Digital Compare A High
//!      - EPWM_DC_TYPE_DCAL  - Digital Compare A Low
//!      - EPWM_DC_TYPE_DCBH  - Digital Compare B High
//!      - EPWM_DC_TYPE_DCBL  - Digital Compare B Low
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareTripCombinationInput(uint32_t base,
                                               uint16_t tripInput,
                                               EPWM_DigitalCompareType dcType)
{
    uint32_t registerOffset;

    //
    // Get the DCAHTRIPSEL, DCALTRIPSEL, DCBHTRIPSEL, DCBLTRIPSEL register
    // offset with respect to DCAHTRIPSEL
    //
    registerOffset = CSL_EPWM_DCAHTRIPSEL +
                     (uint32_t)((uint16_t)dcType * (uint32_t)EPWM_DCxxTRIPSEL);

    //
    // Set the DC trip input
    //
    HW_WR_REG16(base + registerOffset,
        (HW_RD_REG16(base + registerOffset) & ~tripInput));
}

//
// Event capture mode
//
//*****************************************************************************
//
//! Enables the Capture event .
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the CAPIN.sync on which the edge detection logic
//! is performed.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableCaptureInEvent(uint32_t base)
{

    //
    // Enables CAPIN.sync signal
    //
    HW_WR_REG16(
        base + CSL_EPWM_CAPCTL,
        (HW_RD_REG16(base + CSL_EPWM_CAPCTL) | CSL_EPWM_CAPCTL_SRCSEL_MASK)
    );
}

//*****************************************************************************
//
//! Disables the Capture event.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the CAPIN.sync on which the edge detection logic
//! is performed.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableCaptureInEvent(uint32_t base)
{
    //
    // Disables CAPIN.sync signal
    //
    HW_WR_REG16(
        base + CSL_EPWM_CAPCTL,
        (HW_RD_REG16(base + CSL_EPWM_CAPCTL) & (~CSL_EPWM_CAPCTL_SRCSEL_MASK))
    );
}

//*****************************************************************************
//
//! Polarity selection for capture gate input.
//!
//! \param base is the base address of the EPWM module.
//! \param polSel is the polarity to be selected for CAPGATE.
//!
//! This function selects the input polarity for capture gate.
//! Valid values for the \e polSel are:
//!      - EPWM_CAPGATE_INPUT_ALWAYS_ON  - always on
//!      - EPWM_CAPGATE_INPUT_ALWAYS_OFF  - always off
//!      - EPWM_CAPGATE_INPUT_SYNC  - CAPGATE.sync
//!      - EPWM_CAPGATE_INPUT_SYNC_INVERT  - CAPGATE.sync inverted
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_configCaptureGateInputPolarity(uint32_t base,
                                    uint8_t polSel)
{
    //
    // Configures polarity for CAPGATE
    //
    HW_WR_REG16(
        base + CSL_EPWM_CAPCTL,
             (HW_RD_REG16(base + CSL_EPWM_CAPCTL) & (~CSL_EPWM_CAPCTL_CAPGATEPOL_MASK)) |
              ((uint16_t)polSel << CSL_EPWM_CAPCTL_CAPGATEPOL_SHIFT));
}

//*****************************************************************************
//
//! Polarity selection for capture input.
//!
//! \param base is the base address of the EPWM module.
//! \param polSel is the polarity to be selected for CAPIN.
//!
//! This function selects the input polarity for capture.
//! Valid values for the \e polSel are:
//!      - EPWM_CAPTURE_INPUT_CAPIN_SYNC         - not inverted
//!      - EPWM_CAPTURE_INPUT_CAPIN_SYNC_INVERT  - inverted
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_invertCaptureInputPolarity(uint32_t base,
                                uint8_t polSel)
{
    //
    // Configures polarity for Capture Input
    //
    HW_WR_REG16(
        base + CSL_EPWM_CAPCTL,
             (HW_RD_REG16(base + CSL_EPWM_CAPCTL) & (~CSL_EPWM_CAPCTL_CAPINPOL_MASK)) |
              ((uint16_t)polSel << CSL_EPWM_CAPCTL_CAPINPOL_SHIFT));
}

//*****************************************************************************
//
//! Enables independent pulse selection for Blanking and Capture Logic.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables pulse selection determined by the CAPMIXSEL register.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableIndependentPulseLogic(uint32_t base)
{
    //
    // Configures polarity for Capture Input
    //
    HW_WR_REG16(
        base + CSL_EPWM_CAPCTL,
             (HW_RD_REG16(base + CSL_EPWM_CAPCTL) & (~CSL_EPWM_CAPCTL_PULSECTL_MASK)) |
              (((uint16_t)1U) << CSL_EPWM_CAPCTL_PULSECTL_SHIFT));
}

//*****************************************************************************
//
//! Disables independent pulse selection for Blanking and Capture Logic.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables pulse selection determined by the CAPMIXSEL register.
//! The pulse selection is determined by DCFCTL[PULSESEL] bits.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableIndependentPulseLogic(uint32_t base)
{
    //
    // Configures polarity for Capture Input
    //
    HW_WR_REG16(
        base + CSL_EPWM_CAPCTL,
             (HW_RD_REG16(base + CSL_EPWM_CAPCTL) | (CSL_EPWM_CAPCTL_PULSECTL_MASK)) &
              (~(((uint16_t)1U) << CSL_EPWM_CAPCTL_PULSECTL_SHIFT)));
}

//*****************************************************************************
//
//! Capture event force load.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function forces a load to occur on DCCAP.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_forceCaptureEventLoad(uint32_t base)
{
    //
    // Force a Capture Event Load
    //
    HW_WR_REG16(
        base + CSL_EPWM_CAPCTL,
             HW_RD_REG16(base + CSL_EPWM_CAPCTL) | (CSL_EPWM_CAPCTL_FRCLOAD_MASK));
}

//*****************************************************************************
//
//! Set the capture trip input.
//!
//! \param base is the base address of the EPWM module.
//! \param tripSource is the tripSource.
//! \param dcType is the Digital Compare type.
//!
//! This function sets the trip input to the Digital Compare (DC). For a given
//! dcType the function sets the tripSource to be the input to the DC.
//! Valid values for the parameter are:
//!  - tripSource
//!    - EPWM_DC_TRIP_TRIPINx - Trip x,where x ranges from 1 to 15 excluding 13
//!    - EPWM_DC_TRIP_COMBINATION - selects all the Trip signals whose input
//!                                 is enabled by the following function
//!                                 EPWM_enableCaptureTripCombinationInput()
//!  - dcType
//!      - EPWM_CAPTURE_GATE
//!      - EPWM_CAPTURE_INPUT
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_selectCaptureTripInput(uint32_t base,
                            EPWM_DigitalCompareTripInput tripSource,
                            uint8_t dcType)
{
    //
    // Set the Capture trip input
    //
    if(dcType == EPWM_CAPTURE_GATE)
    {
        HW_WR_REG16(
        base + CSL_EPWM_CAPTRIPSEL,
             (HW_RD_REG16(base + CSL_EPWM_CAPTRIPSEL) & (~CSL_EPWM_CAPTRIPSEL_CAPGATECOMPSEL_MASK)) |
              (((uint16_t)tripSource) << CSL_EPWM_CAPTRIPSEL_CAPGATECOMPSEL_SHIFT));
    }
    else
    {
        HW_WR_REG16(
        base + CSL_EPWM_CAPTRIPSEL,
             (HW_RD_REG16(base + CSL_EPWM_CAPTRIPSEL) & (~CSL_EPWM_CAPTRIPSEL_CAPINCOMPSEL_MASK)) |
              (((uint16_t)tripSource) << CSL_EPWM_CAPTRIPSEL_CAPINCOMPSEL_SHIFT));
    }
}


//*****************************************************************************
//
//! Enable Capture TRIP combinational input.
//!
//! \param base is the base address of the EPWM module.
//! \param tripInput is the Trip number.
//! \param dcType is the Digital Compare module.
//!
//! This function enables the specified Trip input.
//! Valid values for the parameters are:
//!  - tripInput
//!      - EPWM_DC_COMBINATIONAL_TRIPINx, where x is 1,2,...12,14,15
//!  - dcType
//!      - EPWM_CAPTURE_GATE
//!      - EPWM_CAPTURE_INPUT
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableCaptureTripCombinationInput(uint32_t base,
                                            uint16_t tripInput,
                                            uint8_t dcType)
{

    if(dcType == EPWM_CAPTURE_GATE)
    {
        //
        // Set the capture trip input
        //
        HW_WR_REG16(
        base + CSL_EPWM_CAPGATETRIPSEL, tripInput);

    }
    else
    {
        //
        // Set the capture trip input
        //
        HW_WR_REG16(
        base + CSL_EPWM_CAPINTRIPSEL, tripInput);
    }
    //
    // Enable the combination input
    //
    EPWM_DigitalCompareTripInput combinational_input = EPWM_DC_TRIP_COMBINATION;
    if(tripInput != 0U)
    {
        EPWM_selectCaptureTripInput(base, combinational_input, dcType);
    }
}

//*****************************************************************************
//
//! Disable Capture TRIP combinational input.
//!
//! \param base is the base address of the EPWM module.
//! \param tripInput is the Trip number.
//! \param dcType is the Digital Compare module.
//!
//! This function disables the specified Trip input.
//! Valid values for the parameters are:
//!  - tripInput
//!      - EPWM_DC_COMBINATIONAL_TRIPINx, where x is 1,2,...12,14,15
//!  - dcType
//!      - EPWM_CAPTURE_GATE
//!      - EPWM_CAPTURE_INPUT
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableCaptureTripCombinationInput(uint32_t base,
                                         uint16_t tripInput,
                                         uint8_t dcType)
{
    if(dcType == EPWM_CAPTURE_GATE)
    {
        //
        // Set the capture trip input
        //
        HW_WR_REG16(
        base + CSL_EPWM_CAPGATETRIPSEL,
        HW_RD_REG16(base + CSL_EPWM_CAPGATETRIPSEL) & (~tripInput));

    }
    else
    {
        //
        // Set the capture trip input
        //
        HW_WR_REG16(
        base + CSL_EPWM_CAPINTRIPSEL,
        HW_RD_REG16(base + CSL_EPWM_CAPGATETRIPSEL) & (~tripInput));
    }
}

//
// Valley switching
//
//*****************************************************************************
//
//! Enable valley capture mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables Valley Capture mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableValleyCapture(uint32_t base)
{
    //
    // Set VCAPE bit
    //
    HW_WR_REG16(base + CSL_EPWM_VCAPCTL,
        (HW_RD_REG16(base + CSL_EPWM_VCAPCTL) | CSL_EPWM_VCAPCTL_VCAPE_MASK));
}

//*****************************************************************************
//
//! Disable valley capture mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables Valley Capture mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableValleyCapture(uint32_t base)
{
    //
    // Clear VCAPE bit
    //
    HW_WR_REG16(base + CSL_EPWM_VCAPCTL,
        (HW_RD_REG16(base + CSL_EPWM_VCAPCTL) & ~CSL_EPWM_VCAPCTL_VCAPE_MASK));
}

//*****************************************************************************
//
//! Start valley capture mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function starts Valley Capture sequence.
//!
//! \b Make sure you invoke EPWM_setValleyTriggerSource with the trigger
//!    variable set to EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE before calling this
//!    function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_startValleyCapture(uint32_t base)
{
    //
    // Set VCAPSTART bit
    //
    HW_WR_REG16(base + CSL_EPWM_VCAPCTL,
        (HW_RD_REG16(base + CSL_EPWM_VCAPCTL) |
        CSL_EPWM_VCAPCTL_VCAPSTART_MASK));
}

//*****************************************************************************
//
//! Set valley capture trigger.
//!
//! \param base is the base address of the EPWM module.
//! \param trigger is the Valley counter trigger.
//!
//! This function sets the trigger value that initiates Valley Capture sequence
//!
//! \b Set the number of Trigger source events for starting and stopping the
//!    valley capture using EPWM_setValleyTriggerEdgeCounts().
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setValleyTriggerSource(uint32_t base, EPWM_ValleyTriggerSource trigger)
{
    //
    // Write to TRIGSEL bits
    //
    HW_WR_REG16(base + CSL_EPWM_VCAPCTL,
        ((HW_RD_REG16(base + CSL_EPWM_VCAPCTL) &
        ~(uint16_t)CSL_EPWM_VCAPCTL_TRIGSEL_MASK) |
        (uint16_t)((uint16_t)trigger << CSL_EPWM_VCAPCTL_TRIGSEL_SHIFT)));
}

//*****************************************************************************
//
//! Set valley capture trigger source count.
//!
//! \param base is the base address of the EPWM module.
//! \param startCount
//! \param stopCount
//!
//! This function sets the number of trigger events required to start and stop
//! the valley capture count.
//! Maximum values for both startCount and stopCount is 15 corresponding to the
//! 15th edge of the trigger event.
//!
//! \b Note:
//!    A startCount value of 0 prevents starting the valley counter.
//!    A stopCount value of 0 prevents the valley counter from stopping.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setValleyTriggerEdgeCounts(uint32_t base, uint16_t startCount,
                                uint16_t stopCount)
{
    //
    // Check the arguments
    //
    DebugP_assert((startCount < 16U) && (stopCount < 16U));

    //
    // Write to STARTEDGE and STOPEDGE bits
    //
    HW_WR_REG16(base + CSL_EPWM_VCNTCFG,
        ((HW_RD_REG16(base + CSL_EPWM_VCNTCFG) &
        ~(CSL_EPWM_VCNTCFG_STARTEDGE_MASK | CSL_EPWM_VCNTCFG_STOPEDGE_MASK)) |
        ((uint32_t)startCount | ((uint32_t)stopCount << CSL_EPWM_VCNTCFG_STOPEDGE_SHIFT))));
}

//*****************************************************************************
//
//! Enable valley switching delay.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables Valley switching delay.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableValleyHWDelay(uint32_t base)
{
    //
    // Set EDGEFILTDLYSEL bit
    //
    HW_WR_REG16(base + CSL_EPWM_VCAPCTL,
        (HW_RD_REG16(base + CSL_EPWM_VCAPCTL) |
        CSL_EPWM_VCAPCTL_EDGEFILTDLYSEL_MASK));
}

//*****************************************************************************
//
//! Disable valley switching delay.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables Valley switching delay.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableValleyHWDelay(uint32_t base)
{
    //
    // Clear EDGEFILTDLYSEL bit
    //
    HW_WR_REG16(base + CSL_EPWM_VCAPCTL,
        (HW_RD_REG16(base + CSL_EPWM_VCAPCTL) &
        ~CSL_EPWM_VCAPCTL_EDGEFILTDLYSEL_MASK));
}

//*****************************************************************************
//
//! Set Valley delay values.
//!
//! \param base is the base address of the EPWM module.
//! \param delayOffsetValue is the software defined delay offset value.
//!
//! This function sets the Valley delay value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setValleySWDelayValue(uint32_t base, uint16_t delayOffsetValue)
{
    //
    // Write to SWVDELVAL bits
    //
    HW_WR_REG16(base + CSL_EPWM_SWVDELVAL, delayOffsetValue);
}

//*****************************************************************************
//
//! Set Valley delay mode.
//!
//! \param base is the base address of the EPWM module.
//! \param delayMode is the Valley delay mode.
//!
//! This function sets the Valley delay mode values.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setValleyDelayDivider(uint32_t base, EPWM_ValleyDelayMode delayMode)
{
    //
    // Write to VDELAYDIV bits
    //
    HW_WR_REG16(base + CSL_EPWM_VCAPCTL,
        ((HW_RD_REG16(base + CSL_EPWM_VCAPCTL) &
        ~(uint16_t)CSL_EPWM_VCAPCTL_VDELAYDIV_MASK) |
        (uint16_t)((uint16_t)delayMode << CSL_EPWM_VCAPCTL_VDELAYDIV_SHIFT)));
}

//*****************************************************************************
//
//! Get the valley edge status bit.
//!
//! \param base is the base address of the EPWM module.
//! \param edge is the start or stop edge.
//!
//! This function returns the status of the start or stop valley status
//! depending on the value of edge.
//! If a start or stop edge has occurred, the function returns true, if not it
//! returns false.
//!
//! \return Returns true if the specified edge has occurred,
//!         Returns false if the specified edge has not occurred.
//
//*****************************************************************************
static inline bool
EPWM_getValleyEdgeStatus(uint32_t base, EPWM_ValleyCounterEdge edge)
{
    bool status = true;
    if(edge == EPWM_VALLEY_COUNT_START_EDGE)
    {
        if((HW_RD_REG16(base + CSL_EPWM_VCNTCFG) & CSL_EPWM_VCNTCFG_STARTEDGESTS_MASK)
         == CSL_EPWM_VCNTCFG_STARTEDGESTS_MASK)
            {
                status = true;
            }
        else
            {
                status = false;
            }
    }
    else
    {
        if((HW_RD_REG16(base + CSL_EPWM_VCNTCFG) &
                CSL_EPWM_VCNTCFG_STOPEDGESTS_MASK) ==
                CSL_EPWM_VCNTCFG_STOPEDGESTS_MASK)
            {
                status = true;
            }
        else
            {
                status = false;
            }
    }

    return status;
}
//*****************************************************************************
//
//! Get the Valley Counter value.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the valley time base count value which is captured
//! upon occurrence of the stop edge condition selected by
//! EPWM_setValleyTriggerSource() and by the stopCount variable of the
//! EPWM_setValleyTriggerEdgeCounts() function.
//!
//! \return Returns the valley base time count.
//
//*****************************************************************************
static inline uint16_t
EPWM_getValleyCount(uint32_t base)
{
    //
    // Read VCNTVAL register
    //
    return(HW_RD_REG16(base + CSL_EPWM_VCNTVAL));
}

//*****************************************************************************
//
//! Get the Valley delay value.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the hardware valley delay count.
//!
//! \return Returns the valley delay count.
//
//*****************************************************************************
static inline uint16_t
EPWM_getValleyHWDelay(uint32_t base)
{
    //
    // Read HWVDELVAL  register
    //
    return(HW_RD_REG16(base + CSL_EPWM_HWVDELVAL));
}

//*****************************************************************************
//
//! Enable Global shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables Global shadow to active load mode of registers.
//! The trigger source for loading shadow to active is determined by
//! EPWM_setGlobalLoadTrigger() function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableGlobalLoad(uint32_t base)
{
    //
    // Shadow to active load is controlled globally
    //
    HW_WR_REG16(base + CSL_EPWM_GLDCTL,
        (HW_RD_REG16(base + CSL_EPWM_GLDCTL) | CSL_EPWM_GLDCTL_GLD_MASK));
}

//*****************************************************************************
//
//! Disable Global shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables Global shadow to active load mode of registers.
//! Loading shadow to active is determined individually.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableGlobalLoad(uint32_t base)
{
    //
    // Shadow to active load is controlled individually
    //
    HW_WR_REG16(base + CSL_EPWM_GLDCTL,
        (HW_RD_REG16(base + CSL_EPWM_GLDCTL) & ~CSL_EPWM_GLDCTL_GLD_MASK));
}

//*****************************************************************************
//
//! Set the Global shadow load pulse.
//!
//! \param base is the base address of the EPWM module.
//! \param loadTrigger is the pulse that causes global shadow load.
//!
//! This function sets the pulse that causes Global shadow to active load.
//! Valid values for the loadTrigger parameter are:
//!
//!   - EPWM_GL_LOAD_PULSE_CNTR_ZERO              - load when counter is equal
//!                                                 to zero
//!   - EPWM_GL_LOAD_PULSE_CNTR_PERIOD            - load when counter is equal
//!                                                 to period
//!   - EPWM_GL_LOAD_PULSE_CNTR_ZERO_PERIOD       - load when counter is equal
//!                                                 to zero or period
//!   - EPWM_GL_LOAD_PULSE_SYNC                    - load on sync event
//!   - EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_ZERO      - load on sync event or when
//!                                                 counter is equal to zero
//!   - EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_PERIOD    - load on sync event or when
//!                                                 counter is equal to period
//!   - EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD  - load on sync event or when
//!                                                 counter is equal to period
//!                                                 or zero
//!   - EPWM_GL_LOAD_PULSE_GLOBAL_FORCE            - load on global force
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setGlobalLoadTrigger(uint32_t base, EPWM_GlobalLoadTrigger loadTrigger)
{
    //
    // Set the Global shadow to active load pulse
    //
    HW_WR_REG16(base + CSL_EPWM_GLDCTL,
        ((HW_RD_REG16(base + CSL_EPWM_GLDCTL) &
        ~(uint16_t)CSL_EPWM_GLDCTL_GLDMODE_MASK) |
        (uint16_t)((uint16_t)loadTrigger << CSL_EPWM_GLDCTL_GLDMODE_SHIFT)));
}

//*****************************************************************************
//
//! Set the number of Global load pulse event counts
//!
//! \param base is the base address of the EPWM module.
//! \param prescalePulseCount is the pulse event counts.
//!
//! This function sets the number of Global Load pulse events that have to
//! occurred before a global load pulse is issued. Valid values for
//! prescaleCount range from 0 to 7. 0 being no event (disables counter), and 7
//! representing 7 events.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setGlobalLoadEventPrescale(uint32_t base, uint16_t prescalePulseCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(prescalePulseCount < 8U);

    //
    // Set the number of counts that have to occur before
    // a load strobe is issued
    //
    HW_WR_REG16(base + CSL_EPWM_GLDCTL,
        ((HW_RD_REG16(base + CSL_EPWM_GLDCTL) & ~CSL_EPWM_GLDCTL_GLDPRD_MASK) |
        ((uint32_t)prescalePulseCount << CSL_EPWM_GLDCTL_GLDPRD_SHIFT)));
}

//*****************************************************************************
//
//! Return the number of Global load pulse event counts
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the number of Global Load pulse events that have
//! occurred. These pulse events are set by the EPWM_setGlobalLoadTrigger()
//! function.
//!
//! \return None.
//
//*****************************************************************************
static inline uint16_t
EPWM_getGlobalLoadEventCount(uint32_t base)
{
    //
    // Return the number of events that have occurred
    //
    return((HW_RD_REG16(base + CSL_EPWM_GLDCTL) >>
            CSL_EPWM_GLDCTL_GLDCNT_SHIFT) & CSL_EPWM_GLDCTL_GLDCNT_MAX);
}

//*****************************************************************************
//
//! Enable continuous global shadow to active load.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables global continuous shadow to active load. Register
//! load happens every time the event set by the
//! EPWM_setGlobalLoadTrigger() occurs.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableGlobalLoadOneShotMode(uint32_t base)
{
    //
    // Enable global continuous shadow to active load
    //
    HW_WR_REG16(base + CSL_EPWM_GLDCTL,
        (HW_RD_REG16(base + CSL_EPWM_GLDCTL) &
        ~CSL_EPWM_GLDCTL_OSHTMODE_MASK));
}

//*****************************************************************************
//
//! Enable One shot global shadow to active load.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables a one time global shadow to active load. Register
//! load happens every time the event set by the
//! EPWM_setGlobalLoadTrigger() occurs.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableGlobalLoadOneShotMode(uint32_t base)
{
    //
    // Enable global continuous shadow to active load
    //
    HW_WR_REG16(base + CSL_EPWM_GLDCTL,
        (HW_RD_REG16(base + CSL_EPWM_GLDCTL) | CSL_EPWM_GLDCTL_OSHTMODE_MASK));
}

//*****************************************************************************
//
//! Set One shot global shadow to active load pulse.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function sets a one time global shadow to active load pulse. The pulse
//! propagates to generate a load signal if any of the events set by
//! EPWM_setGlobalLoadTrigger() occur.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setGlobalLoadOneShotLatch(uint32_t base)
{
    //
    // Set a one shot Global shadow load pulse.
    //
    HW_WR_REG16(base + CSL_EPWM_GLDCTL2,
        (HW_RD_REG16(base + CSL_EPWM_GLDCTL2) | CSL_EPWM_GLDCTL2_OSHTLD_MASK));
}

//*****************************************************************************
//
//! Force a software One shot global shadow to active load pulse.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function forces a software a one time global shadow to active load
//! pulse.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_forceGlobalLoadOneShotEvent(uint32_t base)
{
    //
    // Force a Software Global shadow load pulse
    //
    HW_WR_REG16(base + CSL_EPWM_GLDCTL2,
        (HW_RD_REG16(base + CSL_EPWM_GLDCTL2) | CSL_EPWM_GLDCTL2_GFRCLD_MASK));
}

//*****************************************************************************
//
//! Enable a register to be loaded Globally.
//!
//! \param base is the base address of the EPWM module.
//! \param loadRegister is the register.
//!
//! This function enables the register specified by loadRegister to be globally
//! loaded.
//! Valid values for loadRegister are:
//!  - EPWM_GL_REGISTER_TBPRD_TBPRDHR  - Register TBPRD:TBPRDHR
//!  - EPWM_GL_REGISTER_CMPA_CMPAHR    - Register CMPA:CMPAHR
//!  - EPWM_GL_REGISTER_CMPB_CMPBHR    - Register CMPB:CMPBHR
//!  - EPWM_GL_REGISTER_CMPC           - Register CMPC
//!  - EPWM_GL_REGISTER_CMPD           - Register CMPD
//!  - EPWM_GL_REGISTER_DBRED_DBREDHR  - Register DBRED:DBREDHR
//!  - EPWM_GL_REGISTER_DBFED_DBFEDHR  - Register DBFED:DBFEDHR
//!  - EPWM_GL_REGISTER_DBCTL          - Register DBCTL
//!  - EPWM_GL_REGISTER_AQCTLA_AQCTLA2 - Register AQCTLA/A2
//!  - EPWM_GL_REGISTER_AQCTLB_AQCTLB2 - Register AQCTLB/B2
//!  - EPWM_GL_REGISTER_AQCSFRC        - Register AQCSFRC
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableGlobalLoadRegisters(uint32_t base, uint16_t loadRegister)
{
    //
    // Check the arguments
    //
    DebugP_assert((loadRegister > 0x0000U) && (loadRegister < 0x0800U));

    //
    // The register specified by loadRegister is loaded globally
    //
    HW_WR_REG16(base + CSL_EPWM_GLDCFG,
        (HW_RD_REG16(base + CSL_EPWM_GLDCFG) | loadRegister));
}

//*****************************************************************************
//
//! Disable a register to be loaded Globally.
//!
//! \param base is the base address of the EPWM module.
//! \param loadRegister is the register.
//!
//! This function disables the register specified by loadRegister from being
//! loaded globally. The shadow to active load happens as specified by the
//! register control
//! Valid values for loadRegister are:
//!  - EPWM_GL_REGISTER_TBPRD_TBPRDHR  - Register TBPRD : TBPRDHR
//!  - EPWM_GL_REGISTER_CMPA_CMPAHR    - Register CMPA : CMPAHR
//!  - EPWM_GL_REGISTER_CMPB_CMPBHR    - Register CMPB : CMPBHR
//!  - EPWM_GL_REGISTER_CMPC           - Register CMPC
//!  - EPWM_GL_REGISTER_CMPD           - Register CMPD
//!  - EPWM_GL_REGISTER_DBRED_DBREDHR  - Register DBRED : DBREDHR
//!  - EPWM_GL_REGISTER_DBFED_DBFEDHR  - Register DBFED : DBFEDHR
//!  - EPWM_GL_REGISTER_DBCTL          - Register DBCTL
//!  - EPWM_GL_REGISTER_AQCTLA_AQCTLA2 - Register AQCTLA/A2
//!  - EPWM_GL_REGISTER_AQCTLB_AQCTLB2 - Register AQCTLB/B2
//!  - EPWM_GL_REGISTER_AQCSFRC        - Register AQCSFRC
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableGlobalLoadRegisters(uint32_t base, uint16_t loadRegister)
{
    //
    // Check the arguments
    //
    DebugP_assert((loadRegister > 0x0000U) && (loadRegister < 0x0800U));

    //
    // The register specified by loadRegister is loaded by individual
    // register configuration setting
    //
    HW_WR_REG16(base + CSL_EPWM_GLDCFG,
        (HW_RD_REG16(base + CSL_EPWM_GLDCFG) & ~loadRegister));
}

//*****************************************************************************
//
//! Locks protected register groups
//!
//! \param base is the base address of the EPWM module.
//! \param registerGroup is the register group to be locked.
//!
//! This function provides lock protection to register groups specified by
//! the registerGroup variable.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_lockRegisters(uint32_t base, EPWM_LockRegisterGroup registerGroup)
{
    //
    // Write the Key to EPWMLOCK register
    //
    HW_WR_REG32(base + CSL_EPWM_EPWMLOCK,
        ((uint32_t)EPWM_LOCK_KEY | ((uint32_t)registerGroup)));
}

//
// Minimum Dead Band
//
//*****************************************************************************
//
//! Enable Minimum DeadBand module
//!
//! \param base is the base address of the EPWM module.
//! \param block is the Minimum DeadBand Block to be enabled.
//!
//! This function enables the Minimum DeadBand module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableMinimumDeadBand(uint32_t base, uint32_t block)
{
    if(block == EPWM_MINDB_BLOCK_A)
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBCFG,
            (HW_RD_REG32(base + CSL_EPWM_MINDBCFG) |
            CSL_EPWM_MINDBCFG_ENABLEA_MASK));
    }
    else
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBCFG,
            (HW_RD_REG32(base + CSL_EPWM_MINDBCFG) |
            CSL_EPWM_MINDBCFG_ENABLEB_MASK));
    }
}

//*****************************************************************************
//
//! Disable Minimum DeadBand module
//!
//! \param base is the base address of the EPWM module.
//! \param block is the Minimum DeadBand Block to be enabled.
//!
//! This function disables the Minimum DeadBand module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableMinimumDeadBand(uint32_t base, uint32_t block)
{
    if(block == EPWM_MINDB_BLOCK_A)
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBCFG,
            (HW_RD_REG32(base + CSL_EPWM_MINDBCFG) &
            ~CSL_EPWM_MINDBCFG_ENABLEA_MASK));
    }
    else
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBCFG,
            (HW_RD_REG32(base + CSL_EPWM_MINDBCFG) &
            ~CSL_EPWM_MINDBCFG_ENABLEB_MASK));
    }
}

//*****************************************************************************
//
//! Invert the Minimum DeadBand Reference Signal
//!
//! \param base is the base address of the EPWM module.
//! \param block is the Minimum DeadBand block to be configured.
//! \param invert specifies whether reference signal is to be inverted or not.
//!
//! This function configures the Minimum DeadBand module to invert reference
//! signal which is used in the Minimum DeadBand module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_invertMinimumDeadBandSignal(uint32_t base, uint32_t block,
                                 uint32_t invert)
{
    if(block == EPWM_MINDB_BLOCK_A)
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBCFG,
            ((HW_RD_REG32(base + CSL_EPWM_MINDBCFG) &
            ~CSL_EPWM_MINDBCFG_INVERTA_MASK) |
            (invert<<CSL_EPWM_MINDBCFG_INVERTA_SHIFT)));
    }
    else
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBCFG,
            ((HW_RD_REG32(base + CSL_EPWM_MINDBCFG) &
            ~CSL_EPWM_MINDBCFG_INVERTB_MASK) |
            (invert<<CSL_EPWM_MINDBCFG_INVERTB_SHIFT)));
    }
}

//*****************************************************************************
//
//! Select signal for AND OR logic of Minimum DeadBand Module
//!
//! \param base is the base address of the EPWM module.
//! \param block is the Minimum DeadBand block to be configured.
//! \param referenceSignal specifies how the signal will be combined.
//!
//! This function configures how the signal will be combined with the
//! PWM output. The signal can be inverted and ANDed with PWM output or it can
//! be ORed with the PWM output.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_selectMinimumDeadBandAndOrLogic(uint32_t base, uint32_t block,
                                     uint32_t referenceSignal)
{
    if(block == EPWM_MINDB_BLOCK_A)
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBCFG,
            ((HW_RD_REG32(base + CSL_EPWM_MINDBCFG) &
            ~CSL_EPWM_MINDBCFG_POLSELA_MASK) |
            (referenceSignal<<CSL_EPWM_MINDBCFG_POLSELA_SHIFT)));
    }
    else
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBCFG,
            ((HW_RD_REG32(base + CSL_EPWM_MINDBCFG) &
            ~CSL_EPWM_MINDBCFG_POLSELB_MASK) |
            (referenceSignal<<CSL_EPWM_MINDBCFG_POLSELB_SHIFT)));
    }
}

//*****************************************************************************
//
//! Select Minimum DeadBand Blocking Signal
//!
//! \param base is the base address of the EPWM module.
//! \param block is the Minimum DeadBand block to be configured.
//! \param blockingSignal is the blocking signal to be used for a block.
//!
//! This function configures the blocking signal for Minimum DeadBand module.
//! Either of the Block A or Block B signal can be selected as blocking signal.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_selectMinimumDeadBandBlockingSignal(uint32_t base, uint32_t block,
                                         uint32_t blockingSignal)
{
    if(block == EPWM_MINDB_BLOCK_A)
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBCFG,
            ((HW_RD_REG32(base + CSL_EPWM_MINDBCFG) &
            ~CSL_EPWM_MINDBCFG_SELBLOCKA_MASK) |
            (blockingSignal<<CSL_EPWM_MINDBCFG_SELBLOCKA_SHIFT)));
    }
    else
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBCFG,
            ((HW_RD_REG32(base + CSL_EPWM_MINDBCFG) &
            ~CSL_EPWM_MINDBCFG_SELBLOCKB_MASK) |
            (blockingSignal<<CSL_EPWM_MINDBCFG_SELBLOCKB_SHIFT)));
    }
}

//*****************************************************************************
//
//! Select Minimum DeadBand Reference Signal
//!
//! \param base is the base address of the EPWM module.
//! \param block is the Minimum DeadBand block to be configured.
//! \param referenceSignal specifies the reference signal to be selected.
//!
//! This function selects the reference signal for Minimum DeadBand module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_selectMinimumDeadBandReferenceSignal(uint32_t base, uint32_t block,
                                          uint32_t referenceSignal)
{
    if(block == EPWM_MINDB_BLOCK_A)
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBCFG,
            ((HW_RD_REG32(base + CSL_EPWM_MINDBCFG) &
            ~CSL_EPWM_MINDBCFG_SELA_MASK) |
            (referenceSignal<<CSL_EPWM_MINDBCFG_SELA_SHIFT)));
    }
    else
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBCFG,
            ((HW_RD_REG32(base + CSL_EPWM_MINDBCFG) &
            ~CSL_EPWM_MINDBCFG_SELB_MASK) |
            (referenceSignal<<CSL_EPWM_MINDBCFG_SELB_SHIFT)));
    }
}

//*****************************************************************************
//
//! Get Minimum DeadBand Delay Value
//!
//! \param base is the base address of the EPWM module.
//! \param block is the Minimum DeadBand block whose value is needed.
//!
//! This function returns the delay value for the Minimum DeadBand module.
//!
//! \return Delay value of the Minimum DeadBand module.
//
//*****************************************************************************
static inline uint32_t
EPWM_getMinDeadBandDelay(uint32_t base, uint32_t block)
{
    uint32_t retval;

    if(block == EPWM_MINDB_BLOCK_A)
    {
        retval = (HW_RD_REG32(base + CSL_EPWM_MINDBDLY) &
                  CSL_EPWM_MINDBDLY_DELAYA_MASK);
    }
    else
    {
        retval = (HW_RD_REG32(base + CSL_EPWM_MINDBDLY) &
                  CSL_EPWM_MINDBDLY_DELAYB_MASK);
    }

    return retval;
}

//*****************************************************************************
//
//! Set the Minimum DeadBand Delay Value
//!
//! \param base is the base address of the EPWM module.
//! \param block is the Minimum DeadBand block to be configured.
//! \param delay is the delay value to be programmed for the Minimum DeadBand.
//!
//! This function configures the Minimum DeadBand delay value for the
//! specified block.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setMinDeadBandDelay(uint32_t base, uint32_t block, uint32_t delay)
{
    if(block == EPWM_MINDB_BLOCK_A)
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBDLY,
            ((HW_RD_REG32(base + CSL_EPWM_MINDBDLY) &
            ~CSL_EPWM_MINDBDLY_DELAYA_MASK) |
            (delay<<CSL_EPWM_MINDBDLY_DELAYA_SHIFT)));
    }
    else
    {
        HW_WR_REG32(base + CSL_EPWM_MINDBDLY,
            ((HW_RD_REG32(base + CSL_EPWM_MINDBDLY) &
            ~CSL_EPWM_MINDBDLY_DELAYB_MASK) |
            (delay<<CSL_EPWM_MINDBDLY_DELAYB_SHIFT)));
    }
}

//
// Illegal Combo Logic
//
//*****************************************************************************
//
//! Enable Illegal Combo Logic
//!
//! \param base is the base address of the EPWM module.
//! \param block is the Illegal Combo Logic block to be enabled.
//!
//! This function enables the Illegal Combo Logic block.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableIllegalComboLogic(uint32_t base, uint32_t block)
{
    if(block == EPWM_MINDB_BLOCK_A)
    {
        HW_WR_REG32(base + CSL_EPWM_LUTCTLA,
            (HW_RD_REG32(base + CSL_EPWM_LUTCTLA) &
            ~CSL_EPWM_LUTCTLA_BYPASS_MASK));
    }
    else
    {
        HW_WR_REG32(base + CSL_EPWM_LUTCTLB,
            (HW_RD_REG32(base + CSL_EPWM_LUTCTLB) &
            ~CSL_EPWM_LUTCTLB_BYPASS_MASK));
    }
}

//*****************************************************************************
//
//! Disable Illegal Combo Logic
//!
//! \param base is the base address of the EPWM module.
//! \param block is the Illegal Combo Logic block to be disabled.
//!
//! This function disables the Illegal Combo Logic block.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableIllegalComboLogic(uint32_t base, uint32_t block)
{
    if(block == EPWM_MINDB_BLOCK_A)
    {
        HW_WR_REG32(base + CSL_EPWM_LUTCTLA,
            (HW_RD_REG32(base + CSL_EPWM_LUTCTLA) |
            CSL_EPWM_LUTCTLA_BYPASS_MASK));
    }
    else
    {
        HW_WR_REG32(base + CSL_EPWM_LUTCTLB,
            (HW_RD_REG32(base + CSL_EPWM_LUTCTLB) |
            CSL_EPWM_LUTCTLB_BYPASS_MASK));
    }
}

//*****************************************************************************
//
//! Select XBar input for Illegal Combo Logic
//!
//! \param base is the base address of the EPWM module.
//! \param block is the Illegal Combo Logic block to be configured.
//! \param xbarInput is the Xbar signal to feed into the Illegal Combo Logic.
//!
//! This function selects which Xbar signal feeds into the Illegal Combo Logic.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_selectXbarInput(uint32_t base, uint32_t block, uint32_t xbarInput)
{
    if(block == EPWM_MINDB_BLOCK_A)
    {
        HW_WR_REG32(base + CSL_EPWM_LUTCTLA,
            ((HW_RD_REG32(base + CSL_EPWM_LUTCTLA) &
            ~CSL_EPWM_LUTCTLA_SELXBAR_MASK) |
            (xbarInput<<CSL_EPWM_LUTCTLA_SELXBAR_SHIFT)));
    }
    else
    {
        HW_WR_REG32(base + CSL_EPWM_LUTCTLB,
            ((HW_RD_REG32(base + CSL_EPWM_LUTCTLB) &
            ~CSL_EPWM_LUTCTLB_SELXBAR_MASK) |
            (xbarInput<<CSL_EPWM_LUTCTLB_SELXBAR_SHIFT)));
    }
}

//*****************************************************************************
//
//! Force Decx values
//!
//! \param base is the base address of the EPWM module.
//! \param block is the Illegal Combo Logic block to be configured.
//! \param decx is the register to be programmed.
//! \param force is the value to be programmed in the register.
//!
//! This function programs value in the Decx register fields of the Illegal
//! Combo Logic module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setLutDecX(uint32_t base, uint32_t block, uint32_t decx, uint32_t force)
{
    if(block == EPWM_MINDB_BLOCK_A)
    {
        HW_WR_REG32(base + CSL_EPWM_LUTCTLA,
            ((HW_RD_REG32(base + CSL_EPWM_LUTCTLA) &
            ~(CSL_EPWM_LUTCTLA_LUTDEC0_MAX <<
            (CSL_EPWM_LUTCTLA_LUTDEC0_SHIFT+decx))) |
            (force<<(CSL_EPWM_LUTCTLA_LUTDEC0_SHIFT+decx))));
    }
    else if(block == EPWM_MINDB_BLOCK_B)
    {
        HW_WR_REG32(base + CSL_EPWM_LUTCTLB,
            ((HW_RD_REG32(base + CSL_EPWM_LUTCTLB) &
            ~(CSL_EPWM_LUTCTLB_LUTDEC0_MAX <<
            (CSL_EPWM_LUTCTLB_LUTDEC0_SHIFT+decx))) |
            (force<<(CSL_EPWM_LUTCTLB_LUTDEC0_SHIFT+decx))));
    }
    else
    {
        /* do nothing */
    }
}

//*****************************************************************************
//
//! Sets the consolidated phase shift value in high resolution mode.
//!
//! \param base is the base address of the EPWM module.
//! \param phaseCount is the consolidated phase shift count value.
//!
//! This function sets the consolidated phase shift value, that is, both TBPHS
//! and TBPHSHR values are configured together.
//!
//! Call EPWM_enablePhaseShiftLoad & HRPWM_enableHRPhaseShiftLoad() functions
//! to enable loading of the phaseCount in high resolution mode.
//!
//! \b Note: phaseCount is a 24-bit value.
//! \b Note: For configuring TBPHS = 0x3C, TBPHSHR = 0x2;
//!          phaseCount = 0x3C02
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setPhaseShift(uint32_t base, uint32_t phaseCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(phaseCount <= 0xFFFFFFFFU);

    //
    // Write to TBPHS:TBPHSHR bits
    //
    HW_WR_REG32(base + CSL_EPWM_TBPHS, phaseCount<<8U);
}

//*****************************************************************************
//
//! Sets only the high resolution phase shift value.
//!
//! \param base is the base address of the EPWM module.
//! \param hrPhaseCount is the high resolution phase shift count value.
//!
//! This function sets only the high resolution phase shift(TBPHSHR) value.
//! Call the HRPWM_enableHRPhaseShiftLoad() function to enable loading of
//! the hrPhaseCount.
//!
//! \b Note: hrPhaseCount is an 8-bit value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setHiResPhaseShift(uint32_t base, uint16_t hrPhaseCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(hrPhaseCount <= CSL_EPWM_TBPHS_TBPHSHR_MAX);

    //
    // Write to TBPHSHR bits
    //
    HW_WR_REG32(base + CSL_EPWM_TBPHS,
        ((HW_RD_REG32(base + CSL_EPWM_TBPHS) &
        ~((uint32_t)CSL_EPWM_TBPHS_TBPHSHR_MASK)) |
        ((uint32_t)hrPhaseCount << (CSL_EPWM_TBPHS_TBPHSHR_SHIFT + 8U))));
}

//*****************************************************************************
//
//! Sets high resolution time base counter.
//!
//! \param base is the base address of the EPWM module.
//! \param hrPeriodCount is the high resolution period count value.
//!
//! This function sets only the high resolution time base counter(TBPRDHR)
//! value.
//!
//! User should map the desired period or frequency of the waveform into
//! the correct hrPeriodCount.
//!
//! \b Note: hrPeriodCount is an 16-bit value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setHiResTimeBasePeriod(uint32_t base, uint16_t hrPeriodCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(hrPeriodCount <= CSL_EPWM_TBPRDHR_TBPRDHR_MAX);

    //
    // Write to TBPRDHR bits
    //
    HW_WR_REG16(base + CSL_EPWM_TBPRDHR, hrPeriodCount << 8);
}

//*****************************************************************************
//
//! Gets the high resolution time base period count.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function gets the high resolution time base period(TBPRDHR) value.
//!
//! \return The high resolution time base period count value.
//
//*****************************************************************************
static inline uint16_t
HRPWM_getHiResTimeBasePeriod(uint32_t base)
{
    //
    // Read from TBPRDHR bit
    //
    return(HW_RD_REG16(base + CSL_EPWM_TBPRDHR) >> 8U);
}

//*****************************************************************************
//
//! Sets the high resolution edge controlled by MEP (Micro Edge Positioner).
//!
//! \param base is the base address of the EPWM module.
//! \param channel is high resolution period module.
//! \param mepEdgeMode edge of the PWM that is controlled by MEP (Micro Edge
//!        Positioner).
//!
//! This function sets the edge of the PWM that is controlled by MEP (Micro
//! Edge Positioner). Valid values for the parameters are:
//!  - channel
//!      - HRPWM_CHANNEL_A  - HRPWM A
//!      - HRPWM_CHANNEL_B  - HRPWM B
//!  - mepEdgeMode
//!      - HRPWM_MEP_CTRL_DISABLE                 - HRPWM is disabled
//!      - HRPWM_MEP_CTRL_RISING_EDGE             - MEP (Micro Edge Positioner)
//!                                                 controls rising edge.
//!      - HRPWM_MEP_CTRL_FALLING_EDGE            - MEP (Micro Edge Positioner)
//!                                                 controls falling edge.
//!      - HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE - MEP (Micro Edge Positioner)
//!                                                 controls both edges.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setMEPEdgeSelect(uint32_t base, HRPWM_Channel channel,
                       HRPWM_MEPEdgeMode mepEdgeMode)
{
    //
    // Set the edge mode
    //
    HW_WR_REG16(base + CSL_EPWM_HRCNFG,
        ((HW_RD_REG16(base + CSL_EPWM_HRCNFG) &
        ~(CSL_EPWM_HRCNFG_EDGMODE_MAX << (uint16_t)channel)) |
        ((uint16_t)mepEdgeMode << (uint16_t)channel)));
}

//*****************************************************************************
//
//! Sets the MEP (Micro Edge Positioner) control mode.
//!
//! \param base is the base address of the EPWM module.
//! \param channel is high resolution period module.
//! \param mepCtrlMode is the MEP (Micro Edge Positioner) control mode.
//!
//! This function sets the mode (register type) the MEP (Micro Edge Positioner)
//! will control. Valid values for the parameters are:
//!  - channel
//!      - HRPWM_CHANNEL_A  - HRPWM A
//!      - HRPWM_CHANNEL_B  - HRPWM B
//!  - mepCtrlMode
//!      - HRPWM_MEP_DUTY_PERIOD_CTRL      - MEP (Micro Edge Positioner) is
//!                                          controlled by value of CMPAHR/
//!                                          CMPBHR(depending on the value of
//!                                          channel) or TBPRDHR.
//!      - HRPWM_MEP_PHASE_CTRL            - MEP (Micro Edge Positioner) is
//!                                          controlled by TBPHSHR.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setMEPControlMode(uint32_t base, HRPWM_Channel channel,
                        HRPWM_MEPCtrlMode mepCtrlMode)
{
    //
    // Set the MEP control
    //
    HW_WR_REG16(base + CSL_EPWM_HRCNFG,
        ((HW_RD_REG16(base + CSL_EPWM_HRCNFG) &
        ~(CSL_EPWM_HRCNFG_CTLMODE_MAX << ((uint16_t)channel + 2U))) |
        ((uint16_t)mepCtrlMode << ((uint16_t)channel + 2U))));
}

//*****************************************************************************
//
//! Sets the high resolution comparator load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param channel is high resolution period module.
//! \param loadEvent is the MEP (Micro Edge Positioner) control mode.
//!
//! This function sets the shadow load mode of the high resolution comparator.
//! The function sets the COMPA or COMPB register depending on the channel
//! variable.
//! Valid values for the parameters are:
//!  - channel
//!      - HRPWM_CHANNEL_A  - HRPWM A
//!      - HRPWM_CHANNEL_B  - HRPWM B
//!  - loadEvent
//!      - HRPWM_LOAD_ON_CNTR_ZERO         - load when counter equals zero
//!      - HRPWM_LOAD_ON_CNTR_PERIOD       - load when counter equals period
//!      - HRPWM_LOAD_ON_CNTR_ZERO_PERIOD  - load when counter equals zero or
//!                                          period
//!      - HRPWM_LOAD_ON_CMPB_EQ           - load on translater event CMPB-3
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setCounterCompareShadowLoadEvent(uint32_t base, HRPWM_Channel channel,
                                       HRPWM_LoadMode loadEvent)
{
    //
    // Set the CMPAHR or CMPBHR load mode
    //
    HW_WR_REG16(base + CSL_EPWM_HRCNFG,
        ((HW_RD_REG16(base + CSL_EPWM_HRCNFG) &
        ~(CSL_EPWM_HRCNFG_HRLOAD_MAX << ((uint16_t)channel + 3U))) |
        ((uint16_t)loadEvent << ((uint16_t)channel + 3U))));
}

//*****************************************************************************
//
//! Sets the high resolution output swap mode.
//!
//! \param base is the base address of the EPWM module.
//! \param enableOutputSwap is the output swap flag.
//!
//! This function sets the HRPWM output swap mode. If enableOutputSwap is true,
//! ePWMxA signal appears on ePWMxB output and ePWMxB signal appears on ePWMxA
//! output. If it is false ePWMxA and ePWMxB outputs are unchanged.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setOutputSwapMode(uint32_t base, bool enableOutputSwap)
{
    //
    // Set output swap mode
    //
    if(enableOutputSwap)
    {
        HW_WR_REG16(base + CSL_EPWM_HRCNFG,
            HW_RD_REG16(base + CSL_EPWM_HRCNFG) | CSL_EPWM_HRCNFG_SWAPAB_MASK);
    }
    else
    {
        HW_WR_REG16(base + CSL_EPWM_HRCNFG,
            HW_RD_REG16(base + CSL_EPWM_HRCNFG) & ~CSL_EPWM_HRCNFG_SWAPAB_MASK);
    }
}

//*****************************************************************************
//
//! Sets the high resolution output on ePWMxB
//!
//! \param base is the base address of the EPWM module.
//! \param outputOnB is the output signal on ePWMxB.
//!
//! This function sets the HRPWM output signal on ePWMxB. If outputOnB is
//! HRPWM_OUTPUT_ON_B_INV_A, ePWMxB output is an inverted version of
//! ePWMxA. If outputOnB is HRPWM_OUTPUT_ON_B_NORMAL, ePWMxB output is
//! ePWMxB.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setChannelBOutputPath(uint32_t base, HRPWM_ChannelBOutput outputOnB)
{
    //
    // Set the output on ePWM B
    //
    HW_WR_REG16(base + CSL_EPWM_HRCNFG,
        ((HW_RD_REG16(base + CSL_EPWM_HRCNFG) & ~(uint16_t)(CSL_EPWM_HRCNFG_SELOUTB_MASK)) |
        (uint16_t)((uint16_t)outputOnB << CSL_EPWM_HRCNFG_SELOUTB_SHIFT)));
}

//*****************************************************************************
//
//! Enables MEP (Micro Edge Positioner) automatic scale mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the MEP (Micro Edge Positioner) to automatically
//! scale HRMSTEP.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_enableAutoConversion(uint32_t base)
{
    //
    // Enable MEP automatic scale
    //
    HW_WR_REG16(base + CSL_EPWM_HRCNFG,
        HW_RD_REG16(base + CSL_EPWM_HRCNFG) | CSL_EPWM_HRCNFG_AUTOCONV_MASK);
}

//*****************************************************************************
//
//! Disables MEP automatic scale mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the MEP (Micro Edge Positioner) from automatically
//! scaling HRMSTEP.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_disableAutoConversion(uint32_t base)
{
    //
    // Disable MEP automatic scale
    //
    HW_WR_REG16(base + CSL_EPWM_HRCNFG,
        HW_RD_REG16(base + CSL_EPWM_HRCNFG) & ~CSL_EPWM_HRCNFG_AUTOCONV_MASK);
}

//*****************************************************************************
//
//! Enable high resolution period feature.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the high resolution period feature.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_enablePeriodControl(uint32_t base)
{
    //
    // Set HRPE bit
    //
    HW_WR_REG16(base + CSL_EPWM_HRPCTL,
        HW_RD_REG16(base + CSL_EPWM_HRPCTL) | CSL_EPWM_HRPCTL_HRPE_MASK);
}

//*****************************************************************************
//
//! Disable high resolution period feature.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the high resolution period feature.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_disablePeriodControl(uint32_t base)
{
    //
    // Clear HRPE bit
    //
    HW_WR_REG16(base + CSL_EPWM_HRPCTL,
        HW_RD_REG16(base + CSL_EPWM_HRPCTL) & ~CSL_EPWM_HRPCTL_HRPE_MASK);
}

//*****************************************************************************
//
//! Enable high resolution phase load
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables loading of high resolution phase shift value which is
//! set by the function HRPWM_setPhaseShift().
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_enablePhaseShiftLoad(uint32_t base)
{
    //
    // Set TBPHSHRLOADE bit
    //
    HW_WR_REG16(base + CSL_EPWM_HRPCTL,
        HW_RD_REG16(base + CSL_EPWM_HRPCTL) | CSL_EPWM_HRPCTL_TBPHSHRLOADE_MASK);
}

//*****************************************************************************
//
//! Disable high resolution phase load
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables loading of high resolution phase shift value.
//!
//! \return
//
//*****************************************************************************
static inline void
HRPWM_disablePhaseShiftLoad(uint32_t base)
{
    //
    // Clear TBPHSHRLOADE bit
    //
    HW_WR_REG16(base + CSL_EPWM_HRPCTL,
        HW_RD_REG16(base + CSL_EPWM_HRPCTL) & ~CSL_EPWM_HRPCTL_TBPHSHRLOADE_MASK);
}

//*****************************************************************************
//
//! Set high resolution PWMSYNC source.
//!
//! \param base is the base address of the EPWM module.
//! \param syncPulseSource is the PWMSYNC source.
//!
//! This function sets the high resolution PWMSYNC pulse source.
//! Valid values for syncPulseSource are:
//!  - HRPWM_PWMSYNC_SOURCE_PERIOD      - Counter equals Period.
//!  - HRPWM_PWMSYNC_SOURCE_ZERO        - Counter equals zero.
//!  - HRPWM_PWMSYNC_SOURCE_COMPC_UP    - Counter equals COMPC when
//!                                       counting up.
//!  - HRPWM_PWMSYNC_SOURCE_COMPC_DOWN  - Counter equals COMPC when
//!                                       counting down.
//!  - HRPWM_PWMSYNC_SOURCE_COMPD_UP    - Counter equals COMPD when
//!                                       counting up.
//!  - HRPWM_PWMSYNC_SOURCE_COMPD_DOWN  - Counter equals COMPD when
//!                                       counting down.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setSyncPulseSource(uint32_t base, HRPWM_SyncPulseSource syncPulseSource)
{
    //
    // Set the PWMSYNC source
    //

    //
    // Configuration for sync pulse source equal to HRPWM_PWMSYNC_SOURCE_PERIOD
    // or HRPWM_PWMSYNC_SOURCE_ZERO
    //
    if(syncPulseSource < HRPWM_PWMSYNC_SOURCE_COMPC_UP)
    {
        HW_WR_REG16(base + CSL_EPWM_HRPCTL,
            ((HW_RD_REG16(base + CSL_EPWM_HRPCTL) &
            ~(CSL_EPWM_HRPCTL_PWMSYNCSELX_MASK | CSL_EPWM_HRPCTL_PWMSYNCSEL_MASK)) |
            (uint16_t)((uint16_t)syncPulseSource << 1U)));
    }
    else
    {
        HW_WR_REG16(base + CSL_EPWM_HRPCTL,
            ((HW_RD_REG16(base + CSL_EPWM_HRPCTL) & ~(uint16_t)CSL_EPWM_HRPCTL_PWMSYNCSELX_MASK) |
            (uint16_t)((uint16_t)syncPulseSource << CSL_EPWM_HRPCTL_PWMSYNCSELX_SHIFT)));
    }
}

//*****************************************************************************
//
//! Sets the Translator Remainder value.
//!
//! \param base is the base address of the EPWM module.
//! \param trremVal is the translator remainder value.
//!
//! This function sets the Translator Remainder value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setTranslatorRemainder(uint32_t base, uint16_t trremVal)
{
    //
    // Check the arguments
    //
    DebugP_assert(trremVal <= CSL_EPWM_TRREM_TRREM_MAX);

    //
    // Set Translator Remainder value
    //
    HW_WR_REG16(base + CSL_EPWM_TRREM, trremVal & CSL_EPWM_TRREM_TRREM_MASK);
}

//*****************************************************************************
//
//! Sets the consolidated counter compare values in HR mode.
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the Counter Compare module.
//! \param compCount is the consolidated counter compare count value.
//!
//! This function sets the consolidated counter compare(CMPx:CMPxHR) value
//! required in high resolution mode for counter compare registers.
//! Valid values for compModule are:
//!  - HRPWM_COUNTER_COMPARE_A - counter compare A.
//!  - HRPWM_COUNTER_COMPARE_B - counter compare B.
//!
//! \b Note: compCount is a 24 bit value.
//! \b Note: For configuring CMPA = 0xB4, CMPAHR = 0x64; value of
//!          compCount = 0xB464
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setCounterCompareValue(uint32_t base,
                             HRPWM_CounterCompareModule compModule,
                             uint32_t compCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(compCount <= 0xFFFFFFFFU);

    //
    // Write to counter compare registers
    //
    if(compModule == HRPWM_COUNTER_COMPARE_A)
    {
        //
        // Write to CMPA:CMPAHR
        //
        HW_WR_REG32(base + CSL_EPWM_CMPA, compCount << 8);
    }
    else
    {
        //
        // Write to CMPB:CMPBHR
        //
        HW_WR_REG32(base + CSL_EPWM_CMPB, compCount << 8);
    }
}

//*****************************************************************************
//
//! Gets the consolidated counter compare values.
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the Counter Compare module value.
//!
//! This function gets the consolidated counter compare(CMPx:CMPxHR) value
//! used in high resolution for the counter compare module specified.
//! Valid values for compModule are:
//!  - HRPWM_COUNTER_COMPARE_A - counter compare A.
//!  - HRPWM_COUNTER_COMPARE_B - counter compare B.
//!
//!
//! \return None.
//
//*****************************************************************************
static inline uint32_t
HRPWM_getCounterCompareValue(uint32_t base,
                             HRPWM_CounterCompareModule compModule)
{
    uint32_t compCount;

    //
    // Get counter compare value for selected module
    //
    if(compModule == HRPWM_COUNTER_COMPARE_A)
    {
        //
        // Read from CMPAHR
        //
        compCount = HW_RD_REG32(base + CSL_EPWM_CMPA);
    }
    else
    {
        //
        // Read from CMPBHR
        //
        compCount = HW_RD_REG32(base + CSL_EPWM_CMPB);
    }

    return(compCount>>8U);
}

//*****************************************************************************
//
//! Sets the high resolution counter compare value.
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the Counter Compare module.
//! \param hrCompCount is the high resolution counter compare count value.
//!
//! This function sets the high resolution counter compare value(CMPxHR) for
//! counter compare registers.
//! Valid values for compModule are:
//!  - HRPWM_COUNTER_COMPARE_A - counter compare A.
//!  - HRPWM_COUNTER_COMPARE_B - counter compare B.
//!
//! \b Note: hrCompCount is an 8-bit value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setHiResCounterCompareValue(uint32_t base,
                                 HRPWM_CounterCompareModule compModule,
                                 uint16_t hrCompCount)
{
    //
    // Check the arguments
    //
    DebugP_assert((hrCompCount << 8U) <= CSL_EPWM_CMPA_CMPAHR_MAX);

    //
    // Write to the high resolution counter compare registers
    //
    if(compModule == HRPWM_COUNTER_COMPARE_A)
    {
        //
        // Write to CMPAHR
        //
        HW_WR_REG32(base + CSL_EPWM_CMPA,
            (HW_RD_REG32(base + CSL_EPWM_CMPA) & ~CSL_EPWM_CMPA_CMPAHR_MASK) | (((uint32_t)hrCompCount << (uint32_t)8U) & CSL_EPWM_CMPA_CMPAHR_MASK));
    }
    else
    {
        //
        // Write to CMPBHR
        //
        HW_WR_REG32(base + CSL_EPWM_CMPB,
            (HW_RD_REG32(base + CSL_EPWM_CMPB) & ~CSL_EPWM_CMPA_CMPAHR_MASK) | (((uint32_t)hrCompCount << (uint32_t)8U) & CSL_EPWM_CMPA_CMPAHR_MASK));
    }
}

//*****************************************************************************
//
//! Gets the high resolution counter compare values.
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the Counter Compare module value.
//!
//! This function gets only the high resolution counter compare(CMPxHR) value
//! for the counter compare module specified.
//! Valid values for compModule are:
//!  - HRPWM_COUNTER_COMPARE_A - counter compare A.
//!  - HRPWM_COUNTER_COMPARE_B - counter compare B.
//!
//! \return None.
//
//*****************************************************************************
static inline uint16_t
HRPWM_getHiResCounterCompareValueOnly(uint32_t base,
                                      HRPWM_CounterCompareModule compModule)
{
    uint16_t hrCompCount;

    //
    // Get counter compare value for selected module
    //
    if(compModule == HRPWM_COUNTER_COMPARE_A)
    {
        //
        // Read from CMPAHR
        //
        hrCompCount = (HW_RD_REG16(base + CSL_EPWM_CMPA) & CSL_EPWM_CMPA_CMPAHR_MASK);
    }
    else
    {
        //
        // Read from CMPBHR
        //
        hrCompCount = (HW_RD_REG16(base + CSL_EPWM_CMPB) & CSL_EPWM_CMPB_CMPBHR_MASK);
    }

    return(hrCompCount >> 8U);
}

//*****************************************************************************
//
//! Sets the high resolution RED count.
//!
//! \param base is the base address of the EPWM module.
//! \param hrRedCount is the high resolution RED count.
//!
//! This function sets only the high resolution RED (Rising Edge Delay)
//! count(DBREDHR) value.
//! The value of hrRedCount should be less than 128.
//!
//! \b Note: hrRedCount is a 7-bit value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setHiResRisingEdgeDelay(uint32_t base, uint16_t hrRedCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(hrRedCount <= CSL_EPWM_DBREDHR_DBREDHR_MAX);

    //
    // Set the High Resolution RED (Rising Edge Delay) count only
    //
    HW_WR_REG16(base + CSL_EPWM_DBREDHR,
      (HW_RD_REG16(base + CSL_EPWM_DBREDHR) & ~CSL_EPWM_DBREDHR_DBREDHR_MASK ) |
      ((uint32_t)hrRedCount << CSL_EPWM_DBREDHR_DBREDHR_SHIFT));
}

//*****************************************************************************
//
//! Sets high resolution FED count.
//!
//! \param base is the base address of the EPWM module.
//! \param hrFedCount is the high resolution FED count.
//!
//! This function sets only the high resolution FED (Falling Edge Delay) count
//! (DBFEDHR)value. The value of hrFedCount should be less than 128.
//!
//! \b Note: hrFedCount is a 7-bit value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setHiResFallingEdgeDelayOnly(uint32_t base, uint16_t hrFedCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(hrFedCount <= CSL_EPWM_DBFEDHR_DBFEDHR_MAX);

    //
    // Set the high resolution FED (Falling Edge Delay) count
    //
    HW_WR_REG16(base + CSL_EPWM_DBFEDHR,
        (HW_RD_REG16(base + CSL_EPWM_DBFEDHR) &
        ~CSL_EPWM_DBFEDHR_DBFEDHR_MASK) |
        ((uint32_t)hrFedCount << CSL_EPWM_DBFEDHR_DBFEDHR_SHIFT));
}

//*****************************************************************************
//
//! Set high resolution MEP (Micro Edge Positioner) step.
//!
//! \param base is the base address of the EPWM module.
//! \param mepCount is the high resolution MEP (Micro Edge Positioner) step
//!        count.
//!
//! This function sets the high resolution MEP (Micro Edge Positioner) step
//! count. The maximum value for the MEP count step is 255.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setMEPStep(uint32_t base, uint16_t mepCount)
{
    //
    // Check the arguments
    //
    DebugP_assert(mepCount <= CSL_OTTOCAL_HRMSTEP_HRMSTEP_MAX);

    //
    // Set HRPWM MEP count
    //
    HW_WR_REG16(base + CSL_OTTOCAL_HRMSTEP,
        ((HW_RD_REG16(base + CSL_OTTOCAL_HRMSTEP) & ~CSL_OTTOCAL_HRMSTEP_HRMSTEP_MASK) |
        ((uint32_t)mepCount << CSL_OTTOCAL_HRMSTEP_HRMSTEP_SHIFT)));
}

//*****************************************************************************
//
//! Set high resolution Dead Band MEP (Micro Edge Positioner) control.
//!
//! \param base is the base address of the EPWM module.
//! \param mepDBEdge is the high resolution MEP (Micro Edge Positioner) control
//!        edge.
//!
//! This function sets the high resolution Dead Band edge that the MEP (Micro
//! Edge Positioner) controls Valid values for mepDBEdge are:
//!   - HRPWM_DB_MEP_CTRL_DISABLE  - HRPWM is disabled
//!   - HRPWM_DB_MEP_CTRL_RED      - MEP (Micro Edge Positioner) controls
//!                                  Rising Edge Delay
//!   - HRPWM_DB_MEP_CTRL_FED      - MEP (Micro Edge Positioner) controls
//!                                  Falling Edge Delay
//!   - HRPWM_DB_MEP_CTRL_RED_FED  - MEP (Micro Edge Positioner) controls both
//!                                  Falling and Rising edge delays
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setDeadbandMEPEdgeSelect(uint32_t base,
                              HRPWM_MEPDeadBandEdgeMode mepDBEdge)
{
    //
    // Set the HRPWM DB edge mode
    //
    HW_WR_REG16(base + CSL_EPWM_HRCNFG2,
        ((HW_RD_REG16(base + CSL_EPWM_HRCNFG2) & ~(uint16_t)CSL_EPWM_HRCNFG2_EDGMODEDB_MASK) |
        (uint16_t)((uint16_t)mepDBEdge << CSL_EPWM_HRCNFG2_EDGMODEDB_SHIFT)));
}

//*****************************************************************************
//
//! Set the high resolution Dead Band RED load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param loadEvent is the shadow to active load event.
//!
//! This function sets the high resolution Rising Edge Delay(RED)Dead Band
//! count load mode.
//! Valid values for loadEvent are:
//!   - HRPWM_LOAD_ON_CNTR_ZERO        - load when counter equals zero.
//!   - HRPWM_LOAD_ON_CNTR_PERIOD      - load when counter equals period
//!   - HRPWM_LOAD_ON_CNTR_ZERO_PERIOD - load when counter equals zero
//!                                             or period.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setRisingEdgeDelayLoadMode(uint32_t base,
                                 HRPWM_LoadMode loadEvent)
{
    //
    // Set the HRPWM RED load mode
    //
    HW_WR_REG16(base + CSL_EPWM_HRCNFG2,
        ((HW_RD_REG16(base + CSL_EPWM_HRCNFG2) & ~(uint16_t)CSL_EPWM_HRCNFG2_CTLMODEDBRED_MASK) |
        (uint16_t)((uint16_t)loadEvent << CSL_EPWM_HRCNFG2_CTLMODEDBRED_SHIFT)));
}

//*****************************************************************************
//
//! Set the high resolution Dead Band FED load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param loadEvent is the shadow to active load event.
//!
//! This function sets the high resolution Falling Edge Delay(FED) Dead Band
//! count load mode.
//! Valid values for loadEvent are:
//!   - HRPWM_LOAD_ON_CNTR_ZERO        - load when counter equals zero.
//!   - HRPWM_LOAD_ON_CNTR_PERIOD      - load when counter equals period
//!   - HRPWM_LOAD_ON_CNTR_ZERO_PERIOD - load when counter equals zero
//!                                             or period.
//!
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setFallingEdgeDelayLoadMode(uint32_t base, HRPWM_LoadMode loadEvent)
{
    //
    // Set the HRPWM FED load mode
    //
    HW_WR_REG16(base + CSL_EPWM_HRCNFG2,
        ((HW_RD_REG16(base + CSL_EPWM_HRCNFG2) & ~(uint16_t)CSL_EPWM_HRCNFG2_CTLMODEDBFED_MASK) |
        (uint16_t)((uint16_t)loadEvent << CSL_EPWM_HRCNFG2_CTLMODEDBFED_SHIFT)));
}

//*****************************************************************************
//
//! Writes high resolution values to xcmp registers
//!
//! \param base is the base address of the EPWM module.
//! \param xcmpReg is the xcmp register offset
//! \param xcmpvalue is the value to be written to xcmp registers
//! This function writes xcmpvalue to xcmp registers.
//! Valid values for xcmpReg are:
//!    HRPWM_XCMP[1-8]_[ACTIVE/SHADOW1/SHADOW2/SHADOW3]  -XCMP[1-8]_[ACTIVE/SHADOW1/SHADOW2/SHADOW3]
//!    HRPWM_XTBPRD_ACTIVE                               -XTBPRD_ACTIVE
//!    HRPWM_XTBPRD_SHADOW1                              -XTBPRD_SHADOW1
//!    HRPWM_XTBPRD_SHADOW2                              -XTBPRD_SHADOW2
//!    HRPWM_XTBPRD_SHADOW3                              -XTBPRD_SHADOW3
//! \return None.
//
//*****************************************************************************
static inline void
HRPWM_setXCMPRegValue(uint32_t base, HRPWM_XCMPReg xcmpReg,
                            uint16_t xcmpvalue)
{
    uint32_t registerOffset;

    //
    // Get the register offset for the Counter compare
    //
    registerOffset = base + CSL_EPWM_XCMP1_ACTIVE + (uint16_t)xcmpReg;

    //
    // Write to the xcmp registers.
    //
    HW_WR_REG16(registerOffset, xcmpvalue);
}
//
// XCMP related APIs
//
//*****************************************************************************
//
//! Enable ePWM xcmp mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the ePWM xcmp mode.
//!
//! \return None.
//
//*****************************************************************************

static inline void
EPWM_enableXCMPMode(uint32_t base)
{
    uint32_t registerOffset = base + CSL_EPWM_XCMPCTL1;

    HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) | CSL_EPWM_XCMPCTL1_XCMPEN_MAX ));
}

//*****************************************************************************
//
//! Disable ePWM xcmp mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the ePWM xcmp mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableXCMPMode(uint32_t base)
{
    uint32_t registerOffset = base + CSL_EPWM_XCMPCTL1;

    HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) & ~CSL_EPWM_XCMPCTL1_XCMPEN_MAX ));
}


//*****************************************************************************
//
//! Enable ePWM xcmp Split.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function splits 8 xcmp registers between CMPA and CMPB equally.
//!
//! \return None.
//
//*****************************************************************************

static inline void
EPWM_enableSplitXCMP(uint32_t base)
{
    uint32_t registerOffset = base + CSL_EPWM_XCMPCTL1;
    uint32_t offset = CSL_EPWM_XCMPCTL1_XCMPSPLIT_SHIFT;

    HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) | ((uint32_t) CSL_EPWM_XCMPCTL1_XCMPSPLIT_MAX << offset )));
}

//*****************************************************************************
//
//! Disable ePWM xcmp Split.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function allocates 8 xcmp registers to CMPA.
//!
//! \return None.
//
//*****************************************************************************

static inline void
EPWM_disableSplitXCMP(uint32_t base)
{
    uint32_t registerOffset = base + CSL_EPWM_XCMPCTL1;
    uint32_t offset = CSL_EPWM_XCMPCTL1_XCMPSPLIT_SHIFT;

    HW_WR_REG32(registerOffset,
      (HW_RD_REG32(registerOffset) & ~( CSL_EPWM_XCMPCTL1_XCMPSPLIT_MAX << offset )));

}

//*****************************************************************************
//
//! Allocates xcmp registers to CMPA.
//!
//! \param base is the base address of the EPWM module.
//! \param alloctype is range of xcmp registers to be allocated to CMPA

//! This function gives us choice to allocate range of xcmp registers to CMPA
//! Valid values for alloctype are:
//!    EPWM_XCMP_NONE_CMPA      - Allocate 0 XCMP registers to CMPA
//!    EPWM_XCMP_1_CMPA         - Allocate XCMP1 register to CMPA
//!    EPWM_XCMP_2_CMPA         - Allocate XCMP1- XCMP2 registers to CMPA
//!    EPWM_XCMP_3_CMPA         - Allocate XCMP1- XCMP3 register to CMPA
//!    EPWM_XCMP_4_CMPA         - Allocate XCMP1- XCMP4 registers to CMPA
//!    EPWM_XCMP_5_CMPA         - Allocate XCMP1- XCMP5 register to CMPA
//!    EPWM_XCMP_6_CMPA         - Allocate XCMP1- XCMP6 registers to CMPA
//!    EPWM_XCMP_7_CMPA         - Allocate XCMP1- XCMP7 register to CMPA
//!    EPWM_XCMP_8_CMPA         - Allocate XCMP1- XCMP8 register to CMPA
//! \return None.
//
//*****************************************************************************

static inline void
EPWM_allocAXCMP(uint32_t base, EPWM_XCMP_ALLOC_CMPA alloctype)
{
    uint32_t registerOffset = base + CSL_EPWM_XCMPCTL1;
    uint32_t offset = CSL_EPWM_XCMPCTL1_XCMPA_ALLOC_SHIFT;

   HW_WR_REG32(registerOffset,
        ( (HW_RD_REG32(registerOffset) & ~(uint32_t)CSL_EPWM_XCMPCTL1_XCMPA_ALLOC_MASK) | ( (uint32_t)alloctype << offset )));
}

//*****************************************************************************
//
//! Allocates xcmp registers to CMPB.
//!
//! \param base is the base address of the EPWM module.
//! \param alloctype is range of xcmp registers to be allocated to CMPB

//! This function gives us choice to allocate range of xcmp registers to CMPB
//! Valid values for alloctype are:
//!    EPWM_XCMP_1_CMPB         - Allocate XCMP5 register to CMPB
//!    EPWM_XCMP_2_CMPB         - Allocate XCMP5- XCMP6 registers to CMPB
//!    EPWM_XCMP_3_CMPB         - Allocate XCMP5- XCMP7 register to CMPB
//!    EPWM_XCMP_4_CMPB         - Allocate XCMP5- XCMP8 registers to CMPB
//! \return None.
//
//*****************************************************************************

static inline void
EPWM_allocBXCMP(uint32_t base, EPWM_XCMP_ALLOC_CMPB alloctype)
{
    uint32_t registerOffset = base + CSL_EPWM_XCMPCTL1;
    uint32_t offset = CSL_EPWM_XCMPCTL1_XCMPB_ALLOC_SHIFT;

    HW_WR_REG32(registerOffset,
        ( (HW_RD_REG32(registerOffset) & ~(uint32_t)CSL_EPWM_XCMPCTL1_XCMPB_ALLOC_MASK) | ( (uint32_t)alloctype << offset )));
}

//*****************************************************************************
//
//! Writes values to xcmp registers
//!
//! \param base is the base address of the EPWM module.
//! \param xcmpReg is the xcmp register offset
//! \param xcmpvalue is the value to be written to xcmp registers
//! This function writes xcmpvalue to xcmp registers.
//! Valid values for xcmpReg are:
//!    EPWM_XCMP[1-8]_[ACTIVE/SHADOW1/SHADOW2/SHADOW3]  -XCMP[1-8]_[ACTIVE/SHADOW1/SHADOW2/SHADOW3]
//!    EPWM_XTBPRD_ACTIVE                               -XTBPRD_ACTIVE
//!    EPWM_XTBPRD_SHADOW1                              -XTBPRD_SHADOW1
//!    EPWM_XTBPRD_SHADOW2                              -XTBPRD_SHADOW2
//!    EPWM_XTBPRD_SHADOW3                              -XTBPRD_SHADOW3
//! \return None.
//
//*****************************************************************************

static inline void
EPWM_setXCMPRegValue(uint32_t base, EPWM_XCMPReg xcmpReg,
                            uint16_t xcmpvalue)
{
    uint32_t registerOffset;

    //
    // Get the register offset for the Counter compare
    //
    registerOffset = base + CSL_EPWM_XCMP1_ACTIVE + (uint16_t)xcmpReg;

    //
    // Write to the xcmp registers.
    //
    HW_WR_REG16(registerOffset + 0x2U, xcmpvalue);
}
//*****************************************************************************
//
//! Writes values to CMPx Shadow registers
//!
//! \param base is the base address of the EPWM module.
//! \param cmpReg is the CMP register offset
//! \param cmpvalue is the value to be written to CMPC/D shadow registers
//! This function writes cmpvalue to CMPC/D shadow registers.
//! Valid values for cmpReg are:
//!    EPWM_CMPC_SHADOW1                             -CMPC_SHADOW1
//!    EPWM_CMPD_SHADOW1                             -CMPD_SHADOW1
//!    EPWM_CMPC_SHADOW2                             -CMPC_SHADOW2
//!    EPWM_CMPD_SHADOW2                             -CMPD_SHADOW2
//!    EPWM_CMPC_SHADOW3                             -CMPC_SHADOW3
//!    EPWM_CMPD_SHADOW3                             -CMPD_SHADOW3
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setCMPShadowRegValue(uint32_t base, EPWM_XCompareReg cmpReg,
                            uint16_t cmpvalue)
{
    //
    // Check the arguments
    //
    uint32_t registerOffset;
    registerOffset = base + CSL_EPWM_CMPC_SHDW1 + (uint32_t)cmpReg;

    //
    // Write to the CMPC/D Shadow registers.
    //
    HW_WR_REG16(registerOffset, cmpvalue);
}

//*****************************************************************************
//
//! Writes values to XMINMAX registers
//!
//! \param base is the base address of the EPWM module.
//! \param xminmaxReg is the XCMP register offset
//! \param xcmpvalue is the value to be written to XCMP registers
//! This function writes xcmpvalue to XCMP registers.
//! Valid values for xminmaxReg are:
//!    EPWM_XMIN_ACTIVE                              -XMIN_ACTIVE
//!    EPWM_XMAX_ACTIVE                              -XMAX_ACTIVE
//!    EPWM_XMIN_SHADOW1                             -XMIN_SHADOW1
//!    EPWM_XMAX_SHADOW1                             -XMAX_SHADOW1
//!    EPWM_XMIN_SHADOW2                             -XMIN_SHADOW2
//!    EPWM_XMAX_SHADOW2                             -XMAX_SHADOW2
//!    EPWM_XMIN_SHADOW3                             -XMIN_SHADOW3
//!    EPWM_XMAX_SHADOW3                             -XMAX_SHADOW3
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setXMINMAXRegValue(uint32_t base, EPWM_XMinMaxReg xminmaxReg,
                            uint16_t xcmpvalue)
{
    //
    // Check the arguments
    //
    uint32_t registerOffset;
    registerOffset = base +  CSL_EPWM_XMINMAX_ACTIVE + (uint16_t)xminmaxReg;

    //
    // Write to the XMINMAX register.
    //
    HW_WR_REG16(registerOffset, xcmpvalue);
}
//*****************************************************************************
//
//! Set up Action qualifier outputs based on XAQ registers
//!
//! \param base is the base address of the EPWM module.
//! \param shadowset is the shadow set number or active set.
//! \param epwmOutput is the ePWM pin type.
//! \param output is the Action Qualifier output.
//! \param event is the event that causes a change in output.
//!
//! This function sets up the Action Qualifier output on ePWM A or ePWMB,
//! depending on the value of epwmOutput, to a value specified by outPut based
//! on the input events - specified by event.
//! The following are valid values for the parameters.
//!   - epwmOutput
//!       - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!       - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!   -shadowset
//!       - EPWM_XCMP_ACTIVE          - XCMP set is Active
//!       - EPWM_XCMP_SHADOW1         - XCMP set is Shadow 1
//!       - EPWM_XCMP_SHADOW2         - XCMP set is Shadow 2
//!       - EPWM_XCMP_SHADOW3         - XCMP set is Shadow 3
//!   - output
//!       - EPWM_AQ_OUTPUT_NO_CHANGE  - No change in the output pins
//!       - EPWM_AQ_OUTPUT_LOW        - Set output pins to low
//!       - EPWM_AQ_OUTPUT_HIGH       - Set output pins to High
//!       - EPWM_AQ_OUTPUT_TOGGLE     - Toggle the output pins
//!   - event
//!    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1        -Time base counter equals XCMP1
//!    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2        -Time base counter equals XCMP2
//!    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3        -Time base counter equals XCMP3
//!    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4        -Time base counter equals XCMP4
//!    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5        -Time base counter equals XCMP5
//!    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6        -Time base counter equals XCMP6
//!    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7        -Time base counter equals XCMP7
//!    EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8        -Time base counter equals XCMP8
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setXCMPActionQualifierAction(uint32_t base, uint32_t shadowset,
                              EPWM_ActionQualifierOutputModule epwmOutput,
                              EPWM_ActionQualifierOutput output,
                              EPWM_XCMPActionQualifierOutputEvent event)
{
    uint32_t registerOffset;

    //
    // Get the register offset
    //

    if(shadowset == EPWM_XCMP_ACTIVE)
    {
        registerOffset = (uint32_t)CSL_EPWM_XAQCTLA_ACTIVE + (uint16_t)((uint16_t)epwmOutput/(uint32_t)2);

        HW_WR_REG16(base + registerOffset,
            ((HW_RD_REG16(base + registerOffset) &
            ~(CSL_EPWM_XAQCTLA_ACTIVE_XCMP1_MAX << (uint16_t)event)) |
            ((uint16_t)output << (uint16_t)event)));
    }
    else if(shadowset == EPWM_XCMP_SHADOW1)
    {
        registerOffset = (uint32_t)CSL_EPWM_XAQCTLA_SHDW1 + (uint16_t)((uint16_t)epwmOutput/(uint32_t)2);

        HW_WR_REG16(base + registerOffset,
        ((HW_RD_REG16(base + registerOffset) &
        ~(CSL_EPWM_XAQCTLA_SHDW1_XCMP1_MAX << (uint16_t)event)) |
        ((uint16_t)output << (uint16_t)event)));
    }
    else if(shadowset == EPWM_XCMP_SHADOW2)
    {
        registerOffset = (uint32_t)CSL_EPWM_XAQCTLA_SHDW2 + (uint16_t)((uint16_t)epwmOutput/(uint32_t)2);

        HW_WR_REG16(base + registerOffset,
        ((HW_RD_REG16(base + registerOffset) &
        ~(CSL_EPWM_XAQCTLA_SHDW2_XCMP1_MAX << (uint16_t)event)) |
        ((uint16_t)output << (uint16_t)event)));
    }
    else if(shadowset == EPWM_XCMP_SHADOW3)
    {
        registerOffset = (uint32_t)CSL_EPWM_XAQCTLA_SHDW3 + (uint32_t)((uint16_t)epwmOutput/(uint32_t)2);

        HW_WR_REG16(base + registerOffset,
        ((HW_RD_REG16(base + registerOffset) &
        ~(CSL_EPWM_XAQCTLA_SHDW3_XCMP1_MAX << (uint16_t)event)) |
        ((uint16_t)output << (uint16_t)event)));
    }
    else
    {
        /* do nothing */
    }

}

//*****************************************************************************
//
//! Enables ePWM xcmp reload event.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the ePWM xcmp reload event.
//!
//! \return None.
//
//*****************************************************************************

static inline void
EPWM_enableXLoad(uint32_t base)
{
    uint32_t registerOffset = base + CSL_EPWM_XLOAD;

    HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) | CSL_EPWM_XLOAD_STARTLD_MASK ));
}

//*****************************************************************************
//
//! Disables ePWM xcmp reload event.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the ePWM xcmp reload event.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableXLoad(uint32_t base)
{
    uint32_t registerOffset = base + CSL_EPWM_XLOAD;

    HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) & ~CSL_EPWM_XLOAD_STARTLD_MASK ));
}
//*****************************************************************************
//
//! Forces register loading from shadow buffers.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function is used for software force loading of the events in
//! global load mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_forceXLoad(uint32_t base)
{
    //
    // Check the arguments
    //
    uint32_t registerOffset;
    registerOffset = base + CSL_EPWM_XLOAD;

    HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) | CSL_EPWM_XLOAD_FRCLD_MASK ));
}
//*****************************************************************************
//
//! Selects the mode for the XCMP Shadow registers
//!
//! \param base is the base address of the EPWM module.
//! \param mode is load mode selection for XCMP Shadow registers

//! This function gives us choice to select the mode for XCMP shadow registers
//! Valid values for alloctype are:
//!    EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE         - Load mode is LOADONCE
//!    EPWM_XCMP_XLOADCTL_LOADMODE_LOADMULTIPLE     - Load mode is LOADMULTIPLE
//! \return None.
//
//*****************************************************************************

static inline void
EPWM_setXCMPLoadMode(uint32_t base,EPWM_XCMPXloadCtlLoadMode mode)
{
    uint32_t registerOffset;

    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_XLOADCTL;

    if(mode == EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE)
    {
        HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) & ~CSL_EPWM_XLOADCTL_LOADMODE_MASK));
    }
    else if(mode == EPWM_XCMP_XLOADCTL_LOADMODE_LOADMULTIPLE)
    {
        HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) | CSL_EPWM_XLOADCTL_LOADMODE_MASK));
    }
    else
    {
        /* do nothing */
    }
}

//*****************************************************************************
//
//! Selects the shadow register level allocation
//!
//! \param base is the base address of the EPWM module.
//! \param level is shadow register level allocation options

//! This function gives us choice to select the XCMP shadow registers level
//! Valid values for alloctype are:
//!    EPWM_XCMP_XLOADCTL_SHDWLEVEL_0         - Only Active register is available
//!    EPWM_XCMP_XLOADCTL_SHDWLEVEL_1         - SHDW1 and Active registers are available
//!    EPWM_XCMP_XLOADCTL_SHDWLEVEL_2         - SHDW2, SHDW1 and Active registers are available
//!    EPWM_XCMP_XLOADCTL_SHDWLEVEL_3         - SHDW3, SHDW2, SHDW1 and Active registers are available
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setXCMPShadowLevel(uint32_t base, EPWM_XCMP_XLOADCTL_SHDWLEVEL level)
{
    uint32_t registerOffset;

    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_XLOADCTL;

    HW_WR_REG32(registerOffset,
        ((HW_RD_REG32(registerOffset) & ~CSL_EPWM_XLOADCTL_SHDWLEVEL_MASK) |
        (uint32_t)((uint16_t)level << CSL_EPWM_XLOADCTL_SHDWLEVEL_SHIFT)));
}

//*****************************************************************************
//
//! Selects which shadow register set is in use
//!
//! \param base is the base address of the EPWM module.
//! \param ptr indicates current shadow buffer in use.

//! This function gives us choice to select the shadow buffer to use currently.
//! Valid values for alloctype are:
//!    EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL         - No Shadow buffer is in use
//!    EPWM_XCMP_XLOADCTL_SHDWBUFPTR_ONE          - Shadow buffer 1 is in use
//!    EPWM_XCMP_XLOADCTL_SHDWBUFPTR_TWO          - Shadow buffer 2 is in use
//!    EPWM_XCMP_XLOADCTL_SHDWBUFPTR_THREE        - Shadow buffer 3 is in use
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setXCMPShadowBufPtrLoadOnce(uint32_t base, EPWM_XCMP_XLOADCTL_SHDWBUFPTR ptr)
{
    uint32_t registerOffset;

    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_XLOADCTL;

    HW_WR_REG32(registerOffset,
        ((HW_RD_REG32(registerOffset) & ~CSL_EPWM_XLOADCTL_SHDWBUFPTR_LOADONCE_MASK) |
        (uint32_t)((uint16_t)ptr << CSL_EPWM_XLOADCTL_SHDWBUFPTR_LOADONCE_SHIFT)));
}

//*****************************************************************************
//
//! Chooses how many times Shadow buffer 2 or 3 is applied
//!
//! \param base is the base address of the EPWM module.
//! \param bufferset is the shadow set number.
//! \param count is number of times Shadow Buffer 2 or 3 is applied

//! This function gives us choice to select how many times shadow buffer 2 or 3 is applied. Max value is 7.
//! count = 0   - 1 time
//! count = 1   - 2 times .....
//! count = 7   - 8 times
//! Valid values for bufferset are:
//!    EPWM_XCMP_SHADOW2         - XCMP set is Shadow 2
//!    EPWM_XCMP_SHADOW3         - XCMP set is Shadow 3
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setXCMPShadowRepeatBufxCount(uint32_t base, uint32_t bufferset, uint8_t count)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_XLOADCTL;

    if(bufferset == EPWM_XCMP_SHADOW2)
    {
        HW_WR_REG32(registerOffset,
            ((HW_RD_REG32(registerOffset) & ~(CSL_EPWM_XLOADCTL_RPTBUF2PRD_MASK))
            | ((uint32_t)count<<CSL_EPWM_XLOADCTL_RPTBUF2PRD_SHIFT)) );
    }
    else if(bufferset == EPWM_XCMP_SHADOW3)
    {
         HW_WR_REG32(registerOffset,
        ((HW_RD_REG32(registerOffset) & ~(CSL_EPWM_XLOADCTL_RPTBUF3PRD_MASK))
        | ((uint32_t)count<<CSL_EPWM_XLOADCTL_RPTBUF3PRD_SHIFT)) );
    }
    else
    {
        /*do nothing */
    }
}

//*************************************************
//
// DIODE EMULATION LOGIC APIs
//

//*****************************************************************************
//
//! Enable ePWM diode emulation mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the ePWM diode emulation mode.
//!
//! \return None.
//
//*****************************************************************************

static inline void
EPWM_enableDiodeEmulationMode(uint32_t base)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DECTL;

    HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) | CSL_EPWM_DECTL_ENABLE_MAX ));

}

//*****************************************************************************
//
//! Disable ePWM diode emulation mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the ePWM diode emulation mode.
//!
//! \return None.
//
//*****************************************************************************

static inline void
EPWM_disableDiodeEmulationMode(uint32_t base)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DECTL;

    HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) & ~CSL_EPWM_DECTL_ENABLE_MAX ));

}

//*****************************************************************************
//
//! Selects the mode for the Diode emulation mode
//!
//! \param base is the base address of the EPWM module.
//! \param mode is load mode selection for Diode emulation mode

//! This function gives us choice to select the mode for Diode emulation mode
//!  In CBC mode, on every PWMSYNCOUT, set condition of DEACTIVE flag is evaluated.
//! If the set condition is not present the flag is cleared.
//! In OST mode, once DEACTIVE mode is entered, it will remain there until Software cleared.
//! Valid values for mode are:
//!    EPWM_DIODE_EMULATION_CBC         - Diode Emulation mode is Cycle by Cycle
//!    EPWM_DIODE_EMULATION_OST         - Diode Emulation mode is One Shot
//! \return None.
//
//*****************************************************************************

static inline void
EPWM_setDiodeEmulationMode(uint32_t base,EPWM_DiodeEmulationMode mode)
{
    uint32_t registerOffset;

    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DECTL;

    if(mode == EPWM_DIODE_EMULATION_CBC)
    {
        HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) & ~CSL_EPWM_DECTL_MODE_MASK));
    }
    else if(mode == EPWM_DIODE_EMULATION_OST)
    {
        HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) | CSL_EPWM_DECTL_MODE_MASK));
    }
    else
    {
        /* do nothing */
    }
}

//*****************************************************************************
//
//! Set ePWM diode emulation re-entry delay.
//!
//! \param base is the base address of the EPWM module.
//! \param delay is the re-entry delay in terms of number of PWMSYNCOUT events.
//! This function determines the blocking window after DEACTIVE flag is cleared
//! in which setting of DEACTIVE flag is prevented from being set.
//! Max value of delay is 256.
//!
//! \return None.
//
//*****************************************************************************

static inline void
EPWM_setDiodeEmulationReentryDelay(uint32_t base,uint8_t delay)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DECTL;

     HW_WR_REG32(registerOffset,
        ((HW_RD_REG32(registerOffset) & ~(CSL_EPWM_DECTL_REENTRYDLY_MASK))
        | ((uint32_t)delay<<CSL_EPWM_DECTL_REENTRYDLY_SHIFT)) );
}

//*****************************************************************************
//
//! Set ePWM diode emulation trip source for TripH and TripL.
//!
//! \param base is the base address of the EPWM module.
//! \param source is the trip source
//! \param tripLorH is to choose between tripL and tripH
//! This function is used to configure the comparator whose trip sources will be used.
//! Valid values for source are:
//!    EPWM_DE_TRIP_SRC_INPUTXBAR_OUT[0-31]     - Diode Emulation Trip Source is INPUTXBAR out[0-31] signal
//!    EPWM_DE_TRIP_SRC_CMPSSA[0-9]             - Diode Emulation Trip Source is CMPSSA[0-9] signal
//!    EPWM_DE_TRIP_SRC_CMPSSB[0-9]             - Diode Emulation Trip Source is CMPSSB[0-9] signal
//!
//! Valid values for tripLorH are:
//!    EPWM_DE_TRIPL                            - TripL
//!    EPWM_DE_TRIPH                            - TripH
//! \return None.
//*****************************************************************************

static inline void
EPWM_configureDiodeEmulationTripSources(uint32_t base, EPWM_DiodeEmulationTripSource source,
                                        uint32_t tripLorH)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DECOMPSEL;

    if(tripLorH == EPWM_DE_TRIPL)
    {
        HW_WR_REG32(registerOffset,
            ((HW_RD_REG32(registerOffset) &
            ~CSL_EPWM_DECOMPSEL_TRIPL_MASK) |
            ((uint32_t)source<<CSL_EPWM_DECOMPSEL_TRIPL_SHIFT)));
    }
    else if(tripLorH == EPWM_DE_TRIPH)
    {
        HW_WR_REG32(registerOffset,
            ((HW_RD_REG32(registerOffset) &
            ~CSL_EPWM_DECOMPSEL_TRIPH_MASK) |
            ((uint32_t)source<<CSL_EPWM_DECOMPSEL_TRIPH_SHIFT)));
    }
    else
    {
        /* do nothing */
    }

}

//*****************************************************************************
//
//! Selects ePWM diode emulation pwm signal.
//!
//! \param base is the base address of the EPWM module.
//! \param channel is the epwm output channel
//! \param signal is the signal selected by the TRIPSEL[A/B]
//! This function is used to select the pwm signal in DE mode.
//! Valid values for channel are:
//!    EPWM_DE_CHANNEL_A     - Diode emulation channel A
//!    EPWM_DE_CHANNEL_B     - Diode emulation channel B
//!
//! Valid values for signal are:
//!    EPWM_DE_SYNC_TRIPHorL               - synchronized version of TRIPH or TRIPL signal
//!    EPWM_DE_SYNC_INV_TRIPHorL           - synchronized and inverted version of TRIPH or TRIPL signal
//!    EPWM_DE_LOW                         - a constant low signal
//!    EPWM_DE_HIGH                        - a constant high signal
//! \return None.
//*****************************************************************************

static inline void
EPWM_selectDiodeEmulationPWMsignal(uint32_t base,uint32_t channel,
                                   EPWM_DiodeEmulationSignal signal)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DEACTCTL;

    if(channel == EPWM_DE_CHANNEL_A)
    {
        HW_WR_REG32(registerOffset,
            ((HW_RD_REG32(registerOffset) &
            ~CSL_EPWM_DEACTCTL_PWMA_MASK) |
            ((uint32_t)signal<<CSL_EPWM_DEACTCTL_PWMA_SHIFT)));
    }
    else
    {
        HW_WR_REG32(registerOffset,
            ((HW_RD_REG32(registerOffset) &
            ~CSL_EPWM_DEACTCTL_PWMB_MASK) |
            ((uint32_t)signal<<CSL_EPWM_DEACTCTL_PWMB_SHIFT)));
    }
}

//*****************************************************************************
//
//! Selects between TRIPH and TRIPL for an epwm channel in DE mode.
//!
//! \param base is the base address of the EPWM module.
//! \param channel is the epwm output channel
//! \param signal is the signal selected by the TRIPSEL[A/B]
//! This function is used to select between TRIPH and TRIPL for a channel in DE mode.
//! Valid values for channel are:
//!    EPWM_DE_CHANNEL_A     - Diode emulation channel A
//!    EPWM_DE_CHANNEL_B     - Diode emulation channel B
//!
//! Valid values for signal are:
//! 0 for TRIPH
//! 1 for TRIPL
//! \return None.
//*****************************************************************************

static inline void
EPWM_selectDiodeEmulationTripSignal(uint32_t base,uint32_t channel,
                                   uint32_t signal)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DEACTCTL;

    if(channel == EPWM_DE_CHANNEL_A)
    {
        HW_WR_REG32(registerOffset,
            ((HW_RD_REG32(registerOffset) &
            ~CSL_EPWM_DEACTCTL_TRIPSELA_MASK) |
            (signal<<CSL_EPWM_DEACTCTL_TRIPSELA_SHIFT)));
    }
    else
    {
        HW_WR_REG32(registerOffset,
            ((HW_RD_REG32(registerOffset) &
            ~CSL_EPWM_DEACTCTL_TRIPSELB_MASK) |
            (signal<<CSL_EPWM_DEACTCTL_TRIPSELB_SHIFT)));
    }
}

//*****************************************************************************
//
//! PWMTRIP does not bypass the diode emulation logic
//!
//! \param base is the base address of the EPWM module.
//! This function is used to go through DEL.
//! \return None.
//*****************************************************************************

static inline void
EPWM_nobypassDiodeEmulationLogic(uint32_t base)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DEACTCTL;

    HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) &
        ~(CSL_EPWM_DEACTCTL_TRIPENABLE_MAX << CSL_EPWM_DEACTCTL_TRIPENABLE_SHIFT)));

}

//*****************************************************************************
//
//! PWMTRIP bypasses the diode emulation PWM generation logic
//!
//! \param base is the base address of the EPWM module.
//! This function is used to bypass DEL.
//! \return None.
//*****************************************************************************

static inline void
EPWM_bypassDiodeEmulationLogic(uint32_t base)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DEACTCTL;

    HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) |
        (CSL_EPWM_DEACTCTL_TRIPENABLE_MAX << CSL_EPWM_DEACTCTL_TRIPENABLE_SHIFT)));

}

//*****************************************************************************
//
//! Forces DEACTIVE flag to 1
//!
//! \param base is the base address of the EPWM module.
//! This function is used to force DEACTIVE flag to 1.
//! \return None.
//*****************************************************************************

static inline void
EPWM_forceDiodeEmulationActive(uint32_t base)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DEFRC;

     HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) | CSL_EPWM_DEFRC_DEACTIVE_MASK));

}

//*****************************************************************************
//
//! Software clears DEACTIVE flag
//!
//! \param base is the base address of the EPWM module.
//! This function is used to clear DEACTIVE flag.
//! \return None.
//*****************************************************************************

static inline void
EPWM_clearDiodeEmulationActive(uint32_t base)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DECLR;

     HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) | CSL_EPWM_DECLR_DEACTIVE_MASK));

}
//*****************************************************************************
//
//! Enables DE monitor control mode
//!
//! \param base is the base address of the EPWM module.
//! This function is used to enable the DE monitor counter function.
//! \return None.
//*****************************************************************************


static inline void
EPWM_enableDiodeEmulationMonitorModeControl(uint32_t base)
{
   uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DEMONCTL;

    HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) |
        (CSL_EPWM_DEMONCTL_ENABLE_MAX << CSL_EPWM_DEMONCTL_ENABLE_SHIFT)));
}

//*****************************************************************************
//
//! Disables DE monitor control mode
//!
//! \param base is the base address of the EPWM module.
//! This function is used to disable the DE monitor counter function.
//! \return None.
//*****************************************************************************

static inline void
EPWM_disableDiodeEmulationMonitorModeControl(uint32_t base)
{
   uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DEMONCTL;

    HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) &
        ~(CSL_EPWM_DEMONCTL_ENABLE_MAX << CSL_EPWM_DEMONCTL_ENABLE_SHIFT)));
}

//*****************************************************************************
//
//! Sets the DE monitor counter step size
//!
//! \param base is the base address of the EPWM module.
//! \param direction is the direction of the monitor counter counting i.e. UP or DOWN.
//! \param stepsize is the amount by which the counter increments or decrements at each step.
//! This function is used to set the DE monitor counter step size.
//! Valid values for direction are:
//!    EPWM_DE_COUNT_UP     - Diode emulation count up step size
//!    EPWM_DE_COUNT_DOWN   - Diode emulation count down step size
//! Max value of stepsize is 256.
//! \return None.
//*****************************************************************************

static inline void
EPWM_setDiodeEmulationMonitorModeStep(uint32_t base,uint32_t direction,
                                   uint8_t stepsize)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DEMONSTEP;

    if(direction == EPWM_DE_COUNT_UP)
    {
        HW_WR_REG32(registerOffset,
        (HW_RD_REG32(registerOffset) & ~CSL_EPWM_DEMONSTEP_INCSTEP_MASK)
        | ((uint32_t)stepsize<<CSL_EPWM_DEMONSTEP_INCSTEP_SHIFT));
    }
    else if(direction == EPWM_DE_COUNT_DOWN)
    {
        HW_WR_REG32(registerOffset,
            ((HW_RD_REG32(registerOffset) &
            ~CSL_EPWM_DEMONSTEP_DECSTEP_MASK) |
            ((uint32_t)stepsize<<CSL_EPWM_DEMONSTEP_DECSTEP_SHIFT)));
    }
    else
    {
        /* do nothing */
    }
}

//*****************************************************************************
//
//! Sets the DE monitor counter threshold value.
//!
//! \param base is the base address of the EPWM module.
//! \param threshold is the max value till which the monitor counter can count.
//! This function is used to set the DE monitor counter threshold.
//! Max value of threshold is 2^16.
//! \return None.
//*****************************************************************************
static inline void
EPWM_setDiodeEmulationMonitorCounterThreshold(uint32_t base,uint16_t threshold)
{
    uint32_t registerOffset;
    //
    // Get the register offset
    //
    registerOffset =  base + CSL_EPWM_DEMONTHRES;

     HW_WR_REG32(registerOffset,
        ((HW_RD_REG32(registerOffset) & ~(CSL_EPWM_DEMONTHRES_THRESHOLD_MASK))
        | ((uint32_t)threshold<<CSL_EPWM_DEMONTHRES_THRESHOLD_SHIFT)) );
}


//*****************************************************************************
//
//! Set emulation mode
//!
//! \param base is the base address of the EPWM module.
//! \param emulationMode is the emulation mode.
//!
//! This function sets the emulation behaviours of the time base counter. Valid
//! values for emulationMode are:
//!  - EPWM_EMULATION_STOP_AFTER_NEXT_TB    - Stop after next Time Base counter
//!                                           increment or decrement.
//!  - EPWM_EMULATION_STOP_AFTER_FULL_CYCLE - Stop when counter completes whole
//!                                           cycle.
//!  - EPWM_EMULATION_FREE_RUN               - Free run.
//!
//! \return None.
//
//*****************************************************************************
extern void
EPWM_setEmulationMode(uint32_t base, EPWM_EmulationMode emulationMode);
//*****************************************************************************
//
//! Configures ePWM signal with desired frequency & duty
//!
//! \param base is the base address of the EPWM module.
//! \param signalParams is the desired signal parameters.
//!
//! This function configures the ePWM module to generate a signal with
//! desired frequency & duty.
//!
//! \return None.
//
//*****************************************************************************
extern void
EPWM_configureSignal(uint32_t base, const EPWM_SignalParams *signalParams);
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

#endif // EPWM_V1_H_

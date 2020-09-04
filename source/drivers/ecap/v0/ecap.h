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

/**
 *  \file v0/ecap.h
 *
 *  \brief      This file contains the function prototypes for the device
 *              abstraction layer for ECAP. It also contains some
 *              related macro definitions and some files to be included.
 */

#ifndef ECAP_V0_H_
#define ECAP_V0_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_ecap.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Ecap_OperMode_t
 *  \name ECAP Operating Mode
 *  @{
 */
/** \brief Capture Mode */
#define ECAP_CAPTURE_MODE               (0U)
/** \brief Auxiliary PWM Mode */
#define ECAP_APWM_MODE                  (1U)
/** @} */

/**
 *  \anchor Ecap_CaptEvt_t
 *  \name ECAP time stamp capture on event
 *  @{
 */
/** \brief Time stamp capture on event 1 */
#define ECAP_CAPTURE_EVENT_1            ((uint32_t)0x08U)
/** \brief Time stamp capture on event 2 */
#define ECAP_CAPTURE_EVENT_2            ((uint32_t)0x0cU)
/** \brief Time stamp capture on event 3 */
#define ECAP_CAPTURE_EVENT_3            ((uint32_t)0x10U)
/** \brief Time stamp capture on event 4 */
#define ECAP_CAPTURE_EVENT_4            ((uint32_t)0x14U)
/** @} */

/**
 *  \anchor Ecap_StopCaptEvt_t
 *  \name ECAP stop capture on event
 *  @{
 */
/** \brief Stop capture after event 1 */
#define ECAP_CAPTURE_EVENT1_STOP        ((uint32_t)0x00U)
/** \brief Stop capture after event 2 */
#define ECAP_CAPTURE_EVENT2_STOP        ((uint32_t)0x01U)
/** \brief Stop capture after event 3 */
#define ECAP_CAPTURE_EVENT3_STOP        ((uint32_t)0x02U)
/** \brief Stop capture after event 4 */
#define ECAP_CAPTURE_EVENT4_STOP        ((uint32_t)0x03U)
/** @} */

/**
 *  \anchor Ecap_APWMPolarityConfig_t
 *  \name ECAP APWM Output Polarity
 *  @{
 */
/** \brief Output Polarity HIGH */
#define ECAP_APWM_ACTIVE_HIGH           (0U)
/** \brief Output Polarity LOW */
#define ECAP_APWM_ACTIVE_LOW            (1U)
/** @} */

/**
 *  \anchor Ecap_CounterMode_t
 *  \name ECAP Counter Mode
 *  @{
 */
/** \brief Counter Mode Stop */
#define ECAP_COUNTER_STOP               (0U)
/** \brief Counter Mode Free Running */
#define ECAP_COUNTER_FREE_RUNNING       (1U)
/** @} */

/**
 *  \anchor Ecap_CounterSyncInMode_t
 *  \name ECAP Counter SyncIn Mode
 *  @{
 */
/** \brief Disable SyncIn option */
#define ECAP_SYNC_IN_DISABLE            ((uint32_t)0U)
/** \brief Enable Counter to be loaded from the ECAP_CNTPHS register upon
  *  SYNCI signal */
#define ECAP_ENABLE_COUNTER             ((uint32_t)1U)
/** @} */

/**
 *  \anchor Ecap_CounterSyncOutMode_t
 *  \name ECAP Counter SyncOut Mode
 *  @{
 */
/** \brief Select SyncIn event to be the SyncOut signal */
#define ECAP_SYNC_IN                    ((uint32_t)0x0U)
/** \brief Select PRD event to be the SyncOut signal */
#define ECAP_PRD_EQ                     ((uint32_t)0x1U)
/** \brief Disable SyncOut Signal */
#define ECAP_SYNC_OUT_DISABLE           ((uint32_t)0x2U)
/** @} */

/**
 *  \anchor Ecap_IntrSrc_t
 *  \name ECAP Interrupt Sources
 *  @{
 */
/** \brief Capture Event 1 Interrupt Enable */
#define ECAP_CEVT1_INT                  (ECAP_ECEINT_CEVT1)
/** \brief Capture Event 2 Interrupt Enable */
#define ECAP_CEVT2_INT                  (ECAP_ECEINT_CEVT2)
/** \brief Capture Event 3 Interrupt Enable */
#define ECAP_CEVT3_INT                  (ECAP_ECEINT_CEVT3)
/** \brief Capture Event 4 Interrupt Enable */
#define ECAP_CEVT4_INT                  (ECAP_ECEINT_CEVT4)
/** \brief Counter Overflow Interrupt Enable */
#define ECAP_CNTOVF_INT                 (ECAP_ECEINT_CTROVF)
/** \brief Period Equal Interrupt Enable */
#define ECAP_PRDEQ_INT                  (ECAP_ECEINT_CTR_PRD)
/** \brief Compare Equal Interrupt Enable */
#define ECAP_CMPEQ_INT                  (ECAP_ECEINT_CTR_CMP)
/** @} */

/**
 *  \anchor Ecap_GlobalIntrSrcClear_t
 *  \name ECAP Global Interrupt Source
 *  @{
 */
/** \brief Global Interrupt Source Clear Flag */
#define ECAP_GLOBAL_INT                 (ECAP_ECFLG_INT)
/** @} */

/**
 *  \anchor Ecap_CaptureEvtPolarityConfig_t
 *  \name ECAP Capture Event Polarity
 *  @{
 */
/** \brief Capture Event Rising Edge */
#define ECAP_CAPTURE_EVENT_RISING       (0U)
/** \brief Capture Event Falling Edge */
#define ECAP_CAPTURE_EVENT_FALLING      (1U)
/** @} */

/**
 *  \anchor Ecap_CaptureEvtCntrRstConfig_t
 *  \name ECAP Counter Reset On Capture Event
 *  @{
 */
/** \brief No counter reset upon Capture Event */
#define ECAP_CAPTURE_EVENT_RESET_COUNTER_NO_RESET     (0U)
/** \brief Counter reset upon Capture Event */
#define ECAP_CAPTURE_EVENT_RESET_COUNTER_RESET        (1U)
/** @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   This function enables capture loading.
 *
 * \param   baseAddr   It is the Memory address of the ECAP instance used.
 */
void ECAP_captureLoadingEnable(uint32_t baseAddr);

/**
 * \brief   This function disables capture loading
 *
 * \param   baseAddr   It is the Memory address of the ECAP instance used.
 */
void ECAP_captureLoadingDisable(uint32_t baseAddr);

/**
 * \brief   This function configures prescale value.
 *
 * \param   baseAddr  It is the Memory address of the ECAP instance used.
 * \param   prescale  It is the value which is used to prescale the incoming
 *                    input.
 *
 *          Prescale can take any integer value between 0 to 31
 */
void ECAP_prescaleConfig(uint32_t baseAddr, uint32_t prescale);

/**
 * \brief   This function configures ecapture module to operate in capture
 *          mode or in APWM mode.
 *
 * \param   baseAddr    It is the Memory address of the ECAP instance used.
 * \param   modeSelect  It is the value which determines whether ecapture
 *                      module to operate in capture mode or in APWM mode.\n
 *
 *         modeSelect can take one of the following macros.
 *         - \ref Ecap_OperMode_t.
 */
void ECAP_operatingModeSelect(uint32_t baseAddr, uint32_t modeSelect);

/**
 * \brief   This function returns time-stamp for a given capture event.
 *
 * \param   baseAddr    It is the Memory address of the ECAP instance used.
 * \param   capEvtFlag  It is the value which determines for which capture event
 *                      time-stam has to returned.
 *
 *         capEvtFlag can take one of the following macros.
 *         - \ref Ecap_CaptEvt_t.
 *
 * \return  Returns the time-stamp for given capure event.
 */
uint32_t ECAP_timeStampRead(uint32_t baseAddr, uint32_t capEvtFlag);

/**
 * \brief   This function configures the counter register which is used as
 *          Capture Time base.
 *
 * \param   baseAddr    It is the Memory address of the ECAP instance used.
 * \param   countVal    It is counter value to be configured.
 */
void ECAP_counterConfig(uint32_t baseAddr, uint32_t countVal);

/**
 * \brief   This function configures Capture Event polarity.
 *
 * \param   baseAddr    It is the Memory address of the ECAP instance used.
 * \param   capEvt1pol  It determines whether Capture Event1 has to be generated
 *                      on rising edge or falling edge of pulse.
 *
 * \param   capEvt2pol  It determines whether Capture Event2 has to be generated
 *                      on rising edge or falling edge of pulse.
 *
 * \param   capEvt3pol  It determines whether Capture Event3 has to be generated
 *                      on rising edge or falling edge of pulse.
 *
 * \param   capEvt4pol  It determines whether Capture Event4 has to be generated
 *                      on rising edge or falling edge of pulse.
 *
 *          capEvtpol variables can take one of the following macros.
 *          - \ref Ecap_CaptureEvtPolarityConfig_t.
 */
void ECAP_captureEvtPolarityConfig(uint32_t baseAddr, uint32_t capEvt1pol,
                               uint32_t capEvt2pol, uint32_t capEvt3pol,
                               uint32_t capEvt4pol);

/**
 * \brief   This function enables reset of the counters upon Capture Events.
 *
 * \param   baseAddr     It is the Memory address of the ECAP instance used.
 * \param   counterRst1  It determines whether counter has to be reset upon
 *                       Capture Event1.
 *
 * \param   counterRst2  It determines whether counter has to be reset upon
 *                       Capture Event2.
 *
 * \param   counterRst3  It determines whether counter has to be reset upon
 *                       Capture Event3.
 *
 * \param   counterRst4  It determines whether counter has to be reset upon
 *                       Capture Event4.
 *
 *          counterRst variables can take one of the following macros.
 *          - \ref Ecap_CaptureEvtCntrRstConfig_t.
 */
void ECAP_captureEvtCntrRstConfig(uint32_t baseAddr, uint32_t counterRst1,
                                 uint32_t counterRst2, uint32_t counterRst3,
                                 uint32_t counterRst4);

/**
 * \brief   This function configures ECAP to Continuous mode.
 *
 * This API is valid only if ECAP is configured to Capture Mode.It has
 * no significance when ECAP is configured in APWM mode.
 *
 * \param   baseAddr   It is the Memory address of the ECAP instance used.
 */
void ECAP_continousModeConfig(uint32_t baseAddr);

/**
 * \brief   This function configures ECAP to One-shot mode and also
 *          stop value for this mode.
 *
 * This API is valid only if ECAP is configured to Capture Mode.It has
 * no significance when ECAP is configured in APWM mode.
 *
 * \param   baseAddr  It is the Memory address of the ECAP instance used.
 *
 * \param   stopVal   It is the number of captures allowed to occur before
 *                    Capture register(1-4) are frozen.\n
 *
 *          stopVal can take one of the following macros.
 *          - \ref Ecap_StopCaptEvt_t.
 */
void ECAP_oneShotModeConfig(uint32_t baseAddr, uint32_t stopVal);

/**
 * \brief   This function configures ECAP to One-Short Re-arming.
 *
 * When this API is invoked following things happen.\n
 *
 * 1. Resets Mod4 counter to zero.\n
 * 2. Un-freezes the Mod4 counter.\n
 * 3. Enables capture register loads.\n
 *
 * \param   baseAddr   It is the Memory address of the ECAP instance used.
 */
void ECAP_oneShotReArm(uint32_t baseAddr);

/**
 * \brief   This function configures output polarity for APWM output.
 *
 * \param   baseAddr  It is the Memory address of the ECAP instance used.
 * \param   flag      It is the value which determines the output polarity
 *                    for APWM output.\n
 *
 *          flag can take one of the following macros.
 *          - \ref Ecap_StopCaptEvt_t.
 */
void ECAP_APWM_polarityConfig(uint32_t baseAddr, uint32_t flag);

/**
 * \brief   This function configures counter to stop or free running
 *          based on its input argument flag.
 *
 * \param   baseAddr  It is the Memory address of the ECAP instance used.
 * \param   flag      It is the value which determine counter to be configured
 *                    to stop or free running.\n
 *
 *          flag can take one of the following macros.
 *          - \ref Ecap_CounterMode_t.
 */
void ECAP_counterControl(uint32_t baseAddr, uint32_t flag);

/**
 * \brief   This function configures Sync-In and Sync-Out.
 *
 * \param   baseAddr  It is the Memory address of the ECAP instance used.
 * \param   syncIn    It is the value which determines whether to disable
 *                    syncIn or to enable counter to be loaded from
 *                    CNTPHS register upon a SYNCI signal.\n
 *
 *          syncIn can take one of the following macros.
 *          - \ref Ecap_CounterSyncInMode_t.
 *
 * \param   syncOut   It is the value which select type of syncOut signal
 *                    (i.e select syncIn event to be the Sync-Out signal,
 *                         select PRD_eq event to be Sync-Out signal).\n
 *
 *          syncOut can take one of the following macros.
 *          - \ref Ecap_CounterSyncInMode_t.
 */
void ECAP_syncInOutSelect(uint32_t baseAddr, uint32_t syncIn, uint32_t syncOut);

/**
 * \brief   When ECAP module is configured in APWM mode capture 1 and capture 2
 *          registers are used as period and compare register.This function
 *          configures compare and period values to this register.
 *
 * \param   baseAddr   It is the Memory address of the ECAP instance used.
 * \param   compareVal It is the Compare value to be configured.
 * \param   periodVal  It is the Period value to be configured.
 */
void ECAP_APWM_captureConfig(uint32_t baseAddr, uint32_t compareVal, uint32_t periodVal);

/**
 * \brief   This function configures the Shadow register.
 *
 * \param   baseAddr   It is the Memory address of the ECAP instance used.
 * \param   compareVal It is the Compare value to be configured.
 * \param   periodVal  It is the Period value to be configured.
 */
void ECAP_APWM_shadowCaptureConfig(uint32_t baseAddr, uint32_t compareVal, uint32_t periodVal);

/**
 * \brief   This function configures the counter phase value.
 *
 * \param   baseAddr     It is the Memory address of the ECAP instance used.
 * \param   cntPhaseVal  It is the counter phase value to be programmed for
 *                       phase lag/lead.
 */
void ECAP_counterPhaseValConfig(uint32_t baseAddr, uint32_t cntPhaseVal);

/**
 * \brief   This function clears global interrupt and enables the generation of
 *          interrupts if any of the event interrupt are enabled and
 *          corresponding event interrupt flag is set.
 *
 * \param   baseAddr  It is the Memory address of the ECAP instance used.
 */
void ECAP_globalIntrClear(uint32_t baseAddr);

/**
 * \brief   This function enables the specified interrupts
 *
 * \param   baseAddr  It is the Memory address of the ECAP instance used.
 * \param   flag      It is the value which specifies the interrupts to
 *                    be enabled.\n
 *
 *          flag can take one of the following macros.
 *          - \ref Ecap_IntrSrc_t.
 */
void ECAP_intrEnable(uint32_t baseAddr, uint32_t flag);

/**
 * \brief   This function disables the specified interrupts
 *
 * \param   baseAddr  It is the Memory address of the ECAP instance used.
 * \param   flag      It is the value which specifies the interrupts to
 *                    be disabled.\n
 *
 *          flag can take one of the following macros.
 *          - \ref Ecap_IntrSrc_t.
 */
void ECAP_intrDisable(uint32_t baseAddr, uint32_t flag);

/**
 * \brief   This function returns the status specified interrupts
 *
 * \param   baseAddr  It is the Memory address of the ECAP instance used.
 * \param   flag      It is the value which specifies the status of interrupts
 *                    to be returned.\n
 *
 *          flag can take one of the following macros.
 *          - \ref Ecap_IntrSrc_t.
 *
 * \returns Status of the specified interrupts.
 *
 */
uint32_t ECAP_getIntrStatus(uint32_t baseAddr, uint32_t flag);

/**
 * \brief   This function clears of the status specified interrupts
 *
 * \param   baseAddr  It is the Memory address of the ECAP instance used.
 * \param   flag      It is the value which specifies the status of interrupts
 *                    to be cleared.\n
 *
 *          flag can take one of the following macros.
 *          - \ref Ecap_IntrSrc_t.
 */
void ECAP_intrStatusClear(uint32_t baseAddr, uint32_t flag);

/**
 * \brief   This function returns the peripheral ID
 *
 * \param   baseAddr   It is the Memory address of the ECAP instance used.
 *
 * \return  Peripheral ID.
 *
 */
uint32_t ECAP_peripheralIdGet(uint32_t baseAddr);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef ECAP_V0_H_ */

/** @} */

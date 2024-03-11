/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
 *
 *****************************************************************************/

#ifndef PMIC_INC_PMIC_WDG_H_
#define PMIC_INC_PMIC_WDG_H_

/* ==========================================================================*/
/*                             Include Files                                 */
/* ==========================================================================*/

#include "pmic_core_priv.h"
#include "pmic_wdg_priv.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  @anchor Pmic_WdgEnDisableMode
 *  @name PMIC watchdog timer En/Disable Modes
 *
 *  @{
 */
#define PMIC_WDG_DISABLE                            (0x0U)
#define PMIC_WDG_ENABLE                             (0x1U)
/** @} */

/**
 *  @anchor Pmic_WdgResetEnDisable
 *  @name PMIC watchdog timer warm reset En/Disable
 *
 *  @{
 */
#define PMIC_WDG_RESET_DISABLE                      (0x0U)
#define PMIC_WDG_RESET_ENABLE                       (0x1U)
/** @} */

/**
 *  @anchor Pmic_WdgReturnLongWinEnDisable
 *  @name PMIC watchdog timer Return Long Window En/Disable
 *
 *  @{
 */
#define PMIC_WDG_RETLONGWIN_DISABLE                 (0x0U)
#define PMIC_WDG_RETLONGWIN_ENABLE                  (0x1U)
/** @} */

/**
 *  @anchor Pmic_WdgPwrHoldEnDisable
 *  @name PMIC watchdog timer Power Hold En/Disable
 *
 *  @{
 */
#define PMIC_WDG_PWRHOLD_DISABLE                    (0x0U)
#define PMIC_WDG_PWRHOLD_ENABLE                     (0x1U)
/** @} */

/**
 *  @anchor Pmic_WdgTriggerQAMode
 *  @name PMIC watchdog timer Trigger/QA Mode
 *
 *  @{
 */
#define PMIC_WDG_TRIGGER_MODE                       (0x0U)
#define PMIC_WDG_QA_MODE                            (0x1U)
/** @} */

/**
 *  @anchor Pmic_WdgResetThresholdCount
 *  @name PMIC watchdog timer Reset Threshold Configurations
 *
 *  @{
 */
#define PMIC_WDG_RESET_THRESHOLD_COUNT_0            (0x0U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_1            (0x1U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_2            (0x2U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_3            (0x3U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_4            (0x4U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_5            (0x5U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_6            (0x6U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_7            (0x7U)
/** @} */

/**
 *  @anchor Pmic_WdgFailThresholdCount
 *  @name PMIC watchdog timer Fail Threshold Configurations
 *
 *  @{
 */
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_0             (0x0U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_1             (0x1U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_2             (0x2U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_3             (0x3U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_4             (0x4U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_5             (0x5U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_6             (0x6U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_7             (0x7U)
/** @} */

/**
 *  @anchor Pmic_WdgQaFdbkVal
 *  @name PMIC watchdog timer QA Feedback Values
 *
 *  @{
 */
#define PMIC_WDG_QA_FEEDBACK_VALUE_0                (0x0U)
#define PMIC_WDG_QA_FEEDBACK_VALUE_1                (0x1U)
#define PMIC_WDG_QA_FEEDBACK_VALUE_2                (0x2U)
#define PMIC_WDG_QA_FEEDBACK_VALUE_3                (0x3U)
/** @} */

/**
 *  @anchor Pmic_WdgQaLfsrVal
 *  @name PMIC watchdog timer QA LFSR Values
 *
 *  @{
 */
#define PMIC_WDG_QA_LFSR_VALUE_0                    (0x0U)
#define PMIC_WDG_QA_LFSR_VALUE_1                    (0x1U)
#define PMIC_WDG_QA_LFSR_VALUE_2                    (0x2U)
#define PMIC_WDG_QA_LFSR_VALUE_3                    (0x3U)
/** @} */

/**
 *  @anchor Pmic_WdgQaQuestionSeedVal
 *  @name PMIC watchdog timer QA Question Seed Values
 *
 *  @{
 */
#define PMIC_WDG_QA_QUES_SEED_VALUE_0               (0x0U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_1               (0x1U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_2               (0x2U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_3               (0x3U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_4               (0x4U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_5               (0x5U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_6               (0x6U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_7               (0x7U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_8               (0x8U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_9               (0x9U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_10              (0xAU)
#define PMIC_WDG_QA_QUES_SEED_VALUE_11              (0xBU)
#define PMIC_WDG_QA_QUES_SEED_VALUE_12              (0xCU)
#define PMIC_WDG_QA_QUES_SEED_VALUE_13              (0xDU)
#define PMIC_WDG_QA_QUES_SEED_VALUE_14              (0xEU)
#define PMIC_WDG_QA_QUES_SEED_VALUE_15              (0xFU)
/** @} */

/**
 *  @anchor Pmic_WdgCntSel
 *  @name PMIC Watchdog Fail Counter Schemes
 *
 *  @{
 */
/** @brief +1/-1 counting scheme. WD_FAIL_CNT is incremented by one
 *          for a bad event and decremented by one for a good event
 */
#define PMIC_WDG_CNT_SEL_1_1_SCHEME                 (0x0U)
/** @brief +2/-1 counting scheme. WD_FAIL_CNT is incremented by two
 *          for a bad event and decremented by one for a good event
 */
#define PMIC_WDG_CNT_SEL_2_1_SCHEME                 (0x1U)
/** @} */

/**
 *  @anchor Pmic_WdgEnDrvSel
 *  @name PMIC Watchdog Effect on ENABLE_DRV
 *
 *  @{
 */
/** @brief ENABLE_DRV not cleared when WD_FAIL_INT is high
 */
#define PMIC_WDG_ENDRV_SEL_NO_CLR                   (0x0U)
/** @brief ENABLE_DRV cleared when WD_FAIL_INT is high
 */
#define PMIC_WDG_ENDRV_SEL_CLR                      (0x1U)
/** @} */

/**
 *  @anchor Pmic_WdgCfgValidParamBitPos
 *  @name PMIC watchdog timer Config Structure Param Bit positions
 *
 *  @{
 */
/** @brief validParams value used to set/get Long Window duration */
#define PMIC_CFG_WDG_LONGWINDURATION_VALID          (0U)
/** @brief validParams value used to set/get Window-1 duration */
#define PMIC_CFG_WDG_WIN1DURATION_VALID             (1U)
/** @brief validParams value used to set/get Window-2 duration */
#define PMIC_CFG_WDG_WIN2DURATION_VALID             (2U)
/** @brief validParams value used to set/get Fail threshold value */
#define PMIC_CFG_WDG_FAILTHRESHOLD_VALID            (3U)
/** @brief validParams value used to set/get Reset threshold Value */
#define PMIC_CFG_WDG_RSTTHRESHOLD_VALID             (4U)
/** @brief validParams value used to set/get to enable or diable warm reset on fail */
#define PMIC_CFG_WDG_RSTENABLE_VALID                (5U)
/** @brief validParams value used to set/get watchdog mode */
#define PMIC_CFG_WDG_WDGMODE_VALID                  (6U)
/** @brief validParams value used to set/get to Enable or disable  watchdog pwrHold */
#define PMIC_CFG_WDG_PWRHOLD_VALID                  (7U)
/** @brief validParams value used to set/get to enable or disable return to long window */
#define PMIC_CFG_WDG_RETLONGWIN_VALID               (8U)
/** @brief validParams value used to set/get QA feed back value */
#define PMIC_CFG_WDG_QA_FDBK_VALID                  (9U)
/** @brief validParams value used to set/get QA LFSR value */
#define PMIC_CFG_WDG_QA_LFSR_VALID                  (10U)
/** @brief validParams value used to set/get QA question seed value */
#define PMIC_CFG_WDG_QA_QUES_SEED_VALID             (11U)
/** @brief validParams value used to set/get fail counter configuration */
#define PMIC_CFG_WDG_CNT_SEL_VALID                  (12U)
/** @brief validParams value used to set/get Watchdog ENDRV configuration */
#define PMIC_CFG_WDG_ENDRV_SEL_VALID                (13U)
/** @} */

/**
 * @name PMIC Watchdog Min Wait Count
 * @brief Minimum number of iterations to wait for a Good/Bad event
 */
#define PMIC_WDG_WAIT_CNT_MIN_VAL                   (30U)

/**
 *  @anchor Pmic_WdgCfgValidParamBitShiftVal
 *  @name PMIC WatchDog Config Structure Params Bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  structure member defined in Pmic_WdgCfg_t structure
 *
 *  @{
 */
#define PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT    (1U << PMIC_CFG_WDG_LONGWINDURATION_VALID)
#define PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT       (1U << PMIC_CFG_WDG_WIN1DURATION_VALID)
#define PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT       (1U << PMIC_CFG_WDG_WIN2DURATION_VALID)
#define PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT      (1U << PMIC_CFG_WDG_FAILTHRESHOLD_VALID)
#define PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT       (1U << PMIC_CFG_WDG_RSTTHRESHOLD_VALID)
#define PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT          (1U << PMIC_CFG_WDG_RSTENABLE_VALID)
#define PMIC_CFG_WDG_WDGMODE_VALID_SHIFT            (1U << PMIC_CFG_WDG_WDGMODE_VALID)
#define PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT            (1U << PMIC_CFG_WDG_PWRHOLD_VALID)
#define PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT         (1U << PMIC_CFG_WDG_RETLONGWIN_VALID)
#define PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT            (1U << PMIC_CFG_WDG_QA_FDBK_VALID)
#define PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT            (1U << PMIC_CFG_WDG_QA_LFSR_VALID)
#define PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT       (1U << PMIC_CFG_WDG_QA_QUES_SEED_VALID)
#define PMIC_CFG_WDG_CNT_SEL_VALID_SHIFT            (1U << PMIC_CFG_WDG_CNT_SEL_VALID)
#define PMIC_CFG_WDG_ENDRV_SEL_VALID_SHIFT          (1U << PMIC_CFG_WDG_ENDRV_SEL_VALID)
/** @} */

/**
 *  @anchor Pmic_WdgErrStatCfgValidParamBitPos
 *  @name PMIC watchdog timer error status Structure Param Bit positions.
 *
 *  @{
 */
/** @brief validParams value used to get Long Window timeout error status */
#define PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID   (0U)
/** @brief validParams value used to get Window1 and window2 timeout error
 *         status */
#define PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID           (1U)
/** @brief validParams value used to get Watchdog trigger mode error status */
#define PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID        (2U)
/** @brief validParams value used to get Watchdog early answer error status */
#define PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID        (3U)
/** @brief validParams value used to get Watchdog QA sequence error status */
#define PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID           (4U)
/** @brief validParams value used to get Watchdog QA wrong Answer error status
 */
#define PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID          (5U)
/** @brief validParams value used to get Watchdog fail error status */
#define PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID          (6U)
/** @brief validParams value used to get Watchdog reset error status */
#define PMIC_CFG_WD_RST_INT_ERRSTAT_VALID           (7U)
/** @} */

/**
 *  @anchor Pmic_WdgFailCntStatCfgValidParamBitPos
 *  @name PMIC watchdog Fail count status Structure Param Bit positions.
 *
 *  @{
 */
/** @brief validParams value used to get status of Bad Event is detected or not
 */
#define PMIC_CFG_WD_BAD_EVENT_STAT_VALID            (0U)
/** @brief validParams value used to get status of Good Event is detected or not */
#define PMIC_CFG_WD_GOOD_EVENT_STAT_VALID           (1U)
/** @brief validParams value used to get To get Watchdog Fail Count value */
#define PMIC_CFG_WD_FAIL_CNT_VAL_VALID              (2U)
/** @} */

/**
 *  @anchor Pmic_WdgErrStatValidParamBitShiftVal
 *  @name PMIC WatchDog Error status Structure Params Bit shift values
 *
 *  Application can use below shifted values to set the validParams
 *  structure member defined in Pmic_WdgErrStatus_t structure
 *
 *  @{
 */
#define PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID_SHIFT                        \
  (1U << PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID)
#define PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID_SHIFT                                \
  (1U << PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID)
#define PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID_SHIFT                             \
  (1U << PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID)
#define PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID_SHIFT                             \
  (1U << PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID)
#define PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID_SHIFT                                \
  (1U << PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID)
#define PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID_SHIFT                               \
  (1U << PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID)
#define PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID_SHIFT                               \
  (1U << PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID)
#define PMIC_CFG_WD_RST_INT_ERRSTAT_VALID_SHIFT                                \
  (1U << PMIC_CFG_WD_RST_INT_ERRSTAT_VALID)
/** @} */

/**
 *  @anchor Pmic_WdgFailCntStatValidParamBitShiftVal
 *  @name PMIC WatchDog Fail count status Structure Params Bit shift values
 *
 *  Application can use below shifted values to set the validParams
 *  structure member defined in Pmic_WdgErrStatus_t structure
 *
 *  @{
 */
#define PMIC_CFG_WD_BAD_EVENT_STAT_VALID_SHIFT                                 \
  (1U << PMIC_CFG_WD_BAD_EVENT_STAT_VALID)
#define PMIC_CFG_WD_GOOD_EVENT_STAT_VALID_SHIFT                                \
  (1U << PMIC_CFG_WD_GOOD_EVENT_STAT_VALID)
#define PMIC_CFG_WD_FAIL_CNT_VAL_VALID_SHIFT                                   \
  (1U << PMIC_CFG_WD_FAIL_CNT_VAL_VALID)

/** @} */

/**
 * @name  Macro for PMIC Watchdog QA infinite sequence
 */
#define PMIC_WD_QA_INFINITE_SEQ                     (0xFFFFFFFFU)

/**
 *  @anchor Pmic_WdgErrType
 *  @name PMIC WDG Error TYPE
 *
 *  @{
 */
#define PMIC_WDG_ERR_TIMEOUT                        (0x0U)
#define PMIC_WDG_ERR_TRIGGER_EARLY                  (0x1U)
#define PMIC_WDG_ERR_ANSWER_EARLY                   (0x2U)
#define PMIC_WDG_ERR_SEQ_ERR                        (0x3U)
#define PMIC_WDG_ERR_ANS_ERR                        (0x4U)
#define PMIC_WDG_ERR_LONG_WIN_TIMEOUT               (0x5U)
#define PMIC_WDG_ERR_THRES1                         (0x6U)
#define PMIC_WDG_ERR_THRES2                         (0x7U)
#define PMIC_WDG_ERR_ALL                            (0x8U)
/** @} */

/*!
 * @brief WD_ERR_STATUS Register bit mask values
 */
#define PMIC_WDG_ERR_TIMEOUT_MASK                   (0x1U << PMIC_WDG_ERR_TIMEOUT_SHIFT)
#define PMIC_WDG_ERR_TRIGGER_EARLY_MASK             (0x1U << PMIC_WDG_ERR_TRIGGER_EARLY_SHIFT)
#define PMIC_WDG_ERR_ANSWER_EARLY_MASK              (0x1U << PMIC_WDG_ERR_ANSWER_EARLY_SHIFT)
#define PMIC_WDG_ERR_SEQ_ERR_MASK                   (0x1U << PMIC_WDG_ERR_SEQ_ERR_SHIFT)
#define PMIC_WDG_ERR_ANS_ERR_MASK                   (0x1U << PMIC_WDG_ERR_ANS_ERR_SHIFT)
#define PMIC_WDG_ERR_LONGWIN_TIMEOUT_MASK           (0x1U << PMIC_WDG_ERR_LONGWIN_TIMEOUT_SHIFT)
#define PMIC_WDG_ERR_THRES1_MASK                    (0x1U << PMIC_WDG_ERR_THRES1_SHIFT)
#define PMIC_WDG_ERR_THRES2_MASK                    (0x1U << PMIC_WDG_ERR_THRES2_SHIFT)

/* ========================================================================== */
/*                            Structures and Enums                            */
/* ========================================================================== */

/**
 * @name                        PMIC Watchdog Configuration Structure
 * @brief                       This structure is used in setting or getting
 *                              the Watchdog configurations of supported PMICs
 *                              (TPS6522x, TPS6594x, LP8764x).
 *
 * @note                        ValidParams is input param for all Set and Get
 *                              APIs. Other params except validParams are input
 *                              params for Set APIs and output params for Get
 * APIs.
 *
 * @param   validParams         Selection of structure parameters to be
 *                              set from the combination of the
 *                              \ref Pmic_WdgCfgValidParamBitPos
 *                              and the corresponding member value will be
 *                              updated.
 * @param   longWinDuration_ms  Long Window duration in milli seconds.
 *                              To get more effective results user has to
 *                              program long window with multiples of 3000.
 *                              For PG1.0 Leo/Hera, the valid range is (100,
 *                              3000, 6000, 9000,....12000, ..., 765000).
 *                              For PG2.0 and Burton, the valid range is
 *                              (80, 125, 250, 375,....8000, 12000, 16000,
 *                              20000 ..., 772000).
 * @param   win1Duration_us     Window-1 duration in Micro Seconds.
 *                              To get more effective results user has to
 *                              program window1 with multiples of 550.
 *                              The valid range is (550, 1100, 1650, 2200,
 *                                      2750, ..., 70400).
 * @param   win2Duration_us     Window-2 duration in Micro Seconds.
 *                              To get more effective results user has to
 *                              program window1 with multiples of 550.
 *                              The valid range is (550, 1100, 1650, 2200,
 *                                      2750, ..., 70400).
 * @param   failThreshold       Fail threshold value.
 *                              For valid Values:
 *                                   \ref Pmic_WdgFailThresholdCount.
 * @param   rstThreshold        Reset threshold Value.
 *                              For valid Values:
 *                                   \ref Pmic_WdgResetThresholdCount.
 * @param   wdgMode             Value to set watchdog mode.
 *                              For valid Values:
 *                                   \ref Pmic_WdgTriggerQAMode.
 * @param   pwrHold             Value to Enable or disable  watchdog pwrHold.
 *                              For valid Values:
 *                                   \ref Pmic_WdgPwrHoldEnDisable.
 * @param   rstEnable           To enable or diable warm reset on fail.
 *                              For valid Values:
 *                                   \ref Pmic_WdgResetEnDisable.
 * @param   retLongWin          To enable or disable return to long window
 *                              after completion of the curent sequence.
 *                              For valid Values:
 *                                   \ref Pmic_WdgReturnLongWinEnDisable.
 * @param   cntSel              Configure counting scheme of the watchdog
 *                              fail counter. For valid Values:
 *                                   \ref Pmic_WdgCntSel
 * @param   enDrvSel            Configure Watchdog effect on ENABLE_DRV.
 *                              For valid values:
 *                                   \ref Pmic_WdgEnDrvSel
 * @param   qaFdbk              Configure QA feed back value.
 *                              For valid Values:
 *                                   \ref Pmic_WdgQaFdbkVal.
 * @param   qaLfsr              Configure QA LFSR value.
 *                              For valid Values:
 *                                   \ref Pmic_WdgQaLfsrVal.
 * @param   qaQuesSeed          Configure QA question seed value.
 *                              For valid Values:
 *                                   \ref Pmic_WdgQaQuestionSeedVal.
 *
 * @{
 */
typedef struct Pmic_WdgCfg_s {
    uint32_t validParams;

    uint32_t longWinDuration_ms;
    uint32_t win1Duration_us;
    uint32_t win2Duration_us;

    uint8_t failThreshold;
    uint8_t rstThreshold;

    uint8_t wdgMode;
    uint8_t pwrHold;
    uint8_t rstEnable;
    uint8_t retLongWin;
    uint8_t cntSel;
    uint8_t enDrvSel;

    uint8_t qaFdbk;
    uint8_t qaLfsr;
    uint8_t qaQuesSeed;

}
Pmic_WdgCfg_t;
/** @} */

/**
 * @name                        PMIC Watchdog Error Status Structure
 * @brief                       This struct is used to get the Watchdog error
 * statuses of supported PMICs (TPS6522x, TPS6594x, LP8764x).
 *
 * @note                        ValidParams is input param for all Get APIs.
 * Other params except validParams are output params for Get APIs.
 *
 * @param   validParams         Selection of structure parameters to be
 *                              set from the combination of the
 *                              \ref Pmic_WdgErrStatCfgValidParamBitPos
 *                              and the corresponding member value will be
 *                              updated.
 * @param   wdLongWinTimeout    To get Long Window timeout error status.
 * @param   wdTimeout           To get Window1 and window2 timeout error status.
 * @param   wdTrigEarly         To get Watchdog trigger mode error status.
 * @param   wdAnswearly         To get Watchdog early answer error status.
 * @param   wdSeqErr            To get Watchdog QA sequence error status.
 * @param   wdAnswErr           To get Watchdog QA wrong Answer error status.
 * @param   wdFailInt           To get Watchdog fail error status.
 * @param   wdRstInt            To get Watchdog reset error status.
 *
 * @{
 */
typedef struct Pmic_WdgErrStatus_s {
    uint32_t validParams;
    bool wdLongWinTimeout;
    bool wdTimeout;
    bool wdTrigEarly;
    bool wdAnswearly;
    bool wdSeqErr;
    bool wdAnswErr;
    bool wdFailInt;
    bool wdRstInt;
}
Pmic_WdgErrStatus_t;
/** @} */

/**
 * @name                        PMIC Watchdog Fail Count Status Structure
 * @brief                       This struct is used to get the Watchdog bad/good
 * event and fail count of supported PMICs (TPS6522x, TPS6594x, LP8764x).
 *
 * @note                        ValidParams is input param for all Get APIs.
 * Other params except validParams are output params for Get APIs.
 *
 * @param   validParams         Selection of structure parameters to be
 *                              set from the combination of the
 *                              \ref Pmic_WdgFailCntStatCfgValidParamBitPos
 *                              and the corresponding member value will be
 *                              updated.
 * @param   wdBadEvent          To get status of Bad Event is detected or not
 * @param   wdGudEvent          To get status of Good Event is detected or not
 * @param   wdFailCnt           To get Watchdog Fail Count value.
 *
 * @{
 */
typedef struct Pmic_WdgFailCntStat_s {
    uint32_t validParams;
    bool wdBadEvent;
    bool wdGudEvent;
    uint32_t wdFailCnt;
}
Pmic_WdgFailCntStat_t;
/** @} */

/* ========================================================================== */
/*                            Function Declarations                           */
/* ========================================================================== */
/**
 * @brief   API to Enable Watchdog timer.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to enable the PMIC watchdog. User needs to
 *          ensure that this function is called to enable watchdog timer before
 *          configuring or starting watchdog trigger or QA mode.
 *
 * @param   pPmicCoreHandle [IN]    PMIC Interface Handle
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgEnable(Pmic_CoreHandle_t * pPmicCoreHandle);

/**
 * @brief   API to Disable Watchdog timer.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to disable the PMIC watchdog. User needs to
 *          ensure that after using this function, complete watchdog
 * functionality and configuration will be deactivated.
 *
 * @param   pPmicCoreHandle [IN]    PMIC Interface Handle
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgDisable(Pmic_CoreHandle_t * pPmicCoreHandle);

/**
 *  @brief      This function is used to get the Watchdog Enable state (that is
 * to say, whether WD_EN bit is set to 1 or 0).
 *
 *  @param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  @param      pWdgEnabled         [OUT]       Pointer to Watchdog Enable
 *
 *  @return     Success code if Watchdog Enable state is obtained, error code
 * otherwise. For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgGetEnableState(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t *pWdgEnabled);

/**
 * @brief   API to set PMIC watchdog configurations.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854), REQ_TAG(PDK-9115),
 *              REQ_TAG(PDK-9116)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to configure the watchdog parameters
 *          in the PMIC for trigger mode or Q&A (question and answer) mode,
 *          when corresponding validParam bit fields are set in
 *          Pmic_WdgCfg_t structure.
 *          User has to call Pmic_wdgEnable() before setting the configuration.
 *
 * @param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * @param   wdgCfg          [IN]    Watchdog configuration
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgSetCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg);

/**
 * @brief   API to get PMIC watchdog configurations.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854), REQ_TAG(PDK-9115),
 *              REQ_TAG(PDK-9116)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to get configuration of the watchdog
 *          from the PMIC for trigger mode or Q&A (question and answer) mode,
 *          when corresponding validParam bit fields are set in
 *          Pmic_WdgCfg_t structure.
 *          User has to call Pmic_wdgEnable() before getting the configuration.
 *
 * @param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * @param   pWdgCfg         [IN/OUT]   Watchdog configuration pointer
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgGetCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgCfg_t * pWdgCfg);

/**
 * @brief   API to Start watchdog QA mode.
 *
 * Requirement: REQ_TAG(PDK-5839)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to start watchdog sequence and continues
 *          until the given num_of_sequences. User has to ensure proper
 *          configuration of all Watchdog QA parameters using Pmic_wdgSetCfg()
 *          API, before starting QA sequence using this API.
 *
 *          Note: To perform QA sequences, user has to adjust Long window
 *                time interval, Window1 time interval and Window2 time
 *                intervals depends on errors given by API. If user gets
 *                PMIC_ST_ERR_INV_WDG_WINDOW, then user has to increase the
 *                Long window or window1 time interval. If user gets
 *                PMIC_ST_ERR_WDG_EARLY_ANSWER, then user has to reduce
 *                the Window1 time interval.
 *                Application has to ensure to do proper configuration of WDG
 *                window time intervals. If not configured properly then WDG
 *                will trigger the warm reset to the PMIC device. This may cause
 *                system reset if PMIC is connected to SOC/MCU.
 *                Application has to ensure to do proper configuration of WDG
 *                parameters. If not configured properly then API doesn't
 *                receive good or bad event from the PMIC FSM. Due to this API
 *                returns timeout error.
 *                API receive bad event due to wrong answer then API detects and
 *                returns an error.
 *
 * @param   pPmicCoreHandle  [IN]    PMIC Interface Handle
 * @param   num_of_sequences [IN]    number of QA sequences
 *                                   If PMIC_WD_QA_INFINITE_SEQ is used,
 *                                   then API runs for infinite sequence.
 * @param   maxCnt           [IN]    Number of iterations to wait for an
 *                                   Good/Bad event. The value should be greater
 *                                   than or equal to PMIC_WDG_WAIT_CNT_MIN_VAL.
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgStartQaSequence(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint32_t num_of_sequences, uint32_t maxCnt);

/**
 * @brief   API to get PMIC watchdog error status.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to get the watchdog error status from the
 *          PMIC for trigger mode or Q&A(question and answer) mode,
 *          when corresponding validParam bit fields are set in
 *          Pmic_WdgErrStatus_t structure.
 *          User has to call Pmic_wdgEnable() before getting the error status.
 *
 * @param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * @param   pErrStatus      [IN/OUT]   Watchdog error status pointer
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgGetErrorStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgErrStatus_t * pErrStatus);

/**
 * @brief   API to get PMIC watchdog fail count status.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to get the watchdog fail count status from the
 *          PMIC for trigger mode or Q&A(question and answer) mode.
 *          User has to call Pmic_wdgEnable() before getting the fail count.
 *
 * @param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * @param   pFailCount      [IN/OUT]   Watchdog fail count pointer
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgGetFailCntStat(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgFailCntStat_t * pFailCount);

/**
 * @brief   API to Start watchdog Trigger mode.
 *
 * Requirement: REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to start watchdog trigger mode.
 *          User has to ensure, configure all Watchdog trigger parameters
 *          properly using Pmic_wdgSetCfg() API, before starting watchdog
 *          trigger mode using this API. User can use Pmic_wdgSetCfg() API
 *          to stop watchdog trigger mode.
 *
 *          Note: To perform watchdog trigger mode, user has to
 *                adjust Long window time interval, Window1 time interval
 *                and Window2 time inervals as below, depends on the
 *                time-period of the trigger pulse provided by other
 *                device.
 *                1. Longwindow time interval must be greater than Trigger
 *                   pulse time period.
 *                2. Window1 time interval must be less than T-off time of
 *                   the Trigger pulse time period.
 *                3. Window2 time interval must be greater than T-on time
 *                   of the Trigger pulse time period.
 *                4. (Window1 time interval + Window2 time interval)
 *                   approximately equal to the Trigger pulse time period.
 *                Application has to ensure to do proper configuration of WDG
 *                window time intervals. If not configured properly in Trigger
 *                mode then WDG will trigger the warm reset to the PMIC device.
 *                This may cause system reset if PMIC is connected to SOC/MCU
 *
 * @param   pPmicCoreHandle  [IN]    PMIC Interface Handle
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgStartTriggerSequence(Pmic_CoreHandle_t * pPmicCoreHandle);

/**
 * @brief   API to clear PMIC watchdog error status.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to clear the watchdog error status from the
 *          PMIC for trigger mode or Q&A(question and answer) mode,
 *          Note: User has to clear the WDG Error status only when Error status
 *          bit is set for the corresponding wdgErrType
 *
 * @param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * @param   wdgErrType      [IN]       Watchdog error type to clear the status
 *                                     For Valid values:
 *                                     \ref Pmic_WdgErrType
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgClrErrStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t wdgErrType);

/**
 * @brief   API to Write Answers in Long Window/ Window1/ Window2 Interval for
 *          watchdog QA Sequence.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-9115), REQ_TAG(PDK-9116)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to write Answers in Long Window/ Window1/
 *          Window2 Interval for the WDG QA Sequence
 *          User has to ensure, configure all Watchdog QA parameters properly
 *          using Pmic_wdgSetCfg() API, before writing Answers using this API
 *          for the QA Sequence
 *
 *          Note: To perform QA sequences, user has to adjust Long window
 *                time interval, Window1 time interval and Window2 time
 *                intervals If the Pmic_wdgQaWriteAnswer API returns
 *                PMIC_ST_ERR_INV_WDG_ANSWER error
 *                If the Pmic_wdgQaWriteAnswer API returns
 *                PMIC_ST_ERR_INV_WDG_ANSWER error user has
 *                to call Pmic_wdgGetErrorStatus API to read the WDG error.
 *                If the WDG error is Long Window Timeout or Timeout, user has
 *                to increase the Long window or window1 time interval
 *                accordingly
 *                If the WDG error is Answer early, user has to reduce the
 *                Window1 time interval
 *                For other WDG errors, user has to take action accordingly
 *                Application has to ensure to do proper configuration of WDG
 *                window time intervals. If not configured properly in QA mode
 *                then WDG will trigger the warm reset to the PMIC device. This
 *                may cause system reset if PMIC is connected to SOC/MCU
 *
 * @param   pPmicCoreHandle  [IN]    PMIC Interface Handle
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgQaSequenceWriteAnswer(Pmic_CoreHandle_t * pPmicCoreHandle);

/**
 *  @brief      This API is used to initiate Watchdog (WDG) sequences - either
 *              Trigger or Q&A sequences - for TPS6522x and other PMICs that
 * have the same Watchdog IP block.
 *
 *  \details    The API clears any previous WDG errors and enables the WDG to
 *              receive triggers or answers from the MCU depending on the value
 *              of the \p wdgMode parameter.
 *
 *              This API is meant to be called before sending any MCU triggers
 *              or answers to the PMIC and just after configuration of the WDG
 *              using the Pmic_wdgSetCfg() API.
 *
 *              Once this API is called, depending on the WDG mode of operation,
 *              a trigger or four answer bytes must be sent shortly afterwards
 * to exit Long Window.
 *
 *              Once Long Window is exited, Watchdog sequences will begin. For
 * every Trigger mode sequence, a trigger must be sent in Window-2. For every
 *              Q&A mode sequence, end-user must call
 * Pmic_wdgQaSequenceWriteAnswer() three times in Window-1 and call the API
 * again one time during Window-2.
 *
 *              If end-user wants to end WDG sequences, the user should
 * configure the Watchdog to return to Long Window via Pmic_wdgSetCfg(). Once
 * return to Long Window is enabled, Watchdog enters Long Window at the end of
 * the next Window. User could set WD_PWRHOLD to stay in Long Window.
 *
 *  @note       There are three prerequisites that must be met for this API.
 *              1. <b> Implementation of the triggers are left to the end-user
 * to help enable the driver to be MCU-agnostic and allow flexibility for the
 * end-user. </b> The triggers can be a PWM or a periodic GPIO pulse to
 * TRIG_WDOG (configurable function of GPIO2 pin for TPS6522x). This
 * prerequisite is only applicable when the desired WDG mode of operation is
 * Trigger mode.
 *              2. <b> PMIC Watchdog must be configured appropriately with
 * respect to end-user's application before calling this API by using
 *                 Pmic_wdgSetCfg(). </b> If the Watchdog is not configured
 *                 appropriately, there may be errors that occur. For example,
 * if WDG is operating in Trigger mode and the Long Window duration is shorter
 * than the trigger pulse duration, there will be a Long Window timeout error.
 * As another example, if  Window-1 and/or Window-2 durations are too short or
 * too long, there could be a timeout error, early trigger error (Trigger mode
 * WDG), or early answer error (Q&A mode WDG). For list of configurable Watchdog
 *                 options, refer to Pmic_WdgCfg_s struct and its members.
 *              3. <b> This API must be called during Long Window so that the
 * API can configure the Watchdog appropriately for Trigger mode or Q&A mode.
 * </b> The PMIC should already be in Long Window when power is first supplied
 * to the PMIC. However, the end-user could return to Long Window by using
 * Pmic_wdgSetCfg().
 *
 *  @param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  @param      wdgMode             [IN]    Desired WDG mode during the
 * sequences. For valid values, refer to \ref Pmic_WdgTriggerQAMode
 *
 *  @return     Success code if WDG trigger sequences have been initiated and
 * PMIC WDG is ready to receive triggers from MCU, error code otherwise. For
 * valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgBeginSequences(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t wdgMde);

static int32_t
Pmic_checkPmicCoreHandle(const Pmic_CoreHandle_t * pPmicCoreHandle);

static int32_t
Pmic_WdgValidatePmicCoreHandle(const Pmic_CoreHandle_t * pPmicCoreHandle);

static uint8_t
Pmic_WdgCovertLongWinTimeIntervaltoRegBits(const Pmic_WdgCfg_t wdgCfg);

static int32_t
Pmic_WdgSetWindow1Window2TimeIntervals(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg);

static int32_t
Pmic_WdgSetWindowsTimeIntervals(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg);

static int32_t
Pmic_WdgGetWindow1Window2TimeIntervals(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgCfg_t * pWdgCfg);

static int32_t
Pmic_WdgGetWindowsTimeIntervals(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgCfg_t * pWdgCfg);

static int32_t Pmic_WdgSetThresholdValues(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg);

static int32_t Pmic_WdgGetThresholdValues(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgCfg_t * pWdgCfg);

static int32_t Pmic_WdgSetRetToLongWindowCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
                                             uint8_t returnLongWindow);

static int32_t Pmic_WdgSetWarmRstEnableCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
                                           uint8_t rstEnble);

static int32_t Pmic_WdgSetPwrHoldCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
                                     uint8_t pwrHld);

static int32_t Pmic_WdgSetModeCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
                                  uint8_t wdgMde);

int32_t Pmic_WdgSetCntSelCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t wdgCntSel);

int32_t Pmic_WdgSetEnDrvSelCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t wdgEnDrvSel);

static int32_t Pmic_WdgSetCtrlParams(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg);

static int32_t Pmic_WdgGetCtrlParams(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgCfg_t * pWdgCfg);

static int32_t Pmic_wdgSetQaQuesSeedValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg);

static int32_t Pmic_WdgSetQaConfigurations(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg);

static int32_t Pmic_WdgGetQaConfigurations(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgCfg_t * pWdgCfg);

static int32_t Pmic_wdgEnDisState(Pmic_CoreHandle_t * pPmicCoreHandle,
                                  uint8_t enable);

static int32_t
Pmic_wdgReadQuesandAnswerCount(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * pQaAnsCnt, uint8_t * pQaQuesCnt);

static bool is_wdgBadEventDetected(Pmic_CoreHandle_t * pPmicCoreHandle);

static uint8_t mux_4x1(uint8_t x0, uint8_t x1, uint8_t x2, uint8_t x3,
    uint8_t qaFdk);

static uint8_t Pmic_getAnswerByte(uint8_t qaQuesCnt, uint8_t qaAnsCnt,
    uint8_t qaFbk);

static int32_t
Pmic_wdgQaEvaluateAndWriteAnswer(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t qaAnsCnt, uint8_t qaQuesCnt,
    uint8_t qaFbk);

static int32_t
Pmic_wdgQaEvaluateAndWriteAnswers(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t qaFbk);

static int32_t Pmic_wdgQaWriteAnswers(Pmic_CoreHandle_t * pPmicCoreHandle);

static int32_t Pmic_wdgQaSetModeRetlongwinCfgWriteAnswersLongwindow(
    Pmic_CoreHandle_t * pPmicCoreHandle);

static int32_t
Pmic_wdgQaWriteAnswersNumSequence(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint32_t sequences, uint32_t maxCnt);

static void Pmic_wdgGetSeqAnswErrFailRstIntStat(Pmic_WdgErrStatus_t * pErrStatus,
    uint8_t regVal);

static void Pmic_wdgGetLongwintointTimeoutTrigAnswEarlyErrStat(
    Pmic_WdgErrStatus_t * pErrStatus, uint8_t regVal);

static int32_t
Pmic_wdgClrErrStatusWdgErrType(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t wdgErrType, uint8_t regVal);

#endif /* PMIC_INC_PMIC_WDG_H_ */

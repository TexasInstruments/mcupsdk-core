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

/**
 *   @file    pmic_fsm_priv.h
 *
 *   @brief   This file contains the private MACRO's and function definitions
 * for PMIC FSM state configuration
 *
 */

#ifndef PMIC_INC_PMIC_FSM_PRIV_H_
#define PMIC_INC_PMIC_FSM_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Pmic_FSM PMIC Finite State Machine
 * @{
 * @brief Contains definitions related to PMIC FSM functionality.
 */

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @defgroup Pmic_FSMPrivMacros PMIC Finite State Machine Private Macros
 * @{
 * @ingroup Pmic_FSM
 * @brief Contains private macros used in the FSM module of PMIC driver.
 */

/**
 * @brief Maximum number of FSM states.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STATE_MAX 		(6)

/**
 * @brief Shift value for PMIC state control.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_CTRL_SHIFT 	(0x0U)

/**
 * @brief Mask value for PMIC state control.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_CTRL_MASK 	(0x07U)

/**
 * @brief No change state control value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_CTRL_NO_CHANGE 	(0x00U)

/**
 * @brief Safe to active state control value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_CTRL_SAFE_TO_ACTIVE 	(0x01U)

/**
 * @brief Active to safe state control value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_CTRL_ACTIVE_TO_SAFE 	(0x02U)

/**
 * @brief Reset MCU state control value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_CTRL_RESET_MCU (0x03U)

/**
 * @brief Standby state control value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_CTRL_STANDBY (0x04U)

/**
 * @brief Off state control value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_CTRL_OFF (0x05U)

/**
 * @brief Mask value for PMIC state status.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_STAT_MASK (0x0FU)

/**
 * @brief Off state status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_STAT_OFF_STATE (0x00U)

/**
 * @brief Init state status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_STAT_INIT_STATE (0x01 U | 0x02 U | 0x03 U | 0x04U)

/**
 * @brief Power up sequence state status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_STAT_PWRU_SEQ_STATE (0x05U)

/**
 * @brief Reset MCU state status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_STAT_RESET_MCU_STATE (0x06U)

/**
 * @brief Auto BIST state status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_STAT_AUTO_BIST_STATE (0x07U)

/**
 * @brief Active state status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_STAT_ACTIVE_STATE (0x08U)

/**
 * @brief Safe state status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_STAT_SAFE_STATE (0x09U)

/**
 * @brief Real-time BIST state status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_STAT_RT_BIST_STATE (0x0AU)

/**
 * @brief Powered down sequence state status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_STAT_PWRD_SEQ_STATE (0x0CU)

/**
 * @brief Standby state status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STATE_STAT_STANDBY_STATE (0x0DU)

/**
 * @brief Standby configuration register address.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STBY_CFG_REGADDR (0x19U)

/**
 * @brief Standby select shift value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STBY_CFG_STBY_SEL_SHIFT (0x04U)

/**
 * @brief Standby select mask value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STBY_CFG_STBY_SEL_MASK (0x01U << PMIC_STBY_CFG_STBY_SEL_SHIFT)

/**
 * @brief Standby value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_STBY_CFG_STBY_VAL (0x01U << PMIC_STBY_CFG_STBY_SEL_SHIFT)

/**
 * @brief Wake configuration register address.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_WAKE_CFG_REGADDR (0x12U)

/**
 * @brief Wake1 digital configuration shift value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_WAKE1_DGL_CFG_SHIFT (0x00U)

/**
 * @brief Wake2 digital configuration shift value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_WAKE2_DGL_CFG_SHIFT (0x01U)

/**
 * @brief Wake1 digital configuration mask value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_WAKE1_DGL_CFG_MASK (uint8_t)(0x01U << PMIC_WAKE1_DGL_CFG_SHIFT)

/**
 * @brief Wake2 digital configuration mask value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_WAKE2_DGL_CFG_MASK (uint8_t)(0x01U << PMIC_WAKE2_DGL_CFG_SHIFT)

/**
 * @brief BIST control register address.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_BIST_CTRL_REGADDR (0x53U)

/**
 * @brief BIST enable shift value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_BIST_CTRL_EN_SHIFT (0x02U)

/**
 * @brief BIST enable mask value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_BIST_CTRL_EN_MASK (0x01U << PMIC_BIST_CTRL_EN_SHIFT)

/**
 * @brief BIST configuration shift value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_BIST_CTRL_CFG_SHIFT (0x00U)

/**
 * @brief BIST configuration mask value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_BIST_CTRL_CFG_MASK (0x03U << PMIC_BIST_CTRL_CFG_SHIFT)

/**
 * @brief ILIM configuration register address.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_ILIM_CFG_REGADDR (0x60U)

/**
 * @brief ILIM LDO1 configuration mask value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_ILIM_LDO1_CFG_MASK (0x01U)

/**
 * @brief ILIM LDO2 configuration mask value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_ILIM_LDO2_CFG_MASK (0x02U)

/**
 * @brief ILIM LDO3 configuration mask value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_ILIM_LDO3_CFG_MASK (0x04U)

/**
 * @brief ILIM LDO4 configuration mask value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_ILIM_LDO4_CFG_MASK (0x08U)

/**
 * @brief ILIM PLDO1 configuration mask value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_ILIM_PLDO1_CFG_MASK (0x10U)

/**
 * @brief ILIM PLDO2 configuration mask value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_ILIM_PLDO2_CFG_MASK (0x2FU)

/**
 * @brief BB ILIM level configuration mask value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_BB_ILIM_LVL_CFG_MASK (0x08U)

/**
 * @brief DCDC status register address.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_DCDC_STAT_REGADDR (0x5AU)

/**
 * @brief Off FSM state value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_OFF_STATE (0U)

/**
 * @brief Standby FSM state value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STANBY_STATE (1U)

/**
 * @brief Safe FSM state value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_SAFE_STATE (2U)

/**
 * @brief Active FSM state value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_ACTIVE_STATE (3U)

/**
 * @brief MCU only FSM state value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_MCU_ONLY_STATE (4U)

/**
 * @brief S2R FSM state value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_S2R_STATE (5U)

/**
 * @brief Off 1 FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_OFF_1 (0U)

/**
 * @brief Init 1 FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_INIT_1 (1U)

/**
 * @brief Init 2 FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_INIT_2 (2U)

/**
 * @brief Init 3 FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_INIT_3 (3U)

/**
 * @brief Init 4 FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_INIT_4 (4U)

/**
 * @brief Power up sequence FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_PWRU_SEQ (5U)

/**
 * @brief Reset MCU FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_RESET_MCU (6U)

/**
 * @brief Auto BIST FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_AUTO_BIST (7U)

/**
 * @brief Active FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_ACTIVE (8U)

/**
 * @brief Safe FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_SAFE (9U)

/**
 * @brief Real-time BIST FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_RT_BIST (10U)

/**
 * @brief OTP FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_OTP (11U)

/**
 * @brief Powered down sequence FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_PWRD_SEQ (12U)

/**
 * @brief Standby FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_STANDBY (13U)

/**
 * @brief Off 2 FSM status value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STAT_OFF_2 (14U)

/**
 * @brief Fast BIST enable FSM value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_FAST_BIST_ENABLE (1U)

/**
 * @brief Fast BIST disable FSM value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_FAST_BIST_DISABLE (0U)

/**
 * @brief LP Standby state selection value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_SELECT_LPSTANDBY_STATE (1U)

/**
 * @brief Standby state selection value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_SELECT_STANDBY_STATE (0U)

/**
 * @brief NSLEEP1 signal value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_NSLEEP1_SIGNAL (0U)

/**
 * @brief NSLEEP2 signal value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_NSLEEP2_SIGNAL (1U)

/**
 * @brief ILIM interrupt FSM control enable value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_ILIM_INT_ENABLE (1U)

/**
 * @brief ILIM interrupt FSM control disable value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_ILIM_INT_DISABLE (0U)

/**
 * @brief Fast BIST enable configuration valid value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_CFG_FAST_BIST_EN_VALID (0U)

/**
 * @brief LP Standby selection configuration valid value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_CFG_LP_STBY_SEL_VALID (1U)

/**
 * @brief ILIM interrupt FSM control enable configuration valid value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_CFG_ILIM_INT_EN_VALID (2U)

/**
 * @brief FSM startup destination selection configuration valid value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STARTUP_DEST_SEL_VALID (3U)

/**
 * @brief FSM startup destination active value.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FSM_STARTUPDEST_ACTIVE (3U)

/**
 * @brief Shift value for the valid configuration of enabling fast BIST.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_FAST_BIST_EN_VALID_SHIFT (1U << PMIC_FSM_CFG_FAST_BIST_EN_VALID)

/**
 * @brief Shift value for the valid configuration of enabling ILIM interrupt FSM
 * control.
 *
 * @ingroup Pmic_FSMPrivMacros
 */
#define PMIC_ILIM_INT_EN_VALID_SHIFT (1U << PMIC_FSM_CFG_ILIM_INT_EN_VALID)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_INC_PMIC_FSM_PRIV_H_ */

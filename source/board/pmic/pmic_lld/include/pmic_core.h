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
 *  @file pmic_core.h
 *
 *  @brief PMIC Driver Common API/interface file.
 */

#ifndef PMIC_CORE_H_
#define PMIC_CORE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic.h"
#include "pmic_core_tps65386x.h"
#include "pmic_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Pmic_Core PMIC Core
 * @{
 * @brief Contains definitions related to PMIC core functionality.
 */

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @defgroup Pmic_CoreMacros PMIC Macros
 * @{
 * @ingroup Pmic_Core
 * @brief Contains macros used in the PMIC driver.
 */

/**
 * @brief  PMIC driver Core Handle INIT status Magic Number.
 *         Used to validate Handle to avoid corrupted PmicHandle usage.
 *         On Success: (DRV_INIT_SUCCESS | Pmic_InstType_t)
 *
 * @ingroup Pmic_CoreMacros
 */
#define DRV_INIT_SUCCESS (0xABCD0000U)

/**
 * @brief Silicon Revision ID for Power Group 2.0.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SILICON_REV_ID_PG_2_0 (0x08U)

/**
 * @brief Silicon Revision ID for Power Group 1.0.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SILICON_REV_ID_PG_1_0 (0x0U)

/**
 * @brief Register values for Blackbird scratch pad 1
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SCRATCH_PAD_REG_1 (0x0U)

/**
 * @brief Register values for Blackbird scratch pad 2
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SCRATCH_PAD_REG_2 (0x1U)

/**
 * @brief validParams value used to get Register lock status
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_REG_LOCK_STAT_VALID (1U)

/**
 * @brief validParams value used to get External clock validity status
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID (2U)

/**
 * @brief validParams value used to get Startup(nPWRON/Enable) pin status
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_STARTUP_PIN_STAT_VALID (3U)

/**
 * @brief validParams value used to get nRSTOUT_SOC Pin status
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID (5U)

/**
 * @brief validParams value used to get nRSTOUT Pin status
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_NRSTOUT_PIN_STAT_VALID (6U)

/**
 * @brief validParams value used to get nINT Pin status
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_NINT_PIN_STAT_VALID (7U)

/**
 * @brief validParams value used to get SPMI Low Power Mode status
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SPMI_LPM_STAT_VALID (8U)

/**
 * @brief validParams value used to get status of ENABLE_DRV Configuration by
 * I2C/SPI
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID (9U)

/**
 * @brief validParams value used to get ENABLE_OUT Pin status
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_EN_OUT_PIN_STAT_VALID (0U)

/**
 * @brief validParams value used to get SAFE_OUT1 Pin status
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SAFE_OUT1_PIN_STAT_VALID (1U)

/**
 * @brief validParams value used to get NRST Pin status
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_NRST_PIN_STAT_VALID (2U)

/**
 * @brief Shift value for the EOC (End of Conversion) indication status validity
 * bit.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_BB_EOC_INDICATION_STAT_VALID_SHIFT                            \
  (1U << PMIC_CFG_BB_EOC_INDICATION_STAT_VALID)

/**
 * @brief Shift value for the register lock status validity bit.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_REG_LOCK_STAT_VALID_SHIFT                                \
  (1U << PMIC_CFG_REG_LOCK_STAT_VALID)

/**
 * @brief Shift value for the external clock validity status validity bit.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_EXTCLK_VALIDITY_STAT_VALID_SHIFT                             \
  (1U << PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID)

/**
 * @brief Shift value for the startup pin status validity bit.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_STARTUP_PINSTAT_VALID_SHIFT                                  \
  (1U << PMIC_CFG_STARTUP_PIN_STAT_VALID)

/**
 * @brief Shift value for the enable driver pin status validity bit.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_EN_DRV_PIN_STAT_VALID_SHIFT                                   \
  (1U << PMIC_CFG_EN_DRV_PIN_STAT_VALID)

/**
 * @brief Shift value for the NRSTOUTSOC (Reset Out SOC) pin status validity
 * bit.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_NRSTOUTSOC_PINSTAT_VALID_SHIFT                               \
  (1U << PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID)

/**
 * @brief Shift value for the NRSTOUT (Reset Out) pin status validity bit.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_NRSTOUT_PINSTAT_VALID_SHIFT                                  \
  (1U << PMIC_CFG_NRSTOUT_PIN_STAT_VALID)

/**
 * @brief Shift value for the NINT (Interrupt) pin status validity bit.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_NINT_PIN_STAT_VALID_SHIFT (1U << PMIC_CFG_NINT_PIN_STAT_VALID)

/**
 * @brief Shift value for the SPMI (System Packet Interface - Master) LPM (Low
 * Power Mode) status validity bit.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SPMI_LPM_STAT_VALID_SHIFT (1U << PMIC_CFG_SPMI_LPM_STAT_VALID)

/**
 * @brief Shift value for the force enable driver low status validity bit.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_FORCE_EN_DRV_LOW_STAT_VALID_SHIFT                         \
  (1U << PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID)

/**
 * @brief Signal level low for PMIC pin.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_PIN_SIGNAL_LEVEL_LOW (0U)

/**
 * @brief Signal level high for PMIC pin.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_PIN_SIGNAL_LEVEL_HIGH (1U)

/**
 * @brief Enable driver pin type.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_PIN_TYPE_EN_DRV (0U)

/**
 * @brief NRSTOUT_SOC (Reset Out SOC) pin type.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_PIN_TYPE_NRSTOUT_SOC (1U)

/**
 * @brief NRSTOUT (Reset Out) pin type.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_PIN_TYPE_NRSTOUT (2U)

/**
 * @brief NRST_RDBK_LVL (Reset Readback Level) pin type.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_PIN_TYPE_NRST_RDBK_LVL (0U)

/**
 * @brief SAFE_OUT1_RDBK_LVL (Safe Output 1 Readback Level) pin type.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_PIN_TYPE_SAFEOUT1_RDBK_LVL (1U)

/**
 * @brief EN_OUT_RDBK_LVL (Enable Output Readback Level) pin type.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_PIN_TYPE_EN_OUT_RDBK_LVL (2U)

/**
 * @brief GPO1_RDBK_LVL (General Purpose Output 1 Readback Level) pin type.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_PIN_TYPE_GPO1_RDBK_LVL (3U)

/**
 * @brief GPO2_RDBK_LVL (General Purpose Output 2 Readback Level) pin type.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_PIN_TYPE_GPO2_RDBK_LVL (4U)

/**
 * @brief GPO3_RDBK_LVL (General Purpose Output 3 Readback Level) pin type.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_PIN_TYPE_GPO3_RDBK_LVL (5U)

/**
 * @brief GPO4_RDBK_LVL (General Purpose Output 4 Readback Level) pin type.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_PIN_TYPE_GPO4_RDBK_LVL (6U)

/**
 * @brief Enable driver I2C/SPI configuration enable.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_EN_DRV_I2C_SPI_CFG_ENABLE (0U)

/**
 * @brief Enable driver I2C/SPI configuration disable.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_EN_DRV_I2C_SPI_CFG_DISABLE (1U)

/**
 * @brief Unlock data 1 for register and timer counter.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_REGISTER_UNLOCK_DATA1 (0x98U)

/**
 * @brief Unlock data 2 for register and timer counter.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_REGISTER_UNLOCK_DATA2 (0xB8U)

/**
 * @brief Unlock data 1 for timer and rotational counter.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_TMR_COUNTER_UNLOCK_DATA1 (0x13U)

/**
 * @brief Unlock data 2 for timer and rotational counter.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_TMR_COUNTER_UNLOCK_DATA2 (0x7DU)

/**
 * @brief Lock data for register.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_REGISTER_LOCK_1 (0x10U)

/**
 * @brief Lock data for timer counter.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_REGISTER_LOCK_2 (0x10U)

/**
 * @brief Register status indicating unlocked state.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_REGISTER_STATUS_UNLOCK (0x0U)

/**
 * @brief Register status indicating locked state.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_REGISTER_STATUS_LOCK (0x1U)

/**
 * @brief Spread spectrum configuration disable value.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SPREAD_SPECTRUM_CFG_DISABLE 0U

/**
 * @brief Spread spectrum configuration enable value.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SPREAD_SPECTRUM_CFG_ENABLE 1U

/**
 * @brief Spread spectrum modulation depth indicating no modulation.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE (0U)

/**
 * @brief Spread spectrum modulation depth indicating 6.3% modulation.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SS_MODULATION_DEPTH_6_3_PERCENT (1U)

/**
 * @brief Spread spectrum modulation depth indicating 8.4% modulation.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SS_MODULATION_DEPTH_8_4_PERCENT (2U)

/**
 * @brief Configuration for SPMI (System Packet Interface - Master) LPM (Low
 * Power Mode) mode control when disabled.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SPMI_LPM_MODE_CTRL_CFG_DISABLED (0U)

/**
 * @brief Configuration for SPMI LPM mode control when enabled.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SPMI_LPM_MODE_CTRL_CFG_ENABLED (1U)

/**
 * @brief Configuration for internal clock monitoring when disabled.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE (0U)

/**
 * @brief Configuration for internal clock monitoring when enabled.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_INTERNALCLK_MONITORING_CFG_ENABLE (1U)

/**
 * @brief Configuration for synchronous clock output when disabled.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SYNCCLKOUT_DISABLE (0U)

/**
 * @brief Configuration for synchronous clock output with 1.1 MHz frequency.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SYNCCLKOUT_1_1_MHZ (1U)

/**
 * @brief Configuration for synchronous clock output with 2.2 MHz frequency.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SYNCCLKOUT_2_2_MHZ (2U)

/**
 * @brief Configuration for synchronous clock output with 4.4 MHz frequency.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SYNCCLKOUT_4_4_MHZ (3U)

/**
 * @brief Configuration for internal RC oscillator.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_INTERNAL_RC_OSC (0U)

/**
 * @brief Configuration for automatic external clock.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_AUTOMATIC_EXT_CLK (1U)

/**
 * @brief Status indicating validity of external clock.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_EXT_CLK_STATUS_VALID (0U)

/**
 * @brief Status indicating invalidity of external clock.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_EXT_CLK_STATUS_INVALID (1U)

/**
 * @brief Configuration for CRC (Cyclic Redundancy Check) status when disabled.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CRC_STATUS_DISABLED (0U)

/**
 * @brief Configuration for CRC status when enabled.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CRC_STATUS_ENABLED (1U)

/**
 * @brief Validity of Spread Spectrum Enable configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SPREAD_SPECTRUM_EN_VALID (1U)

/**
 * @brief Validity of Enable safe output configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_ENABLE_SAFEOUT_VALID (1U)

/**
 * @brief Validity of Register Lock configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_REG_LOCK_VALID (2U)

/**
 * @brief Validity of Spread Spectrum Depth configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID (3U)

/**
 * @brief Validity of EEPROM Default Load configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_EEPROM_DEFAULT_VALID (4U)

/**
 * @brief Validity of Skip EEPROM Load configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SKIP_EEPROM_LOAD_VALID (5U)

/**
 * @brief Shift value for the validity of Spread Spectrum Enable configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SPREADSPECTRUM_EN_VALID_SHIFT                                \
  (1U << PMIC_CFG_SPREAD_SPECTRUM_EN_VALID)

/**
 * @brief Shift value for the validity of Enable driver configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_ENABLE_DRV_VALID_SHIFT (1U << PMIC_CFG_ENABLE_DRV_VALID)

/**
 * @brief Shift value for the validity of Register Lock configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_REG_LOCK_VALID_SHIFT (1U << PMIC_CFG_REG_LOCK_VALID)

/**
 * @brief Shift value for the validity of Spread Spectrum Depth configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SPREADSPECTRUM_DEPTH_VALID_SHIFT                             \
  (1U << PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID)

/**
 * @brief Shift value for the validity of EEPROM Default Load configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT                                    \
  (1U << PMIC_CFG_EEPROM_DEFAULT_VALID)

/**
 * @brief Shift value for the validity of Skip EEPROM Load configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SKIPEEPROM_LOAD_VALID_SHIFT                                  \
  (1U << PMIC_CFG_SKIP_EEPROM_LOAD_VALID)

/**
 * @brief Validity value for AMUX output reference output enable configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID (0U)

/**
 * @brief Validity value for AMUX demultiplexer enable configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_AMUX_DMUX_EN_VALID (6U)

/**
 * @brief Validity value for clock monitoring enable configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_CLK_MON_EN_VALID (1U)

/**
 * @brief Validity value for synchronous clock output frequency selection
 * configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID (2U)

/**
 * @brief Validity value for external clock selection configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_EXT_CLK_SEL_VALID (3U)

/**
 * @brief Validity value for synchronous clock input frequency configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SYNC_CLK_IN_FREQ_VALID (4U)

/**
 * @brief Validity value for NRSTOUT_SOC (System On Chip) configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_NRSTOUT_SOC_VALID (5U)

/**
 * @brief Validity value for NRSTOUT (Reset Output) configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_NRSTOUT_VALID (6U)

/**
 * @brief Shift value for the validity of AMUX output reference output enable
 * configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_AMUXOUT_REF_OUT_EN_VALID_SHIFT                               \
  (1U << PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID)

/**
 * @brief Shift value for the validity of clock monitoring enable configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_CLK_MON_EN_VALID_SHIFT (1U << PMIC_CFG_CLK_MON_EN_VALID)

/**
 * @brief Shift value for the validity of synchronous clock output frequency
 * selection configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SYNC_CLKOUT_FREQ_SEL_VALID_SHIFT                             \
  (1U << PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID)

/**
 * @brief Shift value for the validity of external clock selection
 * configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_EXT_CLK_SEL_VALID_SHIFT (1U << PMIC_CFG_EXT_CLK_SEL_VALID)

/**
 * @brief Shift value for the validity of synchronous clock input frequency
 * configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_SYNC_CLKIN_FREQ_VALID_SHIFT                                  \
  (1U << PMIC_CFG_SYNC_CLK_IN_FREQ_VALID)

/**
 * @brief Shift value for the validity of NRSTOUT_SOC (System On Chip)
 * configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_NRSTOUT_SOC_VALID_SHIFT (1U << PMIC_CFG_NRSTOUT_SOC_VALID)

/**
 * @brief Shift value for the validity of NRSTOUT (Reset Output) configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_NRSTOUT_VALID_SHIFT (1U << PMIC_CFG_NRSTOUT_VALID)

/**
 * @brief Shift value for the validity of charging enable configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_CHARGING_EN_VALID_SHIFT (1U << PMIC_CFG_CHARGING_EN_VALID)

/**
 * @brief Shift value for the validity of end of charge voltage configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT                             \
  (1U << PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID)

/**
 * @brief Shift value for the validity of charge current configuration.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_CFG_CHARGE_CURRENT_VALID_SHIFT                                    \
  (1U << PMIC_CFG_CHARGE_CURRENT_VALID)

/**
 * @brief Value representing low enable state for Safe Output 1.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT1_ENABLE_LOW (0x00U)

/**
 * @brief Value representing high enable state for Safe Output 1.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT1_ENABLE_HIGH (0x01U)

/**
 * @brief Value representing low enable state for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_ENABLE_LOW (0x00U)

/**
 * @brief Value representing follow configuration for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_ENABLE_FOLLOW_CFG (0x01U)

/**
 * @brief Value representing inverted level configuration for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_CFG_INV_LEVEL (0x00U)

/**
 * @brief Value representing matching level configuration for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_CFG_MATCH_LEVEL (0x01U)

/**
 * @brief Value representing PWM heartbeat low configuration for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_CFG_PWM_HEARTBEAT_LOW (0x02U)

/**
 * @brief Value representing PWM heartbeat second configuration for Safe
 * Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFEOUT2_CFG_PWM_HEARTBEAT_SECOND (0x03U)

/**
 * @brief Value representing delayed continuous configuration for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_CFG_DELAYED_CONTINUOUS (0x04U)

/**
 * @brief Value representing delayed pulse configuration for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_CFG_DELAYED_PULSE (0x05U)

/**
 * @brief Value representing prescale of 4 microseconds for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_PRESCALE_4US (0x00U)

/**
 * @brief Value representing prescale of 16 microseconds for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_PRESCALE_16US (0x01U)

/**
 * @brief Value representing prescale of 64 microseconds for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_PRESCALE_64US (0x02U)

/**
 * @brief Value representing prescale of 256 microseconds for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_PRESCALE_256US (0x03U)

/**
 * @brief Value representing prescale of 1 millisecond for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_PRESCALE_1MS (0x04U)

/**
 * @brief Value representing prescale of 4 milliseconds for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_PRESCALE_4MS (0x05U)

/**
 * @brief Value representing prescale of 16 milliseconds for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_PRESCALE_16MS (0x06U)

/**
 * @brief Value representing prescale of 65 milliseconds for Safe Output 2.
 *
 * @ingroup Pmic_CoreMacros
 */
#define PMIC_SAFE_OUT2_PRESCALE_65MS (0x07U)

/**
 * @}
 */ /* End of Pmic_CoreMacros */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @defgroup Pmic_CoreStructures PMIC Core Structures
 * @{
 * @ingroup Pmic_Core
 * @brief Contains structures used in the core module of PMIC driver.
 */

/**
 * @brief  PMIC Recovery Counter Configuration
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIS and output
 *         param for Get APIs
 *
 * @param   validParams   Selection of structure parameters to be set,
 *                        from the combination of @ref Pmic_RecoveryCntCfgType
 *                        and the corresponding member value must be updated.
 *                        Valid values @ref Pmic_RecoveryCntCfgType
 *  @param  thrVal        Recovery Counter Threshold Value.
 *                         Valid only when PMIC_CFG_RECOV_CNT_THR_VAL_VALID
 *                         bit is set.
 *  @param  clrCnt        Clear Recovery Counter Value and value should be 1U.
 *                         Valid only when PMIC_CFG_RECOV_CNT_CLR_VAL_VALID
 *                         bit is set.
 *
 * @ingroup Pmic_CoreStructures
 */
typedef struct Pmic_RecovCntCfg_s {
    uint8_t validParams;
    uint8_t thrVal;
    bool clrCnt;
}
Pmic_RecovCntCfg_t;

/**
 * @brief  PMIC common control param configuration
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * @param   validParams             Selection of structure parameters to be set,
 *                                  from the combination of
 *                                  @ref Pmic_CommonCtrlValidParamCfg
 *                                  and the corresponding member value must be
 *                                  updated
 *                                    Valid values
 *                                        @ref Pmic_CommonCtrlValidParamCfg
 *  @param  sreadSpectrumEn         Spread Spectrum Enable Value
 *                                  Valid only when
 *                                  PMIC_CFG_SPREAD_SPECTRUM_EN_VALID bit is set
 *                                    Valid values @ref Pmic_SpreadSpectrum_Cfg
 *  @param  skipEepromDefaultLoadEn Enable/Disable to skip EEPROM defaults load
 *                                  on conf registers when device transition
 *                                  from  Lpstandby to INIT state
 *                                  Valid only for LP8764x Hera Device
 *                                  Valid only when
 *                                  PMIC_CFG_SKIP_EEPROM_LOAD_VALID
 *                                  bit is set.
 *                                    Valid values
 *                                    @ref Pmic_Lp8764xHera_Skip_EepromDef_LdCfg
 *  @param  eepromDefaultLoad       Load/Not Loaded from EEPROM defaults on
 *                                  RTC domain Registers (for TPS6594x Leo
 *                                  Device) when device transition from
 *                                  Lpstandby/SafeRecovery to INIT state
 *                                  Enable/Disable load from EEPROM defaults on
 *                                  conf registers when
 *                                  skipEepromDefaultLoadEn = 0
 *                                  when device transition from Lpstandby to
 *                                  INIT state
 *                                  Load/Not Loaded load from EEPROM defaults on
 *                                  conf registers when device transition from
 *                                  SafeRecovery to INIT state.Doesn't depends
 *                                  on  skipEepromDefaultLoadEn Value
 *                                  (for LP8764x Hera Device)
 *                                  Valid only when
 *                                  PMIC_CFG_EEPROM_DEFAULT_VALID bit is set.
 *                                      Valid values
 *                                         @ref Pmic_Tps6594xLeo_EepromDef_LdCfg
 *                                           (for TPS6594x Leo Device)
 *                                         @ref Pmic_Lp8764xHera_EepromDef_LdCfg
 *                                           (for LP8764x Hera Device)
 *  @param  enDrv                   Control of ENABLE_DRV pin. Can be configured
 *                                  only When forceEnDrvLow set to 0 else
 *                                  ENABLE_DRV pin is set to
 *                                   PMIC_PIN_SIGNAL_LEVEL_LOW.
 *                                     Valid values @ref Pmic_SignalLvl
 *                                  Valid only when PMIC_CFG_ENABLE_DRV_VALID
 *                                  bit is set.
 *  @param  regLock                 Register Lock configuration
 *                                  Valid values @ref Pmic_RegisterLock_Config
 *                                  Valid only when PMIC_CFG_REG_LOCK_VALID
 *                                  bit is set
 *                                  Valid only for Pmic_setCommonCtrlConfig API
 *  @param  spreadSpectrumDepth     Spread Spectrum modulation Depth Value
 *                                     Valid values
 *                                        @ref Pmic_SpreadSpectrum_Mod_Depth_Sel
 *                                  Valid only when
 *                                  PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID
 *                                  bit is set.
 *  @param  eNsafeOut1              Control bit to enable the SAFE_OUT1 output.
 *  @param  eNsafeOut2              Control bit to enable the SAFE_OUT2 output.
 *  @param  regLock_1               Lock/Unlock sequence-1 for configuration
 * register lock/unlock.
 *  @param  regLock_2               Lock/Unlock sequence-2 for configuration
 * register lock/unlock.
 *  @param  cntLock_1               Lock/Unlock sequence-1 for timer and
 * rotation counter register lock/unlock.
 *  @param  cntLock_2               Lock/Unlock sequence-2 for timer and
 * rotation counter register lock/unlock.
 *
 * @ingroup Pmic_CoreStructures
 */
typedef struct Pmic_CommonCtrlCfg_s {
    uint32_t validParams;
    uint32_t sreadSpectrumEn;
    bool skipEepromDefaultLoadEn;
    uint8_t eepromDefaultLoad;
    uint8_t enDrv;
    uint8_t regLock;
    uint8_t spreadSpectrumDepth;
    uint8_t eNsafeOut1;
    uint8_t eNsafeOut2;
    uint8_t regLock_1;
    uint8_t regLock_2;
    uint8_t cntLock_1;
    uint8_t cntLock_2;
}
Pmic_CommonCtrlCfg_t;

/**
 * @brief PMIC Safe State Configuration
 *
 * This structure contains parameters related to PMIC safe state configuration.
 *
 * @param validParams           Selection of structure parameters to be set.
 * @param safeStateTMO          Safe state timeout value.
 * @param safeLockThreshold     Safe state lock threshold value.
 *
 * @ingroup Pmic_CoreStructures
 */
typedef struct Pmic_SafeStateCfg_s {
    uint8_t validParams;
    uint8_t safeStateTMO;
    uint8_t safeLockThreshold;
}
Pmic_SafeStateCfg_t;

/**
 * @brief  PMIC Miscellaneous control param Configuration
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * @param   validParams         Selection of structure parameters to be set,
 *                              from the combination of
 *                              @ref Pmic_MiscCtrlValidParamCfg and the
 *                              corresponding member value must be updated.
 *                                 Valid values @ref Pmic_MiscCtrlValidParamCfg
 *  @param  amuxOutRefOutEn     Enable/Disable Band gap Voltage to AMUX OUT Pin
 *                              (for TPS6594x Leo Device) or RFF OUT Pin
 *                              (for LP8764x Leo Device)
 *                              Valid only when
 *                              PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID bit is set
 *                                 Valid values
 *                                     @ref Pmic_Tps6594xLeo_AMuxOutPinCtrl_Cfg
 *                                      (for TPS6594x Leo Device)
 *                                     @ref Pmic_Lp8764xHera_RefOutPinCtrl_Cfg
 *                                      (for LP8764x Hera Device)
 *  @param  clkMonEn            Enable or Disable internal Clock Monitoring
 *                              Valid only when PMIC_CFG_CLK_MON_EN_VALID
 *                              bit is set.
 *                                 Valid values @ref Pmic_InternalClkMonitor_Cfg
 *  @param  syncClkOutFreqSel   Selects SYNCCLKOUT Frequency
 *                                 Valid values @ref Pmic_SyncClkOut_Freq_Sel
 *                              Valid only when
 *                              PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID bit is set
 *  @param  extClkSel           External clock Selection
 *                                 Valid values @ref Pmic_ExtClk_Sel
 *                              Valid only when PMIC_CFG_EXT_CLK_SEL_VALID
 *                              bit is set.
 *  @param  syncClkInFreq       Selects External clock Frequency
 *                              Valid only when PMIC_CFG_SYNC_CLK_IN_FREQ_VALID
 *                              bit is set.
 *                                 Valid values
 *                                     @ref Pmic_Tps6594xLeo_ExtClk_Freq_Sel
 *                                      (for TPS6594x Leo Device)
 *                                     @ref Pmic_Lp8764xHera_ExtClk_Freq_Sel
 *                                      (for LP8764x Hera Device)
 *  @param  nRstOutSocSignal    Configure NRSTOUT_SOC Signal
 *                              Note: When Application configures
 *                              nRstOutSocSignal as PMIC_PIN_SIGNAL_LEVEL_LOW
 *                              then SOC will be in reset
 *                              Valid only when PMIC_CFG_NRSTOUT_SOC_VALID
 *                              bit is set.
 *                                 Valid values @ref Pmic_SignalLvl
 *  @param  nRstOutSignal       Configure NRSTOUT Signal
 *                              Note: When Application configures
 *                              nRstOutSignal as PMIC_PIN_SIGNAL_LEVEL_LOW
 *                              then MCU will be in reset
 *                              Valid only when PMIC_CFG_NRSTOUT_VALID
 *                              bit is set.
 *                                 Valid values @ref Pmic_SignalLvl
 *
 * @ingroup Pmic_CoreStructures
 */
typedef struct Pmic_MiscCtrlCfg_s {
    uint8_t validParams;
    bool amuxOutRefOutEn;
    bool clkMonEn;
    uint8_t syncClkOutFreqSel;
    uint8_t extClkSel;
    uint8_t syncClkInFreq;
    uint8_t nRstOutSocSignal;
    uint8_t nRstOutSignal;
}
Pmic_MiscCtrlCfg_t;

/**
 * @brief PMIC Diagnostic Output Configuration Control
 *
 * This structure holds the PMIC diagnostic output configuration control
 * information.
 *
 * @param validParams           Selection of structure parameters to be set.
 * @param DiagOutCtrl           Diagnostic output control.
 * @param DiagOutCtrl_AMUXEn    Diagnostic output AMUX enable.
 * @param DiagOutCtrl_DMUXEn    Diagnostic output DMUX enable.
 * @param DiagGrpSel            Diagnostic group selection.
 * @param DiagChannelSel        Diagnostic channel selection.
 *
 * @ingroup Pmic_CoreStructures
 */
typedef struct Pmic_DiagOutCfgCtrl_s {
    uint8_t validParams;
    uint8_t DiagOutCtrl;
    uint8_t DiagOutCtrl_AMUXEn;
    uint8_t DiagOutCtrl_DMUXEn;
    uint8_t DiagGrpSel;
    uint8_t DiagChannelSel;
}
Pmic_DiagOutCfgCtrl_t;

/**
 * @brief  PMIC common control param status
 *         Note: validParams is input param for all Get APIs. other params
 *         except validParams is output param for Get APIs
 *
 * @param   validParams        Selection of structure parameters to be set, from
 *                             the combination of
 *                             @ref Pmic_CommonCtrlStatValidParamCfg
 *                             and the corresponding member value must be
 *                             updated
 *                                Valid values
 *                                         @ref Pmic_CommonCtrlStatValidParamCfg
 *  @param  spmiLpmStat        SPMI Low Power Mode Control Status.
 *                             Valid only when PMIC_CFG_SPMI_LPM_STAT_VALID
 *                             bit is set.
 *                                Valid values @ref Pmic_SpmiLpmModeCtrl_Stat
 *  @param  forceEnDrvLowStat  Status of ENABLE_DRV Configuration by I2C/SPI
 *                             Valid only when
 *                             PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID
 *                             bit is set.
 *                                Valid values @ref Pmic_EnableDrvI2CSPICfg_Stat
 *  @param  bbEocIndication    Backup Battery End of charge Indication Status
 *                             Valid only when
 *                             PMIC_CFG_BB_EOC_INDICATION_STAT_VALID
 *                             bit is set
 *                              Valid only for TPS6594x Leo Device
 *                                 Valid values
 *                                     @ref Pmic_Tps6594xLeo_BBEoCIndicationStat
 *  @param  regLockStat        Register lock status
 *                             Valid only when PMIC_CFG_REG_LOCK_STAT_VALID
 *                             bit is set
 *                                 Valid values @ref Pmic_RegisterLock_Stat
 *  @param  extClkValidity     External clock validity status. The status value
 *                             is valid only when External clock is connected
 *                             Ignore the status value when External clock is
 * not connected Valid only when PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID bit is set
 *                                Valid values @ref Pmic_ExtClkValidStat
 *  @param  startupPin         Startup(nPWRON/Enable) pin status
 *                             Valid only when PMIC_CFG_STARTUP_PIN_STAT_VALID
 *                             bit is set
 *                                Valid values @ref Pmic_SignalLvl
 *  @param  enDrvPin           EN_DRV Pin status
 *                             Valid only when PMIC_CFG_EN_DRV_PIN_STAT_VALID
 *                             bit is set.
 *                                Valid values @ref Pmic_SignalLvl
 *  @param  nRstOutSocPin      nRSTOUT_SOC Pin status
 *                             Valid only when
 *                             PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID bit is set.
 *                                Valid values @ref Pmic_SignalLvl
 *  @param  nRstOutPin        nRSTOUT Pin status
 *                             Valid only when PMIC_CFG_NRSTOUT_PIN_STAT_VALID
 *                             bit is set.
 *                                Valid values @ref Pmic_SignalLvl
 *  @param  nIntPin           nINT Pin status
 *                             Valid only when PMIC_CFG_NINT_PIN_STAT_VALID
 *                             bit is set.
 *                                Valid values @ref Pmic_SignalLvl
 *  @param  cfgregLockStat    Lock status of configuration registers
 *  @param  cntregLockStat
 *
 * @ingroup Pmic_CoreStructures
 */
typedef struct Pmic_CommonCtrlStat_s {
    uint32_t validParams;
    bool spmiLpmStat;
    uint8_t forceEnDrvLowStat;
    uint8_t bbEndOfChargeIndication;
    uint8_t regLockStat;
    uint8_t extClkValidity;
    uint8_t startupPin;
    uint8_t enDrvPin;
    uint8_t nRstOutSocPin;
    uint8_t nRstOutPin;
    uint8_t nIntPin;
    uint8_t enOutPin;
    uint8_t safeOut1Pin;
    uint8_t nRstPin;
    uint8_t cfgregLockStat;
    uint8_t cntregLockStat;
}
Pmic_CommonCtrlStat_t;

/**
 * @brief  PMIC Device Information
 *
 *  @param  deviceID        TI Device ID Value
 *  @param  nvmID           TI NVM ID Value
 *  @param  nvmRev          TI NVM Revision
 *  @param  siliconRev      TI Silicon Revision
 *  @param  customNvmID     Customer configured NVM ID Value
 *                          customNvmID value is valid only for TPS6594x Leo
 *                          PMIC PG2.0 and LP8764x Hera PMIC PG2.0
 *
 * @ingroup Pmic_CoreStructures
 */
typedef struct Pmic_DeviceInfo_s {
    uint8_t deviceID;
    uint8_t nvmID;
    uint8_t nvmRev;
    uint8_t siliconRev;
    uint8_t customNvmID;
}
Pmic_DeviceInfo_t;

/**
 * @brief Common PMIC State Status Information
 *
 * This structure holds common PMIC state status information.
 *
 * @param rstMcuCnt Pointer to RST_MCU_CNT bits (7-6)
 * @param rstMcuRqFlag Pointer to RST_MCU_RQ_FLAG bit (5)
 * @param pwrdDlyActv Pointer to PWRD_DLY_ACTV bit (4)
 * @param state Pointer to STATE bits (3-0)
 *
 * @ingroup Pmic_CoreStructures
 */
typedef struct Pmic_CommonStateStat_s {
    uint8_t * rstMcuCnt;
    uint8_t * rstMcuRqFlag;
    uint8_t * pwrdDlyActv;
    uint8_t * state;
}
Pmic_CommonStateStat_t;

/**
 * @brief Common PMIC State Control Information
 *
 * This structure contains the state request information for PMIC.
 *
 * @param state_req State request value
 *
 * @ingroup Pmic_CoreStructures
 */
typedef struct Pmic_CommonStateCtrl_s {
    uint8_t state_req;
}
Pmic_CommonStateCtrl_t;

typedef struct {
    uint8_t group;
    uint8_t channel;
}
Pmic_DiagMUXFeatureMapping;

#define PMIC_START_AMUX (0U)
#define PMIC_END_AMUX (19U)

#define PMIC_START_DMUX (0U)
#define PMIC_END_DMUX (131U)

typedef enum {
    FEATURE_BUCK_BOOST_OUTPUT_VOLTAGE,
    FEATURE_LDO1_OUTPUT_VOLTAGE,
    FEATURE_LDO2_OUTPUT_VOLTAGE,
    FEATURE_LDO3_OUTPUT_VOLTAGE,
    FEATURE_LDO4_OUTPUT_VOLTAGE,
    FEATURE_PROTECTED_PLDO1_OUTPUT_VOLTAGE,
    FEATURE_PROTECTED_PLDO2_OUTPUT_VOLTAGE,
    FEATURE_VOLTAGE_ON_TRACK_PIN,
    FEATURE_SECONDARY_BATTERY_SUPPLY_VOLTAGE,
    FEATURE_POWER_BATTERY_SUPPLY_VOLTAGE,
    FEATURE_MAIN_BANDGAP,
    FEATURE_COMPARE_BANDGAP,
    FEATURE_TEMP_SENSOR_BUCK_BOOST,
    FEATURE_TEMP_SENSOR_LDO1,
    FEATURE_TEMP_SENSOR_LDO2,
    FEATURE_TEMP_SENSOR_LDO3,
    FEATURE_TEMP_SENSOR_LDO4,
    FEATURE_TEMP_SENSOR_PLDO1,
    FEATURE_TEMP_SENSOR_PLDO2,
    AMUX_NUM_FEATURES
}
Pmic_AMUXFeatures;

typedef enum {
    FEATURE_DIGITAL_0_OUTPUT,
    FEATURE_DIGITAL_1_OUTPUT,
    FEATURE_BUCK_BOOST_AVG_CURRENT_LIMIT,
    FEATURE_BUCK_BOOST_DEGLITCHED_AVG_CURRENT_LIMIT,
    FEATURE_BUCK_BOOST_PEAK_CURRENT_LIMIT,
    FEATURE_BUCK_BOOST_DEGLITCHED_PEAK_CURRENT_LIMIT,
    FEATURE_RESERVED_2,
    FEATURE_LDO1_CURRENT_LIMIT,
    FEATURE_LDO1_DEGLITCHED_CURRENT_LIMIT,
    FEATURE_LDO1_BYPASS_ENABLE,
    FEATURE_LDO2_CURRENT_LIMIT,
    FEATURE_LDO2_DEGLITCHED_CURRENT_LIMIT,
    FEATURE_LDO2_BYPASS_ENABLE,
    FEATURE_LDO3_CURRENT_LIMIT,
    FEATURE_LDO3_DEGLITCHED_CURRENT_LIMIT,
    FEATURE_LDO3_BYPASS_ENABLE,
    FEATURE_LDO4_CURRENT_LIMIT,
    FEATURE_LDO4_DEGLITCHED_CURRENT_LIMIT,
    FEATURE_LDO4_BYPASS_ENABLE,
    FEATURE_PLDO1_CURRENT_LIMIT,
    FEATURE_PLDO1_DEGLITCHED_CURRENT_LIMIT,
    FEATURE_RESERVED_22,
    FEATURE_PLDO1_TRACKING_MODE_ENABLE,
    FEATURE_PLDO2_CURRENT_LIMIT,
    FEATURE_PLDO2_DEGLITCHED_CURRENT_LIMIT,
    FEATURE_RESERVED_25,
    FEATURE_PLDO2_TRACKING_MODE_ENABLE,
    FEATURE_RESERVED_28,
    FEATURE_RESERVED_29,
    FEATURE_VBAT_DEGLITCHED_OV,
    FEATURE_VBAT_DEGLITCHED_OVP,
    FEATURE_VBAT_DEGLITCHED_UV,
    FEATURE_BUCK_BOOST_DEGLITCHED_OV,
    FEATURE_BUCK_BOOST_DEGLITCHED_OVP,
    FEATURE_BUCK_BOOST_DEGLITCHED_UV,
    FEATURE_RESERVED_37,
    FEATURE_RESERVED_38,
    FEATURE_LDO1_DEGLITCHED_UV,
    FEATURE_LDO2_DEGLITCHED_UV,
    FEATURE_LDO3_DEGLITCHED_UV,
    FEATURE_LDO4_DEGLITCHED_UV,
    FEATURE_LDO1_DEGLITCHED_OV,
    FEATURE_LDO2_DEGLITCHED_OV,
    FEATURE_LDO3_DEGLITCHED_OV,
    FEATURE_LDO4_DEGLITCHED_OV,
    FEATURE_PLDO1_DEGLITCHED_UV,
    FEATURE_PLDO2_DEGLITCHED_UV,
    FEATURE_PLDO1_DEGLITCHED_OV,
    FEATURE_PLDO2_DEGLITCHED_OV,
    FEATURE_EXT_VMON1_DEGLITCHED_UV,
    FEATURE_EXT_VMON2_DEGLITCHED_UV,
    FEATURE_EXT_VMON1_DEGLITCHED_OV,
    FEATURE_EXT_VMON2_DEGLITCHED_OV,
    FEATURE_VBAT_OV,
    FEATURE_VBAT_OVP,
    FEATURE_VBAT_UV,
    FEATURE_BUCK_BOOST_OV,
    FEATURE_BUCK_BOOST_OVP,
    FEATURE_BUCK_BOOST_UV,
    FEATURE_RESERVED_67,
    FEATURE_RESERVED_68,
    FEATURE_LDO1_UV,
    FEATURE_LDO2_UV,
    FEATURE_LDO3_UV,
    FEATURE_LDO4_UV,
    FEATURE_LDO1_OV,
    FEATURE_LDO2_OV,
    FEATURE_LDO3_OV,
    FEATURE_LDO4_OV,
    FEATURE_PLDO1_UV,
    FEATURE_PLDO2_UV,
    FEATURE_PLDO1_OV,
    FEATURE_PLDO2_OV,
    FEATURE_EXT_VMON1_UV,
    FEATURE_EXT_VMON2_UV,
    FEATURE_EXT_VMON1_OV,
    FEATURE_EXT_VMON2_OV,
    FEATURE_BUCK_BOOST_TEMP_PREWARNING,
    FEATURE_BUCK_BOOST_OVERTEMP_SHUTDOWN,
    FEATURE_RESERVED_90,
    FEATURE_RESERVED_91,
    FEATURE_LDO1_TEMP_PREWARNING,
    FEATURE_LDO1_OVERTEMP_SHUTDOWN,
    FEATURE_LDO2_TEMP_PREWARNING,
    FEATURE_LDO2_OVERTEMP_SHUTDOWN,
    FEATURE_LDO3_TEMP_PREWARNING,
    FEATURE_LDO3_OVERTEMP_SHUTDOWN,
    FEATURE_LDO4_TEMP_PREWARNING,
    FEATURE_LDO4_OVERTEMP_SHUTDOWN,
    FEATURE_PLDO1_TEMP_PREWARNING,
    FEATURE_PLDO1_OVERTEMP_SHUTDOWN,
    FEATURE_PLDO2_TEMP_PREWARNING,
    FEATURE_PLDO2_OVERTEMP_SHUTDOWN,
    FEATURE_VREG_OVP_MONITOR_1,
    FEATURE_VREG_UVLO_MONITOR_1,
    FEATURE_VREG_SAFETY_OVP_MONITOR_1,
    FEATURE_VREG_SAFETY_UVLO_MONITOR_1,
    FEATURE_VREG_1P8_OVP_MONITOR_1,
    FEATURE_VREG_1P8_UVLO_MONITOR_1,
    FEATURE_VREG_OVP_MONITOR_2,
    FEATURE_VREG_UVLO_MONITOR_2,
    FEATURE_VREG_SAFETY_OVP_MONITOR_2,
    FEATURE_VREG_SAFETY_UVLO_MONITOR_2,
    FEATURE_VREG_1P8_OVP_MONITOR_2,
    FEATURE_VREG_1P8_UVLO_MONITOR_2,
    FEATURE_125KHZ_CLOCK,
    FEATURE_250KHZ_CLOCK,
    FEATURE_500KHZ_CLOCK,
    FEATURE_1_25MHZ_CLOCK,
    FEATURE_10MHZ_CLOCK,
    FEATURE_GPI1_INPUT_LEVEL,
    FEATURE_GPI2_INPUT_LEVEL,
    FEATURE_GPI3_INPUT_LEVEL,
    FEATURE_GPI4_INPUT_LEVEL,
    FEATURE_GPI5_INPUT_LEVEL,
    FEATURE_WAKE1_INPUT_LEVEL,
    FEATURE_WAKE2_INPUT_LEVEL,
    FEATURE_RESERVED_122,
    FEATURE_EXT_VMON1_INPUT_DIGITAL_MODE,
    FEATURE_EXT_VMON2_INPUT_DIGITAL_MODE,
    FEATURE_EN_OUT_READBACK_LEVEL,
    FEATURE_GPO1_READBACK_LEVEL,
    FEATURE_GPO2_READBACK_LEVEL,
    FEATURE_GPO3_READBACK_LEVEL,
    FEATURE_GPO4_READBACK_LEVEL,
    FEATURE_NRST_READBACK_LEVEL,
    FEATURE_SAFE_OUT1_READBACK_LEVEL,
    FEATURE_SPI_NCS_INPUT_LEVEL,
    FEATURE_SPI_SCLK_INPUT_LEVEL,
    FEATURE_SPI_SDI_INPUT_LEVEL,
    FEATURE_SPI_SDO_READBACK_LEVEL,
    DMUX_NUM_FEATURES
}
Pmic_DMUXFeatures;

extern
const Pmic_DiagMUXFeatureMapping
Pmic_amuxFeatureMappings[AMUX_NUM_FEATURES];
extern
const Pmic_DiagMUXFeatureMapping
Pmic_dmuxFeatureMappings[DMUX_NUM_FEATURES];

/**
 * @}
 */ /* End of Pmic_CoreStructures */

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @defgroup Pmic_CoreFunctions PMIC Core Functions
 * @{
 * @ingroup Pmic_Core
 * @brief Contains functions used in the core module of PMIC driver.
 */

/**
 * @brief   This function checks whether a specific bit position in a parameter
 * validity value is set. This function is useful for checking the validity of
 * individual parameters within a larger validity value.
 *
 * @param validParamVal Validity parameter value to check.
 * @param bitPos Bit position to check.
 * @return bool True if the specified bit is set, false otherwise.
 *
 * @ingroup Pmic_CoreFunctions
 */
bool pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos);

/**
 * @brief Start a critical section for PMIC operations.
 * This function is responsible for initiating a critical section for PMIC
 * operations. It checks if the critical section start function pointer is not
 * NULL, and if so, invokes the corresponding function to begin the critical
 * section.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 *                        It holds necessary information required to operate on
 * the PMIC.
 * @return void This function does not return any value.
 *              It simply starts the critical section for PMIC operations.
 *
 * @ingroup Pmic_CoreFunctions
 */
void Pmic_criticalSectionStart(const Pmic_CoreHandle_t *pPmicCoreHandle);

/**
 * @brief Stop a critical section for PMIC operations.
 * This function is responsible for ending a critical section for PMIC
 * operations. It checks if the critical section stop function pointer is not
 * NULL, and if so, invokes the corresponding function to end the critical
 * section.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 *                        It holds necessary information required to operate on
 * the PMIC.
 * @return void This function does not return any value.
 *              It simply stops the critical section for PMIC operations.
 *
 * @ingroup Pmic_CoreFunctions
 */
void Pmic_criticalSectionStop(const Pmic_CoreHandle_t *pPmicCoreHandle);

/**
 * @brief Set register lock/unlock configuration.
 * This function is responsible for configuring the register lock/unlock
 * settings of the PMIC based on the provided parameters in the commonCtrlCfg
 * structure. The function initiates a critical section to ensure atomicity of
 * the operation. It then sends the register unlock configuration data to the
 * PMIC via the communication interface. Upon successful transmission of the
 * unlock data, it sends the register lock configuration data. Finally, it
 * terminates the critical section.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param commonCtrlCfg Common control configuration structure containing the
 * register lock/unlock parameters.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 *                      otherwise, it returns an appropriate error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_setRegisterLockUnlock(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const Pmic_CommonCtrlCfg_t commonCtrlCfg);

/**
 * @brief Set counter lock/unlock configuration.
 * This function is used to configure the lock/unlock settings for the PMIC
 * timer counter. It takes the PMIC core handle pointer and a structure
 * containing the common control configuration as input parameters. The common
 * control configuration structure should contain the necessary parameters for
 * setting the counter lock/unlock values. The function first starts a critical
 * section for PMIC operations, then sends the counter unlock sequence to the
 * PMIC, and finally ends the critical section. If the operation is successful,
 * PMIC_ST_SUCCESS is returned; otherwise, an error code indicating the failure
 * is returned.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 *                        It holds necessary information required to operate on
 * the PMIC.
 * @param commonCtrlCfg Common control configuration structure.
 *                      It contains parameters needed to set the counter
 * lock/unlock values.
 * @return int32_t Returns PMIC_ST_SUCCESS if the operation is successful;
 *                 otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_setCounterLockUnlock(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const Pmic_CommonCtrlCfg_t commonCtrlCfg);

/**
 * @brief Get register lock status.
 * This function retrieves the current register lock status from the PMIC.
 * It reads the register lock status byte from the PMIC, extracts the lock
 * status bit, and stores it in the provided common control status structure.
 * The function starts and ends a critical section for PMIC operations.
 * If the operation is successful, PMIC_ST_SUCCESS is returned; otherwise, an
 * error code indicating the failure is returned.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 *                        It holds necessary information required to operate on
 * the PMIC.
 * @param pCommonCtrlStat Pointer to the common control status structure to
 * store the retrieved register lock status.
 * @return int32_t Returns PMIC_ST_SUCCESS if the operation is successful;
 *                 otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getRegLockStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                            Pmic_CommonCtrlStat_t *pCommonCtrlStat);

/**
 * @brief Get timer counter lock status.
 * This function retrieves the current timer counter lock status from the PMIC.
 * It reads the counter lock status byte from the PMIC, extracts the lock status
 * bit, and stores it in the provided common control status structure.
 * The function starts and ends a critical section for PMIC operations.
 * If the operation is successful, PMIC_ST_SUCCESS is returned; otherwise, an
 * error code indicating the failure is returned.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 *                        It holds necessary information required to operate on
 * the PMIC.
 * @param pCommonCtrlStat Pointer to the common control status structure to
 * store the retrieved counter lock status.
 * @return int32_t Returns PMIC_ST_SUCCESS if the operation is successful;
 *                 otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getTmrCntLockStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_CommonCtrlStat_t *pCommonCtrlStat);

/**
 * @brief Get the MCU reset counter value.
 * This function retrieves the value of the MCU (Microcontroller Unit) reset
 * counter from the PMIC (Power Management Integrated Circuit). The MCU reset
 * counter indicates the number of times the MCU has been reset. This
 * information can be useful for diagnostic and monitoring purposes. The
 * function starts by checking the validity of the PMIC core handle and the
 * pointer to store the recovered counter value. It then initiates a critical
 * section to ensure atomicity of the operation and proceeds to read the MCU
 * reset counter value from the PMIC via the communication interface. Finally,
 * it terminates the critical section and returns the retrieved counter value
 * through the `pRecovCntVal` pointer.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pRecovCntVal Pointer to store the recovered counter value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getRstmcuCnt(Pmic_CoreHandle_t *pPmicCoreHandle,
                          uint8_t *pRecovCntVal);

/**
 * @brief Set the state status register.
 * This function allows the user to set the state status register of the PMIC
 * (Power Management Integrated Circuit) based on the provided parameters in the
 * `pCommonCtrlStat` structure. The state status register contains various
 * status bits related to the PMIC state. This function starts by checking the
 * validity of the `pCommonCtrlStat` pointer to ensure that the necessary data
 * is provided. If the pointer is valid, it retrieves the current value of the
 * state status register from the PMIC via the communication interface. Then, it
 * updates the register with the new values provided in the `pCommonCtrlStat`
 * structure, including the MCU reset counter (`rstMcuCnt`) and the MCU reset
 * request flag
 * (`rstMcuRqFlag`). Finally, it writes back the modified state status register
 * to the PMIC. If successful, the function returns `PMIC_ST_SUCCESS`;
 * otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonCtrlStat Pointer to the common control status structure
 * containing the data to set.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 *                    otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_setStateStatReg(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_CommonStateStat_t *pCommonCtrlStat);

/**
 * @brief Set the state control register.
 * This function allows the user to set the state control register of the PMIC
 * (Power Management Integrated Circuit) based on the provided parameters in the
 * `pCommonStateCtrl` structure. The state control register contains control
 * bits related to the PMIC state. This function starts by checking the validity
 * of the `pCommonStateCtrl` pointer to ensure that the necessary data is
 * provided. If the pointer is valid, it retrieves the current value of the
 * state control register from the PMIC via the communication interface. Then,
 * it updates the register with the new value provided in the `state_req` member
 * of the `pCommonStateCtrl` structure. Finally, it writes back the modified
 * state control register to the PMIC. If successful, the function returns
 * `PMIC_ST_SUCCESS`; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonStateCtrl Pointer to the common state control structure
 *                         containing the data to set.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 *                    otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_setStateCtrlReg(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_CommonStateCtrl_t *pCommonStateCtrl);

/**
 * @brief Retrieve the value of the state control register.
 * This function allows the user to retrieve the current value of the state
 * control register from the PMIC (Power Management Integrated Circuit).
 * The retrieved value is stored in the `pCommonStateCtrl` structure provided
 * by the user. The state control register contains control bits related to
 * the PMIC state. The function starts by checking the validity of the
 * `pCommonStateCtrl` pointer to ensure that the necessary data structure
 * is provided. If the pointer is valid, it retrieves the current value
 * of the state control register from the PMIC via the communication interface.
 * Then, it updates the `state_req` member of the `pCommonStateCtrl` structure
 * with the retrieved value. If successful, the function returns
 * `PMIC_ST_SUCCESS`; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonStateCtrl Pointer to the common state control structure to
 *                         store the retrieved value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 *                    otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getStateCtrlReg(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_CommonStateCtrl_t *pCommonStateCtrl);

/**
 * @brief Retrieve the value of the state status register.
 * This function allows the user to retrieve the current value of the state
 * status register from the PMIC (Power Management Integrated Circuit).
 * The retrieved values are stored in the `pCommonCtrlStat` structure provided
 * by the user. The state status register contains various status bits related
 * to the PMIC state, such as reset MCU counter, reset MCU request flag,
 * power delay activation, and the current state. The function starts by
 * checking if the user provided storage for the retrieved data in the
 * `pCommonCtrlStat` pointer to ensure that the necessary data structure
 * is provided. If the storage is provided, it retrieves the current value
 * of the state status register from the PMIC via the communication interface.
 * Then, it updates the corresponding members of the `pCommonCtrlStat`
 * structure with the retrieved values. If successful, the function returns
 * `PMIC_ST_SUCCESS`; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonCtrlStat Pointer to the common control status structure to
 *                        store the retrieved value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 *                    otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getStateStatReg(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_CommonStateStat_t *pCommonCtrlStat);

/**
 * @brief Initialize the PMIC core handle with basic device configuration
 * parameters. This function initializes the PMIC core handle with basic device
 * configuration parameters, such as the PMIC device type and communication
 * mode. It takes the provided PMIC configuration data structure
 * `pPmicConfigData` and updates the corresponding fields in the PMIC core
 * handle `pPmicCoreHandle`. The function starts by verifying the validity of
 * the provided parameters. If the parameters are valid, it updates the PMIC
 * handle with the device type and communication mode information. If
 * successful, the function returns `PMIC_ST_SUCCESS`; otherwise, it returns an
 * error code. This function is typically called during PMIC initialization to
 * configure the basic device parameters.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
static int32_t
Pmic_initCoreHandleBasicDevCfgParams(const Pmic_CoreCfg_t *pPmicConfigData,
                                     Pmic_CoreHandle_t *pPmicCoreHandle);

/**
 * @brief Initialize the PMIC core handle with communication I/O critical
 * section function pointers. This function initializes the PMIC core handle
 * with communication I/O critical section function pointers, which are
 * necessary for handling communication input/output operations in a thread-safe
 * manner. It takes the provided PMIC configuration data structure
 * `pPmicConfigData` and updates the corresponding function pointers in the PMIC
 * core handle `pPmicCoreHandle`. The function verifies the validity of the
 * provided function pointers and assigns them to the appropriate fields in the
 * PMIC handle. If successful, the function returns `PMIC_ST_SUCCESS`;
 * otherwise, it returns an error code. This function is typically called during
 * PMIC initialization to set up the communication interface functions for I/O
 * operations.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
static int32_t Pmic_initCoreHandleCommIOCriticalSectionFns(
    const Pmic_CoreCfg_t *pPmicConfigData, Pmic_CoreHandle_t *pPmicCoreHandle);

/**
 * @brief Validate the presence of the device on the bus.
 * This function validates the presence of the PMIC device on the communication
 * bus by performing read and write operations to the device identification
 * register. It ensures that the device responds correctly and retrieves the
 * device revision information. If the device is not detected or the revision
 * does not match the expected value, the function returns an appropriate error
 * code. Otherwise, it returns `PMIC_ST_SUCCESS`, indicating successful
 * validation of the device presence. This function is typically called during
 * PMIC initialization to verify the proper functioning and presence of the
 * device on the bus.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_validateDevOnBus(Pmic_CoreHandle_t *pPmicCoreHandle);

/**
 * @brief Update subsystem information and validate main Q&A communication
 * interface read/write. This function updates subsystem information in the PMIC
 * core handle based on the provided configuration data and validates the main
 * Q&A communication interface read/write operations. It retrieves specific
 * register values from the PMIC to verify the proper functioning of the
 * communication interface. If successful, it updates the driver initialization
 * status in the PMIC handle. If any error occurs during the process, an
 * appropriate error code is returned. This function is crucial for initializing
 * the PMIC core and ensuring proper communication with the device.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * updated.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
static int32_t Pmic_updateSubSysInfoValidateMainQaCommIFRdWr(
    const Pmic_CoreCfg_t *pPmicConfigData, Pmic_CoreHandle_t *pPmicCoreHandle);

/**
 * @brief Initialize the PMIC core.
 * This function initializes the PMIC core based on the provided configuration
 * data. It sets up the PMIC core handle with the necessary parameters and
 * function pointers for communication and critical section management. If the
 * initialization process completes successfully, the function returns
 * `PMIC_ST_SUCCESS`; otherwise, it returns an error code. This function is a
 * fundamental step in the initialization of the PMIC core and should be called
 * before any other PMIC-related operations.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_init(const Pmic_CoreCfg_t *pPmicConfigData,
                  Pmic_CoreHandle_t *pPmicCoreHandle);

/**
 * @brief Deinitialize the PMIC core.
 * This function deinitializes the PMIC core by resetting all the relevant
 * fields and function pointers in the PMIC core handle structure. It clears any
 * resources allocated during the initialization process. If the
 * deinitialization process completes successfully, the function returns
 * `PMIC_ST_SUCCESS`; otherwise, it returns an error code. This function is
 * essential for properly shutting down the PMIC core and releasing any acquired
 * resources.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_deinit(Pmic_CoreHandle_t *pPmicCoreHandle);

/**
 * @brief Get the address of the scratch pad register.
 * This function retrieves the address of the specified scratch pad register
 * based on the provided register ID. The scratch pad registers are used for
 * storing temporary data or configuration values. This function is typically
 * called before reading from or writing to a scratch pad register.
 *
 * @param scratchPadRegId Scratch pad register ID.
 * @param pRegAddr Pointer to store the address of the scratch pad register.
 * @return void
 *
 * @ingroup Pmic_CoreFunctions
 */
static void Pmic_getScratchPadRegAddr(uint8_t scratchPadRegId,
                                      uint8_t *pRegAddr);

/**
 * @brief Set a value to the scratch pad register.
 * This function sets a value to the specified scratch pad register. Scratch pad
 * registers are used for storing temporary data or configuration values. The
 * function ensures proper communication with the PMIC device and returns an
 * appropriate status code indicating the success or failure of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param scratchPadRegId Scratch pad register ID.
 * @param data Data to be written to the scratch pad register.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_setScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t scratchPadRegId,
                                const uint8_t data);

/**
 * @brief Get a value from the scratch pad register.
 * This function retrieves a value from the specified scratch pad register.
 * Scratch pad registers are used for storing temporary data or configuration
 * values. The function ensures proper communication with the PMIC device and
 * returns an appropriate status code indicating the success or failure of the
 * operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param scratchPadRegId Scratch pad register ID.
 * @param pData Pointer to store the retrieved data from the scratch pad
 * register.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t scratchPadRegId, uint8_t *pData);

/**
 * @brief Enable or disable spread spectrum.
 * This function enables or disables spread spectrum based on the provided
 * configuration. Spread spectrum is a technique used to reduce electromagnetic
 * interference (EMI) by modulating the frequency or phase of a signal. The
 * function sets the spread spectrum configuration in the PMIC device by
 * modifying the appropriate register. If the operation is successful, it
 * returns PMIC_ST_SUCCESS; otherwise, it returns an error code indicating the
 * failure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param commonCtrlCfg Spread spectrum configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_spreadSpectrumEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const Pmic_CommonCtrlCfg_t commonCtrlCfg);

/**
 * @brief Get the spread spectrum enable status.
 * This function retrieves the status of spread spectrum. It reads the current
 * spread spectrum configuration from the PMIC device and stores the result in
 * the provided configuration structure. Spread spectrum can be either enabled
 * or disabled, and the function updates the structure accordingly. If the
 * operation is successful, it returns PMIC_ST_SUCCESS; otherwise, it returns an
 * error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlCfg Pointer to store the spread spectrum configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getSpreadSpectrumEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     Pmic_CommonCtrlCfg_t *pCommonCtrlCfg);

/**
 * @brief Set the safe output pin configuration.
 * This function sets the configuration for the safe output pin. The safe output
 * pin is typically used to indicate fault conditions or other status
 * information from the PMIC device. The function configures the behavior of the
 * safe output pin based on the provided configuration structure. If the
 * operation is successful, it returns PMIC_ST_SUCCESS; otherwise, it returns an
 * error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param commonCtrlCfg Safe output pin configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_setEnableSafeOutCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const Pmic_CommonCtrlCfg_t commonCtrlCfg);

/**
 * @brief Get the safe output pin configuration.
 * This function retrieves the configuration of the safe output pin. It reads
 * the current configuration from the PMIC device and stores the result in the
 * provided configuration structure. The safe output pin can be configured to
 * different signal levels, and the function updates the structure accordingly.
 * If the operation is successful, it returns PMIC_ST_SUCCESS; otherwise, it
 * returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlCfg Pointer to store the safe output pin configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getSafeOutPinCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_CommonCtrlCfg_t *pCommonCtrlCfg);

/**
 * @brief Set the safe state timeout configuration.
 * This function sets the safe state timeout configuration, which determines
 * the duration after which the PMIC enters a safe state in case of certain
 * fault conditions. The safe state timeout value is configured based on the
 * provided configuration structure. The function reads the current register
 * value, updates the relevant bits with the new timeout value, and writes
 * back to the PMIC device. If the operation is successful, it returns
 * PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param safeCfg Safe state timeout configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_setSafeStateTimeoutCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_SafeStateCfg_t *safeCfg);

/**
 * @brief Get the safe state timeout configuration.
 * This function retrieves the safe state timeout configuration from the PMIC
 * device. It reads the current register value, extracts the timeout value,
 * and stores it in the provided configuration structure. If the operation is
 * successful, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param safeCfg Pointer to store the safe state timeout configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getSafeStateTimeoutCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_SafeStateCfg_t *safeCfg);

/**
 * @brief Set the safe state threshold configuration.
 * This function sets the safe state threshold configuration, which determines
 * the threshold at which the PMIC enters a safe state. The safe state threshold
 * value is configured based on the provided configuration structure. The
 * function reads the current register value, updates the relevant bits with the
 * new threshold value, and writes back to the PMIC device. If the operation is
 * successful, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param safeCfg Safe state threshold configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_setSafeStateThresholdCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      Pmic_SafeStateCfg_t *safeCfg);

/**
 * @brief Get the safe state threshold configuration.
 * This function retrieves the safe state threshold configuration from the PMIC
 * device. It reads the current register value, extracts the threshold value,
 * and stores it in the provided configuration structure. If the operation is
 * successful, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param safeCfg Pointer to store the safe state threshold configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getSafeStateThresholdCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      Pmic_SafeStateCfg_t *safeCfg);

/**
 * @brief Set the common control configuration.
 * This function sets the common control configuration, which includes various
 * parameters such as spread spectrum enable, safe output pin configuration,
 * and register lock/unlock configuration. The function checks the validity of
 * each parameter and calls corresponding sub-functions to apply the
 * configurations. If the operation is successful, it returns PMIC_ST_SUCCESS;
 * otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param CommonCtrlCfg Common control configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_setCommonCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const Pmic_CommonCtrlCfg_t CommonCtrlCfg);

/**
 * @brief Get the common control configuration.
 * This function retrieves the common control configuration from the PMIC
 * device. It reads various control registers to obtain the configuration
 * parameters, including spread spectrum enable status and safe output pin
 * configuration. The retrieved configuration is stored in the provided
 * structure. If the operation is successful, it returns PMIC_ST_SUCCESS;
 * otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlCfg Pointer to store the common control configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getCommonCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_CommonCtrlCfg_t *pCommonCtrlCfg);

/**
 * @brief Set the diagnostics output pin control configuration.
 * This function sets the diagnostics output pin control configuration.
 * It configures the control registers related to the diagnostics output pins
 * based on the provided configuration structure. The function handles the
 * configuration for AMUX_OUT and DMUX_OUT pins. If the operation is successful,
 * it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param DiagOutCfgCtrl Diagnostics output pin control configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_setDiagOutCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const Pmic_DiagOutCfgCtrl_t DiagOutCfgCtrl);

/**
 * @brief Get the diagnostics output pin control configuration.
 * This function retrieves the diagnostics output pin control configuration
 * from the PMIC device. It reads the control registers related to the
 * diagnostics output pins and stores the configuration in the provided
 * structure. If the operation is successful, it returns PMIC_ST_SUCCESS;
 * otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pDiagOutCfgCtrl Pointer to store the diagnostics output pin control
 * configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getDiagOutCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_DiagOutCfgCtrl_t *pDiagOutCfgCtrl);

/**
 * @brief Get the bit field shift and mask values for a given pin type.
 * This function retrieves the bit field shift and mask values from the PMIC
 * device registers based on the specified pin type. It is used internally to
 * decode the register values related to various pin types.
 *
 * @param pinType The type of pin for which bit field values are required.
 * @param pBitShift Pointer to store the bit field shift value.
 * @param pBitMask Pointer to store the bit field mask value.
 *
 * @ingroup Pmic_CoreFunctions
 */
static void Pmic_getPinTypeRegBitFields(const uint8_t pinType,
                                        uint8_t *pBitShift, uint8_t *pBitMask);

/**
 * @brief Get the pin value.
 * This function retrieves the value of the specified pin from the PMIC device.
 * It reads the status register associated with the pin type and extracts the
 * pin value using the corresponding bit shift and mask. The retrieved pin value
 * is stored in the provided pointer. If the operation is successful, it returns
 * PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pinType Type of the pin for which the value is requested.
 * @param pPinValue Pointer to store the retrieved pin value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getPinValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                         const uint8_t pinType, uint8_t *pPinValue);

/**
 * @brief Get the status of the safe output 1 pin.
 * This function retrieves the status of the safe output 1 pin from the PMIC
 * device. It reads the status register associated with the safe output 1 pin
 * and extracts the status bit using the corresponding bit shift and mask.
 * The retrieved status is stored in the provided pointer. If the operation is
 * successful, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
static int32_t Pmic_getSafeOut1Stat(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_CommonCtrlStat_t *pCommonCtrlStat);

/**
 * @brief Get the status of the nRST pin.
 * This function retrieves the status of the nRST pin from the PMIC device.
 * It reads the status register associated with the nRST pin and extracts the
 * status bit using the corresponding bit shift and mask. The retrieved status
 * is stored in the provided pointer. If the operation is successful, it returns
 * PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
static int32_t Pmic_getNRstPinStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_CommonCtrlStat_t *pCommonCtrlStat);

/**
 * @brief Get the status of the EN_OUT pin.
 * This function retrieves the status of the EN_OUT pin from the PMIC device.
 * It reads the status register associated with the EN_OUT pin and extracts the
 * status bit using the corresponding bit shift and mask. The retrieved status
 * is stored in the provided pointer. If the operation is successful, it returns
 * PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
static int32_t Pmic_getEnOutPinStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_CommonCtrlStat_t *pCommonCtrlStat);

/**
 * @brief Get the status of the EN_OUT, nRST, and safe output 1 pins.
 * This function retrieves the status of the EN_OUT, nRST, and safe output 1
 * pins from the PMIC device. It reads the corresponding status registers and
 * extracts the status bits using the appropriate bit shifts and masks.
 * The retrieved status values are stored in the provided structure. If the
 * operation is successful, it returns PMIC_ST_SUCCESS; otherwise, it returns
 * an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
static int32_t
Pmic_getEnOutNrstSafeOut1PinStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_CommonCtrlStat_t *pCommonCtrlStat);

/**
 * @brief Get the common status.
 * This function retrieves the common control status from the PMIC device.
 * It reads various status registers to obtain the status of register lock,
 * EN_OUT pin, nRST pin, and safe output 1 pin. The retrieved status values
 * are stored in the provided structure. If the operation is successful, it
 * returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getCommonStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                           Pmic_CommonCtrlStat_t *pCommonCtrlStat);

int32_t Pmic_setDiagAMUXFeatureCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_AMUXFeatures feature);

int32_t Pmic_getDiagAMUXFeatureCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   uint32_t *feature);

int32_t Pmic_setDiagDMUXFeatureCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_DMUXFeatures feature);

int32_t Pmic_getDiagDMUXFeatureCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   uint32_t *feature);

int32_t Pmic_setAmuxDmuxPinCtrlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const Pmic_DiagOutCfgCtrl_t diagoutCfgCtrl);

int32_t Pmic_getAmuxDmuxPinCtrlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_DiagOutCfgCtrl_t *pDiagOutCfgCtrl);

int32_t Pmic_setDiagMUXSelectionCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_DiagOutCfgCtrl_t diagoutCfgCtrl);

int32_t Pmic_getDiagMUXSelectionCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_DiagOutCfgCtrl_t *diagoutCfgCtrl);

/**
 * @}
 */ /* End of Pmic_CoreFunctions */

/**
 * @}
 */ /* End of Pmic_Core */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_CORE_H_ */

/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *  \addtogroup DRV_PMIC_POWER_MODULE
 *
 *  @{
 */

/**
 * \file   pmic_power_tps6594x.h
 *
 * \brief  PMIC TPS6594x Leo PMIC Power Resources Driver API/interface file.
 *
 */

#ifndef PMIC_POWER_TPS6594X_H_
#define PMIC_POWER_TPS6594X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_Tps6594xLeo_Power_ResourceType
 *  \name   PMIC Power Resource Type for LEO TPS6594x
 *
 *  @{
 */
#define PMIC_TPS6594X_POWER_RESOURCE_TYPE_VCCA              (0U)
#define PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK              (1U)
#define PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO               (2U)
/*  @} */

/**
 *  \anchor Pmic_Tps6594xLeo_Power_Resource
 *  \name   PMIC Power Resources for LEO TPS6594x
 *
 *  @{
 */
#define PMIC_TPS6594X_POWER_SOURCE_VCCA   \
            ((((uint16_t)PMIC_TPS6594X_POWER_RESOURCE_TYPE_VCCA  << 8U) | 0x0U))
#define PMIC_TPS6594X_REGULATOR_BUCK1     \
            ((((uint16_t)PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK  << 8U) | 0x1U))
#define PMIC_TPS6594X_REGULATOR_BUCK2     \
            ((((uint16_t)PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK  << 8U) | 0x2U))
#define PMIC_TPS6594X_REGULATOR_BUCK3     \
            ((((uint16_t)PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK  << 8U) | 0x3U))
#define PMIC_TPS6594X_REGULATOR_BUCK4     \
            ((((uint16_t)PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK  << 8U) | 0x4U))
#define PMIC_TPS6594X_REGULATOR_BUCK5     \
            ((((uint16_t)PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK  << 8U) | 0x5U))
#define PMIC_TPS6594X_REGULATOR_LDO1      \
            ((((uint16_t)PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO  << 8U)  | 0x6U))
#define PMIC_TPS6594X_REGULATOR_LDO2      \
            ((((uint16_t)PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO  << 8U)  | 0x7U))
#define PMIC_TPS6594X_REGULATOR_LDO3      \
            ((((uint16_t)PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO  << 8U)  | 0x8U))
#define PMIC_TPS6594X_REGULATOR_LDO4      \
            ((((uint16_t)PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO  << 8U)  | 0x9U))
/*  @} */

/**
 *  \anchor Pmic_TPS6594x_Regulator_RV_Check
 *  \name   PMIC Residual voltage check Enable/Disable
 *
 *  Valid only for BUCK and LDO regulators
 *
 *  @{
 */
/** \brief Used to enable the residual voltage check */
#define PMIC_TPS6594X_REGULATOR_RV_SEL_ENABLE              (0x1U)
/** \brief Used to disable the residual voltage check */
#define PMIC_TPS6594X_REGULATOR_RV_SEL_DISABLE             (0x0U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Buck_Pull_Down_Resistor
 *  \name   PMIC Pull-down resistor Enable/Disable for BUCK Regulator.
 *
 *  @{
 */
/** \brief Used to enable the pull down resistor for BUCK regulator */
#define PMIC_TPS6594X_REGULATOR_BUCK_PLDN_ENABLE           (0x1U)
/** \brief Used to disable the pull down resistor for BUCK regulator */
#define PMIC_TPS6594X_REGULATOR_BUCK_PLDN_DISABLE          (0x0U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Vmon_Enable
 *  \name   PMIC Voltage monitor Enable/Disable for BUCK/LDO/VCCA.\
 *          Enable/Disable OV and UV comparators for LDO/VCCA. \
 *          Enable/Disable OV, UV, SC and ILIM for BUCK
 *
 *  @{
 */
/** \brief Used to disable the voltage monitor */
#define PMIC_TPS6594X_VMON_DISABLE          (0x0U)
/** \brief Used to enable the voltage monitor */
#define PMIC_TPS6594X_VMON_ENABLE           (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Regulator_Buck_Vout_Sel
 *  \name   PMIC Select output voltage register for BUCK.
 *
 *  Valid only for BUCK Regulator
 *
 *  @{
 */
/** \brief Used to select VOUT2 register for voltage selection */
#define PMIC_TPS6594X_REGULATOR_BUCK_VOUT_SEL_VOUT2        (bool)true
/** \brief Used to select VOUT1 register for voltage selection */
#define PMIC_TPS6594X_REGULATOR_BUCK_VOUT_SEL_VOUT1        (bool)false
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Regulator_Pwm_Pfm_Mode
 *  \name   PMIC Select between Automatic transitions between PFM and PWM  \
 *          modes OR Forced to PWM operation.
 *
 *  Valid only for BUCK regulators.
 *
 *  @{
 */
/** \brief Used to select PWM mode */
#define PMIC_TPS6594X_REGULATOR_PWM_MODE                   (0x1U)
/** \brief Used to select Automatic transition between PFM and PWM modes */
#define PMIC_TPS6594X_REGULATOR_AUTO_PWM_PFM_MODE          (0x0U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Regulator_Pwm_Mp_Mode
 *  \name   PMIC Select between multi-phase operation OR \
 *          AUTO mode with Automatic phase adding and shedding.
 *
 *  Valid only for BUCK regulators.
 *
 *  @{
 */
/** \brief Used to select multi-phase operation */
#define PMIC_TPS6594X_REGULATOR_PWM_MP_MODE                (0x1U)
/** \brief Used to select Automatic phase adding and shedding mode */
#define PMIC_TPS6594X_REGULATOR_AUTO_PHASE_MODE            (0x0U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Power_Regulator_enable
 *  \name   PMIC Enable/Disable BUCK/LDO Regulators.
 *
 *  @{
 */
/** \brief Used to enable the BUCK or LDO regulator */
#define PMIC_TPS6594X_REGULATOR_ENABLE                     (0x1U)
/** \brief Used to disable the BUCK or LDO regulator */
#define PMIC_TPS6594X_REGULATOR_DISABLE                    (0x0U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Regulator_Ldo_Slow_Ramp
 *  \name   PMIC Enable/Disable Slow Ramp for LDO regulator
 *
 *  Valid only for LDO regulators.
 *
 *  @{
 */
/** \brief Used to enable slow ramp for LDO */
#define PMIC_TPS6594X_REGULATOR_LDO_SLOW_RAMP_ENABLE       (0x1U)
/** \brief Used to disable slow ramp for LDO */
#define PMIC_TPS6594X_REGULATOR_LDO_SLOW_RAMP_DISABLE      (0x0U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Regulator_Ldo_Mode
 *  \name   PMIC Selects the LDO Bypass or Linear Regulator mode
 *  Valid only for LDO1/LDO2/LDO3 regulators
 *
 *  @{
 */
/** \brief Used to set to bypass mode */
#define PMIC_TPS6594X_REGULATOR_LDO_BYPASS_MODE            (0x1U)
/** \brief Used to  set to linear regulator mode */
#define PMIC_TPS6594X_REGULATOR_LDO_LINEAR_REGULATOR_MODE  (0x0U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Vmon_DeglitchTime_Sel
 *  \name   PMIC Deglitch time select for BUCKx_VMON/LDOx_VMON/VCCA_VMON
 *
 *  @{
 */
/** \brief Used to select the degitch time as 4 usec */
#define PMIC_TPS6594X_POWER_RESOURCE_DEGLITCH_SEL_4US      (0x0U)
/** \brief Used to select the degitch time as 20 usec */
#define PMIC_TPS6594X_POWER_RESOURCE_DEGLITCH_SEL_20US     (0x1U)
/*  @} */

/**
 *  \anchor Pmic_TPS6594x_VccaPowerGoodLimit
 *  \name   PMIC Powergood level for VCCA
 *
 *  @{
 */
/** \brief Used to select the powergood level for VCCA to be 3.3v */
#define PMIC_TPS6594X_VCCA_PG_3V3_LEVEL                    (0x0U)
/** \brief Used to select the powergood level for VCCA to be 5v */
#define PMIC_TPS6594X_VCCA_PG_5V_LEVEL                     (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Buck_Current_Limit
 *  \name   PMIC Switch Peak Current limit for BUCK Regulator
 *
 *  @{
 */
/** \brief Used to configure BUCK current limit as 2.5 Ampere */
#define PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_2A5     (0x2U)
/** \brief Used to configure BUCK current limit as 3.5 Ampere */
#define PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_3A5     (0x3U)
/** \brief Used to configure BUCK current limit as 4.5 Ampere */
#define PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_4A5     (0x4U)
/** \brief Used to configure BUCK current limit as 5.5 Ampere */
#define PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_5A5     (0x5U)
/*  @} */

/**
 *  \anchor Pmic_TPS6594x_Buck_Slew_Rate
 *  \name   PMIC Output voltage slew rate for BUCK Regulator
 *
 *  @{
 */
/** \brief Used to configure BUCK current limit as 30mv */
#define PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_33MV     (0x0U)
/** \brief Used to configure BUCK current limit as 20mv */
#define PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_20MV     (0x1U)
/** \brief Used to configure BUCK current limit as 10mv */
#define PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_10MV     (0x2U)
/** \brief Used to configure BUCK current limit as 5mv */
#define PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_05MV     (0x3U)
/** \brief Used to configure BUCK current limit as 2.5mv */
#define PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_2MV5     (0x4U)
/** \brief Used to configure BUCK current limit as 1.3mv */
#define PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_1MV3     (0x5U)
/** \brief Used to configure BUCK current limit as 0.63mv */
#define PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_0MV63    (0x6U)
/** \brief Used to configure BUCK current limit as 0.31mv */
#define PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_0MV31    (0x7U)
/*  @} */

/**
 *  \anchor Pmic_TPS6594x_Ldo_Pldn_Resistor_Val
 *  \name   PMIC Output pull-down resistor value for LDO Regulator.
 *
 *  @{
 */
/** \brief Used to select the pull down resistor value as 50KOhm */
#define PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_50KOHM        (0x0U)
/** \brief Used to select the pull down resistor value as 125Ohm */
#define PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_125OHM        (0x1U)
/** \brief Used to select the pull down resistor value as 250Ohm */
#define PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_250OHM        (0x2U)
/** \brief Used to select the pull down resistor value as 500Ohm */
#define PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_500OHM        (0x3U)
/*  @} */

/**
 *  \anchor Pmic_TPS6594x_Ldo_RV_Timeout
 *  \name   PMIC Selects the LDO Residual voltage check Timeout value.
 *
 *  Valid only for LDO regulators (LDO1, 2 and 3 only).
 *
 *  @{
 */
/** \brief Used to set timeout to 0.5ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_0MS5              (0U)
/** \brief Used to set timeout to 1ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_1MS               (1U)
/** \brief Used to set timeout to 1.5ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_1MS5              (2U)
/** \brief Used to set timeout to 2ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_2MS               (3U)
/** \brief Used to set timeout to 2.5ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_2MS5              (4U)
/** \brief Used to set timeout to 3ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_3MS               (5U)
/** \brief Used to set timeout to 3.5ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_3MS5              (6U)
/** \brief Used to set timeout to 4ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_4MS               (7U)
/** \brief Used to set timeout to 2ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_2MS0              (8U)
/** \brief Used to set timeout to 4ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_4MS0              (9U)
/** \brief Used to set timeout to 6ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_6MS               (10U)
/** \brief Used to set timeout to 8ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_8MS               (11U)
/** \brief Used to set timeout to 10ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_10MS              (12U)
/** \brief Used to set timeout to 12ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_12MS              (13U)
/** \brief Used to set timeout to 14ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_14MS              (14U)
/** \brief Used to set timeout to 16ms */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_16MS              (15U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Pg_Ov_Uv_Threshold_lvl
 *  \name   PMIC Power Good Over/Under voltage threshold level for BUCK/LDO/VCCA
 *
 *  For LDO/BUCK - Over/Under Volatge thershold level are +x1 mv/ +x2 %
 *  or -x1 mv/ -x2 % respectively.
 *  For VCCA - Over/Under Volatge thershold level are +x2 % or -x2 %
 *  respectively.
 *  @{
 */
/** \brief Used to select over/under voltage threshold level as +/-30mv or
 *         +/-3%
 */
#define PMIC_TPS6594X_PG_OV_UV_THRESHOLD_LVL_30_OR_3       (0U)
/** \brief Used to select over/under voltage threshold level as +/-35mv or
 *         +/-3.5%
 */
#define PMIC_TPS6594X_PG_OV_UV_THRESHOLD_LVL_35_OR_3P5     (1U)
/** \brief Used to select over/under voltage threshold level as +/-40mv or
 *         +/-4%
 */
#define PMIC_TPS6594X_PG_OV_UV_THRESHOLD_LVL_40_OR_4       (2U)
/** \brief Used to select over/under voltage threshold level as +/-50mv or
 *         +/-5%
 */
#define PMIC_TPS6594X_PG_OV_UV_THRESHOLD_LVL_50_OR_5       (3U)
/** \brief Used to select over/under voltage threshold level as +/-60mv or
 *         +/-6%
 */
#define PMIC_TPS6594X_PG_OV_UV_THRESHOLD_LVL_60_OR_6       (4U)
/** \brief Used to select over/under voltage threshold level as +/-70mv or
 *         +/-7%
 */
#define PMIC_TPS6594X_PG_OV_UV_THRESHOLD_LVL_70_OR_7       (5U)
/** \brief Used to select over/under voltage threshold level as +/-80mv or
 *         +/-8%
 */
#define PMIC_TPS6594X_PG_OV_UV_THRESHOLD_LVL_80_OR_8       (6U)
/** \brief Used to select over/under voltage threshold level as +/-100mv or
 *         +/-10%
 */
#define PMIC_TPS6594X_PG_OV_UV_THRESHOLD_LVL_100_OR_10     (7U)
/*  @} */

/**
 *  \anchor Pmic_TPS6594x_Power_Rail_Sel
 *  \name   PMIC Rail group selection for all power resources.
 *
 *  @{
 */
/** \brief Used to select rail group as no group
 *         Note: Software can prevent the SoC/MCU Power Error Handling, which
 *         normally shuts down the SOC/MCU power rails, by setting the relevent
 *         regulators to PMIC_TPS6594X_POWER_RAIL_SEL_NONE
 */
#define PMIC_TPS6594X_POWER_RAIL_SEL_NONE                  (0x0U)
/** \brief Used to select rail group as MCU rail group */
#define PMIC_TPS6594X_POWER_RAIL_SEL_MCU                   (0x1U)
/** \brief Used to select rail group as SOC rail group */
#define PMIC_TPS6594X_POWER_RAIL_SEL_SOC                   (0x2U)
/** \brief Used to select rail group as other rail group */
#define PMIC_TPS6594X_POWER_RAIL_SEL_OTHER                 (0x3U)
/* @} */

/**
 *  \anchor Pmic_Tps6594xLeo_Pgood_SourceType
 *  \name   PMIC Power-Good source Type for LEO TPS6594x
 *
 *  @{
 */
#define PMIC_TPS6594X_PGOOD_SOURCE_TYPE_VCCA              (0U)
#define PMIC_TPS6594X_PGOOD_SOURCE_TYPE_BUCK              (1U)
#define PMIC_TPS6594X_PGOOD_SOURCE_TYPE_NRSTOUT           (2U)
#define PMIC_TPS6594X_PGOOD_SOURCE_TYPE_NRSTOUT_SOC       (3U)
#define PMIC_TPS6594X_PGOOD_SOURCE_TYPE_TDIE              (4U)
#define PMIC_TPS6594X_PGOOD_SOURCE_TYPE_LDO               (5U)
/* VMON type is not supported by TPS6594x Leo PMIC */
/*  @} */

/**
 *  \anchor Pmic_Tps6594xLeo_Pgood_Source
 *  \name   PMIC  Power-Good sources for LEO TPS6594x
 *
 *  @{
 */
#define PMIC_TPS6594X_PGOOD_SOURCE_VCCA        \
            ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_VCCA  << 8U) | 0U))
#define PMIC_TPS6594X_PGOOD_SOURCE_BUCK1       \
            ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_BUCK  << 8U) | 1U))
#define PMIC_TPS6594X_PGOOD_SOURCE_BUCK2       \
            ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_BUCK  << 8U) | 2U))
#define PMIC_TPS6594X_PGOOD_SOURCE_BUCK3       \
            ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_BUCK  << 8U) | 3U))
#define PMIC_TPS6594X_PGOOD_SOURCE_BUCK4       \
            ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_BUCK  << 8U) | 4U))
#define PMIC_TPS6594X_PGOOD_SOURCE_BUCK5       \
            ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_BUCK  << 8U) | 5U))
#define PMIC_TPS6594X_PGOOD_SOURCE_NRSTOUT     \
            ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_NRSTOUT << 8U) | 6U))
#define PMIC_TPS6594X_PGOOD_SOURCE_NRSTOUT_SOC \
        ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_NRSTOUT_SOC  << 8U) | 7U))
#define PMIC_TPS6594X_PGOOD_SOURCE_TDIE        \
            ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_TDIE   << 8U) | 8U))
#define PMIC_TPS6594X_PGOOD_SOURCE_LDO1        \
            ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_LDO   << 8U) | 9U))
#define PMIC_TPS6594X_PGOOD_SOURCE_LDO2        \
            ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_LDO   << 8U) | 10U))
#define PMIC_TPS6594X_PGOOD_SOURCE_LDO3        \
            ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_LDO   << 8U) | 11U))
#define PMIC_TPS6594X_PGOOD_SOURCE_LDO4        \
            ((((uint16_t)PMIC_TPS6594X_PGOOD_SOURCE_TYPE_LDO   << 8U) | 12U))
/*  @} */

/**
 *  \anchor Pmic_TPS6594x_Power_Good_Window
 *  \name Type of voltage monitoring for PGOOD signal:
 *
 *  @{
 */
/** \brief Only undervoltage is monitored */
#define PMIC_TPS6594X_POWER_GOOD_UV_MONITOR_ENABLE         (0x0U)
/** \brief Both undervoltage and overvoltage are monitored */
#define PMIC_TPS6594X_POWER_GOOD_UV_OV_MONITOR_ENABLE      (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Power_Good_Polarity
 *  \name PGOOD signal polarity
 *
 *  @{
 */
/** \brief PGOOD signal is high when monitored inputs are valid */
#define PMIC_TPS6594X_POWER_PGOOD_POL_HIGH                 (0x0U)
/** \brief PGOOD signal is low when monitored inputs are valid */
#define PMIC_TPS6594X_POWER_PGOOD_POL_LOW                  (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Power_Good_Nrstout_Soc
 *  \name PGOOD signal source control from nRSTOUT_SOC pin
 *
 *  @{
 */
/** \brief Signal is Masked */
#define PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT_SOC_MASKED   (0x0U)
/** \brief nRSTOUT_SOC pin low state forces PGOOD signal to low */
#define PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT_SOC          (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Power_Good_Nrstout
 *  \name PGOOD signal source control from nRSTOUT pin
 *
 *  @{
 */
/** \brief Signal is Masked */
#define PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT_MASKED       (0x0U)
/** \brief nRSTOUT pin low state forces PGOOD signal to low */
#define PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT              (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Power_Good_Thermal_Warn
 *  \name PGOOD signal source control from thermal warning
 *
 *  @{
 */
/** \brief Signal is Masked */
#define PMIC_TPS6594X_POWER_PGOOD_SEL_TDIE_WARN_MASKED     (0x0U)
/** \brief Thermal warning affecting to PGOOD signal */
#define PMIC_TPS6594X_POWER_PGOOD_SEL_TDIE_WARN            (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Power_Good_Vcca
 *  \name PGOOD signal source control from VCCA monitoring
 *
 *  @{
 */
/** \brief Signal is Masked */
#define PMIC_TPS6594X_POWER_PGOOD_SEL_VCCA_DISABLE         (0x0U)
/** \brief VCCA OV/UV threshold affecting PGOOD signal */
#define PMIC_TPS6594X_POWER_PGOOD_SEL_VCCA_ENABLE          (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Power_Good_Regulator_Signal
 *  \name PGOOD signal source control for BUCK and LDO monitoring
 *
 *  @{
 */
/** \brief Signal is Masked */
#define PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_MASKED           (0x0U)
/** \brief Powergood threshold voltage */
#define PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_VOLTAGE          (0x1U)
/** \brief Powergood threshold voltage AND current limit */
#define PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_VOLTAGE_CURRENT  (0x2U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Power_Current_Status
 *  \name Status whether the output current is above/below current limit level
 *
 *  @{
 */
/** \brief Status indicating that output current is above current limit
           level. */
#define PMIC_TPS6594X_POWER_CURRENT_LIMIT_STATUS_ABOVE_LIMIT         (0x0U)
/** \brief Status indicating that output current is below current limit
           level. */
#define PMIC_TPS6594X_POWER_CURRENT_LIMIT_STATUS_BELOW_LIMIT         (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Regulator_Under_Voltage_Status
 *  \name Status whether the output voltage is above/below voltage threshold \
 *        for LDO and BUCK
 *
 *  @{
 */
/** \brief Status indicating that output voltage is above under-voltage
           threshold
*/
#define PMIC_TPS6594X_REGULATOR_OUTPUT_UNDER_VOLTAGE_STATUS_ABOVE_UV      (0x0U)
/** \brief Status indicating that output voltage is below under-voltage
           threshold
*/
#define PMIC_TPS6594X_REGULATOR_OUTPUT_UNDER_VOLTAGE_STATUS_BELOW_UV      (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Vcca_Under_Voltage_Status
 *  \name Status whether the input voltage is above/below under-voltage level \
 *        for VCCA/VMON
 *
 *  @{
 */
/** \brief Status indicating that input voltage is above under-voltage
level */
#define PMIC_TPS6594X_VCCA_INPUT_UNDER_VOLTAGE_STATUS_ABOVE_UV         (0x0U)
/** \brief Status indicating that input voltage is below under-voltage
level */
#define PMIC_TPS6594X_VCCA_INPUT_UNDER_VOLTAGE_STATUS_BELOW_UV         (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Regulator_Over_Voltage_Status
 *  \name Status whether the output voltage is above/below voltage threshold \
 *        for LDO and BUCK
 *
 *  @{
 */
/** \brief Status indicating that output voltage is above over-voltage
           threshold
*/
#define PMIC_TPS6594X_REGULATOR_OUTPUT_OVER_VOLTAGE_STATUS_ABOVE_OV       (0x0U)
/** \brief Status indicating that output voltage is below over-voltage
           threshold
*/
#define PMIC_TPS6594X_REGULATOR_OUTPUT_OVER_VOLTAGE_STATUS_BELOW_OV       (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Vcca_Over_Voltage_Status
 *  \name Status whether the input voltage is above/below over-voltage level \
 *        for VCCA/VMON
 *
 *  @{
 */
/** \brief Status indicating that input voltage is above over-voltage
level */
#define PMIC_TPS6594X_VCCA_INPUT_OVER_VOLTAGE_STATUS_ABOVE_OV       (0x0U)
/** \brief Status indicating that input voltage is below over-voltage
level */
#define PMIC_TPS6594X_VCCA_INPUT_OVER_VOLTAGE_STATUS_BELOW_OV       (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Vcca_Voltage_Status
 *  \name Status whether the voltage is above/below over-voltage protection \
 *        level for VCCA
 *
 *  @{
 */
/** \brief Status indicating that voltage is above over-voltage protection
level */
#define PMIC_TPS6594X_VCCA_OVER_VOLTAGE_LVL_STATUS_ABOVE_OV       (0x0U)
/** \brief Status indicating that voltage is below over-voltage protection
level */
#define PMIC_TPS6594X_VCCA_OVER_VOLTAGE_LVL_STATUS_BELOW_OV       (0x1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Power_Thermal_Shutdown_Level
 *  \name   PMIC Thermal shutdown threshold level.
 *
 *  @{
 */
/** \brief Used to set the Thermal shutdown threshold level to 140 Celsius */
#define PMIC_TPS6594X_THERMAL_TEMP_TSD_ORD_140C    (0U)
/** \brief Used to set the Thermal shutdown threshold level to 145 Celsius */
#define PMIC_TPS6594X_THERMAL_TEMP_TSD_ORD_145C    (1U)
/* @} */

/**
 *  \anchor Pmic_TPS6594x_Pwr_Thermal_Warn_Lvl_PG_2_0
 *  \name   PMIC Thermal warning threshold level for TPS6594x Leo PMIC PG2.0
 *
 *  @{
 */
/** \brief Used to set the Thermal warning threshold level to 140 Celsius */
#define PMIC_TPS6594X_PG_2_0_THERMAL_TEMP_WARN_140C       (1U)
/** \brief Used to set the Thermal warning threshold level to 130 Celsius */
#define PMIC_TPS6594X_PG_2_0_THERMAL_TEMP_WARN_130C       (0U)
/*  @} */

/**
 *  \anchor Pmic_TPS6594x_Pwr_Thermal_Warn_Lvl_PG_1_0
 *  \name   PMIC Thermal warning threshold level for TPS6594x Leo PMIC PG1.0
 *
 *  @{
 */
/** \brief Used to set the Thermal warning threshold level to 140 Celsius */
#define PMIC_TPS6594X_THERMAL_TEMP_WARN_130C       (1U)
/** \brief Used to set the Thermal warning threshold level to 130 Celsius */
#define PMIC_TPS6594X_THERMAL_TEMP_WARN_120C       (0U)
/*  @} */

/**
 *  \anchor Pmic_Tps6594x_PowerInterruptType
 *  \name   PMIC Power Interrupt selection
 *
 *  @{
 */
#define PMIC_TPS6594X_POWER_OV_INT            (0U)
#define PMIC_TPS6594X_POWER_UV_INT            (1U)
#define PMIC_TPS6594X_POWER_ILIM_INT          (3U)
/*  @} */

/**
 *  \anchor Pmic_Tps6594x_PowerInterruptCommonType
 *  \name   PMIC Power Interrupt selection
 *
 *  @{
 */
#define PMIC_TPS6594X_POWER_INTERRUPT_TWARN                 (0U)
#define PMIC_TPS6594X_POWER_INTERRUPT_NRSTOUT_READBACK      (1U)
#define PMIC_TPS6594X_POWER_INTERRUPT_SOC_PWR_ERR           (2U)
#define PMIC_TPS6594X_POWER_INTERRUPT_MCU_PWR_ERR           (3U)
#define PMIC_TPS6594X_POWER_INTERRUPT_ORD_SHUTDOWN          (4U)
#define PMIC_TPS6594X_POWER_INTERRUPT_IMM_SHUTDOWN          (5U)
#define PMIC_TPS6594X_POWER_INTERRUPT_NRSTOUT_SOC_READBACK  (6U)
#define PMIC_TPS6594X_POWER_INTERRUPT_EN_DRV_READBACK       (7U)
/*  @} */

/**
 *  \anchor Pmic_Tps6594x_PowerLdoRtcCfg
 *  \name   PMIC Power LDORTC enable/disable
 *
 *  @{
 */
#define PMIC_TPS6594X_REGULATOR_LDORTC_ENABLE           (0U)
#define PMIC_TPS6594X_REGULATOR_LDORTC_DISABLE          (1U)
/*  @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/*!
 * \brief   API to enable/disable LDORTC regulator
 *
 * Requirement: REQ_TAG(PDK-5841)
 * Design: did_pmic_power_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function is used to enable/disable LDORTC regulator.
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   ldortcEnable       [IN]    Enable/Disable the LDORTC.
 *                                     Valid values:
 *                                     \ref Pmic_Tps6594x_PowerLdoRtcCfg
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetLdoRtc(Pmic_CoreHandle_t *pPmicCoreHandle,
                            bool               ldortcEnable);

/*!
 * \brief   API to get enable/disable status for LDORTC regulator
 *
 * Requirement: REQ_TAG(PDK-5841)
 * Design: did_pmic_power_cfg_readback
 *
 *          This function is used to get enable/disable status for LDORTC
 *          regulator.
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   pLdortcEnable      [IN]    Pointer to hold Enable/Disable status.
 *                                     Valid values:
 *                                     \ref Pmic_Tps6594x_PowerLdoRtcCfg
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetLdoRtc(Pmic_CoreHandle_t *pPmicCoreHandle,
                            bool              *pLdortcEnable);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_POWER_TPS6594X_H_ */

/* @} */

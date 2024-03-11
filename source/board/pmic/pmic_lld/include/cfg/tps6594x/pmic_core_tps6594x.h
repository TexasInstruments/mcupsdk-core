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
 *  \file pmic_core_tps6594x.h
 *
 *  \brief  The macro definitions for TPS6594x Leo PMIC driver specific
 *          PMIC common configuration
 */

#ifndef PMIC_CORE_TPS6594X_H_
#define PMIC_CORE_TPS6594X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */


#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* On J721E 1.0 EVM, PMIC_TPS6594X_DEV_REV_ID is 0x04 */
#define PMIC_TPS6594X_DEV_REV_ID_PG_1_0          (0x04U)

/* On J721E 2.0 EVM, PMIC_TPS6594X_DEV_REV_ID is 0x41 */
#define PMIC_TPS6594X_DEV_REV_ID_PG_2_0          (0x41U)

/**
 *  \anchor Pmic_Tps6594xLeo_EepromDef_LdCfg
 *  \name   PMIC EEPROM Defaults Load to RTC Domain Bits Configuration
 *
 *  @{
 */
 /** \brief EEPROM defaults are loaded to RTC domain bits */
#define PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS             (0U)
/** \brief EEPROM defaults are not loaded to RTC domain bits */
#define PMIC_TPS6594X_EEPROM_DEFAULTS_NOT_LOADED_TO_RTC_DOMAIN_BITS       (1U)

/**
 *  \anchor Pmic_Tps6594xLeo_AMuxOutPinCtrl_Cfg
 *  \name   PMIC AMUX OUT Pin Control Configuration
 *
 *  @{
 */
 /** \brief Enable Bandgap voltage to AMUXOUT pin */
#define PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE      0U
/** \brief Disable Bandgap voltage to AMUXOUT pin */
#define PMIC_TPS6594X_AMUX_OUT_PIN_CFG_ENABLE       1U
/*  @} */

/**
 *  \anchor Pmic_Tps6594xLeo_ExtClk_Freq_Sel
 *  \name   PMIC External Clock (SYNCCLKIN) Frequency selection
 *
 *  @{
 */
/** \brief  SYNCCLKIN Frequency as 1.1 MHz */
#define PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ      (0U)
/** \brief  SYNCCLKIN Frequency as 2.2 MHz */
#define PMIC_TPS6594X_SYNCCLKIN_2_2_MHZ      (1U)
/** \brief  SYNCCLKIN Frequency as 4.4 MHz */
#define PMIC_TPS6594X_SYNCCLKIN_4_4_MHZ      (2U)
/*  @} */

/**
 *  \anchor Pmic_Tps6594xLeo_Charging_Current_Sel
 *  \name   PMIC Selects Backup Battery charging current
 *
 *  @{
 */
/** \brief  Backup Battery charging current as 100 Micro Ampere */
#define PMIC_TPS6594X_BB_CHARGING_CURRENT_100      (0U)
/** \brief  Backup Battery charging current as 500 Micro Ampere */
#define PMIC_TPS6594X_BB_CHARGING_CURRENT_500      (1U)
/*  @} */

/**
 *  \anchor Pmic_Tps6594xLeo_EndOfChargeVoltage_Sel
 *  \name   PMIC Backup Battery charger End of Charge Volatge selection
 *
 *  @{
 */
/** \brief  Backup Battery charger End of Charge Volatge as 2.5V  */
#define PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_2_5_V      (0U)
/** \brief  Backup Battery charger End of Charge Volatge as 2.8V  */
#define PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_2_8_V      (1U)
/** \brief  Backup Battery charger End of Charge Volatge as 3.0V  */
#define PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_3_0_V      (2U)
/** \brief  Backup Battery charger End of Charge Volatge as 3.3V  */
#define PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_3_3_V      (3U)
/*  @} */

/**
 *  \anchor Pmic_Tps6594xLeo_BatteryCharging_Cfg
 *  \name   PMIC Backup Battery Charging Configuration
 *
 *  @{
 */
 /** \brief Disable Backup Battery Charging */
#define PMIC_TPS6594X_BB_CHARGINGING_CFG_DISABLE      0U
/** \brief Enable Backup Battery Charging */
#define PMIC_TPS6594X_BB_CHARGINGING_CFG_ENABLE       1U
/*  @} */

/**
 *  \anchor Pmic_Tps6594xLeo_BBEoCIndicationStat
 *  \name   PMIC Backup Battery End of Charge Indication Status
 *
 *  @{
 */
 /** \brief  Backup Battery Charging active or not enabled  */
#define PMIC_TPS6594X_BB_EOC_STATUS_NOT_ENABLED   (0U)
 /** \brief  Backup Battery Charger reached termination voltage set by BB_VEOC */
#define PMIC_TPS6594X_BB_EOC_STATUS_READY         (1U)
/*  @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_CORE_TPS6594X_H_ */

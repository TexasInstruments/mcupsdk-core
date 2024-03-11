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
 * \file   pmic_power_tps6594x_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         TPS6594X Leo PMIC driver specific PMIC power configuration
 *
 */

#ifndef PMIC_POWER_TPS6594X_PRIV_H_
#define PMIC_POWER_TPS6594X_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <pmic_power_priv.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_Regulator_Ldo_Voltage_Range
 *  \name   PMIC Maximum ans minimum voltages for LDO regulator
 *          Voltage level are in millivolts.
 *
 *  @{
 */
/** \brief Maximum voltage level for Ldo */
#define PMIC_TPS6594X_REGULATOR_LDO_MAX_VOLTAGE                 (3300U)
/** \brief Minimum voltage level for Ldo 1, 2 and 3 */
#define PMIC_TPS6594X_REGULATOR_LDO1_2_3_MIN_VOLTAGE            (600U)
/** \brief Minimum voltage level for Ldo 4 */
#define PMIC_TPS6594X_REGULATOR_LDO4_MIN_VOLTAGE                (1200U)
/* @} */

/*!
 *  * \brief  PMIC Rail bit field
 *   */
#define PMIC_REGULATOR_LDO_RV_TIMEOUT_SEL_BITFIELD              (15U)

/*!
 * \brief  PMIC Power Volatage range for BUCK regulator
 */
#define PMIC_TPS6594X_REGULATOR_BUCK_MIN_VOLTAGE     PMIC_POWER_VOLTAGE_300MV
#define PMIC_TPS6594X_REGULATOR_BUCK_MAX_VOLTAGE     PMIC_POWER_VOLTAGE_3340MV

/*!
 * \brief  PMIC Power Current range for LDO regulator
 */
#define PMIC_TPS6594X_POWER_LDO1_2_3_MIN_VOLTAGE     PMIC_POWER_VOLTAGE_600MV
#define PMIC_TPS6594X_POWER_LDO_MAX_VOLTAGE          PMIC_POWER_VOLTAGE_3300MV
#define PMIC_TPS6594X_POWER_LDO4_MIN_VOLTAGE         PMIC_POWER_VOLTAGE_1200MV

/*!
 * \brief  PMIC Power Current range for BUCK 1, 2 and 3 regulator
 */
#define PMIC_TPS6594X_BUCK1_4_CURRENT_LIMIT_MIN      \
                                PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_2A5
#define PMIC_TPS6594X_BUCK1_4_CURRENT_LIMIT_MAX      \
                                PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_5A5
/*!
 * \brief  PMIC Power Current range for BUCK5 regulator
 */
#define PMIC_TPS6594X_BUCK5_CURRENT_LIMIT_MIN        \
                                PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_2A5
#define PMIC_TPS6594X_BUCK5_CURRENT_LIMIT_MAX        \
                                PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_3A5

/*!
 * \brief  PMIC Power Slew Rate range for BUCK regulator
 */
#define PMIC_TPS6594X_BUCK_SLEW_RATE_MIN            \
                                PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_33MV
#define PMIC_TPS6594X_BUCK_SLEW_RATE_MAX            \
                                PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_0MV31

/*!
 * \brief  PMIC Power LDO Residual voltage check Timeout range for LDO regulator
 */
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_MIN            \
                                PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_0MS5
#define PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_MAX            \
                                PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_16MS

/*!
 * \brief  PMIC Power Buck regulator range
 */
#define PMIC_TPS6594X_BUCK_MIN   PMIC_TPS6594X_REGULATOR_BUCK1
#define PMIC_TPS6594X_BUCK_MAX   PMIC_TPS6594X_REGULATOR_BUCK5

/*!
 * \brief  PMIC Power Ldo regulator range
 */
#define PMIC_TPS6594X_LDO_MIN   PMIC_TPS6594X_REGULATOR_LDO1
#define PMIC_TPS6594X_LDO_MAX   PMIC_TPS6594X_REGULATOR_LDO4

/*!
 * \brief  PMIC PowerGood Buck regulator range
 */
#define PMIC_TPS6594X_PGOOD_BUCK_MIN   PMIC_TPS6594X_PGOOD_SOURCE_BUCK1
#define PMIC_TPS6594X_PGOOD_BUCK_MAX   PMIC_TPS6594X_PGOOD_SOURCE_BUCK5

/*!
 * \brief  PMIC PowerGood Ldo regulator range
 */
#define PMIC_TPS6594X_PGOOD_LDO_MIN   PMIC_TPS6594X_PGOOD_SOURCE_LDO1
#define PMIC_TPS6594X_PGOOD_LDO_MAX   PMIC_TPS6594X_PGOOD_SOURCE_LDO4

/*!
 * \brief  PMIC Power RAIL group range
 */
#define PMIC_TPS6594X_POWER_RAIL_SEL_MIN    PMIC_TPS6594X_POWER_RAIL_SEL_NONE
#define PMIC_TPS6594X_POWER_RAIL_SEL_MAX    PMIC_TPS6594X_POWER_RAIL_SEL_OTHER

/*!
 * \brief  PMIC Power LDO pull down ressistor range
 */
#define PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_MIN    \
                                     PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_50KOHM
#define PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_MAX    \
                                     PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_500OHM

/*!
 * \brief  PMIC Power Common Interrupt Range
 */
#define PMIC_TPS6594X_POWER_COMMON_INTERRUPT_MAX   \
                                   PMIC_TPS6594X_POWER_INTERRUPT_EN_DRV_READBACK

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/*!
 * \brief  PMIC power resources get Configuration function
 *         This function is used to read the PMIC POWER resources register
 *         Configuration
 *
 * \param  pwrRsrcRegCfg   [OUT]  Pointer to store power resource register
 *                                configuration
 */
void pmic_get_tps6594x_pwrRsrceRegCfg(Pmic_powerRsrcRegCfg_t **pPwrRsrcRegCfg);

/*!
 * \brief  PMIC power interrupt get Configuration function
 *         This function is used to read the power interrupt
 *         Configuration
 *
 * \param  pwrRsrcRegCfg   [OUT]  Pointer to store power interrupt
 *                                number.
 */
void pmic_get_tps6594x_pwrCommonIntCfg(Pmic_powerIntCfg_t **pPwrCommonIntCfg);

/*!
 * \brief  PMIC power get Configuration function
 *         This function is used to read the PMIC pgood sources register
 *         Configuration
 *
 * \param  pPgoodSrcRegCfg   [OUT]  Pointer to store power-good source register
 *                                  configuration
 */
void pmic_get_tps6594x_pwrPgoodSrcRegCfg(
                                  Pmic_powerPgoodSrcRegCfg_t **pPgoodSrcRegCfg);
/*!
 * \brief   This function is used to convert the millivolt value to vset value
 *          for LEO TPS6594x PMIC
 */
int32_t Pmic_powerTPS6594xConvertVoltage2VSetVal(uint16_t  millivolt,
                                                 uint16_t  pwrRsrc,
                                                 uint8_t   *pVSetVal);

/*!
 * \brief   This function is used to convert the vset value to voltage in mv
 *          for PMIC LEO TPS6594x
 */
void Pmic_powerTPS6594xConvertVSet2Voltage(const uint8_t  *pVSetVal,
                                           uint16_t        pwrRsrc,
                                           uint16_t       *millivolt);

/*!
 * \brief   This function is to validate the power good signal source selection
 *          limit for the specific PMIC device.
 */
int32_t Pmic_validate_tps6594x_pGoodSrcType(uint16_t pgoodSrc);

/*!
 * \brief   This function is to validate the power good signal source selection
 *          limit for the specific PMIC device.
 */
int32_t Pmic_validate_tps6594x_pGoodSelType(uint16_t pgoodSrc,
                                            uint8_t pgoodSelType);

/**
 * \brief   This function is used to validate the voltage levels for
 *          Regulators/VMON for TPS6594x PMIC
 */
int32_t Pmic_powerTPS6594xValidateVoltageLevel(
                                             uint8_t            pwrRsrcType,
                                             uint16_t           pwrRsrc,
                                             uint16_t           voltage_mV);

/*!
 * \brief   This function is to validate the power resource limit for the
 *          TPS6594x PMIC device.
 */
int32_t Pmic_powerTPS6594xValidatePwrRsrcLimit(
                                    const Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint8_t                  pwrRsrcType,
                                    uint16_t                 pwrRsrc);

/*!
 * \brief   This function is to validate the power resource interrupt type
 *          for the TPS6594x PMIC device.
 */
int32_t Pmic_powerTPS6594xValidateIntrType(uint8_t  pwrResourceType,
                                           uint8_t  intrType);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_POWER_TPS6594X_PRIV_H_ */

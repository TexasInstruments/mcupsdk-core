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
 * \file   pmic_power_lp8764x_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         LPL8764X HERA PMIC driver specific PMIC power configuration
 *
 */

#ifndef PMIC_POWER_LPL8764X_PRIV_H_
#define PMIC_POWER_LPL8764X_PRIV_H_

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

/*!
 * \brief  VMON powergood Window Register Address
 */
#define PMIC_VMON1_PG_WINDOW_REGADDR                            (0x2D)
#define PMIC_VMON2_PG_WINDOW_REGADDR                            (0x2F)

/*!
 * \brief  VMON powergood Level Register Address
 */
#define PMIC_VMON1_PG_LEVEL_REGADDR                             (0x2E)
#define PMIC_VMON2_PG_LEVEL_REGADDR                             (0x30)

/*!
 * \brief  VMON powergood Level Register Address
 */
#define PMIC_VMON_CONF_REGADDR                                  (0xA8U)

/*!
 * \brief  PMIC Power voltage range bit fields
 */
#define PMIC_VMON_PG_WINDOW_VMON_RANGE_SHIFT                    (6U)

/*!
 * \brief  PMIC Powergood level bit fields
 */
#define PMIC_VMONX_PG_LEVEL_VMONX_PG_SET_SHIFT                   (0U)

/*!
 * \brief  PMIC VMON1 and VMON2 slew rate bit fields.
 */
#define PMIC_VMON_CONF_VMON2_SLEW_RATE_SHIFT                    (3U)
#define PMIC_VMON_CONF_VMON1_SLEW_RATE_SHIFT                    (0U)

/*!
 * \brief  PMIC voltage range for VMON when VMON range
            is PMIC_VMON_RANGE_3V35_5V.
 */
#define PMIC_LP8764X_RANGE1_VMON_MIN_VOLTAGE                    (3350U)
#define PMIC_LP8764X_RANGE1_VMON_MAX_VOLTAGE                    (5000U)

/*!
 * \brief  PMIC VMON1 and VMON2 slew rate bit masks.
 */
#define PMIC_VMON_CONF_VMON2_SLEW_RATE_MASK                         \
                                          (uint8_t)(0x07U <<        \
                                          PMIC_VMON_CONF_VMON2_SLEW_RATE_SHIFT)
#define PMIC_VMON_CONF_VMON1_SLEW_RATE_MASK                         \
                                          (uint8_t)(0x07U <<        \
                                          PMIC_VMON_CONF_VMON1_SLEW_RATE_SHIFT)
/*!
 * \brief  PMIC Power voltage range bit masks
 */
#define PMIC_VMON_PG_WINDOW_VMON_RANGE_MASK                        \
                                        (uint8_t)(0x01U <<         \
                                        PMIC_VMON_PG_WINDOW_VMON_RANGE_SHIFT)

/*!
 * \brief  PMIC Powergood level bit masks
 */
#define PMIC_VMONX_PG_LEVEL_VMONX_PG_SET_MASK                      \
                                        (uint8_t)(0xFFU <<         \
                                        PMIC_VMONX_PG_LEVEL_VMONX_PG_SET_SHIFT)
/*!
 * \brief  PMIC Power Volatage range for BUCK regulator
 */
#define PMIC_LP8764X_REGULATOR_BUCK_MIN_VOLTAGE     PMIC_POWER_VOLTAGE_300MV
#define PMIC_LP8764X_REGULATOR_BUCK_MAX_VOLTAGE     PMIC_POWER_VOLTAGE_3340MV

/*!
 * \brief  PMIC Power Volatage range for VMON, when Range is 0
 */
#define PMIC_LP8764X_RANGE0_VMON_MIN_VOLTAGE     PMIC_POWER_VOLTAGE_300MV
#define PMIC_LP8764X_RANGE0_VMON_MAX_VOLTAGE     PMIC_POWER_VOLTAGE_3340MV

/*!
 * \brief  PMIC Power Current range for BUCK regulator
 */
#define  PMIC_LP8764X_BUCK_CURRENT_LIMIT_MIN         \
                                PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_2A5
#define  PMIC_LP8764X_BUCK_CURRENT_LIMIT_MAX         \
                                PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_6A5

/*!
 * \brief  PMIC Power Slew Rate range for BUCK regulator
 */
#define PMIC_LP8764X_BUCK_SLEW_RATE_MIN              \
                                PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_33MV
#define PMIC_LP8764X_BUCK_SLEW_RATE_MAX              \
                                PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_0MV31
/*!
 * \brief  PMIC Power Buck regulator range
 */
#define PMIC_LP8764X_BUCK_MIN   PMIC_LP8764X_REGULATOR_BUCK1
#define PMIC_LP8764X_BUCK_MAX   PMIC_LP8764X_REGULATOR_BUCK4

/*!
 * \brief  PMIC Power VMON range
 */
#define PMIC_LP8764X_VMON_MIN   PMIC_LP8764X_POWER_SOURCE_VMON1
#define PMIC_LP8764X_VMON_MAX   PMIC_LP8764X_POWER_SOURCE_VMON2

/*!
 * \brief  PMIC Power RAIL group range
 */
#define PMIC_LP8764X_POWER_RAIL_SEL_MIN    PMIC_LP8764X_POWER_RAIL_SEL_NONE
#define PMIC_LP8764X_POWER_RAIL_SEL_MAX    PMIC_LP8764X_POWER_RAIL_SEL_OTHER

/*!
 * \brief  PMIC Power Buck regulator range
 */
#define PMIC_LP8764X_PGOOD_BUCK_MIN   PMIC_LP8764X_PGOOD_SOURCE_BUCK1
#define PMIC_LP8764X_PGOOD_BUCK_MAX   PMIC_LP8764X_PGOOD_SOURCE_BUCK4

/*!
 * \brief  PMIC Power VMON range
 */
#define PMIC_LP8764X_PGOOD_VMON_MIN   PMIC_LP8764X_PGOOD_SOURCE_VMON1
#define PMIC_LP8764X_PGOOD_VMON_MAX   PMIC_LP8764X_PGOOD_SOURCE_VMON2
/*!
 * \brief  PMIC Power Common Interrupt Range
 */
#define PMIC_LP8764X_POWER_COMMON_INTERRUPT_MAX   \
                                    PMIC_LP8764X_POWER_INTERRUPT_EN_DRV_READBACK
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
void pmic_get_lp8764x_pwrRsrceRegCfg(Pmic_powerRsrcRegCfg_t **pPwrRsrcRegCfg);

/*!
 * \brief  PMIC power common interrupt get Configuration function
 *         This function is used to read the interrupt
 *         Configuration
 *
 * \param  pwrRsrcRegCfg   [OUT]  Pointer to store power interrupt
 *                                number.
 */
void pmic_get_lp8764x_pwrCommonIntCfg(Pmic_powerIntCfg_t **pPwrCommonIntCfg);

/*!
 * \brief  PMIC power get Configuration function
 *         This function is used to read the PMIC pgood sources register
 *         Configuration
 *
 * \param  pPgoodSrcRegCfg   [OUT]  Pointer to store power-good source register
 *                                  configuration
 */
void pmic_get_lp8764x_pwrPgoodSrcRegCfg(
                                  Pmic_powerPgoodSrcRegCfg_t **pPgoodSrcRegCfg);

/*!
 * \brief   This function is used to convert the millivolt value to vset value
 *          for HERA LP8764x PMIC
 */
int32_t Pmic_powerLP8764xConvertVoltage2VSetVal(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint16_t           millivolt,
                                            uint16_t           pwrRsrc,
                                            uint8_t           *pVSetVal);

/*!
 * \brief   This function is used to convert the vsetvalue to voltage in mv
 *          for PMIC HERA LP8764x
 */
int32_t Pmic_powerLP8764xConvertVSetVal2Voltage(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
                                            const uint8_t     *pVSetVal,
                                            uint16_t           pwrRsrc,
                                            uint16_t          *millivolt);

/*!
 * \brief   This function is to validate the power good source limit for the
 *          specific PMIC device.
 */
int32_t Pmic_validate_lp8764x_pGoodSrcType(uint16_t pgoodSrc);

/*!
 * \brief   This function is to validate the power good signal source selection
 *          limit for the specific PMIC device.
 */
int32_t Pmic_validate_lp8764x_pGoodSelType(uint16_t pgoodSrc,
                                           uint8_t pgoodSelType);

/**
 * \brief   This function is used to validate the voltage levels for
 *          Regulators/VMON for LP8764x PMIC
 */
int32_t Pmic_powerLP8764xValidateVoltageLevel(
                                             Pmic_CoreHandle_t *pPmicCoreHandle,
                                             uint8_t            pwrRsrcType,
                                             uint16_t           pwrRsrc,
                                             uint16_t           voltage_mV);

/*!
 * \brief   This function is to validate the power resource limit for the
 *          LP8764x PMIC device.
 */
int32_t Pmic_powerLP8764xValidatePwrRsrcLimit(
                                    const Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint8_t                  pwrRsrcType,
                                    uint16_t                 pwrRsrc);

/*!
 * \brief   This function is to validate the power resource interrupt type
 *          for the LP8764x PMIC device.
 */
int32_t Pmic_powerLP8764xValidateIntrType(uint8_t  pwrResourceType,
                                          uint8_t  intrType);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_POWER_LP8764X_PRIV_H_ */

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
 * @file pmic_power_priv.h
 *
 * @brief: This file contains macro definitions, structures and function
 *         prototypes for driver specific PMIC power configuration
 */

#ifndef PMIC_POWER_PRIV_H_
#define PMIC_POWER_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

#define PMIC_POWER_LDO_MAX                  (4U)
#define PMIC_POWER_LDO_MIN                  (1U)
#define PMIC_POWER_PLDO_MAX                 (2U)
#define PMIC_POWER_PLDO_MIN                 (1U)

#define PMIC_LDO1                           (1U)
#define PMIC_LDO2                           (2U)
#define PMIC_LDO3                           (3U)
#define PMIC_LDO4                           (4U)

#define PMIC_PLDO1                          (1U)
#define PMIC_PLDO2                          (2U)

#define PMIC_EXT_VMON1                      (1U)
#define PMIC_EXT_VMON2                      (2U)

#define PMIC_EN_OUT1                        (1U)
#define PMIC_EN_OUT2                        (2U)
#define PMIC_EN_OUT_ALL                     (3U)

/**
 * @brief  BUCK configuration Register Address
 */
#define PMIC_POWER_BUCK_CFG_REGADDR         (0x1BU)

/**
 * @brief  LDO configuration Register Address
 */
#define PMIC_LDO1_CFG_REGADDR               (0x1EU)
#define PMIC_LDO2_CFG_REGADDR               (0x1FU)
#define PMIC_LDO3_CFG_REGADDR               (0x20U)
#define PMIC_LDO4_CFG_REGADDR               (0x21U)

#define PMIC_LDO_PGOOD_CFG_REGADDR          (0x25U)
#define PMIC_LDO_CTRL_REGADDR               (0x26U)

#define PMIC_LDO_DSCG_CFG_REGADDR           (0x28U)

/**
 * @brief  PLDO configuration Register Address
 */
#define PMIC_PLDO1_CFG_REGADDR              (0x22U)
#define PMIC_PLDO2_CFG_REGADDR              (0x23U)

#define PMIC_PLDO_CFG_REGADDR               (0x24U)
#define PMIC_PLDO_EN_OUT_CTRL_REGADDR       (0x27U)

/**
 * @brief  VCCA Vmon configuration Register Address
 */
#define PMIC_VMON_TH_CFG1_REGADDR           (0x29U)
#define PMIC_VMON_TH_CFG2_REGADDR           (0x2AU)
#define PMIC_VMON_TH_CFG3_REGADDR           (0x2BU)

#define PMIC_LP_VMON_CTRL_REGADDR           (0x2CU)
#define PMIC_LP_CFG_REGADDR                 (0x2FU)

#define PMIC_VMON_DGL_CFG1_REGADDR          (0x31U)
#define PMIC_VMON_DGL_CFG2_REGADDR          (0x32U)
#define PMIC_VMON_DGL_CFG3_REGADDR          (0x33U)

/**
 * @brief  PMIC power voltage levels
 */
#define PMIC_BUCK_VOLTAGE_4000MV            (0U)
#define PMIC_BUCK_VOLTAGE_BB_VOL            (1U)
#define PMIC_BUCK_VOLTAGE_4300MV            (0U)
#define PMIC_BUCK_VOLTAGE_5000MV            (1U)
#define PMIC_BUCK_VOLTAGE_6000MV            (2U)

/* Define macros for Buck-Boost bit positions and masks */
#define BB_PGOOD_CFG_SHIFT                  (0x05U)
#define BB_PGOOD_CFG_MASK                   (0x01U)

#define BB_SS_EN_SHIFT                      (0x04U)
#define BB_SS_EN_MASK                       (0x01U)

#define BB_STBY_LVL_CFG_SHIFT               (0x03U)
#define BB_STBY_LVL_CFG_MASK                (0x01U)

#define BB_LVL_CFG_SHIFT                    (0x00U)
#define BB_LVL_CFG_MASK                     (0x07U)

/* Define macros for LDO bit positions and masks */
#define LDO_RT_CFG_SHIFT                    (0x07U)
#define LDO_RT_CFG_MASK                     (0x01U)

#define LDO_ILIM_LVL_CFG_SHIFT              (0x05U)
#define LDO_ILIM_LVL_CFG_MASK               (0x03U)

#define LDO_LVL_CFG_SHIFT                   (0x00U)
#define LDO_LVL_CFG_MASK                    (0x1FU)

#define LDO_ALL_CFG_SHIFT                   (0x00U)
#define LDO_ALL_CFG_MASK                    (0xFFU)

/* Define macros for PLDO Config bit positions and masks */
#define PLDO_MODE_SEL_CFG_SHIFT             (0x07U)
#define PLDO_MODE_SEL_CFG_MASK              (0x01U)

#define PLDO_ILIM_LVL_CFG_SHIFT             (0x05U)
#define PLDO_ILIM_LVL_CFG_MASK              (0x03U)

#define PLDO_LVL_CFG_SHIFT                  (0x00U)
#define PLDO_LVL_CFG_MASK                   (0x1FU)

/* Define macros for PLDO VTrack RNG bit positions and masks */
#define PLDO_VTRACK_RNG_CFG_SHIFT           (0x02U)
#define PLDO_VTRACK_RNG_CFG_MASK            (0x01U)

#define PLDO_PLDO2_RT_CFG_SHIFT             (0x01U)
#define PLDO_PLDO2_RT_CFG_MASK              (0x01U)

#define PLDO_PLDO1_RT_CFG_SHIFT             (0x00U)
#define PLDO_PLDO1_RT_CFG_MASK              (0x01U)

/* Define macros for LDO and PLDO PGOOD status */
#define PLDO2_PGOOD_CFG_SHIFT               (0x05U)
#define PLDO2_PGOOD_CFG_MASK                (0x01U)

#define PLDO1_PGOOD_CFG_SHIFT               (0x04U)
#define PLDO1_PGOOD_CFG_MASK                (0x01U)

#define LDO4_PGOOD_CFG_SHIFT                (0x03U)
#define LDO4_PGOOD_CFG_MASK                 (0x01U)

#define LDO3_PGOOD_CFG_SHIFT                (0x02U)
#define LDO3_PGOOD_CFG_MASK                 (0x01U)

#define LDO2_PGOOD_CFG_SHIFT                (0x01U)
#define LDO2_PGOOD_CFG_MASK                 (0x01U)

#define LDO1_PGOOD_CFG_SHIFT                (0x00U)
#define LDO1_PGOOD_CFG_MASK                 (0x01U)

/* Define macros for LDO Control and mode changes configuration */
#define LDO4_CTRL_SHIFT                     (0x06U)
#define LDO4_CTRL_MASK                      (0x03U)

#define LDO3_CTRL_SHIFT                     (0x04U)
#define LDO3_CTRL_MASK                      (0x03U)

#define LDO2_CTRL_SHIFT                     (0x02U)
#define LDO2_CTRL_MASK                      (0x03U)

#define LDO1_CTRL_SHIFT                     (0x00U)
#define LDO1_CTRL_MASK                      (0x03U)

#define LDO_ALL_CTRL_SHIFT                  (0x00U)
#define LDO_ALL_CTRL_MASK                   (0xFFU)

/* Define Macros for PLDO and EN_OUT Control Register configuration */
#define EN_OUTALL_ENABLE_SHIFT              (0x05U)
#define EN_OUTALL_ENABLE_MASK               (0x03U)

#define EN_OUT2_ENABLE_SHIFT                (0x06U)
#define EN_OUT2_ENABLE_MASK                 (0x01U)

#define EN_OUT1_ENABLE_SHIFT                (0x05U)
#define EN_OUT1_ENABLE_MASK                 (0x01U)

#define PLDO2_CTRL_SHIFT                    (0x02U)
#define PLDO2_CTRL_MASK                     (0x07U)

#define PLDO1_CTRL_SHIFT                    (0x00U)
#define PLDO1_CTRL_MASK                     (0x03U)

/* Define Macros for LDO and PLDO Discharge pull-down configuration */
#define PMIC_PLDO2_DSCG_SHIFT               (0x05U)
#define PMIC_PLDO2_DSCG_MASK                (0x01U)

#define PMIC_PLDO1_DSCG_SHIFT               (0x04U)
#define PMIC_PLDO1_DSCG_MASK                (0x01U)

#define PMIC_LDO4_DSCG_SHIFT                (0x03U)
#define PMIC_LDO4_DSCG_MASK                 (0x01U)

#define PMIC_LDO3_DSCG_SHIFT                (0x02U)
#define PMIC_LDO3_DSCG_MASK                 (0x01U)

#define PMIC_LDO2_DSCG_SHIFT                (0x01U)
#define PMIC_LDO2_DSCG_MASK                 (0x01U)

#define PMIC_LDO1_DSCG_SHIFT                (0x00U)
#define PMIC_LDO1_DSCG_MASK                 (0x01U)

/* Define Macros for Low Power VMon Control Register */
#define PMIC_LP_EXT_VMON2_CTRL_SHIFT        (0x07U)
#define PMIC_LP_EXT_VMON2_CTRL_MASK         (0x01U)

#define PMIC_LP_EXT_VMON1_CTRL_SHIFT        (0x06U)
#define PMIC_LP_EXT_VMON1_CTRL_MASK         (0x01U)

#define PMIC_LP_PLDO2_VMON_CTRL_SHIFT       (0x05U)
#define PMIC_LP_PLDO2_VMON_CTRL_MASK        (0x01U)

#define PMIC_LP_PLDO1_VMON_CTRL_SHIFT       (0x04U)
#define PMIC_LP_PLDO1_VMON_CTRL_MASK        (0x01U)

#define PMIC_LP_LDO4_VMON_CTRL_SHIFT        (0x03U)
#define PMIC_LP_LDO4_VMON_CTRL_MASK         (0x01U)

#define PMIC_LP_LDO3_VMON_CTRL_SHIFT        (0x02U)
#define PMIC_LP_LDO3_VMON_CTRL_MASK         (0x01U)

#define PMIC_LP_LDO2_VMON_CTRL_SHIFT        (0x01U)
#define PMIC_LP_LDO2_VMON_CTRL_MASK         (0x01U)

#define PMIC_LP_LDO1_VMON_CTRL_SHIFT        (0x00U)
#define PMIC_LP_LDO1_VMON_CTRL_MASK         (0x01U)

/* Define Macros for Low Power Configuration Register */
#define PMIC_LP_BB_OVP_CTRL_SHIFT           (0x07U)
#define PMIC_LP_BB_OVP_CTRL_MASK            (0x01U)

#define PMIC_LP_BB_VMON_CTRL_SHIFT          (0x06U)
#define PMIC_LP_BB_VMON_CTRL_MASK           (0x01U)

#define PMIC_LP_TSD_PER_CFG_SHIFT           (0x01U)
#define PMIC_LP_TSD_PER_CFG_MASK            (0x01U)

#define PMIC_LP_VMON_PER_CFG_SHIFT          (0x00U)
#define PMIC_LP_VMON_PER_CFG_MASK           (0x01U)

/* Define Macros for LDO VMON Threshold Config-1 Register configuration */
#define PMIC_VMON_CFG1_LDO4_VMON_SHIFT      (0x06U)
#define PMIC_VMON_CFG1_LDO4_VMON_MASK       (0x03U)

#define PMIC_VMON_CFG1_LDO3_VMON_SHIFT      (0x04U)
#define PMIC_VMON_CFG1_LDO3_VMON_MASK       (0x03U)

#define PMIC_VMON_CFG1_LDO2_VMON_SHIFT      (0x02U)
#define PMIC_VMON_CFG1_LDO2_VMON_MASK       (0x03U)

#define PMIC_VMON_CFG1_LDO1_VMON_SHIFT      (0x00U)
#define PMIC_VMON_CFG1_LDO1_VMON_MASK       (0x03U)

/* Define Macros for Ext_VMON and PLDO VMON Threshold Config-2 Register
 * configuration */
#define PMIC_VMON_CFG2_EXT_VMON2_SHIFT      (0x06U)
#define PMIC_VMON_CFG2_EXT_VMON2_MASK       (0x03U)

#define PMIC_VMON_CFG2_EXT_VMON1_SHIFT      (0x04U)
#define PMIC_VMON_CFG2_EXT_VMON1_MASK       (0x03U)

#define PMIC_VMON_CFG2_PLDO2_VMON_SHIFT     (0x02U)
#define PMIC_VMON_CFG2_PLDO2_VMON_MASK      (0x03U)

#define PMIC_VMON_CFG2_PLDO1_VMON_SHIFT     (0x00U)
#define PMIC_VMON_CFG2_PLDO1_VMON_MASK      (0x03U)

/* Define Macros for Buck-Boost VMON Threshold Config-3 Register configuration
 */
#define PMIC_VMON_CFG3_BB_TMO_CFG_SHIFT     (0x02U)
#define PMIC_VMON_CFG3_BB_TMO_CFG_MASK      (0x03U)

#define PMIC_VMON_CFG3_BB_VMON_TH_SHIFT     (0x00U)
#define PMIC_VMON_CFG3_BB_VMON_TH_MASK      (0x03U)

/* Define Macros for VMON De-Glitch Config-1 Register configuration */
#define PMIC_VMON_DGL_CFG1_VBAT_SHIFT       (0x05U)
#define PMIC_VMON_DGL_CFG1_VBAT_MASK        (0x03U)

#define PMIC_VMON_DGL_CFG1_BB_SHIFT         (0x06U)
#define PMIC_VMON_DGL_CFG1_BB_MASK          (0x01U)

/* Define Macros for LDO VMON De-Glitch Config-2 Register configuration */
#define PMIC_VMON_DGL_CFG2_LDO4_SHIFT       (0x06U)
#define PMIC_VMON_DGL_CFG2_LDO4_MASK        (0x03U)

#define PMIC_VMON_DGL_CFG2_LDO3_SHIFT       (0x04U)
#define PMIC_VMON_DGL_CFG2_LDO3_MASK        (0x03U)

#define PMIC_VMON_DGL_CFG2_LDO2_SHIFT       (0x02U)
#define PMIC_VMON_DGL_CFG2_LDO2_MASK        (0x03U)

#define PMIC_VMON_DGL_CFG2_LDO1_SHIFT       (0x00U)
#define PMIC_VMON_DGL_CFG2_LDO1_MASK        (0x03U)

/* Define Macros for Ext_VMON and PLDO VMON De-Glitch Config-3 Register
 * configuration */
#define PMIC_VMON_DGL_CFG3_EXT_V1_SHIFT     (0x06U)
#define PMIC_VMON_DGL_CFG3_EXT_V1_MASK      (0x03U)

#define PMIC_VMON_DGL_CFG3_EXT_V2_SHIFT     (0x04U)
#define PMIC_VMON_DGL_CFG3_EXT_V2_MASK      (0x03U)

#define PMIC_VMON_DGL_CFG3_PLDO2_SHIFT      (0x02U)
#define PMIC_VMON_DGL_CFG3_PLDO2_MASK       (0x03U)

#define PMIC_VMON_DGL_CFG3_PLDO1_SHIFT      (0x00U)
#define PMIC_VMON_DGL_CFG3_PLDO1_MASK       (0x03U)

/* Define PMIC Power EN-OUT Switch case macros*/
#define PMIC_ONLY_ENOUT1_EN                 (0b01U)
#define PMIC_ONLY_ENOUT2_EN                 (0b10U)
#define PMIC_BOTH_ENOUT1_ENOUT2_EN          (0b11U)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @brief   This function is used to get the id of power resource.
 *          BUCK1/BUCK2/LDO1...etc
 */
static inline uint8_t Pmic_powerGetPwrRsrcId(uint16_t pwrRsrc) {
    uint8_t pwrRsrcId;
    pwrRsrcId = (uint8_t)(pwrRsrc & 0xFFU);

    return pwrRsrcId;
}

/**
 * @brief   This function is used to get the type of power resource.
 *          BUCK/LDO/VCCA/VMON
 */
static inline uint8_t Pmic_powerGetPwrRsrcType(uint16_t pwrRsrc) {
    uint8_t pwrRsrcType;
    pwrRsrcType = (uint8_t)((pwrRsrc >> 0x8U) & 0xFFU);

    return pwrRsrcType;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_POWER_PRIV_H_ */

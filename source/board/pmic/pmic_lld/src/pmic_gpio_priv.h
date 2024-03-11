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
 * @file   pmic_gpio_priv.h
 *
 * @brief  This file contains macro definitions, structures and function
 *         prototypes for driver specific PMIC GPIO private configuration.
 */

#ifndef PMIC_GPIO_PRIV_H_
#define PMIC_GPIO_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Pmic_GPIO PMIC General Purpose Input/Output
 * @{
 * @brief Contains definitions related to PMIC GPIO functionality.
 */

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @defgroup Pmic_GPIOPrivMacros PMIC General Purpose Input/Output Private Macros
 * @{
 * @ingroup Pmic_GPIO
 * @brief Contains private macros used in the GPIO module of PMIC driver.
 */

/**
 * @brief Bit shift value for selecting GPIO configuration in GPIOX_CONF register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIOX_CONF_GPIO_SEL_SHIFT          (0x05U)

/**
 * @brief Bit shift value for enabling deglitch for GPIO in GPIOX_CONF register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_SHIFT  (0x04U)

/**
 * @brief Bit shift value for enabling pull-up/pull-down for GPIO in GPIOX_CONF register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIOX_CONF_GPIO_PU_PD_EN_SHIFT     (0x03U)

/**
 * @brief Bit shift value for selecting pull-up for GPIO in GPIOX_CONF register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIOX_CONF_GPIO_PU_SEL_SHIFT       (0x02U)

/**
 * @brief Bit shift value for configuring GPIO as open-drain in GPIOX_CONF register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIOX_CONF_GPIO_OD_SHIFT           (0x01U)

/**
 * @brief Bit shift value for configuring GPIO direction in GPIOX_CONF register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIOX_CONF_GPIO_DIR_SHIFT          (0x00U)

/**
 * @brief Bit shift value for GPIO1_IN in GPIO_IN_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_IN_1_GPIO1_IN_SHIFT           (0x00U)

/**
 * @brief Bit shift value for GPIO2_IN in GPIO_IN_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_IN_1_GPIO2_IN_SHIFT           (0x01U)

/**
 * @brief Bit shift value for GPIO3_IN in GPIO_IN_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_IN_1_GPIO3_IN_SHIFT           (0x02U)

/**
 * @brief Bit shift value for GPIO4_IN in GPIO_IN_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_IN_1_GPIO4_IN_SHIFT           (0x03U)

/**
 * @brief Bit shift value for GPIO5_IN in GPIO_IN_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_IN_1_GPIO5_IN_SHIFT           (0x04U)

/**
 * @brief Bit shift value for GPIO6_IN in GPIO_IN_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_IN_1_GPIO6_IN_SHIFT           (0x05U)

/**
 * @brief Bit shift value for GPIO7_IN in GPIO_IN_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_IN_1_GPIO7_IN_SHIFT           (0x06U)

/**
 * @brief Bit shift value for GPIO8_IN in GPIO_IN_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_IN_1_GPIO8_IN_SHIFT           (0x07U)

/**
 * @brief Bit shift value for GPIO9_IN in GPIO_IN_2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_IN_2_GPIO9_IN_SHIFT           (0x00U)

/**
 * @brief Bit shift value for GPIO10_IN in GPIO_IN_2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_IN_2_GPIO10_IN_SHIFT          (0x01U)

/**
 * @brief Bit shift value for GPIO1_OUT in GPIO_OUT_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_OUT_1_GPIO1_OUT_SHIFT         (0x00U)

/**
 * @brief Bit shift value for GPIO2_OUT in GPIO_OUT_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_OUT_1_GPIO2_OUT_SHIFT         (0x01U)

/**
 * @brief Bit shift value for GPIO3_OUT in GPIO_OUT_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_OUT_1_GPIO3_OUT_SHIFT         (0x02U)

/**
 * @brief Bit shift value for GPIO4_OUT in GPIO_OUT_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_OUT_1_GPIO4_OUT_SHIFT         (0x03U)

/**
 * @brief Bit shift value for GPIO5_OUT in GPIO_OUT_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_OUT_1_GPIO5_OUT_SHIFT         (0x04U)

/**
 * @brief Bit shift value for GPIO6_OUT in GPIO_OUT_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_OUT_1_GPIO6_OUT_SHIFT         (0x05U)

/**
 * @brief Bit shift value for GPIO7_OUT in GPIO_OUT_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_OUT_1_GPIO7_OUT_SHIFT         (0x06U)

/**
 * @brief Bit shift value for GPIO8_OUT in GPIO_OUT_1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_OUT_1_GPIO8_OUT_SHIFT         (0x07U)

/**
 * @brief Bit shift value for GPIO9_OUT in GPIO_OUT_2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_OUT_2_GPIO9_OUT_SHIFT         (0x00U)

/**
 * @brief Bit shift value for GPIO10_OUT in GPIO_OUT_2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_OUT_2_GPIO10_OUT_SHIFT        (0x01U)

/**
 * @brief Mask for selecting GPIO function in GPIOX_CONF register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIOX_CONF_GPIO_SEL_MASK           (uint8_t)(0x07U << PMIC_GPIOX_CONF_GPIO_SEL_SHIFT)

/**
 * @brief Mask for enabling deglitching for GPIO in GPIOX_CONF register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_MASK   (uint8_t)(0x01U << PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_SHIFT)

/**
 * @brief Mask for enabling pull-up/pull-down for GPIO in GPIOX_CONF register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIOX_CONF_GPIO_PU_PD_EN_MASK      (uint8_t)(0x01U << PMIC_GPIOX_CONF_GPIO_PU_PD_EN_SHIFT)

/**
 * @brief Mask for selecting pull-up/pull-down for GPIO in GPIOX_CONF register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIOX_CONF_GPIO_PU_SEL_MASK        (uint8_t)(0x01U << PMIC_GPIOX_CONF_GPIO_PU_SEL_SHIFT)

/**
 * @brief Mask for selecting open-drain mode for GPIO in GPIOX_CONF register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIOX_CONF_GPIO_OD_MASK            (uint8_t)(0x01U << PMIC_GPIOX_CONF_GPIO_OD_SHIFT)

/**
 * @brief Mask for selecting GPIO direction in GPIOX_CONF register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIOX_CONF_GPIO_DIR_MASK           (uint8_t)(0x01U << PMIC_GPIOX_CONF_GPIO_DIR_SHIFT)

/**
 * @brief Bitfield value for GPIO input/output configuration.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_IN_OUT_X_GPIOX_IN_OUT_BITFIELD    (1U)

/**
 * @brief Maximum value for GPIO pin function.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_PINFUNC_MAX                       (7U)

/**
 * @brief Enable value for GPIO interrupt.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_INT_ENABLE                        (0U)

/**
 * @brief Mask value for GPIO interrupt.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_INT_MASK                          (1U)

/**
 * @brief Device ID for the TPS653860XX PMIC.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_DEV_BB_TPS653860XX                     (3U)

/**
 * @brief GPIO pin identifier for GPO1 of the TPS653860XX PMIC.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_BB_GPO1                                (1U)

/**
 * @brief GPIO pin identifier for GPO2 of the TPS653860XX PMIC.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_BB_GPO2                                (2U)

/**
 * @brief GPIO pin identifier for GPO3 of the TPS653860XX PMIC.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_BB_GPO3                                (3U)

/**
 * @brief GPIO pin identifier for GPO4 of the TPS653860XX PMIC.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_BB_GPO4                                (4U)

/**
 * @brief Minimum GPIO pin number for the TPS653860XX PMIC.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_TPS653860XX_GPIO_PIN_MIN               (1U)

/**
 * @brief Maximum GPIO pin number for the TPS653860XX PMIC.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_TPS653860XX_GPIO_PIN_MAX               (6U)

/**
 * @brief Register address for configuring GPI 1.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPI_1_CONF_REGADDR                     (0x7EU)

/**
 * @brief Register address for configuring GPO 1.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_1_CONF_REGADDR                     (0x7CU)

/**
 * @brief Register address for configuring GPO 2.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_2_CONF_REGADDR                     (0x7DU)

/**
 * @brief Bit shift value for GPI 1 within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPI_1_GPI_1_SHIFT                      (0x00U)

/**
 * @brief Bit shift value for GPI 4 within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPI_4_GPI_2_SHIFT                      (0x01U)

/**
 * @brief Bit shift value for GPO 1 within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_1_GPO_1_SHIFT                      (0x00U)

/**
 * @brief Bit shift value for GPO 2 within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_2_GPI_3_SHIFT                      (0x02U)

/**
 * @brief Bit shift value for GPO 3 within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_3_GPO_1_SHIFT                      (0x00U)

/**
 * @brief Bit shift value for GPO 4 within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_4_GPO_3_SHIFT                      (0x03U)

/**
 * @brief Mask for GPI 1 within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPI_1_GPI_1_MASK                       (uint8_t)(0x01U << PMIC_GPI_1_GPI_1_SHIFT)

/**
 * @brief Mask for GPI 4 within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPI_4_GPI_2_MASK                       (uint8_t)(0x03U << PMIC_GPI_4_GPI_2_SHIFT)

/**
 * @brief Mask for GPO 2 within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_1_GPO_2_MASK                       (uint8_t)(0x07U << PMIC_GPO_1_GPO_1_SHIFT)

/**
 * @brief Mask for GPO 3 within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_2_GPO_3_MASK                       (uint8_t)(0x07U << PMIC_GPO_2_GPI_3_SHIFT)

/**
 * @brief Mask for GPO 2 within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_3_GPO_2_MASK                       (uint8_t)(0x07U << PMIC_GPO_3_GPO_1_SHIFT)

/**
 * @brief Mask for GPO 3 within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_4_GPO_3_MASK                       (uint8_t)(0x07U << PMIC_GPO_4_GPO_3_SHIFT)

/**
 * @brief Register address for reading deglitch configuration 2.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_DGL_CFG2_REGADDR                  (0x80U)

/**
 * @brief Register address for reading deglitch configuration 3.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_DGL_CFG3_REGADDR                  (0x81U)

/**
 * @brief Bit shift value for GPO1 falling edge deglitch time.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO1_F_SHIFT                      (0X00U)

/**
 * @brief Bit shift value for GPO1 rising edge deglitch time.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO1_R_SHIFT                      (0X02U)

/**
 * @brief Bit shift value for GPO2 falling edge deglitch time.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO2_F_SHIFT                      (0X04U)

/**
 * @brief Bit shift value for GPO2 rising edge deglitch time.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO2_R_SHIFT                      (0X06U)

/**
 * @brief Mask for GPO1 falling edge deglitch time within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO1_F_MASK                       (uint8_t)(0x03U << PMIC_RDBK_GPO1_F_SHIFT)

/**
 * @brief Mask for GPO1 rising edge deglitch time within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO1_R_MASK                       (uint8_t)(0x03U << PMIC_RDBK_GPO1_R_SHIFT)

/**
 * @brief Mask for GPO2 falling edge deglitch time within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO2_F_MASK                       (uint8_t)(0x03U << PMIC_RDBK_GPO2_F_SHIFT)

/**
 * @brief Mask for GPO2 rising edge deglitch time within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO2_R_MASK                       (uint8_t)(0x03U << PMIC_RDBK_GPO2_R_SHIFT)

/**
 * @brief Bit shift value for GPO3 falling edge deglitch time.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO3_F_SHIFT                      (0X00U)

/**
 * @brief Bit shift value for GPO3 rising edge deglitch time.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO3_R_SHIFT                      (0X02U)

/**
 * @brief Bit shift value for GPO4 falling edge deglitch time.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO4_F_SHIFT                      (0X04U)

/**
 * @brief Bit shift value for GPO4 rising edge deglitch time.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO4_R_SHIFT                      (0X06U)

/**
 * @brief Mask for GPO3 falling edge deglitch time within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO3_F_MASK                       (uint8_t)(0x03U << PMIC_RDBK_GPO3_F_SHIFT)

/**
 * @brief Mask for GPO3 rising edge deglitch time within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO3_R_MASK                       (uint8_t)(0x03U << PMIC_RDBK_GPO3_R_SHIFT)

/**
 * @brief Mask for GPO4 falling edge deglitch time within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO4_F_MASK                       (uint8_t)(0x03U << PMIC_RDBK_GPO4_F_SHIFT)

/**
 * @brief Mask for GPO4 rising edge deglitch time within its configuration register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_RDBK_GPO4_R_MASK                       (uint8_t)(0x03U << PMIC_RDBK_GPO4_R_SHIFT)

/**
 * @brief Bit shift value for the enable output pull-up configuration in GPO_CFG2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG2_EN_OUT_PU_CFG_SHIFT           (6U)

/**
 * @brief Mask for the enable output pull-up configuration in GPO_CFG2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG2_EN_OUT_PU_CFG_MASK            (0x03U << PMIC_GPO_CFG2_EN_OUT_PU_CFG_SHIFT)

/**
 * @brief Value representing high-impedance mode for GPIO output.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_HIGH_IMPEDANCE                     (0x02U)

/**
 * @brief Value representing pull-up from the LDO for GPIO output.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_PULL_UP_LDO                        (0x01U)

/**
 * @brief Value representing pull-up from VDDIO for GPIO output.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_PULL_UP_VDDIO                      (0x00U)

/**
 * @brief Value representing disabled pull-up for GPIO input.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_PULL_DISABLED                     (0x00U)

/**
 * @brief Value representing pull-up enabled for GPIO input.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_PULL_UP                           (0x01U)

/**
 * @brief Value representing pull-up to LDO for GPIO input.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPIO_PULL_UP_TO_LDO                    (0x02U)

/**
 * @brief Register address for GPO_CFG1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG1_REG_ADDR                      (0x7CU)

/**
 * @brief Bit shift value for the M_PMIC_CFG field in GPO_CFG1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG1_M_PMIC_CFG_SHIFT              (6U)

/**
 * @brief Mask for the M_PMIC_CFG field in GPO_CFG1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG1_M_PMIC_CFG_MASK               (0x01U << PMIC_GPO_CFG1_M_PMIC_CFG_SHIFT)

/**
 * @brief Bit shift value for the GPO2_CFG field in GPO_CFG1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG1_GPO2_CFG_SHIFT                (3U)

/**
 * @brief Mask for the GPO2_CFG field in GPO_CFG1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG1_GPO2_CFG_MASK                 (0x07U << PMIC_GPO_CFG1_GPO2_CFG_SHIFT)

/**
 * @brief Bit shift value for the GPO1_CFG field in GPO_CFG1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG1_GPO1_CFG_SHIFT                (0U)

/**
 * @brief Mask for the GPO1_CFG field in GPO_CFG1 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG1_GPO1_CFG_MASK                 (0x07U << PMIC_GPO_CFG1_GPO1_CFG_SHIFT)


/**
 * @brief Register address for GPO_CFG2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG2_REG_ADDR                      (0x7DU)

/**
 * @brief Bit shift value for the GPO_EN field in GPO_CFG2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG2_GPO_EN_SHIFT                  (5U)

/**
 * @brief Mask for the GPO_EN field in GPO_CFG2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG2_GPO_EN_MASK                   (0x06U << PMIC_GPO_CFG2_GPO_EN_SHIFT)

/**
 * @brief Bit shift value for the GPO4_CFG field in GPO_CFG2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG2_GPO4_CFG_SHIFT                (3U)

/**
 * @brief Mask for the GPO4_CFG field in GPO_CFG2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG2_GPO4_CFG_MASK                 (0x07U << PMIC_GPO_CFG2_GPO4_CFG_SHIFT)

/**
 * @brief Bit shift value for the GPO3_CFG field in GPO_CFG2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG2_GPO3_CFG_SHIFT                (0U)

/**
 * @brief Mask for the GPO3_CFG field in GPO_CFG2 register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPO_CFG2_GPO3_CFG_MASK                 (0x07U << PMIC_GPO_CFG2_GPO3_CFG_SHIFT)

/**
 * @brief Register address for GPI_CFG register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPI_CFG_REG_ADDR                       (0x7EU)

/**
 * @brief Bit shift value for the GPI1 field in GPI_CFG register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPI_CFG_GPI1_SHIFT                     (0U)

/**
 * @brief Mask for the GPI1 field in GPI_CFG register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPI_CFG_GPI1_MASK                      (0x1U << PMIC_GPI_CFG_GPI1_SHIFT)

/**
 * @brief Bit shift value for the GPI4 field in GPI_CFG register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPI_CFG_GPI4_SHIFT                     (1U)

/**
 * @brief Mask for the GPI4 field in GPI_CFG register.
 * @ingroup Pmic_GPIOPrivMacros
 */
#define PMIC_GPI_CFG_GPI4_MASK                      (0x3U << PMIC_GPI_CFG_GPI4_SHIFT)

/**
 * @}
 */
/* End of Pmic_GPIOPrivMacros */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @defgroup Pmic_GPIOPrivStructures PMIC GPIO Structures
 * @{
 * @ingroup Pmic_GPIO
 * @brief Contains private structures used in the GPIO module of PMIC driver.
 */

/**
 * @brief Structure to hold GPIO interrupt register configuration.
 *
 * @param   intRegAddr      Address of the interrupt register
 * @param   intRegBitPos    Bit position in the interrupt register
 * @param   intRegPolBitPos Bit position of the interrupt polarity in the register
 *
 * @ingroup Pmic_GPIOPrivStructures
 */
typedef struct Pmic_GpioIntRegCfg_s {
    uint8_t intRegAddr;
    uint8_t intRegBitPos;
    uint8_t intRegPolBitPos;
} Pmic_GpioIntRegCfg_t;

/**
 * @}
 */
/* End of Pmic_GPIOPrivStructures */

/**
 * @}
 */
/* End of Pmic_GPIO */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_GPIO_PRIV_H_ */

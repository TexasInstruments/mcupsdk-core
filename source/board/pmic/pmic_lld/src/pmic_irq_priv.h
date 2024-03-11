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
 * @file   pmic_irq_priv.h
 *
 * @brief  The macro definitions, structures and function prototypes for
 *         configuring PMIC IRQ.
 */

#ifndef PMIC_IRQ_PRIV_H_
#define PMIC_IRQ_PRIV_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#include "pmic_irq.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Pmic_IRQ PMIC Interrupt Request
 * @{
 * @brief Contains definitions related to PMIC IRQ functionality.
 */

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 * @defgroup Pmic_IRQPrivMacros PMIC Interrupt Request Private Macros
 * @{
 * @ingroup Pmic_IRQ
 * @brief Contains private macros used in the IRQ module of PMIC driver.
 */

/**
 * @brief  Interrupt Hierarchy Level 0 Register offsets
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_SAFETY_CFG_REGADDR                             (0x07U)
#define PMIC_RDBK_INT_CFG1_REGADDR                          (0x0DU)
#define PMIC_RDBK_INT_CFG2_REGADDR                          (0x0EU)
#define PMIC_OFF_STATE_STAT1_REGADDR                        (0x0FU)
#define PMIC_OV_INT_CFG1_REGADDR                            (0x35U)
#define PMIC_OV_INT_CFG2_REGADDR                            (0x36U)
#define PMIC_OV_DCDC_CFG_REGADDR                            (0x37U)
#define PMIC_UV_INT_CFG1_REGADDR                            (0x39U)
#define PMIC_UV_INT_CFG2_REGADDR                            (0x3AU)
#define PMIC_UV_DCDC_CFG_REGADDR                            (0x3BU)
#define PMIC_WDG_INT_CFG_REGADDR                            (0x42U)
#define PMIC_WD_ERR_STAT_REGADDR                            (0x46U)
#define PMIC_ESM_CFG_REGADDR                                (0x48U)
#define PMIC_ESM_INT_CFG_REGADDR                            (0x4AU)
#define PMIC_CM_COMP_INT_MSKCFG_REGADDR                     (0x79U)
#define PMIC_CM_VMON_INT_CFG_REGADDR                        (0x7BU)
#define PMIC_INT_UNUSED_REGADDR                             (0xFFU)

/**
 * @brief  PMIC Interrupt MASK register offsets
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_RDBK_INT_MASK_REGADDR                          (0x0CU)
#define PMIC_OV_INT_MASK_REGADDR                            (0x34U)
#define PMIC_UV_INT_MASK_REGADDR                            (0x38U)
#define PMIC_CM_VMON_INT_MASK_REGADDR                       (0x7AU)

/**
 * @brief  SAFETY_CFG Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_SAFETYCFG_CRC_INT_CFG_MASK                     (0x40U)

/**
 * @brief  All Interrupt Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_SAFETY_OFF_STATE_CFG_MASK                      (0x01U)
#define PMIC_RDBK_INT_CFG_MASK                              (0x02U)
#define PMIC_OV_INT_CFG_MASK                                (0x04U)
#define PMIC_UV_INT_CFG_MASK                                (0x08U)
#define PMIC_WD_ERR_STAT_MASK                               (0x10U)
#define PMIC_ESM_INT_CFG_MASK                               (0x20U)
#define PMIC_CM_COMP_INT_MASK_CFG_MASK                      (0x40U)
#define PMIC_CM_VMON_INT_CFG_MASK                           (0x80U)

/**
 * @brief  RDBK_INT_MASK_CFG Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_RDBK_INT_MASK_NRST_RDBK_INT_MASK_MASK          (0x01U)
#define PMIC_RDBK_INT_MASK_SAFE_OUT1_RDBK_INT_MASK_MASK     (0x02U)
#define PMIC_RDBK_INT_MASK_EN_OUT_RDBK_INT_MASK_MASK        (0x04U)
#define PMIC_RDBK_INT_MASK_GPO1_RDBK_INT_MASK_MASK          (0x08U)
#define PMIC_RDBK_INT_MASK_GPO2_RDBK_INT_MASK_MASK          (0x10U)
#define PMIC_RDBK_INT_MASK_GPO3_RDBK_INT_MASK_MASK          (0x20U)
#define PMIC_RDBK_INT_MASK_GPO4_RDBK_INT_MASK_MASK          (0x40U)

/**
 * @brief  RDBK_INT_CFG1 Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_RDBK_INT_CFG1_NRST_RDBK_INT_CFG_MASK           (0x01U)
#define PMIC_RDBK_INT_CFG1_SAFE_OUT1_RDBK_INT_CFG_MASK      (0x04U)
#define PMIC_RDBK_INT_CFG1_EN_OUT_RDBK_INT_CFG_MASK         (0x10U)

/**
 * @brief  RDBK_INT_CFG2 Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_GPO1_RDBK_INT_CFG_MASK                         (0x01U)
#define PMIC_GPO2_RDBK_INT_CFG_MASK                         (0x04U)
#define PMIC_GPO3_RDBK_INT_CFG_MASK                         (0x10U)
#define PMIC_GPO4_RDBK_INT_CFG_MASK                         (0x40U)

/**
 * @brief  OFF_STATE_STAT1 Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_OFF_OFF_INT_EVT_ERR_MASK                       (0x01U)

/**
 * @brief  OV_INT_MASK Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_LDO1_OV_INT_MASK_MASK                          (0x01U)
#define PMIC_LDO2_OV_INT_MASK_MASK                          (0x02U)
#define PMIC_LDO3_OV_INT_MASK_MASK                          (0x04U)
#define PMIC_LDO4_OV_INT_MASK_MASK                          (0x08U)
#define PMIC_PLDO1_OV_INT_MASK_MASK                         (0x10U)
#define PMIC_PLDO2_OV_INT_MASK_MASK                         (0x20U)
#define PMIC_EXT_VMON1_OV_INT_MASK_MASK                     (0x40U)
#define PMIC_EXT_VMON2_OV_INT_MASK_MASK                     (0x80U)

/**
 * @brief  OV_INIT_CFG1 Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_LDO1_OV_INT_CFG_MASK                           (0x01U)
#define PMIC_LDO2_OV_INT_CFG_MASK                           (0x04U)
#define PMIC_LDO3_OV_INT_CFG_MASK                           (0x10U)
#define PMIC_LDO4_OV_INT_CFG_MASK                           (0x40U)

/**
 * @brief  OV_INT_CFG2 Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_PLDO1_OV_INT_CFG_MASK                          (0x01U)
#define PMIC_PLDO2_OV_INT_CFG_MASK                          (0x04U)
#define PMIC_EXT_VMON1_OV_INT_CFG_MASK                      (0x10U)
#define PMIC_EXT_VMON2_OV_INT_CFG_MASK                      (0x40U)

/**
 * @brief  OV_DCDC_CFG Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_BB_OV_INT_MASK_MASK                            (0x01U)
#define PMIC_BB_OV_INT_CFG_MASK                             (0x02U)

/**
 * @brief  UV_INT_MASK Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_LDO1_UV_INT_MASK_MASK                          (0x01U)
#define PMIC_LDO2_UV_INT_MASK_MASK                          (0x02U)
#define PMIC_LDO3_UV_INT_MASK_MASK                          (0x04U)
#define PMIC_LDO4_UV_INT_MASK_MASK                          (0x08U)
#define PMIC_PLDO1_UV_INT_MASK_MASK                         (0x10U)
#define PMIC_PLDO2_UV_INT_MASK_MASK                         (0x20U)
#define PMIC_EXT_VMON1_UV_INT_MASK_MASK                     (0x40U)
#define PMIC_EXT_VMON2_UV_INT_MASK_MASK                     (0x80U)

/**
 * @brief  UV_INT_CFG1 Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_LDO1_UV_INT_CFG_MASK                           (0x01U)
#define PMIC_LDO2_UV_INT_CFG_MASK                           (0x04U)
#define PMIC_LDO3_UV_INT_CFG_MASK                           (0x10U)
#define PMIC_LDO4_UV_INT_CFG_MASK                           (0x40U)

/**
 * @brief  UV_INT_CFG2 Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_PLDO1_UV_INT_CFG_MASK                          (0x01U)
#define PMIC_PLDO2_UV_INT_CFG_MASK                          (0x04U)
#define PMIC_EXT_VMON1_UV_INT_CFG_MASK                      (0x10U)
#define PMIC_EXT_VMON2_UV_INT_CFG_MASK                      (0x40U)

/**
 * @brief  UV_DCDC_CFG Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_BB_UV_INT_MASK_MASK                            (0x01U)
#define PMIC_BB_UV_INT_CFG_MASK                             (0x02U)

/**
 * @brief  WD_INT_CFG Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_WD_TH1_INT_MASK_MASK                           (0x01U)
#define PMIC_WD_TH1_INT_CFG_MASK                            (0x02U)
#define PMIC_WD_TH2_INT_MASK_MASK                           (0x10U)
#define PMIC_WD_TH2_INT_CFG_MASK                            (0x20U)

/**
 * @brief  ESM_CFG1 Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_ESM_CFG1_ESM_ERR_TH_MASK                       (0x00U)
#define PMIC_ESM_CFG1_ESM_EN_MASK                           (0x04U)
#define PMIC_ESM_CFG1_ESM_CFG_MASK                          (0x07U)

/**
 * @brief  ESM_INT_CFG Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_ESM_INT_MASK_MASK                              (0x01U)
#define PMIC_ESM_DLY1_INT_MASK_MASK                         (0x02U)
#define PMIC_ESM_DLY1_INT_CFG_MASK                          (0x04U)
#define PMIC_ESM_DLY2_INT_MASK_MASK                         (0x10U)
#define PMIC_ESM_DLY2_INT_CFG_MASK                          (0x20U)

/**
 * @brief  CM_COMP_INT_MASK_CFG Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_COMP1_INT_MASK_MASK                            (0x01U)
#define PMIC_COMP2_INT_MASK_MASK                            (0x02U)
#define PMIC_COMP1_INT_CFG_MASK                             (0x10U)
#define PMIC_COMP2_INT_CFG_MASK                             (0x40U)

/**
 * @brief  CM_VMON_INT_MASK Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_COMP1P_UV_INT_MASK_MASK                        (0x01U)
#define PMIC_COMP1P_OV_INT_MASK_MASK                        (0x02U)
#define PMIC_COMP1N_UV_INT_MASK_MASK                        (0x04U)
#define PMIC_COMP1N_OV_INT_MASK_MASK                        (0x08U)
#define PMIC_COMP2P_UV_INT_MASK_MASK                        (0x10U)
#define PMIC_COMP2P_OV_INT_MASK_MASK                        (0x20U)
#define PMIC_COMP2N_UV_INT_MASK_MASK                        (0x40U)
#define PMIC_COMP2N_OV_INT_MASK_MASK                        (0x80U)

/**
 * @brief  CM_VMON_INT_CFG Register Bit Masks
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_COMP1P_UV_INT_CFG_MASK                         (0x01U)
#define PMIC_COMP1P_OV_INT_CFG_MASK                         (0x02U)
#define PMIC_COMP1N_UV_INT_CFG_MASK                         (0x04U)
#define PMIC_COMP1N_OV_INT_CFG_MASK                         (0x08U)
#define PMIC_COMP2P_UV_INT_CFG_MASK                         (0x10U)
#define PMIC_COMP2P_OV_INT_CFG_MASK                         (0x20U)
#define PMIC_COMP2N_UV_INT_CFG_MASK                         (0x40U)
#define PMIC_COMP2N_OV_INT_CFG_MASK                         (0x80U)


/**
 * @brief  SAFETY_CFG Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_CFG_REG_CRC_INT_CFG_SHIFT                      (0x06U)

/**
 * @brief  RDBK_INT_MASK_CFG Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_NRST_RDBK_INT_MASK_SHIFT                       (0x00U)
#define PMIC_SFOUT1_RDBK_INT_MASK_SHIFT                     (0x01U)
#define PMIC_EN_OUT_RDBK_INT_MASK_SHIFT                     (0x02U)
#define PMIC_GPO1_RDBK_INT_MASK_SHIFT                       (0x03U)
#define PMIC_GPO2_RDBK_INT_MASK_SHIFT                       (0x04U)
#define PMIC_GPO3_RDBK_INT_MASK_SHIFT                       (0x05U)
#define PMIC_GPO4_RDBK_INT_MASK_SHIFT                       (0x06U)

/**
 * @brief  RDBK_INT_CFG1 Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_NRST_RDBK_INT_CFG_SHIFT                        (0x00U)
#define PMIC_SFOUT1_RDBK_INT_CFG_SHIFT                      (0x02U)
#define PMIC_EN_OUT_RDBK_INT_CFG_SHIFT                      (0x04U)

/**
 * @brief  RDBK_INT_CFG2 Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_GPO1_RDBK_INT_CFG_SHIFT                        (0x00U)
#define PMIC_GPO2_RDBK_INT_CFG_SHIFT                        (0x02U)
#define PMIC_GPO3_RDBK_INT_CFG_SHIFT                        (0x04U)
#define PMIC_GPO4_RDBK_INT_CFG_SHIFT                        (0x06U)

/**
 * @brief  OFF_STATE_STAT1 Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_OFF_INT_EVT_ERR_SHIFT                          (0x01U)

/**
 * @brief  OV_INT_MASK Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_LDO1_OV_INT_MASK_SHIFT                         (0x00U)
#define PMIC_LDO2_OV_INT_MASK_SHIFT                         (0x01U)
#define PMIC_LDO3_OV_INT_MASK_SHIFT                         (0x02U)
#define PMIC_LDO4_OV_INT_MASK_SHIFT                         (0x03U)
#define PMIC_PLDO1_OV_INT_MASK_SHIFT                        (0x04U)
#define PMIC_PLDO2_OV_INT_MASK_SHIFT                        (0x05U)
#define PMIC_EXTVMON1_OV_INT_MASK_SHIFT                     (0x06U)
#define PMIC_EXTVMON2_OV_INT_MASK_SHIFT                     (0x07U)

/**
 * @brief  OV_INster Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_LDO1_OV_INT_CFG_SHIFT                          (0x00U)
#define PMIC_LDO2_OV_INT_CFG_SHIFT                          (0x02U)
#define PMIC_LDO3_OV_INT_CFG_SHIFT                          (0x04U)
#define PMIC_LDO4_OV_INT_CFG_SHIFT                          (0x06U)

/**
 * @brief  OV_INT_CFG2 Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_PLDO1_OV_INT_CFG_SHIFT                         (0x00U)
#define PMIC_PLDO2_OV_INT_CFG_SHIFT                         (0x02U)
#define PMIC_EXT_VMON1_OV_INT_CFG_SHIFT                     (0x04U)
#define PMIC_EXT_VMON2_OV_INT_CFG_SHIFT                     (0x06U)

/**
 * @brief  OV_DCDC_CFG Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_BB_OV_INT_MASK_SHIFT                           (0x00U)
#define PMIC_BB_OV_INT_CFG_SHIFT                            (0x01U)

/**
 * @brief  UV_INT_MASK Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_LDO1_UV_INT_MASK_SHIFT                         (0x00U)
#define PMIC_LDO2_UV_INT_MASK_SHIFT                         (0x01U)
#define PMIC_LDO3_UV_INT_MASK_SHIFT                         (0x02U)
#define PMIC_LDO4_UV_INT_MASK_SHIFT                         (0x03U)
#define PMIC_PLDO1_UV_INT_MASK_SHIFT                        (0x04U)
#define PMIC_PLDO2_UV_INT_MASK_SHIFT                        (0x05U)
#define PMIC_EXTVMON1_UV_INT_MASK_SHIFT                     (0x06U)
#define PMIC_EXTVMON2_UV_INT_MASK_SHIFT                     (0x07U)

/**
 * @brief  UV_INT_CFG1 Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_LDO1_UV_INT_CFG_SHIFT                          (0x00U)
#define PMIC_LDO2_UV_INT_CFG_SHIFT                          (0x02U)
#define PMIC_LDO3_UV_INT_CFG_SHIFT                          (0x04U)
#define PMIC_LDO4_UV_INT_CFG_SHIFT                          (0x06U)

/**
 * @brief  UV_INT_CFG2 Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_PLDO1_UV_INT_CFG_SHIFT                         (0x00U)
#define PMIC_PLDO2_UV_INT_CFG_SHIFT                         (0x02U)
#define PMIC_EXT_VMON1_UV_INT_CFG_SHIFT                     (0x04U)
#define PMIC_EXT_VMON2_UV_INT_CFG_SHIFT                     (0x06U)

/**
 * @brief  UV_DCDC_CFG Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_BB_UV_INT_MASK_SHIFT                           (0x00U)
#define PMIC_BB_UV_INT_CFG_SHIFT                            (0x01U)

/**
 * @brief  WD_INT_CFG Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_WD_TH1_INT_MASK_SHIFT                          (0x00U)
#define PMIC_WD_TH1_INT_CFG_SHIFT                           (0x01U)
#define PMIC_WD_TH2_INT_MASK_SHIFT                          (0x04U)
#define PMIC_WD_TH2_INT_CFG_SHIFT                           (0x05U)

/**
 * @brief  ESM_CFG1 Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_ESM_CFG1_ESM_ERR_TH_SHIFT                      (0x00U)
#define PMIC_ESM_CFG1_ESM_EN_SHIFT                          (0x04U)
#define PMIC_ESM_CFG1_ESM_CFG_SHIFT                         (0x07U)

/**
 * @brief  ESM_INT_CFG Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_ESM_INT_MASK_SHIFT                             (0x00U)
#define PMIC_ESM_DLY1_INT_MASK_SHIFT                        (0x01U)
#define PMIC_ESM_DLY1_INT_CFG_SHIFT                         (0x02U)
#define PMIC_ESM_DLY2_INT_MASK_SHIFT                        (0x04U)
#define PMIC_ESM_DLY2_INT_CFG_SHIFT                         (0x05U)

/**
 * @brief  CM_COMP_INT_MASK_CFG Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_COMP1_INT_MASK_SHIFT                           (0x00U)
#define PMIC_COMP2_INT_MASK_SHIFT                           (0x01U)
#define PMIC_COMP1_INT_CFG_SHIFT                            (0x04U)
#define PMIC_COMP2_INT_CFG_SHIFT                            (0x06U)

/**
 * @brief  CM_VMON_INT_MASK Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_COMP1P_UV_INT_MASK_SHIFT                       (0x00U)
#define PMIC_COMP1P_OV_INT_MASK_SHIFT                       (0x01U)
#define PMIC_COMP1N_UV_INT_MASK_SHIFT                       (0x02U)
#define PMIC_COMP1N_OV_INT_MASK_SHIFT                       (0x03U)
#define PMIC_COMP2P_UV_INT_MASK_SHIFT                       (0x04U)
#define PMIC_COMP2P_OV_INT_MASK_SHIFT                       (0x05U)
#define PMIC_COMP2N_UV_INT_MASK_SHIFT                       (0x06U)
#define PMIC_COMP2N_OV_INT_MASK_SHIFT                       (0x07U)

/**
 * @brief  CM_VMON_INT_CFG Register Bit Positions
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_COMP1P_UV_INT_CFG_SHIFT                        (0x00U)
#define PMIC_COMP1P_OV_INT_CFG_SHIFT                        (0x01U)
#define PMIC_COMP1N_UV_INT_CFG_SHIFT                        (0x02U)
#define PMIC_COMP1N_OV_INT_CFG_SHIFT                        (0x03U)
#define PMIC_COMP2P_UV_INT_CFG_SHIFT                        (0x04U)
#define PMIC_COMP2P_OV_INT_CFG_SHIFT                        (0x05U)
#define PMIC_COMP2N_UV_INT_CFG_SHIFT                        (0x06U)
#define PMIC_COMP2N_OV_INT_CFG_SHIFT                        (0x07U)

/**
 * @brief   PMIC IRQ invalid macros
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_IRQ_INVALID_REGADDR                            (0x0U)
#define PMIC_IRQ_INVALID_BIT_SHIFT                          (0x0U)
#define PMIC_INVALID_DEVICE                                 (0xFFU)

/**
 * @brief Bit field Value for intrMaskBitPos/intrClrBitPos/
 *        gpioRiseMaskBitPos/gpioFallMaskBitPos
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_IRQ_MASK_CLR_BITFIELD                          (1U)

/**
 * @brief Mask Value of the PMIC IRQ
 *
 * @ingroup Pmic_IRQPrivMacros
 */
#define PMIC_IRQ_MASK_VAL_1                                 (1U)

/**
 * @}
 */
/* End of Pmic_IRQPrivMacros */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @defgroup Pmic_IRQPrivStructures PMIC IRQ Private Structures
 * @{
 * @ingroup Pmic_IRQ
 * @brief Contains private structures used in the IRQ module of PMIC driver.
 */


/**
 * @brief Structure for configuring PMIC interrupt handling.
 *
 * @ingroup Pmic_IRQPrivStructures
 */
typedef struct Pmic_IntrCfg_s {
    uint16_t intrClrRegAddr; /**< Address of the interrupt clear register. */
    uint8_t intrClrBitPos;
    /**< Bit position of the interrupt in the clear
                               register. */
    uint16_t intrMaskRegAddr; /**< Address of the interrupt mask register. */
    uint8_t intrMaskBitPos;
    /**< Bit position of the interrupt in the mask
                                register. */
}
Pmic_IntrCfg_t;

/**
 * @brief Structure for configuring GPIO interrupt type.
 */
typedef struct Pmic_GpioIntrTypeCfg_s {
    uint8_t gpioIntrMaskRegAddr; /**< Address of the GPIO interrupt mask register. */
    uint8_t gpioMaskBitPos;      /**< Bit position of the GPIO interrupt mask. */
}
Pmic_GpioIntrTypeCfg_t;

/**
 * @}
 */
/* End of Pmic_IRQPrivStructures */

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @}
 */
/* End of Pmic_IRQ */

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_IRQ_PRIV_H_ */

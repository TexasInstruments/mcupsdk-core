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
 * @file   pmic_irq_tps6594x.h
 *
 * @brief  TPS65386x BlackBird PMIC IRQ Driver API/interface file.
 *
 */

#ifndef PMIC_IRQ_TPS65386X_H_
#define PMIC_IRQ_TPS65386X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_irq_tps65386x_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Pmic_IRQ PMIC Interrupt Request
 * @{
 * @brief Contains definitions related to PMIC IRQ functionality.
 */

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @defgroup Pmic_BB_IRQMacros PMIC Interrupt Request Blackbird Macros
 * @{
 * @ingroup Pmic_IRQ
 * @brief Contains black-bird TPS65386 macros used in the IRQ module of PMIC driver.
 */

/*!
 * @brief Configuration register index for CRC interrupt configuration in the PMIC.
 * This constant defines the index value used for CRC interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_CFG_REG_CRC_INT_CFG         (0U)

/*!
 * @brief Configuration register index for NRST RDBK interrupt configuration in the PMIC.
 * This constant defines the index value used for NRST RDBK interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_NRST_RDBK_INT_CFG           (1U)

/*!
 * @brief Configuration register index for SAFE OUT1 RDBK interrupt configuration in the PMIC.
 * This constant defines the index value used for SAFE OUT1 RDBK interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_SAFE_OUT1_RDBK_INT_CFG      (2U)

/*!
 * @brief Configuration register index for EN OUT RDBK interrupt configuration in the PMIC.
 * This constant defines the index value used for EN OUT RDBK interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_EN_OUT_RDBK_INT_CFG         (3U)

/*!
 * @brief Configuration register index for GPO1 RDBK interrupt configuration in the PMIC.
 * This constant defines the index value used for GPO1 RDBK interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_GPO1_RDBK_INT_CFG           (4U)

/*!
 * @brief Configuration register index for GPO2 RDBK interrupt configuration in the PMIC.
 * This constant defines the index value used for GPO2 RDBK interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_GPO2_RDBK_INT_CFG           (5U)

/*!
 * @brief Configuration register index for GPO3 RDBK interrupt configuration in the PMIC.
 * This constant defines the index value used for GPO3 RDBK interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_GPO3_RDBK_INT_CFG           (6U)

/*!
 * @brief Configuration register index for GPO4 RDBK interrupt configuration in the PMIC.
 * This constant defines the index value used for GPO4 RDBK interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_GPO4_RDBK_INT_CFG           (7U)

/*!
 * @brief Configuration register index for OFF interrupt event error in the PMIC.
 * This constant defines the index value used for OFF interrupt event error in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_OFF_INT_EVT_ERR             (8U)

/*!
 * @brief Configuration register index for LDO1 OV interrupt configuration in the PMIC.
 * This constant defines the index value used for LDO1 OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO1_OV_INT_CFG             (9U)

/*!
 * @brief Configuration register index for LDO2 OV interrupt configuration in the PMIC.
 * This constant defines the index value used for LDO2 OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO2_OV_INT_CFG             (10U)

/*!
 * @brief Configuration register index for LDO3 OV interrupt configuration in the PMIC.
 * This constant defines the index value used for LDO3 OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO3_OV_INT_CFG             (11U)

/*!
 * @brief Configuration register index for LDO4 OV interrupt configuration in the PMIC.
 * This constant defines the index value used for LDO4 OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO4_OV_INT_CFG             (12U)

/*!
 * @brief Configuration register index for PLDO1 OV interrupt configuration in the PMIC.
 * This constant defines the index value used for PLDO1 OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_PLDO1_OV_INT_CFG            (13U)

/*!
 * @brief Configuration register index for PLDO2 OV interrupt configuration in the PMIC.
 * This constant defines the index value used for PLDO2 OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_PLDO2_OV_INT_CFG            (14U)

/*!
 * @brief Configuration register index for external VMON1 OV interrupt configuration in the PMIC.
 * This constant defines the index value used for external VMON1 OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_EXT_VMON1_OV_INT_CFG        (15U)

/*!
 * @brief Configuration register index for external VMON2 OV interrupt configuration in the PMIC.
 * This constant defines the index value used for external VMON2 OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_EXT_VMON2_OV_INT_CFG        (16U)

/*!
 * @brief Configuration register index for BB OV interrupt configuration in the PMIC.
 * This constant defines the index value used for BB OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_BB_OV_INT_CFG               (17U)

/*!
 * @brief Configuration register index for LDO1 UV interrupt configuration in the PMIC.
 * This constant defines the index value used for LDO1 UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO1_UV_INT_CFG             (18U)

/*!
 * @brief Configuration register index for LDO2 UV interrupt configuration in the PMIC.
 * This constant defines the index value used for LDO2 UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO2_UV_INT_CFG             (19U)

/*!
 * @brief Configuration register index for LDO3 UV interrupt configuration in the PMIC.
 * This constant defines the index value used for LDO3 UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO3_UV_INT_CFG             (20U)

/*!
 * @brief Configuration register index for LDO4 UV interrupt configuration in the PMIC.
 * This constant defines the index value used for LDO4 UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO4_UV_INT_CFG             (21U)

/*!
 * @brief Configuration register index for PLDO1 UV interrupt configuration in the PMIC.
 * This constant defines the index value used for PLDO1 UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_PLDO1_UV_INT_CFG            (22U)

/*!
 * @brief Configuration register index for PLDO2 UV interrupt configuration in the PMIC.
 * This constant defines the index value used for PLDO2 UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_PLDO2_UV_INT_CFG            (23U)

/*!
 * @brief Configuration register index for external VMON1 UV interrupt configuration in the PMIC.
 * This constant defines the index value used for external VMON1 UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_EXT_VMON1_UV_INT_CFG        (24U)

/*!
 * @brief Configuration register index for external VMON2 UV interrupt configuration in the PMIC.
 * This constant defines the index value used for external VMON2 UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_EXT_VMON2_UV_INT_CFG        (25U)

/*!
 * @brief Configuration register index for BB UV interrupt configuration in the PMIC.
 * This constant defines the index value used for BB UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_BB_UV_INT_CFG               (26U)

/*!
 * @brief Configuration register index for Watchdog 1 threshold interrupt configuration in the PMIC.
 * This constant defines the index value used for Watchdog 1 threshold interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_WD_TH1_INT_CFG              (27U)

/*!
 * @brief Configuration register index for Watchdog 2 threshold interrupt configuration in the PMIC.
 * This constant defines the index value used for Watchdog 2 threshold interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_WD_TH2_INT_CFG              (28U)

/*!
 * @brief Configuration register index for ESM delay 1 interrupt configuration in the PMIC.
 * This constant defines the index value used for ESM delay 1 interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_ESM_DLY1_INT_CFG            (29U)

/*!
 * @brief Configuration register index for ESM delay 2 interrupt configuration in the PMIC.
 * This constant defines the index value used for ESM delay 2 interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_ESM_DLY2_INT_CFG            (30U)

/*!
 * @brief Configuration register index for Comparator 1 interrupt configuration in the PMIC.
 * This constant defines the index value used for Comparator 1 interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP1_INT_CFG               (31U)

/*!
 * @brief Configuration register index for Comparator 2 interrupt configuration in the PMIC.
 * This constant defines the index value used for Comparator 2 interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP2_INT_CFG               (32U)

/*!
 * @brief Configuration register index for Comparator 1 positive UV interrupt configuration in the PMIC.
 * This constant defines the index value used for Comparator 1 positive UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP1P_UV_INT_CFG           (33U)

/*!
 * @brief Configuration register index for Comparator 1 positive OV interrupt configuration in the PMIC.
 * This constant defines the index value used for Comparator 1 positive OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP1P_OV_INT_CFG           (34U)

/*!
 * @brief Configuration register index for Comparator 1 negative UV interrupt configuration in the PMIC.
 * This constant defines the index value used for Comparator 1 negative UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP1N_UV_INT_CFG           (35U)

/*!
 * @brief Configuration register index for Comparator 1 negative OV interrupt configuration in the PMIC.
 * This constant defines the index value used for Comparator 1 negative OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP1N_OV_INT_CFG           (36U)

/*!
 * @brief Configuration register index for Comparator 2 positive UV interrupt configuration in the PMIC.
 * This constant defines the index value used for Comparator 2 positive UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP2P_UV_INT_CFG           (37U)

/*!
 * @brief Configuration register index for Comparator 2 positive OV interrupt configuration in the PMIC.
 * This constant defines the index value used for Comparator 2 positive OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */

#define PMIC_BB_COMP2P_OV_INT_CFG           (38U)

/*!
 * @brief Configuration register index for Comparator 2 negative UV interrupt configuration in the PMIC.
 * This constant defines the index value used for Comparator 2 negative UV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP2N_UV_INT_CFG           (39U)

/*!
 * @brief Configuration register index for Comparator 2 negative OV interrupt configuration in the PMIC.
 * This constant defines the index value used for Comparator 2 negative OV interrupt configuration in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP2N_OV_INT_CFG           (40U)

/*!
 * @brief Interrupt mask index for NRST RDBK interrupt in the PMIC.
 * This constant defines the index value used for NRST RDBK interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_NRST_RDBK_INT_MASK          (41U)

/*!
 * @brief Interrupt mask index for SAFE OUT1 RDBK interrupt in the PMIC.
 * This constant defines the index value used for SAFE OUT1 RDBK interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_SAFE_OUT1_RDBK_INT_MASK     (42U)

/*!
 * @brief Interrupt mask index for EN OUT RDBK interrupt in the PMIC.
 * This constant defines the index value used for EN OUT RDBK interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_EN_OUT_RDBK_INT_MASK        (43U)

/*!
 * @brief Interrupt mask index for GPO1 RDBK interrupt in the PMIC.
 * This constant defines the index value used for GPO1 RDBK interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_GPO1_RDBK_INT_MASK          (44U)

/*!
 * @brief Interrupt mask index for GPO2 RDBK interrupt in the PMIC.
 * This constant defines the index value used for GPO2 RDBK interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_GPO2_RDBK_INT_MASK          (45U)

/*!
 * @brief Interrupt mask index for GPO3 RDBK interrupt in the PMIC.
 * This constant defines the index value used for GPO3 RDBK interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_GPO3_RDBK_INT_MASK          (46U)

/*!
 * @brief Interrupt mask index for GPO4 RDBK interrupt in the PMIC.
 * This constant defines the index value used for GPO4 RDBK interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_GPO4_RDBK_INT_MASK          (47U)

/*!
 * @brief Interrupt mask index for LDO1 OV interrupt in the PMIC.
 * This constant defines the index value used for LDO1 OV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO1_OV_INT_MASK            (48U)

/*!
 * @brief Interrupt mask index for LDO2 OV interrupt in the PMIC.
 * This constant defines the index value used for LDO2 OV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO2_OV_INT_MASK            (49U)

/*!
 * @brief Interrupt mask index for LDO3 OV interrupt in the PMIC.
 * This constant defines the index value used for LDO3 OV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO3_OV_INT_MASK            (50U)

/*!
 * @brief Interrupt mask index for LDO4 OV interrupt in the PMIC.
 * This constant defines the index value used for LDO4 OV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO4_OV_INT_MASK            (51U)

/*!
 * @brief Interrupt mask index for PLDO1 OV interrupt in the PMIC.
 * This constant defines the index value used for PLDO1 OV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_PLDO1_OV_INT_MASK           (52U)

/*!
 * @brief Interrupt mask index for PLDO2 OV interrupt in the PMIC.
 * This constant defines the index value used for PLDO2 OV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_PLDO2_OV_INT_MASK           (54U)

/*!
 * @brief Interrupt mask index for external VMON2 OV interrupt in the PMIC.
 * This constant defines the index value used for external VMON2 OV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */

#define PMIC_BB_EXT_VMON2_OV_INT_MASK       (55U)

/*!
 * @brief Interrupt mask index for BB OV interrupt in the PMIC.
 * This constant defines the index value used for BB OV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_BB_OV_INT_MASK              (56U)

/*!
 * @brief Interrupt mask index for LDO1 UV interrupt in the PMIC.
 * This constant defines the index value used for LDO1 UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO1_UV_INT_MASK            (57U)

/*!
 * @brief Interrupt mask index for LDO2 UV interrupt in the PMIC.
 * This constant defines the index value used for LDO2 UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO2_UV_INT_MASK            (58U)

/*!
 * @brief Interrupt mask index for LDO3 UV interrupt in the PMIC.
 * This constant defines the index value used for LDO3 UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO3_UV_INT_MASK            (59U)

/*!
 * @brief Interrupt mask index for LDO4 UV interrupt in the PMIC.
 * This constant defines the index value used for LDO4 UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_LDO4_UV_INT_MASK            (60U)

/*!
 * @brief Interrupt mask index for PLDO1 UV interrupt in the PMIC.
 * This constant defines the index value used for PLDO1 UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_PLDO1_UV_INT_MASK           (61U)

/*!
 * @brief Interrupt mask index for PLDO2 UV interrupt in the PMIC.
 * This constant defines the index value used for PLDO2 UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_PLDO2_UV_INT_MASK           (62U)

/*!
 * @brief Interrupt mask index for external VMON1 UV interrupt in the PMIC.
 * This constant defines the index value used for external VMON1 UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_EXT_VMON1_UV_INT_MASK       (63U)

/*!
 * @brief Interrupt mask index for external VMON2 UV interrupt in the PMIC.
 * This constant defines the index value used for external VMON2 UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_EXT_VMON2_UV_INT_MASK       (64U)

/*!
 * @brief Interrupt mask index for BB UV interrupt in the PMIC.
 * This constant defines the index value used for BB UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_BB_UV_INT_MASK              (65U)

/*!
 * @brief Interrupt mask index for Watchdog 1 threshold interrupt in the PMIC.
 * This constant defines the index value used for Watchdog 1 threshold interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_WD_TH1_INT_MASK             (66U)

/*!
 * @brief Interrupt mask index for Watchdog 2 threshold interrupt in the PMIC.
 * This constant defines the index value used for Watchdog 2 threshold interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_WD_TH2_INT_MASK             (67U)

/*!
 * @brief Interrupt mask index for ESM interrupt in the PMIC.
 * This constant defines the index value used for ESM interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_ESM_INT_MASK                (68U)

/*!
 * @brief Interrupt mask index for ESM delay 1 interrupt in the PMIC.
 * This constant defines the index value used for ESM delay 1 interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_ESM_DLY1_INT_MASK           (69U)

/*!
 * @brief Interrupt mask index for ESM delay 2 interrupt in the PMIC.
 * This constant defines the index value used for ESM delay 2 interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_ESM_DLY2_INT_MASK           (70U)

/*!
 * @brief Interrupt mask index for Comparator 1 interrupt in the PMIC.
 * This constant defines the index value used for Comparator 1 interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP1_INT_MASK              (71U)

/*!
 * @brief Interrupt mask index for Comparator 2 interrupt in the PMIC.
 * This constant defines the index value used for Comparator 2 interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP2_INT_MASK              (72U)

/*!
 * @brief Interrupt mask index for Comparator 1 positive UV interrupt in the PMIC.
 * This constant defines the index value used for Comparator 1 positive UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP1P_UV_INT_MASK          (73U)

/*!
 * @brief Interrupt mask index for Comparator 1 positive OV interrupt in the PMIC.
 * This constant defines the index value used for Comparator 1 positive OV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP1P_OV_INT_MASK          (74U)

/*!
 * @brief Interrupt mask index for Comparator 1 negative UV interrupt in the PMIC.
 * This constant defines the index value used for Comparator 1 negative UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP1N_UV_INT_MASK          (75U)

/*!
 * @brief Interrupt mask index for Comparator 1 negative OV interrupt in the PMIC.
 * This constant defines the index value used for Comparator 1 negative OV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP1N_OV_INT_MASK          (76U)

/*!
 * @brief Interrupt mask index for Comparator 2 positive UV interrupt in the PMIC.
 * This constant defines the index value used for Comparator 2 positive UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP2P_UV_INT_MASK          (77U)

/*!
 * @brief Interrupt mask index for Comparator 2 positive OV interrupt in the PMIC.
 * This constant defines the index value used for Comparator 2 positive OV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP2P_OV_INT_MASK          (78U)

/*!
 * @brief Interrupt mask index for Comparator 2 negative UV interrupt in the PMIC.
 * This constant defines the index value used for Comparator 2 negative UV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP2N_UV_INT_MASK          (79U)

/*!
 * @brief Interrupt mask index for Comparator 2 negative OV interrupt in the PMIC.
 * This constant defines the index value used for Comparator 2 negative OV interrupt mask in the PMIC.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_COMP2N_OV_INT_MASK          (80U)

/*!
 * @brief Maximum number of interrupts in the PMIC BB.
 * This constant defines the maximum number of interrupts in the PMIC BB.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_IRQ_MAX_NUM                 (81U)

/*!
 * @brief Interrupt mask number for GPO 0 in the PMIC BB.
 * This constant defines the interrupt mask number for GPO 0 in the PMIC BB.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_IRQ_GPO_0_INT_MASK_NUM      (0U)

/*!
 * @brief Interrupt mask number for GPO 1 in the PMIC BB.
 * This constant defines the interrupt mask number for GPO 1 in the PMIC BB.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_IRQ_GPO_1_INT_MASK_NUM      (1U)

/*!
 * @brief Interrupt mask number for GPO 2 in the PMIC BB.
 * This constant defines the interrupt mask number for GPO 2 in the PMIC BB.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_IRQ_GPO_2_INT_MASK_NUM      (2U)

/*!
 * @brief Interrupt mask number for GPO 3 in the PMIC BB.
 * This constant defines the interrupt mask number for GPO 3 in the PMIC BB.
 * @ingroup Pmic_BB_IRQMacros
 */
#define PMIC_BB_IRQ_GPO_3_INT_MASK_NUM      (3U)


/**
 * @}
 */
/* End of Pmic_BB_IRQMacros */

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @defgroup Pmic_BB_IRQFunctions PMIC Interrupt Request Blackbird Functions
 * @{
 * @ingroup Pmic_IRQ
 * @brief Contains black-bird TPS65386 functions used in the IRQ module of PMIC driver.
 */

/**
 * @brief   Get TPS65386x Interrupt config.
 *          This function is used to get TPS65386x Interrupt configuration.
 *
 * @param   pIntrCfg to store tps65386x Interrupt configuration.
 * @ingroup Pmic_BB_IRQFunctions
 */
void pmic_get_bb_intrCfg(Pmic_IntrCfg_t ** pIntrCfg);

/**
 * @brief   Get TPS65386x Interrupt config.
 *          This function is used to get TPS65386x Interrupt configuration.
 *
* @param   pGpioIntrCfg to store tps65386x Interrupt configuration.
* @ingroup Pmic_BB_IRQFunctions
*/
 void pmic_get_bb_intrGpioCfg(Pmic_GpioIntrTypeCfg_t ** pGpioIntrCfg);


int32_t Pmic_BB_irqGetL2Error(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint16_t l1RegAddr, Pmic_IrqStatus_t * pErrStat);


static int32_t Pmic_bb_getCMVMONErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat);


static int32_t Pmic_bb_getCOMPErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat);


static int32_t Pmic_bb_getESMErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat);


static int32_t Pmic_bb_getOV_VMONPLDOErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat);


static int32_t Pmic_bb_getRDBKErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t regValue, Pmic_IrqStatus_t * pErrStat);


static int32_t Pmic_bb_getUV_LDO_Err(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat);


static int32_t Pmic_bb_getUV_VMONPLDOErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat);


static int32_t Pmic_bb_getOV_LDO_Err(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat);


static int32_t
Pmic_bb_getOV_LDO_VMON_PLDOErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t regValue, Pmic_IrqStatus_t * pErrStat);


static int32_t
Pmic_bb_getUV_LDO_VMON_PLDOErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t regValue, Pmic_IrqStatus_t * pErrStat);


static int32_t Pmic_bb_getWDGErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat);


static int32_t Pmic_bb_getSafetyOffStateErr(uint8_t regValue,
    Pmic_IrqStatus_t * pErrStat);


int32_t Pmic_bb_irqGetL2Error(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint16_t l1RegAddr, Pmic_IrqStatus_t * pErrStat);


void pmic_get_bb_intrCfg(Pmic_IntrCfg_t ** pIntrCfg);


void pmic_get_bb_intrGpioCfg(Pmic_GpioIntrTypeCfg_t ** pGpioIntrCfg);


void Pmic_BB_reInitInterruptConfig(void);

/**
 * @}
 */
/* End of Pmic_BB_IRQFunctions */

/**
 * @}
 */
/* End of Pmic_IRQ */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_IRQ_TPS65386X_H_ */

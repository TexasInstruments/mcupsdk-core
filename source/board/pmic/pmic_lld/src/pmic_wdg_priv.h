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

#ifndef PMIC_INC_PMIC_WDG_PRIV_H_
#define PMIC_INC_PMIC_WDG_PRIV_H_

/* ==========================================================================*/
/*                             Include Files                                 */
/* ==========================================================================*/

#include "pmic.h"
#include "pmic_core.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @brief  PMIC Watch Dog Register Offsets
 */
#define PMIC_WD_CFG_REGADDR                             (0x3CU)
#define PMIC_WD_LONGWIN_CFG_REGADDR                     (0x3DU)
#define PMIC_WD_WIN1_CFG_REGADDR                        (0x3EU)
#define PMIC_WD_WIN2_CFG_REGADDR                        (0x3FU)
#define PMIC_WD_THR_CFG_REGADDR                         (0x40U)
#define PMIC_WD_QA_CFG_REGADDR                          (0x41U)
#define PMIC_WD_THR_INT_CFG_REGADDR                     (0x42U)
#define PMIC_WD_QA_CNT_REGADDR                          (0x43U)
#define PMIC_WD_ANSWER_REG_REGADDR                      (0x44U)
#define PMIC_WD_STATUS_REGADDR                          (0x45U)
#define PMIC_WD_ERR_STATUS_REGADDR                      (0x46U)

/**
 * @brief  PMIC Watch Dog Register Shift Values
 */
/* Watch Dog Configuration Register Shift Values */
#define PMIC_WD_EN_SHIFT                                (0x03U)
#define PMIC_WD_PWRHOLD_SHIFT                           (0x02U)
#define PMIC_WD_MODE_SHIFT                              (0x01U)
#define PMIC_WD_RETURN_LONGWIN_SHIFT                    (0x00U)

/* Watch Dog Long Window Time Configuration Shift Values */
#define PMIC_WD_LONGWIN_SHIFT                           (0x00U)

/* Watch Dog Window 1 Configuration Shift Values */
#define PMIC_WD_WIN1_CFG_WD_WIN1_SHIFT                  (0x00U)

/* Watch Dog Window 2 Configuration Shift Values */
#define PMIC_WD_WIN2_CFG_WD_WIN2_SHIFT                  (0x00U)

/* Watch Dog Threshold Value Configuration Shift Values*/
#define PMIC_WD_THR_CFG_WD_TH1_SHIFT                    (0x03U)
#define PMIC_WD_THR_CFG_WD_TH2_SHIFT                    (0x00U)

/* Watch Dog QA Configuration Shift Values */
#define PMIC_WD_QA_CFG_WD_QUESTION_SEED_SHIFT           (0x00U)
#define PMIC_WD_QA_CFG_WD_QA_LFSR_SHIFT                 (0x04U)
#define PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT                 (0x06U)

/* Watch Dog Threshold Interrupt Configuration Shift Values */
#define PMIC_WD_THR2_INT_CFG_SHIFT                      (0x05U)
#define PMIC_WD_THR2_INT_CFG_MASK_SHIFT                 (0x04U)
#define PMIC_WD_THR1_INT_CFG_SHIFT                      (0x01U)
#define PMIC_WD_THR1_INT_CFG_MASK_SHIFT                 (0x00U)

/* Watch Dog Question Answer Count Register Shift Values */
#define PMIC_WD_QUESTION_ANSW_CNT_WD_ANSW_CNT_SHIFT     (0x04U)
#define PMIC_WD_QUESTION_ANSW_CNT_WD_QUESTION_SHIFT     (0x00U)

/* Watch Dog Answer byte Register Shift Values */
#define PMIC_WD_ANSWER_SHIFT                            (0x00U)

/* Watch Dog Status Register Shift Values */
#define PMIC_WD_STAT_REG_WD_FIRST_OK_SHIFT              (0x07U)
#define PMIC_WD_STAT_REG_WD_LONGWIN_ACTIVE_SHIFT        (0x06U)
#define PMIC_WD_STAT_REG_WD_BAD_EVENT_SHIFT             (0x04U)
#define PMIC_WD_STAT_REG_WD_ERR_CNT_SHIFT               (0x00U)

/* Watch Dog Error Status Register Shift Values */
#define PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_SHIFT           (0x07U)
#define PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_SHIFT           (0x06U)
#define PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_SHIFT       (0x05U)
#define PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_SHIFT          (0x04U)
#define PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_SHIFT           (0x03U)
#define PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_SHIFT        (0x02U)
#define PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_SHIFT        (0x01U)
#define PMIC_WD_ERR_STAT_REG_WD_TMO_SHIFT               (0x00U)

/**
 * @brief  PMIC Watch Dog Register Mask Values
 */
/* Watchdog Configuration Register Shift Values */
#define PMIC_WD_EN_MASK                                                        \
    (0x01U << PMIC_WD_EN_SHIFT)
#define PMIC_WD_PWRHOLD_MASK                                                   \
    (0x01U << PMIC_WD_PWRHOLD_SHIFT)
#define PMIC_WD_MODE_MASK                                                      \
    (0x01U << PMIC_WD_MODE_SHIFT)
#define PMIC_WD_RETURN_LONGWIN_MASK                                            \
    (0x01U << PMIC_WD_RETURN_LONGWIN_SHIFT)

/* Watchdog Long Window Time Configuration Mask Values */
#define PMIC_WD_LONGWIN_MASK                                                   \
    (0xFFU << PMIC_WD_LONGWIN_SHIFT)

/* Watch Dog Window 1 Configuration Mask Values */
#define PMIC_WD_WIN1_CFG_WD_WIN1_MASK                                          \
    (0x7FU << PMIC_WD_WIN1_CFG_WD_WIN1_SHIFT)

/* Watch Dog Window 2 Configuration Mask Values */
#define PMIC_WD_WIN2_CFG_WD_WIN2_MASK                                          \
    (0x7FU << PMIC_WD_WIN2_CFG_WD_WIN2_SHIFT)

/* Watch Dog Threshold Value Configuration Mask Values*/
#define PMIC_WD_THR_CFG_WD_TH1_MASK                                            \
    (0x07U << PMIC_WD_THR_CFG_WD_TH1_SHIFT)
#define PMIC_WD_THR_CFG_WD_TH2_MASK                                            \
    (0x07U << PMIC_WD_THR_CFG_WD_TH2_SHIFT)

/* Watch Dog QA Configuration Mask Values */
#define PMIC_WD_QA_CFG_WD_QUESTION_SEED_MASK                                   \
    (0x0FU << PMIC_WD_QA_CFG_WD_QUESTION_SEED_SHIFT)
#define PMIC_WD_QA_CFG_WD_QA_LFSR_MASK                                         \
    (0x03U << PMIC_WD_QA_CFG_WD_QA_LFSR_SHIFT)
#define PMIC_WD_QA_CFG_WD_QA_FDBK_MASK                                         \
    (0x03U << PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT)

/* Watch Dog Threshold Interrupt Configuration Mask Values */
#define PMIC_WD_THR2_INT_CFG_MASK                                              \
    (0x03U << PMIC_WD_THR2_INT_CFG_SHIFT)
#define PMIC_WD_THR2_INT_MASK_CFG_MASK                                         \
    (0x01U << PMIC_WD_THR2_INT_MASK_CFG_SHIFT)
#define PMIC_WD_THR1_INT_CFG_MASK                                              \
    (0x03U << PMIC_WD_THR1_INT_CFG_SHIFT)
#define PMIC_WD_THR1_INT_MASK_CFG_MASK                                         \
    (0x01U << PMIC_WD_THR1_INT_MASK_CFG_SHIFT)

/* Watch Dog Question Answer Count Register Mask Values */
#define PMIC_WD_QUESTION_ANSW_CNT_WD_ANSW_CNT_MASK                             \
    (0x03U << PMIC_WD_QUESTION_ANSW_CNT_WD_ANSW_CNT_SHIFT)
#define PMIC_WD_QUESTION_ANSW_CNT_WD_QUESTION_MASK                             \
    (0x0FU << PMIC_WD_QUESTION_ANSW_CNT_WD_QUESTION_SHIFT)

/* Watch Dog Answer byte Register Mask Values */
#define PMIC_WD_ANSWER_MASK                                                    \
    (0xFFU << PMIC_WD_ANSWER_SHIFT)

/* Watch Dog Status Register Mask Values */
#define PMIC_WD_STAT_REG_WD_FIRST_OK_MASK                                      \
    (0x01U << PMIC_WD_STAT_REG_WD_FIRST_OK_SHIFT)
#define PMIC_WD_STAT_REG_WD_LONGWIN_ACTIVE_MASK                                \
    (0x01U << PMIC_WD_STAT_REG_WD_LONGWIN_ACTIVE_SHIFT)
#define PMIC_WD_STAT_REG_WD_BAD_EVENT_MASK                                     \
    (0x01U << PMIC_WD_STAT_REG_WD_BAD_EVENT_SHIFT)
#define PMIC_WD_STAT_REG_WD_ERR_CNT_MASK                                       \
    (0x0FU << PMIC_WD_STAT_REG_WD_ERR_CNT_SHIFT)

/* Watch Dog Error Status Register Mask Values */
#define PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_MASK                                   \
    (0x01U << PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_SHIFT)

#define PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_MASK                                   \
    (0x01U << PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_SHIFT)

#define PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_MASK                               \
    (0x01U << PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_SHIFT)

#define PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_MASK                                  \
    (0x01U << PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_SHIFT)

#define PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_MASK                                   \
    (0x01U << PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_SHIFT)

#define PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_MASK                                \
    (0x01U << PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_SHIFT)

#define PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_MASK                                \
    (0x01U << PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_SHIFT)

#define PMIC_WD_ERR_STAT_REG_WD_TMO_MASK                                       \
    (0x01U << PMIC_WD_ERR_STAT_REG_WD_TMO_SHIFT)

/**
 * @brief  Watchdog Long Window Max, Min and Divisor macros
 */
/** @brief  Valid only for PG2.0 */
#define PMIC_WD_LONGWIN_80_MILLISEC                     (80U)
#define PMIC_WD_LONGWIN_125_MILLISEC                    (125U)
#define PMIC_WD_LONGWIN_8000_MILLISEC                   (8000U)

#define PMIC_WD_LONGWIN_MILLISEC_DIV_125                (125U)
#define PMIC_WD_LONGWIN_MILLISEC_DIV_4000               (4000U)

#define PMIC_WD_LONGWIN_MILLISEC_MIN_PG_2_0             (125U)
#define PMIC_WD_LONGWIN_MILLISEC_MAX_PG_2_0             (772000U)

#define PMIC_WD_LONGWIN_REG_VAL_0                       (0x0U)
#define PMIC_WD_LONGWIN_REG_VAL_1                       (0x1U)
#define PMIC_WD_LONGWIN_REG_VAL_64                      (0x40U)

/** @brief  Valid only for PG1.0 */
#define PMIC_WD_LONGWIN_100_MILLISEC                    (100U)
#define PMIC_WD_LONGWIN_MILLISEC_MIN                    (3000U)
#define PMIC_WD_LONGWIN_MILLISEC_MAX                    (765000U)
#define PMIC_WD_LONGWIN_MILLISEC_DIV                    (3000U)

/**
 * @brief  Watchdog Window1 Max, Min and Divisor macros
 */
#define PMIC_WD_WIN1_2_MICROSEC_MIN                     (550U)
#define PMIC_WD_WIN1_2_MICROSEC_MAX                     (70400U)
#define PMIC_WD_WIN1_2_MICROSEC_DIV                     (550U)

/* EXTRAS */
#define PMIC_WD_MODE_REG_REGADDR                        (0x406U)

#define PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_SHIFT        (0x00U)
#define PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT           (0x01U)
#define PMIC_WD_MODE_REG_WD_PWRHOLD_SHIFT               (0x02U)
#define PMIC_WD_MODE_REG_WD_CNT_SEL_SHIFT               (0x07U)
#define PMIC_WD_MODE_REG_WD_ENDRV_SEL_SHIFT             (0x06U)

#define PMIC_WD_THR_CFG_WD_RST_TH_SHIFT                 (0x00U)
#define PMIC_WD_THR_CFG_WD_FAIL_TH_SHIFT                (0x03U)
#define PMIC_WD_THR_CFG_WD_EN_SHIFT                     (0x06U)
#define PMIC_WD_THR_CFG_WD_RST_EN_SHIFT                 (0x07U)

#define PMIC_WD_THR_CFG_WD_RST_TH_MASK                                         \
  (0x7U << PMIC_WD_THR_CFG_WD_RST_TH_SHIFT)
#define PMIC_WD_THR_CFG_WD_FAIL_TH_MASK                                        \
  (0x7U << PMIC_WD_THR_CFG_WD_FAIL_TH_SHIFT)
#define PMIC_WD_THR_CFG_WD_EN_MASK                                             \
  (0x1U << PMIC_WD_THR_CFG_WD_EN_SHIFT)
#define PMIC_WD_THR_CFG_WD_RST_EN_MASK                                         \
  (0x1U << PMIC_WD_THR_CFG_WD_RST_EN_SHIFT)

#define PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_MASK                                \
  (0x1U << PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_SHIFT)
#define PMIC_WD_MODE_REG_WD_MODE_SELECT_MASK                                   \
  (0x1U << PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT)
#define PMIC_WD_MODE_REG_WD_PWRHOLD_MASK                                       \
  (0x1U << PMIC_WD_MODE_REG_WD_PWRHOLD_SHIFT)
#define PMIC_WD_MODE_REG_WD_CNT_SEL_MASK                                       \
  (0x1U << PMIC_WD_MODE_REG_WD_CNT_SEL_SHIFT)
#define PMIC_WD_MODE_REG_WD_ENDRV_SEL_MASK                                     \
  (0x1U << PMIC_WD_MODE_REG_WD_ENDRV_SEL_SHIFT)

#endif /* PMIC_INC_PMIC_WDG_PRIV_H_ */

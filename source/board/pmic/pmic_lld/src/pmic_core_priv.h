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
 *    its contributors may be used to endorse or promote products derived@
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
 *  @file pmic_core_priv.h
 *
 *  @brief This file contains PMIC Driver specific common API
 */

#ifndef PMIC_CORE_PRIV_H_
#define PMIC_CORE_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic.h"

#include "pmic_io_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @brief: PMIC Module Device Revision Infos
 */
#define PMIC_DEV_ID_REGADDR (0x00U)

/**
 * @brief  PMIC STAT_READBACK_ERR register Addresses
 */
#define PMIC_RDBK_ERR_STAT_REGADDR (0xAU)

/**
 * @brief   PMIC AMUX/DMUX Control Register Address
 */
#define PMIC_DIAG_OUT_CFG_CTRL_REGADDR (0x58U)
#define PMIC_DIAG_OUT_CFG_REGADDR (0x59U)

/**
 * @brief: PMIC STATE_STAT and RST_MCU_TMR Registers
 */
#define PMIC_STATE_CTRL_REGADDR (0x16U)
#define PMIC_STATE_STAT_REGADDR (0x17U)
#define PMIC_RST_MCU_TMR_REGADDR (0x18U)

/**
 * @brief  PMIC Dual Random Spread Spectrum register Addresses
 */
#define PMIC_BUCK_BST_CFG_REGADDR (0x1BU)

/**
 * @brief  PMIC Register, Timer and Counter Lock register Addresses
 */

#define PMIC_REGISTER_UNLOCK_REGADDR    (0x03U)
#define PMIC_TMR_COUNTER_UNLOCK_REGADDR (0x04U)
#define PMIC_REG_LOCK_STATUS_REGADDR    (0x09U)

/**
 * @brief  PMIC Scratchpad register Addresses
 */
#define PMIC_CUSTOMER_SCRATCH1_REGADDR (0x68U)
#define PMIC_CUSTOMER_SCRATCH2_REGADDR (0x69U)

/**
 * @brief  PMIC Watchdog Long Window Config register Addresses
 */
#define PMIC_WD_LONGWIN_CFG_REGADDR (0x3DU)

/**
 * @brief  PMIC SAFE_TMO_CFG register Addresses
 */
#define PMIC_SAFE_TMO_CFG_REGADDR (0x52U)

/**
 * @brief  PMIC SAFE_OUT register Addresses
 */
#define PMIC_SAFE_OUT_CFG_CTRL_REGADDR (0x54U)

/**
 * @brief  PMIC SAFE_OUT2_CFG1 register Addresses
 */
#define PMIC_SAFE_OUT2_CFG1_REGADDR (0x55U)

/**
 * @brief  PMIC SAFE_OUT2_CFG2 register Addresses
 */
#define PMIC_SAFE_OUT2_CFG2_REGADDR (0x56U)

/**
 * @brief  PMIC SAFE_OUT2_CFG3 register Addresses
 */
#define PMIC_SAFE_OUT2_CFG3_REGADDR (0x57U)

/**
 * @brief  PMIC invalid register address
 */
#define PMIC_INVALID_REGADDR (0xFFU)

/**
 * @brief  PMIC invalid BIT SHIFT value
 */
#define PMIC_INVALID_BIT_SHIFT (0xFFU)

#define PMIC_CUSTOMER_SCRATCH1_SHIFT (0X00U)
#define PMIC_CUSTOMER_SCRATCH2_SHIFT (0X00U)

/**
 * @brief: PMIC STATE_STAT Register Shift Values
 */
#define PMIC_STATE_SHIFT (0x00U)
#define PMIC_PWRD_DLY_ACTV_SHIFT (0x04U)
#define PMIC_RST_MCU_RQ_FLAG_SHIFT (0x05U)
#define PMIC_RST_MCU_CNT_SHIFT (0x06U)

/**
 * @brief: PMIC RST_MCU_TMR Register Shift Values
 */
#define PMIC_RST_MCU_TMR_SHIFT (0x00U)

/**
 * @brief   PMIC SAFE_OUT Register Shift Values
 */
#define PMIC_ENABLE_SAFE_OUTEN1_SHIFT (0x0U)
#define PMIC_ENABLE_SAFE_OUTEN2_SHIFT (0x1U)

/**
 * @brief   PMIC Dual Random Spread Spectrum Register Shift Values
 */
#define PMIC_DRSS_SS_EN_SHIFT (0x4U)

/**
 * @brief   PMIC Register Unlock Seq Register Shift Values
 */
#define PMIC_REG_UNLOCK_SEQ_SHIFT (0x0U)

/**
 * @brief   PMIC Register Unlock Status Register Shift Values
 */
#define PMIC_CFGREG_LOCKED_STATUS_SHIFT (0x0U)
#define PMIC_CNTREG_LOCKED_STATUS_SHIFT (0x1U)
#define PMIC_REGISTER_LOCK_STATUS_SHIFT (0x0U)

/**
 * @brief   PMIC BB DIAGANOSTIC OUT Register Shift Values
 */
#define PMIC_DIAG_GRP_SEL_SHIFT (0x0U)
#define PMIC_DIAG_OUT_CTRL_SHIFT (0x6U)
#define PMIC_DIAG_CH_SEL_SHIFT (0x0U)

/**
 * @brief  BB_PMIC STAT_READBACK_ERR register Shift Values
 */
#define PMIC_NRST_RDBK_LVL_SHIFT (0x0U)
#define PMIC_SAFE_OUT1_RDBK_LVL_SHIFT (0x1U)
#define PMIC_EN_OUT_RDBK_LVL_SHIFT (0x2U)

/**
 * @brief   PMIC DEV_REV Register Shift Values
 */
#define PMIC_DEV_ID_SHIFT (0x0U)

#define PMIC_STATE_CTRL_STATE_REQ_SHIFT 0
#define PMIC_STATE_CTRL_STATE_REQ_MASK\
    ((uint8_t)(0x03U << PMIC_STATE_CTRL_STATE_REQ_SHIFT))

/**
 * @brief: PMIC State Stat Register  Mask Values
 */
#define PMIC_STATE_MASK\
    ((uint8_t)(0x0FU << PMIC_STATE_SHIFT))
#define PMIC_PWRD_DLY_ACTV_MASK\
    ((uint8_t)(0x01U << PMIC_PWRD_DLY_ACTV_SHIFT))
#define PMIC_RST_MCU_RQ_FLAG_MASK\
    ((uint8_t)(0x01U << PMIC_RST_MCU_RQ_FLAG_SHIFT))
#define PMIC_RST_MCU_CNT_MASK\
    ((uint8_t)(0x03U << PMIC_RST_MCU_CNT_SHIFT))

/**
 * @brief: PMIC Reset mcu Timer Register  Mask Values
 */
#define PMIC_RST_MCU_TMR_MASK ((uint8_t)(0x7FU << PMIC_RST_MCU_TMR_SHIFT))

/**
 * @brief   PMIC Dual Random Spread Spectrum Register Mask Values
 */
#define PMIC_DRSS_SS_EN_MASK ((uint8_t)(0x01U << PMIC_DRSS_SS_EN_SHIFT))

/**
 * @brief   PMIC Safe Out Enable Register Mask Values
 */
#define PMIC_ENABLE_SAFE_OUTEN1_MASK\
    ((uint8_t)(0x01U << PMIC_ENABLE_SAFE_OUTEN1_SHIFT))
#define PMIC_ENABLE_SAFE_OUTEN2_MASK\
    ((uint8_t)(0x01U << PMIC_ENABLE_SAFE_OUTEN2_SHIFT))

#define PMIC_CUSTOMER_SCRATCH1_MASK\
    ((uint8_t)(0xFFU << PMIC_CUSTOMER_SCRATCH1_SHIFT))
#define PMIC_CUSTOMER_SCRATCH2_MASK\
    ((uint8_t)(0xFFU << PMIC_CUSTOMER_SCRATCH2_SHIFT))

/**
 * @brief   PMIC Register Lock Register Mask Values
 */
/*  PMIC Register Lock Register Mask Values to read the register lock status */
#define PMIC_LOCK_REG_CFG_STATUS_MASK\
    ((uint8_t)(0x01U << PMIC_REG_UNLOCK_SEQ_SHIFT))

/*  PMIC Register Lock Register Mask Values to read the register lock status */
#define PMIC_REG_LOCK_STATUS_READ_MASK\
    ((uint8_t)(0x03U << PMIC_REGISTER_LOCK_STATUS_SHIFT))

#define PMIC_CFGREG_LOCK_STATUS_RD_MASK\
    ((uint8_t)(0x01U << PMIC_CFGREG_LOCKED_STATUS_SHIFT))

#define PMIC_CNTREG_LOCK_STATUS_RD_MASK\
    ((uint8_t)(0x01U << PMIC_CNTREG_LOCKED_STATUS_SHIFT))

/**  PMIC Register Lock Register Mask Values to write lock/unlock value to
 *   register lock register */
#define PMIC_REG_LOCK_STATUS_WR_MASK\
    ((uint8_t)(0xFFU << PMIC_REG_UNLOCK_SEQ_SHIFT))

/**
 * @brief   PMIC BB DIAGANOSTIC OUT Register Mask Values
 */
#define PMIC_DIAG_GRP_SEL_MASK ((uint8_t)(0x1FU << PMIC_DIAG_GRP_SEL_SHIFT))
#define PMIC_DIAG_OUT_CTRL_MASK ((uint8_t)(0x03U << PMIC_DIAG_OUT_CTRL_SHIFT))
#define PMIC_DIAG_CH_SEL_MASK ((uint8_t)(0x1FU << PMIC_DIAG_CH_SEL_SHIFT))

/**
 * @brief  BB_PMIC STAT_READBACK_ERR register Mask Values
 */
#define PMIC_NRST_RDBK_LVL_MASK\
    ((uint8_t)(0x01U << PMIC_NRST_RDBK_LVL_SHIFT))
#define PMIC_SAFE_OUT1_RDBK_LVL_MASK\
    ((uint8_t)(0x01U << PMIC_SAFE_OUT1_RDBK_LVL_SHIFT))
#define PMIC_EN_OUT_RDBK_LVL_MASK\
    ((uint8_t)(0x01U << PMIC_EN_OUT_RDBK_LVL_SHIFT))

/**
 * @brief   PMIC DEV_REV Register Mask Values
 */
#define PMIC_DEV_ID_MASK ((uint8_t)(0x7FU << PMIC_DEV_ID_SHIFT))

/**
 * @brief   PMIC REG_STAT Mask Values
 */
#define PMIC_CFG_REG_LOCK_MASK (0x01) /* Bit mask for CFG_REG_LOCK */
#define PMIC_CNT_REG_LOCK_MASK (0x02) /* Bit mask for CNT_REG_LOCK */

/**
 * @brief  PMIC CONFIG_1 register Shift Values
 */
#define PMIC_CFG1_TWARN_LEVEL_SHIFT (0U)
#define PMIC_CFG1_TSD_ORD_LEVEL_SHIFT (1U)
#define PMIC_CFG1_I2C1_HS_SHIFT (3U)
#define PMIC_CFG1_I2C2_HS_SHIFT (4U)
#define PMIC_CFG1_EN_ILM_FSM_CTRL_SHIFT (5U)
#define PMIC_CFG1_NSLEEP1_MASK_SHIFT (6U)
#define PMIC_CFG1_NSLEEP2_MASK_SHIFT (7U)

/**
 * @brief  PMIC CONFIG_1 register bit masks
 */
#define PMIC_CFG1_TWARN_LEVEL_MASK\
    ((uint8_t)(0x01U << PMIC_CFG1_TWARN_LEVEL_SHIFT))
#define PMIC_CFG1_TSD_ORD_LEVEL_MASK\
    ((uint8_t)(0x01U << PMIC_CFG1_TSD_ORD_LEVEL_SHIFT))
#define PMIC_CFG1_I2C1_HS_MASK\
    ((uint8_t)(0x01U << PMIC_CFG1_I2C1_HS_SHIFT))
#define PMIC_CFG1_I2C2_HS_MASK\
    ((uint8_t)(0x01U << PMIC_CFG1_I2C2_HS_SHIFT))
#define PMIC_CFG1_EN_ILIM_FSM_CTRL_MASK\
    ((uint8_t)(0x01U << PMIC_CFG1_EN_ILM_FSM_CTRL_SHIFT))
#define PMIC_CFG1_NSLEEP1_MASK_MASK\
    ((uint8_t)(0x01U << PMIC_CFG1_NSLEEP1_MASK_SHIFT))
#define PMIC_CFG1_NSLEEP2_MASK_MASK\
    ((uint8_t)(0x01U << PMIC_CFG1_NSLEEP2_MASK_SHIFT))

/**
 * @brief   PMIC SAFE STATE TIMEOUT CONFIG SHIFT AND MASK VALUES
 */
/* PMIC SAFE STATE SHIFT VALUES */
#define PMIC_SAFE_TMO_SHIFT (0x5U)
#define PMIC_SAFE_LOCK_TH_SHIFT (0x0U)
/* PMIC SAFE STATE MASK VALUES */
#define PMIC_SAFE_TMO_MASK ((uint8_t)(0x07U << PMIC_SAFE_TMO_SHIFT))
#define PMIC_SAFE_LOCK_TH_MASK ((uint8_t)(0x1FU << PMIC_SAFE_LOCK_TH_SHIFT))

/**
 * @brief   PMIC SAFE STATE OUT CONFIG CONTROL SHIFT AND MASK VALUES
 */
/* PMIC SAFE STATE SHIFT VALUES */
#define PMIC_SAFE_OUT2_CFG_SHIFT (0x2U)
#define PMIC_SAFE_OUT2_EN_SHIFT (0x1U)
#define PMIC_SAFE_OUT1_EN_SHIFT (0x0U)
/* PMIC SAFE STATE MASK VALUES */
#define PMIC_SAFE_OUT2_CFG_MASK ((uint8_t)(0x07U << PMIC_SAFE_OUT2_CFG_SHIFT))
#define PMIC_SAFE_OUT2_EN_MASK ((uint8_t)(0x01U << PMIC_SAFE_OUT2_EN_SHIFT))
#define PMIC_SAFE_OUT1_EN_MASK ((uint8_t)(0x01U << PMIC_SAFE_OUT1_EN_SHIFT))

/**
 * @brief   PMIC SAFE STATE OUT2 CONFIG-1 SHIFT AND MASK VALUES
 */
/* PMIC SAFE STATE SHIFT VALUES */
#define PMIC_SAFE_OUT2_PS2_SHIFT (0x3U)
#define PMIC_SAFE_OUT2_PS1_SHIFT (0x0U)
/* PMIC SAFE STATE MASK VALUES */
#define PMIC_SAFE_OUT2_PS2_MASK ((uint8_t)(0x03U << PMIC_SAFE_OUT2_PS2_SHIFT))
#define PMIC_SAFE_OUT2_PS1_MASK ((uint8_t)(0x03U << PMIC_SAFE_OUT2_PS1_SHIFT))

/**
 * @brief   PMIC SAFE STATE OUT2 CONFIG-2 SHIFT AND MASK VALUES
 */
/* PMIC SAFE STATE SHIFT VALUES */
#define PMIC_SAFE_OUT2_PWMH_NSC_SHIFT (0x4U)
#define PMIC_SAFE_OUT2_PWML_NSC_SHIFT (0x0U)
/* PMIC SAFE STATE MASK VALUES */
#define PMIC_SAFE_OUT2_PWMH_NSC_MASK\
    ((uint8_t)(0x0FU << PMIC_SAFE_OUT2_PWMH_NSC_SHIFT))
#define PMIC_SAFE_OUT2_PWML_NSC_MASK\
    ((uint8_t)(0x0FU << PMIC_SAFE_OUT2_PWML_NSC_SHIFT))

/**
 * @brief   PMIC SAFE STATE OUT2 CONFIG-3 SHIFT AND MASK VALUES
 */
/* PMIC SAFE STATE SHIFT VALUES */
#define PMIC_SAFEOUT2_PWMH_DLY_SC_SHIFT (0x4U)
#define PMIC_SAFEOUT2_PWML_PLS_SC_SHIFT (0x0U)
/* PMIC SAFE STATE MASK VALUES */
#define PMIC_SAFEOUT2_PWMH_DLY_SC_MASK\
    ((uint8_t)(0x0FU << PMIC_SAFEOUT2_PWMH_DLY_SC_SHIFT))
#define PMIC_SAFEOUT2_PWML_PLS_SC_MASK\
    ((uint8_t)(0x0FU << PMIC_SAFEOUT2_PWML_PLS_SC_SHIFT))

/**
 * @brief  PMIC power Configuration Register Address
 */
#define PMIC_CFG1_REGADDR (0x7DU)

/**
 * @brief  PMIC CONFIG_2 register Addresses
 */
#define PMIC_CONFIG_2_REGADDR (0x7EU)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 *  @brief   This function is used to write a specific bit field value
 */
static inline void Pmic_setBitField(uint8_t * pRegVal, uint8_t regFieldShift,
    uint8_t regFieldMask, uint8_t fieldVal) {
    // Calculate the mask for the field location
    uint8_t mask = (uint8_t)(regFieldMask << regFieldShift);
    // Shift the field value into the correct position in the register
    uint8_t value = (uint8_t)(fieldVal << regFieldShift) & mask;
    // Clear the bits of the field in *pRegVal and set them to the new value
    *pRegVal = (*pRegVal & ~mask) | value;
}

/**
 * @brief   This function is used to read a specific bit field value
 */
static inline uint8_t Pmic_getBitField(uint8_t regData, uint8_t regFieldShift,
    uint8_t regFieldMask) {
    uint8_t fieldVal;

    fieldVal = (regData >> regFieldShift) & regFieldMask;
    return fieldVal;
}

bool pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos);

void Pmic_criticalSectionStart(const Pmic_CoreHandle_t * pPmicCoreHandle);

void Pmic_criticalSectionStop(const Pmic_CoreHandle_t * pPmicCoreHandle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_CORE_PRIV_H_ */

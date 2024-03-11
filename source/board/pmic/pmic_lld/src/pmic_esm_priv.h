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
 * @file   pmic_esm_priv.h
 *
 * @brief: This file contains macro definitions, structures and function
 *         prototypes for driver specific PMIC ESM configuration
 */

#ifndef PMIC_ESM_PRIV_H_
#define PMIC_ESM_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Pmic_ESM PMIC Error State Machine
 * @{
 * @brief Contains definitions related to PMIC ESM functionality.
 */

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @defgroup Pmic_ESMPrivMacros PMIC Error State Machine Private Macros
 * @{
 * @ingroup Pmic_ESM
 * @brief Contains private macros used in the ESM module of PMIC driver.
 */

/**
 * @brief Offset value for the ESM configuration register 1.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_PMIC_ESM_CFG1_REG_OFFSET   (0x01U)

/**
 * @brief Offset value for the ESM configuration register 2.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_PMIC_ESM_CFG2_REG_OFFSET   (0x02U)

/**
 * @brief Offset value for the ESM interrupt configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_PMIC_ESM_INT_CFG_REG_OFFSET    (0x03U)

/**
 * @brief Offset value for the ESM delay 1 register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_PMIC_ESM_DELAY1_REG_OFFSET     (0x04U)

/**
 * @brief Offset value for the ESM delay 2 register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_PMIC_ESM_DELAY2_REG_OFFSET     (0x05U)

/**
 * @brief Offset value for the ESM HMAX configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_HMAX_REG_OFFSET    (0x06U)

/**
 * @brief Offset value for the ESM HMIN configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_HMIN_REG_OFFSET    (0x07U)

/**
 * @brief Offset value for the ESM LMAX configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_LMAX_REG_OFFSET    (0x08U)

/**
 * @brief Offset value for the ESM LMIN configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_LMIN_REG_OFFSET    (0x09U)

/**
 * @brief Offset value for the ESM error status register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_PMIC_ESM_ERR_STAT_REG_OFFSET   (0x10U)

/**
 * @brief Address of the ESM control register used to enable ESM.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CTRL_REG_ADDR      (0x47U)

/**
 * @brief Bit shift position for enabling ESM in the control register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CTRL_REG_SHIFT     (0x00U)

/**
 * @brief Bit mask for enabling ESM in the control register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CTRL_REG_MASK      (0x01U)

/**
 * @brief Address of the ESM configuration register 1.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG1_REG           (0x48U)

/**
 * @brief Bit shift position for error threshold configuration in ESM
 * configuration register 1.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG1_ERR_TH_SHIFT  (0x00U)

/**
 * @brief Bit shift position for enabling ESM in ESM configuration register 1.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG1_ESM_EN_SHIFT  (0x06U)

/**
 * @brief Bit shift position for configuring ESM mode in ESM configuration
 * register 1.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG1_ESM_CFG_SHIFT (0x07U)

/**
 * @brief Bit mask for error threshold configuration in ESM configuration
 * register 1.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG1_ERR_TH_MASK   (0x07U << PMIC_ESM_CFG1_ERR_TH_SHIFT)

/**
 * @brief Bit mask for enabling ESM in ESM configuration register 1.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG1_ESM_EN_MASK   (0x01U << PMIC_ESM_CFG1_ESM_EN_SHIFT)

/**
 * @brief Bit mask for configuring ESM mode in ESM configuration register 1.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG1_ESM_CFG_MASK  (0x01U << PMIC_ESM_CFG1_ESM_CFG_SHIFT)

/**
 * @brief Address of the ESM configuration register 2.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG2_REG           (0x49U)

/**
 * @brief Bit shift position for time configuration in ESM configuration
 * register 2.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG2_TIME_CFG_SHIFT (0x00U)

/**
 * @brief Bit shift position for configuring ESM deglitch in ESM configuration
 * register 2.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG2_ESM_DGL_SHIFT  (0x03U)

/**
 * @brief Bit shift position for configuring ESM level polarity in ESM
 * configuration register 2.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG2_ESM_LVL_POL_SHIFT (0x04U)

/**
 * @brief Bit mask for time configuration in ESM configuration register 2.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG2_TIME_CFG_MASK (0x03U << PMIC_ESM_CFG2_TIME_CFG_SHIFT)

/**
 * @brief Bit mask for configuring ESM deglitch in ESM configuration register 2.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG2_ESM_DGL_MASK (0x01U << PMIC_ESM_CFG2_ESM_DGL_SHIFT)

/**
 * @brief Bit mask for configuring ESM level polarity in ESM configuration
 * register 2.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_CFG2_ESM_LVL_POL_MASK\
    (0x01U << PMIC_ESM_CFG2_ESM_LVL_POL_SHIFT)

/**
 * @brief Address of the ESM interrupt configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_INT_CFG_REG    (0x4AU)

/**
 * @brief Bit shift position for ESM interrupt mask in ESM interrupt
 * configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_INT_MASK_SHIFT (0X01U)

/**
 * @brief Bit shift position for ESM delay 1 interrupt mask in ESM interrupt
 * configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_DLY1_INT_MASK_SHIFT (0X02U)

/**
 * @brief Bit shift position for ESM delay 1 interrupt configuration in ESM
 * interrupt configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_DLY1_INT_CFG_SHIFT (0X03U)

/**
 * @brief Bit shift position for ESM delay 2 interrupt mask in ESM interrupt
 * configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_DLY2_INT_MASK_SHIFT (0X04U)

/**
 * @brief Bit shift position for ESM delay 2 interrupt configuration in ESM
 * interrupt configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_DLY2_INT_CFG_SHIFT (0X05U)

/**
 * @brief Bit mask for ESM interrupt mask in ESM interrupt configuration
 * register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_INT_MASK_MASK (0X01U << PMIC_ESM_INT_MASK_SHIFT)

/**
 * @brief Bit mask for ESM delay 1 interrupt mask in ESM interrupt configuration
 * register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_DLY1_INT_MASK_MASK (0X01U << PMIC_ESM_DLY1_INT_MASK_SHIFT)

/**
 * @brief Bit mask for ESM delay 1 interrupt configuration in ESM interrupt
 * configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_DLY1_INT_CFG_MASK (0X03U << PMIC_ESM_DLY1_INT_CFG_SHIFT)

/**
 * @brief Bit mask for ESM delay 2 interrupt mask in ESM interrupt configuration
 * register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_DLY2_INT_MASK_MASK (0X01U << PMIC_ESM_DLY2_INT_MASK_SHIFT)

/**
 * @brief Bit mask for ESM delay 2 interrupt configuration in ESM interrupt
 * configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_DLY2_INT_CFG_MASK (0X03U << PMIC_ESM_DLY2_INT_CFG_SHIFT)

/**
 * @brief Address of the ESM delay 1 register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_DELAY1_REG     (0x4BU)

/**
 * @brief Address of the ESM delay 2 register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_DELAY2_REG     (0x4CU)

/**
 * @brief Address of the ESM HMAX configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_HMAX_CFG_REG   (0x4DU)

/**
 * @brief Address of the ESM HMIN configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_HMIN_CFG_REG   (0x4EU)

/**
 * @brief Address of the ESM LMAX configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_LMAX_CFG_REG   (0x4FU)

/**
 * @brief Address of the ESM LMIN configuration register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_LMIN_CFG_REG   (0x50U)

/**
 * @brief Address of the ESM error status register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_ERR_STAT_REG   (0x51U)

/**
 * @brief Bit shift position for ESM error count in the error status register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_ERR_STAT_ESM_ERR_CNT_SHIFT (0X00U)

/**
 * @brief Bit shift position for ESM error in the error status register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_ERR_STAT_ESM_ERR_SHIFT (0X05U)

/**
 * @brief Bit shift position for ESM delay 1 error in the error status register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_ERR_STAT_ESM_DLY1_ERR_SHIFT (0X06U)

/**
 * @brief Bit shift position for ESM delay 2 error in the error status register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_ERR_STAT_ESM_DLY2_ERR_SHIFT (0X07U)

/**
 * @brief Bit mask for ESM error count in the error status register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_ERR_STAT_ESM_ERR_CNT_MASK\
    (0X07U << PMIC_ESM_ERR_STAT_ESM_ERR_CNT_SHIFT)

/**
 * @brief Bit mask for ESM error in the error status register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_ERR_STAT_ESM_ERR_MASK\
    (0X01U << PMIC_ESM_ERR_STAT_ESM_ERR_SHIFT)

/**
 * @brief Bit mask for ESM delay 1 error in the error status register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_ERR_STAT_ESM_DLY1_ERR_MASK\
    (0X01U << PMIC_ESM_ERR_STAT_ESM_DLY1_ERR_SHIFT)

/**
 * @brief Bit mask for ESM delay 2 error in the error status register.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_ERR_STAT_ESM_DLY2_ERR_MASK\
    (0X01U << PMIC_ESM_ERR_STAT_ESM_DLY2_ERR_SHIFT)

/**
 * @brief Maximum value for error count threshold in ESM configuration.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_ERR_CNT_THR_MAX (15U)

/**
 * @brief Maximum value for delay in microseconds for ESM configuration.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_DELAY_MICROSEC_MAX (522240U)

/**
 * @brief Divider value for converting delay from microseconds to internal
 * units.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_DELAY_MICROSEC_DIV (2048U)

/**
 * @brief Minimum value for PWM pulse duration in microseconds for ESM
 * configuration.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_PWM_PULSE_MICROSEC_MIN (15U)

/**
 * @brief Maximum value for PWM pulse duration in microseconds for ESM
 * configuration.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_PWM_PULSE_MICROSEC_MAX (3840U)

/**
 * @brief Divider value for converting PWM pulse duration from microseconds to
 * internal units.
 *
 * @ingroup Pmic_ESMPrivMacros
 */
#define PMIC_ESM_PWM_PULSE_MICROSEC_DIV (15U)

/**
 * @}
 */
/* End of Pmic_ESMPrivMacros */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @}
 */
/* End of Pmic_ESM */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_ESM_PRIV_H_ */

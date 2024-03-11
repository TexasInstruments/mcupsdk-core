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
 *   @file    pmic_low_iq_timer_priv.h
 *
 *   @brief   This file contains the private MACRO's and function definitions
 * for PMIC LOW IQ Timer state configuration
 *
 */

#ifndef PMIC_INC_LOW_IQ_TIMER_PRIV_H_
#define PMIC_INC_LOW_IQ_TIMER_PRIV_H_

/* ==========================================================================*/
/*                             Include Files                                 */
/* ==========================================================================*/

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @brief  PMIC Timer register Addresses
 */
#define PMIC_TMR_CFG_REG_REGADDR                (0x6AU)
#define PMIC_TMR_LP_WAKE0_REGADDR               (0x6BU)
#define PMIC_TMR_LP_WAKE1_REGADDR               (0x6CU)
#define PMIC_TMR_LP_WAKE2_REGADDR               (0x6DU)
#define PMIC_TMR_CNT0_REGADDR                   (0x6EU)
#define PMIC_TMR_CNT1_REGADDR                   (0x6FU)
#define PMIC_TMR_CNT2_REGADDR                   (0x70U)

/**
 * @brief  PMIC Timer Config register Shift Values
 */
#define PMIC_TMR_CFG_REG_TMR_CFG_SHIFT          (0x00U)
#define PMIC_TMR_CFG_REG_TMR_PS_SHIFT           (0x03U)
#define PMIC_TMR_CFG_REG_TMR_CLR_SHIFT          (0x05U)

/**
 * @brief  PMIC Timer WakeUp0 register Shift Values
 */
#define PMIC_TMR_LP_WAKE_B0_SHIFT               (0x00U)

/**
 * @brief  PMIC Timer WakeUp1 register Shift Values
 */
#define PMIC_TMR_LP_WAKE_B1_SHIFT               (0x00U)

/**
 * @brief  PMIC Timer WakeUp2 register Shift Values
 */
#define PMIC_TMR_LP_WAKE_B2_SHIFT               (0x00U)

/**
 * @brief  PMIC Timer Counter0 register Shift Values
 */
#define PMIC_TMR_CNT0_TMR_CNT_B0_SHIFT          (0x00U)

/**
 * @brief  PMIC Timer Counter1 register Shift Values
 */
#define PMIC_TMR_CNT1_TMR_CNT_B1_SHIFT          (0x00U)

/**
 * @brief  PMIC Timer Counter2 register Shift Values
 */
#define PMIC_TMR_CNT2_TMR_CNT_B2_SHIFT          (0x00U)

/**
 * @brief  PMIC Timer Config register Mask Values
 */
#define PMIC_TMR_CFG_REG_TMR_CFG_MASK\
    ((uint8_t)(0x07U << PMIC_TMR_CFG_REG_TMR_CFG_SHIFT))
#define PMIC_TMR_CFG_REG_TMR_PS_MASK\
    ((uint8_t)(0x03U << PMIC_TMR_CFG_REG_TMR_PS_SHIFT))
#define PMIC_TMR_CFG_REG_TMR_CLR_MASK\
    ((uint8_t)(0x01U << PMIC_TMR_CFG_REG_TMR_CLR_SHIFT))

/**
 * @brief  PMIC Timer Wakeup0 register Mask Values
 */
#define PMIC_TMR_LP_WAKE_B0_MASK ((uint8_t)(0xFFU << PMIC_TMR_LP_WAKE_B0_SHIFT))

/**
 * @brief  PMIC Timer Wakeup1 register Mask Values
 */
#define PMIC_TMR_LP_WAKE_B1_MASK ((uint8_t)(0xFFU << PMIC_TMR_LP_WAKE_B1_SHIFT))

/**
 * @brief  PMIC Timer Wakeup2 register Mask Values
 */
#define PMIC_TMR_LP_WAKE_B2_MASK ((uint8_t)(0xFFU << PMIC_TMR_LP_WAKE_B2_SHIFT))

/**
 * @brief  PMIC Timer Counter0 register Mask Values
 */
#define PMIC_TMR_CNT0_TMR_CNT_B0_MASK\
    ((uint8_t)(0xFFU << PMIC_TMR_CNT0_TMR_CNT_B0_SHIFT))

/**
 * @brief  PMIC Timer Counter1 register Mask Values
 */
#define PMIC_TMR_CNT1_TMR_CNT_B1_MASK\
    ((uint8_t)(0xFFU << PMIC_TMR_CNT1_TMR_CNT_B1_SHIFT))

/**
 * @brief  PMIC Timer Counter2 register Mask Values
 */
#define PMIC_TMR_CNT2_TMR_CNT_B2_MASK\
    ((uint8_t)(0xFFU << PMIC_TMR_CNT2_TMR_CNT_B2_SHIFT))

/* Timer Configuration Data*/
#define PMIC_TMR_CFG_DATA0\
    (0x00U) /* Timer stopped */
#define PMIC_TMR_CFG_DATA1\
    (0x01U) /* Timer counts in operating and sequencing states */
#define PMIC_TMR_CFG_DATA2\
    (0x02U) /* Timer counts only in STANDBY state */
#define PMIC_TMR_CFG_DATA3\
    (0x03U)
/* Timer counts only in STANDBY state and has wake up from STANDBY   \
            state    */
#define PMIC_TMR_CFG_DATA4\
    (\
        0x04U)
/* Timer counts in operating, sequencing states and STANDBY state \
 */
#define PMIC_TMR_CFG_DATA5\
    (0x05U)
/* Timer counts in operating, sequencing states and STANDBY state    \
            and has wake up from STANDBY state */

/* Timer Prescale selection for the counter time base */
#define PMIC_TMR_PS_DATA0                       (0x00U) /* 64.64 µs (typical)   */
#define PMIC_TMR_PS_DATA1                       (0x01U) /* 16.384 ms (typical)  */
#define PMIC_TMR_PS_DATA2                       (0x02U) /* 131.072 ms (typical) */
#define PMIC_TMR_PS_DATA3                       (0x03U) /* 1049 ms (typical)    */

/* Clears (resets) low power timer to 0 (LP_TIMER2, LP_TIMER1, LP_TIMER0) */
#define PMIC_TMR_CLR_DATA                       (0x01U)

#define PMIC_TMR_CFG1                           (0X00U) /* Stopped */
#define PMIC_TMR_CFG2                           (0X01U) /* Operating States Timer */
#define PMIC_TMR_CFG3                           (0X02U) /* STANDBY State Timer */
#define PMIC_TMR_CFG4                           (0X03U) /* STANDBY State Timer */
#define PMIC_TMR_CFG5                           (0X04U) /* STANDBY State and Operating States Timer */
#define PMIC_TMR_CFG6\
    (0X05U) /* STANDBY State with wakeup and Operating States Timer */

#endif /* PMIC_INC_LOW_IQ_TIMER_PRIV_H_ */
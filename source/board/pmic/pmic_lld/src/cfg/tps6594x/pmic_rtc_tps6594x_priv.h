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
 * \file   pmic_rtc_tps6594x_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         configuring PMIC RTC
 */

#ifndef PMIC_RTC_TPS6594X_PRIV_H_
#define PMIC_RTC_TPS6594X_PRIV_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/*!
 * \brief   PMIC RTC time and date Register Address
 */
#define PMIC_RTC_SECONDS_REGADDR                       (0xB5U)
#define PMIC_RTC_MINUTES_REGADDR                       (0xB6U)
#define PMIC_RTC_HOURS_REGADDR                         (0xB7U)
#define PMIC_RTC_DAYS_REGADDR                          (0xB8U)
#define PMIC_RTC_MONTHS_REGADDR                        (0xB9U)
#define PMIC_RTC_YEARS_REGADDR                         (0xBAU)
#define PMIC_RTC_WEEKS_REGADDR                         (0xBBU)

/*!
 * \brief  PMIC Alarm time and date Register Address
 */
#define PMIC_ALARM_SECONDS_REGADDR                     (0xBCU)
#define PMIC_ALARM_MINUTES_REGADDR                     (0xBDU)
#define PMIC_ALARM_HOURS_REGADDR                       (0xBEU)
#define PMIC_ALARM_DAYS_REGADDR                        (0xBFU)
#define PMIC_ALARM_MONTHS_REGADDR                      (0xC0U)
#define PMIC_ALARM_YEARS_REGADDR                       (0xC1U)

/*!
 * \brief  PMIC RTC Control, Status and Interrupt Register Address
 */
#define PMIC_RTC_CTRL_1_REGADDR                        (0xC2U)
#define PMIC_RTC_CTRL_2_REGADDR                        (0xC3U)
#define PMIC_RTC_STATUS_REGADDR                        (0xC4U)
#define PMIC_RTC_INTERRUPTS_REGADDR                    (0xC5U)

/*!
 * \brief  PMIC RTC frequency compensation register Address
 */
#define PMIC_RTC_COMP_LSB_REGADDR                     (0xC6U)
#define PMIC_RTC_COMP_MSB_REGADDR                     (0xC7U)

/*!
 * \brief  PMIC RTC reset status register Address
 */
#define PMIC_RTC_RESET_STATUS_REGADDR                 (0xC8U)

/*!
 * \brief  RTC and alarm time bit fields
 */
#define PMIC_ALARM_SECONDS_ALR_SECOND_1_SHIFT          (0x04U)
#define PMIC_ALARM_SECONDS_ALR_SECOND_0_SHIFT          (0x00U)
#define PMIC_ALARM_MINUTES_ALR_MINUTE_1_SHIFT          (0x04U)
#define PMIC_ALARM_MINUTES_ALR_MINUTE_0_SHIFT          (0x00U)
#define PMIC_ALARM_HOURS_ALR_PM_NAM_SHIFT              (0x07U)
#define PMIC_ALARM_HOURS_ALR_HOUR_1_SHIFT              (0x04U)
#define PMIC_ALARM_HOURS_ALR_HOUR_0_SHIFT              (0x00U)
#define PMIC_ALARM_DAYS_ALR_DAY_1_SHIFT                (0x04U)
#define PMIC_ALARM_DAYS_ALR_DAY_0_SHIFT                (0x00U)
#define PMIC_ALARM_MONTHS_ALR_MONTH_1_SHIFT            (0x04U)
#define PMIC_ALARM_MONTHS_ALR_MONTH_0_SHIFT            (0x00U)
#define PMIC_ALARM_YEARS_ALR_YEAR_1_SHIFT              (0x04U)
#define PMIC_ALARM_YEARS_ALR_YEAR_0_SHIFT              (0x00U)

/*!
 * \brief  PMIC RTC time and date bit fields
 */
#define PMIC_RTC_SECONDS_SECOND_1_SHIFT                (0x04U)
#define PMIC_RTC_SECONDS_SECOND_0_SHIFT                (0x00U)
#define PMIC_RTC_MINUTES_MINUTE_1_SHIFT                (0x04U)
#define PMIC_RTC_MINUTES_MINUTE_0_SHIFT                (0x00U)
#define PMIC_RTC_HOURS_PM_NAM_SHIFT                    (0x07U)
#define PMIC_RTC_HOURS_HOUR_1_SHIFT                    (0x04U)
#define PMIC_RTC_HOURS_HOUR_0_SHIFT                    (0x00U)
#define PMIC_RTC_DAYS_DAY_1_SHIFT                      (0x04U)
#define PMIC_RTC_DAYS_DAY_0_SHIFT                      (0x00U)
#define PMIC_RTC_MONTHS_MONTH_1_SHIFT                  (0x04U)
#define PMIC_RTC_MONTHS_MONTH_0_SHIFT                  (0x00U)
#define PMIC_RTC_YEARS_YEAR_1_SHIFT                    (0x04U)
#define PMIC_RTC_YEARS_YEAR_0_SHIFT                    (0x00U)
#define PMIC_RTC_WEEKS_WEEK_SHIFT                      (0x00U)

/*!
 * \brief  PMIC RTC status bit fields
 */
#define PMIC_RTC_STATUS_ALARM_SHIFT                    (0x06U)
#define PMIC_RTC_STATUS_TIMER_SHIFT                    (0x05U)
#define PMIC_RTC_STATUS_RUN_SHIFT                      (0x01U)
#define PMIC_RTC_STATUS_POWER_UP_SHIFT                 (0x07U)

#define PMIC_RTC_INTERRUPTS_IT_ALARM_SHIFT             (0x03U)
#define PMIC_RTC_INTERRUPTS_IT_TIMER_SHIFT             (0x02U)

/*!
 * \brief  RTC Timer periods interrupt bit fields
 */
#define PMIC_RTC_INTERRUPTS_EVERY_SHIFT               (0x00U)

/*!
 * \brief  PMIC RTC CTRL bit fields
 */
#define PMIC_RTC_CTRL_1_RTC_V_OPT_SHIFT               (0x07U)
#define PMIC_RTC_CTRL_1_GET_TIME_SHIFT                (0x06U)
#define PMIC_RTC_CTRL_1_SET_32_COUNTER_SHIFT          (0x05U)
#define PMIC_RTC_CTRL_1_MODE_12_24_SHIFT              (0x03U)
#define PMIC_RTC_CTRL_1_AUTO_COMP_SHIFT               (0x02U)
#define PMIC_RTC_CTRL_1_ROUND_30S_SHIFT               (0x01U)
#define PMIC_RTC_CTRL_1_STOP_RTC_SHIFT                (0x00U)

#define PMIC_RTC_CTRL_2_FIRST_STARTUP_DONE_SHIFT      (0x07U)
#define PMIC_RTC_CTRL_2_STARTUP_DEST_SHIFT            (0x05U)
#define PMIC_RTC_CTRL_2_FAST_BIST_SHIFT               (0x04U)
#define PMIC_RTC_CTRL_2_LP_STANDBY_SEL_SHIFT          (0x03U)
#define PMIC_RTC_CTRL_2_XTAL_SEL_SHIFT                (0x01U)
#define PMIC_RTC_CTRL_2_XTAL_EN_SHIFT                 (0x00U)

#define PMIC_RTC_RESET_STATUS_RESET_STATUS_RTC_SHIFT  (0x00U)

#define PMIC_RTC_COMP_MSB_COMP_MSB_RTC_SHIFT          (0x08U)
#define PMIC_RTC_COMP_LSB_COMP_LSB_RTC_SHIFT          (0x00U)

/*!
 * \brief   PMIC RTC and alarm time and date bit mask
 */
#define PMIC_RTC_SECONDS_SECOND_1_MASK       (uint8_t)(0x07U <<        \
                                               PMIC_RTC_SECONDS_SECOND_1_SHIFT)

#define PMIC_RTC_SECONDS_SECOND_0_MASK       (uint8_t)(0x0FU <<        \
                                               PMIC_RTC_SECONDS_SECOND_0_SHIFT)

#define PMIC_RTC_MINUTES_MINUTE_1_MASK       (uint8_t)(0x07U <<        \
                                               PMIC_RTC_MINUTES_MINUTE_1_SHIFT)
#define PMIC_RTC_MINUTES_MINUTE_0_MASK       (uint8_t)(0x0FU <<        \
                                               PMIC_RTC_MINUTES_MINUTE_0_SHIFT)

#define PMIC_RTC_HOURS_PM_NAM_MASK           (uint8_t)(0x01U <<        \
                                                   PMIC_RTC_HOURS_PM_NAM_SHIFT)

#define PMIC_RTC_HOURS_HOUR_1_MASK           (uint8_t)(0x03U <<        \
                                                   PMIC_RTC_HOURS_HOUR_1_SHIFT)

#define PMIC_RTC_HOURS_HOUR_0_MASK           (uint8_t)(0x0FU <<        \
                                                   PMIC_RTC_HOURS_HOUR_0_SHIFT)

#define PMIC_RTC_DAYS_DAY_1_MASK             (uint8_t)(0x03U <<        \
                                                   PMIC_RTC_DAYS_DAY_1_SHIFT)

#define PMIC_RTC_DAYS_DAY_0_MASK             (uint8_t)(0x0FU <<        \
                                                   PMIC_RTC_DAYS_DAY_0_SHIFT)

#define PMIC_RTC_MONTHS_MONTH_1_MASK         (uint8_t)(0x01U <<        \
                                                 PMIC_RTC_MONTHS_MONTH_1_SHIFT)

#define PMIC_RTC_MONTHS_MONTH_0_MASK         (uint8_t)(0x0FU <<        \
                                                 PMIC_RTC_MONTHS_MONTH_0_SHIFT)

#define PMIC_RTC_YEARS_YEAR_1_MASK           (uint8_t)(0x0FU <<        \
                                                   PMIC_RTC_YEARS_YEAR_1_SHIFT)

#define PMIC_RTC_YEARS_YEAR_0_MASK           (uint8_t)(0x0FU <<        \
                                                   PMIC_RTC_YEARS_YEAR_0_SHIFT)

#define PMIC_ALARM_SECONDS_ALR_SECOND_1_MASK          (uint8_t)(0x07U <<      \
                                         PMIC_ALARM_SECONDS_ALR_SECOND_1_SHIFT)

#define PMIC_ALARM_SECONDS_ALR_SECOND_0_MASK          (uint8_t)(0x0FU <<      \
                                         PMIC_ALARM_SECONDS_ALR_SECOND_0_SHIFT)

#define PMIC_ALARM_MINUTES_ALR_MINUTE_1_MASK          (uint8_t)(0x07U <<      \
                                         PMIC_ALARM_MINUTES_ALR_MINUTE_1_SHIFT)
#define PMIC_ALARM_MINUTES_ALR_MINUTE_0_MASK          (uint8_t)(0x0FU <<      \
                                         PMIC_ALARM_MINUTES_ALR_MINUTE_0_SHIFT)

#define PMIC_ALARM_HOURS_ALR_PM_NAM_MASK              (uint8_t)(0x01U <<      \
                                             PMIC_ALARM_HOURS_ALR_PM_NAM_SHIFT)

#define PMIC_ALARM_HOURS_ALR_HOUR_1_MASK              (uint8_t)(0x03U <<      \
                                             PMIC_ALARM_HOURS_ALR_HOUR_1_SHIFT)

#define PMIC_ALARM_HOURS_ALR_HOUR_0_MASK              (uint8_t)(0x0FU <<      \
                                         PMIC_ALARM_HOURS_ALR_HOUR_0_SHIFT)

#define PMIC_ALARM_DAYS_ALR_DAY_1_MASK                (uint8_t)(0x03U <<      \
                                               PMIC_ALARM_DAYS_ALR_DAY_1_SHIFT)

#define PMIC_ALARM_DAYS_ALR_DAY_0_MASK                (uint8_t)(0x0FU <<      \
                                         PMIC_ALARM_DAYS_ALR_DAY_0_SHIFT)

#define PMIC_ALARM_MONTHS_ALR_MONTH_1_MASK            (uint8_t)(0x01U <<      \
                                           PMIC_ALARM_MONTHS_ALR_MONTH_1_SHIFT)

#define PMIC_ALARM_MONTHS_ALR_MONTH_0_MASK            (uint8_t)(0x0FU <<      \
                                           PMIC_ALARM_MONTHS_ALR_MONTH_0_SHIFT)

#define PMIC_ALARM_YEARS_ALR_YEAR_1_MASK              (uint8_t)(0x0FU <<      \
                                             PMIC_ALARM_YEARS_ALR_YEAR_1_SHIFT)

#define PMIC_ALARM_YEARS_ALR_YEAR_0_MASK              (uint8_t)(0x0FU <<      \
                                             PMIC_ALARM_YEARS_ALR_YEAR_0_SHIFT)

#define PMIC_RTC_WEEKS_WEEK_MASK                      (uint8_t)(0x07U <<      \
                                                     PMIC_RTC_WEEKS_WEEK_SHIFT)

#define PMIC_RTC_STATUS_ALARM_MASK                    (uint8_t)(0x01U <<      \
                                                   PMIC_RTC_STATUS_ALARM_SHIFT)

#define PMIC_RTC_STATUS_TIMER_MASK                    (uint8_t)(0x01U <<      \
                                                   PMIC_RTC_STATUS_TIMER_SHIFT)

#define PMIC_RTC_STATUS_RUN_MASK                      (uint8_t)(0x01U <<      \
                                                     PMIC_RTC_STATUS_RUN_SHIFT)

#define PMIC_RTC_STATUS_POWER_UP_MASK                 (uint8_t)(0x01U <<      \
                                                PMIC_RTC_STATUS_POWER_UP_SHIFT)

/*!
 * \brief   PMIC RTC CTRL bit mask
 */
#define PMIC_RTC_CTRL_1_RTC_V_OPT_MASK                (uint8_t)(0x01U <<      \
                                               PMIC_RTC_CTRL_1_RTC_V_OPT_SHIFT)

#define PMIC_RTC_CTRL_1_GET_TIME_MASK                 (uint8_t)(0x01U <<      \
                                                PMIC_RTC_CTRL_1_GET_TIME_SHIFT)

#define PMIC_RTC_CTRL_1_SET_32_COUNTER_MASK           (uint8_t)(0x01U <<      \
                                          PMIC_RTC_CTRL_1_SET_32_COUNTER_SHIFT)

#define PMIC_RTC_CTRL_1_MODE_12_24_MASK               (uint8_t)(0x01U <<      \
                                              PMIC_RTC_CTRL_1_MODE_12_24_SHIFT)

#define PMIC_RTC_CTRL_1_ROUND_30S_MASK                (uint8_t)(0x01U <<      \
                                               PMIC_RTC_CTRL_1_ROUND_30S_SHIFT)

#define PMIC_RTC_CTRL_1_STOP_RTC_MASK                 (uint8_t)(0x01U <<      \
                                                PMIC_RTC_CTRL_1_STOP_RTC_SHIFT)

#define PMIC_RTC_CTRL_1_AUTO_COMP_MASK                (uint8_t)(0x01U <<      \
                                               PMIC_RTC_CTRL_1_AUTO_COMP_SHIFT)

#define PMIC_RTC_CTRL_2_FIRST_STARTUP_DONE_MASK       (uint8_t)(0x01U <<      \
                                      PMIC_RTC_CTRL_2_FIRST_STARTUP_DONE_SHIFT)

#define PMIC_RTC_CTRL_2_STARTUP_DEST_MASK             (uint8_t)(0x03U <<      \
                                            PMIC_RTC_CTRL_2_STARTUP_DEST_SHIFT)

#define PMIC_RTC_CTRL_2_FAST_BIST_MASK                (uint8_t)(0x01U <<      \
                                               PMIC_RTC_CTRL_2_FAST_BIST_SHIFT)

#define PMIC_RTC_CTRL_2_LP_STANDBY_SEL_MASK           (uint8_t)(0x01U <<      \
                                          PMIC_RTC_CTRL_2_LP_STANDBY_SEL_SHIFT)

#define PMIC_RTC_CTRL_2_XTAL_SEL_MASK                 (uint8_t)(0x03U <<      \
                                                PMIC_RTC_CTRL_2_XTAL_SEL_SHIFT)

#define PMIC_RTC_CTRL_2_XTAL_EN_MASK                  (uint8_t)(0x01U <<      \
                                                 PMIC_RTC_CTRL_2_XTAL_EN_SHIFT)

/*!
 * \brief   PMIC RTC Timer period bit mask
 */
#define PMIC_RTC_INTERRUPTS_EVERY_MASK                (uint8_t)(0x03U <<      \
                                               PMIC_RTC_INTERRUPTS_EVERY_SHIFT)

/*!
 * \brief   PMIC RTC Interrupt bit mask
 */
#define PMIC_RTC_INTERRUPTS_IT_ALARM_MASK             (uint8_t)(0x01U <<      \
                                            PMIC_RTC_INTERRUPTS_IT_ALARM_SHIFT)

#define PMIC_RTC_INTERRUPTS_IT_TIMER_MASK             (uint8_t)(0x01U <<      \
                                            PMIC_RTC_INTERRUPTS_IT_TIMER_SHIFT)

#define PMIC_RTC_RESET_STATUS_RESET_STATUS_RTC_MASK   (uint8_t)(0x01U <<      \
                                  PMIC_RTC_RESET_STATUS_RESET_STATUS_RTC_SHIFT)

#define PMIC_RTC_COMP_MSB_COMP_MSB_RTC_MASK     (uint16_t)((uint16_t)0xFFU << \
                                  PMIC_RTC_COMP_MSB_COMP_MSB_RTC_SHIFT)
#define PMIC_RTC_COMP_LSB_COMP_LSB_RTC_MASK           (uint8_t)(0xFFU <<      \
                                  PMIC_RTC_COMP_LSB_COMP_LSB_RTC_SHIFT)
/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_RTC_TPS6594X_PRIV_H_ */

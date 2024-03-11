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
 *   @file    pmic_sw_shutdown_priv.h
 *
 *   @brief   This file contains the private MACRO's and function definitions
 * for PMIC SW SHUTDOWN Timer state configuration
 *
 */

#ifndef PMIC_INC_SW_SHUTDOWN_PRIV_H_
#define PMIC_INC_SW_SHUTDOWN_PRIV_H_

/*!
 * \brief  PMIC SW SHUTDOWN register Addresses
 */
#define PMIC_OFF_STATE_STAT1_REGADDR                (0x0FU)
#define PMIC_OFF_STATE_STAT2_REGADDR                (0x10U)
#define PMIC_OFF_STATE_CLR_REGADDR                  (0x11U)
#define PMIC_THERMAL_STAT1_REGADDR                  (0x63U)
#define PMIC_THERMAL_STAT2_REGADDR                  (0x64U)

/*!
 * \brief  PMIC OFF_STATE_STAT1 register Shift Values
 */
#define PMIC_NORMAL_OFF_SHIFT                       (0x00U)
#define PMIC_OFF_INT_EVT_ERR_SHIFT                  (0x01U)
#define PMIC_OFF_PROT_EVT_SHIFT                     (0x02U)
#define PMIC_FIRST_PWR_ON_SHIFT                     (0x04U)
#define PMIC_CLK_ERR_SHIFT                          (0x05U)
#define PMIC_INTERNAL_OV_SHIFT                      (0x06U)
#define PMIC_INIT_AN_TMO_SHIFT                      (0x07U)

/*!
 * \brief  PMIC OFF_STATE_STAT2 register Shift Values
 */
#define PMIC_CRC_ERR_SHIFT                          (0x00U)
#define PMIC_SYS_CLK_ERR_PROT_SHIFT                 (0x01U)
#define PMIC_RST_MCU_TMO_SHIFT                      (0x02U)
#define PMIC_BGXM_ERR_SHIFT                         (0x03U)
#define PMIC_VBAT_OVP_ERR_SHIFT                     (0x04U)
#define PMIC_BB_OVP_ERR_SHIFT                       (0x05U)
#define PMIC_BB_BST_TMO_SHIFT                       (0x06U)
#define PMIC_BB_PK_ILIM_ERR_SHIFT                   (0x07U)

/*!
 * \brief  PMIC THERMAL_STAT1 register Shift Values
 */
#define PMIC_TSTAT1_LDO1_TPRE_ERR_SHIFT             (0x00U)
#define PMIC_TSTAT1_LDO1_TSD_ERR_SHIFT              (0x01U)
#define PMIC_TSTAT1_LDO2_TPRE_ERR_SHIFT             (0x02U)
#define PMIC_TSTAT1_LDO2_TSD_ERR_SHIFT              (0x03U)
#define PMIC_TSTAT1_LDO3_TPRE_ERR_SHIFT             (0x04U)
#define PMIC_TSTAT1_LDO3_TSD_ERR_SHIFT              (0x05U)
#define PMIC_TSTAT1_LDO4_TPRE_ERR_SHIFT             (0x06U)
#define PMIC_TSTAT1_LDO4_TSD_ERR_SHIFT              (0x07U)

/*!
 * \brief  PMIC THERMAL_STAT2 register Shift Values
 */
#define PMIC_TSTAT2_PLDO1_TPRE_ER_SHIFT             (0x00U)
#define PMIC_TSTAT2_PLDO1_TSD_ERR_SHIFT             (0x01U)
#define PMIC_TSTAT2_PLDO2_TPRE_ER_SHIFT             (0x02U)
#define PMIC_TSTAT2_PLDO2_TSD_ERR_SHIFT             (0x03U)
#define PMIC_TSTAT2_BB_TPRE_ERR_SHIFT               (0x04U)
#define PMIC_TSTAT2_BB_TSD_ERR_SHIFT                (0x05U)

/*!
 * \brief  PMIC OFF_STATE_CLR register Shift Values
 */
#define PMIC_OFF_STATE_STAT_CLR_SHIFT               (0x00U)

/*!
 * \brief  PMIC OFF_STATE_STAT1 register Mask Values
 */
#define PMIC_NORMAL_OFF_MASK ((uint8_t)(0X01U << PMIC_NORMAL_OFF_SHIFT))
#define PMIC_OFF_INT_EVT_ERR_MASK                                              \
  ((uint8_t)(0X01U << PMIC_OFF_INT_EVT_ERR_SHIFT))
#define PMIC_OFF_PROT_EVT_MASK ((uint8_t)(0X01U << PMIC_OFF_PROT_EVT_SHIFT))
#define PMIC_FIRST_PWR_ON_MASK ((uint8_t)(0X01U << PMIC_FIRST_PWR_ON_SHIFT))
#define PMIC_CLK_ERR_MASK ((uint8_t)(0X01U << PMIC_CLK_ERR_SHIFT))
#define PMIC_INTERNAL_OV_MASK ((uint8_t)(0X01U << PMIC_INTERNAL_OV_SHIFT))
#define PMIC_INIT_AN_TMO_MASK ((uint8_t)(0X01U << PMIC_INIT_AN_TMO_SHIFT))

/*!
 * \brief  PMIC OFF_STATE_STAT2 register Mask Values
 */
#define PMIC_CRC_ERR_MASK ((uint8_t)(0X01U << PMIC_CRC_ERR_SHIFT))
#define PMIC_SYS_CLK_ERR_PROT_MASK                                             \
  ((uint8_t)(0X01U << PMIC_SYS_CLK_ERR_PROT_SHIFT))
#define PMIC_RST_MCU_TMO_MASK ((uint8_t)(0X01U << PMIC_RST_MCU_TMO_SHIFT))
#define PMIC_BGXM_ERR_MASK ((uint8_t)(0X01U << PMIC_BGXM_ERR_SHIFT))
#define PMIC_VBAT_OVP_ERR_MASK ((uint8_t)(0X01U << PMIC_VBAT_OVP_ERR_SHIFT))
#define PMIC_BB_OVP_ERR_MASK ((uint8_t)(0X01U << PMIC_BB_OVP_ERR_SHIFT))
#define PMIC_BB_BST_TMO_MASK ((uint8_t)(0X01U << PMIC_BB_BST_TMO_SHIFT))
#define PMIC_BB_PK_ILIM_ERR_MASK ((uint8_t)(0X01U << PMIC_BB_PK_ILIM_ERR_SHIFT))

/*!
 * \brief  PMIC OFF_STATE_CLR register Mask Values
 */
#define PMIC_OFF_STATE_STAT_CLR_MASK                                           \
  ((uint8_t)(0X01U << PMIC_OFF_STATE_STAT_CLR_SHIFT))

/*!
 * \brief  PMIC THERMAL_STAT1 register Mask Values
 */
#define PMIC_TSTAT1_LDO1_T_PRE_ERR_MASK                                        \
  ((uint8_t)(0X01U << PMIC_TSTAT1_LDO1_TPRE_ERR_SHIFT))
#define PMIC_TSTAT1_LDO1_TSD_ERR_MASK                                          \
  ((uint8_t)(0X01U << PMIC_TSTAT1_LDO1_TSD_ERR_SHIFT))
#define PMIC_TSTAT1_LDO2_T_PRE_ERR_MASK                                        \
  ((uint8_t)(0X01U << PMIC_TSTAT1_LDO2_TPRE_ERR_SHIFT))
#define PMIC_TSTAT1_LDO2_TSD_ERR_MASK                                          \
  ((uint8_t)(0X01U << PMIC_TSTAT1_LDO2_TSD_ERR_SHIFT))
#define PMIC_TSTAT1_LDO3_T_PRE_ERR_MASK                                        \
  ((uint8_t)(0X01U << PMIC_TSTAT1_LDO3_TPRE_ERR_SHIFT))
#define PMIC_TSTAT1_LDO3_TSD_ERR_MASK                                          \
  ((uint8_t)(0X01U << PMIC_TSTAT1_LDO3_TSD_ERR_SHIFT))
#define PMIC_TSTAT1_LDO4_T_PRE_ERR_MASK                                        \
  ((uint8_t)(0X01U << PMIC_TSTAT1_LDO4_TPRE_ERR_SHIFT))
#define PMIC_TSTAT1_LDO4_TSD_ERR_MASK                                          \
  ((uint8_t)(0X01U << PMIC_TSTAT1_LDO4_TSD_ERR_SHIFT))

/*!
 * \brief  PMIC THERMAL_STAT2 register Mask Values
 */
#define PMIC_TSTAT2_PLDO1_TPRE_ER_MASK                                         \
  ((uint8_t)(0X01U << PMIC_TSTAT2_PLDO1_TPRE_ER_SHIFT))
#define PMIC_TSTAT2_PLDO1_TSD_ERR_MASK                                         \
  ((uint8_t)(0X01U << PMIC_TSTAT2_PLDO1_TSD_ERR_SHIFT))
#define PMIC_TSTAT2_PLDO2_TPRE_ER_MASK                                         \
  ((uint8_t)(0X01U << PMIC_TSTAT2_PLDO2_TPRE_ER_SHIFT))
#define PMIC_TSTAT2_PLDO2_TSD_ERR_MASK                                         \
  ((uint8_t)(0X01U << PMIC_TSTAT2_PLDO2_TSD_ERR_SHIFT))
#define PMIC_TSTAT2_BB_T_PRE_ERR_MASK                                          \
  ((uint8_t)(0X01U << PMIC_TSTAT2_BB_TPRE_ERR_SHIFT))
#define PMIC_TSTAT2_BB_TSD_ERR_MASK                                            \
  ((uint8_t)(0X01U << PMIC_TSTAT2_BB_TSD_ERR_SHIFT))

/* OFF_STATE_STAT1 Configuration Data */

#define PMIC_NORMAL_OFF_DATA1                       (0x00U) /* No clock failure */
#define PMIC_NORMAL_OFF_DATA2                       (0x01U) /* Too slow low-power clock */

#define PMIC_OFF_INT_EVT_ERR_DATA1                  (0x00U) /* No OFF state interrupt event error */
#define PMIC_OFF_INT_EVT_ERR_DATA2                  (0x01U) /* OFF state interrupt event error */

#define PMIC_OFF_PROT_EVT_DATA1                     (0x00U) /* No OFF state protection event error */
#define PMIC_OFF_PROT_EVT_DATA2                     (0x01U) /* OFF state protection event error */

#define PMIC_FIRST_PWR_ON_DATA1                     (0x00U) /* the device has not been in the unpowered state since this bit was cleared */

#define PMIC_CLK_ERR_DATA1                          (0x00U) /* No clock failure */
#define PMIC_CLK_ERR_DATA2                          (0x01U) /* Low power or system clock error */

#define PMIC_INTERNAL_OV_DATA1                      (0x00U) /* the device has not been in the unpowered state since this bit was cleared */
#define PMIC_INTERNAL_OV_DATA2                      (0x01U) /* An internal OV was detected in V1P8, VREG or VSAFETY. */

#define PMIC_INIT_AN_TMO_DATA1                      (0x00U) /* the device has not been in the unpowered state since this bit was cleared */
#define PMIC_INIT_AN_TMO_DATA2                      (0x01U) /* The INIT_AN_TMO timeout indicates a powerup issue where the digital did not start before the timeout */


#define PMIC_CRC_ERR_DATA1                          (0x00U) /* No CRC Error */
#define PMIC_CRC_ERR_DATA2                          (0x01U) /* CRC Error  */

#define PMIC_SYS_CLK_ERR_PROT_DATA1                 (0x00U) /* No clock failure  */
#define PMIC_SYS_CLK_ERR_PROT_DATA2                 (0x01U) /* clock failure  */

#define PMIC_RST_MCU_TMO_DATA1                      (0x00U) /* No RESET-MCU Timeout */
#define PMIC_RST_MCU_TMO_DATA2                      (0x01U) /* RESET-MCU Timeout */

#define PMIC_BGXM_ERR_DATA1                         (0x00U) /* No BGXM error */
#define PMIC_BGXM_ERR_DATA2                         (0x01U) /* BGXM error */

#define PMIC_VBAT_OVP_ERR_DATA1                     (0x00U) /* No Over-voltage */
#define PMIC_VBAT_OVP_ERR_DATA2                     (0x01U) /* Over-voltage */

#define PMIC_BB_OVP_ERR_DATA1                       (0x00U) /* No Over-voltage */
#define PMIC_BB_OVP_ERR_DATA2                       (0x01U) /* Over-voltage */

#define PMIC_BB_BST_TMO_DATA1                       (0x00U) /* No buck-boost boost mode timeout */
#define PMIC_BB_BST_TMO_DATA2                       (0x01U)

#define PMIC_BB_PK_ILIM_ERR_DATA1                   (0x00U) /* No current-limit */
#define PMIC_BB_PK_ILIM_ERR_DATA2                   (0x01U) /* Current-limit */

/* OFF_STATE_CLR Configuration Data */
#define PMIC_OFF_STATE_STAT_CLR_DATA                (0x01U)

/* THERMAL_STAT Configuration Data */
#define PMIC_THERMAL_STAT_TPRE_ER_DATA1             (0x00U) /* No Over-temperature */
#define PMIC_THERMAL_STAT_TPRE_ER_DATA2             (0x01U) /* Over-temperature */

#define PMIC_THERMAL_STAT_TSD_ERR_DATA1             (0x00U)
#define PMIC_THERMAL_STAT_TSD_ERR_DATA2             (0x01U)
#define PMIC_THERMAL_STAT_CLEAR_DATA                (0x01U)

#endif /* PMIC_INC_SW_SHUTDOWN_PRIV_H_ */

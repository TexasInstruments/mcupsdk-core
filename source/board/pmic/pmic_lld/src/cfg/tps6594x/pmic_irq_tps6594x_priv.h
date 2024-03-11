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
 * \file   pmic_irq_tps6594x_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         configuring PMIC IRQ.
 */

#ifndef PMIC_IRQ_TPS6594X_PRIV_H_
#define PMIC_IRQ_TPS6594X_PRIV_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */
#include <pmic_irq_priv.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/*!
 *  \brief  PMIC Interrupt Hierarchy Level 1 Register offsets.
 */
#define PMIC_INT_LDO_VMON_REGADDR             (0x5FU)

/** Interrupt Hierarchy Level 2 Register offsets */
/*!
 * \brief  INT_BUCK Sources
 */
#define PMIC_INT_BUCK5_REGADDR                (0x5EU)

/*!
 * \brief  INT_LDO_VMON Sources
 */
#define PMIC_INT_LDO1_2_REGADDR               (0x60U)
#define PMIC_INT_LDO3_4_REGADDR               (0x61U)

/*!
 * \brief  Interrupt MASK registers address
 */
#define PMIC_MASK_GPIO9_11_REGADDR        (0x51U)
#define PMIC_MASK_BUCK5_REGADDR           (0x4BU)
#define PMIC_MASK_LDO1_2_REGADDR          (0x4CU)
#define PMIC_MASK_LDO3_4_REGADDR          (0x4DU)

/*!
 * \brief  INT_TOP Register Bit Masks
 */
#define PMIC_INT_TOP_LDO_VMON_INT_MASK             (0x02U)

/*!
 * \brief  STARTUP Error Interrupt Mask
 */
#define PMIC_INT_STARTUP_RTC_INT_MASK          (0x04U)

/*!
 * \brief  LDO Interrupt Mask
 */
#define PMIC_INT_LDO_VMON_LDO1_2_INT_MASK      (0x01U)
#define PMIC_INT_LDO_VMON_LDO3_4_INT_MASK      (0x02U)
#define PMIC_INT_LDO_VMON_VCCA_INT_MASK        (0x10U)


#define PMIC_INT_GPIO_GPIO11_INT_MASK                   (0x04U)
#define PMIC_INT_STARTUP_NPWRON_START_INT_MASK          (0x01U)
#define PMIC_INT_MODERATE_ERR_NPWRON_LONG_INT_MASK      (0x20U)

/*!
 * \brief  PMIC_INT_BUCK Register Bit Mask
 */
#define PMIC_INT_BUCK_BUCK5_INT_MASK           (0x04U)

/*!
 * \brief  INT_STARTUP Sources
 */
#define PMIC_RTC_STATUS_REGADDR                 (0xC4U)
#define PMIC_RTC_INTERRUPTS_REGADDR             (0xC5U)

/*!
 * \brief  PMIC_INT_BUCK5 Register Bit Masks
 */
#define PMIC_INT_BUCK5_BUCK5_OV_INT_MASK                         (0x01U)
#define PMIC_INT_BUCK5_BUCK5_UV_INT_MASK                         (0x02U)
#define PMIC_INT_BUCK5_BUCK5_SC_INT_MASK                         (0x04U)
#define PMIC_INT_BUCK5_BUCK5_ILIM_INT_MASK                       (0x08U)

/*!
 * \brief  PMIC_INT_LDO1_2 Register Bit Masks
 */
#define PMIC_INT_LDO1_2_LDO1_OV_INT_MASK                         (0x01U)
#define PMIC_INT_LDO1_2_LDO1_UV_INT_MASK                         (0x02U)
#define PMIC_INT_LDO1_2_LDO1_SC_INT_MASK                         (0x04U)
#define PMIC_INT_LDO1_2_LDO1_ILIM_INT_MASK                       (0x08U)
#define PMIC_INT_LDO1_2_LDO2_OV_INT_MASK                         (0x10U)
#define PMIC_INT_LDO1_2_LDO2_UV_INT_MASK                         (0x20U)
#define PMIC_INT_LDO1_2_LDO2_SC_INT_MASK                         (0x40U)
#define PMIC_INT_LDO1_2_LDO2_ILIM_INT_MASK                       (0x80U)

/*!
 * \brief  PMIC_INT_LDO3_4 Register Bit Masks
 */
#define PMIC_INT_LDO3_4_LDO3_OV_INT_MASK                         (0x01U)
#define PMIC_INT_LDO3_4_LDO3_UV_INT_MASK                         (0x02U)
#define PMIC_INT_LDO3_4_LDO3_SC_INT_MASK                         (0x04U)
#define PMIC_INT_LDO3_4_LDO3_ILIM_INT_MASK                       (0x08U)
#define PMIC_INT_LDO3_4_LDO4_OV_INT_MASK                         (0x10U)
#define PMIC_INT_LDO3_4_LDO4_UV_INT_MASK                         (0x20U)
#define PMIC_INT_LDO3_4_LDO4_SC_INT_MASK                         (0x40U)
#define PMIC_INT_LDO3_4_LDO4_ILIM_INT_MASK                       (0x80U)

/*!
 * \brief  PMIC Interrupt Register Bit positions
 */
/*! PMIC_INT_MODERATE_ERR Register bit position  */
#define PMIC_INT_MODERATE_ERR_NPWRON_LONG_INT_SHIFT             (0x5U)

/*! PMIC_RTC_STATUS Register bit position  */
#define PMIC_RTC_STATUS_ALARM_SHIFT                             (0x06U)
#define PMIC_RTC_STATUS_TIMER_SHIFT                             (0x05U)

/*! PMIC_INT_STARTUP Register bit position  */
#define PMIC_INT_STARTUP_NPWRON_START_INT_SHIFT                 (0x0U)

/*! PMIC_INT_GPIO Register bit position  */
#define PMIC_INT_GPIO_GPIO11_INT_SHIFT                          (0x2U)

/*! PMIC_INT_LDO3_4 Register bit position  */
#define PMIC_INT_LDO3_4_LDO4_ILIM_INT_SHIFT                     (0x7U)
#define PMIC_INT_LDO3_4_LDO4_SC_INT_SHIFT                       (0x6U)
#define PMIC_INT_LDO3_4_LDO4_UV_INT_SHIFT                       (0x5U)
#define PMIC_INT_LDO3_4_LDO4_OV_INT_SHIFT                       (0x4U)
#define PMIC_INT_LDO3_4_LDO3_ILIM_INT_SHIFT                     (0x3U)
#define PMIC_INT_LDO3_4_LDO3_SC_INT_SHIFT                       (0x2U)
#define PMIC_INT_LDO3_4_LDO3_UV_INT_SHIFT                       (0x1U)
#define PMIC_INT_LDO3_4_LDO3_OV_INT_SHIFT                       (0x0U)

/*! PMIC_INT_LDO1_2 Register bit position  */
#define PMIC_INT_LDO1_2_LDO2_ILIM_INT_SHIFT                     (0x7U)
#define PMIC_INT_LDO1_2_LDO2_SC_INT_SHIFT                       (0x6U)
#define PMIC_INT_LDO1_2_LDO2_UV_INT_SHIFT                       (0x5U)
#define PMIC_INT_LDO1_2_LDO2_OV_INT_SHIFT                       (0x4U)
#define PMIC_INT_LDO1_2_LDO1_ILIM_INT_SHIFT                     (0x3U)
#define PMIC_INT_LDO1_2_LDO1_SC_INT_SHIFT                       (0x2U)
#define PMIC_INT_LDO1_2_LDO1_UV_INT_SHIFT                       (0x1U)
#define PMIC_INT_LDO1_2_LDO1_OV_INT_SHIFT                       (0x0U)

/*! PMIC_BUCK5 Register bit position  */
#define PMIC_INT_BUCK5_BUCK5_ILIM_INT_SHIFT                     (0x3U)
#define PMIC_INT_BUCK5_BUCK5_SC_INT_SHIFT                       (0x2U)
#define PMIC_INT_BUCK5_BUCK5_UV_INT_SHIFT                       (0x1U)
#define PMIC_INT_BUCK5_BUCK5_OV_INT_SHIFT                       (0x0U)

/*!
 * \brief  PMIC Mask Register Bit positions
 */
/*! PMIC_MASK_MODERATE_ERR Register Bit Position */
#define PMIC_MASK_MODERATE_ERR_NPWRON_LONG_MASK_SHIFT           (0x5U)

/*! PMIC_STARTUP Register Bit Position */
#define PMIC_MASK_STARTUP_NPWRON_START_MASK_SHIFT               (0x0U)

/*! PMIC_MASK_GPIO9_11 Register Bit Positions */
#define PMIC_MASK_GPIO9_11_GPIO10_RISE_MASK_SHIFT               (0x4U)
#define PMIC_MASK_GPIO9_11_GPIO9_RISE_MASK_SHIFT                (0x3U)
#define PMIC_MASK_GPIO9_11_GPIO10_FALL_MASK_SHIFT               (0x1U)
#define PMIC_MASK_GPIO9_11_GPIO9_FALL_MASK_SHIFT                (0x0U)
#define PMIC_MASK_GPIO9_11_GPIO11_FALL_MASK_SHIFT               (0x2U)
#define PMIC_MASK_GPIO9_11_GPIO11_RISE_MASK_SHIFT               (0x5U)

/*! PMIC_MASK_LDO3_4 Register Bit Position */
#define PMIC_MASK_LDO3_4_LDO4_ILIM_MASK_SHIFT                   (0x7U)
#define PMIC_MASK_LDO3_4_LDO4_UV_MASK_SHIFT                     (0x5U)
#define PMIC_MASK_LDO3_4_LDO4_OV_MASK_SHIFT                     (0x4U)
#define PMIC_MASK_LDO3_4_LDO3_ILIM_MASK_SHIFT                   (0x3U)
#define PMIC_MASK_LDO3_4_LDO3_UV_MASK_SHIFT                     (0x1U)
#define PMIC_MASK_LDO3_4_LDO3_OV_MASK_SHIFT                     (0x0U)

/*! PMIC_MASK_LDO1_2 Register Bit Position */
#define PMIC_MASK_LDO1_2_LDO2_ILIM_MASK_SHIFT                   (0x7U)
#define PMIC_MASK_LDO1_2_LDO2_UV_MASK_SHIFT                     (0x5U)
#define PMIC_MASK_LDO1_2_LDO2_OV_MASK_SHIFT                     (0x4U)
#define PMIC_MASK_LDO1_2_LDO1_ILIM_MASK_SHIFT                   (0x3U)
#define PMIC_MASK_LDO1_2_LDO1_UV_MASK_SHIFT                     (0x1U)
#define PMIC_MASK_LDO1_2_LDO1_OV_MASK_SHIFT                     (0x0U)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief  Function to get the PMIC Interrupt Registers for TPS6594x Leo PMIC.
 */
void pmic_get_tps6594x_intrCfg(Pmic_IntrCfg_t **pIntrCfg);

/*!
 * \brief  Function to get the PMIC GPIO Interrupt Mask Registers for
 *         TPS6594x Leo PMIC.
 */
void pmic_get_tps6594x_intrGpioCfg(Pmic_GpioIntrTypeCfg_t **pIntGpioCfg);

/*!
 * \brief  Function to decipher the L2 Error for TPS6594x Leo PMIC.
 */
int32_t Pmic_tps6594x_irqGetL2Error(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint16_t           l1RegAddr,
                                    Pmic_IrqStatus_t  *pErrStat);

/*!
 * \brief  Function to reinitialise Interrupt configuration based on PMIC
 *         Silicon Revision
 */
void Pmic_tps6594x_reInitInterruptConfig(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_IRQ_TPS6594X_PRIV_H_ */

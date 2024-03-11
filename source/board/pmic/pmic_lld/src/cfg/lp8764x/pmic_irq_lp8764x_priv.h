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
 * \file   pmic_irq_lp8764x_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         configuring PMIC IRQ.
 */

#ifndef PMIC_IRQ_LP8764X_PRIV_H_
#define PMIC_IRQ_LP8764X_PRIV_H_

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
 * \brief  Interrupt MASK registers address
 */
#define PMIC_MASK_GPIO9_10_REGADDR        (0x51U)

/*!
 * \brief  PMIC Interrupt Registers Mask values and bit positions
 */
/*! PMIC_INT_VMON Register Mask values */
#define PMIC_INT_VMON_VMON2_RV_INT_MASK       (0x80U)
#define PMIC_INT_VMON_VMON2_UV_INT_MASK       (0x40U)
#define PMIC_INT_VMON_VMON2_OV_INT_MASK       (0x20U)
#define PMIC_INT_VMON_VMON1_RV_INT_MASK       (0x10U)
#define PMIC_INT_VMON_VMON1_UV_INT_MASK       (0x08U)
#define PMIC_INT_VMON_VMON1_OV_INT_MASK       (0x04U)

/*! PMIC_INT_VMON Register Bit positions */
#define PMIC_INT_VMON_VMON2_RV_INT_SHIFT      (0x7U)
#define PMIC_INT_VMON_VMON2_UV_INT_SHIFT      (0x6U)
#define PMIC_INT_VMON_VMON2_OV_INT_SHIFT      (0x5U)
#define PMIC_INT_VMON_VMON1_RV_INT_SHIFT      (0x4U)
#define PMIC_INT_VMON_VMON1_UV_INT_SHIFT      (0x3U)
#define PMIC_INT_VMON_VMON1_OV_INT_SHIFT      (0x2U)

/*!
 * \brief  Individual interrupt bitmasks for GPIO9_10 RISE/FALL
 */
#define PMIC_MASK_GPIO9_10_GPIO10_RISE_MASK_MASK       (0x10U)
#define PMIC_MASK_GPIO9_10_GPIO9_RISE_MASK_MASK        (0x08U)
#define PMIC_MASK_GPIO9_10_GPIO10_FALL_MASK_MASK       (0x02U)
#define PMIC_MASK_GPIO9_10_GPIO9_FALL_MASK_MASK        (0x01U)

/*! PMIC GPIO9_10 MASK Register bit shift values */
#define PMIC_MASK_GPIO9_10_GPIO10_RISE_MASK_SHIFT       (0x4U)
#define PMIC_MASK_GPIO9_10_GPIO9_RISE_MASK_SHIFT        (0x3U)
#define PMIC_MASK_GPIO9_10_GPIO10_FALL_MASK_SHIFT       (0x1U)
#define PMIC_MASK_GPIO9_10_GPIO9_FALL_MASK_SHIFT        (0x0U)


/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief  Function to get the PMIC Interrupt Registers for LP8764x HERA PMIC.
 */
void pmic_get_lp8764x_intrCfg(Pmic_IntrCfg_t **pIntCfg);

/*!
 * \brief  Function to get the PMIC GPIO Interrupt Mask Registers for
 *         LP8764x Hera PMIC.
 */
void pmic_get_lp8764x_intrGpioCfg(Pmic_GpioIntrTypeCfg_t **pIntGpioCfg);

/*!
 * \brief  Function to decipher the L2 Error for LP8764x Hera PMIC.
 */
int32_t Pmic_lp8764x_irqGetL2Error(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   uint16_t           l1RegAddr,
                                   Pmic_IrqStatus_t  *pErrStat);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_IRQ_LP8764X_PRIV_H_ */

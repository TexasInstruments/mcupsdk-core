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
 * \file   pmic_gpio_lp8764x_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         LP8764x HERA PMIC driver specific PMIC gpio configuration
 *
 */

#ifndef PMIC_GPIO_LP8764X_PRIV_H_
#define PMIC_GPIO_LP8764X_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <pmic_gpio_priv.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/*!
 * \brief  GPIO Register Offsets
 */
#define PMIC_ENABLE_CONF_REGADDR                           (0x3CU)

/*!
 * \brief Bit fields for PMIC_MASK_GPIO9_10_REG
 */
#define PMIC_MASK_GPIO9_10_GPIO9_FALL_MASK_SHIFT           (0x0U)
#define PMIC_MASK_GPIO9_10_GPIO9_RISE_MASK_SHIFT           (0x3U)
#define PMIC_MASK_GPIO9_10_GPIO10_FALL_MASK_SHIFT          (0x1U)
#define PMIC_MASK_GPIO9_10_GPIO10_RISE_MASK_SHIFT          (0x4U)

/*!
 * \brief  GPIO Register Offsets
 */
#define PMIC_MASK_GPIO9_10_REGADDR                         (0x51U)

/*!
 * \brief  ENABLE Register bit fields
 */
#define PMIC_ENABLE_CONF_ENABLE_POL_SHIFT                  (0x05U)

/*!
 * \brief  ENABLE Register bit mask values
 */

#define PMIC_ENABLE_CONF_ENABLE_POL_MASK \
                    ((uint8_t)(0x01U << PMIC_ENABLE_CONF_ENABLE_POL_SHIFT))

/*!
 * \brief  Bit Mask for PMIC_MASK_GPIO9_10_REG
 */
#define PMIC_MASK_GPIO9_10_GPIO9_FALL_MASK_MASK                             \
                                ((uint8_t)(0x01 <<                          \
                                 PMIC_MASK_GPIO9_10_GPIO9_FALL_MASK_SHIFT))
#define PMIC_MASK_GPIO9_10_GPIO10_FALL_MASK_MASK                            \
                                ((uint8_t)(0x01 <<                          \
                                 PMIC_MASK_GPIO9_10_GPIO10_FALL_MASK_SHIFT))
#define PMIC_MASK_GPIO9_10_GPIO9_RISE_MASK_MASK                             \
                                ((uint8_t)(0x01 <<                          \
                                 PMIC_MASK_GPIO9_10_GPIO9_RISE_MASK_SHIFT))
#define PMIC_MASK_GPIO9_10_GPIO10_RISE_MASK_MASK                            \
                                ((uint8_t)(0x01 <<                          \
                                 PMIC_MASK_GPIO9_10_GPIO10_RISE_MASK_SHIFT))

/*
 * \brief  Min and Max PMIC GPIO pin supported
 */
#define PMIC_LP8764X_GPIO_PIN_MIN               (1U)
#define PMIC_LP8764X_GPIO_PIN_MAX               (10U)


/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/*
 * \brief  Function to get the PMIC GPIO Pins with Input Ouput Configuration
 *         for LP8764x HERA PMIC
 *
 * \param   pGpioInOutCfg   [OUT]  to store lp8764x gpio configuration
 */
void pmic_get_lp8764x_gpioInOutCfg(Pmic_GpioInOutCfg_t **pGpioInOutCfg);

/*
 * \brief  Function to get the PMIC GPIO Interrupt Register array for
 *         LP8764x HERA PMIC
 *
 * \param   pGpioIntRegCfg   [OUT]  to store lp8764x gpio register configuration
 */
void pmic_get_lp8764x_gpioIntRegCfg(Pmic_GpioIntRegCfg_t **pGpioIntRegCfg);

/*!
 * \brief   This function is used to configure Enable pin for LP8764X
 *          PMIC HERA Device.
 */
int32_t Pmic_gpioLp8764xSetEnablePinConfiguration(
                                          Pmic_CoreHandle_t   *pPmicCoreHandle,
                                          const Pmic_GpioCfg_t gpioCfg);

/*!
 * \brief   This function is used to read Enable pin configuration for LP8764X
 *          PMIC HERA Device.
 */
int32_t Pmic_gpioLp8764xGetEnablePinConfiguration(
                                          Pmic_CoreHandle_t   *pPmicCoreHandle,
                                          Pmic_GpioCfg_t      *pGpioCfg);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_GPIO_LP8764X_PRIV_H_ */

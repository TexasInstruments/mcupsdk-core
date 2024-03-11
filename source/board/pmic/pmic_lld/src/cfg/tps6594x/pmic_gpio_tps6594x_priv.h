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
 * \file   pmic_gpio_tps6594x_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         TPS6594x Leo PMIC driver specific PMIC gpio configuration
 *
 */

#ifndef PMIC_GPIO_TPS6594X_PRIV_H_
#define PMIC_GPIO_TPS6594X_PRIV_H_

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
#define PMIC_GPIO11_CONF_REGADDR                 (0x3BU)
#define PMIC_NPWRON_CONF_REGADDR                 (0x3CU)

/*!
 * \brief  NPWRON Register bit fields
 */
#define PMIC_NPWRON_CONF_NPWRON_SEL_SHIFT        (0x06U)
#define PMIC_NPWRON_CONF_ENABLE_POL_SHIFT        (0x05U)

/*!
 * \brief  GPIO IN Register bit fields
 */
#define PMIC_GPIO_IN_2_GPIO11_IN_SHIFT           (0x02U)
#define PMIC_GPIO_IN_2_NPWRON_IN_SHIFT           (0x03U)

/*!
 * \brief  GPIO OUT Register bit fields
 */
#define PMIC_GPIO_OUT_2_GPIO11_OUT_SHIFT         (0x02U)

/*!
 * \brief  NPWRON Register bit mask values
 */
#define PMIC_NPWRON_CONF_NPWRON_SEL_MASK    (uint8_t)  \
                                            (0x03U <<  \
                                             PMIC_NPWRON_CONF_NPWRON_SEL_SHIFT)
#define PMIC_NPWRON_CONF_ENABLE_POL_MASK    (uint8_t)  \
                                            (0x01U <<  \
                                             PMIC_NPWRON_CONF_ENABLE_POL_SHIFT)

/*!
 * \brief  GPIO IN Register bit mask values
 */
#define PMIC_GPIO_IN_2_GPIO11_IN_MASK       (uint8_t)  \
                                            (0x01U <<  \
                                             PMIC_GPIO_IN_2_GPIO11_IN_SHIFT)
#define PMIC_GPIO_IN_2_NPWRON_IN_MASK       (uint8_t)  \
                                            (0x01U <<  \
                                             PMIC_GPIO_IN_2_NPWRON_IN_SHIFT)

/*!
 * \brief  GPIO OUT Register bit mask values
 */
#define PMIC_GPIO_OUT_2_GPIO11_OUT_MASK      (uint8_t)  \
                                             (0x01U <<  \
                                              PMIC_GPIO_OUT_2_GPIO11_OUT_SHIFT)

/*!
 * \brief  Max and Min PMIC GPIO pin supported
 */
#define PMIC_TPS6594X_GPIO_PIN_MIN              (1U)
#define PMIC_TPS6594X_GPIO_PIN_MAX              (11U)

/** \brief Max value for NPWRON/ENABLE pin Function */
#define PMIC_TPS6594X_NPWRON_PINFUNC_MAX        (1U)
/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/*!
 * \brief  Get PMIC GPIO pin Input Ouput Configuration
 *         This function is used to read the PMIC GPIO Pins with Input Ouput
 *         Configuration
 *
 * \param  pGpioInOutCfg   [OUT]  Pointer to store gpio Input Ouput
 *                                configuration
 */
void pmic_get_tps6594x_gpioInOutCfg(Pmic_GpioInOutCfg_t **pGpioInOutCfg);

/*!
 * \brief  Get PMIC GPIO pin Interrupt Register configuration
 *         This function is used to read the PMIC GPIO Interrupt Register
 *         configuration
 *
 * \param   pGpioIntRegCfg   [OUT]  Pointer to store gpio Interrupt Register
 *                                  configuration
*/
void pmic_get_tps6594x_gpioIntRegCfg(Pmic_GpioIntRegCfg_t **pGpioIntRegCfg);

/*!
 * \brief   This function is used to configure NPWRON pin for TPS6594x
 *          PMIC LEO Device.
 */
int32_t Pmic_gpioTps6594xSetNPwronPinConfiguration(
                                          Pmic_CoreHandle_t   *pPmicCoreHandle,
                                          const Pmic_GpioCfg_t gpioCfg);

/*!
 * \brief   This function is used to read NPWRON pin configuration for TPS6594x
 *          PMIC LEO Device.
 */
int32_t Pmic_gpioTps6594xGetNPwronPinConfiguration(
                                        Pmic_CoreHandle_t   *pPmicCoreHandle,
                                        Pmic_GpioCfg_t      *pGpioCfg);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_GPIO_TPS6594X_PRIV_H_ */

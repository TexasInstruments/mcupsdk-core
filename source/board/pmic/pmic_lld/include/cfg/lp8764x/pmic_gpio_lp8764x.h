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
 *  \addtogroup DRV_PMIC_GPIO_MODULE
 *
 *  @{
 */

/**
 * \file   pmic_gpio_lp8764x.h
 *
 * \brief  PMIC LP8764x Hera PMIC GPIO API/interface file.
 *
 */

#ifndef PMIC_GPIO_LP8764X_H_
#define PMIC_GPIO_LP8764X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_Lp8764xHera_GpioPin
 *  \name   PMIC GPIO supported pins for LP8764x HERA Device
 *
 *  @{
 */
#define PMIC_LP8764X_GPIO1_PIN                             (1U)
#define PMIC_LP8764X_GPIO2_PIN                             (2U)
#define PMIC_LP8764X_GPIO3_PIN                             (3U)
#define PMIC_LP8764X_GPIO4_PIN                             (4U)
#define PMIC_LP8764X_GPIO5_PIN                             (5U)
#define PMIC_LP8764X_GPIO6_PIN                             (6U)
#define PMIC_LP8764X_GPIO7_PIN                             (7U)
#define PMIC_LP8764X_GPIO8_PIN                             (8U)
#define PMIC_LP8764X_GPIO9_PIN                             (9U)
#define PMIC_LP8764X_GPIO10_PIN                            (10U)
/*  @} */

/**
 *  \anchor Pmic_Lp8764xHera_GpioPinFunc
 *  \name   PMIC GPIO pin functions supported by LP8764x HERA PMIC
 *
 *  @{
 */
/** \brief Used to configure GPIO Pin Function.
 *         Valid for all GPIO Pins */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO                     (0U)
/** \brief Used to configure EN_DRV Pin Function.
 *         Valid only for GPIO1 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO1_EN_DRV             (1U)
/** \brief Used to configure SCL_I2C2 Pin Function.
 *         Valid only for GPIO2 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO2_SCL_I2C2           (1U)
/** \brief Used to configure SDA_I2C2 Pin Function.
 *         Valid only for GPIO3 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO3_SDA_I2C2           (1U)
/** \brief Used to configure ENABLE Pin Function.
 *         Valid only for GPIO4 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO4_ENABLE             (1U)
/** \brief Used to configure SYNCCLKIN Pin Function.
 *         Valid only for GPIO5 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO5_SYNCCLKIN          (1U)
/** \brief Used to configure nERR_MCU Pin Function.
 *         Valid only for GPIO7 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO6_GPIO7_NERR_MCU     (1U)
/** \brief Used to configure SCLK_SPMI Pin Function.
 *         Valid only for GPIO8 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO8_SCLK_SPMI          (1U)
/** \brief Used to configure SDATA_SPMI Pin Function.
 *         Valid only for GPIO9 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO9_SDATA_SPMI         (1U)
/** \brief Used to configure nRSTOUT Pin Function.
 *         Valid only for GPIO10 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO10_NRSTOUT           (1U)
/** \brief Used to configure nRSTOUT_SOC Pin Function.
 *         Valid only for GPIO1 and GPIO10 Pins */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO1_GPIO10_NRSTOUT_SOC (2U)
/** \brief Used to configure CS_SPI Pin Function.
 *         Valid only for GPIO2 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO2_CS_SPI             (2U)
/** \brief Used to configure SDO_SPI Pin Function.
 *         Valid only for GPIO3 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO3_SDO_SPI            (2U)
/** \brief Used to configure TRIG_WDOG Pin Function.
 *         Valid only for GPIO4 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO4_TRIG_WDOG          (2U)
/** \brief Used to configure SYNCCLKOUT Pin Function.
 *         Valid only for GPIO5 and GPIO6 Pins */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO5_GPIO6_SYNCCLKOUT   (2U)
/** \brief Used to configure REFOUT Pin Function.
 *         Valid only for GPIO7 Pins */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO7_REFOUT             (2U)
/** \brief Used to configure VMON2 Pin Function.
 *         Valid only for GPIO8 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO8_VMON2              (2U)
/** \brief Used to configure PGOOD Pin Function.
 *         Valid only for GPIO9 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO9_PGOOD              (2U)
/** \brief Used to configure PGOOD Pin Function.
 *         Valid only for GPIO1 and GPIO6 Pins */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO1_GPIO6_PGOOD        (3U)
/** \brief Used to configure TRIG_WDOG Pin Function.
 *         Valid only for GPIO2 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO2_TRIG_WDOG          (3U)
/** \brief Used to configure BUCK1_VMON Pin Function.
 *         Valid only for GPIO4 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO4_BUCK1_VMON         (3U)
/** \brief Used to configure nRSTOUT_SOC Pin Function.
 *         Valid only for GPIO5 Pins */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO5_NRSTOUT_SOC        (3U)
/** \brief Used to configure VMON1 Pin Function.
 *         Valid only for GPIO7 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO7_VMON1              (3U)
/** \brief Used to configure SYNCCLKIN Pin Function.
 *         Valid only for GPIO9 Pin */
#define PMIC_LP8764X_GPIO_PINFUNC_GPIO9_SYNCCLKIN          (3U)
/** \brief Used to configure NSLEEP1 Pin Function.
 *         Valid only for GPIO1 to GPIO10 Pins */
#define PMIC_LP8764X_GPIO_PINFUNC_NSLEEP1                  (4U)
/** \brief Used to configure NSLEEP2 Pin Function.
 *         Valid only for GPIO1 to GPIO10 Pins */
#define PMIC_LP8764X_GPIO_PINFUNC_NSLEEP2                  (5U)
/** \brief Used to configure WKUP1 Pin Function.
 *         Valid only for GPIO1 to GPIO10 Pins */
#define PMIC_LP8764X_GPIO_PINFUNC_WKUP1                    (6U)
/** \brief Used to configure WKUP2 Pin Function.
 *         Valid only for GPIO1 to GPIO10 Pins */
#define PMIC_LP8764X_GPIO_PINFUNC_WKUP2                    (7U)
/*  @} */


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_GPIO_LP8764X_H_ */

/* @} */

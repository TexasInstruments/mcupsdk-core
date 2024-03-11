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
 * \file   pmic_gpio_tps6594x.h
 *
 * \brief  PMIC TPS6594x Leo PMIC GPIO API/interface file.
 *
 */

#ifndef PMIC_GPIO_TPS6594X_H_
#define PMIC_GPIO_TPS6594X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <pmic_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_Tps6594xLeo_GpioPin
 *  \name   PMIC GPIO supported pins for TPS6594x Leo Device
 *
 *  @{
 */
#define PMIC_TPS6594X_GPIO1_PIN                             (1U)
#define PMIC_TPS6594X_GPIO2_PIN                             (2U)
#define PMIC_TPS6594X_GPIO3_PIN                             (3U)
#define PMIC_TPS6594X_GPIO4_PIN                             (4U)
#define PMIC_TPS6594X_GPIO5_PIN                             (5U)
#define PMIC_TPS6594X_GPIO6_PIN                             (6U)
#define PMIC_TPS6594X_GPIO7_PIN                             (7U)
#define PMIC_TPS6594X_GPIO8_PIN                             (8U)
#define PMIC_TPS6594X_GPIO9_PIN                             (9U)
#define PMIC_TPS6594X_GPIO10_PIN                            (10U)
#define PMIC_TPS6594X_GPIO11_PIN                            (11U)
/*  @} */

/**
 *  \anchor Pmic_Tps6594xLeo_GpioPinFunc
 *  \name   PMIC GPIO pin functions supported for TPS6594x Leo Device
 *
 *  @{
 */
/** \brief Used to configure GPIO Pin Function.
 *         Valid for all GPIO Pins */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO                          (0U)
/** \brief Used to configure SCL_I2C2/CS_SPI Pin Function.
 *         Valid only for GPIO1 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO1_SCL_I2C2_CS_SPI         (1U)
/** \brief Used to configure TRIG_WDOG Pin Function.
 *         Valid only for GPIO2 and GPIO11 pins */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO2_GPIO11_TRIG_WDOG        (1U)
/** \brief Used to configure CLK32KOUT Pin Function.
 *         Valid only for GPIO3, GPIO4 and GPIO8 pins */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_GPIO4_GPIO8_CLK32KOUT   (1U)
/** \brief Used to configure SCLK_SPMI Pin Function.
 *         Valid only for GPIO5 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO5_SCLK_SPMI               (1U)
/** \brief Used to configure SDATA_SPMI Pin Function.
 *         Valid only for GPIO6 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO6_SDATA_SPMI              (1U)
/** \brief Used to configure NERR_MCU Pin Function.
 *         Valid only for GPIO7 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU                (1U)
/** \brief Used to configure PGOOD Pin Function.
 *         Valid only for GPIO9 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO9_PGOOD                   (1U)
/** \brief Used to configure SYNCCLKIN Pin Function.
 *         Valid only for GPIO10 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO10_SYNCCLKIN              (1U)
/** \brief Used to configure NRSTOUT_SOC Pin Function.
 *         Valid only for GPIO1 and GPIO11 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO1_GPIO11_NRSTOUT_SOC      (2U)
/** \brief Used to configure SDA_I2C2/SDO_SPI Pin Function.
 *         Valid only for GPIO2 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO2_SDA_I2C2_SDO_SPI        (2U)
/** \brief Used to configure NERR_SOC Pin Function.
 *         Valid only for GPIO3 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC                (2U)
/** \brief Used to configure SYNCCLKOUT Pin Function.
 *         Valid only for GPIO8 and GPIO10 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO8_GPIO10_SYNCCLKOUT       (2U)
/** \brief Used to configure DISABLE_WDOG Pin Function.
 *         Valid only for GPIO9 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO9_DISABLE_WDOG            (2U)
/** \brief Used to configure DISABLE_WDOG Pin Function.
 *         Valid only for GPIO8 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO8_DISABLE_WDOG            (3U)
/** \brief Used to configure SYNCCLKOUT Pin Function.
 *         Valid only for GPIO9 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO9_SYNCCLKOUT              (3U)
/** \brief Used to configure CLK32KOUT Pin Function.
 *         Valid only for GPIO10 pin */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO10_CLK32KOUT              (3U)
/** \brief Used to configure NSLEEP1 Pin Function.
 *         Valid for all GPIO Pins */
#define PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1                       (4U)
/** \brief Used to configure NSLEEP2 Pin Function.
 *         Valid for all GPIO Pins */
#define PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP2                       (5U)
/** \brief Used to configure WKUP1 Pin Function.
 *         Valid for all GPIO Pins except GPIO3 and GPIO4 pins */
#define PMIC_TPS6594X_GPIO_PINFUNC_WKUP1                         (6U)
/** \brief Used to configure LP_WKUP1 Pin Function.
 *         Valid only for GPIO3 and GPIO4 pins */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_GPIO4_LP_WKUP1          (6U)
/** \brief Used to configure WKUP2 Pin Function.
 *         Valid for all GPIO Pins except GPIO3 and GPIO4 pins */
#define PMIC_TPS6594X_GPIO_PINFUNC_WKUP2                         (7U)
/** \brief Used to configure LP_WKUP2 Pin Function.
 *         Valid only for GPIO3 and GPIO4 pins */
#define PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_GPIO4_LP_WKUP2          (7U)
/*  @} */

/**
 *  \anchor Pmic_GpioNPWRONPinFunc
 *  \name   PMIC GPIO NPWRON pin functions supported for TPS6594x Leo Device
 *
 *  @{
 */
/** \brief Enable Signal Function for NPWRON/ENABLE pin */
#define PMIC_TPS6594X_NPWRON_PINFUNC_ENABLE                 (0U)
/** \brief NPWRON Signal Function for NPWRON/ENABLE pin */
#define PMIC_TPS6594X_NPWRON_PINFUNC_NPWRON                 (1U)
/** \brief None Function for NPWRON/ENABLE pin */
#define PMIC_TPS6594X_NPWRON_PINFUNC_NONE                   (2U)
/*  @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/*!
 * \brief   API to get PMIC GPIO NPWRON/Enable pin value.
 *
 * Requirement: REQ_TAG(PDK-9124)
 * Design: did_pmic_gpio_cfg_readback
 * Architecture: aid_pmic_gpio_cfg
 *
 *          This function is used to read the signal level of the NPWRON/Enable
 *          pin.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   pPinValue       [OUT]   Pointer to store PMIC GPIO signal level
 *                                  High/Low.
 *                                  Valid values \ref Pmic_Gpio_SignalLvl
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioTps6594xNPwronPinGetValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                           uint8_t           *pPinValue);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_GPIO_TPS6594X_H_ */

/* @} */

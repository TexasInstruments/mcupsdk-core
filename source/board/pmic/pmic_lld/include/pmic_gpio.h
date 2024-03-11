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
 * @file   pmic_gpio.h
 *
 * @brief  This file contains macro definitions, structures and function
 *         prototypes for driver specific PMIC GPIO configuration
 */

#ifndef PMIC_GPIO_H_
#define PMIC_GPIO_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic.h"
#include "pmic_core.h"
#include "pmic_types.h"
#include "pmic_core_priv.h"

/**
 * @defgroup Pmic_GPIO PMIC General Purpose Input/Output
 * @{
 * @brief Contains definitions related to PMIC GPIO functionality.
 */

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @defgroup Pmic_GPIOMacros PMIC General Purpose Input/Output Macros
 * @{
 * @ingroup Pmic_GPIO
 * @brief Contains macros used in the GPIO module of PMIC driver.
 */

/**
 * @brief Macro to enable a GPIO pin.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_ENABLE (true)

/**
 * @brief Macro to disable a GPIO pin.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_DISABLE (false)

/**
 * @brief Macro defining the pin for PMIC_BB_GPO1.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPO1   (0x01U)

/**
 * @brief Macro defining the pin for PMIC_BB_GPO2.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPO2   (0x02U)

/**
 * @brief Macro defining the pin for PMIC_BB_GPO3.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPO3   (0x03U)

/**
 * @brief Macro defining the pin for PMIC_BB_GPO4.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPO4   (0x04U)

/**
 * @brief Macro defining the pin for PMIC_GPI1.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPI1   (0x05U)

/**
 * @brief Macro defining the pin for PMIC_GPI4.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPI4   (0x06U)

/**
 * @brief Macro indicating the validity of GPIO pull configuration.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_CFG_PULL_VALID            (0x02U)

/**
 * @brief Macro indicating the validity of GPIO deglitch configuration.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_CFG_DEGLITCH_VALID        (0x03U))

/**
 * @brief Macro representing a low level for GPIO configuration.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_LOW_LEVEL                 (0U)

/**
 * @brief Macro representing a high level for GPIO configuration.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_HIGH_LEVEL                (1U)

/**
 * @brief Macro representing GPIO as not enabled.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_N_EN                      (2U)

/**
 * @brief Macro representing GPIO as an interrupt pin.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_nINT                      (3U)

/**
 * @brief Macro representing GPIO as a watchdog input.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_WD_IN                     (1U)

/**
 * @brief Macro representing GPIO as a chip-over-temperature signal.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_COS_N                     (2U)

/**
 * @brief Macro representing GPIO as a power good signal.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_PGOOD                     (3U)

/**
 * @brief Macro representing GPIO as comparator 1 input.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_COMP1_IN                  (0U)

/**
 * @brief Macro representing GPIO as comparator 1 output.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_COMP1_OUT                 (3U)

/**
 * @brief Macro representing GPIO as comparator 2 output.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_COMP2_OUT                 (4U)

/**
 * @brief Macro representing GPIO as ESM input.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_ESM_IN                    (0U)

/**
 * @brief Macro representing GPIO as enable output 2.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_EN_OUT2                   (4U)

/**
 * @brief Macro representing GPIO as safe output 2.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_SAFE_OUT2                 (3U)

/**
 * @brief Macro representing GPIO as a high impedance output - 1.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_N_EN_HIGH_Z_1             (4U)

/**
 * @brief Macro representing GPIO as a high impedance output - 2.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_N_EN_HIGH_Z_2             (5U)

/**
 * @brief Macro representing GPIO as a high impedance output - 3.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_N_EN_HIGH_Z_3             (6U)

/**
 * @brief Macro representing GPIO as a high impedance output - 4.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_N_EN_HIGH_Z_4             (5U)

/**
 * @brief Macro representing GPIO as a high impedance output - 5.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_N_EN_HIGH_Z_5             (6U)

/**
 * @brief Macro representing GPIO as a high impedance output - 6.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_N_EN_HIGH_Z_6             (4U)

/**
 * @brief Macro representing GPIO as a high impedance output - 7.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_N_EN_HIGH_Z_7             (5U)

/**
 * @brief Macro representing GPIO as a high impedance output - 8.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_N_EN_HIGH_Z_8             (6U)

/**
 * @brief Macro representing GPIO as a high impedance output - 9.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_N_EN_HIGH_Z_9             (5U)

/**
 * @brief Macro representing GPIO as a high impedance output - 10.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_N_EN_HIGH_Z_10            (6U)

/**
 * @brief Macro representing GPIO as pulled up to VDDIO.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_PULL_UP_VDDIO             (0U)

/**
 * @brief Macro representing GPIO as pulled up to LDO input.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_PULL_UP_LDO_IN            (1U)

/**
 * @brief Macro representing GPIO as internally pulled up.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_INTL_PULL_UP              (2U)

/**
 * @brief Macro representing GPIO as read-only counter.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPIO_RO_CNTR                   (7U)

/**
 * @brief Macro representing GPIO readback de-glitch time of 1us.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPO_RISING_DGL_TIME_1US        (0U)

/**
 * @brief Macro representing GPIO readback de-glitch time of 2us.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPO_RISING_DGL_TIME_2US        (1U)

/**
 * @brief Macro representing GPIO readback de-glitch time of 4us.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPO_RISING_DGL_TIME_4US        (2U)

/**
 * @brief Macro representing GPIO readback de-glitch time of 8us.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPO_RISING_DGL_TIME_8US        (3U)

/**
 * @brief Macro representing GPIO readback de-glitch time of 250ns.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPO_FALLING_DGL_TIME_250NS     (0U)

/**
 * @brief Macro representing GPIO readback de-glitch time of 500ns.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPO_FALLING_DGL_TIME_500NS     (1U)

/**
 * @brief Macro representing GPIO readback de-glitch time of 1us.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPO_FALLING_DGL_TIME_1US       (2U)

/**
 * @brief Macro representing GPIO readback de-glitch time of 2us.
 *
 * @ingroup Pmic_GPIOMacros
 */
#define PMIC_GPO_FALLING_DGL_TIME_2US       (3U)


/**
 * @}
 */
/* End of Pmic_GPIOMacros */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @defgroup Pmic_GPIOStructures PMIC GPIO Structures
 * @{
 * @ingroup Pmic_GPIO
 * @brief Contains structures used in the GPIO module of PMIC driver.
 */

/**
 * @brief GPIO pin configuration for input/output selection.
 *
 * This structure defines the configuration parameters for GPIO pins, including register addresses
 * and bit positions for input and output configuration.
 *
 * @param regAddr The register address for GPIO pin configuration.
 * @param outRegAddr The register address for GPIO output configuration.
 * @param inRegAddr The register address for GPIO input configuration.
 * @param inRegBitPos The bit position in the input register.
 * @param outRegBitPos The bit position in the output register.
 *
 * @ingroup Pmic_GPIOStructures
 */
typedef struct Pmic_GpioInOutCfg_s {
    uint8_t regAddr;
    uint8_t outRegAddr;
    uint8_t inRegAddr;
    uint8_t inRegBitPos;
    uint8_t outRegBitPos;
}
Pmic_GpioInOutCfg_t;

/**
 * @brief GPIO pin configuration structure.
 *
 * This structure defines the configuration parameters for GPIO pins, including pin direction,
 * output signal type, pull control, deglitch enable, pin function, and polarity, as well as
 * configurations for specific GPIO pins.
 *
 * @param validParams Valid parameters indicator.
 * @param pinDir Pin direction configuration.
 * @param outputSignalType Output signal type configuration.
 * @param pullCtrl Pull control configuration.
 * @param deglitchEnable Deglitch enable configuration.
 * @param pinFunc Pin function configuration.
 * @param pinPolarity Pin polarity configuration.
 * @param gpo1Cfg Configuration for PMIC_BB_GPO1.
 * @param gpo2Cfg Configuration for PMIC_BB_GPO2.
 * @param gpo3Cfg Configuration for PMIC_BB_GPO3.
 * @param gpo4Cfg Configuration for PMIC_BB_GPO4.
 * @param gpi1Cfg Configuration for GPI1.
 * @param gpi4Cfg Configuration for GPI4.
 *
 * @ingroup Pmic_GPIOStructures
 */
typedef struct Pmic_GpioCfg_s {
    uint8_t validParams;
    uint8_t pinDir;
    uint8_t outputSignalType;
    uint8_t pullCtrl;
    uint8_t deglitchEnable;
    uint8_t pinFunc;
    uint8_t pinPolarity;
    uint8_t gpo1Cfg;
    uint8_t gpo2Cfg;
    uint8_t gpo3Cfg;
    uint8_t gpo4Cfg;
    uint8_t gpi1Cfg;
    uint8_t gpi4Cfg;
}
Pmic_GpioCfg_t;

/**
 * @brief GPIO pin read-back deglitch configuration structure.
 *
 * This structure defines the configuration parameters for GPIO pin read-back deglitching,
 * including valid parameters indicators and configurations for falling and rising edge
 * deglitch times for multiple GPIO pins.
 *
 * @param validParams Valid parameters indicator.
 * @param gpo1FDglConfig GPO1 falling edge deglitch configuration.
 * @param gpo1RDglConfig GPO1 rising edge deglitch configuration.
 * @param gpo2FDglConfig GPO2 falling edge deglitch configuration.
 * @param gpo2RDglConfig GPO2 rising edge deglitch configuration.
 * @param gpo3FDglConfig GPO3 falling edge deglitch configuration.
 * @param gpo3RDglConfig GPO3 rising edge deglitch configuration.
 * @param gpo4FDglConfig GPO4 falling edge deglitch configuration.
 * @param gpo4RDglConfig GPO4 rising edge deglitch configuration.
 * @param gpo1FDglData GPO1 falling edge deglitch data.
 * @param gpo1RDglData GPO1 rising edge deglitch data.
 * @param gpo2FDglData GPO2 falling edge deglitch data.
 * @param gpo2RDglData GPO2 rising edge deglitch data.
 * @param gpo3FDglData GPO3 falling edge deglitch data.
 * @param gpo3RDglData GPO3 rising edge deglitch data.
 * @param gpo4FDglData GPO4 falling edge deglitch data.
 * @param gpo4RDglData GPO4 rising edge deglitch data.
 *
 * @ingroup Pmic_GPIOStructures
 */
typedef struct Pmic_GpioRdbkDglCfg_s {
    uint8_t validParams;
    uint8_t gpo1FDglConfig;
    uint8_t gpo1RDglConfig;
    uint8_t gpo2FDglConfig;
    uint8_t gpo2RDglConfig;
    uint8_t gpo3FDglConfig;
    uint8_t gpo3RDglConfig;
    uint8_t gpo4FDglConfig;
    uint8_t gpo4RDglConfig;
    uint8_t gpo1FDglData;
    uint8_t gpo1RDglData;
    uint8_t gpo2FDglData;
    uint8_t gpo2RDglData;
    uint8_t gpo3FDglData;
    uint8_t gpo3RDglData;
    uint8_t gpo4FDglData;
    uint8_t gpo4RDglData;
}
Pmic_GpioRdbkDglCfg_t;

/**
 * @}
 */
/* End of Pmic_GPIOStructures */

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @defgroup Pmic_GPIOFunctions PMIC General Purpose Input/Output Functions
 * @{
 * @ingroup Pmic_GPIO
 * @brief Contains functions used in the GPIO module of PMIC driver.
 */

/**
 * @brief Retrieves the GPIO pin input/output configuration.
 *
 * This function is used to retrieve the GPIO pin input/output configuration
 * for the TPS653860 PMIC. It assigns the address of the GPIO pin configuration
 * structure to the provided pointer.
 *
 * @param pGpioInOutCfg Pointer to a pointer where the GPIO pin input/output configuration
 *                      will be stored.
 *
 * @ingroup Pmic_GPIOFunctions
 */
void pmic_get_tps653860_gpioInOutCfg(Pmic_GpioInOutCfg_t **pGpioInOutCfg);

/**
 * @brief Retrieves the GPIO pin input/output configuration.
 *
 * This function is used to retrieve the GPIO pin input/output configuration
 * from the PMIC specified by the provided handle. It calls the internal function
 * `pmic_get_tps653860_gpioInOutCfg` to obtain the configuration and stores it
 * in the provided `Pmic_GpioInOutCfg_t` structure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pGpioInOutCfg Pointer to the structure where the GPIO pin input/output
 *                      configuration will be stored.
 *
 * @ingroup Pmic_GPIOFunctions
 */
void Pmic_get_gpioInOutCfg(Pmic_GpioInOutCfg_t *pGpioInOutCfg);

/**
 * @brief Validates the GPIO pin number.
 *
 * This function validates the GPIO pin number to ensure it falls within the
 * valid range specified by the macros PMIC_TPS653860XX_GPIO_PIN_MIN and
 * PMIC_TPS653860XX_GPIO_PIN_MAX. If the pin number is outside this range,
 * it returns an error code PMIC_ST_ERR_INV_PARAM, indicating an invalid parameter.
 *
 * @param pin The GPIO pin number to be validated.
 *
 * @return Returns PMIC_ST_SUCCESS if the pin number is valid, otherwise
 * returns PMIC_ST_ERR_INV_PARAM.
 *
 * @ingroup Pmic_GPIOFunctions
 */
static int32_t Pmic_gpioValidatePin(const uint8_t pin);

/**
 * @brief Validates the parameters related to GPIO functionality.
 *
 * This function checks whether the PMIC handle provided is valid and whether
 * the GPIO functionality is enabled in the PMIC subsystem. If the handle is
 * valid and GPIO functionality is enabled, it returns PMIC_ST_SUCCESS. Otherwise,
 * it returns an appropriate error code, such as PMIC_ST_ERR_INV_HANDLE if the
 * handle is invalid or PMIC_ST_ERR_INV_DEVICE if GPIO functionality is not enabled.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 *
 * @return Returns PMIC_ST_SUCCESS if the parameters are valid, otherwise returns
 * an appropriate error code.
 *
 * @ingroup Pmic_GPIOFunctions
 */
static int32_t Pmic_gpioValidateParams(const Pmic_CoreHandle_t *pPmicCoreHandle);

/**
 * @brief Checks the validity of GPIO parameters.
 *
 * This function first validates the parameters related to GPIO functionality
 * by calling the `Pmic_gpioValidateParams` and `Pmic_gpioValidatePin` functions.
 * It verifies whether the PMIC handle provided is valid and whether the GPIO pin
 * number is within the permissible range. If both validations pass, it returns
 * PMIC_ST_SUCCESS; otherwise, it returns an appropriate error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin number to be checked.
 *
 * @return Returns PMIC_ST_SUCCESS if the parameters are valid, otherwise returns
 * an appropriate error code.
 *
 * @ingroup Pmic_GPIOFunctions
 */
static int32_t Pmic_gpioParamCheck(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const uint8_t pin);

/**
 * @brief Selects the register address for configuring a GPIO pin.
 *
 * This function maps the GPIO pin number to the corresponding register address
 * for configuring the pin. It takes the GPIO pin number as input and assigns
 * the appropriate register address to the provided pointer `pRegAddr`. The register
 * address depends on the GPIO pin specified, which can be PMIC_BB_GPO1, PMIC_BB_GPO2,
 * PMIC_BB_GPO3, or PMIC_BB_GPO4. If the GPIO pin provided is not recognized, it
 * defaults to PMIC_GPO_1_CONF_REGADDR.
 *
 * @param gpo GPIO pin number to select the register address.
 * @param pRegAddr Pointer to store the selected register address.
 *
 * @ingroup Pmic_GPIOFunctions
 */
void Pmic_gpioSelectRegister(uint8_t gpo, uint8_t *pRegAddr);

/**
 * @brief Sets the pull control configuration for a GPIO pin.
 *
 * This function configures the pull control for a specified GPIO pin based on
 * the provided GPIO configuration (`gpioCfg`). It sets the pull-up configuration
 * to either VDDIO or LDO_IN, or disables the pull control entirely to set the pin
 * as high impedance. The GPIO pin number is specified by `pin`. The function reads
 * the GPO_CFG2 register to determine the current configuration and updates it accordingly.
 * It performs this operation within a critical section to ensure atomicity. If the
 * pull control configuration provided is invalid, the function returns an error.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin number to configure the pull control.
 * @param gpioCfg GPIO configuration structure containing the pull control information.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
static int32_t Pmic_gpioSetPullCtrl(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     const Pmic_GpioCfg_t gpioCfg);

/**
 * @brief Gets the pull control configuration for a GPIO pin.
 *
 * This function retrieves the pull control configuration for the specified GPIO pin (`pin`)
 * and updates the provided GPIO configuration structure (`pGpioCfg`) with the retrieved information.
 * It reads the GPO_CFG2 register to obtain the current pull control configuration for the GPIO pin.
 * The function then translates the register data into the corresponding pull control value and stores
 * it in the `pGpioCfg->pullCtrl` field. The operation is performed within a critical section to ensure
 * atomicity. If the read operation fails or the retrieved data does not match any valid pull control
 * configuration, the function returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin number to retrieve the pull control configuration.
 * @param pGpioCfg Pointer to the GPIO configuration structure to store the pull control information.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpioGetPullCtrl(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_GpioCfg_t *pGpioCfg);

/**
 * @brief Sets the deglitch, output signal type, and pull control configurations for a GPIO pin.
 *
 * This function sets the deglitch time, output signal type, and pull control configurations
 * for the specified GPIO pin (`pin`). Depending on the pin number, it invokes the corresponding
 * function to configure the deglitch time for either GPO1/GPO2 or GPO3/GPO4. The deglitch time
 * configuration is provided through the `GpioRdbkDglCfg` parameter. The function then returns
 * the status of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin number for which to set the configurations.
 * @param GpioRdbkDglCfg Pointer to the GPIO read-back deglitch configuration structure.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
static int32_t Pmic_gpioSetDeglitchOutsigtypePulCtrlCfg(
    Pmic_CoreHandle_t *pPmicCoreHandle,
    const uint8_t pin,
    Pmic_GpioRdbkDglCfg_t *GpioRdbkDglCfg);

/**
 * @brief Sets the configuration for a GPIO pin.
 *
 * This function sets the configuration for the specified GPIO pin (`pin`). The configuration
 * includes parameters such as pin direction, output signal type, pull control, deglitch
 * enable, pin function, and pin polarity. The configuration is provided through the `gpioCfg`
 * parameter. The function determines the register address based on the pin number and then reads
 * the corresponding register to modify the configuration bits. It then sends the modified
 * configuration back to the register. This function also performs parameter validation to
 * ensure the correctness of the input parameters. It returns the status of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin number for which to set the configuration.
 * @param gpioCfg Configuration structure containing GPIO parameters.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpioSetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const uint8_t pin,
                                  const Pmic_GpioCfg_t gpioCfg);

/**
 * @brief Gets the configuration of General Purpose Output (GPO) pins.
 *
 * This function retrieves the configuration of GPO pins specified by the register address
 * (`regAddr`) and populates the provided GPIO configuration structure (`pGpioCfg`) with
 * the extracted information. It starts a critical section, reads the GPO configuration
 * register, and then stops the critical section. Depending on the register address,
 * the function extracts the configuration bits for PMIC_BB_GPO1, PMIC_BB_GPO2, PMIC_BB_GPO3,
 * and PMIC_BB_GPO4 pins and stores them in the `pGpioCfg` structure. The function returns
 * the status of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param regAddr Register address for the GPO configuration.
 * @param pGpioCfg Pointer to the structure to store GPIO configuration.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpioGetGPOConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                              uint8_t regAdr,
                              Pmic_GpioCfg_t *pGpioCfg);

/**
 * @brief Gets the configuration of a specific General Purpose Output (GPO) pin.
 *
 * This function retrieves the configuration of the specified GPO pin (`pin`) based on
 * its register address and populates the provided GPIO configuration structure
 * (`pGpioCfg`) with the extracted information. It starts a critical section, reads the
 * GPO configuration register associated with the pin, and then stops the critical section.
 * The function returns the status of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin Pin number representing the GPO pin to get the configuration for.
 * @param pGpioCfg Pointer to the structure to store GPIO configuration.
 * @param GpioRdbkDglCfg Pointer to the structure to store GPIO read-back deglitch configuration.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpioGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const uint8_t pin,
                                  Pmic_GpioCfg_t *pGpioCfg);

/**
 * @brief Sets the value of a specific GPIO pin.
 *
 * This function sets the value of the specified GPIO pin (`pinValue`) based on its
 * configuration provided in the GPIO input/output configuration structure
 * (`pGpioInOutCfg`). It starts a critical section, reads the current register data,
 * modifies the GPIO output value, and then writes the updated value back to the register.
 * The function returns the status of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pGpioInOutCfg Pointer to the structure containing GPIO input/output configuration.
 * @param pinValue Value to be set for the GPIO pin.
 * @param index Index indicating the GPIO pin configuration in the array.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
static int32_t Pmic_gpioSetPinValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    const Pmic_GpioInOutCfg_t *pGpioInOutCfg,
                                    const uint8_t pinValue);

/**
 * @brief Sets the value of a specified GPIO pin.
 *
 * This function sets the value (`pinValue`) for the specified GPIO pin (`pin`) based
 * on its configuration. It first checks the validity of the GPIO parameters, then
 * retrieves the GPIO pin configuration, and finally sets the pin value using the
 * `Pmic_gpioSetPinValue` function. The pin value should be within the valid range
 * (0 for low level, 1 for high level). The function returns the status of the
 * operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin The GPIO pin number to set the value for.
 * @param pinValue The value to set for the GPIO pin (0 for low, 1 for high).
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpioSetValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                          const uint8_t pin,
                          const uint8_t pinValue);

/**
 * @brief Gets the value of a specified GPIO pin.
 *
 * This function retrieves the current value of the specified GPIO pin (`pin`) by
 * reading its input register. It first checks the validity of the GPIO parameters
 * and the existence of the output buffer (`pPinValue`). Then, it retrieves the GPIO
 * pin configuration and reads the pin value from the input register. The pin value
 * is then stored in the provided output buffer. The function returns the status of
 * the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin The GPIO pin number to get the value for.
 * @param pPinValue Pointer to store the retrieved value of the GPIO pin
 *                  (0 for low, 1 for high).
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpioGetValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                          const uint8_t pin,
                          uint8_t *pPinValue);

/**
 * @brief Sets the configuration of a specified GPIO pin used as input.
 *
 * This function sets the configuration of the specified GPIO pin (`pin`) when used
 * as an input. It first checks the validity of the GPIO parameters. Then, it reads
 * the current configuration from the GPI configuration register, modifies the
 * configuration bits based on the provided `gpioCfg`, and writes back the updated
 * configuration to the register. The function returns the status of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin The GPIO pin number to set the configuration for.
 * @param gpioCfg Configuration settings for the GPIO pin.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpiSetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t pin,
                                 const Pmic_GpioCfg_t gpioCfg);

/**
 * @brief Retrieves the configuration of a specified GPI pin.
 *
 * This function reads the configuration of the specified GPI pin from the
 * GPI configuration register located at the provided register address (`regAddr`).
 * It starts a critical section before accessing the register to ensure atomicity.
 * Upon successful read, it populates the `pGpioCfg` structure with the GPI
 * configurations for GPI1 and GPI4. The function returns the status of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param regAddr The register address of the GPI configuration register.
 * @param pGpioCfg Pointer to the structure to store the GPIO configurations.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpioGetGPIConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t regAdr, Pmic_GpioCfg_t * pGpioCfg);

/**
 * @brief Retrieves the configuration of a specified GPI pin.
 *
 * This function retrieves the configuration of the specified GPI pin (GPI1 or GPI4)
 * by reading the GPI configuration register located at the predefined register address.
 * It starts a critical section before accessing the register to ensure atomicity.
 * Upon successful read, it populates the provided `pGpioCfg` structure with the GPI
 * configurations for GPI1 or GPI4 based on the `pin` parameter. The function returns
 * the status of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin The GPI pin number to retrieve the configuration for.
 * @param pGpioCfg Pointer to the structure to store the GPIO configurations.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpiGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t pin,
                                 Pmic_GpioCfg_t *pGpioCfg);

/**
 * @brief Retrieves the deglitch time configuration for GPO1 and GPO2.
 *
 * This function reads the deglitch time configuration for GPO1 and GPO2 from the
 * corresponding register and populates the provided `GpioRdbkDglCfg` structure with
 * the deglitch data. It starts a critical section before accessing the register to
 * ensure atomicity. Upon successful read, it stores the deglitch time data for both
 * rising and falling edges of GPO1 and GPO2 in the provided structure. The function
 * returns the status of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param GpioRdbkDglCfg Pointer to the structure to store the deglitch time data.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpo12GetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_GpioRdbkDglCfg_t *GpioRdbkDglCfg);

/**
 * @brief Sets the deglitch time configuration for GPO1 and GPO2.
 *
 * This function sets the deglitch time configuration for GPO1 and GPO2 based on the
 * provided `GpioRdbkDglCfg` structure. It starts a critical section before accessing
 * the register to ensure atomicity. If the configuration for deglitching is enabled
 * for GPO1 or GPO2, the corresponding deglitch time values are written to the
 * register. The function returns the status of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param GpioRdbkDglCfg Pointer to the structure containing the deglitch time
 *                       configuration for GPO1 and GPO2.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpo12SetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_GpioRdbkDglCfg_t *GpioRdbkDglCfg);

/**
 * @brief Sets the deglitch time configuration for GPO3 and GPO4.
 *
 * This function sets the deglitch time configuration for GPO3 and GPO4 based on the
 * provided `GpioRdbkDglCfg` structure. It starts a critical section before accessing
 * the register to ensure atomicity. If the configuration for deglitching is enabled
 * for GPO3 or GPO4, the corresponding deglitch time values are written to the
 * register. The function returns the status of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param GpioRdbkDglCfg Pointer to the structure containing the deglitch time
 *                       configuration for GPO3 and GPO4.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpo34SetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_GpioRdbkDglCfg_t *GpioRdbkDglCfg);

/**
 * @brief Retrieves the deglitch time configuration for GPO3 and GPO4.
 *
 * This function retrieves the deglitch time configuration for GPO3 and GPO4 from
 * the corresponding register. It starts a critical section before accessing
 * the register to ensure atomicity. The retrieved deglitch time values are stored
 * in the provided `GpioRdbkDglCfg` structure. The function returns the status
 * of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param GpioRdbkDglCfg Pointer to the structure to store the deglitch time
 *                       configuration for GPO3 and GPO4.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - Error code if the operation fails.
 *
 * @ingroup Pmic_GPIOFunctions
 */
int32_t Pmic_gpo34GetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_GpioRdbkDglCfg_t *GpioRdbkDglCfg);

/**
 * @}
 */
/* End of Pmic_GPIOFunctions */

/**
 * @}
 */
/* End of Pmic_GPIO */

#endif /* PMIC_GPIO_H_ */

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
 *   @file    pmic_gpio.c
 *
 *   @brief   This file contains the default API's for PMIC gpio
 *            configuration
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_gpio.h"
#include "pmic_gpio_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/**
 * @brief Array of structures to configure GPIO input and output pins for TPS65386.
 * @ingroup Pmic_GPIOPrivStructures
 */
 Pmic_GpioInOutCfg_t gTps65386_gpioInOutCfg[] = {
    {
        PMIC_GPI_1_CONF_REGADDR,
        PMIC_GPI_1_GPI_1_SHIFT,
        PMIC_GPI_1_GPI_1_MASK,
    },
    {
        PMIC_GPI_1_CONF_REGADDR,
        PMIC_GPI_4_GPI_2_SHIFT,
        PMIC_GPI_4_GPI_2_MASK,
    },
    {
        PMIC_GPO_1_CONF_REGADDR,
        PMIC_GPO_1_GPO_1_SHIFT,
        PMIC_GPO_1_GPO_2_MASK,
    },
    {
        PMIC_GPO_1_CONF_REGADDR,
        PMIC_GPO_2_GPI_3_SHIFT,
        PMIC_GPO_2_GPO_3_MASK,
    },
    {
        PMIC_GPO_2_CONF_REGADDR,
        PMIC_GPO_3_GPO_1_SHIFT,
        PMIC_GPO_3_GPO_2_MASK,
    },
    {
        PMIC_GPO_2_CONF_REGADDR,
        PMIC_GPO_4_GPO_3_SHIFT,
        PMIC_GPO_4_GPO_3_MASK,
    },
};
/**
 * @brief  Get the pointer to the GPIO input/output configuration.
 * This function retrieves the pointer to the GPIO input/output configuration.
 *
 * @param  pGpioInOutCfg Pointer to the GPIO input/output configuration pointer.
 * @return void
 */
void pmic_get_tps653860_gpioInOutCfg(Pmic_GpioInOutCfg_t ** pGpioInOutCfg) {
    * pGpioInOutCfg = gTps65386_gpioInOutCfg;
}

/**
 * @brief Get the GPIO input/output configuration.
 * This function gets the GPIO input/output configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pGpioInOutCfg Pointer to store the GPIO input/output configuration.
 * @return void
 */
void Pmic_get_gpioInOutCfg(Pmic_GpioInOutCfg_t * pGpioInOutCfg) {
    Pmic_GpioInOutCfg_t * temppGpioInOutCfg = pGpioInOutCfg;
    pmic_get_tps653860_gpioInOutCfg(&temppGpioInOutCfg);
}

/**
 * @brief Validate the GPIO pin.
 * This function validates the GPIO pin.
 *
 * @param pin GPIO pin to be validated.
 *
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the pin is valid;
 * otherwise, returns an error code.
 */
static int32_t Pmic_gpioValidatePin(const uint8_t pin) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (((pin >= PMIC_TPS653860XX_GPIO_PIN_MIN) &&
            (pin <= PMIC_TPS653860XX_GPIO_PIN_MAX))) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    return pmicStatus;
}

/**
 * @brief Validate the GPIO parameters.
 * This function validates the GPIO parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the parameters are valid;
 * otherwise, returns an error code.
 */
static int32_t
Pmic_gpioValidateParams(const Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_ERR_INV_HANDLE;

    if (NULL != pPmicCoreHandle) {
        if (true == pPmicCoreHandle -> pPmic_SubSysInfo -> gpioEnable) {
            pmicStatus = PMIC_ST_SUCCESS;
        } else {
            pmicStatus = PMIC_ST_ERR_INV_DEVICE;
        }
    }

    return pmicStatus;
}

/**
 * @brief Check and validate the GPIO parameters.
 * This function checks and validates the GPIO parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin to be checked and validated.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the parameters are valid;
 * otherwise, returns an error code.
 */
static int32_t Pmic_gpioParamCheck(const Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t pin) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    pmicStatus = Pmic_gpioValidateParams(pPmicCoreHandle);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_gpioValidatePin(pin);
    }

    return pmicStatus;
}

/**
 * @brief Select the register for the GPIO.
 * This function selects the register for the GPIO based on the GPO number.
 *
 * @param gpo GPO number.
 * @param pRegAddr Pointer to store the register address.
 * @return void
 */
void Pmic_gpioSelectRegister(uint8_t gpo, uint8_t * pRegAddr) {

    switch (gpo) {
    case (PMIC_BB_GPO1):
        *
        pRegAddr = PMIC_GPO_1_CONF_REGADDR;
        break;
    case (PMIC_BB_GPO2):
        *
        pRegAddr = PMIC_GPO_1_CONF_REGADDR;
        break;
    case (PMIC_BB_GPO3):
        *
        pRegAddr = PMIC_GPO_2_CONF_REGADDR;
        break;
    case (PMIC_BB_GPO4):
        *
        pRegAddr = PMIC_GPO_2_CONF_REGADDR;
        break;
    default:
        *
        pRegAddr = PMIC_GPO_1_CONF_REGADDR;
        break;
    }
}

/**
 * @brief Set the pull control for the GPIO.
 * This function sets the pull control for the GPIO pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin to set the pull control.
 * @param gpioCfg GPIO configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_gpioSetPullCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_GpioCfg_t gpioCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAdr = 0x7DU; /* Register address for GPO_CFG2 */
    Pmic_GpioInOutCfg_t * pGpioInOutCfg = NULL;

    if (gpioCfg.pullCtrl > PMIC_GPIO_PULL_UP_TO_LDO) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Get PMIC gpio configuration */
        Pmic_get_gpioInOutCfg( pGpioInOutCfg);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading GPO_CFG2 register */
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAdr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (PMIC_GPIO_PULL_DISABLED == gpioCfg.pullCtrl) {
                /* Set as high impedance (internal pull-up not enabled) */
                Pmic_setBitField( & regData, PMIC_GPO_CFG2_EN_OUT_PU_CFG_SHIFT,
                                  (uint8_t)PMIC_GPO_CFG2_EN_OUT_PU_CFG_MASK,
                    PMIC_GPO_HIGH_IMPEDANCE);
            }

            if ((PMIC_GPIO_PULL_UP == gpioCfg.pullCtrl)) {
                /* Set pull-up to VDDIO or LDO_IN */
                if (gpioCfg.pullCtrl == PMIC_GPIO_PULL_UP_TO_LDO) {
                    Pmic_setBitField( & regData, PMIC_GPO_CFG2_EN_OUT_PU_CFG_SHIFT,
                                      (uint8_t)PMIC_GPO_CFG2_EN_OUT_PU_CFG_MASK,
                        PMIC_GPO_PULL_UP_LDO);
                } else {
                    Pmic_setBitField( & regData, PMIC_GPO_CFG2_EN_OUT_PU_CFG_SHIFT,
                                      (uint8_t)PMIC_GPO_CFG2_EN_OUT_PU_CFG_MASK,
                        PMIC_GPO_PULL_UP_VDDIO);
                }
            }

            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAdr, regData);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Get the pull control for the GPIO.
 * This function gets the pull control for the GPIO pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin to get the pull control.
 * @param pGpioCfg Pointer to store the GPIO configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_gpioGetPullCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
                             Pmic_GpioCfg_t * pGpioCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAdr = 0x7DU; /* Register address for GPO_CFG2 */
    Pmic_GpioInOutCfg_t * pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    Pmic_get_gpioInOutCfg( pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the GPO_CFG2 register */
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAdr, & regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Reading gpio pull control */
        if (Pmic_getBitField(regData, PMIC_GPO_CFG2_EN_OUT_PU_CFG_SHIFT,
                             (uint8_t)PMIC_GPO_CFG2_EN_OUT_PU_CFG_MASK) ==
            PMIC_GPO_HIGH_IMPEDANCE) {
            pGpioCfg -> pullCtrl = PMIC_GPIO_PULL_DISABLED;
        } else if (Pmic_getBitField(regData, PMIC_GPO_CFG2_EN_OUT_PU_CFG_SHIFT,
                                    (uint8_t)PMIC_GPO_CFG2_EN_OUT_PU_CFG_MASK) ==
            PMIC_GPO_PULL_UP_VDDIO) {
            pGpioCfg -> pullCtrl = PMIC_GPIO_PULL_UP;
        } else if (Pmic_getBitField(regData, PMIC_GPO_CFG2_EN_OUT_PU_CFG_SHIFT,
                                    (uint8_t)PMIC_GPO_CFG2_EN_OUT_PU_CFG_MASK) ==
            PMIC_GPO_PULL_UP_LDO) {
            pGpioCfg -> pullCtrl = PMIC_GPIO_PULL_UP_TO_LDO;
        } else {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    return pmicStatus;
}

/**
 * @brief Set the deglitch time, output signal type, and pull control for the
 * GPIO. This function sets the deglitch time, output signal type, and pull
 * control for the GPIO.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin to set the configuration.
 * @param GpioRdbkDglCfg Pointer to the GPIO debounce and deglitch
 * configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_gpioSetDeglitchOutsigtypePulCtrlCfg(
    Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t pin,
        Pmic_GpioRdbkDglCfg_t * GpioRdbkDglCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* setting deglitch time */
    if ((pin == PMIC_GPO1) || (pin == PMIC_GPO2)) {
        pmicStatus = Pmic_gpo12SetDeglitchTime(pPmicCoreHandle, GpioRdbkDglCfg);
    } else {
        pmicStatus = Pmic_gpo34SetDeglitchTime(pPmicCoreHandle, GpioRdbkDglCfg);
    }

    return pmicStatus;
}

/**
 * @brief Set the configuration for the GPIO.
 * This function sets the configuration for the GPIO pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin to set the configuration.
 * @param gpioCfg GPIO configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_gpioSetConfiguration(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t pin,
        const Pmic_GpioCfg_t gpioCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAdr = 0U;
    uint8_t regData = 0U;

    switch (pin) {
    case PMIC_GPO1:
        regAdr = PMIC_GPO_CFG1_REG_ADDR;
        break;
    case PMIC_GPO2:
        regAdr = PMIC_GPO_CFG1_REG_ADDR;
        break;
    case PMIC_GPO3:
        regAdr = PMIC_GPO_CFG2_REG_ADDR;
        break;
    case PMIC_GPO4:
        regAdr = PMIC_GPO_CFG2_REG_ADDR;
        break;
    default:
        pmicStatus = PMIC_ST_ERR_INV_PARAM; /* Invalid pin */
        break;
    }

    pmicStatus = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading the GPO_CFG register */
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAdr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            /* Setting configurations for PMIC_BB_GPO1, PMIC_BB_GPO2,
             * PMIC_BB_GPO3, PMIC_BB_GPO4 based on pin
             */
            switch (pin) {
            case PMIC_GPO1:
                /* Modify GPO1_CFG bits in GPO_CFG1*/
                Pmic_setBitField( & regData, PMIC_GPO_CFG1_GPO1_CFG_SHIFT,
                    (uint8_t)PMIC_GPO_CFG1_GPO1_CFG_MASK, gpioCfg.gpo1Cfg);
                break;
            case PMIC_GPO2:
                /* Modify GPO2_CFG bits in GPO_CFG1*/
                Pmic_setBitField( & regData, PMIC_GPO_CFG1_GPO2_CFG_SHIFT,
                                  (uint8_t)PMIC_GPO_CFG1_GPO2_CFG_MASK, gpioCfg.gpo2Cfg);
                break;
            case PMIC_GPO3:
                /* Modify GPO3_CFG bits in GPO_CFG2*/
                Pmic_setBitField( & regData, PMIC_GPO_CFG2_GPO3_CFG_SHIFT,
                                  (uint8_t)PMIC_GPO_CFG2_GPO3_CFG_MASK, gpioCfg.gpo3Cfg);
                Pmic_setBitField( & regData, PMIC_GPO_CFG2_GPO_EN_SHIFT,
                                  (uint8_t)PMIC_GPO_CFG2_GPO_EN_MASK, gpioCfg.pinDir);
                break;
            case PMIC_GPO4:
                /* Modify GPO4_CFG bits in GPO_CFG2*/
                Pmic_setBitField( & regData, PMIC_GPO_CFG2_GPO_EN_SHIFT,
                                  (uint8_t)PMIC_GPO_CFG2_GPO_EN_MASK, gpioCfg.pinDir);
                Pmic_setBitField( & regData, PMIC_GPO_CFG2_GPO4_CFG_SHIFT,
                                  (uint8_t)PMIC_GPO_CFG2_GPO4_CFG_MASK, gpioCfg.gpo4Cfg);
                break;
            default:
                /* Invalid Pin */
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }

            if (PMIC_ST_SUCCESS == pmicStatus) {
                /* Sending modified configuration back to the register */
                pmicStatus =
                    Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAdr, regData);
            }
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Get the GPO configuration for the GPIO.
 * This function gets the GPO configuration for the GPIO pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param regAddr Register address for the GPIO.
 * @param pGpioCfg Pointer to store the GPIO configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_gpioGetGPOConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t regAdr, Pmic_GpioCfg_t * pGpioCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Read GPO configuration register */
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAdr, & regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Process and populate the GPIO configuration */
        if (regAdr == PMIC_GPO_CFG1_REG_ADDR) {
            /* PMIC_BB_GPO1 and PMIC_BB_GPO2 configurations */
            pGpioCfg -> gpo1Cfg = Pmic_getBitField(
                regData, PMIC_GPO_CFG1_GPO1_CFG_SHIFT, (uint8_t)PMIC_GPO_CFG1_GPO1_CFG_MASK);
            pGpioCfg -> gpo2Cfg = Pmic_getBitField(
                regData, PMIC_GPO_CFG1_GPO2_CFG_SHIFT, (uint8_t)PMIC_GPO_CFG1_GPO2_CFG_MASK);
        } else if (regAdr == PMIC_GPO_CFG2_REG_ADDR) {
            /* PMIC_BB_GPO3 and PMIC_BB_GPO4 configurations */
            pGpioCfg -> gpo3Cfg = Pmic_getBitField(
                regData, PMIC_GPO_CFG2_GPO3_CFG_SHIFT, (uint8_t)PMIC_GPO_CFG2_GPO3_CFG_MASK);
            pGpioCfg -> gpo4Cfg = Pmic_getBitField(
                regData, PMIC_GPO_CFG2_GPO4_CFG_SHIFT, (uint8_t)PMIC_GPO_CFG2_GPO4_CFG_MASK);
            pGpioCfg -> pinDir = Pmic_getBitField(regData, PMIC_GPO_CFG2_GPO_EN_SHIFT,
                                                  (uint8_t)PMIC_GPO_CFG2_GPO_EN_MASK);
        } else {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    return pmicStatus;
}

/**
 * @brief Get the configuration of a GPIO pin.
 * This function retrieves the configuration of a GPIO pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin to get the configuration for.
 * @param pGpioCfg Pointer to store the GPIO configuration.
 * @param GpioRdbkDglCfg Pointer to store the GPIO debounce and deglitch
 * configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_gpioGetConfiguration(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t pin, Pmic_GpioCfg_t * pGpioCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        switch (pin) {
        case PMIC_GPO1:
            pmicStatus = Pmic_gpioGetGPOConfig(pPmicCoreHandle,
                PMIC_GPO_CFG1_REG_ADDR, pGpioCfg);
            break;
        case PMIC_GPO2:
            pmicStatus = Pmic_gpioGetGPOConfig(pPmicCoreHandle,
                PMIC_GPO_CFG1_REG_ADDR, pGpioCfg);
            break;
        case PMIC_GPO3:
            pmicStatus = Pmic_gpioGetGPOConfig(pPmicCoreHandle,
                PMIC_GPO_CFG2_REG_ADDR, pGpioCfg);
            break;
        case PMIC_GPO4:
            pmicStatus = Pmic_gpioGetGPOConfig(pPmicCoreHandle,
                PMIC_GPO_CFG2_REG_ADDR, pGpioCfg);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM; /* Invalid pin */
            break;
        }
    }

    return pmicStatus;
}

/**
 * @brief Set the value of a GPIO pin.
 * This function sets the value of a GPIO pin based on the provided
 * configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pGpioInOutCfg Pointer to the GPIO input/output configuration.
 * @param pinValue The value to be set for the GPIO pin (0 for LOW, 1 for HIGH).
 * @param index Index of the GPIO pin in the GPIO input/output configuration
 * array.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_gpioSetPinValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_GpioInOutCfg_t * pGpioInOutCfg,
        const uint8_t pinValue) {
    uint8_t regData = 0U;
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t bitMask = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* checking for the pin direction to be output */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        (uint16_t)pGpioInOutCfg->regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Setting the GPIO value */
        pmicStatus = Pmic_commIntf_recvByte(
            pPmicCoreHandle, (uint16_t)pGpioInOutCfg->outRegAddr, & regData);
        if (PMIC_ST_SUCCESS == pmicStatus) {
            bitMask = (uint8_t)(PMIC_GPIO_IN_OUT_X_GPIOX_IN_OUT_BITFIELD <<
                    (uint16_t)pGpioInOutCfg->outRegBitPos);
            Pmic_setBitField( & regData, pGpioInOutCfg->outRegBitPos, bitMask,
                pinValue);
            pmicStatus = Pmic_commIntf_sendByte(
                pPmicCoreHandle, (uint16_t)pGpioInOutCfg->outRegAddr, regData);
        }
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief Set the value of a GPIO pin.
 * This function sets the value of a GPIO pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin to set the value for.
 * @param pinValue Value to set for the GPIO pin.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_gpioSetValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t pin,
        const uint8_t pinValue) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    pmicStatus = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (pinValue > PMIC_GPIO_HIGH_LEVEL)) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Get PMIC gpio configuration */
        Pmic_get_gpioInOutCfg(pGpioInOutCfg);

        if(pGpioInOutCfg == NULL)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
        else
        {
        /* Set PMIC gpio pin value */
            pmicStatus = Pmic_gpioSetPinValue(pPmicCoreHandle, pGpioInOutCfg, pinValue);
        }
    }

    return pmicStatus;
}

/**
 * @brief Get the value of a GPIO pin.
 * This function gets the value of a GPIO pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPIO pin to get the value for.
 * @param pPinValue Pointer to store the value of the GPIO pin.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_gpioGetValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t pin,
        uint8_t * pPinValue) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_GpioInOutCfg_t * pGpioInOutCfg = NULL;
    uint8_t bitMask = 0U;

    /* Parameter Validation */
    pmicStatus = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pPinValue)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Get PMIC gpio configuration */
        Pmic_get_gpioInOutCfg( pGpioInOutCfg);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        if(pGpioInOutCfg == NULL)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
        else
        {
            /* Reading the pin value */
            pmicStatus = Pmic_commIntf_recvByte(
                pPmicCoreHandle, (uint16_t)pGpioInOutCfg->inRegAddr, & regData);
        }
        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        bitMask = (uint8_t)(PMIC_GPIO_IN_OUT_X_GPIOX_IN_OUT_BITFIELD <<
            pGpioInOutCfg->inRegBitPos);

        if (Pmic_getBitField(regData, pGpioInOutCfg->inRegBitPos, bitMask) !=
            0U) {
            * pPinValue = PMIC_GPIO_HIGH_LEVEL;
        } else {
            * pPinValue = PMIC_GPIO_LOW_LEVEL;
        }
    }

    return pmicStatus;
}

/**
 * @brief Set the configuration of a GPI pin.
 * This function sets the configuration of a GPI pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPI pin to set the configuration for.
 * @param gpioCfg GPI configuration to set.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_gpiSetConfiguration(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t pin,
        const Pmic_GpioCfg_t gpioCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAdr = PMIC_GPI_CFG_REG_ADDR;
    uint8_t regData = 0U;

    pmicStatus = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the GPO_CFG register */
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAdr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Setting configurations for PMIC_BB_GPO1, PMIC_BB_GPO2, PMIC_BB_GPO3,
         * PMIC_BB_GPO4 based on pin */
        switch (pin) {
        case PMIC_GPI1:
            /* Modify GPI1_CFG bits in GPI_CFG*/
            Pmic_setBitField( & regData, PMIC_GPI_CFG_GPI1_SHIFT,
                              (uint8_t)PMIC_GPI_CFG_GPI1_MASK, gpioCfg.gpi1Cfg);
            break;
        case PMIC_GPI4:
            /* Modify GPI4_CFG bits in GPI_CFG*/
            Pmic_setBitField( & regData, PMIC_GPI_CFG_GPI4_SHIFT,
                              (uint8_t)PMIC_GPI_CFG_GPI4_MASK, gpioCfg.gpi4Cfg);
            break;
        default:
            /* Invalid pin */
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }
        if (PMIC_ST_SUCCESS == pmicStatus) {
            /* Sending modified configuration back to the register */
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAdr, regData);
        }
    }
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief Get the configuration of GPI pins.
 * This function retrieves the configuration of GPI pins from the specified
 * register address.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param regAddr Register address from which to read the GPI configuration.
 * @param pGpioCfg Pointer to store the GPI configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */

int32_t Pmic_gpioGetGPIConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t regAdr, Pmic_GpioCfg_t * pGpioCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Read GPO configuration register */
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAdr, & regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Process and populate the GPIO configuration */

        /* PMIC_BB_GPO1 and PMIC_BB_GPO2 configurations*/
        pGpioCfg -> gpi1Cfg = Pmic_getBitField(regData, PMIC_GPI_CFG_GPI1_SHIFT,
                                               (uint8_t)PMIC_GPI_CFG_GPI1_MASK);
        pGpioCfg -> gpi4Cfg = Pmic_getBitField(regData, PMIC_GPI_CFG_GPI4_SHIFT,
                                               (uint8_t)PMIC_GPI_CFG_GPI4_MASK);
    }
    return pmicStatus;
}

/**
 * @brief Get the configuration of a GPI pin.
 * This function gets the configuration of a GPI pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pin GPI pin to get the configuration for.
 * @param pGpioCfg Pointer to store the GPI configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_gpiGetConfiguration(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t pin, Pmic_GpioCfg_t * pGpioCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        switch (pin) {
        case PMIC_GPI1:
            pmicStatus = Pmic_gpioGetGPIConfig(pPmicCoreHandle, PMIC_GPI_CFG_REG_ADDR,
                pGpioCfg);
            break;
        case PMIC_GPI4:
            pmicStatus = Pmic_gpioGetGPIConfig(pPmicCoreHandle, PMIC_GPI_CFG_REG_ADDR,
                pGpioCfg);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM; /* Invalid pin */
            break;
        }
    }

    return pmicStatus;
}

/**
 * @brief Get the deglitch time for PMIC_BB_GPO1 and PMIC_BB_GPO2.
 * This function gets the deglitch time for PMIC_BB_GPO1 and PMIC_BB_GPO2.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param GpioRdbkDglCfg Pointer to store the deglitch time configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_gpo12GetDeglitchTime(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_GpioRdbkDglCfg_t * GpioRdbkDglCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the GPIO configuration */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_RDBK_DGL_CFG2_REGADDR, & regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {

        GpioRdbkDglCfg -> gpo1FDglData = Pmic_getBitField(
            regData, PMIC_RDBK_GPO1_F_SHIFT, PMIC_RDBK_GPO1_F_MASK);

        GpioRdbkDglCfg -> gpo1RDglData = Pmic_getBitField(
            regData, PMIC_RDBK_GPO1_R_SHIFT, PMIC_RDBK_GPO1_R_MASK);

        GpioRdbkDglCfg -> gpo2FDglData = Pmic_getBitField(
            regData, PMIC_RDBK_GPO2_F_SHIFT, PMIC_RDBK_GPO2_F_MASK);

        GpioRdbkDglCfg -> gpo2RDglData = Pmic_getBitField(
            regData, PMIC_RDBK_GPO2_R_SHIFT, PMIC_RDBK_GPO2_R_MASK);
    }
    return pmicStatus;
}

/**
 * @brief Set the deglitch time for PMIC_BB_GPO1 and PMIC_BB_GPO2.
 * This function sets the deglitch time for PMIC_BB_GPO1 and PMIC_BB_GPO2.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param GpioRdbkDglCfg Pointer to the deglitch time configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_gpo12SetDeglitchTime(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_GpioRdbkDglCfg_t * GpioRdbkDglCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0;
    Pmic_GpioRdbkDglCfg_t * tempCfg = GpioRdbkDglCfg;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the GPIO configuration */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_RDBK_DGL_CFG2_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        if (tempCfg -> gpo1FDglConfig == PMIC_GPIO_ENABLE) {
            /* Reading signal deglitch time */
            Pmic_setBitField( & regData, PMIC_RDBK_GPO1_F_SHIFT, PMIC_RDBK_GPO1_F_MASK,
                tempCfg -> gpo1FDglData);
        }
        if (tempCfg -> gpo1RDglConfig == PMIC_GPIO_ENABLE) {
            /* Reading signal deglitch time */
            Pmic_setBitField( & regData, PMIC_RDBK_GPO1_R_SHIFT, PMIC_RDBK_GPO1_R_MASK,
                tempCfg -> gpo1RDglData);
        }
        if (tempCfg -> gpo2FDglConfig == PMIC_GPIO_ENABLE) {
            /* Reading signal deglitch time */
            Pmic_setBitField( & regData, PMIC_RDBK_GPO2_F_SHIFT, PMIC_RDBK_GPO2_F_MASK,
                tempCfg -> gpo2FDglData);
        }
        if (tempCfg -> gpo2RDglConfig == PMIC_GPIO_ENABLE) {
            /* Reading signal deglitch time */
            Pmic_setBitField( & regData, PMIC_RDBK_GPO2_R_SHIFT, PMIC_RDBK_GPO2_R_MASK,
                tempCfg -> gpo2RDglData);
        }
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
            PMIC_RDBK_DGL_CFG2_REGADDR, regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }
    return pmicStatus;
}

/**
 * @brief Set the deglitch time for PMIC_BB_GPO3 and PMIC_BB_GPO4.
 * This function sets the deglitch time for PMIC_BB_GPO3 and PMIC_BB_GPO4.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param GpioRdbkDglCfg Pointer to the deglitch time configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_gpo34SetDeglitchTime(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_GpioRdbkDglCfg_t * GpioRdbkDglCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0;
    Pmic_GpioRdbkDglCfg_t * tempCfg = GpioRdbkDglCfg;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the GPIO configuration */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_RDBK_DGL_CFG3_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        if (tempCfg -> gpo3FDglConfig == PMIC_GPIO_ENABLE) {
            /* Reading signal deglitch time */
            Pmic_setBitField( & regData, PMIC_RDBK_GPO3_F_SHIFT, PMIC_RDBK_GPO3_F_MASK,
                tempCfg -> gpo3FDglData);
        }
        if (tempCfg -> gpo3RDglConfig == PMIC_GPIO_ENABLE) {
            /* Reading signal deglitch time */
            Pmic_setBitField( & regData, PMIC_RDBK_GPO3_R_SHIFT, PMIC_RDBK_GPO3_R_MASK,
                tempCfg -> gpo3RDglData);
        }
        if (tempCfg -> gpo4FDglConfig == PMIC_GPIO_ENABLE) {
            /* Reading signal deglitch time */
            Pmic_setBitField( & regData, PMIC_RDBK_GPO4_F_SHIFT, PMIC_RDBK_GPO4_F_MASK,
                tempCfg -> gpo4FDglData);
        }
        if (tempCfg -> gpo4RDglConfig == PMIC_GPIO_ENABLE) {
            /* Reading signal deglitch time */
            Pmic_setBitField( & regData, PMIC_RDBK_GPO4_R_SHIFT, PMIC_RDBK_GPO4_R_MASK,
                tempCfg -> gpo4RDglData);
        }
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
            PMIC_RDBK_DGL_CFG3_REGADDR, regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }
    return pmicStatus;
}

/**
 * @brief Get the deglitch time for PMIC_BB_GPO3 and PMIC_BB_GPO4.
 * This function gets the deglitch time for PMIC_BB_GPO3 and PMIC_BB_GPO4.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param GpioRdbkDglCfg Pointer to store the deglitch time configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_gpo34GetDeglitchTime(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_GpioRdbkDglCfg_t * GpioRdbkDglCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the GPIO configuration */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_RDBK_DGL_CFG3_REGADDR, & regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {

        GpioRdbkDglCfg -> gpo3FDglData = Pmic_getBitField(
            regData, PMIC_RDBK_GPO3_F_SHIFT, PMIC_RDBK_GPO3_F_MASK);

        GpioRdbkDglCfg -> gpo3RDglData = Pmic_getBitField(
            regData, PMIC_RDBK_GPO3_R_SHIFT, PMIC_RDBK_GPO3_R_MASK);

        GpioRdbkDglCfg -> gpo4FDglData = Pmic_getBitField(
            regData, PMIC_RDBK_GPO4_F_SHIFT, PMIC_RDBK_GPO4_F_MASK);

        GpioRdbkDglCfg -> gpo4RDglData = Pmic_getBitField(
            regData, PMIC_RDBK_GPO4_R_SHIFT, PMIC_RDBK_GPO4_R_MASK);
    }
    return pmicStatus;
}

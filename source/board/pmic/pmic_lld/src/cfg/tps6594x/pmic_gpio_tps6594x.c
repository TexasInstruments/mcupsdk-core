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
*   \file    pmic_gpio_tps6594x.c
*
*   \brief   This file contains the TPS6594x Leo PMIC GPIO Specific
*            configuration API's and structures
*
*/

#include <pmic_types.h>
#include <pmic_gpio.h>
#include <pmic_core_priv.h>
#include <pmic_io_priv.h>
#include <pmic_gpio_tps6594x_priv.h>

/* PMIC GPIO Pins with Input Ouput Configuration */
static Pmic_GpioInOutCfg_t gTps6594x_gpioInOutCfg[] =
{
    {
        PMIC_GPIO1_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO1_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO1_OUT_SHIFT
    },
    {
        PMIC_GPIO2_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO2_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO2_OUT_SHIFT
    },
    {
        PMIC_GPIO3_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO3_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO3_OUT_SHIFT
    },
    {
        PMIC_GPIO4_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO4_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO4_OUT_SHIFT
    },
    {
        PMIC_GPIO5_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO5_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO5_OUT_SHIFT
    },
    {
        PMIC_GPIO6_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO6_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO6_OUT_SHIFT
    },
    {
        PMIC_GPIO7_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO7_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO7_OUT_SHIFT
    },
    {
        PMIC_GPIO8_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO8_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO8_OUT_SHIFT
    },
    {
        PMIC_GPIO9_CONF_REGADDR,
        PMIC_GPIO_OUT_2_REGADDR,
        PMIC_GPIO_IN_2_REGADDR,
        PMIC_GPIO_IN_2_GPIO9_IN_SHIFT,
        PMIC_GPIO_OUT_2_GPIO9_OUT_SHIFT
    },
    {
        PMIC_GPIO10_CONF_REGADDR,
        PMIC_GPIO_OUT_2_REGADDR,
        PMIC_GPIO_IN_2_REGADDR,
        PMIC_GPIO_IN_2_GPIO10_IN_SHIFT,
        PMIC_GPIO_OUT_2_GPIO10_OUT_SHIFT
    },
    {
        PMIC_GPIO11_CONF_REGADDR,
        PMIC_GPIO_OUT_2_REGADDR,
        PMIC_GPIO_IN_2_REGADDR,
        PMIC_GPIO_IN_2_GPIO11_IN_SHIFT,
        PMIC_GPIO_OUT_2_GPIO11_OUT_SHIFT
    }
};

/* PMIC GPIO Interrupt Register array */
static Pmic_GpioIntRegCfg_t tps6594x_gpioIntRegCfg[] =
{
    {
        PMIC_FSM_TRIG_MASK_1_REGADDR,
        PMIC_FSM_TRIG_MASK_1_GPIO1_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_1_GPIO1_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_1_REGADDR,
        PMIC_FSM_TRIG_MASK_1_GPIO2_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_1_GPIO2_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_1_REGADDR,
        PMIC_FSM_TRIG_MASK_1_GPIO3_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_1_GPIO3_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_1_REGADDR,
        PMIC_FSM_TRIG_MASK_1_GPIO4_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_1_GPIO4_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_2_REGADDR,
        PMIC_FSM_TRIG_MASK_2_GPIO5_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_2_GPIO5_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_2_REGADDR,
        PMIC_FSM_TRIG_MASK_2_GPIO6_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_2_GPIO6_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_2_REGADDR,
        PMIC_FSM_TRIG_MASK_2_GPIO7_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_2_GPIO7_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_2_REGADDR,
        PMIC_FSM_TRIG_MASK_2_GPIO8_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_2_GPIO8_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_3_REGADDR,
        PMIC_FSM_TRIG_MASK_3_GPIO9_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_3_GPIO9_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_3_REGADDR,
        PMIC_FSM_TRIG_MASK_3_GPIO10_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_3_GPIO10_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_3_REGADDR,
        PMIC_FSM_TRIG_MASK_3_GPIO11_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_3_GPIO11_FSM_MASK_POL_SHIFT
    }
};

/*!
 * \brief  PMIC GPIO pin get Input Ouput Configuration function
 *         This function is used to read the PMIC GPIO Pins with Input Ouput
 *         Configuration
 *
 * \param  pGpioInOutCfg   [OUT]  Pointer to store gpio Input Ouput
 *                                configuration
 */
void pmic_get_tps6594x_gpioInOutCfg(Pmic_GpioInOutCfg_t **pGpioInOutCfg)
{
    *pGpioInOutCfg = gTps6594x_gpioInOutCfg;
}

/*!
 * \brief  Get PMIC GPIO Pin Interrupt Register configuration
 *         This function is used to read the PMIC GPIO Interrupt Register
 *         configuration
 *
 * \param   pGpioIntRegCfg   [OUT]  Pointer to store gpio Interrupt Register
 *                                  configuration
*/
void pmic_get_tps6594x_gpioIntRegCfg(Pmic_GpioIntRegCfg_t **pGpioIntRegCfg)
{
    *pGpioIntRegCfg = tps6594x_gpioIntRegCfg;
}

/*!
 * \brief   API to get PMIC GPIO NPWRON pin value.
 *
 * Requirement: REQ_TAG(PDK-5808)
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
                                           uint8_t           *pPinValue)
{
    int32_t status   = PMIC_ST_SUCCESS;
    uint8_t regData  = 0U;

    /* Parameter Validation */
    if(NULL == pPmicCoreHandle)
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == status) &&
       (PMIC_DEV_LEO_TPS6594X != pPmicCoreHandle->pmicDeviceType))
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)false == pPmicCoreHandle->pPmic_SubSysInfo->gpioEnable))
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }

    if((PMIC_ST_SUCCESS == status) && (NULL == pPinValue))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading the NPWRON pin value */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_GPIO_IN_2_REGADDR,
                                        &regData);
        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == status)
        {
            *pPinValue= Pmic_getBitField(regData,
                                         PMIC_GPIO_IN_2_NPWRON_IN_SHIFT,
                                         PMIC_GPIO_IN_2_NPWRON_IN_MASK);
        }
    }

    return status;
}

/*!
 * \brief   This function is used to configure NPWRON pin for TPS6594x
 *          PMIC LEO Device.
 */
int32_t Pmic_gpioTps6594xSetNPwronPinConfiguration(
                                          Pmic_CoreHandle_t   *pPmicCoreHandle,
                                          const Pmic_GpioCfg_t gpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if((bool)true == pmic_validParamCheck(gpioCfg.validParams,
                                          PMIC_GPIO_CFG_PINFUNC_VALID))
    {
        /* Setting NPWRON/Enable pin function */
        status = Pmic_gpioSetPinFunc(pPmicCoreHandle,
                                     PMIC_NPWRON_ENABLE_PIN,
                                     gpioCfg);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Setting NPWRON/Enable deglitch time and Pull UP/Down Configuration*/
        status = Pmic_gpioSetNPwronEnableDeglitchPullCtrlCfg(
                                                           pPmicCoreHandle,
                                                           gpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(gpioCfg.validParams,
                                   PMIC_ENABLE_CFG_POLARITY_VALID)))
    {
        if(gpioCfg.pinPolarity > PMIC_GPIO_POL_HIGH)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if(PMIC_ST_SUCCESS == status)
        {
             /* Setting ENABLE pin polarity */
            status = Pmic_gpioSetPinPolarity(pPmicCoreHandle,
                                             gpioCfg);
        }
    }

    return status;
}

/*!
 * \brief   This function is used to read NPWRON pin configuration for TPS6594x
 *          PMIC LEO Device.
 */
int32_t Pmic_gpioTps6594xGetNPwronPinConfiguration(
                                        Pmic_CoreHandle_t   *pPmicCoreHandle,
                                        Pmic_GpioCfg_t      *pGpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if((bool)true == pmic_validParamCheck(pGpioCfg->validParams,
                                          PMIC_GPIO_CFG_DEGLITCH_VALID))
    {
        /* Get nPWRON/Enable pin signal deglitch time */
        status = Pmic_gpioGetDeglitchTime(pPmicCoreHandle,
                                          PMIC_NPWRON_ENABLE_PIN,
                                          pGpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(pGpioCfg->validParams,
                                           PMIC_GPIO_CFG_PINFUNC_VALID)))
    {
        /* Get nPWRON/Enable pin signal function */
        status = Pmic_gpioGetPinFunc(pPmicCoreHandle,
                                     PMIC_NPWRON_ENABLE_PIN,
                                     pGpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(pGpioCfg->validParams,
                                           PMIC_GPIO_CFG_PULL_VALID)))
    {
        /* Get nPWRON/Enable pin pull-up/down control */
        status = Pmic_gpioGetPullCtrl(pPmicCoreHandle,
                                      PMIC_NPWRON_ENABLE_PIN,
                                      pGpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(pGpioCfg->validParams,
                                           PMIC_ENABLE_CFG_POLARITY_VALID)))
    {
        /* Get nPWRON pin polarity control */
        status = Pmic_gpioGetPinPolarity(pPmicCoreHandle,
                                         pGpioCfg);
    }

    return status;
}

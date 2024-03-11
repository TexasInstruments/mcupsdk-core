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
*   \file    pmic_power_tps6594x.c
*
*   \brief   This file contains the TPS6594x Leo PMIC power Specific
*            configuration API's and structures
*
*/

#include <pmic_types.h>
#include <pmic_power.h>
#include <pmic_core_priv.h>
#include <pmic_io_priv.h>
#include <pmic_power_tps6594x_priv.h>

static Pmic_powerRsrcRegCfg_t gTps6594x_pwrRsrcRegCfg[] =
{
    {
        PMIC_VCCA_VMON_CTRL_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_VCCA_PG_WINDOW_REGADDR,
        PMIC_TPS6594X_VCCA_OV_INT,
        PMIC_STAT_VMON_REGADDR,
        PMIC_RAIL_SEL_3_REGADDR,
        PMIC_RAIL_SEL_3_VCCA_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_STAT_VMON_VCCA_UV_STAT_SHIFT,
        PMIC_STAT_VMON_VCCA_OV_STAT_SHIFT
    },
    {
        PMIC_BUCK1_CTRL_REGADDR,
        PMIC_BUCK1_CONF_REGADDR,
        PMIC_BUCK1_VOUT_1_REGADDR,
        PMIC_BUCK1_VOUT_2_REGADDR,
        PMIC_BUCK1_PG_WIN_REGADDR,
        PMIC_TPS6594X_BUCK1_OV_INT,
        PMIC_STAT_BUCK1_2_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_RAIL_SEL_1_BUCK1_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_STAT_BUCKX_Y_BUCK1_3_ILIM_STAT_SHIFT,
        PMIC_STAT_BUCKX_Y_BUCK1_3_UV_STAT_SHIFT,
        PMIC_STAT_BUCKX_Y_BUCK1_3_OV_STAT_SHIFT
    },
    {
        PMIC_BUCK2_CTRL_REGADDR,
        PMIC_BUCK2_CONF_REGADDR,
        PMIC_BUCK2_VOUT_1_REGADDR,
        PMIC_BUCK2_VOUT_2_REGADDR,
        PMIC_BUCK2_PG_WIN_REGADDR,
        PMIC_TPS6594X_BUCK2_OV_INT,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_STAT_BUCK1_2_REGADDR,
        PMIC_RAIL_SEL_1_BUCK2_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_STAT_BUCKX_Y_BUCK2_4_ILIM_STAT_SHIFT,
        PMIC_STAT_BUCKX_Y_BUCK2_4_UV_STAT_SHIFT,
        PMIC_STAT_BUCKX_Y_BUCK2_4_OV_STAT_SHIFT
    },
    {
        PMIC_BUCK3_CTRL_REGADDR,
        PMIC_BUCK3_CONF_REGADDR,
        PMIC_BUCK3_VOUT_1_REGADDR,
        PMIC_BUCK3_VOUT_2_REGADDR,
        PMIC_BUCK3_PG_WIN_REGADDR,
        PMIC_TPS6594X_BUCK3_OV_INT,
        PMIC_STAT_BUCK3_4_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_RAIL_SEL_1_BUCK3_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_STAT_BUCKX_Y_BUCK1_3_ILIM_STAT_SHIFT,
        PMIC_STAT_BUCKX_Y_BUCK1_3_UV_STAT_SHIFT,
        PMIC_STAT_BUCKX_Y_BUCK1_3_OV_STAT_SHIFT
    },
    {
        PMIC_BUCK4_CTRL_REGADDR,
        PMIC_BUCK4_CONF_REGADDR,
        PMIC_BUCK4_VOUT_1_REGADDR,
        PMIC_BUCK4_VOUT_2_REGADDR,
        PMIC_BUCK4_PG_WIN_REGADDR,
        PMIC_TPS6594X_BUCK4_OV_INT,
        PMIC_STAT_BUCK3_4_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_RAIL_SEL_1_BUCK4_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_STAT_BUCKX_Y_BUCK2_4_ILIM_STAT_SHIFT,
        PMIC_STAT_BUCKX_Y_BUCK2_4_UV_STAT_SHIFT,
        PMIC_STAT_BUCKX_Y_BUCK2_4_OV_STAT_SHIFT
    },
    {
        PMIC_BUCK5_CTRL_REGADDR,
        PMIC_BUCK5_CONF_REGADDR,
        PMIC_BUCK5_VOUT_1_REGADDR,
        PMIC_BUCK5_VOUT_2_REGADDR,
        PMIC_BUCK5_PG_WIN_REGADDR,
        PMIC_TPS6594X_BUCK5_OV_INT,
        PMIC_STAT_BUCK5_REGADDR,
        PMIC_RAIL_SEL_2_REGADDR,
        PMIC_RAIL_SEL_2_BUCK5_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_STAT_BUCK5_BUCK5_ILIM_STAT_SHIFT,
        PMIC_STAT_BUCK5_BUCK5_UV_STAT_SHIFT,
        PMIC_STAT_BUCK5_BUCK5_OV_STAT_SHIFT
    },
    {
        PMIC_LDO1_CTRL_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_LDO1_VOUT_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_LDO1_PG_WIN_REGADDR,
        PMIC_TPS6594X_LDO1_OV_INT,
        PMIC_STAT_LDO1_2_REGADDR,
        PMIC_RAIL_SEL_2_REGADDR,
        PMIC_RAIL_SEL_2_LDO1_GRP_SEL_SHIFT,
        PMIC_LDO_RV_TIMEOUT_REG_1_REGADDR,
        PMIC_LDO_RV_TIMEOUT_REG_LDO1_3_RV_TIMEOUT_SHIFT,
        PMIC_STAT_LDOX_Y_LDO1_3_ILIM_STAT_SHIFT,
        PMIC_STAT_LDOX_Y_LDO1_3_UV_STAT_SHIFT,
        PMIC_STAT_LDOX_Y_LDO1_3_OV_STAT_SHIFT
    },
    {
        PMIC_LDO2_CTRL_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_LDO2_VOUT_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_LDO2_PG_WIN_REGADDR,
        PMIC_TPS6594X_LDO2_OV_INT,
        PMIC_STAT_LDO1_2_REGADDR,
        PMIC_RAIL_SEL_2_REGADDR,
        PMIC_RAIL_SEL_2_LDO2_GRP_SEL_SHIFT,
        PMIC_LDO_RV_TIMEOUT_REG_1_REGADDR,
        PMIC_LDO_RV_TIMEOUT_REG_LDO2_4_RV_TIMEOUT_SHIFT,
        PMIC_STAT_LDOX_Y_LDO2_4_ILIM_STAT_SHIFT,
        PMIC_STAT_LDOX_Y_LDO2_4_UV_STAT_SHIFT,
        PMIC_STAT_LDOX_Y_LDO2_4_OV_STAT_SHIFT
    },
    {
        PMIC_LDO3_CTRL_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_LDO3_VOUT_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_LDO3_PG_WIN_REGADDR,
        PMIC_TPS6594X_LDO3_OV_INT,
        PMIC_STAT_LDO3_4_REGADDR,
        PMIC_RAIL_SEL_2_REGADDR,
        PMIC_RAIL_SEL_2_LDO3_GRP_SEL_SHIFT,
        PMIC_LDO_RV_TIMEOUT_REG_2_REGADDR,
        PMIC_LDO_RV_TIMEOUT_REG_LDO1_3_RV_TIMEOUT_SHIFT,
        PMIC_STAT_LDOX_Y_LDO1_3_ILIM_STAT_SHIFT,
        PMIC_STAT_LDOX_Y_LDO1_3_UV_STAT_SHIFT,
        PMIC_STAT_LDOX_Y_LDO1_3_OV_STAT_SHIFT
    },
    {
        PMIC_LDO4_CTRL_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_LDO4_VOUT_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_LDO4_PG_WIN_REGADDR,
        PMIC_TPS6594X_LDO4_OV_INT,
        PMIC_STAT_LDO3_4_REGADDR,
        PMIC_RAIL_SEL_3_REGADDR,
        PMIC_RAIL_SEL_3_LDO4_GRP_SEL_SHIFT,
        PMIC_LDO_RV_TIMEOUT_REG_2_REGADDR,
        PMIC_LDO_RV_TIMEOUT_REG_LDO2_4_RV_TIMEOUT_SHIFT,
        PMIC_STAT_LDOX_Y_LDO2_4_ILIM_STAT_SHIFT,
        PMIC_STAT_LDOX_Y_LDO2_4_UV_STAT_SHIFT,
        PMIC_STAT_LDOX_Y_LDO2_4_OV_STAT_SHIFT
    },

};

static Pmic_powerPgoodSrcRegCfg_t tps6594x_pgoodSrcRegCfg[] =
{
    {
        PMIC_PGOOD_SEL_4_REGADDR,
        PMIC_PGOOD_SEL_4_PGOOD_SEL_VCCA_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_1_REGADDR,
        PMIC_PGOOD_SEL_1_PGOOD_SEL_BUCK1_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_1_REGADDR,
        PMIC_PGOOD_SEL_1_PGOOD_SEL_BUCK2_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_1_REGADDR,
        PMIC_PGOOD_SEL_1_PGOOD_SEL_BUCK3_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_1_REGADDR,
        PMIC_PGOOD_SEL_1_PGOOD_SEL_BUCK4_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_2_REGADDR,
        PMIC_PGOOD_SEL_2_PGOOD_SEL_BUCK5_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_4_REGADDR,
        PMIC_PGOOD_SEL_4_PGOOD_SEL_NRSTOUT_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_4_REGADDR,
        PMIC_PGOOD_SEL_4_PGOOD_SEL_NRSTOUT_SOC_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_4_REGADDR,
        PMIC_PGOOD_SEL_4_PGOOD_SEL_TDIE_WARN_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_3_REGADDR,
        PMIC_PGOOD_SEL_3_PGOOD_SEL_LDO1_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_3_REGADDR,
        PMIC_PGOOD_SEL_3_PGOOD_SEL_LDO2_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_3_REGADDR,
        PMIC_PGOOD_SEL_3_PGOOD_SEL_LDO3_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_3_REGADDR,
        PMIC_PGOOD_SEL_3_PGOOD_SEL_LDO4_SHIFT,
    },
};

static Pmic_powerIntCfg_t tps6594x_pwrIntCfg[] =
{
    {
        PMIC_TPS6594X_TWARN_INT,
    },
    {
        PMIC_TPS6594X_NRSTOUT_READBACK_INT,
    },
    {
        PMIC_TPS6594X_SOC_PWR_ERR_INT,
    },
    {
        PMIC_TPS6594X_MCU_PWR_ERR_INT,
    },
    {
        PMIC_TPS6594X_ORD_SHUTDOWN_INT,
    },
    {
        PMIC_TPS6594X_IMM_SHUTOWN_INT,
    },
    {
        PMIC_TPS6594X_NRSTOUT_SOC_READBACK_INT,
    },
    {
        PMIC_TPS6594X_EN_DRV_READBACK_INT,
    },

};

/*!
 * \brief  PMIC power common interrupt get Configuration function
 *         This function is used to read the interrupt
 *         Configuration
 *
 * \param  pwrRsrcRegCfg   [OUT]  Pointer to store power interrupt
 *                                number.
 */
void pmic_get_tps6594x_pwrCommonIntCfg(Pmic_powerIntCfg_t **pPwrCommonIntCfg)
{
    *pPwrCommonIntCfg = tps6594x_pwrIntCfg;
}

/*!
 * \brief  PMIC power resources get Configuration function
 *         This function is used to read the PMIC POWER resources register
 *         Configuration
 *
 * \param  pwrRsrcRegCfg   [OUT]  Pointer to store power resource register
 *                                configuration
 */
void pmic_get_tps6594x_pwrRsrceRegCfg(Pmic_powerRsrcRegCfg_t **pPwrRsrcRegCfg)
{
    *pPwrRsrcRegCfg = gTps6594x_pwrRsrcRegCfg;
}

/*!
 * \brief  PMIC power get Configuration function
 *         This function is used to read the PMIC pgood sources register
 *         Configuration
 *
 * \param  pPgoodSrcRegCfg   [OUT]  Pointer to store power-good source register
 *                                  configuration
 */
void pmic_get_tps6594x_pwrPgoodSrcRegCfg(
                                   Pmic_powerPgoodSrcRegCfg_t **pPgoodSrcRegCfg)
{
    *pPgoodSrcRegCfg = tps6594x_pgoodSrcRegCfg;
}

/*!
 * \brief   This function is used to convert the millivolt value to vset value
 *          for LEO TPS6594x PMIC
 *
 *          Note: In this API, While adding support for New pwrRsrcType,
 *                developer need to update the API functionality for New
 *                pwrRsrcType accordingly.
 */
int32_t Pmic_powerTPS6594xConvertVoltage2VSetVal(uint16_t  millivolt,
                                                 uint16_t  pwrRsrc,
                                                 uint8_t   *pVSetVal)
{
    int32_t  status = PMIC_ST_SUCCESS;
    uint8_t  pwrRsrcType = 0U;
    uint16_t baseMillivolt = 0U;
    uint8_t  millivoltStep = 0U;
    uint8_t  baseVoutCode  = 0U;

    pwrRsrcType = Pmic_powerGetPwrRsrcType(pwrRsrc);

    if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
    {
        status = Pmic_powerBuckVmonConvertVoltage2VSetVal(millivolt,
                                                          &baseMillivolt,
                                                          &millivoltStep,
                                                          &baseVoutCode);

    }
    else
    {
        /* Else case for LDO pwrRsrcType */
        Pmic_powerLdoConvertVoltage2VSetVal(pwrRsrc,
                                            &baseMillivolt,
                                            &millivoltStep,
                                            &baseVoutCode);
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((millivolt % millivoltStep) == 1U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        *pVSetVal = (uint8_t)(baseVoutCode +
                    ((millivolt - baseMillivolt) / millivoltStep));
    }

    return status;
}

/*!
 * \brief   This function is used to convert the vset value to voltage in mv
 *          for PMIC LEO TPS6594x
 *
 *          Note: In this API, While adding support for New pwrRsrcType,
 *                developer need to update the API functionality for New
 *                pwrRsrcType accordingly.
 */
void Pmic_powerTPS6594xConvertVSet2Voltage(const uint8_t  *pVSetVal,
                                           uint16_t        pwrRsrc,
                                           uint16_t       *millivolt)
{
    uint16_t baseMillivolt = 0U;
    uint8_t  millivoltStep = 0U;
    uint8_t  baseVoutCode  = 0U;
    uint8_t pwrRsrcType;

    pwrRsrcType = Pmic_powerGetPwrRsrcType(pwrRsrc);
    if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
    {
        Pmic_powerBuckVmonConvertVSetVal2Voltage(pVSetVal,
                                                 &baseMillivolt,
                                                 &millivoltStep,
                                                 &baseVoutCode);
    }
    else
    {
        /* Else case for LDO pwrRsrcType */
        Pmic_powerLdoConvertVSetVal2Voltage(pwrRsrc,
                                            &baseMillivolt,
                                            &millivoltStep,
                                            &baseVoutCode);
    }

    *millivolt = (baseMillivolt +
                 (((uint16_t)*pVSetVal - baseVoutCode) * millivoltStep));

}

/*!
 * \brief   This function is to validate the power good source limit for VCCA
 *          BUCK, LDO
 */
static int32_t Pmic_validate_tps6594x_pGoodVccaBuckLDOSrcType(
                                                          uint16_t pgoodSrc,
                                                          uint8_t  pGoodSrcType)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(PMIC_TPS6594X_PGOOD_SOURCE_TYPE_VCCA == pGoodSrcType)
    {
        if(pgoodSrc != PMIC_TPS6594X_PGOOD_SOURCE_VCCA)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_TPS6594X_PGOOD_SOURCE_TYPE_BUCK == pGoodSrcType)
    {
        if((pgoodSrc > PMIC_TPS6594X_PGOOD_BUCK_MAX) ||
           (pgoodSrc < PMIC_TPS6594X_PGOOD_BUCK_MIN))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else
    {
        if((pgoodSrc > PMIC_TPS6594X_PGOOD_LDO_MAX) ||
           (pgoodSrc < PMIC_TPS6594X_PGOOD_LDO_MIN))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

/*!
 * \brief   This function is to validate the power good source limit for the
 *          specific PMIC device.
 */
int32_t Pmic_validate_tps6594x_pGoodSrcType(uint16_t pgoodSrc)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t  pGoodSrcType;

    pGoodSrcType = Pmic_powerGetPwrRsrcType(pgoodSrc);

    if((PMIC_TPS6594X_PGOOD_SOURCE_TYPE_VCCA == pGoodSrcType) ||
       (PMIC_TPS6594X_PGOOD_SOURCE_TYPE_BUCK == pGoodSrcType) ||
       (PMIC_TPS6594X_PGOOD_SOURCE_TYPE_LDO == pGoodSrcType))
       {
           status = Pmic_validate_tps6594x_pGoodVccaBuckLDOSrcType(
                                                                  pgoodSrc,
                                                                  pGoodSrcType);
       }
    else if(PMIC_TPS6594X_PGOOD_SOURCE_TYPE_NRSTOUT == pGoodSrcType)
    {
        if(pgoodSrc != PMIC_TPS6594X_PGOOD_SOURCE_NRSTOUT)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_TPS6594X_PGOOD_SOURCE_TYPE_NRSTOUT_SOC == pGoodSrcType)
    {
        if(pgoodSrc != PMIC_TPS6594X_PGOOD_SOURCE_NRSTOUT_SOC)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_TPS6594X_PGOOD_SOURCE_TYPE_TDIE == pGoodSrcType)
    {
        if(pgoodSrc != PMIC_TPS6594X_PGOOD_SOURCE_TDIE)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/*!
 * \brief   This function is to validate the power good signal source selection
 *          limit for BUCK, LDO, NRSTOUT, NRSTOUT_SOC
 */
static int32_t Pmic_validate_tps6594x_pGoodSelBuckLdoNrstoutNrstoutsoc(
                                                        uint8_t  pGoodSrcType,
                                                        uint8_t  pgoodSelType)
{
    int32_t status = PMIC_ST_SUCCESS;

    if((PMIC_TPS6594X_PGOOD_SOURCE_TYPE_BUCK == pGoodSrcType) ||
       (PMIC_TPS6594X_PGOOD_SOURCE_TYPE_LDO == pGoodSrcType))
    {
        if(pgoodSelType > PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_VOLTAGE_CURRENT)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_TPS6594X_PGOOD_SOURCE_TYPE_NRSTOUT == pGoodSrcType)
    {
        if(pgoodSelType > PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else
    {
        if(pgoodSelType > PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT_SOC)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

return status;
}

/*!
 * \brief   This function is to validate the power good signal source selection
 *          limit for the specific PMIC device.
 *
 *          Note: In this API, While adding support for New pGoodSrcType,
 *                developer need to update the API functionality for New
 *                pGoodSrcType accordingly.
 */
int32_t Pmic_validate_tps6594x_pGoodSelType(uint16_t pgoodSrc,
                                            uint8_t pgoodSelType)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t  pGoodSrcType;

    pGoodSrcType = Pmic_powerGetPwrRsrcType(pgoodSrc);

    if((PMIC_TPS6594X_PGOOD_SOURCE_TYPE_BUCK == pGoodSrcType)    ||
       (PMIC_TPS6594X_PGOOD_SOURCE_TYPE_LDO == pGoodSrcType)     ||
       (PMIC_TPS6594X_PGOOD_SOURCE_TYPE_NRSTOUT == pGoodSrcType) ||
       (PMIC_TPS6594X_PGOOD_SOURCE_TYPE_NRSTOUT_SOC == pGoodSrcType))
       {
           status = Pmic_validate_tps6594x_pGoodSelBuckLdoNrstoutNrstoutsoc(
                                                                   pGoodSrcType,
                                                                   pgoodSelType);
       }
    else if(PMIC_TPS6594X_PGOOD_SOURCE_TYPE_TDIE == pGoodSrcType)
    {
        if(pgoodSelType > PMIC_TPS6594X_POWER_PGOOD_SEL_TDIE_WARN)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else
    {
        /* Else case for VCCA pGoodSrcType */
        if(pgoodSelType >  PMIC_TPS6594X_POWER_PGOOD_SEL_VCCA_ENABLE)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

/* Enable/Disable LDORTC regulator */
static int32_t Pmic_powerLdoRtcEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      bool               ldortcEnable)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData;
    uint8_t ldortcEnableVal = 0U;

    switch (pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            Pmic_criticalSectionStart(pPmicCoreHandle);
            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_LDORTC_CTRL_REGADDR,
                                            &regData);

            if(PMIC_ST_SUCCESS == status)
            {
                if(((bool)true) == ldortcEnable)
                {
                    ldortcEnableVal = 1U;
                }

                /* Set ldortcEnable */
                Pmic_setBitField(&regData,
                                 PMIC_LDORTC_CTRL_LDORTC_DIS_SHIFT,
                                 PMIC_LDORTC_CTRL_LDORTC_DIS_MASK,
                                 ldortcEnableVal);
                status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                PMIC_LDORTC_CTRL_REGADDR,
                                                regData);
            }

            Pmic_criticalSectionStop(pPmicCoreHandle);
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/* Get Enable/Disable LDORTC regulator status */
static int32_t Pmic_powerGetLdoRtcEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                         bool              *pLdortcEnable)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData;

    switch (pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            Pmic_criticalSectionStart(pPmicCoreHandle);
            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_LDORTC_CTRL_REGADDR,
                                            &regData);
            Pmic_criticalSectionStop(pPmicCoreHandle);
            if(PMIC_ST_SUCCESS == status)
            {
                /* Get ldortcEnable */
                if(Pmic_getBitField(regData,
                                    PMIC_LDORTC_CTRL_LDORTC_DIS_SHIFT,
                                    PMIC_LDORTC_CTRL_LDORTC_DIS_MASK) != 0U)
                {
                    *pLdortcEnable = (bool)true;
                }
                else
                {
                    *pLdortcEnable = (bool)false;
                }

            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   API to enable/disable LDORTC regulator
 *
 * Requirement: REQ_TAG(PDK-5841)
 * Design: did_pmic_power_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function is used to enable/disable LDORTC regulator.
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   ldortcEnable       [IN]    Enable/Disable the LDORTC.
 *                                     Valid values:
 *                                     \ref Pmic_Tps6594x_PowerLdoRtcCfg
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetLdoRtc(Pmic_CoreHandle_t *pPmicCoreHandle,
                            bool               ldortcEnable)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerLdoRtcEnable(pPmicCoreHandle, ldortcEnable);
    }

    return pmicStatus;
}

/*!
 * \brief   API to get enable/disable status for LODRTC regulator
 *
 * Requirement: REQ_TAG(PDK-5841)
 * Design: did_pmic_power_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function is used to get enable/disable status for LDORTC
 *          regulator.
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   pLdortcEnable      [IN]    Pointer to hold Enable/Disable status.
 *                                     Valid values:
 *                                     \ref Pmic_Tps6594x_PowerLdoRtcCfg
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetLdoRtc(Pmic_CoreHandle_t *pPmicCoreHandle,
                            bool              *pLdortcEnable)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pLdortcEnable))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerGetLdoRtcEnable(pPmicCoreHandle, pLdortcEnable);
    }

    return pmicStatus;
}

/**
 * \brief   This function is used to validate the voltage levels for
 *          Regulators/VMON for TPS6594x PMIC
 *
 *          Note: In this API, While adding support for New pwrRsrcType,
 *                developer need to update the API functionality for New
 *                pwrRsrcType accordingly.
 */
int32_t Pmic_powerTPS6594xValidateVoltageLevel(
                                             uint8_t            pwrRsrcType,
                                             uint16_t           pwrRsrc,
                                             uint16_t           voltage_mV)
{
    int32_t  status = PMIC_ST_SUCCESS;
    uint16_t ldoMinVoltageValue = 0U;

    if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
    {
        if((voltage_mV < PMIC_TPS6594X_REGULATOR_BUCK_MIN_VOLTAGE) ||
           (voltage_mV > PMIC_TPS6594X_REGULATOR_BUCK_MAX_VOLTAGE))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else
    {
        /* Else case for LDO pwrRsrcType */
        if(PMIC_TPS6594X_REGULATOR_LDO4 == pwrRsrc)
        {
            ldoMinVoltageValue = PMIC_TPS6594X_POWER_LDO4_MIN_VOLTAGE;
        }
        else
        {
            ldoMinVoltageValue = \
                               PMIC_TPS6594X_POWER_LDO1_2_3_MIN_VOLTAGE;
        }

        if((voltage_mV < ldoMinVoltageValue) ||
           (voltage_mV > PMIC_TPS6594X_POWER_LDO_MAX_VOLTAGE))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

/*!
 * \brief   This function is to validate the power resource limit for the
 *          TPS6594x PMIC device.
 */
int32_t Pmic_powerTPS6594xValidatePwrRsrcLimit(
                                    const Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint8_t                  pwrRsrcType,
                                    uint16_t                 pwrRsrc)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_VCCA == pwrRsrcType)
    {
        if(pwrRsrc != PMIC_TPS6594X_POWER_SOURCE_VCCA)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
    {
        if(((bool)false) == pPmicCoreHandle->pPmic_SubSysInfo->buckEnable)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            if((pwrRsrc > PMIC_TPS6594X_BUCK_MAX) ||
               (pwrRsrc < PMIC_TPS6594X_BUCK_MIN))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
        }
    }
    else if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO == pwrRsrcType)
    {
        if(((bool)false) == pPmicCoreHandle->pPmic_SubSysInfo->ldoEnable)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            if((pwrRsrc > PMIC_TPS6594X_LDO_MAX) ||
               (pwrRsrc < PMIC_TPS6594X_LDO_MIN))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
        }
    }
    else
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/*!
 * \brief   This function is to validate the power resource interrupt type
 *          for the TPS6594x PMIC device.
 *
 *          Note: In this API, While adding support for New pwrResourceType,
 *                developer need to update the API functionality for New
 *                pwrResourceType accordingly.
 */
int32_t Pmic_powerTPS6594xValidateIntrType(uint8_t  pwrResourceType,
                                           uint8_t  intrType)
{
    int32_t status = PMIC_ST_SUCCESS;

    if((PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK == pwrResourceType) ||
       (PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO == pwrResourceType))
    {
        if((intrType != PMIC_TPS6594X_POWER_OV_INT) &&
           (intrType != PMIC_TPS6594X_POWER_UV_INT) &&
           (intrType != PMIC_TPS6594X_POWER_ILIM_INT))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else
    {
        /* Else case for VCCA pwrResourceType */
        if((intrType != PMIC_TPS6594X_POWER_OV_INT) &&
           (intrType != PMIC_TPS6594X_POWER_UV_INT))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

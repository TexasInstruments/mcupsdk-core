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
*   \file    pmic_power_lp8764x.c
*
*   \brief   This file contains the LP8764X Leo PMIC power Specific
*            configuration API's and structures
*
*/

#include <pmic_types.h>
#include <pmic_power.h>
#include <pmic_core_priv.h>
#include <pmic_io_priv.h>
#include <pmic_power_lp8764x_priv.h>

static Pmic_powerRsrcRegCfg_t gLp8764x_pwrRsrcRegCfg[] =
{
    {
        PMIC_VCCA_VMON_CTRL_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_VCCA_PG_WINDOW_REGADDR,
        PMIC_LP8764X_VCCA_OV_INT,
        PMIC_STAT_VMON_REGADDR,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_RAIL_SEL_3_VCCA_GRP_SEL_SHIFT,
        PMIC_INVALID_BIT_SHIFT,
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
        PMIC_LP8764X_BUCK1_OV_INT,
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
        PMIC_LP8764X_BUCK2_OV_INT,
        PMIC_STAT_BUCK1_2_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
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
        PMIC_LP8764X_BUCK3_OV_INT,
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
        PMIC_LP8764X_BUCK4_OV_INT,
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
        PMIC_VCCA_VMON_CTRL_REGADDR,
        PMIC_VMON_CONF_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_VMON1_PG_WINDOW_REGADDR,
        PMIC_LP8764X_VMON1_OV_INT,
        PMIC_STAT_VMON_REGADDR,
        PMIC_RAIL_SEL_3_REGADDR,
        PMIC_RAIL_SEL_3_VMON1_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_STAT_VMON_VMON1_UV_STAT_SHIFT,
        PMIC_STAT_VMON_VMON1_OV_STAT_SHIFT
    },
    {
        PMIC_VCCA_VMON_CTRL_REGADDR,
        PMIC_VMON_CONF_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_VMON2_PG_WINDOW_REGADDR,
        PMIC_LP8764X_VMON2_OV_INT,
        PMIC_STAT_VMON_REGADDR,
        PMIC_RAIL_SEL_3_REGADDR,
        PMIC_RAIL_SEL_3_VMON2_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_STAT_VMON_VMON2_UV_STAT_SHIFT,
        PMIC_STAT_VMON_VMON2_OV_STAT_SHIFT
    }
};

static Pmic_powerPgoodSrcRegCfg_t lp8764x_pgoodSrcRegCfg[] =
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
        PMIC_PGOOD_SEL_4_REGADDR,
        PMIC_PGOOD_SEL_4_PGOOD_SEL_VMON1_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_4_REGADDR,
        PMIC_PGOOD_SEL_4_PGOOD_SEL_VMON2_SHIFT,
    },
};

static Pmic_powerIntCfg_t lp8764x_pwrIntCfg[] =
{
    {
        PMIC_LP8764X_TWARN_INT,
    },
    {
        PMIC_LP8764X_NRSTOUT_READBACK_INT,
    },
    {
        PMIC_LP8764X_SOC_PWR_ERR_INT,
    },
    {
        PMIC_LP8764X_MCU_PWR_ERR_INT,
    },
    {
        PMIC_LP8764X_ORD_SHUTDOWN_INT,
    },
    {
        PMIC_LP8764X_IMM_SHUTOWN_INT,
    },
    {
        PMIC_LP8764X_NRSTOUT_SOC_READBACK_INT,
    },
    {
        PMIC_LP8764X_EN_DRV_READBACK_INT,
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
void pmic_get_lp8764x_pwrCommonIntCfg(Pmic_powerIntCfg_t **pPwrCommonIntCfg)
{
    *pPwrCommonIntCfg = lp8764x_pwrIntCfg;
}

/*!
 * \brief  PMIC power resources get Configuration function
 *         This function is used to read the PMIC POWER resources register
 *         Configuration
 *
 * \param  pwrRsrcRegCfg   [OUT]  Pointer to store power resource register
 *                                configuration
 */
void pmic_get_lp8764x_pwrRsrceRegCfg(Pmic_powerRsrcRegCfg_t **pPwrRsrcRegCfg)
{
    *pPwrRsrcRegCfg = gLp8764x_pwrRsrcRegCfg;
}

/*!
 * \brief  PMIC power get Configuration function
 *         This function is used to read the PMIC pgood sources register
 *         Configuration
 *
 * \param  pPgoodSrcRegCfg   [OUT]  Pointer to store power-good source register
 *                                  configuration
 */
void pmic_get_lp8764x_pwrPgoodSrcRegCfg(
                                   Pmic_powerPgoodSrcRegCfg_t **pPgoodSrcRegCfg)
{
    *pPgoodSrcRegCfg = lp8764x_pgoodSrcRegCfg;
}

/*!
 * \brief   This function is used to convert the millivolt value to vset value
 *          for HERA LP8764x PMIC
 *
 *          Note: In this API, While adding support for New pwrRsrcType/ New
 *                vmonRange, developer need to update the API functionality for
 *                New pwrRsrcType/New vmonRange accordingly.
 */
int32_t Pmic_powerLP8764xConvertVoltage2VSetVal(
                                             Pmic_CoreHandle_t *pPmicCoreHandle,
                                             uint16_t           millivolt,
                                             uint16_t           pwrRsrc,
                                             uint8_t           *pVSetVal)
{
    int32_t  status = PMIC_ST_SUCCESS;
    uint16_t baseMillivolt = 0U;
    uint8_t  millivoltStep = 0U;
    uint8_t  baseVoutCode  = 0U;
    uint8_t  pwrRsrcType;
    bool     vmonRange;


    pwrRsrcType = Pmic_powerGetPwrRsrcType(pwrRsrc);

    switch(pwrRsrcType)
    {
        case PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON:
            status = Pmic_powerGetVmonRange(pPmicCoreHandle,
                                            pwrRsrc,
                                            &(vmonRange));

            if(PMIC_ST_SUCCESS == status)
            {
                if(PMIC_LP8764X_VMON_RANGE_0V3_3V34 == vmonRange)
                {
                    status = Pmic_powerBuckVmonConvertVoltage2VSetVal(
                                                                millivolt,
                                                                &baseMillivolt,
                                                                &millivoltStep,
                                                                &baseVoutCode);
                }
                else
                {
                    /* Else part checking for (PMIC_LP8764X_VMON_RANGE_3V35_5V
                       == vmonRange) */
                    Pmic_powerVmonRange1ConvertVoltage2VSetVal(&baseMillivolt,
                                                               &millivoltStep,
                                                               &baseVoutCode);
                }
            }

            break;
        default:
            /* Default case for BUCK Resource Type */
            status = Pmic_powerBuckVmonConvertVoltage2VSetVal(millivolt,
                                                              &baseMillivolt,
                                                              &millivoltStep,
                                                              &baseVoutCode);

            break;
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
 * \brief   This function is used to convert the vsetvalue to voltage in mv
 *          for PMIC HERA LP8764x
 *
 *          Note: In this API, While adding support for New pwrRsrcType/ New
 *                vmonRange, developer need to update the API functionality for
 *                New pwrRsrcType/New vmonRange accordingly.
 */
int32_t Pmic_powerLP8764xConvertVSetVal2Voltage(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
                                            const uint8_t     *pVSetVal,
                                            uint16_t           pwrRsrc,
                                            uint16_t          *millivolt)
{
    int32_t  status        = PMIC_ST_SUCCESS;
    bool     vmonRange     = (bool)false;
    uint8_t  pwrRsrcType   = 0U;
    uint16_t baseMillivolt = 0U;
    uint8_t  millivoltStep = 0U;
    uint8_t  baseVoutCode  = 0U;

    pwrRsrcType = Pmic_powerGetPwrRsrcType(pwrRsrc);

    switch(pwrRsrcType)
    {
        case PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON:
            status = Pmic_powerGetVmonRange(pPmicCoreHandle,
                                            pwrRsrc,
                                            &(vmonRange));

            if(PMIC_ST_SUCCESS == status)
            {
               if(PMIC_LP8764X_VMON_RANGE_0V3_3V34 == vmonRange)
                {
                    Pmic_powerBuckVmonConvertVSetVal2Voltage(pVSetVal,
                                                            &baseMillivolt,
                                                            &millivoltStep,
                                                            &baseVoutCode);
                }
                else
                {
                    /* Else part checking for (PMIC_LP8764X_VMON_RANGE_3V35_5V
                       == vmonRange) */
                    Pmic_powerVmonRange1ConvertVSetVal2Voltage(&baseMillivolt,
                                                               &millivoltStep,
                                                               &baseVoutCode);
                }
            }

            break;
        default:
            /* Default case for BUCK Resource Type */
            Pmic_powerBuckVmonConvertVSetVal2Voltage(pVSetVal,
                                                     &baseMillivolt,
                                                     &millivoltStep,
                                                     &baseVoutCode);
            break;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        *millivolt = (baseMillivolt +
                     (((uint16_t)*pVSetVal - baseVoutCode) * millivoltStep));
    }

    return status;
}

/*!
 * \brief   This function is to validate the power good source limit for VCCA
 *          NRSTOUT and NRSTOUT_SOC
 */
static int32_t Pmic_validate_lp8764x_pGoodVccaNrstOutNrstOutsocSrcType(
                                                          uint16_t pgoodSrc,
                                                          uint8_t  pGoodSrcType)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_VCCA == pGoodSrcType)
    {
        if(pgoodSrc != PMIC_LP8764X_PGOOD_SOURCE_VCCA)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_NRSTOUT == pGoodSrcType)
    {
        if(pgoodSrc != PMIC_LP8764X_PGOOD_SOURCE_NRSTOUT)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else
    {
        if(pgoodSrc != PMIC_LP8764X_PGOOD_SOURCE_NRSTOUT_SOC)
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
int32_t Pmic_validate_lp8764x_pGoodSrcType(uint16_t pgoodSrc)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t  pGoodSrcType = 0U;

    pGoodSrcType = Pmic_powerGetPwrRsrcType(pgoodSrc);

    if((PMIC_LP8764X_PGOOD_SOURCE_TYPE_VCCA == pGoodSrcType)    ||
       (PMIC_LP8764X_PGOOD_SOURCE_TYPE_NRSTOUT == pGoodSrcType) ||
       (PMIC_LP8764X_PGOOD_SOURCE_TYPE_NRSTOUT_SOC == pGoodSrcType))
       {
           status = Pmic_validate_lp8764x_pGoodVccaNrstOutNrstOutsocSrcType(
                                                                 pgoodSrc,
                                                                 pGoodSrcType);
       }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_TDIE == pGoodSrcType)
    {
        if(pgoodSrc != PMIC_LP8764X_PGOOD_SOURCE_TDIE)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_BUCK == pGoodSrcType)
    {
        if((pgoodSrc > PMIC_LP8764X_PGOOD_BUCK_MAX) ||
           (pgoodSrc < PMIC_LP8764X_PGOOD_BUCK_MIN))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_VMON == pGoodSrcType)
    {
        if((pgoodSrc > PMIC_LP8764X_PGOOD_VMON_MAX) ||
           (pgoodSrc < PMIC_LP8764X_PGOOD_VMON_MIN))
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
static int32_t Pmic_validate_lp8764x_pGoodSelBuckLdoNrstoutNrstoutsoc(
                                                         uint8_t  pgoodSelType,
                                                         uint8_t  pGoodSrcType)
{
    int32_t status = PMIC_ST_SUCCESS;

    if((PMIC_LP8764X_PGOOD_SOURCE_TYPE_VCCA == pGoodSrcType) ||
       (PMIC_LP8764X_PGOOD_SOURCE_TYPE_VMON == pGoodSrcType))
    {
        if(pgoodSelType > PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_ENABLE)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_NRSTOUT == pGoodSrcType)
    {
        if(pgoodSelType > PMIC_LP8764X_POWER_PGOOD_SEL_NRSTOUT)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else
    {
        if(pgoodSelType > PMIC_LP8764X_POWER_PGOOD_SEL_NRSTOUT_SOC)
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
int32_t Pmic_validate_lp8764x_pGoodSelType(uint16_t pgoodSrc,
                                           uint8_t  pgoodSelType)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t  pGoodSrcType = 0U;

    pGoodSrcType = Pmic_powerGetPwrRsrcType(pgoodSrc);

    if((PMIC_LP8764X_PGOOD_SOURCE_TYPE_VCCA == pGoodSrcType)    ||
       (PMIC_LP8764X_PGOOD_SOURCE_TYPE_VMON == pGoodSrcType)    ||
       (PMIC_LP8764X_PGOOD_SOURCE_TYPE_NRSTOUT == pGoodSrcType) ||
       (PMIC_LP8764X_PGOOD_SOURCE_TYPE_NRSTOUT_SOC == pGoodSrcType))
       {
           status = Pmic_validate_lp8764x_pGoodSelBuckLdoNrstoutNrstoutsoc(
                                                                  pgoodSelType,
                                                                  pGoodSrcType);
       }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_TDIE == pGoodSrcType)
    {
        if(pgoodSelType > PMIC_LP8764X_POWER_PGOOD_SEL_TDIE_WARN)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else
    {
        /* Else case for Buck pGoodSrcType */
        if(pgoodSelType > PMIC_LP8764X_POWER_PGOOD_SEL_SRC_VOLTAGE_CURRENT)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

/**
 * \brief   This function is used to validate the voltage levels for
 *          Regulators/VMON for LP8764x PMIC
 *
*          Note: In this API, While adding support for New pwrRsrcType/ New
 *                vmonRange, developer need to update the API functionality for
 *                New pwrRsrcType/New vmonRange accordingly.
 */
int32_t Pmic_powerLP8764xValidateVoltageLevel(
                                             Pmic_CoreHandle_t *pPmicCoreHandle,
                                             uint8_t            pwrRsrcType,
                                             uint16_t           pwrRsrc,
                                             uint16_t           voltage_mV)
{
    int32_t  status = PMIC_ST_SUCCESS;
    bool     vmonRange = (bool)false;

    if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON == pwrRsrcType)
    {
        status = Pmic_powerGetVmonRange(pPmicCoreHandle,
                                        pwrRsrc,
                                        &(vmonRange));

        if((PMIC_ST_SUCCESS == status) &&
           (PMIC_LP8764X_VMON_RANGE_0V3_3V34 == vmonRange))
        {
            if((voltage_mV < PMIC_LP8764X_RANGE0_VMON_MIN_VOLTAGE)  ||
               (voltage_mV > PMIC_LP8764X_RANGE0_VMON_MAX_VOLTAGE))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
        }
        else
        {
            /* Else case for PMIC_LP8764X_VMON_RANGE_3V35_5V */
            if((voltage_mV < PMIC_LP8764X_RANGE1_VMON_MIN_VOLTAGE)   ||
               (voltage_mV > PMIC_LP8764X_RANGE1_VMON_MAX_VOLTAGE))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
        }
    }
    else
    {
        /* Else case for BUCK pwrRsrcType */
        if((voltage_mV < PMIC_LP8764X_REGULATOR_BUCK_MIN_VOLTAGE) ||
           (voltage_mV > PMIC_LP8764X_REGULATOR_BUCK_MAX_VOLTAGE))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

/*!
 * \brief   This function is to validate the power resource limit for the
 *          LP8764x PMIC device.
 */
int32_t Pmic_powerLP8764xValidatePwrRsrcLimit(
                                    const Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint8_t                  pwrRsrcType,
                                    uint16_t                 pwrRsrc)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VCCA == pwrRsrcType)
    {
        if(pwrRsrc != PMIC_LP8764X_POWER_SOURCE_VCCA)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
    {
        if(((bool)false) == pPmicCoreHandle->pPmic_SubSysInfo->buckEnable)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            if((pwrRsrc > PMIC_LP8764X_BUCK_MAX) ||
               (pwrRsrc < PMIC_LP8764X_BUCK_MIN))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
        }
    }
    else if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON == pwrRsrcType)
    {
        if((pwrRsrc > PMIC_LP8764X_VMON_MAX) ||
           (pwrRsrc < PMIC_LP8764X_VMON_MIN))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if((PMIC_LP8764X_POWER_RESOURCE_TYPE_LDO == pwrRsrcType) &&
           (((bool)false) == pPmicCoreHandle->pPmic_SubSysInfo->ldoEnable))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    else
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/*!
 * \brief   This function is to validate the power resource interrupt type
 *          for the LP8764x PMIC device.
 *
 *          Note: In this API, While adding support for New pwrResourceType,
 *                developer need to update the API functionality for New
 *                pwrResourceType accordingly.
 */
int32_t Pmic_powerLP8764xValidateIntrType(uint8_t  pwrResourceType,
                                          uint8_t  intrType)
{
    int32_t status = PMIC_ST_SUCCESS;

    if((PMIC_LP8764X_POWER_RESOURCE_TYPE_VCCA == pwrResourceType) ||
       (PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON == pwrResourceType))
    {
        if((intrType != PMIC_LP8764X_POWER_OV_INT) &&
           (intrType != PMIC_LP8764X_POWER_UV_INT))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else
    {
        /* Else case for BUCK pwrResourceType */
        if((intrType != PMIC_LP8764X_POWER_OV_INT) &&
           (intrType != PMIC_LP8764X_POWER_UV_INT) &&
           (intrType != PMIC_LP8764X_POWER_ILIM_INT))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

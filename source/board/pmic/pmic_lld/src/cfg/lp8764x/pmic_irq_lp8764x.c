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
 * \file   pmic_irq_lp8764x.c
 *
 * \brief  This file contains the LP8764x Hera PMIC Interrupt APIs definitions
 *         and structures.
 *
 */

#include <pmic_irq.h>
#include <pmic_core_priv.h>
#include <pmic_irq_priv.h>
#include <pmic_irq_lp8764x.h>
#include <pmic_irq_lp8764x_priv.h>
#include <pmic_power_priv.h>
#include <pmic_wdg_priv.h>

/* PMIC LP8764x Interrupt Configuration as per Pmic_lp8764x_IrqNum. */
static Pmic_IntrCfg_t gLp8764x_intCfg[] =
{
    {
        PMIC_WD_ERR_STATUS_REGADDR,
        PMIC_WD_ERR_STATUS_WD_RST_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_WD_ERR_STATUS_REGADDR,
        PMIC_WD_ERR_STATUS_WD_FAIL_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_WD_ERR_STATUS_REGADDR,
        PMIC_WD_ERR_STATUS_WD_LONGWIN_TIMEOUT_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_ESM_REGADDR,
        PMIC_INT_ESM_ESM_MCU_RST_INT_SHIFT,
        PMIC_MASK_ESM_REGADDR,
        PMIC_MASK_ESM_ESM_MCU_RST_MASK_SHIFT
    },
    {
        PMIC_INT_ESM_REGADDR,
        PMIC_INT_ESM_ESM_MCU_FAIL_INT_SHIFT,
        PMIC_MASK_ESM_REGADDR,
        PMIC_MASK_ESM_ESM_MCU_FAIL_MASK_SHIFT
    },
    {
        PMIC_INT_ESM_REGADDR,
        PMIC_INT_ESM_ESM_MCU_PIN_INT_SHIFT,
        PMIC_MASK_ESM_REGADDR,
        PMIC_MASK_ESM_ESM_MCU_PIN_MASK_SHIFT
    },
    {
        PMIC_INT_READBACK_ERR_REGADDR,
        PMIC_INT_READBACK_ERR_NRSTOUT_SOC_READBACK_INT_SHIFT,
        PMIC_MASK_READBACK_ERR_REGADDR,
        PMIC_MASK_READBACK_ERR_NRSTOUT_SOC_READBACK_MASK_SHIFT
    },
    {
        PMIC_INT_READBACK_ERR_REGADDR,
        PMIC_INT_READBACK_ERR_EN_DRV_READBACK_INT_SHIFT,
        PMIC_MASK_READBACK_ERR_REGADDR,
        PMIC_MASK_READBACK_ERR_EN_DRV_READBACK_MASK_SHIFT
    },
    {
        PMIC_INT_COMM_ERR_REGADDR,
        PMIC_INT_COMM_ERR_I2C2_ADR_ERR_INT_SHIFT,
        PMIC_MASK_COMM_ERR_REGADDR,
        PMIC_MASK_COMM_ERR_I2C2_ADR_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_COMM_ERR_REGADDR,
        PMIC_INT_COMM_ERR_I2C2_CRC_ERR_INT_SHIFT,
        PMIC_MASK_COMM_ERR_REGADDR,
        PMIC_MASK_COMM_ERR_I2C2_CRC_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_COMM_ERR_REGADDR,
        PMIC_INT_COMM_ERR_COMM_ADR_ERR_INT_SHIFT,
        PMIC_MASK_COMM_ERR_REGADDR,
        PMIC_MASK_COMM_ERR_COMM_ADR_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_COMM_ERR_REGADDR,
        PMIC_INT_COMM_ERR_COMM_CRC_ERR_INT_SHIFT,
        PMIC_MASK_COMM_ERR_REGADDR,
        PMIC_MASK_COMM_ERR_COMM_CRC_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_COMM_ERR_REGADDR,
        PMIC_INT_COMM_ERR_COMM_FRM_ERR_INT_SHIFT,
        PMIC_MASK_COMM_ERR_REGADDR,
        PMIC_MASK_COMM_ERR_COMM_FRM_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_FSM_ERR_REGADDR,
        PMIC_INT_FSM_ERR_SOC_PWR_ERR_INT_SHIFT,
        PMIC_MASK_FSM_ERR_REGADDR,
        PMIC_MASK_FSM_ERR_SOC_PWR_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_FSM_ERR_REGADDR,
        PMIC_INT_FSM_ERR_MCU_PWR_ERR_INT_SHIFT,
        PMIC_MASK_FSM_ERR_REGADDR,
        PMIC_MASK_FSM_ERR_MCU_PWR_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_FSM_ERR_REGADDR,
        PMIC_INT_FSM_ERR_ORD_SHUTDOWN_INT_SHIFT,
        PMIC_MASK_FSM_ERR_REGADDR,
        PMIC_MASK_FSM_ERR_ORD_SHUTDOWN_MASK_SHIFT
    },
    {
        PMIC_INT_FSM_ERR_REGADDR,
        PMIC_INT_FSM_ERR_IMM_SHUTDOWN_INT_SHIFT,
        PMIC_MASK_FSM_ERR_REGADDR,
        PMIC_MASK_FSM_ERR_IMM_SHUTDOWN_MASK_SHIFT
    },
    {
        PMIC_INT_SEVERE_ERR_REGADDR,
        PMIC_INT_SEVERE_ERR_PFSM_ERR_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_SEVERE_ERR_REGADDR,
        PMIC_INT_SEVERE_ERR_VCCA_OVP_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_SEVERE_ERR_REGADDR,
        PMIC_INT_SEVERE_ERR_TSD_IMM_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_NRSTOUT_READBACK_INT_SHIFT,
        PMIC_MASK_MODERATE_ERR_REGADDR,
        PMIC_MASK_MODERATE_ERR_NRSTOUT_READBACK_MASK_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_NINT_READBACK_INT_SHIFT,
        PMIC_MASK_MODERATE_ERR_REGADDR,
        PMIC_MASK_MODERATE_ERR_NINT_READBACK_MASK_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_SPMI_ERR_INT_SHIFT,
        PMIC_MASK_MODERATE_ERR_REGADDR,
        PMIC_MASK_MODERATE_ERR_SPMI_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_REG_CRC_ERR_INT_SHIFT,
        PMIC_MASK_MODERATE_ERR_REGADDR,
        PMIC_MASK_MODERATE_ERR_REG_CRC_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_BIST_FAIL_INT_SHIFT,
        PMIC_MASK_MODERATE_ERR_REGADDR,
        PMIC_MASK_MODERATE_ERR_BIST_FAIL_MASK_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_TSD_ORD_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_MISC_REGADDR,
        PMIC_INT_MISC_TWARN_INT_SHIFT,
        PMIC_MASK_MISC_REGADDR,
        PMIC_MASK_MISC_TWARN_MASK_SHIFT
    },
    {
        PMIC_INT_MISC_REGADDR,
        PMIC_INT_MISC_EXT_CLK_INT_SHIFT,
        PMIC_MASK_MISC_REGADDR,
        PMIC_MASK_MISC_EXT_CLK_MASK_SHIFT
    },
    {
        PMIC_INT_MISC_REGADDR,
        PMIC_INT_MISC_BIST_PASS_INT_SHIFT,
        PMIC_MASK_MISC_REGADDR,
        PMIC_MASK_MISC_BIST_PASS_MASK_SHIFT
    },
    {
        PMIC_INT_STARTUP_REGADDR,
        PMIC_INT_STARTUP_FSD_INT_SHIFT,
        PMIC_MASK_STARTUP_REGADDR,
        PMIC_MASK_STARTUP_FSD_MASK_SHIFT
    },
    {
        PMIC_INT_STARTUP_REGADDR,
        PMIC_INT_STARTUP_ENABLE_INT_SHIFT,
        PMIC_MASK_STARTUP_REGADDR,
        PMIC_MASK_STARTUP_ENABLE_MASK_SHIFT,
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO8_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO7_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO6_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO5_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO4_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO3_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO2_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO1_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO_REGADDR,
        PMIC_INT_GPIO_GPIO10_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO_REGADDR,
        PMIC_INT_GPIO_GPIO9_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_VMON_REGADDR,
        PMIC_INT_VMON_VMON2_RV_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_VMON_REGADDR,
        PMIC_INT_VMON_VMON2_UV_INT_SHIFT,
        PMIC_MASK_VMON_REGADDR,
        PMIC_MASK_VMON_VMON2_UV_MASK_SHIFT
    },
    {
        PMIC_INT_VMON_REGADDR,
        PMIC_INT_VMON_VMON2_OV_INT_SHIFT,
        PMIC_MASK_VMON_REGADDR,
        PMIC_MASK_VMON_VMON2_OV_MASK_SHIFT
    },
    {
        PMIC_INT_VMON_REGADDR,
        PMIC_INT_VMON_VMON1_RV_INT_MASK,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_VMON_REGADDR,
        PMIC_INT_VMON_VMON1_UV_INT_SHIFT,
        PMIC_MASK_VMON_REGADDR,
        PMIC_MASK_VMON_VMON1_UV_MASK_SHIFT
    },
    {
        PMIC_INT_VMON_REGADDR,
        PMIC_INT_VMON_VMON1_OV_INT_SHIFT,
        PMIC_MASK_VMON_REGADDR,
        PMIC_MASK_VMON_VMON1_OV_MASK_SHIFT
    },
    {
        PMIC_INT_VMON_REGADDR,
        PMIC_INT_VMON_VCCA_UV_INT_SHIFT,
        PMIC_MASK_VMON_REGADDR,
        PMIC_MASK_VMON_VCCA_UV_MASK_SHIFT
    },
    {
        PMIC_INT_VMON_REGADDR,
        PMIC_INT_VMON_VCCA_OV_INT_SHIFT,
        PMIC_MASK_VMON_REGADDR,
        PMIC_MASK_VMON_VCCA_OV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK4_ILIM_INT_SHIFT,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_MASK_BUCK3_4_BUCK4_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK4_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK4_UV_INT_SHIFT,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_MASK_BUCK3_4_BUCK4_UV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK4_OV_INT_SHIFT,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_MASK_BUCK3_4_BUCK4_OV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK3_ILIM_INT_SHIFT,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_MASK_BUCK3_4_BUCK3_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK3_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK3_UV_INT_SHIFT,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_MASK_BUCK3_4_BUCK3_UV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK3_OV_INT_SHIFT,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_MASK_BUCK3_4_BUCK3_OV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK2_ILIM_INT_SHIFT,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_MASK_BUCK1_2_BUCK2_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK2_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK2_UV_INT_SHIFT,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_MASK_BUCK1_2_BUCK2_UV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK2_OV_INT_SHIFT,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_MASK_BUCK1_2_BUCK2_OV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK1_ILIM_INT_SHIFT,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_MASK_BUCK1_2_BUCK1_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK1_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK1_UV_INT_SHIFT,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_MASK_BUCK1_2_BUCK1_UV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK1_OV_INT_SHIFT,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_MASK_BUCK1_2_BUCK1_OV_MASK_SHIFT
    },
    {
        PMIC_INT_STARTUP_REGADDR,
        PMIC_INT_STARTUP_SOFT_REBOOT_INT_SHIFT,
        PMIC_MASK_STARTUP_REGADDR,
        PMIC_MASK_STARTUP_SOFT_REBOOT_MASK_SHIFT
    },
};

/*  PMIC LP8764x GPIO Interrupt Mask Configuration as per Pmic_IrqGpioNum. */
static Pmic_GpioIntrTypeCfg_t lp8764x_gpioIntrCfg[] =
{
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO1_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO1_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO2_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO2_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO3_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO3_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO4_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO4_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO5_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO5_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO6_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO6_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO7_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO7_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO8_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO8_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO9_10_REGADDR,
        PMIC_MASK_GPIO9_10_GPIO9_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO9_10_REGADDR,
        PMIC_MASK_GPIO9_10_GPIO9_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO9_10_REGADDR,
        PMIC_MASK_GPIO9_10_GPIO10_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO9_10_REGADDR,
        PMIC_MASK_GPIO9_10_GPIO10_FALL_MASK_SHIFT
    }
};

/*
 * \brief   Get LP8764x Interrupt config.
 *          This function is used to get LP8764x Interrupt configuration.
 *
 * \param   pIntCfg   [OUT]  To store lp8764x Interrupt configuration.
 */
void pmic_get_lp8764x_intrCfg(Pmic_IntrCfg_t **pIntrCfg)
{
    *pIntrCfg = gLp8764x_intCfg;
}

/*
 * \brief   Get LP8764x Interrupt config.
 *          This function is used to get LP8764x Interrupt configuration.
 *
 * \param   pGpioIntrCfg   [OUT]  To store lp8764x Interrupt configuration.
 */
void pmic_get_lp8764x_intrGpioCfg(Pmic_GpioIntrTypeCfg_t **pGpioIntrCfg)
{
    *pGpioIntrCfg = lp8764x_gpioIntrCfg;
}

/*!
 * \brief  Function to decipher BUCK1 and BUCK 1 Error
 */
static int32_t Pmic_lp8764x_getBuck1Buck2Err(
                                       Pmic_CoreHandle_t *pPmicCoreHandle,
                                       Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_INT_BUCK1_2_REGADDR,
                                        &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
    {
        if((regData & PMIC_INT_BUCK1_2_BUCK2_OV_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK2_OV_INT);
        }

        if((regData & PMIC_INT_BUCK1_2_BUCK2_UV_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK2_UV_INT);
        }

        if((regData & PMIC_INT_BUCK1_2_BUCK2_SC_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK2_SC_INT);
        }

        if((regData & PMIC_INT_BUCK1_2_BUCK2_ILIM_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK2_ILIM_INT);
        }

        if((regData & PMIC_INT_BUCK1_2_BUCK1_OV_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK1_OV_INT);
        }

        if((regData & PMIC_INT_BUCK1_2_BUCK1_UV_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK1_UV_INT);
        }

        if((regData & PMIC_INT_BUCK1_2_BUCK1_SC_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK1_SC_INT);
        }

        if((regData & PMIC_INT_BUCK1_2_BUCK1_ILIM_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK1_ILIM_INT);
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Function to decipher BUCK3 and BUCK 4 Error
 */
static int32_t Pmic_lp8764x_getBuck3Buck4Err(
                                       Pmic_CoreHandle_t *pPmicCoreHandle,
                                       Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_INT_BUCK3_4_REGADDR,
                                        &regData);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
    {
        if((regData & PMIC_INT_BUCK3_4_BUCK4_OV_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK4_OV_INT);
        }

        if((regData & PMIC_INT_BUCK3_4_BUCK4_UV_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK4_UV_INT);
        }

        if((regData & PMIC_INT_BUCK3_4_BUCK4_SC_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK4_SC_INT);
        }

        if((regData & PMIC_INT_BUCK3_4_BUCK4_ILIM_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK4_ILIM_INT);
        }

        if((regData & PMIC_INT_BUCK3_4_BUCK3_OV_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK3_OV_INT);
        }

        if((regData & PMIC_INT_BUCK3_4_BUCK3_UV_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK3_UV_INT);
        }

        if((regData & PMIC_INT_BUCK3_4_BUCK3_SC_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK3_SC_INT);
        }

        if((regData & PMIC_INT_BUCK3_4_BUCK3_ILIM_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BUCK3_ILIM_INT);
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Function to decipher BUCK Error
 */
static int32_t Pmic_lp8764x_getBuckErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                       uint8_t            regValue,
                                       Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* PMIC BUCK3_4 Interrupt Status Check */
    if((regValue & PMIC_INT_BUCK_BUCK3_4_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_lp8764x_getBuck3Buck4Err(pPmicCoreHandle, pErrStat);
    }

    /* PMIC BUCK1_2 Interrupt Status Check */
    if((regValue & PMIC_INT_BUCK_BUCK1_2_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_lp8764x_getBuck1Buck2Err(pPmicCoreHandle, pErrStat);
    }

    return pmicStatus;
}

/*!
 * \brief  Function to decipher VMON Error
 */
static void Pmic_lp8764x_getVmonErr(uint8_t            regValue,
                                    Pmic_IrqStatus_t  *pErrStat)
{
    if((regValue & PMIC_INT_VMON_VCCA_OV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_VCCA_OV_INT);
    }

    if((regValue & PMIC_INT_VMON_VCCA_UV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_VCCA_UV_INT);
    }

    if((regValue & PMIC_INT_VMON_VMON1_OV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_VMON1_OV_INT);
    }

    if((regValue & PMIC_INT_VMON_VMON1_UV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_VMON1_UV_INT);
    }

    if((regValue & PMIC_INT_VMON_VMON1_RV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_VMON1_RV_INT);
    }

    if((regValue & PMIC_INT_VMON_VMON2_OV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_VMON2_OV_INT);
    }

    if((regValue & PMIC_INT_VMON_VMON2_UV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_VMON2_UV_INT);
    }

    if((regValue & PMIC_INT_VMON_VMON2_RV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_VMON2_RV_INT);
    }
}

/*!
 * \brief  Function to get GPIO1 to GPIO8 Error
 */
static void Pmic_lp8764x_getGpio1ToGpio8Err(uint8_t            regData,
                                            Pmic_IrqStatus_t  *pErrStat)
{
    if((regData & PMIC_INT_GPIO1_8_GPIO1_INT_MASK) != 0U)
    {
       Pmic_intrBitSet(pErrStat, PMIC_LP8764X_GPIO1_INT);
    }

    if((regData & PMIC_INT_GPIO1_8_GPIO2_INT_MASK) != 0U)
    {
       Pmic_intrBitSet(pErrStat, PMIC_LP8764X_GPIO2_INT);
    }

    if((regData & PMIC_INT_GPIO1_8_GPIO3_INT_MASK) != 0U)
    {
       Pmic_intrBitSet(pErrStat, PMIC_LP8764X_GPIO3_INT);
    }

    if((regData & PMIC_INT_GPIO1_8_GPIO4_INT_MASK) != 0U)
    {
       Pmic_intrBitSet(pErrStat, PMIC_LP8764X_GPIO4_INT);
    }

    if((regData & PMIC_INT_GPIO1_8_GPIO5_INT_MASK) != 0U)
    {
       Pmic_intrBitSet(pErrStat, PMIC_LP8764X_GPIO5_INT);
    }

    if((regData & PMIC_INT_GPIO1_8_GPIO6_INT_MASK) != 0U)
    {
       Pmic_intrBitSet(pErrStat, PMIC_LP8764X_GPIO6_INT);
    }

    if((regData & PMIC_INT_GPIO1_8_GPIO7_INT_MASK) != 0U)
    {
       Pmic_intrBitSet(pErrStat, PMIC_LP8764X_GPIO7_INT);
    }

    if((regData & PMIC_INT_GPIO1_8_GPIO8_INT_MASK) != 0U)
    {
       Pmic_intrBitSet(pErrStat, PMIC_LP8764X_GPIO8_INT);
    }

}

/*!
 * \brief  Function to decipher GPIO Error
 */
static int32_t Pmic_lp8764x_getGpioErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                       uint8_t            regValue,
                                       Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    /* Checking GPIO1_8 Bit field for INT_GPIO Register */
    if((regValue & PMIC_INT_GPIO_GPIO1_8_INT_MASK) != 0U)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_GPIO1_8_REGADDR,
                                            &regData);
        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);

       if((PMIC_ST_SUCCESS == pmicStatus) &&
          (0U != regData))
       {
           Pmic_lp8764x_getGpio1ToGpio8Err(regData, pErrStat);
       }
    }

    if((regValue & PMIC_INT_GPIO_GPIO9_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_GPIO9_INT);
    }

    if((regValue & PMIC_INT_GPIO_GPIO10_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_GPIO10_INT);
    }

    return pmicStatus;
}

/*!
 * \brief  Function to decipher STARTUP Error
 */
static int32_t Pmic_lp8764x_getStartupErr(
                                       const Pmic_CoreHandle_t *pPmicCoreHandle,
                                       uint8_t                  regValue,
                                       Pmic_IrqStatus_t        *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if((regValue & PMIC_INT_STARTUP_ENABLE_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_ENABLE_INT);
    }

    if((regValue & PMIC_INT_STARTUP_FSD_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_FSD_INT);
    }

    if(PMIC_SILICON_REV_ID_PG_2_0 ==  pPmicCoreHandle->pmicDevSiliconRev)
    {
        if((regValue & PMIC_INT_STARTUP_SOFT_REBOOT_INT_MASK) != 0U)
        {
            Pmic_intrBitSet(pErrStat, PMIC_LP8764X_SOFT_REBOOT_INT);
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Function to decipher MISC Error
 */
static void Pmic_lp8764x_getMiscErr(uint8_t            regValue,
                                    Pmic_IrqStatus_t  *pErrStat)
{
    if((regValue & PMIC_INT_MISC_BIST_PASS_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BIST_PASS_INT);
    }

    if((regValue & PMIC_INT_MISC_EXT_CLK_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_EXT_CLK_INT);
    }

    if((regValue & PMIC_INT_MISC_TWARN_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_TWARN_INT);
    }
}

/*!
 * \brief  Function to decipher MODERATE Error
 */
static void Pmic_lp8764x_getModerateErr(uint8_t            regValue,
                                        Pmic_IrqStatus_t  *pErrStat)
{
    if((regValue & PMIC_INT_MODERATE_ERR_TSD_ORD_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_TSD_ORD_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_BIST_FAIL_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_BIST_FAIL_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_REG_CRC_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_REG_CRC_ERR_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_RECOV_CNT_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_SPMI_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_SPMI_ERR_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_NINT_READBACK_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_NINT_READBACK_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_NRSTOUT_READBACK_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_NRSTOUT_READBACK_INT);
    }
}

/*!
 * \brief  Function to decipher SEVERE Error
 */
static void Pmic_lp8764x_getSevereErr(uint8_t            regValue,
                                      Pmic_IrqStatus_t  *pErrStat)
{
    if((regValue & PMIC_INT_SEVERE_ERR_TSD_IMM_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_TSD_IMM_INT);
    }

    if((regValue & PMIC_INT_SEVERE_ERR_VCCA_OVP_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_VCCA_OVP_INT);
    }

    if((regValue & PMIC_INT_SEVERE_ERR_PFSM_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_PFSM_ERR_INT);
    }
}

/*!
 * \brief  Function to decipher FSM - Communication Error
 */
static void Pmic_lp8764x_getFsmCommErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                       uint8_t            regValue,
                                       Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    if((regValue & PMIC_INT_FSM_ERR_COMM_ERR_INT_MASK) != 0U)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_COMM_ERR_REGADDR,
                                            &regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if((regData & PMIC_INT_COMM_ERR_COMM_FRM_ERR_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_LP8764X_COMM_FRM_ERR_INT);
            }

            if((regData & PMIC_INT_COMM_ERR_COMM_CRC_ERR_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_LP8764X_COMM_CRC_ERR_INT);
            }

            if((regData & PMIC_INT_COMM_ERR_COMM_ADR_ERR_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_LP8764X_COMM_ADR_ERR_INT);
            }

            if((regData & PMIC_INT_COMM_ERR_I2C2_CRC_ERR_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_LP8764X_I2C2_CRC_ERR_INT);
            }

            if((regData & PMIC_INT_COMM_ERR_I2C2_ADR_ERR_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_LP8764X_I2C2_ADR_ERR_INT);
            }
        }
    }
}

/*!
 * \brief  Function to decipher FSM - Readback, ESM Error
 */
static void Pmic_lp8764x_getFsmReadbackEsmErr(
                                      Pmic_CoreHandle_t *pPmicCoreHandle,
                                      uint8_t            regValue,
                                      Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if((regValue & PMIC_INT_FSM_ERR_READBACK_ERR_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_READBACK_ERR_REGADDR,
                                            &regData);

        if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if((regData & PMIC_INT_READBACK_ERR_EN_DRV_READBACK_INT_MASK)
                != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_LP8764X_EN_DRV_READBACK_INT);
            }

            if((regData & PMIC_INT_READBACK_ERR_NRSTOUT_SOC_READBACK_INT_MASK)
                != 0U)
            {
                Pmic_intrBitSet(pErrStat,
                                PMIC_LP8764X_NRSTOUT_SOC_READBACK_INT);
            }
        }
    }

    if((regValue & PMIC_INT_FSM_ERR_ESM_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_ESM_REGADDR,
                                            &regData);

        if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if((regData & PMIC_INT_ESM_ESM_MCU_PIN_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_LP8764X_ESM_MCU_PIN_INT);
            }

            if((regData & PMIC_INT_ESM_ESM_MCU_FAIL_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_LP8764X_ESM_MCU_FAIL_INT);
            }

            if((regData & PMIC_INT_ESM_ESM_MCU_RST_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_LP8764X_ESM_MCU_RST_INT);
            }
        }
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);
}

/*!
 * \brief  Function to decipher FSM Error
 */
static int32_t Pmic_lp8764x_getFSMErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      uint8_t            regValue,
                                      Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    if((regValue & PMIC_INT_FSM_ERR_IMM_SHUTDOWN_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_IMM_SHUTOWN_INT);
    }

    if((regValue & PMIC_INT_FSM_ERR_ORD_SHUTDOWN_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_ORD_SHUTDOWN_INT);
    }

    if((regValue & PMIC_INT_FSM_ERR_MCU_PWR_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_MCU_PWR_ERR_INT);
    }

    if((regValue & PMIC_INT_FSM_ERR_SOC_PWR_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_LP8764X_SOC_PWR_ERR_INT);
    }

    Pmic_lp8764x_getFsmCommErr(pPmicCoreHandle, regValue, pErrStat);

    Pmic_lp8764x_getFsmReadbackEsmErr(pPmicCoreHandle, regValue, pErrStat);

    if((regValue & PMIC_INT_FSM_ERR_WD_INT_MASK) != 0U)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_WD_ERR_STATUS_REGADDR,
                                            &regData);
        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((regData & PMIC_INT_WD_ERR_MASK) != 0U))
        {
            if((regData & PMIC_WD_ERR_STATUS_WD_RST_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_LP8764X_WD_RST_INT);
            }

            if((regData & PMIC_WD_ERR_STATUS_WD_FAIL_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_LP8764X_WD_FAIL_INT);
            }

            if((regData & PMIC_WD_ERR_STATUS_WD_LONGWIN_TIMEOUT_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_LP8764X_WD_LONGWIN_TIMEOUT_INT);
            }
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Function to decipher the the Startup, Miscellaneous, Moderate,
 *         Severe, FSM Error
 */
static int32_t Pmic_lp8764x_irqGetStartupMiscModerateSevereFsmErr(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
                                            Pmic_IrqStatus_t  *pErrStat,
                                            uint16_t           l1RegAddr,
                                            uint8_t            regValue)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;

    switch(l1RegAddr)
    {
        case PMIC_INT_STARTUP_REGADDR:
            pmicStatus = Pmic_lp8764x_getStartupErr(pPmicCoreHandle,
                                                    regValue,
                                                    pErrStat);
            break;

        case PMIC_INT_MISC_REGADDR:
            Pmic_lp8764x_getMiscErr(regValue, pErrStat);
            break;

        case PMIC_INT_MODERATE_ERR_REGADDR:
            Pmic_lp8764x_getModerateErr(regValue,
                                        pErrStat);
            break;

        case PMIC_INT_SEVERE_ERR_REGADDR:
            Pmic_lp8764x_getSevereErr(regValue,
                                      pErrStat);
            break;

        default:
            pmicStatus = Pmic_lp8764x_getFSMErr(pPmicCoreHandle,
                                                regValue,
                                                pErrStat);
            break;
    }

    return pmicStatus;
}

/*!
 * \brief  Function to decipher the L2 Error for LP8764x Hera PMIC
 */
int32_t Pmic_lp8764x_irqGetL2Error(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   uint16_t           l1RegAddr,
                                   Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regValue    = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Read the L1 register value */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        l1RegAddr,
                                        &regValue);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        switch(l1RegAddr)
        {
            case PMIC_INT_BUCK_REGADDR:
                pmicStatus = Pmic_lp8764x_getBuckErr(pPmicCoreHandle,
                                                     regValue,
                                                     pErrStat);
                break;

            case PMIC_INT_VMON_REGADDR:
                Pmic_lp8764x_getVmonErr(regValue,
                                        pErrStat);
                break;

            case PMIC_INT_GPIO_REGADDR:
                pmicStatus = Pmic_lp8764x_getGpioErr(pPmicCoreHandle,
                                                     regValue,
                                                     pErrStat);
                break;

            case PMIC_INT_STARTUP_REGADDR:
            case PMIC_INT_MISC_REGADDR:
            case PMIC_INT_MODERATE_ERR_REGADDR:
            case PMIC_INT_SEVERE_ERR_REGADDR:
            case PMIC_INT_FSM_ERR_REGADDR:
                pmicStatus = Pmic_lp8764x_irqGetStartupMiscModerateSevereFsmErr(
                                                              pPmicCoreHandle,
                                                              pErrStat,
                                                              l1RegAddr,
                                                              regValue);
                break;

            default:
                pmicStatus = PMIC_ST_ERR_INV_INT;
                break;
        }
    }

    return pmicStatus;
}

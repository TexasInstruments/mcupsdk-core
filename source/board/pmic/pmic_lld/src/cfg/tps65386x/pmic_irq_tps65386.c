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
 * @file   pmic_irq_tps65386.c
 *
 * @brief  This file contains the TPS65386 BB(Black-Bird) PMIC Interrupt APIs
 * definitions and structures.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_core_priv.h"

#include "pmic_irq.h"

#include "pmic_irq_priv.h"

#include "pmic_irq_tps65386x.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* PMIC TPS65386x Interrupt Configuration as per Pmic_tps65386x_IrqNum. */
Pmic_IntrCfg_t gTps65386x_intCfg[] = {
    {
        PMIC_CM_VMON_INT_CFG_REGADDR,
        PMIC_COMP1P_UV_INT_CFG_SHIFT,
        PMIC_CM_VMON_INT_MASK_REGADDR,
        PMIC_COMP1P_UV_INT_MASK_SHIFT
    },
    {
        PMIC_CM_VMON_INT_CFG_REGADDR,
        PMIC_COMP1P_OV_INT_CFG_SHIFT,
        PMIC_CM_VMON_INT_MASK_REGADDR,
        PMIC_COMP1P_OV_INT_MASK_SHIFT
    },
    {
        PMIC_CM_VMON_INT_CFG_REGADDR,
        PMIC_COMP1N_UV_INT_CFG_SHIFT,
        PMIC_CM_VMON_INT_MASK_REGADDR,
        PMIC_COMP1N_UV_INT_MASK_SHIFT
    },
    {
        PMIC_CM_VMON_INT_CFG_REGADDR,
        PMIC_COMP1N_OV_INT_CFG_SHIFT,
        PMIC_CM_VMON_INT_MASK_REGADDR,
        PMIC_COMP1N_OV_INT_MASK_SHIFT
    },
    {
        PMIC_CM_VMON_INT_CFG_REGADDR,
        PMIC_COMP2P_UV_INT_CFG_SHIFT,
        PMIC_CM_VMON_INT_MASK_REGADDR,
        PMIC_COMP2P_UV_INT_MASK_SHIFT
    },
    {
        PMIC_CM_VMON_INT_CFG_REGADDR,
        PMIC_COMP2P_OV_INT_CFG_SHIFT,
        PMIC_CM_VMON_INT_MASK_REGADDR,
        PMIC_COMP2P_OV_INT_MASK_SHIFT
    },
    {
        PMIC_CM_VMON_INT_CFG_REGADDR,
        PMIC_COMP2N_UV_INT_CFG_SHIFT,
        PMIC_CM_VMON_INT_MASK_REGADDR,
        PMIC_COMP2N_UV_INT_MASK_SHIFT
    },
    {
        PMIC_CM_VMON_INT_CFG_REGADDR,
        PMIC_COMP2N_OV_INT_CFG_SHIFT,
        PMIC_CM_VMON_INT_MASK_REGADDR,
        PMIC_COMP2N_OV_INT_MASK_SHIFT
    },

    {
        PMIC_CM_COMP_INT_MSKCFG_REGADDR,
        PMIC_COMP1_INT_CFG_SHIFT,
        PMIC_CM_COMP_INT_MSKCFG_REGADDR,
        NULL
    },
    {
        PMIC_CM_COMP_INT_MSKCFG_REGADDR,
        PMIC_COMP2_INT_CFG_SHIFT,
        PMIC_CM_COMP_INT_MSKCFG_REGADDR,
        PMIC_COMP2_INT_MASK_SHIFT
    },

    {
        PMIC_ESM_CFG_REGADDR,
        PMIC_ESM_CFG1_ESM_CFG_SHIFT,
        PMIC_ESM_INT_CFG_REGADDR,
        PMIC_ESM_INT_MASK_SHIFT
    },
    {
        PMIC_ESM_INT_CFG_REGADDR,
        PMIC_ESM_DLY1_INT_CFG_SHIFT,
        PMIC_ESM_INT_CFG_REGADDR,
        PMIC_ESM_DLY1_INT_MASK_SHIFT
    },
    {
        PMIC_ESM_INT_CFG_REGADDR,
        PMIC_ESM_DLY2_INT_CFG_SHIFT,
        PMIC_ESM_INT_CFG_REGADDR,
        PMIC_ESM_DLY2_INT_MASK_SHIFT
    },

    {
        PMIC_WDG_INT_CFG_REGADDR,
        PMIC_WD_TH1_INT_CFG_SHIFT,
        PMIC_WDG_INT_CFG_REGADDR,
        PMIC_WD_TH1_INT_MASK_SHIFT
    },
    {
        PMIC_WDG_INT_CFG_REGADDR,
        PMIC_WD_TH2_INT_CFG_SHIFT,
        PMIC_WDG_INT_CFG_REGADDR,
        PMIC_WD_TH2_INT_MASK_SHIFT
    },

    {
        PMIC_UV_DCDC_CFG_REGADDR,
        PMIC_BB_UV_INT_CFG_SHIFT,
        PMIC_UV_DCDC_CFG_REGADDR,
        PMIC_BB_UV_INT_MASK_SHIFT
    },

    {
        PMIC_UV_INT_CFG2_REGADDR,
        PMIC_PLDO1_UV_INT_CFG_SHIFT,
        PMIC_UV_INT_MASK_REGADDR,
        PMIC_PLDO1_UV_INT_MASK_SHIFT
    },
    {
        PMIC_UV_INT_CFG2_REGADDR,
        PMIC_PLDO2_UV_INT_CFG_SHIFT,
        PMIC_UV_INT_MASK_REGADDR,
        PMIC_PLDO2_UV_INT_MASK_SHIFT
    },
    {
        PMIC_UV_INT_CFG2_REGADDR,
        PMIC_EXT_VMON1_UV_INT_CFG_SHIFT,
        PMIC_UV_INT_MASK_REGADDR,
        PMIC_EXTVMON1_UV_INT_MASK_SHIFT
    },
    {
        PMIC_UV_INT_CFG2_REGADDR,
        PMIC_EXT_VMON2_UV_INT_CFG_SHIFT,
        PMIC_UV_INT_MASK_REGADDR,
        PMIC_EXTVMON2_UV_INT_MASK_SHIFT
    },

    {
        PMIC_UV_INT_CFG1_REGADDR,
        PMIC_LDO1_UV_INT_CFG_SHIFT,
        PMIC_UV_INT_MASK_REGADDR,
        PMIC_LDO1_UV_INT_MASK_SHIFT
    },
    {
        PMIC_UV_INT_CFG1_REGADDR,
        PMIC_LDO2_UV_INT_CFG_SHIFT,
        PMIC_UV_INT_MASK_REGADDR,
        PMIC_LDO2_UV_INT_MASK_SHIFT
    },
    {
        PMIC_UV_INT_CFG1_REGADDR,
        PMIC_LDO3_UV_INT_CFG_SHIFT,
        PMIC_UV_INT_MASK_REGADDR,
        PMIC_LDO3_UV_INT_MASK_SHIFT
    },
    {
        PMIC_UV_INT_CFG1_REGADDR,
        PMIC_LDO4_UV_INT_CFG_SHIFT,
        PMIC_UV_INT_MASK_REGADDR,
        PMIC_LDO4_UV_INT_MASK_SHIFT
    },

    {
        PMIC_OV_DCDC_CFG_REGADDR,
        PMIC_BB_OV_INT_CFG_SHIFT,
        PMIC_OV_DCDC_CFG_REGADDR,
        PMIC_BB_OV_INT_MASK_SHIFT
    },

    {
        PMIC_OV_INT_CFG2_REGADDR,
        PMIC_PLDO1_OV_INT_CFG_SHIFT,
        PMIC_OV_INT_MASK_REGADDR,
        PMIC_PLDO1_OV_INT_MASK_SHIFT
    },
    {
        PMIC_OV_INT_CFG2_REGADDR,
        PMIC_PLDO2_OV_INT_CFG_SHIFT,
        PMIC_OV_INT_MASK_REGADDR,
        PMIC_PLDO2_OV_INT_MASK_SHIFT
    },
    {
        PMIC_OV_INT_CFG2_REGADDR,
        PMIC_EXT_VMON1_OV_INT_CFG_SHIFT,
        PMIC_OV_INT_MASK_REGADDR,
        PMIC_EXTVMON1_OV_INT_MASK_SHIFT
    },
    {
        PMIC_OV_INT_CFG2_REGADDR,
        PMIC_EXT_VMON2_OV_INT_CFG_SHIFT,
        PMIC_OV_INT_MASK_REGADDR,
        PMIC_EXTVMON2_OV_INT_MASK_SHIFT
    },

    {
        PMIC_OV_INT_CFG1_REGADDR,
        PMIC_LDO1_OV_INT_CFG_SHIFT,
        PMIC_OV_INT_MASK_REGADDR,
        PMIC_LDO1_OV_INT_MASK_SHIFT
    },
    {
        PMIC_OV_INT_CFG1_REGADDR,
        PMIC_LDO2_OV_INT_CFG_SHIFT,
        PMIC_OV_INT_MASK_REGADDR,
        PMIC_LDO2_OV_INT_MASK_SHIFT
    },
    {
        PMIC_OV_INT_CFG1_REGADDR,
        PMIC_LDO3_OV_INT_CFG_SHIFT,
        PMIC_OV_INT_MASK_REGADDR,
        PMIC_LDO3_OV_INT_MASK_SHIFT
    },
    {
        PMIC_OV_INT_CFG1_REGADDR,
        PMIC_LDO4_OV_INT_CFG_SHIFT,
        PMIC_OV_INT_MASK_REGADDR,
        PMIC_LDO4_OV_INT_MASK_SHIFT
    },

    {
        PMIC_OFF_STATE_STAT1_REGADDR,
        PMIC_OFF_INT_EVT_ERR_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },

    {
        PMIC_RDBK_INT_CFG2_REGADDR,
        PMIC_GPO1_RDBK_INT_CFG_SHIFT,
        PMIC_RDBK_INT_MASK_REGADDR,
        PMIC_GPO1_RDBK_INT_MASK_SHIFT
    },
    {
        PMIC_RDBK_INT_CFG2_REGADDR,
        PMIC_GPO2_RDBK_INT_CFG_SHIFT,
        PMIC_RDBK_INT_MASK_REGADDR,
        PMIC_GPO2_RDBK_INT_MASK_SHIFT
    },
    {
        PMIC_RDBK_INT_CFG2_REGADDR,
        PMIC_GPO3_RDBK_INT_CFG_SHIFT,
        PMIC_RDBK_INT_MASK_REGADDR,
        PMIC_GPO3_RDBK_INT_MASK_SHIFT
    },
    {
        PMIC_RDBK_INT_CFG2_REGADDR,
        PMIC_GPO4_RDBK_INT_CFG_SHIFT,
        PMIC_RDBK_INT_MASK_REGADDR,
        PMIC_GPO4_RDBK_INT_MASK_SHIFT
    },

    {
        PMIC_RDBK_INT_CFG1_REGADDR,
        PMIC_NRST_RDBK_INT_CFG_SHIFT,
        PMIC_RDBK_INT_MASK_REGADDR,
        PMIC_NRST_RDBK_INT_MASK_SHIFT
    },
    {
        PMIC_RDBK_INT_CFG1_REGADDR,
        PMIC_SFOUT1_RDBK_INT_CFG_SHIFT,
        PMIC_RDBK_INT_MASK_REGADDR,
        PMIC_SFOUT1_RDBK_INT_MASK_SHIFT
    },
    {
        PMIC_RDBK_INT_CFG1_REGADDR,
        PMIC_EN_OUT_RDBK_INT_CFG_SHIFT,
        PMIC_RDBK_INT_MASK_REGADDR,
        PMIC_EN_OUT_RDBK_INT_MASK_SHIFT
    },

    {
        PMIC_SAFETY_CFG_REGADDR,
        PMIC_CFG_REG_CRC_INT_CFG_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
};

/*  PMIC TPS65386x GPIO Interrupt Mask Configuration as per
 *  Pmic_tps65386x_IrqGpioNum.
 */
Pmic_GpioIntrTypeCfg_t tps65386x_gpioIntrCfg[] = {
    {
        PMIC_RDBK_INT_MASK_REGADDR,
        PMIC_GPO1_RDBK_INT_MASK_SHIFT
    },
    {
        PMIC_RDBK_INT_MASK_REGADDR,
        PMIC_GPO2_RDBK_INT_MASK_SHIFT
    },
    {
        PMIC_RDBK_INT_MASK_REGADDR,
        PMIC_GPO3_RDBK_INT_MASK_SHIFT
    },
    {
        PMIC_RDBK_INT_MASK_REGADDR,
        PMIC_GPO4_RDBK_INT_MASK_SHIFT
    },
};

/**
 * @brief   Get TPS65386x Interrupt config.
 *          This function is used to get TPS65386x Interrupt configuration.
 *
 * @param   pIntrCfg to store tps65386x Interrupt configuration.
 */
void pmic_get_bb_intrCfg(Pmic_IntrCfg_t ** pIntrCfg) {
    * pIntrCfg = gTps65386x_intCfg;
}

/**
 * @brief   Get TPS65386x Interrupt config.
 *          This function is used to get TPS65386x Interrupt configuration.
 *
 * @param   pGpioIntrCfg to store tps65386x Interrupt configuration.
 */
void pmic_get_bb_intrGpioCfg(Pmic_GpioIntrTypeCfg_t ** pGpioIntrCfg) {
    * pGpioIntrCfg = tps65386x_gpioIntrCfg;
}

/**
 * @brief  Function to CHECK CM VMON Error
 */
static int32_t Pmic_bb_getCMVMONErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_CM_VMON_INT_CFG_REGADDR, & regData);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData)) {
        if ((regData & PMIC_COMP1P_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_COMP1P_UV_INT_CFG);
        }

        if ((regData & PMIC_COMP1P_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_COMP1P_OV_INT_CFG);
        }

        if ((regData & PMIC_COMP1N_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_COMP1N_UV_INT_CFG);
        }

        if ((regData & PMIC_COMP1N_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_COMP1N_OV_INT_CFG);
        }
        if ((regData & PMIC_COMP2P_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_COMP2P_UV_INT_CFG);
        }

        if ((regData & PMIC_COMP2P_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_COMP2P_OV_INT_CFG);
        }

        if ((regData & PMIC_COMP2N_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_COMP2N_UV_INT_CFG);
        }

        if ((regData & PMIC_COMP2N_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_COMP2N_OV_INT_CFG);
        }
    }

    return pmicStatus;
}

/**
 * @brief  Function to CHECK COMP Error
 */
static int32_t Pmic_bb_getCOMPErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(
        pPmicCoreHandle, PMIC_CM_COMP_INT_MSKCFG_REGADDR, & regData);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData)) {
        if ((regData & PMIC_COMP1_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_COMP1_INT_CFG);
        }

        if ((regData & PMIC_COMP2_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_COMP2_INT_CFG);
        }
    }

    return pmicStatus;
}

/**
 * @brief  Function to CHECK ESM Error
 */
static int32_t Pmic_bb_getESMErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ESM_INT_CFG_REGADDR, &
        regData);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData)) {
        if ((regData & PMIC_ESM_DLY1_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_ESM_DLY1_INT_CFG);
        }

        if ((regData & PMIC_ESM_DLY2_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_ESM_DLY2_INT_CFG);
        }
    }

    return pmicStatus;
}

/**
 * @brief  Function to decipher WDG Error
 */
static int32_t Pmic_bb_getWDGErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WDG_INT_CFG_REGADDR, &
        regData);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData)) {
        if ((regData & PMIC_WD_TH1_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_WD_TH1_INT_CFG);
        }

        if ((regData & PMIC_WD_TH2_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_WD_TH2_INT_CFG);
        }
    }

    return pmicStatus;
}

/**
 * @brief  Function to Check PMIC LDO UV Error
 */
static int32_t Pmic_bb_getUV_LDO_Err(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat) {

    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_UV_INT_CFG1_REGADDR, &
        regData);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData)) {
        if ((regData & PMIC_LDO1_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_LDO1_UV_INT_CFG);
        }

        if ((regData & PMIC_LDO2_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_LDO2_UV_INT_CFG);
        }

        if ((regData & PMIC_LDO3_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_LDO3_UV_INT_CFG);
        }

        if ((regData & PMIC_LDO4_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_LDO4_UV_INT_CFG);
        }
    }

    return pmicStatus;
}

/**
 * @brief  Function to Check PMIC UV VMON and PLDO Error
 */
static int32_t Pmic_bb_getUV_VMONPLDOErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat) {

    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_UV_INT_CFG2_REGADDR, &
        regData);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData)) {
        if ((regData & PMIC_PLDO1_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_PLDO1_UV_INT_CFG);
        }

        if ((regData & PMIC_PLDO2_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_PLDO2_UV_INT_CFG);
        }

        if ((regData & PMIC_EXT_VMON1_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_EXT_VMON1_UV_INT_CFG);
        }

        if ((regData & PMIC_EXT_VMON2_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_EXT_VMON2_UV_INT_CFG);
        }
    }

    return pmicStatus;
}

/**
 * @brief  Function to decipher UV LDO VMON Error
 */
static int32_t
Pmic_bb_getUV_LDO_VMON_PLDOErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t regValue, Pmic_IrqStatus_t * pErrStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_UV_DCDC_CFG_REGADDR, &
        regData);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData)) {
        if ((regData & PMIC_BB_UV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_BB_UV_INT_CFG);
        }
    }

    /* PMIC UV LDO Interrupt Status Check */
    if (regValue != 0U) {
        pmicStatus = Pmic_bb_getUV_LDO_Err(pPmicCoreHandle, pErrStat);
    }

    /* PMIC PLDO VMON Interrupt Status Check */
    if (regValue != 0U) {
        pmicStatus = Pmic_bb_getUV_VMONPLDOErr(pPmicCoreHandle, pErrStat);
    }

    return pmicStatus;
}

/**
 * @brief  Function to Check PMIC LDO OV Error
 */
static int32_t Pmic_bb_getOV_LDO_Err(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat) {

    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_OV_INT_CFG1_REGADDR, &
        regData);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData)) {
        if ((regData & PMIC_LDO1_OV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_LDO1_OV_INT_CFG);
        }

        if ((regData & PMIC_LDO2_OV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_LDO2_OV_INT_CFG);
        }

        if ((regData & PMIC_LDO3_OV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_LDO3_OV_INT_CFG);
        }

        if ((regData & PMIC_LDO4_OV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_LDO4_OV_INT_CFG);
        }
    }

    return pmicStatus;
}

/**
 * @brief  Function to Check PMIC VMON and PLDO Error
 */
static int32_t Pmic_bb_getOV_VMONPLDOErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_IrqStatus_t * pErrStat) {

    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_OV_INT_CFG2_REGADDR, &
        regData);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData)) {
        if ((regData & PMIC_PLDO1_OV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_PLDO1_OV_INT_CFG);
        }

        if ((regData & PMIC_PLDO2_OV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_PLDO2_OV_INT_CFG);
        }

        if ((regData & PMIC_EXT_VMON1_OV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_EXT_VMON1_OV_INT_CFG);
        }

        if ((regData & PMIC_EXT_VMON2_OV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_EXT_VMON2_OV_INT_CFG);
        }
    }

    return pmicStatus;
}

/**
 * @brief  Function to decipher LDO VMON Error
 */
static int32_t
Pmic_bb_getOV_LDO_VMON_PLDOErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t regValue, Pmic_IrqStatus_t * pErrStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_OV_DCDC_CFG_REGADDR, &
        regData);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData)) {
        if ((regData & PMIC_BB_OV_INT_CFG_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_BB_OV_INT_CFG);
        }
    }

    /* PMIC OV LDO Interrupt Status Check */
    if (regValue != 0U) {
        pmicStatus = Pmic_bb_getOV_LDO_Err(pPmicCoreHandle, pErrStat);
    }

    /* PMIC PLDO VMON Interrupt Status Check */
    if (regValue != 0U) {
        pmicStatus = Pmic_bb_getOV_VMONPLDOErr(pPmicCoreHandle, pErrStat);
    }

    return pmicStatus;
}

/**
 * @brief  Function to check SafeOut, En_Out, NRST and GPO Readback  Error
 */
static int32_t Pmic_bb_getRDBKErr(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t regValue,
    Pmic_IrqStatus_t * pErrStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if ((regValue & PMIC_RDBK_INT_MASK_NRST_RDBK_INT_MASK_MASK) != 0U) {
        Pmic_intrBitSet(pErrStat, PMIC_BB_NRST_RDBK_INT_CFG);
    }

    if ((regValue & PMIC_RDBK_INT_MASK_SAFE_OUT1_RDBK_INT_MASK_MASK) != 0U) {
        Pmic_intrBitSet(pErrStat, PMIC_BB_SAFE_OUT1_RDBK_INT_CFG);
    }

    if ((regValue & PMIC_RDBK_INT_MASK_EN_OUT_RDBK_INT_MASK_MASK) != 0U) {
        Pmic_intrBitSet(pErrStat, PMIC_BB_EN_OUT_RDBK_INT_CFG);
    }

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_RDBK_INT_CFG2_REGADDR, & regData);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData)) {
        if ((regValue & PMIC_RDBK_INT_MASK_GPO1_RDBK_INT_MASK_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_GPO1_RDBK_INT_CFG);
        }

        if ((regValue & PMIC_RDBK_INT_MASK_GPO2_RDBK_INT_MASK_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_GPO2_RDBK_INT_CFG);
        }

        if ((regValue & PMIC_RDBK_INT_MASK_GPO3_RDBK_INT_MASK_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_GPO3_RDBK_INT_CFG);
        }

        if ((regValue & PMIC_RDBK_INT_MASK_GPO4_RDBK_INT_MASK_MASK) != 0U) {
            Pmic_intrBitSet(pErrStat, PMIC_BB_GPO4_RDBK_INT_CFG);
        }
    }
    return pmicStatus;
}

/**
 * @brief  Function to decipher Configuration Register CRC Error Interrupt
 * Configuration Bit Error.
 */
static int32_t Pmic_bb_getSafetyOffStateErr(uint8_t regValue,
    Pmic_IrqStatus_t * pErrStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if ((regValue & PMIC_SAFETYCFG_CRC_INT_CFG_MASK) != 0U) {
        Pmic_intrBitSet(pErrStat, PMIC_BB_CFG_REG_CRC_INT_CFG);
    }

    return pmicStatus;
}

/**
 * @brief  Function to decipher the L2 Error for TPS6594x Leo PMIC.
 */
int32_t Pmic_bb_irqGetL2Error(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint16_t l1RegAddr, Pmic_IrqStatus_t * pErrStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regValue = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Read the L1 register value */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, l1RegAddr, & regValue);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        switch (l1RegAddr) {
        case PMIC_SAFETY_CFG_REGADDR:
            pmicStatus = Pmic_bb_getSafetyOffStateErr(regValue, pErrStat);
            break;

        case PMIC_RDBK_INT_CFG1_REGADDR:
            pmicStatus = Pmic_bb_getRDBKErr(pPmicCoreHandle, regValue, pErrStat);
            break;

        case PMIC_OV_INT_CFG1_REGADDR:
            pmicStatus =
                Pmic_bb_getOV_LDO_VMON_PLDOErr(pPmicCoreHandle, regValue, pErrStat);
            break;

        case PMIC_UV_INT_CFG1_REGADDR:
            pmicStatus =
                Pmic_bb_getUV_LDO_VMON_PLDOErr(pPmicCoreHandle, regValue, pErrStat);
            break;
        case PMIC_WDG_INT_CFG_REGADDR:
            pmicStatus = Pmic_bb_getWDGErr(pPmicCoreHandle, pErrStat);
            break;
        case PMIC_ESM_INT_CFG_REGADDR:
            pmicStatus = Pmic_bb_getESMErr(pPmicCoreHandle, pErrStat);
            break;
        case PMIC_CM_COMP_INT_MSKCFG_REGADDR:
            pmicStatus = Pmic_bb_getCOMPErr(pPmicCoreHandle, pErrStat);
            break;
        case PMIC_CM_VMON_INT_CFG_REGADDR:
            pmicStatus = Pmic_bb_getCMVMONErr(pPmicCoreHandle, pErrStat);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_INT;
            break;
        }
    }

    return pmicStatus;
}

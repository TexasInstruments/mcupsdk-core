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
 *   @file    pmic_ilim.c
 *
 *   @brief   This file contains the default API's for PMIC ILIM configuration
 */

#include "pmic_ilim.h"
#include "pmic_ilim_priv.h"

static void initializeILIMeRRReg(Pmic_ilimStatReg_t * config) {
    /* Set default initialization values */
    config -> bbavgILIMErr = PMIC_ST_DEFAULT_DATA;
    config -> pldo2ILIMErr = PMIC_ST_DEFAULT_DATA;
    config -> pldo1ILIMErr = PMIC_ST_DEFAULT_DATA;
    config -> ldo4ILIMErr = PMIC_ST_DEFAULT_DATA;
    config -> ldo3ILIMErr = PMIC_ST_DEFAULT_DATA;
    config -> ldo2ILIMErr = PMIC_ST_DEFAULT_DATA;
    config -> ldo1ILIMErr = PMIC_ST_DEFAULT_DATA;
}

static void initializeILIMCfgReg(Pmic_ilimCfgReg_t * config) {
    /* Set default initialization values */
    config -> pldo2ILIMCfg = PMIC_ST_DEFAULT_DATA;
    config -> pldo1ILIMCfg = PMIC_ST_DEFAULT_DATA;
    config -> ldo4ILIMCfg = PMIC_ST_DEFAULT_DATA;
    config -> ldo3ILIMCfg = PMIC_ST_DEFAULT_DATA;
    config -> ldo2ILIMCfg = PMIC_ST_DEFAULT_DATA;
    config -> ldo1ILIMCfg = PMIC_ST_DEFAULT_DATA;
}

static void initializeILIMDglCfgReg(Pmic_ilimDglCfgReg_t * config) {
    /* Set default initialization values */
    config -> pldo2ILIMdglCfg = PMIC_ST_DEFAULT_DATA;
    config -> pldo1ILIMdglCfg = PMIC_ST_DEFAULT_DATA;
    config -> ldo4ILIMdglCfg = PMIC_ST_DEFAULT_DATA;
    config -> ldo3ILIMdglCfg = PMIC_ST_DEFAULT_DATA;
    config -> ldo2ILIMdglCfg = PMIC_ST_DEFAULT_DATA;
    config -> ldo1ILIMdglCfg = PMIC_ST_DEFAULT_DATA;
}

/**
 * @brief API to set ILIM Configuration
 */
int32_t Pmic_SetILIMConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_ilimCfgReg_t * pPmicILIMConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_ilimCfgReg_t * tempCfg = pPmicILIMConfig;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ILIM_CFG_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        if ((tempCfg -> pldo2ILIMCfg) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_PLDO2_ILIM_CFG_SHIFT,
                PMIC_PLDO2_ILIM_CFG_MASK, tempCfg -> pldo2ILIMCfg);
        }
        if ((tempCfg -> pldo1ILIMCfg) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_PLDO1_ILIM_CFG_SHIFT,
                PMIC_PLDO1_ILIM_CFG_MASK, tempCfg -> pldo1ILIMCfg);
        }
        if ((tempCfg -> ldo4ILIMCfg) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_LDO4_ILIM_CFG_SHIFT,
                PMIC_LDO4_ILIM_CFG_MASK, tempCfg -> ldo4ILIMCfg);
        }
        if ((tempCfg -> ldo3ILIMCfg) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_LDO3_ILIM_CFG_SHIFT,
                PMIC_LDO3_ILIM_CFG_MASK, tempCfg -> ldo3ILIMCfg);
        }
        if ((tempCfg -> ldo2ILIMCfg) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_LDO2_ILIM_CFG_SHIFT,
                PMIC_LDO2_ILIM_CFG_MASK, tempCfg -> ldo2ILIMCfg);
        }
        if ((tempCfg -> ldo1ILIMCfg) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_LDO1_ILIM_CFG_SHIFT,
                PMIC_LDO1_ILIM_CFG_MASK, tempCfg -> ldo1ILIMCfg);
        }
        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_ILIM_CFG_REGADDR, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief API to set ILIM DGL Configuration
 */
int32_t Pmic_SetILIMDglConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_ilimDglCfgReg_t * pPmicILIMdglConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_ilimDglCfgReg_t * tempCfg = pPmicILIMdglConfig;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_ILIM_DGL_CFG_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        if ((tempCfg -> pldo2ILIMdglCfg) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_PLDO2_ILIM_DGL_CFG_SHIFT,
                PMIC_PLDO2_ILIM_DGL_CFG_MASK, tempCfg -> pldo2ILIMdglCfg);
        }
        if ((tempCfg -> pldo1ILIMdglCfg) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_PLDO1_ILIM_DGL_CFG_SHIFT,
                PMIC_PLDO1_ILIM_DGL_CFG_MASK, tempCfg -> pldo1ILIMdglCfg);
        }
        if ((tempCfg -> ldo4ILIMdglCfg) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_LDO4_ILIM_DGL_CFG_SHIFT,
                PMIC_LDO4_ILIM_DGL_CFG_MASK, tempCfg -> ldo4ILIMdglCfg);
        }
        if ((tempCfg -> ldo3ILIMdglCfg) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_LDO3_ILIM_DGL_CFG_SHIFT,
                PMIC_LDO3_ILIM_DGL_CFG_MASK, tempCfg -> ldo3ILIMdglCfg);
        }
        if ((tempCfg -> ldo2ILIMdglCfg) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_LDO2_ILIM_DGL_CFG_SHIFT,
                PMIC_LDO2_ILIM_DGL_CFG_MASK, tempCfg -> ldo2ILIMdglCfg);
        }
        if ((tempCfg -> ldo1ILIMdglCfg) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_LDO1_ILIM_DGL_CFG_SHIFT,
                PMIC_LDO1_ILIM_DGL_CFG_MASK, tempCfg -> ldo1ILIMdglCfg);
        }
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
            PMIC_ILIM_DGL_CFG_REGADDR, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief API to clear ILIM STAT Configuration
 */
int32_t Pmic_ClearILIMErrStat(Pmic_CoreHandle_t * pPmicCoreHandle,
                              Pmic_ilimStatReg_t * pPmicILIMStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_ilimStatReg_t * tempStat = pPmicILIMStat;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ILIM_STAT_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        if ((tempStat -> bbavgILIMErr) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_BB_AVG_ILIM_ERR_SHIFT,
                PMIC_BB_AVG_ILIM_ERR_MASK, tempStat -> bbavgILIMErr);
        }
        if ((tempStat -> pldo2ILIMErr) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_PLDO2_ILIM_ERR_SHIFT,
                PMIC_PLDO2_ILIM_ERR_MASK, tempStat -> pldo2ILIMErr);
        }
        if ((tempStat -> pldo1ILIMErr) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_PLDO1_ILIM_ERR_SHIFT,
                PMIC_PLDO1_ILIM_ERR_MASK, tempStat -> pldo1ILIMErr);
        }
        if ((tempStat -> ldo4ILIMErr) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_LDO4_ILIM_ERR_SHIFT,
                PMIC_LDO4_ILIM_ERR_MASK, tempStat -> ldo4ILIMErr);
        }
        if ((tempStat -> ldo3ILIMErr) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_LDO3_ILIM_ERR_SHIFT,
                PMIC_LDO3_ILIM_ERR_MASK, tempStat -> ldo3ILIMErr);
        }
        if ((tempStat -> ldo2ILIMErr) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_LDO2_ILIM_ERR_SHIFT,
                PMIC_LDO2_ILIM_ERR_MASK, tempStat -> ldo2ILIMErr);
        }
        if ((tempStat -> ldo1ILIMErr) != PMIC_ST_DEFAULT_DATA) {
            Pmic_setBitField( & regData, PMIC_LDO1_ILIM_ERR_SHIFT,
                PMIC_LDO1_ILIM_ERR_MASK, tempStat -> ldo1ILIMErr);
        }
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_ILIM_STAT_REGADDR,
            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief API to get ILIM Configuration
 */
int32_t Pmic_GetILIMConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
                           Pmic_ilimCfgReg_t * pPmicILIMConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ILIM_CFG_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicILIMConfig -> pldo2ILIMCfg = Pmic_getBitField(
            regData, PMIC_PLDO2_ILIM_CFG_SHIFT, PMIC_PLDO2_ILIM_CFG_MASK);

        pPmicILIMConfig -> pldo1ILIMCfg = Pmic_getBitField(
            regData, PMIC_PLDO1_ILIM_CFG_SHIFT, PMIC_PLDO1_ILIM_CFG_MASK);

        pPmicILIMConfig -> ldo4ILIMCfg = Pmic_getBitField(
            regData, PMIC_LDO4_ILIM_CFG_SHIFT, PMIC_LDO4_ILIM_CFG_MASK);

        pPmicILIMConfig -> ldo3ILIMCfg = Pmic_getBitField(
            regData, PMIC_LDO3_ILIM_CFG_SHIFT, PMIC_LDO3_ILIM_CFG_MASK);

        pPmicILIMConfig -> ldo2ILIMCfg = Pmic_getBitField(
            regData, PMIC_LDO2_ILIM_CFG_SHIFT, PMIC_LDO2_ILIM_CFG_MASK);

        pPmicILIMConfig -> ldo1ILIMCfg = Pmic_getBitField(
            regData, PMIC_LDO1_ILIM_CFG_SHIFT, PMIC_LDO1_ILIM_CFG_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief API to get ILIM DGL Configuration
 */
int32_t Pmic_GetILIMDglConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
                              Pmic_ilimDglCfgReg_t * pPmicILIMdglConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_ILIM_DGL_CFG_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicILIMdglConfig -> pldo2ILIMdglCfg = Pmic_getBitField(
            regData, PMIC_PLDO2_ILIM_DGL_CFG_SHIFT, PMIC_PLDO2_ILIM_DGL_CFG_MASK);

        pPmicILIMdglConfig -> pldo1ILIMdglCfg = Pmic_getBitField(
            regData, PMIC_PLDO1_ILIM_DGL_CFG_SHIFT, PMIC_PLDO1_ILIM_DGL_CFG_MASK);

        pPmicILIMdglConfig -> ldo4ILIMdglCfg = Pmic_getBitField(
            regData, PMIC_LDO4_ILIM_DGL_CFG_SHIFT, PMIC_LDO4_ILIM_DGL_CFG_MASK);

        pPmicILIMdglConfig -> ldo3ILIMdglCfg = Pmic_getBitField(
            regData, PMIC_LDO3_ILIM_DGL_CFG_SHIFT, PMIC_LDO3_ILIM_DGL_CFG_MASK);

        pPmicILIMdglConfig -> ldo2ILIMdglCfg = Pmic_getBitField(
            regData, PMIC_LDO2_ILIM_DGL_CFG_SHIFT, PMIC_LDO2_ILIM_DGL_CFG_MASK);

        pPmicILIMdglConfig -> ldo1ILIMdglCfg = Pmic_getBitField(
            regData, PMIC_LDO1_ILIM_DGL_CFG_SHIFT, PMIC_LDO1_ILIM_DGL_CFG_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief API to get ILIM STAT Configuration
 */
int32_t Pmic_GetILIMErrStat(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_ilimStatReg_t * pPmicILIMStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ILIM_STAT_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicILIMStat -> bbavgILIMErr = Pmic_getBitField(
            regData, PMIC_BB_AVG_ILIM_ERR_SHIFT, PMIC_BB_AVG_ILIM_ERR_MASK);

        pPmicILIMStat -> pldo2ILIMErr = Pmic_getBitField(
            regData, PMIC_PLDO2_ILIM_ERR_SHIFT, PMIC_PLDO2_ILIM_ERR_MASK);

        pPmicILIMStat -> pldo1ILIMErr = Pmic_getBitField(
            regData, PMIC_PLDO1_ILIM_ERR_SHIFT, PMIC_PLDO1_ILIM_ERR_MASK);

        pPmicILIMStat -> ldo4ILIMErr = Pmic_getBitField(
            regData, PMIC_LDO4_ILIM_ERR_SHIFT, PMIC_LDO4_ILIM_ERR_MASK);

        pPmicILIMStat -> ldo3ILIMErr = Pmic_getBitField(
            regData, PMIC_LDO3_ILIM_ERR_SHIFT, PMIC_LDO3_ILIM_ERR_MASK);

        pPmicILIMStat -> ldo2ILIMErr = Pmic_getBitField(
            regData, PMIC_LDO2_ILIM_ERR_SHIFT, PMIC_LDO2_ILIM_ERR_MASK);

        pPmicILIMStat -> ldo1ILIMErr = Pmic_getBitField(
            regData, PMIC_LDO1_ILIM_ERR_SHIFT, PMIC_LDO1_ILIM_ERR_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

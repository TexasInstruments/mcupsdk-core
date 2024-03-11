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
 *   \file    pmic_sw_shutdown.c
 *
 *   @brief   This file contains the default API's for PMIC SW SHUTDOWN
 */

#include "pmic_sw_shutdown.h"

#include "pmic_sw_shutdown_priv.h"

/*!
 * \brief API to get OFF STATE STAT1 Configuration
 */
int32_t
Pmic_GetOffStateStat1Config(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_offstatestat1Reg_t * pPmicOffStateStat1Config) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_OFF_STATE_STAT1_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicOffStateStat1Config -> initTMO =
            Pmic_getBitField(regData, PMIC_NORMAL_OFF_SHIFT, PMIC_NORMAL_OFF_MASK);

        pPmicOffStateStat1Config -> internalOV = Pmic_getBitField(
            regData, PMIC_OFF_INT_EVT_ERR_SHIFT, PMIC_OFF_INT_EVT_ERR_MASK);

        pPmicOffStateStat1Config -> ClkErr = Pmic_getBitField(
            regData, PMIC_OFF_PROT_EVT_SHIFT, PMIC_OFF_PROT_EVT_MASK);

        pPmicOffStateStat1Config -> firstPWROn = Pmic_getBitField(
            regData, PMIC_FIRST_PWR_ON_SHIFT, PMIC_FIRST_PWR_ON_MASK);

        pPmicOffStateStat1Config -> OffprotEvt =
            Pmic_getBitField(regData, PMIC_CLK_ERR_SHIFT, PMIC_CLK_ERR_MASK);

        pPmicOffStateStat1Config -> OffinitEvtErr = Pmic_getBitField(
            regData, PMIC_INTERNAL_OV_SHIFT, PMIC_INTERNAL_OV_MASK);

        pPmicOffStateStat1Config -> normalOff = Pmic_getBitField(
            regData, PMIC_INIT_AN_TMO_SHIFT, PMIC_INIT_AN_TMO_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief API to get OFF STATE STAT2 Configuration
 */
int32_t
Pmic_GetOffStateStat2Config(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_offstatestat2Reg_t * pPmicOffStateStat2Config) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_OFF_STATE_STAT2_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicOffStateStat2Config -> BBIlimErr = Pmic_getBitField(
            regData, PMIC_BB_PK_ILIM_ERR_SHIFT, PMIC_BB_PK_ILIM_ERR_MASK);

        pPmicOffStateStat2Config -> BBTMO =
            Pmic_getBitField(regData, PMIC_BB_BST_TMO_SHIFT, PMIC_BB_BST_TMO_MASK);

        pPmicOffStateStat2Config -> BBOVPErr =
            Pmic_getBitField(regData, PMIC_BB_OVP_ERR_SHIFT, PMIC_BB_OVP_ERR_MASK);

        pPmicOffStateStat2Config -> VbatOVPErr = Pmic_getBitField(
            regData, PMIC_VBAT_OVP_ERR_SHIFT, PMIC_VBAT_OVP_ERR_MASK);

        pPmicOffStateStat2Config -> BGXMErr =
            Pmic_getBitField(regData, PMIC_BGXM_ERR_SHIFT, PMIC_BGXM_ERR_MASK);

        pPmicOffStateStat2Config -> RstMcuTMO = Pmic_getBitField(
            regData, PMIC_RST_MCU_TMO_SHIFT, PMIC_RST_MCU_TMO_MASK);

        pPmicOffStateStat2Config -> SysClkErr = Pmic_getBitField(
            regData, PMIC_SYS_CLK_ERR_PROT_SHIFT, PMIC_SYS_CLK_ERR_PROT_MASK);

        pPmicOffStateStat2Config -> CRCErr =
            Pmic_getBitField(regData, PMIC_CRC_ERR_SHIFT, PMIC_CRC_ERR_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief API to clear OFF STATE STAT registers
 */
int32_t Pmic_ClearOffStateStatRegs(Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_OFF_STATE_CLR_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, PMIC_OFF_STATE_STAT_CLR_SHIFT,
            PMIC_OFF_STATE_STAT_CLR_MASK,
            PMIC_OFF_STATE_STAT_CLR_DATA);
    }

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
        PMIC_OFF_STATE_CLR_REGADDR, regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief API to get THERMAL_STAT1 Configuration
 */
int32_t
Pmic_GetThermalStat1Config(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_thermalstat1Reg_t * pPmicThermalStat1Config) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_THERMAL_STAT1_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicThermalStat1Config -> ldo4TSDErr = Pmic_getBitField(
            regData, PMIC_TSTAT1_LDO4_TSD_ERR_SHIFT, PMIC_TSTAT1_LDO4_TSD_ERR_MASK);

        pPmicThermalStat1Config -> ldo4TpreErr =
            Pmic_getBitField(regData, PMIC_TSTAT1_LDO4_TPRE_ERR_SHIFT,
                PMIC_TSTAT1_LDO4_T_PRE_ERR_MASK);

        pPmicThermalStat1Config -> ldo3TSDErr = Pmic_getBitField(
            regData, PMIC_TSTAT1_LDO3_TSD_ERR_SHIFT, PMIC_TSTAT1_LDO3_TSD_ERR_MASK);

        pPmicThermalStat1Config -> ldo3TpreErr =
            Pmic_getBitField(regData, PMIC_TSTAT1_LDO3_TPRE_ERR_SHIFT,
                PMIC_TSTAT1_LDO3_T_PRE_ERR_MASK);

        pPmicThermalStat1Config -> ldo2TSDErr = Pmic_getBitField(
            regData, PMIC_TSTAT1_LDO2_TSD_ERR_SHIFT, PMIC_TSTAT1_LDO2_TSD_ERR_MASK);

        pPmicThermalStat1Config -> ldo2TpreErr =
            Pmic_getBitField(regData, PMIC_TSTAT1_LDO2_TPRE_ERR_SHIFT,
                PMIC_TSTAT1_LDO2_T_PRE_ERR_MASK);

        pPmicThermalStat1Config -> ldo1TSDErr = Pmic_getBitField(
            regData, PMIC_TSTAT1_LDO1_TSD_ERR_SHIFT, PMIC_TSTAT1_LDO1_TSD_ERR_MASK);

        pPmicThermalStat1Config -> ldo1TpreErr =
            Pmic_getBitField(regData, PMIC_TSTAT1_LDO1_TPRE_ERR_SHIFT,
                PMIC_TSTAT1_LDO1_T_PRE_ERR_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief API to get THERMAL_STAT2 Configuration
 */
int32_t
Pmic_GetThermalStat2Config(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_thermalstat2Reg_t * pPmicThermalStat2Config) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_THERMAL_STAT2_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicThermalStat2Config -> BBTSDErr = Pmic_getBitField(
            regData, PMIC_TSTAT2_BB_TSD_ERR_SHIFT, PMIC_TSTAT2_BB_TSD_ERR_MASK);

        pPmicThermalStat2Config -> BBTpreErr = Pmic_getBitField(
            regData, PMIC_TSTAT2_BB_TPRE_ERR_SHIFT, PMIC_TSTAT2_BB_T_PRE_ERR_MASK);

        pPmicThermalStat2Config -> pldo2TSDErr =
            Pmic_getBitField(regData, PMIC_TSTAT2_PLDO2_TSD_ERR_SHIFT,
                PMIC_TSTAT2_PLDO2_TSD_ERR_MASK);

        pPmicThermalStat2Config -> pldo2TpreErr =
            Pmic_getBitField(regData, PMIC_TSTAT2_PLDO2_TPRE_ER_SHIFT,
                PMIC_TSTAT2_PLDO2_TPRE_ER_MASK);

        pPmicThermalStat2Config -> pldo1TSDErr =
            Pmic_getBitField(regData, PMIC_TSTAT2_PLDO1_TSD_ERR_SHIFT,
                PMIC_TSTAT2_PLDO1_TSD_ERR_MASK);

        pPmicThermalStat2Config -> pldo1TpreErr =
            Pmic_getBitField(regData, PMIC_TSTAT2_PLDO1_TPRE_ER_SHIFT,
                PMIC_TSTAT2_PLDO1_TPRE_ER_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief API to clear THERMAL_STAT1 register flags
 */
int32_t Pmic_ClearThermalStat1flags(Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_THERMAL_STAT1_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, PMIC_TSTAT1_LDO4_TSD_ERR_SHIFT,
            PMIC_TSTAT1_LDO4_TSD_ERR_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);

        Pmic_setBitField( & regData, PMIC_TSTAT1_LDO4_TPRE_ERR_SHIFT,
            PMIC_TSTAT1_LDO4_T_PRE_ERR_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);

        Pmic_setBitField( & regData, PMIC_TSTAT1_LDO3_TSD_ERR_SHIFT,
            PMIC_TSTAT1_LDO3_TSD_ERR_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);

        Pmic_setBitField( & regData, PMIC_TSTAT1_LDO3_TPRE_ERR_SHIFT,
            PMIC_TSTAT1_LDO3_T_PRE_ERR_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);

        Pmic_setBitField( & regData, PMIC_TSTAT1_LDO2_TSD_ERR_SHIFT,
            PMIC_TSTAT1_LDO2_TSD_ERR_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);

        Pmic_setBitField( & regData, PMIC_TSTAT1_LDO2_TPRE_ERR_SHIFT,
            PMIC_TSTAT1_LDO2_T_PRE_ERR_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);

        Pmic_setBitField( & regData, PMIC_TSTAT1_LDO1_TSD_ERR_SHIFT,
            PMIC_TSTAT1_LDO1_TSD_ERR_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);

        Pmic_setBitField( & regData, PMIC_TSTAT1_LDO1_TPRE_ERR_SHIFT,
            PMIC_TSTAT1_LDO1_T_PRE_ERR_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);
    }

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
        PMIC_THERMAL_STAT1_REGADDR, regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief API to clear THERMAL_STAT2 register flags
 */
int32_t Pmic_ClearThermalStat2flags(Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_THERMAL_STAT2_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, PMIC_TSTAT2_BB_TSD_ERR_SHIFT,
            PMIC_TSTAT2_BB_TSD_ERR_MASK, PMIC_THERMAL_STAT_CLEAR_DATA);

        Pmic_setBitField( & regData, PMIC_TSTAT2_BB_TPRE_ERR_SHIFT,
            PMIC_TSTAT2_BB_T_PRE_ERR_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);

        Pmic_setBitField( & regData, PMIC_TSTAT2_PLDO2_TSD_ERR_SHIFT,
            PMIC_TSTAT2_PLDO2_TSD_ERR_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);

        Pmic_setBitField( & regData, PMIC_TSTAT2_PLDO2_TPRE_ER_SHIFT,
            PMIC_TSTAT2_PLDO2_TPRE_ER_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);

        Pmic_setBitField( & regData, PMIC_TSTAT2_PLDO1_TSD_ERR_SHIFT,
            PMIC_TSTAT2_PLDO1_TSD_ERR_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);

        Pmic_setBitField( & regData, PMIC_TSTAT2_PLDO1_TPRE_ER_SHIFT,
            PMIC_TSTAT2_PLDO1_TPRE_ER_MASK,
            PMIC_THERMAL_STAT_CLEAR_DATA);
    }

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
        PMIC_THERMAL_STAT2_REGADDR, regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}
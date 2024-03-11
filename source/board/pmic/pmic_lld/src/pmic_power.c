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
 *   @file    pmic_power_tps65386x.c
 *
 *   @brief   This file contains the TPS65386x BB PMIC power Specific
 *            configuration API's and structures
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_power.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static inline void
initializeBuckBoostCfgReg(Pmic_powerBuckBoostCfgReg_t * config) {
    /* Set default initialization values */
    config -> bbPgoodCfg = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> bbSsEn = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> bbStbyLvlCfg = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> bbLvlCfg = (uint8_t) PMIC_ST_DEFAULT_DATA;
}

static inline void initializeLDOCfgReg(Pmic_ldoCfgReg_t * config) {
    /* Set default initialization values */
    config -> ldoRtCfg = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> ldoIlimLvlCfg = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> ldoLvlCfg = (uint8_t) PMIC_ST_DEFAULT_DATA;
}

static inline void initializePLDOCfgReg(Pmic_pldoCfgReg_t * config) {
    /* Set default initialization values */
    config -> pldoModeSel = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> pldoIlimLvlCfg = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> pldoLvlCfg = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> pldoRegAddr = (uint8_t) PMIC_ST_DEFAULT_DATA;
}

static inline void initializePLDOVTrackRTReg(Pmic_pldoVTrackRtReg_t * config) {
    /* Set default initialization values */
    config -> pldoVTrackRng = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> pldo1RTCfgVal = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> pldo2RTCfgVal = (uint8_t) PMIC_ST_DEFAULT_DATA;
}

static inline void initializePLDOEnOutReg(Pmic_pldoEnOutCtrlReg_t * config) {
    /* Set default initialization values */
    config -> enOut1Enable = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> enOut2Enable = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> pldo1Ctrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> pldo2Ctrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> pldoEnOutRegAddr = (uint8_t) PMIC_ST_DEFAULT_DATA;
}

static inline void initializeLPVMonCtrlReg(Pmic_lpVMonCtrlReg_t * config) {
    /* Set default initialization values */
    config -> lpExtVMon2Ctrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> lpExtVMon1Ctrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> lppldo2VMonCtrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> lppldo1VMonCtrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> lpldo4VMonCtrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> lpldo3VMonCtrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> lpldo2VMonCtrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> lpldo1VMonCtrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> lpVmonCtrlRegAddr = (uint8_t) PMIC_ST_DEFAULT_DATA;
}

static inline void initializeLPConfigReg(Pmic_lpConfigCtrlReg_t * config) {
    /* Set default initialization values */
    config -> lpBBOVPCtrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> lpBBVMonCtrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> lpTSDperConfig = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> lpVMonperConfig = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> lpCfgRegAddr = (uint8_t) PMIC_ST_DEFAULT_DATA;
}

static inline void initializeLDOControlReg(Pmic_ldoCtrlReg_t * config) {
    /* Set default initialization values */
    config -> ldo1Ctrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> ldo2Ctrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> ldo3Ctrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
    config -> ldo4Ctrl = (uint8_t) PMIC_ST_DEFAULT_DATA;
}

int32_t
Pmic_powerSetBuckBstPgoodCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerBuckBoostCfgReg_t * buckBstCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = buckBstCfg -> bbCfgRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, buckBstCfg -> bbCfgRegShift,
            buckBstCfg -> bbCfgRegMask, buckBstCfg -> bbPgoodCfg);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_powerGetBuckBstPgoodCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = buckBstCfg -> bbCfgRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        buckBstCfg -> bbPgoodCfg = Pmic_getBitField(
            regData, buckBstCfg -> bbCfgRegShift, buckBstCfg -> bbCfgRegMask);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t
Pmic_powerSetBuckBstSsEn(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerBuckBoostCfgReg_t * buckBstCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = buckBstCfg -> bbCfgRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, buckBstCfg -> bbCfgRegShift,
            buckBstCfg -> bbCfgRegMask, buckBstCfg -> bbSsEn);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_powerGetBuckBstSsEn(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = buckBstCfg -> bbCfgRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        buckBstCfg -> bbSsEn = Pmic_getBitField(regData, buckBstCfg -> bbCfgRegShift,
            buckBstCfg -> bbCfgRegMask);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t
Pmic_powerSetBuckBstStbyLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerBuckBoostCfgReg_t * buckBstCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = buckBstCfg -> bbCfgRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, buckBstCfg -> bbCfgRegShift,
            buckBstCfg -> bbCfgRegMask, buckBstCfg -> bbStbyLvlCfg);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t
Pmic_powerGetBuckBstStbyLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = buckBstCfg -> bbCfgRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        buckBstCfg -> bbStbyLvlCfg = Pmic_getBitField(
            regData, buckBstCfg -> bbCfgRegShift, buckBstCfg -> bbCfgRegMask);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t
Pmic_powerSetBuckBstLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerBuckBoostCfgReg_t * buckBstCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = buckBstCfg -> bbCfgRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, buckBstCfg -> bbCfgRegShift,
            buckBstCfg -> bbCfgRegMask, buckBstCfg -> bbLvlCfg);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_powerGetBuckBstLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = buckBstCfg -> bbCfgRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        buckBstCfg -> bbLvlCfg = Pmic_getBitField(regData, buckBstCfg -> bbCfgRegShift,
            buckBstCfg -> bbCfgRegMask);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_powerSetBuckBstCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg,
        const Pmic_powerRsrcCfg_t * pwrRsrcCfg) {

    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        buckBstCfg -> bbCfgRegAddr = pwrRsrcRegCfg -> buckConfigRegAddr;
        buckBstCfg -> bbCfgRegShift = pwrRsrcCfg -> pmicConfigShiftVal;
        buckBstCfg -> bbCfgRegMask = pwrRsrcCfg -> pmicConfigMaskVal;

        if (buckBstCfg -> bbPgoodCfg != (uint8_t) PMIC_ST_DEFAULT_DATA) {
            pmicStatus = Pmic_powerSetBuckBstPgoodCfg(pPmicCoreHandle, buckBstCfg);
        }

        if (buckBstCfg -> bbSsEn != (uint8_t) PMIC_ST_DEFAULT_DATA) {
            pmicStatus = Pmic_powerSetBuckBstSsEn(pPmicCoreHandle, buckBstCfg);
        }

        if (buckBstCfg -> bbStbyLvlCfg != (uint8_t) PMIC_ST_DEFAULT_DATA) {
            pmicStatus = Pmic_powerSetBuckBstStbyLvlCfg(pPmicCoreHandle, buckBstCfg);
        }

        if (buckBstCfg -> bbLvlCfg != (uint8_t) PMIC_ST_DEFAULT_DATA) {
            pmicStatus = Pmic_powerSetBuckBstLvlCfg(pPmicCoreHandle, buckBstCfg);
        }
    }

    initializeBuckBoostCfgReg(buckBstCfg);

    return pmicStatus;
}

int32_t Pmic_powerGetBuckBstCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg,
        const Pmic_powerRsrcCfg_t * pwrRsrcCfg) {

    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        buckBstCfg -> bbCfgRegAddr = pwrRsrcRegCfg -> buckConfigRegAddr;
        buckBstCfg -> bbCfgRegShift = pwrRsrcCfg -> pmicConfigShiftVal;
        buckBstCfg -> bbCfgRegMask = pwrRsrcCfg -> pmicConfigMaskVal;

        if (buckBstCfg -> bbPgoodCfg == (uint8_t) PMIC_ST_DEFAULT_DATA) {
            pmicStatus = Pmic_powerGetBuckBstPgoodCfg(pPmicCoreHandle, buckBstCfg);
        }

        if (buckBstCfg -> bbSsEn == (uint8_t) PMIC_ST_DEFAULT_DATA) {
            pmicStatus = Pmic_powerGetBuckBstSsEn(pPmicCoreHandle, buckBstCfg);
        }

        if (buckBstCfg -> bbStbyLvlCfg == (uint8_t) PMIC_ST_DEFAULT_DATA) {
            pmicStatus = Pmic_powerGetBuckBstStbyLvlCfg(pPmicCoreHandle, buckBstCfg);
        }

        if (buckBstCfg -> bbLvlCfg == (uint8_t) PMIC_ST_DEFAULT_DATA) {
            pmicStatus = Pmic_powerGetBuckBstLvlCfg(pPmicCoreHandle, buckBstCfg);
        }
    }

    initializeBuckBoostCfgReg(buckBstCfg);

    return pmicStatus;
}

int32_t
Pmic_powerSetLdoConfigRegister(Pmic_CoreHandle_t * pPmicCoreHandle,
                               uint8_t ldoNumber, Pmic_ldoCfgReg_t *ldoConfig)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t regData = 0U;
    uint8_t regBitPos = 0U;
    uint8_t regBitMask = 0U;
    uint8_t ldoCfgFeature = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        if ((ldoNumber < PMIC_POWER_LDO_MIN) || (ldoNumber > PMIC_POWER_LDO_MAX)) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        } else {
            switch (ldoNumber) {
            case PMIC_LDO1:
                regAddr = PMIC_LDO1_CFG_REGADDR;
                regBitPos = LDO_ALL_CFG_SHIFT;
                regBitMask = LDO_ALL_CFG_MASK;
                Pmic_criticalSectionStart(pPmicCoreHandle);
                ldoCfgFeature = Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);
                if (ldoConfig->ldoRtCfg != (uint8_t)PMIC_ST_DEFAULT_DATA) {
                    Pmic_setBitField(&ldoCfgFeature, LDO_RT_CFG_SHIFT, LDO_RT_CFG_MASK, ldoConfig->ldoRtCfg);
                }

                if (ldoConfig->ldoIlimLvlCfg != (uint8_t)PMIC_ST_DEFAULT_DATA) {
                    Pmic_setBitField(&ldoCfgFeature, LDO_ILIM_LVL_CFG_SHIFT, LDO_ILIM_LVL_CFG_MASK, ldoConfig->ldoIlimLvlCfg);
                }

                if (ldoConfig->ldoLvlCfg != (uint8_t)PMIC_ST_DEFAULT_DATA) {
                    Pmic_setBitField(&ldoCfgFeature, LDO_LVL_CFG_SHIFT, LDO_LVL_CFG_MASK, ldoConfig->ldoLvlCfg);
                }
                Pmic_setBitField( &regData, regBitPos, regBitMask, ldoCfgFeature);
                pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);
                Pmic_criticalSectionStop(pPmicCoreHandle);
                break;
            case PMIC_LDO2:
                regAddr = PMIC_LDO2_CFG_REGADDR;
                regBitPos = LDO_ALL_CFG_SHIFT;
                regBitMask = LDO_ALL_CFG_MASK;
                Pmic_criticalSectionStart(pPmicCoreHandle);
                ldoCfgFeature = Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);
                if (ldoConfig->ldoRtCfg != (uint8_t)PMIC_ST_DEFAULT_DATA) {
                    Pmic_setBitField(&ldoCfgFeature, LDO_RT_CFG_SHIFT, LDO_RT_CFG_MASK, ldoConfig->ldoRtCfg);
                }

                if (ldoConfig->ldoIlimLvlCfg != (uint8_t)PMIC_ST_DEFAULT_DATA) {
                    Pmic_setBitField(&ldoCfgFeature, LDO_ILIM_LVL_CFG_SHIFT, LDO_ILIM_LVL_CFG_MASK, ldoConfig->ldoIlimLvlCfg);
                }

                if (ldoConfig->ldoLvlCfg != (uint8_t)PMIC_ST_DEFAULT_DATA) {
                    Pmic_setBitField(&ldoCfgFeature, LDO_LVL_CFG_SHIFT, LDO_LVL_CFG_MASK, ldoConfig->ldoLvlCfg);
                }
                Pmic_setBitField( &regData, regBitPos, regBitMask, ldoCfgFeature);
                pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);
                Pmic_criticalSectionStop(pPmicCoreHandle);
                break;
            case PMIC_LDO3:
                regAddr = PMIC_LDO3_CFG_REGADDR;
                regBitPos = LDO_ALL_CFG_SHIFT;
                regBitMask = LDO_ALL_CFG_MASK;
                Pmic_criticalSectionStart(pPmicCoreHandle);
                ldoCfgFeature = Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);
                if (ldoConfig->ldoRtCfg != (uint8_t)PMIC_ST_DEFAULT_DATA) {
                    Pmic_setBitField(&ldoCfgFeature, LDO_RT_CFG_SHIFT, LDO_RT_CFG_MASK, ldoConfig->ldoRtCfg);
                }

                if (ldoConfig->ldoIlimLvlCfg != (uint8_t)PMIC_ST_DEFAULT_DATA) {
                    Pmic_setBitField(&ldoCfgFeature, LDO_ILIM_LVL_CFG_SHIFT, LDO_ILIM_LVL_CFG_MASK, ldoConfig->ldoIlimLvlCfg);
                }

                if (ldoConfig->ldoLvlCfg != (uint8_t)PMIC_ST_DEFAULT_DATA) {
                    Pmic_setBitField(&ldoCfgFeature, LDO_LVL_CFG_SHIFT, LDO_LVL_CFG_MASK, ldoConfig->ldoLvlCfg);
                }
                Pmic_setBitField( &regData, regBitPos, regBitMask, ldoCfgFeature);
                pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);
                Pmic_criticalSectionStop(pPmicCoreHandle);
                break;
            case PMIC_LDO4:
                regAddr = PMIC_LDO4_CFG_REGADDR;
                regBitPos = LDO_ALL_CFG_SHIFT;
                regBitMask = LDO_ALL_CFG_MASK;
                Pmic_criticalSectionStart(pPmicCoreHandle);
                ldoCfgFeature = Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);
                if (ldoConfig->ldoRtCfg != (uint8_t)PMIC_ST_DEFAULT_DATA) {
                    Pmic_setBitField(&ldoCfgFeature, LDO_RT_CFG_SHIFT, LDO_RT_CFG_MASK, ldoConfig->ldoRtCfg);
                }

                if (ldoConfig->ldoIlimLvlCfg != (uint8_t)PMIC_ST_DEFAULT_DATA) {
                    Pmic_setBitField(&ldoCfgFeature, LDO_ILIM_LVL_CFG_SHIFT, LDO_ILIM_LVL_CFG_MASK, ldoConfig->ldoIlimLvlCfg);
                }

                if (ldoConfig->ldoLvlCfg != (uint8_t)PMIC_ST_DEFAULT_DATA) {
                    Pmic_setBitField(&ldoCfgFeature, LDO_LVL_CFG_SHIFT, LDO_LVL_CFG_MASK, ldoConfig->ldoLvlCfg);
                }
                Pmic_setBitField( &regData, regBitPos, regBitMask, ldoCfgFeature);
                pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);
                Pmic_criticalSectionStop(pPmicCoreHandle);
                break;
            default:
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    return pmicStatus;
}

int32_t
Pmic_powerGetLdoConfigRegister(Pmic_CoreHandle_t * pPmicCoreHandle,
                               uint8_t ldoNumber, Pmic_ldoCfgReg_t *ldoConfig)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        if ((ldoNumber < PMIC_POWER_LDO_MIN) || (ldoNumber > PMIC_POWER_LDO_MAX)) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        } else {
            switch (ldoNumber) {
            case PMIC_LDO1:
                regAddr = PMIC_LDO1_CFG_REGADDR;
                Pmic_criticalSectionStart(pPmicCoreHandle);
                pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);
                Pmic_criticalSectionStop(pPmicCoreHandle);

                if (PMIC_ST_SUCCESS == pmicStatus) {
                    ldoConfig->ldoRtCfg = Pmic_getBitField(regData, LDO_RT_CFG_SHIFT, LDO_RT_CFG_MASK);
                }

                if (PMIC_ST_SUCCESS == pmicStatus) {
                    ldoConfig->ldoIlimLvlCfg = Pmic_getBitField(regData, LDO_ILIM_LVL_CFG_SHIFT, LDO_ILIM_LVL_CFG_MASK);
                }

                if (PMIC_ST_SUCCESS == pmicStatus) {
                    ldoConfig->ldoLvlCfg = Pmic_getBitField(regData, LDO_LVL_CFG_SHIFT, LDO_LVL_CFG_MASK);
                }
                break;
            case PMIC_LDO2:
                regAddr = PMIC_LDO2_CFG_REGADDR;
                Pmic_criticalSectionStart(pPmicCoreHandle);
                pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);
                Pmic_criticalSectionStop(pPmicCoreHandle);

                if (PMIC_ST_SUCCESS == pmicStatus) {
                    ldoConfig->ldoRtCfg = Pmic_getBitField(regData, LDO_RT_CFG_SHIFT, LDO_RT_CFG_MASK);
                }

                if (PMIC_ST_SUCCESS == pmicStatus) {
                    ldoConfig->ldoIlimLvlCfg = Pmic_getBitField(regData, LDO_ILIM_LVL_CFG_SHIFT, LDO_ILIM_LVL_CFG_MASK);
                }

                if (PMIC_ST_SUCCESS == pmicStatus) {
                    ldoConfig->ldoLvlCfg = Pmic_getBitField(regData, LDO_LVL_CFG_SHIFT, LDO_LVL_CFG_MASK);
                }
                break;
            case PMIC_LDO3:
                regAddr = PMIC_LDO3_CFG_REGADDR;
                Pmic_criticalSectionStart(pPmicCoreHandle);
                pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);
                Pmic_criticalSectionStop(pPmicCoreHandle);

                if (PMIC_ST_SUCCESS == pmicStatus) {
                    ldoConfig->ldoRtCfg = Pmic_getBitField(regData, LDO_RT_CFG_SHIFT, LDO_RT_CFG_MASK);
                }

                if (PMIC_ST_SUCCESS == pmicStatus) {
                    ldoConfig->ldoIlimLvlCfg = Pmic_getBitField(regData, LDO_ILIM_LVL_CFG_SHIFT, LDO_ILIM_LVL_CFG_MASK);
                }

                if (PMIC_ST_SUCCESS == pmicStatus) {
                    ldoConfig->ldoLvlCfg = Pmic_getBitField(regData, LDO_LVL_CFG_SHIFT, LDO_LVL_CFG_MASK);
                }
                break;
            case PMIC_LDO4:
                regAddr = PMIC_LDO4_CFG_REGADDR;
                Pmic_criticalSectionStart(pPmicCoreHandle);
                pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);
                Pmic_criticalSectionStop(pPmicCoreHandle);

                if (PMIC_ST_SUCCESS == pmicStatus) {
                    ldoConfig->ldoRtCfg = Pmic_getBitField(regData, LDO_RT_CFG_SHIFT, LDO_RT_CFG_MASK);
                }

                if (PMIC_ST_SUCCESS == pmicStatus) {
                    ldoConfig->ldoIlimLvlCfg = Pmic_getBitField(regData, LDO_ILIM_LVL_CFG_SHIFT, LDO_ILIM_LVL_CFG_MASK);
                }

                if (PMIC_ST_SUCCESS == pmicStatus) {
                    ldoConfig->ldoLvlCfg = Pmic_getBitField(regData, LDO_LVL_CFG_SHIFT, LDO_LVL_CFG_MASK);
                }
                break;
            default:
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }

        }
    }

    return pmicStatus;
}

void Pmic_getPLDOCfgFields(uint8_t pldoNumber, Pmic_pldoCfgReg_t * pldoCfg,
    const Pmic_powerRsrcCfg_t * pwrRsrcCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    pldoCfg -> pldoRegShift = pwrRsrcCfg -> pmicConfigShiftVal;
    pldoCfg -> pldoRegMask = pwrRsrcCfg -> pmicConfigMaskVal;

    switch (pldoNumber) {
    case PMIC_PLDO1:
        pldoCfg -> pldoRegAddr = pwrRsrcRegCfg -> pldo1ConfigRegAddr;
        break;
    case PMIC_PLDO2:
        pldoCfg -> pldoRegAddr = pwrRsrcRegCfg -> pldo2ConfigRegAddr;
        break;
    default:
        pldoCfg -> pldoRegAddr = (uint8_t) PMIC_ST_DEFAULT_DATA;
        break;
    }
}

int32_t Pmic_powerSetPLDOModeSel(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoCfgReg_t * pldoCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoCfg -> pldoRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, pldoCfg -> pldoRegShift, pldoCfg -> pldoRegMask,
            pldoCfg -> pldoModeSel);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_powerGetPLDOModeSel(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoCfgReg_t * pldoCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoCfg -> pldoRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pldoCfg -> pldoModeSel =
            Pmic_getBitField(regData, pldoCfg -> pldoRegShift, pldoCfg -> pldoRegMask);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_powerSetPLDOIlimLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoCfgReg_t * pldoCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoCfg -> pldoRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, pldoCfg -> pldoRegShift, pldoCfg -> pldoRegMask,
            pldoCfg -> pldoIlimLvlCfg);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_powerGetPLDOIlimLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoCfgReg_t * pldoCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoCfg -> pldoRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pldoCfg -> pldoIlimLvlCfg =
            Pmic_getBitField(regData, pldoCfg -> pldoRegShift, pldoCfg -> pldoRegMask);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_powerSetPLDOLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoCfgReg_t * pldoCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoCfg -> pldoRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, pldoCfg -> pldoRegShift, pldoCfg -> pldoRegMask,
            pldoCfg -> pldoLvlCfg);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_powerGetPLDOLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoCfgReg_t * pldoCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoCfg -> pldoRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pldoCfg -> pldoLvlCfg =
            Pmic_getBitField(regData, pldoCfg -> pldoRegShift, pldoCfg -> pldoRegMask);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_powerSetPLDOConfigRegister(
    Pmic_CoreHandle_t * pPmicCoreHandle, uint8_t pldoNumber,
    Pmic_pldoCfgReg_t * pldoConfig,
    const Pmic_powerRsrcCfg_t * pwrRsrcCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        if ((pldoNumber < PMIC_POWER_PLDO_MIN) ||
            (pldoNumber > PMIC_POWER_PLDO_MAX)) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_getPLDOCfgFields(pldoNumber, pldoConfig, pwrRsrcCfg, pwrRsrcRegCfg);

            if (pldoConfig -> pldoModeSel != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                pmicStatus = Pmic_powerSetPLDOModeSel(pPmicCoreHandle, pldoConfig);
            }

            if (pldoConfig -> pldoIlimLvlCfg != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                pmicStatus = Pmic_powerSetPLDOIlimLvlCfg(pPmicCoreHandle, pldoConfig);
            }

            if (pldoConfig -> pldoLvlCfg != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                pmicStatus = Pmic_powerSetPLDOLvlCfg(pPmicCoreHandle, pldoConfig);
            }

            initializePLDOCfgReg(pldoConfig);
        }
    }

    return pmicStatus;
}

int32_t Pmic_powerGetPLDOConfigRegister(
    Pmic_CoreHandle_t * pPmicCoreHandle, uint8_t pldoNumber,
    Pmic_pldoCfgReg_t * pldoConfig,
    const Pmic_powerRsrcCfg_t * pwrRsrcCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        if ((pldoNumber < PMIC_POWER_PLDO_MIN) ||
            (pldoNumber > PMIC_POWER_PLDO_MAX)) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_getPLDOCfgFields(pldoNumber, pldoConfig, pwrRsrcCfg, pwrRsrcRegCfg);

            if (pldoConfig -> pldoModeSel != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                pmicStatus = Pmic_powerGetPLDOModeSel(pPmicCoreHandle, pldoConfig);
            }

            if (pldoConfig -> pldoIlimLvlCfg != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                pmicStatus = Pmic_powerGetPLDOIlimLvlCfg(pPmicCoreHandle, pldoConfig);
            }

            if (pldoConfig -> pldoLvlCfg != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                pmicStatus = Pmic_powerGetPLDOLvlCfg(pPmicCoreHandle, pldoConfig);
            }

            initializePLDOCfgReg(pldoConfig);
        }
    }

    return pmicStatus;
}

int32_t Pmic_setPLDOVTrackRng(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoVTrackRtReg_t * pldoCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoCfg -> pldoVTrackRTRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, pldoCfg -> pldoVTrackRTRegShift,
            pldoCfg -> pldoVTrackRTRegMask, pldoCfg -> pldoVTrackRng);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_getPLDOVTrackRng(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoVTrackRtReg_t * pldoCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoCfg -> pldoVTrackRTRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pldoCfg -> pldoVTrackRng = Pmic_getBitField(
            regData, pldoCfg -> pldoVTrackRTRegShift, pldoCfg -> pldoVTrackRTRegMask);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_setPLDO1RTCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoVTrackRtReg_t * pldoCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoCfg -> pldoVTrackRTRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, pldoCfg -> pldoVTrackRTRegShift,
            pldoCfg -> pldoVTrackRTRegMask, pldoCfg -> pldo1RTCfgVal);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_getPLDO1RTCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoVTrackRtReg_t * pldoCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoCfg -> pldoVTrackRTRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pldoCfg -> pldo1RTCfgVal = Pmic_getBitField(
            regData, pldoCfg -> pldoVTrackRTRegShift, pldoCfg -> pldoVTrackRTRegMask);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_setPLDO2RTCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoVTrackRtReg_t * pldoCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoCfg -> pldoVTrackRTRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, pldoCfg -> pldoVTrackRTRegShift,
            pldoCfg -> pldoVTrackRTRegMask, pldoCfg -> pldo2RTCfgVal);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_getPLDO2RTCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoVTrackRtReg_t * pldoCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoCfg -> pldoVTrackRTRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pldoCfg -> pldo2RTCfgVal = Pmic_getBitField(
            regData, pldoCfg -> pldoVTrackRTRegShift, pldoCfg -> pldoVTrackRTRegMask);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_SetPLDOVTrackRTRegCfg(
    Pmic_CoreHandle_t * pPmicCoreHandle, uint8_t pldoNumber,
    Pmic_pldoVTrackRtReg_t * pldoConfig,
    const Pmic_powerRsrcCfg_t * pwrRsrcCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        if ((pldoNumber < PMIC_POWER_PLDO_MIN) ||
            (pldoNumber > PMIC_POWER_PLDO_MAX)) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        } else {
            pldoConfig -> pldoVTrackRTRegAddr = pwrRsrcRegCfg -> pldoConfigRegAddr;
            pldoConfig -> pldoVTrackRTRegShift = pwrRsrcCfg -> pmicConfigShiftVal;
            pldoConfig -> pldoVTrackRTRegMask = pwrRsrcCfg -> pmicConfigMaskVal;

            if (pldoConfig -> pldoVTrackRng != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                pmicStatus = Pmic_setPLDOVTrackRng(pPmicCoreHandle, pldoConfig);
            }

            if (pldoConfig -> pldo1RTCfgVal != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                pmicStatus = Pmic_setPLDO1RTCfg(pPmicCoreHandle, pldoConfig);
            }

            if (pldoConfig -> pldo2RTCfgVal != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                pmicStatus = Pmic_setPLDO2RTCfg(pPmicCoreHandle, pldoConfig);
            }

            initializePLDOVTrackRTReg(pldoConfig);
        }
    }

    return pmicStatus;
}

int32_t Pmic_GetPLDOVTrackRTRegCfg(
    Pmic_CoreHandle_t * pPmicCoreHandle, uint8_t pldoNumber,
    Pmic_pldoVTrackRtReg_t * pldoConfig,
    const Pmic_powerRsrcCfg_t * pwrRsrcCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        if ((pldoNumber < PMIC_POWER_PLDO_MIN) ||
            (pldoNumber > PMIC_POWER_PLDO_MAX)) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        } else {
            pldoConfig -> pldoVTrackRTRegAddr = pwrRsrcRegCfg -> pldoConfigRegAddr;
            pldoConfig -> pldoVTrackRTRegShift = pwrRsrcCfg -> pmicConfigShiftVal;
            pldoConfig -> pldoVTrackRTRegMask = pwrRsrcCfg -> pmicConfigMaskVal;

            if (pldoConfig -> pldoVTrackRng != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                pmicStatus = Pmic_getPLDOVTrackRng(pPmicCoreHandle, pldoConfig);
            }

            if (pldoConfig -> pldo1RTCfgVal != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                pmicStatus = Pmic_getPLDO1RTCfg(pPmicCoreHandle, pldoConfig);
            }

            if (pldoConfig -> pldo2RTCfgVal != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                pmicStatus = Pmic_getPLDO2RTCfg(pPmicCoreHandle, pldoConfig);
            }

            initializePLDOVTrackRTReg(pldoConfig);
        }
    }

    return pmicStatus;
}

int32_t Pmic_setPldoPgoodCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    const Pmic_pgoodCfgReg_t * pldoPgoodCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoPgoodCfg -> pgoodRegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        switch (pldoNumber) {
        case PMIC_PLDO1:
            Pmic_setBitField( & regData, pldoPgoodCfg -> pgoodRegShift,
                pldoPgoodCfg -> pgoodRegMask, pldoPgoodCfg -> pldo1PgoodCfg);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_PLDO2:
            Pmic_setBitField( & regData, pldoPgoodCfg -> pgoodRegShift,
                pldoPgoodCfg -> pgoodRegMask, pldoPgoodCfg -> pldo2PgoodCfg);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_getPldoPgoodCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    Pmic_pgoodCfgReg_t * pldoPgoodCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoPgoodCfg -> pgoodRegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        switch (pldoNumber) {
        case PMIC_PLDO1:
            pldoPgoodCfg -> pldo1PgoodCfg = Pmic_getBitField(
                regData, pldoPgoodCfg -> pgoodRegShift, pldoPgoodCfg -> pgoodRegMask);
            break;
        case PMIC_PLDO2:
            pldoPgoodCfg -> pldo2PgoodCfg = Pmic_getBitField(
                regData, pldoPgoodCfg -> pgoodRegShift, pldoPgoodCfg -> pgoodRegMask);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_setLdoPgoodCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t ldoNumber,
    const Pmic_pgoodCfgReg_t * ldoPgoodCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = ldoPgoodCfg -> pgoodRegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        switch (ldoNumber) {
        case PMIC_LDO1:
            Pmic_setBitField( & regData, ldoPgoodCfg -> pgoodRegShift,
                ldoPgoodCfg -> pgoodRegMask, ldoPgoodCfg -> ldo1PgoodCfg);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_LDO2:
            Pmic_setBitField( & regData, ldoPgoodCfg -> pgoodRegShift,
                ldoPgoodCfg -> pgoodRegMask, ldoPgoodCfg -> ldo2PgoodCfg);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_LDO3:
            Pmic_setBitField( & regData, ldoPgoodCfg -> pgoodRegShift,
                ldoPgoodCfg -> pgoodRegMask, ldoPgoodCfg -> ldo3PgoodCfg);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_LDO4:
            Pmic_setBitField( & regData, ldoPgoodCfg -> pgoodRegShift,
                ldoPgoodCfg -> pgoodRegMask, ldoPgoodCfg -> ldo4PgoodCfg);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_getLdoPgoodCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
                            uint8_t ldoNumber,
                            Pmic_pgoodCfgReg_t *ldoPgoodCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = ldoPgoodCfg -> pgoodRegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        switch (ldoNumber) {
        case PMIC_LDO1:
            ldoPgoodCfg->ldo1PgoodCfg = Pmic_getBitField(
                regData, ldoPgoodCfg->pgoodRegShift, ldoPgoodCfg->pgoodRegMask);
            break;
        case PMIC_LDO2:
            ldoPgoodCfg->ldo2PgoodCfg = Pmic_getBitField(
                regData, ldoPgoodCfg->pgoodRegShift, ldoPgoodCfg->pgoodRegMask);
            break;
        case PMIC_LDO3:
            ldoPgoodCfg->ldo3PgoodCfg = Pmic_getBitField(
                regData, ldoPgoodCfg->pgoodRegShift, ldoPgoodCfg->pgoodRegMask);
            break;
        case PMIC_LDO4:
            ldoPgoodCfg->ldo4PgoodCfg = Pmic_getBitField(
                regData, ldoPgoodCfg->pgoodRegShift, ldoPgoodCfg->pgoodRegMask);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_setLdoCtrl(Pmic_CoreHandle_t *pPmicCoreHandle,
                        Pmic_ldoCtrlReg_t *ldoControl) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = PMIC_LDO_CTRL_REGADDR;
    uint8_t regData = 0U;
    uint8_t bitPos = LDO_ALL_CTRL_SHIFT;
    uint8_t bitMask = LDO_ALL_CTRL_MASK;
    uint8_t ldoCtrlFeature = 0x00U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        if ((ldoControl->ldo1Ctrl == PMIC_LDO_NOT_ENABLED) ||
                (ldoControl->ldo2Ctrl == PMIC_LDO_NOT_ENABLED) ||
                (ldoControl->ldo3Ctrl == PMIC_LDO_NOT_ENABLED) ||
                (ldoControl->ldo4Ctrl == PMIC_LDO_NOT_ENABLED)) {
            ldoCtrlFeature = PMIC_ALL_LDO_NOT_ENABLED;
        }
        if ((ldoControl->ldo1Ctrl == PMIC_LDO_ENABLED_LDO_MODE) ||
                (ldoControl->ldo2Ctrl == PMIC_LDO_ENABLED_LDO_MODE) ||
                (ldoControl->ldo3Ctrl == PMIC_LDO_ENABLED_LDO_MODE) ||
                (ldoControl->ldo4Ctrl == PMIC_LDO_ENABLED_LDO_MODE)) {
            ldoCtrlFeature = PMIC_ALL_LDO_ENABLED_LDO_MODE;
        }
        if ((ldoControl->ldo1Ctrl == PMIC_LDO_NOT_ENABLED) ||
                (ldoControl->ldo2Ctrl == PMIC_LDO_NOT_ENABLED) ||
                (ldoControl->ldo3Ctrl == PMIC_LDO_NOT_ENABLED) ||
                (ldoControl->ldo4Ctrl == PMIC_LDO_NOT_ENABLED)) {
            ldoCtrlFeature = PMIC_ALL_LDO_ENABLED_STBY_MODE;
        }
        if ((ldoControl->ldo1Ctrl == PMIC_LDO_NOT_ENABLED) ||
                (ldoControl->ldo2Ctrl == PMIC_LDO_NOT_ENABLED) ||
                (ldoControl->ldo3Ctrl == PMIC_LDO_NOT_ENABLED) ||
                (ldoControl->ldo4Ctrl == PMIC_LDO_NOT_ENABLED)) {
            ldoCtrlFeature = PMIC_ALL_LDO_ENABLED_VMON_MODE;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);
        if (pmicStatus != PMIC_ST_SUCCESS) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        } else {
            Pmic_setBitField(&regData, bitPos, bitMask, ldoCtrlFeature);
            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);
        }
        Pmic_criticalSectionStop(pPmicCoreHandle);

        initializeLDOControlReg(ldoControl);
    }

    return pmicStatus;
}

int32_t Pmic_getLdoCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
                        uint8_t ldoNumber,
                        Pmic_ldoCtrlReg_t *ldoControl)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = PMIC_LDO_CTRL_REGADDR;
    uint8_t regData = 0U;
    uint8_t bitPos = LDO_ALL_CTRL_SHIFT;
    uint8_t bitMask = LDO_ALL_CTRL_MASK;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
        if (pmicStatus != PMIC_ST_SUCCESS) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        } else {
            switch(ldoNumber) {
            case PMIC_LDO1:
                bitPos = LDO1_CTRL_SHIFT;
                bitMask = LDO1_CTRL_MASK;
                ldoControl->ldo1Ctrl = Pmic_getBitField(regData, bitPos, bitMask);
                break;
            case PMIC_LDO2:
                bitPos = LDO2_CTRL_SHIFT;
                bitMask = LDO2_CTRL_MASK;
                ldoControl->ldo2Ctrl = Pmic_getBitField(regData, bitPos, bitMask);
                break;
            case PMIC_LDO3:
                bitPos = LDO3_CTRL_SHIFT;
                bitMask = LDO3_CTRL_MASK;
                ldoControl->ldo3Ctrl = Pmic_getBitField(regData, bitPos, bitMask);
                break;
            case PMIC_LDO4:
                bitPos = LDO4_CTRL_SHIFT;
                bitMask = LDO4_CTRL_MASK;
                ldoControl->ldo4Ctrl = Pmic_getBitField(regData, bitPos, bitMask);
                break;
            }
        }
    }

    return pmicStatus;
}

void Pmic_getEnOutCtrlFields(uint8_t enableNumber, uint8_t * pBitPos,
                             uint8_t * pBitMask)
{
    switch (enableNumber) {
    case PMIC_EN_OUT_ALL:
        *pBitPos = EN_OUTALL_ENABLE_SHIFT;
        *pBitMask = EN_OUTALL_ENABLE_MASK;
        break;
    case PMIC_EN_OUT2:
        *pBitPos = EN_OUT2_ENABLE_SHIFT;
        *pBitMask = EN_OUT2_ENABLE_MASK;
        break;
    case PMIC_EN_OUT1:
        *pBitPos = EN_OUT1_ENABLE_SHIFT;
        *pBitMask = EN_OUT1_ENABLE_MASK;
        break;
    default:
        *pBitPos = (uint8_t) PMIC_ST_DEFAULT_DATA;
        *pBitMask = (uint8_t) PMIC_ST_DEFAULT_DATA;
        break;
    }
}

void Pmic_getPLDOCtrlFields(uint8_t pldoNumber, uint8_t * pBitPos,
    uint8_t * pBitMask) {
    switch (pldoNumber) {
    case PMIC_PLDO1:
        *
        pBitPos = PLDO1_CTRL_SHIFT;
        * pBitMask = PLDO1_CTRL_MASK;
        break;
    case PMIC_PLDO2:
        *
        pBitPos = PLDO2_CTRL_SHIFT;
        * pBitMask = PLDO2_CTRL_MASK;
        break;
    default:
        *
        pBitPos = (uint8_t) PMIC_ST_DEFAULT_DATA;
        * pBitMask = (uint8_t) PMIC_ST_DEFAULT_DATA;
        break;
    }
}

int32_t Pmic_setPowerEnOutCtrlReg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoEnOutCtrlReg_t * pldoEnOutCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoEnOutCfg -> pldoEnOutRegAddr;
    uint8_t regData = 0U;
    uint8_t bitPos = 0U;
    uint8_t bitMask = 0U;
    uint8_t bitVal = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        uint8_t combinedValue =
            (uint8_t)(pldoEnOutCfg -> enOut2Enable << 1) | pldoEnOutCfg -> enOut1Enable;
        switch (combinedValue) {
            /* enOut2Enable = 0, enOut1Enable = 1 */
        case PMIC_ONLY_ENOUT1_EN:
            bitVal = 1U;
            Pmic_getEnOutCtrlFields(PMIC_EN_OUT1, & bitPos, & bitMask);
            break;
            /* enOut2Enable = 1, enOut1Enable = 0 */
        case PMIC_ONLY_ENOUT2_EN:
            bitVal = 1U;
            Pmic_getEnOutCtrlFields(PMIC_EN_OUT2, & bitPos, & bitMask);
            break;
            /* enOut2Enable = 1, enOut1Enable = 1 */
        case PMIC_BOTH_ENOUT1_ENOUT2_EN:
            bitVal = 3U;
            Pmic_getEnOutCtrlFields(PMIC_EN_OUT_ALL, & bitPos, & bitMask);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_FAIL;
            break;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        } else {
            Pmic_setBitField( & regData, bitPos, bitMask, bitVal);

            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);

            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }

    return pmicStatus;
}

int32_t Pmic_setPowerPLDOCtrlReg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoEnOutCtrlReg_t * pldoEnOutCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pldoEnOutCfg -> pldoEnOutRegAddr;
    uint8_t regData = 0U;
    uint8_t bitPos = 0U;
    uint8_t bitMask = 0U;
    uint8_t bitVal = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        if (pldoEnOutCfg -> pldo2Ctrl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
            bitVal = pldoEnOutCfg -> pldo2Ctrl;
            Pmic_getPLDOCtrlFields(PMIC_PLDO2, & bitPos, & bitMask);
        }

        if (pldoEnOutCfg -> pldo1Ctrl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
            bitVal = pldoEnOutCfg -> pldo1Ctrl;
            Pmic_getPLDOCtrlFields(PMIC_PLDO1, & bitPos, & bitMask);
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        } else {
            Pmic_setBitField( & regData, bitPos, bitMask, bitVal);

            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);

            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }

    return pmicStatus;
}

int32_t
Pmic_setPowerPLDOEnOutControl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoEnOutCtrlReg_t * pldoEnOutCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if ((pldoEnOutCfg -> pldo2Ctrl != (uint8_t) PMIC_ST_DEFAULT_DATA) ||
        (pldoEnOutCfg -> pldo1Ctrl != (uint8_t) PMIC_ST_DEFAULT_DATA)) {
        pldoEnOutCfg -> pldoEnOutRegAddr = pwrRsrcRegCfg -> enoutCtrlRegAddr;
        pmicStatus = Pmic_setPowerPLDOCtrlReg(pPmicCoreHandle, pldoEnOutCfg);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    if ((pldoEnOutCfg -> enOut2Enable != (uint8_t) PMIC_ST_DEFAULT_DATA) ||
        (pldoEnOutCfg -> enOut1Enable != (uint8_t) PMIC_ST_DEFAULT_DATA)) {
        pldoEnOutCfg -> pldoEnOutRegAddr = pwrRsrcRegCfg -> enoutCtrlRegAddr;
        pmicStatus = Pmic_setPowerEnOutCtrlReg(pPmicCoreHandle, pldoEnOutCfg);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    return pmicStatus;
}

int32_t Pmic_setPLDODscgDisCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerRsrcCfg_t * dscgConfig,
        const Pmic_DscgDisCtrlReg_t * dscgDisCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = dscgDisCtrlCfg -> dscgDisCtrlRegAddr;
    uint8_t regData = 0U;
    uint8_t bitPos = 0U;
    uint8_t bitMask = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        bitPos = dscgConfig -> pmicConfigShiftVal;
        bitMask = dscgConfig -> pmicConfigMaskVal;

        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (dscgDisCtrlCfg -> pldo2DscgDis != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, bitPos, bitMask,
                    dscgDisCtrlCfg -> pldo2DscgDis);
            }

            if (dscgDisCtrlCfg -> pldo1DscgDis != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, bitPos, bitMask,
                    dscgDisCtrlCfg -> pldo1DscgDis);
            }

            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_setLDODscgDisCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerRsrcCfg_t * dscgConfig,
        const Pmic_DscgDisCtrlReg_t * dscgDisCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = dscgDisCtrlCfg -> dscgDisCtrlRegAddr;
    uint8_t regData = 0U;
    uint8_t bitPos = 0U;
    uint8_t bitMask = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        bitPos = dscgConfig -> pmicConfigShiftVal;
        bitMask = dscgConfig -> pmicConfigMaskVal;

        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (dscgDisCtrlCfg -> ldo4DscgDis != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, bitPos, bitMask,
                    dscgDisCtrlCfg -> ldo4DscgDis);
            }

            if (dscgDisCtrlCfg -> ldo3DscgDis != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, bitPos, bitMask,
                    dscgDisCtrlCfg -> ldo3DscgDis);
            }

            if (dscgDisCtrlCfg -> ldo2DscgDis != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, bitPos, bitMask,
                    dscgDisCtrlCfg -> ldo2DscgDis);
            }

            if (dscgDisCtrlCfg -> ldo1DscgDis != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, bitPos, bitMask,
                    dscgDisCtrlCfg -> ldo1DscgDis);
            }

            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
        } else {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t
Pmic_setPowerDscgDisControl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_DscgDisCtrlReg_t * dscgDisCtrlCfg,
    const Pmic_powerRsrcCfg_t * dscgConfig,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {

        dscgDisCtrlCfg -> dscgDisCtrlRegAddr = pwrRsrcRegCfg -> dscgConfigRegAddr;

        if ((dscgDisCtrlCfg -> pldo1DscgDis != (uint8_t) PMIC_ST_DEFAULT_DATA) ||
            (dscgDisCtrlCfg -> pldo2DscgDis != (uint8_t) PMIC_ST_DEFAULT_DATA)) {
            pmicStatus =
                Pmic_setPLDODscgDisCtrl(pPmicCoreHandle, dscgConfig, dscgDisCtrlCfg);
            if (PMIC_ST_SUCCESS != pmicStatus) {
                pmicStatus = PMIC_ST_ERR_FAIL;
            }
        } else {
            pmicStatus =
                Pmic_setLDODscgDisCtrl(pPmicCoreHandle, dscgConfig, dscgDisCtrlCfg);
            if (PMIC_ST_SUCCESS != pmicStatus) {
                pmicStatus = PMIC_ST_ERR_FAIL;
            }
        }
    }

    return pmicStatus;
}

int32_t Pmic_setldoVmonThresholdConfig(
    Pmic_CoreHandle_t * pPmicCoreHandle, uint8_t ldoNumber,
    const Pmic_ldoVMONThresholdReg_t * ldomonThreshCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonTHCfg1RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        switch (ldoNumber) {
        case PMIC_LDO1:
            Pmic_setBitField( & regData, bitPos, bitMask,
                ldomonThreshCfg -> ldo1vmonthresh);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_LDO2:
            Pmic_setBitField( & regData, bitPos, bitMask,
                ldomonThreshCfg -> ldo2vmonthresh);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_LDO3:
            Pmic_setBitField( & regData, bitPos, bitMask,
                ldomonThreshCfg -> ldo3vmonthresh);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_LDO4:
            Pmic_setBitField( & regData, bitPos, bitMask,
                ldomonThreshCfg -> ldo4vmonthresh);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t
Pmic_getldoVmonThresholdConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t ldoNumber,
    Pmic_ldoVMONThresholdReg_t * ldomonThreshCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonTHCfg1RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        switch (ldoNumber) {
        case PMIC_LDO1:
            ldomonThreshCfg -> ldo1vmonthresh =
                Pmic_getBitField(regData, bitPos, bitMask);
            break;
        case PMIC_LDO2:
            ldomonThreshCfg -> ldo2vmonthresh =
                Pmic_getBitField(regData, bitPos, bitMask);
            break;
        case PMIC_LDO3:
            ldomonThreshCfg -> ldo3vmonthresh =
                Pmic_getBitField(regData, bitPos, bitMask);
            break;
        case PMIC_LDO4:
            ldomonThreshCfg -> ldo4vmonthresh =
                Pmic_getBitField(regData, bitPos, bitMask);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }
    }

    return pmicStatus;
}

int32_t
Pmic_setpldoVMONThreshConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    const Pmic_extpldoVMONThreshReg_t * pldovmonthCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg2RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        switch (pldoNumber) {
        case PMIC_PLDO1:
            Pmic_setBitField( & regData, bitPos, bitMask,
                pldovmonthCfg -> pldo1vmonthresh);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_PLDO2:
            Pmic_setBitField( & regData, bitPos, bitMask,
                pldovmonthCfg -> pldo2vmonthresh);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t
Pmic_getpldoVMONThreshConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    Pmic_extpldoVMONThreshReg_t * pldovmonthCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg2RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        switch (pldoNumber) {
        case PMIC_PLDO1:
            pldovmonthCfg -> pldo1vmonthresh =
                Pmic_getBitField(regData, bitPos, bitMask);
            break;
        case PMIC_PLDO2:
            pldovmonthCfg -> pldo2vmonthresh =
                Pmic_getBitField(regData, bitPos, bitMask);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }
    }

    return pmicStatus;
}

int32_t
Pmic_setextVMONThreshConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t extVmonNumber,
    const Pmic_extpldoVMONThreshReg_t * extvmonthCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg2RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        switch (extVmonNumber) {
        case PMIC_EXT_VMON1:
            Pmic_setBitField( & regData, bitPos, bitMask, extvmonthCfg -> extvmon1thresh);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_EXT_VMON2:
            Pmic_setBitField( & regData, bitPos, bitMask, extvmonthCfg -> extvmon2thresh);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t
Pmic_getextVMONThreshConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t extVmonNumber,
    Pmic_extpldoVMONThreshReg_t * extvmonthCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg2RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        switch (extVmonNumber) {
        case PMIC_EXT_VMON1:
            extvmonthCfg -> extvmon1thresh = Pmic_getBitField(regData, bitPos, bitMask);
            break;
        case PMIC_EXT_VMON2:
            extvmonthCfg -> extvmon2thresh = Pmic_getBitField(regData, bitPos, bitMask);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }
    }

    return pmicStatus;
}

int32_t Pmic_setbbTimeoutConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_BuckBoostVMONConfigReg_t * bbtmoCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonTHCfg3RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            Pmic_setBitField( & regData, bitPos, bitMask, bbtmoCfg -> buckBoostTmoCfg);
        }
        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_getbbTimeoutConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_BuckBoostVMONConfigReg_t * bbtmoCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonTHCfg3RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            bbtmoCfg -> buckBoostTmoCfg = Pmic_getBitField(regData, bitPos, bitMask);
        }
    }

    return pmicStatus;
}

int32_t Pmic_setbbVmonThConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_BuckBoostVMONConfigReg_t * bbtmoCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonTHCfg3RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            Pmic_setBitField( & regData, bitPos, bitMask, bbtmoCfg -> buckBoostVmonTh);
        }
        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_getbbVmonThConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_BuckBoostVMONConfigReg_t * bbtmoCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonTHCfg3RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            bbtmoCfg -> buckBoostVmonTh = Pmic_getBitField(regData, bitPos, bitMask);
        }
    }

    return pmicStatus;
}

int32_t
Pmic_setBBVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_VbatBBVMONDglReg_t * bbvmondglCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg1RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {

        if (bbvmondglCfg -> bbvmondgl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        } else {
            Pmic_criticalSectionStart(pPmicCoreHandle);

            pmicStatus =
                Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

            if (PMIC_ST_SUCCESS == pmicStatus) {
                Pmic_setBitField( & regData, bitPos, bitMask, bbvmondglCfg -> bbvmondgl);
            }
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);

            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }

    return pmicStatus;
}

int32_t
Pmic_getBBVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg,
        uint8_t * bbVmonDglStatus) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg1RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
        if (pmicStatus != PMIC_ST_SUCCESS) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        } else {
            * bbVmonDglStatus = Pmic_getBitField(regData, bitPos, bitMask);
            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }

    return pmicStatus;
}

int32_t
Pmic_setVbatVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_VbatBBVMONDglReg_t * vbatvmondglCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg1RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            Pmic_setBitField( & regData, bitPos, bitMask, vbatvmondglCfg -> vbatvmondgl);
        }
        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t
Pmic_getVbatVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg,
        uint8_t * VbatVmonDglStatus) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg1RegAddr;
    uint8_t bitPos = pwrRsrcRegCfg -> bitPosVal;
    uint8_t bitMask = pwrRsrcRegCfg -> bitMaskVal;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
        if (pmicStatus != PMIC_ST_SUCCESS) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        } else {
            * VbatVmonDglStatus = Pmic_getBitField(regData, bitPos, bitMask);
            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }

    return pmicStatus;
}

int32_t
Pmic_setldoVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t ldoNumber,
    const Pmic_ldoVMONDglReg_t * ldovmondglCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg2RegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        switch (ldoNumber) {
        case PMIC_LDO1:
            Pmic_setBitField( & regData, pwrRsrcRegCfg -> bitPosVal,
                pwrRsrcRegCfg -> bitMaskVal, ldovmondglCfg -> ldo1vmondgl);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_LDO2:
            Pmic_setBitField( & regData, pwrRsrcRegCfg -> bitPosVal,
                pwrRsrcRegCfg -> bitMaskVal, ldovmondglCfg -> ldo2vmondgl);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_LDO3:
            Pmic_setBitField( & regData, pwrRsrcRegCfg -> bitPosVal,
                pwrRsrcRegCfg -> bitMaskVal, ldovmondglCfg -> ldo3vmondgl);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_LDO4:
            Pmic_setBitField( & regData, pwrRsrcRegCfg -> bitPosVal,
                pwrRsrcRegCfg -> bitMaskVal, ldovmondglCfg -> ldo4vmondgl);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t
Pmic_getldoVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t ldoNumber,
    Pmic_ldoVMONDglReg_t * ldovmondglCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg2RegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        switch (ldoNumber) {
        case PMIC_LDO1:
            ldovmondglCfg -> ldo1vmondgl = Pmic_getBitField(
                regData, pwrRsrcRegCfg -> bitPosVal, pwrRsrcRegCfg -> bitMaskVal);
            break;
        case PMIC_LDO2:
            ldovmondglCfg -> ldo2vmondgl = Pmic_getBitField(
                regData, pwrRsrcRegCfg -> bitPosVal, pwrRsrcRegCfg -> bitMaskVal);
            break;
        case PMIC_LDO3:
            ldovmondglCfg -> ldo3vmondgl = Pmic_getBitField(
                regData, pwrRsrcRegCfg -> bitPosVal, pwrRsrcRegCfg -> bitMaskVal);
            break;
        case PMIC_LDO4:
            ldovmondglCfg -> ldo4vmondgl = Pmic_getBitField(
                regData, pwrRsrcRegCfg -> bitPosVal, pwrRsrcRegCfg -> bitMaskVal);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }
    }

    return pmicStatus;
}

int32_t
Pmic_setpldoVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    const Pmic_extpldoVMONDglReg_t * pldovmondglCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg3RegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        switch (pldoNumber) {
        case PMIC_PLDO1:
            Pmic_setBitField( & regData, pwrRsrcRegCfg -> bitPosVal,
                pwrRsrcRegCfg -> bitMaskVal, pldovmondglCfg -> pldo1vmondgl);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_PLDO2:
            Pmic_setBitField( & regData, pwrRsrcRegCfg -> bitPosVal,
                pwrRsrcRegCfg -> bitMaskVal, pldovmondglCfg -> pldo2vmondgl);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t
Pmic_getpldoVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    Pmic_extpldoVMONDglReg_t * pldovmondglCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg3RegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        switch (pldoNumber) {
        case PMIC_PLDO1:
            pldovmondglCfg -> pldo1vmondgl = Pmic_getBitField(
                regData, pwrRsrcRegCfg -> bitPosVal, pwrRsrcRegCfg -> bitMaskVal);
            break;
        case PMIC_PLDO2:
            pldovmondglCfg -> pldo2vmondgl = Pmic_getBitField(
                regData, pwrRsrcRegCfg -> bitPosVal, pwrRsrcRegCfg -> bitMaskVal);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }
    }

    return pmicStatus;
}

int32_t
Pmic_setextVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t extVmonNumber,
    const Pmic_extpldoVMONDglReg_t * extvmondglCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg3RegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        switch (extVmonNumber) {
        case PMIC_EXT_VMON1:
            Pmic_setBitField( & regData, pwrRsrcRegCfg -> bitPosVal,
                pwrRsrcRegCfg -> bitMaskVal, extvmondglCfg -> extvmon1dgl);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        case PMIC_EXT_VMON2:
            Pmic_setBitField( & regData, pwrRsrcRegCfg -> bitPosVal,
                pwrRsrcRegCfg -> bitMaskVal, extvmondglCfg -> extvmon2dgl);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t
Pmic_getextVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t extVmonNumber,
    Pmic_extpldoVMONDglReg_t * extvmondglCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = pwrRsrcRegCfg -> vmonDglCfg3RegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        switch (extVmonNumber) {
        case PMIC_EXT_VMON1:
            extvmondglCfg -> extvmon1dgl = Pmic_getBitField(
                regData, pwrRsrcRegCfg -> bitPosVal, pwrRsrcRegCfg -> bitMaskVal);
            break;
        case PMIC_EXT_VMON2:
            extvmondglCfg -> extvmon2dgl = Pmic_getBitField(
                regData, pwrRsrcRegCfg -> bitPosVal, pwrRsrcRegCfg -> bitMaskVal);
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
            break;
        }
    }

    return pmicStatus;
}

int32_t Pmic_SetLPExtVMonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpVMonCtrlCfg -> lpVmonCtrlRegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (lpVMonCtrlCfg -> lpExtVMon2Ctrl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, PMIC_LP_EXT_VMON2_CTRL_SHIFT,
                    PMIC_LP_EXT_VMON2_CTRL_MASK,
                    lpVMonCtrlCfg -> lpExtVMon2Ctrl);
            }

            if (lpVMonCtrlCfg -> lpExtVMon1Ctrl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, PMIC_LP_EXT_VMON1_CTRL_SHIFT,
                    PMIC_LP_EXT_VMON1_CTRL_MASK,
                    lpVMonCtrlCfg -> lpExtVMon1Ctrl);
            }
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }
    return pmicStatus;
}

int32_t Pmic_SetLPPLDOVMonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpVMonCtrlCfg -> lpVmonCtrlRegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (lpVMonCtrlCfg -> lppldo2VMonCtrl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, PMIC_LP_PLDO2_VMON_CTRL_SHIFT,
                    PMIC_LP_PLDO2_VMON_CTRL_MASK,
                    lpVMonCtrlCfg -> lppldo2VMonCtrl);
            }

            if (lpVMonCtrlCfg -> lppldo1VMonCtrl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, PMIC_LP_PLDO1_VMON_CTRL_SHIFT,
                    PMIC_LP_PLDO1_VMON_CTRL_MASK,
                    lpVMonCtrlCfg -> lppldo1VMonCtrl);
            }
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }
    return pmicStatus;
}

int32_t Pmic_SetLPLDOVMonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpVMonCtrlCfg -> lpVmonCtrlRegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (lpVMonCtrlCfg -> lpldo4VMonCtrl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, PMIC_LP_LDO4_VMON_CTRL_SHIFT,
                    PMIC_LP_LDO4_VMON_CTRL_MASK,
                    lpVMonCtrlCfg -> lpldo4VMonCtrl);
            }

            if (lpVMonCtrlCfg -> lpldo3VMonCtrl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, PMIC_LP_LDO3_VMON_CTRL_SHIFT,
                    PMIC_LP_LDO3_VMON_CTRL_MASK,
                    lpVMonCtrlCfg -> lpldo3VMonCtrl);
            }
            if (lpVMonCtrlCfg -> lpldo2VMonCtrl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, PMIC_LP_LDO2_VMON_CTRL_SHIFT,
                    PMIC_LP_LDO2_VMON_CTRL_MASK,
                    lpVMonCtrlCfg -> lpldo2VMonCtrl);
            }

            if (lpVMonCtrlCfg -> lpldo1VMonCtrl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, PMIC_LP_LDO1_VMON_CTRL_SHIFT,
                    PMIC_LP_LDO1_VMON_CTRL_MASK,
                    lpVMonCtrlCfg -> lpldo1VMonCtrl);
            }
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }
    return pmicStatus;
}

int32_t Pmic_SetLowPowerVmonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        lpVMonCtrlCfg -> lpVmonCtrlRegAddr = PMIC_LP_VMON_CTRL_REGADDR;

        pmicStatus = Pmic_SetLPExtVMonCtrl(pPmicCoreHandle, lpVMonCtrlCfg);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
        pmicStatus = Pmic_SetLPPLDOVMonCtrl(pPmicCoreHandle, lpVMonCtrlCfg);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
        pmicStatus = Pmic_SetLPLDOVMonCtrl(pPmicCoreHandle, lpVMonCtrlCfg);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }
    return pmicStatus;
}

int32_t Pmic_GetLPExtVMonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpVMonCtrlCfg -> lpVmonCtrlRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        lpVMonCtrlCfg -> lpExtVMon1Ctrl = Pmic_getBitField(
            regData, PMIC_LP_EXT_VMON1_CTRL_SHIFT, PMIC_LP_EXT_VMON1_CTRL_MASK);
        lpVMonCtrlCfg -> lpExtVMon2Ctrl = Pmic_getBitField(
            regData, PMIC_LP_EXT_VMON2_CTRL_SHIFT, PMIC_LP_EXT_VMON2_CTRL_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_GetLPPLDOVMonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpVMonCtrlCfg -> lpVmonCtrlRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        lpVMonCtrlCfg -> lppldo1VMonCtrl = Pmic_getBitField(
            regData, PMIC_LP_PLDO1_VMON_CTRL_SHIFT, PMIC_LP_PLDO1_VMON_CTRL_MASK);
        lpVMonCtrlCfg -> lppldo2VMonCtrl = Pmic_getBitField(
            regData, PMIC_LP_PLDO2_VMON_CTRL_SHIFT, PMIC_LP_PLDO2_VMON_CTRL_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_GetLPLDOVMonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpVMonCtrlCfg -> lpVmonCtrlRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        lpVMonCtrlCfg -> lpldo1VMonCtrl = Pmic_getBitField(
            regData, PMIC_LP_LDO1_VMON_CTRL_SHIFT, PMIC_LP_LDO1_VMON_CTRL_SHIFT);
        lpVMonCtrlCfg -> lpldo2VMonCtrl = Pmic_getBitField(
            regData, PMIC_LP_LDO2_VMON_CTRL_SHIFT, PMIC_LP_LDO2_VMON_CTRL_MASK);
        lpVMonCtrlCfg -> lpldo3VMonCtrl = Pmic_getBitField(
            regData, PMIC_LP_LDO3_VMON_CTRL_SHIFT, PMIC_LP_LDO3_VMON_CTRL_MASK);
        lpVMonCtrlCfg -> lpldo4VMonCtrl = Pmic_getBitField(
            regData, PMIC_LP_LDO4_VMON_CTRL_SHIFT, PMIC_LP_LDO4_VMON_CTRL_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_GetLowPowerVmonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        lpVMonCtrlCfg -> lpVmonCtrlRegAddr = PMIC_LP_VMON_CTRL_REGADDR;

        pmicStatus = Pmic_GetLPExtVMonCtrl(pPmicCoreHandle, lpVMonCtrlCfg);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
        pmicStatus = Pmic_GetLPPLDOVMonCtrl(pPmicCoreHandle, lpVMonCtrlCfg);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
        pmicStatus = Pmic_GetLPLDOVMonCtrl(pPmicCoreHandle, lpVMonCtrlCfg);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }
    return pmicStatus;
}

int32_t Pmic_SetlpBBOVPCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpConfigCtrlReg_t * lpConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpConfig -> lpCfgRegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (lpConfig -> lpBBOVPCtrl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, PMIC_LP_BB_OVP_CTRL_SHIFT,
                    PMIC_LP_BB_OVP_CTRL_SHIFT, lpConfig -> lpBBOVPCtrl);
            }
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }
    return pmicStatus;
}

int32_t Pmic_SetlpBBVmonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpConfigCtrlReg_t * lpConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpConfig -> lpCfgRegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (lpConfig -> lpBBVMonCtrl != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, PMIC_LP_BB_VMON_CTRL_SHIFT,
                    PMIC_LP_BB_VMON_CTRL_MASK, lpConfig -> lpBBVMonCtrl);
            }
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }
    return pmicStatus;
}

int32_t Pmic_SetlpTSDperCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpConfigCtrlReg_t * lpConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpConfig -> lpCfgRegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (lpConfig -> lpTSDperConfig != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, PMIC_LP_TSD_PER_CFG_SHIFT,
                    PMIC_LP_TSD_PER_CFG_MASK, lpConfig -> lpTSDperConfig);
            }
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }
    return pmicStatus;
}

int32_t Pmic_SetlpVmonperCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpConfigCtrlReg_t * lpConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpConfig -> lpCfgRegAddr;
    uint8_t regData = 0U;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (lpConfig -> lpVMonperConfig != (uint8_t) PMIC_ST_DEFAULT_DATA) {
                Pmic_setBitField( & regData, PMIC_LP_VMON_PER_CFG_SHIFT,
                    PMIC_LP_VMON_PER_CFG_MASK, lpConfig -> lpVMonperConfig);
            }
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }
    return pmicStatus;
}

int32_t Pmic_SetLowPowerConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpConfigCtrlReg_t * lpConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        lpConfig -> lpCfgRegAddr = PMIC_LP_CFG_REGADDR;

        pmicStatus = Pmic_SetlpBBOVPCtrl(pPmicCoreHandle, lpConfig);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
        pmicStatus = Pmic_SetlpBBVmonCtrl(pPmicCoreHandle, lpConfig);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
        pmicStatus = Pmic_SetlpTSDperCfg(pPmicCoreHandle, lpConfig);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
        pmicStatus = Pmic_SetlpVmonperCfg(pPmicCoreHandle, lpConfig);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }
    return pmicStatus;
}

int32_t Pmic_GetlpBBOVPCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpConfigCtrlReg_t * lpConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpConfig -> lpCfgRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        lpConfig -> lpBBOVPCtrl = Pmic_getBitField(regData, PMIC_LP_BB_OVP_CTRL_SHIFT,
            PMIC_LP_BB_OVP_CTRL_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_GetlpBBVmonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpConfigCtrlReg_t * lpConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpConfig -> lpCfgRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        lpConfig -> lpBBVMonCtrl = Pmic_getBitField(
            regData, PMIC_LP_BB_VMON_CTRL_SHIFT, PMIC_LP_BB_VMON_CTRL_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_GetlpTSDperCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpConfigCtrlReg_t * lpConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpConfig -> lpCfgRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        lpConfig -> lpTSDperConfig = Pmic_getBitField(
            regData, PMIC_LP_TSD_PER_CFG_SHIFT, PMIC_LP_TSD_PER_CFG_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_GetlpVmonperCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpConfigCtrlReg_t * lpConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = lpConfig -> lpCfgRegAddr;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        lpConfig -> lpVMonperConfig = Pmic_getBitField(
            regData, PMIC_LP_VMON_PER_CFG_SHIFT, PMIC_LP_VMON_PER_CFG_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_GetLowPowerConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpConfigCtrlReg_t * lpConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (pPmicCoreHandle == NULL) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        lpConfig -> lpCfgRegAddr = PMIC_LP_CFG_REGADDR;

        pmicStatus = Pmic_GetlpBBOVPCtrl(pPmicCoreHandle, lpConfig);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
        pmicStatus = Pmic_GetlpBBVmonCtrl(pPmicCoreHandle, lpConfig);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
        pmicStatus = Pmic_GetlpTSDperCfg(pPmicCoreHandle, lpConfig);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
        pmicStatus = Pmic_GetlpVmonperCfg(pPmicCoreHandle, lpConfig);
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }
    return pmicStatus;
}

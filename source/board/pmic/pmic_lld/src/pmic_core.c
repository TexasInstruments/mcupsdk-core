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
 *  @file  pmic_core.c
 *
 *  @brief This file contains PMIC generic driver APIs
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_core.h"

#include "pmic_core_priv.h"

#include "pmic_core_tps65386x.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

const Pmic_DiagMUXFeatureMapping Pmic_amuxFeatureMappings[AMUX_NUM_FEATURES] = {
    [FEATURE_BUCK_BOOST_OUTPUT_VOLTAGE] = {1, NULL},
    [FEATURE_LDO1_OUTPUT_VOLTAGE] = {1,1},
    [FEATURE_LDO2_OUTPUT_VOLTAGE] = {1,2},
    [FEATURE_LDO3_OUTPUT_VOLTAGE] = {1,3},
    [FEATURE_LDO4_OUTPUT_VOLTAGE] = {1,4},
    [FEATURE_PROTECTED_PLDO1_OUTPUT_VOLTAGE] = {1,5},
    [FEATURE_PROTECTED_PLDO2_OUTPUT_VOLTAGE] = {1,6},
    [FEATURE_VOLTAGE_ON_TRACK_PIN] = {1,7},
    [FEATURE_SECONDARY_BATTERY_SUPPLY_VOLTAGE] = {1,8},
    [FEATURE_POWER_BATTERY_SUPPLY_VOLTAGE] = {1,9},
    [FEATURE_MAIN_BANDGAP] = {1,10},
    [FEATURE_COMPARE_BANDGAP] = {1,11},
    [FEATURE_TEMP_SENSOR_BUCK_BOOST] = {12,NULL},
    [FEATURE_TEMP_SENSOR_LDO1] = {12,1},
    [FEATURE_TEMP_SENSOR_LDO2] = {12,2},
    [FEATURE_TEMP_SENSOR_LDO3] = {12,3},
    [FEATURE_TEMP_SENSOR_LDO4] = {13,NULL},
    [FEATURE_TEMP_SENSOR_PLDO1] = {13,1},
    [FEATURE_TEMP_SENSOR_PLDO2] = {13,2}
};

/* Define the lookup table mapping features to group and channel numbers */
const Pmic_DiagMUXFeatureMapping Pmic_dmuxFeatureMappings[DMUX_NUM_FEATURES] = {
    [FEATURE_DIGITAL_0_OUTPUT] = {NULL,NULL},
    [FEATURE_DIGITAL_1_OUTPUT] = {NULL,1},
    [FEATURE_BUCK_BOOST_AVG_CURRENT_LIMIT] = {1,NULL},
    [FEATURE_BUCK_BOOST_DEGLITCHED_AVG_CURRENT_LIMIT] = {1,1},
    [FEATURE_BUCK_BOOST_PEAK_CURRENT_LIMIT] = {1,2},
    [FEATURE_BUCK_BOOST_DEGLITCHED_PEAK_CURRENT_LIMIT] = {1,3},
    [FEATURE_RESERVED_2] = {2,NULL},
    [FEATURE_LDO1_CURRENT_LIMIT] = {3,NULL},
    [FEATURE_LDO1_DEGLITCHED_CURRENT_LIMIT] = {3,1},
    [FEATURE_LDO1_BYPASS_ENABLE] = {3,2},
    [FEATURE_LDO2_CURRENT_LIMIT] = {4,NULL},
    [FEATURE_LDO2_DEGLITCHED_CURRENT_LIMIT] = {4,1},
    [FEATURE_LDO2_BYPASS_ENABLE] = {4,2},
    [FEATURE_LDO3_CURRENT_LIMIT] = {5,NULL},
    [FEATURE_LDO3_DEGLITCHED_CURRENT_LIMIT] = {5,1},
    [FEATURE_LDO3_BYPASS_ENABLE] = {5,2},
    [FEATURE_LDO4_CURRENT_LIMIT] = {6,NULL},
    [FEATURE_LDO4_DEGLITCHED_CURRENT_LIMIT] = {6,1},
    [FEATURE_LDO4_BYPASS_ENABLE] = {6,2},
    [FEATURE_PLDO1_CURRENT_LIMIT] = {7,NULL},
    [FEATURE_PLDO1_DEGLITCHED_CURRENT_LIMIT] = {7,1},
    [FEATURE_RESERVED_22] = {7,2},
    [FEATURE_PLDO1_TRACKING_MODE_ENABLE] = {7,3},
    [FEATURE_PLDO2_CURRENT_LIMIT] = {8,NULL},
    [FEATURE_PLDO2_DEGLITCHED_CURRENT_LIMIT] = {8,1},
    [FEATURE_RESERVED_25] = {8,2},
    [FEATURE_PLDO2_TRACKING_MODE_ENABLE] = {8,3},
    [FEATURE_RESERVED_28] = {8,4},
    [FEATURE_RESERVED_29] = {9,NULL},
    [FEATURE_VBAT_DEGLITCHED_OV] = {9,1},
    [FEATURE_VBAT_DEGLITCHED_OVP] = {9,2},
    [FEATURE_VBAT_DEGLITCHED_UV] = {9,3},
    [FEATURE_BUCK_BOOST_DEGLITCHED_OV] = {9,4},
    [FEATURE_BUCK_BOOST_DEGLITCHED_OVP] = {9,5},
    [FEATURE_BUCK_BOOST_DEGLITCHED_UV] = {9,6},
    [FEATURE_RESERVED_37] = {9,7},
    [FEATURE_RESERVED_38] = {9,8},
    [FEATURE_LDO1_DEGLITCHED_UV] = {9,9},
    [FEATURE_LDO2_DEGLITCHED_UV] = {9,10},
    [FEATURE_LDO3_DEGLITCHED_UV] = {9,11},
    [FEATURE_LDO4_DEGLITCHED_UV] = {9,12},
    [FEATURE_LDO1_DEGLITCHED_OV] = {9,13},
    [FEATURE_LDO2_DEGLITCHED_OV] = {9,14},
    [FEATURE_LDO3_DEGLITCHED_OV] = {9,15},
    [FEATURE_LDO4_DEGLITCHED_OV] = {9,16},
    [FEATURE_PLDO1_DEGLITCHED_UV] = {9,17},
    [FEATURE_PLDO2_DEGLITCHED_UV] = {9,18},
    [FEATURE_PLDO1_DEGLITCHED_OV] = {9,19},
    [FEATURE_PLDO2_DEGLITCHED_OV] = {9,20},
    [FEATURE_EXT_VMON1_DEGLITCHED_UV] = {9,21},
    [FEATURE_EXT_VMON2_DEGLITCHED_UV] = {9,22},
    [FEATURE_EXT_VMON1_DEGLITCHED_OV] = {9,23},
    [FEATURE_EXT_VMON2_DEGLITCHED_OV] = {10,NULL},
    [FEATURE_VBAT_OV] = {10,1},
    [FEATURE_VBAT_OVP] = {10,2},
    [FEATURE_VBAT_UV] = {10,3},
    [FEATURE_BUCK_BOOST_OV] = {10,4},
    [FEATURE_BUCK_BOOST_OVP] = {10,5},
    [FEATURE_BUCK_BOOST_UV] = {10,6},
    [FEATURE_RESERVED_67] = {10,7},
    [FEATURE_RESERVED_68] = {10,8},
    [FEATURE_LDO1_UV] = {10,9},
    [FEATURE_LDO2_UV] = {10,10},
    [FEATURE_LDO3_UV] = {10,11},
    [FEATURE_LDO4_UV] = {10,12},
    [FEATURE_LDO1_OV] = {10,13},
    [FEATURE_LDO2_OV] = {10,14},
    [FEATURE_LDO3_OV] = {10,15},
    [FEATURE_LDO4_OV] = {10,16},
    [FEATURE_PLDO1_UV] = {10,17},
    [FEATURE_PLDO2_UV] = {10,18},
    [FEATURE_PLDO1_OV] = {10,19},
    [FEATURE_PLDO2_OV] = {10,20},
    [FEATURE_EXT_VMON1_UV] = {10,21},
    [FEATURE_EXT_VMON2_UV] = {10,22},
    [FEATURE_EXT_VMON1_OV] = {10,23},
    [FEATURE_EXT_VMON2_OV] = {11,NULL},
    [FEATURE_BUCK_BOOST_TEMP_PREWARNING] = {11,1},
    [FEATURE_BUCK_BOOST_OVERTEMP_SHUTDOWN] = {11,2},
    [FEATURE_RESERVED_90] = {11,3},
    [FEATURE_RESERVED_91] = {11,4},
    [FEATURE_LDO1_TEMP_PREWARNING] = {11,5},
    [FEATURE_LDO1_OVERTEMP_SHUTDOWN] = {11,6},
    [FEATURE_LDO2_TEMP_PREWARNING] = {11,7},
    [FEATURE_LDO2_OVERTEMP_SHUTDOWN] = {11,8},
    [FEATURE_LDO3_TEMP_PREWARNING] = {11,9},
    [FEATURE_LDO3_OVERTEMP_SHUTDOWN] = {11,10},
    [FEATURE_LDO4_TEMP_PREWARNING] = {11,11},
    [FEATURE_LDO4_OVERTEMP_SHUTDOWN] = {11,12},
    [FEATURE_PLDO1_TEMP_PREWARNING] = {11,13},
    [FEATURE_PLDO1_OVERTEMP_SHUTDOWN] = {11,14},
    [FEATURE_PLDO2_TEMP_PREWARNING] = {11,15},
    [FEATURE_PLDO2_OVERTEMP_SHUTDOWN] = {12,NULL},
    [FEATURE_VREG_OVP_MONITOR_1] = {12,1},
    [FEATURE_VREG_UVLO_MONITOR_1] = {12,2},
    [FEATURE_VREG_SAFETY_OVP_MONITOR_1] = {12,3},
    [FEATURE_VREG_SAFETY_UVLO_MONITOR_1] = {12,4},
    [FEATURE_VREG_1P8_OVP_MONITOR_1] = {12,5},
    [FEATURE_VREG_1P8_UVLO_MONITOR_1] = {12,6},
    [FEATURE_VREG_OVP_MONITOR_2] = {12,7},
    [FEATURE_VREG_UVLO_MONITOR_2] = {12,8},
    [FEATURE_VREG_SAFETY_OVP_MONITOR_2] = {12,9},
    [FEATURE_VREG_SAFETY_UVLO_MONITOR_2] = {12,10},
    [FEATURE_VREG_1P8_OVP_MONITOR_2] = {12,11},
    [FEATURE_VREG_1P8_UVLO_MONITOR_2] = {12,12},
    [FEATURE_125KHZ_CLOCK] = {12,13},
    [FEATURE_250KHZ_CLOCK] = {12,14},
    [FEATURE_500KHZ_CLOCK] = {12,15},
    [FEATURE_1_25MHZ_CLOCK] = {12,16},
    [FEATURE_10MHZ_CLOCK] = {13,0},
    [FEATURE_GPI1_INPUT_LEVEL] = {13,1},
    [FEATURE_GPI2_INPUT_LEVEL] = {13,2},
    [FEATURE_GPI3_INPUT_LEVEL] = {13,3},
    [FEATURE_GPI4_INPUT_LEVEL] = {13,4},
    [FEATURE_GPI5_INPUT_LEVEL] = {13,5},
    [FEATURE_WAKE1_INPUT_LEVEL] = {13,6},
    [FEATURE_WAKE2_INPUT_LEVEL] = {13,7},
    [FEATURE_RESERVED_122] = {13,8},
    [FEATURE_EXT_VMON1_INPUT_DIGITAL_MODE] = {13,9},
    [FEATURE_EXT_VMON2_INPUT_DIGITAL_MODE] = {13,10},
    [FEATURE_EN_OUT_READBACK_LEVEL] = {13,11},
    [FEATURE_GPO1_READBACK_LEVEL] = {13,12},
    [FEATURE_GPO2_READBACK_LEVEL] = {13,13},
    [FEATURE_GPO3_READBACK_LEVEL] = {13,14},
    [FEATURE_GPO4_READBACK_LEVEL] = {13,15},
    [FEATURE_NRST_READBACK_LEVEL] = {13,16},
    [FEATURE_SAFE_OUT1_READBACK_LEVEL] = {13,17},
    [FEATURE_SPI_NCS_INPUT_LEVEL] = {13,18},
    [FEATURE_SPI_SCLK_INPUT_LEVEL] = {13,19},
    [FEATURE_SPI_SDI_INPUT_LEVEL] = {13,20},
    [FEATURE_SPI_SDO_READBACK_LEVEL] = {13,21}
};

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
const Pmic_DevSubSysInfo_t pmicSubSysInfo[] = {
    /* PMIC_DEV_BB_TPS65386x */
    {
        .gpioEnable = true,
            .rtcEnable = true,
            .wdgEnable = true,
            .buckEnable = true,
            .ldoEnable = true,
            .esmEnable = true
    }
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * @brief Check if a specific bit position in a parameter validity value is set.
 * This function checks whether a specific bit position in a parameter validity
 * value is set.
 *
 * @param validParamVal Validity parameter value to check.
 * @param bitPos Bit position to check.
 * @return bool True if the specified bit is set, false otherwise.
 */
bool pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos) {
    bool retVal = false;

    if (((validParamVal >> bitPos) & 0x01U) != 0U) {
        retVal = true;
    }

    return retVal;
}

/**
 * @brief Start a critical section for PMIC operations.
 * This function starts a critical section for PMIC operations, if the critical
 * section start function pointer is not NULL.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return void No return value.
 */
void Pmic_criticalSectionStart(const Pmic_CoreHandle_t * pPmicCoreHandle) {
    if (NULL != pPmicCoreHandle -> pFnPmicCritSecStart) {
        pPmicCoreHandle -> pFnPmicCritSecStart();
    }
}

/**
 * @brief Stop a critical section for PMIC operations.
 * This function stops a critical section for PMIC operations, if the critical
 * section stop function pointer is not NULL.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return void No return value.
 */
void Pmic_criticalSectionStop(const Pmic_CoreHandle_t * pPmicCoreHandle) {
    if (NULL != pPmicCoreHandle -> pFnPmicCritSecStop) {
        pPmicCoreHandle -> pFnPmicCritSecStop();
    }
}

/**
 * @brief Set register lock/unlock configuration.
 * This function sets the register lock/unlock configuration based on the
 * provided parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param commonCtrlCfg Common control configuration structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setRegisterLockUnlock(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_CommonCtrlCfg_t commonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_sendByte(
            pPmicCoreHandle, PMIC_REGISTER_UNLOCK_REGADDR, commonCtrlCfg.regLock_1);

        pmicStatus = Pmic_commIntf_sendByte(
            pPmicCoreHandle, PMIC_REGISTER_UNLOCK_REGADDR, commonCtrlCfg.regLock_2);

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Set counter lock/unlock configuration.
 * This function sets the counter lock/unlock configuration based on the
 * provided parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param commonCtrlCfg Common control configuration structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setCounterLockUnlock(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_CommonCtrlCfg_t commonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_TMR_COUNTER_UNLOCK_REGADDR,
                commonCtrlCfg.cntLock_1);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_TMR_COUNTER_UNLOCK_REGADDR,
                commonCtrlCfg.cntLock_2);

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Get register lock status.
 * This function retrieves the register lock status.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonCtrlStat Pointer to the common control status structure to
 * store the retrieved status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getRegLockStat(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonCtrlStat_t * pCommonCtrlStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_REG_LOCK_STATUS_REGADDR, &
            pCommonCtrlStat -> cfgregLockStat);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pCommonCtrlStat -> cfgregLockStat = Pmic_getBitField(
            pCommonCtrlStat -> cfgregLockStat, PMIC_CFGREG_LOCKED_STATUS_SHIFT,
            PMIC_CFGREG_LOCK_STATUS_RD_MASK);
    }

    return pmicStatus;
}

/**
 * @brief Get timer counter lock status.
 * This function retrieves the timer counter lock status.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonCtrlStat Pointer to the common control status structure to
 * store the retrieved status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getTmrCntLockStat(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonCtrlStat_t * pCommonCtrlStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_REG_LOCK_STATUS_REGADDR, &
            pCommonCtrlStat -> cntregLockStat);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pCommonCtrlStat -> cntregLockStat = Pmic_getBitField(
            pCommonCtrlStat -> cntregLockStat, PMIC_CNTREG_LOCKED_STATUS_SHIFT,
            PMIC_CNTREG_LOCK_STATUS_RD_MASK);
    }

    return pmicStatus;
}

/**
 * @brief Get the MCU reset counter value.
 * This function retrieves the MCU reset counter value.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pRecovCntVal Pointer to store the recovered counter value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getRstmcuCnt(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * pRecovCntVal) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pRecovCntVal)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_STATE_STAT_REGADDR, & regVal);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        * pRecovCntVal =
            Pmic_getBitField(regVal, PMIC_RST_MCU_CNT_SHIFT, PMIC_RST_MCU_CNT_MASK);
    }

    return pmicStatus;
}

/**
 * @brief Set state status register.
 * This function sets the state status register based on the provided
 * parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonCtrlStat Pointer to the common control status structure
 * containing the data to set.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setStateStatReg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonStateStat_t * pCommonCtrlStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CommonStateStat_t * tempCfg = pCommonCtrlStat;

    if (tempCfg != NULL) {
        uint8_t regData = 0U;

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_STATE_STAT_REGADDR, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            /* Set RST_MCU_CNT bits (7-6) if pCommonCtrlStat->rstMcuCnt is not NULL */
            if (tempCfg -> rstMcuCnt != NULL) {
                Pmic_setBitField( & regData, PMIC_RST_MCU_CNT_SHIFT,
                    PMIC_RST_MCU_CNT_MASK, *(tempCfg -> rstMcuCnt));
            }

            /* Set RST_MCU_RQ_FLAG bit (5) if pCommonCtrlStat->rstMcuRqFlag is not
             * NULL */
            if (tempCfg -> rstMcuRqFlag != NULL) {
                Pmic_setBitField( & regData, PMIC_RST_MCU_RQ_FLAG_SHIFT,
                    PMIC_RST_MCU_RQ_FLAG_MASK, *(tempCfg -> rstMcuRqFlag));
            }
        }

        /* Update the modified STATE_STAT register */
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
            PMIC_STATE_STAT_REGADDR, regData);

        Pmic_criticalSectionStop(pPmicCoreHandle);
    } else {
        /* User did not provide necessary data */
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    return pmicStatus;
}

/**
 * @brief Set state control register.
 * This function sets the state control register based on the provided
 * parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonStateCtrl Pointer to the common state control structure
 * containing the data to set.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setStateCtrlReg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonStateCtrl_t * pCommonStateCtrl) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CommonStateCtrl_t * tempCfg = pCommonStateCtrl;

    if (tempCfg != NULL) {
        uint8_t regData = 0U;

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_STATE_CTRL_REGADDR, & regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (tempCfg -> state_req != (uint8_t) NULL) {
                Pmic_setBitField( & regData, PMIC_STATE_CTRL_STATE_REQ_SHIFT,
                    PMIC_STATE_CTRL_STATE_REQ_MASK, tempCfg -> state_req);
            }
        } else {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    } else {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    return pmicStatus;
}

/**
 * @brief Get state control register.
 * This function retrieves the state control register value.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonStateCtrl Pointer to the common state control structure to
 * store the retrieved value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getStateCtrlReg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonStateCtrl_t * pCommonStateCtrl) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    if (pCommonStateCtrl != NULL) {
        uint8_t regData = 0U;

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_STATE_CTRL_REGADDR, & regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (pCommonStateCtrl -> state_req != (uint8_t) NULL) {
                pCommonStateCtrl -> state_req =
                    Pmic_getBitField(regData, PMIC_STATE_CTRL_STATE_REQ_SHIFT,
                        PMIC_STATE_CTRL_STATE_REQ_MASK);
            }
        } else {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    } else {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    return pmicStatus;
}

/**
 * @brief Get state status register.
 * This function retrieves the state status register value.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonCtrlStat Pointer to the common control status structure to
 * store the retrieved value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getStateStatReg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonStateStat_t * pCommonCtrlStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CommonStateStat_t * tempCfg = pCommonCtrlStat;

    /* Check if user provides storage for the STATE_STAT data */
    if (tempCfg != NULL) {
        uint8_t regData = 0U;

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_STATE_STAT_REGADDR, & regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            /* Extract RST_MCU_CNT bits (7-6) into pCommonCtrlStat->rstMcuCnt if not
             * NULL */
            if (tempCfg -> rstMcuCnt != NULL) {
                *(tempCfg -> rstMcuCnt) = Pmic_getBitField(
                    regData, PMIC_RST_MCU_CNT_SHIFT, PMIC_RST_MCU_CNT_MASK);
            }

            /* Extract RST_MCU_RQ_FLAG bit (5) into pCommonCtrlStat->rstMcuRqFlag if
             * not NULL */
            if (tempCfg -> rstMcuRqFlag != NULL) {
                *(tempCfg -> rstMcuRqFlag) = Pmic_getBitField(
                    regData, PMIC_RST_MCU_RQ_FLAG_SHIFT, PMIC_RST_MCU_RQ_FLAG_MASK);
            }

            /* Extract PWRD_DLY_ACTV bit (4) into pCommonCtrlStat->pwrdDlyActv if not
             * NULL */
            if (tempCfg -> pwrdDlyActv != NULL) {
                *(tempCfg -> pwrdDlyActv) = Pmic_getBitField(
                    regData, PMIC_PWRD_DLY_ACTV_SHIFT, PMIC_PWRD_DLY_ACTV_MASK);
            }

            /* Extract STATE bits (3-0) into pCommonCtrlStat->state if not NULL */
            if (tempCfg -> state != NULL) {
                *(tempCfg -> state) =
                Pmic_getBitField(regData, PMIC_STATE_SHIFT, PMIC_STATE_MASK);
            }
        }
    } else {
        /* User did not provide storage for the retrieved data */
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    return pmicStatus;
}

/**
 * @brief Initialize the PMIC core handle with basic device configuration
 * parameters. This function initializes the PMIC core handle with basic device
 * configuration parameters.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t
Pmic_initCoreHandleBasicDevCfgParams(const Pmic_CoreCfg_t * pPmicConfigData,
    Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle device type */
    if ((true) == pmic_validParamCheck(pPmicConfigData -> validParams,
            PMIC_CFG_DEVICE_TYPE_VALID)) {
        if (PMIC_DEV_BB_TPS65386X != pPmicConfigData -> pmicDeviceType) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }

        if (PMIC_ST_SUCCESS == pmicStatus) {
            pPmicCoreHandle -> pmicDeviceType = pPmicConfigData -> pmicDeviceType;
        }
    }

    /* Check and update PMIC Handle Comm Mode */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((true) == pmic_validParamCheck(pPmicConfigData -> validParams,
            PMIC_CFG_COMM_MODE_VALID))) {
        if (PMIC_INTF_SPI != pPmicConfigData -> commMode) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        } else {
            pPmicCoreHandle -> commMode = pPmicConfigData -> commMode;
        }
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    /* Check and update PMIC Handle Comm Handle */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((true) == pmic_validParamCheck(pPmicConfigData -> validParams,
            PMIC_CFG_COMM_HANDLE_VALID))) {
        if (NULL == pPmicConfigData -> pCommHandle) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        } else {
            pPmicCoreHandle -> pCommHandle = pPmicConfigData -> pCommHandle;
        }
        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    return pmicStatus;
}

/**
 * @brief Initialize the PMIC core handle with communication I/O critical
 * section function pointers. This function initializes the PMIC core handle
 * with communication I/O critical section function pointers.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_initCoreHandleCommIOCriticalSectionFns(
    const Pmic_CoreCfg_t * pPmicConfigData, Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle Comm IO RD Fn */
    if ((true) == pmic_validParamCheck(pPmicConfigData -> validParams,
            PMIC_CFG_COMM_IO_RD_VALID)) {
        if (NULL == pPmicConfigData -> pFnPmicCommIoRead) {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        } else {
            pPmicCoreHandle -> pFnPmicCommIoRead = pPmicConfigData -> pFnPmicCommIoRead;
        }
    }

    /* Check and update PMIC Handle Comm IO WR Fn */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((true) == pmic_validParamCheck(pPmicConfigData -> validParams,
            PMIC_CFG_COMM_IO_WR_VALID))) {
        if (NULL == pPmicConfigData -> pFnPmicCommIoWrite) {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        } else {
            pPmicCoreHandle -> pFnPmicCommIoWrite = pPmicConfigData -> pFnPmicCommIoWrite;
        }
    }

    /* Check and update PMIC Handle Critical Section Start Fn */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((true) == pmic_validParamCheck(pPmicConfigData -> validParams,
            PMIC_CFG_CRITSEC_START_VALID))) {
        if (NULL == pPmicConfigData -> pFnPmicCritSecStart) {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        } else {
            pPmicCoreHandle -> pFnPmicCritSecStart =
                pPmicConfigData -> pFnPmicCritSecStart;
        }
    }

    /* Check and update PMIC Handle Critical Section Stop Fn */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((true) == pmic_validParamCheck(pPmicConfigData -> validParams,
            PMIC_CFG_CRITSEC_STOP_VALID))) {
        if (NULL == pPmicConfigData -> pFnPmicCritSecStop) {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        } else {
            pPmicCoreHandle -> pFnPmicCritSecStop = pPmicConfigData -> pFnPmicCritSecStop;
        }
    }

    return pmicStatus;
}

/**
 * @brief Validate the presence of the device on the bus.
 * This function validates the presence of the device on the bus.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_validateDevOnBus(Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;
    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_DEV_ID_REGADDR, regVal);

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_DEV_ID_REGADDR, & regVal);

    Pmic_criticalSectionStop(pPmicCoreHandle);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicCoreHandle -> pmicDevRev =
            Pmic_getBitField(regVal, PMIC_DEV_ID_SHIFT, PMIC_DEV_ID_MASK);

        /* Validate if the device requested is the one on the bus */
        if (PMIC_DEV_BB_TPS65386X == (pPmicCoreHandle -> pmicDeviceType)) {
            if (PMIC_TPS65386X_DEV_ID != pPmicCoreHandle -> pmicDevRev) {
                pmicStatus = PMIC_ST_WARN_INV_DEVICE_ID;
            }
        }
    }

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    return pmicStatus;
}

/**
 * @brief Update subsystem information and validate main Q&A communication
 * interface read/write.
 * This function updates subsystem information and validates the main Q&A
 * communication interface read/write.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * updated.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_updateSubSysInfoValidateMainQaCommIFRdWr(
    const Pmic_CoreCfg_t * pPmicConfigData, Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Update PMIC subsystem info to PMIC handle */
    pPmicCoreHandle -> pPmic_SubSysInfo = &
        pmicSubSysInfo[pPmicCoreHandle -> pmicDeviceType];

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_WD_LONGWIN_CFG_REGADDR, & regVal);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        if (PMIC_ST_SUCCESS == pmicStatus) {
            pPmicCoreHandle -> drvInitStatus |= pPmicConfigData -> instType;
        }
    }

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    return pmicStatus;
}

/**
 * @brief Initialize the PMIC core.
 * This function initializes the PMIC core based on the provided configuration
 * data.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_init(const Pmic_CoreCfg_t * pPmicConfigData,
    Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if ((NULL == pPmicCoreHandle) || (NULL == pPmicConfigData)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    /* Check and update PMIC Handle for device type, Comm Mode, Main Slave Address
     * and NVM Slave Address */
    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus =
            Pmic_initCoreHandleBasicDevCfgParams(pPmicConfigData, pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    /* Check and update PMIC Handle for Comm IO RD Fn, Comm IO Wr Fn, Critical
     * Section Start Fn and Critical Section Stop Fn */
    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_initCoreHandleCommIOCriticalSectionFns(pPmicConfigData,
            pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    /* Check for required members for I2C/SPI Main handle comm */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((NULL == pPmicCoreHandle -> pFnPmicCritSecStart) ||
            (NULL == pPmicCoreHandle -> pFnPmicCritSecStop) ||
            (NULL == pPmicCoreHandle -> pFnPmicCommIoRead) ||
            (NULL == pPmicCoreHandle -> pFnPmicCommIoWrite))) {
        pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    return pmicStatus;
}

/**
 * @brief Deinitialize the PMIC core.
 * This function deinitializes the PMIC core.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_deinit(Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicCoreHandle -> pCommHandle = NULL;
        pPmicCoreHandle -> pQACommHandle = NULL;
        pPmicCoreHandle -> pFnPmicCritSecStart = NULL;
        pPmicCoreHandle -> pFnPmicCritSecStop = NULL;
        pPmicCoreHandle -> pFnPmicCommIoRead = NULL;
        pPmicCoreHandle -> pFnPmicCommIoWrite = NULL;
        pPmicCoreHandle -> pPmic_SubSysInfo = NULL;
        pPmicCoreHandle -> drvInitStatus = 0x00U;
    }

    return pmicStatus;
}

/**
 * @brief Get the address of the scratch pad register.
 * This function retrieves the address of the scratch pad register based on the
 * specified register ID.
 *
 * @param scratchPadRegId Scratch pad register ID.
 * @param pRegAddr Pointer to store the address of the scratch pad register.
 * @return void
 */
static void Pmic_getScratchPadRegAddr(uint8_t scratchPadRegId,
    uint8_t * pRegAddr) {
    switch (scratchPadRegId) {
    case PMIC_SCRATCH_PAD_REG_1:
        *
        pRegAddr = PMIC_CUSTOMER_SCRATCH1_REGADDR;
        break;
    case PMIC_SCRATCH_PAD_REG_2:
        *
        pRegAddr = PMIC_CUSTOMER_SCRATCH2_REGADDR;
        break;
    default:
        *
        pRegAddr = PMIC_INVALID_REGADDR;
        break;
    }
}

/**
 * @brief Set value to the scratch pad register.
 * This function sets a value to the scratch pad register.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param scratchPadRegId Scratch pad register ID.
 * @param data Data to be written to the scratch pad register.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setScratchPadValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t scratchPadRegId,
        const uint8_t data) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (scratchPadRegId > PMIC_SCRATCH_PAD_REG_2)) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_getScratchPadRegAddr(scratchPadRegId, & regAddr);
        Pmic_criticalSectionStart(pPmicCoreHandle);
        if (regAddr != PMIC_INVALID_REGADDR) {
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, data);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Get value from the scratch pad register.
 * This function retrieves a value from the scratch pad register.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param scratchPadRegId Scratch pad register ID.
 * @param pData Pointer to store the retrieved data from the scratch pad
 * register.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getScratchPadValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t scratchPadRegId, uint8_t * pData) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (scratchPadRegId > PMIC_SCRATCH_PAD_REG_2)) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pData)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_getScratchPadRegAddr(scratchPadRegId, & regAddr);

        Pmic_criticalSectionStart(pPmicCoreHandle);
        if (regAddr != PMIC_INVALID_REGADDR) {
            pmicStatus =
                Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, pData);
        }
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Enable or disable spread spectrum.
 * This function enables or disables spread spectrum based on
 * Pmic_criticalSectionStartthe provided configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param commonCtrlCfg Spread spectrum configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_spreadSpectrumEnable(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_CommonCtrlCfg_t commonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_BUCK_BST_CFG_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        if (PMIC_SPREAD_SPECTRUM_CFG_ENABLE == commonCtrlCfg.sreadSpectrumEn) {
            Pmic_setBitField( & regData, PMIC_DRSS_SS_EN_SHIFT, PMIC_DRSS_SS_EN_MASK,
                PMIC_SPREAD_SPECTRUM_CFG_ENABLE);
        } else {
            Pmic_setBitField( & regData, PMIC_DRSS_SS_EN_SHIFT, PMIC_DRSS_SS_EN_MASK,
                PMIC_SPREAD_SPECTRUM_CFG_DISABLE);
        }

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
            PMIC_BUCK_BST_CFG_REGADDR, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief Get the spread spectrum enable status.
 * This function retrieves the status of spread spectrum.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlCfg Pointer to store the spread spectrum configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getSpreadSpectrumEnable(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonCtrlCfg_t * pCommonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_BUCK_BST_CFG_REGADDR, & regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pCommonCtrlCfg -> sreadSpectrumEn = false;

        if (Pmic_getBitField(regData, PMIC_DRSS_SS_EN_SHIFT,
                PMIC_DRSS_SS_EN_MASK) == 1U) {
            pCommonCtrlCfg -> sreadSpectrumEn = true;
        }
    }

    return pmicStatus;
}

/**
 * @brief Set the safe output pin configuration.
 * This function sets the configuration for safe output pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param commonCtrlCfg Safe output pin configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setEnableSafeOutCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_CommonCtrlCfg_t commonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if ((commonCtrlCfg.eNsafeOut1 > PMIC_PIN_SIGNAL_LEVEL_HIGH) ||
        (commonCtrlCfg.eNsafeOut2 > PMIC_PIN_SIGNAL_LEVEL_HIGH)) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
            pPmicCoreHandle, PMIC_SAFE_OUT_CFG_CTRL_REGADDR, & regData);
        if (PMIC_ST_SUCCESS == pmicStatus) {
            Pmic_setBitField( & regData, PMIC_ENABLE_SAFE_OUTEN1_SHIFT,
                PMIC_ENABLE_SAFE_OUTEN1_MASK, commonCtrlCfg.eNsafeOut1);

            Pmic_setBitField( & regData, PMIC_ENABLE_SAFE_OUTEN2_SHIFT,
                PMIC_ENABLE_SAFE_OUTEN2_MASK, commonCtrlCfg.eNsafeOut2);

            pmicStatus = Pmic_commIntf_sendByte(
                pPmicCoreHandle, PMIC_SAFE_OUT_CFG_CTRL_REGADDR, regData);
        }
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Get the safe output pin configuration.
 * This function retrieves the configuration of safe output pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlCfg Pointer to store the safe output pin configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getSafeOutPinCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonCtrlCfg_t * pCommonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_SAFE_OUT_CFG_CTRL_REGADDR, & regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pCommonCtrlCfg -> eNsafeOut1 = Pmic_getBitField(
            regData, PMIC_ENABLE_SAFE_OUTEN1_SHIFT, PMIC_ENABLE_SAFE_OUTEN1_MASK);

        pCommonCtrlCfg -> eNsafeOut2 = Pmic_getBitField(
            regData, PMIC_ENABLE_SAFE_OUTEN2_SHIFT, PMIC_ENABLE_SAFE_OUTEN2_MASK);
    }

    return pmicStatus;
}

/**
 * @brief Set the safe state timeout configuration.
 * This function sets the safe state timeout configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param safeCfg Safe state timeout configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setSafeStateTimeoutCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_SafeStateCfg_t * safeCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_SAFE_TMO_CFG_REGADDR, & regData);
        if (PMIC_ST_SUCCESS == pmicStatus) {
            safeCfg -> safeStateTMO =
                Pmic_getBitField(regData, PMIC_SAFE_TMO_SHIFT, PMIC_SAFE_TMO_MASK);

            Pmic_setBitField( & regData, PMIC_SAFE_TMO_SHIFT, PMIC_SAFE_TMO_MASK,
                safeCfg -> safeStateTMO);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                PMIC_SAFE_TMO_CFG_REGADDR, regData);
        }
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Get the safe state timeout configuration.
 * This function retrieves the safe state timeout configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param safeCfg Pointer to store the safe state timeout configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getSafeStateTimeoutCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_SafeStateCfg_t * safeCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_SAFE_TMO_CFG_REGADDR, & regData);
        if (PMIC_ST_SUCCESS == pmicStatus) {
            safeCfg -> safeStateTMO =
                Pmic_getBitField(regData, PMIC_SAFE_TMO_SHIFT, PMIC_SAFE_TMO_MASK);
        }
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Set the safe state threshold configuration.
 * This function sets the safe state threshold configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param safeCfg Safe state threshold configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setSafeStateThresholdCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_SafeStateCfg_t * safeCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_SAFE_TMO_CFG_REGADDR, & regData);
        if (PMIC_ST_SUCCESS == pmicStatus) {
            safeCfg -> safeLockThreshold = Pmic_getBitField(
                regData, PMIC_SAFE_LOCK_TH_SHIFT, PMIC_SAFE_LOCK_TH_MASK);

            Pmic_setBitField( & regData, PMIC_SAFE_LOCK_TH_SHIFT,
                PMIC_SAFE_LOCK_TH_MASK, safeCfg -> safeLockThreshold);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                PMIC_SAFE_TMO_CFG_REGADDR, regData);
        }
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Get the safe state threshold configuration.
 * This function retrieves the safe state threshold configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param safeCfg Pointer to store the safe state threshold configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getSafeStateThresholdCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_SafeStateCfg_t * safeCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_SAFE_TMO_CFG_REGADDR, & regData);
        if (PMIC_ST_SUCCESS == pmicStatus) {
            safeCfg -> safeLockThreshold = Pmic_getBitField(
                regData, PMIC_SAFE_LOCK_TH_SHIFT, PMIC_SAFE_LOCK_TH_MASK);
        }
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Set the common control configuration.
 * This function sets the common control configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param CommonCtrlCfg Common control configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setCommonCtrlConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_CommonCtrlCfg_t CommonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == ((pmic_validParamCheck(CommonCtrlCfg.sreadSpectrumEn,
            PMIC_CFG_SPREAD_SPECTRUM_EN_VALID))))) {
        /* Enable/Disable Spread Spectrum */
        pmicStatus = Pmic_spreadSpectrumEnable(pPmicCoreHandle, CommonCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == ((pmic_validParamCheck((uint32_t) CommonCtrlCfg.eNsafeOut1,
                PMIC_CFG_ENABLE_SAFEOUT_VALID)) ||
            (pmic_validParamCheck((uint32_t) CommonCtrlCfg.eNsafeOut2,
                PMIC_CFG_ENABLE_SAFEOUT_VALID))))) {
        /* Set ENABLE_DRV Pin Configuration */
        pmicStatus = Pmic_setEnableSafeOutCfg(pPmicCoreHandle, CommonCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == ((pmic_validParamCheck((uint32_t) CommonCtrlCfg.regLock_1,
                PMIC_REGISTER_UNLOCK_DATA1)) ||
            (pmic_validParamCheck((uint32_t) CommonCtrlCfg.regLock_2,
                PMIC_REGISTER_UNLOCK_DATA2))))) {
        /* Set Register Lock/UnLock Configuration */
        pmicStatus = Pmic_setRegisterLockUnlock(pPmicCoreHandle, CommonCtrlCfg);
    }
    return pmicStatus;
}

/**
 * @brief Get the common control configuration.
 * This function retrieves the common control configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlCfg Pointer to store the common control configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getCommonCtrlConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonCtrlCfg_t * pCommonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pCommonCtrlCfg)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == pmic_validParamCheck(pCommonCtrlCfg -> validParams,
            PMIC_CFG_SPREAD_SPECTRUM_EN_VALID))) {
        /* Get the status of Spread Spectrum is Enabled/Disabled  */
        pmicStatus = Pmic_getSpreadSpectrumEnable(pPmicCoreHandle, pCommonCtrlCfg);
    }
    return pmicStatus;
}

/**
 * @brief Set the configuration of AMUX and DMUX pins for diagnostic output.
 * This function sets the configuration of AMUX and DMUX pins for diagnostic
 * output. It communicates with the PMIC to configure the specified settings.
 *
 * @param pPmicCoreHandle  Pointer to the PMIC Core Handle.
 * @param diagoutCfgCtrl   Diagnostic output configuration control.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setAmuxDmuxPinCtrlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_DiagOutCfgCtrl_t diagoutCfgCtrl) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t diagoutAMUXEn = diagoutCfgCtrl.DiagOutCtrl_AMUXEn;
    uint8_t diagoutDMUXEn = diagoutCfgCtrl.DiagOutCtrl_DMUXEn;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_DIAG_OUT_CFG_CTRL_REGADDR, & regData);

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (PMIC_DEV_BB_TPS65386X == pPmicCoreHandle -> pmicDeviceType)) {
        if ((PMIC_BB_DIAG_OUT_AMUX_ENABLE) == diagoutAMUXEn) {
            Pmic_setBitField( & regData, PMIC_DIAG_OUT_CTRL_SHIFT,
                PMIC_DIAG_OUT_CTRL_MASK, PMIC_BB_DIAG_OUT_AMUX_ENABLE);
        } else if ((PMIC_BB_DIAG_OUT_DMUX_ENABLE) == diagoutDMUXEn) {
            Pmic_setBitField( & regData, PMIC_DIAG_OUT_CTRL_SHIFT,
                PMIC_DIAG_OUT_CTRL_MASK, PMIC_BB_DIAG_OUT_DMUX_ENABLE);
        } else {
            Pmic_setBitField( & regData, PMIC_DIAG_OUT_CTRL_SHIFT,
                PMIC_DIAG_OUT_CTRL_MASK, PMIC_BB_DIAG_OUT_DISABLE);
        }
        pmicStatus = Pmic_commIntf_sendByte(
            pPmicCoreHandle, PMIC_DIAG_OUT_CFG_CTRL_REGADDR, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief Get the configuration of AMUX and DMUX pins for diagnostic output.
 * This function gets the configuration of AMUX and DMUX pins for diagnostic
 * output. It communicates with the PMIC to read the current settings.
 *
 * @param  pPmicCoreHandle  Pointer to the PMIC Core Handle.
 * @param  pDiagOutCfgCtrl  Pointer to the structure to store diagnostic output
 * configuration control.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getAmuxDmuxPinCtrlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_DiagOutCfgCtrl_t * pDiagOutCfgCtrl) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_DIAG_OUT_CFG_CTRL_REGADDR, & regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pDiagOutCfgCtrl -> DiagOutCtrl = Pmic_getBitField(
            regData, PMIC_DIAG_OUT_CTRL_SHIFT, PMIC_DIAG_OUT_CTRL_MASK);
    }

    return pmicStatus;
}

/**
 * @brief Set the diagnostics output pin control configuration.
 * This function sets the diagnostics output pin control configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param DiagOutCfgCtrl Diagnostics output pin control configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setDiagOutCtrlConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_DiagOutCfgCtrl_t DiagOutCfgCtrl) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Set AMUX_OUT/REF_OUT Pin Control Configuration */
        pmicStatus = Pmic_setAmuxDmuxPinCtrlCfg(pPmicCoreHandle, DiagOutCfgCtrl);
    }

    return pmicStatus;
}

/**
 * @brief Get the diagnostics output pin control configuration.
 * This function retrieves the diagnostics output pin control configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pDiagOutCfgCtrl Pointer to store the diagnostics output pin control
 * configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getDiagOutCtrlConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_DiagOutCfgCtrl_t * pDiagOutCfgCtrl) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Get AMUX/DMUX Pin Control Configuration */
        pmicStatus = Pmic_getAmuxDmuxPinCtrlCfg(pPmicCoreHandle, pDiagOutCfgCtrl);
    }

    return pmicStatus;
}

int32_t Pmic_setDiagMUXSelectionCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_DiagOutCfgCtrl_t diagoutCfgCtrl) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t diagGrpSel = diagoutCfgCtrl.DiagGrpSel;
    uint8_t diagChannelSel = diagoutCfgCtrl.DiagChannelSel;

    /* Set Diagnostic Control Group number */
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_DIAG_OUT_CFG_CTRL_REGADDR, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( & regData, PMIC_DIAG_GRP_SEL_SHIFT, PMIC_DIAG_GRP_SEL_MASK,
            diagGrpSel);
    }
    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
        PMIC_DIAG_OUT_CFG_CTRL_REGADDR, regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    /* Set Diagnostic Control Channel number */
    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_DIAG_OUT_CFG_REGADDR, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            Pmic_setBitField( & regData, PMIC_DIAG_CH_SEL_SHIFT, PMIC_DIAG_CH_SEL_MASK,
                diagChannelSel);
        }
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
            PMIC_DIAG_OUT_CFG_REGADDR, regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_getDiagMUXSelectionCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_DiagOutCfgCtrl_t * diagoutCfgCtrl) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_DIAG_OUT_CFG_CTRL_REGADDR, & regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        diagoutCfgCtrl -> DiagGrpSel = Pmic_getBitField(
            regData, PMIC_DIAG_GRP_SEL_SHIFT, PMIC_DIAG_GRP_SEL_MASK);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_DIAG_OUT_CFG_REGADDR, & regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            diagoutCfgCtrl -> DiagChannelSel = Pmic_getBitField(
                regData, PMIC_DIAG_CH_SEL_SHIFT, PMIC_DIAG_CH_SEL_MASK);
        }
    }

    return pmicStatus;
}

int32_t Pmic_setDiagAMUXFeatureCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_AMUXFeatures feature) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_DiagOutCfgCtrl_t diagoutCfgCtrl;

    if (feature >= AMUX_NUM_FEATURES) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        diagoutCfgCtrl.DiagGrpSel = Pmic_amuxFeatureMappings[feature].group;
        diagoutCfgCtrl.DiagChannelSel = Pmic_amuxFeatureMappings[feature].channel;

        pmicStatus = Pmic_setDiagMUXSelectionCfg(pPmicCoreHandle, diagoutCfgCtrl);
    }

    return pmicStatus;
}

int32_t Pmic_getDiagAMUXFeatureCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
                                   uint32_t * feature) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_DiagOutCfgCtrl_t diagoutCfgCtrl;
    uint32_t diagGrpSel = 0U;
    uint32_t diagChannelSel = 0U;

    pmicStatus = Pmic_getDiagMUXSelectionCfg(pPmicCoreHandle, & diagoutCfgCtrl);

    diagGrpSel = diagoutCfgCtrl.DiagGrpSel;
    diagChannelSel = diagoutCfgCtrl.DiagChannelSel;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Loop through the feature mappings to find a match */
        for (int8_t i = 0; i < (int8_t) PMIC_END_AMUX; ++i) {
            if ((Pmic_amuxFeatureMappings[i].group == diagGrpSel) &&
                (Pmic_amuxFeatureMappings[i].channel == diagChannelSel)) {
                /* Return the corresponding feature */
                * feature = (uint32_t)PMIC_START_AMUX + (uint32_t)i;
            }
        }
    }

    return pmicStatus;
}

int32_t Pmic_setDiagDMUXFeatureCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_DMUXFeatures feature) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_DiagOutCfgCtrl_t diagoutCfgCtrl;

    if (feature >= DMUX_NUM_FEATURES) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
        diagoutCfgCtrl.DiagGrpSel = Pmic_dmuxFeatureMappings[feature].group;
        diagoutCfgCtrl.DiagChannelSel = Pmic_dmuxFeatureMappings[feature].channel;

        pmicStatus = Pmic_setDiagMUXSelectionCfg(pPmicCoreHandle, diagoutCfgCtrl);
    }

    return pmicStatus;
}

int32_t Pmic_getDiagDMUXFeatureCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
                                   uint32_t * feature) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_DiagOutCfgCtrl_t diagoutCfgCtrl;
    uint32_t diagGrpSel = 0U;
    uint32_t diagChannelSel = 0U;

    pmicStatus = Pmic_getDiagMUXSelectionCfg(pPmicCoreHandle, & diagoutCfgCtrl);

    diagGrpSel = diagoutCfgCtrl.DiagGrpSel;
    diagChannelSel = diagoutCfgCtrl.DiagChannelSel;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Loop through the feature mappings to find a match */
        for (int8_t i = 0; i < (int8_t) PMIC_END_DMUX; ++i) {
            if ((Pmic_dmuxFeatureMappings[i].group == diagGrpSel) &&
                (Pmic_dmuxFeatureMappings[i].channel == diagChannelSel)) {
                /* Return the corresponding feature */
                * feature = (uint32_t)PMIC_START_DMUX + (uint32_t)i;
            }
        }
    }

    return pmicStatus;
}

static void Pmic_getPinTypeRegBitFields(const uint8_t pinType,
    uint8_t * pBitShift, uint8_t * pBitMask) {
    switch (pinType) {
    case PMIC_PIN_TYPE_NRST_RDBK_LVL:
        *
        pBitShift = PMIC_NRST_RDBK_LVL_SHIFT;
        * pBitMask = PMIC_NRST_RDBK_LVL_MASK;
        break;
    case PMIC_PIN_TYPE_SAFEOUT1_RDBK_LVL:
        *
        pBitShift = PMIC_SAFE_OUT1_RDBK_LVL_SHIFT;
        * pBitMask = PMIC_SAFE_OUT1_RDBK_LVL_MASK;
        break;
    default:
        *
        pBitShift = PMIC_EN_OUT_RDBK_LVL_SHIFT;
        * pBitMask = PMIC_EN_OUT_RDBK_LVL_MASK;
        break;
    }
}

/**
 * @brief Get the pin value.
 * This function retrieves the value of the specified pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pinType Type of the pin.
 * @param pPinValue Pointer to store the pin value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getPinValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t pinType, uint8_t * pPinValue) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal;
    uint8_t bitShift, bitMask;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (pinType > PMIC_PIN_TYPE_GPO4_RDBK_LVL)) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pPinValue)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_RDBK_ERR_STAT_REGADDR, & regVal);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_getPinTypeRegBitFields(pinType, & bitShift, & bitMask);

        * pPinValue = Pmic_getBitField(regVal, bitShift, bitMask);
    }

    return pmicStatus;
}

/**
 * @brief Get the status of the safe output 1 pin.
 * This function retrieves the status of the safe output 1 pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_getSafeOut1Stat(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonCtrlStat_t * pCommonCtrlStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_RDBK_ERR_STAT_REGADDR, & regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pCommonCtrlStat -> safeOut1Pin = Pmic_getBitField(
            regData, PMIC_SAFE_OUT1_RDBK_LVL_SHIFT, PMIC_SAFE_OUT1_RDBK_LVL_MASK);
    }

    return pmicStatus;
}

/**
 * @brief Get the status of the nRST pin.
 * This function retrieves the status of the nRST pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_getNRstPinStat(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonCtrlStat_t * pCommonCtrlStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_RDBK_ERR_STAT_REGADDR, & regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pCommonCtrlStat -> nRstPin = Pmic_getBitField(
            regData, PMIC_NRST_RDBK_LVL_SHIFT, PMIC_NRST_RDBK_LVL_MASK);
    }

    return pmicStatus;
}

/**
 * @brief Get the status of the EN_OUT pin.
 * This function retrieves the status of the EN_OUT pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_getEnOutPinStat(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonCtrlStat_t * pCommonCtrlStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
        PMIC_RDBK_ERR_STAT_REGADDR, & regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pCommonCtrlStat -> enOutPin = Pmic_getBitField(
            regData, PMIC_EN_OUT_RDBK_LVL_SHIFT, PMIC_EN_OUT_RDBK_LVL_MASK);
    }

    return pmicStatus;
}

/**
 * @brief Get the status of the EN_OUT, nRST, and safe output 1 pins.
 * This function retrieves the status of the EN_OUT, nRST, and safe output 1
 * pins.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t
Pmic_getEnOutNrstSafeOut1PinStat(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonCtrlStat_t * pCommonCtrlStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == pmic_validParamCheck(pCommonCtrlStat -> validParams,
            PMIC_CFG_SAFE_OUT1_PIN_STAT_VALID))) {
        /* Get SAFE_OUT1 Pin Status*/
        pmicStatus = Pmic_getSafeOut1Stat(pPmicCoreHandle, pCommonCtrlStat);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == pmic_validParamCheck(pCommonCtrlStat -> validParams,
            PMIC_CFG_NRST_PIN_STAT_VALID))) {
        /* Get NRST Pin Status*/
        pmicStatus = Pmic_getNRstPinStat(pPmicCoreHandle, pCommonCtrlStat);
    }
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == pmic_validParamCheck(pCommonCtrlStat -> validParams,
            PMIC_CFG_EN_OUT_PIN_STAT_VALID))) {
        /* Get EN_OUT Pin Status*/
        pmicStatus = Pmic_getEnOutPinStat(pPmicCoreHandle, pCommonCtrlStat);
    }

    return pmicStatus;
}

/**
 * @brief Get the common status.
 * This function retrieves the common control status.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getCommonStat(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_CommonCtrlStat_t * pCommonCtrlStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == pmic_validParamCheck(pCommonCtrlStat -> validParams,
            PMIC_CFG_REG_LOCK_STAT_VALID))) {
        /* Get Register Lock Status*/
        pmicStatus = Pmic_getRegLockStat(pPmicCoreHandle, pCommonCtrlStat);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Get EN_Out Pin, NRST Pin, and Safe_Out1 Pin Status */
        pmicStatus =
            Pmic_getEnOutNrstSafeOut1PinStat(pPmicCoreHandle, pCommonCtrlStat);
    }

    return pmicStatus;
}

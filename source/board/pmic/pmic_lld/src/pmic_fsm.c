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
 *   @file    pmic_fsm.c
 *
 *   @brief   This file contains the default API's for PMIC FSM state
 *            configuration
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_fsm.h"
#include "pmic_fsm_priv.h"

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
 * @brief  Gets the PMIC Standby Configuration Register Fields
 *
 * @param  pmicNextState   Next state for PMIC FSM
 * @param  pRegAddr        Pointer to register address
 * @param  pBitPos         Pointer to bit position
 * @param  pBitMask        Pointer to bit mask
 * @param  pBitVal         Pointer to bit value
 * @return void
 */
void Pmic_fsmGetstandByCfgRegFields(uint8_t pmicNextState, uint8_t * pRegAddr,
    uint8_t * pBitPos, uint8_t * pBitMask,
    uint8_t * pBitVal, uint8_t * pDeviceState) {
    * pRegAddr = PMIC_STATE_CTRL_REGADDR;
    * pBitPos = PMIC_STATE_CTRL_SHIFT;
    * pBitMask = PMIC_STATE_CTRL_MASK;

    switch (pmicNextState) {
    case PMIC_FSM_STANBY_STATE:
        *
        pBitVal = PMIC_STATE_CTRL_STANDBY;
        * pDeviceState = PMIC_FSM_STAT_STANDBY;
        break;
    case PMIC_FSM_OFF_STATE:
        *
        pBitVal = PMIC_STATE_CTRL_OFF;
        * pDeviceState = PMIC_FSM_STAT_OFF_1;
        break;
    case PMIC_FSM_SAFE_STATE:
        *
        pBitVal = PMIC_STATE_CTRL_ACTIVE_TO_SAFE;
        * pDeviceState = PMIC_FSM_STAT_SAFE;
        break;
    case PMIC_FSM_ACTIVE_STATE:
        *
        pBitVal = PMIC_STATE_CTRL_SAFE_TO_ACTIVE;
        * pDeviceState = PMIC_FSM_STAT_ACTIVE;
        break;
    case PMIC_FSM_MCU_ONLY_STATE:
        *
        pBitVal = PMIC_STATE_CTRL_RESET_MCU;
        * pDeviceState = PMIC_FSM_STAT_RESET_MCU;
        break;
    default:
        *
        pBitVal = PMIC_STATE_CTRL_NO_CHANGE;
        * pDeviceState = PMIC_FSM_STAT_OFF_1;
        break;
    }
}

/**
 * @brief  Gets the NSLEEP Mask Bit Field
 *
 * @param  nsleepType  Type of NSLEEP signal
 * @param  pBitPos     Pointer to bit position
 * @param  pBitMask    Pointer to bit mask
 * @return void
 */
void Pmic_fsmGetNsleepMaskBitField(uint8_t nsleepType, uint8_t * pBitPos,
    uint8_t * pBitMask) {
    if (nsleepType == PMIC_NSLEEP1_SIGNAL) {
        * pBitPos = PMIC_WAKE1_DGL_CFG_SHIFT;
        * pBitMask = PMIC_WAKE1_DGL_CFG_MASK;
    }

    if (nsleepType == PMIC_NSLEEP2_SIGNAL) {
        * pBitPos = PMIC_WAKE2_DGL_CFG_SHIFT;
        * pBitMask = PMIC_WAKE2_DGL_CFG_MASK;
    }
}

/**
 * @brief  Sets the S2R State for PMIC
 *
 * @param  pPmicCoreHandle     Pointer to PMIC Core Handle
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setS2RState(Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (NULL == pPmicCoreHandle) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Standby CFG to set STBY_EN bit */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STBY_CFG_REGADDR, &
            regData);

        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField( & regData, PMIC_STBY_CFG_STBY_SEL_SHIFT,
                (uint8_t) PMIC_STBY_CFG_STBY_SEL_MASK,
                (uint8_t) PMIC_STBY_CFG_STBY_VAL);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_STBY_CFG_REGADDR,
                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);

        /* State CTRL to set the STANDBY request */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STATE_CTRL_REGADDR, &
            regData);

        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField( & regData, PMIC_STATE_CTRL_SHIFT, PMIC_STATE_CTRL_MASK,
                PMIC_STATE_CTRL_STANDBY);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_STATE_CTRL_REGADDR,
                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/**
 * @brief  Gets the Device State Configuration for PMIC
 *
 * @param  pPmicCoreHandle     Pointer to PMIC Core Handle
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_fsmGetDeviceStateCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * deviceState) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_STATE_STAT_REGADDR, & regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        * deviceState = Pmic_getBitField(regData, PMIC_STATE_SHIFT, PMIC_STATE_MASK);

        if (pmicStatus != PMIC_ST_SUCCESS) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    return pmicStatus;
}

/**
 * @brief  Handles PMIC FSM device request configuration
 *
 * @param  pPmicCoreHandle     Pointer to the PMIC Core Handle
 * @param  fsmState            FSM State to set
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_fsmDeviceRequestCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t fsmState) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t regData = 0U;
    uint8_t bitPos = 0U;
    uint8_t bitMask = 0U;
    uint8_t bitVal = 0U;
    uint8_t deviceState = 0;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_fsmGetstandByCfgRegFields(fsmState, & regAddr, & bitPos, & bitMask, &
            bitVal, & deviceState);

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            Pmic_setBitField( & regData, bitPos, bitMask, bitVal);

            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    pmicStatus = Pmic_fsmGetDeviceStateCfg(pPmicCoreHandle, & deviceState);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        if (pmicStatus != (int32_t) deviceState) {
            pmicStatus = PMIC_ST_ERR_INV_FSM_MODE;
        }
    }

    return pmicStatus;
}

/**
 * @brief  Sets the PMIC state based on the given FSM state
 *
 * @param  pPmicCoreHandle     Pointer to the PMIC Core Handle
 * @param  pmicNextState       Next state for PMIC FSM
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setState(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pmicNextState) {
    int32_t status = PMIC_ST_SUCCESS;

    switch (pmicNextState) {
    case PMIC_FSM_OFF_STATE:
        status = Pmic_fsmDeviceRequestCfg(pPmicCoreHandle, PMIC_FSM_OFF_STATE);
        break;
    case PMIC_FSM_STANBY_STATE:
        status = Pmic_fsmDeviceRequestCfg(pPmicCoreHandle, PMIC_FSM_STANBY_STATE);
        break;
    case PMIC_FSM_SAFE_STATE:
        status = Pmic_fsmDeviceRequestCfg(pPmicCoreHandle, PMIC_FSM_SAFE_STATE);
        break;
    case PMIC_FSM_ACTIVE_STATE:
        status = Pmic_fsmDeviceRequestCfg(pPmicCoreHandle, PMIC_FSM_ACTIVE_STATE);
        break;
    case PMIC_FSM_MCU_ONLY_STATE:
        status = Pmic_fsmDeviceRequestCfg(pPmicCoreHandle, PMIC_FSM_MCU_ONLY_STATE);
        break;
    case PMIC_FSM_S2R_STATE:
        status = Pmic_setS2RState(pPmicCoreHandle);
        break;
    default:
        /* Default case is valid only for STANBY_STATE */
        status = Pmic_fsmDeviceRequestCfg(pPmicCoreHandle, PMIC_FSM_STANBY_STATE);
        break;
    }

    return status;
}

/**
 * @brief  Enables or disables Fast Built-in Self Test (BIST) for the PMIC
 *
 * @param  pPmicCoreHandle     Pointer to the PMIC Core Handle
 * @param  fsmCfg              FSM configuration settings
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_fsmRequestRuntimeBist(Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos = PMIC_BIST_CTRL_EN_SHIFT;
    uint8_t bitMask = (uint8_t) PMIC_BIST_CTRL_CFG_MASK;
    uint8_t maskEnableVal = 0U;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_BIST_CTRL_REGADDR, &
            regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            maskEnableVal = 1U;

            Pmic_setBitField( & regData, bitPos, bitMask, maskEnableVal);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                PMIC_BIST_CTRL_REGADDR, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief  Sets the PMIC state to the provided state
 *
 * @param  pPmicCoreHandle     Pointer to the PMIC Core Handle
 * @param  pmicState           State to be set for the PMIC
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_fsmSetMissionState(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t pmicState) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((int32_t) pmicState > PMIC_FSM_STATE_MAX)) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_setState(pPmicCoreHandle, pmicState);
    }

    return pmicStatus;
}

/**
 * @brief  Sets or clears the NSLEEP signal mask for PMIC
 *
 * @param  pPmicCoreHandle     Pointer to the PMIC Core Handle
 * @param  nsleepType          Type of NSLEEP signal (NSLEEP1 or NSLEEP2)
 * @param  maskEnable          Enable or disable the mask for the NSLEEP signal
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_fsmSetNsleepSignalMask(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t nsleepType,
        const bool maskEnable) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos = 0U;
    uint8_t bitMask = 0U;
    uint8_t maskEnableVal = 0U;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WAKE_CFG_REGADDR, &
            regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            maskEnableVal = 1U;

            Pmic_fsmGetNsleepMaskBitField(nsleepType, & bitPos, & bitMask);

            if (true == maskEnable) {
                maskEnableVal = 1U;
            }

            Pmic_setBitField( & regData, bitPos, bitMask, maskEnableVal);
            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                PMIC_WAKE_CFG_REGADDR, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief  Gets the status of the NSLEEP signal mask for PMIC
 *
 * @param  pPmicCoreHandle     Pointer to the PMIC Core Handle
 * @param  nsleepType          Type of NSLEEP signal (NSLEEP1 or NSLEEP2)
 * @param  pNsleepStat         Pointer to the variable to store the NSLEEP
 * status
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_fsmGetNsleepSignalMaskStat(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t nsleepType,
        bool * pNsleepStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos = 0U;
    uint8_t bitMask = 0U;
    uint8_t maskEnableStat = 0U;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pNsleepStat)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WAKE_CFG_REGADDR, &
            regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        * pNsleepStat = false;

        Pmic_fsmGetNsleepMaskBitField(nsleepType, & bitPos, & bitMask);
        maskEnableStat = Pmic_getBitField(regData, bitPos, bitMask);

        if (maskEnableStat == 1U) {
            * pNsleepStat = true;
        }
    }

    return pmicStatus;
}

/**
 * @brief  Enables or disables the Fast BIST for PMIC
 *
 * @param  pPmicCoreHandle     Pointer to the PMIC Core Handle
 * @param  fsmCfg              FSM configuration settings for Fast BIST control
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_fsmEnableFastBIST(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_FsmCfg_t fsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t regData = 0U;
    uint8_t bitMask = 0U;
    uint8_t bitPos = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    regAddr = PMIC_BIST_CTRL_REGADDR;
    bitMask = (uint8_t) PMIC_BIST_CTRL_EN_MASK;
    bitPos = PMIC_BIST_CTRL_EN_SHIFT;

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        if (PMIC_FSM_FAST_BIST_ENABLE == fsmCfg.fastBistEn) {
            /* Enable BIST (set bit 2) and set BIST_CFG to 00 (bits 1-0) */
            Pmic_setBitField( & regData, bitPos, bitMask, PMIC_BIST_BIT_ENABLE);
        } else {
            /* Disable BIST (clear bit 2) and set BIST_CFG to 00 (bits 1-0) */
            Pmic_setBitField( & regData, bitPos, bitMask, PMIC_BIST_BIT_DISABLE);
        }

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t) regAddr, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief  Gets the configuration of Fast BIST for PMIC
 *
 * @param  pPmicCoreHandle     Pointer to the PMIC Core Handle
 * @param  pFsmCfg             Pointer to store the Fast BIST configuration
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_fsmGetFastBISTCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_FsmCfg_t * pFsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t regData = 0U;
    uint8_t bitMask = 0U;
    uint8_t bitPos = 0U;
    uint8_t bistMode = 0U;

    regAddr = PMIC_BIST_CTRL_REGADDR;
    bitMask = (uint8_t) PMIC_BIST_CTRL_CFG_MASK;
    bitPos = PMIC_BIST_CTRL_CFG_SHIFT;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t) regAddr, & regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        bistMode = Pmic_getBitField(regData, bitPos, bitMask);
        if (bistMode == PMIC_ABIST_MODE_SEL) {
            pFsmCfg -> fastBistEn = true;
        } else {
            pFsmCfg -> fastBistEn = false;
        }
    } else {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    return pmicStatus;
}

/**
 * @brief  Enables or disables the Buck LDO ILIM INT affecting FSM
 *
 * @param  pPmicCoreHandle     Pointer to the PMIC Core Handle
 * @param  fsmCfg              FSM configuration settings for ILIM INT FSM
 * control
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t
Pmic_fsmEnabBckLdoIlimIntAffect(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_FsmCfg_t fsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ILIM_CFG_REGADDR, &
            regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (PMIC_FSM_ILIM_INT_ENABLE == fsmCfg.ilimIntfsmCtrlEn) {
                Pmic_setBitField( & regData, PMIC_CFG1_EN_ILM_FSM_CTRL_SHIFT,
                    PMIC_CFG1_EN_ILIM_FSM_CTRL_MASK,
                    PMIC_FSM_ILIM_INT_ENABLE);
            } else {
                Pmic_setBitField( & regData, PMIC_CFG1_EN_ILM_FSM_CTRL_SHIFT,
                    PMIC_CFG1_EN_ILIM_FSM_CTRL_MASK,
                    PMIC_FSM_ILIM_INT_DISABLE);
            }

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                PMIC_ILIM_CFG_REGADDR, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief  Gets the configuration of Buck LDO ILIM INT affecting FSM
 *
 * @param  pPmicCoreHandle     Pointer to the PMIC Core Handle
 * @param  pFsmCfg             Pointer to FSM configuration settings
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t
Pmic_fsmGetBuckLdoIlimIntAftCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_FsmCfg_t * pFsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ILIM_CFG_REGADDR, &
            regData);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            pFsmCfg -> ilimIntfsmCtrlEn = false;

            if (1U == Pmic_getBitField(regData, PMIC_CFG1_EN_ILM_FSM_CTRL_SHIFT,
                    PMIC_CFG1_EN_ILIM_FSM_CTRL_MASK)) {
                pFsmCfg -> ilimIntfsmCtrlEn = true;
            }
        }
    }

    return pmicStatus;
}

/**
 * @brief Set the Power Management Integrated Circuit (PMIC) FSM configuration.
 * This function sets various configurations for the PMIC FSM (Finite State
 * Machine).
 *
 * @param  pPmicCoreHandle Pointer to the PMIC core handle.
 * @param  fsmCfg          Configuration structure for FSM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_fsmSetConfiguration(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_FsmCfg_t fsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == pmic_validParamCheck((uint32_t) fsmCfg.validParams,
            PMIC_FSM_CFG_FAST_BIST_EN_VALID))) {
        /* Enable/Disable Fast BIST */
        pmicStatus = Pmic_fsmEnableFastBIST(pPmicCoreHandle, fsmCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == pmic_validParamCheck((uint32_t) fsmCfg.validParams,
            PMIC_FSM_CFG_ILIM_INT_EN_VALID))) {
        /* Enable/Disable Buck/LDO regulators ILIM interrupts affect FSM
         * triggers */
        pmicStatus = Pmic_fsmEnabBckLdoIlimIntAffect(pPmicCoreHandle, fsmCfg);
    }

    return pmicStatus;
}

/**
 * @brief Get the Power Management Integrated Circuit (PMIC) FSM configuration.
 * This function retrieves the configured settings of the PMIC FSM.
 *
 * @param  pPmicCoreHandle Pointer to the PMIC core handle.
 * @param  pFsmCfg         Pointer to the FSM configuration structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_fsmGetConfiguration(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_FsmCfg_t * pFsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pFsmCfg)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == pmic_validParamCheck((uint32_t) pFsmCfg -> validParams,
            PMIC_FSM_CFG_FAST_BIST_EN_VALID))) {
        /* Get Fast BIST is enabled or not*/
        pmicStatus = Pmic_fsmGetFastBISTCfg(pPmicCoreHandle, pFsmCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == pmic_validParamCheck((uint32_t) pFsmCfg -> validParams,
            PMIC_FSM_CFG_ILIM_INT_EN_VALID))) {
        /* Get Buck/LDO regulators ILIM interrupts affect FSM
         * triggers is enabled or not */
        pmicStatus = Pmic_fsmGetBuckLdoIlimIntAftCfg(pPmicCoreHandle, pFsmCfg);
    }

    return pmicStatus;
}

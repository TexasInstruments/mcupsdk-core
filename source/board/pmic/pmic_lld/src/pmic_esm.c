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
 *   @file    pmic_esm.c
 *
 *   @brief   This file contains the default API's for PMIC esm
 *            configuration
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_esm.h"
#include "pmic_esm_priv.h"

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
 * @brief Get the equivalent uint8_t value from a boolean value.
 *
 * @param esmVal Boolean value to convert.
 * @return esmU8Val Equivalent uint8_t value.
 */
static uint8_t Pmic_esmGetU8Val(bool esmVal) {
    uint8_t esmU8Val = 0U;

    if (true == esmVal) {
        esmU8Val = 1U;
    }

    return esmU8Val;
}

/**
 * @brief Validate the parameters for ESM.
 * This function validates the parameters for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t
Pmic_esmValidateParams(const Pmic_CoreHandle_t *pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (false == pPmicCoreHandle -> pPmic_SubSysInfo -> esmEnable)) {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    return pmicStatus;
}

/**
 * @brief Get the base register address for ESM.
 * This function retrieves the base register address for ESM.
 *
 * @param esmType Type of ESM.
 * @param pEsmBaseAddr Pointer to store the base register address.
 * @return void
 */
static void Pmic_esmGetBaseRegAddr(uint8_t * pEsmBaseAddr) {
    * pEsmBaseAddr = PMIC_ESM_CTRL_REG_ADDR;
}

/**
 * @brief Check the state of ESM.
 * This function checks the state of ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmCheckState(Pmic_CoreHandle_t *pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t regAddr = 0U;
    uint8_t regData = 0U;

    regAddr = PMIC_ESM_CTRL_REG_ADDR;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (Pmic_getBitField(regData, PMIC_ESM_CTRL_REG_SHIFT,
            PMIC_ESM_CTRL_REG_MASK) == PMIC_ESM_VAL_1)) {
        pmicStatus = PMIC_ST_ERR_ESM_STARTED;
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief Start or stop the ESM.
 * This function starts or stops the ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmBaseRegAddr Base register address for ESM.
 * @param esmState State of ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmXStart(Pmic_CoreHandle_t *pPmicCoreHandle,
    const bool esmState) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t regAddr = 0U;
    uint8_t regData = 0U;
    uint8_t esmU8Val = 0U;

    regAddr = PMIC_ESM_CTRL_REG_ADDR;
    esmU8Val = Pmic_esmGetU8Val(esmState);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Read the PMIC ESM_MCU/ESM_SOC START Register */
    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

    /* Write 0 or 1 to ESM_MCU/ESM_SOC START BIT to Start/Stop ESM */
    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( &regData, PMIC_ESM_CTRL_REG_SHIFT, PMIC_ESM_CTRL_REG_MASK,
            esmU8Val);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief Enable or disable ESM.
 * This function enables or disables ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmBaseRegAddr Base register address for ESM.
 * @param esmToggle Toggle value for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmXEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
    const bool esmToggle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t regAddr = 0U;
    uint8_t regData = 0U;
    uint8_t esmU8Val = 0U;

    /* Check if ESM is started */
    pmicStatus = Pmic_esmCheckState(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        regAddr = PMIC_ESM_CFG1_REG;
        esmU8Val = Pmic_esmGetU8Val(esmToggle);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Read the PMIC ESM MODE CFG Register */
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

        /* Configure ESM_X_EN(where X is ESM_MCU or ESM_SOC) Bit to enable
         * or Disable.
         */
        if (PMIC_ST_SUCCESS == pmicStatus) {
            Pmic_setBitField( &regData, PMIC_ESM_CFG1_ESM_EN_SHIFT,
                (uint8_t)PMIC_ESM_CFG1_ESM_EN_MASK, esmU8Val);

            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Set the delay value for ESM.
 * This function sets the delay value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmSetDelay1Value(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_DELAY1_REG;
    if (PMIC_ESM_DELAY_MICROSEC_MAX < esmCfg.esmDelay1_us) {
        pmicStatus = PMIC_ST_ERR_INV_ESM_VAL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        regData = (uint8_t)(esmCfg.esmDelay1_us / PMIC_ESM_DELAY_MICROSEC_DIV);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Set the delay value for ESM delay 2.
 * This function sets the delay value for ESM delay 2.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmSetDelay2Value(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_DELAY2_REG;
    if (PMIC_ESM_DELAY_MICROSEC_MAX < esmCfg.esmDelay2_us) {
        pmicStatus = PMIC_ST_ERR_INV_ESM_VAL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        regData = (uint8_t)(esmCfg.esmDelay2_us / PMIC_ESM_DELAY_MICROSEC_DIV);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Set the error count threshold value for ESM.
 * This function sets the error count threshold value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmSetErrCntThrValue(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_ERR_STAT_REG;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( &regData, PMIC_ESM_ERR_STAT_ESM_ERR_CNT_SHIFT,
            (uint8_t)PMIC_ESM_ERR_STAT_ESM_ERR_CNT_MASK, esmCfg.esmErrCntThr);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief Set the maximum high pulse duration value for ESM.
 * This function sets the maximum high pulse duration value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmSetHmaxValue(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_HMAX_CFG_REG;

    if ((PMIC_ESM_PWM_PULSE_MICROSEC_MAX < esmCfg.esmHmax_us) ||
        (PMIC_ESM_PWM_PULSE_MICROSEC_MIN > esmCfg.esmHmax_us)) {
        pmicStatus = PMIC_ST_ERR_INV_ESM_VAL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        regData = (uint8_t)((esmCfg.esmHmax_us - PMIC_ESM_PWM_PULSE_MICROSEC_MIN) /
            PMIC_ESM_PWM_PULSE_MICROSEC_DIV);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Set the minimum high pulse duration value for ESM.
 * This function sets the minimum high pulse duration value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmSetHminValue(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_HMIN_CFG_REG;

    if ((PMIC_ESM_PWM_PULSE_MICROSEC_MAX < esmCfg.esmHmin_us) ||
        (PMIC_ESM_PWM_PULSE_MICROSEC_MIN > esmCfg.esmHmin_us)) {
        pmicStatus = PMIC_ST_ERR_INV_ESM_VAL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        regData = (uint8_t)((esmCfg.esmHmin_us - PMIC_ESM_PWM_PULSE_MICROSEC_MIN) /
            PMIC_ESM_PWM_PULSE_MICROSEC_DIV);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Set the maximum low pulse duration value for ESM.
 * This function sets the maximum low pulse duration value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmSetLmaxValue(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_LMAX_CFG_REG;

    if ((PMIC_ESM_PWM_PULSE_MICROSEC_MAX < esmCfg.esmLmax_us) ||
        (PMIC_ESM_PWM_PULSE_MICROSEC_MIN > esmCfg.esmLmax_us)) {
        pmicStatus = PMIC_ST_ERR_INV_ESM_VAL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        regData = (uint8_t)((esmCfg.esmLmax_us - PMIC_ESM_PWM_PULSE_MICROSEC_MIN) /
            PMIC_ESM_PWM_PULSE_MICROSEC_DIV);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Set the minimum low pulse duration value for ESM.
 * This function sets the minimum low pulse duration value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmSetLminValue(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_LMIN_CFG_REG;

    if ((PMIC_ESM_PWM_PULSE_MICROSEC_MAX < esmCfg.esmLmin_us) ||
        (PMIC_ESM_PWM_PULSE_MICROSEC_MIN > esmCfg.esmLmin_us)) {
        pmicStatus = PMIC_ST_ERR_INV_ESM_VAL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        regData = (uint8_t)((esmCfg.esmLmin_us - PMIC_ESM_PWM_PULSE_MICROSEC_MIN) /
            PMIC_ESM_PWM_PULSE_MICROSEC_DIV);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Get the maximum high pulse duration value for ESM.
 * This function gets the maximum high pulse duration value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmGetHmaxValue(Pmic_CoreHandle_t *pPmicCoreHandle,
    Pmic_EsmCfg_t *pEsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_HMAX_CFG_REG;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pEsmCfg -> esmHmax_us =
            ((uint16_t) regData * PMIC_ESM_PWM_PULSE_MICROSEC_DIV) +
            (uint16_t) PMIC_ESM_PWM_PULSE_MICROSEC_MIN;
    }

    return pmicStatus;
}

/**
 * @brief Get the minimum high pulse duration value for ESM.
 * This function gets the minimum high pulse duration value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmGetHminValue(Pmic_CoreHandle_t *pPmicCoreHandle,
    Pmic_EsmCfg_t *pEsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_HMIN_CFG_REG;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pEsmCfg -> esmHmin_us =
            ((uint16_t) regData * PMIC_ESM_PWM_PULSE_MICROSEC_DIV) +
            (uint16_t) PMIC_ESM_PWM_PULSE_MICROSEC_MIN;
    }

    return pmicStatus;
}

/**
 * @brief Get the maximum low pulse duration value for ESM.
 * This function gets the maximum low pulse duration value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmGetLmaxValue(Pmic_CoreHandle_t *pPmicCoreHandle,
    Pmic_EsmCfg_t *pEsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_LMAX_CFG_REG;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pEsmCfg -> esmLmax_us =
            ((uint16_t) regData * PMIC_ESM_PWM_PULSE_MICROSEC_DIV) +
            (uint16_t) PMIC_ESM_PWM_PULSE_MICROSEC_MIN;
    }

    return pmicStatus;
}

/**
 * @brief Get the minimum low pulse duration value for ESM.
 * This function gets the minimum low pulse duration value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmGetLminValue(Pmic_CoreHandle_t *pPmicCoreHandle,
    Pmic_EsmCfg_t *pEsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_LMIN_CFG_REG;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pEsmCfg -> esmLmin_us =
            ((uint16_t) regData * PMIC_ESM_PWM_PULSE_MICROSEC_DIV) +
            (uint16_t) PMIC_ESM_PWM_PULSE_MICROSEC_MIN;
    }

    return pmicStatus;
}

/**
 * @brief Set the mode for ESM.
 * This function sets the mode for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmSetMode(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;
    uint8_t esmU8Val = 0U;

    regAddr = PMIC_ESM_CFG1_REG;
    esmU8Val = Pmic_esmGetU8Val(esmCfg.esmMode);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_setBitField( &regData, PMIC_ESM_CFG1_ESM_CFG_SHIFT,
            (uint8_t)PMIC_ESM_CFG1_ESM_CFG_MASK, esmU8Val);

        pmicStatus =
            Pmic_commIntf_sendByte(pPmicCoreHandle, (uint16_t)regAddr, regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief Get the delay value for ESM delay 1.
 * This function gets the delay value for ESM delay 1.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmGetDelay1Value(Pmic_CoreHandle_t *pPmicCoreHandle,
    Pmic_EsmCfg_t *pEsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_DELAY1_REG;
    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pEsmCfg -> esmDelay1_us = (uint32_t) regData * PMIC_ESM_DELAY_MICROSEC_DIV;
    }

    return pmicStatus;
}

/**
 * @brief Get the delay value for ESM delay 2.
 * This function gets the delay value for ESM delay 2.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmGetDelay2Value(Pmic_CoreHandle_t *pPmicCoreHandle,
    Pmic_EsmCfg_t *pEsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_DELAY2_REG;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pEsmCfg -> esmDelay2_us = (uint32_t) regData * PMIC_ESM_DELAY_MICROSEC_DIV;
    }

    return pmicStatus;
}

/**
 * @brief Get the error count threshold value for ESM.
 * This function gets the error count threshold value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmGetErrCntThrValue(Pmic_CoreHandle_t *pPmicCoreHandle,
    Pmic_EsmCfg_t *pEsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;

    regAddr = PMIC_ESM_ERR_STAT_REG;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pEsmCfg -> esmErrCntThr =
            Pmic_getBitField(regData, PMIC_ESM_ERR_STAT_ESM_ERR_CNT_SHIFT,
                (uint8_t)PMIC_ESM_ERR_STAT_ESM_ERR_CNT_MASK);
    }

    return pmicStatus;
}

/**
 * @brief Get the mode value for ESM.
 * This function gets the mode value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmGetModeValue(Pmic_CoreHandle_t *pPmicCoreHandle,
    Pmic_EsmCfg_t *pEsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t regAddr = 0U;
    uint8_t bitFieldVal = 0U;

    regAddr = PMIC_ESM_CFG1_REG;
    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        bitFieldVal = Pmic_getBitField(regData, PMIC_ESM_CFG1_ESM_EN_SHIFT,
            (uint8_t)PMIC_ESM_CFG1_ESM_EN_MASK);
        if (bitFieldVal != 0U) {
            pEsmCfg -> esmMode = true;
        } else {
            pEsmCfg -> esmMode = false;
        }
    }

    return pmicStatus;
}

/**
 * @brief Start or stop the ESM module.
 * This function starts or stops the ESM module based on the specified type and
 * state.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param esmState State of ESM module (start or stop).
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_esmStart(Pmic_CoreHandle_t *pPmicCoreHandle,
    const bool esmState) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_esmGetBaseRegAddr( & esmBaseRegAddr);
        pmicStatus = Pmic_esmXStart(pPmicCoreHandle, esmState);
    }

    return pmicStatus;
}

/**
 * @brief Enable or disable the ESM module.
 * This function enables or disables the ESM module based on the specified type
 * and toggle.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param esmToggle Toggle to enable or disable the ESM module.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_esmEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
    const bool esmToggle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_esmGetBaseRegAddr( & esmBaseRegAddr);
        pmicStatus = Pmic_esmXEnable(pPmicCoreHandle, esmToggle);
    }

    return pmicStatus;
}

/**
 * @brief Get the enable state of the ESM module.
 * This function gets the enable state of the ESM module based on the specified
 * type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param pEsmState Pointer to store the enable state of the ESM module.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_esmGetEnableState(Pmic_CoreHandle_t *pPmicCoreHandle,
    bool *pEsmState) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr = 0U;
    uint16_t regAddr = 0U;
    uint8_t regData = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pEsmState)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_esmGetBaseRegAddr( & esmBaseRegAddr);
        regAddr = (uint16_t)esmBaseRegAddr + (uint16_t)PMIC_PMIC_ESM_CFG1_REG_OFFSET;
        *pEsmState = false;

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if ((PMIC_ST_SUCCESS == pmicStatus) &&
            (Pmic_getBitField(regData, PMIC_ESM_CFG1_ESM_EN_SHIFT,
                (uint8_t)PMIC_ESM_CFG1_ESM_EN_MASK) == PMIC_ESM_VAL_1)) {
            *pEsmState = true;
        }
    }

    return pmicStatus;
}

/**
 * @brief Set error count threshold and endrv/clr mode configuration for the ESM
 * module. This function sets the error count threshold and endrv/clr mode
 * configuration for the ESM module.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t
Pmic_esmSetErrcntthresholdEndrvClrModeCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (true == pmic_validParamCheck(esmCfg.validParams,
            PMIC_ESM_CFG_ERR_CNT_THR_VALID)) {
        if (PMIC_ESM_ERR_CNT_THR_MAX < esmCfg.esmErrCntThr) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }

        if (PMIC_ST_SUCCESS == pmicStatus) {
            pmicStatus = Pmic_esmSetErrCntThrValue(pPmicCoreHandle, esmCfg);
        }
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true ==
            pmic_validParamCheck(esmCfg.validParams, PMIC_ESM_CFG_MODE_VALID))) {
        /* Set ESM Mode */
        pmicStatus = Pmic_esmSetMode(pPmicCoreHandle, esmCfg);
    }

    return pmicStatus;
}

/**
 * @brief Set Hmax, Hmin, Lmax, and Lmin configuration for the ESM module.
 * This function sets the Hmax, Hmin, Lmax, and Lmin configuration for the ESM
 * module.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t
Pmic_esmSetHmaxHminLmaxLminCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (true ==
        pmic_validParamCheck(esmCfg.validParams, PMIC_ESM_CFG_HMAX_VALID)) {
        pmicStatus = Pmic_esmSetHmaxValue(pPmicCoreHandle, esmCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true ==
            pmic_validParamCheck(esmCfg.validParams, PMIC_ESM_CFG_HMIN_VALID))) {
        pmicStatus = Pmic_esmSetHminValue(pPmicCoreHandle, esmCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true ==
            pmic_validParamCheck(esmCfg.validParams, PMIC_ESM_CFG_LMAX_VALID))) {
        pmicStatus = Pmic_esmSetLmaxValue(pPmicCoreHandle, esmCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true ==
            pmic_validParamCheck(esmCfg.validParams, PMIC_ESM_CFG_LMIN_VALID))) {
        pmicStatus = Pmic_esmSetLminValue(pPmicCoreHandle, esmCfg);
    }

    return pmicStatus;
}

/**
 * @brief Set the configuration for the ESM module.
 * This function sets the configuration for the ESM module based on the
 * specified type and configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmSetConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (true ==
        pmic_validParamCheck(esmCfg.validParams, PMIC_ESM_CFG_DELAY1_VALID)) {
        pmicStatus = Pmic_esmSetDelay1Value(pPmicCoreHandle, esmCfg);
    }

    if (((PMIC_ST_SUCCESS == pmicStatus) &&
            (true ==
                pmic_validParamCheck(esmCfg.validParams, PMIC_ESM_CFG_DELAY2_VALID)))) {
        pmicStatus = Pmic_esmSetDelay2Value(pPmicCoreHandle, esmCfg);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_esmSetHmaxHminLmaxLminCfg(pPmicCoreHandle, esmCfg);
    }

    return pmicStatus;
}

/**
 * @brief Get Hmax, Hmin, Lmax, and Lmin configuration for the ESM module.
 * This function gets the Hmax, Hmin, Lmax, and Lmin configuration for the ESM
 * module.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t
Pmic_esmGetHmaxHminLmaxLminCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
    Pmic_EsmCfg_t *pEsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (true ==
        pmic_validParamCheck(pEsmCfg -> validParams, PMIC_ESM_CFG_HMAX_VALID)) {
        pmicStatus = Pmic_esmGetHmaxValue(pPmicCoreHandle, pEsmCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true ==
            pmic_validParamCheck(pEsmCfg -> validParams, PMIC_ESM_CFG_HMIN_VALID))) {
        pmicStatus = Pmic_esmGetHminValue(pPmicCoreHandle, pEsmCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true ==
            pmic_validParamCheck(pEsmCfg -> validParams, PMIC_ESM_CFG_LMAX_VALID))) {
        pmicStatus = Pmic_esmGetLmaxValue(pPmicCoreHandle, pEsmCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true ==
            pmic_validParamCheck(pEsmCfg -> validParams, PMIC_ESM_CFG_LMIN_VALID))) {
        pmicStatus = Pmic_esmGetLminValue(pPmicCoreHandle, pEsmCfg);
    }

    return pmicStatus;
}

/**
 * @brief Get the configuration for the ESM module.
 * This function gets the configuration for the ESM module based on the
 * specified type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmGetConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
    Pmic_EsmCfg_t *pEsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (true ==
        pmic_validParamCheck(pEsmCfg -> validParams, PMIC_ESM_CFG_DELAY1_VALID)) {
        pmicStatus = Pmic_esmGetDelay1Value(pPmicCoreHandle, pEsmCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true ==
            pmic_validParamCheck(pEsmCfg -> validParams, PMIC_ESM_CFG_DELAY2_VALID))) {
        pmicStatus = Pmic_esmGetDelay2Value(pPmicCoreHandle, pEsmCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true == pmic_validParamCheck(pEsmCfg -> validParams,
            PMIC_ESM_CFG_ERR_CNT_THR_VALID))) {
        pmicStatus = Pmic_esmGetErrCntThrValue(pPmicCoreHandle, pEsmCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (true ==
            pmic_validParamCheck(pEsmCfg -> validParams, PMIC_ESM_CFG_MODE_VALID))) {
        /* Get ESM Mode */
        pmicStatus = Pmic_esmGetModeValue(pPmicCoreHandle, pEsmCfg);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_esmGetHmaxHminLmaxLminCfg(pPmicCoreHandle, pEsmCfg);
    }

    return pmicStatus;
}

/**
 * @brief Set the configuration for the ESM module.
 * This function sets the configuration for the ESM module based on the
 * specified type and configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param esmCfg ESM configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_esmSetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U == esmCfg.validParams)) {
        pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_esmGetBaseRegAddr( & esmBaseRegAddr);
        pmicStatus = Pmic_esmCheckState(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_esmSetConfig(pPmicCoreHandle, esmCfg);
    }

    return pmicStatus;
}

/**
 * @brief Get the configuration for the ESM module.
 * This function gets the configuration for the ESM module based on the
 * specified type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_esmGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
    Pmic_EsmCfg_t *pEsmCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pEsmCfg)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (0U == pEsmCfg -> validParams)) {
        pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_esmGetBaseRegAddr( & esmBaseRegAddr);

        pmicStatus = Pmic_esmGetConfig(pPmicCoreHandle, pEsmCfg);
    }

    return pmicStatus;
}

/**
 * @brief Get the error count for the ESM module.
 * This function gets the error count for the ESM module based on the specified
 * type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param pEsmErrCnt Pointer to store the error count.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_esmGetErrCnt(Pmic_CoreHandle_t *pPmicCoreHandle,
    uint8_t * pEsmErrCnt) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr = 0U;
    uint16_t regAddr = 0U;
    uint8_t regData = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pEsmErrCnt)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_esmGetBaseRegAddr( & esmBaseRegAddr);
        regAddr = PMIC_ESM_ERR_STAT_REG;

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        * pEsmErrCnt = Pmic_getBitField(regData, PMIC_ESM_ERR_STAT_ESM_ERR_CNT_SHIFT,
            (uint8_t)PMIC_ESM_ERR_STAT_ESM_ERR_CNT_MASK);
    }

    return pmicStatus;
}

/**
 * @brief Get the status of the ESM module.
 * This function gets the status of the ESM module based on the specified type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param pEsmState Pointer to store the status of the ESM module.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_esmGetStatus(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t *pEsmState) {

    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr = 0U;
    uint16_t regAddr = 0U;
    uint8_t regData = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pEsmState)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_esmGetBaseRegAddr( & esmBaseRegAddr);

        regAddr = PMIC_ESM_CTRL_REG_ADDR;
        *pEsmState = PMIC_ESM_STOP;

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, (uint16_t)regAddr, &regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (Pmic_getBitField(regData, PMIC_ESM_CTRL_REG_SHIFT,
            PMIC_ESM_CTRL_REG_MASK) == PMIC_ESM_VAL_1)) {
        *pEsmState = PMIC_ESM_START;
    }

    return pmicStatus;
}

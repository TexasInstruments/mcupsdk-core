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
 *   @file    pmic_wdg.c
 *
 *   @brief   This file contains the API definitions for PMIC Watchdog
 *            configuration
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_wdg.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t
Pmic_checkPmicCoreHandle(const Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t drvInitStats = 0U;

    /* Validate pPmicCoreHandle */
    if (NULL == pPmicCoreHandle) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == status) {
        if (PMIC_INTF_SINGLE_I2C == pPmicCoreHandle -> commMode) {
            drvInitStats = (uint32_t)(DRV_INIT_SUCCESS | (uint8_t)PMIC_MAIN_INST);
        } else if (PMIC_INTF_DUAL_I2C == pPmicCoreHandle -> commMode) {
            drvInitStats = (uint32_t)(DRV_INIT_SUCCESS | (uint8_t)PMIC_MAIN_INST | (uint8_t)PMIC_QA_INST);
        } else if (PMIC_INTF_SPI == pPmicCoreHandle -> commMode) {
            drvInitStats = (uint32_t)(DRV_INIT_SUCCESS | (uint8_t)PMIC_MAIN_INST);
        } else {
            drvInitStats = 0x00U;
        }

        if (drvInitStats != pPmicCoreHandle -> drvInitStatus) {
            status = PMIC_ST_ERR_INV_HANDLE;
        }
    }

    return status;
}

static int32_t
Pmic_WdgValidatePmicCoreHandle(const Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t wdgStatus = PMIC_ST_SUCCESS;

    wdgStatus = Pmic_checkPmicCoreHandle(pPmicCoreHandle);

    /* Check the watch dog sub-system supported by pmic device */
    if (PMIC_ST_SUCCESS == wdgStatus) {
        if (true != pPmicCoreHandle -> pPmic_SubSysInfo -> wdgEnable) {
            wdgStatus = PMIC_ST_ERR_INV_DEVICE;
        }
    }

    return wdgStatus;
}

static uint8_t
Pmic_WdgCovertLongWinTimeIntervaltoRegBits(const Pmic_WdgCfg_t wdgCfg) {
    uint8_t regVal = 0U, baseVal = 0U;

    if (PMIC_WD_LONGWIN_80_MILLISEC == wdgCfg.longWinDuration_ms) {
        regVal = PMIC_WD_LONGWIN_REG_VAL_0;
    } else if (wdgCfg.longWinDuration_ms <= PMIC_WD_LONGWIN_8000_MILLISEC) {
        regVal =
            (uint8_t)(wdgCfg.longWinDuration_ms / PMIC_WD_LONGWIN_MILLISEC_DIV_125);
    } else {
        baseVal = (uint8_t)(PMIC_WD_LONGWIN_8000_MILLISEC /
            PMIC_WD_LONGWIN_MILLISEC_DIV_125);
        regVal =
            baseVal +
            (uint8_t)((wdgCfg.longWinDuration_ms - PMIC_WD_LONGWIN_8000_MILLISEC) /
                PMIC_WD_LONGWIN_MILLISEC_DIV_4000);
    }

    return regVal;
}

static int32_t
Pmic_WdgSetWindow1Window2TimeIntervals(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg) {
    int32_t wdgStatus = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg window1 time interval */
    if (true == pmic_validParamCheck(wdgCfg.validParams,
            PMIC_CFG_WDG_WIN1DURATION_VALID)) {
        if ((PMIC_WD_WIN1_2_MICROSEC_MIN > wdgCfg.win1Duration_us) ||
            (PMIC_WD_WIN1_2_MICROSEC_MAX < wdgCfg.win1Duration_us)) {
            wdgStatus = PMIC_ST_ERR_INV_WDG_WINDOW;
        }
        if (PMIC_ST_SUCCESS == wdgStatus) {
            regVal = (uint8_t)(
                (wdgCfg.win1Duration_us / PMIC_WD_WIN1_2_MICROSEC_DIV) - 1U);
            regVal &= (uint8_t) PMIC_WD_WIN1_CFG_WD_WIN1_MASK;

            Pmic_criticalSectionStart(pPmicCoreHandle);

            wdgStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_WIN1_CFG_REGADDR,
                regVal);

            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }

    /* Set wdg window2 time interval */
    if ((PMIC_ST_SUCCESS == wdgStatus) &&
        (true == pmic_validParamCheck(wdgCfg.validParams,
            PMIC_CFG_WDG_WIN2DURATION_VALID))) {
        if ((PMIC_WD_WIN1_2_MICROSEC_MIN > wdgCfg.win2Duration_us) ||
            (PMIC_WD_WIN1_2_MICROSEC_MAX < wdgCfg.win2Duration_us)) {
            wdgStatus = PMIC_ST_ERR_INV_WDG_WINDOW;
        }
        if (PMIC_ST_SUCCESS == wdgStatus) {
            regVal = (uint8_t)(
                (wdgCfg.win2Duration_us / PMIC_WD_WIN1_2_MICROSEC_DIV) - 1U);
            regVal &= (uint8_t) PMIC_WD_WIN2_CFG_WD_WIN2_MASK;

            Pmic_criticalSectionStart(pPmicCoreHandle);

            wdgStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_WIN2_CFG_REGADDR,
                regVal);

            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }

    return wdgStatus;
}

static int32_t
Pmic_WdgSetWindowsTimeIntervals(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg long window time interval */
    if (pmic_validParamCheck(wdgCfg.validParams,
            PMIC_CFG_WDG_LONGWINDURATION_VALID) == true) {
        if (((pPmicCoreHandle -> pmicDeviceType == PMIC_DEV_HERA_LP8764X) ||
                (pPmicCoreHandle -> pmicDeviceType == PMIC_DEV_LEO_TPS6594X)) &&
            (pPmicCoreHandle -> pmicDevSiliconRev == PMIC_SILICON_REV_ID_PG_1_0)) {
            if ((wdgCfg.longWinDuration_ms != PMIC_WD_LONGWIN_100_MILLISEC) &&
                ((wdgCfg.longWinDuration_ms < PMIC_WD_LONGWIN_MILLISEC_MIN) ||
                    (wdgCfg.longWinDuration_ms > PMIC_WD_LONGWIN_MILLISEC_MAX))) {
                status = PMIC_ST_ERR_INV_WDG_WINDOW;
            }

            if (status == PMIC_ST_SUCCESS) {
                if (wdgCfg.longWinDuration_ms == PMIC_WD_LONGWIN_100_MILLISEC) {
                    regVal = PMIC_WD_LONGWIN_REG_VAL_0;
                } else {
                    regVal = (uint8_t)(wdgCfg.longWinDuration_ms /
                        PMIC_WD_LONGWIN_MILLISEC_DIV);
                }
            }
        } else {
            if ((wdgCfg.longWinDuration_ms != PMIC_WD_LONGWIN_80_MILLISEC) &&
                ((wdgCfg.longWinDuration_ms < PMIC_WD_LONGWIN_MILLISEC_MIN_PG_2_0) ||
                    (wdgCfg.longWinDuration_ms > PMIC_WD_LONGWIN_MILLISEC_MAX_PG_2_0))) {
                status = PMIC_ST_ERR_INV_WDG_WINDOW;
            }

            if (status == PMIC_ST_SUCCESS) {
                regVal = Pmic_WdgCovertLongWinTimeIntervaltoRegBits(wdgCfg);
            }
        }
        if (status == PMIC_ST_SUCCESS) {
            Pmic_criticalSectionStart(pPmicCoreHandle);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                PMIC_WD_LONGWIN_CFG_REGADDR, regVal);

            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }

    if (status == PMIC_ST_SUCCESS) {
        /* Set wdg window1 and window2 time interval */
        status = Pmic_WdgSetWindow1Window2TimeIntervals(pPmicCoreHandle, wdgCfg);
    }

    return status;
}

static int32_t
Pmic_WdgGetWindow1Window2TimeIntervals(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgCfg_t * pWdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Get wdg window1 time interval */
    if (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_WIN1DURATION_VALID)) {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_WIN1_CFG_REGADDR, &
            regVal);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == status) {
            regVal &= (uint8_t) PMIC_WD_WIN1_CFG_WD_WIN1_MASK;

            pWdgCfg -> win1Duration_us =
                ((uint32_t) regVal + 1U) * (uint32_t) PMIC_WD_WIN1_2_MICROSEC_DIV;
        }
    }

    /* Get wdg window2 time interval */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_WIN2DURATION_VALID))) {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_WIN2_CFG_REGADDR, &
            regVal);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == status) {
            regVal &= (uint8_t) PMIC_WD_WIN2_CFG_WD_WIN2_MASK;

            pWdgCfg -> win2Duration_us =
                ((uint32_t) regVal + 1U) * (uint32_t) PMIC_WD_WIN1_2_MICROSEC_DIV;
        }
    }

    return status;
}

static int32_t
Pmic_WdgGetWindowsTimeIntervals(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgCfg_t * pWdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Get wdg long window time interval */
    if (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_LONGWINDURATION_VALID)) {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_WD_LONGWIN_CFG_REGADDR, & regVal);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == status) {
            if (((pPmicCoreHandle -> pmicDeviceType == PMIC_DEV_HERA_LP8764X) ||
                    (pPmicCoreHandle -> pmicDeviceType == PMIC_DEV_LEO_TPS6594X)) &&
                (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle -> pmicDevSiliconRev)) {
                if (PMIC_WD_LONGWIN_REG_VAL_0 == regVal) {
                    pWdgCfg -> longWinDuration_ms = PMIC_WD_LONGWIN_100_MILLISEC;
                } else {
                    pWdgCfg -> longWinDuration_ms =
                        (uint32_t) regVal * PMIC_WD_LONGWIN_MILLISEC_DIV;
                }
            } else {
                if (PMIC_WD_LONGWIN_REG_VAL_0 == regVal) {
                    pWdgCfg -> longWinDuration_ms = PMIC_WD_LONGWIN_80_MILLISEC;
                } else if (regVal <= PMIC_WD_LONGWIN_REG_VAL_64) {
                    pWdgCfg -> longWinDuration_ms =
                        (uint32_t) regVal * PMIC_WD_LONGWIN_MILLISEC_DIV_125;
                } else {
                    pWdgCfg -> longWinDuration_ms =
                        regVal - ((PMIC_WD_LONGWIN_REG_VAL_64 *
                                PMIC_WD_LONGWIN_MILLISEC_DIV_4000) +
                            PMIC_WD_LONGWIN_8000_MILLISEC);
                }
            }
        }
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Get wdg window1 and window2 time interval */
        status = Pmic_WdgGetWindow1Window2TimeIntervals(pPmicCoreHandle, pWdgCfg);
    }

    return status;
}

static int32_t Pmic_WdgSetThresholdValues(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg fail threshold value */
    if (true == pmic_validParamCheck(wdgCfg.validParams,
            PMIC_CFG_WDG_FAILTHRESHOLD_VALID)) {
        if (wdgCfg.failThreshold > PMIC_WDG_FAIL_THRESHOLD_COUNT_7) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_THR_CFG_REGADDR, &
                regVal);
        }
        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField( & regVal, PMIC_WD_THR_CFG_WD_FAIL_TH_SHIFT,
                (uint8_t) PMIC_WD_THR_CFG_WD_FAIL_TH_MASK,
                wdgCfg.failThreshold);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_THR_CFG_REGADDR,
                regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Set wdg reset threshold value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(wdgCfg.validParams,
            PMIC_CFG_WDG_RSTTHRESHOLD_VALID))) {
        if (wdgCfg.rstThreshold > PMIC_WDG_RESET_THRESHOLD_COUNT_7) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_THR_CFG_REGADDR, &
                regVal);
        }

        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField( & regVal, PMIC_WD_THR_CFG_WD_RST_TH_SHIFT,
                (uint8_t) PMIC_WD_THR_CFG_WD_RST_TH_MASK,
                wdgCfg.rstThreshold);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_THR_CFG_REGADDR,
                regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

static int32_t Pmic_WdgGetThresholdValues(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgCfg_t * pWdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_THR_CFG_REGADDR, & regVal);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    /* Get wdg fail threshold value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_FAILTHRESHOLD_VALID))) {
        pWdgCfg -> failThreshold =
            Pmic_getBitField(regVal, PMIC_WD_THR_CFG_WD_FAIL_TH_SHIFT,
                (uint8_t) PMIC_WD_THR_CFG_WD_FAIL_TH_MASK);
    }

    /* Get wdg reset threshold value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_RSTTHRESHOLD_VALID))) {
        pWdgCfg -> rstThreshold =
            Pmic_getBitField(regVal, PMIC_WD_THR_CFG_WD_RST_TH_SHIFT,
                (uint8_t)PMIC_WD_THR_CFG_WD_RST_TH_MASK);
    }

    /* Get wdg warm reset value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_RSTENABLE_VALID))) {
        pWdgCfg -> rstEnable =
            Pmic_getBitField(regVal, PMIC_WD_THR_CFG_WD_RST_EN_SHIFT,
                (uint8_t)PMIC_WD_THR_CFG_WD_RST_EN_MASK);
    }

    return status;
}

/*!
 * \brief  Function to set watchdog return long window control
 */
static int32_t Pmic_WdgSetRetToLongWindowCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
                                             uint8_t returnLongWindow) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;
    uint8_t retrnLongWindow = returnLongWindow;
    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR, &regVal);

    if (PMIC_ST_SUCCESS == status) {
        Pmic_setBitField(&regVal, PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_SHIFT,
            (uint8_t)PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_MASK, retrnLongWindow);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR,
            regVal);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief  Function to set wdg warm reset enable value
 */
static int32_t Pmic_WdgSetWarmRstEnableCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
                                           uint8_t rstEnble) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_THR_CFG_REGADDR, & regVal);

    if (PMIC_ST_SUCCESS == status) {
        Pmic_setBitField( &regVal, PMIC_WD_THR_CFG_WD_RST_EN_SHIFT,
            (uint8_t)PMIC_WD_THR_CFG_WD_RST_EN_MASK, rstEnble);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_THR_CFG_REGADDR,
            regVal);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief  Function to set wdg power hold value
 */
static int32_t Pmic_WdgSetPwrHoldCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
                                     uint8_t pwrHld) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR, &
        regVal);

    if (PMIC_ST_SUCCESS == status) {
        Pmic_setBitField( &regVal, PMIC_WD_MODE_REG_WD_PWRHOLD_SHIFT,
            (uint8_t)PMIC_WD_MODE_REG_WD_PWRHOLD_MASK, pwrHld);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR,
            regVal);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief  Function to set wdg Mode
 */
static int32_t Pmic_WdgSetModeCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
                                  uint8_t wdgMde) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR, &
        regVal);
    if (PMIC_ST_SUCCESS == status) {
        Pmic_setBitField( &regVal, PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT,
            (uint8_t)PMIC_WD_MODE_REG_WD_MODE_SELECT_MASK, wdgMde);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR,
            regVal);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_WdgSetCntSelCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t wdgCntSel) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0;

    if ((status == PMIC_ST_SUCCESS) &&
        (wdgCntSel > PMIC_WDG_CNT_SEL_2_1_SCHEME)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    Pmic_criticalSectionStart(pPmicCoreHandle);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR, &
            regVal);
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField( &regVal, PMIC_WD_MODE_REG_WD_CNT_SEL_SHIFT,
            (uint8_t)PMIC_WD_MODE_REG_WD_CNT_SEL_MASK, wdgCntSel);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR,
            regVal);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_WdgSetEnDrvSelCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t wdgEnDrvSel) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0;

    if ((status == PMIC_ST_SUCCESS) && (wdgEnDrvSel > PMIC_WDG_ENDRV_SEL_CLR)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    Pmic_criticalSectionStart(pPmicCoreHandle);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR, &
            regVal);
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField( &regVal, PMIC_WD_MODE_REG_WD_ENDRV_SEL_SHIFT,
            (uint8_t)PMIC_WD_MODE_REG_WD_ENDRV_SEL_MASK, wdgEnDrvSel);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR,
            regVal);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief  Function to set watchdog control parameters
 */
static int32_t Pmic_WdgSetCtrlParams(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Set wdg mode */
    if (true ==
        pmic_validParamCheck(wdgCfg.validParams, PMIC_CFG_WDG_WDGMODE_VALID)) {
        status = Pmic_WdgSetModeCfg(pPmicCoreHandle, wdgCfg.wdgMode);
    }

    /* Set wdg power hold value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true ==
            pmic_validParamCheck(wdgCfg.validParams, PMIC_CFG_WDG_PWRHOLD_VALID))) {
        status = Pmic_WdgSetPwrHoldCfg(pPmicCoreHandle, wdgCfg.pwrHold);
    }

    /* Set wdg warm reset enable value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(wdgCfg.validParams,
            PMIC_CFG_WDG_RSTENABLE_VALID))) {
        status = Pmic_WdgSetWarmRstEnableCfg(pPmicCoreHandle, wdgCfg.rstEnable);
    }

    /* Set wdg return to long window bit */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(wdgCfg.validParams,
            PMIC_CFG_WDG_RETLONGWIN_VALID))) {
        status = Pmic_WdgSetRetToLongWindowCfg(pPmicCoreHandle, wdgCfg.retLongWin);
    }

    /* Set wdg fail counter configuration */
    if ((PMIC_ST_SUCCESS == status) &&
        (true ==
            pmic_validParamCheck(wdgCfg.validParams, PMIC_CFG_WDG_CNT_SEL_VALID))) {
        status = Pmic_WdgSetCntSelCfg(pPmicCoreHandle, wdgCfg.cntSel);
    }

    /* Set wdg effect on ENDRV_SEL */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(wdgCfg.validParams,
            PMIC_CFG_WDG_ENDRV_SEL_VALID))) {
        status = Pmic_WdgSetEnDrvSelCfg(pPmicCoreHandle, wdgCfg.enDrvSel);
    }

    return status;
}

/*!
 * \brief  Function to get watchdog control parameters
 */
static int32_t Pmic_WdgGetCtrlParams(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgCfg_t * pWdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR, &
        regVal);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    /* Get wdg mode */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_WDGMODE_VALID))) {
        pWdgCfg -> wdgMode =
            Pmic_getBitField(regVal, PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT,
                (uint8_t)PMIC_WD_MODE_REG_WD_MODE_SELECT_MASK);
    }

    /* Get wdg power hold value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_PWRHOLD_VALID))) {
        pWdgCfg -> pwrHold =
            Pmic_getBitField(regVal, PMIC_WD_MODE_REG_WD_PWRHOLD_SHIFT,
                             (uint8_t)PMIC_WD_MODE_REG_WD_PWRHOLD_MASK);
    }

    /* Get wdg return to long window bit */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_RETLONGWIN_VALID))) {
        pWdgCfg -> retLongWin =
            Pmic_getBitField(regVal, PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_SHIFT,
                             (uint8_t)PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_MASK);
    }

    /* Get wdg fail counter configuration */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_CNT_SEL_VALID))) {
        pWdgCfg -> cntSel =
            Pmic_getBitField(regVal, PMIC_WD_MODE_REG_WD_CNT_SEL_SHIFT,
                             (uint8_t)PMIC_WD_MODE_REG_WD_CNT_SEL_MASK);
    }

    /* Get wdg ENDRV_SEL configuration */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_ENDRV_SEL_VALID))) {
        pWdgCfg -> enDrvSel =
            Pmic_getBitField(regVal, PMIC_WD_MODE_REG_WD_ENDRV_SEL_SHIFT,
                             (uint8_t)PMIC_WD_MODE_REG_WD_ENDRV_SEL_MASK);
    }

    return status;
}

/*!
 * \brief  Function to set watchdog QA Question Seed value
 */
static int32_t Pmic_wdgSetQaQuesSeedValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg QA Question Seed value */
    if (true == pmic_validParamCheck(wdgCfg.validParams,
            PMIC_CFG_WDG_QA_QUES_SEED_VALID)) {
        if (wdgCfg.qaQuesSeed > PMIC_WDG_QA_QUES_SEED_VALUE_15) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_QA_CFG_REGADDR, &
                regVal);
        }

        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField( &regVal, PMIC_WD_QA_CFG_WD_QUESTION_SEED_SHIFT,
                              (uint8_t)PMIC_WD_QA_CFG_WD_QUESTION_SEED_MASK, wdgCfg.qaQuesSeed);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_QA_CFG_REGADDR,
                regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief  Function to set watchdog QA configurations
 */
static int32_t Pmic_WdgSetQaConfigurations(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg QA Feedback value */
    if (true ==
        pmic_validParamCheck(wdgCfg.validParams, PMIC_CFG_WDG_QA_FDBK_VALID)) {
        if (wdgCfg.qaFdbk > PMIC_WDG_QA_FEEDBACK_VALUE_3) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_QA_CFG_REGADDR, &
                regVal);
        }
        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField( & regVal, PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT,
                              (uint8_t)PMIC_WD_QA_CFG_WD_QA_FDBK_MASK, wdgCfg.qaFdbk);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_QA_CFG_REGADDR,
                regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Set wdg QA LFSR value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true ==
            pmic_validParamCheck(wdgCfg.validParams, PMIC_CFG_WDG_QA_LFSR_VALID))) {
        if (wdgCfg.qaLfsr > PMIC_WDG_QA_LFSR_VALUE_3) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_QA_CFG_REGADDR, &
                regVal);
        }

        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField( &regVal, PMIC_WD_QA_CFG_WD_QA_LFSR_SHIFT,
                              (uint8_t)PMIC_WD_QA_CFG_WD_QA_LFSR_MASK, wdgCfg.qaLfsr);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_QA_CFG_REGADDR,
                regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Set wdg QA Question Seed value */
        status = Pmic_wdgSetQaQuesSeedValue(pPmicCoreHandle, wdgCfg);
    }

    return status;
}

/*!
 * \brief  Function to get watchdog QA configurations
 */
static int32_t Pmic_WdgGetQaConfigurations(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgCfg_t * pWdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_QA_CFG_REGADDR, & regVal);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    /* Get wdg QA Feedback value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_QA_FDBK_VALID))) {
        pWdgCfg -> qaFdbk = Pmic_getBitField(regVal, PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT,
                                             (uint8_t)PMIC_WD_QA_CFG_WD_QA_FDBK_MASK);
    }

    /* Get wdg QA LFSR value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_QA_LFSR_VALID))) {
        pWdgCfg -> qaLfsr = Pmic_getBitField(regVal, PMIC_WD_QA_CFG_WD_QA_LFSR_SHIFT,
                                             (uint8_t)PMIC_WD_QA_CFG_WD_QA_LFSR_MASK);
    }

    /* Get wdg QA Question Seed value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pWdgCfg -> validParams,
            PMIC_CFG_WDG_QA_QUES_SEED_VALID))) {
        pWdgCfg -> qaQuesSeed =
            Pmic_getBitField(regVal, PMIC_WD_QA_CFG_WD_QUESTION_SEED_SHIFT,
                             (uint8_t)PMIC_WD_QA_CFG_WD_QUESTION_SEED_MASK);
    }

    return status;
}

/*!
 * \brief  Function to Enable/Disable Watchdog Timer
 */
static int32_t Pmic_wdgEnDisState(Pmic_CoreHandle_t * pPmicCoreHandle,
                                  uint8_t enable) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_THR_CFG_REGADDR, & regVal);

    if (PMIC_ST_SUCCESS == status) {
        Pmic_setBitField( &regVal, PMIC_WD_THR_CFG_WD_EN_SHIFT,
                          (uint8_t)PMIC_WD_THR_CFG_WD_EN_MASK, enable);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_THR_CFG_REGADDR,
            regVal);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief  Function to get watchdog QA answer count and question value
 */
static int32_t
Pmic_wdgReadQuesandAnswerCount(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * pQaAnsCnt, uint8_t * pQaQuesCnt) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /*! Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /*! Reading answer count and question value */
    status =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_QA_CNT_REGADDR, & regVal);

    /*! Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == status) {
        * pQaAnsCnt =
            Pmic_getBitField(regVal, PMIC_WD_QUESTION_ANSW_CNT_WD_ANSW_CNT_SHIFT,
                             (uint8_t)PMIC_WD_QUESTION_ANSW_CNT_WD_ANSW_CNT_MASK);
        * pQaQuesCnt =
            Pmic_getBitField(regVal, PMIC_WD_QUESTION_ANSW_CNT_WD_QUESTION_SHIFT,
                             (uint8_t)PMIC_WD_QUESTION_ANSW_CNT_WD_QUESTION_MASK);
    }

    return status;
}

/*!
 * \brief  Function to get watchdog bad event
 */
static bool is_wdgBadEventDetected(Pmic_CoreHandle_t * pPmicCoreHandle) {
    uint8_t regVal = 0U;
    bool bitFieldVal = false;
    int32_t status = PMIC_ST_SUCCESS;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    status =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_STATUS_REGADDR, & regVal);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == status) &&
        (Pmic_getBitField(regVal, PMIC_WD_STAT_REG_WD_BAD_EVENT_SHIFT,
                          (uint8_t)PMIC_WD_STAT_REG_WD_BAD_EVENT_MASK) != 0U)) {
        bitFieldVal = true;
    }

    return bitFieldVal;
}

/*!
 * \brief  Function to get 4X1 mux output.
 *
 *         Note: In this API, the default case is for qaFdbk value is 3U.
 */
static uint8_t mux_4x1(uint8_t x0, uint8_t x1, uint8_t x2, uint8_t x3,
    uint8_t qaFdk) {
    uint8_t y = 0U;

    switch (qaFdk) {
    case 0U:
        y = x0;
        break;
    case 1U:
        y = x1;
        break;
    case 2U:
        y = x2;
        break;
    default:
        y = x3;
        break;
    }

    return y;
}

/*!
 * \brief  Function to Evaluate Watchdog Answers
 */
static uint8_t Pmic_getAnswerByte(uint8_t qaQuesCnt, uint8_t qaAnsCnt,
    uint8_t qaFbk) {
    uint8_t q0 = 0U, q1 = 0U, q2 = 0U, q3 = 0U;
    uint8_t a0 = 0U, a1 = 0U;
    uint8_t qaAns = 0U;

    q0 = (qaQuesCnt >> 0U) & 1U;
    q1 = (qaQuesCnt >> 1U) & 1U;
    q2 = (qaQuesCnt >> 2U) & 1U;
    q3 = (qaQuesCnt >> 3U) & 1U;

    a0 = (qaAnsCnt >> 0U) & 1U;
    a1 = (qaAnsCnt >> 1U) & 1U;

    /* Reference-Answer-X[0] */
    qaAns = mux_4x1(q0, q1, q2, q3, qaFbk) ^ mux_4x1(q3, q2, q1, q0, qaFbk) ^ a1;
    /* Reference-Answer-X[1] */
    qaAns |= (uint8_t)((mux_4x1(q0, q1, q2, q3, qaFbk) ^ mux_4x1(q2, q1, q0, q3, qaFbk) ^
            a1 ^ q1) << 1U);
    /* Reference-Answer-X[2] */
    qaAns |= (uint8_t)((mux_4x1(q0, q3, q1, q1, qaFbk) ^ mux_4x1(q3, q2, q1, q0, qaFbk) ^
            a1 ^ q1) << 2U);
    /* Reference-Answer-X[3] */
    qaAns |= (uint8_t)((mux_4x1(q2, q1, q0, q3, qaFbk) ^ mux_4x1(q0, q3, q2, q1, qaFbk) ^
            a1 ^ q3) << 3U);
    /* Reference-Answer-X[4] */
    qaAns |= (uint8_t)((mux_4x1(q1, q0, q2, q3, qaFbk) ^ a0) << 4U);
    /* Reference-Answer-X[5] */
    qaAns |= (uint8_t)((mux_4x1(q3, q2, q1, q0, qaFbk) ^ a0) << 5U);
    /* Reference-Answer-X[6] */
    qaAns |= (uint8_t)((mux_4x1(q0, q3, q2, q1, qaFbk) ^ a0) << 6U);
    /* Reference-Answer-X[7] */
    qaAns |= (uint8_t)((mux_4x1(q2, q1, q0, q3, qaFbk) ^ a0) << 7U);

    return qaAns;
}

/*!
 * \brief  Function to Evaluate and write Watchdog Answer based on
 *         qaFdbk, qaAnsCnt and qaQuesCnt Value
 */
static int32_t
Pmic_wdgQaEvaluateAndWriteAnswer(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t qaAnsCnt, uint8_t qaQuesCnt,
    uint8_t qaFbk) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t answer = 0U;

    answer = Pmic_getAnswerByte(qaQuesCnt, qaAnsCnt, qaFbk);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /*! Writing watch dog four Answers */
    status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_ANSWER_REG_REGADDR,
        answer);
    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief  Function to Evaluate and write Watchdog Four Answers based on
 *         qaFdbk Value
 */
static int32_t
Pmic_wdgQaEvaluateAndWriteAnswers(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t qaFbk) {
    int32_t status = PMIC_ST_SUCCESS;
    int8_t ansIndex = 0;
    uint8_t qaAnsCnt = 0U;
    uint8_t qaQuesCnt = 0U;

    for (ansIndex = 3; ansIndex >= 0; ansIndex--) {
        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_wdgReadQuesandAnswerCount(pPmicCoreHandle, & qaAnsCnt, &
                qaQuesCnt);
        }

        if ((PMIC_ST_SUCCESS == status) && (qaAnsCnt == (uint8_t) ansIndex)) {
            status = Pmic_wdgQaEvaluateAndWriteAnswer(pPmicCoreHandle, qaAnsCnt,
                qaQuesCnt, qaFbk);

            if ((PMIC_ST_SUCCESS == status) &&
                (true == is_wdgBadEventDetected(pPmicCoreHandle))) {
                status = PMIC_ST_ERR_INV_WDG_ANSWER;
                break;
            }
        }
    }

    return status;
}

/*!
 * \brief  Function to Evaluate and write Watchdog Four Answers
 */
static int32_t Pmic_wdgQaWriteAnswers(Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t qaFbk = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    status =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_QA_CFG_REGADDR, & qaFbk);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    /* Get wdg QA Feedback value */
    if (PMIC_ST_SUCCESS == status) {
        qaFbk = Pmic_getBitField(qaFbk, PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT,
                                 (uint8_t)PMIC_WD_QA_CFG_WD_QA_FDBK_MASK);
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Evaluate and write Watchdog Four Answers based on qaFdbk Value*/
        status = Pmic_wdgQaEvaluateAndWriteAnswers(pPmicCoreHandle, qaFbk);
    }

    return status;
}

int32_t Pmic_wdgEnable(Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgEnDisState(pPmicCoreHandle, PMIC_WDG_ENABLE);
    }

    return status;
}

int32_t Pmic_wdgDisable(Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgEnDisState(pPmicCoreHandle, PMIC_WDG_DISABLE);
    }

    return status;
}

int32_t Pmic_wdgGetEnableState(Pmic_CoreHandle_t *pPmicCoreHandle,
    uint8_t *pWdgEnabled) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0;

    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (pWdgEnabled == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    Pmic_criticalSectionStart(pPmicCoreHandle);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_THR_CFG_REGADDR, &
            regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *pWdgEnabled = Pmic_getBitField(regData, PMIC_WD_THR_CFG_WD_EN_SHIFT,
                                         (uint8_t)PMIC_WD_THR_CFG_WD_EN_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_wdgSetCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_WdgCfg_t wdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgSetWindowsTimeIntervals(pPmicCoreHandle, wdgCfg);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgSetThresholdValues(pPmicCoreHandle, wdgCfg);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgSetCtrlParams(pPmicCoreHandle, wdgCfg);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgSetQaConfigurations(pPmicCoreHandle, wdgCfg);
    }

    return status;
}

int32_t Pmic_wdgGetCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgCfg_t * pWdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == status) && (NULL == pWdgCfg)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgGetWindowsTimeIntervals(pPmicCoreHandle, pWdgCfg);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgGetThresholdValues(pPmicCoreHandle, pWdgCfg);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgGetCtrlParams(pPmicCoreHandle, pWdgCfg);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgGetQaConfigurations(pPmicCoreHandle, pWdgCfg);
    }

    return status;
}

/*!
 * \brief  API to set Watch Dog QA Mode, Disable ret to Long Window and
 *         Write Answers for Long Window
 */
static int32_t Pmic_wdgQaSetModeRetlongwinCfgWriteAnswersLongwindow(
    Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0x0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading watchdog mode value */
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR, &
        regVal);

    if (PMIC_ST_SUCCESS == status) {
        Pmic_setBitField( & regVal, PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT,
            (uint8_t)PMIC_WD_MODE_REG_WD_MODE_SELECT_MASK, PMIC_WDG_QA_MODE);
        /* Set watchdog mode to QA mode */
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR,
            regVal);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    /* Disable ret to Long Window */
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgSetRetToLongWindowCfg(pPmicCoreHandle,
            PMIC_WDG_RETLONGWIN_DISABLE);
    }

    /* Clear WDG Error bits */
    if (PMIC_ST_SUCCESS == status) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_ERR_STATUS_REGADDR, &
            regVal);
        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                PMIC_WD_ERR_STATUS_REGADDR, regVal);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Write Answers for Long Window */
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgQaWriteAnswers(pPmicCoreHandle);
        if (PMIC_ST_ERR_INV_WDG_ANSWER == status) {
            /* Start Critical Section */
            Pmic_criticalSectionStart(pPmicCoreHandle);

            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                PMIC_WD_ERR_STATUS_REGADDR, & regVal);

            /* Stop Critical Section */
            Pmic_criticalSectionStop(pPmicCoreHandle);

            if (PMIC_ST_SUCCESS == status) {
                if ((uint8_t)0U != (regVal & (uint8_t)0x01U)) {
                    status = PMIC_ST_ERR_INV_WDG_WINDOW;
                }
            }
        }
    }

    return status;
}

/*!
 * \brief  API to Write QA Answers for given numbers of sequences
 */
static int32_t
Pmic_wdgQaWriteAnswersNumSequence(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint32_t sequences, uint32_t maxCnt) {
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t loopCount = 0U;
    uint8_t failCnt = 0U;
    int8_t flag = 0;
    uint8_t regVal = 0x0U;
    uint32_t qaSequences = sequences;

    /* Write QA Answers for given numbers of sequences */
    while ((PMIC_ST_SUCCESS == status) &&
        ((PMIC_WD_QA_INFINITE_SEQ == qaSequences) || (qaSequences > 0U))) {
        /*! Write Answer to WDOG for the sequence */
        status = Pmic_wdgQaWriteAnswers(pPmicCoreHandle);

        if (PMIC_ST_ERR_INV_WDG_ANSWER == status) {
            /* Start Critical Section */
            Pmic_criticalSectionStart(pPmicCoreHandle);
            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                PMIC_WD_ERR_STATUS_REGADDR, & regVal);
            /* Stop Critical Section */
            Pmic_criticalSectionStop(pPmicCoreHandle);

            if (PMIC_ST_SUCCESS == status) {
                if ((uint8_t)0U != (regVal & (uint8_t)0x10U)) {
                    status = PMIC_ST_ERR_INV_WDG_WINDOW;
                    break;
                }
            }
        }

        /* Update loopCount value for while loop */
        loopCount = maxCnt;
        while ((PMIC_ST_SUCCESS == status) && (loopCount > 0U)) {
            /* Start Critical Section */
            Pmic_criticalSectionStart(pPmicCoreHandle);

            status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_STATUS_REGADDR, &
                failCnt);

            if ((PMIC_ST_SUCCESS == status) && ((uint8_t)0U != (failCnt & (uint8_t)0x40U))) {
                status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                    PMIC_WD_ERR_STATUS_REGADDR, & regVal);
                if ((PMIC_ST_SUCCESS == status) && ((uint8_t)0U != (regVal & (uint8_t)0x08U))) {
                    status = PMIC_ST_ERR_WDG_EARLY_ANSWER;
                }

                qaSequences = 0U;
                flag = 1;
            } else {
                if ((PMIC_ST_SUCCESS == status) && ((uint8_t)0U != (failCnt & (uint8_t)0x20U))) {
                    qaSequences--;
                    flag = 1;
                }
            }

            /* Stop Critical Section */
            Pmic_criticalSectionStop(pPmicCoreHandle);

            if (flag == 1) {
                break;
            }

            loopCount--;
        }
    }

    return status;
}

int32_t Pmic_wdgStartQaSequence(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint32_t num_of_sequences, uint32_t maxCnt) {
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t sequences = num_of_sequences;
    uint8_t regVal = 0x0U;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == status) && (maxCnt < PMIC_WDG_WAIT_CNT_MIN_VAL)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    /* Set Watchdog QA Mode, disable return to Long Window, and
     * write answers for Long Window */
    if (PMIC_ST_SUCCESS == status) {
        status =
            Pmic_wdgQaSetModeRetlongwinCfgWriteAnswersLongwindow(pPmicCoreHandle);
    }

    /* Dummy Read operations to sync the WatchDog */
    if (PMIC_ST_SUCCESS == status) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_ERR_STATUS_REGADDR, &
            regVal);

        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_STATUS_REGADDR, &
                regVal);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Write QA Answers for given numbers of sequences */
    if (PMIC_ST_SUCCESS == status) {
        status =
            Pmic_wdgQaWriteAnswersNumSequence(pPmicCoreHandle, sequences, maxCnt);
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Enable Return long window Enable */
        status = Pmic_WdgSetRetToLongWindowCfg(pPmicCoreHandle,
            PMIC_WDG_RETLONGWIN_ENABLE);
    }

    return status;
}

/*!
 * \brief  API to Get watchdog error status - SEQ_ERR, ANSW_ERR, FAIL_INT,
 *         RST_INT
 */
static void Pmic_wdgGetSeqAnswErrFailRstIntStat(Pmic_WdgErrStatus_t * pErrStatus,
    uint8_t regVal) {

    if (true == pmic_validParamCheck(pErrStatus -> validParams,
            PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID)) {
        if (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_SHIFT,
                (uint8_t)PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_MASK) != 0U) {
            pErrStatus -> wdSeqErr = true;
        } else {
            pErrStatus -> wdSeqErr = false;
        }
    }

    if (true == pmic_validParamCheck(pErrStatus -> validParams,
            PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID)) {
        if (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_SHIFT,
                (uint8_t)PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_MASK) != 0U) {
            pErrStatus -> wdAnswErr = true;
        } else {
            pErrStatus -> wdAnswErr = false;
        }
    }
    if (true == pmic_validParamCheck(pErrStatus -> validParams,
            PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID)) {
        if (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_SHIFT,
                (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_MASK) != 0U) {
            pErrStatus -> wdFailInt = true;
        } else {
            pErrStatus -> wdFailInt = false;
        }
    }

    if (true == pmic_validParamCheck(pErrStatus -> validParams,
            PMIC_CFG_WD_RST_INT_ERRSTAT_VALID)) {
        if (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_SHIFT,
                (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_MASK) != 0U) {
            pErrStatus -> wdRstInt = true;
        } else {
            pErrStatus -> wdRstInt = false;
        }
    }
}

/*!
 * \brief  API to Get watchdog error status - TRIG_EARLY, TIMEOUT,
 *         LONGWIN_TIMEOUT_INT, ANSW_EARLY
 */
static void Pmic_wdgGetLongwintointTimeoutTrigAnswEarlyErrStat(
    Pmic_WdgErrStatus_t * pErrStatus, uint8_t regVal) {

    if (true == pmic_validParamCheck(pErrStatus -> validParams,
            PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID)) {
        if (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_SHIFT,
                (uint8_t)PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_MASK) != 0U) {
            pErrStatus -> wdLongWinTimeout = true;
        } else {
            pErrStatus -> wdLongWinTimeout = false;
        }
    }

    if (true == pmic_validParamCheck(pErrStatus -> validParams,
            PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID)) {
        if (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_TMO_SHIFT,
                (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TMO_MASK) != 0U) {
            pErrStatus -> wdTimeout = true;
        } else {
            pErrStatus -> wdTimeout = false;
        }
    }

    if (true == pmic_validParamCheck(pErrStatus -> validParams,
            PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID)) {
        if (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_SHIFT,
                (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_MASK) != 0U) {
            pErrStatus -> wdTrigEarly = true;
        } else {
            pErrStatus -> wdTrigEarly = false;
        }
    }

    /* Get watchdog error status */
    if (true == pmic_validParamCheck(pErrStatus -> validParams,
            PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID)) {
        if (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_SHIFT,
                (uint8_t)PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_MASK) != 0U) {
            pErrStatus -> wdAnswearly = true;
        } else {
            pErrStatus -> wdAnswearly = false;
        }
    }
}

int32_t Pmic_wdgGetErrorStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgErrStatus_t * pErrStatus) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0x0U;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == status) && (NULL == pErrStatus)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading error status register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_ERR_STATUS_REGADDR, &
            regVal);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Get watchdog error status */
    if (PMIC_ST_SUCCESS == status) {
        /* Get watchdog error status - TRIG_EARLY, TIMEOUT, LONGWIN_TIMEOUT_INT,
         * ANSW_EARLY*/
        Pmic_wdgGetLongwintointTimeoutTrigAnswEarlyErrStat(pErrStatus, regVal);

        /* Get watchdog error status - SEQ_ERR, ANSW_ERR, FAIL_INT, RST_INT */
        Pmic_wdgGetSeqAnswErrFailRstIntStat(pErrStatus, regVal);
    }

    return status;
}

int32_t Pmic_wdgGetFailCntStat(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_WdgFailCntStat_t * pFailCount) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0x00U;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == status) && (NULL == pFailCount)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading error status register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_STATUS_REGADDR, &
            regVal);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Get watchdog Bad Event status */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pFailCount -> validParams,
            PMIC_CFG_WD_BAD_EVENT_STAT_VALID))) {
        if (Pmic_getBitField(regVal, PMIC_WD_STAT_REG_WD_BAD_EVENT_SHIFT,
                (uint8_t)PMIC_WD_STAT_REG_WD_BAD_EVENT_MASK) != 0U) {
            pFailCount -> wdBadEvent = true;
        } else {
            pFailCount -> wdBadEvent = false;
        }
    }

    /* Get watchdog Good Event status */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pFailCount -> validParams,
            PMIC_CFG_WD_GOOD_EVENT_STAT_VALID))) {
        if (Pmic_getBitField(regVal, PMIC_WD_STAT_REG_WD_FIRST_OK_SHIFT,
                (uint8_t)PMIC_WD_STAT_REG_WD_FIRST_OK_MASK) != 0U) {
            pFailCount -> wdGudEvent = true;
        } else {
            pFailCount -> wdGudEvent = false;
        }
    }

    /* Get watchdog Fail count Value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pFailCount -> validParams,
            PMIC_CFG_WD_FAIL_CNT_VAL_VALID))) {
        pFailCount -> wdFailCnt =
            Pmic_getBitField(regVal, PMIC_WD_STAT_REG_WD_ERR_CNT_SHIFT,
                (uint8_t)PMIC_WD_STAT_REG_WD_ERR_CNT_MASK);
    }

    return status;
}

int32_t Pmic_wdgStartTriggerSequence(Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0x0U;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == status) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading watchdog mode value */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR, &
            regVal);

        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField( & regVal, PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT,
                (uint8_t)PMIC_WD_MODE_REG_WD_MODE_SELECT_MASK,
                PMIC_WDG_TRIGGER_MODE);
            /* Set watchdog mode to trigger mode */
            status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR,
                regVal);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   API to clear PMIC watchdog error status based on wdgErrType
 */
static int32_t
Pmic_wdgClrErrStatusWdgErrType(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t wdgErrType, uint8_t regVal) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t errStatus = 1U, regData = 0U;

    if ((PMIC_WDG_ERR_LONG_WIN_TIMEOUT == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_MASK) != 0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_MASK, errStatus);
    } else if ((PMIC_WDG_ERR_TIMEOUT == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_TMO_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TMO_MASK) != 0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_TMO_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TMO_MASK, errStatus);
    } else if ((PMIC_WDG_ERR_TRIGGER_EARLY == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_MASK) !=
            0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_MASK, errStatus);
    } else if ((PMIC_WDG_ERR_ANSWER_EARLY == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_MASK) !=
            0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_MASK, errStatus);
    } else if ((PMIC_WDG_ERR_SEQ_ERR == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_MASK) != 0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_MASK, errStatus);
    } else if ((PMIC_WDG_ERR_ANS_ERR == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_MASK) != 0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_MASK, errStatus);
    } else if ((PMIC_WDG_ERR_THRES1 == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_MASK) != 0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_MASK, errStatus);
    } else {
        if (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_SHIFT,
                             (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_MASK) != 0U) {
            Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_SHIFT,
                              (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_MASK, errStatus);
        }
    }

    if (0U != regData) {
        /*! Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_ERR_STATUS_REGADDR,
            regData);
        /*! Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

int32_t Pmic_wdgClrErrStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t wdgErrType) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0x0U;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if ((PMIC_ST_SUCCESS == status) && (wdgErrType > PMIC_WDG_ERR_ALL)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading error status register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_ERR_STATUS_REGADDR, &
            regVal);
        if (0U == regVal) {
            status = PMIC_ST_ERR_FAIL;
        }

        if ((PMIC_ST_SUCCESS == status) && (PMIC_WDG_ERR_ALL == wdgErrType)) {
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                PMIC_WD_ERR_STATUS_REGADDR, regVal);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if ((PMIC_ST_SUCCESS == status) && (PMIC_WDG_ERR_ALL != wdgErrType)) {
        status =
            Pmic_wdgClrErrStatusWdgErrType(pPmicCoreHandle, wdgErrType, regVal);
    }

    return status;
}

int32_t Pmic_wdgQaSequenceWriteAnswer(Pmic_CoreHandle_t * pPmicCoreHandle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t qaAnsCnt = 0U;
    uint8_t qaQuesCnt = 0U;
    uint8_t qaFbk = 0U;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    /* Write Answers for Long Window */
    if (PMIC_ST_SUCCESS == status) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status =
            Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_QA_CFG_REGADDR, & qaFbk);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Get wdg QA Feedback value */
    if (PMIC_ST_SUCCESS == status) {
        qaFbk = Pmic_getBitField(qaFbk, PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT,
                                 (uint8_t)PMIC_WD_QA_CFG_WD_QA_FDBK_MASK);
    }

    if (PMIC_ST_SUCCESS == status) {
        status =
            Pmic_wdgReadQuesandAnswerCount(pPmicCoreHandle, & qaAnsCnt, & qaQuesCnt);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgQaEvaluateAndWriteAnswer(pPmicCoreHandle, qaAnsCnt,
            qaQuesCnt, qaFbk);
    }

    return status;
}

int32_t Pmic_wdgBeginSequences(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t wdgMde) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t wdgEnabled = 0U;
    uint8_t regVal = 0U;

    /* Parameter check*/
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (wdgMde > PMIC_WDG_QA_MODE)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    /* Check if WDG is enabled*/
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_wdgGetEnableState(pPmicCoreHandle, &wdgEnabled);
        if ((status == PMIC_ST_SUCCESS) && (wdgEnabled == 0U)) {
            status = PMIC_ST_ERR_WDG_DISABLED;
        }
    }

    /* Start critical section before serial comm. operations*/
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Clear all WDG error statuses (all bit fields in the*/
    /* WD_ERR_STATUS register are W1C - write 1 to clear)*/
    regVal = 0xFFU;
    status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_ERR_STATUS_REGADDR,
        regVal);

    if (status == PMIC_ST_SUCCESS) {
        /* Read WD_MODE_REG register*/
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR, &
            regVal);
    }

    if (status == PMIC_ST_SUCCESS) {
        /* Select WDG mode of operation*/
        Pmic_setBitField( & regVal, PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT,
                          (uint8_t)PMIC_WD_MODE_REG_WD_MODE_SELECT_MASK, wdgMde);

        /* Clear WD_PWRHOLD bit field to enable WDG to exit Long Window*/
        Pmic_setBitField( & regVal, PMIC_WD_MODE_REG_WD_PWRHOLD_SHIFT,
                          (uint8_t)PMIC_WD_MODE_REG_WD_PWRHOLD_MASK,
                          PMIC_WDG_PWRHOLD_DISABLE);

        /* Clear WD_RETURN_LONGWIN bit field to disable return to Long Window*/
        Pmic_setBitField( & regVal, PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_SHIFT,
                          (uint8_t)PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_MASK,
                          PMIC_WDG_RETLONGWIN_DISABLE);

        /* Write new register value back to PMIC*/
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_WD_MODE_REG_REGADDR,
            regVal);
    }

    /* Stop critical section after serial comm. operations*/
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

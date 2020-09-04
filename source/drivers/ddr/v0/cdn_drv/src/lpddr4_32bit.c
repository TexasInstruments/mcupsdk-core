/******************************************************************************
 * Copyright (C) 2012-2021 Cadence Design Systems, Inc.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************
 * lpddr4_32bit.c
 *
 * Implementation of driver API functions for 32bit configuration
 ******************************************************************************
 */
#include "../include/common/cdn_errno.h"

#include "../include/common/cps_drv_lpddr4.h"
#include "../include/16bit/lpddr4_ctl_regs.h"
#include "../include/common/lpddr4_if.h"
#include "lpddr4.h"
#include "../include/common/lpddr4_structs_if.h"

static void LPDDR4_SetRxOffsetError(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_DebugInfo* debugInfo, bool* errorFound);

uint32_t LPDDR4_EnablePIInitiator(const LPDDR4_PrivateData* pD)
{
    uint32_t result = 0U;
    uint32_t regVal = 0U;

    LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

    /* Enable PI as the initiator for DRAM */
    regVal = CPS_FLD_SET(LPDDR4__PI_INIT_LVL_EN__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__PI_INIT_LVL_EN__REG)));
    regVal = CPS_FLD_SET(LPDDR4__PI_NORMAL_LVL_SEQ__FLD, regVal);
    CPS_REG_WRITE((&(ctlRegBase->LPDDR4__PI_INIT_LVL_EN__REG)), regVal);
    return result;
}

uint32_t LPDDR4_GetCtlInterruptMask(const LPDDR4_PrivateData* pD, uint64_t* mask) {
    uint32_t result = 0U;
    uint32_t lowerMask = 0U;

    result = LPDDR4_GetCtlInterruptMaskSF(pD,  mask);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        /* Reading the lower mask register */
        lowerMask = (uint32_t)(CPS_FLD_READ(LPDDR4__INT_MASK_0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_MASK_0__REG))));
        /* Reading the upper mask register */
        *mask     = (uint64_t)(CPS_FLD_READ(LPDDR4__INT_MASK_1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_MASK_1__REG))));
        /* Concatenate both register informations */
        *mask     = (uint64_t)((*mask << WORD_SHIFT) | lowerMask);
    }
    return result;
}

uint32_t LPDDR4_SetCtlInterruptMask(const LPDDR4_PrivateData* pD, const uint64_t* mask) {
    uint32_t result;
    uint32_t regVal = 0;
    const uint64_t ui64One   = 1ULL;
    const uint32_t ui32IrqCount = (uint32_t)LPDDR4_INTR_LOR_BITS + 1U;

    result = LPDDR4_SetCtlInterruptMaskSF(pD, mask);
    if ((result == (uint32_t)CDN_EOK) && (ui32IrqCount < 64U)) {
        /* Return if the user given value is higher than the field width */
        if (*mask >= (ui64One << ui32IrqCount)) {
            result = (uint32_t)CDN_EINVAL;
        }
    }

    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        /* Extracting the lower 32 bits and writing to lower mask register */
        regVal = (uint32_t)(*mask & WORD_MASK);
        regVal = CPS_FLD_WRITE(LPDDR4__INT_MASK_0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_MASK_0__REG)), regVal);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_MASK_0__REG), regVal);

        /* Extracting the upper 32 bits and writing to upper mask register */
        regVal = (uint32_t)((*mask >> WORD_SHIFT) & WORD_MASK);
        regVal = CPS_FLD_WRITE(LPDDR4__INT_MASK_1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_MASK_1__REG)), regVal);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_MASK_1__REG), regVal);
    }
    return result;
}

uint32_t LPDDR4_CheckCtlInterrupt(const LPDDR4_PrivateData* pD, LPDDR4_INTR_CtlInterrupt intr, bool* irqStatus) {
    uint32_t result;
    uint32_t ctlIrqStatus = 0;
    uint32_t fieldShift = 0;

    /* NOTE:This function assume irq status is mentioned in NOT more than 2 registers.
     * Value of 'interrupt' should be less than 64 */
    result = LPDDR4_INTR_CheckCtlIntSF(pD, intr, irqStatus);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        if ((uint32_t)intr >=  (uint32_t)WORD_SHIFT) {
            ctlIrqStatus = CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_1__REG));
            /* Reduce the shift value as we are considering upper register */
            fieldShift   = (uint32_t)intr - ((uint32_t)WORD_SHIFT);
        } else {
            ctlIrqStatus = CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_0__REG));
            /* The shift value remains same for lower interrupt register */
            fieldShift   = (uint32_t)intr;
        }

        /* MISRA compliance (Shifting operation) check */
        if (fieldShift < WORD_SHIFT) {
            if (((ctlIrqStatus >> fieldShift) & BIT_MASK) > 0U) {
                *irqStatus = true;
            } else {
                *irqStatus = false;
            }
        }
    }
    return result;
}

uint32_t LPDDR4_AckCtlInterrupt(const LPDDR4_PrivateData* pD, LPDDR4_INTR_CtlInterrupt intr) {
    uint32_t result = 0;
    uint32_t regVal = 0;
    uint32_t localInterrupt = (uint32_t)intr;

    /* NOTE:This function assume irq status is mentioned in NOT more than 2 registers.
     * Value of 'interrupt' should be less than 64 */
    result = LPDDR4_INTR_AckCtlIntSF(pD, intr);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        /* Check if the requested bit is in upper register */
        if (localInterrupt >  WORD_SHIFT) {
            localInterrupt = (localInterrupt - (uint32_t)WORD_SHIFT);
            regVal = ((uint32_t)BIT_MASK << localInterrupt);
            CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_1__REG), regVal);
        } else {
            regVal = ((uint32_t)BIT_MASK << localInterrupt);
            CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_0__REG), regVal);
        }
    }

    return result;
}

/* Check for  wrLvlError */
void LPDDR4_CheckWrLvlError(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_DebugInfo* debugInfo, bool* errFoundPtr) {

    uint32_t regVal;
    uint32_t errBitMask = 0U;
    uint32_t snum;
    volatile uint32_t* regAddress;

    regAddress = (volatile uint32_t*)(&(ctlRegBase->LPDDR4__PHY_WRLVL_ERROR_OBS_0__REG));
    /* PHY_WRLVL_ERROR_OBS_X[1:0] should be zero */
    errBitMask = (BIT_MASK << 1) | (BIT_MASK);
    for (snum = 0U; snum < DSLICE_NUM; snum++) {
        regVal = CPS_REG_READ(regAddress);
        if ((regVal & errBitMask) != 0U) {
            debugInfo->wrLvlError = CDN_TRUE;
            *errFoundPtr = true;
        }
        regAddress = LPDDR4_AddOffset(regAddress, (uint32_t)SLICE_WIDTH);
    }
}

static void LPDDR4_SetRxOffsetError(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_DebugInfo* debugInfo, bool* errorFound) {

    volatile uint32_t* regAddress;
    uint32_t snum = 0U;
    uint32_t errBitMask = 0U;
    uint32_t regVal = 0U;

    /* Check for rxOffsetError*/
    if (*errorFound == (bool)false) {
        regAddress = (volatile uint32_t*) (&(ctlRegBase->LPDDR4__PHY_RX_CAL_LOCK_OBS_0__REG));
        errBitMask = (RX_CAL_DONE) | (NIBBLE_MASK);
        /* PHY_RX_CAL_LOCK_OBS_x[4] – RX_CAL_DONE : should be high
           PHY_RX_CAL_LOCK_OBS_x[3:0] – RX_CAL_STATE : should be zero. */
        for (snum = (uint32_t)0U; snum < DSLICE_NUM; snum++) {
            regVal = CPS_FLD_READ(LPDDR4__PHY_RX_CAL_LOCK_OBS_0__FLD, CPS_REG_READ(regAddress));
            if ((regVal & errBitMask) != RX_CAL_DONE) {
                debugInfo->rxOffsetError = (uint8_t)true;
                *errorFound = true;
            }
            regAddress = LPDDR4_AddOffset(regAddress, (uint32_t)SLICE_WIDTH);
        }
    }
}

uint32_t LPDDR4_GetDebugInitInfo(const LPDDR4_PrivateData* pD, LPDDR4_DebugInfo* debugInfo) {

    uint32_t result = 0U;
    bool errorFound = false;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_GetDebugInitInfoSF(pD, debugInfo);
    if (result == (uint32_t)CDN_EOK) {

        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        LPDDR4_SetErrors(ctlRegBase, debugInfo, (uint8_t *)&errorFound);
        /* Function to setup Snap for OBS registers */
        LPDDR4_SetSettings(ctlRegBase, errorFound);
        /* Function to check for Rx offset error */
        LPDDR4_SetRxOffsetError (ctlRegBase, debugInfo, &errorFound);
        /* Function Check various levelling errors */
        errorFound = (bool)LPDDR4_CheckLvlErrors(pD, debugInfo, errorFound);
    }

    if (errorFound == (bool)true) {
        result = (uint32_t)CDN_EPROTO;
    }

    return result;
}

uint32_t LPDDR4_GetEccEnable(const LPDDR4_PrivateData* pD, LPDDR4_EccEnable* eccParam) {
    uint32_t result = 0U;
    uint32_t fldVal = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_GetEccEnableSF(pD, eccParam);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        /* Reading the ECC_Enable field  from the register.*/
        fldVal = CPS_FLD_READ(LPDDR4__ECC_ENABLE__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__ECC_ENABLE__REG)));
        switch (fldVal) {
        case 3:
            *eccParam = LPDDR4_ECC_ERR_DETECT_CORRECT;
            break;
        case 2:
            *eccParam = LPDDR4_ECC_ERR_DETECT;
            break;
        case 1:
            *eccParam = LPDDR4_ECC_ENABLED;
            break;
        default:
            /* Default ECC (Sanity function already confirmed the value to be in expected range.)*/
            *eccParam = LPDDR4_ECC_DISABLED;
            break;
        }
    }
    return result;
}

uint32_t LPDDR4_SetEccEnable(const LPDDR4_PrivateData* pD, const LPDDR4_EccEnable* eccParam) {

    uint32_t result = 0U;
    uint32_t regVal = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_SetEccEnableSF(pD, eccParam);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        /* Updating the ECC_Enable field based on the user given value.*/
        regVal = CPS_FLD_WRITE(LPDDR4__ECC_ENABLE__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__ECC_ENABLE__REG)), *eccParam);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__ECC_ENABLE__REG), regVal);
    }
    return result;
}

uint32_t LPDDR4_GetReducMode(const LPDDR4_PrivateData* pD, LPDDR4_ReducMode* mode) {
    uint32_t result = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_GetReducModeSF(pD, mode);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        /* Read the value of reduc parameter. */
        if (CPS_FLD_READ(LPDDR4__REDUC__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__REDUC__REG))) == 0U) {
            *mode = LPDDR4_REDUC_ON;
        } else {
            *mode = LPDDR4_REDUC_OFF;
        }
    }
    return result;
}
uint32_t LPDDR4_SetReducMode(const LPDDR4_PrivateData* pD, const LPDDR4_ReducMode* mode) {
    uint32_t result = 0U;
    uint32_t regVal = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_SetReducModeSF(pD, mode);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        /* Setting to enable Half data path. */
        regVal = (uint32_t)CPS_FLD_WRITE(LPDDR4__REDUC__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__REDUC__REG)), *mode);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__REDUC__REG), regVal);
    }
    return result;
}

uint32_t LPDDR4_CheckMmrReadError(const LPDDR4_PrivateData* pD, uint64_t* mmrValue, uint8_t* mrrStatus) {

    uint32_t lowerData;
    LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
    uint32_t result  = (uint32_t)CDN_EOK;

    /* Check if mode register read error interrupt occurred */
    if (LPDDR4_PollCtlIrq(pD, LPDDR4_INTR_MRR_ERROR, 100) == 0U) {
        /* Mode register read error interrupt, read MRR status register and return.*/
        *mrrStatus = (uint8_t)CPS_FLD_READ(LPDDR4__MRR_ERROR_STATUS__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__MRR_ERROR_STATUS__REG)));
        *mmrValue  = (uint64_t)0;
        result = (uint32_t)CDN_EIO;
    } else {
        *mrrStatus = (uint8_t)0;
        /* Mode register read was successful, read DATA */
        lowerData = CPS_REG_READ(&(ctlRegBase->LPDDR4__PERIPHERAL_MRR_DATA_0__REG));
        *mmrValue = CPS_REG_READ(&(ctlRegBase->LPDDR4__PERIPHERAL_MRR_DATA_1__REG));
        *mmrValue = (uint64_t)((*mmrValue << WORD_SHIFT) | lowerData);
        /* Acknowledge MR_READ_DONE interrupt to clear it */
        result = LPDDR4_AckCtlInterrupt(pD, LPDDR4_INTR_MR_READ_DONE);
    }
    return result;
}

/**
 *  * Internal Function: To fetch the read-write mask for the PHY data slice registers.
 * @param[in] dSliceNum The data slice
 * @param[in] arrayOffset The offset in data slice read-write array
 * @return the read-write mask for the corresponding offset.
 */
uint32_t LPDDR4_getDSliceMask(uint32_t dSliceNum, uint32_t arrayOffset ){

    uint32_t rwMask      = 0U;

    /* Fetch the read-write mask from the respective mask array */
    switch (dSliceNum) {
    case 0:
        if (arrayOffset < DSLICE0_REG_COUNT) {
            rwMask = g_lpddr4_data_slice_0_rw_mask[arrayOffset];
        }
        break;
    case 1:
        if (arrayOffset < DSLICE1_REG_COUNT) {
            rwMask = g_lpddr4_data_slice_1_rw_mask[arrayOffset];
        }
        break;
    case 2:
        if (arrayOffset < DSLICE2_REG_COUNT) {
            rwMask = g_lpddr4_data_slice_2_rw_mask[arrayOffset];
        }
        break;
    default:
        /* Since the regOffset is valid, should be data_slice_3 */
        if (arrayOffset < DSLICE3_REG_COUNT) {
            rwMask = g_lpddr4_data_slice_3_rw_mask[arrayOffset];
        }
        break;
    }
    return rwMask;
}

/* parasoft-end-suppress METRICS-36-3 */

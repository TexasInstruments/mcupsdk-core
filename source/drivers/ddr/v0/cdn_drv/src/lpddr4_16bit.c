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
 * lpddr4_16bit.c
 *
 * Cadence DRAM Core Driver
 * Implementation specific to 16bit series
 ******************************************************************************
 */
#include "../include/common/cdn_errno.h" 
#include "../include/common/cdn_log.h"
#include "../include/common/cps_drv_lpddr4.h"
#include "../include/16bit/lpddr4_ctl_regs.h"
#include "../include/common/lpddr4_if.h"
#include "lpddr4.h"
#include "../include/common/lpddr4_structs_if.h"

static uint16_t CtlIntMap[51][3] = {
    {0, 0, 7},
    {1, 0, 8},
    {2, 0, 9},
    {3, 0, 14},
    {4, 0, 15},
    {5, 0, 16},
    {6, 0, 17},
    {7, 0, 19},
    {8, 1, 0},
    {9, 2, 0},
    {10, 2, 3},
    {11, 3, 0},
    {12, 4, 0},
    {13, 5, 11},
    {14, 5, 12},
    {15, 5, 13},
    {16, 5, 14},
    {17, 5, 15},
    {18, 6, 0},
    {19, 6, 1},
    {20, 6, 2},
    {21, 6, 6},
    {22, 6, 7},
    {23, 7, 3},
    {24, 7, 4},
    {25, 7, 5},
    {26, 7, 6},
    {27, 7, 7},
    {28, 8, 0},
    {29, 9, 0},
    {30, 10, 0},
    {31, 10, 1},
    {32, 10, 2},
    {33, 10, 3},
    {34, 10, 4},
    {35, 10, 5},
    {36, 11, 0},
    {37, 12, 0},
    {38, 12, 1},
    {39, 12, 2},
    {40, 12, 3},
    {41, 12, 4},
    {42, 12, 5},
    {43, 13, 0},
    {44, 13, 1},
    {45, 13, 3},
    {46, 14, 0},
    {47, 14, 2},
    {48, 14, 3},
    {49, 15, 2},
    {50, 16, 0},
};

static void LPDDR4_CheckCtlInterrupt_4(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_INTR_CtlInterrupt intr,
                                       uint32_t* ctlGrpIrqStatus, uint32_t* CtlMasterIntFlag);
static void LPDDR4_CheckCtlInterrupt_3(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_INTR_CtlInterrupt intr,
                                       uint32_t* ctlGrpIrqStatus, uint32_t* CtlMasterIntFlag);
static void LPDDR4_CheckCtlInterrupt_2(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_INTR_CtlInterrupt intr,
                                       uint32_t* ctlGrpIrqStatus, uint32_t* CtlMasterIntFlag);
static void LPDDR4_AckCtlInterrupt_4(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_INTR_CtlInterrupt intr);
static void LPDDR4_AckCtlInterrupt_3(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_INTR_CtlInterrupt intr);
static void LPDDR4_AckCtlInterrupt_2(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_INTR_CtlInterrupt intr);
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions" */

uint32_t LPDDR4_EnablePIInitiator(const LPDDR4_PrivateData* pD)
{
    uint32_t result = 0U;
    uint32_t regVal = 0U;

    LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

    /* Enable PI as the initiator for DRAM */
    regVal = CPS_FLD_SET(LPDDR4__PI_NORMAL_LVL_SEQ__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__PI_NORMAL_LVL_SEQ__REG)));
    CPS_REG_WRITE((&(ctlRegBase->LPDDR4__PI_NORMAL_LVL_SEQ__REG)), regVal);
    regVal = CPS_FLD_SET(LPDDR4__PI_INIT_LVL_EN__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__PI_INIT_LVL_EN__REG)));
    CPS_REG_WRITE((&(ctlRegBase->LPDDR4__PI_INIT_LVL_EN__REG)), regVal);

    return result;
}

uint32_t LPDDR4_GetCtlInterruptMask(const LPDDR4_PrivateData* pD, uint64_t* mask) {
    uint32_t result = 0U;

    result = LPDDR4_GetCtlInterruptMaskSF(pD,  mask);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        /* Reading the appropriate mask register */
        *mask = (uint64_t)(CPS_FLD_READ(LPDDR4__INT_MASK_MASTER__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_MASK_MASTER__REG))));
    }
    return result;
}

uint32_t LPDDR4_SetCtlInterruptMask(const LPDDR4_PrivateData* pD, const uint64_t* mask) {
    uint32_t result;
    uint32_t regVal = 0;
    const uint64_t ui64One   = 1ULL;
    const uint32_t ui32IrqCount = (uint32_t)32U;

    result = LPDDR4_SetCtlInterruptMaskSF(pD, mask);
    if ((result == (uint32_t)CDN_EOK) && (ui32IrqCount < 64U)) {
        /* Return if the user given value is higher than the field width */
        if (*mask >= (ui64One << ui32IrqCount)) {
            result = (uint32_t)CDN_EINVAL;
        }
    }

    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        /* Extracting the value from the register and writing to mask register again after modify*/
        regVal = CPS_FLD_WRITE(LPDDR4__INT_MASK_MASTER__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_MASK_MASTER__REG)), *mask);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_MASK_MASTER__REG), regVal);
    }
    return result;
}

static void LPDDR4_CheckCtlInterrupt_4(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_INTR_CtlInterrupt intr,
                                       uint32_t* ctlGrpIrqStatus, uint32_t* CtlMasterIntFlag) {

    /* Check interrupt status for the interrupt source.*/
    /* Checked interrupt status related to init process */
    if ((intr >= LPDDR4_INTR_INIT_MEM_RESET_DONE) && (intr <= LPDDR4_INTR_INIT_POWER_ON_STATE)) {
        *ctlGrpIrqStatus = CPS_FLD_READ(LPDDR4__INT_STATUS_INIT__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_INIT__REG)));
    }
    /* Checked interrupt status related to mode register settings */
    else if ((intr >= LPDDR4_INTR_MRR_ERROR) && (intr <= LPDDR4_INTR_MR_WRITE_DONE)) {
        *ctlGrpIrqStatus = CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_MODE__REG));
    }
    else if (intr == LPDDR4_INTR_BIST_DONE) {
        *ctlGrpIrqStatus = CPS_FLD_READ(LPDDR4__INT_STATUS_BIST__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_BIST__REG)));
    }
    else if (intr == LPDDR4_INTR_PARITY_ERROR) {
        *ctlGrpIrqStatus = CPS_FLD_READ(LPDDR4__INT_STATUS_PARITY__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_PARITY__REG)));
    }
    else {
        *CtlMasterIntFlag = (uint32_t)1U;
    }
}

static void LPDDR4_CheckCtlInterrupt_3(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_INTR_CtlInterrupt intr,
                                       uint32_t* ctlGrpIrqStatus, uint32_t* CtlMasterIntFlag) {

    /* Check interrupt status for the interrupt source.*/
    /* Checked interrupt status related to frequency settings */
    if ((intr >= LPDDR4_INTR_FREQ_DFS_REQ_HW_IGNORE) && (intr <= LPDDR4_INTR_FREQ_DFS_SW_DONE)) {
        *ctlGrpIrqStatus = CPS_FLD_READ(LPDDR4__INT_STATUS_FREQ__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_FREQ__REG)));
    }
    /* Checked interrupt status related to low power mode */
    else if ((intr >= LPDDR4_INTR_LP_DONE) && (intr <= LPDDR4_INTR_LP_TIMEOUT)) {
        *ctlGrpIrqStatus = CPS_FLD_READ(LPDDR4__INT_STATUS_LOWPOWER__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_LOWPOWER__REG)));
    }
    else {
        /*Splitted interrupt checking into sub functions due to MISRA rules*/
        LPDDR4_CheckCtlInterrupt_4(ctlRegBase, intr, ctlGrpIrqStatus, CtlMasterIntFlag);
    }
}

static void LPDDR4_CheckCtlInterrupt_2(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_INTR_CtlInterrupt intr,
                                       uint32_t* ctlGrpIrqStatus, uint32_t* CtlMasterIntFlag) {

    /* Check interrupt status for the interrupt source.*/
    /* Checked interrupt status related to timeout operation */
    if (intr <= LPDDR4_INTR_TIMEOUT_AUTO_REFRESH_MAX) {
        *ctlGrpIrqStatus = CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_TIMEOUT__REG));
    }
    /* Checked interrupt status related to training operation */
    else if ((intr >= LPDDR4_INTR_TRAINING_ZQ_STATUS) && (intr <= LPDDR4_INTR_TRAINING_DQS_OSC_VAR_OUT)) {
        *ctlGrpIrqStatus = CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_TRAINING__REG));
    }
    /* Checked interrupt status related to user interface settings */
    else if ((intr >= LPDDR4_INTR_USERIF_OUTSIDE_MEM_ACCESS) && (intr <= LPDDR4_INTR_USERIF_INVAL_SETTING)) {
        *ctlGrpIrqStatus = CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_USERIF__REG));
    }
    /* Checked interrupt status related to misc settings */
    else if ((intr >= LPDDR4_INTR_MISC_MRR_TRAFFIC) && (intr <= LPDDR4_INTR_MISC_REFRESH_STATUS)) {
        *ctlGrpIrqStatus = CPS_FLD_READ(LPDDR4__INT_STATUS_MISC__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_MISC__REG)));
    }
    /* Checked interrupt status related to DFI settings */
    else if ((intr >= LPDDR4_INTR_DFI_UPDATE_ERROR) && (intr <= LPDDR4_INTR_DFI_TIMEOUT)) {
        *ctlGrpIrqStatus = CPS_FLD_READ(LPDDR4__INT_STATUS_DFI__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_DFI__REG)));
    }
    else {
        /*Splitted interrupt checking into sub functions due to MISRA rules*/
        LPDDR4_CheckCtlInterrupt_3(ctlRegBase, intr, ctlGrpIrqStatus, CtlMasterIntFlag);
    }
}

uint32_t LPDDR4_CheckCtlInterrupt(const LPDDR4_PrivateData* pD, LPDDR4_INTR_CtlInterrupt intr, bool* irqStatus) {
    uint32_t result;
    uint32_t ctlMasterIrqStatus = 0U;
    uint32_t ctlGrpIrqStatus = 0U;
    uint32_t CtlMasterIntFlag = 0U;

    /* NOTE:This function assume irq status is mentioned in master and group register.
     * This function check the set bit in both the registers master as well as group.
     * function returns a irqstatus true based on the set bits
     */
    result = LPDDR4_INTR_CheckCtlIntSF(pD, intr, irqStatus);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        /* Checked interrupt source bits in master register */
        ctlMasterIrqStatus = (CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_STATUS_MASTER__REG)) & (~((uint32_t)1 << 31)));

        /*Splitted interrupt checking into sub functions due to MISRA rules*/
        LPDDR4_CheckCtlInterrupt_2(ctlRegBase, intr, &ctlGrpIrqStatus,&CtlMasterIntFlag);

        /* MISRA compliance (Shifting operation) check */
        if ((CtlIntMap[intr][INT_SHIFT] < WORD_SHIFT) && (CtlIntMap[intr][GRP_SHIFT] < WORD_SHIFT)) {
            if ((((ctlMasterIrqStatus  >> CtlIntMap[intr][GRP_SHIFT]) & BIT_MASK) > 0U) &&
                (((ctlGrpIrqStatus >> CtlIntMap[intr][INT_SHIFT]) & BIT_MASK) > 0U) &&
                (CtlMasterIntFlag == (uint32_t)0)) {
                *irqStatus = true;
            } else if ((((ctlMasterIrqStatus >> CtlIntMap[intr][GRP_SHIFT]) & BIT_MASK) > 0U) &&
                       (CtlMasterIntFlag == (uint32_t)1U)) {
                *irqStatus = true;
            }
            else {
                *irqStatus = false;
            }
        }
    }
    return result;
}

static void LPDDR4_AckCtlInterrupt_4(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_INTR_CtlInterrupt intr) {
    uint32_t regVal = 0;

    /* Set appropriate ACK bit for respective interrupt source*/
    /* Handled interrupts related to mode register settings and verify intr number within range*/
    if ((intr >= LPDDR4_INTR_MRR_ERROR) && (intr <= LPDDR4_INTR_MR_WRITE_DONE) && ((uint32_t)CtlIntMap[intr][INT_SHIFT] < WORD_SHIFT)) {
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_MODE__REG), (uint32_t)BIT_MASK << (uint32_t)CtlIntMap[intr][INT_SHIFT]);
    }
    /* Handled interrupts related to BIST functionalities and verify intr number within range*/
    else if ((intr == LPDDR4_INTR_BIST_DONE) && ((uint32_t)CtlIntMap[intr][INT_SHIFT] < WORD_SHIFT)) {
        regVal = CPS_FLD_WRITE(LPDDR4__INT_ACK_BIST__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_ACK_BIST__REG)),
                               (uint32_t)BIT_MASK << (uint32_t)CtlIntMap[intr][INT_SHIFT]);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_BIST__REG), regVal);
    }
    /* Handled interrupts related to parity and verify intr number within range*/
    else if ((intr == LPDDR4_INTR_PARITY_ERROR) && ((uint32_t)CtlIntMap[intr][INT_SHIFT] < WORD_SHIFT)) {
        regVal = CPS_FLD_WRITE(LPDDR4__INT_ACK_PARITY__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_ACK_PARITY__REG)),
                               (uint32_t)BIT_MASK << (uint32_t)CtlIntMap[intr][INT_SHIFT]);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_PARITY__REG), regVal);
    }
    else {
        /* MISRA compliant */
    }
}

static void LPDDR4_AckCtlInterrupt_3(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_INTR_CtlInterrupt intr) {
    uint32_t regVal = 0;

    /* Set appropriate ACK bit for respective interrupt source*/
    /* Handled interrupts related to low powermode */
    if ((intr >= LPDDR4_INTR_LP_DONE) && (intr <= LPDDR4_INTR_LP_TIMEOUT) && ((uint32_t)CtlIntMap[intr][INT_SHIFT] < WORD_SHIFT)) {
        regVal = CPS_FLD_WRITE(LPDDR4__INT_ACK_LOWPOWER__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_ACK_LOWPOWER__REG)),
                               (uint32_t)((uint32_t)BIT_MASK << (uint32_t)CtlIntMap[intr][INT_SHIFT]));
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_LOWPOWER__REG), regVal);
    }
    /* Handled interrupts related to initialization sequence */
    else if ((intr >= LPDDR4_INTR_INIT_MEM_RESET_DONE) && (intr <= LPDDR4_INTR_INIT_POWER_ON_STATE) && ((uint32_t)CtlIntMap[intr][INT_SHIFT] < WORD_SHIFT)) {
        regVal = CPS_FLD_WRITE(LPDDR4__INT_ACK_INIT__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_ACK_INIT__REG)),
                               (uint32_t)((uint32_t)BIT_MASK << (uint32_t)CtlIntMap[intr][INT_SHIFT]));
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_INIT__REG), regVal);
    }
    else {
        /*Splitted into another function due to misra rules*/
        LPDDR4_AckCtlInterrupt_4(ctlRegBase, intr);
    }
}

static void  LPDDR4_AckCtlInterrupt_2(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_INTR_CtlInterrupt intr) {
    uint32_t regVal = 0;

    /* Set appropriate ACK bit for respective interrupt source*/
    /* Handled interrupts related to DFI interfaces */
    if ((intr >= LPDDR4_INTR_DFI_UPDATE_ERROR) && (intr <= LPDDR4_INTR_DFI_TIMEOUT) && ((uint32_t)CtlIntMap[intr][INT_SHIFT] < WORD_SHIFT)) {
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_DFI__REG), (uint32_t)((uint32_t)BIT_MASK << (uint32_t)CtlIntMap[intr][INT_SHIFT]));
    }
    /* Handled interrupts related to frequency settings */
    else if ((intr >= LPDDR4_INTR_FREQ_DFS_REQ_HW_IGNORE) && (intr <= LPDDR4_INTR_FREQ_DFS_SW_DONE) && ((uint32_t)CtlIntMap[intr][INT_SHIFT] < WORD_SHIFT)) {
        regVal = CPS_FLD_WRITE(LPDDR4__INT_ACK_FREQ__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__INT_ACK_FREQ__REG)),
                               (uint32_t)((uint32_t)BIT_MASK << (uint32_t)CtlIntMap[intr][INT_SHIFT]));
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_FREQ__REG), regVal);
    }
    else {
        /*Splitted into another function due to misra rules*/
        LPDDR4_AckCtlInterrupt_3(ctlRegBase, intr);
    }
}

uint32_t LPDDR4_AckCtlInterrupt(const LPDDR4_PrivateData* pD, LPDDR4_INTR_CtlInterrupt intr) {
    uint32_t result;

    /* NOTE:This function set the respective bit the ACK register based on the interrupt type */
    result = LPDDR4_INTR_AckCtlIntSF(pD, intr);
    if ((result == (uint32_t)CDN_EOK) && ((uint32_t)CtlIntMap[intr][INT_SHIFT] < WORD_SHIFT)) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        /* Handled interrupts related to timeout settings and verify intr number within range*/
        if (intr <= LPDDR4_INTR_TIMEOUT_AUTO_REFRESH_MAX) {
            CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_TIMEOUT__REG), ((uint32_t)BIT_MASK << (uint32_t)CtlIntMap[intr][INT_SHIFT]));
        }
        /* Handled interrupts related to training and calibration settings and verify intr number within range*/
        else if ((intr >= LPDDR4_INTR_TRAINING_ZQ_STATUS) && (intr <= LPDDR4_INTR_TRAINING_DQS_OSC_VAR_OUT)) {
            CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_TRAINING__REG), ((uint32_t)BIT_MASK << (uint32_t)CtlIntMap[intr][INT_SHIFT]));
        }
        /* Handled interrupts related to user interface settings and verify intr number within range*/
        else if ((intr >= LPDDR4_INTR_USERIF_OUTSIDE_MEM_ACCESS) && (intr <= LPDDR4_INTR_USERIF_INVAL_SETTING)) {
            CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_USERIF__REG), ((uint32_t)BIT_MASK << (uint32_t)CtlIntMap[intr][INT_SHIFT]));
        }
        /* Handled interrupts related to misc settings and verify intr number within range*/
        else if ((intr >= LPDDR4_INTR_MISC_MRR_TRAFFIC) && (intr <= LPDDR4_INTR_MISC_REFRESH_STATUS)) {
            CPS_REG_WRITE(&(ctlRegBase->LPDDR4__INT_ACK_MISC__REG), ((uint32_t)BIT_MASK << (uint32_t)CtlIntMap[intr][INT_SHIFT]));
        }
        else {
            /*Splitted into another function due to misra rules*/
            LPDDR4_AckCtlInterrupt_2(ctlRegBase, intr);
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

    regAddress = (volatile uint32_t*)(&(ctlRegBase->LPDDR4__PHY_WRLVL_STATUS_OBS_0__REG));
    /* Bit 12 is used for error check */
    errBitMask = ((uint32_t)BIT_MASK << (uint32_t)12U);
    for (snum = 0U; snum < DSLICE_NUM; snum++) {
        regVal = CPS_REG_READ(regAddress);
        if ((regVal & errBitMask) != 0U) {
            debugInfo->wrLvlError = CDN_TRUE;
            *errFoundPtr = true;
        }
        regAddress = LPDDR4_AddOffset(regAddress, (uint32_t)SLICE_WIDTH);
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
        /* Function Check various levelling errors */
        errorFound = (bool)LPDDR4_CheckLvlErrors(pD, debugInfo, errorFound);
    }

    if (errorFound == (bool)true) {
        result = (uint32_t)CDN_EPROTO;
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
        if (CPS_FLD_READ(LPDDR4__MEM_DP_REDUCTION__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__MEM_DP_REDUCTION__REG))) == 0U) {
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
        regVal = (uint32_t)CPS_FLD_WRITE(LPDDR4__MEM_DP_REDUCTION__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__MEM_DP_REDUCTION__REG)), *mode);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__MEM_DP_REDUCTION__REG), regVal);
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
        lowerData = CPS_REG_READ(&(ctlRegBase->LPDDR4__PERIPHERAL_MRR_DATA__REG));
        *mmrValue = (uint64_t)((*mmrValue << WORD_SHIFT) | lowerData);
        /* Acknowledge MR_READ_DONE interrupt to clear it */
        result = LPDDR4_AckCtlInterrupt(pD, LPDDR4_INTR_MR_READ_DONE);
    }
    return result;
}

/**
 * Internal Function: To fetch the read-write mask for the PHY data slice registers.
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
    default:
        /* Since the regOffset is valid, should be data_slice_3 */
        if (arrayOffset < DSLICE1_REG_COUNT) {
            rwMask = g_lpddr4_data_slice_1_rw_mask[arrayOffset];
        }
        break;

    }
    return rwMask;
}

/*Dummy functions. This feature is not supported in 16 bit version*/
uint32_t LPDDR4_GetEccEnable(const LPDDR4_PrivateData* pD, LPDDR4_EccEnable* eccParam)
{
    uint32_t result = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_GetEccEnableSF(pD, eccParam);
    if (result == (uint32_t)CDN_EOK) {
        *eccParam = LPDDR4_ECC_DISABLED;
        result = (uint32_t)CDN_EOPNOTSUPP;
    }

    return result;
}
uint32_t LPDDR4_SetEccEnable(const LPDDR4_PrivateData* pD, const LPDDR4_EccEnable* eccParam)
{
    uint32_t result = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_SetEccEnableSF(pD, eccParam);
    if (result == (uint32_t)CDN_EOK) {
        /*This feature is not supported in 16 bit version*/
        result = (uint32_t)CDN_EOPNOTSUPP;
    }

    return result;
}
/* parasoft-end-suppress METRICS-36-3 */

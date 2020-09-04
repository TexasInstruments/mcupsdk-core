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
 * lpddr4.c
 *
 * Cadence DRAM Core Driver
 * Implementation of driver API functions
 ******************************************************************************
 */
#include "../include/common/cdn_errno.h"
#include "../include/common/cps_drv_lpddr4.h"
#include "../include/common/lpddr4_if.h"
#include "lpddr4.h"
#include "../include/common/lpddr4_structs_if.h"

#define  CPS_DelayNs(x) {uint32_t i; for(i=0U; i<x ;i++) {} }

/* Define prototypes for all static functions */
/* to complient with MISRA Rule 8.1 */
static uint32_t LPDDR4_PollPhyIndepIrq(const LPDDR4_PrivateData* pD, LPDDR4_INTR_PhyIndepInterrupt irqBit,  uint32_t delay);
static uint32_t LPDDR4_PollAndAckIrq(const LPDDR4_PrivateData* pD);
static uint32_t LPDDR4_StartSequenceController(const LPDDR4_PrivateData* pD);
static uint32_t LPDDR4_WriteMmrRegister(const LPDDR4_PrivateData* pD, uint32_t writeModeRegVal);
static void LPDDR4_CheckCaTrainingError(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_DebugInfo* debugInfo, bool* errFoundPtr);
static void LPDDR4_CheckGateLvlError(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_DebugInfo* debugInfo, bool* errFoundPtr);
static void LPDDR4_CheckReadLvlError(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_DebugInfo* debugInfo, bool* errFoundPtr);
static void LPDDR4_CheckDqTrainingError(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_DebugInfo* debugInfo, bool* errFoundPtr);
static uint8_t LPDDR4_SetError(volatile uint32_t* reg, uint32_t errBitMask, uint8_t* errFoundPtr, const uint32_t errorInfoBits);
static void LPDDR4_SetPhySnapSettings(LPDDR4_CtlRegs* ctlRegBase, const bool errorFound);
static void LPDDR4_SetPhyAdrSnapSettings(LPDDR4_CtlRegs* ctlRegBase, const bool errorFound);
static void readPDWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles);
static void readSRShortWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles);
static void readSRLongWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles);
static void readSRLongGateWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles);
static void readSRDPShortWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles);
static void readSRDPLongWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles);
static void readSRDPLongGateWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles);
static void LPDDR4_ReadLpiWakeUpTime(LPDDR4_CtlRegs* ctlRegBase, const LPDDR4_LpiWakeUpParam* lpiWakeUpParam, const LPDDR4_CtlFspNum* fspNum, uint32_t* cycles);
static void writePDWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles);
static void writeSRShortWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles);
static void writeSRLongWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles);
static void writeSRLongGateWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles);
static void writeSRDPShortWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles);
static void writeSRDPLongWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles);
static void writeSRDPLongGateWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles);
static void LPDDR4_WriteLpiWakeUpTime(LPDDR4_CtlRegs* ctlRegBase, const LPDDR4_LpiWakeUpParam* lpiWakeUpParam, const LPDDR4_CtlFspNum* fspNum, const uint32_t* cycles);
static void LPDDR4_UpdateFSP2RefRateParams(const LPDDR4_PrivateData* pD, const uint32_t* tref, const uint32_t* tras_max);
static void LPDDR4_UpdateFSP1RefRateParams(const LPDDR4_PrivateData* pD, const uint32_t* tref, const uint32_t* tras_max);
static void LPDDR4_UpdateFSP0RefRateParams(const LPDDR4_PrivateData* pD, const uint32_t* tref, const uint32_t* tras_max);
static uint32_t LPDDR4_getPhyRwMask( uint32_t regOffset );
#ifdef REG_WRITE_VERIF
static uint32_t LPDDR4_VerifyRegWrite( const LPDDR4_PrivateData* pD,LPDDR4_RegBlock cpp, uint32_t regOffset, uint32_t regValue);
#endif
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions" */

/**
 * Internal Function:Poll for status of interrupt received by the Controller.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] irqBit Interrupt status bit to be checked.
 * @param[in] delay time delay.
 * @return CDN_EOK on success (Interrupt status high).
 * @return CDN_EIO on poll time out.
 * @return CDN_EINVAL checking status was not successful.
 */
uint32_t LPDDR4_PollCtlIrq(const LPDDR4_PrivateData* pD, LPDDR4_INTR_CtlInterrupt irqBit,  uint32_t delay) {

    uint32_t result = 0U;
    uint32_t timeout = 0U;
    bool irqStatus   = false;

    /* Loop until irqStatus found to be 1 or if value of 'result' !=CDN_EOK */
    do {
        if (++timeout == delay) {
            result = (uint32_t)CDN_EIO;
            break;
        }
        CPS_DelayNs(LPDDR4_CPS_NS_DELAY_TIME);
        result = LPDDR4_CheckCtlInterrupt(pD, irqBit, &irqStatus);
    } while ((irqStatus == (bool)false) && (result == (uint32_t)CDN_EOK));

    return result;
}

/**
 * Internal Function:Poll for status of interrupt received by the PHY Independent Module.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] irqBit Interrupt status bit to be checked.
 * @param[in] delay time delay.
 * @return CDN_EOK on success (Interrupt status high).
 * @return CDN_EIO on poll time out.
 * @return CDN_EINVAL checking status was not successful.
 */
static uint32_t LPDDR4_PollPhyIndepIrq(const LPDDR4_PrivateData* pD, LPDDR4_INTR_PhyIndepInterrupt irqBit,  uint32_t delay) {

    uint32_t result = 0U;
    uint32_t timeout = 0U;
    bool irqStatus   = false;

    /* Loop until irqStatus found to be 1 or if value of 'result' !=CDN_EOK */
    do {
        if (++timeout == delay) {
            result = (uint32_t)CDN_EIO;
            break;
        }
        CPS_DelayNs(LPDDR4_CPS_NS_DELAY_TIME);
        result = LPDDR4_CheckPhyIndepInterrupt(pD, irqBit, &irqStatus);
    } while ((irqStatus == (bool)false) && (result == (uint32_t)CDN_EOK));

    return result;
}

/**
 * Internal Function:Trigger function to poll and Ack IRQs
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK on success (Interrupt status high).
 * @return CDN_EIO on poll time out.
 * @return CDN_EINVAL checking status was not successful.
 */
static uint32_t LPDDR4_PollAndAckIrq(const LPDDR4_PrivateData* pD){
    uint32_t result = 0U;

    /* Wait for PhyIndependent module to finish up ctl init sequence */
    result = LPDDR4_PollPhyIndepIrq(pD,LPDDR4_INTR_PHY_INDEP_INIT_DONE_BIT, LPDDR4_CUSTOM_TIMEOUT_DELAY);

    /* Ack to clear the PhyIndependent interrupt bit */
    if (result == (uint32_t)CDN_EOK) {
        result = LPDDR4_AckPhyIndepInterrupt(pD,LPDDR4_INTR_PHY_INDEP_INIT_DONE_BIT);
    }
    /* Wait for the CTL end of initialization */
    if (result == (uint32_t)CDN_EOK) {
        result = LPDDR4_PollCtlIrq(pD,LPDDR4_INTR_MC_INIT_DONE, LPDDR4_CUSTOM_TIMEOUT_DELAY);
    }
    /* Ack to clear the Ctl interrupt bit */
    if (result == (uint32_t)CDN_EOK) {
        result = LPDDR4_AckCtlInterrupt(pD,LPDDR4_INTR_MC_INIT_DONE);
    }
    return result;
}

/**
 * Internal Function: Controller start sequence.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK on success.
 * @return CDN_EINVAL starting controller was not successful.
 */
static uint32_t LPDDR4_StartSequenceController(const LPDDR4_PrivateData* pD)
{
    uint32_t result = 0U;
    uint32_t regVal = 0U;
    LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
    LPDDR4_InfoType infoType;

    /* Set the PI_start to initiate leveling procedure */
    regVal = CPS_FLD_SET(LPDDR4__PI_START__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__PI_START__REG)));
    CPS_REG_WRITE((&(ctlRegBase->LPDDR4__PI_START__REG)), regVal);

    /* Set the Ctl_start  */
    regVal = CPS_FLD_SET(LPDDR4__START__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__START__REG)));
    CPS_REG_WRITE(&(ctlRegBase->LPDDR4__START__REG), regVal);

    if (pD->infoHandler != (LPDDR4_InfoCallback)NULL) {
        /* If a handler is registered, call it with the relevant information type */
        infoType = LPDDR4_DRV_SOC_PLL_UPDATE;
        pD->infoHandler(pD, infoType);
    }

    result = LPDDR4_PollAndAckIrq(pD);

    return result;
}

/**
 * Internal Function: To add the offset to given address.
 * @param[in] addr Address to which the offset has to be added.
 * @param[in] regOffset The offset
 * @return regAddr The address value after the summation.
 */
volatile uint32_t* LPDDR4_AddOffset(volatile uint32_t* addr, uint32_t regOffset) {

    volatile uint32_t* local_addr = addr;
    /* Declaring as array to add the offset value.*/
    volatile uint32_t* regAddr = &local_addr[regOffset];
    return regAddr;
}

/**
 * Checks configuration object.
 * @param[in] config Driver/hardware configuration required.
 * @param[out] configSize Size of memory allocations required.
 * @return CDN_EOK on success (requirements structure filled).
 * @return CDN_ENOTSUP if configuration cannot be supported due to driver/hardware constraints.
 */
uint32_t LPDDR4_Probe(const LPDDR4_Config* config, uint16_t* configSize)
{
    uint32_t result;

    result = (uint32_t)(LPDDR4_ProbeSF(config, configSize));
    if ( result == (uint32_t)CDN_EOK ) {
        *configSize = (uint16_t)(sizeof(LPDDR4_PrivateData));
    }
    return result;
}

/**
 * Init function to be called after LPDDR4_probe() to set up the driver configuration.
 * Memory should be allocated for drv_data (using the size determined using LPDDR4_probe) before
 * calling  this API, init_settings should be initialized with base addresses for PHY Independent Module,
 * Controller and PHY before calling this function.
 * If callbacks are required for interrupt handling, these should also be configured in init_settings.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] cfg Specifies driver/hardware configuration.
 * @return CDN_EOK on success
 * @return CDN_EINVAL if illegal/inconsistent values in cfg.
 * @return CDN_ENOTSUP if hardware has an inconsistent configuration or doesn't support feature(s)
 * required by 'config' parameters.
 */
uint32_t LPDDR4_Init(LPDDR4_PrivateData* pD, const LPDDR4_Config* cfg)
{
    uint32_t result = 0U;

    result = LPDDR4_InitSF(pD, cfg);
    if ( result == (uint32_t)CDN_EOK ) {
        /* Validate Magic number */
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)cfg->ctlBase;
        /* Populating configuration data to pD */
        pD->ctlBase      = ctlRegBase;
        pD->infoHandler = (LPDDR4_InfoCallback)cfg->infoHandler;
        pD->ctlInterruptHandler = (LPDDR4_CtlCallback)cfg->ctlInterruptHandler;
        pD->phyIndepInterruptHandler = (LPDDR4_PhyIndepCallback)cfg->phyIndepInterruptHandler;
    }
    return result;
}

/**
 * Start the driver.
 * @param[in] pD Driver state info specific to this instance.
 */
uint32_t LPDDR4_Start(const LPDDR4_PrivateData* pD)
{
    uint32_t result = 0U;

    result = LPDDR4_StartSF(pD);
    if (result == (uint32_t)CDN_EOK) {
        /* Enable PI as the initiator for DRAM */
        result = LPDDR4_EnablePIInitiator(pD);
        /* Start PI init sequence. */
        result = LPDDR4_StartSequenceController(pD);
    }
    return result;
}

/**
 * Read a register from the controller, PHY or PHY Independent Module
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] cpp Indicates whether controller, PHY or PHY Independent Module register
 * @param[in] regOffset Register offset
 * @param[out] regValue Register value read
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if regOffset if out of range or regValue is NULL
 */
uint32_t LPDDR4_ReadReg(const LPDDR4_PrivateData* pD, LPDDR4_RegBlock cpp, uint32_t regOffset, uint32_t* regValue)
{
    uint32_t result = 0U;

    result = LPDDR4_ReadRegSF(pD, cpp, regValue);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        if (cpp == LPDDR4_CTL_REGS) {
            if (regOffset >= LPDDR4_INTR_CTL_REG_COUNT) {
                /* Return if user provider invalid register number */
                result = (uint32_t)CDN_EINVAL;
            } else {
                *regValue = CPS_REG_READ(LPDDR4_AddOffset(&(ctlRegBase->DENALI_CTL_0), regOffset));
            }
        } else if (cpp == LPDDR4_PHY_REGS) {
            if (regOffset >= LPDDR4_INTR_PHY_REG_COUNT) {
                /* Return if user provider invalid register number */
                result = (uint32_t)CDN_EINVAL;
            } else {
                *regValue = CPS_REG_READ(LPDDR4_AddOffset(&(ctlRegBase->DENALI_PHY_0), regOffset));
            }

        } else {
            if (regOffset >= LPDDR4_INTR_PHY_INDEP_REG_COUNT) {
                /* Return if user provider invalid register number */
                result = (uint32_t)CDN_EINVAL;
            } else {
                *regValue = CPS_REG_READ(LPDDR4_AddOffset(&(ctlRegBase->DENALI_PI_0), regOffset));
            }
        }
    }
    return result;
}

/**
 * Internal Function: To fetch the read-write mask for the phy registers.
 * @param[in] regOffset The offset
 * @return the read-write mask for the corresponding offset.
 */
static uint32_t LPDDR4_getPhyRwMask( uint32_t regOffset ){

    uint32_t rwMask      = 0U;
    uint32_t arrayOffset = 0U;
    uint32_t sliceNum, sliceOffset = 0U;

    /* Iterate to figure out the slice for the given offset */
    for (sliceNum = (uint32_t)0U; sliceNum <= (DSLICE_NUM + ASLICE_NUM); sliceNum++) {
        sliceOffset = sliceOffset + (uint32_t)SLICE_WIDTH;
        if (regOffset < sliceOffset) {
            break;
        }
    }
    arrayOffset = regOffset - (sliceOffset - (uint32_t)SLICE_WIDTH);

    /* Fetch the read-write mask from the respective mask array */
    if (sliceNum < DSLICE_NUM) {
        rwMask = LPDDR4_getDSliceMask( sliceNum, arrayOffset );
    } else {
        if (sliceNum == DSLICE_NUM) {
            if (arrayOffset < ASLICE0_REG_COUNT) {
                rwMask = g_lpddr4_address_slice_0_rw_mask[arrayOffset];
            }
        } else {
            if (arrayOffset < PHY_CORE_REG_COUNT) {
                /* Since the regOffset is valid, should be phy_core */
                rwMask = g_lpddr4_phy_core_rw_mask[arrayOffset];
            }
        }
    }
    return rwMask;
}

#ifdef REG_WRITE_VERIF
/**
 * Internal Function: To verify register writes.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] regOffset The offset
 * @param[in] regValue The value written to the register
 * @return CDN_EOK on success
 * @return CDN_EIO if value read looks different than the written.
 * @return CDN_EINVAL if illegal/inconsistent values in cfg.
 */
static uint32_t LPDDR4_VerifyRegWrite( const LPDDR4_PrivateData* pD,LPDDR4_RegBlock cpp, uint32_t regOffset, uint32_t regValue) {

    uint32_t result     = (uint32_t) CDN_EOK;
    uint32_t regReadVal = 0U;
    uint32_t rwMask     = 0U;

    /* Read the concern register to compare */
    result = LPDDR4_ReadReg(pD, cpp, regOffset, &regReadVal);

    if (result == (uint32_t)CDN_EOK) {
        /* Selecting the appropriate mask for the given register block*/
        switch (cpp) {
        case LPDDR4_PHY_INDEP_REGS:
            rwMask = g_lpddr4_pi_rw_mask[regOffset];
            break;
        case LPDDR4_PHY_REGS:
            rwMask = LPDDR4_getPhyRwMask(regOffset);
            break;
        default:
            /* LPDDR4_CTL_REGS is considered as the default (sanity check already confirmed it as valid cpp) */
            rwMask = g_lpddr4_ddr_controller_rw_mask[regOffset];
            break;
        }

        /* Compare the read and written values. */
        if ((rwMask & regReadVal) != (regValue & rwMask)) {
            result = CDN_EIO;
        }
    }
    return result;
}
#endif

uint32_t LPDDR4_DeferredRegVerify(const LPDDR4_PrivateData* pD, LPDDR4_RegBlock cpp, uint32_t regValues[], uint16_t regNum[], uint16_t regCount) {
    uint32_t result = (uint32_t) CDN_EOK;
    uint32_t aIndex;
    uint32_t regReadVal = 0U;
    uint32_t rwMask     = 0U;

    result = LPDDR4_DeferredRegVerifySF(pD, cpp);

    if ((regValues == (uint32_t *)NULL) || (regNum == (uint16_t *)NULL)) {
        result = CDN_EINVAL;
    }
    if (result == (uint32_t)CDN_EOK) {
        /* Iterate through register numbers that user prefer to read.*/
        for (aIndex = 0; aIndex < regCount; aIndex++) {

            /* Read the concern register to compare */
            result = LPDDR4_ReadReg(pD, cpp, (uint32_t)regNum[aIndex], &regReadVal);

            if (result == (uint32_t)CDN_EOK) {
                /* Selecting the appropriate mask for the given register block*/
                switch (cpp) {
                case LPDDR4_PHY_INDEP_REGS:
                    rwMask = g_lpddr4_pi_rw_mask[(uint32_t)regNum[aIndex]];
                    break;
                case LPDDR4_PHY_REGS:
                    rwMask = LPDDR4_getPhyRwMask((uint32_t)regNum[aIndex]);
                    break;
                default:
                    /* LPDDR4_CTL_REGS is considered as the default (sanity check already confirmed it as valid cpp) */
                    rwMask = g_lpddr4_ddr_controller_rw_mask[(uint32_t)regNum[aIndex]];
                    break;
                }

                /* Compare the read and written values. */
                if ((rwMask & regReadVal) != ((uint32_t)(regValues[aIndex]) & rwMask)) {
                    result = CDN_EIO;
                    break;
                }
            }
        }
    }
    return result;
}

uint32_t LPDDR4_WriteReg(const LPDDR4_PrivateData* pD, LPDDR4_RegBlock cpp, uint32_t regOffset, uint32_t regValue)
{
    uint32_t result = 0U;

    result = LPDDR4_WriteRegSF(pD, cpp);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        if (cpp == LPDDR4_CTL_REGS) {
            if (regOffset >= LPDDR4_INTR_CTL_REG_COUNT) {
                /* Return if user provider invalid register number */
                result = (uint32_t)CDN_EINVAL;
            } else {
                CPS_REG_WRITE(LPDDR4_AddOffset(&(ctlRegBase->DENALI_CTL_0), regOffset), regValue);
            }
        } else if (cpp == LPDDR4_PHY_REGS) {
            if (regOffset >= LPDDR4_INTR_PHY_REG_COUNT) {
                /* Return if user provider invalid register number */
                result = (uint32_t)CDN_EINVAL;
            } else {
                CPS_REG_WRITE(LPDDR4_AddOffset(&(ctlRegBase->DENALI_PHY_0), regOffset), regValue);
            }
        } else {
            if (regOffset >= LPDDR4_INTR_PHY_INDEP_REG_COUNT) {
                /* Return if user provider invalid register number */
                result = (uint32_t)CDN_EINVAL;
            } else {
                CPS_REG_WRITE(LPDDR4_AddOffset(&(ctlRegBase->DENALI_PI_0), regOffset), regValue);
            }
        }
    }
#ifdef REG_WRITE_VERIF
    if (result == (uint32_t)CDN_EOK) {
        /* Call the verify function to confirm the register write */
        result = LPDDR4_VerifyRegWrite(pD, cpp, regOffset, regValue);
    }
#endif

    return result;
}

uint32_t LPDDR4_GetMmrRegister(const LPDDR4_PrivateData* pD, uint32_t readModeRegVal, uint64_t* mmrValue, uint8_t* mmrStatus) {

    uint32_t result   = 0U;
    uint32_t tDelay   = 1000U;
    uint32_t regVal   = 0U;

    result = LPDDR4_GetMmrRegisterSF(pD, mmrValue, mmrStatus);
    if (result == (uint32_t)CDN_EOK) {

        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        /* Populate the calculated value to the register  */
        regVal = CPS_FLD_WRITE(LPDDR4__READ_MODEREG__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__READ_MODEREG__REG)), readModeRegVal);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__READ_MODEREG__REG), regVal);

        /* Wait until the Read is done */
        result = LPDDR4_PollCtlIrq(pD, LPDDR4_INTR_MR_READ_DONE, tDelay);
    }
    if (result == (uint32_t)CDN_EOK) {
        result = LPDDR4_CheckMmrReadError(pD, mmrValue, mmrStatus);
    }
    return result;
}

static uint32_t LPDDR4_WriteMmrRegister(const LPDDR4_PrivateData* pD, uint32_t writeModeRegVal) {

    uint32_t result = (uint32_t) CDN_EOK;
    uint32_t tDelay = 1000U;
    uint32_t regVal = 0U;
    LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

    /* Populate the calculated value to the register  */
    regVal = CPS_FLD_WRITE(LPDDR4__WRITE_MODEREG__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__WRITE_MODEREG__REG)), writeModeRegVal);
    CPS_REG_WRITE(&(ctlRegBase->LPDDR4__WRITE_MODEREG__REG), regVal);

    result = LPDDR4_PollCtlIrq(pD, LPDDR4_INTR_MR_WRITE_DONE, tDelay);

    return result;
}

uint32_t LPDDR4_SetMmrRegister(const LPDDR4_PrivateData* pD, uint32_t writeModeRegVal, uint8_t* mrwStatus) {
    uint32_t result = 0U;

    result = LPDDR4_SetMmrRegisterSF(pD, mrwStatus);
    if (result == (uint32_t)CDN_EOK) {

        /* Function call to trigger Mode register write */
        result = LPDDR4_WriteMmrRegister(pD, writeModeRegVal);

        if (result == (uint32_t)CDN_EOK) {
            result = LPDDR4_AckCtlInterrupt(pD, LPDDR4_INTR_MR_WRITE_DONE);
        }
        /* Read the status of mode register write */
        if (result == (uint32_t)CDN_EOK) {
            LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
            *mrwStatus = (uint8_t)CPS_FLD_READ(LPDDR4__MRW_STATUS__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__MRW_STATUS__REG)));
            if ((*mrwStatus) != 0U) {
                result = (uint32_t)CDN_EIO;
            }
        }
    }

#ifdef ASILC
/* Read mode register and confirm the written value (part of ASIL-C) */
#endif

    return result;
}

uint32_t LPDDR4_WriteCtlConfig(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount) {
    uint32_t result;
    uint32_t aIndex;

    result = LPDDR4_WriteCtlConfigSF(pD );
    if ( (regValues == (uint32_t *)NULL) || (regNum == (uint16_t *)NULL )) {
        result = CDN_EINVAL;
    }

    if (result == (uint32_t)CDN_EOK) {
        /* Iterate through controller register numbers that user prefer to update.*/
        for (aIndex = 0; aIndex < regCount; aIndex++) {
            /* Calling the write API with the user provided data */
            result =  (uint32_t)LPDDR4_WriteReg(pD, LPDDR4_CTL_REGS, (uint32_t)regNum[aIndex],
                                                (uint32_t)regValues[aIndex]);
        }
    }
    return result;
}

uint32_t LPDDR4_WritePhyIndepConfig(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount) {
    uint32_t result;
    uint32_t aIndex;

    result = LPDDR4_WritePhyIndepConfigSF(pD);
    if ( (regValues == (uint32_t *)NULL) || (regNum == (uint16_t *)NULL) ) {
        result = CDN_EINVAL;
    }
    if (result == (uint32_t)CDN_EOK) {
        /* Iterate through PHY Independent module  register numbers that user prefer to update.*/
        for (aIndex = 0; aIndex < regCount; aIndex++) {
            /* Calling the write API with the user provided data */
            result =  (uint32_t)LPDDR4_WriteReg(pD, LPDDR4_PHY_INDEP_REGS, (uint32_t)regNum[aIndex],
                                                (uint32_t)regValues[aIndex]);
        }
    }
    return result;
}

uint32_t LPDDR4_WritePhyConfig(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount) {
    uint32_t result;
    uint32_t aIndex;

    result = LPDDR4_WritePhyConfigSF(pD);
    if ( (regValues == (uint32_t *)NULL) || ( regNum == (uint16_t *)NULL )) {
        result = CDN_EINVAL;
    }
    if (result == (uint32_t)CDN_EOK) {
        /* Iterate through PHY  register numbers that user prefer to update.*/
        for (aIndex = 0; aIndex < regCount; aIndex++) {
            /* Calling the write API with the user provided data */
            result =  (uint32_t)LPDDR4_WriteReg(pD, LPDDR4_PHY_REGS, (uint32_t)regNum[aIndex],
                                                (uint32_t)regValues[aIndex]);
        }
    }
    return result;
}

uint32_t LPDDR4_ReadCtlConfig(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount) {
    uint32_t result;
    uint32_t aIndex;

    result = LPDDR4_ReadCtlConfigSF(pD);
    if ( (regValues == (uint32_t *)NULL) || (regNum == (uint16_t *)NULL) ) {
        result = CDN_EINVAL;
    }
    if (result == (uint32_t)CDN_EOK) {
        /* Iterate through CTL register numbers that user prefer to read.*/
        for (aIndex = 0; aIndex < regCount; aIndex++) {
            /* Calling the read API with the user provided data */
            result =  (uint32_t)LPDDR4_ReadReg(pD, LPDDR4_CTL_REGS, (uint32_t)regNum[aIndex],
                                               (uint32_t*)(&regValues[aIndex]));
        }
    }
    return result;
}

uint32_t LPDDR4_ReadPhyIndepConfig(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount) {
    uint32_t result;
    uint32_t aIndex;

    result = LPDDR4_ReadPhyIndepConfigSF(pD);
    if ( (regValues == (uint32_t *)NULL) || (regNum == (uint16_t *)NULL) ) {
        result = CDN_EINVAL;
    }
    if (result == (uint32_t)CDN_EOK) {
        /* Iterate through PHY  register numbers that user prefer to read.*/
        for (aIndex = 0; aIndex < regCount; aIndex++) {
            /* Calling the read API with the user provided data */
            result =  (uint32_t)LPDDR4_ReadReg(pD, LPDDR4_PHY_INDEP_REGS, (uint32_t)regNum[aIndex],
                                               (uint32_t*)(&regValues[aIndex]));
        }
    }
    return result;
}

uint32_t LPDDR4_ReadPhyConfig(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount) {
    uint32_t result;
    uint32_t aIndex;

    result = LPDDR4_ReadPhyConfigSF(pD);
    if ( (regValues == (uint32_t *)NULL) || (regNum == (uint16_t *)NULL )) {
        result = CDN_EINVAL;
    }
    if (result == (uint32_t)CDN_EOK) {
        /* Iterate through PHY  register numbers that user prefer to read.*/
        for (aIndex = 0; aIndex < regCount; aIndex++) {
            /* Calling the read API with the user provided data */
            result =  (uint32_t)LPDDR4_ReadReg(pD, LPDDR4_PHY_REGS, (uint32_t)regNum[aIndex],
                                               (uint32_t*)(&regValues[aIndex]));
        }
    }
    return result;
}

uint32_t LPDDR4_GetPhyIndepInterruptMask(const LPDDR4_PrivateData* pD, uint32_t* mask) {
    uint32_t result;

    result = LPDDR4_GetPhyIndepInterruptMSF(pD,  mask);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        /* Reading mask register */
        *mask = CPS_FLD_READ(LPDDR4__PI_INT_MASK__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__PI_INT_MASK__REG)));
    }
    return result;
}

uint32_t LPDDR4_SetPhyIndepInterruptMask(const LPDDR4_PrivateData* pD, const uint32_t* mask) {
    uint32_t result;
    uint32_t regVal = 0;
    const uint32_t ui32IrqCount = (uint32_t)LPDDR4_INTR_PHY_INDEP_DLL_LOCK_STATE_CHANGE_BIT + 1U;

    result = LPDDR4_SetPhyIndepInterruptMSF(pD, mask);
    if ((result == (uint32_t)CDN_EOK) && (ui32IrqCount < WORD_SHIFT)) {
        /* Return if the user given value is higher than the field width */
        if (*mask >= (1U << ui32IrqCount)) {
            result = (uint32_t)CDN_EINVAL;
        }
    }
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        /* Writing to the user requested interrupt mask */
        regVal = CPS_FLD_WRITE(LPDDR4__PI_INT_MASK__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__PI_INT_MASK__REG)), *mask);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__PI_INT_MASK__REG), regVal);
    }
    return result;
}

uint32_t LPDDR4_CheckPhyIndepInterrupt(const LPDDR4_PrivateData* pD, LPDDR4_INTR_PhyIndepInterrupt intr, bool* irqStatus) {
    uint32_t result = 0;
    uint32_t phyIndepIrqStatus = 0;

    result = LPDDR4_INTR_CheckPhyIndepIntSF(pD, intr, irqStatus);
    /* Confirming that the value of interrupt is less than register width - MISRA2012-RULE-12_2*/
    if ((result == (uint32_t)CDN_EOK) && ((uint32_t)intr < WORD_SHIFT)) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        /* Reading the requested bit to check interrupt status */
        phyIndepIrqStatus = CPS_REG_READ(&(ctlRegBase->LPDDR4__PI_INT_STATUS__REG));
        *irqStatus = (bool)(((phyIndepIrqStatus >> (uint32_t)intr ) & BIT_MASK) > 0U);
    }
    return result;
}

uint32_t LPDDR4_AckPhyIndepInterrupt(const LPDDR4_PrivateData* pD, LPDDR4_INTR_PhyIndepInterrupt intr) {
    uint32_t result = 0U;
    uint32_t regVal = 0U;

    result = LPDDR4_INTR_AckPhyIndepIntSF(pD, intr);
    /* Confirming that the value of interrupt is less than register width */
    if ((result == (uint32_t)CDN_EOK) && ((uint32_t)intr < WORD_SHIFT)) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        /* Write 1 to the requested bit to ACk the interrupt */
        regVal = ((uint32_t)BIT_MASK << (uint32_t)intr);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__PI_INT_ACK__REG), regVal);
    }

    return result;
}

/* Check for caTrainingError */
static void LPDDR4_CheckCaTrainingError(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_DebugInfo* debugInfo, bool* errFoundPtr) {

    uint32_t regVal;
    uint32_t errBitMask = 0U;
    uint32_t snum;
    volatile uint32_t* regAddress;

    regAddress = (volatile uint32_t*)(&(ctlRegBase->LPDDR4__PHY_ADR_CALVL_OBS1_0__REG));
    errBitMask = (CA_TRAIN_RL) | (NIBBLE_MASK);
    /* PHY_ADR_CALVL_OBS1[4] – Right found
       PHY_ADR_CALVL_OBS1[5] – left found
       Both the above fields should be high and below field should be zero.
       PHY_ADR_CALVL_OBS1[3:0] – calvl_state
     */
    for (snum = 0U; snum < ASLICE_NUM; snum++) {
        regVal = CPS_REG_READ(regAddress);
        if ((regVal & errBitMask) != CA_TRAIN_RL) {
            debugInfo->caTraingError = CDN_TRUE;
            *errFoundPtr = true;
        }
        regAddress = LPDDR4_AddOffset(regAddress, (uint32_t)SLICE_WIDTH);
    }
}

/* Check for  GateLvlError */
static void LPDDR4_CheckGateLvlError(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_DebugInfo* debugInfo, bool* errFoundPtr) {

    uint32_t regVal;
    uint32_t errBitMask = 0U;
    uint32_t snum;
    volatile uint32_t* regAddress;

    regAddress = (volatile uint32_t*)(&(ctlRegBase->LPDDR4__PHY_GTLVL_STATUS_OBS_0__REG));
    /* PHY_GTLVL_STATUS_OBS[6] – gate_level min error
     * PHY_GTLVL_STATUS_OBS[7] – gate_level max error
     * All the above bit fields should be zero */
    errBitMask = GATE_LVL_ERROR_FIELDS;
    for (snum = (uint32_t)0U; snum < DSLICE_NUM; snum++) {
        regVal = CPS_REG_READ(regAddress);
        if ((regVal & errBitMask) != 0U) {
            debugInfo->gateLvlError = CDN_TRUE;
            *errFoundPtr = true;
        }
        regAddress = LPDDR4_AddOffset(regAddress, (uint32_t)SLICE_WIDTH);
    }
}

/* Check for  ReadLvlError */
static void LPDDR4_CheckReadLvlError(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_DebugInfo* debugInfo, bool* errFoundPtr) {

    uint32_t regVal;
    uint32_t errBitMask = 0U;
    uint32_t snum;
    volatile uint32_t* regAddress;

    regAddress = (volatile uint32_t*)(&(ctlRegBase->LPDDR4__PHY_RDLVL_STATUS_OBS_0__REG));
    /* PHY_RDLVL_STATUS_OBS[23:16] – failed bits : should be zero.
       PHY_RDLVL_STATUS_OBS[31:28] – rdlvl_state : should be zero */
    errBitMask = READ_LVL_ERROR_FIELDS;
    for (snum = (uint32_t)0U; snum < DSLICE_NUM; snum++) {
        regVal = CPS_REG_READ(regAddress);
        if ((regVal & errBitMask) != 0U) {
            debugInfo->readLvlError = CDN_TRUE;
            *errFoundPtr = true;
        }
        regAddress = LPDDR4_AddOffset(regAddress, (uint32_t)SLICE_WIDTH);
    }
}

/* Check for  DqTrainingError */
static void LPDDR4_CheckDqTrainingError(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_DebugInfo* debugInfo, bool* errFoundPtr) {

    uint32_t regVal;
    uint32_t errBitMask = 0U;
    uint32_t snum;
    volatile uint32_t* regAddress;

    regAddress = (volatile uint32_t*)(&(ctlRegBase->LPDDR4__PHY_WDQLVL_STATUS_OBS_0__REG));
    /* PHY_WDQLVL_STATUS_OBS[26:18] should all be zero. */
    errBitMask = DQ_LVL_STATUS;
    for (snum = (uint32_t)0U; snum < DSLICE_NUM; snum++) {
        regVal = CPS_REG_READ(regAddress);
        if ((regVal & errBitMask) != 0U) {
            debugInfo->dqTrainingError = CDN_TRUE;
            *errFoundPtr = true;
        }
        regAddress = LPDDR4_AddOffset(regAddress, (uint32_t)SLICE_WIDTH);
    }
}

/**
 * Internal Function:For checking errors in training/levelling sequence.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] debugInfo pointer to debug information.
 * @param[out] errFoundPtr pointer to return if error found.
 * @return CDN_EOK on success (Interrupt status high).
 * @return CDN_EINVAL checking or unmasking was not successful.
 */
bool LPDDR4_CheckLvlErrors(const LPDDR4_PrivateData* pD, LPDDR4_DebugInfo* debugInfo, bool errFound) {

    bool localErrFound = errFound;

    LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

    if (localErrFound == (bool)false) {
        /* Check for ca training error */
        LPDDR4_CheckCaTrainingError(ctlRegBase, debugInfo, &localErrFound);
    }

    if (localErrFound == (bool)false) {
        /* Check for Write leveling error */
        LPDDR4_CheckWrLvlError(ctlRegBase, debugInfo, &localErrFound);
    }

    if (localErrFound == (bool)false) {
        /* Check for Gate leveling error */
        LPDDR4_CheckGateLvlError(ctlRegBase, debugInfo, &localErrFound);
    }

    if (localErrFound == (bool)false) {
        /* Check for Read leveling error */
        LPDDR4_CheckReadLvlError(ctlRegBase, debugInfo, &localErrFound);
    }

    if (localErrFound == (bool)false) {
        /* Check for DQ training error */
        LPDDR4_CheckDqTrainingError(ctlRegBase, debugInfo, &localErrFound);
    }
    return localErrFound;
}

static uint8_t LPDDR4_SetError(volatile uint32_t* reg, uint32_t errBitMask, uint8_t* errFoundPtr, const uint32_t errorInfoBits) {

    uint32_t regVal = 0U;

    /* Read the respective observation register */
    regVal = CPS_REG_READ(reg);
    /* Compare the error bit values */
    if ((regVal & errBitMask) != errorInfoBits) {
        *errFoundPtr = CDN_TRUE;
    }
    return *errFoundPtr;
}

void LPDDR4_SetErrors(LPDDR4_CtlRegs* ctlRegBase,  LPDDR4_DebugInfo* debugInfo, uint8_t* errFoundPtr){

    uint32_t errBitMask = (BIT_MASK << 0x1U) | (BIT_MASK);
    /* Check PLL observation registers for PLL lock errors */

    debugInfo->pllError = LPDDR4_SetError(&(ctlRegBase->LPDDR4__PHY_PLL_OBS_0__REG),
                                          errBitMask, errFoundPtr, PLL_READY);
    if (*errFoundPtr == CDN_FALSE) {
        debugInfo->pllError = LPDDR4_SetError(&(ctlRegBase->LPDDR4__PHY_PLL_OBS_1__REG),
                                              errBitMask, errFoundPtr, PLL_READY);
    }

    /* Check for IO Calibration errors */
    if (*errFoundPtr == CDN_FALSE) {
        debugInfo->ioCalibError = LPDDR4_SetError(&(ctlRegBase->LPDDR4__PHY_CAL_RESULT_OBS_0__REG),
                                                  IO_CALIB_DONE, errFoundPtr, IO_CALIB_DONE);
    }
    /* Check for IO Calibration errors */
    if (*errFoundPtr == CDN_FALSE) {
        debugInfo->ioCalibError = LPDDR4_SetError(&(ctlRegBase->LPDDR4__PHY_CAL_RESULT2_OBS_0__REG),
                                                  IO_CALIB_DONE, errFoundPtr, IO_CALIB_DONE);
    }
    /* Check for IO Calibration errors */
    if (*errFoundPtr == CDN_FALSE) {
        debugInfo->ioCalibError = LPDDR4_SetError(&(ctlRegBase->LPDDR4__PHY_CAL_RESULT3_OBS_0__REG),
                                                  IO_CALIB_FIELD, errFoundPtr, IO_CALIB_STATE);
    }
}

static void LPDDR4_SetPhySnapSettings(LPDDR4_CtlRegs* ctlRegBase, const bool errorFound) {

    uint32_t snum = 0U;
    volatile uint32_t* regAddress;
    uint32_t regVal = 0U;

    /* Setting SC_PHY_SNAP_OBS_REGS_x to get a snapshot*/
    if (errorFound == (bool)false) {
        regAddress = (volatile uint32_t*)(&(ctlRegBase->LPDDR4__SC_PHY_SNAP_OBS_REGS_0__REG));
        /* Iterate through each PHY Data Slice */
        for (snum = (uint32_t)0U; snum < DSLICE_NUM; snum++) {
            regVal = CPS_FLD_SET(LPDDR4__SC_PHY_SNAP_OBS_REGS_0__FLD,CPS_REG_READ(regAddress));
            CPS_REG_WRITE(regAddress, regVal);
            regAddress = LPDDR4_AddOffset(regAddress, (uint32_t)SLICE_WIDTH);
        }
    }
}

static void LPDDR4_SetPhyAdrSnapSettings(LPDDR4_CtlRegs* ctlRegBase, const bool errorFound) {

    uint32_t snum = 0U;
    volatile uint32_t* regAddress;
    uint32_t regVal = 0U;

    /* Setting SC_PHY ADR_SNAP_OBS_REGS_x to get a snapshot*/
    if (errorFound == (bool)false) {
        regAddress = (volatile uint32_t*)(&(ctlRegBase->LPDDR4__SC_PHY_ADR_SNAP_OBS_REGS_0__REG));
        /* Iterate through each PHY Address Slice */
        for (snum = (uint32_t)0U; snum < ASLICE_NUM; snum++) {
            regVal = CPS_FLD_SET(LPDDR4__SC_PHY_ADR_SNAP_OBS_REGS_0__FLD,CPS_REG_READ(regAddress));
            CPS_REG_WRITE(regAddress, regVal);
            regAddress = LPDDR4_AddOffset(regAddress, (uint32_t)SLICE_WIDTH);
        }
    }
}

void LPDDR4_SetSettings(LPDDR4_CtlRegs* ctlRegBase, const bool errorFound) {

    /* Calling functions to enable snap shots of OBS registers */
    LPDDR4_SetPhySnapSettings(ctlRegBase, errorFound);
    LPDDR4_SetPhyAdrSnapSettings(ctlRegBase, errorFound);
}

static void readPDWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles) {

    /* Read the appropriate register, based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_PD_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_PD_WAKEUP_F0__REG)));
    } else if (*fspNum == LPDDR4_FSP_1) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_PD_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_PD_WAKEUP_F1__REG)));
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        *cycles = CPS_FLD_READ(LPDDR4__LPI_PD_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_PD_WAKEUP_F2__REG)));
    }
}

static void readSRShortWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles) {

    /* Read the appropriate register, based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SR_SHORT_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_SHORT_WAKEUP_F0__REG)));
    } else if (*fspNum == LPDDR4_FSP_1) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SR_SHORT_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_SHORT_WAKEUP_F1__REG)));
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SR_SHORT_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_SHORT_WAKEUP_F2__REG)));
    }
}

static void readSRLongWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles) {

    /* Read the appropriate register, based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SR_LONG_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_LONG_WAKEUP_F0__REG)));
    } else if (*fspNum == LPDDR4_FSP_1) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SR_LONG_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_LONG_WAKEUP_F1__REG)));
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SR_LONG_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_LONG_WAKEUP_F2__REG)));
    }
}

static void readSRLongGateWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles) {

    /* Read the appropriate register, based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F0__REG)));
    } else if (*fspNum == LPDDR4_FSP_1) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F1__REG)));
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F2__REG)));
    }
}

static void readSRDPShortWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles) {

    /* Read the appropriate register, based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SRPD_SHORT_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_SHORT_WAKEUP_F0__REG)));
    } else if (*fspNum == LPDDR4_FSP_1) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SRPD_SHORT_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_SHORT_WAKEUP_F1__REG)));
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SRPD_SHORT_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_SHORT_WAKEUP_F2__REG)));
    }
}

static void readSRDPLongWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles) {

    /* Read the appropriate register, based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SRPD_LONG_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_WAKEUP_F0__REG)));
    } else if (*fspNum == LPDDR4_FSP_1) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SRPD_LONG_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_WAKEUP_F1__REG)));
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SRPD_LONG_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_WAKEUP_F2__REG)));
    }
}

static void readSRDPLongGateWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, uint32_t* cycles) {

    /* Read the appropriate register, based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F0__REG)));
    } else if (*fspNum == LPDDR4_FSP_1) {
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F1__REG)));
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        *cycles = CPS_FLD_READ(LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F2__REG)));
    }

}

static void LPDDR4_ReadLpiWakeUpTime(LPDDR4_CtlRegs* ctlRegBase, const LPDDR4_LpiWakeUpParam* lpiWakeUpParam, const LPDDR4_CtlFspNum* fspNum, uint32_t* cycles) {

    /* Iterate through each of the Wake up parameter type */
    if (*lpiWakeUpParam == LPDDR4_LPI_PD_WAKEUP_FN) {
        /* Calling appropriate function for register read */
        readPDWakeUp(fspNum, ctlRegBase, cycles);
    } else if (*lpiWakeUpParam == LPDDR4_LPI_SR_SHORT_WAKEUP_FN) {
        readSRShortWakeUp(fspNum, ctlRegBase, cycles);
    } else if (*lpiWakeUpParam == LPDDR4_LPI_SR_LONG_WAKEUP_FN) {
        readSRLongWakeUp(fspNum, ctlRegBase, cycles);
    } else if (*lpiWakeUpParam == LPDDR4_LPI_SR_LONG_MCCLK_GATE_WAKEUP_FN) {
        readSRLongGateWakeUp(fspNum, ctlRegBase, cycles);
    } else if (*lpiWakeUpParam == LPDDR4_LPI_SRPD_SHORT_WAKEUP_FN) {
        readSRDPShortWakeUp(fspNum, ctlRegBase, cycles);
    } else if (*lpiWakeUpParam == LPDDR4_LPI_SRPD_LONG_WAKEUP_FN) {
        readSRDPLongWakeUp(fspNum, ctlRegBase, cycles);
    } else {
        /* Default function (sanity function already confirmed the variable value) */
        readSRDPLongGateWakeUp(fspNum, ctlRegBase, cycles);
    }
}

uint32_t LPDDR4_GetLpiWakeUpTime(const LPDDR4_PrivateData* pD, const LPDDR4_LpiWakeUpParam* lpiWakeUpParam, const LPDDR4_CtlFspNum* fspNum, uint32_t* cycles) {

    uint32_t result = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_GetLpiWakeUpTimeSF(pD, lpiWakeUpParam, fspNum, cycles);
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        LPDDR4_ReadLpiWakeUpTime(ctlRegBase, lpiWakeUpParam, fspNum, cycles);
    }
    return result;
}

static void writePDWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles) {

    uint32_t regVal = 0U;
    /* Write to appropriate register ,based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_PD_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_PD_WAKEUP_F0__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_PD_WAKEUP_F0__REG), regVal);
    } else if (*fspNum == LPDDR4_FSP_1) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_PD_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_PD_WAKEUP_F1__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_PD_WAKEUP_F1__REG), regVal);
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_PD_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_PD_WAKEUP_F2__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_PD_WAKEUP_F2__REG), regVal);
    }
}

static void writeSRShortWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles) {

    uint32_t regVal = 0U;
    /* Write to appropriate register ,based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SR_SHORT_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_SHORT_WAKEUP_F0__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SR_SHORT_WAKEUP_F0__REG), regVal);
    } else if (*fspNum == LPDDR4_FSP_1) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SR_SHORT_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_SHORT_WAKEUP_F1__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SR_SHORT_WAKEUP_F1__REG), regVal);
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SR_SHORT_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_SHORT_WAKEUP_F2__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SR_SHORT_WAKEUP_F2__REG), regVal);
    }
}

static void writeSRLongWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles) {

    uint32_t regVal = 0U;
    /* Write to appropriate register ,based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SR_LONG_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_LONG_WAKEUP_F0__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SR_LONG_WAKEUP_F0__REG), regVal);
    } else if (*fspNum == LPDDR4_FSP_1) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SR_LONG_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_LONG_WAKEUP_F1__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SR_LONG_WAKEUP_F1__REG), regVal);
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SR_LONG_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_LONG_WAKEUP_F2__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SR_LONG_WAKEUP_F2__REG), regVal);
    }
}

static void writeSRLongGateWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles) {

    uint32_t regVal = 0U;
    /* Write to appropriate register ,based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F0__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F0__REG), regVal);
    } else if (*fspNum == LPDDR4_FSP_1) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F1__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F1__REG), regVal);
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F2__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SR_LONG_MCCLK_GATE_WAKEUP_F2__REG), regVal);
    }
}

static void writeSRDPShortWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles) {

    uint32_t regVal = 0U;
    /* Write to appropriate register ,based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SRPD_SHORT_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_SHORT_WAKEUP_F0__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SRPD_SHORT_WAKEUP_F0__REG), regVal);
    } else if (*fspNum == LPDDR4_FSP_1) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SRPD_SHORT_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_SHORT_WAKEUP_F1__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SRPD_SHORT_WAKEUP_F1__REG), regVal);
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SRPD_SHORT_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_SHORT_WAKEUP_F2__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SRPD_SHORT_WAKEUP_F2__REG), regVal);
    }
}

static void writeSRDPLongWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles) {

    uint32_t regVal = 0U;
    /* Write to appropriate register ,based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SRPD_LONG_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_WAKEUP_F0__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_WAKEUP_F0__REG), regVal);
    } else if (*fspNum == LPDDR4_FSP_1) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SRPD_LONG_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_WAKEUP_F1__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_WAKEUP_F1__REG), regVal);
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SRPD_LONG_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_WAKEUP_F2__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_WAKEUP_F2__REG), regVal);
    }
}
static void writeSRDPLongGateWakeUp(const LPDDR4_CtlFspNum* fspNum, LPDDR4_CtlRegs* ctlRegBase, const uint32_t* cycles) {

    uint32_t regVal = 0U;
    /* Write to appropriate register ,based on user given frequency.*/
    if (*fspNum == LPDDR4_FSP_0) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F0__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F0__REG), regVal);
    } else if (*fspNum == LPDDR4_FSP_1) {
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F1__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F1__REG), regVal);
    } else {
        /* Default register (sanity function already confirmed the variable value) */
        regVal = CPS_FLD_WRITE(LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F2__REG)), *cycles);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_F2__REG), regVal);
    }
}

static void LPDDR4_WriteLpiWakeUpTime(LPDDR4_CtlRegs* ctlRegBase, const LPDDR4_LpiWakeUpParam* lpiWakeUpParam, const LPDDR4_CtlFspNum* fspNum, const uint32_t* cycles) {

    /* Iterate through each of the Wake up parameter type */
    if (*lpiWakeUpParam == LPDDR4_LPI_PD_WAKEUP_FN) {
        /* Calling appropriate function for register write */
        writePDWakeUp(fspNum, ctlRegBase, cycles);
    } else if (*lpiWakeUpParam == LPDDR4_LPI_SR_SHORT_WAKEUP_FN) {
        writeSRShortWakeUp(fspNum, ctlRegBase, cycles);
    } else if (*lpiWakeUpParam == LPDDR4_LPI_SR_LONG_WAKEUP_FN) {
        writeSRLongWakeUp(fspNum, ctlRegBase, cycles);
    } else if (*lpiWakeUpParam == LPDDR4_LPI_SR_LONG_MCCLK_GATE_WAKEUP_FN) {
        writeSRLongGateWakeUp(fspNum, ctlRegBase, cycles);
    } else if (*lpiWakeUpParam == LPDDR4_LPI_SRPD_SHORT_WAKEUP_FN) {
        writeSRDPShortWakeUp(fspNum, ctlRegBase, cycles);
    } else if (*lpiWakeUpParam == LPDDR4_LPI_SRPD_LONG_WAKEUP_FN) {
        writeSRDPLongWakeUp(fspNum, ctlRegBase, cycles);
    } else {
        /* Default function (sanity function already confirmed the variable value) */
        writeSRDPLongGateWakeUp(fspNum, ctlRegBase, cycles);
    }
}

uint32_t LPDDR4_SetLpiWakeUpTime(const LPDDR4_PrivateData* pD, const LPDDR4_LpiWakeUpParam* lpiWakeUpParam, const LPDDR4_CtlFspNum* fspNum, const uint32_t* cycles) {
    uint32_t result = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_SetLpiWakeUpTimeSF(pD, lpiWakeUpParam, fspNum, cycles);
    if (result == (uint32_t)CDN_EOK) {
        /* Return if the user given value is higher than the field width */
        if (*cycles > NIBBLE_MASK) {
            result = (uint32_t)CDN_EINVAL;
        }
    }
    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        LPDDR4_WriteLpiWakeUpTime(ctlRegBase, lpiWakeUpParam, fspNum, cycles);
    }
    return result;
}

uint32_t LPDDR4_GetDbiReadMode(const LPDDR4_PrivateData* pD, bool *on_off) {

    uint32_t result = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_GetDbiReadModeSF(pD, on_off);

    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        /* Reading the field value from the register.*/
        if (CPS_FLD_READ(LPDDR4__RD_DBI_EN__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__RD_DBI_EN__REG))) == 0U) {
            *on_off = false;
        } else {
            *on_off = true;
        }
    }
    return result;
}

uint32_t LPDDR4_GetDbiWriteMode(const LPDDR4_PrivateData* pD, bool *on_off) {

    uint32_t result = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_GetDbiReadModeSF(pD, on_off);

    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        /* Reading the field value from the register.*/
        if (CPS_FLD_READ(LPDDR4__WR_DBI_EN__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__WR_DBI_EN__REG))) == 0U) {
            *on_off = false;
        } else {
            *on_off = true;
        }
    }
    return result;
}

uint32_t LPDDR4_SetDbiMode(const LPDDR4_PrivateData* pD, const LPDDR4_DbiMode* mode) {

    uint32_t result = 0U;
    uint32_t regVal = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_SetDbiModeSF(pD, mode);

    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        /* Updating the appropriate field value based on the user given mode*/
        if (*mode == LPDDR4_DBI_RD_ON) {
            regVal = CPS_FLD_WRITE(LPDDR4__RD_DBI_EN__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__RD_DBI_EN__REG)), 1U);
        } else if (*mode == LPDDR4_DBI_RD_OFF) {
            regVal = CPS_FLD_WRITE(LPDDR4__RD_DBI_EN__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__RD_DBI_EN__REG)), 0U);
        } else if (*mode == LPDDR4_DBI_WR_ON) {
            regVal = CPS_FLD_WRITE(LPDDR4__WR_DBI_EN__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__WR_DBI_EN__REG)), 1U);
        } else {
            /* Default field (Sanity function already confirmed the value to be in expected range.)*/
            regVal = CPS_FLD_WRITE(LPDDR4__WR_DBI_EN__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__WR_DBI_EN__REG)), 0U);
        }
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__RD_DBI_EN__REG), regVal);
    }
    return result;
}

uint32_t LPDDR4_GetRefreshRate(const LPDDR4_PrivateData* pD, const LPDDR4_CtlFspNum* fspNum, uint32_t* tref, uint32_t* tras_max) {
    uint32_t result = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_GetRefreshRateSF(pD, fspNum, tref, tras_max);

    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

        /* Selecting the appropriate register for the user requested Frequency*/
        switch (*fspNum) {
        case LPDDR4_FSP_2:
            /* Reading the required values from the corresponding fields */
            *tref = CPS_FLD_READ(LPDDR4__TREF_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TREF_F2__REG)));
            *tras_max = CPS_FLD_READ(LPDDR4__TRAS_MAX_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TRAS_MAX_F2__REG)));
            break;
        case LPDDR4_FSP_1:
            /* Setting for FSP1 */
            *tref = CPS_FLD_READ(LPDDR4__TREF_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TREF_F1__REG)));
            *tras_max = CPS_FLD_READ(LPDDR4__TRAS_MAX_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TRAS_MAX_F1__REG)));
            break;
        default:
            /* FSP_0 is considered as default due to MISRA Rule */
            *tref = CPS_FLD_READ(LPDDR4__TREF_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TREF_F0__REG)));
            *tras_max = CPS_FLD_READ(LPDDR4__TRAS_MAX_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TRAS_MAX_F0__REG)));
            break;
        }
    }
    return result;
}

/* Helper function to update TREF and TRAS MAX parameters for FSP 2*/
static void LPDDR4_UpdateFSP2RefRateParams(const LPDDR4_PrivateData* pD, const uint32_t* tref, const uint32_t* tras_max) {
    uint32_t regVal = 0U;
    LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

    /* Update tref parameter */
    regVal = CPS_FLD_WRITE(LPDDR4__TREF_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TREF_F2__REG)), *tref);
    CPS_REG_WRITE(&(ctlRegBase->LPDDR4__TREF_F2__REG), regVal);
    /* Update tRAS MAX parameter */
    regVal = CPS_FLD_WRITE(LPDDR4__TRAS_MAX_F2__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TRAS_MAX_F2__REG)), *tras_max);
    CPS_REG_WRITE(&(ctlRegBase->LPDDR4__TRAS_MAX_F2__REG), regVal);

}

/* Helper function to update TREF and TRAS MAX parameters for FSP 1*/
static void LPDDR4_UpdateFSP1RefRateParams(const LPDDR4_PrivateData* pD, const uint32_t* tref, const uint32_t* tras_max) {
    uint32_t regVal = 0U;
    LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

    /* Update tref parameter */
    regVal = CPS_FLD_WRITE(LPDDR4__TREF_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TREF_F1__REG)), *tref);
    CPS_REG_WRITE(&(ctlRegBase->LPDDR4__TREF_F1__REG), regVal);
    /* Update tRAS MAX parameter */
    regVal = CPS_FLD_WRITE(LPDDR4__TRAS_MAX_F1__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TRAS_MAX_F1__REG)), *tras_max);
    CPS_REG_WRITE(&(ctlRegBase->LPDDR4__TRAS_MAX_F1__REG), regVal);;

}

/* Helper function to update TREF and TRAS MAX parameters for FSP 0*/
static void LPDDR4_UpdateFSP0RefRateParams(const LPDDR4_PrivateData* pD, const uint32_t* tref, const uint32_t* tras_max) {
    uint32_t regVal = 0U;
    LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;

    /* Update tref parameter */
    regVal = CPS_FLD_WRITE(LPDDR4__TREF_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TREF_F0__REG)), *tref);
    CPS_REG_WRITE(&(ctlRegBase->LPDDR4__TREF_F0__REG), regVal);
    /* Update tRAS MAX parameter */
    regVal = CPS_FLD_WRITE(LPDDR4__TRAS_MAX_F0__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TRAS_MAX_F0__REG)), *tras_max);
    CPS_REG_WRITE(&(ctlRegBase->LPDDR4__TRAS_MAX_F0__REG), regVal);

}

uint32_t LPDDR4_SetRefreshRate(const LPDDR4_PrivateData* pD, const LPDDR4_CtlFspNum* fspNum, const uint32_t* tref, const uint32_t* tras_max) {
    uint32_t result = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_SetRefreshRateSF(pD, fspNum, tref, tras_max);

    if (result == (uint32_t)CDN_EOK) {

        /* Selecting the appropriate register for the user requested Frequency*/
        switch (*fspNum) {
        case LPDDR4_FSP_2:
            LPDDR4_UpdateFSP2RefRateParams (pD, tref, tras_max);
            break;
        case LPDDR4_FSP_1:
            LPDDR4_UpdateFSP1RefRateParams ( pD, tref, tras_max);
            break;
        default:
            /* FSP_0 is considered as the default (sanity check already confirmed it as valid FSP) */
            LPDDR4_UpdateFSP0RefRateParams ( pD, tref,  tras_max);
            break;
        }
    }
    return result;
}

uint32_t LPDDR4_RefreshPerChipSelect(const LPDDR4_PrivateData* pD, const uint32_t trefInterval){
    uint32_t result = 0U;
    uint32_t regVal = 0U;

    /* Calling Sanity Function to verify the input variables*/
    result = LPDDR4_RefreshPerChipSelectSF(pD);

    if (result == (uint32_t)CDN_EOK) {
        LPDDR4_CtlRegs* ctlRegBase = (LPDDR4_CtlRegs*)pD->ctlBase;
        /* Setting tref_interval parameter to enable/disable Refresh per chip select. */
        regVal = CPS_FLD_WRITE(LPDDR4__TREF_INTERVAL__FLD, CPS_REG_READ(&(ctlRegBase->LPDDR4__TREF_INTERVAL__REG)), trefInterval);
        CPS_REG_WRITE(&(ctlRegBase->LPDDR4__TREF_INTERVAL__REG), regVal);
    }
    return result;
}

uint32_t CPS_UncachedRead32(volatile uint32_t* address) 
{
    return *address;
}

void CPS_UncachedWrite32(volatile uint32_t* address, uint32_t value) 
{
    *address = value;
}

/* parasoft-end-suppress METRICS-36-3 */

/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 */

/**
 *  \file elm_v0.c
 *
 *  \brief File containing ELM Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/elm.h>
/*******************************************************************************
*                       API DEFINITIONS
*******************************************************************************/

uint32_t ELM_revisionGet(uint32_t baseAddr)
{
    uint32_t ipRev;

    ipRev = HW_RD_REG32(baseAddr + CSL_ELM_REVISION);

    return (ipRev);
}

void ELM_autoGatingConfig(uint32_t baseAddr, uint32_t configVal)
{
	HW_WR_FIELD32(baseAddr + CSL_ELM_SYSCONFIG,
	              CSL_ELM_SYSCONFIG_AUTOGATING,
	              configVal);
}

void ELM_idleModeSelect(uint32_t baseAddr, uint32_t mode)
{
	HW_WR_FIELD32(baseAddr + CSL_ELM_SYSCONFIG,
	              CSL_ELM_SYSCONFIG_SIDLEMODE,
	              mode);
}

void ELM_moduleReset(uint32_t baseAddr)
{
    /* Initiate the soft reset of the module. */
    HW_WR_FIELD32(baseAddr + CSL_ELM_SYSCONFIG,
                  CSL_ELM_SYSCONFIG_SOFTRESET,
                  ELM_BIT_SET_HIGH);

    /* Wait until the process of Module Reset is complete. */
    while(CSL_ELM_SYSSTS_RESETDONE_RST_DONE !=
        HW_RD_FIELD32(baseAddr + CSL_ELM_SYSSTS, CSL_ELM_SYSSTS_RESETDONE)){}
}

void ELM_clockActivityOCPConfig(uint32_t baseAddr, uint32_t configVal)
{
    HW_WR_FIELD32(baseAddr + CSL_ELM_SYSCONFIG,
                  CSL_ELM_SYSCONFIG_CLOCKACTIVITYOCP,
	              configVal);
}

uint32_t ELM_moduleResetStatusGet(uint32_t baseAddr)
{
    uint32_t resetStat;

    resetStat = HW_RD_FIELD32(baseAddr + CSL_ELM_SYSSTS,
                              CSL_ELM_SYSSTS_RESETDONE);

    return (resetStat);
}

uint32_t ELM_interuptStatusGet(uint32_t baseAddr, uint32_t flag)
{
    uint32_t retVal;

    retVal = 0;

    switch(flag)
    {
        case ELM_LOC_VALID_0_STATUS:
        	 retVal = HW_RD_FIELD32(baseAddr + CSL_ELM_IRQSTS,
        	                        CSL_ELM_IRQSTS_LOC_VALID_0);
        break;
        case ELM_LOC_VALID_1_STATUS:
        	 retVal = HW_RD_FIELD32(baseAddr + CSL_ELM_IRQSTS,
        	                        CSL_ELM_IRQSTS_LOC_VALID_1);
        break;
        case ELM_LOC_VALID_2_STATUS:
        	 retVal = HW_RD_FIELD32(baseAddr + CSL_ELM_IRQSTS,
        	                        CSL_ELM_IRQSTS_LOC_VALID_2);
        break;
        case ELM_LOC_VALID_3_STATUS:
        	 retVal = HW_RD_FIELD32(baseAddr + CSL_ELM_IRQSTS,
        	                        CSL_ELM_IRQSTS_LOC_VALID_3);
        break;
        case ELM_LOC_VALID_4_STATUS:
        	 retVal = HW_RD_FIELD32(baseAddr + CSL_ELM_IRQSTS,
        	                        CSL_ELM_IRQSTS_LOC_VALID_4);
        break;
        case ELM_LOC_VALID_5_STATUS:
        	 retVal = HW_RD_FIELD32(baseAddr + CSL_ELM_IRQSTS,
        	                        CSL_ELM_IRQSTS_LOC_VALID_5);
        break;
        case ELM_LOC_VALID_6_STATUS:
        	 retVal = HW_RD_FIELD32(baseAddr + CSL_ELM_IRQSTS,
        	                        CSL_ELM_IRQSTS_LOC_VALID_6);
        break;
        case ELM_LOC_VALID_7_STATUS:
        	 retVal = HW_RD_FIELD32(baseAddr + CSL_ELM_IRQSTS,
        	                        CSL_ELM_IRQSTS_LOC_VALID_7);
        break;
        case ELM_PAGE_VALID_STATUS:
        	 retVal = HW_RD_FIELD32(baseAddr + CSL_ELM_IRQSTS,
        	                        CSL_ELM_IRQSTS_PAGE_VALID);
        break;
        default:
        /* Nothing to do here */
        break;
    }

    return (retVal);
}

void ELM_interuptStatusClear(uint32_t baseAddr, uint32_t flag)
{

    uint32_t regVal = 0;
    switch(flag)
    {
        case ELM_LOC_VALID_0_STATUS:
            HW_SET_FIELD32(regVal,
                           CSL_ELM_IRQSTS_LOC_VALID_0,
                           ELM_BIT_SET_HIGH);

        break;
        case ELM_LOC_VALID_1_STATUS:
            HW_SET_FIELD32(regVal,
                           CSL_ELM_IRQSTS_LOC_VALID_1,
                           ELM_BIT_SET_HIGH);
        break;
        case ELM_LOC_VALID_2_STATUS:
            HW_SET_FIELD32(regVal,
                           CSL_ELM_IRQSTS_LOC_VALID_2,
                           ELM_BIT_SET_HIGH);
        break;
        case ELM_LOC_VALID_3_STATUS:
            HW_SET_FIELD32(regVal,
                           CSL_ELM_IRQSTS_LOC_VALID_3,
                           ELM_BIT_SET_HIGH);
        break;
        case ELM_LOC_VALID_4_STATUS:
            HW_SET_FIELD32(regVal,
                           CSL_ELM_IRQSTS_LOC_VALID_4,
                           ELM_BIT_SET_HIGH);
        break;
        case ELM_LOC_VALID_5_STATUS:
            HW_SET_FIELD32(regVal,
                           CSL_ELM_IRQSTS_LOC_VALID_5,
                           ELM_BIT_SET_HIGH);
        break;
        case ELM_LOC_VALID_6_STATUS:
            HW_SET_FIELD32(regVal,
                           CSL_ELM_IRQSTS_LOC_VALID_6,
                           ELM_BIT_SET_HIGH);
        break;
        case ELM_LOC_VALID_7_STATUS:
            HW_SET_FIELD32(regVal,
                           CSL_ELM_IRQSTS_LOC_VALID_7,
                           ELM_BIT_SET_HIGH);
        break;
        case ELM_PAGE_VALID_STATUS:
            HW_SET_FIELD32(regVal,
                           CSL_ELM_IRQSTS_PAGE_VALID,
                           ELM_BIT_SET_HIGH);
        break;
        default:
        	 /* Nothing to do here */
        break;
    }

    HW_WR_REG32(baseAddr + CSL_ELM_IRQSTS, regVal);
}

void ELM_interuptConfig(uint32_t baseAddr, uint32_t intFlag, uint32_t configVal)
{
    switch(intFlag)
    {
        case ELM_LOC_VALID_0_STATUS:
            if(configVal == ELM_INT_ENALBLE)
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_0,
                              ELM_BIT_ENABLE);
            }
            else
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_0,
                              ELM_BIT_DISABLE);
            }
        break;
        case ELM_LOC_VALID_1_STATUS:
            if(configVal == ELM_INT_ENALBLE)
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_1,
                              ELM_BIT_ENABLE);
            }
            else
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_1,
                              ELM_BIT_DISABLE);
            }
        break;
        case ELM_LOC_VALID_2_STATUS:
            if(configVal == ELM_INT_ENALBLE)
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_2,
                              ELM_BIT_ENABLE);
            }
            else
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_2,
                              ELM_BIT_DISABLE);
            }
        break;
        case ELM_LOC_VALID_3_STATUS:
            if(configVal == ELM_INT_ENALBLE)
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_3,
                              ELM_BIT_ENABLE);
            }
            else
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_3,
                              ELM_BIT_DISABLE);
            }
        break;
        case ELM_LOC_VALID_4_STATUS:
            if(configVal == ELM_INT_ENALBLE)
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_4,
                              ELM_BIT_ENABLE);
            }
            else
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_4,
                              ELM_BIT_DISABLE);
            }
        break;
        case ELM_LOC_VALID_5_STATUS:
            if(configVal == ELM_INT_ENALBLE)
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_5,
                              ELM_BIT_ENABLE);
            }
            else
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_5,
                              ELM_BIT_DISABLE);
            }
        break;
        case ELM_LOC_VALID_6_STATUS:
            if(configVal == ELM_INT_ENALBLE)
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_6,
                              ELM_BIT_ENABLE);
            }
            else
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_6,
                              ELM_BIT_DISABLE);
            }
        break;
        case ELM_LOC_VALID_7_STATUS:
            if(configVal == ELM_INT_ENALBLE)
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_7,
                              ELM_BIT_ENABLE);
            }
            else
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_LOCATION_MASK_7,
                              ELM_BIT_DISABLE);
            }
        break;
        case ELM_PAGE_VALID_STATUS:
            if(configVal == ELM_INT_ENALBLE)
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_PAGE_MASK,
                              ELM_BIT_ENABLE);
            }
            else
            {
                HW_WR_FIELD32(baseAddr + CSL_ELM_IRQEN,
                              CSL_ELM_IRQEN_PAGE_MASK,
                              ELM_BIT_DISABLE);
            }
        break;
        default:
        	/* Nothing to do here */
        break;
    }
}

void ELM_errorCorrectionLevelSet(uint32_t baseAddr, uint32_t eccLevel)
{
    HW_WR_FIELD32(baseAddr + CSL_ELM_LOCATION_CONFIG,
                  CSL_ELM_LOCATION_CONFIG_ECC_BCH_LEVEL,
                  eccLevel);
}

void ELM_setECCSize(uint32_t baseAddr, uint32_t eccSize)
{
    HW_WR_FIELD32(baseAddr + CSL_ELM_LOCATION_CONFIG,
                  CSL_ELM_LOCATION_CONFIG_ECC_SIZE,
                  eccSize);
}

void ELM_setSectorMode(uint32_t baseAddr, uint32_t mode, uint32_t sectorNum)
{
	uint32_t pageCtrl = HW_RD_REG32(baseAddr + CSL_ELM_PAGE_CTRL);

    if(mode == ELM_MODE_CONTINUOUS)
    {
        pageCtrl &= ~( ((uint32_t)1U) << sectorNum );
    }
    else
    {
        pageCtrl |= ( mode << sectorNum );
    }
    HW_WR_REG32(baseAddr + CSL_ELM_PAGE_CTRL, pageCtrl);
}

void ELM_setSyndromeFragment(uint32_t baseAddr, uint32_t synFrgmtId,
                         uint32_t synFrgmtVal, uint32_t sector)
{
    uint32_t synFrgmtAddr = 0;

    switch (synFrgmtId)
    {
        case ELM_SYNDROME_FRGMT_0:
            synFrgmtAddr = baseAddr + CSL_ELM_SYNDROME_FRAGMENT_0(sector);
            break;
        case ELM_SYNDROME_FRGMT_1:
            synFrgmtAddr = baseAddr + CSL_ELM_SYNDROME_FRAGMENT_1(sector);
            break;
        case ELM_SYNDROME_FRGMT_2:
            synFrgmtAddr = baseAddr + CSL_ELM_SYNDROME_FRAGMENT_2(sector);
            break;
        case ELM_SYNDROME_FRGMT_3:
            synFrgmtAddr = baseAddr + CSL_ELM_SYNDROME_FRAGMENT_3(sector);
            break;
        case ELM_SYNDROME_FRGMT_4:
            synFrgmtAddr = baseAddr + CSL_ELM_SYNDROME_FRAGMENT_4(sector);
            break;
        case ELM_SYNDROME_FRGMT_5:
            synFrgmtAddr = baseAddr + CSL_ELM_SYNDROME_FRAGMENT_5(sector);
            break;
        case ELM_SYNDROME_FRGMT_6:
            synFrgmtAddr = baseAddr + CSL_ELM_SYNDROME_FRAGMENT_6(sector);
            break;

        default:
            break;
    }

    if(synFrgmtId != ELM_SYNDROME_FRGMT_6)
    {
        HW_WR_REG32(synFrgmtAddr, synFrgmtVal);
    }
    else
    {
        HW_WR_REG32(synFrgmtAddr, synFrgmtVal & CSL_ELM_SYNDROME_FRAGMENT_6_SYNDROME_6_MASK);
    }
}

void ELM_errorLocationProcessingStart(uint32_t baseAddr, uint32_t sector)
{
    uint32_t synFrgmtAddr;
    uint32_t synFrgmtVal;

    /* Initiate the processing of syndrome polynomial. */
    synFrgmtAddr = baseAddr + CSL_ELM_SYNDROME_FRAGMENT_6(sector);
    synFrgmtVal = HW_RD_REG32(synFrgmtAddr) | \
                  CSL_ELM_SYNDROME_FRAGMENT_6_SYNDROME_VALID_MASK;
    HW_WR_REG32(synFrgmtAddr, synFrgmtVal);
}

uint32_t ELM_errorLocationProcessingStatusGet(uint32_t baseAddr, uint32_t sector)
{
    uint32_t status;

    /* Get the status of the Error location process. */
    status = HW_RD_FIELD32(baseAddr + CSL_ELM_LOCATION_STS(sector),
                           CSL_ELM_LOCATION_STS_ECC_CORRECTBL);
    return (status);
}

uint32_t ELM_getNumError(uint32_t baseAddr, uint32_t sector)
{
    uint32_t numOfErrs;

    /* The number of errors located for the particular syndrome polynomial. */
    numOfErrs = HW_RD_FIELD32(baseAddr + CSL_ELM_LOCATION_STS(sector),
                              CSL_ELM_LOCATION_STS_ECC_NB_ERRORS);

    return (numOfErrs);
}

uint32_t ELM_errorLocationBitAddrGet(uint32_t baseAddr, uint32_t errNum, uint32_t sector)
{
    uint32_t errLocOffset = 0;
    uint32_t bitAddr;

    switch(errNum)
    {
        case ELM_ERROR_NUM_0:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_0(sector);
            break;
        case ELM_ERROR_NUM_1:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_1(sector);
            break;
        case ELM_ERROR_NUM_2:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_2(sector);
            break;
        case ELM_ERROR_NUM_3:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_3(sector);
            break;
        case ELM_ERROR_NUM_4:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_4(sector);
            break;
        case ELM_ERROR_NUM_5:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_5(sector);
            break;
        case ELM_ERROR_NUM_6:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_6(sector);
            break;
        case ELM_ERROR_NUM_7:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_7(sector);
            break;
        case ELM_ERROR_NUM_8:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_8(sector);
            break;
        case ELM_ERROR_NUM_9:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_9(sector);
            break;
        case ELM_ERROR_NUM_10:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_10(sector);
            break;
        case ELM_ERROR_NUM_11:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_11(sector);
            break;
        case ELM_ERROR_NUM_12:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_12(sector);
            break;
        case ELM_ERROR_NUM_13:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_13(sector);
            break;
        case ELM_ERROR_NUM_14:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_14(sector);
            break;
        case ELM_ERROR_NUM_15:
            errLocOffset = baseAddr + CSL_ELM_ERROR_LOCATION_15(sector);
            break;
        default:
            break;
    }

    bitAddr = HW_RD_FIELD32(errLocOffset,
                              CSL_ELM_ERROR_LOCATION_0_ECC_ERROR_LOCATION);

    return (bitAddr);
}

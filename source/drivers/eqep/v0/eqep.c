/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file   eqep.c
 *
 *  \brief  This file contains the driver APIs for EQEP.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/eqep.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EQEP_enableModule(uint32_t baseAddr)
{
    uint16_t regVal;

    /* Enable the eQEP module */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPCTL);
    regVal |= CSL_EQEP_QEPCTL_QPEN_MASK;
    HW_WR_REG16(baseAddr + CSL_EQEP_QEPCTL, regVal);
}

void EQEP_setDecoderConfig(uint32_t baseAddr, uint16_t config)
{
    uint16_t regVal;

    /* Write the new decoder configuration to the hardware */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2);
    regVal = (regVal
            & ~(CSL_EQEP_QDECCTL_TYPE2_SWAP_MASK
                    | CSL_EQEP_QDECCTL_TYPE2_XCR_MASK
                    | CSL_EQEP_QDECCTL_TYPE2_QSRC_MASK)) | config;
    HW_WR_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2, regVal);
}

void EQEP_disableModule(uint32_t baseAddr)
{
    uint16_t regVal;

    /* Disable the eQEP module */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPCTL);
    regVal &= (uint16_t) (~CSL_EQEP_QEPCTL_QPEN_MASK);
    HW_WR_REG16(baseAddr + CSL_EQEP_QEPCTL, regVal);
}

void EQEP_setPositionCounterConfig(uint32_t baseAddr,
        eqepPositionResetMode_t mode, uint32_t maxPosition)
{
    uint32_t regVal;

    /* Write the position counter reset configuration to the hardware */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPCTL);
    regVal = (regVal & ~CSL_EQEP_QEPCTL_PCRM_MASK) | (uint16_t) mode;
    HW_WR_REG16(baseAddr + CSL_EQEP_QEPCTL, regVal);

    /* Set the maximum position */
    HW_WR_REG32(baseAddr + CSL_EQEP_QPOSMAX, maxPosition);
}

uint32_t EQEP_getPosition(uint32_t baseAddr)
{
    uint32_t retVal;

    /* Return the current position counter */
    retVal = HW_RD_REG32(baseAddr + CSL_EQEP_QPOSCNT);

    return (retVal);
}

void EQEP_setPosition(uint32_t baseAddr, uint32_t position)
{
    /* Set the position counter */
    HW_WR_REG32(baseAddr + CSL_EQEP_QPOSCNT, position);
}

int32_t EQEP_getDirection(uint32_t baseAddr)
{
    int32_t retVal;
    uint16_t regVal;

    /* Return the direction of rotation */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPSTS_TYPE1);
    if ((regVal & CSL_EQEP_QEPSTS_TYPE1_QDF_MASK) != 0U)
    {
        retVal = EQEP_MOD_FORWARD_DIR;
    }
    else
    {
        retVal = EQEP_MOD_BACKWARD_DIR;
    }

    return (retVal);
}

void EQEP_enableInterrupt(uint32_t baseAddr, uint16_t intFlags)
{
    uint16_t regVal;

    /* Enable the specified interrupts */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEINT_TYPE1);
    regVal |= intFlags;
    HW_WR_REG16(baseAddr + CSL_EQEP_QEINT_TYPE1, regVal);
}

void EQEP_disableInterrupt(uint32_t baseAddr, uint16_t intFlags)
{
    uint16_t regVal;

    /* Disable the specified interrupts */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEINT_TYPE1);
    regVal &= ~(intFlags);
    HW_WR_REG16(baseAddr + CSL_EQEP_QEINT_TYPE1, regVal);
}

uint16_t EQEP_getEnabledInterrupt(uint32_t baseAddr)
{
    uint16_t regVal;

    /* Enable the specified interrupts */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEINT_TYPE1);

    return regVal;
}

uint16_t EQEP_getInterruptStatus(uint32_t baseAddr)
{
    uint16_t retVal;

    retVal = HW_RD_REG16(baseAddr + CSL_EQEP_QFLG_TYPE1);

    return (retVal);
}

void EQEP_clearInterruptStatus(uint32_t baseAddr, uint16_t intFlags)
{
    /* Clear the requested interrupt sources */
    HW_WR_REG16(baseAddr + CSL_EQEP_QCLR_TYPE1, intFlags);
}

void EQEP_forceInterrupt(uint32_t baseAddr, uint16_t intFlags)
{
    uint16_t regVal;

    /* Force the specified interrupts */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QFRC_TYPE1);
    regVal |= intFlags;
    HW_WR_REG16(baseAddr + CSL_EQEP_QFRC_TYPE1, regVal);
}

Bool EQEP_isErrorSet(uint32_t baseAddr)
{
    Bool retVal = FALSE;
    uint16_t regVal;

    /* Return the error indicator */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QFLG_TYPE1);
    if ((regVal & CSL_EQEP_QFLG_TYPE1_PHE_MASK) != 0U)
    {
        retVal = TRUE;
    }

    return (retVal);
}

uint32_t EQEP_getStatus(uint32_t baseAddr)
{
    uint32_t retVal;
    uint16_t regVal;

    /* Return the status register */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPSTS_TYPE1);
    retVal = regVal & 0x00FFU;

    return (retVal);
}

void EQEP_clearStatus(uint32_t baseAddr, uint16_t statusFlags)
{
    /* Clear the requested interrupt sources */
    HW_WR_REG16(baseAddr + CSL_EQEP_QEPSTS_TYPE1, statusFlags);
}

void EQEP_setCaptureConfig(uint32_t baseAddr, eqepCapClkPrescale_t capPrescale,
        eqeqUpEvntPrescale_t evntPrescale)
{
    uint16_t regVal;

    /* Write new prescaler configurations to the appropriate registers */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QCAPCTL);
    regVal = (regVal
            & ~(CSL_EQEP_QCAPCTL_UPPS_MASK | CSL_EQEP_QCAPCTL_CCPS_MASK))
            | ((uint16_t) evntPrescale | (uint16_t) capPrescale);
    HW_WR_REG16(baseAddr + CSL_EQEP_QCAPCTL, regVal);
}

void EQEP_enableCapture(uint32_t baseAddr)
{
    uint16_t regVal;

    /* Enable edge capture */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QCAPCTL);
    regVal |= CSL_EQEP_QCAPCTL_CEN_MASK;
    HW_WR_REG16(baseAddr + CSL_EQEP_QCAPCTL, regVal);
}

void EQEP_disableCapture(uint32_t baseAddr)
{
    uint16_t regVal;

    /* Disable edge capture */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QCAPCTL);
    regVal &= ~(CSL_EQEP_QCAPCTL_CEN_MASK);
    HW_WR_REG16(baseAddr + CSL_EQEP_QCAPCTL, regVal);
}

uint32_t EQEP_getCapturePeriod(uint32_t baseAddr)
{
    uint32_t retVal;

    /* Return the capture period */
    retVal = HW_RD_REG16(baseAddr + CSL_EQEP_QCPRD);

    return (retVal);
}

uint32_t EQEP_getCaptureTimer(uint32_t baseAddr)
{
    uint32_t retVal;

    /* Return the capture timer value */
    retVal = HW_RD_REG16(baseAddr + CSL_EQEP_QCTMR);

    return (retVal);
}

void EQEP_enableCompare(uint32_t baseAddr)
{
    uint16_t regVal;

    /* Enable position compare */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QPOSCTL);
    regVal |= CSL_EQEP_QPOSCTL_PCE_MASK;
    HW_WR_REG16(baseAddr + CSL_EQEP_QPOSCTL, regVal);
}

void EQEP_disableCompare(uint32_t baseAddr)
{
    uint16_t regVal;

    /* Disable position compare */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QPOSCTL);
    regVal &= ~(CSL_EQEP_QPOSCTL_PCE_MASK);
    HW_WR_REG16(baseAddr + CSL_EQEP_QPOSCTL, regVal);
}

int32_t EQEP_setComparePulseWidth(uint32_t baseAddr, uint16_t cycles)
{
    int32_t retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (cycles > (CSL_EQEP_QPOSCTL_PCSPW_MASK + 1U))
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        /* Set the pulse width */
        regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QPOSCTL);
        regVal = (regVal & ~CSL_EQEP_QPOSCTL_PCSPW_MASK) | (cycles - 1U);
        HW_WR_REG16(baseAddr + CSL_EQEP_QPOSCTL, regVal);
    }

    return (retVal);
}

void EQEP_enableUnitTimer(uint32_t baseAddr, uint32_t period)
{
    uint16_t regVal;

    /* Set the period of the unit timer */
    HW_WR_REG32(baseAddr + CSL_EQEP_QUPRD, period);

    /* Enable peripheral unit timer */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPCTL);
    regVal |= CSL_EQEP_QEPCTL_UTE_MASK;
    HW_WR_REG16(baseAddr + CSL_EQEP_QEPCTL, regVal);
}

void EQEP_disableUnitTimer(uint32_t baseAddr)
{
    uint16_t regVal;

    /* Disable peripheral unit timer */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPCTL);
    regVal &= ~(CSL_EQEP_QEPCTL_UTE_MASK);
    HW_WR_REG16(baseAddr + CSL_EQEP_QEPCTL, regVal);
}

void EQEP_enableWatchdog(uint32_t baseAddr, uint16_t period)
{
    uint16_t regVal;

    /* Set the timeout count for the eQEP peripheral watchdog timer */
    HW_WR_REG16(baseAddr + CSL_EQEP_QWDPRD, period);

    /* Enable peripheral watchdog */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPCTL);
    regVal |= CSL_EQEP_QEPCTL_WDE_MASK;
    HW_WR_REG16(baseAddr + CSL_EQEP_QEPCTL, regVal);
}

void EQEP_disableWatchdog(uint32_t baseAddr)
{
    uint16_t regVal;

    /* Disable peripheral watchdog */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPCTL);
    regVal &= ~(CSL_EQEP_QEPCTL_WDE_MASK);
    HW_WR_REG16(baseAddr + CSL_EQEP_QEPCTL, regVal);
}

void EQEP_setWatchdogTimerValue(uint32_t baseAddr, uint16_t value)
{
    /* Write the value to the watchdog timer register */
    HW_WR_REG16(baseAddr + CSL_EQEP_QWDTMR, value);
}

uint32_t EQEP_getWatchdogTimerValue(uint32_t baseAddr)
{
    uint32_t retVal;

    /* Read the value from the watchdog timer register */
    retVal = HW_RD_REG16(baseAddr + CSL_EQEP_QWDTMR);

    return (retVal);
}

void EQEP_setPositionInitMode(uint32_t baseAddr, uint16_t initMode)
{
    uint16_t regVal;

    /* Set the init mode in the QEP Control register */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPCTL);
    regVal = (regVal & ~(CSL_EQEP_QEPCTL_IEI_MASK | CSL_EQEP_QEPCTL_SEI_MASK))
            | initMode;
    HW_WR_REG16(baseAddr + CSL_EQEP_QEPCTL, regVal);
}

void EQEP_setSWPositionInit(uint32_t baseAddr, Bool initialize)
{
    uint16_t regVal;

    /* Set or clear the software initialization bit */
    if (initialize)
    {
        regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPCTL);
        regVal |= CSL_EQEP_QEPCTL_SWI_MASK;
        HW_WR_REG16(baseAddr + CSL_EQEP_QEPCTL, regVal);
    }
    else
    {
        regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPCTL);
        regVal &= ~CSL_EQEP_QEPCTL_SWI_MASK;
        HW_WR_REG16(baseAddr + CSL_EQEP_QEPCTL, regVal);
    }
}

void EQEP_setInitialPosition(uint32_t baseAddr, uint32_t position)
{
    /* Write position to position counter init register */
    HW_WR_REG32(baseAddr + CSL_EQEP_QPOSINIT, position);
}

void EQEP_setLatchMode(uint32_t baseAddr, uint32_t latchMode)
{
    uint16_t regVal;

    /* Set the latch mode in the QEP Control register */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPCTL);
    regVal = (regVal
            & ~(CSL_EQEP_QEPCTL_QCLM_MASK | CSL_EQEP_QEPCTL_IEL_MASK
                    | CSL_EQEP_QEPCTL_SEL_MASK)) | latchMode;
    HW_WR_REG16(baseAddr + CSL_EQEP_QEPCTL, regVal);
}

uint32_t EQEP_getIndexPositionLatch(uint32_t baseAddr)
{
    uint32_t retVal;

    /* Return the current position counter */
    retVal = HW_RD_REG32(baseAddr + CSL_EQEP_QPOSILAT);

    return (retVal);
}

uint32_t EQEP_getStrobePositionLatch(uint32_t baseAddr)
{
    uint32_t retVal;

    /* Return the current position counter */
    retVal = HW_RD_REG32(baseAddr + CSL_EQEP_QPOSSLAT);

    return (retVal);
}

uint32_t EQEP_getPositionLatch(uint32_t baseAddr)
{
    uint32_t retVal;

    /* Return the current position counter */
    retVal = HW_RD_REG32(baseAddr + CSL_EQEP_QPOSLAT);

    return (retVal);
}

uint32_t EQEP_getCaptureTimerLatch(uint32_t baseAddr)
{
    uint32_t retVal;

    /* Return the current position counter */
    retVal = HW_RD_REG16(baseAddr + CSL_EQEP_QCTMRLAT);

    return (retVal);
}

uint32_t EQEP_getCapturePeriodLatch(uint32_t baseAddr)
{
    uint32_t retVal;

    /* Return the current position counter */
    retVal = HW_RD_REG16(baseAddr + CSL_EQEP_QCPRDLAT);

    return (retVal);
}

void EQEP_setQMAModuleMode(uint32_t baseAddr, eqepQmaMode_t qmaMode)
{
    uint32_t regVal;

    /* Write the QMA module mode into the appropriate register */
    regVal = HW_RD_REG32(baseAddr + CSL_EQEP_QMACTRL);
    regVal = (regVal & ~CSL_EQEP_QMACTRL_MODE_MASK) | (uint16_t) qmaMode;
    HW_WR_REG32(baseAddr + CSL_EQEP_QMACTRL, regVal);
}

void EQEP_setStrobeSource(uint32_t baseAddr, eqepStrobeSource_t strobeSrc)
{
    uint32_t regVal;

    /* Write the strobe source selection into the appropriate register */
    regVal = HW_RD_REG32(baseAddr + CSL_EQEP_QEPSTROBESEL);
    regVal = (regVal & ~CSL_EQEP_QEPSTROBESEL_STROBESEL_MASK)
            | (uint16_t) strobeSrc;
    HW_WR_REG32(baseAddr + CSL_EQEP_QEPSTROBESEL, regVal);
}

void EQEP_setEmulationMode(uint32_t baseAddr, eqepEmulationMode_t emuMode)
{
    uint16_t regVal;

    /* Write the emulation mode to the FREE_SOFT bits */
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QEPCTL);
    regVal = (regVal & ~CSL_EQEP_QEPCTL_FREE_SOFT_MASK)
            | ((uint16_t) emuMode << CSL_EQEP_QEPCTL_FREE_SOFT_SHIFT);
    HW_WR_REG16(baseAddr + CSL_EQEP_QEPCTL, regVal);
}

void EQEP_setCompareConfig(uint32_t baseAddr, uint16_t config,
                           uint32_t compareValue, uint16_t cycles)
{
    uint16_t regVal, temp;

    /* Set the compare match value */
    HW_WR_REG32(baseAddr + CSL_EQEP_QPOSCMP, compareValue);

    /* Set the shadow register settings and pulse width */
    temp = (config
            & (CSL_EQEP_QPOSCTL_PCSHDW_MASK | CSL_EQEP_QPOSCTL_PCLOAD_MASK))
            | (cycles - 1U);

    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QPOSCTL);
    regVal = (regVal
            & ~(CSL_EQEP_QPOSCTL_PCSPW_MASK | CSL_EQEP_QPOSCTL_PCLOAD_MASK
                    | CSL_EQEP_QPOSCTL_PCSHDW_MASK)) | temp;

    HW_WR_REG16(baseAddr + CSL_EQEP_QPOSCTL, regVal);

    /* Set position compare sync-output mode */
    temp = config
            & (CSL_EQEP_QDECCTL_TYPE2_SOEN_MASK
                    | CSL_EQEP_QDECCTL_TYPE2_SPSEL_MASK);
    regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2);
    regVal = (regVal
            & ~(CSL_EQEP_QDECCTL_TYPE2_SOEN_MASK
                    | CSL_EQEP_QDECCTL_TYPE2_SPSEL_MASK)) | temp;
    HW_WR_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2, regVal);
}

void EQEP_setInputPolarity(uint32_t baseAddr, Bool invertQEPA, Bool invertQEPB,
                           Bool invertIndex, Bool invertStrobe)
{
    uint16_t regVal;

    /* Configure QEPA signal */
    if (invertQEPA)
    {
        regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2);
        regVal |= CSL_EQEP_QDECCTL_TYPE2_QAP_MASK;
        HW_WR_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2, regVal);
    }
    else
    {
        regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2);
        regVal &= ~CSL_EQEP_QDECCTL_TYPE2_QAP_MASK;
        HW_WR_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2, regVal);
    }

    /* Configure QEPB signal */
    if (invertQEPB)
    {
        regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2);
        regVal |= CSL_EQEP_QDECCTL_TYPE2_QBP_MASK;
        HW_WR_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2, regVal);
    }
    else
    {
        regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2);
        regVal &= ~CSL_EQEP_QDECCTL_TYPE2_QBP_MASK;
        HW_WR_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2, regVal);
    }

    /* Configure index signal */
    if (invertIndex)
    {
        regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2);
        regVal |= CSL_EQEP_QDECCTL_TYPE2_QIP_MASK;
        HW_WR_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2, regVal);
    }
    else
    {
        regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2);
        regVal &= ~CSL_EQEP_QDECCTL_TYPE2_QIP_MASK;
        HW_WR_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2, regVal);
    }

    /* Configure strobe signal */
    if (invertStrobe)
    {
        regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2);
        regVal |= CSL_EQEP_QDECCTL_TYPE2_QSP_MASK;
        HW_WR_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2, regVal);
    }
    else
    {
        regVal = HW_RD_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2);
        regVal &= ~CSL_EQEP_QDECCTL_TYPE2_QSP_MASK;
        HW_WR_REG16(baseAddr + CSL_EQEP_QDECCTL_TYPE2, regVal);
    }
}

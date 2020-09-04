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
 *  \file   ecap.c
 *
 *  \brief  Low lever APIs performing hardware register writes and reads for
 *          ECAP IP version 0.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/ecap.h>
#include <drivers/hw_include/hw_types.h>

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

void ECAP_captureLoadingEnable(uint32_t baseAddr)
{
    uint32_t temp_addr = baseAddr + ECAP_ECCTL1;
    uint16_t value     = HW_RD_REG16(temp_addr);
    value |= (uint16_t)ECAP_ECCTL1_CAPLDEN;
    HW_WR_REG16(temp_addr, value);
}

void ECAP_captureLoadingDisable(uint32_t baseAddr)
{
    uint32_t temp_addr = baseAddr + ECAP_ECCTL1;
    uint16_t value     = HW_RD_REG16(temp_addr);
    value &= ((uint16_t)(~ECAP_ECCTL1_CAPLDEN));
    HW_WR_REG16(temp_addr, value);
}

void ECAP_prescaleConfig(uint32_t baseAddr, uint32_t prescale)
{
    uint32_t temp_addr;

    if (prescale <= ECAP_ECCTL1_PRESCALE_MAX)
    {
        temp_addr = baseAddr + ECAP_ECCTL1;
        HW_WR_FIELD16(temp_addr, ECAP_ECCTL1_PRESCALE, (uint16_t)prescale);
    }
}

void ECAP_operatingModeSelect(uint32_t baseAddr, uint32_t modeSelect)
{
    uint32_t temp_addr = baseAddr + ECAP_ECCTL2;
    HW_WR_FIELD16(temp_addr, ECAP_ECCTL2_CAP_APWM, (uint16_t)modeSelect);
}

uint32_t ECAP_timeStampRead(uint32_t baseAddr, uint32_t capEvtFlag)
{
    return(HW_RD_REG32(baseAddr + capEvtFlag));
}

void ECAP_counterConfig(uint32_t baseAddr, uint32_t countVal)
{
    HW_WR_REG32((baseAddr + ECAP_TSCTR), countVal);
}

void ECAP_captureEvtPolarityConfig(uint32_t baseAddr, uint32_t capEvt1pol,
                               uint32_t capEvt2pol, uint32_t capEvt3pol,
                               uint32_t capEvt4pol)
{
    uint32_t temp_addr = baseAddr + ECAP_ECCTL1;
    uint16_t value = HW_RD_REG16(temp_addr);

    value &= ~ECAP_ECCTL1_CAP1POL_MASK;
    value |= (uint16_t)((capEvt1pol << ECAP_ECCTL1_CAP1POL_SHIFT) &
                ECAP_ECCTL1_CAP1POL_MASK);
    value &= ~ECAP_ECCTL1_CAP2POL_MASK;
    value |= (uint16_t)((capEvt2pol << ECAP_ECCTL1_CAP2POL_SHIFT) &
                ECAP_ECCTL1_CAP2POL_MASK);
    value &= ~ECAP_ECCTL1_CAP3POL_MASK;
    value |= (uint16_t)((capEvt3pol << ECAP_ECCTL1_CAP3POL_SHIFT) &
                ECAP_ECCTL1_CAP3POL_MASK);
    value &= ~ECAP_ECCTL1_CAP4POL_MASK;
    value |= (uint16_t)((capEvt4pol << ECAP_ECCTL1_CAP4POL_SHIFT) &
                ECAP_ECCTL1_CAP4POL_MASK);
    HW_WR_REG16(temp_addr, value);
}

void ECAP_captureEvtCntrRstConfig(uint32_t baseAddr, uint32_t counterRst1,
                                 uint32_t counterRst2,uint32_t counterRst3,
                                 uint32_t counterRst4)
{
    uint32_t temp_addr = baseAddr + ECAP_ECCTL1;
    uint16_t value = HW_RD_REG16(temp_addr);

    value &= ~ECAP_ECCTL1_CTRRST1_MASK;
    value |= (uint16_t)((counterRst1 << ECAP_ECCTL1_CTRRST1_SHIFT) &
                ECAP_ECCTL1_CTRRST1_MASK);
    value &= ~ECAP_ECCTL1_CTRRST2_MASK;
    value |= (uint16_t)((counterRst2 << ECAP_ECCTL1_CTRRST2_SHIFT) &
                ECAP_ECCTL1_CTRRST2_MASK);
    value &= ~ECAP_ECCTL1_CTRRST3_MASK;
    value |= (uint16_t)((counterRst3 << ECAP_ECCTL1_CTRRST3_SHIFT) &
                ECAP_ECCTL1_CTRRST3_MASK);
    value &= ~ECAP_ECCTL1_CTRRST4_MASK;
    value |= (uint16_t)((counterRst4 << ECAP_ECCTL1_CTRRST4_SHIFT) &
                ECAP_ECCTL1_CTRRST4_MASK);
    HW_WR_REG16(temp_addr, value);
}

void ECAP_continousModeConfig(uint32_t baseAddr)
{
    uint32_t temp_addr = baseAddr + ECAP_ECCTL2;
    uint16_t value = HW_RD_REG16(temp_addr);
    value  &= ((uint16_t)(~ECAP_ECCTL2_CONT_ONESHT));
    HW_WR_REG16(temp_addr, value);
}

void ECAP_oneShotModeConfig(uint32_t baseAddr, uint32_t stopVal)
{
    uint32_t temp_addr = baseAddr + ECAP_ECCTL2;
    uint16_t value = HW_RD_REG16(temp_addr);
    value |= (uint16_t)ECAP_ECCTL2_CONT_ONESHT;
    HW_WR_REG16(temp_addr, value);

    /* Configure Stop Value for One-Shot Mode */
    HW_WR_FIELD16(temp_addr, ECAP_ECCTL2_STOP_WRAP, (uint16_t)stopVal);
}

void ECAP_oneShotReArm(uint32_t baseAddr)
{
    uint32_t temp_addr = baseAddr + ECAP_ECCTL2;
    uint16_t value = HW_RD_REG16(temp_addr);
    value |= ((uint16_t)ECAP_ECCTL2_RE_ARM);
    HW_WR_REG16(temp_addr, value);
}

void ECAP_APWM_polarityConfig(uint32_t baseAddr, uint32_t flag)
{
    uint32_t temp_addr = baseAddr + ECAP_ECCTL2;
    HW_WR_FIELD16(temp_addr, ECAP_ECCTL2_APWMPOL, (uint16_t)flag);
}

void ECAP_counterControl(uint32_t baseAddr, uint32_t flag)
{
    uint32_t temp_addr = baseAddr + ECAP_ECCTL2;
    HW_WR_FIELD16(temp_addr, ECAP_ECCTL2_TSCTRSTOP, (uint16_t)flag);
}

void ECAP_syncInOutSelect(uint32_t baseAddr, uint32_t syncIn,
                         uint32_t syncOut)
{
    uint32_t temp_addr = baseAddr + ECAP_ECCTL2;
    HW_WR_FIELD16(temp_addr, ECAP_ECCTL2_SYNCI_EN, (uint16_t)syncIn);
    HW_WR_FIELD16(temp_addr, ECAP_ECCTL2_SYNCO_SEL, (uint16_t)syncOut);
}

void ECAP_APWM_captureConfig(uint32_t baseAddr, uint32_t compareVal,
                           uint32_t periodVal)
{
    HW_WR_REG32((baseAddr + ECAP_CAP1),periodVal);
    HW_WR_REG32((baseAddr + ECAP_CAP2),compareVal);
}

void ECAP_APWM_shadowCaptureConfig(uint32_t baseAddr, uint32_t compareVal,
                                 uint32_t periodVal)
{
    HW_WR_REG32((baseAddr + ECAP_CAP1),periodVal);
    HW_WR_REG32((baseAddr + ECAP_CAP2),compareVal);
}

void ECAP_counterPhaseValConfig(uint32_t baseAddr, uint32_t cntPhaseVal)
{
    HW_WR_REG32((baseAddr + ECAP_CTRPHS),cntPhaseVal);
}

void ECAP_globalIntrClear(uint32_t baseAddr)
{
    uint32_t temp_addr = baseAddr + ECAP_ECCLR;
    uint16_t value = HW_RD_REG16(temp_addr);
    value  |= ((uint16_t)ECAP_ECCLR_INT);
    HW_WR_REG16(temp_addr, value);
}

void ECAP_intrEnable(uint32_t baseAddr, uint32_t flag)
{
    uint32_t temp_addr = baseAddr + ECAP_ECEINT;
    uint16_t value = HW_RD_REG16(temp_addr);
    value  |= ((uint16_t)flag);
    HW_WR_REG16(temp_addr, value);
}

void ECAP_intrDisable(uint32_t baseAddr, uint32_t flag)
{
    uint32_t temp_addr = baseAddr + ECAP_ECEINT;
    uint32_t value = HW_RD_REG32(temp_addr);
    uint16_t value16;
    value16  = ((uint16_t)(value & ~flag));
    HW_WR_REG16(temp_addr, value16);
}

uint32_t ECAP_getIntrStatus(uint32_t baseAddr, uint32_t flag)
{
    uint32_t temp_addr = baseAddr + ECAP_ECFLG;
    uint16_t value = HW_RD_REG16(temp_addr);
    return (value & flag);
}

void ECAP_intrStatusClear(uint32_t baseAddr, uint32_t flag)
{
    uint32_t temp_addr = baseAddr + ECAP_ECFLG;
    uint16_t value = HW_RD_REG16(temp_addr);
    temp_addr = baseAddr + ECAP_ECCLR;
    HW_WR_REG16(temp_addr,  value  & ((uint16_t)flag));
}

uint32_t ECAP_peripheralIdGet(uint32_t baseAddr)
{
    return(HW_RD_REG32(baseAddr + ECAP_REVID));
}

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
 *          ECAP IP version 2.
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
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCTL1_ECCTL2;
    uint32_t value     = HW_RD_REG32(temp_addr);
    value |= CSL_ECAP_ECCTL1_ECCTL2_CAPLDEN_MASK;
    HW_WR_REG32(temp_addr, value);
}

void ECAP_captureLoadingDisable(uint32_t baseAddr)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCTL1_ECCTL2;
    uint32_t value     = HW_RD_REG32(temp_addr);
    value &= (~CSL_ECAP_ECCTL1_ECCTL2_CAPLDEN_MASK);
    HW_WR_REG32(temp_addr, value);
}

void ECAP_prescaleConfig(uint32_t baseAddr, uint32_t prescale)
{
    uint32_t temp_addr;

    if (prescale <= CSL_ECAP_ECCTL1_ECCTL2_PRESCALE_MAX)
    {
        temp_addr = baseAddr + CSL_ECAP_ECCTL1_ECCTL2;
        HW_WR_FIELD32(temp_addr,CSL_ECAP_ECCTL1_ECCTL2_PRESCALE, prescale);
    }
}

void ECAP_operatingModeSelect(uint32_t baseAddr, uint32_t modeSelect)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCTL1_ECCTL2;
    HW_WR_FIELD32(temp_addr, CSL_ECAP_ECCTL1_ECCTL2_CAP_APWM, modeSelect);
}

uint32_t ECAP_timeStampRead(uint32_t baseAddr, uint32_t capEvtFlag)
{
    return(HW_RD_REG32(baseAddr + capEvtFlag));
}

void ECAP_counterConfig(uint32_t baseAddr, uint32_t countVal)
{
    HW_WR_REG32((baseAddr + CSL_ECAP_TSCTR), countVal);
}

void ECAP_captureEvtPolarityConfig(uint32_t baseAddr, uint32_t capEvt1pol,
                               uint32_t capEvt2pol, uint32_t capEvt3pol,
                               uint32_t capEvt4pol)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCTL1_ECCTL2;
    uint32_t value = HW_RD_REG32(temp_addr);

    value &= ~CSL_ECAP_ECCTL1_ECCTL2_CAP1POL_MASK;
    value |= ((capEvt1pol << CSL_ECAP_ECCTL1_ECCTL2_CAP1POL_SHIFT) &
                CSL_ECAP_ECCTL1_ECCTL2_CAP1POL_MASK);
    value &= ~CSL_ECAP_ECCTL1_ECCTL2_CAP2POL_MASK;
    value |= ((capEvt2pol << CSL_ECAP_ECCTL1_ECCTL2_CAP2POL_SHIFT) &
                CSL_ECAP_ECCTL1_ECCTL2_CAP2POL_MASK);
    value &= ~CSL_ECAP_ECCTL1_ECCTL2_CAP3POL_MASK;
    value |= ((capEvt3pol << CSL_ECAP_ECCTL1_ECCTL2_CAP3POL_SHIFT) &
                CSL_ECAP_ECCTL1_ECCTL2_CAP3POL_MASK);
    value &= ~CSL_ECAP_ECCTL1_ECCTL2_CAP4POL_MASK;
    value |= (uint32_t)((capEvt4pol << CSL_ECAP_ECCTL1_ECCTL2_CAP4POL_SHIFT) &
                CSL_ECAP_ECCTL1_ECCTL2_CAP4POL_MASK);
    HW_WR_REG32(temp_addr, value);
}

void ECAP_captureEvtCntrRstConfig(uint32_t baseAddr, uint32_t counterRst1,
                                 uint32_t counterRst2,uint32_t counterRst3,
                                 uint32_t counterRst4)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCTL1_ECCTL2;
    uint32_t value = HW_RD_REG32(temp_addr);

    value &= ~CSL_ECAP_ECCTL1_ECCTL2_CTRRST1_MASK;
    value |= (uint32_t)((counterRst1 << CSL_ECAP_ECCTL1_ECCTL2_CTRRST1_SHIFT) &
                CSL_ECAP_ECCTL1_ECCTL2_CTRRST1_MASK);
    value &= ~CSL_ECAP_ECCTL1_ECCTL2_CTRRST2_MASK;
    value |= (uint32_t)((counterRst2 << CSL_ECAP_ECCTL1_ECCTL2_CTRRST2_SHIFT) &
                CSL_ECAP_ECCTL1_ECCTL2_CTRRST2_MASK);
    value &= ~CSL_ECAP_ECCTL1_ECCTL2_CTRRST3_MASK;
    value |= (uint32_t)((counterRst3 << CSL_ECAP_ECCTL1_ECCTL2_CTRRST3_SHIFT) &
                CSL_ECAP_ECCTL1_ECCTL2_CTRRST3_MASK);
    value &= ~CSL_ECAP_ECCTL1_ECCTL2_CTRRST4_MASK;
    value |= (uint32_t)((counterRst4 << CSL_ECAP_ECCTL1_ECCTL2_CTRRST4_SHIFT) &
                CSL_ECAP_ECCTL1_ECCTL2_CTRRST4_MASK);
    HW_WR_REG32(temp_addr, value);
}

void ECAP_continousModeConfig(uint32_t baseAddr)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCTL1_ECCTL2;
    uint32_t value = HW_RD_REG32(temp_addr);
    value  &= ((~CSL_ECAP_ECCTL1_ECCTL2_CONT_ONESHT_MASK));
    HW_WR_REG32(temp_addr, value);
}

void ECAP_oneShotModeConfig(uint32_t baseAddr, uint32_t stopVal)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCTL1_ECCTL2;
    uint32_t value = HW_RD_REG32(temp_addr);
    value |= CSL_ECAP_ECCTL1_ECCTL2_CONT_ONESHT_MASK;
    HW_WR_REG32(temp_addr, value);

    /* Configure Stop Value for One-Shot Mode */
    HW_WR_FIELD32(temp_addr, CSL_ECAP_ECCTL1_ECCTL2_STOP_WRAP, stopVal);
}

void ECAP_oneShotReArm(uint32_t baseAddr)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCTL1_ECCTL2;
    uint32_t value = HW_RD_REG32(temp_addr);
    value |= (CSL_ECAP_ECCTL1_ECCTL2_REARM_MASK);
    HW_WR_REG32(temp_addr, value);
}

void ECAP_APWM_polarityConfig(uint32_t baseAddr, uint32_t flag)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCTL1_ECCTL2;
    HW_WR_FIELD32(temp_addr, CSL_ECAP_ECCTL1_ECCTL2_APWMPOL, flag);
}

void ECAP_counterControl(uint32_t baseAddr, uint32_t flag)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCTL1_ECCTL2;
    HW_WR_FIELD32(temp_addr, CSL_ECAP_ECCTL1_ECCTL2_TSCTRSTOP, flag);
}

void ECAP_syncInOutSelect(uint32_t baseAddr, uint32_t syncIn,
                         uint32_t syncOut)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCTL1_ECCTL2;
    HW_WR_FIELD32(temp_addr, CSL_ECAP_ECCTL1_ECCTL2_SYNCI_EN, syncIn);
    HW_WR_FIELD32(temp_addr, CSL_ECAP_ECCTL1_ECCTL2_SYNCO_SEL, syncOut);
}

void ECAP_APWM_captureConfig(uint32_t baseAddr, uint32_t compareVal,
                           uint32_t periodVal)
{
    HW_WR_REG32((baseAddr + CSL_ECAP_CAP1),periodVal);
    HW_WR_REG32((baseAddr + CSL_ECAP_CAP2),compareVal);
}

void ECAP_APWM_shadowCaptureConfig(uint32_t baseAddr, uint32_t compareVal,
                                 uint32_t periodVal)
{
    HW_WR_REG32((baseAddr + CSL_ECAP_CAP3),periodVal);
    HW_WR_REG32((baseAddr + CSL_ECAP_CAP4),compareVal);
}

void ECAP_counterPhaseValConfig(uint32_t baseAddr, uint32_t cntPhaseVal)
{
    HW_WR_REG32((baseAddr + CSL_ECAP_CTRPHS),cntPhaseVal);
}

void ECAP_globalIntrClear(uint32_t baseAddr)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCLR_ECFRC;
    uint32_t value = HW_RD_REG32(temp_addr);
    value  |= ((uint32_t)CSL_ECAP_ECCLR_ECFRC_INT_MASK);
    HW_WR_REG32(temp_addr, value);
}

void ECAP_intrEnable(uint32_t baseAddr, uint32_t flag)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECEINT_ECFLG;
    uint32_t value = HW_RD_REG32(temp_addr);
    value  |= (flag);
    HW_WR_REG32(temp_addr, value);
}

void ECAP_intrDisable(uint32_t baseAddr, uint32_t flag)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECEINT_ECFLG;
    uint32_t value = HW_RD_REG32(temp_addr);
    uint32_t value32;
    value32 = (value & ~flag);
    HW_WR_REG32(temp_addr, value32);
}

uint32_t ECAP_getIntrStatus(uint32_t baseAddr, uint32_t flag)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECEINT_ECFLG;
    uint32_t value = HW_RD_REG32(temp_addr);
    return (value & flag);
}

void ECAP_intrStatusClear(uint32_t baseAddr, uint32_t flag)
{
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECEINT_ECFLG;
    uint32_t value = HW_RD_REG32(temp_addr);
    temp_addr = baseAddr + CSL_ECAP_ECCLR_ECFRC;
    HW_WR_REG32(temp_addr,  value  & (flag));
}

void ECAP_captureInputSourceSelect(uint32_t baseAddr, uint8_t srcSelect)
{
    uint8_t value = 0;
    uint32_t temp_addr = baseAddr + CSL_ECAP_ECCTL0;
    /* Write 0 to select the input select as ECAPxINPUT[0] */
    value = ((uint8_t)(srcSelect));
    HW_WR_REG16(temp_addr, value);
}

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

#include <drivers/eqep.h>
#include <stdint.h>

//*****************************************************************************
//
// EQEP_setCompareConfig
//
//*****************************************************************************
void
EQEP_setCompareConfig(uint32_t base, uint16_t config, uint32_t compareValue,
                      uint16_t cycles)
{
    uint16_t regValue;

    //
    // Check the arguments.
    //
    DebugP_assert(cycles <= (CSL_EQEP_QPOSCTL_PCSPW_MASK + 1U));

    //
    // Set the compare match value
    //
    HW_WR_REG32(base + CSL_EQEP_QPOSCMP, compareValue);

    //
    // Set the shadow register settings and pulse width.
    //
    regValue = (config & (uint16_t)(CSL_EQEP_QPOSCTL_PCSHDW_MASK |
                  CSL_EQEP_QPOSCTL_PCSHDW_MASK)) | (cycles - 1U);

    HW_WR_REG16(base + CSL_EQEP_QPOSCTL,
        ((HW_RD_REG16(base + CSL_EQEP_QPOSCTL) & ~(CSL_EQEP_QPOSCTL_PCSPW_MASK |
        CSL_EQEP_QPOSCTL_PCSHDW_MASK | CSL_EQEP_QPOSCTL_PCSHDW_MASK)) |
        regValue));

    //
    // Set position compare sync-output mode.
    //
    regValue = config & (uint16_t)(CSL_EQEP_QDECCTL_SOEN_MASK |
               CSL_EQEP_QDECCTL_SPSEL_MASK);

    HW_WR_REG16(base + CSL_EQEP_QDECCTL,
    ((HW_RD_REG16(base + CSL_EQEP_QDECCTL) & ~(CSL_EQEP_QDECCTL_SOEN_MASK |
    CSL_EQEP_QDECCTL_SPSEL_MASK)) | regValue));
}

//*****************************************************************************
//
// EQEP_setInputPolarity
//
//*****************************************************************************
void
EQEP_setInputPolarity(uint32_t base, bool invertQEPA, bool invertQEPB,
                      bool invertIndex, bool invertStrobe)
{
    //
    // Configure QEPA signal
    //
    if(invertQEPA)
    {
        HW_WR_REG16(base + CSL_EQEP_QDECCTL,
            (HW_RD_REG16(base + CSL_EQEP_QDECCTL) | CSL_EQEP_QDECCTL_QAP_MASK));
    }
    else
    {
        HW_WR_REG16(base + CSL_EQEP_QDECCTL,
            (HW_RD_REG16(base + CSL_EQEP_QDECCTL) & ~CSL_EQEP_QDECCTL_QAP_MASK));
    }

    //
    // Configure QEPB signal
    //
    if(invertQEPB)
    {
        HW_WR_REG16(base + CSL_EQEP_QDECCTL,
            (HW_RD_REG16(base + CSL_EQEP_QDECCTL) | CSL_EQEP_QDECCTL_QBP_MASK));
    }
    else
    {
        HW_WR_REG16(base + CSL_EQEP_QDECCTL,
            (HW_RD_REG16(base + CSL_EQEP_QDECCTL) & ~CSL_EQEP_QDECCTL_QBP_MASK));
    }

    //
    // Configure index signal
    //
    if(invertIndex)
    {
        HW_WR_REG16(base + CSL_EQEP_QDECCTL,
            (HW_RD_REG16(base + CSL_EQEP_QDECCTL) | CSL_EQEP_QDECCTL_QIP_MASK));
    }
    else
    {
        HW_WR_REG16(base + CSL_EQEP_QDECCTL,
            (HW_RD_REG16(base + CSL_EQEP_QDECCTL) & ~CSL_EQEP_QDECCTL_QIP_MASK));
    }

    //
    // Configure strobe signal
    //
    if(invertStrobe)
    {
        HW_WR_REG16(base + CSL_EQEP_QDECCTL,
            (HW_RD_REG16(base + CSL_EQEP_QDECCTL) | CSL_EQEP_QDECCTL_QSP_MASK));
    }
    else
    {
        HW_WR_REG16(base + CSL_EQEP_QDECCTL,
            (HW_RD_REG16(base + CSL_EQEP_QDECCTL) & ~CSL_EQEP_QDECCTL_QSP_MASK));
    }
}

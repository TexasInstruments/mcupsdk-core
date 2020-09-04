/********************************************************************
 * Copyright (C) 2021 Texas Instruments Incorporated.
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

#include <drivers/cmpss.h>
#include <stdint.h>

//*****************************************************************************
//
// CMPSS_configFilterHigh
//
//*****************************************************************************
void
CMPSS_configFilterHigh(uint32_t base, uint16_t samplePrescale,
                       uint16_t sampleWindow, uint16_t threshold)
{
    uint16_t regValue;

    //
    // Check the arguments.
    //
    DebugP_assert(samplePrescale < 1024U);
    DebugP_assert((sampleWindow >= 1U) && (sampleWindow <= 32U));
    DebugP_assert((threshold - 1U) >= ((sampleWindow - 1U) / 2U));

    //
    // Shift the sample window and threshold values into the correct positions
    // and write them to the appropriate register.
    //
    regValue = ((sampleWindow - 1U) << CSL_CMPSSA_CTRIPHFILCTL_SAMPWIN_SHIFT) |
               ((threshold - 1U) << CSL_CMPSSA_CTRIPHFILCTL_THRESH_SHIFT);


    HW_WR_REG16((base + CSL_CMPSSA_CTRIPHFILCTL),
        (HW_RD_REG16(base + CSL_CMPSSA_CTRIPHFILCTL) &
        ~(CSL_CMPSSA_CTRIPHFILCTL_SAMPWIN_MASK |
        CSL_CMPSSA_CTRIPHFILCTL_THRESH_MASK)) | regValue);

    //
    // Set the filter sample clock prescale for the high comparator.
    //
    HW_WR_REG16((base + CSL_CMPSSA_CTRIPHFILCLKCTL), samplePrescale);

}

//*****************************************************************************
//
// CMPSS_configFilterLow
//
//*****************************************************************************
void
CMPSS_configFilterLow(uint32_t base, uint16_t samplePrescale,
                      uint16_t sampleWindow, uint16_t threshold)
{
    uint16_t regValue;

    //
    // Check the arguments.
    //
    DebugP_assert(samplePrescale < 1024U);
    DebugP_assert((sampleWindow >= 1U) && (sampleWindow <= 32U));
    DebugP_assert((threshold - 1U) >= ((sampleWindow - 1U) / 2U));

    //
    // Shift the sample window and threshold values into the correct positions
    // and write them to the appropriate register.
    //
    regValue = ((sampleWindow - 1U) << CSL_CMPSSA_CTRIPLFILCTL_SAMPWIN_SHIFT) |
               ((threshold - 1U) << CSL_CMPSSA_CTRIPLFILCTL_THRESH_SHIFT);


    HW_WR_REG16((base + CSL_CMPSSA_CTRIPLFILCTL),
        (HW_RD_REG16(base + CSL_CMPSSA_CTRIPLFILCTL) &
        ~(CSL_CMPSSA_CTRIPLFILCTL_SAMPWIN_MASK |
        CSL_CMPSSA_CTRIPLFILCTL_THRESH_MASK)) | regValue);

    //
    // Set the filter sample clock prescale for the low comparator.
    //
    HW_WR_REG16((base + CSL_CMPSSA_CTRIPLFILCLKCTL), samplePrescale);

}

//*****************************************************************************
//
// CMPSS_configLatchOnPWMSYNC
//
//*****************************************************************************
void
CMPSS_configLatchOnPWMSYNC(uint32_t base, bool highEnable, bool lowEnable)
{
    //
    // If the highEnable is true, set the bit that will enable PWMSYNC to reset
    // the high comparator digital filter latch. If not, clear the bit.
    //

    if(highEnable)
    {
        HW_WR_REG16((base + CSL_CMPSSA_COMPSTSCLR),
            HW_RD_REG16(base + CSL_CMPSSA_COMPSTSCLR) |
            CSL_CMPSSA_COMPSTSCLR_HSYNCCLREN_MASK);
    }
    else
    {
        HW_WR_REG16((base + CSL_CMPSSA_COMPSTSCLR),
            HW_RD_REG16(base + CSL_CMPSSA_COMPSTSCLR) &
            ~CSL_CMPSSA_COMPSTSCLR_HSYNCCLREN_MASK);
    }

    //
    // If the lowEnable is true, set the bit that will enable PWMSYNC to reset
    // the low comparator digital filter latch. If not, clear the bit.
    //
    if(lowEnable)
    {
        HW_WR_REG16((base + CSL_CMPSSA_COMPSTSCLR),
            HW_RD_REG16(base + CSL_CMPSSA_COMPSTSCLR) |
            CSL_CMPSSA_COMPSTSCLR_LSYNCCLREN_MASK);
    }
    else
    {
        HW_WR_REG16((base + CSL_CMPSSA_COMPSTSCLR),
            HW_RD_REG16(base + CSL_CMPSSA_COMPSTSCLR) &
            ~CSL_CMPSSA_COMPSTSCLR_LSYNCCLREN_MASK);
    }

}

//*****************************************************************************
//
// CMPSS_configRamp
//
//*****************************************************************************
void
CMPSS_configRamp(uint32_t base, uint16_t maxRampVal, uint16_t decrementVal,
                 uint16_t delayVal, uint16_t pwmSyncSrc, bool useRampValShdw)
{
    //
    // Check the arguments.
    //
    DebugP_assert(delayVal <= CSL_CMPSSA_RAMPDLYS_DELAY_MASK);
    DebugP_assert((pwmSyncSrc >= 1U) && (pwmSyncSrc <= 32U));

    //
    // Write the ramp generator source to the register
    //
    HW_WR_REG16((base + CSL_CMPSSA_COMPDACCTL),
        (HW_RD_REG16(base + CSL_CMPSSA_COMPDACCTL) &
        ~CSL_CMPSSA_COMPDACCTL_RAMPSOURCE_MASK) |
        ((uint16_t)(pwmSyncSrc - 1U) << CSL_CMPSSA_COMPDACCTL_RAMPSOURCE_SHIFT));

    HW_WR_REG16((base + CSL_CMPSSA_COMPDACCTL2),
        (HW_RD_REG16(base + CSL_CMPSSA_COMPDACCTL2) &
        ~CSL_CMPSSA_COMPDACCTL2_RAMPSOURCEUSEL_MASK) |
        ((((pwmSyncSrc - 1U)>>4) << CSL_CMPSSA_COMPDACCTL2_RAMPSOURCEUSEL_SHIFT) &
        CSL_CMPSSA_COMPDACCTL2_RAMPSOURCEUSEL_MASK));

    //
    // Set or clear the bit that determines from where the max ramp value
    // should be loaded.
    //
    if(useRampValShdw)
    {
        HW_WR_REG16((base + CSL_CMPSSA_COMPDACCTL),
            HW_RD_REG16(base + CSL_CMPSSA_COMPDACCTL) |
            CSL_CMPSSA_COMPDACCTL_RAMPLOADSEL_MASK);
    }
    else
    {
        HW_WR_REG16((base + CSL_CMPSSA_COMPDACCTL),
            HW_RD_REG16(base + CSL_CMPSSA_COMPDACCTL) &
            ~CSL_CMPSSA_COMPDACCTL_RAMPLOADSEL_MASK);
    }


    //
    // Write the maximum ramp value to the shadow register.
    //
    HW_WR_REG16((base + CSL_CMPSSA_RAMPMAXREFS), maxRampVal);

    //
    // Write the ramp decrement value to the shadow register.
    //
    HW_WR_REG16((base + CSL_CMPSSA_RAMPDECVALS), decrementVal);

    //
    // Write the ramp delay value to the shadow register.
    //
    HW_WR_REG16((base + CSL_CMPSSA_RAMPDLYS), delayVal);
}

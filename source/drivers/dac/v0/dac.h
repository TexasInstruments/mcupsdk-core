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
 *  \defgroup DRV_DAC_MODULE APIs for DAC
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the DAC module.
 *
 *  @{
 */

#ifndef DAC_V0_H_
#define DAC_V0_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! Header Files
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/cslr_dac.h>

//*****************************************************************************
//
// The following are defines for the reg parameter of the
// DAC_lockRegister() and DAC_isRegisterLocked() functions.
//
//*****************************************************************************
#define DAC_LOCK_CONTROL  (0x1U)  //!< Lock the control register
#define DAC_LOCK_SHADOW  (0x2U)  //!< Lock the shadow value register
#define DAC_LOCK_OUTPUT  (0x4U)  //!< Lock the output enable register

//*****************************************************************************
//
//! Values that can be passed to DAC_setReferenceVoltage() as the \e source
//! parameter.
//
//*****************************************************************************
typedef enum
{
    DAC_REF_VREF = 0,  //!< VREF reference voltage (external)
    DAC_REF_VDDA = 1  //!< VDDA reference voltage (internal)
}DAC_ReferenceVoltage;

//*****************************************************************************
//
//! Values that can be passed to DAC_setLoadMode() as the \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    DAC_LOAD_SYSCLK = 0,  //!< Load on next SYSCLK
    DAC_LOAD_PWMSYNC = 4  //!< Load on next PWMSYNC specified by SYNCSEL
}DAC_LoadMode;

//*****************************************************************************
//
// Defines used by the driver
//
//*****************************************************************************
#define DAC_REG_BYTE_MASK  (0xFFU)  //!< Register Byte Mask
#define DAC_LOCK_KEY  (0xA000U)  //!< DAC Lock Key

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! Get the DAC Revision value
//!
//! \param base is the DAC module base address
//!
//! This function gets the DAC revision value.
//!
//! \return Returns the DAC revision value.
//
//*****************************************************************************
static inline uint16_t
DAC_getRevision(uint32_t base)
{
    //
    // Get the revision value.
    //
    return(HW_RD_REG16(base + CSL_DAC_DACREV) & CSL_DAC_DACREV_REV_MASK);
}

//*****************************************************************************
//
//! Sets the DAC Reference Voltage
//!
//! \param base is the DAC module base address
//! \param source is the selected reference voltage
//!
//! This function sets the DAC reference voltage.
//!
//! The \e source parameter can have one of two values:
//! - \b DAC_REF_VREF      - The VREF reference voltage (external)
//! - \b DAC_REF_VDDA - The VDDA reference voltage (internal)
//!
//! \return None.
//
//*****************************************************************************
static inline void
DAC_setReferenceVoltage(uint32_t base, DAC_ReferenceVoltage source)
{
    //
    // Set the reference  voltage
    //
    HW_WR_REG16(base + CSL_DAC_DACCTL,
        ((HW_RD_REG16(base + CSL_DAC_DACCTL) &
        ~CSL_DAC_DACCTL_DACREFSEL_MASK) | (uint16_t)source));
}

//*****************************************************************************
//
//! Sets the DAC Load Mode
//!
//! \param base is the DAC module base address
//! \param mode is the selected load mode
//!
//! This function sets the DAC load mode.
//!
//! The \e mode parameter can have one of two values:
//! - \b DAC_LOAD_SYSCLK   - Load on next SYSCLK
//! - \b DAC_LOAD_PWMSYNC  - Load on next PWMSYNC specified by SYNCSEL
//!
//! \return None.
//
//*****************************************************************************
static inline void
DAC_setLoadMode(uint32_t base, DAC_LoadMode mode)
{
    //
    // Set the load mode
    //
    HW_WR_REG16(base + CSL_DAC_DACCTL,
        ((HW_RD_REG16(base + CSL_DAC_DACCTL) &
        ~CSL_DAC_DACCTL_LOADMODE_MASK) | (uint16_t)mode));
}

//*****************************************************************************
//
//! Sets the DAC PWMSYNC Signal
//!
//! \param base is the DAC module base address
//! \param signal is the selected PWM signal
//!
//! This function sets the DAC PWMSYNC signal.
//!
//! The \e signal parameter must be set to a number that represents the PWM
//! signal that will    be set. For instance, passing 2 into \e signal will
//! select PWM sync signal 2.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DAC_setPWMSyncSignal(uint32_t base, uint16_t signal)
{
    //
    // Check the arguments.
    //
    DebugP_assert((signal > 0U) && (signal < 33U));

    //
    // Set the PWM sync signal
    //
    HW_WR_REG16(base + CSL_DAC_DACCTL,
        ((HW_RD_REG16(base + CSL_DAC_DACCTL) &
        ~CSL_DAC_DACCTL_SYNCSEL_MASK) |
        ((uint16_t)(signal - 1U) << CSL_DAC_DACCTL_SYNCSEL_SHIFT)));
}

//*****************************************************************************
//
//! Get the DAC Active Output Value
//!
//! \param base is the DAC module base address
//!
//! This function gets the DAC active output value.
//!
//! \return Returns the DAC active output value.
//
//*****************************************************************************
static inline uint16_t
DAC_getActiveValue(uint32_t base)
{
    //
    // Get the active value
    //
    return(HW_RD_REG16(base + CSL_DAC_DACVALA) & CSL_DAC_DACVALA_DACVALA_MASK);
}

//*****************************************************************************
//
//! Set the DAC Shadow Output Value
//!
//! \param base is the DAC module base address
//! \param value is the 12-bit code to be loaded into the active value register
//!
//! This function sets the DAC shadow output value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DAC_setShadowValue(uint32_t base, uint16_t value)
{
    //
    // Check the arguments.
    //
    DebugP_assert(value <= CSL_DAC_DACVALS_DACVALS_MASK);

    //
    // Set the shadow value
    //
    HW_WR_REG16(base + CSL_DAC_DACVALS,
        ((HW_RD_REG16(base + CSL_DAC_DACVALS) &
        ~CSL_DAC_DACVALS_DACVALS_MASK) |
        (uint16_t)(value & CSL_DAC_DACVALS_DACVALS_MASK)));
}

//*****************************************************************************
//
//! Get the DAC Shadow Output Value
//!
//! \param base is the DAC module base address
//!
//! This function gets the DAC shadow output value.
//!
//! \return Returns the DAC shadow output value.
//
//*****************************************************************************
static inline uint16_t
DAC_getShadowValue(uint32_t base)
{
    //
    // Get the shadow value
    //
    return(HW_RD_REG16(base + CSL_DAC_DACVALS) & CSL_DAC_DACVALS_DACVALS_MASK);
}

//*****************************************************************************
//
//! Enable the DAC Output
//!
//! \param base is the DAC module base address
//!
//! This function enables the DAC output.
//!
//! \note A delay is required after enabling the DAC. Further details
//! regarding the exact delay time length can be found in the device datasheet.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DAC_enableOutput(uint32_t base)
{
    //
    // Enable the output
    //
    HW_WR_REG16(base + CSL_DAC_DACOUTEN,
        HW_RD_REG16(base + CSL_DAC_DACOUTEN) | CSL_DAC_DACOUTEN_DACOUTEN_MASK);
}

//*****************************************************************************
//
//! Disable the DAC Output
//!
//! \param base is the DAC module base address
//!
//! This function disables the DAC output.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DAC_disableOutput(uint32_t base)
{
    //
    // Disable the output
    //
    HW_WR_REG16(base + CSL_DAC_DACOUTEN,
        HW_RD_REG16(base + CSL_DAC_DACOUTEN) &
        ~CSL_DAC_DACOUTEN_DACOUTEN_MASK);
}

//*****************************************************************************
//
//! Set DAC Offset Trim
//!
//! \param base is the DAC module base address
//! \param offset is the specified value for the offset trim
//!
//! This function sets the DAC offset trim. The \e offset value should be a
//! signed number in the range of -128 to 127.
//!
//! \note The offset should not be modified unless specifically indicated by
//! TI Errata or other documentation. Modifying the offset value could cause
//! this module to operate outside of the datasheet specifications.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DAC_setOffsetTrim(uint32_t base, int16_t offset)
{
    //
    // Check the arguments.
    //
    DebugP_assert((offset > -129) && (offset < 128));

    //
    // Set the offset trim value
    //
    HW_WR_REG16(base + CSL_DAC_DACTRIM,
        ((HW_RD_REG16(base + CSL_DAC_DACTRIM) &
        ~CSL_DAC_DACTRIM_OFFSET_TRIM_MASK) | (int16_t)offset));
}

//*****************************************************************************
//
//! Get DAC Offset Trim
//!
//! \param base is the DAC module base address
//!
//! This function gets the DAC offset trim value.
//!
//! \return None.
//
//*****************************************************************************
static inline int16_t
DAC_getOffsetTrim(uint32_t base)
{
    uint16_t value;

    //
    // Get the sign-extended offset trim value
    //
    value = (HW_RD_REG16(base + CSL_DAC_DACTRIM) &
        CSL_DAC_DACTRIM_OFFSET_TRIM_MASK);
    value = ((value & (uint16_t)DAC_REG_BYTE_MASK) ^ (uint16_t)0x80) -
            (uint16_t)0x80;

    return((int16_t)value);
}

//*****************************************************************************
//
//! Lock write-access to DAC Register
//!
//! \param base is the DAC module base address
//! \param reg is the selected DAC registers
//!
//! This function locks the write-access to the specified DAC register. Only a
//! system reset can unlock the register once locked.
//!
//! The \e reg parameter can be an ORed combination of any of the following
//! values:
//! - \b DAC_LOCK_CONTROL  - Lock the DAC control register
//! - \b DAC_LOCK_SHADOW   - Lock the DAC shadow value register
//! - \b DAC_LOCK_OUTPUT   - Lock the DAC output enable/disable register
//!
//! \return None.
//
//*****************************************************************************
static inline void
DAC_lockRegister(uint32_t base, uint16_t reg)
{
    //
    // Check the arguments.
    //
    DebugP_assert((reg & ~(DAC_LOCK_CONTROL | DAC_LOCK_SHADOW |
                    DAC_LOCK_OUTPUT)) == 0U);

    //
    // Lock the specified registers
    //
    HW_WR_REG16(base + CSL_DAC_DACLOCK,
        HW_RD_REG16(base + CSL_DAC_DACLOCK) |
        (CSL_DAC_DACLOCK_KEY_MASK | reg));
}

//*****************************************************************************
//
//! Check if DAC Register is locked
//!
//! \param base is the DAC module base address
//! \param reg is the selected DAC register locks to check
//!
//! This function checks if write-access has been locked on the specified DAC
//! register.
//!
//! The \e reg parameter can be an ORed combination of any of the following
//! values:
//! - \b DAC_LOCK_CONTROL  - Lock the DAC control register
//! - \b DAC_LOCK_SHADOW   - Lock the DAC shadow value register
//! - \b DAC_LOCK_OUTPUT   - Lock the DAC output enable/disable register
//!
//! \return Returns \b true if any of the registers specified are locked, and
//! \b false if all specified registers aren't locked.
//
//*****************************************************************************
static inline bool
DAC_isRegisterLocked(uint32_t base, uint16_t reg)
{
    //
    // Check the arguments.
    //
    DebugP_assert((reg & ~(DAC_LOCK_CONTROL | DAC_LOCK_SHADOW |
                    DAC_LOCK_OUTPUT)) == 0U);

    //
    // Return the lock status on the specified registers
    //
    return(((HW_RD_REG16(base + CSL_DAC_DACLOCK) & reg) != 0U));
}

//*****************************************************************************
//
//! Tune DAC Offset Trim
//!
//! \param base is the DAC module base address
//! \param referenceVoltage is the reference voltage the DAC
//! module is operating at.
//!
//! This function adjusts/tunes the DAC offset trim. The \e referenceVoltage
//! value should be a floating point number in the range specified in the
//! device data manual.
//!
//! \note Use this function to adjust the DAC offset trim if operating
//! at a reference voltage other than 2.5v. Since this function modifies
//! the DAC offset trim register, it should only be called once after
//! Device_cal. If it is called multiple times after Device_cal, the offset
//! value scaled would be the wrong value.
//!
//! \return None.
//
//*****************************************************************************
extern void
DAC_tuneOffsetTrim(uint32_t base, Float32 referenceVoltage);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // DAC_V0_H_

/******************************************************************************
 *
 * Copyright 2017-2018 Cadence Design Systems, Inc.
 *
 ******************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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
 *
 ******************************************************************************
 * cps_drv_lpddr4.h
 * Interface for the Register Accaess Layer of Cadence Platform Service (CPS)
 *****************************************************************************/

#ifndef CPS_DRV_H_
#define CPS_DRV_H_

#ifdef __cplusplus
extern "C"
{
#endif


#ifdef DEMO_TB
#include <cdn_demo.h>
#else
#include "cps.h"
#endif

// parasoft-begin-suppress MISRA2012-DIR-4_9-4 "function-like macro"
// parasoft-begin-suppress MISRA2012-RULE-20_10-4 "## preprocessor operator"
// parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions"

/**
 *  \brief    Read a 32-bit value from memory.
 *  \param    reg   address of the memory mapped hardware register
 *  \return   the value at the given address
 */
#define CPS_REG_READ(reg) (CPS_RegRead((volatile uint32_t*)(reg)))

/**
 *  \brief   Write a 32-bit address value to memory.
 *  \param   reg     address of the memory mapped hardware register
 *  \param   value   unsigned 32-bit value to write
 */
#define CPS_REG_WRITE(reg, value) (CPS_RegWrite((volatile uint32_t*)(reg), (uint32_t)(value)))

/**
 *  \brief    Subtitue the value of fld macro and concatinate with required string
 *  \param    fld         field name
 */
#define CPS_FLD_MASK(fld)  (fld##_MASK)
#define CPS_FLD_SHIFT(fld) (fld##_SHIFT)
#define CPS_FLD_WIDTH(fld) (fld##_WIDTH)
#define CPS_FLD_WOCLR(fld) (fld##_WOCLR)
#define CPS_FLD_WOSET(fld) (fld##_WOSET)

/**
 *  \brief    Read a value of bit-field from the register value.
 *  \param    reg         register name
 *  \param    fld         field name
 *  \param    reg_value   register value
 *  \return   bit-field value
 */
#define CPS_FLD_READ(fld, reg_value) (CPS_FldRead((uint32_t)(CPS_FLD_MASK(fld)),  \
                                                       (uint32_t)(CPS_FLD_SHIFT(fld)), \
                                                       (uint32_t)(reg_value)))


/**
 *  \brief    Write a value of the bit-field into the register value.
 *  \param    reg         register name
 *  \param    fld         field name
 *  \param    reg_value   register value
 *  \param    value       value to be written to bit-field
 *  \return   modified register value
 */
#define CPS_FLD_WRITE(fld, reg_value, value) (CPS_FldWrite((uint32_t)(CPS_FLD_MASK(fld)),  \
                                                           (uint32_t)(CPS_FLD_SHIFT(fld)), \
                                                           (uint32_t)(reg_value), (uint32_t)(value)))


/**
 *  \brief    Set bit within the register value.
 *  \param    reg         register name
 *  \param    fld         field name
 *  \param    reg_value   register value
 *  \return   modified register value
 */
#define CPS_FLD_SET(fld, reg_value) (CPS_FldSet((uint32_t)(CPS_FLD_WIDTH(fld)), \
                                                (uint32_t)(CPS_FLD_MASK(fld)),  \
                                                (uint32_t)(CPS_FLD_WOCLR(fld)), \
                                                (uint32_t)(reg_value)))

#ifdef CLR_USED
/**
 *  \brief    Clear bit within the register value.
 *  \param    reg         register name
 *  \param    fld         field name
 *  \param    reg_value   register value
 *  \return   modified register value
 */
#define CPS_FLD_CLEAR(reg, fld, reg_value) (CPS_FldClear((uint32_t)(CPS_FLD_WIDTH(fld)), \
                                                         (uint32_t)(CPS_FLD_MASK(fld)),  \
                                                         (uint32_t)(CPS_FLD_WOSET(fld)), \
                                                         (uint32_t)(CPS_FLD_WOCLR(fld)), \
                                                         (uint32_t)(reg_value)))

#endif
/**
 *  \brief    Read a 32-bit value from memory.
 *  \param    reg   address of the memory mapped hardware register
 *  \return   the value at the given address
 */
static inline uint32_t CPS_RegRead(volatile uint32_t* reg);
static inline uint32_t CPS_RegRead(volatile uint32_t* reg)
{
    /* Read and return the value of given address */
    return (CPS_UncachedRead32(reg));
}

/**
 *  \brief   Write a 32-bit address value to memory.
 *  \param   reg     address of the memory mapped hardware register
 *  \param   value   unsigned 32-bit value to write
 */
static inline void CPS_RegWrite(volatile uint32_t* reg, uint32_t value);
static inline void CPS_RegWrite(volatile uint32_t* reg, uint32_t value)
{
    /* Write the value to given address */
    CPS_UncachedWrite32(reg, value);
}

/**
 *  \brief    Read a value of bit-field from the register value.
 *  \param    mask        mask for the bit-field
 *  \param    shift       bit-field shift from LSB
 *  \param    reg_value   register value
 *  \return   bit-field value
 */
static inline uint32_t CPS_FldRead(uint32_t mask, uint32_t shift, uint32_t reg_value);
static inline uint32_t CPS_FldRead(uint32_t mask, uint32_t shift, uint32_t reg_value)
{
    // parasoft-begin-suppress MISRA2012-RULE-12_2-2 "shift ranges"
    uint32_t result = (reg_value & mask) >> shift;
    // parasoft-end-suppress MISRA2012-RULE-12_2-2

    return (result);
}

/**
 *  \brief    Write a value of the bit-field into the register value.
 *  \param    mask        mask for the bit-field
 *  \param    shift       bit-field shift from LSB
 *  \param    reg_value   register value
 *  \param    value       value to be written to bit-field
 *  \return   modified register value
 */
static inline uint32_t CPS_FldWrite(uint32_t mask, uint32_t shift, uint32_t reg_value, uint32_t value);
static inline uint32_t CPS_FldWrite(uint32_t mask, uint32_t shift, uint32_t reg_value, uint32_t value)
{
    // parasoft-begin-suppress MISRA2012-RULE-12_2-2 "shift ranges"
    uint32_t new_value = (value << shift) & mask;
    // parasoft-end-suppress MISRA2012-RULE-12_2-2

    new_value = (reg_value & ~mask) | new_value;
    return (new_value);
}

/**
 *  \brief    Set bit within the register value.
 *  \param    width       width of the bit-field
 *  \param    mask        mask for the bit-field
 *  \param    is_woclr    is bit-field has 'write one to clear' flag set
 *  \param    reg_value   register value
 *  \return   modified register value
 */
static inline uint32_t CPS_FldSet(uint32_t width, uint32_t mask, uint32_t is_woclr, uint32_t reg_value);
static inline uint32_t CPS_FldSet(uint32_t width, uint32_t mask, uint32_t is_woclr, uint32_t reg_value)
{
    uint32_t new_value = reg_value;
    /* Confirm the field to be bit and not write to clear type */
    if ((width == 1U) && (is_woclr == 0U)) {
        new_value |= mask;
    }

    return (new_value);
}

#ifdef CLR_USED
/**
 *  \brief    Clear bit within the register value.
 *
 *  \param    width       Width of the bit-field.
 *  \param    mask        Mask for the bit-field.
 *  \param    is_woset    Is bit-field has 'write one to set' flag set.
 *  \param    is_woclr    Is bit-field has 'write one to clear' flag set.
 *  \param    reg_value   Register value.
 *
 *  \return   Modified register value.
 */
static inline uint32_t CPS_FldClear(uint32_t width, uint32_t mask, uint32_t is_woset, uint32_t is_woclr,  uint32_t reg_value);
static inline uint32_t CPS_FldClear(uint32_t width, uint32_t mask, uint32_t is_woset, uint32_t is_woclr,  uint32_t reg_value)
{
    uint32_t new_value = reg_value;
    /* Confirm the field to be bit and not write to set type */
    if ((width == 1U) && (is_woset == 0U)) {
        new_value = (new_value & ~mask) | ((is_woclr != 0U) ? mask : 0U);
    }

    return (new_value);
}
#endif /* CLR_USED */

// parasoft-end-suppress MISRA2012-RULE-20_10-4
// parasoft-end-suppress MISRA2012-DIR-4_9-4
// parasoft-end-suppress METRICS-36-3


#ifdef __cplusplus
}
#endif

#endif /* CPS_DRV_H_ */


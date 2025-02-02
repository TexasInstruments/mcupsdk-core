/*  ============================================================================
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 *
*/
/** ============================================================================
 *   \file  sdlr.h
 *
 *   \brief  This file contains the macro definations for Register layer
 *
 */

/* Register layer central -- contains field-manipulation macro definitions */

#ifndef SDLR_H
#define SDLR_H
#include <stdint.h>

/* the "expression" macros */

/* the Field MaKe macro */
#define SDL_FMK(PER_REG_FIELD, val)                                         \
    (((val) << SDL_##PER_REG_FIELD##_SHIFT) & SDL_##PER_REG_FIELD##_MASK)

/* the Field EXTract macro */
#define SDL_FEXT(reg, PER_REG_FIELD)                                        \
    (((reg) & SDL_##PER_REG_FIELD##_MASK) >> SDL_##PER_REG_FIELD##_SHIFT)

/* the Field INSert macro */
#define SDL_FINS(reg, PER_REG_FIELD, val)                                   \
    ((reg) = ((reg) & ~SDL_##PER_REG_FIELD##_MASK)                          \
      | (((val) << SDL_##PER_REG_FIELD##_SHIFT) & SDL_##PER_REG_FIELD##_MASK))


/* the "token" macros */

/* the Field MaKe (Token) macro */
#define SDL_FMKT(PER_REG_FIELD, TOKEN)                                      \
    (((SDL_##PER_REG_FIELD##_##TOKEN) << SDL_##PER_REG_FIELD##_SHIFT) & SDL_##PER_REG_FIELD##_MASK)

/* the Field INSert (Token) macro */
#define SDL_FINST(reg, PER_REG_FIELD, TOKEN)                                \
    ((reg) = ((reg) & ~SDL_##PER_REG_FIELD##_MASK)                          \
      | (((SDL_##PER_REG_FIELD##_##TOKEN) << SDL_##PER_REG_FIELD##_SHIFT) & SDL_##PER_REG_FIELD##_MASK))


/* the "raw" macros */
#define kw_val (1u)                                             \

/* the Field MaKe (Raw) macro */
#define SDL_FMKR(msb, lsb, val)                                             \
    (((val) & ((((uint32_t)1U) << ((msb) - (lsb) + ((uint32_t)1U))) - ((uint32_t)1U))) << (lsb))

/* the Field EXTract (Raw) macro */
#define SDL_FEXTR(reg, msb, lsb)                                            \
    (((reg) >> (lsb)) & ((((uint32_t)1U) << ((msb) - (lsb) + ((uint32_t)1U))) - ((uint32_t)1U)))

/* the Field INSert (Raw) macro */
#define SDL_FINSR(reg, msb, lsb, val)                                       \
    ((reg) = ((reg) & (~(((((uint32_t)1U) << ((msb) - (lsb) + ((uint32_t)1U))) - ((uint32_t)1U)) << (lsb))))         \
    | SDL_FMKR((msb), (lsb), (val)))

/**
 *  \brief   This macro reads a pointer from a provided address
 *           and returns the value. The access type is implicit based on
 *           the pointer declaration.
 *
 *  \param   p      Pointer to the memory or hardware register.
 *
 *  \return  Value read from the pointer.
 */
#define SDL_REG_RD(p)           (*(p))

/**
 *  \brief   This macro writes a value to the pointer. The access type is
 *           implicit based on the pointer declaration.
 *
 *  \param   p      Pointer to the memory or hardware register.
 *  \param   v      Value which has to be written to the pointer.
 */
#define SDL_REG_WR(p, v)        (*(p) = (v))

/**
 *  \brief   This macro reads a 32-bit value from a hardware register
 *           and returns the value.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *
 *  \return  Unsigned 32-bit value read from a register.
 */
#define SDL_REG32_RD(p)         (SDL_REG32_RD_RAW((volatile uint32_t *) (p)))

/**
 *  \brief   This macro writes a 32-bit value to a hardware register.
 *
 *  \param   p       Address of the memory mapped hardware register.
 *  \param   v       Unsigned 32-bit value which has to be written to the
 *                   register.
 */
#define SDL_REG32_WR(p, v)      (SDL_REG32_WR_RAW( \
                                    (volatile uint32_t *) (p), (uint32_t) (v)))

/**
 *  \brief   This macro reads a 16-bit value from a hardware register
 *           and returns the value.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *
 *  \return  Unsigned 16-bit value read from a register.
 */
#define SDL_REG16_RD(p)         (SDL_REG16_RD_RAW((volatile uint16_t *) (p)))

/**
 *  \brief   This macro writes a 16-bit value to a hardware register.
 *
 *  \param   p       Address of the memory mapped hardware register.
 *  \param   v       Unsigned 16-bit value which has to be written to the
 *                   register.
 */
#define SDL_REG16_WR(p, v)      (SDL_REG16_WR_RAW( \
                                    (volatile uint16_t *) (p), (uint16_t) (v)))

/**
 *  \brief   This macro reads a 8-bit value from a hardware register
 *           and returns the value.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *
 *  \return  Unsigned 8-bit value read from a register.
 */
#define SDL_REG8_RD(p)          (SDL_REG8_RD_RAW((volatile uint8_t *) (p)))

/**
 *  \brief   This macro writes a 8-bit value to a hardware register.
 *
 *  \param   p       Address of the memory mapped hardware register.
 *  \param   v       Unsigned 8-bit value which has to be written to the
 *                   register.
 */
#define SDL_REG8_WR(p, v)       (SDL_REG8_WR_RAW( \
                                    (volatile uint8_t *) (p), (uint8_t) (v)))

/**
 *  \brief   This macro reads a 32-bit value from a hardware register
 *           with an offset and returns the value.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   off    Offset in bytes.
 *
 *  \return  Unsigned 32-bit value read from a register.
 */
#define SDL_REG32_RD_OFF(p, off)    (SDL_REG32_RD_OFF_RAW( \
                                        (volatile uint32_t *) (p), \
                                        (uint32_t) (off)))

/**
 *  \brief   This macro writes a 32-bit value to a hardware register with
 *           an offset.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   off    Offset in bytes.
 *  \param   v      Unsigned 32-bit value which has to be written to the
 *                  register.
 */
#define SDL_REG32_WR_OFF(p, off, v) (SDL_REG32_WR_OFF_RAW( \
                                        (volatile uint32_t *) (p), \
                                        (uint32_t) (off), \
                                        (uint32_t) (v)))

/**
 *  \brief This macro calls read field API for 32 bit register. It also
 *         frames the mask and shift from register field macro.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   fld    Peripheral register bit field name, from which
 *                  specified bit-field value has to be read.
 *
 *  \return Value of the bit-field (absolute value - shifted to LSB position)
 */
#define SDL_REG32_FEXT(p, fld)      (SDL_REG32_FEXT_RAW( \
                                        ((volatile uint32_t *) (p)), \
                                        ((uint32_t) SDL_##fld##_MASK), \
                                        ((uint32_t) SDL_##fld##_SHIFT)))

/**
 *  \brief This macro calls read field API for 16 bit register. It also
 *         frames the mask and shift from register field macro.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   fld    Peripheral register bit field name, from which
 *                  specified bit-field value has to be read.
 *
 *  \return Value of the bit-field (absolute value - shifted to LSB position)
 */
#define SDL_REG16_FEXT(p, fld)      (SDL_REG16_FEXT_RAW( \
                                        ((volatile uint16_t *) (p)), \
                                        ((uint16_t) SDL_##fld##_MASK), \
                                        ((uint32_t) SDL_##fld##_SHIFT)))

/**
 *  \brief This macro calls read field API for 8 bit register. It also
 *         frames the mask and shift from register field macro.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   fld    Peripheral register bit field name, from which
 *                  specified bit-field value has to be read.
 *
 *  \return Value of the bit-field (absolute value - shifted to LSB position)
 */
#define SDL_REG8_FEXT(p, fld)       (SDL_REG8_FEXT_RAW( \
                                        ((volatile uint8_t *) (p)), \
                                        ((uint8_t) SDL_##fld##_MASK), \
                                        ((uint32_t) SDL_##fld##_SHIFT)))

/**
 *  \brief This macro calls read-modify-write API for 32 bit register. It also
 *         frames the mask and shift from register field macro.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   fld    Peripheral register bit field name, from which
 *                  specified bit-field value has to be set.
 *  \param   v      Value of the field which has to be set.
 */
#define SDL_REG32_FINS(p, fld, v)   (SDL_REG32_FINS_RAW( \
                                        ((volatile uint32_t *) (p)), \
                                        ((uint32_t) SDL_##fld##_MASK), \
                                        ((uint32_t) SDL_##fld##_SHIFT), \
                                        ((uint32_t) v)))

/**
 *  \brief This macro calls read-modify-write API for 16 bit register. It also
 *         frames the mask and shift from register field macro.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   fld    Peripheral register bit field name, from which
 *                  specified bit-field value has to be set.
 *  \param   v      Value of the field which has to be set.
 */
#define SDL_REG16_FINS(p, fld, v)   (SDL_REG16_FINS_RAW( \
                                        ((volatile uint16_t *) (p)), \
                                        ((uint16_t) SDL_##fld##_MASK), \
                                        ((uint32_t) SDL_##fld##_SHIFT), \
                                        ((uint16_t) v)))

/**
 *  \brief This macro calls read-modify-write API for 8 bit register. It also
 *         frames the mask and shift from register field macro.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   fld    Peripheral register bit field name, from which
 *                  specified bit-field value has to be set.
 *  \param   v      Value of the field which has to be set.
 */
#define SDL_REG8_FINS(p, fld, v)    (SDL_REG8_FINS_RAW( \
                                        ((volatile uint8_t *) (p)), \
                                        ((uint8_t) SDL_##fld##_MASK), \
                                        ((uint32_t) SDL_##fld##_SHIFT), \
                                        ((uint8_t) v)))

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief   This function reads a 32-bit value from a hardware register
 *           and returns the value.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *
 *  \return  Unsigned 32-bit value read from a register.
 */
static inline uint32_t SDL_REG32_RD_RAW(volatile const uint32_t * const p);
static inline uint32_t SDL_REG32_RD_RAW(volatile const uint32_t * const p)
{
    return (*p);
}

/**
 *  \brief   This function writes a 32-bit value to a hardware register.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   v      Unsigned 32-bit value which has to be written to the
 *                  register.
 */
static inline void SDL_REG32_WR_RAW(volatile uint32_t * const p, uint32_t v);
static inline void SDL_REG32_WR_RAW(volatile uint32_t * const p, uint32_t v)
{
    *p = v;
    return;
}

/**
 *  \brief   This function reads a 16-bit value from a hardware register
 *           and returns the value.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *
 *  \return  Unsigned 16-bit value read from a register.
 */
static inline uint16_t SDL_REG16_RD_RAW(volatile const uint16_t * const p);
static inline uint16_t SDL_REG16_RD_RAW(volatile const uint16_t * const p)
{
    return (*p);
}

/**
 *  \brief   This function writes a 16-bit value to a hardware register.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   v      Unsigned 16-bit value which has to be written to the
 *                  register.
 */
static inline void SDL_REG16_WR_RAW(volatile uint16_t * const p, uint16_t v);
static inline void SDL_REG16_WR_RAW(volatile uint16_t * const p, uint16_t v)
{
    *p = v;
    return;
}

/**
 *  \brief   This function reads a 8-bit value from a hardware register
 *           and returns the value.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *
 *  \return  Unsigned 8-bit value read from a register.
 */
static inline uint8_t SDL_REG8_RD_RAW(volatile const uint8_t * const p);
static inline uint8_t SDL_REG8_RD_RAW(volatile const uint8_t * const p)
{
    return (*p);
}

/**
 *  \brief   This function writes a 8-bit value to a hardware register.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   v      Unsigned 8-bit value which has to be written to the
 *                  register.
 */
static inline void SDL_REG8_WR_RAW(volatile uint8_t * const p, uint8_t v);
static inline void SDL_REG8_WR_RAW(volatile uint8_t * const p, uint8_t v)
{
    *p = v;
    return;
}

/**
 *  \brief   This function reads a 32-bit value from a hardware register
 *           with an offset and returns the value.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   off    Offset in bytes.
 *
 *  \return  Unsigned 32-bit value read from a register.
 */
static inline uint32_t SDL_REG32_RD_OFF_RAW(volatile const uint32_t *p, uint32_t off);
static inline uint32_t SDL_REG32_RD_OFF_RAW(volatile const uint32_t *p, uint32_t off)
{
    uintptr_t pOff = ((uintptr_t) p) + off;
    return (*(volatile const uint32_t *)(pOff));
}

/**
 *  \brief   This function writes a 32-bit value to a hardware register with
 *           an offset.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   off    Offset in bytes.
 *  \param   v      Unsigned 32-bit value which has to be written to the
 *                  register.
 */
static inline void SDL_REG32_WR_OFF_RAW(volatile uint32_t *const p,
                                        uint32_t off,
                                        uint32_t v);
static inline void SDL_REG32_WR_OFF_RAW(volatile uint32_t *const p,
                                        uint32_t off,
                                        uint32_t v)
{
    uintptr_t pOff = ((uintptr_t) p) + off;
    (*(volatile uint32_t *)(pOff)) = (v);
    return;
}

/**
 *  \brief   This function reads a 32 bit register, masks specific set of bits
 *           and returns the left shifted value.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   mask   Mask for the bit field.
 *  \param   shift  Bit field shift from LSB.
 *
 *  \return  Bit-field value (absolute value - shifted to LSB position)
 */
static inline uint32_t SDL_REG32_FEXT_RAW(volatile const uint32_t * const p,
                                          uint32_t mask,
                                          uint32_t shift);
static inline uint32_t SDL_REG32_FEXT_RAW(volatile const uint32_t * const p,
                                          uint32_t mask,
                                          uint32_t shift)
{
    uint32_t regVal = SDL_REG32_RD_RAW(p);
    regVal = (regVal & mask) >> shift;
    return (regVal);
}

/**
 *  \brief   This function reads a 16 bit register, masks specific set of bits
 *           and returns the left shifted value.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   mask   Mask for the bit field.
 *  \param   shift  Bit field shift from LSB.
 *
 *  \return  Bit-field value (absolute value - shifted to LSB position)
 */
static inline uint16_t SDL_REG16_FEXT_RAW(volatile const uint16_t * const p,
                                          uint16_t mask,
                                          uint32_t shift);
static inline uint16_t SDL_REG16_FEXT_RAW(volatile const uint16_t * const p,
                                          uint16_t mask,
                                          uint32_t shift)
{
    uint16_t regVal = SDL_REG16_RD_RAW(p);
    regVal = (regVal & mask) >> shift;
    return (regVal);
}

/**
 *  \brief   This function reads a 8 bit register, masks specific set of bits
 *           and returns the left shifted value.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   mask   Mask for the bit field.
 *  \param   shift  Bit field shift from LSB.
 *
 *  \return  Bit-field value (absolute value - shifted to LSB position)
 */
static inline uint8_t SDL_REG8_FEXT_RAW(volatile const uint8_t * const p,
                                        uint8_t mask,
                                        uint32_t shift);
static inline uint8_t SDL_REG8_FEXT_RAW(volatile const uint8_t * const p,
                                        uint8_t mask,
                                        uint32_t shift)
{
    uint8_t regVal = SDL_REG8_RD_RAW(p);
    regVal = (regVal & mask) >> shift;
    return (regVal);
}

/**
 *  \brief   This function reads a 32 bit register, modifies specific set of
 *           bits and writes back to the register.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   mask   Mask for the bit field.
 *  \param   shift  Bit field shift from LSB.
 *  \param   v      Value to be written to bit-field.
 */
static inline void SDL_REG32_FINS_RAW(volatile uint32_t * const p,
                                      uint32_t mask,
                                      uint32_t shift,
                                      uint32_t v);
static inline void SDL_REG32_FINS_RAW(volatile uint32_t * const p,
                                      uint32_t mask,
                                      uint32_t shift,
                                      uint32_t v)
{
    uint32_t regVal = SDL_REG32_RD_RAW(p);
    regVal = (regVal & ~(mask));
    regVal |= (v << shift) & mask;
    SDL_REG32_WR_RAW(p, regVal);
    return;
}

/**
 *  \brief   This function reads a 16 bit register, modifies specific set of
 *           bits and writes back to the register.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   mask   Mask for the bit field.
 *  \param   shift  Bit field shift from LSB.
 *  \param   v      Value to be written to bit-field.
 */
static inline void SDL_REG16_FINS_RAW(volatile uint16_t * const p,
                                      uint16_t mask,
                                      uint32_t shift,
                                      uint16_t v);
static inline void SDL_REG16_FINS_RAW(volatile uint16_t * const p,
                                      uint16_t mask,
                                      uint32_t shift,
                                      uint16_t v)
{
    uint16_t regVal = SDL_REG16_RD_RAW(p);
    regVal = (regVal & ~(mask));
    regVal |= (v << shift) & mask;
    SDL_REG16_WR_RAW(p, regVal);
    return;
}

/**
 *  \brief   This function reads a 8 bit register, modifies specific set of
 *           bits and writes back to the register.
 *
 *  \param   p      Address of the memory mapped hardware register.
 *  \param   mask   Mask for the bit field.
 *  \param   shift  Bit field shift from LSB.
 *  \param   v      Value to be written to bit-field.
 */
static inline void SDL_REG8_FINS_RAW(volatile uint8_t * const p,
                                     uint8_t mask,
                                     uint32_t shift,
                                     uint8_t v);
static inline void SDL_REG8_FINS_RAW(volatile uint8_t * const p,
                                     uint8_t mask,
                                     uint32_t shift,
                                     uint8_t v)
{
    uint8_t regVal = SDL_REG8_RD_RAW(p);
    regVal = (regVal & ~(mask));
    regVal |= (v << shift) & mask;
    SDL_REG8_WR_RAW(p, regVal);
    return;
}

#endif /* SDLR_H_ */

/* Copyright (C) 2017-2020 Texas Instruments Incorporated
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
 *   @file  csl_utils.h
 *
 *   @path  $(CSLPATH)\inc
 *
 *   @desc  This file contains the pragma support for C/C++ styles
 *
 */

#ifndef CSL_UTILS_H
#define CSL_UTILS_H

#include <stdint.h>


#ifdef __cplusplus
extern "C"
{
#endif



/* remove c++'s implicit static built into const */
#ifdef __cplusplus
#define CSL_PUBLIC_CONST                extern const
#else
#define CSL_PUBLIC_CONST                const
#endif

#if defined(__GNUC__) && !defined(__ti__)
/* GCC */
#define CSL_SET_DSECT(section_name)     __attribute__((section(section_name)))
#define CSL_SET_DALIGN(x)               __attribute__((aligned(x)))
#define CSL_SET_CSECT(section_name)     __attribute__((section(section_name)))
#else
/* non GCC */
#define PRAGMA(x) _Pragma(#x)
#ifdef __cplusplus
#define CSL_SET_CSECT(f,s)            PRAGMA(CODE_SECTION(s))
#define CSL_SET_DSECT(f,s)            PRAGMA(DATA_SECTION(s))
#define CSL_SET_DALIGN(d,s)           PRAGMA(DATA_ALIGN(s))
#else
#define CSL_SET_CSECT(f,s)            PRAGMA(CODE_SECTION(f, s))
#define CSL_SET_DSECT(f,s)            PRAGMA(DATA_SECTION(f, s))
#define CSL_SET_DALIGN(d,s)           PRAGMA(DATA_ALIGN(d, s))
#endif /* __cplusplus */
#endif /*  defined(__GNUC__) && !defined(__ti__) */

/*************************************************************
 * Common MACROs
 ************************************************************/
/*! Max macro */
#define CSL_MAX(x,y) ((x) > (y) ? (x) : (y))
/*! Min macro */
#define CSL_MIN(x,y) ((x) < (y) ? (x) : (y))

/*! Memory alignment */
#define CSL_MEM_ALIGN(addr, byteAlignment) ((((uintptr_t)(addr)) + ((byteAlignment)-1)) & ~((byteAlignment)-1))

/*! Check for memory non-alignment, returns 1 if not aligned, 0 otherwise */
#define CSL_MEM_IS_NOT_ALIGN(addr, byteAlignment) (((uintptr_t)(addr) & ((byteAlignment) - 1)) != 0)

/*! Check for memory alignment, returns 1 if aligned, 0 otherwise */
#define CSL_MEM_IS_ALIGN(addr, byteAlignment) (!CSL_MEM_IS_NOT_ALIGN(addr, byteAlignment))

/*! Find the next multiple of y for x */
#define CSL_NEXT_MULTIPLE_OF(x, y) CSL_MEM_ALIGN(x, y)

/*! Macro for memory alignment to double word (= 2x32-bit = 64-bit). Typically
 *  in signal processing the datum sizes are 16 or 32-bits, but optimized code
 *  for C6X DSPs(C64+, C674 and C66)tends to manipulate this data in chunks of
 *  64-bits which are more optimally implemented if the data is aligned to
 *  64-bits even though the underlying datum sizes are less than 64-bits. Such
 *  requirements are present for example in several mmwavelib functions.
 */
#define CSL_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_C6X  (8)

/*! Maximum structure alignment on C6X DSPs(C64+, C674 and C66). Assumes the
 *  largest sized field is present in the structure, which is 64-bit. Note
 *  actual structure may need less alignment because the largest size of a field
 *  in the structure may be less than 64-bit.
 */
#define CSL_MEMORY_ALLOC_MAX_STRUCT_BYTE_ALIGNMENT_C6X sizeof(uint64_t)

/*! Maximum structure alignment on ARM. Assumes the largest sized field is
 *  present in the structure, which is 64-bit. Note actual structure may need
 *  less alignment because the largest size of a field in the structure may be
 *  less than 64-bit.
 */
#define CSL_MEMORY_ALLOC_MAX_STRUCT_BYTE_ALIGNMENT_ARM sizeof(uint64_t)

/*! \brief Macro to get the size of an array. */
#define CSL_ARRAYSIZE(x)                     (sizeof(x) / sizeof(x[0]))

#ifdef __cplusplus
}
#endif

#endif /* CSL_UTILS_H */
/* Nothing past this point */

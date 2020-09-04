/**
 *   @file  csl_complex_math_types.h
 *
 *   @brief
 *      This is the common header file that contains complex data types definition.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2020, Texas Instruments, Inc.
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
#ifndef CSL_COMPLEX_MATH_TYPES_H_
#define CSL_COMPLEX_MATH_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/hw_include/tistdtypes.h>

/*! @brief  Complex data type. This type of input, (first real than
 * imaginary part), is required for DSP lib FFT functions */
#ifdef _TMS320C6X
/* Complex type is often manipulated in the DSP using special compiler intrinsics
   (e.g _cmpyr) which requires the data of this type
   to be 32-bit (4 byte) aligned. Using intrinsics on unaligned memory
   can cause unexpected results.
*/
typedef struct __attribute__((aligned(4))) cmplx16ReIm_t_
#else
typedef struct cmplx16ReIm_t_
#endif
{
    int16_t real; /*!< @brief real part */
    int16_t imag; /*!< @brief imaginary part */
} cmplx16ReIm_t;

/*! @brief   Complex data type, natural for C674x complex
 * multiplication instructions. */
#ifdef _TMS320C6X
/* Complex type is often manipulated in the DSP using special compiler intrinsics
   (e.g _cmpyr) which requires the data of this type
   to be 32-bit (4 byte) aligned. Using intrinsics on unaligned memory
   can cause unexpected results.
*/
typedef struct __attribute__((aligned(4))) cmplx16ImRe_t_
#else
typedef struct cmplx16ImRe_t_
#endif
{
    int16_t imag; /*!< @brief imaginary part */
    int16_t real; /*!< @brief real part */
} cmplx16ImRe_t;

/*! @brief  Complex data type. This type of input, (first real than
 * imaginary part), is required for DSP lib FFT functions */
typedef struct cmplx32ReIm_t_
{
    int32_t real; /*!< @brief real part */
    int32_t imag; /*!< @brief imaginary part */
} cmplx32ReIm_t;

/*! @brief  Complex data type, natural for C674x complex
 * multiplication instructions */
typedef struct cmplx32ImRe_t_
{
    int32_t imag; /*!< @brief imaginary part */
    int32_t real; /*!< @brief real part */
} cmplx32ImRe_t;

/*! @brief  Complex data type */
typedef struct cmplx64ImRe_t_
{
    int64_t real; /*!< @brief real part */
    int64_t imag; /*!< @brief imaginary part */
} cmplx64ImRe_t;

#ifdef __cplusplus
}
#endif

#endif /* CSL_COMPLEX_MATH_TYPES_H_ */

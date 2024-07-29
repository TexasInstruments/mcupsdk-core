/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

/**
 *  \defgroup DRV_FASTRTS_MODULE APIs for MATHLIB
 *  \ingroup DRV_MODULE
 *
 *  This module contains FastRTS APIs for optimized trignometric and math functions.
 *
 *  @{
 */

#ifndef FASTRTS_H
#define FASTRTS_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

#define RTS_TEXT_SECTION   __attribute__((section(".trigText"))) __attribute__((always_inline)) inline
#define RTS_DATA_SECTION   __attribute__((aligned(8), section(".trigData")))

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief The Cosine Function
 * @param[in] theta - Angle in radians
 * @return The cosine of the angle
 */
extern float FastRTS_cosf(const float theta);

/**
 * @brief The Sine Function
 * @param[in] theta - Angle in radians
 * @return The sine of the angle
 */
extern float FastRTS_sinf(const float theta);

/**
 * @brief The Sine/Cosine Function
 * @param[in] theta - Angle in radians
 * @param[in] retValues - pointer that stores return sine and cosine values
 */
extern void FastRTS_sincos(const float theta, float *retValues);

/**
 * @brief The Arccosine Function
 * @param[in] theta - Angle in radians
 * @return The cosine of the angle
 */
extern float FastRTS_acosf(const float theta);

/**
 * @brief The Arcsine Function
 * @param[in] theta - Angle in radians
 * @return The sine of the angle
 */
extern float FastRTS_asinf(const float theta);

/**
 * @brief The Arctangent function
 * @param[in] x the arc subtended by the angle theta
 * @return theta , the angle subtended by the arc of length x
 */
extern float FastRTS_atanf(const float x);

/**
 * @brief The Arctangent(y,x) function
 * @param[in] y - numerator of the ratio y/x which is the length of the arc
 *            subtended by the angle theta
 * @param[in] x - denominator of the ratio y/x which is the length of the arc
 *            subtended by the angle theta
 * @return theta , the angle subtended by the arc of length y/x
 */
extern float FastRTS_atan2f(const float y, const float x);

/**
 * @brief The Exponent function
 * @param[in] x - argument whose exponent is to be calculated
 * @note due to the allowable range of 32-bit float, the maximum exponent allowed is +/-88
 * @return e^x , the exponent of the argument
 */
extern float FastRTS_expf(const float x);

/**
 * @brief The Natural Logarithm function
 * @param[in] x - argument whose logarithm is to be calculated
 * @return log(x) , the natural logarithm of the argument
 */
extern float FastRTS_logf(const float x);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

// Originates from ti_arm_trig.h and is slightly faster than standard math version
RTS_TEXT_SECTION static float FastRTS_sqrtf(const float x)
{
    float r = 0;
    __asm ("VSQRT.F32 %0, %1" : "=t" (r) : "t" (x));
    return r;
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* FASTRTS_H */

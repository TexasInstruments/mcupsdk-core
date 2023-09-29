/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  \defgroup DRV_MATHLIB_MODULE APIs for MATHLIB
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs for optimized trignometric functions.
 *
 *  @{
 */


#ifndef TI_ARM_TRIG_H_
#define TI_ARM_TRIG_H_

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

#define TRIG_TEXT_SECTION   __attribute__((section(".trigText"))) __attribute__((always_inline)) //avoid function call overhead
#define TRIG_DATA_SECTION   __attribute__((aligned(8), section(".trigData")))

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   Computes the trigonometric sine value of the input angle using polynomial approximation techniques.
 *
 * \param   [in] angleRad - input angle in radians within [0, 2PI]
 *
 * \return  Computed sine value
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between 0 to 2PI. Add 2PI to negative values before calling this function.
 *          No error checking is performed on input.
 */
extern float ti_arm_sin(float angleRad);


/**
 * \brief   Computes the trigonometric cosine value of the input angle using polynomial approximation techniques.
 *
 * \param   [in] angleRad - - input angle in radians within [0, 2PI]
 *
 * \return  Computed cosine value
 *
 *  \note   Usage Considerations:
 *          Valid input is limited to values between 0 to 2PI. Add 2PI to negative values before calling this function.
 *          No error checking is performed on input.
 */
extern float ti_arm_cos(float angleRad);


/**
 * \brief   Computes the trigonometric sine and cosine values of the input angle using polynomial approximation techniques.
 *
 * \param   [in] angleRad - - input angle in radians
 * \param   [out] retValues - points to the computed sine and cosine values.
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between 0 to 2PI. Add 2PI to negative values before calling this function.
 *          No error checking is performed on input.
 */
extern void ti_arm_sincos(float angleRad, float *retValues);

/**
 * \brief   Computes the trigonometric arcsine value of the input value using polynomial approximation techniques.
 *
 * \param   [in] x - input value within the domain of arcsine
 *
 * \return  Computed arcsine value
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values within the domain of arcsine, [-1, 1].
 *          No error checking is performed on input.
 */
extern float ti_arm_asin(float x);

/**
 * \brief   Computes the trigonometric arccosine value of the input value using polynomial approximation techniques.
 *
 * \param   [in] x - input value within the domain of arccosine
 *
 * \return  Computed arccosine value
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values within the domain of arccosine, [-1, 1].
 *          No error checking is performed on input.
 */
extern float ti_arm_acos(float x);

/**
 * \brief   Computes the trigonometric arctangent value of the input value using polynomial approximation techniques.
 *
 * \param   [in] x - input value within the domain of arctangent
 *
 * \return  Computed arctangent value
 *
 * \note    Usage Considerations:
 *          No error checking is performed on input.
 */
extern float ti_arm_atan(float x);


/**
 * \brief   Computes the trigonometric atan2 value of the input values using polynomial approximation techniques.
 *
 * \param   [in] x - input value within the domain of arctangent
 * \param   [in] y - input value within the domain of arctangent
 *
 * \return  Computed atan2 value
 *
 * \note    Usage Considerations:
 *          No error checking is performed on input.
 */
extern float ti_arm_atan2(float y, float x);

/**
 * \brief   Arm FPU Hardware Single-Precision Square Root Function.
 *
 * \param   [in] x - input value
 *
 * \return  Square root of an input number
 *
 */
TRIG_TEXT_SECTION static inline float ti_arm_sqrt(float x);

/**
 * \brief   Arm FPU Hardware Single-Precision Absolute Value Function.
 *
 * \param   [in] x - input value
 *
 * \return  Absolute value of an input number
 *
 */
TRIG_TEXT_SECTION static inline float ti_arm_abs(float x);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

TRIG_TEXT_SECTION static inline float ti_arm_sqrt(float x)
{
    float r = 0;
    __asm ("VSQRT.F32 %0, %1" : "=t" (r) : "t" (x));
    return r;
}

TRIG_TEXT_SECTION static inline float ti_arm_abs(float x)
{
    float r = 0;
    __asm ("VABS.F32 %0, %1" : "=t" (r) : "t" (x));
    return r;
}

#endif /* TI_ARM_TRIG_H */

/** @} */
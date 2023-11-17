/* Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/**
 *  \defgroup DRV_TMU_MODULE APIs for TMU
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs for TMU operations.
 *
 *  @{
 */

#ifndef TI_TMU_TRIG_H_
#define TI_TMU_TRIG_H_

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   Computes the trigonometric sine value of the input angle using TMU.
 *
 * \param   [in] anglePU - input angle in per unit within [-1.0f, +1.0f]
 *
 * \return  Computed sine value
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between -1.0f to +1.0f.
 *          No error checking is performed on input.
 */
extern float ti_tmu_sin_pu(float anglePU);


/**
 * \brief   Computes the trigonometric cosine value of the input angle using TMU.
 *
 * \param   [in] anglePU - input angle in per unit within [-1.0f, +1.0f]
 *
 * \return  Computed cosine value
 *
 *  \note   Usage Considerations:
 *          Valid input is limited to values between -1.0f to +1.0f.
 *          No error checking is performed on input.
 */
extern float ti_tmu_cos_pu(float anglePU);


/**
 * \brief   Computes the trigonometric atan value of the input using TMU.
 *
 * \param   [in] x - input value from -1.0f to 1.0f
 *
 * \return  Computed atan value −0.125f to +0.125f (the value whose tan is x)
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between -1.0f to 1.0f.
 *          No error checking is performed on input.
 */
extern float ti_tmu_atan_pu(float x);


/**
 * \brief   Computes the logarithmic value of the input using TMU.
 *
 * \param   [in] x - input value in float
 *
 * \return  Computed log value (base of 2)
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between negative infinity to positive infinity.
 *          No error checking is performed on input.
 */
extern float ti_tmu_log_pu(float x);


/**
 * \brief   Computes the inverse exponential value of the input using TMU.
 *
 * \param   [in] x - input value in float
 *
 * \return  Computed exponential value (base of 2)
 *
 * \note    Usage Considerations:
 *          No error checking is performed on input.
 */
extern float ti_tmu_iexp_pu(float x);


/**
 * \brief   Computes the trigonometric sine value of the input angle using TMU.
 *
 * \param   [in] angleRad - input angle in radians within [-2PI, +2PI]
 *
 * \return  Computed sine value
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between -2PI to 2PI. The input is multiplied with 1/2PI to convert it to per unit value for using TMU.
 *          No error checking is performed on input.
 */
extern float ti_tmu_sin(float angleRad);


/**
 * \brief   Computes the trigonometric cosine value of the input angle using TMU.
 *
 * \param   [in] angleRad - input angle in radians within [-2PI, +2PI]
 *
 * \return  Computed cosine value
 *
 *  \note   Usage Considerations:
 *          Valid input is limited to values between -2PI to 2PI. The input is multiplied with 1/2PI to convert it to per unit value for using TMU.
 *          No error checking is performed on input.
 */
extern float ti_tmu_cos(float angleRad);

/**
 * \brief   Computes the trigonometric atan value of the input angle using TMU.
 *
 * \param   [in] x - input value from -1.0f to 1.0f
 *
 * \return  Computed atan value
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between -1.0f to 1.0f. The computed output value is multiplied with 2PI to change it to radians.
 *          No error checking is performed on input.
 */
extern float ti_tmu_atan(float x);


/**
 * \brief   Computes the trigonometric atan2 value of the input values using TMU. Uses the quadratic built in TMU function to comput the ratio and quadrant value, which is then used to comput atan2 value of input.
 *
 * \param   [in] x - input value within the domain of arctangent
 * \param   [in] y - input value within the domain of arctangent
 *
 * \return  Computed atan2 value
 *
 * \note    Usage Considerations:
 *          No error checking is performed on input.
 */
extern float ti_tmu_atan2(float x, float y);

#endif /* TI_TMU_TRIG_H */

/** @} */

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

#include <stdint.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/am263px/cslr_tmu.h>
#include <drivers/hw_include/am263px/cslr_soc_r5_baseaddress.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>


#ifndef ReciprocalOf2PI
#define ReciprocalOf2PI                0.159154943091895335768f
#endif

#ifndef TwoPI
#define TwoPI                          6.283185307F
#endif

#ifndef Log2ofe
#define Log2ofe                       1.44269F
#endif

#ifndef OnebyLog2ofe
#define OnebyLog2ofe                  0.693147F
#endif

#define ISR_TMU_CONTEXT_SAVE HW_WR_REG32((CSL_MSS_TMU_BASE + CSL_TMU_CONTEXT_SAVE), HW_RD_REG32(CSL_MSS_TMU_BASE + CSL_TMU_CONTEXT_SAVE)| (1));
#define ISR_TMU_CONTEXT_RESTORE HW_WR_REG32((CSL_MSS_TMU_BASE + CSL_TMU_CONTEXT_RESTORE), HW_RD_REG32(CSL_MSS_TMU_BASE + CSL_TMU_CONTEXT_RESTORE)| (1));


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
static inline float ti_tmu_sin_pu(float anglePU)
{
    __asm__ volatile("str %0, [%1]\n\t"
                     "DMB ST \n\t"
                    :
                    : "r" (anglePU), "r" (CSL_MSS_TMU_BASE  + CSL_TMU_SINPUF32_R0));

    return  *((float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R0));
}

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
static inline float ti_tmu_cos_pu(float anglePU)
{
    __asm__ volatile("str %0, [%1]\n\t"
                     "DMB ST \n\t"
                    :
                    : "r" (anglePU), "r" (CSL_MSS_TMU_BASE + CSL_TMU_COSPUF32_R0));

    return *((float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R0));
}


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
 static inline float ti_tmu_atan_pu(float x)
{
    __asm__ volatile("str %0, [%1]\n\t"
                     "DMB ST \n\t"
                    :
                    : "r" (x), "r" (CSL_MSS_TMU_BASE + CSL_TMU_ATANPUF32_R2));

    return *((float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R2));

}



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
 static inline float ti_tmu_log_pu(float x)
{
     __asm__ volatile("str %0, [%1]\n\t"
                     "DMB ST \n\t"
                    :
                    : "r" (x), "r" (CSL_MSS_TMU_BASE + CSL_TMU_LOG2F32_R3));

      return *((float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R3));
}

/**
 * \brief   Computes the logarithmic value to the base e of the input using TMU.
 *
 * \param   [in] x - input value in float
 *
 * \return  Computed log value (base of e)
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between negative infinity to positive infinity.
 *          No error checking is performed on input.
 */
 static inline float ti_tmu_log_e_pu(float x)
{
     __asm__ volatile("str %0, [%1]\n\t"
                     "DMB ST \n\t"
                    :
                    : "r" (x), "r" (CSL_MSS_TMU_BASE + CSL_TMU_LOG2F32_R3));

      return (*((float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R3))) * OnebyLog2ofe;
}



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
 static inline float ti_tmu_iexp_pu(float x)
{
 __asm__ volatile("str %0, [%1]\n\t"
                     "DMB ST \n\t"
                    :
                    : "r" (x), "r" (CSL_MSS_TMU_BASE + CSL_TMU_IEXP2F32_R3));

    return *((float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R3));
}

/**
 * \brief   Computes the inverse exponential value to the base e of the input using TMU.
 *
 * \param   [in] x - input value in float
 *
 * \return  Computed exponential value (base of e)
 *
 * \note    Usage Considerations:
 *          No error checking is performed on input.
 */
 static inline float ti_tmu_iexp_e_pu(float x)
{
 __asm__ volatile("str %0, [%1]\n\t"
                     "DMB ST \n\t"
                    :
                    : "r" (x * Log2ofe), "r" (CSL_MSS_TMU_BASE + CSL_TMU_IEXP2F32_R3));

    return *((float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R3));
}



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
static inline float ti_tmu_sin(float angleRad)
{
    __asm__ volatile("str %0, [%1]\n\t"
                     "DMB ST \n\t"
                    :
                    : "r" (angleRad * ReciprocalOf2PI), "r" (CSL_MSS_TMU_BASE + CSL_TMU_SINPUF32_R0));

    return *((float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R0));
}


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
static inline float ti_tmu_cos(float angleRad)
{
    __asm__ volatile("str %0, [%1]\n\t"
                     "DMB ST \n\t"
                    :
                    : "r" (angleRad * ReciprocalOf2PI), "r" (CSL_MSS_TMU_BASE + CSL_TMU_COSPUF32_R1));

    return *((float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R1));
}

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
static inline float ti_tmu_atan(float x)
{
    __asm__ volatile("str %0, [%1]\n\t"
                    "DMB ST \n\t"
                    :
                    : "r" (x), "r" (CSL_MSS_TMU_BASE + CSL_TMU_ATANPUF32_R2));

    return (*((float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R2))) * TwoPI;
}


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
static inline float ti_tmu_atan2(float x, float y)
{
    __asm__ volatile("str %1, [%2, #0x240] \n\t"
                    "str %0, [%2, #0x1F0] \n\t"
                    "DMB ST               \n\t"
                    "ldr %0, [%2, #0x2B0] \n\t"
                    "str %0, [%2, #0xC0]  \n\t"
                    "DMB ST               \n\t"
                    :
                    : "r" (x), "r" (y), "r" (CSL_MSS_TMU_BASE));

    return  (TwoPI * ((*(float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R0)) + (*(float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R7))));

}

/**
 * \brief   Computes the trigonometric sine and cosine value of the input using TMU.
 *
 * \param   [in] anglePU - input angle in per unit within [-1.0f, +1.0f]
 * \param   [in] sin_val - input value storing the value of sin after sincos operations
 * \param   [in] cos_val - input value storing the value of cos after sincos operations
 *
 * \note    Usage Considerations:
 *          No error checking is performed on input.
 */

static inline void ti_tmu_sincos_pu(float anglePU, float *sin_val, float *cos_val)
{
     __asm__ volatile("str %0, [%1]\n\t"
                     "str %0, [%2]\n\t"
                     "DMB ST \n\t"
                     ".rept 4 ; nop ; .endr\n\t"
                    :
                    : "r" (anglePU), "r" (CSL_MSS_TMU_BASE  + CSL_TMU_SINPUF32_R0), "r" (CSL_MSS_TMU_BASE  + CSL_TMU_COSPUF32_R1));

    *sin_val = *(float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R0);
    *cos_val = *(float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R1);
    return;
}

/**
 * \brief   Computes the trigonometric sine and cosine value of the input using TMU.
 *
 * \param   [in] angleRad - input angle in radians  within [-2PI, +2PI]
 * \param   [in] sin_val - input value storing the value of sin after sincos operations
 * \param   [in] cos_val - input value storing the value of cos after sincos operations
 *
 * \note    Usage Considerations:
 *          No error checking is performed on input.
 */

static inline void ti_tmu_sincos(float angleRad, float *sin_val, float *cos_val)
{
     __asm__ volatile("str %0, [%1]\n\t"
                     "str %0, [%2]\n\t"
                     "DMB ST \n\t"
                     ".rept 4 ; nop ; .endr\n\t"
                    :
                    : "r" (angleRad * ReciprocalOf2PI), "r" (CSL_MSS_TMU_BASE  + CSL_TMU_SINPUF32_R0), "r" (CSL_MSS_TMU_BASE  + CSL_TMU_COSPUF32_R1));

    *sin_val = *(float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R0);
    *cos_val = *(float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R1);
    return;
}

/**
 * \brief   Computes the power of the input value
 *
 * \param   [in] x - input value which is base of power function
 * \param   [in] y - input value that needs to be raised
 *
 * \return  Computed power value
 *
 * \note    Usage Considerations:
 *          No error checking is performed on input.
 */

static inline float ti_tmu_powf(float x, float y)
{
    __asm__ volatile("str %0, [%1, #0x180] \n\t"
                    "DMB ST               \n\t"
                    :
                    : "r" (x), "r" (CSL_MSS_TMU_BASE));

    float res = (*(float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R0)) * y;

    __asm__ volatile("str %0, [%1, #0x148] \n\t"
                     "DMB ST                \n\t"
                     :
                     : "r" (res), "r" (CSL_MSS_TMU_BASE));

    return res > 0 ? *(float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R1) : (1.0f / (*(float *)(CSL_MSS_TMU_BASE + CSL_TMU_RESULT_R1)));

}



#endif /* TI_TMU_TRIG_H */

/** @} */
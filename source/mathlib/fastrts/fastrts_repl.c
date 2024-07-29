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
 * @file  fastrts_repl.c
 *
 * @brief
 * FastRTS replacements for standard math library (math.h) functions,
 * include this source file if user wanted to replace the standard functions listed below
 * Not included in mathlib.lib by default
 */

#include "fastrts.h"

inline __attribute__((always_inline)) float sinf(float x)
{
    return FastRTS_sinf(x);
}

inline __attribute__((always_inline)) float cosf(float x)
{
    return FastRTS_cosf(x);
}

inline __attribute__((always_inline)) float acosf(float x)
{
    return FastRTS_acosf(x);
}

inline __attribute__((always_inline)) float asinf(float x)
{
    return FastRTS_asinf(x);
}

inline __attribute__((always_inline)) float atanf(float x)
{
    return FastRTS_atanf(x);
}

inline __attribute__((always_inline)) float atan2f(float y, float x)
{
    return FastRTS_atan2f(y, x);
}

inline __attribute__((always_inline)) float expf(float x)
{
    return FastRTS_expf(x);
}

inline __attribute__((always_inline)) float logf(float x)
{
    return FastRTS_logf(x);
}

inline __attribute__((always_inline)) float sqrtf(float x)
{
    return FastRTS_sqrtf(x);
}

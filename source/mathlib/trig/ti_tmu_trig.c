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

#include <stdint.h>
#include "ti_tmu_trig.h"


float __attribute__((noinline, naked)) ti_tmu_sin_pu(float anglePU)
{
     __asm__ volatile(                                   \
        "   PUSH {r10}                            \n"    \
        "   MOV.W    r10, #0x60000                \n"    \
        "   VSTR     s0, [r10, #0x40]             \n"    \
        "   DMB ST                                \n"    \
        "   VLDR s0, [r10, #0x280]                \n"    \
        "   POP {r10}                             \n"    \
        "   BX   R14                              \n"    \
      );
}

float __attribute__((noinline, naked)) ti_tmu_cos_pu(float anglePU)
{

    __asm__ volatile(                                    \
        "   PUSH {r10}                            \n"    \
        "   MOV.W    r10, #0x60000                \n"    \
        "   VSTR     s0, [r10, #0x88]             \n"    \
        "   DMB ST                                \n"    \
        "   VLDR s0, [r10, #0x288]                \n"    \
        "   POP {r10}                             \n"    \
        "   BX   R14                              \n"    \

      );
}

float __attribute__((noinline, naked)) ti_tmu_atan_pu(float x)
{

    __asm__ volatile(                                    \
        "   PUSH {r10}                            \n"    \
        "   MOV.W    r10, #0x60000                \n"    \
        "   VSTR     s0, [r10, #0xd0]             \n"    \
        "   DMB ST                                \n"    \
        "   VLDR s0, [r10, #0x290]                \n"    \
        "   POP {r10}                             \n"    \
        "   BX   R14                              \n"    \
      );

}

float __attribute__((noinline, naked)) ti_tmu_log_pu(float x)
{

    __asm__ volatile(                                    \
        "   PUSH {r10}                            \n"    \
        "   MOV.W    r10, #0x60000                \n"    \
        "   VSTR     s0, [r10, #0x198]            \n"    \
        "   DMB ST                                \n"    \
        "   VLDR s0, [r10, #0x298]                \n"    \
        "   POP {r10}                             \n"    \
        "   BX   R14                              \n"    \
      );

}

float __attribute__((noinline, naked)) ti_tmu_iexp_pu(float x)
{

    __asm__ volatile(                                    \
        "   PUSH {r10}                            \n"    \
        "   MOV.W    r10, #0x60000                \n"    \
        "   VSTR     s0, [r10, #0x158]            \n"    \
        "   DMB ST                                \n"    \
        "   VLDR s0, [r10, #0x298]                \n"    \
        "   POP {r10}                             \n"    \
        "   BX   R14                              \n"    \
      );

}

float __attribute__((noinline, naked)) ti_tmu_sin(float angleRad)
{

     __asm__ volatile(                                    \
        "   PUSH {r9, r10}                         \n"    \
        "   MOV.W    r10, #0x60000                 \n"    \
        "   LDR r9, =0x3e22f983                    \n"    \
        "   VMOV s1, r9                            \n"    \
        "   VMUL.F32 s0, s0, s1                    \n"    \
        "   VSTR     s0, [r10, #0x40]              \n"    \
        "   DMB ST                                 \n"    \
        "   VLDR s0, [r10, #0x280]                 \n"    \
        "   POP {r9, r10}                          \n"    \
        "   BX   R14                               \n"    \

      );

}

float __attribute__((noinline, naked)) ti_tmu_cos(float angleRad)
{

   __asm__ volatile(                                      \
        "   PUSH {r9, r10}                         \n"    \
        "   MOV.W    r10, #0x60000                 \n"    \
        "   LDR r9, =0x3e22f983                    \n"    \
        "   VMOV s1, r9                            \n"    \
        "   VMUL.F32 s0, s0, s1                    \n"    \
        "   VSTR     s0, [r10, #0x88]              \n"    \
        "   DMB ST                                 \n"    \
        "   VLDR s0, [r10, #0x288]                 \n"    \
        "   POP {r9, r10}                          \n"    \
        "   BX   R14                               \n"    \

      );

}

float __attribute__((noinline, naked)) ti_tmu_atan(float x)
{

   __asm__ volatile(                                       \
        "   PUSH {r9, r10}                          \n"    \
        "   MOV.W    r10, #0x60000                  \n"    \
        "   VSTR     s0, [r10, #0xd0]               \n"    \
        "   DMB ST                                  \n"    \
        "   VLDR s0, [r10, #0x290]                  \n"    \
        "   LDR   r9, =0x40c90fdb                   \n"    \
        "   VMOV s1, r9                             \n"    \
        "   VMUL.F32  s0, s0, s1                    \n"    \
        "   POP {r9, r10}                           \n"    \
        "   BX   R14                                \n"    \

      );

}

float __attribute__((noinline, naked)) ti_tmu_atan2(float x, float y)
{

    __asm__ volatile(                                    \
        "   PUSH {r0, r1}                         \n"    \
        "   MOV.W    r0, #0x60000                 \n"    \
        "   VSTR     s1, [r0, #0x240]             \n"    \
        "   VSTR     s0, [r0, #0x1F0]             \n"    \
        "   DMB ST                                \n"    \
        "   VLDR     s1, [r0, #0x2B0]             \n"    \
        "   VSTR     s1, [r0, #0xF8]              \n"    \
        "   VLDR     s2, [r0, #0x2B8]             \n"    \
        "   DMB ST                                \n"    \
        "   VLDR     s1, [r0, #0x2B8]             \n"    \
        "   VADD.F32  s0, s1, s2                  \n"    \
        "   LDR r1, =0x40c90fdb                   \n"    \
        "   VMOV    s2,r1                         \n"    \
        "   VMUL.F32       s0, s0, s2             \n"    \
        "   POP {r0, r1}                          \n"    \
        "   BX r14   \n" \
);

}

#include <stdint.h>
#include "ti_tmu_trig.h"


float __attribute__((always_inline, naked)) ti_tmu_sin_pu(float angleRad)
{
     __asm__ volatile(                  \
        "   PUSH {r10}                            \n" \
        "   MOV.W    r10, #0x60000                  \n"    \
        "   VSTR     s0, [r10, #0x40]         \n"    \
        "   dmb st         \n"    \
        "   VLDR s0, [r10, #0x280]              \n"    \
        "   POP {r10}                            \n" \
        "   BX   R14                            \n"   \
      );
}

float __attribute__((always_inline, naked)) ti_tmu_cos_pu(float angleRad)
{

    __asm__ volatile(                  \
        "   PUSH {r10}                            \n" \
        "   MOV.W    r10, #0x60000                  \n"    \
        "   VSTR     s0, [r10, #0x88]         \n"    \
        "   dmb st         \n"    \
        "   VLDR s0, [r10, #0x288]              \n"    \
        "   POP {r10}                            \n" \
        "   BX   R14                            \n"   \

      );
}

float __attribute__((always_inline, naked)) ti_tmu_atan_pu(float angleRad)
{

    __asm__ volatile(                  \
        "   PUSH {r10}                            \n" \
        "   MOV.W    r10, #0x60000                  \n"    \
        "   VSTR     s0, [r10, #0xd0]         \n"    \
        "   dmb st         \n"    \
        "   VLDR s0, [r10, #0x290]              \n"    \
        "   POP {r10}                          \n"    \
        "   BX   R14                            \n"   \
      );

}

float __attribute__((always_inline, naked)) ti_tmu_log_pu(float angleRad)
{

    __asm__ volatile(                  \
        "   PUSH {r10}                            \n" \
        "   MOV.W    r10, #0x60000                  \n"    \
        "   VSTR     s0, [r10, #0x198]         \n"    \
        "   dmb st         \n"    \
        "   VLDR s0, [r10, #0x298]              \n"    \
        "   POP {r10}                          \n"    \
        "   BX   R14                            \n"   \
      );

}

float __attribute__((always_inline, naked)) ti_tmu_iexp_pu(float angleRad)
{

    __asm__ volatile(                  \
        "   PUSH {r10}                            \n" \
        "   MOV.W    r10, #0x60000                  \n"    \
        "   VSTR     s0, [r10, #0x158]         \n"    \
        "   dmb st         \n"    \
        "   VLDR s0, [r10, #0x298]              \n"    \
        "   POP {r10}                          \n"    \
        "   BX   R14                            \n"   \
      );

}

float __attribute__((always_inline, naked)) ti_tmu_sin(float angleRad)
{

     __asm__ volatile(                  \
        "   PUSH {r9, r10}                            \n" \
        "   MOV.W    r10, #0x60000                  \n"    \
        "   LDR r9, =0x3e22f983              \n"  \
        "   VMOV s1, r9      \n"   \
        "   VMUL.F32 s0, s0, s1   \n"  \
        "   VSTR     s0, [r10, #0x40]         \n"    \
        "   dmb st         \n"    \
        "   VLDR s0, [r10, #0x280]              \n"    \
        "   POP {r9, r10}                            \n" \
        "   BX   R14                            \n"   \

      );

}

float __attribute__((always_inline, naked)) ti_tmu_cos(float angleRad)
{

   __asm__ volatile(                  \
        "   PUSH {r9, r10}                            \n" \
        "   MOV.W    r10, #0x60000                  \n"    \
        "   LDR r9, =0x3e22f983              \n"  \
        "   VMOV s1, r9      \n"   \
        "   VMUL.F32 s0, s0, s1   \n"  \
        "   VSTR     s0, [r10, #0x88]         \n"    \
        "   dmb st         \n"    \
        "   VLDR s0, [r10, #0x288]              \n"    \
        "   POP {r9, r10}                            \n" \
        "   BX   R14                            \n"   \

      );

}

float __attribute__((always_inline, naked)) ti_tmu_atan(float angleRad)
{

   __asm__ volatile(                  \
        "   PUSH {r9, r10}                            \n" \
        "   MOV.W    r10, #0x60000                  \n"    \
        "   VSTR     s0, [r10, #0xd0]         \n"    \
        "   dmb st         \n"    \
        "   VLDR s0, [r10, #0x290]              \n"    \
        "   LDR   r9, =0x40c90fdb            \n"  \
        "   VMOV s1, r9                      \n"  \
        "   VMUL.F32  s0, s0, s1   \n"  \
        "   POP {r9, r10}                            \n" \
        "   BX   R14                            \n"   \

      );

}

float __attribute__((always_inline, naked)) ti_tmu_atan2(float x, float y)
{

    __asm__ volatile(                  \
        "   PUSH {r0, r1}                            \n" \
        "   MOV.W    r0, #0x60000                  \n" \
        "   VSTR     s1, [r0, #0x240]       \n" \
        "   VSTR     s0, [r0, #0x1F0]        \n"  \
        "   dmb st          \n"  \
        "   VLDR     s1, [r0, #0x2B0]         \n"  \
        "   VSTR     s1, [r0, #0xF8]     \n" \
        "   VLDR     s2, [r0, #0x2B8] \n"  \
        "   dmb st             \n"  \
        "   VLDR     s1, [r0, #0x2B8]  \n" \
        "   VADD.F32  s0, s1, s2      \n"    \
        "   LDR r1, =0x40c90fdb    \n"  \
        "   VMOV    s2,r1       \n"  \

        "   vmul.f32       s0, s0, s2   \n"   \
        "   POP {r0, r1}                            \n" \
        "   BX r14   \n" \
);

}

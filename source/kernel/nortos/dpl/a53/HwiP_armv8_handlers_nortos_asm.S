
/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

        .global HwiP_defaultHandler
        .global HwiP_intrHandler


.macro POP_CALLER_SAVE_FPU_REGS stackPtr
        ldp     q30, q31, [\stackPtr], #32
        ldp     q28, q29, [\stackPtr], #32
        ldp     q26, q27, [\stackPtr], #32
        ldp     q24, q25, [\stackPtr], #32
        ldp     q22, q23, [\stackPtr], #32
        ldp     q20, q21, [\stackPtr], #32
        ldp     q18, q19, [\stackPtr], #32
        ldp     q16, q17, [\stackPtr], #32
        ldp     q14, q15, [\stackPtr], #32
        ldp     q12, q13, [\stackPtr], #32
        ldp     q10, q11, [\stackPtr], #32
        ldp     q8, q9, [\stackPtr], #32
        ldp     q6, q7, [\stackPtr], #32
        ldp     q4, q5, [\stackPtr], #32
        ldp     q2, q3, [\stackPtr], #32
        ldp     q0, q1, [\stackPtr], #32
.endm


.macro POP_CALLER_SAVE_CPU_REGS stackPtr
        ldp     x29, x30, [\stackPtr], #16
        ldp     x18, x19, [\stackPtr], #16
        ldp     x16, x17, [\stackPtr], #16
        ldp     x14, x15, [\stackPtr], #16
        ldp     x12, x13, [\stackPtr], #16
        ldp     x10, x11, [\stackPtr], #16
        ldp     x8, x9, [\stackPtr], #16
        ldp     x6, x7, [\stackPtr], #16
        ldp     x4, x5, [\stackPtr], #16
        ldp     x2, x3, [\stackPtr], #16
        ldp     x0, x1, [\stackPtr], #16    /* x0 & x1 saved by vector entry */
.endm

        .align  2

/*
 *  ======== HwiP_dispatchIRQ ========
 *
 *  Void HwiP_dispatchIRQ(uint32_t usingEL0Stack);
 */
        .global HwiP_dispatchIRQ
        .section .text.HwiP_dispatchIRQ, "ax"
        .func HwiP_dispatchIRQ

HwiP_dispatchIRQ:
        mov     x19, x0              /* copy 'usingEL0Stack' argument */
        mrs     x0, spsr_el1
        mrs     x1, elr_el1
        stp     x0, x1, [sp, #-16]!  /* save spsr_el1 and elr_el1 */
#if defined(__ARM_FP)
        mrs     x0, fpcr
        mrs     x1, fpsr
        stp     x0, x1, [sp, #-16]!  /* save fpcr and fpsr */
#endif

        /* call dispatchIRQC */
        mrs     x0, elr_el1          /* pass IRP to dispatchIRQC() */
        ldr     x1, =HwiP_intrHandler
        blr     x1

                                    /* returns with interrupts disabled */
#if defined(__ARM_FP)
        ldp     x0, x1, [sp], #16
        msr     fpcr, x0
        msr     fpsr, x1
#endif
        ldp     x0, x1, [sp], #16
        msr     spsr_el1, x0
        msr     elr_el1, x1
#if defined(__ARM_FP)
        POP_CALLER_SAVE_FPU_REGS sp  /* restore vfp scratch regs */
#endif
        POP_CALLER_SAVE_CPU_REGS sp  /* restore cpu scratch regs */
        eret
        .endfunc


/*
 *  ======== HwiP_defaultExcHandler ========
 *
 *  Void HwiP_defaultExcHandler(uint32_t usingEL0Stack);
 */

        .global HwiP_defaultExcHandler
        .section .text.HwiP_defaultExcHandler, "ax"
        .func HwiP_defaultExcHandler

HwiP_defaultExcHandler:

        mov     x19, x0              /* copy 'usingEL0Stack' argument */
        mrs     x0, spsr_el1
        mrs     x1, elr_el1
        stp     x0, x1, [sp, #-16]!  /* save spsr_el1 and elr_el1 */
#if defined(__ARM_FP)
        mrs     x0, fpcr
        mrs     x1, fpsr
        stp     x0, x1, [sp, #-16]!  /* save fpcr and fpsr */
#endif

        /* call dispatch default handler */
        mrs     x0, elr_el1          /* pass IRP to dispatchIRQC() */
        ldr     x1, =HwiP_defaultHandler
        blr     x1

                                    /* returns with interrupts disabled */
#if defined(__ARM_FP)
        ldp     x0, x1, [sp], #16
        msr     fpcr, x0
        msr     fpsr, x1
#endif
        ldp     x0, x1, [sp], #16
        msr     spsr_el1, x0
        msr     elr_el1, x1
#if defined(__ARM_FP)
        POP_CALLER_SAVE_FPU_REGS sp  /* restore vfp scratch regs */
#endif
        POP_CALLER_SAVE_CPU_REGS sp  /* restore cpu scratch regs */
        eret
        .endfunc
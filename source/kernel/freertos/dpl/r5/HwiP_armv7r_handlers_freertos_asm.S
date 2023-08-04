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

        .text
        .arm
        .align 2

/* FUNCTION DEF: void HwiP_data_abort_handler(void) */
        .global HwiP_data_abort_handler
        .type HwiP_data_abort_handler,%function
        .section ".text.hwi","ax",%progbits
        .arm
        .align 2
HwiP_data_abort_handler:
        /* Return to the instruction following the interrupted.
         * The instruction that caused the data abort is at R14_ABT – 8 which means
         * that the pointer points two instructions beyond the instruction that caused
         * the abort. (pc - 4) in arm mode and (pc - 6) in thumb mode (2 bytes aligned
         * instruction set) points to the second instruction beyond the address where
         * the exception was generated.
         */

        /*   Push used registers. */
        PUSH	{r0-r4, r12}

        /* SPSR has the snapshot of CPSR before data abort. Compare thumb state bit in SPSR */
        MRS r0, SPSR
        AND r1, r0, #0x20
        CMP R1, #0

        /* branches to label ARM_STATE if the thumb state bit is not set */
        BEQ ARM_STATE
        SUB lr, lr, #2
        ARM_STATE:
        SUB lr, lr, #4
        END:

        /*   Push the return address and SPSR. */
        PUSH	{lr}
        MRS	lr, SPSR
        PUSH	{lr}

        /*   Call the interrupt handler. */
        LDR	r1, HwiP_data_abort_handler_const
        BLX	r1

        /*  Restore used registers, LR and SPSR before  returning. */
        POP	{LR}
        MSR	SPSR_cxsf, LR
        POP	{LR}
        POP	{r0-r4, r12}
        MOVS	PC, LR

HwiP_data_abort_handler_const: .word HwiP_data_abort_handler_c

        .end

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
        .global _vectors
        .type   _vectors,%function
        .section ".vectors","ax",%progbits
        .arm
        .align 2
_vectors:
        LDR pc, reset_addr          // Reset
        LDR pc, undefined_addr      // Undefined Instruction
        LDR pc, svc_addr            // Software interrupt
        LDR pc, prefetch_abort_addr // Abort (prefetch)
        LDR pc, data_abort_addr     // Abort (data)
        LDR pc, reserved_addr       // rsvd
        LDR pc, irq_addr            // IRQ
        LDR pc, fiq_addr            // FIQ

reset_addr: .long _c_int00
undefined_addr: .long HwiP_undefined_handler
svc_addr: .long HwiP_svc_handler
prefetch_abort_addr: .long HwiP_prefetch_abort_handler
data_abort_addr: .long HwiP_data_abort_handler
reserved_addr: .long HwiP_reserved_handler
irq_addr: .long HwiP_irq_handler
fiq_addr: .long HwiP_fiq_handler

	.end


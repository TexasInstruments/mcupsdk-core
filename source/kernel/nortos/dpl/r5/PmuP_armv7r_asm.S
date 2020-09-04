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

/* FUNCTION DEF: uint32_t CycleCounterP_getCount32(void) */
        .global CycleCounterP_getCount32
        .type CycleCounterP_getCount32,%function
        .section ".text.pmu","ax",%progbits
        .arm
        .align 2
CycleCounterP_getCount32:
        MRC     p15, #0, r0, c9, c13, 0  // read PMCCNTR
        bx      LR

/* FUNCTION DEF: void PmuP_setup(uint32_t) */
        .global PmuP_setup
        .type PmuP_setup,%function
        .section ".text.pmu","ax",%progbits
        .arm
        .align 2
PmuP_setup:
        MCR     p15, #0, r0, c9, c12, 0 // write PMCR
        bx      LR

/* FUNCTION DEF: void PmuP_enableCounters(uint32_t) */
        .global PmuP_enableCounters
        .type PmuP_enableCounters,%function
        .section ".text.pmu","ax",%progbits
        .arm
        .align 2
PmuP_enableCounters:
        MCR     p15, #0, r0, c9, c12, 1  // write PMCNTENSET
        bx      LR

/* FUNCTION DEF: void PmuP_disableCounters(uint32_t) */
        .global PmuP_disableCounters
        .type PmuP_disableCounters,%function
        .section ".text.pmu","ax",%progbits
        .arm
        .align 2
PmuP_disableCounters:
        MCR     p15, #0, r0, c9, c12, 2  // write PMCNTENCLR
        bx      LR

/* FUNCTION DEF: uint32_t PmuP_getOverflowStatus(void) */
        .global PmuP_getOverflowStatus
        .type PmuP_getOverflowStatus,%function
        .section ".text.pmu","ax",%progbits
        .arm
        .align 2
PmuP_getOverflowStatus:
        MRC     p15, 0, r0, c9, c12, 3  // read PMOVSR
        bx      LR

/* FUNCTION DEF: uint32_t PmuP_clearOverflowStatus(void) */
        .global PmuP_clearOverflowStatus
        .type PmuP_clearOverflowStatus,%function
        .section ".text.pmu","ax",%progbits
        .arm
        .align 2
PmuP_clearOverflowStatus:
        MCR     p15, 0, r0, c9, c12, 3  // write PMOVSR
        bx      LR

	.end


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


/* FUNCTION DEF: uint32_t MmuP_isEnabled() */
        .global MmuP_isEnabled
        .section .text.MmuP_isEnabled
        .func MmuP_isEnabled

MmuP_isEnabled:
        mrs x0, sctlr_el1
        and x0, x0, #0x1
        ret
        .endfunc


/* FUNCTION DEF: void MmuP_tlbInvAll(void); */
        .global MmuP_tlbInvAll
        .section .text.MmuP_tlbInvAll
        .func MmuP_tlbInvAll

MmuP_tlbInvAll:
        tlbi   vmalle1
        dsb    sy
        isb
        ret
        .endfunc
        .balign 16


/* FUNCTION DEF: void MmuP_enableI(void) */
        .global MmuP_enableI
        .section .text.MmuP_enableI
        .func MmuP_enableI

MmuP_enableI:
        mrs x0, sctlr_el1
        orr x0, x0, #0x1
        msr sctlr_el1, x0
        ret
        .endfunc


/* FUNCTION DEF: void MmuP_disableI(void) */
        .global MmuP_disableI
        .section .text.MmuP_disableI
        .func MmuP_disableI

MmuP_disableI:
        mrs x0, sctlr_el1
        bic x0, x0, #0x1
        msr sctlr_el1, x0
        ret
        .endfunc


/* FUNCTION DEF: void MmuP_setMAIRAsm(uint8_t idx, uint8_t attr) */
        .global MmuP_setMAIRAsm
        .section .text.MmuP_setMAIRAsm
        .func MmuP_setMAIRAsm

MmuP_setMAIRAsm:
        lsl x0, x0, #3
        lslv x1, x1, x0
        movz x2, 0xFF
        lsl  x2, x2, x0
        mrs x0, MAIR_EL1
        bic x0, x0, x2
        orr x0, x0, x1
        msr MAIR_EL1, x0
        ret
        .endfunc


/* FUNCTION DEF: void MmuP_setTCR(uint64_t tcr) */
        .global MmuP_setTCR
        .section .text.MmuP_setTCR
        .func MmuP_setTCR

MmuP_setTCR:
        msr tcr_el1, x0
        ret
        .endfunc


/* FUNCTION DEF: void MmuP_setTableBase(uint64_t tableBase) */
        .global MmuP_setTableBase
        .section .text.MmuP_setTableBase
        .func MmuP_setTableBase
MmuP_setTableBase:
        msr ttbr0_el1, x0
        ret
        .endfunc

;/*
; *  Copyright (C) 2021 Texas Instruments Incorporated
; *
; *  Redistribution and use in source and binary forms, with or without
; *  modification, are permitted provided that the following conditions
; *  are met:
; *
; *    Redistributions of source code must retain the above copyright
; *    notice, this list of conditions and the following disclaimer.
; *
; *    Redistributions in binary form must reproduce the above copyright
; *    notice, this list of conditions and the following disclaimer in the
; *    documentation and/or other materials provided with the
; *    distribution.
; *
; *    Neither the name of Texas Instruments Incorporated nor the names of
; *    its contributors may be used to endorse or promote products derived
; *    from this software without specific prior written permission.
; *
; *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
; *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
; *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
; *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
; *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
; *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
; *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
; */
    .text

    .global HwiP_Entry
    .def HwiP_Entry
    .ref __c_int00
HwiP_Entry:
    MVKL __c_int00,B0
    MVKH __c_int00,B0
    B B0
    NOP 5
    NOP 5
    NOP 5
    NOP 5
    NOP 5

    .global HwiP_intcNmiDispatcher
    .def HwiP_intcNmiDispatcher
HwiP_intcNmiDispatcher:
    b nrp
    nop 5

    ; A0 contains the Register to be read
_HwiP_INTC_GIE_MASK .equ 0x00000001
_HwiP_INTC_GIE_MASK_DISABLE .equ 0xfffffffe
_HwiP_INTC_GEE .equ 4h
_HwiP_INTC_XEN .equ 8h

    .global HwiP_intcInterruptEnable
HwiP_intcInterruptEnable:
    ;input argument is in a4
    ;return value is in a4
    bnop b3, 1
    mvk   1, a5
    mvc ier, b0
    ||shl  a5, a4, a5 ; Or mask obtained
    shru b0, a5, a4 ; return value obtained
    ||or   a5, b0, b0
    mvc  b0, ier
    ||and a4,1,a4

    .global HwiP_intcInterruptDisable
HwiP_intcInterruptDisable:
    ;input argument is in a4
    ;return value is in a4
    bnop b3, 1
    mvk   1, a5
    mvc ier, b0
    ||shl  a5, a4, a5 ; Or mask obtained
    shru b0, a4, a4 ; return value obtained
    ||andn  b0, a5, b0
    mvc  b0, ier
    ||and a4,1,a4

    .global HwiP_intcInterruptClear
HwiP_intcInterruptClear:
    ;input argument is in a4
    bnop b3, 1
    mv   a4,b2
    mvk   1, b1
    shl  b1, b2, b1 ; Or mask obtained
    mvc  b1, icr

   .global HwiP_intcInterruptSet
HwiP_intcInterruptSet:
    bnop b3, 1
    mv   a4,b2
    mvk   1, b1
    shl  b1, b2, b1 ; Or mask obtained
    mvc  b1, isr

   .global HwiP_intcInterruptRestore
HwiP_intcInterruptRestore:
    ;a4 contains the vectorId
    ;b4 contains the value to be restored
    b b3
    mv a4,b1
    shl .s2 b4,b1,b4
    mvc ier,b0
    or b0,b4,b0
    mvc b0,ier

    .global HwiP_intcQueryInterruptStatus
HwiP_intcQueryInterruptStatus:
    b b3
    mvk 1, b1
    mvc ifr, b0
    shl b1, a4,a5
    and a5,b0,a5
    shru a5,a4,a4

    .global HwiP_intcGlobalEnable
HwiP_intcGlobalEnable:
    bnop b3
    mv a4,a0
    ||mvc csr, b0
    and b0 , _HwiP_INTC_GIE_MASK, a3
 [a0]  stnw a3 , *a0
    ||or  b0 , _HwiP_INTC_GIE_MASK, b0
    mvc b0 , csr
    mvk 1, a4

    .global HwiP_intcGlobalDisable
HwiP_intcGlobalDisable:
    bnop b3
    mv a4,a0
    ||mvc csr, b0
    and b0 , 1, a3
 [a0]stnw a3 , *a0
    ||and  b0 , _HwiP_INTC_GIE_MASK_DISABLE, b0
    mvc b0 , csr
    mvk 1, a4

    .global HwiP_intcGlobalRestore
HwiP_intcGlobalRestore:
    bnop b3
    mvc csr, b0
    and a4, _HwiP_INTC_GIE_MASK, b1
    or b1, b0,b0
    mvk 1, a4 ; status
    mvc b0 , csr

    .global HwiP_intcGlobalNmiEnable
HwiP_intcGlobalNmiEnable:
    bnop b3, 1
    mvc ier, b0
    or 2, b0, b0
    mvc b0, ier
    mvk 1, a4

    .global HwiP_intcGlobalExcepEnable
HwiP_intcGlobalExcepEnable:
    bnop b3, 1
    mvc tsr, b0
    or _HwiP_INTC_GEE, b0, b0
    mvc b0, tsr
    mvk 1, a4

    .global HwiP_intcGlobalExtExcepEnable
HwiP_intcGlobalExtExcepEnable:
    bnop b3, 1
    mvc tsr, b0
    or _HwiP_INTC_XEN, b0, b0
    mvc b0, tsr
    mvk 1, a4

    .global HwiP_intcGlobalExcepClear
HwiP_intcGlobalExcepClear:
    b b3
    mvk 1, a3
    shl a3, a4, a3
    mv a3, b2
    mvc b2, ecr
    mvk 1, a4


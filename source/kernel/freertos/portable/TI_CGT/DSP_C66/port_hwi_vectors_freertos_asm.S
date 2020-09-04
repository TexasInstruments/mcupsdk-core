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

    .def _vectors

    .ref HwiP_Entry
    .ref gHwiIntcIntrTable
    .ref Hwi_dispatchAlways

    .global HwiP_intcIvpSet
HwiP_intcIvpSet:
    bnop b3,2
    mvkl _vectors, b0
    mvkh _vectors, b0
    mvc b0, istp

RESV    .macro num
    .loop num
    mvkh gHwiIntcIntrTable,a4
    .endloop
    .endm

_HwiP_intcpush .macro reg
    stw reg,*b15--[2]
    .endm

_HwiP_intcpop .macro reg
    ldw *++b15[2],reg
    .endm

CALLDISP .macro intr
    _HwiP_intcpush a0
    mvk  intr, a0
    stw  a0, *b15[1]
    mvkl Hwi_dispatchAlways, a0
    mvkh Hwi_dispatchAlways, a0
    b       a0
    _HwiP_intcpop a0
    nop 4
    .endm

    .text
    .sect ".text:vectors"
    .align 1024
    .nocmp
_vectors:
    NOP
    NOP
    NOP
    mvkl HwiP_Entry,b0
    mvkh HwiP_Entry,b0
    b       b0
    NOP
    NOP 4

    .align 0x20
__HwiP_intcIsrNMI:
    CALLDISP 1

    .align 0x20
__HwiP_intcRsv2:
    RESV 8

    .align 0x20
__HwiP_intcRsv3:
    RESV 8

    .align 0x20
__HwiP_intcIsr4:
    CALLDISP 4

    .align 0x20
__HwiP_intcIsr5:
    CALLDISP 5

    .align 0x20
__HwiP_intcIsr6:
    CALLDISP 6

    .align 0x20
__HwiP_intcIsr7:
    CALLDISP 7

    .align 0x20
__HwiP_intcIsr8:
    CALLDISP 8

    .align 0x20
__HwiP_intcIsr9:
    CALLDISP 9

    .align 0x20
__HwiP_intcIsr10:
    CALLDISP 10

    .align 0x20
__HwiP_intcIsr11:
    CALLDISP 11

    .align 0x20
__HwiP_intcIsr12:
    CALLDISP 12

    .align 0x20
__HwiP_intcIsr13:
    CALLDISP 13

    .align 0x20
__HwiP_intcIsr14:
    CALLDISP 14

    .align 0x20
__HwiP_intcIsr15:
    CALLDISP 15

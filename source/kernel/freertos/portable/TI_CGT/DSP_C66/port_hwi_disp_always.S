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
;
;  ======== port_hwi_disp_always.S ========
;
;

        .cdecls C,NOLIST,"port_hwi.h"

;
; We should probably define a C struct for the stack frame below ...
;

;
; The following are double word offsets (offset must be < 32 for A-reg access)
;
Hwi_SFSSRITSR   .set    4
Hwi_SFILCRILC   .set    5
Hwi_SFGPLYAB    .set    6
Hwi_SFA1A0      .set    7
Hwi_SFA3A2      .set    8
Hwi_SFA5A4      .set    9
Hwi_SFA7A6      .set    10
Hwi_SFA9A8      .set    11
Hwi_SFA17A16    .set    12
Hwi_SFA19A18    .set    13
Hwi_SFA21A20    .set    14
Hwi_SFA23A22    .set    15
Hwi_SFA25A24    .set    16
Hwi_SFA27A26    .set    17
Hwi_SFA29A28    .set    18
Hwi_SFA31A30    .set    19
Hwi_SFB1B0      .set    20
Hwi_SFB3B2      .set    21
Hwi_SFB5B4      .set    22
Hwi_SFB7B6      .set    23
Hwi_SFB9B8      .set    24
Hwi_SFB17B16    .set    25
Hwi_SFB19B18    .set    26
Hwi_SFB21B20    .set    27
Hwi_SFB23B22    .set    28
Hwi_SFB25B24    .set    29
Hwi_SFB27B26    .set    30
Hwi_SFB29B28    .set    31
Hwi_SFB31B30    .set    32
Hwi_SFB31B30_SP32 .set    32-(32/2)

;
; The following are single word offsets, keep them in the low part of the frame
; (offset must be < 32 for A-reg access)
;
Hwi_SFoldCSR    .set    2
Hwi_SFoldAMR    .set    3
Hwi_SFoldIRP    .set    4
Hwi_SFoldIER    .set    5
Hwi_SFB14       .set    6

Hwi_STACKFRAMESIZE      .set    33      ; DW (8 byte) size

SP              .set    b15
SP32            .set    b0
a1_intNum       .set    a1
b4_TSR          .set    b4
TSR_XEN_BIT     .set    0x0008  ; eXternal exception ENable bit in TSR

    .if $isdefed("__TI_ELFABI__")
    .if __TI_ELFABI__
        .asg Hwi_dispatchC__I, _Hwi_dispatchC__I
        .asg Hwi_dispatchAlways, _Hwi_dispatchAlways
        .asg Hwi_Module__state__V, _Hwi_Module__state__V
    .endif
    .endif

        .global _Hwi_dispatchC__I
        .global _Hwi_dispatchAlways

_Hwi_Module__state__V .tag Hwi_Module_State__

;
; This dispatcher always aligns stack
;
        .sect ".text:_Hwi_dispatchAlways"
    .clink
_Hwi_dispatchAlways:
        .asmfunc

        stw     b10, *SP--[2]
        ldw     *SP[1], a1_intNum               ; get intr #
||      mv      SP, b10                         ; save pre-aligned SP
||      and     SP, ~0x7, SP                    ; align SP
        addk    -(Hwi_STACKFRAMESIZE*8), SP     ; alloc stack frame
        stdw    a1:a0, *SP[Hwi_SFA1A0]
||      mv      SP, a0
        stdw    b1:b0, *a0[Hwi_SFB1B0]
||      mvc     CSR, b1
||      addaw   SP, 32, SP32
        stdw    a9:a8, *a0[Hwi_SFA9A8]
||      stdw    b9:b8, *SP[Hwi_SFB9B8]
||      mvc     RILC, b8
||      mvkl    _Hwi_Module__state__V.bss, a9
        stdw    a3:a2, *a0[Hwi_SFA3A2]
||      stdw    b3:b2, *SP[Hwi_SFB3B2]
||      mv      b1, a3
||      mvc     AMR, b1
||      mvkh    _Hwi_Module__state__V.bss, a9
        stw     a3, *a0[Hwi_SFoldCSR]
||      stw     b1, *SP[Hwi_SFoldAMR]
||      mvc     IRP, b1
||      mvk     0, b2
        stdw    a5:a4, *a0[Hwi_SFA5A4]
||      stdw    b5:b4, *SP[Hwi_SFB5B4]
||      mv      b1, a3
||      mvc     b2, AMR
        ldw     *a9, b14                        ; init $bss b14
        stw     a3, *a0[Hwi_SFoldIRP]
||      stw     b14, *SP[Hwi_SFB14]
||      mvc     ILC, b9
        stdw    a7:a6, *a0[Hwi_SFA7A6]
||      stdw    b7:b6, *SP[Hwi_SFB7B6]
||      mvc     GPLYA, b7
        stdw    a17:a16, *a0[Hwi_SFA17A16]
||      stdw    b17:b16, *SP[Hwi_SFB17B16]
||      mvkl    _Hwi_dispatchC__I, a3
        stdw    a19:a18, *a0[Hwi_SFA19A18]
||      stdw    b19:b18, *SP[Hwi_SFB19B18]
||      mvkh    _Hwi_dispatchC__I, a3
||      mvc     TSR, b4_TSR
        stdw    a21:a20, *a0[Hwi_SFA21A20]
||      stdw    b21:b20, *SP[Hwi_SFB21B20]
||      mv      a1_intNum, a4
||      mvc     GPLYB, b6
        stdw    a23:a22, *a0[Hwi_SFA23A22]
||      stdw    b23:b22, *SP[Hwi_SFB23B22]
||      mvc     ITSR, b22
||      mv      b8, a20

        b       a3
||      stdw    a25:a24, *a0[Hwi_SFA25A24]
||      stdw    b25:b24, *SP[Hwi_SFB25B24]
        stdw    a27:a26, *a0[Hwi_SFA27A26]
||      stdw    b27:b26, *SP[Hwi_SFB27B26]
||      mvc     SSR, b23
||      mv      b9, a21
        stdw    a29:a28, *a0[Hwi_SFA29A28]
||      stdw    b29:b28, *SP[Hwi_SFB29B28]
||      mvkl    dispatchCRet1, b3
||      and     b22, TSR_XEN_BIT, a1
        stdw    a31:a30, *a0[Hwi_SFA31A30]
||      stdw    b31:b30, *SP32[Hwi_SFB31B30_SP32]
||      mvkh    dispatchCRet1, b3
        stdw    a21:a20, *a0[Hwi_SFILCRILC]
||      stdw    b7:b6, *SP[Hwi_SFGPLYAB]
||      or      b4_TSR, a1, b4_TSR
        stdw    b23:b22, *SP[Hwi_SFSSRITSR]
||      mvc     b4_TSR, TSR

dispatchCRet1:

        ;
        ; interrupts must be disabled upon return from above call
        ;

        mv      SP, a0
||      addaw   SP, 32, a1
||      lddw    *SP[Hwi_SFSSRITSR], b29:b28

        lddw    *a0[Hwi_SFILCRILC], a31:a30
||      lddw    *SP[Hwi_SFGPLYAB], b27:b26
||      mv      a1, SP32

        ldw     *SP[Hwi_SFoldAMR], b1
||      ldw     *a0[Hwi_SFoldCSR], a1
        ldw     *SP[Hwi_SFB14], b14
||      ldw     *a0[Hwi_SFoldIRP], a2
        lddw    *a0[Hwi_SFA5A4], a5:a4
||      lddw    *SP[Hwi_SFB5B4], b5:b4
        lddw    *a0[Hwi_SFA7A6], a7:a6
||      lddw    *SP[Hwi_SFB7B6], b7:b6
        lddw    *a0[Hwi_SFA9A8], a9:a8
||      lddw    *SP[Hwi_SFB9B8], b9:b8
||      mvc     b27, GPLYA
||      mv      a31, b31
        lddw    *a0[Hwi_SFA17A16], a17:a16
||      lddw    *SP[Hwi_SFB17B16], b17:b16
||      mvc     b1, AMR
||      mv      a1, b1
        lddw    *a0[Hwi_SFA19A18], a19:a18
||      lddw    *SP[Hwi_SFB19B18], b19:b18
||      mvc     b1, CSR
||      and     b1, 0x4, b2
        lddw    *a0[Hwi_SFA21A20], a21:a20
||      lddw    *SP[Hwi_SFB21B20], b21:b20
||      mvk     0x4, b23
||      mv      a2, b1
        lddw    *a0[Hwi_SFA23A22], a23:a22
||      lddw    *SP[Hwi_SFB23B22], b23:b22
||[ b2] sat     b23:b22, b22
||      mvc     b1, IRP

        lddw    *a0[Hwi_SFA25A24], a25:a24
||      lddw    *SP[Hwi_SFB25B24], b25:b24
||      mvc     b26, GPLYB
||      mv      a30, b30
        lddw    *a0[Hwi_SFA27A26], a27:a26
||      lddw    *SP[Hwi_SFB27B26], b27:b26
||      mvc     b29, SSR
        lddw    *a0[Hwi_SFA29A28], a29:a28
||      lddw    *SP[Hwi_SFB29B28], b29:b28
||      mvc     b28, ITSR
        lddw    *a0[Hwi_SFA3A2], a3:a2
||      lddw    *SP[Hwi_SFB3B2], b3:b2
||      mvc     b31, ILC
        lddw    *a0[Hwi_SFA31A30], a31:a30
||      lddw    *SP32[Hwi_SFB31B30_SP32], b31:b30
||      mvc     b30, RILC

        b       IRP
||      lddw    *a0[Hwi_SFA1A0], a1:a0
||      lddw    *SP[Hwi_SFB1B0], b1:b0
||      mv      b10, SP
        ldw     *++SP[2], b10
        nop     4

        .endasmfunc


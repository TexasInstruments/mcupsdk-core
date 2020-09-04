; Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; Redistributions of source code must retain the above copyright
; notice, this list of conditions and the following disclaimer.
;
; Redistributions in binary form must reproduce the above copyright
; notice, this list of conditions and the following disclaimer in the
; documentation and/or other materials provided with the
; distribution.
;
; Neither the name of Texas Instruments Incorporated nor the names of
; its contributors may be used to endorse or promote products derived
; from this software without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
; OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
; SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
; LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
; file:     icss_struct_defines.inc
;
; brief:    Common structure definitions

    .if !$isdefed("____icss_struct_defines_h")
____icss_struct_defines_h   .set    1

    .include "icss_constant_defines.inc"
    .include "icss_xfer_defines.inc"

;--------------------------------------------------------------------------;
;   Struct: Struct_1_Reg
;
;   Struct to expose 1 register as dword, words and bytes
;
;--------------------------------------------------------------------------;
Struct_1_Reg .struct
reg1    .union
x       .ulong
x_w     .struct
w0      .ushort
w2      .ushort
    .endstruct
x_b     .struct
b0      .ubyte
b1      .ubyte
b2      .ubyte
b3      .ubyte
    .endstruct
    .endunion
    .endstruct

;--------------------------------------------------------------------------;
;   Struct: Struct_2_Reg
;
;   Struct to expose 2 registers as dword, words and bytes
;
;--------------------------------------------------------------------------;
Struct_2_Reg .struct
reg1 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg2 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
    .endstruct

;--------------------------------------------------------------------------;
;   Struct: Struct_3_Reg
;
;   Struct to expose 3 registers as dword, words and bytes
;
;--------------------------------------------------------------------------;
Struct_3_Reg .struct
reg1 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg2 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg3 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
    .endstruct

;--------------------------------------------------------------------------;
;   Struct: Struct_4_Reg
;
;   Struct to expose 4 registers as dword, words and bytes
;
;--------------------------------------------------------------------------;
Struct_4_Reg .struct
reg1 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg2 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg3 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg4 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
    .endstruct

;--------------------------------------------------------------------------;
;   Struct: Struct_5_Reg
;
;   Struct to expose 5 registers as dword, words and bytes
;
;--------------------------------------------------------------------------;
Struct_5_Reg .struct
reg1 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg2 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg3 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg4 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg5 .union
x .ulong
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
    .endstruct

 .endif ;____icss_struct_defines_h
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

    .if    !$defined("__adc_preprocess_macros_h")
__adc_preprocess_macros_h    .set 1

;************************************************************************************
;   Macro: m_sign_ext
;
;   For sign extension from A bits to B bits, converts an A-bits wide signed number to a B-bits wide signed number.
;   A: 1 - 32
;   B: 8, 16, 24, 32
;
;   PEAK cycles:
;           3 cycle
;
;   Pseudo code:
;       ========== C ==========
;       if (18th bit == 1)
;           set(bits 18 to 32 HIGH)
;       =======================
;
;   Parameters:
;           A          -  Datawidth of input number
;           B          -  Required datawidth
;           data_reg   -  Input number
;           CONFIG     -  clears upper bits also if = EXTEND_0, else only extends 1
;
;   Returns:
;           data_reg   -  Output sign extended number
;************************************************************************************
m_sign_ext      .macro  A, B, data_reg, CONFIG
    ; .if     (A = 0) || (A > 32) || ((B != 8) && (B != 16) && (B != 24) && (B != 32))  ; Parameter check
    ;     ; throw compilation warning
    ;     .wmsg "ERROR: m_sign_ext -- Incorrect Parameter Value"
    ;     .mexit
    ; .endif  ; Parameter check

    ; If we just want to extend 1s at upper bits. Expecting all the upper bits are already 0s.
    .if     $symcmp(CONFIG,"EXTEND_0") != 0

    qbbc    SKIP_SIGN_EXT_OF_1?, data_reg, A-1    ; Check bit at position: (1<<A-1)

    ; Input datawidth upto 8 bits
    .if     A <= 8

    .if     B = 8
    or      data_reg.b0, data_reg.b0, $cvi($pow(2, 8)) - $cvi($pow(2, A))

    .elseif B = 16
    .if     A = 8
    sub     data_reg.b1, data_reg.b1, 1
    .else
    sub     data_reg.w0, data_reg.w0, 1<<A
    .endif  ; A = 8

    .elseif B = 24
    or      data_reg.b0, data_reg.b0, $cvi($pow(2, 8)) - $cvi($pow(2, A))
    ldi     data_reg.w1, 0xFFFF

    .elseif B = 32
    .if     A = 8
    sub     data_reg.b1, data_reg.b1, 1
    sub     data_reg.w2, data_reg.w2, 1
    .else
    sub     data_reg, data_reg, 1<<A
    .endif  ; A = 8
    .endif  ; B = 8

    ; Input datawidth 9 - 16 bits
    .elseif A <= 16

    .if     B = 16
    or      data_reg.b1, data_reg.b1, $cvi($pow(2, 8)) - $cvi($pow(2, A-8))

    .elseif B = 24
    .if     A = 16
    sub     data_reg.b2, data_reg.b2, 1
    .else
    sub     data_reg.w1, data_reg.w1, 1<<(A-8)
    .endif  ; A = 16

    .elseif B = 32
    or      data_reg.b1, data_reg.b1, $cvi($pow(2, 8)) - $cvi($pow(2, A-8))
    ldi     data_reg.w2, 0xFFFF
    .endif  ; B = 16

    ; Input datawidth 17 - 24 bits
    .elseif A <= 24

    .if     B = 24
    or      data_reg.b2, data_reg.b2, $cvi($pow(2, 8)) - $cvi($pow(2, A-16))

    .elseif B = 32
    .if     A = 24
    sub     data_reg.b3, data_reg.b3, 1
    .else
    sub     data_reg.w2, data_reg.w2, 1<<(A-16)
    .endif  ; A = 24
    .endif  ; B = 24

    ; Input datawidth 25 - 32 bits
    .elseif A <= 32

    .if     B = 32
    or      data_reg.b3, data_reg.b3, $cvi($pow(2, 8)) - $cvi($pow(2, A-24))
    .endif  ; B = 32

    .endif  ; A <= 8

SKIP_SIGN_EXT_OF_1?:

    ; If we need to write 0s also at upper bits. Incase there are some unwanted bits set which needs to be cleared.
    .elseif     $symcmp(CONFIG,"EXTEND_0") = 0

    qbbc    SKIP_SIGN_EXT_OF_1?, data_reg, A-1    ; Check bit at position: (1<<A-1)

    ; Input datawidth upto 8 bits
    .if     A <= 8

    .if     B = 8
    or      data_reg.b0, data_reg.b0, $cvi($pow(2, 8)) - $cvi($pow(2, A))
    qba     SKIP_TO_END?
SKIP_SIGN_EXT_OF_1?:
    and     data_reg.b0, data_reg.b0, $cvi($pow(2, A)) - 1

    .elseif B = 16
    or      data_reg.b0, data_reg.b0, $cvi($pow(2, 8)) - $cvi($pow(2, A))
    ldi     data_reg.b1, 0xFF
    qba     SKIP_TO_END?
SKIP_SIGN_EXT_OF_1?:
    and     data_reg.w0, data_reg.w0, $cvi($pow(2, A)) - 1

    .elseif B = 24
    or      data_reg.b0, data_reg.b0, $cvi($pow(2, 8)) - $cvi($pow(2, A))
    ldi     data_reg.w1, 0xFFFF
    qba     SKIP_TO_END?
SKIP_SIGN_EXT_OF_1?:
    and     data_reg.b0, data_reg.b0, $cvi($pow(2, A)) - 1
    ldi     data_reg.w1, 0x0000

    .elseif B = 32
    or      data_reg.b0, data_reg.b0, $cvi($pow(2, 8)) - $cvi($pow(2, A))
    ldi     data_reg.b1, 0xFF
    ldi     data_reg.w2, 0xFFFF
    qba     SKIP_TO_END?
SKIP_SIGN_EXT_OF_1?:
    and     data_reg, data_reg, $cvi($pow(2, A)) - 1
    .endif  ; B = 8

    ; Input datawidth 9 - 16 bits
    .elseif A <= 16

    .if     B = 16
    or      data_reg.b1, data_reg.b1, $cvi($pow(2, 8)) - $cvi($pow(2, A-8))
    qba     SKIP_TO_END?
SKIP_SIGN_EXT_OF_1?:
    and     data_reg.b1, data_reg.b1, $cvi($pow(2, A-8)) - 1

    .elseif B = 24
    or      data_reg.b1, data_reg.b1, $cvi($pow(2, 8)) - $cvi($pow(2, A-8))
    ldi     data_reg.b2, 0xFF
    qba     SKIP_TO_END?
SKIP_SIGN_EXT_OF_1?:
    and     data_reg.w1, data_reg.w1, $cvi($pow(2, A-8)) - 1

    .elseif B = 32
    or      data_reg.b1, data_reg.b1, $cvi($pow(2, 8)) - $cvi($pow(2, A-8))
    ldi     data_reg.w2, 0xFFFF
    qba     SKIP_TO_END?
SKIP_SIGN_EXT_OF_1?:
    and     data_reg.b1, data_reg.b1, $cvi($pow(2, A-8)) - 1
    ldi     data_reg.w2, 0x0000
    .endif  ; B = 16

    ; Input datawidth 17 - 24 bits
    .elseif A <= 24

    .if     B = 24
    or      data_reg.b2, data_reg.b2, $cvi($pow(2, 8)) - $cvi($pow(2, A-16))
    qba     SKIP_TO_END?
SKIP_SIGN_EXT_OF_1?:
    and     data_reg.b2, data_reg.b2, $cvi($pow(2, A-16)) - 1

    .elseif B = 32
    or      data_reg.b2, data_reg.b2, $cvi($pow(2, 8)) - $cvi($pow(2, A-16))
    ldi     data_reg.b3, 0xFF
    qba     SKIP_TO_END?
SKIP_SIGN_EXT_OF_1?:
    and     data_reg.w2, data_reg.w2, $cvi($pow(2, A-16)) - 1
    .endif  ; B = 24

    ; Input datawidth 25 - 32 bits
    .elseif A <= 32

    .if     B = 32
    or      data_reg.b3, data_reg.b3, $cvi($pow(2, 8)) - $cvi($pow(2, A-24))
    qba     SKIP_TO_END?
SKIP_SIGN_EXT_OF_1?:
    and     data_reg.b3, data_reg.b3, $cvi($pow(2, A-24)) - 1
    .endif  ; B = 32

    .endif  ; A <= 8

SKIP_TO_END?:
    .endif  ; CONFIG = EXTEND_0

    .endm

;************************************************************************************
;   Macro: m_sign_ext_32bit
;
;   For sign extension from 18 to 32 bits
;
;   PEAK cycles:
;           3 cycle
;
;   Pseudo code:
;       ========== C ==========
;       if (18th bit == 1)
;           set(bits 18 to 32 HIGH)
;       =======================
;
;   Parameters:
;           sample_reg
;
;   Returns:
;           sample_reg
;************************************************************************************
m_sign_ext_32bit    .macro  sample_reg
    qbbc   SKIP_SIGN_EXT?, sample_reg, 17    ; bit position: (1<<17)
    or     sample_reg.b3, sample_reg.b3, 0xFF
    or     sample_reg.b2, sample_reg.b2, 0xFE
SKIP_SIGN_EXT?:
    .endm

;************************************************************************************
;   Macro: m_sign_ext_16bit
;
;   For sign extension from 14 to 16 bits
;
;   PEAK cycles:
;           2 cycle
;
;   Pseudo code:
;       ========== C ==========
;       if (14th bit == 1)
;           set(bits 14 to 16 HIGH)
;       =======================
;
;   Parameters:
;           sample_reg
;
;   Returns:
;           sample_reg
;************************************************************************************
m_sign_ext_16bit    .macro  sample_reg
    qbbc   SKIP_SIGN_EXT?, sample_reg, 13    ; bit position: (1<<13)
    or     sample_reg.b1, sample_reg.b1, 0xE0
SKIP_SIGN_EXT?:
    .endm

;************************************************************************************
;   Macro: m_sign_ext_16to32_bits
;
;   For sign extension from 16 to 32 bits
;
;   PEAK cycles:
;           2 cycle
;
;   Pseudo code:
;       ========== C ==========
;       if (16th bit == 1)
;           set(bits 17 to 32 HIGH)
;       =======================
;
;   Parameters:
;           sample_reg
;
;   Returns:
;           sample_reg
;************************************************************************************
m_sign_ext_16to32_bits      .macro  sample_reg
    qbbc   SKIP_SIGN_EXT?, sample_reg, 15    ; bit position: (1<<15)
    ldi    sample_reg.w2, 0xFFFF
SKIP_SIGN_EXT?:
    .endm

;************************************************************************************
;   Macro: m_sign_ext_24to32_bits
;
;   For sign extension from 24 to 32 bits
;
;   PEAK cycles:
;           2 cycle
;
;   Pseudo code:
;       ========== C ==========
;       // set(bits 25 to 32 LOW)
;       if (24th bit == 1)
;           set(bits 25 to 32 HIGH)
;       =======================
;
;   Parameters:
;           sample_reg
;
;   Returns:
;           sample_reg
;************************************************************************************
m_sign_ext_24to32_bits      .macro  sample_reg
    ; ldi    sample_reg.b3, 0x00
    qbbc   SKIP_SIGN_EXT?, sample_reg, 23    ; bit position: (1<<23)
    ldi    sample_reg.b3, 0xFF
SKIP_SIGN_EXT?:
    .endm

    .endif  ; __adc_preprocess_macros_h

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

;************************************************************************************
;   File:     time_macros.inc
;
;   Brief:    Timing Control Related Macros
;************************************************************************************
    .if    !$defined("__time_macros_inc")
__time_macros_inc    .set 1

; Symbol  PRU_CLK_CYC_PRD_CONST  will be imported from sysconfig generated file
;   |   333MHz   |   3   |
;   |   250MHz   |   4   |
;   |   200MHz   |   5   |
; TODO: For other clocks??

;************************************************************************************
;   Macro: m_wait_cycles
;
;   Wait for specified number of cpu cycles
;
;   PEAK cycles:
;           count cycles (1 for loop inst. + (count - 1) * NOP inst.)
;
;   Pseudo code:
;       ========== C ==========
;       wait(count-cycles);
;       =======================
;
;   Parameters:
;           count       - number of cycles to wait
;************************************************************************************
m_wait_cycles       .macro count
    loop    endloop?, count - 1
    NOP
endloop?:
    .endm

;************************************************************************************
;   Macro: m_wait_nano_sec
;
;   Wait for specified number of nano seconds
;
;   PEAK cycles:
;           $ceil(time/PRU_CLK_CYC_PRD_CONST) cycles
;
;   Pseudo code:
;       ========== C ==========
;       wait(time);
;       =======================
;
;   Parameters:
;           time        - number of nano seconds to wait
;************************************************************************************
m_wait_nano_sec     .macro  time
    m_wait_cycles   (time + PRU_CLK_CYC_PRD_CONST -1)/PRU_CLK_CYC_PRD_CONST; $ceil(time/PRU_CLK_CYC_PRD_CONST) not working
    .endm

    .endif  ; __time_macros_inc
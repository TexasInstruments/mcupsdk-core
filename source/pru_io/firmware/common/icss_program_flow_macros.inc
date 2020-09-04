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
;   File:   icss_program_flow_macros.inc
;
;   DRAM Memory Used:
;           8 Bytes
;************************************************************************************
    .if    !$defined("__icss_program_flow_macros_inc")
__icss_program_flow_macros_inc    .set 1

    ; .include "icss_struct_defines.inc"
    .include "icss_constant_defines.inc"

; -----------------------------------------------
;  Configuration parameters memory map:
;                             RSV_PRGM_FLOW_MEM
;        RSV_PRGM_FLOW_MEM_OFFSET |---------|
;  Index value acc. to section -  | 4 Bytes |
;                                 |---------|
;  Additional data             -  | 4 Bytes |
;                                 |---------|
; -----------------------------------------------
RSV_PRGM_FLOW_MEM           .set    PRU_DMEM_ADDR
RSV_PRGM_FLOW_MEM_OFFSET    .set    0xFC   ; 8 byte length

SECTION1_ID                 .set   0
SECTION2_ID                 .set   1
SECTION3_ID                 .set   2
SECTION4_ID                 .set   3
SECTION5_ID                 .set   4
SECTION6_ID                 .set   5
SECTION7_ID                 .set   6
SECTION8_ID                 .set   7
SECTION9_ID                 .set   8
SECTION10_ID                .set   9

;************************************************************************************
;   Macro: m_polling_check
;
;   Check if there is some data to act upon in RSV_PRGM_FLOW_MEM
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       if !(DRAM(RSV_PRGM_FLOW_MEM + RSV_PRGM_FLOW_MEM_OFFSET))
;       GOTO POLLING_TASK
;       =======================
;
;   Parameters:
;           POLLING_TASK  -  Task to jump to in case data is present
;           temp_reg      -  Temporary register (32 bit)
;************************************************************************************
m_polling_check         .macro  temp_reg, POLLING_TASK
    lbco    &temp_reg, RSV_PRGM_FLOW_MEM, RSV_PRGM_FLOW_MEM_OFFSET, 4
    qbne    POLLING_TASK, temp_reg, 0
    .endm

;************************************************************************************
;   Macro: m_jump_to_task
;
;   Jump to the respective code section according to the value in RSV_PRGM_FLOW_MEM
;
;   PEAK cycles:
;           (3 + total sections) cycle (can be reduced by dividing into odd and even sections)
;
;   Pseudo code:
;       ========== C ==========
;       GOTO SECTION(DRAM(RSV_PRGM_FLOW_MEM + RSV_PRGM_FLOW_MEM_OFFSET))
;       =======================
;
;   Parameters:
;           sections      -  Total number of code sections
;           temp_reg      -  Temporary register (32 bit)
;           SECTIONn      -  SECTIONn symbol to jump to if the value is SECTIONn_ID
;************************************************************************************
m_jump_to_task          .macro  sections, temp_reg, SECTION1, SECTION2, SECTION3, SECTION4, SECTION5, SECTION6, SECTION7, SECTION8, SECTION9, SECTION10
    ; do not need this if same temp_reg is passed after <m_polling_check>
    lbco    &temp_reg, RSV_PRGM_FLOW_MEM, RSV_PRGM_FLOW_MEM_OFFSET, 4
    .if     sections > 0
    qbeq    SECTION1,  temp_reg, SECTION1_ID
    .endif
    .if     sections > 1
    qbeq    SECTION2,  temp_reg, SECTION2_ID
    .endif
    .if     sections > 2
    qbeq    SECTION3,  temp_reg, SECTION3_ID
    .endif
    .if     sections > 3
    qbeq    SECTION4,  temp_reg, SECTION4_ID
    .endif
    .if     sections > 4
    qbeq    SECTION5,  temp_reg, SECTION5_ID
    .endif
    .if     sections > 5
    qbeq    SECTION6,  temp_reg, SECTION6_ID
    .endif
    .if     sections > 6
    qbeq    SECTION7,  temp_reg, SECTION7_ID
    .endif
    .if     sections > 7
    qbeq    SECTION8,  temp_reg, SECTION8_ID
    .endif
    .if     sections > 8
    qbeq    SECTION9,  temp_reg, SECTION9_ID
    .endif
    .if     sections > 9
    qbeq    SECTION10, temp_reg, SECTION10_ID
    .endif

    .endm

;************************************************************************************
;   Macro: m_clr_prgm_flow_mem
;
;   Clear RSV_PRGM_FLOW_MEM
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       DRAM(RSV_PRGM_FLOW_MEM + RSV_PRGM_FLOW_MEM_OFFSET) = 0
;       =======================
;
;   Parameters:
;           temp_reg      -  Temporary register (32 bit)
;************************************************************************************
m_clr_prgm_flow_mem       .macro  temp_reg
    ldi     temp_reg, 0
    sbco    &temp_reg, RSV_PRGM_FLOW_MEM, RSV_PRGM_FLOW_MEM_OFFSET, 4
    .endm

;************************************************************************************
;   Macro: m_prgm_flow_jump_on_intr
;
;   Check interrupt status on INTR_BIT of register R31. This will be set by other core
;   to indicate this core to jump to a different code section for execution.
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       __R31(INTR_BIT) == 1
;       =======================
;
;   Parameters:
;           EVENT_NUM     -  Interrupt event number
;           INTR_BIT      -  Interrupt bit on R31 register where interrupt status will be available
;           sections      -  Total number of code sections
;           temp_reg      -  Temporary register (32 bit)
;           SECTIONn      -  SECTIONn symbol to jump to if the value is SECTIONn_ID
;************************************************************************************
m_prgm_flow_jump_on_intr       .macro  EVENT_NUM, INTR_BIT, sections, temp_reg, SECTION1, SECTION2, SECTION3, SECTION4, SECTION5, SECTION6, SECTION7, SECTION8, SECTION9, SECTION10
    qbbc    SKIP?, R31, INTR_BIT
    ldi     temp_reg, EVENT_NUM
    sbco    &temp_reg, ICSS_INTC_CONST, 0x24, 4
    m_jump_to_task  sections, temp_reg, SECTION1, SECTION2, SECTION3, SECTION4, SECTION5, SECTION6, SECTION7, SECTION8, SECTION9, SECTION10
SKIP?:
    .endm

    .endif  ; __icss_program_flow_macros_inc

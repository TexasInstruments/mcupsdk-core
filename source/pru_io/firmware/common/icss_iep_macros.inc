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
;   File:     icss_iep_macros.inc
;
;   Brief:    PRU IEP related macro definitions
;
;   Includes:
;            icss_constant_defines.inc
;            icss_iep_regs.inc
;************************************************************************************
    .if    !$defined("__icss_iep_macros_inc")
__icss_iep_macros_inc    .set 1

    .include "icss_constant_defines.inc"
    .include "icss_iep_regs.inc"

;************************************************************************************
;   Macro: m_set_iep_sync_pwidth_reg
;
;   Set value of iep_sync_pwidth_reg. Defines the number of clock cycles SYNC0/1
;   will be high.
;       Note if SYNC0/1 is disabled during pulse width time (that is,
;       SYNC_CTRL[SYNC0_EN | SYNC1_EN | SYNC_EN] = 0), the ongoing pulse will be
;       terminated.
;       0h = 1 clock cycle.
;       1h = 2 clock cycles.
;       Nh = N+1 clock cycles.
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP0_1_CONST + ICSS_IEP_SYNC_PWIDTH_REG - 0x0100, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_1_CONST + ICSS_IEP_SYNC_PWIDTH_REG - 0x0100, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_sync_pwidth_reg
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_sync_pwidth_reg          .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP0_1_CONST, ICSS_IEP_SYNC_PWIDTH_REG - 0x0100, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_1_CONST, ICSS_IEP_SYNC_PWIDTH_REG - 0x0100, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_global_cfg_reg
;
;   Set value of iep_global_cfg_reg.
;   Bit   | Field       | Type | Reset | Description
;   19-8  | CMP_INC     | R/W  | 5h    | Defines the increment value when compensation is active
;   7-4   | DEFAULT_INC | R/W  | 5h    | Defines the default increment value
;   0     | CNT_ENABLE  | R/W  | 0h    | Counter enable.
;                                      |  0h = Disables the counter. The counter maintains the current count.
;                                      |  1h = Enables the counter.
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_GLOBAL_CFG_REG, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_GLOBAL_CFG_REG, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_global_cfg_reg
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_global_cfg_reg   .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_GLOBAL_CFG_REG, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_count_reg0
;
;   Set value of iep_count_reg0.
;   When Shadow Mode is disabled, this is the 64-bit count value
;   (lower 32-bits). The count is incremented (DEFAULT_INC or
;   CMP_INC) on every positive edge of ICSSGn_IEP_CLK (200MHz)
;   or ICSSGn_ICLK. In 64-bit mode, reading this value locks the
;   IEP_COUNT_REG1[31-0] COUNT_HI value.
;   When Shadow Mode is enabled, the counter will be in 32-bit mode.
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_COUNT_REG, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_COUNT_REG, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_count_reg0
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_count_reg0       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_COUNT_REG, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_count_reg1
;
;   Set value of iep_count_reg1.
;   When Shadow Mode is disabled:
;   The master upper 32-bits of the 64-bit count value. The count
;   is incremented (DEFAULT_INC or CMP_INC) on every positive
;   edge of ICSSGn_IEP_CLK (200MHz) or ICSSGn_ICLK. This
;   value only updates when software reads IEP_COUNT_REG0[31-0]
;   COUNT_LO.
;   When Shadow Mode is enabled:
;   This counts the counter reset events caused by CMP0. But, limited
;   to 24-bit counter, so software must clear before this threshold is met.
;   Any writes to COUNT_LO or COUNT_HI will reset COUNT_HI to 0.
;   Max COUNT_HI is 0xFF_FFFF and will stop at this value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_COUNT_REG + 4, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_COUNT_REG + 4, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_count_reg1
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_count_reg1       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG + 4, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_COUNT_REG + 4, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp_cfg_reg
;
;   Set value of iep_cmp_cfg_reg.
;   Bit   | Field           | Type | Reset | Description
;   31-18 | RESERVED        | R/W  | X     |
;   17    | SHADOW_EN       | R/W  | 0h    |
;   16-1  | CMP_EN          | R/W  | 0h    | Enable bits for each of the compare registers
;                                          | 0h = Disables CMPj/k Event
;                                          | 1h = Enables CMPj/k Event
;                                          | CMP_EN[0] (bit 1 of register) maps to CMP0 event
;   0     | CMP0_RST_CNT_EN | R/W  | 0h    | Enable the reset of the counter
;                                          | 0h = Disable
;                                          | 1h = Enable the reset of the counter if a CMP0 event occurs
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP_CFG_REG, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP_CFG_REG, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp_cfg_reg
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp_cfg_reg      .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP_CFG_REG, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp0_reg0
;
;   Set value of iep_cmp0_reg0. Compare 0 low value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP0_REG, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP0_REG, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp0_reg0
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp0_reg0       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP0_REG, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP0_REG, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp0_reg1
;
;   Set value of iep_cmp0_reg1. Compare 0 high value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP0_REG + 4, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP0_REG + 4, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp0_reg1
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp0_reg1       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP0_REG + 4, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP0_REG + 4, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp1_reg0
;
;   Set value of iep_cmp1_reg0. Compare 1 low value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP1_REG, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP1_REG, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp1_reg0
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp1_reg0       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP1_REG, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP1_REG, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp1_reg1
;
;   Set value of iep_cmp1_reg1. Compare 1 high value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP1_REG + 4, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP1_REG + 4, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp1_reg1
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp1_reg1       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP1_REG + 4, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP1_REG + 4, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp2_reg0
;
;   Set value of iep_cmp2_reg0. Compare 2 low value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP2_REG, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP2_REG, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp2_reg0
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp2_reg0       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP2_REG, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP2_REG, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp2_reg1
;
;   Set value of iep_cmp2_reg1. Compare 2 high value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP2_REG + 4, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP2_REG + 4, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp2_reg1
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp2_reg1       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP2_REG + 4, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP2_REG + 4, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp3_reg0
;
;   Set value of iep_cmp3_reg0. Compare 3 low value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP3_REG, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP3_REG, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp3_reg0
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp3_reg0       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP3_REG, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP3_REG, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp3_reg1
;
;   Set value of iep_cmp3_reg1. Compare 3 high value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP3_REG + 4, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP3_REG + 4, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp3_reg1
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp3_reg1       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP3_REG + 4, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP3_REG + 4, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp4_reg0
;
;   Set value of iep_cmp4_reg0. Compare 4 low value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP4_REG, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP4_REG, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp4_reg0
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp4_reg0       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP4_REG, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP4_REG, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp4_reg1
;
;   Set value of iep_cmp4_reg1. Compare 4 high value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP4_REG + 4, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP4_REG + 4, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp4_reg1
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp4_reg1       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP4_REG + 4, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP4_REG + 4, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp5_reg0
;
;   Set value of iep_cmp5_reg0. Compare 5 low value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP5_REG, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP5_REG, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp5_reg0
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp5_reg0       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP5_REG, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP5_REG, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp5_reg1
;
;   Set value of iep_cmp5_reg1. Compare 5 high value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP5_REG + 4, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP5_REG + 4, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp5_reg1
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp5_reg1       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP5_REG + 4, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP5_REG + 4, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp6_reg0
;
;   Set value of iep_cmp6_reg0. Compare 6 low value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP6_REG, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP6_REG, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp6_reg0
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp6_reg0       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP6_REG, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP6_REG, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp6_reg1
;
;   Set value of iep_cmp6_reg1. Compare 6 high value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP6_REG + 4, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP6_REG + 4, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp6_reg1
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp6_reg1       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP6_REG + 4, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP6_REG + 4, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp7_reg0
;
;   Set value of iep_cmp7_reg0. Compare 7 low value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP7_REG, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP7_REG, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp7_reg0
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp7_reg0       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP7_REG, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP7_REG, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp7_reg1
;
;   Set value of iep_cmp7_reg1. Compare 7 high value
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP7_REG + 4, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP7_REG + 4, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp7_reg1
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp7_reg1       .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP7_REG + 4, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP7_REG + 4, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_cmp_status_reg
;
;   Set value of iep_cmp_status_reg.
;   Status bit for each of the compare registers. "Match" indicates the
;   current counter is greater than or equal to the compare value. Note it
;   is the firmware's responsibility to handle the IEP overflow.
;   0h = No match has occured
;   1h = A match occured. The associated hardware event signal will
;   assert and remain high until the status is cleared.
;   Shadow Mode Disabled: Any write to a CMP register will cause the
;   corresponding CMP_STATUS bit to get cleared
;   Shadow Mode Enabled: Any write to the active CMP<n>_0 will
;   cause the corresponding CMP_STATUS to get cleared. Any write
;   to the shadow CMP<n>_1 will have no effect on pending state
;   If PWM_EFC_EN is set, hardware will auto clear the bit on next clock
;   cycle.
;
;   PEAK cycles:
;           3 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP_CONST + ICSS_IEP_CMP_STATUS_REG, 2);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_0_CONST + ICSS_IEP_CMP_STATUS_REG, 2);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_cmp_status_reg
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_cmp_status_reg       .macro  iep_inst, temp_reg, value
    ldi     temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 2
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_0_CONST, ICSS_IEP_CMP_STATUS_REG, 2
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_sync_ctrl_reg
;
;   Set value of iep_sync_ctrl_reg.
;   Refer table Table 6-1614 TRM
;   | 0:SYNC_EN | 1:SYNC0_EN | 2:SYNC1_EN |
;
;   PEAK cycles:
;           3 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP0_1_CONST + ICSS_IEP_SYNC_CTRL_REG - 0x0100, 2);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_1_CONST + ICSS_IEP_SYNC_CTRL_REG - 0x0100, 2);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_sync_ctrl_reg
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_sync_ctrl_reg        .macro  iep_inst, temp_reg, value
    ldi     temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP0_1_CONST, ICSS_IEP_SYNC_CTRL_REG - 0x0100, 2
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_1_CONST, ICSS_IEP_SYNC_CTRL_REG - 0x0100, 2
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_sync_start_reg
;
;   Set value of iep_sync_start_reg.
;    Defines the start time after the activation event.
;    0h = 1 clock cycle delay.
;    Nh = N+1 clock cycles delay
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP0_1_CONST + ICSS_IEP_SYNC_START_REG - 0x0100, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_1_CONST + ICSS_IEP_SYNC_START_REG - 0x0100, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_sync_start_reg
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_sync_start_reg        .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP0_1_CONST, ICSS_IEP_SYNC_START_REG - 0x0100, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_1_CONST, ICSS_IEP_SYNC_START_REG - 0x0100, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_sync0_period_reg
;
;   Set value of iep_sync0_period_reg.
;    Defines the period between the rising edges of SYNC0.
;    (Check SYNC0_CYCLIC_EN bit in IEP_SYNC_CTRL_REG is set)
;    0h = Reserved
;    1h = 2 clk cycles period
;    N: N+1 clk cycles period
;
;   PEAK cycles:
;           4 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP0_1_CONST + ICSS_IEP_SYNC0_PERIOD_REG - 0x0100, 4);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_1_CONST + ICSS_IEP_SYNC0_PERIOD_REG - 0x0100, 4);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_sync0_period_reg
;           temp_reg  - Temporary register (32 bit)
;
;************************************************************************************
m_set_iep_sync0_period_reg      .macro  iep_inst, temp_reg, value
    ldi32   temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP0_1_CONST, ICSS_IEP_SYNC0_PERIOD_REG - 0x0100, 4
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_1_CONST, ICSS_IEP_SYNC0_PERIOD_REG - 0x0100, 4
    .endif
    .endm

;************************************************************************************
;   Macro: m_set_iep_sync0_stat_reg
;
;   Set value of iep_sync0_stat_reg.
;    0: SYNC0_PEND
;       SYNC0 pending state
;       0h = SYNC0 is not pending
;       1h = SYNC0 is pending or has occurred when SYNC0_ACK_EN = 0
;       (Disable). Write "1" to clear
;
;   PEAK cycles:
;           3 cycle
;
;   Pseudo code:
;       ========== C ==========
;       temp_reg = value;
;       if (iep_inst == 0)
;           memcpy(offset(temp_reg), ICSS_IEP0_1_CONST + ICSS_IEP_SYNC0_STATUS_REG - 0x0100, 1);
;       else if (iep_inst == 1)
;           memcpy(offset(temp_reg), ICSS_IEP1_1_CONST + ICSS_IEP_SYNC0_STATUS_REG - 0x0100, 1);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           value     - Value to store in iep_sync0_stat_reg
;           temp_reg  - Temporary register (8 bit)
;
;************************************************************************************
m_set_iep_sync0_stat_reg      .macro  iep_inst, temp_reg, value
    ldi     temp_reg, value
    .if     iep_inst = 0
    sbco    &temp_reg, ICSS_IEP0_1_CONST, ICSS_IEP_SYNC0_STATUS_REG - 0x0100, 1
    .elseif iep_inst = 1
    sbco    &temp_reg, ICSS_IEP1_1_CONST, ICSS_IEP_SYNC0_STATUS_REG - 0x0100, 1
    .endif
    .endm

;************************************************************************************
;   Macro: m_get_iep_sync0_stat_reg
;
;   Get value of get_iep_sync0_stat_reg.
;    0: SYNC0_PEND
;       SYNC0 pending state
;       0h = SYNC0 is not pending
;       1h = SYNC0 is pending or has occurred when SYNC0_ACK_EN = 0
;       (Disable). Write "1" to clear
;
;   PEAK cycles:
;           3 cycle
;
;   Pseudo code:
;       ========== C ==========
;       if (iep_inst == 0)
;           memcpy(ICSS_IEP0_1_CONST + ICSS_IEP_SYNC0_STATUS_REG - 0x0100, offset(reg_val), 1);
;       else if (iep_inst == 1)
;           memcpy(ICSS_IEP1_1_CONST + ICSS_IEP_SYNC0_STATUS_REG - 0x0100, offset(reg_val), 1);
;       =======================
;
;   Parameters:
;           iep_inst  - IEP Instance (0 or 1)
;           reg_val   - Register to store value of iep_sync0_stat_reg (8 bit)
;
;   Returns:
;           reg_val   - Value of register iep_sync0_stat_reg (8 bit)
;************************************************************************************
m_get_iep_sync0_stat_reg      .macro  iep_inst, reg_val
    .if     iep_inst = 0
    lbco    &reg_val, ICSS_IEP0_1_CONST, ICSS_IEP_SYNC0_STATUS_REG - 0x0100, 1
    .elseif iep_inst = 1
    lbco    &reg_val, ICSS_IEP1_1_CONST, ICSS_IEP_SYNC0_STATUS_REG - 0x0100, 1
    .endif
    .endm

    .endif  ; __icss_iep_macros_inc
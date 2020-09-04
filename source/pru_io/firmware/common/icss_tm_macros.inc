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

; file:     icss_tm_macros.inc
;
; brief:    PRU Task Manager Related Macro definitions

    .if !$isdefed("____icss_tm_macros_h")
____icss_tm_macros_h   .set    1

    .include "icss_xfer_defines.inc"
    .include "icss_struct_defines.inc"

    ;Task IDs
TS1_S0  .set    0
TS1_S1  .set    1
TS1_S2  .set    2
TS1_S3  .set    3
TS1_S4  .set    4
TS2_S0  .set    5
TS2_S1  .set    6
TS2_S2  .set    7
TS2_S3  .set    8
TS2_S4  .set    9

;************************************************************************************
;   Macro: m_pru_tasks_mgr_global_cfg_reg
;
;   Set value of tasks_mgr_global_cfg_reg.
;   Subtasks:     0: Disabled   1: Enabled
;   Bit  | Field          | Type | Reset | Description
;   11   | TS2_EN_S4      | R/W  | 0h    | TS2 Sub4
;   10   | TS2_EN_S3      | R/W  | 0h    | TS2 Sub3
;   9    | TS2_EN_S2      | R/W  | 0h    | TS2 Sub2
;   8    | TS2_EN_S1      | R/W  | 0h    | TS2 Sub1
;   7    | TS2_EN_S0      | R/W  | 0h    | TS2 Sub0
;   6    | TS1_EN_S4      | R/W  | 0h    | TS1 Sub4
;   5    | TS1_EN_S3      | R/W  | 0h    | TS1 Sub3
;   4    | TS1_EN_S2      | R/W  | 0h    | TS1 Sub2
;   3    | TS1_EN_S1      | R/W  | 0h    | TS1 Sub1
;   2    | TS1_EN_S0      | R/W  | 0h    | TS1 Sub0
;   1-0  | TASKS_MGR_MODE | R/W  | 0h    | TaskSwap Mode
;                                        |   0:  Disabled
;                                        |   1:  RXTX
;                                        |   2:  General_HW
;
;   PEAK cycles:
;           5 cycle
;
;   Pseudo code:
;       ========== C ==========
;       TEMP_REG.reg1.x = TM_BASE;
;       TEMP_REG.reg2.x = value;
;       memcpy(offset(TEMP_REG.reg2.x), TEMP_REG.reg1.x, 2);
;       =======================
;
;   Parameters:
;       TEMP_REG  -  Register Structure with atleast 2 Registers
;       TM_BASE   -  Base address of the corresponding PRU's task manager
;       value     -  Value to store in tasks_mgr_global_cfg_reg
;
;************************************************************************************
m_pru_tasks_mgr_global_cfg_reg      .macro  TEMP_REG, TM_BASE, value
    ldi32   TEMP_REG.reg1.x, TM_BASE
    ldi     TEMP_REG.reg2.x, value
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, 0, 2
    .endm

;************************************************************************************
;   Macro: m_pru_tm_rst
;
;   Clears task manager status
;
;   PEAK cycles:
;           11 cycle
;
;   Invokes:
;           <m_pru_tasks_mgr_global_cfg_reg>
;
;   Parameters:
;       TEMP_REG  -  Register Structure with atleast 2 Registers
;       TM_BASE   -  Base address of the corresponding PRU's task manager
;
;************************************************************************************
m_pru_tm_rst    .macro  TEMP_REG, TM_BASE
    ; clear task manager status which is sticky after debug halt
    m_pru_tasks_mgr_global_cfg_reg  TEMP_REG, TM_BASE, 0x0ffe
    ; m_pru_tm_yield
    xin     TM_YIELD_XID, &R0.b3, 1
    m_pru_tasks_mgr_global_cfg_reg  TEMP_REG, TM_BASE, 0
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_set_cfg_gpmode_enable
;
;  Task Manager CFG for PRU to setup TM in General Purpose Mode.
;
;   PeakCycles:
;       5 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write to PRU0_TM_GLOBAL_CFG register to set tasks_mgr_mode to 2.
;       Also enables all subtasks.
;      (end code)
;
;   Parameters:
;       TEMP_REG  -  Register Structure with atleast 2 Registers
;       TM_BASE   -  Base address of the corresponding PRU's task manager
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_set_cfg_gpmode_enable .macro TEMP_REG, TM_BASE
    ldi32   TEMP_REG.reg1.x, TM_BASE
    ldi     TEMP_REG.reg2.x, (0x2 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TASKS_MGR_MODE_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S0_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S1_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S2_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S3_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S4_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S0_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S1_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S2_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S3_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S4_SHIFT)
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG, 4
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_pc_set
;
;  Task Manager set the PC value for given task.
;
;   PeakCycles:
;       7 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       3 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write ts_pc to register for ts_number.
;      (end code)
;
;   Parameters:
;       TEMP_REG   -  Register Structure with atleast 3 Registers
;       TM_BASE    -  Base address of the corresponding PRU's task manager
;       ts_pc      -  Program counter value for task
;       ts_number  -  Task number (TS1_S0 to TS2_S4)
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_pc_set .macro TEMP_REG, TM_BASE, ts_pc, ts_number
    ldi32   TEMP_REG.reg1.x, TM_BASE
    ldi     TEMP_REG.reg2.x, $CODE(ts_pc)
    ldi     TEMP_REG.reg3.x, (ts_number << 2)
    add     TEMP_REG.reg3.x, TEMP_REG.reg3.x, 8
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, TEMP_REG.reg3.x, 4
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_ts1_pc_set
;
;  Task Manager set the PC values for TS1_S{0 to 4}
;
;   PeakCycles:
;       14 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       4 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write ts1_pc_s0 to TS1_PC_S0 register
;       Write ts1_pc_s1 to TS1_PC_S1 register
;       Write ts1_pc_s2 to TS1_PC_S2 register
;       Write ts1_pc_s3 to TS1_PC_S3 register
;       Write ts1_pc_s4 to TS1_PC_S4 register
;      (end code)
;
;   Parameters:
;       TEMP_REG    -  Register Structure with atleast 4 Registers
;       TM_BASE     -  Base address of the corresponding PRU's task manager
;       ts1_pc_s0   -  Program counter value for TS1_S0
;       ts1_pc_s1   -  Program counter value for TS1_S1
;       ts1_pc_s2   -  Program counter value for TS1_S2
;       ts1_pc_s3   -  Program counter value for TS1_S3
;       ts1_pc_s4   -  Program counter value for TS1_S4
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_ts1_pc_set .macro TEMP_REG, TM_BASE, ts1_pc_s0, ts1_pc_s1, ts1_pc_s2, ts1_pc_s3, ts1_pc_s4
    ldi32   TEMP_REG.reg1.x, TM_BASE
    ldi     TEMP_REG.reg2.x, $CODE(ts1_pc_s0)
    ldi     TEMP_REG.reg3.x, $CODE(ts1_pc_s1)
    ldi     TEMP_REG.reg4.x, $CODE(ts1_pc_s2)
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS1_PC_S0, 12
    ldi     TEMP_REG.reg2.x, $CODE(ts1_pc_s3)
    ldi     TEMP_REG.reg3.x, $CODE(ts1_pc_s4)
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS1_PC_S3, 8
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_ts2_pc_set
;
;  Task Manager set the PC values for TS2_S{0 to 4}
;
;   PeakCycles:
;       14 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       4 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       TEMP_REG -> Register Structure with atleast 4 Registers
;       TM_BASE -> Base address of the corresponding PRU's task manager
;       Write ts2_pc_s0 to TS2_PC_S0 register
;       Write ts2_pc_s1 to TS2_PC_S1 register
;       Write ts2_pc_s2 to TS2_PC_S2 register
;       Write ts2_pc_s3 to TS2_PC_S3 register
;       Write ts2_pc_s4 to TS2_PC_S4 register
;      (end code)
;
;   Parameters:
;       ts2_pc_s0  -  Program counter value for TS2_S0
;       ts2_pc_s1  -  Program counter value for TS2_S1
;       ts2_pc_s2  -  Program counter value for TS2_S2
;       ts2_pc_s3  -  Program counter value for TS2_S3
;       ts2_pc_s4  -  Program counter value for TS2_S4
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_ts2_pc_set .macro TEMP_REG, TM_BASE, ts2_pc_s0, ts2_pc_s1, ts2_pc_s2, ts2_pc_s3, ts2_pc_s4
    ldi32   TEMP_REG.reg1.x, TM_BASE
    ldi     TEMP_REG.reg2.x, $CODE(ts2_pc_s0)
    ldi     TEMP_REG.reg3.x, $CODE(ts2_pc_s1)
    ldi     TEMP_REG.reg4.x, $CODE(ts2_pc_s2)
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS2_PC_S0, 12
    ldi     TEMP_REG.reg2.x, $CODE(ts2_pc_s3)
    ldi     TEMP_REG.reg3.x, $CODE(ts2_pc_s4)
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS2_PC_S3, 8
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_disable_task
;
;  Disable given task in Task Manager.
;
;   PeakCycles:
;       8 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       3 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Disable given task (ts_number) in Task Manager.
;      (end code)
;
;   Parameters:
;       TEMP_REG   -  Register Structure with atleast 3 Registers
;       TM_BASE    -  Base address of the corresponding PRU's task manager
;       ts_number  -  Task number (TS1_S0 to TS2_S4)
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_disable_task .macro TEMP_REG, TM_BASE, ts_number
    ldi32   TEMP_REG.reg1.x, TM_BASE
    lbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG, 4
    clr     TEMP_REG.reg2.x, TEMP_REG.reg2.x, (ts_number + 2)
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG, 4
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_enable_task
;
;  Enable given task in Task Manager.
;
;   PeakCycles:
;       8 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       3 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Enable given task (ts_number) in Task Manager.
;      (end code)
;
;   Parameters:
;       TEMP_REG   -  Register Structure with atleast 3 Registers
;       TM_BASE    -  Base address of the corresponding PRU's task manager
;       ts_number  -  Task number (TS1_S0 to TS2_S4)
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_enable_task .macro TEMP_REG, TM_BASE, ts_number
    ldi32   TEMP_REG.reg1.x, TM_BASE
    lbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG, 4
    set     TEMP_REG.reg2.x, TEMP_REG.reg2.x, (ts_number + 2)
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG, 4
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_set_cfg_gpmode_ts1_mux_set
;
;  Task Manager CFG for PRU to setup TM GP Mode Mux for TS1.
;
;   PeakCycles:
;       11 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       3 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write to TS1_GEN_CFG1 and TS1_GEN_CFG2 register to setup GP mode mux for TS1.
;      (end code)
;
;   Parameters:
;       TEMP_REG -> Register Structure with atleast 3 Registers
;
;       TM_BASE -> Base address of the corresponding PRU's task manager
;
;       ts1_mx_s0 -> Mux value for TS1_S0
;
;       ts1_mx_s1 -> Mux value for TS1_S1
;
;       ts1_mx_s2 -> Mux value for TS1_S2
;
;       ts1_mx_s3 -> Mux value for TS1_S3
;
;       ts1_mx_s4 -> Mux value for TS1_S4
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_set_cfg_gpmode_ts1_mux_set .macro TEMP_REG, TM_BASE, ts1_mx_s0, ts1_mx_s1, ts1_mx_s2, ts1_mx_s3, ts1_mx_s4
    ldi32   TEMP_REG.reg1.x, TM_BASE
    zero    &TEMP_REG.reg2.x, 8
    or      TEMP_REG.reg2.x_b.b0, TEMP_REG.reg2.x_b.b0, ts1_mx_s0
    or      TEMP_REG.reg2.x_b.b1, TEMP_REG.reg2.x_b.b1, ts1_mx_s1
    or      TEMP_REG.reg2.x_b.b2, TEMP_REG.reg2.x_b.b2, ts1_mx_s2
    or      TEMP_REG.reg2.x_b.b3, TEMP_REG.reg2.x_b.b3, ts1_mx_s3
    or      TEMP_REG.reg3.x_b.b0, TEMP_REG.reg3.x_b.b0, ts1_mx_s4
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS1_GEN_CFG1, 5
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_set_cfg_gpmode_ts2_mux_set
;
;  Task Manager CFG for PRU to setup TM GP Mode Mux for TS2.
;
;   PeakCycles:
;       11 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       3 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write to TS2_GEN_CFG1 and TS2_GEN_CFG2 register to setup GP mode mux for TS2.
;      (end code)
;
;   Parameters:
;       TEMP_REG -> Register Structure with atleast 3 Registers
;
;       TM_BASE -> Base address of the corresponding PRU's task manager
;
;       ts2_mx_s0 -> Mux value for TS1_S0
;
;       ts2_mx_s1 -> Mux value for TS1_S1
;
;       ts2_mx_s2 -> Mux value for TS1_S2
;
;       ts2_mx_s3 -> Mux value for TS1_S3
;
;       ts2_mx_s4 -> Mux value for TS1_S4
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_set_cfg_gpmode_ts2_mux_set .macro TEMP_REG, TM_BASE, ts2_mx_s0, ts2_mx_s1, ts2_mx_s2, ts2_mx_s3, ts2_mx_s4
    ldi32   TEMP_REG.reg1.x, TM_BASE
    zero    &TEMP_REG.reg2.x, 8
    or      TEMP_REG.reg2.x_b.b0, TEMP_REG.reg2.x_b.b0, ts2_mx_s0
    or      TEMP_REG.reg2.x_b.b1, TEMP_REG.reg2.x_b.b1, ts2_mx_s1
    or      TEMP_REG.reg2.x_b.b2, TEMP_REG.reg2.x_b.b2, ts2_mx_s2
    or      TEMP_REG.reg2.x_b.b3, TEMP_REG.reg2.x_b.b3, ts2_mx_s3
    or      TEMP_REG.reg3.x_b.b0, TEMP_REG.reg3.x_b.b0, ts2_mx_s4
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS2_GEN_CFG1, 5
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_yield
;
;  Current Task yield to Task Manager
;
;   PeakCycles:
;       4 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       R0.b3
;
;   PseudoCode:
;      (start code)
;       xin to TM_YIELD_XID with 1 byte
;      (end code)
;
;   Parameters:
;       None
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_yield .macro
    xin     TM_YIELD_XID, &R0.b3, 1
deadloop_tm_yield?:
    jmp     deadloop_tm_yield?
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_yield_only
;
;  Current Task yield to Task Manager
;
;   PeakCycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       R0.b3
;
;   PseudoCode:
;      (start code)
;       xin to TM_YIELD_XID with 1 byte
;      (end code)
;
;   Parameters:
;       None
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_yield_only .macro
    xin     TM_YIELD_XID, &R0.b3, 1
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_yield_self_loop
;
;  Deadloop during Task yield to Task Manager
;
;   PeakCycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       None
;
;   PseudoCode:
;      (start code)
;       Deadloop.
;      (end code)
;
;   Parameters:
;       None
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_yield_self_loop .macro
deadloop_tm_yield?:
    jmp     deadloop_tm_yield?
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_enable
;
;  Task Manager enable
;
;   PeakCycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       None
;
;   PseudoCode:
;      (start code)
;       A new instruction "TSEN 1" is defined for Task Manager Enable
;       Using .word as a workaround because CLPRU doesnt support TSEN instruction
;      (end code)
;
;   Parameters:
;       None
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_enable .macro
;    .word 0x32800000
    tsen    1
    .endm


;******************************************************************************
;
;   Macro: m_pru_tm_disable
;
;  Task Manager disable
;
;   PeakCycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       None
;
;   PseudoCode:
;      (start code)
;       A new instruction "TSEN 0" is defined for Task Manager Disable
;       Using .word as a workaround because CLPRU doesnt support TSEN instruction
;      (end code)
;
;   Parameters:
;       None
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_disable .macro
;     .word 0x32000000
    tsen    0
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_context_save
;
;  Save previous task's context (R0-R29) to Scratch PAD.
;
;   PeakCycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       R0
;
;   PseudoCode:
;      (start code)
;       xout to TM_SAVE_XID with 120 bytes
;      (end code)
;
;   Parameters:
;       None
;
;   Returns:
;None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_context_save .macro bank_id
    xout    bank_id, &R0, 120
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_context_restore
;
;  Restore previous task's context (R0-R29) to Scratch PAD.
;
;   PeakCycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       R0
;
;   PseudoCode:
;      (start code)
;       xin 120 bytes from TM_SAVE_XID SPAD bank
;      (end code)
;
;   Parameters:
;       None
;
;   Returns:
;None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_context_restore .macro bank_id
    xin     bank_id, &R0, 120
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_context_swap
;
;  Save previous task's context (R0-R29) to Scratch PAD and restore current task's context.
;
;   PeakCycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       R0
;
;   PseudoCode:
;      (start code)
;       xchg to TM_SAVE_XID with 120 bytes
;      (end code)
;
;   Parameters:
;       None
;
;   Returns:
;None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_context_swap .macro bank_id
    xchg    bank_id, &R0, 120
    .endm

    .endif ; ____icss_tm_macros_h

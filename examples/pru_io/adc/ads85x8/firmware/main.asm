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
;
;     file:   main.asm
;
;     brief:  PRU ADC Interface

;************************************* includes *************************************
    .include "icss_constant_defines.inc"
    .include "icss_iep_macros.inc"
    .include "icss_iep_regs.inc"
    .include "icss_struct_defines.inc"
    .include "icss_tm_macros.inc"
    .include "icss_gpio_cntrl_macros.inc"
    .include "parallel_interface_macros.inc"
    .include "pru_ipc_macros.inc"
    .include "adc_preprocess_macros.inc"
    .include "icss_program_flow_macros.inc"
    .include "icss_pin_macros.inc"

; For invoking Task Manager Macros
tmp_reg_struct  .sassign r20, Struct_4_Reg

    .if    !$defined("ICSSG0")
ICSSG0      .set 1
    .endif
    .if    !$defined("PRU0")
PRU0        .set 1
    .endif

; Import symbols from sysconfig generated files
    .include "ti_pru_io_config.inc"
    .asg    0x16,       PRGM_FLOW_EVENT
    .asg    31,         PRGM_FLOW_EVENT_BIT

    .cdecls C,  NOLIST
%{
#include "drivers/pruicss/g_v0/cslr_icss_g.h"
%}

;CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

    .global     main
    .sect       ".text"

    .asg	R2,	    ADC_DATA_REG_1
    .asg	R3,	    ADC_DATA_REG_2
    .asg	R4,	    ADC_DATA_REG_3
    .asg	R5,	    ADC_DATA_REG_4
    .asg	R6,	    ADC_DATA_REG_5
    .asg	R7,	    ADC_DATA_REG_6
    .asg	R8,	    ADC_DATA_REG_7
    .asg	R9,	    ADC_DATA_REG_8
    .asg	R10,	TEMP_REG_1

    .if     $defined(PRU0)
    .asg    CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_BASE, PRU_TM_BASE
    .elseif $defined(PRU1)
    .asg    CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU1_MMR_REGS_BASE, PRU_TM_BASE
    .endif

;********
;* MAIN *
;********

main:

init:
    zero	&r0, 120

; PRU_ICSSG Shared RAM (local)      00nn_nn00h, nnnn = c28_pointer[15:0]
    .if     $defined(PRU0)
    ldi     TEMP_REG_1, 0x0100
    .else
    ldi     TEMP_REG_1, 0x0110
    .endif
    sbco    &TEMP_REG_1, c11, 0x28, 2

;----------------------------------------------------------------------------
;   SECTION: IDLE
;   PRU enters the program, executes intialization and then stays in this
;   section until any further command from r5f
;----------------------------------------------------------------------------
IDLE:
    m_prgm_flow_jump_on_intr    PRGM_FLOW_EVENT, PRGM_FLOW_EVENT_BIT, 6, TEMP_REG_1, IDLE, task_manager, sample_transfer, iep, main_loop, reset
    qba     IDLE

task_manager:
    ; disable task manager
    tsen   0
    m_pru_tm_rst    tmp_reg_struct, PRU_TM_BASE ; clear comment inside macro

    ; Task manager configuration
    ; Mode: GP mode
    ; Priority: T1_S1 (highest) > T1_S2 > T1_S3 > T1_S4 > T1_S0 (Lowest)
    ; Priority: T2_S0 (highest) > T2_S1 > T2_S2 > T2_S3 > T2_S4 (Lowest)
    ;****************************************************************************************************
    ; Subtask       State       Function         Event
    ;****************************************************************************************************
    ; T1_S0         Enabled    TM_IEP1_TASK      40 (CMP0)
    ; Rest of the tasks are disabled.
    ;****************************************************************************************************
    m_pru_tm_set_cfg_gpmode_enable tmp_reg_struct,     PRU_TM_BASE   ; 5 cyc. Macro - Setup Task Manager in GP mode.
    ; configure task manager for iep_tasks, TS2 is highest priority and can pre-empt TS1
    m_pru_tm_ts1_pc_set tmp_reg_struct, PRU_TM_BASE, TM_IEP1_TASK, FN_NOP_TASK, FN_NOP_TASK, FN_NOP_TASK, FN_NOP_TASK    ; 11 cyc

    ; m_pru_tm_disable_task tmp_reg_struct, PRU_TM_BASE, TS1_S0
    m_pru_tm_disable_task tmp_reg_struct, PRU_TM_BASE, TS1_S1
    m_pru_tm_disable_task tmp_reg_struct, PRU_TM_BASE, TS1_S2
    m_pru_tm_disable_task tmp_reg_struct, PRU_TM_BASE, TS1_S3
    m_pru_tm_disable_task tmp_reg_struct, PRU_TM_BASE, TS1_S4

    ; set TS1 trigger to IEP1 cmp0 hit event = 40
    m_pru_tm_set_cfg_gpmode_ts1_mux_set tmp_reg_struct, PRU_TM_BASE, 40, 0, 0, 0, 0

; enable task manager instruction
    tsen     1
    qba             IDLE

sample_transfer:
    m_pru_ipc_init  R0.b0, R29, CONFIG_PRU_IPC0_CONFIG_MEM_OFFSET ; Initialize Sample Transfer
    qba             IDLE

iep:
; reset iep1 timer
    m_set_iep_global_cfg_reg   1, TEMP_REG_1, 0x20

; set starting count value
    m_set_iep_count_reg0       1, TEMP_REG_1, 0xffffffff
    m_set_iep_count_reg1       1, TEMP_REG_1, 0xffffffff

; Set SYNC0 pulse width: e.g. 375 ns (125 * 3ns)
    m_set_iep_sync_pwidth_reg  1, TEMP_REG_1, 125

; Set SYNC0 start time: e.g. 500 ns
    m_set_iep_cmp1_reg0        1, TEMP_REG_1, 0

; set CMP0 wrap around mode
    m_set_iep_cmp_cfg_reg      1, TEMP_REG_1, 0x0f  ; CMP2 + CMP1 + CMP0 + wrap around

; set CMP0 period - 40 kHZ 25000 ns, - cycle as we start from 0
; This decides the trigerring period of conversion start signal to ADC
    m_set_iep_cmp0_reg0        1, TEMP_REG_1, ADC_CONVST_IEP_PRD_CONST ; 25000-2

    qba             IDLE

;************************************* sample read loop *************************************
main_loop:
; set default increment to 2 ->  500 MHz IEP clock and enable count
    m_set_iep_global_cfg_reg   1, TEMP_REG_1, 0x21

START:
    ; Wait for Busy to go low indicating conversion has completed
    m_wait_high_pulse       4, 16
    ; ADC always samples data from all channels, independent of how many channels there are in use
    ; Fetch and store channel 1-8 Sample
    m_pru_clr_pin       CS_PIN
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_1, RD_PIN
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_2, RD_PIN
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_3, RD_PIN
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_4, RD_PIN
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_5, RD_PIN
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_6, RD_PIN
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_7, RD_PIN
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_8, RD_PIN
    m_pru_set_pin       CS_PIN
    ; Dummy read remaining channels
    ; m_adc_read_dummy
    ; m_adc_read_dummy
    ; m_adc_read_dummy

    ; Time: (3*8*3ns = 72ns)
    m_sign_ext_32bit    ADC_DATA_REG_1
    m_sign_ext_32bit    ADC_DATA_REG_2
    m_sign_ext_32bit    ADC_DATA_REG_3
    m_sign_ext_32bit    ADC_DATA_REG_4
    m_sign_ext_32bit    ADC_DATA_REG_5
    m_sign_ext_32bit    ADC_DATA_REG_6
    m_sign_ext_32bit    ADC_DATA_REG_7
    m_sign_ext_32bit    ADC_DATA_REG_8

    ldi        R1.b0, &ADC_DATA_REG_1
    m_pru_ipc_send      R1.b0, ADC_DATA_REG_1, R0.b0, R29, CONFIG_PRU_IPC0_RX_INTR_ENABLE, CONFIG_PRU_IPC0_RX_EVENT

    m_prgm_flow_jump_on_intr    PRGM_FLOW_EVENT, PRGM_FLOW_EVENT_BIT, 6, TEMP_REG_1, IDLE, task_manager, sample_transfer, iep, main_loop, reset

    qba    START ; loop
    halt

reset:
; reset iep1 timer
    m_set_iep_global_cfg_reg   1, TEMP_REG_1, 0x20
    m_pru_ipc_init      R0.b0, R29, CONFIG_PRU_IPC0_CONFIG_MEM_OFFSET ; Reset Sample Transfer

TM_IEP1_TASK:   .asmfunc
    m_set_iep_cmp_status_reg    1, TEMP_REG_1, 0x0f       ; clear cmp0, cmp1, cmp2 and cmp3 events
    m_set_iep_cmp2_reg0         1, TEMP_REG_1, 1000-2     ; set CMP2 offset - e.g. 1 us

    ; re-enable SYNC0: clear and set sync_en bit in IEP timer to re-enable trigger
    m_set_iep_sync_ctrl_reg     1, TEMP_REG_1, 0x00    ; clear sync_en (sync0_en, sync1_en)
    m_set_iep_sync_ctrl_reg     1, TEMP_REG_1, 0x0007  ; set sync_en (sync0_en, sync1_en)

    xin     TM_YIELD_XID, &R0.b3, 1
    NOP
    NOP
    .endasmfunc

;************************************************************************************
;
;   Function: FN_NOP_TASK
;
;   PeakCycles:
;       1 cycles
;
;   Invokes:
;       <m_pru_tm_yield>
;
;   PseudoCode:
;       Yield task
;
;   Parameters:
;       None
;
;   Returns:
;       None
;
;   See Also:
;
;************************************************************************************
FN_NOP_TASK:    .asmfunc
    ; m_pru_tm_yield
    xin     TM_YIELD_XID, &R0.b3, 1
    NOP
    NOP                                                       ; 4 cycles
    .endasmfunc

; Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
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
;   File:     main.asm
;************************************************************************************

;************************************* includes *************************************
; Include files for importing symbols and macros
    .include "icss_constant_defines.inc"
    .include "icss_iep_macros.inc"
    .include "icss_struct_defines.inc"

; Import the Chip Support Library Register Address defines
    .cdecls C,  NOLIST
%{
#include "drivers/pruicss/m_v0/cslr_icss_m.h"
%}
; CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

    .global     main
    .sect       ".text"
        .asg	    R0.w2,	data_send_count
        .asg        R1,     TEMP_REG
        .asg	    R2,	    state_change_mask
        .asg	    R3,	    channel0_pulse_len
        .asg	    R4,	    channel1_pulse_len
        .asg	    R5,	    channel2_pulse_len
        .asg	    R6,	    channel3_pulse_len
        .asg	    R7,	    channel4_pulse_len
        .asg	    R8,	    channel5_pulse_len
        .asg	    R9,	    channel6_pulse_len
        .asg	    R10,    channel7_pulse_len
        .asg        R11,    initial_state_mask
        .asg        R12,    final_state_mask
        .asg	    R13,    current_state_timestamp
        .asg	    R14,	TEMP_REG1
        .asg        R15,    channel0_previous_falledge
        .asg        R16,    channel1_previous_falledge
        .asg        R17,    channel2_previous_falledge
        .asg        R18,    channel3_previous_falledge
        .asg        R19,    channel4_previous_falledge
        .asg        R20,    channel5_previous_falledge
        .asg        R21,    channel6_previous_falledge
        .asg        R22,    channel7_previous_falledge

IEP_COUNT_REG0_OFFSET      .set    0x10
IEP_COUNT_REG1_OFFSET      .set    0x14

    .if $defined("SOC_AM261X")
channel0_state .set    0
channel1_state .set    1
channel2_state .set    3
channel3_state .set    6
channel4_state .set    7
channel5_state .set    8
channel6_state .set    11
channel7_state .set    13
    .else
channel0_state .set    0
channel1_state .set    1
channel2_state .set    2
channel3_state .set    3
channel4_state .set    4
channel5_state .set    5
channel6_state .set    6
channel7_state .set    8
    .endif

;********
;* MAIN *
;********

main:

init:
    zero	&r0, 120            ; Clear the register space
iep:
	; disable iep timer
    m_set_iep_global_cfg_reg   	0, TEMP_REG, 0x20

    ; set starting count value of IEP counter
    m_set_iep_count_reg0       	0, TEMP_REG, 0xffffffff
    m_set_iep_count_reg1       	0, TEMP_REG, 0xffffffff
    m_set_iep_global_cfg_reg   0, TEMP_REG, 0x51          ; Start IEP timer
enable_PRU_cycle_counter:
	lbco	&R0, C11, 0, 4
	set 	R0, R0, 3
	sbco	&R0, C11, 0, 4
    mov initial_state_mask, R31
loop_sampling:
clear_ch_ack_reg:
;If ack reg is set then clear status bit as ch is serviced
    ldi     R0, 11
    xin     PRU_SPAD_B0_XID, &TEMP_REG, 4
    qbeq    do_not_clr_ch0, TEMP_REG, 0
    ldi     R1, 0
    xout    PRU_SPAD_B0_XID, &R1, 4
do_not_clr_ch0:
    ldi     R0, 12
    xin     PRU_SPAD_B0_XID, &TEMP_REG, 4
    qbeq    do_not_clr_ch1, TEMP_REG, 0
    ldi     R1, 0
    xout    PRU_SPAD_B0_XID, &R1, 4
do_not_clr_ch1:
    ldi     R0, 13
    xin     PRU_SPAD_B0_XID, &TEMP_REG, 4
    qbeq    do_not_clr_ch2, TEMP_REG, 0
    ldi     R1, 0
    xout    PRU_SPAD_B0_XID, &R1, 4
do_not_clr_ch2:
    ldi     R0, 14
    xin     PRU_SPAD_B0_XID, &TEMP_REG, 4
    qbeq    do_not_clr_ch3, TEMP_REG, 0
    ldi     R1, 0
    xout    PRU_SPAD_B0_XID, &R1, 4
do_not_clr_ch3:
; store IEP timestamp for this cycle in current_state_timestamp
    lbco    &current_state_timestamp, ICSS_IEP_CONST, IEP_COUNT_REG0_OFFSET, 4
; XOR final_state_mask with initial_state_mask to identify edge transistion
	mov		final_state_mask, R31
    xor 	state_change_mask, initial_state_mask, final_state_mask
; AND initial state with statemask to find which pins went low
    and     state_change_mask, state_change_mask, initial_state_mask
    mov		initial_state_mask, final_state_mask
; If channel0 have falling edge registered, go to next instruction else skip_sample_ch0
    qbbc	skip_sample_ch0, state_change_mask, channel0_state
; Update channel0_pulse_len with the pulse length = current_state_timestamp - initial_value
    sub     channel0_pulse_len, current_state_timestamp, channel0_previous_falledge
; Store this falling edge on channel0 as previous falling edge for comparision reference
    mov     channel0_previous_falledge, current_state_timestamp
skip_sample_ch0:
; ; If channel1 have falling edge registered, go to next instruction else skip_sample_ch1
    qbbc	skip_sample_ch1, state_change_mask, channel1_state
; Update channel1_pulse_len with the pulse length = current_state_timestamp - initial_value
	sub     channel1_pulse_len, current_state_timestamp, channel1_previous_falledge
; Store this falling edge on channel1 as previous falling edge for comparision reference
    mov     channel1_previous_falledge, current_state_timestamp
skip_sample_ch1:
; If channel2 have falling edge registered, go to next instruction else skip_sample_ch2
    qbbc	skip_sample_ch2, state_change_mask, channel2_state

; Update channel2_pulse_len with the pulse length = current_state_timestamp - initial_value
    sub     channel2_pulse_len, current_state_timestamp, channel2_previous_falledge
; Store this falling edge on channel2 as previous falling edge for comparision reference
    mov     channel2_previous_falledge, current_state_timestamp
skip_sample_ch2:
; If channel3 have falling edge registered, go to next instruction else skip_sample_ch3
    qbbc	skip_sample_ch3, state_change_mask, channel3_state
; Update channel3_pulse_len with the pulse length = current_state_timestamp - initial_value
    sub     channel3_pulse_len, current_state_timestamp, channel3_previous_falledge
; Store this falling edge on channel3 as previous falling edge for comparision reference
    mov     channel3_previous_falledge, current_state_timestamp
skip_sample_ch3:
; If channel4 have falling edge registered, go to next instruction else skip_sample_ch4
    qbbc	skip_sample_ch4, state_change_mask, channel4_state
; Update channel4_pulse_len with the pulse length = current_state_timestamp - initial_value
    sub     channel4_pulse_len, current_state_timestamp, channel4_previous_falledge
; Store this falling edge on channel4 as previous falling edge for comparision reference
    mov     channel4_previous_falledge, current_state_timestamp
skip_sample_ch4:
; If channel5 have falling edge registered, go to next instruction else skip_sample_ch5
    qbbc	skip_sample_ch5, state_change_mask, channel5_state
; Update channel5_pulse_len with the pulse length = current_state_timestamp - initial_value
    sub     channel5_pulse_len, current_state_timestamp, channel5_previous_falledge
; Store this falling edge on channel5 as previous falling edge for comparision reference
    mov     channel5_previous_falledge, current_state_timestamp
skip_sample_ch5:
; If channel6 have falling edge registered, go to next instruction else skip_sample_ch6
    qbbc	skip_sample_ch6, state_change_mask, channel6_state
; Update channel6_pulse_len with the pulse length = current_state_timestamp - initial_value
    sub     channel6_pulse_len, current_state_timestamp, channel6_previous_falledge
; Store this falling edge on channel6 as previous falling edge for comparision reference
    mov     channel6_previous_falledge, current_state_timestamp
skip_sample_ch6:
; If channel7 have falling edge registered, go to next instruction else skip_sample_ch7. Here channel 7 is mapped to GPIO 8 so we are using offset 8
    qbbc	skip_sample_ch7, state_change_mask, channel7_state
; Update channel7_pulse_len with the pulse length = current_state_timestamp - initial_value
    sub     channel7_pulse_len, current_state_timestamp, channel7_previous_falledge
; Store this falling edge on channel7 as previous falling edge for comparision reference
    mov     channel7_previous_falledge, current_state_timestamp
skip_sample_ch7:
send_data:
;Send data to PRU1 for data processing
    ldi     R0, 0
	xout 	PRU_SPAD_B0_XID, &R2, 36
skip_send_data:
    qba 	loop_sampling


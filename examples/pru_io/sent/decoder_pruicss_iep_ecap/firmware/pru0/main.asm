; Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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
    .include "icss_iep_regs.inc"
    .include "header.inc"

;************************************************************************************
;   /*Tick period of channel(for reference only)*/
;   chx_ticktime                .set   500
;
;   Below two parameters can be modified to support different tick period(x(0,7))
;   /*Minimum duration(taking 20% minimun deviation) of sync pulse for a given tick period sensor[(tp*0.8)*56]*/
;   chx_syncpulse_min_dur       .set   400*56
;   /*Maximum duration(taking 20% maximum deviation)  of sync pulse for a given tick period sensor[(tp*1.2)*56]*/
;   chx_syncpulse_max_dur       .set   600*56
;
;************************************************************************************

ch0_ticktime                .set   500
ch0_syncpulse_min_dur       .set   400*56
ch0_syncpulse_max_dur       .set   600*56

ch1_ticktime                .set   500
ch1_syncpulse_min_dur       .set   400*56
ch1_syncpulse_max_dur       .set   600*56

ch2_ticktime                .set   500
ch2_syncpulse_min_dur       .set   400*56
ch2_syncpulse_max_dur       .set   600*56

ch3_ticktime                .set   500
ch3_syncpulse_min_dur       .set   400*56
ch3_syncpulse_max_dur       .set   600*56

ch4_ticktime                .set   500
ch4_syncpulse_min_dur       .set   400*56
ch4_syncpulse_max_dur       .set   600*56

ch5_ticktime                .set   500
ch5_syncpulse_min_dur       .set   400*56
ch5_syncpulse_max_dur       .set   600*56

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

;************************************************************************************
;
;   Macro: m_process_ch
;
;  Initialises the channel setup parameter and updates channel state ro start decoding pulse
;
;   PEAK cycles:
;       63 cycles
;
;   Invokes:
;       None
;
;   Pseudo code:
;       if(ch_current_fall_edge = ch_previous_fall_edge)
;       {
;           jump to skip_sample_ch;
;       }
;       else
;		{
;           ch_previous_fall_edge = ch_current_fall_edge
;       	BNS_ARG_LUTBASE = ch_lut_base;
;       	load CHx_PULSE_LEN with channel pulse length
;       	BNS_CH_DATA_ERROR = ch_data_error;
;       	jump to ch_state and retun back to here
;       }
;
;   Parameters:
;      skip_sample_ch: skip sample handler for a given channel
;      ch_current_fall_edge: current time stamp of capture register for a given channel
;      ch_previous_fall_edge: previous time stamp of capture register for a given channel
;      ch_lut_base: Lookup Table base address
;      ch_data_error: data error handler for a given channel
;      ch_state: State variable of channel
;
;   Returns:
;      None
;
;   See Also:
;
;************************************************************************************
m_process_ch .macro  skip_sample_ch ,ch_current_fall_edge, ch_previous_fall_edge, ch_lut_base, ch_data_error, ch_state
;check if capture event occured on channel 0
	qbeq	skip_sample_ch, ch_previous_fall_edge,ch_current_fall_edge ;1
    sub     CHx_PULSE_LEN, ch_current_fall_edge, ch_previous_fall_edge ;1
    qbge	skip_sample_ch, CHx_PULSE_LEN,25
; store this falling edge on channel0 as previous falling edge for comparision reference
    mov     ch_previous_fall_edge, ch_current_fall_edge ;

    ldi     BNS_ARG_LUTBASE, ch_lut_base
    mov     BNS_ARG_VAL, CHx_PULSE_LEN
    ldi     BNS_CH_DATA_ERROR, $CODE(ch_data_error)
    jal     return_addr1, ch_state
    .endm
;************************************************************************************
;
;   Macro: m_ch_sync
;
;  Initialises the channel setup parameter and updates channel state ro start decoding pulse
;
;   PEAK cycles:
;       55 cycles
;
;   Invokes:
;       None
;
;   Pseudo code:
;       if(CHx_PULSE_LEN<ch0_syncpulse_min_dur || CHx_PULSE_LEN>ch0_syncpulse_max_dur)
;       {
;           jump to error_ch;
;       }
;       else
;       { //Divide CHx_PULSE_LEN by 56
;           CHx_PULSE_LEN = CHx_PULSE_LEN<< 3 ;//Divide by 8
;           CHx_PULSE_LEN = CHx_PULSE_LEN*(1/7) ;//Divide by 7
;           m_calculate_tick(TEMP_REG1, CHx_PULSE_LEN);
;           CH0_BUF.w0 = ch_calculated_tick_period// Stored in R27.w0
;           populate the hashtable
;        }
;
;   Parameters:
;      error_ch: Error handler for channel
;      ch_syncpulse_max_dur: Max pulse length for a given tick period (tp+20%)
;      ch_syncpulse_min_dur: Min pulse length for  a given tick period (tp-20%)
;      ch_buf: Buffer for channel to store calculated value
;      ch_lut_base: Lookup Table base address
;      ch_state: State variable of channel
;      ch_status_comm: Next channel state to be loaded into ch_state
;   Returns:
;      None
;
;   See Also:
;
;************************************************************************************
m_ch_sync .macro error_ch, ch_syncpulse_max_dur, ch_syncpulse_min_dur, ch_buf, ch_lut_base, ch_state, ch_status_comm
;check if it is sync pulse
    ldi32 	TEMP_REG1, ch_syncpulse_max_dur ;2
    qblt    error_ch, CHx_PULSE_LEN, TEMP_REG1 ;1
    ldi32 	TEMP_REG1, ch_syncpulse_min_dur ;2
    qbgt    error_ch,  CHx_PULSE_LEN, TEMP_REG1 ;1
;Calculate tick period by dividing pulse length with 56, Divide by 8, then by 7
    lsr     CHx_PULSE_LEN, CHx_PULSE_LEN, 3 ;1
    ldi32 	TEMP_REG1, TICK_7_CONST ;2
    m_calculate_tick TEMP_REG1, CHx_PULSE_LEN ;4
    mov     channelx_tick_period, R27.w0 ; load channelx_tick_period with update tick period ;1
    mov     ch_buf.w0, channelx_tick_period.w0 ;1
;find duration for 12Tp with Tickperiod(Tp) in channelx_tick_period
	ldi 	TEMP_REG1, 12 ;1
    m_multiply TEMP_REG1, channelx_tick_period ;4
;update the lut using R4 to R19 registers(16 reg)
    xout	PRU_SPAD_B1_XID, &R4, 64 ;1
;store lower product in R4 Register
    mov     R4, R26 ;1
;update R4 to R19
    m_update_lut channelx_tick_period ;15
;Store lookup table in dmem at CH0_LUT_BASE
    ldi    TEMP_REG1, ch_lut_base ;1
    sbbo    &R4, TEMP_REG1, 0, 64 ;7
    xin 	PRU_SPAD_B1_XID, &R4, 64 ;1
;Update Ch0 state register for status & Comm data calc
  	ldi    ch_state, $CODE(ch_status_comm) ;1
  	jmp     return_addr1 ;1
    .endm

;************************************************************************************
;
;   Macro: m_ch_status_comm
;
;  Used for status and comms nibble decoding
;
;   PEAK cycles:
;       4 cycles
;
;   Invokes:
;       None
;
;   Pseudo code:
;       None
;
;   Parameters:
;      ch_buf: Buffer for channel to store calculated value
;      ch_state: State variable of channel
;      ch_data0: Data0 decoding function
;
;   Returns:
;      None
;
;   See Also:
;
;************************************************************************************
m_ch_status_comm .macro ch_buf, ch_state, ch_data0
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  ch_buf.b2, *BNS_ARG_RETVAL_ADDR
    ldi   ch_state, $CODE(ch_data0)
    jmp   return_addr1
    .endm

;************************************************************************************
;
;   Macro: m_ch_data0
;   Used for status and comms nibble decoding
;
;   PEAK cycles:
;       15 cycles
;
;   Invokes:
;       FN_BINARY_SEARCH
;
;   Pseudo code:
;       None
;
;   Parameters:
;      ch_buf: Buffer for channel to store calculated value
;      ch_state: State variable of channel
;      ch_data0_spad_base: Channel Scratch pad base address
;      ch_data1: Data1 decoding function
;      ch_crc4_res: CRC variable for channel
;
;************************************************************************************
m_ch_data0 .macro ch_buf, ch_state, ch_data0_spad_base, ch_data1, ch_crc4_res
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  ch_buf.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, ch_data0_spad_base
    xout  PRU_SPAD_B2_XID, &ch_buf, 4
  	ldi   ch_state, $CODE(ch_data1)
    ldi   ch_crc4_res, CRC4_SEED
    lsr   ch_crc4_res, ch_crc4_res, 4
    add   ch_crc4_res, ch_crc4_res, ch_buf.b3
    lbbo  &ch_crc4_res, TEMP_REG2 , ch_crc4_res, 1
    jmp   return_addr1
    .endm

;************************************************************************************
;
;   Macro: m_ch_data1
;   Used for status and comms nibble decoding
;
;   PEAK cycles:
;       10 cycles
;
;   Invokes:
;       FN_BINARY_SEARCH
;
;   Pseudo code:
;       None
;
;   Parameters:
;      ch_buf: Buffer for channel to store calculated value
;      ch_state: State variable of channel
;      ch_data2: Data2 decoding function
;      ch_crc4_res: CRC variable for channel
;
;************************************************************************************
m_ch_data1 .macro ch_buf, ch_state, ch_data2, ch_crc4_res
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  ch_buf.b0, *BNS_ARG_RETVAL_ADDR
    ldi   ch_state, $CODE(ch_data2)
    lsr   ch_crc4_res, ch_crc4_res, 4
    add   ch_crc4_res, ch_crc4_res, ch_buf.b0
    lbbo  &ch_crc4_res, TEMP_REG2 , ch_crc4_res, 1
    jmp   return_addr1
    .endm

;************************************************************************************
;
;   Macro: m_ch_data2
;   Used for status and comms nibble decoding
;
;   PEAK cycles:
;       15 cycles
;
;   Invokes:
;       FN_BINARY_SEARCH
;
;   Pseudo code:
;       None
;
;   Parameters:
;      ch_buf: Buffer for channel to store calculated value
;      ch_state: State variable of channel
;      ch_data3: Data3 decoding function
;      ch_crc4_res: CRC variable for channel
;
;************************************************************************************
m_ch_data2 .macro ch_buf, ch_state, ch_data3, ch_crc4_res
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  ch_buf.b1, *BNS_ARG_RETVAL_ADDR
  	ldi   ch_state, $CODE(ch_data3)
    lsr   ch_crc4_res, ch_crc4_res, 4
    add   ch_crc4_res, ch_crc4_res, ch_buf.b1
    lbbo  &ch_crc4_res, TEMP_REG2 , ch_crc4_res, 1
    jmp   return_addr1
    .endm

;************************************************************************************
;
;   Macro: m_ch_data3
;   Used for status and comms nibble decoding
;
;   PEAK cycles:
;       15 cycles
;
;   Invokes:
;       FN_BINARY_SEARCH
;
;   Pseudo code:
;       None
;
;   Parameters:
;      ch_buf: Buffer for channel to store calculated value
;      ch_state: State variable of channel
;      ch_data4: Data4 decoding function
;      ch_crc4_res: CRC variable for channel
;
;************************************************************************************
m_ch_data3 .macro ch_buf, ch_state, ch_data4, ch_crc4_res
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  ch_buf.b2, *BNS_ARG_RETVAL_ADDR
    ldi   ch_state, $CODE(ch_data4)
    lsr   ch_crc4_res, ch_crc4_res, 4
    add   ch_crc4_res, ch_crc4_res, ch_buf.b2
    lbbo  &ch_crc4_res, TEMP_REG2 , ch_crc4_res, 1
    jmp   return_addr1
    .endm

;************************************************************************************
;
;   Macro: m_ch_data4
;   Used for status and comms nibble decoding
;
;   PEAK cycles:
;       15 cycles
;
;   Invokes:
;       FN_BINARY_SEARCH
;
;   Pseudo code:
;       None
;
;   Parameters:
;      ch_buf: Buffer for channel to store calculated value
;      ch_state: State variable of channel
;      ch_data4_spad_base: Channel Scratch pad base address for storing till data4
;      ch_data5: Data5 decoding function
;      ch_crc4_res: CRC variable for channel
;
;************************************************************************************
m_ch_data4 .macro ch_buf, ch_state, ch_data4_spad_base, ch_data5, ch_crc4_res
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  ch_buf.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, ch_data4_spad_base
    xout  PRU_SPAD_B2_XID, &ch_buf, 4
  	ldi   ch_state, $CODE(ch_data5)
    lsr   ch_crc4_res, ch_crc4_res, 4
    add   ch_crc4_res, ch_crc4_res, ch_buf.b3
    lbbo  &ch_crc4_res, TEMP_REG2 , ch_crc4_res, 1
    jmp   return_addr1
    .endm

;************************************************************************************
;
;   Macro: m_ch_data5
;   Used for 5th data nibble decoding
;
;   PEAK cycles:
;       15 cycles
;
;   Invokes:
;       FN_BINARY_SEARCH
;
;   Pseudo code:
;       None
;
;   Parameters:
;      ch_buf: Buffer for channel to store calculated value
;      ch_state: State variable of channel
;      ch_crc: CRC decoding function
;      ch_crc4_res: CRC variable for channel
;
;************************************************************************************
m_ch_data5 .macro ch_buf, ch_state, ch_crc, ch_crc4_res
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  ch_buf.b0, *BNS_ARG_RETVAL_ADDR
    ldi   ch_state, $CODE(ch_crc)
    lsr   ch_crc4_res, ch_crc4_res, 4
    add   ch_crc4_res, ch_crc4_res, ch_buf.b0
    lbbo  &ch_crc4_res, TEMP_REG2 , ch_crc4_res, 1
    jmp   return_addr1
    .endm

;************************************************************************************
;
;   Macro: m_ch_crc
;   Used for crc nibble decoding
;
;   PEAK cycles:
;       15 cycles
;
;   Invokes:
;       FN_BINARY_SEARCH
;
;   Pseudo code:
;       None
;
;   Parameters:
;      ch_buf: Buffer for channel to store calculated value
;      ch_state: State variable of channel
;      ch_crcdata_spad_base: Channel Scratch pad base address for crc data
;      ch_crc4_res: CRC variable for channel
;      ch_crc_error: CRC error handler function
;
;************************************************************************************
m_ch_crc .macro ch_buf, ch_state, ch_crcdata_spad_base, ch_crc4_res, ch_crc_error
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  ch_buf.b1, *BNS_ARG_RETVAL_ADDR
    lsr   ch_crc4_res, ch_crc4_res, 4
    lbbo  &ch_crc4_res, TEMP_REG2 , ch_crc4_res, 1
    mov   ch_buf.b2, ch_crc4_res
    mov   ch_buf.b3, BNS_ARG_STATUS
    ldi   R0.b0, ch_crcdata_spad_base
    xout  PRU_SPAD_B2_XID, &ch_buf, 4
    qbne  ch_crc_error, ch_crc4_res, 0
    .endm

;************************************************************************************
;
;   Macro: m_ch_data_out
;   Used for 5th data nibble decoding
;
;   PEAK cycles:
;       15 cycles
;
;   Invokes:
;       FN_BINARY_SEARCH
;
;   Pseudo code:
;       None
;
;   Parameters:
;      ch_reg: Buffer for channel to store calculated value
;      ch_data_base: IPC base address for sending data to R5F
;      ch_intr_byte: Interrupt byte for channel
;
;************************************************************************************
m_ch_data_out .macro ch_reg, ch_data_base, ch_intr_byte
;Put data in DMEM or Collect data for different mode
    ldi   R0, 0
;Store existing data first
    xout  PRU_SPAD_B1_XID, &ch_reg, 12
    xin   PRU_SPAD_B2_XID, &ch_reg, 12
;Write 10 byte of data in SMEM(Sync, Data and CRC)
    sbco  &ch_reg, C28, ch_data_base, 12
;Bring back stored data
    zero	&ch_reg, 12
    xout  PRU_SPAD_B2_XID, &ch_reg, 12
    xin   PRU_SPAD_B1_XID, &ch_reg, 12
;Trigger Interrupt to R5F/*TODO*/
    ldi   R0, 1
    sbco  &R0, C28, ch_intr_byte, 1
    .endm

;************************************************************************************
;
;   Macro: m_update_lut
;
;   Used for updating lut for decoding nibbles used by FN_BINARY_SEARCH
;
;   PEAK cycles:
;       15 cycles
;
;   Invokes:
;       None
;
;   Pseudo code:
;       None
;
;   Parameters:
;      increment
;
;   Returns:
;      None
;
;   See Also:
;
;************************************************************************************
m_update_lut .macro increment
    add     R5, R4, increment
    add     R6, R5, increment
    add     R7, R6, increment
    add     R8, R7, increment
    add     R9, R8, increment
    add     R10, R9, increment
    add     R11, R10, increment
    add     R12, R11, increment
    add     R13, R12, increment
    add     R14, R13, increment
    add     R15, R14, increment
    add     R16, R15, increment
    add     R17, R16, increment
    add     R18, R17, increment
    add     R19, R18, increment
    .endm

;************************************************************************************
;
;   Macro: m_multiply
;
;  Used for multiplication
;
;   PEAK cycles:
;       4 cycles
;
;   Invokes:
;       None
;
;   Pseudo code:
;      op1*op2
;
;   Parameters:
;      op1
;      op2
;
;   Returns:
;      Product
;
;   See Also:
;
;************************************************************************************

m_multiply .macro op1, op2
;Load R28 and R29 with multiplication factors
    mov     R29, op1
    mov     R28, op2
    NOP
;Fetch quotient from R26 and R27 register
    xin     MAC_XID, &R26, 8
    .endm

;************************************************************************************
;
;   Macro: m_calculate_tick
;
;  Calculates tick period for given sync pulse length
;
;   PEAK cycles:
;       4 cycles
;
;   Invokes:
;       None
;
;   Pseudo code:
;      sync pulse length * 1/56
;
;   Parameters:
;      divisor_inverse
;      dividend
;
;   Returns:
;      Tick period
;
;   See Also:
;
;************************************************************************************

m_calculate_tick .macro divisor_inverse, dividend

;Load R28 and R29 with multiplication factors
    mov     R29, dividend
    mov     R28, divisor_inverse
    NOP
;Fetch quotient from R26 and R27 register
    xin     MAC_XID, &R26, 8
    .endm


;********
;* MAIN *
;********

main:

init:
    zero	&r0, 120            ; Clear the register space
iep:
	; disable iep timer
    m_set_iep_global_cfg_reg   	0, TEMP_REG1, 0x20

    ; set starting count value of IEP counter
    m_set_iep_count_reg0       	0, TEMP_REG1, 0xffffffff
    m_set_iep_count_reg1       	0, TEMP_REG1, 0xffffffff

	;Enable the reset of iep counter with compare 0
	ldi     TEMP_REG1,  0x03
	sbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 4
	fill	&TEMP_REG1, 0x4
	sbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CMP0_REG, 4
	;clear the compare status
	ldi     TEMP_REG1, 0x01
	sbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 4

	;set cap 0 to 5 in continouous mode with External capture enable
	ldi32   TEMP_REG1,  0x00FC0000
	sbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAP_CFG_REG, 4
	lbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAPR0_REG, 4
    lbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAPR1_REG, 4
    lbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAPR2_REG, 4
    lbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAPR3_REG, 4
    lbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAPR4_REG, 4
    lbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAPR5_REG, 4

	; Start IEP timer with increment of 5 (increment of 1->1ns)
    m_set_iep_global_cfg_reg   0, TEMP_REG1, 0x51

enable_PRU_cycle_counter:
	; enable PRU0 cycle counter
	lbco	&R0, C11, 0, 4
	set 	R0, R0, 3
	sbco	&R0, C11, 0, 4
spad_shift_enable:
	ldi     TEMP_REG1, 2
    sbco    &TEMP_REG1, ICSS_CFG_CONST, 0x34, 4
; Configure the Constant Table entry C28 to point to start of shared memory
; PRU_ICSSG Shared RAM (local-C28) : 00nn_nn00h, nnnn = c28_pointer[15:0]
; By default it is set to 0000_0000h so it will point to DMEM1 address
    ldi     TEMP_REG1, 0x0100
    sbco    &TEMP_REG1, ICSS_PRU_CTRL_CONST, 0x28, 2
;For using mvib instruction load BNS_ARG_RETVAL_ADDR with BNS_ARG_RETVAL(R1.b1 pointer address)
    ldi     BNS_ARG_RETVAL_ADDR, 5
;Intialize  CRC_LUT_ADDR
    ldi    CRC_LUT_ADDR, (CRC4_LUT_OFFSET+PDMEM00)

	ldi32   APPROX_VAL_MAX, 0xFFFFFF00
	ldi   APPROX_VAL_MIN, 256
    ldi    CH0_STATE, $CODE(CH0_SYNC)
    ldi    CH1_STATE, $CODE(CH1_SYNC)
    ldi    CH2_STATE, $CODE(CH2_SYNC)
    ldi    CH3_STATE, $CODE(CH3_SYNC)
    ldi    CH4_STATE, $CODE(CH4_SYNC)
    ldi    CH5_STATE, $CODE(CH5_SYNC)

loop_sampling:
; Update channel0_pulse_len with the pulse length = current_timestamp - previous_timestamp
	lbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAPR0_REG, 4 ;2
	m_process_ch	skip_sample_ch0, TEMP_REG1, CH0_PREVIOUS_FALLEDGE, CH0_LUT_BASE, ch0_data_error, CH0_STATE
skip_sample_ch0:
; Update channel1_pulse_len with the pulse length = current_timestamp - previous_timestamp
	lbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAPR1_REG, 4
	m_process_ch	skip_sample_ch1, TEMP_REG1, CH1_PREVIOUS_FALLEDGE, CH1_LUT_BASE, ch1_data_error, CH1_STATE
skip_sample_ch1:
; Update channel0_pulse_len with the pulse length = current_timestamp - previous_timestamp
	lbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAPR2_REG, 4
	m_process_ch	skip_sample_ch2, TEMP_REG1, CH2_PREVIOUS_FALLEDGE, CH2_LUT_BASE, ch2_data_error, CH2_STATE
skip_sample_ch2:
; Update channel0_pulse_len with the pulse length = current_timestamp - previous_timestamp
	lbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAPR3_REG, 4
	m_process_ch	skip_sample_ch3, TEMP_REG1, CH3_PREVIOUS_FALLEDGE, CH3_LUT_BASE, ch3_data_error, CH3_STATE
skip_sample_ch3:
; Update channel0_pulse_len with the pulse length = current_timestamp - previous_timestamp
	lbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAPR4_REG, 4
	m_process_ch	skip_sample_ch4, TEMP_REG1, CH4_PREVIOUS_FALLEDGE, CH4_LUT_BASE, ch4_data_error, CH4_STATE
skip_sample_ch4:
; Update channel0_pulse_len with the pulse length = current_timestamp - previous_timestamp
	lbco    &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_CAPR5_REG, 4
	m_process_ch	skip_sample_ch5, TEMP_REG1, CH5_PREVIOUS_FALLEDGE, CH5_LUT_BASE, ch5_data_error, CH5_STATE
skip_sample_ch5:
    qba 	loop_sampling


CH0_SYNC:
    m_ch_sync ERROR_CH0, ch0_syncpulse_max_dur, ch0_syncpulse_min_dur, CH0_BUF, CH0_LUT_BASE, CH0_STATE, CH0_STATUS_COMM
CH0_DATA0:
    m_ch_data0 CH0_BUF, CH0_STATE, CH0_DATA0_SPAD_BASE, CH0_DATA1, CH0_CRC4_RES
CH0_DATA2:
    m_ch_data2 CH0_BUF, CH0_STATE, CH0_DATA3, CH0_CRC4_RES
CH0_DATA4:
    m_ch_data4 CH0_BUF, CH0_STATE, CH0_DATA4_SPAD_BASE, CH0_DATA5, CH0_CRC4_RES
CH0_CRC:
    m_ch_crc CH0_BUF, CH0_STATE, CH0_CRCDATA_SPAD_BASE, CH0_CRC4_RES, CH0_CRC_ERROR
CH0_DATA_OUT:
    m_ch_data_out  CH0_REG, CH0_DATA_BASE, CH0_INTR_BYTE
;/*TODO: Add error handling mechnaism for each label below*/
CH0_CRC_ERROR:
ch0_data_error:
ERROR_CH0:
    ldi   CH0_STATE, $CODE(CH0_SYNC)
    jmp   return_addr1
CH0_STATUS_COMM:
    m_ch_status_comm  CH0_BUF, CH0_STATE, CH0_DATA0
CH0_DATA1:
    m_ch_data1 CH0_BUF, CH0_STATE, CH0_DATA2, CH0_CRC4_RES
CH0_DATA3:
    m_ch_data3 CH0_BUF, CH0_STATE, CH0_DATA4, CH0_CRC4_RES
CH0_DATA5:
    m_ch_data5 CH0_BUF, CH0_STATE, CH0_CRC, CH0_CRC4_RES


CH1_SYNC:
    m_ch_sync ERROR_CH1, ch1_syncpulse_max_dur, ch1_syncpulse_min_dur, CH1_BUF, CH1_LUT_BASE, CH1_STATE, CH1_STATUS_COMM

CH1_DATA0:
    m_ch_data0 CH1_BUF, CH1_STATE, CH1_DATA0_SPAD_BASE, CH1_DATA1, CH1_CRC4_RES
CH1_DATA2:
    m_ch_data2 CH1_BUF, CH1_STATE, CH1_DATA3, CH1_CRC4_RES
CH1_DATA4:
    m_ch_data4 CH1_BUF, CH1_STATE, CH1_DATA4_SPAD_BASE, CH1_DATA5, CH1_CRC4_RES
CH1_CRC:
    m_ch_crc CH1_BUF, CH1_STATE, CH1_CRCDATA_SPAD_BASE, CH1_CRC4_RES, CH1_CRC_ERROR
CH1_DATA_OUT:
    m_ch_data_out  CH1_REG, CH1_DATA_BASE, CH1_INTR_BYTE
;/*TODO: Add error handling mechnaism for each label below*/
CH1_CRC_ERROR:
ch1_data_error:
ERROR_CH1:
    ldi   CH1_STATE, $CODE(CH1_SYNC)
    jmp   return_addr1
CH1_STATUS_COMM:
    m_ch_status_comm  CH1_BUF, CH1_STATE, CH1_DATA0
CH1_DATA1:
    m_ch_data1 CH1_BUF, CH1_STATE, CH1_DATA2, CH1_CRC4_RES
CH1_DATA3:
    m_ch_data3 CH1_BUF, CH1_STATE, CH1_DATA4, CH1_CRC4_RES
CH1_DATA5:
    m_ch_data5 CH1_BUF, CH1_STATE, CH1_CRC, CH1_CRC4_RES


CH2_SYNC:
    m_ch_sync ERROR_CH2, ch2_syncpulse_max_dur, ch2_syncpulse_min_dur, CH2_BUF, CH2_LUT_BASE, CH2_STATE, CH2_STATUS_COMM
CH2_DATA0:
    m_ch_data0 CH2_BUF, CH2_STATE, CH2_DATA0_SPAD_BASE, CH2_DATA1, CH2_CRC4_RES
CH2_DATA2:
    m_ch_data2 CH2_BUF, CH2_STATE, CH2_DATA3, CH2_CRC4_RES
CH2_DATA4:
    m_ch_data4 CH2_BUF, CH2_STATE, CH2_DATA4_SPAD_BASE, CH2_DATA5, CH2_CRC4_RES
CH2_CRC:
    m_ch_crc CH2_BUF, CH2_STATE, CH2_CRCDATA_SPAD_BASE, CH2_CRC4_RES, CH2_CRC_ERROR
CH2_DATA_OUT:
    m_ch_data_out  CH2_REG, CH2_DATA_BASE, CH2_INTR_BYTE
;/*TODO: Add error handling mechnaism for each label below*/
CH2_CRC_ERROR:
ch2_data_error:
ERROR_CH2:
    ldi   CH2_STATE, $CODE(CH2_SYNC)
    jmp   return_addr1
CH2_STATUS_COMM:
    m_ch_status_comm  CH2_BUF, CH2_STATE, CH2_DATA0
CH2_DATA1:
    m_ch_data1 CH2_BUF, CH2_STATE, CH2_DATA2, CH2_CRC4_RES
CH2_DATA3:
    m_ch_data3 CH2_BUF, CH2_STATE, CH2_DATA4, CH2_CRC4_RES
CH2_DATA5:
    m_ch_data5 CH2_BUF, CH2_STATE, CH2_CRC, CH2_CRC4_RES


CH3_SYNC:
    m_ch_sync ERROR_CH3, ch3_syncpulse_max_dur, ch3_syncpulse_min_dur, CH3_BUF, CH3_LUT_BASE, CH3_STATE, CH3_STATUS_COMM

CH3_DATA0:
    m_ch_data0 CH3_BUF, CH3_STATE, CH3_DATA0_SPAD_BASE, CH3_DATA1, CH3_CRC4_RES
CH3_DATA2:
    m_ch_data2 CH3_BUF, CH3_STATE, CH3_DATA3, CH3_CRC4_RES
CH3_DATA4:
    m_ch_data4 CH3_BUF, CH3_STATE, CH3_DATA4_SPAD_BASE, CH3_DATA5, CH3_CRC4_RES
CH3_CRC:
    m_ch_crc CH3_BUF, CH3_STATE, CH3_CRCDATA_SPAD_BASE, CH3_CRC4_RES, CH3_CRC_ERROR
CH3_DATA_OUT:
    m_ch_data_out  CH3_REG, CH3_DATA_BASE, CH3_INTR_BYTE
;/*TODO: Add error handling mechnaism for each label below*/
CH3_CRC_ERROR:
ch3_data_error:
ERROR_CH3:
    ldi   CH3_STATE, $CODE(CH3_SYNC)
    jmp   return_addr1
CH3_STATUS_COMM:
    m_ch_status_comm  CH3_BUF, CH3_STATE, CH3_DATA0
CH3_DATA1:
    m_ch_data1 CH3_BUF, CH3_STATE, CH3_DATA2, CH3_CRC4_RES
CH3_DATA3:
    m_ch_data3 CH3_BUF, CH3_STATE, CH3_DATA4, CH3_CRC4_RES
CH3_DATA5:
    m_ch_data5 CH3_BUF, CH3_STATE, CH3_CRC, CH3_CRC4_RES


CH4_SYNC:
    m_ch_sync ERROR_CH4, ch4_syncpulse_max_dur, ch4_syncpulse_min_dur, CH4_BUF, CH4_LUT_BASE, CH4_STATE, CH4_STATUS_COMM

CH4_DATA0:
    m_ch_data0 CH4_BUF, CH4_STATE, CH4_DATA0_SPAD_BASE, CH4_DATA1, CH4_CRC4_RES
CH4_DATA2:
    m_ch_data2 CH4_BUF, CH4_STATE, CH4_DATA3, CH4_CRC4_RES
CH4_DATA4:
    m_ch_data4 CH4_BUF, CH4_STATE, CH4_DATA4_SPAD_BASE, CH4_DATA5, CH4_CRC4_RES
CH4_CRC:
    m_ch_crc CH4_BUF, CH4_STATE, CH4_CRCDATA_SPAD_BASE, CH4_CRC4_RES, CH4_CRC_ERROR
CH4_DATA_OUT:
    m_ch_data_out  CH4_REG, CH4_DATA_BASE, CH4_INTR_BYTE
;/*TODO: Add error handling mechnaism for each label below*/
CH4_CRC_ERROR:
ch4_data_error:
ERROR_CH4:
    ldi   CH4_STATE, $CODE(CH4_SYNC)
    jmp   return_addr1

CH4_STATUS_COMM:
    m_ch_status_comm  CH4_BUF, CH4_STATE, CH4_DATA0
CH4_DATA1:
    m_ch_data1 CH4_BUF, CH4_STATE, CH4_DATA2, CH4_CRC4_RES
CH4_DATA3:
    m_ch_data3 CH4_BUF, CH4_STATE, CH4_DATA4, CH4_CRC4_RES
CH4_DATA5:
    m_ch_data5 CH4_BUF, CH4_STATE, CH4_CRC, CH4_CRC4_RES


CH5_SYNC:
    m_ch_sync ERROR_CH5, ch5_syncpulse_max_dur, ch5_syncpulse_min_dur, CH5_BUF, CH5_LUT_BASE, CH5_STATE, CH5_STATUS_COMM

CH5_DATA0:
    m_ch_data0 CH5_BUF, CH5_STATE, CH5_DATA0_SPAD_BASE, CH5_DATA1, CH5_CRC4_RES
CH5_DATA2:
    m_ch_data2 CH5_BUF, CH5_STATE, CH5_DATA3, CH5_CRC4_RES
CH5_DATA4:
    m_ch_data4 CH5_BUF, CH5_STATE, CH5_DATA4_SPAD_BASE, CH5_DATA5, CH5_CRC4_RES
CH5_CRC:
    m_ch_crc CH5_BUF, CH5_STATE, CH5_CRCDATA_SPAD_BASE, CH5_CRC4_RES, CH5_CRC_ERROR
CH5_DATA_OUT:
    m_ch_data_out  CH5_REG, CH5_DATA_BASE, CH5_INTR_BYTE
;/*TODO: Add error handling mechnaism for each label below*/
CH5_CRC_ERROR:
ch5_data_error:
ERROR_CH5:
    ldi   CH5_STATE, $CODE(CH5_SYNC)
    jmp   return_addr1

CH5_STATUS_COMM:
    m_ch_status_comm  CH5_BUF, CH5_STATE, CH5_DATA0
CH5_DATA1:
    m_ch_data1 CH5_BUF, CH5_STATE, CH5_DATA2, CH5_CRC4_RES
CH5_DATA3:
    m_ch_data3 CH5_BUF, CH5_STATE, CH5_DATA4, CH5_CRC4_RES
CH5_DATA5:
    m_ch_data5 CH5_BUF, CH5_STATE, CH5_CRC, CH5_CRC4_RES


;************************************************************************************
;
;   Function: FN_BINARY_SEARCH
;
;  Binary search on LUT with given pulse length as parameter and return nibble
;   value associated with it, if not found return error code
;
;   PEAK cycles:
;
;      48 cycles
;
;   Invokes:
;       None
;
;   Pseudo code:
;
;
;   Parameters:
;       None
;
;   Returns:
;       Nibble value associated with pulse length
;
;   See Also:
;
;************************************************************************************
FN_BINARY_SEARCH:
bn_search_mid7?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 28, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 7
check_sign7?:
    qbbs    check_lower_limit7?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit7?: ;if +ve
    qblt    bn_search_mid3?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit7?:
    qbgt    bn_search_mid11?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid5?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 20, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 5
check_sign5?:
    qbbs    check_lower_limit5?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit5?: ;if +ve
    qblt    bn_search_mid4?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit5?:
    qbgt    bn_search_mid6?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid4?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 16, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 4
check_sign4?:
    qbbs    check_lower_limit4?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit4?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit4?:
    qbgt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid6?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 24, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 6
check_sign6?:
    qbbs    check_lower_limit6?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit6?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit6?:
    qbgt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid3?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 12, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 3
check_sign3?:
    qbbs    check_lower_limit3?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit3?: ;if +ve
    qblt    bn_search_mid1?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit3?:
    qbgt    bn_search_mid5?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid1?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 4, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 1
check_sign1?:
    qbbs    check_lower_limit1?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit1?: ;if +ve
    qblt    bn_search_mid0?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit1?:
    qbgt    bn_search_mid2?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid0?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 0, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 0
check_sign0?:
    qbbs    check_lower_limit0?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit0?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit0?:
    qbgt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid2?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 8, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 2
check_sign2?:
    qbbs    check_lower_limit2?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit2?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit2?:
    qbgt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid11?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 44, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 11
check_sign11?:
    qbbs    check_lower_limit11?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit11?: ;if +ve
    qblt    bn_search_mid9?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit11?:
    qbgt    bn_search_mid13?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid9?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 36, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 9
check_sign9?:
    qbbs    check_lower_limit9?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit9?: ;if +ve
    qblt    bn_search_mid8?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit9?:
    qbgt    bn_search_mid10?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid8?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 32, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 8
check_sign8?:
    qbbs    check_lower_limit8?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit8?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit8?:
    qbgt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid10?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 40, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 10
check_sign10?:
    qbbs    check_lower_limit10?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit10?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit10?:
    qbgt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid13?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 52, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 13
check_sign13?:
    qbbs    check_lower_limit13?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit13?: ;if +ve
    qblt    bn_search_mid12?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit13?:
    qbgt    bn_search_mid14?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid12?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 48, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 12
check_sign12?:
    qbbs    check_lower_limit12?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit12?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit12?:
    qbgt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid14?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 56, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 14
check_sign14?:
    qbbs    check_lower_limit14?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit14?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit14?:
    qbgt    bn_search_mid15?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid15?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 60, 4
	sub     TEMP_REG1, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 15
check_sign15?:
    qbbs    check_lower_limit15?, TEMP_REG1, 31  ;check if diff  is -ve / +ve
check_upper_limit15?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit15?:
    qbgt    bn_search_notfound?, TEMP_REG1, APPROX_VAL_MAX    ; sign already checked so TEMP_REG1 is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
bn_search_notfound?:
    ldi     BNS_ARG_RETVAL, 0xFF
    ldi     BNS_ARG_STATUS, 0xFF
	jmp     BNS_CH_DATA_ERROR
bn_search_done?:
    ldi     BNS_ARG_STATUS, 0x0
    jmp     return_addr2

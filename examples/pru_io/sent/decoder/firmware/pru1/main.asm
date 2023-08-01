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

ch6_ticktime                .set   500
ch6_syncpulse_min_dur       .set   400*56
ch6_syncpulse_max_dur       .set   600*56

ch7_ticktime                .set   500
ch7_syncpulse_min_dur       .set   400*56
ch7_syncpulse_max_dur       .set   600*56

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
;   Macro: m_update_lut
;
;  Used for Updating LUT for decoding Nibbles used by FN_BINARY_SEARCH
;
;   PEAK cycles:
;       15 cycle
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
;       4 cycle
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
;       4 cycle
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

;************************************************************************************
;   MAIN
;************************************************************************************

main:

init:
    zero	&r0, 120            ; Clear the register space
    ; enable PRU0 cycle counter
	lbco	&R0, C11, 0, 4
	set 	R0, R0, 3
	sbco	&R0, C11, 0, 4
spad_shift_enable:
	ldi     TEMP_REG, 2
    sbco    &TEMP_REG, ICSS_CFG_CONST, 0x34, 4
; Configure the Constant Table entry C28 to point to start of shared memory
; PRU_ICSSG Shared RAM (local-C28) : 00nn_nn00h, nnnn = c28_pointer[15:0]
; By default it is set to 0000_0000h so it will point to DMEM1 address
    ldi     TEMP_REG, 0x0100
    sbco    &TEMP_REG, ICSS_PRU_CTRL_CONST, 0x28, 2
;For using mvib instruction load BNS_ARG_RETVAL_ADDR with BNS_ARG_RETVAL(R10.b0 pointer address)
    ldi     BNS_ARG_RETVAL_ADDR, 28
	ldi32   APPROX_VAL_MAX, 0xFFFFFF00
	ldi   APPROX_VAL_MIN, 256
    ldi    CH0_STATE, $CODE(CH0_SYNC)
    ldi    CH1_STATE, $CODE(CH1_SYNC)
    ldi    CH2_STATE, $CODE(CH2_SYNC)
    ldi    CH3_STATE, $CODE(CH3_SYNC)
    ldi    CH4_STATE, $CODE(CH4_SYNC)
    ldi    CH5_STATE, $CODE(CH5_SYNC)
    ldi    CH6_STATE, $CODE(CH6_SYNC)
    ldi    CH7_STATE, $CODE(CH7_SYNC)
loop_process:
;Preserve initial register state before fetching data
 	ldi     R0.b0, 0
    xout    PRU_SPAD_B1_XID, &R3, 32
;Fetch 36 byte STATE_CHANGE_MASK, 8 channels pulse_length from scratch pad
    xin     PRU_SPAD_B0_XID, &R2, 36
;Store 8 channels pulse_length(R3-R10) in SPAD0 from R20-R27
    ldi     R0.b0, 17
    xout    PRU_SPAD_B0_XID, &R3, 32
;Restore initial register state
  	ldi     R0.b0, 0
    xin    PRU_SPAD_B1_XID, &R3, 32
    ldi    TEMP_REG2, (CRC4_LUT_OFFSET+PDMEM00)
process_ch0:
; check if ch have data recieved
    qbbc    process_ch1, STATE_CHANGE_MASK, 0
;Set ACK to inform PRU0 that data is read
    ldi     R0, 16
    ldi     TEMP_REG, 1
    xout    PRU_SPAD_B0_XID, &TEMP_REG, 4
    ldi     BNS_ARG_LUTBASE, CH0_LUT_BASE
;Fetch pulse length from SPAD0 R20 to R3
    ldi     R0.b0, 17
    xin     PRU_SPAD_B0_XID, &R3, 4
    mov     BNS_ARG_VAL, CHx_PULSE_LEN
    ldi     BNS_CH_DATA_ERROR, $CODE(ch0_data_error)
    jal     return_addr1, CH0_STATE
process_ch1:
 	qbbc    process_ch2, STATE_CHANGE_MASK, 1
;Set ACK to inform PRU0 that data is read
    ldi     R0, 17
    ldi     TEMP_REG, 1
    xout    PRU_SPAD_B0_XID, &TEMP_REG, 4
    ldi     BNS_ARG_LUTBASE, CH1_LUT_BASE
;Fetch pulse length from SPAD0 R21 to R3
    ldi     R0.b0, 18
    xin     PRU_SPAD_B0_XID, &R3, 4
    mov     BNS_ARG_VAL, CHx_PULSE_LEN
    ldi     BNS_CH_DATA_ERROR, $CODE(ch1_data_error)
    jal     return_addr1, CH1_STATE
process_ch2:
    qbbc    process_ch3, STATE_CHANGE_MASK, 2
;Set ACK to inform PRU0 that data is read
    ldi     R0, 18
    ldi     TEMP_REG, 1
    xout    PRU_SPAD_B0_XID, &TEMP_REG, 4
    ldi     BNS_ARG_LUTBASE, CH2_LUT_BASE
;Fetch pulse length from SPAD0 R22 to R3
    ldi     R0.b0, 19
    xin     PRU_SPAD_B0_XID, &R3, 4
    mov     BNS_ARG_VAL, CHx_PULSE_LEN
    ldi     BNS_CH_DATA_ERROR, $CODE(ch2_data_error)
    jal     return_addr1, CH2_STATE
process_ch3:
    qbbc    process_ch4, STATE_CHANGE_MASK, 3
;Set ACK to inform PRU0 that data is read
    ldi     R0, 19
    ldi     TEMP_REG, 1
    xout    PRU_SPAD_B0_XID, &TEMP_REG, 4
    ldi     BNS_ARG_LUTBASE, CH3_LUT_BASE
;Fetch pulse length from SPAD0 R23 to R3
    ldi     R0.b0, 20
    xin     PRU_SPAD_B0_XID, &R3, 4
    mov     BNS_ARG_VAL, CHx_PULSE_LEN
    ldi     BNS_CH_DATA_ERROR, $CODE(ch3_data_error)
    jal     return_addr1, CH3_STATE
process_ch4:
    qbbc    process_ch5, STATE_CHANGE_MASK, 4
;Set ACK to inform PRU0 that data is read
    ldi     R0, 20
    ldi     TEMP_REG, 1
    xout    PRU_SPAD_B0_XID, &TEMP_REG, 4
    ldi     BNS_ARG_LUTBASE, CH4_LUT_BASE
;Fetch pulse length from SPAD0 R24 to R3
    ldi     R0.b0, 21
    xin     PRU_SPAD_B0_XID, &R3, 4
    mov     BNS_ARG_VAL, CHx_PULSE_LEN
    ldi     BNS_CH_DATA_ERROR, $CODE(ch4_data_error)
    jal     return_addr1, CH4_STATE
process_ch5:
    qbbc    process_ch6, STATE_CHANGE_MASK, 5
;Set ACK to inform PRU0 that data is read
    ldi     R0, 21
    ldi     TEMP_REG, 1
    xout    PRU_SPAD_B0_XID, &TEMP_REG, 4
    ldi     BNS_ARG_LUTBASE, CH5_LUT_BASE
;Fetch pulse length from SPAD0 R25 to R3
    ldi     R0.b0, 22
    xin     PRU_SPAD_B0_XID, &R3, 4
    mov     BNS_ARG_VAL, CHx_PULSE_LEN
    ldi     BNS_CH_DATA_ERROR, $CODE(ch5_data_error)
    jal     return_addr1, CH5_STATE
process_ch6:
    qbbc    process_ch7, STATE_CHANGE_MASK, 6
;Set ACK to inform PRU0 that data is read
    ldi     R0, 22
    ldi     TEMP_REG, 1
    xout    PRU_SPAD_B0_XID, &TEMP_REG, 4
    ldi     BNS_ARG_LUTBASE, CH6_LUT_BASE
;Fetch pulse length from SPAD0 R26 to R3
    ldi     R0.b0, 23
    xin     PRU_SPAD_B0_XID, &R3, 4
    mov     BNS_ARG_VAL, CHx_PULSE_LEN
    ldi     BNS_CH_DATA_ERROR, $CODE(ch6_data_error)
    jal     return_addr1, CH6_STATE
process_ch7:
    qbbc    restart_loop, STATE_CHANGE_MASK, 8
;Set ACK to inform PRU0 that data is read
    ldi     R0, 23
    ldi     TEMP_REG, 1
    xout    PRU_SPAD_B0_XID, &TEMP_REG, 4
    ldi     BNS_ARG_LUTBASE, CH7_LUT_BASE
;Fetch pulse length from SPAD0 R27 to R3
    ldi     R0.b0, 24
    xin     PRU_SPAD_B0_XID, &R3, 4
    mov     BNS_ARG_VAL, CHx_PULSE_LEN
    ldi     BNS_CH_DATA_ERROR, $CODE(ch7_data_error)
    jal     return_addr1, CH7_STATE
restart_loop:
	jmp     loop_process


CH0_SYNC:
;check if it is sync pulse
	ldi32 	TEMP_REG, ch0_syncpulse_max_dur
    qblt    ERROR_CH0, CHx_PULSE_LEN, TEMP_REG
    ldi32 	TEMP_REG, ch0_syncpulse_min_dur
    qbgt    ERROR_CH0, CHx_PULSE_LEN, TEMP_REG
;Calculate tick period by dividing pulse length with 56, Divide by 8, then by 7
    lsr     CHx_PULSE_LEN, CHx_PULSE_LEN, 3
    ldi32 	TEMP_REG, TICK_7_CONST
    m_calculate_tick TEMP_REG, CHx_PULSE_LEN
    mov     TEMP_REG0, R27.w0 ; load temp_re0 with update tick period
    mov     CH0_BUF.w0, TEMP_REG0.w0
;Update the Hash table with ch0 temp tickperiod placeholder(R0), store in hash table
	ldi 	TEMP_REG, 12
    m_multiply TEMP_REG, TEMP_REG0
;update the lut using R4 to R20 registers(16 reg)
    xout	PRU_SPAD_B1_XID, &R4, 64
    mov     R4, R26
    m_update_lut TEMP_REG0
;Store lookup table in dmem at CH0_LUT_BASE
    ldi    TEMP_REG, CH0_LUT_BASE
    sbbo    &R4, TEMP_REG, 0, 64
    xin 	PRU_SPAD_B1_XID, &R4, 64
;Update Ch0 state register for status & Comm data calc
  	ldi    CH0_STATE, $CODE(CH0_STATUS_COMM)
  	jmp     return_addr1

CH0_DATA0:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH0_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 15
    xout  PRU_SPAD_B2_XID, &CH0_BUF, 4
  	ldi   CH0_STATE, $CODE(CH0_DATA1)
    ldi   CH0_CRC4_RES, 5
    lsr   CH0_CRC4_RES, CH0_CRC4_RES, 4
    add   CH0_CRC4_RES, CH0_CRC4_RES, CH0_BUF.b3
    lbbo  &CH0_CRC4_RES, TEMP_REG2 , CH0_CRC4_RES, 1
    jmp   return_addr1
CH0_DATA2:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH0_BUF.b1, *BNS_ARG_RETVAL_ADDR
  	ldi   CH0_STATE, $CODE(CH0_DATA3)
    lsr   CH0_CRC4_RES, CH0_CRC4_RES, 4
    add   CH0_CRC4_RES, CH0_CRC4_RES, CH0_BUF.b1
    lbbo  &CH0_CRC4_RES, TEMP_REG2 , CH0_CRC4_RES, 1
    jmp   return_addr1
CH0_DATA4:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH0_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 16
    xout  PRU_SPAD_B2_XID, &CH0_BUF, 4
  	ldi   CH0_STATE, $CODE(CH0_DATA5)
    lsr   CH0_CRC4_RES, CH0_CRC4_RES, 4
    add   CH0_CRC4_RES, CH0_CRC4_RES, CH0_BUF.b3
    lbbo  &CH0_CRC4_RES, TEMP_REG2 , CH0_CRC4_RES, 1
    jmp   return_addr1
CH0_CRC:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH0_BUF.b1, *BNS_ARG_RETVAL_ADDR
    lsr   CH0_CRC4_RES, CH0_CRC4_RES, 4
    lbbo  &CH0_CRC4_RES, TEMP_REG2 , CH0_CRC4_RES, 1
    mov   CH0_BUF.b2, CH0_CRC4_RES
    mov   CH0_BUF.b3, BNS_ARG_STATUS
    ldi   R0.b0, 17
    xout  PRU_SPAD_B2_XID, &CH0_BUF, 4
    qbne  CH0_CRC_ERROR, CH0_CRC4_RES, 0
CH0_DATA_OUT:
;Put data in DMEM or Collect data for different mode
    ldi   R0, 0
;Store existing data first
    xout  PRU_SPAD_B1_XID, &R1, 12
    xin   PRU_SPAD_B2_XID, &R1, 12
;Write 10 byte of data in SMEM(Sync, Data and CRC)
    sbco  &R1, C28, CH0_DATA_BASE, 12
;Bring back stored data
    zero	&R1, 12
    xout  PRU_SPAD_B2_XID, &R1, 12
    xin   PRU_SPAD_B1_XID, &R1, 12
;Trigger Interrupt to R5F/*TODO*/
    ldi   R0, 1
    sbco  &R0, C28, CH0_INTR_BYTE, 1
;/*TO_DO: Add error handling mechnaism for each label below*/
CH0_CRC_ERROR:
ch0_data_error:
ERROR_CH0:
    ldi   CH0_STATE, $CODE(CH0_SYNC)
    jmp   return_addr1

CH0_STATUS_COMM:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH0_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH0_STATE, $CODE(CH0_DATA0)
    jmp   return_addr1
CH0_DATA1:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH0_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH0_STATE, $CODE(CH0_DATA2)
    lsr   CH0_CRC4_RES, CH0_CRC4_RES, 4
    add   CH0_CRC4_RES, CH0_CRC4_RES, CH0_BUF.b0
    lbbo  &CH0_CRC4_RES, TEMP_REG2 , CH0_CRC4_RES, 1

    jmp   return_addr1
CH0_DATA3:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH0_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH0_STATE, $CODE(CH0_DATA4)
    lsr   CH0_CRC4_RES, CH0_CRC4_RES, 4
    add   CH0_CRC4_RES, CH0_CRC4_RES, CH0_BUF.b2
    lbbo  &CH0_CRC4_RES, TEMP_REG2 , CH0_CRC4_RES, 1
    jmp   return_addr1
CH0_DATA5:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH0_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH0_STATE, $CODE(CH0_CRC)
    lsr   CH0_CRC4_RES, CH0_CRC4_RES, 4
    add   CH0_CRC4_RES, CH0_CRC4_RES, CH0_BUF.b0
    lbbo  &CH0_CRC4_RES, TEMP_REG2 , CH0_CRC4_RES, 1
    jmp   return_addr1


CH1_SYNC:
;check if it is sync pulse
	ldi32 	TEMP_REG, ch1_syncpulse_max_dur
    qblt    ERROR_CH1, CHx_PULSE_LEN, TEMP_REG
    ldi32 	TEMP_REG, ch1_syncpulse_min_dur
    qbgt    ERROR_CH1, CHx_PULSE_LEN, TEMP_REG
;Calculate tick period by dividing pulse length with 56, Divide by 8, then by 7
    lsr     CHx_PULSE_LEN, CHx_PULSE_LEN, 3
    ldi32 	TEMP_REG, TICK_7_CONST
    m_calculate_tick TEMP_REG, CHx_PULSE_LEN
    mov     TEMP_REG0, R27.w0 ; load temp_re0 with update tick period
    mov     CH1_BUF.w0, TEMP_REG0.w0
;Update the Hash table with ch1 temp tickperiod placeholder(R0), store in hash table
	ldi 	TEMP_REG, 12
    m_multiply TEMP_REG, TEMP_REG0
;update the lut using R4 to R20 registers(16 reg)
    xout	PRU_SPAD_B1_XID, &R4, 64
    mov     R4, R26
    m_update_lut TEMP_REG0
;Store lookup table in dmem at CH1_LUT_BASE
    ldi     TEMP_REG, CH1_LUT_BASE
    sbbo    &R4, TEMP_REG, 0, 64
    xin 	PRU_SPAD_B1_XID, &R4, 64
;Update ch1 state register for status & Comm data calc
  	ldi    CH1_STATE, $CODE(CH1_STATUS_COMM)
  	jmp     return_addr1

CH1_DATA0:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH1_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 17
    xout  PRU_SPAD_B2_XID, &CH1_BUF, 4
  	ldi   CH1_STATE, $CODE(CH1_DATA1)
    ldi   CH1_CRC4_RES, 5
    lsr   CH1_CRC4_RES, CH1_CRC4_RES, 4
    add   CH1_CRC4_RES, CH1_CRC4_RES, CH1_BUF.b3
    lbbo  &CH1_CRC4_RES, TEMP_REG2 , CH1_CRC4_RES, 1
    jmp   return_addr1
CH1_DATA2:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH1_BUF.b1, *BNS_ARG_RETVAL_ADDR
  	ldi   CH1_STATE, $CODE(CH1_DATA3)
    lsr   CH1_CRC4_RES, CH1_CRC4_RES, 4
    add   CH1_CRC4_RES, CH1_CRC4_RES, CH1_BUF.b1
    lbbo  &CH1_CRC4_RES, TEMP_REG2 , CH1_CRC4_RES, 1
    jmp   return_addr1
CH1_DATA4:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH1_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 18
    xout  PRU_SPAD_B2_XID, &CH1_BUF, 4
  	ldi   CH1_STATE, $CODE(CH1_DATA5)
    lsr   CH1_CRC4_RES, CH1_CRC4_RES, 4
    add   CH1_CRC4_RES, CH1_CRC4_RES, CH1_BUF.b3
    lbbo  &CH1_CRC4_RES, TEMP_REG2 , CH1_CRC4_RES, 1
    jmp   return_addr1
CH1_CRC:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH1_BUF.b1, *BNS_ARG_RETVAL_ADDR
    lsr   CH1_CRC4_RES, CH1_CRC4_RES, 4
    lbbo  &CH1_CRC4_RES, TEMP_REG2 , CH1_CRC4_RES, 1
    mov   CH1_BUF.b2, CH1_CRC4_RES
    mov   CH1_BUF.b3, BNS_ARG_STATUS
    ldi   R0.b0, 19
    xout  PRU_SPAD_B2_XID, &CH1_BUF, 4
    qbne  CH1_CRC_ERROR, CH1_CRC4_RES, 0
CH1_DATA_OUT:
;Put data in DMEM or Collect data for different mode
    ldi   R0, 0
;Store existing data first
    xout  PRU_SPAD_B1_XID, &R4, 12
    xin   PRU_SPAD_B2_XID, &R4, 12
;Write 10 byte of data in SMEM(Sync, Data and CRC)
    sbco  &R4, C28, CH1_DATA_BASE, 12
;Bring back stored data
    zero  &R4, 12
    xout  PRU_SPAD_B2_XID, &R4, 12
    xin   PRU_SPAD_B1_XID, &R4, 12
;Trigger Interrupt to R5F/*TODO*/
    ldi   R0, 1
    sbco  &R0, C28, CH1_INTR_BYTE, 1
;/*TO_DO: Add error handling mechnaism for each label below*/
CH1_CRC_ERROR:
ch1_data_error:
ERROR_CH1:
    ldi   CH1_STATE, $CODE(CH1_SYNC)
    jmp   return_addr1

CH1_STATUS_COMM:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH1_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH1_STATE, $CODE(CH1_DATA0)
    jmp   return_addr1
CH1_DATA1:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH1_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH1_STATE, $CODE(CH1_DATA2)
    lsr   CH1_CRC4_RES, CH1_CRC4_RES, 4
    add   CH1_CRC4_RES, CH1_CRC4_RES, CH1_BUF.b0
    lbbo  &CH1_CRC4_RES, TEMP_REG2 , CH1_CRC4_RES, 1

    jmp   return_addr1
CH1_DATA3:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH1_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH1_STATE, $CODE(CH1_DATA4)
    lsr   CH1_CRC4_RES, CH1_CRC4_RES, 4
    add   CH1_CRC4_RES, CH1_CRC4_RES, CH1_BUF.b2
    lbbo  &CH1_CRC4_RES, TEMP_REG2 , CH1_CRC4_RES, 1
    jmp   return_addr1
CH1_DATA5:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH1_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH1_STATE, $CODE(CH1_CRC)
    lsr   CH1_CRC4_RES, CH1_CRC4_RES, 4
    add   CH1_CRC4_RES, CH1_CRC4_RES, CH1_BUF.b0
    lbbo  &CH1_CRC4_RES, TEMP_REG2 , CH1_CRC4_RES, 1
    jmp   return_addr1

CH2_SYNC:
;check if it is sync pulse
	ldi32 	TEMP_REG, ch2_syncpulse_max_dur
    qblt    ERROR_CH2, CHx_PULSE_LEN, TEMP_REG
    ldi32 	TEMP_REG, ch2_syncpulse_min_dur
    qbgt    ERROR_CH2, CHx_PULSE_LEN, TEMP_REG
;Calculate tick period by dividing pulse length with 56, Divide by 8, then by 7
    lsr     CHx_PULSE_LEN, CHx_PULSE_LEN, 3
    ldi32 	TEMP_REG, TICK_7_CONST
    m_calculate_tick TEMP_REG, CHx_PULSE_LEN
    mov     TEMP_REG0, R27.w0 ; load temp_re0 with update tick period
    mov     CH2_BUF.w0, TEMP_REG0.w0
;Update the Hash table with ch2 temp tickperiod placeholder(R0), store in hash table
	ldi 	TEMP_REG, 12
    m_multiply TEMP_REG, TEMP_REG0
;update the lut using R4 to R20 registers(16 reg)
    xout	PRU_SPAD_B1_XID, &R4, 64
    mov     R4, R26
    m_update_lut TEMP_REG0
;Store lookup table in dmem at CH2_LUT_BASE
    ldi     TEMP_REG, CH2_LUT_BASE
    sbbo    &R4, TEMP_REG, 0, 64
    xin 	PRU_SPAD_B1_XID, &R4, 64
;Update ch2 state register for status & Comm data calc
  	ldi    CH2_STATE, $CODE(CH2_STATUS_COMM)
  	jmp     return_addr1

CH2_DATA0:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH2_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 19
    xout  PRU_SPAD_B2_XID, &CH2_BUF, 4
  	ldi   CH2_STATE, $CODE(CH2_DATA1)
    ldi   CH2_CRC4_RES, 5
    lsr   CH2_CRC4_RES, CH2_CRC4_RES, 4
    add   CH2_CRC4_RES, CH2_CRC4_RES, CH2_BUF.b3
    lbbo  &CH2_CRC4_RES, TEMP_REG2 , CH2_CRC4_RES, 1
    jmp   return_addr1
CH2_DATA2:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH2_BUF.b1, *BNS_ARG_RETVAL_ADDR
  	ldi   CH2_STATE, $CODE(CH2_DATA3)
    lsr   CH2_CRC4_RES, CH2_CRC4_RES, 4
    add   CH2_CRC4_RES, CH2_CRC4_RES, CH2_BUF.b1
    lbbo  &CH2_CRC4_RES, TEMP_REG2 , CH2_CRC4_RES, 1
    jmp   return_addr1
CH2_DATA4:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH2_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 20
    xout  PRU_SPAD_B2_XID, &CH2_BUF, 4
  	ldi   CH2_STATE, $CODE(CH2_DATA5)
    lsr   CH2_CRC4_RES, CH2_CRC4_RES, 4
    add   CH2_CRC4_RES, CH2_CRC4_RES, CH2_BUF.b3
    lbbo  &CH2_CRC4_RES, TEMP_REG2 , CH2_CRC4_RES, 1
    jmp   return_addr1
CH2_CRC:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH2_BUF.b1, *BNS_ARG_RETVAL_ADDR
    lsr   CH2_CRC4_RES, CH2_CRC4_RES, 4
    lbbo  &CH2_CRC4_RES, TEMP_REG2 , CH2_CRC4_RES, 1
    mov   CH2_BUF.b2, CH2_CRC4_RES
    mov   CH2_BUF.b3, BNS_ARG_STATUS
    ldi   R0.b0, 21
    xout  PRU_SPAD_B2_XID, &CH2_BUF, 4
    qbne  CH2_CRC_ERROR, CH2_CRC4_RES, 0
CH2_DATA_OUT:
;Put data in DMEM or Collect data for different mode
    ldi   R0, 0
;Store existing data first
    xout  PRU_SPAD_B1_XID, &R7, 12
    xin   PRU_SPAD_B2_XID, &R7, 12
;Write 10 byte of data in SMEM(Sync, Data and CRC)
    sbco  &R7, C28, CH2_DATA_BASE, 12
;Bring back stored data
    zero	&R7, 12
    xout  PRU_SPAD_B2_XID, &R7, 12
    xin   PRU_SPAD_B1_XID, &R7, 12
;Trigger Interrupt to R5F/*TODO*/
    ldi   R0, 1
    sbco  &R0, C28, CH2_INTR_BYTE, 1
;/*TO_DO: Add error handling mechnaism for each label below*/
CH2_CRC_ERROR:
ch2_data_error:
ERROR_CH2:
;Send data to r5f
    ldi    CH2_STATE, $CODE(CH2_SYNC)
  	jmp    return_addr1


CH2_STATUS_COMM:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH2_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH2_STATE, $CODE(CH2_DATA0)
    jmp   return_addr1
CH2_DATA1:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH2_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH2_STATE, $CODE(CH2_DATA2)
    lsr   CH2_CRC4_RES, CH2_CRC4_RES, 4
    add   CH2_CRC4_RES, CH2_CRC4_RES, CH2_BUF.b0
    lbbo  &CH2_CRC4_RES, TEMP_REG2 , CH2_CRC4_RES, 1
    jmp   return_addr1
CH2_DATA3:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH2_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH2_STATE, $CODE(CH2_DATA4)
    lsr   CH2_CRC4_RES, CH2_CRC4_RES, 4
    add   CH2_CRC4_RES, CH2_CRC4_RES, CH2_BUF.b2
    lbbo  &CH2_CRC4_RES, TEMP_REG2 , CH2_CRC4_RES, 1
    jmp   return_addr1
CH2_DATA5:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH2_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH2_STATE, $CODE(CH2_CRC)
    lsr   CH2_CRC4_RES, CH2_CRC4_RES, 4
    add   CH2_CRC4_RES, CH2_CRC4_RES, CH2_BUF.b0
    lbbo  &CH2_CRC4_RES, TEMP_REG2 , CH2_CRC4_RES, 1
    jmp   return_addr1

CH3_SYNC:
;check if it is sync pulse
	ldi32 	TEMP_REG, ch3_syncpulse_max_dur
    qblt    ERROR_CH3, CHx_PULSE_LEN, TEMP_REG
    ldi32 	TEMP_REG, ch3_syncpulse_min_dur
    qbgt    ERROR_CH3, CHx_PULSE_LEN, TEMP_REG
;Calculate tick period by dividing pulse length with 56, Divide by 8, then by 7
    lsr     CHx_PULSE_LEN, CHx_PULSE_LEN, 3
    ldi32 	TEMP_REG, TICK_7_CONST
    m_calculate_tick TEMP_REG, CHx_PULSE_LEN
    mov     TEMP_REG0, R27.w0 ; load temp_re0 with update tick period
    mov     CH3_BUF.w0, TEMP_REG0.w0
;Update the Hash table with ch3 temp tickperiod placeholder(R0), store in hash table
	ldi 	TEMP_REG, 12
    m_multiply TEMP_REG, TEMP_REG0
;update the lut using R4 to R20 registers(16 reg)
    xout	PRU_SPAD_B1_XID, &R4, 64
    mov     R4, R26
    m_update_lut TEMP_REG0
;Store lookup table in dmem at CH3_LUT_BASE
    ldi     TEMP_REG, CH3_LUT_BASE
    sbbo    &R4, TEMP_REG, 0, 64
    xin 	PRU_SPAD_B1_XID, &R4, 64
;Update ch3 state register for status & Comm data calc
  	ldi    CH3_STATE, $CODE(CH3_STATUS_COMM)
  	jmp     return_addr1

CH3_DATA0:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH3_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 21
    xout  PRU_SPAD_B2_XID, &CH3_BUF, 4
  	ldi   CH3_STATE, $CODE(CH3_DATA1)
    ldi   CH3_CRC4_RES, 5
    lsr   CH3_CRC4_RES, CH3_CRC4_RES, 4
    add   CH3_CRC4_RES, CH3_CRC4_RES, CH3_BUF.b3
    lbbo  &CH3_CRC4_RES, TEMP_REG2 , CH3_CRC4_RES, 1
    jmp   return_addr1
CH3_DATA2:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH3_BUF.b1, *BNS_ARG_RETVAL_ADDR
  	ldi   CH3_STATE, $CODE(CH3_DATA3)
    lsr   CH3_CRC4_RES, CH3_CRC4_RES, 4
    add   CH3_CRC4_RES, CH3_CRC4_RES, CH3_BUF.b1
    lbbo  &CH3_CRC4_RES, TEMP_REG2 , CH3_CRC4_RES, 1
    jmp   return_addr1
CH3_DATA4:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH3_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 22
    xout  PRU_SPAD_B2_XID, &CH3_BUF, 4
  	ldi   CH3_STATE, $CODE(CH3_DATA5)
    lsr   CH3_CRC4_RES, CH3_CRC4_RES, 4
    add   CH3_CRC4_RES, CH3_CRC4_RES, CH3_BUF.b3
    lbbo  &CH3_CRC4_RES, TEMP_REG2 , CH3_CRC4_RES, 1
    jmp   return_addr1
CH3_CRC:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH3_BUF.b1, *BNS_ARG_RETVAL_ADDR
    lsr   CH3_CRC4_RES, CH3_CRC4_RES, 4
    lbbo  &CH3_CRC4_RES, TEMP_REG2 , CH3_CRC4_RES, 1
    mov   CH3_BUF.b2, CH3_CRC4_RES
    mov   CH3_BUF.b3, BNS_ARG_STATUS
    ldi   R0.b0, 23
    xout  PRU_SPAD_B2_XID, &CH3_BUF, 4
    qbne  CH3_CRC_ERROR, CH3_CRC4_RES, 0
CH3_DATA_OUT:
;Put data in DMEM or Collect data for different mode
    ldi   R0, 0
;Store existing data first
    xout  PRU_SPAD_B1_XID, &R10, 12
    xin   PRU_SPAD_B2_XID, &R10, 12
;Write 10 byte of data in SMEM(Sync, Data and CRC)
    sbco  &R10, C28, CH3_DATA_BASE, 12
;Bring back stored data
    zero  &R10, 12
    xout  PRU_SPAD_B2_XID, &R10, 12
    xin   PRU_SPAD_B1_XID, &R10, 12
;Trigger Interrupt to R5F/*TODO*/
    ldi   R0, 1
    sbco  &R0, C28, CH3_INTR_BYTE, 1
;/*TO_DO: Add error handling mechnaism for each label below*/
CH3_CRC_ERROR:
ch3_data_error:
ERROR_CH3:
;Send data to r5f and reset channel state
    ldi    CH3_STATE, $CODE(CH3_SYNC)
  	jmp    return_addr1

CH3_STATUS_COMM:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH3_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH3_STATE, $CODE(CH3_DATA0)
    jmp   return_addr1
CH3_DATA1:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH3_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH3_STATE, $CODE(CH3_DATA2)
    lsr   CH3_CRC4_RES, CH3_CRC4_RES, 4
    add   CH3_CRC4_RES, CH3_CRC4_RES, CH3_BUF.b0
    lbbo  &CH3_CRC4_RES, TEMP_REG2 , CH3_CRC4_RES, 1

    jmp   return_addr1
CH3_DATA3:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH3_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH3_STATE, $CODE(CH3_DATA4)
    lsr   CH3_CRC4_RES, CH3_CRC4_RES, 4
    add   CH3_CRC4_RES, CH3_CRC4_RES, CH3_BUF.b2
    lbbo  &CH3_CRC4_RES, TEMP_REG2 , CH3_CRC4_RES, 1
    jmp   return_addr1
CH3_DATA5:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH3_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH3_STATE, $CODE(CH3_CRC)
    lsr   CH3_CRC4_RES, CH3_CRC4_RES, 4
    add   CH3_CRC4_RES, CH3_CRC4_RES, CH3_BUF.b0
    lbbo  &CH3_CRC4_RES, TEMP_REG2 , CH3_CRC4_RES, 1
    jmp   return_addr1


CH4_SYNC:
;check if it is sync pulse
	ldi32 	TEMP_REG, ch4_syncpulse_max_dur
    qblt    ERROR_CH4, CHx_PULSE_LEN, TEMP_REG
    ldi32 	TEMP_REG, ch4_syncpulse_min_dur
    qbgt    ERROR_CH4, CHx_PULSE_LEN, TEMP_REG
;Calculate tick period by dividing pulse length with 56, Divide by 8, then by 7
    lsr     CHx_PULSE_LEN, CHx_PULSE_LEN, 3
    ldi32 	TEMP_REG, TICK_7_CONST
    m_calculate_tick TEMP_REG, CHx_PULSE_LEN
    mov     TEMP_REG0, R27.w0 ; load temp_re0 with update tick period
    mov     CH4_BUF.w0, TEMP_REG0.w0
;Update the Hash table with ch4 temp tickperiod placeholder(R0), store in hash table
	ldi 	TEMP_REG, 12
    m_multiply TEMP_REG, TEMP_REG0
;update the lut using R4 to R20 registers(16 reg)
    xout	PRU_SPAD_B1_XID, &R4, 64
    mov     R4, R26
    m_update_lut TEMP_REG0
;Store lookup table in dmem at CH4_LUT_BASE
    ldi     TEMP_REG, CH4_LUT_BASE
    sbbo    &R4, TEMP_REG, 0, 64
    xin 	PRU_SPAD_B1_XID, &R4, 64
;Update Ch4 state register for status & Comm data calc
  	ldi    CH4_STATE, $CODE(CH4_STATUS_COMM)
  	jmp     return_addr1

CH4_DATA0:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH4_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 23
    xout  PRU_SPAD_B2_XID, &CH4_BUF, 4
  	ldi   CH4_STATE, $CODE(CH4_DATA1)
    ldi   CH4_CRC4_RES, 5
    lsr   CH4_CRC4_RES, CH4_CRC4_RES, 4
    add   CH4_CRC4_RES, CH4_CRC4_RES, CH4_BUF.b3
    lbbo  &CH4_CRC4_RES, TEMP_REG2 , CH4_CRC4_RES, 1
    jmp   return_addr1
CH4_DATA2:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH4_BUF.b1, *BNS_ARG_RETVAL_ADDR
  	ldi   CH4_STATE, $CODE(CH4_DATA3)
    lsr   CH4_CRC4_RES, CH4_CRC4_RES, 4
    add   CH4_CRC4_RES, CH4_CRC4_RES, CH4_BUF.b1
    lbbo  &CH4_CRC4_RES, TEMP_REG2 , CH4_CRC4_RES, 1
    jmp   return_addr1
CH4_DATA4:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH4_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 24
    xout  PRU_SPAD_B2_XID, &CH4_BUF, 4
  	ldi   CH4_STATE, $CODE(CH4_DATA5)
    lsr   CH4_CRC4_RES, CH4_CRC4_RES, 4
    add   CH4_CRC4_RES, CH4_CRC4_RES, CH4_BUF.b3
    lbbo  &CH4_CRC4_RES, TEMP_REG2 , CH4_CRC4_RES, 1
    jmp   return_addr1
CH4_CRC:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH4_BUF.b1, *BNS_ARG_RETVAL_ADDR
    lsr   CH4_CRC4_RES, CH4_CRC4_RES, 4
    lbbo  &CH4_CRC4_RES, TEMP_REG2 , CH4_CRC4_RES, 1
    mov   CH4_BUF.b2, CH4_CRC4_RES
    mov   CH4_BUF.b3, BNS_ARG_STATUS
    ldi   R0.b0, 25
    xout  PRU_SPAD_B2_XID, &CH4_BUF, 4
    qbne  CH4_CRC_ERROR, CH4_CRC4_RES, 0
CH4_DATA_OUT:
;Put data in DMEM or Collect data for different mode
    ldi   R0, 0
;Store existing data first
    xout  PRU_SPAD_B1_XID, &R13, 12
    xin   PRU_SPAD_B2_XID, &R13, 12
;Write 10 byte of data in SMEM(Sync, Data and CRC)
    sbco  &R13, C28, CH4_DATA_BASE, 12
;Bring back stored data
    zero	&R13, 12
    xout  PRU_SPAD_B2_XID, &R13, 12
    xin   PRU_SPAD_B1_XID, &R13, 12
;Trigger Interrupt to R5F/*TODO*/
    ldi   R0, 1
    sbco  &R0, C28, CH4_INTR_BYTE, 1
;/*TO_DO: Add error handling mechnaism for each label below*/
CH4_CRC_ERROR:
ch4_data_error:
ERROR_CH4:
;Send data to r5f and reset channel state
    ldi    CH4_STATE, $CODE(CH4_SYNC)
  	jmp    return_addr1

CH4_STATUS_COMM:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH4_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH4_STATE, $CODE(CH4_DATA0)
    jmp   return_addr1
CH4_DATA1:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH4_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH4_STATE, $CODE(CH4_DATA2)
    lsr   CH4_CRC4_RES, CH4_CRC4_RES, 4
    add   CH4_CRC4_RES, CH4_CRC4_RES, CH4_BUF.b0
    lbbo  &CH4_CRC4_RES, TEMP_REG2 , CH4_CRC4_RES, 1

    jmp   return_addr1
CH4_DATA3:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH4_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH4_STATE, $CODE(CH4_DATA4)
    lsr   CH4_CRC4_RES, CH4_CRC4_RES, 4
    add   CH4_CRC4_RES, CH4_CRC4_RES, CH4_BUF.b2
    lbbo  &CH4_CRC4_RES, TEMP_REG2 , CH4_CRC4_RES, 1
    jmp   return_addr1
CH4_DATA5:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH4_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH4_STATE, $CODE(CH4_CRC)
    lsr   CH4_CRC4_RES, CH4_CRC4_RES, 4
    add   CH4_CRC4_RES, CH4_CRC4_RES, CH4_BUF.b0
    lbbo  &CH4_CRC4_RES, TEMP_REG2 , CH4_CRC4_RES, 1
    jmp   return_addr1


CH5_SYNC:
;check if it is sync pulse
	ldi32 	TEMP_REG, ch5_syncpulse_max_dur
    qblt    ERROR_CH5, CHx_PULSE_LEN, TEMP_REG
    ldi32 	TEMP_REG, ch5_syncpulse_min_dur
    qbgt    ERROR_CH5, CHx_PULSE_LEN, TEMP_REG
;Calculate tick period by dividing pulse length with 56, Divide by 8, then by 7
    lsr     CHx_PULSE_LEN, CHx_PULSE_LEN, 3
    ldi32 	TEMP_REG, TICK_7_CONST
    m_calculate_tick TEMP_REG, CHx_PULSE_LEN
    mov     TEMP_REG0, R27.w0 ; load temp_re0 with update tick period
    mov     CH5_BUF.w0, TEMP_REG0.w0
;Update the Hash table with ch5 temp tickperiod placeholder(R0), store in hash table
	ldi 	TEMP_REG, 12
    m_multiply TEMP_REG, TEMP_REG0
;update the lut using R4 to R20 registers(16 reg)
    xout	PRU_SPAD_B1_XID, &R4, 64
    mov     R4, R26
    m_update_lut TEMP_REG0
;Store lookup table in dmem at CH5_LUT_BASE
    ldi     TEMP_REG, CH5_LUT_BASE
    sbbo    &R4, TEMP_REG, 0, 64
    xin 	PRU_SPAD_B1_XID, &R4, 64
;Update Ch5 state register for status & Comm data calc
  	ldi    CH5_STATE, $CODE(CH5_STATUS_COMM)
  	jmp     return_addr1

CH5_DATA0:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH5_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 25
    xout  PRU_SPAD_B2_XID, &CH5_BUF, 4
  	ldi   CH5_STATE, $CODE(CH5_DATA1)
    ldi   CH5_CRC4_RES, 5
    lsr   CH5_CRC4_RES, CH5_CRC4_RES, 4
    add   CH5_CRC4_RES, CH5_CRC4_RES, CH5_BUF.b3
    lbbo  &CH5_CRC4_RES, TEMP_REG2 , CH5_CRC4_RES, 1
    jmp   return_addr1
CH5_DATA2:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH5_BUF.b1, *BNS_ARG_RETVAL_ADDR
  	ldi   CH5_STATE, $CODE(CH5_DATA3)
    lsr   CH5_CRC4_RES, CH5_CRC4_RES, 4
    add   CH5_CRC4_RES, CH5_CRC4_RES, CH5_BUF.b1
    lbbo  &CH5_CRC4_RES, TEMP_REG2 , CH5_CRC4_RES, 1
    jmp   return_addr1
CH5_DATA4:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH5_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 26
    xout  PRU_SPAD_B2_XID, &CH5_BUF, 4
  	ldi   CH5_STATE, $CODE(CH5_DATA5)
    lsr   CH5_CRC4_RES, CH5_CRC4_RES, 4
    add   CH5_CRC4_RES, CH5_CRC4_RES, CH5_BUF.b3
    lbbo  &CH5_CRC4_RES, TEMP_REG2 , CH5_CRC4_RES, 1
    jmp   return_addr1
CH5_CRC:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH5_BUF.b1, *BNS_ARG_RETVAL_ADDR
    lsr   CH5_CRC4_RES, CH5_CRC4_RES, 4
    lbbo  &CH5_CRC4_RES, TEMP_REG2 , CH5_CRC4_RES, 1
    mov   CH5_BUF.b2, CH5_CRC4_RES
    mov   CH5_BUF.b3, BNS_ARG_STATUS
    ldi   R0.b0, 27
    xout  PRU_SPAD_B2_XID, &CH5_BUF, 4
    qbne  CH5_CRC_ERROR, CH5_CRC4_RES, 0
CH5_DATA_OUT:
;Put data in DMEM or Collect data for different mode
    ldi   R0, 0
;Store existing data first
    xout  PRU_SPAD_B1_XID, &R16, 12
    xin   PRU_SPAD_B2_XID, &R16, 12
;Write 10 byte of data in SMEM(Sync, Data and CRC)
    sbco  &R16, C28, CH5_DATA_BASE, 12
;Bring back stored data
    zero  &R16, 12
    xout  PRU_SPAD_B2_XID, &R16, 12
    xin   PRU_SPAD_B1_XID, &R16, 12
;Trigger Interrupt to R5F/*TODO*/
    ldi   R0, 1
    sbco  &R0, C28, CH5_INTR_BYTE, 1
;/*TO_DO: Add error handling mechnaism for each label below*/
CH5_CRC_ERROR:
ch5_data_error:
ERROR_CH5:
;Send data to r5f
    ldi    CH5_STATE, $CODE(CH5_SYNC)
  	jmp    return_addr1

CH5_STATUS_COMM:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH5_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH5_STATE, $CODE(CH5_DATA0)
    jmp   return_addr1
CH5_DATA1:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH5_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH5_STATE, $CODE(CH5_DATA2)
    lsr   CH5_CRC4_RES, CH5_CRC4_RES, 4
    add   CH5_CRC4_RES, CH5_CRC4_RES, CH5_BUF.b0
    lbbo  &CH5_CRC4_RES, TEMP_REG2 , CH5_CRC4_RES, 1

    jmp   return_addr1
CH5_DATA3:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH5_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH5_STATE, $CODE(CH5_DATA4)
    lsr   CH5_CRC4_RES, CH5_CRC4_RES, 4
    add   CH5_CRC4_RES, CH5_CRC4_RES, CH5_BUF.b2
    lbbo  &CH5_CRC4_RES, TEMP_REG2 , CH5_CRC4_RES, 1
    jmp   return_addr1
CH5_DATA5:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH5_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH5_STATE, $CODE(CH5_CRC)
    lsr   CH5_CRC4_RES, CH5_CRC4_RES, 4
    add   CH5_CRC4_RES, CH5_CRC4_RES, CH5_BUF.b0
    lbbo  &CH5_CRC4_RES, TEMP_REG2 , CH5_CRC4_RES, 1
    jmp   return_addr1

CH6_SYNC:
;check if it is sync pulse
	ldi32 	TEMP_REG, ch6_syncpulse_max_dur
    qblt    ERROR_CH6, CHx_PULSE_LEN, TEMP_REG
    ldi32 	TEMP_REG, ch6_syncpulse_min_dur
    qbgt    ERROR_CH6, CHx_PULSE_LEN, TEMP_REG
;Calculate tick period by dividing pulse length with 56, Divide by 8, then by 7
    lsr     CHx_PULSE_LEN, CHx_PULSE_LEN, 3
    ldi32 	TEMP_REG, TICK_7_CONST
    m_calculate_tick TEMP_REG, CHx_PULSE_LEN
    mov     TEMP_REG0, R27.w0 ; load temp_re0 with update tick period
    mov     CH6_BUF.w0, TEMP_REG0.w0
;Update the Hash table with ch6 temp tickperiod placeholder(R0), store in hash table
	ldi 	TEMP_REG, 12
    m_multiply TEMP_REG, TEMP_REG0
;update the lut using R4 to R20 registers(16 reg)
    xout	PRU_SPAD_B1_XID, &R4, 64
    mov     R4, R26
    m_update_lut TEMP_REG0
;Store lookup table in dmem at CH6_LUT_BASE
    ldi     TEMP_REG, CH6_LUT_BASE
    sbbo    &R4, TEMP_REG, 0, 64
    xin 	PRU_SPAD_B1_XID, &R4, 64
;Update Ch6 state register for status & Comm data calc
  	ldi    CH6_STATE, $CODE(CH6_STATUS_COMM)
  	jmp     return_addr1

CH6_DATA0:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH6_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 27
    xout  PRU_SPAD_B2_XID, &CH6_BUF, 4
  	ldi   CH6_STATE, $CODE(CH6_DATA1)
    ldi   CH6_CRC4_RES, 5
    lsr   CH6_CRC4_RES, CH6_CRC4_RES, 4
    add   CH6_CRC4_RES, CH6_CRC4_RES, CH6_BUF.b3
    lbbo  &CH6_CRC4_RES, TEMP_REG2 , CH6_CRC4_RES, 1
    jmp   return_addr1
CH6_DATA2:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH6_BUF.b1, *BNS_ARG_RETVAL_ADDR
  	ldi   CH6_STATE, $CODE(CH6_DATA3)
    lsr   CH6_CRC4_RES, CH6_CRC4_RES, 4
    add   CH6_CRC4_RES, CH6_CRC4_RES, CH6_BUF.b1
    lbbo  &CH6_CRC4_RES, TEMP_REG2 , CH6_CRC4_RES, 1
    jmp   return_addr1
CH6_DATA4:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH6_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in SPAD2 for sending it to r5f
    ldi   R0.b0, 28
    xout  PRU_SPAD_B2_XID, &CH6_BUF, 4
  	ldi   CH6_STATE, $CODE(CH6_DATA5)
    lsr   CH6_CRC4_RES, CH6_CRC4_RES, 4
    add   CH6_CRC4_RES, CH6_CRC4_RES, CH6_BUF.b3
    lbbo  &CH6_CRC4_RES, TEMP_REG2 , CH6_CRC4_RES, 1
    jmp   return_addr1
CH6_CRC:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH6_BUF.b1, *BNS_ARG_RETVAL_ADDR
    lsr   CH6_CRC4_RES, CH6_CRC4_RES, 4
    lbbo  &CH6_CRC4_RES, TEMP_REG2 , CH6_CRC4_RES, 1
    mov   CH6_BUF.b2, CH6_CRC4_RES
    mov   CH6_BUF.b3, BNS_ARG_STATUS
    ldi   R0.b0, 29
    xout  PRU_SPAD_B2_XID, &CH6_BUF, 4
    qbne  CH6_CRC_ERROR, CH6_CRC4_RES, 0
CH6_DATA_OUT:
;Put data in DMEM or Collect data for different mode
    ldi   R0, 0
;Store existing data first
    xout  PRU_SPAD_B1_XID, &R19, 12
    xin   PRU_SPAD_B2_XID, &R19, 12
;Write 10 byte of data in SMEM(Sync, Data and CRC)
    sbco  &R19, C28, CH6_DATA_BASE, 12
;Bring back stored data
    zero  &R19, 12
    xout  PRU_SPAD_B2_XID, &R19, 12
    xin   PRU_SPAD_B1_XID, &R19, 12
;Trigger Interrupt to R5F/*TODO*/
    ldi   R0, 1
    sbco  &R0, C28, CH6_INTR_BYTE, 1
;/*TO_DO: Add error handling mechnaism for each label below*/
CH6_CRC_ERROR:
ch6_data_error:
ERROR_CH6:
;Send data to r5f and reset channel state
    ldi    CH6_STATE, $CODE(CH6_SYNC)
  	jmp    return_addr1


CH6_STATUS_COMM:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH6_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH6_STATE, $CODE(CH6_DATA0)
    jmp   return_addr1
CH6_DATA1:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH6_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH6_STATE, $CODE(CH6_DATA2)
    lsr   CH6_CRC4_RES, CH6_CRC4_RES, 4
    add   CH6_CRC4_RES, CH6_CRC4_RES, CH6_BUF.b0
    lbbo  &CH6_CRC4_RES, TEMP_REG2 , CH6_CRC4_RES, 1

    jmp   return_addr1
CH6_DATA3:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH6_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH6_STATE, $CODE(CH6_DATA4)
    lsr   CH6_CRC4_RES, CH6_CRC4_RES, 4
    add   CH6_CRC4_RES, CH6_CRC4_RES, CH6_BUF.b2
    lbbo  &CH6_CRC4_RES, TEMP_REG2 , CH6_CRC4_RES, 1
    jmp   return_addr1
CH6_DATA5:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH6_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH6_STATE, $CODE(CH6_CRC)
    lsr   CH6_CRC4_RES, CH6_CRC4_RES, 4
    add   CH6_CRC4_RES, CH6_CRC4_RES, CH6_BUF.b0
    lbbo  &CH6_CRC4_RES, TEMP_REG2 , CH6_CRC4_RES, 1
    jmp   return_addr1


CH7_SYNC:
;check if it is sync pulse
	ldi32 	TEMP_REG, ch7_syncpulse_max_dur
    qblt    ERROR_CH7, CHx_PULSE_LEN, TEMP_REG
    ldi32 	TEMP_REG, ch7_syncpulse_min_dur
    qbgt    ERROR_CH7, CHx_PULSE_LEN, TEMP_REG
;Calculate tick period by dividing pulse length with 56, Divide by 8, then by 7
    lsr     CHx_PULSE_LEN, CHx_PULSE_LEN, 3
    ldi32 	TEMP_REG, TICK_7_CONST
    m_calculate_tick TEMP_REG, CHx_PULSE_LEN
    mov     TEMP_REG0, R27.w0 ; load temp_re0 with update tick period
    mov     CH7_BUF.w0, TEMP_REG0.w0
;Update the Hash table with ch7 temp tickperiod placeholder(R0), store in hash table
	ldi 	TEMP_REG, 12
    m_multiply TEMP_REG, TEMP_REG0
;update the lut using R4 to R20 registers(16 reg)
    xout	PRU_SPAD_B1_XID, &R4, 64
    mov     R4, R26
    m_update_lut TEMP_REG0
;Store lookup table in dmem at CH7_LUT_BASE
    ldi     TEMP_REG, CH7_LUT_BASE
    sbbo    &R4, TEMP_REG, 0, 64
    xin 	PRU_SPAD_B1_XID, &R4, 64
;Update Ch7 state register for status & Comm data calc
  	ldi    CH7_STATE, $CODE(CH7_STATUS_COMM)
  	jmp     return_addr1

CH7_DATA0:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH7_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in R22 of SPAD2 for sending it to r5f
    ldi   R0.b0, 29
    xout  PRU_SPAD_B2_XID, &CH7_BUF, 4
  	ldi   CH7_STATE, $CODE(CH7_DATA1)
    ldi   CH7_CRC4_RES, 5
    lsr   CH7_CRC4_RES, CH7_CRC4_RES, 4
    add   CH7_CRC4_RES, CH7_CRC4_RES, CH7_BUF.b3
    lbbo  &CH7_CRC4_RES, TEMP_REG2 , CH7_CRC4_RES, 1
    jmp   return_addr1
CH7_DATA2:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH7_BUF.b1, *BNS_ARG_RETVAL_ADDR
  	ldi   CH7_STATE, $CODE(CH7_DATA3)
    lsr   CH7_CRC4_RES, CH7_CRC4_RES, 4
    add   CH7_CRC4_RES, CH7_CRC4_RES, CH7_BUF.b1
    lbbo  &CH7_CRC4_RES, TEMP_REG2 , CH7_CRC4_RES, 1
    jmp   return_addr1
CH7_DATA4:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH7_BUF.b3, *BNS_ARG_RETVAL_ADDR
;Place data in R23 of SPAD2 for buffering
    ldi   R0.b0, 0
    xout  PRU_SPAD_B2_XID, &CH7_BUF, 4
  	ldi   CH7_STATE, $CODE(CH7_DATA5)
    lsr   CH7_CRC4_RES, CH7_CRC4_RES, 4
    add   CH7_CRC4_RES, CH7_CRC4_RES, CH7_BUF.b3
    lbbo  &CH7_CRC4_RES, TEMP_REG2 , CH7_CRC4_RES, 1
    jmp   return_addr1
CH7_CRC:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH7_BUF.b1, *BNS_ARG_RETVAL_ADDR
    lsr   CH7_CRC4_RES, CH7_CRC4_RES, 4
    lbbo  &CH7_CRC4_RES, TEMP_REG2 , CH7_CRC4_RES, 1
    mov   CH7_BUF.b2, CH7_CRC4_RES
    mov   CH7_BUF.b3, BNS_ARG_STATUS
;Place data in R24 of SPAD2 for buffering
    ldi   R0.b0, 1
    xout  PRU_SPAD_B2_XID, &CH7_BUF, 4
    qbne  CH7_CRC_ERROR, CH7_CRC4_RES, 0
CH7_DATA_OUT:
;Put data in DMEM or Collect data for different mode
    ldi   R0, 0
;Store existing data first
    xout  PRU_SPAD_B1_XID, &R22, 12
    xin   PRU_SPAD_B2_XID, &R22, 12
;Write 10 byte of data in SMEM(Sync, Data and CRC)
    sbco  &R22, C28, CH7_DATA_BASE, 12
;Bring back stored data
    zero  &R22, 12
    xout  PRU_SPAD_B2_XID, &R22, 12
    xin   PRU_SPAD_B1_XID, &R22, 12
;Trigger Interrupt to R5F/*TODO*/
    ldi   R0, 1
    sbco  &R0, C28, CH7_INTR_BYTE, 1
;/*TO_DO: Add error handling mechnaism for each label below*/
CH7_CRC_ERROR:
ch7_data_error:
ERROR_CH7:
;Send data to r5f and reset channel state
    ldi    CH7_STATE, $CODE(CH7_SYNC)
    jmp    return_addr1


CH7_STATUS_COMM:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH7_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH7_STATE, $CODE(CH7_DATA0)
    jmp   return_addr1
CH7_DATA1:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH7_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH7_STATE, $CODE(CH7_DATA2)
    lsr   CH7_CRC4_RES, CH7_CRC4_RES, 4
    add   CH7_CRC4_RES, CH7_CRC4_RES, CH7_BUF.b0
    lbbo  &CH7_CRC4_RES, TEMP_REG2 , CH7_CRC4_RES, 1

    jmp   return_addr1
CH7_DATA3:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH7_BUF.b2, *BNS_ARG_RETVAL_ADDR
    ldi   CH7_STATE, $CODE(CH7_DATA4)
    lsr   CH7_CRC4_RES, CH7_CRC4_RES, 4
    add   CH7_CRC4_RES, CH7_CRC4_RES, CH7_BUF.b2
    lbbo  &CH7_CRC4_RES, TEMP_REG2 , CH7_CRC4_RES, 1
    jmp   return_addr1
CH7_DATA5:
    jal   return_addr2, FN_BINARY_SEARCH
    mvib  CH7_BUF.b0, *BNS_ARG_RETVAL_ADDR
    ldi   CH7_STATE, $CODE(CH7_CRC)
    lsr   CH7_CRC4_RES, CH7_CRC4_RES, 4
    add   CH7_CRC4_RES, CH7_CRC4_RES, CH7_BUF.b0
    lbbo  &CH7_CRC4_RES, TEMP_REG2 , CH7_CRC4_RES, 1
    jmp   return_addr1

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
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 7
check_sign7?:
    qbbs    check_lower_limit7?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit7?: ;if +ve
    qblt    bn_search_mid3?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit7?:
    qbgt    bn_search_mid11?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid5?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 20, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 5
check_sign5?:
    qbbs    check_lower_limit5?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit5?: ;if +ve
    qblt    bn_search_mid4?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit5?:
    qbgt    bn_search_mid6?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid4?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 16, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 4
check_sign4?:
    qbbs    check_lower_limit4?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit4?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit4?:
    qbgt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid6?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 24, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 6
check_sign6?:
    qbbs    check_lower_limit6?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit6?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit6?:
    qbgt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid3?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 12, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 3
check_sign3?:
    qbbs    check_lower_limit3?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit3?: ;if +ve
    qblt    bn_search_mid1?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit3?:
    qbgt    bn_search_mid5?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid1?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 4, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 1
check_sign1?:
    qbbs    check_lower_limit1?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit1?: ;if +ve
    qblt    bn_search_mid0?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit1?:
    qbgt    bn_search_mid2?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid0?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 0, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 0
check_sign0?:
    qbbs    check_lower_limit0?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit0?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit0?:
    qbgt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid2?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 8, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 2
check_sign2?:
    qbbs    check_lower_limit2?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit2?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit2?:
    qbgt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid11?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 44, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 11
check_sign11?:
    qbbs    check_lower_limit11?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit11?: ;if +ve
    qblt    bn_search_mid9?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit11?:
    qbgt    bn_search_mid13?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid9?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 36, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 9
check_sign9?:
    qbbs    check_lower_limit9?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit9?: ;if +ve
    qblt    bn_search_mid8?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit9?:
    qbgt    bn_search_mid10?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid8?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 32, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 8
check_sign8?:
    qbbs    check_lower_limit8?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit8?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit8?:
    qbgt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid10?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 40, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 10
check_sign10?:
    qbbs    check_lower_limit10?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit10?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit10?:
    qbgt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid13?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 52, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 13
check_sign13?:
    qbbs    check_lower_limit13?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit13?: ;if +ve
    qblt    bn_search_mid12?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit13?:
    qbgt    bn_search_mid14?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid12?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 48, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 12
check_sign12?:
    qbbs    check_lower_limit12?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit12?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit12?:
    qbgt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid14?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 56, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 14
check_sign14?:
    qbbs    check_lower_limit14?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit14?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit14?:
    qbgt    bn_search_mid15?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done

bn_search_mid15?:
    lbbo    &BNS_LUT_VAL, BNS_ARG_LUTBASE, 60, 4
	sub     TEMP_REG, BNS_LUT_VAL, BNS_ARG_VAL
    ldi     BNS_ARG_RETVAL, 15
check_sign15?:
    qbbs    check_lower_limit15?, TEMP_REG, 31  ;check if diff  is -ve / +ve
check_upper_limit15?: ;if +ve
    qblt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MIN ; if diff<256
    qba     bn_search_done?   ;diff(<+256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
check_lower_limit15?:
    qbgt    bn_search_notfound?, TEMP_REG, APPROX_VAL_MAX    ; sign already checked so TEMP_REG is positive jmp to check for value
    qba     bn_search_done? ;diff>(-256), so correct value is already in BNS_ARG_RETVAL, so bn_search_done
bn_search_notfound?:
    ldi     BNS_ARG_RETVAL, 0xFF
    ldi     BNS_ARG_STATUS, 0xFF
	jmp     BNS_CH_DATA_ERROR
bn_search_done?:
    ldi     BNS_ARG_STATUS, 0x0
    jmp     return_addr2



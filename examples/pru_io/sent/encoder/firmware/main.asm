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

; CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

    .global     main
    .sect       ".text"

;************************************* includes *************************************
; icss_constant_defines.inc: Defines symbols corresponding to Constant Table Entries
    .include "icss_constant_defines.inc"
    .include "defines.inc"

;************************************************************************************
;Modify this to generate sent frames with different tick period
; Ex for 500ns tick period set it to 5
TICK_PRD_100ns .set  5 ;Hard coding Tick period for now
;************************************************************************************

;----------------------------------------------------------------------------
;   /* Call Main delay loop(delay value)*/
;	while(delay_value-1)
;	{
;	    /*call nop_delay_loop()*/
;		sleep(199us)
;   }
;
;----------------------------------------------------------------------------
main_delay_100ns .macro     delay_value
    ldi32   TEMP_REG, delay_value
    loop    EndNopDelayLoop_init?, 14
    nop
EndNopDelayLoop_init?:
    sub     TEMP_REG, TEMP_REG, 1
nop_delay_loop?:
    loop    EndNopDelayLoop?, 19
    nop
EndNopDelayLoop?:
    sub     TEMP_REG, TEMP_REG, 1
    qbne    nop_delay_loop?, TEMP_REG, 0
    .endm

n_clock_tick .macro n
	and COUNTER, n, n
create_clock_ticks?:
	main_delay_100ns TICK_PRD_100ns
	sub COUNTER, COUNTER, 1
	qbne create_clock_ticks?, COUNTER, 0
	.endm

five_tick_pull_down .macro
    ;Set R30 pin low
    ldi r30, 0x000
    ldi TEMP_REG2, 5
    n_clock_tick  TEMP_REG2
	.endm ;five_tick_pull_down

create_high_pulse .macro TICKS
    ;Set R30 pin high(1-6 and 8th GPIO pins high as 7th pin is not routed out)
    ldi r30, r30_value
    n_clock_tick    TICKS
	.endm ;create high pulse

create_pulse .macro	PULSE_PRD_REG
    ; Before every pulse 5 tick low period preceeds
    five_tick_pull_down
    ; High period = set pin high
    ;PULSE_PRD is (total pulse length - 5 tick period)
    create_high_pulse	PULSE_PRD_REG
    .endm ;create_pulse

;********
;* MAIN *
;********

main:

init:
;----------------------------------------------------------------------------
;   Clear the register space
;   Before begining with the application, make sure all the registers are set
;   to 0. PRU has 32 - 4 byte registers: R0 to R31, with R30 and R31 being special
;   registers for output and input respectively.
;----------------------------------------------------------------------------

; Give the starting address and number of bytes to clear.
    zero	&r0, 120

;----------------------------------------------------------------------------
;   START ENCODER LOGIC
; 	PRU clk period = 5ns
; 	200 cycles = 1us
;
;-----------------------------------------------------------------------
;----------------------------------------------------------------------------
    ldi    TEMP_REG1, PDMEM00
loop_process:
;Extract number of frames to transmit from DMEM
    lbbo  &NUM_FRAMES, TEMP_REG1 , 0, 4
;If number of frames is non zero then start tx
    qbeq  loop_process, NUM_FRAMES, 0
;Maintain total frame tx count
    add   TOTAL_FRAMES, TOTAL_FRAMES, NUM_FRAMES
Create_sent_pulse:
    ldi NIBBLE_VALUE, SYNC_PULSE_PERIOD
    create_pulse     NIBBLE_VALUE ;51+5 =56 Sync
    lbbo    &NIBBLE_VALUE, TEMP_REG1 , SC_NIBBLE_OFFSET, 1
    add     NIBBLE_VALUE, NIBBLE_VALUE, 7
    create_pulse    NIBBLE_VALUE ; Status Comm
	lbbo    &NIBBLE_VALUE, TEMP_REG1 , DATA0_OFFSET, 1
    add     NIBBLE_VALUE, NIBBLE_VALUE, 7
    create_pulse    NIBBLE_VALUE; 14+5 =19 => 7
	lbbo    &NIBBLE_VALUE, TEMP_REG1 , DATA1_OFFSET, 1
    add     NIBBLE_VALUE, NIBBLE_VALUE, 7
    create_pulse    NIBBLE_VALUE; 11+5 =16 => 4
	lbbo    &NIBBLE_VALUE, TEMP_REG1 , DATA2_OFFSET, 1
    add     NIBBLE_VALUE, NIBBLE_VALUE, 7
    create_pulse    NIBBLE_VALUE; 15+5 =20 => 8
	lbbo    &NIBBLE_VALUE, TEMP_REG1 , DATA3_OFFSET, 1
    add     NIBBLE_VALUE, NIBBLE_VALUE, 7
    create_pulse    NIBBLE_VALUE; 14+5 =19 => 7
	lbbo    &NIBBLE_VALUE, TEMP_REG1 , DATA4_OFFSET, 1
    add     NIBBLE_VALUE, NIBBLE_VALUE, 7
    create_pulse    NIBBLE_VALUE; 11+5 =16 => 4
	lbbo    &NIBBLE_VALUE, TEMP_REG1 , DATA5_OFFSET, 1
    add     NIBBLE_VALUE, NIBBLE_VALUE, 7
    create_pulse    NIBBLE_VALUE; 15+5 =20 => 8
	lbbo    &NIBBLE_VALUE, TEMP_REG1 , CRC_OFFSET, 1
    add     NIBBLE_VALUE, NIBBLE_VALUE, 7
    create_pulse    NIBBLE_VALUE; 10+5 =15 =>3 CRC
    ldi NIBBLE_VALUE, 100
    create_pulse    NIBBLE_VALUE; Pause Pulse
next_pulse:
	sub NUM_FRAMES, NUM_FRAMES, 1
	qbne Create_sent_pulse, NUM_FRAMES, 0
;Clear the value after reading
    ldi TEMP_REG2, 0
    sbbo  &TEMP_REG2, TEMP_REG1 , 0, 4
;signal R5F once Tx is done
    sbbo  &NUM_FRAMES, TEMP_REG1 , 4, 4
    qba loop_process

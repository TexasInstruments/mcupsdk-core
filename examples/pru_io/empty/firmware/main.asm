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
;   File:     main.asm
;
;   Brief:    Template asm file example
;************************************************************************************

; CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

    .global     main
    .sect       ".text"

;************************************* includes *************************************
; icss_constant_defines.inc: Defines symbols corresponding to Constant Table Entries
    .include "icss_constant_defines.inc"

    .asg	R2,	   TEMP_REG
    .asg    R3.w0, OFFSET_ADDR

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
;   Constant Table Entries Configuration
;   Sample code to configure Constant Table Entries.
;----------------------------------------------------------------------------

; Configure the Constant Table entry C28 to point to start of shared memory
; PRU_ICSSG Shared RAM (local-C28) : 00nn_nn00h, nnnn = c28_pointer[15:0]
; By default it is set to 0000_0000h so it will point to DMEM0 address
    ldi     TEMP_REG, 0x0100
    sbco    &TEMP_REG, ICSS_PRU_CTRL_CONST, 0x28, 2

; Load 32 bit value to TEMP_REG reg
; To load values greater than 16 bits, we use ldi32 which performs ldi 2 times
; - for lsb 16 bits and then for msb 16 bits
	ldi32	TEMP_REG, 0x12345678

    .if	$isdefed("PRU0")
    ldi     OFFSET_ADDR, 0 
    .elseif	$isdefed("PRU1")
    ldi     OFFSET_ADDR, 4
    .elseif	$isdefed("RTU_PRU0")
    ldi     OFFSET_ADDR, 8 
    .elseif	$isdefed("RTU_PRU1")
    ldi     OFFSET_ADDR, 12 
    .elseif	$isdefed("TX_PRU0")
    ldi     OFFSET_ADDR, 16
    .elseif	$isdefed("TX_PRU1")
    ldi     OFFSET_ADDR, 20  
    .endif

;----------------------------------------------------------------------------
;   Writing to PRU memories
;   Sample code to write to DMEM and SMEM.
;----------------------------------------------------------------------------

; Write 4 byte register value to DMEM0 at OFFSET_ADDR
    sbco    &TEMP_REG, ICSS_DMEM0_CONST, OFFSET_ADDR, 4

; Write 3 byte register value to DMEM1 at OFFSET_ADDR
    sbco    &TEMP_REG, ICSS_DMEM1_CONST, OFFSET_ADDR, 3

; Write 2 byte register value to SMEM0 at OFFSET_ADDR
    sbco    &TEMP_REG, ICSS_SMEM_CONST,  OFFSET_ADDR, 2

    halt ; end of program

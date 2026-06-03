; Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/
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

    .asg    R0,     BEAT0
    .asg    R1,     BEAT1
    .asg    R2,     BEAT2
    .asg    R3,     BEAT3
    .asg    R4,     ECC_ERR_STAT1_ERROR_STAT
    .asg    R5,     ECC_ERR_STAT2_ERROR_ADD ;STAT2 contains address
    .asg    R6,     TEMP_REG1
    .asg    R7,     TEMP_REG2
    .asg    R8.b0,  RAM_ID
    .asg    R8.b1,  TEMP_BYTE
    .asg    R28,    ADDRESS_TO_READ
    .asg    R29,    NUM_OF_BURSTS

;********
;* MAIN *
;********
;set size of burst to scrub
;4 beats and each beat is of 4 bytes (1 register)
BYTES_TO_READ   .set  4*4
;DMEM offsets
ADDRESS_TO_READ_OFFSET          .set    0
NUMBER_OF_BURSTS_OFFSET     	.set    4
PRU_ECC_STATUS_READ_OFFSET		.set    8
PRU_TRIGGER_OFFSET              .set    9

;NOTE : only word 0 will be used for loop counter
;so maximum bytes that can be read is 0x73FF8C with 116 byte burst read (0x73FF8C/116 = 0xFFFF)

m_read_ecc_agg_add        .macro        temp_reg1, temp_reg2, ecc_agg_reg_addr, ecc_agg_vec_config_val
    LDI32    temp_reg1, 0x53010008
    LDI32    temp_reg2, ecc_agg_vec_config_val
    ADD      temp_reg2, temp_reg2, RAM_ID
    sbbo    &temp_reg2, temp_reg1, 0, 4
wait?:
    lbbo    &temp_reg2, temp_reg1, 0, 4
    qbbc     wait?, temp_reg2, 24
    ;read the ecc_agg_reg_add
    lbbo    &temp_reg1, ecc_agg_reg_addr, 0, 4
    .endm

main:
;----------------------------------------------------------------------------
;   Clear the register space
;   Before begining with the application, make sure all the registers are set
;   to 0. PRU has 32 - 4 byte registers: R0 to R31, with R30 and R31 being special
;   registers for output and input respectively.
;----------------------------------------------------------------------------
; Give the starting address and number of bytes to clear.
    ZERO    &r0, 120
    LDI32    ECC_ERR_STAT1_ERROR_STAT, 0x53010040
    LDI32   ECC_ERR_STAT2_ERROR_ADD,  0x53010024

init:
    ;load number of loop bursts from burst count offset
    LBCO    &NUM_OF_BURSTS, ICSS_DMEM0_CONST, NUMBER_OF_BURSTS_OFFSET, 4
    QBEQ    init, NUM_OF_BURSTS, 0
    ;load address to read from address offset
    LBCO    &ADDRESS_TO_READ, ICSS_DMEM0_CONST, ADDRESS_TO_READ_OFFSET, 4
next_address:
    ;trigger R5F by writing 1 at PRU_ECC_STATUS_READ_OFFSET
    LDI     TEMP_BYTE, 1
    SBCO    &TEMP_BYTE, ICSS_DMEM0_CONST, PRU_ECC_STATUS_READ_OFFSET, 1
    ;read   the data
    LBBO    &BEAT0, ADDRESS_TO_READ, 0, BYTES_TO_READ
    ;initialize RAM_ID
    LSR      RAM_ID, ADDRESS_TO_READ.b2, 3
	.if	$isdefed("SOC_AM263PX")
    ;In AM263Px, RAM_ID for bank 4 should be 7
check_ram_id_4:
	QBNE	check_ram_id_5, RAM_ID, 4
	LDI		RAM_ID, 7
	QBA		check_sec
	;In AM263Px, RAM_ID for bank 5 should be 8
check_ram_id_5:
	QBNE	check_sec, RAM_ID, 5
	LDI		RAM_ID, 8
	.endif
check_sec:
    ;if SEC has not occured then jump to go_to_next_add
    m_read_ecc_agg_add    TEMP_REG1, TEMP_REG2, ECC_ERR_STAT1_ERROR_STAT, 0x00408000
    ;JAG: Using register causing issue so hard coded with 2 directly
    QBBC     go_to_next_beat, TEMP_REG1, RAM_ID
    ;load at which SEC_ERROR is triggered
    m_read_ecc_agg_add    TEMP_REG1, TEMP_REG2, ECC_ERR_STAT2_ERROR_ADD, 0x00248000
    LSL      TEMP_REG1, TEMP_REG1, 3
    LDI      TEMP_REG1.b3, 0x70
    MOV      TEMP_REG1.b2, ADDRESS_TO_READ.b2
    ;check if SEC has occurred at ADDRESS_TO_READ (BEAT0)
    QBEQ     trigger_r5f_not_to_reset, TEMP_REG1, ADDRESS_TO_READ
    ;check if SEC has occurred at ADDRESS_TO_READ + 4 (BEAT1)
check_at_beat_1:
    ADD      TEMP_REG2, ADDRESS_TO_READ, 4
  	QBEQ     trigger_r5f_not_to_reset, TEMP_REG1, TEMP_REG2
    ;check if SEC has occurred at ADDRESS_TO_READ + 4 (BEAT2)
check_at_beat_2:
    ADD      TEMP_REG2, TEMP_REG2, 4
  	QBEQ     trigger_r5f_not_to_reset, TEMP_REG1, TEMP_REG2
    ;check if SEC has occurred at ADDRESS_TO_READ + 4 (BEAT3)
check_at_beat_3:
    ADD      TEMP_REG2, TEMP_REG2, 4
  	QBEQ     trigger_r5f_not_to_reset, TEMP_REG1, TEMP_REG2
  	QBA	     go_to_next_beat
trigger_r5f_not_to_reset:
    ;trigger R5F by writing 1 at PRU_TRIGGER_OFFSET
    LDI     TEMP_BYTE, 1
    SBCO    &TEMP_BYTE, ICSS_DMEM0_CONST, PRU_TRIGGER_OFFSET, 1
    ;wait until R5F acknowledges and clear at PRU PRU_TRIGGER_OFFSET
wait:
    LBCO    &TEMP_BYTE, ICSS_DMEM0_CONST, PRU_TRIGGER_OFFSET, 1
    QBEQ	wait, TEMP_BYTE, 1
    ;go to next beat
go_to_next_beat:
	;trigger R5F by writing 0 at PRU_ECC_STATUS_READ_OFFSET
    LDI     TEMP_BYTE, 0
    SBCO    &TEMP_BYTE, ICSS_DMEM0_CONST, PRU_ECC_STATUS_READ_OFFSET, 1
    ADD     ADDRESS_TO_READ, ADDRESS_TO_READ, BYTES_TO_READ
    SUB     NUM_OF_BURSTS, NUM_OF_BURSTS, 1
    QBNE    next_address, NUM_OF_BURSTS, 0
end_loop:
    QBA        init

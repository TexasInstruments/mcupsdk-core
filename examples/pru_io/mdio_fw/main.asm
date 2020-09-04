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
;             Copies of this file are present in EtherCat, EtherNet and Profinet sources.
;             Make usre to make changes at all locations
;************************************************************************************

; CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

    .global     main
    .sect       ".text"

;************************************* includes *************************************
    .include "mdio_macros.inc"

    .asg	R1,	userAccessReg
    .asg	R2,	tempReg
    .asg	R4,	phyAddr		; 0-31 - 8 bits only
    .asg	R5,	regAddr		; 0-31 - 8 bits only
    .asg	R6,	mdio_manual_if_reg ; 8 bits only
    .asg	R7.b0,	bitNum  ; 8 bits only
    .asg	R8,	value
    .asg	R29.w0,	RET_WRITE
    .asg	R29.w2,	RET_READ

    .asg	R11, ACTUAL_MDIO_BASE_ADDR          ; 0x30032400 - can use Constant Table entry
    .asg	R12, DEFINED_MDIO_BASE_ADDR

    .asg	0x00032400, ACTUAL_MDIO_BASE_ADDR_CONST
    .asg	0x00010E40, DEFINED_MDIO_BASE_ADDR_CONST

;********
;* MAIN *
;********

main:

init:
    zero	&r0, 40                                ; Clear the register space

    ldi32 	ACTUAL_MDIO_BASE_ADDR, ACTUAL_MDIO_BASE_ADDR_CONST    ; 0x300B2400
    ; Will be passed by the R5F application
    ; ldi32 	DEFINED_MDIO_BASE_ADDR, DEFINED_MDIO_BASE_ADDR_CONST
    m_mdio_init_manual_mode  ACTUAL_MDIO_BASE_ADDR, tempReg

    ; Loop performing CLAUSE22 read/write operations according to User Access Registers
MDIO_MAIN_LOOP:
    ; Update MDIO_USER_PHY_SEL_0/1_REG at actual MMRs
    lbbo            &tempReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_PHY_SEL_0_REG_OFFSET, 4
    sbbo            &tempReg, ACTUAL_MDIO_BASE_ADDR,  MDIO_USER_PHY_SEL_0_REG_OFFSET, 4
    lbbo            &tempReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_PHY_SEL_1_REG_OFFSET, 4
    sbbo            &tempReg, ACTUAL_MDIO_BASE_ADDR,  MDIO_USER_PHY_SEL_1_REG_OFFSET, 4

    ; Update MDIO_LINK_REG at new defined location
    lbbo            &tempReg, ACTUAL_MDIO_BASE_ADDR,  MDIO_LINK_REG_OFFSET, 4
    sbbo            &tempReg, DEFINED_MDIO_BASE_ADDR, MDIO_LINK_REG_OFFSET, 4

    ;---------------------------------------------------------------------------------------
    ; Reads and checks 1st User Access Reg if we need to perform PHY register read/write
    ; operation. performs when the GO bit is high.
    ;---------------------------------------------------------------------------------------
    ; Load 1st User Access Reg
    lbbo            &userAccessReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_ACCESS_0_REG_OFFSET, 4
    ; Check GO bit, skip if it is clear
    qbbc            END_USERACCESS1_OPERATION?, userAccessReg, MDIO_USER_ACCESS_REG_GO_BIT

    ; Extract values from register
    and             phyAddr, userAccessReg.b2, MASK_5_BITS
    lsr             regAddr, userAccessReg.w2, 5
    and             regAddr, regAddr, MASK_5_BITS
    ; Check WRITE bit to determine for Write or Read operation
    qbbc            PERFORM_READ1?, userAccessReg, MDIO_USER_ACCESS_REG_WRITE_BIT
    mov             value, userAccessReg.w0
    jal             RET_WRITE, mdio_write
    sbbo            &userAccessReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_ACCESS_0_REG_OFFSET, 4    ; Update userAccess0Reg
    qba             END_USERACCESS1_OPERATION?

PERFORM_READ1?:
    jal             RET_READ, mdio_read
    sbbo            &userAccessReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_ACCESS_0_REG_OFFSET, 4    ; Update userAccess0Reg

END_USERACCESS1_OPERATION?:

    ;---------------------------------------------------------------------------------------
    ; Reads and checks 2nd User Access Reg if we need to perform PHY register read/write
    ; operation. performs when the GO bit is high.
    ;---------------------------------------------------------------------------------------
    ; Load 2nd User Access Reg
    lbbo            &userAccessReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_ACCESS_1_REG_OFFSET, 4
    ; Check GO bit, skip if it is clear
    qbbc            END_USERACCESS2_OPERATION?, userAccessReg, MDIO_USER_ACCESS_REG_GO_BIT

    ; Extract values from register
    and             phyAddr, userAccessReg.b2, MASK_5_BITS
    lsr             regAddr, userAccessReg.w2, 5
    and             regAddr, regAddr, MASK_5_BITS
    ; Check WRITE bit to determine for Write or Read operation
    qbbc            PERFORM_READ2?, userAccessReg, MDIO_USER_ACCESS_REG_WRITE_BIT
    mov             value, userAccessReg.w0
    jal             RET_WRITE, mdio_write
    sbbo            &userAccessReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_ACCESS_1_REG_OFFSET, 4    ; Update userAccess1Reg
    qba             END_USERACCESS2_OPERATION?

PERFORM_READ2?:
    jal             RET_READ, mdio_read
    sbbo            &userAccessReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_ACCESS_1_REG_OFFSET, 4    ; Update userAccess1Reg

END_USERACCESS2_OPERATION?:
    qba             MDIO_MAIN_LOOP

mdio_read:
    m_mdio_read     "CLAUSE22", ACTUAL_MDIO_BASE_ADDR, phyAddr, regAddr, userAccessReg, bitNum, value
    clr             userAccessReg, userAccessReg, MDIO_USER_ACCESS_REG_GO_BIT                   ; Reset GO Bit
    jmp             RET_READ

mdio_write:
    m_mdio_write    "CLAUSE22", ACTUAL_MDIO_BASE_ADDR, phyAddr, regAddr, tempReg, bitNum, value
    clr             userAccessReg, userAccessReg, MDIO_USER_ACCESS_REG_GO_BIT                   ; Reset GO Bit
    clr             userAccessReg, userAccessReg, MDIO_USER_ACCESS_REG_WRITE_BIT                ; Reset WRITE Bit - required?
    jmp             RET_WRITE

    halt            ; end of program

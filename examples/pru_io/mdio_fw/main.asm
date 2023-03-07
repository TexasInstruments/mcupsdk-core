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
;   Brief:    MDIO Workaround implementation file
;************************************************************************************

; CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

    .global     main
    .sect       ".text"

;************************************* includes *************************************
    .include "mdio_macros.inc"

    .asg	R1,	    userAccessReg
    .asg	R2,	    tempReg
    .asg	R4,	    phyAddr		; values range: 0-31 | required 8 bits only
    .asg	R5,	    regAddr		; values range: 0-31 | required 8 bits only
    .asg	R6,	    mdio_manual_if_reg ; 8 bits only
    .asg	R7.b0,	bitNum      ; values range: 0-31 | required 8 bits only
    .asg	R8,	    phyRegValue

; ------------------------------------------------------------------------------
; |  R10 register's bits will be used to configure the firmware working modes  |
; ------------------------------------------------------------------------------
; | Bit    | Description                                                       |
; | R10.t0 | Link detection method                                             |
; |        |    0 : MLINK based                                                |
; |        |    1 : Polling based                                              |
; | R10.t1 | Link status change interrupt for polling mode                     |
; |        |    0 : disabled                                                   |
; |        |    1 : enabled                                                    |
; | R10.t2 | Configure FW acc to PRU CLK = 250                                 |
; |        |    0 : disabled                                                   |
; |        |    1 : enabled                                                    |
; | R10.t3 | Configure FW acc to PRU CLK = 333                                 |
; |        |    0 : disabled                                                   |
; |        |    1 : enabled                                                    |
; ------------------------------------------------------------------------------
    .asg	R10,    FW_CONFIG_REG

    .asg	R11,    ACTUAL_MDIO_BASE_ADDR
    .asg	R12,    DEFINED_MDIO_BASE_ADDR

    .asg	R29.w0,	RET_PHY_ACCESS
    .asg	R29.w2,	RET_LINK_STATUS_UPDATE
    .asg	R29.w0,	RET_WRITE
    .asg	R29.w2,	RET_READ

    .asg	R13,    mdioLinkReg
    .asg	R14.w0, counter_for_link_status_update_reg
    .asg	R14.w2, value_to_update_link_status_on_reg
    .asg	R15.w0, value_to_inc_on_phy_access_reg
    .asg	R15.w2, miiEventNumReg
    .asg	R16.b0, userPhySel_phyAddr0
    .asg	R16.b1, userPhySel_phyAddr1
    .asg	R19,    MDIO_CLK_PERIOD_CONFIG_REG  ; This reg is used by the include file: mdio_macros.inc
    .asg	R20,    mdioAliveReg

    .asg	0x00032400, ACTUAL_MDIO_BASE_ADDR_CONST
    ; .asg	0x00010E40, DEFINED_MDIO_BASE_ADDR_CONST

; For polling PHY status bit:
; Will update link status on count value acc to PRU CLK speed

    .asg	2100,       VALUE_TO_UPDATE_LINK_ON_CONST_200
    .asg	120,        PHY_ACCESS_COUNT_VALUE_CONST_200
    .asg	2650,       VALUE_TO_UPDATE_LINK_ON_CONST_250
    .asg	115,        PHY_ACCESS_COUNT_VALUE_CONST_250
    .asg	3500,       VALUE_TO_UPDATE_LINK_ON_CONST_333
    .asg	105,        PHY_ACCESS_COUNT_VALUE_CONST_333

; MDIO CLK Freq: Currently 2.5MHz
; MDIO CLK Freq is about = (PRU_CLK/(2*MDIO_CLK_PERIOD_CONFIG+12))
; Need to subtract 1 as delay loop takes 1 extra cycle than specified
MDIO_CLK_PERIOD_CONFIG_200          .set    34-1
MDIO_CLK_PERIOD_CONFIG_250          .set    44-1
MDIO_CLK_PERIOD_CONFIG_333          .set    61-1

; Create USER_ACCESS_REG_LINK_STATUS_CONST template for getting link status
; Set GO bit, clear WRITE bit, REGADR = 1 (PHY BMSR Reg)
    .asg	0x8020,     USER_ACCESS_REG_LINK_STATUS_CONST

MASK_5_BITS                         .set    0x1F
CLEAR_5_BITS                        .set    0x00

; Improvements:
; * Link status change interrupt (2 different intrs)
;   * Polling mode  - interrupt using ICSS_INTC_STATUS_SET_INDEX_REG
;   * MLINK mode    - automatic interrupt generation
; * Alive register emulation - around every 250us
; * Link register emulation  - around every 250us (for Link Polling mode )

;********
;* MAIN *
;********

main:

; --------------------------------------- FW Initialization ----------------------------------------
; Initialization for PRU firmware
; --------------------------------------------------------------------------------------------------
init:
    zero	        &r0, 40                         ; Clear initial 10 registers: R0-R9

    ldi32 	        ACTUAL_MDIO_BASE_ADDR, ACTUAL_MDIO_BASE_ADDR_CONST
    ; Above resolves to 0x30032400 or 0x300B2400 depending on the ICSSG instance executing the instruction

    ; Following DEFINED_MDIO_BASE_ADDR will be set by the R5F application so commented out
    ; ldi32 	DEFINED_MDIO_BASE_ADDR, DEFINED_MDIO_BASE_ADDR_CONST

    ; Reset following registers
    ldi             mdioLinkReg, 0
    ldi             mdioAliveReg, 0

    ; Configure the MDIO FW parameters according to PRU Clock Frequency
    ldi             MDIO_CLK_PERIOD_CONFIG_REG, MDIO_CLK_PERIOD_CONFIG_250  ; Update MDIO_CLK_PERIOD_CONFIG_REG if PRU CLK = 250
    ldi             counter_for_link_status_update_reg, VALUE_TO_UPDATE_LINK_ON_CONST_250
    ldi             value_to_update_link_status_on_reg, VALUE_TO_UPDATE_LINK_ON_CONST_250
    ldi             value_to_inc_on_phy_access_reg, PHY_ACCESS_COUNT_VALUE_CONST_250
    qbbs            END_MDIO_CLK_CONFIG, FW_CONFIG_REG, 2                   ; conditional exit
    ldi             MDIO_CLK_PERIOD_CONFIG_REG, MDIO_CLK_PERIOD_CONFIG_333  ; Update MDIO_CLK_PERIOD_CONFIG_REG if PRU CLK = 300
    ldi             counter_for_link_status_update_reg, VALUE_TO_UPDATE_LINK_ON_CONST_333
    ldi             value_to_update_link_status_on_reg, VALUE_TO_UPDATE_LINK_ON_CONST_333
    ldi             value_to_inc_on_phy_access_reg, PHY_ACCESS_COUNT_VALUE_CONST_333
    qbbs            END_MDIO_CLK_CONFIG, FW_CONFIG_REG, 3                   ; conditional exit
    ldi             MDIO_CLK_PERIOD_CONFIG_REG, MDIO_CLK_PERIOD_CONFIG_200  ; Default PRU CLK = 200
    ldi             counter_for_link_status_update_reg, VALUE_TO_UPDATE_LINK_ON_CONST_200
    ldi             value_to_update_link_status_on_reg, VALUE_TO_UPDATE_LINK_ON_CONST_200
    ldi             value_to_inc_on_phy_access_reg, PHY_ACCESS_COUNT_VALUE_CONST_200
END_MDIO_CLK_CONFIG:

; --------------------------------------- MDIO_CONFIGURATION ---------------------------------------
; Initialization of MDIO hardware
; --------------------------------------------------------------------------------------------------
    m_mdio_init_manual_mode  ACTUAL_MDIO_BASE_ADDR, tempReg


; ----------------------------------------- MDIO_MAIN_LOOP -----------------------------------------
; Loop performing CLAUSE22 read/write operations according to User Access Registers
; --------------------------------------------------------------------------------------------------
MDIO_MAIN_LOOP:

    ; Update MDIO_USER_PHY_SEL_0/1_REG at actual MMRs (required in case of MLINK based detection)
    ; Firmware continuously keeps on checking for changes in MDIO_USER_PHY_SEL_0/1_REG
    lbbo            &tempReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_PHY_SEL_0_REG_OFFSET, 4
    sbbo            &tempReg, ACTUAL_MDIO_BASE_ADDR,  MDIO_USER_PHY_SEL_0_REG_OFFSET, 4
    ; Extract PHY0 Address from userPhySelReg0
    and             userPhySel_phyAddr0, tempReg, MASK_5_BITS
    lbbo            &tempReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_PHY_SEL_1_REG_OFFSET, 4
    sbbo            &tempReg, ACTUAL_MDIO_BASE_ADDR,  MDIO_USER_PHY_SEL_1_REG_OFFSET, 4
    ; Extract PHY1 Address from userPhySelReg1
    and             userPhySel_phyAddr1, tempReg, MASK_5_BITS

;---------------------------------------------------------------------------------------
; MDIO_USER_ACCESS_0_REG emulation
; Reads and checks 1st User Access Reg if we need to perform PHY register read/write
; operation. performs when the GO bit is high.
;---------------------------------------------------------------------------------------
    ; Load 1st User Access Reg
    lbbo            &userAccessReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_ACCESS_0_REG_OFFSET, 4
    ; Check GO bit, skip PHY access if it is clear:
    qbbc            END_USERACCESS0_OPERATION?, userAccessReg, MDIO_USER_ACCESS_REG_GO_BIT
    ; Do PHY register access:
    jal             RET_PHY_ACCESS, ACCESS_PHY_REG

UPDATE_USERACCESS0_REG?:        ; Update userAccess0Reg at emulated register space
    sbbo            &userAccessReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_ACCESS_0_REG_OFFSET, 4    ; Update userAccess0Reg at emulated register space

END_USERACCESS0_OPERATION?:

;---------------------------------------------------------------------------------------
; MDIO_USER_ACCESS_1_REG emulation
; Reads and checks 2nd User Access Reg if we need to perform PHY register read/write
; operation. performs when the GO bit is high.
;---------------------------------------------------------------------------------------
    ; Load 2nd User Access Reg
    lbbo            &userAccessReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_ACCESS_1_REG_OFFSET, 4
    ; Check GO bit, skip PHY access if it is clear:
    qbbc            END_USERACCESS1_OPERATION?, userAccessReg, MDIO_USER_ACCESS_REG_GO_BIT
    ; Do PHY register access:
    jal             RET_PHY_ACCESS, ACCESS_PHY_REG

UPDATE_USERACCESS1_REG?:        ; Update userAccess1Reg at emulated register space
    sbbo            &userAccessReg, DEFINED_MDIO_BASE_ADDR, MDIO_USER_ACCESS_1_REG_OFFSET, 4

END_USERACCESS1_OPERATION?:

;---------------------------------------------------------------------------------------
; MDIO_LINK_REG & MDIO_ALIVE_REG emulation
; Polling based LINK update configuration:
;       Reads and checks 2nd bit of BMSR Register to detect link status
; MLINK based LINK update configuration:
;       Copies data from MMRS
;---------------------------------------------------------------------------------------
    ; Update counter which maintains value for link status update
    ; When counter reaches VALUE_TO_UPDATE_LINK_ON_CONST we update MDIO_LINK_REG
    add             counter_for_link_status_update_reg, counter_for_link_status_update_reg, 1
    qbgt            SKIP_LINK_UPDATE_BY_POLLING?, counter_for_link_status_update_reg, value_to_update_link_status_on_reg

    ; For PHY0 from MDIO_USER_PHY_SEL_0_REG
    mov             phyAddr, userPhySel_phyAddr0
    ldi             miiEventNumReg, 41                ; Interrupt event 41: MDIO_MII_LINK[0]
    jal             RET_LINK_STATUS_UPDATE, UPDATE_LINK_STATUS      ; Update link status
    ; jal             RET_LINK_STATUS_UPDATE, UPDATE_LINK_STATUS    ; Uncomment if we need to read 2 times

    ; For PHY1 from MDIO_USER_PHY_SEL_1_REG
    mov             phyAddr, userPhySel_phyAddr1
    ldi             miiEventNumReg, 53                ; Interrupt event 53: MDIO_MII_LINK[1]
    jal             RET_LINK_STATUS_UPDATE, UPDATE_LINK_STATUS      ; Update link status
    ; jal             RET_LINK_STATUS_UPDATE, UPDATE_LINK_STATUS    ; Uncomment if we need to read 2 times

    ; Reset counter for link status update to 0
    ldi             counter_for_link_status_update_reg, 0
    qba             SKIP_LINK_UPDATE_BY_POLLING?

SKIP_LINK_UPDATE_BY_POLLING?:

    ; Perform MLINK based LINK Update, if enabled
    qbbs            SKIP_MLINK_BASED_LINK_UPDATE?, FW_CONFIG_REG, 0
    ; Copy the MDIO_LINK_REG at new emulated register location from MMR
    lbbo            &tempReg, ACTUAL_MDIO_BASE_ADDR,  MDIO_LINK_REG_OFFSET, 4
    sbbo            &tempReg, DEFINED_MDIO_BASE_ADDR, MDIO_LINK_REG_OFFSET, 4
SKIP_MLINK_BASED_LINK_UPDATE?:

    qba             MDIO_MAIN_LOOP  ; loop
; ---------------------------------------------- ××× -----------------------------------------------


;---------------------------------------------------------------------------------------
; ACCESS_PHY_REG
; Performs PHY register read/write operation according to the userAccessReg value
;---------------------------------------------------------------------------------------
ACCESS_PHY_REG:
    ; Extract bit-fields from userAccessReg register
    and             phyAddr, userAccessReg.b2, MASK_5_BITS
    lsr             regAddr, userAccessReg.w2, 5
    and             regAddr, regAddr, MASK_5_BITS
    ; Check WRITE bit to determine if we need to do Write or Read operation
    qbbc            READ_PHY_REG?, userAccessReg, MDIO_USER_ACCESS_REG_WRITE_BIT
    mov             phyRegValue, userAccessReg.w0

    ; Perform PHY register write operation
    m_mdio_write    "CLAUSE22", ACTUAL_MDIO_BASE_ADDR, phyAddr, regAddr, tempReg, bitNum, phyRegValue
    ; clr             userAccessReg, userAccessReg, MDIO_USER_ACCESS_REG_WRITE_BIT      ; Reset WRITE Bit - required?

    qba             UPDATE_LOCAL_USER_ACCESS_REG?

READ_PHY_REG?:
    ; Perform PHY register read operation
    m_mdio_read     "CLAUSE22", ACTUAL_MDIO_BASE_ADDR, phyAddr, regAddr, userAccessReg, bitNum, phyRegValue

UPDATE_LOCAL_USER_ACCESS_REG?:
    clr             userAccessReg, userAccessReg, MDIO_USER_ACCESS_REG_GO_BIT           ; Reset GO Bit

    ; Increment the counter for link status update
    add             counter_for_link_status_update_reg, counter_for_link_status_update_reg, value_to_inc_on_phy_access_reg

    jmp             RET_PHY_ACCESS  ; return

;---------------------------------------------------------------------------------------------------
; UPDATE_LINK_STATUS
; Triggers PHY status register read operation to get LINK status
; --------------------------------------------------------------------------------------------------
; DP83869HM:
;   BMSR Register (Address = 0x1) [reset = 0x7949]
; --------------------------------------------------------------------------------------------------
; Bit | Field       | Type  | Reset | Description
; --------------------------------------------------------------------------------------------------
;  2  | LINK_STS1   |  R    |  0x0  | Link Status
;                                   | This is latch low and needs to be read twice for valid link up
;                                   |     0x0 = Link down
;                                   |     0x1 = Link up
; --------------------------------------------------------------------------------------------------
UPDATE_LINK_STATUS:
    ; Format User Access Reg for reading BMSR Register:
    ldi             userAccessReg.w2, USER_ACCESS_REG_LINK_STATUS_CONST
    or              userAccessReg.b2, userAccessReg.b2, phyAddr

    jal             RET_PHY_ACCESS, ACCESS_PHY_REG  ; Read PHY BMSR Register

    ; First reset bits corresponding to PHYs other then specified in PHY_SEL_REGs
    ldi             tempReg, 0
    set             tempReg, tempReg, userPhySel_phyAddr0
    set             tempReg, tempReg, userPhySel_phyAddr1
    and             mdioAliveReg, mdioAliveReg, tempReg
    and             mdioLinkReg, mdioLinkReg, tempReg

    ; --------------------- Update MDIO Alive Reg --------------------------- ;
    clr             mdioAliveReg, mdioAliveReg, phyAddr
    qbbc            PHY_NOT_ALIVE?, userAccessReg, 29   ; Check MDIO_USER_ACCESS_REG_ACK_BIT=29 for PHY alive status
    ; If PHY is not alive then we cannot determine the link status also (as there is no response from PHY)
    set             mdioAliveReg, mdioAliveReg, phyAddr
    ; ----------------------------------------------------------------------- ;

    ; ---------------- Update MDIO link Reg & Interrupt --------------------- ;
    mov             tempReg, mdioLinkReg                ; Store previous Link Reg Value
    ; Update status of PHY link in Link Reg:
    clr             mdioLinkReg, mdioLinkReg, phyAddr
    qbbc            PHY_LINK_DOWN?, userAccessReg, 2    ; BMSR_REG_LINK_STS1_BIT
    set             mdioLinkReg, mdioLinkReg, phyAddr
PHY_LINK_DOWN?:

    ; Update MDIO_LINK_REG at emulated register space offset
    sbbo            &mdioLinkReg, DEFINED_MDIO_BASE_ADDR, MDIO_LINK_REG_OFFSET, 4

    ; Skip updating MDIO_LINK_REG if we are using MLINK based link detection
    qbbc            SKIP_LINK_UPDATE_USING_POLLING?, FW_CONFIG_REG, 0

    ; Interrupt on link status change
    ; No interrupt if link status is unchanged (Check linkReg value with previous value)
    qbeq            SKIP_LINK_CHANGE_INTERRUPT?, mdioLinkReg, tempReg

    ; Create interrupt, if enabled (Only in case of Polling based LINK update)
    qbbc            SKIP_LINK_CHANGE_INTERRUPT?, FW_CONFIG_REG, 1
    sbco            &miiEventNumReg, C0, 0x20, 2    ; C0: PRU_ICSSG INTC_CONST, 0x20: ICSS_INTC_STATUS_SET_INDEX_REG_OFFSET
SKIP_LINK_CHANGE_INTERRUPT?:
SKIP_LINK_UPDATE_USING_POLLING?:
    ; ----------------------------------------------------------------------- ;

PHY_NOT_ALIVE?:
    ; Update MDIO_ALIVE_REG at emulated register space offset
    sbbo            &mdioAliveReg, DEFINED_MDIO_BASE_ADDR, MDIO_ALIVE_REG_OFFSET, 4

    jmp             RET_LINK_STATUS_UPDATE  ; return


    halt            ; end of program - program will not reach here

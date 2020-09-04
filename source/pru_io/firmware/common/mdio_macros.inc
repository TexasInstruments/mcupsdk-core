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
;   File:     mdio_macros.inc
;
;   Brief:    MDIO module - manual control macros
;
;   Includes:
;************************************************************************************
    .if    !$defined("__mdio_macros_inc")
__mdio_macros_inc           .set    1

; MDIO Register Offsets
    .asg    0x0,     MDIO_VERSION_REG_OFFSET
    .asg    0x4,     MDIO_CONTROL_REG_OFFSET
    .asg    0x8,     MDIO_ALIVE_REG_OFFSET
    .asg    0xC,     MDIO_LINK_REG_OFFSET
    .asg    0x10,    MDIO_LINK_INT_RAW_REG_OFFSET
    .asg    0x14,    MDIO_LINK_INT_MASKED_REG_OFFSET
    .asg    0x18,    MDIO_LINK_INT_MASK_SET_REG_OFFSET
    .asg    0x1C,    MDIO_LINK_INT_MASK_CLEAR_REG_OFFSET
    .asg    0x20,    MDIO_USER_INT_RAW_REG_OFFSET
    .asg    0x24,    MDIO_USER_INT_MASKED_REG_OFFSET
    .asg    0x28,    MDIO_USER_INT_MASK_SET_REG_OFFSET
    .asg    0x2C,    MDIO_USER_INT_MASK_CLEAR_REG_OFFSET
    .asg    0x30,    MDIO_MANUAL_IF_REG_OFFSET
    .asg    0x34,    MDIO_POLL_REG_OFFSET
    .asg    0x38,    MDIO_POLL_EN_REG_OFFSET
    .asg    0x3C,    MDIO_CLAUS45_REG_OFFSET
    .asg    0x40,    MDIO_USER_ADDR0_REG_OFFSET
    .asg    0x44,    MDIO_USER_ADDR1_REG_OFFSET
    .asg    0x80,    MDIO_USER_ACCESS_0_REG_OFFSET
    .asg    0x84,    MDIO_USER_PHY_SEL_0_REG_OFFSET
    .asg    0x88,    MDIO_USER_ACCESS_1_REG_OFFSET
    .asg    0x8C,    MDIO_USER_PHY_SEL_1_REG_OFFSET

; MDIO CLK Freq is about = (PRU_CLK/(2*MDIO_CLK_PERIOD_CONFIG+12))
MDIO_CLK_PERIOD_CONFIG                  .set    34

; Bit positions corresponding to MDIO registers
MDIO_PIN_BIT                        .set    0
MDIO_OE_BIT                         .set    1
MDIO_MDCLK_O_BIT                    .set    2
ENABLE_BIT                          .set    30
PREAMBLE_BIT                        .set    20
MDIO_POLL_REG_MANUALMODE_BIT        .set    31
MDIO_POLL_REG_STATECHANGEMODE_BIT   .set    30
MDIO_USER_ACCESS_REG_GO_BIT         .set    31
MDIO_USER_ACCESS_REG_WRITE_BIT      .set    30
MDIO_USER_ACCESS_REG_ACK_BIT        .set    29

MASK_5_BITS                         .set    0x1F
DEFINED_MDIO_MEM_OFFSET             .set    0xE40

; Global Registers:
;   mdio_manual_if_reg

nopx    .macro  COUNT
    loop    endLoop?, COUNT-1
    nop
endLoop?:

  .endm

;************************************************************************************
;   Macro: m_mdio_set_bit
;
;   Set the specified bit in MDIO_MANUAL_IF_REG
;
;   PEAK cycles:
;           3 cycle (1 + 2 for sbbo)
;
;   Pseudo code:
;       ========== C ==========
;       MDIO_MANUAL_IF_REG |= (1<<BIT);
;       =======================
;
;   Parameters:
;           BIT         - BIT name to set {MCLK, MDATA, MTXEN}
;           baseAddr    - baseAddr of MDIO module
;************************************************************************************
m_mdio_set_bit  .macro  BIT, baseAddr
    .if     $symcmp(BIT,"MCLK") = 0
    set     mdio_manual_if_reg, mdio_manual_if_reg, MDIO_MDCLK_O_BIT
    .elseif $symcmp(BIT,"MDATA") = 0
    set     mdio_manual_if_reg, mdio_manual_if_reg, MDIO_PIN_BIT
    .elseif $symcmp(BIT,"MTXEN") = 0
    set     mdio_manual_if_reg, mdio_manual_if_reg, MDIO_OE_BIT
    .endif  ; BIT
    sbbo    &mdio_manual_if_reg, baseAddr, MDIO_MANUAL_IF_REG_OFFSET, 1

  .endm

;************************************************************************************
;   Macro: m_mdio_clr_bit
;
;   Clear the specified bit in MDIO_MANUAL_IF_REG
;
;   PEAK cycles:
;           3 cycle (1 + 2 for sbbo)
;
;   Pseudo code:
;       ========== C ==========
;       MDIO_MANUAL_IF_REG &= ~(1<<BIT);
;       =======================
;
;   Parameters:
;           BIT         - BIT name to clear {MCLK, MDATA, MTXEN}
;           baseAddr    - baseAddr of MDIO module
;************************************************************************************
m_mdio_clr_bit  .macro  BIT, baseAddr
    .if     $symcmp(BIT,"MCLK") = 0
    clr     mdio_manual_if_reg, mdio_manual_if_reg, MDIO_MDCLK_O_BIT
    .elseif $symcmp(BIT,"MDATA") = 0
    clr     mdio_manual_if_reg, mdio_manual_if_reg, MDIO_PIN_BIT
    .elseif $symcmp(BIT,"MTXEN") = 0
    clr     mdio_manual_if_reg, mdio_manual_if_reg, MDIO_OE_BIT
    .endif  ; BIT
    sbbo    &mdio_manual_if_reg, baseAddr, MDIO_MANUAL_IF_REG_OFFSET, 1

  .endm

;************************************************************************************
;   Macro: m_mdio_toggle_bit_lh
;
;   Toggle specified bit in MDIO_MANUAL_IF_REG
;
;   PEAK cycles:
;           6 + 2*MDIO_CLK_PERIOD_CONFIG cycle
;
;   Pseudo code:
;       ========== C ==========
;       CLR(BIT)
;       WAIT(MDIO_CLK_PERIOD_CONFIG)
;       SET(BIT)
;       WAIT(MDIO_CLK_PERIOD_CONFIG)
;       =======================
;
;   Parameters:
;           BIT         - name of BIT to toggle {MCLK, MDATA, MTXEN}
;           baseAddr    - baseAddr of MDIO module
;************************************************************************************
m_mdio_toggle_bit_lh   .macro   BIT, baseAddr
    m_mdio_clr_bit      BIT, baseAddr
    nopx    MDIO_CLK_PERIOD_CONFIG
    m_mdio_set_bit      BIT, baseAddr
    nopx    MDIO_CLK_PERIOD_CONFIG

  .endm

;************************************************************************************
;   Macro: m_mdio_send_preamble
;
;   Send the preamble required for MDIO communication - 32 high bits continously
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       DISABLE(MTXEN)
;       CLR(MCLK)
;       WAIT(50 cycles)
;       for i: 1 to 32
;           TOGGLE(MCLK)
;       =======================
;
;   Parameters:
;           baseAddr    - baseAddr of MDIO module
;************************************************************************************
m_mdio_send_preamble   .macro   baseAddr, bitNum
    ; Disable MDIO Data output line
    m_mdio_clr_bit      "MTXEN", baseAddr

    m_mdio_clr_bit      "MCLK", baseAddr    ; Give time for pull-up to work
    nopx    50

    ldi     bitNum, 32
SEND_PREAMBLE?:
    m_mdio_toggle_bit_lh    "MCLK", baseAddr
    sub     bitNum, bitNum, 1
    qbne    SEND_PREAMBLE?, bitNum, 0

  .endm

;************************************************************************************
;   Macro: m_mdio_send_field
;
;   Send the bit field for MDIO communication
;
;   PEAK cycles:
;           2 + (5 + (6 + 2*MDIO_CLK_PERIOD_CONFIG))*NO_OF_BITS cycle = 2 + (31*NO_OF_BITS)
;
;   Pseudo code:
;       ========== C ==========
;       bitNum = NO_OF_BITS-1
;       for bitNum != 0xFF
;           SEND(value(bitNum))
;           bitNum--
;       =======================
;
;   Parameters:
;           baseAddr    - baseAddr of MDIO module
;           noOfBits    - Number of Bits to send
;           value       - Bitfield value to send
;           bitNum      - 8 bit register
;************************************************************************************
m_mdio_send_field       .macro   baseAddr, NO_OF_BITS, value, bitNum

    ldi     bitNum, NO_OF_BITS
    sub     bitNum, bitNum, 1
SEND_FIELD?:
    qbbc    SEND_BIT_LOW?, value, bitNum
    m_mdio_set_bit          "MDATA", baseAddr   ; Drive the MDATA line high
    qba     END_SEND_BIT_HIGH?
SEND_BIT_LOW?:
    m_mdio_clr_bit          "MDATA", baseAddr   ; Drive the MDATA line low
END_SEND_BIT_HIGH?:
    m_mdio_toggle_bit_lh    "MCLK", baseAddr    ; Create an MDIO Clk pulse
    sub     bitNum, bitNum, 1
    qbne    SEND_FIELD?, bitNum, 0xFF

  .endm

;************************************************************************************
;   Macro: m_mdio_init_manual_mode
;
;   Initialize the MDIO module registers to work in manual mode
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       Enable MDIO state machine
;       Disable Preamble for MDIO Clause22 communication
;       Disable MDIO state machine
;       Enable manual mode of control
;       Disable Poll Enable for all PHYs
;       Update manual interface register value to local copy
;       =======================
;
;   Parameters:
;           baseAddr    - baseAddr of MDIO module
;           temp_reg    - temporary register to store intermediate data
;************************************************************************************
m_mdio_init_manual_mode     .macro  baseAddr, temp_reg
    lbbo    &temp_reg, baseAddr, MDIO_CONTROL_REG_OFFSET, 4

    set     temp_reg, temp_reg, ENABLE_BIT                      ; Enable MDIO state machine once
    set     temp_reg, temp_reg, PREAMBLE_BIT                    ; Disable Preamble
    sbbo    &temp_reg, baseAddr, MDIO_CONTROL_REG_OFFSET, 4
    nopx    250
    clr     temp_reg, temp_reg, ENABLE_BIT                      ; Disable MDIO state machine
    sbbo    &temp_reg, baseAddr, MDIO_CONTROL_REG_OFFSET, 4
    lbbo    &temp_reg, baseAddr, MDIO_POLL_REG_OFFSET, 4
    ldi     temp_reg, 0
    set     temp_reg, temp_reg, MDIO_POLL_REG_MANUALMODE_BIT    ; Enable manual mode of control
    sbbo    &temp_reg, baseAddr, MDIO_POLL_REG_OFFSET, 4
    ldi     temp_reg, 0
    ldi32   temp_reg, 1<<31                                     ; Disable Poll Enable for all PHYs (due to harware limitation bit 31 needs to be 1)
    sbbo    &temp_reg, baseAddr, MDIO_POLL_EN_REG_OFFSET, 4
    lbbo    &mdio_manual_if_reg, baseAddr, MDIO_MANUAL_IF_REG_OFFSET, 1 ; Update manual interface register value to local copy

    m_mdio_toggle_bit_lh    "MCLK", baseAddr

  .endm

;************************************************************************************
;   Macro: m_mdio_read
;
;   Read from the specified PHY's specified register
;
;   PEAK cycles:
;           1187 cycle - CLAUSE22 - confirm sbbo cycles
;
;   Pseudo code:
;       ========== C ==========
;       Send initial bits according to Clause selected for read operation
;       Check for acknowledgement from PHY
;       Store the received data
;       =======================
;
;   Parameters:
;           CLAUSE      - Clause to use for the read operation {CLAUSE22, CLAUSE45}
;           baseAddr    - baseAddr of MDIO module
;           phyAddr     - register containing PHY address
;           regAddr     - register containing PHY's register address
;           value       - register which will be saved with the read data [15:0]
;           bitNum      - register used for bit number tracking [7:0]
;************************************************************************************
m_mdio_read         .macro   CLAUSE, baseAddr, phyAddr, regAddr, userAccessReg, bitNum, value
    ; Send preamble
    .if     $symcmp(CLAUSE,"CLAUSE45") = 0
    m_mdio_send_preamble    baseAddr, bitNum
    .endif  ; CLAUSE

    ; Send Fields
    m_mdio_clr_bit          "MCLK", baseAddr        ; Disable Phy Interrupt driver
    m_mdio_clr_bit          "MDATA", baseAddr       ; MDIO Data out Low
    nopx    50
    m_mdio_set_bit          "MTXEN", baseAddr       ; Enable MDIO Data output line

    .if     $symcmp(CLAUSE,"CLAUSE22") = 0
    ldi     value, 0x6                              ; Issue clause 22 MII read function {0,1,1,0}
    .elseif $symcmp(CLAUSE,"CLAUSE45") = 0
    ldi     value, 0x3                              ; Issue clause 45 MII read function {0,0,1,1}
    .endif  ; CLAUSE
    lsl     value, value, 5
    or      value, value, phyAddr
    lsl     value, value, 5
    or      value, value, regAddr
    m_mdio_send_field       baseAddr, 14, value, bitNum      ; Send read function, device number, register number

    m_mdio_clr_bit          "MTXEN", baseAddr       ; Disable MDIO Data output line
    m_mdio_toggle_bit_lh    "MCLK", baseAddr        ; send Turn-around cycles

    ; Read Field
    clr     userAccessReg, userAccessReg, MDIO_USER_ACCESS_REG_ACK_BIT ; Reset Ack first
    lbbo    &mdio_manual_if_reg, baseAddr, MDIO_MANUAL_IF_REG_OFFSET, 1
    qbbc    ACK?, mdio_manual_if_reg, MDIO_PIN_BIT  ; Get PHY Ack - acknowledged if = 0

NO_ACK?:
    m_mdio_toggle_bit_lh    "MCLK", baseAddr
    ldi     value, 0xFFFF
    ldi     bitNum, 16
DUMMY_LOOP?:
    m_mdio_toggle_bit_lh    "MCLK", baseAddr
    sub    bitNum, bitNum, 1
    qbne   DUMMY_LOOP?, bitNum, 0
    qba     SKIP_ACK?

ACK?:
    ldi     value, 0
    m_mdio_toggle_bit_lh    "MCLK", baseAddr
    ldi     bitNum, 15
GET_VALUE?:
    lbbo    &mdio_manual_if_reg, baseAddr, MDIO_MANUAL_IF_REG_OFFSET, 1
    qbbc    ADD_BIT_0?, mdio_manual_if_reg, MDIO_PIN_BIT
    set     value, value, bitNum
    qba     SKIP_ADD_BIT_0?
ADD_BIT_0?:
    clr     value, value, bitNum
SKIP_ADD_BIT_0?:
    m_mdio_toggle_bit_lh    "MCLK", baseAddr
    sub     bitNum, bitNum, 1
    qbne    GET_VALUE?, bitNum, 0xFF
    set     userAccessReg, userAccessReg, MDIO_USER_ACCESS_REG_ACK_BIT ; Update Ack received status

SKIP_ACK?:
    mov     userAccessReg.w0, value ; Update Read Value
    m_mdio_clr_bit          "MCLK", baseAddr
    nopx    50                                      ; Give time for pull-up to work
    m_mdio_toggle_bit_lh    "MCLK", baseAddr        ; re-enable PHY Interrupt function

  .endm

;************************************************************************************
;   Macro: m_mdio_write
;
;   Write to the specified PHY's specified register
;
;   PEAK cycles:
;           1135 cycle - CLAUSE22
;
;   Pseudo code:
;       ========== C ==========
;       Send initial bits according to Clause selected for write operation
;       Send the addresses
;       Send the data to the PHY
;       =======================
;
;   Parameters:
;           CLAUSE      - Clause to use for the read operation {CLAUSE22, CLAUSE45}
;           baseAddr    - baseAddr of MDIO module
;           phyAddr     - register containing PHY address
;           regAddr     - register containing PHY's register address
;           value       - value which will be written to the specified address
;           WRITE_MODE  - optional parameter in CLAUSE22 case
;                         {WRITE_ADDRESS, WRITE_VALUE} options in CLAUSE45 case
;************************************************************************************
m_mdio_write        .macro   CLAUSE, baseAddr, phyAddr, regAddr, tempReg, bitNum, value, WRITE_MODE
    ; Send preamble
    .if     $symcmp(CLAUSE,"CLAUSE45") = 0
    m_mdio_send_preamble    baseAddr, bitNum
    .endif  ; CLAUSE

    ; Send Frame
    m_mdio_clr_bit      "MCLK", baseAddr                ; Disable Phy Interrupt driver
    nopx    50
    m_mdio_set_bit      "MTXEN", baseAddr               ; Enable MDIO Data output line

    .if     $symcmp(CLAUSE,"CLAUSE22") = 0
      ldi     tempReg, 0x5                              ; Issue clause 22 MII write function {0,1,0,1}
      .elseif $symcmp(CLAUSE,"CLAUSE45") = 0
      .if     $symcmp(WRITE_MODE,"WRITE_ADDRESS") = 0
        ldi     tempReg, 0x0                            ; Issue clause 45 MII write function {0,0,0,0}
        .elseif $symcmp(WRITE_MODE,"WRITE_VALUE") = 0
        ldi     tempReg, 0x1                            ; Issue clause 45 MII write function {0,0,0,1}
      .endif  ; WRITE_MODE
    .endif  ; CLAUSE

    lsl     tempReg, tempReg, 5
    or      tempReg, tempReg, phyAddr
    lsl     tempReg, tempReg, 5
    or      tempReg, tempReg, regAddr
    m_mdio_send_field   baseAddr, 14, tempReg, bitNum     ; Send write function, device number, register number

    ; Can combine the below code into single with above m_mdio_send_field call
    m_mdio_set_bit          "MDATA", baseAddr
    m_mdio_toggle_bit_lh    "MCLK",  baseAddr           ; send Turn-around cycles
    m_mdio_clr_bit          "MDATA", baseAddr
    m_mdio_toggle_bit_lh    "MCLK",  baseAddr

    m_mdio_send_field   baseAddr, 16, value, bitNum     ; Send the value MSB first
    m_mdio_clr_bit      "MTXEN", baseAddr               ; Disable MDIO Data output line

    m_mdio_clr_bit          "MCLK", baseAddr
    nopx    50                                          ; Give time for pull-up to work
    m_mdio_toggle_bit_lh    "MCLK", baseAddr            ; re-enable PHY Interrupt function

  .endm

    .endif      ; __mdio_macros_inc

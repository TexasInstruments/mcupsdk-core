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
;   File:     icss_gpio_cntrl_macros.inc
;
;   Brief:    Macros for controlling GPIO pins
;************************************************************************************

    .if    !$defined("__icss_gpio_cntrl_macros_inc")
__icss_gpio_cntrl_macros_inc    .set 1

;    .include "icss_gpio_regs.inc"
    .include "icss_struct_defines.inc"

GPIO_DIR01              .set    0x10
GPIO_SET_DATA01         .set    0x18
GPIO_CLR_DATA01         .set    0x1C
GPIO_DIR23              .set    0x38
GPIO_SET_DATA23         .set    0x40
GPIO_CLR_DATA23         .set    0x44
GPIO_DIR45              .set    0x60
GPIO_SET_DATA45         .set    0x68
GPIO_CLR_DATA45         .set    0x6C
GPIO_DIR67              .set    0x88
GPIO_SET_DATA67         .set    0x90
GPIO_CLR_DATA67         .set    0x94
GPIO_DIR8               .set    0xB0
GPIO_SET_DATA8          .set    0xB8
GPIO_CLR_DATA8          .set    0xBC

;******************************************************************************
;   Macro: m_set_bank01
;
;   Set the output value of GPIO Instance's bank01 gpio pins
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_SET_DATA01
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_set_bank01    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi32   TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_SET_DATA01, 4
    .endm

;******************************************************************************
;   Macro: m_clr_bank01
;
;   Clear the output value of GPIO Instance's bank01 gpio pins
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_CLR_DATA01
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_clr_bank01    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi32   TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_CLR_DATA01, 4
    .endm

;******************************************************************************
;   Macro: m_dir_bank01
;
;   Set the direction of GPIO Instance's bank01 gpio pins
;       0h = output
;       1h = input
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_DIR01
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_dir_bank01    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi32   TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_DIR01 , 4
    .endm

;******************************************************************************
;   Macro: m_set_bank23
;
;   Set the output value of GPIO Instance's bank23 gpio pins
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_SET_DATA23
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_set_bank23    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi32   TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_SET_DATA23, 4
    .endm

;******************************************************************************
;   Macro: m_clr_bank23
;
;   Clear the output value of GPIO Instance's bank23 gpio pins
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_CLR_DATA23
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_clr_bank23    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi32   TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_CLR_DATA23, 4
    .endm

;******************************************************************************
;   Macro: m_dir_bank23
;
;   Set the direction of GPIO Instance's bank23 gpio pins
;       0h = output
;       1h = input
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_DIR23
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_dir_bank23    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi32   TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_DIR23 , 4
    .endm

;******************************************************************************
;   Macro: m_set_bank45
;
;   Set the output value of GPIO Instance's bank45 gpio pins
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_SET_DATA45
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_set_bank45    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi32   TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_SET_DATA45, 4
    .endm

;******************************************************************************
;   Macro: m_clr_bank45
;
;   Clear the output value of GPIO Instance's bank45 gpio pins
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_CLR_DATA45
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_clr_bank45    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi32   TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_CLR_DATA45, 4
    .endm

;******************************************************************************
;   Macro: m_dir_bank45
;
;   Set the direction of GPIO Instance's bank45 gpio pins
;       0h = output
;       1h = input
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_DIR45
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_dir_bank45    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi32   TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_DIR45 , 4
    .endm

;******************************************************************************
;   Macro: m_set_bank67
;
;   Set the output value of GPIO Instance's bank67 gpio pins
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_SET_DATA67
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_set_bank67    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi32   TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_SET_DATA67, 4
    .endm

;******************************************************************************
;   Macro: m_clr_bank67
;
;   Clear the output value of GPIO Instance's bank67 gpio pins
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_CLR_DATA67
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_clr_bank67    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi32   TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_CLR_DATA67, 4
    .endm

;******************************************************************************
;   Macro: m_dir_bank67
;
;   Set the direction of GPIO Instance's bank67 gpio pins
;       0h = output
;       1h = input
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_DIR67
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_dir_bank67    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi32   TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_DIR67 , 4
    .endm

;******************************************************************************
;   Macro: m_set_bank8
;
;   Set the output value of GPIO Instance's bank8 gpio pins
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_SET_DATA8
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_set_bank8    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi     TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_SET_DATA8, 2
    .endm

;******************************************************************************
;   Macro: m_clr_bank8
;
;   Clear the output value of GPIO Instance's bank8 gpio pins
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_CLR_DATA8
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_clr_bank8    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi     TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_CLR_DATA8, 2
    .endm

;******************************************************************************
;   Macro: m_dir_bank8
;
;   Set the direction of GPIO Instance's bank8 gpio pins
;       0h = output
;       1h = input
;
;   PeakCycles:
;       6 cycles
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   PseudoCode:
;      (start code)
;       Write bank_data to register GPIO_INSTANCE + GPIO_DIR8
;      (end code)
;
;   Parameters:
;       TEMP_REG      -  Register Structure with atleast 2 Registers
;       GPIO_INSTANCE -  Base address of the corresponding GPIO Instance (Peripheral Identification Register)
;       bank_data     -  Value to write to the bank register
;
;******************************************************************************
m_dir_bank8    .macro  TEMP_REG, GPIO_INSTANCE, bank_data
    ldi32   TEMP_REG.reg1.x, GPIO_INSTANCE
    ldi     TEMP_REG.reg2.x, bank_data
    sbbo    &TEMP_REG.reg2.x, TEMP_REG.reg1.x, GPIO_DIR8 , 2
    .endm

    .endif  ; __icss_gpio_cntrl_macros_inc
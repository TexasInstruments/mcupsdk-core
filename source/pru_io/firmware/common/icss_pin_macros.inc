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
;   File:     icss_pin_macros.inc
;
;   Brief:    PRU PINS related macro definitions
;
;   Includes:
;            icss_constant_defines.inc
;************************************************************************************
    .if    !$defined("__icss_pin_macros_inc")
__icss_pin_macros_inc    .set 1

    .include "icss_constant_defines.inc"

;************************************************************************************
;   Macro: m_wait_high_pulse
;
;   Detect a high pulse on specified pin
;   PRU_PIN:    _____᠆᠆᠆᠆᠆______
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       wait (PRU_PIN to go HIGH)
;       wait (PRU_PIN to go LOW)
;       =======================
;
;   Parameters:
;           FILTER_CYCLES   - Debounce filter cycles
;           PRU_PIN         - PRU pin on which we want to detect pulse
;************************************************************************************
m_wait_high_pulse       .macro  FILTER_CYCLES, PRU_PIN
    ; TODO: if FILTER_CYCLES = 0, throw compilation error
wait_for_high_transition?:
    .loop   FILTER_CYCLES
    qbbc    wait_for_high_transition?, r31, PRU_PIN
    .endloop ;  FILTER_CYCLES
wait_for_low_transition?:
    ; read FILTER_CYCLES times to make sure there was no glitch
    .loop   FILTER_CYCLES
    qbbs    wait_for_low_transition?, r31, PRU_PIN
    .endloop ;  FILTER_CYCLES
    .endm

;************************************************************************************
;   Macro: m_wait_low_pulse
;
;   Detect a low pulse on specified pin
;   PRU_PIN:    ᠆᠆᠆᠆᠆᠆᠆᠆᠆___᠆᠆᠆᠆᠆᠆᠆᠆᠆
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       wait (PRU_PIN to go LOW)
;       wait (PRU_PIN to go HIGH)
;       =======================
;
;   Parameters:
;           FILTER_CYCLES   - Debounce filter cycles
;           PRU_PIN   - PRU pin on which we want to detect pulse
;************************************************************************************
m_wait_low_pulse        .macro  FILTER_CYCLES, PRU_PIN
    ; TODO: if FILTER_CYCLES = 0, throw compilation error
wait_for_low_transition?:
    .loop   FILTER_CYCLES
    qbbs    wait_for_low_transition?, r31, PRU_PIN
    .endloop ;  FILTER_CYCLES
wait_for_high_transition?:
    ; read FILTER_CYCLES times to make sure there was no glitch
    .loop   FILTER_CYCLES
    qbbc    wait_for_high_transition?, r31, PRU_PIN
    .endloop ;  FILTER_CYCLES
    .endm

;************************************************************************************
;   Macro: m_pru_clr_pin
;
;   CLEAR specified pru pin
;
;   PEAK cycles:
;           1 cycle
;
;   Pseudo code:
;       ========== C ==========
;       set (PRU_PIN to LOW)
;       =======================
;
;   Parameters:
;           PRU_PIN   - PRU pin which we want to drive LOW
;************************************************************************************
m_pru_clr_pin       .macro  PRU_PIN
    clr     r30, r30, PRU_PIN
    .endm

;************************************************************************************
;   Macro: m_pru_set_pin
;
;   SET specified pru pin
;
;   PEAK cycles:
;           1 cycle
;
;   Pseudo code:
;       ========== C ==========
;       set (PRU_PIN to HIGH)
;       =======================
;
;   Parameters:
;           PRU_PIN   - PRU pin which we want to drive HIGH
;************************************************************************************
m_pru_set_pin      .macro  PRU_PIN
    set     r30, r30, PRU_PIN
    .endm

;************************************************************************************
;   Macro: m_pru_set_pins
;
;   SET specified pru pin
;
;   PEAK cycles:
;           1 cycle
;
;   Pseudo code:
;       ========== C ==========
;       set (PRU_PIN to HIGH)
;       =======================
;
;   Parameters:
;           PRU_PIN   - PRU pin which we want to drive HIGH
;************************************************************************************
m_pru_set_pins      .macro  pin0, pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8, pin9, pin10, pin11, pin12, pin13, pin14, pin15, pin16, pin17, pin18, pin19
    ; maybe .loop will simplify this by using only 1 parameter
    .var    value
    .eval    0x00000, value
    .if      $symlen(pin19)
    .eval   value | (1 << pin19), value
    .endif
    .if      $symlen(pin18)
    .eval   value | (1 << pin18), value
    .endif
    .if      $symlen(pin17)
    .eval   value | (1 << pin17), value
    .endif
    .if      $symlen(pin16)
    .eval   value | (1 << pin16), value
    .endif
    .if      $symlen(pin15)
    .eval   value | (1 << pin15), value
    .endif
    .if      $symlen(pin14)
    .eval   value | (1 << pin14), value
    .endif
    .if      $symlen(pin13)
    .eval   value | (1 << pin13), value
    .endif
    .if      $symlen(pin12)
    .eval   value | (1 << pin12), value
    .endif
    .if      $symlen(pin11)
    .eval   value | (1 << pin11), value
    .endif
    .if      $symlen(pin10)
    .eval   value | (1 << pin10), value
    .endif
    .if      $symlen(pin9)
    .eval   value | (1 << pin9), value
    .endif
    .if      $symlen(pin8)
    .eval   value | (1 << pin8), value
    .endif
    .if      $symlen(pin7)
    .eval   value | (1 << pin7), value
    .endif
    .if      $symlen(pin6)
    .eval   value | (1 << pin6), value
    .endif
    .if      $symlen(pin5)
    .eval   value | (1 << pin5), value
    .endif
    .if      $symlen(pin4)
    .eval   value | (1 << pin4), value
    .endif
    .if      $symlen(pin3)
    .eval   value | (1 << pin3), value
    .endif
    .if      $symlen(pin2)
    .eval   value | (1 << pin2), value
    .endif
    .if      $symlen(pin1)
    .eval   value | (1 << pin1), value
    .endif
    .if      $symlen(pin0)
    .eval   value | (1 << pin0), value
    .endif
    ldi32      r30, value

    .endm

;************************************************************************************
;   Macro: m_drive_pru_pins
;
;   Write to r30 register to drive all PRU pins
;
;   PEAK cycles:
;           2 cycle
;
;   Pseudo code:
;       ========== C ==========
;       r30 = value
;       =======================
;
;   Parameters:
;           PRU_PIN   - PRU pin which we want to drive HIGH
;************************************************************************************
m_drive_pru_pins        .macro  value
    .if     value >> 16
    ldi32   r30, value
    .else
    ldi     r30, value
    .endif
    .endm

;************************************************************************************
;   Macro: m_read_pru_pins
;
;   Read from r31 register to get value of all PRU pins
;
;   PEAK cycles:
;           2 cycle
;
;   Pseudo code:
;       ========== C ==========
;       reg = r31
;       =======================
;
;   Parameters:
;           reg   - register to store the read pin values
;************************************************************************************
m_read_pru_pins     .macro  reg
    ldi32   reg, r31
    .endm

    .endif  ; __icss_pin_macros_inc

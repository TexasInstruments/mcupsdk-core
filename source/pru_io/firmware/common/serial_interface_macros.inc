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
;   File:     serial_interface_macros.inc
;
;   Brief:    Serial Interface related macro definitions
;
;   Includes:
;            icss_pin_macros.inc
;            time_macros.inc
;************************************************************************************
    .if    !$defined("__adc_serial_interface_macros_inc")
__adc_serial_interface_macros_inc       .set 1

    .include "icss_pin_macros.inc"
    .include "time_macros.inc"

;************************************************************************************
;   Macro: m_read_packet_spi_mode1_msb_gpo_sclk
;
;   Read 1 data packet from the SPI interface.
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       dataReg = 0
;       bitId = PACKETSIZE - 1
;       loop:   READ_PACKET
;           set   SCLK_PIN
;           wait  16 cycles
;           clear SCLK_PIN
;       =======================
;
;   Parameters:
;           dataReg         -  To store read serial data packet
;           PACKETSIZE      -  Size of data packet in bits
;           bitId           -  To track Bit index (8 bits wide)
;           SCLK_PIN        -  PRU Pin number for SCLK
;           SDI_PIN         -  PRU Pin number for SDI
;           DELAY_COMPEN_1  -  PRU cycles to wait for while SCLK is high
;           DELAY_COMPEN_2  -  PRU cycles to wait for while SCLK is low
;************************************************************************************
m_read_packet_spi_mode1_msb_gpo_sclk   .macro  dataReg, PACKETSIZE, bitId, SCLK_PIN, SDI_PIN, DELAY_COMPEN_1, DELAY_COMPEN_2
    ; clear dataReg which is used to store received data
    ldi     bitId, PACKETSIZE       ; bitId is bit position pointer

READ_BIT_LOOP?:
    m_pru_set_pin   SCLK_PIN
    sub     bitId, bitId, 1
    .loop   DELAY_COMPEN_1 ; 6
    NOP
    .endloop
    m_pru_clr_pin   SCLK_PIN
    qbbc    BIT_ENTRY_0?, r31, SDI_PIN
    set     dataReg, dataReg, bitId     ; store data in register
    qba     SKIP_BIT_ENTRY_0?
BIT_ENTRY_0?:
    clr     dataReg, dataReg, bitId
    nop
SKIP_BIT_ENTRY_0?:
    .loop   DELAY_COMPEN_2 ; 0
    NOP
    .endloop
    qbne    READ_BIT_LOOP?, bitId, 0
    .endm

;************************************************************************************
;   Macro: m_read_packet_spi_mode1_lsb_gpo_sclk
;
;   Read 1 data packet from the SPI interface.
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       dataReg = 0
;       bitId = PACKETSIZE - 1
;       loop:   READ_PACKET
;           set   SCLK_PIN
;           wait  16 cycles
;           clear SCLK_PIN
;       =======================
;
;   Parameters:
;           dataReg         -  To store read serial data packet
;           PACKETSIZE      -  Size of data packet in bits
;           bitId           -  To track Bit index (8 bits wide)
;           SCLK_PIN        -  PRU Pin number for SCLK
;           SDI_PIN         -  PRU Pin number for SDI
;           DELAY_COMPEN_1  -  PRU cycles to wait for while SCLK is high
;           DELAY_COMPEN_2  -  PRU cycles to wait for while SCLK is low
;************************************************************************************
m_read_packet_spi_mode1_lsb_gpo_sclk   .macro  dataReg, PACKETSIZE, bitId, SCLK_PIN, SDI_PIN, DELAY_COMPEN_1, DELAY_COMPEN_2
    ; clear dataReg which is used to store received data
    ldi     bitId, 0       ; bitId is bit position pointer

READ_BIT_LOOP?:
    m_pru_set_pin   SCLK_PIN
    ; m_wait_cycles   7
    .loop   DELAY_COMPEN_1 ; 6
    NOP
    .endloop
    m_pru_clr_pin   SCLK_PIN
    qbbc    BIT_ENTRY_0?, r31, SDI_PIN
    set     dataReg, dataReg, bitId     ; store data in register
    qba     SKIP_BIT_ENTRY_0?
BIT_ENTRY_0?:
    clr     dataReg, dataReg, bitId
    nop
SKIP_BIT_ENTRY_0?:
    add     bitId, bitId, 1
    .loop   DELAY_COMPEN_2 ; 0
    NOP
    .endloop
    qbne    READ_BIT_LOOP?, bitId, PACKETSIZE
    .endm

;************************************************************************************
;   Macro: m_read_packet_spi_mode1_msb_iep_sclk
;
;   Read 1 data packet from the SPI interface.
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       dataReg = 0
;       bitId = PACKETSIZE - 1
;       loop:   READ_PACKET
;           set   SCLK_PIN
;           wait  16 cycles
;           clear SCLK_PIN
;       =======================
;
;   Parameters:
;           dataReg         -  To store read serial data packet
;           PACKETSIZE      -  Size of data packet in bits
;           bitId           -  To track Bit index (8 bits wide)
;           SDI_PIN         -  PRU Pin number for SDI
;           temp_reg        -  temporary register
;           IEP_INST        -  IEP Instance being used for SCLK
;           SYNC_SIGNAL     -  IEP Sync out signal being used for SCLK
;           DELAY_COMPEN_1  -  PRU cycles to wait for before reading PRU pins
;           DELAY_COMPEN_2  -  PRU cycles to wait for to sync with SCLK period
;************************************************************************************
m_read_packet_spi_mode1_msb_iep_sclk       .macro  dataReg, PACKETSIZE, bitId, temp_reg, IEP_INST, SDI_PIN, SYNC_SIGNAL, DELAY_COMPEN_1, DELAY_COMPEN_2
    ldi     bitId, PACKETSIZE
    ; enable SYNC OUT SIGNAL: set sync_en bit in IEP timer to enable SCLK clock output
    .if     $symcmp(SYNC_SIGNAL,"SYNC0") = 0
    m_set_iep_sync_ctrl_reg     IEP_INST, temp_reg, 0x0023  ; set sync_en (sync0_en)
    .elseif $symcmp(SYNC_SIGNAL,"SYNC1") = 0
    m_set_iep_sync_ctrl_reg     IEP_INST, temp_reg, 0x0085  ; set sync_en (sync1_en)
    .endif  ; SYNC_SIGNAL
    sub     bitId, bitId, 1
    m_wait_cycles       DELAY_COMPEN_1 ; 15 ; DELAY_COMPENSATION ; delay compensation - needs to be adjusted per usecase
    ; loop  READ_BIT_LOOP_END, PACKETSIZE
READ_BIT_LOOP?:
    qbbc    BIT_ENTRY_0?, r31, SDI_PIN
    set     dataReg, dataReg, bitId     ; store data in register
    qba     SKIP_BIT_ENTRY_0?
BIT_ENTRY_0?:
    clr     dataReg, dataReg, bitId
    nop
SKIP_BIT_ENTRY_0?:
    sub     bitId, bitId, 1
    .loop   DELAY_COMPEN_2 ; 3 ; To align PRU pin sampling timings with SCLK timings
    nop
    .endloop
    qbne    READ_BIT_LOOP?, bitId, 0xFF ; TODO: Replace with loop instruction
    ; READ_BIT_LOOP_END:
    ; disable SYNC OUT SIGNAL: clear sync_en bit in IEP timer to diable SCLK clock output
    m_set_iep_sync_ctrl_reg     IEP_INST, temp_reg, 0x00    ; clear sync_en (sync0_en, sync1_en)
    ; TODO: SHould Make it 0x01?
    .endm

;************************************************************************************
;   Macro: m_read_packet_spi_mode1_lsb_iep_sclk
;
;   Read 1 data packet from the SPI interface.
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       dataReg = 0
;       bitId = PACKETSIZE - 1
;       loop:   READ_PACKET
;           set   SCLK_PIN
;           wait  16 cycles
;           clear SCLK_PIN
;       =======================
;
;   Parameters:
;           dataReg         -  To store read serial data packet
;           PACKETSIZE      -  Size of data packet in bits
;           bitId           -  To track Bit index (8 bits wide)
;           SDI_PIN         -  PRU Pin number for SDI
;           temp_reg        -  temporary register
;           IEP_INST        -  IEP Instance being used for SCLK
;           SYNC_SIGNAL     -  IEP Sync out signal being used for SCLK
;           DELAY_COMPEN_1  -  PRU cycles to wait for before reading PRU pins
;           DELAY_COMPEN_2  -  PRU cycles to wait for to sync with SCLK period
;************************************************************************************
m_read_packet_spi_mode1_lsb_iep_sclk       .macro  dataReg, PACKETSIZE, bitId, temp_reg, IEP_INST, SDI_PIN, SYNC_SIGNAL, DELAY_COMPEN_1, DELAY_COMPEN_2
    ldi     bitId, 0
    ; NOTE: Period should be LCM of iep_clk and core_clk
    ; enable SYNC OUT SIGNAL: set sync_en bit in IEP timer to enable SCLK clock output
    .if     $symcmp(SYNC_SIGNAL,"SYNC0") = 0
    m_set_iep_sync_ctrl_reg     IEP_INST, temp_reg, 0x0023  ; set sync_en (sync0_en)
    .elseif $symcmp(SYNC_SIGNAL,"SYNC1") = 0
    m_set_iep_sync_ctrl_reg     IEP_INST, temp_reg, 0x0085  ; set sync_en (sync1_en)
    .endif  ; SYNC_SIGNAL
    m_wait_cycles       DELAY_COMPEN_1 ; 16 ; DELAY_COMPENSATION ; delay compensation - needs to be adjusted per usecase
    ; loop  READ_BIT_LOOP_END, PACKETSIZE
READ_BIT_LOOP?:
    qbbc    BIT_ENTRY_0?, r31, SDI_PIN
    set     dataReg, dataReg, bitId     ; store data in register
    qba     SKIP_BIT_ENTRY_0?
BIT_ENTRY_0?:
    clr     dataReg, dataReg, bitId
    nop
SKIP_BIT_ENTRY_0?:
    add     bitId, bitId, 1
    .loop   DELAY_COMPEN_2 ; 3 ; To align PRU pin sampling timings with SCLK timings
    nop
    .endloop
    qbne    READ_BIT_LOOP?, bitId, PACKETSIZE ; TODO: Replace with loop instruction
    ; READ_BIT_LOOP_END:
    ; disable SYNC OUT SIGNAL: clear sync_en bit in IEP timer to diable SCLK clock output
    m_set_iep_sync_ctrl_reg     0, temp_reg, 0x00    ; clear sync_en (sync0_en, sync1_en)
    .endm

;************************************************************************************
;   Macro: m_send_packet_spi_mode1_msb_gpo_sclk
;
;   Send 1 data packet through the SPI interface.
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       dataReg = 0
;       bitId = PACKETSIZE - 1
;       loop:   READ_PACKET
;           set   SCLK_PIN
;           wait  16 cycles
;           clear SCLK_PIN
;       =======================
;
;   Parameters:
;           dataReg         -  Contains data packet to send
;           PACKETSIZE      -  Size of data packet in bits
;           bitId           -  To track Bit index (8 bits wide)
;           SCLK_PIN        -  PRU Pin number for SCLK
;           SDO_PIN         -  PRU Pin number for SDO
;           DELAY_COMPEN_1  -  PRU cycles to wait for while SCLK is high
;           DELAY_COMPEN_2  -  PRU cycles to wait for while SCLK is low
;************************************************************************************
m_send_packet_spi_mode1_msb_gpo_sclk   .macro  dataReg, PACKETSIZE, bitId, SCLK_PIN, SDO_PIN, DELAY_COMPEN_1, DELAY_COMPEN_2
    ldi     bitId, PACKETSIZE - 1   ; bitId is bit position pointer
    ; sub             bitId, bitId, 1 ; If using register for PACKETSIZE
    ; loop  SEND_BIT_LOOP_END, PACKETSIZE
SEND_BIT_LOOP?:
    m_pru_set_pin   SCLK_PIN
    qbbc            skip_data_high, dataReg, bitId
    m_pru_set_pin   SDO_PIN
    qba             skip_data_low
skip_data_high:
    m_pru_clr_pin   SDO_PIN
    NOP
skip_data_low:
    .loop   DELAY_COMPEN_1 ; 0
    NOP
    .endloop
    m_pru_clr_pin   SCLK_PIN
    .loop   DELAY_COMPEN_2 ; 1
    NOP
    .endloop
    sub             bitId, bitId, 1
    qbne            SEND_BIT_LOOP?, bitId, 0xFF
    ; SEND_BIT_LOOP_END:
    .endm

;************************************************************************************
;   Macro: m_send_packet_spi_mode1_lsb_gpo_sclk
;
;   Send 1 data packet through the SPI interface.
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       dataReg = 0
;       bitId = PACKETSIZE - 1
;       loop:   READ_PACKET
;           set   SCLK_PIN
;           wait  16 cycles
;           clear SCLK_PIN
;       =======================
;
;   Parameters:
;           dataReg         -  Contains data packet to send
;           PACKETSIZE      -  Size of data packet in bits
;           bitId           -  To track Bit index (8 bits wide)
;           SCLK_PIN        -  PRU Pin number for SCLK
;           SDO_PIN         -  PRU Pin number for SDO
;           DELAY_COMPEN_1  -  PRU cycles to wait for while SCLK is high
;           DELAY_COMPEN_2  -  PRU cycles to wait for while SCLK is low
;************************************************************************************
m_send_packet_spi_mode1_lsb_gpo_sclk   .macro  dataReg, PACKETSIZE, bitId, SCLK_PIN, SDO_PIN, DELAY_COMPEN_1, DELAY_COMPEN_2
    ldi     bitId, 0   ; bitId is bit position pointer
    ; sub             bitId, bitId, 1 ; If using register for PACKETSIZE
    ; loop  SEND_BIT_LOOP_END, PACKETSIZE
SEND_BIT_LOOP?:
    m_pru_set_pin   SCLK_PIN
    qbbc            skip_data_high, dataReg, bitId
    m_pru_set_pin   SDO_PIN
    qba             skip_data_low
skip_data_high:
    m_pru_clr_pin   SDO_PIN
    NOP
skip_data_low:
    .loop   DELAY_COMPEN_1 ; 0
    NOP
    .endloop
    m_pru_clr_pin   SCLK_PIN
    .loop   DELAY_COMPEN_2 ; 1
    NOP
    .endloop
    add             bitId, bitId, 1
    qbne            SEND_BIT_LOOP?, bitId, PACKETSIZE
    ; SEND_BIT_LOOP_END:
    .endm

;************************************************************************************
;   Macro: m_send_packet_spi_mode1_msb_iep_sclk
;
;   Send 1 data packet through the SPI interface.
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       dataReg = 0
;       bitId = PACKETSIZE - 1
;       loop:   READ_PACKET
;           set   SCLK_PIN
;           wait  16 cycles
;           clear SCLK_PIN
;       =======================
;
;   Parameters:
;           dataReg         -  Contains data packet to send
;           PACKETSIZE      -  Size of data packet in bits
;           bitId           -  To track Bit index (8 bits wide)
;           SDO_PIN         -  PRU Pin number for SDO
;           temp_reg        -  temporary register
;           IEP_INST        -  IEP Instance being used for SCLK
;           SYNC_SIGNAL     -  IEP Sync out signal being used for SCLK
;           DELAY_COMPEN_1  -  PRU cycles to wait for before sending data, to sync with IEP signal
;           DELAY_COMPEN_2  -  PRU cycles to wait for to sync with SCLK period
;************************************************************************************
m_send_packet_spi_mode1_msb_iep_sclk   .macro  dataReg, PACKETSIZE, bitId, temp_reg, IEP_INST, SDO_PIN, SYNC_SIGNAL, DELAY_COMPEN_1, DELAY_COMPEN_2
    ldi     bitId, PACKETSIZE       ; bitId is bit position pointer

    ; enable SYNC OUT SIGNAL: set sync_en bit in IEP timer to enable SCLK clock output
    .if     $symcmp(SYNC_SIGNAL,"SYNC0") = 0
    m_set_iep_sync_ctrl_reg     IEP_INST, temp_reg, 0x0023  ; set sync_en (sync0_en)
    .elseif $symcmp(SYNC_SIGNAL,"SYNC1") = 0
    m_set_iep_sync_ctrl_reg     IEP_INST, temp_reg, 0x0085  ; set sync_en (sync1_en)
    .endif  ; SYNC_SIGNAL
    sub     bitId, bitId, 1
    .loop   DELAY_COMPEN_1 ; 3
    NOP
    .endloop
SEND_PACKET_LOOP?:      ; time taken should be same as sync out cycle period
    qbbc            skip_data_high, dataReg, bitId
    m_pru_set_pin   SDO_PIN
    qba             skip_data_low
skip_data_high:
    m_pru_clr_pin   SDO_PIN
    NOP
skip_data_low:
    .loop   DELAY_COMPEN_2 ; 3
    NOP
    .endloop
    sub             bitId, bitId, 1
    qbne            SEND_PACKET_LOOP?, bitId, 0xFF
    m_set_iep_sync_ctrl_reg     0, temp_reg, 0x00    ; clear sync_en (sync0_en, sync1_en)

    .endm

;************************************************************************************
;   Macro: m_send_packet_spi_mode1_lsb_iep_sclk
;
;   Send 1 data packet through the SPI interface.
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       dataReg = 0
;       bitId = PACKETSIZE - 1
;       loop:   READ_PACKET
;           set   SCLK_PIN
;           wait  16 cycles
;           clear SCLK_PIN
;       =======================
;
;   Parameters:
;           dataReg         -  Contains data packet to send
;           PACKETSIZE      -  Size of data packet in bits
;           bitId           -  To track Bit index (8 bits wide)
;           SDO_PIN         -  PRU Pin number for SDO
;           temp_reg        -  temporary register
;           IEP_INST        -  IEP Instance being used for SCLK
;           SYNC_SIGNAL     -  IEP Sync out signal being used for SCLK
;           DELAY_COMPEN_1  -  PRU cycles to wait for before sending data, to sync with IEP SCLK signal
;           DELAY_COMPEN_2  -  PRU cycles to wait for to sync with SCLK period
;************************************************************************************
m_send_packet_spi_mode1_lsb_iep_sclk   .macro  dataReg, PACKETSIZE, bitId, temp_reg, IEP_INST, SDO_PIN, SYNC_SIGNAL, DELAY_COMPEN_1, DELAY_COMPEN_2
    ldi     bitId, PACKETSIZE       ; bitId is bit position pointer

    ; enable SYNC OUT SIGNAL: set sync_en bit in IEP timer to enable SCLK clock output
    .if     $symcmp(SYNC_SIGNAL,"SYNC0") = 0
    m_set_iep_sync_ctrl_reg     IEP_INST, temp_reg, 0x0023  ; set sync_en (sync0_en)
    .elseif $symcmp(SYNC_SIGNAL,"SYNC1") = 0
    m_set_iep_sync_ctrl_reg     IEP_INST, temp_reg, 0x0085  ; set sync_en (sync1_en)
    .endif  ; SYNC_SIGNAL
    .loop   DELAY_COMPEN_1 ; 3
    NOP
    .endloop
SEND_PACKET_LOOP?:      ; time taken should be same as sync out cycle period
    qbbc            skip_data_high, dataReg, bitId
    m_pru_set_pin   SDO_PIN
    qba             skip_data_low
skip_data_high:
    m_pru_clr_pin   SDO_PIN
    NOP
skip_data_low:
    .loop   DELAY_COMPEN_2 ; 3
    NOP
    .endloop
    add             bitId, bitId, 1
    qbne            SEND_PACKET_LOOP?, bitId, PACKETSIZE
    m_set_iep_sync_ctrl_reg     0, temp_reg, 0x00    ; clear sync_en (sync0_en, sync1_en)

    .endm

;************************************************************************************
;   Macro: m_send_packet_spi
;
;   Send 1 data packet through the serial interface.
;   ___ Text
;  SPI Mode | CPOL | CPHA |  Idle State Clock Polarity | Clock Phase Used to Sample and/or Shift the Data
;     0     |  0   |  0   |       Logic low            | Data sampled on rising edge and shifted out on the falling edge
;     1     |  0   |  1   |       Logic low            | Data sampled on the falling edge and shifted out on the rising edge
;     2     |  1   |  0   |       Logic high           | Data sampled on the rising edge and shifted out on the falling edge
;     3     |  1   |  1   |       Logic high           | Data sampled on the falling edge and shifted out on the rising edge
;   ___
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       dataReg = 0
;       bitId = PACKETSIZE - 1
;       loop:   READ_PACKET
;           set   SCLK_PIN
;           wait  16 cycles
;           clear SCLK_PIN
;       =======================
;
;   Parameters:
;       dataReg      -  Contains data packet to send
;       MODE         -  SPI MODE: MODE0, MODE1, MODE2, MODE3
;       ENDIAN       -  MSB: MSB First, LSB: LSB First
;       SCLK_TRGR    -  IEP_SYNC: Use IEP Sync_out for SCLK, BIT_BANG: Drive PRU GPO using R30
;       CONFIG       -  Dependent on the invoked macro
;************************************************************************************
m_send_packet_spi       .macro  dataReg, MODE, ENDIAN, SCLK_TRGR, CONFIG
    .if     $symcmp(MODE,"MODE0") = 0

    .elseif $symcmp(MODE,"MODE1") = 0
    .if     $symcmp(ENDIAN,"MSB") = 0
    .if     $symcmp(SCLK_TRGR,"IEP_SYNC") = 0
    m_send_packet_spi_mode1_msb_iep_sclk    dataReg, CONFIG

    .elseif $symcmp(SCLK_TRGR,"BIT_BANG") = 0
    m_send_packet_spi_mode1_msb_gpo_sclk    dataReg, CONFIG

    .endif  ; SCLK_TRGR
    .elseif $symcmp(ENDIAN,"LSB") = 0
    .if     $symcmp(SCLK_TRGR,"IEP_SYNC") = 0
    m_send_packet_spi_mode1_lsb_iep_sclk    dataReg, CONFIG

    .elseif $symcmp(SCLK_TRGR,"BIT_BANG") = 0
    m_send_packet_spi_mode1_lsb_gpo_sclk    dataReg, CONFIG

    .endif  ; SCLK_TRGR
    .endif  ; ENDIAN

    .elseif $symcmp(MODE,"MODE2") = 0
    .elseif $symcmp(MODE,"MODE3") = 0
    .else
    ; compilation error/warning
    .endif  ; MODE

    .endm

;************************************************************************************
;   Macro: m_read_packet_spi
;
;   Read 1 data packet through the serial interface.
;   ___ Text
;  SPI Mode | CPOL | CPHA |  Idle State Clock Polarity | Clock Phase Used to Sample and/or Shift the Data
;     0     |  0   |  0   |       Logic low            | Data sampled on rising edge and shifted out on the falling edge
;     1     |  0   |  1   |       Logic low            | Data sampled on the falling edge and shifted out on the rising edge
;     2     |  1   |  0   |       Logic high           | Data sampled on the rising edge and shifted out on the falling edge
;     3     |  1   |  1   |       Logic high           | Data sampled on the falling edge and shifted out on the rising edge
;   ___
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       dataReg = 0
;       bitId = PACKETSIZE - 1
;       loop:   READ_PACKET
;           set   SCLK_PIN
;           wait  16 cycles
;           clear SCLK_PIN
;       =======================
;
;   Parameters:
;       dataReg      -  To store read data packet
;       MODE         -  SPI MODE: MODE0, MODE1, MODE2, MODE3
;       ENDIAN       -  MSB: MSB First, LSB: LSB First
;       SCLK_TRGR    -  IEP_SYNC: Use IEP Sync_out for SCLK, BIT_BANG: Drive PRU GPO using R30
;       CONFIG       -  Dependent on the invoked macro
;************************************************************************************
m_read_packet_spi       .macro  dataReg, MODE, ENDIAN, SCLK_TRGR, CONFIG
    .if     $symcmp(MODE,"MODE0") = 0

    .elseif $symcmp(MODE,"MODE1") = 0
    .if     $symcmp(ENDIAN,"MSB") = 0
    .if     $symcmp(SCLK_TRGR,"IEP_SYNC") = 0
    m_read_packet_spi_mode1_msb_iep_sclk    dataReg, CONFIG

    .elseif $symcmp(SCLK_TRGR,"BIT_BANG") = 0
    m_read_packet_spi_mode1_msb_gpo_sclk    dataReg, CONFIG

    .endif  ; SCLK_TRGR
    .elseif $symcmp(ENDIAN,"LSB") = 0
    .if     $symcmp(SCLK_TRGR,"IEP_SYNC") = 0
    m_read_packet_spi_mode1_lsb_iep_sclk    dataReg, CONFIG

    .elseif $symcmp(SCLK_TRGR,"BIT_BANG") = 0
    m_read_packet_spi_mode1_lsb_gpo_sclk    dataReg, CONFIG

    .endif  ; SCLK_TRGR
    .endif  ; ENDIAN

    .elseif $symcmp(MODE,"MODE2") = 0
    .elseif $symcmp(MODE,"MODE3") = 0
    .else
    ; compilation error/warning
    .endif  ; MODE

    .endm

;********************************
;  TODO: IMPLEMENTATION Options:
;  * FW_SIZE_OPTIMIZED
;  * SPEED_OPTIMIZED
;********************************

;************************************************************************************
;   Macro: m_read_frame_spi
;
;   Read 1 data packet from the serial interface.
;
;   PEAK cycles:
;           ~ cycle
;
;   Pseudo code:
;       ========== C ==========
;       dataReg = 0
;       bitId = PACKETSIZE - 1
;       loop:   READ_PACKET
;           set   SCLK_PIN
;           wait  16 cycles
;           clear SCLK_PIN
;       =======================
;
;   Parameters:
;       NO_OF_PACKETS    -  Number of data packets per frame
;       WAIT_CSRD        -  Delay required between CS and SCLK trigger
;       tempDataReg      -  To store read serial data packet temporarily
;       packetId         -  To track Packet indices (8 bits wide)
;       dataRegBaseAddr  -  (8 bits) Address of starting data register
;                                   => register R1.bx must be used
;       CONFIG           -  Dependent on the invoked macro
;************************************************************************************
m_read_frame_spi    .macro  NO_OF_PACKETS, WAIT_CSRD, dataRegBaseAddr, tempDataReg, packetId, CONFIG
    ; set /CS low
    m_pru_clr_pin   CS_PIN
    m_wait_cycles   WAIT_CSRD   ; wait for WAIT_CSRD cycles from CS to RD/SCLK trigger
    ldi     packetId, 0
READ_FRAME_LOOP:
    m_read_packet_serial  tempDataReg, CONFIG
    mvid    *dataRegBaseAddr++, tempDataReg
    add     packetId, packetId, 1
    qbne    READ_FRAME_LOOP, packetId, NO_OF_PACKETS
    ; set /CS high
    m_set_pru_pin   CS_PIN
    m_wait_cycles   WAIT_CSRD   ; wait for WAIT_CSRD cycles from CS to RD/SCLK trigger
    .endm

    .endif      ; __adc_serial_interface_macros_inc
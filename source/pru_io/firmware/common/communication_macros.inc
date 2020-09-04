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
;   File:     communication_macros.inc
;
;   Brief:    Communication related macro definitions
;
;   Includes:
;            serial_interface_macros.inc
;            parallel_interface_macros.inc
;************************************************************************************
    .if    !$defined("__communication_macros_inc")
__communication_macros_inc          .set 1

    .include "serial_interface_macros.inc"
    .include "parallel_interface_macros.inc"

;************************************************************************************
;   Macro: m_comm_read_packet
;
;   Read a data packet packet from the specified interface
;
;   PEAK cycles:
;           ~ cycle - depends on configuration
;
;   Pseudo code:
;       ========== C ==========
;       data_reg = value;
;       =======================
;
;   Parameters:
;       INTERFACE    -  PARALLEL16 : 16 bit paralllel interface
;                       PARALLEL8  : 8 bit paralllel interface
;                       SPI        : SPI interface
;       dataReg      -  To store read data packet
;       CONFIG       -  Dependent on the invoked macro
;************************************************************************************
m_comm_read_packet      .macro  INTERFACE, dataReg, CONFIG
    ; TODO: decide parameter sequence and case
    .if         $symcmp(INTERFACE,"PARALLEL16") = 0
    m_read_packet_parallel16    dataReg, CONFIG
    .elseif     $symcmp(INTERFACE,"PARALLEL8") = 0
    m_read_packet_parallel8     dataReg, CONFIG
    .elseif     $symcmp(INTERFACE,"SPI") = 0
    m_read_packet_spi           dataReg, CONFIG
    .endif      ; INTERFACE

    .endm

;************************************************************************************
;   Macro: m_comm_read_frame
;
;   Read a data frame composed of packets from the specified interface
;
;   PEAK cycles:
;           ~ cycle - depends on configuration
;
;   Pseudo code:
;       ========== C ==========
;       data_reg = value;
;       =======================
;
;   Parameters:
;       INTERFACE        - PARALLEL16 : 16 bit paralllel interface
;                          PARALLEL8  : 8 bit paralllel interface
;                          SPI        : SPI interface
;       NO_OF_PACKETS    -  Number of data packets per frame
;       WAIT_CSRD        -  Delay required between CS and SCLK trigger
;       tempDataReg      -  To store read serial data packet temporarily
;       packetId         -  To track Packet indices (8 bits wide)
;       dataRegBaseAddr  -  (8 bits) Address of starting data register
;                                   => register R1.bx must be used
;       CONFIG           -  Dependent on the invoked macro
;************************************************************************************
m_comm_read_frame       .macro  INTERFACE, NO_OF_PACKETS, WAIT_CSRD, dataRegBaseAddr, tempDataReg, packetId, CONFIG
    ; .if     $symcmp(CS_ACTIVE_LEVEL,"CS_ACTIVE_LOW") = 0
    m_pru_clr_pin       CS_PIN      ; set CS low
    m_wait_nano_sec     WAIT_CSRD   ; wait for min WAIT_CSRD ns from CS to RD/SCLK trigger
    ; .endif  ; CS_ACTIVE_LEVEL

    ldi     packetId, 0
READ_FRAME_LOOP:            ; cannot use loop instr as it is used in invoked macros
    m_comm_read_packet      INTERFACE, tempDataReg, CONFIG
    mvid    *dataRegBaseAddr++, tempDataReg
    add     packetId, packetId, 1
    qbne    READ_FRAME_LOOP, packetId, NO_OF_PACKETS

    ; .if     $symcmp(CS_ACTIVE_LEVEL,"CS_ACTIVE_LOW") = 0
    m_set_pru_pin       CS_PIN      ; set CS high
    m_wait_nano_sec     WAIT_CSRD   ; wait for min WAIT_CSRD ns from CS to RD/SCLK trigger
    ; .endif  ; CS_ACTIVE_LEVEL
    .endm

    .endif      ; __communication_macros_inc
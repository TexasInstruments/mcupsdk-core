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
;
;   Brief:    ADS127 Interface Implementaion Example
;************************************************************************************

;************************************* includes *************************************
; Include files for importing symbols and macros
    .include "icss_constant_defines.inc"
    .include "icss_iep_macros.inc"
    .include "icss_struct_defines.inc"
    .include "icss_tm_macros.inc"
    .include "pru_ipc_macros.inc"
    .include "icss_program_flow_macros.inc"
    .include "serial_interface_macros.inc"
    .include "adc_preprocess_macros.inc"

; Import symbols from sysconfig generated files
    .include "ti_pru_io_config.inc"

; Used to receive command values and interrupt from R5F core
    .asg    0x16,       PRGM_FLOW_EVENT
    .asg    31,         PRGM_FLOW_EVENT_BIT

; Import the Chip Support Library Register Address defines
    .cdecls C,  NOLIST
%{
#include "drivers/pruicss/g_v0/cslr_icss_g.h"
%}

; CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

    .global     main
    .sect       ".text"

; ADC_DATA_REG_1 to ADC_DATA_REG_8 should be continous as ADC_DATA_REG_1 address is passed to ipc driver
    .asg	R2,	    ADC_DATA_REG_1
    .asg	R3,	    ADC_DATA_REG_2
    .asg	R4,	    ADC_DATA_REG_3
    .asg	R5,	    ADC_DATA_REG_4
    .asg	R6,	    ADC_DATA_REG_5
    .asg	R7,	    ADC_DATA_REG_6
    .asg	R8,	    ADC_DATA_REG_7
    .asg	R9,	    ADC_DATA_REG_8
    .asg	R10,	TEMP_REG_1

;********
;* MAIN *
;********

main:

init:
    zero	&r0, 120  ; Clear the register space

; Configure the Constant Table entry C28 to point to start of shared memory
; PRU_ICSSG Shared RAM (local-C28) : 00nn_nn00h, nnnn = c28_pointer[15:0]
    ldi     TEMP_REG_1, 0x0100
    sbco    &TEMP_REG_1, ICSS_PRU_CTRL_CONST, 0x28, 2

;----------------------------------------------------------------------------
;   SECTION: IDLE
;   PRU enters the program, executes intialization and then stays in this
;   section until any further command from r5f
;----------------------------------------------------------------------------
IDLE:
    m_prgm_flow_jump_on_intr    PRGM_FLOW_EVENT, PRGM_FLOW_EVENT_BIT, 5, TEMP_REG_1, IDLE, SAMPLE_TRANSFER, ADC_CONFIG, READ_SAMPLES, RESET
    qba     IDLE


;----------------------------------------------------------------------------
;   SECTION: SAMPLE_TRANSFER
;   Configures the Sample Transfer using PRU IPC
;----------------------------------------------------------------------------
SAMPLE_TRANSFER:
    m_pru_ipc_init   R0.b0, R29, CONFIG_PRU_IPC0_CONFIG_MEM_OFFSET  ; Initialize Sample Transfer
    qba              IDLE

;----------------------------------------------------------------------------
;   SECTION: ADC_CONFIG
;   Configures the ADC according to the information received from R5F
;----------------------------------------------------------------------------
    .asg    0,         CONFIG_ADC0_SMEM_OFFSET
ADC_CONFIG:
    ; make sure the pins are correctly driven
    m_pru_set_pin       ADC_CS_PIN
    ; CONFIG_ADC0_SMEM_OFFSET: <No of commands - n>
    ; <Command 1>
    ; ...
    ; <Command n>
    lbco      &R23, C28, CONFIG_ADC0_SMEM_OFFSET, 4
    ldi32     R24,   4
    ldi32     R25,   0
    ;R23->no of commands to be sent
    ;R24->command offset
    ;R25->iterator
SEND_COMMAND?:
    lbco    &R26, C28, R24, 4
	;R26 is command to send
    m_pru_clr_pin       ADC_CS_PIN
    m_wait_nano_sec     10
    m_send_packet_spi_mode1_msb_gpo_sclk	R26, 24, R11.b0, SCLK_PIN, SDO_PIN, 7, 7
    ;m_pru_set_pin       ADC_CS_PIN
    m_wait_nano_sec     10

    add     R25, R25, 1
    add     R24,  R24,  4
    qbne    SEND_COMMAND?, R25, R23

	qba		IDLE

;----------------------------------------------------------------------------
;   SECTION: READ_SAMPLES
;   Starts ADC conversion and then reads the samples using SPI interface
;----------------------------------------------------------------------------
READ_SAMPLES:
    m_pru_set_pin       ADC_CS_PIN
    m_wait_nano_sec     20


START:
    ; Wait for DRDY to go low indicating conversion has completed
    ; DRDY: _᠆᠆᠆᠆_____________________________________________________
    ; SDI:  _____|  Status  |  < Channel 0 - 7 Data >  |  CRC  |____
    m_wait_high_pulse       2, DRDY_PIN

	m_wait_nano_sec     10
    m_pru_clr_pin       ADC_CS_PIN      ; set CS low
    m_wait_nano_sec     10

	;Fetch status register value using serial interface
	m_read_packet_spi_mode1_msb_gpo_sclk       TEMP_REG_1, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4

	; Fetch samples using serial interface from each 8 channels
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_1, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_2, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_3, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_4, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_5, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_6, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_7, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_8, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4

	;Fetch crc register value using serial interface
	m_read_packet_spi_mode1_msb_gpo_sclk       TEMP_REG_1, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4

    m_wait_nano_sec     10
    m_pru_set_pin       ADC_CS_PIN      ; set CS high

	;Convert each sample to 32-bit
   	m_sign_ext          24, 32, ADC_DATA_REG_1, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_2, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_3, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_4, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_5, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_6, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_7, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_8, EXTEND_0

	;Send samples to shared memory using ipc
    ldi                 R1.b0, &ADC_DATA_REG_1
    m_pru_ipc_send      R1.b0, ADC_DATA_REG_1, R0.b0, R29, CONFIG_PRU_IPC0_RX_INTR_ENABLE, CONFIG_PRU_IPC0_RX_EVENT

    m_prgm_flow_jump_on_intr    PRGM_FLOW_EVENT, PRGM_FLOW_EVENT_BIT, 5, TEMP_REG_1, IDLE, SAMPLE_TRANSFER, ADC_CONFIG, READ_SAMPLES, RESET

    qba    START    ; loop to continue collecting samples
    halt

;----------------------------------------------------------------------------
;   SECTION: RESET
;   Reset some modules before restarting conversion
;----------------------------------------------------------------------------
RESET:
    m_pru_ipc_init      R0.b0, R29, CONFIG_PRU_IPC0_CONFIG_MEM_OFFSET ; Reset Sample Transfer

	qba		IDLE

;----------------------------------------------------------------------------
;   SECTION: READ_ADC_REG
;   Read all ADC registers and write the contents into shared memory
;----------------------------------------------------------------------------
; TODO

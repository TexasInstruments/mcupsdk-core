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

    .asg	R2,	    ADC_DATA_REG_1
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
    m_prgm_flow_jump_on_intr    PRGM_FLOW_EVENT, PRGM_FLOW_EVENT_BIT, 6, TEMP_REG_1, IDLE, IEP, SAMPLE_TRANSFER, ADC_CONFIG, READ_SAMPLES, RESET
    qba     IDLE

;----------------------------------------------------------------------------
;   SECTION: IEP
;   Initializes the IEP peripheral for timer and sync signals generation.
;   Sync-out signal is used by SPI to generate clock output to talk to ADC
;----------------------------------------------------------------------------
IEP:
    ; disable iep timer
    m_set_iep_global_cfg_reg   	SPI_IEP_INST, TEMP_REG_1, 0x20

    ; set starting count value of IEP counter
    m_set_iep_count_reg0       	SPI_IEP_INST, TEMP_REG_1, 0xffffffff
    m_set_iep_count_reg1       	SPI_IEP_INST, TEMP_REG_1, 0xffffffff

    ; clear cmp0, cmp1, cmp2 and cmp3 event-status
    m_set_iep_cmp_status_reg    SPI_IEP_INST, TEMP_REG_1, 0x0f

    ; Set SYNC0 start time: e.g. 0 ns (IEP_CMP1_REG0/1 triggers SYNC0 activation time)
    m_set_iep_cmp1_reg0        	SPI_IEP_INST, TEMP_REG_1, 0

    ; Enable CMP1 event
    m_set_iep_cmp_cfg_reg      	SPI_IEP_INST, TEMP_REG_1, 0x04

    ; clear sync_en (sync0_en, sync1_en)
    m_set_iep_sync_ctrl_reg     SPI_IEP_INST, TEMP_REG_1, 0x00

    ; set SYNC0 period: period length is N+1 cycles on setting it as N
    m_set_iep_sync0_period_reg	SPI_IEP_INST, TEMP_REG_1, 12-1

    ; Set SYNC0 pulse width: n * iep period (pulse width)
    m_set_iep_sync_pwidth_reg  	SPI_IEP_INST, TEMP_REG_1, 0
    m_set_iep_sync_pwidth_reg  	SPI_IEP_INST, TEMP_REG_1, 6

    ; set default increment to 2 ->  500 MHz IEP clock and enable count
    m_set_iep_global_cfg_reg    SPI_IEP_INST, TEMP_REG_1, 0x21
    qba              IDLE

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
    m_pru_clr_pin       START_PIN
    m_pru_set_pin       ADC_CS_PIN
    ; CONFIG_ADC0_SMEM_OFFSET: <No of commands - n>
    ; <Command 1>
    ; ...
    ; <Command n>
    lbco    &R23.w0, C28, CONFIG_ADC0_SMEM_OFFSET, 2
    ldi     R23.w2,   2
    ldi     R24.w0,  0
SEND_COMMAND?:
    lbco    &R24.w2, C28, R23.w2, 2

    m_pru_clr_pin       ADC_CS_PIN
    m_wait_nano_sec     10
    m_send_packet_spi_mode1_msb_iep_sclk   	R24.w2, 16, R3.b0, TEMP_REG_1, 0, SDO_PIN, SYNC0, 3, 3
    m_pru_set_pin       ADC_CS_PIN
    m_wait_nano_sec     10

    add     R24.w0, R24.w0, 1
    add     R23.w2,  R23.w2,  2
    qbne    SEND_COMMAND?, R24.w0, R23.w0

	qba		IDLE

;----------------------------------------------------------------------------
;   SECTION: READ_SAMPLES
;   Starts ADC conversion and then reads the samples using SPI interface
;----------------------------------------------------------------------------
READ_SAMPLES:
    m_pru_set_pin       ADC_CS_PIN
    m_wait_nano_sec     20

    m_pru_set_pin       START_PIN           ; Start conversion

START:
    ; Wait for DRDY to go low indicating conversion has completed
    ; DRDY: _᠆᠆᠆᠆_____________________________________________________
    ; SDI:  _____|  Status  |  < Channel 0 - 4 Data >  |  CRC  |____
    m_wait_high_pulse       2, DRDY_PIN
    ; Fetch samples using serial interface
	m_wait_nano_sec     10
    m_pru_clr_pin       ADC_CS_PIN      ; set CS low
    m_wait_nano_sec     10

	m_read_packet_spi_mode1_msb_iep_sclk       ADC_DATA_REG_1, 24, R5.b0, TEMP_REG_1, 0, SDI_PIN, SYNC0, 15, 3

    m_wait_nano_sec     10
    m_pru_set_pin       ADC_CS_PIN      ; set CS high

    m_sign_ext          24, 32, ADC_DATA_REG_1, EXTEND_0

    ldi                 R1.b0, &ADC_DATA_REG_1
    m_pru_ipc_send      R1.b0, ADC_DATA_REG_1, R0.b0, R29, CONFIG_PRU_IPC0_RX_INTR_ENABLE, CONFIG_PRU_IPC0_RX_EVENT

    m_prgm_flow_jump_on_intr    PRGM_FLOW_EVENT, PRGM_FLOW_EVENT_BIT, 6, TEMP_REG_1, IDLE, IEP, SAMPLE_TRANSFER, ADC_CONFIG, READ_SAMPLES, RESET

    qba    START    ; loop to continue collecting samples
    halt

;----------------------------------------------------------------------------
;   SECTION: RESET
;   Reset some modules before restarting conversion
;----------------------------------------------------------------------------
RESET:
    m_pru_clr_pin       START_PIN           ; Stop conversion
    m_pru_ipc_init      R0.b0, R29, CONFIG_PRU_IPC0_CONFIG_MEM_OFFSET ; Reset Sample Transfer

	qba		IDLE

;----------------------------------------------------------------------------
;   SECTION: READ_ADC_REG
;   Read all ADC registers and write the contents into shared memory
;----------------------------------------------------------------------------
; TODO

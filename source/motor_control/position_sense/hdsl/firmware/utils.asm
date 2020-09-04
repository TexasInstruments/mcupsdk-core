
; Copyright (C) 2021 Texas Instruments Incorporated
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
;   Redistributions of source code must retain the above copyright
;   notice, this list of conditions and the following disclaimer.
;
;   Redistributions in binary form must reproduce the above copyright
;   notice, this list of conditions and the following disclaimer in the
;   documentation and/or other materials provided with the
;   distribution.
;
;   Neither the name of Texas Instruments Incorporated nor the names of
;   its contributors may be used to endorse or promote products derived
;   from this software without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; "AS IS" AND ANY EXPgResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
; OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
; SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
; LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;

	.include "memory.inc"
	.include "defines.inc"
	.include "macros.inc"

	.sect	".text"
	.global update_events
	.global demp_data_symbols
	.global calc_acc_crc
	.global calc_16bit_crc
	.global load_code

;--------------------------------------------------------------------------------------------------
;Function: update_events (RET_ADDR1)
;Updates event register and generates interrupts if necessary
; 9 cycles
;input:
;	REG_FNC.b0: event number
;output:
;modifies:
;--------------------------------------------------------------------------------------------------
update_events:
;read events and masks
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	set		REG_TMP0.w0, REG_TMP0.w0, REG_FNC.b0
	qbbc		update_events_no_int, REG_TMP0.w2, REG_TMP0.w2
;TODO: generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int:
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	RET1
;--------------------------------------------------------------------------------------------------
;Function: load_code (RET_ADDR)
;Loads PRU Code to isntruction memory
;input:
;	REG_TMP0: src offset
;	REG_TMP2: dst offset
;	RG_TMP1: size in bytes
;output:
;modifies:
;--------------------------------------------------------------------------------------------------
	.if !$defined(ICSS_G_V_1_0)   
load_code:
	sbco			&REG_TMP0, MASTER_REGS_CONST, PIMEM_SRC, 12
	ldi			REG_TMP0, 56
	ldi			REG_TMP1, 16
	ldi32			REG_TMP2, (0x00040000+0x00020000)
	ldi			R31.b0, (0x20 | 0x00)
	;reset INTCs
	sbbo			&REG_TMP0, REG_TMP2, INTC_SICR, 4
	sbco			&REG_TMP1, INTC_CONST, INTC_SICR, 4
	lbco			&REG_TMP0, PRU_CTRL_CONST, 0x00, 4
	clr			REG_TMP0, REG_TMP0, 1
	sbco			&REG_TMP0, PRU_CTRL_CONST, 0x00, 4
	RET
	.endif
;--------------------------------------------------------------------------------------------------
;Function: calc_16bit_crc (RET_ADD1)
;This function checks the crc for the acceleration channel
;11*REG_FNC.b2+3 cycles -> 69 cycles for 6 bytes data
;input:
;	REG_FNC.b0: num bytes
;	r1.b0: pointer to register + 1 byte, counting down from there
;output:
;	REG_FNC.w0: 16 bit CRC
;modifies:
;	REG_TMP0, REG_FNC, r1
;--------------------------------------------------------------------------------------------------
calc_16bit_crc:
	ldi		REG_TMP2, (LUT_CRC16+PDMEM00)
	mov		REG_TMP0.b2, REG_FNC.b0
	ldi		REG_FNC.w0, 0
calc_16bit_crc_loop:
	ldi		REG_TMP0.w0, 0
	mvib		REG_TMP0.b0, *--r1.b0
	xor		REG_TMP0.b0, REG_FNC.b1, REG_TMP0.b0
	lsl		REG_TMP0.w0, REG_TMP0.w0, 1
	lbbo		&REG_TMP0.w0, REG_TMP2, REG_TMP0.w0, 2
	lsl		REG_FNC.w0, REG_FNC.w0, 8
	xor		REG_FNC.w0, REG_TMP0.w0, REG_FNC.w0
	sub		REG_TMP0.b2, REG_TMP0.b2, 1
	qblt		calc_16bit_crc_loop, REG_TMP0.b2, 0
;CRC_L is flipped
	xor		REG_FNC.b0, REG_FNC.b0, 0xff
	RET1

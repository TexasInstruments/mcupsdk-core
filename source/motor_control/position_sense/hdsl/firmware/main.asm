
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

	.retain 	; Required for building .out with assembly file
	.retainrefs ; Required for building .out with assembly file
	.ref datalink_init
	.ref transport_init
	.ref load_code
	.ref datalink_init_start
	.global	main
	.if !$defined(ICSS_G_V_1_0)
	.sect	".text: 0x00000000"
	.else
	.sect	".text"
	.endif

main:
;init code
;enable cyclecount
	lbco		&REG_TMP2, PRU_CTRL_CONST, 0x00, 4
	or		REG_TMP2, REG_TMP2, (1<<3)
	sbco		&REG_TMP2, PRU_CTRL_CONST, 0x00, 4
	.if $defined(ICSS_G_V_1_0)
	;skip code loading since there is no edma overlay in ICSSG
	jmp datalink_init_start
	.endif
;--------------------------------------------------------------------------------------------------
;load init code

;copy pseudo instrution memory to L3
	ldi32		REG_TMP2, CODE_BASE
	ldi32		REG_TMP1, 0x2100
	ldi		REG_TMP11, CODE_SIZE*2
load_loop:
	sub		REG_TMP11, REG_TMP11, 4
	lbbo		&REG_TMP0, REG_TMP1, REG_TMP11, 4
	sbbo		&REG_TMP0, REG_TMP2, REG_TMP11, 4
	qbne		load_loop, REG_TMP11, 0


GPIO_4_BA .set					0x48320000
GPIO_CLRDATAOUT .set			0x190
GPIO_SETDATAOUT .set			0x194
GPIO_OE .set					0x134
SWITCH3 .set					0x2
GPIO_DATAIN .set				0x138


	.if !$defined(ICSS_G_V_1_0)   
	LOAD_CODE	CODE_DATALINK_INIT, 0x00, 0x00, CODE_SIZE
	.endif

	jmp datalink_init

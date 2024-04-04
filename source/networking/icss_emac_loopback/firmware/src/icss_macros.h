;
;  TEXAS INSTRUMENTS TEXT FILE LICENSE
;
;   Copyright (c) 2024 Texas Instruments Incorporated
;
;  All rights reserved not granted herein.
;
;  Limited License.
;
;  Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
;  license under copyrights and patents it now or hereafter owns or controls to
;  make, have made, use, import, offer to sell and sell ("Utilize") this software
;  subject to the terms herein.  With respect to the foregoing patent license,
;  such license is granted  solely to the extent that any such patent is necessary
;  to Utilize the software alone.  The patent license shall not apply to any
;  combinations which include this software, other than combinations with devices
;  manufactured by or for TI (“TI Devices”).  No hardware patent is licensed hereunder.
;
;  Redistributions must preserve existing copyright notices and reproduce this license
;  (including the above copyright notice and the disclaimer and (if applicable) source
;  code license limitations below) in the documentation and/or other materials provided
;  with the distribution.
;
;  Redistribution and use in binary form, without modification, are permitted provided
;  that the following conditions are met:
; 	No reverse engineering, decompilation, or disassembly of this software is
;   permitted with respect to any software provided in binary form.
; 	Any redistribution and use are licensed by TI for use only with TI Devices.
; 	Nothing shall obligate TI to provide you with source code for the software
;   licensed and provided to you in object code.
;
;  If software source code is provided to you, modification and redistribution of the
;  source code are permitted provided that the following conditions are met:
; 	Any redistribution and use of the source code, including any resulting derivative
;   works, are licensed by TI for use only with TI Devices.
; 	Any redistribution and use of any object code compiled from the source code
;   and any resulting derivative works, are licensed by TI for use only with TI Devices.
;
;  Neither the name of Texas Instruments Incorporated nor the names of its suppliers
;  may be used to endorse or promote products derived from this software without
;  specific prior written permission.
;
;  DISCLAIMER.
;
;  THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED
;  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
;  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S
;  LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
;  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
;  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
; file:   icss_macros.h
;
; brief:  Implements common macros & defines.
;
;
;  (C) Copyright 2017, Texas Instruments, Inc
;
;

	.if !$defined("__icss_macros_hp")
__icss_macros_hp	.set	1


	;------------------------------------
	; Macros
	;------------------------------------

	;-----------------------------------
	; Macro Name: M_POLL_WORD_RDY
	; Description: Polls for word ready or EOF or RX_ERR.
	; Input Parameters: Label if EOF, Label if RX_ERR
	; Output Parameters: none
	;-----------------------------------
	;** The word ready check is done twice to ensure
	;** minimum delay between a word to be ready and
	;** word ready detection. Also word ready should
	;** be checked before EOF to ensure it has
	;** highest priority to handle scenarios like
	;** RX FIFO having lot of unprocessed words
	;** at the end of the frame.
M_POLL_WORD_RDY	 .macro 	 EOF_LABEL, RX_ERR_LABEL
	;QBBS WORD_RDY, R31.D_WORD_READY_FLAG_BITNUM
WORD_CHECK_EOF?:
	.if $defined("NO_RX_EOF_ERROR_FLAG_BITNUM")
		QBBS EOF_LABEL, R31.D_EOF_FLAG_BITNUM
		QBBS RX_ERR_LABEL, R31.D_RX_ERROR_FLAG_BITNUM
	.else
		QBBS	EOF_LABEL, R31, D_RX_EOF_ERROR_FLAG_BITNUM
	.endif
	QBBC	WORD_CHECK_EOF?, R31, D_WORD_READY_FLAG_BITNUM
WORD_RDY?:
	.endm

	;-----------------------------------
	; Macro Name: M_POLL_BYTE_RDY
	; Description: Polls for byte ready or EOF or RX_ERR
	; Input Parameters: Label if EOF, Label if RX_ERR
	; Output Parameters: none
	;-----------------------------------
	;** The byte ready check is done twice to ensure
	;** minimum delay between a byte to be ready and
	;** byte ready detection. Also byte ready should
	;** be checked before EOF to ensure it has
	;** highest priority to handle scenarios like
	;** RX FIFO having lot of unprocessed bytes
	;** at the end of the frame.
M_POLL_BYTE_RDY	 .macro 		EOF_LABEL, RX_ERR_LABEL
	;QBBS BYTE_RDY, R31.D_BYTE_READY_FLAG_BITNUM
BYTE_CHECK_EOF?:
	.if $defined("NO_RX_EOF_ERROR_FLAG_BITNUM")
			QBBS EOF_LABEL, R31.D_EOF_FLAG_BITNUM
			QBBS RX_ERR_LABEL, R31.D_RX_ERROR_FLAG_BITNUM
	.else
	QBBS	EOF_LABEL, R31, D_RX_EOF_ERROR_FLAG_BITNUM
	.endif
	QBBC	BYTE_CHECK_EOF?, R31, D_BYTE_READY_FLAG_BITNUM
BYTE_RDY?:
	.endm

	;-----------------------------------
	; Macro Name: M_CMD16
	; Description: Issue the given command.
	; Input Parameters: 16bit command value
	; Output Parameters: none
	;-----------------------------------
M_CMD16	 .macro 	 cmd_val16
	LDI	R31.w2 , cmd_val16
	.endm

	;-----------------------------------
	;-----------------------------------
	; Macro Name: M_SET_DATA_MASK16
	; Description: Sets the given mask in R30.w2.
	; Input Parameters: 16bit mask value
	; Output Parameters: none
	;-----------------------------------
M_SET_DATA_MASK16	 .macro 	 mask_val16
	AND R30.w2 , mask_val16 , mask_val16	;Warning: converted from MOV
	.endm
	;-----------------------------------
	;-----------------------------------
	; Macro Name: M_SET_CMD
	; Description: Sets the given command R31.w2.
	; Input Parameters: 16bit mask value
	; Output Parameters: none
	;-----------------------------------
M_SET_CMD	 .macro 	 cmd_val16
	LDI	R31.w2 , cmd_val16
	.endm
	; Macro Name: M_POP_BYTE
	; Description: Pop a byte from RX FIFO.
	; Input Parameters: none
	; Output Parameters: none
	;-----------------------------------
M_POP_BYTE	 .macro
	M_CMD16 D_POP_BYTE_CMD
	.endm

	;-----------------------------------
	; Macro Name: M_POP_WORD
	; Description: Pop a word from RX FIFO.
	; Input Parameters: none
	; Output Parameters: none
	;-----------------------------------
M_POP_WORD	 .macro
	M_CMD16 D_POP_WORD_CMD
	.endm

	;-----------------------------------
	; Macro Name: M_PUSH_BYTE
	; Description: Push a byte into TX fifo.
	; Input Parameters: none
	; Output Parameters: none
	;-----------------------------------
M_PUSH_BYTE	 .macro
	M_CMD16 D_PUSH_BYTE_CMD
	.endm

	;-----------------------------------
	; Macro Name: M_PUSH_WORD
	; Description: Push a word into TX fifo.
	; Input Parameters: none
	; Output Parameters: none
	;-----------------------------------
M_PUSH_WORD_CMD	 .macro
	M_CMD16 D_PUSH_WORD_CMD
	.endm

	;-----------------------------------
	; Macro Name: M_PUSH_N_POP_BYTE
	; Description: Push a byte into TX fifo and pop byte from RX FIFO.
	; Input Parameters: none
	; Output Parameters: none
	;-----------------------------------
M_PUSH_N_POP_BYTE	 .macro
	M_CMD16 D_PUSH_N_POP_BYTE_CMD
	.endm

	;-----------------------------------
	; Macro Name: M_PUSH_N_POP_WORD
	; Description: Push a word into TX fifo and pop word from RX FIFO.
	; Input Parameters: none
	; Output Parameters: none
	;-----------------------------------
M_PUSH_N_POP_WORD	 .macro
	M_CMD16 D_PUSH_N_POP_WORD_CMD
	.endm

	;-----------------------------------
	; Macro Name: M_PUSH_CRC_MSWORD
	; Description: Push 31:16 bits of CRC into TX FIFO.
	; Input Parameters: none
	; Output Parameters: none
	;-----------------------------------
M_PUSH_CRC_MSWORD	 .macro
	M_CMD16 D_PUSH_CRC_MSWORD_CMD
	.endm

	;-----------------------------------
	; Macro Name: M_PUSH_CRC_LSWORD
	; Description: Push 15:0 bits of CRC into TX FIFO.
	; Input Parameters: none
	; Output Parameters: none
	;-----------------------------------
M_PUSH_CRC_LSWORD	 .macro
	M_CMD16 D_PUSH_CRC_LSWORD_CMD
	.endm

	;-----------------------------------
	; Macro Name: M_PUSH_ERR_NIBBLE
	; Description: Push error marker nibble in TX FIFO.
	; Input Parameters: none
	; Output Parameters: none
	;-----------------------------------
M_PUSH_ERR_NIBBLE	 .macro
	M_CMD16 D_PUSH_ERR_NIBBLE_CMD
	.endm

	;-----------------------------------
	; Macro Name: M_PUSH_TX_EOF
	; Description: Push error marker nibble in TX FIFO.
	; Input Parameters: none
	; Output Parameters: none
	;-----------------------------------
M_PUSH_TX_EOF	 .macro
	M_CMD16 D_TX_EOF	;D_PUSH_TX_EOF_CMD
	.endm

	;-----------------------------------
	; Macro Name: M_RESET_RXFIFO
	; Description: Reset RXFIFO
	; Input Parameters: none
	; Output Parameters: none
	;-----------------------------------
M_RESET_RXFIFO	 .macro
	M_CMD16 D_RESET_RXFIFO
	.endm
	;-----------------------------------
	; Macro Name: M_RESET_TXFIFO
	; Description: Reset TXFIFO
	; Input Parameters: none
	; Output Parameters: none
	;-----------------------------------
M_RESET_TXFIFO	 .macro
	M_CMD16 D_RESET_TXFIFO
	.endm

wait	 .macro 	 delay
l1?:	sub delay,delay,1
	QBLT	l1?, delay, 0
	.endm

M_PUSH_WORD	 .macro 	 arg1, arg2
loop?:
    .if !$defined("TX_L2_ENABLED")
	MVIW	TX_DATA_WORD, *arg1
	M_PUSH_WORD_CMD
    .else
	MVIW	R2.w0, *arg1
    XOUT    TX_L2_BANK_ID, &R2, 2
    .endif ;TX_L2_ENABLED
	.if $defined("xmt_debug")
	MVIW	Debug_reg, *arg1
	SBBO	Debug_reg, Debug_base, R12.w0, 2
	ADD		R12.w0, R12.w0, 2
	.endif
	ADD		arg1, arg1, 2
	SUB		arg2, arg2, 1
	QBNE	loop?, arg2, 0
	.endm

M_TX_PORT_CFG	 .macro
	LDI	TEMP_REG_1.b0, 0xb8	; This translates to 920ns( 0xb8 = 184 *5ns) , 40ns is hardware delay, so we get IFG of 960ns
	SBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, ICSS_MIIRT_TXIPG0, 1
	SBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, ICSS_MIIRT_TXIPG1, 1
	LDI		TEMP_REG_1.w2, 0x0040
	LDI		TEMP_REG_1.w0, 0x0003
    .if    $defined("TWO_PORT_CFG")
    SBCO	&TEMP_REG_1, MII_RT_CFG_CONST, TX_ENABLE_CTRL_PORT2, 4
    LDI		TEMP_REG_1.w0, 0x0103
    SBCO	&TEMP_REG_1, MII_RT_CFG_CONST, TX_ENABLE_CTRL_PORT1, 4
    .else
	SBCO	&TEMP_REG_1, MII_RT_CFG_CONST, TX_ENABLE_CTRL_PORT1, 4
	LDI		TEMP_REG_1.w0, 0x0103
	SBCO	&TEMP_REG_1, MII_RT_CFG_CONST, TX_ENABLE_CTRL_PORT2, 4

    .endif
	.endm

M_TX_EN	 .macro
    .if    $defined("TWO_PORT_CFG")
    .if    $defined("PRU0")
    LDI     TEMP_REG_1.b0, 0x03
    SBCO 	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TX_ENABLE_CTRL_PORT2, #1
    .else
    LDI     TEMP_REG_1.b0, 0x03
    SBCO 	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TX_ENABLE_CTRL_PORT1, #1
    .endif  ;PRU0
    .else
	.if $defined("PRU0")
	LDI		TEMP_REG_1.b0, 0x03
	SBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TX_ENABLE_CTRL_PORT1, 1
    .else
	LDI		TEMP_REG_1.b0, 0x03
	SBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TX_ENABLE_CTRL_PORT2, 1
	.endif  ;PRU0

    .endif ;TWO_PORT_CFG
	.endm

M_TX_DISABLE	 .macro
    .if    $defined("TWO_PORT_CFG")
    .if    $defined("PRU0")
    LDI  	TEMP_REG_1.b0, 0x02
    SBCO 	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TX_ENABLE_CTRL_PORT2, #1
    .else
    LDI  	TEMP_REG_1.b0, 0x02
    SBCO 	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TX_ENABLE_CTRL_PORT1, #1
    .endif  ;PRU0
    .else
	.if $defined("PRU0")
	LDI		TEMP_REG_1.b0, 0x02
	SBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TX_ENABLE_CTRL_PORT1, 1
	.else
	LDI		TEMP_REG_1.b0, 0x02
	SBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TX_ENABLE_CTRL_PORT2, 1
	.endif  ;PRU0

    .endif ;TWO_PORT_CFG
	.endm

M_TX_RESET	 .macro
	SET	R31 , R31 , 30
	.endm

M_PUSH_8	 .macro
	SET	R31 , R31 , 24
	.endm
	; DIVU5 : integer divide by 5
; n is in arg1, result is in arg3, arg2 has remainder 1..4 if not modulo 5
; arg2 and arg3 are registers used, arg1 remains unchanged
; cycles: 14/15  (70/75ns)
; instructions: 14/15
DIVU5	 .macro 	 arg1, arg2, arg3
	; q = (n>>1) + (n>>2)
	LSR     arg2, arg1, 1
	LSR     arg3, arg1, 2
	ADD     arg2, arg2, arg3
	; q = q + (q>>4)
	LSR     arg3, arg2, 4
	ADD		arg2, arg2, arg3
	; q = q + (q>>8)
	LSR     arg3, arg2, 8
	ADD		arg2, arg2, arg3
	; q = q + (q>>16)
	LSR     arg3, arg2, 16
	ADD		arg2, arg2, arg3
	; q = q >> 2
	LSR		arg3, arg2, 2
	; r = n - (q << 2) + q
	; arg2 has already q << 2, but LSB is not correct
	LSL 	arg2, arg3, 2
	ADD  	arg2, arg2 , arg3
	SUB     arg2, arg1, arg2
	; r is in arg2 and q in arg3
	; if arg2 >= 5 then q++
	QBGT	L_DIVU5?, arg2, 5
	ADD     arg3, arg3, 1
L_DIVU5?:
	.endm

	.endif	;__icss_macros_hp

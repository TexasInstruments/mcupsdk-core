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
; file:   emac_MII_Xmt.h
;
; brief:  Defines and macros to be used by Xmt task.
;
;
;  (C) Copyright 2017, Texas Instruments, Inc
;
;

	.if !$defined("__mii_xmt_hp")
__mii_xmt_hp	.set	1

	.include "icss_intc_regs.h"

MII_CONFIG    		.set	0x4a310000        ; pointer to the start of MII_CONFIG data
host_if_mac    		.set	0x10        ; start address of MAC address
	.asg	R2, BUFFER
buffer_ptr			.set	0x8
	.asg	R11.w0, loop_cnt

	.if $defined("xmt_debug")
#define Debug_base			R11
#define debug_offset 		R12.w0
#define Debug_reg			R12.w2
	.endif

    .if    $defined("TWO_PORT_CFG")
QUEUE_DESP_BASE    	.set	PRU1_DMEM_CONST                ; buffer need to be in consecutive register space
    .else
QUEUE_DESP_BASE    	.set	ICSS_SHARED_CONST                ; buffer need to be in consecutive register space
    .endif ;TWO_PORT_CFG
	.asg	R1.b3, TX_DATA_POINTER

; MII transmit structure definition   -- TBD
;.struct MII_TX_DESC

;.ends

;;//////saved in scratch pad bank id 11 for context saving//////////
	.asg	R13, MII_TX_CONTEXT
	.asg	R13.t0, PACKET_FROM_HOST			;1: from host, 0: from port 1
;	.asg	R13.t1, INCREMENT_WRK_BUFFER_DESC_OFFSET
INCREMENT_WRK_BUFFER_DESC_OFFSET			.set 	1
;	.asg	R13.t2, PACKET_FROM_COLL_QUEUE
PACKET_FROM_COLL_QUEUE						.set 	2
	.asg	R13.b1, BYTES_TRANSFERRED_IN_LAST_CALL
	.asg	R13.w2, QUEUE_DESC_OFFSET
	.asg	R14.w0, BYTE_CNT			;cnt for number of byte transferred in the packet
	.asg	R14.w2, Packet_Length			;length of the packet
	.asg	R15.w0, BUFFER_DESC_OFFSET
	.asg	R15.w2, BUFFER_INDEX
	.asg	R16.w0, BUFFER_OFFSET
	.asg	R16.w2, TOP_MOST_BUFFER_INDEX
	.asg	R17.w0, BASE_BUFFER_DESC_OFFSET
	.asg	R17.w2, TOP_MOST_BUFFER_DESC_OFFSET			; offset of the top most buffer desc for the selected queue
;;/////////////////////////////////////////////////////////

	.if $defined("ICSS_DUAL_EMAC_BUILD")
	.asg	R4.w0, TX_CONTEXT_OFFSET
	.endif

	.if $defined("ICSS_SWITCH_BUILD")
	.asg    R28.w0, TX_CONTEXT_OFFSET
	.endif

struct_buffer_desc .struct
x_length        .ushort
pkt_length      .ushort
        .endstruct

BUFFER_DESC_REG     .sassign    R11, struct_buffer_desc

        .asg	R10.w0, block_size
	.asg	R10.b2, FREE_SPACE_IN_FIFO
	.asg	R10.b3, size

	.asg	R30.b0, TX_DATA_BYTE
	.asg	R30.w0, TX_DATA_WORD
	.asg	R30, TX_DATA_DOUBLE_WORD
	.asg	R30.w2, TX_DATA_WORD_MASK

QUEUE_DESC_REG	.sassign	 R2,    struct_queue

;-----------------------------------
; Macro Name: M_XMT_RX_EOF_CHECK_ICSS_REV1
; Description: Check for RX EOF on ICSS_REV1.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_XMT_RX_EOF_CHECK_ICSS_REV1	 .macro 		EOF_LABEL
	.if $defined("PRU0")
	QBBS	EOF_LABEL, R31, 30
	.else
	QBBS	EOF_LABEL, R31, 31
	.endif
	.endm

;-----------------------------------
; Macro Name: M_XMT_RX_EOF_CHECK_ICSS_REV2
; Description: Check for RX EOF on ICSS_REV2.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_XMT_RX_EOF_CHECK_ICSS_REV2	 .macro 		EOF_LABEL
	QBBS	EOF_LABEL, R31, 20
	.endm

;-----------------------------------
; Macro Name: M_XMT_GET_TXSOF_ICSS_REV1
; Description: Get TX SOF on ICSS_REV1 for fill level calculation.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_XMT_GET_TXSOF_ICSS_REV1	 .macro
	; Check if there is enough space in TX FIFO. To avoid TX Overflow.
	; Port TX SOF capture for port 1 (PRU1) and port 2 (PRU0) - 2 port config.
	.if $defined("PRU0")
	LBCO	&TEMP_REG_3, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 4
	.else
	LBCO	&TEMP_REG_3, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 4
	.endif
	.endm

;-----------------------------------
; Macro Name: M_XMT_FILL_LEVEL_CALC_ICSS_REV1
; Description: Fill level calculation on ICSS_REV1.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_XMT_FILL_LEVEL_CALC_ICSS_REV1	 .macro
    LBCO	&TEMP_REG_1, IEP_CONST, IEP_COUNTER_OFFSET, 4
    QBGE	iep_wrapped?, TEMP_REG_1, TEMP_REG_3	; branch if current time is less than SOF time stamp
    SUB	TEMP_REG_1, TEMP_REG_1, TEMP_REG_3
    JMP	NO_IEP_WRAP?
iep_wrapped?:
    .if $defined("TTS")
    LDI	TEMP_REG_4.w0 , ICSS_EMAC_TTS_IEP_MAX_VAL & 0xFFFF
    LDI	TEMP_REG_4.w2 , ICSS_EMAC_TTS_IEP_MAX_VAL >> 16
    .else
    .if $defined("TWO_PORT_CFG")
;check compare enable and see if wraparound is enabled
    LBCO	&TEMP_REG_2, IEP_CONST, IEP_CMP_CFG_REG, 1
    QBBC	IEP_CMP0_NOT_ENABLED?, TEMP_REG_2, 1
    LBCO	&TEMP_REG_4, IEP_CONST, IEP_CMP0_REG, 4
    QBA     ADJUST_FOR_WRAPAROUND?
IEP_CMP0_NOT_ENABLED?:
    .endif
    LDI	TEMP_REG_4.w0 , 0xffffffff & 0xFFFF
    LDI	TEMP_REG_4.w2 , 0xffffffff >> 16
    .endif	;TTS
ADJUST_FOR_WRAPAROUND?:
    SUB	TEMP_REG_4, TEMP_REG_4, TEMP_REG_3	; total bytes sent is time stamp till end of IEP counter + current time
    ADD	TEMP_REG_1, TEMP_REG_4, TEMP_REG_1
NO_IEP_WRAP?:
    LSR	TEMP_REG_1, TEMP_REG_1, 4	;	Divide by 16
    DIVU5	TEMP_REG_1, TEMP_REG_2, TEMP_REG_3	;	Again divide it by 5 .. divided by 80ns for 100 Mbps mode
    QBBS	XMT_100Mbps_Mode?, R23, 7
    LSR	TEMP_REG_3, TEMP_REG_3, 1	; Divide by 2
    AND TEMP_REG_1 , TEMP_REG_3 , TEMP_REG_3
    DIVU5	TEMP_REG_1, TEMP_REG_2, TEMP_REG_3	; Divide by 5  .. divided by 800ns for 10 Mbps mode
XMT_100Mbps_Mode?:
    ADD	TEMP_REG_4, BYTE_CNT, 8
    SUB	TEMP_REG_1, TEMP_REG_4, TEMP_REG_3	; = Fill level
    QBLT	error_in_fill_level, TEMP_REG_1.b0, 64	; Overflow situation. Reset FIFO and stop further transmission.
    RSB	FREE_SPACE_IN_FIFO, TEMP_REG_1.b0, 64	; Compute the free space in Tx Fifo
    .endm

;-----------------------------------
; Macro Name: M_XMT_FILL_LEVEL_CALC_ICSS_REV2
; Description: Fill level calculation on ICSS_REV2.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_XMT_FILL_LEVEL_CALC_ICSS_REV2	 .macro
	.if    $defined("TWO_PORT_CFG")
	.if    $defined("PRU0")
	LBCO    &TEMP_REG_1.b0, MII_RT_CFG_CONST, TX_FIFO_FILL_LEVEL_PORT2, 4 ; = Fill level
	.else
	LBCO    &TEMP_REG_1.b0, MII_RT_CFG_CONST, TX_FIFO_FILL_LEVEL_PORT1, 4
	.endif
    .else
    .if $defined("PRU0")
	LBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TX_FIFO_FILL_LEVEL_PORT1, 4	; = Fill level
	.else
	LBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TX_FIFO_FILL_LEVEL_PORT2, 4
	.endif
	.endif
	LSR	TEMP_REG_1, TEMP_REG_1, 1	;Divide the fill level by 2 to convert it in bytes
	.if $defined("TX_L2_ENABLED")
    AND     TEMP_REG_2, R19, R19
    XIN     TX_L2_BANK_ID, &R19, 4
    AND     TEMP_REG_1.b0, R19.b2, R19.b2
    AND     R19, TEMP_REG_2, TEMP_REG_2
    .endif
	.if $defined("ICSS_DUAL_EMAC_BUILD")
	M_XMT_UNDER_OVER_FLOW_CHECK_ICSS_REV2
	.endif
    .if $defined("TX_L2_ENABLED")
	RSB	FREE_SPACE_IN_FIFO, TEMP_REG_1.b0, 60	;Compute the free space in Tx Fifo
    .else
	RSB	FREE_SPACE_IN_FIFO, TEMP_REG_1.b0, 96	;Compute the free space in Tx Fifo
    .endif
	.endm

;-----------------------------------
; Macro Name: M_XMT_INSERT_CRC_ICSS_REV1
; Description: Insert the CRC in outgoing frame on ICSS_REV1.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_XMT_INSERT_CRC_ICSS_REV1	 .macro
	M_PUSH_CRC_MSWORD
	LDI	TEMP_REG_4.b2, 4
	WAIT	R3.b2
	M_PUSH_CRC_LSWORD
	.endm

;-----------------------------------
; Macro Name: M_XMT_INSERT_CRC_ICSS_REV2
; Description: Insert the CRC in outgoing frame on ICSS_REV2.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_XMT_INSERT_CRC_ICSS_REV2	 .macro
	LDI	R31.w2 , 0x2C00
	.endm

;-----------------------------------
; Macro Name: M_XMT_UNDER_OVER_FLOW_CHECK_ICSS_REV2
; Description: Check underflow and overflow flags on ICSS_REV2.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_XMT_UNDER_OVER_FLOW_CHECK_ICSS_REV2	 .macro
	QBGE	CHECK_UNDERFLOW?, TEMP_REG_1.b0, 96
	;Overflow has occured. Increment counter and reset the FIFO.
	;We cannot use INTC because of harware issue.
	LDI	TEMP_REG_2.w0 , TX_OVERFLOW_COUNTER
	LBCO	&TEMP_REG_3, PRU_DMEM_ADDR, TEMP_REG_2.w0, 4
	ADD	TEMP_REG_3, TEMP_REG_3, 1
	SBCO	&TEMP_REG_3, PRU_DMEM_ADDR, TEMP_REG_2.w0, 4
	JMP		error_in_fill_level
CHECK_UNDERFLOW?:
	LDI	TEMP_REG_2.w0 , ICSS_INTC_SECR1
	LBCO	&TEMP_REG_3, ICSS_INTC_CONST, TEMP_REG_2.w0, 4
	.if $defined("PRU0")
	QBBC	NO_UNDERFLOW?, TEMP_REG_3, 7
	.else
	QBBC	NO_UNDERFLOW?, TEMP_REG_3, 19
	.endif
	;Underflow bit is set, clear the error and reset the FIFO.
	LDI	TEMP_REG_3 , 0
	.if $defined("PRU0")
	SET	TEMP_REG_3 , TEMP_REG_3 , 7
	.else
	SET	TEMP_REG_3 , TEMP_REG_3 , 19
	.endif
	SBCO	&TEMP_REG_3, ICSS_INTC_CONST, TEMP_REG_2.w0, 4
	LDI	TEMP_REG_2.w0 , TX_UNDERFLOW_COUNTER
	LBCO	&TEMP_REG_3, PRU_DMEM_ADDR, TEMP_REG_2.w0, 4
	ADD	TEMP_REG_3, TEMP_REG_3, 1
	SBCO	&TEMP_REG_3, PRU_DMEM_ADDR, TEMP_REG_2.w0, 4
	JMP		error_in_fill_level
NO_UNDERFLOW?:
	.endm

	.endif	;__mii_xmt_hp

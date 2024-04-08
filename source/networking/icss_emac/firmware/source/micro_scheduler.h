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
; file:   micro_scheduler.h
;
; brief:  ICSS defines and macros used by Micro_Scheduler
;
;
;  (C) Copyright 2017, Texas Instruments, Inc
;
;

	.if !$defined("__ICSS_MICRO_SCHEDULER_H__")
__ICSS_MICRO_SCHEDULER_H__	.set	1

; mov32 : Move a 32bit value to a register
MOV32    .macro    arg1, arg2
	ldi     arg1.w0, (arg2 & 0xFFFF)
	ldi     arg1.w2, (arg2 >> 16)
	.endm

	.if $defined("ICSS_DUAL_EMAC_BUILD")
	.include "icss_emacSwitch.h"
	.endif
	.if $defined("ICSS_SWITCH_BUILD")
        .include "icss_switch.h"
	.endif
	.include "icss_defines.h"

	.asg	R0, TEMP_REG_1
	.asg	R4, TEMP_REG_2
	.asg	R2, TEMP_REG_3
	.asg	R3, TEMP_REG_4

MS_OVERHEAD	.set	0x17
	.asg	R19, TASK_TABLE_ROW0

;R22 bit usage. This is reserved usage, cannot be used for other purposes
PACKET_TX_ALLOWED   		.set    31
Entire_Tx_Data_Not_Pushed	.set	30
RX_LB_PARTIALLY_PROCESSED	.set	29
RX_BC_FRAME					.set	28
RX_MC_FRAME					.set	27
TX_BC_FRAME					.set	26
TX_MC_FRAME					.set	25
RX_FWD_FLAG					.set	24

	.if $defined("HALF_DUPLEX_ENABLED")
PORT_IS_HALF_DUPLEX			.set    23
TX_LATE_COLLISION			.set	22
	.endif

OPPOSITE_PORT_LINK_UP			.set    10
	.if $defined("TTS")
TTS_ENABLE					.set	21
TTS_FIRST_SETUP				.set	20
TTS_RT_QUEUE_PKT			.set	19
TTS_FIRST_PKT_done			.set	18
TTS_FIRST_PKT_since_enable	.set	17
TTS_CMP_3456_FIRST_Exception	.set	16
TTS_CMP5_CMP6_in_next_IEP_cycle	.set	15
	.endif	;TTS
INTR_TO_HOST_PENDING_RX             .set     5         ;R22.t5     ; set if selected queue is empty
HOST_QUEUE_EMPTY_STATUS             .set     4         ;R22.t4     ; set if interrupt to host is pending in receive task

RX_TASK_POINTER				.set	0x4c
TX_TASK_POINTER				.set	0x4d
BG_TASK_POINTER				.set	0x4e

ECAP_BASE_ADDR				.set	0x00030000
ICSS_RAM_BASE				.set	0x00010000

PERIOD_COUNT_OFFSET			.set	0x0
RX_COUNT_OFFSET				.set	0x4
TX_COUNT_OFFSET				.set	0x8
BRIDGE_COUNT_OFFSET			.set	0xc
STAT_COUNT_OFFSET			.set	0x10
HOST_COUNT_OFFSET			.set	0x14

ECAP_CNT					.set	0x00
ECAP_CMP1					.set	0x08
ECAP_ECCTL2					.set	0x2a
ECAP_ECEINT					.set	0x2c
ECAP_ECFLG					.set	0x2e
ECAP_ECCLR					.set	0x30
RX_FB_THRESHOLD				.set	0x0E
THRESHOLD_TASKS_SCAN		.set	0x1E

INTC_BASE					.set	0x00020000

	.asg	R1.b2, CURRENT_TASK_POINTER

PRU_CONTROL_REG				.set	0x00
PRU_CYCLE_COUNT_REG			.set	0x0c
PRU_CONTROL_REGS			.set	0x00022000

IEP_GLOBAL_CFG_REG			.set	0x00
IEP_COMPEN_REG				.set	0x08

	.if $defined("ICSS_REV1")
IEP_COUNTER_OFFSET				.set	0x0c
CAP_RISE_RX_SOF_PORT1_OFFSET	.set	0x18
CAP_RISE_RX_SFD_PORT1_OFFSET	.set	0x1c
CAP_RISE_RX_SOF_PORT2_OFFSET	.set	0x20
CAP_RISE_RX_SFD_PORT2_OFFSET	.set	0x24
CAP_RISE_TX_SOF_PORT1_OFFSET	.set	0x28
CAP_RISE_TX_SOF_PORT2_OFFSET	.set	0x2c
IEP_CMP_CFG_REG					.set	0x40
IEP_CMP_STATUS_REG				.set	0x44
IEP_CMP0_REG					.set	0x48
IEP_CMP1_REG					.set	0x4c
IEP_CMP2_REG					.set	0x50
IEP_CMP6_REG					.set	0x60
IEP_CMP7_REG					.set	0x64

IEP_SYNC_CTRL_REG				.set	0x100
IEP_SYNC_PWIDTH_REG				.set	0x110
	.endif	;ICSS_REV1

	.if $defined("ICSS_REV2")
IEP_COUNTER_OFFSET				.set	0x10
CAP_RISE_RX_SOF_PORT1_OFFSET	.set	0x20
CAP_RISE_RX_SFD_PORT1_OFFSET	.set	0x28
CAP_RISE_RX_SOF_PORT2_OFFSET	.set	0x30
CAP_RISE_RX_SFD_PORT2_OFFSET	.set	0x38
	.if $defined("MII_TX_PIN_SWAP")
CAP_RISE_TX_SOF_PORT1_OFFSET	.set	0x48
CAP_RISE_TX_SOF_PORT2_OFFSET	.set	0x40
TX_FIFO_FILL_LEVEL_PORT1		.set	0x6C
TX_FIFO_FILL_LEVEL_PORT2		.set	0x68
TX_ENABLE_CTRL_PORT1			.set 	0x14
TX_ENABLE_CTRL_PORT2			.set 	0x10
	.else
CAP_RISE_TX_SOF_PORT1_OFFSET	.set	0x40
CAP_RISE_TX_SOF_PORT2_OFFSET	.set	0x48
TX_FIFO_FILL_LEVEL_PORT1		.set	0x68
TX_FIFO_FILL_LEVEL_PORT2		.set	0x6C
TX_ENABLE_CTRL_PORT1			.set 	0x10
TX_ENABLE_CTRL_PORT2			.set 	0x14
	.endif ;MII_TX_PIN_SWAP
IEP_CMP_CFG_REG					.set	0x70
IEP_CMP_STATUS_REG				.set	0x74
IEP_CMP0_REG					.set	0x78
IEP_CMP1_REG					.set	0x80
IEP_CMP2_REG					.set	0x88
IEP_CMP6_REG					.set	0xa8
IEP_CMP7_REG					.set	0xb0
IEP_SYNC_CTRL_REG    			.set    0x180

	.endif	;ICSS_REV2

CSL_ICSSIEP_PDI_WD_TIM_REG		.set	0x204
CSL_ICSSIEP_PD_WD_TIM_REG		.set	0x208
CSL_ICSSIEP_WD_STATUS_REG		.set	0x20C
CSL_ICSSIEP_WD_EXP_CNT_REG		.set	0x210
CSL_ICSSIEP_WD_CTRL_REG			.set	0x214
;-----------------------------------
; Macro Name: M_MS_TEF_ICSS_REV1
; Description: Set of instructions to execute when Task Execution is Finished (TEF) for ICSS_REV1.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_MS_TEF_ICSS_REV1	 .macro
	;Check whether T(EOF) has come
	LDI	TEMP_REG_1.w0, 0x0204
	LBCO	&TEMP_REG_2, ICSS_INTC_CONST, TEMP_REG_1.w0, 4
	.if $defined("TWO_PORT_CFG")
	.if $defined("PRU0")
	QBBC	LOOP_OVER_TASKS, TEMP_REG_2, 19
	.else
	QBBC	LOOP_OVER_TASKS, TEMP_REG_2, 7
	.endif
	.else
	.if $defined("PRU0")
	QBBC	LOOP_OVER_TASKS, TEMP_REG_2, 7
	.else
	QBBC	LOOP_OVER_TASKS, TEMP_REG_2, 19
	.endif
	.endif ;TWO_PORT_CFG

	M_TX_RESET
	M_PUSH_TX_EOF

	;Clear the Underflow error
	.if $defined("TWO_PORT_CFG")
	.if $defined("PRU0")
	LDI		TEMP_REG_4.w0, 0x0284
	LDI32	TEMP_REG_3, 0x00080000
	SBCO	&TEMP_REG_3, ICSS_INTC_CONST, TEMP_REG_4.w0, 4
	.else
	LDI		TEMP_REG_4.w0, 0x0284
	LDI		TEMP_REG_3, 0x00000080
	SBCO	&TEMP_REG_3, ICSS_INTC_CONST, TEMP_REG_4.w0, 4
	.endif
	.else
	.if $defined("PRU0")
	LDI		TEMP_REG_4.w0, 0x0284
	LDI		TEMP_REG_3 , 0x00000080
	SBCO	&TEMP_REG_3, ICSS_INTC_CONST, TEMP_REG_4.w0, 4
	.else
	LDI		TEMP_REG_4.w0, 0x0284
	LDI32	TEMP_REG_3 , 0x00080000
	SBCO	&TEMP_REG_3, ICSS_INTC_CONST, TEMP_REG_4.w0, 4
	.endif
	.endif ;TWO_PORT_CFG

	.if $defined("ICSS_DUAL_EMAC_BUILD")
	;Raise Tx Completion interrupt to Host
	;Two interrupts for two ports in MAC mode.
	.if $defined("PRU0")
	LDI	R31, 0x26	;Maps to system event 22, 0x26 = 100110
	.else
	LDI	R31, 0x27	;Maps to system event 23, 0x27 = 100111
	.endif
	.endif
	.endm

;-----------------------------------
; Macro Name: M_MS_TEF_ICSS_REV2
; Description: Set of instructions to execute when Task Execution is Finished (TEF) for ICSS_REV2.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_MS_TEF_ICSS_REV2	 .macro
	;Check whether T(EOF) has come
	QBBC	LOOP_OVER_TASKS, R31, 16

	.if $defined("ICSS_DUAL_EMAC_BUILD")
	; Raise Tx Completion Interrupt to host
	;Two interrupts for two ports in MAC mode.
	.if $defined("PRU0")
	LDI	R31, 0x26	; Maps to system event 22, 0x26 = 100110
	.else
	LDI	R31, 0x27	; Maps to system event 23, 0x27 = 100111
	.endif
	.endif
	.endm

;----------------------------------------------------
; Macro Name: M_MS_RX_EOF_CHECK_ICSS_REV1
; Description: Check for RX EOF on ICSS_REV1.
; Input Parameters: none
; Output Parameters: none
;
; NOTE: The TRM indicates Bit 20 of R31 for RX_EOF.
; But we can not use Bit 20 for this ICSS
; revision as On AM335x and AM437x, RX_EOF
; is auto clear when a new frame arrives
; in RXL2 mode. Hence Bit 20 can not be used
;----------------------------------------------------
M_MS_RX_EOF_CHECK_ICSS_REV1	 .macro
	.if $defined("PRU0")
	QBBC	Check_RCV_Active, R31, 30
	.else
	QBBC	Check_RCV_Active, R31, 31
	.endif
	.endm

;-----------------------------------
; Macro Name: M_MS_RX_EOF_CHECK_ICSS_REV2
; Description: Check for RX EOF on ICSS_REV2.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_MS_RX_EOF_CHECK_ICSS_REV2	 .macro
	QBBC	Check_RCV_Active, R31, 20
	.endm

	.endif ;__ICSS_MICRO_SCHEDULER_H__

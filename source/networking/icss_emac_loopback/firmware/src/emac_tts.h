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
; file:   emac_tts.h
;
; brief:  ICSS EMAC time triggered send
;         Includes:
;         1. TTS Macros
;		   2. TTS Defines
;
;
;  (C) Copyright 2017, Texas Instruments, Inc
;
;

	.if !$defined("__emac_tts_hp")
__emac_tts_hp	.set	1

	.if $defined("TTS")

;----------------------- Time Triggered Send Defines ------------------------;
; TTS Status Bits
	.asg	t0, ICSS_EMAC_TTS_PRU_ENABLE
	.asg	t1, ICSS_EMAC_TTS_MISSED_CYCLE
	.asg	t2, ICSS_EMAC_TTS_INSERT_CYC_FRAME_EVENT
	.asg	t3, ICSS_EMAC_TTS_CYC_TX_SOF_ENABLE
	.asg	t4, ICSS_EMAC_TTS_CYC_INTERRUPT_ENABLE
	.asg	t5, ICSS_EMAC_TTS_CMP0_CMP7_SETUP

;TTS Registers
	.asg	R20, TTS_CRITICAL_IEP_REG_PRU0
	.asg	R21, TTS_CRITICAL_IEP_REG_PRU1

;TTS Cyclic Interrupt
TTS_PRU0_CYC_INTERRUPT_EVENT_ID	.set				(0x28)
TTS_PRU1_CYC_INTERRUPT_EVENT_ID	.set				(0x29)

;TTS Compare Registers Bit Masks
TTS_CMP7_ENABLE	.set								(0x01)
TTS_CMP0_ENABLE_IEP_WRAP	.set					(0x01)
TTS_CMP0_ENABLE	.set								(TTS_CMP0_ENABLE_IEP_WRAP << 1)
TTS_CMP3_ENABLE	.set								(TTS_CMP0_ENABLE << 3)
TTS_CMP4_ENABLE	.set								(TTS_CMP3_ENABLE << 1)
TTS_CMP5_ENABLE	.set								(TTS_CMP4_ENABLE << 1)
TTS_CMP6_ENABLE	.set								(TTS_CMP5_ENABLE << 1)
TTS_CMP7_DISABLE	.set							(TTS_CMP7_ENABLE ^ 0xFF)
TTS_CMP6_DISABLE	.set							(TTS_CMP6_ENABLE ^ 0xFF)
TTS_CMP5_DISABLE	.set							(TTS_CMP5_ENABLE ^ 0xFF)
TTS_CMP4_DISABLE	.set							(TTS_CMP4_ENABLE ^ 0xFF)
TTS_CMP3_DISABLE	.set							(TTS_CMP3_ENABLE ^ 0xFF)
TTS_CMP0_CLEAR_STATUS	.set						(0x01)
TTS_CMP3_CLEAR_STATUS	.set						(TTS_CMP0_CLEAR_STATUS << 3)
TTS_CMP4_CLEAR_STATUS	.set						(TTS_CMP3_CLEAR_STATUS << 1)
TTS_CMP5_CLEAR_STATUS	.set						(TTS_CMP4_CLEAR_STATUS << 1)
TTS_CMP6_CLEAR_STATUS	.set						(TTS_CMP5_CLEAR_STATUS << 1)
TTS_CMP7_CLEAR_STATUS	.set						(TTS_CMP6_CLEAR_STATUS << 1)
TTS_R22_CLEAR_BITS_b1	.set						(0x7F)
TTS_R22_CLEAR_BITS_b2	.set						(0xC0)
;;////////////////////////////////////////////////////////////////////////////
;------------------- End of Time Triggered Send Defines ---------------------;
;;////////////////////////////////////////////////////////////////////////////

;----------------------- Time Triggered Send Macros ------------------------;

;------------------------------------------------------------------------------
; Macro Name: M_TTS_XMT_SCHEDULER
; Description: Checks which queue has the packet and sets appropriate bits.
; Input Parameters: R4.w0/TX_CONTEXT_OFFSET
; Output Parameters: none
;------------------------------------------------------------------------------
M_TTS_XMT_SCHEDULER	 .macro 		tx_context_offset
	QBBC	TTS_disabled_XMT_Scheduler?, R22 , TTS_ENABLE
	LDI	R21.w2 , Q1_TX_CONTEXT_OFFSET;Check if current packet is from Queue 1.
	QBNE	NOT_Queue1?, tx_context_offset, R21.w2	;R4.w0 = tx_context_offset

	;Need to check one condition before transmitting RT packet.
	;If a packet has already been transmitted in this cycle,
	;we do not transmit RT packet.
	QBBS	CHECK_NEXT_QUEUE, R22 , TTS_FIRST_PKT_done

	;Since the above condition is not true, check size.
	SET	R22 , R22 , TTS_RT_QUEUE_PKT

	;Found cyclic frame in queue.
	;Clear ICSS_EMAC_TTS_INSERT_CYC_FRAME_EVENT.
	LDI	R21.w2 , ICSS_EMAC_TTS_STATUS_OFFSET
	LBCO	&R21.b0, PRU_DMEM_ADDR, R21.w2, 1
	CLR	R21 , R21 , 2
	SBCO	&R21.b0, PRU_DMEM_ADDR, R21.w2, 1
	.if $defined("ICSS_REV1")
	JMP 	FN_TTS_PKT_SIZE_CHECK_ICSS_REV1
	.endif
	.if $defined("ICSS_REV2")
	JMP 	FN_TTS_PKT_SIZE_CHECK_ICSS_REV2
	.endif

NOT_Queue1?:
	;Packet not available in Queue 1; available in Queue 2/3/4.
	;Check if size is allowed.
	CLR	R22 , R22 , TTS_RT_QUEUE_PKT
	.if $defined("ICSS_REV1")
	JMP 	FN_TTS_PKT_SIZE_CHECK_ICSS_REV1
	.endif
	.if $defined("ICSS_REV2")
	JMP 	FN_TTS_PKT_SIZE_CHECK_ICSS_REV2
	.endif

TTS_disabled_XMT_Scheduler?:
	.endm

;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_TTS_MISSED_CYCLE_CHECK
; Description: Checks if cycle was missed by RT frame and updates missed_cycle counter.
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_TTS_MISSED_CYCLE_CHECK	 .macro
	QBBC	TTS_Not_Missed_Cycle?, R22 , TTS_FIRST_PKT_since_enable
	QBBS	TTS_Not_Missed_Cycle?, R22 , TTS_FIRST_PKT_done
	LDI	R20.w0 , ICSS_EMAC_TTS_STATUS_OFFSET
	LBCO	&R20.b2, PRU_DMEM_ADDR, R20.w0, 1
	SET	R20 , R20 , 17 ;Set Missed_Cycle bit in TTS_STATUS
	SBCO	&R20.b2, PRU_DMEM_ADDR, R20.w0, 1

	LDI	R20.w0 , ICSS_EMAC_TTS_MISSED_CYCLE_CNT_OFFSET
	LBCO	&R21, PRU_DMEM_ADDR, R20.w0, 4	;Update Missed_Cycle Counter.
	ADD	R21, R21, 1
	SBCO	&R21, PRU_DMEM_ADDR, R20.w0, 4
TTS_Not_Missed_Cycle?:
	.endm

;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_CHECK_TTS_ENABLE
; Description: Check if time triggered send has been enabled by the host.
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_CHECK_TTS_ENABLE	 .macro
	LDI	R20.w0 , ICSS_EMAC_TTS_STATUS_OFFSET
	LBCO	&R20.b2, PRU_DMEM_ADDR, R20.w0, 1
;	QBBC	FN_TTS_IEP_CFG_CLEAR, R20, 16
	QBBS	FN_TTS_IEP_CFG_SET?, R20, 16
        JMP     FN_TTS_IEP_CFG_CLEAR
FN_TTS_IEP_CFG_SET?:
	SET	R22 , R22 , TTS_ENABLE

	.if $defined("ICSS_REV1")
	;Setup CMP7(PRU1) event to check IEP wraparound.
	;CMP0(PRU0) setup done in microscheduler.
	;Only need to clear CMP0 status for first time.
	QBBS	CHECK_TTS_ENABLE_done?, R20, 21
	SET	R20 , R20 , 21 ;Set ICSS_EMAC_TTS_CMP0_CMP7_SETUP bit
	SBCO	&R20.b2, PRU_DMEM_ADDR, R20.w0, 1	;Update TTS status

	.if $defined("PRU0")
	;Clearing CMP0 status.
	LDI	R21.b0 , TTS_CMP0_CLEAR_STATUS
	SBCO	&R21.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
	.else	;PRU1
	;Setup CMP7 event to check for wraparound on PRU1.
	;Enter the critical section.
	JAL	R5.w0, FN_TTS_IEP_CMPCFG_ARBITRATION

	;Disable compare 7 event.
	LBCO	&R6.w0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 2	;Read compare cfg register.
	AND	R6.b1, R6.b1, TTS_CMP7_DISABLE	;Disable CMP7.
	SBCO	&R6.w0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 2	;Write to CFG register.

	;Setup compare 7 register.
	LDI	R6.w0 , ICSS_EMAC_TTS_IEP_MAX_VAL & 0xFFFF
	LDI	R6.w2 , ICSS_EMAC_TTS_IEP_MAX_VAL >> 16
	SBCO	&R6, IEP_CONST, ICSS_IEP_CMP7_REG, 4

	;Enable compare 7 event.
	LBCO	&R6.w0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 2	;Read compare cfg register.
	OR	R6.b1, R6.b1, TTS_CMP7_ENABLE	;Enable CMP7.
	SBCO	&R6.w0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 2	;Write to CFG register.

	; Exit the critical section
	JAL	R5.w0, FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION
	.endif
	.endif	;ICSS_REV1

CHECK_TTS_ENABLE_done?:
	.endm

	.if $defined("ICSS_REV1")
;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_TTS_SET_CMP0_IEP_WRAP
; Description: Sets Compare 0 IEP to wraparound at 1s. (ICSS_REV1 specific)
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_TTS_SET_CMP0_IEP_WRAP	 .macro
	LDI	R21.w0 , ICSS_EMAC_TTS_IEP_MAX_VAL & 0xFFFF
	LDI	R21.w2 , ICSS_EMAC_TTS_IEP_MAX_VAL >> 16
	SBCO	&R21, IEP_CONST, ICSS_IEP_CMP0_REG, 4
	;Enter the critical section.
	JAL	R5.w0, FN_TTS_IEP_CMPCFG_ARBITRATION
	LBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1	;Read compare cfg register.
	OR	R6.b0, R6.b0, (TTS_CMP0_ENABLE | TTS_CMP0_ENABLE_IEP_WRAP)	;Enable CMP0 and CMP0 RST.
	SBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1	;Write to CFG register.
	; Exit the critical section
	JAL	R5.w0, FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION
	.endm
	.endif	;ICSS_REV1

;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_TTS_CFG_CONDITIONAL_CONSTRUCT
; Description: Check essential conditions before TTS configuration.
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_TTS_CFG_CONDITIONAL_CONSTRUCT	 .macro
	QBBC	TTS_COMPARE_RECFG_false?, R22 , TTS_ENABLE
	QBBS	TTS_FIRST_SETUP_done?, R22 , TTS_FIRST_SETUP
	M_TTS_CYC_FRAME_NOTIFICATION											;Notify host that it is time to insert cyc frame.
	.if $defined("ICSS_REV1")
	JMP		FN_TTS_IEP_CFG_PRE_ICSS_REV1										;Setup compare events for first time.
	.endif
	.if $defined("ICSS_REV2")
	JMP		FN_TTS_IEP_CFG_PRE_ICSS_REV2										;Setup compare events for first time.
	.endif
TTS_FIRST_SETUP_done?:
	LBCO	&R20.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1	;Reading IEP Compare Status Reg.
	.if $defined("PRU0")
	QBBC	TTS_COMPARE_RECFG_false?, R20, 5
	.else 	;PRU1
	QBBC	TTS_COMPARE_RECFG_false?, R20, 6
	.endif
	M_TTS_CYC_FRAME_NOTIFICATION											;Notify host that it is time to insert cyc frame.

	.if $defined("ICSS_REV1")
	JMP		FN_TTS_IEP_CFG_PRE_ICSS_REV1										;If compare 5/6 event has occured, setup next compare events.
	.endif
	.if $defined("ICSS_REV2")
	JMP		FN_TTS_IEP_CFG_PRE_ICSS_REV2										;If compare 5/6 event has occured, setup next compare events.
	.endif
TTS_COMPARE_RECFG_false?:
	.endm

;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_SET_MIIRT_TXCFG_TX_ENABLE
; Description: Set TX_ENABLE bit in MIIRT_TXCFG register if TTS is enabled.
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_SET_MIIRT_TXCFG_TX_ENABLE	 .macro
	QBBC	SKIP_MIIRT_TXCFG_TX_ENABLE?, R22 , TTS_ENABLE
	.if $defined("PRU0")
	LBCO	&R20.b0, MII_RT_CFG_CONST, ICSS_MIIRT_TXCFG0, 1
	OR	R20.b0, R20.b0, TX_ENABLE
	SBCO	&R20.b0, MII_RT_CFG_CONST, ICSS_MIIRT_TXCFG0, 1
	.else	;PRU1
	LBCO	&R20.b0, MII_RT_CFG_CONST, ICSS_MIIRT_TXCFG1, 1
	OR	R20.b0, R20.b0, TX_ENABLE
	SBCO	&R20.b0, MII_RT_CFG_CONST, ICSS_MIIRT_TXCFG1, 1
	.endif	;PRU0
SKIP_MIIRT_TXCFG_TX_ENABLE?:
	.endm

	.if $defined("ICSS_REV1")
;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_TTS_CMP0_CMP7_CHECK
; Description: Checks for CMP0 and CMP7 event,
;			  : which are set for IEP counter wraparound.	(ICSS_REV1 specific)
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_TTS_CMP0_CMP7_CHECK	 .macro
	QBBC	TTS_CMP_0_7_CHECK_disabled?, R22 , TTS_ENABLE
	LBCO	&R21.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1

	.if $defined("PRU0")
	;Checking for CMP0 event.
	QBBC	TTS_CMP_0_7_CHECK_disabled?, R21, 0
	;CMP0 event has occured. Clear CMP0 status.
	LDI	R21.b0 , TTS_CMP0_CLEAR_STATUS
	.else	;PRU1
	;Checking for CMP7 event.
	QBBC	TTS_CMP_0_7_CHECK_disabled?, R21, 7
	;CMP7 event has occured. Clear CMP7 status.
	LDI	R21.b0 , TTS_CMP7_CLEAR_STATUS
	.endif
	SBCO	&R21.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
	CLR	R22 , R22 , TTS_CMP5_CMP6_in_next_IEP_cycle

	;Check if this is the first wraparound after TTS enable.
	QBBC	TTS_CMP_0_7_CHECK_disabled?, R22 , TTS_CMP_3456_FIRST_Exception
	CLR	R22 , R22 , TTS_CMP_3456_FIRST_Exception

	;Enable compare 3/4/5/6 events.
	;Enter the critical section.
	JAL	R5.w0, FN_TTS_IEP_CMPCFG_ARBITRATION
	;Read compare cfg register.
	LBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1
	.if $defined("PRU0")
	OR	R6.b0, R6.b0, (TTS_CMP3_ENABLE | TTS_CMP5_ENABLE)	;Enable CMP3 and CMP5.
	.else	;PRU1
	OR	R6.b0, R6.b0, (TTS_CMP4_ENABLE | TTS_CMP6_ENABLE)	;Enable CMP4 and CMP6.
	.endif
	;Write to config register.
	SBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1
	; Exit the critical section
	JAL	R5.w0, FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION
TTS_CMP_0_7_CHECK_disabled?:
	.endm
	.endif	;ICSS_REV1

;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_TTS_CMP3_CMP4_CHECK
; Description: Checks for CMP3 and CMP4 event and then clears ICSS_EMAC_TTS_INSERT_CYC_FRAME_EVENT.
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_TTS_CMP3_CMP4_CHECK	 .macro
	QBBC	TTS_CMP3_CMP4_EVENT_false?, R22 , TTS_ENABLE
	LBCO	&R21.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
	.if $defined("PRU0")
	QBBC	TTS_CMP3_CMP4_EVENT_false?, R21, 3
	.else	;PRU1
	QBBC	TTS_CMP3_CMP4_EVENT_false?, R21, 4
	.endif
	;CMP3 or CMP4 event has occured.
	;Clear ICSS_EMAC_TTS_INSERT_CYC_FRAME_EVENT.
	LDI	R21.w2 , ICSS_EMAC_TTS_STATUS_OFFSET
	LBCO	&R21.b0, PRU_DMEM_ADDR, R21.w2, 1
	CLR	R21 , R21 , 2
	SBCO	&R21.b0, PRU_DMEM_ADDR, R21.w2, 1
TTS_CMP3_CMP4_EVENT_false?:
	.endm

;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_TTS_CYC_FRAME_NOTIFICATION
; Description: Sets ICSS_EMAC_TTS_INSERT_CYC_FRAME_EVENT bit to notify that it is time to insert cyc frame.
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_TTS_CYC_FRAME_NOTIFICATION	 .macro
	LDI	R21.w2 , ICSS_EMAC_TTS_STATUS_OFFSET
	LBCO	&R21.b0, PRU_DMEM_ADDR, R21.w2, 1
	SET	R21 , R21 , 2
	SBCO	&R21.b0, PRU_DMEM_ADDR, R21.w2, 1
	QBBC	TTS_CYC_INTERRUPT_DISABLED?, R21, 4
	;two interrupts for two ports in MAC mode
	.if $defined("PRU0")
	LDI	R31, TTS_PRU0_CYC_INTERRUPT_EVENT_ID	;Maps to system event 24, 0x28 = 10 1000
	.else	;PRU1
	LDI	R31, TTS_PRU1_CYC_INTERRUPT_EVENT_ID	;Maps to system event 25, 0x29 = 10 1001
	.endif
TTS_CYC_INTERRUPT_DISABLED?:
	.endm

;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_TTS_TX_SOF_PREV_STORE
; Description: Saves TX_SOF of previous packet to PRU DMEM0/1.
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_TTS_TX_SOF_PREV_STORE	 .macro
	QBBC	TTS_DISABLED_TX_SOF_store?, R22 , TTS_ENABLE

	.if $defined("ICSS_REV1")
	.if $defined("PRU0")
	LBCO	&R20, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 4
	.else	;PRU1
	LBCO	&R20, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 4
	.endif
	LDI	TEMP_REG_1.w0 , ICSS_EMAC_TTS_PREV_TX_SOF
	SBCO	&R20, PRU_DMEM_ADDR, TEMP_REG_1.w0, 4
	.endif	;ICSS_REV1

	.if $defined("ICSS_REV2")
	.if $defined("PRU0")
	LBCO	&R20, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 8
	.else	;PRU1
	LBCO	&R20, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 8
	.endif
	LDI	TEMP_REG_1.w0 , ICSS_EMAC_TTS_PREV_TX_SOF
	SBCO	&R20, PRU_DMEM_ADDR, TEMP_REG_1.w0, 8
	.endif	;ICSS_REV2

TTS_DISABLED_TX_SOF_store?:
	.endm

;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_TTS_TX_SOF_CYC_STORE
; Description: Saves TX_SOF of current cyclic packet (to be added in next cyclic packet for testing).
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_TTS_TX_SOF_CYC_STORE	 .macro
	LDI	R21.w2 , ICSS_EMAC_TTS_STATUS_OFFSET
	LBCO	&R21.b0, PRU_DMEM_ADDR, R21.w2, 1
	QBBC	TTS_DISABLED_TX_SOF_CYC_store?, R21, 3
	QBBC	TTS_DISABLED_TX_SOF_CYC_store?, R22 , TTS_RT_QUEUE_PKT
	LDI	TEMP_REG_1.w0 , ICSS_EMAC_TTS_CYC_TX_SOF
	.if $defined("ICSS_REV1")
	SBCO	&TEMP_REG_3, PRU_DMEM_ADDR, TEMP_REG_1.w0, 4
	.endif

	.if $defined("ICSS_REV2")
	SBCO	&TEMP_REG_3, PRU_DMEM_ADDR, TEMP_REG_1.w0, 8
	.endif
TTS_DISABLED_TX_SOF_CYC_store?:
	.endm

	.if $defined("ICSS_REV2")
;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_TTS_TX_SOF_COMPARE_ICSS_REV2
; Description: Compares previous and current TX SOF.
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_TTS_TX_SOF_COMPARE_ICSS_REV2	 .macro
	LDI	R21.w0 , ICSS_EMAC_TTS_PREV_TX_SOF
	LBCO	&R20, PRU_DMEM_ADDR, R21.w0, 8	;Read previous TX_SOF.
TTS_TX_SOF_not_updated?:
	.if $defined("PRU0")
	LBCO	&TEMP_REG_3, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 8	;Read current TX_SOF.
	.else
	LBCO	&TEMP_REG_3, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 8	;Read current TX_SOF.
	.endif
	QBNE	TTS_TX_SOF_CHECK_passed?, TEMP_REG_3, R20
	QBEQ	TTS_TX_SOF_not_updated?, TEMP_REG_4, R21
TTS_TX_SOF_CHECK_passed?:
	M_TTS_TX_SOF_CYC_STORE
	.endm
	.endif	;ICSS_REV2

	.if $defined("ICSS_REV1")
;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_TTS_TX_SOF_COMPARE_ICSS_REV1
; Description: Compares previous and current TX SOF.
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_TTS_TX_SOF_COMPARE_ICSS_REV1	 .macro
	LDI	R21.w0 , ICSS_EMAC_TTS_PREV_TX_SOF
	LBCO	&R20, PRU_DMEM_ADDR, R21.w0, 4	;Read previous TX_SOF.
TTS_TX_SOF_not_updated?:
	.if $defined("PRU0")
	LBCO	&TEMP_REG_3, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 4	;Read current TX_SOF.
	.else
	LBCO	&TEMP_REG_3, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 4	;Read current TX_SOF.
	.endif
	QBEQ	TTS_TX_SOF_not_updated?, TEMP_REG_3, R20
TTS_TX_SOF_CHECK_passed?:
	M_TTS_TX_SOF_CYC_STORE
	.endm
	.endif	;ICSS_REV1

;;////////////////////////////////////////////////////////////////////////////
;------------------------------------------------------------------------------
; Macro Name: M_TTS_FIFO_FILL_MOD
; Description: Needed to modify FIFO fill level based on CMP event status.
; Input Parameters: none
; Output Parameters: none
;------------------------------------------------------------------------------
M_TTS_FIFO_FILL_MOD	 .macro
	QBBC	TTS_DISABLED_fifofill?, R22 , TTS_ENABLE
	LBCO	&R21.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1	;Reading IEP Compare Status Reg.
	.if $defined("PRU0")
	QBBS	TTS_TX_SOF_COMPARE_start?, R21, 3
	.else 	;PRU1
	QBBS	TTS_TX_SOF_COMPARE_start?, R21, 4
	.endif
	JMP	TASK_EXECUTION_FINISHED
TTS_TX_SOF_COMPARE_start?:
	.if $defined("ICSS_REV2")
	M_TTS_TX_SOF_COMPARE_ICSS_REV2
	.endif
	.if $defined("ICSS_REV1")
	M_TTS_TX_SOF_COMPARE_ICSS_REV1
	.endif
TTS_DISABLED_fifofill?:
	.if $defined("ICSS_REV1")
	M_XMT_GET_TXSOF_ICSS_REV1
	.endif	;ICSS_REV1
	.endm

;;////////////////////////////////////////////////////////////////////////////
;-------------------- End of Time Triggered Send Macros ---------------------;
;;////////////////////////////////////////////////////////////////////////////
	.endif	;TTS
	.endif	;__emac_tts_hp


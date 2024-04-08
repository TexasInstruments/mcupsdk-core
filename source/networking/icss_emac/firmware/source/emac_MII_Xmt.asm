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
; file:   emac_MII_Xmt.asm
;
; brief:  Transmit task.
;
;
;  (C) Copyright 2017-2018, Texas Instruments, Inc
;
;

    .if !$defined("__mii_xmt_p")
__mii_xmt_p	.set	1

;;///////////////////////////////////////////////////////
; Includes Section
;;///////////////////////////////////////////////////////
    .include "icss_intc_regs.h"
    .if $defined("ICSS_DUAL_EMAC_BUILD")
    .include "icss_emacSwitch.h"
    .endif
    .if $defined("ICSS_SWITCH_BUILD")
        .include "icss_switch.h"
    .endif
    .include "icss_miirt_regs.h"
    .include "icss_defines.h"
    .include "micro_scheduler.h"
    .include "emac_MII_Xmt.h"
    .include "emac_MII_Rcv.h"
    .include "icss_macros.h"
    .include "emac_MII_Xmt.h"
    .include "icss_iep_regs.h"
    .if $defined("TTS")
    .include "emac_tts.h"
    .endif

    .if $defined(PTP)
    .include "icss_ptp.h"
    .include "icss_ptp_macro.h"
    .cdecls C,NOLIST
%{
#include "icss_timeSync_memory_map.h"
%}
    .endif ;PTP
    .global  FN_RCV_LB
    .global  XMT_QUEUE
    .global  MII_TX_TASK
    .global  TASK_EXECUTION_FINISHED
    .global  CHECK_NEXT_QUEUE
    .global  START_XMT_QUEUE
    .if $defined("TTS")
    .global  FN_TTS_PKT_SIZE_CHECK_ICSS_REV1
    .global  FN_TTS_PKT_SIZE_CHECK_ICSS_REV2
    .endif

    .if $defined(PTP)
    .global FN_PTP_TX_ADD_DELAY
    .global FN_PTP_TX_ADD_DELAY_UDP
    .endif

MII_TX_TASK:
    .if $defined("ICSS_DUAL_EMAC_BUILD")
    LDI	R0.w2 , PORT_STATUS_OFFSET      ; load PORT status value from DRAM
    .if    $defined("TWO_PORT_CFG")
    LBCO    &R0.w0, PRU_CROSS_DMEM, R0.w2, 1
    .else
    LBCO	&R0.w0, PRU_DMEM_ADDR, R0.w2, 1
    .endif
    .endif ;ICSS_DUAL_EMAC_BUILD
        ; if half-duplex is enabled set the half-duplex bit else clear it
    .if $defined("HALF_DUPLEX_ENABLED")
    QBBS	SET_HALF_DUPLEX, R0, 1
    CLR	R22 , R22 , PORT_IS_HALF_DUPLEX
    QBA     CHECK_XMT_ACTIVE
SET_HALF_DUPLEX:
    SET	R22 , R22 , PORT_IS_HALF_DUPLEX

CHECK_XMT_ACTIVE:
    .endif
        ; if transmission is active then continue with it else check if any queue has some packet
    QBBS	XMT_QUEUE, R23, Xmt_active

;****************************************************************************
;
;     NAME			: FN_XMT_scheduler
;     DESCRIPTION	: if any queue is not empty then schedules the trasmit
;     RETURNS		:
;     ARGS			:
;     USES 		:
;     INVOKES 		:
;
;****************************************************************************
TX_QUEUE_CONT:
    ;check if port link is up else jump to next task
    .if $defined("ICSS_SWITCH_BUILD")
    QBBS	XMT_FB_EGRESS_LINK_UP_FWD, R22, 10	 ;replaced: QBBS    XMT_FB_EGRESS_LINK_UP_FWD, OPPOSITE_PORT_LINK_UP
    .endif ;ICSS_SWITCH_BUILD
    .if $defined("ICSS_DUAL_EMAC_BUILD")
    QBBS	XMT_FB_EGRESS_LINK_UP_FWD, R0, 0
    .endif ;ICSS_DUAL_EMAC_BUILD
    QBA     NO_TRANSMIT_PACKET

    .if $defined("HALF_DUPLEX_ENABLED")
COLLISION_DETECTED:
        ;this is the collision detect procedure
    CLR	R23 , R23 , 0       ; clear the XMT_active flag indicating active transmission

        ;reset TX FIFO
    M_TX_RESET

    ;Add 1 to collision counter
    LDI	R0 , COLLISION_COUNTER
    LBCO	&R2, PRU_DMEM_ADDR, R0, 1
    ADD	R2.b0, R2.b0, 1
    SBCO	&R2, PRU_DMEM_ADDR, R0, 1

    ;clear the stat flags for TX_BC_FRAME and TX_MC_FRAME
    CLR	R22 , R22 , TX_BC_FRAME
    CLR	R22 , R22 , TX_MC_FRAME

        ; if there is late collision then jump else continue to next task
    QBBS	ADD_TX_STATS_DO_NOT_RESET_COLLISION_COUNTER, R22 , TX_LATE_COLLISION
    QBA     NO_TRANSMIT_PACKET

CARRIER_SENSE_DETECTED:
        ;this is the carrier sense procedure
    CLR	R23 , R23 , 0
    LDI	R0 , TX_DEFERRED_OFFSET
    QBA      INCREMENT_TX_COUNT

ADD_TX_STATS:
        ;initialize the collision counter
    LDI	R3 , COLLISION_COUNTER
    LDI	R2.b0 , 0
    SBCO	&R2, PRU_DMEM_ADDR, R3, 1
    QBA     INCREMENT_TX_COUNT

ADD_TX_STATS_DO_NOT_RESET_COLLISION_COUNTER:
        ; increment and update the late collision counter
    CLR	R22 , R22 , TX_LATE_COLLISION
    LDI	R0 , LATE_COLLISION_OFFSET
INCREMENT_TX_COUNT:
    LBCO	&R2, PRU_DMEM_ADDR, R0, 4
    ADD	R2, R2, 1
    SBCO	&R2, PRU_DMEM_ADDR, R0, 4
    .endif ;HALF_DUPLEX_ENABLED

NO_TRANSMIT_PACKET:
    JMP     TASK_EXECUTION_FINISHED

XMT_FB_EGRESS_LINK_UP_FWD:
; New optimized code for checking the transmit queues
; Loop four times as there are four transmit queues

    .if $defined("PRU0")
        .if    $defined("TWO_PORT_CFG")

        LDI     R20.w0, P2_Q4_TX_CONTEXT_OFFSET
        LDI     QUEUE_DESC_OFFSET, P2_QUEUE_DESC_OFFSET - 8
        LDI     TX_CONTEXT_OFFSET, P2_Q1_TX_CONTEXT_OFFSET - 8
CHECK_NEXT_QUEUE:
        QBEQ    NO_TRANSMIT_PACKET, TX_CONTEXT_OFFSET, R20.w0
        ADD     QUEUE_DESC_OFFSET, QUEUE_DESC_OFFSET, 8
        LBCO    &QUEUE_DESC_REG, QUEUE_DESP_BASE, QUEUE_DESC_OFFSET, 8
        ADD     TX_CONTEXT_OFFSET, TX_CONTEXT_OFFSET, 8
        QBEQ    CHECK_NEXT_QUEUE, QUEUE_DESC_REG.wr_ptr, QUEUE_DESC_REG.rd_ptr
        .else ;MAC MODE
    	LDI	R20.w0, Q4_TX_CONTEXT_OFFSET	    ; set R20 to last offset before it begins
    	LDI	QUEUE_DESC_OFFSET , PORT_QUEUE_DESC_OFFSET - 8      ;set QUEUE_DESC_OFFSET to base before it begin
    	LDI	TX_CONTEXT_OFFSET , Q1_TX_CONTEXT_OFFSET - 8        ;set TX_CONTEXT_OFFSET to base before it begin
CHECK_NEXT_QUEUE:
    	.if $defined("HALF_DUPLEX_ENABLED")
    	;skip if half duplex is not set
    	QBBC	SKIP_CRS_Q, R22 , PORT_IS_HALF_DUPLEX
    	;defer if carrier sense is on via reading the PRUSS_MII_RT_PRS0 register
    	LBCO	&TEMP_REG_1, MII_RT_CFG_CONST, MII_CARRIER_SENSE_REG, 1
    	QBBS	CARRIER_SENSE_DETECTED, TEMP_REG_1, 1
SKIP_CRS_Q:
    	.endif ;HALF_DUPLEX_ENABLED
    	QBEQ	NO_TRANSMIT_PACKET, TX_CONTEXT_OFFSET, R20.w0	    ; if TX_CONTEXT_OFFSET matches the last descriptor then nothing to transmit
    	ADD	QUEUE_DESC_OFFSET, QUEUE_DESC_OFFSET, 8
    	LBCO	&QUEUE_DESC_REG, PRU_DMEM_ADDR, QUEUE_DESC_OFFSET, 8
    	ADD	TX_CONTEXT_OFFSET, TX_CONTEXT_OFFSET, 8	            ; increment the queue and buffer offset
    	QBEQ    CHECK_NEXT_QUEUE, QUEUE_DESC_REG.wr_ptr, QUEUE_DESC_REG.rd_ptr      ; if read == write pointer then move to next queue
    	.if $defined("TTS")
    	M_TTS_XMT_SCHEDULER	TX_CONTEXT_OFFSET	; check if queue has any TTS packets
    	.endif	;TTS
        .endif
    .else
    ;PRU1
        .if    $defined("TWO_PORT_CFG")

        LDI     R20.w0, P1_Q4_TX_CONTEXT_OFFSET
        LDI     QUEUE_DESC_OFFSET, P1_QUEUE_DESC_OFFSET - 8
        LDI     TX_CONTEXT_OFFSET, P1_Q1_TX_CONTEXT_OFFSET - 8
CHECK_NEXT_QUEUE:
        QBEQ    NO_TRANSMIT_PACKET, TX_CONTEXT_OFFSET, R20.w0
        ADD     QUEUE_DESC_OFFSET, QUEUE_DESC_OFFSET, 8
        LBCO    &QUEUE_DESC_REG, QUEUE_DESP_BASE, QUEUE_DESC_OFFSET, 8
        ADD     TX_CONTEXT_OFFSET, TX_CONTEXT_OFFSET, 8
        QBEQ    CHECK_NEXT_QUEUE, QUEUE_DESC_REG.wr_ptr, QUEUE_DESC_REG.rd_ptr
        .else  ;MAC MODE
    	LDI	R20.w0, Q4_TX_CONTEXT_OFFSET        ; set R20 to last offset before it begins
    	LDI	QUEUE_DESC_OFFSET , PORT_QUEUE_DESC_OFFSET - 8     ;set QUEUE_DESC_OFFSET to base before it begin
    	LDI	TX_CONTEXT_OFFSET , Q1_TX_CONTEXT_OFFSET - 8        ;set TX_CONTEXT_OFFSET to base before it begin
CHECK_NEXT_QUEUE:
    	.if $defined("HALF_DUPLEX_ENABLED")
    	;skip if half duplex is not set
    	QBBC	SKIP_CRS_Q, R22 , PORT_IS_HALF_DUPLEX
    	;defer if carrier sense is on via reading the PRUSS_MII_RT_PRS1 register
    	LBCO	&TEMP_REG_1, MII_RT_CFG_CONST, MII_CARRIER_SENSE_REG, 1
    	QBBS	CARRIER_SENSE_DETECTED, TEMP_REG_1, 1
SKIP_CRS_Q:
    	.endif ;HALF_DUPLEX_ENABLED
    	QBEQ	NO_TRANSMIT_PACKET, TX_CONTEXT_OFFSET, R20.w0	    ; if TX_CONTEXT_OFFSET matches the last descriptor then nothing to transmit
    	ADD	QUEUE_DESC_OFFSET, QUEUE_DESC_OFFSET, 8
    	LBCO	&QUEUE_DESC_REG, PRU_DMEM_ADDR, QUEUE_DESC_OFFSET, 8
    	ADD	TX_CONTEXT_OFFSET, TX_CONTEXT_OFFSET, 8	            ; increment the queue and buffer offset
    	QBEQ    CHECK_NEXT_QUEUE, QUEUE_DESC_REG.wr_ptr, QUEUE_DESC_REG.rd_ptr      ; if read == write pointer then move to next queue
    	.if $defined("TTS")
    	M_TTS_XMT_SCHEDULER	TX_CONTEXT_OFFSET	; check if queue has any TTS packets
    	.endif	;TTS
    	.endif
    .endif

;;////////////////////////////////////////////////////////////////////////////////////////

;****************************************************************************
;
;     NAME		: FN_XMT_queue
;     DESCRIPTION	: trasmits the first block of 32B data from the queue into TX FIFO
;     RETURNS		:
;     ARGS		:
;     USES 		:
;     INVOKES 		:
;
;****************************************************************************
START_XMT_QUEUE:
    ; Read the TX Context of 8 Bytes.
    .if    $defined("TWO_PORT_CFG")
    LBCO     &BUFFER_OFFSET, PRU1_DMEM_CONST, TX_CONTEXT_OFFSET, 8
    .else
    LBCO	&BUFFER_OFFSET, PRU_DMEM_ADDR, TX_CONTEXT_OFFSET, 8
    .endif ;TWO_PORT_CFG
    ; init MII_XMT parameter
    SET	R23 , R23 , Xmt_active ; set global flag to indicate an ongoing transmission
    SUB	BUFFER_DESC_OFFSET, QUEUE_DESC_REG.rd_ptr, BASE_BUFFER_DESC_OFFSET      ;subtract the rd-ptr value from base
    LSL	BUFFER_DESC_OFFSET, BUFFER_DESC_OFFSET, 3	        ; shift the value by 3 as descriptor size is 8 bytes
    ADD	BUFFER_INDEX, BUFFER_OFFSET, BUFFER_DESC_OFFSET	        ; find the actual buffer index.
    AND     BUFFER_DESC_OFFSET , QUEUE_DESC_REG.rd_ptr , QUEUE_DESC_REG.rd_ptr
    .if    $defined("TWO_PORT_CFG")
    LBCO    &BUFFER_DESC_REG, ICSS_SHARED_CONST, BUFFER_DESC_OFFSET, 4
    .else
    LBCO	&BUFFER_DESC_REG, PRU_DMEM_ADDR, BUFFER_DESC_OFFSET, 4	; load the buffer_desc_reg form offset
    .endif ;TWO_PORT_CFG
    LSL	Packet_Length, BUFFER_DESC_REG.pkt_length, 3	;bit 18...28

    LSR	Packet_Length, Packet_Length, 5
    LDI	BYTE_CNT , 0
    ; Read the Phy speed and set the flag accordingly
    LDI	R2.w0, PHY_SPEED_OFFSET
    .if    $defined("TWO_PORT_CFG")
    LBCO    &R2.w2, PRU_CROSS_DMEM , R2.w0, 2
    .else
    LBCO	&R2.w2, PRU_DMEM_ADDR, R2.w0, 2
    .endif

    SET	R23 , R23 , TX_PHY_SPEED ; means PHY is set to 100 Mbps
    QBEQ	XMT_FB_100Mbps_MODE, R2.b2, 100
    CLR	R23 , R23 , TX_PHY_SPEED ; means PHY is set to 10 Mbps
XMT_FB_100Mbps_MODE:
    .if    $defined("TWO_PORT_CFG")
    CLR     R13, R13, PACKET_FROM_COLL_QUEUE
    QBBC    no_collision_occured, R11, 14    ;BUFFER_DESC_REG.Shadow    ;BUFFER_DESC_REG.Shadow
    SET     R13, R13, PACKET_FROM_COLL_QUEUE
    ; Change the Precompute offsets to one's for the collision buffer
    .if    $defined("PRU0")
    LDI        TX_CONTEXT_OFFSET, COL_TX_CONTEXT_P2_Q1_OFFSET_ADDR
    .else
    LDI        TX_CONTEXT_OFFSET, COL_TX_CONTEXT_P1_Q1_OFFSET_ADDR
    .endif
    ; Read the Collision TX Context of 6 Bytes.
    LBCO     &BUFFER_INDEX, PRU1_DMEM_CONST, TX_CONTEXT_OFFSET, 6
no_collision_occured:
    .endif ;TWO_PORT_CFG
CHECK_RX_EOF:
;Check RX EOF.
    .if $defined("ICSS_REV1")
    M_XMT_RX_EOF_CHECK_ICSS_REV1	process_rx_eof_tx_fb    ; check EOF of RCV is receive is active
    .endif
    .if $defined("ICSS_REV2")
    M_XMT_RX_EOF_CHECK_ICSS_REV2	process_rx_eof_tx_fb    ; check EOF of RCV is receive is active
    .endif

    LBCO	&BUFFER, L3_OCMC_RAM_CONST, BUFFER_INDEX, 32    ; load the buffer value from its index
    .if $defined("HALF_DUPLEX_ENABLED")
        ;load the value of PRUSS_MII_RT_PRS0 register and check for carrier sense
    QBBC	CONTINUE_TX, R22 , PORT_IS_HALF_DUPLEX
    LBCO	&R0, MII_RT_CFG_CONST, MII_CARRIER_SENSE_REG, 1
    QBBC	CONTINUE_TX, R0, 1
    QBA     CARRIER_SENSE_DETECTED

CONTINUE_TX:
    .endif ;HALF_DUPLEX_ENABLED

;Do TX stats here
    QBBS	TX_IS_MC_OR_BC, R2, 0
    QBA     START_TX_FIFO_FILL

TX_IS_MC_OR_BC:
        ; based on mac address find out if it BC or MC frame
    FILL	&R0, 4  ;Fill with 0xffffffff
    QBNE	TX_IS_MC, R2, R0	;First four bytes of MAC ID
    QBNE	TX_IS_MC, R3.w0, R0.w0	;upper two bytes of MAC ID
    SET	R22 , R22 , TX_BC_FRAME
    QBA     START_TX_FIFO_FILL

TX_IS_MC:
    SET	R22 , R22 , TX_MC_FRAME

START_TX_FIFO_FILL:
    .if $defined("TTS")
    M_TTS_TX_SOF_PREV_STORE
    .endif	;TTS

    .if $defined("ICSS_REV1")
    ;Just before pushing to FIFO store the current Tx SOF TS
    ;This is used for comparison later to make sure SOF has actually
    ;updated
    .if $defined(PTP)
        .if $defined("ICSS_SWITCH_BUILD")
            .if $defined (PRU0)
            LBCO    &R20, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 4
            LDI		RCV_TEMP_REG_2, PTP_PREV_TX_TIMESTAMP_P2
            SBCO    &R20, ICSS_SHARED_CONST, RCV_TEMP_REG_2, 4
            ;SBCO    &R20, ICSS_SHARED_CONST, PTP_PREV_TX_TIMESTAMP_P2, 4
            .else
            LBCO    &R20, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 4
            ;LDI		RCV_TEMP_REG_1, PTP_PREV_TX_TIMESTAMP_P1
            ;SBCO    &R20, ICSS_SHARED_CONST, RCV_TEMP_REG_1, 4
            SBCO    &R20, ICSS_SHARED_CONST, PTP_PREV_TX_TIMESTAMP_P1, 4
            .endif
        .else
            .if $defined (PRU0)
            LBCO    &R20, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 4
            LDI		RCV_TEMP_REG_2, PTP_PREV_TX_TIMESTAMP_P1
            SBCO    &R20, ICSS_SHARED_CONST, RCV_TEMP_REG_2, 4
            ;SBCO    &R20, ICSS_SHARED_CONST, PTP_PREV_TX_TIMESTAMP_P1, 4
           .else
            LBCO    &R20, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 4
            LDI		RCV_TEMP_REG_1, PTP_PREV_TX_TIMESTAMP_P2
            SBCO    &R20, ICSS_SHARED_CONST, RCV_TEMP_REG_1, 4
            ;SBCO    &R20, ICSS_SHARED_CONST, PTP_PREV_TX_TIMESTAMP_P2, 4
           .endif
        .endif ; ICSS_SWITCH_BUILD
    .endif ;PTP
    LDI	TX_DATA_WORD_MASK , 0xffff
    .if !$defined("TX_L2_ENABLED")
    AND TX_DATA_BYTE , BUFFER.b0 , BUFFER.b0
    M_PUSH_BYTE
    AND TX_DATA_BYTE , BUFFER.b1 , BUFFER.b1
    M_PUSH_BYTE
    .else
	MOV		R2.b0, BUFFER.b0
    XOUT    TX_L2_BANK_ID, &R2, 1
	MOV		R2.b0, BUFFER.b1
    XOUT    TX_L2_BANK_ID, &R2, 1
    .endif ;TX_L2_ENABLED
    .endif	;ICSS_REV1

PUSH_FB:
    .if $defined(PTP)
    M_GPTP_TX_PRE_PROC
    .endif ;PTP
        ; Insert the data in Tx Fifo
    .if $defined("ICSS_REV1")
    LDI	TX_DATA_POINTER, buffer_ptr + 2
    loop	EndLoop_FB1, 15					; Insert 32 bytes for ICSS_REV1
    .if !$defined("TX_L2_ENABLED")
    MVIW	TX_DATA_WORD, *TX_DATA_POINTER
    M_PUSH_WORD_CMD
    .else
	MVIW	R2.w0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 2
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 2
EndLoop_FB1:
    .endif	;ICSS_REV1

    .if $defined("ICSS_REV2")
    LDI	TX_DATA_POINTER, buffer_ptr
    loop	EndLoop_FB1, 8					; Insert 32 bytes for ICSS_REV2
    .if !$defined("TX_L2_ENABLED")
    MVID	TX_DATA_DOUBLE_WORD, *TX_DATA_POINTER
    .else
	MVID	R2, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 4
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 4
EndLoop_FB1:
    .endif	;ICSS_REV2
    ;For EMAC we don't do bridge delay correction
    ;So this can be called after pushing 32 bytes
    .if $defined(PTP)
    JAL     R0.w0, FN_PTP_TX_ADD_DELAY
    .endif

        ;increment both descriptor and index with offset
    ADD	BUFFER_INDEX, BUFFER_INDEX, 32
    ADD	BYTE_CNT, BYTE_CNT, 32
    SET	R13 , R13 , INCREMENT_WRK_BUFFER_DESC_OFFSET
    LDI	BYTES_TRANSFERRED_IN_LAST_CALL, 0x00	; indicate how many bytes have been transfered in last call
    LDI	SHIFT_REG, SHIFT_NONE
        ; store the context back
    .if $defined("PRU0")
    XOUT	BANK1, &MII_TX_CONTEXT, 20
    .else
    XOUT	BANK2, &MII_TX_CONTEXT, 20
    .endif

    .if $defined("HALF_DUPLEX_ENABLED")
    ;load the value of PRUSS_MII_RT_PRS0 register and check for carrier sense
    QBBC	NO_COLLISION1, R22 , PORT_IS_HALF_DUPLEX
    LBCO	&TEMP_REG_1, MII_RT_CFG_CONST, MII_CARRIER_SENSE_REG, 1
    QBBC	NO_COLLISION1, TEMP_REG_1, 0
    QBA     COLLISION_DETECTED
NO_COLLISION1:
    .endif ;HALF_DUPLEX_ENABLED

    .if $defined("ICSS_REV1")
    M_XMT_RX_EOF_CHECK_ICSS_REV1	process_rx_eof_tx_fb_after_inserting_32bytes        ;check for EOF for recieve buffer if recieve is active
    .endif	;ICSS_REV1
    .if $defined("ICSS_REV2")
    M_XMT_RX_EOF_CHECK_ICSS_REV2	process_rx_eof_tx_fb_after_inserting_32bytes        ;check for EOF for recieve buffer if recieve is active
    .endif	;ICSS_REV2

    ; No RX EOF event so fill Tx FIFO more bytes
    QBNE	NO_QUEUE_WRAP_XMT_FB, BUFFER_DESC_OFFSET, TOP_MOST_BUFFER_DESC_OFFSET
    AND BUFFER_DESC_OFFSET , BASE_BUFFER_DESC_OFFSET , BASE_BUFFER_DESC_OFFSET
    CLR	R13 , R13 , INCREMENT_WRK_BUFFER_DESC_OFFSET ; Since the Queue has wrapped here itself ..no need to check for it in xmt_nb for first time
    .if    $defined("TWO_PORT_CFG")
    QBBS    NO_QUEUE_WRAP_XMT_FB, R13, 2    ;PACKET_FROM_COLL_QUEUE
    .endif ;TWO_PORT_CFG
    AND BUFFER_INDEX , BUFFER_OFFSET , BUFFER_OFFSET

NO_QUEUE_WRAP_XMT_FB:

    LBCO	&BUFFER, L3_OCMC_RAM_CONST, BUFFER_INDEX, 32	    ; load new data from buffer
    LDI	TX_DATA_POINTER, buffer_ptr

    .if $defined(PTP)
    LDI    TEMP_REG_1.w0, PTP_IPV4_UDP_E2E_ENABLE
    LBCO   &TEMP_REG_1.b0, ICSS_SHARED_CONST, TEMP_REG_1.w0, 1
    QBEQ   PTP_NOT_ENABLED_TX, TEMP_REG_1.b0, 0
    JAL     R0.w0, FN_PTP_TX_ADD_DELAY_UDP
PTP_NOT_ENABLED_TX:
    .endif

    .if $defined("ICSS_REV1")
    ; Insert next 22 bytes in Tx FIFO
    loop	EndLoop_FB_22bytes_more, 11
    .if !$defined("TX_L2_ENABLED")
    MVIW	TX_DATA_WORD, *TX_DATA_POINTER
    M_PUSH_WORD_CMD
    .else
	MVIW	R2.w0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 2
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 2
EndLoop_FB_22bytes_more:
        ; increment the buffer and byte count
    ADD	BUFFER_INDEX, BUFFER_INDEX, 22
    ADD	BYTE_CNT, BYTE_CNT, 22
    LDI	BYTES_TRANSFERRED_IN_LAST_CALL, 22
    LDI	SHIFT_REG, SHIFT_NONE
    .endif	;ICSS_REV1

    .if $defined("ICSS_REV2")
    ; Insert next 28 bytes in Tx FIFO
    loop	EndLoop_FB_28bytes_more, 7
    .if !$defined("TX_L2_ENABLED")
    MVID	TX_DATA_DOUBLE_WORD, *TX_DATA_POINTER
    .else
	MVID	R2, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 4
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 4
EndLoop_FB_28bytes_more:
        ;increment the buffer and byte count
    ADD	BUFFER_INDEX, BUFFER_INDEX, 28
    ADD	BYTE_CNT, BYTE_CNT, 28
    LDI	BYTES_TRANSFERRED_IN_LAST_CALL, 28
    LDI	SHIFT_REG, SHIFT_NONE
    .endif	;ICSS_REV2

         ; store the context back
    .if $defined("PRU0")
    XOUT	BANK1, &MII_TX_CONTEXT, 20
    .else
    XOUT	BANK2, &MII_TX_CONTEXT, 20
    .endif
    .if $defined("HALF_DUPLEX_ENABLED")
    ;load the value of PRUSS_MII_RT_PRS0 register and check for carrier sense
    QBBC	NO_COLLISION2, R22 , PORT_IS_HALF_DUPLEX
    LBCO	&TEMP_REG_1, MII_RT_CFG_CONST, MII_CARRIER_SENSE_REG, 1
    QBBC	NO_COLLISION2, TEMP_REG_1, 0
    QBA     COLLISION_DETECTED
NO_COLLISION2:
    .endif ;HALF_DUPLEX_ENABLED

TASK_EXECUTION_FINISHED_intr:
    JMP		TASK_EXECUTION_FINISHED

process_rx_eof_tx_fb:
    CLR	R23 , R23 , Xmt_active
process_rx_eof_tx_fb_after_inserting_32bytes:
    JAL	CALL_REG, FN_RCV_LB         ; jump to transmit last 32 bytes

;****************************************************************************
;
;     NAME			: XMT_QUEUE
;     DESCRIPTION	: trasmits the next block of 32B data from the queue into TX FIFO
;     RETURNS		:
;     ARGS			:
;     USES 		:
;     INVOKES 		:
;
;****************************************************************************
XMT_QUEUE:
    .if $defined("ICSS_DUAL_EMAC_BUILD")
    ;check if port link is up else move to next task
    LDI	R0.w2 , PORT_STATUS_OFFSET
    LBCO	&R0.b0, PRU_DMEM_ADDR, R0.w2, 1
    QBBS	XMT_QUEUE_PORT_LINK_IS_UP, R0, 0
    ;Port link has gone down while frame was in transmission.
    CLR	R23 , R23 , 0
    M_TX_RESET
    SET	R22 , R22 , PACKET_TX_ALLOWED
    JMP		TASK_EXECUTION_FINISHED
XMT_QUEUE_PORT_LINK_IS_UP:
    .endif  ;ICSS_DUAL_EMAC_BUILD
    ; load the content of tx context back from the banks
    LDI	SHIFT_REG, SHIFT_NONE
    .if $defined("PRU0")
    XIN	BANK1, &MII_TX_CONTEXT, 20
    .else
    XIN	BANK2, &MII_TX_CONTEXT, 20
    .endif

        ; check and set if half duplex is set
    .if $defined("HALF_DUPLEX_ENABLED")
    LDI	TEMP_REG_3.w2 , PORT_STATUS_OFFSET
    LBCO	&TEMP_REG_3.w0, ICSS_SHARED_CONST, TEMP_REG_3.w2, 1
    QBBS	SET_HALF_DUPLEX1, TEMP_REG_3, 0
    CLR	R22 , R22 , PORT_IS_HALF_DUPLEX
    QBA     CONTINUE_XMT_QUEUE
SET_HALF_DUPLEX1:
    SET	R22 , R22 , PORT_IS_HALF_DUPLEX
    .endif ;HALF_DUPLEX_ENABLED

CONTINUE_XMT_QUEUE:

    .if $defined("TTS")
    M_TTS_FIFO_FILL_MOD     ;find the current fifo fill level based on compare event
    .endif	;TTS

    .if $defined("ICSS_DUAL_EMAC_BUILD")
    .if !$defined("TTS")
;Only need to read in non-TTS EMAC.
;For TTS EMAC, it is read in M_TTS_FIFO_FILL_MOD-->M_TTS_TX_SOF_COMPARE_ICSS_REV1.
    .if $defined("ICSS_REV1")
    M_XMT_GET_TXSOF_ICSS_REV1
    .endif	;ICSS_REV1
    .endif	;TTS
    .endif  ;ICSS_DUAL_EMAC_BUILD

    ; port tx sof capture for port 1 (PRU1) and port 2 (PRU0) - 2 port config.
    .if $defined("TWO_PORT_CFG")
    .if $defined("PRU0")
    LBCO	&TEMP_REG_3, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 4
    .else
    LBCO	&TEMP_REG_3, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 4
    .endif
    .endif ;TWO_PORT_CFG
    .if $defined("HALF_DUPLEX_ENABLED")
    ;load the value of PRUSS_MII_RT_PRS0 register and check for carrier sense
    QBBC	NO_COLLISION3, R22 , PORT_IS_HALF_DUPLEX
    LBCO	&TEMP_REG_1, MII_RT_CFG_CONST, MII_CARRIER_SENSE_REG, 1
    QBBC	NO_COLLISION3, TEMP_REG_1, 0
    QBA     COLLISION_DETECTED
NO_COLLISION3:
    .endif ;HALF_DUPLEX_ENABLED

    .if $defined("ICSS_REV1")
    M_XMT_FILL_LEVEL_CALC_ICSS_REV1     ; calculate the current TX fifo fill level
    .endif	;ICSS_REV1

    .if $defined("ICSS_REV2")
    M_XMT_FILL_LEVEL_CALC_ICSS_REV2     ; calculate the current TX fifo fill level
    .endif	;ICSS_REV2

    QBEQ	XMT_NB_DONE, FREE_SPACE_IN_FIFO, 0
    QBGE	FILL_TX_FIFO, FREE_SPACE_IN_FIFO, 32	; if free space in fifo is less than 32 bytes then jmp else continue
    LDI	FREE_SPACE_IN_FIFO, 32

FILL_TX_FIFO:
    ;Check for RX EOF.
    .if $defined("ICSS_REV1")
    M_XMT_RX_EOF_CHECK_ICSS_REV1	process_rx_eof_tx_nb    ;check for EOF for recieve buffer if recieve is active
    .endif	;ICSS_REV1
    .if $defined("ICSS_REV2")
    M_XMT_RX_EOF_CHECK_ICSS_REV2	process_rx_eof_tx_nb    ;check for EOF for recieve buffer if recieve is active
    .endif	;ICSS_REV2

    QBBC	QUEUE_WRAP_XMT, R13, 1	 ;check if buffer increment flag is cleared
    ; if buffer descriptor == top most buffer descriptor than set it to base value else increment it
    QBNE	NO_QUEUE_WRAP_XMT, BUFFER_DESC_OFFSET, TOP_MOST_BUFFER_DESC_OFFSET
    AND BUFFER_DESC_OFFSET , BASE_BUFFER_DESC_OFFSET , BASE_BUFFER_DESC_OFFSET
    CLR	R13 , R13 , INCREMENT_WRK_BUFFER_DESC_OFFSET
    .if    $defined("TWO_PORT_CFG")
    QBBS    QUEUE_WRAP_XMT, R13, PACKET_FROM_COLL_QUEUE    ;PACKET_FROM_COLL_QUEUE
    .endif ;TWO_PORT_CFG
    AND BUFFER_INDEX , BUFFER_OFFSET , BUFFER_OFFSET
    JMP		QUEUE_WRAP_XMT

NO_QUEUE_WRAP_XMT:
    ADD	BUFFER_DESC_OFFSET, BUFFER_DESC_OFFSET, 4	; working rd_ptr
    CLR	R13 , R13 , INCREMENT_WRK_BUFFER_DESC_OFFSET
QUEUE_WRAP_XMT:
    SUB	block_size, Packet_Length, BYTE_CNT	;find out how many bytes are left
    QBLT	PUSH_NB, block_size, 32
    LDI	SHIFT_REG, SHIFT_NONE
    .if $defined("PRU0")
    XOUT	BANK1, &MII_TX_CONTEXT, 20
    .else
    XOUT	BANK2, &MII_TX_CONTEXT, 20
    .endif
    JMP		XMT_LB

PUSH_NB:

    ; Check whether BUFFER_INDEX is pointing to the top desc in TX Queue
    QBLE	fetch_data_from_ocmc, TOP_MOST_BUFFER_INDEX, BUFFER_INDEX
    ; Subtract the bytes which have been already transmitted
    SUB	TEMP_REG_3.b0, BUFFER_INDEX, TOP_MOST_BUFFER_INDEX
    RSB	TEMP_REG_3.b1, TEMP_REG_3.b0, 32
    QBLE	enough_data_in_top_block, TEMP_REG_3.b1, FREE_SPACE_IN_FIFO ;if space is less than bytes to be sent than jump else continue
    AND FREE_SPACE_IN_FIFO , TEMP_REG_3.b1 , TEMP_REG_3.b1
enough_data_in_top_block:
fetch_data_from_ocmc:

    ; Check if the RX EOF has come
    LBCO	&BUFFER, L3_OCMC_RAM_CONST, BUFFER_INDEX, 32        ;load new buffer data

    .if $defined(PTP)
    LDI    TEMP_REG_1.w0, PTP_IPV4_UDP_E2E_ENABLE
    LBCO   &TEMP_REG_1.b0, ICSS_SHARED_CONST, TEMP_REG_1.w0, 1
    QBEQ   PTP_NOT_ENABLED_TX_NB, TEMP_REG_1.b0, 0
    JAL     R0.w0, FN_PTP_TX_ADD_DELAY_UDP
PTP_NOT_ENABLED_TX_NB:
    .endif

    ; Insert Tx Data in the Tx Fifo
    LDI	TX_DATA_POINTER, buffer_ptr
    .if $defined("ICSS_REV1")
        ; push free_space_fifo bytes more and update the flags
    QBEQ	PUSH_ONE_BYTE, FREE_SPACE_IN_FIFO, 1
    LSR	loop_cnt, FREE_SPACE_IN_FIFO, 1
    LOOP	EndLoop_NB, loop_cnt
    .if !$defined("TX_L2_ENABLED")
    MVIW	TX_DATA_WORD, *TX_DATA_POINTER
    M_PUSH_WORD_CMD
    .else
	MVIW	R2.w0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 2
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 2
EndLoop_NB:
    .endif	;ICSS_REV1

    .if $defined("ICSS_REV2")
        ; push free_space_fifo bytes more and update the flags
    LSR	loop_cnt, FREE_SPACE_IN_FIFO, 2	;divide by 4
    LOOP	EndLoop_NB, loop_cnt
    .if !$defined("TX_L2_ENABLED")
    MVID	TX_DATA_DOUBLE_WORD, *TX_DATA_POINTER
    .else
	MVID	R2, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 4
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 4
EndLoop_NB:
    .endif	;ICSS_REV2

    .if $defined("HALF_DUPLEX_ENABLED")
        ;load the value of PRUSS_MII_RT_PRS0 register and check for carrier sense
    QBBC	NO_COLLISION4, R22 , PORT_IS_HALF_DUPLEX
    LBCO	&TEMP_REG_1, MII_RT_CFG_CONST, MII_CARRIER_SENSE_REG, 1
    QBBC	NO_COLLISION4, TEMP_REG_1, 0
    QBA     COLLISION_DETECTED
NO_COLLISION4:
    .endif ;HALF_DUPLEX_ENABLED

;Check if any bytes are left to be added to TX FIFO.
    .if $defined("ICSS_REV1")
    QBBC	FIFO_INSERTION_OVER, FREE_SPACE_IN_FIFO, 0
PUSH_ONE_BYTE:
    .if !$defined("TX_L2_ENABLED")
    MVIB	TX_DATA_BYTE, *TX_DATA_POINTER
    M_PUSH_BYTE            ;write 1 byte in output fifo
    .else
	MVIB	R2.b0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 1
    .endif ;TX_L2_ENABLED
    .endif	;ICSS_REV1

    .if $defined("ICSS_REV2")
    AND	R20.b0, FREE_SPACE_IN_FIFO, 0x03	; Pick only last two bits
    QBEQ	FIFO_INSERTION_OVER, R20.b0, 0
    QBEQ	PUSH_ONE_BYTE, R20.b0, 1
    .if !$defined("TX_L2_ENABLED")
    MVIW	TX_DATA_WORD, *TX_DATA_POINTER
    .else
	MVIW	R2.w0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 2
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 2
    QBEQ	FIFO_INSERTION_OVER, R20.b0, 2
PUSH_ONE_BYTE:
        ;check how many bytes left and fill in the rest of the bytes based on what is left
    .if !$defined("TX_L2_ENABLED")
    MVIB	TX_DATA_BYTE, *TX_DATA_POINTER
    .else
	MVIB	R2.b0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 1
    .endif ;TX_L2_ENABLED
    .endif	;ICSS_REV2

FIFO_INSERTION_OVER:
        ;check if bytes transferred in last call was more than 32 bytes or not
        ;and based on it increment the byte count or move to next descriptor
    ADD	BYTES_TRANSFERRED_IN_LAST_CALL, BYTES_TRANSFERRED_IN_LAST_CALL, FREE_SPACE_IN_FIFO
    QBGT	DO_NOT_INCREMENT_BUFFER_DESC_OFFSET, BYTES_TRANSFERRED_IN_LAST_CALL, 32
    SET	R13 , R13 , INCREMENT_WRK_BUFFER_DESC_OFFSET
    SUB	BYTES_TRANSFERRED_IN_LAST_CALL, BYTES_TRANSFERRED_IN_LAST_CALL, 32
DO_NOT_INCREMENT_BUFFER_DESC_OFFSET:
    ADD	BUFFER_INDEX, BUFFER_INDEX, FREE_SPACE_IN_FIFO
    ADD	BYTE_CNT, BYTE_CNT, FREE_SPACE_IN_FIFO

    LDI	SHIFT_REG, SHIFT_NONE
        ;store back value in the bank
    .if $defined("PRU0")
    XOUT	BANK1, &MII_TX_CONTEXT, 20
    .else
    XOUT	BANK2, &MII_TX_CONTEXT, 20
    .endif
XMT_NB_DONE:
    JMP		TASK_EXECUTION_FINISHED
process_rx_eof_tx_nb:
    JAL	CALL_REG, FN_RCV_LB

error_in_fill_level:
        ;if some error occurred than halt the code else reset tx and continue
    .if $defined("DEBUG")
    HALT
    .else
    ;This is debug code, flow doesn't come here if there is no error.
    M_TX_RESET							;Reset the Tx Fifo
    CLR	R23 , R23 , 0 ; clear global flag to indicate the completion of transmission
    JMP		TASK_EXECUTION_FINISHED
    .endif

;****************************************************************************
;
;     NAME			: FN_XMT_LB
;     DESCRIPTION	: transmit the last block of data from the frame in TX FIFO
;     RETURNS		:
;     ARGS			:
;     USES 		:
;     INVOKES 		:
;
;****************************************************************************
XMT_LB:
        ;check the number of bytes left to send, if none left than exit.
        ;If Free space is less than current byte count then jump to next procedure
    SUB	size, Packet_Length, BYTE_CNT
    QBEQ	XMT_OVER, size, 0
    ADD	R11.b0, size, 4
    QBGE	PUSH_LB_FIFO, R11.b0, FREE_SPACE_IN_FIFO
    SET	R22 , R22 , Entire_Tx_Data_Not_Pushed
    QBGE	PUSH_LB_FIFO, size, FREE_SPACE_IN_FIFO
    AND size , FREE_SPACE_IN_FIFO , FREE_SPACE_IN_FIFO
PUSH_LB_FIFO:
    ; Check whether BUFFER_INDEX is pointing to the top desc in TX Queue
    QBLE	fetch_data_from_ocmc_lb, TOP_MOST_BUFFER_INDEX, BUFFER_INDEX
    ; Subtract the bytes which have been already transmitted
    SUB	TEMP_REG_3.b0, BUFFER_INDEX, TOP_MOST_BUFFER_INDEX
    RSB	TEMP_REG_3.b1, TEMP_REG_3.b0, 32
    QBLE	enough_data_in_top_block_lb, TEMP_REG_3.b1, size
    AND size , TEMP_REG_3.b1 , TEMP_REG_3.b1
enough_data_in_top_block_lb:
fetch_data_from_ocmc_lb:

    .if $defined("HALF_DUPLEX_ENABLED")
        ;load the value of PRUSS_MII_RT_PRS0 register and check for carrier sense.
        ;In these scenerio this will be late collision
    QBBC	NO_COLLISION5, R22 , PORT_IS_HALF_DUPLEX
    LBCO	&TEMP_REG_1, MII_RT_CFG_CONST, MII_CARRIER_SENSE_REG, 1
    QBBC	NO_COLLISION5, TEMP_REG_1, 0
    SET	R22 , R22 , TX_LATE_COLLISION
    QBA     COLLISION_DETECTED
NO_COLLISION5:
    .endif ;HALF_DUPLEX_ENABLED

    ; Check if the RX EOF has come
    .if $defined("ICSS_REV1")
    M_XMT_RX_EOF_CHECK_ICSS_REV1	process_rx_eof_tx_lb
    .endif	;ICSS_REV1
    .if $defined("ICSS_REV2")
    M_XMT_RX_EOF_CHECK_ICSS_REV2	process_rx_eof_tx_lb
    .endif	;ICSS_REV2

    AND R0.b0 , size , size
    LBCO	&BUFFER, L3_OCMC_RAM_CONST, BUFFER_INDEX, b0    ;load the new amount of data which can  be sent to Tx fifo

    .if $defined("ICSS_REV1")
    LDI	TX_DATA_POINTER, buffer_ptr
    QBEQ	PUSH_LB_LASTBYTE, size, 1       ;if only 1 bytes check to next procedure
    LSR	loop_cnt, size, 1
    LOOP	EndLoop_LB, loop_cnt
    .if !$defined("TX_L2_ENABLED")
    MVIW	TX_DATA_WORD, *TX_DATA_POINTER
    M_PUSH_WORD_CMD
    .else
	MVIW	R2.w0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 2
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 2     ;push free_space_fifo bytes more and update the flags
EndLoop_LB:
    QBBC	LB_OVER, size, 0
PUSH_LB_LASTBYTE:
    .if !$defined("TX_L2_ENABLED")
    MVIB	TX_DATA_BYTE, *TX_DATA_POINTER	        ;write the final byte in the tx fifo
    M_PUSH_BYTE
    .else
	MVIB	R2.b0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 1
    .endif ;TX_L2_ENABLED
    .endif	;ICSS_REV1

    .if $defined("ICSS_REV2")
    LDI	TX_DATA_POINTER, buffer_ptr
    LSR	loop_cnt, size, 2	;divide by 4
    LOOP	EndLoop_LB, loop_cnt
    .if !$defined("TX_L2_ENABLED")
    MVID	TX_DATA_DOUBLE_WORD, *TX_DATA_POINTER
    .else
	MVID	R2, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 4
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 4         ;push free_space_fifo bytes more and update the flags
EndLoop_LB:
    AND	R20.b0, size, 0x03	; Pick only last two bits
    QBEQ	LB_OVER, R20.b0, 0
    QBEQ	PUSH_LB_LASTBYTE, R20.b0, 1
    .if !$defined("TX_L2_ENABLED")
    MVIW	TX_DATA_WORD, *TX_DATA_POINTER
    .else
    MVIW	R2.w0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 2
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 2
    QBEQ	LB_OVER, R20.b0, 2	    ;check how many bytes left and fill in the rest of the bytes based on what is left
PUSH_LB_LASTBYTE:
    .if !$defined("TX_L2_ENABLED")
    MVIB	TX_DATA_BYTE, *TX_DATA_POINTER
    .else
	MVIB	R2.b0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 1
    .endif ;TX_L2_ENABLED
    .endif	;ICSS_REV2

LB_OVER:
        ;check the number of bytes left to send, if none left than exit.
        ;If Free space is less than current byte count then jump to next procedure
    ADD	BYTE_CNT, BYTE_CNT, size
    SUB	FREE_SPACE_IN_FIFO, FREE_SPACE_IN_FIFO, size
    ADD	BUFFER_INDEX, BUFFER_INDEX, size
    ADD	BYTES_TRANSFERRED_IN_LAST_CALL, BYTES_TRANSFERRED_IN_LAST_CALL, size
    QBGT	NO_BD_OFFSET_INCR, BYTES_TRANSFERRED_IN_LAST_CALL, 32
    SUB	BYTES_TRANSFERRED_IN_LAST_CALL, BYTES_TRANSFERRED_IN_LAST_CALL, 32

        ;check if bytes transferred in last call was more than 32 bytes or not and
        ;based on it increment the byte count/set it to base or move to next descriptor
    QBNE	NO_QUEUE_WRAP_XMT_1, BUFFER_DESC_OFFSET, TOP_MOST_BUFFER_DESC_OFFSET
    AND     BUFFER_DESC_OFFSET , BASE_BUFFER_DESC_OFFSET , BASE_BUFFER_DESC_OFFSET
    .if    $defined("TWO_PORT_CFG")
    QBBS    QUEUE_WRAP_XMT_1, R13, 2    ;PACKET_FROM_COLL_QUEUE
    .endif ;TWO_PORT_CFG
    AND     BUFFER_INDEX , BUFFER_OFFSET , BUFFER_OFFSET
    JMP		QUEUE_WRAP_XMT_1
NO_QUEUE_WRAP_XMT_1:
    ADD	BUFFER_DESC_OFFSET, BUFFER_DESC_OFFSET, 4	; working rd_ptr for Tx Queue
QUEUE_WRAP_XMT_1:

NO_BD_OFFSET_INCR:
    ;Check whether we have transmitted all the bytes ..if not than call Tx Task again for this packet
    QBBS	no_xmt_over_check, R22, Entire_Tx_Data_Not_Pushed
    QBEQ	XMT_OVER, BYTE_CNT, Packet_Length
no_xmt_over_check:
    CLR	R22 , R22 , Entire_Tx_Data_Not_Pushed
    LDI	SHIFT_REG, SHIFT_NONE
    .if $defined("PRU0")
    XOUT	BANK1, &MII_TX_CONTEXT, 20
    .else
    XOUT	BANK2, &MII_TX_CONTEXT, 20
    .endif
    JMP		TASK_EXECUTION_FINISHED

XMT_OVER:
        ;check if link speed is 100 mbps or free space is more than 4 bytes than jump else continue
    QBBS	XMT_LB_1OOMbps_MODE, R23, TX_PHY_SPEED
    QBLE	XMT_LB_1OOMbps_MODE, FREE_SPACE_IN_FIFO, 4
    JMP	TASK_EXECUTION_FINISHED
XMT_LB_1OOMbps_MODE:
    ; Insert the CRC in the outgoing frame by writing to R30
    .if $defined("ICSS_REV1")
    M_XMT_INSERT_CRC_ICSS_REV1
    .endif	;ICSS_REV1
    .if $defined("ICSS_REV2")
    M_XMT_INSERT_CRC_ICSS_REV2
    .endif	;ICSS_REV2

    ; Don't allow insertion of next packet in TX Fifo unless it is empty
    CLR	R22 , R22 , PACKET_TX_ALLOWED
    CLR	R22 , R22 , Entire_Tx_Data_Not_Pushed

    ;fix for issue where Tx FIFO goes into overflow and locks up
    ;when cable is removed during high traffic scenario

    ;Detect FIFO overflow and reset
    .if $defined("ICSS_REV1")
        M_XMT_FILL_LEVEL_CALC_ICSS_REV1	 ;calculates Fill level and resets in case of underflow/overflow
    .endif
    .if $defined("ICSS_REV2")
        M_XMT_FILL_LEVEL_CALC_ICSS_REV2	;calculates Fill level and resets in case of underflow/overflow
    .endif

xmt_save_context:

    ; save the context back to the bank
    LDI	SHIFT_REG, SHIFT_NONE
    .if $defined("PRU0")
    XOUT	BANK1, &MII_TX_CONTEXT, 20
    .else
    XOUT	BANK2, &MII_TX_CONTEXT, 20
    .endif

    ; Set the Tx Stat Pend bit
    SET	R23 , R23 , TX_STAT_PEND
        ;finally write the final descriptor value that it has been sent
    QBEQ	QUEUE_WRAP_XMT_LB, BYTES_TRANSFERRED_IN_LAST_CALL, 0
    QBNE	NO_QUEUE_WRAP_XMT_LB, BUFFER_DESC_OFFSET, TOP_MOST_BUFFER_DESC_OFFSET
    AND     BUFFER_DESC_OFFSET , BASE_BUFFER_DESC_OFFSET , BASE_BUFFER_DESC_OFFSET
    JMP	QUEUE_WRAP_XMT_LB
NO_QUEUE_WRAP_XMT_LB:
    ADD	BUFFER_DESC_OFFSET, BUFFER_DESC_OFFSET, 4	; working rd_ptr for Tx Queue

QUEUE_WRAP_XMT_LB:

    .if    $defined("TWO_PORT_CFG")
    SBCO    &BUFFER_DESC_OFFSET, QUEUE_DESP_BASE, QUEUE_DESC_OFFSET, 2    ; update rd_ptr in the Queue desp.
    QBBC    XMT_NOT_FROM_COLLISION_QUEUE, R13, 2    ;PACKET_FROM_COLL_QUEUE
    ; Check and Update the wr_ptr of the collision Queue if there was a collision
    LDI	R20.b2, 0
    LDI	R20.w0, COLLISION_STATUS_ADDR
    .if    $defined("PRU0")
    LDI        QUEUE_DESC_OFFSET, P2_COL_QUEUE_DESC_OFFSET
    LBCO    &QUEUE_DESC_REG, QUEUE_DESP_BASE, QUEUE_DESC_OFFSET, 4
    ADD        QUEUE_DESC_OFFSET, QUEUE_DESC_OFFSET, 2
    SBCO    &QUEUE_DESC_REG.rd_ptr, QUEUE_DESP_BASE, QUEUE_DESC_OFFSET, 2
    ; Clear bit which indicates that data has been transferred from the collision buffer
    ADD	R20.w0, R20.w0, 2
    SBCO	&R20.b2, PRU1_DMEM_CONST, R20.w0, 1
    .else
    LDI        QUEUE_DESC_OFFSET, P1_COL_QUEUE_DESC_OFFSET
    LBCO    &QUEUE_DESC_REG, QUEUE_DESP_BASE, QUEUE_DESC_OFFSET, 4
    ADD        QUEUE_DESC_OFFSET, QUEUE_DESC_OFFSET, 2
    SBCO    &QUEUE_DESC_REG.rd_ptr, QUEUE_DESP_BASE, QUEUE_DESC_OFFSET, 2
    ; Clear bit which indicates that data has been transferred from the collision buffer
    ADD	R20.w0, R20.w0, 1
    SBCO	&R20.b2, PRU1_DMEM_CONST, R20.w0, 1
    .endif
    .else
    SBCO	&BUFFER_DESC_OFFSET, PRU_DMEM_ADDR, QUEUE_DESC_OFFSET, 2	; update rd_ptr in the Queue desp.

    .endif ;TWO_PORT_CFG
XMT_NOT_FROM_COLLISION_QUEUE:


SKIP_DESC_UPDATE:
    CLR	R23 , R23 , Xmt_active ; clear global flag to indicate the completion of transmission
    .if $defined(PTP)
    M_GPTP_SET_CALLBACK_INTERRUPT
    .endif ;PTP
    .if $defined("HALF_DUPLEX_ENABLED")
    QBBC	DO_NOT_UPDATE_COLLISION_STATS, R22 , PORT_IS_HALF_DUPLEX

    ;load collision counter
    LDI	TEMP_REG_1 , COLLISION_COUNTER
    LBCO	&TEMP_REG_2, PRU_DMEM_ADDR, TEMP_REG_1, 1

;Update collision status
    QBLT	UPDATE_MULTIPLE_COLLISION, TEMP_REG_2, 1
    QBEQ	DO_NOT_UPDATE_COLLISION_STATS, TEMP_REG_2, 0
    LDI	TEMP_REG_1 , SINGLE_COLLISION_OFFSET
    QBA     ADD_TX_STATS
UPDATE_MULTIPLE_COLLISION:
    QBLT	UPDATE_EXCESS_COLLISION, TEMP_REG_2, 15
    LDI	TEMP_REG_1 , MULTIPLE_COLLISION_OFFSET
    QBA     ADD_TX_STATS

UPDATE_EXCESS_COLLISION:
    LDI	TEMP_REG_1 , EXCESS_COLLISION_OFFSET;Reset counter for collision
    QBA     ADD_TX_STATS

DO_NOT_UPDATE_COLLISION_STATS:
    .endif ;HALF_DUPLEX_ENABLED
    JMP	TASK_EXECUTION_FINISHED

process_rx_eof_tx_lb:
    JAL	CALL_REG, FN_RCV_LB

    .endif  ; __mii_xmt_p


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
; file:  emac_MII_Rcv.h
;
; brief:	Defines and macros to be used in receive task.
;
;
;  (C) Copyright 2017-2019, Texas Instruments, Inc
;
;

	.if !$defined("__mii_rcv_hp")
__mii_rcv_hp	.set	1

	.if $defined("ICSS_DUAL_EMAC_BUILD")
	.include "icss_emacSwitch.h"
    .cdecls C,NOLIST
%{
#include "icss_vlan_mcast_filter_mmap.h"
%}
	.endif ; MULTICAST Filtering
	.if $defined("ICSS_SWITCH_BUILD")
        .include "icss_switch.h"

	.if $defined("ICSS_STP_SWITCH")
        .cdecls C,NOLIST
%{
#include "icss_vlan_mcast_filter_mmap.h"
%}
	.endif ; include vlan/multicast filtering

	.endif
	.include "icss_defines.h"
    .cdecls C,NOLIST
%{
#include "icss_rx_int_pacing_mmap.h"
%}

; MII receive structure definition
	.asg	t0, host_rcv_flag		; rcv will place frame into host queue
host_rcv_flag_shift	.set	0
	.asg	t1, fwd_flag		; rcv will place frame into port queue
fwd_flag_shift	.set	1
	.asg	t2, ct_bank_index		; indication for that data is either in R2..R5 or R6..R9; used only by rx function
ct_bank_index_shift	.set	2
	.asg	t3, rx_bank_index		; indication for rcv of where to get the next 32 bytes of data (bank0/1)
rx_bank_index_shift	.set	3
	.asg	t4, rx_frame_error		; used to indicate a frame error to Xmt function
rx_frame_error_shift	.set	4
	.asg	t5, port_collision_queue_selected		; will indicate if the port collision queue has been selected
port_collision_queue_selected_shift	.set	5
	.asg	t6, host_collision_queue_selected		; will indicate if the host collision queue has been selected
host_collision_queue_selected_shift	.set	6
	.asg	t7, last_block_multi_bank_processing		; during last_block function, we need to process more than one bank of data
last_block_multi_bank_processing_shift	.set	7

	.asg	t0, cut_through_flag		; rcv in cut-through (16 byte) mode
cut_through_flag_shift	.set	0
	.asg	t1, ct_data_available		; indication for Xmt that new data is in bank0 (scratchpad)
ct_data_available_shift	.set	1
	.asg	t2, ct_last_block		; indication for Xmt that this is the last block of data, R18.b0 provides the amount of data to transmit
ct_last_block_shift	.set	2

	.asg	t0, delay_resp
delay_resp_shift	.set	0
	.asg	t1, delay_resp_followup
delay_resp_followup_shift	.set	1
	.asg	t2, delay_followup_resp
delay_followup_resp_shift	.set	2
	.asg	t3, delay_request
delay_request_shift	.set	3
	.asg	t4, process_lb_part1_executed		; When processing multibank, we spilt it into two parts ..this bit indicate that 1st part is done.
process_lb_part1_executed_shift	.set	4
	.asg	t5, unicast_pkt_cut_through
unicast_pkt_cut_through_shift	.set	5
	.asg	t6, host_queue_overflow
host_queue_overflow_shift	.set	6
	.asg	t7, port_queue_overflow
port_queue_overflow_shift	.set	7

; MII structure for receive register descriptor
MII_RCV_DESC	.struct
rx_flags	.ubyte
tx_flags	.ubyte
rx_flags_extended	.ubyte
qos_queue	.ubyte	; for host/port queue, this stores the priority queue as determined by the frame
byte_cntr	.ushort	; total amount of bytes received
wrkng_wr_ptr	.ushort	; same as port, but points to the host buffer
rd_ptr	.ushort
buffer_index	.ushort	;offset in L3 where data is to be stored
base_buffer_index	.ushort	; offset in L3 of 32 bytes data correspondin to first buffer_desc
rcv_queue_pointer	.ushort	; host queue  pointer
base_buffer_desc_offset	.ushort	; offset of the base buffer desc for the queue selected
top_most_buffer_desc_offset	.ushort	; offset of the last/top buffer desc for the queue selected
	.endstruct

; MII structure for PORT receive
MII_RCV_PORT_DESC	.struct
byte_cntr	.ushort	; total amount of bytes received
wrkng_wr_ptr	.ushort	; same as port, but points to the host buffer
rd_ptr	.ushort
buffer_index	.ushort	; offset in L3 where data is to be stored
base_buffer_index	.ushort	; offset in L3 of 32 bytes data correspondin to first buffer_desc
rcv_queue_pointer	.ushort	; port queue  pointer
base_buffer_desc_offset	.ushort	; offset of the base buffer desc for the queue selected
top_most_buffer_desc_offset	.ushort	; offset of the last/top buffer desc for the queue selected
	.endstruct

	.if $defined("ICSS_SWITCH_BUILD")
; add MII structure for CPM receive
MII_RCV_CPM_DESC    .struct
rx_flags    .ubyte
tx_flags    .ubyte
Buffer_Offset    .ushort                    ; current offset into L3 CPM buffer for write operation
Desc_Offset    .ushort                        ; offset to current CPM buffer descriptor
byte_cntr    .ushort                        ; current number of bytes received
CPM_Flags    .ubyte                            ; flags for CPM processing
CPM_NUMBER    .ubyte
CPM_idle2    .ushort
CPM_idle3    .uint
    .endstruct

    .asg    t0    ,    VLAN_tag    ; indicates that received CPM frame is tagged
    .asg    t1    ,    BC    ; indicates buffer complete to host. This flag is cleared by PRU when set by host only
    .asg    t2    ,    idx1    ; buffer index = 1
    .asg    t3    ,    idx2    ; buffer index = 2
    .asg    t4    ,    CPM_Error    ; indicates received error which can be CRC, Length,
	.endif  ;ICSS_SWITCH_BUILD

	.asg	R2.w0, RX_CONTEXT_OFFSET
	.asg	R2.w2, RX_PORT_CONTEXT_OFFSET

	.asg	R18.b0, R18_RCV_BYTECOUNT			; wrt pointer of L2 buffer, has amount of data
;	.asg	R23.t0, Xmt_active			; global flag that indicates if transmission is ongoing
Xmt_active							.set 	0
;	.asg	R23.t1, Rcv_active			; global flag that indicates if reception is ongoing
Rcv_active							.set 	1
;	.asg	R23.t2, TX_STAT_PEND
TX_STAT_PEND						.set 	2
;	.asg	R23.t3, RX_STAT_PEND
RX_STAT_PEND						.set 	3
;	.asg	R23.t4, TX_DELAY_REQ      	; This flag is just for debug ...set when pkt is received and cleared in RX_FB and STATS
TX_DELAY_REQ						.set 	4
;	.asg	R23.t5, TX_DELAY_RESP
TX_DELAY_RESP						.set 	5
;	.asg	R23.t6, TX_DELAY_FOLLOWUP_RESP
TX_DELAY_FOLLOWUP_RESP				.set 	6

;	.asg	R23.t7, TX_PHY_SPEED   	 	; If 0 for 10 Mbps. 1 for 100 Mbps.
TX_PHY_SPEED						.set 	7

	.asg	R23.b1, PREVIOUS_R18_RCV_BYTECOUNT
	.asg	R23.w2, CUT_THROUGH_BYTE_CNT

	.asg	R21, COLLISION_STATUS_REG
	.asg	R20, RCV_TEMP_REG_1			; buffer need to be in consecutive register space
	.asg	R21, RCV_TEMP_REG_2
	.asg	R13, RCV_TEMP_REG_3
	.asg	R12, RCV_TEMP_REG_4
	.asg	R21, RCV_BUFFER_DESC_OFFSET
	.asg	R6.t0, CODE_EXECUTING_FOR_PORT_RECEIVE

PORT_COLLISION_BIT					.set	2

QPR_ERROR_NO_SPACE_IN_QUEUE			.set	0xff
QPR_ERROR_BYTES_TO_WRITE_IS_ZERO	.set	0xfe

	.asg	RCV_TEMP_REG_2.w0, FORWARD_TABLE_POINTER
	.asg	RCV_TEMP_REG_2.w0, RECEIVE_TABLE_POINTER

; macro defines
Q_RD_PTR_OFFSET				.set	0
Q_RD_PTR_SIZE				.set	2
Q_WR_PTR_OFFSET				.set	2
Q_WR_PTR_SIZE				.set	2
Q_BUSY_S_OFFSET				.set	4
Q_BUSY_S_SIZE				.set	1
Q_STATUS_OFFSET				.set	5
Q_STATUS_SIZE				.set	1

Q_BUSY_S_BIT				.set	0
Q_BUSY_M_BIT				.set	0
Q_COLLISION_BIT				.set	1
Q_OVERFLOW_BIT				.set	2

QUEUE_FAILED				.set	0
QUEUE_AQUIRED				.set	1
COLLISION_AQUIRED			.set	2

; RX L2 status register specific meanings
	.asg	t0, RXL2_RX_ERR_BIT
	.asg	t1, RXL2_STATUS_RDY_BIT
	.asg	t2, RXL2_RX_ERROR_BIT
	.asg	t3, RXL2_RX_EOF_BIT
	.asg	t4, RXL2_RX_SFD_BIT
	.asg	t5, RXL2_RX_SOF_BIT
	.asg	t6, RXL2_ERROR_NIBBLE
	.asg	t7, RXL2_ERROR_CRC

RANGE_R2_R5					.set	(4 * 4)
RANGE_R2_R6					.set	(4 * 5)
RANGE_R6_R9					.set	(4 * 4)
RANGE_R2_R9					.set	(4 * 8)   ; used by XFER, amount of bytes from R2 - R9
RANGE_R2_R13				.set	(4 * 12)
RANGE_R2_R18				.set	(4 * 17)  ; used by XFER, amount of bytes from R2 - R18
RANGE_R10_R18				.set	(4 * 9)
RANGE_R10_R13				.set	(4 * 4)
RANGE_R18_b0				.set	1

	.asg	R0.b0, SHIFT_REG
SHIFT_NONE					.set	0
SHIFT_R2_TO_R5    			.set	3        ; dest(2+3)
SHIFT_R2_TO_R6    			.set	4        ; dest(2+4)
SHIFT_R2_TO_R8    			.set	6        ; dest(2+6)
SHIFT_R2_TO_R10    			.set	8        ; dest(2+8)
SHIFT_R2_TO_R17    			.set	15       ; dest(2+15)
SHIFT_R2_TO_R0    			.set	28       ; dest(2+28-30)
SHIFT_R7_TO_R18				.set	11
SHIFT_R2_TO_R18				.set	16
; Below are used for Port Receive code
SHIFT_R14_TO_R0    			.set	16       ; dest(14+16-30)  ; For PRU0
SHIFT_R14_TO_R4    			.set	20       ; dest(14+20-30)  ; For PRU1
SHIFT_R14_TO_R8    			.set	24       ; dest(14+24-30)  ; For PRU0
SHIFT_R10_TO_R6    			.set	26       ; dest(10+26-30)
SHIFT_R10_TO_R8    			.set	28       ; dest(10+28-30)  ; For PRU0
SHIFT_R9_TO_R0    			.set	21       ; dest(9+21-30) ;for intercom

SHIFT_R21_TO_R10    		.set	19       ; dest(21+19-30)  ; For PRU0

SHIFT_R12_TO_R9    			.set	27       ; dest(12+27-30)  ; For PRU0
SHIFT_R12_TO_R10    		.set    28       ; dest(12+28-30)  ; For PRU1

SHIFT_R14_TO_R18    		.set	4        ; dest(14+4-30)  ; For PRU0
SHIFT_R14_TO_R20    		.set	6        ; dest(14+6-30)  ; For PRU0
SHIFT_R18_TO_R16    		.set	28       ; dest(18+28-30)
SHIFT_R18_TO_R25    		.set	7        ; dest(18+7)

SHIFT_R2_TO_R26    			.set	24       ; dest(26+6-30)
SHIFT_R2_TO_R2    			.set	0        ; dest(0+2)
SHIFT_R2_TO_R4    			.set	2        ; dest(4+28-30)

; TPID = tag protocol ID, typ 8100 for tagged frames
; TCI  = tag control info with user priority in 3 MSBs
struct_Ethernet	.struct
DstAddr_0123	.uint	;R2
DstAddr_45	.ushort	;R3
SrcAddr_01	.ushort
SrcAddr_23	.ushort	;R4
SrcAddr_45	.ushort
TPID	        .ushort	;R5
TCI .union
x_b .struct
b0              .ubyte
b1              .ubyte
        .endstruct
        .endunion ; TCI
ProtWord2	.ushort	;R6
ProtWord3	.ushort
ProtWord4	.ushort	;R7
ProtWord5	.ushort
ProtWord6	.ushort	;R8
ProtWord7	.ushort
ProtWord8	.ushort	;R9
ProtWord9	.ushort
	.endstruct

	.asg	t0, multicast					; multicast bit in DstByte_0
struct_EthByte	.struct
DstByte_0	.ubyte
DstByte_1	.ubyte
DstByte_2	.ubyte
DstByte_3	.ubyte
DstByte_4	.ubyte
DstByte_5	.ubyte
SrcByte_0	.ubyte
SrcByte_1	.ubyte
SrcByte_2	.ubyte
SrcByte_3	.ubyte
SrcByte_4	.ubyte
SrcByte_5	.ubyte
	.endstruct

    .if $defined("ICSS_STP_SWITCH")
struct_EthWord	.struct
DstWord_01	.ushort	;R2
DstWord_23	.ushort
DstWord_45	.ushort	;R3
SrcWord_01	.ushort
SrcWord_23	.ushort	;R4
SrcWord_45	.ushort
        .endstruct
    .endif ; ICSS_STP_SWITCH

	.asg	R25, RCV_REGISTER_1
	.asg	R29, RCV_REGISTER_2

Ethernet			.sassign	 R2	,	struct_Ethernet
EthByte	        	.sassign	 R2	,	struct_EthByte

    .if $defined("ICSS_STP_SWITCH")
EthWord   .sassign    R2, struct_EthWord
    .endif ; ICSS_STP_SWITCH

MII_RCV	        	.sassign	 RCV_REGISTER_1	,	MII_RCV_DESC	; This context is for Host Receive
MII_RCV_PORT		.sassign	 R14 ,	MII_RCV_PORT_DESC	; This context is for Port Receive
RCV_CONTEXT			.sassign	 R2 ,	MII_RCV_PORT_DESC
RCV_QUEUE_DESC_REG	.sassign	 R20 ,	struct_queue

	.asg	R5, PROTOCOL_TYPE_FRAME_ID
PTP_DELAY_REQUEST				.set	0x40ff9288
PTP_DELAY_RESPONSE				.set	0x43ff9288
PTP_DELAY_RESPONSE_FOLLLOWUP	.set	0x41ff9288
PTP_DELAY_FOLLOWUP_RESPONSE		.set	0x42ff9288


;------------------------------------------------------------------------------
; Macro Name: M_MULTICAST_TABLE_SEARCH_OP
; Description:  Multicast table lookup
; Input Parameters:  -
; Use of registers: RCV_TEMP_REG_3/4
; Peak PRU cycles : 16
; output registers : -
;------------------------------------------------------------------------------
M_MULTICAST_TABLE_SEARCH_OP  .macro

; load multicast mask offset | one time operation => register can be reused
; load the mask 48 bits (6 bytes)
    LDI    RCV_TEMP_REG_4.w0, ICSS_EMAC_FW_MULTICAST_FILTER_MASK_OFFSET
    LBCO   &RCV_TEMP_REG_4.w0, PRU_DMEM_ADDR, RCV_TEMP_REG_4.w0, 6
;----------------------------------------
;               MASK UPPER              | R12 | RCV_TEMP_REG_4
;----------------------------------------
;    MASK LOWER     |                   | R13 | RCV_TEMP_REG_3
;----------------------------------------

; Perform AND operation on the MAC ID with the mask | replace the mask
;   32 bit AND b/w RCV_TEMP_REG_4 & DstAddr upper
;   16 bit AND b/w RCV_TEMP_REG_3.w0 & DstAddr lower
    AND    RCV_TEMP_REG_4, RCV_TEMP_REG_4, Ethernet.DstAddr_0123
    AND    RCV_TEMP_REG_3.w0, RCV_TEMP_REG_3.w0, Ethernet.DstAddr_45
;----------------------------------------
;         MAC ID MASKED UPPER           | R12 | RCV_TEMP_REG_4
;----------------------------------------
; MAC ID MASKED LOWER |                 | R13 | RCV_TEMP_REG_3
;----------------------------------------

; XOR the 6 bytes of the MAC ID MASKED to obtain hash
; optimised algo to obtain hash in 3 iterations avaiable at https://sps05.itg.ti.com/sites/tiisw/bu/armmpu/Documents/
; Industrial%20FieldBus/HSR_PRP/Add%20Multicast%20filtering%20support%20to%20HSR%20PRP/XOR%20comparison.pptx
; This XOR result will be used as an index into MULTICAST_FILTER_TABLE
    XOR    RCV_TEMP_REG_4.w0, RCV_TEMP_REG_4.w0, RCV_TEMP_REG_4.w2
    XOR    RCV_TEMP_REG_4.w0, RCV_TEMP_REG_4.w0, RCV_TEMP_REG_3.w0
    XOR    RCV_TEMP_REG_4.b0, RCV_TEMP_REG_4.b0, RCV_TEMP_REG_4.b1
;-----------------------------------------
; 8-BIT-XOR-RESULT |      |       |      | R12 | RCV_TEMP_REG_4
;-----------------------------------------

; MULTICAST_FILTER_TABLE size = 1 byte per entry * 256 entries = 256 bytes
; Lookup the byte in MULTICAST_FILTER_TABLE using the 8-BIT-XOR-RESULT as index | index in RCV_TEMP_REG_4.b0

; Load multicast lookup table base address | one time operation => register can be reused
    LDI    RCV_TEMP_REG_3.w0, ICSS_EMAC_FW_MULTICAST_FILTER_TABLE
;-------------------------------------------------------
;  8-BIT-XOR-RESULT   |        |            |          | R12 | RCV_TEMP_REG_4
;-------------------------------------------------------
;    MULTICAST_FILTER_TABLE    |            |          | R13 | RCV_TEMP_REG_3
;-------------------------------------------------------

; byte offset = MULTICAST_FILTER_TABLE + 8-BIT-XOR-RESULT
; load the byte into BYTE-LOADED
    ADD    RCV_TEMP_REG_3.w0, RCV_TEMP_REG_3.w0, RCV_TEMP_REG_4.b0
    LBCO   &RCV_TEMP_REG_3.b3, PRU_DMEM_ADDR, RCV_TEMP_REG_3.w0, 1
;--------------------------------------------------------------------
;       8-BIT-XOR-RESULT     |              |         |             | R12 | RCV_TEMP_REG_4
;--------------------------------------------------------------------
; MULTICAST_FILTER_TABLE + 8-BIT-XOR-RESULT |         | BYTE-LOADED | R13 | RCV_TEMP_REG_3
;--------------------------------------------------------------------

; The functionality is defined as below
;   =0 | MULTICAST_FILTER_HOST_RCV_NOT_ALLOWED : no MAC ID added to this bin => do not allow packet to host
;   =1 | MULTICAST_FILTER_HOST_RCV_ALLOWED : MAC ID added to this bin => allow packet to host
;
; if BYTE-LOADED is 1 => host recieve | else => no host recieve
; host_rcv_flag_shift bit is already SET. CLR it if host recieve needs to be disabled
    QBEQ   M_MULTICAST_TABLE_SEARCH_OP_END, RCV_TEMP_REG_3.b3, ICSS_EMAC_FW_MULTICAST_FILTER_HOST_RCV_ALLOWED

    CLR    MII_RCV.rx_flags, MII_RCV.rx_flags, host_rcv_flag_shift
    LDI    RCV_TEMP_REG_4.w2, ICSS_EMAC_FW_MULTICAST_FILTER_DROP_CNT_OFFSET
    LBCO    &RCV_TEMP_REG_3, PRU_DMEM_ADDR, RCV_TEMP_REG_4.w2, 4
    ADD    RCV_TEMP_REG_3, RCV_TEMP_REG_3, 1
    SBCO    &RCV_TEMP_REG_3, PRU_DMEM_ADDR, RCV_TEMP_REG_4.w2, 4

M_MULTICAST_TABLE_SEARCH_OP_END:

  .endm

  ;------------------------------------------------------------------------------
  ; Macro Name: M_VLAN_FLTR_SRCH_OP
  ; Description:  VLAN table lookup
  ; Input Parameters : -
  ; Use of registers :
  ; Peak PRU cycles  :
  ; output registers : -
  ;------------------------------------------------------------------------------
M_VLAN_FLTR_SRCH_OP  .macro

  ;-----------------------------------------------------------
  ;   VLAN_FLTR_CTRL_BYTE  |          |          |             | R13 | RCV_TEMP_REG_3
  ;-----------------------------------------------------------

  ; check for VLAN tag in Ethernet.TPID
  ; host_rcv_flag_shift bit is already SET
      QBNE     VLAN_FLTR_UNTAG_FLOW, Ethernet.TPID, 0x0081

  ; extract the VID value from Ethernet.TCI
  ; If the packet bytes were (81 00 12 34), R5 looks like 0x34 12 00 81
  ; So Ethernet.TPID has (0xb1b0) 0x0081 & Ethernet.TCI has (0xb3b2) 0x3412
  ; we need to swap the bytes in Ethernet.TCI & extarct the lower 12 bytes
  ;-----------------------------------------------------------
  ;      0x81       |    0x00      |    0x12    |    0x34     | R5
  ;-----------------------------------------------------------

      AND    RCV_TEMP_REG_4.b1, R5.b2, 0x0F      ; Ethernet.TCI.b0 = R5.b2
      AND    RCV_TEMP_REG_4.b0, R5.b3, R5.b3   ; Ethernet.TCI.b1 = R5.b3
  ;-----------------------------------------------------------
  ;      0x34       |     0x02      |            |             | R12 | RCV_TEMP_REG_4
  ;-----------------------------------------------------------
  ; VLAN_FLTR_CTRL_BYTE |          |            |             | R13 | RCV_TEMP_REG_3
  ;-----------------------------------------------------------

  ; VID is loaded in RCV_TEMP_REG_4.w0
  ; if VID=0, do priority frame handling
      QBEQ     VLAN_FLTR_PRIOTAG_FLOW, RCV_TEMP_REG_4.w0, ICSS_EMAC_FW_VLAN_FILTER_PRIOTAG_VID

  ; VLAN filter table size = 1 bit per entry * 4096 entries = 4096 bits = 512 bytes = 0x200 bytes
  ; Lookup the bit in VLAN filter table using the VID as index | index in RCV_TEMP_REG_4.w0

  ; Load VLAN filter table base address | one time operation => register can be reused
      LDI     RCV_TEMP_REG_3.w0, ICSS_EMAC_FW_VLAN_FLTR_TBL_BASE_ADDR
  ;-------------------------------------------------------
  ;               VID              |              |          | R12 | RCV_TEMP_REG_4
  ;-------------------------------------------------------
  ;    VLAN_FLTR_TBL_BASE_ADDR     |              |          | R13 | RCV_TEMP_REG_3
  ;-------------------------------------------------------

  ; Load the corresponding byte (byte_lookup) & bit (bit_lookup) for lookup using the calculation
  ; byte_lookup = VID / 8      | RCV_TEMP_REG_4.w2
  ; bit_lookup    = VID & 0x7   | RCV_TEMP_REG_3.b3
      LSR     RCV_TEMP_REG_4.w2, RCV_TEMP_REG_4.w0, 3
      AND     RCV_TEMP_REG_3.b3, RCV_TEMP_REG_4.w0, 0x7
  ;-------------------------------------------------------
  ;            VID            |         byte_lookup         | R12 | RCV_TEMP_REG_4
  ;-------------------------------------------------------
  ; VLAN_FLTR_TBL_BASE_ADDR   |                | bit_lookup | R13 | RCV_TEMP_REG_3
  ;-------------------------------------------------------

  ; Load the byte (byte_loaded) into RCV_TEMP_REG_3.b2
  ; byte_loaded = VLAN_FLTR_TBL_BASE_ADDR + byte_lookup
      ADD     RCV_TEMP_REG_3.w0, RCV_TEMP_REG_3.w0, RCV_TEMP_REG_4.w2
      LBCO     &RCV_TEMP_REG_3.b2, PRU_DMEM_ADDR, RCV_TEMP_REG_3.w0, 1
  ;---------------------------------------------------------------------------------
  ;            VID            |         byte_lookup           | R12 | RCV_TEMP_REG_4
  ;---------------------------------------------------------------------------------
  ; VLAN_FLTR_TBL_BASE_ADDR   |                 |             |
  ;   + byte_lookup           |  byte_loaded    |  bit_lookup | R13 | RCV_TEMP_REG_3
  ;---------------------------------------------------------------------------------

  ; The functionality is defined as below
  ;   =0 | do not allow packet to host
  ;   =1 | allow packet to host
  ;
  ; if byte_loaded is 1 => host recieve | else => no host recieve
  ; host_rcv_flag_shift bit is already SET. CLR it if host recieve needs to be disabled
      QBBS     M_VLAN_FLTR_SRCH_OP_END, RCV_TEMP_REG_3.b2, RCV_TEMP_REG_3.b3
      QBA     CLR_HOST_RCV_FLAG

VLAN_FLTR_PRIOTAG_FLOW:
      ; check if host receive for priority frames is enabled or not i.e. check VLAN_FLTR_PRIOTAG_HOST_RCV_CTRL_SHIFT in VLAN_FLTR_CTRL_BYTE stored in shared memory
      ; one bit field | 0 : priotag host rcv allowed | 1 : priotag host rcv not allowed
      ; host_rcv_flag_shift bit is already SET. CLR it if host recieve needs to be disabled
      QBBC     M_VLAN_FLTR_SRCH_OP_END, RCV_TEMP_REG_3.b0, ICSS_EMAC_FW_VLAN_FILTER_PRIOTAG_HOST_RCV_ALLOW_CTRL_BIT
      QBA     CLR_HOST_RCV_FLAG

VLAN_FLTR_UNTAG_FLOW:
      ; check if host receive for untagged frames is enabled or not i.e. check VLAN_FLTR_UNTAG_HOST_RCV_CTRL_SHIFT in VLAN_FLTR_CTRL_BYTE stored in shared memory
      ; one bit field | 0 : untagged host rcv allowed | 1 : untagged host rcv not allowed
      ; host_rcv_flag_shift bit is already SET. CLR it if host recieve needs to be disabled
      QBBC     M_VLAN_FLTR_SRCH_OP_END, RCV_TEMP_REG_3.b0, ICSS_EMAC_FW_VLAN_FILTER_UNTAG_HOST_RCV_ALLOW_CTRL_BIT

CLR_HOST_RCV_FLAG:
      CLR     MII_RCV.rx_flags, MII_RCV.rx_flags, host_rcv_flag_shift
      LDI     RCV_TEMP_REG_4.w2, ICSS_EMAC_FW_VLAN_FILTER_DROP_CNT_OFFSET
      LBCO     &RCV_TEMP_REG_3, PRU_DMEM_ADDR, RCV_TEMP_REG_4.w2, 4
      ADD     RCV_TEMP_REG_3, RCV_TEMP_REG_3, 1
      SBCO     &RCV_TEMP_REG_3, PRU_DMEM_ADDR, RCV_TEMP_REG_4.w2, 4

M_VLAN_FLTR_SRCH_OP_END:
    .endm ; M_VLAN_FLTR_SRCH_OP

;-----------------------------------
; Macro Name: M_RCV_RX_EOF_CHECK_ICSS_REV1
; Description: Check for RX EOF on ICSS_REV1.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_RCV_RX_EOF_CHECK_ICSS_REV1	 .macro
	.if $defined("PRU0")
	QBBS	process_rx_eof_rx_nb, R31, 30
	.else
	QBBS	process_rx_eof_rx_nb, R31, 31
	.endif
	.endm

;-----------------------------------
; Macro Name: M_RCV_RX_EOF_CHECK_ICSS_REV2
; Description: Check for RX EOF on ICSS_REV2.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_RCV_RX_EOF_CHECK_ICSS_REV2	 .macro
	QBBS	process_rx_eof_rx_nb, R31, 20
	.endm

;-----------------------------------
; Macro Name: M_RCV_RX_EOF_CLEAR_INTC_ICSS_REV1
; Description: Clear RX EOF system event in INTC on ICSS_REV1.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_RCV_RX_EOF_CLEAR_INTC_ICSS_REV1	 .macro
	LDI	TEMP_REG_4.w0, 0x0284	; Clear RX_EOF
	.if $defined("PRU0")
	LDI	TEMP_REG_3 , 0x00000430
	.else
	LDI	TEMP_REG_3.w0 , 0x00430000 & 0xFFFF
	LDI	TEMP_REG_3.w2 , 0x00430000 >> 16
	.endif
	SBCO	&TEMP_REG_3, ICSS_INTC_CONST, TEMP_REG_4.w0, 4
	.endm

;-----------------------------------
; Macro Name: M_RCV_RX_EOF_CLEAR_INTC_ICSS_REV2
; Description: Clear RX EOF and CRC system event in INTC on ICSS_REV2.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_RCV_RX_EOF_CLEAR_INTC_ICSS_REV2	 .macro
	LDI	TEMP_REG_4.w0, 0x0284	; Clear RX CRC
	.if $defined("PRU0")
	LDI	TEMP_REG_3 , 0x00000030
	.else
	LDI	TEMP_REG_3.w0 , 0x00030000 & 0xFFFF
	LDI	TEMP_REG_3.w2 , 0x00030000 >> 16
	.endif
	SBCO	&TEMP_REG_3, ICSS_INTC_CONST, TEMP_REG_4.w0, 4
	SET	R31 , R31 , 22 ; Clear RX_EOF
	.endm

;-----------------------------------
; Macro Name: M_RCV_RX_EOF_INTERRUPT_RAISE_INTC
; Description: Raise the interrupt in the intc controller for host.
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
M_RCV_RX_EOF_INTERRUPT_RAISE_INTC	 .macro
        .if    $defined("TWO_PORT_CFG")
        ; Give a interrupt to host as frame has been received
        LDI        R31, 0x24       ; Maps to system event 20, 0x24 = 10 0100
        .else
        ; Rx interrupt pacing
        ; Check INTR_PAC_STATUS_OFFSET for Interrupt Pacing logic status
        ;   if = 0 | INTR_PAC_DIS_ADP_LGC_DIS => skip HOST_QUEUE_EMPTY_STATUS check & fire interrupt
        ;   else if = 1 | INTR_PAC_ENA_ADP_LGC_DIS => set R22 INTR_TO_HOST_PENDING_RX & goto LB_DONE
        ;   else perform HOST_QUEUE_EMPTY_STATUS check
        ;       if HOST_QUEUE_EMPTY_STATUS is SET => fire interrupt
        ;       else set R22 INTR_TO_HOST_PENDING_RX & goto LB_DONE
        ;
        ; NOTE : no need to CLR R22 bit HOST_QUEUE_EMPTY_STATUS as R22.w0 gets cleared per Rx packet
        .if $defined(PRU0)
        LDI     RCV_TEMP_REG_2.w0, INTR_PAC_STATUS_OFFSET_PRU0
        .else
        LDI     RCV_TEMP_REG_2.w0, INTR_PAC_STATUS_OFFSET_PRU1
        .endif ; PRU0
        LBCO    &RCV_TEMP_REG_2.b0, ICSS_SHARED_CONST, RCV_TEMP_REG_2.w0, 1
        QBEQ    FIRE_INTERRUPT, RCV_TEMP_REG_2.b0, INTR_PAC_DIS_ADP_LGC_DIS
        QBEQ    SET_INTR_TO_HOST_PENDING_RX, RCV_TEMP_REG_2.b0, INTR_PAC_ENA_ADP_LGC_DIS
        QBBS    FIRE_INTERRUPT, R22, HOST_QUEUE_EMPTY_STATUS    ; fire interrupt if host queue bit is SET i.e. host queue is empty
SET_INTR_TO_HOST_PENDING_RX:
        SET     R22, R22, INTR_TO_HOST_PENDING_RX
        QBA     LB_DONE

FIRE_INTERRUPT:
        ; two interrupts for two ports in MAC mode
        .if $defined(PRU0)
        LDI     R31, 0x24
        .else
        LDI     R31, 0x25
        .endif ; PRU0

        .if $defined(EDIO_INTR_PAC)
        ; EDIO output J7 pin 4 AM572x
        LDI32 RCV_TEMP_REG_1, 0x4B22E310
        LDI RCV_TEMP_REG_2, 0x0040
        SBBO &RCV_TEMP_REG_2.b0, RCV_TEMP_REG_1, 0, 1
        SBBO &RCV_TEMP_REG_2.b1, RCV_TEMP_REG_1, 0, 1
        .endif  ; EDIO_INTR_PAC
LB_DONE:
        .endif ;TWO_PORT_CFG
	.endm

	.endif	;__mii_rcv_hp

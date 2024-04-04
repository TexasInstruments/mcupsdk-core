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
; file:   emac_MII_Rcv.asm
;
; brief:  Receive Task
;
;
;  (C) Copyright 2017-2019, Texas Instruments, Inc
;
;

    .if !$defined("__mii_rcv_p")
__mii_rcv_p	.set	1

;;///////////////////////////////////////////////////////
; Includes Section
;;///////////////////////////////////////////////////////
    .include "emac_MII_Rcv.h"
    .include "icss_macros.h"
;	.include "icss_regs.h"
    .if $defined("ICSS_SWITCH_BUILD")
    .include "icss_switch_macros.h"

    .if $defined("ICSS_STP_SWITCH")
    .include "icss_stp_switch_macros.h"
    .cdecls C,NOLIST
%{
#include "icss_stp_switch.h"
%}
    .endif ; ICSS_STP_SWITCH

    .endif
    .include "micro_scheduler.h"
    .include "emac_MII_Xmt.h"
    .include "icss_miirt_regs.h"

    .if $defined(PTP)
    .include "icss_ptp.h"
    .include "icss_ptp_macro.h"
    .cdecls C,NOLIST
%{
#include "icss_timeSync_memory_map.h"
%}
    .endif ;PTP

        .global  FN_RCV_FB
        .global  FN_RCV_NB
        .global  FN_RCV_LB
        .global  XMT_QUEUE
        .global  TASK_EXECUTION_FINISHED
        .global  FN_TIMESTAMP_GPTP_PACKET
        .global  FN_CHECK_AND_CLR_PTP_FWD_FLAG
        .global  FN_CHECK_AND_CLR_PTP_FWD_FLAG_L2
        .global  FN_COMPARE_DELAY_RESP_ID


;****************************************************************************
;
;     NAME		: FN_RCV_FB
;     DESCRIPTION	: receives the first block of data from new frame (out of RX L2)
;     RETURNS		:
;     ARGS		:
;     USES 		:
;     INVOKES 		:
;
;****************************************************************************
FN_RCV_FB:
    ; Data is already present in the registers. So, no need to read it again.

    ZERO	&MII_RCV, $sizeof(MII_RCV)	; init MII_RCV parameter
    ; OPT: Possible to make them both context next to each other
    .if    $defined("TWO_PORT_CFG")
    ZERO    &MII_RCV_PORT, $sizeof(MII_RCV_PORT)    ; init MII_RCV parameter  (R14....R17)
    .endif
    ; Check if RX port is enabled (information provided by host)
    LDI	R10.w0 , PORT_CONTROL_ADDR
    LBCO	&R10, PRU_DMEM_ADDR, R10.w0, 10	; 4 + 6 bytes of Interface MAC Addr

    ; OPT: Port can be disabled directly in the MII Hardware
    ; Is port enabled?
    QBBC	EXIT_FB, R10, 0

    .if $defined("ICSS_STP_SWITCH")
CHECK_STP_DISABLE:
    ; Prepare STP State register address
    .if $defined("PRU0")
    ; Read Port 1 STP state
    LDI32 RCV_TEMP_REG_3, ICSS_EMAC_FW_FDB__STP_P1_STP_STATE_ADDR
    .else
    ; Read Port 2 STP state
    LDI32 RCV_TEMP_REG_3, ICSS_EMAC_FW_FDB__STP_P2_STP_STATE_ADDR
    .endif ; $defined("PRU0")
    LBBO  &RCV_TEMP_REG_1.b0, RCV_TEMP_REG_3, 0, 1 ; read state from memory
    AND   STP_STATE__R22_BYTE, STP_STATE__R22_BYTE, STP_STATE__R22_INV_MASK ; clear relevant bits in R22
    OR    STP_STATE__R22_BYTE, STP_STATE__R22_BYTE, RCV_TEMP_REG_1.b0 ; set the relevant bits in R22

    ; If Port disabled, disable directly in MII Hardware
    AND   RCV_TEMP_REG_1.b0, STP_STATE__R22_BYTE, STP_STATE__R22_MASK ; read STP state in R22
    QBEQ  EXIT_FB, RCV_TEMP_REG_1.b0, STP_STATE_DISABLED

    ; If STP state is invalid, log error statistics and drop packet
    QBGE  CONTINUE_PROCESSING, RCV_TEMP_REG_1.b0, STP_STATE_BLOCKING
    LDI   RCV_TEMP_REG_3 , STP_INVALID_STATE_OFFSET       ; else drop frame and increase the stats
    QBA   COUNT_RX_STATS
    .endif ; ICSS_STP_SWITCH

CONTINUE_PROCESSING:
    .if $defined("ICSS_STP_SWITCH")
    .if $defined("ICSS_SWITCH_BUILD")
    ; We are in a valid STP state, thus there is a chance packets get to the host
    ; By default, both of these flags should be cleared
    CLR   R22, R22, FDB_LOOKUP_SUCCESS__R22_BIT
    CLR   R22, R22, PKT_FLOODED__R22_BIT
    .endif
    .endif ; ICSS_STP_SWITCH

    ; set Rcv_active flag to indicate an ongoing reception
    SET	R23 , R23 , Rcv_active

;*********************************Check for Errors*****************************************
;******************************************************************************************
    .if $defined("ICSS_DUAL_EMAC_BUILD")
;check for SFD Errors
    .if $defined("HALF_DUPLEX_ENABLED")
; below code not needed for Full Duple
    .if $defined("PRU0")
    LBCO	&RCV_TEMP_REG_3.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR0, 1      ;read the PRUSS_MII_RT_RX_ERR0 register
    QBBC	CHECK_FOR_SHORT_SFD, RCV_TEMP_REG_3, 1      ; check for any error if not then jump else continue
    ;clear error and writeback
    SET	RCV_TEMP_REG_3 , RCV_TEMP_REG_3 , 1
    SBCO	&RCV_TEMP_REG_3.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR0, 1
    QBA     COUNT_SFD_ERROR
CHECK_FOR_SHORT_SFD:
    QBBC	CONTINUE_PROCESSING_PKT, RCV_TEMP_REG_3, 0      ; check for any error if not then jump else continue
    ;clear error and writeback
    SET	RCV_TEMP_REG_3 , RCV_TEMP_REG_3 , 0
    SBCO	&RCV_TEMP_REG_3.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR0, 1
    QBA     COUNT_SFD_ERROR
    .else
    LBCO	&RCV_TEMP_REG_3.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR1, 1	;read the PRUSS_MII_RT_RX_ERR0 register
    QBBC	CHECK_FOR_SHORT_SFD, RCV_TEMP_REG_3, 1      ; check for any error if not then jump else continue
    ;clear error and writeback
    SET	RCV_TEMP_REG_3 , RCV_TEMP_REG_3 , 1
    SBCO	&RCV_TEMP_REG_3.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR1, 1
    QBA     COUNT_SFD_ERROR
CHECK_FOR_SHORT_SFD:
    QBBC	CONTINUE_PROCESSING_PKT, RCV_TEMP_REG_3, 0      ; check for any error if not then jump else continue
    ;clear error and writeback
    SET	RCV_TEMP_REG_3 , RCV_TEMP_REG_3 , 0
    SBCO	&RCV_TEMP_REG_3.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR1, 1
    QBA     COUNT_SFD_ERROR
    .endif

    ;check R31 bit 19 for error
    QBBC	CONTINUE_PROCESSING_PKT, R31, 19
    ;clear the error
    M_CMD16 D_RX_ERROR_CLEAR

    ;set error flag
    LDI	RCV_TEMP_REG_3 , RX_ERROR_OFFSET
    QBA     COUNT_RX_STATS
    ; Check if SA matches with Slave's Interface MAC addr
    .endif  ;HALF_DUPLEX_ENABLED
    .endif  ;ICSS_DUAL_EMAC_BUILD
CONTINUE_PROCESSING_PKT:

    .if $defined("ICSS_STP_SWITCH")
CHECK_STP_DISCARDING:
    ; Listening and Blocking states all resolve to the same action:
    ; .. only allow BPDU frames to pass. Aka DISCARDING in RSTP.

    ; STP State for current port is stored in R22
    AND   RCV_TEMP_REG_1.b0, STP_STATE__R22_BYTE, STP_STATE__R22_MASK ; read STP state in R22
    QBEQ  STP_DISCARD, RCV_TEMP_REG_1.b0, STP_STATE_BLOCKING
    QBEQ  STP_DISCARD, RCV_TEMP_REG_1.b0, STP_STATE_LISTENING

    ; Not discarding, continue processing
    QBA CHECK_RSVD_ADDR

STP_DISCARD:
    ; Only allow BPDU packets
CHECK_MC_BPDU:
    ; Check Multicast BPDU Address (allow all 01:80:c2:xx:xx:xx traffic)
    LDI   RCV_TEMP_REG_3.w0, 0x8001
    QBNE  CHECK_VLAN_BPDU, EthWord.DstWord_01, RCV_TEMP_REG_3.w0
    LDI   RCV_TEMP_REG_3.b0, 0xC2
    MOV   RCV_TEMP_REG_3.b2, EthByte.DstByte_2
    QBNE  CHECK_VLAN_BPDU, RCV_TEMP_REG_3.b0, RCV_TEMP_REG_3.b2

    ; Address is Multicast BPDU
    QBA   CHECK_RSVD_ADDR

CHECK_VLAN_BPDU:
    ; Check VLAN BPDU Address
    LDI   RCV_TEMP_REG_3.w0, 0x0001
    LDI   RCV_TEMP_REG_3.w2, 0xCC0C
    QBNE  DROP_PKT, Ethernet.DstAddr_0123, RCV_TEMP_REG_3
    LDI   RCV_TEMP_REG_3.w0, 0xCDCC
    QBNE  DROP_PKT, Ethernet.DstAddr_45, RCV_TEMP_REG_3.w0

CHECK_RSVD_ADDR:
    ; Reserved addresses are 01:80:c2:00:00:00 - 01:80:c2:00:00:10
    LDI   RCV_TEMP_REG_3.w0, 0x8001
    LDI   RCV_TEMP_REG_3.w2, 0x00C2
    QBNE  CHECK_PKT_SOURCE, Ethernet.DstAddr_0123, RCV_TEMP_REG_3
    MOV   RCV_TEMP_REG_3.w2, Ethernet.DstAddr_45
    QBNE  CHECK_PKT_SOURCE, RCV_TEMP_REG_3.b2, 0x00
    QBLT  CHECK_PKT_SOURCE, RCV_TEMP_REG_3.b3, 0x10

    ; Packets for reserved addresses are not to be forwarded
    ; .. so set the local STP state to "Learning"
    AND   STP_STATE__R22_BYTE, STP_STATE__R22_BYTE, STP_STATE__R22_INV_MASK ; clr STP state in R22
    OR    STP_STATE__R22_BYTE, STP_STATE__R22_BYTE, STP_STATE_LEARNING

    ; Address is VLAN BPDU, continue
    .endif ; ICSS_STP_SWITCH

CHECK_PKT_SOURCE:
    QBNE	FB_SA_NO_MATCH_INTERFACE_MAC, Ethernet.SrcAddr_01, R11.w0	;Check if Source Address matches with own address
    QBNE	FB_SA_NO_MATCH_INTERFACE_MAC, Ethernet.SrcAddr_23, R11.w2	;Source Address is stored into 3 words (16 bits)
    QBNE	FB_SA_NO_MATCH_INTERFACE_MAC, Ethernet.SrcAddr_45, R12.w0

    .if $defined("ICSS_STP_SWITCH")
DROP_PKT:
    .endif ; ICSS_STP_SWITCH
    LDI	RCV_TEMP_REG_3 , RX_DROPPED_FRAMES_OFFSET       ; else drop frame and increase the stats
    QBA     COUNT_RX_STATS
    ;QBA     EXIT_FB
    .if $defined("HALF_DUPLEX_ENABLED")
COUNT_SFD_ERROR:
    LDI	RCV_TEMP_REG_3 , SFD_ERROR_OFFSET
    .endif	;HALF_DUPLEX_ENABLED

COUNT_RX_STATS:
;count statistics based on offset set in RCV_TEMP_REG_3
    LBCO	&RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    ADD	RCV_TEMP_REG_2, RCV_TEMP_REG_2, 1
    SBCO	&RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
EXIT_FB:
    SET	MII_RCV.rx_flags , MII_RCV.rx_flags , rx_frame_error_shift
    QBA	FB_DONE

FB_SA_NO_MATCH_INTERFACE_MAC:
    .if $defined(PTP)
    M_GPTP_CHECK_AND_SET_FLAGS
    .endif ;PTP
    ; optimize the program memory by loading the promiscuous mode offset
    ; before jumping to multicast or broadcast packet check
    ; this would save loading the same when needed to check to bypass
    ; the storm prevention later when promiscuous mode is enabled
    .if $defined("ICSS_DUAL_EMAC_BUILD")
        LDI	RCV_TEMP_REG_3.w0 , ICSS_EMAC_FIRMWARE_PROMISCUOUS_MODE_OFFSET
        LBCO	&RCV_TEMP_REG_2, ICSS_SHARED_CONST, RCV_TEMP_REG_3.w0, 4
    .endif
    ; check for multi-cast
    .if $defined("ICSS_STP_SWITCH")
    QBBS	SKIP_SELF_CHECK, EthByte.DstByte_0, 0 ; if packet is destined for multi/broadcast, skip the self-reference check and DST FDB lookup
    .else
    QBBS	FB_MULTI_OR_BROADCAST, EthByte.DstByte_0, 0         ; check if packet is Multi/Broad cast type
    .endif ; ICSS_STP_SWITCH

;*********************************
; uni-cast handling
;*********************************
FB_UNICAST:

FB_UNICAST_SA_NO_MATCH:
    .if $defined("ICSS_DUAL_EMAC_BUILD")
        QBBS	FB_PROMISCUOUS_MODE_ENABLE, RCV_TEMP_REG_2, ICSS_EMAC_PROMISCOUS_BIT
    .endif
    ; check if DA matches with our Interface MAC address
    QBNE	FB_UNICAST_DA_NO_MATCH, Ethernet.DstAddr_0123, R11	;Check if Destination Address matches with own address
    QBNE	FB_UNICAST_DA_NO_MATCH, Ethernet.DstAddr_45, R12.w0

FB_PROMISCUOUS_MODE_ENABLE:
    ; the frame DST address matches with our own address
    SET	MII_RCV.rx_flags , MII_RCV.rx_flags , host_rcv_flag_shift ; set flag that host queue will receive data
    .if    $defined("TWO_PORT_CFG")
    .if    !$defined("ICSS_STP_SWITCH") ; STP needs to do an FDB lookup
    QBA		FB_LT_VT
    .endif
    .else
    QBA		FB_UNICAST_CHECK_CT
    .endif

FB_UNICAST_DA_NO_MATCH:
    .if $defined("ICSS_STP_SWITCH")
    ;; Linux needs to also check if the packet is destined for the other port MAC
    LDI    RCV_TEMP_REG_1.w0, PORT_MAC_ADDR ; Read from other port DMEM
    LBCO   &RCV_TEMP_REG_1, PRU_CROSS_DMEM, RCV_TEMP_REG_1.w0, 6 ; 6 bytes of MAC

    ; check if DA matches with other port MAC address
    QBNE   FB_UNICAST_DA_NO_MATCH_OTHER_PORT, Ethernet.DstAddr_0123, RCV_TEMP_REG_1
    QBNE   FB_UNICAST_DA_NO_MATCH_OTHER_PORT, Ethernet.DstAddr_45, RCV_TEMP_REG_2.w0

    ; the frame DST address matches with other port addr
    SET	MII_RCV.rx_flags , MII_RCV.rx_flags , host_rcv_flag_shift ; set flag that host queue will receive data

FB_UNICAST_DA_NO_MATCH_OTHER_PORT:
    .endif

    .if    $defined("TWO_PORT_CFG")

    .if $defined("ICSS_STP_SWITCH")
SKIP_SELF_CHECK:
FDB_LOCK:
    ; Take the lock and check for timeout
    LDI RCV_TEMP_REG_1.w0, 0x8F
    M_SPIN_LOCK RCV_TEMP_REG_4.b2, RCV_TEMP_REG_1.w0
    QBNE  FDB_CHECK_DST, RCV_TEMP_REG_4.b2, 0 ; continue if no timeout

    ; Flood/insert upon timeout
    SET   MII_RCV.rx_flags , MII_RCV.rx_flags , host_rcv_flag_shift
    SET   R22, R22, PKT_FLOODED__R22_BIT
    QBA   FDB_UNLOCK

FDB_CHECK_DST:
    ; If the packet is multi/broadcast, then skip DST lookup
    QBBS  FDB_SRC_LOOKUP, EthByte.DstByte_0, 0

    ; If the packet is not destined for the Host, continue with DST lookup
    QBBC  FDB_DST_LOOKUP, MII_RCV.rx_flags, host_rcv_flag_shift

    ; Set the local STP state to "Learning" as a way to ensure we can check the
    ; .. SRC address but not forward the packet. This will not affect the global
    ; .. STP state.
    AND   STP_STATE__R22_BYTE, STP_STATE__R22_BYTE, STP_STATE__R22_INV_MASK ; clr STP state in R22
    OR    STP_STATE__R22_BYTE, STP_STATE__R22_BYTE, STP_STATE_LEARNING

    QBA   FDB_SRC_LOOKUP ; skip DST lookup

FDB_DST_LOOKUP:
    ; Look up the destination in the FDB to determine if it needs to be flooded or directly forwarded
    M_UNICAST_FDB_HASH_LOOKUP     EthWord.DstWord_01, EthWord.DstWord_23, Ethernet.DstAddr_45, RCV_TEMP_REG_1.b2

    ; Check lookup success
    QBEQ  FDB_DST_LOOKUP_SUCCESS, RCV_TEMP_REG_1.b2, 1

    ; Not found in FDB, flood the packet
FLOOD_UNICAST_PACKET:
    ; Unicast address not found in FDB, flood it by forwarding the packet
    ; to the host as well as the other port
    SET   MII_RCV.rx_flags , MII_RCV.rx_flags , host_rcv_flag_shift
    SET   R22, R22, PKT_FLOODED__R22_BIT
    QBA   FDB_DST_LOOKUP_END

FDB_DST_LOOKUP_SUCCESS:
    ; Check destination port is not current port
    LBBO  &RCV_TEMP_REG_1.b0, RCV_TEMP_REG_3, FDB_MAC_INFO__PORT_NO__OFFSET, FDB_MAC_INFO__PORT_NO__SIZE ; TEMP_REG_3 still has the table pointer from lookup

    ; If the destination port is the current port, set the local STP state
    ; .. to "Learning" so we don't forward the packet
    .if $defined("PRU0")
    QBNE  FDB_DST_LOOKUP_END, RCV_TEMP_REG_1.b0, 0x0 ; Port 1
    .else
    QBNE  FDB_DST_LOOKUP_END, RCV_TEMP_REG_1.b0, 0x1 ; Port 2
    .endif

    ; Set the local STP state to "Learning" as a way to ensure we can learn the
    ; .. SRC address but not forward the packet. This will not affect the global
    ; .. STP state.
    AND   STP_STATE__R22_BYTE, STP_STATE__R22_BYTE, STP_STATE__R22_INV_MASK ; clr STP state in R22
    OR    STP_STATE__R22_BYTE, STP_STATE__R22_BYTE, STP_STATE_LEARNING

FDB_DST_LOOKUP_END:
FDB_SRC_LOOKUP:
    ; If the src addr is multi/broadcast, then skip the SRC lookup
    QBBS  FDB_SRC_LOOKUP_END, EthByte.SrcByte_0, 0

    ; Now we must look up the source address in the FDB for learning purposes
    M_UNICAST_FDB_HASH_LOOKUP     Ethernet.SrcAddr_01, Ethernet.SrcAddr_23, Ethernet.SrcAddr_45, RCV_TEMP_REG_1.b3

    ; Check lookup success
    QBEQ  FDB_SRC_LOOKUP_FAIL, RCV_TEMP_REG_1.b3, 0

FDB_SRC_LOOKUP_SUCCESS:
    ; The source address was found in the FDB, alert the host and proceed to reset the ageing timer
    SET   R22, R22, FDB_LOOKUP_SUCCESS__R22_BIT

FDB_TOUCH_ENTRY:
    ; Reset FDB entry age and update port number information

    ; Reset ageing timer
    LDI   RCV_TEMP_REG_1.w0, 0x0000

    ; Set port information
    .if $defined("PRU0")
    LDI   RCV_TEMP_REG_1.b2, 0x0 ; Port 1
    .else
    LDI   RCV_TEMP_REG_1.b2, 0x1 ; Port 2
    .endif

    SBBO  &RCV_TEMP_REG_1, RCV_TEMP_REG_3, FDB_MAC_INFO__AGE__OFFSET, (FDB_MAC_INFO__AGE__SIZE + FDB_MAC_INFO__PORT_NO__SIZE) ; TEMP_REG_3 still has the table pointer from lookup
    QBA   FDB_SRC_LOOKUP_END

FDB_SRC_LOOKUP_FAIL:
    ; The source MAC isn't found in the FDB, send to the host for insertion
    SET   MII_RCV.rx_flags , MII_RCV.rx_flags , host_rcv_flag_shift

FDB_SRC_LOOKUP_END:
FDB_UNLOCK:
    ; Release the lock
    M_SPIN_UNLOCK

    ; If the packet is multi/broadcast, process it as such
    QBBS  FB_MULTI_OR_BROADCAST, EthByte.DstByte_0, 0
    .endif ; ICSS_STP_SWITCH
    ; Begin process of forwarding unicast packet, cut-through if possible
    QBA   FB_UNICAST_CHECK_CT

    .else
    LDI	RCV_TEMP_REG_3 , RX_DROPPED_FRAMES_OFFSET
    QBA     COUNT_RX_STATS
    .endif
;*********************************
; multicast/broadcast handling
;*********************************
FB_MULTI_OR_BROADCAST:
FB_CONTINUE:
    ; Check if destination is broadcast
    FILL	&RCV_TEMP_REG_1, 4   ; Fill with 0xffffffff
    QBNE	FB_MULTICAST, Ethernet.DstAddr_0123, RCV_TEMP_REG_1
    QBNE	FB_MULTICAST, Ethernet.DstAddr_45, RCV_TEMP_REG_1.w0

FB_BROADCAST:
    SET	R22 , R22 , RX_BC_FRAME      ; set broad-cast bit to indicate the broadcast frame
    SET	MII_RCV.rx_flags , MII_RCV.rx_flags , host_rcv_flag_shift
    QBA	FB_BROADCAST_CHECK_CT

FB_MULTICAST:
    SET	MII_RCV.rx_flags , MII_RCV.rx_flags , host_rcv_flag_shift
    SET	R22 , R22 , RX_MC_FRAME      ; set multi-cast bit to indicate the multicast frame

    .if $defined("ICSS_DUAL_EMAC_BUILD") | $defined("ICSS_STP_SWITCH")
        ; check if multicast filtering is enabled or not i.e. check MULTICAST_CONTROL_BIT in PRU1 data memory
        LDI     RCV_TEMP_REG_3.w0, ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_OFFSET
        LBCO    &RCV_TEMP_REG_3.b0, PRU_DMEM_ADDR, RCV_TEMP_REG_3.w0, 1
        QBEQ    FB_MULTICAST_CONTINUE, RCV_TEMP_REG_3.b0, ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_DISABLED    ; one byte field : 0 -> multicast filtering disabled | 1 -> multicast filtering enabled

        ; search in multicast table to determine if host receive is to be enabled or not for the incoming multicast frame
        M_MULTICAST_TABLE_SEARCH_OP
        ; QBA     FB_MULTICAST_CONTINUE
    .endif

FB_MULTICAST_CONTINUE:
FB_UNICAST_CHECK_CT:
FB_BROADCAST_CHECK_CT:
    .if $defined("ICSS_DUAL_EMAC_BUILD") | $defined("ICSS_STP_SWITCH")
        ; check if VLAN filtering is enabled or not i.e. check VLAN_FLTR_CTRL_SHIFT in VLAN_FLTR_CTRL_BYTE stored in data memory
        LDI     RCV_TEMP_REG_3.w0, ICSS_EMAC_FW_VLAN_FILTER_CTRL_BITMAP_OFFSET
        LBCO    &RCV_TEMP_REG_3.b0, PRU_DMEM_ADDR, RCV_TEMP_REG_3.w0, 1
    ;-----------------------------------------------------------
    ;   VLAN_FLTR_CTRL_BYTE  |          |          |           | R13 | RCV_TEMP_REG_3
    ;-----------------------------------------------------------
        QBBC    FB_SKIP_VLAN_FLTR, RCV_TEMP_REG_3.b0, ICSS_EMAC_FW_VLAN_FILTER_CTRL_ENABLE_BIT   ; one bit field | 0 : VLAN filter disabled | 1 : VLAN filter enabled

        ; search in VLAN table to determine if host receive is to be enabled or not for the incoming frame
        M_VLAN_FLTR_SRCH_OP

FB_SKIP_VLAN_FLTR:
    .endif

    ;PINDSW-4577: Fix to avoid cut-through packets from going via Storm Prevention
    ;Storm prevention is done only if host receive flag is set
    LDI     RCV_TEMP_REG_3.w2, DISABLE_STORM_PREV_FOR_HOST
    LBCO    &RCV_TEMP_REG_3.b0, PRU_DMEM_ADDR, RCV_TEMP_REG_3.w2, 1   ; load the control value
    ;This check is for maintaining backward compatibility, driver needs to write non-zero value for enabling
    QBEQ    FB_SKIP_HOST_CHECK_STORM_PREV, RCV_TEMP_REG_3.b0, 0
    QBBC    FB_CONT_CT_CHECK, MII_RCV.rx_flags , host_rcv_flag_shift
FB_SKIP_HOST_CHECK_STORM_PREV:

    ; Storm Prevention
    QBBC    FB_STORM_NOT_MC, R22, RX_MC_FRAME
    LDI     RCV_TEMP_REG_3 , STORM_PREVENTION_OFFSET_MC
    ;Do storm prevention here
    LBCO  &RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4   ; load the current storm prevent count
    QBBC  FB_CONT_CT_CHECK, RCV_TEMP_REG_2, 0
    LSR   RCV_TEMP_REG_2, RCV_TEMP_REG_2, 8
    QBGT  FB_DISCARD_MC, RCV_TEMP_REG_2.w0, 1    ; check if the counter is less than zero and discard packet
    SUB   RCV_TEMP_REG_2, RCV_TEMP_REG_2, 1
    LSL   RCV_TEMP_REG_2, RCV_TEMP_REG_2, 8
    SET   RCV_TEMP_REG_2 , RCV_TEMP_REG_2 , 0
    SBCO  &RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4

    QBA     FB_CONT_CT_CHECK
FB_STORM_NOT_MC:
    QBBC    FB_STORM_NOT_BC, R22, RX_BC_FRAME
    LDI     RCV_TEMP_REG_3 , STORM_PREVENTION_OFFSET_BC
    ;Do storm prevention here
    LBCO  &RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4   ; load the current storm prevent count
    QBBC  FB_CONT_CT_CHECK, RCV_TEMP_REG_2, 0
    LSR   RCV_TEMP_REG_2, RCV_TEMP_REG_2, 8
    QBGT  FB_DISCARD_BC, RCV_TEMP_REG_2.w0, 1    ; check if the counter is less than zero and discard packet
    SUB   RCV_TEMP_REG_2, RCV_TEMP_REG_2, 1
    LSL   RCV_TEMP_REG_2, RCV_TEMP_REG_2, 8
    SET   RCV_TEMP_REG_2 , RCV_TEMP_REG_2 , 0
    SBCO  &RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4

    QBA     FB_CONT_CT_CHECK
FB_STORM_NOT_BC:
    LDI     RCV_TEMP_REG_3 , STORM_PREVENTION_OFFSET_UC
    ;Do storm prevention here
    LBCO  &RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4   ; load the current storm prevent count
    QBBC  FB_CONT_CT_CHECK, RCV_TEMP_REG_2, 0
    LSR   RCV_TEMP_REG_2, RCV_TEMP_REG_2, 8
    QBGT  FB_DISCARD_UC, RCV_TEMP_REG_2.w0, 1    ; check if the counter is less than zero and discard packet
    SUB   RCV_TEMP_REG_2, RCV_TEMP_REG_2, 1
    LSL   RCV_TEMP_REG_2, RCV_TEMP_REG_2, 8
    SET   RCV_TEMP_REG_2 , RCV_TEMP_REG_2 , 0
    SBCO  &RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4

    QBA     FB_CONT_CT_CHECK

FB_DISCARD_MC:
    LDI RCV_TEMP_REG_3 , STORM_PREVENTION_COUNTER_MC
    QBA COUNT_RX_STATS
FB_DISCARD_BC:
    LDI RCV_TEMP_REG_3 , STORM_PREVENTION_COUNTER_BC
    QBA COUNT_RX_STATS
FB_DISCARD_UC:
    LDI RCV_TEMP_REG_3 , STORM_PREVENTION_COUNTER_UC
    QBA COUNT_RX_STATS

FB_CONT_CT_CHECK:
    .if    $defined("TWO_PORT_CFG")

    .if $defined("ICSS_STP_SWITCH")
CHECK_STP_LEARNING:
    ; Only forward packet if STP state is FORWARDING
    AND   RCV_TEMP_REG_1.b0, STP_STATE__R22_BYTE, STP_STATE__R22_MASK ; read STP state in R22
    QBNE  SKIP_FORWARDING, RCV_TEMP_REG_1.b0, STP_STATE_FORWARDING ; STP State for current port is stored in R22

    ; Only forward packet if the OTHER port state is FORWARDING
    .if $defined("PRU0")
    ; Read Port 2 STP state
    LDI32 RCV_TEMP_REG_3, ICSS_EMAC_FW_FDB__STP_P2_STP_STATE_ADDR
    .else
    ; Read Port 1 STP state
    LDI32 RCV_TEMP_REG_3, ICSS_EMAC_FW_FDB__STP_P1_STP_STATE_ADDR
    .endif ; $defined("PRU0")
    LBBO  &RCV_TEMP_REG_1.b0, RCV_TEMP_REG_3, 0, 1

    QBNE  SKIP_FORWARDING, RCV_TEMP_REG_1.b0, STP_STATE_FORWARDING

SET_FORWARDING:
    .endif ; ICSS_STP_SWITCH
    QBBS    FB_NO_CT, R23, 0 ;Xmt_active            ; check if we can set cut-through
    QBBC    FB_NO_CT, R22, 31 ;PACKET_TX_ALLOWED
    QBA        FB_CT_HANDLING

FB_NO_CT:
    SET        MII_RCV.rx_flags, MII_RCV.rx_flags, fwd_flag_shift    ;MII_RCV.rx_flags.fwd_flag
    JMP        FB_LT_VT
FB_CT_HANDLING:
  SET        MII_RCV.tx_flags, MII_RCV.tx_flags, cut_through_flag_shift    ;MII_RCV.tx_flags.cut_through_flag

    .if $defined("ICSS_STP_SWITCH")
SKIP_FORWARDING:
    .endif ; ICSS_STP_SWITCH
FB_LT_VT:

;check link on other port
    QBBS	FB_EGRESS_LINK_UP, R22, 10	 ;replaced: QBBS    FB_EGRESS_LINK_UP, OPPOSITE_PORT_LINK_UP

; OPT: See whether they can be moved next to each other
    CLR        MII_RCV.rx_flags, MII_RCV.rx_flags, fwd_flag_shift
    CLR        MII_RCV.tx_flags, MII_RCV.tx_flags, cut_through_flag_shift

    ;If host receive flag is not set then increment statistics for dropped frames
    QBBS        FB_EGRESS_LINK_UP, MII_RCV.rx_flags, host_rcv_flag_shift
    LDI         RCV_TEMP_REG_3, RX_DROPPED_FRAMES_OFFSET
    CLR         R22, R22, RX_BC_FRAME
    CLR         R22, R22, RX_MC_FRAME
    LBCO    &RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    ADD     RCV_TEMP_REG_2, RCV_TEMP_REG_2, 1
    SBCO    &RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4


FB_EGRESS_LINK_UP:

    QBBC    FB_LT_VT_2, MII_RCV.tx_flags, cut_through_flag_shift

    LDI        CUT_THROUGH_BYTE_CNT, 0x0000

    ; Insert first 12 bytes in the TX FIFO of opposit
    .if $defined("ICSS_REV1")
    .if !$defined("TX_L2_ENABLED")
    LDI	TX_DATA_WORD_MASK , 0xffff
    AND TX_DATA_BYTE , BUFFER.b0 , BUFFER.b0
    M_PUSH_BYTE
    AND TX_DATA_BYTE , BUFFER.b1 , BUFFER.b1
    M_PUSH_BYTE
    .else
	AND     RCV_TEMP_REG_2, R2, R2
	MOV		R2.b0, BUFFER.b0
    XOUT    TX_L2_BANK_ID, &R2, 1
    AND     R2, RCV_TEMP_REG_2, RCV_TEMP_REG_2
    AND     RCV_TEMP_REG_2, R2, R2
	MOV		R2.b0, BUFFER.b1
    XOUT    TX_L2_BANK_ID, &R2, 1
    AND     R2, RCV_TEMP_REG_2, RCV_TEMP_REG_2
    .endif ;TX_L2_ENABLED
    ;MOV		loop_cnt, 2
    LDI	TX_DATA_POINTER, buffer_ptr + 2
    loop	FB_RCV_EndLoop_1, 2
    .if !$defined("TX_L2_ENABLED")
    MVIW	TX_DATA_WORD, *TX_DATA_POINTER
    M_PUSH_WORD_CMD
    .else
    AND     RCV_TEMP_REG_2, R2, R2
	MVIW	R2.w0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 2
    AND     R2, RCV_TEMP_REG_2, RCV_TEMP_REG_2
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 2
FB_RCV_EndLoop_1:
    .endif
    .if $defined("ICSS_REV2")
    LDI        TX_DATA_POINTER, buffer_ptr
    loop    FB_RCV_EndLoop_1, 3
    .if !$defined("TX_L2_ENABLED")
    MVID    TX_DATA_DOUBLE_WORD, *TX_DATA_POINTER
    .else
    AND     RCV_TEMP_REG_2, R2, R2
	MVID	R2, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 4
    AND     R2, RCV_TEMP_REG_2, RCV_TEMP_REG_2
    .endif ;TX_L2_ENABLED
    ADD        TX_DATA_POINTER, TX_DATA_POINTER, 4
FB_RCV_EndLoop_1:
    .endif
    .endif ;TWO_PORT_CFG
FB_LT_VT_2: ; Quality of service --> select RX queue priority based on Ethernet frame tags

    XIN	RX_L2_BANK0_ID, &R18, RANGE_R18_b0	; XIN RX L2 bank 0 R18 value
    XIN	RX_L2_BANK0_ID, &R2, RANGE_R2_R13	; XIN RX L2 bank 0 R2 - R13 value

    .if $defined(PTP)
    M_GPTP_ASSIGN_QOS
    .endif ;PTP

    .if    $defined("TWO_PORT_CFG")
; The naming of the QoS queue goes from 1..4, but we need to count from 0..3
    LDI        MII_RCV.qos_queue, 4-1    ; default (lowest priority) queue is 4

FB_QOS_APPLY_RULES:
    LDI        RCV_TEMP_REG_1.w0, 0x0081        ; 0x8100 --> indicates QOS tagged frame
    QBNE    FB_QOS_DONE, Ethernet.TPID, RCV_TEMP_REG_1.w0
    LSR        RCV_TEMP_REG_1.b0, R5.b2, 5        ; extract PCP info. FIXME: clpru : Ethernet.TCI.b0 -> R5.b2
    QBGT    FB_QOS_CHECK_FOR_PRIO2, RCV_TEMP_REG_1.b0, 6    ; PCP == 7/6
    LDI        MII_RCV.qos_queue, 1-1    ; highest priority is 1, select queue 1

  QBA        FB_QOS_DONE
FB_QOS_CHECK_FOR_PRIO2:
    QBGT    FB_QOS_CHECK_FOR_PRIO3, RCV_TEMP_REG_1.b0, 4    ; PCP == 5/4
    LDI        MII_RCV.qos_queue, 2-1    ; priority is 2, select queue 2
    QBA        FB_QOS_DONE
FB_QOS_CHECK_FOR_PRIO3:
    QBGT    FB_QOS_DONE, RCV_TEMP_REG_1.b0, 2    ; PCP == 3/2
    LDI        MII_RCV.qos_queue, 3-1    ; priority is 3, select queue 3
  QBA        FB_QOS_DONE
; if PCP == 1/0, this priority 4, default queue already selected
FB_QOS_LOADED:
FB_QOS_DONE:
FB_DONE:
    .else
    .if $defined("PRU0")
; The naming of the QoS queue goes from 1..4, but we need to count from 0..3
    LDI	MII_RCV.qos_queue, 2-1	; default (lowest priority) queue is 2 for PRU0
    .else
    LDI	MII_RCV.qos_queue, 4-1	; default (lowest priority) queue is 4	for PRU1
    .endif

FB_QOS_APPLY_RULES:
    LDI	RCV_TEMP_REG_1.w0, 0x0081	; 0x8100 --> indicates QOS tagged frame
    QBNE	FB_QOS_DONE, Ethernet.TPID, RCV_TEMP_REG_1.w0
        LSR	RCV_TEMP_REG_1.b0, Ethernet.TCI.x_b.b0, 5	; extract PCP info

    .if $defined("PRU0")
    QBGT	FB_QOS_DONE, RCV_TEMP_REG_1.b0, 4	; PCP == 7/6
    LDI	MII_RCV.qos_queue, 1-1	; highest priority is 1, select queue 1
    .else
    QBGT	FB_QOS_DONE, RCV_TEMP_REG_1.b0, 4	; PCP == 7/6
    LDI	MII_RCV.qos_queue, 3-1	; highest priority is 3, select queue 3
    .endif
    QBA		FB_QOS_DONE

; if PCP == 1/0, this priority 4, default queue already selected
FB_QOS_LOADED:
FB_QOS_DONE:
FB_DONE:
    .endif ;TWO_PORT_CFG

    .if    $defined("TWO_PORT_CFG")
    ;Check if cut-through bit is set ..if yes then push 10 more bytes in Tx_fifo and remain in Rx_Task itself
    ; OPT : Below bytes can be pushed early in the Profinet
    QBBC    FB_DONE_NORMAL, MII_RCV.tx_flags, cut_through_flag_shift    ; is cut-through flag set?
  ; We enter the task with 14 bytes but here we always have 16 bytes even for non HSR frames
    .if $defined("ICSS_REV1")
    loop	FB_RCV_EndLoop_2, 5
    .if !$defined("TX_L2_ENABLED")
    MVIW	TX_DATA_WORD, *TX_DATA_POINTER
    M_PUSH_WORD_CMD
    .else
    AND     RCV_TEMP_REG_2, R2, R2
	MVIW	R2.w0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 2
    AND     R2, RCV_TEMP_REG_2, RCV_TEMP_REG_2
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 2
FB_RCV_EndLoop_2:
    .endif
    .if $defined("ICSS_REV2")
    .if !$defined("TX_L2_ENABLED")
    MVID    TX_DATA_DOUBLE_WORD, *TX_DATA_POINTER
    .else
    AND     RCV_TEMP_REG_2, R2, R2
	MVID	R2, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 4
    AND     R2, RCV_TEMP_REG_2, RCV_TEMP_REG_2
    .endif ;TX_L2_ENABLED
    ADD        TX_DATA_POINTER, TX_DATA_POINTER, 4
    .endif

    LDI        PREVIOUS_R18_RCV_BYTECOUNT, 16
    ADD        CUT_THROUGH_BYTE_CNT, CUT_THROUGH_BYTE_CNT, 16

FB_DONE_NORMAL:
    .endif
; Fill the rcv context with host/port queue base pointer, rd_ptr and wrk_pointer
    QBNE	FB_CHECK_QUEUE_2, MII_RCV.qos_queue,  0
    .if    $defined("TWO_PORT_CFG")
    LDI        RX_CONTEXT_OFFSET, P0_Q1_RX_CONTEXT_OFFSET

        .if    $defined("PRU0")
        LDI        RX_PORT_CONTEXT_OFFSET, P2_Q1_RX_CONTEXT_OFFSET
        .else
        LDI        RX_PORT_CONTEXT_OFFSET, P1_Q1_RX_CONTEXT_OFFSET
        .endif
    .else
    LDI	RX_CONTEXT_OFFSET , HOST_Q1_RX_CONTEXT_OFFSET       ; Select queue based on QOS
    .endif
    QBA	Initialize_rcv_context

    ;end_buffer_desc_offset
FB_CHECK_QUEUE_2:
    QBNE	FB_CHECK_QUEUE_3, MII_RCV.qos_queue,  1
    .if    $defined("TWO_PORT_CFG")
    LDI        RX_CONTEXT_OFFSET, P0_Q2_RX_CONTEXT_OFFSET
        .if    $defined("PRU0")
        LDI        RX_PORT_CONTEXT_OFFSET, P2_Q2_RX_CONTEXT_OFFSET
        .else
        LDI        RX_PORT_CONTEXT_OFFSET, P1_Q2_RX_CONTEXT_OFFSET
        .endif

    .else
    LDI	RX_CONTEXT_OFFSET , HOST_Q2_RX_CONTEXT_OFFSET       ; Select queue based on QOS
    .endif
    QBA	Initialize_rcv_context

FB_CHECK_QUEUE_3:
    QBNE	FB_CHECK_QUEUE_4, MII_RCV.qos_queue,  2
    .if    $defined("TWO_PORT_CFG")

    LDI        RX_CONTEXT_OFFSET, P0_Q3_RX_CONTEXT_OFFSET

        .if    $defined("PRU0")
        LDI        RX_PORT_CONTEXT_OFFSET, P2_Q3_RX_CONTEXT_OFFSET
        .else
        LDI        RX_PORT_CONTEXT_OFFSET, P1_Q3_RX_CONTEXT_OFFSET
        .endif

    .else
    LDI	RX_CONTEXT_OFFSET , HOST_Q3_RX_CONTEXT_OFFSET       ; Select queue based on QOS
    .endif
    QBA	Initialize_rcv_context

FB_CHECK_QUEUE_4:
    .if    $defined("TWO_PORT_CFG")

    LDI        RX_CONTEXT_OFFSET, P0_Q4_RX_CONTEXT_OFFSET
        .if    $defined("PRU0")
        LDI        RX_PORT_CONTEXT_OFFSET, P2_Q4_RX_CONTEXT_OFFSET
        .else
        LDI        RX_PORT_CONTEXT_OFFSET, P1_Q4_RX_CONTEXT_OFFSET
        .endif
    .else
    LDI	RX_CONTEXT_OFFSET , HOST_Q4_RX_CONTEXT_OFFSET       ; Select queue based on QOS
    .endif

Initialize_rcv_context:
    QBBC	FB_FILL_RCV_CONTEXT_PORT_QUEUE, MII_RCV.rx_flags, host_rcv_flag_shift
    ; Read the RX Context of Host Port of 8 bytes
    .if    $defined("TWO_PORT_CFG")
    LBCO     &MII_RCV.base_buffer_index, PRU1_DMEM_CONST, RX_CONTEXT_OFFSET, 8
    .else
    LBCO	&MII_RCV.base_buffer_index, ICSS_SHARED_CONST, RX_CONTEXT_OFFSET, 8
    .endif

FB_FILL_RCV_CONTEXT_PORT_QUEUE:
    .if    $defined("TWO_PORT_CFG")

    QBBC    FB_PKT_NOT_FORWARDED, MII_RCV.rx_flags, fwd_flag_shift
    ; Read the RX Context of Port of 8 bytes
    LBCO     &MII_RCV_PORT.base_buffer_index, PRU1_DMEM_CONST, RX_PORT_CONTEXT_OFFSET, 8
FB_PKT_NOT_FORWARDED:
    .endif ;TWO_PORT_CFG
    ; OPT: If R0.b0 is not changed then remove below instruction. it is to make sure at R0.b0 is 0
    LDI	R0.b0, SHIFT_NONE
    .if $defined("PRU0")
    XOUT	BANK1, &MII_RCV, $sizeof(MII_RCV)	; store task parameters in parameter bank
    .else
    XOUT	BANK2, &MII_RCV, $sizeof(MII_RCV)	; store task parameters in parameter bank
    .endif
    .if    $defined("TWO_PORT_CFG")
    .if    $defined("PRU0")
    LDI        R0.b0, SHIFT_R14_TO_R0
    XOUT    BANK0, &MII_RCV_PORT, $sizeof(MII_RCV_PORT)        ; store task parameters in parameter bank
    .else
    LDI        R0.b0, SHIFT_R14_TO_R4
    XOUT    BANK0, &MII_RCV_PORT, $sizeof(MII_RCV_PORT)        ; store task parameters in parameter bank
    .endif


    QBBS    FN_RCV_NB, MII_RCV.tx_flags, cut_through_flag_shift
    .endif ;TWO_PORT_CFG

    .if $defined("ICSS_DUAL_EMAC_BUILD")
    ;The following check is required in case of firmware lag
    ;when the firmware runs late compared to RX L2 FIFO.
    ;This was being experienced in Profinet Switch and the same is possible here.
    ;Haven't been able to reproduce this in EMAC.
    XIN	RX_L2_BANK0_ID, &R18, RANGE_R18_b0
    QBGT	FB_RCV_EXIT, R18.b0, 32
    LDI	CURRENT_TASK_POINTER, TX_TASK_POINTER
    JMP	FN_RCV_NB
    .endif

FB_RCV_EXIT:
    JMP	CALL_REG


;****************************************************************************
;
;     NAME		: FN_RCV_NB
;     DESCRIPTION	: receives the next 32Byte block of RX L2
;     RETURNS		:
;     ARGS		:
;     USES 		:
;     INVOKES 		:
;
;****************************************************************************
FN_RCV_NB:

    ; for XIN, make sure that shift is set to 0
    LDI	R0.b0, SHIFT_NONE
    ; restore task parameters in parameter bank
    .if $defined("PRU0")
    XIN	BANK1, &MII_RCV, $sizeof(MII_RCV)
    .else
    XIN	BANK2, &MII_RCV, $sizeof(MII_RCV)
    .endif
    .if    $defined("TWO_PORT_CFG")
    ;parameters for Port Receive
    ; OPT: Move it after the XIN of data
    .if    $defined("PRU0")
    LDI        R0.b0, SHIFT_R14_TO_R0
    XIN        BANK0, &MII_RCV_PORT, $sizeof(MII_RCV_PORT)
    .else
    LDI        R0.b0, SHIFT_R14_TO_R4
    XIN        BANK0, &MII_RCV_PORT, $sizeof(MII_RCV_PORT)
    .endif
    .endif
    ; this task is getting called by MS, first check is to see if we are currently receiving
    QBBC	NB_DONE_NO_PARAM_STORE, R23, Rcv_active
    QBBS	NB_DONE_NO_PARAM_STORE, MII_RCV.rx_flags, rx_frame_error_shift

FN_RCV_NB_CT_NO_RxEOF:          ; If Rx EOF has not come yet then directly return here ..saves 6 cycles and that helps in finding faster
                                ;     that L2 fifo has received 32 or 64 bytes ..which helps in calling NB_PROCESS_32BYTES early
    ; depending on the bank index, we load bank 0 or bank 1
    QBBS	NB_XIN_UPPER_BANK, MII_RCV.rx_flags, rx_bank_index_shift

NB_XIN_LOWER_BANK:
    ; OPT: Just one XIN is needed
    XIN	RX_L2_BANK0_ID, &R18, RANGE_R18_b0
    XIN	RX_L2_BANK0_ID, &R2, RANGE_R2_R13	; XIN RX L2 bank 0
    QBA	NB_CHECK_AMOUNT_OF_BYTES
NB_XIN_UPPER_BANK:
    ; OPT: Just one XIN is needed
    XIN	RX_L2_BANK1_ID, &R18, RANGE_R18_b0
    XIN	RX_L2_BANK1_ID, &R2, RANGE_R2_R13	; XIN RX L2 bank 1

NB_CHECK_AMOUNT_OF_BYTES:
    .if $defined("ICSS_SWITCH_BUILD")
    QBBC    check_for_32bytes, MII_RCV.tx_flags, cut_through_flag_shift    ;MII_RCV.tx_flags.cut_through_flag
    SUB        RCV_TEMP_REG_1, R18_RCV_BYTECOUNT, PREVIOUS_R18_RCV_BYTECOUNT
    .if $defined("ICSS_REV1")
    QBGT	NB_DONE, RCV_TEMP_REG_1, 2

NB_PUSH_NEXT_WORD_TO_FIFO:
    QBEQ	check_for_32bytes, R1.b3, 0x28
    ADD	PREVIOUS_R18_RCV_BYTECOUNT, PREVIOUS_R18_RCV_BYTECOUNT, 2
    .if !$defined("TX_L2_ENABLED")
    MVIW	TX_DATA_WORD, *R1.b3
    M_PUSH_WORD_CMD
    .else
	AND     RCV_TEMP_REG_2, R2, R2
	MVIW	R2.w0, *R1.b3
    XOUT    TX_L2_BANK_ID, &R2, 2
    AND     R2, RCV_TEMP_REG_2, RCV_TEMP_REG_2
    .endif ;TX_L2_ENABLED
    ADD	CUT_THROUGH_BYTE_CNT, CUT_THROUGH_BYTE_CNT, 2
    ADD	R1.b3, R1.b3, 2
    SUB	RCV_TEMP_REG_1, RCV_TEMP_REG_1, 2
    QBLE	NB_PUSH_NEXT_WORD_TO_FIFO, RCV_TEMP_REG_1, 2
    .endif
    .if $defined("ICSS_REV2")
    QBGT    NB_DONE, RCV_TEMP_REG_1, 4

NB_PUSH_NEXT_DOUBLE_WORD_TO_FIFO:
    QBEQ    check_for_32bytes, R1.b3, 0x28
    ADD        PREVIOUS_R18_RCV_BYTECOUNT, PREVIOUS_R18_RCV_BYTECOUNT, 4
    .if !$defined("TX_L2_ENABLED")
    MVID    TX_DATA_DOUBLE_WORD, *R1.b3
    .else
	AND     RCV_TEMP_REG_2, R2, R2
	MVID	R2, *R1.b3
    XOUT    TX_L2_BANK_ID, &R2, 4
    AND     R2, RCV_TEMP_REG_2, RCV_TEMP_REG_2
    .endif ;TX_L2_ENABLED
    ADD        CUT_THROUGH_BYTE_CNT, CUT_THROUGH_BYTE_CNT, 4
    ADD        R1.b3, R1.b3, 4
    SUB        RCV_TEMP_REG_1, RCV_TEMP_REG_1, 4
    QBLE    NB_PUSH_NEXT_DOUBLE_WORD_TO_FIFO,     RCV_TEMP_REG_1, 4
    .endif
check_for_32bytes:
    .endif
    QBBS	NB_CHECK_AMOUNT_OF_BYTES_BIGGER_32_BANK1, MII_RCV.rx_flags, rx_bank_index_shift   ; check which bank to read

    ; for bank 0, R18.b0 >=32 bytes
    QBGT	NB_DONE, R18_RCV_BYTECOUNT, 32
    ;LDI	R1.b3, buffer_ptr
    QBA	NB_PROCESS_32BYTES

NB_CHECK_AMOUNT_OF_BYTES_BIGGER_32_BANK1:

    ; for bank 1, R18.b0 <32 bytes
    QBLE	NB_DONE, R18_RCV_BYTECOUNT, 32

    .if $defined("ICSS_SWITCH_BUILD")
    QBBC    NB_PROCESS_32BYTES_CT, MII_RCV.tx_flags, cut_through_flag_shift    ;MII_RCV.tx_flags.cut_through_flag
    QBEQ    NB_PROCESS_32BYTES_CT, R1.b3, 0x28
    .if $defined("ICSS_REV1")
    .if !$defined("TX_L2_ENABLED")
    MVIW	TX_DATA_WORD, *R1.b3
    M_PUSH_WORD_CMD
    .else
	AND     RCV_TEMP_REG_2, R2, R2
	MVIW	R2.w0, *R1.b3
    XOUT    TX_L2_BANK_ID, &R2, 2
    AND     R2, RCV_TEMP_REG_2, RCV_TEMP_REG_2
    .endif ;TX_L2_ENABLED
    ADD	CUT_THROUGH_BYTE_CNT, CUT_THROUGH_BYTE_CNT, 2
    .endif
    .if $defined("ICSS_REV2")
    .if !$defined("TX_L2_ENABLED")
    MVID    TX_DATA_DOUBLE_WORD, *R1.b3
    .else
	AND     RCV_TEMP_REG_2, R2, R2
	MVID	R2, *R1.b3
    XOUT    TX_L2_BANK_ID, &R2, 4
    AND     R2, RCV_TEMP_REG_2, RCV_TEMP_REG_2
    .endif ;TX_L2_ENABLED
    ADD        CUT_THROUGH_BYTE_CNT, CUT_THROUGH_BYTE_CNT, 4
    .endif
NB_PROCESS_32BYTES_CT:
    .endif
    LDI	PREVIOUS_R18_RCV_BYTECOUNT, 0x00

;***********************************
; 32 bytes handling, no cut through
;***********************************
NB_PROCESS_32BYTES:	; 32 Bytes of new data are in the buffer

    LDI	R1.b3, buffer_ptr       ; load the buffer pointer

NB_PROCESS_CHECK_BYTE_CNTR:
    ; if this is the first 32 bytes of data, need to init the buffer queues
    QBNE	NB_PROCESS_32BYTES_CHECK_FLAGS, MII_RCV.byte_cntr, 0

    .if $defined("ICSS_SWITCH_BUILD")
    .if    $defined("TWO_PORT_CFG")
    QBNE    NB_PROCESS_32BYTES_CHECK_FLAGS, MII_RCV_PORT.byte_cntr, 0
    .endif ;TWO_PORT_CFG

  QBBC    NB_PROCESS_32BYTES_INIT_PORT_QUEUE, MII_RCV.rx_flags, host_rcv_flag_shift    ;MII_RCV.rx_flags.host_rcv_flag

NB_PROCESS_32BYTES_INIT_HOST_QUEUE:
    .if    $defined("TWO_PORT_CFG")
    LDI        RCV_TEMP_REG_3.w0, P0_QUEUE_DESC_OFFSET
    LDI        RCV_TEMP_REG_3.w2, P0_COL_QUEUE_DESC_OFFSET
    ; Calling for host queue
    LDI        RCV_TEMP_REG_1.b1, 0
    ; get queue
    JAL	CALL_REG, FN_QUEUE_ARBITRATION
    ;RCV_TEMP_REG_1.b0 --> OUTPUT: returns information if the queue has been
    ;aquired successful ((0-failed to aquire; 1-queue; 2-collision)
    QBNE    NB_PROCESS_32BYTES_INIT_HOST_QUEUE_OK, RCV_TEMP_REG_1.b0, 0
    ; OPT: Can't fail
NB_PROCESS_32BYTES_INIT_HOST_QUEUE_FAILED:
    ; clear host receive flag
    CLR        MII_RCV.rx_flags, MII_RCV.rx_flags, host_rcv_flag_shift

    ; ToDo: Do we need to add code for statistics? We should never fail to aquire the queue or the coll-queue
    QBA        NB_PROCESS_32BYTES_INIT_PORT_QUEUE
NB_PROCESS_32BYTES_INIT_HOST_QUEUE_OK:
    ;CLR        MII_RCV.rx_flags.host_collision_queue_selected         ; ToDo: this could be removed when parameter are init with 0 at b/o frame
    ; 2-collision selected?
    QBNE    NB_PROCESS_32BYTES_INIT_HOST_QUEUE_NO_COLL_SELECTED, RCV_TEMP_REG_1.b0, COLLISION_AQUIRED

    ; Initialize the RCV CONTEXT with the data of Host Collision Queue
    SET        MII_RCV.rx_flags, MII_RCV.rx_flags, host_collision_queue_selected_shift
    LDI        RCV_TEMP_REG_1.w0, COL_RX_CONTEXT_P0_OFFSET_ADDR
    LBCO    &MII_RCV.buffer_index, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 10
    .endif ;TWO_PORT_CFG
NB_PROCESS_32BYTES_INIT_HOST_QUEUE_NO_COLL_SELECTED:
    .endif
    ;Initialize the wrkng_wr_ptr and rd_ptr here as the other PRU might have changed them between Now and RCV_FB
    LBCO	&RCV_QUEUE_DESC_REG, QUEUE_DESP_BASE, MII_RCV.rcv_queue_pointer, 4
    AND MII_RCV.rd_ptr , RCV_QUEUE_DESC_REG.rd_ptr , RCV_QUEUE_DESC_REG.rd_ptr
    AND MII_RCV.wrkng_wr_ptr , RCV_QUEUE_DESC_REG.wr_ptr , RCV_QUEUE_DESC_REG.wr_ptr

    ; Rx interrupt pacing
    ; If the rd_ptr & wr_ptr are equal => queue is empty => SET HOST_QUEUE_EMPTY_STATUS
    QBNE    HOST_QUEUE_NOT_EMPTY, RCV_QUEUE_DESC_REG.rd_ptr, RCV_QUEUE_DESC_REG.wr_ptr
    SET     R22, R22, HOST_QUEUE_EMPTY_STATUS

HOST_QUEUE_NOT_EMPTY:
    .if    $defined("TWO_PORT_CFG")
    QBBS    NB_RCV_CONTEXT_INITIALIZED_WITH_COLLISION, MII_RCV.rx_flags, host_collision_queue_selected_shift
    .endif ;TWO_PORT_CFG
    ; Compute the adress in L3 RAM where the packet is received
    SUB	RCV_BUFFER_DESC_OFFSET, MII_RCV.wrkng_wr_ptr, MII_RCV.base_buffer_desc_offset
    LSL	RCV_BUFFER_DESC_OFFSET, RCV_BUFFER_DESC_OFFSET, 3
    ADD	MII_RCV.buffer_index, MII_RCV.base_buffer_index, RCV_BUFFER_DESC_OFFSET

    .if $defined("ICSS_SWITCH_BUILD")
NB_RCV_CONTEXT_INITIALIZED_WITH_COLLISION:

NB_PROCESS_32BYTES_INIT_PORT_QUEUE:
    ; check if selected
    .if    $defined("TWO_PORT_CFG")

  QBBC    NB_PROCESS_32BYTES_CHECK_FLAGS, MII_RCV.rx_flags, fwd_flag_shift

    .if    $defined("PRU0")
    LDI        RCV_TEMP_REG_3.w0, P2_QUEUE_DESC_OFFSET
    LDI        RCV_TEMP_REG_3.w2, P2_COL_QUEUE_DESC_OFFSET
    .else
    LDI        RCV_TEMP_REG_3.w0, P1_QUEUE_DESC_OFFSET
    LDI        RCV_TEMP_REG_3.w2, P1_COL_QUEUE_DESC_OFFSET
    .endif
    ; get queue
    ;CALL    FN_QUEUE_ARBITRATION_PORT_RCV
    ; Calling for port queue
    LDI        RCV_TEMP_REG_1.b1, 1
    JAL	CALL_REG, FN_QUEUE_ARBITRATION
    ;RCV_TEMP_REG_1.b0 --> OUTPUT: returns information if the queue has
    ;been acquired successful ((0-failed to acquire; 1-queue; 2-collision)
    QBNE    NB_PROCESS_32BYTES_INIT_PORT_QUEUE_OK, RCV_TEMP_REG_1.b0, 0
    ; OPT: Can't fail
NB_PROCESS_32BYTES_INIT_PORT_QUEUE_FAILED:


  ; clear port receive flag
    CLR        MII_RCV.rx_flags, MII_RCV.rx_flags, fwd_flag_shift
    ; ToDo: Do we need to add code for statistics? We should never fail to aquire the queue or the coll-queue
    QBA        NB_PROCESS_32BYTES_CHECK_FLAGS
NB_PROCESS_32BYTES_INIT_PORT_QUEUE_OK:
    ;CLR        MII_RCV.rx_flags.port_collision_queue_selected         ; ToDo: this could be removed when parameter are init with 0 at b/o frame
    ; 2-collision selected?
    QBNE    NB_PROCESS_32BYTES_INIT_PORT_QUEUE_NO_COLL_SELECTED, RCV_TEMP_REG_1.b0, 2

    ; Initialize the RCV CONTEXT with the data of Port Collision Queue
    SET        MII_RCV.rx_flags, MII_RCV.rx_flags, port_collision_queue_selected_shift
    .if    $defined("PRU0")
    LDI        RCV_TEMP_REG_1.w0, COL_RX_CONTEXT_P2_OFFSET_ADDR
    .else
    LDI        RCV_TEMP_REG_1.w0, COL_RX_CONTEXT_P1_OFFSET_ADDR
    .endif
    LBCO    &MII_RCV_PORT.buffer_index, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 10

NB_PROCESS_32BYTES_INIT_PORT_QUEUE_NO_COLL_SELECTED:
    ;Initialize the wrkng_wr_ptr and rd_ptr here as the other PRU might have changed them between Now and RCV_FB
    LBCO    &RCV_QUEUE_DESC_REG, QUEUE_DESP_BASE, MII_RCV_PORT.rcv_queue_pointer, 4
    AND MII_RCV_PORT.rd_ptr , RCV_QUEUE_DESC_REG.rd_ptr , RCV_QUEUE_DESC_REG.rd_ptr	;Warning: converted from MOV
    AND MII_RCV_PORT.wrkng_wr_ptr , RCV_QUEUE_DESC_REG.wr_ptr , RCV_QUEUE_DESC_REG.wr_ptr	;Warning: converted from MOV
    QBBS    NB_RCV_PORT_CONTEXT_INITIALIZED_WITH_COLLISION, MII_RCV.rx_flags, port_collision_queue_selected_shift
    ; Compute the adress in L3 RAM where the packet is received
    SUB        RCV_BUFFER_DESC_OFFSET, MII_RCV_PORT.wrkng_wr_ptr, MII_RCV_PORT.base_buffer_desc_offset
    LSL        RCV_BUFFER_DESC_OFFSET, RCV_BUFFER_DESC_OFFSET, 3
    ADD        MII_RCV_PORT.buffer_index, MII_RCV_PORT.base_buffer_index, RCV_BUFFER_DESC_OFFSET

NB_RCV_PORT_CONTEXT_INITIALIZED_WITH_COLLISION:
    .endif ;TWO_PORT_CFG
    .endif ;ICSS_SWITCH_BUILD

NB_PROCESS_32BYTES_CHECK_FLAGS:

    .if $defined("ICSS_SWITCH_BUILD")
    QBBC    NB_PROCESS_32BYTES_CHECK_FWD_FLAG, MII_RCV.rx_flags, host_rcv_flag_shift     ; MII_RCV.rx_flags.host_rcv_flag
    .endif

    QBGE    LESS_THAN_64_BYTES_RCVD, MII_RCV.byte_cntr,  32
    QBA        EXIT_PREPROCESSING_NB

LESS_THAN_64_BYTES_RCVD:
    ;between 32 and 64 bytes
    QBGE    LESS_THAN_32_BYTES_RCVD, MII_RCV.byte_cntr,  0

    .if $defined(PTP)
    LDI    RCV_TEMP_REG_3.w2, PTP_IPV4_UDP_E2E_ENABLE
    LBCO   &RCV_TEMP_REG_3.b2, ICSS_SHARED_CONST, RCV_TEMP_REG_3.w2, 1
    QBEQ   PTP_NOT_ENABLED_RX_B2, RCV_TEMP_REG_3.b2, 0
    QBBC   PTP_NOT_ENABLED_RX_B2, R22, RX_IS_UDP_PTP_BIT
    CLR    R22, R22, RX_IS_UDP_PTP_BIT
    JAL    RCV_TEMP_REG_3.w2, FN_TIMESTAMP_GPTP_PACKET
    JAL    RCV_TEMP_REG_3.w2, FN_CHECK_AND_CLR_PTP_FWD_FLAG
PTP_NOT_ENABLED_RX_B2:
    .endif    ; "PTP"
    QBA       EXIT_PREPROCESSING_NB

LESS_THAN_32_BYTES_RCVD:

    .if $defined(PTP)
    ;we don't want to execute this for UDP frames, since this will be executed again later
    QBBS    PTP_SKIP_EARLY_TS_FOR_UDP_1, R22, RX_IS_UDP_PTP_BIT
    JAL     RCV_TEMP_REG_3.w2, FN_CHECK_AND_CLR_PTP_FWD_FLAG_L2
    JAL     RCV_TEMP_REG_3.w2, FN_TIMESTAMP_GPTP_PACKET
PTP_SKIP_EARLY_TS_FOR_UDP_1:
    .endif    ;PTP

EXIT_PREPROCESSING_NB:
    SBCO    &Ethernet, L3_OCMC_RAM_CONST, MII_RCV.buffer_index, 32
    ADD MII_RCV.byte_cntr , MII_RCV.byte_cntr ,  32     ; increment byte count by 32

    ; Compare current wrk pointer to top_most queue desc pointer ..check for wrap around
    QBNE	RCV_NB_NO_QUEUE_WRAP, MII_RCV.wrkng_wr_ptr, MII_RCV.top_most_buffer_desc_offset
    AND MII_RCV.wrkng_wr_ptr , MII_RCV.base_buffer_desc_offset , MII_RCV.base_buffer_desc_offset
    AND MII_RCV.buffer_index , MII_RCV.base_buffer_index , MII_RCV.base_buffer_index
    QBA	RCV_NB_QUEUE_WRAPPED
RCV_NB_NO_QUEUE_WRAP:
    ADD	MII_RCV.buffer_index, MII_RCV.buffer_index,  32
    ADD	MII_RCV.wrkng_wr_ptr,  MII_RCV.wrkng_wr_ptr,  4

RCV_NB_QUEUE_WRAPPED:

    ; Prepare for next call of RCV_NB ..whether next 32 bytes can be received or not
    QBNE	NB_PROCESS_32BYTES_CHECK_FLAGS_QUEUE_NOT_FULL, MII_RCV.wrkng_wr_ptr, MII_RCV.rd_ptr

    CLR	MII_RCV.rx_flags , MII_RCV.rx_flags , host_rcv_flag_shift
    SET	MII_RCV.rx_flags_extended , MII_RCV.rx_flags_extended , host_queue_overflow_shift
    .if $defined("ICSS_DUAL_EMAC_BUILD")
    ;For EMAC mode, set rx_frame_error bit. This saves cycles in FN_RCV_LB.
    SET	MII_RCV.rx_flags , MII_RCV.rx_flags , 4
    .endif

NB_PROCESS_32BYTES_CHECK_FLAGS_QUEUE_NOT_FULL:
    .if $defined("ICSS_SWITCH_BUILD")
NB_PROCESS_32BYTES_CHECK_FWD_FLAG:
    .if    $defined("TWO_PORT_CFG")

    QBBC    NB_PROCESS_32BYTES_CHECK_FLAG_DONE, MII_RCV.rx_flags, 1    ;MII_RCV.rx_flags.fwd_flag
    SBCO    &Ethernet, L3_OCMC_RAM_CONST, MII_RCV_PORT.buffer_index, 32
    ADD        MII_RCV_PORT.byte_cntr, MII_RCV_PORT.byte_cntr, 32

    ; Compare current wrk pointer to top_most queue desc pointer ..check for wrap around
    QBNE    RCV_NB_PORT_NO_QUEUE_WRAP, MII_RCV_PORT.wrkng_wr_ptr, MII_RCV_PORT.top_most_buffer_desc_offset
    AND MII_RCV_PORT.wrkng_wr_ptr , MII_RCV_PORT.base_buffer_desc_offset , MII_RCV_PORT.base_buffer_desc_offset	;Warning: converted from MOV
    AND MII_RCV_PORT.buffer_index , MII_RCV_PORT.base_buffer_index , MII_RCV_PORT.base_buffer_index	;Warning: converted from MOV
    QBA        RCV_NB_PORT_QUEUE_WRAPPED
RCV_NB_PORT_NO_QUEUE_WRAP:
    ADD        MII_RCV_PORT.buffer_index,     MII_RCV_PORT.buffer_index, 32
    ADD        MII_RCV_PORT.wrkng_wr_ptr,  MII_RCV_PORT.wrkng_wr_ptr, 4
RCV_NB_PORT_QUEUE_WRAPPED:

    ; Prepare for next call of RCV_NB ..whether next 32 bytes can be received or not
    QBNE    NB_PROCESS_32BYTES_CHECK_FWD_FLAG_NOT_FULL, MII_RCV_PORT.wrkng_wr_ptr, MII_RCV_PORT.rd_ptr
    CLR        MII_RCV.rx_flags, MII_RCV.rx_flags, fwd_flag_shift
    SET        MII_RCV.rx_flags_extended, MII_RCV.rx_flags_extended, port_queue_overflow_shift

NB_PROCESS_32BYTES_CHECK_FWD_FLAG_NOT_FULL:

NB_PROCESS_32BYTES_CHECK_FLAG_DONE:
    .endif ;TWO_PORT_CFG

NB_PROCESS_DONE:
    .endif
    XOR		MII_RCV.rx_flags, MII_RCV.rx_flags, (1<<rx_bank_index_shift)	; toggle rx_bank_index flag

NB_DONE:

    ; restore call register pointer
    AND CALL_REG , L1_CALL_REG , L1_CALL_REG

    ; store task parameters from parameter bank
NB_STORE_CONTEXT:
    LDI	R0.b0, SHIFT_NONE
    .if $defined("PRU0")
    XOUT	BANK1, &MII_RCV, $sizeof(MII_RCV)
    .else
    XOUT	BANK2, &MII_RCV, $sizeof(MII_RCV)
    .endif

    .if $defined("ICSS_SWITCH_BUILD")
    .if	$defined("TWO_PORT_CFG")
    .if $defined("PRU0")
    LDI        R0.b0, SHIFT_R14_TO_R0
    XOUT    BANK0, &MII_RCV_PORT, $sizeof(MII_RCV_PORT)
    .else
    LDI        R0.b0, SHIFT_R14_TO_R4
    XOUT    BANK0, &MII_RCV_PORT, $sizeof(MII_RCV_PORT)
    .endif
    QBBS    NB_PROCESS_CUT_THROUGH, MII_RCV.tx_flags, cut_through_flag_shift    ;MII_RCV.tx_flags.cut_through_flag
    .endif ;TWO_PORT_CFG
    .endif

    .if $defined("ICSS_REV1")
    M_RCV_RX_EOF_CHECK_ICSS_REV1        ; check for EOF for REV1
    .endif
    .if $defined("ICSS_REV2")
    M_RCV_RX_EOF_CHECK_ICSS_REV2        ; check for EOF for REV2
    .endif

NB_DONE_NO_PARAM_STORE:
    JMP		TASK_EXECUTION_FINISHED

process_rx_eof_rx_nb:
    JMP		FN_RCV_LB

    .if $defined("ICSS_SWITCH_BUILD")
    .if    $defined("TWO_PORT_CFG")
NB_PROCESS_CUT_THROUGH:
    ; check for RX EOF condition
    .if $defined("ICSS_REV1")
    .if $defined("PRU0")
    QBBC	FN_RCV_NB_CT_NO_RxEOF, R31, 30	 ;replaced: QBBC	FN_RCV_NB_CT_NO_RxEOF, R31.t30
    .else
    QBBC	FN_RCV_NB_CT_NO_RxEOF, R31, 31	 ;replaced: QBBC	FN_RCV_NB_CT_NO_RxEOF, R31.t31
    .endif
    .endif
    .if $defined("ICSS_REV2")
    QBBC    FN_RCV_NB_CT_NO_RxEOF, R31, 20        ; Port0 Rx EOF
    .endif

FN_RCV_NB_CT_RxEOF:
    QBBS    CT_NB_XIN_UPPER_BANK, MII_RCV.rx_flags, rx_bank_index_shift    ;MII_RCV.rx_flags.rx_bank_index

CT_NB_XIN_LOWER_BANK:

    XIN        RX_L2_BANK0_ID, &R18, RANGE_R18_b0
    XIN        RX_L2_BANK0_ID, &R2, RANGE_R2_R13                ; XIN RX L2 bank 0

    QBA        CT_NB_CHECK_AMOUNT_OF_BYTES
CT_NB_XIN_UPPER_BANK:

    XIN        RX_L2_BANK1_ID, &R18, RANGE_R18_b0
    XIN        RX_L2_BANK1_ID, &R2, RANGE_R2_R13                ; XIN RX L2 bank 1

CT_NB_CHECK_AMOUNT_OF_BYTES:
    ; Put the remaining bytes in Tx_FIFO at this point of time itself
    AND        RCV_TEMP_REG_1.b0, CUT_THROUGH_BYTE_CNT, 0x003f
    ; Below code added to fix a bug when for a 65 byte packet R18 has 1 but cut_through_byte_cnt is 62.
    ; Check whether cut_through_byte_cnt is more than R18 count
    QBGE    CT_R18_IS_GREATER_OR_EQUAL, RCV_TEMP_REG_1.b0, R18_RCV_BYTECOUNT
    LDI        RCV_TEMP_REG_2.b0, 64   ; Subtract from 64 bytes
    SUB        RCV_TEMP_REG_1.b0, RCV_TEMP_REG_2.b0, RCV_TEMP_REG_1.b0
    ; L2 FIFO wrapped around but few bytes of upper bank didn't get pushed in fifo. It is possible that RX EOF is detected before pushing
    ; the last two bytes and R18 gets wrapped around by the time we reach here
    ADD        RCV_TEMP_REG_1.b0, RCV_TEMP_REG_1.b0, R18_RCV_BYTECOUNT
    QBA        CT_R18_IS_LESS
CT_R18_IS_GREATER_OR_EQUAL:

    SUB        RCV_TEMP_REG_1.b0, R18_RCV_BYTECOUNT, RCV_TEMP_REG_1.b0

CT_R18_IS_LESS:
    .if $defined("ICSS_REV1")
    ;QBEQ	CT_PUSH_LB_LASTBYTE_nb, RCV_TEMP_REG_1.b0,  1
    LSR	loop_cnt, RCV_TEMP_REG_1.b0, 1
    ; OPT: Use function for the loop below
    ;QBEQ	ct_assert_eof_nb, loop_cnt, 0x0000
    LOOP	CT_NB_ENDLOOP, loop_cnt
    .if !$defined("TX_L2_ENABLED")
    MVIW	TX_DATA_WORD, *TX_DATA_POINTER
    M_PUSH_WORD_CMD
    .else
	AND     RCV_TEMP_REG_3, R2, R2
	MVIW	R2.w0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 2
    AND     R2, RCV_TEMP_REG_3, RCV_TEMP_REG_3
    .endif ;TX_L2_ENABLED
    ADD	TX_DATA_POINTER, TX_DATA_POINTER, 2
CT_NB_ENDLOOP:
    QBBC	ct_assert_eof_nb, RCV_TEMP_REG_1.b0, 0
CT_PUSH_LB_LASTBYTE_nb:
    .if !$defined("TX_L2_ENABLED")
    MVIB	TX_DATA_BYTE, *R1.b3
    M_PUSH_BYTE
    .else
	AND     RCV_TEMP_REG_3, R2, R2
	MVIB	R2.b0, *R1.b3
    XOUT    TX_L2_BANK_ID, &R2, 1
    AND     R2, RCV_TEMP_REG_3, RCV_TEMP_REG_3
    .endif ;TX_L2_ENABLED
ct_assert_eof_nb:
    ;M_PUSH_TX_EOF
    .endif
    .if $defined("ICSS_REV2")
    LSR        loop_cnt, RCV_TEMP_REG_1.b0, 2

    loop    CT_NB_ENDLOOP, loop_cnt
    .if !$defined("TX_L2_ENABLED")
    MVID    TX_DATA_DOUBLE_WORD, *TX_DATA_POINTER
    ADD        TX_DATA_POINTER, TX_DATA_POINTER, 4
    .else
	AND     RCV_TEMP_REG_3, R2, R2
	MVID	R2, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 4
    AND     R2, RCV_TEMP_REG_3, RCV_TEMP_REG_3
    .endif ;TX_L2_ENABLED
CT_NB_ENDLOOP:

    AND        RCV_TEMP_REG_2.b0, RCV_TEMP_REG_1.b0, 0x03
    QBEQ    ct_assert_eof_nb, RCV_TEMP_REG_2.b0, 0
    QBEQ    CT_PUSH_LB_LASTBYTE_nb, RCV_TEMP_REG_2.b0, 1
    .if !$defined("TX_L2_ENABLED")
    MVIW    TX_DATA_WORD, *TX_DATA_POINTER
    .else
	AND     RCV_TEMP_REG_3, R2, R2
	MVIW	R2.w0, *TX_DATA_POINTER
    XOUT    TX_L2_BANK_ID, &R2, 2
    AND     R2, RCV_TEMP_REG_3, RCV_TEMP_REG_3
    .endif ;TX_L2_ENABLED
    ADD        TX_DATA_POINTER, TX_DATA_POINTER, 2
    QBEQ    ct_assert_eof_nb, RCV_TEMP_REG_2.b0, 2
CT_PUSH_LB_LASTBYTE_nb:
    .if !$defined("TX_L2_ENABLED")
    MVIB    TX_DATA_BYTE, *R1.b3
    .else
	AND     RCV_TEMP_REG_3, R2, R2
	MVIB	R2.b0, *R1.b3
    XOUT    TX_L2_BANK_ID, &R2, 1
    AND     R2, RCV_TEMP_REG_3, RCV_TEMP_REG_3
    .endif ;TX_L2_ENABLED

ct_assert_eof_nb:
    ; Insert the TX_EOF for outgoing frame
    LDI        R31.w2, 0x2000
    .endif
    ADD        CUT_THROUGH_BYTE_CNT, CUT_THROUGH_BYTE_CNT, RCV_TEMP_REG_1.b0
ct_deassert_ct:
SKIP_CRC_PUSH:

    LDI        R0.b0, SHIFT_NONE
    .if    $defined("PRU0")
    XOUT    BANK1, &MII_RCV, $sizeof(MII_RCV)
    .else
    XOUT    BANK2, &MII_RCV, $sizeof(MII_RCV)
    .endif
    CLR     R22, R22, PACKET_TX_ALLOWED
;Check for CRC
    LDI	RCV_TEMP_REG_2.w0, 0x0204
    LBCO	&RCV_TEMP_REG_1, ICSS_INTC_CONST, RCV_TEMP_REG_2.w0, 4
    .if $defined("PRU0")
    QBBC	NO_CRC_ERROR_CT, RCV_TEMP_REG_1, 4	 ;replaced: QBBC    NO_CRC_ERROR_CT, RCV_TEMP_REG_1.t4
    .else
    QBBC	NO_CRC_ERROR_CT, RCV_TEMP_REG_1, 16	 ;replaced: QBBC    NO_CRC_ERROR_CT, RCV_TEMP_REG_1.t16
    .endif
    LDI	RCV_TEMP_REG_3 , RX_CRC_COUNT_OFFSET

    ;Add to statistics counter
    LBCO	&RCV_TEMP_REG_1, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    ADD	RCV_TEMP_REG_1, RCV_TEMP_REG_1, 1
    SBCO	&RCV_TEMP_REG_1, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
NO_CRC_ERROR_CT:
    .endif ;TWO_PORT_CFG
    .endif ;ICSS_SWITCH_BUILD

;****************************************************************************
;
;     NAME			: FN_RCV_LB
;     DESCRIPTION	: receives the last block(s) of RX L2
;     RETURNS		:
;     ARGS			:
;     USES 		:
;     INVOKES 		:
;
;****************************************************************************
FN_RCV_LB:

    ; Check if RCV_Active is set. If not drop the frame. It handles the case of undersize errors
    QBBS	RCV_LB_PROCESS_NORMAL, R23, Rcv_active

LB_CHECK_ERRORS:
    ;Short frame received then increment the error offset statistics
    LDI	RCV_TEMP_REG_3 , RX_ERROR_OFFSET
    LBCO	&RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    ADD	RCV_TEMP_REG_2, RCV_TEMP_REG_2, 1
    SBCO	&RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    .if $defined("ICSS_DUAL_EMAC_BUILD")
    .if $defined("HALF_DUPLEX_ENABLED")
    ;check R31 error
    QBBC	NO_R31_ERROR, R31, 19       ; if cleared so no error and continue normal operation
    ;else clear the error
    M_CMD16 D_RX_ERROR_CLEAR        ; else clear error flag

NO_R31_ERROR:
    ;check for SFD Errors
    .if $defined("PRU0")
    LBCO	&RCV_TEMP_REG_3.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR0, 1     ;read the PRUSS_MII_RT_RX_ERR0 register
    QBBC	CHECK_FOR_SHORT_SFD1, RCV_TEMP_REG_3, 1         ; check if not error then jump else count stats
    ;clear error and writeback
    SET	RCV_TEMP_REG_3 , RCV_TEMP_REG_3 , 1
    SBCO	&RCV_TEMP_REG_3.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR0, 1
    QBA     COUNT_SFD_ERROR1
CHECK_FOR_SHORT_SFD1:
    QBBC	NO_PREAMBLE_ERROR, RCV_TEMP_REG_3, 0       ; check if not error then jump else count stats
    ;clear error and writeback
    SET	RCV_TEMP_REG_3 , RCV_TEMP_REG_3 , 0
    SBCO	&RCV_TEMP_REG_3.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR0, 1
    .else
    LBCO	&RCV_TEMP_REG_3.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR1, 1     ;read the PRUSS_MII_RT_RX_ERR0 register
    QBBC	CHECK_FOR_SHORT_SFD1, RCV_TEMP_REG_3, 1         ; check if not error then jump else count stats
    ;clear error and writeback
    SET	RCV_TEMP_REG_3 , RCV_TEMP_REG_3 , 1
    SBCO	&RCV_TEMP_REG_3.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR1, 1
    QBA     COUNT_SFD_ERROR1
CHECK_FOR_SHORT_SFD1:
    QBBC	NO_PREAMBLE_ERROR, RCV_TEMP_REG_3, 0        ; check if not error then jump else count stats
    ;clear error and writeback
    SET	RCV_TEMP_REG_3 , RCV_TEMP_REG_3 , 0
    SBCO	&RCV_TEMP_REG_3.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR1, 1
    .endif

COUNT_SFD_ERROR1:
        ;increment and store the count
    LDI	RCV_TEMP_REG_3 , SFD_ERROR_OFFSET
    LBCO	&RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    ADD	RCV_TEMP_REG_2, RCV_TEMP_REG_2, 1
    SBCO	&RCV_TEMP_REG_2, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    .endif ;HALF_DUPLEX_ENABLED
    .endif ;ICSS_DUAL_EMAC_BUILD
NO_PREAMBLE_ERROR:

    ; RCV_CONTEXT is not initialized for this error frame.
    ; reset RX FIFO, this places the R18 counter back to position 0
    CLR	R23 , R23 , Rcv_active
    M_SET_CMD	D_RESET_RXFIFO
    QBA		LB_NO_RX_STAT
RCV_LB_PROCESS_NORMAL:

    ; for XIN, make sure that shift is set to 0
    LDI	R0.b0, SHIFT_NONE
    ; restore task parameters from parameter bank
    .if $defined("PRU0")
    XIN	BANK1, &MII_RCV, $sizeof(MII_RCV)
    .else
    XIN	BANK2, &MII_RCV, $sizeof(MII_RCV)
    .endif

    .if    $defined("TWO_PORT_CFG")
     ; OPT: MOve the port context read after the XIN of data
    ; for port receive
    .if    $defined("PRU0")
    LDI        R0.b0, SHIFT_R14_TO_R0
    XIN        BANK0, &MII_RCV_PORT, $sizeof(MII_RCV_PORT)
    .else
    LDI        R0.b0, SHIFT_R14_TO_R4
    XIN        BANK0, &MII_RCV_PORT, $sizeof(MII_RCV_PORT)
    .endif
    .endif ;TWO_PORT_CFG
    QBBS	LB_RESET_RX_FIFO, MII_RCV.rx_flags, rx_frame_error_shift
    ; depending on the bank index, we load bank 0 or bank 1
    QBBS	LB_XIN_UPPER_BANK, MII_RCV.rx_flags, rx_bank_index_shift

LB_XIN_LOWER_BANK:
    XIN	RX_L2_BANK0_ID, &R18, RANGE_R18_b0
    XIN	RX_L2_BANK0_ID, &R2, RANGE_R2_R13	; XIN RX L2 bank 0

    ; have we received more than 32 bytes?
    QBGE	LB_XIN_STORE_LESS_THAN_32_FROM_LOWER_BANK, R18_RCV_BYTECOUNT, 32
    QBA		LB_STORE_FIRST_32_BYTES

LB_XIN_UPPER_BANK:
    XIN	RX_L2_BANK1_ID, &R18, RANGE_R18_b0
    XIN	RX_L2_BANK1_ID, &R2, RANGE_R2_R13	; XIN RX L2 bank 1

    ;For PTP frames if flow comes here it
    ; can only mean that 32-64 bytes have been received
    .if $defined("PTP")
    LDI    RCV_TEMP_REG_1.w0, PTP_IPV4_UDP_E2E_ENABLE
    LBCO   &RCV_TEMP_REG_1.b0, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, 1
    QBEQ   PTP_NOT_ENABLED_RX_LB, RCV_TEMP_REG_1.b0, 0
    QBBC   PTP_NOT_ENABLED_RX_LB, R22, RX_IS_UDP_PTP_BIT
    CLR    R22, R22, RX_IS_UDP_PTP_BIT
    JAL    RCV_TEMP_REG_3.w2, FN_TIMESTAMP_GPTP_PACKET
    JAL    RCV_TEMP_REG_3.w2, FN_CHECK_AND_CLR_PTP_FWD_FLAG
PTP_NOT_ENABLED_RX_LB:
    .endif    ; "PTP"

    ; have we received more than 32 bytes?
    QBLE	LB_XIN_STORE_LESS_THAN_32_FROM_UPPER_BANK, R18_RCV_BYTECOUNT, 32

LB_STORE_FIRST_32_BYTES:
    .if $defined("ICSS_SWITCH_BUILD")
    QBBC    LB_PROCESS_32BYTES_CHECK_FWD_FLAG, MII_RCV.rx_flags, host_rcv_flag_shift    ;MII_RCV.rx_flags.host_rcv_flag
    .endif
    .if $defined(PTP)
    ADD     RCV_TEMP_REG_3.w2, MII_RCV.byte_cntr, 32
    QBLT    MORE_THAN_32_BYTES_RCVD, RCV_TEMP_REG_3.w2, 32

    ;we don't want to execute this for UDP frames, since this will be executed again later
    QBBS    MORE_THAN_32_BYTES_RCVD, R22, RX_IS_UDP_PTP_BIT
    JAL     RCV_TEMP_REG_3.w2, FN_CHECK_AND_CLR_PTP_FWD_FLAG_L2
    JAL     RCV_TEMP_REG_3.w2, FN_TIMESTAMP_GPTP_PACKET

MORE_THAN_32_BYTES_RCVD:
    .endif ;PTP
    SBCO	&Ethernet, L3_OCMC_RAM_CONST, MII_RCV.buffer_index, 32
    ADD		MII_RCV.byte_cntr, MII_RCV.byte_cntr,  32

    ; Update the buffer descriptor for the received packet
    ; Compare current wrk pointer to top_most queue desc pointer ..check for wrap around
    QBNE	RCV_LB_NO_QUEUE_WRAP_LOWER, MII_RCV.wrkng_wr_ptr, MII_RCV.top_most_buffer_desc_offset
    AND MII_RCV.wrkng_wr_ptr , MII_RCV.base_buffer_desc_offset , MII_RCV.base_buffer_desc_offset
    AND MII_RCV.buffer_index , MII_RCV.base_buffer_index , MII_RCV.base_buffer_index
    QBA		RCV_LB_QUEUE_WRAPPED_LOWER
RCV_LB_NO_QUEUE_WRAP_LOWER:
    ADD		MII_RCV.buffer_index, MII_RCV.buffer_index,  32
    ADD		MII_RCV.wrkng_wr_ptr,  MII_RCV.wrkng_wr_ptr,  4

RCV_LB_QUEUE_WRAPPED_LOWER:
    ; Check if the queue got completely filled with the last few bytes and the remaining bytes cannot be added.
    ;This scenario of queue overflow has been verified by dry run.
    QBNE	LB_PROCESS_32BYTES_CHECK_FLAGS_QUEUE_NOT_FULL_1, MII_RCV.wrkng_wr_ptr, MII_RCV.rd_ptr
    CLR	MII_RCV.rx_flags , MII_RCV.rx_flags , host_rcv_flag_shift
    SET	MII_RCV.rx_flags_extended , MII_RCV.rx_flags_extended , host_queue_overflow_shift
    .if $defined("ICSS_DUAL_EMAC_BUILD")
    ;For EMAC mode, set rx_frame_error bit.
    SET	MII_RCV.rx_flags , MII_RCV.rx_flags , 4
    .endif
LB_PROCESS_32BYTES_CHECK_FLAGS_QUEUE_NOT_FULL_1:
    .if $defined("ICSS_SWITCH_BUILD")
LB_PROCESS_32BYTES_CHECK_FWD_FLAG:
    .if    $defined("TWO_PORT_CFG")
    QBBC    LB_PROCESS_32BYTES_CHECK_FLAG_DONE, MII_RCV.rx_flags, fwd_flag_shift    ;MII_RCV.rx_flags.fwd_flag

    SBCO    &Ethernet, L3_OCMC_RAM_CONST, MII_RCV_PORT.buffer_index, 32
    ADD        MII_RCV_PORT.byte_cntr, MII_RCV_PORT.byte_cntr, 32

    ; Update the buffer descriptor for the received packet
    ; Compare current wrk pointer to top_most queue desc pointer ..check for wrap around
    QBNE    RCV_LB_NO_PORT_QUEUE_WRAP_LOWER, MII_RCV_PORT.wrkng_wr_ptr, MII_RCV_PORT.top_most_buffer_desc_offset
    AND MII_RCV_PORT.wrkng_wr_ptr , MII_RCV_PORT.base_buffer_desc_offset , MII_RCV_PORT.base_buffer_desc_offset	;Warning: converted from MOV
    AND MII_RCV_PORT.buffer_index , MII_RCV_PORT.base_buffer_index , MII_RCV_PORT.base_buffer_index	;Warning: converted from MOV
    QBA        RCV_LB_PORT_QUEUE_WRAPPED_LOWER
RCV_LB_NO_PORT_QUEUE_WRAP_LOWER:
    ADD        MII_RCV_PORT.buffer_index, MII_RCV_PORT.buffer_index, 32
    ADD        MII_RCV_PORT.wrkng_wr_ptr,  MII_RCV_PORT.wrkng_wr_ptr, 4

RCV_LB_PORT_QUEUE_WRAPPED_LOWER:
    ; Prepare for next call of RCV_LB ..whether next 32 bytes can be received or not
    QBNE    LB_PROCESS_32BYTES_CHECK_FLAGS_PORT_QUEUE_NOT_FULL_1, MII_RCV_PORT.wrkng_wr_ptr, MII_RCV_PORT.rd_ptr
    CLR        MII_RCV.rx_flags, MII_RCV.rx_flags, fwd_flag_shift    ;MII_RCV.rx_flags.fwd_flag
    SET        MII_RCV.rx_flags_extended, MII_RCV.rx_flags_extended, port_queue_overflow_shift    ;MII_RCV.rx_flags_extended.port_queue_overflow

LB_PROCESS_32BYTES_CHECK_FLAGS_PORT_QUEUE_NOT_FULL_1:

LB_PROCESS_32BYTES_CHECK_FLAG_DONE:
    .endif ;TWO_PORT_CFG
    .endif ;ICSS_SWITCH_BUILD
    QBBC	LB_STORE_UPPER_DATA, MII_RCV.rx_flags, rx_bank_index_shift

    ; get and store lower data
    XIN	RX_L2_BANK0_ID, &R2, RANGE_R2_R9	; recieve the remaining data
    QBA	LB_XIN_STORE_LESS_THAN_32_FROM_LOWER_BANK
LB_STORE_UPPER_DATA:
    ; get and store upper data
    XIN	RX_L2_BANK1_ID, &R2, RANGE_R2_R13       ; recieve the remaining data

LB_XIN_STORE_LESS_THAN_32_FROM_UPPER_BANK:
    SUB	R0.b1, R18, 32          ; count remaining data
    QBA	LB_STORE_FROM_UPPER_BUFFER
LB_XIN_STORE_LESS_THAN_32_FROM_LOWER_BANK:
    AND R0.b1 , R18 , R18
LB_STORE_FROM_UPPER_BUFFER:

    QBBS    RCV_LB_APPEND_TS, R22, 14    ;check PTP flag
    ; Check if 0 bytes are there to store
    QBEQ	LB_PROCESS_CHECK_FWD_FLAG, R0.b1, 0

    ; Receive for Host Queue
    .if $defined("ICSS_SWITCH_BUILD")
    QBBC    LB_PROCESS_CHECK_FWD_FLAG, MII_RCV.rx_flags, host_rcv_flag_shift    ;MII_RCV.rx_flags.host_rcv_flag
    .endif
    SBCO	&Ethernet, L3_OCMC_RAM_CONST, MII_RCV.buffer_index, b1

    ADD	    MII_RCV.byte_cntr, MII_RCV.byte_cntr, R0.b1     ; increment the count by R1 bytes
    QBGE	LB_PROCESS_CHECK_FWD_FLAG, R0.b1, 4	            ;If only CRC is left to store then skip
    QBNE	RCV_LB_NO_QUEUE_WRAP_2, MII_RCV.wrkng_wr_ptr, MII_RCV.top_most_buffer_desc_offset
    AND     MII_RCV.wrkng_wr_ptr , MII_RCV.base_buffer_desc_offset , MII_RCV.base_buffer_desc_offset
    QBA		RCV_LB_QUEUE_WRAPPED_2
RCV_LB_NO_QUEUE_WRAP_2:
    ADD		MII_RCV.wrkng_wr_ptr,  MII_RCV.wrkng_wr_ptr,  4
RCV_LB_QUEUE_WRAPPED_2:
    QBA     RCV_LB_CHECK_OVERFLOW
RCV_LB_APPEND_TS:
    ;--------------Logic to append 10 bytes timestamp to the end of packet----------
    .if $defined("ICSS_SWITCH_BUILD")
        QBBC    LB_PROCESS_CHECK_FWD_FLAG, MII_RCV.rx_flags, host_rcv_flag_shift    ;MII_RCV.rx_flags.host_rcv_flag
    .endif

    ;If there are 0 bytes to store then append timestamp in new block
    QBEQ    LB_TS_STORE_TS, R0.b1, 0
    ADD	    MII_RCV.byte_cntr, MII_RCV.byte_cntr, R0.b1     ; increment the count by b1 bytes
    QBGE	LB_TS_STORE_TS, R0.b1, 4	                    ;If only CRC is left to store then skip and append TS

    SBCO	&Ethernet, L3_OCMC_RAM_CONST, MII_RCV.buffer_index, b1  ;Store the data

    ;Compare current wrk pointer to top_most queue desc pointer ..check for wrap around
    QBNE	LB_TS_NO_WRAP_1, MII_RCV.wrkng_wr_ptr, MII_RCV.top_most_buffer_desc_offset
    AND     MII_RCV.wrkng_wr_ptr , MII_RCV.base_buffer_desc_offset , MII_RCV.base_buffer_desc_offset
    AND     MII_RCV.buffer_index , MII_RCV.base_buffer_index , MII_RCV.base_buffer_index
    QBA		LB_TS_STORE_TS
LB_TS_NO_WRAP_1:
    ADD		MII_RCV.buffer_index, MII_RCV.buffer_index,  32
    ADD		MII_RCV.wrkng_wr_ptr,  MII_RCV.wrkng_wr_ptr,  4

LB_TS_STORE_TS:   ;store timestamp in new 32B block

    ;Load offset
    .if $defined(PTP)
        M_GPTP_LOAD_TS_OFFSET
    .endif

    ;Load the TS from Shared RAM
    LBCO    &R10, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, 10
    ;Store into L3 OCMC
    SBCO	&R10, L3_OCMC_RAM_CONST, MII_RCV.buffer_index, 10

    ;check wraparound
    QBNE	LB_TS_NO_WRAP_2, MII_RCV.wrkng_wr_ptr, MII_RCV.top_most_buffer_desc_offset
    AND     MII_RCV.wrkng_wr_ptr , MII_RCV.base_buffer_desc_offset , MII_RCV.base_buffer_desc_offset
    QBA		RCV_LB_CHECK_OVERFLOW
LB_TS_NO_WRAP_2:
    ADD		MII_RCV.wrkng_wr_ptr,  MII_RCV.wrkng_wr_ptr,  4

RCV_LB_CHECK_OVERFLOW:
    .if $defined("ICSS_DUAL_EMAC_BUILD")
    ; Check if the queue got completely filled with the last few bytes.
    ;If yes, the wr_ptr and rd_prt might become equal and there could be
    ;possible data overwrite when the next packet arrives.
    ;This has been verfied by dry run.
    QBNE	LB_PROCESS_CHECK_FLAGS_QUEUE_NOT_FULL, MII_RCV.wrkng_wr_ptr, MII_RCV.rd_ptr
    CLR	MII_RCV.rx_flags , MII_RCV.rx_flags , 0
    SET	MII_RCV.rx_flags_extended , MII_RCV.rx_flags_extended , 6
    ;For EMAC mode, set rx_frame_error bit
    SET	MII_RCV.rx_flags , MII_RCV.rx_flags , 4

LB_PROCESS_CHECK_FLAGS_QUEUE_NOT_FULL:
    .endif ;ICSS_DUAL_EMAC_BUILD
LB_PROCESS_CHECK_FWD_FLAG:
    .if $defined("ICSS_SWITCH_BUILD")
    ; Receive for Port Queue
    .if    $defined("TWO_PORT_CFG")
    QBBC    LB_CLEAR_RX_FIFO, MII_RCV.rx_flags, fwd_flag_shift    ;MII_RCV.rx_flags.fwd_flag
    SBCO    &Ethernet, L3_OCMC_RAM_CONST, MII_RCV_PORT.buffer_index, b1
    ADD        MII_RCV_PORT.byte_cntr, MII_RCV_PORT.byte_cntr, R0.b1
    QBGE    LB_CLEAR_RX_FIFO, R0.b1, 4
    QBNE    RCV_LB_NO_PORT_QUEUE_WRAP_2, MII_RCV_PORT.wrkng_wr_ptr, MII_RCV_PORT.top_most_buffer_desc_offset
    AND MII_RCV_PORT.wrkng_wr_ptr , MII_RCV_PORT.base_buffer_desc_offset , MII_RCV_PORT.base_buffer_desc_offset	;Warning: converted from MOV
    QBA        RCV_LB_PORT_QUEUE_WRAPPED_2
RCV_LB_NO_PORT_QUEUE_WRAP_2:
    ADD        MII_RCV_PORT.wrkng_wr_ptr,  MII_RCV_PORT.wrkng_wr_ptr, 4
RCV_LB_PORT_QUEUE_WRAPPED_2:
    .endif ;TWO_PORT_CFG

LB_CLEAR_RX_FIFO:

    QBBS    LB_CHECK_RX_ERRORS, MII_RCV.rx_flags, host_rcv_flag_shift    ;MII_RCV.rx_flags.host_rcv_flag
    QBBC    LB_RESET_RX_FIFO, MII_RCV.rx_flags, fwd_flag_shift    ;MII_RCV.rx_flags.fwd_flag

LB_CHECK_RX_ERRORS:
    .endif  ;ICSS_SWITCH_BUILD
    ; Check for CRC Error in the received frame
    LDI	RCV_TEMP_REG_1.w0, 0x0204
    LBCO	&RCV_TEMP_REG_2, ICSS_INTC_CONST, RCV_TEMP_REG_1.w0, 4      ;load value of INTC_SRSR1

;check odd nibble error
CHECK_MISALIGNMENT_ERROR:
    .if $defined("PRU0")
    QBBC	CHECK_CRC_ERROR, RCV_TEMP_REG_2, 5      ; if interrupt line is clear check for CRC error else continue
    .else
    QBBC	CHECK_CRC_ERROR, RCV_TEMP_REG_2, 17     ; if interrupt line is clear check for CRC error else continue
    .endif
    LDI	RCV_TEMP_REG_3 , RX_MISALIGNMENT_COUNT_OFFSET

;Add to statistics counter
    LBCO	&RCV_TEMP_REG_1, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    ADD	RCV_TEMP_REG_1, RCV_TEMP_REG_1, 1
    SBCO	&RCV_TEMP_REG_1, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    .if $defined("ICSS_DUAL_EMAC_BUILD")
    SET	MII_RCV.rx_flags , MII_RCV.rx_flags , 4
    .endif

CHECK_CRC_ERROR:
    .if $defined("PRU0")
    QBBC	LB_CHECK_MIN_FRM_ERR, RCV_TEMP_REG_2, 4
    .else
    QBBC	LB_CHECK_MIN_FRM_ERR, RCV_TEMP_REG_2, 16
    .endif
    LDI	RCV_TEMP_REG_3 , RX_CRC_COUNT_OFFSET

;Add to statistics counter
    LBCO	&RCV_TEMP_REG_1, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    ADD	RCV_TEMP_REG_1, RCV_TEMP_REG_1, 1
    SBCO	&RCV_TEMP_REG_1, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    SET	MII_RCV.rx_flags , MII_RCV.rx_flags , rx_frame_error_shift

    ;Check whether received frame length is less then defined min value
LB_CHECK_MIN_FRM_ERR:
    .if $defined("PRU0")
    LBCO	&RCV_TEMP_REG_2.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR0, 1
    .else
    LBCO	&RCV_TEMP_REG_2.b0, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR1, 1
    .endif
    QBBC	LB_CHECK_MAX_FRM_ERR, RCV_TEMP_REG_2, 2         ; check if undersize error exist
    LDI	RCV_TEMP_REG_3 , RX_UNDERSIZED_FRAME_OFFSET
    QBA	LB_CLR_RX_ERROR_REG

    ;Check whether received frame length is more then defined max value
LB_CHECK_MAX_FRM_ERR:
    QBBC	LB_RESET_RX_FIFO, RCV_TEMP_REG_2, 3         ; check for oversize error
    LDI	RCV_TEMP_REG_3 , RX_OVERSIZED_FRAME_OFFSET

LB_CLR_RX_ERROR_REG:

;Add to statistics counter
    LBCO	&RCV_TEMP_REG_1, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    ADD	RCV_TEMP_REG_1, RCV_TEMP_REG_1, 1
    SBCO	&RCV_TEMP_REG_1, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4

    ; Clear the RX MIN or RX MAX ERROR
    LDI	RCV_TEMP_REG_2.b1, 0x0c
    .if $defined("PRU0")
    SBCO	&RCV_TEMP_REG_2.b1, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR0, 1
    .else
    SBCO	&RCV_TEMP_REG_2.b1, MII_RT_CFG_CONST, ICSS_MIIRT_RXERR1, 1
    .endif
    SET	MII_RCV.rx_flags , MII_RCV.rx_flags , rx_frame_error_shift

LB_RESET_RX_FIFO:
    ; reset RX fifo, this places the R18 counter back to position 0
    M_SET_CMD	D_RESET_RXFIFO


    QBBC	LB_NO_RX_FRAME_ERROR, MII_RCV.rx_flags, rx_frame_error_shift	 ;replaced: QBBC	LB_NO_RX_FRAME_ERROR, MII_RCV.rx_flags.rx_frame_error
    ;clear RX_BC_FRAME & RX_MC_FRAME flags
    CLR	R22 , R22 , RX_BC_FRAME
    CLR	R22 , R22 , RX_MC_FRAME
    JMP	LB_RELEASE_QUEUE   ; Clear the EOF and other possible error flags.

LB_NO_RX_FRAME_ERROR:

    .if $defined("ICSS_SWITCH_BUILD")
    .if $defined(PTP)
    ;This code is used to control PTP packet forwarding from driver, if mem location
    ;is set to 1 then FW skips the flow. By default (0) flow is taken
    LBCO    &RCV_TEMP_REG_2.b0, ICSS_SHARED_CONST, DISABLE_PTP_FRAME_FORWARDING_CTRL_OFFSET, 1
    QBEQ    PTP_PORT_QUEUE_RELEASE_DONE, RCV_TEMP_REG_2.b0, 1
    QBBS    LB_RELEASE_PORT_QUEUE, R22, PTP_RELEASE_PORT_QUEUE_BIT

PTP_PORT_QUEUE_RELEASE_DONE:

    .endif ;PTP
    .endif ;ICSS_SWITCH_BUILD

    LDI	R0.b0, 0
    .if $defined("PRU0")
    XOUT	BANK1, &MII_RCV, $sizeof(MII_RCV)	; store task parameters from parameter bank
    .else
    XOUT	BANK2, &MII_RCV, $sizeof(MII_RCV)	; store task parameters from parameter bank
    .endif

    .if    $defined("TWO_PORT_CFG")

    .if    $defined("PRU0")
    LDI        R0.b0, SHIFT_R14_TO_R0
    XOUT    BANK0, &MII_RCV_PORT, $sizeof(MII_RCV_PORT)        ; store task parameters from parameter bank
    .else
    LDI        R0.b0, SHIFT_R14_TO_R4
    XOUT    BANK0, &MII_RCV_PORT, $sizeof(MII_RCV_PORT)
    .endif
    ; For Host Receive
    QBBC    LB_UPDATE_CHECK_FWD_FLAG, MII_RCV.rx_flags, host_rcv_flag_shift    ;MII_RCV.rx_flags.host_rcv_flag
    .endif ;TWO_PORT_CFG
    LDI	R0.b0, SHIFT_R2_TO_R26
    .if $defined("PRU0")
    XIN	BANK1, &RCV_CONTEXT, $sizeof(MII_RCV_PORT)	; store task parameters from parameter bank
    .else
    XIN	BANK2, &RCV_CONTEXT, $sizeof(MII_RCV_PORT)	; store task parameters from parameter bank
    .endif
    .if    $defined("TWO_PORT_CFG")
    CLR	R6 , R6 , 0
    QBA        LB_UPDATE_FOR_HOST_RECEIVE
LB_UPDATE_CHECK_FWD_FLAG:

    QBBC    LB_UPDATE_CHECK_DONE, MII_RCV.rx_flags, fwd_flag_shift    ;MII_RCV.rx_flags.fwd_flag
    .if    $defined("PRU0")
    LDI        R0.b0, SHIFT_R2_TO_R0
    XIN        BANK0, &RCV_CONTEXT, $sizeof(MII_RCV_PORT)        ; store task parameters from parameter bank
    .else
    LDI        R0.b0, SHIFT_R2_TO_R4
    XIN        BANK0, &RCV_CONTEXT, $sizeof(MII_RCV_PORT)        ; store task parameters from parameter bank
    .endif
    SET	R6 , R6 , 0
    .else
    ;store the length of the packet for statistics
    LDI	RCV_TEMP_REG_2 , RX_PKT_SIZE_OFFSET
    SBCO	&RCV_CONTEXT.byte_cntr, PRU_DMEM_ADDR, RCV_TEMP_REG_2, 2        ; load the byte count in working register

    .endif ;TWO_PORT_CFG
LB_UPDATE_FOR_HOST_RECEIVE:
    ; Update the Receive Buffer Descriptor
    ; Read the wr_ptr of first buffer descriptor
    .if    $defined("TWO_PORT_CFG")
    LBCO    &RCV_TEMP_REG_1, PRU1_DMEM_CONST, RCV_CONTEXT.rcv_queue_pointer, 4
    .else
    LBCO	&RCV_TEMP_REG_1, ICSS_SHARED_CONST, RCV_CONTEXT.rcv_queue_pointer, 4
    .endif ;TWO_PORT_CFG
    ; clear length field (18..28) and update length with current received frame
    LDI	RCV_TEMP_REG_2.w0, 0
    SUB	   RCV_TEMP_REG_2.w2, RCV_CONTEXT.byte_cntr, 4	    ;4 byte of FCS
    LSL	RCV_TEMP_REG_2.w2, RCV_TEMP_REG_2.w2, 2

        ;Set the Port number on which packet was received
    .if $defined("PRU0")
    SET	RCV_TEMP_REG_2 , RCV_TEMP_REG_2 , 16 ; Port 1
    .else
    SET	RCV_TEMP_REG_2 , RCV_TEMP_REG_2 , 17 ;Port 2
    .endif

    CLR     RCV_TEMP_REG_2, RCV_TEMP_REG_2, 15       ;Clear PTP descriptor bit

    .if $defined(PTP)  ;check if bit 15 needs to be set
        QBBC   LB_SKIP_PTP_DESC_BIT_SET, R22, RX_IS_PTP_BIT
        CLR    R22, R22, RX_IS_PTP_BIT
        SET	   RCV_TEMP_REG_2 , RCV_TEMP_REG_2 , 15  ;Indicate to the driver that this is a PTP frame
LB_SKIP_PTP_DESC_BIT_SET:
    .endif ;PTP

    .if $defined("ICSS_STP_SWITCH")
    ; Set FDB Lookup Success bit if the FDB lookup was successful
    QBBC  FDB_LOOKUP_FAIL, R22, FDB_LOOKUP_SUCCESS__R22_BIT
    SET   RCV_TEMP_REG_2, RCV_TEMP_REG_2, FDB_LOOKUP_SUCCESS__BD_BIT
FDB_LOOKUP_FAIL:

    ; Set Packet Flooded bit if the packet was indeed flooded
    QBBC  PKT_NOT_FLOODED, R22, PKT_FLOODED__R22_BIT
    SET   RCV_TEMP_REG_2, RCV_TEMP_REG_2, PKT_FLOODED__BD_BIT
PKT_NOT_FLOODED:
    .endif ; ICSS_STP_SWITCH

    ; the first buffer descriptor of the frame has been updated with length and port information
    SBCO	&RCV_TEMP_REG_2, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w2, 4

    ;Update the queue max fill level
    QBGT 	LB_RCV_QUEUE_WRAP, RCV_CONTEXT.wrkng_wr_ptr, RCV_CONTEXT.rd_ptr
    SUB	RCV_TEMP_REG_2.w0, RCV_CONTEXT.wrkng_wr_ptr, RCV_CONTEXT.rd_ptr
    JMP     SKIP_LB_RCV_QUEUE_WRAP

LB_RCV_QUEUE_WRAP:
    ; add queue size to rd_ptr and then subtract wr_ptr
    SUB	RCV_TEMP_REG_2.w0, RCV_CONTEXT.top_most_buffer_desc_offset, RCV_CONTEXT.rd_ptr
    ADD	RCV_TEMP_REG_2.w0, RCV_TEMP_REG_2.w0, 4
    SUB	RCV_TEMP_REG_2.w2, RCV_CONTEXT.wrkng_wr_ptr, RCV_CONTEXT.base_buffer_desc_offset
    ADD	RCV_TEMP_REG_2.w0, RCV_TEMP_REG_2.w0, RCV_TEMP_REG_2.w2

SKIP_LB_RCV_QUEUE_WRAP:
    ;divide the queue fill level by 4
    LSR	RCV_TEMP_REG_2.w0, RCV_TEMP_REG_2.w0, 2
    ;Read the queue max fill level
    ADD	RCV_TEMP_REG_1.w0, RCV_CONTEXT.rcv_queue_pointer, Q_MAX_FILL_LEVEL_OFFSET
    .if    $defined("TWO_PORT_CFG")
    LBCO    &RCV_TEMP_REG_2.b2, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 1    ;RCV_TEMP_REG_2.w2.b0
    .else
    LBCO	&RCV_TEMP_REG_2.b2, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, 1

    .endif ;TWO_PORT_CFG
    ;compare the new queue fill level with max fill level
    QBGE	LB_SKIP_NEW_FILL_LEVEL, RCV_TEMP_REG_2.b0, RCV_TEMP_REG_2.b2
    AND RCV_TEMP_REG_2.b2 , RCV_TEMP_REG_2.b0 , RCV_TEMP_REG_2.b0
    ; store the new max fill level
    .if    $defined("TWO_PORT_CFG")
    SBCO    &RCV_TEMP_REG_2.b2, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 1     ;RCV_TEMP_REG_2.w2.b0
    .else
    SBCO	&RCV_TEMP_REG_2.b2, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, 1

    .endif ;TWO_PORT_CFG
LB_SKIP_NEW_FILL_LEVEL:
LB_HOST_PORT_QUEUE_OVERFLOW_CHECK:
LB_HOST_QUEUE_OVERFLOW:

    ADD	RCV_TEMP_REG_1.w0, RCV_CONTEXT.rcv_queue_pointer, Q_OVERFLOW_CNT_OFFSET
    QBBS	LB_PORT_QUEUE_OVERFLOW_CHECK, R6, 0	 ;replaced: QBBS	LB_PORT_QUEUE_OVERFLOW_CHECK, CODE_EXECUTING_FOR_PORT_RECEIVE ; 0 -> host recieve | 1 -> port recieve
    QBBC	LB_HOST_PORT_QUEUE_OVERFLOW_CHECK_DONE, MII_RCV.rx_flags_extended, 6	 ;replaced: QBBC	LB_HOST_PORT_QUEUE_OVERFLOW_CHECK_DONE, MII_RCV.rx_flags_extended.host_queue_overflow
    QBA 	HOST_QUEUE_OVERFLOW_STATS

LB_PORT_QUEUE_OVERFLOW_CHECK:
    QBBC	LB_HOST_PORT_QUEUE_OVERFLOW_CHECK_DONE, MII_RCV.rx_flags_extended, 7	 ;replaced: QBBC	LB_HOST_PORT_QUEUE_OVERFLOW_CHECK_DONE, MII_RCV.rx_flags_extended.port_queue_overflow

HOST_QUEUE_OVERFLOW_STATS:
PORT_QUEUE_OVERFLOW_STATS:

    .if    $defined("TWO_PORT_CFG")
    LBCO	&RCV_TEMP_REG_2.b2, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 1
    .else
    LBCO	&RCV_TEMP_REG_2.b2, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, 1
    .endif
    ADD	RCV_TEMP_REG_2.b2, RCV_TEMP_REG_2.b2, 1
    .if    $defined("TWO_PORT_CFG")
    SBCO	&RCV_TEMP_REG_2.b2, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 1
    .else
    SBCO	&RCV_TEMP_REG_2.b2, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, 1
    .endif
LB_HOST_PORT_QUEUE_OVERFLOW_CHECK_DONE:
    ; Update the Queue Descriptor with the new wr_ptr
    ADD	RCV_TEMP_REG_1.w0, RCV_CONTEXT.rcv_queue_pointer, Q_RD_PTR_SIZE
    .if    $defined("TWO_PORT_CFG")
    SBCO    &RCV_CONTEXT.wrkng_wr_ptr, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, Q_WR_PTR_SIZE
    .else
    SBCO	&RCV_CONTEXT.wrkng_wr_ptr, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, Q_WR_PTR_SIZE
    .endif ;TWO_PORT_CFG
    .if $defined("ICSS_SWITCH_BUILD")
    .if    $defined("TWO_PORT_CFG")
    QBBS    LB_CHECK_COLLISION_FOR_PORT, R6, 0    ;CODE_EXECUTING_FOR_PORT_RECEIVE
    ; Update the collision status register with info required by the collision task.
    QBBC    LB_UPDATE_CHECK_FWD_FLAG, MII_RCV.rx_flags, host_collision_queue_selected_shift    ;MII_RCV.rx_flags.host_collision_queue_selected
    QBA        LB_UPDATE_COLLISION_STATUS
    .endif ;TWO_PORT_CFG
LB_CHECK_COLLISION_FOR_PORT:
    .if    $defined("TWO_PORT_CFG")
    ; Update the collision status register with info required by the collision task.
    QBBC    LB_NO_COLLISIION_OCCURED, MII_RCV.rx_flags, port_collision_queue_selected_shift    ;MII_RCV.rx_flags.port_collision_queue_selected
LB_UPDATE_COLLISION_STATUS:
    LDI     RCV_TEMP_REG_1.w0, COLLISION_STATUS_ADDR
    AND COLLISION_STATUS_REG.b2 , MII_RCV.qos_queue , MII_RCV.qos_queue	;Warning: converted from MOV
    LSL        COLLISION_STATUS_REG.b2, COLLISION_STATUS_REG.b2, 1
    OR        COLLISION_STATUS_REG.b2, COLLISION_STATUS_REG.b2, 0x01
    QBBC    LB_COLLISION_SATUS_FOR_HOST_QUEUE, R6, 0    ;CODE_EXECUTING_FOR_PORT_RECEIVE
    .if    $defined("PRU0")
    ADD        RCV_TEMP_REG_1.w0, RCV_TEMP_REG_1.w0, 2
    .else
    ADD        RCV_TEMP_REG_1.w0, RCV_TEMP_REG_1.w0, 1
    .endif

    CLR        MII_RCV.rx_flags, MII_RCV.rx_flags, port_collision_queue_selected_shift   ; Clear the collision bit for port
LB_COLLISION_SATUS_FOR_HOST_QUEUE:
    SBCO     &COLLISION_STATUS_REG.b2, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 1

    QBBC    LB_UPDATE_CHECK_FWD_FLAG, R6, 0    ;CODE_EXECUTING_FOR_PORT_RECEIVE
    .endif ;TWO_PORT_CFG
LB_NO_COLLISIION_OCCURED:
LB_UPDATE_CHECK_DONE:
    .endif  ;ICSS_SWITCH_BUILD
LB_RELEASE_QUEUE:
    .if $defined("ICSS_SWITCH_BUILD")
    QBBS    LB_RELEASE_HOST_QUEUE, MII_RCV.rx_flags_extended, host_queue_overflow_shift    ;MII_RCV.rx_flags_extended.host_queue_overflow
    QBBC    LB_RELEASE_QUEUE_CHECK_FWD_FLAG, MII_RCV.rx_flags, host_rcv_flag_shift    ;MII_RCV.rx_flags.host_rcv_flag
    .endif  ;ICSS_SWITCH_BUILD
LB_RELEASE_HOST_QUEUE:
    ;Release the Receive Queue. De-assert the Queue busy bit
    ; PRU0 is master so it clear's only "busy_m" bit
    .if $defined("PRU0")
    ADD	RCV_TEMP_REG_1.w0, MII_RCV.rcv_queue_pointer, Q_STATUS_OFFSET
    .if    $defined("TWO_PORT_CFG")
    LBCO    &RCV_TEMP_REG_2, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 3
    .else
    LBCO	&RCV_TEMP_REG_2, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, 3

    .endif ;TWO_PORT_CFG
    AND	RCV_TEMP_REG_2.b0, RCV_TEMP_REG_2.b0, 0xFC	; !(1<<Q_BUSY_M_BIT | 1<<Q_COLLISION_BIT)
    QBBC	LB_NO_HOST_QUEUE_OVERFLOW_OCCURED, MII_RCV.rx_flags_extended, host_queue_overflow_shift	 ;replaced: QBBC	LB_NO_HOST_QUEUE_OVERFLOW_OCCURED, MII_RCV.rx_flags_extended.host_queue_overflow
    OR	RCV_TEMP_REG_2.b0, RCV_TEMP_REG_2.b0, 0x04	; set the overflow bit
    ADD	RCV_TEMP_REG_2.b2, RCV_TEMP_REG_2.b2, 1	; increment the overflow count by 1
LB_NO_HOST_QUEUE_OVERFLOW_OCCURED:
    .if    $defined("TWO_PORT_CFG")
    SBCO    &RCV_TEMP_REG_2.b0, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 3
    .else
    SBCO	&RCV_TEMP_REG_2.b0, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, 3
    .endif ;TWO_PORT_CFG

    .else
    ; PRU1 is slave so it clear's only "busy_s" bit
    ADD	RCV_TEMP_REG_1.w0, MII_RCV.rcv_queue_pointer, Q_BUSY_S_OFFSET
    .if    $defined("TWO_PORT_CFG")
    LBCO    &RCV_TEMP_REG_2, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 4
    .else
    LBCO	&RCV_TEMP_REG_2, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, 4
    .endif ;TWO_PORT_CFG
    LDI	RCV_TEMP_REG_2.b0, 0
    QBBC	LB_NO_HOST_QUEUE_OVERFLOW_OCCURED_SLAVE, MII_RCV.rx_flags_extended, host_queue_overflow_shift	 ;replaced: QBBC	LB_NO_HOST_QUEUE_OVERFLOW_OCCURED_SLAVE, MII_RCV.rx_flags_extended.host_queue_overflow
    OR	RCV_TEMP_REG_2.b1, RCV_TEMP_REG_2.b1, 0x04	; set the overflow bit
    ADD	RCV_TEMP_REG_2.b3, RCV_TEMP_REG_2.b3, 1	; increment the overflow count by 1
LB_NO_HOST_QUEUE_OVERFLOW_OCCURED_SLAVE:
    .if    $defined("TWO_PORT_CFG")
    SBCO    &RCV_TEMP_REG_2, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 4
    .else
    SBCO	&RCV_TEMP_REG_2, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, 4
    .endif ;TWO_PORT_CFG
    .endif

LB_RELEASE_QUEUE_CHECK_FWD_FLAG:
    .if $defined("ICSS_SWITCH_BUILD")
    QBBS    LB_RELEASE_PORT_QUEUE, MII_RCV.rx_flags_extended, port_queue_overflow_shift    ;MII_RCV.rx_flags_extended.port_queue_overflow
    QBBC    LB_RELEASE_QUEUE_CHECK_DONE, MII_RCV.rx_flags, fwd_flag_shift    ;MII_RCV.rx_flags.fwd_flag

LB_RELEASE_PORT_QUEUE:
    ;Release the Port Receive Queue. clear busy_m bit in status as PRU is master

    ADD        RCV_TEMP_REG_1.w0, MII_RCV_PORT.rcv_queue_pointer, Q_STATUS_OFFSET
    .if    $defined("TWO_PORT_CFG")
    LBCO    &RCV_TEMP_REG_2, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 3
    .else
    LBCO    &RCV_TEMP_REG_2, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, 3
    .endif ;TWO_PORT_CFG
    AND        RCV_TEMP_REG_2.b0, RCV_TEMP_REG_2.b0, 0xFC  ; !(1<<Q_BUSY_M_BIT | 1<<Q_COLLISION_BIT)
    QBBC    LB_NO_PORT_QUEUE_OVERFLOW_OCCURED, MII_RCV.rx_flags_extended, port_queue_overflow_shift    ;MII_RCV.rx_flags_extended.port_queue_overflow
    OR        RCV_TEMP_REG_2.b0, RCV_TEMP_REG_2.b0, 0x04   ; set the overflow bit
    ADD        RCV_TEMP_REG_2.b2, RCV_TEMP_REG_2.b2, 1     ; increment the overflow count by 1
LB_NO_PORT_QUEUE_OVERFLOW_OCCURED:
    .if    $defined("TWO_PORT_CFG")
    SBCO    &RCV_TEMP_REG_2.b0, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w0, 3
    .else
    SBCO    &RCV_TEMP_REG_2.b0, ICSS_SHARED_CONST, RCV_TEMP_REG_1.w0, 3
    .endif ;TWO_PORT_CFG
    .if $defined(PTP)
    QBBC    LB_RELEASE_QUEUE_CHECK_DONE, R22, PTP_RELEASE_PORT_QUEUE_BIT
    CLR     R22, R22, PTP_RELEASE_PORT_QUEUE_BIT
    CLR     MII_RCV.rx_flags, MII_RCV.rx_flags, fwd_flag_shift
    QBA     PTP_PORT_QUEUE_RELEASE_DONE
    .endif
LB_RELEASE_QUEUE_CHECK_DONE:
LB_MAINTENANCE:
    .endif ;ICSS_SWITCH_BUILD

    CLR	R23 , R23 , Rcv_active ; indicate that rcv has been completed
    QBBS	LB_NO_RX_STAT, MII_RCV.rx_flags, rx_frame_error_shift	 ;replaced: QBBS	LB_NO_RX_STAT, MII_RCV.rx_flags.rx_frame_error
    SET	R23 , R23 , RX_STAT_PEND

    .if $defined("ICSS_SWITCH_BUILD")
    QBBC    LB_DONE, MII_RCV.rx_flags, host_rcv_flag_shift    ;MII_RCV.rx_flags.host_rcv_flag
    .if    $defined("TWO_PORT_CFG")
    QBBS    LB_DONE, MII_RCV.rx_flags, host_collision_queue_selected_shift        ;MII_RCV.rx_flags.host_collision_queue_selected
    .endif ;TWO_PORT_CFG
    .endif ;ICSS_SWITCH_BUILD

    M_RCV_RX_EOF_INTERRUPT_RAISE_INTC           ; raise RX interrupt on intc controller.

    .if $defined("ICSS_SWITCH_BUILD")
LB_DONE:
    .if    $defined("TWO_PORT_CFG")
    CLR        MII_RCV.rx_flags, MII_RCV.rx_flags,  host_collision_queue_selected_shift    ;MII_RCV.rx_flags.host_collision_queue_selected
    .endif ;TWO_PORT_CFG
    QBBC    DO_NOT_SET_RX_FWD, MII_RCV.rx_flags, fwd_flag_shift    ;MII_RCV.rx_flags.fwd_flag
    .if    $defined("TWO_PORT_CFG")
    SET     R22, R22, RX_FWD_FLAG
    .endif ;TWO_PORT_CFG

DO_NOT_SET_RX_FWD:
    .endif ;ICSS_SWITCH_BUILD

LB_NO_RX_STAT:

    .if $defined("ICSS_REV1")
    M_RCV_RX_EOF_CLEAR_INTC_ICSS_REV1
    .endif	;ICSS_REV1

    .if $defined("ICSS_REV2")
    M_RCV_RX_EOF_CLEAR_INTC_ICSS_REV2
    .endif	;ICSS_REV2

    ; Execute the TX Task as next task
    QBBC	TASK_EXECUTION_FINISHED_inter, R23, 0	 ;replaced: QBBC	TASK_EXECUTION_FINISHED_inter, Xmt_active
    LDI	CURRENT_TASK_POINTER, BG_TASK_POINTER
    JMP		XMT_QUEUE

TASK_EXECUTION_FINISHED_inter:
    JMP	TASK_EXECUTION_FINISHED

    .if    $defined("TWO_PORT_CFG")
;/////////////////////////////// Experimenting Start /////////////////////////////////////////////////////

;-------------------------------------------------------------------------------------------
; Function:             F_QUEUE_ARBITRATION
; Description:         Function will aquire token for either queue or collision buffer
; Input:    MII_RCV.qos_queue - contains the Qos queue number as been determined by the RX_FB function
; Input:     RCV_TEMP_REG_3.w0 - contains the base address of host/port queue
; Input:     RCV_TEMP_REG_3.w2 - contains the base address of the collision queue
; Input:   RCV_TEMP_REG_1.b1 - contains information about whether called for host or port queue arbitration
; Output:    RCV_TEMP_REG_1.b0 - returns value for caller, to indicate that the queue has been aquired
;                                successful ((0-failed to aquire; 1-queue; 2-collision)
; Output:    RCV_TEMP_REG_2.w0 - returns the queue pointer after aquiring the queue
; Output:    RCV_TEMP_REG_2.w2 - returns the current write pointer of the queue that has been aquired
;                                is used as wrkng_wr_ptr during the following packet processing
;            RCV_TEMP_REG_1.w2 - temporayr use
;-------------------------------------------------------------------------------------------

FN_QUEUE_ARBITRATION:
    LDI        RCV_TEMP_REG_1.b0, 0x00
QAM_CHECK:
    LSL        RCV_TEMP_REG_1.w2, MII_RCV.qos_queue, 3        ; adjust queue-number offset to 8byte boundaries
    QBNE    PICK_NORMAL_QUEUE, RCV_TEMP_REG_1.b0, COLLISION_AQUIRED
    LDI        RCV_TEMP_REG_1.w2, 0x0
PICK_NORMAL_QUEUE:
    ADD        RCV_TEMP_REG_1.w2, RCV_TEMP_REG_3.w0, RCV_TEMP_REG_1.w2    ; adjust for queue base address
    ADD        RCV_TEMP_REG_3.w0, RCV_TEMP_REG_1.w2, Q_BUSY_S_OFFSET    ; .w2 points to Q_BUSY_S_OFFSET
    ADD        RCV_TEMP_REG_1.w2, RCV_TEMP_REG_1.w2, Q_STATUS_OFFSET    ; .w0 points to Q_STATUS_OFFSET
    LBCO    &RCV_TEMP_REG_2.w2, PRU1_DMEM_CONST, RCV_TEMP_REG_3.w0, Q_BUSY_S_SIZE + Q_STATUS_SIZE ; load busy_s + status of this queue
    ; Check if it is called for the Port Arbitration
    QBBS    PORT_QUEUE_ARBITRATION, RCV_TEMP_REG_1.b1, 0    ;RCV_TEMP_REG_1.b1.t0
    .if    $defined("PRU0")
    QBBS    QAM_BUSY, RCV_TEMP_REG_2.b2, Q_BUSY_S_BIT    ; is queue already busy?  PRU0 is master    ;RCV_TEMP_REG_2.w2.b0
    SET        RCV_TEMP_REG_2.b3, RCV_TEMP_REG_2.b3, Q_BUSY_M_BIT    ;RCV_TEMP_REG_2.w2.b1
    SBCO    &RCV_TEMP_REG_2.b3, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w2, Q_STATUS_SIZE    ; set busy_m into memory
    QBA        QAM_CALC_QUEUE_POINTER
    .else
    QBBS    QAM_BUSY, RCV_TEMP_REG_2.b3, Q_BUSY_M_BIT    ; is queue already busy? PRU1 is slave    ;RCV_TEMP_REG_2.w2.b1
    SET        RCV_TEMP_REG_2.b2, RCV_TEMP_REG_2.b2, Q_BUSY_S_BIT    ;RCV_TEMP_REG_2.w2.b0
    SBCO    &RCV_TEMP_REG_2.b2, PRU1_DMEM_CONST, RCV_TEMP_REG_3.w0, Q_BUSY_S_SIZE     ; set busy_s into memory
    ; load again busy_m ..only for Slave because it needs to check that master has not acquired the queue inbetween
    LBCO    &RCV_TEMP_REG_2.b3, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w2, Q_STATUS_SIZE    ;RCV_TEMP_REG_2.w2.b1
    QBBC    QAM_CALC_QUEUE_POINTER, RCV_TEMP_REG_2.b3, Q_BUSY_M_BIT    ; is queue already busy?
    ; Clear busy_s bit because busy_m bit is set
    CLR        RCV_TEMP_REG_2.b2, RCV_TEMP_REG_2.b2, Q_BUSY_S_BIT    ;RCV_TEMP_REG_2.w2.b0
    SBCO    &RCV_TEMP_REG_2.b2, PRU1_DMEM_CONST, RCV_TEMP_REG_3.w0, Q_BUSY_S_SIZE     ; clear busy_s into memory    ;RCV_TEMP_REG_2.w2.b0
    QBA        QAM_BUSY
    .endif

PORT_QUEUE_ARBITRATION:
    QBBS    QAM_BUSY, RCV_TEMP_REG_2.b2, Q_BUSY_S_BIT    ; is queue already busy?  PRU0 is master    ;RCV_TEMP_REG_2.w2.b0
    SET        RCV_TEMP_REG_2.b3, RCV_TEMP_REG_2.b3, Q_BUSY_M_BIT    ;RCV_TEMP_REG_2.w2.b1
    SBCO    &RCV_TEMP_REG_2.b3, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w2, Q_STATUS_SIZE    ; set busy_m into memory
    QBA        QAM_CALC_QUEUE_POINTER

QAM_BUSY: ; failed to aquire the queue, now try the colission
    QBEQ    QAM_FAILED, RCV_TEMP_REG_1.b0, COLLISION_AQUIRED
    AND RCV_TEMP_REG_3.w0 , RCV_TEMP_REG_3.w2 , RCV_TEMP_REG_3.w2
    LDI        RCV_TEMP_REG_1.b0, COLLISION_AQUIRED        ; failed to aquire the queue, indicate this to macro caller by returing 0
    ;check that host has emptied the queue before acquiring it
    LDI      RCV_TEMP_REG_1.w2, COLLISION_STATUS_ADDR
    ADD      RCV_TEMP_REG_1.w2, RCV_TEMP_REG_1.w2, 3
    LBCO     &RCV_TEMP_REG_1.b3, PRU1_DMEM_CONST, RCV_TEMP_REG_1.w2, 1
	QBBC	QAM_CHECK, RCV_TEMP_REG_1, 3	 ;replaced: QBBC    QAM_CHECK, RCV_TEMP_REG_1.HOST_COLLISION_READ_PENDING

QAM_FAILED:
    LDI        RCV_TEMP_REG_1.b0, QUEUE_FAILED    ; failed to aquire queue & collision
    LDI        RCV_TEMP_REG_2.w2, 0
    QBA        QAM_DONE
QAM_CALC_QUEUE_POINTER:
    QBEQ    PICK_COL_QUEUE, RCV_TEMP_REG_1.b0, COLLISION_AQUIRED
    ; return value that queue had been aquired OK.
    LDI        RCV_TEMP_REG_1.b0, QUEUE_AQUIRED
PICK_COL_QUEUE:
    ; adjust the pointer to the wr_ptr offset
    SUB        RCV_TEMP_REG_2.w0, RCV_TEMP_REG_1.w2, Q_STATUS_OFFSET - Q_WR_PTR_OFFSET
    ; load wr_ptr from memory, and us it to init the wrkng_wr_ptr
    LBCO    &RCV_TEMP_REG_2.w2, PRU1_DMEM_CONST, RCV_TEMP_REG_2.w0, Q_WR_PTR_SIZE
    ; ToDo: this can be optimized!
    SUB        RCV_TEMP_REG_2.w0, RCV_TEMP_REG_2.w0, Q_WR_PTR_OFFSET
QAM_DONE:
    JMP	CALL_REG

;////////////////////////////// Experimenting End ////////////////////////////////////////////////////////////////////
    .endif ;TWO_PORT_CFG
    .endif  ; e/o MII_Rcv_p

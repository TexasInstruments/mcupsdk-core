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
; file:   icss_ptp.h
;
; brief:  Definitions and mapping of PTP over PRU
;         Includes:
;         1. PTP structures
;           2. PTP variables
;
;
;  (C) Copyright 2012-2018, Texas Instruments, Inc
;
;

    .if    !$isdefed("__ICSS_PTP_H__")
__ICSS_PTP_H__    .set    1

    ;This corresponds to 1 second
IEP_WRAP_AROUND_VAL                 .set        0x3B9ACA00
IEP_COMPENSATION_FIRST_ADJUST       .set        0x1720      ;corresponds to 64 bytes rcv

;======================================GPTP DEFINES========================================================;
PTP_SYNC_MSG_ID                .set                        0x00
PTP_DLY_REQ_MSG_ID             .set                        0x01
PTP_PDLY_REQ_MSG_ID            .set                        0x02
PTP_PDLY_RSP_MSG_ID            .set                        0x03
PTP_FOLLOW_UP_MSG_ID           .set                        0x08
PTP_FOLLOW_UP_MSG_ID_CTRL      .set                        0x02
PTP_DLY_RESP_MSG_ID            .set                        0x09
PTP_PDLY_RESP_FLW_UP_MSG_ID    .set                        0x0A
PTP_ANNOUNCE_MSG_ID            .set                        0x0B
PTP_MGMT_MSG_ID                .set                        0x0D

SEC_TO_NS                      .set                        0x3B9ACA00

PTP_HSR_PRP_NON_LL_MAC_ID_L    .set                        0x0000
PTP_HSR_PRP_NON_LL_MAC_ID_H    .set                        0x00191b01
PTP_HSR_PRP_LL_MAC_ID_L        .set                        0x0e00
PTP_HSR_PRP_LL_MAC_ID_H        .set                        0x00c28001

PTP_E2E_UDP_MAC_ID_H           .set                        0x005E0001
PTP_E2E_UDP_MAC_ID_L           .set                        0x8101
PTP_E2E_UDP_PDELAY_MAC_ID_L    .set                        0x6B00

GPTP_EtherType                 .set                        0xf788
LLDP_EtherType                 .set                        0xcc88
HSR_EtherType                  .set                        0x2f89
VLAN_EtherType                 .set                        0x0081
IPV4_EtherType                 .set                        0x0008
GPTP_messageType_Sync          .set                        0x0210
GPTP_messageType_Pdelay_Req    .set                        0x0212
GPTP_messageType_Pdelay_Resp   .set                        0x0213
GPTP_QOS                       .set                        0
UDP_PROTOCOL_TYPE              .set                        0x11
UDP_PTP_PORT_319               .set                        0x3F01
UDP_PTP_PORT_320               .set                        0x4001


GPTP_802_3_two_step_bit        .set                        1
        .asg    R7,       two_step_reg
        .asg    R5.b2,    PTP_802_3_msg_id_reg
        .asg    R8,       two_step_reg_vlan
        .asg    R6,       two_step_reg_udp
        .asg    R4.b2,    PTP_MSG_ID_REG_UDP
        .asg    R7,       two_step_reg_udp_vlan
        .asg    R5.b2,    PTP_MSG_ID_REG_UDP_VLAN
        .asg    R7.b3,    IP_PROT_REG
        .asg    R8.b3,    IP_PROT_VLAN_REG
        .asg    R2.w2,    UDP_SRC_PORT_REG
        .asg    R3.w2,    UDP_SRC_PORT_VLAN_REG

;-----------R22 flags------------
    ;This flag is used to indicate a PTP frame on Rx
RX_IS_PTP_BIT  .set    14
    ;This flag is used to indicate a PTP frame on Tx
TX_IS_PTP_BIT  .set    13
    ;This is used for internal book-keeping to generate 1PPS sync0 pulse
CHECK_SYNC0_BIT .set   12
    ;Set to indicate that a Tx callback interrupt is pending
    ;On successful completion of transmit the interrupt is set
TX_CALLBACK_INTERRUPT_BIT .set   11
    ;This is set when host rcv flag is cleared in the middle
    ;Firmware checks this and releases host queue without
    ;updating the write pointer
    ;Analogous to the one above but for port forwarding queues
PTP_RELEASE_PORT_QUEUE_BIT .set   9

    ; Set if RX frame has VLAN tag
RX_IS_VLAN_BIT .set 8
    ; Set if TX frame has VLAN tag
TX_IS_VLAN_BIT .set 7
    ;This flag is used to indicate a UDP PTP frame on Rx
RX_IS_UDP_PTP_BIT .set   6
    ;This flag is used to indicate a UDP PTP frame on Tx
TX_IS_UDP_PTP_BIT  .set   3


GPTP_NUM_DOMAINS                       .set                2
GPTP_ENABLE_BG_TASK_BIT                .set                0

    .endif ;__ICSS_PTP_H__


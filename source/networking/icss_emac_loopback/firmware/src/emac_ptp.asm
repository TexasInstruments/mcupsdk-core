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
;  manufactured by or for TI ("TI Devices").  No hardware patent is licensed hereunder.
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
;  THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED
;  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
;  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI'S
;  LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
;  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
;  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
; file:   emac_ptp.asm
;
; brief:  ICSS EMAC precision time protocol

    .if !$defined(____pn_gPtp_asm)
____pn_gPtp_asm    .set    1
    .if $defined("ICSS_DUAL_EMAC_BUILD")
    .include "icss_emacSwitch.h"
    .endif
    .if $defined("ICSS_SWITCH_BUILD")
        .include "icss_switch.h"
    .endif

    .include "icss_defines.h"
    .include "emac_MII_Xmt.h"
    .include "emac_MII_Rcv.h"
    .include "icss_ptp.h"
    .include "micro_scheduler.h"
    .include "icss_iep_regs.h"
    .include "icss_miirt_regs.h"
    .include "icss_ptp_macro.h"
    .cdecls C,NOLIST
%{
#include "icss_timeSync_memory_map.h"
%}

    .global  FN_PTP_BACKGROUND_TASK
    .global  FN_TIMESTAMP_GPTP_PACKET
    .global  FN_CHECK_AND_CLR_PTP_FWD_FLAG
    .global  FN_CHECK_AND_CLR_PTP_FWD_FLAG_L2
    .global  FN_PTP_TX_ADD_DELAY
    .global  FN_PTP_TX_ADD_DELAY_UDP
    .global  FN_COMPARE_DELAY_RESP_ID

    .if $defined (ICSS_REV1)

;****************************************************************************
;
;     NAME             : FN_TIMESTAMP_GPTP_PACKET
;     DESCRIPTION      : Store timestamp for PTP frames.
;                        It is called during Rx
;     Cycles           : 34
;     Register Usage   : R22 (flags), R20, R21, R10, R11, R13
;     Pseudocode       :
;if PTP frame:
;   if not link local
;       if sync frame:
;           if PTP not enabled:
;               discard frame
;           else:
;               if not from master:
;                   do not forward to host
;               else:
;                   if first sync:
;                       update the master port
;                   take timestamp()
;       elif follow up
;           if PTP not enabled:
;               discard frame
;           else:
;               if not from master:
;                   do not forward to host
;               else:
;                   clear fwd_flag
;                   exit without timestamp
;
;   else:
;       check if hsr tag is there and load appropriate offset
;       if pdelay req frame or pdelay response frame:
;           take timestamp()
;def take_timestamp()
;   load SFD timestamp
;   exit
;else:
;   exit
;
;***************************************************************************

FN_TIMESTAMP_GPTP_PACKET:

    MOV     R11.w2, R13.w2
    QBBC    FN_TIMESTAMP_GPTP_PACKET_EXIT, R22, RX_IS_PTP_BIT


    .if $defined (PRU0)
        LDI     R11.w0, RX_TIMESTAMP_OFFSET_P1
    .else
        LDI     R11.w0, RX_TIMESTAMP_OFFSET_P2
    .endif

GPTP_STORE_TIMESTAMP:

    ;This also loads 2 bytes in RCV_TEMP_REG_2.w0
    LBCO    &RCV_TEMP_REG_1, ICSS_SHARED_CONST, TIMESYNC_SECONDS_COUNT_OFFSET, 6
    .if $defined (PRU0)
    LBCO    &R13, IEP_CONST, CAP_RISE_RX_SFD_PORT1_OFFSET, 4
    .else
    LBCO    &R13, IEP_CONST, CAP_RISE_RX_SFD_PORT2_OFFSET, 4
    .endif
    ;current IEP counter value
    LBCO    &R12, IEP_CONST, IEP_COUNTER_OFFSET, 4
    ;check if counter has been incremented
    LBCO    &R10.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
    ;check if there has been a wrap around
    QBGT    NO_WRAPAROUND_SINCE_RX, R13, R12
    ;make sure counter has not incremented
    QBBS    COMPENSATION_DONE_RX, R10, 0
    LBCO    &R10, ICSS_SHARED_CONST, TIMESYNC_IEP_VAL_CYCLE_COUNTER, 4
    QBGT    COMPENSATION_DONE_RX, R12, R10
    SUB     RCV_TEMP_REG_1, RCV_TEMP_REG_1, 1
    SUC     RCV_TEMP_REG_2.w0, RCV_TEMP_REG_2.w0, 0
    QBA     COMPENSATION_DONE_RX

NO_WRAPAROUND_SINCE_RX:

    ;make sure that counter has incremented
    QBBC    COMPENSATION_DONE_RX, R10, 0
    LBCO    &R10, ICSS_SHARED_CONST, TIMESYNC_IEP_VAL_CYCLE_COUNTER, 4
    QBGT    COMPENSATION_DONE_RX, R10, R12
    ADD     RCV_TEMP_REG_1, RCV_TEMP_REG_1, 1
    ADC     RCV_TEMP_REG_2.w0, RCV_TEMP_REG_2.w0, 0

COMPENSATION_DONE_RX:

    ;Do PHY compensation
    LBCO    &R12.w0, ICSS_SHARED_CONST, MII_RX_CORRECTION_OFFSET, 2
    QBLE    NO_WRAPAROUND_RX, R13, R12.w0
    ;The seconds timestamp should be decremented by 1
    SUB     RCV_TEMP_REG_1, RCV_TEMP_REG_1, 1
    SUC     RCV_TEMP_REG_2.w0, RCV_TEMP_REG_2.w0, 0
    LDI32     R10, IEP_WRAP_AROUND_VAL
    ADD     R13, R13, R10

NO_WRAPAROUND_RX:

    SUB     R13, R13, R12.w0

PHY_COMPENSATION_DONE:

    ;store nanoseconds TS
    SBCO    &R13, ICSS_SHARED_CONST, R11.w0, 4
    ADD     R11.w0, R11.w0, 4
    ;store seconds TS
    SBCO    &RCV_TEMP_REG_1, ICSS_SHARED_CONST, R11.w0, 6

FN_TIMESTAMP_GPTP_PACKET_EXIT:

    JMP     R11.w2

;***************************************************************************
;
;     NAME             : FN_GET_TX_TIMESTAMP
;     DESCRIPTION      : calculates the Tx timestamp in seconds and nanoseconds format
;                        It is called during Tx
;     Assumption       : This uses registers R2-R6 as temp registers, They also contain Tx
;                      : data which has been pushed to FIFO. Calling function must not use
;                        the data.
;     Cycles           :
;     Return           : Returns nanoseconds in R4, Seconds in R2 and R3.w0. Return address in R0.w2
;     Register Usage   :
;     Pseudocode       :
;
;
;***************************************************************************

FN_GET_TX_TIMESTAMP:

    ;we need to make sure that the timestamp here is actual value
    ;this we do by comparing with Tx SOF of previous frame
    .if $defined("ICSS_SWITCH_BUILD")
        .if $defined (PRU0)
        LDI    R5.w0, PTP_PREV_TX_TIMESTAMP_P2
        LBCO    &R5.w0, ICSS_SHARED_CONST, R5.w0, 4
        .else
        ; LDI    R5.w0, PTP_PREV_TX_TIMESTAMP_P1
        LBCO    &R5.w0, ICSS_SHARED_CONST, PTP_PREV_TX_TIMESTAMP_P1, 4
        .endif
        ;LBCO    &R5, ICSS_SHARED_CONST, R5.w0, 4
        ; get the timestamp
    .else
        .if $defined (PRU0)
        LDI    R5.w0, PTP_PREV_TX_TIMESTAMP_P1
        LBCO    &R5.w0, ICSS_SHARED_CONST, R5.w0, 4
        .else
        LDI    R5.w0, PTP_PREV_TX_TIMESTAMP_P2
        LBCO    &R5.w0, ICSS_SHARED_CONST, R5.w0, 4
        .endif
        ;LBCO    &R5, ICSS_SHARED_CONST, R5.w0, 4
        ; get the timestamp
    .endif ; switch build

LOAD_TX_SOF_TS:

    .if $defined("ICSS_SWITCH_BUILD")
    .if $defined (PRU0)
    LBCO    &R4, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 4
    .else
    LBCO    &R4, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 4
    .endif
    .else
    .if $defined (PRU0)
    LBCO    &R4, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 4
    .else
    LBCO    &R4, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 4
    .endif
    .endif

    QBEQ    LOAD_TX_SOF_TS, R4, R5

    ;This also loads 2 bytes in R3.w0
    LBCO    &R2, ICSS_SHARED_CONST, TIMESYNC_SECONDS_COUNT_OFFSET, 6

    ;current IEP counter value
    LBCO    &R5, IEP_CONST, IEP_COUNTER_OFFSET, 4
    ;check if counter has been incremented
    LBCO    &R11.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
    ;check if there has been a wrap around
    QBGT    NO_WRAPAROUND_SINCE_TX, R4, R5
    ;make sure counter has not incremented
    QBBS    COMPENSATION_DONE_TX, R11, 0
    LBCO    &R10, ICSS_SHARED_CONST, TIMESYNC_IEP_VAL_CYCLE_COUNTER, 4
    QBGT    COMPENSATION_DONE_TX, R5, R10
    SUB     R2, R2, 1
    SUC     R3.w0, R3.w0, 0
    QBA     COMPENSATION_DONE_TX

NO_WRAPAROUND_SINCE_TX:

    ;make sure that counter has incremented
    QBBC    COMPENSATION_DONE_TX, R11, 0
    LBCO    &R10, ICSS_SHARED_CONST, TIMESYNC_IEP_VAL_CYCLE_COUNTER, 4
    QBGT    COMPENSATION_DONE_TX, R10, R5
    ADD     R2, R2, 1
    ADC     R3.w0, R3.w0, 0

COMPENSATION_DONE_TX:

    LDI32     R10, IEP_WRAP_AROUND_VAL
    ;Do PHY compensation
    LBCO    &R5.w0, ICSS_SHARED_CONST, MII_TX_CORRECTION_OFFSET, 2
    ADD     R4, R4, R5.w0
    QBGT    NO_WRAPAROUND_TX, R4, R10
    ;Add 1 to seconds timestamp
    ADD     R2, R2, 1
    ADC     R3.w0, R3.w0, 0
    SUB     R4, R10, R4

NO_WRAPAROUND_TX:

    ;nanoseconds in R4 and seconds in R2 and R3.w0

    JMP     R0.w2

;***************************************************************************
;
;     NAME             : FN_PTP_TX_ADD_DELAY
;     DESCRIPTION      : 1. Function for measuring bridge delay
;                        2. In Sync frames
;                        3. For Pdelay Response frames
;                        4. For storing Tx timestamp for Sync and Pdelay Req frames
;     Cycles           : 40 PRU cycles (worst case)
;     Register Usage   : R22 (flags), R0, R13, R25, R26, R28, R29
;     Pseudocode       :
;22 bytes have been put in Tx FIFO
;if ptp frame:
;   load tx SOF timestamp
;   do phy delay correction on tx SOF
;       if Pdelay Request frame:
;           store tx SOF TS
;           indicate that this is a Pdelay request frame (by writing into shared memory)
;           set flag for Tx callback interrupt
;       elif Pdelay response frame:
;           store tx SOF TS
;           indicate that this is a Pdelay response frame (by writing into shared memory)
;           set flag for Tx callback interrupt
;       elif sync frame:
;           store tx SOF TS
;           indicate that this is a Sync frame (by writing into shared memory)
;           set flag for Tx callback interrupt
;       else:
;           exit
;else:
;   exit
;
;
;***************************************************************************

FN_PTP_TX_ADD_DELAY:

    QBBC    EXIT_PTP_TX_ADD_DELAY, R22, TX_IS_PTP_BIT
    CLR     R22, R22, TX_IS_PTP_BIT

    ;check if frame carries VLAN tag
    LDI     R1.b0, &R5.b2
    QBNE    NO_VLAN_TX, R5.w0, VLAN_EtherType
    ADD     R1.b0, R1.b0, 4
NO_VLAN_TX:
    MVIB    R25.b0, *R1.b0

NO_HSR_TAG_TX:

    MVIB    R25.b0, *R1.b0

    QBNE    NOT_PDELAY_REQ_TX, R25.b0, PTP_PDLY_REQ_MSG_ID
    ;***************************PDELAY REQ PACKET PROCESSING***************************
    ;Take timestamp T1 here
    JAL     R0.w2, FN_GET_TX_TIMESTAMP

    .if $defined("ICSS_SWITCH_BUILD")
    .if $defined (PRU0)
    LDI     R10.w0, TX_PDELAY_REQ_TIMESTAMP_OFFSET_P2
    LDI     R10.w2, TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P2
    .else
    LDI     R10.w0, TX_PDELAY_REQ_TIMESTAMP_OFFSET_P1
    LDI     R10.w2, TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P1
    .endif
    .else
    .if $defined (PRU0)
    LDI     R10.w0, TX_PDELAY_REQ_TIMESTAMP_OFFSET_P1
    LDI     R10.w2, TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P1
    .else
    LDI     R10.w0, TX_PDELAY_REQ_TIMESTAMP_OFFSET_P2
    LDI     R10.w2, TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P2
    .endif
    .endif
    ;Store the Tx TS, set flag for
    QBA     STORE_TX_TS_SET_FLAG_EXIT

NOT_PDELAY_REQ_TX:

    QBNE    NOT_PDELAY_RES_TX, R25.b0, PTP_PDLY_RSP_MSG_ID

    ;***************************PDELAY RES PACKET PROCESSING***************************
    ;Load the Delay Request Timestamp to compute T3 - T2
    ;Load T2, we are echoing back here so load same port timestamps
    ;Store the Tx timestamp
    JAL     R0.w2, FN_GET_TX_TIMESTAMP

    .if $defined("ICSS_SWITCH_BUILD")
    .if $defined (PRU0)
    LDI     R10.w0, TX_PDELAY_RESP_TIMESTAMP_OFFSET_P2
    LDI     R10.w2, TX_TS_NOTIFICATION_OFFSET_PDEL_RES_P2
    .else
    LDI     R10.w0, TX_PDELAY_RESP_TIMESTAMP_OFFSET_P1
    LDI     R10.w2, TX_TS_NOTIFICATION_OFFSET_PDEL_RES_P1
    .endif
    .else
    .if $defined (PRU0)
    LDI     R10.w0, TX_PDELAY_RESP_TIMESTAMP_OFFSET_P1
    LDI     R10.w2, TX_TS_NOTIFICATION_OFFSET_PDEL_RES_P1
    .else
    LDI     R10.w0, TX_PDELAY_RESP_TIMESTAMP_OFFSET_P2
    LDI     R10.w2, TX_TS_NOTIFICATION_OFFSET_PDEL_RES_P2
    .endif
    .endif

    ;Store the Tx TS, set flag for
    QBA     STORE_TX_TS_SET_FLAG_EXIT

NOT_PDELAY_RES_TX:

    QBNE    EXIT_PTP_TX_ADD_DELAY, R25.b0, PTP_SYNC_MSG_ID
    ;***************************SYNC PACKET PROCESSING***************************
    ;processing two-step sync frame
    ;Irrespective of slave/master status we just store timestamp and
    ;set flag to give a callback interrupt

    JAL     R0.w2, FN_GET_TX_TIMESTAMP

    .if $defined("ICSS_SWITCH_BUILD")
    .if $defined (PRU0)
    LDI     R10.w0, TX_SYNC_TIMESTAMP_OFFSET_P2
    LDI     R10.w2, TX_TS_NOTIFICATION_OFFSET_SYNC_P2
    .else
    LDI     R10.w0, TX_SYNC_TIMESTAMP_OFFSET_P1
    LDI     R10.w2, TX_TS_NOTIFICATION_OFFSET_SYNC_P1
    .endif
    .else
    .if $defined (PRU0)
    LDI     R10.w0, TX_SYNC_TIMESTAMP_OFFSET_P1
    LDI     R10.w2, TX_TS_NOTIFICATION_OFFSET_SYNC_P1
    .else
    LDI     R10.w0, TX_SYNC_TIMESTAMP_OFFSET_P2
    LDI     R10.w2, TX_TS_NOTIFICATION_OFFSET_SYNC_P2
    .endif
    .endif

    ;Store the Tx TS, set flag for

STORE_TX_TS_SET_FLAG_EXIT:

    ;nanoseconds in R4, Seconds in R2 and R3.w0
    ;TS offset is in R10.w0 and notification offset in R10.w2
    SBCO    &R4, ICSS_SHARED_CONST, R10.w0, 4
    ADD     R10.w0, R10.w0, 4
    SBCO    &R2, ICSS_SHARED_CONST, R10.w0, 6

    ;store notification
    LDI     R25.b2, 1
    SBCO    &R25.b2, ICSS_SHARED_CONST, R10.w2, 1
    ;set indicator for callback interrupt
    SET     R22, R22, TX_CALLBACK_INTERRUPT_BIT

EXIT_PTP_TX_ADD_DELAY:

    JMP     R0.w0

    .if $defined (PRU0)

;***************************************************************************
;
;     NAME             : FN_PTP_BACKGROUND_TASK
;     DESCRIPTION      : Background task which reprograms CMP1 and disables/enables
;                        Sync0 for 1PPS output
;     Cycles           : 30 PRU cycles (worst case)
;     Register Usage   : R22 (flags), R0, R2, R3, R12, R13, R20, R21
;     Pseudocode       :
;
;
;***************************************************************************

FN_PTP_BACKGROUND_TASK:

    ;check if compare0 has been asserted and increment counter
    LBCO    &TEMP_REG_3.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
    ;load IEP Counter for later usage
    LBCO    &RCV_TEMP_REG_1, IEP_CONST, IEP_COUNTER_OFFSET, 4
    QBBC    CHECK_CMP1, TEMP_REG_3, 0
    LDI     TEMP_REG_3.b2, 1
    ;clear the status
    SBCO    &TEMP_REG_3.b2, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
    ;increment the counter
    LBCO    &TEMP_REG_4, ICSS_SHARED_CONST, TIMESYNC_SECONDS_COUNT_OFFSET, 6
    ADD     TEMP_REG_4, TEMP_REG_4, 1
    ADC     TEMP_REG_2.w0, TEMP_REG_2.w0, 0
    SBCO    &TEMP_REG_4, ICSS_SHARED_CONST, TIMESYNC_SECONDS_COUNT_OFFSET, 6

    ;save the current IEP value for comparison later
    SBCO    &RCV_TEMP_REG_1, ICSS_SHARED_CONST, TIMESYNC_IEP_VAL_CYCLE_COUNTER, 4

CHECK_CMP1:
    ;check PTP enable bit and exit if not enabled
    LBCO    &TEMP_REG_3.b2, ICSS_SHARED_CONST, TIMESYNC_CTRL_VAR_OFFSET, 1
    QBBC    EXIT_PTP_BACKGROUND_TASK, TEMP_REG_3.b2, GPTP_ENABLE_BG_TASK_BIT

    QBBC    CHECK_SYNC0_SIGNAL, TEMP_REG_3, 1
    SET     R22, R22, CHECK_SYNC0_BIT
    LBCO    &RCV_TEMP_REG_4, IEP_CONST, ICSS_IEP_CMP1_REG, 4
    ;read cmp1 register and reprogram the value
    LBCO    &RCV_TEMP_REG_2, ICSS_SHARED_CONST, TIMESYNC_CMP1_PERIOD_OFFSET, 4
    LBCO    &TEMP_REG_1, ICSS_SHARED_CONST, TIMESYNC_SYNC0_WIDTH_OFFSET, 4
    ADD     RCV_TEMP_REG_3, RCV_TEMP_REG_4, TEMP_REG_1
    ADD     RCV_TEMP_REG_4, RCV_TEMP_REG_4, RCV_TEMP_REG_2
    LDI32     R25, IEP_WRAP_AROUND_VAL
    QBLE    CMP1_IS_LESS_THAN_WRAPAROUND, R25, RCV_TEMP_REG_4
    MOV     RCV_TEMP_REG_4, RCV_TEMP_REG_2
    MOV     RCV_TEMP_REG_3, TEMP_REG_1

CMP1_IS_LESS_THAN_WRAPAROUND:

    SBCO    &RCV_TEMP_REG_4, IEP_CONST, ICSS_IEP_CMP1_REG, 4
    SBCO    &RCV_TEMP_REG_3, ICSS_SHARED_CONST, TIMESYNC_SYNC0_CMP_VALUE, 4
    LDI     TEMP_REG_3.b2, 2
    SBCO    &TEMP_REG_3.b2, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
    QBA     EXIT_PTP_BACKGROUND_TASK

CHECK_SYNC0_SIGNAL:

    QBBC    EXIT_PTP_BACKGROUND_TASK, R22, CHECK_SYNC0_BIT
    LBCO    &RCV_TEMP_REG_4, ICSS_SHARED_CONST, TIMESYNC_SYNC0_CMP_VALUE, 4
    QBLE    SYNC0_STILL_HIGH, RCV_TEMP_REG_4, RCV_TEMP_REG_1
    CLR     R22, R22, CHECK_SYNC0_BIT
    ;disable and enable the sync0
    LDI     TEMP_REG_4.b0, 0
    LDI     TEMP_REG_4.w2, IEP_SYNC_CTRL_REG    ; clear sync enable
    SBCO    &TEMP_REG_4.b0, IEP_CONST, TEMP_REG_4.w2, 1
    LDI     TEMP_REG_4.b0, 0x3    ; set sync enable
    SBCO    &TEMP_REG_4.b0, IEP_CONST, TEMP_REG_4.w2, 1

SYNC0_STILL_HIGH:

EXIT_PTP_BACKGROUND_TASK:

    JMP     CALL_REG
    .endif ;PRU0

    .else

;****************************************************************************
;
;     NAME             : FN_TIMESTAMP_GPTP_PACKET
;     DESCRIPTION      : Store timestamp for PTP frames.
;                        It is called during Rx
;     Cycles           : 34
;     Register Usage   : R22 (flags), R20, R21, R10, R11, R13
;     Pseudocode       :
;if PTP frame:
;   if not link local
;       if sync frame:
;           if PTP not enabled:
;               discard frame
;           else:
;               if not from master:
;                   do not forward to host
;               else:
;                   if first sync:
;                       update the master port
;                   take timestamp()
;       elif follow up
;           if PTP not enabled:
;               discard frame
;           else:
;               if not from master:
;                   do not forward to host
;               else:
;                   clear fwd_flag
;                   exit without timestamp
;
;   else:
;       check if hsr tag is there and load appropriate offset
;       if pdelay req frame or pdelay response frame:
;           take timestamp()
;def take_timestamp()
;   load SFD timestamp
;   exit
;else:
;   exit
;
;****************************************************************************

FN_TIMESTAMP_GPTP_PACKET:
    QBBC    FN_TIMESTAMP_GPTP_PACKET_EXIT, R22, RX_IS_PTP_BIT


    .if $defined (PRU0)
        LDI     R13.w0, RX_TIMESTAMP_OFFSET_P1
    .else
        LDI     R13.w0, RX_TIMESTAMP_OFFSET_P2
    .endif

GPTP_STORE_TIMESTAMP:

    .if $isdefed("PRU0")
    LBCO    &R20, IEP_CONST, CAP_RISE_RX_SFD_PORT1_OFFSET, 8    ; Load RX SFD
    .else
    LBCO    &R20, IEP_CONST, CAP_RISE_RX_SFD_PORT2_OFFSET, 8    ; Load RX SFD
    .endif
    ;subtract rx correction and store the timestamp
    LDI     R11.w2, MII_RX_CORRECTION_OFFSET
    LBCO    &R11.w0, ICSS_SHARED_CONST, R11.w2, 2
    SUB     R20, R20, R11.w0
    SUC     R21, R21, 0

    SBCO    &R20, ICSS_SHARED_CONST, R13.w0, 8

FN_TIMESTAMP_GPTP_PACKET_EXIT:

    JMP     RCV_TEMP_REG_3.w2

;****************************************************************************
;
;     NAME             : FN_PTP_TX_ADD_DELAY
;     DESCRIPTION      : 1. Function for measuring bridge delay
;                        2. In Sync frames
;                        3. For Pdelay Response frames
;                        4. For storing Tx timestamp for Sync and Pdelay Req frames
;     Cycles           : 40 PRU cycles (worst case)
;     Register Usage   : R22 (flags), R0, R13, R25, R26, R28, R29
;     Pseudocode       :
;22 bytes have been put in Tx FIFO
;if ptp frame:
;   load tx SOF timestamp
;   do phy delay correction on tx SOF
;       if Pdelay Request frame:
;           store tx SOF TS
;           indicate that this is a Pdelay request frame (by writing into shared memory)
;           set flag for Tx callback interrupt
;       elif Pdelay response frame:
;           store tx SOF TS
;           indicate that this is a Pdelay response frame (by writing into shared memory)
;           set flag for Tx callback interrupt
;       elif sync frame:
;           store tx SOF TS
;           indicate that this is a Sync frame (by writing into shared memory)
;           set flag for Tx callback interrupt
;       else:
;           exit
;else:
;   exit
;
;
;****************************************************************************

FN_PTP_TX_ADD_DELAY:
    QBBC    EXIT_PTP_TX_ADD_DELAY, R22, TX_IS_PTP_BIT
    CLR     R22, R22, TX_IS_PTP_BIT

    ;we need to make sure that the timestamp here is actual value
    ;this we do by comparing with Tx SOF of previous frame
    .if    $isdefed("PRU0")
    LDI     R20.w0, PTP_PREV_TX_TIMESTAMP_P1
    .else
    LDI     R20.w0, PTP_PREV_TX_TIMESTAMP_P2
    .endif
    LBCO    &R20, ICSS_SHARED_CONST, R20.w0, 8
    ;get the timestamp

LOAD_TX_SOF_TS:

    .if  $defined("ICSS_SWITCH_BUILD")
    .if    $isdefed("PRU0")
    LBCO    &R10, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 8
    .else
    LBCO    &R10, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 8
    .endif
    .else
    .if    $isdefed("PRU0")
    LBCO    &R10, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 8
    .else
    LBCO    &R10, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 8
    .endif
    .endif

    QBNE    TX_SOF_OK, R10, R20
    QBEQ    LOAD_TX_SOF_TS, R11, R21

TX_SOF_OK:

    ;Do phy delay correction here
    LDI     R0.w2, MII_TX_CORRECTION_OFFSET
    LBCO    &R25.w0, ICSS_SHARED_CONST, R0.w2, 2
    ADD     R10, R10, R25.w0
    ADC     R11, R11, 0
    ;check if frame carries VLAN tag
    LDI     R1.b0, &R5.b2
    QBNE    NO_VLAN_TX, R5.w0, VLAN_EtherType
    ADD     R1.b0, R1.b0, 4
NO_VLAN_TX:
    MVIB    R25.b0, *R1.b0

NO_HSR_TAG_TX:

    MVIB    R25.b0, *R1.b0

    QBNE    NOT_PDELAY_REQ_TX, R25.b0, PTP_PDLY_REQ_MSG_ID
    ;****************************PDELAY REQ PACKET PROCESSING***************************
    ;Take timestamp T1 here
    .if    $defined("ICSS_SWITCH_BUILD")
    .if    $isdefed("PRU0")
    SBCO    &R10, ICSS_SHARED_CONST, TX_PDELAY_REQ_TIMESTAMP_OFFSET_P2, 8
    .else
    SBCO    &R10, ICSS_SHARED_CONST, TX_PDELAY_REQ_TIMESTAMP_OFFSET_P1, 8
    .endif
    .else
    .if    $isdefed("PRU0")
    SBCO    &R10, ICSS_SHARED_CONST, TX_PDELAY_REQ_TIMESTAMP_OFFSET_P1, 8
    .else
    SBCO    &R10, ICSS_SHARED_CONST, TX_PDELAY_REQ_TIMESTAMP_OFFSET_P2, 8
    .endif
    .endif

    ;store the value and set and interrupt
    LDI     R25.b2, 1

    .if    $defined("ICSS_SWITCH_BUILD")
    .if    $isdefed("PRU0")
    SBCO    &R25.b2, ICSS_SHARED_CONST, TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P2, 1    ; store the notification
    .else
    SBCO    &R25.b2, ICSS_SHARED_CONST, TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P1, 1    ; store the notification
    .endif
    .else
    .if    $isdefed("PRU0")
    SBCO    &R25.b2, ICSS_SHARED_CONST, TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P1, 1    ; store the notification
    .else
    SBCO    &R25.b2, ICSS_SHARED_CONST, TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P2, 1    ; store the notification
    .endif
    .endif
    ;set tx callback interrupt. Just set the flag, interrupt is set upon successful transmission
    SET     R22, R22, TX_CALLBACK_INTERRUPT_BIT

    QBA     EXIT_PTP_TX_ADD_DELAY

NOT_PDELAY_REQ_TX:

    QBNE    NOT_PDELAY_RES_TX, R25.b0, PTP_PDLY_RSP_MSG_ID

    ;****************************PDELAY RES PACKET PROCESSING***************************
    ;Load the Delay Request Timestamp to compute T3 - T2
    ;Load T2, we are echoing back here so load same port timestamps
    ;Store the Tx timestamp

    .if    $defined("ICSS_SWITCH_BUILD")
    .if    $isdefed("PRU0")
    SBCO    &R10, ICSS_SHARED_CONST, TX_PDELAY_RESP_TIMESTAMP_OFFSET_P2, 8
    .else
    SBCO    &R10, ICSS_SHARED_CONST, TX_PDELAY_RESP_TIMESTAMP_OFFSET_P1, 8
    .endif
    .else
    .if    $isdefed("PRU0")
    SBCO    &R10, ICSS_SHARED_CONST, TX_PDELAY_RESP_TIMESTAMP_OFFSET_P1, 8
    .else
    SBCO    &R10, ICSS_SHARED_CONST, TX_PDELAY_RESP_TIMESTAMP_OFFSET_P2, 8
    .endif
    .endif
    ;store the indicator and set and interrupt
    LDI     R25.b2, 1

    .if    $defined("ICSS_SWITCH_BUILD")
    .if    $isdefed("PRU0")
    SBCO    &R25.b2, ICSS_SHARED_CONST, TX_TS_NOTIFICATION_OFFSET_PDEL_RES_P2, 1    ; store the notification
    .else
    SBCO    &R25.b2, ICSS_SHARED_CONST, TX_TS_NOTIFICATION_OFFSET_PDEL_RES_P1, 1    ; store the notification
    .endif
    .else
    .if    $isdefed("PRU0")
    SBCO    &R25.b2, ICSS_SHARED_CONST, TX_TS_NOTIFICATION_OFFSET_PDEL_RES_P1, 1    ; store the notification
    .else
    SBCO    &R25.b2, ICSS_SHARED_CONST, TX_TS_NOTIFICATION_OFFSET_PDEL_RES_P2, 1    ; store the notification
    .endif
    .endif
    ;set tx callback interrupt. The interrupt is set upon successful transmission
    SET     R22, R22, TX_CALLBACK_INTERRUPT_BIT
    QBA     EXIT_PTP_TX_ADD_DELAY

NOT_PDELAY_RES_TX:

    QBNE    EXIT_PTP_TX_ADD_DELAY, R25.b0, PTP_SYNC_MSG_ID
    ;****************************SYNC PACKET PROCESSING***************************
    ;Irrespective of slave/master status we just store timestamp and
    ;set flag to give a callback interrupt

    .if    $defined("ICSS_SWITCH_BUILD")
    .if    $isdefed("PRU0")
    SBCO    &R10, ICSS_SHARED_CONST, TX_SYNC_TIMESTAMP_OFFSET_P2, 8
    .else
    SBCO    &R10, ICSS_SHARED_CONST, TX_SYNC_TIMESTAMP_OFFSET_P1, 8
    .endif
    .else
    .if    $isdefed("PRU0")
    SBCO    &R10, ICSS_SHARED_CONST, TX_SYNC_TIMESTAMP_OFFSET_P1, 8
    .else
    SBCO    &R10, ICSS_SHARED_CONST, TX_SYNC_TIMESTAMP_OFFSET_P2, 8
    .endif
    .endif
    ;store the indicator and set and interrupt
    LDI     R25.b2, 1

    .if    $defined("ICSS_SWITCH_BUILD")
    .if    $isdefed("PRU0")
    SBCO    &R25.b2, ICSS_SHARED_CONST, TX_TS_NOTIFICATION_OFFSET_SYNC_P2, 1    ; store the notification
    .else
    SBCO    &R25.b2, ICSS_SHARED_CONST, TX_TS_NOTIFICATION_OFFSET_SYNC_P1, 1    ; store the notification
    .endif
    .else
    .if    $isdefed("PRU0")
    SBCO    &R25.b2, ICSS_SHARED_CONST, TX_TS_NOTIFICATION_OFFSET_SYNC_P1, 1    ; store the notification
    .else
    SBCO    &R25.b2, ICSS_SHARED_CONST, TX_TS_NOTIFICATION_OFFSET_SYNC_P2, 1    ; store the notification
    .endif
    .endif
    ;set tx callback interrupt. The interrupt is set upon successful transmission
    SET     R22, R22, TX_CALLBACK_INTERRUPT_BIT

EXIT_PTP_TX_ADD_DELAY:

    JMP     R0.w0

;****************************************************************************
;
;     NAME             : FN_PTP_BACKGROUND_TASK
;     DESCRIPTION      : Background task which reprograms CMP1 and disables/enables
;                        Sync0 for 1PPS output
;     Cycles           : 30 PRU cycles (worst case)
;     Register Usage   : R22 (flags), R0, R2, R3, R12, R13, R20, R21
;     Pseudocode       :
;
;
;****************************************************************************

FN_PTP_BACKGROUND_TASK:

    ;check PTP enable bit and exit if not enabled
    LBCO    &TEMP_REG_3.b0, ICSS_SHARED_CONST, TIMESYNC_CTRL_VAR_OFFSET, 1
    QBBC    EXIT_PTP_BACKGROUND_TASK, TEMP_REG_3, GPTP_ENABLE_BG_TASK_BIT

    ;check if compare1 has been asserted and increment counter
    LBCO    &TEMP_REG_3.b0, IEP_CONST, IEP_CMP_STATUS_REG, 1

CHECK_CMP1:

    QBBC    CHECK_SYNC0_SIGNAL, TEMP_REG_3, 1
    SET     R22, R22, CHECK_SYNC0_BIT

    ;a load from IEP is 15 cycles but shared memory load is 3 cycles
    ;so instead of loading from IEP to reprogram we use a shared location offset i.e. GPTP_CMP1_CMP_OFFSET
    ;a similar location i.e. GPTP_SYNC0_CMP_OFFSET is used to store dynamic value of CMP1 value plus sync width
    LBCO  &RCV_TEMP_REG_4, ICSS_SHARED_CONST, TIMESYNC_CMP1_CMP_OFFSET, 8    ;loads into RCV_TEMP_REG_4 & RCV_TEMP_REG_3

    LBCO    &TEMP_REG_3, ICSS_SHARED_CONST, TIMESYNC_CMP1_PERIOD_OFFSET, 8    ;loads TIMESYNC_CMP1_PERIOD_OFFSET to TEMP_REG_3 and TIMESYNC_SYNC0_WIDTH_OFFSET to TEMP_REG_4
    ADD     RCV_TEMP_REG_1, RCV_TEMP_REG_4, TEMP_REG_3
    ADC     RCV_TEMP_REG_2, RCV_TEMP_REG_3, 0

    ADD     RCV_TEMP_REG_4, RCV_TEMP_REG_4, TEMP_REG_4
    ADC     RCV_TEMP_REG_3, RCV_TEMP_REG_3, 0

    SBCO    &RCV_TEMP_REG_1, IEP_CONST, IEP_CMP1_REG, 8            ;program new cmp1 value
    SBCO    &RCV_TEMP_REG_1, ICSS_SHARED_CONST, TIMESYNC_CMP1_CMP_OFFSET, 8            ;also store new cmp1 value into memory, this saves cycles as explained above
    SBCO    &RCV_TEMP_REG_4, ICSS_SHARED_CONST, TIMESYNC_SYNC0_CMP_OFFSET, 8            ;store into memory, used for comparison later
    LDI     TEMP_REG_3.b2, 2
    SBCO    &TEMP_REG_3.b2, IEP_CONST, IEP_CMP_STATUS_REG, 1

    QBA     EXIT_PTP_BACKGROUND_TASK

CHECK_SYNC0_SIGNAL:

    ;check if cmp1 has been hit already. Check the R22 bit set earlier
    QBBC    EXIT_PTP_BACKGROUND_TASK, R22, CHECK_SYNC0_BIT
    LBCO    &RCV_TEMP_REG_4, IEP_CONST, IEP_COUNTER_OFFSET, 8                ;load current IEP into RCV_TEMP_REG_4 & RCV_TEMP_REG_3
    LBCO    &RCV_TEMP_REG_1, ICSS_SHARED_CONST, TIMESYNC_SYNC0_CMP_OFFSET, 8  ;load the time when sync0 signal will go down. Into RCV_TEMP_REG_1 & RCV_TEMP_REG_2

    ;compare current IEP with loaded value
    QBLT    EXIT_PTP_BACKGROUND_TASK, RCV_TEMP_REG_2, RCV_TEMP_REG_3
    QBNE    RESET_SYNC0, RCV_TEMP_REG_2, RCV_TEMP_REG_3
    QBLT    EXIT_PTP_BACKGROUND_TASK, RCV_TEMP_REG_1, RCV_TEMP_REG_4

RESET_SYNC0:

    CLR     R22, R22, CHECK_SYNC0_BIT
    ;disable and enable the sync0
    LDI     TEMP_REG_4.b0, 0
    LDI     TEMP_REG_4.w2, IEP_SYNC_CTRL_REG    ; clear sync enable
    SBCO    &TEMP_REG_4.b0, IEP_CONST, TEMP_REG_4.w2, 1
    LDI     TEMP_REG_4.b0, 3    ; set sync enable
    SBCO    &TEMP_REG_4.b0, IEP_CONST, TEMP_REG_4.w2, 1

EXIT_PTP_BACKGROUND_TASK:

    JMP     CALL_REG

    .endif

;****************************************************************************
;
;     NAME             : FN_PTP_TX_ADD_DELAY_UDP
;     DESCRIPTION      : 1. Function for measuring bridge delay
;                        2. In Sync frames
;                        3. For Pdelay Response frames
;                        4. For storing Tx timestamp for Sync and Pdelay Req frames
;     Cycles           : 75 PRU cycles (worst case)
;     Register Usage   : R22 (flags), R0, R13, R25, R26, R28, R29
;     Pseudocode       :
;22 bytes have been put in Tx FIFO
;if ptp frame:
;   load tx SOF timestamp
;   do phy delay correction on tx SOF
;       if Pdelay Request frame:
;           store tx SOF TS
;           indicate that this is a Pdelay request frame (by writing into shared memory)
;           set flag for Tx callback interrupt
;       elif Pdelay response frame:
;           store tx SOF TS
;           indicate that this is a Pdelay response frame (by writing into shared memory)
;           set flag for Tx callback interrupt
;       elif sync frame:
;           store tx SOF TS
;           indicate that this is a Sync frame (by writing into shared memory)
;           set flag for Tx callback interrupt
;       else:
;           exit
;else:
;   exit
;
;
;****************************************************************************
FN_PTP_TX_ADD_DELAY_UDP:
    QBBC    EXIT_PTP_TX_ADD_DELAY_UDP, R22, TX_IS_UDP_PTP_BIT
    CLR    R22, R22, TX_IS_UDP_PTP_BIT


CHECK_UDP_PORT_TX:
    QBBS    CHECK_UDP_PORT_TX_VLAN, R22, TX_IS_VLAN_BIT
    LDI     R20.w0, UDP_PTP_PORT_319
    QBEQ    DONE_UDP_CHECK, UDP_SRC_PORT_REG, R20.w0
    LDI     R20.w0, UDP_PTP_PORT_320
    QBNE    EXIT_PTP_TX_ADD_DELAY_UDP, UDP_SRC_PORT_REG, R20.w0
    QBA     DONE_UDP_CHECK

CHECK_UDP_PORT_TX_VLAN:
    CLR     R22, R22, TX_IS_VLAN_BIT
    LDI     R20.w0, UDP_PTP_PORT_319
    QBEQ    DONE_UDP_CHECK, UDP_SRC_PORT_VLAN_REG, R20.w0
    LDI     R20.w0, UDP_PTP_PORT_320
    QBNE    EXIT_PTP_TX_ADD_DELAY_UDP, UDP_SRC_PORT_VLAN_REG, R20.w0

DONE_UDP_CHECK:

    ;forcibly set 2-step flag. We don't support 1-step
    QBBS    PTP_TX_IS_VLAN_UDP, R22, TX_IS_VLAN_BIT
    ;forcibly set 2-step flag for no vlan case
    SET     two_step_reg_udp, two_step_reg_udp, GPTP_802_3_two_step_bit
    QBA     PTP_TX_UDP_GET_TS
PTP_TX_IS_VLAN_UDP:
    SET     two_step_reg_udp_vlan, two_step_reg_udp_vlan, GPTP_802_3_two_step_bit
PTP_TX_UDP_GET_TS:

    ;we need to make sure that the timestamp here is actual value
    ;this we do by comparing with Tx SOF of previous frame
    .if    $isdefed("PRU0")
    LDI     R20.w0, PTP_PREV_TX_TIMESTAMP_P1
    .else
    LDI     R20.w0, PTP_PREV_TX_TIMESTAMP_P2
    .endif
    LBCO    &R20, ICSS_SHARED_CONST, R20.w0, 8
    ;get the timestamp

LOAD_TX_SOF_TS_UDP:

    .if  $defined("ICSS_SWITCH_BUILD")
    .if    $isdefed("PRU0")
    LBCO    &R10, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 8
    .else
    LBCO    &R10, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 8
    .endif
    .else
    .if    $isdefed("PRU0")
    LBCO    &R10, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 8
    .else
    LBCO    &R10, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 8
    .endif
    .endif

    QBNE    TX_SOF_OK_UDP, R10, R20
    QBEQ    LOAD_TX_SOF_TS_UDP, R11, R21

TX_SOF_OK_UDP:

    QBNE    NOT_DELAY_REQ_TX_UDP, PTP_MSG_ID_REG_UDP, PTP_DLY_REQ_MSG_ID
    ;****************************DELAY REQ PACKET PROCESSING***************************
    ;If packet is from host then we just store the exit timestamp
    ;else we do bridge delay correction
    ;Take timestamp T1 here
    ;JAL       R0.w2, FN_GET_TX_TIMESTAMP

    .if $isdefed("ICSS_V_1_0")

    JAL    R0.w2, FN_GET_TX_TIMESTAMP

    .else    ; "ICSS_V_1_0"

    .if    $defined("ICSS_SWITCH_BUILD")
    .if $isdefed("PRU0")
    LBCO    &R20, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 8
    .else    ; "PRU0"
    LBCO    &R20, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 8
    .endif    ; "PRU0"
    .else
    .if $isdefed("PRU0")
    LBCO    &R20, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 8
    .else    ; "PRU0"
    LBCO    &R20, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 8
    .endif    ; "PRU0"
    .endif    ; "ICSS_SWITCH_BUILD"
    .endif    ; "ICSS_V_1_0"

    .if    $defined("ICSS_SWITCH_BUILD")
    .if $isdefed("PRU0")
    LDI    R10.w0, TX_PDELAY_REQ_TIMESTAMP_OFFSET_P2
    LDI    R10.w2, TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P2
    .else    ; "PRU0"
    LDI    R10.w0, TX_PDELAY_REQ_TIMESTAMP_OFFSET_P1
    LDI    R10.w2, TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P1
    .endif    ; "PRU0"
    .else
    .if $isdefed("PRU0")
    LDI    R10.w0, TX_PDELAY_REQ_TIMESTAMP_OFFSET_P1
    LDI    R10.w2, TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P1
    .else    ; "PRU0"
    LDI    R10.w0, TX_PDELAY_REQ_TIMESTAMP_OFFSET_P2
    LDI    R10.w2, TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P2
    .endif    ; "PRU0"
    .endif    ; "ICSS_SWITCH_BUILD"

    ;Store the Tx TS, set flag for
    QBA       STORE_TX_TS_SET_FLAG_EXIT_UDP

NOT_DELAY_REQ_TX_UDP:
    QBNE    NOT_DELAY_RESP_UDP, PTP_MSG_ID_REG_UDP, PTP_DLY_RESP_MSG_ID

    ;****************************DELAY RES PACKET PROCESSING***************************
    ;Nothing to be done here. Add code here when master is implemented

    QBA       EXIT_PTP_TX_ADD_DELAY_UDP

NOT_DELAY_RESP_UDP:
    QBNE    EXIT_PTP_TX_ADD_DELAY_UDP, PTP_MSG_ID_REG_UDP, PTP_SYNC_MSG_ID
    ;****************************SYNC PACKET PROCESSING***************************

    ;processing two-step sync frame
    ;Irrespective of slave/master status we just store timestamp and
    ;set flag to give a callback interrupt

    .if $isdefed("ICSS_V_1_0")
    JAL    R0.w2, FN_GET_TX_TIMESTAMP
    .else    ; "ICSS_V_1_0"
    .if    $defined("ICSS_SWITCH_BUILD")
    .if $isdefed("PRU0")
    LBCO    &R20, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 8
    .else    ; "PRU0"
    LBCO    &R20, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 8
    .endif    ; "PRU0"
    .else
    .if $isdefed("PRU0")
    LBCO    &R20, IEP_CONST, CAP_RISE_TX_SOF_PORT1_OFFSET, 8
    .else    ; "PRU0"
    LBCO    &R20, IEP_CONST, CAP_RISE_TX_SOF_PORT2_OFFSET, 8
    .endif    ; "PRU0"
    .endif    ; "ICSS_SWITCH_BUILD"
    .endif    ; "ICSS_V_1_0"

    .if    $defined("ICSS_SWITCH_BUILD")
    .if $isdefed("PRU0")
    LDI    R10.w0, TX_SYNC_TIMESTAMP_OFFSET_P2
    LDI    R10.w2, TX_TS_NOTIFICATION_OFFSET_SYNC_P2
    .else    ; "PRU0"
    LDI    R10.w0, TX_SYNC_TIMESTAMP_OFFSET_P1
    LDI    R10.w2, TX_TS_NOTIFICATION_OFFSET_SYNC_P1
    .endif    ; "PRU0"
    .else
    .if $isdefed("PRU0")
    LDI    R10.w0, TX_SYNC_TIMESTAMP_OFFSET_P1
    LDI    R10.w2, TX_TS_NOTIFICATION_OFFSET_SYNC_P1
    .else    ; "PRU0"
    LDI    R10.w0, TX_SYNC_TIMESTAMP_OFFSET_P2
    LDI    R10.w2, TX_TS_NOTIFICATION_OFFSET_SYNC_P2
    .endif    ; "PRU0"
    .endif

    .if $isdefed("ICSS_V_1_0")
STORE_TX_TS_SET_FLAG_EXIT_UDP:
    ;nanoseconds in R4, Seconds in R2 and R3.w0
    ;TS offset is in R10.w0 and notification offset in R10.w2
    SBCO    &R4, ICSS_SHARED_CONST, R10.w0, 4
    ADD    R10.w0, R10.w0, 4
    SBCO    &R2, ICSS_SHARED_CONST, R10.w0, 6

    ;store notification
    LDI    R25.b2, 1
    SBCO    &R25.b2, ICSS_SHARED_CONST, R10.w2, 1
    ;set indicator for callback interrupt
    SET    R22, R22, TX_CALLBACK_INTERRUPT_BIT

    ;restore R2-R5 bytes. The timestamps in R2, R3 and R4 have been consumed
    LDI    R10.w0, PTP_SCRATCH_MEM
    LBCO    &R2, ICSS_SHARED_CONST, R10.w0, 16
    .else    ; "ICSS_V_1_0"

STORE_TX_TS_SET_FLAG_EXIT_UDP:
    ;Timestamp is in R20 + R21 (8 bytes of IEP)
    ;add Tx PHY delay
    LBCO    &R25.w0, ICSS_SHARED_CONST, MII_TX_CORRECTION_OFFSET, 2
    ADD    R20, R20, R25.w0
    ADC    R21, R21, 0

    SBCO    &R20, ICSS_SHARED_CONST, R10.w0, 8

    ;store notification
    LDI    R25.b2, 1
    SBCO    &R25.b2, ICSS_SHARED_CONST, R10.w2, 1
    ;set indicator for callback interrupt
    SET    R22, R22, TX_CALLBACK_INTERRUPT_BIT
    .endif    ; "ICSS_V_1_0"

EXIT_PTP_TX_ADD_DELAY_UDP:
    JMP    R0.w0

;****************************************************************************
;
;     NAME             : FN_CHECK_AND_CLR_PTP_FWD_FLAG
;     DESCRIPTION      : Check if the PTP-UDP packet is Pdelay Req/Response
;                        so it's not forwarded. Called when 32-64 bytes are in R2-R9
;     Cycles           : TBD
;     Register Usage   : R22 (flags), R20, R21, R10, R11, R13
;     Pseudocode       :
;***************************************************************************

FN_CHECK_AND_CLR_PTP_FWD_FLAG:
    QBBS   PTP_RX_IS_VLAN, R22, RX_IS_VLAN_BIT
    AND    R20.b0, R4.b2, 0xF          ;check PTP message ID type
    QBA    PTP_CHECK_UDP_RX_LINK_LOCAL
PTP_RX_IS_VLAN:
    AND    R20.b0, R5.b2, 0xF          ;check PTP message ID type
PTP_CHECK_UDP_RX_LINK_LOCAL:
    QBNE   PTP_CHECK_PDLY_RESP_MSG_ID, R20.b0, PTP_PDLY_REQ_MSG_ID
    SET    R22, R22, PTP_RELEASE_PORT_QUEUE_BIT
    QBA    EXIT_FN_CHECK_AND_CLR_PTP_FWD_FLAG
PTP_CHECK_PDLY_RESP_MSG_ID:
    QBNE   PTP_CHECK_FLW_UP_MSG_ID, R20.b0, PTP_PDLY_RSP_MSG_ID
    SET    R22, R22, PTP_RELEASE_PORT_QUEUE_BIT
PTP_CHECK_FLW_UP_MSG_ID:
    QBNE   EXIT_FN_CHECK_AND_CLR_PTP_FWD_FLAG, R20.b0, PTP_FOLLOW_UP_MSG_ID
    SET    R22, R22, PTP_RELEASE_PORT_QUEUE_BIT
EXIT_FN_CHECK_AND_CLR_PTP_FWD_FLAG:
    JMP    RCV_TEMP_REG_3.w2

;****************************************************************************
;
;     NAME             : FN_CHECK_AND_CLR_PTP_FWD_FLAG_L2
;     DESCRIPTION      : Check if the PTP-UDP packet is Follow Up
;                        so it's not forwarded. Called when 0-32 bytes are in R2-R9
;     Cycles           : TBD
;     Register Usage   : R22 (flags), R20, R21, R10, R11, R13
;     Pseudocode       :
;***************************************************************************

FN_CHECK_AND_CLR_PTP_FWD_FLAG_L2:
    QBBC    EXIT_FN_CHECK_AND_CLR_PTP_FWD_FLAG_L2, R22, RX_IS_PTP_BIT
    LDI     R1.b0, &R5.b2
    QBNE    NO_VLAN_RX_L2, R5.w0, VLAN_EtherType
    ADD     R1.b0, R1.b0, 4
NO_VLAN_RX_L2:
    MVIB    R20.b0, *R1.b0
    ;check for follow up frame
    QBNE    EXIT_FN_CHECK_AND_CLR_PTP_FWD_FLAG_L2, R20.b0, PTP_FOLLOW_UP_MSG_ID
    SET     R22, R22, PTP_RELEASE_PORT_QUEUE_BIT
EXIT_FN_CHECK_AND_CLR_PTP_FWD_FLAG_L2:
    JMP    RCV_TEMP_REG_3.w2


    .endif    ;____pn_gPtp_asm

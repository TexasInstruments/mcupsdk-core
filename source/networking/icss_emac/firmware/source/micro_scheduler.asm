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
; file:   micro_scheduler.asm
;
; brief:  Round-robin based micro_scheduler which controls program flow.
;
;
;  (C) Copyright 2017, Texas Instruments, Inc
;
;

;;///////////////////////////////////////////////////////
; Includes Section
;;///////////////////////////////////////////////////////
    .include "firmware_version.h"
    .include "micro_scheduler.h"

    .if $defined("ICSS_DUAL_EMAC_BUILD")
    .include "icss_emacSwitch.h"
    .endif

    .if $defined("ICSS_SWITCH_BUILD")
        .include "icss_switch.h"
    .endif

    .include "emac_MII_Rcv.h"
    .include "emac_MII_Xmt.h"
    .include "icss_defines.h"
    .include "icss_miirt_regs.h"
    .include "icss_iep_regs.h"
    .include "icss_macros.h"
    .include "icss_ecap_regs.h"
    .cdecls C,NOLIST
%{
#include "icss_rx_int_pacing_mmap.h"
%}

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

;.setcallreg CALL_REG
        .retain     ; Required for building .out with assembly file
        .retainrefs ; Required for building .out with assembly file

        .global  FN_RCV_FB
        .global  STATS_TASK
        .global  MII_TX_TASK
        .global  FN_RCV_NB
        .global  FN_RCV_LB
        .global  TASK_EXECUTION_FINISHED
        .if $defined("ICSS_SWITCH_BUILD")
        .global  COLLISION_TASK_EXEC
        .endif
    .if $defined("TTS")
        .global  FN_TTS_IEP_CFG_PRE_ICSS_REV2
        .global  FN_TTS_IEP_CFG_CLEAR
        .global  RET_TTS_IEP_CFG_CLEAR
        .global  RET_TTS_IEP_CFG_PRE
        .global  FN_TTS_IEP_CMPCFG_ARBITRATION
        .global  FN_TTS_IEP_CFG_PRE_ICSS_REV1
        .global  FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION
    .endif

	.if $defined("PTP")
        .global  FN_PTP_BACKGROUND_TASK
    .endif
        .sect ".data"
        .retain
    ;.include ".version"
        .sect    ".micro_scheduler"
        .global  micro_scheduler

micro_scheduler:
    JMP		fw_main

fw_main:
    ;Firmware Version: To store in shared memory. Can be used by application.
    LDI32   R0, ICSS_FIRMWARE_RELEASE_1
    LDI32   R1, ICSS_FIRMWARE_RELEASE_2
    LDI32   R2, ICSS_FIRMWARE_FEATURE_SET
    SBCO    &R0, PRU_DMEM_ADDR, ICSS_FIRMWARE_RELEASE_1_OFFSET, 12

    ZERO	&R0, 124		;Zero all registers
    LDI	R31 , 0

        ; reset the R31 bits for RXFIFO and TXFIFO
    M_SET_CMD	D_RESET_RXFIFO | D_RESET_TXFIFO		; reset RX and TX FIFO

    ; set the PACKET TX Allowed bit
    SET	R22 , R22 , PACKET_TX_ALLOWED

        ; EDIO pins are used the measure the latency of the firmware by connecting them to the oscilloscope
        ; and measure the toggle frequency. Pin value is set to low here
    .if $defined("PRU_DEBUG_EDIO")
    LDI EDIO_OFFSET, 0x310  ; HW debug pins - specific offset to register
    EDIO_OUT0_CLR
    .endif

        ;Configure the Task Table.  (Currently no used as the task table is very small.)
    LDI32	TASK_TABLE_ROW0, 0x00020100

    ;load storm prevention timer interval
    MOV32   RCV_TEMP_REG_1, SP_COUNTER_UPDATE_INTERVAL_DEFAULT
    LDI     RCV_TEMP_REG_2, SP_COUNTER_UPDATE_INTERVAL_OFFSET
    SBCO    &RCV_TEMP_REG_1, PRU_DMEM_ADDR, RCV_TEMP_REG_2, 4

START_THE_MS:

    .if $defined("TTS")
    .if $defined("ICSS_REV1")
    ;Set IEP wraparound to 1s. This will be standard for 32-bit IEP in EMAC. This is done for setting up TTS
    M_TTS_SET_CMP0_IEP_WRAP
    .endif	;ICSS_REV1
    .endif	;TTS

    ; Load Address of R19 Register, Begin with RX TASK.
    LDI	CURRENT_TASK_POINTER, RX_TASK_POINTER

    ; Loop over the Task's List
LOOP_OVER_TASKS:

    ;Check if time triggered send is enabled.
    .if $defined("TTS")
    M_CHECK_TTS_ENABLE
RET_TTS_IEP_CFG_CLEAR:
    .endif	;TTS

    ; check for RX EOF condition i.e. check the R31 register corresponding bit based on the ICSS REVISION.
        ; if the recieption is still going on, continue to check Rx active.
    .if $defined("ICSS_REV1")
    M_MS_RX_EOF_CHECK_ICSS_REV1
    .endif
    .if $defined("ICSS_REV2")
    M_MS_RX_EOF_CHECK_ICSS_REV2
    .endif

    ; Call the RCV_LB which resets the Rx L2 Fifo and clear all confiquration.
    JAL	CALL_REG, FN_RCV_LB

    ;Check for SFD (START FRAME DETECTED) event
check_sfd:
    XIN	RX_L2_BANK0_ID, &R2, (4 * 17)	; Load Bank 0 values into R2 - R18

    QBBC	EXECUTE_NEXT_TASK, R10, 5       ; check if SOF is low, then jump to next task else continue
    QBGT	SKIP_RCV_TASK, R18_RCV_BYTECOUNT, 18        ; check if no. of bytes recieved is greater than 18 then
    JAL	CALL_REG, FN_RCV_FB	                    ; jump to FN_RCV_FB else continue
SKIP_RCV_TASK:
        ; increment the task pointer.
    QBNE	EXECUTE_NEXT_TASK, CURRENT_TASK_POINTER, RX_TASK_POINTER
    ADD	CURRENT_TASK_POINTER, CURRENT_TASK_POINTER, 1

Check_RCV_Active:
    QBBS	EXECUTE_NEXT_TASK, R23, 1       ; if Rx is already active then continue to next task else
    QBA     check_sfd                       ; again check for SOF

EXECUTE_NEXT_TASK:
    MVIB	TEMP_REG_3.b0, *CURRENT_TASK_POINTER	    ; save the value of last task pointer
    QBEQ	TASK_LIST_WRAPS, CURRENT_TASK_POINTER, BG_TASK_POINTER
    ADD	CURRENT_TASK_POINTER, CURRENT_TASK_POINTER, 1	    ;increment the Task pointer
    JMP	NO_TASK_LIST_WRAP

TASK_LIST_WRAPS:
    LDI	CURRENT_TASK_POINTER, RX_TASK_POINTER	; modify the Task pointer to Rx.

NO_TASK_LIST_WRAP:
        ;based on value of last task pointer jump to task routine
    QBEQ	MII_RX_Task, TEMP_REG_3.b0, 0x00
    QBEQ	MII_TX_Task_inter, TEMP_REG_3.b0, 0x01
    QBEQ	BACKGROUND_TASK, TEMP_REG_3.b0, 0x02

;This should never happen
ERROR:
    HALT

TASK_EXECUTION_FINISHED:
    .if $defined("TTS")
    .if $defined("ICSS_REV1")
    M_TTS_CMP0_CMP7_CHECK       ;check if IEP timer event has occured in order to trigger TTS packet
    .endif
    M_TTS_CMP3_CMP4_CHECK       ;check if IEP timer event has occured in order to trigger TTS packet
    .endif	;TTS

    QBBS	LOOP_OVER_TASKS, R22, PACKET_TX_ALLOWED    ;check if Tx_allowed bit is set, then jump to loop over task else continue

    .if $defined("ICSS_REV1")
    M_MS_TEF_ICSS_REV1          ; perform the task execution finish job i.e. clear interrupt, error etc
    .endif
    .if $defined("ICSS_REV2")
    M_MS_TEF_ICSS_REV2          ; perform the task execution finish job i.e. clear interrupt, error etc
    .endif

    .if $defined("TTS")
    ;TX_ENABLE bit is auto cleared in TX_EN_MODE.
    ;Need to set TX_ENABLE bit if TTS is enabled.
    M_SET_MIIRT_TXCFG_TX_ENABLE
    .endif	;TTS

    SET	R22 , R22 , PACKET_TX_ALLOWED          ; set Tx_allowed bit to indicate ongoing active transmit
    JMP	LOOP_OVER_TASKS


;;///////// Receive Task ////////////////////////
MII_RX_Task:

        ; EDIO pins are used the measure the latency of the firmware by connecting them to the oscilloscope
        ; and measure the toggle frequency. Pin value is set to high here
        .if $defined("PRU_MEAS_RXTASK")
    EDIO_OUT0_SET
    .endif

        JMP		FN_RCV_NB

;;/////////   Transmit Task	/////////////////
MII_TX_Task_inter:

        ; EDIO pins are used the measure the latency of the firmware by connecting them to the oscilloscope
        ; and measure the toggle frequency. Pin value is set to high here
        .if $defined("PRU_MEAS_TXTASK")
    EDIO_OUT0_SET
    .endif

        QBBC	TASK_EXECUTION_FINISHED, R22, PACKET_TX_ALLOWED    ;check if Tx_allowed bit is set, then jump to next task else continue

    .if $defined("TTS")
    ;Need to check essential conditions before TTS configuration.
    M_TTS_CFG_CONDITIONAL_CONSTRUCT
RET_TTS_IEP_CFG_PRE:
    .endif	;TTS

    JAL	CALL_REG, MII_TX_TASK       ; jump to Tx task
    JMP	TASK_EXECUTION_FINISHED

;;////////   Stats Task
BACKGROUND_TASK:
    .if $defined("ICSS_SWITCH_BUILD")
; check for link status on opposite port and half duplex status
    LDI	R20.w2 , PORT_STATUS_OFFSET
    LBCO	&R20.w0, PRU_CROSS_DMEM, R20.w2, 1
    CLR	R22 , R22 , 10
    CLR	R22 , R22 , 23
    QBBC	MS_SCH_OPPOSITE_PORT_LINK_DOWN, R20, 0	 ;replaced: QBBC	MS_SCH_OPPOSITE_PORT_LINK_DOWN, R20.PORT_LINK_UP
    SET	R22 , R22 , 10
MS_SCH_OPPOSITE_PORT_LINK_DOWN:
    QBBC	MS_SCH_OPP_PORT_FULL_DUPLEX, R20, 1	 ;replaced: QBBC    MS_SCH_OPP_PORT_FULL_DUPLEX, R20.PORT_IS_HD
    SET	R22 , R22 , 23
MS_SCH_OPP_PORT_FULL_DUPLEX:
    .endif	;ICSS_SWITCH_BUILD

    ; Check INTR_PAC_STATUS_OFFSET for Interrupt Pacing logic status
    .if $defined(PRU0)
    LDI     RCV_TEMP_REG_3.w0, INTR_PAC_STATUS_OFFSET_PRU0
    .else
    LDI     RCV_TEMP_REG_3.w0, INTR_PAC_STATUS_OFFSET_PRU1
    .endif  ;   PRU0
    LBCO    &RCV_TEMP_REG_3.b0, ICSS_SHARED_CONST, RCV_TEMP_REG_3.w0, 1
    QBEQ    INTR_PAC_DIS, RCV_TEMP_REG_3.b0, INTR_PAC_DIS_ADP_LGC_DIS

    ; Rx interrupt pacing
    ; check if timer has elapsed by comparing the previous TS & current TS from eCAP TSCNT
    ;   if timer not elapsed => goto TIMER_NOT_ELAPSED
    ;   else => timer has elapsed
    ;       recalibrate timer window
    ;       if R22 bit INTR_TO_HOST_PENDING_RX is SET
    ;           => fire interrupt
    ;           => clear INTR_TO_HOST_PENDING_RX
    ;           => increment LRE_INTR_TMR_EXP i.e. count no of interrupts raised due to timer expiry
    ;       else => goto NO_INTR_PENDING
    .if $defined(PRU0)
    LDI     RCV_TEMP_REG_3.w0, INTR_PAC_PREV_TS_OFFSET_PRU0
    .else
    LDI     RCV_TEMP_REG_3.w0, INTR_PAC_PREV_TS_OFFSET_PRU1
    .endif  ;   PRU0
    LBCO    &RCV_TEMP_REG_1, ICSS_SHARED_CONST, RCV_TEMP_REG_3.w0, 8
;----------------------------------------
;               previous TS             | R20 | RCV_TEMP_REG_1
;----------------------------------------
;            timer expiry value         | R21 | RCV_TEMP_REG_2
;----------------------------------------
;INTR_PAC_PREV_TS_OFFSET_PRU0/1|        | R13 | RCV_TEMP_REG_3
;----------------------------------------
    LBCO    &RCV_TEMP_REG_4, ICSS_ECAP_CONST, ICSS_eCAP_TSCNT, 4
;----------------------------------------
;             eCAP TSCNT value          | R12 | RCV_TEMP_REG_4
;----------------------------------------
    QBLT    TS_DIFF, RCV_TEMP_REG_4, RCV_TEMP_REG_1
    ; previous TS >= eCAP TSCNT value
    ; => wrap around handling
    SUB     R2, RCV_TEMP_REG_1, RCV_TEMP_REG_4
    FILL    &R3, 4  ; 32 bit wraparound value i.e. 0xFFFFFFFF
    SUB     R2, R3, R2
    QBA     TS_DIFF_DONE

TS_DIFF:
    ; previous TS < eCAP TSCNT value
    SUB     R2, RCV_TEMP_REG_4, RCV_TEMP_REG_1
TS_DIFF_DONE:
;----------------------------------------
;   diff(eCAP TSCNT value,previous TS)  | R2
;----------------------------------------
;            timer expiry value         | R21 | RCV_TEMP_REG_2
;----------------------------------------
;INTR_PAC_PREV_TS_OFFSET_PRU0/1|        | R13 | RCV_TEMP_REG_3
;----------------------------------------
;             eCAP TSCNT value          | R12 | RCV_TEMP_REG_4
;----------------------------------------
    QBLT  TIMER_NOT_ELAPSED, RCV_TEMP_REG_2, R2
    ; timer expired, recalibrate timer window
    ; => set previous TS to present TS i.e. previous TS = eCAP TSCNT value
    SBCO    &RCV_TEMP_REG_4, ICSS_SHARED_CONST, RCV_TEMP_REG_3.w0, 4

    ; check if interrupt to host INTR_TO_HOST_PENDING_RX is pending
    ; if INTR_TO_HOST_PENDING_RX is CLR => jump to NO_INTR_PENDING
    ; else fire interrupt
    QBBC    NO_INTR_PENDING, R22, INTR_TO_HOST_PENDING_RX

    ; fire interrupt
    .if $defined(PRU0)
    LDI     R31, 0x24
    .else
    LDI     R31, 0x25
    .endif ; PRU0

    ; clear INTR_TO_HOST_PENDING_RX
    CLR     R22, R22, INTR_TO_HOST_PENDING_RX

    .if $defined(EDIO_INTR_PAC)
    ; EDIO output J7 pin 3 AM572x
    LDI32 RCV_TEMP_REG_1, 0x4B22E310
    LDI RCV_TEMP_REG_2, 0x0010
    SBBO &RCV_TEMP_REG_2.b0, RCV_TEMP_REG_1, 0, 1
    SBBO &RCV_TEMP_REG_2.b1, RCV_TEMP_REG_1, 0, 1
    .endif  ; EDIO_INTR_PAC

NO_INTR_PENDING:
TIMER_NOT_ELAPSED:
INTR_PAC_DIS:

    .if $defined(PTP)
    .if $defined(PRU0)
    JAL     CALL_REG, FN_PTP_BACKGROUND_TASK
    .endif ;PRU0
    .endif ;PTP

STORM_PREVENTION_COUNTER_UPDATE_CHECK:
    ;current IEP counter value
    LBCO    &RCV_TEMP_REG_2, IEP_CONST, IEP_COUNTER_OFFSET, 4
    AND     RCV_TEMP_REG_1, RCV_TEMP_REG_2, RCV_TEMP_REG_2
    LDI     RCV_TEMP_REG_3, SP_UPDATE_TIMESTAMP_OFFSET
    LBCO    &RCV_TEMP_REG_4, PRU_DMEM_ADDR, RCV_TEMP_REG_3 , 4
    QBGE    IEP_WRAPAROUND_HAPPENED, RCV_TEMP_REG_2, RCV_TEMP_REG_4
    SUB     RCV_TEMP_REG_2, RCV_TEMP_REG_2, RCV_TEMP_REG_4
    QBA     CHECK_100_MS_PASSED
IEP_WRAPAROUND_HAPPENED:
    ;check compare enable and see if wraparound is enabled
    LBCO    &RCV_TEMP_REG_3, IEP_CONST, IEP_CMP_CFG_REG, 1
    QBBC    IEP_CMP0_NOT_ENABLED_HERE, RCV_TEMP_REG_3, 1
    LBCO    &RCV_TEMP_REG_3, IEP_CONST, IEP_CMP0_REG, 4
    QBA     CALCULATE_PASSED_TIME_FOR_IEP_WRAPAROUND
IEP_CMP0_NOT_ENABLED_HERE:
    FILL    &RCV_TEMP_REG_3, 4
CALCULATE_PASSED_TIME_FOR_IEP_WRAPAROUND:
    SUB     RCV_TEMP_REG_4, RCV_TEMP_REG_3, RCV_TEMP_REG_4
    ADD     RCV_TEMP_REG_2, RCV_TEMP_REG_4, RCV_TEMP_REG_2
CHECK_100_MS_PASSED:
    ;check if 100ms has passed
    LDI     RCV_TEMP_REG_3, SP_COUNTER_UPDATE_INTERVAL_OFFSET
    LBCO    &RCV_TEMP_REG_3, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    QBGE    SKIP_STORM_PREVENTION_COUNTER_UPDATE, RCV_TEMP_REG_2, RCV_TEMP_REG_3
    LDI     RCV_TEMP_REG_3, SP_UPDATE_TIMESTAMP_OFFSET
    SBCO    &RCV_TEMP_REG_1, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
;for BC
    LDI     RCV_TEMP_REG_3, STORM_PREVENTION_OFFSET_BC_DRIVER
    LBCO    &RCV_TEMP_REG_4, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    LDI     RCV_TEMP_REG_3, STORM_PREVENTION_OFFSET_BC
    SBCO    &RCV_TEMP_REG_4, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
;for MC
    LDI     RCV_TEMP_REG_3, STORM_PREVENTION_OFFSET_MC_DRIVER
    LBCO    &RCV_TEMP_REG_4, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    LDI     RCV_TEMP_REG_3, STORM_PREVENTION_OFFSET_MC
    SBCO    &RCV_TEMP_REG_4, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
;for UC
    LDI     RCV_TEMP_REG_3, STORM_PREVENTION_OFFSET_UC_DRIVER
    LBCO    &RCV_TEMP_REG_4, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
    LDI     RCV_TEMP_REG_3, STORM_PREVENTION_OFFSET_UC
    SBCO    &RCV_TEMP_REG_4, PRU_DMEM_ADDR, RCV_TEMP_REG_3, 4
SKIP_STORM_PREVENTION_COUNTER_UPDATE:

    QBBS	stat_task_called, R23, TX_STAT_PEND	 ;check if TX_STAT_PEND is set then jump to stat task
    QBBC	coll_task_called, R23, RX_STAT_PEND	 ;check if RX_STAT_PEND is clear then jump to next task else jump to stat task
stat_task_called:
    JAL	CALL_REG, STATS_TASK
coll_task_called:
    .if    $defined("TWO_PORT_CFG")
    JAL	CALL_REG, COLLISION_TASK_EXEC
    .endif ;TWO_PORT_CFG
    JMP	TASK_EXECUTION_FINISHED

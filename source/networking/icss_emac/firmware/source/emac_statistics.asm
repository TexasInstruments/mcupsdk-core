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
; file:   emac_statistics.asm
;
; brief:  Statistics task.
;
;
;  (C) Copyright 2017, Texas Instruments, Inc
;
;

    .if $defined("ICSS_DUAL_EMAC_BUILD")
    .include "icss_emacSwitch.h"
    .endif
    .if $defined("ICSS_SWITCH_BUILD")
    .include "icss_switch.h"
    .endif
    .include "micro_scheduler.h"
    .include "emac_MII_Rcv.h"
    .include "emac_MII_Xmt.h"
    .include "icss_miirt_regs.h"

    .global  STATS_TASK


    .asg    R8, STATS_REG    ; stores current stats counter offset

    .asg    R9, RX_STATS_REG
    .asg    R5, PACKET_SIZE
    .asg    R6.b0, RX_PACKET_SIZE
    .if    !$defined("TWO_PORT_CFG")
    .asg    R9.w0, RX_PKT_SIZE_REG

    .endif ;TWO_PORT_CFG
RX_TX_BIN_DIFF    .set    0x20

    .asg    R3, STAT_TEMP_REG
    .asg    R2, INTC_RAW_OFFSET

RX_TX_BINNING_OFFSET    .set    0x18

;-----------------------------------
;CYCLE BUDGET :
;TX_TASK ~30 cycles
;RX_TASK (store and forward) ~30 cycles
;RX_TASK (cut through) ~57 cycles
;-----------------------------------


;-----------------------------------
; Macro Name: INCREMENT_STAT_COUNTER
; Description: Read stat value from memory, increment by 1 and write back
; Input Parameters: none
; Output Parameters: none
;-----------------------------------
INCREMENT_STAT_COUNTER    .macro
    LBCO    &R2, PRU_DMEM_ADDR, STATS_REG, 4
    ADD    R2, R2, 1
    SBCO    &R2, PRU_DMEM_ADDR, STATS_REG, 4
    .endm

    .if    $defined("TWO_PORT_CFG")
INCREMENT_CROSS_STAT_COUNTER    .macro
    LBCO    &R2, PRU_CROSS_DMEM, STATS_REG, 4
    ADD    R2, R2, 1
    SBCO    &R2, PRU_CROSS_DMEM, STATS_REG, 4
    .endm
    .endif ;TWO_PORT_CFG

; TX_STAT_PEND - Set by TX_LB, cleared by TX_STATS_TASK
; RX_STAT_PEND - Set by RX_LB, cleared by RX_STATS_TASK

STATS_TASK:
    ;check if Tx task statistics is pending
    AND    L1_CALL_REG , CALL_REG , CALL_REG
    QBBC    RX_STAT_TASK, R23, 2

    LDI    R4 , STATISTICS_OFFSET
    LDI    SHIFT_REG, SHIFT_NONE
    .if $defined("PRU0")
    XIN    BANK1, &MII_TX_CONTEXT, 8
    .else
    XIN    BANK2, &MII_TX_CONTEXT, 8
    .endif

    ;add crc packet to the packet length
    ADD    Packet_Length, Packet_Length, 4


TX_GOOD_FRAMES:
    ;Read stat value from memory, increment by packet_length and write back
    ADD    STATS_REG, R4, TX_BYTE_CNT_OFFSET
    .if    $defined("TWO_PORT_CFG")
    LBCO    &R2, PRU_CROSS_DMEM, STATS_REG, 4
    .else
    LBCO    &R2, PRU_DMEM_ADDR, STATS_REG, 4
    .endif ;TWO_PORT_CFG
    ADD    R2, R2, Packet_Length
    .if    $defined("TWO_PORT_CFG")
    SBCO    &R2, PRU_CROSS_DMEM, STATS_REG, 4
    .else
    SBCO    &R2, PRU_DMEM_ADDR, STATS_REG, 4
    .endif ;TWO_PORT_CFG

TX_64_FRAME:
    ;Read stat value from memory, increment by packet_length for frame length less than 64
    QBLT    TX_65_127_BYTE_FRAME, Packet_Length, 64
    ADD    STATS_REG, R4, TX_64_BYTE_FRAME_OFFSET
    JMP    ADD_TX_BINNING_COUNTER

    ;Read stat value from memory, increment by packet_length for frame length 65 < packet size < 127
TX_65_127_BYTE_FRAME:
    QBLT    TX_128_255_BYTE_FRAME, Packet_Length, 127
    ADD    STATS_REG, R4, TX_65_127_BYTE_FRAME_OFFSET
    JMP    ADD_TX_BINNING_COUNTER

    ;Read stat value from memory, increment by packet_length for frame length 128 < packet size < 255
TX_128_255_BYTE_FRAME:
    QBLT    TX_256_511_BYTE_FRAME, Packet_Length, 255
    ADD    STATS_REG, R4, TX_128_255_BYTE_FRAME_OFFSET
    JMP    ADD_TX_BINNING_COUNTER

    ;Read stat value from memory, increment by packet_length for frame length 256 < packet size < 511
TX_256_511_BYTE_FRAME:
    LDI    STAT_TEMP_REG, 511
    QBLT    TX_512_1023_BYTE_FRAME, Packet_Length, STAT_TEMP_REG
    ADD    STATS_REG, R4, TX_256_511_BYTE_FRAME_OFFSET
    JMP    ADD_TX_BINNING_COUNTER

    ;Read stat value from memory, increment by packet_length for frame length 512 < packet size < 1023
TX_512_1023_BYTE_FRAME:
    LDI    STAT_TEMP_REG, 1023
    QBLT    TX_1024_MAX_BYTE_FRAME, Packet_Length, STAT_TEMP_REG
    ADD    STATS_REG, R4, TX_512_1023_BYTE_FRAME_OFFSET
    JMP    ADD_TX_BINNING_COUNTER

    ;Read stat value from memory, increment by packet_length for frame length > 1024
TX_1024_MAX_BYTE_FRAME:
    ADD    STATS_REG, R4, TX_1024_MAX_BYTE_FRAME_OFFSET

ADD_TX_BINNING_COUNTER:
    .if    $defined("TWO_PORT_CFG")
    INCREMENT_CROSS_STAT_COUNTER
    .else
    INCREMENT_STAT_COUNTER
    .endif ;TWO_PORT_CFG

COUNT_TX_BC_FRAMES:
    QBBC    COUNT_TX_MC_FRAMES, R22 , TX_BC_FRAME
    ADD    STATS_REG, R4, TX_BC_FRAMES_OFFSET
    CLR    R22 , R22 , TX_BC_FRAME
    QBA    COUNT_TX_GOOD_FRAMES

COUNT_TX_MC_FRAMES:
    QBBC    COUNT_TX_UC_FRAMES, R22 , TX_MC_FRAME
    ADD    STATS_REG, R4, TX_MC_FRAMES_OFFSET
    CLR    R22 , R22 , TX_MC_FRAME
    QBA    COUNT_TX_GOOD_FRAMES

COUNT_TX_UC_FRAMES:
    ADD    STATS_REG, R4, TX_UC_FRAMES_OFFSET

COUNT_TX_GOOD_FRAMES:
    .if    $defined("TWO_PORT_CFG")
    INCREMENT_CROSS_STAT_COUNTER
    .else
    INCREMENT_STAT_COUNTER
    .endif ;TWO_PORT_CFG

TX_STAT_CLR:
    CLR    R23 , R23 , 2

RX_STAT_TASK:
    ;check if Rx task statistics is pending
    QBBC    STAT_DONE, R23, 3
    .if    $defined("TWO_PORT_CFG")
    QBBS    SHIFT_FOR_FWD, R22, 24    ;RX_FWD_FLAG

RX_PKT_SIZE:
    LDI    R0.b0, 0 ;  This is to make sure that no offset is added
    .if    $defined("PRU0")
    XIN    BANK1, &MII_RCV, $sizeof(MII_RCV)
    .else
    XIN    BANK2, &MII_RCV, $sizeof(MII_RCV)
    .endif
    QBBC    READ_BYTE_CNT_FOR_SIZE, MII_RCV.tx_flags, cut_through_flag_shift
    AND MII_RCV.byte_cntr , CUT_THROUGH_BYTE_CNT , CUT_THROUGH_BYTE_CNT
READ_BYTE_CNT_FOR_SIZE:
    QBA    RX_GOOD_FRAMES

SHIFT_FOR_FWD:
    CLR    R22, R22, RX_FWD_FLAG

    .if    $defined("PRU0")
    LDI    R0.b0, SHIFT_R14_TO_R0
    XIN    BANK0, &MII_RCV_PORT, $sizeof(MII_RCV_PORT)
    .else
    LDI    R0.b0, SHIFT_R14_TO_R4
    XIN    BANK0, &MII_RCV_PORT, $sizeof(MII_RCV_PORT)
    .endif
    AND MII_RCV.byte_cntr , MII_RCV_PORT.byte_cntr , MII_RCV_PORT.byte_cntr	;Warning: converted from MOV
    .else
    ;load the size
    LDI    R2 , RX_PKT_SIZE_OFFSET
    LBCO    &RX_PKT_SIZE_REG, PRU_DMEM_ADDR, R2, 2

    .endif ;TWO_PORT_CFG

RX_GOOD_FRAMES:
    ;Read stat value from memory, increment by packet_length and write back
    LDI    R4 , STATISTICS_OFFSET
    QBBC    COUNT_RX_MC_FRAMES, R22 , RX_BC_FRAME
    ADD    STATS_REG, R4, RX_BC_FRAMES_OFFSET
    QBA    RX_INCREMENT_COUNTER

COUNT_RX_MC_FRAMES:
    QBBC    COUNT_RX_UC_FRAMES, R22 , RX_MC_FRAME
    ADD    STATS_REG, R4, RX_MC_FRAMES_OFFSET
    QBA    RX_INCREMENT_COUNTER

COUNT_RX_UC_FRAMES:
    ADD    STATS_REG, R4, RX_UC_FRAMES_OFFSET

RX_INCREMENT_COUNTER:
    INCREMENT_STAT_COUNTER

;Add byte count
    ADD    STATS_REG, R4, RX_BYTE_CNT_OFFSET
    LBCO    &R2, PRU_DMEM_ADDR, STATS_REG, 4
    .if    $defined("TWO_PORT_CFG")
    ADD    R2, R2,  MII_RCV.byte_cntr
    .else
    ADD    R2, R2, RX_PKT_SIZE_REG
    .endif ;TWO_PORT_CFG
    SBCO    &R2, PRU_DMEM_ADDR, STATS_REG, 4

RX_64_FRAME:
    ;Read stat value from memory, increment by packet_length for frame length less than 64
    QBLT    RX_65_127_BYTE_FRAME, MII_RCV.byte_cntr, 64
    ADD    STATS_REG, R4, RX_64_BYTE_FRAME_OFFSET
    JMP    ADD_RX_BINNING_COUNTER

    ;Read stat value from memory, increment by packet_length for frame length 65 < packet size < 127
RX_65_127_BYTE_FRAME:
    .if    $defined("TWO_PORT_CFG")
    QBLT    RX_128_255_BYTE_FRAME, MII_RCV.byte_cntr, 127
    .else
    QBLT    RX_128_255_BYTE_FRAME, RX_PKT_SIZE_REG, 127
    .endif ;TWO_PORT_CFG
    ADD    STATS_REG, R4, RX_65_127_BYTE_FRAME_OFFSET
    JMP    ADD_RX_BINNING_COUNTER

    ;Read stat value from memory, increment by packet_length for frame length 128 < packet size < 255
RX_128_255_BYTE_FRAME:
    .if    $defined("TWO_PORT_CFG")
    QBLT    RX_256_511_BYTE_FRAME, MII_RCV.byte_cntr, 255
    .else
    QBLT    RX_256_511_BYTE_FRAME, RX_PKT_SIZE_REG, 255

    .endif ;TWO_PORT_CFG
    ADD    STATS_REG, R4, RX_128_255_BYTE_FRAME_OFFSET
    JMP    ADD_RX_BINNING_COUNTER

    ;Read stat value from memory, increment by packet_length for frame length 256 < packet size < 511
RX_256_511_BYTE_FRAME:
    LDI    STAT_TEMP_REG, 511
    .if    $defined("TWO_PORT_CFG")
    QBLT    RX_512_1023_BYTE_FRAME, MII_RCV.byte_cntr, STAT_TEMP_REG
    .else
    QBLT    RX_512_1023_BYTE_FRAME, RX_PKT_SIZE_REG, STAT_TEMP_REG
    .endif ;TWO_PORT_CFG
    ADD    STATS_REG, R4, RX_256_511_BYTE_FRAME_OFFSET
    JMP    ADD_RX_BINNING_COUNTER

    ;Read stat value from memory, increment by packet_length for frame length 512 < packet size < 1023
RX_512_1023_BYTE_FRAME:
    LDI    STAT_TEMP_REG, 1023
    .if    $defined("TWO_PORT_CFG")
    QBLT    RX_1024_MAX_BYTE_FRAME, MII_RCV.byte_cntr, STAT_TEMP_REG
    .else
    QBLT    RX_1024_MAX_BYTE_FRAME, RX_PKT_SIZE_REG, STAT_TEMP_REG
    .endif ;TWO_PORT_CFG
    ADD    STATS_REG, R4, RX_512_1023_BYTE_FRAME_OFFSET
    JMP    ADD_RX_BINNING_COUNTER

    ; Read stat value from memory, increment by packet_length for frame length > 1024
RX_1024_MAX_BYTE_FRAME:
    ADD    STATS_REG, R4, RX_1024_MAX_BYTE_FRAME_OFFSET

ADD_RX_BINNING_COUNTER:
    INCREMENT_STAT_COUNTER
    AND    STAT_TEMP_REG , STATS_REG , STATS_REG
    .if    $defined("TWO_PORT_CFG")
CUT_THROUGH_PACKET:
    QBBC    RX_STAT_CLR, MII_RCV.tx_flags, cut_through_flag_shift
RX_TX_FRAMES: ;increment TX counters for cut through


    QBBC    COUNT_RX_TX_MC_FRAMES, R22, 28    ;RX_BC_FRAME
    ADD    STATS_REG, R4, TX_BC_FRAMES_OFFSET
    QBA    COUNT_RX_TX_GOOD_FRAMES

COUNT_RX_TX_MC_FRAMES:
    QBBC    COUNT_RX_TX_UC_FRAMES, R22, 27    ;RX_MC_FRAME
    ADD    STATS_REG, R4, TX_MC_FRAMES_OFFSET
    QBA    COUNT_RX_TX_GOOD_FRAMES

COUNT_RX_TX_UC_FRAMES:
    ADD    STATS_REG, R4, TX_UC_FRAMES_OFFSET

COUNT_RX_TX_GOOD_FRAMES:
    INCREMENT_CROSS_STAT_COUNTER

COUNT_RX_TX_OCTETS:
    ADD    STATS_REG, R4, TX_BYTE_CNT_OFFSET

    LBCO    &R2, PRU_CROSS_DMEM, STATS_REG, 4
    ADD    R2, R2,  MII_RCV.byte_cntr
    SBCO    &R2, PRU_CROSS_DMEM, STATS_REG, 4

;since for cut through the binning counter is same for rx and tx
;we use this to save code size. We need not do tx binning again
;Just subtract the offset from rx binning counter to get tx binning counter
    SUB    STATS_REG, STAT_TEMP_REG, RX_TX_BINNING_OFFSET
    INCREMENT_CROSS_STAT_COUNTER
    .endif ;TWO_PORT_CFG

RX_STAT_CLR:
    CLR    R22 , R22 , RX_BC_FRAME
    CLR    R22 , R22 , RX_MC_FRAME
    CLR    R23 , R23 , 3
    QBA    STAT_DONE

CLEAR_AND_EXIT_STATS:

STAT_DONE:
    AND    CALL_REG , L1_CALL_REG , L1_CALL_REG
    JMP    CALL_REG
    ;JMP    TASK_EXECUTION_FINISHED

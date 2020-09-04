; Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; Redistributions of source code must retain the above copyright
; notice, this list of conditions and the following disclaimer.
;
; Redistributions in binary form must reproduce the above copyright
; notice, this list of conditions and the following disclaimer in the
; documentation and/or other materials provided with the
; distribution.
;
; Neither the name of Texas Instruments Incorporated nor the names of
; its contributors may be used to endorse or promote products derived
; from this software without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
; OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
; SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
; LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

; file:     icss_xfer_defines.inc
;
; brief:    XFER ID defines
;           Includes:
;           1.
;

    .if !$isdefed("____icss_xfer_defines_h")
____icss_xfer_defines_h   .set    1


;--------------------------------------------------------------------------;
;   Constants: XFER IDS
;
;   MAC_XID             - XID for Multiplier (with optional accumulation)
;   CRC_XID             - XID for CRC8, CRC16 and CRC32 accelerator BS
;   SUM32_RESULTS_XID   - XID for SUM32 results (RTU only)
;   PRU_STICH_FIFO_XID  - XID for Stich FIFO used for preemption (PRU only)
;   RTU_QUEUE_PTR_XID   - XID for Queue acceleration widget (RTU only)
;   TX_PRU_SPAD_B0_XID  - XID for TXPRU Scratch PAD Bank 0 (not shared/ no remap)
;   RTU_SPAD_B0_XID     - XID for RTU Scratch PAD Bank 0
;   RTU_SPAD_B1_XID     - XID for RTU Scratch PAD Bank 1
;   RTU_SPAD_B2_XID     - XID for RTU Scratch PAD Bank 2
;   PRU_SPAD_B0_XID     - XID for PRU Scratch PAD Bank 0
;   PRU_SPAD_B1_XID     - XID for PRU Scratch PAD Bank 1
;   PRU_SPAD_B2_XID     - XID for PRU Scratch PAD Bank 2
;   PRU_RTU_SPAD_XID    - XID for PRU-RTU 32 byte Scratch PAD
;   RXL2_SIDEA_XID      - XID for RXL2 Bank A
;   RXL2_SIDEB_XID      - XID for RXL2 Bank B
;   RXL2_FILTER_XID     - XID for RXL2 Filter
;   RTU_BSRAM_XID       - XID for RTU Broadside RAM
;   FDB_DATA_XID        - XID for FDB Data
;   FDB_ADDR_XID        - XID for FDB Address & Config
;   FDB_GEN_PATTERN_XID - XID for General Pattern Config
;   FDB_P0_RES_XID      - XID for FDB Physical Port1 results
;   FDB_P1_RES_XID      - XID for FDB Physical Port2 results
;   FDB_HOST_RES_XID    - XID for FDB Host Port results
;   FDB_GEN_RES_XID     -  XID for General compare Results
;   RTU_BSRAM_SUM32_XID - XID for RTU Broadside RAM + SUM32
;   RTU_SUM32_XID       - XID for RTU SUM32 widget
;   TXL2_XID            - XID for TXL2 FIFO
;   PRU_BSRAM_XID       - XID for PRU Broadside RAM
;   PRU_BSRAM_SUM32_XID - XID for PRU Broadside RAM + SUM32
;   PRU_SUM32_XID       - XID for PRU SUM32 widget
;   XFR2PSI_STAT_XID    - XID for XFR2PSI Status
;   XFR2PSI_RD0_XID     - XID for XFR2PSI Read Widget0
;   XFR2PSI_WR0_XID     - XID for XFR2PSI Write Widget0
;   XFR2PSI_RD1_XID     - XID for XFR2PSI Read Widget1
;   XFR2PSI_RD2_XID     - XID for XFR2PSI Read Widget2
;   XFR2PSI_RD3_XID     - XID for XFR2PSI Read Widget3
;   RTU_XFR2PSI_SHARE_PRU0_XID - XID for sharing PRU0 XFR2PSI widget with RTU
;   RTU_XFR2PSI_SHARE_PRU1_XID - XID for sharing PRU1 XFR2PSI widget with RTU
;   XFR2VBUSP_RD0_XID   - XID for XFR2VBUS Read Widget0 (for external memory access)
;   XFR2VBUSP_RD1_XID   - XID for XFR2VBUS Read Widget1 (for external memory access)
;   XFR2VBUSP_RD2_XID   - XID for XFR2VBUS Read Widget2 (for external memory access) (TXPRU only)
;   XFR2VBUSP_WR0_XID   - XID for XFR2VBUS Write Widget0 (for external memory access)
;   XFR2VBUSP_WR1_XID   - XID for XFR2VBUS Write Widget1 (for external memory access)
;   XFR2TR_MASTER_XID   - XID for Source TR pool
;   XFR2TR_RING_XID     - XID for Destination TR ring
;   XFR2TR_SUBMIT_XID   - XID for Trigger submit from Source TR pool to Destination TR ring
;   INT_SPIN_XID        - XID for Local spinlock
;   EXT0_SPIN_XID       - XID for Spinlock in other ICSSG
;   EXT1_SPIN_XID       - XID for Spinlock in other ICSSG
;   BSWAP_XID           - XID for Endian swap
;   BSWAP_4_8_XID       - XID for Pivot swap (R2:R5 <=> R6:R9)
;   BSWAP_4_16_XID      - XID for Pivot swap (R2:R5 <=> R14:R17 && R6:R9 <=> R10:R13)
;   TM_YIELD_XID        - XID for Task manager yield xin instruction
;   TM_SAVE_XID         - PRU_SPAD_B0_XID for PRU0 and PRU_SPAD_B1_XID for PRU1
;   QUEUE_EMPTY_XID     - XID for QM empty status
;
;--------------------------------------------------------------------------;
MAC_XID             .set 0
CRC_XID             .set 1
SUM32_RESULTS_XID   .set 2 ; RTU only ?
PRU_STICH_FIFO_XID  .set 8
RTU_QUEUE_PTR_XID   .set 9
TX_PRU_SPAD_B0_XID  .set 10
RTU_SPAD_B0_XID     .set 10
RTU_SPAD_B1_XID     .set 11
RTU_SPAD_B2_XID     .set 12
PRU_SPAD_B0_XID     .set 10
PRU_SPAD_B1_XID     .set 11
PRU_SPAD_B2_XID     .set 12
PRU_RTU_SPAD_XID    .set 15
RXL2_SIDEA_XID      .set 20
RXL2_SIDEB_XID      .set 21
RXL2_FILTER_XID     .set 22
RTU_BSRAM_XID       .set 30
FDB_DATA_XID        .set 30
FDB_ADDR_XID        .set 30
FDB_GEN_PATTERN_XID .set 31
FDB_P0_RES_XID      .set 32
FDB_P1_RES_XID      .set 33
FDB_HOST_RES_XID    .set 34
FDB_GEN_RES_XID     .set 35
RTU_RX_FB_XID       .set 35
RTU_BSRAM_SUM32_XID .set 38
TXL2_XID            .set 40
PRU_BSRAM_XID       .set 48
PRU_BSRAM_SUM32_XID .set 49
XFR2PSI_STAT_XID    .set 80
XFR2PSI_RD0_XID     .set 81
XFR2PSI_WR0_XID     .set 81
XFR2PSI_RD1_XID     .set 82
XFR2PSI_RD2_XID     .set 83
XFR2PSI_RD3_XID     .set 84
RTU_XFR2PSI_SHARE_PRU0_XID .set 88
RTU_XFR2PSI_SHARE_PRU1_XID .set 89
XFR2VBUSP_RD0_XID   .set 96
XFR2VBUSP_RD1_XID   .set 97
XFR2VBUSP_WR0_XID   .set 98
XFR2VBUSP_WR1_XID   .set 99
XFR2VBUSP_RD2_XID   .set 100; TXPRU only
XFR2TR_MASTER_XID   .set 112
XFR2TR_RING_XID     .set 113
XFR2TR_SUBMIT_XID   .set 114
INT_SPIN_XID        .set 144
EXT0_SPIN_XID       .set 145
EXT1_SPIN_XID       .set 146
BSWAP_XID           .set 160
BSWAP_4_8_XID       .set 161
BSWAP_4_16_XID      .set 162
QUEUE_EMPTY_XID     .set 240
TM_YIELD_XID        .set 252
    .endif ; ____icss_xfer_defines_h

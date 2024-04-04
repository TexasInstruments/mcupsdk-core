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
; file:   icss_miirt_regs.h
;
; brief:  ICSS MII_RT Module Registers
;
;
;  (C) Copyright 2017, Texas Instruments, Inc
;
;

	.if !$defined("__icss_miirt_regs_h")
__icss_miirt_regs_h	.set	1

ICSS_MIIRT_RXCFG0    	.set    0x0000    ; RX Port attached to PRU0 Configuration
ICSS_MIIRT_RXCFG1    	.set    0x0004    ; RX Port attached to PRU1 Configuration
ICSS_MIIRT_RSVD0    	.set    0x0008    ; Reserved
ICSS_MIIRT_TXCFG0    	.set    0x0010    ; TX Port0 Configuration
ICSS_MIIRT_TXCFG1    	.set    0x0014    ; TX Port1 Configuration
ICSS_MIIRT_RSVD1    	.set    0x0018    ; Reserved
ICSS_MIIRT_TXCRC0    	.set    0x0020    ; HW Accumulated TX Port0 CRC32 Read Only
ICSS_MIIRT_TXCRC1    	.set    0x0024    ; HW Accumulated TX POrt1 CRC32 Read Only
ICSS_MIIRT_TXIPG0    	.set    0x0030    ; TX Port0 Min Inter Packet Gap
ICSS_MIIRT_TXIPG1    	.set    0x0034    ; TX Port1 Min Inter Packet Gap
ICSS_MIIRT_PRS0    		.set    0x0038    ; PRU0 Raw Status Read Only
ICSS_MIIRT_PRS1    		.set    0x003c    ; PRU1 Raw Status Read Only
ICSS_MIIRT_RXFRMS0    	.set    0x0040    ; RX Port0 Frame Length Min/Max Limits
ICSS_MIIRT_RXFRMS1    	.set    0x0044    ; RX Port1 Frame Length Min/Max Limits
ICSS_MIIRT_RXPCNT0    	.set    0x0048    ; RX Port0 Preamble Length Min/Max Limits
ICSS_MIIRT_RXPCNT1    	.set    0x004C    ; RX Port1 Preamble Length Min/Max Limits
ICSS_MIIRT_RXERR0    	.set    0x0050    ; RX Port0 Error Status, Write 1 to Clear
ICSS_MIIRT_RXERR1    	.set    0x0054    ; RX Port1 Error Status, Write 1 to Clear

; Primitives
; ICSS_MIIRT_RXCFG0/1
RX_DISABLE    			.set    0x00    ; Disable RX traffic
RX_ENABLE    			.set    0x01    ; Enables RX traffic which is set by RX_MUX_SELECT
RX_CUT_PREAMBLE    		.set    0x04    ; Cuts preamble, first data seen by PRU is MAC destination address
RX_MUX_SELECT_P0    	.set    0x00    ; Select MII RX Data from Port 0
RX_MUX_SELECT_P1    	.set    0x08    ; Select MII RX Data from Port 1
RX_L2_ENABLE    		.set    0x10    ; Enable RX L2 Buffer
RX_BYTE_SWAP_ENABLE    	.set 	0x20    ; Byte 1 maps to bit [7:0] and byte 0 maps to bit [15:8] of R31
RX_AUTO_FWD_PRE    		.set    0x40    ; Forwards preamble to attached TX_FIFO in TX_AUTO_SEQUENCE MODE

; ICSS_MIIRT_TXCFG0/1
TX_DISABLE    			.set    0x00    		; Disable TX traffic
TX_ENABLE    			.set    0x01    		; Enables TX traffic gated by TX_START_DELAY and IPG counter
TX_AUTO_PREAMBLE	    .set    0x02    		; TX Fifo inserts preamble
TX_EN_MODE    			.set    0x04    		; TX_EOF event clears TX_ENABLE - manual restart by PRU needed
TX_BYTE_SWAP    		.set    0x08    		; Byte 1 maps to bit [7:0] and byte 0 maps to bit [15:8] of R30
TX_MUX_SELECT_PRU0    	.set  	0x000   		; TX data from PRU0 is selected
TX_MUX_SELECT_PRU1    	.set  	0x100   		; TX data from PRU1 is selected
TX_AUTO_SEQUENCE    	.set    0x200   		; Incoming data is automatically forwarded to TX_MII - e.g ESC reverse path
TX_START_DELAY_0    	.set    0x000000  		; 0 ns tx delay after RXDV received - used of on the fly processing (min)
TX_START_DELAY_320    	.set    0x00400000  	; 320 ns tx delay after RXDV received - used of on the fly processing (default)
TX_START_DELAY_360    	.set    0x00480000  	; 360 ns tx delay after RXDV received - used of on the fly processing
TX_START_DELAY_640    	.set    0x00800000  	; 640 ns tx delay after RXDV received - used of on the fly processing
TX_START_DELAY_5120    	.set    0x003F0000 		; 5120 ns tx delay after RXDV received - used of on the fly processing (max)
TX_CLK_DELAY_10ns    	.set	0x00000000    	; 10 ns of MII_TXCLK delay (default)
TX_CLK_DELAY_15ns    	.set	0x10000000    	; 15 ns of MII_TXCLK delay
TX_CLK_DELAY_20ns    	.set	0x20000000    	; 20 ns of MII_TXCLK delay
TX_CLK_DELAY_25ns    	.set	0x30000000    	; 25 ns of MII_TXCLK delay
TX_CLK_DELAY_30ns    	.set	0x40000000    	; 30 ns of MII_TXCLK delay
TX_CLK_DELAY_35ns    	.set	0x50000000    	; 35 ns of MII_TXCLK delay

; Register settings for EMAC0 on port 0 - RX FIFO Mode
RXCFG0_EMAC0	.set	   	(RX_ENABLE | RX_CUT_PREAMBLE | RX_MUX_SELECT_P0 | RX_L2_ENABLE )
TXCFG0_EMAC0	.set		(TX_ENABLE | TX_AUTO_PREAMBLE | TX_MUX_SELECT_PRU0 | TX_START_DELAY_0 | TX_CLK_DELAY_10ns)

; Register settings for EMAC1 on port 1 - RX FIFO Mode
RXCFG1_EMAC1	.set	   	(RX_ENABLE | RX_CUT_PREAMBLE | RX_MUX_SELECT_P1 | RX_L2_ENABLE )
TXCFG1_EMAC1	.set		(TX_ENABLE | TX_AUTO_PREAMBLE | TX_MUX_SELECT_PRU1 | TX_START_DELAY_0 | TX_CLK_DELAY_10ns)

; Register settings for on the fly processing with cross connect - 360ns MII2MII delay
; Rx port0 -> PRU0 -> TX port1
; RX port1 -> PRU1 -> TX port0
RXCFG0_OTF	.set	   	(RX_ENABLE | RX_CUT_PREAMBLE | RX_MUX_SELECT_P0 )
TXCFG0_OTF	.set			(TX_ENABLE | TX_AUTO_PREAMBLE | TX_MUX_SELECT_PRU1 | TX_START_DELAY_360 | TX_CLK_DELAY_10ns)
RXCFG1_OTF	.set	   	(RX_ENABLE | RX_CUT_PREAMBLE | RX_MUX_SELECT_P1 )
TXCFG1_OTF	.set			(TX_ENABLE | TX_AUTO_PREAMBLE | TX_MUX_SELECT_PRU0 | TX_START_DELAY_360 | TX_CLK_DELAY_10ns)

; Register settings for cut-through switch with cross connect - RX FIFO Mode
; Rx port0 -> PRU0 -> TX port1
; RX port1 -> PRU1 -> TX port0
RXCFG0_CTS	.set	   	(RX_ENABLE | RX_CUT_PREAMBLE | RX_MUX_SELECT_P0 | RX_L2_ENABLE )
TXCFG0_CTS	.set			(TX_ENABLE | TX_AUTO_PREAMBLE | TX_MUX_SELECT_PRU1 | TX_START_DELAY_0 | TX_CLK_DELAY_10ns)
RXCFG1_CTS	.set	   	(RX_ENABLE | RX_CUT_PREAMBLE | RX_MUX_SELECT_P1 | RX_L2_ENABLE )
TXCFG1_CTS	.set			(TX_ENABLE | TX_AUTO_PREAMBLE | TX_MUX_SELECT_PRU0 | TX_START_DELAY_0 | TX_CLK_DELAY_10ns)

;R31 Status flags
	.asg	t16, D_DATA_READY_FLAG_BITNUM
	.asg	t17, D_BYTE_READY_FLAG_BITNUM
	.asg	t18, D_WORD_READY_FLAG_BITNUM
	.asg	t19, D_RX_ERROR_FLAG_BITNUM
	.asg	t20, D_EOF_FLAG_BITNUM
	.asg	t21, D_SFD_FLAG_BITNUM
	.asg	t22, D_SOF_FLAG_BITNUM
	.asg	t23, D_ERROR_NIBBLE_FLAG_BITNUM
	.asg	t24, D_CRC_ERR_FLAG_BITNUM
	.asg	t25, D_RX_ERR_FLAG_BITNUM
	.asg	t26, D_RX_MAX_PRE_CNT_ERR_FLAG_BITNUM
	.asg	t27, D_RX_EOF_ERROR_FLAG_BITNUM
	.asg	t28, D_RX_MAX_FRM_CNT_ERR_FLAG_BITNUM
	.asg	t29, D_RX_MIN_FRM_CNT_ERR_FLAG_BITNUM

D_DATA_READY_FLAG_BITNUM			.set		16
D_BYTE_READY_FLAG_BITNUM			.set		17
D_WORD_READY_FLAG_BITNUM			.set		18
D_RX_ERROR_FLAG_BITNUM				.set		19
D_EOF_FLAG_BITNUM					.set		20
D_SFD_FLAG_BITNUM					.set		21
D_SOF_FLAG_BITNUM					.set		22
D_ERROR_NIBBLE_FLAG_BITNUM			.set		23
D_CRC_ERR_FLAG_BITNUM				.set		24
D_RX_ERR_FLAG_BITNUM				.set		25
D_RX_MAX_PRE_CNT_ERR_FLAG_BITNUM	.set		26
D_RX_EOF_ERROR_FLAG_BITNUM			.set		27
D_RX_MAX_FRM_CNT_ERR_FLAG_BITNUM	.set		28
D_RX_MIN_FRM_CNT_ERR_FLAG_BITNUM	.set		29


D_DATA_READY_FLAG_MASK	.set			(1 << 0)
D_BYTE_READY_FLAG_MASK	.set			(1 << 1)
D_WORD_READY_FLAG_MASK	.set			(1 << 2)
D_RX_ERROR_FLAG_MASK	.set			(1 << 3)
D_EOF_FLAG_MASK	.set					(1 << 4)
D_SFD_FLAG_MASK	.set					(1 << 5)
D_SOF_FLAG_MASK	.set					(1 << 6)
D_ERROR_NIBBLE_FLAG_MASK	.set		(1 << 7)
D_CRC_ERR_FLAG_MASK	.set				(1 << 8)
D_RX_ERR_FLAG_MASK	.set				(1 << 9)


;R31 Command masks

D_TX_CRC_ERR			.set	0x8000
D_RESET_TXFIFO			.set	0x4000
D_TX_EOF				.set	0x2000
D_PUSH_ERR_NIBBLE_CMD	.set	0x1000

D_PUSH_CRC_MSWORD_CMD	.set	0x0800
D_PUSH_CRC_LSWORD_CMD	.set	0x0400
D_PUSH_TX_EOF_CMD		.set	0x0C00
D_PUSH_WORD_CMD			.set	0x0200
D_PUSH_BYTE_CMD			.set	0x0100


D_RX_ERROR_CLEAR		.set	0x0080
D_RX_SOF_CLEAR			.set	0x0040
D_RX_SFD_CLEAR			.set	0x0020
D_RX_EOF_CLEAR			.set	0x0010

D_RESET_RXFIFO			.set	0x0004
D_POP_WORD_CMD			.set	0x0002
D_POP_BYTE_CMD			.set	0x0001

D_PUSH_N_POP_BYTE_CMD	.set	D_POP_BYTE_CMD | D_PUSH_BYTE_CMD
D_PUSH_N_POP_WORD_CMD	.set	D_POP_WORD_CMD | D_PUSH_WORD_CMD

	.endif	;__icss_miirt_regs_h

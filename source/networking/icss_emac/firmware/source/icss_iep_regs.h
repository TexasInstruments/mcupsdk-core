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
; file:   icss_iep_regs.h
;
; brief:  ICSS Industrial Ethernet Peripheral Registers
;
;
;  (C) Copyright 2017, Texas Instruments, Inc
;
;

	.if !$defined("__icss_iep_regs_h")
__icss_iep_regs_h	.set	1

ICSS_IEP_TIMER_BLK_INDEX		.set	0

ICSS_IEP_GLOBAL_CFG_REG			.set	0x0000
ICSS_IEP_GLOBAL_STATUS_REG		.set	0x0004
ICSS_IEP_COMPEN_REG				.set	0x0008

	.if $defined("ICSS_REV1")
ICSS_IEP_COUNT_REG				.set	0x000C
ICSS_IEP_CAP_CFG_REG			.set	0x0010
ICSS_IEP_CAP_STATUS_REG			.set	0x0014
ICSS_IEP_CAPR0_REG				.set	0x0018
ICSS_IEP_CAPR1_REG				.set	0x001C
ICSS_IEP_CAPR2_REG				.set	0x0020
ICSS_IEP_CAPR3_REG				.set	0x0024
ICSS_IEP_CAPR4_REG				.set	0x0028
ICSS_IEP_CAPR5_REG				.set	0x002C
ICSS_IEP_CAPR6_REG				.set	0x0030
ICSS_IEP_CAPF6_REG				.set	0x0034
ICSS_IEP_CAPR7_REG				.set	0x0038
ICSS_IEP_CAPF7_REG				.set	0x003C
ICSS_IEP_CMP_CFG_REG			.set	0x0040
ICSS_IEP_CMP_STATUS_REG			.set	0x0044
ICSS_IEP_CMP0_REG				.set	0x0048
ICSS_IEP_CMP1_REG				.set	0x004C
ICSS_IEP_CMP2_REG				.set	0x0050
ICSS_IEP_CMP3_REG				.set	0x0054
ICSS_IEP_CMP4_REG				.set	0x0058
ICSS_IEP_CMP5_REG				.set	0x005C
ICSS_IEP_CMP6_REG				.set	0x0060
ICSS_IEP_CMP7_REG				.set	0x0064
ICSS_IEP_RXIPG0_REG				.set	0x0080
ICSS_IEP_RXIPG1_REG				.set	0x0084
	.endif	;ICSS_REV1

	.if $defined("ICSS_REV2")
ICSS_IEP_COUNT_REG				.set	0x0010
ICSS_IEP_CAP_CFG_REG			.set	0x0018
ICSS_IEP_CAP_STATUS_REG			.set	0x001c
ICSS_IEP_CAPR0_REG				.set	0x0020
ICSS_IEP_CAPR1_REG				.set	0x0028
ICSS_IEP_CAPR2_REG				.set	0x0030
ICSS_IEP_CAPR3_REG				.set	0x0038
ICSS_IEP_CAPR4_REG				.set	0x0040
ICSS_IEP_CAPR5_REG				.set	0x0048
ICSS_IEP_CAPR6_REG				.set	0x0050
ICSS_IEP_CAPF6_REG				.set	0x0058
ICSS_IEP_CAPR7_REG				.set	0x0060
ICSS_IEP_CAPF7_REG				.set	0x0068
ICSS_IEP_CMP_CFG_REG			.set	0x0070
ICSS_IEP_CMP_STATUS_REG			.set	0x0074
ICSS_IEP_CMP0_REG				.set	0x0078
ICSS_IEP_CMP1_REG				.set	0x0080
ICSS_IEP_CMP2_REG				.set	0x0088
ICSS_IEP_CMP3_REG				.set	0x0090
ICSS_IEP_CMP4_REG				.set	0x0098
ICSS_IEP_CMP5_REG				.set	0x00a0
ICSS_IEP_CMP6_REG				.set	0x00a8
ICSS_IEP_CMP7_REG				.set	0x00b0
ICSS_IEP_RXIPG0_REG				.set	0x00b8
ICSS_IEP_RXIPG1_REG				.set	0x00bc
ICSS_IEP_CMP8_REG   			.set	0x00c0
ICSS_IEP_CMP9_REG   			.set	0x00c8
ICSS_IEP_CMP10_REG  			.set	0x00d0
ICSS_IEP_CMP11_REG  			.set	0x00d8
ICSS_IEP_CMP12_REG  			.set	0x00e0
ICSS_IEP_CMP13_REG  			.set	0x00e8
ICSS_IEP_CMP14_REG  			.set	0x00f0
ICSS_IEP_CMP15_REG  			.set	0x00f8
	.endif	;ICSS_REV2

ICSS_IEP_SYNC_BLK_INDEX			.set	1
ICSS_IEP_SYNC_CTRL_REG			.set	0x0000
ICSS_IEP_SYNC_FIRST_STATUS_REG	.set	0x0004
ICSS_IEP_SYNC0_STATUS_REG		.set	0x0008
ICSS_IEP_SYNC1_STATUS_REG		.set	0x000C
ICSS_IEP_SYNC_PWIDTH_REG		.set	0x0010
ICSS_IEP_SYNC0_PERIOD_REG		.set	0x0014
ICSS_IEP_SYNC1_DELAY_REG		.set	0x0018
ICSS_IEP_SYNC_START_REG			.set	0x001C

ICSS_IEP_WD_BLK_INDEX			.set	2
ICSS_IEP_WD_PREDIV_REG			.set	0x0000
ICSS_IEP_PDI_WD_TIM_REG			.set	0x0004
ICSS_IEP_PD_WD_TIM_REG			.set	0x0008
ICSS_IEP_WD_STATUS_REG			.set	0x000C
ICSS_IEP_EXP_COUNTER_REG		.set	0x0010
ICSS_IEP_WD_CTRL_REG			.set	0x0014

ICSS_IEP_DIGIO_BLK_INDEX		.set	3
ICSS_IEP_DIGIO_CTRL_REG			.set	0x0000
ICSS_IEP_DIGIO_STATUS_REG		.set	0x0004
ICSS_IEP_DIGIO_DATA_IN_REG		.set	0x0008
ICSS_IEP_DIGIO_DATA_IN_RAW_REG	.set	0x000C
ICSS_IEP_DIGIO_DATA_OUT_REG		.set	0x0010
ICSS_IEP_DIGIO_DATA_OUT_EN_REG	.set	0x0014
ICSS_IEP_DIGIO_EXP_REG			.set	0x0018

	.endif	;__icss_iep_regs_h

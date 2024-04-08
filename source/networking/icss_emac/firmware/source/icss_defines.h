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
; file:   icss_defines.h
;
; brief:  ICSS Global Defines
;
;
;  (C) Copyright 2017, Texas Instruments, Inc
;
;

    .if    !$defined("__icss_defines_h")
__icss_defines_h    .set 1

; Bank ids for Xfer instructions
BANK0    			.set	10	; scratch pad R2-R5 is permanent for cut-through
BANK1    			.set	11	; R20-R23 is permanent for rcv context
BANK2    			.set	12	; XFR with other PRU

RX_L2_BANK0_ID    	.set    20  ; RX_L2 Fifo bank 0
RX_L2_BANK1_ID    	.set    21  ; RX_L2 Fifo bank 1

TX_L2_BANK_ID		.set    40

; Constants need to be defined with correct offset before using. Typically done on ARM side.
	.asg    c0,    ICSS_INTC_CONST
	.asg    c3,    ICSS_ECAP_CONST

	.if $defined("PRU0")
	.asg    c24,    PRU0_DMEM_CONST
	.asg    c25,    PRU1_DMEM_CONST
PRU0_DMEM    			.set	0x0000
PRU1_DMEM    			.set	0x2000
PRU_DMEM_ADDR    		.set	PRU0_DMEM_CONST
PRU_CROSS_DMEM    		.set	PRU1_DMEM_CONST
	.else
    .asg    c24,    PRU1_DMEM_CONST
    .asg    c25,    PRU0_DMEM_CONST
PRU0_DMEM    			.set	0x2000
PRU1_DMEM    			.set	0x0000
PRU_DMEM_ADDR    		.set	PRU1_DMEM_CONST
PRU_CROSS_DMEM    		.set	PRU0_DMEM_CONST
	.endif

	.if    $defined("TWO_PORT_CFG")
	.if    $defined("PRU0")
MII_CARRIER_SENSE_REG    .set  ICSS_MIIRT_PRS1
	.else
MII_CARRIER_SENSE_REG    .set  ICSS_MIIRT_PRS0
	.endif
	.else
	.if    $defined("PRU0")
MII_CARRIER_SENSE_REG    .set  ICSS_MIIRT_PRS0
	.else
MII_CARRIER_SENSE_REG    .set  ICSS_MIIRT_PRS1
	.endif
	.endif

    .asg    c21,    ICSS_MDIO_CONST
    .asg    c26,    IEP_CONST
    .asg    c27,    MII_RT_CFG_CONST
    .asg    c28,    ICSS_SHARED_CONST
ICSS_SHARED    .set            0x00010000
    .asg    c30,    L3_OCMC_RAM_CONST

; Permanent Registers not used by other tasks
	.asg	R19, Reserved_R19
	.asg	R20, Reserved_R20
	.asg	R21, Reserved_R21
	.asg	R22, Reserved_R22
	.asg	R23.b0, Reserved_R23

	.asg	R24.w2, CALL_REG
	.asg	R24.w0, L1_CALL_REG

CALL    .macro function
        JAL     CALL_REG, function
        .endm

RET     .macro
        JMP     CALL_REG
        .endm

; Note R25-R29 is multiplier
; R0 and R1 is used by some instructions

	.endif	;__icss_defines_h

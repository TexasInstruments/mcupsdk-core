
;
; Copyright (C) 2021 Texas Instruments Incorporated
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
;   Redistributions of source code must retain the above copyright
;   notice, this list of conditions and the following disclaimer.
;
;   Redistributions in binary form must reproduce the above copyright
;   notice, this list of conditions and the following disclaimer in the
;   documentation and/or other materials provided with the
;   distribution.
;
;   Neither the name of Texas Instruments Incorporated nor the names of
;   its contributors may be used to endorse or promote products derived
;   from this software without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; "AS IS" AND ANY EXPgResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
; OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
; SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
; LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;

	.if	!$isdefed("__icss_regs_h")
__icss_regs_h	.set	1

	.asg	c0,	ICSS_INTC
	.asg	c3,	ICSS_ECAP
	.asg	c4,	ICSS_CFG
	.asg	c21,	ICSS_MDIO
	.if $isdefed("PRU1")
	.asg	c24,	PRU1_DMEM
	.asg	c25,	PRU0_DMEM
	.else
	.asg	c24,	PRU0_DMEM
	.asg	c25,	PRU1_DMEM
	.endif
	.asg	c26,	ICSS_IEP
	.asg	c27,	MII_RT_CFG
	.asg	c28,	ICSS_SHARED_RAM
	.asg	c30,	OCMC_RAM

;  ==== Base Address of ICSS resources EXPORTED to OCP Clients outside... ====
ICSS_DRAM0_BASE	.set		(0x00000000)	;  PRU0's Private Data RAM (8kB)
ICSS_DRAM1_BASE	.set		(0x00002000)	;  PRU1's Private Data RAM (8kB)
ICSS_SHMEM_BASE	.set		(0x00010000)	;  PRU0-PRU1 Shared RAM (12kB)
ICSS_INTC_BASE	.set		(0x00020000)	;  PRU0-PRU1 System Events INTC Module
ICSS_CNTL0_BASE	.set		(0x00022000)	;  PRU0's Control Module ??spec??
ICSS_DBG0_BASE	.set		(0x00022400)	;  PRU0's Debug Module ??spec??
ICSS_CNTL1_BASE	.set		(0x00024000)	;  PRU1's Control Module ??spec??
ICSS_DBG1_BASE	.set		(0x00024400)	;  PRU1's Debug Module ??spec??
ICSS_RSVD1_BASE	.set		(0x00024400)	;  Reserved
ICSS_CFG_BASE	.set		(0x00026000)	;  PRU0-PRU1 Configuration Module
ICSS_UART0_BASE	.set		(0x00028000)	;  UART Peripheral
ICSS_RSVD2_BASE	.set		(0x0002A000)	;  Reserved
ICSS_RSVD3_BASE	.set		(0x0002C000)	;  Reserved
ICSS_IEP0_BASE	.set		(0x0002E000)	;  Industrial Ethernet Peripheral
ICSS_ECAP0_BASE	.set		(0x00030000)	;  Enhanced Capture Module
ICSS_MIIRT_CFG_BASE	.set	(0x00032000)	;  802.3 MII Real-Time Unit
ICSS_MIIRT_MDIO_BASE	.set	(0x00032400)	;  802.3 PHY Management Module
ICSS_IRAM0_BASE	.set		(0x00034000)	;  PRU0's Private Instruction RAM (8kB)
ICSS_IRAM1_BASE	.set		(0x00038000)	;  PRU1's Private Instruction RAM (8kB)
ICSS_RSVD4_BASE	.set		(0x00040000)	;  Reserved

;  ==== Base Address of ICSS resources as seen locally by the PRUs... ====
PRUx_DSELF_BASE	.set	(0x00000000)	;  Own Data RAM (8kB)
PRUx_DPEER_BASE	.set	(0x00002000)	;  Data RAM (8kB) of Peer PRU inside ICSS
PRUx_SHMEM_BASE	.set		(0x00010000)	;  Shared RAM (12kB) common to PRU0 and PRU1
PRUx_INTC_BASE	.set		(0x00020000)
PRUx_CNTLSELF_BASE	.set	(0x00022000)	;  Own Control Module
PRUx_RSVD0_BASE	.set		(0x00022400)	;  Reserved
PRUx_CNTLPEER_BASE	.set	(0x00024000)	;  Control Module of Peer PRU inside ICSS
PRUx_RSVD1_BASE	.set		(0x00024400)
PRUx_CFG_BASE	.set		(0x00026000)
PRUx_UART0_BASE	.set		(0x00028000)
PRUx_RSVD2_BASE	.set		(0x0002A000)
PRUx_RSVD3_BASE	.set		(0x0002C000)
PRUx_IEP0_BASE	.set		(0x0002E000)
PRUx_ECAP0_BASE	.set		(0x00030000)
PRUx_MIIRT_CFG_BASE	.set	(0x00032000)
PRUx_MIIRT_MDIO_BASE	.set	(0x00032400)
PRUx_RSVD4_BASE	.set		(0x00034000)
PRUx_RSVD5_BASE	.set		(0x00038000)
PRUx_RSVD6_BASE	.set		(0x00040000)
PRUx_SYS_OCP_BASE	.set	(0x00080000)	;  Resources off of OCP i/f, outside ICSS

;  ==== Register Includes for ICSS Constituent Peripheral Devices/Modules...
	.include "icss_intc_regs.h"
	.include "icss_cntl_regs.h"
	.include "icss_cfg_regs.h"
	; .include "icss_uart_regs.h"
	.include "icss_iep_regs.h"
	.include "icss_ecap_regs.h"
	; .include "icss_miirt_regs.h"
	; .include "icss_mdio_regs.h"
	; .include "icss_esc_defs.h"

	.endif

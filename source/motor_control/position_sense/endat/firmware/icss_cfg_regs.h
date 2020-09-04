
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

	.if	!$isdefed("__icss_cfg_regs_h")
__icss_cfg_regs_h	.set	1

ICSS_CFG_REVID	.set		0x0000	;  Module Revision ID
ICSS_CFG_SYSCFG	.set		0x0004	;  System Idle/Standby Configuration
ICSS_CFG_GPCFG0	.set		0x0008	;  GP IO Configuration Register 0
ICSS_CFG_GPCFG1	.set		0x000C	;  GP IO Configuration Register 1
ICSS_CFG_CGR	.set		0x0010	;  Clock Gating Register
ICSS_CFG_ISRP	.set		0x0014	;  IRQ Status RAW Parity
ICSS_CFG_ISP	.set		0x0018	;  IRQ Status Parity (Masked)
ICSS_CFG_IESP	.set		0x001C	;  IRQ Enable Set Parity
ICSS_CFG_IECP	.set		0x0020	;  IRQ Enable Clear Parity (ie., Disable)
ICSS_CFG_SCRP	.set		0x0024	;  SCR Interconnect Arbitration Priority
ICSS_CFG_PMAO	.set		0x0028	;  PRU Master OCP Address Offset
ICSS_CFG_MIIRTEN	.set	0x002C	;  MII RealTime Module Enable
ICSS_CFG_IEPCLK	.set		0x0030	;  Industrial Ethernet Peripheral Clocking
ICSS_CFG_SPPC	.set		0x0034	;  Scratch PAD priority and config
ICSS_CFG_MUXSEL	.set		0x0040	;  Mux sel for SA
; ENDAT configuration registers
ICSS_CFG_PRU0_ENDAT_RXCFG	.set		0xE0
ICSS_CFG_PRU0_ENDAT_TXCFG	.set		0xE4
ICSS_CFG_PRU0_ENDAT_CH0_CFG0	.set	0xE8
ICSS_CFG_PRU0_ENDAT_CH0_CFG1	.set	0xEC
ICSS_CFG_PRU0_ENDAT_CH1_CFG0	.set	0xF0
ICSS_CFG_PRU0_ENDAT_CH1_CFG1	.set	0xF4
ICSS_CFG_PRU0_ENDAT_CH2_CFG0	.set	0xF8
ICSS_CFG_PRU0_ENDAT_CH2_CFG1	.set	0xFC

ICSS_CFG_PRU1_ENDAT_RXCFG	.set		0x100
ICSS_CFG_PRU1_ENDAT_TXCFG	.set		0x104
ICSS_CFG_PRU1_ENDAT_CH0_CFG0	.set	0x108
ICSS_CFG_PRU1_ENDAT_CH0_CFG1	.set	0x10C
ICSS_CFG_PRU1_ENDAT_CH1_CFG0	.set	0x110
ICSS_CFG_PRU1_ENDAT_CH1_CFG1	.set	0x114
ICSS_CFG_PRU1_ENDAT_CH2_CFG0	.set	0x118
ICSS_CFG_PRU1_ENDAT_CH2_CFG1	.set	0x11C

ICSS_CFG_PRU0_SD0_CLK 		.set	0x48
ICSS_CFG_PRU0_SD0_SAMPLE_SIZE	.set	0x4C
ICSS_CFG_PRU0_SD1_CLK		.set	0x50
ICSS_CFG_PRU0_SD1_SAMPLE_SIZE	.set	0x54
ICSS_CFG_PRU0_SD2_CLK		.set	0x58
ICSS_CFG_PRU0_SD2_SAMPLE_SIZE	.set	0x5C
ICSS_CFG_PRU0_SD3_CLK		.set	0x60
ICSS_CFG_PRU0_SD3_SAMPLE_SIZE	.set	0x64
ICSS_CFG_PRU0_SD4_CLK		.set	0x68
ICSS_CFG_PRU0_SD4_SAMPLE_SIZE	.set	0x6C
ICSS_CFG_PRU0_SD5_CLK		.set	0x70
ICSS_CFG_PRU0_SD5_SAMPLE_SIZE	.set	0x74
ICSS_CFG_PRU0_SD6_CLK		.set	0x78
ICSS_CFG_PRU0_SD6_SAMPLE_SIZE	.set	0x7C
ICSS_CFG_PRU0_SD7_CLK		.set	0x80
ICSS_CFG_PRU0_SD7_SAMPLE_SIZE	.set	0x84
ICSS_CFG_PRU0_SD8_CLK		.set	0x88
ICSS_CFG_PRU0_SD8_SAMPLE_SIZE	.set	0x8C

ICSS_CFG_PRU1_SD0_CLK 		.set	0x94
ICSS_CFG_PRU1_SD0_SAMPLE_SIZE	.set	0x98
ICSS_CFG_PRU1_SD1_CLK		.set	0x9C
ICSS_CFG_PRU1_SD1_SAMPLE_SIZE	.set	0xA0	
ICSS_CFG_PRU1_SD2_CLK		.set	0xA4
ICSS_CFG_PRU1_SD2_SAMPLE_SIZE	.set	0xA8
ICSS_CFG_PRU1_SD3_CLK		.set	0xAC
ICSS_CFG_PRU1_SD3_SAMPLE_SIZE	.set	0xB0
ICSS_CFG_PRU1_SD4_CLK		.set	0xB4
ICSS_CFG_PRU1_SD4_SAMPLE_SIZE	.set	0xB8
ICSS_CFG_PRU1_SD5_CLK		.set	0xBC
ICSS_CFG_PRU1_SD5_SAMPLE_SIZE	.set	0xC0
ICSS_CFG_PRU1_SD6_CLK		.set	0xC4
ICSS_CFG_PRU1_SD6_SAMPLE_SIZE	.set	0xC8
ICSS_CFG_PRU1_SD7_CLK		.set	0xCC
ICSS_CFG_PRU1_SD7_SAMPLE_SIZE	.set	0xD0
ICSS_CFG_PRU1_SD8_CLK		.set	0xD4
ICSS_CFG_PRU1_SD8_SAMPLE_SIZE	.set	0xD8

	.endif

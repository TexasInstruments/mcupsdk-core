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

;************************************************************************************
;   File:     icss_constant_defines.inc
;
;   Brief:    Constant defines
;************************************************************************************

    .if !$isdefed("____icss_constant_defines_h")
____icss_constant_defines_h   .set    1

;--------------------------------------------------------------------------;
;   Constants: Global Constant Table Entries
;
;   ICSS_INTC_CONST  - ICSS Interrupt Controller Registers Base Address
;   ICSS_IEP1_0_CONST - ICSS IEP1 offset
;   ICSS_IEP1_1_CONST - ICSS IEP1 + 0x100 offset for optimal access from firmware
;   ICSS_ECAP_CONST  - ICSS Enhanced Capture Registers Base Address
;   ICSS_CFG_CONST - ICSS CFG Registers Base Address
;   ICSS_CFG_1_CONST - ICSS ICFG + 0x100 offset for optimal access from firmware
;   ICSS_INTC_2_CONST - ICSS INTC + 0x200 offset for optimal access from firmware
;   ICSS_UART_CONST - ICSS UART Registers Base Address
;   ICSS_IEP0_1_CONST - ICSS IEP0 + 0x100 offset for optimal access from firmware
;   ICSS_G_CFG_CONST - ICSS_G CFG Registers Base Address
;   ICSS_TM_CFG_CONST - ICSS Task Manager Config Base Address for PRU0/1 and RTU0/1
;   ICSS_PRU_CTRL_CONST - ICSS PRU Control Registers Base Address for PRU/RTU/0/1
;   ICSS_PA_STAT_QRAM_CONST - ICSS PASTAT Query RAM Base Address
;   ICSS_PA_STAT_CRAM_CONST - ICSS PASTAT Clear on Read RAM Base Address
;   ICSS_PROTECT_CONST - ICSS Protect Registers Base Address
;   ICSS_RAT_REGION0_CONST - ICSS RAT REGION0 (Map SoC address to 0x6000_0000 in RAT)
;   ICSS_RAT_REGION1_CONST - ICSS RAT REGION1 (Map SoC address to 0x7000_0000 in RAT)
;   ICSS_RAT_REGION2_CONST - ICSS RAT REGION2 (Map SoC address to 0x8000_0000 in RAT)
;   ICSS_RAT_REGION3_CONST - ICSS RAT REGION3 (Map SoC address to 0x9000_0000 in RAT)
;   ICSS_RAT_REGION4_CONST - ICSS RAT REGION4 (Map SoC address to 0xA000_0000 in RAT)
;   ICSS_RAT_REGION5_CONST - ICSS RAT REGION5 (Map SoC address to 0xB000_0000 in RAT)
;   ICSS_MDIO_CONST  - ICSS MII-MDIO Registers Base Address
;   ICSS_RAT_CONST - ICSS RAT Config Registers Base Address
;   ICSS_RAT_REGION6_CONST - ICSS RAT REGION6 (Map SoC address to 0xC000_0000 in RAT)
;   ICSS_DMEM0_CONST  - ICSS Data Memory 0 Base Address
;   ICSS_DMEM1_CONST  - ICSS Data Memory 1 Base Address
;   ICSS_IEP_CONST  - ICSS Industrial Ethernet Peripheral Registers Base Address
;   ICSS_MII_RT_CONST  - ICSS Real-time MII Registers Base Address
;   ICSS_SMEM_CONST  - ICSS Shared Memory Base Address
;   ICSS_RAT_REGION7_CONST - ICSS RAT REGION7 (Map SoC address to 0xD000_0000 in RAT)
;   ICSS_RAT_REGION8_CONST - ICSS RAT REGION8 (Map SoC address to 0xE000_0000 in RAT)
;   ICSS_RAT_REGION9_CONST - ICSS RAT REGION9 (Map SoC address to 0xF000_0000 in RAT)
;--------------------------------------------------------------------------;

;From ICSSG Functional Spec, section 5.2.4
;Entry	Region Pointed To	value
;0	Local INTC	0x0002_0000
;1	IEP1 (local)	0x0002_F000
;2	IEP1_0x100(local)	0x0002_F100
;3	eCAP(local)	0x0003_0000
;4	ICSS CFG(local)	0x0002_6000
;5	ICSS CFG_0x100 (local)	0x0002_6100
;6	INTC_0x200 (local)	0x0002_0200
;7	UART 0(local)	0x0002_8000
;8	IEP0_0x100(local)	0x0002_E100
;9	ICSS_G_CFG (local)	0x0003_3000
;10	TM_CFG_PRU/RTU0/1 (local)	0x0002_A000 PRU0 0x0002_A100 RTU0 0x0002_A200 PRU1 0x0002_A300 RTU1
;11	PRU/RTU/0/1 control (local)	0x0002_2000 PRU0 0x0002_4000 PRU1 0x0002_3000 RTU0 0x0002_3800 RTU1
;12	PA_STAT_QRAM(local)	0x0002_7000
;13	PA_STAT_CRAM(local)	0x0002_C000
;14	ICSS_Protect(local)	0x0002_4800
;15	Timer Manager (NAVSS) – RAT0/1	0x6000_0000
;16	RA (NAVSS) – RAT0/1	0x7000_0000
;17	IA (NAVSS) – RAT0/1	0x8000_0000
;18	GPMC – RAT0/1	0x9000_0000
;19	PCIE – RAT0/1	0xA000_0000
;20	UDMA-P – RAT0/1	0xB000_0000
;21	MDIO(local)	0x0003_2400
;22	RAT slice 0/1 (local)	0x0000_8000 PRU0/RTU0 0x0000_9000 PRU1/RTU1
;23	ADC - RAT0/1	0xC000_0000
;24	PRU0/1 Local Data	{20’h0000_0,c24_blk_index[3:0],8’h00}
;25	PRU1/0 Local Data	{20’h0000_2,c25_blk_index[3:0],8’h00}
;26	IEP(local)	{20’h0002_E,c26_blk_index[3:0],8’h00}
;27	MII_RT (local)/SGMII0/1_CFG	{20’h0003_2,c27_blk_index[3:0],8’h00}
;28	RAM 1_0(local)	{8'h00,ct_prt0[15:0],8'h00}
;29	DDR - RAT0/1	{8’hD0,ct_prt1[15:0],8'h00}
;30	MSMC - RAT0/1	{8’hE0,ct_prt2[15:0],8'h00}
;31	R5 RAM - RAT0/1	{8’hF0,ct_prt3[15:0],8'h00}

    .asg c0,  ICSS_INTC_CONST
    .asg c1,  ICSS_IEP1_0_CONST
    .asg c2,  ICSS_IEP1_1_CONST ; IEP1 + 0x100 offset
    .asg c3,  ICSS_ECAP_CONST
    .asg c4,  ICSS_CFG_CONST
    .asg c5,  ICSS_CFG_1_CONST ; CFG + 0x100 offset
    .asg c6,  ICSS_INTC_2_CONST ; INTC + 0x200 offset
    .asg c7,  ICSS_UART_CONST
    .asg c8,  ICSS_IEP0_1_CONST ; IEP0 + 0x100 offset
    .asg c9,  ICSS_G_CFG_CONST
    .asg c10, ICSS_TM_CFG_CONST
    .asg c11, ICSS_PRU_CTRL_CONST
    .asg c12, ICSS_PA_STAT_QRAM_CONST
    .asg c13, ICSS_PA_STAT_CRAM_CONST
    .asg c14, ICSS_PROTECT_CONST
    .asg c15, ICSS_RAT_REGION0_CONST ;0x6000_0000
    .asg c16, ICSS_RAT_REGION1_CONST ;0x7000_0000
    .asg c17, ICSS_RAT_REGION2_CONST ;0x8000_0000
    .asg c18, ICSS_RAT_REGION3_CONST ;0x9000_0000
    .asg c19, ICSS_RAT_REGION4_CONST ;0xA000_0000
    .asg c20, ICSS_RAT_REGION5_CONST ;0xB000_0000
    .asg c21, ICSS_MDIO_CONST
    .asg c22, ICSS_RAT_CONST
    .asg c23, ICSS_RAT_REGION6_CONST ;0xC000_0000
    .asg c24, ICSS_DMEM0_CONST
    .asg c25, ICSS_DMEM1_CONST
    .asg c26, ICSS_IEP_CONST
    .asg c27, ICSS_MII_RT_CONST
    .asg c28, ICSS_SMEM_CONST
    .asg c29, ICSS_RAT_REGION7_CONST ;0xD000_0000
    .asg c30, ICSS_RAT_REGION8_CONST ;0xE000_0000
    .asg c31, ICSS_RAT_REGION9_CONST ;0xF000_0000

ICSS_MSMC_RAM_CONST .set ICSS_RAT_REGION8_CONST

    .if $isdefed("PRU0")
PRU_DMEM_ADDR       .set    ICSS_DMEM0_CONST
PRU_CROSS_DMEM      .set    ICSS_DMEM1_CONST
    .else ;PRU1
PRU_DMEM_ADDR       .set    ICSS_DMEM1_CONST
PRU_CROSS_DMEM      .set    ICSS_DMEM0_CONST
    .endif

BANK0_ID             .set   10

    .endif ; ____icss_constant_defines_h

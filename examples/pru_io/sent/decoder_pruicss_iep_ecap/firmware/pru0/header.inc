; Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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
;   File:     memory.inc
;
;   Brief:    Sent decoder_pruicss_iep_ecap memory defines
;************************************************************************************

    .asg	    R0.w2,	BNS_CH_DATA_ERROR
    .asg        R1.b0,  BNS_ARG_RETVAL_ADDR
    .asg		R1.b1,  BNS_ARG_RETVAL
    .asg        R1.b2,  BNS_ARG_STATUS
    .asg 		R1.b3,  CH0_CRC4_RES
    .asg	    R2.b0,	CH1_CRC4_RES
    .asg	    R2.b1,	CH2_CRC4_RES
    .asg	    R2.b2,	CH3_CRC4_RES
    .asg	    R2.b3,	CH4_CRC4_RES
    .asg	    R3.b0,	CH5_CRC4_RES
    ;.asg		R3.b1,  unused
    .asg		R3.w2,  APPROX_VAL_MIN
    .asg        R4,     BNS_ARG_LUTBASE
    .asg        R5,     BNS_ARG_VAL
    .asg	    R6,	    CHx_PULSE_LEN
    .asg        R7,     CH0_PREVIOUS_FALLEDGE
    .asg        R8,     CH1_PREVIOUS_FALLEDGE
    .asg        R9,     CH2_PREVIOUS_FALLEDGE
    .asg        R10,    CH3_PREVIOUS_FALLEDGE
    .asg        R11,    CH4_PREVIOUS_FALLEDGE
    .asg        R12,    CH5_PREVIOUS_FALLEDGE
    .asg    	R13.w0, CH0_STATE
    .asg    	R13.w2, CH1_STATE
    .asg   	 	R14.w0, CH2_STATE
    .asg    	R14.w2, CH3_STATE
    .asg    	R15.w0, CH4_STATE
    .asg    	R15.w2, CH5_STATE
    ;Don't change assignment of R16-R21, shift offset is hardcoded in xin/xout instructions
    .asg        R16,    CH0_BUF
    .asg    	R17,    CH1_BUF
    .asg    	R18,    CH2_BUF
    .asg        R19,    CH3_BUF
    .asg    	R20,    CH4_BUF
    .asg        R21,    CH5_BUF
    .asg        R22,    channelx_tick_period
    .asg		R23,	APPROX_VAL_MAX
    .asg		R24.w0, return_addr1
    .asg		R24.w2, return_addr2
    .asg        R25,    CRC_LUT_ADDR
    ;Reserved for mac module but can be used for TEMP REG
    ;TODO: PASS BNS_LUT_VAL as TEMP_REG1 & create macro for binary search
    .asg		R26,    TEMP_REG1
    .asg		R27,    TEMP_REG2
    .asg		R28,	BNS_LUT_VAL



IEP_COUNT_REG0_OFFSET       .set    0x10
IEP_COUNT_REG1_OFFSET       .set    0x14
TICK_7_CONST                .set   613566757

PDMEM00 				    .set     0x0000
PDMME01                     .set     0x2000
ICSS_CFGx 				    .set     C5

;DMEM

;crc look up table
CRC4_LUT_OFFSET      	    .set     (0x0)
CRC4_LUT_SIZE               .set     (0x100)
CRC4_SEED            		.set     (0x5)

CH0_REG .set   					(R1)
CH1_REG .set   					(R4)
CH2_REG .set   					(R7)
CH3_REG .set   					(R10)
CH4_REG .set   					(R13)
CH5_REG .set   					(R16)



CH0_DATA0_SPAD_BASE .set					(15)
CH1_DATA0_SPAD_BASE .set					(17)
CH2_DATA0_SPAD_BASE .set					(19)
CH3_DATA0_SPAD_BASE .set					(21)
CH4_DATA0_SPAD_BASE .set					(23)
CH5_DATA0_SPAD_BASE .set					(25)


CH0_DATA4_SPAD_BASE .set					(16)
CH1_DATA4_SPAD_BASE .set					(18)
CH2_DATA4_SPAD_BASE .set					(20)
CH3_DATA4_SPAD_BASE .set					(22)
CH4_DATA4_SPAD_BASE .set					(24)
CH5_DATA4_SPAD_BASE .set					(26)


CH0_CRCDATA_SPAD_BASE .set					(17)
CH1_CRCDATA_SPAD_BASE .set					(19)
CH2_CRCDATA_SPAD_BASE .set					(21)
CH3_CRCDATA_SPAD_BASE .set					(23)
CH4_CRCDATA_SPAD_BASE .set					(25)
CH5_CRCDATA_SPAD_BASE .set					(27)


;look tables for pulse width for all the channels
CH0_LUT_BASE                .set    (0x100)
CH1_LUT_BASE 			    .set	(0x140)
CH2_LUT_BASE                .set	(0x180)
CH3_LUT_BASE                .set    (0x1C0)
CH4_LUT_BASE                .set	(0x200)
CH5_LUT_BASE                .set	(0x240)


;SMEM

;(sync+data+crc is stored)
CH0_DATA_BASE               .set	(0x40)
CH1_DATA_BASE               .set	(0x4C)
CH2_DATA_BASE               .set	(0x58)
CH3_DATA_BASE               .set	(0x64)
CH4_DATA_BASE               .set    (0x70)
CH5_DATA_BASE               .set	(0x7C)

;status whether data is received or not
CH0_INTR_BYTE               .set	(0x00)
CH1_INTR_BYTE               .set	(0x01)
CH2_INTR_BYTE               .set	(0x02)
CH3_INTR_BYTE               .set	(0x03)
CH4_INTR_BYTE               .set	(0x04)
CH5_INTR_BYTE               .set	(0x05)

;SCRATCH PAD 2
;--------------------------------------------
;   Channel0
;    .asg    R1.w0, ch0_tick period
;    .asg    R1.b2, ch0_status
;    .asg    R1.b3, ch0_data0
;    .asg    R2.b0, ch0_data1
;    .asg    R2.b1, ch0_data2
;    .asg    R2.b2, ch0_data3
;    .asg    R2.b3, ch0_data4
;    .asg    R3.b0, ch0_data5
;    .asg    R3.b1, ch0_crc
;    .asg    R3.b2, crc0_status
;   Channel1
;    .asg    R4.w0, ch0_tick period
;    .asg    R4.b2, ch0_status
;    .asg    R4.b3, ch0_data0
;    .asg    R5.b0, ch0_data1
;    .asg    R5.b1, ch0_data2
;    .asg    R5.b2, ch0_data3
;    .asg    R5.b3, ch0_data4
;    .asg    R6.b0, ch0_data5
;    .asg    R6.b1, ch0_crc
;    .asg    R6.b2, crc1_status
;   Channel2
;    .asg    R7.w0, ch2_tick period
;    .asg    R7.b2, ch2_status
;    .asg    R7.b3, ch2_data0
;    .asg    R8.b0, ch2_data1
;    .asg    R8.b1, ch2_data2
;    .asg    R8.b2, ch2_data3
;    .asg    R8.b3, ch2_data4
;    .asg    R9.b0, ch2_data5
;    .asg    R9.b1, ch2_crc
;    .asg    R9.b2, crc2_status
;   Channel3
;    .asg    R10.w1, ch3_tick period
;    .asg    R10.b2, ch3_status
;    .asg    R10.b3, ch3_data0
;    .asg    R11.b0, ch3_data1
;    .asg    R11.b1, ch3_data2
;    .asg    R11.b2, ch3_data3
;    .asg    R11.b3, ch3_data4
;    .asg    R12.b0, ch3_data5
;    .asg    R12.b1, ch3_crc
;    .asg    R12.b2, crc3_status
;Channel4(R13-R15)
;Channel5(R16-R18)
;Channel6(R19-R21)
;Channel7(R22-R24)
;-----------------------------------------------
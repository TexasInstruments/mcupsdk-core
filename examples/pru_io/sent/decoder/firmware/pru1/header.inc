; Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
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
;   Brief:    Sent decoder memory defines
;************************************************************************************

TICK_7_CONST            .set   613566757
    .asg    R0, TEMP_REG0
    .asg    R1.b0, BNS_ARG_RETVAL_ADDR
    ;.asg    R1.b1, unused
    .asg    R1.w2, BNS_CH_DATA_ERROR
    .asg    R2, STATE_CHANGE_MASK
    .asg    R3, CHx_PULSE_LEN
    ;Need: 4 reg for pulse length
    .asg    R4, BNS_LUT_VAL
    .asg    R5, BNS_ARG_LUTBASE
    .asg    R6, BNS_ARG_VAL

    .asg    R7.b0, BNS_ARG_RETVAL
    .asg    R7.b1, BNS_ARG_STATUS
    .asg    R7.w2, return_addr1
    .asg    R8.w0, return_addr2
    .asg    R8.w2, CH0_STATE
    .asg    R9.w0, CH1_STATE
    .asg    R9.w2, CH2_STATE
    .asg    R10.w0, CH3_STATE
    .asg    R10.w2, CH4_STATE
    .asg    R11.w0, CH5_STATE
    .asg    R11.w2, CH6_STATE
    .asg    R12.w0, CH7_STATE
    ;.asg    R12.w2, unused
    .asg    R13.b0, CH0_CRC4_RES
    .asg    R13.b1, CH1_CRC4_RES
    .asg    R13.b2, CH2_CRC4_RES
    .asg    R13.b3, CH3_CRC4_RES
    .asg    R14.b0, CH4_CRC4_RES
    .asg    R14.b1, CH5_CRC4_RES
    .asg    R14.b2, CH6_CRC4_RES
    .asg    R14.b3, CH7_CRC4_RES
    ;Need: 1 Reg for 4 crc res
    ;.asg    R15, unused
    .asg    R16, CH0_BUF
    .asg    R17, CH1_BUF
    .asg    R18, CH2_BUF
    .asg    R19, CH3_BUF
    .asg    R20, CH4_BUF
    .asg    R21, CH5_BUF
    .asg    R22, CH6_BUF
    .asg    R23, CH7_BUF
    ;Need 1 reg for Ch buf
    .asg    R24, APPROX_VAL_MAX
    ;asg R25.b0 is for mode of MAC module
    .asg    R25.w1, APPROX_VAL_MIN
    ;.asg    R25.b3, unused
;Reserved for mac module but can be used for TEMP REG
    .asg    R26, TEMP_REG
    .asg    R27, TEMP_REG1
    .asg    R28, TEMP_REG2
    .asg    R29, TEMP_REG3

CH0_SPAD_DATA_OFFSET .set					(0x40)
CH1_SPAD_DATA_OFFSET .set					(0x4C)
CH2_SPAD_DATA_OFFSET .set					(0x58)
CH3_SPAD_DATA_OFFSET .set					(0x64)
CH4_SPAD_DATA_OFFSET .set					(0x70)
CH5_SPAD_DATA_OFFSET .set					(0x7C)
CH6_SPAD_DATA_OFFSET .set					(0x88)
CH7_SPAD_DATA_OFFSET .set					(0x94)

PDMEM00 .set                                0x00
PDMME01 .set 								0x2000
ICSS_CFGx .set                              C5
CRC4_LUT_OFFSET      .set                   (0x0)
CRC4_LUT_SIZE        .set                   (0x100)
CRC4_SEED            .set                   (0x5)
CH0_LUT_BASE .set					(0x100)
CH1_LUT_BASE .set					(0x140)
CH2_LUT_BASE .set					(0x180)
CH3_LUT_BASE .set					(0x1C0)
CH4_LUT_BASE .set					(0x200)
CH5_LUT_BASE .set					(0x240)
CH6_LUT_BASE .set					(0x280)
CH7_LUT_BASE .set					(0x2C0)

CH0_DATA_BASE .set					(0x40)
CH1_DATA_BASE .set					(0x4C)
CH2_DATA_BASE .set					(0x58)
CH3_DATA_BASE .set					(0x64)
CH4_DATA_BASE .set					(0x70)
CH5_DATA_BASE .set					(0x7C)
CH6_DATA_BASE .set					(0x88)
CH7_DATA_BASE .set					(0x94)

CH0_INTR_BYTE .set					(0x00)
CH1_INTR_BYTE .set					(0x01)
CH2_INTR_BYTE .set					(0x02)
CH3_INTR_BYTE .set					(0x03)
CH4_INTR_BYTE .set					(0x04)
CH5_INTR_BYTE .set					(0x05)
CH6_INTR_BYTE .set					(0x06)
CH7_INTR_BYTE .set					(0x07)

    .if	$isdefed("SOC_AM263X")
CH0_MASK .set   					(0x00)
CH1_MASK .set   					(0x01)
CH2_MASK .set   					(0x02)
CH3_MASK .set   					(0x03)
CH4_MASK .set   					(0x04)
CH5_MASK .set   					(0x05)
CH6_MASK .set   					(0x06)
CH7_MASK .set   					(0x08)
    .elseif $isdefed("SOC_AM261X")
CH0_MASK .set   					(0x00)
CH1_MASK .set   					(0x01)
CH2_MASK .set   					(0x03)
CH3_MASK .set   					(0x06)
CH4_MASK .set   					(0x07)
CH5_MASK .set   					(0x08)
CH6_MASK .set   					(0x0B)
CH7_MASK .set   					(0x0D)
    .elseif $isdefed("SOC_AM263PX")
CH0_MASK .set   					(0x00)
CH1_MASK .set   					(0x01)
CH2_MASK .set   					(0x02)
CH3_MASK .set   					(0x03)
CH4_MASK .set   					(0x04)
CH5_MASK .set   					(0x05)
CH6_MASK .set   					(0x06)
CH7_MASK .set   					(0x08)
    .endif

CH0_REG .set   					(R1)
CH1_REG .set   					(R4)
CH2_REG .set   					(R7)
CH3_REG .set   					(R10)
CH4_REG .set   					(R13)
CH5_REG .set   					(R16)
CH6_REG .set   					(R19)
CH7_REG .set   					(R22)

CH0_PRU0_READ_ACK .set					(16)
CH1_PRU0_READ_ACK .set					(17)
CH2_PRU0_READ_ACK .set					(18)
CH3_PRU0_READ_ACK .set					(19)
CH4_PRU0_READ_ACK .set					(20)
CH5_PRU0_READ_ACK .set					(21)
CH6_PRU0_READ_ACK .set					(22)
CH7_PRU0_READ_ACK .set					(23)

CH0_PRU0_DATA_BASE .set					(17)
CH1_PRU0_DATA_BASE .set					(18)
CH2_PRU0_DATA_BASE .set					(19)
CH3_PRU0_DATA_BASE .set					(20)
CH4_PRU0_DATA_BASE .set					(21)
CH5_PRU0_DATA_BASE .set					(22)
CH6_PRU0_DATA_BASE .set					(23)
CH7_PRU0_DATA_BASE .set					(24)

CH0_DATA0_SPAD_BASE .set					(15)
CH1_DATA0_SPAD_BASE .set					(17)
CH2_DATA0_SPAD_BASE .set					(19)
CH3_DATA0_SPAD_BASE .set					(21)
CH4_DATA0_SPAD_BASE .set					(23)
CH5_DATA0_SPAD_BASE .set					(25)
CH6_DATA0_SPAD_BASE .set					(27)
CH7_DATA0_SPAD_BASE .set					(29)

CH0_DATA4_SPAD_BASE .set					(16)
CH1_DATA4_SPAD_BASE .set					(18)
CH2_DATA4_SPAD_BASE .set					(20)
CH3_DATA4_SPAD_BASE .set					(22)
CH4_DATA4_SPAD_BASE .set					(24)
CH5_DATA4_SPAD_BASE .set					(26)
CH6_DATA4_SPAD_BASE .set					(28)
CH7_DATA4_SPAD_BASE .set					(00)

CH0_CRCDATA_SPAD_BASE .set					(17)
CH1_CRCDATA_SPAD_BASE .set					(19)
CH2_CRCDATA_SPAD_BASE .set					(21)
CH3_CRCDATA_SPAD_BASE .set					(23)
CH4_CRCDATA_SPAD_BASE .set					(25)
CH5_CRCDATA_SPAD_BASE .set					(27)
CH6_CRCDATA_SPAD_BASE .set					(29)
CH7_CRCDATA_SPAD_BASE .set					(01)
;SCRATCH PAD 0
;--------------------------------------------
;    .asg    R2   final_state_change_mask
;    .asg    R3   ch0_pulse_length
;    .asg    R4   ch1_pulse_length
;    .asg    R5   ch2_pulse_length
;    .asg    R6   ch3_pulse_length
;    .asg    R7   ch4_pulse_length
;    .asg    R8   ch5_pulse_length
;    .asg    R9   ch6_pulse_length
;    .asg    R10  ch7_pulse_length
;    .asg    R11  Unused
;    .asg    R12  ch0_ack_status
;    .asg    R13, ch1_ack_status
;    .asg    R14, ch2_ack_status
;    .asg    R15, ch3_ack_status
;    .asg    R16, ch4_ack_status
;    .asg    R17, ch5_ack_status
;    .asg    R18, ch6_ack_status
;    .asg    R19, ch7_ack_status
;R20-R27: Reserved for storing ch pulse length

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
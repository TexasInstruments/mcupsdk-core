
;
; Copyright (C) 2021-23 Texas Instruments Incorporated
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
;offsets for ch0 (without loadshare mode use these ch0 offset for all three channels)
ENDAT_CH0_OPMODE_CONFIG_OFFSET	.set		0x0	; 0: PRU trigger 1: Host trigger
ENDAT_CH0_CHANNEL_CONFIG_OFFSET	.set		0x1	; Select single channel (1, 2, 3 or Multi (0))- Not implemented yet
ENDAT_CH0_INTFC_CMD_TRIGGER_OFFSET	.set	0x2 ;  Command trigger indication to firmware
ENDAT_CH0_INTFC_CMD_STATUS_OFFSET	.set	0x3 ;  Status word for channel busy indication etc - Not used now

;offsets for ch1 with loadshare mode, without loadshare mode no use for this offset configuration
ENDAT_CH1_OPMODE_CONFIG_OFFSET  .set        0x4 ; 0: PRU trigger 1: Host trigger
ENDAT_CH1_CHANNEL_CONFIG_OFFSET .set        0x5 ; Select single channel 1
ENDAT_CH1_INTFC_CMD_TRIGGER_OFFSET  .set    0x6 ;  Command trigger indication to firmware
ENDAT_CH1_INTFC_CMD_STATUS_OFFSET   .set    0x7 ;  Status word for channel busy indication etc - Not used now

;offsets for ch2 with loadshare mode,
ENDAT_CH2_OPMODE_CONFIG_OFFSET  .set        0x8 ; 0: PRU trigger 1: Host trigger
ENDAT_CH2_CHANNEL_CONFIG_OFFSET .set        0x9 ; Select single channel 2
ENDAT_CH2_INTFC_CMD_TRIGGER_OFFSET  .set    0xA ;  Command trigger indication to firmware
ENDAT_CH2_INTFC_CMD_STATUS_OFFSET   .set    0xB ;  Status word for channel busy indication etc - Not used now

;Command offset for ch0 and (command offset for all three channels in without load share mode)
ENDAT_CH0_CMD_WORD_0_OFFSET	.set			0xC ; Command parameters for firmware
ENDAT_CH0_CMD_WORD_0_CMDNADDR_OFFSET	.set	0xC ; 1-bit dummy +  6-bits command mode + 9 bits (address + 1-bit param)
ENDAT_CH0_CMD_WORD_0_PARAM_OFFSET	.set		0xE ; 15-bit params


ENDAT_CH0_CMD_WORD_1_OFFSET	.set			0x10 ; Command parameters for firmware
ENDAT_CH0_CMD_WORD_1_RXBITS_OFFSET	.set	0x10 ; Number of bits to receive in command response for ch1
ENDAT_CH0_CMD_WORD_1_TXBITS_OFFSET	.set	0x11 ; Number of bits to transmit to encoder
ENDAT_CH0_CMD_WORD_1_CMDTYP_OFFSET	.set	0x12 ; 0: EnDat2.2 otherwise: EnDat2.1 or ENDAT_CMD_SEND_POSVAL_WITH_DATA
ENDAT_CH0_CMD_WORD_1_BLKLEN_OFFSET	.set	0x13 ; Block address length in bytes, 0 if not present

ENDAT_CH0_CMD_WORD_2_OFFSET	.set			0x14 ; Command parameters for firmware - 2.2 command supplement
ENDAT_CH0_CMD_WORD_2_ADDR_OFFSET	.set	0x14 ;  8-bit address
ENDAT_CH0_CMD_WORD_2_PARAM_OFFSET	.set	0x15 ;  16-bit data
ENDAT_CH0_CMD_WORD_2_BLKADDR_OFFSET	.set	0x17 ;  8-bit optional block address

;Command offset for ch1, use only with loadshare
ENDAT_CH1_CMD_WORD_0_OFFSET .set            0x18 ; Command parameters for firmware
ENDAT_CH1_CMD_WORD_0_CMDNADDR_OFFSET    .set    0x18 ; 1-bit dummy +  6-bits command mode + 9 bits (address + 1-bit param)
ENDAT_CH1_CMD_WORD_0_PARAM_OFFSET   .set        0x1A ; 15-bit params


ENDAT_CH1_CMD_WORD_1_OFFSET .set            0x1C ; Command parameters for firmware
ENDAT_CH1_CMD_WORD_1_RXBITS_OFFSET  .set    0x1C ; Number of bits to receive in command response for ch1
ENDAT_CH1_CMD_WORD_1_TXBITS_OFFSET  .set    0x1D ; Number of bits to transmit to encoder
ENDAT_CH1_CMD_WORD_1_CMDTYP_OFFSET  .set    0x1E ; 0: EnDat2.2 otherwise: EnDat2.1 or ENDAT_CMD_SEND_POSVAL_WITH_DATA
ENDAT_CH1_CMD_WORD_1_BLKLEN_OFFSET  .set    0x1F ; Block address length in bytes, 0 if not present

ENDAT_CH1_CMD_WORD_2_OFFSET .set            0x20 ; Command parameters for firmware - 2.2 command supplement
ENDAT_CH1_CMD_WORD_2_ADDR_OFFSET    .set    0x20 ;  8-bit address
ENDAT_CH1_CMD_WORD_2_PARAM_OFFSET   .set    0x21 ;  16-bit data
ENDAT_CH1_CMD_WORD_2_BLKADDR_OFFSET .set    0x23 ;  8-bit optional block address

;Command offset for ch2, use only with loadshare
ENDAT_CH2_CMD_WORD_0_OFFSET .set            0x24 ; Command parameters for firmware
ENDAT_CH2_CMD_WORD_0_CMDNADDR_OFFSET    .set  0x24 ; 1-bit dummy +  6-bits command mode + 9 bits (address + 1-bit param)
ENDAT_CH2_CMD_WORD_0_PARAM_OFFSET   .set        0x26 ; 15-bit params


ENDAT_CH2_CMD_WORD_1_OFFSET .set            0x28 ; Command parameters for firmware
ENDAT_CH2_CMD_WORD_1_RXBITS_OFFSET  .set    0x28 ; Number of bits to receive in command response for ch1
ENDAT_CH2_CMD_WORD_1_TXBITS_OFFSET  .set    0x29 ; Number of bits to transmit to encoder
ENDAT_CH2_CMD_WORD_1_CMDTYP_OFFSET  .set    0x2A ; 0: EnDat2.2 otherwise: EnDat2.1 or ENDAT_CMD_SEND_POSVAL_WITH_DATA
ENDAT_CH2_CMD_WORD_1_BLKLEN_OFFSET  .set    0x2B ; Block address length in bytes, 0 if not present

ENDAT_CH2_CMD_WORD_2_OFFSET .set            0x2C ; Command parameters for firmware - 2.2 command supplement
ENDAT_CH2_CMD_WORD_2_ADDR_OFFSET    .set    0x2C ;  8-bit address
ENDAT_CH2_CMD_WORD_2_PARAM_OFFSET   .set    0x2D ;  16-bit data
ENDAT_CH2_CMD_WORD_2_BLKADDR_OFFSET .set    0x2F ;  8-bit optional block address


; NOTE: For ENDAT_CHx_POSITION_DATA_LSW_OFFSET: Bit0 is Error flag (EnDat2.1) and Bit0,Bit1 are Error flags (EnDat2.2)
; CH0 registers
ENDAT_CH0_POSITION_DATA_WORD0_OFFSET	.set	0x30
ENDAT_CH0_POSITION_DATA_WORD1_OFFSET	.set	0x34
ENDAT_CH0_POSITION_DATA_WORD2_OFFSET	.set	0x38
ENDAT_CH0_POSITION_DATA_WORD3_OFFSET	.set	0x3C
ENDAT_CH0_CRC_ERR_COUNTER_OFFSET	.set	  0X40  ; CRC error count
ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET	.set	 0X44
ENDAT_CH0_ENDAT22_STAT_OFFSET	.set		 0x45
ENDAT_CH0_CLOCK_LESS_FOR_TD	.set		0x46
ENDAT_CH0_MEAS_PROPDELAY_OFFSET	.set	  0x48

; CH1 registers
ENDAT_CH1_POSITION_DATA_WORD0_OFFSET	.set	0x50
ENDAT_CH1_POSITION_DATA_WORD1_OFFSET	.set	0x54
ENDAT_CH1_POSITION_DATA_WORD2_OFFSET	.set	0x58
ENDAT_CH1_POSITION_DATA_WORD3_OFFSET	.set	0X5C
ENDAT_CH1_CRC_ERR_COUNTER_OFFSET	.set	0x60 ; CRC error count
ENDAT_CH1_NUM_CLOCK_PULSES_OFFSET	.set	0x64
ENDAT_CH1_ENDAT22_STAT_OFFSET	.set		0x65
ENDAT_CH1_CLOCK_LESS_FOR_TD	.set		0x66
ENDAT_CH1_MEAS_PROPDELAY_OFFSET	.set		0x68

; CH2 registers
ENDAT_CH2_POSITION_DATA_WORD0_OFFSET	.set  0x70
ENDAT_CH2_POSITION_DATA_WORD1_OFFSET	.set	0x74
ENDAT_CH2_POSITION_DATA_WORD2_OFFSET	.set	0x78
ENDAT_CH2_POSITION_DATA_WORD3_OFFSET	.set	0x7C
ENDAT_CH2_CRC_ERR_COUNTER_OFFSET	.set	0x80  ; CRC error count
ENDAT_CH2_NUM_CLOCK_PULSES_OFFSET	.set	0x84
ENDAT_CH2_ENDAT22_STAT_OFFSET	.set		0x85
ENDAT_CH2_CLOCK_LESS_FOR_TD	.set		0x86
ENDAT_CH2_MEAS_PROPDELAY_OFFSET	.set	0x88

; clock configuration
ENDAT_CONFIG_CLOCK_RX_OFFSET	.set		0x90 ; clock to be configure for rx = tx * oversample rate
ENDAT_CONFIG_CLOCK_TX_OFFSET	.set		0x92 ; clock to be configure for tx
ENDAT_CONFIG_CLOCK_RX_EN_CNT_OFFSET	.set	0x94 ; rx arm delay

; Delay Configuration
ENDAT_CONFIG_DELAY_125NS_OFFSET  .set   0x98
ENDAT_CONFIG_DELAY_5US_OFFSET  .set    0x9C
ENDAT_CONFIG_DELAY_51US_OFFSET  .set    0xA0
ENDAT_CONFIG_DELAY_1MS_OFFSET   .set    0xA4
ENDAT_CONFIG_DELAY_2MS_OFFSET	.set	0xA8  ; delay for 2ms counter
ENDAT_CONFIG_DELAY_12MS_OFFSET	.set	0xAC  ; delay for 12ms counter
ENDAT_CONFIG_DELAY_50MS_OFFSET	.set	0xB0  ; delay for 20ms counter
ENDAT_CONFIG_DELAY_380MS_OFFSET  .set   0xB4
ENDAT_CONFIG_DELAY_900MS_OFFSET  .set   0XB8

; mask for core set
MASK_FOR_PRIMARY_CORE        .set    0xBC
;rx complete status
ENDAT_CH0_CONFIG_SYN_BIT        .set    0xBD ;0th bit for ch0
ENDAT_CH1_CONFIG_SYN_BIT        .set    0xBE ;0th bit for ch0
ENDAT_CH2_CONFIG_SYN_BIT        .set    0xBF ;0th bit for ch0
; Recovery Time
ENDAT_CH0_RT_OFFSET .set 0xC0 ;
ENDAT_CH1_RT_OFFSET .set 0xC4 ;
ENDAT_CH2_RT_OFFSET .set 0xC8 ;

;icssgclock
PRUICSSG_CLOCK         .set  0xCD;


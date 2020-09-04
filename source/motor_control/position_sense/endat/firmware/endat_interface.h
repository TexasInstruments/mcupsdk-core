
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

ENDAT_OPMODE_CONFIG_OFFSET	.set		0x0	; 0: PRU trigger 1: Host trigger
ENDAT_CHANNEL_CONFIG_OFFSET	.set		0x1	; Select single channel (1, 2, 3 or Multi (0))- Not implemented yet
ENDAT_INTFC_CMD_TRIGGER_OFFSET	.set	0x2 ;  Command trigger indication to firmware
ENDAT_INTFC_CMD_STATUS_OFFSET	.set	0x3 ;  Status word for channel busy indication etc - Not used now

ENDAT_CMD_WORD_0_OFFSET	.set			0x4 ; Command parameters for firmware
ENDAT_CMD_WORD_0_CMDNADDR_OFFSET	.set	0x4 ; 1-bit dummy +  6-bits command mode + 9 bits (address + 1-bit param)
ENDAT_CMD_WORD_0_PARAM_OFFSET	.set		0x6 ; 15-bit params


ENDAT_CMD_WORD_1_OFFSET	.set			0x8 ; Command parameters for firmware
ENDAT_CMD_WORD_1_RXBITS_OFFSET	.set	0x8 ; Number of bits to receive in command response
ENDAT_CMD_WORD_1_TXBITS_OFFSET	.set	0x9 ; Number of bits to transmit to encoder
ENDAT_CMD_WORD_1_CMDTYP_OFFSET	.set	0xA ; 0: EnDat2.2 otherwise: EnDat2.1 or ENDAT_CMD_SEND_POSVAL_WITH_DATA
ENDAT_CMD_WORD_1_BLKLEN_OFFSET	.set	0xB ; Block address length in bytes, 0 if not present

ENDAT_CMD_WORD_2_OFFSET	.set			0xC ; Command parameters for firmware - 2.2 command supplement
ENDAT_CMD_WORD_2_ADDR_OFFSET	.set	0xC ;  8-bit address
ENDAT_CMD_WORD_2_PARAM_OFFSET	.set	0xD ;  16-bit data
ENDAT_CMD_WORD_2_BLKADDR_OFFSET	.set	0xF ;  8-bit optional block address

; NOTE: For ENDAT_CHx_POSITION_DATA_LSW_OFFSET: Bit0 is Error flag (EnDat2.1) and Bit0,Bit1 are Error flags (EnDat2.2)
; CH0 registers
ENDAT_CH0_POSITION_DATA_WORD0_OFFSET	.set	0x10
ENDAT_CH0_POSITION_DATA_WORD1_OFFSET	.set	0x14
ENDAT_CH0_POSITION_DATA_WORD2_OFFSET	.set	0x18
ENDAT_CH0_POSITION_DATA_WORD3_OFFSET	.set	0x1C
ENDAT_CH0_CRC_ERR_COUNTER_OFFSET	.set	0x20 ; CRC error count
ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET	.set	0x24
ENDAT_CH0_ENDAT22_STAT_OFFSET	.set		0x25
ENDAT_CH0_CLOCK_LESS_FOR_TD	.set		0x26
ENDAT_CH0_MEAS_PROPDELAY_OFFSET	.set		0x28

; CH1 registers
ENDAT_CH1_POSITION_DATA_WORD0_OFFSET	.set	0x30
ENDAT_CH1_POSITION_DATA_WORD1_OFFSET	.set	0x34
ENDAT_CH1_POSITION_DATA_WORD2_OFFSET	.set	0x38
ENDAT_CH1_POSITION_DATA_WORD3_OFFSET	.set	0x3C
ENDAT_CH1_CRC_ERR_COUNTER_OFFSET	.set	0x40 ; CRC error count
ENDAT_CH1_NUM_CLOCK_PULSES_OFFSET	.set	0x44
ENDAT_CH1_ENDAT22_STAT_OFFSET	.set		0x45
ENDAT_CH1_CLOCK_LESS_FOR_TD	.set		0x46
ENDAT_CH1_MEAS_PROPDELAY_OFFSET	.set		0x48

; CH2 registers
ENDAT_CH2_POSITION_DATA_WORD0_OFFSET	.set	0x50
ENDAT_CH2_POSITION_DATA_WORD1_OFFSET	.set	0x54
ENDAT_CH2_POSITION_DATA_WORD2_OFFSET	.set	0x58
ENDAT_CH2_POSITION_DATA_WORD3_OFFSET	.set	0x5C
ENDAT_CH2_CRC_ERR_COUNTER_OFFSET	.set	0x60 ; CRC error count
ENDAT_CH2_NUM_CLOCK_PULSES_OFFSET	.set	0x64
ENDAT_CH2_ENDAT22_STAT_OFFSET	.set		0x65
ENDAT_CH2_CLOCK_LESS_FOR_TD	.set		0x66
ENDAT_CH2_MEAS_PROPDELAY_OFFSET	.set		0x68

; clock configuration
ENDAT_CONFIG_CLOCK_RX_OFFSET	.set		0x70 ; clock to be configure for rx = tx * oversample rate
ENDAT_CONFIG_CLOCK_TX_OFFSET	.set		0x72 ; clock to be configure for tx
ENDAT_CONFIG_CLOCK_RX_EN_CNT_OFFSET	.set	0x74 ; rx arm delay

; Delay Configuration
ENDAT_CONFIG_DELAY_2MS_OFFSET	.set	0x78 ; delay for 2ms counter
ENDAT_CONFIG_DELAY_12MS_OFFSET	.set	0x7C ; delay for 12ms counter
ENDAT_CONFIG_DELAY_50MS_OFFSET	.set	0x80 ; delay for 20ms counter

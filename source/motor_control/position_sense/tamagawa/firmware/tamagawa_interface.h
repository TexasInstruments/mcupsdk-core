; Copyright (C) 2022 Texas Instruments Incorporated
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
 ;


	.if	!$isdefed("__tamagawa_interface_h")
__tamagawa_interface_h	.set	1

TAMAGAWA_OPMODE_CONFIG_OFFSET      .set    0x0
TAMAGAWA_CHANNEL_CONFIG_OFFSET     .set    0x1 ;/* Select single channel (1, 2, 3 or Multi (0))- Not implemented yet*/
TAMAGAWA_INTFC_CMD_TRIGGER_OFFSET  .set    0x2 ;/*  Command trigger indication to firmware*/
TAMAGAWA_INTFC_CMD_STATUS_OFFSET   .set    0x3 ;/*  Status word for channel busy indication etc - Not used now*/

TAMAGAWA_WORD_0_OFFSET              .set    0x4    ;/*Tx Data without start and stop bits*/
TAMAGAWA_WORD_1_OFFSET              .set    0x8    ;/*No. of Tx frames,Rx frames expected (first byte is Tx frames second is Rx frames)*/

TAMAGAWA_CH0_POSITION_DATA_WORD0_OFFSET   .set 0xC     ;/*Rx position data for ch0 */
TAMAGAWA_CH0_POSITION_DATA_WORD1_OFFSET    .set 0x10   ;
TAMAGAWA_CH0_POSITION_DATA_WORD2_OFFSET    .set 0x14   ;
TAMAGAWA_CH0_CRC_OFFSET        .set 0x18

TAMAGAWA_CH1_POSITION_DATA_WORD0_OFFSET   .set 0x1C     ;/*Rx position data for ch1 */
TAMAGAWA_CH1_POSITION_DATA_WORD1_OFFSET    .set 0x20   ;
TAMAGAWA_CH1_POSITION_DATA_WORD2_OFFSET    .set 0x24   ;
TAMAGAWA_CH1_CRC_OFFSET        .set 0x28    ;

TAMAGAWA_CH2_POSITION_DATA_WORD0_OFFSET   .set 0x2C     ;/*Rx position data for ch2 */
TAMAGAWA_CH2_POSITION_DATA_WORD1_OFFSET    .set 0x30   ;
TAMAGAWA_CH2_POSITION_DATA_WORD2_OFFSET    .set 0x34   ;
TAMAGAWA_CH2_CRC_OFFSET         .set 0x38   ;

TAMAGAWA_CHANNEL_MASK_OFFSET            .set 0x3C     ;/*mask for getting what channels are required*/

TAMAGAWA_RX_DIV_FACTOR_OFFSET           .set 0x40     ;/*rx divide factor offset*/
TAMAGAWA_TX_DIV_FACTOR_OFFSET           .set 0x44      ;/*tx divide factor offset*/
TAMAGAWA_OVERSAMPLE_RATE_OFFSET         .set 0x48       ;/*oversample rate offset*/

TAMAGAWA_EEPROM_CMD_OFFSET                .set 0x68       ;/*tamagawa eeprom command id offset*/
TAMAGAWA_EEPROM_ADF_OFFSET               .set 0x6C       ;/*tamagawa eeprom adf offset*/
TAMAGAWA_EEPROM_EDF_OFFSET               .set 0x70       ;/*tamagawa eeprom edf offset*/
TAMAGAWA_EEPROM_CRC_OFFSET               .set 0x74       ;/*tamagawa eeprom crc offset*/
TAMAGAWA_EEPROM_TX_CMD_0_CH0         .set 0x88       ;/*tamagawa eeprom tx command offset for bytes 0-3 for channel 0*/
TAMAGAWA_EEPROM_TX_CMD_1_CH0         .set 0x8C       ;/*tamagawa eeprom tx command offset for bytes 4-7 for channel 0*/
TAMAGAWA_EEPROM_TX_CMD_0_CH1         .set 0xB0       ;/*tamagawa eeprom tx command offset for bytes 0-3 for channel 1*/
TAMAGAWA_EEPROM_TX_CMD_1_CH1         .set 0xB4       ;/*tamagawa eeprom tx command offset for bytes 4-7 for channel 1*/
TAMAGAWA_EEPROM_TX_CMD_0_CH2         .set 0xD8       ;/*tamagawa eeprom tx command offset for bytes 0-3 for channel 2*/
TAMAGAWA_EEPROM_TX_CMD_1_CH2         .set 0xDC       ;/*tamagawa eeprom tx command offset for bytes 4-7 for channel 2*/
	.endif

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


;************************************************************************************
;*   File:     eeprom_write.h											            *
;*																					*
;*   Brief:    Macro for performing EEPROM Write on Tamagawa encoder				*
;************************************************************************************

;*************************************************************************************************************************************
;   Macro: EEPROM_WRITE_MACRO
;
;   		Perform the EEPROM Read operation on the Tamagawa encoders.
;
;	Registers:
;						R25- Used to store the Tx data to be sent to the encoder
;						R26.b0- Used to store the offset of the 64 bit integer holding Tx data
;						R30.b0- Used to load Tx FIFO for a selected channel
;                       R30.b2- Used to store the currently selected channel
;                       R31- Used to monitor the Tx FIFO level for different channels
;
;
;
;   PseudoCode:
;      (start code)
;		1.Check if the current channel is enabled.
;       2.Load the offset of 5th byte of the 64 bit integer holding Tx data to R26.b0 for the currently selected channel.
;		3.Load 1 byte of data from the offset stored in R26.b0 to R25.
;		4.Load the 5th byte of Tx data to the Tx FIFO for the currently seelcted channel by writing to R30.b0 once.
;		5.Load the offset of 0th byte of the 64 bit integer holding Tx data to R26.b0 for the currently selected channel.
;		6.Load 4 bytes of data from the offset stored in R26.b0 to R25.
;       7.Load the Tx FIFO for the currently selected channel by writing to the R30.b0 byte-wise 3 times.
;       8.Loop through the steps 1-7 thrice in case of multi-channel configuration to fill Tx FIFO for all selected channels.
;       9.Once the Tx FIFO of all the selected channels is loaded with 4 bytes, make a call to the FN_SEND function that performs Tx in Continuous mode.
;       10.Monitor the Tx FIFO level for the current channel until the FIFO level reaches 3 bytes or less.
;       11.Load the last byte of Tx data into the Tx FIFO by writing to the R30.b0 once.
;       12.Repeat the steps 10-11 for all the selected channels.
;		(end code)
;
;   Parameters:
;       ch_mask - Channel mask, used to check if the current channel is enabled
;		pru_dmem -	Data Memory address for PRU
;
;   Worst case peak cycle usage:   750 (aprpox.)
;   For EEPROM Write, the Control Field (CF), Address (ADF), Data (EDF) and CRC frames should be transmitted to the encoder in about 8.0 μs and 16.0 μs for 5 Mbps and 2.5 Mbps Tamagawa Encoders respectively.
;
;**************************************************************************************************************************************

	.if	!$isdefed("__eeprom_write_h")
__eeprom_write_h	.set	1

	.include "tamagawa_interface.h"
    .include "tamagawa_constants.h"
    .global FN_SEND
    .global CALL
    .global CALL2

EEPROM_WRITE_MACRO   .macro  ch_mask, pru_dmem

    ;In case of EEPROM Write, there is a need to send 40 bits of data to the Encoder
    ;First 32 bits of data is loaded into the Tx FIFO and Tx in Continuous FIFO loading is initiated
    ;Keep polling r31[20-18, 12-10, 4-2] (tx_fifo_sts<m>) and reload the FIFO with the last 8 bits when the FIFO level reaches 3 bytes

	;In case of EEPROM write, we need to first load the
	;value stored at the 5th byte of the 64 bit integer used to store the Tx data.

    .if	$defined("ENABLE_MULTI_CHANNEL")
    ;In each iteration, channel is selected and Tx Fifo is loaded .
    LOOP	EEPROM_WRITE_MULTI_CHANNEL?,	3
	.endif

    ; if channel 0 is enabled, use the Tx command offset for ch 0
    QBBC    TAMAGAWA_WRITE_SKIP_CH0?, ch_mask,	0
    ; Check if the current channel is 0
    AND    R26.b0, R30.b2, CHANNEL_CHECK_MASK
    ; If the current channel is not 0, skip loading the offset for Channel 0
    QBNE    TAMAGAWA_WRITE_SKIP_CH0?, R26.b0,	0
    ; Load the offset of the address where EEPROM Tx command's 5th byte is stored
    LDI     R26.b0, TAMAGAWA_EEPROM_TX_CMD_1_CH0
    ;load the 5th byte of data from the offset stored in R26.b0 to R25
    LBCO	&R25.b0,	pru_dmem,	R26.b0,	1
    ;load Tx Fifo
    MOV R30.b0,R25.b0
    ; Load the offset of the address where EEPROM Tx command is stored
    LDI     R26.b0, TAMAGAWA_EEPROM_TX_CMD_0_CH0
    ;load 4 bytes of data from the offset stored in R26.b0 to R25
    LBCO	&R25,	pru_dmem,	R26.b0,	4
    ;load Tx Fifo
    MOV R30.b0,R25.b3
    ;load Tx Fifo
    MOV R30.b0,R25.b2
    ;load Tx Fifo
    MOV R30.b0,R25.b1

    JMP     TAMAGAWA_EEPROM_LOADED_BYTE1?

TAMAGAWA_WRITE_SKIP_CH0?:
    ; if channel 1 is enabled, use the Tx command offset for ch 1
    QBBC    TAMAGAWA_WRITE_SKIP_CH1?, TAMAGAWA_ENABLE_CHx,	1
    ; Check if the current channel is 1
    AND    R26.b0, R30.b2, CHANNEL_CHECK_MASK
    ; If the current channel is not 1, skip loading the offset for Channel 1
    QBNE    TAMAGAWA_WRITE_SKIP_CH1?, R26.b0,	1
    ; Load the offset of the address where EEPROM Tx command's 5th byte is stored
    LDI     R26.b0, TAMAGAWA_EEPROM_TX_CMD_1_CH1
    ;load the 5th byte of data from the offset stored in R26.b0 to R25
    LBCO	&R25.b0,	pru_dmem,	R26.b0,	1
    ;load Tx Fifo
    MOV R30.b0,R25.b0
    ; Load the offset of the address where EEPROM Tx command is stored
    LDI     R26.b0, TAMAGAWA_EEPROM_TX_CMD_0_CH1
    ;load 4 bytes of data from the offset stored in R26.b0 to R25
    LBCO	&R25,	pru_dmem,	R26.b0,	4
    ;load Tx Fifo
    MOV R30.b0,R25.b3
    ;load Tx Fifo
    MOV R30.b0,R25.b2
    ;load Tx Fifo
    MOV R30.b0,R25.b1

    JMP     TAMAGAWA_EEPROM_LOADED_BYTE1?

TAMAGAWA_WRITE_SKIP_CH1?:
    ; if channel 2 is enabled, use the Tx command offset for ch 2
    QBBC    TAMAGAWA_WRITE_SKIP_CH2?, TAMAGAWA_ENABLE_CHx,	2
    ; Check if the current channel is 2
    AND    R26.b0, R30.b2, CHANNEL_CHECK_MASK
    ; If the current channel is not 1, skip loading the offset for Channel 2
    QBNE    TAMAGAWA_WRITE_SKIP_CH2?, R26.b0,	2
    ; Load the offset of the address where EEPROM Tx command's 5th byte is stored
    LDI     R26.b0, TAMAGAWA_EEPROM_TX_CMD_1_CH2
    ;load the 5th byte of data from the offset stored in R26.b0 to R25
    LBCO	&R25.b0,	pru_dmem,	R26.b0,	1
    ;load Tx Fifo
    MOV R30.b0,R25.b0
    ; Load the offset of the address where EEPROM Tx command is stored
    LDI     R26.b0, TAMAGAWA_EEPROM_TX_CMD_0_CH2
    ;load 4 bytes of data from the offset stored in R26.b0 to R25
    LBCO	&R25,	pru_dmem,	R26.b0,	4
    ;load Tx Fifo
    MOV R30.b0,R25.b3
    ;load Tx Fifo
    MOV R30.b0,R25.b2
    ;load Tx Fifo
    MOV R30.b0,R25.b1

    JMP     TAMAGAWA_EEPROM_LOADED_BYTE1?
TAMAGAWA_WRITE_SKIP_CH2?:

TAMAGAWA_EEPROM_LOADED_BYTE1?:

EEPROM_WRITE_END?:

    .if	$defined("ENABLE_MULTI_CHANNEL")
    ;	 last channel would have been selected first
    ;R30[17:16] tx_ch_sel is decremented to select next channel and load Tx Fifo.
	SUB		R30.b2,	R30.b2,	1
EEPROM_WRITE_MULTI_CHANNEL?:
	.endif

    ;	Sending Tx Fifo data via Data line(PERIF0_OUT,PERIF1_OUT,PERIF2_OUT) to encoders
    CALL2		FN_SEND

;EEPROM_WRITE_CONT?:
    ; Clear the R26 register in order to monitor the Tx FIFO level for different channels
    ZERO &R26.b0,4
    ; Move the value of number of channels selected to R30.b2
    MOV R30.b2, R10.b0

    .if	$defined("ENABLE_MULTI_CHANNEL")
    ;In each iteration, channel is selected and Tx Fifo is loaded .
    LOOP	TAMAGAWA_EEPROM_CONTINUE_MULTI_CH?,	3
	.endif

WAITLABEL0?:
    ; if channel 0 is not enabled, skip polling the bits 2-4 of R31.
    QBBC    TAMAGAWA_EEPROM_SKIP_CH0?, TAMAGAWA_ENABLE_CHx,	0
    ; Check if the current channel is 0
    AND    R26.b0, R30.b2, CHANNEL_CHECK_MASK
    ; If the current channel is not 0, skip polling the bits 2-4 of R31
    QBNE    TAMAGAWA_EEPROM_SKIP_CH0?, R26.b0,	0
    ; Monitor the bits 2-4 of R31 to get the Tx FIFO Level
    AND R26.b0,R31.b0,POLLING_MASK
    ;Keep waiting till the Tx FIFO level is not at 3 bytes
    QBLT WAITLABEL0?, R26.b0, THREE_BYTES_FIFO_LEVEL

    ; Load the offset of the Tx Command
    LDI     R26.b0, TAMAGAWA_EEPROM_TX_CMD_0_CH0
    ; Load 1 byte of data from TAMAGAWA_EEPROM_TX_CMD to R25.b0
    LBCO	&R25.b0,	pru_dmem,	R26.b0,	1
    ;load Tx Fifo
    MOV R30.b0,R25.b0
    JMP TAMAGAWA_EEPROM_LOADED_BYTE4?
TAMAGAWA_EEPROM_SKIP_CH0?:

WAITLABEL1?:
    ; if channel 1 is not enabled, skip polling the bits 10-12 of R31.
    QBBC    TAMAGAWA_EEPROM_SKIP_CH1?, TAMAGAWA_ENABLE_CHx,	1
    ; Check if the current channel is 1
    AND    R26.b0, R30.b2, CHANNEL_CHECK_MASK
    ; If the current channel is not 1, skip polling the bits 10-12 of R31
    QBNE    TAMAGAWA_EEPROM_SKIP_CH1?, R26.b0,	1
    ; Monitor the bits 10-12 of R31 to get the Tx FIFO Level
    AND R26.b0,R31.b1,POLLING_MASK
    ;Keep waiting till the Tx FIFO level is not at 3 bytes
    QBLT WAITLABEL1?, R26.b0, THREE_BYTES_FIFO_LEVEL

    ; Load the offset of the Tx Command
    LDI     R26.b0, TAMAGAWA_EEPROM_TX_CMD_0_CH1
    ; Load 1 byte of data from TAMAGAWA_EEPROM_TX_CMD to R25.b0
    LBCO	&R25.b0,	pru_dmem,	R26.b0,	1
    ;load Tx Fifo
    MOV R30.b0,R25.b0
    JMP TAMAGAWA_EEPROM_LOADED_BYTE4?
TAMAGAWA_EEPROM_SKIP_CH1?:

WAITLABEL2?:
    ; if channel 2 is not enabled, skip polling the bits 18-20 of R31.
    QBBC    TAMAGAWA_EEPROM_SKIP_CH2?, TAMAGAWA_ENABLE_CHx,	2
    ; Check if the current channel is 1
    AND    R26.b0, R30.b2, CHANNEL_CHECK_MASK
    ; If the current channel is not 1, skip polling the bits 18-20 of R31
    QBNE    TAMAGAWA_EEPROM_SKIP_CH2?, R26.b0,	2
    ; Monitor the bits 18-20 of R31 to get the Tx FIFO Level
    AND R26.b0,R31.b2,POLLING_MASK
    ;Keep waiting till the Tx FIFO level is not at 3 bytes
    QBLT WAITLABEL2?, R26.b0, THREE_BYTES_FIFO_LEVEL

    ; Load the offset of the Tx Command
    LDI     R26.b0, TAMAGAWA_EEPROM_TX_CMD_0_CH2
    ; Load 1 byte of data from TAMAGAWA_EEPROM_TX_CMD to R25.b0
    LBCO	&R25.b0,	pru_dmem,	R26.b0,	1
    ;load Tx Fifo
    MOV R30.b0,R25.b0
    JMP TAMAGAWA_EEPROM_LOADED_BYTE4?
TAMAGAWA_EEPROM_SKIP_CH2?:

TAMAGAWA_EEPROM_LOADED_BYTE4?:

    .if	$defined("ENABLE_MULTI_CHANNEL")
    ;	 last channel would have been selected first
    ;R30[17:16] tx_ch_sel is decremented to select next channel and load Tx Fifo.
	SUB		R30.b2,	R30.b2,	1
TAMAGAWA_EEPROM_CONTINUE_MULTI_CH?:
	.endif

	.endm

	.endif

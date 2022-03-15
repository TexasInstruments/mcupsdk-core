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
;*   File:     eeprom_read.h											            *
;*																					*
;*   Brief:    Macro for performing EEPROM Read on Tamagawa encoder				    *
;************************************************************************************

;*************************************************************************************************************************************
;   Macro: EEPROM_READ_MACRO
;
;   		Perform the EEPROM Read operation on the Tamagawa encoders.
;
;	Registers:
;						R25- Used to store the Tx data to be sent to the encoder
;						R26.b0- Used to store the offset of the 64 bit integer holding Tx data
;   					R30.b0- Used to load Tx FIFO for a selected channel
;                       R30.b2- Used to store the currently selected channel
;
;
;
;   PseudoCode:
;      (start code)
;		1.Check if the current channel is enabled.
;       2.Load the offset of the 64 bit integer holding Tx data to R26.b0 for the currently selected channel.
;		3.Load 4 bytes of data from the offset stored in R26.b0 to R25.
;		4.Load the Tx FIFO for the currently selected channel by writing to the R30.b0 byte-wise 4 times.
;		5.Loop through the steps 1-4 thrice in case of multi-channel configuration to fill Tx FIFO for all selected channels.
;		6.Once the Tx FIFO of all the selected channels is loaded with 4 bytes, make a call to the FN_SEND function that performs Tx in single shot mode with Tx frame size = 30 bits.
;		(end code)
;
;   Parameters:
;       ch_mask - Channel mask, used to check if the current channel is enabled
;		pru_dmem -	Data Memory address for PRU
;
;   Worst case peak cycle usage:   110 (approx.)
;   For EEPROM Read, the Control Field (CF), Address (ADF) and CRC frames should be transmitted to the encoder in about 6.0 μs and 12.0 μs for 5 Mbps and 2.5 Mbps Tamagawa Encoders respectively..
;
;**************************************************************************************************************************************

	.if	!$isdefed("__eeprom_read_h")
__eeprom_read_h	.set	1

	.include "tamagawa_interface.h"
    .include "tamagawa_constants.h"
    .global FN_SEND
    .global CALL
    .global CALL2

EEPROM_READ_MACRO   .macro  ch_mask, pru_dmem

    ;In case of EEPROM Read, there is a need to send 30 bits of data to the Encoder
    ;32 bits of data is first loaded into the Tx FIFO and Tx in Single shot mode is started with Tx frame size = 30 bits

    .if	$defined("ENABLE_MULTI_CHANNEL")
	LOOP	EEPROM_READ_MULTI_CHANNEL?,	3	;In each iteration, channel is selected and Tx Fifo is loaded .
	.endif

    ; if channel 0 is enabled, use the Tx command offset for ch 0.
    QBBC    TAMAGAWA_CH_SEL_SKIP_CH0?, ch_mask,	0
    ; Check if the current channel is 0
    AND     R26.b0, R30.b2, CHANNEL_CHECK_MASK
    ; If the current channel is not 0, skip loading the offset for Channel 0
    QBNE    TAMAGAWA_CH_SEL_SKIP_CH0?, R26.b0,	0
    ; Load the offset of the address where EEPROM Tx command is stored
    LDI     R26.b0, TAMAGAWA_EEPROM_TX_CMD_0_CH0
    JMP     TAMAGAWA_LOADED_OFFSET?

TAMAGAWA_CH_SEL_SKIP_CH0?:
    ; if channel 1 is enabled, use the Tx command offset for ch 1
    QBBC    TAMAGAWA_CH_SEL_SKIP_CH1?, ch_mask,	1
    ; Check if the current channel is 1
    AND     R26.b0, R30.b2, CHANNEL_CHECK_MASK
    ; If the current channel is not 1, skip loading the offset for Channel 1
    QBNE    TAMAGAWA_CH_SEL_SKIP_CH1?, R26.b0,	1
    ; Load the offset of the address where EEPROM Tx command is stored
    LDI     R26.b0, TAMAGAWA_EEPROM_TX_CMD_0_CH1
    JMP     TAMAGAWA_LOADED_OFFSET?

TAMAGAWA_CH_SEL_SKIP_CH1?:
    ; if channel 2 is enabled, use the Tx command offset for ch 2
    QBBC    TAMAGAWA_CH_SEL_SKIP_CH2?, ch_mask,	2
    ; Check if the current channel is 2
    AND     R26.b0, R30.b2, CHANNEL_CHECK_MASK
    ; If the current channel is not 2, skip loading the offset for Channel 2
    QBNE    TAMAGAWA_CH_SEL_SKIP_CH2?, R26.b0,	2
    ; Load the offset of the address where EEPROM Tx command is stored
    LDI     R26.b0, TAMAGAWA_EEPROM_TX_CMD_0_CH2
    JMP     TAMAGAWA_LOADED_OFFSET?

TAMAGAWA_CH_SEL_SKIP_CH2?:

TAMAGAWA_LOADED_OFFSET?:
    ;load 4 bytes of data from the TAMAGAWA_EEPROM_TX_CMD offset to R25
	LBCO	&R25,	pru_dmem,	R26.b0,	4

    ;load Tx Fifo
    MOV R30.b0, R25.b3
    ;load Tx Fifo
    MOV R30.b0, R25.b2
    ;load Tx Fifo
    MOV R30.b0, R25.b1
    ;load Tx Fifo
    MOV R30.b0, R25.b0


    .if	$defined("ENABLE_MULTI_CHANNEL")
    ;	 last channel would have been selected first
	;R30[17:16] tx_ch_sel is decremented to select next channel and load Tx Fifo.
    SUB		R30.b2,	R30.b2,	1
EEPROM_READ_MULTI_CHANNEL?:
	.endif

    ;	Sending Tx Fifo data via Data line(PERIF0_OUT,PERIF1_OUT,PERIF2_OUT) to encoders
    CALL2		FN_SEND

	.endm

	.endif

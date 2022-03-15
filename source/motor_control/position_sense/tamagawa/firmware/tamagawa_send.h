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
;*   File:     tamagawa_send.h											            *
;*																					*
;*   Brief:    Macro for sending commands to the Tamagawa encoder				    *
;************************************************************************************

;*************************************************************************************************************************************
;   Macro: TAMAGAWA_SEND_MACRO
;
;   		Send the Tx data for non-EEPROM commands to the Tamagawa encoders.
;
;	Registers:
;						R27.b0- Stores the first byte of the TX data
;						R27.b1- Stores the second byte of the TX data
;						R30.b0- Used to load Tx FIFO for a selected channel
;                       R30.b2- Used to select the channel (R30[17:16])
;
;
;
;   PseudoCode:
;      (start code)
;		1.Load the Tx FIFO with the first byte for the currently selected channel by writing to the R30.b0.
;       2.Load the Tx FIFO with the second byte for the currently selected channel by writing to the R30.b0.
;		3.Loop through the steps 1-4 thrice in case of multi-channel configuration to fill Tx FIFO for all selected channels.
;		4.Once the Tx FIFO of all the selected channels is loaded with 2 bytes, make a call to the FN_SEND function that performs Tx in single shot mode with Tx frame size = 10 bits.
;		(end code)
;
;   Parameters:
;       tx_data0 - Stores the first byte of the tx data to be sent to the encoder
;       tx_data1 - Stores the second byte of the tx data to be sent to the encoder
;
;   Worst case peak cycle usage:   68 (approx.)
;   For all commands except EEPROM Read and Write, the Control Field (CF) should be transmitted to the encoder in about 2.0 +/- 0.05 μs and 4.0 +/- 0.1 μs for 5 Mbps and 2.5 Mbps Tamagawa encoders respectively.
;
;**************************************************************************************************************************************

	.if	!$isdefed("__tamagawa_send_h")
__tamagawa_send_h	.set	1

	.include "tamagawa_interface.h"
    .include "tamagawa_constants.h"
    .global FN_SEND
    .global CALL
    .global CALL2

TAMAGAWA_SEND_MACRO   .macro     tx_data0, tx_data1
    .if	$defined("ENABLE_MULTI_CHANNEL")
    ;In each iteration, channel is selected and Tx Fifo is loaded .
    LOOP	SEND_RECEIVE_TAMAGAWA_MULTI_CHANNEL?,	3
	.endif
    ;In case of non EEPROM commands, we need to send the control field to the encoder. This control field is of 10 bits.
    ;The FIFO is loaded with 16 bits first and then a call to the FN_SEND is made in order to start Tx in Single shot mode with Tx frame size = 10 bits.
    ;load Tx FIFO
    MOV R30.b0,tx_data0
    ;load Tx FIFO
    MOV R30.b0,tx_data1

    .if	$defined("ENABLE_MULTI_CHANNEL")
    ;	 last channel would have been selected first
    ;R30[17:16] tx_ch_sel is decremented to select next channel and load Tx Fifo.
    SUB		R30.b2,	R30.b2,	1
SEND_RECEIVE_TAMAGAWA_MULTI_CHANNEL?:
	.endif
    ;	Sending Tx Fifo data via Data line(PERIF0_OUT,PERIF1_OUT,PERIF2_OUT) to encoders
    CALL2		FN_SEND

	.endm

	.endif

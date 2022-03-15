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
;*   File:     single_ch_receive_frames.h											*
;*																					*
;*   Brief:    Macro for receiving data frames from Tamagawa encoder				*
;************************************************************************************

;*************************************************************************************************************************************
;   Macro: SINGLE_CH_RECEIVE_DATA_FRAME
;			Receive complete data frame in Intermediate Register of particular channel
;
;
;	Registers:
;						R11.b0-Intermediate Frame register for CH0 or CH1 or CH2
;						R12-Intermediate channel register for CH0 or CH1 or CH2
;						R16-R18: Channel registers for CH0 or CH1 or CH2
;
;
;
;   PseudoCode:
;      (start code)
;		1.Perform LSL by 8 on Intermediate channel register for making b0 level(R12.b0) available for upcoming frame.
;		2.Wait till first valid bit flag raised.
;		3.Store the mid data bit from oversampled data for respective channel in intermediate frame register at MSB position
;		4.Perform LSR to shift right Intermediate frame register's bit by 1
;		5.clear the valid flag.
;       6.Repeat instructions from 2-5 until all bits for 1 data frame received in Intermediate frame register.
;		7.Transfer intermediate frame register(R11.b0) value to Intermediate channel Register(R12) for channel at b0 level(R12.b0).
;		(end code)
;
;   Parameters:
;
;		Ry0 -	Intermediate Frame register
;		Rx0 -	Intermediate channel register
;		cnt  -		size of data frame
;		scratch -	temporary variable
;		scratch1 -	temporary variable
;		valid_mask - Mask of channels which are enabled
;		bit_idx -	Mid bit of particular channel's oversample rx data that need to be captured for exact Rx data bit.
;
;   Worst case peak cycle usage:   8480 (approx.)
;
;**************************************************************************************************************************************

	.if	!$isdefed("__single_ch_receive_frames_h")
__single_ch_receive_frames_h	.set	1

	.include "tamagawa_interface.h"

SINGLE_CH_RECEIVE_DATA_FRAME    .macro  Ry0, Rx0, cnt ,scratch, scratch1, valid_mask, bit_idx,temp1,temp2,val,crc,rx_frames,dmem,crc_offset

    ;To Shift 1 byte left in Intermediate register for loading incoming rx frame of 1 byte at Rx0.b0 position.
    LSL     Rx0 ,Rx0 ,8
    ;Run loop for receive bits for each data frame
    LOOP    RX_RECEIVE_DOWNSAMPLE_LOOP1?,    cnt
    ;Shift 1 position right for align the bit to LSB position (As we are receiving bit sequence in LSB bit first mode for each frame).
    LSR     Ry0, Ry0, 1

    ; This is just added to increase clock cycle so that valid bit can be check again for obtaining correct data.
    ;(There is some minimum threshold time after which valid bit can be polled again)
	; In multi channel it is not required. Other instructions used there compensates the time )
	LDI     scratch1.b0,0
WB13?:

	AND     scratch.b0 ,R31.b3  ,valid_mask
    ;wait for valid bit
    QBNE    WB13?,  scratch.b0,valid_mask

CHx?:
    ;  Check the value of Mid bit of received oversampled data for the selected channel
    QBBS    BIT_LOGIC_1?,   R31,    bit_idx

    JMP     BIT_CAPTURED?
BIT_LOGIC_1?:
    ;If Mid bit of oversampled data has logic 1 then store as logic 1.
    SET     Ry0, Ry0.t7

BIT_CAPTURED?:
    ;Clear valid bit
    MOV     R31.b3, scratch.b0

RX_RECEIVE_DOWNSAMPLE_LOOP1?:
; keeping inverted data frame(which is correct info ,as we receive LSB first from encoder) on particular channel rx register
    ;Loading received frame to Respective Channel register.
    MOV     Rx0.b0,Ry0
CRC_CALC?:

CHECH_LAST_RX_FRAME?:
	QBEQ    CRC_2?, rx_frames,1

	LOOP    CRC_1?,8
	LSR     temp1,Ry0,7
	LSR     temp2,crc,7
	XOR     val,temp1,temp2
	LSL     crc,crc,1
	LSL     Ry0,Ry0,1
	OR      crc,crc,val

CRC_1?:
	JMP     CRC_4?

CRC_2?:
	LDI     temp1,0
	QBEQ    CRC_SUCCESS?,Ry0,crc
	JMP     CRC_3?

CRC_SUCCESS?:
	LDI     temp1,1

CRC_3?:

	SBCO    &temp1,dmem,crc_offset,4

CRC_4?:

	.endm


;*************************************************************************************************************************************
;   Macro: RECEIVE_FRAMES_S
;
;   		Receive data from encoder and store it in respective registers assigned to channels
;
;	Registers:
;						R11.b0- Intermediate frame register for CH0 or CH1 or CH2
;						R12- Intermediate channel register for CH0 or CH1 or CH2
;						R16-R18:Channel registers CH0 or CH1 or CH2
;
;
;
;   PseudoCode:
;      (start code)
;		1.Read start bit and simply clear the valid flag without storing it.
;       2.Read and store next 8 bits in Intermediate register
;		3.Read stop bit and simply clear the valid flag without storing it.
;		4.When Intermediate Register gets full ,unload its data to Assigned Registers for particular channel
;		5. Continue loop between 1-4 instruction no. till last frame receive
;		6. After last frame received in intermediate register, unload its data to Assigned Registers for particular channel
;		(end code)
;
;   Parameters:
;       df_size - Size of data frame (8 bits in tamagawa)
;		bytes_filled -	Current byte level received from encoder
;		rx_frames -		no. of rx frames left that are expected to from encoder for the current command id.
;		flag  -		used as a trigger to unload last frame intermediate register data to channel registers
;		scratch -	temporary variable
;		scratch1 -	temporary variable
;		valid_mask - Mask of channels which are enabled
;		bit_idx -	Mid bit of particular channel's oversample rx data that need to be captured for exact Rx data bit.
;
;**************************************************************************************************************************************

RECEIVE_FRAMES_S   .macro  df_size, bytes_filled ,rx_frames, flag ,scratch, scratch1,valid_mask,bit_idx,crc,dmem,crc_offset

;;Now data for first frame comes up FOR TAMAGAWA COMMAND

	LDI     R10.w0,10000
	LDI     R9.w1,0
START_BIT?:
	ADD     R9.w1,R9.w1,1
    ; Getting valid bit flag for enabled channel
	AND     scratch.b0 ,R31.b3  ,valid_mask

    QBLT	END_MACRO?, R9.w1, R10.w0
    ;  wait for valid bit
    QBNE    START_BIT?,  scratch.b0, valid_mask
    ;Set the valid bit for clearing flag so that next bit can come.
    ; This bit is start bit which is not stored.
	MOV     R31.b3, scratch.b0

;For receiving upcoming data frame
DATA_FRAME_S?:

	SINGLE_CH_RECEIVE_DATA_FRAME 	R11.b0 , R12 , df_size ,scratch,scratch1,valid_mask,bit_idx,R3.b1,R3.b2,R3.b3,R3.b0	,rx_frames,dmem,crc_offset		;Receive data frame(1 byte)
	;Increment the data bytes captured already by 1
    ADD     bytes_filled,bytes_filled,1

;For handling stop bit after data frame
STOP_BIT_HANDLE_S?:
    ;decrementing the expected rx frames by 1
    SUB     rx_frames ,rx_frames ,1

VALID_BIT_S?:
    ;Getting valid bit flag for enabled channel
    AND     scratch.b0 ,R31.b3  ,valid_mask
    ;  wait for valid
    QBNE    VALID_BIT_S?,  scratch.b0, valid_mask
    ;clear valid bit
    MOV     R31.b3, scratch.b0

			;As each channel using 3 registers for Rx data. 1 Register is used as intermediate register to load frames received.
			; If intermediate register gets full, then its data will be flushed to respective registers assigned for each channel.

    ;If Intermediate Reg full , Unload its data to First register of enabled channel as bytes_filled level crosses value 4
    QBEQ	LOAD_FIRST_REG_S?,bytes_filled ,4
    ;If Intermediate Reg full , Unload its data to Second register of enabled channel as bytes_filled level crosses value 8
    QBEQ	LOAD_SECOND_REG_S?,bytes_filled ,8
    ;If Intermediate Reg full , Unload its data to Third register of enabled channel as bytes_filled level crosses value 12
    QBEQ	LOAD_THIRD_REG_S?,bytes_filled ,12

CONTINUE_RX_LOAD_S?:
    ;If all frames over, last Unload of intermidiate register needs to be  done.
    QBEQ 	END_RX_S?,rx_frames ,0
    ;If frames are left to receive,Jump to receive next frame( Jump to Handle next Start bit)
    JMP     START_BIT?

LOAD_FIRST_REG_S?:
		;transferring 4bytes from intermediate to respective channel registers

    ;loading CH0, Reg1
    MOV     R16,R12
    ;clearing  intermediate register
    ZERO    &R12 ,4
    ;If flag is 0: Indication of last frame,Rx done.
    QBEQ	END_RX_4_S?, flag ,0
	JMP     CONTINUE_RX_LOAD_S?

LOAD_SECOND_REG_S?:

	;transferring 4bytes from intermediate to respective channel registers
    ;loading CH0, Reg2
    MOV     R17,R12
    ;clearing all intermediate registers
    ZERO    &R12 ,4
    ;If flag is 0: Indication of last frame,Rx done.
    QBEQ	END_RX_4_S?,flag ,0
	JMP     CONTINUE_RX_LOAD_S?

LOAD_THIRD_REG_S?:
	;transferring 4bytes from intermediate to respective channel registers
    ;loading CH0, Reg3
    MOV     R18,R12
    ;clearing all intermediate registers
    ZERO    &R12 ,4

    ;If flag is 0: Indication of last frame,Rx done.
    QBEQ	END_RX_4_S?, flag ,0
	JMP     CONTINUE_RX_LOAD_S?

END_RX_S?:
    ;As Rx ends here, last bytes present in intermediate register needs to be loaded into respective channel register
	LDI     flag,0
	QBGT	LOAD_FIRST_REG_S?,bytes_filled,4
	QBGT	LOAD_SECOND_REG_S?,bytes_filled,8
	QBGT	LOAD_THIRD_REG_S?,bytes_filled,12

END_RX_4_S?:
	OR      scratch1.b3,scratch1.b3,valid_mask
END_MACRO?:

	.endm

	.endif

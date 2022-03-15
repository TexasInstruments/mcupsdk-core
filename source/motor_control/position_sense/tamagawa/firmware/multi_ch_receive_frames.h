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
;*   File:     multi_ch_receive_frames.h											*
;*																					*
;*   Brief:    Macro for receiving data frames from Tamagawa encoder				*
;************************************************************************************

;************************************************************************************
;*   File:     multi_ch_rx.h														*
;*																					*
;*   Brief:    Macro for receiving one single data frame (which contains 8 bits) 	*
;*				from Tamagawa encoder												*
;************************************************************************************

;******************************************************************************************************************************************
;   Macro: MULTI_CH_RECEIVE_DATA_FRAME
;			Receive complete data frame in Intermediate Register of particular channel
;
;
;	Registers:
;						R11.b0,R11.b1,R11.b2 - Intermediate Frame register for CH0/CH1/CH2
;						R12,R13,R14-Intermediate channel register for CH0/CH1/CH2
;						R16-R18: Channel register for CH0
;						R19-R21: Channel register for CH1
;						R22-R24: Channel register for CH2
;
;   PseudoCode:
;      (start code)
;		1.Perform LSL by 8 on Intermediate channel register for making b0 level(R12.b0,) available for upcoming frame.
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
;		Ry0,Ry1,Ry2 -	Intermediate Frame register for CH0/CH1/CH2
;		Rx0,Rx1,Rx2 -	Intermediate channel register for CH0/CH1/CH2
;		cnt  -		size of data frame
;		scratch -	temporary variable
;		scratch1 -	temporary variable
;		valid_mask - Mask of channels which are enabled
;
;   Worst case peak cycle usage:   6070 (approx.)
;
;**************************************************************************************************************************************


	.if	!$isdefed("__multi_ch_receive_frames_h")
__multi_ch_receive_frames_h	.set	1

	.include "tamagawa_interface.h"

MULTI_CH_RECEIVE_DATA_FRAME    .macro  Ry0,Ry1,Ry2,Rx0,Rx1,Rx2, cnt ,scratch,scratch1,valid_mask,temp1,temp2,temp3,temp4,temp5,temp6,val1,val2,val3,crc1,crc2,crc3,rx_frames,dmem
	;To Shift 1 byte left in Intermediate registers for loading incoming rx frame of 1 byte at Rx0.b0,Rx1.b0,Rx2.b0 positions.
    ;Intermediate Register for channel 0
    LSL     Rx0 ,Rx0 ,8
    ;Intermediate Register for channel 1
    LSL     Rx1 ,Rx1 ,8
    ;Intermediate Register for channel 2
    LSL     Rx2 ,Rx2 ,8

    ;   Run loop for receive bits for each data frame
    LOOP    RX_RECEIVE_DOWNSAMPLE_LOOP1?,    cnt

	;Shift 1 position right for align the bit to LSB position (As we are receiving bit sequence in LSB bit first mode for each frame).
    LSR     Ry0, Ry0, 1
    LSR     Ry1, Ry1, 1
    LSR     Ry2, Ry2, 1


WB13?:
	AND scratch.b0 ,R31.b3  ,valid_mask
    ;  wait for valid
    QBNE    WB13?,  scratch.b0,valid_mask


CH0?:
    ;  CHANNEL 0:Check the Mid bit of received oversampled data
    QBBS    BIT_LOGIC_1_CH0?,   R31,    4
    JMP     CH1?
BIT_LOGIC_1_CH0?:
    ;If Mid bit of oversampled data has logic 1 then store as logic 1.
    SET     Ry0, Ry0.t7

CH1?:
    ;  CHANNEL 1:Check the Mid bit of received oversampled data
    QBBS    BIT_LOGIC_1_CH1?,   R31,    4+8
    JMP     CH2?
BIT_LOGIC_1_CH1?:
    ;If Mid bit of oversampled data has logic 1 then store as logic 1.
    SET     Ry1, Ry1.t7

CH2?:
    ;  CHANNEL 2:Check the Mid bit of received oversampled data
    QBBS    BIT_LOGIC_1_CH2?,   R31,    4+16
    JMP     BIT_CAPTURED?
BIT_LOGIC_1_CH2?:
    ;If Mid bit of oversampled data has logic 1 then store as logic 1.
    SET      Ry2, Ry2.t7

BIT_CAPTURED?:
    ;Clear valid bit
    MOV R31.b3, scratch.b0
RX_RECEIVE_DOWNSAMPLE_LOOP1?:
;Moving inverted data frame(which is correct info, as we receive LSB first from encoder) on particular intermediate channel rx register
    ;1 byte load to CH0 intermediate Reg
    MOV     Rx0.b0,Ry0
    ;1 byte load to CH1 intermediate Reg
    MOV     Rx1.b0,Ry1
    ;1 byte load to CH2 intermediate Reg
    MOV     Rx2.b0,Ry2

CRC_CALC?:

CHECH_LAST_RX_FRAME?:
	QBEQ CRC_2?, rx_frames,1

	LOOP     CRC_1?,8
	LSR     temp1,Ry0,7
	LSR     temp2,crc1,7
	XOR     val1,temp1,temp2
	LSL     crc1,crc1,1
	LSL     Ry0,Ry0,1
	OR      crc1,crc1,val1

	LSR     temp3,Ry1,7
	LSR     temp4,crc2,7
	XOR     val2,temp3,temp4
	LSL     crc2,crc2,1
	LSL     Ry1,Ry1,1
	OR      crc2,crc2,val2

	LSR     temp5,Ry2,7
	LSR     temp6,crc3,7
	XOR     val3,temp5,temp6
	LSL     crc3,crc3,1
	LSL     Ry2,Ry2,1
	OR      crc3,crc3,val3


CRC_1?:
	JMP     CRC_4?

CRC_2?:
	LDI     temp1,0
	LDI     temp2,0
	LDI     temp3,0

CRC_RES_1?:
	QBEQ    CRC_SUCCESS_1?,Ry0,crc1

CRC_RES_2?:
	QBEQ    CRC_SUCCESS_2?,Ry1,crc2

CRC_RES_3?:
	QBEQ    CRC_SUCCESS_3?,Ry2,crc3

	JMP     CRC_3?

CRC_SUCCESS_1?:
	LDI     temp1,1
	JMP     CRC_RES_2?
CRC_SUCCESS_2?:
	LDI     temp2,1
	JMP     CRC_RES_3?
CRC_SUCCESS_3?:
	LDI     temp3,1

CRC_3?:

	SBCO    &temp1,dmem,TAMAGAWA_CH0_CRC_OFFSET,4
	SBCO    &temp2,dmem,TAMAGAWA_CH1_CRC_OFFSET,4
	SBCO    &temp3,dmem,TAMAGAWA_CH2_CRC_OFFSET,4

CRC_4?:


	.endm


;*************************************************************************************************************************************
;   Macro: RECEIVE_FRAMES_M
;
;   		Receive data from encoder and store it in respective registers assigned to channels
;
;	Registers:
;						R11.b0,R11.b1,R11.b2 - Intermediate Frame register for CH0/CH1/CH2
;						R12,R13,R14- Intermediate channel registers for CH0/CH1/CH2
;						R16-R18: Channel register for CH0
;						R19-R21: Channel register for CH1
;						R22-R24: Channel register for CH2
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
;
;**************************************************************************************************************************************


RECEIVE_FRAMES_M   .macro  df_size, bytes_filled ,rx_frames, flag ,scratch, scratch1,valid_mask,dmem

;;Now data for first frame comes up FOR TAMAGAWA COMMAND
START_BIT?:
    ; Getting valid bit flag for enabled channel
    AND     scratch.b0 ,R31.b3  ,valid_mask
    ;  wait for valid
    QBNE    START_BIT?,  scratch.b0, valid_mask
    ;Set the valid bit for clearing flag so that next bit can come.
    ; This bit is start bit which is not stored.
	MOV     R31.b3, scratch.b0

DATA_FRAME_M?:			;For receiving upcoming data frame

	MULTI_CH_RECEIVE_DATA_FRAME 	R11.b0, R11.b1, R11.b2 , R12 ,R13 , R14, df_size ,scratch,scratch1,valid_mask,R3.b0,R3.b1,R3.b2,R3.b3,R5.b0,R5.b1,R5.b2,R5.b3,R6.b1,R6.b2,R6.b3,R7.b1,rx_frames,dmem		;Receive data frame(1 byte)
    ;Increment the data bytes captured already by 1
    ADD     bytes_filled,bytes_filled,1
;For handling stop bit after data frame
STOP_BIT_HANDLE_M?:
    ;decrementing the expected rx frames by 1
    SUB     rx_frames ,rx_frames ,1

VALID_BIT_M?:
    ;Getting valid bit flag for enabled channel
    AND     scratch.b0 ,R31.b3  ,valid_mask
    ;  wait for valid
    QBNE    VALID_BIT_M?,  scratch.b0, valid_mask
	MOV     R31.b3, scratch.b0	;clear valid bit

;As each channel using 3 registers for Rx data. 1 Register is used as intermediate register to load frames received.
; If intermediate register gets full, then its data will be flushed to respective registers assigned for each channel.
    ;If Intermediate Reg full , Unload its data to First register of enabled channel as bytes_filled level crosses value 4
    QBEQ	LOAD_FIRST_REG_M?,bytes_filled ,4
    ;If Intermediate Reg full , Unload its data to First register of enabled channel as bytes_filled level crosses value 8
    QBEQ	LOAD_SECOND_REG_M?,bytes_filled ,8
    ;If Intermediate Reg full , Unload its data to First register of enabled channel as bytes_filled level crosses value 12
    QBEQ	LOAD_THIRD_REG_M?,bytes_filled ,12

CONTINUE_RX_LOAD_M?:
    ;If all frames over, last Unload of intermidiate register needs to be  done.
    QBEQ 	END_RX_M?,rx_frames ,0
    ;If frames are left to receive,Jump to receive next frame( Jump to Handle next Start bit)
    JMP     START_BIT?

LOAD_FIRST_REG_M?:
	;transferring 4bytes from intermediate to respective channel registers
    ;loading CH0, Reg1
    MOV     R16,R12
    ;loading CH1, Reg1
    MOV     R19,R13
    ;loading CH2, Reg1
    MOV     R22,R14
    ;clearing all intermediate registers
    ZERO    &R12 ,12
	QBEQ	END_RX_4_M?, flag ,0
	JMP     CONTINUE_RX_LOAD_M?

LOAD_SECOND_REG_M?:
	;transferring 4bytes from intermediate to respective channel registers
    ;loading CH0,Reg2
    MOV     R17,R12
    ;loading CH1, Reg2
    MOV     R20,R13
    ;loading CH2, Reg2
    MOV     R23,R14
    ;clearing all intermediate registers
    ZERO    &R12 ,12
    ;If flag is 0: Indication of last frame,Rx done.
    QBEQ	END_RX_4_M?,flag ,0
	JMP     CONTINUE_RX_LOAD_M?

LOAD_THIRD_REG_M?:
	;transferring 4bytes from intermediate to respective channel registers
    ;loading CH0, Reg3
    MOV     R18,R12
    ;loading CH1, Reg3
    MOV     R21,R13
    ;loading CH2, Reg3
    MOV     R24,R14
    ;clearing all intermediate registers
    ZERO    &R12 ,12
	QBEQ	END_RX_4_M?, flag ,0
	JMP     CONTINUE_RX_LOAD_M?


END_RX_M?:
    ;As Rx ends here, last bytes present in intermediate register needs to be loaded into respective channel register
	LDI     flag,0
	QBGT	LOAD_FIRST_REG_M?,bytes_filled,4
	QBGT	LOAD_SECOND_REG_M?,bytes_filled,8
	QBGT	LOAD_THIRD_REG_M?,bytes_filled,12

END_RX_4_M?:
	.endm

	.endif

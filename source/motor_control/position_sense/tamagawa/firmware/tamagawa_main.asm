
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

CALL	.macro function
	JAL	R8.w0,	function
	.endm
RET	.macro
	JMP	R8.w0
	.endm

CALL2	.macro function
	JAL	R8.w2,	function
	.endm
RET2	.macro
	JMP	R8.w2
	.endm



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;/
; Assembler Directives Section
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;/
	.sect   ".text"
	.retain		; Required for building .out with assembly file
	.retainrefs ; Required for building .out with assembly file
	.global TAMAGAWA_INIT
    .global FN_SEND
    .global FN_SEND_WAIT_TILL_TX_CH_BUSY
    .global CALL
    .global CALL2

;     file:   tamagawa_main.asm
;
;     brief:  Tamagawa protocol Interface

;************************************* includes *************************************

	.include "tamagawa_icss_reg_defs.h"
	.include "tamagawa_interface.h"
	.include "..\..\..\..\pru_io\firmware\common\icss_regs.inc"
	.include "single_ch_receive_frames.h"
	.include "multi_ch_receive_frames.h"
    .include "firmware_version.h"
    .include "tamagawa_send.h"
    .include "eeprom_read.h"
    .include "eeprom_write.h"

	.asg	R28,	SCRATCH
	.asg	R29,	SCRATCH1
	.asg	R7.b0,	TAMAGAWA_ENABLE_CHx
	.asg	c25,	PRUx_DMEM

	.asg	R27.b0,	TX_DATA0
	.asg	R27.b1,	TX_DATA1
	.asg	R1.b1,	TX_FRAMES
	.asg	R6.b0,	RX_FRAMES


;**********************
;*        MAIN        *
;**********************

main:
    ;Required for the firmware_version.h file
    .word       ICSS_FIRMWARE_RELEASE_1,	ICSS_FIRMWARE_RELEASE_2
TAMAGAWA_INIT:

    ;If PRU0 is defined in symbols, it will select all PRU0 CFG registers.
    .if	$isdefed("PRU0")
    ;Data Memory address for PRU0 is loaded in PRUx_DMEM
    .asg	PRU0_DMEM,		PRUx_DMEM
	.asg    ICSS_CFG_PRU0_ED_CH0_CFG1, ICSS_CFG_PRUx_ED_CH0_CFG1
	.asg    ICSS_CFG_PRU0_ED_CH1_CFG1, ICSS_CFG_PRUx_ED_CH1_CFG1
	.asg    ICSS_CFG_PRU0_ED_CH2_CFG1, ICSS_CFG_PRUx_ED_CH2_CFG1
	.asg    ICSS_CFG_PRU0_ED_CH0_CFG0, ICSS_CFG_PRUx_ED_CH0_CFG0
	.asg    ICSS_CFG_PRU0_ED_CH1_CFG0, ICSS_CFG_PRUx_ED_CH1_CFG0
	.asg    ICSS_CFG_PRU0_ED_CH2_CFG0, ICSS_CFG_PRUx_ED_CH2_CFG0
	.asg    ICSS_CFG_PRU0_ED_TXCFG,    ICSS_CFG_PRUx_ED_TXCFG
	.asg    ICSS_CFG_PRU0_ED_RXCFG,    ICSS_CFG_PRUx_ED_RXCFG
	.endif

    ;If PRU1 is defined in symbols, it will select all PRU1 CFG registers.
    .if	$isdefed("PRU1")
    ;Data Memory address for PRU1 is loaded in PRUx_DMEM
    .asg	PRU1_DMEM,		PRUx_DMEM
	.asg    ICSS_CFG_PRU1_ED_CH0_CFG1, ICSS_CFG_PRUx_ED_CH0_CFG1
	.asg    ICSS_CFG_PRU1_ED_CH1_CFG1, ICSS_CFG_PRUx_ED_CH1_CFG1
	.asg    ICSS_CFG_PRU1_ED_CH2_CFG1, ICSS_CFG_PRUx_ED_CH2_CFG1
	.asg    ICSS_CFG_PRU1_ED_CH0_CFG0, ICSS_CFG_PRUx_ED_CH0_CFG0
	.asg    ICSS_CFG_PRU1_ED_CH1_CFG0, ICSS_CFG_PRUx_ED_CH1_CFG0
	.asg    ICSS_CFG_PRU1_ED_CH2_CFG0, ICSS_CFG_PRUx_ED_CH2_CFG0
	.asg    ICSS_CFG_PRU1_ED_TXCFG,    ICSS_CFG_PRUx_ED_TXCFG
	.asg    ICSS_CFG_PRU1_ED_RXCFG,    ICSS_CFG_PRUx_ED_RXCFG
	.endif

	; Initalize ENDAT mode
	; 	ICSS_CFG.GPCFG1[27:26] = 1
	LDI		R0.b0,	4
    ;It will initialize Endat Mode for PRU1
	SBCO	&R0.b0,	ICSS_CFG,	ICSS_CFG_GPCFG1+3,	1

	; Initialize PRUx_TAMAGAWA_CH0_CFG0/1 by clearing all channel CFG registers
	ZERO	&R0,	4

	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_CH0_CFG0
	SBCO	&R0,	ICSS_CFG,	SCRATCH1.w0,	4
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_CH1_CFG0
	SBCO	&R0,	ICSS_CFG,	SCRATCH1.w0,	4
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_CH2_CFG0
	SBCO	&R0,	ICSS_CFG,	SCRATCH1.w0,	4

	; clear all registers
	ZERO	&R0,	120

    ; (Channel Mask) Record channel enabled by host, save it after zeroing registers (done above)
	LBCO	&TAMAGAWA_ENABLE_CHx,	PRUx_DMEM,	TAMAGAWA_CHANNEL_CONFIG_OFFSET,	1

	.if	$defined("ENABLE_MULTI_CHANNEL")=0
        ;  if no channel selected,set channel mask default to ch0
        ;  if more than 1 channel selected,set channel mask default to ch0
        AND     TAMAGAWA_ENABLE_CHx,	TAMAGAWA_ENABLE_CHx, 0x7
        QBEQ    TAMAGAWA_DEFAULT_CH,	TAMAGAWA_ENABLE_CHx, 0x7
        QBEQ    TAMAGAWA_DEFAULT_CH,	TAMAGAWA_ENABLE_CHx, 0
        JMP     TAMAGAWA_SKIP_DEFAULT_CH
TAMAGAWA_DEFAULT_CH:
        LDI     TAMAGAWA_ENABLE_CHx, 0x1
TAMAGAWA_SKIP_DEFAULT_CH:
	.endif
 ;setting Rx and Tx clocks
TAMAGAWA_SET_CLOCK:
    ;Set the TX Div Factor value based on the value of RX_CLK
    ;div factor = 10 for RX_CLK: 8*2.5 MHz and div factor = 5 for RX_CLK: 8*5 MHz
	LBCO	&R0.w0,	PRUx_DMEM,	TAMAGAWA_RX_DIV_FACTOR_OFFSET,	2
    ;Set the RX Div Factor value based on the value of TX_CLK
    ;div factor = 80 for TX_CLK: 2.5 MHz and div factor = 40 for TX_CLK: 5 Mhz
	LBCO	&R0.w2,	PRUx_DMEM,	TAMAGAWA_TX_DIV_FACTOR_OFFSET ,	2
    ;8x oversample rate
	LBCO	&R1.b0,	PRUx_DMEM,	TAMAGAWA_OVERSAMPLE_RATE_OFFSET ,	1
	;Set clock to 2.5Mhz or 5MHz
    CALL	FN_SET_TX_CLK

TAMAGAWA_SKIP_INIT_SUCCESS:
	LDI     R0.b0,  1
    ;Initialization successful status update ends here
	SBCO	&R0.b0,	PRUx_DMEM,	TAMAGAWA_INTFC_CMD_STATUS_OFFSET,	1
	LBCO	&R0.b0,	PRUx_DMEM,	TAMAGAWA_OPMODE_CONFIG_OFFSET,	1
    ;If opmode=1, Host trigger is done
	;If opmode=0, Periodic trigger is done
	QBNE	HANDLE_HOST_TRIGGER_MODE,	R0.b0,		0

HANDLE_HOST_TRIGGER_MODE:
    ;If Host Trigger=1, request made by R5F to Firmware, now Firmware do processing and when done set trigger to 0 so that R5F application can act further.
    LBCO	&R0.b0,	PRUx_DMEM,	TAMAGAWA_INTFC_CMD_TRIGGER_OFFSET,	1
    ;wait till host trigger is set to 1.
	QBEQ            HANDLE_HOST_TRIGGER_MODE, R0.b0,	0
    ;load Tx data which will be loaded in Tx FIFO.
	LBCO	&TX_DATA0,	PRUx_DMEM,	TAMAGAWA_WORD_0_OFFSET,	1
    ;load Tx data which will be loaded in Tx FIFO.
	LBCO	&TX_DATA1,	PRUx_DMEM,	TAMAGAWA_WORD_0_OFFSET+1,	1
    ;load Tx frames expected
	LBCO	&TX_FRAMES , PRUx_DMEM,	TAMAGAWA_WORD_1_OFFSET,	1
    ; load Rx frames expected
	LBCO	&RX_FRAMES , PRUx_DMEM,	TAMAGAWA_WORD_1_OFFSET+1 ,	1
    ;Call SEND RECEIVE FUNCTION
	CALL	FN_SEND_RECEIVE_TAMAGAWA

TAMAGAWA_HOST_CMD_END:
	LDI		R3.w0,	0
    ;Clear Host Trigger
	SBCO	&R3.b0,	PRUx_DMEM,	TAMAGAWA_INTFC_CMD_TRIGGER_OFFSET,	1
    ;Handle next Position request by user.
    JMP		HANDLE_HOST_TRIGGER_MODE


;******************************************************************************************************************************************************
;	Function: FN_SEND_RECEIVE_TAMAGAWA
;
;	Brief:
;			1.Select Channels
;			2.Load Tx fifo for currently selected channel by using EEPROM_READ_MACRO, EEPROM_WRITE_MACRO and TAMAGAWA_SEND_MACRO for EEPROM Read, EEPROM Write and non-EEPROM commands respectively.
;			3.Perform step 2 till all channels which are enabled are selected and their Tx FIFO is loaded.
;			4.Call FN_SEND to transmit Fifo data in different modes based on the command.
;			5.Receive data. For multi-channel, RECEIVE_FRAMES_M macro is called. For single channel, RECEIVE_FRAMES_S macro is called.
;			6.Transfer Rx data stored in Channel Registers to Tamagawa Interface variable having offset- TAMAGAWA_CHx_POSITION_DATA_WORD0_OFFSET
;	Registers:
;           R26.b3: Holds the value of the command id
;			R30.w2: For Select Channel
;			R30.b0:	Load Tx fifo for selected channel
;
;	Parameters:
;			R11-R24: Registers used in macros for Rx data.
;			R2.b1 : Rx Dataframe size
;			R4.b1 : bytes_filled level
;			R4.b2 :	 flag(explained further)
;
; *********************************************************************************************************************************************************


FN_SEND_RECEIVE_TAMAGAWA:
;Select Channel (R30[17:16] tx_ch_sel) to push data into respective Tx Fifo.
;R30[17:16] tx_ch_sel=0 :Channel 0 selected
;					 =1: Channel 1 selected
;					 =2: Channel 2 selected
    ;If channel 0 is asked by user,continue further
    QBBC    TAMAGAWA_SKIP25_CH0, TAMAGAWA_ENABLE_CHx, 0
    ;Select Channel 0
    LDI		R30.w2,	TAMAGAWA_TX_CH0_SEL
TAMAGAWA_SKIP25_CH0:
    ;If channel 1 is asked by user,continue further
    QBBC    TAMAGAWA_SKIP25_CH1, TAMAGAWA_ENABLE_CHx, 1
    ;Select Channel 1
    LDI		R30.w2,	TAMAGAWA_TX_CH1_SEL
TAMAGAWA_SKIP25_CH1:
    ;If channel 2 is asked by user,continue further
    QBBC    TAMAGAWA_SKIP25_CH2, TAMAGAWA_ENABLE_CHx, 2
    ;Select Channel 2
    LDI		R30.w2,	TAMAGAWA_TX_CH2_SEL
TAMAGAWA_SKIP25_CH2:
    ;Clear the R10 register to store the channel number
    ZERO    &R10, 4
    ;Store the channel number from R30.b2 to R10.b0
    MOV     R10.b0, R30.b2

    ;Clear the R25 and R26 registers
    ZERO    &R25, 8
    ;Load the value of the type of command into R26.b3
    LBCO    &R26.b3, PRUx_DMEM, TAMAGAWA_EEPROM_CMD_OFFSET, 1
    ;If the command is EEPROM Read, jump to EEPROM_READ and continue forward
    QBEQ    EEPROM_READ, R26.b3, EEPROM_READ_CMD_ID
    ;If the command is EEPROM Write, jump to EEPROM_READ and continue forward
    QBEQ    EEPROM_WRITE, R26.b3, EEPROM_WRITE_CMD_ID

FN_SEND_START:
    TAMAGAWA_SEND_MACRO     TX_DATA0, TX_DATA1
    ;Jump to the end of section where we send Tx data
    JMP TX_END

EEPROM_READ:
    EEPROM_READ_MACRO   TAMAGAWA_ENABLE_CHx, PRUx_DMEM
    ;Jump to the end of section where we send Tx data
    JMP TX_END

EEPROM_WRITE:
    EEPROM_WRITE_MACRO   TAMAGAWA_ENABLE_CHx, PRUx_DMEM

TX_END:

    ;R11-R24 used for Rx
    ZERO    &R11, 60
	ZERO    &R3, 4
	ZERO    &R5, 4
	AND     R6,R6, 0xFF
	AND     R7,R7, 0xFF
    ;dataframe size(8 in tamagawa)
    LDI     R2.b1, 8
    ;bytes_filled (no. of frames or bytes received)
    LDI     R4.b1, 0
    ;flag for loading last register for respective channels(initially 1)
    LDI     R4.b2, 1

	.if	$defined("ENABLE_MULTI_CHANNEL")
MULTI_CHANNEL_RECEIVE:
    ;enable Rx for channel 0
    SET     R30,R30.t24
    ;enable Rx for channel 1
    SET     R30,R30.t25
    ;enable Rx for channel 2
    SET     R30,R30.t26

	RECEIVE_FRAMES_M R2.b1 ,R4.b1 , RX_FRAMES , R4.b2 , SCRATCH, SCRATCH1,TAMAGAWA_ENABLE_CHx,PRUx_DMEM
CH0_RX_TO_INTERFACE:
    ;If channel 0 enabled,then load Rx data to tamagawa interface
    QBBC    CH1_RX_TO_INTERFACE, TAMAGAWA_ENABLE_CHx,0
    ;storing Channel 0 Rx data to Tamagawa Interface created in R5F application
    SBCO	&R16,	PRUx_DMEM,	TAMAGAWA_CH0_POSITION_DATA_WORD0_OFFSET,	12
CH1_RX_TO_INTERFACE:
    ;If channel 0 enabled,then load Rx data to tamagawa interface
    QBBC    CH2_RX_TO_INTERFACE, TAMAGAWA_ENABLE_CHx,1
    ;storing Channel 1 Rx data to Tamagawa Interface created in R5F application
    SBCO	&R19,	PRUx_DMEM,	TAMAGAWA_CH1_POSITION_DATA_WORD0_OFFSET,	12
CH2_RX_TO_INTERFACE:
    ;If channel 0 enabled,then load Rx data to tamagawa interface
    QBBC    INTERFACE_LOADED, TAMAGAWA_ENABLE_CHx,2
    ;storing Channel 2 Rx data to Tamagawa Interface created in R5F application
    SBCO	&R22,	PRUx_DMEM,	TAMAGAWA_CH2_POSITION_DATA_WORD0_OFFSET,	12

INTERFACE_LOADED:

	.else
SINGLE_CHANNEL_RECEIVE:
	ZERO    &R3,4
CHECK_CH0:
    ;If channel 0 enabled, start Rx for channel 0
    QBBC    CHECK_CH1, TAMAGAWA_ENABLE_CHx,0
    ;enable Rx for channel 0
    SET     R30,R30.t24
    ;4 is mid bit no. for rx oversample data for channel 0
    RECEIVE_FRAMES_S    R2.b1,R4.b1,RX_FRAMES,R4.b2,SCRATCH, SCRATCH1,0x1,4,R3.b0,PRUx_DMEM,TAMAGAWA_CH0_CRC_OFFSET
    ;storing Channel 0 Rx data to Tamagawa Interface created in R5F application
    SBCO    &R16, PRUx_DMEM, TAMAGAWA_CH0_POSITION_DATA_WORD0_OFFSET, 12
CHECK_CH1:
    ;If channel 1 enabled, start Rx for channel 1
    QBBC    CHECK_CH2, TAMAGAWA_ENABLE_CHx,1
    ;enable Rx for channel 1
    SET     R30,R30.t25
    ; 4+8 is mid bit no. for rx oversample data for channel 1
    RECEIVE_FRAMES_S    R2.b1,R4.b1,RX_FRAMES,R4.b2,SCRATCH, SCRATCH1,0x2,4+8,R3.b0,PRUx_DMEM,TAMAGAWA_CH1_CRC_OFFSET
    ;storing Channel 1 Rx data to Tamagawa Interface created in R5F application
    SBCO	&R16,	PRUx_DMEM,	TAMAGAWA_CH1_POSITION_DATA_WORD0_OFFSET,	12
CHECK_CH2:
    ;If channel 0 enabled, start Rx for channel 2
    QBBC    ALL_CHANNEL_DONE, TAMAGAWA_ENABLE_CHx,2
    ;enable Rx for channel 2
    SET     R30,R30.t26
    ; 4+16 is mid bit no. for rx oversample data for channel 2
    RECEIVE_FRAMES_S    R2.b1,R4.b1,RX_FRAMES,R4.b2,SCRATCH, SCRATCH1,0x4,4+16,R3.b0,PRUx_DMEM,TAMAGAWA_CH2_CRC_OFFSET
    ;storing Channel 2 Rx data to Tamagawa Interface created in R5F application
    SBCO	&R16, PRUx_DMEM, TAMAGAWA_CH2_POSITION_DATA_WORD0_OFFSET, 12
ALL_CHANNEL_DONE:

	.endif

	RET



;****************************************************************************************************
;	Function: FN_SEND
;
;	Brief:
;			1.Select Rx,Tx frames size based on the command id
;			2.Wait till Tx is busy for particular channel
;			3.For triggering Tx to transmit FIFO data, CHANNEL_GO is set for single channel, GLOBAL_GO is set for multi channel
;	Registers:
;			ICSS_CFG_PRUx_ED_CHx_CFG0[15:11] - for selecting Tx frame size
;			ICSS_CFG_PRUx_ED_CHx_CFG0[27:16] - for selecting Rx frame size
;			ICSS_CFG_PRUx_ED_TXCFG - for asserting Tx Go to issue new Tx frame
;			R31 - For setting CHANNEL_GO and GLOBAL_GO
;	Parameters:
;			R2.w1 : Rx frame size
;
; ***************************************************************************************************

FN_SEND:
	; Program tx_frame_size ICSS_CFG_PRUx_ED_CH0_CFG0[15:11] to 10
	; Program rx_frame_size in ICSS_CFG_PRUx_ED_CH0_CFG0[27:16]	to 110
    ; loading rx frame size to maximum bits we can receive from tamagawa encoder for a particular command ID
    LDI     R2.w1 , 110
    ; if channel 0 is enabled, updation of frame sizes will be done.
    QBBC    TAMAGAWA_SKIP15_CH0, TAMAGAWA_ENABLE_CHx,	0
    ;loading PRUx_ED_CFG0 Register for updating Tx frame size
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_CH0_CFG0+1
	LBCO	&R2.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    ;In order to clear the previous value of Tx frame size
    AND     R2.b0 , R2.b0 , BIT_CLEAR_MASK
    ;In case of non-EEPROM commands, use single shot mode with Tx frame size = 10
    QBEQ    SINGLESHOT10_CH0, R26.b3, NON_EEPROM_CMD_ID
    ;In case of EEPROM Read command, use single shot mode with Tx frame size = 30
    QBEQ    SINGLESHOT30_CH0, R26.b3, EEPROM_READ_CMD_ID
    ;In case of EEPROM Write command, use the Tx Continuous FIFO loading
    QBEQ    CONTINUOUS_MODE0, R26.b3, EEPROM_WRITE_CMD_ID
SINGLESHOT10_CH0:
    ;10 is used for selecting Tx frame size for channel 0
    OR	    R2.b0 , R2.b0 , (NON_EEPROM_TX_FRAME_SIZE<<3)
    JMP     TXSIZE0_END

SINGLESHOT30_CH0:
    ;30 is used for selecting Tx frame size for channel 0
    OR	R2.b0 , R2.b0 , (EEPROM_READ_TX_FRAME_SIZE<<3)
    JMP TXSIZE0_END

CONTINUOUS_MODE0:
    ;In case of EEPROM Write Command, use Tx Continuous FIFO loading
    AND R2.b0 , R2.b0 , (EEPROM_WRITE_TX_FRAME_SIZE<<3)
TXSIZE0_END:

	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_CH0_CFG0+1
    ; store Tx frame size to ICSS_CFG_PRUx_ED_CH0 for channel 0
    SBCO	&R2,	ICSS_CFG,	SCRATCH1.w0,	1
    ;Loading Rx frame size
    MOV     SCRATCH.w0, R2.w1
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_CH0_CFG0+2
    ;store rx frame size for channel 0 in ICSS_CFG_PRUx_ED_CH0
    SBCO	&SCRATCH.w0,	ICSS_CFG,	SCRATCH1.w0,	2

TAMAGAWA_SKIP15_CH0:
    ; if channel 1 is enabled, updation of frame sizes will be done.
    QBBC            TAMAGAWA_SKIP15_CH1, TAMAGAWA_ENABLE_CHx,	1
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_CH1_CFG0+1
	LBCO	&R2.b0,	ICSS_CFG,	SCRATCH1.w0,	1

    ;In order to clear the previous value of Tx frame size
    AND     R2.b0 , R2.b0 , BIT_CLEAR_MASK
    ;In case of non-EEPROM commands, use single shot mode with Tx frame size = 10
    QBEQ    SINGLESHOT10_CH1, R26.b3, NON_EEPROM_CMD_ID
    ;In case of EEPROM Read command, use single shot mode with Tx frame size = 30
    QBEQ    SINGLESHOT30_CH1, R26.b3, EEPROM_READ_CMD_ID
    ;In case of EEPROM Write command, use the Tx Continuous FIFO loading
    QBEQ    CONTINUOUS_MODE1, R26.b3, EEPROM_WRITE_CMD_ID

SINGLESHOT10_CH1:
    ;10 is used for selecting Tx frame size for channel 1
    OR	    R2.b0 , R2.b0 , (NON_EEPROM_TX_FRAME_SIZE<<3)
    JMP     TXSIZE1_END

SINGLESHOT30_CH1:
    ;30 is used for selecting Tx frame size for channel 1
    OR	    R2.b0 , R2.b0 , (EEPROM_READ_TX_FRAME_SIZE<<3)
    JMP     TXSIZE1_END

CONTINUOUS_MODE1:
    ;In case of EEPROM Write Command, use Tx Continuous FIFO loading
    AND     R2.b0 , R2.b0 , (EEPROM_WRITE_TX_FRAME_SIZE<<3)
TXSIZE1_END:

	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_CH1_CFG0+1
    ; store Tx frame size to ICSS_CFG_PRUx_ED_CH1 for channel 1
    SBCO	&R2,	ICSS_CFG,	SCRATCH1.w0,	1

	MOV     SCRATCH.w0, R2.w1
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_CH1_CFG0+2
    ;store rx frame size for channel 1 in ICSS_CFG_PRUx_ED_CH1
    SBCO	&SCRATCH.w0,	ICSS_CFG,	SCRATCH1.w0,	2

TAMAGAWA_SKIP15_CH1:
    ; if channel 2 is enabled, updation of frame sizes will be done.
    QBBC    TAMAGAWA_SKIP15_CH2, TAMAGAWA_ENABLE_CHx,	2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_CH2_CFG0+1
	LBCO	&R2.b0,	ICSS_CFG,	SCRATCH1.w0,	1

    ;In order to clear the previous value of Tx frame size
    AND     R2.b0 , R2.b0 , BIT_CLEAR_MASK
    ;In case of non-EEPROM commands, use single shot mode with Tx frame size = 10
    QBEQ    SINGLESHOT10_CH2, R26.b3, NON_EEPROM_CMD_ID
    ;In case of EEPROM Read command, use single shot mode with Tx frame size = 30
    QBEQ    SINGLESHOT30_CH2, R26.b3, EEPROM_READ_CMD_ID
    ;In case of EEPROM Write command, use the Tx Continuous FIFO loading
    QBEQ    CONTINUOUS_MODE2, R26.b3, EEPROM_WRITE_CMD_ID

SINGLESHOT10_CH2:
    ;10 is used for selecting Tx frame size for channel 2
    OR	    R2.b0 , R2.b0 , (NON_EEPROM_TX_FRAME_SIZE<<3)
    JMP     TXSIZE2_END

SINGLESHOT30_CH2:
    ;30 is used for selecting Tx frame size for channel 2
    OR	    R2.b0 , R2.b0 , (EEPROM_READ_TX_FRAME_SIZE<<3)
    JMP     TXSIZE2_END

CONTINUOUS_MODE2:
    ;In case of EEPROM Write Command, use Tx Continuous FIFO loading
    AND     R2.b0 , R2.b0 , (EEPROM_WRITE_TX_FRAME_SIZE<<3)
TXSIZE2_END:

	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_CH2_CFG0+1
    ;store Tx frame size to ICSS_CFG_PRUx_ED_CH2 for channel 2
    SBCO	&R2,	ICSS_CFG,	SCRATCH1.w0,	1

	MOV     SCRATCH.w0, R2.w1
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_CH2_CFG0+2
    ;store rx frame size for channel 2 in ICSS_CFG_PRUx_ED_CH2
    SBCO	&SCRATCH.w0,	ICSS_CFG,	SCRATCH1.w0,	2

TAMAGAWA_SKIP15_CH2:



FN_SEND_WAIT_TILL_TX_CH_BUSY:
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_TXCFG
	LBCO	&R2.b3,	ICSS_CFG,	SCRATCH1.w0,	1
	; Determines when you can assert tx go to issue a new TX frame
	.if	$defined("ENABLE_MULTI_CHANNEL")

    ; TODO : Assumption is that all 3 channels are enabled
    AND		R2.b3,	R2.b3,	0xE0
	QBNE	FN_SEND_WAIT_TILL_TX_CH_BUSY,	R2.b3,	0
	.else
    QBBC    TAMAGAWA_SKIP17_CH0, TAMAGAWA_ENABLE_CHx,	0
    ; Wait till Ch0 is busy
    QBBS	FN_SEND_WAIT_TILL_TX_CH_BUSY,	R2.b3,	5
TAMAGAWA_SKIP17_CH0:
    QBBC    TAMAGAWA_SKIP17_CH1, TAMAGAWA_ENABLE_CHx,	1
    ; Wait till Ch1 is busy
    QBBS	FN_SEND_WAIT_TILL_TX_CH_BUSY,	R2.b3,	6
TAMAGAWA_SKIP17_CH1:
    QBBC            TAMAGAWA_SKIP17_CH2, TAMAGAWA_ENABLE_CHx,	2
    ; Wait till Ch2 is busy
    QBBS	FN_SEND_WAIT_TILL_TX_CH_BUSY,	R2.b3,	7
TAMAGAWA_SKIP17_CH2:
	.endif

	.if	$defined("ENABLE_MULTI_CHANNEL")
	SET		R31, TAMAGAWA_TX_GLOBAL_GO
	.else

	SET	    R31, TAMAGAWA_TX_CHANNEL_GO
	.endif

	RET2

;****************************************************************************************************
;	Function: FN_SET_TX_CLK
;
;	Brief:	Setting clock/baud rate for Tx and Rx  (currently set for 2.5MHZ/2.5Mbps)
;	Registers:
;			ICSS_CFG_PRUx_ED_RXCFG -for setting rx clock, selecting ICSSG clock(200Mhz)
;			ICSS_CFG_PRUx_ED_TXCFG - for setting tx clock
;	Parameters:
; 			R0.w0 - DIV for RX_CLK
; 			R0.w2 - DIV for TX_CLK
; 			R1.b0 - Oversample rate for RX
;
; ***************************************************************************************************

FN_SET_TX_CLK:
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_RXCFG+2
	;div factor for rx = 10 or 5 based on the baud rate (value written in register is 9 or 4 respectively)
    SBCO	&R0.w0,	ICSS_CFG, SCRATCH1.w0, 2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_TXCFG+2
	;div factor for tx =80 or 40 based on the baud rate(value written in register is 79 or 40 respectively)
    SBCO	&R0.w2,	ICSS_CFG, SCRATCH1.w0, 2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ED_RXCFG

	; For using ICSSG clock(200Mhz)
    SET		R1.t4

	; Update ICSSG clock and oversampling value
    SBCO	&R1.b0,	ICSS_CFG, SCRATCH1.w0, 1
	RET

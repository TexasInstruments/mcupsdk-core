
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

CALL	.macro function
	JAL	R29.w0,	function
	.endm
RET	.macro
	JMP	R29.w0
	.endm

CALL2	.macro function
	JAL	R29.w2,	function
	.endm
RET2	.macro
	JMP	R29.w2
	.endm


; macro to wait for all three chanels to reach at same point
; USE: R14.b0-> store syn_bit, R14.b2-> channel bit, R14.b1-> R14.b0&R14.b2
M_WAIT_FOR_ENABLED_CHANNELS .macro
LOOP_WAIT_FOR_ENABLED_CHANNELS?:
    LDI   R14.b1,0
    LBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
    LBCO  &R14.b2,    PRUx_DMEM,  ENDAT_CH0_CHANNEL_CONFIG_OFFSET, 1
    AND   R14.b0,R14.b0,R14.b2
    OR  R14.b1, R14.b1, R14.b0
    LBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
    LBCO  &R14.b2,    PRUx_DMEM,  ENDAT_CH1_CHANNEL_CONFIG_OFFSET, 1
    AND   R14.b0,R14.b0,R14.b2
    OR  R14.b1, R14.b1,R14.b0
    LBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
    LBCO  &R14.b2,    PRUx_DMEM,  ENDAT_CH2_CHANNEL_CONFIG_OFFSET, 1
    AND   R14.b0,R14.b0,R14.b2
    OR   R14.b1, R14.b1,R14.b0

    QBNE  LOOP_WAIT_FOR_ENABLED_CHANNELS?, R14.b1, 0
    .endm

; macro for enable PRU cycle counter
; USE: R14 register
M_ENABLE_PRU_CYCLE_COUNTER .macro
   .if $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LBCO	&R14, c28, 0, 4
	SET 	R14, R14, 3
	SBCO	&R14, c28, 0, 4
	.else
	LBCO	&R14, c11, 0, 4
	SET 	R14, R14, 3
	SBCO	&R14, c11, 0, 4
	.endif
  .endm
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;/
; Assembler Directives Section
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;/
	.sect   ".text"
	.retain		; Required for building .out with assembly file
	.retainrefs ; Required for building .out with assembly file
	.global ENDAT_INIT


	.include "endat_icss_reg_defs.h"
	.include "endat_params.h"
	.include "endat_interface.h"
	.include "firmware_version.h"
; ENABLE_MULTI_CHANNEL	.set	1; Enable this for EnDat send/receive functions to work across channels


	; register usage
	; R0, R1, R2 - cmd i/f (R0 also used as status & scratch in b/n where it is safe)
	; R3 - used in single channel, can be used in multi (also used in prop delay, free after it)
	; R4 - scratch
	; R5 - holds selected channel
	; R8, R9 - used by prop delay, but can used after it
	; R6, R7, R8, R9, R10, R11, R12, R15-R18, R19-R22, R23-R26 used for multi ch Rx
	; R13 - scratch
	; R14, R27, R28 still free in multi ch

	.asg	R4,		SCRATCH
	.asg	R13,	SCRATCH1
	.asg	R5.b0,		ENDAT_ENABLE_CHx
	.asg	R5.b1,		ENDAT_ENABLE_CHx_IN_USE
	.asg	R1.b2,	ENDAT_CMDTYP_NO_SUPPLEMENT_REG
	;.asg	c24,	PRUx_DMEM
	;.asg	c25,	PRU0_DMEM

ENABLE_PROPDELAY_MESUREMENT	.set	1

TRIGGER_ENDAT_COMPLETE_EVENT		.set	19
TRIGGER_ENDAT_COMPLETE_IRQ		.set	(0x20 | (TRIGGER_ENDAT_COMPLETE_EVENT - 16))


ENDAT_INIT:
        JMP             ENDAT_MAIN
        .word       ICSS_FIRMWARE_RELEASE_1,	ICSS_FIRMWARE_RELEASE_2
ENDAT_MAIN:
	.if	$isdefed("PRU0")
	.asg	PRU0_DMEM,		PRUx_DMEM
	.asg    ICSS_CFG_PRU0_ENDAT_CH0_CFG1, ICSS_CFG_PRUx_ENDAT_CH0_CFG1
	.asg    ICSS_CFG_PRU0_ENDAT_CH1_CFG1, ICSS_CFG_PRUx_ENDAT_CH1_CFG1
	.asg    ICSS_CFG_PRU0_ENDAT_CH2_CFG1, ICSS_CFG_PRUx_ENDAT_CH2_CFG1
	.asg    ICSS_CFG_PRU0_ENDAT_CH0_CFG0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0
	.asg    ICSS_CFG_PRU0_ENDAT_CH1_CFG0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0
	.asg    ICSS_CFG_PRU0_ENDAT_CH2_CFG0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0
	.asg    ICSS_CFG_PRU0_ENDAT_TXCFG,    ICSS_CFG_PRUx_ENDAT_TXCFG
	.asg    ICSS_CFG_PRU0_ENDAT_RXCFG,    ICSS_CFG_PRUx_ENDAT_RXCFG
	.endif

	.if	$isdefed("PRU1")
	.asg	PRU1_DMEM,		PRUx_DMEM
	.asg    ICSS_CFG_PRU1_ENDAT_CH0_CFG1, ICSS_CFG_PRUx_ENDAT_CH0_CFG1
	.asg    ICSS_CFG_PRU1_ENDAT_CH1_CFG1, ICSS_CFG_PRUx_ENDAT_CH1_CFG1
	.asg    ICSS_CFG_PRU1_ENDAT_CH2_CFG1, ICSS_CFG_PRUx_ENDAT_CH2_CFG1
	.asg    ICSS_CFG_PRU1_ENDAT_CH0_CFG0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0
	.asg    ICSS_CFG_PRU1_ENDAT_CH1_CFG0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0
	.asg    ICSS_CFG_PRU1_ENDAT_CH2_CFG0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0
	.asg    ICSS_CFG_PRU1_ENDAT_TXCFG,    ICSS_CFG_PRUx_ENDAT_TXCFG
	.asg    ICSS_CFG_PRU1_ENDAT_RXCFG,    ICSS_CFG_PRUx_ENDAT_RXCFG
	.endif

	; enable PRU cycle counter
    M_ENABLE_PRU_CYCLE_COUNTER


	.if	$isdefed("ENDAT_FW_HW_INIT")
	; Initalize ENDAT mode
	; 	ICSS_CFG.GPCFG0[27:26] = 1
	LDI		R0.b0,	4
	SBCO	&R0.b0,	ICSS_CFG,	ICSS_CFG_GPCFG1+3,	1

	LDI		R0.w0,	119 ;  RX_CLK: 200*8 KHz
	LDI		R0.w2,	959 ; TX_CLK: 200 KHz
	LDI		R1.w0,	10000 ; Enable receiver after 2 clocks => 2us
	; Initialize ENDAT clocks
	CALL	FN_SET_TX_CLK

	; Initialize PRU0_ENDAT_CH0_CFG0/1
	ZERO	&R0,	4

	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0
	SBCO	&R0,	ICSS_CFG,	SCRATCH1.w0,	4
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0
	SBCO	&R0,	ICSS_CFG,	SCRATCH1.w0,	4
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0
	SBCO	&R0,	ICSS_CFG,	SCRATCH1.w0,	4
	.endif

	; clear all registers
	ZERO	&R0,	120

    ;  record channel enabled by host, save it after zeroing registers (done above)
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	LDI	ENDAT_ENABLE_CHx,	0x7
    LBCO    &ENDAT_ENABLE_CHx_IN_USE,   PRUx_DMEM,  ENDAT_CH0_CHANNEL_CONFIG_OFFSET,  1
	.else
	LBCO	&ENDAT_ENABLE_CHx,	PRUx_DMEM,	ENDAT_CH0_CHANNEL_CONFIG_OFFSET,	1
        ;  if no channel selected, default to ch0
        ;  TODO: till multichannel is supported, if more than 1 channel selected, default to ch0
        AND     ENDAT_ENABLE_CHx,	ENDAT_ENABLE_CHx, 0x7
        QBEQ    ENDAT_DEFAULT_CH,	ENDAT_ENABLE_CHx, 0x7
        QBEQ    ENDAT_DEFAULT_CH,	ENDAT_ENABLE_CHx, 0
        JMP     ENDAT_SKIP_DEFAULT_CH
ENDAT_DEFAULT_CH:
        LDI     ENDAT_ENABLE_CHx, 0x1
ENDAT_SKIP_DEFAULT_CH:
	.endif



	CALL	FN_ENDAT_POWER_ON_INIT


	; Configure the TX clock mode for channel 0 to FREERUN mode (already done - confirm)
	; Program command to TX_DATA for channel 0 (R30[7:0]) and Set go bit
	; Will be send MSB first - X M2 M1 M0 (M2) (M1) (M0) x

	; NOTE: Use tx_frame_size for if there is a need to transmit N-bits towars the end
	; NOTE: rx_en_counter/manual rx_en is required after transmit to enable receiver
	; NOTE: At Low clock encoder can be parked upto 50ms (LC exception: 30us)
	; NOTE: At High clock encoder will reset it self after 10us (elapse of recovery time)

	LDI		R0.w0,	ENDAT_CMD_RECEIVE_RESET
	LDI		R0.w2,	0
	LDI		R1.b0,	ENDAT_RX_29BITS ; 1 (sb) + 29 (8+16+5) bits before setting clock to high
	LDI		R1.b1,	ENDAT_TX_30BITS  ; 6(Mode bits)+16(dummy)+8(dummy)
	LDI		R1.b2,	ENDAT_CMDTYP_2_1
	CALL	FN_SEND_RECEIVE_ENDAT


	LBCO	&R0,	PRUx_DMEM,	ENDAT_CONFIG_DELAY_50MS_OFFSET, 4
	CALL2 	FN_DELAY_CYCLES ; Wait 50ms

	; Read out and buffer alarms and warnings
	; Clear the alarms
	LDI		R0.w0,	(ENDAT_CMD_SEL_MEM_AREA | ((MRS_CODE_OPERATING_STATUS&0x80) >> 7) | (((MRS_CODE_OPERATING_STATUS << 1)&0xFF)<<8))
	LDI		R0.w2,	0
	LDI		R1.b0,	ENDAT_RX_29BITS ; 1 (sb) + 29 (8+16+5) bits before setting clock to high
	LDI		R1.b1,	ENDAT_TX_30BITS  ;  6(Mode bits)+16(dummy)+8(dummy)
	LDI		R1.b2,	ENDAT_CMDTYP_2_1
	CALL	FN_SEND_RECEIVE_ENDAT

	LBCO	&R0,	PRUx_DMEM,	ENDAT_CONFIG_DELAY_12MS_OFFSET, 4
	CALL2 	FN_DELAY_CYCLES ; Wait 12ms

	; Clear error messages
	; Select MRS:10111001(C7..C0) and read Word0	(address:0x00)
	LDI		R0.w0,	(ENDAT_CMD_RECEIVE_PARAMETERS | ((WORD_0&0x80) >> 7) | (((WORD_0 << 1)&0xFF)<<8))
	LDI		R0.w2,	0
	LDI		R1.b0,	ENDAT_RX_29BITS ; 1 (sb) + 29 (8+16+5) bits before setting clock to high
	LDI		R1.b1,	ENDAT_TX_30BITS  ;  6(Mode bits)+16(dummy)+8(dummy)
	LDI		R1.b2,	ENDAT_CMDTYP_2_1
	CALL	FN_SEND_RECEIVE_ENDAT

	LBCO	&R0,	PRUx_DMEM,	ENDAT_CONFIG_DELAY_12MS_OFFSET, 4
	CALL2 	FN_DELAY_CYCLES ; Wait 12ms

	; Clear the warnings
	; Select MRS:10111001(C7..C0) and read Word1	(address:0x01)
	LDI		R0.w0,	(ENDAT_CMD_RECEIVE_PARAMETERS | ((WORD_1&0x80) >> 7) | (((WORD_1 << 1)&0xFF)<<8))
	LDI		R0.w2,	0
	LDI		R1.b0,	ENDAT_RX_29BITS ; 1 (sb) + 29 (8+16+5) bits before setting clock to high
	LDI		R1.b1,	ENDAT_TX_30BITS  ;  6(Mode bits)+16(dummy)+8(dummy)
	LDI		R1.b2,	ENDAT_CMDTYP_2_1
	CALL	FN_SEND_RECEIVE_ENDAT

	LBCO	&R0,	PRUx_DMEM,	ENDAT_CONFIG_DELAY_12MS_OFFSET, 4
	CALL2 	FN_DELAY_CYCLES ; Wait 12ms

	; Read out number of clock pulses for transfer of the position value
	; Parameters of encoder manufacturer Word 13
	; Select MRS:10100001(C7..C0) and read Word13	(address:0x0D)
	LDI		R0.w0,	(ENDAT_CMD_SEL_MEM_AREA | ((MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0&0x80) >> 7) | (((MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0 << 1)&0xFF)<<8))
	LDI		R0.w2,	0
	LDI		R1.b0,	ENDAT_RX_29BITS ; 1 (sb) + 29 (8+16+5) bits before setting clock to high
	LDI		R1.b1,	ENDAT_TX_30BITS  ;  6(Mode bits)+16(dummy)+8(dummy)
	LDI		R1.b2,	ENDAT_CMDTYP_2_1
	CALL	FN_SEND_RECEIVE_ENDAT

	LBCO	&R0,	PRUx_DMEM,	ENDAT_CONFIG_DELAY_12MS_OFFSET, 4
	CALL2 	FN_DELAY_CYCLES ; Wait 12ms

	; Read out number of clock pulses for transfer of the position value
	; Parameters of encoder manufacturer Word 13
	; Select MRS:10100001(C7..C0) to get Word13	(address:0x0D)
	LDI		R0.w0,	(ENDAT_CMD_SEND_PARAMETERS | ((WORD_13&0x80) >> 7) | (((WORD_13 << 1)&0xFF)<<8))
	LDI		R0.w2,	0
	LDI		R1.b0,	ENDAT_RX_29BITS ; 1 (sb) + 29 (8+16+5) bits before setting clock to high
	LDI		R1.b1,	ENDAT_TX_30BITS  ;  6(Mode bits)+16(dummy)+8(dummy)
	LDI		R1.b2,	ENDAT_CMDTYP_2_1
	CALL	FN_SEND_RECEIVE_ENDAT
; Need to save the response and extract the clock pulses info

	.if	$isdefed("ENABLE_MULTI_CHANNEL")
        LSL		R15, R15, 3
        LSL		R19, R19, 3
        LSL		R23, R23, 3

        QBBC            ENDAT_SKIP2_CH0, ENDAT_ENABLE_CHx,	0
	    SBCO	&R15.b1,	PRUx_DMEM,	ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP2_CH0:
        QBBC            ENDAT_SKIP2_CH1, ENDAT_ENABLE_CHx,	1
	    SBCO	&R19.b1,	PRUx_DMEM,	ENDAT_CH1_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP2_CH1:
        QBBC            ENDAT_SKIP2_CH2, ENDAT_ENABLE_CHx,	2
	    SBCO	&R23.b1,	PRUx_DMEM,	ENDAT_CH2_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP2_CH2:

	.else	; ENABLE_MULTI_CHANNEL

      LSL             R15, R15, 3
    .if $isdefed("ENABLE_MULTI_MAKE_RTU") ;save response for ch0 in PRU
	   SBCO	 &R15.b1,	PRUx_DMEM,	ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET,	1
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;save response for  ch1 in RTU
	   SBCO	 &R15.b1,	PRUx_DMEM,	ENDAT_CH1_NUM_CLOCK_PULSES_OFFSET,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;save response for  CH2 in TXPRU
	   SBCO	 &R15.b1,	PRUx_DMEM,	ENDAT_CH2_NUM_CLOCK_PULSES_OFFSET,	1
	.else
       QBBC            ENDAT_SKIP2_CH0, ENDAT_ENABLE_CHx,	0
	   SBCO	 &R15.b1,	PRUx_DMEM,	ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP2_CH0:
        QBBC            ENDAT_SKIP2_CH1, ENDAT_ENABLE_CHx,	1
	SBCO	&R15.b1,	PRUx_DMEM,	ENDAT_CH1_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP2_CH1:
        QBBC            ENDAT_SKIP2_CH2, ENDAT_ENABLE_CHx,	2
	SBCO	&R15.b1,	PRUx_DMEM,	ENDAT_CH2_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP2_CH2:

	.endif
    .endif ; ENABLE_MULTI_CHANNEL

	LBCO	&R0,	PRUx_DMEM,	ENDAT_CONFIG_DELAY_2MS_OFFSET, 4
	CALL2 	FN_DELAY_CYCLES ; Wait 2ms

	; Check whether encoder supports EnDat2.2
	; Parameters of encoder manufacturer Word 37
	; Select MRS:10100101(C7..C0) and read Word37 (address:0x05)

	LDI		R0.w0,	(ENDAT_CMD_SEL_MEM_AREA | ((MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE2&0x80) >> 7) | (((MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE2 << 1)&0xFF)<<8))
	LDI		R0.w2,	0
	LDI		R1.b0,	ENDAT_RX_29BITS ; 1 (sb) + 29 (8+16+5) bits before setting clock to high
	LDI		R1.b1,	ENDAT_TX_30BITS  ;  6(Mode bits)+16(dummy)+8(dummy)
	LDI		R1.b2,	ENDAT_CMDTYP_2_1
	CALL	FN_SEND_RECEIVE_ENDAT



	LBCO	&R0,	PRUx_DMEM,	ENDAT_CONFIG_DELAY_12MS_OFFSET, 4
	CALL2 	FN_DELAY_CYCLES ; Wait 12ms


	; Read out EnDat command set
	; Parameters of encoder manufacturer Word 37
	; Select MRS:10100101(C7..C0) to get Word 37	(address:0x05)
	LDI		R0.w0,	(ENDAT_CMD_SEND_PARAMETERS | ((WORD_5&0x80) >> 7) | (((WORD_5 << 1)&0xFF)<<8))
	LDI		R0.w2,	0
	LDI		R1.b0,	ENDAT_RX_29BITS ; 1 (sb) + 29 (8+16+5) bits before setting clock to high
	LDI		R1.b1,	ENDAT_TX_30BITS  ;  6(Mode bits)+16(dummy)+8(dummy)
	LDI		R1.b2,	ENDAT_CMDTYP_2_1
	CALL	FN_SEND_RECEIVE_ENDAT
; Need to save the response and extract the clock pulses info

	.if	$isdefed("ENABLE_MULTI_CHANNEL")
        LSL		R15, R15, 3
        LSL		R19, R19, 3
        LSL		R23, R23, 3


        QBBC            ENDAT_SKIP4_CH0, ENDAT_ENABLE_CHx,	0
	SBCO	&R15.b1,    PRUx_DMEM,	ENDAT_CH0_ENDAT22_STAT_OFFSET,	1
ENDAT_SKIP4_CH0:
        QBBC            ENDAT_SKIP4_CH1, ENDAT_ENABLE_CHx,	1
	SBCO	&R19.b1,	PRUx_DMEM,	ENDAT_CH1_ENDAT22_STAT_OFFSET,	1
ENDAT_SKIP4_CH1:
        QBBC            ENDAT_SKIP4_CH2, ENDAT_ENABLE_CHx,	2
	SBCO	&R23.b1,	PRUx_DMEM,	ENDAT_CH2_ENDAT22_STAT_OFFSET,	1
ENDAT_SKIP4_CH2:

	.else	; ENABLE_MULTI_CHANNEL
	  LSL             R15, R15, 3
    .if $isdefed("ENABLE_MULTI_MAKE_RTU") ;save response for ch0 in PRU
	    SBCO	 &R15.b1,	PRUx_DMEM,	ENDAT_CH0_ENDAT22_STAT_OFFSET,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;save response for  ch1 in RTU
	    SBCO	 &R15.b1,	PRUx_DMEM,	ENDAT_CH1_ENDAT22_STAT_OFFSET,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;save response for  CH2 in TXPRU
	    SBCO	 &R15.b1,	PRUx_DMEM,	ENDAT_CH2_ENDAT22_STAT_OFFSET,	1
	.else
        QBBC            ENDAT_SKIP4_CH0, ENDAT_ENABLE_CHx,	0
	SBCO	&R15.b1,	PRUx_DMEM,	ENDAT_CH0_ENDAT22_STAT_OFFSET,	1
ENDAT_SKIP4_CH0:
        QBBC            ENDAT_SKIP4_CH1, ENDAT_ENABLE_CHx,	1
	SBCO	&R15.b1,	PRUx_DMEM,	ENDAT_CH1_ENDAT22_STAT_OFFSET,	1
ENDAT_SKIP4_CH1:
        QBBC            ENDAT_SKIP4_CH2, ENDAT_ENABLE_CHx,	2
	SBCO	&R15.b1,	PRUx_DMEM,	ENDAT_CH2_ENDAT22_STAT_OFFSET,	1
ENDAT_SKIP4_CH2:
    .endif
	.endif	; ENABLE_MULTI_CHANNEL

       ;set syn_bits of all channels for clock configuration
     .if $isdefed("ENABLE_MULTI_MAKE_RTU")
	    LDI    R14.b0, 0x1 ; set syn_bit bit for ch0
	    SBCO	  &R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	    LDI   R14.b0, 0x2 ;set syn_bit bit for ch1
	    SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	    LDI R14.b0, 0x4 ; set syn_bit bit for ch2
	    SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
    .endif

	LBCO	&R0,	PRUx_DMEM,	ENDAT_CONFIG_DELAY_12MS_OFFSET, 4
	CALL2 	FN_DELAY_CYCLES ; Wait 12ms

	; Perform propagation delay compensation
	; Create a function to support per channel delay computation
	.if	$isdefed("ENABLE_PROPDELAY_MESUREMENT")

     ; tx 200KHz, rx 8*12MHz
	LDI		R0.w0,	1 ;  RX_CLK: 8*12MHz
	LDI		R0.w2,	959 ; TX_CLK: 200 KHz
	LDI		R1.w0,	10000 ; Enable receiver after 2 clocks


    ;wait to reach at same point for all cores before clk configuration
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
	    LDI R14.b0, 0 ;clear  syn_bit bit for ch0
	    SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
        M_WAIT_FOR_ENABLED_CHANNELS
        LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
        QBBC  SKIP_CLOCK_CONFIG1, R14.b2, 0 ; check if RTU is  primary core
        CALL	FN_SET_TX_CLK
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	    LDI R14.b0, 0 ; clear syn_bit bit for ch1
	    SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
        M_WAIT_FOR_ENABLED_CHANNELS
        LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
        QBBC  SKIP_CLOCK_CONFIG1, R14.b2, 1 ; check if PRU is  primary core
        CALL	FN_SET_TX_CLK
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	    LDI R14.b0, 0 ;clear syn_bit bit for ch2
	    SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
        M_WAIT_FOR_ENABLED_CHANNELS
        LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
        QBBC  SKIP_CLOCK_CONFIG1, R14.b2, 2; check if TXPRU is  primary core
        CALL	FN_SET_TX_CLK
    .else
        CALL	FN_SET_TX_CLK ;When load share mode is not used, that is single channel or multi channel using single PRU
    .endif

SKIP_CLOCK_CONFIG1:

;set syn_bits of all channels for delay calculation
     .if $isdefed("ENABLE_MULTI_MAKE_RTU")
	    LDI R14.b0, 0x1 ;set  syn_bit  bit for ch0
	    SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	    LDI R14.b0, 0x2 ; set syn_bit bit for ch1
	    SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	    LDI R14.b0, 0x4 ; set syn_bit bit for ch2
	    SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
    .endif

	.if	$isdefed("ENABLE_MULTI_CHANNEL")

	; start loop with ch0 per channel variables
	LDI		R3.w0,	0 | 8 << 8
	LDI		R3.w2,	24 | 5 << 8
	LDI		R8.b0,	ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET

	LOOP		PROP_DELAY_MULTI_END,	3

	; # read position bits (offset ch[0,1,2]: [0x44, 0x64, 0x84])
	LBCO		&R2.b1,	PRUx_DMEM,	R8.b0,	1

	; bypass prop delay estimation if encoder not detected for the ch, else would wait indefinitely in prop delay fn
	QBEQ		ENDAT_SKIP_PROP_DELAY_CALC,	R2.b1,	0
	QBEQ		ENDAT_SKIP_PROP_DELAY_CALC,	R2.b1,	0xFF

	CALL		FN_PROP_DELAY_CALC

ENDAT_SKIP_PROP_DELAY_CALC:
	; write prop delay (offset ch[0,1,2]: [0x28, 0x48, 0x68])
	ADD		R8.b0,	R8.b0,	0x4
	SBCO		&R9,	PRUx_DMEM,	R8.b0,	4

	; update per ch variables for next channel
	; 0x4 + 0x1C = 0x20, corresponds to i/f buffer of one channel
	ADD		R8.b0,	R8.b0,	0x1C
	ADD		R3.b0,	R3.b0,	1 ; ENDAT_TX_CHx_SEL
	ADD		R3.b1,	R3.b1,	1 ; ENDAT_CHx_CLK
	ADD		R3.b2,	R3.b2,	1 ; ENDAT_CHx_SB
	ADD		R3.b3,	R3.b3,	8 ; ENDAT_CHx_TX_REINIT

PROP_DELAY_MULTI_END:

	.else
	 .if $isdefed("ENABLE_MULTI_MAKE_RTU") ;ch0: R3.b0 - ENDAT_TX_CHx_SEL, R2.b1 - clock plus for position, R3.b2 - ENDAT_CHx_SB, R3.b3 - ENDAT_CHx_TX_REINIT
	    LDI		R3.w0,	0 | 8 << 8
	    LDI		R3.w2,	24 | 5 << 8
	    LBCO		&R2.b1,	PRUx_DMEM,	ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET,	1
        CALL		FN_PROP_DELAY_CALC
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;ch1 : R3.b0 - ENDAT_TX_CHx_SEL, R2.b1 - clock plus for position, R3.b2 - ENDAT_CHx_SB, R3.b3 - ENDAT_CHx_TX_REINIT
	    LDI		R3.w0,	1 | 9 << 8
	    LDI		R3.w2,	25 | 13 << 8
	    LBCO		&R2.b1,	PRUx_DMEM,	ENDAT_CH1_NUM_CLOCK_PULSES_OFFSET,	1
        CALL		FN_PROP_DELAY_CALC
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;ch2: R3.b0 - ENDAT_TX_CHx_SEL, R2.b1 - clock plus for position, R3.b2 - ENDAT_CHx_SB, R3.b3 - ENDAT_CHx_TX_REINIT
	    LDI		R3.w0,	2 | 10 << 8
	    LDI		R3.w2,	26 | 21 << 8
	    LBCO		&R2.b1,	PRUx_DMEM,	ENDAT_CH2_NUM_CLOCK_PULSES_OFFSET,	1
        CALL		FN_PROP_DELAY_CALC
	.else
    QBBC            ENDAT_SKIP5A_CH0, ENDAT_ENABLE_CHx,	0
	LDI		R3.w0,	0 | 8 << 8
	LDI		R3.w2,	24 | 5 << 8
	LBCO		&R2.b1,	PRUx_DMEM,	ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP5A_CH0:
        QBBC            ENDAT_SKIP5A_CH1, ENDAT_ENABLE_CHx,	1
	LDI		R3.w0,	1 | 9 << 8
	LDI		R3.w2,	25 | 13 << 8
	LBCO		&R2.b1,	PRUx_DMEM,	ENDAT_CH1_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP5A_CH1:
        QBBC            ENDAT_SKIP5A_CH2, ENDAT_ENABLE_CHx,	2
	LDI		R3.w0,	2 | 10 << 8
	LDI		R3.w2,	26 | 21 << 8
	LBCO		&R2.b1,	PRUx_DMEM,	ENDAT_CH2_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP5A_CH2:
     CALL		FN_PROP_DELAY_CALC
   .endif



    .if $isdefed("ENABLE_MULTI_MAKE_RTU") ;store prop_delay value in DMEM for ch0
		SBCO	&R9,	PRUx_DMEM,	ENDAT_CH0_MEAS_PROPDELAY_OFFSET,	4
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;store prop_delay value in  DMEM for ch1
	    SBCO	&R9,	PRUx_DMEM,	ENDAT_CH1_MEAS_PROPDELAY_OFFSET,	4
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;store prop_delay value in DMEM for ch2
	    SBCO	&R9,	PRUx_DMEM,	ENDAT_CH2_MEAS_PROPDELAY_OFFSET,	4
	.else
        QBBC            ENDAT_SKIP7_CH0, ENDAT_ENABLE_CHx,	0
	SBCO	&R9,	PRUx_DMEM,	ENDAT_CH0_MEAS_PROPDELAY_OFFSET,	4
ENDAT_SKIP7_CH0:
        QBBC            ENDAT_SKIP7_CH1, ENDAT_ENABLE_CHx,	1
	SBCO	&R9,	PRUx_DMEM,	ENDAT_CH1_MEAS_PROPDELAY_OFFSET,	4
ENDAT_SKIP7_CH1:
        QBBC            ENDAT_SKIP7_CH2, ENDAT_ENABLE_CHx,	2
	SBCO	&R9,	PRUx_DMEM,	ENDAT_CH2_MEAS_PROPDELAY_OFFSET,	4
ENDAT_SKIP7_CH2:
    .endif
	.endif	; ENABLE_MULTI_CHANNEL
	.endif	; ENABLE_PROPDELAY_MESUREMENT

	LBCO	&R0,	PRUx_DMEM,	ENDAT_CONFIG_CLOCK_RX_OFFSET,           6
        QBNE            ENDAT_SKIP_DEFAULT_CLOCK,       R0,                     0
	LDI		R0.w0,	2 ;  RX_CLK: 8*8MHz
	LDI		R0.w2,	23 ;  TX_CLK: 8 MHz
	LDI		R1.w0,	250 ; Enable receiver after 2 clocks => 2us
ENDAT_SKIP_DEFAULT_CLOCK:

;wait to reach at same point for all cores before clk configuration
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
	    LDI R14.b0, 0 ;clear  syn_bit bit for ch0
	    SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
        M_WAIT_FOR_ENABLED_CHANNELS
        LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
        QBBC  SKIP_CLOCK_CONFIG2, R14.b2, 0; check RTU is  primary core
        CALL	FN_SET_TX_CLK
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	    LDI R14.b0, 0 ;clear syn_bit bit for ch1
	    SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
        M_WAIT_FOR_ENABLED_CHANNELS
        LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
        QBBC  SKIP_CLOCK_CONFIG2, R14.b2, 1; check PRU is  primary core
        CALL	FN_SET_TX_CLK
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	    LDI R14.b0, 0 ;clear syn_bit bit for ch2
	    SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
        M_WAIT_FOR_ENABLED_CHANNELS
        LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
        QBBC  SKIP_CLOCK_CONFIG2, R14.b2, 2; check TXPRU is  primary core
        CALL	FN_SET_TX_CLK
    .else
        CALL	FN_SET_TX_CLK ;When load share mode is not used, that is single channel or multi channel using single PRU
    .endif

SKIP_CLOCK_CONFIG2:

        ;  status indication that host is ready (bit0 in ) - currently based on non-zero clock pulse value
        ;  TODO: The way status is indicated to host should be made a robust rather than the one used here
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	LDI	R0.b0,	0
	LBCO	&R0.b1,	PRUx_DMEM,	ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET,	1
        QBEQ	ENDAT_SKIP8A_CH0, R0.b1,	0
        QBEQ	ENDAT_SKIP8A_CH0, R0.b1,	0xFF
	OR	R0.b0,	R0.b0,	0x1
ENDAT_SKIP8A_CH0:

	LBCO	&R0.b1,	PRUx_DMEM,	ENDAT_CH1_NUM_CLOCK_PULSES_OFFSET,	1
        QBEQ	ENDAT_SKIP8A_CH1, R0.b1,	0
        QBEQ	ENDAT_SKIP8A_CH1, R0.b1,	0xFF
	OR	R0.b0,	R0.b0,	0x2
ENDAT_SKIP8A_CH1:

	LBCO	&R0.b1,	PRUx_DMEM,	ENDAT_CH2_NUM_CLOCK_PULSES_OFFSET,	1
        QBEQ	ENDAT_SKIP8A_CH2, R0.b1,	0
        QBEQ	ENDAT_SKIP8A_CH2, R0.b1,	0xFF
	OR	R0.b0,	R0.b0,	0x4
ENDAT_SKIP8A_CH2:

	; check whether user specified channels has been detected
	LBCO	&R0.b2,	PRUx_DMEM,	ENDAT_CH0_CHANNEL_CONFIG_OFFSET,	1
	; store detected channels, this has to be done after reading user specified as done above
	SBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH0_CHANNEL_CONFIG_OFFSET,	1
	AND	R0.b0,	R0.b0,	R0.b2
	QBEQ	ENDAT_SKIP8A_END, R0.b2,	R0.b0
	LDI	R0.b0,	0
ENDAT_SKIP8A_END:
	MOV	ENDAT_ENABLE_CHx_IN_USE,	R0.b0

	.else
	LDI	R0.b1,	0
	.if $isdefed("ENABLE_MULTI_MAKE_RTU") ;store clock plus  value in register  from DMEM for ch0
	    LBCO		&R0.b0,	PRUx_DMEM,	ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;store clock plus  value in register  from DMEM for ch1
	    LBCO		&R0.b0,	PRUx_DMEM,	ENDAT_CH1_NUM_CLOCK_PULSES_OFFSET,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;store clock plus  value in register  from DMEM for ch2
	    LBCO		&R0.b0,	PRUx_DMEM,	ENDAT_CH2_NUM_CLOCK_PULSES_OFFSET,	1
	.else
        QBBC            ENDAT_SKIP8_CH0, ENDAT_ENABLE_CHx,	0
	LBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP8_CH0:
        QBBC            ENDAT_SKIP8_CH1, ENDAT_ENABLE_CHx,	1
	LBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH1_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP8_CH1:
        QBBC            ENDAT_SKIP8_CH2, ENDAT_ENABLE_CHx,	2
	LBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH2_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP8_CH2:
    .endif
	.endif ;ENABLE_MULTI_CHANNEL

        QBEQ    ENDAT_SKIP_INIT_SUCCESS,      R0.b0, 0 ; if eq 0, r0.b0 can be reused as status
        LDI     R0.b0,  1

ENDAT_SKIP_INIT_SUCCESS:
    .if $isdefed("ENABLE_MULTI_MAKE_RTU") ;update success status for ch0
	    SBCO	&R0.b0,	PRUx_DMEM, ENDAT_CH0_INTFC_CMD_STATUS_OFFSET,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;update success  status for ch1
	    SBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH1_INTFC_CMD_STATUS_OFFSET,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;update success status for ch2
	    SBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH2_INTFC_CMD_STATUS_OFFSET,	1
	.else
        SBCO	&R0.b0,	PRUx_DMEM, ENDAT_CH0_INTFC_CMD_STATUS_OFFSET,	1 ; update success status when load share mode is not used
	.endif
    ; status update ends here

	.if $isdefed("ENABLE_MULTI_MAKE_RTU") ; Check RTU host trigger  for ch0
	    LBCO		&R0.b0,	PRUx_DMEM,	ENDAT_CH0_OPMODE_CONFIG_OFFSET,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;Check PRU host trigger  for ch1
	    LBCO		&R0.b0,	PRUx_DMEM,	ENDAT_CH1_OPMODE_CONFIG_OFFSET,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;Check TXPRU host trigger  for ch2
	    LBCO		&R0.b0,	PRUx_DMEM,	ENDAT_CH2_OPMODE_CONFIG_OFFSET,	1
	.else                                    ;check PRU host trigger for all three channels
	    LBCO		&R0.b0,	PRUx_DMEM,	ENDAT_CH0_OPMODE_CONFIG_OFFSET,	1
	.endif
	QBNE	HANDLE_HOST_TRIGGER_MODE,	R0.b0,		0

HANDLE_PERIODIC_TRIGGER_MODE:
	; Get pending events from IEP
	LBCO	&R0,	ICSS_IEP,	ICSS_IEP_CMP_STATUS_REG,	4
	; wait till IEP CMP2 event
	QBBC	HANDLE_PERIODIC_TRIGGER_MODE,	R0,	2
	; Clear IEP CMP2 event
	SET	R0,	R0,	2
	SBCO	&R0,	ICSS_IEP,	ICSS_IEP_CMP_STATUS_REG,	4

	; Let the fall thr' to trigger mode happen properly and trigger bit
	; will be cleared after command processing
	LDI		R0.b0,	1
	.if $isdefed("ENABLE_MULTI_MAKE_RTU") ;set command trigger  for ch0
	    SBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH0_INTFC_CMD_TRIGGER_OFFSET,	1
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;set command trigger   for ch1
	    SBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH1_INTFC_CMD_TRIGGER_OFFSET,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;set command trigger  for ch2
	    SBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH2_INTFC_CMD_TRIGGER_OFFSET,	1
	.else     ;SET command TRIGGER FOR ALL CHANNELS IN WITHOUT LOAD SHARE MODE
	    SBCO    	&R0.b0,	PRUx_DMEM,	ENDAT_CH0_INTFC_CMD_TRIGGER_OFFSET,	1
    .endif

HANDLE_HOST_TRIGGER_MODE:
      .if $isdefed("ENABLE_MULTI_MAKE_RTU") ;wait until command trigger is not set  for ch0
	    LBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH0_INTFC_CMD_TRIGGER_OFFSET,	1
	  .elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;wait command host trigger  is not set  for ch1
	    LBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH1_INTFC_CMD_TRIGGER_OFFSET,	1
	  .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;wait until command trigger  is not set  for ch2
	    LBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH2_INTFC_CMD_TRIGGER_OFFSET,	1
	  .else
	    LBCO	&R0.b0,	PRUx_DMEM,	ENDAT_CH0_INTFC_CMD_TRIGGER_OFFSET,	1 ; without load share mode(one command trigger for all three channels)
	  .endif

	QBBC            HANDLE_HOST_TRIGGER_MODE, R0.b0,	0

	QBBC            ENDAT_SKIP_CONTINUOUS_MODE, R0.b0,	7
     .if !$isdefed("ENABLE_MULTI_CHANNEL") & !$isdefed("ENABLE_MULTI_MAKE_RTU") & !$isdefed("ENABLE_MULTI_MAKE_PRU") & !$isdefed("ENABLE_MULTI_MAKE_TXPRU")
        CALL    FN_CONTINUOUS_MODE
	 .endif

        JMP             ENDAT_HOST_CMD_END

ENDAT_SKIP_CONTINUOUS_MODE:
    .if $isdefed("ENABLE_MULTI_MAKE_RTU") ;command parameters for ch0
	    LBCO   &R0,  PRUx_DMEM, ENDAT_CH0_CMD_WORD_0_OFFSET,   8
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;command parameters for ch1
	    LBCO	&R0,	PRUx_DMEM,	ENDAT_CH1_CMD_WORD_0_OFFSET,	8
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;command parameters for ch2
	    LBCO   &R0,  PRUx_DMEM, ENDAT_CH2_CMD_WORD_0_OFFSET,   8
	.else  ;command parameters for all enabled  channels, when single pru used or  load share mode is nor used
	    LBCO	&R0,	PRUx_DMEM,	ENDAT_CH0_CMD_WORD_0_OFFSET,	8
	.endif

	CALL	FN_SEND_RECEIVE_ENDAT
	QBBS	ENDAT_HOST_CMD_DONE, ENDAT_CMDTYP_NO_SUPPLEMENT_REG,	0
	.if $isdefed("ENABLE_MULTI_MAKE_RTU") ; command supplement parameters for ch0
	    LBCO   &R1,  PRUx_DMEM, ENDAT_CH0_CMD_WORD_2_OFFSET,   4
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;command supplement parameters for ch1
	    LBCO	&R1,	PRUx_DMEM,	ENDAT_CH1_CMD_WORD_2_OFFSET,	4
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU");command  supplement parameters for ch2
	    LBCO   &R1,  PRUx_DMEM, ENDAT_CH2_CMD_WORD_2_OFFSET,   4
	.else  ; command supplement parameters for all channels (without load share mode)
	    LBCO	&R1,	PRUx_DMEM,	ENDAT_CH0_CMD_WORD_2_OFFSET,	4
	.endif

	CALL	FN_SEND_ENDAT22_COMMAND_SUPPLEMENT
ENDAT_HOST_CMD_DONE:
        ;  result in R15-R18, R0.b0 holds CRC status, R2.b0 indicates addinfo presence
   .if $isdefed("ENABLE_MULTI_MAKE_RTU") ;ch0: store encoder response and crc in DMEM
        ; R0.b0 has CRC status
	    MOV		SCRATCH.b0,	R0.b0
	    LBCO	&SCRATCH.b1,	PRUx_DMEM,	ENDAT_CH0_CRC_ERR_COUNTER_OFFSET + 1,	3
	    QBBS		ENDAT_SKIP14A_RTU_CH0,	SCRATCH.b0,	0	; pos/data CRC check
	    ADD		SCRATCH.b1,	SCRATCH.b1,	1
	    ZERO		&R15,	8
ENDAT_SKIP14A_RTU_CH0:
	    QBEQ		ENDAT_SKIP14D_RTU_CH0,	R2.b0,	0	; R2.b0 indicates whether addinfo1/2 present
	    QBBS		ENDAT_SKIP14B_RTU_CH0,	SCRATCH.b0,	1	; addinfo1/2 CRC check
	    ADD		SCRATCH.b2,	SCRATCH.b2,	1
ENDAT_SKIP14D_RTU_CH0:
	    ZERO		&R17,	4
ENDAT_SKIP14B_RTU_CH0:
	    QBNE		ENDAT_SKIP14E_RTU_CH0,	R2.b0,	(0x3 << 3)	; R2.b0 indicates whether both addinfo are present
	    QBBS		ENDAT_SKIP14C_RTU_CH0,	SCRATCH.b0,	2	; addinfo1 CRC check
	    ADD		SCRATCH.b3,	SCRATCH.b3,	1
ENDAT_SKIP14E_RTU_CH0:
	    ZERO		&R18,	4
ENDAT_SKIP14C_RTU_CH0:

	    SBCO	&R15,	PRUx_DMEM,	ENDAT_CH0_POSITION_DATA_WORD0_OFFSET,	16
	    SBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH0_CRC_ERR_COUNTER_OFFSET,	4
   .elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;ch1 store encoder response and crc in DMEM
        MOV		SCRATCH.b0,	R0.b0
	    LBCO	&SCRATCH.b1,	PRUx_DMEM,	ENDAT_CH1_CRC_ERR_COUNTER_OFFSET + 1,	3
	    QBBS		ENDAT_SKIP14A_PRU_CH1,	SCRATCH.b0,	0	; pos/data CRC check
	    ADD		SCRATCH.b1,	SCRATCH.b1,	1
	    ZERO		&R15,	8
ENDAT_SKIP14A_PRU_CH1:
	    QBEQ		ENDAT_SKIP14D_PRU_CH1,	R2.b0,	0	; R2.b0 indicates whether addinfo1/2 present
	    QBBS		ENDAT_SKIP14B_PRU_CH1,	SCRATCH.b0,	1	; addinfo1/2 CRC check
	    ADD		SCRATCH.b2,	SCRATCH.b2,	1
ENDAT_SKIP14D_PRU_CH1:
	    ZERO		&R17,	4
ENDAT_SKIP14B_PRU_CH1:
	    QBNE		ENDAT_SKIP14E_PRU_CH1,	R2.b0,	(0x3 << 3)	; R2.b0 indicates whether both addinfo are present
	    QBBS		ENDAT_SKIP14C_PRU_CH1,	SCRATCH.b0,	2	; addinfo1 CRC check
	    ADD		SCRATCH.b3,	SCRATCH.b3,	1
ENDAT_SKIP14E_PRU_CH1:
	    ZERO		&R18,	4
ENDAT_SKIP14C_PRU_CH1:

	    SBCO	&R15,	PRUx_DMEM,	ENDAT_CH1_POSITION_DATA_WORD0_OFFSET,	16
	    SBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH1_CRC_ERR_COUNTER_OFFSET,	4
   .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU");ch2 store encoder response and crc in DMEM
        MOV		SCRATCH.b0,	R0.b0
	    LBCO	&SCRATCH.b1,	PRUx_DMEM,	ENDAT_CH2_CRC_ERR_COUNTER_OFFSET + 1,	3
	    QBBS		ENDAT_SKIP14A_TXPRU_CH2,	SCRATCH.b0,	0	; pos/data CRC check
	    ADD		SCRATCH.b1,	SCRATCH.b1,	1
	    ZERO		&R15,	8
ENDAT_SKIP14A_TXPRU_CH2:
	    QBEQ		ENDAT_SKIP14D_TXPRU_CH2,	R2.b0,	0	; R2.b0 indicates whether addinfo1/2 present
	    QBBS		ENDAT_SKIP14B_TXPRU_CH2,	SCRATCH.b0,	1	; addinfo1/2 CRC check
	    ADD		SCRATCH.b2,	SCRATCH.b2,	1
ENDAT_SKIP14D_TXPRU_CH2:
	    ZERO		&R17,	4
ENDAT_SKIP14B_TXPRU_CH2:
	    QBNE		ENDAT_SKIP14E_TXPRU_CH2,	R2.b0,	(0x3 << 3)	; R2.b0 indicates whether both addinfo are present
	    QBBS		ENDAT_SKIP14C_TXPRU_CH2,	SCRATCH.b0,	2	; addinfo1 CRC check
	    ADD		SCRATCH.b3,	SCRATCH.b3,	1
ENDAT_SKIP14E_TXPRU_CH2:
	    ZERO		&R18,	4
ENDAT_SKIP14C_TXPRU_CH2:

	    SBCO	&R15,	PRUx_DMEM,	ENDAT_CH2_POSITION_DATA_WORD0_OFFSET,	16
	    SBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH2_CRC_ERR_COUNTER_OFFSET,	4


    .else    ;store store encoder response and crc when load share mode is not enabled
    QBBC            ENDAT_SKIP14_CH0, ENDAT_ENABLE_CHx,	0
	; R0.b0 has CRC status
	MOV		SCRATCH.b0,	R0.b0
	LBCO	&SCRATCH.b1,	PRUx_DMEM,	ENDAT_CH0_CRC_ERR_COUNTER_OFFSET + 1,	3
	QBBS		ENDAT_SKIP14A_CH0,	SCRATCH.b0,	0	; pos/data CRC check
	ADD		SCRATCH.b1,	SCRATCH.b1,	1
	ZERO		&R15,	8
ENDAT_SKIP14A_CH0:
	QBEQ		ENDAT_SKIP14D_CH0,	R2.b0,	0	; R2.b0 indicates whether addinfo1/2 present
	QBBS		ENDAT_SKIP14B_CH0,	SCRATCH.b0,	1	; addinfo1/2 CRC check
	ADD		SCRATCH.b2,	SCRATCH.b2,	1
ENDAT_SKIP14D_CH0:
	ZERO		&R17,	4
ENDAT_SKIP14B_CH0:
	QBNE		ENDAT_SKIP14E_CH0,	R2.b0,	(0x3 << 3)	; R2.b0 indicates whether both addinfo are present
	QBBS		ENDAT_SKIP14C_CH0,	SCRATCH.b0,	2	; addinfo1 CRC check
	ADD		SCRATCH.b3,	SCRATCH.b3,	1
ENDAT_SKIP14E_CH0:
	ZERO		&R18,	4
ENDAT_SKIP14C_CH0:

	SBCO	&R15,	PRUx_DMEM,	ENDAT_CH0_POSITION_DATA_WORD0_OFFSET,	16
	SBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH0_CRC_ERR_COUNTER_OFFSET,	4
ENDAT_SKIP14_CH0:

        QBBC            ENDAT_SKIP14_CH1, ENDAT_ENABLE_CHx,	1
	; R0.b0 has CRC status
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	MOV		SCRATCH.b0,	R0.b1
	.else
	MOV		SCRATCH.b0,	R0.b0
	.endif
	LBCO	&SCRATCH.b1,	PRUx_DMEM,	ENDAT_CH1_CRC_ERR_COUNTER_OFFSET + 1,	3
	QBBS		ENDAT_SKIP14A_CH1,	SCRATCH.b0,	0	; pos/data CRC check
	ADD		SCRATCH.b1,	SCRATCH.b1,	1
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	ZERO		&R19,	8
	.else
	ZERO		&R15,	8
	.endif
ENDAT_SKIP14A_CH1:
	QBEQ		ENDAT_SKIP14D_CH1,	R2.b0,	0	; R2.b0 indicates whether addinfo1/2 present
	QBBS		ENDAT_SKIP14B_CH1,	SCRATCH.b0,	1	; addinfo1/2 CRC check
	ADD		SCRATCH.b2,	SCRATCH.b2,	1
ENDAT_SKIP14D_CH1:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	ZERO		&R21,	4
	.else
	ZERO		&R17,	4
	.endif
ENDAT_SKIP14B_CH1:
	QBNE		ENDAT_SKIP14E_CH1,	R2.b0,	(0x3 << 3)	; R2.b0 indicates whether both addinfo are present
	QBBS		ENDAT_SKIP14C_CH1,	SCRATCH.b0,	2	; addinfo1 CRC check
	ADD		SCRATCH.b3,	SCRATCH.b3,	1
ENDAT_SKIP14E_CH1:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	ZERO		&R22,	4
	.else
	ZERO		&R18,	4
	.endif
ENDAT_SKIP14C_CH1:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	SBCO	&R19,	PRUx_DMEM,	ENDAT_CH1_POSITION_DATA_WORD0_OFFSET,	16
	.else
	SBCO	&R15,	PRUx_DMEM,	ENDAT_CH1_POSITION_DATA_WORD0_OFFSET,	16
	.endif
	SBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH1_CRC_ERR_COUNTER_OFFSET,	4
ENDAT_SKIP14_CH1:

        QBBC            ENDAT_SKIP14_CH2, ENDAT_ENABLE_CHx,	2
	; R0.b0 has CRC status
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	MOV		SCRATCH.b0,	R0.b2
	.else
	MOV		SCRATCH.b0,	R0.b0
	.endif
	LBCO	&SCRATCH.b1,	PRUx_DMEM,	ENDAT_CH2_CRC_ERR_COUNTER_OFFSET + 1,	3
	QBBS		ENDAT_SKIP14A_CH2,	SCRATCH.b0,	0	; pos/data CRC check
	ADD		SCRATCH.b1,	SCRATCH.b1,	1
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	ZERO		&R23,	8
	.else
	ZERO		&R15,	8
	.endif
ENDAT_SKIP14A_CH2:
	QBEQ		ENDAT_SKIP14D_CH2,	R2.b0,	0	; R2.b0 indicates whether addinfo1/2 present
	QBBS		ENDAT_SKIP14B_CH2,	SCRATCH.b0,	1	; addinfo1/2 CRC check
	ADD		SCRATCH.b2,	SCRATCH.b2,	1
ENDAT_SKIP14D_CH2:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	ZERO		&R25,	4
	.else
	ZERO		&R17,	4
	.endif
ENDAT_SKIP14B_CH2:
	QBNE		ENDAT_SKIP14E_CH2,	R2.b0,	(0x3 << 3)	; R2.b0 indicates whether both addinfo are present
	QBBS		ENDAT_SKIP14C_CH2,	SCRATCH.b0,	2	; addinfo1 CRC check
	ADD		SCRATCH.b3,	SCRATCH.b3,	1
ENDAT_SKIP14E_CH2:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	ZERO		&R26,	4
	.else
	ZERO		&R18,	4
	.endif
ENDAT_SKIP14C_CH2:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	SBCO	&R23,	PRUx_DMEM,	ENDAT_CH2_POSITION_DATA_WORD0_OFFSET,	16
	.else
	SBCO	&R15,	PRUx_DMEM,	ENDAT_CH2_POSITION_DATA_WORD0_OFFSET,	16
	.endif
	SBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH2_CRC_ERR_COUNTER_OFFSET,	4
ENDAT_SKIP14_CH2:
  .endif

ENDAT_HOST_CMD_END:

     LDI  R3.w0, 0
	.if $isdefed("ENABLE_MULTI_MAKE_RTU") ;clear command trigger  for ch0
	    SBCO	&R3.b0,	PRUx_DMEM,	ENDAT_CH0_INTFC_CMD_TRIGGER_OFFSET,	1
        LBCO		&SCRATCH.b0,	PRUx_DMEM,	ENDAT_CH0_OPMODE_CONFIG_OFFSET,	1  ; Check RTU host trigger  for ch0
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;clear command trigger   for ch1
	    SBCO	&R3.b0,	PRUx_DMEM,	ENDAT_CH1_INTFC_CMD_TRIGGER_OFFSET,	1
        LBCO		&SCRATCH.b0,	PRUx_DMEM,	ENDAT_CH1_OPMODE_CONFIG_OFFSET,	1  ;Check PRU host trigger  for ch1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;clear command trigger  for ch2
	    SBCO	&R3.b0,	PRUx_DMEM,	ENDAT_CH2_INTFC_CMD_TRIGGER_OFFSET,	1
        LBCO		&SCRATCH.b0,	PRUx_DMEM,	ENDAT_CH2_OPMODE_CONFIG_OFFSET,	1  ;Check TXPRU host trigger  for ch2
	.else  ; when load share mode is not used
	    SBCO	&R3.b0,	PRUx_DMEM,	ENDAT_CH0_INTFC_CMD_TRIGGER_OFFSET,	1
        LBCO		&SCRATCH.b0,	PRUx_DMEM,	ENDAT_CH0_OPMODE_CONFIG_OFFSET,	1 ;check PRU host trigger for all three channels
	.endif

	QBEQ		HANDLE_PERIODIC_TRIGGER_MODE,	SCRATCH.b0,		0
	JMP		HANDLE_HOST_TRIGGER_MODE


	.if	$isdefed("ENABLE_MULTI_CHANNEL")
M_CALC_RECOV_TIME_MULTI_CH .macro
    ;enable PRU cycle counter
    M_ENABLE_PRU_CYCLE_COUNTER
	ZERO		&R14,	4
 ; waiting for raising edge of clock for all connected channels
ENDAT_TD_LAST_RISING_CLOCK_MULTI_CH?:
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_TXCFG  ;read clcok_status
	LBCO	&R27.w0,	ICSS_CFG,	SCRATCH1.w0,	2
    AND     R27.b2, R27.b1, ENDAT_ENABLE_CHx_IN_USE  ; doing  AND of 8th(ch0), 9th(ch1) and 10th(ch2) bits with channel mask
    QBNE   ENDAT_TD_LAST_RISING_CLOCK_MULTI_CH?,  R27.b2, ENDAT_ENABLE_CHx_IN_USE ; check clock_status bits for all connected channels
;set PRU counter to zero
	SBCO 	&R14, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
;waiting for RX complete
    LDI  R28.w0, 0
W_RX_MULTI?:
    QBBC    ENDAT_SKIP_MULTI_WRX_CH0?, ENDAT_ENABLE_CHx_IN_USE,	0
    QBBS    ENDAT_SKIP_MULTI_WRX_CH0?, R28.b1,	0
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0
	LBCO	&R27,	ICSS_CFG,	SCRATCH1.w0,	4
	QBBS	ENDAT_SKIP_MULTI_WRX_CH0?,	R27,	28
    LBCO 	&R27, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
    SBCO    &R27, PRUx_DMEM,  ENDAT_CH0_RT_OFFSET, 4
    SET     R28.b1, R28.b1, 0
ENDAT_SKIP_MULTI_WRX_CH0?:

;ch1 waiting for RX complete
    QBBC    ENDAT_SKIP_MULTI_WRX_CH1?, ENDAT_ENABLE_CHx_IN_USE,	1
    QBBS    ENDAT_SKIP_MULTI_WRX_CH1?, R28.b1,	1
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0
	LBCO	&R27,	ICSS_CFG,	SCRATCH1.w0,	4
	QBBS	ENDAT_SKIP_MULTI_WRX_CH1?,	R27,	28
    LBCO 	&R27, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
    SBCO    &R27, PRUx_DMEM,  ENDAT_CH1_RT_OFFSET, 4
    SET     R28.b1, R28.b1, 1
ENDAT_SKIP_MULTI_WRX_CH1?:
;ch2 waiting for RX complete
    QBBC    ENDAT_SKIP_MULTI_WRX_CH2?, ENDAT_ENABLE_CHx_IN_USE,	2
    QBBS    ENDAT_SKIP_MULTI_WRX_CH2?, R28.b1,	2
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0
	LBCO	&R27,	ICSS_CFG,	SCRATCH1.w0,	4
	QBBS	ENDAT_SKIP_MULTI_WRX_CH2?,	R27,	28
    LBCO 	&R27, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
    SBCO    &R27, PRUx_DMEM,  ENDAT_CH2_RT_OFFSET, 4
    SET     R28.b1, R28.b1, 2
ENDAT_SKIP_MULTI_WRX_CH2?:
    AND     R28.b0, R28.b1, ENDAT_ENABLE_CHx_IN_USE
    QBNE    W_RX_MULTI?,  R28.b0, ENDAT_ENABLE_CHx_IN_USE
    .endm


; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; receive & down sample (invoked for non-CRC bits)
; Returns: received data in Rx, Ry, Rz - caller has to zero or provide value to continue
; cnt - number of rx bits, should be <= 32
; ff[0-4] - flip flops, ex - temporary variable
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	.macro	Rx, Ry, Rz, cnt, ff0, ff1, ff2, ff3, ff4, ex
	LOOP            RX_RECEIVE_DOWNSAMPLE_CRC_LOOP?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
	LSL		Ry,	Ry,	1
	LSL		Rz,	Rz,	1
WB13?:
	AND		SCRATCH.b0,	R31.b3,	ENDAT_ENABLE_CHx_IN_USE
	QBNE		WB13?,	SCRATCH.b0,	ENDAT_ENABLE_CHx_IN_USE ;  wait for valid

	QBBS	        ENDAT_SKIP33_CH0?,	R31,	4 ;  Check the Mid bit of received oversampled data
	MOV		ex.b0,	ff4.b0	; if code[i] = 0, ex = ff4 (ex = ff4 ^ 0)
	JMP		ENDAT_SKIP33A_CH0?
ENDAT_SKIP33_CH0?:
	NOT		ex.b0,	ff4.b0	; if code[i] = 1, ex = !ff4 (ex = ff4 ^ 1)
	OR		Rx,	Rx,	1
ENDAT_SKIP33A_CH0?:

	QBBS	        ENDAT_SKIP33_CH1?,	R31,	4 + 8 ;  Check the Mid bit of received oversampled data
	MOV		ex.b1,	ff4.b1	; if code[i] = 0, ex = ff4 (ex = ff4 ^ 0)
	JMP		ENDAT_SKIP33A_CH1?
ENDAT_SKIP33_CH1?:
	NOT		ex.b1,	ff4.b1	; if code[i] = 1, ex = !ff4 (ex = ff4 ^ 1)
	OR		Ry,	Ry,	1
ENDAT_SKIP33A_CH1?:

	QBBS	        ENDAT_SKIP33_CH2?,	R31,	4 + 16 ;  Check the Mid bit of received oversampled data
	MOV		ex.b2,	ff4.b2	; if code[i] = 0, ex = ff4 (ex = ff4 ^ 0)
	JMP		ENDAT_SKIP33A_CH2?
ENDAT_SKIP33_CH2?:
	NOT		ex.b2,	ff4.b2	; if code[i] = 1, ex = !ff4 (ex = ff4 ^ 1)
	OR		Rz,	Rz,	1
ENDAT_SKIP33A_CH2?:

	MOV		R31.b3,	SCRATCH.b0 ;  clear valid bit

	MOV		ff4,	ff3
	XOR		ff3,	ff2,	ex
	MOV		ff2,	ff1
	XOR		ff1,	ff0,	ex
	MOV		ff0,	ex
RX_RECEIVE_DOWNSAMPLE_CRC_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; receive & down sample (invoked for CRC bits)
; Returns: received CRC bits pushed onto Rx, Ry, Rz - caller has to provide value to continue, Ra - received CRC
; cnt - number of rx bits, should be <= 32
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
M_OTF_RECEIVE_AND_DOWNSAMPLE	.macro	Rx, Ry, Rz, Ra, cnt
	LOOP            RX_RECEIVE_DOWNSAMPLE_LOOP?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
	LSL		Ry,	Ry,	1
	LSL		Rz,	Rz,	1
	LSL		Ra,	Ra,	1 ; this will do for all 3 channels
WB14?:
	AND		SCRATCH.b0,	R31.b3,	ENDAT_ENABLE_CHx_IN_USE
	QBNE		WB14?,	SCRATCH.b0,	ENDAT_ENABLE_CHx_IN_USE ;  wait for valid

	QBBS	        ENDAT_SKIP34_CH0?,	R31,	4 ;  Check the Mid bit of received oversampled data
	JMP		ENDAT_SKIP34A_CH0?
ENDAT_SKIP34_CH0?:
	OR		Rx,	Rx,	1
	OR		Ra.b0,	Ra.b0,	1
ENDAT_SKIP34A_CH0?:

	QBBS	        ENDAT_SKIP34_CH1?,	R31,	4 + 8 ;  Check the Mid bit of received oversampled data
	JMP		ENDAT_SKIP34A_CH1?
ENDAT_SKIP34_CH1?:
	OR		Ry,	Ry,	1
	OR		Ra.b1,	Ra.b1,	1
ENDAT_SKIP34A_CH1?:

	QBBS	        ENDAT_SKIP34_CH2?,	R31,	4 + 16 ;  Check the Mid bit of received oversampled data
	JMP		ENDAT_SKIP34A_CH2?
ENDAT_SKIP34_CH2?:
	OR		Rz,	Rz,	1
	OR		Ra.b2,	Ra.b2,	1
ENDAT_SKIP34A_CH2?:

	MOV		R31.b3,	SCRATCH.b0 ;  clear valid bit
RX_RECEIVE_DOWNSAMPLE_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Format received CRC - transalate byte to bit & reverse
; requires: ff[0-4] in	R6, R7, R8, R9, R10
; returns: formattted CRC in R12
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
FN_FORMAT_CRC:
	ZERO		&R12,	4

	LOOP		ENDAT_CRC_CH0_END?,	3

	LSL		R6,	R6,	8
	LSL		R7,	R7,	8
	LSL		R8,	R8,	8
	LSL		R9,	R9,	8
	LSL		R10,	R10,	8

	; ch2 would be initially in b0, at the end of the loop it would have reached b2, finally ch[0-2] would end up @b[0-2] as reqd.
	; no-op in first interation
	LSL		R12,	R12,	8

	QBBS		ENDAT_CRC_CH0_BIT1?,	R6.b3,	0
	SET		R12.b0,	R12.b0,	0
ENDAT_CRC_CH0_BIT1?:
	QBBS		ENDAT_CRC_CH0_BIT2?,	R7.b3,	0
	SET		R12.b0,	R12.b0,	1
ENDAT_CRC_CH0_BIT2?:
	QBBS		ENDAT_CRC_CH0_BIT3?,	R8.b3,	0
	SET		R12.b0,	R12.b0,	2
ENDAT_CRC_CH0_BIT3?:
	QBBS		ENDAT_CRC_CH0_BIT4?,	R9.b3,	0
	SET		R12.b0,	R12.b0,	3
ENDAT_CRC_CH0_BIT4?:
	QBBS		ENDAT_CRC_CH0_END?,	R10.b3,	0
	SET		R12.b0,	R12.b0,	4
ENDAT_CRC_CH0_END?:

	RET2

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; receive, down sample, calculate CRC
; Returns: received data (Ra, Rb, Rc, Rd, Re, Rf), CRC status (Ts-th bit of R0.b[0-2]) - set for success, clear for failure
; Requires: rx_size (R2.w1)
; Uses: R6.b[0-2] (ff0), R7.b[0-2] (ff1), R8.b[0-2] (ff2), R9.b[0-2] (ff3), R10.b[0-2] (ff4), R11.b[0-2](ex,crc_recvd), R12.b[0-2](crc_calc), R2.b3(cnt)
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
M_OTF_RECEIVE	.macro	Ra, Rb, Rc, Rd, Re, Rf, Rt, Ts

	; initialize CRC flip-flops, i.e. ff[0-4] = 1
	LDI32		R6,	0x010101
	MOV		R7,	R6
	MOV		R8,	R6
	MOV		R9,	R6
	MOV		R10,	R6

	; wait for valid bit
WB11?:
	AND		SCRATCH.b0,	R31.b3,	ENDAT_ENABLE_CHx_IN_USE
	QBNE		WB11?,	SCRATCH.b0,	ENDAT_ENABLE_CHx_IN_USE;  wait for valid
	; Note: 2 cycles are reqd before again checking for valid bit, and that is achieved below
	MOV		R31.b3,	SCRATCH.b0 ;  clear valid bit

	; receive data
	SUB		R2.b3,	R2.w1,	5	; minus CRC bits
	QBGE		ENDAT_RX_LE_32_BITS?,	R2.b3,	32
	LDI		R2.b3,	32
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	Ra, Rc, Re, R2.b3, R6, R7, R8, R9, R10, R11
	SUB		R2.b3,	R2.w1,	(32 + 5)	; note taking care of CRC bits
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	Rb, Rd, Rf, R2.b3, R6, R7, R8, R9, R10, R11
	ZERO		&R11,	4	; clear crc_recvd
	LDI		R2.b3,	5
	M_OTF_RECEIVE_AND_DOWNSAMPLE	Rb, Rd, Rf, R11, R2.b3
	JMP		ENDAT_RX_CRC?
ENDAT_RX_LE_32_BITS?:
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	Ra, Rc, Re, R2.b3, R6, R7, R8, R9, R10, R11
	ZERO		&R11,	4	; clear crc_recvd
	RSB		R2.b3,	R2.b3,	32
	MIN		R2.b3,	R2.b3,	5
	M_OTF_RECEIVE_AND_DOWNSAMPLE	Ra, Rc, Re, R11, R2.b3
	QBEQ		ENDAT_RX_CRC?,	R2.b3,	5
	RSB		R2.b3,	R2.b3,	5
	M_OTF_RECEIVE_AND_DOWNSAMPLE	Rb, Rd, Rf, R11, R2.b3
ENDAT_RX_CRC?:
    QBEQ    ENDAT_MULTI_SKIP_CALCU_RT?, Rt,	0
    QBBC    ENDAT_MULTI_SKIP_CALCU_RT?,  Rt,   Ts
    M_CALC_RECOV_TIME_MULTI_CH
ENDAT_MULTI_SKIP_CALCU_RT?:
	; FN_FORMAT_CRC returns formatted CRC in R12, requires R6-R10 (ff0-4)
        CALL2		FN_FORMAT_CRC

	; crc: calculated - R12, received - R11
	QBEQ		ENDAT_CRC_CH0_SUCCESS?,	R12.b0,	R11.b0
	CLR		R0.b0,	R0.b0,	Ts
	JMP		ENDAT_CRC_CH0_FAILURE?
ENDAT_CRC_CH0_SUCCESS?:
	SET		R0.b0,	R0.b0,	Ts
ENDAT_CRC_CH0_FAILURE?:

	QBEQ		ENDAT_CRC_CH1_SUCCESS?,	R12.b1,	R11.b1
	CLR		R0.b1,	R0.b1,	Ts
	JMP		ENDAT_CRC_CH1_FAILURE?
ENDAT_CRC_CH1_SUCCESS?:
	SET		R0.b1,	R0.b1,	Ts
ENDAT_CRC_CH1_FAILURE?:

	QBEQ		ENDAT_CRC_CH2_SUCCESS?,	R12.b2,	R11.b2
	CLR		R0.b2,	R0.b2,	Ts
	JMP		ENDAT_CRC_CH2_FAILURE?
ENDAT_CRC_CH2_SUCCESS?:
	SET		R0.b2,	R0.b2,	Ts
ENDAT_CRC_CH2_FAILURE?:
	.endm
	.else ; ENABLE_MULTI_CHANNEL



; macro for ch0
M_CALC_RECOV_TIME_CH0 .macro
    ;enable PRU cycle counter
    M_ENABLE_PRU_CYCLE_COUNTER
	ZERO		&R14,	4
ENDAT_TD_LAST_RISING_CLOCK_CH0?:
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_TXCFG
	LBCO	&R27.w0,	ICSS_CFG,	SCRATCH1.w0,	2
    QBBC            ENDAT_TD_LAST_RISING_CLOCK_CH0?,  R27.w0,  8
    ;set pru counter to zero
	SBCO 	&R14, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
    ; wait for rx complete
WRXCH0?:
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0
	LBCO	&R27,	ICSS_CFG,	SCRATCH1.w0,	4
	QBBS		WRXCH0?,	R27,	28

    ; read pru counter at time when rx start

	LBCO 	&R27, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
    ; store in DMEM
    SBCO    &R27, PRUx_DMEM,  ENDAT_CH0_RT_OFFSET, 4

   .endm
;macro for channel 1
M_CALC_RECOV_TIME_CH1 .macro
    ; enable PRU cycle counter
    M_ENABLE_PRU_CYCLE_COUNTER
	ZERO		&R14,	4
ENDAT_TD_LAST_RISING_CLOCK_CH1?:
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_TXCFG
	LBCO	&R27.w0,	ICSS_CFG,	SCRATCH1.w0,	2
    QBBC            ENDAT_TD_LAST_RISING_CLOCK_CH1?,  R27.w0,  9

    ;set pru counter to zero
	SBCO 	&R14, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4

    ; wait for rx complete
WRXCH1?:
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0
	LBCO	&R27,	ICSS_CFG,	SCRATCH1.w0,	4
	QBBS		WRXCH1?,	R27,	28

    ; read pru counter at time when rx start
	LBCO 	&R27, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4

    ; store in DMEM
    SBCO    &R27, PRUx_DMEM,  ENDAT_CH1_RT_OFFSET, 4

   .endm
;macro for ch2
M_CALC_RECOV_TIME_CH2 .macro
    ;enable PRU counter
    M_ENABLE_PRU_CYCLE_COUNTER
	ZERO		&R14,	4
ENDAT_TD_LAST_RISING_CLOCK_CH2?:
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_TXCFG
	LBCO	&R27.w0,	ICSS_CFG,	SCRATCH1.w0,	2
    QBBC            ENDAT_TD_LAST_RISING_CLOCK_CH2?,  R27.w0,  10

    ;set pru counter to zero
    .if $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	    SBCO 	&R14, c28, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
    .else
	    SBCO 	&R14, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
	.endif


        ; wait for rx complete
WRXCH2?:
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0
	LBCO	&R27,	ICSS_CFG,	SCRATCH1.w0,	4
	QBBS		WRXCH2?,	R27,	28


    ; read pru counter at time when rx start
    .if $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	    LBCO 	&R27, c28, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
    .else
	    LBCO 	&R27, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
	.endif

    ; store in DMEM
    SBCO    &R27, PRUx_DMEM,  ENDAT_CH2_RT_OFFSET, 4
   .endm

   ;macro for RT
M_RECOVERY_TIME_CALCU .macro
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
        M_CALC_RECOV_TIME_CH0
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
        M_CALC_RECOV_TIME_CH1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
        M_CALC_RECOV_TIME_CH2
	.else
        QBBC            ENDAT_SKIP_RT_CH0?, ENDAT_ENABLE_CHx,	0
        M_CALC_RECOV_TIME_CH0
ENDAT_SKIP_RT_CH0?:
        QBBC            ENDAT_SKIP_RT_CH1?, ENDAT_ENABLE_CHx,	1
        M_CALC_RECOV_TIME_CH1
ENDAT_SKIP_RT_CH1?:
        QBBC            ENDAT_SKIP_RT_CH2?, ENDAT_ENABLE_CHx,	2
        M_CALC_RECOV_TIME_CH2
ENDAT_SKIP_RT_CH2?:
    .endif
    .endm
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; receive & down sample (invoked for non-CRC bits)
; Returns: received data in Rx - caller has to zero or provide value to continue
; valid_bit - valid bit for selected channel, bit_idx - midbit of rx fifo of selected channel, cnt - number of rx bits, should be <= 32
; ff[0-4] - flip flops, ex - temporary variable
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	.macro	Rx, cnt, valid_bit, bit_idx, ff0, ff1, ff2, ff3, ff4, ex
	LOOP            RX_RECEIVE_DOWNSAMPLE_CRC_LOOP?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
WB13?:
	QBBC		WB13?,	R31,	valid_bit ;  wait for valid
	QBBS	        ENDAT_SKIP33_CHx?,	R31,	bit_idx	;  Check the Mid bit of received oversampled data
	MOV		ex,	ff4	; if code[i] = 0, ex = ff4 (ex = ff4 ^ 0)
	JMP		ENDAT_SKIP33A_CHx?
ENDAT_SKIP33_CHx?:
	NOT		ex,	ff4	; if code[i] = 1, ex = !ff4 (ex = ff4 ^ 1)
	OR		Rx,	Rx,	1
ENDAT_SKIP33A_CHx?:
	SET		R31,	R31, valid_bit ;  clear valid bit
	MOV		ff4,	ff3
	XOR		ff3,	ff2,	ex
	MOV		ff2,	ff1
	XOR		ff1,	ff0,	ex
	MOV		ff0,	ex
RX_RECEIVE_DOWNSAMPLE_CRC_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; receive & down sample (invoked for CRC bits)
; Returns: received CRC bits pushed onto Rx - caller has to provide value to continue, Ry - received CRC
; valid_bit - valid bit for selected channel, bit_idx - midbit of rx fifo of selected channel, cnt - number of rx bits, should be <= 32
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
M_OTF_RECEIVE_AND_DOWNSAMPLE	.macro	Rx, Ry, cnt, valid_bit, bit_idx
	LOOP            RX_RECEIVE_DOWNSAMPLE_LOOP?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
	LSL		Ry,	Ry,	1
WB14?:
	QBBC		WB14?,	R31,	valid_bit ;  wait for valid
	QBBS	        ENDAT_SKIP34_CHx?,	R31,	bit_idx ;  Check the Mid bit of received oversampled data
	JMP		ENDAT_SKIP34A_CHx?
ENDAT_SKIP34_CHx?:
	OR		Rx,	Rx,	1
	OR		Ry,	Ry,	1
ENDAT_SKIP34A_CHx?:
	SET		R31,	R31, valid_bit ;  clear valid bit
RX_RECEIVE_DOWNSAMPLE_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; calculate CRC, result in crc
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
M_CALC_CRC	.macro	crc, ff0, ff1, ff2, ff3, ff4
	LDI		crc,	0
	QBBS		ENDAT_CRC_BIT1?,	ff0,	0
	SET		crc,	crc,	0
ENDAT_CRC_BIT1?:
	QBBS		ENDAT_CRC_BIT2?,	ff1,	0
	SET		crc,	crc,	1
ENDAT_CRC_BIT2?:
	QBBS		ENDAT_CRC_BIT3?,	ff2,	0
	SET		crc,	crc,	2
ENDAT_CRC_BIT3?:
	QBBS		ENDAT_CRC_BIT4?,	ff3,	0
	SET		crc,	crc,	3
ENDAT_CRC_BIT4?:
	QBBS		ENDAT_CRC_END?,	ff4,	0
	SET		crc,	crc,	4
ENDAT_CRC_END?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; receive, down sample, calculate CRC
; Returns: received data (Ra,Rb), CRC status (Ts-th bit of R0.b0) - set for success, clear for failure
; Requires: rx_size (R2.w1), valid_bit (R3.b0), bit_idx (R3.b1)
; Uses: R12.b0 (ff0), R12.b1 (ff1), R12.b2 (ff2), R12.b3 (ff3), R3.b2 (ff4), R3.b3(ex,crc_recvd), R2.b3 (crc_calc,cnt)
; REVISIT: More optimization may be required or can be done at least in M_INIT_CRC_FLIP_FLOPS - defer those till a requirement comes
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
M_OTF_RECEIVE	.macro	Ra, Rb, Rt, Ts

	; initialize CRC flip-flops, i.e. ff[0-4] = 1
	LDI32		R12,	0x01010101
	LDI		R3.b2,	0x01

	; wait for valid bit
WB11?:
	QBBC		WB11?,	R31,	R3.b0 ;  wait for valid
	; Note: 2 cycles are reqd before again checking for valid bit, and that is achieved below
	SET		R31,	R31, R3.b0 ;  clear valid bit

	; receive data
	SUB		R2.b3,	R2.w1,	5	; minus CRC bits
	QBGE		ENDAT_RX_LE_32_BITS?,	R2.b3,	32
	LDI		R2.b3,	32
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	Ra, R2.b3, R3.b0, R3.b1, R12.b0, R12.b1, R12.b2, R12.b3, R3.b2, R3.b3
	SUB		R2.b3,	R2.w1,	(32 + 5)	; note taking care of CRC bits
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	Rb, R2.b3, R3.b0, R3.b1, R12.b0, R12.b1, R12.b2, R12.b3, R3.b2, R3.b3
	LDI		R3.b3,	0	; clear crc_recvd
	LDI		R2.b3,	5
	M_OTF_RECEIVE_AND_DOWNSAMPLE	Rb, R3.b3, R2.b3, R3.b0, R3.b1
	JMP		ENDAT_RX_CRC?
ENDAT_RX_LE_32_BITS?:
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	Ra, R2.b3, R3.b0, R3.b1, R12.b0, R12.b1, R12.b2, R12.b3, R3.b2, R3.b3
	LDI		R3.b3,	0	; clear crc_recvd
	RSB		R2.b3,	R2.b3,	32
	MIN		R2.b3,	R2.b3,	5
	M_OTF_RECEIVE_AND_DOWNSAMPLE	Ra, R3.b3, R2.b3, R3.b0, R3.b1
	QBEQ		ENDAT_RX_CRC?,	R2.b3,	5
	RSB		R2.b3,	R2.b3,	5
	M_OTF_RECEIVE_AND_DOWNSAMPLE	Rb, R3.b3, R2.b3, R3.b0, R3.b1
ENDAT_RX_CRC?:
    QBEQ    ENDAT_SKIP_CALCU_RT?, Rt,	0
    QBBC    ENDAT_SKIP_CALCU_RT?,  Rt,   Ts
    M_RECOVERY_TIME_CALCU
ENDAT_SKIP_CALCU_RT?:
	M_CALC_CRC	R2.b3, R12.b0, R12.b1, R12.b2, R12.b3, R3.b2

	QBEQ		ENDAT_CRC_SUCCESS?,	R2.b3,	R3.b3	; crc: calculated - R2.b3, received - R3.b3
	CLR		R0.b0,	R0.b0,	Ts
	JMP		ENDAT_RECEIVE_DOWN_SAMPLE_END?
ENDAT_CRC_SUCCESS?:
	SET		R0.b0,	R0.b0,	Ts
ENDAT_RECEIVE_DOWN_SAMPLE_END?:
	.endm

	.endif	; ENABLE_MULTI_CHANNEL


; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; continuous mode handling
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
	.if	!$isdefed("ENABLE_MULTI_CHANNEL")
FN_CONTINUOUS_MODE:

	LBCO    	&R0,	PRUx_DMEM,	ENDAT_CH0_CMD_WORD_0_OFFSET,	8

        QBBC            ENDAT_SKIP31_CH0, ENDAT_ENABLE_CHx,	0
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN | ENDAT_TX_CH0_SEL)
ENDAT_SKIP31_CH0:
        QBBC            ENDAT_SKIP31_CH1, ENDAT_ENABLE_CHx,	1
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN | ENDAT_TX_CH1_SEL)
ENDAT_SKIP31_CH1:
        QBBC            ENDAT_SKIP31_CH2, ENDAT_ENABLE_CHx,	2
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN | ENDAT_TX_CH2_SEL)
ENDAT_SKIP31_CH2:

	MOV		R30.b0,	R0.b0 ;  load cmd to tx fifo

	MOV		R2.w1,	R1.b0
	LSL		R1.b0,	R1.b1,	3
        CALL2		FN_SEND

LOOP_CONTINUOUS_MODE:

	LDI32	        R15, 0
	LDI32	        R16, 0

	; store valid bit & rx fifo bit to be checked based on selected channel
	QBBC            ENDAT_SKIP34_PRE_CH0, ENDAT_ENABLE_CHx,	0
	LDI		R3.b0,	24
	LDI		R3.b1,	4
ENDAT_SKIP34_PRE_CH0:
	QBBC            ENDAT_SKIP34_PRE_CH1, ENDAT_ENABLE_CHx,	1
	LDI		R3.b0,	25
	LDI		R3.b1,	4 + 8
ENDAT_SKIP34_PRE_CH1:
	QBBC            ENDAT_SKIP34_PRE_CH2, ENDAT_ENABLE_CHx,	2
	LDI		R3.b0,	26
	LDI		R3.b1,	4 + 16
ENDAT_SKIP34_PRE_CH2:
   ; set bit for calculation of RT
    LDI  R27.b0,  0x1
    QBBC	ENDAT_SKIP_RT_FOR_CD2_1, ENDAT_CMDTYP_NO_SUPPLEMENT_REG, 2
    QBBS	ENDAT_RT_FOR_CD2_2, ENDAT_CMDTYP_NO_SUPPLEMENT_REG,	0
ENDAT_SKIP_RT_FOR_CD2_1:
   ; unset bit for calculation of RT
    LDI  R27.b0,  0
ENDAT_RT_FOR_CD2_2:

	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	M_OTF_RECEIVE	R15,	R16,	R19,	R20,	R23,	R24,	R27.b0,   0
	.else
	M_OTF_RECEIVE	R15,	R16,	R27.b0,   0
	.endif

	QBBC            ENDAT_SKIP35_CH0, ENDAT_ENABLE_CHx,	0
	CLR             R30.b3,	R30.b3.t0 ;  disable rx
ENDAT_SKIP35_CH0:
	QBBC            ENDAT_SKIP35_CH1, ENDAT_ENABLE_CHx,	1
	CLR             R30.b3,	R30.b3.t1 ;  disable rx
ENDAT_SKIP35_CH1:
	QBBC            ENDAT_SKIP35_CH2, ENDAT_ENABLE_CHx,	2
	CLR             R30.b3,	R30.b3.t2 ;  disable rx
ENDAT_SKIP35_CH2:

	; Update CRC status. Update i/f buffer iff CRC success, if failure increment error count
	QBBC            ENDAT_SKIP36_CH0, ENDAT_ENABLE_CHx,	0
	LBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH0_CRC_ERR_COUNTER_OFFSET,	4
	QBBC		ENDAT_SKIP36A_CH0,	R0,	0	; (R0.t0 = 0) => crc failure
	SET		SCRATCH.b0,	SCRATCH.b0,	0
	SBCO	&R15,	PRUx_DMEM,	ENDAT_CH0_POSITION_DATA_WORD0_OFFSET,	8
	SBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH0_CRC_ERR_COUNTER_OFFSET,	4
	JMP		ENDAT_SKIP36_CH0
ENDAT_SKIP36A_CH0:
	CLR		SCRATCH.b0,	SCRATCH.b0,	0
	ADD		SCRATCH.b1,	SCRATCH.b1,	1
	SBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH0_CRC_ERR_COUNTER_OFFSET,	4
ENDAT_SKIP36_CH0:
	QBBC            ENDAT_SKIP36_CH1, ENDAT_ENABLE_CHx,	1
	LBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH1_CRC_ERR_COUNTER_OFFSET,	4
	QBBC		ENDAT_SKIP36A_CH1,	R0,	0	; (R0.t0 = 0) => crc failure
	SET		SCRATCH.b0,	SCRATCH.b0,	0
	SBCO	&R15,	PRUx_DMEM,	ENDAT_CH1_POSITION_DATA_WORD0_OFFSET,	8
	SBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH1_CRC_ERR_COUNTER_OFFSET,	4
	JMP		ENDAT_SKIP36_CH1
ENDAT_SKIP36A_CH1:
	CLR		SCRATCH.b0,	SCRATCH.b0,	0
	ADD		SCRATCH.b1,	SCRATCH.b1,	1
	SBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH1_CRC_ERR_COUNTER_OFFSET,	4
ENDAT_SKIP36_CH1:
	QBBC            ENDAT_SKIP36_CH2, ENDAT_ENABLE_CHx,	2
	LBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH2_CRC_ERR_COUNTER_OFFSET,	4
	QBBC		ENDAT_SKIP36A_CH2,	R0,	0	; (R0.t0 = 0) => crc failure
	SET		SCRATCH.b0,	SCRATCH.b0,	0
	SBCO	&R15,	PRUx_DMEM,	ENDAT_CH2_POSITION_DATA_WORD0_OFFSET,	8
	SBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH2_CRC_ERR_COUNTER_OFFSET,	4
	JMP		ENDAT_SKIP36_CH2
ENDAT_SKIP36A_CH2:
	CLR		SCRATCH.b0,	SCRATCH.b0,	0
	ADD		SCRATCH.b1,	SCRATCH.b1,	1
	SBCO	&SCRATCH,	PRUx_DMEM,	ENDAT_CH2_CRC_ERR_COUNTER_OFFSET,	4
ENDAT_SKIP36_CH2:
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG1+2
	LBCO	&R0.w0,	ICSS_CFG,	SCRATCH1.w0,	2
	LSR             R0.w0, R0.w0, 2
	LOOP            RX_EN_LOOP, R0.w0
	MOV             R0, R0
RX_EN_LOOP:

	QBBC            ENDAT_SKIP37_CH0, ENDAT_ENABLE_CHx,	0
	SET             R30.b3,	R30.b3.t0 ;  enable rx
ENDAT_SKIP37_CH0:
	QBBC            ENDAT_SKIP37_CH1, ENDAT_ENABLE_CHx,	1
	SET             R30.b3,	R30.b3.t1 ;  enable rx
ENDAT_SKIP37_CH1:
	QBBC            ENDAT_SKIP37_CH2, ENDAT_ENABLE_CHx,	2
	SET             R30.b3,	R30.b3.t2 ;  enable rx
ENDAT_SKIP37_CH2:
	 LBCO    	&R0.b0,	PRUx_DMEM,	ENDAT_CH0_INTFC_CMD_TRIGGER_OFFSET,	1
        QBBS            LOOP_CONTINUOUS_MODE, R0.b0,	7

	SET		R31,	ENDAT_TX_GLOBAL_REINIT
        QBBC            ENDAT_SKIP32_CH0, ENDAT_ENABLE_CHx,	0
        CLR             R30.b3,	R30.b3.t0 ;  disable rx
WB8:
	QBBS		WB8,	R31,	5
ENDAT_SKIP32_CH0:
        QBBC            ENDAT_SKIP32_CH1, ENDAT_ENABLE_CHx,	1
        CLR             R30.b3,	R30.b3.t1 ;  disable rx
WB9:
	QBBS		WB9,	R31,	13
ENDAT_SKIP32_CH1:
        QBBC            ENDAT_SKIP32_CH2, ENDAT_ENABLE_CHx,	2
        CLR             R30.b3,	R30.b3.t2 ;  disable rx
WB10:
	QBBS		WB10,	R31,	21
ENDAT_SKIP32_CH2:

        RET

	.endif	; ENABLE_MULTI_CHANNEL


; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; R1.b0: Number of transmit bits * 8
; R2.w1: Number of receive bits
; Program rx_frame_size and tx_frame_size for clock/FIFO control
; Start tranmit on channel 0/1/2
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
FN_SEND:
	; From spec: Time out for polling Start bit (50 ms), Min Time between back to back command is 1 ms
	; Program tx_frame_size ICSS_CFG_PRUx_ENDAT_CH0_CFG0[15:11] to 30 bits
	; Program rx_frame_size in ICSS_CFG_PRUx_ENDAT_CH0_CFG0[27:16]
  .if $isdefed("ENABLE_MULTI_MAKE_RTU")  ;load rx and tx size in channel 0 for RTU PRU
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+1
	LBCO	&R2.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	;LBCO	&R2.b0,	ICSS_CFG,	ICSS_CFG_PRUx_ENDAT_CH0_CFG0+1,	1
	AND		R2.b0,	R2.b0,	0x7
	OR		R2.b0,	R2.b0, R1.b0
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+1
	SBCO	&R2,	ICSS_CFG,	SCRATCH1.w0,	1
	LBCO    &SCRATCH.w2, PRUx_DMEM,      ENDAT_CH0_CLOCK_LESS_FOR_TD,   2
	SUB             SCRATCH.w0, R2.w1, SCRATCH.w2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+2
	SBCO	&SCRATCH.w0,	ICSS_CFG,	SCRATCH1.w0,	2
  .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")    ;load rx and tx size in channel 1 for PRU
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+1
	LBCO	&R2.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	AND		R2.b0,	R2.b0,	0x7
	OR		R2.b0,	R2.b0, R1.b0
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+1
	SBCO	&R2,	ICSS_CFG,	SCRATCH1.w0,	1
	LBCO    &SCRATCH.w2, PRUx_DMEM,      ENDAT_CH1_CLOCK_LESS_FOR_TD,   2
	SUB             SCRATCH.w0, R2.w1, SCRATCH.w2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+2
	SBCO	&SCRATCH.w0,	ICSS_CFG,	SCRATCH1.w0,	2
  .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")   ;load rx and tx size in channel 2 for TXPRU
    LDI     SCRATCH1.w0,    ICSS_CFG_PRUx_ENDAT_CH2_CFG0+1
	LBCO	&R2.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	AND		R2.b0,	R2.b0,	0x7
	OR		R2.b0,	R2.b0, R1.b0
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+1
	SBCO	&R2,	ICSS_CFG,	SCRATCH1.w0,	1
	LBCO    &SCRATCH.w2, PRUx_DMEM,      ENDAT_CH2_CLOCK_LESS_FOR_TD,   2
	SUB             SCRATCH.w0, R2.w1, SCRATCH.w2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+2
	SBCO	&SCRATCH.w0,	ICSS_CFG,	SCRATCH1.w0,	2
  .else                                         ;load rx and tx size when load share mode is not enabled
    QBBC            ENDAT_SKIP15_CH0, ENDAT_ENABLE_CHx,	0
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+1
	LBCO	&R2.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	;LBCO	&R2.b0,	ICSS_CFG,	ICSS_CFG_PRUx_ENDAT_CH0_CFG0+1,	1
	AND		R2.b0,	R2.b0,	0x7
	OR		R2.b0,	R2.b0, R1.b0
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+1
	SBCO	&R2,	ICSS_CFG,	SCRATCH1.w0,	1
	LBCO    &SCRATCH.w2, PRUx_DMEM,      ENDAT_CH0_CLOCK_LESS_FOR_TD,   2
	SUB             SCRATCH.w0, R2.w1, SCRATCH.w2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+2
	SBCO	&SCRATCH.w0,	ICSS_CFG,	SCRATCH1.w0,	2
ENDAT_SKIP15_CH0:
        QBBC            ENDAT_SKIP15_CH1, ENDAT_ENABLE_CHx,	1
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+1
	LBCO	&R2.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	AND		R2.b0,	R2.b0,	0x7
	OR		R2.b0,	R2.b0, R1.b0
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+1
	SBCO	&R2,	ICSS_CFG,	SCRATCH1.w0,	1
	LBCO    &SCRATCH.w2, PRUx_DMEM,      ENDAT_CH1_CLOCK_LESS_FOR_TD,   2
	SUB             SCRATCH.w0, R2.w1, SCRATCH.w2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+2
	SBCO	&SCRATCH.w0,	ICSS_CFG,	SCRATCH1.w0,	2
ENDAT_SKIP15_CH1:
        QBBC            ENDAT_SKIP15_CH2, ENDAT_ENABLE_CHx,	2
    LDI     SCRATCH1.w0,    ICSS_CFG_PRUx_ENDAT_CH2_CFG0+1
	LBCO	&R2.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	AND		R2.b0,	R2.b0,	0x7
	OR		R2.b0,	R2.b0, R1.b0
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+1
	SBCO	&R2,	ICSS_CFG,	SCRATCH1.w0,	1
	LBCO    &SCRATCH.w2, PRUx_DMEM,      ENDAT_CH2_CLOCK_LESS_FOR_TD,   2
	SUB             SCRATCH.w0, R2.w1, SCRATCH.w2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+2
	SBCO	&SCRATCH.w0,	ICSS_CFG,	SCRATCH1.w0,	2
ENDAT_SKIP15_CH2:
  .endif
	; CLR		R30,	ENDAT_CH0_RX_EN ; Received required number of bits - disable RX
FN_SEND_WAIT_TILL_TX_CH0_BUSY:
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_TXCFG
	LBCO	&R2.b3,	ICSS_CFG,	SCRATCH1.w0,	1
	; Determines when you can assert tx go to issue a new TX frame
    .if $isdefed("ENABLE_MULTI_MAKE_RTU") ;waiting to complete tx fifo for ch0 in RTU
        QBBS	FN_SEND_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	5 ; Ch0
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;waiting to complete tx fifo for ch1 in PRU
        QBBS	FN_SEND_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	6 ; Ch1
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;waiting to complete tx fifo for ch2 in TXPRU
        QBBS	FN_SEND_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	7 ; Ch2
    .elseif	$isdefed("ENABLE_MULTI_CHANNEL")
	    AND		R2.b3,	R2.b3,	0xE0 ; TODO : Assumption is that all 3 channels are enabled
	    QBNE	FN_SEND_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	0
	.else
        QBBC            ENDAT_SKIP17_CH0, ENDAT_ENABLE_CHx,	0
	QBBS	FN_SEND_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	5 ; Ch0
ENDAT_SKIP17_CH0:
        QBBC            ENDAT_SKIP17_CH1, ENDAT_ENABLE_CHx,	1
	QBBS	FN_SEND_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	6 ; Ch1
ENDAT_SKIP17_CH1:
        QBBC            ENDAT_SKIP17_CH2, ENDAT_ENABLE_CHx,	2
	QBBS	FN_SEND_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	7 ; Ch2
ENDAT_SKIP17_CH2:
    .endif

	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	SET		R31,	ENDAT_TX_GLOBAL_GO
	.else
	SET		R31,	ENDAT_TX_CHANNEL_GO
    .endif
	RET2

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Send EnDat2.2 command supplement to encoder after receiving additional info1/2
; Block till last bit is sent
; Inputs:
; R0.b3: Number of bytes to send in command suplement block address. Its normally one byte
; post 8-bit address, 16-bit dummy parameter...
; R1.b3: Block address
; Uses: R2, SCRATCH
; TODO: Check for multi channel and channel3 support
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
FN_SEND_2_2:
	; Program tx_frame_size ICSS_CFG_PRUx_ENDAT_CH0_CFG0[15:11] to 0
	; Program rx_frame_size in ICSS_CFG_PRUx_ENDAT_CH0_CFG0[27:16] to 0
	ZERO	&SCRATCH,	4
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+1
	LBCO	&SCRATCH,	ICSS_CFG,	SCRATCH1.w0,	1
	AND	SCRATCH.b0,	SCRATCH.b0,	0x7
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+1
	SBCO	&SCRATCH,	ICSS_CFG,	SCRATCH1.w0,	3 ; Ch0
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+1
	LBCO	&SCRATCH,	ICSS_CFG,	SCRATCH1.w0,	1
	AND	SCRATCH.b0,	SCRATCH.b0,	0x7
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+1
	SBCO	&SCRATCH,	ICSS_CFG,	SCRATCH1.w0,	3 ; Ch1
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+1
	LBCO	&SCRATCH,	ICSS_CFG,	SCRATCH1.w0,	1
	AND	SCRATCH.b0,	SCRATCH.b0,	0x7
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+1
	SBCO	&SCRATCH,	ICSS_CFG,	SCRATCH1.w0,	3
    .else
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+1
	LBCO	&SCRATCH,	ICSS_CFG,	SCRATCH1.w0,	1
	AND	SCRATCH.b0,	SCRATCH.b0,	0x7
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+1
	SBCO	&SCRATCH,	ICSS_CFG,	SCRATCH1.w0,	3
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+1
	LBCO	&SCRATCH,	ICSS_CFG,	SCRATCH1.w0,	1
	AND	SCRATCH.b0,	SCRATCH.b0,	0x7
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+1
	SBCO	&SCRATCH,	ICSS_CFG,	SCRATCH1.w0,	3
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+1
	LBCO	&SCRATCH,	ICSS_CFG,	SCRATCH1.w0,	1
	AND	SCRATCH.b0,	SCRATCH.b0,	0x7
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+1
	SBCO	&SCRATCH,	ICSS_CFG,	SCRATCH1.w0,	3
	.endif
FN_SEND_2_2_WAIT_TILL_TX_CH0_BUSY:
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_TXCFG
	LBCO	&R2.b3,	ICSS_CFG,	SCRATCH1.w0,	1
	; Determines when you can assert tx go to issue a new TX frame
	.if $isdefed("ENABLE_MULTI_MAKE_RTU") ;waiting to complete tx fifo for ch0 in RTU
    QBBS	FN_SEND_2_2_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	5 ; Ch0
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;waiting to complete tx fifo for ch1 in PRU
    QBBS	FN_SEND_2_2_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	6 ; Ch1
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;waiting to complete tx fifo for ch2 in TXPRU
    QBBS	FN_SEND_2_2_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	7 ; Ch2
    .elseif	$isdefed("ENABLE_MULTI_CHANNEL")
	AND		R2.b3,	R2.b3,	0xE0
	QBNE	FN_SEND_2_2_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	0
	.else
        QBBC            ENDAT_SKIP18_CH0, ENDAT_ENABLE_CHx,	0
	QBBS	FN_SEND_2_2_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	5 ; Ch0
ENDAT_SKIP18_CH0:
        QBBC            ENDAT_SKIP18_CH1, ENDAT_ENABLE_CHx,	1
	QBBS	FN_SEND_2_2_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	6 ; Ch1
ENDAT_SKIP18_CH1:
        QBBC            ENDAT_SKIP18_CH2, ENDAT_ENABLE_CHx,	2
	QBBS	FN_SEND_2_2_WAIT_TILL_TX_CH0_BUSY,	R2.b3,	7 ; Ch2
ENDAT_SKIP18_CH2:
	.endif

	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	SET		R31,	ENDAT_TX_GLOBAL_GO
	.else
	SET		R31,	ENDAT_TX_CHANNEL_GO
	.endif

	; Check whether last bit is sent out

	.if $isdefed("ENABLE_MULTI_MAKE_RTU")
;waiting for Tx fifo complete
WB_RTU_17:
    AND R28.b0, R31.b0, 0x1C
    QBNE   WB_RTU_17, R28.b0, 0
;waiting for Rx start first time
W_RX_RTU_CH0:
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0
	LBCO	&R28,	ICSS_CFG,	SCRATCH1.w0,	4
	QBBS		W_RX_RTU_CH0,	R28,	28
    ;RT claculation
    M_CALC_RECOV_TIME_CH0
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
;waiting for Tx fifo complete
WB_PRU_17:
    AND R28.b0, R31.b1, 0x1C
    QBNE   WB_PRU_17, R28.b0, 0
;waiting for Rx start first time
W_RX_PRU_CH1:
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0
	LBCO	&R28,	ICSS_CFG,	SCRATCH1.w0,	4
	QBBS		W_RX_PRU_CH1,	R28,	28
    ;RT Calculation
    M_CALC_RECOV_TIME_CH1
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
;waiting for Tx fifo complete
WB_TXPRU_17:
    AND R28.b0, R31.b2, 0x1C
    QBNE   WB_TXPRU_17, R28.b0, 0
;waiting for Rx start first time
W_RX_TXPRU_CH2:
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0
	LBCO	&R28,	ICSS_CFG,	SCRATCH1.w0,	4
	QBBS		W_RX_TXPRU_CH2,	R28,	28
    ;RT Calculation
    M_CALC_RECOV_TIME_CH2
    .elseif	$isdefed("ENABLE_MULTI_CHANNEL")
    ;waiting for Tx fifo complete
    ZERO &R28, 4
WB_TX_FIFO_COM_MULTI_CH:
    QBBC            ENDAT_SKIP100_MULTI_CH0, ENDAT_ENABLE_CHx_IN_USE,	0
    AND R28.b0, R31.b0, 0x1C
ENDAT_SKIP100_MULTI_CH0:
    QBBC            ENDAT_SKIP100_MULTI_CH1, ENDAT_ENABLE_CHx_IN_USE,	1
    AND R28.b1, R31.b1, 0x1C
ENDAT_SKIP100_MULTI_CH1:
    QBBC            ENDAT_SKIP100_MULTI_CH2, ENDAT_ENABLE_CHx_IN_USE,	2
    AND R28.b2, R31.b2, 0x1C
ENDAT_SKIP100_MULTI_CH2:
    QBNE   WB_TX_FIFO_COM_MULTI_CH, R28, 0

 ; waiting for TX complete for all three channels
    ZERO &R28, 4
WB_TX_COM_MULTI_CH:
    QBBC            ENDAT_SKIP101_MULTI_CH0, ENDAT_ENABLE_CHx_IN_USE,	0
    AND R28.b0, R31.b0, 0x20
ENDAT_SKIP101_MULTI_CH0:
    QBBC            ENDAT_SKIP101_MULTI_CH1, ENDAT_ENABLE_CHx_IN_USE,	1
    AND R28.b1, R31.b1, 0x20
ENDAT_SKIP101_MULTI_CH1:
    QBBC            ENDAT_SKIP101_MULTI_CH2, ENDAT_ENABLE_CHx_IN_USE,	2
    AND R28.b2, R31.b2, 0x20
ENDAT_SKIP101_MULTI_CH2:
    QBNE   WB_TX_COM_MULTI_CH, R28, 0
    M_CALC_RECOV_TIME_MULTI_CH
	.else
    QBBC            ENDAT_SKIP19_CH0, ENDAT_ENABLE_CHx,	0
;waiting for Tx fifo complete
WB_TX_FIFO_COM_CH0:
    AND R28.b0, R31.b0, 0x1C
    QBNE   WB_TX_FIFO_COM_CH0, R28.b0, 0
;waiting for Rx start first time
W_RX_LOW_CH0:
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0
	LBCO	&R28,	ICSS_CFG,	SCRATCH1.w0,	4
	QBBS		W_RX_LOW_CH0,	R28,	28
    M_CALC_RECOV_TIME_CH0

ENDAT_SKIP19_CH0:
	QBBC            ENDAT_SKIP19_CH1, ENDAT_ENABLE_CHx,	1
;waiting for Tx fifo complete
WB_TX_FIFO_COM_CH1:
    AND R28.b0, R31.b1, 0x1C
    QBNE   WB_TX_FIFO_COM_CH1, R28.b0, 0
;waiting for Rx start first time
W_RX_LOW_CH1:
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0
	LBCO	&R28,	ICSS_CFG,	SCRATCH1.w0,	4
	QBBS		W_RX_LOW_CH1,	R28,	28
    M_CALC_RECOV_TIME_CH1

ENDAT_SKIP19_CH1:
    QBBC            ENDAT_SKIP19_CH2, ENDAT_ENABLE_CHx,	2
;waiting for Tx fifo complete
WB_TX_FIFO_COM_CH2:
    AND R28.b0, R31.b2, 0x1C
    QBNE   WB_TX_FIFO_COM_CH2, R28.b0, 0
;waiting for Rx start first time
W_RX_LOW_CH2:
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0
	LBCO	&R28,	ICSS_CFG,	SCRATCH1.w0,	4
	QBBS		W_RX_LOW_CH2,	R28,	28
    M_CALC_RECOV_TIME_CH2
ENDAT_SKIP19_CH2:
  .endif
	RET2

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Function to send EnDat2.2 command supplement
; R1.b0: address/MRS code/port address
; R1.w1:16-bit parameter data
; R1.b3: Block address - used by FN_SEND_2_2
; R0.b3: Length of block address in bytes - used by FN_SEND_2_2. Force this to 1 from firmware now
; Section 2.4.2 EnDat spec: Encoder to send position value and selection of memory area
; or block address.Command supplement in this case 8-bit MRS code, 16-bit parameter(zero),
; 8-bit block address
; Uses: R30
; Invokes: FN_SEND_2_2  - TODO: check whether both can be merged
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
FN_SEND_ENDAT22_COMMAND_SUPPLEMENT:
	; Send zero pad bits+start_bit+MRS_code+16bit Data (Low/zero)+8bit(block address)
	.if $isdefed("ENABLE_MULTI_MAKE_RTU") ;set clock high after txfor ch0 in RTU
    LDI		R30.w2,	(ENDAT_TX_CLK_MODE_STOPHIGH_AFTER_TX | ENDAT_TX_CH0_SEL)
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;set clock  high after tx for ch1 in PRU
    LDI		R30.w2,	(ENDAT_TX_CLK_MODE_STOPHIGH_AFTER_TX | ENDAT_TX_CH1_SEL)
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;set clock  high after tx for ch1 in TXPRU
    LDI		R30.w2,	(ENDAT_TX_CLK_MODE_STOPHIGH_AFTER_TX | ENDAT_TX_CH2_SEL)
   .elseif	$isdefed("ENABLE_MULTI_CHANNEL")
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_STOPHIGH_AFTER_TX | ENDAT_TX_CH0_SEL)
	LOOP	FN_SEND_ENDAT22_MULTI_CHANNEL,	3 ; TODO: assumption all 3 channels are enabled in multi channel mode
	.else
        QBBC            ENDAT_SKIP24_CH0, ENDAT_ENABLE_CHx,	0
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_STOPHIGH_AFTER_TX | ENDAT_TX_CH0_SEL)
ENDAT_SKIP24_CH0:
        QBBC            ENDAT_SKIP24_CH1, ENDAT_ENABLE_CHx,	1
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_STOPHIGH_AFTER_TX | ENDAT_TX_CH1_SEL)
ENDAT_SKIP24_CH1:
        QBBC            ENDAT_SKIP24_CH2, ENDAT_ENABLE_CHx,	2
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_STOPHIGH_AFTER_TX | ENDAT_TX_CH2_SEL)
ENDAT_SKIP24_CH2:
	.endif


	LDI		R30.b0,	1 ; Simplicity 7 clocks till start bit - can optimize later
	MOV		R30.b0,	R1.b0
	MOV		R30.b0,	R1.b1 ; Data15..8 Low
	MOV		R30.b0,	R1.b2 ; Data7..0 Low
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	ADD		R30.b2,	R30.b2,	1
FN_SEND_ENDAT22_MULTI_CHANNEL:
	.endif;ENABLE_MULTI_CHANNEL

	; FIFO is full at this point, now monitor the FIFO level and send the remaining bytes
	CALL2		FN_SEND_2_2
	RET

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Function to send and receive EnDat2.1/2.2 command/response
; Inputs:
; R0.w0: Command mode(6)+9 bits
; R0.w2: Remaining 16 bits
; R1.b0: Reecive bits
; R1.b1: Transmit bits
; R1.b2: 0: EnDat2.2 otherwise: EnDat2.1 or ENDAT_CMD_SEND_POSVAL_WITH_DATA
; R1.b3: For EnDat2.2, Block address length in bytes for command supplement
; Uses: R2, R30
; Invokes: FN_SEND, FN_RECEIVE
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
FN_SEND_RECEIVE_ENDAT:
    .if $isdefed("ENABLE_MULTI_MAKE_RTU") ;set clock low or high for ch0 in RTU
    QBBC	        FN_SEND_RECEIVE_RTU_ENDAT22,        ENDAT_CMDTYP_NO_SUPPLEMENT_REG,	0
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN_STOPHIGH | ENDAT_TX_CH0_SEL)
	JMP		FN_SEND_START
FN_SEND_RECEIVE_RTU_ENDAT22:
    LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN_STOPLOW | ENDAT_TX_CH0_SEL)
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;set clock low or high for ch1 in PRU
    QBBC	        FN_SEND_RECEIVE_PRU_ENDAT22,        ENDAT_CMDTYP_NO_SUPPLEMENT_REG,	0
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN_STOPHIGH | ENDAT_TX_CH1_SEL)
	JMP		FN_SEND_START
FN_SEND_RECEIVE_PRU_ENDAT22:
    LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN_STOPLOW | ENDAT_TX_CH1_SEL)
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;set clock low or high for ch1 in TXPRU
   QBBC	        FN_SEND_RECEIVE_TXPRU_ENDAT22,        ENDAT_CMDTYP_NO_SUPPLEMENT_REG,	0
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN_STOPHIGH | ENDAT_TX_CH2_SEL)
	JMP		FN_SEND_START
FN_SEND_RECEIVE_TXPRU_ENDAT22:
    LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN_STOPLOW | ENDAT_TX_CH2_SEL)
   .else                                              ;clock set When load share mode is not used, that is single channel or multi channel using single PRU
	QBBC	        FN_SEND_RECEIVE_ENDAT22,        ENDAT_CMDTYP_NO_SUPPLEMENT_REG,	0
        QBBC            ENDAT_SKIP25_CH0, ENDAT_ENABLE_CHx,	0
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN_STOPHIGH | ENDAT_TX_CH0_SEL)
ENDAT_SKIP25_CH0:
        QBBC            ENDAT_SKIP25_CH1, ENDAT_ENABLE_CHx,	1
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN_STOPHIGH | ENDAT_TX_CH1_SEL)
ENDAT_SKIP25_CH1:
        QBBC            ENDAT_SKIP25_CH2, ENDAT_ENABLE_CHx,	2
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN_STOPHIGH | ENDAT_TX_CH2_SEL)
ENDAT_SKIP25_CH2:
	JMP		FN_SEND_START
FN_SEND_RECEIVE_ENDAT22:
        QBBC            ENDAT_SKIP26_CH0, ENDAT_ENABLE_CHx,	0
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN_STOPLOW | ENDAT_TX_CH0_SEL)
ENDAT_SKIP26_CH0:
        QBBC            ENDAT_SKIP26_CH1, ENDAT_ENABLE_CHx,	1
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN_STOPLOW | ENDAT_TX_CH1_SEL)
ENDAT_SKIP26_CH1:
        QBBC            ENDAT_SKIP26_CH2, ENDAT_ENABLE_CHx,	2
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN_STOPLOW | ENDAT_TX_CH2_SEL)
ENDAT_SKIP26_CH2:
   .endif

FN_SEND_START:
    .if	$isdefed("ENABLE_MULTI_CHANNEL")
	LOOP	FN_SEND_RECEIVE_ENDAT_MULTI_CHANNEL,	3
	.endif
	MOV		R30.b0,	R0.b0
	QBGE	FN_SEND_TX_DONE,	R1.b1,	8
	MOV		R30.b0,	R0.b1
	MOV		R30.b0,	R0.b2
	MOV		R30.b0,	R0.b3
FN_SEND_TX_DONE:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	; since ENABLE_RUNTIME_CH is kept enabled with all 3 channels selected for multi channel, last channel would have been selected by now
	SUB		R30.b2,	R30.b2,	1
FN_SEND_RECEIVE_ENDAT_MULTI_CHANNEL:
	.endif
	MOV		R2.w1,	R1.b0
	LSL		R1.b0,	R1.b1,	3
	CALL2		FN_SEND


 ;set syn_bit of all connected channels for TX_GLOBAL_INIT

    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LDI R14.b0, 0x1 ; set syn_bit  of ch0
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LDI R14.b0, 0x2 ;set syn_bit of ch1
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LDI R14.b0, 0x4 ;set syn_bit of ch2
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
    .endif

	ZERO	        &R15, 4
	ZERO	        &R16, 4
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	ZERO	        &R19, 4
	ZERO	        &R20, 4
	ZERO	        &R23, 4
	ZERO	        &R24, 4
	.endif


	; store valid bit & rx fifo bit to be checked based on selected channel
	.if $isdefed("ENABLE_MULTI_MAKE_RTU") ;store valid bit & rx fifo bit to be checked for ch0 in PRU
	  LDI		R3.b0,	24
	  LDI		R3.b1,	4
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;store valid bit & rx fifo bit to be checked for  ch1 in RTU
	  LDI		R3.b0,	25
	  LDI		R3.b1,	4 + 8
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;store valid bit & rx fifo bit to be checked for  CH2 in TXPRU
	 LDI		R3.b0,	26
	 LDI		R3.b1,	4 + 16
	.else                                    ;without loadshare mode
	QBBC            ENDAT_SKIP41_CH0, ENDAT_ENABLE_CHx,	0
	LDI		R3.b0,	24
	LDI		R3.b1,	4
ENDAT_SKIP41_CH0:
	QBBC            ENDAT_SKIP41_CH1, ENDAT_ENABLE_CHx,	1
	LDI		R3.b0,	25
	LDI		R3.b1,	4 + 8
ENDAT_SKIP41_CH1:
	QBBC            ENDAT_SKIP41_CH2, ENDAT_ENABLE_CHx,	2
	LDI		R3.b0,	26
	LDI		R3.b1,	4 + 16
ENDAT_SKIP41_CH2:
    .endif
    LDI R27.b0,  0x1
	AND		R2.b0,	R1.b2,	(0x3 << 3)

	QBEQ		ENDAT_START_OTF,	R2.b0,	0
	SUB		R2.w1,	R2.w1,	30
    ;RT after additional info 1
    LDI  R27.b0,  0x2
	QBNE		ENDAT_START_OTF,	R2.b0,	(0x3 << 3)
	SUB		R2.w1,	R2.w1,	30
    ;RT after additional info 2
    LDI  R27.b0,   0x4
ENDAT_START_OTF:
   ;If it is 2.1 command  then skip RT calculation
    QBBC	ENDAT_SKIP_RT_FOR_CMD2_1, ENDAT_CMDTYP_NO_SUPPLEMENT_REG,	2
    QBBS	ENDAT_RT_FOR_NSP_CMD2_2, ENDAT_CMDTYP_NO_SUPPLEMENT_REG,	0
    ;Skip RT calculation for supplement command
ENDAT_SKIP_RT_FOR_CMD2_1:
    LDI  R27.b0,  0
ENDAT_RT_FOR_NSP_CMD2_2:
    LDI		R0.b0,	0
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	LDI		R0.b1,	0
	LDI		R0.b2,	0
	.endif
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	M_OTF_RECEIVE	R15,	R16,	R19,	R20,	R23,	R24,	R27.b0,  0
	.else
	M_OTF_RECEIVE	R15,	R16,	R27.b0,  0
	.endif

	QBEQ		ENDAT_END_OF_RX,	R2.b0,	0
	LDI		R2.w1,	29

	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	M_OTF_RECEIVE	R17,	SCRATCH,	R21,	SCRATCH,	R25,	SCRATCH,	R27.b0,   1
	.else
	M_OTF_RECEIVE	R17,	SCRATCH,	R27.b0,    1

	.endif
	QBNE		ENDAT_END_OF_RX,	R2.b0,	(0x3 << 3)

	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	M_OTF_RECEIVE	R18,	SCRATCH,	R22,	SCRATCH,	R26,	SCRATCH,   R27.b0,	 2
	.else
	M_OTF_RECEIVE	R18,	SCRATCH,	R27.b0,   2
	.endif

ENDAT_END_OF_RX:
	QBBC            SKIP_FOR_ENDAT_2_2,     ENDAT_CMDTYP_NO_SUPPLEMENT_REG,	0
	.if $isdefed("ENABLE_MULTI_MAKE_RTU")
	 LDI  R14.b0 , 0 ;clear Syn_bit of ch0
	 SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
     M_WAIT_FOR_ENABLED_CHANNELS
     LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
     QBBC  SKIP_GLOBAL_TX_REINIT1, R14.b2, 0  ; check RTU is  primary core
	 SET		R31,	ENDAT_TX_GLOBAL_REINIT
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	 LDI R14.b0, 0  ;clear syn_bit of ch1
	 SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
     M_WAIT_FOR_ENABLED_CHANNELS
     LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
     QBBC  SKIP_GLOBAL_TX_REINIT1, R14.b2, 1   ; check PRU is  primary core
	 SET		R31,	ENDAT_TX_GLOBAL_REINIT
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	 LDI R14.b0, 0 ;clear syn_bit of ch2
	 SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
     M_WAIT_FOR_ENABLED_CHANNELS
     LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
     QBBC  SKIP_GLOBAL_TX_REINIT1, R14.b2, 2; check TXPRU is  primary core
	 SET		R31,	ENDAT_TX_GLOBAL_REINIT

	.else  ; for without load share mode
     SET		R31,	ENDAT_TX_GLOBAL_REINIT
    .endif

SKIP_GLOBAL_TX_REINIT1:

	.if $isdefed("ENABLE_MULTI_MAKE_RTU") ; ch0 wait to complete TX_GLOBAL_INIT action
WBRTU28:
	QBBS		WBRTU28,	R31,	5
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;ch1 wait to complete TX_GLOBAL_INIT action
WBPRU29:
	QBBS		WBPRU29,	R31,	13
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU");ch2 ; wait to complete TX_GLOBAL_INIT action
WBTXPRU30:
	QBBS		WBTXPRU30,	R31,	21
    .else
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
        QBBC            ENDAT_SKIP23_CH0, ENDAT_ENABLE_CHx_IN_USE,	0
	.else
        QBBC            ENDAT_SKIP23_CH0, ENDAT_ENABLE_CHx,	0
	.endif
WB28:
	QBBS		WB28,	R31,	5
ENDAT_SKIP23_CH0:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
        QBBC            ENDAT_SKIP23_CH1, ENDAT_ENABLE_CHx_IN_USE,	1
	.else
        QBBC            ENDAT_SKIP23_CH1, ENDAT_ENABLE_CHx,	1
	.endif
WB29:
	QBBS		WB29,	R31,	13
ENDAT_SKIP23_CH1:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
        QBBC            ENDAT_SKIP23_CH2, ENDAT_ENABLE_CHx_IN_USE,	2
	.else
        QBBC            ENDAT_SKIP23_CH2, ENDAT_ENABLE_CHx,	2
	.endif
WB30:
	QBBS		WB30,	R31,	21

ENDAT_SKIP23_CH2:
  .endif

SKIP_FOR_ENDAT_2_2:

	RET


; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Args:
;	FN_DELAY_CYCLES: R0 - number of cycles to delay
; Uses: R1
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
FN_DELAY_CYCLES:
	; clear the PRU cycle counter by writing 0x0 to it
	 ZERO 	&R2, 4
    .if $isdefed("ENABLE_MULTI_MAKE_TXPRU") ; c11 is not correct for TXPRU
    SBCO	&R2, c28, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
    .else
	SBCO	&R2, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
    .endif

DELAY_LOOP:
    .if $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LBCO 	&R2, c28, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
    .else
	LBCO 	&R2, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
	.endif
	QBLT	DELAY_LOOP, R0, R2

	RET2

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; EnDat power on init procedure
; Start Power on procedure to determine EnDAT/SSI interface - ENDAT 2.2 spec(D297403) - Appendix A5
; Uses R0, R1
; USE R14 : in synchronization part
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
FN_ENDAT_POWER_ON_INIT:
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LDI R14.b0, 0 ; clear syn_bit of ch0
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
    M_WAIT_FOR_ENABLED_CHANNELS ; wait till syn_bit  not clear for all three channels
    LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
    QBBC  SKIP_GLOBAL_TX_REINIT2, R14.b2, 0  ; check RTUPRU is  primary core
	SET		R31,	ENDAT_TX_GLOBAL_REINIT
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LDI R14.b0, 0  ; clear syn_bit of ch1
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
    M_WAIT_FOR_ENABLED_CHANNELS ; wait till syn_bit  not clear  for all three channels
    LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
    QBBC  SKIP_GLOBAL_TX_REINIT2, R14.b2, 1  ;check PRU is  primary core
	SET		R31,	ENDAT_TX_GLOBAL_REINIT
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LDI R14.b0, 0 ; clear syn_bit of ch2
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
    M_WAIT_FOR_ENABLED_CHANNELS ; wait till syn_bit not clear for all three channels
    LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
    QBBC  SKIP_GLOBAL_TX_REINIT2, R14.b2, 2  ; check TXPRU is  primary core
	SET		R31,	ENDAT_TX_GLOBAL_REINIT
    .else    ;without load share
    SET	R31,	ENDAT_TX_GLOBAL_REINIT ; Set TX_EN low
    .endif

SKIP_GLOBAL_TX_REINIT2:

     .if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LDI R14.b0, 0x1 ; set syn_bit ch0
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LDI R14.b0, 0x2 ; set syn_bit of ch1
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LDI R14.b0, 0x4 ; set syn_bit of ch2
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
    .endif

	.if $isdefed("ENABLE_MULTI_MAKE_RTU")  ;waiting to complete tx_globle action for ch0 in RTU
WBRTU33:
	QBBS		WBRTU33,	R31,	5
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")  ;waiting to complete tx_globle action for ch1  in PRU
WBPRU34:
	QBBS		WBPRU34,	R31,	13
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")  ;;waiting to complete tx_globle action for ch2 in TXXPRU
WBTXPRU35:
	QBBS		WBTXPRU35,	R31,	21
    .else
    .if	$isdefed("ENABLE_MULTI_CHANNEL")
        QBBC            ENDAT_SKIP27_CH0, ENDAT_ENABLE_CHx_IN_USE,	0
	.else
	 QBBC            ENDAT_SKIP27_CH0, ENDAT_ENABLE_CHx,	0
	.endif
WB33:
	QBBS	WB33,	R31,	5
ENDAT_SKIP27_CH0:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
        QBBC            ENDAT_SKIP27_CH1, ENDAT_ENABLE_CHx_IN_USE,	1
	.else
        QBBC            ENDAT_SKIP27_CH1, ENDAT_ENABLE_CHx,	1
	.endif
WB34:
	QBBS	WB34,	R31,	13
ENDAT_SKIP27_CH1:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
        QBBC            ENDAT_SKIP27_CH2, ENDAT_ENABLE_CHx_IN_USE,	2
	.else
        QBBC            ENDAT_SKIP27_CH2, ENDAT_ENABLE_CHx,	2
	.endif
WB35:
	QBBS	WB35,	R31,	21
ENDAT_SKIP27_CH2:
    .endif

	; Use ICSS_CFG_PRUx_ENDAT_CH0_CFG0 to program CLK overide and do power on init sequence
	; t1:TX_CLK HIGH or LOW for at least 800ms
	; t2: 80ms < TX_CLK HIGH < 120ms
	; TX_CLK LOW > 125ns
	; t3: 380ms < TX_CLK HIGH < 420 ms
	; Poll for encoder data after above sequence

	; Keep TX_CLK HIGH for 900ms

	LDI		R0.b0,	0x60 ;  Set endat_clk_out_override_en and pru<n>_ endat<m>_clk
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .else
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .endif



    LBCO      &R0, PRUx_DMEM,	ENDAT_CONFIG_DELAY_900MS_OFFSET, 4
    CALL2 FN_DELAY_CYCLES ;900ms wait

	; Keep TX_CLK LOW for > 125ns
	LDI		R0.b0,	0x20
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .else
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .endif

    LBCO      &R0, PRUx_DMEM,	ENDAT_CONFIG_DELAY_125NS_OFFSET, 4
    CALL2 FN_DELAY_CYCLES ;wait 125ns

	LDI		R0.b0,	0x60 ;  Set endat_clk_out_override_en and pru<n>_ endat<m>_clk
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .else
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .endif

	;wait 380ms
    LBCO      &R0, PRUx_DMEM,	ENDAT_CONFIG_DELAY_380MS_OFFSET, 4
    CALL2 FN_DELAY_CYCLES

	LDI		R0.b0,	0 ;  Disable clock override
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
    LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
    .else
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG0+3
	SBCO	&R0.b0,	ICSS_CFG,	SCRATCH1.w0,	1
   .endif


     .if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LDI R14.b0, 0 ; clear  syn_bit of ch0
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
    M_WAIT_FOR_ENABLED_CHANNELS ; wait till syn_bit bit not clear for all channels
    LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
    QBBC  SKIP_GLOBAL_TX_REINIT3, R14.b2, 0  ; check if RTUPRU is  primary core
	SET		R31,	ENDAT_TX_GLOBAL_REINIT
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LDI R14.b0, 0 ; clear  syn_bit  of ch1
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
    M_WAIT_FOR_ENABLED_CHANNELS ; wait till syn_bit not clear for all channels
     LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
    QBBC  SKIP_GLOBAL_TX_REINIT3, R14.b2, 1  ; check if PRU is  primary core
	SET		R31,	ENDAT_TX_GLOBAL_REINIT
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LDI R14.b0, 0 ;clear syn_bit for ch2
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
    M_WAIT_FOR_ENABLED_CHANNELS ; wait till syn_bit  not clear for all channels
    LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
    QBBC  SKIP_GLOBAL_TX_REINIT3, R14.b2, 2 ; check if TXPRU is  primary core
	SET		R31,	ENDAT_TX_GLOBAL_REINIT
    .else
    SET	R31,	ENDAT_TX_GLOBAL_REINIT ; Set TX_EN low
    .endif

SKIP_GLOBAL_TX_REINIT3:

    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LDI R14.b0, 0x1 ;set syn_bit for ch0
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LDI R14.b0, 0x2 ; set syn_bit for ch1
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LDI R14.b0, 0x4 ; set syn_bit for ch2
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
    .endif

    .if $isdefed("ENABLE_MULTI_MAKE_RTU")  ;waiting to complete tx_globle for ch0 in RTU
WBRTU39:
	QBBS		WBRTU39,	R31,	5
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")  ;waiting to complete tx_globle for ch1 in PRU
WBPRU40:
	QBBS		WBPRU40,	R31,	13
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")  ;;waiting to complete tx-globle for ch2 in TXXPRU
WBTXPRU41:
	QBBS		WBTXPRU41,	R31,	21
    .else
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
        QBBC            ENDAT_SKIP28_CH0, ENDAT_ENABLE_CHx_IN_USE,	0
	.else
        QBBC            ENDAT_SKIP28_CH0, ENDAT_ENABLE_CHx,	0
	.endif
WB39:
	QBBS		WB39,	R31,	5
ENDAT_SKIP28_CH0:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
        QBBC            ENDAT_SKIP28_CH1, ENDAT_ENABLE_CHx_IN_USE,	1
	.else
        QBBC            ENDAT_SKIP28_CH1, ENDAT_ENABLE_CHx,	1
	.endif
WB41:
	QBBS		WB41,	R31,	13
ENDAT_SKIP28_CH1:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
        QBBC            ENDAT_SKIP28_CH2, ENDAT_ENABLE_CHx_IN_USE,	2
	.else
        QBBC            ENDAT_SKIP28_CH2, ENDAT_ENABLE_CHx,	2
	.endif
WB42:
	QBBS		WB42,	R31,	21
ENDAT_SKIP28_CH2:
    .endif

	; Start free running clock and poll for RX_VALID
	; Set continuous clock for channel0
	.if $isdefed("ENABLE_MULTI_MAKE_RTU") ; Start free running clock and poll for RX_VALID for ch0
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN | ENDAT_TX_CH0_SEL)
	; Send dummy 8-bits
	LDI		R30.b0,	0
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;Start free running clock and poll for RX_VALID for ch1
    LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN | ENDAT_TX_CH1_SEL)
	; Send dummy 8-bits
	LDI		R30.b0,	0
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;Start free running clock and poll for RX_VALID for ch2
    LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN | ENDAT_TX_CH2_SEL)
	; Send dummy 8-bits
	LDI		R30.b0,	0
    .else
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN | ENDAT_TX_CH0_SEL)
	; Send dummy 8-bits
	LDI		R30.b0,	0
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN | ENDAT_TX_CH1_SEL)
	; Send dummy 8-bits
	LDI		R30.b0,	0
	LDI		R30.w2,	(ENDAT_TX_CLK_MODE_FREERUN | ENDAT_TX_CH2_SEL)
	; Send dummy 8-bits
	LDI		R30.b0,	0
	.endif

WAIT_TILL_TX_CH0_1_2_BUSY:
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_TXCFG
	LBCO	&R2.b3,	ICSS_CFG,	SCRATCH1.w0,	1
    .if $isdefed("ENABLE_MULTI_MAKE_RTU") ;waiting to complete tx fifo for ch0 in RTU
    QBBS	WAIT_TILL_TX_CH0_1_2_BUSY,	R2.b3,	5 ; Ch0
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;waiting to complete tx fifo for ch1 in PRU
    QBBS	WAIT_TILL_TX_CH0_1_2_BUSY,	R2.b3,	6 ; Ch1
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;waiting to complete tx fifo for ch2 in TXPRU
    QBBS	WAIT_TILL_TX_CH0_1_2_BUSY,	R2.b3,	7 ; Ch2
    .else
	AND		R2.b3,	R2.b3,	0xE0
	QBNE	WAIT_TILL_TX_CH0_1_2_BUSY,	R2.b3,	0
    .endif

	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	SET		R31,	ENDAT_TX_GLOBAL_GO
	.else
	SET		R31,	ENDAT_TX_CHANNEL_GO ; NOTE: Need to send at least 1-bit to use h/w clock modes
	.endif
	; Poll for RX_VALID to complete Power On Sequence
; WB46:
	; QBBC		WB46,	R31,	24
	.if $isdefed("ENABLE_MULTI_MAKE_RTU") ;ch0
	SET		R31,	ENDAT_CH0_RX_CLR_VALID
	QBBC	SKIP_OVFG_RTU_CLR0,	R31,	27
	SET		R31,	ENDAT_CH0_RX_CLR_OVF
SKIP_OVFG_RTU_CLR0:
	CLR		R30,	ENDAT_CH0_RX_EN
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ;ch1
    SET		R31,	ENDAT_CH1_RX_CLR_VALID
	QBBC	SKIP_OVFG_PRU_CLR1,	R31,	28
	SET		R31,	ENDAT_CH1_RX_CLR_OVF

SKIP_OVFG_PRU_CLR1:
	CLR		R30,	ENDAT_CH1_RX_EN
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ;ch2
    SET		R31,	ENDAT_CH2_RX_CLR_VALID
	QBBC	SKIP_OVFG_TXPRU_CLR2,	R31,	29
	SET		R31,	ENDAT_CH2_RX_CLR_OVF
SKIP_OVFG_TXPRU_CLR2:
	CLR		R30,	ENDAT_CH2_RX_EN
    .else          ;without loadshare mode
        QBBC            ENDAT_SKIP46_CH0, ENDAT_ENABLE_CHx,	0
	SET		R31,	ENDAT_CH0_RX_CLR_VALID
	QBBC	SKIP_OVFG_CLR0,	R31,	27
	SET		R31,	ENDAT_CH0_RX_CLR_OVF
SKIP_OVFG_CLR0:
	CLR		R30,	ENDAT_CH0_RX_EN
ENDAT_SKIP46_CH0:

        QBBC            ENDAT_SKIP46_CH1, ENDAT_ENABLE_CHx,	1
	SET		R31,	ENDAT_CH1_RX_CLR_VALID
	QBBC	SKIP_OVFG_CLR1,	R31,	28
	SET		R31,	ENDAT_CH1_RX_CLR_OVF
SKIP_OVFG_CLR1:
	CLR		R30,	ENDAT_CH1_RX_EN
ENDAT_SKIP46_CH1:

        QBBC            ENDAT_SKIP46_CH2, ENDAT_ENABLE_CHx,	2
	SET		R31,	ENDAT_CH2_RX_CLR_VALID
	QBBC	SKIP_OVFG_CLR2,	R31,	29
	SET		R31,	ENDAT_CH2_RX_CLR_OVF
SKIP_OVFG_CLR2:
	CLR		R30,	ENDAT_CH2_RX_EN
ENDAT_SKIP46_CH2:
    .endif


	.if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LDI R14.b0, 0 ;clear  syn_bit  for ch0
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
    M_WAIT_FOR_ENABLED_CHANNELS ; wait till syn_bit  not clear for all channels
    LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
    QBBC  SKIP_GLOBAL_TX_REINIT4, R14.b2, 0 ; check if RTU is  primary core
	SET		R31,	ENDAT_TX_GLOBAL_REINIT
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LDI R14.b0, 0 ;clear syn_bit  for ch1
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
    M_WAIT_FOR_ENABLED_CHANNELS ; wait till syn_bit  not clear for all channels
    LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
    QBBC  SKIP_GLOBAL_TX_REINIT4, R14.b2, 1 ; check if PRU is  primary core
	SET		R31,	ENDAT_TX_GLOBAL_REINIT
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LDI R14.b0, 0 ;clear syn_bit for ch2
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
    M_WAIT_FOR_ENABLED_CHANNELS ; wait till syn_bit not clear for all channels
    LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
    QBBC  SKIP_GLOBAL_TX_REINIT4, R14.b2, 2 ; check if TXPRU is  primary core
	SET		R31,	ENDAT_TX_GLOBAL_REINIT
    .else  ; without load share
    SET	R31,	ENDAT_TX_GLOBAL_REINIT ; Set TX_EN low
    .endif
SKIP_GLOBAL_TX_REINIT4:

    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LDI R14.b0, 0x1 ;set syn_bit  for ch0
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LDI R14.b0, 0x2 ;set syn_bit for ch1
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LDI R14.b0, 0x4 ;set syn_bit for ch2
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
    .endif

    .if $isdefed("ENABLE_MULTI_MAKE_RTU")  ;waiting to complete tx in RTU
WB47RTU_A:
	QBBS		WB47RTU_A,	R31,	5
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")  ;waiting to complete tx in PRU
WB47PRU_B:
	QBBS		WB47PRU_B,	R31,	13
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")  ;waiting to complete tx in TXXPRU
WB47TXPRU_C:
	QBBS		WB47TXPRU_C,	R31,	21
    .else
    .if	$isdefed("ENABLE_MULTI_CHANNEL")
        QBBC            ENDAT_SKIP47_CH0, ENDAT_ENABLE_CHx_IN_USE,	0
	.else
        QBBC            ENDAT_SKIP47_CH0, ENDAT_ENABLE_CHx,	0
	.endif
WB47_A:
	QBBS		WB47_A,	R31,	5
ENDAT_SKIP47_CH0:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
        QBBC            ENDAT_SKIP47_CH1, ENDAT_ENABLE_CHx_IN_USE,	1
	.else
        QBBC            ENDAT_SKIP47_CH1, ENDAT_ENABLE_CHx,	1
	.endif
WB47_B:
	QBBS		WB47_B,	R31,	13
ENDAT_SKIP47_CH1:
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
        QBBC            ENDAT_SKIP47_CH2, ENDAT_ENABLE_CHx_IN_USE,	2
	.else
        QBBC            ENDAT_SKIP47_CH2, ENDAT_ENABLE_CHx,	2
	.endif
WB47_C:
	QBBS		WB47_C,	R31,	21
ENDAT_SKIP47_CH2:
  .endif

	RET

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Initialize ECAP coninuous free running timer mode
; Uses: R1
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
FN_ECAP_INIT:
	;  Disable all interrupts on ECAP interface
	LDI		R1.w0, 0
	SBCO    &R1.w0, ICSS_ECAP, ICSS_eCAP_ECEINT, 2
	;  Clear spurious eCAP interrupt flags
	LDI     R1.w0, 0xFFFF
	SBCO    &R1.w0, ICSS_ECAP, ICSS_eCAP_ECCLR, 2
	;  Bypass prescaler and disable REGISTER load on capture event
	LDI     R1.w0, 0x8100
	SBCO    &R1.w0, ICSS_ECAP, ICSS_eCAP_ECCTL1, 2
	;  Select Capture Mode, Continuous Mode and free run TS Counter
	LDI     R1.w0, 0x0086          ;  bit 4 start / stop counter
	SBCO    &R1.w0, ICSS_ECAP, ICSS_eCAP_ECCTL2, 2
	;  Enable Counter overflow as Interrupt sources
	LDI     R1.w0, 0x0020
	SBCO    &R1.w0, ICSS_ECAP, ICSS_eCAP_ECEINT, 2
	RET

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Send EnDat master clock
; Args:R0.w0, R0.w2, R1.w0
; R0.w0: DIV for RX_CLK
; R0.w2: DIV for TX_CLK
; R1.w0: RX_EN timer
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

FN_SET_TX_CLK:
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_RXCFG+2
	SBCO	&R0.w0,	ICSS_CFG,	SCRATCH1.w0,	2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_TXCFG+2
	SBCO	&R0.w2,	ICSS_CFG,	SCRATCH1.w0,	2
	; Program rx_en_counter in ICSS_CFG_PRUx_ENDAT_CH0_CFG1[31:16]
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH0_CFG1+2
	SBCO	&R1.w0,	ICSS_CFG,	SCRATCH1.w0,	2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH1_CFG1+2
	SBCO	&R1.w0,	ICSS_CFG,	SCRATCH1.w0,	2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_CH2_CFG1+2
	SBCO	&R1.w0,	ICSS_CFG,	SCRATCH1.w0,	2
	RET

	.if	$isdefed("ENABLE_PROPDELAY_MESUREMENT")
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Estimate prop delay
; args:
;	R2.b1 - clock plus for position
;	R3.b0 - ENDAT_TX_CHx_SEL
;	R3.b1 - ENDAT_CHx_CLK
;	R3.b2 - ENDAT_CHx_SB
;	R3.b3 - ENDAT_CHx_TX_REINIT
; uses: R0, R1, R2, R8, R9
; return: R9 - prop delay
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
FN_PROP_DELAY_CALC:
	ZERO	&R8.w2,	6
PROPAGATION_DELAY_CALC_LOOP:
	 OR		R30.w2,	R3.b0, ENDAT_TX_CLK_MODE_FREERUN_STOPHIGH

        ; setup registers for transfer
	LDI		R30.b0,	ENDAT_CMD_SEND_POSITION_VALUES

        ; calc rx frame size

	; Note: position bits are read before invoking this fn in R2.b1, thus it
	; is supposed be an arguement, but for undiscovered yet reason, R2.b1 is
	; required to be read again here (except for ROQ437) irrespective of
	; whether it is updated before fn invocation. It gets stuck at waiting
	; for rising clock edge below.
	; This fixes it by reading # position bits in prop delay fn again, thus
	; causing prop delay to again being channel gnostic to some extent.
	.if	$isdefed("ENABLE_MULTI_CHANNEL")
	LBCO	&R2.b1,	PRUx_DMEM,	R8.b0,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_RTU") ;load clock plus for position bit  for ch0 in PRU
	  LBCO	&R2.b1,	PRUx_DMEM,	ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU") ; load clock plus for position bit  for ch1 in RTU
	  LBCO	&R2.b1,	PRUx_DMEM,	ENDAT_CH1_NUM_CLOCK_PULSES_OFFSET,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU") ; load clock plus for position bit for CH2 in TXPRU
	 LBCO	&R2.b1,	PRUx_DMEM,	ENDAT_CH2_NUM_CLOCK_PULSES_OFFSET,	1
	.else
	QBBC	ENDAT_SKIP39_CH0,	ENDAT_ENABLE_CHx,	0
	LBCO	&R2.b1,	PRUx_DMEM,	ENDAT_CH0_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP39_CH0:
	QBBC	ENDAT_SKIP39_CH1,	ENDAT_ENABLE_CHx,	1
	LBCO	&R2.b1,	PRUx_DMEM,	ENDAT_CH1_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP39_CH1:
	QBBC	ENDAT_SKIP39_CH2,	ENDAT_ENABLE_CHx,	2
	LBCO	&R2.b1,	PRUx_DMEM,	ENDAT_CH2_NUM_CLOCK_PULSES_OFFSET,	1
ENDAT_SKIP39_CH2:
	.endif
	LDI		R2.b2,	0
	; rx bits = position + CRC + F1
        ADD		R2.w1,	R2.w1, 6 ; CRC + F1

        ; rx frame size = rx bits * rx clk / (rx oversample * tx clk) = rx bits * (12000 * 8) / (8 * 200) = rx bits * 60
	LSL		R0.w0,	R2.w1,	6
	LSL		R2.w1,	R2.w1,	2
	SUB		R2.w1,	R0.w0, R2.w1

	LDI		R1.b0,	(7 << 3) ;  6(Mode bits)+1 pad
	LDI		R1.b2,	ENDAT_CMDTYP_2_1
	CALL2		FN_SEND

	LBCO      &R0, PRUx_DMEM,	ENDAT_CONFIG_DELAY_51US_OFFSET, 4;51 us (10.2T @200KHz, 0.2T to avoid boundary ambiguities)
	CALL2	   FN_DELAY_CYCLES


        ; wait for rising clock edge
ENDAT_TD_RISING_CLOCK:
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_ENDAT_TXCFG
	LBCO	&R0.w0,	ICSS_CFG,	SCRATCH1.w0,	2
        QBBC            ENDAT_TD_RISING_CLOCK,  R0.w0,  R3.b1
  ;set pru counter to zero
	ZERO		&R0,	4
    .if $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	SBCO 	&R0, c28, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
    .else
	SBCO 	&R0, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
	.endif
        ; wait for start bit
WB1:
	QBBC		WB1,	R31,	R3.b2


    ; read pru counter at time when rx start
    .if $isdefed("ENABLE_MULTI_MAKE_TXPRU")

	LBCO 	&R0, c28, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
    .else
	LBCO 	&R0, c11, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
	.endif
        ; rx_en = 0
	LDI		R30.b3, 0

     ;add counter value to register
	 ADD	R9,	R9,	R0

        ; wait for tx clk to get stopped naturally so that slave will not go to bad state (max ~ 54 * 5us < 1 ms)

    LBCO      &R0, PRUx_DMEM,	ENDAT_CONFIG_DELAY_1MS_OFFSET, 4; 1ms cycle counter
	CALL2		FN_DELAY_CYCLES

        ; bring to sane state for next transfer

     .if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LDI R14.b0, 0 ;clear syn_bit  for ch0
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
    M_WAIT_FOR_ENABLED_CHANNELS ; wait till syn_bit  not clear for all channels
    LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
    QBBC  SKIP_GLOBAL_TX_REINIT5, R14.b2, 0 ; check RTU is  primary core
	SET		R31,	ENDAT_TX_GLOBAL_REINIT
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LDI R14.b0, 0 ; clear syn_bit for ch1
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
     M_WAIT_FOR_ENABLED_CHANNELS ; wait till syn_bit  not clear for all channels
    LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
    QBBC  SKIP_GLOBAL_TX_REINIT5, R14.b2, 1 ; check PRU is  primary core
	SET		R31,	ENDAT_TX_GLOBAL_REINIT
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LDI R14.b0, 0 ;clear syn_bit for ch2
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
    M_WAIT_FOR_ENABLED_CHANNELS ; wait till syn_bit  not clear for all channels
    LBCO  &R14.b2,    PRUx_DMEM,  MASK_FOR_PRIMARY_CORE, 1
    QBBC  SKIP_GLOBAL_TX_REINIT5, R14.b2, 2 ; check TXPRU is  primary core
	SET		R31,	ENDAT_TX_GLOBAL_REINIT
    .else
    SET	R31,	ENDAT_TX_GLOBAL_REINIT ; Set TX_EN low
    .endif

SKIP_GLOBAL_TX_REINIT5:

WB4:
	QBBS		WB4,	R31,	R3.b3


    LBCO      &R0, PRUx_DMEM,	ENDAT_CONFIG_DELAY_2MS_OFFSET, 4
	CALL2	FN_DELAY_CYCLES
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LDI R14.b0, 0x1 ; //set syn_bit of ch0
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH0_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LDI R14.b0, 0x2 ; //set syn_bit of ch1
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH1_CONFIG_SYN_BIT,	1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LDI R14.b0, 0x4 ; //set syn_bit of ch2
	SBCO	&R14.b0,	PRUx_DMEM,	ENDAT_CH2_CONFIG_SYN_BIT,	1
    .endif

	ADD		R8.w2,	R8.w2,	1
	QBGT	PROPAGATION_DELAY_CALC_LOOP,	R8.w2,	8
	LSR		R9,	R9,	3 ; Average the 8 samples


    LBCO	&R27,	PRUx_DMEM,	ENDAT_CONFIG_DELAY_5US_OFFSET,	4

ENDAT_PROP_DELAY_MODULUS:
        QBGT            ENDAT_SKIP_PROP_DELAY_MODULUS, R9,   R27
	SUB		R9,     R9,     R27
        JMP             ENDAT_PROP_DELAY_MODULUS
ENDAT_SKIP_PROP_DELAY_MODULUS:

	RET
	.endif


;
; Copyright (C) 2021-2023 Texas Instruments Incorporated
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

	.include "memory.inc"
	.include "defines.inc"
	.include "macros.inc"
	.ref transport_init
	.ref qm_add
	.ref calc_rssi
	.ref send_stuffing
	.ref send_header
	.ref send_trailer
	.ref wait_delay
	.ref datalink_loadfw
	.ref recv_dec
	.ref datalink_wait_vsynch
	.global datalink_reset
	.global datalink_init_start
	.global send_01
	.global int_div

	.sect	".text"

relocatable0:

datalink_init_start:
;State RESET
	zero			&r0, 124
;send 2 times
datalink_reset:

;setup ICSS encoder peripheral for Hiperface DSL
	ldi			DISPARITY, 0x00
	TX_EN
	SET_TX_CH0
	REINIT_TX
	TX_FRAME_SIZE		0, REG_TMP0
    .if $defined("HDSL_MULTICHANNEL")
	TX_CLK_DIV 		CLKDIV_FAST, REG_TMP0
    .else
    TX_CLK_DIV 		CLKDIV_NORMAL, REG_TMP0
    .endif
	zero			&H_FRAME, (4*2)
;init transport layer here
	CALL			transport_init
;QualityMonitor is initialized with 8
	ldi			QM, 8
;free running mode frame size is 108
	ldi			EXTRA_SIZE, 0
	ldi			NUM_STUFFING, 0
;reset PRST bit in SYS_CTRL
	lbco			&REG_TMP0, MASTER_REGS_CONST, SYS_CTRL, 1
	clr			REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbco			&REG_TMP0, MASTER_REGS_CONST, SYS_CTRL, 1
;reset SAFE_CTRL register
    zero        &REG_TMP0.b0, 1
	sbco        &REG_TMP0.b0, MASTER_REGS_CONST, SAFE_CTRL, 1
; Write the fixed bits and reset PRST bits in ONLINE_STATUS_D, ONLINE_STATUS_1 and ONLINE_STATUS_2
; In ONLINE_STATUS_D high, bit 2 is FIX0, bit 4 is FIX1 and bit 5 is FIX0
; In ONLINE_STATUS_D low, bit 0 is FIX0 and bit 3 is FIX0
	lbco        &REG_TMP0.w0, MASTER_REGS_CONST, ONLINE_STATUS_D, 2
    ; clearing bits with fix0 and PRST bit
    and         REG_TMP0.w0, REG_TMP0.w0, ((~((1<<ONLINE_STATUS_D_PRST) | (1<<ONLINE_STATUS_D_HIGH_BIT5_FIX0) | (1<<ONLINE_STATUS_D_HIGH_BIT2_FIX0) | (1<<ONLINE_STATUS_D_LOW_BIT3_FIX0) | (1<<ONLINE_STATUS_D_LOW_BIT0_FIX0))) & 0xFF)
    ; setting bits with fix1
    or         REG_TMP0.w0, REG_TMP0.w0, (1<<ONLINE_STATUS_D_HIGH_BIT4_FIX1)
	sbco        &REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_D, 2
; In ONLINE_STATUS_1 high, bit 1 is FIX0, bit 3 is FIX0 and bit 4 is FIX1
; In ONLINE_STATUS_1 low, bit 1 is FIX0, bit 3 is FIX0 and bit 4 is FIX0
	lbco        &REG_TMP0.w0, MASTER_REGS_CONST, ONLINE_STATUS_1, 2
    ; clearing bits with fix0 and PRST bit
    and         REG_TMP0.w0, REG_TMP0.w0, ((~((1<<ONLINE_STATUS_1_PRST) | (1<<ONLINE_STATUS_1_HIGH_BIT1_FIX0) | (1<<ONLINE_STATUS_1_HIGH_BIT3_FIX0) | (1<<ONLINE_STATUS_1_LOW_BIT4_FIX0) | (1<<ONLINE_STATUS_1_LOW_BIT3_FIX0) | (1<<ONLINE_STATUS_1_LOW_BIT1_FIX0))) & 0xFF)
    ; setting bits with fix1
    or         REG_TMP0.w0, REG_TMP0.w0, (1<<ONLINE_STATUS_1_HIGH_BIT4_FIX1)
	sbco        &REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_1, 2
; In ONLINE_STATUS_2 high, bit 1 is FIX0, bit 3 is FIX0, bit 4 is FIX1 and bit7 is FIX1
; In ONLINE_STATUS_2 low, bits 0, 1, 3, 4, 5 are FIX0
	lbco        &REG_TMP0.w0, MASTER_REGS_CONST, ONLINE_STATUS_2, 2
    ; clearing bits with fix0 and PRST bit
    and         REG_TMP0.w0, REG_TMP0.w0, ((~((1<<ONLINE_STATUS_2_PRST) | (1<<ONLINE_STATUS_2_HIGH_BIT1_FIX0) | (1<<ONLINE_STATUS_2_HIGH_BIT3_FIX0) | (1<<ONLINE_STATUS_2_HIGH_BIT7_FIX0) | (1<<ONLINE_STATUS_2_LOW_BIT0_FIX0) | (1<<ONLINE_STATUS_2_LOW_BIT1_FIX0) | (1<<ONLINE_STATUS_2_LOW_BIT3_FIX0) | (1<<ONLINE_STATUS_2_LOW_BIT4_FIX0) | (1<<ONLINE_STATUS_2_LOW_BIT5_FIX0))) & 0xFF)
    ; setting bits with fix1
    or         REG_TMP0.w0, REG_TMP0.w0, (1<<ONLINE_STATUS_2_HIGH_BIT4_FIX1)
	sbco        &REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_2, 2
;check for SPOL and configure eCAP accordingly
	ldi			REG_TMP1, (ECAP+ECAP_ECCTL1)
	lbco			&REG_TMP2, PWMSS1_CONST, REG_TMP1, 4
	clr			REG_TMP2, REG_TMP2, 0
	qbbc			datalink_reset_spol_rising_edge, REG_TMP0, SYS_CTRL_SPOL
	set			REG_TMP2, REG_TMP2, 1
datalink_reset_spol_rising_edge:
	sbco			&REG_TMP2, PWMSS1_CONST, REG_TMP1, 4
;wait for pusle synch if register != 0
	lbco			&NUM_PULSES, MASTER_REGS_CONST, SYNC_CTRL, 1
	qbeq			datalink_no_sync, NUM_PULSES, 0
; remove this once external sync pulse is supported
	.if !$defined(EXT_SYNC_ENABLE)
	qba			datalink_no_sync
	.endif


;*************************************************************************************************************************;
;synchronize with SYNC Pulse here
	CALL1			sync_pulse
;*************************************************************************************************************************;

;***************************************************************************************************/
;wait logic for aligning with sync pulse for the very first  time
;pseudocode:
	;/*idea is that we should wait here and start such that first time extra edge alignes with sync pulse rise edge */
	;R20 has the pulse time in cycles. all the time here is i terms of pru cycles(4.44ns) unless specified
	;num_of_bits = 8*3 + EXTRA_SIZE //we push two times 0x00 and then actual 8 bits of frame start
	;/*each bit is 24 cycles in normal clock and 3 cycles in overclock, now extra edge is overclocked bits*
	;num_of_overclock_bits = num_of_bits*8 + TIME_REST
	;num_of_cycles = num_of_overclock_bits*3
	;wait_time = R20 - num_of_cycles
	;also we add 43 cycles to wait_time, this is coming from experimental data, we saw we were still ahead by 43 cycles
	lbco	   &R12, MASTER_REGS_CONST, SYNC_PARAM1, 4
	ldi         REG_TMP2, 0
	lbco	    &REG_TMP2, MASTER_REGS_CONST, WAIT_BEFORE_START, 2
	mov         EXTRA_EDGE_COMP, EXTRA_EDGE
	mov         EXTRA_SIZE_COMP, EXTRA_SIZE
	mov         TIME_REST_COMP, TIME_REST
	mov         NUM_STUFFING_COMP, NUM_STUFFING
;***************************************************************************************************/


sync_calc_time_rest_not_0:
	CALL1			sync_pulse
	WAIT			REG_TMP2
datalink_no_sync:
;init state machine here
;--------------------------------------------------------------------------------------------------
;State RESET
;--------------------------------------------------------------------------------------------------
datalink_reset2:
	;push dummy values to TX FIFO to gain processing time
    ;Push 8 bytes for single byte 0x00
    ;push first 4 bytes to fill fifo to max level then trigger channel for transmitting data
    ;later push further bytes in continous fifo load way
    .if $defined("HDSL_MULTICHANNEL")
	PUSH_FIFO_CONST			0x00
	PUSH_FIFO_CONST			0x00
	PUSH_FIFO_CONST			0x00
	PUSH_FIFO_CONST			0x00
    TX_CHANNEL
    WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
    ;Push 8 bytes for single byte 0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00

    .else
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	TX_CHANNEL
    .endif
	;send RESET 2 times to reset protocol
	ldi   LOOP_CNT_0 ,0
RESET_LOOP:
	;loop			datalink_reset2_end, 4
	;send m_par_reset 8b/10b: 5b/6b and 3b/4b, first=0,vsync=0,reserved=0
	ldi			REG_FNC.w0, (0x0000 | M_PAR_RESET)
	CALL			send_header
	CALL1			send_stuffing
	add 			LOOP_CNT_0, LOOP_CNT_0, 1
	qbne RESET_LOOP,LOOP_CNT_0,2
datalink_reset2_end:

;--------------------------------------------------------------------------------------------------
;State SYNC
;--------------------------------------------------------------------------------------------------
	ldi   LOOP_CNT_0 ,0
SYNC_LOOP:

datalink_sync:
	;send m_par_reset 8b/10b: 5b/6b and 3b/4b, first=0,vsync=0,reserved=0
	ldi			REG_FNC.w0, (0x0000 | M_PAR_SYNC)
	CALL			send_header
	CALL1			send_stuffing
	;TX_CHANNEL
datalink_sync_end:
	add 			LOOP_CNT_0, LOOP_CNT_0, 1
	qbne SYNC_LOOP,LOOP_CNT_0,16
;--------------------------------------------------------------------------------------------------
;State LEARN
; DLS response window is 1 switch bit + 61 slave answer and 12 delay bits
; at 100 meter cable we get a response delay of up to 11 bits, so we need to read max 72 bits for
; full test pattern
; VAL flag on last bit comes late. Need to rearrange code for last bit processing
; at 0 meter cable we get a response delay of 0 bits, need to make sure we dont sample to early
;--------------------------------------------------------------------------------------------------
;M_PAR_START is repeated 9 times
	;loop			datalink_learn_end, 9
	ldi			LEARN_STATE_STARTED , 1			;state indicator
	ldi			LOOP_CNT.b1, 9			;9

datalink_learn:
	;;WAIT_TX_FIFO_FREE
;send m_par_reset 8b/10b: 5b/6b and 3b/4b, first=0,vsync=0,reserved=0
	ldi			REG_FNC.w0, (0x0000 | M_PAR_START)
	CALL			send_header
; indication of TX_DONE comes about 53ns after wire timing
	WAIT_TX_DONE
    .if $defined("FREERUN_300_MHZ")
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
    .endif
 	.if $defined("HDSL_MULTICHANNEL")
    .if $defined("CHANNEL_0")
	NOP_2
	NOP_2
    .endif
    .endif
; measured starting point at 0 cable length
; first 8 bits will be all ones is delay from encoder and transceiver
; second 8 bits is oversampled DSL bit which is 0 on test pattern
; channel enable will always hit a high state which allows for save EDGE detection
; and SLAVE_DELAY determination
; the measured offset used below is 4 bits (OVS) * (13.333 / 4.444) + 4.444 ns.
        loop	wait_on_rx_transtion_in_learn_state, 1 ; (4*7) for 100m
        add		r0,r0,0
wait_on_rx_transtion_in_learn_state:
; now receive starts in save high state. First VAL comes after 180ns. Following ones in DSL
; bit times of 106.66 ns. Make sure code before next VAL is less than 106 ns!!!
	RX_EN

; measue the time of receive window, 2 cycles
; compensation value should be
;  108 bits
;      - 12 cycles (53 ns)
;      - 2 cycles (RESET_CYCLCNT)
;      - time to switch to TX and start sending trailer
	RESET_CYCLCNT

;read 61+11 bits
;Channel is already tiggered. First 0-1 transition will be from bit 1 to bit 2 of test pattern
;response from encoder.
;DSL data will be stored to r20-r18
	ldi			LOOP_CNT.b0, 72
	zero			&r18, (4*3)
	ldi			r1.b0, &r21.b0
; reset flag that indicates first rising edge detected in this DSL frame
	ldi 		REG_TMP11.b0, 0
; this loop executes one DSL byte in oversample mode
datalink_learn_recv_oloop:
	ldi			REG_TMP1.b1, 0
	ldi			LOOP_CNT.b2, 8

; this loop executes one DSL bit in 8x oversample mode
datalink_learn_recv_loop:
; this is after 7 cycles ~30 ns on first VAL
	qbbc			datalink_learn_recv_loop, r31, RX_VALID_FLAG
	POP_FIFO		REG_TMP0.b0
; after clearing VAL there needs to be 2 PRU cycles before we can poll for VAL again!
	CLEAR_VAL

; for each frame, detect the SAMPLE_EDGE,
; detect first falling edge which is received byte < 255
    	qbne			datalink_learn_recv_loop_not_first,REG_TMP11.b0, 0
    	qbeq			datalink_learn_recv_loop_not_first,REG_TMP0.b0,0xff
; result is in SAMPLE_EDGE and gives the sampling bit number
	FIND_EDGE		REG_TMP0.b0, REG_TMP2
; bits are counted from LSB to MSB with SET cmd
; data bits are moved MSB to LSB, hence we need to reverse the bit position
	RSB				SAMPLE_EDGE,SAMPLE_EDGE, 7

; at the 100 meter boundary do not move to next sample with
; this is 10 bits delay, and SAMPLE_EDGE with wrap around
	qbne			datalink_learn_recv_loop_100m, LOOP_CNT.b0, 64
    	qbne		    datalink_learn_recv_loop_100m, LOOP_CNT.b2, 6
	qbge			datalink_learn_recv_loop_100m, SAMPLE_EDGE, 3
; cap SAMPLE_EDGE to last bit position at 100 meter
	ldi				SAMPLE_EDGE, 0
datalink_learn_recv_loop_100m:
; temp save
	set				R22, R22, SAMPLE_EDGE
; do it only once
	ldi				REG_TMP11.b0, 1
datalink_learn_recv_loop_not_first:
	sub			LOOP_CNT.b2, LOOP_CNT.b2, 1
	qbbc		datalink_learn_recv_loop_0, REG_TMP0.w0, SAMPLE_EDGE
	set			REG_TMP1.b1, REG_TMP1.b1, LOOP_CNT.b2
datalink_learn_recv_loop_0:
;TODO: get EDGES
	mov			REG_TMP0.b1, REG_TMP0.b0
; on last byte do only 7 bits
	qbne		datalink_learn_skip_one_bit, LOOP_CNT.b0, 8
	qbne		datalink_learn_recv_loop, LOOP_CNT.b2, 1
    	qba 		datalink_learn_skip_one_bit_1
datalink_learn_skip_one_bit:
	qbne			datalink_learn_recv_loop, LOOP_CNT.b2, 0
	mvib			*--r1.b0, REG_TMP1.b1
	sub			LOOP_CNT.b0, LOOP_CNT.b0, 8
	qbne			datalink_learn_recv_oloop, LOOP_CNT.b0, 0

datalink_learn_skip_one_bit_1:
; this code section minimizes time between last VAL and TX_EN

; pre-load register to save time on last bit
;	ldi			REG_TMP2, (74*CYCLES_BIT-9) ; 100 m
    .if $defined("FREERUN_300_MHZ")
	ldi			r3, (74*CYCLES_BIT+9-2)
    .else
    ldi			r3, (74*CYCLES_BIT+9)
    .endif

datalink_learn_recv_loop_last_bit:
	qbbc			datalink_learn_recv_loop_last_bit, r31, RX_VALID_FLAG

; now finisch with last bit sample and store
	POP_FIFO		REG_TMP0.b0
	sub			LOOP_CNT.b2, LOOP_CNT.b2, 1
	qbbc		datalink_learn_recv_loop_final, REG_TMP0.b0, SAMPLE_EDGE
	set			REG_TMP1.b1, REG_TMP1.b1, LOOP_CNT.b2
datalink_learn_recv_loop_final:
	mov			REG_TMP0.b1, REG_TMP0.b0
    	mvib		*--r1.b0, REG_TMP1.b1

; this delay code should handle the case where both values are equal
; WAIT macro takes n+2 cycles, which is 3 for n = 1
; in case of n = 0 we skip 5 cycles wich causes the encoder to go out of sync!!!

	.if $defined(EXT_SYNC_ENABLE_DEBUG)
	lbco        &REG_TMP2, c25, 0, 4
	add         REG_TMP2,REG_TMP2,12
	sbco        &R18, c25, REG_TMP2, 12
	sbco        &REG_TMP2, c25, 0, 4
	.endif

	READ_CYCLCNT	r25
; avoid wrap around, need to skip on equal as wait does not work for 0.
;	qble	    datalink_learn_skip_wait, r25, r3
	qble	    datalink_abort2, r25, r3
	sub			REG_TMP11, r3, r25
	MOV			r25.b0, REG_TMP11.b0
; WAIT subracts -1 from parameter before compare. On 0 it wraps around!!!
	WAIT		REG_TMP11
datalink_learn_skip_wait:
	TX_EN
    .if $defined("HDSL_MULTICHANNEL")
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	nop
    .endif
;send TRAILER 0x03 (skipping first 2 bits of logic 0 to avoid some extra delays)
    .if $defined("HDSL_MULTICHANNEL")
	PUSH_FIFO_CONST		0x00
	TX_CHANNEL
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0xff
    .else
    PUSH_FIFO_CONST		0x03
	TX_CHANNEL
    .endif
;	2 dummy cycles
	NOP_2
; test: we are in oversample mode (3 PRU clocks per bit)
; extra NOPs should make it shorter
	NOP_2
	NOP_2
	NOP_2
    .if $defined("FREERUN_300_MHZ")
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
    .endif
    .if !$defined("HDSL_MULTICHANNEL")
    TX_CLK_DIV		CLKDIV_SLOW, REG_TMP2
    .endif
;reset DISPARITY
	ldi			DISPARITY, 0
	;2 dummy cycles
	NOP_2
    .if $defined("HDSL_MULTICHANNEL")
    TX_CLK_DIV_WAIT		CLKDIV_NORMAL, REG_TMP2
    .else
    TX_CLK_DIV		CLKDIV_NORMAL, REG_TMP2
    .endif

;syn with clock before resetting counter
	WAIT_CLK_LOW		REG_TMP2
;reset cycle count
	RESET_CYCLCNT
datalink_learn_pattern:
	.if $defined(EXT_SYNC_ENABLE)
	.else
    WAIT_TX_FIFO_FREE
    .if $defined("HDSL_MULTICHANNEL")
;add stuffing to gain processing time
	;PUSH 8 bytes for 1 byte data (0x2c) in FIFO
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00

	;PUSH 8 bytes for 1 byte data (0xb2) in FIFO
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0xff
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0x00

;	PUSH 8 bytes for 1 byte data (0xcb) in FIFO
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0xff
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0xff
    .else
;add stuffing to gain processing time
	PUSH_FIFO_CONST		0x2c
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xb2
	PUSH_FIFO_CONST		0xcb
    .endif  ;HDSL_MULTICHANNEL

    .endif  ;EXT_SYNC_ENABLE
;extensive search for test pattern
	ldi			LOOP_CNT.b3, 0
	qbne		datalink_learn_delay, LOOP_CNT.b1, 1

datalink_learn_delay:

	.if $defined(EXT_SYNC_ENABLE_DEBUG)
	lbco        &REG_TMP2, c25, 0, 4
	add         REG_TMP2,REG_TMP2,12
	sbco        &R18, c25, REG_TMP2, 12
	sbco        &REG_TMP2, c25, 0, 4
	.endif
;shift data and remove first switch bit
	add			LOOP_CNT.b3, LOOP_CNT.b3, 1
	lsl			r20, r20, 1
	lsr			REG_TMP0.b0, r19.b3, 7
	or			r20, r20, REG_TMP0.b0
	lsl			r19, r19, 1
	lsr			REG_TMP0.b0, r18.b3, 7
	or			r19.b0, r19.b0, REG_TMP0.b0
	lsl			r18, r18, 1
;check pattern
	CALL1		check_test_pattern
	qbeq		datalink_abort2, LOOP_CNT.b3, 14
;	qbeq		datalink_learn_end_test, LOOP_CNT.b3, 14
	qbne		datalink_learn_delay, REG_FNC.b0, 1
datalink_learn_end_test:
; SLAVE_DELAY has no switch bit
    	mov			SLAVE_DELAY, LOOP_CNT.b3
;send STUFFING
	.if $defined(EXT_SYNC_ENABLE)
	CALL1			send_stuffing
	.endif
datalink_learn_end:
	sub			LOOP_CNT.b1, LOOP_CNT.b1, 1
	qblt		datalink_learn, LOOP_CNT.b1, 0
;	qba			datalink_abort2_no_wait
;we need a rel. jump here
	qba			datalink_learn2_before
;--------------------------------------------------------------------------------------------------
datalink_abort2:
    	halt
	qbbs			datalink_abort2_no_wait, r30, RX_ENABLE						;changed here from 24 to 26
	WAIT_TX_DONE
    .if $defined("FREERUN_300_MHZ")
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
    .endif
datalink_abort3:
    	halt
datalink_abort2_no_wait:
	lbco			&REG_TMP0.b0, MASTER_REGS_CONST, NUM_RESETS, 1
	add			REG_TMP0.b0, REG_TMP0.b0, 1
	sbco			&REG_TMP0.b0, MASTER_REGS_CONST, NUM_RESETS, 1
; Set EVENT_PRST in EVENT_H register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_PRST
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	qbbc		update_events_no_int15, REG_TMP0.w2, EVENT_PRST
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int15:
; Set EVENT_S_PRST in EVENT_S register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_S, 2
	set		REG_TMP0.b0, REG_TMP0.b0, EVENT_S_PRST
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_S, 1
	qbbc		update_events_no_int22, REG_TMP0.b1, EVENT_S_PRST
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ4
update_events_no_int22:
;we need rel. jump here
	qba			datalink_reset
;--------------------------------------------------------------------------------------------------
;M_PAR_LEARN does not seem to have further meaning...
datalink_learn2_before:
	ldi			LOOP_CNT.b1, 9; 16
datalink_learn2:
    .if !$defined("HDSL_MULTICHANNEL")
	WAIT_TX_FIFO_FREE
    .endif
;send m_par_reset 8b/10b: 5b/6b and 3b/4b, first=0,vsync=0,reserved=0
	ldi			REG_FNC.w0, (0x0000 | M_PAR_LEARN)
	CALL			send_header
	CALL			receive
	.if $defined(EXT_SYNC_ENABLE_DEBUG)
	lbco        &REG_TMP2, c25, 0, 4
	add         REG_TMP2,REG_TMP2,12
	sbco        &R18, c25, REG_TMP2, 12
	sbco        &REG_TMP2, c25, 0, 4
	.endif
	;check test pattern
	CALL1			check_test_pattern
	qbne			datalink_abort3, REG_FNC.b0, 1
	sub			LOOP_CNT.b1, LOOP_CNT.b1, 1
	qblt			datalink_learn2, LOOP_CNT.b1, 0
datalink_learn2_end:
;--------------------------------------------------------------------------------------------------
;State LINK CHECK
;this is repeated 16 times
	ldi			LOOP_CNT.b1, 16
datalink_line_check:
	ldi			REG_FNC.w0, (0x0000 | M_PAR_CHECK)
	CALL			send_header
	CALL			receive
;check test pattern
	CALL1			check_test_pattern
	qbne			datalink_abort2, REG_FNC.b0, 1
	sub			LOOP_CNT.b1, LOOP_CNT.b1, 1
	qblt			datalink_line_check, LOOP_CNT.b1, 0
	;qba datalink_line_check
datalink_line_check_end:
;--------------------------------------------------------------------------------------------------
;State ID REQ
;save delay to master registers after all checks were successful
	sbco			&SLAVE_DELAY, MASTER_REGS_CONST, DELAY, 1
datalink_id_req:
	ldi			REG_FNC.w0, (0x0000 | M_PAR_IDREQ)
	CALL			send_header
	CALL			recv_dec
;--------------------------------------------------------------------------------------------------
;State ID STORE
datalink_id_store:
;check if 40 bits is palindrome (bytewise! not bitwise!), if not repeat this step
;-> vert == acc.b0 && par == acc.b1 && pipe.nibble0 == pipe.nibble1
;second half of palindrome is actually our ENC_ID!
	qbne			datalink_abort2, H_FRAME.vert, H_FRAME_acc0
	qbne			datalink_abort2, H_FRAME.s_par, H_FRAME_acc1
	and			REG_TMP0.b0,  H_FRAME.pipe, 0xf
	lsr			REG_TMP0.b1,  H_FRAME.pipe, 4
	qbne			datalink_abort2, REG_TMP0.b0, REG_TMP0.b1;
;now store the encoder ID
	mov			REG_TMP0.w0, H_FRAME.acc
	mov			REG_TMP0.b2, H_FRAME.pipe
;now in memory for master registers
;big endian format
	mov			REG_TMP1.b0, REG_TMP0.b2
	mov			REG_TMP1.b1, REG_TMP0.b1
	mov			REG_TMP1.b2, REG_TMP0.b0
	sbco			&REG_TMP1, MASTER_REGS_CONST, ENC_ID2, 3
;Safe QM + set LINK bit, which means we have established a link
;just add 0 to QM to update
	QM_ADD			0
;Synchronization with Drive Cycle is enabled here!
	set			H_FRAME.flags, H_FRAME.flags, FLAG_DRIVE_SYNC
;--------------------------------------------------------------------------------------------------
;State ID COMPUTE
datalink_id_compute:
;decode the ENC_ID here
;num of acc bits is always +8
	and			NUM_ACC_BITS, REG_TMP0, 0x0f
	add			NUM_ACC_BITS, NUM_ACC_BITS, 8
;num pos bits is +num acc bits
	lsr			NUM_ST_BITS, REG_TMP0, 4
	and			NUM_ST_BITS, NUM_ST_BITS, 0x3f
	add			NUM_ST_BITS, NUM_ST_BITS, NUM_ACC_BITS
	sub			NUM_ST_BITS, NUM_ST_BITS, NUM_MT_BITS
;finding the mask for position
	add			REG_TMP0.b0, NUM_ST_BITS, NUM_MT_BITS
	rsb			REG_TMP0.b0, REG_TMP0.b0, 40
	ldi32			REG_TMP1, 0xffffffff
	lsr			REG_TMP1, REG_TMP1, REG_TMP0.b0
	sbco			&REG_TMP1, MASTER_REGS_CONST, MASK_POS, 4
	;qba datalink_id_req
	CALL1		send_stuffing
	jmp         datalink_wait_vsynch
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
;Function:
;This functions receives data without deocoding.
;output:
;	r20-r19: data
;--------------------------------------------------------------------------------------------------
receive:
	ldi			LOOP_CNT.b0, 32
	CALL1		wait_delay
	ldi			REG_TMP0.w0, 0
	ldi			REG_TMP11, (PDMEM00+0x5a4);LUT_B2B)
	zero		&r18, (4*5)
datalink_receive_signal_0_31_1:
	qbbc			datalink_receive_signal_0_31_1, r31, RX_VALID_FLAG
	POP_FIFO		REG_TMP0.b0
	CLEAR_VAL
	sub			LOOP_CNT.b0, LOOP_CNT.b0, 1
	qbbc			datalink_receive_signal_0_31_received_0_1, REG_TMP0.w0, SAMPLE_EDGE
	set			r20, r20, LOOP_CNT.b0
datalink_receive_signal_0_31_received_0_1:
	mov			REG_TMP0.b1, REG_TMP0.b0
;get edges
	lsr			REG_TMP1.b0, REG_TMP0.w0, 1
	xor			CUR_EDGES, REG_TMP1.b0, REG_TMP0.b0
	CALL1		calc_rssi
	qbne		datalink_receive_signal_0_31_1, LOOP_CNT.b0, 0
;receive next bits
	ldi			LOOP_CNT.b0, 29
datalink_receive_signal_32_60_1:
	qbbc			datalink_receive_signal_32_60_1, r31, RX_VALID_FLAG					;changed here from 24 to 26
	POP_FIFO		REG_TMP0.b0
	CLEAR_VAL
	sub			LOOP_CNT.b0, LOOP_CNT.b0, 1
	qbbc			datalink_receive_signal_32_60_received_0_1, REG_TMP0.w0, SAMPLE_EDGE
	set			r19, r19, LOOP_CNT.b0
datalink_receive_signal_32_60_received_0_1:
	mov			REG_TMP0.b1, REG_TMP0.b0
;get edges
	lsr			REG_TMP1.b0, REG_TMP0.w0, 1
	xor			CUR_EDGES, REG_TMP1.b0, REG_TMP0.b0
	CALL1			calc_rssi
	qbne			datalink_receive_signal_32_60_1, LOOP_CNT.b0, 1
datalink_receive_signal_last_1:
	qbbc			datalink_receive_signal_last_1, r31, RX_VALID_FLAG					;changed here from 24 to 26
	POP_FIFO		REG_TMP0.b0
	qbbc			datalink_receive_signal_last_received_0_1, REG_TMP0.w0, SAMPLE_EDGE
	set			r19, r19, 0
datalink_receive_signal_last_received_0_1:
	lsl			r19, r19, 3
	CLEAR_VAL
; same delay code as in learn
	ldi			REG_TMP1, (74*CYCLES_BIT+7)  ; -9 for 100 m
	READ_CYCLCNT		REG_TMP0
	qble	    receive_skip_wait, REG_TMP0, REG_TMP1
	sub			REG_TMP0, REG_TMP1, REG_TMP0
	add			r0,r0,1
	WAIT		REG_TMP0
receive_skip_wait:
	TX_EN
; datalink_receive_signal_no_delay_wait_1:
	CALL1			send_trailer
	CALL1			send_stuffing
	RET
;--------------------------------------------------------------------------------------------------
;Function: sync_pulse (RET_ADDR1)
;functions bussy waits for sync pulse
;input:
;modifies:
;--------------------------------------------------------------------------------------------------
;stores sync pulse period in R20 in unit of cycles
sync_pulse:
	lbco        &REG_TMP1, c1, IEP_CAPR6_RISE, 4
wait_next_pulse:
	lbco        &R20, c1, IEP_CAPR6_RISE, 4
	QBEQ		wait_next_pulse, R20, REG_TMP1
	SUB         R20, R20, REG_TMP1
	RET1
;--------------------------------------------------------------------------------------------------
;Function: int_div (RET_ADDR1)
;integer divides
;input:
;	REG_FNC.w0: Number
;	REG_FNC.w2: Divisor
;output:
;	REG_FNC.w2: Result
;	REG_FNC.w0: Rest
;modifies:
;--------------------------------------------------------------------------------------------------
int_div:
	ldi			REG_TMP0, 0
int_div_loop:
	qbgt			int_div_end, REG_FNC.w0, REG_FNC.w2
	sub			REG_FNC.w0, REG_FNC.w0, REG_FNC.w2
	add			REG_TMP0, REG_TMP0, 1
	qba			int_div_loop
int_div_end:
	mov			REG_FNC.w2, REG_TMP0
	RET1
;--------------------------------------------------------------------------------------------------
;Function: check_test_pattern (RET_ADDR1)
;This function checks if the test pattern was received
;input:
;	r18-r20: data
;output:
;	REG_FNC.b0: 1 if true
;modifies:
;	REG_TMP0, REG_FNC
;--------------------------------------------------------------------------------------------------
check_test_pattern:
;load test pattern and mask from memory
	lbco			&REG_TMP0, MASTER_REGS_CONST, TEST_PATTERN0, 12
;rm switch bit
	and			REG_TMP11, r19, REG_TMP2
	ldi			REG_TMP2, 0xff8
	and			REG_TMP2, r19, REG_TMP2
	lsl			REG_TMP2, REG_TMP2, 1
	or			REG_TMP11, REG_TMP2, REG_TMP11
;if found go to next step
	qbne			check_test_pattern_false, r20, REG_TMP0
	qbne			check_test_pattern_false, REG_TMP11, REG_TMP1
check_test_pattern_true:
	ldi			REG_FNC.b0, 1
	RET1
check_test_pattern_false:
	ldi			REG_FNC.b0, 0
	RET1
;--------------------------------------------------------------------------------------------------
;Function: send_01 (RET_ADDR1)
;This function sends 01 pattern in RESET and SYNC state
;input:
;	REG_FNC.b2: last two bits of parameter channel
;output:
;modifies:
;--------------------------------------------------------------------------------------------------
send_01:
;send 01 pattern
;2 para bits, 1 switch bit, 5 slave bit
	or			REG_FNC.b2, REG_FNC.b2, 0x15;0bPPS10101
    .if $defined("HDSL_MULTICHANNEL")
	mov FIFO_L,REG_FNC.b2
	PUSH_FIFO_8x	FIFO_L
    .else
	PUSH_FIFO		REG_FNC.b2
    .endif
;56+12 line delay slave bits
	ldi			REG_TMP0.b0, 8
send_header_send_01_pattern_loop:
	;;PUSH 8 bytes for 1 byte data (0x55) in FIFO
	WAIT_TX_FIFO_FREE
    .if $defined("HDSL_MULTICHANNEL")
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0xff
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0xff
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0xff
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0xff
    .else
    PUSH_FIFO_CONST		0x55
    .endif
	sub			REG_TMP0.b0, REG_TMP0.b0, 1
	qbne			send_header_send_01_pattern_loop, REG_TMP0.b0, 0
;send last 0101 (4 bits)
	WAIT_TX_FIFO_FREE
;overclock(8x)
	PUSH_FIFO_CONST		0x00
	ldi			REG_TMP0, (9*(CLKDIV_NORMAL+1)-9)
    .if !$defined("HDSL_MULTICHANNEL")
	WAIT			REG_TMP0
	TX_CLK_DIV		CLKDIV_FAST, REG_TMP0
    .endif
	PUSH_FIFO_CONST		0xff
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0xff
;push TRAILER
	;PUSH 8 bytes for 1 byte data (0x03) in FIFO
	WAIT_TX_FIFO_FREE
    .if $defined("HDSL_MULTICHANNEL")
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0xff
    .else
    PUSH_FIFO_CONST		0x03
    .endif
    .if $defined("FREERUN_300_MHZ")
	ldi			REG_TMP0, (6*(CLKDIV_FAST+1)-8+2)
    .else
    ldi			REG_TMP0, (6*(CLKDIV_FAST+1)-8)
    .endif
    .if !$defined("HDSL_MULTICHANNEL")
    WAIT			REG_TMP0
	TX_CLK_DIV		CLKDIV_NORMAL, REG_TMP0
    ;wait to have same timing as send_trailer
	ldi			REG_TMP0, 30;6
    WAIT			REG_TMP0
    .endif


;reset cyclecount
	RESET_CYCLCNT
	RET1

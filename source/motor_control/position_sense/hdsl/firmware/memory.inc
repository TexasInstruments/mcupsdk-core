
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

;--------------------------------------------------------------------------------------------------
;Offsets
;--------------------------------------------------------------------------------------------------
;ICSS_CFG .set                           C4
;SYSCFG .set                            0x04
;GPCFG0 .set                            0x08
;GPCFG1 .set                            0x0c
;PMAO .set                              0x28
P0EDRXCFG .set                          0xe0
P0EDTXCFG .set                          0xe4
P0EDCH0CFG0 .set                        0xe8
P0EDCH0CFG1 .set                        0xec
P0EDCH1CFG0 .set                        0xf0
P0EDCH1CFG1 .set                        0xf4
P0EDCH2CFG0 .set                        0xf8
P0EDCH2CFG1 .set                        0xfc

P1EDRXCFG .set                          0x100
P1EDTXCFG .set                          0x104
P1EDCH0CFG0 .set                        0x108
P1EDCH0CFG1 .set                        0x10C
P1EDCH1CFG0 .set                        0x110
P1EDCH1CFG1 .set                        0x114
P1EDCH2CFG0 .set                        0x118
P1EDCH2CFG1 .set                        0x11C

;--------------------------------------------------------------------------------------------------
;ICSS0 PRU0 DMEM
;--------------------------------------------------------------------------------------------------
    .if $defined(PRU1)
PDMEM00 .set                           0x00
ICSS_CFGx .set                          C5
EDTXCFG   .set                          (P1EDTXCFG-0x100)
EDRXCFG   .set                          (P1EDRXCFG-0x100)
EDCH0CFG0 .set                          (P1EDCH0CFG0-0x100)
EDCH0CFG1 .set                          (P1EDCH0CFG1-0x100)
EDCH1CFG0 .set                          (P1EDCH1CFG0-0x100)
EDCH1CFG1 .set                          (P1EDCH1CFG1-0x100)
EDCH2CFG0 .set                          (P1EDCH2CFG0-0x100)
EDCH2CFG1 .set                          (P1EDCH2CFG1-0x100)
    .else
PDMEM00 .set                           0x00
ICSS_CFGx .set                          C4
EDTXCFG   .set                          P0EDTXCFG
EDRXCFG   .set                          P0EDRXCFG
EDCH0CFG0 .set                          P0EDCH0CFG0
EDCH0CFG1 .set                          P0EDCH0CFG1
EDCH1CFG0 .set                          P0EDCH1CFG0
EDCH1CFG1 .set                          P0EDCH1CFG1
EDCH2CFG0 .set                          P0EDCH2CFG0
EDCH2CFG1 .set                          P0EDCH2CFG1
    .endif


PDMEM00_CONST .set						C24
MASTER_REGS_CONST .set					C24
IEP1_BASE_CONST .set					C1
;Hiperface DSL Master Registers
MASTER_REGS .set						0x00
MASTER_REGS_SIZE .set						(DUMMY_END + 1)
;LUT for 5b/6b encoding
LUT_5b6b_ENC .set						(MASTER_REGS + MASTER_REGS_SIZE)
LUT_5b6b_ENC_SIZE .set						32
;LUT for 5b/6b decoding
LUT_5b6b_DEC .set						(LUT_5b6b_ENC + LUT_5b6b_ENC_SIZE)
LUT_5b6b_DEC_SIZE .set						64
;LUT for 3b/4b encoding
LUT_3b4b_ENC .set						(LUT_5b6b_DEC + LUT_5b6b_DEC_SIZE)
LUT_3b4b_ENC_SIZE .set						8
;LUT for 3b/4b decoding
LUT_3b4b_DEC .set						(LUT_3b4b_ENC + LUT_3b4b_ENC_SIZE)
LUT_3b4b_DEC_SIZE .set						16
LUT_BITCNT .set							(LUT_3b4b_DEC + LUT_3b4b_DEC_SIZE)
LUT_BITCNT_SIZE .set						256
LUT_CRC16 .set							(LUT_BITCNT + LUT_BITCNT_SIZE)
LUT_CRC16_SIZE .set						(256*2)
LUT_CRC5 .set							(LUT_CRC16 + LUT_CRC16_SIZE)
LUT_CRC5_SIZE .set						(1*256)
ABS_ERR_PTR .set						(LUT_CRC5 + LUT_CRC5_SIZE)
ABS_ERR_PTR_SIZE .set						2
ABS_ERR_BUF .set						(ABS_ERR_PTR + ABS_ERR_PTR_SIZE)
ABS_ERR_BUF_SIZE .set						(NUM_FRAMES_AVG*4)
LUT_B2B .set							(ABS_ERR_BUF + 2 + ABS_ERR_BUF_SIZE) ;+2 is for 4-byte address alignment
LUT_B2B_SIZE .set						(4*16)
LUT_RSSI .set							(LUT_B2B + LUT_B2B_SIZE)
LUT_RSSI_SIZE .set						(256)
LUT_EE .set							(0x5e4 + LUT_RSSI_SIZE);(LUT_RSSI + LUT_RSSI_SIZE)
LUT_EE_SIZE .set						9

;--------------------------------------------------------------------------------------------------
;Register Usage
;--------------------------------------------------------------------------------------------------
;channel structure
channel_s .struct
ch_vertl	.int
ch_verth	.int
ch_secl		.int
ch_sech		.int
ch_paral	.int
ch_parah	.int
	.endstruct
CHANNEL .sassign r18, channel_s
	.asg	r18.w0, crc_vert
	.asg	r18, VERT_L
	.asg	r19, VERT_H

h_frame_s .struct
vert		.byte
s_par		.byte
pipe		.byte
secondary	.byte
acc		.short
flags		.short
	.endstruct
H_FRAME .sassign r3, h_frame_s
	.asg							r4.b0, H_FRAME_acc0
	.asg							r4.b1, H_FRAME_acc1
	.asg							r4.b2, H_FRAME_flags_l
	.asg							r4.b3, H_FRAME_flags_h


short_msg_s .struct
crc		.short
data		.byte
addr		.byte
bits_left	.byte
timeout		.byte
	.endstruct
SHORT_MSG .sassign r26, short_msg_s
	.asg							r26, SHORT_MSG32
	.asg							r26.b0, SHORT_MSG_CRC_L
	.asg							r26.b1, SHORT_MSG_CRC_H

long_msg_recv_s .struct
crc		.short
bits_left	.byte
ptr		.byte
	.endstruct
LONG_MSG_RECV .sassign r28, long_msg_recv_s
	.asg							r28.b1, LONG_MSG_RECV_CRC_H
	.asg							r28.b0, LONG_MSG_RECV_CRC_L


	.asg							r6.b3, QM
	.asg							r6.b2, SLAVE_DELAY
	.asg							r6.b1, LAST_BIT_SENT
	.asg							r6.b0, SAMPLE_EDGE
	.asg							r5, LOOP_CNT
	.asg							r0, REG_TMP0
	.asg							r1, REG_TMP1
	.asg							r2, REG_TMP2
	.asg							r20, REG_SCRATCH
	.asg							r11, REG_TMP11
	.asg							r16.b0, RSSI
	.asg							r16.b1, CUR_EDGES
	.asg							r16.b2, SPECIAL_CHARACTER
	.asg							r16.b3, BYTE_ERROR
	.asg							r15, FAST_POSH
	.asg							r14.b3, FAST_POSL
	.asg							r14, SPEED
	.asg							r17.b0, DISPARITY
	.asg							r17.b1, SEND_PARA
	.asg							r29.w0, RET_ADDR0
	.asg							r17.w2, RET_ADDR1
	.asg							r29.b2, NUM_ACC_BITS
	.asg							r29.b3, NUM_ST_BITS
	.asg							r13, REG_FNC
	.asg							r7.w0, DELTA_ACC0
	.asg							r7.w2, DELTA_ACC1
	.asg 							r8.w0, DELTA_ACC2
	.asg							r8.w2, DELTA_ACC3
	.asg							r9.w0, DELTA_ACC4
	.asg							r9.w2, LAST_ACC
	;.asg							r10, EXTRA_POS
	;.asg							r10.w0, TIME_COMP
	;.asg							r10.w2, LAST_TIME_COMP
	.asg							r10.w0, SYNC_DIFF0
	.asg							r12.b0, EXTRA_SIZE
	.asg							r12.b1, NUM_STUFFING
	.asg							r12.b2, EXTRA_EDGE
	.asg							r12.b3, TIME_REST
	.asg							r27.b2, NUM_PULSES
	.asg							r25.w2, TIME_EXTRA_WINDOW ;free for am65xx use case
	.asg							r25.b2, MODIFIED_HEADER_STARTED
	.asg							r27.b3, LEARN_STATE_STARTED
	.asg							r25.b0, ALIGN_PH
	.asg							r24, CRC
	.asg							r24.w0, CRC_VERT
	.asg							r24.b1, CRC_VERT_H
	.asg							r24.b0, CRC_VERT_L
	.asg							r24.w2, CRC_SEC
	.asg							r24.b3, CRC_SEC_H
	.asg							r24.b2, CRC_SEC_L
	
	.asg							r2.b0, EXTRA_SIZE_SELF ;take care! these are not persistent registers, r2 is actually temp reigister. these names are 
	.asg							r2.b1, EXTRA_EDGE_SELF ;given to temp register for ease of reading. they are used as temp.
	
	.asg							r10.b0, EXTRA_EDGE_COMP
	.asg							r10.b1, EXTRA_SIZE_COMP
	.asg							r10.b2, TIME_REST_COMP
	.asg							r10.b3, NUM_STUFFING_COMP

;--------------------------------------------------------------------------------------------------
;DSL Master Register offsets
SYS_CTRL	.set						0x00
SYNC_CTRL	.set						0x01
MASTER_QM	.set						0x03
EVENT_H		.set						0x04
EVENT_L		.set						0x05
MASK_H		.set						0x06
MASK_L		.set						0x07
MASK_SUM	.set						0x08
EDGES		.set						0x09
DELAY		.set						0x0A
VERSION		.set						0x0B
ENC_ID2		.set						0x0D
ENC_ID1		.set						0x0E
ENC_ID0		.set						0x0F
POS4		.set						0x10
POS3		.set						0x11
POS2		.set						0x12
POS1		.set						0x13
POS0		.set						0x14
VEL2		.set						0x15
VEL1		.set						0x16
VEL0		.set						0x17
SUMMARY		.set						0x18
VPOS4		.set						0x19
VPOS3		.set						0x1A
VPOS2		.set						0x1B
VPOS1		.set						0x1C
VPOS0		.set						0x1D
VPOSCRC_H	.set						0x1E
VPOSCRC_L	.set						0x1F
PC_BUFFER0	.set						0x20
PC_BUFFER1	.set						0x21
PC_BUFFER2	.set						0x22
PC_BUFFER3	.set						0x23
PC_BUFFER4	.set						0x24
PC_BUFFER5	.set						0x25
PC_BUFFER6	.set						0x26
PC_BUFFER7	.set						0x27
PC_ADD_H	.set						0x28
PC_ADD_L	.set						0x29
PC_OFF_H	.set						0x2A
PC_OFF_L	.set						0x2B
PC_CTRL		.set						0x2C
PIPE_S		.set						0x2D
PIPE_D		.set						0x2E
PC_DATA		.set						0x2F

SAFE_CTRL	.set						0x35
SAFE_SUM	.set						0x36
S_PC_DATA	.set						0x37

ACC_ERR_CNT	.set						0x38
MAXACC		.set						0x39
MAXDEV_H	.set						0x3A
MAXDEV_L	.set						0x3B

EVENT_S		.set						0x3D
MASK_S		.set						0x3E

DUMMY		.set						0x3F
;extra Master Registers as workaround for different access method depended behaviour (r/w)
SLAVE_REG_CTRL	.set						0x40
ACC_ERR_CNT_TRESH .set						0x41
MAXDEV_TRESH_H	.set						0x42
MAXDEV_TRESH_L	.set						0x43
;extra MASTER registers (not in the specification! little endian!)
NUM_VERT_ERR0	.set						0x90
NUM_VERT_ERR1	.set						0x91
NUM_VERT_ERR2	.set						0x92
NUM_VERT_ERR3	.set						0x93
NUM_VERT_FRAMES0	.set					0x94
NUM_VERT_FRAMES1	.set					0x95
NUM_VERT_FRAMES2	.set					0x96
NUM_VERT_FRAMES3	.set					0x97
REL_POS0	.set						0x98
REL_POS1	.set						0x99
REL_POS2	.set						0x9a
REL_POS3	.set						0x9b
MISMATCH0	.set						0x9c
MISMATCH1	.set						0x9d
MISMATCH2	.set						0x9e
MISMATCH3	.set						0x9f
NUM_H_ERR0	.set						0xa0
NUM_H_ERR1	.set						0xa1
NUM_H_ERR2	.set						0xa2
NUM_H_ERR3	.set						0xa3
LAST_FAST_POS0	.set						0xa4
LAST_FAST_POS1	.set						0xa5
LAST_FAST_POS2	.set						0xa6
LAST_FAST_POS3	.set						0xa7
LAST_FAST_POS4	.set						0xa8
NUM_RESETS	.set						0xa9
EDMA_FLAG	.set						0xaa
TEST_PATTERN0	.set						0xac
TEST_PATTERN1	.set						0xad
TEST_PATTERN2	.set						0xae
TEST_PATTERN3	.set						0xaf
TEST_PATTERN4	.set						0xb0
TEST_PATTERN5	.set						0xb1
TEST_PATTERN6	.set						0xb2
TEST_PATTERN7	.set						0xb3
TEST_PAT_MASK0	.set						0xb4
TEST_PAT_MASK1	.set						0xb5
TEST_PAT_MASK2	.set						0xb6
TEST_PAT_MASK3	.set						0xb7
SYNC_DIFF	.set						0xb8
SYNC_PULSE_PERIOD .set						0xbc
LONG_MSG_BUFFER .set						0xc4
SYNC_PULSE_ERROR .set						0xc0
ECAP_OFFSETS	.set						0xce
	.if !$defined(ICSS_G_V_1_0)                      ;don't need these here in am6 since no edma overlay, lets use it for ext sync pulse support
PIMEM_SRC	.set						0xd8;4bytes
PIMEM_SIZE	.set						0xdc;4bytes
PIMEM_DST	.set						0xe0;4bytes
	.else
EXTRA_EDGE_TIMESTAMP    .set			    0xd8;4bytes
SYNC_PARAM1	            .set				0xdc;4bytes
SYNC_EXTRA_REMAINDER    .set				0xe0;1bytes
SYNC_STUFFING_REMAINDER .set				0xe1;1bytes
WAIT_BEFORE_START       .set				0xe2;2bytes ;free
	.endif
MASK_POS	.set						0xe4;4bytes
SYNC_JITTER	.set						0xe8;2bytes
SYNC_DIFF_HIST_CNT	.set						0xea;2bytes
SYNC_DIFF_HIST	.set						0xec;2*8bytes
DUMMY_END	.set						0xff
;--------------------------------------------------------------------------------------------------
;Memory offsets
;--------------------------------------------------------------------------------------------------
PRU_CTRL_CONST	.set						C28  
;PRU_CTRL	.set						0x00022000
PRU_CYCLCNT	.set						0x0c
PWMSS1_CONST	.set						C19     ;AM437x specific for sync pule synchronization 
PWMSS1_ECAP	.set						0x48302100  ;AM437x specific for sync pule synchronization
PWMSS2_CONST	.set						C20     ;AM437x specific for sync pule synchronization   
ECAP		.set						0x100       ;AM437x specific for sync pule synchronization
ECAP_ECCLR	.set						0x30        ;AM437x specific for sync pule synchronization
ECAP_ECEINT	.set						0x2c        ;AM437x specific for sync pule synchronization
ECAP_CAP1	.set						0x08        ;AM437x specific for sync pule synchronization
ECAP_ECCTL1	.set						0x28        ;AM437x specific for sync pule synchronization
INTC_SICR	.set						0x24        ;AM437x specific for sync pule synchronization
INTC_SECR1	.set						0x284       ;AM437x specific for sync pule synchronization
INTC_SECR0	.set						0x280       ;AM437x specific for sync pule synchronization
INTC_CONST	.set						C0          ;AM437x specific for sync pule synchronization
;EDMA_CONST	.set						C29         ;AM437x specific for sync pule synchronization
EDMA_EESRH	.set						0x34        ;AM437x specific for sync pule synchronization
EDMA_EECRH	.set						0x28        ;AM437x specific for sync pule synchronization
IEP_CONST	.set						C26
IEP1_CONST  .set                        0x2F000
	.if !$defined(ICSS_G_V_1_0)
IEP_CNT		.set						0x0c
	.else
IEP_CNT		.set						0x14 ;todo TSmod check as 64bit now (old0x0c)
	.endif
IEP_CAPR6_RISE .set                     0x50
SHARED_MEM_CONST  .set                  0x0b010000;0x54442000+0x10000
;--------------------------------------------------------------------------------------------------
;Memory offsets for dynamic loading of PRU code
;--------------------------------------------------------------------------------------------------
CODE_BASE	.set						(0x40300000);(0x54400000+0x00040000+0x2100)
CODE_SIZE	.set						0x600
DYNAMIC_CODE_OFFSET .set				0x1000 ;0x2100

;--------------------------------------------------------------------------------------------------
;R31 flags for RX
;--------------------------------------------------------------------------------------------------
	.if $defined(ENDAT_CHANNEL_2)
ENDAT_RX_OVERFLOW_FLAG		      .set				       29
ENDAT_RX_VALID_FLAG               .set                     26
	.endif

	.if $defined(ENDAT_CHANNEL_1)
ENDAT_RX_OVERFLOW_FLAG		      .set					   28
ENDAT_RX_VALID_FLAG               .set                     25
	.endif

	.if $defined(ENDAT_CHANNEL_0)
ENDAT_RX_OVERFLOW_FLAG		      .set					   27
ENDAT_RX_VALID_FLAG               .set                     24
	.endif

;--------------------------------------------------------------------------------------------------
;R30 flags for RX
;--------------------------------------------------------------------------------------------------
	.if $defined(ENDAT_CHANNEL_2)
ENDAT_RX_ENABLE	                  .set					   26
	.endif

	.if $defined(ENDAT_CHANNEL_1)
ENDAT_RX_ENABLE		              .set					   25
	.endif

	.if $defined(ENDAT_CHANNEL_0)
ENDAT_RX_ENABLE			          .set					   24
	.endif

;--------------------------------------------------------------------------------------------------
;R31 flags for TX
;--------------------------------------------------------------------------------------------------

;global flags
ENDAT_TX_GLOABL_TX_GO             .set                     20
ENDAT_TX_GLOABL_REINIT            .set                     19
ENDAT_TX_CHANNEL_GO               .set                     18

	.if $defined(ENDAT_CHANNEL_2)
ENDAT_TX_GLOABL_REINIT_ACTIVE     .set                     21
ENDAT_TX_UNDERRUN                 .set                     17
ENDAT_TX_OVERRUN                  .set                     16
	.endif

	.if $defined(ENDAT_CHANNEL_1)
ENDAT_TX_GLOABL_REINIT_ACTIVE     .set                     13
ENDAT_TX_UNDERRUN                 .set                      9 
ENDAT_TX_OVERRUN                  .set                      8
	.endif

	.if $defined(ENDAT_CHANNEL_0)
ENDAT_TX_GLOABL_REINIT_ACTIVE     .set                      5
ENDAT_TX_UNDERRUN                 .set                      1
ENDAT_TX_OVERRUN                  .set                      0
	.endif

;--------------------------------------------------------------------------------------------------
;R30 flags for TX
;--------------------------------------------------------------------------------------------------

	.if $defined(ENDAT_CHANNEL_2)
ENDAT_CHANNEL_NUM                 .set                     1
	.endif

	.if $defined(ENDAT_CHANNEL_1)
ENDAT_CHANNEL_NUM                 .set                     0
	.endif



;TS: secondary
VERSION2    .set                        0x44  ;TS: adress not correct
ENC2_ID     .set                        0x45
STATUS2     .set                        0x46
VPOS24      .set                        0x47
VPOS23      .set                        0x48
VPOS22      .set                        0x49
VPOS21      .set                        0x4A
VPOS20      .set                        0x4B
VPOSCRC2_H  .set                        0x4C
VPOSCRC2_L  .set                        0x4D
DUMMY2      .set                        0x4E
;TS: end

ONLINE_STATUS_D	.set					0x50
ONLINE_STATUS_1	.set					0x52
ONLINE_STATUS_2	.set					0x54

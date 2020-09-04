
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

;number of cycles for 1 bit
CYCLES_BIT .set							24

;clock divider
CLKDIV_225MHZ .set						0
CLKDIV_FAST .set						2
CLKDIV_FASTER .set						16
CLKDIV_DOUBLE .set						11
CLKDIV_NORMAL .set						23
CLKDIV_HALF .set						47
CLKDIV_SLOWER .set						61;60
CLKDIV_SLOW .set						47;50;47

;ENDAT CFG
ENDAT_CH0CFG0 .set						(0<<16)
ENDAT_CH0CFG1 .set						0

ENDAT_CH2CFG0 .set						((0<<16)| (1<<17))
ENDAT_CH2CFG1 .set						0

;6 bits sync + 2 bits sample
SYNC .set								0x2f
;1 bit sample + 3 bit cycle_reset
CYCLE_RESET .set						0x06
; cycle_reset + equalization in first state
FIRST_CYCLE_EQ .set						((0x06<<4) | 0x0a)
;8 bits trailer
TRAILER .set							0x03
;line delay is 12 bits
LINE_DELAY .set							12

;m_par symbols
M_PAR_IDLE .set							0x10
M_PAR_SYNC .set							0x19
M_PAR_START .set						0x1a
M_PAR_LEARN .set						0x1b
M_PAR_CHECK .set						0x1c
M_PAR_IDREQ .set						0x1d
M_PAR_INIT .set							0x1e
M_PAR_RESET .set						0x1f
M_PAR_0x1 .set							0x01
M_PAR_0x2 .set							0x02
M_PAR_0x3 .set							0x03
M_PAR_0x4 .set							0x04
M_PAR_0x5 .set							0x05
M_PAR_0x6 .set							0x06
M_PAR_0x7 .set							0x07
M_PAR_0x8 .set							0x08
M_PAR_0x9 .set							0x09
M_PAR_0xa .set							0x0a
M_PAR_0xb .set							0x0b
M_PAR_0xc .set							0x0c
M_PAR_0xd .set							0x0d
M_PAR_0xe .set							0x0e
M_PAR_0xf .set							0x0f

;s_par symbols
S_PAR_IDLE .set							0x10
S_PAR_SNAK .set							0x15
S_PAR_LNAK .set							0x1a
S_PAR_INIT .set							0x1f

;datalink parameters
LINK_TIMEOUT .set						((DELAY_BITS-3)*CYCLES_BIT)
DELAY_BITS .set							12

;FLAGS
FLAG_NORMAL_FLOW .set						1
FLAG_MSYNC .set							2
FLAG_ERR_ACC .set						3
FLAG_ERR_VERT .set						4
FLAG_ERR_SEC .set						5
UPDATE_FPOS_WITH_SPOS .set					6
MPAR_BUSY .set							7
FLAG_DRIVE_SYNC .set						8
FLAG_PARA_BUSY .set						9
FLAG_WAIT_IDLE .set						10

FLAG_ERRORS .set						((~((1<<FLAG_ERR_VERT) | (1<<FLAG_ERR_SEC)))&0xff)


;Fixed values here
NUM_MT_BITS .set						12

;Masks
MASK_SPEED .set							0x00FFFFFF
;Special Characters

K28_0 .set							0x1c
K28_1 .set							0x3c
K28_2 .set							0x5c
K28_3 .set							0x7c
K28_4 .set							0x9c
K28_5 .set							0xbc
K28_6 .set							0xdc
K28_7 .set							0xfc
K23_7 .set							0xf7
K27_7 .set							0xfb
K30_7 .set							0xfe
K29_7 .set							0xFD
DOUBLE_K29_7 .set						(K29_7 << 8) | K29_7

;Number of V-Frames the average error should be calculated
NUM_FRAMES_AVG .set						10

;Byte number of channel in slave answer
BYTE_CH_VERTICAL .set						0
BYTE_CH_PARAMETER .set						1
BYTE_CH_PIPE .set						2
BYTE_CH_ACC_H .set						3
BYTE_CH_ACC_L .set						4
BYTE_CH_SECONDARY .set						5

;int number of sync pulse event
INT_SYNC_PULSE .set						35
INT_SAMPLE_EDGE .set						36

;size of fastpos
SIZE_FAST_POS .set						5

;
RSSI_MAX .set							12
QM_MAX .set							15

;Message constants
MSG_SHORT .set							0
MSG_LONG .set							1
MSG_WRITE .set							0
MSG_READ .set							1
;Message flags
FLAG_ACK .set							7
LOFF .set							5


PRU_ARM_EVENT0 .set						(18 - 16) ;SYS_EVENT_18 is pr1_pru_mst_intr<2>_int_req
PRU0_ARM_IRQ .set						(0x20 | PRU_ARM_EVENT0)

PRU_ARM_EVENT1 .set						(19 - 16) ;SYS_EVENT_19 is pr1_pru_mst_intr<3>_int_req
PRU0_ARM_IRQ1 .set						(0x20 | PRU_ARM_EVENT1)

PRU_ARM_EVENT2 .set						(20 - 16) ;SYS_EVENT_20 is pr1_pru_mst_intr<4>_int_req
PRU0_ARM_IRQ2 .set						(0x20 | PRU_ARM_EVENT2)

PRU_ARM_EVENT3 .set						(21 - 16) ;SYS_EVENT_21 is pr1_pru_mst_intr<5>_int_req
PRU0_ARM_IRQ3 .set						(0x20 | PRU_ARM_EVENT3)

;constants for evnet_h/l (event_h is saved in low part of register when read from memory!)
;event_h
EVENT_SUM .set							6
EVENT_SCE .set							5
EVENT_POS .set							3
EVENT_VPOS .set							2
EVENT_DTE .set							1
EVENT_PRST .set							0

;event_l
EVENT_MIN .set							(5+8)
EVENT_ANS .set							(4+8)
EVENT_QMLW .set							(2+8)
EVENT_FREL .set							(1+8)
EVENT_FRES .set							(0+8)


;SYS_CTRL constants
SYS_CTRL_PRST .set						7
SYS_CTRL_MRST .set						6
SYS_CTRL_FRST .set						5
SYS_CTRL_LOOP .set						4
SYS_CTRL_PRDY .set						3
SYS_CTRL_SPPE .set						2
SYS_CTRL_SPOL .set						1
SYS_CTRL_OEN .set						0


;code section definitions
CODE_DATALINK_INIT .set						0
CODE_TRANSPORT_LAYER .set					1


;STUFFING is at max. 26*6 bits
MAX_STUFFING .set						26

;Num. of LOADFW states (CODE_SIZE/NUM_LOADFW = size of code per load)
NUM_LOADFW .set							1

;MAX cycle differene allowed between the SYNC pulse and extra edge
MAX_ALLOWED_CYCLE_DIFF .set             10000

;MAX cycle differene allowed between the SYNC pulse and extra edge
CAPPING_CYCLE_DIFF .set             18
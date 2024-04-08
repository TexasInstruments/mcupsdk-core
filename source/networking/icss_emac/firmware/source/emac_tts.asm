;
;  TEXAS INSTRUMENTS TEXT FILE LICENSE
;
;   Copyright (c) 2024 Texas Instruments Incorporated
;
;  All rights reserved not granted herein.
;
;  Limited License.
;
;  Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
;  license under copyrights and patents it now or hereafter owns or controls to
;  make, have made, use, import, offer to sell and sell ("Utilize") this software
;  subject to the terms herein.  With respect to the foregoing patent license,
;  such license is granted  solely to the extent that any such patent is necessary
;  to Utilize the software alone.  The patent license shall not apply to any
;  combinations which include this software, other than combinations with devices
;  manufactured by or for TI (“TI Devices”).  No hardware patent is licensed hereunder.
;
;  Redistributions must preserve existing copyright notices and reproduce this license
;  (including the above copyright notice and the disclaimer and (if applicable) source
;  code license limitations below) in the documentation and/or other materials provided
;  with the distribution.
;
;  Redistribution and use in binary form, without modification, are permitted provided
;  that the following conditions are met:
; 	No reverse engineering, decompilation, or disassembly of this software is
;   permitted with respect to any software provided in binary form.
; 	Any redistribution and use are licensed by TI for use only with TI Devices.
; 	Nothing shall obligate TI to provide you with source code for the software
;   licensed and provided to you in object code.
;
;  If software source code is provided to you, modification and redistribution of the
;  source code are permitted provided that the following conditions are met:
; 	Any redistribution and use of the source code, including any resulting derivative
;   works, are licensed by TI for use only with TI Devices.
; 	Any redistribution and use of any object code compiled from the source code
;   and any resulting derivative works, are licensed by TI for use only with TI Devices.
;
;  Neither the name of Texas Instruments Incorporated nor the names of its suppliers
;  may be used to endorse or promote products derived from this software without
;  specific prior written permission.
;
;  DISCLAIMER.
;
;  THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED
;  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
;  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S
;  LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
;  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
;  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
; file:   emac_tts.asm
;
; brief:  ICSS EMAC time triggered send
;         Includes:
;         1. TTS Functions
;
;
;  (C) Copyright 2017, Texas Instruments, Inc
;
;

	.if !$defined("__emac_tts_p")
__emac_tts_p	.set	1

        .include "firmware_version.h"
	.include "micro_scheduler.h"
	.include "icss_emacSwitch.h"
	.include "emac_MII_Rcv.h"
	.include "emac_MII_Xmt.h"
	.include "icss_defines.h"
	.include "icss_miirt_regs.h"
	.include "icss_iep_regs.h"
	.include "icss_macros.h"
	.include "emac_tts.h"

        .global  START_XMT_QUEUE
        .global  RET_TTS_IEP_CFG_CLEAR
        .global  RET_TTS_IEP_CFG_PRE
        .global  TASK_EXECUTION_FINISHED
        .global  CHECK_NEXT_QUEUE
        .global  FN_TTS_IEP_CFG_CLEAR
        .global  FN_TTS_IEP_CFG_PRE_ICSS_REV2
        .global  FN_TTS_PKT_SIZE_CHECK_ICSS_REV2
        .global  FN_TTS_IEP_CMPCFG_ARBITRATION
        .global  FN_TTS_IEP_CFG_PRE_ICSS_REV1
        .global  FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION
        .global  FN_TTS_PKT_SIZE_CHECK_ICSS_REV1

        .if $defined("TTS")

	.if $defined("ICSS_REV2")
;******************************************************************************
;
;		NAME				: FN_TTS_IEP_CFG_PRE_ICSS_REV2
;		DESCRIPTION			: Configures IEP Compare 3/4/5/6 for Time Triggered Send.
;		RETURNS				: None
;		ARGS				: None
;		ARGS PASSED BY		        : None
;		REGISTERS USED
;		-reserved			: None
;		-temporary			: R0, R2, R3, R4, R5, R6, R20, R21
;		INVOKES 			: None
;
;******************************************************************************
FN_TTS_IEP_CFG_PRE_ICSS_REV2:
	;Check if no packet was transmitted in last cycle.
	;Check if RT packet missed previous cycle.
	QBBC	TTS_FIRST_SETUP_not_done_ICSS_REV2, R22 , TTS_FIRST_SETUP
	M_TTS_MISSED_CYCLE_CHECK
TTS_FIRST_SETUP_not_done_ICSS_REV2:

	;Clear bit on every cfg.
	CLR	R22 , R22 , TTS_FIRST_PKT_done

	;Disable compare 3/4/5/6 events. Only for first time.
	QBBS	TTS_DISABLE_CMP_done_ICSS_REV2, R22 , TTS_FIRST_SETUP

	;Enter critical section.
	JAL	R5.w0, FN_TTS_IEP_CMPCFG_ARBITRATION

	LBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1

	.if $defined("PRU0")
	AND	R6.b0, R6.b0, (TTS_CMP3_DISABLE & TTS_CMP5_DISABLE)	;Disable CMP3 and CMP5.
	.else	;PRU1
	AND	R6.b0, R6.b0, (TTS_CMP4_DISABLE & TTS_CMP6_DISABLE)	;Disable CMP4 and CMP6.
	.endif
	SBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1

	; Exit the critical section
	JAL	R5.w0, FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION
TTS_DISABLE_CMP_done_ICSS_REV2:

	;Clear compare 3/4/5/6 status.
	.if $defined("PRU0")
	LDI	R6.b0 , (TTS_CMP3_CLEAR_STATUS | TTS_CMP5_CLEAR_STATUS);Clear CMP3 and CMP5 status.
	.else	;PRU1
	LDI	R6.b0 , (TTS_CMP4_CLEAR_STATUS | TTS_CMP6_CLEAR_STATUS);Clear CMP4 and CMP6 status.
	.endif
	SBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1

	;Reading cycle start into temp 3/temp 4 registers.
	LDI	TEMP_REG_2.w0 , ICSS_EMAC_TTS_CYCLE_START_OFFSET
	LBCO	&TEMP_REG_3, PRU_DMEM_ADDR, TEMP_REG_2.w0, 8

	;Set Compare 3/4 Registers.
	.if $defined("PRU0")
	SBCO	&TEMP_REG_3, IEP_CONST, ICSS_IEP_CMP3_REG, 8
	.else	;PRU1
	SBCO	&TEMP_REG_3, IEP_CONST, ICSS_IEP_CMP4_REG, 8
	.endif

	;Reading cycle period into temp 2 register.
	LDI	TEMP_REG_2.w0 , ICSS_EMAC_TTS_CYCLE_PERIOD_OFFSET
	LBCO	&TEMP_REG_2, PRU_DMEM_ADDR, TEMP_REG_2.w0, 4

	;Calculating config start time, STEP 1: cycle start + cycle period
	;Storing in temp 3 and temp 4 registers.
	ADD	TEMP_REG_3, TEMP_REG_3, TEMP_REG_2
	ADC	TEMP_REG_4, TEMP_REG_4, 0

	; ^^ cycle start (new) = cycle start (old) + cycle period
	;Storing above calculated time as updated cycle start time to memory.
	LDI	TEMP_REG_2.w0 , ICSS_EMAC_TTS_CYCLE_START_OFFSET
	SBCO	&TEMP_REG_3, PRU_DMEM_ADDR, TEMP_REG_2.w0, 8

	;Reading config time into temp 2 register.
	LDI	TEMP_REG_2.w0 , ICSS_EMAC_TTS_CFG_TIME_OFFSET
	LBCO	&TEMP_REG_2, PRU_DMEM_ADDR, TEMP_REG_2.w0, 4

	;Calculating config start time, STEP 2:
	;config start time = above result(cycle start + cycle period) - config time
	;Storing into temp 3/temp 4 registers.
	SUB	TEMP_REG_3, TEMP_REG_3, TEMP_REG_2
	SUC	TEMP_REG_4, TEMP_REG_4, 0

	;Set Compare 5/6 Reg.
	.if $defined("PRU0")
	SBCO	&TEMP_REG_3, IEP_CONST, ICSS_IEP_CMP5_REG, 8
	.else	;PRU1
	SBCO	&TEMP_REG_3, IEP_CONST, ICSS_IEP_CMP6_REG, 8
	.endif

	;Enable compare 3/4/5/6 events.
	;Enter the critical section.
	JAL	R5.w0, FN_TTS_IEP_CMPCFG_ARBITRATION

	LBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1
	.if $defined("PRU0")
	OR	R6.b0, R6.b0, (TTS_CMP3_ENABLE | TTS_CMP5_ENABLE)	;Enable CMP3 and CMP5
	.else	;PRU1
	OR	R6.b0, R6.b0, (TTS_CMP4_ENABLE | TTS_CMP6_ENABLE)	;Enable CMP4 and CMP6
	.endif
	SBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1

	; Exit the critical section
	JAL	R5.w0, FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION

	;Enable TX_EN_MODE
	.if $defined("PRU0")
	LDI	TEMP_REG_3.w0 , ICSS_MIIRT_TXCFG0
	.else	;PRU1
	LDI	TEMP_REG_3.w0 , ICSS_MIIRT_TXCFG1
	.endif
	LBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TEMP_REG_3.w0, 1
	OR	TEMP_REG_1.b0, TEMP_REG_1.b0, TX_EN_MODE
	SBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TEMP_REG_3.w0, 1

	SET	R22 , R22 , TTS_FIRST_SETUP
	JMP		RET_TTS_IEP_CFG_PRE
	.endif	;ICSS_REV2

	.if $defined("ICSS_REV1")
;******************************************************************************
;
;		NAME				: FN_TTS_IEP_CFG_PRE_ICSS_REV1
;		DESCRIPTION			: Configures IEP Compare 3/4/5/6 for Time Triggered Send.
;		RETURNS				: None
;		ARGS				: None
;		ARGS PASSED BY		: None
;		REGISTERS USED
;		-reserved			: None
;		-temporary			: R0, R2, R3, R4, R5, R6, R20, R21
;		INVOKES 			: None
;
;******************************************************************************
FN_TTS_IEP_CFG_PRE_ICSS_REV1:
	;Check if no packet was transmitted in last cycle.
	;Check if RT packet missed previous cycle.
	QBBC	TTS_FIRST_SETUP_not_done_ICSS_REV1, R22 , TTS_FIRST_SETUP
	M_TTS_MISSED_CYCLE_CHECK
TTS_FIRST_SETUP_not_done_ICSS_REV1:

	;Clear bit on every cfg.
	CLR	R22 , R22 , TTS_FIRST_PKT_done

	;Disable compare 3/4/5/6 events. Only for first time because
	;the compare event gets triggered under the following 2 circumstances:
	;1. The counter crossesover the compare value.
	;2. The cmp event is toggled disabled-to-enabled and cmp value<counter val.
	;If we don't disable the cmp event before programming for the first time,
	;the compare event might get triggered (if it was previously enabled)
	;as soon as we program the compare value and if cmp val<counter val,
	;even though we wanted it to get triggered in the next cycle after wraparound.
	QBBS	TTS_DISABLE_CMP_done_ICSS_REV1, R22 , TTS_FIRST_SETUP

	;Enter critical section.
	JAL	R5.w0, FN_TTS_IEP_CMPCFG_ARBITRATION

	LBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1

	.if $defined("PRU0")
	AND	R6.b0, R6.b0, (TTS_CMP3_DISABLE & TTS_CMP5_DISABLE)	;Disable CMP3 and CMP5.
	.else	;PRU1
	AND	R6.b0, R6.b0, (TTS_CMP4_DISABLE & TTS_CMP6_DISABLE)	;Disable CMP4 and CMP6.
	.endif
	SBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1

	; Exit the critical section
	JAL	R5.w0, FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION
TTS_DISABLE_CMP_done_ICSS_REV1:

	;Clear compare 3/4/5/6 status.
	.if $defined("PRU0")
	LDI	R6.b0 , (TTS_CMP3_CLEAR_STATUS | TTS_CMP5_CLEAR_STATUS);Clear CMP3 and CMP5 status.
	.else	;PRU1
	LDI	R6.b0 , (TTS_CMP4_CLEAR_STATUS | TTS_CMP6_CLEAR_STATUS);Clear CMP4 and CMP6 status.
	.endif
	SBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1

	;Reading cycle start into temp 1 register.
	LDI	TEMP_REG_4.w0 , ICSS_EMAC_TTS_CYCLE_START_OFFSET
	LBCO	&TEMP_REG_1, PRU_DMEM_ADDR, TEMP_REG_4.w0, 4

	;Need to check if the cycle start value is greater than 1s.
	;This might not be required for first time as it is handled in driver.
	LDI	TEMP_REG_4.w0 , ICSS_EMAC_TTS_IEP_MAX_VAL & 0xFFFF
	LDI	TEMP_REG_4.w2 , ICSS_EMAC_TTS_IEP_MAX_VAL >> 16
	QBLE	TTS_CYCLE_START_LT_1S_ICSS_REV1, TEMP_REG_4, TEMP_REG_1
	SUB	TEMP_REG_1, TEMP_REG_1, TEMP_REG_4
TTS_CYCLE_START_LT_1S_ICSS_REV1:

	;Check if CMP event is being set to 0.
	;This might not be required for first time as it is handled in driver.
	QBNE	TTS_CMP34_NOT_ZERO_ICSS_REV1, TEMP_REG_1, 0
	LDI	TEMP_REG_1 , 1
TTS_CMP34_NOT_ZERO_ICSS_REV1:

	;Set Compare 3/4 Registers.
	.if $defined("PRU0")
	SBCO	&TEMP_REG_1, IEP_CONST, ICSS_IEP_CMP3_REG, 4
	.else	;PRU1
	SBCO	&TEMP_REG_1, IEP_CONST, ICSS_IEP_CMP4_REG, 4
	.endif

	;Reading cycle period into temp 3 register.
	LDI	TEMP_REG_4.w0 , ICSS_EMAC_TTS_CYCLE_PERIOD_OFFSET
	LBCO	&TEMP_REG_3, PRU_DMEM_ADDR, TEMP_REG_4.w0, 4

	;Calculating config start time, STEP 1: cycle start + cycle period
	;Storing in TEMP_REG_3 register.
	ADD	TEMP_REG_3, TEMP_REG_1, TEMP_REG_3

	; ^^ cycle start (new) = cycle start (old) + cycle period
	;Storing above calculated time as updated cycle start time to memory.
	LDI	TEMP_REG_4.w0 , ICSS_EMAC_TTS_CYCLE_START_OFFSET
	SBCO	&TEMP_REG_3, PRU_DMEM_ADDR, TEMP_REG_4.w0, 4

	;Reading config time into R21 register.
	LDI	TEMP_REG_4.w0 , ICSS_EMAC_TTS_CFG_TIME_OFFSET
	LBCO	&R21, PRU_DMEM_ADDR, TEMP_REG_4.w0, 4

	;Calculating config start time, STEP 2:
	;config start time = above result(cycle start + cycle period) - config time
	;Storing into TEMP_REG_3 register.
	SUB	TEMP_REG_3, TEMP_REG_3, R21

	;Need to check if the config start value is greater than 1s.
	LDI	TEMP_REG_4.w0 , ICSS_EMAC_TTS_IEP_MAX_VAL & 0xFFFF
	LDI	TEMP_REG_4.w2 , ICSS_EMAC_TTS_IEP_MAX_VAL >> 16
	QBLE	TTS_CONFIG_START_LT_1S_ICSS_REV1, TEMP_REG_4, TEMP_REG_3
	SUB	TEMP_REG_3, TEMP_REG_3, TEMP_REG_4
TTS_CONFIG_START_LT_1S_ICSS_REV1:

	;Check if CMP event is being set to 0.
	QBNE	TTS_CMP56_NOT_ZERO_ICSS_REV1, TEMP_REG_3, 0
	LDI	TEMP_REG_3 , 1
TTS_CMP56_NOT_ZERO_ICSS_REV1:

	;Set Compare 5/6 Reg.
	.if $defined("PRU0")
	SBCO	&TEMP_REG_3, IEP_CONST, ICSS_IEP_CMP5_REG, 4
	.else	;PRU1
	SBCO	&TEMP_REG_3, IEP_CONST, ICSS_IEP_CMP6_REG, 4
	.endif

	;Copying current cycle_start from TEMP_REG_1 to TEMP_REG_2.
	;TEMP_REG_1 get overwritten when jumping to FN_TTS_IEP_CMPCFG_ARBITRATION.
	AND TEMP_REG_2 , TEMP_REG_1 , TEMP_REG_1

	;Enable compare 3/4/5/6 events.
	;Enter the critical section.
	JAL	R5.w0, FN_TTS_IEP_CMPCFG_ARBITRATION

	;Read compare cfg register.
	LBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1

	;Reading current counter value.
	LBCO	&TEMP_REG_4, IEP_CONST, ICSS_IEP_COUNT_REG, 4

	;Need to check if CMP3/4/5/6 values are less than current counter before enabling.
	;Only for first time because the compare event gets triggered
	;under the following 2 circumstances:
	;1. The counter crossesover the compare value.
	;2. The cmp event is toggled disabled-to-enabled and cmp value<counter val.
	;The compare event might get triggered as soon as we program the compare value
	;and if cmp val<counter val, due to the cmp event disabled-to-enabled toggle,
	;even though we wanted it to get triggered in the next cycle after wraparound.
	QBBS	TTS_CMP_CHECK_done_ICSS_REV1, R22 , TTS_FIRST_SETUP

	;Checking if Counter is greater than CMP5/6 value.
	QBGE	Counter_Greater_Than_CMP5_6_ICSS_REV1, TEMP_REG_3, TEMP_REG_4
	.if $defined("PRU0")
	OR	R6.b0, R6.b0, TTS_CMP5_ENABLE	;Enable CMP5.
	.else	;PRU1
	OR	R6.b0, R6.b0, TTS_CMP6_ENABLE	;Enable CMP6.
	.endif
;Counter is greater than next config start value.
;Will have to wait for IEP wraparound for enabling.
Counter_Greater_Than_CMP5_6_ICSS_REV1:

	;Checking if Counter is greater than CMP3/4 value.
	QBGE	Counter_Greater_Than_CMP3_4_ICSS_REV1, TEMP_REG_2, TEMP_REG_4
	.if $defined("PRU0")
	OR	R6.b0, R6.b0, TTS_CMP3_ENABLE	;Enable CMP3.
	.else	;PRU1
	OR	R6.b0, R6.b0, TTS_CMP4_ENABLE	;Enable CMP4.
	.endif
;Counter is greater than next cycle start value.
;Will have to wait for IEP wraparound for enabling.
Counter_Greater_Than_CMP3_4_ICSS_REV1:
	SET	R22 , R22 , TTS_CMP_3456_FIRST_Exception
TTS_CMP_CHECK_done_ICSS_REV1:

	;Need to check if CMP5/6 value is in next cycle.
	QBLT	Counter_Less_Than_CMP5_6_ICSS_REV1, TEMP_REG_3, TEMP_REG_4
	SET	R22 , R22 , TTS_CMP5_CMP6_in_next_IEP_cycle

Counter_Less_Than_CMP5_6_ICSS_REV1:
	;Write to config register.
	SBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1

	; Exit the critical section
	JAL	R5.w0, FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION

	;Enable TX_EN_MODE
	.if $defined("PRU0")
	LDI	TEMP_REG_3.w0 , ICSS_MIIRT_TXCFG0
	.else	;PRU1
	LDI	TEMP_REG_3.w0 , ICSS_MIIRT_TXCFG1
	.endif
	LBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TEMP_REG_3.w0, 1
	OR	TEMP_REG_1.b0, TEMP_REG_1.b0, TX_EN_MODE
	SBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TEMP_REG_3.w0, 1

	SET	R22 , R22 , TTS_FIRST_SETUP
	JMP		RET_TTS_IEP_CFG_PRE
	.endif	;ICSS_REV1

;;////////////////////////////////////////////////////////////////////////////
;******************************************************************************
;
;		NAME				: FN_TTS_IEP_CFG_CLEAR
;		DESCRIPTION			: Clears IEP CFG and IEP CMP status bits if TTS is disabled.
;		RETURNS				: None
;		ARGS				: None
;		ARGS PASSED BY		: None
;		REGISTERS USED
;		-reserved			: R22
;		-temporary			: R0, R2, R5, R6, R20, R21
;		INVOKES 			: None
;
;******************************************************************************
FN_TTS_IEP_CFG_CLEAR:
	QBBS	RET_TTS_IEP_CFG_SET, R22 , TTS_ENABLE
        JMP     RET_TTS_IEP_CFG_CLEAR
	;Clear all TTS R22 bits.
;	CLR		TTS_ENABLE
;	CLR		TTS_FIRST_SETUP
;	CLR		TTS_RT_QUEUE_PKT
;	CLR		TTS_FIRST_PKT_done
;	CLR		TTS_FIRST_PKT_since_enable
;	CLR		TTS_CMP_3456_FIRST_Exception
;	CLR		TTS_CMP5_CMP6_in_next_IEP_cycle
RET_TTS_IEP_CFG_SET:
	AND	R22.b1, R22.b1, TTS_R22_CLEAR_BITS_b1
	AND	R22.b2, R22.b2, TTS_R22_CLEAR_BITS_b2

	;Enter the critical section.
	JAL	R5.w0, FN_TTS_IEP_CMPCFG_ARBITRATION

	;Disable compare 3/4/5/6 (ICSS_REV2) or 3/4/5/6/7 (ICSS_REV1) events.
	LBCO	&R6.w0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 2
	.if $defined("PRU0")
	AND	R6.b0, R6.b0, (TTS_CMP3_DISABLE & TTS_CMP5_DISABLE)
	.else	;PRU1
	AND	R6.b0, R6.b0, (TTS_CMP4_DISABLE & TTS_CMP6_DISABLE)
	.if $defined("ICSS_REV1")
	AND	R6.b1, R6.b1, (TTS_CMP7_DISABLE)	;Disable CMP7
	.endif
	.endif
	SBCO	&R6.w0, IEP_CONST, ICSS_IEP_CMP_CFG_REG, 2

	;Clear compare 3/4/5/6 (ICSS_REV2) or 3/4/5/6/7 (ICSS_REV1) status.
	.if $defined("PRU0")
	LDI	R6.b0 , (TTS_CMP3_CLEAR_STATUS | TTS_CMP5_CLEAR_STATUS)
	.else	;PRU1
	.if $defined("ICSS_REV1")
	LDI	R6.b0 , (TTS_CMP4_CLEAR_STATUS | TTS_CMP6_CLEAR_STATUS | TTS_CMP7_CLEAR_STATUS)
	.endif
	.if $defined("ICSS_REV2")
	LDI	R6.b0 , (TTS_CMP4_CLEAR_STATUS | TTS_CMP6_CLEAR_STATUS)
	.endif
	.endif
	SBCO	&R6.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1

	; Exit the critical section
	JAL	R5.w0, FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION

	;Clear compare 3/4/5/6 reg (ICSS_REV2) or 3/4/5/6/7 reg (ICSS_REV1).
	ZERO	&R5, 8

	.if $defined("ICSS_REV1")
	.if $defined("PRU0")
	SBCO	&R5, IEP_CONST, ICSS_IEP_CMP3_REG, 4
	SBCO	&R5, IEP_CONST, ICSS_IEP_CMP5_REG, 4
	.else	;PRU1
	SBCO	&R5, IEP_CONST, ICSS_IEP_CMP4_REG, 4
	SBCO	&R5, IEP_CONST, ICSS_IEP_CMP6_REG, 8	;This will clear CMP6 and CMP7.
	.endif
	.endif	;ICSS_REV1

	.if $defined("ICSS_REV2")
	.if $defined("PRU0")
	SBCO	&R5, IEP_CONST, ICSS_IEP_CMP3_REG, 8
	SBCO	&R5, IEP_CONST, ICSS_IEP_CMP5_REG, 8
	.else	;PRU1
	SBCO	&R5, IEP_CONST, ICSS_IEP_CMP4_REG, 8
	SBCO	&R5, IEP_CONST, ICSS_IEP_CMP6_REG, 8
	.endif
	.endif	;ICSS_REV2

	;Disable TX_EN_MODE
	.if $defined("PRU0")
	LDI	TEMP_REG_3.w0 , ICSS_MIIRT_TXCFG0
	.else	;PRU1
	LDI	TEMP_REG_3.w0 , ICSS_MIIRT_TXCFG1
	.endif
	LBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TEMP_REG_3.w0, 1
	AND	TEMP_REG_1.b0, TEMP_REG_1.b0, (0xFF ^ TX_EN_MODE)
	SBCO	&TEMP_REG_1.b0, MII_RT_CFG_CONST, TEMP_REG_3.w0, 1

	JMP		RET_TTS_IEP_CFG_CLEAR

	.if $defined("ICSS_REV2")
;;////////////////////////////////////////////////////////////////////////////
;******************************************************************************
;
;		NAME				: FN_TTS_PKT_SIZE_CHECK_ICSS_REV2
;		DESCRIPTION			: Checks if packet size is within permissible range.
;		RETURNS				: none
;		ARGS				: R2, R3
;		ARGS PASSED BY		: FN_XMT_scheduler
;		REGISTERS USED
;		-reserved			: R11, R14.w2, R15.w0,
;		-temporary			: R0, R5, R6, R20, R21
;		INVOKES 			: START_XMT_QUEUE or CHECK_NEXT_QUEUE or TASK_EXECUTION_FINISHED
;
;******************************************************************************
FN_TTS_PKT_SIZE_CHECK_ICSS_REV2:

	AND BUFFER_DESC_OFFSET , QUEUE_DESC_REG.rd_ptr , QUEUE_DESC_REG.rd_ptr	;Warning: converted from MOV
	LBCO	&BUFFER_DESC_REG, PRU_DMEM_ADDR, BUFFER_DESC_OFFSET, 4
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;	LSL	Packet_Length, BUFFER_DESC_REG.Length, 3	;bit 18...28
	LSL	Packet_Length, BUFFER_DESC_REG.pkt_length, 3	;bit 18...28
        LSR	Packet_Length, Packet_Length, 5

	;Multiplying Packet length by 80 to calculate time. (For 100Mbps)
	;TODO: Add support for 10Mbps here.
	;Add 12. 8 bytes of Preamble + 4 bytes of CRC
	ADD	R21, Packet_Length, 12
	; x*80 ns = x*64 + x*16
	LSL	R5, R21, 6
	LSL	R21, R21, 4
	ADD	R21, R5, R21
	AND TEMP_REG_1 , R21 , R21
	;TEMP_REG_1 now has time required to transmit packet.

	;Reading current counter value.
	LBCO	&R20, IEP_CONST, ICSS_IEP_COUNT_REG, 8

	.if $defined("PRU0")
	;Reading Compare 5/6 time.
	LBCO	&R5, IEP_CONST, ICSS_IEP_CMP5_REG, 8
	.else	;PRU1
	LBCO	&R5, IEP_CONST, ICSS_IEP_CMP6_REG, 8
	.endif

	;Checking if the counter has crossed comapre 5/6 value.
	;If counter is greater than compare 5/6 value,
	;it means compare event occured while somewhere in XMT_SCHEDULER.
	;But the subtraction below will still give positive result due to unsigned subtraction.
	;Hence we cannot proceed further.
	QBLT	COUNTER_LT_COMPARE_5_6_ICSS_REV2, R6, R21
        QBLE	CONTINUE_V1, R6, R21
        JMP     TASK_EXECUTION_FINISHED
CONTINUE_V1:
        QBLT	COUNTER_LT_COMPARE_5_6_ICSS_REV2, R5, R20
        JMP     TASK_EXECUTION_FINISHED

COUNTER_LT_COMPARE_5_6_ICSS_REV2:

	;Subtracting (Compare 5/6 time - Counter Value)
	SUB	R20, R5, R20
	SUC	R21, R6, R21

	;Comparing above value with required transmit time.
	QBGE	SIZE_ALLOWED_local_ICSS_REV2, TEMP_REG_1, R20
	QBLE	SIZE_ALLOWED_local_ICSS_REV2, R21, 1

	;Size not allowed.
        QBBS	CONTINUE_V2, R22 , TTS_RT_QUEUE_PKT
        JMP     TASK_EXECUTION_FINISHED
CONTINUE_V2:
        LDI	R20.w0, Q4_TX_CONTEXT_OFFSET
	JMP 	CHECK_NEXT_QUEUE

SIZE_ALLOWED_local_ICSS_REV2:
	QBBC	NOT_Queue1_SA_local_ICSS_REV2, R22 , TTS_RT_QUEUE_PKT
	SET	R22 , R22 , TTS_FIRST_PKT_done
	SET	R22 , R22 , TTS_FIRST_PKT_since_enable
	JMP		START_XMT_QUEUE
NOT_Queue1_SA_local_ICSS_REV2:
	;Check if RT packet missed cycle, in case NRT packet was the first packet in the cycle.
	M_TTS_MISSED_CYCLE_CHECK
	SET	R22 , R22 , TTS_FIRST_PKT_done
	SET	R22 , R22 , TTS_FIRST_PKT_since_enable
	JMP		START_XMT_QUEUE
	.endif	;ICSS_REV2

	.if $defined("ICSS_REV1")
;;////////////////////////////////////////////////////////////////////////////
;******************************************************************************
;
;		NAME				: FN_TTS_PKT_SIZE_CHECK_ICSS_REV1
;		DESCRIPTION			: Checks if packet size is within permissible range.
;		RETURNS				: none
;		ARGS				: R2, R3
;		ARGS PASSED BY		: FN_XMT_scheduler
;		REGISTERS USED
;		-reserved			: R11, R14.w2, R15.w0,
;		-temporary			: R0, R5, R6, R20, R21
;		INVOKES 			: START_XMT_QUEUE or CHECK_NEXT_QUEUE or TASK_EXECUTION_FINISHED
;
;******************************************************************************
FN_TTS_PKT_SIZE_CHECK_ICSS_REV1:

	AND BUFFER_DESC_OFFSET , QUEUE_DESC_REG.rd_ptr , QUEUE_DESC_REG.rd_ptr	;Warning: converted from MOV
	LBCO	&BUFFER_DESC_REG, PRU_DMEM_ADDR, BUFFER_DESC_OFFSET, 4
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;	LSL	Packet_Length, BUFFER_DESC_REG.Length, 3	;bit 18...28
	LSL	Packet_Length, BUFFER_DESC_REG.pkt_length, 3	;bit 18...28
        LSR	Packet_Length, Packet_Length, 5

	;Multiplying Packet length by 80 to calculate time. (For 100Mbps)
	;TODO: Add support for 10Mbps here.
	;Add 12. 8 bytes of Preamble + 4 bytes of CRC
	ADD	R21, Packet_Length, 12
	; x*80 ns = x*64 + x*16
	LSL	R5, R21, 6
	LSL	R21, R21, 4
	ADD	R21, R5, R21
	AND TEMP_REG_1 , R21 , R21
	;TEMP_REG_1 now has time required to transmit packet.

	;Need to check if CMP5/6 event has already occured.
	;Counter can be > CM5/6 values in two situations:
	;1. If CMP5/6 value is in next IEP cycle.
	;2. If CMP5/6 event has occured since previous check.
	;	 i.e., it has probably occured while flow was somewhere in XMT_SCHEDULER.
	;We have to avoid transmitting in situation 2.
	;There is another corner condition in which the CMP5/6 value was set to a value just after wraparound.
	;In that case, it is possible that CMP5/6 has not been enabled yet and we will not be able to get correct status.
	;This is if we enter XMT_SCHEDULER before wraparound, and then wraparound occurs in background.
	;Thus we also check for CMP0/7 event.

	;Reading current counter value.
	LBCO	&R20, IEP_CONST, ICSS_IEP_COUNT_REG, 4
	.if $defined("PRU0")
	;Reading Compare 5/6 time.
	LBCO	&R5, IEP_CONST, ICSS_IEP_CMP5_REG, 4
	.else	;PRU1
	LBCO	&R5, IEP_CONST, ICSS_IEP_CMP6_REG, 4
	.endif

	;Checking if the counter is greater than comapre 5/6 value.
	QBLT	COUNTER_LT_COMPARE_5_6_ICSS_REV1, R5, R20

	;If counter is greater than compare 5/6, we check for compare 0/7 again
	;to eliminate the corner condition as mentioned above. This will clear
	;TTS_CMP5_CMP6_in_next_IEP_cycle in case of a wraparound.
	M_TTS_CMP0_CMP7_CHECK

	;Checking if CMP5/6 is in next IEP cycle.
        QBBS	CONTINUE_V3, R22 , TTS_CMP5_CMP6_in_next_IEP_cycle
        JMP	TASK_EXECUTION_FINISHED
CONTINUE_V3:
	;Available time = x + y
	LDI	R21.w0 , ICSS_EMAC_TTS_IEP_MAX_VAL & 0xFFFF
	LDI	R21.w2 , ICSS_EMAC_TTS_IEP_MAX_VAL >> 16
	SUB	R21, R21, R20	;x = Counter_Max - Counter_Current
	ADD	R20, R21, R5	;x + y; y = Compare_5/6_Value
	JMP		COUNTER_GT_COMPARE_5_6_ICSS_REV1

COUNTER_LT_COMPARE_5_6_ICSS_REV1:

	;Available time: Subtracting (Compare 5/6 time - Counter Value)
	SUB	R20, R5, R20

	;We also need to check for a boundary condition wherein
	;the counter is ahead of CMP5/6 but it wrapped around.
	;Hence the value of counter is < CMP5/6 value.
	;So we check for CMP0/CMP7 event.
	LBCO	&R21.b0, IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
	.if $defined("PRU0")
	;Checking for CMP0 event.
        QBBC	COUNTER_GT_COMPARE_5_6_ICSS_REV1, R21, 0
        JMP	TASK_EXECUTION_FINISHED
	.else	;PRU1
	;Checking for CMP7 event.
        QBBC	COUNTER_GT_COMPARE_5_6_ICSS_REV1, R21, 7
        JMP	TASK_EXECUTION_FINISHED
        .endif

COUNTER_GT_COMPARE_5_6_ICSS_REV1:

	;Comparing above value with required transmit time.
	QBGE	SIZE_ALLOWED_local_ICSS_REV1, TEMP_REG_1, R20

	;Size not allowed.
        QBBS	CONTINUE_V4, R22 , TTS_RT_QUEUE_PKT
        JMP	TASK_EXECUTION_FINISHED
CONTINUE_V4:
	LDI	R20.w0, Q4_TX_CONTEXT_OFFSET
	JMP 	CHECK_NEXT_QUEUE

SIZE_ALLOWED_local_ICSS_REV1:
	QBBC	NOT_Queue1_SA_local_ICSS_REV1, R22 , TTS_RT_QUEUE_PKT
	SET	R22 , R22 , TTS_FIRST_PKT_done
	SET	R22 , R22 , TTS_FIRST_PKT_since_enable
	JMP		START_XMT_QUEUE
NOT_Queue1_SA_local_ICSS_REV1:
	;Check if RT packet missed cycle, in case NRT packet was the first packet in the cycle.
	M_TTS_MISSED_CYCLE_CHECK
	SET	R22 , R22 , TTS_FIRST_PKT_done
	SET	R22 , R22 , TTS_FIRST_PKT_since_enable
	JMP		START_XMT_QUEUE
	.endif	;ICSS_REV1

;;////////////////////////////////////////////////////////////////////////////
;******************************************************************************
;
;		NAME				: FN_TTS_IEP_CMPCFG_ARBITRATION
;		DESCRIPTION			: Function will aquire the IEP CMP CFG Register for configuration.
;							: This is required when TTS is enabled on both PRUs as they share resources.
;		RETURNS				: None
;		ARGS				: R5
;		ARGS PASSED BY		: FN_TTS_IEP_CFG_PRE
;		REGISTERS USED
;		-reserved			: None
;		-temporary			: R0, R20, R21
;		INVOKES 			: None
;
;******************************************************************************
FN_TTS_IEP_CMPCFG_ARBITRATION:
	LDI	R0.b0, SHIFT_NONE

; PRU0 acts as master as it is assigned higher priority in acquiring IEP CMP CFG Reg.
	.if $defined("PRU0")
wait_critical_iep_master:
	XIN	BANK0, &TTS_CRITICAL_IEP_REG_PRU1, 1
	QBBS	wait_critical_iep_master, TTS_CRITICAL_IEP_REG_PRU1, 0
	SET	TTS_CRITICAL_IEP_REG_PRU0 , TTS_CRITICAL_IEP_REG_PRU0 , 0
	XOUT	BANK0, &TTS_CRITICAL_IEP_REG_PRU0, 1
	.else	;PRU1 acts as slave.
wait_critical_iep_slave:
	XIN	BANK0, &TTS_CRITICAL_IEP_REG_PRU0, 1
	QBBS	wait_critical_iep_slave, TTS_CRITICAL_IEP_REG_PRU0, 0
	SET	TTS_CRITICAL_IEP_REG_PRU1 , TTS_CRITICAL_IEP_REG_PRU1 , 0
	XOUT	BANK0, &TTS_CRITICAL_IEP_REG_PRU1, 1
	LOOP	slave_wait_over, 11
	ADD	R0, R0, 0
slave_wait_over:
	;Check if PRU0 acquired cfg reg in between.
	XIN	BANK0, &TTS_CRITICAL_IEP_REG_PRU0, 1
	QBBC	execute_critical_iep_slave, TTS_CRITICAL_IEP_REG_PRU0, 0
	CLR	TTS_CRITICAL_IEP_REG_PRU1 , TTS_CRITICAL_IEP_REG_PRU1 , 0
	XOUT	BANK0, &TTS_CRITICAL_IEP_REG_PRU1, 1
	QBA		wait_critical_iep_slave
execute_critical_iep_slave:
	.endif
	JMP	R5.w0

;;////////////////////////////////////////////////////////////////////////////
;******************************************************************************
;
;		NAME				: FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION
;		DESCRIPTION			: Function will clear the IEP CMP CFG arbitration bits.
;							: This is required when TTS is enabled on both PRUs as they share resources.
;		RETURNS				: None
;		ARGS				: R5
;		ARGS PASSED BY		: FN_TTS_IEP_CFG_PRE
;		REGISTERS USED
;		-reserved			: None
;		-temporary			: R0, R20, R21
;		INVOKES 			: None
;
;******************************************************************************
FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION:
	LDI	R0.b0, SHIFT_NONE
	.if $defined("PRU0")
	CLR	TTS_CRITICAL_IEP_REG_PRU0 , TTS_CRITICAL_IEP_REG_PRU0 , 0
	XOUT	BANK0, &TTS_CRITICAL_IEP_REG_PRU0, 1
	.else
	CLR	TTS_CRITICAL_IEP_REG_PRU1 , TTS_CRITICAL_IEP_REG_PRU1 , 0
	XOUT	BANK0, &TTS_CRITICAL_IEP_REG_PRU1, 1
	.endif
	JMP	R5.w0

;;////////////////////////////////////////////////////////////////////////////

	.endif	;TTS
	.endif	;__emac_tts_p


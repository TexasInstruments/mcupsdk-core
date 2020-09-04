
;
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

	.include "icss_regs.h"
	.include "icss_cfg_regs.h"

	.asg	R30.t24,	ENDAT_CH0_RX_EN
	.asg	R31.t27,	ENDAT_CH0_RX_CLR_OVF
	.asg	R31.t24,	ENDAT_CH0_RX_CLR_VALID
	.asg	R31.t27,	ENDAT_CH0_RX_OVF
	.asg	R31.t24,	ENDAT_CH0_RX_VALID

	.asg	R30.t25,	ENDAT_CH1_RX_EN
	.asg	R31.t28,	ENDAT_CH1_RX_CLR_OVF
	.asg	R31.t25,	ENDAT_CH1_RX_CLR_VALID
	.asg	R31.t28,	ENDAT_CH1_RX_OVF
	.asg	R31.t25,	ENDAT_CH1_RX_VALID

	.asg	R30.t26,	ENDAT_CH2_RX_EN
	.asg	R31.t29,	ENDAT_CH2_RX_CLR_OVF
	.asg	R31.t26,	ENDAT_CH2_RX_CLR_VALID
	.asg	R31.t29,	ENDAT_CH2_RX_OVF
	.asg	R31.t26,	ENDAT_CH2_RX_VALID


; Channel select bits R30[17:16]
ENDAT_TX_CH0_SEL	.set					0
ENDAT_TX_CH1_SEL	.set					1
ENDAT_TX_CH2_SEL	.set					2

; CLK MODE bits R30[20:19]
ENDAT_TX_CLK_MODE_FREERUN_STOPLOW	.set	(0 << 3)
ENDAT_TX_CLK_MODE_FREERUN_STOPHIGH	.set	(1 << 3)
ENDAT_TX_CLK_MODE_FREERUN	.set			(2 << 3)
ENDAT_TX_CLK_MODE_STOPHIGH_AFTER_TX	.set	(3 << 3)

	.asg	R31.t18,	ENDAT_TX_CHANNEL_GO
	.asg	R31.t19,	ENDAT_TX_GLOBAL_REINIT
	.asg	R31.t20,	ENDAT_TX_GLOBAL_GO

	.asg	R31.t5,	ENDAT_CH0_TX_GLOBAL_REINIT_ACTIVE
	.asg	R31.t5,	ENDAT_CH0_TX_BUSY
	.asg	R31.t0,	ENDAT_CH0_TX_OVERUN
	.asg	R31.t1,	ENDAT_CH0_TX_UNDERRUN

	.asg	R31.t13,	ENDAT_CH1_TX_GLOBAL_REINIT_ACTIVE
	.asg	R31.t13,	ENDAT_CH1_TX_BUSY
	.asg	R31.t8,	ENDAT_CH1_TX_OVERUN
	.asg	R31.t9,	ENDAT_CH1_TX_UNDERRUN

	.asg	R31.t21,	ENDAT_CH2_TX_GLOBAL_REINIT_ACTIVE
	.asg	R31.t21,	ENDAT_CH2_TX_BUSY
	.asg	R31.t16,	ENDAT_CH2_TX_OVERUN
	.asg	R31.t17,	ENDAT_CH2_TX_UNDERRUN


; ENDAT

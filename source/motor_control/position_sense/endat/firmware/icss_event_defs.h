
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

SYS_EVT_TSC_ADC_PEND				.set	53
SYS_EVT_MAG_ADC_PEND				.set	34
SYS_EVT_PWM3_PEND					.set	43
SYS_EVT_PWM4_PEND					.set	46
SYS_EVT_PWM5_PEND					.set	37

SYS_EVT_PRU1_RX_EOF					.set	54
SYS_EVT_MDIO_MII_LINK1				.set	53
SYS_EVT_PORT1_TX_OVERFLOW			.set	52
SYS_EVT_PORT1_TX_UNDERFLOW			.set	51
SYS_EVT_PRU1_RX_OVERFLOW			.set	50
SYS_EVT_PRU1_RX_SOF					.set	47
SYS_EVT_PRU1_RX_ERR32				.set	45
SYS_EVT_PRU0_RX_EOF					.set	42
SYS_EVT_MDIO_MII_LINK0				.set	41
SYS_EVT_PORT0_TX_OVERFLOW			.set	40
SYS_EVT_PORT0_TX_UNDERFLOW			.set	39
SYS_EVT_PRU0_RX_OVERFLOW			.set	38
SYS_EVT_PRU0_RX_SOF					.set	35
SYS_EVT_PRU0_RX_ERR32				.set	33
; system events 31 to 16 are PRU generated events
; please define them in the protocol specific files
SYS_EVT_ECAP_PEND					.set	15
SYS_EVT_SYNC0_OUT_PEND				.set	14
SYS_EVT_SYNC1_OUT_PEND				.set	13
SYS_EVT_LATCH0_IN_PEND				.set	12
SYS_EVT_LATCH1_IN_PEND				.set	11
SYS_EVT_PDI_WD_EXP_PEND				.set	10
SYS_EVT_PD_WD_EXP_PEND				.set	9
SYS_EVT_IEP_TIM_CAP_CMP_PEND		.set	7

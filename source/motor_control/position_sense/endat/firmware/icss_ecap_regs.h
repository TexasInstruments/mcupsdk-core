
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

	.if	!$isdefed("__icss_ecap_regs_h")
__icss_ecap_regs_h	.set	1

ICSS_eCAP_TSCNT	.set			0x0000	;  32b time stamp counter
ICSS_eCAP_CNTPHS	.set		0x0004	;  counter phase offset value
ICSS_eCAP_CAP1	.set			0x0008	;  32b capture 1 reg
ICSS_eCAP_CAP2	.set			0x000C	;  32b capture 2 reg
ICSS_eCAP_CAP3	.set			0x0010	;  32b capture 3 reg
ICSS_eCAP_CAP4	.set			0x0014	;  32b capture 4 reg
ICSS_eCAP_ECCTL1	.set		0x0028	;  capture control reg 1
ICSS_eCAP_ECCTL2	.set		0x002A	;  capture control reg 2
ICSS_eCAP_ECEINT	.set		0x002C	;  capture interrupt enable reg
ICSS_eCAP_ECFLG	.set			0x002E	;  capture interrupt flag reg
ICSS_eCAP_ECCLR	.set			0x0030	;  capture interrupt clear reg
ICSS_eCAP_PID	.set			0x005c	;  peripheral ID reg

	.endif


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

	.if	!$isdefed("__icss_intc_regs_h")
__icss_intc_regs_h	.set	1

ICSS_INTC_REVID	.set		0x0000	;  Revision ID
ICSS_INTC_CR	.set		0x0004	;  Control Reg
ICSS_INTC_HCR	.set		0x000C	;  Host Control
ICSS_INTC_GER	.set		0x0010	;  Global Enable
ICSS_INTC_GNLR	.set		0x001C	;  Global Nesting Level
ICSS_INTC_SISR	.set		0x0020	;  Sys Interrupt Indexed Assert
ICSS_INTC_SICR	.set		0x0024	;  Sys Interrupt Indexed Deassert
ICSS_INTC_EISR	.set		0x0028	;  Sys Interrupt Indexed Enabling
ICSS_INTC_EICR	.set		0x002C	;  Sys Interrupt Indexed Disabling
ICSS_INTC_HIEISR	.set	0x0034	;  Host Interrupt Indexed Enabling
ICSS_INTC_HIDSR	.set		0x0038	;  Host Interrupt Indexed Disabling
ICSS_INTC_GPIR	.set		0x0080	;  Global Prioritized Index
ICSS_INTC_SRSR1	.set		0x0200	;  Sys Interrupt Raw Assert Reg 1
ICSS_INTC_SRSR2	.set		0x0204	;  Sys Interrupt Raw Assert Reg 2
ICSS_INTC_SECR1	.set		0x0280	;  Sys Interrupt Raw Deassert Reg 1
ICSS_INTC_SECR2	.set		0x0284	;  Sys Interrupt Raw Deassert Reg 2
ICSS_INTC_ESR1	.set		0x0300	;  Sys Interrupt Enable Set Reg 1
ICSS_INTC_ESR2	.set		0x0304	;  Sys Interrupt Enable Set Reg 2
ICSS_INTC_ECR1	.set		0x0380	;  Sys Interrupt Disable Set Reg 1
ICSS_INTC_ECR2	.set		0x0384	;  Sys Interrupt Disable Set Reg 2
ICSS_INTC_CMR1	.set		0x0400	;  Channel Map Reg  1
ICSS_INTC_CMR2	.set		0x0404
ICSS_INTC_CMR3	.set		0x0408
ICSS_INTC_CMR4	.set		0x040C
ICSS_INTC_CMR5	.set		0x0410
ICSS_INTC_CMR6	.set		0x0414
ICSS_INTC_CMR7	.set		0x0418
ICSS_INTC_CMR8	.set		0x041C
ICSS_INTC_CMR9	.set		0x0420
ICSS_INTC_CMR10	.set		0x0424
ICSS_INTC_CMR11	.set		0x0428
ICSS_INTC_CMR12	.set		0x042C
ICSS_INTC_CMR13	.set		0x0430
ICSS_INTC_CMR14	.set		0x0434
ICSS_INTC_CMR15	.set		0x0438
ICSS_INTC_CMR16	.set		0x043C	;  Channel Map Reg  16
ICSS_INTC_HMR0	.set		0x0800	;  Host Map Reg  1
ICSS_INTC_HMR1	.set		0x0804
ICSS_INTC_HMR2	.set		0x0808	;  Host Map Reg  3
ICSS_INTC_HIPIR0	.set	0x0900	;  Host Interrupt Prioritized Index Reg 0
ICSS_INTC_HIPIR1	.set	0x0904
ICSS_INTC_HIPIR2	.set	0x0908
ICSS_INTC_HIPIR3	.set	0x090C
ICSS_INTC_HIPIR4	.set	0x0910
ICSS_INTC_HIPIR5	.set	0x0914
ICSS_INTC_HIPIR6	.set	0x0918
ICSS_INTC_HIPIR7	.set	0x091C
ICSS_INTC_HIPIR8	.set	0x0920
ICSS_INTC_HIPIR9	.set	0x0924	;  Host Interrupt Prioritized Index Reg 9
ICSS_INTC_SIPR1	.set		0x0D00	;  Sys Interrupt Polarity Reg 1
ICSS_INTC_SIPR2	.set		0x0D04
ICSS_INTC_SITR1	.set		0x0D80	;  Sys Interrupt Type
ICSS_INTC_SITR2	.set		0x0D84
ICSS_INTC_HINLR1	.set	0x1100	;  Sys Interrupt Nesting Level Reg 1
ICSS_INTC_HINLR2	.set	0x1104
ICSS_INTC_HINLR3	.set	0x1108
ICSS_INTC_HINLR4	.set	0x110C
ICSS_INTC_HINLR5	.set	0x1110
ICSS_INTC_HINLR6	.set	0x1114
ICSS_INTC_HINLR7	.set	0x1118
ICSS_INTC_HINLR8	.set	0x111C
ICSS_INTC_HINLR9	.set	0x1120
ICSS_INTC_HINLR10	.set	0x1124	;  Sys Interrupt Nesting Level Reg 10
ICSS_INTC_HIER	.set		0x1500	;  Host Interrupt Enable Reg

	.endif

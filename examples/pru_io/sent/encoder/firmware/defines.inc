; Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; Redistributions of source code must retain the above copyright
; notice, this list of conditions and the following disclaimer.
;
; Redistributions in binary form must reproduce the above copyright
; notice, this list of conditions and the following disclaimer in the
; documentation and/or other materials provided with the
; distribution.
;
; Neither the name of Texas Instruments Incorporated nor the names of
; its contributors may be used to endorse or promote products derived
; from this software without specific prior written permission.
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

;************************************************************************************
;   File:     memory.inc
;
;   Brief:    Sent encoder defines
;************************************************************************************
    .asg    R2,         TEMP_REG_LOOP
    .asg    R3,			TEMP_REG_SYNC
    .asg    R4,         TEMP_REG
    .asg    R5,         COUNTER
    .asg    R6,         NUM_FRAMES
    .asg    R7,         TOTAL_FRAMES
    .asg    R8,         TEMP_REG1
    .asg    R9,         TEMP_REG2
    .asg    R10,        NIBBLE_VALUE
PDMEM00 .set                                0x00
PULSE_LOW_TICKS .set 5 ; Predefined 5 ticks low period after every pulse
SYNC_PULSE_PERIOD .set  51; Predefined Sync pulse is always 56 period


TX_DONE_OFFSET      .set    (0x4)
SC_NIBBLE_OFFSET    .set    (0x8)
DATA0_OFFSET        .set    (0x9)
DATA1_OFFSET        .set    (0xA)
DATA2_OFFSET        .set    (0xB)
DATA3_OFFSET        .set    (0xC)
DATA4_OFFSET        .set    (0xD)
DATA5_OFFSET        .set    (0xE)
CRC_OFFSET          .set    (0xF)

;************************************************************************************
;   For am263px and am263x, PRU0-GPIO0,GPIO1,GPIO2,GPIO3,GPIO4,GPIO5,GPIO6,GPIO8 are  
;   set to high so the corresponding hex value 0x17F is loaded in r30.
;
;   For am261-lp, PRU0-GPIO0,GPIO1,GPIO3,GPIO6,GPIO7,GPIO8,GPIO11,GPIO13 are
;   set to high so the corresponding hex value 0x29CB is loaded in r30.
;************************************************************************************
    .if	$isdefed("SOC_AM263X")
r30_value           .set    (0x17F)
    .elseif $isdefed("SOC_AM263PX")
r30_value           .set    (0x17F)
    .elseif $isdefed("SOC_AM261X")
r30_value           .set    (0x29CB)
    .endif
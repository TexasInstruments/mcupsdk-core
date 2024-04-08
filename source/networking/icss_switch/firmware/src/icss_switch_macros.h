;
;  TEXAS INSTRUMENTS TEXT FILE LICENSE
; 
;   Copyright (c) 2017 Texas Instruments Incorporated
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
; file:   icss_switch_macros.h
;
; brief:  Macros used for ICSS Ethernet switch
;
;
;

    .if    !$defined("__ICSS_SWITCH_MACRO_H__")
__ICSS_SWITCH_MACRO_H__    .set    1

    .include "icss_defines.h"

; M_GET_DESCRIPTOR_OFFSET port,queue, offset
; port = 0,1,2
; queue = 0,1,2,3
; offset = 16 bit address offset
M_GET_DESCRIPTOR_OFFSET    .macro    port_val8, queue_val8, offset_val16
  LSL  offset_val16.b0,port_val8,3
  LSL  offset_val16.b1,queue_val8,1
  ADD  offset_val16.b0,offset_val16.b0,offset_val16.b1
  ADD    offset_val16.w0, offset_val16.b0, QUEUE_DESCRIPTOR_OFFSET_ADDR & 0xFF
  ADD    offset_val16.b1,offset_val16.b1, (QUEUE_DESCRIPTOR_OFFSET_ADDR >> 8 )& 0xFF
  LBCO &offset_val16.w0,PRU1_DMEM_CONST,offset_val16.w0, 2
    .endm    

M_GET_COL_DESCRIPTOR_OFFSET    .macro    port_val8, desc_val32
	QBEQ	M_GET_COL_BD_P0?, port_val8, 0	
	QBEQ	M_GET_COL_BD_P1?, port_val8, 1	
    
; port 2
    LDI        desc_val32.w0, 0x1980
	JMP	M_GET_COL_BD_OFFSET_END?	
M_GET_COL_BD_P0?:
    LDI        desc_val32.w0, 0x1800
	JMP	M_GET_COL_BD_OFFSET_END?	
M_GET_COL_BD_P1?:
    LDI        desc_val32.w0, 0x18c0

M_GET_COL_BD_OFFSET_END?:
    .endm


; M_GET_QUEUE_OFFSET port,queue,offset
; port = 0,1,2
; queue = 0,1,2,3
; offset = 16 bit address offset
M_GET_QUEUE_OFFSET    .macro    port_val8, queue_val8, offset_val16
  LSL  offset_val16.b0,port_val8,3
  LSL  offset_val16.b1,queue_val8,1
  ADD  offset_val16.b0,offset_val16.b0,offset_val16.b1
  ADD    offset_val16.w0,offset_val16.b0, QUEUE_OFFSET_ADDR & 0xFF
  ADD  offset_val16.b1,offset_val16.b1, (QUEUE_OFFSET_ADDR >> 8) & 0xFF
	LBCO	&offset_val16.w0, PRU1_DMEM_CONST, offset_val16.w0, 2	
    .endm

M_GET_COL_QUEUE_OFFSET    .macro    port_val8, desc_val32
	QBEQ	M_GET_COL_BD_P0?, port_val8, 0	
	QBEQ	M_GET_COL_BD_P1?, port_val8, 1	
; port 2
    LDI        desc_val32.w0, 0xFA00
	JMP	M_GET_COL_QUEUE_OFFSET_END?	
M_GET_COL_BD_P0?:
    LDI        desc_val32.w0, 0xEE00
	JMP	M_GET_COL_QUEUE_OFFSET_END?	
M_GET_COL_BD_P1?:
    LDI        desc_val32.w0, 0xF400

M_GET_COL_QUEUE_OFFSET_END?:
    .endm

; M_GET_QUEUE_LENGTH port, queue, length
M_GET_QUEUE_LENGTH    .macro    port_val8, queue_val8, length_val16
  LSL  length_val16.b0,port_val8,3
  LSL  length_val16.b1,queue_val8,1
  ADD  length_val16.b0,length_val16.b0,length_val16.b1
    .if    $defined("TWO_PORT_CFG")  
  ADD    length_val16.w0,length_val16.b0, QUEUE_SIZE_ADDR & 0xFF
  ADD    length_val16.b1,length_val16.b1, (QUEUE_SIZE_ADDR >> 8) & 0xFF
    .else
  ADD    length_val16.w0,length_val16.b0, HOST_QUEUE_SIZE_ADDR & 0xFF
  ADD    length_val16.b1,length_val16.b1, (HOST_QUEUE_SIZE_ADDR >> 8) & 0xFF
    .endif  
  LBCO &length_val16.w0,PRU1_DMEM_CONST,length_val16.w0, 2
    .endm 

; transfer in the task parameter for the receive task from bank 1
M_XIN_RCV_TASK_PARAMS    .macro
    ; use R0 to shift register through bank
    ;LDI        R0.b0, RCV_PARAMS_IN
	XIN	BANK1, &MII_RCV, $sizeof(MII_RCV)	
    .endm

; transfer out the task parameter for the receive task to bank 1
M_OUT_RCV_TASK_PARAMS    .macro
    ; use R0 to shift register through bank
    ;LDI        R0.b0, RCV_PARAMS_OUT
	XOUT	BANK1, &MII_RCV, $sizeof(MII_RCV)	
    .endm

    .endif


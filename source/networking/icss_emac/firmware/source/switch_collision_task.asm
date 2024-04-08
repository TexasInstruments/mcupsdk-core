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
; file:   switch_collision_task.asm
;
; brief:  Task to be called from micro scheduler every 2.56us which clears collision buffer
;           There is max one packet per queue per task, i.e. max 3 packets to be cleared at a time
;         Min cycles if no collision is pending: 10
;         Max cycles if all three queues have pending packets: xxx
;         Total number of instructions: yyy
;
;
; description:
; Collision flag and queue is filled by tasks filling the queue which can be receive or host.
; Host can deceide whether to put in collision or wait for queue not busy.
; There is only one collision packet at a time. Collision buffer must be cleared before now packet
; can be inserted into collision buffer. Collision task will reset pending flag after buffer is
; transfered with no errors. If packet from collision does not fit then packet is dropped and 
; overflow counter is increased. 
 
    .if    !$defined("__switch_collision_task_h")
__switch_collision_task_h    .set    1

    .if    $defined("TWO_PORT_CFG")

;/////////////////////////////////////////////////////////
; Includes Section
;/////////////////////////////////////////////////////////

    .include "icss_switch.h"
    .include "icss_defines.h"
    .include "icss_switch_macros.h"
    .include "emac_MII_Rcv.h"
    .include "icss_miirt_regs.h"
    
    .global  COLLISION_TASK_EXEC

QUEUE_DESCRIPTOR_REG    .sassign R10, struct_queue
COLLISION_QUEUE_DESCRIPTOR_REG    .sassign R12, struct_queue

    ;COLLISION_STATUS_REG    .set    R14        ; collision status register loaded from memory
        .asg    R15.w0,    QUEUE_DSC_PTR_REG1     ; lower word of queue descriptor
        .asg    R16.w0,    QUEUE_DSC_PTR_REG2        ; higher word of queue descriptor
        .asg    R17.w0,    COLLQ_DSC_PTR_REG    ; 16 bit address pointer to collision descriptor
        .asg    R18 ,    QUEUE_SIZE_REG    ; actual size in bytes of current queue
        .asg    R9    ,    BD_REG    ; actual buffer descriptor for current queue and port
        .asg    R8.b0,    PORT_REG    ; port number 0,1 or 2
        .asg    R8.b1,    QUEUE_REG   ; queue number 0..3
        .asg    R8.w2,    COL_BD_LENGTH    ; length of packet in collision buffer in bytes
    .asg	R7, COL_CONST			; address of PRU memory where collision buffer is located
        .asg    R6    ,    COL_BD_REG    ; collision buffer descriptor


;/////////////////////////////////////////////////////////
; Task Execution Entry Point (called by scheduler)
;/////////////////////////////////////////////////////////

COLLISION_TASK_EXEC:
    LDI     R0, COLLISION_STATUS_ADDR
    LBCO    &COLLISION_STATUS_REG, PRU1_DMEM_CONST, R0, 4
    AND L1_CALL_REG , CALL_REG , CALL_REG
    LDI        R2.b0, 0x00   ; used for clearing
    .if    $defined("PRU0")    
    ; parse for collision on all three ports
    QBBS      P0_COLLISION,COLLISION_STATUS_REG, 0    ;P0_COLLISION_PENDING - t0
    QBBS      P2_COLLISION,COLLISION_STATUS_REG, 16   ;P2_COLLISION_PENDING - t16
    .else
    QBBS      P1_COLLISION,COLLISION_STATUS_REG, 8    ;P1_COLLISION_PENDING - t8
    .endif    
FINISH_COLLISION_CHECK:    
    AND CALL_REG , L1_CALL_REG , L1_CALL_REG

    ; no collision pending, return to scheduler, consumes 9 cycles
    JMP	CALL_REG	

    .if    $defined("PRU0")
; collision detected on port 0    
P0_COLLISION:

    ; extract queue number from collision status into QUEUE_REG
    ; load port number into PORT_REG

    LSR      QUEUE_REG, COLLISION_STATUS_REG.b0, 1
    LDI        PORT_REG, 0 
    LDI	COL_CONST.w0 , ICSS_SHARED & 0xFFFF
    LDI	COL_CONST.w2 , ICSS_SHARED >> 16
    JAL	CALL_REG, FN_PROCESS_COLLISION	
    JMP     FINISH_COLLISION_CHECK
    .endif    

    .if    $defined("PRU1")
P1_COLLISION:

    ; extract queue number from collision status into QUEUE_REG
    ; load port number into PORT_REG

    LSR      QUEUE_REG, COLLISION_STATUS_REG.b1, 1
    LDI     PORT_REG, 0x20
    ;LDI        COL_CONST, PRU0_DMEM
    JAL	CALL_REG, FN_PROCESS_COLLISION	
    JMP     FINISH_COLLISION_CHECK
    .endif

    .if    $defined("PRU0")    
P2_COLLISION:

    ; extract queue number from collision status into QUEUE_REG
    ; load port number into PORT_REG

    LSR      QUEUE_REG, COLLISION_STATUS_REG.b2, 1
    LDI     PORT_REG, 0x40
    ;LDI        COL_CONST, PRU1_DMEM
    JAL	CALL_REG, FN_PROCESS_COLLISION	
    JMP     FINISH_COLLISION_CHECK
    .endif
; end of taks section    
    
;****************************************************************************
;
;     NAME            : FN_PROCESS_COLLISION
;     DESCRIPTION    : Clears pending collision on all queues. Operates one collision at a time.
;     RETURNS        : 
;     ARGS            : PORT_REG, QUEUE_REG
;     USES         : register, no scratch
;     INVOKES         : 
;
;****************************************************************************
    
FN_PROCESS_COLLISION:    
    ; get queue descriptor from PRU Data Memory 
    ; start with P0 address, add offset for port and then add offset for queue
    ; PORT_REG is port*32, R2 is local
    
    LDI        R2.w0, P0_QUEUE_DESC_OFFSET
    ADD        R2.w0, R2.w0, PORT_REG
    LSL     R2.b2, QUEUE_REG, 3
    ADD        QUEUE_DSC_PTR_REG1, R2.w0, R2.b2
    LBCO    &QUEUE_DESCRIPTOR_REG, PRU1_DMEM_CONST, QUEUE_DSC_PTR_REG1, 8

    ; get collision queue descriptor by adding 8 byte offset to P0
    ; need to shift right by 2 first as it was 32 byte offset before
    ; PORT_REG is now port*8, r2 is local
    
    LDI        R2.w0, P0_COL_QUEUE_DESC_OFFSET
    LSR     PORT_REG, PORT_REG, 2
    ADD        COLLQ_DSC_PTR_REG, R2.w0, PORT_REG
    LBCO    &COLLISION_QUEUE_DESCRIPTOR_REG, PRU1_DMEM_CONST, COLLQ_DSC_PTR_REG, 8
    
    ; check on queue busy_s flag set in status register
    QBBS    QUEUE_BUSY, QUEUE_DESCRIPTOR_REG.busy_s, 0
    
    ; Check on queue busy_m flag set in status register.. this is bcoz same PRU might be receiving a packet in that Queue 
    QBBS    QUEUE_BUSY, QUEUE_DESCRIPTOR_REG.status, 0    ;busy_m - t0
    
    ; set busy flag for controller which resides in upper 4 bytes
    ; QUEUE_DSC_PTR_REG2 points now to upper half of descriptor, use byte write for atomic operation
    ; R2 is local
    LDI        R2.b0, 1
    ADD        QUEUE_DSC_PTR_REG2, QUEUE_DSC_PTR_REG1, 4
    ADD        QUEUE_DSC_PTR_REG2, QUEUE_DSC_PTR_REG2, 1
    SBCO    &R2.b0, PRU1_DMEM_CONST, QUEUE_DSC_PTR_REG2, 1

    ; as device we need to read and check busy_m again
    ; QUEUE_DSC_PTR_REG1 is still pointing to the right queue descriptor 
    ;LBCO    R11, PRU1_DMEM_CONST, QUEUE_DSC_PTR_REG2, 4
    ;QBBS    QUEUE_BUSY, QUEUE_DESCRIPTOR_REG.status.busy_m
    
    ; need to check on space available, i.e. free space in queue
    ; first get queue length from pre-initialized table
    ; PORT_REG is now port*1, QUEUE_SIZE is in 32 byte blocks -> need to * 8 to get to 4 bytes ptr
    LSR        PORT_REG, PORT_REG, 3
    M_GET_QUEUE_LENGTH PORT_REG, QUEUE_REG, QUEUE_SIZE_REG
    LSL     QUEUE_SIZE_REG.w0, QUEUE_SIZE_REG, 2
    
    QBLE     Px_LENGTH_CHECK_WRAP, QUEUE_DESCRIPTOR_REG.wr_ptr, QUEUE_DESCRIPTOR_REG.rd_ptr
    SUB     R2.w0, QUEUE_DESCRIPTOR_REG.rd_ptr, QUEUE_DESCRIPTOR_REG.wr_ptr
    JMP     SKIP_Px_LENGTH_CHECK_WRAP
    
Px_LENGTH_CHECK_WRAP:
    ; add queue size to rd_ptr and then subtract wr_ptr
    ADD     R2.w0, QUEUE_DESCRIPTOR_REG.rd_ptr, QUEUE_SIZE_REG.w0
    SUB        R2.w0, R2.w0, QUEUE_DESCRIPTOR_REG.wr_ptr    

SKIP_Px_LENGTH_CHECK_WRAP:
    ; free space is in R2.w0 in 4 byte blocks, i.e. pointer to bd
    ; now check on required space from collision buffer queue values
    ; which is in length field (bit 18..28) of descriptor in rd_ptr
     
    LBCO    &COL_BD_REG, ICSS_SHARED_CONST, COLLISION_QUEUE_DESCRIPTOR_REG.rd_ptr, 4
    
    ; extract length field which is in bytes
    ; shift left by 3 clears the upper 3 bits
    LSL     COL_BD_LENGTH, COL_BD_REG.w2, 3
    ; shift right by 5 moves length 16 bit aligned and byte pointer 
    LSR      COL_BD_LENGTH, COL_BD_LENGTH, 5
    ; Add 31 for rounding up to next block
    ADD        COL_BD_LENGTH, COL_BD_LENGTH, 31
    LSR        COL_BD_LENGTH, COL_BD_LENGTH, 5    
    LSL        COL_BD_LENGTH, COL_BD_LENGTH, 2
    ; exit on overflow
    ;QBGT    COLLISION_OVERFLOW, R2.w2, COL_BD_LENGTH
    QBGT    COLLISION_OVERFLOW, R2.w0, COL_BD_LENGTH
    
    ; save max_fill_level
    ; new fill_level = (free_space - COL_BD_LENGTH) >> 5
    ; free space in queue is in R2.w2 (byte)
    ; max_fill_level is in 32 byte blocks unit
    SUB      R2.w0, R2.w0, COL_BD_LENGTH
    SUB        R2.w2, QUEUE_SIZE_REG.w0, R2.w0
    LSR        R2.w2, R2.w2, 2
    QBGE    COL_SKIP_NEW_FILL_LEVEL, R2.w2, QUEUE_DESCRIPTOR_REG.max_fill_level
    AND QUEUE_DESCRIPTOR_REG.max_fill_level , R2.w2 , R2.w2
    ADD        r3.w0, QUEUE_DSC_PTR_REG1, 6    ;OFFSET(QUEUE_DESCRIPTOR_REG.max_fill_level)
    SBCO    &QUEUE_DESCRIPTOR_REG.max_fill_level, PRU1_DMEM_CONST, r3.w0, 1

COL_SKIP_NEW_FILL_LEVEL:
    ; copy packet descriptor to queue
    ; write collision buffer descriptor to port queue
    ;Set the Shadow bit
    SET        COL_BD_REG, COL_BD_REG.t14
    SBCO    &COL_BD_REG, ICSS_SHARED_CONST, QUEUE_DESCRIPTOR_REG.wr_ptr, 4
    ADD     R2.w2, QUEUE_DESCRIPTOR_REG.wr_ptr, COL_BD_LENGTH 
    ; is new wr_ptr going above queue size, QUEUE_SIZE_REG is still 4 byte pointer
    M_GET_DESCRIPTOR_OFFSET PORT_REG, QUEUE_REG, r3
    ADD     R2.w0, r3.w0, QUEUE_SIZE_REG.w0
    
    QBLT    COL_UPDATE_WR_PTR_WRAP, R2.w0, R2.w2
    ; then adjust by queue size
    ;SUB        QUEUE_DESCRIPTOR_REG.wr_ptr, QUEUE_DESCRIPTOR_REG.wr_ptr, QUEUE_SIZE_REG
    SUB        R2.w2, R2.w2, R2.w0
    ADD        R2.w2, R2.w2, r3.w0    
COL_UPDATE_WR_PTR_WRAP:
    ; store new write pointer  
    ;ADD        r1.w2, QUEUE_DSC_PTR_REG1, OFFSET(QUEUE_DESCRIPTOR_REG.wr_ptr)  ;REG1 is reserved
    ADD        r2.w0, QUEUE_DSC_PTR_REG1, 2    ;OFFSET(QUEUE_DESCRIPTOR_REG.wr_ptr)
    ;SBCO    R2.w2, PRU1_DMEM_CONST, r1.w2, 2    
    SBCO    &R2.w2, PRU1_DMEM_CONST, r2.w0, 2
    
    ; pointer on collision queue will be updated when packet is taken out of queue
    ; this avoids additional PRU copy of large packets. xmt function needs to check collision flag first before it 
    ; start to copy data from regular queue. drawback is that collision buffer is blocked until packet is taken out
    ; of host queueu. 
    
    ;Clear the busy_m bit
    LBCO    &R2.b0, PRU1_DMEM_CONST, QUEUE_DSC_PTR_REG2, 1
    AND        R2.b0, R2.b0, 0xFC  ; !(1<<Q_BUSY_M_BIT | 1<<Q_COLLISION_BIT)
    SBCO    &R2.b0, PRU1_DMEM_CONST, QUEUE_DSC_PTR_REG2, 1
    
    ;Clear the "collision occured" bit in collision status
    .if    $defined("PRU0")
    QBBC	CLEAR_P2_COLLISION, COLLISION_STATUS_REG, 0	 ;replaced: QBBC	CLEAR_P2_COLLISION, COLLISION_STATUS_REG.P0_COLLISION_PENDING 
    SBCO	&R2.b0, PRU1_DMEM_CONST, R0, 1	
    
    LDI	R2.b0 , 1
    ADD	R0, R0, 3	
    SBCO	&R2.b0, PRU1_DMEM_CONST, R0, 1	;set bit which indicates that host is yet to empty the collision queue	
    ; Give a Interrupt to indicate that a Packet has been received
    LDI	R31, 0x24	; Maps to system event 20, 0x24 = 10 0100
    QBA		FINISH_COLLISION_CHECK
CLEAR_P2_COLLISION:	
    ADD	R0, R0, 2	
    SBCO	&R2.b0, PRU1_DMEM_CONST, R0, 1	
    .else
    ADD	R0, R0, 1	
    SBCO	&R2.b0, PRU1_DMEM_CONST, R0, 1	
    .endif
    
QUEUE_BUSY:
    ; need to clear busy_s flag in queue descriptor
    ;LDI        R2.b0,     0
    ;SBCO    R2.b0, PRU1_DMEM_CONST, QUEUE_DSC_PTR_REG2, 1
    
    QBNE    FINISH_COLLISION_CHECK,    PORT_REG, 0
    ; Give a Interrupt to indicate that a Packet has been received
    ;LDI        R31, 0x24       ; Maps to system event 20, 0x24 = 10 0100
    ; set flag for stats update "collision busy"
    JMP        FINISH_COLLISION_CHECK

COLLISION_OVERFLOW:
    
    ; increment overflow_cnt in queue descriptor of port
    ADD     QUEUE_DESCRIPTOR_REG.overflow_cnt, QUEUE_DESCRIPTOR_REG.overflow_cnt, 1
    ADD     R2.w0, QUEUE_DSC_PTR_REG2, 7    ;OFFSET(QUEUE_DESCRIPTOR_REG.overflow_cnt)
    
    SBCO    &QUEUE_DESCRIPTOR_REG.overflow_cnt, PRU1_DMEM_CONST, R2.w0, 1

    ;Clear the busy_m bit and set the overflow bit
    LDI        R2.b0,     0x04
    SBCO    &R2.b0, PRU1_DMEM_CONST, QUEUE_DSC_PTR_REG2, 1
    
    ; set overflow flag in status update register - optional as it is already in queue descriptor
    
    ; clear busy flag
    JMP QUEUE_BUSY
    
    ;.leave     COLLISION_TASK_PROCESSING
    .endif ;TWO_PORT_CFG
    .endif

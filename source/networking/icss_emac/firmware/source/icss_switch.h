;
;  TEXAS INSTRUMENTS TEXT FILE LICENSE
; 
;   Copyright (c) 2017-2019 Texas Instruments Incorporated
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
;  manufactured by or for TI (�TI Devices�).  No hardware patent is licensed hereunder.
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
;  THIS SOFTWARE IS PROVIDED BY TI AND TI�S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED 
;  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
;  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI�S 
;  LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
;  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
;  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
;  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
;  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
; 
; file:   icss_switch.h
;
; brief:  Definitions and mapping of Ethernet switch over PRU
;         Includes:
;         1. Buffer pools, queus, descriptors
;         2. Switch configutation parameter
;         3. Statistics
;         4. Events for switch interaction
;         5. Memory Map and Control Register definition
;
;

    .if !$defined("ICSS_SWITCH__H")	
ICSS_SWITCH__H	.set	1

;****************************************************************************
;*                       Basic Switch Parameters                            *
;* Used to auto compute offset addresses on L3 OCMC RAM. Do not modify these*
;*       without changing firmware accordingly                              *
;****************************************************************************

SWITCH_BUFFER_SIZE    .set      64*1024 ; L3 buffer
BLOCK_SIZE    .set              32      ; bytes derived from ICSS architecture
BD_SIZE    .set                 4       ; one buffer descriptor is 4 bytes
MAX_NUMBER_OF_QUEUES    .set    4       ; up to 4 queues supported
MIN_NUMBER_OF_QUEUES    .set    1       ;
NUMBER_OF_PORTS    .set         3       ; two phy ports and one host port
;****************************************************************************
;*                       End of Section                                     *
;****************************************************************************




;****************************************************************************
;*                       LINK/DUPLEX MACROS & MASKS                         *
;*  Bits for Port Status and Duplexity. This is set by ARM and read by FW   *
;*          Possible to add more info as bits 2-7 are free                  *
;****************************************************************************
    .asg    t0,    PORT_LINK_UP
    .asg    t1,    PORT_IS_HD

PORT_LINK_MASK    .set          0x1
PORT_IS_HD_MASK    .set         0x2

NUM_EXCESS_COLLISIONS    .set       15

;****************************************************************************
;*                       End of Section                                     *
;****************************************************************************

;****************************************************************************
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;                     Shared RAM Offsets for Switch                         *
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;****************************************************************************
; ICSS Shared RAM 12kB.

; Queues on PHY PORT 1/2
NUMBER_OF_QUEUES    .set    4           ; different number of queues will have significant impact to memory map
; 48 blocks per max packet
; 2 full sized ETH packets: 96 blocks, 3 packets = 144, 4 packets = 192

; Physical Port queue size. Same for both ports
QUEUE_1_SIZE    .set        97  ; Network Management high
QUEUE_2_SIZE    .set        97  ; Network Management low
QUEUE_3_SIZE    .set        97  ; Protocol specific
QUEUE_4_SIZE    .set        97  ; NRT (IP,ARP, ICMP, �)

; HOST PORT QUEUES can buffer up to 4 full sized frames per queue
HOST_QUEUE_1_SIZE    .set       194 ; Protocol and/or VLAN priority 7 and 6
HOST_QUEUE_2_SIZE    .set       194 ; Protocol mid
HOST_QUEUE_3_SIZE    .set       194 ; Protocol low
HOST_QUEUE_4_SIZE    .set       194 ; NRT (IP, ARP, ICMP �)

COLLISION_QUEUE_SIZE    .set    48
P0_COL_TOP_MOST_BD_OFFSET    .set   (4*COLLISION_QUEUE_SIZE) + P0_COL_BD_OFFSET - 4
P1_COL_TOP_MOST_BD_OFFSET    .set   (4*COLLISION_QUEUE_SIZE) + P1_COL_BD_OFFSET - 4
P2_COL_TOP_MOST_BD_OFFSET    .set   (4*COLLISION_QUEUE_SIZE) + P2_COL_BD_OFFSET - 4

TOTAL_BUFFER_POOL    .set   2*(QUEUE_1_SIZE + QUEUE_2_SIZE + QUEUE_3_SIZE + QUEUE_4_SIZE) + HOST_QUEUE_1_SIZE + HOST_QUEUE_2_SIZE + HOST_QUEUE_3_SIZE + HOST_QUEUE_4_SIZE

; NRT Buffer descriptor definition
; Each buffer descriptor points to a max 32 byte block and has 32 bit in size to have atomic operation.
; PRU can address bytewise into memory.
; Definition of 32 bit desriptor is as follows
;
; Bits     Name            Meaning
; ==================================================================================================
; 0..7     Index           points to index in buffer queue, max 256 x 32 byte blocks can be addressed
; 8..12    Block_length    number of valid bytes in this specific block. Will be <=32 bytes on last block of packet
; 13       More            "More" bit indicating that there are more blocks for the
; 14       Shadow          inidcates that "index" is pointing into shadow buffer
; 15       TimeStamp       indicates that this packet has time stamp in seperate buffer - only needed of PTCP runs on host
; 16..17   Port            different meaning for ingress and egress, ingress Port=0 inidcates phy port 1 and Port = 1
;                          inidcates phy port 2. Egress: 0 sends on phy port 1 and 1 sends on phy port 2. Port = 2 goes
;                          over MAC table look-up
; 18..28   Length          11 bit of total packet length which is put into first BD only so that host access only one BD
; 29       VlanTag         indicates that packet has Length/Type field of 0x08100 with VLAN tag in following byte
; 30       Broadcast       inidcates that packet goes out on both physical ports,  there will be two bd but only one buffer
; 31       Error           indicates there was an error in the packet
;
    .if    $defined("PRU")
        .asg    b0,    Index
        .asg    b1,    Block_length
        .asg    t13,    More
        .asg    t14,    Shadow
        .asg    t16,    TimeStamp
        .asg    b2,    Port
        .asg    w2,    Length
        .asg    t29,    VlanTag
        .asg    t30,    Broadcat
        .asg    t31,    Error
    .else
    ; #defines for bd masks
    .endif

; NRT Queue Defintion
; Each port has up to 4 queues with variable length. The queus is processed as ring buffer with read and write pointer.
; Both pointer are address pointers and increment by 4 for each buffer descriptor/position.
; Queue has a length defined in constants and a status. Status is defined as described below
;
; Bits     Name            Meaning
; =====================================================================================================
; 0        Busy_M          This queue is busy by the controller port which is the PRU receiving packets from the controller
; 1        Collision       Device is/has written into shadow queue, both descriptors and data.
; 2        Overflow        there was not enough space to write to queue and packet was discarded
; 4..7     Reserved        reserved
;
; There is busy device flag in different byte address to grant access to queue to controller in case
; of simultaneous access. Host will alwasys be device in this case. The PRU which is sending the
; packet on phy port will be the controller. When both PRUs wants to write to host queues PRU0
; is controller and PRU1 is device.
;
; Bits     Name            Meaning
; =====================================================================================================
; 0        Busy_S          This queue is busy by the controller port which is the PRU receiving packets from the controller
; 1..7     Reserved
;
; Length is the number of 32 byte blocks per queue. max_fill_level tells the minimum distance between write and read pointer.
; over_flow_cnt tells how many times the write pointer runs into the read_pointer.

    .if    $defined("PRU")
struct_queue    .struct
rd_ptr    .ushort
wr_ptr    .ushort
busy_s    .ubyte
status    .ubyte
max_fill_level    .ubyte
overflow_cnt    .ubyte
;struct_queue_len
    .endstruct

    .asg    t0,    busy_m
;    .else
;typedef struct _Queue
;{
;    uint16_t  rd_ptr;
;    uint16_t  wr_ptr;
;    uint8_t busy_s;
;    uint8_t   status;
;    uint8_t   max_fill_level;
;    uint8_t   overflow_cnt;
;} Queue;
;
    .endif

    .if    $defined("PRU")
QUEUE_DESCRIPTOR_SIZE	.set	 $sizeof(struct_queue) 	 ; typically 8 bytes per register
;    .else
;    #define QUEUE_DESCRIPTOR_SIZE sizeof(Queue)         ; typically 8 bytes per register
     .endif
TOTAL_QUEUE_DESCRIPTOR_SIZE    .set NUMBER_OF_PORTS * NUMBER_OF_QUEUES

Q_MAX_FILL_LEVEL_OFFSET    .set     6
Q_OVERFLOW_CNT_OFFSET    .set       7
; Memory Map for buffer pool, buffer descriptor and queue descriptor
    .asg    t7,    TX0_UNDERFLOW
    .asg    t8,    TX0_OVERFLOW
    .asg    t19,    TX1_UNDERFLOW
    .asg    t20,    TX1_OVERFLOW

;//**************************************************************************
;                                                                           *
;                                                                           *
;           Diagrams below show the memory usage for PRU DRAM               *
;                  and shared RAM for switch and EMAC                       *
;                                                                           *
;                                                                           *
;****************************************************************************

;//**************************************************************************
;                                                                           *
;                                                                           *
;                 Usage for switch.  Numbers indicate bytes                 *
;                                                                           *
;                                                                           *
;****************************************************************************

;          DRAM0                                                        DRAM1                                             Shared RAM
;***********0x2000**************                         ************0x2000***************                 *************0x3000**************
;**         Reserved           *                         **          Reserved            *                 **     Available for Protocol   *
;***********0x1FAC**************                         ***********0x1FAC****************                 **             Usage            *
;**    Used for statistics     *                         **      Used for statistics     *                 *                               *
;**        and misc            *                         **         and misc             *                 ** ***********0x2290*************
;***********0x1F00**************                         ************0x1F00***************                 **   Multicast Filtering Table  *
;**                            *                         **          Reserved            *                 **   (Only used by Profinet)    *
;**                            *                         *************0x1EDC**************                 *    Free for other protocols   *
;**                            *                         **     Switch Context + Misc    *                 *************0X1E80**************
;**   Available for Protocol   *                         **         Stored here          *                 **                              *
;**         Usage              *                         ************0X1D00***************                 **         BD Offsets for       *
;**                            *                         **                              *                 **           3 queues           *
;**                            *                         **     Available for Protocol   *                 **      and collision queue     *
;**                            *                         **           Usage              *                 **                              *
;**                            *                         **                              *                 **************0x400**************
;************0x200**************                         **************0x200**************                 **    Available for Protocols   *
;***       Reserved            *                         **          Reserved            *                 **                              *
;***********0x0000**************                         *************0x0000**************                 *************0x0000**************

;***********************************************************************************************************
;                                                                                                          *
;                                                                                                          *
;                                            End of Section                                                *
;                                                                                                          *
;                                                                                                          *
;***********************************************************************************************************


;//**************************************************************************
;                                                                           *
;                                                                           *
;                 Usage for EMAC.  Numbers indicate bytes                   *
;                                                                           *
;                                                                           *
;****************************************************************************

;          DRAM0                                                        DRAM1                                             Shared RAM
;***********0x2000**************                         ************0x2000***************                 *************0x3000**************
;**         Reserved           *                         **          Reserved            *                 **                              *
;***********0x1FA8**************                         *************0x1FA8**************                 **                              *
;**    Used for statistics     *                         **      Used for statistics     *                 **    Available for Protocol    *
;**        and misc            *                         **         and misc             *                 **            Usage             *
;***********0x1F00**************                         ************0x1F00***************                 **                              *
;**    Queue Context Offsets   *                         **     Queue Context Offsets    *                 **                              *
;************0x1EC0*************                         *************0x1EC0**************                 *                               *
;**                            *                         **                              *                 *************0X1D00**************
;**   Available for Protocol   *                         **     Available for Protocol   *                 **                              *
;**         Usage              *                         **           Usage              *                 **         BD Offsets for       *
;**                            *                         **                              *                 **           3 queues           *
;**                            *                         **                              *                 **      and host queue BD       *
;**                            *                         **                              *                 **         + reserved           *
;**                            *                         **                              *                 **************0x400**************
;************0x200**************                         **************0x200**************                 **    Available for Protocols   *
;**        Reserved            *                         **           Reserved           *                 **                              *
;***********0x0000**************                         *************0x0000**************                 *************0x0000**************

;***********************************************************************************************************
;                                                                                                          *
;                                                                                                          *
;                                            End of Section                                                *
;                                                                                                          *
;                                                                                                          *
;***********************************************************************************************************

;*******************************************************************************
;        The following offsets indicate which sections of the memory
;                are reserved/provided for other protocol
;*******************************************************************************
;NOTE:Some memory is reserved for future use
;See map above for usage
SWITCH_PROTOCOL_SPECIFIC_DRAM0_MEMORY_SIZE    .set       0x1F00
SWITCH_PROTOCOL_SPECIFIC_DRAM0_MEMORY_OFFSET    .set     0x400

SWITCH_PROTOCOL_SPECIFIC_DRAM1_USAGE_SIZE    .set        0x1D00
SWITCH_PROTOCOL_SPECIFIC_DRAM1_MEMORY_OFFSET    .set     0x400

EMAC_PROTOCOL_SPECIFIC_DRAM_USAGE_SIZE    .set           0x1D00
EMAC_PROTOCOL_SPECIFIC_DRAM_MEMORY_OFFSET    .set        0x400

SWITCH_PROTOCOL_SPECIFIC_SRAM_USAGE_SIZE    .set         0x3000
SWITCH_PROTOCOL_SPECIFIC_SRAM_MEMORY_OFFSET    .set      0x2400

EMAC_PROTOCOL_SPECIFIC_SRAM_USAGE_SIZE    .set         0x300
EMAC_PROTOCOL_SPECIFIC_SRAM_MEMORY_OFFSET    .set      0x1C00

;The following offsets indicate which sections of the memory are used for switch and EMAC internal tasks
SWITCH_SPECIFIC_DRAM0_START_SIZE    .set         0x100
SWITCH_SPECIFIC_DRAM0_START_OFFSET    .set         0x1F00

SWITCH_SPECIFIC_DRAM1_START_SIZE    .set         0x300
SWITCH_SPECIFIC_DRAM1_START_OFFSET    .set         0x1D00

EMAC_SPECIFIC_DRAM_USAGE_SIZE    .set             0x140
EMAC_SPECIFIC_DRAM_START_OFFSET    .set             0x1EC0

SWITCH_SPECIFIC_SRAM_USAGE_SIZE    .set             0x2010
SWITCH_SPECIFIC_SRAM_START_OFFSET    .set         0x400    ;same as EMAC. The BD offsets are common between EMAC and Switch. See diagram

EMAC_SPECIFIC_SRAM_USAGE_SIZE    .set             0x1900
EMAC_SPECIFIC_SRAM_START_OFFSET    .set             0x0



;***********************************************************************************************************
;                                                                                                          *
;                                                                                                          *
;                                            End of Section                                                *
;                                                                                                          *
;                                                                                                          *
;***********************************************************************************************************


;****************************************************************************
;*                       General Purpose Statistics                         *
;*              These are present on both PRU0 and PRU1 DRAM                *
;****************************************************************************
; base statistics offset
STATISTICS_OFFSET    .set               0x1F00

; MII port TX statistics flags
TX_BC_FRAMES_OFFSET    .set              0x0
TX_MC_FRAMES_OFFSET    .set              0x4
TX_UC_FRAMES_OFFSET    .set              0x8

TX_BYTE_CNT_OFFSET    .set               0xc

; MII port RX statistics flags
RX_BC_FRAMES_OFFSET    .set             0x10
RX_MC_FRAMES_OFFSET    .set             0x14
RX_UC_FRAMES_OFFSET    .set             0x18

;Binning counters : disabled by default
RX_BYTE_CNT_OFFSET    .set              0x1c

;Binning counters : disabled by default
TX_64_BYTE_FRAME_OFFSET    .set         0x20
TX_65_127_BYTE_FRAME_OFFSET    .set     0x24
TX_128_255_BYTE_FRAME_OFFSET    .set    0x28
TX_256_511_BYTE_FRAME_OFFSET    .set    0x2c
TX_512_1023_BYTE_FRAME_OFFSET    .set   0x30
TX_1024_MAX_BYTE_FRAME_OFFSET    .set   0x34

RX_64_BYTE_FRAME_OFFSET    .set         0x38
RX_65_127_BYTE_FRAME_OFFSET    .set     0x3c
RX_128_255_BYTE_FRAME_OFFSET    .set    0x40
RX_256_511_BYTE_FRAME_OFFSET    .set    0x44
RX_512_1023_BYTE_FRAME_OFFSET    .set   0x48
RX_1024_MAX_BYTE_FRAME_OFFSET    .set   0x4c

;**********************************Collision Counters*******************************/
LATE_COLLISION_OFFSET    .set           STATISTICS_OFFSET + 0x50      ;set for a frame where collision took place after 512 bytes were sent on wire
SINGLE_COLLISION_OFFSET    .set         STATISTICS_OFFSET + 0x54      ;frame was sent after a single collision was reported
MULTIPLE_COLLISION_OFFSET    .set       STATISTICS_OFFSET + 0x58      ;frame was sent after more than one collision
EXCESS_COLLISION_OFFSET    .set         STATISTICS_OFFSET + 0x5c      ;more than 16 collisions
;***********************************************************************************/

;********************Error packets****************
;Packed dropped on account of storm prevention
RX_MISALIGNMENT_COUNT_OFFSET	.set	  	EXCESS_COLLISION_OFFSET + 0x4  ;frame size in bits is not a multiple of 8
STORM_PREVENTION_COUNTER_BC	.set	      	RX_MISALIGNMENT_COUNT_OFFSET + 0x4
STORM_PREVENTION_COUNTER_MC	.set	      	STORM_PREVENTION_COUNTER_BC + 0x4
STORM_PREVENTION_COUNTER_UC	.set	      	STORM_PREVENTION_COUNTER_MC + 0x4
RX_ERROR_OFFSET	.set	               	STORM_PREVENTION_COUNTER_UC + 0x4 
SFD_ERROR_OFFSET	.set	              	RX_ERROR_OFFSET + 0x4 
TX_DEFERRED_OFFSET	.set	            	SFD_ERROR_OFFSET + 0x4 
TX_ERROR_OFFSET	.set	               	TX_DEFERRED_OFFSET + 0x4 
RX_OVERSIZED_FRAME_OFFSET	.set	     	TX_ERROR_OFFSET + 0x4   ;count of frames with size greater than 1518 bytes
RX_UNDERSIZED_FRAME_OFFSET	.set	    	RX_OVERSIZED_FRAME_OFFSET + 0x4   ;count of frames shorter than 64 bytes (including CRC)
RX_CRC_COUNT_OFFSET	.set	           	RX_UNDERSIZED_FRAME_OFFSET + 0x4   ;number of frames with CRC error
RX_DROPPED_FRAMES_OFFSET	.set	      	RX_CRC_COUNT_OFFSET + 0x4 

;****************Debug Statistics***************************************************/
TX_OVERFLOW_COUNTER	.set					RX_DROPPED_FRAMES_OFFSET + 0x4 
TX_UNDERFLOW_COUNTER	.set				TX_OVERFLOW_COUNTER + 0x4 

;**********************************************************************************/


STAT_SIZE    .set  0x98    ;always keep it's value same as the last stat offset

;****************************************************************************
;                          End of Statistics                                *
;****************************************************************************


;****************************************************************************
;*                           Offset for storing                             *
;              1. Storm Prevention Params                                   *
;              2. PHY Speed Offset                                          *
;              3. Port Status Offset                                        *
;*              These are present on both PRU0 and PRU1                     *
;****************************************************************************
STORM_PREVENTION_OFFSET_BC_DRIVER   .set    STATISTICS_OFFSET + STAT_SIZE       ;4 bytes
PHY_SPEED_OFFSET                    .set    STATISTICS_OFFSET + STAT_SIZE + 4   ;4 bytes
PORT_STATUS_OFFSET                  .set    STATISTICS_OFFSET + STAT_SIZE + 8   ;1 byte
COLLISION_COUNTER                   .set    STATISTICS_OFFSET + STAT_SIZE + 9   ;1 byte
RX_PKT_SIZE_OFFSET                  .set    STATISTICS_OFFSET + STAT_SIZE + 10  ;4 bytes
PORT_CONTROL_ADDR                   .set    STATISTICS_OFFSET + STAT_SIZE + 14  ;4 bytes
PORT_MAC_ADDR                       .set    STATISTICS_OFFSET + STAT_SIZE + 18  ;6 bytes
RX_INT_STATUS_OFFSET                .set    STATISTICS_OFFSET + STAT_SIZE + 24  ;1 byte
STORM_PREVENTION_OFFSET_MC_DRIVER   .set    STATISTICS_OFFSET + STAT_SIZE + 25  ;4 bytes
STORM_PREVENTION_OFFSET_UC_DRIVER   .set    STATISTICS_OFFSET + STAT_SIZE + 29  ;4 bytes
STORM_PREVENTION_OFFSET_BC          .set    STATISTICS_OFFSET + STAT_SIZE + 33  ;4 bytes
STORM_PREVENTION_OFFSET_MC          .set    STATISTICS_OFFSET + STAT_SIZE + 37  ;4 bytes
STORM_PREVENTION_OFFSET_UC          .set    STATISTICS_OFFSET + STAT_SIZE + 41  ;4 bytes
SP_UPDATE_TIMESTAMP_OFFSET          .set    STATISTICS_OFFSET + STAT_SIZE + 45  ;4 bytes
SP_INCREMENT_COUNT_OFFSET           .set    STATISTICS_OFFSET + STAT_SIZE + 49  ;4 bytes
SP_COUNTER_UPDATE_INTERVAL_OFFSET   .set    STATISTICS_OFFSET + STAT_SIZE + 53  ;4 bytes
DISABLE_STORM_PREV_FOR_HOST         .set    STATISTICS_OFFSET + STAT_SIZE + 57  ;1 byte

SP_COUNTER_UPDATE_INTERVAL_DEFAULT  .set    100000000

    
;****************************************************************************
;                          Protocol-specific Stats                          *
;****************************************************************************
; Placing these AFTER cfg offsets so as to not interfere with icss_emac
STP_INVALID_STATE_OFFSET    .set           STATISTICS_OFFSET + STAT_SIZE + 58 ; number of invalid STP state errors
;****************************************************************************
RX_QUEUE_OVERFLOW_FRAMES_OFFSET     .set    STATISTICS_OFFSET + STAT_SIZE + 62	;4 bytes
;***********************************************************************************************************
;                                                                                                          *
;                                                                                                          *
;                                            End of Section                                                *
;                                                                                                          *
;                                                                                                          *
;***********************************************************************************************************


;****************************************************************************
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;                      DRAM1 Offsets for Switch                             *
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;****************************************************************************
;*****************Queue Descriptors********************
P2_QUEUE_DESC_OFFSET    .set    0x1EBC          ; 4 queue descriptors for port 2 (PRU0 xmt)
P1_QUEUE_DESC_OFFSET    .set    0x1E9C          ; 4 queue descriptors for port 1 (PRU1 xmt)
P0_QUEUE_DESC_OFFSET    .set    0x1E7C          ; 4 queue descriptors for port 0 (host receive)
P2_COL_QUEUE_DESC_OFFSET    .set    0x1E74      ; collision descriptor of port 2
P1_COL_QUEUE_DESC_OFFSET    .set    0x1E6C      ; collision descriptor of port 1
P0_COL_QUEUE_DESC_OFFSET    .set    0x1E64      ; collision descriptor of port 0
COLLISION_STATUS_ADDR    .set       0x1E60      ; Collision Status Register, P0: bit 0 is pending flag, bit 1..2 inidicates which queue,
; P1: bit 8 is pending flag, 9..10 is queue number
; p2: bit 16 is pending flag, 17..18 is queue number, remaining bits are 0.

INTERFACE_MAC_ADDR    .set          0x1E58      ; Interface MAC Address
P2_MAC_ADDR    .set                 0x1E50      ; Port 2 MAC Address
P1_MAC_ADDR    .set                 0x1E48      ; Port 1 MAC Address
; With dynamic configuration of queue size the offsets are variablel. For PRU to find which queue and descriptor needs to be served there
; is a look-up table with index of port and queue number.
; table definition to access queue size, buffer descriptor and buffer

; table offset for queue size: 3 ports * 4 Queues * 1 byte offset = 24 bytes
QUEUE_SIZE_ADDR    .set                 0x1E30
; table offset for queue: 3 ports * 4 Queues * 2 byte offset = 24 bytes
QUEUE_OFFSET_ADDR    .set               0x1E18
; table offset for queue descriptors: 3 ports * 4 Queues * 2 byte offset = 24 bytes
QUEUE_DESCRIPTOR_OFFSET_ADDR    .set    0x1E00
; todo: remove this older definition
QUEUE_SIZE_TBL_ADDR    .set     0x1E00          ; table is filled by host before operation and structured in
; 0x1E08: P0Q0 0x1E09: P0Q1 0x1E0A: P0Q2 0x1E0B: P0Q3
; 0x1E04: P1Q0 0x1E05: P1Q1 0x1E06: P1Q2 0x1E07: P0Q3
; 0x1E00: P0Q0 0x1E01: P0Q1 0x1E02: P0Q2 0x1E03: P0Q3

CONFIGURATION_OFFSET    .set    0x1E00          ; 256 bytes of Configuratuin which also includes queue descriptors

; Port 1
TX_CONTEXT_P1_Q1_OFFSET_ADDR    .set    SWITCH_SPECIFIC_DRAM1_START_OFFSET
P1_Q1_TX_CONTEXT_OFFSET    .set         TX_CONTEXT_P1_Q1_OFFSET_ADDR
P1_Q2_TX_CONTEXT_OFFSET    .set         P1_Q1_TX_CONTEXT_OFFSET + 8
P1_Q3_TX_CONTEXT_OFFSET    .set         P1_Q2_TX_CONTEXT_OFFSET + 8
P1_Q4_TX_CONTEXT_OFFSET    .set         P1_Q3_TX_CONTEXT_OFFSET + 8
; Port 2
TX_CONTEXT_P2_Q1_OFFSET_ADDR    .set    P1_Q4_TX_CONTEXT_OFFSET + 8
P2_Q1_TX_CONTEXT_OFFSET    .set         TX_CONTEXT_P2_Q1_OFFSET_ADDR
P2_Q2_TX_CONTEXT_OFFSET    .set         P2_Q1_TX_CONTEXT_OFFSET + 8
P2_Q3_TX_CONTEXT_OFFSET    .set         P2_Q2_TX_CONTEXT_OFFSET + 8
P2_Q4_TX_CONTEXT_OFFSET    .set         P2_Q3_TX_CONTEXT_OFFSET + 8

; Port 1 Tx Collision Context
COL_TX_CONTEXT_P1_Q1_OFFSET_ADDR    .set    P2_Q4_TX_CONTEXT_OFFSET + 8
; Port 2 Tx Collision Context
COL_TX_CONTEXT_P2_Q1_OFFSET_ADDR    .set    COL_TX_CONTEXT_P1_Q1_OFFSET_ADDR + 8

; Host Port Rx Context
RX_CONTEXT_P0_Q1_OFFSET_ADDR    .set    COL_TX_CONTEXT_P2_Q1_OFFSET_ADDR + 8
P0_Q1_RX_CONTEXT_OFFSET    .set         RX_CONTEXT_P0_Q1_OFFSET_ADDR
P0_Q2_RX_CONTEXT_OFFSET    .set         P0_Q1_RX_CONTEXT_OFFSET + 8
P0_Q3_RX_CONTEXT_OFFSET    .set         P0_Q2_RX_CONTEXT_OFFSET + 8
P0_Q4_RX_CONTEXT_OFFSET    .set         P0_Q3_RX_CONTEXT_OFFSET + 8

; Port 1 Rx Context
RX_CONTEXT_P1_Q1_OFFSET_ADDR    .set    P0_Q4_RX_CONTEXT_OFFSET + 8
P1_Q1_RX_CONTEXT_OFFSET    .set         RX_CONTEXT_P1_Q1_OFFSET_ADDR
P1_Q2_RX_CONTEXT_OFFSET    .set         P1_Q1_RX_CONTEXT_OFFSET + 8
P1_Q3_RX_CONTEXT_OFFSET    .set         P1_Q2_RX_CONTEXT_OFFSET + 8
P1_Q4_RX_CONTEXT_OFFSET    .set         P1_Q3_RX_CONTEXT_OFFSET + 8

; Port 2 Rx Context
RX_CONTEXT_P2_Q1_OFFSET_ADDR    .set    P1_Q4_RX_CONTEXT_OFFSET + 8
P2_Q1_RX_CONTEXT_OFFSET    .set         RX_CONTEXT_P2_Q1_OFFSET_ADDR
P2_Q2_RX_CONTEXT_OFFSET    .set         P2_Q1_RX_CONTEXT_OFFSET + 8
P2_Q3_RX_CONTEXT_OFFSET    .set         P2_Q2_RX_CONTEXT_OFFSET + 8
P2_Q4_RX_CONTEXT_OFFSET    .set         P2_Q3_RX_CONTEXT_OFFSET + 8

; Host Port Collision Context
COL_RX_CONTEXT_P0_OFFSET_ADDR    .set   P2_Q4_RX_CONTEXT_OFFSET + 8   ;10 bytes
; Port 1 Rx Collision context
COL_RX_CONTEXT_P1_OFFSET_ADDR    .set   COL_RX_CONTEXT_P0_OFFSET_ADDR + 12   ;10 bytes
; Port 2 Rx Collision context
COL_RX_CONTEXT_P2_OFFSET_ADDR    .set   COL_RX_CONTEXT_P1_OFFSET_ADDR + 12   ;10 bytes


;***********************************************************************************************************
;                                                                                                          *
;                                                                                                          *
;                                            End of Section                                                *
;                                                                                                          *
;                                                                                                          *
;***********************************************************************************************************


PORT_QUEUE_DESC_OFFSET    .set          EMAC_SPECIFIC_DRAM_START_OFFSET                ; 4 queue descriptors for port tx. 32 bytes
TX_CONTEXT_Q1_OFFSET_ADDR    .set       PORT_QUEUE_DESC_OFFSET + 32
;***********************************************************************************************************
;                                                                                                          *
;                                                                                                          *
;                                            End of Section                                                *
;                                                                                                          *
;                                                                                                          *
;***********************************************************************************************************

;****************************************************************************
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;                         DRAM Offsets for EMAC                              *
;                      Present on Both DRAM0 and DRAM1                      *
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;****************************************************************************
; table offset for Port queue descriptors: 1 ports * 4 Queues * 2 byte offset = 8 bytes
Q1_TX_CONTEXT_OFFSET    .set            TX_CONTEXT_Q1_OFFSET_ADDR
Q2_TX_CONTEXT_OFFSET    .set            Q1_TX_CONTEXT_OFFSET + 8
Q3_TX_CONTEXT_OFFSET    .set            Q2_TX_CONTEXT_OFFSET + 8
Q4_TX_CONTEXT_OFFSET    .set            Q3_TX_CONTEXT_OFFSET + 8

HOST_BD_SIZE    .set           (HOST_QUEUE_1_SIZE + HOST_QUEUE_2_SIZE + HOST_QUEUE_3_SIZE + HOST_QUEUE_4_SIZE) * BD_SIZE
PORT_BD_SIZE    .set           (QUEUE_1_SIZE + QUEUE_2_SIZE + QUEUE_3_SIZE + QUEUE_4_SIZE) * 2 * BD_SIZE

P0_BUFFER_DESC_OFFSET    .set   SWITCH_SPECIFIC_SRAM_START_OFFSET
EOF_48K_BUFFER_BD    .set       P0_BUFFER_DESC_OFFSET + HOST_BD_SIZE + PORT_BD_SIZE
P0_COL_BD_OFFSET    .set        EOF_48K_BUFFER_BD
EOF_COL_BUFFER_BD    .set       P0_COL_BD_OFFSET + 3* BD_SIZE * 48

;***********************************************************************************************************
;                                                                                                          *
;                                                                                                          *
;                                            End of Section                                                *
;                                                                                                          *
;                                                                                                          *
;***********************************************************************************************************

;****************************************************************************
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;                     Shared RAM offsets for EMAC                           *
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;****************************************************************************

P1_COL_BD_OFFSET    .set        P0_COL_BD_OFFSET  + BD_SIZE * 48
P2_COL_BD_OFFSET    .set        P1_COL_BD_OFFSET  + BD_SIZE * 48

;EMAC Firmware Version Information
ICSS_EMAC_FIRMWARE_RELEASE_1_OFFSET	.set			0
ICSS_EMAC_FIRMWARE_RELEASE_2_OFFSET	.set			ICSS_EMAC_FIRMWARE_RELEASE_1_OFFSET + 4 
ICSS_EMAC_FIRMWARE_FEATURE_OFFSET	.set			ICSS_EMAC_FIRMWARE_RELEASE_2_OFFSET + 4 
ICSS_EMAC_FIRMWARE_RESERVED_FEATURE_OFFSET	.set	ICSS_EMAC_FIRMWARE_FEATURE_OFFSET + 4 
; Host Port Rx Context
HOST_Q1_RX_CONTEXT_OFFSET    .set           EOF_48K_BUFFER_BD    ;1; Prashant - TODO EOF_48K_BUFFER_BD
HOST_Q2_RX_CONTEXT_OFFSET    .set           HOST_Q1_RX_CONTEXT_OFFSET + 8
HOST_Q3_RX_CONTEXT_OFFSET    .set           HOST_Q2_RX_CONTEXT_OFFSET + 8
HOST_Q4_RX_CONTEXT_OFFSET    .set           HOST_Q3_RX_CONTEXT_OFFSET + 8

;***********************************************************************************************************
;                                                                                                          *
;                                                                                                          *
;                                            End of Section                                                *
;                                                                                                          *
;                                                                                                          *
;***********************************************************************************************************


;****************************************************************************
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;             Shared RAM offsets for both Switch and EMAC                   *
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;****************************************************************************

; allow for max 48k switch buffer which spans the descriptors up to 0x1800 6kB

; table offset for Host queue descriptors: 1 ports * 4 Queues * 2 byte offset = 8 bytes
HOST_QUEUE_DESCRIPTOR_OFFSET_ADDR    .set   HOST_Q4_RX_CONTEXT_OFFSET + 8           ;8 bytes
; table offset for queue: 4 Queues * 2 byte offset = 8 bytes
HOST_QUEUE_OFFSET_ADDR    .set              HOST_QUEUE_DESCRIPTOR_OFFSET_ADDR + 8
; table offset for queue size: 3 ports * 4 Queues * 1 byte offset = 12 bytes
HOST_QUEUE_SIZE_ADDR    .set                HOST_QUEUE_OFFSET_ADDR + 8

;------------------------------Queue Descriptors--------------------------------;
HOST_QUEUE_DESC_OFFSET    .set              HOST_QUEUE_SIZE_ADDR + 16           ; 4 queue descriptors for port 0 (host receive). 32 bytes
;------------------------------End of queue descriptors---------------------------;
P0_Q1_BD_OFFSET    .set         P0_BUFFER_DESC_OFFSET
P0_Q2_BD_OFFSET    .set         P0_Q1_BD_OFFSET + HOST_QUEUE_1_SIZE * BD_SIZE
P0_Q3_BD_OFFSET    .set         P0_Q2_BD_OFFSET + HOST_QUEUE_2_SIZE * BD_SIZE
P0_Q4_BD_OFFSET    .set         P0_Q3_BD_OFFSET + HOST_QUEUE_3_SIZE * BD_SIZE

P1_Q1_BD_OFFSET    .set         P0_Q4_BD_OFFSET + HOST_QUEUE_4_SIZE * BD_SIZE
P1_Q2_BD_OFFSET    .set         P1_Q1_BD_OFFSET + QUEUE_1_SIZE * BD_SIZE
P1_Q3_BD_OFFSET    .set         P1_Q2_BD_OFFSET + QUEUE_2_SIZE * BD_SIZE
P1_Q4_BD_OFFSET    .set         P1_Q3_BD_OFFSET + QUEUE_3_SIZE * BD_SIZE

P2_Q1_BD_OFFSET    .set         P1_Q4_BD_OFFSET + QUEUE_4_SIZE * BD_SIZE
P2_Q2_BD_OFFSET    .set         P2_Q1_BD_OFFSET + QUEUE_1_SIZE * BD_SIZE
P2_Q3_BD_OFFSET    .set         P2_Q2_BD_OFFSET + QUEUE_2_SIZE * BD_SIZE
P2_Q4_BD_OFFSET    .set         P2_Q3_BD_OFFSET + QUEUE_3_SIZE * BD_SIZE

;***********************************************************************************************************
;                                                                                                          *
;                                                                                                          *
;                                            End of Section                                                *
;                                                                                                          *
;                                                                                                          *
;***********************************************************************************************************


;****************************************************************************
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;                    Memory Usage of L3 OCMC RAM                            *
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;                                                                           *
;****************************************************************************
; L3 64KB Memory - mainly buffer Pool
; put collision buffer at end of L3 memory. Simplifies PRU coding to be on same memory as queue buffer
P2_COL_BUFFER_OFFSET    .set    0xFA00  ; 1536 byte collision buffer for port 2 send queue
P1_COL_BUFFER_OFFSET    .set    0xF400  ; 1536 byte collision buffer for port 1 send queue
P0_COL_BUFFER_OFFSET    .set    0xEE00  ; 1536 byte collision buffer for port 0 send queue

P0_Q1_BUFFER_OFFSET    .set     0x0000
P0_Q2_BUFFER_OFFSET    .set     P0_Q1_BUFFER_OFFSET + HOST_QUEUE_1_SIZE * BLOCK_SIZE
P0_Q3_BUFFER_OFFSET    .set     P0_Q2_BUFFER_OFFSET + HOST_QUEUE_2_SIZE * BLOCK_SIZE
P0_Q4_BUFFER_OFFSET    .set     P0_Q3_BUFFER_OFFSET + HOST_QUEUE_3_SIZE * BLOCK_SIZE
P1_Q1_BUFFER_OFFSET    .set     P0_Q4_BUFFER_OFFSET + HOST_QUEUE_4_SIZE * BLOCK_SIZE
P1_Q2_BUFFER_OFFSET    .set     P1_Q1_BUFFER_OFFSET + QUEUE_1_SIZE * BLOCK_SIZE
P1_Q3_BUFFER_OFFSET    .set     P1_Q2_BUFFER_OFFSET + QUEUE_2_SIZE * BLOCK_SIZE
P1_Q4_BUFFER_OFFSET    .set     P1_Q3_BUFFER_OFFSET + QUEUE_3_SIZE * BLOCK_SIZE
P2_Q1_BUFFER_OFFSET    .set     P1_Q4_BUFFER_OFFSET + QUEUE_4_SIZE * BLOCK_SIZE
P2_Q2_BUFFER_OFFSET    .set     P2_Q1_BUFFER_OFFSET + QUEUE_1_SIZE * BLOCK_SIZE
P2_Q3_BUFFER_OFFSET    .set     P2_Q2_BUFFER_OFFSET + QUEUE_2_SIZE * BLOCK_SIZE
P2_Q4_BUFFER_OFFSET    .set     P2_Q3_BUFFER_OFFSET + QUEUE_3_SIZE * BLOCK_SIZE
END_OF_BUFFER_POOL    .set      P2_Q4_BUFFER_OFFSET + QUEUE_4_SIZE * BLOCK_SIZE

;***********************************************************************************************************
;                                                                                                          *
;                                                                                                          *
;                                            End of Section                                                *
;                                                                                                          *
;                                                                                                          *
;***********************************************************************************************************

    .if $defined("PRU")	
; Collision Status Register
struct_collision_status	.struct	
port0    .ubyte            ; port 0 pending and queue number
port1    .ubyte            ; port 1 pending and queue number
port2    .ubyte            ; port 2 pending and queue number
    .endstruct

        .asg    t0,    P0_COLLISION_PENDING
        .asg    t8,    P1_COLLISION_PENDING
        .asg    t16,    P2_COLLISION_PENDING
        .asg    t24,    HOST_COLLISION_READ_PENDING ;Host yet to clear the collision buffer


    .endif

; switch configuration
; enable ports : receive enable , transmit enable, block, learning, flush table, age out table (older are removed), tear
; rate limiting setup
; filter database setup for MC
; learn multicast option one for both ports
; learn broadcast option one for both ports
; statistics clear
	.endif	;ICSS_SWITCH__H

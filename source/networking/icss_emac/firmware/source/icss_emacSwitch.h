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
; file:   icss_emacSwitch.h
;
; brief:  Definitions and mapping of Ethernet switch over PRU
;         Includes:
;         1. Buffer pools, queus, descriptors
;         2. Switch configutation parameter
;         3. Statistics
;         4. Events for switch interaction
;         5. Memory Map and Control Register definition
;
; Copyright (C) {2017} Texas Instruments Incorporated - http:;www.ti.com/

	.if !$defined("ICSS_EMAC_SWITCH__H")
ICSS_EMAC_SWITCH__H	.set	1

;****************************************************************************
;*                       Basic Switch Parameters                            *
;* Used to auto compute offset addresses on L3 OCMC RAM. Do not modify these*
;*       without changing firmware accordingly                              *
;****************************************************************************

SWITCH_BUFFER_SIZE	.set			64*1024  ; L3 buffer
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
	.asg	t0, PORT_LINK_UP
	.asg	t1, PORT_IS_HD

PORT_LINK_MASK	.set	0x1
PORT_IS_HD_MASK	.set	0x2

NUM_EXCESS_COLLISIONS	.set	15

;****************************************************************************
;*                       End of Section                                     *
;****************************************************************************

; Queues on PHY PORT 1/2
NUMBER_OF_QUEUES	.set	4
; 48 blocks per max packet
; 2 full sized ETH packets: 96 blocks, 3 packets = 144, 4 packets = 192

; Physical Port queue size. Same for both ports
QUEUE_1_SIZE	.set	97
QUEUE_2_SIZE	.set	97
QUEUE_3_SIZE	.set	97
QUEUE_4_SIZE	.set	97

; HOST PORT QUEUES can buffer up to 4 full sized frames per queue
HOST_QUEUE_1_SIZE	.set	194
HOST_QUEUE_2_SIZE	.set	194
HOST_QUEUE_3_SIZE	.set	194
HOST_QUEUE_4_SIZE	.set	194

COLLISION_QUEUE_SIZE	.set	48
P0_COL_TOP_MOST_BD_OFFSET	.set		(4*COLLISION_QUEUE_SIZE) + P0_COL_BD_OFFSET - 4  ; Warning: Some symbols not defined before.
P1_COL_TOP_MOST_BD_OFFSET	.set		(4*COLLISION_QUEUE_SIZE) + P1_COL_BD_OFFSET - 4  ; Warning: Some symbols not defined before.
P2_COL_TOP_MOST_BD_OFFSET	.set		(4*COLLISION_QUEUE_SIZE) + P2_COL_BD_OFFSET - 4  ; Warning: Some symbols not defined before.

TOTAL_BUFFER_POOL	.set		2*(QUEUE_1_SIZE + QUEUE_2_SIZE + QUEUE_3_SIZE + QUEUE_4_SIZE) +	HOST_QUEUE_1_SIZE + HOST_QUEUE_2_SIZE + HOST_QUEUE_3_SIZE + HOST_QUEUE_4_SIZE

; NRT Buffer descriptor definition
; Each buffer descriptor points to a max 32 byte block and has 32 bit in size to have atomic operation.
; PRU can address bytewise into memory.
; Definition of 32 bit desriptor is as follows
;
; Bits		Name			Meaning
; ==================================================================================================
; 0..7 	Index			points to index in buffer queue, max 256 x 32 byte blocks can be addressed
; 8..12	Block_length    number of valid bytes in this specific block. Will be <=32 bytes on last block of packet
; 13		More			"More" bit indicating that there are more blocks for the
; 14 		Shadow			inidcates that "index" is pointing into shadow buffer
; 15		TimeStamp		indicates that this packet has time stamp in seperate buffer - only needed of PTCP runs on host
; 16..17	Port			different meaning for ingress and egress, ingress Port=0 inidcates phy port 1 and Port = 1
;							inidcates phy port 2. Egress: 0 sends on phy port 1 and 1 sends on phy port 2. Port = 2 goes
; 							over MAC table look-up
; 18..28   Length			11 bit of total packet length which is put into first BD only so that host access only one BD
; 29		VlanTag			indicates that packet has Length/Type field of 0x08100 with VLAN tag in following byte
; 30		Broadcast		inidcates that packet goes out on both physical ports,  there will be two bd but only one buffer
; 31		Error			indicates there was an error in the packet
;
;	.if $defined("PRU")
	.asg	b0, Index
	.asg	b1, Block_length
	.asg	t13, More
	.asg	t14, Shadow
	.asg	t16, TimeStamp
	.asg	b2, Port
	.asg	w2, Length
	.asg	t29, VlanTag
	.asg	t30, Broadcat
	.asg	t31, Error
;	.else
  ; #defines for bd masks
;	.endif

; NRT Queue Defintion
; Each port has up to 4 queues with variable length. The queus is processed as ring buffer with read and write pointer.
; Both pointer are address pointers and increment by 4 for each buffer descriptor/position.
; Queue has a length defined in constants and a status. Status is defined as described below
;
; Bits     Name            Meaning
; =====================================================================================================
; 0        Busy_M          This queue is busy by the master port which is the PRU receiving packets from the master
; 1        Collision       Slave is/has written into shadow queue, both descriptors and data.
; 2        Overflow        there was not enough space to write to queue and packet was discarded
; 4..7     Reserved        reserved
;
; There is busy slave flag in different byte address to grant access to queue to master in case
; of simultaneous access. Host will alwasys be slave in this case. The PRU which is sending the
; packet on phy port will be the master. When both PRUs wants to write to host queues PRU0
; is master and PRU1 is slave.
;
; Bits     Name            Meaning
; =====================================================================================================
; 0        Busy_S          This queue is busy by the master port which is the PRU receiving packets from the master
; 1..7     Reserved
;
; Length is the number of 32 byte blocks per queue. max_fill_level tells the minimum distance between write and read pointer.
; over_flow_cnt tells how many times the write pointer runs into the read_pointer.

;	.if $defined("PRU")
struct_queue	.struct
rd_ptr	.ushort
wr_ptr	.ushort
busy_s	.ubyte
status	.ubyte
max_fill_level	.ubyte
overflow_cnt	.ubyte
	.endstruct

	.asg	t0, busy_m
;	.else
;typedef struct _Queue {
;	uint16_t  rd_ptr;
;	uint16_t  wr_ptr;
;	uint8_t	busy_s;
;	uint8_t   status;
;	uint8_t   max_fill_level;
;	uint8_t   overflow_cnt;
;} Queue;
;
;	.endif

;	.if $defined("PRU")
QUEUE_DESCRIPTOR_SIZE	.set	 $sizeof(struct_queue) 	 ; typically 8 bytes per register
;	.else
;QUEUE_DESCRIPTOR_SIZE	.set	 $sizeof(Queue) 		; typically 8 bytes per register
;	.endif
TOTAL_QUEUE_DESCRIPTOR_SIZE	.set	 NUMBER_OF_PORTS * NUMBER_OF_QUEUES

Q_MAX_FILL_LEVEL_OFFSET	.set	6
Q_OVERFLOW_CNT_OFFSET	.set	7

	.asg	t7, TX0_UNDERFLOW
	.asg	t8, TX0_OVERFLOW
	.asg	t19, TX1_UNDERFLOW
	.asg	t20, TX1_OVERFLOW

;EMAC Time Triggered Send Status Bit Masks
ICSS_EMAC_TTS_PRU_ENABLE_MASK	.set	0x1
ICSS_EMAC_TTS_MISSED_CYCLE_MASK	.set					(ICSS_EMAC_TTS_PRU_ENABLE_MASK << 1)
ICSS_EMAC_TTS_INSERT_CYC_FRAME_MASK	.set				(ICSS_EMAC_TTS_MISSED_CYCLE_MASK << 1)
ICSS_EMAC_TTS_CYC_TX_SOF_MASK	.set					(ICSS_EMAC_TTS_INSERT_CYC_FRAME_MASK << 1)
ICSS_EMAC_TTS_CYC_INTERRUPT_ENABLE_MASK	.set			(ICSS_EMAC_TTS_CYC_TX_SOF_MASK << 1)

;EMAC Time Triggered Send Constants
ICSS_EMAC_TTS_IEP_MAX_VAL	.set	0x3B9ACA00
ICSS_EMAC_TTS_FIRST_CST_SAFETY_MARGIN	.set	0x3A98
SP_COUNTER_UPDATE_INTERVAL_DEFAULT      .set    100000000

;Other protocols related defines
TRANSMIT_QUEUES_BUFFER_OFFSET	.set	0
    .if $defined("PRU0")
ICSS_EMAC_PROMISCOUS_BIT        .set    0
    .else
ICSS_EMAC_PROMISCOUS_BIT        .set    1
    .endif
;;***************************************************************************
;									    *
;									    *
;           Diagrams below show the memory usage for PRU DRAM               *
;		and shared RAM for switch and EMAC           		    *
;									    *
;									    *
;****************************************************************************

;;***************************************************************************
;									    *
;									    *
;                 Usage for switch.  Numbers indicate bytes                 *
;									    *
;									    *
;****************************************************************************

;          DRAM0                                             DRAM1                                        Shared RAM
;***********0x2000**************                  ************0x2000***************		*************0x3000**************
;**         Empty              *		  **          Empty               *	        **     Available for Protocol   *
;***********0x1FA2**************		  ***********0x1FA2****************		**             Usage            *
;**    Used for statistics     * 		  **      Used for statistics     *		*************0x2400**************
;**        and misc            *		  **         and misc             *		**       Reserved for future    *
;***********0x1F00**************                  ************0x1F00***************		**	    Usage               *
;**                            *		  **           Empty              *       	*************0x2010**************
;**                            *		  *************0x1E88**************		*   Multicast Filtering Table   *
;**                            *		  **       Switch Context         *		*************0X1C10**************
;**   Available for Protocol   *		  **         Stored here          *		**                              *
;**         Usage              *		  ************0X1D00***************		**                              *
;**                            *		  **                              *		**                              *
;**                            *		  **     Available for Protocol   *		**        BD Offsets for        *
;**                            *		  **           Usage              *		**           3 queues           *
;***********0x400***************		  *************0x400***************		**                              *
;**   Reserved for Future      *		  **     Reserved for Future      *		**                              *
;**          usage             *		  **            usage             *		**                              *
;***********0x0000**************		  *************0x0000**************		*************0x0000**************

;****************************************************************************
;								    	    *
;									    *
;									    *
;									    *
;                           End of Section                                  *
;									    *
;									    *
;									    *
;									    *
;									    *
;****************************************************************************


;;***************************************************************************
;									    *
;									    *
;                 Usage for EMAC.  Numbers indicate bytes                   *
;									    *
;									    *
;****************************************************************************

;          DRAM0                                        DRAM1                               Shared RAM
;***********0x2000**************          *************0x2000**************	    *************0x3000**************
;**         Empty              *		  **           Empty              *	    **           Empty              *
;***********0x1FA9**************		  *************0x1FA9**************	    **				*
;**    Used for statistics     * 		  **      Used for statistics     *	    *************0x1FA0**************
;**        and misc            *		  **         and misc             *	    **     Available for Protocol   *
;***********0x1F00**************          *************0x1F00**************     **			  Usage *
;**    Queue Context Offsets   *		  **     Queue Context Offsets    *     *************0x1CA0**************
;**    TTS Control Variables   *          **     TTS Control Variables	  *	    **				*
;***********0x1E98**************		  *************0x1E98**************	    **				*
;**                            *		  **                              *	    **				*
;**   Available for Protocol   *		  **     Available for Protocol   *	    **        BD Offsets for        *
;**         Usage              *		  **           Usage              *	    **     4 queues of 3 ports      *
;**                            *		  **                              *	    **                              *
;***********0x400***************		  *************0x400***************	    *************0x400***************
;**   VLAN Filtering           *		  **     VLAN filtering           *	    **     Reserved for Future      *
;**       usage                *		  **        usage                 *	    **          usage               *
;**********0x0200***************          ************0x0200***************     *************0x11C***************
;**   Multicast Filtering      *          **    Multicast Filtering       *     **     TimeSync Variables       *
;***********0x00F4**************          ************0x00F4***************     *************0x0008**************
;**   VLAN filter ctrl         *          **    VLAN filter ctrl          *     *                               *
;***********0x00EF**************          ***********0x00EF****************     **     Reserved for Future      *
;**   Reserved for Future      *		  **     Reserved for Future      *     **            usage             *
;**          usage             *		  **            usage             *     *                               *
;***********0x0000**************		  *************0x0000**************     *************0x0000**************

;********************************************************************************************************
;													*
;													*
;                         					  End of Section                        *
;													*
;													*
;********************************************************************************************************

;************************************************************************
;									*
;									*
;									*
;									*
;               Memory Usage of DRAM0/DRAM1 and Shared RAM              *
;									*
;									*
;									*
;									*
;									*
;************************************************************************
;The following offsets indicate which sections of the memory are available for protocols
;NOTE:Some memory is reserved for future use
SWITCH_PROTOCOL_SPECIFIC_DRAM0_MEMORY_SIZE	.set	0x1F00
SWITCH_PROTOCOL_SPECIFIC_DRAM0_MEMORY_OFFSET	.set	0x400

SWITCH_PROTOCOL_SPECIFIC_DRAM1_USAGE_SIZE	.set	0x1D00
SWITCH_PROTOCOL_SPECIFIC_DRAM1_MEMORY_OFFSET	.set	0x400

EMAC_PROTOCOL_SPECIFIC_DRAM_USAGE_SIZE	.set	0x1A98
EMAC_PROTOCOL_SPECIFIC_DRAM_MEMORY_OFFSET	.set	0x400

SWITCH_PROTOCOL_SPECIFIC_SRAM_USAGE_SIZE	.set	0x3000
SWITCH_PROTOCOL_SPECIFIC_SRAM_MEMORY_OFFSET	.set	0x2400

EMAC_PROTOCOL_SPECIFIC_SRAM_USAGE_SIZE	.set	0x300
EMAC_PROTOCOL_SPECIFIC_SRAM_MEMORY_OFFSET	.set	0x1CA0

;The following offsets indicate which sections of the memory are used for switch and EMAC internal tasks
SWITCH_SPECIFIC_DRAM0_START_SIZE	.set	0x100
SWITCH_SPECIFIC_DRAM0_START_OFFSET	.set	0x1F00

SWITCH_SPECIFIC_DRAM1_START_SIZE	.set	0x300
SWITCH_SPECIFIC_DRAM1_START_OFFSET	.set	0x1D00

EMAC_SPECIFIC_DRAM_USAGE_SIZE	.set	0x111
EMAC_SPECIFIC_DRAM_START_OFFSET	.set	0x1E98

SWITCH_SPECIFIC_SRAM_USAGE_SIZE	.set	0x2010
SWITCH_SPECIFIC_SRAM_START_OFFSET	.set	0x400

EMAC_SPECIFIC_SRAM_USAGE_SIZE	.set	0x18A0
EMAC_SPECIFIC_SRAM_START_OFFSET	.set	0x400

;********************************************************************************************************
;													*
;													*
;                         					  End of Section                        *
;													*
;													*
;********************************************************************************************************


;****************************************************************************
;*                       General Purpose Statistics                         *
;*              These are present on both PRU0 and PRU1 DRAM                *
;****************************************************************************
; base statistics offset
STATISTICS_OFFSET	.set	0x1F00

; MII port TX statistics flags
TX_BC_FRAMES_OFFSET	.set	0x0
TX_MC_FRAMES_OFFSET	.set					TX_BC_FRAMES_OFFSET + 0x4
TX_UC_FRAMES_OFFSET	.set					TX_MC_FRAMES_OFFSET + 0x4
TX_BYTE_CNT_OFFSET	.set					TX_UC_FRAMES_OFFSET + 0x4

; MII port RX statistics flags
RX_BC_FRAMES_OFFSET	.set	             TX_BYTE_CNT_OFFSET + 0x4
RX_MC_FRAMES_OFFSET	.set	             RX_BC_FRAMES_OFFSET + 0x4
RX_UC_FRAMES_OFFSET	.set	             RX_MC_FRAMES_OFFSET + 0x4
RX_BYTE_CNT_OFFSET	.set	              RX_UC_FRAMES_OFFSET + 0x4

;Binning counters : disabled by default
TX_64_BYTE_FRAME_OFFSET	.set	         RX_BYTE_CNT_OFFSET + 0x4
TX_65_127_BYTE_FRAME_OFFSET	.set	     TX_64_BYTE_FRAME_OFFSET + 0x4
TX_128_255_BYTE_FRAME_OFFSET	.set	    TX_65_127_BYTE_FRAME_OFFSET + 0x4
TX_256_511_BYTE_FRAME_OFFSET	.set	    TX_128_255_BYTE_FRAME_OFFSET + 0x4
TX_512_1023_BYTE_FRAME_OFFSET	.set	   TX_256_511_BYTE_FRAME_OFFSET + 0x4
TX_1024_MAX_BYTE_FRAME_OFFSET	.set	   TX_512_1023_BYTE_FRAME_OFFSET + 0x4

RX_64_BYTE_FRAME_OFFSET	.set	         TX_1024_MAX_BYTE_FRAME_OFFSET + 0x4
RX_65_127_BYTE_FRAME_OFFSET	.set	     RX_64_BYTE_FRAME_OFFSET + 0x4
RX_128_255_BYTE_FRAME_OFFSET	.set	    RX_65_127_BYTE_FRAME_OFFSET + 0x4
RX_256_511_BYTE_FRAME_OFFSET	.set	    RX_128_255_BYTE_FRAME_OFFSET + 0x4
RX_512_1023_BYTE_FRAME_OFFSET	.set	   RX_256_511_BYTE_FRAME_OFFSET + 0x4
RX_1024_MAX_BYTE_FRAME_OFFSET	.set	   RX_512_1023_BYTE_FRAME_OFFSET + 0x4

;**********************************Collision Counters*******************************/
COLLISION_COUNTERS_BASE_OFFSET	.set	0x1F50
LATE_COLLISION_OFFSET	.set	           COLLISION_COUNTERS_BASE_OFFSET       ;set for a frame where collision took place after 512 bytes were sent on wire
SINGLE_COLLISION_OFFSET	.set	         LATE_COLLISION_OFFSET + 0x4       ;frame was sent after a single collision was reported
MULTIPLE_COLLISION_OFFSET	.set	       SINGLE_COLLISION_OFFSET + 0x4       ;frame was sent after more than one collision
EXCESS_COLLISION_OFFSET	.set	         MULTIPLE_COLLISION_OFFSET + 0x4       ;more than 16 collisions
;***********************************************************************************/

;******************************Error packets****************************************/
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
;***********************************************************************************/

;****************Debug Statistics***************************************************/
TX_OVERFLOW_COUNTER	.set					RX_DROPPED_FRAMES_OFFSET + 0x4
TX_UNDERFLOW_COUNTER	.set				TX_OVERFLOW_COUNTER + 0x4
;***********************************************************************************/

;#define CS_ERROR_OFFSET               0x7c  HW Does not support at this Point of time
;#define SQE_TEST_ERROR_OFFSET         0x80  HW Does not support at this Point of time

;Total statistics size =  (TX_UNDERFLOW_COUNTER - STATISTICS_OFFSET + 0x4)
;Always keep it's value same as the last stat offset and update as and when new stats are added.
STAT_SIZE	.set	0x98

;****************************************************************************
;                          End of Statistics				    *
;****************************************************************************


;****************************************************************************
;*                           Offset for storing                             *
;				1. Storm Prevention Params           	    *
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

;***********************************************************************************************************
;											                   *
;											                   *
;                         					  End of Section                           *
;											                   *
;											                   *
;***********************************************************************************************************


;****************************************************************************
;									 *
;									 *
;									 *
;									 *
;                      DRAM1 Offsets for Switch                          *
;									 *
;									 *
;									 *
;									 *
;									 *
;****************************************************************************
;*****************Queue Descriptors********************
P2_QUEUE_DESC_OFFSET	.set	0x1EBC
P1_QUEUE_DESC_OFFSET	.set	0x1E9C
P0_QUEUE_DESC_OFFSET	.set	0x1E7C
P2_COL_QUEUE_DESC_OFFSET	.set	0x1E74
P1_COL_QUEUE_DESC_OFFSET	.set	0x1E6C
P0_COL_QUEUE_DESC_OFFSET	.set	0x1E64
COLLISION_STATUS_ADDR	.set	0x1E60
	; P1: bit 8 is pending flag, 9..10 is queue number
	; p2: bit 16 is pending flag, 17..18 is queue number, remaining bits are 0.

INTERFACE_MAC_ADDR	.set	0x1E58
P2_MAC_ADDR	.set	0x1E50
P1_MAC_ADDR	.set	0x1E48
; With dynamic configuration of queue size the offsets are variablel. For PRU to find which queue and descriptor needs to be served there
; is a look-up table with index of port and queue number.
; table definition to access queue size, buffer descriptor and buffer

; table offset for queue size: 3 ports * 4 Queues * 1 byte offset = 24 bytes
QUEUE_SIZE_ADDR	.set	0x1E30
; table offset for queue: 3 ports * 4 Queues * 2 byte offset = 24 bytes
QUEUE_OFFSET_ADDR	.set	0x1E18
; table offset for queue descriptors: 3 ports * 4 Queues * 2 byte offset = 24 bytes
QUEUE_DESCRIPTOR_OFFSET_ADDR	.set	0x1E00
; todo: remove this older definition
QUEUE_SIZE_TBL_ADDR	.set	0x1E00
	; 0x1E08: P0Q0 0x1E09: P0Q1 0x1E0A: P0Q2 0x1E0B: P0Q3
	; 0x1E04: P1Q0 0x1E05: P1Q1 0x1E06: P1Q2 0x1E07: P0Q3
	; 0x1E00: P0Q0 0x1E01: P0Q1 0x1E02: P0Q2 0x1E03: P0Q3

CONFIGURATION_OFFSET	.set	0x1E00

; Port 2 Rx Collision context
TX_CONTEXT_P1_Q1_OFFSET_ADDR	.set		SWITCH_SPECIFIC_DRAM1_START_OFFSET
; Port 1 Rx Collision context
P1_Q1_TX_CONTEXT_OFFSET	.set				TX_CONTEXT_P1_Q1_OFFSET_ADDR
; Host Port Collision Context
P1_Q2_TX_CONTEXT_OFFSET	.set				P1_Q1_TX_CONTEXT_OFFSET + 8

; Port 2 Rx Context
P1_Q3_TX_CONTEXT_OFFSET	.set				P1_Q2_TX_CONTEXT_OFFSET + 8
P1_Q4_TX_CONTEXT_OFFSET	.set				P1_Q3_TX_CONTEXT_OFFSET + 8
TX_CONTEXT_P2_Q1_OFFSET_ADDR	.set		P1_Q4_TX_CONTEXT_OFFSET + 8
P2_Q1_TX_CONTEXT_OFFSET	.set				TX_CONTEXT_P2_Q1_OFFSET_ADDR
P2_Q2_TX_CONTEXT_OFFSET	.set				P2_Q1_TX_CONTEXT_OFFSET + 8

; Port 1 Rx Context
P2_Q3_TX_CONTEXT_OFFSET	.set				P2_Q2_TX_CONTEXT_OFFSET + 8
P2_Q4_TX_CONTEXT_OFFSET	.set				P2_Q3_TX_CONTEXT_OFFSET + 8
COL_TX_CONTEXT_P1_Q1_OFFSET_ADDR	.set		P2_Q4_TX_CONTEXT_OFFSET + 8
COL_TX_CONTEXT_P2_Q1_OFFSET_ADDR	.set		COL_TX_CONTEXT_P1_Q1_OFFSET_ADDR + 8
RX_CONTEXT_P0_Q1_OFFSET_ADDR	.set		COL_TX_CONTEXT_P2_Q1_OFFSET_ADDR + 8

; Host Port Rx Context
P0_Q1_RX_CONTEXT_OFFSET	.set				RX_CONTEXT_P0_Q1_OFFSET_ADDR
P0_Q2_RX_CONTEXT_OFFSET	.set				P0_Q1_RX_CONTEXT_OFFSET + 8
P0_Q3_RX_CONTEXT_OFFSET	.set				P0_Q2_RX_CONTEXT_OFFSET + 8
P0_Q4_RX_CONTEXT_OFFSET	.set				P0_Q3_RX_CONTEXT_OFFSET + 8
RX_CONTEXT_P1_Q1_OFFSET_ADDR	.set		P0_Q4_RX_CONTEXT_OFFSET + 8

; Port 2 Tx Collision Context
P1_Q1_RX_CONTEXT_OFFSET	.set				RX_CONTEXT_P1_Q1_OFFSET_ADDR
; Port 1 Tx Collision Context
P1_Q2_RX_CONTEXT_OFFSET	.set				P1_Q1_RX_CONTEXT_OFFSET + 8

; Port 2
P1_Q3_RX_CONTEXT_OFFSET	.set				P1_Q2_RX_CONTEXT_OFFSET + 8
P1_Q4_RX_CONTEXT_OFFSET	.set				P1_Q3_RX_CONTEXT_OFFSET + 8
RX_CONTEXT_P2_Q1_OFFSET_ADDR	.set		P1_Q4_RX_CONTEXT_OFFSET + 8
P2_Q1_RX_CONTEXT_OFFSET	.set				RX_CONTEXT_P2_Q1_OFFSET_ADDR
P2_Q2_RX_CONTEXT_OFFSET	.set				P2_Q1_RX_CONTEXT_OFFSET + 8
; Port 1
P2_Q3_RX_CONTEXT_OFFSET	.set				P2_Q2_RX_CONTEXT_OFFSET + 8
P2_Q4_RX_CONTEXT_OFFSET	.set				P2_Q3_RX_CONTEXT_OFFSET + 8
COL_RX_CONTEXT_P0_OFFSET_ADDR	.set		P2_Q4_RX_CONTEXT_OFFSET + 8    ;10 bytes
COL_RX_CONTEXT_P1_OFFSET_ADDR	.set		COL_RX_CONTEXT_P0_OFFSET_ADDR + 12    ;10 bytes
COL_RX_CONTEXT_P2_OFFSET_ADDR	.set		COL_RX_CONTEXT_P1_OFFSET_ADDR + 12    ;10 bytes
;***********************************************************************************************************
;												           *
;												           *
;                         					  End of Section                           *
;												           *
;												           *
;***********************************************************************************************************


;****************************************************************************
;									    *
;									    *
;									    *
;									    *
;                     Shared RAM Offsets for Switch                         *
;									    *
;									    *
;									    *
;									    *
;								            *
;****************************************************************************
; ICSS Shared RAM 12kB
ICSS_EMAC_TTS_BASE_OFFSET	.set						EMAC_SPECIFIC_DRAM_START_OFFSET
ICSS_EMAC_TTS_CYCLE_START_OFFSET	.set				ICSS_EMAC_TTS_BASE_OFFSET						 ;8 bytes, to store counter value of trigger

ICSS_EMAC_TTS_CYCLE_PERIOD_OFFSET	.set				ICSS_EMAC_TTS_CYCLE_START_OFFSET + 8			 ;4 bytes, to store cycle period (max: 4.2s)
ICSS_EMAC_TTS_CFG_TIME_OFFSET	.set					ICSS_EMAC_TTS_CYCLE_PERIOD_OFFSET + 4			 ;4 bytes, to store cfg time (max: 4.2s)
ICSS_EMAC_TTS_STATUS_OFFSET	.set						ICSS_EMAC_TTS_CFG_TIME_OFFSET + 4				 ;1 byte, to store TTS status, 3 bytes, for future use
ICSS_EMAC_TTS_MISSED_CYCLE_CNT_OFFSET	.set			ICSS_EMAC_TTS_STATUS_OFFSET + 4					 ;4 bytes, to store no. of cycles missed by RT frames (max: 4.2Bn)

ICSS_EMAC_TTS_PREV_TX_SOF	.set						ICSS_EMAC_TTS_MISSED_CYCLE_CNT_OFFSET + 4		 ;8 bytes, to store previous TX SOF.
ICSS_EMAC_TTS_CYC_TX_SOF	.set						ICSS_EMAC_TTS_PREV_TX_SOF + 8					 ;8 bytes, to store cyclic pkt TX SOF.
PORT_QUEUE_DESC_OFFSET	.set			    ICSS_EMAC_TTS_CYC_TX_SOF + 8				 ; 4 queue descriptors for port tx. 32 bytes
TX_CONTEXT_Q1_OFFSET_ADDR	.set		    PORT_QUEUE_DESC_OFFSET + 32

;***********************************************************************************************************
;													    *
;													    *
;                         					  End of Section                            *
;													    *
;													    *
;***********************************************************************************************************

;****************************************************************************
;									    *
;									    *
;									    *
;									    *
;                         DRAM Offsets for EMAC                             *
;		    	Present on Both DRAM0 and DRAM1          	    *
;									    *
;									    *
;									    *
;									    *
;****************************************************************************
; table offset for Port queue descriptors: 1 ports * 4 Queues * 2 byte offset = 8 bytes
Q1_TX_CONTEXT_OFFSET	.set				TX_CONTEXT_Q1_OFFSET_ADDR
Q2_TX_CONTEXT_OFFSET	.set				Q1_TX_CONTEXT_OFFSET + 8
Q3_TX_CONTEXT_OFFSET	.set				Q2_TX_CONTEXT_OFFSET + 8
Q4_TX_CONTEXT_OFFSET	.set				Q3_TX_CONTEXT_OFFSET + 8
HOST_BD_SIZE	.set	           (HOST_QUEUE_1_SIZE + HOST_QUEUE_2_SIZE + HOST_QUEUE_3_SIZE + HOST_QUEUE_4_SIZE) * BD_SIZE
PORT_BD_SIZE	.set	           (QUEUE_1_SIZE + QUEUE_2_SIZE + QUEUE_3_SIZE + QUEUE_4_SIZE) * 2 * BD_SIZE

;EMAC Time Triggered Send Offsets
P0_BUFFER_DESC_OFFSET	.set		SWITCH_SPECIFIC_SRAM_START_OFFSET
EOF_48K_BUFFER_BD	.set	       P0_BUFFER_DESC_OFFSET + HOST_BD_SIZE + PORT_BD_SIZE
P0_COL_BD_OFFSET	.set			EOF_48K_BUFFER_BD
EOF_COL_BUFFER_BD	.set	       P0_COL_BD_OFFSET + 3* BD_SIZE * 48

;***********************************************************************************************************
;													    *
;													    *
;                         					  End of Section                            *
;													    *
;													    *
;***********************************************************************************************************

;****************************************************************************
;									    *
;									    *
;									    *
;									    *
;                     Shared RAM offsets for EMAC                           *
;									    *
;									    *
;									    *
;									    *
;									    *
;****************************************************************************
;------------------------------End of queue descriptors---------------------------;

; table offset for queue: 4 Queues * 2 byte offset = 8 bytes
P1_COL_BD_OFFSET	.set			P0_COL_BD_OFFSET  + BD_SIZE * 48
; table offset for Host queue descriptors: 1 ports * 4 Queues * 2 byte offset = 8 bytes
P2_COL_BD_OFFSET	.set			P1_COL_BD_OFFSET  + BD_SIZE * 48

;EMAC Firmware Version Information
ICSS_EMAC_FIRMWARE_RELEASE_1_OFFSET	.set			0
ICSS_EMAC_FIRMWARE_RELEASE_2_OFFSET	.set			ICSS_EMAC_FIRMWARE_RELEASE_1_OFFSET + 4
ICSS_EMAC_FIRMWARE_FEATURE_OFFSET	.set			ICSS_EMAC_FIRMWARE_RELEASE_2_OFFSET + 4
ICSS_EMAC_FIRMWARE_RESERVED_FEATURE_OFFSET	.set	        ICSS_EMAC_FIRMWARE_FEATURE_OFFSET + 4
; Host Port Rx Context
;Promiscuous Feature offset
ICSS_EMAC_FIRMWARE_PROMISCUOUS_MODE_OFFSET	.set	                EOF_48K_BUFFER_BD + 4

HOST_Q1_RX_CONTEXT_OFFSET	.set					ICSS_EMAC_FIRMWARE_PROMISCUOUS_MODE_OFFSET + 4
HOST_Q2_RX_CONTEXT_OFFSET	.set					HOST_Q1_RX_CONTEXT_OFFSET + 8
HOST_Q3_RX_CONTEXT_OFFSET	.set					HOST_Q2_RX_CONTEXT_OFFSET + 8
HOST_Q4_RX_CONTEXT_OFFSET	.set					HOST_Q3_RX_CONTEXT_OFFSET + 8

;***********************************************************************************************************
;												            *
;													    *
;                         					  End of Section                            *
;													    *
;													    *
;***********************************************************************************************************


;****************************************************************************
;									    *
;									    *
;									    *
;									    *
;             Shared RAM offsets for both Switch and EMAC                   *
;									    *
;									    *
;									    *
;									    *
;								            *
;****************************************************************************

; allow for max 48k switch buffer which spans the descriptors up to 0x1800 6kB
HOST_QUEUE_DESCRIPTOR_OFFSET_ADDR	.set		HOST_Q4_RX_CONTEXT_OFFSET + 8			 ;8 bytes

HOST_QUEUE_OFFSET_ADDR	.set					HOST_QUEUE_DESCRIPTOR_OFFSET_ADDR + 8
HOST_QUEUE_SIZE_ADDR	.set					HOST_QUEUE_OFFSET_ADDR + 8

HOST_QUEUE_DESC_OFFSET	.set					HOST_QUEUE_SIZE_ADDR + 16			 ; 4 queue descriptors for port 0 (host receive). 32 bytes
P0_Q1_BD_OFFSET	.set				P0_BUFFER_DESC_OFFSET
P0_Q2_BD_OFFSET	.set				P0_Q1_BD_OFFSET + HOST_QUEUE_1_SIZE * BD_SIZE
P0_Q3_BD_OFFSET	.set				P0_Q2_BD_OFFSET + HOST_QUEUE_2_SIZE * BD_SIZE
P0_Q4_BD_OFFSET	.set				P0_Q3_BD_OFFSET + HOST_QUEUE_3_SIZE * BD_SIZE
P1_Q1_BD_OFFSET	.set				P0_Q4_BD_OFFSET + HOST_QUEUE_4_SIZE * BD_SIZE
P1_Q2_BD_OFFSET	.set				P1_Q1_BD_OFFSET + QUEUE_1_SIZE * BD_SIZE
P1_Q3_BD_OFFSET	.set				P1_Q2_BD_OFFSET + QUEUE_2_SIZE * BD_SIZE
P1_Q4_BD_OFFSET	.set				P1_Q3_BD_OFFSET + QUEUE_3_SIZE * BD_SIZE
P2_Q1_BD_OFFSET	.set				P1_Q4_BD_OFFSET + QUEUE_4_SIZE * BD_SIZE
P2_Q2_BD_OFFSET	.set				P2_Q1_BD_OFFSET + QUEUE_1_SIZE * BD_SIZE

P2_Q3_BD_OFFSET	.set				P2_Q2_BD_OFFSET + QUEUE_2_SIZE * BD_SIZE
P2_Q4_BD_OFFSET	.set				P2_Q3_BD_OFFSET + QUEUE_3_SIZE * BD_SIZE

;***********************************************************************************************************
;													    *
;													    *
;                         					  End of Section                            *
;													    *
;													    *
;***********************************************************************************************************


;****************************************************************************
;									    *
;									    *
;									    *
;									    *
;                    Memory Usage of L3 OCMC RAM                            *
;									    *
;									    *
;									    *
;									    *
;									    *
;****************************************************************************
; L3 64KB Memory - mainly buffer Pool
; put collision buffer at end of L3 memory. Simplifies PRU coding to be on same memory as queue buffer
P2_COL_BUFFER_OFFSET	.set	0xFA00
P1_COL_BUFFER_OFFSET	.set	0xF400
P0_COL_BUFFER_OFFSET	.set	0xEE00

P2_BUFFER_POOL_SIZE	.set				((QUEUE_1_SIZE + QUEUE_2_SIZE + QUEUE_3_SIZE + QUEUE_4_SIZE) * BLOCK_SIZE)
P1_BUFFER_POOL_SIZE	.set				((QUEUE_1_SIZE + QUEUE_2_SIZE + QUEUE_3_SIZE + QUEUE_4_SIZE) * BLOCK_SIZE)
P0_BUFFER_POOL_SIZE	.set				((HOST_QUEUE_1_SIZE + HOST_QUEUE_2_SIZE + HOST_QUEUE_3_SIZE + HOST_QUEUE_4_SIZE) * BLOCK_SIZE)
EMAC_L3_BUFFER_POOL_SIZE	.set		(P0_BUFFER_POOL_SIZE + P1_BUFFER_POOL_SIZE + P2_BUFFER_POOL_SIZE)

P0_Q1_BUFFER_OFFSET	.set	0x0000
P0_Q2_BUFFER_OFFSET	.set	     P0_Q1_BUFFER_OFFSET + HOST_QUEUE_1_SIZE * BLOCK_SIZE
P0_Q3_BUFFER_OFFSET	.set	     P0_Q2_BUFFER_OFFSET + HOST_QUEUE_2_SIZE * BLOCK_SIZE
P0_Q4_BUFFER_OFFSET	.set	     P0_Q3_BUFFER_OFFSET + HOST_QUEUE_3_SIZE * BLOCK_SIZE
P1_Q1_BUFFER_OFFSET	.set	     P0_Q4_BUFFER_OFFSET + HOST_QUEUE_4_SIZE * BLOCK_SIZE
P1_Q2_BUFFER_OFFSET	.set	     P1_Q1_BUFFER_OFFSET + QUEUE_1_SIZE * BLOCK_SIZE
P1_Q3_BUFFER_OFFSET	.set	     P1_Q2_BUFFER_OFFSET + QUEUE_2_SIZE * BLOCK_SIZE
P1_Q4_BUFFER_OFFSET	.set	     P1_Q3_BUFFER_OFFSET + QUEUE_3_SIZE * BLOCK_SIZE
P2_Q1_BUFFER_OFFSET	.set	     P1_Q4_BUFFER_OFFSET + QUEUE_4_SIZE * BLOCK_SIZE
P2_Q2_BUFFER_OFFSET	.set	     P2_Q1_BUFFER_OFFSET + QUEUE_1_SIZE * BLOCK_SIZE
P2_Q3_BUFFER_OFFSET	.set	     P2_Q2_BUFFER_OFFSET + QUEUE_2_SIZE * BLOCK_SIZE
P2_Q4_BUFFER_OFFSET	.set	     P2_Q3_BUFFER_OFFSET + QUEUE_3_SIZE * BLOCK_SIZE
END_OF_BUFFER_POOL	.set			P2_Q4_BUFFER_OFFSET + QUEUE_4_SIZE * BLOCK_SIZE

;***********************************************************************************************************
;													    *
;													    *
;                         					  End of Section                            *
;													    *
;													    *
;***********************************************************************************************************

        .if $defined("PRU")
; Collision Status Register
struct_collision_status	.struct
port0	.ubyte	; port 0 pending and queue number
port1	.ubyte	; port 1 pending and queue number
port2	.ubyte	; port 2 pending and queue number
	.endstruct

	.asg	t0, P0_COLLISION_PENDING
	.asg	t8, P1_COLLISION_PENDING
	.asg	t16, P2_COLLISION_PENDING


	.endif

; switch configuration
; enable ports : receive enable , transmit enable, block, learning, flush table, age out table (older are removed), tear
; rate limiting setup
; filter database setup for MC
; learn multicast option one for both ports
; learn broadcast option one for both ports
; statistics clear
	.endif	;ICSS_EMAC_SWITCH__H


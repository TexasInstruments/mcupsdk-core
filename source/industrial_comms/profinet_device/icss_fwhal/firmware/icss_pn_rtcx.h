/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//*****************************************************************************/
// file:   icss_pn_rtcx.h
//
// brief:  Definitions and mapping of Profinet cyclic data exchange for RT and IRT
//         Includes:
//         1. Sendlist and Receivelist with shadow operation
//         2. Descriptors in lists for cyclic data exchange
//         3. Configuration for cyclic data exchange and start-up
//         4. Configuration for IRT phase management
//	       5. Parameter exchange between host and PRU for time sync
//	       6. Events for cyclic data exchange
//

#ifndef __ICSS_PN_RTCX__
#define __ICSS_PN_RTCX__

#ifdef __cplusplus
extern "C"
{
#endif


// Target switches: PRU / ARM
// Protocol switches: PROFINET , PN_IOD, PN_IOC
//
// Global settings

#ifndef		PN_IOD
#define 	PN_IOD
#endif

// IOD Definitions
#ifdef PN_IOD
 #define NO_PPM		8
 #define NO_CPM		8
 #define NO_PM 		8 		// FW driver uses this number for CPM/PPM size!
 #define NO_AR		8
#else
// IOC definitions
 #define NO_PPM		256
 #define NO_CPM		256
 #define NO_AR		256
#endif

// CONSTANT ARE DEFINED HERE
// Yellow safety margin is 640 ns.
#define YELLOW_SAFETY_MARGIN		0x0280
#define DCP_IDENTREQ_FRAMEID		0xfefe
#define DCP_NAME_CMP_NO_OF_CHAR 	8
// offset of the first character of dcp name in incoming dcp frame assuming it is in R2 register.
#define DCP_NAME_FRAME_PTR_OFFSET	0x8
// offset of the first character of the set of "last eight characters" of slave name assuming it is read in R10 register
#define DCP_NAME_SLAVE_PTR_OFFSET	0x28

#define RTC_3125_CLK_CONST			0x7A12		// const for 31.25us cycle

#define MAX_BRIDGE_DELAY			3020        // 2920 + 100ns

#define RTC3_PPM_FIFO_PUSH_MARGIN	6720

// Common scratch pad usage for CPM DHT in bank0
// r29.t0 indicates critical section of DHT updates in pn_rtc1_schedule
// when set to 1 indicates that DHT cannot be modified in pn_rtc1_receive
// when set to 0 indicates DHT can be overwritten. The critical section needs to consider
// latency of SBCO command which is like a 2-1-1-1 burst for 16 bytes of DHT update.

#define PN_CRITICAL_DHT_REG		R29.b0
#define PN_CRITICAL_DHT_FLAG	t0

// Sendlist (PPMs) and Receivelist (CPMs)
//
// Cyclic data is exchanged using separate lists for send and receive. For dynamic handling of connections there is a shadow
// list which can be prepared by host while active list is processed by PRU on send and receive. There is an index in
// RTC_CPM_INDEX and RTC_PPM_INDEX indicating which list is active.
// Each list is pre-sorted by period, port and time. The index for period and port is also given in index register.
// Index register: entries in this register are position in list (0-7) * 12 bytes. Simplifies PRU code as *12 is not easily done
// with shift.
//

// RTC_CPM_INDEX_L1/2 / RTC_PPM_INDEX_L1/2
// Bit		Name				Description
//

// 0..7		RTC_RED_PORT_IDX	points to first port 2 descriptor in red period
// 8..15    RTC_GREEN_IDX		points to first descriptor of green period,
// 16..23	RTC_GREEN_PORT_IDX	points to first port 2 descriptor in green period
// 24..31	RTC_GREEN_END_IDX	points to last descriptor of green period

// RTC_LIST_INDEX_OFFSET
// Bit		Name				Description
// 0		RTC_CPM_IDX_PRU		Index which CPM list is used. 0 = list 1, 1 = list 2
// 1 		RTC_PPM_IDX_PRU
// 8		RTC_CPM_IDX_HOST	Index which PPM list is used. 0 = list 1, 1 = list 2
// 9 		RTC_PPM_IDX_HOST

// All the following offsets are applied to RTC_CONFIG_OFFSET which is after PPM_LIST on PRU0 DMEM.
// 8 bytes for DHT expire source. Each CPM has one byte to indicate if it's DHT expired

#define RTC_NOTIFY_DHT_EXPIRE_OFFSET	0x10   // 8 bytes

#define RTC_PPM_INDEX_L1_OFFSET 	RTC_NOTIFY_DHT_EXPIRE_OFFSET +8
#define RTC_PPM_INDEX_L2_OFFSET		RTC_PPM_INDEX_L1_OFFSET + 4
#define RTC_LIST_INDEX_OFFSET		RTC_PPM_INDEX_L2_OFFSET + 4


#ifdef PRU
#define RTC_RED_PORT_IDX		b0
#define RTC_GREEN_IDX			b1
#define RTC_GREEN_PORT_IDX		b2
#define RTC_GREEN_END_IDX		b3
// there is only one pru writing to this field at different times - assumes CPM and PPM are on same port.
#define RTC_CPM_IDX_PRU			t0   // host read, pru write
#define RTC_PPM_IDX_PRU			t1	 // host read, pru write

#define RTC_CPM_IDX_HOST		t8	 // host write, pru read
#define RTC_PPM_IDX_HOST		t9	 // host write, pru read
#else
#define RTC_RED_PORT_IDX		0
#define RTC_GREEN_IDX			1	// byte offset to rtc_index_ptr
#define RTC_GREEN_PORT_IDX		2
#define RTC_GREEN_END_IDX		3
#endif

// directions - make sure all lists are using this defines
#define CPM	0
#define PPM 1

// CRITICAL mapping from 0x0012 to 0x0020 which allows for single PRU reads of CPM and PPM configuration

// CPM monitoring has activation bit which needs to be set per CPM.
// PPM is removed on CPM failure. This function is also activated.
// when RTC_CPM_ACTIVE_x bit is set then monitoring of incoming packet is activated including DHT
// when RTC_PPM_ACTIVE_x bit is set then PPM is stopped on corresponding CPM error.
// This is for IOD only. IOC needs 256 bits per direction.

#define RTC_CPM_ACTIVE_OFFSET		RTC_LIST_INDEX_OFFSET + 2

#ifdef PRU
#define RTC_CPM_ACTIVE_1		t0
#define RTC_CPM_ACTIVE_2		t1
#define RTC_CPM_ACTIVE_3		t2
#define RTC_CPM_ACTIVE_4		t3
#define RTC_CPM_ACTIVE_5		t4
#define RTC_CPM_ACTIVE_6		t5
#define RTC_CPM_ACTIVE_7		t6
#define RTC_CPM_ACTIVE_8		t7
#define RTC_PPM_ACTIVE_1		t0
#define RTC_PPM_ACTIVE_2		t1
#define RTC_PPM_ACTIVE_3		t2
#define RTC_PPM_ACTIVE_4		t3
#define RTC_PPM_ACTIVE_5		t4
#define RTC_PPM_ACTIVE_6		t5
#define RTC_PPM_ACTIVE_7		t6
#define RTC_PPM_ACTIVE_8		t7
#endif

// CPM/PPM STATUS
// consumer and provider can be of status ERROR or OK/RUN. For this one bit per connection is reserved in
// Status register for CPM and PPM.
#define RTC_PPM_OK				1
#define RTC_PPM_ERROR			0
#define RTC_CPM_RUN				1
#define	RTC_CPM_FAILURE			0
#define RTC_CPM_STATUS_OFFSET	RTC_CPM_ACTIVE_OFFSET + 1
#define RTC_PPM_STATUS_OFFSET   RTC_CPM_STATUS_OFFSET + 2

#define RTC_EXECUTE_DHT_CODE    RTC_PPM_STATUS_OFFSET + 2

// PPM_SEND_STATUS is defined as
// 0 = PPM not sent
// 0xFF = PPM descriptor not yet read from PRU b
// 0x10 = PPM send active (PRU is transferring packet from PPM buffer to MII TX fifo)
// 0x20 = PPM send complete (PRU has pushed last bytes of packet into MII TX fifo)
// state transitions:
// 0 -> 1 : PRU sets to one at the beginning of PPM send, host does not write to PPM_SEND_STATUS when value is 0 or 1
// 1 -> 2 : PRU sets to two at the end of PPM send. host can now reset to 0
// 2 -> 0 : host received interrupt or polls state and sees a value of 2. Host then resets state to 0
// 2 -> 1 : in case host did not see last packet send complete and PRU needs to send another packet.
// all other transitions are not defined. Normal sequence is 0 -> 1 -> 2 -> 0.

#define RTC_PPM_SEND_STATUS_OFFSET  RTC_EXECUTE_DHT_CODE + 1 	  // 8 bytes ... 1 byte for each PPM

// Buffer locked indication for CPM index indicated to consumer is locked, provider will not overwrite until cleared
// CPM: one byte per CPM set and cleared by host, 00 = buffer 1, 01 = buffer 2, 02 = buffer 3, 03 = no locked

// moved to free space as it is now 1 byte per CPM.
#define RTC_CPM_BUFFER_LOCK_OFFSET	0x30 //RTC_PPM_SEND_STATUS_OFFSET + 8   // 8 bytes ... 1 byte for each CPM

// Register to indicate fdb_flush mode - 1 byte. 0 = not active, 1 - fdb_flush mode active
#define RTC_MRP_FDB_FLUSH_OFFSET			RTC_CPM_BUFFER_LOCK_OFFSET + 9

// Base clock is used to schedule provider data in RT and IRT. For RT there is a min base clock of 250us.
// RTC_IRT_BASE_CLK: is multiplier of 31.25 us (default = 8 which corresponds to 250 us base clock)
// Base clock is given as a shift value as send_clock_factor is power of 2. E.g. value of 3 in this field
// means send_clock_Factor of 8 = 250 us base clock. Value of 5 provides 1 ms base clock.

#define PRU0_PHASE_EVENT_OFFSET				RTC_MRP_FDB_FLUSH_OFFSET + 1		// 8 bit to separate writes
#define PRU1_PHASE_EVENT_OFFSET				PRU0_PHASE_EVENT_OFFSET + 1

#define RTC_PHASE_COUNTER_OFFSET			PRU1_PHASE_EVENT_OFFSET + 3 		// 16 bit for phase counter
#define RTC_SEND_LIST_P1_OFFSET				RTC_PHASE_COUNTER_OFFSET + 2
#define RTC_SEND_LIST_P2_OFFSET				RTC_SEND_LIST_P1_OFFSET + 8

// SCF = send clock factor is power of 2. Up to 4096 (128ms) with RTC1 and 32 (1ms)with RTC3
// RTC_SCF and RTC_CYLE_COUNTER need to be next to each other as used with single LBCO!!!

#define RTC_SCF_OFFSET				RTC_SEND_LIST_P2_OFFSET + 8
#define RTC_CYCLE_COUNTER_OFFSET	RTC_SCF_OFFSET + 2
#define RTC_BASE_CLK_OFFSET			RTC_CYCLE_COUNTER_OFFSET + 2
#define PTCP_ABS_COUNTER_OFFSET 	RTC_BASE_CLK_OFFSET + 4 			// moved here for optimization

// RTC3 frames need to check for valid FRAME ID range also called red guard
// FID is 16 bit
#define RTC3_SOF_RedFrameID_OFFSET	0x5c//PTCP_ABS_COUNTER_OFFSET + 4
#define RTC3_EOF_RedFrameID_OFFSET  RTC3_SOF_RedFrameID_OFFSET + 2

#define RTC_DEVICE_SYNC_STATUS_OFFSET			RTC3_EOF_RedFrameID_OFFSET + 2
#define RTC_YELLOW_PERIOD_CONFIGURED_OFFSET		RTC_DEVICE_SYNC_STATUS_OFFSET + 1
#define PORT1_STATUS_OFFSET RTC_YELLOW_PERIOD_CONFIGURED_OFFSET + 1
#define PORT2_STATUS_OFFSET	PORT1_STATUS_OFFSET + 1

#define MAXLINE_RXDELAY_P1_OFFSET	PORT2_STATUS_OFFSET + 1
#define MAXLINE_RXDELAY_P2_OFFSET	MAXLINE_RXDELAY_P1_OFFSET + 4

// Bytes where PRU's store the index of active list using which send list has been prepared
// These are then used by the RTC Send code
#define PRU0_ACTIVE_LIST_INDEX_OFFSET		MAXLINE_RXDELAY_P2_OFFSET + 4
#define PRU1_ACTIVE_LIST_INDEX_OFFSET		PRU0_ACTIVE_LIST_INDEX_OFFSET + 1

// Below registers are used by PRU firmware for internal usage
#define RTC_SCH_EXECUTED_HALF_PRU0_OFFSET	PRU1_ACTIVE_LIST_INDEX_OFFSET + 1   // 2 bytes
#define RTC_SCH_EXECUTED_HALF_PRU1_OFFSET	RTC_SCH_EXECUTED_HALF_PRU0_OFFSET + 2   // 2 bytes

#define MRP_PORT1_STATE_OFFSET		0x72//RTC_SCH_EXECUTED_HALF_PRU1_OFFSET + 2
#define MRP_PORT2_STATE_OFFSET		MRP_PORT1_STATE_OFFSET + 1

#define RTC_YELLOW_PRD_START_TIME_OFFSET				MRP_PORT2_STATE_OFFSET + 1    // 4 bytes
#define RTC_YELLOW_SAFETY_MARGIN_PRD_START_TIME_OFFSET	RTC_YELLOW_PRD_START_TIME_OFFSET + 4    // 4 bytes
#define RTC_BASE_CLK_CHANGED_OFFSET						RTC_YELLOW_SAFETY_MARGIN_PRD_START_TIME_OFFSET + 4    // 1 byte
// Free byte x006d

#define RTC_SEND_EXECUTED_HALF_PRU0_OFFSET	RTC_BASE_CLK_CHANGED_OFFSET + 2
#define RTC_SEND_EXECUTED_HALF_PRU1_OFFSET	RTC_SEND_EXECUTED_HALF_PRU0_OFFSET + 1

// Parameters for IRT relative forwarder
// There are 5 configurations for GreenPeriodBegin per port and direction. Max value is 0x7A120 equal to 500us.
// If 5 configurations then one is 0 which means no red period in this phase.
// Orange period is leagay and should be supported.

#define RTC_GREEN_BEGIN_P1_1_RX_OFFSET	0x80//RTC_SEND_EXECUTED_HALF_PRU1_OFFSET + 1
#define RTC_GREEN_BEGIN_P1_1_TX_OFFSET	RTC_GREEN_BEGIN_P1_1_RX_OFFSET + 4
#define RTC_GREEN_BEGIN_P1_2_RX_OFFSET	RTC_GREEN_BEGIN_P1_1_TX_OFFSET + 4
#define RTC_GREEN_BEGIN_P1_2_TX_OFFSET	RTC_GREEN_BEGIN_P1_2_RX_OFFSET + 4
#define RTC_GREEN_BEGIN_P1_3_RX_OFFSET	RTC_GREEN_BEGIN_P1_2_TX_OFFSET + 4
#define RTC_GREEN_BEGIN_P1_3_TX_OFFSET	RTC_GREEN_BEGIN_P1_3_RX_OFFSET + 4
#define RTC_GREEN_BEGIN_P1_4_RX_OFFSET	RTC_GREEN_BEGIN_P1_3_TX_OFFSET + 4
#define RTC_GREEN_BEGIN_P1_4_TX_OFFSET	RTC_GREEN_BEGIN_P1_4_RX_OFFSET + 4
#define RTC_GREEN_BEGIN_P1_5_RX_OFFSET	RTC_GREEN_BEGIN_P1_4_TX_OFFSET + 4
#define RTC_GREEN_BEGIN_P1_5_TX_OFFSET	RTC_GREEN_BEGIN_P1_5_RX_OFFSET + 4

#define RTC_GREEN_BEGIN_P2_1_RX_OFFSET	0xA8//RTC_GREEN_BEGIN_P1_5_TX_OFFSET + 4
#define RTC_GREEN_BEGIN_P2_1_TX_OFFSET	RTC_GREEN_BEGIN_P2_1_RX_OFFSET + 4
#define RTC_GREEN_BEGIN_P2_2_RX_OFFSET	RTC_GREEN_BEGIN_P2_1_TX_OFFSET + 4
#define RTC_GREEN_BEGIN_P2_2_TX_OFFSET	RTC_GREEN_BEGIN_P2_2_RX_OFFSET + 4
#define RTC_GREEN_BEGIN_P2_3_RX_OFFSET	RTC_GREEN_BEGIN_P2_2_TX_OFFSET + 4
#define RTC_GREEN_BEGIN_P2_3_TX_OFFSET	RTC_GREEN_BEGIN_P2_3_RX_OFFSET + 4
#define RTC_GREEN_BEGIN_P2_4_RX_OFFSET	RTC_GREEN_BEGIN_P2_3_TX_OFFSET + 4
#define RTC_GREEN_BEGIN_P2_4_TX_OFFSET	RTC_GREEN_BEGIN_P2_4_RX_OFFSET + 4
#define RTC_GREEN_BEGIN_P2_5_RX_OFFSET	RTC_GREEN_BEGIN_P2_4_TX_OFFSET + 4
#define RTC_GREEN_BEGIN_P2_5_TX_OFFSET	RTC_GREEN_BEGIN_P2_5_RX_OFFSET + 4

// Each setting can be mapped to up to 16 phases. There are 3 bits required per mapping. Total 48 bits or 6 bytes
// bits 0..2 index of RTC_GREEN_BEGIN_Px_y_OFFSET for phase 1
// ...
// bits 45..47 index of RTC_GREEN_BEGIN_Px_y_OFFSET for phase 16

#define RTC_PAHSE_MAPPING_OFFSET		0xD0//RTC_GREEN_BEGIN_P2_5_TX_OFFSET + 4

// 4 bytes for each phase.. 64 bytes in total. Next data location is 0x100

#define PN_DCPF_NAME_OFFSET			RTC_PAHSE_MAPPING_OFFSET + 64   // 8 bytes
#define	PN_DCPF_NAME_LENGTH_OFFSET 	PN_DCPF_NAME_OFFSET + 8 	// 8 bit

// Previous cycle counter per CPM is verified with current cycle counter. Not to remember previous
// counter per CPM. For 8 CPMs this is 8 * 16 bit = 16 bytes
#define CPM_PREV_CYCLE_COUNTER_OFFSET	PN_DCPF_NAME_LENGTH_OFFSET + 4

// Monitoring of incoming packets is done with DHT. Time-out value is specified by host in RTC mode
// default setting is 3 (sendcycles). Max value is 255. There is one DHT per CPM.
// todo: need 16 bit values and decouple from location from RTC_CPM_ACTIVE_OFFSET
//  RTC_CPM_DHT_OFFSET is updated when CPM is active with every scf*31.25us
//  RTC_CPM_DHT_TIME_OUT_OFFSET is fixed timeout set by host when new CPM is activated
// 16 bit timout/value * 8 CPM = 16 bytes each

#define RTC_CPM_DHT_OFFSET				CPM_PREV_CYCLE_COUNTER_OFFSET + 16
#define RTC_DHT_TIMEOUT_OFFSET			RTC_CPM_DHT_OFFSET + 16

// AR Group - PPM mapping- up to 8 groups. Each group can have up to 8 PPMs which are indicated in this inverted mask
// we use this mask and XOR with PPM active field
// byte 0  = AR Group 1
// ..
// byte 7  = AR Group 8
// bit 0 = 0 PPM 1 belongs to this group
// bit 0 = 1 PPM 1 does not belong to this group
// ...
// bit 7 = 1 PPM 7 belongs to this group

#define RTC_AR_GROUP_PPM_OFFSET 		RTC_DHT_TIMEOUT_OFFSET + 16

// for faster detection of AR group each CPM has one byte which gives the group number it belongs to
// byte 0 (CPM 1) = AR Group number 0..7 indicating group number 1..8
// ...
// byte 7 (CPM 8) = AR Group number 1..8

#define	RTC_CPM_AR_GROUP_OFFSET		RTC_AR_GROUP_PPM_OFFSET + 8

#define RTC_NOTIFY_DHT_EVENT_OFFSET				RTC_CPM_AR_GROUP_OFFSET + 8 		// signals the DHT event (see below)
#define RTC_NOTIFY_LIST_TOGGLE_EVENT_OFFSET		RTC_NOTIFY_DHT_EVENT_OFFSET + 10 		// signals the PPM List toggle event (see below)
#define RTC_NOTIFY_DHT_EXPIRE					0x01		// indicate DHT expire event
#define RTC_NOTIFY_CPM_CRC						0x02		// indicate CPM CRC
#define RTC_NOTIFY_CPM_SEQ						0x03		// CPM sequence counter error
#define RTC_NOTIFY_CPM_STAT						0x04		// CPM status error
#define RTC_NOTIFY_CPM_LIST_CHANGE				0x05		// CPM List changed
#define RTC_NOTIFY_PPM_LIST_CHANGE				0x06		// PPM List changed
#define RTC_NOTIFY_ALL_LIST_CHANGE				0x0B		// spcial case: both lists changed

// One byte for each PPM. This will allow to resolve a race condition where PPM Active is cleared when DHT expires
// and it is re-written when the list toggle happens.
#define RTC_PPM_ACTIVE_OFFSET		0x160//RTC_NOTIFY_LIST_TOGGLE_EVENT_OFFSET + 3

// Free space - 0x00158 = 4 Bytes
#define RTC_PPM_ACTIVE_SHADOW_OFFSET		RTC_PPM_ACTIVE_OFFSET + 8

// Storage space for the CPM buffer addresses for CPM connections. Max 8 AR's -> 8*3 possible number of buffer addresses.
// Each CPM connection would have three buffers allocated. Since, actual CPM frame length may be smaller than statically allocated
// buffer size it is necessary to store the offset addresses of all the three buffers. Total space used = 24*2= 48 bytes.
#define RTC_CPM_BUFFER_ADDRESSES_OFFSET  	RTC_PPM_ACTIVE_SHADOW_OFFSET + 8 //0x0160

#define ISOM_TIO_TIMEVAL1                    0x190
#define ISOM_TIO_DURATION1                   ISOM_TIO_TIMEVAL1 + 4
#define ISOM_TIO_TYPE1                       ISOM_TIO_DURATION1 + 4
#define ISOM_TIO_ENABLE_OFFSET               ISOM_TIO_TYPE1 + 1
#define ISOM_TIO_TIMEVAL2                    ISOM_TIO_TYPE1 + 4
#define ISOM_TIO_DURATION2                   ISOM_TIO_TIMEVAL2 + 4
#define ISOM_TIO_TYPE2                       ISOM_TIO_DURATION2 + 4
// Free Space - From 0x01AC to 0x01CC
#define RTC_IRT_YELLOW_TIME_OFFSET  		0x01D0

#define	RTC_AR_GROUP_PPM_SHADOW_OFFSET		0x01D4  // 8 Bytes

// PRU clears the event and ARM sets it - 1 bit per PPM
// ARM clear the event and PRU sets it - 1 bit per CPM
#define RTC_CPM_BC_EVENT_OFFSET 			0x01DC

#define COMPENSATION_OFFSET			0x01E4
#define MAXBRIDGE_DELAY_OFFSET		0x01E8

// Free space from 0x1EC to 0x200 is available for future registers

#define EOF_RTC_CONFIG						0x0200

// Event generation
// PRU issues an interrupt to ARM when packet is received or packet is transmitted. This is aligned with buffer complete flag.

// Descriptor for CPM and PPM is as follows: total 16 bytes or 4 32 bit word.

// CPM RTC descriptor for red and green period
// Bit 		Name 			Description
// 0..15 	FrameReference 	16 bit offset into start address pointer for triple buffer CPM (fixed address). Points into cpm triple buffer start addresses (RTC_CPM_BUFFER_ADDRESSES_OFFSET+ offset of current CPM first buffer)
// 16..31 	FrameLength 	11 bits of frame length including VLAN and FCS, set by host only. PRU may verify incoming frame on length which needs to be flexible in terms of stripped VLAN tag. Bit12.15 need to be zero.
// 0..15 	FrameDataPointer 16 bit absolute address pointing to Profinet data of current index buffer. This pointer masks VLAN tag offset. Points to first byte after FID
// 16..23 	FrameIndex	 	Current index of data source which is host on PPM and PRU on CPM.
//							CPM: Before first packet received idx indicated to host with FrameIndex is idx_2 whereas PRU is on idx_0 (not shown in register) . After first packet received by PRU FrameIndex changes to idx_0 and BufferComplete flag is reset by PRU. PRU advances to next index which is idx_1.
//24..31	FrameFlags1		CPM: Bit 0: 1 = VLAN TAG present
//                          CPM: Bit 1: 0 = CPM received on port 1 (PRU0), 1 = CPM received on port 2 (PRU1)
//							Bit 1..7: reserved
// 24..31	Reserved
// 0..15 	FrameID 		FrameID as provided by engineering. Set by host and used only for CPM. Not used for PPM. MRPD?
// 16..23 	FrameFlags2		CPM: Bit 0: 0 = received in red, 1 = received in green
//							Bit 1..7: reserved
// 24..31	Reserved
// 0..15	RR				Reduction Ratio for red period which is max 16.
//							Send in GREEN period: reduction ratio (max 512) set by host and used by PRU to find out whether current phase matches.
// 16..31	Phase			CPM: used for DHT update

// -----------------------------------------------

// PPM RTC descriptor for red and green period
// Bit 		Name 			Description
// 0..15 	FrameReference 	bytes address offset into RTCx - PPM buffer which has fixed base address.
// 16..31 	FrameLength 	11 bits of frame length, set by host only. PRU may verify incoming frame on length.
// 0..31 	FrameSendOffset PPM: Send in RED period: FSO as provided by engineering and limited to 22 bits, i.e. 4.1ms. Set by host only.
// 0..15 	FrameID 		FrameID as provided by engineering. Set by host and used only for CPM. Not used for PPM. MRPD?
// 16..23 	FrameFlags1		PPM: Bit 0: 1 = RTC3 frame in red period, note: first RTC3 will come in green and is handled with additional descriptor for legacy start-up
//							Bit 1..7: reserved
// 24..31	FrameIndex		Current index of data source which is host on PPM and PRU on CPM.
//							CPM: Before first packet received idx indicated to host with FrameIndex is idx_2 whereas PRU is on idx_0 (not shown in register) . After first packet received by PRU FrameIndex changes to idx_0 and BufferComplete flag is reset by PRU. PRU advances to next index which is idx_1.
// 0..15	RR				Reduction Ratio for red period which is max 16.
//							Send in GREEN period: reduction ratio (max 512) set by host and used by PRU to find out whether current phase matches.
// 16..31	Phase			PPM: used for sendlist generation.
#ifdef PRU
.struct struct_rtc_cpm_desc
    .u16	FrameReference
    .u16	FrameLength
	.u16	FrameDataPointer
	.u8		FrameIndex
	.u8		FrameFlags1
	.u16	FrameID
	.u8		FrameFlags2
	.u8		Reserved
    .u16	RR
	.u16	Phase
.ends

.struct struct_rtc_ppm_desc
    .u16	FrameReference
    .u16	FrameLength
	.u32	FrameSendOffset
	.u16	FrameID
	.u8		Reserved
	.u8		FrameIndex
    .u16	RR
	.u16	Phase
.ends

// cpm FrameFlags
#define 	VLAN_TAG		t0
// cpm/ppm FrameFlags
#define		RED_GREEN_FLAG	t0
// FW ARM descriptor structure hard coded in driver
#endif

// CPM/PPM list size is 16 * 8 = 128 bytes
// each list has shadow list -> 192 bytes

#define RTC_DESC_SIZE			16						// increased for IRT
#define RTC_CPM_LIST_SIZE		RTC_DESC_SIZE * NO_CPM
#define RTC_PPM_LIST_SIZE		RTC_DESC_SIZE * NO_PPM

// ICSS PRU0 DMEM 8 kB

// RTC descriptors

// CPM/PPM descriptors have 16 bytes length. There is one descriptor per triple buffer.
// There is a second list for the host to prepare new connection in background.

#define	RTC_CPM_IDX0_OFFSET		0x200
#define RTC_PPM_IDX0_OFFSET		RTC_CPM_IDX0_OFFSET + RTC_DESC_SIZE * NO_CPM
#define RTC_PPM_IDX1_OFFSET		RTC_PPM_IDX0_OFFSET + RTC_DESC_SIZE * NO_PPM
#define EOF_PPM_LIST_OFFSET		RTC_PPM_IDX1_OFFSET + RTC_DESC_SIZE * NO_PPM

// Blocking tables for MRP. Each Port has it's own Blocking static multicast receive and forward table.
#define NO_ROWS			50
#define SIZE_OF_ROW		4
#define BLOCKING_STATIC_MAC_TABLE_RCV		EOF_PPM_LIST_OFFSET
#define BLOCKING_STATIC_MAC_TABLE_FWD		BLOCKING_STATIC_MAC_TABLE_RCV + NO_ROWS * SIZE_OF_ROW


#define DCP_IDENT_REQ_OFFSET 				0x0600

// ICSS PRU1 DMEM 8 kB

// PPM buffer has 1440 bytes IO data which can be shared over 8 connections
// Each connection has a full packet with Ethernet header, FID, DATA, Status.
// Host prepares complete packet which may include VLAN tag. Although RTC
// packet will bypass send queues and has no effect on tags.
// Total size needed is:
// 8 * 26 bytes (header, FID, cycle counter, data status, transfer status)
// 1440 bytes IO data
// triple buffered: 3 * 1440 + 3 * 8 (26) = 4944 bytes = 0x1350
// absolute address is referenced by RTC descriptor FrameIndex
// organization on PPM buffer is managed by host. PRU only executes send list
// and fills in cycles counter and transfer status.

#define PPM_IO_DATA_SIZE		1440		// FW driver uses this value also for CPM data size!
#define PPM_ETH_HEADER			12+4+2+2	// includes VLAN and FID
#define PPM_TRAILER				2+1+1		// cycle counter, data status, transfer status
// FW extended PPM buffer model requires two blocks in different Data RAMs
#define PPM_BUFFER_OFFSET0		0x0D00		// buffer block 0	Data RAM0
#define PPM_BUFFER_OFFSET1		0x10			// buffer block 1	Data RAM1. Inital 16 bytes used for Firmware version

// Following two buffers are used to store a RTC3 frame which has to be forwarded
// PRU0
#define	RTC3_SF_FSO_PRU0_OFFSET		0xBC00
#define RTC3_SF_LENGTH_PRU0_OFFSET	RTC3_SF_FSO_PRU0_OFFSET    + 4
#define RTC3_SF_BUFFER_PRU0_OFFSET	RTC3_SF_LENGTH_PRU0_OFFSET + 4
// PRU1
#define RTC3_SF_FSO_PRU1_OFFSET		0xE814
#define RTC3_SF_LENGTH_PRU1_OFFSET	RTC3_SF_FSO_PRU1_OFFSET    + 4
#define RTC3_SF_BUFFER_PRU1_OFFSET	RTC3_SF_LENGTH_PRU1_OFFSET + 4

// now each block has max of 4 packets
#define EOF_PPM_BUFFER_OFFSET	0x1218		// 3 * (1440+4*26) = 0x1218 (each block...)

// ICSS Shared RAM 12kB

// L3 64KB Memory - switch buffer Pool and CPM buffer

// CPM buffer is at end of L3 Memory. Extend to two CPM of 1440 max...
// 2-8 CPMs including header and CRC 3 * (2*1440 + 8*30) = 9360 = 0x2490
// Tagged frames have tag bit set in descriptor for host to simplify parsing.
// Incoming frames are verified in terms of FID, MAC, period, time, data hold time, status.
// In case there is a match and no error packet is put in the corresponding CPM buffer.
// Triple buffer management is managed through buffer complete flag and FrameIndex in RTC descriptor.
//

#define CPM_IO_DATA_SIZE		1440
#define CPM_ETH_HEADER			12+4+2+2	// includes VLAN and FID
#define CPM_TRAILER				2+1+1+4		// cycle counter, data status, transfer status, CRC

#define EOF_CPM_BUFFER_OFFSET	0xE690		// 0xC200+0x2490
#define CPM_BUFFER_OFFSET 		0xC200
#define EOF_48K_BUFFER_OFFSET	0xC200

//=====================================PTCP RELATED DATA=====================================//
#define	P2_T2_ABS_TS_OFFSET     0x278
#define	P1_T2_ABS_TS_OFFSET     0x274

#define	SYNC_CYCLE_COUNTER	0x0270
#define SYNC_SF_BUF_OFFSET_P2							PTCP_BASE_ADDR_OFFSET + 528		//94 bytes

//these four offsets are only accessed from ARM
#define	PORT2_DELAY_REQ_FRAME_OFFSET					PTCP_BASE_ADDR_OFFSET + 452		//60-bytes
#define	PORT1_DELAY_REQ_FRAME_OFFSET					PTCP_BASE_ADDR_OFFSET + 392		//60-bytes

#define PORT2_DELAY_RESP_FRAME_OFFSET					PTCP_BASE_ADDR_OFFSET + 324		//66-bytes
#define	PORT1_DELAY_RESP_FRAME_OFFSET					PTCP_BASE_ADDR_OFFSET + 256		//66-bytes

#define SYNC_INIT_FLAG_OFFSET							PTCP_BASE_ADDR_OFFSET + 254		// 1 byte required
#define SYNC_MASTER_MAC_OFFSET							PTCP_BASE_ADDR_OFFSET + 248		//6-bytes required
#define SYNC_UUID_OFFSET								PTCP_BASE_ADDR_OFFSET + 232		//16-bytes


#define SYNC_SF_BUF_OFFSET_P1							PTCP_BASE_ADDR_OFFSET + 136		//94 bytes
#define SYNC_SBLOCK_OFFSET								PTCP_BASE_ADDR_OFFSET + 104		//32 bytes	// cut-through
#define SYNC_W_FUP_CTRL_BYTE_OFFSET						PTCP_BASE_ADDR_OFFSET + 100		//1 byte
#define SYNC_CTRL_BYTE_OFFSET							PTCP_BASE_ADDR_OFFSET + 96		//1 byte
#define SYNC_RCV_CYCLE_CTR_OFFSET						PTCP_BASE_ADDR_OFFSET + 92		//2 bytes	// not being used

#define SYNC_TORG_TIME_OFFSET							PTCP_BASE_ADDR_OFFSET + 84		// 8 bytes	// cut-through
#define	SYNC_FUP_DELAY_OFFSET							PTCP_BASE_ADDR_OFFSET + 80
#define P2_DELAY_RESP_CTRL_OFFSET						PTCP_BASE_ADDR_OFFSET + 78
#define P1_DELAY_RESP_CTRL_OFFSET						PTCP_BASE_ADDR_OFFSET + 77
#define	SYNC_FWD_ENABLED_OFFSET							PTCP_BASE_ADDR_OFFSET + 76		// 1 byte required
#define SYNC_RX_SOF_OFFSET_P1							PTCP_BASE_ADDR_OFFSET + 72
#define SYNC_RX_SOF_OFFSET								PTCP_BASE_ADDR_OFFSET + 68		// cut-through
#define SYNC_INDELAY_PLUS_LD_OFFSET						PTCP_BASE_ADDR_OFFSET + 64		// cut-through
#define SYNC_RX_SOF_OFFSET_P2							PTCP_BASE_ADDR_OFFSET + 60
#define SYNC_SEQID_OFFSET								PTCP_BASE_ADDR_OFFSET + 56

#define PRU1_HANDSHAKE_OFFSET							PTCP_BASE_ADDR_OFFSET + 52
#define P2_SMA_LINE_DELAY_OFFSET						PTCP_BASE_ADDR_OFFSET + 48
#define P2_T2_TIME_STAMP_OFFSET							PTCP_BASE_ADDR_OFFSET + 44
#define P2_T4_CYCLE_CTR_OFFSET							PTCP_BASE_ADDR_OFFSET + 40
#define P2_T4_TIME_STAMP_OFFSET							PTCP_BASE_ADDR_OFFSET + 36
#define P2_T1_CYCLE_CTR_OFFSET							PTCP_BASE_ADDR_OFFSET + 32
#define P2_T1_TIME_STAMP_OFFSET							PTCP_BASE_ADDR_OFFSET + 28

#define PRU0_HANDSHAKE_OFFSET							PTCP_BASE_ADDR_OFFSET + 24
#define P1_SMA_LINE_DELAY_OFFSET						PTCP_BASE_ADDR_OFFSET + 20
#define P1_T2_TIME_STAMP_OFFSET							PTCP_BASE_ADDR_OFFSET + 16
#define P1_T4_CYCLE_CTR_OFFSET							PTCP_BASE_ADDR_OFFSET + 12
#define P1_T4_TIME_STAMP_OFFSET							PTCP_BASE_ADDR_OFFSET + 8
#define P1_T1_CYCLE_CTR_OFFSET							PTCP_BASE_ADDR_OFFSET + 4
#define P1_T1_TIME_STAMP_OFFSET							PTCP_BASE_ADDR_OFFSET + 0

#define PTCP_BASE_ADDR_OFFSET							0x0000

//==========================================================================================//

#define ECAP_CLR_CONFIG_OFFSET		PTCP_L3_OCMC_BASE + 272		//4 bytes
#define IEP_CONFIG_ADJ_OFFSET		PTCP_L3_OCMC_BASE + 260		//12 bytes

#define P2_PTCP_CTRL_OFFSET			PTCP_L3_OCMC_BASE + 257
#define P1_PTCP_CTRL_OFFSET			PTCP_L3_OCMC_BASE + 256
#define P2_DLY_FUP_PACKET_OFFSET	PTCP_L3_OCMC_BASE + 192
#define P1_DLY_FUP_PACKET_OFFSET	PTCP_L3_OCMC_BASE + 128
#define P2_DLY_RSP_PACKET_OFFSET	PTCP_L3_OCMC_BASE + 64
#define P1_DLY_RSP_PACKET_OFFSET	PTCP_L3_OCMC_BASE
#define PTCP_L3_OCMC_BASE			0xE700				// moved to accomondate larger CPM buffer

#define PTCP_PHASE_COUNTER_OFFSET		RTC_PHASE_COUNTER_OFFSET
#define PTCP_BASE_CLK_OFFSET			RTC_BASE_CLK_OFFSET
#define PTCP_PM_CYCLE_COUNTER_OFFSET	RTC_CYCLE_COUNTER_OFFSET
#define PTCP_PM_PHASE_COUNTER_OFFSET	RTC_PHASE_COUNTER_OFFSET

#define P2_MAC_ADDR				0x1E50
#define P1_MAC_ADDR				0x1E48

#define STATIC_MAC_TABLE_FWD_PORT2 STATIC_MAC_TABLE_RCV_PORT2  + 256
#define STATIC_MAC_TABLE_RCV_PORT2 STATIC_MAC_TABLE_FWD_PORT1  + 256
#define STATIC_MAC_TABLE_FWD_PORT1 STATIC_MAC_TABLE_RCV_PORT1  + 256
#define STATIC_MAC_TABLE_RCV_PORT1 0x2000

#ifdef __cplusplus
}
#endif

#endif //__ICSS_PN_RTCX__


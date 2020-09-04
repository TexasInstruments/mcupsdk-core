/**
 * \file hsrPrp_firmware.h
 * \brief Macros for HSR/PRP Implementation
 *
 * \par
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
 * \par
 */

#ifndef __HSR_PRP_FIRMWARE_H__
#define __HSR_PRP_FIRMWARE_H__

#ifdef __cplusplus
extern "C"
{
#endif

// #define PRU_DEBUG
//uncomment the the define shared by the host and the PRU
#define COMMON_HOST_PRU_ENABLE

// BUILD defines are set in Predefined Symbols of Project Configuration
// Set required Project Configuration as Active to enable BUILD mode.

// Build PRP
// #define DEBUG_RX_TX
#define CUT_THROUGH_EARLY

#ifdef BUILD_PRP
#define PRP
#define HSR_AND_PRP
#define ICSS_PROTOCOL_RED
#define ICSS_PROTOCOL_PRP
#endif //* BUILD_PRP */

// Build HSR Mode H
#ifdef BUILD_HSR_H
#define BUILD_HSR_COMMON
#define HSR_MODE_H
#endif //* BUILD_HSR_H */

// Build HSR Mode N
#ifdef BUILD_HSR_N
#define BUILD_HSR_COMMON
#define HSR_MODE_N
#endif //* BUILD_HSR_N */

// Build HSR Mode T
#ifdef BUILD_HSR_T
#define BUILD_HSR_COMMON
#define HSR_MODE_T
// uncomment to avoid port duplicate rejection
#define NO_DUPLICATE_PORT_REJECT
// uncomment to avoid host duplicate rejection
#define NO_DUPLICATE_HOST_REJECT
#endif //* BUILD_HSR_T */

// Build HSR Mode M
#ifdef BUILD_HSR_M
#define BUILD_HSR_COMMON
#define HSR_MODE_M
#endif //* BUILD_HSR_M */

// Build HSR Mode U
#ifdef BUILD_HSR_U
#define BUILD_HSR_COMMON
#define HSR_MODE_U
#endif //* BUILD_HSR_U */

#ifdef BUILD_HSR_COMMON
#define HSR
#define HSR_AND_PRP
#define ICSS_PROTOCOL_RED
#define ICSS_PROTOCOL_HSR
#endif //* BUILD_HSR_COMMON */

#ifdef HSR_AND_PRP
// the following define ca be used for debug

// uncomment to enable debug counter for HSR
// #define HSR_DEBUG
// #define DEBUG
//uncomment to enable host's RX path debug output
//#define ICSS_PROTOCOL_HSR_TX_C_DBG
// uncomment to disable removing HSR TAG for frame going to the host
//#define NO_TAG_REMOVAL
// uncomment to not Adapt the line ID
// Uncomment to disable CUT_THROUGH
//#define DONT_ALLOW_CUT_THROUGH
// Uncomment to disable the task: check port table
//#define NO_PORT_DUPLI_TABLE_CHECK_TASK
// Uncomment to disable the task: check host table
//#define NO_HOST_DUPLI_TABLE_CHECK_TASK
// uncomment to disable the node table
//#define NO_NODE_TABLE
// uncomment to disable the  port duplicate rejection
//#define NO_DUPLICATE_PORT_REJECT
// uncomment to disable host duplicate rejection
//#define NO_DUPLICATE_HOST_REJECT

#endif

//uncomment this flag to enable half duplex
//#define HALF_DUPLEX_ENABLED
#ifdef HSR
//#define HSR_MODE                    0x1E74      // w1: END of MAC address (do not modify)
#define HSR_MODE                    0x1E74      // w1: END of MAC address (do not modify)
// w2: modeh (1), moden (2), modet (3), modeu (4), modem (5)
//#define LRE_HSR_MODE                0x1E76      // access trough HWREGH in the host
#define LRE_HSR_MODE                0x1E76      // access trough HWREGH in the host
#define MODEH                       0x01
#define MODEN                       0x02
#define MODET                       0x03
#define MODEU                       0x04
#define MODEM                       0x05
#endif
//****************************************************************************
//*                       ICSS PRU0 DRAM Mem Usage                           *
//* Please update all mem usage here to make it easier for other developers  *
//*       Not updating memory usage leads to difficult to fix errors         *
//****************************************************************************

////**************************************************************************
//                                                                           *
//                                                                           *
//                 Memory map for HSR/PRP.  Numbers indicate bytes           *
//                            AMIC11x, AM335x, AM437x                        *
//                                                                           *
//****************************************************************************

//             DRAM0                               DRAM1                              Shared RAM
// ***********0x2000**************    ************0x2000***************    *************0x3000**************
// **    LINUX RESOURCE TABLE    *    **      LINUX RESOURCE TABLE    *    **          HSR/PRP             *
// ***********0x1FB4**************    ***********0x1FB4****************    **         Node Table           *
// **       Used by Switch       *    **                              *    *                               *
// **                            *    **        Used by Switch        *    *************0x1FC0**************
// ***********0x1F00**************    **                              *    **            Free              *
// **                            *    **                              *    ************0x1E80***************
// **                            *    **                              *    *        Used by Switch         *
// **                            *    **                              *    *************0X400***************
// **     Used by HSR/PRP        *    **                              *    **                              *
// **                            *    ************0X1D00***************    **   HSR/PRP VLAN filtering     *
// **                            *    **                              *    **                              *
// **                            *    **        Used by HSR/PRP       *    **************0x1FE**************
// **                            *    **************0x200**************    **       Used by HSR/PRP        *
// **                            *    **    HSR/PRP MC filtering      *    **************0x140**************
// **                            *    **************0xE0***************    **         Used for PTP         *
// **                            *    **          Reserved            *    **                              *
// ***********0x0000**************    *************0x0000**************    *************0x0000**************

////**************************************************************************
//                                                                           *
//                                                                           *
//                 Memory map for HSR/PRP.  Numbers indicate bytes           *
//                                AM571x, AM572x, K2G                        *
//                                                                           *
//****************************************************************************

//             DRAM0                               DRAM1                              Shared RAM
// ***********0x2000**************    ************0x2000***************    *************0x8000**************
// **    LINUX RESOURCE TABLE    *    **      LINUX RESOURCE TABLE    *    **            Free              *
// ***********0x1FB4**************    ***********0x1FB4****************    *************0x5E00**************
// **       Used by Switch       *    **                              *    *       HSR/PRP Node Table      *
// **                            *    **        Used by Switch        *    *************0x3000**************
// ***********0x1F00**************    **                              *    **            Free              *
// **          Free              *    **                              *    ************0x1E80***************
// ***********0x1C80**************    **                              *    *        Used by Switch         *
// **                            *    **                              *    *************0X400***************
// **                            *    **                              *    **                              *
// **                            *    ************0X1D00***************    **   HSR/PRP VLAN filtering     *
// **                            *    **                              *    **                              *
// **                            *    **        Used by HSR/PRP       *    **************0x1FE**************
// **                            *    **************0x200**************    **       Used by HSR/PRP        *
// **                            *    **    HSR/PRP MC filtering      *    **************0x140**************
// **                            *    **************0xE0***************    **         Used for PTP         *
// **                            *    **          Reserved            *    **                              *
// ***********0x0000**************    *************0x0000**************    *************0x0000**************

//***********************************************************************************************************
//                                                                                                          *
//                                                                                                          *
//                                            End of Section                                                *
//                                                                                                          *
//                                                                                                          *
//***********************************************************************************************************

//-----------------HSR-------------------------------------------------------------

#ifdef COMMON_HOST_PRU_ENABLE
// PRU0 DMEM
//------------------------
// FOR DEBUG
#define DBG_START                         0x1E00
#define DBG_NODE_TABLE_INSERTION_ERROR    DBG_START + 4
#define DBG_RXA_OVERFLOW                  DBG_START + 8   //duplicate found in PRU0 for host duplicate rejection
#define DBG_RXB_OVERFLOW                  DBG_START + 12
#define DBG_RXA_FWD_OVERFLOW              DBG_START + 16   //duplicate found in PRU0 for port duplicate rejection
#define DBG_RXB_FWD_OVERFLOW              DBG_START + 20
#define DBG_RXA_FAILACQU_QUEUE            DBG_START + 24 //count all SFD in PRU0
#define DBG_RXB_FAILACQU_QUEUE            DBG_START + 28 //count all SFD in PRU1
#define DBG_RXA_FWD_FAILACQU_QUEUE        DBG_START + 32
#define DBG_RXB_FWD_FAILACQU_QUEUE        DBG_START + 36
#define DBG_DEBUG_1                       DBG_START + 40  //counter incr when failling to access host queue
#define DBG_DEBUG_2                       DBG_START + 44
#define DBG_DEBUG_3                       DBG_START + 48
#define DBG_DEBUG_4                       DBG_START + 56

// END FOR DEBUG
#define DUPLICATE_HOST_TABLE              0x0200
#define DUPLICATE_HOST_TABLE_END          0x19f4      // last entry of the host table

//PRU1 DMEM

// Multicast filter defines & offsets
//*************************************************************************************
#define M_MULTICAST_TABLE_SEARCH_OP_CONTROL_BIT                 0xE0   // one byte field : 0 -> multicast filtering disabled | 1 -> multicast filtering enabled
#define MULTICAST_FILTER_DISABLED                               0x00
#define MULTICAST_FILTER_ENABLED                                0x01

#define MULTICAST_FILTER_MASK                                   0xE4
#define MULTICAST_FILTER_INIT_VAL                               0xFF
#define ADD_MULTICAST_MAC_ID                                    0x00
#define REMOVE_MULTICAST_MAC_ID                                 0x01

#define MULTICAST_FILTER_TABLE                                  0x100
#define MULTICAST_TABLE_SIZE                                    ( 256 )

#define MULTICAST_FILTER_HOST_RCV_ALLOWED                       0x01
#define MULTICAST_FILTER_HOST_RCV_NOT_ALLOWED                   0x00

#define DEBUG_MC_FLT                                            ( 0 )
//*************************************************************************************

#define DUPLICATE_PORT_TABLE_PRU0         0x0200    // Offset of port duplicate table for PRU0
#define DUPLICATE_PORT_TABLE_PRU0_END     0x0df4    // last entry of the duplicate table for PRU0
#define DUPLICATE_PORT_TABLE_PRU1         0x0E00    // Offset of port duplicate table for PRU1
#define DUPLICATE_PORT_TABLE_PRU1_END     0x19f4    // last entry of the duplicate table for PRU1

#define NODE_TABLE_SIZE                   0x1C00    // Size of the node table [0..128]
#define NODE_TABLE_ARBITRATION            0x1C04    // Busy slave flag and busy master flag for 3 lock used to protect the node table
#define DUPLICATE_HOST_TABLE_SIZE         0x1C08    // Size and setup (N and M) of duplicate host table
#define DUPLICATE_PORT_TABLE_SIZE         0x1C1C    // Size and setup (N and M) of duplicate port table
#define NODE_FORGET_TIME                  0x1C20    // Time after which a node entry is cleared (10ms resolution)
#define DUPLI_FORGET_TIME                 0x1C24    // Time after which an entry is removed from the duplicate table (10ms resolution)
#define PATH_BROKEN_NB_FRAM_DIFF          0x1C28    // Supervision frame Counter minimum difference to detect a broken path
#define DUPLI_PORT_CHECK_RESO             0x1C2C    // Time interval to check the port duplicate table
#define DUPLI_HOST_CHECK_RESO             0x1C30    // Time interval to check the host duplicate table
#define NODETABLE_CHECK_RESO              0x1C34    // Time interval to check the node duplicate table
#define HOST_TIMER_CHECK_FLAGS            0x1C38    /// NodeTable | Host | Port
#define HOST_DUPLICATE_ARBITRATION        0x1C3C    // Arbitration flag for the host duplicate t
#define ICSS_FIRMWARE_RELEASE             0x1C40    // Time counter to trigger the host _dupli_table check task
#define RED_FIRMWARE_RELEASE              0x1C44    // Time counter to trigger the Node_Table check task
#define SUP_ADDR                          0x1C4C    // Supervision address in HSR
#define SUP_ADDR_LOW                      0x1C50

#define DUPLICATE_FORGET_TIME_400_MS        (40)     // Time in TimeTicks (1/100s)
#define DUPLICATE_FORGET_TIME_400_MS_PRP    0x0028  //in Time in TimeTicks (1/100s)  To be used as a constant in the PRU (save loading cycle)(Compile time configuration)
#define NODE_FORGET_TIME_60000_MS           (6000)  // Time in TimeTicks (1/100s)
#define CONST_NODE_FORGET_TIME_60000_MS     0x1770  // Time in TimeTicks (1/100s)
#define MAX_FORGET_TIME_BEFORE_WRAP         0xFFDF  // max value possible for timelastseen before wrap around
//  To be used as a constant in the PRU (save loading cycle) 60 second = 60000 ms (Compile time configuration)

#define DUPLICATE_PORT_TABLE_DMEM_SIZE        (0x0C00)
#define DUPLICATE_HOST_TABLE_DMEM_SIZE        (0x1800)
#define BAD_FRAME_QUEUE_DMEM_SIZE             (0x0080)
#define LRE_STATS_DMEM_SIZE                   (0x0080)
#define DEBUG_COUNTER_DMEM_SIZE               (0x0050)

#define DUPLICATE_HOST_TABLE_SIZE_INIT    (0x00800004)  // N = 128, M = 4
#define DUPLICATE_PORT_TABLE_SIZE_INIT    (0x00400004)  // N = 64, M = 4
#define MASTER_SLAVE_BUSY_BITS_CLEAR      (0x00000000)
#define TABLE_CHECK_RESOLUTION_10_MS      (0x0000000A) // set to
#define TIME_TIC_INC_PRU                  1 // time tick according to resolution Time in TimeTicks (1/100s)
#define SUP_ADDRESS_INIT_OCTETS_HIGH      (0x004E1501)  // 01-15-4E-00-xx-xx
#define SUP_ADDRESS_INIT_OCTETS_LOW       (0x00000001)  // xx-xx-xx-xx-01-00

//SHARED RAM
#define QUEUE_2_PCP_MAP_OFFSET              0x124
#define LRE_Interface_Stats_and_Monitoring  0x140 //Value is always 0 and is used as lreInterfaceStatsIndex. Starts after PTP. See Map
#define LRE_START                           0x140 //Value is always 0 and is used as lreInterfaceStatsIndex. Starts after PTP. See Map
#define LRE_CNT_TX_A                        LRE_START + 4 //Number of frames successfully sent over port A that are HSR/PRP tagged
#define LRE_CNT_TX_B                        LRE_START + 8 //Same for port B
#define LRE_CNT_TX_C                        LRE_START + 12//Number of frames sent successfully towards the application interface of the DANH. Frames with and without PRP/HSR tag are counted
#define LRE_CNT_ERRWRONGLAN_A               LRE_START + 16 //number of frames with the wrong LAN identifier received on LRE port B
#define LRE_CNT_ERRWRONGLAN_B               LRE_START + 20  //Same for port  B
#define LRE_CNT_ERRWRONGLAN_C               LRE_START + 24  //Same for port  C
#define LRE_CNT_RX_A                        LRE_START + 28  //number of frames received successfully with HSR or PRP TAG on a LRE port A
#define LRE_CNT_RX_B                        LRE_START + 32  //Same for port B
#define LRE_CNT_RX_C                        LRE_START + 36  //Same for port C
#define LRE_CNT_ERRORS_A                    LRE_START + 40  //number of frames with errors received on this LRE port A
#define LRE_CNT_ERRORS_B                    LRE_START + 44  //Same for port B
#define LRE_CNT_ERRORS_C                    LRE_START + 48  //Same for port C
#define LRE_CNT_NODES                       LRE_START + 52  //Number of active nodes in the node table
#define LRE_CNT_PROXY_NODES                 LRE_START + 56  //Number of active proxy nodes in the node table
#define LRE_CNT_UNIQUE_RX_A                 LRE_START + 60  //Number of entries in the duplicate detection mechanism on port A for which no duplicate was received.
#define LRE_CNT_UNIQUE_RX_B                 LRE_START + 64  //Same for port B
#define LRE_CNT_UNIQUE_RX_C                 LRE_START + 68  //Same for port C
#define LRE_CNT_DUPLICATE_RX_A              LRE_START + 72  //number of entries in the duplicate detection mechanism on port A for which one single duplicate was received
#define LRE_CNT_DUPLICATE_RX_B              LRE_START + 76  //Same for port B
#define LRE_CNT_DUPLICATE_RX_C              LRE_START + 80  //Same for port C
#define LRE_CNT_MULTIPLE_RX_A               LRE_START + 84  //number of entries in the duplicate detection mechanism on port A for which more than one duplicate was received
#define LRE_CNT_MULTIPLE_RX_B               LRE_START + 88  //Same for port B
#define LRE_CNT_MULTIPLE_RX_C               LRE_START + 92
#define LRE_CNT_OWN_RX_A                    LRE_START + 96
#define LRE_CNT_OWN_RX_B                    LRE_START + 100
#define LRE_DUPLICATE_DISCARD               LRE_START + 104 //Number of frame retreive by the host
#define LRE_TRANSPARENT_RECEPTION           LRE_START + 108//Number of frame retreive by the host
#define LRE_NODE_TABLE_FULL                 LRE_START + 112
#define LRE_MULTICAST_DROPPED               LRE_START + 116
#define LRE_VLAN_DROPPED                    LRE_START + 120
#define LRE_INTR_TMR_EXP                    LRE_START + 124

#ifdef LRE_DEBUG_STATS
#define LRE_TOTAL_RX_A                      LRE_INTR_TMR_EXP + 4
#define LRE_TOTAL_RX_B                      LRE_TOTAL_RX_A + 4
#define LRE_OVERFLOW_PRU0                   LRE_TOTAL_RX_B + 4
#define LRE_OVERFLOW_PRU1                   LRE_OVERFLOW_PRU0 + 4
#define LRE_DD_PRU0                         LRE_OVERFLOW_PRU1 + 4
#define LRE_DD_PRU1                         LRE_DD_PRU0 + 4
#define LRE_CNT_SUP_PRU0                    LRE_DD_PRU1 + 4
#define LRE_CNT_SUP_PRU1                    LRE_CNT_SUP_PRU0 + 4
#endif

// Interrupt pacing offsets
// 0x1FC wasted
//*************************************************************************************
#define INTR_PAC_STATUS_OFFSET                  0x1FAF  // 1 byte | 0 : Interrupt Pacing disabled | 1 : Interrupt Pacing enabled
#define INTR_PAC_DIS_ADP_LGC_DIS                0x0     // Interrupt Pacing disabled, Adaptive logic disabled
#define INTR_PAC_ENA_ADP_LGC_DIS                0x1     // Interrupt Pacing enabled, Adaptive logic disabled
#define INTR_PAC_ENA_ADP_LGC_ENA                0x2     // Interrupt Pacing enabled, Adaptive logic enabled

#define INTR_PAC_PREV_TS_OFFSET_PRU0            0x1FB0  // 4 bytes | previous TS from eCAP TSCNT for PRU 0
#define INTR_PAC_TMR_EXP_OFFSET_PRU0            0x1FB4  // 4 bytes | timer expiration value for PRU 0

#define INTR_PAC_PREV_TS_OFFSET_PRU1            0x1FB8  // 4 bytes | timer expiration value for PRU 1
#define INTR_PAC_TMR_EXP_OFFSET_PRU1            0x1FBC  // 4 bytes | previous TS from eCAP TSCNT for PRU 1
#define INTR_PAC_PREV_TS_RESET_VAL              0x0

//*************************************************************************************
/* Enable/disable interrupts for high/low priority instead of per port.
 * 0 = disabled (default) 1 = enabled
 */
#define PRIORITY_INTRS_STATUS_OFFSET    0x1FAA
/* Enable/disable timestamping of packets. 0 = disabled (default) 1 = enabled */
#define TIMESTAMP_PKTS_STATUS_OFFSET    0x1FAB
#define TIMESTAMP_ARRAY_OFFSET          0xC200
#define TIMESTAMP_ENABLE          0x1
#define TIMESTAMP_DISABLE          0x0


// VLAN filter defines & offsets
//*************************************************************************************
#define VLAN_FLTR_CTRL_BYTE                     0x1FE   // VLAN filter control byte address
#define VLAN_FLTR_CTRL_BYTE_DFLT_VAL            ( ( VLAN_FLTR_DIS << VLAN_FLTR_CTRL_SHIFT ) | ( VLAN_FLTR_UNTAG_HOST_RCV_ALL << VLAN_FLTR_UNTAG_HOST_RCV_CTRL_SHIFT ) | (VLAN_FLTR_PRIOTAG_HOST_RCV_ALL << VLAN_FLTR_PRIOTAG_HOST_RCV_CTRL_SHIFT ) | ( VLAN_FLTR_SV_VLAN_FLOW_SKIP << VLAN_FLTR_SV_CTRL_SHIFT ) )

#define VLAN_FLTR_CTRL_SHIFT                    0x0     // one bit field | 0 : VLAN filter disabled | 1 : VLAN filter enabled
#define VLAN_FLTR_UNTAG_HOST_RCV_CTRL_SHIFT     0x1     // one bit field | 0 : untagged host rcv allowed | 1 : untagged host rcv not allowed
#define VLAN_FLTR_PRIOTAG_HOST_RCV_CTRL_SHIFT   0x2     // one bit field | 0 : priotag host rcv allowed | 1 : priotag host rcv not allowed
#define VLAN_FLTR_SV_CTRL_SHIFT                 0x3     // one bit field | 0 : SV frames skip VLAN flow | 1 : SV frames enter VLAN flow

#define VLAN_FLTR_DIS                           0x0     // VLAN filter disabled
#define VLAN_FLTR_ENA                           0x1     // VLAN filter enabled

#define VLAN_FLTR_UNTAG_HOST_RCV_ALL            0x0     // untagged host rcv allowed
#define VLAN_FLTR_UNTAG_HOST_RCV_NAL            0x1     // untagged host rcv not allowed

#define VLAN_FLTR_PRIOTAG_HOST_RCV_ALL          0x0     // priotag host rcv allowed
#define VLAN_FLTR_PRIOTAG_HOST_RCV_NAL          0x1     // priotag host rcv not allowed

#define VLAN_FLTR_PRIOTAG_VID                   0x0     // VID = 0 for priority tagged frames

#define VLAN_FLTR_SV_VLAN_FLOW_SKIP             0x0     // SV frames skip VLAN flow
#define VLAN_FLTR_SV_VLAN_FLOW_TAKE             0x1     // SV frames enter VLAN flow

#define VLAN_FLTR_TBL_BASE_ADDR                 0x200   // VLAN filter table base address
#define VLAN_FLTR_TBL_SIZE                      0x200   // 4096 bits = 512 bytes = 0x200 bytes

#define ADD_VLAN_VID                            0x00    // Add VLAN ID
#define REM_VLAN_VID                            0x01    // Remove VLAN ID

#define VLAN_VID_MIN                            0x0000  // VID minimum possible value
#define VLAN_VID_MAX                            0x0FFF  // VID maximum possible value

#define DEBUG_VLAN_FLT                          0
//*************************************************************************************

#define IEC62439_CONST_DUPLICATE_ACCEPT                 0x01
#define IEC62439_CONST_DUPLICATE_DISCARD                0x02
#define IEC62439_CONST_TRANSPARENT_RECEPTION_REMOVE_RCT 0x01
#define IEC62439_CONST_TRANSPARENT_RECEPTION_PASS_RCT   0x02

// OFFSET and CONSTANT USE FOR NODE TABLE
#define NT_ENTRY_STATE_OFFSET     6
#define NT_ENTRY_VALID            0x01
#define NT_ENTRY_TO_BE_ERASED     0x10
#define ERASE_ENTRY_FLAG_OFFSET   4 // flag for erasing a entry if it was created with an erroneous frame
#define VALID_ENTRY_FLAG_OFFSET   0 // flag for erasing a entry if it was created with an erroneous frame
#define NT_STATUSFIELD_OFFSET     7
#define NT_CNTA_OFFSET            8 //frame received on port A
#define NT_CNTB_OFFSET            12//frame received on port B
#define NT_ERRWLA_OFFSET          16//erroneous frame received on port A
#define NT_ERRWLB_OFFSET          20//erroneous frame received on port B
#define NT_CNTRXSUPA_OFFSET       24//Number of supervision frame received on port A
#define NT_CNTRXSUPB_OFFSET       25//Number of supervision frame received on port B
#define NT_TLSS_OFFSET            26//Time last seen of a supervision frame
#define NT_TLSA_OFFSET            28//Time last seen of a frame received on port A
#define NT_TLSB_OFFSET            30//Time last seen of a frame received on port B

// OFFSET and CONSTANT USE FOR NODE TABLE
#define DT_ENTRY_DMEM_SIZE        12 // Duplicate table entry size
#define DT_BIN_SIZE               4 // Bin size of the duplicate table (host and port are identical)
#define DT_ENTRY_NOT_VALID        0
#define DT_ENTRY_VALID            0x01
#define PORT_DUPLI_HASH_MASK      0x3F // Mask for the port duplicate hash index value (used against host duplicate hash index)
#define ENTRY_IS_UNIQUE           1 // Value to determine a duplicate entry as Unique
#define ENTRY_IS_DUPLI            2 // Value to determine a duplicate entry as Unique
#define ENTRY_IS_MULTI            3 // Value to determine a duplicate entry as Unique
#define DT_LIFESPAN_OFFSET        8//OFFSET of the lifeSpan value of a duplicate entry in the memory
// Define regarding SUPERVISION FRAME
#define PRP_SV_TLV_TYPE_DUP_DISCARD   0x14
#define PRP_SV_TLV_TYPE_DUP_ACCEPT    0x15
#define PRP_SV_TLV_TYPE_RBX           0x1E
#define PRP_SV_LENGTH_MAC             0x06
#ifdef HSR
#define NB_BYTES_RCV_BF_RBX           40 // Number of bytes received before RedBoxMacAddress of a supervision frame is received
#define NB_BYTES_RCV_BF_RBX_VLAN      44 // Number of bytes received before RedBoxMacAddress of a supervision frame is received with VLAN
#else //PRP
#define NB_BYTES_RCV_BF_RBX           34 // Number of bytes received before RedBoxMacAddress of a supervision frame is received
#define NB_BYTES_RCV_BF_RBX_VLAN      38 // Number of bytes received before RedBoxMacAddress of a supervision frame is received with VLAN
#endif
// Constant value for locking machanism for Nodetable
#define MASTER_BIT_SHIFT              2 // The MASTER_BIT is shifted of 2 bytes in the locking register
#define MASTER_STATUS_SIZE            2 // 2 bytes use for master status (master bit t0, supervision bit t8)
#define SLAVE_STATUS_SIZE             2 // 2 bytes use for master status (master bit t0, supervision bit t8)
//Other constant values
#define RX_FIFO_FILL_SKIP_NT_PROCESS  1    //number of bytes in the Rx Fifo to be late in scheduling
#define RX_FIFO_FILL_LATE_SCHEDULING  20    //number of bytes in the Rx Fifo to be late in scheduling
#define NT_ENTRY_INVALID              0x00
#define VLAN_ID_CROSS                 0x0081 // VLAND ID WIRH BYTE SWAP
#define PRP_ID_CROSS                  0xFB88  // PRP ID WIRH BYTE SWAP
#define SUP_ADDR_ID_12                0x1501  // CONST OF THE supervision frame
#define SUP_ADDR_ID_34                0x004E// CONST OF THE supervision frame
#define SUP_ADDR_ID_56                0x0001 // CONST OF THE supervision frame (LAS BYTE is XX)
#define RES_ADDR_ID_12                0x8001 //See IEEE 802.1D Table 7-10 Reserved addresses
#define RES_ADDR_ID_34                0x00C2 //01-80-C2-00-00-XX Link local Address
#define RES_ADDR_ID_56                0x0000
#define PRP_SUPPATH_VERSION           0x0100 //Sup Path and Version of a supervision frames
#define NT_REM_NODE_TYPE_MASK         0x1F
#define NT_REM_NODE_TYPE_SHIFT        0x00

#define NT_REM_NODE_TYPE_SANA         0x01 //Node entry type of : SANA
#define NT_REM_NODE_TYPE_SANB         0x02 //Node entry type of : SANB
#define NT_REM_NODE_TYPE_SANAB        0x03 //Node entry type of : SANAB
#define NT_REM_NODE_TYPE_DAN          0x04 //Node entry type of : DAN
#define NT_REM_NODE_TYPE_REDBOX       0x08 //Node entry type of : REDBOX
#define NT_REM_NODE_TYPE_VDAN         0x10 //Node entry type of : VDAN

#define NT_REM_NODE_HSR_BIT           0x20 // if set node is HSR

#define NT_REM_NODE_DUP_MASK          0xC0
#define NT_REM_NODE_DUP_SHIFT         0x06

#define NT_REM_NODE_DUP_ACCEPT        0x40 // Node entry duplicate type: Duplicate Accept
#define NT_REM_NODE_DUP_DISCARD       0x80 // Node entry duplicate type: Duplicate Discard
// This are pre-defined  type with duplicate type with HSR or PRP
#define NT_REM_NODE_TYPE_DAN_DUP_ACCEPT   0x44
#define NT_REM_NODE_TYPE_DAN_DUP_DISCARD  0x84
#define NT_REM_NODE_TYPE_RDX_DUP_ACCEPT   0x48
#define NT_REM_NODE_TYPE_RDX_DUP_DISCARD  0x88
#define NT_REM_NODE_TYPE_VDAN_DUP_ACCEPT  0x50
#define NT_REM_NODE_TYPE_VDAN_DUP_DISCARD 0x90

#define NT_REM_NODE_TYPE_DAN_HSR          0x24
#define NT_REM_NODE_TYPE_VDAN_HSR         0x30
#define NT_REM_NODE_TYPE_RBX_HSR          0x28
#define TVL_1_HSR                         0x17

// HOST_TIMER_CHECK_FLAGS bits
#define HOST_TIMER_NODE_TABLE_CHECK_BIT   (0x00000001)
#define HOST_TIMER_NODE_TABLE_CLEAR_BIT   (0x00000010)
#define HOST_TIMER_HOST_TABLE_CHECK_BIT   (0x00000100)
#define HOST_TIMER_PORT_TABLE_CHECK_BIT   (0x01010000)


/* DEBUG macro */
#define DEBUG_HSR                         (0)

// defining new nodetable values
//*************************************************************************************
#define INDEX_ARRAY_NT                          ( 0x3000 )
#define INDEX_TABLE_MAX_ENTRIES                 ( 256 )
#define INDEX_ARRAY_SIZE                        ( INDEX_TABLE_MAX_ENTRIES * 6 ) // 5 bytes per entry | 1 byte padding
#define INDEX_ARRAY_LOCATION                    ICSS_SHARED_CONST
#define HASHVAL_MASK                            ( 0xFF)

#define BIN_ARRAY                               ( INDEX_ARRAY_NT + INDEX_ARRAY_SIZE )
#define BIN_ARRAY_MAX_ENTRIES                   ( 256 )
#define BIN_ARRAY_SIZE                          ( BIN_ARRAY_MAX_ENTRIES * 8 )  // 6 bytes MAC ID + 2 bytes nodetable_offset
#define BIN_ARRAY_LOCATION                      ICSS_SHARED_CONST

#define NODE_TABLE_NT                           ( BIN_ARRAY + BIN_ARRAY_SIZE )
#define NODE_TABLE_NT_MAX_ENTRIES               ( 256 )
#define NODE_TABLE_LAST_ENTRY_OFFSET            ( NODE_TABLE_NT_MAX_ENTRIES - 1 )
#define NODE_TABLE_NT_SIZE                      ( NODE_TABLE_NT_MAX_ENTRIES * 32 )

#define NEXT_FREE_SLOT                          ( NODE_TABLE_NT + NODE_TABLE_NT_SIZE ) // 4 bytes
#define BIN_ARRAY_LOCK                          ( NEXT_FREE_SLOT + 4 )  // 1 byte
#define BIN_ARRAY_LOCK_FIRMWARE_PRU0            ( NEXT_FREE_SLOT + 6 )  // 1 byte
#define BIN_ARRAY_LOCK_FIRMWARE_PRU1            ( NEXT_FREE_SLOT + 7 )  // 1 byte

#define CONST_NODETABLE_INVALID                 ( 0xFFFF )

//*************************************************************************************

#endif // HSR_AND_PRP //


#ifdef __cplusplus
}
#endif

#endif // __HSR_PRP_FIRMWARE_H__



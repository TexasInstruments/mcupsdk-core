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

// file:   icss_switch_dlr.h
//*
// brief:  Definitions and mapping of Ethernet IP over PRU
//         Includes:
//         1. DLR structures
//         2. DLR variables

#ifndef ICSS_DLR_SWITCH_H_
#define ICSS_DLR_SWITCH_H_

#ifdef __cplusplus
extern "C"
{
#endif


//This file contains Ethernet/IP specific definitions
// Target switches: PRU / ARM
// Protocol switches: EIP

// Port assignment:
// PORT 0: ARM or host port
// PORT 1: PHY Port 1 - PRU0 receive and PRU1 transmit
// PORT 2: PHY Port 2 - PRU1 receive and PRU0 transmit
// DLR MAC ID's

#define ETHERNET_IP

#define DLR_BEACON_MAC_ID       0x01216C000001
#define DLR_ANNOUNCE_MAC_ID     0x01216C000003
#define DLR_ADVRT_MAC_ID        0x01216C000004
#define DLR_LEARNUPDT_MAC_ID    0x01216C000005
#define DLR_LINK_STAT_MAC_ID    0x01216C000006

#define DLR_MAC_IDENTIFIER      0x006C2101

#define DLR_BEACON_FRAME_TYPE       0x1
#define DLR_NCREQ_FRAME_TYPE        0x2
#define DLR_NCRES_FRAME_TYPE        0x3
#define DLR_LINKSTAT_FRAME_TYPE     0x4
#define DLR_LOCFAULT_FRAME_TYPE     0x5
#define DLR_ANNOUNCE_FRAME_TYPE     0x6
#define DLR_SIGNON_FRAME_TYPE       0x7
#define DLR_ADVERTISE_FRAME_TYPE    0x8
#define DLR_FLUSHTABLE_FRAME_TYPE   0x9
#define DLR_LRNUPDT_FRAME_TYPE      0xA

#define DLR_PORT0   0x1
#define DLR_PORT1   0x2

#define DLR_BEACON_MAC_TYPE     0x0100
#define DLR_SIGNON_MAC_TYPE     0x0200

#define DLR_ENABLE      0x1
#define DLR_DISABLE     0x0

#define DLR_TRUE        0x1
#define DLR_FALSE       0x0

#define DLR_PORT0_FAIL  0x0
#define DLR_PORT1_FAIL  0x1

//node starts in idle mode*/
#define DLR_INIT_STATE_MACHINE      0x10000

//Administrative flag offsets*/


//Reserved byte offsets*/
#define DLR_BEACON_RESERVED_BYTES_OFFSET        40
#define DLR_ADVRT_RESERVED_BYTES_OFFSET         41
#define DLR_NCREQ_RESERVED_BYTES_OFFSET         30
#define DLR_LOCFAULT_RESERVED_BYTES_OFFSET      30
#define DLR_LEARNUPDT_RESERVED_BYTES_OFFSET     26
#define DLR_ANNOUNCE_RESERVED_BYTES_OFFSET      31
#define DLR_NCRES_RESERVED_BYTES_OFFSET         31
#define DLR_LSTAT_RESERVED_BYTES_OFFSET         31
#define DLR_FLUSHTABLE_RESERVED_BYTES_OFFSET    31
#define DLR_SIGNON_RESERVED_BYTES_OFFSET        42
#define DLR_LEARN_UPDATE_IP_OFFSET              18
#define DLR_LEARN_UPDATE_SEQ_ID_OFFSET          22
#define DLR_LEARN_UPDATE_SRC_PORT_OFFSET        17

//Reserved byte sizes*/
#define DLR_BEACON_RESERVED_BYTES_SIZE          20
#define DLR_ADVRT_RESERVED_BYTES_SIZE           19
#define DLR_NCREQ_RESERVED_BYTES_SIZE           30
#define DLR_LOCFAULT_RESERVED_BYTES_SIZE        30
#define DLR_LEARNUPDT_RESERVED_BYTES_SIZE       34
#define DLR_ANNOUNCE_RESERVED_BYTES_SIZE        29
#define DLR_NCRES_RESERVED_BYTES_SIZE           29
#define DLR_LSTAT_RESERVED_BYTES_SIZE           29
#define DLR_FLUSHTABLE_RESERVED_BYTES_SIZE      29
#define DLR_SIGNON_RESERVED_BYTES_SIZE          18

//Offsets for syntax elements -- from start of packet*/
#define DLR_DST_MAC_OFFSET                      0
#define DLR_SRC_MAC_OFFSET                      6
#define DLR_START_OF_VLAN_ID                    14
#define DLR_FRAME_TYPE_OFFSET                   20
#define DLR_SOURCE_PORT_OFFSET                  21
#define DLR_SOURCE_IP_OFFSET                    22
#define DLR_REQ_SRC_PORT_OFFSET                 30
#define DLR_SEQUENCE_ID_OFFSET                  26
#define DLR_RING_STATE_OFFSET                   30
#define DLR_SUP_PRECEDENCE_OFFSET               31
#define DLR_LINK_STATUS_OFFSET                  30
#define DLR_GATEWAY_STATE_OFFSET                30
#define DLR_GATEWAY_PRED_OFFSET                 31
#define DLR_ADV_TIMEOUT_OFFSET                  36
#define DLR_ADV_INTERVAL_OFFSET                 32
#define DLR_LEARN_UPDATE_ENABLE_OFFSET          30
#define DLR_LEARN_UPDT_ETHRTYPE_OFFSET          12
#define DLR_LEARN_UPDT_RING_SUBTYPE_OFFSET      14
#define DLR_LEARN_PROT_VER_OFFSET               15
#define DLR_SIGNON_NUM_NODES_OFFSET             30
#define DLR_SIGNON_SUP_MAC_ADD_OFFSET           32
#define DLR_SIGNON_SUP_IP_ADD_OFFSET            38

#define NUM_DLR_PKTS            0x00
#define NUM_NCREQ_PKTS          0x04
#define NUM_NCRES_PKTS          0x08
#define NUM_LINKSTAT_PKTS       0x0C
#define NUM_LOCFAULT_PKTS       0x10
#define NUM_ANNOUNCE_PKTS       0x14
#define NUM_ADVERT_PKTS         0x18
#define NUM_FLUSH_PKTS          0x1C
#define NUM_LRNUPDT_PKTS        0x20
#define NUM_SIGNON_PKTS         0x24

//DLR stats common to both PRU's, stored in shared RAM*/
//******Different kind of ring faults****/

#define NUM_COMPLETE_FAULT      0x00
#define NUM_PARTIAL_FAULT       0x04
#define NUM_RAPID_SWITCH_FAULT  0x08

//***********LED Task reads from this and displays lights accordingly
//***********This is modified by both ARM and Firmware**************/


//when host needs to process dlr, this flag is set
//this is to distinguish a dlr event from a regular host
#define DLR_EVENT_OCCURED   0x1b94*/


//**************Event flag masks*****************/

//******************************************************************************************
//Shared PRU Offsets
//******************************************************************************************
//This address exists on shared RAM, contains the state machine as well

//*******************************************************************************************

//******************************************************************************************
//DRAM 0 and 1 offsets
//******************************************************************************************
//this same offset is used on DRAM 0 as well as DRAM 1 for port 0 and port 1 respectively
//*************************************************************************************************/

//*************Common events******************/
#define DLR_RING_NORMAL_TRANSITION_MASK     0x1
#define DLR_RING_FAULT_TRANSITION_MASK      0x2

#define DLR_STOP_BOTH_TIMERS_MASK           0x4
#define DLR_PORT0_BEACON_RCVD_MASK          0x8
#define DLR_PORT1_BEACON_RCVD_MASK          0x10
#define DLR_RESET_EVENT_OCCURED_MASK        0x20
//*********************************************************


//********************************************************************
//*************port specific events***************/
#define DLR_LOCFAULT_RCVD_MASK              0x1
#define DLR_NCREQ_RCVD_MASK                 0x2

//neighbor check response received*/
#define DLR_NCRES_RCVD_MASK                 0x4
#define DLR_START_TIMER_MASK                0x8

//this is used to communicate to the firmware if particular timer is halted*/
#define DLR_RING_FAULT_RCVD_MASK            0x10

//drop the packet, does not get forwarded*/
#define DLR_DROP_PACKET_MASK                0x20

#define DLR_UPDATE_SUP_CFG_MASK             0x40

#define DLR_HOST_FWD_MASK                   0x80

#define DLR_TIMER_RUNNING_MASK              0x100

#define DLR_FLUSH_TABLE_RCVD_MASK           0x1000
#define DLR_SEND_LEARNING_UPDATE_MASK       0x2000


//****************************************************************

#define DLR_NODE_STATE_FLAG                 0xff
#define LAST_PORT_BEACON_RCVD_FLAG          0xff00
#define DLR_RING_NORMAL_MASK                0xffff
#define DLR_RING_FAULT_MASK                 0x1

///***********************************************************************************************************************************

//****************************************************************************
//                      DLR Memory Usage for Shared RAM                    *
//****************************************************************************/

//In shared RAM*/
#define DLR_SRAM_SIZE                       48
#define DLR_BEACON_TIMEOUT_OFFSET           0x148
#define DLR_SCALED_BEACON_TIMEOUT           DLR_BEACON_TIMEOUT_OFFSET + 36
#define DLR_LOOP_DETECTED_OFFSET            DLR_BEACON_TIMEOUT_OFFSET + 32
#define DLR_ACTIVE_SUP_IP_OFFSET            DLR_BEACON_TIMEOUT_OFFSET + 28    //4 //4 bytes
#define DLR_ENABLED_OFFSET                  DLR_BEACON_TIMEOUT_OFFSET + 24
#define DLR_SIGNON_PKT_RCVD_OFFSET          DLR_BEACON_TIMEOUT_OFFSET + 20
#define DLR_ACTIVE_SUP_MAC_45               DLR_BEACON_TIMEOUT_OFFSET + 16
#define DLR_ACTIVE_SUP_MAC_0123             DLR_BEACON_TIMEOUT_OFFSET + 12

//vlan id + supervisor id
#define DLR_VLAN_ID_SUPID_OFFSET            DLR_BEACON_TIMEOUT_OFFSET + 8
#define DLR_BEACON_INTERVAL_OFFSET          DLR_BEACON_TIMEOUT_OFFSET + 4

//Since they are accessed often, they sit at the bottom in shared RAM
//#define DLR_COMMON_EVENTS_OFFSET          DLR_STATE_MACHINE_OFFSET + 3
#define   DLR_STATE_MACHINE_OFFSET          0x0

//****************************************************************************
//*                       DLR Memory Usage                                   *
//*              These are present on both PRU0 and PRU1 DRAM                *
//****************************************************************************

#define DLR_TIMEOUT_ISR_ACK_OFFSET          DLR_NCREQ_NUM_RETRIES_OFFSET + 1
#define DLR_NCREQ_NUM_RETRIES_OFFSET        DLR_PORT_EVENTS_OFFSET + 4  //1 byte
#define DLR_PORT_EVENTS_OFFSET              0x10 //Identical for both DRAM0 and DRAM1

//*******************************************************************************************
//State values
//******************************************************************************************/
#define DLR_IDLE_STATE_VAL              0x0
#define DLR_FAULT_STATE_VAL             0x1
#define DLR_NORMAL_STATE_VAL            0x2

#define DLR_RING_NORMAL_STATE_VAL       0x1
#define DLR_RING_FAULT_STATE_VAL        0x2

//vlan is added to this and sent, so while extracting it the hardware needs to subtract this from the*/
//value read to get the real vlan id*/
#define DLR_VLAN_ID_CONST       0xE000

#ifdef __cplusplus
}
#endif

#endif /* ICSS_DLR_SWITCH_H_ */


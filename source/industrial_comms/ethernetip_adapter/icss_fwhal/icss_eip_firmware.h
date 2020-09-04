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

#ifndef ICSS_EIP_FIRMWARE_H_
#define ICSS_EIP_FIRMWARE_H_

#ifdef __cplusplus
extern "C"
{
#endif


////**************************************************************************
//                                                                           *
//                                                                           *
//                 Memory map for EIP    .  Numbers indicate bytes           *
//                                                                           *
//                                                                           *
//****************************************************************************

//             DRAM0                               DRAM1                              Shared RAM
// ***********0x2000**************    ************0x2000***************    *************0x3000**************
// **                            *    **                              *    **                              *
// **                            *    **     Used for statistics      *    **    Available for Protocol    *
// **    Used for statistics     *    **         and misc             *    **           Usage              *
// **       and misc             *    **                              *    **                              *
// ***********0x1F00**************    ***********0x1F00****************    *************0X1E80**************
// **  Multicast filter offsets  *    **                              *    **       Used by Switch         *
// ***********0x1DF4**************    **                              *    **    BD offsets for 3 queues   *
// **                            *    **        Switch Context        *    *************0X400***************
// **                            *    **                              *    **      OVERFLOW_CNT_PRU1       *
// **  Available for Protocol    *    **                              *    **      OVERFLOW_CNT_PRU0       *
// **         Usage              *    ************0X1D00***************    *************0X3F8***************
// **                            *    **                              *    **    Available for Protocol    *
// **                            *    **   Available for Protocol     *    **            Usage             *
// **                            *    **           Usage              *    **************0x178**************
// **                            *    **                              *    **         DLR offsets          *
// ***********0x0014**************    *************0x0014**************    **************0x148**************
// **     TX_EOF_TS_OFFSET       *    **       TX_EOF_TS_OFFSET       *    **         Used for PTP         *
// ***********0x0010**************    *************0x0010**************    *************0x0008**************
// **       firmware info        *    **       firmware info          *    **    DLR_STATE_MACHINE_OFFSET  *
// ***********0x0000**************    *************0x0000**************    *************0x0000**************

#define ETHER_ADDR_LEN 6

#define OVERFLOW_CNT_PRU0                   0x3F8   // 4 bytes
#define OVERFLOW_CNT_PRU1                   OVERFLOW_CNT_PRU0 + 4   // 4 bytes

// Multicast filter defines & offsets
// Total memory computation
//      256 bytes MC table
//      6   bytes mask
//      1   byte control variable
//      4   bytes counter
//      1 byte padding
//-----------------------------
//      268 bytes
//-----------------------------
//*************************************************************************************
#define MC_FLTR_MSK                         0x1DF4
#define MC_FLTR_MSK_SIZE                    6
#define MC_FLTR_INIT_VAL                    0xFF
#define ADD_MC_MAC_ID                       0x00
#define REM_MC_MAC_ID                       0x01

#define MC_OP_CTRL_BYT                      ( MC_FLTR_MSK + MC_FLTR_MSK_SIZE )   // one byte field
#define MC_OP_CTRL_BYT_SIZE                 1
#define MC_FLTR_CTRL_BIT                    0    // bit 0 | multicast filtering : 0 -> disabled | 1 -> enabled
#define MC_FLTR_DIS                         0x0
#define MC_FLTR_ENA                         0x1
#define MC_SP_CTRL_BIT                      1    // bit 1 | storm prevention : 0 -> enabled | 1 -> disabled
#define MC_SP_ENA                           0x0
#define MC_SP_DIS                           0x1
#define MC_OP_CTRL_BYT_DFT                  ( (MC_FLTR_DIS << MC_FLTR_CTRL_BIT) | (MC_SP_ENA << MC_SP_CTRL_BIT) ) // default | multicast filtering disabled , storm prevention enabled

#define MC_FLTR_BYT_PADDING                 1

#define LRE_MC_DROPPED                      ( MC_OP_CTRL_BYT + MC_OP_CTRL_BYT_SIZE + MC_FLTR_BYT_PADDING )
#define LRE_MC_DROPPED_SIZE                 4

#define MC_FLTR_TABLE                       ( LRE_MC_DROPPED + LRE_MC_DROPPED_SIZE )
#define MC_TABLE_SIZE                       256
#define MC_FLTR_HOST_RCV_ALL                0x01
#define MC_FLTR_HOST_RCV_NAL                0x00

#define DEBUG_MC_FLT                        1
#define FEATURE_MULTICAST_FILTER_SHIFT      26
//*************************************************************************************


#ifdef __cplusplus
}
#endif

#endif /* ICSS_EIP_FIRMWARE_H_ */

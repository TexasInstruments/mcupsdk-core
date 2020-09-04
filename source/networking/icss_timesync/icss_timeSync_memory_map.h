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

#ifndef _ICSS_TIMESYNC_MM_H_
#define _ICSS_TIMESYNC_MM_H_

#ifdef __cplusplus
extern "C"
{
#endif


#define GPTP_BASE_ADDR_OFFSET                            0x8

#define RX_SYNC_TIMESTAMP_OFFSET_P1                      ( GPTP_BASE_ADDR_OFFSET + 0 )                  //12 bytes
#define RX_TIMESTAMP_OFFSET_P1                           ( GPTP_BASE_ADDR_OFFSET + 0 )                  //12 bytes
#define RX_PDELAY_REQ_TIMESTAMP_OFFSET_P1                ( RX_SYNC_TIMESTAMP_OFFSET_P1 + 12 )           //12 bytes
#define RX_PDELAY_RESP_TIMESTAMP_OFFSET_P1               ( RX_PDELAY_REQ_TIMESTAMP_OFFSET_P1 + 12 )     //12 bytes
#define RX_SYNC_TIMESTAMP_OFFSET_P2                      ( RX_PDELAY_RESP_TIMESTAMP_OFFSET_P1 + 12 )    //12 bytes
#define RX_TIMESTAMP_OFFSET_P2                           ( RX_PDELAY_RESP_TIMESTAMP_OFFSET_P1 + 12 )                  //12 bytes
#define RX_PDELAY_REQ_TIMESTAMP_OFFSET_P2                ( RX_SYNC_TIMESTAMP_OFFSET_P2 + 12 )           //12 bytes
#define RX_PDELAY_RESP_TIMESTAMP_OFFSET_P2               ( RX_PDELAY_REQ_TIMESTAMP_OFFSET_P2 + 12 )     //12 bytes
#define TIMESYNC_DOMAIN_NUMBER_LIST                      ( RX_PDELAY_RESP_TIMESTAMP_OFFSET_P2 + 12 )    //2 domains(2 bytes) supported in firmware
#define P1_SMA_LINE_DELAY_OFFSET                         ( TIMESYNC_DOMAIN_NUMBER_LIST + 2 )            //4 bytes
#define P2_SMA_LINE_DELAY_OFFSET                         ( P1_SMA_LINE_DELAY_OFFSET + 4 )               //4 bytes
#define TIMESYNC_SECONDS_COUNT_OFFSET                    ( P2_SMA_LINE_DELAY_OFFSET + 4 )               //6 bytes
#define TIMESYNC_TC_RCF_OFFSET                           ( TIMESYNC_SECONDS_COUNT_OFFSET + 6 )          //4 bytes
#define DUT_IS_MASTER_OFFSET                             ( TIMESYNC_TC_RCF_OFFSET + 4 )                 //1 byte. Tells if port is master or slave
#define MASTER_PORT_NUM_OFFSET                           ( DUT_IS_MASTER_OFFSET + 1 )                   //1 byte
#define SYNC_MASTER_MAC_OFFSET                           ( MASTER_PORT_NUM_OFFSET + 1 )                 //6 bytes

#define TX_TS_NOTIFICATION_OFFSET_SYNC_P1                ( SYNC_MASTER_MAC_OFFSET + 6 )                 //1 byte
#define TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P1            ( TX_TS_NOTIFICATION_OFFSET_SYNC_P1 + 1 )      //1 byte
#define TX_TS_NOTIFICATION_OFFSET_PDEL_RES_P1            ( TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P1 + 1 )  //1 byte
#define TX_TS_NOTIFICATION_OFFSET_SYNC_P2                ( TX_TS_NOTIFICATION_OFFSET_PDEL_RES_P1 + 1 )  //1 byte
#define TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P2            ( TX_TS_NOTIFICATION_OFFSET_SYNC_P2 + 1 )      //1 byte
#define TX_TS_NOTIFICATION_OFFSET_PDEL_RES_P2            ( TX_TS_NOTIFICATION_OFFSET_PDEL_REQ_P2 + 1 )  //1 byte
#define TX_SYNC_TIMESTAMP_OFFSET_P1                      ( TX_TS_NOTIFICATION_OFFSET_PDEL_RES_P2 + 1 )  // 12 bytes
#define TX_PDELAY_REQ_TIMESTAMP_OFFSET_P1                ( TX_SYNC_TIMESTAMP_OFFSET_P1 + 12 )           // 12 bytes
#define TX_PDELAY_RESP_TIMESTAMP_OFFSET_P1               ( TX_PDELAY_REQ_TIMESTAMP_OFFSET_P1 + 12 )     // 12 bytes
#define TX_SYNC_TIMESTAMP_OFFSET_P2                      ( TX_PDELAY_RESP_TIMESTAMP_OFFSET_P1 + 12 )    // 12 bytes
#define TX_PDELAY_REQ_TIMESTAMP_OFFSET_P2                ( TX_SYNC_TIMESTAMP_OFFSET_P2 + 12 )           // 12 bytes
#define TX_PDELAY_RESP_TIMESTAMP_OFFSET_P2               ( TX_PDELAY_REQ_TIMESTAMP_OFFSET_P2 + 12 )     // 12 bytes
#define TIMESYNC_CTRL_VAR_OFFSET                         ( TX_PDELAY_RESP_TIMESTAMP_OFFSET_P2 + 12 )    //1 byte
#define DISABLE_SWITCH_SYNC_RELAY_OFFSET                 ( TIMESYNC_CTRL_VAR_OFFSET + 1 )               //1 byte
#define MII_RX_CORRECTION_OFFSET                         ( DISABLE_SWITCH_SYNC_RELAY_OFFSET + 1 )       //2 bytes
#define MII_TX_CORRECTION_OFFSET                         ( MII_RX_CORRECTION_OFFSET + 2 )               //2 bytes
#define TIMESYNC_IEP_VAL_CYCLE_COUNTER                   ( MII_TX_CORRECTION_OFFSET + 2 )               //4 bytes
#define TIMESYNC_SYNC0_CMP_VALUE                         ( TIMESYNC_IEP_VAL_CYCLE_COUNTER + 4 )         //4 bytes
#define TIMESYNC_CMP1_CMP_OFFSET                         ( MII_TX_CORRECTION_OFFSET + 2 )               //8 bytes
#define TIMESYNC_SYNC0_CMP_OFFSET                        ( TIMESYNC_CMP1_CMP_OFFSET + 8 )               //8 bytes
#define TIMESYNC_CMP1_PERIOD_OFFSET                      ( TIMESYNC_SYNC0_CMP_OFFSET + 8 )              //4 bytes
#define TIMESYNC_SYNC0_WIDTH_OFFSET                      ( TIMESYNC_CMP1_PERIOD_OFFSET + 4 )            //4 bytes
#define SINGLE_STEP_IEP_OFFSET_P1                        ( TIMESYNC_SYNC0_WIDTH_OFFSET + 4 )            //8 bytes
#define SINGLE_STEP_SECONDS_OFFSET_P1                    ( SINGLE_STEP_IEP_OFFSET_P1 + 8 )              //8 bytes
#define SINGLE_STEP_IEP_OFFSET_P2                        ( SINGLE_STEP_SECONDS_OFFSET_P1 + 8 )          //8 bytes
#define SINGLE_STEP_SECONDS_OFFSET_P2                    ( SINGLE_STEP_IEP_OFFSET_P2 + 8 )              //8 bytes
#define LINK_LOCAL_FRAME_HAS_HSR_TAG                     ( SINGLE_STEP_SECONDS_OFFSET_P2 + 8 )          //1 byte
#define PTP_PREV_TX_TIMESTAMP_P1                         ( LINK_LOCAL_FRAME_HAS_HSR_TAG + 1 )           //8 bytes
#define PTP_PREV_TX_TIMESTAMP_P2                         ( PTP_PREV_TX_TIMESTAMP_P1 + 8 )               //8 bytes
#define PTP_CLK_IDENTITY_OFFSET                          ( PTP_PREV_TX_TIMESTAMP_P2 + 8 )               //8 byte
#define PTP_SCRATCH_MEM                                  ( PTP_CLK_IDENTITY_OFFSET + 8 )                //8 byte


#ifdef __cplusplus
}
#endif

#endif /* _ICSS_TIMESYNC_MM_H_ */

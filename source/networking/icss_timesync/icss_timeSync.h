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

#ifndef ICSS_TIMESYNC_H_
#define ICSS_TIMESYNC_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdlib.h>
#include <math.h>
#include <networking/icss_emac/icss_emac.h>
#include "icss_timeSyncApi.h"

/* ========================================================================== */
/*                                Error Codes                                 */
/* ========================================================================== */
/**
 \brief Success
 */
#define TIME_SYNC_OK                                                         (1)
/**
 \brief Time Sync module is unable to create a Task
 */
#define TIME_SYNC_UNABLE_TO_CREATE_TASK                                     (-1)
/**
 \brief Time Sync module is unable to create a Semaphore
 */
#define TIME_SYNC_UNABLE_TO_CREATE_SEMAPHORE                                (-2)
/**
 \brief Time Sync module is unable to create an interrupt
 */
#define TIME_SYNC_UNABLE_TO_CREATE_INTERRUPT                                (-3)
/**
 \brief Time Sync module is unable to allocate memory
 */
#define TIME_SYNC_UNABLE_TO_ALLOC_MEM                                       (-4)
/**
 \brief Time Sync module is unable to create timer
 */
#define TIME_SYNC_UNABLE_TO_CREATE_CLOCK                                    (-5)
/**
 \brief Time Sync module is unable to create a mailbox
 */
#define TIME_SYNC_UNABLE_TO_CREATE_MAILBOX                                  (-6)
/**
 \brief Time Sync module is unable to create an event
 */
#define TIME_SYNC_UNABLE_TO_CREATE_EVENT                                    (-7)
/**
 \brief Time Sync unsupported format
 */
#define TIME_SYNC_UNSUPPORTED_FORMAT                                        (-8)
/**
 \brief Time Sync not able to initialize EDMA
 */
#define TIME_SYNC_EDMA_INIT_FAILED                                          (-9)
/**
 \brief Time Sync feature disabled
 */
#define TIME_SYNC_FEATURE_NOT_ENABLED                                       (-10)
/**
 \brief Time Sync handle uninitialized
 */
#define TIME_SYNC_HANDLE_NOT_INITIALIZED                                    (-11)


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**
 * \def DEFAULT_PDELAY_REQ_LOG_INTERVAL
 *      Default PDelay Request packet interval. Actual value is 2^value seconds
 */
#define DEFAULT_PDELAY_REQ_LOG_INTERVAL         3
/**
 * \def DEFAULT_SYNC_SEND_LOG_INTERVAL
 *      Default Sync transmit interval. Actual value is 2^value seconds
 */
#define DEFAULT_SYNC_SEND_LOG_INTERVAL          0
/**
 * \def DEFAULT_ANNOUNCE_SEND_LOG_INTERVAL
 *      Default Announce packet transmit interval. Actual value is 2^value seconds
 */
#define DEFAULT_ANNOUNCE_SEND_LOG_INTERVAL      1
/**
 * \def DEFAULT_ANNOUNCE_TIMEOUT_LOG_INTERVAL
 *      Default Announce packet timeout. Actual value is 2^value seconds
 */
#define DEFAULT_ANNOUNCE_TIMEOUT_LOG_INTERVAL   3

/**
 * \def TIMESYNC_SYNC_ISR_MASK_P1
 *      Interrupt mask for PRU event 22
 */
#define TIMESYNC_SYNC_ISR_MASK_P1               0x400000

/**
 * \def TIMESYNC_SYNC_ISR_MASK_P2
 *      Interrupt mask for PRU event 25
 */
#define TIMESYNC_SYNC_ISR_MASK_P2               0x2000000

/*
 * \def TIMESYNC_TX_TS_ISR_MASK_P1
 *      Interrupt mask for PRU event 23
 */
#define TIMESYNC_TX_TS_ISR_MASK_P1              0x800000
/**
 * \def TIMESYNC_TX_TS_ISR_MASK_P2
 *      Interrupt mask for PRU event 24
 */
#define TIMESYNC_TX_TS_ISR_MASK_P2              0x1000000

/**
 * \def PTP_SYNC_MSG_ID
 *      Sync message ID value
 */
#define PTP_SYNC_MSG_ID                     0x00
/**
 * \def PTP_DLY_REQ_MSG_ID
 *      Delay request message ID value
 */
#define PTP_DLY_REQ_MSG_ID                  0x01
/**
 * \def PTP_PDLY_REQ_MSG_ID
 *      PDelay request message ID value
 */
#define PTP_PDLY_REQ_MSG_ID                 0x02
/**
 * \def PTP_PDLY_RSP_MSG_ID
 *      PDelay response message ID value
 */
#define PTP_PDLY_RSP_MSG_ID                 0x03
/**
 * \def PTP_FOLLOW_UP_MSG_ID
 *      Follow up message ID value
 */
#define PTP_FOLLOW_UP_MSG_ID                0x08
/**
 * \def PTP_DLY_RESP_MSG_ID
 *      Delay response message ID value
 */
#define PTP_DLY_RESP_MSG_ID                 0x09
/**
 * \def PTP_PDLY_RESP_FLW_UP_MSG_ID
 *      PDelay response follow up message ID value
 */
#define PTP_PDLY_RESP_FLW_UP_MSG_ID         0x0A
/**
 * \def PTP_ANNOUNCE_MSG_ID
 *      Announce message ID value
 */
#define PTP_ANNOUNCE_MSG_ID                 0x0B
/**
 * \def PTP_MGMT_MSG_ID
 *      Management message ID value
 */
#define PTP_MGMT_MSG_ID                     0x0D

/**
 * \def PTP_FLW_UP_CTRL_MSG_ID
 *      Control message ID value for Follow Up
 */
#define PTP_FLW_UP_CTRL_MSG_ID              0x02

/**
 * \def SEC_TO_NS
 * Value of seconds in nanoseconds. Useful for calculations
 */
#define SEC_TO_NS                           1000000000

/**
 * \def FILTER_ALPHA_COEFF
 *      Alpha coefficient for sync interval exponential filter and other filters
 */
#define FILTER_ALPHA_COEFF                          0.85

/**
 * \def PTP_SYNC0_PERIOD_DIVIDER
 *      CMP1 period divided by this number makes the width of the Sync pulse
 */
#define PTP_SYNC0_PERIOD_DIVIDER        4

/**
 * \def OFFSET_THRESHOLD_FOR_RESET
 *      If offset from Master goes above this threshold it will
 *      trigger a reset. Value is in nanoseconds
 */
#define OFFSET_THRESHOLD_FOR_RESET     10000

/**
 * \def STABLE_FILTER_THRESHOLD
 *      When clock drift goes below this value it indicates drift has
 *      stabilized and SMA filter can kick in
 */
#define STABLE_FILTER_THRESHOLD         100

/**
 * \def TIMESYNC_PEER_DELAY_ERROR_THRESHOLD
 *      When peer delay exceeds this value it gets reset to 0
 *      Sometimes wrong values are calculated when doing peer delay
 *      calculation when clock on peer changes in between peer delay
 *      messages and because of the filter this value stays for a long time
 *      This helps in fixing it. This is set to a value much higher than
 *      max peer delay (of a 100 mtr cable)
 */
#define TIMESYNC_PEER_DELAY_ERROR_THRESHOLD     10000

/**
 * \def NUM_SYNC_MISSED_THRESHOLD
 *      If this many consecutive sync frames are missed
 *      the DUT gets reset
 */
#define NUM_SYNC_MISSED_THRESHOLD       3

/**
 * \def AVG_NUM_DELAY_MEASUREMENTS
 *      Number of values in the running average
 */
#define AVG_NUM_DELAY_MEASUREMENTS      3

/*Hex Offsets for different fields in PTP Delay Req/Res Packet. Annex E*/
/*To get Annex F offsets subtract ANNEX_D_ANNEX_F_DIFF from it*/

/**
 * \def SRC_MAC_OFFSET
 *      Source MAC address offset
 */
#define SRC_MAC_OFFSET              6
/**
 * \def IP_DSCP_OFFSET
 *      Differentiated Services Code Point (DSCP) offset in IP4/UDP
 */
#define IP_DSCP_OFFSET              15
/**
 * \def SRC_IP_OFFSET
 *      Source IP address offset in IP4/UDP
 */
#define SRC_IP_OFFSET               26
/**
 * \def DST_IP_OFFSET
 *      Destination IP address offset in IP4/UDP
 */
#define DST_IP_OFFSET               30
/**
 * \def IP_CHKSUM_OFFSET
 *      IP4 checksum offset in IP4/UDP
 */
#define IP_CHKSUM_OFFSET            40
/**
 * \def PTP_MSG_ID_OFFSET
 *      PTP message ID offset
 */
#define PTP_MSG_ID_OFFSET           42
/**
 * \def PTP_DOMAIN_NUM_OFFSET
 *      PTP domain number offset
 */
#define PTP_DOMAIN_NUM_OFFSET       46
/**
 *
 * Offset for PTP flags (including 2 step)
 */
#define PTP_FLAG_OFFSET             48
/**
 *  \def PTP_CORRECTION_OFFSET
 *      Offset for PTP correction field
 */
#define PTP_CORRECTION_OFFSET       50
/**
 * \def PTP_SRC_CLK_IDENTITY
 *      Offset for PTP source clock identity (MAC + protocol id)
 */
#define PTP_SRC_CLK_IDENTITY        62
/**
 * \def PTP_SRC_PORT_ID_OFFSET
 *      Offset source port id
 */
#define PTP_SRC_PORT_ID_OFFSET      70
/**
 * \def PTP_SEQ_ID_OFFSET
 *      Offset for sequence identifier (incremented every frame)
 */
#define PTP_SEQ_ID_OFFSET           72
/**
 * \def PTP_CONTROL_MSG_ID_OFFSET
 *      Offset for message id. This categorizes frames into two groups
 *      time critical and non time critical
 */
#define PTP_CONTROL_MSG_ID_OFFSET   74
/**
 * \def PTP_LOG_MSG_PERIOD
 *      Log message interval offset
 */
#define PTP_LOG_MSG_PERIOD          75
/**
 * \def PTP_REQ_RCPT_TS_SEC_OFFSET
 *      Offset for seconds timestamp
 */
#define PTP_REQ_RCPT_TS_SEC_OFFSET  76
/**
 * \def PTP_REQ_RCPT_TS_NSEC_OFFSET
 *      Offset for nanoseconds timestamp
 */
#define PTP_REQ_RCPT_TS_NSEC_OFFSET 82
/**
 * \def PTP_REQ_SRC_PORT_IDENTITY
 *      Offset for requestor clock identity
 */
#define PTP_REQ_SRC_PORT_IDENTITY   86
/**
 * \def PTP_REQ_SRC_PORT_ID
 *      Offset for requestor port identity
 */
#define PTP_REQ_SRC_PORT_ID         94

/**
 * \def PTP_UTC_OFFSET
 *      Offset for UTC offset in Announce frame
 */
#define PTP_UTC_OFFSET              86

/**
 * \def PTP_PRIORITY1_OFFSET
 *      Offset for Priority 1 field in Announce frame
 */
#define PTP_PRIORITY1_OFFSET        89

/**
 * \def PTP_GM_CLK_CLASS_OFFSET
 *      Offset for GM clock class field in Announce frame
 */
#define PTP_GM_CLK_CLASS_OFFSET     90

/**
 * \def PTP_GM_CLK_ACCU_OFFSET
 *      Offset for GM clock accuracy field in Announce frame
 */
#define PTP_GM_CLK_ACCU_OFFSET          91

/**
 * \def PTP_GM_CLK_VARIANCE_OFFSET
 *      Offset for GM clock variance field in Announce frame
 */
#define PTP_GM_CLK_VARIANCE_OFFSET      92

/**
 * \def PTP_PRIORITY2_OFFSET
 *      Offset for priority 2 field in Announce frame
 */
#define PTP_PRIORITY2_OFFSET            94

/**
 * \def PTP_GM_CLK_IDENTITY_OFFSET
 *      Offset for GM clock identity field in Announce frame
 */
#define PTP_GM_CLK_IDENTITY_OFFSET      95

/**
 * \def PTP_STEPS_REMOVED_OFFSET
 *      Offset for steps removed field in Announce frame
 */
#define PTP_STEPS_REMOVED_OFFSET        103

/**
 * \def PTP_TIME_SRC_OFFSET
 *      Offset for GM time source field in Announce frame
 */
#define PTP_TIME_SRC_OFFSET             105

/**
 * \def PTP_E2E_BUFFER_OFFSET
 *      Offset for start of PTP data in UDP/IP4 frame
 */
#define PTP_E2E_BUFFER_OFFSET           42
/**
 * \def PTP_P2P_BUFFER_OFFSET
 *      Offset for start of PTP data in 802.3 frame
 */
#define PTP_P2P_BUFFER_OFFSET           14

/**
 * \def ANNEX_D_ANNEX_F_DIFF
 *      Number of Extra bytes from Annex F(802.1) to Annex E(UDP)
 */
#define ANNEX_D_ANNEX_F_DIFF            28
/**
 * \def HSR_CORRECTION
 *      Number of bytes in HSR Header
 */
#define HSR_CORRECTION                  6

/**
 * \def SRC_DST_MAC_SIZE
 *      Num bytes in source plus destination MAC
 */
#define SRC_DST_MAC_SIZE                12

/**
 * \def PTP_LEAP_61_MASK
 *      Mask for Leap 61 status in PTP Flags
 */
#define  PTP_LEAP_61_MASK               (1<<0)
/**
 * \def PTP_LEAP_59_MASK
 *      Mask for Leap 59 status in PTP Flags
 */
#define  PTP_LEAP_59_MASK               (1<<1)
/**
 * \def PTP_UTC_REASONABLE_MASK
 *      Mask for UTC reasonable value in PTP Flags
 */
#define  PTP_UTC_REASONABLE_MASK        (1<<2)
/**
 * \def PTP_TIMESCALE_MASK
 *      Mask for value indicating whether PTP Timescale
 *      in PTP Flags
 */
#define  PTP_TIMESCALE_MASK             (1<<3)
/**
 * \def PTP_TIME_TRACEABLE_MASK
 *      Mask for value indicating whether
 *      time traceable in PTP Flags
 */
#define  PTP_TIME_TRACEABLE_MASK        (1<<4)
/**
 * \def PTP_FREQ_TRACEABLE_MASK
 *      Mask for value indicating whether
 *      freq traceable in PTP Flags
 */
#define  PTP_FREQ_TRACEABLE_MASK        (1<<5)
/**
 * \def PTP_ALTERNATE_MASTER_MASK
 *      Mask for value indicating whether
 *      alternate master in PTP Flags
 */
#define  PTP_ALTERNATE_MASTER_MASK      (1<<8)

 /**
  * \def PTP_TWO_STEP_MASK
  *      Mask for value indicating whether
  *      two step sync in PTP Flags
  */
#define  PTP_TWO_STEP_MASK              (1<<9)
/**
 * \def PTP_UNICAST_MASK
 *      Mask for value indicating whether
 *      messaging is unicast in PTP Flags
*/
#define  PTP_UNICAST_MASK               (1<<10)
/**
 * \def PTP_PROFILE_SPECIFIC_1_MASK
 *      Mask for value indicating alternate
 *      PTP Profile in PTP Flags
 *
 */
#define  PTP_PROFILE_SPECIFIC_1_MASK    (1<<13)
/**
 * \def PTP_PROFILE_SPECIFIC_2_MASK
 *      Mask for value indicating alternate
 *      PTP Profile in PTP Flags
 *
 */
#define  PTP_PROFILE_SPECIFIC_2_MASK    (1<<14)
/**
 * \def PTP_SECURITY_MASK
 *  Mask for value indicating whether PTP security
 *  is enabled or not in PTP Flags
 */
#define  PTP_SECURITY_MASK              (1<<15)

/**
 * \def SYNC_INTERVAL_PERIOD_CHANGE_THRESHOLD
 *      If current sync interval varies from previously measured sync
 *      interval by this much percentage then it triggers a PTP reset
 */
#define SYNC_INTERVAL_PERIOD_CHANGE_THRESHOLD   60

/**
 * \def GPTP_NUM_DOMAINS
 *      Number of domains supported by GPTP implementation
 */
#define GPTP_NUM_DOMAINS                        2

/**
 *
 * Size for PTP flags (including 2 step)
 */
#define PTP_FLAG_SIZE               (2)

/**
 * \def PTP_SRC_CLK_IDENTITY_SIZE
 *      Size for PTP source clock identity (MAC + protocol id)
 */
#define PTP_SRC_CLK_IDENTITY_SIZE   (8)
/**
 * \def PTP_SRC_PORT_ID_SIZE
 *      Size for source port id field
 */
#define PTP_SRC_PORT_ID_SIZE        (2)

/**
 * \def PTP_GM_CLK_IDENTITY_SIZE
 *      Size for GM clock identity field in Announce frame
 */
#define PTP_GM_CLK_IDENTITY_SIZE    (8)

/**
 * \def PTP_GM_CLK_CLASS_SIZE
 *      Size for GM clock class field in Announce frame
 */
#define PTP_GM_CLK_CLASS_SIZE       (1)

/**
 * \def PTP_GM_CLK_ACCU_SIZE
 *      Size for GM clock accuracy field in Announce frame
 */
#define PTP_GM_CLK_ACCU_SIZE        (1)

/**
 * \def PTP_GM_CLK_VARIANCE_SIZE
 *      Size for GM clock variance field in Announce frame
 */
#define PTP_GM_CLK_VARIANCE_SIZE    (2)

/**
 * \def PTP_UTC_SIZE
 *      Size for UTC offset in Announce frame
 */
#define PTP_UTC_SIZE                (2)

/**
 * \def PTP_TIME_SRC_SIZE
 *      Size for GM time source field in Announce frame
 */
#define PTP_TIME_SRC_SIZE           (1)

/**
 * \def PTP_PRIORITY1_SIZE
 *      Size for Priority 1 field in Announce frame
 */
#define PTP_PRIORITY1_SIZE          (1)

/**
 * \def PTP_PRIORITY2_SIZE
 *      Size for Priority 2 field in Announce frame
 */
#define PTP_PRIORITY2_SIZE          (1)

/**
 * \def PTP_STEPS_REMOVED_SIZE
 *      Offset for steps removed field in Announce frame
 */
#define PTP_STEPS_REMOVED_SIZE      (2)

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/**
 * \brief Resets CMP1 once first adjustment is done, else 1PPS signal doesn't trigger
 * \param timeSyncHandle pointer to PTP Handle structure
 */
void TimeSync_resetIEP(TimeSync_ParamsHandle_t timeSyncHandle);
/**
 * \brief Do initial adjustment for the IEP based on sync timestamp
 * \param timeSyncHandle pointer to PTP Handle structure
 * \param portNum ICSS_EMAC_PORT_1 or ICSS_EMAC_PORT_2
 */
void TimeSync_doFirstAdjustment(TimeSync_ParamsHandle_t timeSyncHandle,
                                uint8_t portNum);

/**
 *  \addtogroup NETWORKING_ICSS_TIMESYNC_MODULE
 *  @{
 */

/**
 * \brief Callback for link status change on Port 1
 * \param linkStatus 1/0 whether up or down
 * \param arg2 timeSyncHandle pointer to PTP Handle structure
 */
void TimeSync_Port1linkResetCallBack(uint8_t linkStatus, void *arg2);

/**
 * \brief Callback for link status change on Port 2
 * \param linkStatus 1/0 whether up or down
 * \param arg2 timeSyncHandle pointer to PTP Handle structure
 */
void TimeSync_Port2linkResetCallBack(uint8_t linkStatus, void *arg2);

/**
 *  \brief  Reset the state machine in firmware to restart synchronization (First adjustment happens again)
 *
 *          If Sync interval changes or link break happens or there is a large adjustment in time
 *          this function is called.
 *
 *  \param  timeSyncHandle pointer to PTP Handle structure
 *
 *
 */
void TimeSync_reset(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \brief Processes a PTP message and extract fields
 * \param timeSyncHandle pointer to PTP Handle structure
 * \param pktBuffer pointer to packet data
 * \param portNum port number on which the packet was received
 * \param size size of the packet
 * \param isLinkLocal
 *
 */
void TimeSync_processPTPFrame(TimeSync_ParamsHandle_t timeSyncHandle,
                              uint8_t *pktBuffer, \
                              uint8_t portNum, uint16_t size, uint8_t isLinkLocal);

/**
 * \brief Return True/False 1/0 if timeSync module is enabled
 * \param timeSyncHandle pointer to PTP Handle structure
 */
uint8_t TimeSync_isEnabled(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \brief Write the MAC ID of PTP master to Firmware. BMC Algorithm should call this API.
 * \param timeSyncHandle pointer to PTP Handle structure
 * \param parentMac Pointer to MAC ID of master.
 */
void TimeSync_updateParentAddress(TimeSync_ParamsHandle_t timeSyncHandle,
                                  uint8_t *parentMac);
/**
 * \brief Retuns the previous master's MAC ID.
 * \param timeSyncHandle pointer to PTP Handle structure
 * \param prevMac Pointer to MAC ID of previous master.
 */
void TimeSync_getPrevAddress(TimeSync_ParamsHandle_t timeSyncHandle,
                             uint8_t *prevMac);
/**
 * \brief Copies the PTP announce message data to PTP stack buffer.
 * \param timeSyncHandle pointer to PTP Handle structure
 * \param buff Pointer to buffer of PTP Stack.
 */
void TimeSync_getGeneralMessage(TimeSync_ParamsHandle_t timeSyncHandle,
                                int8_t *buff);
/**
 * \brief Performs PHY delay correction on Rx timestamp
 * Assumption : The function expects the timestamp in rxTimestamp_gPTP
 * field on which it does correction
 * \param timeSyncHandle pointer to PTP Handle structure
 */
void TimeSync_rxPhyDelayCorrection(TimeSync_ParamsHandle_t timeSyncHandle);

/** @} */

/**
 * \internal
 * \brief Get the timestamp of transmitted frame and process if required
 * This is called by both PTP_TxTSTask_P1 and PTP_TxTSTask_P2 tasks with port number as input
 * \param  timeSyncHandle pointer to PTP Handle structure
 * \param  portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 * \param frameType Whether Sync, Delay req or Delay response
 */
void TimeSync_getTxTS(TimeSync_ParamsHandle_t timeSyncHandle, uint8_t portNum,
                      ptpFrameTypes_t frameType);

/**
 * \internal
 * \brief Calculate Line Delay given the parameters from Delay Response
 * This is called from inside the interrupt context
 * \param  timeSyncHandle pointer to PTP Handle structure
 */
void TimeSync_lineDelayCalc(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \internal
 * \brief Calculate Peer Delay given the parameters from Peer Delay Req/Res
 * \param timeSyncHandle pointer to PTP Handle structure
 * \param twoStep Indicates if calculation is through 2 step mechanism
 * \param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 */
void TimeSync_peerDelayCalc(TimeSync_ParamsHandle_t timeSyncHandle,
                            uint8_t twoStep, uint8_t portNum);
/**
 * \internal
 * \brief Process Sync/Follow Up frames and do synchronization
 * \param timeSyncHandle pointer to PTP Handle structure
 * \param buff pointer to the sync and follow up frame
 * \param followUp Set to 1 if it's a follow up frame else 0
 * \param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 * \param size size of the frame
 */
void TimeSync_processSyncFrame(TimeSync_ParamsHandle_t timeSyncHandle,
                               uint8_t *buff, \
                               uint8_t followUp, uint8_t portNum, uint16_t size);
/**
 * \internal
 * \brief Process Pdelay response frame
 * \param timeSyncHandle pointer to PTP Handle structure
 * \param buff pointer to the Pdelay Response Frame
 * \param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 */
void TimeSync_processDelayResFrame(TimeSync_ParamsHandle_t timeSyncHandle,
                                   uint8_t *buff, \
                                   uint8_t portNum);
/**
 * \internal
 * \brief Process Pdelay request frame
 * \param timeSyncHandle pointer to PTP Handle structure
 * \param buff pointer to the Pdelay Request Frame
 * \param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 */
void TimeSync_processPdelayReqFrame(TimeSync_ParamsHandle_t timeSyncHandle,
                                    uint8_t *buff, \
                                    uint8_t portNum);
/**
 * \internal
 * \brief Process Pdelay response and Pdelay response followUp frame
 * \param  timeSyncHandle pointer to PTP Handle structure
 * \param buff pointer to the Pdelay Response/Response Follow Up Frame
 * \param followUp 0 if it's a response frame, 1 if response follow up
 * \param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 */
void TimeSync_processPdelayRespFrame(TimeSync_ParamsHandle_t timeSyncHandle,
                                     uint8_t *buff, \
                                     uint8_t followUp, uint8_t portNum);
/**
 * \internal
 * \brief Called for forced 2-step mode. Sends out follow up frame with bridge delay
 * \param  timeSyncHandle pointer to PTP Handle structure
 * \param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 */
void TimeSync_forced2StepBDCalc(TimeSync_ParamsHandle_t timeSyncHandle,
                                uint8_t portNum);

/**
 * \internal
 * \brief Synchronize clock upon receipt of Sync Frame. Applicable for OC
 * This is called from inside the interrupt context
 * \param  timeSyncHandle pointer to PTP Handle structure
 */
void TimeSync_synchronizeClock(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \internal
 * \brief Calculate Neighbor rate ratio for adjacent nodes
 * \param  timeSyncHandle pointer to PTP Handle structure
 * \param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 */
void TimeSync_calcNRR(TimeSync_ParamsHandle_t timeSyncHandle, uint8_t portNum);
/**
 * \internal
 * \brief Updates NRR info which is subsequently used to calculate
 * the final NRR value
 * \param  timeSyncHandle pointer to PTP Handle structure
 * \param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 */
void TimeSync_updateNRRParams(TimeSync_ParamsHandle_t timeSyncHandle,
                              uint8_t portNum);

/**
 * \internal
 * \brief Calculate rcf for syntonization from line delay parameters
 * \param  timeSyncHandle pointer to PTP Handle structure
 */
void TimeSync_calcRcfAndSyncInterval(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \internal
 * \brief For single step master, computation of Tx seconds and nanoseconds timestamp requires
 * some techniques to avoid a long division in firmware. Logic is inside the definition
 * \param  timeSyncHandle pointer to PTP Handle structure
 */
void TimeSync_writeTS_SingleStep_Sync(TimeSync_ParamsHandle_t timeSyncHandle,
                                      uint8_t portNum);

void TimeSync_dummyBMCA(TimeSync_ParamsHandle_t timeSyncHandle,
                        uint8_t *pktBuffer);


uint64_t getIEPTimestamp(TimeSync_ParamsHandle_t timeSyncHandle);


#ifdef __cplusplus
}
#endif

#endif /* ICSS_TIMESYNC_H_ */

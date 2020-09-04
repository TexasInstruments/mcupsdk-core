/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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

/*!
 * \file timeSync_ptp_priv.h
 *
 * \brief This file contains the private APIs to PTP stack.
 *
 */

#ifndef TIMESYNC_PTP_PRIV_H_
#define TIMESYNC_PTP_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <include/core/enet_osal.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/EventP.h>

#include "timeSync.h"
#include "timeSync_ptp.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * @def TIMESYNC_PTP_DEFAULT_PDELAY_REQ_LOG_INTERVAL
 *      Default PDelay Request packet interval. Actual value is 2^value seconds
 */
#define TIMESYNC_PTP_DEFAULT_PDELAY_REQ_LOG_INTERVAL         (3U)

/**
 * @def TIMESYNC_PTP_DEFAULT_SYNC_SEND_LOG_INTERVAL
 *      Default Sync transmit interval. Actual value is 2^value seconds
 */
#define TIMESYNC_PTP_DEFAULT_SYNC_SEND_LOG_INTERVAL          (0U)

/**
 * @def TIMESYNC_PTP_DEFAULT_ANNOUNCE_SEND_LOG_INTERVAL
 *      Default Announce packet transmit interval. Actual value is 2^value seconds
 */
#define TIMESYNC_PTP_DEFAULT_ANNOUNCE_SEND_LOG_INTERVAL      (1U)

/**
 * @def TIMESYNC_PTP_DEFAULT_ANNOUNCE_TIMEOUT_LOG_INTERVAL
 *      Default Announce packet timeout. Actual value is 2^value seconds
 */
#define TIMESYNC_PTP_DEFAULT_ANNOUNCE_TIMEOUT_LOG_INTERVAL   (3U)

/**
 * @def TIMESYNC_PTP_SYNC_MSG_ID
 *      Sync message ID value
 */
#define TIMESYNC_PTP_SYNC_MSG_ID                             (0x00U)

/**
 * @def TIMESYNC_PTP_DLY_REQ_MSG_ID
 *      Delay request message ID value
 */
#define TIMESYNC_PTP_DLY_REQ_MSG_ID                          (0x01U)

/**
 * @def TIMESYNC_PTP_PDLY_REQ_MSG_ID
 *      PDelay request message ID value
 */
#define TIMESYNC_PTP_PDLY_REQ_MSG_ID                         (0x02U)

/**
 * @def TIMESYNC_PTP_PDLY_RSP_MSG_ID
 *      PDelay response message ID value
 */
#define TIMESYNC_PTP_PDLY_RSP_MSG_ID                         (0x03U)

/**
 * @def TIMESYNC_PTP_FOLLOW_UP_MSG_ID
 *      Follow up message ID value
 */
#define TIMESYNC_PTP_FOLLOW_UP_MSG_ID                        (0x08U)

/**
 * @def TIMESYNC_PTP_DLY_RESP_MSG_ID
 *      Delay response message ID value
 */
#define TIMESYNC_PTP_DLY_RESP_MSG_ID                         (0x09U)

/**
 * @def TIMESYNC_PTP_PDLY_RESP_FLW_UP_MSG_ID
 *      PDelay response follow up message ID value
 */
#define TIMESYNC_PTP_PDLY_RESP_FLW_UP_MSG_ID                 (0x0AU)

/**
 * @def TIMESYNC_PTP_ANNOUNCE_MSG_ID
 *      Announce message ID value
 */
#define TIMESYNC_PTP_ANNOUNCE_MSG_ID                         (0x0BU)

/**
 * @def TIMESYNC_PTP_MGMT_MSG_ID
 *      Management message ID value
 */
#define TIMESYNC_PTP_MGMT_MSG_ID                             (0x0DU)

/**
 * @def TIMESYNC_PTP_FILTER_ALPHA_COEFF
 *      Alpha coefficient for sync interval exponential filter and other filters
 */
#define TIMESYNC_PTP_FILTER_ALPHA_COEFF                      (0.85)

/**
 * @def TIMESYNC_PTP_OFFSET_THRESHOLD_FOR_RESET
 *      If offset from Master goes above this threshold it will
 *      trigger a reset. Value is in nanoseconds
 */
#define TIMESYNC_PTP_OFFSET_THRESHOLD_FOR_RESET              (5000U)

/**
 * @def TIMESYNC_PTP_STABLE_FILTER_THRESHOLD
 *      When clock drift goes below this value it indicates drift has
 *      stabilized and SMA filter can kick in
 */
#define TIMESYNC_PTP_STABLE_FILTER_THRESHOLD                 (100U)

/**
 * @def TIMESYNC_PTP_PEER_DELAY_ERROR_THRESHOLD
 *      When peer delay exceeds this value it gets reset to 0
 *      Sometimes wrong values are calculated when doing peer delay
 *      calculation when clock on peer changes in between peer delay
 *      messages and because of the filter this value stays for a long time
 *      This helps in fixing it. This is set to a value much higher than
 *      max peer delay (of a 100 mtr cable)
 */
#define TIMESYNC_PTP_PEER_DELAY_ERROR_THRESHOLD              (10000U)

/**
 * @def TIMESYNC_PTP_NUM_SYNC_MISSED_THRESHOLD
 *      If this many consecutive sync frames are missed
 *      the DUT gets reset
 */
#define TIMESYNC_PTP_NUM_SYNC_MISSED_THRESHOLD               (3U)

/*Hex Offsets for different fields in PTP Delay Req/Res Packet. Annex E*/
/*To get Annex F offsets subtract TIMESYNC_PTP_ANNEX_D_ANNEX_F_DIFF from it*/

#define TIMESYNC_PTP_PAYLOAD_START_OFFSET                    (42U)//(14U) (42U)
/**
 * @def TIMESYNC_PTP_SRC_MAC_OFFSET
 *      Source MAC address offset
 */
#define TIMESYNC_PTP_SRC_MAC_OFFSET                          (6U)

/**
 * @def TIMESYNC_PTP_SRC_IP_OFFSET
 *      Source IP address offset in IP4/UDP
 */
#define TIMESYNC_PTP_SRC_IP_OFFSET                           (26U)

/**
 * @def TIMESYNC_PTP_DST_IP_OFFSET
 *      Destination IP address offset in IP4/UDP
 */
#define TIMESYNC_PTP_DST_IP_OFFSET                           (30U)

/**
 * @def TIMESYNC_PTP_MSG_ID_OFFSET
 *      PTP message ID offset
 */
#define TIMESYNC_PTP_MSG_ID_OFFSET                           (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 0U)//(42U)

/**
 * @def TIMESYNC_PTP_DOMAIN_NUM_OFFSET
 *      PTP domain number offset
 */
#define TIMESYNC_PTP_DOMAIN_NUM_OFFSET                       (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 4U)//(46U)

/**
 *
 * Offset for PTP flags (including 2 step)
 */
#define TIMESYNC_PTP_FLAG_OFFSET                             (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 6U)//(48U)

/**
 *  @def TIMESYNC_PTP_CORRECTION_OFFSET
 *      Offset for PTP correction field
 */
#define TIMESYNC_PTP_CORRECTION_OFFSET                       (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 8U)//(50U)

/**
 * @def TIMESYNC_PTP_SRC_CLK_IDENTITY
 *      Offset for PTP source clock identity (MAC + protocol id)
 */
#define TIMESYNC_PTP_SRC_CLK_IDENTITY                        (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 20U)//(62U)

/**
 * @def TIMESYNC_PTP_SRC_PORT_ID_OFFSET
 *      Offset source port id
 */
#define TIMESYNC_PTP_SRC_PORT_ID_OFFSET                      (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 28U)//(70U)

/**
 * @def TIMESYNC_PTP_SEQ_ID_OFFSET
 *      Offset for sequence identifier (incremented every frame)
 */
#define TIMESYNC_PTP_SEQ_ID_OFFSET                           (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 30U)//(72U)

/**
 * @def TIMESYNC_PTP_LOG_MSG_PERIOD
 *      Log message interval offset
 */
#define TIMESYNC_PTP_LOG_MSG_PERIOD                          (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 33U)//(75U)

/**
 * @def TIMESYNC_PTP_REQ_RCPT_TS_SEC_OFFSET
 *      Offset for seconds timestamp
 */
#define TIMESYNC_PTP_REQ_RCPT_TS_SEC_OFFSET                  (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 34U)//(76U)

/**
 * @def TIMESYNC_PTP_REQ_RCPT_TS_NSEC_OFFSET
 *      Offset for nanoseconds timestamp
 */
#define TIMESYNC_PTP_REQ_RCPT_TS_NSEC_OFFSET                 (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 40U)//(82U)

/**
 * @def TIMESYNC_PTP_REQ_SRC_PORT_IDENTITY
 *      Offset for requestor clock identity
 */
#define TIMESYNC_PTP_REQ_SRC_PORT_IDENTITY                   (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 44U)//(86U)

/**
 * @def TIMESYNC_PTP_REQ_SRC_PORT_ID
 *      Offset for requestor port identity
 */
#define TIMESYNC_PTP_REQ_SRC_PORT_ID                         (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 52U)//(94U)

/**
 * @def TIMESYNC_PTP_UTC_OFFSET
 *      Offset for UTC offset in Announce frame
 */
#define TIMESYNC_PTP_UTC_OFFSET                              (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 44U)//(86U)

/**
 * @def TIMESYNC_PTP_PRIORITY1_OFFSET
 *      Offset for Priority 1 field in Announce frame
 */
#define TIMESYNC_PTP_PRIORITY1_OFFSET                        (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 47U)//(89U)

/**
 * @def TIMESYNC_PTP_GM_CLK_CLASS_OFFSET
 *      Offset for GM clock class field in Announce frame
 */
#define TIMESYNC_PTP_GM_CLK_CLASS_OFFSET                     (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 48U)//(90U)

/**
 * @def TIMESYNC_PTP_GM_CLK_ACCU_OFFSET
 *      Offset for GM clock accuracy field in Announce frame
 */
#define TIMESYNC_PTP_GM_CLK_ACCU_OFFSET                      (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 49U)//(91U)

/**
 * @def TIMESYNC_PTP_GM_CLK_VARIANCE_OFFSET
 *      Offset for GM clock variance field in Announce frame
 */
#define TIMESYNC_PTP_GM_CLK_VARIANCE_OFFSET                  (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 50U)//(92U)

/**
 * @def TIMESYNC_PTP_PRIORITY2_OFFSET
 *      Offset for priority 2 field in Announce frame
 */
#define TIMESYNC_PTP_PRIORITY2_OFFSET                        (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 52U)//(94U)

/**
 * @def TIMESYNC_PTP_GM_CLK_IDENTITY_OFFSET
 *      Offset for GM clock identity field in Announce frame
 */
#define TIMESYNC_PTP_GM_CLK_IDENTITY_OFFSET                  (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 53U)//(95U)

/**
 * @def TIMESYNC_PTP_STEPS_REMOVED_OFFSET
 *      Offset for steps removed field in Announce frame
 */
#define TIMESYNC_PTP_STEPS_REMOVED_OFFSET                    (TIMESYNC_PTP_PAYLOAD_START_OFFSET + 62U)//(103U)

/**
 * @def TIMESYNC_PTP_TIME_SRC_OFFSET
 *      Offset for GM time source field in Announce frame
 */
#define TIMESYNC_PTP_TIME_SRC_OFFSET                         (04U)

/**
 * @def TIMESYNC_PTP_E2E_BUFFER_OFFSET
 *      Offset for start of PTP data in UDP/IP4 frame
 */
#define TIMESYNC_PTP_E2E_BUFFER_OFFSET                       (42U)

/**
 * @def TIMESYNC_PTP_P2P_BUFFER_OFFSET
 *      Offset for start of PTP data in 802.3 frame
 */
#define TIMESYNC_PTP_P2P_BUFFER_OFFSET                       (14U)

/**
 * @def TIMESYNC_PTP_ANNEX_D_ANNEX_F_DIFF
 *      Number of Extra bytes from Annex F(802.1) to Annex E(UDP)
 */
#define TIMESYNC_PTP_ANNEX_D_ANNEX_F_DIFF                    (28U)

/**
 * @def TIMESYNC_PTP_SINGLE_TAG_VLAN_HDR_SIZE
 *      Number of extra bytes for single tagged VLAN
 */
#define TIMESYNC_PTP_SINGLE_TAG_VLAN_HDR_SIZE                (4U)

/**
 * @def TIMESYNC_PTP_DOUBLE_TAG_VLAN_HDR_SIZE
 *      Number of extra bytes for double tagged VLAN
 */
#define TIMESYNC_PTP_DOUBLE_TAG_VLAN_HDR_SIZE                (8U)

/**
 * @def TIMESYNC_PTP_HSR_CORRECTION
 *      Number of bytes in HSR Header
 */
#define TIMESYNC_PTP_HSR_CORRECTION                          (6U)

/**
 * @def TIMESYNC_PTP_SRC_DST_MAC_SIZE
 *      Num bytes in source plus destination MAC
 */
#define TIMESYNC_PTP_SRC_DST_MAC_SIZE                        (12U)

/**
 * @def TIMESYNC_PTP_TWO_STEP_MASK
 *      Mask used to extract two step bit value
 */
#define TIMESYNC_PTP_TWO_STEP_MASK                           (0x200U)

/**
 * @def TIMESYNC_PTP_RX_MAX_MTU
 *      Rx MTU value for received packets
 */
#define TIMESYNC_PTP_RX_MAX_MTU                              (1518U)

/**
 * @def TIMESYNC_PTP_HSR_ETHERTYPE
 *      Ethertype value for HSR frame
 */
#define TIMESYNC_PTP_HSR_ETHERTYPE                           (0x892F)

/**
 * @def TIMESYNC_PTP_VLAN_ETHERTYPE
 *      Ethertype value for VLAN frame
 */
#define TIMESYNC_PTP_VLAN_ETHERTYPE                          (0x8100)

/**
 * @def TIMESYNC_PTP_SYNT_DEPTH
 *      Syntonization depth.
 *      No. of timestamps stored to do RCF & NRR calculation
 */
#define TIMESYNC_PTP_SYNT_DEPTH                              (3U)

/** Below are the masks used for PTP state machine */

/**
 * @def TIMESYNC_PTP_STATE_MACHINE_FIRST_ADJUSTMENT_DONE
 *      Bit 0. Indicates that driver has copied TS value
 *      directly to IEP registers*/
#define  TIMESYNC_PTP_STATE_MACHINE_FIRST_ADJUSTMENT_DONE    (1U)
/**
 * @def TIMESYNC_PTP_STATE_MACHINE_LINE_DELAY_COMPUTED
 *      Bit 1. Indicates that Peer delay or
 *       Line delay (in case of E2E) has been computed
 */
#define  TIMESYNC_PTP_STATE_MACHINE_LINE_DELAY_COMPUTED      (2U)
/**
 * @def TIMESYNC_PTP_STATE_MACHINE_SYNC_INTERVAL_COMPUTED
 *      Bit 2. Sync interval computed
 */
#define  TIMESYNC_PTP_STATE_MACHINE_SYNC_INTERVAL_COMPUTED   (4U)
/**
 * @def TIMESYNC_PTP_STATE_MACHINE_READY_FOR_SYNC
 *      Mask for all three bits.
 *      Indicates that driver is now ready to do synchronization
 */
#define  TIMESYNC_PTP_STATE_MACHINE_READY_FOR_SYNC           (7U)

/**
 * @def TIMESYNC_PTP_STATE_MACHINE_DEVICE_IN_SYNC
 *      Bit 3. Device is in sync with master
 *      This is continuously monitored
 */
#define  TIMESYNC_PTP_STATE_MACHINE_DEVICE_IN_SYNC           (8U)

/**
 * @def TIMESYNC_PTP_OFFSET_ALGO_BIN_SIZE
 *      Bin size used for stabilization algo
 *      See design doc for details
 */
#define TIMESYNC_PTP_OFFSET_ALGO_BIN_SIZE                    (5U)

/**
 * @def TIMESYNC_PTP_OFFSET_ALGO_CLUSTER_SIZE
 *      Number of entries used for clustering
 *      See design doc for details
 */
#define TIMESYNC_PTP_OFFSET_ALGO_CLUSTER_SIZE                (3U)

//TODO check for the event bits...copied from ti/osal/EventP.h
#define EventP_ID_NONE     0x0
/*!
 *  @brief    Event ID 0
 */
#define EventP_ID_00       0x1
/*!
 *  @brief    Event ID 1
 */
#define EventP_ID_01       0x2
/*!
 *  @brief    Event ID 2
 */
#define EventP_ID_02       0x4
/*!
 *  @brief    Event ID 3
 */
#define EventP_ID_03       0x8
/*!
 *  @brief    Event ID 4
 */
#define EventP_ID_04       0x10
/*!
 *  @brief    Event ID 5
 */
#define EventP_ID_05       0x20
/*!
 *  @brief    Event ID 6
 */
#define EventP_ID_06       0x40
/*!
 *  @brief    Event ID 7
 */
#define EventP_ID_07       0x80
/*!
 *  @brief    Event ID 8
 */
#define EventP_ID_08       0x100
/*!
 *  @brief    Event ID 9
 */
#define EventP_ID_09       0x200
/*!
 *  @brief    Event ID 10
 */
#define EventP_ID_10       0x400
/*!
 *  @brief    Event ID 11
 */
#define EventP_ID_11       0x800
/*!
 *  @brief    Event ID 12
 */
#define EventP_ID_12       0x1000
/*!
 *  @brief    Event ID 13
 */
#define EventP_ID_13       0x2000
/*!
 *  @brief    Event ID 14
 */
#define EventP_ID_14       0x4000
/*!
 *  @brief    Event ID 15
 */
#define EventP_ID_15       0x8000
/*!
 *  @brief    Event ID 16
 */
#define EventP_ID_16       0x10000
/*!
 *  @brief    Event ID 17
 */
#define EventP_ID_17       0x20000
/*!
 *  @brief    Event ID 18
 */
#define EventP_ID_18       0x40000
/*!
 *  @brief    Event ID 19
 */
#define EventP_ID_19       0x80000
/*!
 *  @brief    Event ID 20
 */
#define EventP_ID_20       0x100000
/*!
 *  @brief    Event ID 21
 */
#define EventP_ID_21       0x200000
/*!
 *  @brief    Event ID 22
 */
#define EventP_ID_22       0x400000
/*!
 *  @brief    Event ID 23
 */
#define EventP_ID_23       0x800000
/**
 * @defgroup TIMESYNC_PTP_API PTP/1588 APIs
 */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 * @brief Time synchronization configuration structure
 */
typedef struct
{
    /*Rx buffers*/
    uint8_t *pdelayReqRxBuf[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];
    uint8_t *pdelayResRxBuf[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];
    uint8_t *pdelayResFlwUpRxBuf[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];

    /*Tx buffers*/
    uint8_t *pdelayReqTxBuf[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];
    uint8_t *pdelayResTxBuf[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];
    uint8_t *pdelayResFlwUpTxBuf[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];
    uint8_t *syncTxBuf;
    uint8_t *announceTxBuf;
    uint8_t *followUpTxBuf[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];
    uint8_t *delayReqTxBuf;

    /*Buffer sizes. These change based on whether
     * it's Annex E or Annex F. Pdelay buffer size and
     * delay request (E2E) buffer sizes are fixed
     * because we only support fixed modes*/
    uint8_t syncBufSize;
    uint8_t flwUpBufSize;
    uint8_t announceBufSize;
} TimeSyncPtp_FrameBuffers;

/**
 * @brief Runtime variables for Time Sync implementation
 */
typedef struct
{
    /**Enable/Disable status*/
    uint8_t enabled;

    /**PTP State machine. This is a bitmask*/

    /**
     * @brief PTP State machine used internally by driver
     * This is a bitmap
     * Bit 0 : Set 1 if First adjustment done
     * Bit 1 : Set 1 if delay has been computed
     * Bit 2 : Set 1 if sync interval has been computed
     * Bit 3 : Set 1 if device is in sync with master
     * if first 3 bits are set then clock adjustment is performed
     */
    uint8_t stateMachine;

    /**Sequence ID to be sent for Delay Request/Pdelay Request packets*/
    uint16_t pDelReqSequenceID[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];

    /**Sequence ID of received peer delay request packet*/
    uint16_t rxPDelReqSequenceID[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];

    /**Current sequence id from Sync frame*/
    uint16_t curSyncSeqId[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];

    /**Sync interval between last two sync frames*/
    uint32_t currSyncInterval;

    /**Long term average calculated using an exponential filter*/
    uint64_t ltaSyncInterval;

    /**First sync interval value*/
    uint32_t firstSyncInterval;

    /**Offset from master : current value*/
    int32_t currOffset;

    /**Offset from master : present value*/
    int32_t prevOffset;

    /**Initial offset. Taken when adjustment settles down.*/
    int32_t initialOffset;

    /*Once drift stabilizes, this is set to 1*/
    uint8_t driftStable;

    /*Once offset becomes zero, this is set to 1*/
    uint8_t offsetStable;

    /**Port number on which master is connected*/
    uint8_t syncPortNum;

    /**Line delay or Peer Delay for each port*/
    uint32_t pathDelay[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];

    /*Place holder to calculate line delay.
     * Since line delay can sometimes be negative, we need a persistent variable*/
    int64_t meanPathDelay;

    /**Difference between current and previous offset*/
    uint32_t clockDrift;

    /*long term average of offset. Compare with initial offset to find drift*/
    int32_t ltaOffset;

    /**Running average of correction field in Sync frame*/
    uint32_t avgCorrectionField;

    /**background tick counter, increments inside
     * TimeSyncPtp_backgroundTask()  */
    uint64_t tickCounter;

    /**Ageing counter to detect missed Sync frames*/
    uint32_t syncLastSeenCounter;

    /**Ageing counter is compared against this value
     * to detect missed sync frames. 1.5x the sync interval*/
    uint64_t syncTimeoutInterval;

    /*debug*/
    uint8_t bmcaDone;
} TimeSyncPtp_RuntimeVar;

/**
 * @brief Variables used for offset stabilization
 * algorithm
 */
typedef struct
{
    /**Count for last sync which had drift
     * lower than threshold. This index keeps incrementing
     * for every sync frame and gets reset to 0 if it
     * crosses a threshold.
     */
    uint8_t lastSeenGoodDriftIndex;

    /**Only offsets lower than this threshold are
     * recorded for purpose of stabilization
     */
    uint8_t driftThreshold;

    /**Counter for recording the offsets*/
    uint8_t numEntriesIndex;

    /**Indicating to the averaging algo to process
     * the data in the array
     */
    uint8_t binFull;

    /**Array of correction values with
     * low drift which are clustered together
     */
    int32_t correction[TIMESYNC_PTP_OFFSET_ALGO_BIN_SIZE];
} TimeSyncPtp_OffsetStableAlgo;

/**
 * @brief Parameters required for calculating Cable Delay
 */
typedef struct
{
    /**Set to 1 if Two step sync frame*/
    uint8_t ifTwoStep;

    /**Correction Field. Nanoseconds. Contains sum of sync and followup correction fields
     * in case master is a two step clock*/
    uint64_t correctionField;

    /**Timestamp (seconds). If it's a followup packet then it's precise origin timestamp of follow up*/
    uint64_t originTsSec;
    /**Timestamp in nanoseconds.If it's a followup packet then it's precise origin timestamp of follow up*/
    uint32_t originTsNs;

    /**Input timestamp of Sync Frame*/
    uint32_t rxTs;

    /**Input timestamp of Sync Frame in seconds*/
    uint64_t rxTsSec;

    /**Transmit timestamp of Sync frame on other port in nanoseconds*/
    uint32_t txTs;

    /**Transmit timestamp of Sync frame on other port in Seconds*/
    uint64_t txTsSec;
} TimeSyncPtp_SyncParam;

/**
 * @brief Parameters required for clock syntonization
 */
typedef struct
{
    /**Sync RX TS*/
    uint64_t syncIngressTs[TIMESYNC_PTP_SYNT_DEPTH];

    /**Corrected master TS*/
    uint64_t correctedMasterTs[TIMESYNC_PTP_SYNT_DEPTH];

    /**Syntonization factor*/
    double rcf;

    /**Internal index for book keeping*/
    uint8_t index;

    /** enable syntonization flag*/
    uint8_t syntEnable;

    /**Internal index for book keeping*/
    uint32_t syntIndex;
} TimeSyncPtp_SyntInfo;

/**
 * @brief Parameters required for calculating Nighbor Rate Ratio
 */
typedef struct
{
    /**Rx TS of Peer Delay response*/
    uint64_t deviceRxTS[TIMESYNC_PTP_SYNT_DEPTH];

    /**Origin timestamp of Peer Delay response received in Follow Up*/
    uint64_t correctedPeerTS[TIMESYNC_PTP_SYNT_DEPTH];

    /**Neighbor rate ratio*/
    double nrr;

    /**Internal index for book keeping*/
    uint8_t curIndex;

    /** enable syntonization flag*/
    uint8_t nrrEnable;

    /**Internal index for book keeping*/
    uint32_t nrrIndex;
} TimeSyncPtp_NrrInfo;

/**
 * @brief Parameters required for calculating Cable Delay
 */
typedef struct
{
    /**Set to 1 if Two step Peer delay response*/
    uint8_t ifTwoStep;

    /**Delay Request TX timestamp or T1. In Seconds*/
    uint32_t T1Sec;

    /**Delay Request TX timestamp or T1. In Nanoseconds*/
    uint32_t T1Nsec;

    /**Delay Request Receipt Timestamp T2. In Seconds*/
    uint64_t T2Sec;

    /**Delay Request Receipt Timestamp T2. In Nanoseconds*/
    uint32_t T2Nsec;

    /**Original Delay Response Tx TS (On Peer). In Seconds*/
    uint64_t T3Sec;

    /**Original Delay Response Tx TS (On Peer). In Nanoseconds*/
    uint32_t T3Nsec;

    /**Delay Response receipt Rx TS. In Seconds*/
    uint64_t T4Sec;

    /**Delay Response receipt Rx TS. In Nanoseconds*/
    uint32_t T4Nsec;

    /**Rx timestamp of Peer delay request received. In seconds
     * Used for replying with a peer delay response*/
    uint64_t pDelayReqRcvdTSSec;

    /**Rx timestamp of Peer delay request received. In nanoseconds
     * Used for replying with a peer delay response*/
    uint32_t pDelayReqRcvdTSNsec;

    /**Delay Response Correction Field. T3 - T2. In NS*/
    uint64_t delayResCorrField;

    /**Delay Response Follow Up Correction Field. In NS*/
    uint64_t delayResFwUpCorrField;
} TimeSyncPtp_PeerDelayParams;

/**
 * @brief Used to populate IP/UDP Parameters. These must be populated from Switch params
 */
typedef struct
{
    /**Source IP Address*/
    uint32_t srcIP;

    /**Destination IP Address*/
    uint32_t dstIP;

    /**DSCP Field. Not used right now*/
    uint8_t dscp;

    /**TTL Value, copy from Switch config params or Stack params*/
    uint8_t ttlVal;
} TimeSyncPtp_IpParams;

/**
 * @brief PTPd stack parameters
 */
typedef struct
{
    /**Set this to indicate to PTPd that new frame is available*/
    uint8_t generalFrameFlag;
    /**PTPd stack frame buffer*/
    uint8_t ptpGeneralFrame[1518];
    /**Size of frame to be copied by PTPd*/
    uint16_t ptpGeneralSize;

    /**MAC id of Announce frame, PTPd has no way of knowing the MAC ID without this
     * Revisit the design
     */
    uint8_t ptpSrcMacID[6];
} TimeSyncPtp_PtpDparams;

/**
 * @brief Time synchronization PTP parameter handle structure
 */
typedef struct TimeSyncPtp_Obj_s
{
    /**PTP Enable/Disable status*/
    uint8_t enabled;

    /** Copy of PTP configuration */
    TimeSyncPtp_Config ptpConfig;

    /* TimeSync HAL configuration */
    TimeSync_Config timeSyncCfg;

    /** Handle to TimeSync HAL driver */
    EnetApp_PerCtxt* enet_perctxt;

    /** Sync interval(nanoseconds) */
    uint32_t syncInterval;

    /**PTPd stack parameters*/
    TimeSyncPtp_PtpDparams stackParams;

    /**Line delay or Peer Delay for each port*/
    uint32_t pathDelay[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];

    /*Syntonization index mapping.
     * This is a fixed mapping with a gap of 1 index for syntonization */
    uint8_t syntIndexMap[3];

    /*Syntonization previus index mapping.
     * This is a fixed mapping with a gap of 1 index for syntonization */
    uint8_t prevIndexMap[3];

    /**Clock Identity = MAC ID + Protocol Identifier*/
    uint8_t clockIdentity[8];

    /** Port number to which master clock is connected */
    uint8_t masterPortNum;

    /**Time Sync Task handle for Delay Request Task which sends task periodically*/
    TaskP_Object pDelayReqSendTask;

    /**-----------------Slave tasks, semaphores and events----------------*/
    /**For processing frames like
     * Pdelay Request, Pdelay Response
     * and Pdelay Response Follow Up which
     * don't require real time response
     */
    TaskP_Object nRTTask;

    /**Used to do resource intensive but non-critical tasks
     * like doing floating point calculations, algorithms,
     * status monitoring etc
     */
    TaskP_Object backgroundTask;

    /**Two event handlers for two ports
     * for processing Pdelay response
     * and Pdelay response follow up
     */
    EventP_Object ptpPdelayResEvtHandle[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];

    /**Event ID for Sync frame*/
    uint32_t eventIdSync;

    /**Event ID for Pdelay Request frame*/
    uint32_t eventIdPdelayReq;

    /**Event ID for Pdelay Response frame*/
    uint32_t eventIdPdelayResp;

    /**Event ID for Pdelay Response Follow Up frame*/
    uint32_t eventIdPdelayRespFlwUp;

    /**Event ID for Delay Request frame*/
    uint32_t eventIdDelayReq;

    /**Event ID to indicate follow up frame generated*/
    uint32_t eventIdFlwUpGenerated;

    /*Semaphore handle used to post packet transmission notification */
    SemaphoreP_Object pktTxSemHandle;

    /*Semaphore for indicating packet reception */
    SemaphoreP_Object pktRxSemHandle;

    /**-----------------Master tasks, semaphores and events----------------*/
    /** Tx packet notify task handle*/
    TaskP_Object pktTxNotifyTask;

    /** Rx packet notify task handle*/
    TaskP_Object pktRxNotifyTask;

    /*Number of sync frames missed*/
    uint32_t numSyncMissed;

    TimeSyncPtp_OffsetStableAlgo offsetAlgo;

    /**IP/UDP Params*/
    TimeSyncPtp_IpParams udpParams;

    /**Peer Delay params*/
    TimeSyncPtp_PeerDelayParams pDelayParams[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];

    /*Rx and Tx frame buffers*/
    TimeSyncPtp_FrameBuffers timeSyncBuff;

    /**Data for Syntonization*/
    TimeSyncPtp_SyntInfo tsSyntInfo;

    /**Data for calculating NRR on both ports*/
    TimeSyncPtp_NrrInfo tsNrrInfo[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];

    /**Delay params for Sync Frame*/
    TimeSyncPtp_SyncParam syncParam[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];

    /**Time Sync Runtime variables*/
    TimeSyncPtp_RuntimeVar tsRunTimeVar;
} TimeSyncPtp_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void TimeSyncPtp_processRxNotify(void *arg);

void TimeSyncPtp_processTxNotify(void *arg,
                                 uint8_t portNum,
                                 uint8_t frameType,
                                 uint16_t seqId);
/**
 * @brief Do initial adjustment for the IEP based on sync timestamp
 * @param timeSyncPtpHandle pointer to PTP Handle structure
 * @param portNum ICSS_EMAC_PORT_1 or ICSS_EMAC_PORT_2
 * @return None
 */
void TimeSyncPtp_doFirstAdjustment(TimeSyncPtp_Handle timeSyncPtpHandle,
                                   uint8_t portNum);

/**
 * @brief Processes a PTP Announce message and extract fields
 * @param timeSyncPtpHandle pointer to PTP Handle structure
 * @param pktBuffer pointer to packet data
 * @param portNum port number on which the packet was received
 * @param size size of the packet
 * @param isLinkLocal
 *
 * @return none
 */
void TimeSyncPtp_processPtpFrame(TimeSyncPtp_Handle hTimeSyncPtp,
                                 uint8_t *pktBuffer,
                                 uint8_t portNum,
                                 uint32_t size,
                                 uint8_t isLinkLocal);

/**
 * @internal
 * @brief Calculate Peer Delay given the parameters from Peer Delay Req/Res
 * @param timeSyncPtpHandle pointer to PTP Handle structure
 * @param twoStep Indicates if calculation is through 2 step mechanism
 * @param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 * @return none
 */
void TimeSyncPtp_peerDelayCalc(TimeSyncPtp_Handle timeSyncPtpHandle,
                               uint8_t twoStep,
                               uint8_t portNum);

/**
 * @internal
 * @brief Process Sync/Follow Up frames and do synchronization
 * @param timeSyncPtpHandle pointer to PTP Handle structure
 * @param buff pointer to the sync and follow up frame
 * @param followUp Set to 1 if it's a follow up frame else 0
 * @param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 * @param size size of the frame
 * @return none
 */
void TimeSyncPtp_processSyncFrame(TimeSyncPtp_Handle timeSyncPtpHandle,
                                  uint8_t *buff,
                                  uint8_t followUp,
                                  uint8_t portNum,
                                  uint32_t size);

/**
 * @internal
 * @brief Process Pdelay response and Pdelay response followUp frame
 * @param  timeSyncPtpHandle pointer to PTP Handle structure
 * @param buff pointer to the Pdelay Response/Response Follow Up Frame
 * @param followUp 0 if it's a response frame, 1 if response follow up
 * @param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 * @return none
 */
void TimeSyncPtp_processPdelayRespFrame(TimeSyncPtp_Handle timeSyncPtpHandle,
                                        uint8_t *buff,
                                        uint8_t followUp,
                                        uint8_t portNum);

/**
 * @internal
 * @brief Synchronize clock upon receipt of Sync Frame. Applicable for OC
 * This is called from inside the interrupt context
 * @param  timeSyncPtpHandle pointer to PTP Handle structure
 * @return none
 */
void TimeSyncPtp_synchronizeClock(TimeSyncPtp_Handle timeSyncPtpHandle);

/**
 * @internal
 * @brief Calculate Neighbor rate ratio for adjacent nodes
 * @param  timeSyncPtpHandle pointer to PTP Handle structure
 * @param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 * @return none
 */
void TimeSyncPtp_calcNRR(TimeSyncPtp_Handle timeSyncPtpHandle,
                         uint8_t portNum);

/**
 * @internal
 * @brief Updates NRR info which is subsequently used to calculate
 * the final NRR value
 * @param  timeSyncPtpHandle pointer to PTP Handle structure
 * @param portNum port number. ICSS_EMAC_PORT_1/ICSS_EMAC_PORT_2
 * @return none
 */
void TimeSyncPtp_updateNRRParams(TimeSyncPtp_Handle timeSyncPtpHandle,
                                 uint8_t portNum);

/**
 * @internal
 * @brief Calculate rcf for syntonization from line delay parameters
 * @param  timeSyncPtpHandle pointer to PTP Handle structure
 * @return none
 */
void TimeSyncPtp_calcRcfAndSyncInterval(TimeSyncPtp_Handle timeSyncPtpHandle);

/**
 * @internal
 * @brief Dummy BMCA which takes first sync frame sender as master.
 * @param  timeSyncPtpHandle pointer to PTP Handle structure
 * @param  pktBuffer pointer to first sync frame
 * @param  portNum port number in which first sync was received
 *
 * @return none
 */
void TimeSyncPtp_BMCA(TimeSyncPtp_Handle timeSyncPtpHandle,
                      uint8_t *pktBuffer,
                      uint8_t portNum);
#endif /* TIMESYNC_PTP_PRIV_H_ */

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

#ifndef ICSS_TIMESYNC_API_H_
#define ICSS_TIMESYNC_API_H_

#ifdef __cplusplus
extern "C" {
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <networking/icss_emac/icss_emac.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/TimerP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/EventP.h>

#define lswap(x) ((((x) & 0xff000000) >> 24) | \
                  (((x) & 0x00ff0000) >>  8) | \
                  (((x) & 0x0000ff00) <<  8) | \
                  (((x) & 0x000000ff) << 24))

#define htonl_ts(l) (lswap(l))
#define ntohl_ts(l) (lswap(l))

#define htons_ts(s) ((u_short)((u_short)(s) >> 8) | (u_short)((u_short)(s) << 8))
#define ntohs_ts(s) htons_ts(s)
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*TODO: Review this*/
#define TASK_STACK_SIZE (2048)

#define ICSS_TIMESYNC_EVENT_BIT0    (0x000001)
#define ICSS_TIMESYNC_EVENT_BIT1    (0x000002)
#define ICSS_TIMESYNC_EVENT_BIT2    (0x000004)
#define ICSS_TIMESYNC_EVENT_BIT3    (0x000008)
#define ICSS_TIMESYNC_EVENT_BIT4    (0x000010)
#define ICSS_TIMESYNC_EVENT_BIT5    (0x000020)
#define ICSS_TIMESYNC_EVENT_BIT6    (0x000040)
#define ICSS_TIMESYNC_EVENT_BIT7    (0x000080)

/* Data types */
/* Primitive data types */

/**
 * \def SYNT_DEPTH
 *      Syntonization depth.
 *      No. of timestamps stored to do RCF & NRR calculation
 */
#define SYNT_DEPTH  3
/**
 * \def TS_NUM_DOMAINS
 *      Number of domains supported by implementation
*/
#define TS_NUM_DOMAINS  1

/*Below are the masks used for PTP state machine*/

/**
 * \def TS_STATE_MACHINE_FIRST_ADJUSTMENT_DONE
 *      Bit 0. Indicates that driver has copied TS value
 *      directly to IEP registers*/
#define  TS_STATE_MACHINE_FIRST_ADJUSTMENT_DONE             1
/**
 * \def TS_STATE_MACHINE_LINE_DELAY_COMPUTED
 *      Bit 1. Indicates that Peer delay or
 *       Line delay (in case of E2E) has been computed
 */
#define  TS_STATE_MACHINE_LINE_DELAY_COMPUTED               2
/**
 * \def TS_STATE_MACHINE_SYNC_INTERVAL_COMPUTED
 *      Bit 2. Sync interval computed
 */
#define  TS_STATE_MACHINE_SYNC_INTERVAL_COMPUTED            4
/**
 * \def TS_STATE_MACHINE_READY_FOR_SYNC
 *      Mask for all three bits.
 *      Indicates that driver is now ready to do synchronization
 */
#define  TS_STATE_MACHINE_READY_FOR_SYNC                    7

/**
 * \def TS_STATE_MACHINE_DEVICE_IN_SYNC
 *      Bit 3. Device is in sync with master
 *      This is continuously monitored
 */
#define  TS_STATE_MACHINE_DEVICE_IN_SYNC                    8

/**
 * \def OFFSET_ALGO_BIN_SIZE
 *      Bin size used for stabilization algo
 *      See design doc for details
 */
#define OFFSET_ALGO_BIN_SIZE    5

/**
 * \def OFFSET_ALGO_CLUSTER_SIZE
 *      Number of entries used for clustering
 *      See design doc for details
 */
#define OFFSET_ALGO_CLUSTER_SIZE   3

/**
 * \brief Port States, from IEEE 1588v2
 */
typedef enum
{
    INITIALIZING       =  1,
    FAULTY             =  2,
    DISABLED           =  3,
    LISTENING          =  4,
    PRE_MASTER         =  5,
    MASTER             =  6,
    PASSIVE            =  7,
    UNCALIBRATED       =  8,
    SLAVE              =  9

} port_State;

/**
 * \brief Network Protocol Type
 */
typedef enum
{

    /**IPv4*/
    UDP_IPV4 = 1,
    /**IPv6*/
    UDP_IPV6 = 2,
    /**Ethernet without TCP/IP*/
    IEEE_802_3 = 3,
    DEVICE_NET = 4,
    CONTROL_NET = 5,
    PROFINET = 6,
    UNKNOWN = 0xFFFE

} netWorkProtType;

/**
 * \brief Current Device configuration
 */
typedef enum
{

    /**Only Transparent Clock*/
    TC  = 1,
    /**Only Ordinary Clock*/
    OC =  2,
    /**Both OC and TC*/
    BOTH = 3
} deviceConfig;

/**
 * \brief PTP Delay Mechanism Type. From Table 9 of Standard
 */
typedef enum
{
    /**Used for CIP Sync*/
    E2E  = 1,
    /**Used for IEC 61558*/
    P2P =  2
} delayType;

/**
 * \brief PTP Delay Mechanism Type. From Table 9 of Standard
 */
typedef enum
{
    /**Non Sync or Delay Req/Res frame*/
    NON_SYNC_DELAY_FRAME  = 0,
    /**SYNC Frame*/
    SYNC_FRAME =  1,
    /**Delay Request Frame*/
    DELAY_REQ_FRAME =  2,
    /**Delay Response Frame*/
    DELAY_RESP_FRAME =  3

} ptpFrameTypes_t;

/**
 * \brief 802.1AS-rev Enumeration8
 */
typedef enum
{
    ATOMIC_CLOCK = 0x10,
    GPS = 0x20,
    TERRESTRIAL_RADIO = 0x30,
    PTP = 0x40,
    NTP = 0x50,
    HAND_SET = 0x60,
    OTHER = 0x90,
    INTERNAL_OSCILLATOR = 0xA0,
} timeSource;

/**
 * \brief 802.1AS-rev Timestamp structure
 */
typedef struct
{
    uint64_t seconds;           /**<48 bit seconds field*/
    uint32_t nanoseconds;       /**<32 bit nanoseconds field*/
} timeStamp;


/**
 * \brief 802.1AS-rev ExtendedTimestamp
 */
typedef struct
{
    uint64_t seconds;                   /**<48 bit seconds*/
    uint64_t fractionalNanoseconds;     /**<48 bit nanoseconds*/
} ExtendedTimestamp;

/**
 * \brief Sync loss callback function.
 * Call cross module functions in case of loss of synchronization
 */
typedef void (*TimeSync_SyncLossCallBack_t)(void);

/**
 * \brief PTP Stack Reset callback function.
 * Timesync handle (of type TimeSync_ParamsHandle_t) will be passed as arg.
 */
typedef void (*TimeSync_stackResetCallback_t)(void *arg);

/**
 * \brief Parent clock params
 */
typedef struct
{
    /**Clock Identity*/
    uint8_t clockIdentity[8];
    /**PTP Port Number*/
    uint16_t ptpPortNumber;
    /**Observed offset scaled log variance field in standard. Refer section 7.6.4.3.
     * The variance is based on the theory of Allan deviation. The number of samples
     * is 3, and the interval between samples will be sync interval. */
    uint16_t observedOffsetScaledLogVariance;
    /**Observed phase change rate field in standard. Refer section 7.6.4.4 */
    int32_t observedClockPhaseChangeRate;
} TimeSync_parent_params_t;

/**
 * \brief Master and grand master clock params
 */
typedef struct
{
    /**Priority 1 field in standard. Refer section 7.6.2.2*/
    uint8_t priority1;
    /**Priority 2 field in standard. Refer section 7.6.2.3*/
    uint8_t priority2;
    /**clockClass field in standard. Refer section 7.6.2.4*/
    uint8_t clockClass;
    /**clockAccuracy field in standard. Refer section 7.6.2.5*/
    uint8_t clockAccuracy;
    /**timeSource field in standard. Refer section 7.6.2.6*/
    uint8_t timeSource;
    /**Applicable to Boundary clocks. Refer section 8.2.2.2*/
    uint16_t stepRemoved;
    /**clockVariance field in standard. Refer section 7.6.3*/
    uint16_t clockVariance;
    /*Current UTC offset*/
    uint16_t UTCOffset;
    /**Grand master identity*/
    uint8_t gmIdentity[8];

    /*PTP Flag params. See standard
     * index 0 - PTP Leap second 61
     * index 1 - PTP Leap second 59
     * ....
     * ....
     * index 11 - PTP Security*/

    uint8_t ptp_flags[12];

} TimeSync_master_params_t;

/**
 * \brief Time synchronization configuration structure
 */
typedef struct
{
    /*Rx buffers*/
    uint8_t *pdelayReq_RxBuf[2];
    uint8_t *pdelayRes_RxBuf[2];
    uint8_t *pdelayResFlwUp_RxBuf[2];
    /*For E2E master. Not supported right now*/
    uint8_t *delayReq_RxBuf[2];

    /*Tx buffers*/
    uint8_t *pdelayReq_TxBuf[2];
    uint8_t *pdelayRes_TxBuf[2];
    uint8_t *pdelayResFlwUp_TxBuf[2];
    uint8_t *sync_TxBuf;
    uint8_t *announce_TxBuf;
    uint8_t *followUp_TxBuf[2];
    uint8_t *delayReq_TxBuf;

    /*Buffer sizes. These change based on whether
     * it's Annex E or Annex F. Pdelay buffer size and
     * delay request (E2E) buffer sizes are fixed
     * because we only support fixed modes*/
    uint8_t  syncBuf_size;
    uint8_t  flwUpBuf_size;
    uint8_t  announceBuf_size;

} TimeSync_frame_buffers_t;

/**
 * \brief Time synchronization configuration structure
 */
typedef struct
{
    /**List of 802.1AS-rev domainNumbers*/
    uint8_t domainNumber[TS_NUM_DOMAINS];
    /**SYNCOUT sync0 start time*/
    uint32_t syncOut_sync0Start;
    /**SYNCOUT sync0 pulse width*/
    uint32_t syncOut_sync0PWidth;
    /*Synchronization loss callback function*/
    TimeSync_SyncLossCallBack_t timeSyncSyncLossCallBackfn;
    /*PTP stack reset callback function*/
    TimeSync_stackResetCallback_t ptpDrvStackReset;
    /**PDelay Request interval. Actual value is 2^value seconds*/
    int8_t  logPDelReqPktInterval;

    /**Sync send interval. Actual value is 2^value seconds*/
    int8_t  logSyncInterval;

    /**Announce send interval. Actual value is 2^value seconds*/
    int8_t  logAnnounceSendInterval;

    /**Announce receipt timeout interval. Actual value is 2^value seconds*/
    int8_t  logAnnounceRcptTimeoutInterval;

    /**Delay Request/PDelay Request interval. Mapped here from logPDelReqPktInterval
     * Value in milliseconds*/
    uint32_t pDelReqPktInterval;

    /**Log of sync send interval. Mapped here from logSyncInterval
     * value in microseconds*/
    uint32_t syncSendInterval;

    /**Log of Announce send interval. Mapped here from logAnnounceSendInterval
     * value in milliseconds*/
    uint32_t announceSendInterval;

    /**Log of Announce receipt timeout interval. Min value of 3
     * value in milliseconds*/
    uint32_t announceRcptTimeoutInterval;

    /**Whether clock is OC or TC or both*/
    deviceConfig config;

    /**Packet type in which PTP is encapsulated*/
    netWorkProtType protocol;

    /**Which type of Measurement scheme. E2E or P2P*/
    delayType type;

    /**PTP Frame Tx Interrupt number*/
    uint32_t  txIntNum;

    /**Delay asymmetry correction value for both ports*/
    uint16_t asymmetryCorrection[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

    /**In a Pdelay request burst packets are sent at this interval
     * value in miliseconds*/
    uint32_t  pdelayBurstInterval;

    /**In a Pdelay request burst these many frames are sent*/
    uint8_t   pdelayBurstNumPkts;

    /**Clock Identity = MAC ID + Protocol Identifier*/
    uint8_t clockIdentity[8];

    /**ICSS generates a periodic pulse, can be configured from 1ms to 1s.
     * When the interval is configured to 1s it becomes the 1PPS signal
     * but has been kept to make it familiar*/
    uint32_t sync0_interval;

    /**Whether HSR is enabled or not*/
    uint8_t hsrEnabled;

    /**This field is only applicable to HSR frames.
     * This is set to 1 if link local frames carry HSR tag
     * By default this is set to 0 which means no HSR tag for
     * Peer delay request/response frames.
     * It Can be overridden from application (see HSR/PRP example)
     * This gets over ridden in the driver once the peer sends delay request response frames
     * */
    uint8_t ll_has_hsrTag;

    /**Set to 1 if a customized Tx API is being used*/

    uint8_t custom_tx_api;

    /**Set to 1 if Rx timestamps are being read from ICSS shared RAM,
     * a value of 0 indicates that timestamps are appended to the PTP frames
     * being received in L3 RAM
     */
    uint8_t timestamp_from_shared_ram;

    /**Set this to 1 to configure PTP to EMAC mode which means no
     * bridge delay computation and forwarding
     */
    uint8_t emac_mode;

    /**All PTP packet offsets are w.r.t PTP Layer 3 packets
     * an offset variable is hence used to calculate actual offset
     * based on whether the configuration is E2E + L3 or P2P + L2
     */
    uint8_t frame_offset;

    /*OS tick period (in microseconds). Normalized value is 1000 means factor of 1, 500 means
     * OS counts twice as fast so divide time period by 2*/
    uint32_t tickPeriod;

    /* Rx PHY Latency in nano seconds */
    uint32_t rxPhyLatency;

    /* Tx PHY Latency in nano seconds */
    uint32_t txPhyLatency;

    /* Priority of task which sends Pdelay request frames */
    uint8_t delayReqSendTaskPriority;

    /* Priority of task which takes time stamp and processes them */
    uint8_t txTsTaskPriority;

    /* Priority of task which processes Peer delay frames in the background and      calculate peer delay. NRT stands for non real time.*/
    uint8_t nrtTaskPriority;

    /* Priority of task which Checks for Sync timeout and performs offset stabilization */
    uint8_t backgroundTaskPriority;

    /**Grand master parameters like priority 1, priority 2 etc. Populated in Announce frame*/
    TimeSync_master_params_t masterParams;

} TimeSync_Config_t;

/**
 * \brief Runtime variables for Time Sync implementation
 */
typedef struct
{
    /**Enable/Disable status*/
    uint8_t enabled;

    /**PTP State machine. This is a bitmask*/
    /**
     * \brief PTP State machine used internally by driver
     * This is a bitmap
     * Bit 0 : Set 1 if First adjustment done
     * Bit 1 : Set 1 if delay has been computed
     * Bit 2 : Set 1 if sync interval has been computed
     * Bit 3 : Set 1 if device is in sync with master
     * if first 3 bits are set then clock adjustment is performed
     */
    uint8_t stateMachine;

    /**Sequence ID to be sent for Delay Request/Pdelay Request packets*/
    uint16_t pDelReqSequenceID[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

    /**Sequence ID used for Delay request frames*/
    uint16_t delReqSequenceID;

    /**Sequence ID used for Sync frames*/
    uint16_t syncSequenceID;

    /**Sequence ID used for announce frames*/
    uint16_t announceSequenceID;

    /**Current sequence id from Sync frame*/
    uint16_t curSyncSeqId[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

    /**Sync interval between last two sync frames*/
    uint32_t currSyncInterval;
    /**Long term average calculated using an exponential filter*/
    uint32_t ltaSyncInterval;
    /**First sync interval value*/
    uint32_t firstSyncInterval;

    /**Offset from master : current value*/
    int32_t currOffset;
    /**Offset from master : previous values.
     * prevOffset[0] is the old value of currOffset.
     * prevOffset[1] is the old value of prevOffset[0].
    */
    int32_t prevOffset[2];

    /*Flags to determine if values in prevOffset[] are valid*/
    int32_t prevOffsetValid[2];

    /**Initial offset. Taken when adjustment settles down.*/
    int32_t initialOffset;

    /*Once drift stabilizes, this is set to 1*/
    uint8_t driftStable;

    /*Once offset becomes zero, this is set to 1*/
    uint8_t offsetStable;

    /**Port number on which master is connected*/
    uint8_t  syncPortNum;

    /**Line delay or Peer Delay for each port*/
    uint32_t pathDelay[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

    /*Place holder to calculate line delay.
     * Since line delay can sometimes be negative, we need a persistent variable*/
    int64_t meanPathDelay;

    /**Difference between current and previous offset*/
    uint32_t clockDrift;

    /*long term average of offset. Compare with initial offset to find drift*/
    int32_t ltaOffset;

    /*Flags to determine if ltaOffset value is valid*/
    int32_t ltaOffsetValid;

    /**Running average of correction field in Sync frame*/
    uint32_t avgCorrectionField;

    /**background tick counter, increments inside
     * TimeSync_BackgroundTask()  */
    uint64_t tickCounter;

    /**Ageing counter to detect missed Sync frames*/
    uint32_t syncLastSeenCounter;

    /**Ageing counter is compared against this value
     * to detect missed sync frames. 1.5x the sync interval*/
    uint32_t syncTimeoutInterval;

    /**Flag to indicate that forced 2-step mode is active*/
    uint8_t forced2step[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

    /*debug*/
    uint8_t bmcaDone;

    /**Grand master parameters like priority 1, priority 2 etc. Populated in Announce frame*/
    TimeSync_master_params_t masterParams;

    /**Parent clock parameters*/
    TimeSync_parent_params_t parentParams;

} timeSync_RuntimeVar_t;

/**
 * \brief Variables used for offset stabilization
 * algorithm
 */
typedef struct
{
    /**Count for last sync which had drift
     * lower than threshold. This index keeps incrementing
     * for every sync frame and gets reset to 0 if it
     * crosses a threshold.
     */
    uint8_t lastSeen_good_drift_index;

    /**Only offsets lower than this threshold are
     * recorded for purpose of stabilization
     */
    uint8_t driftThreshold;

    /**Counter for recording the offsets*/
    uint8_t num_entries_index;

    /**Indicating to the averaging algo to process
     * the data in the array
     */
    uint8_t binFull;

    /**Array of correction values with
     * low drift which are clustered together
     */
    int32_t correction[OFFSET_ALGO_BIN_SIZE];

} timeSync_Offset_Stable_Algo_t;

/**
 * \brief Parameters required for calculating Cable Delay
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

} syncParam_t;

/**
 * \brief Parameters required for clock syntonization
 */
typedef struct
{
    /**Sync RX TS*/
    uint64_t syncIngressTs[SYNT_DEPTH];

    /**Corrected master TS*/
    uint64_t correctedMasterTs[SYNT_DEPTH];

    /**Syntonization factor*/
    double  rcf;

    /**Internal index for book keeping*/
    uint8_t   index;

    /** enable syntonization flag*/
    uint8_t    syntEnable;

    /**Internal index for book keeping*/
    uint32_t   syntIndex;

} timeSync_SyntInfo_t;

/**
 * \brief Parameters required for calculating Nighbor Rate Ratio
 */
typedef struct
{
    /**Rx TS of Peer Delay response*/
    uint64_t deviceRxTS[SYNT_DEPTH];

    /**Origin timestamp of Peer Delay response received in Follow Up*/
    uint64_t correctedPeerTS[SYNT_DEPTH];

    /**Neighbor rate ratio*/
    double  nrr;

    /**Internal index for book keeping*/
    uint8_t   curIndex;

    /** enable syntonization flag*/
    uint8_t    nrrEnable;

    /**Internal index for book keeping*/
    uint32_t   nrrIndex;

} timeSync_NrrInfo_t;


/**
 * \brief Parameters required for calculating Cable Delay
 */
typedef struct
{
    /**Correction Field. Nanoseconds*/
    uint64_t correctionField;

    /**Delay Response Rx Timestamp on master in (seconds)*/
    uint64_t   timeStampSec;
    /**Delay Response Rx Timestamp on master in nanoseconds*/
    uint32_t   timeStampNS;

    /**Delay Request Tx Timestamp in Seconds*/
    uint32_t delReqTxTsSec;
    /**Delay Request Tx Timestamp in Nanoseconds*/
    uint32_t delReqTxTsNS;

} delayReqRespParams_t;

/**
 * \brief Parameters required for calculating Cable Delay
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

} peerDelayParams_t;

/**
 * \brief Used to populate IP/UDP Parameters. These must be populated from Switch params
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

} ipParams_t;

/**
 * \brief PTPd stack parameters
 */
typedef struct
{
    /**Set this to indicate to PTPd that new frame is available*/
    uint8_t generalFrameFlag;
    /**PTPd stack frame buffer*/
    uint8_t ptpGeneralFrame[ICSS_EMAC_MAXMTU];
    /**Size of frame to be copied by PTPd*/
    uint16_t ptpGeneralSize;
    /**MAC id of Announce frame, PTPd has no way of knowing the MAC ID without this
     * Revisit the design
     */
    uint8_t ptpSrcMacID[6];

} ptpDparams_t;

/**
 * \brief Time synchronization parameter handle structure
 */
typedef struct TimeSync_ParamsHandle_s
{
    /**ICSS EMAC handle*/
    ICSS_EMAC_Handle emacHandle;

    uint8_t             macId[6];
    /* MAC ID passed to ICSS-EMAC during ICSS-EMAC initialization*/

    /**PRUICSS handle*/
    PRUICSS_Handle pruicssHandle;

    /**PTP Enable/Disable status*/
    uint8_t enabled;

    /*Get TX timestamp*/
    ptpFrameTypes_t
    txFrameType;        /**<Frame type transmitted. Valid values: 0,1,2,3                               */
    uint8_t txPort;                     /**<Port used for transmit. Valid values: ICSS_EMAC_PORT_1/ ICSS_EMAC_PORT_2    */
    uint8_t txprotocol;                 /**<Transmitted frame protocol. Valid values: gPTP/PTCP 0/1                     */
    uint8_t sequenceIdOut_txTS;         /**<*/

    /*Get RX timestamp*/
    ptpFrameTypes_t
    rxFrameType;                /**<Frame type received. Valid values: 0,1,2,3                                  */
    uint8_t rxPort;                     /**<Port used for receive. Valid values: ICSS_EMAC_PORT_1/ ICSS_EMAC_PORT_2     */
    uint8_t rxprotocol;                 /**<protocol. Valid values: gPTP/PTCP 0/1                                       */
    timeStamp *rxTimestamp_gPTP;        /**<timestamp if rxprotocol=gPTP                                                */
    uint8_t sequenceIdOut_rxTS;         /**<Sequence ID                                                                 */

    /*Set clock*/
    timeStamp clockTime;                /**<Clock time to set               */
    uint8_t sequenceIdIn;               /**<Sequence ID                     */

    /*Adjust clock*/
    int32_t drift;                      /**<Drift(nanoseconds)              */
    uint32_t syncInterval;              /**<Sync interval(nanoseconds)      */

    /**PTPd stack parameters*/
    ptpDparams_t stackParams;

    /**Line delay or Peer Delay for each port*/
    uint32_t pathDelay[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

    uint8_t syntIndexMap[3];
    uint8_t prevIndexMap[3];


    /**Time Sync Task handle for Delay Request Task which sends task periodically*/
    TaskP_Object timeSync_pDelayReqSendTask;

    /**Time Sync Task handle for sending a delay request to Master on receiving a Sync*/
    TaskP_Object timeSync_delayReqSendTask;

    /*-----------------Slave tasks, semaphores and events----------------*/

    /**PTP Task for storing Tx timestamps for Port 1*/
    TaskP_Object timeSync_TxTSTaskP1;
    /**PTP Task for storing Tx timestamps for Port 2*/
    TaskP_Object timeSync_TxTSTaskP2;

    /**For processing frames like
     * Pdelay Request, Pdelay Response
     * and Pdelay Response Follow Up which
     * don't require real time response
     */
    TaskP_Object timeSync_NRT_Task;

    /**Used to do resource intensive but non-critical tasks
     * like doing floating point calculations, algorithms,
     * status monitoring etc
     */
    TaskP_Object timeSync_backgroundTask;

    /**Event handle for indicating that a Tx Timestamp interrupt
     * has been generated and needs to be serviced. index 0 contains
     * handle for Port 1.
     */
    EventP_Object txTSAvailableEvtObject[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

    /**Two event handlers for two ports
     * for processing Pdelay response
     * and Pdelay response follow up
     */
    EventP_Object ptpPdelayResEvtObject[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /**Two event handlers for two ports
     * for processing Pdelay request frames
     */
    EventP_Object ptpPdelayReqEvtObject[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /**Two event handlers for two ports
     * for sending out follow up frame in case of forced 2-step
     */
    EventP_Object ptpSendFollowUpEvtObject[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

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

    /*Semaphore for sending Delay Request frames*/
    SemaphoreP_Object delayReqTxSemObject;

    HwiP_Object timeSync_txTSIsrObject;

    /*Number of sync frames missed*/
    uint32_t numSyncMissed;

    timeSync_Offset_Stable_Algo_t *offsetAlgo;

    /**IP/UDP Params*/
    ipParams_t udpParams;

    /**Peer Delay params*/
    peerDelayParams_t pDelayParams[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

    /**Delay params for Delay Response*/
    delayReqRespParams_t *delayParams;

    /*Time sync configuration handle*/
    TimeSync_Config_t timeSyncConfig;

    /*Rx and Tx frame buffers*/
    TimeSync_frame_buffers_t timeSyncBuff;

    /**Data for Syntonization*/
    timeSync_SyntInfo_t *tsSyntInfo;

    /**Data for calculating NRR on both ports*/
    timeSync_NrrInfo_t *tsNrrInfo[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

    /**Delay params for Sync Frame*/
    syncParam_t *syncParam[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

    /**Time Sync Runtime variables*/
    timeSync_RuntimeVar_t *tsRunTimeVar;

    uint32_t pDelayReqSendTaskStack[TASK_STACK_SIZE/sizeof(uint32_t)]   __attribute__((aligned(32)));
    uint32_t delayReqSendTaskStack[TASK_STACK_SIZE/sizeof(uint32_t)]    __attribute__((aligned(32)));
    uint32_t txTSTaskP1Stack[TASK_STACK_SIZE/sizeof(uint32_t)]          __attribute__((aligned(32)));
    uint32_t txTSTaskP2Stack[TASK_STACK_SIZE/sizeof(uint32_t)]          __attribute__((aligned(32)));
    uint32_t NRT_TaskStack[TASK_STACK_SIZE/sizeof(uint32_t)]            __attribute__((aligned(32)));
    uint32_t backgroundTaskStack[TASK_STACK_SIZE/sizeof(uint32_t)]      __attribute__((aligned(32)));

} TimeSync_ParamsHandle;

typedef struct TimeSync_ParamsHandle_s *TimeSync_ParamsHandle_t;

/* ========================================================================== */
/*                                Error Codes                                 */
/* ========================================================================== */
#define TIMESYNC_OK                     (0)
#define ERROR_TX_FRAMETYPE_NOTVALID     (-1)
#define ERROR_TX_PORTNUMBER_NOTVALID    (-2)
#define ERROR_HANDLE_INVALID            (-3)
#define ERROR_RX_FRAMETYPE_NOTVALID     (-4)
#define ERROR_RX_PORTNUMBER_NOTVALID    (-5)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \addtogroup NETWORKING_ICSS_TIMESYNC_MODULE
 *  @{
 */

/**
 * \brief Reads and provides timestamp of a received event message.
 *
 * \param[in] timeSyncParamsHandle Structure of type TimeSync_ParamsHandle_t
 * \param[in] rxFrameType          (INPUT) frame type received Valid values: 0,1,2,3                               \n
 * \param[in] rxPort               (INPUT) Port used for receive ICSS_EMAC_PORT_1/ ICSS_EMAC_PORT_2                \n
 * \param[out] nanoseconds          (OUTPUT) ptr to 32 bit nanoseconds field
 * \param[out] seconds              (OUTPUT)  ptr to 64 bit(48 used) seconds field
 *
 * \retval <0 value on error
 * \retval  0 on success
 *
 */
int32_t TimeSync_getRxTimestamp(TimeSync_ParamsHandle_t timeSyncParamsHandle,
                                ptpFrameTypes_t rxFrameType, uint8_t rxPort, \
                                uint32_t *nanoseconds, uint64_t *seconds);

/**
 * \brief Reads and provides timestamp of a received event message.
 *
 * \param[in] timeSyncParamsHandle Structure of type TimeSync_ParamsHandle_t
 * \param[in] rxPort               (INPUT) Port used for receive ICSS_EMAC_PORT_1/ ICSS_EMAC_PORT_2                \n
 * \param[out] nanoseconds          (OUTPUT) ptr to 32 bit nanoseconds field
 * \param[out] seconds              (OUTPUT)  ptr to 64 bit(48 used) seconds field
 * \param[out] TimeStampFromFrame   (OUTPUT)  frame timestamp
 * \retval <0 value on error
 * \retval  0 on success
 *
 */
int32_t TimeSync_getRxTimestampFromFrame(TimeSync_ParamsHandle_t timeSyncParamsHandle,
                                uint8_t rxPort, \
                                uint32_t *nanoseconds, uint64_t *seconds, uint8_t* TimeStampFromFrame);
/* API */

/**
 * \brief Function to configure domain number and/or log message intervals and/or
 *        sync0 signal start time, pulse width
 *
 * \param[in] timeSyncHandle pointer to PTP Handle structure with updated values
 *                           in following members :
 *                           timeSyncHandle->timeSyncConfig.domainNumber, and/or
 *                           timeSyncHandle->timeSyncConfig.logSyncInterval, and/or
 *                           timeSyncHandle->timeSyncConfig.logAnnounceSendInterval, and/or
 *                           timeSyncHandle->timeSyncConfig.syncOut_sync0Start, and/or
 *                           timeSyncHandle->timeSyncConfig.syncOut_sync0PWidth.
 *
 * \retval <0 value on error
 * \retval  0 on success
 *
 */
int32_t TimeSync_config(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \brief Reads and provides timestamp of a transmitted event message.
 *
 * \param[in] timeSyncParamsHandle Structure of type TimeSync_ParamsHandle_t
 * \param[in] txFrameType          (INPUT) frame type transmitted Valid values: 0,1,2,3                                                        \n
 * \param[in] txPort               (INPUT) Port used for transmit ICSS_EMAC_PORT_1/ ICSS_EMAC_PORT_2                                           \n
 * \param[out] nanoseconds          (OUTPUT) ptr to 32 bit nanoseconds field
 * \param[out] seconds              (OUTPUT)  ptr to 64 bit(48 used) seconds field
 *
 * \retval <0 value on error
 * \retval  0 on success
 *
 */
int32_t TimeSync_getTxTimestamp(TimeSync_ParamsHandle_t timeSyncParamsHandle, \
                                ptpFrameTypes_t txFrameType, uint8_t txPort, \
                                uint32_t *nanoseconds, uint64_t *seconds);


/**
 * \brief Adjusts the IEP clock by the drift value over one Sync interval.
 *
 * \param[in] timeSyncParamsHandle Structure of type TimeSync_ParamsHandle_t
 * \param[in] adjOffset (filtered offset from master in nanoseconds)
 *
 * \retval <0 value on error
 * \retval  0 on success
 *
 * Adjustment Offset  (INPUT) Set drift value Example: 300 (300ns)                                                                    \n
 * syncInterval       (INPUT) Set sync interval Example: 30000000 (30ms)
 *
 * Writes the IEP compensation registers to adjust the IEP counter by adjustment offset value provided in nanoseconds over one Sync interval. syncInterval/drift value will be written to PRUSS_IEP_SLOW_COMPENSATION register. Compensation increment (10 or 0) will be written to CMP_INC field of PRUSS_IEP_GLOBAL_CFG register.
 *
 *
 */
int8_t TimeSync_adjTimeSlowComp(TimeSync_ParamsHandle_t timeSyncParamsHandle,
                                int32_t adjOffset);


/**
 * \brief Configures the driver memory
 *
 * \param[in] timeSyncParamsHandle Structure of type TimeSync_ParamsHandle_t
 *
 * \retval <0 value on error
 * \retval  0 on success
 *
 * clockTime    (INPUT) Clock time to set.  Provide in Timestamp format
 */
int32_t TimeSync_setClockTime(TimeSync_ParamsHandle_t timeSyncParamsHandle);

/**
 *  \brief Returns the current time on device
 *
 *          Returns the current time in
 *          1. Seconds : Seconds as on master
 *          2. Nanoseconds : Current nanoseconds as on master
 *          The format is identical to the one followed in PTP frames for origin timestamp
 *          The time is returned in timeSyncParamsHandle->clockTime
 *
 *  \param[in] timeSyncParamsHandle Pointer to PTP Handle
 *  \param[out] nanoseconds Pointer to nanoseconds field which is updated by API
 *  \param[out] seconds Pointer to seconds field which is updated by API
 *
 *
 */
void TimeSync_getCurrentTime(TimeSync_ParamsHandle_t timeSyncParamsHandle,
                             uint32_t *nanoseconds, uint64_t *seconds);

/**
 * \brief Update PTP domain number in TX Buffers
 * \param timeSyncHandle pointer to PTP Handle structure with updated value in timeSyncHandle->timeSyncConfig.domainNumber
 */
void TimeSync_updateDomainNumberInPTPFrames(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \brief Update log message interval in PTP TX buffers
 * \param timeSyncHandle pointer to PTP Handle structure with updated values in timeSyncHandle->timeSyncConfig.logSyncInterval and timeSyncHandle->timeSyncConfig.logAnnounceSendInterval
 */
void TimeSync_updateLogMessageIntervalInPTPFrames(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \brief Update DSCP value in IP header in PTP Delay Request buffer
 * \param timeSyncHandle    Pointer to PTP Handle
 * \param dscpValue         New 6-bit DSCP value to be set
 */
void TimeSync_updateDscpValueInDelayRequestFrame(TimeSync_ParamsHandle_t timeSyncHandle,
                                                 uint8_t dscpValue);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ICSS_TIMESYNC_API_H_ */

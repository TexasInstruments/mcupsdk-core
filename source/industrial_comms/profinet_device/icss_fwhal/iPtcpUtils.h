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

#ifndef IPTCPUTILS_H_
#define IPTCPUTILS_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "PN_Handle.h"
#include <networking/icss_emac/icss_emac.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \addtogroup PN_PTCP
 @{ */
/**
 * \internal
 * \def PRU_IEP_COUNT_REG
 *      IEP count register offset
 */
#define PRU_IEP_COUNT_REG               0x0C

/**
 * \internal
 * \def PRU_IEP_CMP_CFG_REG
 *      IEP cmp cfg register offset
 */
#define PRU_IEP_CMP_CFG_REG             0x40

/**
 * \internal
 * \def PRU_IEP_CMP1_REG
 *      IEP cmp1 register offset
 */
#define PRU_IEP_CMP1_REG                0x4C

/**
 * \internal
 * \def PRU_IEP_SYNC_PWIDTH_REG
 *      IEP sync pwidth register offset
 */
#define PRU_IEP_SYNC_PWIDTH_REG         0x110

/**
 * \internal
 * \def PRU_IEP_CAP_CFG_REG
 *      IEP cap cfg register offset
 */
#define PRU_IEP_CAP_CFG_REG             0x10


/**
 * \internal
 * \def PTCP_DEBUG_SI
 *      Macro for debugging sync for short interval
 */
/*#define   PTCP_DEBUG_SI*/

/**
 * \internal
 * \def PTCP_DEBUG_LI
 *      Macro for debugging sync for long interval
 */
/*#define     PTCP_DEBUG_LI*/

/**
 * \internal
 * \def PTCP_DEBUG
 *      Macro for ptcp debugging
 */
/*#define     PTCP_DEBUG*/

/**
 * \internal
 * \def SYNC_ANALYSIS
 *      Macro for analysis of sync during multiple resets
 */
/*#define   SYNC_ANALYSIS*/

/**
 * \internal
 * \def SYNC_SYS_LOG
 *      Macro for enabling sys log related to sync for debugging
 */
/*#define   SYNC_SYS_LOG*/

/**
 * \internal
 * \def SYNC_SYS_LOG_1
 *      Macro for enabling detailed sys log related to sync for debugging
 */
/*#define   SYNC_SYS_LOG_1*/

/**
 * \internal
 * \def PTCP_DIRECT_IEP_ADJ
 *      Macro for enabling direct IEP adjustment instead of chain adjustment
 */
/*#define   PTCP_DIRECT_IEP_ADJ*/

/**
 * \internal
 * \def PTCP_ENABLE_FILTER
 *      Enables sync filtering (SMA)
 */
#define PTCP_ENABLE_FILTER

/**
 * \internal
 * \def ISR_PTCP_NUM
 *      PTCP Sync interrupt number
 */
#define ISR_PTCP_NUM            24

/**
 * \internal
 * \def ISR_PTCP_NUM
 *      PTCP Sync interrupt number
 */
#define ISR_PTCP_NUM_ARM            124+32

/**
 * \internal
 * \def NUM_DELAYS_SMA
 *      Number of iterations for cable delay SMA calculation
 */
#define NUM_DELAYS_SMA          7

/**
 * \internal
 * \def NUM_DELAYS_BURST
 *      Number of delay req/resp in single burst of cable delay measurement process
 */
#define NUM_DELAYS_BURST        5

/**
 * \internal
 * \def PTCP_NUM_PORTS
 *      Number of physical ports
 */
#define PTCP_NUM_PORTS              2

/**
 * \internal
 * \def MAX_SEQID
 *      Maximum value of the sequence id of delay request/response frames
 */
#define MAX_SEQID               61435

/**
 * \internal
 * \def PTCP_DELAY_REQ_LEN
 *      Delay request frame length
 */
#define PTCP_DELAY_REQ_LEN      60

/**
 * \internal
 * \def MAX_CTR_VAL
 *      Max value of the counter(IEP wrap-around), which is used in cable delay calculation
 */
#define MAX_CTR_VAL             512

/**
 * \internal
 * \def INTER_DEL_REQ_GAP
 *      Interval between delay request frames from device (200 ms in ns)
 */
#define INTER_DEL_REQ_GAP       200000000
/**
 * \internal
 * \def INTER_DEL_REQ_GAP_MS
 *      Interval between delay request frames from device (205 ms)
 *      Allowed range 199ms to 301ms. SPIRTA measures 195ms if set to 200.
 */
#define INTER_DEL_REQ_GAP_MS    205

/**
 * \internal
 * \def INTER_DEL_REQ_BURST_GAP_MS
 *      Interval between delay request bursts from device (8205 ms)
 *
 */
#define INTER_DEL_REQ_BURST_GAP_MS  8205

/**
 * \internal
 * \def SYNC_INTERVAL
 *      Interval between sync frames (30 ms in ns)
 */
#define SYNC_INTERVAL           30000000

/**
 * \internal
 * \def PORT_RX_DELAY
 *      Port delay on RX side
 */
#ifdef PROFINET_RGMII_MODE
#define PORT_RX_DELAY           374+160 //FIXME : MAC delay needs to be accounted correctly
#else
#define PORT_RX_DELAY           220
#endif

/**
 * \internal
 * \def PORT_TX_DELAY
 *      Port delay on TX side
 */
#ifdef PROFINET_RGMII_MODE
#define PORT_TX_DELAY           248+160 //FIXME : MAC delay needs to be accounted correctly
#else
#define PORT_TX_DELAY           64
#endif


/**
 * \internal
 * \def DEBUG_DELAY_N_ITER
 *      Number of cable delay iterations for cable delay debug
 */
#define DEBUG_DELAY_N_ITER      200

/**
 * \internal
 * \def DEBUG_SYNC_N_ITER
 *      Number of sync iterations for sync debug
 */
#ifdef PTCP_DEBUG_SI
#define DEBUG_SYNC_N_ITER       300
#else
#define DEBUG_SYNC_N_ITER       40000
#endif

/**
 * \internal
 * \def SYNC_FILTER_SIZE
 *      depth of sync filter for SMA
 */
#define SYNC_FILTER_SIZE        8

/**
 * \internal
 * \def SYNC_ANALYSIS_N_ITER
 *      number of iterations per reset for sync analysis
 */
#define SYNC_ANALYSIS_N_ITER    100

/**
 * \internal
 * \def SYNC_ANALYSIS_N_RESETS
 *      number of resets for sync analysis
 */
#define SYNC_ANALYSIS_N_RESETS  40

#define LATCH0_EVENT                12/* Latch Event number */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \internal
 * \brief Structure containing the various parsed fields of incoming delay response
 */
typedef struct
{
    uint16_t    seqId;
    uint32_t    T1PortTXDelay;
    uint32_t    T2PortRXDelay;
    uint32_t    T3PortTXDelay;
    uint32_t    T4PortRXDelay;
    uint32_t    T1TimeStamp;
    uint32_t    T2TimeStamp;
    uint32_t    T4TimeStamp;
    uint32_t    T1_cycle_ctr;
    uint32_t    T4_cycle_ctr;
    uint32_t    reqDelay;
    uint32_t    resDelay;
    float           rcf_peer;
    uint32_t    resDelay_peer;
    uint32_t    cable_delay;
    uint32_t    line_delay;
} ptcp_iDelayResp_struct_t;

/**
 * \internal
 * \brief Structure containing current sync status information and parameters of the device
 */
typedef struct
{
    syncState_t syncState;
    /*TODO: Review NUM_PORTS -> PTCP_NUM_PORTS at all places*/
    uint32_t    cDelay[PTCP_NUM_PORTS];
    ptcpPortStatus_t    cDelayEnable[PTCP_NUM_PORTS];

    uint32_t    syncPllWnd;
    uint32_t    syncTimeoutFactor;
    uint32_t    takeoverTimeoutFactor;

    uint8_t firstSyncRcv;
    uint8_t syncRcv;
    uint32_t    nSyncMissed;

    uint8_t subdomainUUID[16];
    uint8_t masterSA[6];
} currentPtcpStatus_t;

/**
 * \internal
 * \brief Structure containing cable delay and line delay calculated during multiple iterations
 */
typedef struct
{
    uint32_t  lDelays[NUM_DELAYS_SMA];
    uint32_t    cDelays[NUM_DELAYS_SMA];
} deviceDelays_t;

/**
 * \internal
 * \brief Structure containing parmaters related to cable delay calculation
 */
typedef struct
{
    uint32_t    *pT1TS;
    uint32_t    *pT4TS;
    uint32_t  *pT1CycleCtr;
    uint32_t    *pT4CycleCtr;

    uint8_t *pDelayReqPacket;
    uint8_t *pDelayResPacket;
    uint16_t    *pSeqIdInDelayPacket;

    uint8_t     *pInDelayResPacket;
    uint8_t     *pInDelayFupResPacket;
    uint8_t     *pInDelayResCtrl;

    uint32_t    *pSmaLineDelay;
} devicePortOffsets_t;

/**
 * \internal
 * \brief Structure which records cable delay related parameters for debugging
 */
typedef struct
{
    int32_t     cDelay[DEBUG_DELAY_N_ITER];
    uint32_t    lDelay[DEBUG_DELAY_N_ITER];
    float   rcfPeer[DEBUG_DELAY_N_ITER];
    uint32_t    reqDelay[DEBUG_DELAY_N_ITER];
    uint32_t    resDelay[DEBUG_DELAY_N_ITER];
    uint16_t    seqId[DEBUG_DELAY_N_ITER];
} debugDelay_t;

/**
 * \internal
 * \brief Structure which records sync related parameters for debugging
 */
typedef struct
{
    int32_t     syncDeltaT[DEBUG_SYNC_N_ITER];
    int32_t     syncSmaDeltaT[DEBUG_SYNC_N_ITER];

    uint32_t    syncLocalT[DEBUG_SYNC_N_ITER];
    uint32_t    syncMasterT[DEBUG_SYNC_N_ITER];

    uint32_t    syncTorgT[DEBUG_SYNC_N_ITER];
    uint32_t    syncInDelayPlusLDT[DEBUG_SYNC_N_ITER];
} debugSync_t;


#define SYNC_SINGLE_ITER_SIZE 200
/**
 * \internal
 * \brief Structure for analysis/debug of Sync during multiple resets
 */
typedef struct
{
    int32_t     deltaT[SYNC_SINGLE_ITER_SIZE];
    int32_t     smaDeltaT[SYNC_SINGLE_ITER_SIZE];
    int32_t     seqId[SYNC_SINGLE_ITER_SIZE];
    int32_t     tState[SYNC_SINGLE_ITER_SIZE];
    int32_t     tSeqId[SYNC_SINGLE_ITER_SIZE];
    int32_t     cycleTime[SYNC_SINGLE_ITER_SIZE];
} syncAnalysis_t;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief Triggers the start of ptcp protocol
 *  Should be called from stack after initialization
 *  It only controls ARM side tasks of PTCP, not PRU
 *  \param pnHandle Profinet Handle
 */
void PN_PTCP_start(PN_Handle pnHandle);

/**
 * \brief Handles the clock (IEP CMP0) change scenario
 *
 * \param[in] pnHandle Profinet Handle
 * \param cycleTime new cycle time with which IEP CMP0 is programmed
 */
void PN_PTCP_ClockChange(PN_Handle pnHandle, uint32_t cycleTime);

/**
 * \brief reset relevant data on start or sync timeout
 * \param[in] pnHandle Profinet Handle
 */
void PN_PTCP_reset(PN_Handle pnHandle);

/**
 * \brief Initializes the PTCP data structure and memory for PTCP pre-formatted packets
 * \param[in] pnHandle Profinet Handle
 */
void PN_PTCP_init(PN_Handle pnHandle);

/**
 * \internal
 * \brief Controls the single burst of delay measurement process
 * \param[in] icssEmacHandle ICSS Emac LLD Handle
 */
void PN_PTCP_delayMeasurement(PN_Handle pnHandle);

/**
 * \internal
 * \brief Resets the parameters related to cable delay
 * \param pnHandle Profinet handle
 */
void PN_PTCP_resetDelayValues(PN_Handle pnHandle);

/**
 * \internal
 * \brief Controls the complete cable delay measurement process
 * \param[in] pnHandle Profinet Handle
 */
void PN_PTCP_smaDelayMeasurement(PN_Handle pnHandle);

/**
 * \internal
 * \brief Calculates the requestor side delay, for line/cable delay calculation
 *
 * \param pnHandle Profinet Handle
 * \param ptcp_port_desc Reference to structure containing various parsed fields of incoming delay response
 */
int32_t PN_PTCP_lineDelayCalc(PN_Handle pnHandle,
                              ptcp_iDelayResp_struct_t *ptcp_iDelayResp_parsed);

/**
 * \internal
 * \brief Parses the incoming delay response packet
 *
 * \param ptcp_iDelayResp_parsed Reference to structure containing various parsed fields of incoming delay response
 * \param ptcp_iDelayResp_packet Reference to the incoming delay response packet
 * \param ptcp_iDelayFupResp_packet Reference to the incoming delay response FUP packet
 * \param w_FUP                  Indicates whether there is any FUP or not
 *
 * \retval 0 on success, <0 on failure
 */
int32_t PN_PTCP_parseInDelayResp(ptcp_iDelayResp_struct_t
                                 *ptcp_iDelayResp_parsed, uint8_t *ptcp_iDelayResp_packet,
                                 uint8_t *ptcp_iDelayFupResp_packet, int32_t w_FUP);

/**
 * \internal
 * \brief Calculates rcf_peer, cable delay and line delay
 * \param pnHandle Profinet Handle
 * \param ptcp_iDelayResp_parsed Reference to structure containing various parsed fields of incoming delay response
 * \param port                   Port on which delay response was received
 * \retval 0 on success
 */
int32_t PN_PTCP_cableDelayCalc(PN_Handle pnHandle,
                               ptcp_iDelayResp_struct_t *ptcp_iDelayResp_parsed, uint8_t port);

/**
 * \internal
 * \brief Rotates unsigned int i.e. from big endian to little endian conversion
 *
 * \param input Reference to the big endian data
 * \retval unsigned int little endian value
 */
uint32_t PN_PTCP_rotUint(uint32_t *input);

/**
 * \internal
 * \brief Rotates unsigned short i.e. from big endian to little endian conversion
 *
 * \param input Reference to the big endian data
 * \retval unsigned short little endian value
 */
uint16_t PN_PTCP_rotUshort(uint16_t *input);

/**
 * \internal
 * \brief Initial setup for PTCP Sync Interrupt
 * \param[in] pnHandle Profinet Handle
 * \retval 0 on success, -1 on failure
 */
int32_t PN_PTCP_setupIsr(PN_Handle pnHandle);

/**
 * \internal
 * \brief Initiates the sync handling process i.e. deltaT calculation and clock adjustment
 */
void PN_PTCP_isrHandler(void* arg);

/**
 * \internal
 * \brief Enables the ISR for PTCP Sync
 * \param[in] pnHandle Profinet Handle
 * \retval 0 on success
 */
int32_t PN_PTCP_enableIsr(PN_Handle pnHandle);

/**
 * \internal
 * \brief Calculates the deltaT b/w master & slave and initiates the clock adjustment process
 * \param[in] pnHandle Profinet Handle
 */
void PN_PTCP_syncHandling(PN_Handle pnHandle);

/**
 * \internal
 * \brief Adjust IEP with new compensation value and reconfigure the ECAP period
 * \param pnHandle profinet Handle
 * \param ecapPeriod    new calculated period for the ECAP
 * \param compensation  compensation value for IEP compensation register based on deltaT
 */
void PN_PTCP_syncIepAdjustment(PN_Handle pnHandle, int32_t ecapPeriod,
                               uint32_t compensation);

/**
 * \internal
 * \brief Parses the fields from 2nd block of Sync packet
 *
 * \param pnHandle Profinet Handle
 * \param ptcp_sync_parsed Reference to the sync info structure
 * \param sync_sblock         Reference to the second block of sync packet
 * \retval 0 on success
 */
int32_t PN_PTCP_parseSyncFields(PN_Handle pnHandle,
                                volatile ptcpSyncInfo_t *ptcp_sync_parsed, uint8_t *sync_sblock);


/**
 * \internal
 * \brief Preprocess the Sync related fields before Sync Handling, incase of Store-and-Forward of Sync packets
 * \param[in] pnHandle Profinet Handle
 * \param ctrlByte Indicates on which port sync packet was received
 *                0 : cut-through mode
 *                1 : Received on port#1
 *                2 : Received on port#2
 */
void PN_PTCP_syncPreprocess(PN_Handle pnHandle, uint8_t ctrlByte);

/**
 * \internal
 * \param pnHandle Profinet Handle
 * \brief Calculates the SMA cable and line delay
 *
 * \param portNum port no. for which calculation is done
 */
void PN_PTCP_portDelaySmaCalc(PN_Handle pnHandle, uint8_t portNum);

/**
 * \internal
 * \brief Process the incoming delay response and calculates the cable delay
 *
 * \param pnHandle Profinet Handle
 * \param portNum port no. on which delay delay response was received
 */
void PN_PTCP_processDelayResponse(PN_Handle pnHandle, uint8_t portNum);

/**
 * \internal
 * \brief Resets the timing values before sending delay request
 *
 * \param pnHandle Profinet Handle
 * \param portNum port no. for which timing value will be reset
 */
void PN_PTCP_resetDelayTimings(PN_Handle pnHandle, uint8_t portNum);

/**
 * \internal
 * \brief Adjusts ctrDiff based on cycle period
 *
 * \param pnHandle Profinet Handle
 * \param ctrDiff the value of the counter difference without adjustment
 * \retval adjusted ctrDiff based on cycle period
 */
int32_t PN_PTCP_adjCtrDiff(PN_Handle pnHandle, int32_t ctrDiff);

/**
 * \internal
 * \brief Absolute value of an integer
 *
 * \param num integer input
 * \retval absolute value of the given number
 */
int32_t PN_PTCP_absVal(int32_t num);

/**
 * \internal
 * \brief Monitors the sync timeout event
 * \param[in] pnHandle Profinet Handle
 */
void PN_PTCP_syncTimeoutMonitor(PN_Handle pnHandle);

/**
 * \internal
 * \brief Task goes for sleep for the specified time
 *
 * \param mSec: Time in mSec for which task goes to sleep
 */
void PN_PTCP_taskSleep(uint32_t mSec);

/**
 * \internal
 * \brief Configures the sync0 Pin output
 * \param[in] pnHandle Profinet Handle
 */
void PN_PTCP_configureSync0Pin(PN_Handle pnHandle);

/**
 *  \internal
 * \brief Provides modulo function which can handle neagative numbers as well
 *
 * \param num: dividend
 * \param mod : divisor
 * \retval : result of num%mod
 */
int32_t PN_PTCP_modFunc(int32_t num, uint32_t mod);

/**
 *  \internal
 *  \brief Triggers the delay measurement
 *  Should be called when link change the status
 *  \param pnHandle Profinet Handle
 */
void PN_PTCP_triggerMeasurement(PN_Handle pnHandle);
/**
@}
*/


#ifdef __cplusplus
}
#endif

#endif  /*PTCP_UTILS*/

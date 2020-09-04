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
 * \file timeSync.h
 *
 * \brief This file contains the interface to IP abstracted functions
 *        for PTP stack
 */

/**
 * \defgroup DRV_TIMESYNC_MODULE TimeSync Driver
 *
 * The TimeSync driver provides time synchronization support to
 * CPSW and ICSS using HAL and PTP stack.
 *
 *  @{
 */

/*!
 * \defgroup TIMESYNC_HAL_API TIMESYNC HAL API
 */

/*!
 * \defgroup TIMESYNC_PTP_API TIMESYNC PTP API
 */

/* @} */

/*!
 * \addtogroup TIMESYNC_HAL_API
 * @{
 */

#ifndef TIMESYNC_H_
#define TIMESYNC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>

#include "enet_mod_timesync.h"

/* ========================================================================== */
/*                                Error Codes                                 */
/* ========================================================================== */

/**
 *  @brief Success
 */
#define TIMESYNC_OK                                                    (0)

/**
 * @brief Time Sync feature not supported
 */
#define TIMESYNC_UNABLE_TO_INIT_HAL                                    (-1)

/**
 * @brief Time Sync unsupported format
 */
#define TIMESYNC_UNSUPPORTED_FORMAT                                    (-9)

/**
 * @brief Time Sync handle uninitialized
 */
#define TIMESYNC_HANDLE_NOT_INITIALIZED                               (-12)

/**
 * @brief Time Sync invalid parameter
 */
#define TIMESYNC_PARAM_INVALID                                         (-17)

/**
 * @brief Time Sync frame not available
 */
#define TIMESYNC_FRAME_NOT_AVAILABLE                                   (-19)

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * @def TIMESYNC_NUM_DOMAINS
 *      Number of domains supported by this PTP implementation
 */
#define TIMESYNC_NUM_DOMAINS                                           (2U)

/** Value of seconds in nanoseconds. Useful for calculations*/
#define TIME_SEC_TO_NS                           (1000000000U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 * @brief Transmit callback function
 * @param[in] cbArg     Pointer to callback argument
 * @param[in] portNum   Port number to which frame was transmitted
 * @param[in] frameType PTP frame type of transmitted frame
 * @param[in] seqId     PTP sequence ID of transmitted frame
 * @return None
 */
typedef void (*TimeSync_txNotifyCb)(void *cbArg,
                                    uint8_t portNum,
                                    uint8_t frameType,
                                    uint16_t seqId);

/**
 * @brief Receive callback function
 * @param[in] cbArg     Pointer to callback argument
 * @return None
 */
typedef void (*TimeSync_rxNotifyCb)(void *cbArg);

/**
 * @brief PTP Device configuration
 */
typedef enum
{
    /** Master Clock */
    TIMESYNC_MASTER_CLOCK      = 1U,

    /** Only Transparent Clock */
    TIMESYNC_TRANSPARENT_CLOCK = 2U,

    /** Only Ordinary Clock */
    TIMESYNC_ORDINARY_CLOCK    = 3U,

    /** Both Ordinary and Transparent clock */
    TIMESYNC_OC_AND_TC         = 4U,

    /** Boundary clock */
    TIMESYNC_BOUNDARY_CLOCK    = 5U,
} TimeSync_DeviceConfig;

/**
 * @brief Network Protocol Type
 */
typedef enum
{
    /** IPv4 */
    TIMESYNC_PROT_UDP_IPV4    = 1U,

    /** IPv6 */
    TIMESYNC_PROT_UDP_IPV6    = 2U,

    /** Ethernet without TCP/IP */
    TIMESYNC_PROT_IEEE_802_3  = 3U,

    /** Device Net */
    TIMESYNC_PROT_DEVICE_NET  = 4U,

    /** Control Net */
    TIMESYNC_PROT_CONTROL_NET = 5U,

    /** PROFINET */
    TIMESYNC_PROT_PROFINET    = 6U,

    /** Unknown protocol type */
    TIMESYNC_PROT_UNKNOWN     = 0xFFFEU
} TimeSync_NetworkProtType;

/*!
 *  @brief Enum of VLAN usage type
 */
typedef enum
{
    /** No VLAN tag */
    TIMESYNC_VLAN_TYPE_NONE       = 0x00U,

    /** Single VLAN tag */
    TIMESYNC_VLAN_TYPE_SINGLE_TAG = 0x01U,

    /** Double VLAN tag */
    TIMESYNC_VLAN_TYPE_DOUBLE_TAG = 0x02U,
} TimeSync_VlanType;

/**
 * @brief Enum of PTP step modes
 */
typedef enum
{
    /** PTP in single step mode */
    TIMESYNC_SINGLE_STEP = 1U,

    /** PTP in double step mode */
    TIMESYNC_DOUBLE_STEP = 2U,
} TimeSync_StepMode;

/**
 * @brief TimeSync VLAN configuration structure
 */
typedef struct
{
    /** VLAN type */
    TimeSync_VlanType vlanType;

    /** Inner VLAN tag */
    uint32_t iVlanTag;

    /** Outer VLAN tag */
    uint32_t oVlanTag;
} TimeSync_VlanConfig;

/**
 * @brief TimeSync protocol configuration structure
 */
typedef struct
{
    /** Packet type in which PTP is encapsulated*/
    TimeSync_NetworkProtType protocol;

    /** Device's PTP mode of operation*/
    TimeSync_DeviceConfig deviceCfg;

    /** List of 802.1AS-rev domainNumbers*/
    uint8_t domainNumber[TIMESYNC_NUM_DOMAINS];

    /** Clock Identity = MAC ID + Protocol Identifier*/
    uint8_t clockIdentity[8];

    /** PTP step mode */
    TimeSync_StepMode stepMode;

    /** VLAN comfiguration*/
    TimeSync_VlanConfig vlanCfg;

    /** Port mask indicating ports to be used for PTP*/
    uint8_t portMask;
}TimeSync_ProtocolConfig;

/**
 * @brief TimeSync frame notify configuration structure
 */
typedef struct
{
    // Network driver handle 
    void *nwDrvHandle;

    //Tx callback function 
    TimeSync_txNotifyCb txNotifyCb;

    // Tx callback argument 
    void *txNotifyCbArg;

    // Rx callback function 
    TimeSync_rxNotifyCb rxNotifyCb;

    // Rx callback argument 
    void *rxNotifyCbArg;
} TimeSync_FrameNotifyConfig;


/**
 * @brief Time synchronization configuration structure
 */
typedef struct
{
    /** TimeSync protocol configuration */
    TimeSync_ProtocolConfig protoCfg;

    /** TimeSync frame notification configuration */
    TimeSync_FrameNotifyConfig frameNotifyCfg;
} TimeSync_Config;

/**
 * @brief EnetApp handle
 *
 * EnetApp opaque handle
 */
typedef struct EnetApp_PerCtxt_s EnetApp_PerCtxt;
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Function to configure low-level network driver
 *
 * @param[in] timeSyncConfig Pointer to time sync configuration structure
 * @return    timeSyncHandle Handle to TimeSync HAL object
 */
EnetApp_PerCtxt* EnetApp_TimeSyncOpen(TimeSync_Config *timeSyncConfig);

/**
 * @brief Reads and provides timestamp of a received event message.
 *
 * @param[in]  timeSyncHandle      Pointer to TimeSync object
 * @param[in]  rxFrameType         Frame type of received packet
 * @param[in]  rxPort              Port used for receiving packet
 * @param[in]  seqId               Sequence ID of the packet
 * @param[out] nanoseconds         Pointer to 32 bit nanoseconds field
 * @param[out] seconds             Pointer to 64 bit(48 used) seconds field
 * @return     TimeSync error code
 */
int32_t EnetApp_getRxTimestamp(EnetApp_PerCtxt *perCtxt,
                                EnetTimeSync_MsgType rxFrameType,
                                uint8_t rxPort,
                                uint16_t seqId,
                                uint32_t *nanoseconds,
                                uint64_t *seconds);

/**
 * @brief Reads and provides timestamp of a transmitted event message.
 *
 * @param[in]  timeSyncHandle      Pointer to TimeSync object
 * @param[in]  txFrameType         Frame type transmitted packet
 * @param[in]  txPort              Port used for transmit
 * @param[in]  seqId               Sequence ID of the packet
 * @param[out] nanoseconds         Pointer to 32 bit nanoseconds field
 * @param[out] seconds             Pointer to 64 bit(48 used) seconds field
 * @return     TimeSync error code
 *
 */
 
int32_t EnetApp_getTxTimestamp(EnetApp_PerCtxt *perCtxt,
                                EnetTimeSync_MsgType txFrameType,
                                uint8_t txPort,
                                uint16_t seqId,
                                uint32_t *nanoseconds,
                                uint64_t *seconds);

/**
 * @brief Adjusts the clock by the drift value over one Sync interval.
 *
 * @param[in] timeSyncHandle Pointer to TimeSync object
 * @param[in] adjOffset      Filtered offset from master in nanoseconds
 * @param[in] syncInterval   Sync interval
 * @return    TimeSync error code
 */
int32_t EnetApp_adjTimeSlowComp(EnetApp_PerCtxt* enet_perctxt,
                                 int32_t adjOffset,
                                 uint64_t syncInterval);

/**
 * @brief Changes the timer value
 *
 * @param[in] timeSyncHandle      Pointer to TimeSync object
 * @param[in] nanoseconds         32 bit nanoseconds value
 * @param[in] seconds             64 bit seconds value
 * @return    TimeSync error code
 */
int32_t EnetApp_setClockTime(EnetApp_PerCtxt* enet_perctxt,
                              uint32_t nanoseconds,
                              uint64_t seconds);

/**
 *  @brief Returns the current time on device.
 *
 * Returns the current time in
 * 1. Seconds : Seconds as on master.
 * 2. Nanoseconds : Current nanoseconds as on master.
 * The format is identical to the one followed in PTP frames for origin timestamp.
 *
 *  @param[in]  timeSyncHandle Pointer to TimeSync object
 *  @param[out] nanoseconds    Pointer to nanoseconds field
 *  @param[out] seconds        Pointer to seconds field
 *  @return     None
 */
void EnetApp_getCurrentTime(EnetApp_PerCtxt *perCtxt,
                             uint32_t *nanoseconds,
                             uint64_t *seconds);

/**
 *  @brief Receive the PTP frame from hardware queue
 *
 *  @param[in]  timeSyncHandle Pointer to TimeSync object
 *  @param[out] frame          Pointer to PTP frame buffer
 *  @param[out] size           Size of received frame
 *  @param[out] rxPort         Port number frame was received
 *  @return     TimeSync error code
 */
int32_t EnetApp_getPtpFrame(EnetApp_PerCtxt* enet_perctxt,
                             uint8_t *frame,
                             uint32_t *size,
                             uint8_t *rxPort);

/**
 *  @brief Transmit the PTP frame to the directed port
 *
 *  @param[in] timeSyncHandle Pointer to TimeSync object
 *  @param[in] frame          Pointer to PTP frame buffer
 *  @param[in] size           Size of received frame
 *  @param[in] txPort         Port number to which frame is to be sent
 *  @return    TimeSync error code
 */
int32_t EnetApp_sendPtpFrame(EnetApp_PerCtxt* enet_perctxt,
                              uint8_t *frame,
                              uint32_t size,
                              uint8_t txPort);

/**
 * @brief Time sync port link status helper function.
 *
 * @param[in]  timeSyncHandle Pointer to TimeSync object
 * @param[in]  portNum        Port number
 * @return     TRUE -  if link is up
 *             FALSE - if link is down.
 */
int8_t EnetApp_isPortLinkUp(EnetApp_PerCtxt* enet_perctxt,
                             uint8_t portNum);

/**
 * @brief Time sync module reset
 *
 * @param[in] timeSyncHandle Pointer to TimeSync object
 * @return None
 */
void EnetApp_reset(EnetApp_PerCtxt* enet_perctxt);

/**
 * @brief Time sync module close
 *
 * @param[in] timeSyncHandle Pointer to TimeSync object
 * @return None
 */
void TimeSync_close(EnetApp_PerCtxt* enet_perctxt);
#ifdef __cplusplus
}
#endif

#endif /* TIMESYNC_H_ */

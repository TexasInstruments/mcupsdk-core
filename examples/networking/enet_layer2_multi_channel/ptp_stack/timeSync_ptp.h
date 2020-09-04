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
 * \file timeSync_ptp.h
 *
 * \brief This file contains the interface to PTP stack functions.
 *
 */

/*!
 * \addtogroup TIMESYNC_PTP_API
 * @{
 */

#ifndef TIMESYNC_PTP_H_
#define TIMESYNC_PTP_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdlib.h>
#include "timeSync.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * @def TIMESYNC_PTP_MAX_PORTS_SUPPORTED
 *      Number of ports supported by this PTP
 */
#define TIMESYNC_PTP_MAX_PORTS_SUPPORTED                    (8U)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 * @brief PTP Delay Mechanism Type. From Table 9 of Standard
 */
typedef enum
{
    /** End to end delay mechanism*/
    TIMESYNC_PTP_DELAY_E2E = 1U,

    /** Peer to peer delay mechanism*/
    TIMESYNC_PTP_DELAY_P2P = 2U,
} TimeSyncPtp_DelayType;

/**
 * @brief 802.1AS-rev Enumeration8
 */
typedef enum
{
    /** Atomic clock */
    TIMESYNC_CLKSRC_ATOMIC_CLOCK        = 0x10U,

    /** GPS clock */
    TIMESYNC_CLKSRC_GPS                 = 0x20U,

    /** Terrestrial radio clock */
    TIMESYNC_CLKSRC_TERRESTRIAL_RADIO   = 0x30U,

    /** PTP clock */
    TIMESYNC_CLKSRC_PTP                 = 0x40U,

    /** NTP clock */
    TIMESYNC_CLKSRC_NTP                 = 0x50U,

    /** Handset clock */
    TIMESYNC_CLKSRC_HAND_SET            = 0x60U,

    /** Other clock */
    TIMESYNC_CLKSRC_OTHER               = 0x90U,

    /** Internal oscillator clock */
    TIMESYNC_CLKSRC_INTERNAL_OSCILLATOR = 0xA0U,
} TimeSyncPtp_TimeSource;

/**
 * @brief 802.1AS-rev Timestamp structure
 */
typedef struct
{
    /** 48 bit seconds field */
    uint64_t seconds;

    /** 32 bit nanoseconds field */
    uint32_t nanoseconds;
} TimeSyncPtp_TimeStamp;

/**
 * @brief Master and grand master clock params
 */
typedef struct
{
    /** Priority 1 field in standard. Refer section 7.6.2.2 */
    uint8_t priority1;
    /** Priority 2 field in standard. Refer section 7.6.2.3 */
    uint8_t priority2;
    /** ClockClass field in standard. Refer section 7.6.2.4 */
    uint8_t clockClass;
    /** ClockAccuracy field in standard. Refer section 7.6.2.5 */
    uint8_t clockAccuracy;
    /** TimeSource field in standard. Refer section 7.6.2.6 */
    uint8_t timeSource;
    /** Applicable to Boundary clocks. Refer section 8.2.2.2 */
    uint16_t stepRemoved;
    /** ClockVariance field in standard. Refer section 7.6.3 */
    uint16_t clockVariance;
    /** Current UTC offset */
    uint16_t UTCOffset;
    /** Grand master identity */
    uint8_t gmIdentity[8];

    /** PTP Flag params. See standard
     *  index 0 - PTP Leap second 61
     *  index 1 - PTP Leap second 59
     *  ....
     *  ....
     *  index 11 - PTP Security */
    uint8_t ptpFlags[12];
} TimeSyncPtp_MasterParams;

/**
 * @brief TimeSync PTP sync loss callback function.
 */
typedef void (*TimeSyncPtp_SyncLossNotify)(void);

/**
 * @brief Time synchronization PTP configuration structure
 */
typedef struct
{
    /** Network driver handle */
    void *nwDrvHandle;

    /** Whether clock is OC or TC or both */
    TimeSync_DeviceConfig deviceMode;

    /** Which type of Measurement scheme. E2E or P2P */
    TimeSyncPtp_DelayType type;

    /** Packet type in which PTP is encapsulated */
    TimeSync_NetworkProtType protocol;

    /** VLAN comfiguration*/
    TimeSync_VlanConfig vlanCfg;

    /** Set to 1 if device is master, else set to 0 */
    uint8_t isMaster;

    /** Grand master parameters like priority 1, priority 2 etc. Populated in Announce frame */
    TimeSyncPtp_MasterParams masterParams;

    /** Port mask consisting information about ports to be used for PTP
     *  Example: To use macports  0, 2 ,4 set portMask = 0b00040101 */
    uint8_t portMask;

    /** Callback function to notify app about sync loss */
    TimeSyncPtp_SyncLossNotify syncLossNotifyFxn;

    /** Delay asymmetry correction value for ports */
    uint16_t asymmetryCorrection[TIMESYNC_PTP_MAX_PORTS_SUPPORTED];

    /** List of 802.1AS-rev domainNumbers */
    uint8_t domainNumber[TIMESYNC_NUM_DOMAINS];

    /** All PTP packet offsets are w.r.t PTP Layer 3 packets
     *  an offset variable is hence used to calculate actual offset
     *  based on whether the configuration is E2E + L3 or P2P + L2
     */
    uint8_t frameOffset;

    /* BIOS tick period (in microseconds). Normalized value is 1000 means factor of 1, 500 means
     * BIOS counts twice as fast so divide time period by 2*/
    uint32_t tickPeriod;

    /** PDelay Request interval. Actual value is 2^value seconds */
    int8_t logPDelReqPktInterval;

    /** Sync send interval. Actual value is 2^value seconds */
    int8_t logSyncInterval;

    /** Announce send interval. Actual value is 2^value seconds */
    int8_t logAnnounceSendInterval;

    /** Announce receipt timeout interval. Actual value is 2^value seconds */
    int8_t logAnnounceRcptTimeoutInterval;

    /** Delay Request/PDelay Request interval. Mapped here from logPDelReqPktInterval.
     *  value in milliseconds*/
    uint32_t pDelReqPktInterval;

    /** Log of sync send interval. Mapped here from logSyncInterval.
     *  value in microseconds*/
    uint32_t syncSendInterval;

    /** Log of Announce send interval. Mapped here from logAnnounceSendInterval.
     *  value in milliseconds*/
    uint32_t announceSendInterval;

    /** Log of Announce receipt timeout interval. Min value of 3.
     *  value in milliseconds*/
    uint32_t announceRcptTimeoutInterval;

    /** In a Pdelay request burst packets are sent at this interval.
     *  value in miliseconds*/
    uint32_t pdelayBurstInterval;

    /** In a Pdelay request burst these many frames are sent*/
    uint8_t pdelayBurstNumPkts;

    /** Interface MAC address, this is used in all PTP frames, and also used
     *  in clock identifier for a master device */
    uint8_t ifMacID[6];

    /** Network device's IPv4 address, this is used in all PTP frames */
    uint8_t ipAddr[4];

    /** Whether HSR is enabled or not*/
    uint8_t hsrEnabled;

    /** This field is only applicable to HSR frames.
     *  This is set to 1 if link local frames carry HSR tag.
     *  By default this is set to 0 which means no HSR tag for
     *  peer delay request/response frames.
     *  It Can be overridden from application (see HSR/PRP example).
     *  This gets over ridden in the driver once the peer sends delay request response frames.
     * */
    uint8_t ll_has_hsrTag;
} TimeSyncPtp_Config;

typedef struct TimeSyncPtp_Obj_s *TimeSyncPtp_Handle;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  @brief  Function to initialize PTP configuration with default values
 *  @param   ptpConfig Pointer to PTP configuration
 *  @return  None
 */
void TimeSyncPtp_setDefaultPtpConfig(TimeSyncPtp_Config *ptpConfig);

/**
 * @brief Initialize PTP stack
 * @param ptpConfig Pointer to PTP configuration
 * @return SUCCESS: Handle to PTP object,
 *         FAILURE: NULL
 */
TimeSyncPtp_Handle TimeSyncPtp_init(TimeSyncPtp_Config *ptpConfig);

/**
 * @brief De-initialize PTP stack
 * @param hTimeSyncPtp Pointer to PTP Handle structure
 * @return None
 */
void TimeSyncPtp_deInit(TimeSyncPtp_Handle hTimeSyncPtp);

/**
 * @brief Enable PTP firmware
 * @param hTimeSyncPtp Pointer to PTP Handle structure
 * @return None
 */
void TimeSyncPtp_enable(TimeSyncPtp_Handle hTimeSyncPtp);

/**
 * @brief Disable PTP firmware
 * @param hTimeSyncPtp pointer to PTP Handle structure
 * @return None
 */
void TimeSyncPtp_disable(TimeSyncPtp_Handle hTimeSyncPtp);

/**
 * @brief Callback for link status change on Port
 * @param arg callback argument
 * @param portNum
 * @param linkStatus 1/0 whether up or down
 *
 * @return None
 */
void TimeSyncPtp_PortLinkResetCallBack(void *arg,
                                       uint8_t portNum,
                                       uint8_t linkStatus);

/**
 *  @brief  Reset the state machine in firmware to restart synchronization (First adjustment happens again).
 *          If Sync interval changes or link break happens or there is a large adjustment in time
 *          this function is called.
 *
 *  @param  timeSyncPtpHandle pointer to PTP Handle structure
 *
 *  @retval None
 *
 */
void TimeSyncPtp_reset(TimeSyncPtp_Handle timeSyncPtpHandle);

/**
 * @brief Return TRUE/FALSE if timeSync module is enabled/disabled
 *
 * @param timeSyncPtpHandle pointer to PTP Handle structure
 *
 */
uint8_t TimeSyncPtp_isEnabled(TimeSyncPtp_Handle timeSyncPtpHandle);
#endif /* TIMESYNC_PTP_H_ */

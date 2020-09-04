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
#ifndef ICSS_EIP_DRIVER_H_
#define ICSS_EIP_DRIVER_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                          Doxygen                                           */
/* ========================================================================== */

/**
 *  \defgroup INDUSTRIAL_COMMS_ETHERNETIP_ADAPTER_FWHAL_MODULE APIs for EtherNet/IP Adapter FWHAL
 *  \ingroup INDUSTRIAL_COMMS_MODULE
 *
 *  EtherNet/IP Adapter FWHAL(Firmware and Hardware Abstraction Layer) APIs implement the key
 *  interface between EtherNet/IP firmware and slave stack.
 *
 *  @{
 */

/**
* \defgroup EIP_API EtherNet/IP FWHAL APIs
* \ingroup INDUSTRIAL_COMMS_ETHERNETIP_ADAPTER_FWHAL_MODULE
*/

/**
* \defgroup EIP_DLR DLR APIs
* \ingroup INDUSTRIAL_COMMS_ETHERNETIP_ADAPTER_FWHAL_MODULE
*/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include "icss_dlr.h"
#include <networking/icss_timesync/icss_timeSyncApi.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/*Use compare reg 4*/
#define IEP_CMP_DEFAULT_VAL      0x11
#define IEP_CMP4_DEFAULT_VAL     0x5000
#define IEP_CMP0_DEFAULT_VAL     0xffffffff

/**IEP Compare Register*/
#define PRU_IEP_CMP_CFG_REG             0x40
/*Compare 0 register*/
#define PRU_IEP_CMP0_REG                0x48

/**EIP Tick duration, used for DLR link detection as well as Half Duplex*/
#define EIP_TICK_PERIOD 1 /*in milliseconds*/
#define LOOPBK_PKT_SEND_PERIOD 1000

#define DEFAULT_BC_PKT_SIZE     60

#define ONE_SECOND_INTERVAL 1000    /*in milliseconds*/
#define TWO_MINUTE_INTERVAL  120000 /*in milliseconds*/

/**Mask for carrier sense status on MII RT for Half Duplex*/
#define CRS_STATUS_MASK                     0x2
/**Shift Val for carrier sense status on MII RT for Half Duplex*/
#define CRS_STATUS_SHIFT                    0x1

/**PTP Protocol type*/
#define PTP_PROT_TYPE       0x88F7
#define DLR_MDIO_PHY0 1
#define DLR_MDIO_PHY1 2

/*DLR port 0 Interrupt*/
#define PHYBMSR_OFFSET                      0x1
#define PHYSTS_OFFSET                       0x10

#define PHY_LINK_STATUS_MASK                0x4
#define PHY_LINK_STATUS_SHIFT               0x2

typedef struct eip_Config_s *EIP_Handle;

/**
 * \brief Type of clock supported by PTP Implementation. This is not specified by standard
 */
typedef enum
{
    /**Ordinary clock/End Node*/
    ORDINARY_CLOCK = 0,
    /**Transparent clock*/
    TRANSPARENT_CLOCK = 1,
    /**Transparent clock*/
    BOUNDARY_CLOCK = 2

} clockType;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief Specifies the properties of a clock
 */
typedef struct
{
    /**Unique identifier for Clock, See standard for exact definition*/
    uint8_t clockIdentity[8];
    /**Specifies class of clock Quality*/
    uint16_t clockClass;
    /**Accuracy in seconds. See spec for more detail*/
    uint16_t timeAccuracy;
    /**OffsetScaledLogVariance is the variance measure of clock quality used by the Best Master
    algorithm to determine the grandmaster.*/
    uint16_t offsetScaledLogVariance;
    /**CurrentUtcOffset specifies the current UTC offset in
    seconds from International Atomic Time
    (TAI) of the clock. As of 0 hours 1 January 2006 UTC,
    the offset was 33 seconds*/
    uint16_t currentUTCOffset;
    /**TimePropertyFlags specifies the time property flags of the clock.*/
    uint16_t timePropertyFlags;
    /**TimeSource specifies the primary time source of the clock*/
    uint16_t timeSource;

    /**Same as offsetScaledLogVariance. Observed value.
     * Applicable for Grandmaster and Parent*/
    uint16_t observedOffsetScaledLogVariance;

    /**specifies an estimated measure of the parent clock's drift as
        observed by the slave clock */
    uint16_t observedPhaseChangeRate;

} clockClass_t;

/**
 * \brief Description in unicode
 */
typedef struct
{
    /**number of unicode characters, capped to 32 characters*/
    uint32_t size;
    /**Stores unicode characters*/
    uint8_t descr[32];
} descr_t;

/**
 * \brief Specifies the PTP profile of each port of the device
 */
typedef struct
{
    uint8_t portProfileIdentity[8];
} portProfileIdentity_t;

/**
 * \brief Specifies the physical protocol and physical address (e.g. IEEE 802.3)
    of each port of the device (e.g. MAC address). The maximum number of characters is 16.
    Unused array elements are zero-filled
 */
typedef struct
{
    uint8_t physicalProtocol[16];
    uint16_t sizeOfAddress;
    uint8_t portPhysicalAddress[16];
} portPhysAddr_t;

/**
 * \brief Specifies the specifies the network and protocol address of each port of the device
    (e.g. IP address). The Network Protocol specifies the protocol for the network.
 */
typedef struct
{
    uint16_t portNumber;
    uint16_t networkProtocol;
    uint16_t addressSize;
    uint8_t portProtocolAddress[16];
} portProtAddr_t;

/**
 * \brief Specifies the system time in microseconds and
 *  the Offset to the local clock value
 */
typedef struct
{
    uint64_t systemTime;
    uint64_t systemOffset;
} sysTimeOffset_t;

/**
 * \brief CIP Sync configuration. Instance Attribute for
 * PTP Class (Class Code 0x43. CIP Spec Vol 1)
 */
typedef struct
{
    /**If PTP is enabled on system. Only applicable for OC*/
    uint8_t ifPTPEnable;
    /**If PTP is synchronized with Master*/
    uint8_t IsSynchronized;
    /**Current system time in Microseconds*/
    uint64_t systemTimeMicrosec;
    /**Current system time in Nanoseconds*/
    uint64_t systemTimeNanosec;
    /**Offset between local clock and master clock*/
    int64_t offsetFromMaster;
    /**Maximum offset between local clock and master clock*/
    uint64_t maxOffsetFromMaster;
    /**Mean path delay to master*/
    int64_t meanPathDelayToMaster;
    /**Grand master clock info*/
    clockClass_t grandMasterClkInfo;
    /**Parent clock info*/
    clockClass_t parentClkInfo;
    /**Local clock info*/
    clockClass_t localClkInfo;
    /**Number of PTP Ports on the device. See Spec*/
    uint16_t numberOfPorts;
    /** Status of PORT. PTP State machine values. See Spec*/
    /*TODO: Review ICSS_EMAC_MAX_PORTS_PER_INSTANCE -> ICSS_EMAC_MAX_PORTS_PER_INSTANCE*/
    port_State portState[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /**Port Enable/Disable*/
    uint16_t portEnable[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /**PTP announce interval between successive "Announce"
     messages issued by a master clock on each PTP port of the device*/
    uint16_t portLogAnnounceInterval[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /**specifies the PTP sync interval between successive "Sync"
     messages issued by a master clock on each PTP port of the device */
    uint16_t portLogSyncInterval[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

    /**attribute allows the user to override the automatic selection of the
    best master clock before any quality measures are evaluated*/
    uint8_t priority1;
    /**Same as above with a difference that it is used after clock is chosen*/
    uint8_t priority2;

    /**Domain Number as per PTP algorithm*/
    uint8_t domainNumber;
    /**Clock Type. OC, TC, BC etc*/
    clockType clockType;
    /**OUI given by IEEE*/
    uint8_t manufacturerIdentity[4];
    /**Product Type, Product description in Unicode format*/
    descr_t productType;
    /**Revision data of Clock, Firmware and Software. In Unicode*/
    descr_t revData;
    /**Description of device that contains the clock*/
    descr_t userDesc;

    /**the PTP profile of each port of the device*/
    portProfileIdentity_t profileInfo[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /**Physical Address Info of each port*/
    portPhysAddr_t physInfo[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /**Protocol Address Info of each port*/
    portProtAddr_t protInfo[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];

    /**number of communication paths traversed between
      the local clock and the grandmaster clock*/
    uint16_t stepsRemoved;
    /**Specifies the system time in microseconds and the Offset to the local clock value*/
    sysTimeOffset_t timeOffset;

} cipSyncConfig_t;

/*
*  \brief     EIP Config
*             Structure storing the EIP info
*/
typedef struct eip_Config_s
{
    PRUICSS_Handle pruicssHandle;
    ICSS_EMAC_Handle emacHandle;
    TimeSync_ParamsHandle_t timeSyncHandle;
    cipSyncConfig_t cipSyncObj;
    /**Temporary placeholder to copy packets*/
    uint8_t  tempFrame[ICSS_EMAC_MAXMTU];
    EIP_DLRHandle dlrHandle;
} eip_Config;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/** @addtogroup EIP_API
 @{ */

/**
 *  \brief  Initialization routine for Ethernet/IP driver functions
 *
 *          This API does the following functionalities:
 *              Loads the firmware on PRU0 and PRU1 cores
 *              Call DLR init API
 *              Call TimeSync(PTP) init API
 *
 *  \param  icssEipHandle [in] EIP handle
 *
 */
void EIP_drvInit(EIP_Handle icssEipHandle);
/**
 *  \brief  EIP driver stop API
 *
 *          This API stops DLR and disables PTP
 *
 *  \param  icssEipHandle [in] EIP handle
 *
 */
void EIP_drvStop(EIP_Handle icssEipHandle);
/**
 *  \brief  EIP driver start API
 *
 *          This API starts DLR and enables PTP
 *
 *  \param  icssEipHandle [in] EIP handle
 *
 */
void EIP_drvStart(EIP_Handle icssEipHandle);
/**
 *  \brief  API to process the real time Packets
 *
 *          This API will be registered as Real Tme Rx Call back. Incase of EIP, the driver
 *          receives DLR and PTP packets. This API receives the packet, checks the packet
 *          type and passes the packet to DLR driver or PTP driver
 *
 *  \param  queue_number [in] Queue where the packet is present
 *  \param  userArg      [in] userArgumment. EIP handle
 *
 */
void EIP_processProtocolFrames(uint32_t *queue_number, void *userArg);

/**
 *  \brief  API to initialize the CIP Sync objects in the EIP handle
 *          Initialize CIP Sync member variables based on PTP implementation
 *
 *  \param  icssEipHandle [in] EIP handle
 *
 *  \retval  0  - On success
 *           <0 - On failure
 *
 */
int8_t EIP_initializeCIPSync(EIP_Handle icssEipHandle);

/**
@}
*/

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* ICSS_EIP_DRIVER_H_ */

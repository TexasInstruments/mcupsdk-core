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

#ifndef IPNDRV_H_
#define IPNDRV_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include "PN_Handle.h"

/* ========================================================================== */
/*                          Doxygen                                           */
/* ========================================================================== */
/**
 * \defgroup PN_PHASE_MANAGEMENT Phase Management APIs
 * \ingroup INDUSTRIAL_COMMS_PROFINET_DEVICE_FWHAL_MODULE
 */

/**
* \defgroup PN_DCP_FILTER DCP Filter APIs
* \ingroup INDUSTRIAL_COMMS_PROFINET_DEVICE_FWHAL_MODULE
*/

/**
* \defgroup PN_FILTER_TABLE Multicast Filter Table APIs
* \ingroup INDUSTRIAL_COMMS_PROFINET_DEVICE_FWHAL_MODULE
*/

/**
* \defgroup PN_WATCHDOG Watchdog timer APIs
* \ingroup INDUSTRIAL_COMMS_PROFINET_DEVICE_FWHAL_MODULE
*/

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \def ERR_STATION_NAME_LENGTH
 *      Error code for Wrong station name length
 */
#define ERR_STATION_NAME_LENGTH     -1

/**
 * \def ERR_FIRMWARE_VERSION_BAD
 *      Error code for Wrong firmware version
 */
#define ERR_FIRMWARE_VERSION_BAD    -2

/**
 * \def ERR_FIRMWARE_LOAD_FAIL
 *      Error code for failure in firmware loading
 */
#define ERR_FIRMWARE_LOAD_FAIL      -3

/**
 * \def ERR_DRIVER_INIT_FAIL
 *      Error code for failure in initializing driver
 */
#define ERR_DRIVER_INIT_FAIL        -4

#define PRU_IEP_CMP_CFG_EN_SOG_EVENTS   0x000001cf  /*sets cmp0, cmp2/5/6/7*/

/**
 * \def NO_RCV_NO_FWD
 *      No receive and no forward
 */
#define NO_RCV_NO_FWD 0

/**
 * \def RCV_NO_FWD
 *      Receive but no forward
 */
#define RCV_NO_FWD    1

/**
 * \def NO_RCV_FWD
 *      No receive but forward
 */
#define NO_RCV_FWD    2

/**
 * \def RCV_FWD
 *      Receive and forward
 */
#define RCV_FWD       3

/**
 * \def OFF
 *      RTClass3 status values of a port
 */
#define OFF                         0

/**
 * \def UP
 *      RTClass3 status values of a port
 */
#define UP                          2

/**
 * \def RUN
 *      RTClass3 status values of a port
 */
#define RUN                         4

/** \addtogroup PN_MRP
 @{ */

/**
 * \def DISABLED
 *      MRP Port disabled state
 */
#define DISABLED                    0

/**
 * \def BLOCKING
 *      MRP Port blocking state
 */
#define BLOCKING                    1
/**
 * \def FORWARDING
 *      MRP Port forwarding state
 */
#define FORWARDING                  2

#ifdef WATCHDOG_SUPPORT
/**
 * \def watchDogExpireDuration
 *      ICSS WatchDog Expiry duration in milli seconds
 */
#define watchDogExpireDuration      100
#endif


/**
@}*/

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief Initializes the Profinet Driver
 *
 * Sets the Port MAC addresses \n
 * Sets the Compensation value \n
 * Initialize the RTC driver   \n
 * Initialize the PTCP driver  \n
 * Set MRP port state \n
 * Loads the forward and receive multicast tables \n
 * Loads the Profinet firmware
 *
 * \param pnHandle Profinet Handle
 * \retval 0 on success     \n
 *  Error codes :   \n
 *      \ref ERR_DRIVER_INIT_FAIL \n
 *      \ref ERR_FIRMWARE_VERSION_BAD
 */
int32_t PN_initDrv(PN_Handle pnHandle);

 /**
  * \brief Get the details on version of firmware
  *
  * \param version_major Reference to get the major version number
  * \param version_minor Reference to get the minor version number
  * \param version_build Reference to get the version build number
  * \param version_release_type Reference to get the firmware release type
  *
  */
 void PN_getFirmwareVersion(uint32_t *version_major, uint32_t *version_minor,
                            uint32_t *version_build, uint32_t *version_release_type);

 /**
  * \brief Get the details on release information and features of firmware
  *
  * \param firmware_release_1 Reference to get the details from ICSS_FIRMWARE_RELEASE_1
  * \param firmware_release_2 Reference to get the details from ICSS_FIRMWARE_RELEASE_2
  * \param firmware_feature_mask Reference to get the firmware feature mask
  *
  */
 void PN_getFirmwareReleaseInfoAndFeatures(uint32_t *firmware_release_1,
                                           uint32_t *firmware_release_2,
                                           uint32_t *firmware_feature_mask);

/** \addtogroup PN_DCP_FILTER
 @{ */
/**
 * \brief API to configure the station name and length of station name of device for filtering DCP Identify requests.
 *
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 * \param[in] dcpNameOfStation      Pointer to the array which contains the station name
 * \param[in] lengthOfStationName   Length of the station name. Valid values are from 0 to 240.
 *                                  If the length is configured as zero then all DCP Ident request are passed to host.
 *
 * \retval 0 on success     \n
 *  Error codes :   \n
 *      \ref ERR_STATION_NAME_LENGTH
 */
int32_t PN_setDcpFilterStationName(PRUICSS_HwAttrs const *pruicssHwAttrs,
                                   const uint8_t *dcpNameOfStation,
                                   uint8_t lengthOfStationName);

/**
@}
*/

/**
 * \brief API to set the MRP state for a port
 *
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 * \param[in] portNumber Port for which MRP state has to be configured \n
 *              ICSS_EMAC_PORT_1 (1) \n
 *              ICSS_EMAC_PORT_2 (2)
 * \param[in] pState There are three valid values of the MRP port state \n
 *              \ref DISABLED \n
 *              \ref BLOCKING \n
 *              \ref FORWARDING
 * \retval 0 on success
 */
int32_t PN_MRP_setPortState(PRUICSS_HwAttrs const *pruicssHwAttrs,
                            uint8_t portNumber,
                            uint8_t pState);

/**
 * \brief Returns the MRP state of a port
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 * \param[in] portNumber Port number whose MRP port state has to be returned \n
 *              ICSS_EMAC_PORT_1 (1) \n
 *              ICSS_EMAC_PORT_2 (2)
 * \param[in] pState Reference to the MRP port state of the port \n
 *
 * \retval 0 on success
 */
int32_t PN_MRP_getPortState(PRUICSS_HwAttrs const *pruicssHwAttrs,
                            uint8_t portNumber,
                            uint8_t *pState);

/** \addtogroup PN_PHASE_MANAGEMENT
 @{ */
/**
 * \brief API to map a Phase to start of green Profile.
 *
 * A phase can be mapped to different profiles for Receive (Rx) and Transmit (Tx) on a port.
 *

 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 * \param[in] portNumber Port on which phase to profile mapping has to be done \n
 *              ICSS_EMAC_PORT_1 (1) \n
 *              ICSS_EMAC_PORT_2 (2)
 * \param[in] phaseNumber IRT Phase Number \n
 *              Valid values are from 1 to 16
 * \param[in] profileNumberRx Profile number mapped for Receive \n
 *              Valid values are from 1 to 5
 * \param[in] profileNumberTx Profile number mapped for Transmit \n
 *              Valid values are from 1 to 5
 *
 * \retval 0 on success
 */
int32_t PN_mapPhaseToProfile(PRUICSS_HwAttrs const *pruicssHwAttrs, int32_t portNumber, int32_t phaseNumber,
                             int32_t profileNumberRx, int32_t profileNumberTx);



/**
 * \brief API to configure the maximum line receive delay.
 *
 * This value is provided by the PLC when it establishes connection with the device.
 *

 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 * \param[in] portNumber Port for which receive line delay is configured \n
 *              ICSS_EMAC_PORT_1 (1) \n
 *              ICSS_EMAC_PORT_2 (2)
 * \param[in] maxLineRxDelayValue Line delay as seen by the PLC at a port. This value is computed by the PLC/Engineering tool.
 *
 * \retval 0 on success
 */
int32_t PN_setMaxLineRxDelay(PRUICSS_HwAttrs const *pruicssHwAttrs, uint8_t portNumber, int32_t maxLineRxDelayValue);

/**
 * \brief API to configure the maximum bridge delay. This value comes from the GSD file.
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 * \param[in] maxBridgeDelayValue \n
 *              Valid value for our device is 2900ns \n
 *              This value is defined in the GSD
 *
 * \retval 0 on success
 */
int32_t PN_setMaxBridgeDelay(PRUICSS_HwAttrs const *pruicssHwAttrs, int32_t maxBridgeDelayValue);


/**
 * \brief API to configure the yellow period.
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 * \param[in] yellowPeriodTime Length of the yellow period \n
 *              Valid value is 125 us as our device does not support fragmentation
 *
 * \retval 0 on success
 */
int32_t PN_setYellowPeriod(PRUICSS_HwAttrs const *pruicssHwAttrs, int32_t yellowPeriodTime);


/**
 * \brief API to configure a Profile.
 * In a profile the start of green time in a cycle for Receive (Rx) and Transmit (Tx) is set for a port.
 *
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 * \param[in] portNumber Port for which a profile is configured \n
 *              PORT1 (1)    \n
 *              PORT2 (2)
 * \param[in] profileNumber Five profiles can be defined for a port \n
 *              Valid values are from 1 to 5
 * \param[in] rxSoGValue Start of Green value for receive at the port \n
 *              Valid value can range from 0 to the max of cycle time
 * \param[in] txSoGValue Start of Green value for transmit at the port \n
 *              Valid value can range from 0 to the max of cycle time
 *
 * \retval 0 on success
 */
int32_t PN_setProfile(PRUICSS_HwAttrs const *pruicssHwAttrs,
                      int32_t portNumber, int32_t profileNumber, int32_t rxSoGValue,
                      int32_t txSoGValue);

/**
 * \brief API to configure the compensation value which is used while computing the forward FSO for a RTC3 frame which has to be forwarded.
 * \param[in] compensationValue  compensation for the jitter
 *              Valid range is from 0 to 100
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 */
void PN_setCompensationValue(PRUICSS_HwAttrs const *pruicssHwAttrs, uint16_t compensationValue);


/**
 * \brief API to set the Red Guard for the device.
 *
 * Received RTC3 frames are dropped if their FID doesn't fall in the range set by the Red Guard.
 *
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 * \param[in] validLowerFrameId Lower value of RTC3 FID for the Red Guard \n
 *              Valid range is 0x0100 to 0x0FFF \n
 * \param[in] validUpperFrameId \n
 *              Valid range is 0x0100 to 0x0FFF and equal or greater then validLowerFrameId \n
 * \retval 0 on success
 */
int32_t PN_setRedGuard(PRUICSS_HwAttrs const *pruicssHwAttrs,
                       uint16_t validLowerFrameId, uint16_t validUpperFrameId);

/**
 * \brief API to set the RTClass3 port status of a port.
 *
 * If the status of a port is set to UP or RUN then firmware internally enables Phase
 *        Management on that port.
 *
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 * \param[in] portNumber Port whose status has to be configured \n
 *              ICSS_EMAC_PORT_1 (1)     \n
 *              ICSS_EMAC_PORT_2 (2)
 * \param[in] status There are three valid values of the RTClass3 port status \n
 *              \ref OFF \n
 *              \ref UP  \n
 *              \ref RUN
 * \retval 0 on success
 */
int32_t PN_setRtc3PortStatus(PRUICSS_HwAttrs const *pruicssHwAttrs, uint8_t portNumber, uint8_t status);
/**
@}
*/

/** \addtogroup PN_FILTER_TABLE
 @{ */
/**
 * \brief API to enable/disable the reception & forward for a Multicast address in port multicast filter tables.
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 * \param[in] macAddr MAC Address for which the reception or forward has to be enabled or disabled \n
 *              Following are the valid ranges of multicast addresses: \n
 *              (01-0E-CF-00-00-00 TO 01-0E-CF-00-05-FF) \n
 *              (01-15-4E-00-00-00 TO 01-15-4E-00-00-1F) \n
 *              (01-80-C2-00-00-00 TO 01-80-C2-00-00-1F)
 * \param[in] ctrl  A MAC address can be configured using one of four valid configuration for reception and foward \n
                For example, if MAC address is called with NO_RCV_NO_FWD then it's reception and forwarding will be disabled \n
 *              \ref NO_RCV_NO_FWD \n
 *              \ref RCV_NO_FWD \n
 *              \ref NO_RCV_FWD \n
 *              \ref RCV_FWD
 * \param[in] portNumber Port on which reception and forwarding is configured for a MAC address
 *              PORT1 (1) \n
 *              PORT2 (2) \n
 * \param[in] table There are two sets of table one for BLOCKING state and second one for FORWARDING state \n
 *              \ref BLOCKING \n
 *              \ref FORWARDING
 * \retval 0 on success
 */
int32_t PN_setStaticFilterTable(PRUICSS_HwAttrs const *pruicssHwAttrs, const uint8_t *macAddr, uint8_t ctrl,
                                uint8_t portNumber, uint8_t table);
/**
@}
*/
#ifdef WATCHDOG_SUPPORT
/** \addtogroup PN_WATCHDOG
 @{ */

/**
* \brief API to configure the ICSS watchdog expiry duration.
* \param pnHandle Profinet Handle
* \param[in] timerPeriod Time duration (in milliseconds) after which watchdog expires if not tapped in-between.
*              Valid range is from 2 ms to 6553 ms
* \retval 0 on success
*/

int32_t PN_setWatchDogTimer(PN_Handle pnHandle, int32_t timerPeriod);

/**
@}
*/
#endif

/**
 *  \ingroup PN_FILTER_TABLE
 *  \brief   PN_loadStaticTable API to load multicast static table to ICSS memory
 *  \details API to load multicast static table to ICSS memory
 *
 *  \param[in]  pruicssHwAttrs      PRUICSS HW Attributes for base addresses
 *  \param[in]  staticTable         start address of the static table
 *  \param[in]  staticTableLength   length of the static table in bytes
 *  \param[in]  staticTableType     type of static table 0 == Forward Table, 1 == Receive Table
 *  \param[in]  portNumber          Allowed values : \ref ICSS_EMAC_PORT_1, \ref ICSS_EMAC_PORT_2
 *
 *  \retval     0 on sucess or <0 on failure
 */

int32_t PN_loadStaticTable(PRUICSS_HwAttrs const *pruicssHwAttrs,
                           const uint32_t *staticTable,
                           uint8_t staticTableLength,
                           uint8_t staticTableType,
                           uint8_t portNumber);

/**
 *  \ingroup PN_CPM_PPM_MANAGEMENT
 *  \brief   Profinet CPM ISR handler
 *  \details
 *
 *  \param[in]  arg
 */
void PN_cpmIsrHandler(void* arg);
/**
 *  \ingroup PN_CPM_PPM_MANAGEMENT
 *  \brief   Profinet PPM ISR handler
 *  \details
 *
 *  \param[in]  arg
 */
void PN_ppmIsrHandler(void* arg);
/**
 *  \ingroup PN_CPM_PPM_MANAGEMENT
 *  \brief   Profinet DHT ISR handler
 *  \details
 *
 *  \param[in]  arg
 */
void PN_dhtIsrHandler(void* arg);
/**
 *  \ingroup PN_PTCP
 *  \brief   Profinet PTCP ISR handler
 *  \details
 *
 *  \param[in]  arg
 */
void PN_PTCP_isrHandler(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* IPNDRV_H_ */

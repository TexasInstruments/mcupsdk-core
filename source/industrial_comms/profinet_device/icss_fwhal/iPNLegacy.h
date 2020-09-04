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

#ifndef IPNLEGACY_H_
#define IPNLEGACY_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include "iRtcDrv2.h"
#include "iPtcpDrv.h"
#include "PN_Handle.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**Interface macid index*/
#define INTERFACE_MAC 0
/**Port1 macid index*/
#define PORT1_MAC     1
/**Port2 macid index*/
#define PORT2_MAC     2
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
  * \brief Stores the local copy of handle to support legacy APIs
  * \param pnHandle Profinet Handle
  */
void PN_setHandle(PN_Handle pnHandle);

/**
  * \brief Returns the ICSS EMAC LLD handle. This is used to maintain the old APIs that doesn't use the Handle based scheme
  * \retval returns Profinet Handle
  */
PN_Handle PN_getPnHandle();

/**
 * \brief Returns the MAC Addr for ports. User can implement this as required
 *
 * \param pnHandle Profinet Handle
 * \param instance Port ID - INTERFACE_MAC, PORT1_MAC, PORT2_MAC
 * \param[out] macId  stores the MACID in the pointer
 *
 * \retval E_FAIL on failure, S_PASS on success
 */
uint8_t PN_EmacSocMACAddrGet(PN_Handle pnHandle, uint8_t instance,
                             uint8_t *macId);

/**
 * \brief Legacy API. Calls PN_setStaticFilterTable
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
 *              \ref BLOCKING   \n
 *              \ref FORWARDING
 * \retval 0 on success
 */
int32_t setStaticFilterTable(const uint8_t *macAddr, uint8_t ctrl,
                             uint8_t portNumber, uint8_t table);

/**
 * \brief Legacy API, calls PN_setRtc3PortStatus
 *
 * If the status of a port is set to UP or RUN then firmware internally enables Phase
 *        Management on that port.
 *
 * \param[in] portNumber Port whose status has to be configured \n
 *              PORT1 (1)    \n
 *              PORT2 (2)
 * \param[in] status There are three valid values of the RTClass3 port status \n
 *              \ref OFF \n
 *              \ref UP  \n
 *              \ref RUN
 * \retval 0 on success
 */
int32_t setRtc3PortStatus(uint8_t portNumber, uint8_t status);

/**
 * \brief Legacy API. Calls PN_getLastCpm
 * \param pkt pointer to packet object
 * \return buffer PROC buffer address
 */
uint8_t *getLastCpm(t_rtcPacket *pkt);

/**
 * \brief Legacy API. Calls PN_setRedGuard
 *
 * Received RTC3 frames are dropped if their FID doesn't fall in the range set by the Red Guard.
 *
 * \param[in] validLowerFrameId Lower value of RTC3 FID for the Red Guard \n
 *              Valid range is 0x0100 to 0x0FFF \n
 * \param[in] validUpperFrameId \n
 *              Valid range is 0x0100 to 0x0FFF and equal or greater then validLowerFrameId \n
 * \retval 0 on success
 */
int32_t setRedGuard(uint16_t validLowerFrameId, uint16_t validUpperFrameId);

/**
 * \brief Legacy API. Calls PN_setMaxLineRxDelay
 *
 * This value is provided by the PLC when it establishes connection with the device.
 *
 * \param[in] portNumber Port for which receive line delay is configured \n
 *              PORT1 (1) \n
 *              PORT2 (2)
 * \param[in] maxLineRxDelayValue Line delay as seen by the PLC at a port. This value is computed by the PLC/Engineering tool.
 *
 * \retval 0 on success
 */
int32_t setMaxLineRxDelay(uint8_t portNumber, int32_t maxLineRxDelayValue);

/**
 * \brief Legacy API. Calls PN_setMaxBridgeDelay
 * \param[in] maxBridgeDelayValue \n
 *              Valid value for our device is 2900ns \n
 *              This value is defined in the GSD
 *
 * \retval 0 on success
 */
int32_t setMaxBridgeDelay(int32_t maxBridgeDelayValue);

/**
 * \brief Legacy API. Calls PN_setYellowPeriod
 * \param[in] yellowPeriodTime Length of the yellow period \n
 *              Valid value is 125 us as our device does not support fragmentation
 *
 * \retval 0 on success
 */
int32_t setYellowPeriod(int32_t yellowPeriodTime);

/**
 * \brief Legacy API. Calls PN_mapPhaseToProfile
 *
 * A phase can be mapped to different profiles for Receive (Rx) and Transmit (Tx) on a port.
 *
 * \param[in] portNumber Port on which phase to profile mapping has to be done \n
 *              PORT1 (1) \n
 *              PORT2 (2)
 * \param[in] phaseNumber IRT Phase Number \n
 *              Valid values are from 1 to 16
 * \param[in] profileNumberRx Profile number mapped for Receive \n
 *              Valid values are from 1 to 5
 * \param[in] profileNumberTx Profile number mapped for Transmit \n
 *              Valid values are from 1 to 5
 *
 * \retval 0 on success
 */
int32_t mapPhaseToProfile(int32_t portNumber, int32_t phaseNumber,
                          int32_t profileNumberRx, int32_t profileNumberTx);

/**
 * \brief Legacy API. Calls PN_setProfile
 * In a profile the start of green time in a cycle for Receive (Rx) and Transmit (Tx) is set for a port.
 *
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
int32_t setProfile(int32_t portNumber, int32_t profileNumber,
                   int32_t rxSoGValue, int32_t txSoGValue);

/**
 * \brief Legacy API. Calls PN_setDcpFilterStationName
 *
 * \param[in] dcpNameOfStation     Pointer to the array which contains the station name
 * \param[in] lengthOfStationName  Length of the station name. Valid values are from 0 to 240.
 *                                 If the length is configured as zero then all DCP Ident request are passed to host.
 *
 * \retval 0 on success
 */
int32_t setDcpFilterStationName(const uint8_t *dcpNameOfStation,
                                uint8_t lengthOfStationName);

/**
 * \brief Legacy API. Calls PN_OS_txPacket
 *
 * this is using a critical section to protect re-entry of TX function
 * the protection scheme is borrowed from NDK and we use their code too...
 * this requires to adhere to NDK priority scheme
 *
 * \param srcAddress        pointer to TX packet
 * \param portNumber        output port number
 * \param queuePriority     output queue priority
 * \param lengthOfPacket    TX packet length (without CRC)
 * \callgraph
 */
int32_t TxPacketOS(const uint8_t *srcAddress, int32_t portNumber,
                   int32_t queuePriority, int32_t lengthOfPacket);

/**
 *       Legacy API. Calls ICSS_EmacRxPktInfo
 *
 *  \param[out]  portNumber    Return pointer of port number where frame was received
 *  \param[out]  queueNumber   Return pointer of host queue where the received frame is queued

 *  \retval     none
 */
int32_t PN_RxPktInfo(int32_t *portNumber, int32_t *queueNumber);

/**
 *      Legacy API. Calls ICSS_EmacRxPktGet
 *
 *  \param[in]  destAddress   Base address of data buffer where received frame has
 *                            to be stored
 *  \param[in]  queueNumber   Receive queue from which frame has to be copied
 *  \param[in]  port    Returns port number on which frame was received
 *  \param[out]  more    Returns more which is set to 1 if there are more frames in the queue

 *  \retval     Length of the frame received in number of bytes or -1 on Failure
 */
int32_t PN_RxPktGet(uint32_t destAddress, int32_t queueNumber, int32_t *port,
                    int32_t *more);

/**
* \brief Legacy API calls findMAC
*
* \param  macId MAC ID which is to be found
*
* \retval portNumber where the MAC ID exists, 0 means not found 1 means port 0 and 2 means port 1
*/
uint8_t PN_findMAC(uint8_t *macId);

/**
 * \brief  Legacy API. Calls PN_setCpmDHT
 *
 *  \param dht          Data hold timeout value (lost count * reduction ration)*
 *  \param pos          position of CPM descriptor
 *                          Valid Range 0-7
 *
 * \retval 0 on success
 * \retval <0 if failure
 *
 */
int32_t setCpmDHT(uint16_t dht, uint8_t pos);

/**
 * \brief Legacy API, calls PN_setBaseClock
 *
 * \param factor Clock send factor: min value 3, max value 128
 *
 * \retval Slot number (>=0)
 * \retval <0 if failure
 *
 */
int32_t setBaseClock(uint16_t factor);

/**
 * \internal
 * \brief Legacy API. Calls PN_clearList
 *
 *  \param list     Direction
 *                      Valid values for 'list' - CPM or PPM
 *
 * \retval 0 on Success
 * \retval <0 if Failure
 *
 */
int32_t clearList(uint8_t list);

/**
 * \brief Enables the PN interrupts. Legacy API. Calls PN_RTC_enableISR
 * \retval 0 on Success
 */
int32_t iRtcEnableIsr();

/**
 * \brief Legacy API. Calls PN_delPmList
 * \param pmPkt pointer to RTC packet to be deleted from list
 *
 * \retval negative value on error, zero on success
 *
 * - stop sending the CPM/PPM packet (clears active bit) and clear buffer usage
 * - clears the ARgroup relation
 * - remove packet from shadow descriptor list and toggle lists
 */
int32_t delPmList(t_rtcPacket *pmPkt);

/**
 * \brief Legacy API. Calls PN_insPpmList
 *
 * \param ppmPkt pointer to PPM packet to be inserted in list
 * \param legMode TRUE(1): enables legacy mode support (same packet sent as RTC1 with
 *                RR=128 during startup)
 *
 * \retval negative value on error, zero on success
 *
 * this now manages single RED packet and index list processing in general
 * it will call toggleList() in case of a good setup
 */
int32_t insPpmList(t_rtcPacket *ppmPkt, uint8_t legMode);

/**
 *
 * \brief Legacy API. Calls PN_insCpmList
 * \param cpmPkt pointer to CPM packet to be inserted in list
 *
 * \retval negative value on error, zero on success
 *
 * processes list toggling
 * no index list setup for CPM so far
 *
 * currently we do not distinguish between green and red packets
 * also there are no checks on receive at correct time or phase
 * so that fields are currently not set!
 */
int32_t insCpmList(t_rtcPacket *cpmPkt);

/**
 * \internal
 * \brief Legacy API, Calls PN_PTCP_configureSync0Pin
 */
void configureSync0Pin();

/**
 * \brief               Legacy API, calls PN_PTCP_configureSyncFwd
 *
 * \param[in] state     disable /enable
 */
void ptcpConfigureSyncFwd(ptcpPortStatus_t state);

/**
 * \brief               Legacy API, Used to register RX Callback
 *
 * \param[in] callBackPtr   Function pointer for RX Callback
 * \param[in] userArg       User params for callBackPtr
 */
void PN_EmacRegisterProtocolCallback(void *callBackPtr, void *userArg);

/**
 * \brief               Legacy API, Used to register Link ISR Callback
 *
 * \param[in] callBack   Function pointer for Link ISR Callback
 * \param[in] userArg       User params for callBack
 */
void PN_EmacRegisterPort0ISRCallback(ICSS_EMAC_CallBack callBack, void *userArg);

/**
 * \brief               Legacy API, Used to register Link ISR Callback
 *
 * \param[in] callBack   Function pointer for Link ISR Callback
 * \param[in] userArg       User params for callBack
 */
void PN_EmacRegisterPort1ISRCallback(ICSS_EMAC_CallBack callBack, void *userArg);

/**
 * \brief               Legacy API, Used to purgeTable
 *
 * \param[in] portNum   Port number to purge learning table
 */
void PN_purgeTable(uint8_t portNum);

/**
 * \brief               Legacy API, Used to call PN_MRP_setPortState
 *
 * \param[in] portNumber   Port number to purge learning table
 * \param[in] pState    Set MRP state
 */
int32_t setMrpPortState(uint8_t portNumber, uint8_t pState);

/**
 * \brief               Legacy API, Used to call PN_MRP_getPortState
 *
 * \param[in] portNumber   Port number to purge learning table
 * \param[out] pState   Return MRP state
 */
int32_t getMrpPortState(uint8_t portNumber, uint8_t *pState);

/**
 * \brief Legacy API. Calls PN_cfgRtcMem
 * \param
 *    ar    number of allowed number of ARs (symmetrical PPM/CPM)
 *          valid values only: 1, 2, 4, 8
 * \param   size    RTC payload size (see below)
 *
 * \retval negative value on error, zero on success
 *
 * configuration function for static PPM buffer allocation
 * currently only supports symmetric PPM config with same max size
 * e.g.
 *
 * \arg 2 * 1440 bytes payload
 * \arg 4 * 720 bytes
 * \arg 8 * 360 bytes (lower size is ok...)
 *
 * PPM buffers will be allocated in buffer blocks 0/1 evenly distributed
 * Start addresses are only valid for PRU1/Port 1 TX...
 * Port 2 PPM needs to be adapted with correct offset at creation of PPM
 * descriptor as the port is not known yet.
 */
int32_t cfgRtcMem(uint8_t ar, uint16_t size);

/**
 * \brief               Legacy API, Used to call \ref PN_allocPkt
 *
 * \param[out] pPkt a packet object pointer
 * \param type  \ref CPM or \ref PPM
 * \retval index in the list on success
 * \retval <0 if failure
 */
int32_t allocPkt(t_rtcPacket **pPkt, uint8_t type);

/**
 * \brief               Legacy API, Used to call \ref PN_PTCP_getSyncInfo
 */
void ptcpGetSyncInfo(ptcpSyncInfo_t *syncInfo);

/**
 * \brief               Legacy API, Used to call \ref PN_PTCP_registerDelayUpdateCall
 */
void ptcpRegisterDelayUpdateCall(ptcpCallBack_t callBack);

/**
 * \brief               Legacy API, Used to call \ref PN_PTCP_registerSyncStatusCall
 */
void ptcpRegisterSyncStatusCall(ptcpCallBack_t callBack);

/**
 * \brief               Legacy API, Used to call \ref PN_PTCP_setPllWindow
 */
void ptcpSetPllWindow(uint32_t pllWindowSize);

/**
 * \brief               Legacy API, Used to call \ref PN_PTCP_registerSyncStatusCall
 */
void ptcpSetSyncTimeoutFactor(uint32_t syncTimeoutFactor);

/**
 * \brief               Legacy API, Used to call \ref PN_PTCP_setSyncUUID
 */
void ptcpSetSyncUUID(uint8_t *subdomainUUID);

/**
 * \brief               Legacy API, Used to call \ref PN_registerCpmCall
 */
void registerCpmCall(pnCallBack_t callBack);

/**
 * \brief               Legacy API, Used to call \ref PN_registerStatCall
 */
void registerStatCall(pnCallBack_t callBack);

/**
 * \brief               Legacy API, Used to call \ref PN_relPpmBuff
 */
uint8_t *relPpmBuff(t_rtcPacket *pkt);

/**
 * \brief               Legacy API, Used to call \ref PN_RTC_disableISR
 */
int32_t rtcDisableISR();


#ifdef __cplusplus
}
#endif

#endif /* IPNLEGACY_H_ */

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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "PN_Handle.h"
#include "PN_HandleDef.h"
#include "iPNLegacy.h"
#include "iPNDrv.h"
#include "iRtcDrv.h"
#include "iPnOs.h"
#include "iPtcpUtils.h"

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* ========================================================================== */
/*                          Static variables                                  */
/* ========================================================================== */
/** \brief Local copy of ICSS_EMAC_Handle used to maintain old APIs that don't use handles */
static PN_Handle localHandle;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void PN_setHandle(PN_Handle pnHandle)
{
    localHandle = pnHandle;
}

PN_Handle PN_getPnHandle()
{
    return localHandle;
}

uint8_t PN_EmacSocMACAddrGet(PN_Handle pnHandle, uint8_t instance,
                             uint8_t *macId)
{
    /*TODO: Review this*/
    pnHandle->getMACAddress(instance, macId);
    return 0;
}

uint8_t pnEmacSocMACAddrGet(uint8_t instance, uint8_t *macId)
{
    return PN_EmacSocMACAddrGet(PN_getPnHandle(), instance, macId);
}

int32_t setStaticFilterTable(const uint8_t *macAddr, uint8_t ctrl,
                             uint8_t portNumber, uint8_t table)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);

    return PN_setStaticFilterTable(pruicssHwAttrs, macAddr, ctrl, portNumber, table);
}

int32_t setRtc3PortStatus(uint8_t portNumber, uint8_t status)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);
    return PN_setRtc3PortStatus(pruicssHwAttrs, portNumber, status);
}

uint8_t *getLastCpm(t_rtcPacket *pkt)
{
    return PN_getLastCpm(PN_getPnHandle() , pkt);
}

int32_t setRedGuard(uint16_t validLowerFrameId, uint16_t validUpperFrameId)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);
    return PN_setRedGuard(pruicssHwAttrs, validLowerFrameId, validUpperFrameId);
}

int32_t setMaxLineRxDelay(uint8_t portNumber, int32_t maxLineRxDelayValue)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);
    return PN_setMaxLineRxDelay(pruicssHwAttrs, portNumber, maxLineRxDelayValue);
}

int32_t setMaxBridgeDelay(int32_t maxBridgeDelayValue)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);
    return PN_setMaxBridgeDelay(pruicssHwAttrs, maxBridgeDelayValue);
}

int32_t setYellowPeriod(int32_t yellowPeriodTime)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);
    return PN_setYellowPeriod(pruicssHwAttrs, yellowPeriodTime);
}

int32_t mapPhaseToProfile(int32_t portNumber, int32_t phaseNumber,
                          int32_t profileNumberRx, int32_t profileNumberTx)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);
    return PN_mapPhaseToProfile(pruicssHwAttrs, portNumber, phaseNumber,
                                profileNumberRx, profileNumberTx);
}

int32_t setProfile(int32_t portNumber, int32_t profileNumber,
                   int32_t rxSoGValue, int32_t txSoGValue)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);
    return PN_setProfile(pruicssHwAttrs, portNumber, profileNumber, rxSoGValue,
                         txSoGValue);
}

int32_t setDcpFilterStationName(const uint8_t *dcpNameOfStation,
                                uint8_t lengthOfStationName)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);
    return PN_setDcpFilterStationName(pruicssHwAttrs, dcpNameOfStation, lengthOfStationName);
}

int32_t TxPacketOS(const uint8_t *srcAddress, int32_t portNumber,
                   int32_t queuePriority, int32_t lengthOfPacket)
{
    ICSS_EMAC_Handle emacHandle = PN_getPnHandle()->emacHandle;
    return PN_OS_txPacket(emacHandle,  srcAddress, portNumber, queuePriority,
                          lengthOfPacket);
}

int32_t PN_RxPktInfo(int32_t *portNumber, int32_t *queueNumber)
{
    ICSS_EMAC_Handle emacHandle = PN_getPnHandle()->emacHandle;
    return ICSS_EMAC_rxPktInfo(emacHandle, portNumber, queueNumber);
}

int32_t PN_RxPktGet(uint32_t destAddress, int32_t queueNumber, int32_t *port,
                    int32_t *more)
{
    int32_t retval = 0;
    ICSS_EMAC_Handle emacHandle = PN_getPnHandle()->emacHandle;

    ICSS_EMAC_RxArgument rxArgs;
    rxArgs.icssEmacHandle = emacHandle;
    rxArgs.destAddress = destAddress;
    rxArgs.queueNumber = queueNumber;
    rxArgs.more = 0;
    rxArgs.port = 0;
    retval = ICSS_EMAC_rxPktGet(&rxArgs, NULL);
    *more = rxArgs.more;
    *port = rxArgs.port;
    return retval;
}

uint8_t PN_findMAC(uint8_t *macId)
{
    ICSS_EMAC_Handle emacHandle;
    ICSS_EMAC_IoctlCmd ioctlParams;
    ioctlParams.ioctlVal = macId;

    ioctlParams.command = ICSS_EMAC_LEARN_CTRL_FIND_MAC;

    emacHandle = PN_getPnHandle()->emacHandle;

    return ICSS_EMAC_ioctl(emacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL, (uint32_t)NULL,
                          (void *)&ioctlParams);
}

int32_t setCpmDHT(uint16_t dht, uint8_t pos)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);
    return PN_setCpmDHT(pruicssHwAttrs, dht, pos);
}

int32_t setBaseClock(uint16_t factor)
{
    return PN_setBaseClock(PN_getPnHandle() , factor);
}

int32_t clearList(uint8_t list)
{
    return PN_clearList(PN_getPnHandle() , list);
}

int32_t iRtcEnableIsr()
{
    return PN_RTC_enableISR(PN_getPnHandle());
}

int32_t delPmList(t_rtcPacket *pmPkt)
{
    return PN_delPmList(PN_getPnHandle() , pmPkt);
}

int32_t insPpmList(t_rtcPacket *ppmPkt, uint8_t legMode)
{
    return PN_insPpmList(PN_getPnHandle() , ppmPkt, legMode);
}

int32_t insCpmList(t_rtcPacket *cpmPkt)
{
    return PN_insCpmList(PN_getPnHandle()  , cpmPkt);
}

#ifdef PTCP_SUPPORT
void configureSync0Pin()
{
    PN_PTCP_configureSync0Pin(PN_getPnHandle());
}
#endif

void ptcpConfigureSyncFwd(ptcpPortStatus_t state)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);

    PN_PTCP_configureSyncFwd(pruicssHwAttrs, state);
}

void PN_purgeTable(uint8_t portNum)
{
    ICSS_EMAC_IoctlCmd ioctlParams;
    ICSS_EMAC_Handle emacHandle;
    ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
    emacHandle = PN_getPnHandle()->emacHandle;
    ICSS_EMAC_ioctl(emacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL, portNum,
                   (void *)&ioctlParams);
}

int32_t setMrpPortState(uint8_t portNumber, uint8_t pState)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);

    return PN_MRP_setPortState(pruicssHwAttrs, portNumber, pState);
}

int32_t getMrpPortState(uint8_t portNumber, uint8_t *pState)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((PN_getPnHandle())->pruicssHandle->hwAttrs);

    return PN_MRP_getPortState(pruicssHwAttrs, portNumber, pState);
}

int32_t cfgRtcMem(uint8_t ar, uint16_t size)
{
    return PN_cfgRtcMem(PN_getPnHandle(), ar, size);
}

int32_t allocPkt(t_rtcPacket **pPkt, uint8_t type)
{
    return PN_allocPkt(PN_getPnHandle(), pPkt, type);
}
#ifdef PTCP_SUPPORT
void ptcpGetSyncInfo(ptcpSyncInfo_t *syncInfo)
{
    PN_PTCP_getSyncInfo(PN_getPnHandle(), syncInfo);
}

void ptcpRegisterDelayUpdateCall(ptcpCallBack_t callBack)
{
    PN_PTCP_registerDelayUpdateCall(PN_getPnHandle(), callBack);
}

void ptcpRegisterSyncStatusCall(ptcpCallBack_t callBack)
{
    PN_PTCP_registerSyncStatusCall(PN_getPnHandle(), callBack);
}

void ptcpSetPllWindow(uint32_t pllWindowSize)
{
    PN_PTCP_setPllWindow(PN_getPnHandle(), pllWindowSize);
}

void ptcpSetSyncTimeoutFactor(uint32_t syncTimeoutFactor)
{
    PN_PTCP_setSyncTimeoutFactor(PN_getPnHandle(), syncTimeoutFactor);
}

void ptcpSetSyncUUID(uint8_t *subdomainUUID)
{
    PN_PTCP_setSyncUUID(PN_getPnHandle(), subdomainUUID);
}
#endif

void registerCpmCall(pnCallBack_t callBack)
{
    PN_registerCpmCall(PN_getPnHandle(), callBack);
}

void registerStatCall(pnCallBack_t callBack)
{
    PN_registerStatCall(PN_getPnHandle(), callBack);
}

uint8_t *relPpmBuff(t_rtcPacket *pkt)
{
    return PN_relPpmBuff(PN_getPnHandle(), pkt);
}

int32_t rtcDisableISR()
{
    return PN_RTC_disableISR(PN_getPnHandle());
}


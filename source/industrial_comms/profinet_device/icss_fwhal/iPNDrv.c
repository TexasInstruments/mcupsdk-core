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

/*
 * pnDrvConfig.h needs to be first first file included in driver!
 * It is application dependent and as such part of the application code.
 * It defines all basic feature options in driver (compile time!)
 */
#include "pnDrvConfig.h"
#include "PN_Handle.h"
#include "PN_HandleDef.h"
#include "iPNDrv.h"
#include "iRtcDrv2.h"
#include "iPNLegacy.h"
#include "iPtcpDrv.h"
#include "iPtcpUtils.h"
#include "PN_ForwardDecisionTable.h"
#include "PN_ReceiveDecisionTable.h"

#include <string.h>
#include <stdint.h>
#include <drivers/hw_include/hw_types.h>

#if defined PROFINET_RGMII_MODE
#include "firmware/rgmii/profinet_irt_pru0_bin.h"
#include "firmware/rgmii/profinet_irt_pru1_bin.h"
#include "firmware/rgmii/firmware_version.h"
#elif defined PROFINET_MII_MODE
#include "firmware/mii/profinet_irt_pru0_bin.h"
#include "firmware/mii/profinet_irt_pru1_bin.h"
#include "firmware/mii/firmware_version.h"
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
* @internal
* @def PRU0_FIRMWARE_NAME
*      name of the C struct in PRU header file. For PRU0
*/
#define PRU0_FIRMWARE_NAME  PRU0_FIRMWARE

/**
* @internal
* @def PRU1_FIRMWARE_NAME
*      name of the C struct in PRU header file. For PRU1
*/
#define PRU1_FIRMWARE_NAME  PRU1_FIRMWARE
/**
* @internal
* @def FILTER_MAC_ID_TYPE1_OCTET0
*      MAC_ID Type 1 Octets
*/
#define FILTER_MAC_ID_TYPE1_OCTET0 0x01
#define FILTER_MAC_ID_TYPE1_OCTET1 0x0E
#define FILTER_MAC_ID_TYPE1_OCTET2 0xCF
#define FILTER_MAC_ID_TYPE1_OCTET3 0x00
#define FILTER_MAC_ID_TYPE1_OCTET4 0x05
#define FILTER_MAC_ID_TYPE1_OCTET5 0xFF

/**
* @internal
* @def FILTER_MAC_ID_TYPE2_OCTET0
*      MAC_ID Type 2 Octets
*/
#define FILTER_MAC_ID_TYPE2_OCTET0 0x01
#define FILTER_MAC_ID_TYPE2_OCTET1 0x15
#define FILTER_MAC_ID_TYPE2_OCTET2 0x4E
#define FILTER_MAC_ID_TYPE2_OCTET3 0x00
#define FILTER_MAC_ID_TYPE2_OCTET4 0x00
#define FILTER_MAC_ID_TYPE2_OCTET5 0x1F

/**
* @internal
* @def FILTER_MAC_ID_TYPE3_OCTET0
*      MAC_ID Type 3 Octets
*/
#define FILTER_MAC_ID_TYPE3_OCTET0 0x01
#define FILTER_MAC_ID_TYPE3_OCTET1 0x80
#define FILTER_MAC_ID_TYPE3_OCTET2 0xC2
#define FILTER_MAC_ID_TYPE3_OCTET3 0x00
#define FILTER_MAC_ID_TYPE3_OCTET4 0x00
#define FILTER_MAC_ID_TYPE3_OCTET5 0x1F

/*TODO: Review this*/
/**Load Forward table*/
#define STATIC_FORWARD_TABLE (0U)
/**Load receive table*/
#define STATIC_RECEIVE_TABLE (1U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * @internal
 * @brief This API is an internal API which is used by setStaticFilterTable.
 * @param macAddr
 *      Following are the valid ranges of multicast addresses:
 *     (01-0E-CF-00-00-00 TO 01-0E-CF-00-05-FF)
 *     (01-15-4E-00-00-00 TO 01-15-4E-00-00-1F)
 *     (01-80-C2-00-00-00 TO 01-80-C2-00-00-1F)
 * @param filterTableAddr Base Address of static filter table
 * @param enable Set or Clear the receive/forwarding corresponding the passed MAC address in the filter table
 *          0
 *          1
 * @retval 0 on success
 */
int32_t PN_writeFilterTable(const uint8_t *macAddr, uint32_t filterTableAddr,
                            int32_t enable);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */



int32_t PN_initDrv(PN_Handle pnHandle)
{
    uint8_t firmwareLoad_done = FALSE;
    uint8_t *pTemp8;
    uint16_t *pTemp16;
    PRUICSS_Handle pruHandle = pnHandle->pruicssHandle;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruHandle->hwAttrs);

    /* Local copy stored to contain old APIs that don't use handles*/

    /* Storing a local copy of the handle to support legacy APIs */
    PN_setHandle(pnHandle);
    /*
     * set Port MAC Addresses
     */
    /* PORT1 MAC Address*/
    pTemp8 = (uint8_t *)(pruicssHwAttrs->pru1DramBase + P1_MAC_ADDR);
    PN_EmacSocMACAddrGet(pnHandle, PORT1_MAC, pTemp8);

    pTemp8 = (uint8_t *)(pruicssHwAttrs->pru1DramBase + P2_MAC_ADDR);
    PN_EmacSocMACAddrGet(pnHandle, PORT2_MAC, pTemp8);

    pTemp8 = (uint8_t *)(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TIMEVAL1);
    memset((void*)pTemp8, 0, (size_t)24);

    pTemp8 = (uint8_t *)(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP_STATUS_REG);
    *pTemp8 = 0xFE;

    pTemp16 = (uint16_t *)(pruicssHwAttrs->pru0DramBase + PRU0_PHASE_EVENT_OFFSET);
    *pTemp16 = 0x0101;

    /* remaining (internal) phase management code*/
    PN_setCompensationValue(pruicssHwAttrs, 75); /*  75ns default*/

    /* init RTC driver*/
    if(PN_initRtcDrv(pnHandle) != 0)
    {
        DebugP_log("RTC Driver init failed - aborting\n");
        return ERR_DRIVER_INIT_FAIL;
    }

#ifdef PTCP_SUPPORT
    PN_PTCP_init(pnHandle);
#ifdef  ENABLE_LATCH_SUPPORT
    PN_PTCP_latchInit(
        pnHandle);/*Setup latch ISR for absolute time API - needs emac handle*/
#endif
#else
    /*disable sync forwarding*/
    PN_PTCP_configureSyncFwd(pruicssHwAttrs, disable);

    /*disable delay_resp, in rensponse to delay_req from peer*/
    PN_PTCP_configureDelayResp(pruicssHwAttrs, 1, disable);
    PN_PTCP_configureDelayResp(pruicssHwAttrs, 2, disable);

    /*disabling transmission of delay request from the device*/
    /* this gets disabled by disabling ptcpTask*/

    /* enabling Single shot mode for capture register 4/5, i.e. TX PORT1 and TX PORT2*/
    HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CAP_CFG_REG, 0x0001FC30);
#endif
    /* Write the Profinet Static Tables for PORT1*/
    PN_loadStaticTable(pruicssHwAttrs, PN_Forward_Decision_Table, 50,
                       STATIC_FORWARD_TABLE, ICSS_EMAC_PORT_1);
    PN_loadStaticTable(pruicssHwAttrs, PN_Receive_Decision_Table, 50,
                       STATIC_RECEIVE_TABLE, ICSS_EMAC_PORT_1);


    /* Write the Profinet Static Tables for PORT2*/
    PN_loadStaticTable(pruicssHwAttrs, PN_Forward_Decision_Table, 50,
                       STATIC_FORWARD_TABLE, ICSS_EMAC_PORT_2);
    PN_loadStaticTable(pruicssHwAttrs, PN_Receive_Decision_Table, 50,
                       STATIC_RECEIVE_TABLE, ICSS_EMAC_PORT_2);


    /*Load the firmware*/
    PRUICSS_disableCore(pruHandle, ICSS_EMAC_PORT_1 - 1);
    PRUICSS_disableCore(pruHandle, ICSS_EMAC_PORT_2 - 1);

    if(PRUICSS_writeMemory(pruHandle, PRUICSS_IRAM_PRU(0) , 0,
                              (uint32_t *) PRU0_FIRMWARE_NAME,
                              sizeof(PRU0_FIRMWARE_NAME)))
    {
        if(PRUICSS_writeMemory(pruHandle, PRUICSS_IRAM_PRU(1) , 0,
                                  (uint32_t *) PRU1_FIRMWARE_NAME,
                                  sizeof(PRU1_FIRMWARE_NAME)))
        {
            firmwareLoad_done = TRUE;
        }
    }

    if(!firmwareLoad_done)
    {
        DebugP_log("PRU Loading failed - aborting\n");
        return ERR_FIRMWARE_LOAD_FAIL;
    }

    PRUICSS_enableCore(pruHandle, ICSS_EMAC_PORT_1 - 1);
    PRUICSS_enableCore(pruHandle, ICSS_EMAC_PORT_2 - 1);

    if(HW_RD_REG32(((PRUICSS_HwAttrs *)(pruHandle->hwAttrs))->baseAddr + PRUICSS_DATARAM(
                 0)) != ICSS_FIRMWARE_RELEASE_1)     /*hard coded version number address!*/
    {
        DebugP_log("PRU firmware bad - aborting\n");
        return ERR_FIRMWARE_VERSION_BAD;
    }

    else
    {
        if(HW_RD_REG32(((PRUICSS_HwAttrs *)(pruHandle->hwAttrs))->baseAddr + PRUICSS_DATARAM(
                     0) + 4) != ICSS_FIRMWARE_RELEASE_2)
        {
            DebugP_log("PRU firmware bad - aborting\n");
            return ERR_FIRMWARE_VERSION_BAD;
        }
    }

    /*By default MRP Ports are in FORWARDING Mode*/
    PN_MRP_setPortState(pruicssHwAttrs, ICSS_EMAC_PORT_1, FORWARDING);
    PN_MRP_setPortState(pruicssHwAttrs, ICSS_EMAC_PORT_2, FORWARDING);
#ifdef WATCHDOG_SUPPORT
    PN_setWatchDogTimer(pnHandle,
                        watchDogExpireDuration); /*ICSS WatchDog Expiry duration */
#endif

    return 0;
}

void PN_getFirmwareVersion(uint32_t *version_major, uint32_t *version_minor,
                           uint32_t *version_build, uint32_t *version_release_type)
{
    *version_major = FIRMWARE_VERSION_MAJOR;
    *version_minor = FIRMWARE_VERSION_MINOR;
    *version_build = FIRMWARE_VERSION_BUILD;
    *version_release_type = FIRMWARE_RELEASE_TYPE;
}

void PN_getFirmwareReleaseInfoAndFeatures(uint32_t *firmware_release_1,
                                          uint32_t *firmware_release_2,
                                          uint32_t *firmware_feature_mask)
{
    *firmware_release_1 = ICSS_FIRMWARE_RELEASE_1;
    *firmware_release_2 = ICSS_FIRMWARE_RELEASE_2;
    *firmware_feature_mask = ICSS_FIRMWARE_FEATURE_MASK;
}

int32_t PN_setDcpFilterStationName(PRUICSS_HwAttrs const *pruicssHwAttrs, const uint8_t *dcpNameOfStation,
                                   uint8_t lengthOfStationName)
{
    int i = 0;
    int j = 0;
    int numOfCharInStationName = 0;

    if(lengthOfStationName > 240)
    {
        return ERR_STATION_NAME_LENGTH;
    }

    numOfCharInStationName = lengthOfStationName;

    if(lengthOfStationName > DCP_NAME_CMP_NO_OF_CHAR)
    {
        j = (lengthOfStationName - DCP_NAME_CMP_NO_OF_CHAR);
        numOfCharInStationName = DCP_NAME_CMP_NO_OF_CHAR;
    }

    /* Load the DCP station name*/
    for(i = 0; i < numOfCharInStationName; i++)
    {
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + PN_DCPF_NAME_OFFSET + i, dcpNameOfStation[j++]);
    }

    /* Length byte is after the DCP_NAME_CMP_NO_OF_CHAR nunber of characters in the memory*/
    HW_WR_REG8(pruicssHwAttrs->pru0DramBase + PN_DCPF_NAME_LENGTH_OFFSET, lengthOfStationName);

    return 0;
}

/*
    This function programs the Profile. In a profile the SoG time for Rx and Tx is set for a port.
*/
int32_t PN_setProfile(PRUICSS_HwAttrs const *pruicssHwAttrs,
                      int32_t portNumber, int32_t profileNumber, int32_t rxSoGValue,
                      int32_t txSoGValue)
{
    int32_t port1ProfileBaseAddr = RTC_GREEN_BEGIN_P1_1_RX_OFFSET;
    int32_t port2ProfileBaseAddr = RTC_GREEN_BEGIN_P2_1_RX_OFFSET;
    int32_t profile_base_addr = 0;

    if(profileNumber == 0 || profileNumber > 5)
    {
        return -1;    /* out of range*/
    }
    /*TBD .. param checking for errors*/
    if(portNumber == ICSS_EMAC_PORT_1)
    {
        profile_base_addr = port1ProfileBaseAddr + (profileNumber - 1) * 8;
    }
    else
    {
        profile_base_addr = port2ProfileBaseAddr + (profileNumber - 1) * 8;
    }

    HW_WR_REG32(pruicssHwAttrs->pru0DramBase + profile_base_addr, rxSoGValue);
    HW_WR_REG32(pruicssHwAttrs->pru0DramBase + profile_base_addr + 4,txSoGValue);

    return 0;
}

/*
  This funtion does the mapping for Phase to Profile for SoG
*/
int32_t PN_mapPhaseToProfile(PRUICSS_HwAttrs const *pruicssHwAttrs, int32_t portNumber, int32_t phaseNumber,
                             int32_t profileNumberRx, int32_t profileNumberTx)
{
    int32_t phaseToProfileBaseAddr = 0;
    int32_t portRxProfileOffset = 0;
    int32_t portTxProfileOffset = 0;

    if(profileNumberRx == 0 || profileNumberRx > 5)
    {
        return -1;    /* out of range*/
    }

    if(profileNumberTx == 0 || profileNumberTx > 5)
    {
        return -2;
    }

    if(phaseNumber == 0 || phaseNumber > 16)
    {
        return -3;    /* out of range*/
    }

    phaseToProfileBaseAddr = RTC_PAHSE_MAPPING_OFFSET + (phaseNumber - 1) *
                             4; /* per phase four bytes are allocated.Two bytes for each port, one for Rx and second one for Tx.*/

    if(portNumber == ICSS_EMAC_PORT_2)
    {
        phaseToProfileBaseAddr = phaseToProfileBaseAddr + 2;
    }

    /* Compute the offset of Profile Rx and Tx SoG registers for Port1*/
    if(portNumber == ICSS_EMAC_PORT_1)
    {
        portRxProfileOffset = RTC_GREEN_BEGIN_P1_1_RX_OFFSET + (profileNumberRx - 1) *
                              8;
        portTxProfileOffset = RTC_GREEN_BEGIN_P1_1_TX_OFFSET + (profileNumberTx - 1) *
                              8;

    }

    else
    {
        /* Compute the offset of Profile Rx and Tx SoG registers for Port2*/
        portRxProfileOffset = RTC_GREEN_BEGIN_P2_1_RX_OFFSET + (profileNumberRx - 1) *
                              8;
        portTxProfileOffset = RTC_GREEN_BEGIN_P2_1_TX_OFFSET + (profileNumberTx - 1) *
                              8;
    }

    HW_WR_REG8(pruicssHwAttrs->pru0DramBase + phaseToProfileBaseAddr, portRxProfileOffset);
    HW_WR_REG8(pruicssHwAttrs->pru0DramBase + phaseToProfileBaseAddr + 1, portTxProfileOffset);

    return 0;
}

int32_t PN_setYellowPeriod(PRUICSS_HwAttrs const *pruicssHwAttrs, int32_t yellowPeriodTime)
{
    HW_WR_REG32(pruicssHwAttrs->pru0DramBase + RTC_IRT_YELLOW_TIME_OFFSET, yellowPeriodTime);
    return 0;

}

int32_t PN_setMaxBridgeDelay(PRUICSS_HwAttrs const *pruicssHwAttrs, int32_t maxBridgeDelayValue)
{
    HW_WR_REG32(pruicssHwAttrs->pru0DramBase + MAXBRIDGE_DELAY_OFFSET, maxBridgeDelayValue);
    return 0;
}

int32_t PN_setMaxLineRxDelay(PRUICSS_HwAttrs const *pruicssHwAttrs, uint8_t portNumber, int32_t maxLineRxDelayValue)
{
    if(portNumber == ICSS_EMAC_PORT_1)
    {
        HW_WR_REG32(pruicssHwAttrs->pru0DramBase + MAXLINE_RXDELAY_P1_OFFSET, maxLineRxDelayValue);
    }
    else
    {
        HW_WR_REG32(pruicssHwAttrs->pru0DramBase + MAXLINE_RXDELAY_P2_OFFSET, maxLineRxDelayValue);
    }

    return 0;
}

void PN_setCompensationValue(PRUICSS_HwAttrs const *pruicssHwAttrs, uint16_t compensationValue)
{
    HW_WR_REG32(pruicssHwAttrs->pru0DramBase + COMPENSATION_OFFSET, compensationValue);
    return;
}

int32_t PN_setRedGuard(PRUICSS_HwAttrs const *pruicssHwAttrs,
                       uint16_t validLowerFrameId, uint16_t validUpperFrameId)
{

    uint16_t tempFrameId = validLowerFrameId;
    uint16_t tempFrameIdLowerByte = (tempFrameId >> 8);
    uint16_t tempFrameIdHigherByte = (tempFrameId << 8);
    tempFrameId = (tempFrameIdLowerByte | tempFrameIdHigherByte);

    HW_WR_REG16(pruicssHwAttrs->pru0DramBase + RTC3_SOF_RedFrameID_OFFSET, tempFrameId);

    tempFrameId = validUpperFrameId;
    tempFrameIdLowerByte = (tempFrameId >> 8);
    tempFrameIdHigherByte = (tempFrameId << 8);
    tempFrameId = (tempFrameIdLowerByte | tempFrameIdHigherByte);
    HW_WR_REG16(pruicssHwAttrs->pru0DramBase + RTC3_EOF_RedFrameID_OFFSET, tempFrameId);

    return 0;
}

int32_t PN_setRtc3PortStatus(PRUICSS_HwAttrs const *pruicssHwAttrs, uint8_t portNumber, uint8_t status)
{

    if((status != OFF) && (status != UP) && (status != RUN))
    {
        return -1;
    }

    if(portNumber == ICSS_EMAC_PORT_1)
    {
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + PORT1_STATUS_OFFSET, status);
    }

    else
    {
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + PORT2_STATUS_OFFSET, status);
    }

    return 0;
}
int32_t PN_MRP_setPortState(PRUICSS_HwAttrs const *pruicssHwAttrs, uint8_t portNumber, uint8_t pState)
{
    if((pState != DISABLED) && (pState != BLOCKING) && (pState != FORWARDING))
    {
        return -1;
    }

    if(portNumber == ICSS_EMAC_PORT_1)
    {
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + MRP_PORT1_STATE_OFFSET, pState);
    }

    else
    {
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + MRP_PORT2_STATE_OFFSET, pState);
    }

    return 0;
}

int32_t PN_MRP_getPortState(PRUICSS_HwAttrs const *pruicssHwAttrs, uint8_t portNumber, uint8_t *pState)
{
    if(0 == pState)
    {
        return -1;
    }

    if(portNumber == ICSS_EMAC_PORT_1)
    {
        *pState = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + MRP_PORT1_STATE_OFFSET);
    }

    else
    {
        *pState = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + MRP_PORT2_STATE_OFFSET);
    }

    return 0;
}

int32_t PN_setStaticFilterTable(PRUICSS_HwAttrs const *pruicssHwAttrs, const uint8_t *macAddr, uint8_t ctrl,
                                uint8_t portNumber, uint8_t table)
{

    uint32_t rcvTableAddr = 0;
    uint32_t fwdTableAddr = 0;

    if(table == BLOCKING)
    {
        rcvTableAddr = pruicssHwAttrs->pru0DramBase +
                       BLOCKING_STATIC_MAC_TABLE_RCV;
        fwdTableAddr = pruicssHwAttrs->pru0DramBase +
                       BLOCKING_STATIC_MAC_TABLE_FWD;
    }

    else if(table == FORWARDING)
    {
        if(portNumber == ICSS_EMAC_PORT_1)
        {
            rcvTableAddr = pruicssHwAttrs->sharedDramBase +
                           STATIC_MAC_TABLE_RCV_PORT1;
            fwdTableAddr = pruicssHwAttrs->sharedDramBase +
                           STATIC_MAC_TABLE_FWD_PORT1;
        }

        else
        {

            rcvTableAddr = pruicssHwAttrs->sharedDramBase +
                           STATIC_MAC_TABLE_RCV_PORT2;
            fwdTableAddr = pruicssHwAttrs->sharedDramBase +
                           STATIC_MAC_TABLE_FWD_PORT2;

        }
    }

    else
    {
        return -1;
    }

    switch(ctrl)
    {
        case NO_RCV_NO_FWD:
            PN_writeFilterTable(macAddr, rcvTableAddr, 0);
            PN_writeFilterTable(macAddr, fwdTableAddr, 0);
            break;

        case RCV_NO_FWD:
            PN_writeFilterTable(macAddr, rcvTableAddr, 1);
            PN_writeFilterTable(macAddr, fwdTableAddr, 0);
            break;

        case NO_RCV_FWD:
            PN_writeFilterTable(macAddr, rcvTableAddr, 0);
            PN_writeFilterTable(macAddr, fwdTableAddr, 1);
            break;

        case RCV_FWD:
            PN_writeFilterTable(macAddr, rcvTableAddr, 1);
            PN_writeFilterTable(macAddr, fwdTableAddr, 1);
            break;

        default:
            return -2;

    }

    return 0;
}

int32_t PN_writeFilterTable(const uint8_t *macAddr, uint32_t filterTableAddr,
                            int32_t enable)
{
    uint16_t    filterTableRowAddr = 0;
    uint16_t    tempRowAddr = 0;
    uint32_t  filterTableRowValue = 0;
    uint32_t  filterBitValue = 1;

    /* Check if MAC Address falls within range 0x010ECF000000 .. 0x010ECF05FF*/
    if((macAddr[0] == FILTER_MAC_ID_TYPE1_OCTET0)
            && (macAddr[1] == FILTER_MAC_ID_TYPE1_OCTET1) &&
            (macAddr[2] == FILTER_MAC_ID_TYPE1_OCTET2)
            && (macAddr[3] == FILTER_MAC_ID_TYPE1_OCTET3) &&
            (macAddr[4] <= FILTER_MAC_ID_TYPE1_OCTET4)
            && (macAddr[5] <= FILTER_MAC_ID_TYPE1_OCTET5))
    {
        filterTableRowAddr =  macAddr[4] << 3;
        tempRowAddr = macAddr[5] & 0xF0;
        tempRowAddr = tempRowAddr >> 5;
        filterTableRowAddr = filterTableRowAddr + tempRowAddr;
        filterTableRowAddr = filterTableRowAddr << 2;
        filterTableRowValue = HW_RD_REG32(filterTableAddr + filterTableRowAddr);
        tempRowAddr = macAddr[5] & 0x1F;
        filterBitValue = filterBitValue << tempRowAddr;

        if(enable == 0)
        {
            filterTableRowValue = filterTableRowValue & (~filterBitValue);
        }

        else
        {
            filterTableRowValue = filterTableRowValue | filterBitValue;
        }

        HW_WR_REG32(filterTableAddr + filterTableRowAddr, filterTableRowValue);
    }

    else
    {
        /*Check if MAC Address falls within range 0x01154E000000 .. 0x01154E00001F*/
        if((macAddr[0] == FILTER_MAC_ID_TYPE2_OCTET0)
                && (macAddr[1] == FILTER_MAC_ID_TYPE2_OCTET1) &&
                (macAddr[2] == FILTER_MAC_ID_TYPE2_OCTET2)
                && (macAddr[3] == FILTER_MAC_ID_TYPE2_OCTET3) &&
                (macAddr[4] == FILTER_MAC_ID_TYPE2_OCTET4)
                && (macAddr[5] <= FILTER_MAC_ID_TYPE2_OCTET5))
        {
            filterTableRowValue = HW_RD_REG32(filterTableAddr + 0xC0);
            tempRowAddr = macAddr[5] & 0x1F;
            filterBitValue = filterBitValue << tempRowAddr;

            if(enable == 0)
            {
                filterTableRowValue = filterTableRowValue & (~filterBitValue);
            }

            else
            {
                filterTableRowValue = filterTableRowValue | filterBitValue;
            }

            HW_WR_REG32(filterTableAddr + 0xC0, filterTableRowValue);
        }

        /* Check if MAC Address falls within range 0x0180C2000000 .. 0x0180C200001F*/
        else if((macAddr[0] == FILTER_MAC_ID_TYPE3_OCTET0)
                && (macAddr[1] == FILTER_MAC_ID_TYPE3_OCTET1) &&
                (macAddr[2] == FILTER_MAC_ID_TYPE3_OCTET2)
                && (macAddr[3] == FILTER_MAC_ID_TYPE3_OCTET3) &&
                (macAddr[4] == FILTER_MAC_ID_TYPE3_OCTET4)
                && (macAddr[5] <= FILTER_MAC_ID_TYPE3_OCTET5))
        {
            filterTableRowValue = HW_RD_REG32(filterTableAddr + 0xC4);
            tempRowAddr = macAddr[5] & 0x1F;
            filterBitValue = filterBitValue << tempRowAddr;

            if(enable == 0)
            {
                filterTableRowValue = filterTableRowValue & (~filterBitValue);
            }

            else
            {
                filterTableRowValue = filterTableRowValue | filterBitValue;
            }

            HW_WR_REG32(filterTableAddr + 0xC4, filterTableRowValue);
        }

        else
        {
            /*MAC Address is out of valid range*/
            return -1;
        }
    }

    return 0;
}

/*
    This function programs ICSS Watchdog Timer to detect if Application is alive or not.
*/
#ifdef WATCHDOG_SUPPORT
int32_t PN_setWatchDogTimer(PN_Handle pnHandle
                            , int32_t timerPeriod)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((pnHandle->pruicssHandle)->hwAttrs);

    if((timerPeriod < 2) || (timerPeriod > 6553))
    {
        return -1;    /* out of range*/
    }

    HW_WR_REG32((pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_PD_WD_TIM_REG), 10 * timerPeriod);
    HW_WR_REG32((pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_WD_CTRL_REG), 1);

    pnHandle->icssWachDogTimerPeriod = 10 * timerPeriod;
    pnHandle->icssWatchDogEnabled = 1;

    return 0;
}
#endif


int32_t PN_loadStaticTable(PRUICSS_HwAttrs const *pruicssHwAttrs,
                           const uint32_t *staticTable,
                           uint8_t staticTableLength,
                           uint8_t staticTableType,
                           uint8_t portNumber)
{
    int32_t i;
    int32_t staticTableOffset;

    uint32_t temp_addr = 0U;
    int32_t ret_val = 0;

    if(portNumber == (uint8_t)ICSS_EMAC_PORT_1)
    {
        if(staticTableType == STATIC_FORWARD_TABLE)
        {
            staticTableOffset = (int32_t)STATIC_MAC_TABLE_FWD_PORT1;
        }

        else if(staticTableType == STATIC_RECEIVE_TABLE)
        {
            staticTableOffset = (int32_t)STATIC_MAC_TABLE_RCV_PORT1;
        }

        else
        {
            ret_val = -1;
        }
    }

    else if(portNumber == (uint8_t)ICSS_EMAC_PORT_2)
    {
        if(staticTableType == STATIC_FORWARD_TABLE)
        {
            staticTableOffset = (int32_t)STATIC_MAC_TABLE_FWD_PORT2;
        }

        else if(staticTableType == STATIC_RECEIVE_TABLE)
        {
            staticTableOffset = (int32_t)STATIC_MAC_TABLE_RCV_PORT2;
        }

        else
        {
            ret_val = -1;
        }
    }

    else
    {
        ret_val = -1;
    }

    if(ret_val == 0)
    {
        for(i = 0; i < (int32_t)staticTableLength; i++)
        {
            temp_addr = (pruicssHwAttrs->sharedDramBase + ((uint32_t)(
                             staticTableOffset)) + (((uint32_t)i) << 2U));
            HW_WR_REG32(temp_addr, staticTable[i]);
        }
    }

    return ret_val;
}

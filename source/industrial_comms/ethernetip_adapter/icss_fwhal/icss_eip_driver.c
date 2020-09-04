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

#include <string.h>
#include "icss_dlr.h"
#include "icss_eip_driver.h"
#include "icss_eip_mcFltr.h"
#include <drivers/pruicss.h>
#include <networking/icss_timesync/icss_timeSync_init.h>

#if defined ETHERNETIP_RGMII_MODE
#include "firmware/rgmii/ethernetip_adapter_pru0_bin.h"
#include "firmware/rgmii/ethernetip_adapter_pru1_bin.h"
#elif defined ETHERNETIP_MII_MODE
#include "firmware/mii/ethernetip_adapter_pru0_bin.h"
#include "firmware/mii/ethernetip_adapter_pru1_bin.h"
#endif

/** name of the C struct in PRU header file*/
#define PRU0_FIRMWARE_NAME      PRU0_FIRMWARE
#define PRU1_FIRMWARE_NAME      PRU1_FIRMWARE

/**PTP MAC ID for comparison*/
uint8_t ptpMAC[6] = {0x1, 0x0, 0x5e, 0x0, 0x1, 0x81};
/**DLR MAC ID for comparison*/
uint8_t dlrMAC[6] = {0x1, 0x21, 0x6c, 0x0, 0x0, 0x2};

/**Product Description for CIP Sync. Reads Texas Instruments:AM3359 in unicode*/
uint8_t productDesc[] = {84, 101, 120, 97, 115, 32, 73, 110, 115, 116, 114, 117, 109, 101,   \
                         110, 116, 115, 59, 65, 77, 51, 51, 53, 57
                        };
/**Product Revision Description for CIP Sync*/
uint8_t revDesc[] = {49, 59, 49, 59, 49};

/**Used to initialize Port Profile identity*/
uint8_t portProfIdentity[] = {0x1, 0x21, 0x6c, 0x0, 0x1, 0x0};

/**Manufacturers Identity*/
uint8_t manufacturerIdentity[] = {0xC4, 0xED, 0xBA};

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */



/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/**
 *  @brief  Initialization routine for Ethernet/IP driver functions
 *
 *          This API does the following functionalities:
 *              Loads the firmware on PRU0 and PRU1 cores
 *              Call DLR init API
 *              Call TimeSync(PTP) init API
 *
 *  @param  icssEipHandle [in] EIP handle
 *
 *  @retval  none
 *
 */
void EIP_drvInit(EIP_Handle icssEipHandle)
{
    uint8_t firmwareLoad_done = 0;

    /*Load the firmware*/
    PRUICSS_disableCore(icssEipHandle->pruicssHandle, ICSS_EMAC_PORT_1 - 1);
    PRUICSS_disableCore(icssEipHandle->pruicssHandle, ICSS_EMAC_PORT_2 - 1);

    if(PRUICSS_writeMemory(icssEipHandle->pruicssHandle, PRUICSS_IRAM_PRU(0) , 0,
                              (uint32_t *) PRU0_FIRMWARE_NAME,
                              sizeof(PRU0_FIRMWARE_NAME)))
    {
        if(PRUICSS_writeMemory(icssEipHandle->pruicssHandle, PRUICSS_IRAM_PRU(1) , 0,
                                  (uint32_t *) PRU1_FIRMWARE_NAME,
                                  sizeof(PRU1_FIRMWARE_NAME)))
        {
            firmwareLoad_done = TRUE;
        }
    }

    if(firmwareLoad_done)
    {
        PRUICSS_enableCore(icssEipHandle->pruicssHandle, ICSS_EMAC_PORT_1 - 1);
        PRUICSS_enableCore(icssEipHandle->pruicssHandle, ICSS_EMAC_PORT_2 - 1);
    }

    /*TODO: Review this. This needs to be done before ICSS_EMAC_open*/
    /*Packet processing callback*/
    // ((((ICSS_EmacObject *)
    //    icssEmacHandle->object)->callBackHandle)->rxRTCallBack)->callBack =
    //        (ICSS_EmacCallBack)EIP_processProtocolFrames;
    // ((((ICSS_EmacObject *)
    //    icssEmacHandle->object)->callBackHandle)->rxRTCallBack)->userArg =
    //        icssEipHandle;

    icssEipHandle->dlrHandle->pruicssHandle = icssEipHandle->pruicssHandle;
    icssEipHandle->dlrHandle->emacHandle = icssEipHandle->emacHandle;
    /*Initialize DLR Engine*/
    EIP_DLR_init(icssEipHandle->dlrHandle);

    /*Initialise Multicast filter*/
    /*TODO: Review this*/
    // eip_multicast_filter_init(icssEmacHandle);
    eip_multicast_filter_init(icssEipHandle->pruicssHandle);

    icssEipHandle->timeSyncHandle->pruicssHandle = icssEipHandle->pruicssHandle;
    icssEipHandle->timeSyncHandle->emacHandle = icssEipHandle->emacHandle;
    /*Initialize PTP Handle*/
    TimeSync_drvInit(icssEipHandle->timeSyncHandle);

    /*In case of EIP the TimeSync_addIP() function
     * is called before TimeSync_drvInit() resulting in IP address
     * getting overwritten. This is the fix for it
     */
    TimeSync_addIP(icssEipHandle->timeSyncHandle,
                   icssEipHandle->timeSyncHandle->udpParams.srcIP);
}

/**
 *  @brief  EIP driver start API
 *
 *          This API starts DLR and enables PTP
 *
 *  @param  icssEipHandle [in] EIP handle
 *
 *  @retval  none
 *
 */
void EIP_drvStart(EIP_Handle icssEipHandle)
{
    /*Start DLR Engine*/
    EIP_DLR_start(icssEipHandle->dlrHandle);

    /*Start PTP Engine*/
    TimeSync_drvEnable(icssEipHandle->timeSyncHandle);
}
/**
 *  @brief  EIP driver stop API
 *
 *          This API stops DLR and disables PTP
 *
 *  @param  icssEipHandle [in] EIP handle
 *
 *  @retval  none
 *
 */
void EIP_drvStop(EIP_Handle icssEipHandle)
{

    /*Halt DLR Engine*/
    EIP_DLR_stop(icssEipHandle->dlrHandle);

    /*Halt PTP Engine*/
    TimeSync_drvDisable(icssEipHandle->timeSyncHandle);
}

/**
 *  @brief  API to process the real time Packets
 *
 *          This API will be registered as Real Tme Rx Call back. Incase of EIP, the driver
 *          receives DLR and PTP packets. This API receives the packet, checks the packet
 *          type and passes the packet to DLR driver or PTP driver
 *
 *  @param  queue_number [in] Queue where the packet is present
 *  @param  userArg      [in] userArgumment. EIP handle
 *
 *  @retval  none
 *
 */
/*TODO: Review this function*/
void EIP_processProtocolFrames(uint32_t *queue_number, void *userArg)
{

    int32_t retVal = 0;
    uint16_t size;
    ICSS_EMAC_RxArgument rxArg;

    EIP_Handle icssEipHandle = (EIP_Handle)userArg;
    ICSS_EMAC_Handle eipIcssEmacHandle = icssEipHandle->emacHandle;

    uint8_t *dstMacId = icssEipHandle->tempFrame;

    rxArg.icssEmacHandle = eipIcssEmacHandle;
    rxArg.destAddress = (uint32_t)(icssEipHandle->tempFrame);
    rxArg.more = 0;
    rxArg.queueNumber = *queue_number;
    rxArg.port = 0;

    retVal = ICSS_EMAC_rxPktGet(&rxArg, NULL);

    if(retVal != SystemP_FAILURE)
    {
        size = (uint16_t)retVal;
        /*Compare Destination MAC ID and determine if this is a DLR packet*/
        if((memcmp((void *)dstMacId, (void *)dlrMAC, 6U) == 0))
        {
            EIP_DLR_processDLRFrame(icssEipHandle->dlrHandle, icssEipHandle->tempFrame,
                                    rxArg.port - 1, size);
        }
        /*Compare Destination MAC ID and determine if this is a PTP packet*/
        else if((memcmp((void *)dstMacId, (void *)ptpMAC, 6U) == 0) &&
                (rxArg.port >= ICSS_EMAC_PORT_1) &&
                (rxArg.port <= ICSS_EMAC_PORT_2))
        {
            /*Link local field doesn't matter in case of EIP*/
            TimeSync_processPTPFrame(icssEipHandle->timeSyncHandle,
                                    icssEipHandle->tempFrame, rxArg.port, size, FALSE);
        }
    }
}
/**
 *  @brief  API to initialize the CIP Sync objects in the EIP handle
 *          Initialize CIP Sync member variables based on PTP implementation
 *
 *  @param  icssEipHandle [in] EIP handle
 *
 *  @retval  0  - On success
 *           <0 - On failure
 *
 */
int8_t EIP_initializeCIPSync(EIP_Handle icssEipHandle)
{
    int8_t retVal = 0;
    int8_t i;
    TimeSync_Config_t *timeSyncConfig = &
                                        (icssEipHandle->timeSyncHandle->timeSyncConfig);

    /*The section below is applicable to both OC and TC*/
    for(i = 0; i < 8; i++)
    {
        icssEipHandle->cipSyncObj.localClkInfo.clockIdentity[i] =
            timeSyncConfig->clockIdentity[i];
    }

    icssEipHandle->cipSyncObj.portEnable[0] = TRUE;
    icssEipHandle->cipSyncObj.portEnable[1] = TRUE;

    /*Add OUI of Texas Instruments*/
    memcpy(icssEipHandle->cipSyncObj.manufacturerIdentity, manufacturerIdentity,
           3);

    /*Unicode for Product Description*/
    icssEipHandle->cipSyncObj.productType.size = sizeof(productDesc);
    memcpy(icssEipHandle->cipSyncObj.productType.descr, productDesc,
           icssEipHandle->cipSyncObj.productType.size);

    /*Unicode for Firmware and Software Revision*/
    icssEipHandle->cipSyncObj.revData.size = sizeof(revDesc);
    memcpy(icssEipHandle->cipSyncObj.revData.descr, revDesc,
           icssEipHandle->cipSyncObj.revData.size);

    /*The unicode for User Description should be filled by application*/

    /*Based on CIP Sync Recommendations*/
    memcpy(icssEipHandle->cipSyncObj.profileInfo[0].portProfileIdentity,
           portProfIdentity, 6);

    /*The section below is only applicable to Ordinary Clocks*/
    if(timeSyncConfig->config == OC)
    {

        icssEipHandle->cipSyncObj.clockType = ORDINARY_CLOCK;

        icssEipHandle->cipSyncObj.ifPTPEnable = 1;
        icssEipHandle->cipSyncObj.IsSynchronized = 0;
        icssEipHandle->cipSyncObj.systemTimeMicrosec = 0;
        icssEipHandle->cipSyncObj.systemTimeNanosec = 0;
        icssEipHandle->cipSyncObj.offsetFromMaster = 0;
        icssEipHandle->cipSyncObj.maxOffsetFromMaster = 0;
        icssEipHandle->cipSyncObj.meanPathDelayToMaster = 0;

        icssEipHandle->cipSyncObj.numberOfPorts = 1;

        icssEipHandle->cipSyncObj.portState[0] = PASSIVE;
        icssEipHandle->cipSyncObj.portState[1] = PASSIVE;

        icssEipHandle->cipSyncObj.portLogAnnounceInterval[0] = 0;
        icssEipHandle->cipSyncObj.portLogAnnounceInterval[1] = 0;

        icssEipHandle->cipSyncObj.portLogSyncInterval[0] = 0;
        icssEipHandle->cipSyncObj.portLogSyncInterval[1] = 0;

        icssEipHandle->cipSyncObj.priority1 = 0;
        icssEipHandle->cipSyncObj.priority2 = 1;

        icssEipHandle->cipSyncObj.domainNumber = 0;
        icssEipHandle->cipSyncObj.stepsRemoved = 0;

        icssEipHandle->cipSyncObj.timeOffset.systemOffset = 0;
        icssEipHandle->cipSyncObj.timeOffset.systemTime = 0;

    }

    return retVal;

}

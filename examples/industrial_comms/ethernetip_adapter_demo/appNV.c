/*!
 *  \example appPerm.c
 *
 *  \brief
 *  EtherNet/IP&trade; Adapter Example Application, handling of permanent data storage.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2021-06-09
 *
 *  \copyright
 *  Copyright (c) 2021, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <string.h>
#include <portmacro.h>
#include <FreeRTOSConfig.h>
#include <ti_board_open_close.h>

#include <api/EI_API.h>
#include <api/EI_API_def.h>
#include <drivers/CUST_drivers.h>
#include "board.h"
#include "appNV.h"

bool EI_APP_PERM_factoryReset(int16_t serviceFlag_p);

static EI_API_ADP_T *pAdp_s = NULL;
static bool resetRequired_s = false;
static bool configChanged_s = false;
static uint32_t resetTime_s = 0;
static int16_t resetServiceFlag_s;

static EI_APP_PERM_SCfgData_t permData_s;
static EI_APP_PERM_SCfgData_t defaultPermData_s =
{
    .ipAddr =       0xc0a8010a,
    .ipNwMask =     0xffffff00,
    .ipGateway =    0xc0a80101,
    .nameServer1 =  0x00000000,
    .nameServer2 =  0x00000000,
    .aDomainName = "",
    .aHostName = "",
    .configurationMethod = EIP_eCFGMETHOD_STATIC,
    .ttlValue = 1,
    .acdActive = true,
    .aAcdAddr = { 0,0,0,0,0,0 },
    .aAcdHdr = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
    .intfConfig[0].bit.ETHIntfActive = 1,
    .intfConfig[0].bit.ETHIntfAutoNeg = 1,
    .intfConfig[0].bit.ETHIntfFDuplex = 0,
    .intfConfig[0].bit.ETHIntf100MB = 0,
    .intfConfig[1].bit.ETHIntfActive = 1,
    .intfConfig[1].bit.ETHIntfAutoNeg = 1,
    .intfConfig[1].bit.ETHIntfFDuplex = 0,
    .intfConfig[1].bit.ETHIntf100MB = 0,
    .qosParameter.Q_Tag_Enable = EI_API_ADP_DEFAULT_8021Q,
    .qosParameter.DSCP_PTP_Event = EI_API_ADP_DEFAULT_DSCP_PTP_EVENT,
    .qosParameter.DSCP_PTP_General = EI_API_ADP_DEFAULT_DSCP_PTP_GENERAL,
    .qosParameter.DSCP_Urgent = EI_API_ADP_DEFAULT_DSCP_URGENT,
    .qosParameter.DSCP_Scheduled = EI_API_ADP_DEFAULT_DSCP_SCHEDULED,
    .qosParameter.DSCP_High = EI_API_ADP_DEFAULT_DSCP_HIGH,
    .qosParameter.DSCP_Low = EI_API_ADP_DEFAULT_DSCP_LOW,
    .qosParameter.DSCP_Explicit = EI_API_ADP_DEFAULT_DSCP_EXPLICIT,

    .encapInactTimeout = 120,
    .mcastConfig.allocControl = 0,

#if defined(TIME_SYNC)
    .ptpEnable                  = 1,
    .portEnable                 = 1,
    .portLogAnnounceInterval    = 1,
    .portLogSyncInterval        = 0,
    .domainNumber               = 0,
    .aUserDescription           = {"Kunbus;EthernetIP Adapter;"},
#else
    .ptpEnable                  = 0,
    .portEnable                 = 1,
    .portLogAnnounceInterval    = 1,
    .portLogSyncInterval        = 0,
    .domainNumber               = 0,
    .aUserDescription           = {"Kunbus;EthernetIP Adapter;"},
#endif

#if defined(QUICK_CONNECT)
    .quickConnectEnabled       = true,
#else
    .quickConnectEnabled       = false,
#endif
    .lldpParameter.enableArrayLength   = 3,
    .lldpParameter.enableArray.allBits = 7,
    .lldpParameter.msgTxInterval       = 30,
    .lldpParameter.msgTxHold           = 4,
};

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Initialize handle to permanent data in Flash memory.
 *
 *  \details
 *  Initialize handle to permanent data in Flash memory.
 *
 */
bool EI_APP_PERM_init(EI_API_ADP_T *pAdpNode_p)
{
    pAdp_s = pAdpNode_p;

    return (true);
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Write the permanent data.
 *
 *  \details
 *  Write the permanent data to memory. Operation can be blocking or non-blocking.
 *
 */
bool EI_APP_PERM_write(bool blocking_p)
{
    void*    prmHandle = NULL;
    uint32_t err       = CUST_DRIVERS_eERR_NOERROR;

    CUST_DRIVERS_PRM_EType_t memType = PERMANENT_DATA_MEMORY_TYPE;

    if (memType == CUST_DRIVERS_PRM_eTYPE_UNDEFINED)
    {
        return false;
    }

    permData_s.permHdr.magicNumber = (('M' << 8) | 'R');
    permData_s.permHdr.version     = APP_PERM_DATA_VERSION;
    permData_s.permHdr.checksum    = 0;

    prmHandle = CUST_DRIVERS_PRM_getHandle(PERMANENT_DATA_MEMORY_TYPE, PERMANENT_DATA_MEMORY_INSTANCE);

    err = CUST_DRIVERS_PRM_write (prmHandle,
                                  PERMANENT_DATA_MEMORY_TYPE,
                                  PERMANENT_DATA_MEMORY_OFFSET,
                                  (uint8_t*)&permData_s,
                                  sizeof(EI_APP_PERM_SCfgData_t),
                                  blocking_p);

    if (CUST_DRIVERS_eERR_NOERROR != err)
    {
        return false;
    }

    return true;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Reads the permanent data.
 *
 *  \details
 *  Reads the permanent data. If no data are present,
 *  a new structure is created with default data.
 *
 */
bool EI_APP_PERM_read(void)
{
    void*    prmHandle;
    uint32_t err;
    uint32_t checkSum;
    EIP_SConfigurationControl_t configurationControl;

    CUST_DRIVERS_PRM_EType_t memType = PERMANENT_DATA_MEMORY_TYPE;

    if ( (memType == CUST_DRIVERS_PRM_eTYPE_FLASH) ||
         (memType == CUST_DRIVERS_PRM_eTYPE_EEPROM) )
    {
        prmHandle = CUST_DRIVERS_PRM_getHandle(PERMANENT_DATA_MEMORY_TYPE, PERMANENT_DATA_MEMORY_INSTANCE);

        err = CUST_DRIVERS_PRM_read (prmHandle,
                                     PERMANENT_DATA_MEMORY_TYPE,
                                     PERMANENT_DATA_MEMORY_OFFSET,
                                     (uint8_t *)&permData_s,
                                     sizeof (EI_APP_PERM_SCfgData_t));
    }
    else
    {
        err = CUST_DRIVERS_eERR_NO_PERMANENT_STORAGE;
    }

    if ( (CUST_DRIVERS_eERR_NOERROR              != err) &&
         (CUST_DRIVERS_eERR_NO_PERMANENT_STORAGE != err) )
    {
        OSAL_printf ("\r\nPermanent data read failed");
        goto laError;
    }

    checkSum = 0;  // TBD calculate a proper checksum

    if (   (permData_s.permHdr.magicNumber != (('M' << 8) | 'R'))
        || (permData_s.permHdr.version != APP_PERM_DATA_VERSION)
        || (permData_s.permHdr.checksum != checkSum)
       )
    {
        OSAL_printf ("The data is corrupted, write default values.\r\n");
        EI_APP_PERM_factoryReset(1);
    }

    EI_API_ADP_setHostName(pAdp_s, permData_s.aHostName);
    EI_API_ADP_setIpTTL(pAdp_s, permData_s.ttlValue);
    EI_API_ADP_setACD(pAdp_s, permData_s.acdActive);
    EI_API_ADP_setIntfConfig(pAdp_s, 0, permData_s.intfConfig[0]);
    EI_API_ADP_setIntfConfig(pAdp_s, 1, permData_s.intfConfig[1]);
    EI_API_ADP_setEnipAcdState(pAdp_s, permData_s.acdState);
    EI_API_ADP_SParam_t aAcdAddr = { 6, (uint8_t*)&permData_s.aAcdAddr };
    EI_API_ADP_setEnipAcdAddr(pAdp_s, &aAcdAddr);
    EI_API_ADP_SParam_t aAcdHdr = { 28, (uint8_t*)&permData_s.aAcdHdr };
    EI_API_ADP_setEnipAcdHdr(pAdp_s, &aAcdHdr);
    EI_API_ADP_setEncapInactTimeout(pAdp_s, permData_s.encapInactTimeout);
    EI_API_ADP_setQoS(pAdp_s, &permData_s.qosParameter);
    EI_API_ADP_setMcastConfiguration(pAdp_s, &permData_s.mcastConfig);

    if(permData_s.permHdr.version == APP_PERM_DATA_VERSION)
    {
        EI_API_ADP_setPtpEnable(pAdp_s, permData_s.ptpEnable);
        EI_API_ADP_setPortEnable(pAdp_s, permData_s.portEnable);
        EI_API_ADP_setPortLogAnnounceInterval(pAdp_s, permData_s.portLogAnnounceInterval);
        EI_API_ADP_setPortLogSyncInterval(pAdp_s, permData_s.portLogSyncInterval);
        EI_API_ADP_setDomainNumber(pAdp_s, permData_s.domainNumber);
        EI_API_ADP_setTimeSyncUserDescription(pAdp_s, permData_s.aUserDescription);
    }
    else
    {
        EI_API_ADP_setPtpEnable(pAdp_s, defaultPermData_s.ptpEnable);
        EI_API_ADP_setPortEnable(pAdp_s, defaultPermData_s.portEnable);
        EI_API_ADP_setPortLogAnnounceInterval(pAdp_s, defaultPermData_s.portLogAnnounceInterval);
        EI_API_ADP_setPortLogSyncInterval(pAdp_s, defaultPermData_s.portLogSyncInterval);
        EI_API_ADP_setDomainNumber(pAdp_s, defaultPermData_s.domainNumber);
        EI_API_ADP_setTimeSyncUserDescription(pAdp_s, defaultPermData_s.aUserDescription);
    }

#if defined(QUICK_CONNECT)
    // Enable QuickConnect
    EI_API_ADP_setQuickConnectEnabled(pAdp_s, permData_s.quickConnectEnabled);
#endif

    configurationControl.configurationMethod = permData_s.configurationMethod;
    configurationControl.dnsEnable = 0;
    configurationControl.reserved = 0;
    EI_API_ADP_setIpConfig(pAdp_s, configurationControl, permData_s.ipAddr, permData_s.ipNwMask, permData_s.ipGateway,
                           permData_s.nameServer1, permData_s.nameServer2, permData_s.aDomainName, false);
    EI_API_ADP_setLldpParameter(pAdp_s, permData_s.lldpParameter);
    return true;

    //-------------------------------------------------------------------------------------------------
    laError:

        exit (-1);
}


/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Performs a factory reset.
 *
 *  \details
 *  Performs a factory reset
 *  - With serviceFlag == 1: restore default data
 *  - With serviceFlag == 2: restore default data, except communication link attributes.
 *  - Other values:          do nothing
 *
 */
bool EI_APP_PERM_factoryReset(int16_t serviceFlag_p)
{
    switch (serviceFlag_p)
    {
    case 1:
        // Restore default data.
        OSAL_MEMORY_memcpy(&permData_s, &defaultPermData_s, sizeof(EI_APP_PERM_SCfgData_t));
        break;
    case 2:
        // Restore default data, except communication link attributes, these are:
        // TCP/IP object 0xF5, attributes 3, 5 and 6.
        // Ethernet Link object 0xF6, attribute 6.
        permData_s.ttlValue = defaultPermData_s.ttlValue;
        permData_s.acdActive  = defaultPermData_s.acdActive;
        permData_s.acdState = defaultPermData_s.acdState;

        OSAL_MEMORY_memcpy(permData_s.aAcdAddr, defaultPermData_s.aAcdAddr, sizeof(defaultPermData_s.aAcdAddr));
        OSAL_MEMORY_memcpy(permData_s.aAcdHdr,  defaultPermData_s.aAcdHdr,  sizeof(defaultPermData_s.aAcdHdr));

        permData_s.encapInactTimeout = defaultPermData_s.encapInactTimeout;
        OSAL_MEMORY_memcpy(&permData_s.qosParameter, &defaultPermData_s.qosParameter, sizeof(defaultPermData_s.qosParameter));

        //timeSync attributes
        permData_s.ptpEnable                = defaultPermData_s.ptpEnable;
        permData_s.portEnable               = defaultPermData_s.portEnable;
        permData_s.portLogSyncInterval      = defaultPermData_s.portLogSyncInterval;
        permData_s.portLogAnnounceInterval  = defaultPermData_s.portLogAnnounceInterval;
        permData_s.domainNumber             = defaultPermData_s.domainNumber;
        OSAL_MEMORY_memcpy(permData_s.aUserDescription, defaultPermData_s.aUserDescription, 128);

        OSAL_MEMORY_memcpy(&permData_s.lldpParameter, &defaultPermData_s.lldpParameter, sizeof(defaultPermData_s.lldpParameter));

#if defined(QUICK_CONNECT)
        permData_s.quickConnectEnabled = defaultPermData_s.quickConnectEnabled;
#endif
        break;
    default:
        return false;
    }

    return EI_APP_PERM_write(true);
}


/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback function for write accesses of several attributes.
 *
 *  \details
 *  Callback function for write accesses of several attributes. Saves the new permanent data.
 *  Sets new network configuration, if necessary. Sets aHostName, if necessary.
 */
void EI_APP_PERM_configCb(EI_API_CIP_NODE_T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, EI_API_CIP_ESc_t serviceCode_p, int16_t serviceFlag_p)
{
   static uint32_t lastCalled;

   // Early exit, because we are only interested if Set_Attribute was executed.
   if (serviceCode_p != EI_API_CIP_eSC_SETATTRSINGLE) return;

   if ((OSAL_getMsTick() - lastCalled) < 100)
   {
       OSAL_SCHED_sleep(100);
   }
   lastCalled = OSAL_getMsTick();

   if (classId_p == 0xf5)
   {
       if (attrId_p == 3 || attrId_p == 5)
       {
           bool hwConfigEnabled;
           char aDomainName[49];
           uint32_t ipAddr;                 // IP address
           uint32_t ipNwMask;               // Network mask
           uint32_t ipGateway;              // Gateway address
           uint32_t nameServer1;            // First name server address
           uint32_t nameServer2;            // Second name server address
           EIP_SConfigurationControl_t configurationControl; // TCP/IP object attribute 3

           EI_API_ADP_isHwSettingEnabled(pAdp_s, &hwConfigEnabled);
           if (hwConfigEnabled)
           {
               // Network configuration is controlled by hardware, do nothing.
               return;
           }

           // Attribute 3 (configuration control) or
           // Attribute 5 (interface configuration) is set.
           EI_API_ADP_getIpAddr(pAdp_s, &ipAddr);
           EI_API_ADP_getIpNwMask(pAdp_s, &ipNwMask);
           EI_API_ADP_getIpGateway(pAdp_s, &ipGateway);
           EI_API_ADP_getIpPriNameServer(pAdp_s, &nameServer1);
           EI_API_ADP_getIpSecNameServer(pAdp_s, &nameServer2);
           EI_API_ADP_getDomainName(pAdp_s, aDomainName);
           EI_API_ADP_getConfigurationControl(pAdp_s, &configurationControl);

           if ( (ipAddr      != permData_s.ipAddr)      ||
                (ipNwMask    != permData_s.ipNwMask)    ||
                (ipGateway   != permData_s.ipGateway)   ||
                (nameServer1 != permData_s.nameServer1) ||
                (nameServer2 != permData_s.nameServer2) ||
                (configurationControl.configurationMethod != permData_s.configurationMethod) ||
                (strncmp(aDomainName, permData_s.aDomainName, sizeof(permData_s.aDomainName)) != 0) )
           {
               // Attribute 3 (configuration control) or
               // Attribute 5 (interface configuration) is set.
               permData_s.ipAddr      = ipAddr;
               permData_s.ipNwMask    = ipNwMask;
               permData_s.ipGateway   = ipGateway;
               permData_s.nameServer1 = nameServer1;
               permData_s.nameServer2 = nameServer2;
               permData_s.configurationMethod = configurationControl.configurationMethod;

               strncpy(permData_s.aDomainName, aDomainName, sizeof(permData_s.aDomainName));

               configChanged_s = true;
           }
       }
       else if (attrId_p == 6)
       {
           char aHostName[65];
           EI_API_ADP_getHostName(pAdp_s, aHostName);
           if(0 != strcmp(aHostName, permData_s.aHostName))
           {
               strncpy(permData_s.aHostName, aHostName, sizeof(permData_s.aHostName));
               configChanged_s = true;
           }

       }
       else if (attrId_p == 8)
       {
           uint8_t ttlValue;
           EI_API_ADP_getIpTTL(pAdp_s, &ttlValue);
           if(ttlValue != permData_s.ttlValue)
           {
               permData_s.ttlValue = ttlValue;
               configChanged_s = true;
           }
       }
       else if (attrId_p == 9)
       {
           EI_API_ADP_SMcastConfig_t mcastConfig;
           EI_API_ADP_getMcastConfiguration(pAdp_s, &mcastConfig);

           if(OSAL_MEMORY_memcmp(&mcastConfig, &permData_s.mcastConfig, sizeof(permData_s.mcastConfig)) != 0)
           {
               OSAL_MEMORY_memcpy (&permData_s.mcastConfig, &mcastConfig, sizeof(permData_s.mcastConfig));
               configChanged_s = true;
           }
       }
       else if (attrId_p == 10)
       {
           bool acdActive;
           EI_API_ADP_getACD(pAdp_s, &acdActive);
           if(acdActive != permData_s.acdActive)
           {
               permData_s.acdActive = acdActive;
               configChanged_s = true;
           }
       }
       else if (attrId_p == 11)
       {
           uint8_t acdState;                 // Attribute 11, state of acd
           EI_API_ADP_SParam_t aAcdAddr;
           EI_API_ADP_SParam_t aAcdHdr;

           EI_API_ADP_getEnipAcdState(pAdp_s, &acdState);
           EI_API_ADP_getEnipAcdAddr(pAdp_s, &aAcdAddr);
           EI_API_ADP_getEnipAcdHdr(pAdp_s, &aAcdHdr);

           if(acdState != permData_s.acdState ||
              OSAL_MEMORY_memcmp(aAcdAddr.data, permData_s.aAcdAddr, sizeof(permData_s.aAcdAddr) != 0) ||
              OSAL_MEMORY_memcmp(aAcdHdr.data, permData_s.aAcdHdr, sizeof(permData_s.aAcdHdr) != 0)
             )
           {
               permData_s.acdState = acdState;
               OSAL_MEMORY_memcpy(permData_s.aAcdAddr, aAcdAddr.data, sizeof(permData_s.aAcdAddr));
               OSAL_MEMORY_memcpy(permData_s.aAcdHdr, aAcdHdr.data, sizeof(permData_s.aAcdHdr));
               configChanged_s = true;
           }
       }
#if defined(QUICK_CONNECT)
       else if (attrId_p == 12)
       {
           bool quickConnectEnabled;
           // Enable/Disable QuickConnect
           EI_API_ADP_getQuickConnectEnabled(pAdp_s, &quickConnectEnabled);
           if(quickConnectEnabled != permData_s.quickConnectEnabled)
           {
               permData_s.quickConnectEnabled = quickConnectEnabled;
               configChanged_s = true;
           }
       }
#endif
       else if (attrId_p == 13)
       {
           uint16_t encapInactTimeout;
           EI_API_ADP_getEncapInactTimeout(pAdp_s, &encapInactTimeout);
           if(encapInactTimeout != permData_s.encapInactTimeout)
           {
               permData_s.encapInactTimeout = encapInactTimeout;
               configChanged_s = true;
           }
       }
       else
       {
           // Nothing has changed.
           configChanged_s = false;
       }
   }
   else if (classId_p == 0x43)
   {
       if (attrId_p == 1)
        {
            bool ptpEnable;
            EI_API_ADP_getPtpEnable(pAdp_s, &ptpEnable);
            if(ptpEnable != permData_s.ptpEnable)
            {
                permData_s.ptpEnable = ptpEnable;
                configChanged_s = true;
            }
        }
        else if (attrId_p == 13)
        {
            bool portEnable;
            EI_API_ADP_getPortEnable(pAdp_s, &portEnable);
            if(portEnable != permData_s.portEnable)
            {
                permData_s.portEnable = portEnable;
                configChanged_s = true;
            }
        }
        else if (attrId_p == 14)
        {
            uint16_t portLogAnnounceInterval;
            EI_API_ADP_getPortLogAnnounceInterval(pAdp_s, &portLogAnnounceInterval);
            if(portLogAnnounceInterval != permData_s.portLogAnnounceInterval)
            {
                permData_s.portLogAnnounceInterval = portLogAnnounceInterval;
                configChanged_s = true;
            }
        }
        else if (attrId_p == 15)
        {
            int16_t portLogSyncInterval;
            EI_API_ADP_getPortLogSyncInterval(pAdp_s, &portLogSyncInterval);
            if(portLogSyncInterval != permData_s.portLogSyncInterval)
            {
                permData_s.portLogSyncInterval = portLogSyncInterval;
                configChanged_s = true;
            }
        }
        else if (attrId_p == 18)
        {
            uint8_t domainNumber;

            EI_API_ADP_getDomainNumber(pAdp_s, &domainNumber);
            if(domainNumber != permData_s.domainNumber)
            {
                permData_s.domainNumber = domainNumber;
                configChanged_s = true;
            }
        }
        else if (attrId_p == 23)
        {
            char aUserDescription[128];
            EI_API_ADP_getTimeSyncUserDescription(pAdp_s, aUserDescription);
            if(OSAL_MEMORY_memcmp(aUserDescription, permData_s.aUserDescription, sizeof(permData_s.aUserDescription)) != 0)
            {
                OSAL_MEMORY_memcpy (permData_s.aUserDescription, aUserDescription, sizeof(permData_s.aUserDescription));
                configChanged_s = true;
            }
        }
    }
   else if (classId_p == 0x48)
   {
       EI_API_ADP_SQos_t qos;

       EI_API_ADP_getQoS(pAdp_s, &qos);
       if(OSAL_MEMORY_memcmp(&permData_s.qosParameter, &qos, sizeof(EI_API_ADP_SQos_t)) != 0)
       {
           OSAL_MEMORY_memcpy (&permData_s.qosParameter, &qos, sizeof(EI_API_ADP_SQos_t));
           configChanged_s = true;
       }
   }

   else if (classId_p == 0xf6)
   {
       // Ethernet link object has changed for instance 1 or 2.
       if (instanceId_p == 1)
       {
           EI_API_ADP_UIntfConf_t intfConf;
           EI_API_ADP_getIntfConfig(pAdp_s, 0, &intfConf);
           if(intfConf.all != permData_s.intfConfig[0].all)
           {
               permData_s.intfConfig[0] = intfConf;
               configChanged_s = true;
           }
       }
       else if (instanceId_p == 2)
       {
           EI_API_ADP_UIntfConf_t intfConf;
           EI_API_ADP_getIntfConfig(pAdp_s, 1, &intfConf);
           if(intfConf.all != permData_s.intfConfig[1].all)
           {
               permData_s.intfConfig[1] = intfConf;
               configChanged_s = true;
           }
       }
       else
       {
           // Nothing has changed.
           configChanged_s = false;
       }
   }

   else if (classId_p == 0x0109)
   {
       EI_API_ADP_SLldp_Parameter_t lldpParameter;
       switch (attrId_p)
       {
       case 1:
       case 2:
       case 3:
           EI_API_ADP_getLldpParameter(pAdp_s, &lldpParameter);
           if(OSAL_MEMORY_memcmp(&permData_s.lldpParameter, &lldpParameter, sizeof(EI_API_ADP_SLldp_Parameter_t)) != 0)
           {
               OSAL_MEMORY_memcpy (&permData_s.lldpParameter, &lldpParameter, sizeof(EI_API_ADP_SLldp_Parameter_t));
               configChanged_s = true;
           }
           break;
       default:
           configChanged_s = false;
       }
   }

   else
   {
       // Nothing has changed.
       configChanged_s = false;
   }

}


/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback for reset service of class 0x01.
 *
 *  \details
 *  Callback for reset service of class 0x01. Sets the timestamp for a delayed reset.
 */
void EI_APP_PERM_reset(EI_API_CIP_NODE_T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, EI_API_CIP_ESc_t serviceCode_p, int16_t serviceFlag_p)
{
    if (serviceFlag_p == 1 || serviceFlag_p == 2)
    {
        // Reset with parameter 1 or 2 means factory reset.
        EI_APP_PERM_factoryReset(serviceFlag_p);
    }

    resetTime_s  = OSAL_getMsTick();
    resetRequired_s = true;
    resetServiceFlag_s = serviceFlag_p;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Checks if a reset is required.
 *
 *  \details
 *  Checks if a reset is required and 2 seconds are expired.
 +
 */
int16_t EI_APP_PERM_getResetRequired(void)
{
    uint32_t actTime = OSAL_getMsTick();
    uint32_t difTime = 0;

    if (resetRequired_s)
    {
        // Wait 2 seconds for reset:
        if (actTime < resetTime_s)
        {
            difTime = (0xFFFFFFFF - resetTime_s) + actTime;
        }
        else
        {
            difTime = actTime - resetTime_s;
        }

        if ( (difTime > 2000) &&
             (false  == CUST_DRIVERS_PRM_isWritePending()) )
        {
            resetRequired_s = false;
            return resetServiceFlag_s;
        }
    }
    return -1;
}

bool EI_APP_PERM_getConfigChanged(void)
{
    if (configChanged_s)
    {
        configChanged_s = false;
        return true;
    }
    else
    {
        return false;
    }
}


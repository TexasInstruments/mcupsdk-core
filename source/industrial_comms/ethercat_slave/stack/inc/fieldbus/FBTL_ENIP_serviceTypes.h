/*!
* \file FBTL_ENIP_serviceTypes.h
*
* \brief
* FBTL EtherNET/IP slave service Types.
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-19
*
* \copyright
* Copyright (c) 2021, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#if !(defined __FBTL_ENIP_SERVICETYPES_H__)
#define __FBTL_ENIP_SERVICETYPES_H__		1

/**

\brief Service Code Enum

This enumeration is used to identify the service.
The Service IDs are splitted into groups. The 4 bit MSB are used to idetify the group
- 0 = General Service
- 1 = EtherCAT Service
- ...

\ingroup grp_gen_service

*/
// Service 16 bit -> highest 4 Bit = Unit (General or Bus)
typedef enum FBTL_ENIP_EService
{
    /**
    \brief Unknown ENIP Service

    Reserved for transmission and used as Generic callback indicator
    */
    FBTL_SVC_eENIP_Unknown = 0x2000,

    /**
    \brief ENIP Service PRUICSS_Init
    */
    FBTL_SVC_eENIP_ADP_pruicssInit,

    /**
    \brief ENIP Service PRUICSS_Start
    */
    FBTL_SVC_eENIP_ADP_pruicssStart,

    /**
    \brief ENIP Service PRUICSS_Stop
    */
    FBTL_SVC_eENIP_ADP_pruicssStop,

    /**
    \brief ENIP Service Get Vendor ID
    */
    FBTL_SVC_eENIP_ADP_getVendorId,

    /**
    \brief ENIP Service Set Vendor ID
    */
    FBTL_SVC_eENIP_ADP_setVendorId,

    /**
    \brief ENIP Service Get Device Type
    */
    FBTL_SVC_eENIP_ADP_getDeviceType,

    /**
    \brief ENIP Service Set Device Type
    */
    FBTL_SVC_eENIP_ADP_setDeviceType,

    /**
    \brief ENIP Service Get Product Code
    */
    FBTL_SVC_eENIP_ADP_getProductCode,

    /**
    \brief ENIP Service Set Product Code
    */
    FBTL_SVC_eENIP_ADP_setProductCode,

    /**
    \brief ENIP Service Get Revision
    */
    FBTL_SVC_eENIP_ADP_getRevision,

    /**
    \brief ENIP Service Set Revision
    */
    FBTL_SVC_eENIP_ADP_setRevision,

    /**
    \brief ENIP Service Get serial number
    */
    FBTL_SVC_eENIP_ADP_getSerialNumber,

    /**
    \brief ENIP Service Set serial number
    */
    FBTL_SVC_eENIP_ADP_setSerialNumber,

    /**
    \brief ENIP Service Get product name
    */
    FBTL_SVC_eENIP_ADP_getProductName,

    /**
    \brief ENIP Service Set product name
    */
    FBTL_SVC_eENIP_ADP_setProductName,

    /**
    \brief ENIP Service Set CMGR callback
    */
    FBTL_SVC_eENIP_ADP_setCmgrCb,

    /**
    \brief ENIP Service CMGR callback
    */
    FBTL_SVC_eENIP_ADP_callSetCmgrCb,

    /**
    \brief ENIP Service Get MAC address
    */
    FBTL_SVC_eENIP_ADP_getMacAddr,

    /**
    \brief ENIP Service Set IP config
    */
    FBTL_SVC_eENIP_ADP_setIpConfig,

    /**
    \brief ENIP Service Get IP address
    */
    FBTL_SVC_eENIP_ADP_getIpAddr,

    /**
    \brief ENIP Service Set IP address
    */
    FBTL_SVC_eENIP_ADP_setIpAddr,

    /**
    \brief ENIP Service Get IP network mask
    */
    FBTL_SVC_eENIP_ADP_getIpNwMask,

    /**
    \brief ENIP Service Set IP network mask
    */
    FBTL_SVC_eENIP_ADP_setIpNwMask,

    /**
    \brief ENIP Service Get IP gateway
    */
    FBTL_SVC_eENIP_ADP_getIpGateway,

    /**
    \brief ENIP Service Set IP gateway
    */
    FBTL_SVC_eENIP_ADP_setIpGateway,

    /**
    \brief ENIP Service Get IP primary name server
    */
    FBTL_SVC_eENIP_ADP_getIpPriNameServer,

    /**
    \brief ENIP Service Set IP primary name server
    */
    FBTL_SVC_eENIP_ADP_setIpPriNameServer,

    /**
    \brief ENIP Service Get IP secondary name server
    */
    FBTL_SVC_eENIP_ADP_getIpSecNameServer,

    /**
    \brief ENIP Service Set IP secondary name server
    */
    FBTL_SVC_eENIP_ADP_setIpSecNameServer,

    /**
    \brief ENIP Service Get domain name
    */
    FBTL_SVC_eENIP_ADP_getDomainName,

    /**
    \brief ENIP Service Set domain name
    */
    FBTL_SVC_eENIP_ADP_setDomainName,

    /**
    \brief ENIP Service Get hostname
    */
    FBTL_SVC_eENIP_ADP_getHostName,

    /**
    \brief ENIP Service Set hostname
    */
    FBTL_SVC_eENIP_ADP_setHostName,

    /**
    \brief ENIP Service Get IP TTL
    */
    FBTL_SVC_eENIP_ADP_getIpTTL,

    /**
    \brief ENIP Service Set IP TTL
    */
    FBTL_SVC_eENIP_ADP_setIpTTL,

    /**
    \brief ENIP Service Get configuration control
    */
    FBTL_SVC_eENIP_ADP_getConfigurationControl,

    /**
    \brief ENIP Service Get DHCP
    */
    FBTL_SVC_eENIP_ADP_getDHCP,

    /**
    \brief ENIP Service Get ACD
    */
    FBTL_SVC_eENIP_ADP_getACD,

    /**
    \brief ENIP Service Set ACD
    */
    FBTL_SVC_eENIP_ADP_setACD,

    /**
    \brief ENIP Service Get interface config
    */
    FBTL_SVC_eENIP_ADP_getIntfConfig,

    /**
    \brief ENIP Service Set interface config
    */
    FBTL_SVC_eENIP_ADP_setIntfConfig,

    /**
    \brief ENIP Service Set quick connect supported
    */
    FBTL_SVC_eENIP_ADP_setQuickConnectSupported,

    /**
    \brief ENIP Service Get quick connect enabled
    */
    FBTL_SVC_eENIP_ADP_getQuickConnectEnabled,

    /**
    \brief ENIP Service Set quick connect enabled
    */
    FBTL_SVC_eENIP_ADP_setQuickConnectEnabled,

    /**
    \brief ENIP Service Get encapsulate inactive timeout
    */
    FBTL_SVC_eENIP_ADP_getEncapInactTimeout,

    /**
    \brief ENIP Service Set encapsulate inactive timeout
    */
    FBTL_SVC_eENIP_ADP_setEncapInactTimeout,

    /**
    \brief ENIP Service Get ENIP ACD state
    */
    FBTL_SVC_eENIP_ADP_getEnipAcdState,

    /**
    \brief ENIP Service Set ENIP ACD state
    */
    FBTL_SVC_eENIP_ADP_setEnipAcdState,

    /**
    \brief ENIP Service Get ENIP ACD address
    */
    FBTL_SVC_eENIP_ADP_getEnipAcdAddr,

    /**
    \brief ENIP Service Set ENIP ACD address
    */
    FBTL_SVC_eENIP_ADP_setEnipAcdAddr,

    /**
    \brief ENIP Service Get ENIP ACD header
    */
    FBTL_SVC_eENIP_ADP_getEnipAcdHdr,

    /**
    \brief ENIP Service Set ENIP ACD header
    */
    FBTL_SVC_eENIP_ADP_setEnipAcdHdr,

    /**
    \brief ENIP Service Get multicast config
    */
    FBTL_SVC_eENIP_ADP_getMcastConfiguration,

    /**
    \brief ENIP Service Set multicast config
    */
    FBTL_SVC_eENIP_ADP_setMcastConfiguration,

    /**
    \brief ENIP Service Get module network status
    */
    FBTL_SVC_eENIP_ADP_getModuleNetworkStatus,

    /**
    \brief ENIP Service Set module network status function
    */
    FBTL_SVC_eENIP_ADP_setModuleNetworkStatusFunc,

    /**
    \brief ENIP Service call network status function
    */
    FBTL_SVC_eENIP_ADP_callModuleNetworkStatus,

    /**
    \brief ENIP Service Get quality of service
    */
    FBTL_SVC_eENIP_ADP_getQoS,

    /**
    \brief ENIP Service Set quality of service
    */
    FBTL_SVC_eENIP_ADP_setQoS,

    /**
    \brief ENIP Service add ports
    */
    FBTL_SVC_eENIP_ADP_addPorts,

    /**
    \brief ENIP Service Get port
    */
    FBTL_SVC_eENIP_ADP_getPort,

    /**
    \brief ENIP Service Set port
    */
    FBTL_SVC_eENIP_ADP_setPort,

    /**
    \brief ENIP Service Set HW settings
    */
    FBTL_SVC_eENIP_ADP_setHwSettings,

    /**
    \brief ENIP Service is DHCP enabled
    */
    FBTL_SVC_eENIP_ADP_isDhcpEnabled,

    /**
    \brief ENIP Service is hardware setting enabled
    */
    FBTL_SVC_eENIP_ADP_isHwSettingEnabled,

    /**
    \brief ENIP Service Set hardware configurable
    */
    FBTL_SVC_eENIP_ADP_setHwConfigurable,

    /**
    \brief ENIP Service Set timesync supported
    */
    FBTL_SVC_eENIP_ADP_setTimeSyncSupported,

    /**
    \brief ENIP Service Get PTP enable
    */
    FBTL_SVC_eENIP_ADP_getPtpEnable,

    /**
    \brief ENIP Service Set PTP enable
    */
    FBTL_SVC_eENIP_ADP_setPtpEnable,

    /**
    \brief ENIP Service Get port enable
    */
    FBTL_SVC_eENIP_ADP_getPortEnable,

    /**
    \brief ENIP Service Set port enable
    */
    FBTL_SVC_eENIP_ADP_setPortEnable,

    /**
    \brief ENIP Service Get port log announce interval
    */
    FBTL_SVC_eENIP_ADP_getPortLogAnnounceInterval,

    /**
    \brief ENIP Service Set port log announce interval
    */
    FBTL_SVC_eENIP_ADP_setPortLogAnnounceInterval,

    /**
    \brief ENIP Service Get port log sync interval
    */
    FBTL_SVC_eENIP_ADP_getPortLogSyncInterval,

    /**
    \brief ENIP Service Set port log sync interval
    */
    FBTL_SVC_eENIP_ADP_setPortLogSyncInterval,

    /**
    \brief ENIP Service Get domain number
    */
    FBTL_SVC_eENIP_ADP_getDomainNumber,

    /**
    \brief ENIP Service Set domain number
    */
    FBTL_SVC_eENIP_ADP_setDomainNumber,

    /**
    \brief ENIP Service Get timesync manufacture ID
    */
    FBTL_SVC_eENIP_ADP_getTimeSyncManufactureID,

    /**
    \brief ENIP Service Set timesync manufacture ID
    */
    FBTL_SVC_eENIP_ADP_setTimeSyncManufactureID,

    /**
    \brief ENIP Service Get timesync product description
    */
    FBTL_SVC_eENIP_ADP_getTimeSyncProductDescription,

    /**
    \brief ENIP Service Set timesync product description
    */
    FBTL_SVC_eENIP_ADP_setTimeSyncProductDescription,

    /**
    \brief ENIP Service Get timesync revision data
    */
    FBTL_SVC_eENIP_ADP_getTimeSyncRevisionData,

    /**
    \brief ENIP Service Set timesync revision data
    */
    FBTL_SVC_eENIP_ADP_setTimeSyncRevisionData,

    /**
    \brief ENIP Service Get timesync user description
    */
    FBTL_SVC_eENIP_ADP_getTimeSyncUserDescription,

    /**
    \brief ENIP Service Set timesync user description
    */
    FBTL_SVC_eENIP_ADP_setTimeSyncUserDescription,

    /**
    \brief ENIP Service Set error handler function
    */
    FBTL_SVC_eENIP_ADP_setErrorHandlerFunc,

    /**
    \brief ENIP Service call error handler function
    */
    FBTL_SVC_eENIP_ADP_callErrorHandler,

    /**
    \brief ENIP Service new CIP node
    */
    FBTL_SVC_eENIP_CIP_NODE_new,

    /**
    \brief ENIP Service delete CIP node
    */
    FBTL_SVC_eENIP_CIP_NODE_delete,

    /**
    \brief ENIP Instance service call
    */
    FBTL_SVC_eENIP_CIP_serviceInstanceCall,

    /**
    \brief ENIP Service create instance
    */
    FBTL_SVC_eENIP_CIP_createInstance,

    /**
    \brief ENIP Service Add instance service
    */
    FBTL_SVC_eENIP_CIP_addInstanceService,

    /**
    \brief ENIP Service Set instance service function
    */
    FBTL_SVC_eENIP_CIP_setInstanceServiceFunc,

    /**
    \brief ENIP Service instance attribute get call
    */
    FBTL_SVC_eENIP_CIP_instanceAttr_get,

    /**
    \brief ENIP Service instance attribute set call
    */
    FBTL_SVC_eENIP_CIP_instanceAttr_set,

    /**
    \brief ENIP Service Add instance attribute
    */
    FBTL_SVC_eENIP_CIP_addInstanceAttr,

    /**
    \brief ENIP Service Add instance attribute function
    */
    FBTL_SVC_eENIP_CIP_setInstanceAttrFunc,

    /**
    \brief ENIP Service Clone instance
    */
    FBTL_SVC_eENIP_CIP_cloneInstance,

    /**
    \brief ENIP Service Get instance attribute
    */
    FBTL_SVC_eENIP_CIP_getInstanceAttr,

    /**
    \brief ENIP Service Set instance attribute
    */
    FBTL_SVC_eENIP_CIP_setInstanceAttr,

    /**
    \brief ENIP Service Create assembly
    */
    FBTL_SVC_eENIP_CIP_createAssembly,

    /**
    \brief ENIP Service Add assembly member
    */
    FBTL_SVC_eENIP_CIP_addAssemblyMember,

    /**
    \brief ENIP Service Get assembly size
    */
    FBTL_SVC_eENIP_CIP_getAssemblySize,

    /**
    \brief ENIP Service Get assembly data
    */
    FBTL_SVC_eENIP_CIP_getAssemblyData,

    /**
    \brief ENIP Service Set assembly data
    */
    FBTL_SVC_eENIP_CIP_setAssemblyData,

    /**
    \brief ENIP Service Make attribute cyclic input
    */
    FBTL_SVC_eENIP_CIP_makeAttributeCyclicInput,

    /**
    \brief ENIP Service Make attribute cyclic output
    */
    FBTL_SVC_eENIP_CIP_makeAttributeCyclicOutput,

    /**
    \brief ENIP Service Allocate cyclic buffers
    */
    FBTL_SVC_eENIP_CIP_allocateCyclicBuffers,

    /**
    \brief ENIP Service Get cyclic input producer buffer
    */
    FBTL_SVC_eENIP_CIP_getCyclicInputProducerBuffer,

    /**
    \brief ENIP Service Find cyclic input producer buffer
    */
    FBTL_SVC_eENIP_CIP_flipCyclicInputProducerBuffer,

    /**
    \brief ENIP Service Get cyclic output consumber buffer
    */
    FBTL_SVC_eENIP_CIP_getCyclicOutputConsumerBuffer,

    /**
    \brief ENIP Service Flip output consumer buffer
    */
    FBTL_SVC_eENIP_CIP_flipOutputConsumerBuffer,

    /**
    \brief ENIP Service Get cyclic input buffer size
    */
    FBTL_SVC_eENIP_CIP_getCyclicInputBufferSize,

    /**
    \brief ENIP Service Get cyclic output buffer size
    */
    FBTL_SVC_eENIP_CIP_getCyclicOutputBufferSize,

    FBTL_SVC_eENIP_Max,
} FBTL_ENIP_EService_t;

#if (defined __cplusplus)
extern "C" {
#endif

extern char* FBTL_SVC_ENIP_enumString(FBTL_ENIP_EService_t enumVal);

#if (defined __cplusplus)
}
#endif

#endif /* __FBTL_ENIP_SERVICETYPES_H__ */

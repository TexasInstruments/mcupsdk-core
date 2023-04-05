/*
 *  Copyright (c) 2021, KUNBUS GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
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
 *
 */

/*
 *  \file
 */
#ifndef EI_API_ADP_H_INC
#define EI_API_ADP_H_INC

#include "EI_API_def.h"         // include global definitions
#include "EI_API_ADP_define.h"  // include common EtherNet/IP definitions

#include <stdint.h>
#include <stdbool.h>

// Type forward declaration

/// @cond INTERNAL
#define T EI_API_ADP_T
/// @endcond

/// @cond INTERNAL
typedef struct T T;
/// @endcond

#ifdef __cplusplus
extern "C" {
#endif

// Basic adapter functions
extern ETHIP_API uint32_t EI_API_ADP_pruicssInit(EIP_SLoadParameter* ptPara_p);
extern ETHIP_API void     EI_API_ADP_pruicssStart(void);
extern ETHIP_API void     EI_API_ADP_pruicssStop(void);

extern ETHIP_API T*       EI_API_ADP_new(uint8_t numInterfaces_p);
extern ETHIP_API uint32_t EI_API_ADP_delete(T* pAdp_p);
extern ETHIP_API uint32_t EI_API_ADP_init(T* pAdp_p, EI_API_ADP_SInit_t params_p);
extern ETHIP_API void     EI_API_ADP_run(void);

// functions for standard CIP object 0x01 Identity
extern ETHIP_API uint32_t EI_API_ADP_getVendorId(T* pAdp_p, uint16_t* pVendorId_p);
extern ETHIP_API uint32_t EI_API_ADP_setVendorId(T* pAdp_p, uint16_t vendorId_p);

extern ETHIP_API uint32_t EI_API_ADP_getDeviceType(T* pAdp_p, uint16_t* pDeviceType_p);
extern ETHIP_API uint32_t EI_API_ADP_setDeviceType(T* pAdp_p, uint16_t deviceType_p);

extern ETHIP_API uint32_t EI_API_ADP_getProductCode(T* pAdp_p, uint16_t* pProductCode_p);
extern ETHIP_API uint32_t EI_API_ADP_setProductCode(T* pAdp_p, uint16_t productCode_p);

extern ETHIP_API uint32_t EI_API_ADP_getRevision(T* pAdp_p, EI_API_ADP_SRevision_t* pRevision_p);
extern ETHIP_API uint32_t EI_API_ADP_setRevision(T* pAdp_p, EI_API_ADP_SRevision_t revison_p);

extern ETHIP_API uint32_t EI_API_ADP_getSerialNumber(T* pAdp_p, uint32_t* pSerialNumber_p);
extern ETHIP_API uint32_t EI_API_ADP_setSerialNumber(T* pAdp_p, uint32_t serialNumber_p);

extern ETHIP_API uint32_t EI_API_ADP_getProductName(T* pAdp_p, char* pProductName_p);
extern ETHIP_API uint32_t EI_API_ADP_setProductName(T* pAdp_p, const char *pProductName_p);

// functions for standard CIP object 0x06 Connection Manager
extern ETHIP_API uint32_t EI_API_ADP_setCmgrCb(EI_API_ADP_CBCmgr fuCallback_p);

// functions for standard CIP object 0xF6 Ethernet Link
extern ETHIP_API uint32_t EI_API_ADP_getMacAddr(T* pAdp_p, EI_API_ADP_SParam_t* pMacAddr_p);

// functions for standard CIP object 0xF5 TCP/IP
extern ETHIP_API uint32_t EI_API_ADP_setIpConfig(T* pAdp_p,    EIP_SConfigurationControl_t configurationControl_p,
                                       uint32_t ipAddr_p,      uint32_t netwMask_p,
                                       uint32_t gateway_p,     uint32_t nameServer1_p,
                                       uint32_t nameServer2_p, char*    pDomainName_p,
                                       bool     applyChanges);

extern ETHIP_API uint32_t EI_API_ADP_getIpAddr(T* pAdp_p, uint32_t* pIpAddr_p);
//extern ETHIP_API uint32_t EI_API_ADP_setIpAddr(T* pAdp_p, uint32_t ipAddr_p);

extern ETHIP_API uint32_t EI_API_ADP_getIpNwMask(T* pAdp_p, uint32_t* pIpNwMask_p);
//extern ETHIP_API uint32_t EI_API_ADP_setIpNwMask(T* pAdp_p, uint32_t nwMask_p);

extern ETHIP_API uint32_t EI_API_ADP_getIpGateway(T* pAdp_p, uint32_t* pIpGateway_p);
//extern ETHIP_API uint32_t EI_API_ADP_setIpGateway(T* pAdp_p, uint32_t ipGateway_p);

extern ETHIP_API uint32_t EI_API_ADP_getIpPriNameServer(T* pAdp_p, uint32_t* pIpNameServer1_p);
//extern ETHIP_API uint32_t EI_API_ADP_setIpPriNameServer(T* pAdp_p, uint32_t ipNameServer1_p);

extern ETHIP_API uint32_t EI_API_ADP_getIpSecNameServer(T* pAdp_p, uint32_t* pIpNameServer2_p);
//extern ETHIP_API uint32_t EI_API_ADP_setIpSecNameServer(T* pAdp_p, uint32_t ipNameServer2_p);

extern ETHIP_API uint32_t EI_API_ADP_getDomainName(T* pAdp_p, char* pDomainName_p);
//extern ETHIP_API uint32_t EI_API_ADP_setDomainName(T* pAdp_p, const char* pDomainName_p);

extern ETHIP_API uint32_t EI_API_ADP_getHostName(T* pAdp_p, char* pHostName_p);
extern ETHIP_API uint32_t EI_API_ADP_setHostName(T* pAdp_p, const char* pHostName_p);

extern ETHIP_API uint32_t EI_API_ADP_getIpTTL(T* pAdp_p, uint8_t* pIpTTL_p);
extern ETHIP_API uint32_t EI_API_ADP_setIpTTL(T* pAdp_p, uint8_t ipTTL_p);

extern ETHIP_API uint32_t EI_API_ADP_getConfigurationControl(T* pAdp_p, EIP_SConfigurationControl_t *pConfigurationControl_p);

extern ETHIP_API uint32_t EI_API_ADP_getDHCP(T* pAdp_p, bool* pDhcpEnabled_p);

extern ETHIP_API uint32_t EI_API_ADP_getACD(T* pAdp_p, bool* pAcdEnabled_p);
extern ETHIP_API uint32_t EI_API_ADP_setACD(T* pAdp_p, bool enable_p);

extern ETHIP_API uint32_t EI_API_ADP_getIntfConfig(T* pAdp_p, uint8_t intfId_p, EI_API_ADP_UIntfConf_t* pIntfConf_p);
extern ETHIP_API uint32_t EI_API_ADP_setIntfConfig(T* adp, uint8_t intfId, EI_API_ADP_UIntfConf_t intfConfig);

extern ETHIP_API uint32_t EI_API_ADP_setQuickConnectSupported(T* pAdp_p);
extern ETHIP_API uint32_t EI_API_ADP_getQuickConnectEnabled(T* pAdp_p, bool* pQuickConnectEnabled_p);
extern ETHIP_API uint32_t EI_API_ADP_setQuickConnectEnabled(T* pAdp_p, bool quickConnectEnabled_p);

extern ETHIP_API uint32_t EI_API_ADP_getEncapInactTimeout(T* pAdp_p, uint16_t* pEncapInactTimeout_p);
extern ETHIP_API uint32_t EI_API_ADP_setEncapInactTimeout(T* pAdp_p, uint16_t encapInactTimeout_p);

extern ETHIP_API uint32_t EI_API_ADP_getEnipAcdState(T* pAdp_p, uint8_t* pEnipAcdState_p);
extern ETHIP_API uint32_t EI_API_ADP_setEnipAcdState(T* pAdp_p, uint8_t enipAcdState_p);

extern ETHIP_API uint32_t EI_API_ADP_getEnipAcdAddr(T* pAdp_p, EI_API_ADP_SParam_t* pEnipAcdAddr_p);
extern ETHIP_API uint32_t EI_API_ADP_setEnipAcdAddr(T* pAdp_p, const EI_API_ADP_SParam_t* pEnipAcdAddr_p);

extern ETHIP_API uint32_t EI_API_ADP_getEnipAcdHdr(T* pAdp_p, EI_API_ADP_SParam_t* pEnipAcdHdr_p);
extern ETHIP_API uint32_t EI_API_ADP_setEnipAcdHdr(T* pAdp_p, const EI_API_ADP_SParam_t* pEnipAcdHdr_p);

extern ETHIP_API uint32_t EI_API_ADP_getMcastConfiguration(T* pAdp_p, EI_API_ADP_SMcastConfig_t* pMcastConfig_p);
extern ETHIP_API uint32_t EI_API_ADP_setMcastConfiguration(T* pAdp_p, const EI_API_ADP_SMcastConfig_t* pMcast_p);


// general adapter status functions
extern ETHIP_API uint32_t EI_API_ADP_getModuleNetworkStatus(T* pAdp_p, EI_API_ADP_SModNetStatus_t* pStatus_p);
extern ETHIP_API uint32_t EI_API_ADP_setModuleNetworkStatusFunc(T* pAdp_p, EI_API_ADP_CBStatus callback_p);

// functions for standard CIP object 0x48 QoS
extern ETHIP_API uint32_t EI_API_ADP_getQoS(T* pAdp_p, EI_API_ADP_SQos_t* pQoS_p);
extern ETHIP_API uint32_t EI_API_ADP_setQoS(T* pAdp_p, const EI_API_ADP_SQos_t* pQos_p);


// functions for hardware specific settings
// extern ETHIP_API uint32_t EI_API_ADP_addPorts(T* pAdp_p, uint8_t numPorts_p);
// extern ETHIP_API uint32_t EI_API_ADP_getPort(T* pAdp_p, uint8_t portId_p, EI_API_ADP_SPort_t* pPort_p);
// extern ETHIP_API uint32_t EI_API_ADP_setPort(T* pAdp_p, uint8_t portId_p, const EI_API_ADP_SPort_t* pPort_p);
extern ETHIP_API uint32_t EI_API_ADP_setHwSettings(T* pAdp_p, bool hwSettingsEnabled_p, bool dhcpEnabled_p);
extern ETHIP_API uint32_t EI_API_ADP_isDhcpEnabled(T* pAdp_p, bool* pHwDhcpEnabled_p);
extern ETHIP_API uint32_t EI_API_ADP_isHwSettingEnabled(T* pAdp_p, bool* pHwConfigEnabled_p);
extern ETHIP_API uint32_t EI_API_ADP_setHwConfigurable( T* pAdp_p, bool hwConfigurable_p);

// functions for standard CIP object 0x43 timeSync
extern ETHIP_API uint32_t EI_API_ADP_setTimeSyncSupported(T* pAdp_p);
extern ETHIP_API uint32_t EI_API_ADP_getPtpEnable(T* pAdp_p, bool* pPtpEnable_p);
extern ETHIP_API uint32_t EI_API_ADP_setPtpEnable(T* pAdp_p, bool  ptpEnable_p);
extern ETHIP_API uint32_t EI_API_ADP_getPortEnable(T* pAdp_p, bool* pPortEnable_p);
extern ETHIP_API uint32_t EI_API_ADP_setPortEnable(T* pAdp_p, bool portEnable_p);
extern ETHIP_API uint32_t EI_API_ADP_getPortLogAnnounceInterval(T* pAdp_p, uint16_t* pPortLogAnnounceInterval_p);
extern ETHIP_API uint32_t EI_API_ADP_setPortLogAnnounceInterval(T* pAdp_p, uint16_t portLogAnnounceInterval_p);
extern ETHIP_API uint32_t EI_API_ADP_getPortLogSyncInterval(T* pAdp_p, int16_t* pPortLogSyncInterval_p);
extern ETHIP_API uint32_t EI_API_ADP_setPortLogSyncInterval(T* pAdp_p, int16_t portLogSyncInterval_p);
extern ETHIP_API uint32_t EI_API_ADP_getDomainNumber(T* pAdp_p, uint8_t* pDomainNumber_p);
extern ETHIP_API uint32_t EI_API_ADP_setDomainNumber(T* pAdp_p, uint8_t domainNumber_p);
extern ETHIP_API uint32_t EI_API_ADP_getTimeSyncManufactureID(T* pAdp_p, char* pManufactureID_p);
extern ETHIP_API uint32_t EI_API_ADP_setTimeSyncManufactureID(T* pAdp_p, const char* pManufactureID_p);
extern ETHIP_API uint32_t EI_API_ADP_getTimeSyncProductDescription(T* pAdp_p, char* pProductDesc_p);
extern ETHIP_API uint32_t EI_API_ADP_setTimeSyncProductDescription(T* pAdp_p, const char* pProductDesc_p);
extern ETHIP_API uint32_t EI_API_ADP_getTimeSyncRevisionData(T* pAdp_p, char* pRevisionData_p);
extern ETHIP_API uint32_t EI_API_ADP_setTimeSyncRevisionData(T* pAdp_p, const char* pRevisionData_p);
extern ETHIP_API uint32_t EI_API_ADP_getTimeSyncUserDescription(T* pAdp_p, char* pUserDescription_p);
extern ETHIP_API uint32_t EI_API_ADP_setTimeSyncUserDescription(T* pAdp_p, const char* pUserDescription_p);

// functions for Bridging and Routing feature
extern ETHIP_API uint32_t EI_API_ADP_setBridgingAndRoutingSupported (T* pAdp_p);

// functions for error handling
extern ETHIP_API uint32_t EI_API_ADP_setErrorHandlerFunc(EI_API_ADP_CBStackError callback_p);

// functions for lldp parameter
extern uint32_t EI_API_ADP_setLldpParameter(T* pAdp_p, EI_API_ADP_SLldp_Parameter_t lldpParameter);
extern uint32_t EI_API_ADP_getLldpParameter(T* pAdp_p, EI_API_ADP_SLldp_Parameter_t *lldpParameter);

#ifdef  __cplusplus 
}
#endif

#undef T

#endif // EI_API_ADP_H_INC

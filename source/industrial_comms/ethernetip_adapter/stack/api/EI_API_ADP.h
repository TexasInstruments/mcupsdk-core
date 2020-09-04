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

#include "EI_API_def.h" // include global definitions

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif 

/*!
 *  \brief Default values for QoS object instance attributes.
 *
 */
#define EI_API_ADP_DEFAULT_8021Q 0
#define EI_API_ADP_DEFAULT_DSCP_PTP_EVENT 59
#define EI_API_ADP_DEFAULT_DSCP_PTP_GENERAL 47
#define EI_API_ADP_DEFAULT_DSCP_URGENT 55
#define EI_API_ADP_DEFAULT_DSCP_SCHEDULED 47
#define EI_API_ADP_DEFAULT_DSCP_HIGH 43
#define EI_API_ADP_DEFAULT_DSCP_LOW 31
#define EI_API_ADP_DEFAULT_DSCP_EXPLICIT 27


/*!
 *  \brief
 *  ADP error codes (base 0x38020Axx).
 *  \ingroup EI_API_ADP_ERROR_CODES
 */
typedef enum EI_API_ADP_EError
{
    // ADP general error codes (base 0x38020Axx)
    EI_API_ADP_eERR_OK                                  = 0x00000000,     /*!< No error, everything should be fine. */
    EI_API_ADP_eERR_GENERAL                             = 0x38020A01,     /*!< General ADP error. */
    EI_API_ADP_eERR_NOT_IMPLEMENTED                     = 0x38020A02,     /*!< Adapter function not implemented. */
    EI_API_ADP_eERR_LENGTH                              = 0x38020A03,     /*!< Length error. */
    EI_API_ADP_eERR_MEMALLOC                            = 0x38020A04,     /*!< Error during memory allocation. */

    // ADP identity object error codes (base 0x38020Bxx)

    // ADP QoS object error codes (base 0x38020Cxx)
    EI_API_ADP_eERR_QOS_802_1Q_NOT_SUPPORTED            = 0x38020C01,     /*!< 802.1Q tagging not supported. */
    EI_API_ADP_eERR_QOS_PTP_EVENT_VALUE_OUT_OF_RANGE    = 0x38020C02,     /*!< DSCP value for PTP (IEEE 1588) Event messages out of range. */
    EI_API_ADP_eERR_QOS_PTP_GENERAL_VALUE_OUT_OF_RANGE  = 0x38020C03,     /*!< DSCP value for PTP (IEEE 1588) General messages out of range. */
    EI_API_ADP_eERR_QOS_URGENT_VALUE_OUT_OF_RANGE       = 0x38020C04,     /*!< DSCP value for CIP Urgent messages out of range. */
    EI_API_ADP_eERR_QOS_SCHEDULED_VALUE_OUT_OF_RANGE    = 0x38020C05,     /*!< DSCP value for CIP Scheduled priority messages out of range. */
    EI_API_ADP_eERR_QOS_HIGH_VALUE_OUT_OF_RANGE         = 0x38020C06,     /*!< DSCP value for CIP High priority messages out of range. */
    EI_API_ADP_eERR_QOS_LOW_VALUE_OUT_OF_RANGE          = 0x38020C07,     /*!< DSCP value for CIP Low priority messages out of range. */
    EI_API_ADP_eERR_QOS_EXPLICIT_VALUE_OUT_OF_RANGE     = 0x38020C08,     /*!< DSCP value for CIP UCMM, transport class 2/3 messages out of range. */

    /// ADP TCP/IP object error codes (base 0x38020Dxx)
    EI_API_ADP_eERR_TCPIP_IPADDR_VALUE_INVALID          = 0x38020D01,     /*!< IP address value is invalid or reserved. */
    EI_API_ADP_eERR_TCPIP_GATEWAY_VALUE_INVALID         = 0x38020D02,     /*!< Gateway address value is invalid or reserved. */
    EI_API_ADP_eERR_TCPIP_DOMAINNAME_NULL_POINTER       = 0x38020D03,     /*!< Domain name can't to be referenced with NULL pointer. */
    EI_API_ADP_eERR_TCPIP_DOMAINNAME_LENGTH             = 0x38020D04,     /*!< Domain name length is restricted to 48 bytes. */

    /// ADP Ethernet Link object error codes(base 0x38020Exx)

    /// ADP timeSync object error codes (base 0x38020Fxx)
    EI_API_ADP_eERR_TIMESYNC_WRONG_FORMAT               = 0x38020F01,     /*!< Format error. Check specification for this attribute.  */

} EI_API_ADP_EError_t;


// Type forward declaration

/// @cond INTERNAL
#define T EI_API_ADP_T
/// @endcond

/// @cond INTERNAL
typedef struct T T;
/// @endcond


/*!
 *  \details
 *  Structure to use as function parameter (see adapter setter and getter functions).
 */
typedef struct EI_API_ADP_SParam
{
    uint8_t len;                /*!< Parameter length. */
    uint8_t *data;              /*!< Pointer to parameter data. */
} EI_API_ADP_SParam_t;


/*!
 *  \details
 *  The Revision attribute, which consists of Major and Minor Revisions,
 *  identifies the Revision of the item the Identity Object is representing.
 *  The value zero is not valid for either the Major or Minor Revision fields.
 *  If the Device Type of the instance is Embedded Component (0xC8),
 *  the Major Revision and/or Minor Revision may be zero.
 */
typedef struct EI_API_ADP_SRevision
{
    uint8_t major;              /*!< Major Revision number. Limited to values between 1 and 127;
                                     the eighth bit is reserved by CIP and shall have a value of zero. */
    uint8_t minor;              /*!< Used to identify changes in a product that do not affect user configuration choices.
                                     Changes in minor revision are not used by a configuration tool to match a device
                                     with an Electronic Data Sheet. */
} EI_API_ADP_SRevision_t;


///*!
// *  \details
// *  Structure with port specific settings.
// */
typedef struct EI_API_ADP_SPort
{
    // PHY
    void* mdioBaseAddr;         /*!< TODO */
    uint8_t type;               /*!< TODO */
    uint8_t address;            /*!< TODO */
    uint16_t resetTime;         /*!< Reset de-assertion time to be prepared for MDIO communication. */

    // Link
    bool fullDuplex;            /*!< TODO */
    bool linkUp;                /*!< TODO */
    uint8_t linkIntType;        /*!< TODO */
    uint8_t linkSysEvent;       /*!< TODO */
    uint8_t userPhySel;         /*!< TODO */
    uint32_t linkSpeed;         /*!< TODO */
} EI_API_ADP_SPort_t;

typedef struct EI_API_ADP_SInit
{
    OSAL_TASK_EPriority_t taskPrioPacket;
    OSAL_TASK_EPriority_t taskPrioStatistic;
}EI_API_ADP_SInit_t;

typedef void (*fnResetPhyT)(void* ctxt, uint8_t idx, bool bReset);

#define EIP_MAC_ADDR_LEN             6

typedef struct PRUICSS_Config* PRUICSS_ConfigPtr;
typedef struct ETHPHY_Config*  ETHPHY_ConfigPtr;
typedef void*  ETHPHY_Handle;

typedef struct EIP_SLoadParameter
{
    uint8_t                 ai8uMacAddr[EIP_MAC_ADDR_LEN];
    uint32_t                pruIcssCfgId;          /* PRU-ICSS block enumeration id listed in SysConfig */
    PRUICSS_ConfigPtr       pPruIcssCfg;           /* pointer to PRU-ICSS block configuration */
    ETHPHY_ConfigPtr        pEthPhyCfg[2];         /* array of pointers to ETHPHY configurations */
    ETHPHY_Handle           ethPhyHandle[2];       /* array of ETHPHY handlers */
    OSAL_TASK_EPriority_t   taskPrioPhyMdixTask;   /* PHY MDIX task priority */
    OSAL_TASK_EPriority_t   taskPrioTsDelayRqTx;   /* TimeSync task priority for TX Delay Request */
    OSAL_TASK_EPriority_t   taskPrioTxTimeStamp;   /* TimeSync task priority for TX Time Stamp P1 and P2*/
    OSAL_TASK_EPriority_t   taskPrioNRT;           /* TimeSync task priority for NRT */
    OSAL_TASK_EPriority_t   taskPrioBackground;    /* TimeSync task priority for Background thread*/
} EIP_SLoadParameter;

typedef enum
{
    EIP_enLST_INVALID,
    EIP_enLST_UP,
    EIP_enLST_DOWN,
    EIP_enLST_FORCE32BIT = 0xffffffff
} EIP_ELinkState;

typedef enum
{
    EIP_enPORT_INVALID,
    EIP_enPORT_DEFAULT,
    EIP_enPORT_1,
    EIP_enPORT_2,
    EIP_enPORT_ALL,
    EIP_enPORT_FORCE32BIT = 0xffffffff,
} EIP_EEthernetPort;

typedef enum
{
    EIP_enPHS_INVALID,
    EIP_enPHS_10MB,
    EIP_enPHS_100MB,
    EIP_enPHS_1GB,
    EIP_enPHS_FORCE32BIT = 0xffffffff
} EIP_EPhySpeed;

typedef enum
{
    EIP_enPHM_INVALID,
    EIP_enPHM_HALF,
    EIP_enPHM_FULL,
    EIP_enPHM_FORCE32BIT = 0xffffffff
} EIP_EPhyDuplexMode;

typedef enum
{
    EIP_eCFGMETHOD_STATIC = 0,
    EIP_eCFGMETHOD_BOOTP  = 1,
    EIP_eCFGMETHOD_DHCP   = 2,
} EIP_EConfigurationMethod_t;

typedef struct
{
    EIP_EConfigurationMethod_t configurationMethod  : 4;  //
    bool                       dnsEnable            : 1;  // DNS Enable, not supported now
    uint32_t                   reserved             : 27;
} EIP_SConfigurationControl_t;

typedef struct EIP_SPortState
{
    EIP_ELinkState      enLink;
    EIP_EPhySpeed       enSpeed;
    EIP_EPhyDuplexMode  enMode;
} EIP_TPortState;

// Basic adapter functions
extern ETHIP_API uint32_t           EI_API_ADP_pruicssInit(EIP_SLoadParameter* ptPara_p);
extern ETHIP_API void               EI_API_ADP_pruicssStart(void);
extern ETHIP_API void               EI_API_ADP_pruicssStop(void);

extern ETHIP_API T* EI_API_ADP_new(uint8_t numInterfaces_p);
extern ETHIP_API uint32_t EI_API_ADP_delete(T* pAdp_p);
extern ETHIP_API uint32_t EI_API_ADP_init(T* pAdp_p, EI_API_ADP_SInit_t params_p);
extern ETHIP_API void EI_API_ADP_run(void);

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

// functions for error handling
extern ETHIP_API uint32_t EI_API_ADP_setErrorHandlerFunc(EI_API_ADP_CBStackError callback_p);

#undef T

#ifdef  __cplusplus 
}
#endif 

#endif // EI_API_ADP_H_INC

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

#ifndef INC_PROT__IOLM_SMI_TYPES_H__
#define INC_PROT__IOLM_SMI_TYPES_H__

#ifndef IOL_ONLY_TYPE_INCLUDE
#include "IOL_Types.h"
#include "IOLM_Types.h"
#include "IOLM_Port_Definition.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IOLM_SMI_SUPPORT_OLD_SERVICES
#define IOLM_SMI_SUPPORT_OLD_SERVICES 1
#endif

#ifndef IOLM_SMI_MAX_DIAG_ENTRIES
#define IOLM_SMI_MAX_DIAG_ENTRIES 5
#endif

#ifndef IOLM_MAX_SCAN_RESULTS
#define IOLM_MAX_SCAN_RESULTS       128
#endif

/**
\addtogroup grp_smi_arg SMI Arg Blocks
\{
*/
/**
\brief Used for internal communication.
*/
#define IOLM_SMI_CLIENTID_INTERN                        (0xFE)
/**
\brief Used for broadcasting (events).
*/
#define IOLM_SMI_CLIENTID_BROADCAST                     (0x00)

/**
\brief SGI dependent defines
*/
#ifdef IOLM_SGI_ENABLED
/**
\brief Max lengh of the client name.
*/
#define IOLM_SGI_CLIENT_MAX_NAME_LENGTH                 (128u)
/**
\brief Minimal auto release timeout (in seconds) in case of idle communication state
*/
#define IOLM_SGI_AUTO_RELEASE_TIME_MIN                  (1u)
/**
\brief Maximal auto release timeout  (in seconds) in case of idle communication state
*/
#define IOLM_SGI_AUTO_RELEASE_TIME_MAX                  (10u)
/**
\brief Minimal release timeout (in seconds) in case of delegation of the exclusive rights
*/
#define IOLM_SGI_EXCLUSIVE_ACYCLIC_TIMEOUT_MIN          (1u)
/**
\brief Maximal release timeout  (in seconds) in case of delegation of the exclusive rights
*/
#define IOLM_SGI_EXCLUSIVE_ACYCLIC_TIMEOUT_MAX          (600u)
/**
\brief The id of the client which is allowed to setup rights
*/
#define SGI_MANAGER_CLIENT_ID                           (240u)
#endif //  



#pragma pack(push, 1)

/**
\brief SGI dependent enums
*/
#ifdef IOLM_SGI_ENABLED
/**
\brief SMI client role

structure contains the roles of a smi client

IOLM_SMI_CLIENT_ROLE_NO_ACCESS:             - Gateway management blocks the
                                            whole functionality (read/ write).
                                            Therefore, the IO-Link system is not
                                            visible from client point of view

IOLM_SMI_CLIENT_ROLE_MASTER_SUPERUSER:      - Writable access to all services
                                            of Master / Port / Device
                                            - Readable access to all services
                                            of Master / Port / Device

IOLM_SMI_CLIENT_ROLE_MASTER_COMMISSIONING : - Writable access to all services
                                            of Master/Port/Device except for
                                            output data
                                            - Readable access to all services
                                            of Master/Port/Device

IOLM_SMI_CLIENT_ROLE_PORT_SUPERUSER:        - Writable access to all services
                                            of Device
                                            - Readable access to all services
                                            of Master/Port/Device

IOLM_SMI_CLIENT_ROLE_DEVICE_COMMISSIONING:  - Writable access to all services
                                            of Device except for output data
                                            - Readable access to all services
                                            of Master/Port/Device

IOLM_SMI_CLIENT_ROLE_MONITORING:            - Readable access to all services
                                            of Master/Port/Device

*/
typedef IOL_ENUM_DECL IOLM_SGI_SClientRole
{
    IOLM_SMI_CLIENT_ROLE_NO_ACCESS =                    0x00000000,
    IOLM_SMI_CLIENT_ROLE_MASTER_SUPERUSER =             0x00000001,
    IOLM_SMI_CLIENT_ROLE_MASTER_COMMISSIONING =         0x00000002,
    IOLM_SMI_CLIENT_ROLE_PORT_SUPERUSER =               0x00000004,
    IOLM_SMI_CLIENT_ROLE_DEVICE_COMMISSIONING =         0x00000008,
    IOLM_SMI_CLIENT_ROLE_MONITORING =                   0x00000010,
}IOLM_SGI_SClientRole;

typedef IOL_ENUM_DECL IOLM_SGI_SRight
{
    IOLM_SMI_PORT_NO_ACCESS =                           0x00000000,   //Bit
    IOLM_SMI_PORT_CFG_WR_ACCESS =                       0x00000001,   // 0
    IOLM_SMI_PORT_CFG_RD_ACCESS =                       0x00000002,   // 1

    IOLM_SMI_PORT_PDIN_ACCESS =                         0x00000004,   // 2

    IOLM_SMI_PORT_ISDU_RD_ACCESS  =                     0x00000008,   // 3
    IOLM_SMI_PORT_ISDU_WR_ACCESS =                      0x00000010,   // 4
    IOLM_SMI_PORT_ISDU_EXCLUSIVE =                      0x00000020,   // 5

    IOLM_SMI_PORT_PDOUT_ACCESS =                        0x00000040,   // 6
    IOLM_SMI_PORT_EVENT_SUBSCRIPTION =                  0x00000080,   // 7

    IOLM_SMI_PORT_TEMP_PDOUT_ACCESS =                   0x00000100,   // 8

    IOLM_SMI_PORT_FORCE_PDIN_ACCESS =                   0x00000200,   // 9

    IOLM_SMI_PORT_POWER_CFG_ACCESS =                    0x00000400,   // 10

    IOLM_SMI_MASTER_CFG_ACCESS =                        0x00001000,   // 12

    IOLM_SMI_TRACK_CFG_ACCESS =                         0x00002000,   // 13
}IOLM_SGI_SRight;

typedef IOL_ENUM_DECL IOLM_SGI_EAccessCommand
{
    IOLM_SGI_eSet =                                     0x00,
    IOLM_SGI_eRemove =                                  0x01,
    IOLM_SGI_eAdd =                                     0x02,
}IOLM_SGI_EAccessCommand;

typedef IOL_ENUM_DECL IOLM_SGI_EGatewayStatusInfo
{
    IOLM_SGI_eStop =                                    0x00,
    IOLM_SGI_eRunning =                                 0x01,
    IOLM_SGI_eProcessDataOverride =                     0x02,
}IOLM_SGI_EGatewayStatusInfo;
#endif

/**
\brief SMI ServiceIDs (IDs >= 0x80 are used for non standard, proprietary services).
*/
typedef IOL_ENUM_DECL IOLM_SMI_EServiceID
{
    IOLM_SMI_eServiceID_MasterIdentification =              0x00,
    IOLM_SMI_eServiceID_PortConfiguration =                 0x01,
    IOLM_SMI_eServiceID_ReadbackPortConfiguration =         0x02,
    IOLM_SMI_eServiceID_PortStatus =                        0x03,
    IOLM_SMI_eServiceID_DSBackupToParServ =                 0x04,
    IOLM_SMI_eServiceID_DSRestoreFromParServ =              0x05,
    IOLM_SMI_eServiceID_DeviceWrite =                       0x06,
    IOLM_SMI_eServiceID_DeviceRead =                        0x07,
    IOLM_SMI_eServiceID_PortPairing =                       0x08,
    IOLM_SMI_eServiceID_DeviceEvent =                       0x09,
    IOLM_SMI_eServiceID_PortEvent =                         0x0A,
    IOLM_SMI_eServiceID_PDIn =                              0x0B,
    IOLM_SMI_eServiceID_PDOut =                             0x0C,
    IOLM_SMI_eServiceID_PDInOut =                           0x0D,
    IOLM_SMI_eServiceID_PDInIQ =                            0x0E,
    IOLM_SMI_eServiceID_PDOutIQ =                           0x0F,
    IOLM_SMI_eServiceID_MasterConfiguration =               0x10,
    IOLM_SMI_eServiceID_ReadbackMasterConfiguration =       0x11,
    IOLM_SMI_eServiceID_TrackConfiguration =                0x12,   //outdated: left for compatibility
    IOLM_SMI_eServiceID_ReadbackTrackConfiguration =        0x13,   //outdated: left for compatibility
    IOLM_SMI_eServiceID_TrackScanResult =                   0x14,   //outdated: left for compatibility
    IOLM_SMI_eServiceID_TrackScanEnd =                      0x15,   //outdated: left for compatibility
    IOLM_SMI_eServiceID_TrackStatus =                       0x16,   //outdated: left for compatibility
    IOLM_SMI_eServiceID_Scan =                              0x17,   //outdated: left for compatibility
    IOLM_SMI_eServiceID_FSMasterAccess =                    0x18,
    IOLM_SMI_eServiceID_SPDUIn =                            0x19,
    IOLM_SMI_eServiceID_SPDUOut =                           0x1A,
    IOLM_SMI_eServiceID_PortPowerOffOn =                    0x1B,
    
    // services added for wireless specification V1.1.3
    IOLM_SMI_V113_eServiceID_WMasterConfiguration =         0x20, // new SMI_WMasterConfiguration service in V1.1.3
    IOLM_SMI_V113_eServiceID_ReadbackWMasterConfiguration = 0x21, // new SMI_ReadbackWMasterConfiguration service in V1.1.3
    IOLM_SMI_V113_eServiceID_WScanConfig =                  0x22, // new SMI_WScanConfig service in V1.1.3
    IOLM_SMI_V113_eServiceID_WScanStatus =                  0x23, // new SMI_WScanStatus service in V1.1.3
    IOLM_SMI_V113_eServiceID_WPortPairing =                 0x24, // new SMI_WPortPairing service in V1.1.3
    IOLM_SMI_V113_eServiceID_WTrackStatus =                 0x25, // new SMI_WTrackStatus service in V1.1.3
    IOLM_SMI_V113_eServiceID_WQualityStatus =               0x26, // new SMI_WQualityStatus service in V1.1.3
    
    // KUNBUS specific services
    IOLM_SMI_eServiceID_GetChipInfo =                       0x80,
    IOLM_SMI_eServiceID_ResetDefaults =                     0x81,
    IOLM_SMI_eServiceID_Settings =                          0x82,
    IOLM_SMI_eServiceID_DataLog =                           0x83,
    IOLM_SMI_eServiceID_FWUpdate =                          0x90,
    
    IOLM_SMI_eServiceID_FSPDIn =                            0xA0,
    IOLM_SMI_eServiceID_FSPDOut =                           0xA1,
    IOLM_SMI_eServiceID_SPDUExchangeStarted =               0xA2,
    IOLM_SMI_eServiceID_VerifyRecordSent =                  0xA3,
    IOLM_SMI_eServiceID_TestCommand =                       0xA4,
#ifdef IOLM_SGI_ENABLED
    IOLM_SMI_eServiceID_GatewayManagerIdentification =      0xB0,
    IOLM_SMI_eServiceID_GatewayManagerStatus =              0xB1,
    IOLM_SMI_eServiceID_RegisterClient =                    0xB2,
    IOLM_SMI_eServiceID_SetClientName =                     0xB3,
    IOLM_SMI_eServiceID_GetClientName =                     0xB4,
    IOLM_SMI_eServiceID_SetAccessRights =                   0xB6,
    IOLM_SMI_eServiceID_GetAccessRights =                   0xB7,
    IOLM_SMI_eServiceID_SetPDOutAccessRight =               0xB9,
    IOLM_SMI_eServiceID_SubscribeForEvent =                 0xBA,
    IOLM_SMI_eServiceID_GetEventSubscription =              0xBB,
    IOLM_SMI_eServiceID_TemporaryPDOutAccess =              0xBD,
    IOLM_SMI_eServiceID_ExclusiveAcyclicAccess =            0xBE,
    IOLM_SMI_eServiceID_ForcePDIn =                         0xBF,
    IOLM_SMI_eServiceID_SetGWMAccessRights  =               0xC0,
    IOLM_SMI_eServiceID_GetGWMAccessRights =                0xC1,
#endif
    IOLM_SMI_eServiceID_KeepAlive =                         0xFF,
}IOLM_SMI_EServiceID;

/**
\brief SMI ArgBlock IDs.

Coding: 0x N4 N3 N2 N1

N4 - Service group:
- 0: Identification (Master)
- 1: Process Data (PD)
- 2: Reserved
- 3: On-request Data (OD)
- 4-6: Reserved
- 7: Mixed system features
- 8: Configurations
- 9: Status information (port, Device, etc.)
- A: Events
- B-E: Manufacturer specific
- F: SMI management

N3 - Domain:
- 0: General
- 1: Safety extension
- 2: Wireless extension
- 3-E: Reserved
- F: SMI management

N2/N1
- Assigned by domain

For a list of all currently available ArgBlock IDs, see the overview above.
*/
typedef IOL_ENUM_DECL IOLM_SMI_EArgBlockID
{
    IOLM_SMI_eArgBlockID_Reserved =                     0x0000,

    // Identification
    IOLM_SMI_eArgBlockID_MasterIdent =                  0x0001,
    IOLM_SMI_eArgBlockID_FSMasterAccess =               0x0100,
    IOLM_SMI_eArgBlockID_FSCPAuthenticity =             0x0101,
    IOLM_SMI_eArgBlockID_WMasterConfig =                0x0200,

    // Process Data
    IOLM_SMI_eArgBlockID_PDIn =                         0x1001,
    IOLM_SMI_eArgBlockID_PDOut =                        0x1002,
    IOLM_SMI_eArgBlockID_PDInOut =                      0x1003,
    IOLM_SMI_eArgBlockID_SPDUIn =                       0x1101,
    IOLM_SMI_eArgBlockID_SPDUOut =                      0x1102,
    IOLM_SMI_eArgBlockID_PDInIQ =                       0x1FFE,
    IOLM_SMI_eArgBlockID_PDOutIQ =                      0x1FFF,

    // On-request Data
    IOLM_SMI_eArgBlockID_OnRequestDataWrite =           0x3000,
    IOLM_SMI_eArgBlockID_OnRequestDataRead =            0x3001,

    // Mixed System Features
    IOLM_SMI_eArgBlockID_DS_Data =                      0x7000,
    IOLM_SMI_eArgBlockID_DeviceParBatch =               0x7001,
    IOLM_SMI_eArgBlockID_IndexList =                    0x7002,
    IOLM_SMI_eArgBlockID_PortPowerOffOn =               0x7003,
    IOLM_SMI_eArgBlockID_WPortPairing =                 0x7200,

    // Configuration
    IOLM_SMI_eArgBlockID_PortConfigList =               0x8000,
    IOLM_SMI_eArgBlockID_FSPortConfigList =             0x8100,
    IOLM_SMI_eArgBlockID_WTrackConfigList =             0x8200,     //outdated: left for compatibility
    IOLM_SMI_eArgBlockID_WPortConfigList =              0x8201,     //outdated: left for compatibility
    IOLM_SMI_eArgBlockID_WScan =                        0x8202,     //outdated: left for compatibility

    // Status Information
    IOLM_SMI_eArgBlockID_PortStatusList =               0x9000,     
    IOLM_SMI_eArgBlockID_FSPortStatusList =             0x9100,
    IOLM_SMI_eArgBlockID_WTrackStatusList =             0x9200,     //outdated: left for compatibility   
    IOLM_SMI_eArgBlockID_ScanResult =                   0x9201,     //outdated: left for compatibility
    IOLM_SMI_eArgBlockID_WPortStatusList =              0x9202,     //outdated: left for compatibility   
    
    // new or changed ArgBlocks for wireless specification V1.1.3
    IOLM_SMI_V113_eArgBlockID_WPortConfigList =         0x8200,     //according to wireless specification V1.1.3
    IOLM_SMI_V113_eArgBlockID_WScanConfigList =         0x8201,     //according to wireless specification V1.1.3
    IOLM_SMI_V113_eArgBlockID_WPortStatusList =         0x9200,     //ID changed to 0x9200, according to wireless specification V1.1.3
    IOLM_SMI_V113_eArgBlockID_WScanStatusList =         0x9201,     //new ArgBlock structure, according to wireless specification V1.1.3
    IOLM_SMI_V113_eArgBlockID_WTrackStatusList =        0x9202,     //ID changed to 0x9202, according to wireless specification V1.1.3
    IOLM_SMI_V113_eArgBlockID_WQualityStatusList =      0x9203,     //according to wireless specification V1.1.3

    // Events
    IOLM_SMI_eArgBlockID_DeviceEvent =                  0xA000,
    IOLM_SMI_eArgBlockID_PortEvent =                    0xA001,

    // Manufacturer specific
    IOLM_SMI_eArgBlockID_PDInAllConnectedPorts =        0xB001,
    IOLM_SMI_eArgBlockID_PDInAllUpdatedPorts =          0xB002,
    IOLM_SMI_eArgBlockID_PDOutMultiplePorts =           0xB003,
    IOLM_SMI_eArgBlockID_PDInOutMultiplePorts =         0xB004,
    IOLM_SMI_eArgBlockID_FSPDIn =                       0xB101,
    IOLM_SMI_eArgBlockID_FSPDOut =                      0xB102,
    IOLM_SMI_eArgBlockID_StackInformation =             0xE000,
    IOLM_SMI_eArgBlockID_DataLog =                      0xE001,
    IOLM_SMI_eArgBlockID_TestCommand =                  0xE002,
 
#ifdef IOLM_SGI_ENABLED
    // SMI Management
    IOLM_SMI_eArgBlockID_GatewayManagerIdentification = 0xFF00,
    IOLM_SMI_eArgBlockID_GatewayManagerStatus =         0xFF01,
    IOLM_SMI_eArgBlockID_RegisterClient =               0xFF02,
    IOLM_SMI_eArgBlockID_SetClientName =                0xFF03,
    IOLM_SMI_eArgBlockID_GetClientNameReq =             0xFF04,
    IOLM_SMI_eArgBlockID_GetClientNameResp =            0xFF05,
    IOLM_SMI_eArgBlockID_SetAccessRights =              0xFF06,
    IOLM_SMI_eArgBlockID_GetAccessRightsReq =           0xFF07,
    IOLM_SMI_eArgBlockID_GetAccessRightsResp =          0xFF08,
    IOLM_SMI_eArgBlockID_SetPDOutAccessRight =          0xFF09,
    IOLM_SMI_eArgBlockID_EventSubscription =            0xFF0A,
    IOLM_SMI_eArgBlockID_EventSubscriptionReq =         0xFF0B,
    IOLM_SMI_eArgBlockID_EventSubscriptionResp =        0xFF0C,
    IOLM_SMI_eArgBlockID_TemporaryPDOutAccess =         0xFF0D,
    IOLM_SMI_eArgBlockID_ExclusiveAcyclicAccess =       0xFF0E,
    IOLM_SMI_eArgBlockID_ForcePDIn =                    0xFF0F,
    IOLM_SMI_eArgBlockID_SetGWMAccessRights =           0xFF10,
    IOLM_SMI_eArgBlockID_GetGWMAccessRightsReq =        0xFF11,
    IOLM_SMI_eArgBlockID_GetGWMAccessRightsResp =       0xFF12,
#endif
    IOLM_SMI_eArgBlockID_VoidBlock =                    0xFFF0,
    IOLM_SMI_eArgBlockID_JobError =                     0xFFFF,
}IOLM_SMI_EArgBlockID;

/**
\brief SMI Master types.
*/
typedef IOL_ENUM_DECL IOLM_SMI_EMasterTypes
{
    IOLM_SMI_eMasterTypes_Unspecific =                  0,
    IOLM_SMI_eMasterTypes_Reserved =                    1,
    IOLM_SMI_eMasterTypes_Master_acc =                  2,
    IOLM_SMI_eMasterTypes_FS_Master =                   3,
    IOLM_SMI_eMasterTypes_W_Master =                    4,
}IOLM_SMI_EMasterTypes;

/**
\brief SMI port types.
*/
typedef IOL_ENUM_DECL IOLM_SMI_EPortTypes
{
    IOLM_SMI_ePortTypes_ClassA =                        0,
    IOLM_SMI_ePortTypes_ClassAWithPowerOffOn =          1,
    IOLM_SMI_ePortTypes_ClassB =                        2,
    IOLM_SMI_ePortTypes_FSPortAWithoutOSSDe =           3,
    IOLM_SMI_ePortTypes_FSPortAwithOSSDe =              4,
    IOLM_SMI_ePortTypes_FSPortB =                       5,
    IOLM_SMI_ePortTypes_WMaster =                       6,
}IOLM_SMI_EPortTypes;

/**
\brief SMI port mode.
*/
typedef IOL_ENUM_DECL IOLM_SMI_EPortMode
{
    IOLM_SMI_ePortMode_DEACTIVATED =                    0,
    IOLM_SMI_ePortMode_IOL_MANUAL =                     1,
    IOLM_SMI_ePortMode_IOL_AUTOSTART =                  2,
    IOLM_SMI_ePortMode_DI_CQ =                          3,
    IOLM_SMI_ePortMode_DO_CQ =                          4,
    
    IOLM_SMI_ePortMode_SAFETYCOM =                      49,
    IOLM_SMI_ePortMode_MIXEDSAFETYCOM =                 50,
    IOLM_SMI_ePortMode_OSSDE =                          51,

    IOLM_SMI_ePortMode_CYCLIC_AUTO =                    52,
    IOLM_SMI_ePortMode_CYCLIC =                         53,
    IOLM_SMI_ePortMode_ROAMING_AUTO =                   54,
    IOLM_SMI_ePortMode_ROAMING =                        55,
}IOLM_SMI_EPortMode;

/**
\brief SMI port mode (only used for compatibility to old V1.1).
*/
typedef IOL_ENUM_DECL IOLM_SMI_EPortModeOld
{
    IOLM_SMI_ePortModeOld_DEACTIVATED =                 0,
    IOLM_SMI_ePortModeOld_IOL_MANUAL =                  1,
    IOLM_SMI_ePortModeOld_IOL_AUTOSTART =               2,
    IOLM_SMI_ePortModeOld_DI_CQ =                       3,
    IOLM_SMI_ePortModeOld_DO_CQ =                       4,

    IOLM_SMI_ePortModeOld_SAFETYCOM =                   49,
    IOLM_SMI_ePortModeOld_MIXEDSAFETYCOM =              50,
    IOLM_SMI_ePortModeOld_OSSDE =                       51,

    IOLM_SMI_ePortModeOld_CYCLIC =                      52,
    IOLM_SMI_ePortModeOld_ROAMING =                     53,
}IOLM_SMI_EPortModeOld;

/**
\brief SMI pairing commands.
*/
typedef IOL_ENUM_DECL IOLM_SMI_EPairCmd
{
    IOLM_SMI_ePairCmd_Unpairing =                       0,
    IOLM_SMI_ePairCmd_PairingUnique =                   1,
    IOLM_SMI_ePairCmd_PairingButton =                   2,
    IOLM_SMI_ePairCmd_AbortPairing =                    4,
}IOLM_SMI_EPairCmd;

/**
\brief SMI port validation and backup.
*/
typedef IOL_ENUM_DECL IOLM_SMI_EValidationBackup
{
    IOLM_SMI_ePortValBack_NoDeviceCheck =               0,
    IOLM_SMI_ePortValBack_V10 =                         1,
    IOLM_SMI_ePortValBack_V11 =                         2,
    IOLM_SMI_ePortValBack_V11_BackupRestore =           3,
    IOLM_SMI_ePortValBack_V11_Restore =                 4,
}IOLM_SMI_EValidationBackup;

/**
\brief SMI port IQ behavior.
*/
typedef IOL_ENUM_DECL IOLM_SMI_EIQBehavior
{
    IOLM_SMI_ePortIQBehavior_NotSupported =             0,
    IOLM_SMI_ePortIQBehavior_DigitalInput =             1,
    IOLM_SMI_ePortIQBehavior_DigitalOutput =            2,
    IOLM_SMI_ePortIQBehavior_AnalogInput =              3,
    IOLM_SMI_ePortIQBehavior_AnalogOutput =             4,
    IOLM_SMI_ePortIQBehavior_Power2 =                   5,
}IOLM_SMI_EIQBehavior;

/**
\brief SMI port status.
*/
typedef IOL_ENUM_DECL IOLM_SMI_EPortStatus
{
    IOLM_SMI_ePortStatus_NO_DEVICE =                    0,
    IOLM_SMI_ePortStatus_DEACTIVATED =                  1,
    IOLM_SMI_ePortStatus_PORT_DIAG =                    2,
    IOLM_SMI_ePortStatus_PREOPERATE =                   3,
    IOLM_SMI_ePortStatus_OPERATE =                      4,
    IOLM_SMI_ePortStatus_DI_CQ =                        5,
    IOLM_SMI_ePortStatus_DO_CQ =                        6,
    IOLM_SMI_ePortStatus_OSSDE =                        7,
    IOLM_SMI_ePortStatus_SPDU_EXCHANGE =                8,
    IOLM_SMI_ePortStatus_PAIRING_FAULT =                10,
    IOLM_SMI_ePortStatus_PORT_POWER_OFF =               254,
    IOLM_SMI_ePortStatus_NOT_AVAILABLE =                255,
}IOLM_SMI_EPortStatus;


/**
\brief SMI service track mode.
*/
typedef IOL_ENUM_DECL IOLM_SMI_ServiceTrackMode
{
    IOLM_SMI_ServiceTrackMode_CYCLIC = 0,
    IOLM_SMI_ServiceTrackMode_ROAMING = 1,
}IOLM_SMI_ServiceTrackMode;

/**
\brief SMI track mode.
*/
typedef IOL_ENUM_DECL IOLM_SMI_ETrackMode
{
    IOLM_SMI_eTrackMode_STOP =                          0,
    IOLM_SMI_eTrackMode_CYCLIC =                        1,
    IOLM_SMI_eTrackMode_ROAMING =                       2,
    IOLM_SMI_eTrackMode_SERVICE =                       3,
}IOLM_SMI_ETrackMode;

/**
\brief SMI track status.
*/
typedef IOL_ENUM_DECL IOLM_SMI_ETrackStatus
{
    IOLM_SMI_eTrackStatus_STOP =                        0,
    IOLM_SMI_eTrackStatus_CYCLIC =                      1,
    IOLM_SMI_eTrackStatus_ROAMING =                     2,
    IOLM_SMI_eTrackStatus_SCAN =                        3,
    IOLM_SMI_eTrackStatus_PAIRING =                     4,
}IOLM_SMI_ETrackStatus;

/**
\brief SMI scan behavior.
*/
typedef IOL_ENUM_DECL IOLM_SMI_EScanBehavior
{
    IOLM_SMI_eScanBehavior_POLL_RESULT = 0,
    IOLM_SMI_eScanBehavior_SINGLE_RESULT_BROADCAST = 1,
    IOLM_SMI_eScanBehavior_FINAL_RESULT_BROADCAST = 2,
}IOLM_SMI_EScanBehavior;

/**
\brief SMI scan status.
*/
typedef IOL_ENUM_DECL IOLM_SMI_EScanStatus
{
    IOLM_SMI_eScanStatus_NOT_PERFORMED_YET = 0,
    IOLM_SMI_eScanStatus_IN_PROGRESS = 1,
    IOLM_SMI_eScanStatus_ENDED = 2,
}IOLM_SMI_EScanStatus;

/**
\brief SMI port power mode.
*/
typedef IOL_ENUM_DECL IOLM_SMI_EPortPowerMode
{
    IOLM_SMI_ePortPowerMode_ONE_TIME_SWITCH_OFF =       0,
    IOLM_SMI_ePortPowerMode_OFF =                       1,
    IOLM_SMI_ePortPowerMode_ON =                        2,
}IOLM_SMI_EPortPowerMode;

#define IOLM_SMI_PQI_PQ                                 (1<<7)  // Valid PD Input
#define IOLM_SMI_PQI_DEVERR                             (1<<6)  // Device Error
#define IOLM_SMI_PQI_DEVCOM                             (1<<5)  // Device Com available

////////////////////////////////////////////////
/**
\brief This structure is used as general and void ArgBlock.

The following data and the ArgBlockID(#IOLM_SMI_EArgBlockID)
depend on the specific use case.
*/
typedef struct IOLM_SMI_SVoidArgBlock
{
    INT16U u16ArgBlockID;  /**< \brief Big endian. */
    //void * pvData; // following data depends on Block ID
}IOLM_SMI_SVoidArgBlock;
typedef IOLM_SMI_SVoidArgBlock IOLM_SMI_SGeneralArgBlock;

/**
\brief This structure is used as JobError ArgBlock.

*/
typedef struct IOLM_SMI_SJobError
{
    INT16U u16ArgBlockID;  /**< \brief Big endian. */
    INT16U u16ExpArgBlockID; /**< \brief Expected ArgBlockID of result. */
    INT16U u16ErrorCode; /**< \brief ErrorCode. */
}IOLM_SMI_SJobError;

/**
\brief This structure is used to get Master identification.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_MasterIdent = 0x0000.
*/
typedef struct IOLM_SMI_SMasterident
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT16U u16VendorID; /**< \brief Big endian. */
    INT32U u32MasterID; /**< \brief Big endian. */
    INT8U u8MasterType;
    INT8U u8Features_1;
    INT8U u8Features_2;
    INT8U u8MaxNumberOfPorts;
    // PortTypes
}IOLM_SMI_SMasterident;

/**
\brief This structure is used to get FS Master access.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_FSMasterAccess = 0x0100.
*/
typedef struct IOLM_SMI_SFSMasterAccess
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT32U u32FSMasterPassword; /**< \brief Big endian. */
    INT32U u32FSResetMasterPW; /**< \brief Big endian. */
    INT8U  u8FSUserRole;
}IOLM_SMI_SFSMasterAccess;

/**
\brief This structure is used to retrieve FSCPAuthenticity.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_FSCPAuthenticity = 0x0101.
*/
typedef struct IOLM_SMI_SFSCPAuthenticity
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT32U u32FSCPAuthenticity1; /**< \brief Big endian. */
    INT32U u32FSCPAuthenticity2; /**< \brief Big endian. */
}IOLM_SMI_SFSCPAuthenticity;

#ifdef IOLM_WIRELESS
/**
\brief This structure is used for the wireless Master configuration.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_WMasterConfig = 0x0200.
*/
typedef struct IOLM_SMI_SWMasterConfigList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8WMasterID; /**< \brief 1 - 29. */
    INT16U u16AdvancedConnectivity; /**< \brief Enable bit per functionality. 0 = disabled, 1 = enabled. */
    INT8U au8Blocklist[10]; /**< \brief Disabled frequencies, bit per frequency. */
    INT8U u8PairingTimeout; /**< \brief Pairing timeout in seconds */
    INT8U u8Reserved;
    INT8U u8ServiceTrackN; /**< \brief Track number used for service requests(Scan / Pairing / Roaming). Service track must be enabled. Values: 1 to 5 (0 only allowed if all tracks are disabled) */
    INT8U u8ServiceTrackMode; /**< \brief Mode of service track  0: CYCLIC  1: ROAMING */
    INT8U u8NumberOfTracks; /**< \brief Number of tracks */
    INT8U au8TrackTxPower[IOLM_NUM_OF_TRACKS]; /**< \brief Array for transmission power of all Tracks. Values: 0: Disabled,  1 to 0x1F (-20dBm to 10dBm) */
} IOLM_SMI_SWMasterConfigList;
#endif

#if IOLM_SMI_SUPPORT_OLD_SERVICES == 1
/**
\brief This structure is obsolete.  It used for the wireless Master configuration before V1.1.3 release.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_WMasterConfig = 0x0200.
*/
typedef struct IOLM_SMI_SWMasterConfigListOld
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8MasterID; /**< \brief 1 - 29. */
    INT8U u8AHTEnable; /**< \brief 0 = disabled, 1 = enabled. */
    INT8U au8Blacklist[10];
    INT16U u16PairingTimeout; /**< \brief Pairing timeout in seconds */
} IOLM_SMI_SWMasterConfigListOld;
#endif

/**
\brief This structure is used for the wireless port configuration.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_V113_eArgBlockID_WPortConfigList = 0x8200.
*/
typedef struct IOLM_SMI_SWPortConfigList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8PortMode;
    INT8U u8ValidationBackup;
    INT8U u8IQBehavior;
    INT8U u8PortCycleTime;
    INT16U u16VendorID; /**< \brief Big endian. */
    INT32U u32DeviceID; /**< \brief Big endian. */
    // Wireless extension
    INT8U u8Slot; /**< \brief slot number starting with 0 */
    INT8U u8Track; /**< \brief track number starting with 1 */
    INT8U u8DeviceTxPower;
    INT8U u8MaxRetry;
    INT8U u8ImaTimeBase /**< \brief Time Base 0x01 - 0x04. */;
    INT8U u8ImaTimeMulti /**< \brief Big endian 0x01 - 0xFF. */;
    INT8U u8SlotType; /**< \brief 0 = SSLOT / 1 = DSLOT. */
    INT8U u8LowPowerDevice; /**< \brief 0 = Normal Device / 1 = Low Power. */
    INT8U u8MaxPDSegLength;
    INT8U u8WMasterCycleTimePDOut;
    INT8U u8WMasterCycleTimePDIn;
    INT8U au8UniqueID[9];
}IOLM_SMI_SWPortConfigList;

#if IOLM_SMI_SUPPORT_OLD_SERVICES == 1
/**
\brief This structure is obsolete.  It used for the wireless port configuration before V1.1.3 release.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_WPortConfigList = 0x8201.
*/
typedef struct IOLM_SMI_SWPortConfigListOld
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8PortMode;
    INT8U u8ValidationBackup;
    INT8U u8IQBehavior;
    INT8U u8PortCycleTime;
    INT16U u16VendorID; /**< \brief Big endian. */
    INT32U u32DeviceID; /**< \brief Big endian. */
    // Wireless extension
    INT8U u8Slot; /**< \brief slot number starting with 0 */
    INT8U u8Track; /**< \brief track number starting with 1 */
    INT8U u8DeviceTxPower;
    INT8U u8MaxRetry;
    INT16U u16ImaTime /**< \brief Big endian. */;
    INT8U u8SlotType; /**< \brief 0 = SSLOT / 1 = DSLOT. */
    INT8U u8LowPowerDevice; /**< \brief 0 = Normal Device / 1 = Low Power. */
    INT8U u8MaxPDSegLength;
    INT8U au8UniqueID[9];
}IOLM_SMI_SWPortConfigListOld;
#endif

/**
\brief This structure is used to store FS port configuration.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_FSPortConfigList = 0x8100.
*/
typedef struct IOLM_SMI_SFSPortConfigList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8PortMode;
    INT8U u8ValidationBackup;
    INT8U u8IQBehavior;
    INT8U u8PortCycleTime;
    INT16U u16VendorID; /**< \brief Big endian. */
    INT32U u32DeviceID; /**< \brief Big endian. */
    INT16U u16FSPTime2Ready;
    INT32U u32FSCPAuthenticity1;
    INT32U u32FSCPAuthenticity2;
    INT8U u8FSPPort;
    INT16U u16FSPAuthentCRC;
    INT8U u8FSPProtVersion;
    INT8U u8FSPProtMode;
    INT16U u16FSPWatchdogTime;
    INT16U u16FSPIOStructCRC;
    INT32U u32FSPTechParCRC;
    INT16U u16FSPProtParCRC;
    INT8U u8IODescVersion;
    INT8U u8SPDUInLength;
    INT8U u8TotalOfInBits;
    INT8U u8TotalOfInOctets;
    INT8U u8TotalOfInInt16;
    INT8U u8TotalOfInInt32;
    INT8U u8SPDUOutLength;
    INT8U u8TotalOfOutBits;
    INT8U u8TotalOfOutOctets;
    INT8U u8TotalOfOutInt16;
    INT8U u8TotalOfOutInt32;
}IOLM_SMI_SFSPortConfigList;

/**
\brief This structure is used to store port configuration.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_PortConfigList = 0x8000.
*/
typedef struct IOLM_SMI_SPortConfigList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8PortMode; /**< \brief See #IOLM_SMI_EPortMode. */
    INT8U u8ValidationBackup; /**< \brief See #IOLM_SMI_EValidationBackup. */
    INT8U u8IQBehavior; /**< \brief See #IOLM_SMI_EIQBehavior. */
    INT8U u8PortCycleTime;
    INT16U u16VendorID; /**< \brief Big endian */
    INT32U u32DeviceID; /**< \brief Big endian */
}IOLM_SMI_SPortConfigList;

/**
\brief This structure is used to store wireless port status.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_WPortStatusList = 0x9002.
*/
typedef struct IOLM_SMI_SWPortStatusList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8PortStatusInfo; /**< \brief IOLM_SMI_EPortStatus. */
    INT8U u8PortQualityInfo;
    INT8U u8RevisionID;
    INT8U u8ConnectionQuality;
    INT8U u8MasterCycleTime;// old u8MasterCycleTime;
    INT8U u8InputDataLength;
    INT8U u8OutputDataLength;
    INT16U u16VendorID; /**< \brief Big endian */
    INT32U u32DeviceID; /**< \brief Big endian */
    INT8U u8NumberOfDiags;
    INT8U au8DiagData[3 * IOLM_SMI_MAX_DIAG_ENTRIES];
    INT8U u8WMasterCycleTimePDOut;
    INT8U u8WMasterCycleTimePDIn;
}IOLM_SMI_SWPortStatusList;

/**
\brief This structure is used to store wireless port status in V1.1.3.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_WPortStatusList = 0x9200.
*/
typedef struct IOLM_SMI_V113_SWPortStatusList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8PortStatusInfo; /**< \brief IOLM_SMI_EPortStatus. */
    INT8U u8PortQualityInfo;
    INT8U u8RevisionID;
    INT8U au8Reserved[2];
    INT8U u8InputDataLength;
    INT8U u8OutputDataLength;
    INT16U u16VendorID; /**< \brief Big endian */
    INT32U u32DeviceID; /**< \brief Big endian */
    INT8U u8MasterCycleTimePDOut;
    INT8U u8MasterCycleTimePDIn;
    INT8U u8NumberOfDiags;
    INT8U au8DiagData[3 * IOLM_SMI_MAX_DIAG_ENTRIES];
}IOLM_SMI_V113_SWPortStatusList;

/**
\brief This structure is used to store track status.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_WTrackStatusList = 0x9003.
*/
typedef struct IOLM_SMI_SWTrackStatusList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8TrackStatus; /**< \brief IOLM_SMI_ETrackStatus. */
    INT8U u8TXPower; /**< \brief 0 - 31. */
} IOLM_SMI_SWTrackStatusList;

#ifdef IOLM_WIRELESS
/**
\brief This structure is used to store a single track status entry.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_V113_eArgBlockID_WTrackStatusList = 0x9202.
*/
typedef struct IOLM_SMI_V113_SWTrackStatusEntry
{
    INT8U u8TrackMode; /**< \brief IOLM_SMI_ETrackStatus. */
    INT8U u8TrackTXPower; /**< \brief 0 - 31. */
} IOLM_SMI_V113_SWTrackStatusEntry;

/**
\brief This structure is used to store track status.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_V113_eArgBlockID_WTrackStatusList = 0x9202.
*/
typedef struct IOLM_SMI_V113_SWTrackStatusList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    IOLM_SMI_V113_SWTrackStatusEntry asTrackStatus[IOLM_NUM_OF_TRACKS]; /**< \brief IOLM_SMI_V113_SWTrackStatusEntry. */
} IOLM_SMI_V113_SWTrackStatusList;
#endif


#ifdef IOLM_WIRELESS
/**
\brief This structure is used to store a single track status entry.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_V113_eArgBlockID_WTrackStatusList = 0x9202.
*/
typedef struct IOLM_SMI_SWQualityStatusEntry
{
    INT8U u8PortLQI_M; /**< \brief 0...100 percent. */
    INT8S s8PortRSSI_M;
    INT8U u8PortLQI_D; /**< \brief 0...100 percent. */
    INT8S s8PortRSSI_D;
} IOLM_SMI_SWQualityStatusEntry;

/**
\brief This structure is used to store track status.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_V113_eArgBlockID_WTrackStatusList = 0x9202.
*/
typedef struct IOLM_SMI_SWQualityStatusList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    IOLM_SMI_SWQualityStatusEntry asPortQuality[8 * IOLM_NUM_OF_TRACKS]; /**< \brief IOLM_SMI_SWQualityStatusEntry. */
} IOLM_SMI_SWQualityStatusList;
#endif


/**
\brief This structure is used to trigger a scan request.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_WScanList.
*/
typedef struct IOLM_SMI_SWScanConfigList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8TXPower; /**< \brief 0 - 31. */
    INT8U u8ScanBehavior; /**< \brief Behavior how scan results are transmitted. */
} IOLM_SMI_SWScanConfigList;

/**
\brief This structure is used to trigger a pairing request.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_WPairing.
*/
typedef struct IOLM_SMI_SWPairingList
{
    INT16U u16ArgBlockID; /**< \brief Big endian */
    INT8U u8PairingCommand; /**< \brief according #IOLM_SMI_EPairCmd  */
} IOLM_SMI_SWPairingList;

/**
\brief This structure is used for the track configuration.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_WTrackConfigList = 0x8003.
*/
typedef struct IOLM_SMI_SWTrackConfigList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8TrackMode; /**< \brief IOLM_SMI_ETrackMode. */
    INT8U u8TXPower; /**< \brief 0 - 31. */
} IOLM_SMI_SWTrackConfigList;

/**
\brief This structure is used to store FS port status.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_FSPortStatusList = 0x9100.
*/
typedef struct IOLM_SMI_SFSPortStatusList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8PortStatusInfo; /**< \brief IOLM_SMI_EPortStatus. */
    INT8U u8PortQualityInfo;
    INT8U u8RevisionID;
    INT8U u8TransmissionRate;
    INT8U u8MasterCycleTime;
    INT8U u8InputDataLength;
    INT8U u8OutputDataLength;
    INT16U u16VendorID; /**< \brief Big endian */
    INT32U u32DeviceID; /**< \brief Big endian */
    INT8U u8NumberOfDiags;
    INT8U au8DiagData[3 * IOLM_SMI_MAX_DIAG_ENTRIES];
}IOLM_SMI_SFSPortStatusList;

/**
\brief This structure is used to store port status.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_PortStatusList = 0x9000.
*/
typedef struct IOLM_SMI_SPortStatusList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8PortStatusInfo; /**< \brief #IOLM_SMI_EPortStatus. */
    INT8U u8PortQualityInfo;
    INT8U u8RevisionID;
    INT8U u8TransmissionRate;
    INT8U u8MasterCycleTime;
    INT8U u8InputDataLength;
    INT8U u8OutputDataLength;
    INT16U u16VendorID; /**< \brief Big endian. */
    INT32U u32DeviceID; /**< \brief Big endian. */
    INT8U u8NumberOfDiags;
    INT8U au8DiagData[3 * IOLM_SMI_MAX_DIAG_ENTRIES];
}IOLM_SMI_SPortStatusList;

/**
\brief This structure is used for Process Data input.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_PDIn = 0x1001.
*/
typedef struct IOLM_SMI_SPDIn
{
    INT16U u16ArgBlockID;       /**< \brief Big endian. */
    INT8U u8PQI;                /**< \brief Process Data qualifier. */
    INT8U u8InputDataLength;    /**< \brief Input data length in bytes. */
    INT8U au8Data[32];          /**< \brief 32 bytes. */
}IOLM_SMI_SPDIn;
#define IOLM_SMI_ARGBLOCK_PDIN_LEN(Datalen) (sizeof(IOLM_SMI_SPDIn) - 32 + Datalen)

/**
\brief This structure is used for Process Data output.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_PDOut = 0x1002.
*/
typedef struct IOLM_SMI_SPDOut
{
    INT16U u16ArgBlockID;       /**< \brief Big endian. */
    INT8U u8OE;                 /**< \brief Output enable. */
    INT8U u8OutputDataLength;   /**< \brief Output data length in bytes. */
    INT8U au8Data[32];          /**< \brief 32 bytes. */
}IOLM_SMI_SPDOut;

#define IOLM_SMI_ARGBLOCK_PDOUT_LEN(Datalen) (sizeof(IOLM_SMI_SPDOut) - 32 + Datalen)

/**
\brief This structure is used for IQ input.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_PDInIQ = 0x1FFE.
*/
typedef struct IOLM_SMI_SPDInIQ
{
    INT16U u16ArgBlockID;       /**< \brief Big endian. */
    INT8U u8IQ;                 /**< \brief Input IQ signal. */
}IOLM_SMI_SPDInIQ;

/**
\brief This structure is used for IQ output.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_PDOutIQ = 0x1FFF.
*/
typedef struct IOLM_SMI_SPDOutIQ
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8IQ; /**< \brief Output IQ signal. */
}IOLM_SMI_SPDOutIQ;

/**
\brief This structure is used for SPDU input.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_SPDUIn = 0x1101.
*/
typedef struct IOLM_SMI_SSPDUIn
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U au8Data[32]; /**< \brief 32 bytes. */
}IOLM_SMI_SSPDUIn;
#define IOLM_SMI_ARGBLOCK_SSPDUIN_LEN(Datalen) (sizeof(IOLM_SMI_SSPDUIn) - 32 + Datalen)

/**
\brief This structure is used for SPDU output.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_SPDUOut = 0x1102.
*/
typedef struct IOLM_SMI_SSPDUOut
{
    INT16U u16ArgBlockID; /**< \brief Big endian */
    INT8U au8Data[32]; /**< \brief 32 bytes. */
}IOLM_SMI_SSPDUOut;

#define IOLM_SMI_ARGBLOCK_SSPDUOUT_LEN(Datalen) (sizeof(IOLM_SMI_SSPDUOut) - 32 + Datalen)

/**
\brief This structure is used for Process Data input and output read back.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_PDInOut = 0x1003.
*/
typedef struct IOLM_SMI_SPDInOut
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8PQI; /**< \brief Process Data qualifier. */
    INT8U u8OE; /**< \brief Output enable. */
    INT8U au8Data[2 * (32 + 1)]; /**< \brief input len + data + output len + data */
}IOLM_SMI_SPDInOut;
#define IOLM_SMI_ARGBLOCK_PDINOUT_LEN(InputLen, OutputLen) (4 + 2 + InputLen + OutputLen)

/**
\brief This generic structure is used for events.

*/
typedef struct IOLM_SMI_SGenericEvent
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8EventQualifier;
    INT16U u16EventCode; /**< \brief Big endian. */
}IOLM_SMI_SGenericEvent;

/**
\struct IOLM_SMI_SDeviceEvent
\brief This structure is used for Device Events (remote).

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_DeviceEvent = 0xA000.
*/
typedef struct IOLM_SMI_SGenericEvent IOLM_SMI_SDeviceEvent;

/**
\struct IOLM_SMI_SPortEvent
\brief This structure is used for port events (local).

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_PortEvent = 0xA001.
*/
typedef struct IOLM_SMI_SGenericEvent IOLM_SMI_SPortEvent;

/**
\brief This structure is used for scan results.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_ScanResult = 0xA002.
*/
typedef struct IOLM_SMI_SScanResult
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8SlotType;
    INT8U u8RevisionID;
    INT8U au8UniqueID[IOL_UNIQUEID_SIZE];
} IOLM_SMI_SScanResult;

/**
\brief This structure is used for a single scan result.

*/
typedef struct IOLM_SMI_SScanStatusResult
{
    INT8U u8ResSlotType; /**< \brief Slot type */
    INT8U au8ResUniqueID[9]; /**< \brief Unique ID */
    INT8U u8ResRevID;   /**< \brief Revision ID */
}IOLM_SMI_SScanStatusResult;

/**
\brief This structure is used for the Scan Status List.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_WScanStatusList = 0x9201.
*/
typedef struct IOLM_SMI_SWScanStatusList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8ScanStatus; /**< \brief Scan status. */
    INT8U u8ScanResults; /**< \brief Number of scan results (max 255). */
    IOLM_SMI_SScanStatusResult asResult[IOLM_MAX_SCAN_RESULTS]; /**< \brief array of scan results */
}IOLM_SMI_SWScanStatusList;

/**
\brief This structure is used for the Scan Status List of a single scan result.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_WScanStatusList = 0x9201.
*/
typedef struct IOLM_SMI_SWSingleScanStatusList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8ScanStatus; /**< \brief Scan status. */
    INT8U u8ScanResults; /**< \brief Number of scan results (max 255). */
    IOLM_SMI_SScanStatusResult asResult[1]; /**< \brief array of scan results */
}IOLM_SMI_SWSingleScanStatusList;

/**
\brief This structure is used for On-request Data.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_OnRequestData = 0x3000.
*/
typedef struct IOLM_SMI_SOnRequestData
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT16U u16Index; /**< \brief Big endian. */
    INT8U u8Subindex; 
    INT8U au8Data[IOLM_MAX_ISDU_LENGTH];
}IOLM_SMI_SOnRequestData;

#define IOLM_SMI_ARGBLOCK_ONREQ_LEN(Datalen) (sizeof(IOLM_SMI_SOnRequestData) - IOLM_MAX_ISDU_LENGTH + Datalen)

/**
\brief This structure is used for PortPowerOffOn requests.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_PortPowerOffOn = 0x7003.
*/
typedef struct IOLM_SMI_SPortPowerOffOn
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8PortPowerMode;
    INT16U u16PowerOffTime; /**< \brief Big endian. */
}IOLM_SMI_SPortPowerOffOn;

/**
\brief This structure is used for DataStorage Data.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_DS_Data = 0x7000.
*/
typedef struct IOLM_SMI_SDSData
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT32U u32ParamCheckSum; /**< \brief Big endian. */
    INT16U u16VendorID; /**< \brief Big endian. */
    INT32U u32DeviceID; /**< \brief Big endian. */
    INT16U u16FunctionID; /**< \brief Big endian. */
    INT8U u8DSData[IOLM_DS_MAX_SIZE];
}IOLM_SMI_SDSData;

/**
\brief SGI dependent types
*/
#ifdef IOLM_SGI_ENABLED
/**
\brief This structure is used for SGI manager control

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_SubscribeForEvent = 0xFF09.
*/
typedef struct IOLM_SGI_SGatewayManagerStatusReq
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8TargetClientID; /**< \brief ID of the client*/
}IOLM_SGI_SGatewayManagerStatusReq;

/**
\brief This structure is used for SGI manager control

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_SubscribeForEvent = 0xFF09.
*/
typedef struct IOLM_SGI_SGatewayManagerStatusList
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    IOLM_SGI_EGatewayStatusInfo sStatusInfo; /**< \brief Current status of the Gateway. */
    INT8U u8NumberConnectedClients; /**< \brief Number of clients connected. */
    INT8U u8ConnectedClient[255]; /**< \brief The pointer to the data, where the client Unique IDs are listed. */

}IOLM_SGI_SGatewayManagerStatusList;

/**
\brief This structure is used for SGI manager control

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_RegisterClient = 0xFF02.
*/
typedef struct IOLM_SGI_SRegisterClient
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8Register;     /**< \brief 1 - register current, 0 unregister client*/
    INT8U u8NameLength; /**< \brief The length of the client name*/
    CHAR8 ac8Name[IOLM_SGI_CLIENT_MAX_NAME_LENGTH]; /**< \brief A pointer to the data */
}IOLM_SGI_SRegisterClient;

/**
\brief This structure is used for SGI manager control

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_SetClientName = 0xFF03.
*/
typedef struct IOLM_SGI_SClientName
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8TargetClientID; /**< \brief not used*/
    INT8U u8NameLength; /**< \brief Length of the name */
    CHAR8 ac8Name[IOLM_SGI_CLIENT_MAX_NAME_LENGTH]; /**< \brief Pointer to the name */
}IOLM_SGI_SClientName;

/**
\brief This structure is used for SGI manager control

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_SetAccessRights = 0xFF05.
*/
typedef struct IOLM_SGI_SSetAccessRights
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8TargetClientID; /**< \brief ID of the client*/
    INT8U u8TargetPort; /**< \brief port to set the access */
    IOLM_SGI_EAccessCommand eAction; /**< \brief Action : Set, Reset or Update */
    INT32U u32Rights; /**< \brief The flags with the access rights*/
}IOLM_SGI_SSetAccessRights;

/**
\brief This structure is used service SGI_ReadAccessRights

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_RequestAccessRights = 0xFF06.
*/
typedef struct IOLM_SGI_GetAccessRights
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8TargetClientID; /**< \brief ID of the client*/
    INT8U u8TargetPort;     /**< \brief a port to be read out*/
    INT32U u32Rights; /**< \brief The flags with the access rights*/
}IOLM_SGI_SGetAccessRights;

/**
\brief This structure is used for SGI manager control

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_SetPDOutAccessRight = 0xFF08.
*/
typedef struct IOLM_SGI_SSetPDOutAccessRights
{
    INT16U u16ArgBlockID;   /**< \brief Big endian. */
    INT8U u8TargetClientID; /**< \brief ID of the client*/
}IOLM_SGI_SSetPDOutAccessRights;

/**
\brief This structure is used for SGI manager control

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_SubscribeForEvent = 0xFF09.
*/
typedef struct IOLM_SGI_SEventSubscription
{
    INT16U u16ArgBlockID;   /**< \brief Big endian. */
    INT8U u8Subscribe;      /**< \subscribe command: 1 - subscribe, 0 unsubscribe*/
}IOLM_SGI_SEventSubscription;

/**
\brief This structure is used for SGI manager control

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_TemporaryPDOutAccess = 0xFF0C.
*/
typedef struct IOLM_SGI_SSetTempPDOutAccess
{
    INT16U u16ArgBlockID;   /**< \brief Big endian. */
    INT8U u8Subscribe;      /**< \brief subscribe for exclusive rights: 1 - subscribe, 0 unsubscribe*/
    INT8U u8AutoReleaseTimeout; /**< \brief auto release if no PDOut requests called within given timeout. [1 - 10] Seconds */
}IOLM_SGI_SSetTempPDOutAccess;

/**
\brief This structure is used for SGI manager control

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_ExclusiveAcyclicAccess = 0xFF0D.
*/
typedef struct IOLM_SGI_SSetExclAcyclicAccess
{
    INT16U u16ArgBlockID;   /**< \brief Big endian. */
    INT8U u8Subscribe;      /**< \bried subscribe for exclusive rights: 1 - subscribe, 0 unsubscribe*/
    INT8U u8TargetPort;     /**< \brief a port to ask for subscription. 0 - check all ports*/
    INT32U u32TimeOut;      /**< \brief timeout in seconds the subscription will be released after [0 - 600]*/
    INT8U u8AutoReleaseTimeout; /**< \brief autorelease if no acyclic requests/responces within automatic timeout were registred. [1 - 10] Seconds*/
}IOLM_SGI_SSetExclAcyclicAccess;

/**
\brief This structure is used for SGI manager control

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_ForcePDIn = 0xFF0E.
*/
typedef struct IOLM_SGI_SSetForcePDIn
{
    INT16U u16ArgBlockID;   /**< \brief Big endian. */
    INT8U u8Subscribe;      /**< \bried subscribe for exclusive rights: 1 - subscribe, 0 unsubscribe*/
    INT8U u8TargetPort;     /**< \brief a port to ask for subscription. 0 - check all ports*/
    INT8U u8AutoReleaseTimeout; /**< \brief autorelease if no PDOut requests called within given timeout. [1 - 10] Seconds */
}IOLM_SGI_SSetForcePDIn;

/**
\brief This structure is used to set global client rights in SGI manager control

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_SetGWMAccessRights = 0xFF10.
*/
typedef struct IOLM_SGI_SGWMAccessRights
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8TargetClientID; /**< \brief ID of the client*/
    INT8U eAction; /**< \brief Action : Set, Reset or Update */
    INT32U u32Rights; /**< \brief The flags with the access rights*/
}IOLM_SGI_SGWMAccessRights;

/**
\brief This structure is used to request global client rights in SGI manager control

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_GetGWMAccessRightsReq = 0xFF11.
*/
typedef struct IOLM_SGI_SGetGWMAccessRightsReq
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8TargetClientID; /**< \brief ID of the client*/
}IOLM_SGI_SGetGWMAccessRightsReq;

#endif

///////////////////////////////

/**
\brief This structure is used for the SMI_Header.

The SMI_Header is sent before every data exchange to prepare the recipient for the following data.
On serial transmission after the Header follows an ArgBlock with u16ArgBlockLength number of bytes.
*/
typedef struct IOLM_SMI_SHeader
{
    INT8U u8Service; /**< \brief IOLM_SMI_EServiceID. */
    INT8U u8Reserved; /**< \brief Reserved. */
    INT16U u16ExpRefArgBlockId; /**< \brief Expected response or referenced ArgBlock Id in BigEndian */

    INT8U u8Instance; /**< \brief Addressed instance (port, track, etc.). */
    INT8U u8ClientId; /**< \brief Client id. #IOLM_SMI_CLIENTID_INTERN and #IOLM_SMI_CLIENTID_BROADCAST are reserved. */
    INT16U u16ArgBlockLength;  /**< \brief Little endian. */
}IOLM_SMI_SHeader;

/**
\brief This structure follows the #IOLM_SMI_SHeader struct.

pu8ArgBlock points to the data. u16ArgBlockLength is used to tell the recipient the used length of the ArgBlock.
u16ArgBlockLengthMax defines the maximum length of the ArgBlock and can be bigger than u16ArgBlockLength, if a bigger answer is expected.
*/
typedef struct IOLM_SMI_SArgBlockParam
{
    INT8U *pu8ArgBlock; /**< \brief Data buffer (can be NULL if u16ArgBlockLengthMax = 0). */
    INT16U u16ArgBlockLength; /**< \brief Used length of pu8ArgBlock. */
    INT16U u16ArgBlockLengthMax; /**< \brief Total memory Length of pu8ArgBlock. */
    INT16U u16ExpRefArgBlockId; /**< \brief Expected Response or referenced ArgBlock Id (0 = not used). */
}IOLM_SMI_SArgBlockParam;

////////////////////////////////////////////////

#define IOLM_SMI_ENDIAN_32(x) ( (((x) >> 24) & 0x000000FF) | (((x) >> 8) & 0x0000FF00) | \
(((x) << 24) & 0xFF000000) | (((x) << 8) & 0x00FF0000))
#define IOLM_SMI_ENDIAN_16(x) ( (((x) >> 8) & 0x00FF) | (((x) << 8) & 0xFF00) )

////////////////////////////////////////////////

#pragma pack(pop)

/** 
\}
*/

#ifdef __cplusplus
}
#endif

#endif

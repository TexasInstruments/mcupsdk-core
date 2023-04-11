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

#ifndef INC_PROT__IOLM_SMI_H__
#define INC_PROT__IOLM_SMI_H__

#include "IOLM_SMI_Types.h"
#include "IOLM_SMI_ExtTypes.h"

#ifdef __cplusplus
extern "C" {
#endif


#define IOLM_SMI_TIMEOUT_MS     10000
#define IOLM_SMI_CFG_INSTANCE   0xFF
#define IOLM_SMI_NVCFG_CRC_SEED 0x12345678

#ifndef IOLM_SMI_CLIENT_COUNT
#define IOLM_SMI_CLIENT_COUNT 4 // default number of clients
#endif 


/** 
\page page_smi SMI

The Standardized Master Interface (SMI) is a generic interface which is
set on top of the regular API of the stack.
It matches the interface with other IO-Link standards and acts as a connecting interface
for multiple clients. It also includes the configuration management and event dispatching.

\section sect_smi_arch Architecture

Most services have ArgBlocks as their parameters. These ArgBlocks are made up of an identifier
and contain with different content, depending on its type. The supported ArgBlocks are listed in a following
chapter (see \ref IOLM_SMI_EArgBlockID). If no ArgBlock is used "NULL" is passed instead of the ArgBlock parameter,
e.g. for a request. Most SMI services are acknowledged by a separate confirmation.
They are executed in the Mainloop.
Some simple services are also available with direct confirmation.
The normal API should not be used if the SMI is enabled.

\section sect_smi_init Initialization and Run

The following code snippet shows how to initialize the SMI interface.

\code{.c}
// Setup the desired callbacks
IOLM_SMI_SCallbacks suSMICallbacks_g =
{
    .cbGenericCnf = IOLM_SMI_vGenericCnf,
    .cbMemFree = IOLM_SMI_vMemFree,
    .cbMemAlloc = LOLM_SMI_pu8ArgBlockAlloc,
    .cbLoadNvCfg = IOLM_SMI_LoadNVCfg,

    .cbDeviceEventInd = IOLM_SMI_DeviceEventInd,
    .cbPortEventInd = IOLM_SMI_PortEventInd,
    .cbDeviceWriteCnf = IOLM_SMI_DeviceWriteCnf,
    .cbDeviceReadCnf = IOLM_SMI_DeviceReadCnf,
};

// Initialize stack and SMI (suSMICallbacks_g has to be global)
IOLM_SMI_vInit(&suSMICallbacks_g);

// Application Mainloop
while (1)
{
    IOLM_SMI_vRun();    // SMI Mainloop (at least once per ms)
    User_Application(); // ToDo: Insert application specific code here
}
\endcode

It is only required to provide the desired callbacks. There is
a generic API for all services, or an API for each single service.


\section sect_smi_services Services

- #IOLM_SMI_vGenericReq
- #IOLM_SMI_vMasterIdentificationReq
@if (!IOLM_WIRELESS && IOL_SAFETY)
- #IOLM_SMI_vFSMasterAccessReq
@endif
- #IOLM_SMI_vMasterConfigurationReq
- #IOLM_SMI_vPortConfigurationReq
@if (IOLM_WIRELESS)
- #IOLM_SMI_vTrackConfigurationReq
- #IOLM_SMI_vReadbackTrackConfigurationReq
- #IOLM_SMI_vTrackStatusReq
- #IOLM_SMI_vScanReq
- #IOLM_SMI_vPortPairingReq
@endif
- #IOLM_SMI_vPortStatusReq
- #IOLM_SMI_vReadbackPortConfigurationReq
- #IOLM_SMI_vDSBackupToParServReq
- #IOLM_SMI_vDSBackupFromParServReq
- #IOLM_SMI_vDeviceWriteReq
- #IOLM_SMI_vDeviceReadReq
- #IOLM_SMI_vPDInReq
- #IOLM_SMI_vPDOutReq
- #IOLM_SMI_vPDInOutReq
@if (!IOLM_WIRELESS)
@if IOL_SAFETY
- #IOLM_SMI_vSPDUInReq
- #IOLM_SMI_vSPDUOutReq
@endif
- #IOLM_SMI_vPDInIQReq
- #IOLM_SMI_vPDOutIQReq
@endif
- #IOLM_SMI_vSaveNvFinished

\section sect_smi_callbacks Callbacks

- #IOLM_SMI_CBGenericCnf
- #IOLM_SMI_CBMasterIdentificationCnf
- #IOLM_SMI_CBLoadMasterIdentification
@if (!IOLM_WIRELESS && IOL_SAFETY)
- #IOLM_SMI_CBFSMasterAccessCnf
@endif
- #IOLM_SMI_CBMasterConfigurationCnf
- #IOLM_SMI_CBPortConfigurationCnf
@if (IOLM_WIRELESS)
- #IOLM_SMI_CBTrackConfigurationCnf
- #IOLM_SMI_CBReadbackTrackConfigurationCnf
- #IOLM_SMI_CBTrackStatusCnf
- #IOLM_SMI_CBPortPairingCnf
@endif
- #IOLM_SMI_CBReadbackPortConfigurationCnf
- #IOLM_SMI_CBPortStatusCnf
- #IOLM_SMI_CBDSBackupToParServCnf
- #IOLM_SMI_CBDSBackupFromParServCnf
- #IOLM_SMI_CBDeviceWriteCnf
- #IOLM_SMI_CBDeviceReadCnf
- #IOLM_SMI_CBDeviceEventInd
- #IOLM_SMI_CBPortEventInd
- #IOLM_SMI_CBPDInCnf
- #IOLM_SMI_CBPDOutCnf
- #IOLM_SMI_CBPDInOutCnf
@if (!IOLM_WIRELESS)
@if IOL_SAFETY
- #IOLM_SMI_CBSPDUInCnf
- #IOLM_SMI_CBSPDUOutCnf
@endif
- #IOLM_SMI_CBPDInIQCnf
- #IOLM_SMI_CBPDOutIQCnf
@endif
- #IOLM_SMI_CBLoadNVCfg
- #IOLM_SMI_CBSaveNVCfg

\section sect_smi_argblock ArgBlocks

The ArgBlock IDs are listed in #IOLM_SMI_EArgBlockID. The following
structures are available

- #IOLM_SMI_SMasterident
@if (IOLM_WIRELESS)
- #IOLM_SMI_SWMasterConfigList
- #IOLM_SMI_SWPortConfigList
- #IOLM_SMI_SWPortStatusList
- #IOLM_SMI_SWTrackStatusList
- #IOLM_SMI_SWTrackConfigList
- #IOLM_SMI_SScanResult
@else
- #IOLM_SMI_SFSMasterAccess
- #IOLM_SMI_SPortConfigList
- #IOLM_SMI_SFSPortConfigList
- #IOLM_SMI_SFSPortStatusList
- #IOLM_SMI_SPortStatusList
@endif
- #IOLM_SMI_SPDIn
- #IOLM_SMI_SPDOut
- #IOLM_SMI_SPDInOut
@if (!IOLM_WIRELESS)
- #IOLM_SMI_SSPDUIn
- #IOLM_SMI_SSPDUOut
@endif
- #IOLM_SMI_SDeviceEvent
- #IOLM_SMI_SPortEvent
- #IOLM_SMI_SOnRequestData

\section sect_smi_directservice Direct Services

@if (IOLM_WIRELESS)
- #IOLM_SMI_u16TrackStatusReqCnf
- #IOLM_SMI_u16PortStatusReqCnf
@else
- #IOLM_SMI_u16PortPowerOffOnReqCnf
@endif
- #IOLM_SMI_u16PDOutReqCnf
- #IOLM_SMI_u16PDInReqCnf
- #IOLM_SMI_u16PDInOutReqCnf
@if (!IOLM_WIRELESS && IOL_SAFETY)
- #IOLM_SMI_u16SPDUInReqCnf
- #IOLM_SMI_u16SPDUOutReqCnf
@endif

*/


#ifndef IOLM_SMI_TICKINTERVAL_MS   
#define IOLM_SMI_TICKINTERVAL_MS            (100)
#endif

#ifndef IOLM_SMI_SAVE_TICKINTERVAL_MS   
#define IOLM_SMI_SAVE_TICKINTERVAL_MS     (10000)
#endif

#define IOLM_SMI_PD_OUTPUT_LENGTH            (32)
#define IOLM_SMI_SERIAL_NUMBER_MAX           (16)

#ifdef IOLM_WIRELESS
typedef IOL_ENUM_DECL IOLM_SMI_WMasterAutoPairingState
{
    SMI_WMasterAutoPairingState_Idle = 0,
    SMI_WMasterAutoPairingState_Requested,
    SMI_WMasterAutoPairingState_Running,
} IOLM_SMI_WMasterAutoPairingState;

typedef IOL_ENUM_DECL IOLM_SMI_WMasterConfigState
{
    SMI_WMasterConfigState_Idle= 0,
    SMI_WMasterConfigState_SettingTrackConfig,
    SMI_WMasterConfigState_SettingTrackMode,
    SMI_WMasterConfigState_StoppingTracks,
    SMI_WMasterConfigState_Configured,
} IOLM_SMI_WMasterConfigState;

typedef struct IOLM_SMI_WMasterConfigReq
{
    IOLM_SMI_SWMasterConfigList requestedConfig;
    IOLM_SMasterConfigList configListSM;
    bool newConfigRequired;
    bool tracksToBeChanged[IOLM_NUM_OF_TRACKS];
    bool tracksToBeStopped[IOLM_NUM_OF_TRACKS];
    IOLM_SMI_WMasterConfigState stateBackup;
}IOLM_SMI_WMasterConfigReq;
#endif

typedef struct IOLM_SMI_SJob
{
    IOLM_SMI_SHeader suHeader; ///< Header struct.
    TBOOL boGeneric;    ///< TRUE if response should be by generic API.
    INT8U u8Arg;        ///< Optional internal argument.
    INT16U u16ArgBlockReq;   ///< Request argument.
    INT32S s32TimeStart;///< SysTick start value for timeout.
    INT8U *pu8Data;     ///< Pointer to payload if available.

    volatile struct IOLM_SMI_SJob *psuNext; ///< Pointer to next job in queue.
}IOLM_SMI_SJob;

typedef struct IOLM_SMI_SJobList
{
    volatile IOLM_SMI_SJob *psuFirst;
    volatile IOLM_SMI_SJob *psuLast;
    volatile IOLM_SMI_SJob *psuPending; ///< Job that could not be immediately finished.
}IOLM_SMI_SJobList;

typedef struct IOLM_SMI_SPortInstance
{
    // Port Status
    INT8U u8PortStatusInfo;
    INT8U u8PortQualityInfo;
#ifdef IOLM_WIRELESS
    INT8U u8LinkQualityM;
    INT8S s8RssiM;
    INT8U u8LinkQualityD;
    INT8S s8RssiD;
#endif
    // Diagnosis Unit Events 
    INT8U u8DiagWritePos;
    TBOOL boDiagFull;
    INT8U au8DiagEvents[3 * IOLM_SMI_MAX_DIAG_ENTRIES];
    

    IOLM_SMI_SJobList suFastQueue; ///< For attributes without delayed response.
    IOLM_SMI_SJobList suSlowQueue; ///< For ISDU read/write with delayed response.

    // ISDU
    INT16U u16ISDUIndex;
    INT8U u8ISDUSubIndex;

#ifndef IOLM_WIRELESS
    // IQ Mode
    IOL_EIQMode eIQMode;
#endif

    // Process Data
    INT8U u8PortQualifier;
    INT8U u8PortOE;
    INT8U u8LastPDInReqClient;
    IOLM_SMI_SPDIn suLastPDInBuffer;
#ifdef IOL_SAFETY
    INT8U u8PDInOffset; ///< Offset of non safe input Process Data.
#endif
    INT8U u8PDOutOffset; ///< Offset of non safe output Process Data.
    INT8U u8OutputDataLength; ///< Output data length
    INT8U au8PDOutCache[IOLM_SMI_PD_OUTPUT_LENGTH]; ///< Cache Process Data

    // Kunbus regression test attributes
    TBOOL boRegTestEnabled;
    IOLM_EDSActivationState eDSActState;
    IOLM_EDSMode eDSMode;
    INT8U u8StackPortStatusInfo;
    INT8U au8SerialNumber[IOLM_SMI_SERIAL_NUMBER_MAX];
    TBOOL boSerialNumberSet;
}IOLM_SMI_SPortInstance;

typedef struct IOLM_SMI_SNVConfiguration
{
    INT32U u32NVCrc;
#ifdef IOLM_WIRELESS
    IOLM_SMI_SWPortConfigList asuPortConfig[IOLM_PORT_COUNT];
    IOLM_SMI_SWMasterConfigList suMasterConfig;
#else
#if IOL_SAFETY
    IOLM_SMI_SFSPortConfigList asuPortConfig[IOLM_PORT_COUNT];
#else
    IOLM_SMI_SPortConfigList asuPortConfig[IOLM_PORT_COUNT];
#endif
    IOLM_SMI_SGeneralArgBlock suMasterConfig;
#endif
    IOLM_SMI_SStackInformation suStackVersion;

} IOLM_SMI_SNVConfiguration;

typedef struct IOLM_SMI_SInstance
{
    INT8U *pu8MasterIdentification;
    INT16U u16MasterIdentLength;

    IOLM_SMI_SJob asuJobs[IOLM_PORT_COUNT * 4]; // \todo: number of clients?
    IOLM_SMI_SJobList suEmptyQueue;
    IOLM_SMI_SJobList suGeneralQueue;

    IOLM_SMI_SPortInstance asuPort[IOLM_PORT_COUNT];

    INT32U u32NvAlignmentDummy;
    IOLM_SMI_SNVConfiguration suNvConfiguration;
#ifdef IOLM_WIRELESS
#if IOLM_SMI_SUPPORT_OLD_SERVICES == 1
    IOLM_SMI_SWMasterConfigListOld suMasterConfigOld;
    IOLM_SMI_SWTrackConfigList asuTrackConfigOld[IOLM_NUM_OF_TRACKS];
#endif
#endif
    TBOOL boNvConfigChanged;
    TBOOL boNVSaveInProgress;
    INT32S s32TickValNvSave;                ///< Tick to limit frequency of save to EEPROM operations.
    INT32S s32TickValMaintenance;          ///< Tick for maintenance actions.
    INT8U u8MaintenancePort;

#ifdef IOLM_SMI_MONITOR
    INT8U u8LogSettings; // bit 0 = enable/disable
#endif

    INT8U u8DSPort;
    INT8U *pu8DSData; ///< If not NULL there is a waiting storage.
    INT16U u16DSLength;
    TBOOL boDSInProgress;
#ifdef IOLM_WIRELESS
    IOLM_SMI_WMasterConfigState wMasterConfigState;
    IOLM_SMI_WMasterConfigReq wMasterRequestedConfig;
    IOLM_SMI_WMasterAutoPairingState wMasterAutoPairingState;
    TBOOL aboAutopairingPort[IOLM_PORT_COUNT];
#endif
}IOLM_SMI_SInstance;

extern IOLM_SMI_SInstance IOLM_SMI_suInst_g;

////////////////////////////////////////////////

/**
\brief SMI generic request.

The function is used for a generic access of all services of the SMI.
The answer to this request is the callback function #IOLM_SMI_CBGenericCnf.

\param[in]  psuHeader_p     Header.
\param[in]  pu8ArgBlock_p   Generic ArgBlock.

\par Example

\code{.c}
// Allocate header memory
INT8U au8HeaderMem[sizeof(IOLM_SMI_SHeader)];
IOLM_SMI_SHeader *psuHeader = (IOLM_SMI_SHeader *)au8HeaderMem;

// Allocate ArgBlock memory (pu8Data contains the data of length u16DataLength)
INT16U u16ArgBlockLength = sizeof(IOLM_SMI_SGeneralArgBlock) + u16DataLength;
INT8U au8ArgBlockMem[u16ArgBlockLength];
IOLM_SMI_SGeneralArgBlock *psuArgblock = (IOLM_SMI_SGeneralArgBlock *)au8ArgBlockMem;

// Fill header
psuHeader->u16ArgBlockLength = u16ArgBlockLength;
psuHeader->u8Service = ...
...

// Fill request
psuArgblock->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_SGeneralArgBlock); // ArgBlock ID (little endian)
...

// Copy user data into ArgBlock request
memcpy(psuArgblock + sizeof(IOLM_SMI_SGeneralArgBlock), pu8Data, u16ArgBlockLength);

// ToDo: Insert application specific code here

// Send request
IOLM_SMI_vGenericReq(psuHeader, (INT8U *)psuArgblock);

// wait for IOLM_SMI_CBGenericCnf
\endcode

\ingroup grp_smi_general

*/
IOL_FUNC_DECL void IOLM_SMI_vGenericReq(IOLM_SMI_SHeader *psuHeader_p, INT8U *pu8ArgBlock_p);

/**
\fn IOLM_SMI_CBGenericCnf
\brief SMI Generic request confirmation callback.

This callback function is called by the stack for a generic access of all services of the SMI.
Confirmation to the #IOLM_SMI_vGenericReq request.

\param[in]  psuHeader_p     Header.
\param[in]  pu8ArgBlock_p   Generic ArgBlock.

\par Example

\code{.c}
void IOLM_SMI_vGenericCnf(IOLM_SMI_SHeader *psuHeader_p, INT8U *pu8ArgBlock_p)
{
    IOLM_SMI_SGeneralArgBlock *psuGenericArgBlock;
    INT8U u16ArgBlockLength = psuHeader_p->u16ArgBlockLength;
    psuGenericArgBlock = malloc(u16ArgBlockLength);
    ...

    // Get header parameters
    psuHeader_p->u8ClientId;
    ...

    // Get ArgBlock data
    memcpy(psuGenericArgBlock, pu8ArgBlock_p, u16ArgBlockLength);

    // ToDo: Insert application specific code here
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_general

*/
typedef void (*IOLM_SMI_CBGenericCnf)(IOLM_SMI_SHeader *psuHeader_p, INT8U *pu8ArgBlock_p);

#ifdef IOLM_OS_AVAILABLE
/**
\fn IOLM_SMI_CBMainLoopRequest
\brief Trigger a mainloop request

This callback function is called by the stack requires a mainloop run.
This is used in combination with operating systems

\par Example

\code{.c}
void IOLM_SMI_vMainLoopRequest(void)
{

    // ToDo: Wakeup main thread
}
\endcode

\ingroup grp_smi_general

*/
typedef void (*IOLM_SMI_CBMainLoopRequest)(void);
#endif

/**
\brief Get Master identification.

Request of the Master identification.
The answer to the request is the function #IOLM_SMI_CBMasterIdentificationCnf.

\param[in]  u8ClientID_p   Client ID.

\ingroup grp_smi_config

*/
IOL_FUNC_DECL void IOLM_SMI_vMasterIdentificationReq(INT8U u8ClientID_p);

/**
\fn IOLM_SMI_CBMasterIdentificationCnf
\brief Get Master identification confirmation callback.

Confirmation callback to a request of the Master identification from the stack
to the #IOLM_SMI_vMasterIdentificationReq request.
Since this is hardware specific, it has to be implemented in the application code.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.
\param[in]  u16ArgBlockLength_p     Length of the ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the master identification (#IOLM_SMI_SMasterident).

\par Example
\code{.c}
void IOLM_SMI_vMasterIdentificationCnf(INT8U u8ClientID_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p)
{
    IOLM_SMI_SMasterident *psuMasterIdent = (IOLM_SMI_SMasterident *)pu8ArgBlock_p;

    psuMasterIdent->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_MasterIdent);
    ...

    // ToDo: Insert application specific code here
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBMasterIdentificationCnf)(INT8U u8ClientID_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

/**
\fn IOLM_SMI_CBLoadMasterIdentification
\brief Load Master configuration callback 

This callback service is called by the stack and requests the Master configuration from the application.
Since this is hardware specific, it has to be implemented in the application code.

\param[in]  u16ArgBlockLength_p     Length of the ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the Master identification (#IOLM_SMI_SMasterident).

\par Example
\code{.c}
void IOLM_SMI_vLoadMasterIdentification(INT16U *u16ArgBlockLength_p, INT8U *pu8ArgBlock_p)
{
    IOLM_SMI_SMasterident *psuMasterIdent = (IOLM_SMI_SMasterident *)pu8ArgBlock_p;

    psuMasterIdent->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_MasterIdent);
    ...

    // ToDo: Set up psuMasterIdent parameters
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_general

*/
typedef void(*IOLM_SMI_CBLoadMasterIdentification)(INT16U *u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

/**
\brief Set Master configuration.

This service allows to set the general configuration of the Master.
The answer to the request is the function #IOLM_SMI_CBMasterConfigurationCnf.


\param[in]  u8ClientID_p            Client ID.
\param[in]  u16ArgBlockLength_p     Length of the ArgBlock.
@if (IOLM_WIRELESS)
\param[in]  pu8ArgBlock_p           Data pointer which points to the Master configuration (#IOLM_SMI_SWMasterConfigList).
@else
\param[in]  pu8ArgBlock_p           Data pointer which points to the Master configuration (#IOLM_SMI_SVoidArgBlock).
@endif

\par Example
@if (IOLM_WIRELESS)
\code{.c}
IOLM_SMI_SWMasterConfigList suMasterConfig;

// ToDo: Set up suMasterConfig parameters

IOLM_SMI_vMasterConfigurationReq(u8ClientID,
    sizeof(suMasterConfig),
    (INT8U *)&suMasterConfig);
\endcode
@else
\code{.c}
IOLM_SMI_SGeneralArgBlock suMasterConfig;

// ToDo: Set up suMasterConfig parameters

IOLM_SMI_vMasterConfigurationReq(u8ClientID,
    sizeof(suMasterConfig),
    (INT8U *)&suMasterConfig);
\endcode
@endif

\ingroup grp_smi_config 
*/
IOL_FUNC_DECL void IOLM_SMI_vMasterConfigurationReq(INT8U u8ClientID_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

/**
\fn IOLM_SMI_CBMasterConfigurationCnf
\brief Get Master configuration confirmation callback.

This callback service is called by the stack and is used to get the general configuration of the Master.
Confirmation to the #IOLM_SMI_vMasterConfigurationReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_config 
*/
typedef void (*IOLM_SMI_CBMasterConfigurationCnf)(INT8U u8ClientID_p, INT16U u16Error_p);

#ifdef IOL_SAFETY
/**
\brief Get FS Master access.

Request the FS Master access.
The answer to the request is the function #IOLM_SMI_CBFSMasterAccessCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u16ArgBlockLength_p     Length of the ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the FS Master access (#IOLM_SMI_SFSMasterAccess).

\ingroup grp_smi_config

*/
IOL_FUNC_DECL void IOLM_SMI_vFSMasterAccessReq(INT8U u8ClientID_p, INT16U u16ArgBlockLength_p, INT8U* pu8ArgBlock_p);

/**
\fn IOLM_SMI_CBFSMasterAccessCnf
\brief Get FS Master access confirmation callback.

Request the FS Master access.
Confirmation to the #IOLM_SMI_vFSMasterAccessReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.
\param[in]  u16ArgBlockLength_p     Length of the ArgBlock.
\param[out] pu8ArgBlock_p           Data pointer which points to the FSCP Authenticity (#IOLM_SMI_SFSCPAuthenticity).

\par Example
\code{.c}
void IOLM_SMI_vFSMasterAccessCnf(INT8U u8ClientID_p, INT16U u16Error_p,
                                 INT16U u16ArgBlockLength_p, INT8U* pu8ArgBlock_p)
{
    IOLM_SMI_SFSMasterAccess *psuFSMasterAccess = (IOLM_SMI_SFSMasterAccess *)pu8ArgBlock_p;
    ...

    // ToDo: Insert application specific code here
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBFSMasterAccessCnf)(INT8U u8ClientID_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U* pu8ArgBlock_p);

/**
\fn IOLM_SMI_CBSetFSSMasterAccessAndGetFSCPAuthenticity
\brief Lets the application set FS MasterAccess Passwort and retrieves the FSCP Authenticity codes

\param[in]  u8ClientID_p            Client ID.
\param[in]  u16ArgBlockLengthReq_p     Length of the ArgBlock.
\param[in]  pu8ArgBlockReq_p           Data pointer which points to the FS Master access (#IOLM_SMI_SFSMasterAccess).
\param[in]  u16ArgBlockLengthResp_p    Length of the ArgBlock.
\param[out] pu8ArgBlockResp_p         Data pointer which points to the FSCP Authenticity (#IOLM_SMI_SFSCPAuthenticity).

\par Example
\code{.c}
void SetFSSMasterAccessAndGetFSCPAuthenticity(INT8U u8ClientID_p, INT16U u16Error_p,
    INT16U u16ArgBlockLengthReq_p, INT8U* pu8ArgBlockReq_p,
    INT16U u16ArgBlockLengthResp_p, INT8U* pu8ArgBlockResp_p)
{
    IOLM_SMI_SFSMasterAccess *psuFSMasterAccess = (IOLM_SMI_SFSMasterAccess *)pu8ArgBlockReq_p;

    psuFSMasterAccess->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_FSMasterAccess);

    IOLM_SMI_SFSCPAuthenticity *psuFSCPAuthenticity = (IOLM_SMI_SFSCPAuthenticity *)pu8ArgBlockResp_p;

    psuFSCPAuthenticity->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_FSCPAuthenticity);
    ...

    // ToDo: Insert application specific code here
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBSetFSMasterAccessAndGetFSCPAuthenticity)(INT8U u8ClientID_p,
    INT16U u16ArgBlockLengthReq_p, INT8U* pu8ArgBlockReq_p,
    INT16U u16ArgBlockLengthResp_p, INT8U* pu8ArgBlockResp_p);

/**
\fn IOLM_SMI_CBFSPortConfiguration
\brief Set FS port configuration callback.

Set FS port configuration.
FS part of the port configuration.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u16ArgBlockLength_p     Length of the ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the FS Port Configuration (#IOLM_SMI_SFSPortConfigurationList).

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBFSPortConfiguration)(INT8U u8ClientID_p, void* psuPortConfig);
#endif

/**
\brief Set port configuration.

With the help of this service, an SMI client such as a gateway application launches
the indicated Master port and the connected Device using the elements in parameter PortConfigList.
The service shall be accepted immediately and performed without delay. Content of Data
Storage for that port will be deleted at each new port configuration via "DS_Delete".
The answer to the request is the function #IOLM_SMI_CBPortConfigurationCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16ArgBlockLength_p     Length of the ArgBlock.
@if (IOLM_WIRELESS)
\param[in]  pu8ArgBlock_p           Data pointer which points to the port configuration (#IOLM_SMI_SWPortConfigList).
@else
\param[in]  pu8ArgBlock_p           Data pointer which points to the port configuration (#IOLM_SMI_SPortConfigList).
@endif

\par Example
@if (IOLM_WIRELESS)
\code{.c}
IOLM_SMI_SWPortConfigList suPortConfig;

// Convert ArgBlock ID to big endian
suPortConfig.u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_WPortConfigList);
...

// ToDo: Set up suPortConfig parameters

IOLM_SMI_vPortConfigurationReq(u8ClientID,
    u8Port,
    sizeof(suPortConfig),
    (INT8U *)&suPortConfig);
\endcode
@else
\code{.c}
IOLM_SMI_SPortConfigList suPortConfig;

// Set up port configuration ArgBlock
suPortConfig.u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_PortConfigList);
suPortConfig.u8PortMode = IOLM_SMI_ePortMode_IOL_AUTOSTART;
suPortConfig.u8ValidationBackup = IOLM_SMI_ePortValBack_NoDeviceCheck;
suPortConfig.u8IQBehavior = IOLM_SMI_ePortIQBehavior_NotSupported;

// If IOLM_SMI_ePortValBack_NoDeviceCheck is set,
// the stack will retrieve the following values from the device
suPortConfig.u8PortCycleTime = 0;
suPortConfig.u16VendorID = 0;
suPortConfig.u32DeviceID = 0;

IOLM_SMI_vPortConfigurationReq(u8ClientID,
    u8Port,
    sizeof(suPortConfig),
    (INT8U *)&suPortConfig);
\endcode
@endif

\ingroup grp_smi_config

*/
IOL_FUNC_DECL void IOLM_SMI_vPortConfigurationReq(INT8U u8ClientID_p, INT8U u8Port_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

/**
\fn IOLM_SMI_CBPortConfigurationCnf
\brief Set port configuration confirmation callback.

With the help of this service, an SMI client such as a gateway application launches
the indicated Master port and the connected Device using the elements in parameter PortConfigList.
The service shall be accepted immediately and performed without delay. Content of Data
Storage for that port will be deleted at each new port configuration via "DS_Delete".
Confirmation to the #IOLM_SMI_vPortConfigurationReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBPortConfigurationCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p);


#ifdef IOLM_WIRELESS
/**
\brief Set track configuration.

This service allows to setup the configuration for a single Track.
The answer to the request is the function #IOLM_SMI_CBTrackConfigurationCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Track_p               Track ID.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the track configuration (#IOLM_STrackparameterList).

\ingroup grp_smi_config

*/
IOL_FUNC_DECL void IOLM_SMI_vTrackConfigurationReq(INT8U u8ClientID_p, INT8U u8Track_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);
#endif

#ifdef IOLM_WIRELESS
/**
\fn IOLM_SMI_CBTrackConfigurationCnf
\brief Set track configuration confirmation callback.

This service allows to setup the configuration for a single track.
Confirmation to the #IOLM_SMI_vTrackConfigurationReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Track_p               Track ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBTrackConfigurationCnf)(INT8U u8ClientID_p, INT8U u8Track_p, INT16U u16Error_p);
#endif

#ifdef IOLM_WIRELESS
/**
\brief Read back track configuration.

This service allows to read back the configuration which was written by the track configuration service.
The answer to the request is the function #IOLM_SMI_CBReadbackTrackConfigurationCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Track_p               Track ID.

\ingroup grp_smi_config

*/
IOL_FUNC_DECL void IOLM_SMI_vReadbackTrackConfigurationReq(INT8U u8ClientID_p, INT8U u8Track_p);
#endif 

#ifdef IOLM_WIRELESS
/**
\fn IOLM_SMI_CBReadbackTrackConfigurationCnf
\brief Read back track configuration confirmation callback.

This service allows to read back the configuration which was written by the track configuration service.
Confirmation to the #IOLM_SMI_vReadbackTrackConfigurationReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Track_p               Track ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the track configuration (#IOLM_STrackparameterList).

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBReadbackTrackConfigurationCnf)(INT8U u8ClientID_p, INT8U u8Track_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);
#endif

#ifdef IOLM_WIRELESS
/**
\brief Get track status.

This service allows to get the current track status.
The answer to the request is the function #IOLM_SMI_CBTrackStatusCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Track_p               Track ID.

\ingroup grp_smi_config

*/
IOL_FUNC_DECL void IOLM_SMI_vTrackStatusReq(INT8U u8ClientID_p, INT8U u8Track_p);
#endif 

#ifdef IOLM_WIRELESS
/**
\fn IOLM_SMI_CBTrackStatusCnf
\brief Get track status confirmation.

This service allows to get the current track status.
Confirmation to the #IOLM_SMI_vTrackStatusReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Track_p               Track ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the track status (#IOLM_SMI_SWTrackStatusList).

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBTrackStatusCnf)(INT8U u8ClientID_p, INT8U u8Track_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);
#endif

#ifdef IOLM_WIRELESS
/**
\brief Start scan.

This service starts a Device scan. For this at least one track need to be in service or roaming mode.
The answer to the request is the function #IOLM_SMI_CBScanCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the track configuration (#IOLM_STrackparameterList).

\ingroup grp_smi_config

*/
IOL_FUNC_DECL void IOLM_SMI_vScanReq(INT8U u8ClientID_p, INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);
#endif 

#ifdef IOLM_WIRELESS
/**
\fn IOLM_SMI_CBScanCnf
\brief Scan confirmation callback.

This is the response of the scan start request
Confirmation to the #IOLM_SMI_vScanReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.

\ingroup grp_smi_config

*/
typedef void(*IOLM_SMI_CBScanCnf)(INT8U u8ClientID_p, INT16U u16Error_p);
#endif

#ifdef IOLM_WIRELESS
/**
\brief Get track status request and confirmation.

This service allows to get the current track status. It is a combination of the request and confirmation.

\param[in]     u8Track_p               Track ID.
\param[inout]  pu16ArgBlockLength_p    Pointer to length of ArgBlock.
\param[inout]  pu8ArgBlock_p           Data pointer which points to the track status (#IOLM_SMI_SWTrackStatusList).

\return INT16U Error as #IOL_EErrorType

\ingroup grp_smi_config

*/
IOL_FUNC_DECL INT16U IOLM_SMI_u16TrackStatusReqCnf(INT8U u8Track_p, INT16U *pu16ArgBlockLength_p, INT8U *pu8ArgBlock_p);
#endif

#ifdef IOLM_WIRELESS
/**
\fn IOLM_SMI_CBTrackScanInd
\brief Scan result indication callback.

This service indicates information about unpaired wireless Devices in scan mode.

\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the scan result (#IOLM_SMI_SScanResult).

\ingroup grp_smi_config

*/
typedef void(*IOLM_SMI_CBTrackScanInd)(INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);
#endif

#ifdef IOLM_WIRELESS
/**
\fn IOLM_SMI_CBTrackScanEndInd
\brief Scan end indication callback.

This service indicates that scanning for unpaired Devices has finished.

\ingroup grp_smi_config

*/
typedef void(*IOLM_SMI_CBTrackScanEndInd)(void);
#endif

/**
\brief Read back port configuration.

This service allows for retrieval of the effective configuration of the indicated Master port.
The answer to the request is the function #IOLM_SMI_CBReadbackPortConfigurationCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.

\ingroup grp_smi_config

*/
IOL_FUNC_DECL void IOLM_SMI_vReadbackPortConfigurationReq(INT8U u8ClientID_p, INT8U u8Port_p);

/**
\fn IOLM_SMI_CBReadbackPortConfigurationCnf
\brief Read back port configuration confirmation callback.

This service allows for retrieval of the effective configuration of the indicated Master port.
Confirmation to the #IOLM_SMI_vReadbackPortConfigurationReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
@if (IOLM_WIRELESS)
\param[in]  pu8ArgBlock_p           Data pointer which points to the port configuration (#IOLM_SMI_SWPortConfigList).
@else
\param[in]  pu8ArgBlock_p           Data pointer which points to the port configuration (#IOLM_SMI_SPortConfigList).
@endif

\par Example
@if (IOLM_WIRELESS)
\code{.c}
void IOLM_SMI_vReadbackPortConfigurationCnf(INT8U u8Port_p, INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p)
{
    IOLM_SMI_SWPortConfigList *psuPortConfig = (IOLM_SMI_SWPortConfigList *)pu8ArgBlock_p;

    // ArgBlockID
    psuPortConfig->u16ArgBlockID;

    // Port mode (See IOLM_SMI_EPortMode)
    psuPortConfig->u8PortMode;
    ...

    // ToDo: Insert application specific code here

}
\endcode
@else
\code{.c}
void IOLM_SMI_vReadbackPortConfigurationCnf(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p, INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p)
{
    IOLM_SMI_SPortConfigList *psuPortConfig = (IOLM_SMI_SPortConfigList *)pu8ArgBlock_p;

    // ArgBlockID
    psuPortConfig->u16ArgBlockID;

    // Port mode (See IOLM_SMI_EPortMode)
    psuPortConfig->u8PortMode;
    ...

    // ToDo: Insert application specific code here

}
\endcode
@endif

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBReadbackPortConfigurationCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

/**
\brief Get port status.

This service allows for retrieval of the effective status of the indicated Master port.
The answer to the request is the function #IOLM_SMI_CBPortStatusCnf.
See also #IOLM_SMI_u16PortStatusReqCnf for a combined request/confirmation service.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.

\ingroup grp_smi_config

*/
IOL_FUNC_DECL void IOLM_SMI_vPortStatusReq(INT8U u8ClientID_p, INT8U u8Port_p);

/**
\fn IOLM_SMI_CBPortStatusCnf
\brief Get port status confirmation callback.

This service allows for retrieval of the effective status of the indicated Master port.
Confirmation to the #IOLM_SMI_vPortStatusReq request.
See also #IOLM_SMI_u16PortStatusReqCnf for a combined request/confirmation service.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the port status (#IOLM_SMI_SPortStatusList).

\par Example

\code{.c}
void IOLM_SMI_vPortStatusCnf(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p)
{
    IOLM_SMI_SPortStatusList *psuPortStatus = (IOLM_SMI_SPortStatusList *)pu8ArgBlock_p;

    // ArgBlockID
    psuPortStatus->u16ArgBlockID;

    // Port mode (See IOLM_SMI_EPortMode)
    psuPortStatus->u8PortStatusInfo; // See #IOLM_SMI_EPortStatus.
    ...

    // ToDo: Insert application specific code here
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBPortStatusCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

/**
\brief Get port status request and confirmation.

This service allows for retrieval of the effective status of the indicated Master port.
It combines the request (#IOLM_SMI_vPortStatusReq) and confirmation
(#IOLM_SMI_CBPortStatusCnf) service.

\param[in]      u8Port_p                Port ID.
\param[inout]   pu16ArgBlockLength_p    Pointer to length of ArgBlock.
\param[inout]   pu8ArgBlock_p           Data pointer which points to the port status (#IOLM_SMI_SPortStatusList).

\return Error as #IOL_EErrorType

\par Example

\code{.c}
IOLM_SMI_SPortStatusList suPortStatus;
INT16U u16Length = sizeof(IOLM_SMI_SPortStatusList);
INT16U u16Error;

// Get Port Status
u16Error = IOLM_SMI_u16PortStatusReqCnf(u8Port, &u16Length, (INT8U *)&suPortStatus);
if (u16Error == IOL_eErrorType_NONE)
{
    // ToDo: Insert application specific code here
}
\endcode

\ingroup grp_smi_config

*/
IOL_FUNC_DECL INT16U IOLM_SMI_u16PortStatusReqCnf(INT8U u8Port_p, INT16U *pu16ArgBlockLength_p, INT8U *pu8ArgBlock_p);


/**
\brief Backup to parameter server.

With the help of this service, an SMI client such as a gateway application is able to retrieve
the technology parameter set of a Device from Data Storage and back it up within an upper
level parameter server.
The answer to the request is the function #IOLM_SMI_CBDSBackupToParServCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.

\ingroup grp_smi_config

*/
IOL_FUNC_DECL void IOLM_SMI_vDSBackupToParServReq(INT8U u8ClientID_p, INT8U u8Port_p);

/**
\fn IOLM_SMI_CBDSBackupToParServCnf
\brief Backup to parameter server confirmation callback.

With the help of this service, an SMI client such as a gateway application is able to retrieve
the technology parameter set of a Device from Data Storage and back it up within an upper
level parameter server.
Confirmation to the #IOLM_SMI_vDSBackupToParServReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the backup to parameter server.

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBDSBackupToParServCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);


/**
\brief Backup from parameter server.

With the help of this service, an SMI client such as a gateway application is able to restore
the technology parameter set of a Device within Data Storage from an upper level parameter server.
The answer to the request is the function #IOLM_SMI_CBDSBackupFromParServCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the backup to parameter server.

\ingroup grp_smi_config

*/
IOL_FUNC_DECL void IOLM_SMI_vDSBackupFromParServReq(INT8U u8ClientID_p, INT8U u8Port_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

/**
\fn IOLM_SMI_CBDSBackupFromParServCnf
\brief Backup from parameter server confirmation callback.

With the help of this service, an SMI client such as a gateway application is able to restore
the technology parameter set of a Device within Data Storage from an upper level parameter server.
Confirmation to the #IOLM_SMI_vDSBackupFromParServReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBDSBackupFromParServCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p);


/**
\brief Set Device On-request Data.

This service allows for writing On-request Data (OD) for propagation to the Device.
The answer to the request is the function #IOLM_SMI_CBDeviceWriteCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the Device's On-request Data (#IOLM_SMI_SOnRequestData).

\par Example

\code{.c}
INT8U au8Mem[IOLM_SMI_ARGBLOCK_ONREQ_LEN(2)]; // Allocate ArgBlock memory
IOLM_SMI_SOnRequestData *psuReq = (IOLM_SMI_SOnRequestData *)au8Mem;

// Fill request
psuReq->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_OnRequestDataWrite);
psuReq->u16Index = IOLM_SMI_ENDIAN_16(123); // Index
psuReq->u8Subindex = 0;                     // SubIndex
psuReq->au8Data[0] = 0x12;                  // Data
psuReq->au8Data[1] = 0x34;                  // Data

// Send request
IOLM_SMI_vDeviceWriteReq(
    0,                              // Client ID
    0,                              // Port
    IOLM_SMI_ARGBLOCK_ONREQ_LEN(2), // Length
    au8Mem
);

// Wait for IOLM_SMI_CBDeviceWriteCnf

\endcode

\ingroup grp_smi_onreq

*/
IOL_FUNC_DECL void IOLM_SMI_vDeviceWriteReq(INT8U u8ClientID_p, INT8U u8Port_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

/**
\fn IOLM_SMI_CBDeviceWriteCnf
\brief Set Device On-request Data confirmation callback.

This callback indicates an answer to an On-request Data (OD) write request from a Device.
Confirmation to the #IOLM_SMI_vDeviceWriteReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.

\par Example

\code{.c}
void IOLM_SMI_vDeviceWriteCnf(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p)
{
    if (u16Error_p != IOL_eErrorType_NONE)
    {
        UART_printf("IO-Link port %u: Write failed with error 0x%04x\n", u8Port_p, u16Error_p);
    }
    else
    {
        // ToDo: Insert application specific code here
    }
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_onreq

*/
typedef void (*IOLM_SMI_CBDeviceWriteCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p);


/**
\brief Get Device On-request Data.

This service allows for reading On-request Data (OD) from the Device via the Master.
The answer to the request is the function #IOLM_SMI_CBDeviceReadCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the Device On-request Data (#IOLM_SMI_SOnRequestData).

\par Example

\code{.c}
INT8U au8Mem[IOLM_SMI_ARGBLOCK_ONREQ_LEN(0)]; // Allocate ArgBlock memory
IOLM_SMI_SOnRequestData *psuReq = (IOLM_SMI_SOnRequestData *)au8Mem;

// Fill request
psuReq->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_OnRequestDataRead);
psuReq->u16Index = IOLM_SMI_ENDIAN_16(16); // Index 16 = VendorName
psuReq->u8Subindex = 0;

// Send request
IOLM_SMI_vDeviceReadReq(
    0,                              // Client ID
    0,                              // Port
    IOLM_SMI_ARGBLOCK_ONREQ_LEN(0), // Length
    au8Mem
);

// Wait for IOLM_SMI_CBDeviceReadCnf

\endcode

\ingroup grp_smi_onreq

*/
IOL_FUNC_DECL void IOLM_SMI_vDeviceReadReq(INT8U u8ClientID_p, INT8U u8Port_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

/**
\fn IOLM_SMI_CBDeviceReadCnf
\brief Get Device On-request Data confirmation callback.

This service allows for reading On-request Data (OD) from the Device via the Master.
Confirmation to the #IOLM_SMI_vDeviceReadReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the Device On-request Data (#IOLM_SMI_SOnRequestData).

\par Example

\code{.c}
void IOLM_SMI_vDeviceReadCnf(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
                            INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p)
{
    if (u16Error_p != IOL_eErrorType_NONE)
    {
        UART_printf("IO-Link port %u: Read failed with error 0x%04x\n", u8Port_p, u16Error_p);
    }
    else
    {
        // ToDo: Insert application specific code here
    }
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_onreq

*/
typedef void (*IOLM_SMI_CBDeviceReadCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p, 
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

#ifdef IOLM_WIRELESS
/**
\brief Wireless port pairing.

This service allows for performing certain methods (functions) at a port that are defined by the argument CMD.
A first method (CMD = 0) is supporting the transfer of a large number of consistent Device parameters via multiple ISDUs.
A second method (CMD = 1) allows for switching power of a particular port off and on.

Both methods are optional. Availability is indicated via Master identification.
The answer to the request is the function #IOLM_SMI_CBPortPairingCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer to ArgBlock.

\par Example
\code{.c}
IOLM_SMI_SWPairingList suPairing;

suPairing.u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_WPairing);
suPairing.u8PairingCommand = IOLM_SMI_ePairCmd_PairingButton;

IOLM_SMI_vPortPairingReq(IOLM_SMI_CLIENT_APP,   // Client
u8TestPort,                                     // Port
sizeof(IOLM_SMI_SWPairingList),
(INT8U *)&suPairing);                           //  ArgBlock
); 

// IOLM_SMI_CBPortPairingCnf is called if command has finished
\endcode

\ingroup grp_smi_config 

*/
IOL_FUNC_DECL void IOLM_SMI_vPortPairingReq(INT8U u8ClientID_p, INT8U u8Port_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

/**
\fn IOLM_SMI_CBPortPairingCnf
\brief Wireless port command confirmation callback.

This service allows for performing certain methods (functions) at a port that are defined by the argument CMD.
A first method (CMD = 0) is supporting the transfer of a large number of consistent Device parameters via multiple ISDUs.
A second method (CMD = 1) allows for switching power of a particular port off and on.

Both methods are optional. Availability is indicated via Master identification.
Confirmation to the #IOLM_SMI_vPortPairingReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.

\ingroup grp_smi_config

*/
typedef void (*IOLM_SMI_CBPortPairingCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p);
#endif

/**
\fn IOLM_SMI_CBDeviceEventInd
\brief Device Event indicator callback.

This service allows for signaling an event generated by the Device.

\param[in]  u8Port_p                Port ID.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the Device Event data (#IOLM_SMI_SDeviceEvent).

\par Example
\code{.c}
void IOLM_SMI_DeviceEventInd(INT8U u8Port_p, INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p)
{
    IOLM_SMI_SDeviceEvent *psuEvent = (IOLM_SMI_SDeviceEvent *)pu8ArgBlock_p;

    // Event Code
    psuEvent->u16EventCode;

    // Event Qualifier (according specification Annex A.6.4)
    psuEvent->u8EventQualifier;

    // Example Output
    UART_printf("IO-Link port %u: Device event - qualifier: 0x%02x - code: 0x%04x\n",
                u8Port_p,
                psuEvent->u8EventQualifier,
                psuEvent->u16EventCode);

    // ToDo: Insert application specific code here
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_event

*/
typedef void (*IOLM_SMI_CBDeviceEventInd)(INT8U u8Port_p, INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);


/**
\fn IOLM_SMI_CBPortEventInd
\brief Port event indicator callback.

This service allows for signaling an event generated by the Master port.

\param[in]  u8Port_p                Port ID.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the port event data (#IOLM_SMI_SPortEvent).

\par Example
\code{.c}
void IOLM_SMI_PortEventInd(INT8U u8Port_p, INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p)
{
    IOLM_SMI_SPortEvent *psuEvent = (IOLM_SMI_SPortEvent *)pu8ArgBlock_p;

    // Event Code
    psuEvent->u16EventCode;

    // Event Qualifier (according specification Annex A.6.4)
    psuEvent->u8EventQualifier;

    // Example Output
    UART_printf("IO-Link port %u: Port event - qualifier: 0x%02x - code: 0x%04x\n",
                u8Port_p,
                psuEvent->u8EventQualifier,
                psuEvent->u16EventCode);

    // ToDo: Insert application specific code here
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_event

*/
typedef void (*IOLM_SMI_CBPortEventInd)(INT8U u8Port_p, INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);


/**
\brief Get input data.

This service allows for cyclically reading input Process Data from an input buffer.
The answer to the request is one of these functions:
#IOLM_SMI_CBPDInCnf

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL void IOLM_SMI_vPDInReq(INT8U u8ClientID_p, INT8U u8Port_p);

/**
\fn IOLM_SMI_CBPDInCnf
\brief Get input data confirmation callback.

This service allows for cyclically reading input process data from an input buffer.
Confirmation to the #IOLM_SMI_vPDInReq request if a single port was read.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error Message as #IOL_EErrorType.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data Pointer which points to the PDIn Data (#IOLM_SMI_SPDIn).

\par Example

\code{.c}
void IOLM_SMI_vPDInCnf(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p)
{
    IOLM_SMI_SPDIn *psuPDIn = (IOLM_SMI_SPDIn *)pu8ArgBlock_p;
    INT8U au8Data[32] = {0, };

    // Copy data
    memcpy(au8Data, psuPDIn->au8Data, psuPDIn->u8InputDataLength);

    // ToDo: Insert application specific code here
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID, IOLM_SMI_u16PDInReqCnf

\ingroup grp_smi_pd

*/
typedef void (*IOLM_SMI_CBPDInCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);


/**
\brief Get input data request and confirmation.

This service allows for cyclically reading input Process Data to an input buffer
and combines request (#IOLM_SMI_vPDInReq) and confirmation (#IOLM_SMI_CBPDInCnf).

\param[in]      u8Port_p                Port ID.
\param[inout]   pu16ArgBlockLength_p    Pointer which points to the length of ArgBlock.
\param[inout]   pu8ArgBlock_p           Data pointer which points to the PDIn data (#IOLM_SMI_SPDIn).

\return Error as #IOL_EErrorType

\par Example

\code{.c}
IOLM_SMI_SPDIn suPDIn;
INT16U u16Length = sizeof(IOLM_SMI_SPDIn);
INT16U u16Error;

// Get PDIn
u16Error = IOLM_SMI_u16PDInReqCnf(u8Port, &u16Length, (INT8U *)&suPDIn);
if (u16Error == IOL_eErrorType_NONE)
{
    // ToDo: Insert application specific code here
}
\endcode

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL INT16U IOLM_SMI_u16PDInReqCnf(INT8U u8Port_p, INT16U *pu16ArgBlockLength_p, INT8U *pu8ArgBlock_p);


/**
\brief Set output data.

This service allows for cyclically writing output Process Data to an output buffer.
The answer to the request is the function #IOLM_SMI_CBPDOutCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the PDOut data (#IOLM_SMI_SPDOut).
\par Example

\code{.c}
IOLM_SMI_SPDOut *psuReq = malloc(sizeof(IOLM_SMI_SPDOut));

// Fill request
psuReq->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_PDOut);
...

// Send request
IOLM_SMI_vPDOutReq(
    0,                          // Client ID
    0,                          // Port
    sizeof(IOLM_SMI_SPDOut),    // Length
    (INT8U *)psuReq
);

// Wait for IOLM_SMI_CBPDOutCnf
\endcode

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL void IOLM_SMI_vPDOutReq(INT8U u8ClientID_p, INT8U u8Port_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

/**
\fn IOLM_SMI_CBPDOutCnf
\brief Set output data confirmation callback.

This service allows for cyclically writing output Process Data to an output buffer.
Confirmation to the #IOLM_SMI_vPDOutReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_pd

*/
typedef void (*IOLM_SMI_CBPDOutCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p);


/**
\brief Set output data request and confirmation.

This service allows for cyclically writing output Process Data to an output buffer and combines request and confirmation.

\param[in]     u8Port_p                Port ID.
\param[inout]  u16ArgBlockLength_p     Pointer which points to the Length of ArgBlock.
\param[inout]  pu8ArgBlock_p           Data pointer which points to the PDOut data (#IOLM_SMI_SPDOut).

\return Error as #IOL_EErrorType

\par Example

\code{.c}
IOLM_SMI_SPDOut suPDOut;
INT16U u16Length = sizeof(IOLM_SMI_SPDOut);
INT16U u16Error;

// Set PDOut
u16Error = IOLM_SMI_u16PDOutReqCnf(u8Port, &u16Length, (INT8U *)&suPDOut);
if (u16Error == IOL_eErrorType_NONE)
{
    // ToDo: Insert application specific code here
}
\endcode

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL INT16U IOLM_SMI_u16PDOutReqCnf(INT8U u8Port_p, INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);

#ifdef IOL_SAFETY
/**
\brief Get SPDU input data.

This service allows for cyclically reading safety protocol data units from an FSInBuffer.
The answer to the request is the function #IOLM_SMI_CBSPDUInCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL void IOLM_SMI_vSPDUInReq(INT8U u8ClientID_p, INT8U u8Port_p);

/**
\fn IOLM_SMI_CBSPDUInCnf
\brief Get SPDU input data confirmation.

This service allows for cyclically reading safety protocol data units from an FSInBuffer.
Confirmation to the #IOLM_SMI_vSPDUInReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the SPDUIn data (#IOLM_SMI_SSPDUIn).


\par Example

\code{.c}
void IOLM_SMI_vSPDUInCnf(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p)
{
    IOLM_SMI_SSPDUIn* psuSPDU = (IOLM_SMI_SSPDUIn * )pu8ArgBlock_p;
    INT8U au8SPDU[32] = {0, };

    // Copy data
    memcpy(au8SPDU, psuSPDU->au8Data, psuSPDU->u8SPDUInputDataLength);

    // ToDo: Insert application specific code here
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID, IOLM_SMI_u16SPDUInReqCnf

\ingroup grp_smi_pd

*/
typedef void (*IOLM_SMI_CBSPDUInCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U* pu8ArgBlock_p);

/**
\brief Get SPDU input data request and confirmation.

This service allows for cyclically reading safety protocol data units from an FSInBuffer and combines request and confirmation.

\param[in]      u8Port_p                Port ID.
\param[inout]   pu16ArgBlockLength_p    Pointer which points to the length of ArgBlock.
\param[inout]   pu8ArgBlock_p           Data pointer which points to the SPDUIn data (#IOLM_SMI_SSPDUIn).

\return Error as #IOL_EErrorType

\par Example

\code{.c}
IOLM_SMI_SSPDUIn suSPDUIn;
INT16U u16Length = sizeof(IOLM_SMI_SSPDUIn);
INT16U u16Error;

// Get SPDUIn
u16Error = IOLM_SMI_u16SPDUInReqCnf(u8Port, &u16Length, (INT8U *)&suSPDUIn);
if (u16Error == IOL_eErrorType_NONE)
{
    // ToDo: Insert application specific code here
}
\endcode

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL INT16U IOLM_SMI_u16SPDUInReqCnf(INT8U u8Port_p, INT16U* pu16ArgBlockLength_p, INT8U* pu8ArgBlock_p);


/**
\brief Set SPDU output data.

This service allows for cyclically writing safety protocol data units (SPDU) to an FSOutBuffer.
The answer to the request is the function #IOLM_SMI_CBSPDUOutCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data pointer which points to the SPDUOut data (#IOLM_SMI_SSPDUOut).
\par Example

\code{.c}
IOLM_SMI_SSPDUOut *psuReq = malloc(sizeof(IOLM_SMI_SSPDUOut));

// Fill request
psuReq->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_SPDUOut);
...

// Send request
IOLM_SMI_vSPDUOutReq(
    0,                          // Client ID
    0,                          // Port
    sizeof(IOLM_SMI_SSPDUOut),    // Length
    (INT8U *)psuReq
);

// Wait for IOLM_SMI_CBSPDUOutCnf
\endcode

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL void IOLM_SMI_vSPDUOutReq(INT8U u8ClientID_p, INT8U u8Port_p,
    INT16U u16ArgBlockLength_p, INT8U* pu8ArgBlock_p);

/**
\fn IOLM_SMI_CBSPDUOutCnf
\brief Set SPDU output data confirmation.

This service allows for cyclically writing safety protocol data units (SPDU) to an FSOutBuffer.
Confirmation to the #IOLM_SMI_vSPDUOutReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.

\ingroup grp_smi_pd


*/
typedef void (*IOLM_SMI_CBSPDUOutCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p);


/**
\brief Set SPDU output data request and confirmation.

This service allows for cyclically writing safety protocol data units (SPDU) to an FSOutBuffer and combines request and confirmation.

\param[in]     u8Port_p                Port ID.
\param[inout]  u16ArgBlockLength_p     Pointer which points to the length of ArgBlock.
\param[inout]  pu8ArgBlock_p           Data pointer which points to the PDOut data (#IOLM_SMI_SSPDUOut).

\return Error as #IOL_EErrorType

\par Example

\code{.c}
IOLM_SMI_SSPDUOut suSPDUOut;
INT16U u16Length = sizeof(IOLM_SMI_SSPDUOut);
INT16U u16Error;

// Set SPDUout
u16Error = IOLM_SMI_u16SPDUOutReqCnf(u8Port, &u16Length, (INT8U *)&suSPDUOut);
if (u16Error == IOL_eErrorType_NONE)
{
    // ToDo: Insert application specific code here
}
\endcode

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL INT16U IOLM_SMI_u16SPDUOutReqCnf(INT8U u8Port_p, INT16U u16ArgBlockLength_p, INT8U* pu8ArgBlock_p);

#endif

#ifndef IOLM_WIRELESS
/**
\brief Port power off on request and confirmation.

This service allows for switching power of a particular port off and on.

\param[in]     u8Port_p                Port ID.
\param[inout]  u16ArgBlockLength_p     Pointer which points to the length of ArgBlock.
\param[inout]  pu8ArgBlock_p           Data pointer which points to the PortPowerOffOn data (#IOLM_SMI_SPortPowerOffOn).

\return Error as #IOL_EErrorType

\par Example

\code{.c}
IOLM_SMI_SPortPowerOffOn suPortPowerOffOn;
INT16U u16Error;

// Fill in request
suPortPower.u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_PortPowerOffOn);
suArgBlock.pu8ArgBlock = (INT8U*)&suPortPowerOffOn;
suArgBlock.u16ArgBlockLength = sizeof(suPortPowerOffOn);
suArgBlock.u16ArgBlockLengthMax = sizeof(suPortPowerOffOn);

// Set PowerOff time to 500 ms and select ONE_TIME_SWITCH_OFF to switch off port power only for 500 ms.
// To switch off permanently use IOLM_SMI_ePortPowerMode_OFF
// and to switch on use IOLM_SMI_ePortPowerMode_ON
suPortPower.u8PortPowerMode = (INT8U)IOLM_SMI_ePortPowerMode_ONE_TIME_SWITCH_OFF;
suPortPower.u16PowerOffTime = IOLM_SMI_ENDIAN_16(500);

u16Error = (IOL_EErrorType)IOLM_SMI_u16PortPowerOffOnCnf(u8Port, sizeof(IOLM_SMI_SPortPowerOffOn), &suPortPowerOffOn);

if (u16Error =/= IOL_eErrorType_NONE)
{
    // ToDo: Insert application specific code here
}
\endcode

\ingroup grp_smi_config
*/
IOL_FUNC_DECL INT16U IOLM_SMI_u16PortPowerOffOnReqCnf(INT8U u8Port_p, INT16U u16ArgBlockLength_p, INT8U* pu8ArgBlock_p);
#endif

/**
\brief Get input and set output data.

This service allows for periodically reading input from an InBuffer and periodically reading output Process Data from an OutBuffer.
The answer to the request is the function #IOLM_SMI_CBPDInOutCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL void IOLM_SMI_vPDInOutReq(INT8U u8ClientID_p, INT8U u8Port_p);

/**
\fn IOLM_SMI_CBPDInOutCnf
\brief Get input and set output data confirmation.

This service allows for periodically reading input from an InBuffer and periodically reading output Process Data from an OutBuffer.
Confirmation to the #IOLM_SMI_vPDInOutReq request.

\param[in]      u8ClientID_p            Client ID.
\param[in]      u8Port_p                Port ID.
\param[in]      u16Error_p              Error Message as #IOL_EErrorType.
\param[in]      u16ArgBlockLength_p     Length of ArgBlock.
\param[inout]   pu8ArgBlock_p           Data Pointer which points to the PDInOut Data (#IOLM_SMI_SPDInOut).

\par Example

\code{.c}
void IOLM_SMI_vPDInOutCnf(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p)
{
    IOLM_SMI_SPDInOut *psuPDInOut = (IOLM_SMI_SPDInOut *)pu8ArgBlock_p;
    INT8U au8Data[2*(32+1)] = {0, };

    // Copy data
    memcpy(au8Data, psuPDInOut->au8Data, psuPDInOut->u8InputDataLength);

    // ToDo: Insert application specific code here
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_pd

*/
typedef void (*IOLM_SMI_CBPDInOutCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);


/**
\brief Get input and output data request and confirmation.

This service allows for cyclically reading input and output Process Data to an input buffer
and combines request (#IOLM_SMI_vPDInOutReq) and confirmation (#IOLM_SMI_CBPDInOutCnf).

\param[in]      u8Port_p                Port ID.
\param[inout]   pu16ArgBlockLength_p    Pointer which points to the length of ArgBlock.
\param[inout]   pu8ArgBlock_p           Data pointer which points to the PDInOut data (#IOLM_SMI_SPDInOut).

\return Error as #IOL_EErrorType

\par Example

\code{.c}
IOLM_SMI_SPDInOut suPDInOut;
INT16U u16Length = sizeof(IOLM_SMI_SPDInOut);
INT16U u16Error;

// Get PDIn
u16Error = IOLM_SMI_u16PDInReqCnf(u8Port, &u16Length, (INT8U *)&suPDInOut);
if (u16Error == IOL_eErrorType_NONE)
{
    // ToDo: Insert application specific code here
}
\endcode

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL INT16U IOLM_SMI_u16PDInOutReqCnf(INT8U u8Port_p, INT16U* pu16ArgBlockLength_p, INT8U* pu8ArgBlock_p);

#ifndef IOLM_WIRELESS
/**
\brief Get input data.

This service allows for cyclically reading input Process Data from an InBuffer containing the value of the input "I" signal (Pin 2 at M12).
The answer to the request is the function #IOLM_SMI_CBPDInIQCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL void IOLM_SMI_vPDInIQReq(INT8U u8ClientID_p, INT8U u8Port_p);
#endif

#ifndef IOLM_WIRELESS
/**
\fn IOLM_SMI_CBPDInIQCnf
\brief Get input data confirmation callback.

This service allows for cyclically reading input Process Data from an InBuffer containing the value of the input "I" signal (Pin 2 at M12).
Confirmation to the #IOLM_SMI_vPDInIQReq request.

\param[in]      u8ClientID_p            Client ID.
\param[in]      u8Port_p                Port ID.
\param[in]      u16Error_p              Error Message as #IOL_EErrorType.
\param[in]      u16ArgBlockLength_p     Length of ArgBlock.
\param[in]      pu8ArgBlock_p           Data Pointer which points to the PDInIQ Data.

\par Example

\code{.c}
void IOLM_SMI_vPDInIQCnf(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p)
{
    IOLM_SMI_SPDInIQ *psuPDInIQ = (IOLM_SMI_SPDInIQ *)pu8ArgBlock_p;

    // State of the input
    psuPDInIQ->u8PQI;

    // ToDo: Insert application specific code here
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_pd

*/
typedef void (*IOLM_SMI_CBPDInIQCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);
#endif


#ifndef IOLM_WIRELESS
/**
\brief Set Output Data.

This service allows for cyclically writing output Process Data to an OutBuffer containing the value of the output "Q" signal (Pin 2 at M12).
The answer to the request is the function #IOLM_SMI_CBPDOutIQCnf.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16ArgBlockLength_p     Length of ArgBlock.
\param[in]  pu8ArgBlock_p           Data Pointer which points to the PDOutIQ Data.

\par Example

\code{.c}
IOLM_SMI_SPDOutIQ *psuReq = malloc(sizeof(IOLM_SMI_SPDOutIQ));

// Fill request
psuReq->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_PDOutIQ);
...

// Send request
IOLM_SMI_vPDOutIQReq(
    0,                          // Client ID
    0,                          // Port
    sizeof(IOLM_SMI_SPDOutIQ),    // Length
    (INT8U *)psuReq
);

// Wait for IOLM_SMI_vPDOutIQCnf
\endcode

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL void IOLM_SMI_vPDOutIQReq(INT8U u8ClientID_p, INT8U u8Port_p,
    INT16U u16ArgBlockLength_p, INT8U *pu8ArgBlock_p);
#endif 


#ifndef IOLM_WIRELESS
/**
\fn IOLM_SMI_CBPDOutIQCnf
\brief Set output data confirmation callback.

This service allows for cyclically writing output Process Data to an OutBuffer containing the value of the output "Q" signal (Pin 2 at M12).
Confirmation to the #IOLM_SMI_vPDOutIQReq request.

\param[in]  u8ClientID_p            Client ID.
\param[in]  u8Port_p                Port ID.
\param[in]  u16Error_p              Error message as #IOL_EErrorType.

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_pd

*/
typedef void (*IOLM_SMI_CBPDOutIQCnf)(INT8U u8ClientID_p, INT8U u8Port_p, INT16U u16Error_p);
#endif 


/**
\fn IOLM_SMI_CBLoadNVCfg
\brief Load configuration callback.

This callback service is called by the stack and requests the saved configuration struct from non volatile memory.
Since this is hardware specific, it has to be implemented in the application code.

\param[in]  u8Instance_p       0-x for DataStorage Port and 0xFF for SMI configuration.
\param[in]  pu8Data_p          Pointer to the configuration struct to be loaded (#IOLM_SMI_SNVConfiguration).
\param[in]  pu32Length_p       Pointer to the length of the configuration struct.

\par Example

\code{.c}
void LOLM_SMI_vLoadNVCfg(INT8U u8Instance_p, INT8U *pu8Data_p, INT32U *pu32Length_p)
{
    // Create local configuration struct in your application
    IOLM_SMI_SNVConfiguration sNVConfiguration;

    // Load configuration from non-volatile memory into struct
    // ToDo: Insert hardware specific code here

    // Copy configuration to pu8Data_p for the stack
    memcpy(pu8Data_p, &sNVConfiguration, *pu32Length_p);
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_general

*/
typedef void(*IOLM_SMI_CBLoadNVCfg)(INT8U u8Instance_p, INT8U *pu8Data_p, INT32U *pu32Length_p);



/**
\fn IOLM_SMI_CBSaveNVCfg
\brief Save configuration callback.

This callback service is called by the stack and is used to save the configuration struct to non volatile memory.
If this function is finished writing the data it must call #IOLM_SMI_vSaveNvFinished
to notify the stack.
Since this is hardware specific, it has to be implemented in the application code.

\param[in]  u8Instance_p      0-x for Data Storage port and 0xFF for SMI configuration.
\param[in]  pu8Data_p         Pointer to the configuration struct to be saved (#IOLM_SMI_SNVConfiguration).
\param[in]  u32Length_p       Length of the configuration struct.

\par Example

\code{.c}
void LOLM_SMI_vSaveNVCfg(INT8U u8Instance_p, INT8U *pu8Data_p, INT32U u32Length_p)
{
    // Create local configuration struct in your application
    IOLM_SMI_SNVConfiguration *psuNVConfiguration = (IOLM_SMI_SNVConfiguration *)pu8Data_p;

    // Save configuration to non-volatile memory
    // ToDo: Insert hardware specific code here

    // Tell the stack that saving the configuration is finished
    IOLM_SMI_vSaveNvFinished();
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_general

*/
typedef void(*IOLM_SMI_CBSaveNVCfg)(INT8U u8Instance_p, INT8U *pu8Data_p, INT32U u32Length_p);


/**
\brief Save configuration finished.

This service needs to be called by the application code, to inform the stack,
that the configuration is successfully written to the non volatile memory function
in the application code (which can take a few hundred milliseconds).


\code{.c}
void LOLM_SMI_vSaveNVCfg(INT8U u8Instance_p, INT8U *pu8Data_p, INT32U u32Length_p)
{
    // Create local configuration struct in your application
    IOLM_SMI_SNVConfiguration *psuNVConfiguration = (IOLM_SMI_SNVConfiguration *)pu8Data_p;

    // Save configuration to non-volatile memory
    // ToDo: Insert hardware specific code here

    // Tell the stack that saving the configuration is finished
    IOLM_SMI_vSaveNvFinished();
}
\endcode

\see IOLM_SMI_CBSaveNVCfg

\ingroup grp_smi_general

*/
IOL_FUNC_DECL void IOLM_SMI_vSaveNvFinished(void);


/**
\fn IOLM_SMI_CBSMIExt
\brief SMI extension function callback.

Callback for extension functions which are proprietary and not included in the IOL/IOLW specification.
Can be used for additional application specific services, firmware updates, etc.
Since this is hardware specific, it has to be implemented in the application code.


\param[in]      u8Instance_p      Instance (defined by caller).
\param[in]      pu8Data_p         Pointer to the response data.
\param[in]      u16Length_p       Length of the response data

\par Example

\code{.c}
void IOLM_SMI_vExtensionService(INT8U u8Instance_p, INT8U *pu8Data_p, INT16U u16Length_p)
{
    IOL_EErrorType eErrorResponse;
    INT8U *pu8DataResponse;

    // ToDo: Insert application specific code here

    // Response to stack
    IOLM_SMI_vExtRsp(eErrorResponse, pu8DataResponse, 0);
}
\endcode

\see IOLM_SMI_SCallbacks, IOLM_SMI_vInit, IOLM_SMI_EArgBlockID

\ingroup grp_smi_general

*/
typedef void(*IOLM_SMI_CBSMIExt)(INT8U u8Instance_p, INT8U *pu8Data_p, INT16U u16Length_p);


/**
\brief SMI extension response.

Responses of extension functions which are proprietary and not included in the IOL/IOLW specification.


\param[in]      eError_p          Error message as #IOL_EErrorType.
\param[in]      pu8Data_p         Pointer to the response data.
\param[in]      u16Length_p       Length of the response data

\par Example

\code{.c}
void IOLM_SMI_vExtensionService(INT8U u8Instance_p, INT8U *pu8Data_p, INT16U u16Length_p)
{
    IOL_EErrorType eErrorResponse;
    INT8U *pu8DataResponse;

    // ToDo: Insert application specific code here

    // Response to stack
    IOLM_SMI_vExtRsp(eErrorResponse, pu8DataResponse, 0);
}
\endcode

\see IOLM_SMI_CBSMIExt

\ingroup grp_smi_general

*/
void IOLM_SMI_vExtRsp(IOL_EErrorType eError_p, INT8U *pu8Data_p, INT16U u16Length_p);

#ifdef IOL_SAFETY
/**
\brief Signalize SPDU Exchange has started.

This service needs to be called by the application code, to inform the stack,
that IO-Link Safety process data exchange has started.

\param[in]  u8Port_p                Port ID.

\ingroup grp_smi_pd

*/
IOL_FUNC_DECL void IOLM_SMI_vSPDUExchangeStarted(INT8U u8Port_p);

#endif

/**
\brief SMI Callbacks. Must be initialized via #IOLM_SMI_vInit.

Callbacks from the SMI API to the user application. Must be initialized via
#IOLM_SMI_vInit before running the user application and stack Mainloops.
*/
typedef struct IOLM_SMI_SCallbacks
{
    /** \brief Callback for #IOLM_SMI_vGenericReq. */
    IOLM_SMI_CBGenericCnf cbGenericCnf;
    /** \brief Callback for #IOLM_SMI_vMasterIdentificationReq. */
    IOLM_SMI_CBMasterIdentificationCnf cbMasterIdentificationCnf;
    /** \brief Callback for #IOLM_SMI_CBLoadMasterIdentification. */
    IOLM_SMI_CBLoadMasterIdentification cbLoadMasterIdentification;
#ifdef IOL_SAFETY
    /** \brief Callback for #IOLM_SMI_FSMasterAccessReq. */
    IOLM_SMI_CBFSMasterAccessCnf cbFSMasterAccessCnf;
    IOLM_SMI_CBFSPortConfiguration cbFSPortConfiguration;
#endif
    /** \brief Callback for #IOLM_SMI_vPortConfigurationReq. */
    IOLM_SMI_CBPortConfigurationCnf cbPortConfigurationCnf;
    /** \brief Callback for #IOLM_SMI_vReadbackPortConfigurationReq. */
    IOLM_SMI_CBReadbackPortConfigurationCnf cbReadbackPortConfigurationCnf;
    /** \brief Callback for #IOLM_SMI_vPortStatusReq. */
    IOLM_SMI_CBPortStatusCnf cbPortStatusCnf;
    /** \brief Callback for #IOLM_SMI_vDSBackupToParServReq. */
    IOLM_SMI_CBDSBackupToParServCnf cbDSBackupToParServCnf;
    /** \brief Callback for #IOLM_SMI_vDSBackupFromParServReq. */
    IOLM_SMI_CBDSBackupFromParServCnf cbDSBackupFromParServCnf;
    /** \brief Callback for #IOLM_SMI_vDeviceWriteReq. */
    IOLM_SMI_CBDeviceWriteCnf cbDeviceWriteCnf;
    /** \brief Callback for #IOLM_SMI_vDeviceReadReq. */
    IOLM_SMI_CBDeviceReadCnf cbDeviceReadCnf;
    /** \brief Callback for Device Events. */
    IOLM_SMI_CBDeviceEventInd cbDeviceEventInd;
    /** \brief Callback for Port Events. */
    IOLM_SMI_CBPortEventInd cbPortEventInd;
    /** \brief Callback for #IOLM_SMI_vPDInReq if a single port was read. */
    IOLM_SMI_CBPDInCnf cbPDInCnf;
    /** \brief Callback for #IOLM_SMI_vPDOutReq if a single port was updated. */
    IOLM_SMI_CBPDOutCnf cbPDOutCnf;
    /** \brief Callback for #IOLM_SMI_vPDInOutReq. */
    IOLM_SMI_CBPDInOutCnf cbPDInOutCnf;
#ifdef IOL_SAFETY
    /** \brief Callback for #IOLM_SMI_vSPDUInReq. */
    IOLM_SMI_CBSPDUInCnf cbSPDUInCnf;
    /** \brief Callback for #IOLM_SMI_vSPDUOutReq. */
    IOLM_SMI_CBSPDUOutCnf cbSPDUOutCnf;
#endif
#ifndef IOLM_WIRELESS
    /** \brief Callback for #IOLM_SMI_vPDInIQReq. */
    IOLM_SMI_CBPDInIQCnf cbPDInIQCnf;
    /** \brief Callback for #IOLM_SMI_vPDOutIQReq. */
    IOLM_SMI_CBPDOutIQCnf cbPDOutIQCnf;
#endif
#ifdef IOLM_WIRELESS
    /** \brief Callback for #IOLM_SMI_vTrackConfigurationReq . */
    IOLM_SMI_CBTrackConfigurationCnf cbTrackConfigurationCnf;
    /** \brief Callback for #IOLM_SMI_vReadbackTrackConfigurationReq. */
    IOLM_SMI_CBReadbackTrackConfigurationCnf cbReadbackTrackConfigurationCnf;
    /** \brief Callback for #IOLM_SMI_vTrackStatusReq. */
    IOLM_SMI_CBTrackStatusCnf cbTrackStatusCnf;
    /** \brief Callback for #IOLM_SMI_vScanReq. */
    IOLM_SMI_CBScanCnf cbScanCnf;
    /** \brief Callback for found Devices in scan mode. */
    IOLM_SMI_CBTrackScanInd cbTrackScanInd;
    /** \brief Callback for Device scan has finished. */
    IOLM_SMI_CBTrackScanEndInd cbTrackScanEndInd;
    /** \brief Callback for #IOLM_SMI_vPortPairingReq. */
    IOLM_SMI_CBPortPairingCnf cbPortPairingCnf;
#endif
    /** \brief Callback for #IOLM_SMI_vMasterConfigurationReq. */
    IOLM_SMI_CBMasterConfigurationCnf cbMasterConfigurationCnf;
    /** \brief Callback for loading non volatile configuration . */
    IOLM_SMI_CBLoadNVCfg cbLoadNvCfg;
    /** \brief Callback for saving non volatile configuration. */
    IOLM_SMI_CBSaveNVCfg cbSaveNvCfg;
    /** \brief Callback for ArgBlock free. */

    /** \brief Callback to inform about new PDCycle trigger. */
    IOLM_SMI_CBSMIExt cbALPDCycle;
    /** \brief Callback for firmware update. */
    IOLM_SMI_CBSMIExt cbFwUpdate;
    /** \brief Callback for settings. */
    IOLM_SMI_CBSMIExt cbSettings;
    /** \brief Callback for chip info. */
    IOLM_SMI_CBSMIExt cbChipInfo;
    /** \brief Callback for reboot. */
    IOLM_SMI_CBSMIExt cbResetDefaults;
#ifdef IOL_SAFETY
    /** \brief Callback for FS Process Data read. */
    IOLM_SMI_CBSMIExt cbFSPDIn;
    /** \brief Callback for FS Process Data write. */
    IOLM_SMI_CBSMIExt cbFSPDOut;
    /** \brief Callback for FS VerifyRecord write. */
    IOLM_SMI_CBSMIExt cbFSWriteVerifyRecord;
    /** \brief Callback for FS Master Access. */
    IOLM_SMI_CBSetFSMasterAccessAndGetFSCPAuthenticity cbSetFSMasterAccessAndGetFSCPAuthenticity;
#endif
#ifdef IOLM_OS_AVAILABLE
    /** \brief Callback for mainloop request */
    IOLM_SMI_CBMainLoopRequest cbMainLoopRequest;
#endif

#ifdef IOLM_SGI_ENABLED
    IOLM_SMI_CBGenericCnf cbSGICallbacks[IOLM_SMI_CLIENT_COUNT];
#endif
    
}IOLM_SMI_SCallbacks;

/**
\brief SMI Mainloop.

Runs periodically to execute the next jobs in the queue, etc.
Must be called at least once every millisecond to guarantee a
stable stack environment.

\par Example

\code{.c}
// ToDo: Setup the desired callbacks
IOLM_SMI_SCallbacks suSMICallbacks_g =
{
    .cbGenericCnf = IOLM_SMI_vGenericCnf,
    .cbMemFree = IOLM_SMI_vMemFree,
    .cbMemAlloc = LOLM_SMI_pu8ArgBlockAlloc,
    .cbLoadNvCfg = ...
    ...
};

// Initialize stack and SMI (suSMICallbacks_g must be global)
IOLM_SMI_vInit(&suSMICallbacks_g);

// Application Mainloop
while (1)
{
    IOLM_SMI_vRun();    // SMI Mainloop (at least once per ms)
    User_Application(); // ToDo: Insert application specific code here
}
\endcode

\ingroup grp_smi_general

*/
IOL_FUNC_DECL void IOLM_SMI_vRun(void);

/**
\brief Initialize SMI interface.

Initializes the SMI Interface, sets initial values,
loads saved configuration (if present), etc.
Must be called once before SMI Mainloop (#IOLM_SMI_vRun).

\param[in] psuCallbacks_p Set of callback functions.

\par Example

\code{.c}
// ToDo: Setup the desired callbacks
IOLM_SMI_SCallbacks suSMICallbacks_g =
{
    .cbGenericCnf = IOLM_SMI_cbGenericCnf,
    .cbPortConfigurationCnf = IOLM_SMI_cbPortConfigurationCn,
    .cbReadbackPortConfigurationCnf = IOLM_SMI_cbReadbackPortConfigurationCnf,
    .cbPortStatusCnf ...
    ...
};

// Initialize stack and SMI (suSMICallbacks_g must be global)
IOLM_SMI_vInit(&suSMICallbacks_g);

// Application Mainloop
while (1)
{
    IOLM_SMI_vRun();    // SMI Mainloop (at least once per ms)
    User_Application(); // ToDo: Insert application specific code here
}
\endcode

\see IOLM_SMI_SCallbacks

\ingroup grp_smi_general

*/
IOL_FUNC_DECL void IOLM_SMI_vInit(IOLM_SMI_SCallbacks *psuCallbacks_p);

#ifdef IOLM_SGI_ENABLED
IOL_FUNC_DECL void IOLM_SMI_ClientManagerInit(void);
#endif

/* for internal usage */

void IOLM_SMI_vInternalCnf(IOLM_SMI_SJob* psuJob_p, IOLM_SMI_SHeader* psuHeader_p, INT8U* pu8ArgBlock_p, INT16U u16Error_p);

void IOLM_SMI_vGenericReqInternal(INT8U u8ClientID_p, IOLM_SMI_EServiceID eService_p,
    TBOOL boGeneric_p, INT8U u8Instance_p, INT16U u16ExpArgBlock_p,
    INT16U u16ArgBlockLength_p, INT8U* pu8ArgBlock_p);

#ifdef __cplusplus
}
#endif



#endif

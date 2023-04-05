/*!
* \file FBTL_ECSLV_serviceTypes.h
*
* \brief
* FBTL EtherCAT slave service Types.
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

#if !(defined __FBTL_ECSLV_SERVICETYPES_H__)
#define __FBTL_ECSLV_SERVICETYPES_H__		1

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
typedef enum FBTL_ECSLV_EService
{
    FBTL_SVC_eECSLV_Unknown = 0x1000,

    /**
    \brief Initialize configuration
    */
    FBTL_SVC_eECSLV_initConfig,

    /**
    \brief Set Vendor ID
    */
    FBTL_SVC_eECSLV_setVendorId,

    /**
    \brief Set Device ID
    */
    FBTL_SVC_eECSLV_setProductCode,

    /**
    \brief Set Revision Number
    */
    FBTL_SVC_eECSLV_setRevisionNumber,

    /**
    \brief Set Serial Number
    */
    FBTL_SVC_eECSLV_setSerialNumber,

    /**
    \brief Set Product Name
    */
    FBTL_SVC_eECSLV_setProductName,

    /**
    \brief Set Group Type
    */
    FBTL_SVC_eECSLV_setGroupType,

    /**
    \brief Set Hw Version
    */
    FBTL_SVC_eECSLV_setHwVersion,

    /**
    \brief Set Sw Version
    */
    FBTL_SVC_eECSLV_setSwVersion,

    /**
    \brief Get Vendor ID
    */
    FBTL_SVC_eECSLV_getVendorId,

    /**
    \brief Get Product Code
    */
    FBTL_SVC_eECSLV_getProductCode,

    /**
    \brief Get Revision Number
    */
    FBTL_SVC_eECSLV_getRevisionNumber,

    /**
    \brief Get Serial Number
    */
    FBTL_SVC_eECSLV_getSerialNumber,

    /**
    \brief Get Product Name
    */
    FBTL_SVC_eECSLV_getProductName,

    /**
    \brief Get Group Type
    */
    FBTL_SVC_eECSLV_getGroupType,

    /**
    \brief Get Sw Revision
    */
    FBTL_SVC_eECSLV_getSwVersion,

    /**
    \brief Get Hw Revision
    */
    FBTL_SVC_eECSLV_getHwVersion,

    /**
    \brief Get EtherCAT State
    */
    //FBTL_SVC_eECSLV_getState,

    /**
    \brief Set EtherCAT State
    */
    FBTL_SVC_eECSLV_setState,

    /**
    \brief Get PDO Offset
    */
    FBTL_SVC_eECSLV_getPdoOffset,

    /**
    \brief Get PDO Bit SIze
    */
    FBTL_SVC_eECSLV_getPdoSize,

    /**
    \brief Get Input Processdata Length
    */
    FBTL_SVC_eECSLV_getInputProcDataLength,

    /**
    \brief Get Output Processdata Length
    */
    FBTL_SVC_eECSLV_getOutputProcDataLength,

    /**
    \brief Create PDO
    */
    FBTL_SVC_eECSLV_createPdo,

    /**
    \brief Create PDO Entry
    */
    FBTL_SVC_eECSLV_createPdoEntry,
    /**
    \brief Get PDO Entry Count
    */
    FBTL_SVC_eECSLV_getPdoEntryCount,
    /**
    \brief Get PDO Data Length
    */
    FBTL_SVC_eECSLV_getPdoEntryDataLength,

    /**
    \brief Set PDO Data
    */
    FBTL_SVC_eECSLV_setPdoData,

    /**
    \brief Get PDO Data
    */
    FBTL_SVC_eECSLV_getPdoData,

    /**
    \brief Set PDO Entry Data
    */
    FBTL_SVC_eECSLV_setPdoEntryData,

    /**
    \brief Get PDO Entry Data
    */
    FBTL_SVC_eECSLV_getPdoEntryData,

    /**
    \brief Fixed PDO
    */
    FBTL_SVC_eECSLV_fixedPdo,

    /**
    \brief PDO Assignment
    */
    FBTL_SVC_eECSLV_pdoAssignment,

    /**
    \brief Enable PDO
    */
    FBTL_SVC_eECSLV_enablePdo,

    /**
    \brief Disable PDO
    */
    FBTL_SVC_eECSLV_disablePdo,

    /**
    \brief Get PDO Enable state
    */
    FBTL_SVC_eECSLV_pdoIsEnabled,

    /**
    \brief Create Variable Sdo
    */
    FBTL_SVC_eECSLV_createVariableSdo,

    /**
    \brief Create Array Sdo
    */
    FBTL_SVC_eECSLV_createArraySdo,

    /**
    \brief Create Record Sdo
    */
    FBTL_SVC_eECSLV_createRecordSdo,

    /**
    \brief Config Record Subindex
    */
    FBTL_SVC_eECSLV_configRecordSubIndex,

    /**
    \brief Get Config Status
    */
    FBTL_SVC_eECSLV_getConfigStatus,

    /**
    \brief Get Cyclic Status
    */
    FBTL_SVC_eECSLV_getCyclicStatus,

    /**
    \brief Write Emergency
    */
    FBTL_SVC_eECSLV_writeEmergency,

    /**
    \brief Send/Receive EOE Frame
    */
    FBTL_SVC_eECSLV_EoEFrame,

    /**
    \brief Set EtherCAT Station Alias
    */
    FBTL_SVC_eECSLV_setStationAlias,

    /**
    \brief Get EtherCAT Stats
    */
    FBTL_SVC_eECSLV_getStats,

    /**
    \brief Set Slave Device Type
    */
    FBTL_SVC_eECSLV_setDeviceType,

    /**
    \brief Set PDO Size
    */
    FBTL_SVC_eECSLV_setPdoSize,

    /**
    \brief Set PDI Config
    */
    FBTL_SVC_eECSLV_setPdiCfg,

    /**
    \brief Clean up CoE object dictionary
    */
    FBTL_SVC_eECSLV_obdCleanUp,

    /**
    \brief Reset stack configuration preparation
    */
    FBTL_SVC_eECSLV_Reset,

    /**
    \brief Read ESC Register (Byte)
    */
    FBTL_SVC_eECSLV_ReadByteESCRegister,

    /**
    \brief Read ESC Register (Word)
    */
    FBTL_SVC_eECSLV_ReadWordESCRegister,

    /**
    \brief Read ESC Register (DWord)
    */
    FBTL_SVC_eECSLV_ReadDwordESCRegister,

    /**
    \brief Write ESC Register (Byte)
    */
    FBTL_SVC_eECSLV_WriteByteESCRegister,

    /**
    \brief Write ESC Register (Word)
    */
    FBTL_SVC_eECSLV_WriteWordESCRegister,

    /**
    \brief Write ESC Register (DWord)
    */
    FBTL_SVC_eECSLV_WriteDwordESCRegister,

    /**
    \brief Set ESC Error register
    */
    FBTL_SVC_eECSLV_setErrorRegister,

    /**
    \brief Enable Error Handler Callback
    */
    FBTL_SVC_eECSLV_registerErrorHandlerCb,

    /**
    \brief Error Handler Call
    */
    FBTL_SVC_eECSLV_ErrHandlerCall,

    /**
    \brief Enable EEPROM Load Callback
    */
    FBTL_SVC_eECSLV_registerEepromLoadCb,

    /**
    \brief EEPROM Load Call
    */
    FBTL_SVC_eECSLV_EepromLoadCall,

    /**
    \brief Enable EEPROM Write Callback
    */
    FBTL_SVC_eECSLV_registerEepromWriteCb,

    /**
    \brief EEPROM Write Call
    */
    FBTL_SVC_eECSLV_EepromWriteCall,

    /**
    \brief Enable Flash Init Callback
    */
    FBTL_SVC_eECSLV_registerFlashInitCb,

    /**
    \brief Flash Init Call
    */
    FBTL_SVC_eECSLV_FlashInitCall,

    /**
    \brief Enable Board Status LED Callback
    */
    //FBTL_SVC_eECSLV_registerBoardStatusLedCb,

    /**
    \brief Board Status LED Call
    */
    //FBTL_SVC_eECSLV_BoardStatusLedCall,

    /**
    \brief Enable CiA402 Set Dictionary Callback
    */
    FBTL_SVC_eECSLV_registerCiA402SetDictionaryCb,

    /**
    \brief CiA402 Set Dictionary Call
    */
    FBTL_SVC_eECSLV_CiA402SetDictionaryCall,

    /**
    \brief Enable CiA402 Set Dictionary Values Callback
    */
    FBTL_SVC_eECSLV_registerCiA402SetDictValsCb,

    /**
    \brief CiA402 Set Dictionary Values Call
    */
    FBTL_SVC_eECSLV_CiA402SetDictValsCall,

    /**
    \brief Enable CiA402 StateMachine Callback
    */
    FBTL_SVC_eECSLV_registerCiA402StateMachine,

    /**
    \brief CiA402 CiA402 StateMachine Call
    */
    FBTL_SVC_eECSLV_CiA402StateMachineCall,

    /**
    \brief Enable CiA402 Application Callback
    */
    FBTL_SVC_eECSLV_registerCiA402Application,

    /**
    \brief CiA402 CiA402 Application Call
    */
    FBTL_SVC_eECSLV_CiA402ApplicationCall,

    /**
    \brief Enable CiA402 Application Callback
    */
    FBTL_SVC_eECSLV_registerCiA402LocalError,

    /**
    \brief CiA402 Application Call
    */
    FBTL_SVC_eECSLV_CiA402LocalErrorCall,

    /**
    \brief CiA402 Set Axis Number
    */
    FBTL_SVC_eECSLV_CiA402SetAxisNumber,

    /**
    \brief CiA402 Activate Axis
    */
    FBTL_SVC_eECSLV_CiA402ActivateAxis,

    /**
    \brief CiA402 Set Pending Error Option Code
    */
    FBTL_SVC_eECSLV_CiA402_SM_clearErrorCode,

    /**
    \brief CiA402 Get Pending Error Option Code
    */
    FBTL_SVC_eECSLV_CiA402_SM_getErrorCode,

    /**
    \brief Enable EoE Receive Handler Callback
    */
    FBTL_SVC_eECSLV_registerEoEReceiveHandlerCb,

    /**
    \brief EoE Receive Handler call
    */
    FBTL_SVC_eECSLV_EoEReceiveCall,

    /**
    \brief Enable EoE Settings Indicator Callback
    */
    FBTL_SVC_eECSLV_registerEoESettingsIndCb,

    /**
    \brief EoE Settings Indicator call
    */
    FBTL_SVC_eECSLV_EoESettingsIndCall,

    /**
    \brief Enable User Application MainLoop Callback
    */
    FBTL_SVC_eECSLV_registerUsrApplMainLoopCb,

    /**
    \brief User Application MainLoop call
    */
    FBTL_SVC_eECSLV_UsrApplMainLoopCall,

    /**
    \brief Enable User Application Run Callback
    */
    FBTL_SVC_eECSLV_registerUsrApplRunCb,

    /**
    \brief User Application Run call
    */
    FBTL_SVC_eECSLV_UsrApplRunCall,

    /**
    \brief Enable Start Mailbox Handler Callback
    */
    FBTL_SVC_eECSLV_registerStartMbxHandlerCb,

    /**
    \brief Start Mailbox call
    */
    FBTL_SVC_eECSLV_StartMbxCall,

    /**
    \brief Enable Stop Mailbox Handler Callback
    */
    FBTL_SVC_eECSLV_registerStopMbxHandlerCb,

    /**
    \brief Stop Mailbox call
    */
    FBTL_SVC_eECSLV_StopMbxCall,

    /**
    \brief Enable Start Input Handler Callback
    */
    FBTL_SVC_eECSLV_registerStartInputHandlerCb,

    /**
    \brief Start Input call
    */
    FBTL_SVC_eECSLV_StartInputCall,

    /**
    \brief Enable Stop Input Handler Callback
    */
    FBTL_SVC_eECSLV_registerStopInputHandlerCb,

    /**
    \brief Stop Input call
    */
    FBTL_SVC_eECSLV_StopInputCall,

    /**
    \brief Enable Start Output Handler Callback
    */
    FBTL_SVC_eECSLV_registerStartOutputHandlerCb,

    /**
    \brief Start Output call
    */
    FBTL_SVC_eECSLV_StartOutputCall,

    /**
    \brief Enable Stop Output Handler Callback
    */
    FBTL_SVC_eECSLV_registerStopOutputHandlerCb,

    /**
    \brief Stop Output call
    */
    FBTL_SVC_eECSLV_StopOutputCall,

    /**
    \brief Enable FoE Open File Handler Callback
    */
    FBTL_SVC_eECSLV_registerFoEOpenFileHandlerCb,

    /**
    \brief FoE open file call
    */
    FBTL_SVC_eECSLV_FoEOpenFileCall,

    /**
    \brief Enable FoE Read File Handler Callback
    */
    FBTL_SVC_eECSLV_registerFoEReadFileHandlerCb,

    /**
    \brief FoE read file call
    */
    FBTL_SVC_eECSLV_FoEReadFileCall,

    /**
    \brief Enable FoE Write File Handler Callback
    */
    FBTL_SVC_eECSLV_registerFoEWriteFileHandlerCb,

    /**
    \brief FoE write file call
    */
    FBTL_SVC_eECSLV_FoEWriteFileCall,

    /**
    \brief Enable FoE Close File Handler Callback
    */
    FBTL_SVC_eECSLV_registerFoECloseFileHandlerCb,

    /**
    \brief FoE close file call
    */
    FBTL_SVC_eECSLV_FoECloseFileCall,

    /**
    \brief Enable FoE FW Update Handler Callback
    */
    FBTL_SVC_eECSLV_registerFoEFWUpdateHandlerCb,

    /**
    \brief FoE FW update call
    */
    FBTL_SVC_eECSLV_FoEFWUpdateCall,

    /**
    \brief Enable System Reboot Handler Callback
    */
    FBTL_SVC_eECSLV_registerSystemRebootHandlerCb,

    /**
    \brief System reboot call
    */
    FBTL_SVC_eECSLV_SystemRebootCall,

    /**
    \brief Set Firmware Update Mode
    */
    FBTL_SVC_eECSLV_setFwUpdateMode,

    /**
    \brief Set BootStrap Mailbox
    */
    FBTL_SVC_eECSLV_setBootStrapMailbox,

    /**
    \brief Set Standard Mailbox
    */
    FBTL_SVC_eECSLV_setStandardMailbox,

    /**
    \brief Set SyncManager config
    */
    FBTL_SVC_eECSLV_setSyncManConfig,

    /**
    \brief Set SyncManager error sequence counter limit
    */
    FBTL_SVC_eECSLV_setSyncManErrLimit,

    /**
    \brief Generate Object dictionary
    */
    FBTL_SVC_eECSLV_generateObjDict,

    /**
    \brief Enable SoE Send Handler Callback
    */
    FBTL_SVC_eECSLV_registerSoESendHandlerCb,

    /**
    \brief SoE send call
    */
    FBTL_SVC_eECSLV_SoESendCall,

    /**
    \brief Enable SoE Receive Handler Callback
    */
    FBTL_SVC_eECSLV_registerSoERecvHandlerCb,

    /**
    \brief SoE receive call
    */
    FBTL_SVC_eECSLV_SoERecvCall,

    /**
    \brief Enable SoE Notification Handler Callback
    */
    FBTL_SVC_eECSLV_registerSoENotifHandlerCb,

    /**
    \brief SoE notification call
    */
    FBTL_SVC_eECSLV_SoENotifCall,

    /**
    \brief SoE Notification Response
    */
    FBTL_SVC_eECSLV_SoENotifResponse,

    /**
    \brief Stack Version
    */
    FBTL_SVC_eECSLV_StackVersion,

    /**
    \brief Stack Version String
    */
    FBTL_SVC_eECSLV_StackVersionStr,

    /**
    \brief Stack Commit Id
    */
    FBTL_SVC_eECSLV_StackVersionId,

    /**
    \brief CoE Get object data
    */
    FBTL_SVC_eECSLV_CoEGetObjectData,

    /**
    \brief CoE Set object data
    */
    FBTL_SVC_eECSLV_CoESetObjectData,

    /**
    \brief CoE Get object entry data
    */
    FBTL_SVC_eECSLV_CoEGetObjectEntryData,

    /**
    \brief CoE Set object entry data
    */
    FBTL_SVC_eECSLV_CoESetObjectEntryData,

    /**
    \brief SDO Read Request from the Bus
    */
    FBTL_SVC_eECSLV_CoEReadCallback,

    /**
    \brief SDO Write Request from the Bus
    */
    FBTL_SVC_eECSLV_CoEWriteCallback,

    /**
    \brief CoE Get object entry count
    */
    FBTL_SVC_eECSLV_CoEGetObjectEntryCount,

    /**
    \brief CoE Get object type
    */
    FBTL_SVC_eECSLV_CoEGetObjectType,

    /**
    \brief Update Slave Stack state
    */
    FBTL_SVC_eECSLV_UpdateslaveState,

    /**
    \brief Set SubIdx write flag
    */
    FBTL_SVC_eECSLV_CoESubIdx0WrFlag,

    /**
    \brief Enable AoE Read Request Callback
    */
    FBTL_SVC_eECSLV_registerAoEReadRequestCb,

    /**
    \brief AoE read call
    */
    FBTL_SVC_eECSLV_AoEReadCall,

    /**
    \brief Enable AoE Write Request Callback
    */
    FBTL_SVC_eECSLV_registerAoEWriteRequestCb,

    /**
    \brief AoE write call
    */
    FBTL_SVC_eECSLV_AoEWriteCall,

    /**
    \brief CoE get object length
    */
    FBTL_SVC_eECSLV_CoEGetObjectLength,

    /**
    \brief CoE get object entry length
    */
    FBTL_SVC_eECSLV_CoEGetObjectEntryLength,
    /**
    \brief Register Pdo assignment change Callback
    */
    FBTL_SVC_eECSLV_registerPdoAssignmentChangesCb,

    /**
    \brief Pdo assignment change call
    */
    FBTL_SVC_eECSLV_PdoAssignmentChangeCall,

    /**
    \brief Register Pdo mapping change Callback
    */
    FBTL_SVC_eECSLV_registerPdoMappingChangesCb,

    /**
    \brief Pdo mapping change call
    */
    FBTL_SVC_eECSLV_PdoMappingChangeCall,

    /**
    \brief Pdo max subIndex
    */
    FBTL_SVC_eECSLV_PdoMaxSubIndex,

    /**
    \brief Get PDO List
    */
    FBTL_SVC_eECSLV_GetPdoAssigmentInfo,

    /**
    \brief Get PDO entry mapping
    */
    FBTL_SVC_eECSLV_GetPdoMappingInfo,

    /**
    \brief Enable Diagnosis
    */
    FBTL_SVC_eECSLV_DiagnosisEnable,

    /**
    \brief New Diagnosis Message
    */
    FBTL_SVC_eECSLV_DiagnosisNewMessage,

    FBTL_SVC_eECSLV_Max,
} FBTL_ECSLV_EService_t;

#if (defined __cplusplus)
extern "C" {
#endif

extern char* FBTL_SVC_ECSLV_enumString(FBTL_ECSLV_EService_t enumVal);

#if (defined __cplusplus)
}
#endif

#endif /* __FBTL_ECSLV_SERVICETYPES_H__ */

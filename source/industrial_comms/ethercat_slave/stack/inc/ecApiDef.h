/*!
* \file ecApiDef.h
*
* \brief
* EtherCAT defines.
*
* \author
* KUNBUS GmbH
*
* \date
* 2022-03-31
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

#if !(defined __ECAPIDEF_H__)
#define __ECAPIDEF_H__		1



#if (defined __cplusplus)
extern "C" {
#endif


/*-----------------------------------------------------------------------------------------
------
------    Defines and Types
------
-----------------------------------------------------------------------------------------*/

/*---------------------------------------------
-    DataTypes
-----------------------------------------------*/
/**
* \addtogroup SdoTypes SDO Datatype Definition
* @{
*/
#define     DEFTYPE_NULL                0x0000 /**< \brief Null*/
#define     DEFTYPE_BOOLEAN             0x0001 /**< \brief BOOLEAN*/
#define     DEFTYPE_INTEGER8            0x0002 /**< \brief INTEGER8*/
#define     DEFTYPE_INTEGER16           0x0003 /**< \brief INTEGER16*/
#define     DEFTYPE_INTEGER32           0x0004 /**< \brief INTEGER32*/
#define     DEFTYPE_UNSIGNED8           0x0005 /**< \brief UNSIGNED8*/
#define     DEFTYPE_UNSIGNED16          0x0006 /**< \brief UNSIGNED16*/
#define     DEFTYPE_UNSIGNED32          0x0007 /**< \brief UNSIGNED32*/
#define     DEFTYPE_REAL32              0x0008 /**< \brief REAL32*/
#define     DEFTYPE_VISIBLESTRING       0x0009 /**< \brief VISIBLE_STRING*/
#define     DEFTYPE_OCTETSTRING         0x000A /**< \brief OCTET_STRING*/
#define     DEFTYPE_UNICODE_STRING      0x000B /**< \brief UNICODE_STRING*/
#define     DEFTYPE_TIME_OF_DAY         0x000C /**< \brief TIME_OF_DAY*/
#define     DEFTYPE_TIME_DIFFERENCE     0x000D /**< \brief TIME_DIFFERENCE*/
#define     DEFTYPE_INTEGER24           0x0010 /**< \brief INTEGER24*/
#define     DEFTYPE_REAL64              0x0011 /**< \brief REAL64*/
#define     DEFTYPE_INTEGER40           0x0012 /**< \brief INTEGER40*/
#define     DEFTYPE_INTEGER48           0x0013 /**< \brief INTEGER48*/
#define     DEFTYPE_INTEGER56           0x0014 /**< \brief INTEGER56*/
#define     DEFTYPE_INTEGER64           0x0015 /**< \brief INTEGER64*/
#define     DEFTYPE_UNSIGNED24          0x0016 /**< \brief UNSIGNED24*/
#define     DEFTYPE_UNSIGNED40          0x0018 /**< \brief UNSIGNED40*/
#define     DEFTYPE_UNSIGNED48          0x0019 /**< \brief UNSIGNED48*/
#define     DEFTYPE_UNSIGNED56          0x001A /**< \brief UNSIGNED56*/
#define     DEFTYPE_UNSIGNED64          0x001B /**< \brief UNSIGNED64*/
#define     DEFTYPE_GUID                0x001D /**< \brief DEFTYPE_GUID*/
#define     DEFTYPE_BYTE                0x001E /**< \brief DEFTYPE_BYTE*/
#define     DEFTYPE_WORD                0x001F /**< \brief DEFTYPE_WORD*/
#define     DEFTYPE_DWORD               0x0020 /**< \brief DEFTYPE_DWORD*/
#define     DEFTYPE_PDOMAPPING          0x0021 /**< \brief PDO_MAPPING*/
#define     DEFTYPE_IDENTITY            0x0023 /**< \brief IDENTITY*/
#define     DEFTYPE_COMMAND             0x0025 /**< \brief COMMAND_PAR*/
#define     DEFTYPE_PDOCOMPAR           0x0027 /**< \brief PDO_PARAMETER*/
#define     DEFTYPE_ENUM                0x0028 /**< \brief DEFTYPE_ENUM */
#define     DEFTYPE_SMPAR               0x0029 /**< \brief SM_SYNCHRONISATION*/
#define     DEFTYPE_RECORD              0x002A /**< \brief DEFTYPE_RECORD */
#define     DEFTYPE_BACKUP              0x002B /**< \brief BACKUP_PARAMETER*/
#define     DEFTYPE_MDP                 0x002C /**< \brief MODULAR_DEVICE_PROFILE*/
#define     DEFTYPE_BITARR8             0x002D /**< \brief BITARR8*/
#define     DEFTYPE_BITARR16            0x002E /**< \brief BITARR16*/
#define     DEFTYPE_BITARR32            0x002F /**< \brief BITARR32*/
#define     DEFTYPE_BIT1                0x0030 /**< \brief BIT1*/
#define     DEFTYPE_BIT2                0x0031 /**< \brief BIT2*/
#define     DEFTYPE_BIT3                0x0032 /**< \brief BIT3*/
#define     DEFTYPE_BIT4                0x0033 /**< \brief BIT4*/
#define     DEFTYPE_BIT5                0x0034 /**< \brief BIT5*/
#define     DEFTYPE_BIT6                0x0035 /**< \brief BIT6*/
#define     DEFTYPE_BIT7                0x0036 /**< \brief BIT7*/
#define     DEFTYPE_BIT8                0x0037 /**< \brief BIT8*/
#define     DEFTYPE_ARRAY_OF_INT        0x0260 /**< \brief DEFTYPE_ARRAY_OF_INT*/
#define     DEFTYPE_ARRAY_OF_SINT       0x0261 /**< \brief DEFTYPE_ARRAY_OF_SINT*/
#define     DEFTYPE_ARRAY_OF_DINT       0x0262 /**< \brief DEFTYPE_ARRAY_OF_DINT*/
#define     DEFTYPE_ARRAY_OF_UDINT      0x0263 /**< \brief DEFTYPE_ARRAY_OF_UDINT*/
#define     DEFTYPE_ERRORHANDLING       0x0281 /**< \brief DEFTYPE_ERRORHANDLING*/
#define     DEFTYPE_DIAGHISTORY         0x0282 /**< \brief DEFTYPE_DIAGHISTORY*/
#define     DEFTYPE_SYNCSTATUS          0x0283 /**< \brief DEFTYPE_SYNCSTATUS*/
#define     DEFTYPE_SYNCSETTINGS        0x0284 /**< \brief DEFTYPE_SYNCSETTINGS*/
#define     DEFTYPE_FSOEFRAME           0x0285 /**< \brief DEFTYPE_FSOEFRAME*/
#define     DEFTYPE_FSOECOMMPAR         0x0286 /**< \brief DEFTYPE_FSOECOMMPAR*/
/** @}*/




/**
* \addtogroup SdoAccess SDO Access Rigths
* @{
*/
#define    ACCESS_READWRITE             0x003F /**< \brief Read/write in all states*/
#define    ACCESS_READ                  0x0007 /**< \brief Read only in all states*/
#define    ACCESS_READ_PREOP            0x0001 /**< \brief Read only in PreOP*/
#define    ACCESS_READ_SAFEOP           0x0002 /**< \brief Read only in SafeOP*/
#define    ACCESS_READ_OP               0x0004 /**< \brief Read only in OP*/
#define    ACCESS_WRITE                 0x0038 /**< \brief Write only in all states*/
#define    ACCESS_WRITE_PREOP           0x0008 /**< \brief Write only in PreOP*/
#define    ACCESS_WRITE_SAFEOP          0x0010 /**< \brief Write only in SafeOP*/
#define    ACCESS_WRITE_OP              0x0020 /**< \brief Write only in OP*/
#define    OBJACCESS_NOPDOMAPPING       0x0000 /**< \brief Not PDO mappable*/
#define    OBJACCESS_RXPDOMAPPING       0x0040 /**< \brief Mappable in RxPDOs*/
#define    OBJACCESS_TXPDOMAPPING       0x0080 /**< \brief Mappable in TxPDOs*/
#define    OBJACCESS_BACKUP             0x0100 /**< \brief Backup entry*/
#define    OBJACCESS_SETTINGS           0x0200 /**< \brief Setting Entry*/
#define    OBJACCESS_SAFEINPUTS         0x0400 /**< \brief Safe input*/
#define    OBJACCESS_SAFEOUTPUTS        0x0800 /**< \brief Safe output*/
#define    OBJACCESS_SAFEPARAMETER      0x1000 /**< \brief Safe parameter*/
/** @}*/


/**
* \addtogroup SDOAbort SDO Abort Codes
* @{
*/
#define     ABORT_NOERROR                                                   0x00000000 /**< \brief No SDO error*/
#define     ABORT_TOGGLE_BIT_NOT_CHANGED                                    0x05030000 /**< \brief Toggle bit not changed*/
#define     ABORT_SDO_PROTOCOL_TIMEOUT                                      0x05040000 /**< \brief SDO timeout*/
#define     ABORT_COMMAND_SPECIFIER_UNKNOWN                                 0x05040001 /**< \brief Command specifier unknown*/
#define     ABORT_OUT_OF_MEMORY                                             0x05040005 /**< \brief Out of memory*/
#define     ABORT_UNSUPPORTED_ACCESS                                        0x06010000 /**< \brief Unsupported Access*/
#define     ABORT_WRITE_ONLY_ENTRY                                          0x06010001 /**< \brief Write only entry*/
#define     ABORT_READ_ONLY_ENTRY                                           0x06010002 /**< \brief Read only entry*/
#define     ABORT_ENTRY_CANT_BE_WRITTEN_SI0_NOT_0                           0x06010003 /**< \brief Entry can not be written because Subindex0 is not 0*/
#define     ABORT_COMPLETE_ACCESS_NOT_SUPPORTED                             0x06010004 /**< \brief The object can not be accessed via complete access*/
#define     ABORT_OBJECT_NOT_EXISTING                                       0x06020000 /**< \brief Object not existing*/
#define     ABORT_OBJECT_CANT_BE_PDOMAPPED                                  0x06040041 /**< \brief Object can not be mapped to PDO*/
#define     ABORT_MAPPED_OBJECTS_EXCEED_PDO                                 0x06040042 /**< \brief Mapped Object exceeds PDO*/
#define     ABORT_PARAM_IS_INCOMPATIBLE                                     0x06040043 /**< \brief Parameter is incompatible*/
#define     ABORT_INTERNAL_DEVICE_INCOMPATIBILITY                           0x06040047 /**< \brief Device incompatibility*/
#define     ABORT_HARDWARE_ERROR                                            0x06060000 /**< \brief Hardware error*/
#define     ABORT_PARAM_LENGTH_ERROR                                        0x06070010 /**< \brief Parameter length error*/
#define     ABORT_PARAM_LENGTH_TOO_LONG                                     0x06070012 /**< \brief Parameter is too long*/
#define     ABORT_PARAM_LENGTH_TOO_SHORT                                    0x06070013 /**< \brief Parameter is too short*/
#define     ABORT_SUBINDEX_NOT_EXISTING                                     0x06090011 /**< \brief Subindex (Entry) not exists*/
#define     ABORT_VALUE_EXCEEDED                                            0x06090030 /**< \brief Value exceeds*/
#define     ABORT_VALUE_TOO_GREAT                                           0x06090031 /**< \brief Value is too great*/
#define     ABORT_VALUE_TOO_SMALL                                           0x06090032 /**< \brief Value is too small*/
#define     ABORT_MODULE_ID_LIST_NOT_MATCH                                  0x06090033 /**< \brief Detected Module Ident List (0xF030) and Configured Module Ident list (0xF050) does not match*/
#define     ABORT_MAX_VALUE_IS_LESS_THAN_MIN_VALUE                          0x06090036 /**< \brief Value is less than minimum value*/
#define     ABORT_GENERAL_ERROR                                             0x08000000 /**< \brief General error*/
#define     ABORT_DATA_CANNOT_BE_READ_OR_STORED                             0x08000020 /**< \brief Data can not be read or written*/
#define     ABORT_DATA_CANNOT_BE_READ_OR_STORED_BECAUSE_OF_LOCAL_CONTROL    0x08000021 /**< \brief Data can not be accessed because of local control*/
#define     ABORT_DATA_CANNOT_BE_READ_OR_STORED_IN_THIS_STATE               0x08000022 /**< \brief Data can not be read or written in the current state*/
#define     ABORT_NO_OBJECT_DICTIONARY_IS_PRESENT                           0x08000023 /**< \brief Object is not in the object dictionary*/
/** @}*/

/**
* \addtogroup ObjectTypes CoE Object types
* @{
*/
#define    OBJCODE_VAR                 0x07 /**< \brief Object code VARIABLE*/
#define    OBJCODE_ARR                 0x08 /**< \brief Object code ARRAY*/
#define    OBJCODE_REC                 0x09 /**< \brief Object code RECORD*/
/** @}*/


/**
* \addtogroup SoE SoE Service and Flags
* @{
*/
/*---------------------------------------------
-    SOE services
-----------------------------------------------*/
#define    ECAT_SOE_OPCODE_RRQ      0x0001 /**< \brief SoE Read Request*/
#define    ECAT_SOE_OPCODE_RRS      0x0002 /**< \brief SoE Read Response*/
#define    ECAT_SOE_OPCODE_WRQ      0x0003 /**< \brief SoE Write Request*/
#define    ECAT_SOE_OPCODE_WRS      0x0004 /**< \brief SoE Write Response*/
#define    ECAT_SOE_OPCODE_NFC      0x0005 /**< \brief SoE Notification Request*/
#define    ECAT_SOE_OPCODE_EMCY     0x0006 /**< \brief SoE Emergency*/

/*---------------------------------------------
-    SOE flags
-----------------------------------------------*/
#define    SOEFLAGS_OPCODE          0x0007    /**< \brief SoE Flags*/
                                                /**<
                                                * 0 = unused<br>
                                                * 1 = readReq<br>
                                                * 2 = readRes<br>
                                                * 3 = writeReq<br>
                                                * 4 = writeRes<br>
                                                * 5 = notification (command changed notification)*/
#define    SOEFLAGS_INCOMPLETE      0x0008    /**< \brief more follows*/
#define    SOEFLAGS_ERROR           0x0010    /**< \brief an error word follows*/
#define    SOEFLAGS_DRIVENO         0x00E0    /**< \brief drive number*/

#define    SOEFLAGS_DATASTATE       0x0100    /**< \brief Data state follows or requested*/
#define    SOEFLAGS_NAME            0x0200    /**< \brief Name follows or requested*/
#define    SOEFLAGS_ATTRIBUTE       0x0400    /**< \brief Attribute follows or requested*/
#define    SOEFLAGS_UNIT            0x0800    /**< \brief Unit follows or requested*/
#define    SOEFLAGS_MIN             0x1000    /**< \brief Min value follows or requested*/
#define    SOEFLAGS_MAX             0x2000    /**< \brief Max value follows or requested*/
#define    SOEFLAGS_VALUE           0x4000    /**< \brief Value follows or requested*/
#define    SOEFLAGS_DEFAULT         0x8000    /**< \brief Default value*/
/** @}*/

/**
* \addtogroup ALStatus AL Status Codes
* @{
*/
/*---------------------------------------------
-    AL Status Codes
-----------------------------------------------*/
#define    ALSTATUSCODE_NOERROR                        0x0000 /**< \brief No error*/
#define    ALSTATUSCODE_UNSPECIFIEDERROR               0x0001 /**< \brief Unspecified error*/
#define    ALSTATUSCODE_NOMEMORY                       0x0002 /**< \brief No Memory*/
#define    ALSTATUSCODE_INVALID_REVISION               0x0004 /**< \brief Output/Input mapping is not valid for this hardware or software revision (0x1018:03)*/
#define    ALSTATUSCODE_FW_SII_NOT_MATCH               0x0006 /**< \brief Firmware and EEPROM do not match. Slave needs BOOT-INIT transition*/
#define    ALSTATUSCODE_FW_UPDATE_FAILED               0x0007 /**< \brief Firmware update not successful. Old firmware still running*/
#define    ALSTATUSCODE_INVALIDALCONTROL               0x0011 /**< \brief Invalid requested state change*/
#define    ALSTATUSCODE_UNKNOWNALCONTROL               0x0012 /**< \brief Unknown requested state*/
#define    ALSTATUSCODE_BOOTNOTSUPP                    0x0013 /**< \brief Bootstrap not supported*/
#define    ALSTATUSCODE_NOVALIDFIRMWARE                0x0014 /**< \brief No valid firmware*/
#define    ALSTATUSCODE_INVALIDMBXCFGINBOOT            0x0015 /**< \brief Invalid mailbox configuration (BOOT state)*/
#define    ALSTATUSCODE_INVALIDMBXCFGINPREOP           0x0016 /**< \brief Invalid mailbox configuration (PreOP state)*/
#define    ALSTATUSCODE_INVALIDSMCFG                   0x0017 /**< \brief Invalid sync manager configuration*/
#define    ALSTATUSCODE_NOVALIDINPUTS                  0x0018 /**< \brief No valid inputs available*/
#define    ALSTATUSCODE_NOVALIDOUTPUTS                 0x0019 /**< \brief No valid outputs*/
#define    ALSTATUSCODE_SYNCERROR                      0x001A /**< \brief Synchronization error*/
#define    ALSTATUSCODE_SMWATCHDOG                     0x001B /**< \brief Sync manager watchdog*/
#define    ALSTATUSCODE_SYNCTYPESNOTCOMPATIBLE         0x001C /**< \brief Invalid Sync Manager Types*/
#define    ALSTATUSCODE_INVALIDSMOUTCFG                0x001D /**< \brief Invalid Output Configuration*/
#define    ALSTATUSCODE_INVALIDSMINCFG                 0x001E /**< \brief Invalid Input Configuration*/
#define    ALSTATUSCODE_INVALIDWDCFG                   0x001F /**< \brief Invalid Watchdog Configuration*/
#define    ALSTATUSCODE_WAITFORCOLDSTART               0x0020 /**< \brief Slave needs cold start*/
#define    ALSTATUSCODE_WAITFORINIT                    0x0021 /**< \brief Slave needs INIT*/
#define    ALSTATUSCODE_WAITFORPREOP                   0x0022 /**< \brief Slave needs PREOP*/
#define    ALSTATUSCODE_WAITFORSAFEOP                  0x0023 /**< \brief Slave needs SAFEOP*/
#define    ALSTATUSCODE_INVALIDINPUTMAPPING            0x0024 /**< \brief Invalid Input Mapping*/
#define    ALSTATUSCODE_INVALIDOUTPUTMAPPING           0x0025 /**< \brief Invalid Output Mapping*/
#define    ALSTATUSCODE_INCONSISTENTSETTINGS           0x0026 /**< \brief Inconsistent Settings*/
#define    ALSTATUSCODE_FREERUNNOTSUPPORTED            0x0027 /**< \brief FreeRun not supported*/
#define    ALSTATUSCODE_SYNCHRONNOTSUPPORTED           0x0028 /**< \brief SyncMode not supported*/
#define    ALSTATUSCODE_FREERUNNEEDS3BUFFERMODE        0x0029 /**< \brief FreeRun needs 3Buffer Mode*/
#define    ALSTATUSCODE_BACKGROUNDWATCHDOG             0x002A /**< \brief Background Watchdog*/
#define    ALSTATUSCODE_NOVALIDINPUTSANDOUTPUTS        0x002B /**< \brief No Valid Inputs and Outputs*/
#define    ALSTATUSCODE_FATALSYNCERROR                 0x002C /**< \brief Fatal Sync Error*/
#define    ALSTATUSCODE_NOSYNCERROR                    0x002D /**< \brief No Sync Error*/
#define    ALSTATUSCODE_CYCLETIMETOOSMALL              0x002E /**< \brief EtherCAT cycle time smaller Minimum Cycle Time supported by slave*/
#define    ALSTATUSCODE_DCINVALIDSYNCCFG               0x0030 /**< \brief Invalid DC SYNCH Configuration*/
#define    ALSTATUSCODE_DCINVALIDLATCHCFG              0x0031 /**< \brief Invalid DC Latch Configuration*/
#define    ALSTATUSCODE_DCPLLSYNCERROR                 0x0032 /**< \brief PLL Error*/
#define    ALSTATUSCODE_DCSYNCIOERROR                  0x0033 /**< \brief DC Sync IO Error*/
#define    ALSTATUSCODE_DCSYNCMISSEDERROR              0x0034 /**< \brief DC Sync Timeout Error*/
#define    ALSTATUSCODE_DCINVALIDSYNCCYCLETIME         0x0035 /**< \brief DC Invalid Sync Cycle Time*/
#define    ALSTATUSCODE_DCSYNC0CYCLETIME               0x0036 /**< \brief DC Sync0 Cycle Time*/
#define    ALSTATUSCODE_DCSYNC1CYCLETIME               0x0037 /**< \brief DC Sync1 Cycle Time*/
#define    ALSTATUSCODE_MBX_AOE                        0x0041 /**< \brief MBX_AOE*/
#define    ALSTATUSCODE_MBX_EOE                        0x0042 /**< \brief MBX_EOE*/
#define    ALSTATUSCODE_MBX_COE                        0x0043 /**< \brief MBX_COE*/
#define    ALSTATUSCODE_MBX_FOE                        0x0044 /**< \brief MBX_FOE*/
#define    ALSTATUSCODE_MBX_SOE                        0x0045 /**< \brief MBX_SOE*/
#define    ALSTATUSCODE_MBX_VOE                        0x004F /**< \brief MBX_VOE*/
#define    ALSTATUSCODE_EE_NOACCESS                    0x0050 /**< \brief EEPROM no access*/
#define    ALSTATUSCODE_EE_ERROR                       0x0051 /**< \brief EEPROM Error*/
#define    ALSTATUSCODE_EXT_HARDWARE_NOT_READY         0x0052 /**< \brief External hardware not ready. This AL Status Code should be used if the EtherCAT-Slave refused the state transition due to an external connection to another device or signal is missing*/
#define    ALSTATUSCODE_DEVICE_IDENT_VALUE_UPDATED     0x0061 /**< \brief In legacy identification mode (dip switch mapped to register 0x12) this error is returned if the EEPROM ID value does not match to dipswitch value*/
#define    ALSTATUSCODE_MODULE_ID_LIST_NOT_MATCH       0x0070 /**< \brief Detected Module Ident List (0xF030) and Configured Module Ident List (0xF050) does not match*/
#define    ALSTATUSCODE_SUPPLY_VOLTAGE_TOO_LOW         0x0080 /**< \brief The slave supply voltage is too low*/
#define    ALSTATUSCODE_SUPPLY_VOLTAGE_TOO_HIGH        0x0081 /**< \brief The slave supply voltage is too high*/
#define    ALSTATUSCODE_TEMPERATURE_TOO_LOW            0x0082 /**< \brief The slave temperature is too low*/
#define    ALSTATUSCODE_TEMPERATURE_TOO_HIGH           0x0083 /**< \brief The slave temperature is too high*/
#define    NOERROR_INWORK                              0x00FF /**< \brief Indication for no error but operation is pending*/
/**
* @}
*/

/*---------------------------------------------
-    CiA 402
-----------------------------------------------*/

/**
* \addtogroup CiA402 CiA402 Codes
* @{
*/
/*---------------------------------------------
-    ControlWord Commands Mask (IEC61800_184e)
-----------------------------------------------*/
#define CONTROLWORD_COMMAND_SHUTDOWN_MASK                    0x0087 /**< \brief Shutdown command mask*/
/*ECATCHANGE_START(V5.12) CIA402 2*/
#define CONTROLWORD_COMMAND_SWITCHON_MASK                    0x008F /**< \brief Switch on command mask*/
/*ECATCHANGE_END(V5.12) CIA402 2*/
#define CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION_MASK    0x008F /**< \brief Switch on & Enable command mask*/
#define CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK              0x0082 /**< \brief Disable voltage command mask*/
#define CONTROLWORD_COMMAND_QUICKSTOP_MASK                   0x0086 /**< \brief Quickstop command mask*/
#define CONTROLWORD_COMMAND_DISABLEOPERATION_MASK            0x008F /**< \brief Disable operation command mask*/
#define CONTROLWORD_COMMAND_ENABLEOPERATION_MASK             0x008F /**< \brief Enable operation command mask*/
#define CONTROLWORD_COMMAND_FAULTRESET_MASK                  0x0080 /**< \brief Fault reset command mask*/


/*---------------------------------------------
-    ControlWord Commands (IEC61800_184e)
-----------------------------------------------*/
#define CONTROLWORD_COMMAND_SHUTDOWN                         0x0006 /**< \brief Shutdown command*/
#define CONTROLWORD_COMMAND_SWITCHON                         0x0007 /**< \brief Switch on command*/
#define CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION         0x000F /**< \brief Switch on & Enable command*/
#define CONTROLWORD_COMMAND_DISABLEVOLTAGE                   0x0000 /**< \brief Disable voltage command*/
#define CONTROLWORD_COMMAND_QUICKSTOP                        0x0002 /**< \brief Quickstop command*/
#define CONTROLWORD_COMMAND_DISABLEOPERATION                 0x0007 /**< \brief Disable operation command*/
#define CONTROLWORD_COMMAND_ENABLEOPERATION                  0x000F /**< \brief Enable operation command*/
#define CONTROLWORD_COMMAND_FAULTRESET                       0x0080 /**< \brief Fault reset command*/


/*---------------------------------------------
-    StatusWord Masks and Flags
-----------------------------------------------*/
#define STATUSWORD_STATE_MASK                                0x006F /**< \brief State mask*/
#define STATUSWORD_VOLTAGE_ENABLED                           0x0010 /**< \brief Indicate high voltage enabled*/
#define STATUSWORD_WARNING                                   0x0080 /**< \brief Warning active*/
#define STATUSWORD_MANUFACTORSPECIFIC                        0x0100 /**< \brief Manufacturer specific*/
#define STATUSWORD_INTERNAL_LIMIT                            0x0800 /**< \brief Internal limit*/
#define STATUSWORD_REMOTE                                    0x0200 /**< \brief Set if the control word is processed*/
#define STATUSWORD_TARGET_REACHED                            0x0400 /**< \brief Target reached*/
#define STATUSWORD_INTERNALLIMITACTIVE                       0x0800 /**< \brief Internal limit active*/
#define STATUSWORD_DRIVE_FOLLOWS_COMMAND                     0x1000 /**< \brief Drive follows command (used in cyclic synchronous modes)*/


/*---------------------------------------------
-    StatusWord
-----------------------------------------------*/
#define STATUSWORD_STATE_NOTREADYTOSWITCHON                  0x0000 /**< \brief Not ready to switch on*/
/* ECATCHANGE_START(V5.12) CIA402 1*/
#define STATUSWORD_STATE_SWITCHEDONDISABLED                  0x0040 /**< \brief Switched on but disabled*/
/* ECATCHANGE_END(V5.12) CIA402 1*/
#define STATUSWORD_STATE_READYTOSWITCHON                     0x0021 /**< \brief Ready to switch on*/
#define STATUSWORD_STATE_SWITCHEDON                          0x0023 /**< \brief Switched on*/
#define STATUSWORD_STATE_OPERATIONENABLED                    0x0027 /**< \brief Operation enabled*/
#define STATUSWORD_STATE_QUICKSTOPACTIVE                     0x0007 /**< \brief Quickstop active*/
#define STATUSWORD_STATE_FAULTREACTIONACTIVE                 0x000F /**< \brief Fault reaction active*/
#define STATUSWORD_STATE_FAULT                               0x0008 /**< \brief Fault state*/


/*---------------------------------------------
-    CiA402 State machine
-----------------------------------------------*/
#define STATE_NOT_READY_TO_SWITCH_ON        0x0001 /**< \brief Not ready to switch on (optional)*/
#define STATE_SWITCH_ON_DISABLED            0x0002 /**< \brief Switch on but disabled (optional)*/
#define STATE_READY_TO_SWITCH_ON            0x0004 /**< \brief Ready to switch on (mandatory)*/
#define STATE_SWITCHED_ON                   0x0008 /**< \brief Switch on (mandatory)*/
#define STATE_OPERATION_ENABLED             0x0010 /**< \brief Operation enabled (mandatory)*/
#define STATE_QUICK_STOP_ACTIVE             0x0020 /**< \brief Quick stop active (optional)*/
#define STATE_FAULT_REACTION_ACTIVE         0x0040 /**< \brief Fault reaction active (mandatory)*/
#define STATE_FAULT                         0x0080 /**< \brief Fault state (mandatory)*/


/*---------------------------------------------
-    CiA402 Modes of Operation (object 0x6060) (IEC61800_184e)
-----------------------------------------------*/
// -128 to -1 Manufacturer-specific operation modes
#define NO_MODE                     0 /**< \brief No mode*/
#define PROFILE_POSITION_MODE       1 /**< \brief Position Profile mode*/
#define VELOCITY_MODE               2 /**< \brief Velocity mode*/
#define PROFILE_VELOCITY_MOCE       3 /**< \brief Velocity Profile mode*/
#define PROFILE_TORQUE_MODE         4 /**< \brief Torque Profile mode*/
//5 reserved
#define HOMING_MODE                 6 /**< \brief Homing mode*/
#define INTERPOLATION_POSITION_MODE 7 /**< \brief Interpolation Position mode*/
#define CYCLIC_SYNC_POSITION_MODE   8 /**< \brief Cyclic Synchronous Position mode*/
#define CYCLIC_SYNC_VELOCITY_MODE   9 /**< \brief Cyclic Synchronous Velocity mode*/
#define CYCLIC_SYNC_TORQUE_MODE     10/**< \brief Cyclic Synchronous Torque mode*/
//+11 to +127 reserved


/***************************************
 CiA402 Error Codes (object 0x603F) (IEC61800_184e)
 ***************************************/
#define ERROR_SHORT_CIRCUIT_EARTH_LEAKAGE_INPUT             0x2110 /**< \brief Short circuit/earth leakage (input)*/
#define ERROR_EARTH_LEAKAGE_INPUT                           0x2120 /**< \brief Earth leakage (input)*/
#define ERROR_EARTH_LEAKAGE_PHASE_L1                        0x2121 /**< \brief Earth leakage phase L1*/
#define ERROR_EARTH_LEAKAGE_PHASE_L2                        0x2122 /**< \brief Earth leakage phase L2*/
#define ERROR_EARTH_LEAKAGE_PHASE_L3                        0x2123 /**< \brief Earth leakage phase L3*/
#define ERROR_SHORT_CIRCUIT_INPUT                           0x2130 /**< \brief Short circuit (input)*/
#define ERROR_SHORT_CIRCUIT_PHASES_L1_L2                    0x2131 /**< \brief Short circuit phases L1-L2*/
#define ERROR_SHORT_CIRCUIT_PHASES_L2_L3                    0x2132 /**< \brief Short circuit phases L2-L3*/
#define ERROR_SHORT_CIRCUIT_PHASES_L3_L1                    0x2133 /**< \brief Short circuit phases L3-L1*/
#define ERROR_INTERNAL_CURRENT_NO1                          0x2211 /**< \brief Internal current no 1*/
#define ERROR_INTERNAL_CURRENT_NO2                          0x2212 /**< \brief Internal current no 2*/
#define ERROR_OVER_CURRENT_IN_RAMP_FUNCTION                 0x2213 /**< \brief Over-current in ramp function*/
#define ERROR_OVER_CURRENT_IN_THE_SEQUENCE                  0x2214 /**< \brief Over-current in the sequence*/
#define ERROR_CONTINUOUS_OVER_CURRENT_DEVICE_INTERNAL       0x2220 /**< \brief Continuous over current (device internal)*/
#define ERROR_CONTINUOUS_OVER_CURRENT_DEVICE_INTERNAL_NO1   0x2221 /**< \brief Continuous over current no 1*/
#define ERROR_CONTINUOUS_OVER_CURRENT_DEVICE_INTERNAL_NO2   0x2222 /**< \brief Continuous over current no 2*/
#define ERROR_SHORT_CIRCUIT_EARTH_LEAKAGE_DEVICE_INTERNAL   0x2230 /**< \brief Short circuit/earth leakage (device internal)*/
#define ERROR_EARTH_LEAKAGE_DEVICE_INTERNAL                 0x2240 /**< \brief Earth leakage (device internal)*/
#define ERROR_SHORT_CIRCUIT_DEVICE_INTERNAL                 0x2250 /**< \brief Short circuit (device internal)*/
#define ERROR_CONTINUOUS_OVER_CURRENT                       0x2310 /**< \brief Continuous over current*/
#define ERROR_CONTINUOUS_OVER_CURRENT_NO1                   0x2311 /**< \brief Continuous over current no 1*/
#define ERROR_CONTINUOUS_OVER_CURRENT_NO2                   0x2312 /**< \brief Continuous over current no 2*/
#define ERROR_SHORT_CIRCUIT_EARTH_LEAKAGE_MOTOR_SIDE        0x2320 /**< \brief Short circuit/earth leakage (motor-side)*/
#define ERROR_EARTH_LEAKAGE_MOTOR_SIDE                      0x2330 /**< \brief Earth leakage (motor-side)*/
#define ERROR_EARTH_LEAKAGE_PHASE_U                         0x2331 /**< \brief Earth leakage phase U*/
#define ERROR_EARTH_LEAKAGE_PHASE_V                         0x2332 /**< \brief Earth leakage phase V*/
#define ERROR_EARTH_LEAKAGE_PHASE_W                         0x2333 /**< \brief Earth leakage phase W*/
#define ERROR_SHORT_CIRCUIT_MOTOR_SIDE                      0x2340 /**< \brief Short circuit (motor-side)*/
#define ERROR_SHORT_CIRCUIT_PHASES_U_V                      0x2341 /**< \brief Short circuit phases U-V*/
#define ERROR_EARTH_LEAKAGE_PHASE_V_W                       0x2342 /**< \brief Earth leakage phase V-W*/
#define ERROR_EARTH_LEAKAGE_PHASE_W_U                       0x2343 /**< \brief Earth leakage phase W-U*/
#define ERROR_LOAD_LEVEL_FAULT_I2T_THERMAL_STATE            0x2350 /**< \brief Load level fault (I2t, thermal state)*/
#define ERROR_LOAD_LEVEL_WARNING_I2T_THERMAL_STATE          0x2351 /**< \brief Load level warning (I2t, thermal state)*/
#define ERROR_MAINS_OVER_VOLTAGE                            0x3110 /**< \brief Mains over-voltage*/
#define ERROR_MAINS_OVER_VOLTAGE_PHASE_L1                   0x3111 /**< \brief Mains over-voltage phase L1*/
#define ERROR_MAINS_OVER_VOLTAGE_PHASE_L2                   0x3112 /**< \brief Mains over-voltage phase L2 */
#define ERROR_MAINS_OVER_VOLTAGE_PHASE_L3                   0x3113 /**< \brief Mains over-voltage phase L3*/
#define ERROR_MAINS_UNDER_VOLTAGE                           0x3120 /**< \brief Mains under-voltage*/
#define ERROR_MAINS_UNDER_VOLTAGE_PHASE_L1                  0x3121 /**< \brief Mains under-voltage phase L1*/
#define ERROR_MAINS_UNDER_VOLTAGE_PHASE_L2                  0x3122 /**< \brief Mains under-voltage phase L2*/
#define ERROR_MAINS_UNDER_VOLTAGE_PHASE_L3                  0x3123 /**< \brief Mains under-voltage phase L3*/
#define ERROR_PHASE_FAILURE                                 0x3130 /**< \brief Phase failure*/
#define ERROR_PHASE_FAILURE_L1                              0x3131 /**< \brief Phase failure L1*/
#define ERROR_PHASE_FAILURE_L2                              0x3132 /**< \brief Phase failure L2*/
#define ERROR_PHASE_FAILURE_L3                              0x3133 /**< \brief Phase failure L3*/
#define ERROR_PHASE_SEQUENCE                                0x3134 /**< \brief Phase sequence*/
#define ERROR_MAINS_FREQUENCY                               0x3140 /**< \brief Mains frequency*/
#define ERROR_MAINS_FREQUENCY_TOO_GREAT                     0x3141 /**< \brief Mains frequency too great*/
#define ERROR_MAINS_FREQUENCY_TOO_SMALL                     0x3142 /**< \brief Mains frequency too small*/
#define ERROR_DC_LINK_OVER_VOLTAGE                          0x3210 /**< \brief DC link over-voltage*/
#define ERROR_OVER_VOLTAGE_NO_1                             0x3211 /**< \brief Over-voltage no  1*/
#define ERROR_OVER_VOLTAGE_NO_2                             0x3212 /**< \brief Over voltage no  2 */
#define ERROR_DC_LINK_UNDER_VOLTAGE                         0x3220 /**< \brief DC link under-voltage*/
#define ERROR_UNDER_VOLTAGE_NO_1                            0x3221 /**< \brief Under-voltage no  1*/
#define ERROR_UNDER_VOLTAGE_NO_2                            0x3222 /**< \brief Under-voltage no  2*/
#define ERROR_LOAD_ERROR                                    0x3230 /**< \brief Load error*/
#define ERROR_OUTPUT_OVER_VOLTAGE                           0x3310 /**< \brief Output over-voltage*/
#define ERROR_OUTPUT_OVER_VOLTAGE_PHASE_U                   0x3311 /**< \brief Output over-voltage phase U*/
#define ERROR_OUTPUT_OVER_VOLTAGE_PHASE_V                   0x3312 /**< \brief Output over-voltage phase V*/
#define ERROR_OUTPUT_OVER_VOLTAGE_PHASE_W                   0x3313 /**< \brief Output over-voltage phase W*/
#define ERROR_ARMATURE_CIRCUIT                              0x3320 /**< \brief Armature circuit*/
#define ERROR_ARMATURE_CIRCUIT_INTERRUPTED                  0x3321 /**< \brief Armature circuit interrupted*/
#define ERROR_FIELD_CIRCUIT                                 0x3330 /**< \brief Field circuit error */
#define ERROR_FIELD_CIRCUIT_INTERRUPTED                     0x3331 /**< \brief Field circuit interrupted*/
#define ERROR_EXCESS_AMBIENT_TEMPERATURE                    0x4110 /**< \brief Excess ambient temperature*/
#define ERROR_TOO_LOW_AMBIENT_TEMPERATURE                   0x4120 /**< \brief Too low ambient temperature*/
#define ERROR_TEMPERATURE_SUPPLY_AIR                        0x4130 /**< \brief Temperature supply air*/
#define ERROR_TEMPERATURE_AIR_OUTLET                        0x4140 /**< \brief Temperature air outlet*/
#define ERROR_EXCESS_TEMPERATURE_DEVICE                     0x4210 /**< \brief Excess temperature device*/
#define ERROR_TOO_LOW_TEMPERATURE_DEVICE                    0x4220 /**< \brief Too low temperature device*/
#define ERROR_TEMPERATURE_DRIVE                             0x4300 /**< \brief Temperature drive error*/
#define ERROR_EXCESS_TEMPERATURE_DRIVE                      0x4310 /**< \brief Excess temperature drive error*/
#define ERROR_TOO_LOW_TEMPERATURE_DRIVE                     0x4320 /**< \brief Too low temperature drive error*/
#define ERROR_TEMPERATURE_SUPPLY                            0x4400 /**< \brief Temperature supply error*/
#define ERROR_EXCESS_TEMPERATURE_SUPPLY                     0x4410 /**< \brief Excess temperature supply*/
#define ERROR_TOO_LOW_TEMPERATURE_SUPPLY                    0x4420 /**< \brief Too low temperature supply*/
#define ERROR_SUPPLY_ERROR                                  0x5100 /**< \brief Supply error*/
#define ERROR_SUPPLY_LOW_VOLTAGE                            0x5110 /**< \brief Supply low voltage*/
#define ERROR_U1_SUPPLY_15V                                 0x5111 /**< \brief U1 = supply +15V/-15V*/
#define ERROR_U2_SUPPLY_24_V                                0x5112 /**< \brief U2 = supply +24 V*/
#define ERROR_U3_SUPPLY_5_V                                 0x5113 /**< \brief U3 = supply +5 V*/
#define ERROR_U4_MANUFACTURER_SPECIFIC                      0x5114 /**< \brief U4 = manufacturer-specific error*/
#define ERROR_U5_MANUFACTURER_SPECIFIC                      0x5115 /**< \brief U5 = manufacturer-specific error*/
#define ERROR_U6_MANUFACTURER_SPECIFIC                      0x5116 /**< \brief U6 = manufacturer-specific error*/
#define ERROR_U7_MANUFACTURER_SPECIFIC                      0x5117 /**< \brief U7 = manufacturer-specific error*/
#define ERROR_U8_MANUFACTURER_SPECIFIC                      0x5118 /**< \brief U8 = manufacturer-specific error*/
#define ERROR_U9_MANUFACTURER_SPECIFIC                      0x5119 /**< \brief U9 = manufacturer-specific error*/
#define ERROR_SUPPLY_INTERMEDIATE_CIRCUIT                   0x5120 /**< \brief Supply intermediate circuit*/
//#define ERROR_CONTROL                                     0x5200
#define ERROR_CONTROL_MEASUREMENT_CIRCUIT                   0x5210 /**< \brief Measurement circuit*/
#define ERROR_CONTROL_COMPUTING_CIRCUIT                     0x5220 /**< \brief Computing circuit*/
#define ERROR_OPERATING_UNIT                                0x5300 /**< \brief Operating unit error*/
#define ERROR_POWER_SECTION                                 0x5400 /**< \brief Power section error*/
#define ERROR_OUTPUT_STAGES                                 0x5410 /**< \brief Output stages error*/
#define ERROR_CHOPPER                                       0x5420 /**< \brief Chopper error*/
#define ERROR_INPUT_STAGES                                  0x5430 /**< \brief Input stages error*/
#define ERROR_CONTACTS_ERROR                                0x5440 /**< \brief Contacts error*/
#define ERROR_CONTACT_1_MANUFACTURER_SPECIFIC               0x5441 /**< \brief Contact 1 = manufacturer-specific error*/
#define ERROR_CONTACT_2_MANUFACTURER_SPECIFIC               0x5442 /**< \brief Contact 2 = manufacturer-specific error*/
#define ERROR_CONTACT_3_MANUFACTURER_SPECIFIC               0x5443 /**< \brief Contact 3 = manufacturer-specific error*/
#define ERROR_CONTACT_4_MANUFACTURER_SPECIFIC               0x5444 /**< \brief Contact 4 = manufacturer-specific error*/
#define ERROR_CONTACT_5_MANUFACTURER_SPECIFIC               0x5445 /**< \brief Contact 5 = manufacturer-specific error*/
#define ERROR_FUSES_ERROR                                   0x5450 /**< \brief Fuses error*/
#define ERROR_S1_L1                                         0x5451 /**< \brief S1 = l1 error*/
#define ERROR_S2_L2                                         0x5452 /**< \brief S2 = l2 error*/
#define ERROR_S3_L3                                         0x5453 /**< \brief S3 = l3 error*/
#define ERROR_S4_MANUFACTURER_SPECIFIC                      0x5454 /**< \brief S4 = manufacturer-specific error*/
#define ERROR_S5_MANUFACTURER_SPECIFIC                      0x5455 /**< \brief S5 = manufacturer-specific error*/
#define ERROR_S6_MANUFACTURER_SPECIFIC                      0x5456 /**< \brief S6 = manufacturer-specific error*/
#define ERROR_S7_MANUFACTURER_SPECIFIC                      0x5457 /**< \brief S7 = manufacturer-specific error*/
#define ERROR_S8_MANUFACTURER_SPECIFIC                      0x5458 /**< \brief S8 = manufacturer-specific error*/
#define ERROR_S9_MANUFACTURER_SPECIFIC                      0x5459 /**< \brief S9 = manufacturer-specific error*/
#define ERROR_HARDWARE_MEMORY                               0x5500 /**< \brief Hardware memory error*/
#define ERROR_RAM                                           0x5510 /**< \brief RAM error*/
#define ERROR_ROM_EPROM                                     0x5520 /**< \brief ROM/EPROM error*/
#define ERROR_EEPROM                                        0x5530 /**< \brief EEPROM error*/
#define ERROR_SOFTWARE_RESET_WATCHDOG                       0x6010 /**< \brief Software reset (watchdog)*/
//0x6301_TO_0x630F        ERROR_DATA_RECORD_NO_1_TO_NO_15
#define ERROR_LOSS_OF_PARAMETERS                            0x6310 /**< \brief Loss of parameters*/
#define ERROR_PARAMETER_ERROR                               0x6320 /**< \brief Parameter error*/
#define ERROR_POWER_ERROR                                   0x7100 /**< \brief Power error*/
#define ERROR_BRAKE_CHOPPER                                 0x7110 /**< \brief Brake chopper*/
#define ERROR_FAILURE_BRAKE_CHOPPER                         0x7111 /**< \brief Failure brake chopper*/
#define ERROR_OVER_CURRENT_BRAKE_CHOPPER                    0x7112 /**< \brief Over current brake chopper*/
#define ERROR_PROTECTIVE_CIRCUIT_BRAKE_CHOPPER              0x7113 /**< \brief Protective circuit brake chopper error*/
#define ERROR_MOTOR_ERROR                                   0x7120 /**< \brief Motor error*/
#define ERROR_MOTOR_BLOCKED                                 0x7121 /**< \brief Motor blocked error*/
#define ERROR_MOTOR_ERROR_OR_COMMUTATION_MALFUNC            0x7122 /**< \brief Motor error or commutation malfunc */
#define ERROR_MOTOR_TILTED                                  0x7123 /**< \brief Motor tilted*/
#define ERROR_MEASUREMENT_CIRCUIT                           0x7200 /**< \brief Measurement circuit*/
#define ERROR_SENSOR_ERROR                                  0x7300 /**< \brief Sensor error*/
#define ERROR_TACHO_FAULT                                   0x7301 /**< \brief Tacho fault*/
#define ERROR_TACHO_WRONG_POLARITY                          0x7302 /**< \brief Tacho wrong polarity*/
#define ERROR_RESOLVER_1_FAULT                              0x7303 /**< \brief Resolver 1 fault*/
#define ERROR_RESOLVER_2_FAULT                              0x7304 /**< \brief Resolver 2 fault*/
#define ERROR_INCREMENTAL_SENSOR_1_FAULT                    0x7305 /**< \brief Incremental sensor 1 fault*/
#define ERROR_INCREMENTAL_SENSOR_2_FAULT                    0x7306 /**< \brief Incremental sensor 2 fault*/
#define ERROR_INCREMENTAL_SENSOR_3_FAULT                    0x7307 /**< \brief Incremental sensor 3 fault*/
#define ERROR_SPEED                                         0x7310 /**< \brief Speed error*/
#define ERROR_POSITION                                      0x7320 /**< \brief Position error*/
#define ERROR_COMPUTATION_CIRCUIT                           0x7400 /**< \brief Computation circuit*/
#define ERROR_COMMUNICATION                                 0x7500 /**< \brief Communication error*/
#define ERROR_SERIAL_INTERFACE_NO_1                         0x7510 /**< \brief Serial interface no  1 error*/
#define ERROR_SERIAL_INTERFACE_NO_2                         0x7520 /**< \brief Serial interface no  2 error*/
#define ERROR_DATA_STORAGE_EXTERNAL                         0x7600 /**< \brief Data storage (external) error*/
#define ERROR_TORQUE_CONTROL                                0x8300 /**< \brief Torque control error*/
#define ERROR_EXCESS_TORQUE                                 0x8311 /**< \brief Excess torque error*/
#define ERROR_DIFFICULT_START_UP                            0x8312 /**< \brief Difficult start up error*/
#define ERROR_STANDSTILL_TORQUE                             0x8313 /**< \brief Standstill torque error*/
#define ERROR_INSUFFICIENT_TORQUE                           0x8321 /**< \brief Insufficient torque error*/
#define ERROR_TORQUE_FAULT                                  0x8331 /**< \brief Torque fault*/
#define ERROR_VELOCITY_SPEED_CONTROLLER                     0x8400 /**< \brief Velocity speed controller*/
#define ERROR_POSITION_CONTROLLER                           0x8500 /**< \brief Position controller*/
#define ERROR_POSITIONING_CONTROLLER                        0x8600 /**< \brief Positioning controller*/
#define ERROR_FOLLOWING_ERROR                               0x8611 /**< \brief Following error*/
#define ERROR_REFERENCE_LIMIT                               0x8612 /**< \brief Reference limit*/
#define ERROR_SYNC_CONTROLLER                               0x8700 /**< \brief Sync controller*/
#define ERROR_WINDING_CONTROLLER                            0x8800 /**< \brief Winding controller*/
#define ERROR_PROCESS_DATA_MONITORING                       0x8900 /**< \brief Process data monitoring*/
//#define ERROR_CONTROL                                     0x8A00
#define ERROR_DECELERATION                                  0xF001 /**< \brief Deceleration error*/
#define ERROR_SUB_SYNCHRONOUS_RUN                           0xF002 /**< \brief Sub-synchronous run error*/
#define ERROR_STROKE_OPERATION                              0xF003 /**< \brief Stroke operation error*/
//#define ERROR_CONTROL                                     0xF004
//0xFF00_TO_0xFFFF        MANUFACTURER_SPECIFIC

/*---------------------------------------------
-    CiA402 generic error option code values
        Note: Not all values are valid for each error option code.
        A detailed description of the option code values are listed in the specification IEC 61800-7-200
        0x605B    : action in state transition 8
        0x605C    : action in state transition 5
-----------------------------------------------*/
#define DISABLE_DRIVE                    0 /**< \brief Disable drive (options: 0x605B; 0x605C; 0x605E)*/
#define SLOW_DOWN_RAMP                   1 /**< \brief Slow down ramp (options: 0x605B; 0x605C; 0x605E)*/
#define QUICKSTOP_RAMP                   2 /**< \brief Quick stop ramp (options: 0x605E)*/
#define STOP_ON_CURRENT_LIMIT            3 /**< \brief Stop on current limit (options: 0x605E)*/
#define STOP_ON_VOLTAGE_LIMIT            4 /**< \brief Stop on voltage limit (options: 0x605E)*/


/*---------------------------------------------
-    Specific values for Quick stop option code (object 0x605A) (IEC61800_184e)
        indicated the quick stop function
-----------------------------------------------*/
//-32768 to -1        MANUFACTURER_SPECIFIC
#define SLOWDOWN_RAMP_NO_TRANSIT                5 /**< \brief Slow down on slow down ramp and stay in Quick Stop Active*/
#define QUICKSTOP_RAMP_NO_TRANSIT               6 /**< \brief Slow down on quick stop ramp and stay in Quick Stop Active*/
#define CURRENT_LIMIT_NO_TRANSIT                7 /**< \brief Slow down on current limit and stay in Quick Stop Active*/
#define VOLTAGE_LIMIT_NO_TRANSIT                8 /**< \brief Slow down on voltage limit and stay in Quick Stop Active*/



/***************************************
 CiA402 Object Dictionary indexes
 ***************************************/
#define OBD_CIA402_AXIS_OFFSET                          0x0800

#define OBD_ABORT_CONNECTION_OPTION_CODE_INDEX(x)       (0x6007 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_CONTROLWORD_INDEX(x)                        (0x6040 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_STATUSWORD_INDEX(x)                         (0x6041 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_QUICKSTOP_INDEX(x)                          (0x605A + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_SHUTDOWN_INDEX(x)                           (0x605B + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_DISABLE_OPERATION_INDEX(x)                  (0x605C + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_HALT_OPTION_CODE_INDEX(x)                   (0x605D + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_FAULT_REACTION_INDEX(x)                     (0x605E + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_MODES_OF_OPERATION_INDEX(x)                 (0x6060 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_MODES_OF_OPERATION_DISPLAY_INDEX(x)         (0x6061 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_POSITION_DEMAND_VALUE_INDEX(x)              (0x6062 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_POSITION_ACTUAL_INTERNAL_VALUE_INDEX(x)     (0x6063 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_POSITION_ACTUAL_VALUE_INDEX(x)              (0x6064 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_FOLLOWING_ERROR_WINDOW_INDEX(x)             (0x6065 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_FOLLOWING_ERROR_TIMEOUT_INDEX(x)            (0x6066 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_POSITION_WINDOW_INDEX(x)                    (0x6067 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_POSITION_WINDOW_TIME_INDEX(x)               (0x6068 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_VELOCITY_SENSOR_ACTUAL_VALUE_INDEX(x)       (0x6069 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_DEMAND_VALUE_INDEX(x)              (0x606B + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_ACTUAL_VALUE_INDEX(x)              (0x606C + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_WINDOW_INDEX(x)                    (0x606D + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_WINDOW_TIME_INDEX(x)               (0x606E + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_THRESHOLD_INDEX(x)                 (0x606F + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_THRESHOLD_TIME_INDEX(x)            (0x6070 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_TARGET_TORQUE_INDEX(x)                      (0x6071 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_MAX_TORQUE_INDEX(x)                         (0x6072 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TORQUE_DEMAND_INDEX(x)                      (0x6074 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TORQUE_ACTUAL_VALUE_INDEX(x)                (0x6077 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_CURRENT_ACTUAL_VALUE_INDEX(x)               (0x6078 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TARGET_POSITION_INDEX(x)                    (0x607A + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_POSITION_RANGE_LIMIT_INDEX(x)               (0x607B + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_HOME_OFFSET_INDEX(x)                        (0x607C + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_SW_POSITION_LIMIT_INDEX(x)                  (0x607D + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_POLARITY_INDEX(x)                           (0x607E + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TARGET_VELOCITY_INDEX(x)                    (0x60FF + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_MAX_MOTOR_SPEED_INDEX(x)                    (0x6080 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_PROFILE_VELOCITY_INDEX(x)                   (0x6081 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_PROFILE_ACCELERATION_INDEX(x)               (0x6083 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_PROFILE_DECELERATION_INDEX(x)               (0x6084 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_QUISTOP_DECELERATION_INDEX(x)               (0x6085 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_MOTION_PROFILE_TYPE_INDEX(x)                (0x6086 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_POSITION_ENCODER_RESOLUTION_INDEX(x)        (0x608F + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_ENCONDER_RESOLUTION_INDEX(x)       (0x6090 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_GEAR_RATIO_INDEX(x)                         (0x6091 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_FEED_CONSTANT_INDEX(x)                      (0x6092 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_FACTOR_INDEX(x)                    (0x6096 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_HOMING_METHOD_INDEX(x)                      (0x6098 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_HOMING_SPEEDS_INDEX(x)                      (0x6099 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_HOMING_ACCELERATION_INDEX(x)                (0x609A + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_POSITION_OFFSET_INDEX(x)                    (0x60B0 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_OFFSET_INDEX(x)                    (0x60B1 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TORQUE_OFFSET_INDEX(x)                      (0x60B2 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_TOUCH_PROBE_FUNCTION_INDEX(x)               (0x60B8 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_STATUS_INDEX(x)                 (0x60B9 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_1_POS_EDGE_INDEX(x)             (0x60BA + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_1_NEG_EDGE_INDEX(x)             (0x60BB + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_2_POS_EDGE_INDEX(x)             (0x60BC + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_2_NEG_EDGE_INDEX(x)             (0x60BD + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_INTERPOLATION_TIME_PERIOD_INDEX(x)          (0x60C2 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_MAX_ACCELERATION_INDEX(x)                   (0x60C5 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_MAX_DECELERATION_INDEX(x)                   (0x60C6 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_TOUCH_PROBE_SOURCE(x)                       (0x60D0 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_1_POS_EDGE_CNT_INDEX(x)         (0x60D5 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_1_NEG_EDGE_CNT_INDEX(x)         (0x60D6 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_2_POS_EDGE_CNT_INDEX(x)         (0x60D7 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_2_NEG_EDGE_CNT_INDEX(x)         (0x60D8 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_POSITIVE_TORQUE_LIMIT_VALUE_INDEX(x)        (0x60E0 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_NEGATIVE_TORQUE_LIMIT_VALUE_INDEX(x)        (0x60E1 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_FOLLOWING_ERROR_ACTUAL_VALUE_INDEX(x)       (0x60F4 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TARGET_VELOCITY_INDEX(x)                    (0x60FF + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_SUPPORTED_DRIVE_MODES_INDEX(x)              (0x6502 + x * OBD_CIA402_AXIS_OFFSET)


/**
* @}
*/

#if (defined __cplusplus)
}
#endif

#endif /* __ECAPIDEF_H__ */

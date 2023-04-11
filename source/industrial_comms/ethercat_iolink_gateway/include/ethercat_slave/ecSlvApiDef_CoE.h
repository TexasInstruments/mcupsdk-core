/*!
* \file ecSlvApiDef_CoE.h
*
* \brief
* CoE defines.
*
* \author
* KUNBUS GmbH
*
* \date
* 2022-08-19
*
* \copyright
* Copyright (c) 2022, KUNBUS GmbH<br /><br />
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

#if !(defined __ECSLVAPIDEF_COE_H__)
#define __ECSLVAPIDEF_COE_H__		1



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
#define     DEFTYPE_NULL                                                    0x0000      /**< \brief Null*/
#define     DEFTYPE_BOOLEAN                                                 0x0001      /**< \brief BOOLEAN*/
#define     DEFTYPE_INTEGER8                                                0x0002      /**< \brief INTEGER8*/
#define     DEFTYPE_INTEGER16                                               0x0003      /**< \brief INTEGER16*/
#define     DEFTYPE_INTEGER32                                               0x0004      /**< \brief INTEGER32*/
#define     DEFTYPE_UNSIGNED8                                               0x0005      /**< \brief UNSIGNED8*/
#define     DEFTYPE_UNSIGNED16                                              0x0006      /**< \brief UNSIGNED16*/
#define     DEFTYPE_UNSIGNED32                                              0x0007      /**< \brief UNSIGNED32*/
#define     DEFTYPE_REAL32                                                  0x0008      /**< \brief REAL32*/
#define     DEFTYPE_VISIBLESTRING                                           0x0009      /**< \brief VISIBLE_STRING*/
#define     DEFTYPE_OCTETSTRING                                             0x000A      /**< \brief OCTET_STRING*/
#define     DEFTYPE_UNICODE_STRING                                          0x000B      /**< \brief UNICODE_STRING*/
#define     DEFTYPE_TIME_OF_DAY                                             0x000C      /**< \brief TIME_OF_DAY*/
#define     DEFTYPE_TIME_DIFFERENCE                                         0x000D      /**< \brief TIME_DIFFERENCE*/
#define     DEFTYPE_INTEGER24                                               0x0010      /**< \brief INTEGER24*/
#define     DEFTYPE_REAL64                                                  0x0011      /**< \brief REAL64*/
#define     DEFTYPE_INTEGER40                                               0x0012      /**< \brief INTEGER40*/
#define     DEFTYPE_INTEGER48                                               0x0013      /**< \brief INTEGER48*/
#define     DEFTYPE_INTEGER56                                               0x0014      /**< \brief INTEGER56*/
#define     DEFTYPE_INTEGER64                                               0x0015      /**< \brief INTEGER64*/
#define     DEFTYPE_UNSIGNED24                                              0x0016      /**< \brief UNSIGNED24*/
#define     DEFTYPE_UNSIGNED40                                              0x0018      /**< \brief UNSIGNED40*/
#define     DEFTYPE_UNSIGNED48                                              0x0019      /**< \brief UNSIGNED48*/
#define     DEFTYPE_UNSIGNED56                                              0x001A      /**< \brief UNSIGNED56*/
#define     DEFTYPE_UNSIGNED64                                              0x001B      /**< \brief UNSIGNED64*/
#define     DEFTYPE_GUID                                                    0x001D      /**< \brief DEFTYPE_GUID*/
#define     DEFTYPE_BYTE                                                    0x001E      /**< \brief DEFTYPE_BYTE*/
#define     DEFTYPE_WORD                                                    0x001F      /**< \brief DEFTYPE_WORD*/
#define     DEFTYPE_DWORD                                                   0x0020      /**< \brief DEFTYPE_DWORD*/
#define     DEFTYPE_PDOMAPPING                                              0x0021      /**< \brief PDO_MAPPING*/
#define     DEFTYPE_IDENTITY                                                0x0023      /**< \brief IDENTITY*/
#define     DEFTYPE_COMMAND                                                 0x0025      /**< \brief COMMAND_PAR*/
#define     DEFTYPE_PDOCOMPAR                                               0x0027      /**< \brief PDO_PARAMETER*/
#define     DEFTYPE_ENUM                                                    0x0028      /**< \brief DEFTYPE_ENUM */
#define     DEFTYPE_SMPAR                                                   0x0029      /**< \brief SM_SYNCHRONISATION*/
#define     DEFTYPE_RECORD                                                  0x002A      /**< \brief DEFTYPE_RECORD */
#define     DEFTYPE_BACKUP                                                  0x002B      /**< \brief BACKUP_PARAMETER*/
#define     DEFTYPE_MDP                                                     0x002C      /**< \brief MODULAR_DEVICE_PROFILE*/
#define     DEFTYPE_BITARR8                                                 0x002D      /**< \brief BITARR8*/
#define     DEFTYPE_BITARR16                                                0x002E      /**< \brief BITARR16*/
#define     DEFTYPE_BITARR32                                                0x002F      /**< \brief BITARR32*/
#define     DEFTYPE_BIT1                                                    0x0030      /**< \brief BIT1*/
#define     DEFTYPE_BIT2                                                    0x0031      /**< \brief BIT2*/
#define     DEFTYPE_BIT3                                                    0x0032      /**< \brief BIT3*/
#define     DEFTYPE_BIT4                                                    0x0033      /**< \brief BIT4*/
#define     DEFTYPE_BIT5                                                    0x0034      /**< \brief BIT5*/
#define     DEFTYPE_BIT6                                                    0x0035      /**< \brief BIT6*/
#define     DEFTYPE_BIT7                                                    0x0036      /**< \brief BIT7*/
#define     DEFTYPE_BIT8                                                    0x0037      /**< \brief BIT8*/
#define     DEFTYPE_ARRAY_OF_INT                                            0x0260      /**< \brief DEFTYPE_ARRAY_OF_INT*/
#define     DEFTYPE_ARRAY_OF_SINT                                           0x0261      /**< \brief DEFTYPE_ARRAY_OF_SINT*/
#define     DEFTYPE_ARRAY_OF_DINT                                           0x0262      /**< \brief DEFTYPE_ARRAY_OF_DINT*/
#define     DEFTYPE_ARRAY_OF_UDINT                                          0x0263      /**< \brief DEFTYPE_ARRAY_OF_UDINT*/
#define     DEFTYPE_ERRORHANDLING                                           0x0281      /**< \brief DEFTYPE_ERRORHANDLING*/
#define     DEFTYPE_DIAGHISTORY                                             0x0282      /**< \brief DEFTYPE_DIAGHISTORY*/
#define     DEFTYPE_SYNCSTATUS                                              0x0283      /**< \brief DEFTYPE_SYNCSTATUS*/
#define     DEFTYPE_SYNCSETTINGS                                            0x0284      /**< \brief DEFTYPE_SYNCSETTINGS*/
#define     DEFTYPE_FSOEFRAME                                               0x0285      /**< \brief DEFTYPE_FSOEFRAME*/
#define     DEFTYPE_FSOECOMMPAR                                             0x0286      /**< \brief DEFTYPE_FSOECOMMPAR*/
/** @}*/

/**
 * \addtogroup SDOAbortIndex SDO Abort Codes Identifier
 * @{
 * Abort error code index values. Use for mailbox callback return values.
 */
#define     ABORTIDX_TOGGLE_BIT_NOT_CHANGED                                 0x01        /**< \brief Index of "Toggle bit not changed"*/
#define     ABORTIDX_SDO_PROTOCOL_TIMEOUT                                   0x02        /**< \brief Index of "SDO timeout"*/
#define     ABORTIDX_COMMAND_SPECIFIER_UNKNOWN                              0x03        /**< \brief Index of "Command specifier unknown"*/
#define     ABORTIDX_OUT_OF_MEMORY                                          0x04        /**< \brief Index of "Out of memory"*/
#define     ABORTIDX_UNSUPPORTED_ACCESS                                     0x05        /**< \brief Index of "Unsupported Access"*/
#define     ABORTIDX_WRITE_ONLY_ENTRY                                       0x06        /**< \brief Index of "Write only entry"*/
#define     ABORTIDX_READ_ONLY_ENTRY                                        0x07        /**< \brief Index of "Read only entry"*/
#define     ABORTIDX_OBJECT_NOT_EXISTING                                    0x08        /**< \brief Index of "Object not existing"*/
#define     ABORTIDX_OBJECT_CANT_BE_PDOMAPPED                               0x09        /**< \brief Index of "Object can not be mapped to PDO"*/
#define     ABORTIDX_MAPPED_OBJECTS_EXCEED_PDO                              0x0A        /**< \brief Index of "Mapped Object exceeds PDO"*/
#define     ABORTIDX_PARAM_IS_INCOMPATIBLE                                  0x0B        /**< \brief Index of "Parameter is incompatible"*/
#define     ABORTIDX_INTERNAL_DEVICE_INCOMPATIBILITY                        0x0C        /**< \brief Index of "Device incompatibility"*/
#define     ABORTIDX_HARDWARE_ERROR                                         0x0D        /**< \brief Index of "Hardware error"*/
#define     ABORTIDX_PARAM_LENGTH_ERROR                                     0x0E        /**< \brief Index of "Parameter length error"*/
#define     ABORTIDX_PARAM_LENGTH_TOO_LONG                                  0x0F        /**< \brief Index of "Parameter is too long"*/
#define     ABORTIDX_PARAM_LENGTH_TOO_SHORT                                 0x10        /**< \brief Index of "Parameter is too short"*/
#define     ABORTIDX_SUBINDEX_NOT_EXISTING                                  0x11        /**< \brief Index of "Subindex (Entry) not exists"*/
#define     ABORTIDX_VALUE_EXCEEDED                                         0x12        /**< \brief Index of "Value exceeds"*/
#define     ABORTIDX_VALUE_TOO_GREAT                                        0x13        /**< \brief Index of "Value is too great"*/
#define     ABORTIDX_VALUE_TOO_SMALL                                        0x14        /**< \brief Index of "Value is too small"*/
#define     ABORTIDX_MODULE_ID_LIST_NOT_MATCH                               0x15        /**< \brief  Index of "Unequal Module Id list"*/
#define     ABORTIDX_MAX_VALUE_IS_LESS_THAN_MIN_VALUE                       0x16        /**< \brief Index of "Value is less than minimum value"*/
#define     ABORTIDX_GENERAL_ERROR                                          0x17        /**< \brief Index of "General error"*/
#define     ABORTIDX_DATA_CANNOT_BE_READ_OR_STORED                          0x18        /**< \brief Index of "Data can not be read or written"*/
#define     ABORTIDX_DATA_CANNOT_BE_ACCESSED_BECAUSE_OF_LOCAL_CONTROL       0x19        /**< \brief Index of "Data can not be accessed because of local control"*/
#define     ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED            0x1A        /**< \brief Index of "Data can not be read or written in the current state"*/
#define     ABORTIDX_NO_OBJECT_DICTIONARY_IS_PRESENT                        0x1B        /**< \brief Index of "Object is not in the object dictionary"*/
#define     ABORTIDX_ENTRY_CANT_BE_WRITTEN_SI0_NOT_0                        0x1C        /**< \brief Index of "Entry can not be written because Subindex0 is not 0"*/
#define     ABORTIDX_COMPLETE_ACCESS_NOT_SUPPORTED                          0x1D        /**< \brief The object can not be accessed via complete access*/
#define     ABORTIDX_WORKING                                                0xFF        /**< \brief Index of application is handling the SDO request*/
/** @}*/




/**
* \addtogroup SdoAccess SDO Access Rigths
* @{
*/
#define    ACCESS_READWRITE                                                 0x003F      /**< \brief Read/write in all states*/
#define    ACCESS_READ                                                      0x0007      /**< \brief Read only in all states*/
#define    ACCESS_READ_PREOP                                                0x0001      /**< \brief Read only in PreOP*/
#define    ACCESS_READ_SAFEOP                                               0x0002      /**< \brief Read only in SafeOP*/
#define    ACCESS_READ_OP                                                   0x0004      /**< \brief Read only in OP*/
#define    ACCESS_WRITE                                                     0x0038      /**< \brief Write only in all states*/
#define    ACCESS_WRITE_PREOP                                               0x0008      /**< \brief Write only in PreOP*/
#define    ACCESS_WRITE_SAFEOP                                              0x0010      /**< \brief Write only in SafeOP*/
#define    ACCESS_WRITE_OP                                                  0x0020      /**< \brief Write only in OP*/
#define    OBJACCESS_NOPDOMAPPING                                           0x0000      /**< \brief Not PDO mappable*/
#define    OBJACCESS_RXPDOMAPPING                                           0x0040      /**< \brief Mappable in RxPDOs*/
#define    OBJACCESS_TXPDOMAPPING                                           0x0080      /**< \brief Mappable in TxPDOs*/
#define    OBJACCESS_BACKUP                                                 0x0100      /**< \brief Backup entry*/
#define    OBJACCESS_SETTINGS                                               0x0200      /**< \brief Setting Entry*/
#define    OBJACCESS_SAFEINPUTS                                             0x0400      /**< \brief Safe input*/
#define    OBJACCESS_SAFEOUTPUTS                                            0x0800      /**< \brief Safe output*/
#define    OBJACCESS_SAFEPARAMETER                                          0x1000      /**< \brief Safe parameter*/
/** @}*/


/**
* \addtogroup SDOAbort SDO Abort Codes
* @{
*/
#define     ABORT_NOERROR                                                   0x00000000  /**< \brief No SDO error*/
#define     ABORT_TOGGLE_BIT_NOT_CHANGED                                    0x05030000  /**< \brief Toggle bit not changed*/
#define     ABORT_SDO_PROTOCOL_TIMEOUT                                      0x05040000  /**< \brief SDO timeout*/
#define     ABORT_COMMAND_SPECIFIER_UNKNOWN                                 0x05040001  /**< \brief Command specifier unknown*/
#define     ABORT_OUT_OF_MEMORY                                             0x05040005  /**< \brief Out of memory*/
#define     ABORT_UNSUPPORTED_ACCESS                                        0x06010000  /**< \brief Unsupported Access*/
#define     ABORT_WRITE_ONLY_ENTRY                                          0x06010001  /**< \brief Write only entry*/
#define     ABORT_READ_ONLY_ENTRY                                           0x06010002  /**< \brief Read only entry*/
#define     ABORT_ENTRY_CANT_BE_WRITTEN_SI0_NOT_0                           0x06010003  /**< \brief Entry can not be written because Subindex0 is not 0*/
#define     ABORT_COMPLETE_ACCESS_NOT_SUPPORTED                             0x06010004  /**< \brief The object can not be accessed via complete access*/
#define     ABORT_OBJECT_NOT_EXISTING                                       0x06020000  /**< \brief Object not existing*/
#define     ABORT_OBJECT_CANT_BE_PDOMAPPED                                  0x06040041  /**< \brief Object can not be mapped to PDO*/
#define     ABORT_MAPPED_OBJECTS_EXCEED_PDO                                 0x06040042  /**< \brief Mapped Object exceeds PDO*/
#define     ABORT_PARAM_IS_INCOMPATIBLE                                     0x06040043  /**< \brief Parameter is incompatible*/
#define     ABORT_INTERNAL_DEVICE_INCOMPATIBILITY                           0x06040047  /**< \brief Device incompatibility*/
#define     ABORT_HARDWARE_ERROR                                            0x06060000  /**< \brief Hardware error*/
#define     ABORT_PARAM_LENGTH_ERROR                                        0x06070010  /**< \brief Parameter length error*/
#define     ABORT_PARAM_LENGTH_TOO_LONG                                     0x06070012  /**< \brief Parameter is too long*/
#define     ABORT_PARAM_LENGTH_TOO_SHORT                                    0x06070013  /**< \brief Parameter is too short*/
#define     ABORT_SUBINDEX_NOT_EXISTING                                     0x06090011  /**< \brief Subindex (Entry) not exists*/
#define     ABORT_VALUE_EXCEEDED                                            0x06090030  /**< \brief Value exceeds*/
#define     ABORT_VALUE_TOO_GREAT                                           0x06090031  /**< \brief Value is too great*/
#define     ABORT_VALUE_TOO_SMALL                                           0x06090032  /**< \brief Value is too small*/
#define     ABORT_MODULE_ID_LIST_NOT_MATCH                                  0x06090033  /**< \brief Detected Module Ident List (0xF030) and Configured Module Ident list (0xF050) does not match*/
#define     ABORT_MAX_VALUE_IS_LESS_THAN_MIN_VALUE                          0x06090036  /**< \brief Value is less than minimum value*/
#define     ABORT_GENERAL_ERROR                                             0x08000000  /**< \brief General error*/
#define     ABORT_DATA_CANNOT_BE_READ_OR_STORED                             0x08000020  /**< \brief Data can not be read or written*/
#define     ABORT_DATA_CANNOT_BE_READ_OR_STORED_BECAUSE_OF_LOCAL_CONTROL    0x08000021  /**< \brief Data can not be accessed because of local control*/
#define     ABORT_DATA_CANNOT_BE_READ_OR_STORED_IN_THIS_STATE               0x08000022  /**< \brief Data can not be read or written in the current state*/
#define     ABORT_NO_OBJECT_DICTIONARY_IS_PRESENT                           0x08000023  /**< \brief Object is not in the object dictionary*/
/** @}*/

/**
* \addtogroup ObjectTypes CoE Object types
* @{
*/
#define    OBJCODE_VAR                                                      0x07        /**< \brief Object code VARIABLE*/
#define    OBJCODE_ARR                                                      0x08        /**< \brief Object code ARRAY*/
#define    OBJCODE_REC                                                      0x09        /**< \brief Object code RECORD*/
/** @}*/


/**
* @}
*/

#if (defined __cplusplus)
}
#endif

#endif /* __ECSLVAPIDEF_COE_H__ */

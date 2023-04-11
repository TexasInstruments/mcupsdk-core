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

#ifndef INC_PROT__IOL_TYPES_H__
#define INC_PROT__IOL_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IOL_ONLY_TYPE_INCLUDE
#include "IOL_Definition.h"
#include "IOL_Port_Types.h"
#endif

#ifndef IOL_ENUM_DECL
#define IOL_ENUM_DECL enum
#endif

#ifndef UNIT_TESTS
#define STATIC static
#else 
#define STATIC
#endif

/**
\addtogroup group_iol_types IO-Link Types
\{

\brief IO-Link definitions for Master and Device.

*/

/**
\brief This enumeration indicates the requested operation mode.
*/
typedef IOL_ENUM_DECL IOL_ETransferMode
{
    /** \brief C/Q line in high impedance. */
    IOL_eTransferMode_INACTIVE = 0,
    /** \brief C/Q line in COM1 mode. */
    IOL_eTransferMode_COM1,
    /** \brief C/Q line in COM2 mode. */
    IOL_eTransferMode_COM2,
    /** \brief C/Q line in COM3 mode. */
    IOL_eTransferMode_COM3,
    /** \brief C/Q line in digital input mode. */
    IOL_eTransferMode_DI,
    /** \brief C/Q line in digital output mode. */
    IOL_eTransferMode_DO,
} IOL_ETransferMode;


/**
\brief This enumeration indicates the number of OD bytes in M-sequence.
*/
typedef IOL_ENUM_DECL IOL_EODLen
{
    IOL_eODLen_1_Byte = 0,
    IOL_eODLen_2_Byte,
    IOL_eODLen_8_Byte,
    IOL_eODLen_32_Byte
} IOL_EODLen;


/**
\brief This enumeration contains supplementary information on the transfer.
status.
*/
typedef IOL_ENUM_DECL IOL_ETransferStatus
{
    /** \brief No error occurred. */
    IOL_eTransferStatus_OK,
    /** \brief UART detected a parity error. */
    IOL_eTransferStatus_PARITY_ERROR,
    /** \brief Invalid UART stop bit detected. */
    IOL_eTransferStatus_FRAMING_ERROR,
    /** \brief Octet collision within the UART. */
    IOL_eTransferStatus_OVERRUN
} IOL_ETransferStatus;

/**
\brief This enumeration contains error information.
*/
typedef IOL_ENUM_DECL IOL_EErrorInfo
{
    /** \brief No error occurred. */
    IOL_eErrorInfo_NONE = 0,
    /** \brief No communication available. */
    IOL_eErrorInfo_NO_COMM,
    /** \brief Service unavailable within current state. */
    IOL_eErrorInfo_STATE_CONFLICT,
    /** \brief Consistency of parameter set violated. */
    IOL_eErrorInfo_PARAMETER_CONFLICT,
    /** \brief The requested combination of cycle times for the activated ports
    is not possible. */
    IOL_eErrorInfo_TIMING_CONFLICT,
    /** \brief DL did not provide Process Data. */
    IOL_eErrorInfo_NO_DATA,
    /** \brief Parameter is out of range. */
    IOL_eErrorInfo_VALUE_OUT_OF_RANGE,
    /** \brief ISDU request timed out. */
    IOL_eErrorInfo_ISDU_TIMEOUT,
    /** \brief ISDU not supported by Device. */
    IOL_eErrorInfo_ISDU_NOT_SUPPORTED,
    /** \brief Timeout. */
    IOL_eErrorInfo_TIMEOUT,
} IOL_EErrorInfo;

/**
\brief This enumeration indicates the communication channel for the access to
the user data.
*/
typedef IOL_ENUM_DECL IOL_EComChannel
{
    IOL_eComChannel_PROCESS = 0x00,
    IOL_eComChannel_PAGE = 0x20,
    IOL_eComChannel_DIAGNOSIS = 0x40,
    IOL_eComChannel_ISDU = 0x60
} IOL_EComChannel;

/**
\brief This enumeration contains ISDU transport direction.
*/
typedef IOL_ENUM_DECL IOL_EDirection
{
    /** \brief Write operation. */
    IOL_eDirection_WRITE = 0x00,
    /** \brief Read operation. */
    IOL_eDirection_READ = 0x80
} IOL_EDirection;

/**
\brief This enumeration indicates the requested mode of the Master's DL on an
individual port.
*/
typedef IOL_ENUM_DECL IOL_EDLMode
{
    /** \brief Handler shall change to the INACTIVE state. */
    IOL_eDLMode_INACTIVE = 0,
    /** \brief Handler shall change to the STARTUP state. */
    IOL_eDLMode_STARTUP,
    /** \brief Handler shall change to the PREOPERATE state. */
    IOL_eDLMode_PREOPERATE,
    /** \brief Handler shall change to the OPERATE state. */
    IOL_eDLMode_OPERATE,
    /** \brief Handler shall change to the INACTIVE after unpairing state. */
    IOL_eDLMode_UNPAIRING,
} IOL_EDLMode;

/**
\brief This enumeration indicates the status of the DL-mode handler.
*/
typedef IOL_ENUM_DECL IOL_EDLRealMode
{
    /** \brief Handler changed to the INACTIVE state. */
    IOL_eDLRealMode_INACTIVE = 0,
    /** \brief COM1 mode established. */
    IOL_eDLRealMode_COM1,
    /** \brief COM2 mode established. */
    IOL_eDLRealMode_COM2,
    /** \brief COM3 mode established. */
    IOL_eDLRealMode_COM3,
    /** \brief Lost communication. */
    IOL_eDLRealMode_COMLOST,
    /** \brief Handler changed to the ESTABCOM state. */
    IOL_eDLRealMode_ESTABCOM,
    /** \brief Handler changed to the ACTIVE state. */
    IOL_eDLRealMode_ACTIVE,
    /** \brief Handler changed to the STARTUP state. */
    IOL_eDLRealMode_STARTUP,
    /** \brief Handler changed to the PREOPERATE state. */
    IOL_eDLRealMode_PREOPERATE,
    /** \brief Handler changed to the OPERATE state. */
    IOL_eDLRealMode_OPERATE,
    /** \brief Invalid value. */
    IOL_eDLRealMode_INVALID,
} IOL_EDLRealMode;

/**
\brief This enumeration contains the M-sequence types.

TYPE_1_1 forces interleave mode of process and On-request Data transmission,
see 7.3.4.2 of \ref section_iol_spec_1_1.
*/
typedef IOL_ENUM_DECL IOL_EMSequence
{
    IOL_eMSequence_TYPE_0 = 0,
    IOL_eMSequence_TYPE_1 = 0x40,
    IOL_eMSequence_TYPE_2 = 0x80,
    IOL_eMSequence_TYPE_MASK = 0xC0
} IOL_EMSequence;

/**
\brief This enumeration indicates the qualifier status of the Process Data (PD).
*/
typedef IOL_ENUM_DECL IOL_EPDInStatus
{
    /** \brief Input Process Data is valid; see 7.2.2.5, 8.2.2.12. */
    IOL_ePDInStatus_VALID = 0,
    /** \brief Input Process Data is invalid. */
    IOL_ePDInStatus_INVALID = 0x40,
} IOL_EPDInStatus;

/**
\brief This enumeration indicates the qualifier status of the Process Data (PD).
*/
typedef IOL_ENUM_DECL IOL_EPDOutStatus
{
    /** \brief Output Process Data is valid; see 7.3.7.1. */
    IOL_ePDOutStatus_PDOUTVALID = 0,
    /** \brief Output Process Data is invalid or missing. */
    IOL_ePDOutStatus_PDOUTINVALID,
    /** \brief Real time fault happened. max retry limit reached. */
    IOL_ePDOutStatus_RTFAULT,
} IOL_EPDOutStatus;

/**
\brief This enumeration contains the exception indication of the message
handler.
*/
typedef IOL_ENUM_DECL IOL_EMHInfo
{
    /** \brief Lost communication. */
    IOL_eMHInfo_COMLOST = 0,
    /** \brief Unexpected M-sequence type detected. */
    IOL_eMHInfo_ILLEGAL_MESSAGETYPE,
    /** \brief Checksum error detected. */
    IOL_eMHInfo_CHECKSUM_MISMATCH
} IOL_EMHInfo;

/**
\brief This enumeration indicates the requested operational mode of the port.
*/
typedef IOL_ENUM_DECL IOL_ETargetMode
{
    /** \brief Communication disabled, no DI, no DO. */
    IOL_eTargetMode_INACTIVE = 0,
    /** \brief Port in digital input mode (SIO). */
    IOL_eTargetMode_DI,
    /** \brief Port in digital output mode (SIO). */
    IOL_eTargetMode_DO,
    /** \brief Device communicating in mode CFGCOM after successful inspection (FIXEDMODE). */
    IOL_eTargetMode_CFGCOM,
    /** \brief Device communicating in mode AUTOCOM without inspection (SCANMODE). */
    IOL_eTargetMode_AUTOCOM,
    /** \brief Port in COM1 mode. */
    IOL_eTargetMode_COM1,
    /** \brief Port in COM2 mode. */
    IOL_eTargetMode_COM2,
    /** \brief Port in COM3 mode. */
    IOL_eTargetMode_COM3,
    /** \brief Port in digital input mode at C/Q and I/Q. */
    IOL_eTargetMode_OSSDE,
    /** \brief Device communicating safety Process Data. */
    IOL_eTargetMode_SAFETYCOM,
    /** \brief Device communicating mixed safety and non safety Process Data. */
    IOL_eTargetMode_MIXEDSAFETYCOM
} IOL_ETargetMode;

#if !defined(IOLM_WIRELESS)
/**
\brief This enumeration indicates the requested IQ mode of the port.
*/
typedef IOL_ENUM_DECL IOL_EIQMode
{
    /** \brief IQ disabled. */
    IOL_eIQMode_INACTIVE = 0,
    /** \brief IQ is in input mode. */
    IOL_eIQMode_IN,
    /** \brief IQ is in output mode. */
    IOL_eIQMode_OUT,
} IOL_EIQMode;
#endif

/**
\brief This enumeration indicates the transmission rate.
*/
typedef IOL_ENUM_DECL IOL_EBaudrate
{
    /** \brief Master accepts transmission rate found during
    #IOL_eDLRealMode_ESTABCOM "ESTABLISHCOM". */
    IOL_eBaudrate_AUTO = 0,
    /** \brief Transmission rate of COM1 (4,8 kbit/s). */
    IOL_eBaudrate_COM1,
    /** \brief Transmission rate of COM2 (38,4 kbit/s). */
    IOL_eBaudrate_COM2,
    /** \brief Transmission rate of COM3 (230,4 kbit/s). */
    IOL_eBaudrate_COM3
} IOL_EBaudrate;

/**
\brief This enumeration indicates changes or faults of the local communication
mode.
*/
typedef IOL_ENUM_DECL IOL_EPortMode
{
    /** \brief Communication disabled, no DI, no DO. */
    IOL_ePortMode_INACTIVE = 0,
    /** \brief Port in digital input mode (SIO). */
    IOL_ePortMode_DI,
    /** \brief Port in digital output mode (SIO). */
    IOL_ePortMode_DO,
    /** \brief Communication established and inspection successful. */
    IOL_ePortMode_COMREADY,
    /** \brief Data Storage finished and ready for operate. */
    IOL_ePortMode_READY_TO_OPERATE,
    /** \brief Port is ready to exchange Process Data. */
    IOL_ePortMode_SM_OPERATE,
    /** \brief Communication failed, new wake-up procedure required. */
    IOL_ePortMode_COMLOST,
    /** \brief Incompatible protocol revision. */
    IOL_ePortMode_REVISION_FAULT,
    /** \brief Incompatible Device or Legacy-Device according to the
    InspectionLevel. */
    IOL_ePortMode_COMP_FAULT,
    /** \brief Mismatching Serial Number according to the InspectionLevel. */
    IOL_ePortMode_SERNUM_FAULT,
    /** \brief Data Storage fault. */
    IOL_ePortMode_DS_FAULT,
    /** \brief Cycle Time fault. */
    IOL_ePortMode_CYCTIME_FAULT,
    /** \brief Port in Power-Off mode. */
    IOL_ePortMode_PORTPOWEROFF,
#if (defined(IOLD_WIRELESS) || defined(IOLM_WIRELESS))
    /** \brief W-Device has been paired. */
    IOL_ePortMode_PAIRING_SUCCESS,
    /** \brief W-Device hasn‘t been paired within the given timeout. */
    IOL_ePortMode_PAIRING_TIMEOUT,
    /** \brief W-Device has different SlotType as requested. */
    IOL_ePortMode_PAIRING_WRONG_SLOTTYPE,
#endif
#ifdef IOL_SAFETY
    /** \brief Port in OSSDe mode. */
    IOL_ePortMode_OSSDE,
    /** \brief Port in Authentication mode. */
    IOL_ePortMode_AUTHENTICATION
#endif

} IOL_EPortMode;

/**
\brief This enumeration indicates changes of communication states to the Device
application.
*/
typedef IOL_ENUM_DECL IOL_EDeviceMode
{
    /** \brief Handler changed to the INACTIVE state. */
    IOL_eDeviceMode_INACTIVE = 0,
    /** \brief Device changed to waiting for configuration. */
    IOL_eDeviceMode_IDLE,
    /** \brief Device changed to the mode defined in service "SM_SetDeviceCom". */
    IOL_eDeviceMode_SIO,
    /** \brief Device changed to the SM mode "SM_ComEstablish". */
    IOL_eDeviceMode_ESTABCOM,
#if 0
    /** \brief Device changed to the COM1 mode. */
    IOL_eDeviceMode_COM1,
    /** \brief Device changed to the COM2 mode. */
    IOL_eDeviceMode_COM2,
    /** \brief Device changed to the COM3 mode. */
    IOL_eDeviceMode_COM3,
#else 
    /** \brief Device changed to the configured COM mode. */
    IOL_eDeviceMode_COMx,
#endif
    /** \brief Device changed to the STARTUP mode. */
    IOL_eDeviceMode_STARTUP,
    /** \brief Device changed to the SM mode "SM_IdentStartup". */
    IOL_eDeviceMode_IDENT_STARTUP,
    /** \brief Device changed to the SM mode "SM_IdentCheck". */
    IOL_eDeviceMode_IDENT_CHANGE,
    /** \brief Device changed to the PREOPERATE mode. */
    IOL_eDeviceMode_PREOPERATE,
    /** \brief Device changed to the OPERATE mode. */
    IOL_eDeviceMode_OPERATE,
    /** \brief Change to button pairing state. */
    IOL_eDeviceMode_PAIRING,
    /** \brief Pairing failed by timeout. */
    IOL_eDeviceMode_TIMEOUT,
    /** \brief Temporary pairing successful. */
    IOL_eDeviceMode_TEMPORARY,
    /** \brief Permanent pairing successful. */
    IOL_eDeviceMode_PERMANENT,
    /** \brief Permanent pairing successful. */
    IOL_eDeviceMode_PAIRED,
    /** \brief Permanent pairing successful. */
    IOL_eDeviceMode_UNPAIRED,
    /** \brief Try to do unpairing by Device. */
    IOL_eDeviceMode_UNPAIRING,
} IOL_EDeviceMode;

/**
\brief This enumeration defines the I-Services of the ISDU.
*/
typedef IOL_ENUM_DECL IOL_EIService
{
    IOL_eIService_NO_SERVICE = 0,
    IOL_eIService_WR_REQ_8B_IDX,
    IOL_eIService_WR_REQ_8B_IDX_SIDX,
    IOL_eIService_WR_REQ_16B_IDX_SIDX,
    IOL_eIService_WR_RES_NOK,
    IOL_eIService_WR_RES_OK,
    IOL_eIService_RESERVED_1,
    IOL_eIService_RESERVED_2,
    IOL_eIService_RESERVED_3,
    IOL_eIService_RD_REQ_8B_IDX,
    IOL_eIService_RD_REQ_8B_IDX_SIDX,
    IOL_eIService_RD_REQ_16B_IDX_SIDX,
    IOL_eIService_RD_RES_NOK,
    IOL_eIService_RD_RES_OK,
    IOL_eIService_RESERVED_4,
    IOL_eIService_RESERVED_5
} IOL_EIService;


/**
\brief This enumeration is a index list of the Direct Parameter Page 1.
*/
typedef IOL_ENUM_DECL IOL_EDirectParam
{
    IOL_eDirectParam_MasterCommand = 0,
    IOL_eDirectParam_MasterCycleTime,
    IOL_eDirectParam_MinCycleTime,
    IOL_eDirectParam_MSequenceCapability,
    IOL_eDirectParam_RevisionID,
    IOL_eDirectParam_ProcessDataIn,
    IOL_eDirectParam_ProcessDataOut,
    IOL_eDirectParam_VendorID1,
    IOL_eDirectParam_VendorID2,
    IOL_eDirectParam_DeviceID1,
    IOL_eDirectParam_DeviceID2,
    IOL_eDirectParam_DeviceID3,
    IOL_eDirectParam_FunctionID1,
    IOL_eDirectParam_FunctionID2,
    IOL_eDirectParam_Reserved,
    IOL_eDirectParam_SystemCommand,
    IOL_eDirectParam_MAX,
} IOL_EDirectParam;

/**
\brief This enumeration indicates the Master Commands.
*/
typedef IOL_ENUM_DECL IOL_EMasterCommand
{
    IOL_eMasterCommand_Fallback = 0x5A,
    IOL_eMasterCommand_Inactive = 0x5C,
    IOL_eMasterCommand_PreDLink = 0x5D,
    IOL_eMasterCommand_FullDLink = 0x5E,
    IOL_eMasterCommand_Unpairing = 0x5F,
    IOL_eMasterCommand_MasterIdent = 0x95,
    IOL_eMasterCommand_DeviceIdent = 0x96,
    IOL_eMasterCommand_DeviceStartup = 0x97,
    IOL_eMasterCommand_ProcessDataOutputOperate = 0x98,
    IOL_eMasterCommand_DeviceOperate = 0x99,
    IOL_eMasterCommand_DevicePreoperate = 0x9A,
    IOL_eMasterCommand_Jump_0 = 0xF0,
    IOL_eMasterCommand_Jump_1 = 0xF1,
    IOL_eMasterCommand_Jump_2 = 0xF2,
    IOL_eMasterCommand_Jump_3 = 0xF3,
    IOL_eMasterCommand_Jump_4 = 0xF4,
    IOL_eMasterCommand_Jump_5 = 0xF5,
    IOL_eMasterCommand_Jump_6 = 0xF6,
    IOL_eMasterCommand_Jump_7 = 0xF7,
    IOL_eMasterCommand_Jump_8 = 0xF8,
    IOL_eMasterCommand_Jump_9 = 0xF9,
    IOL_eMasterCommand_Jump_10 = 0xFA,
    IOL_eMasterCommand_Jump_11 = 0xFB,
    IOL_eMasterCommand_Jump_12 = 0xFC,
    IOL_eMasterCommand_Jump_13 = 0xFD,
    IOL_eMasterCommand_Jump_14 = 0xFE,
    IOL_eMasterCommand_WakeUp = 0xFF,
} IOL_EMasterCommand;


/**
\brief This enumeration indicates the predefined index values for ISDU.
*/
typedef IOL_ENUM_DECL IOL_EISDUIndex
{
    IOL_eISDUIndex_DirectParam1 = 0,
    IOL_eISDUIndex_DirectParam2 = 1,
    IOL_eISDUIndex_SystemCommand = 2,
    IOL_eISDUIndex_DataStorage = 3,

    IOL_eISDUIndex_DeviceAccessLocks = 12,
    IOL_eISDUIndex_ProfileCharacteristic = 13,
    IOL_eISDUIndex_PDInputDescriptor = 14,
    IOL_eISDUIndex_PDOutputDescriptor = 15,
    IOL_eISDUIndex_VendorName = 16,
    IOL_eISDUIndex_VendorText = 17,
    IOL_eISDUIndex_ProductName = 18,
    IOL_eISDUIndex_ProductID = 19,
    IOL_eISDUIndex_ProductText = 20,
    IOL_eISDUIndex_SerialNumber = 21,
    IOL_eISDUIndex_HardwareRevision = 22,
    IOL_eISDUIndex_FirmwareRevision = 23,
    IOL_eISDUIndex_AppSpecificTag = 24,
    IOL_eISDUIndex_FunctionTag = 25,
    IOL_eISDUIndex_LocationTag = 26,
    
    IOL_eISDUIndex_ErrorCount = 32,
    IOL_eISDUIndex_DeviceStatus = 36,
    IOL_eISDUIndex_DetailedDeviceStatus = 37,
    IOL_eISDUIndex_ProcessDataInput      = 40,
    IOL_eISDUIndex_ProcessDataOutput     = 41,
    
    IOL_eISDUIndex_WirelessSystemMgmnt = 0x5001,
    IOL_eISDUIndex_WirelessSystemCfg = 0x5002,
    IOL_eISDUIndex_LinkQuality = 0x5003,
    IOL_eISDUIndex_WBridgeInfo = 0x5004,
    
    IOL_eISDUIndex_WirelessRadioInfo = 0x5005,
    IOL_eISDUIndex_AdaptiveHopTable = 0x5006,
    IOL_eISDUIndex_WCycleTime = 0x5007,
}IOL_EISDUIndex;

/**
\brief This enumeration indicates the permissible ISDU ErrorTypes resulting
from the Device application.
*/
typedef IOL_ENUM_DECL IOL_EErrorType
{
    /** \brief No error. */
    IOL_eErrorType_NONE = 0,

    /** \brief Device application error – no details.

    This ErrorType shall be used if the requested service has been refused by
    the Device application and no detailed information of the incident is
    available. */
    IOL_eErrorType_APP_DEV = 0x8000,
    /** \brief Index not available.

    This ErrorType shall be used whenever a read or write access occurs to a
    not existing Index. */
    IOL_eErrorType_IDX_NOTAVAIL = 0x8011,
    /** \brief SubIndex not available.

    This ErrorType shall be used whenever a read or write access occurs to a
    not existing SubIndex. */
    IOL_eErrorType_SUBIDX_NOTAVAIL = 0x8012,
    /** \brief Service temporarily not available

    This ErrorType shall be used if a parameter is not accessible for a read or
    write service due to the current state of the Device application. */
    IOL_eErrorType_SERV_NOTAVAIL = 0x8020,
    /** \brief Service temporarily not available – local control.

    This ErrorType shall be used if a parameter is not accessible for a read or
    write service due to an ongoing local operation at the Device (for example
    operation or parameterization via an on-board Device control panel). */
    IOL_eErrorType_SERV_NOTAVAIL_LOCCTRL = 0x8021,
    /** \brief Service temporarily not available – Device control.

    This ErrorType shall be used if a read or write service is not accessible
    due to a remote triggered state of the Device application (for example
    parameterization during a remote triggered teach-in operation or
    calibration). */
    IOL_eErrorType_SERV_NOTAVAIL_DEVCTRL = 0x8022,
    /** \brief Access denied.

    This ErrorType shall be used if a write service tries to access a read-only
    parameter. */
    IOL_eErrorType_IDX_NOT_WRITEABLE = 0x8023,
    /** \brief Parameter value out of range.

    This ErrorType shall be used for a write service to a parameter outside its
    permitted range of values.
    Version 1.1.2 – 232 – IO-Link Interface and System © IO-Link. */
    IOL_eErrorType_PAR_VALOUTOFRNG = 0x8030,
    /** \brief Parameter value above limit.

    This ErrorType shall be used for a write service to a parameter above its
    specified value range. */
    IOL_eErrorType_PAR_VALGTLIM = 0x8031,
    /** \brief Parameter value below limit.

    This ErrorType shall be used for a write service to a parameter below its
    specified value range. */
    IOL_eErrorType_PAR_VALLTLIM = 0x8032,
    /** \brief Parameter length overrun.

    This ErrorType shall be used when the content of a write service to a
    parameter is greater than the parameter specified length. This ErrorType
    shall also be used, if a data object is too large to be processed by the
    Device application (for example ISDU buffer restriction). */
    IOL_eErrorType_VAL_LENOVRRUN = 0x8033,
    /** \brief Parameter length underrun.

    This ErrorType shall be used when the content of a write service to a
    parameter is less than the parameter specified length (for example write
    access of an Unsigned16 value to an Unsigned32 parameter). */
    IOL_eErrorType_VAL_LENUNDRUN = 0x8034,
    /** \brief Function not available.

    This ErrorType shall be used for a write service with a command value not
    supported by the Device application (for example a system command with a
    value not implemented). */
    IOL_eErrorType_FUNC_NOTAVAIL = 0x8035,
    /** \brief Function temporarily unavailable.

    This ErrorType shall be used for a write service with a command value
    calling a Device function not available due to the current state of the
    Device application (for example a system command). */
    IOL_eErrorType_FUNC_UNAVAILTEMP = 0x8036,
    /** \brief Invalid parameter set.

    This ErrorType shall be used if values sent via single parameter transfer
    are not consistent with other actual parameter settings (for example
    overlapping set points for a binary data setting; see 10.3.4). */
    IOL_eErrorType_PAR_SETINVALID = 0x8040,
    /** \brief Inconsistent parameter set.

    This ErrorType shall be used at the termination of a block parameter
    transfer with ParamDownloadEnd or ParamDownloadStore if the plausibility
    check shows inconsistencies (see 10.3.5 and B.2.2). */
    IOL_eErrorType_PAR_SETINCONSIST = 0x8041,
    /** \brief Application not ready.

    This ErrorType shall be used if a read or write service is refused due to a
    temporarily unavailable application (for example peripheral controllers
    during startup). */
    IOL_eErrorType_APP_DEVNOTRDY = 0x8082,
    /** \brief Vendor specific.

    This ErrorType will be propagated directly to higher level processing
    elements as an error (no warning) by the Master. */
    IOL_eErrorType_UNSPECIFIC = 0x8100,
    /** \brief Begin of Vendor specific ErrorTypes.

    This ErrorType will be propagated directly to higher level processing
    elements as an error (no warning) by the Master. */
    IOL_eErrorType_VENDOR_SPECIFIC_BEGIN = 0x8101,
    /** \brief End of Vendor specific ErrorTypes. */
    IOL_eErrorType_VENDOR_SPECIFIC_END = 0x81FF,

    /** \brief Master – Communication error.

    The Master generates a negative service response with this ErrorType if a
    communication error occurred during a read or write service, for
    example the SDCI connection is interrupted. */
    IOL_eErrorType_COM_ERR = 0x1000,
    /** \brief ISDU timeout, ISDU error or ISDU illegal service primitive.

    - Master – ISDU timeout:
    The Master generates a negative service response with this ErrorType, if a
    Read or Write service is pending longer than the specified I-Service timeout
    (see Table 97) in the Master.
    - Device Event – ISDU error:
    If the Master received an event with the EventQualifier (see A.6.4: DL,
    Error, Event single shot) and the EventCode 0x5600, a negative service
    response indicating a service timeout is generated and returned to the
    requester (see C.3.3).
    - Device Event – ISDU illegal service primitive:
    If the Master received an event with the EventQualifier (see A.6.4: AL,
    Error, Event single shot) and the EventCode 0x5800, a negative service
    response indicating a service timeout is generated and returned to the
    requester (see C.3.3).
    */
    IOL_eErrorType_I_SERVICE_TIMEOUT = 0x1100,
    /** \brief Master – ISDU checksum error.

    The Master generates a negative service response with this ErrorType, if its
    data link layer detects an ISDU checksum error. */
    IOL_eErrorType_M_ISDU_CHECKSUM = 0x5600,
    /** \brief Master – ISDU illegal service primitive.

    The Master generates a negative service response with this ErrorType, if its
    data link layer detects an ISDU illegal service primitive. */
    IOL_eErrorType_M_ISDU_ILLEGAL = 0x5700,

    /** \brief SMI ErrorTypes. */
    IOL_eErrorType_ARGBLOCK_NOT_SUPPORTED = 0x4001,
    IOL_eErrorType_ARGBLOCK_ID_NOT_SUPPORTED = 0x4002,
    IOL_eErrorType_DEVICE_NOT_ACCESSIBLE = 0x4003,
    IOL_eErrorType_SERVICE_NOT_SUPPORTED = 0x4004,
    IOL_eErrorType_CMD_NOT_SUPPORTED = 0x4005,
    IOL_eErrorType_PORT_NUM_INVALID = 0x4011,
    IOL_eErrorType_ARGBLOCK_LENGTH_INVALID = 0x4034,
    IOL_eErrorType_SERVICE_TEMP_UNAVAILABLE = 0x4036,
    IOL_eErrorType_PORT_CONFIG_INCONSISTENT = 0x4041,
    IOL_eErrorType_ACCESS_RIGHT_NOT_GRANTED = 0x4042,
} IOL_EErrorType;

/**
\brief This enumeration lists the specified EventCode identifiers and their
definitions. The EventCodes are created by the technology specific Device
application (instance = APP).
*/
typedef IOL_ENUM_DECL IOL_EECode
{
    /** \brief No malfunction (Notification). */
    IOL_eECode_NO_MALFUNCTION = 0x0000,
    /** \brief General malfunction – unknown error (Error). */
    IOL_eECode_GENERAL_MALFUNCTION = 0x1000,

    /* 0x1800 - 0x18FF: Vendor specific. */

    /** \brief Temperature fault – Overload (Error). */
    IOL_eECode_TEMP_FAULT = 0x4000,
    /** \brief Device temperature overrun – Clear source of heat (Warning). */
    IOL_eECode_TEMP_OVERRUN = 0x4210,
    /** \brief Device temperature underrun – Insulate Device (Warning). */
    IOL_eECode_TEMP_UNDERRUN = 0x4220,
    /** \brief Device hardware fault – Device exchange (Error). */
    IOL_eECode_HW_FAULT = 0x5000,
    /** \brief Component malfunction – Repair or exchange (Error). */
    IOL_eECode_COMP_MALFUNCTION = 0x5010,
    /** \brief Non volatile memory loss – Check batteries (Error). */
    IOL_eECode_MEM_LOSS = 0x5011,
    /** \brief Batteries low – Exchange batteries (Warning). */
    IOL_eECode_BATTERIES_LOW = 0x5012,
    /** \brief General power supply fault – Check availability (Error). */
    IOL_eECode_SUPPLY_FAULT = 0x5100,
    /** \brief Fuse blown/open – Exchange fuse (Error). */
    IOL_eECode_FUSE_OPEN = 0x5101,
    /** \brief Primary supply voltage over-run – Check tolerance (Warning). */
    IOL_eECode_SUPPLY_OVERRUN = 0x5110,
    /** \brief Primary supply voltage under-run – Check tolerance (Warning). */
    IOL_eECode_SUPPLY_UNDERRUN = 0x5111,
    /** \brief Secondary supply voltage fault (Port Class B) – Check tolerance (Warning). */
    IOL_eECode_SEC_SUPPLY_FAULT = 0x5112,
    /** \brief Device software fault – Check firmware revision (Error). */
    IOL_eECode_SW_FAULT = 0x6000,
    /** \brief Parameter error – Check data sheet and values (Error). */
    IOL_eECode_PARAM_ERROR = 0x6320,
    /** \brief Parameter missing – Check data sheet (Error). */
    IOL_eECode_PARAM_MISSING = 0x6321,
    /** \brief Parameter changed – Check configuration (Error). */
    IOL_eECode_PARAM_CHANGED = 0x6350,
    /** \brief Wire break of a subordinate Device – Check installation (Error). */
    IOL_eECode_WIRE_BREAK = 0x7700,
    /** \brief Wire break of subordinate Device 1-15 – Check installation (Error). */
    IOL_eECode_WIRE_BREAK_D1 = 0x7701,
    IOL_eECode_WIRE_BREAK_D2 = 0x7702,
    IOL_eECode_WIRE_BREAK_D3 = 0x7703,
    IOL_eECode_WIRE_BREAK_D4 = 0x7704,
    IOL_eECode_WIRE_BREAK_D5 = 0x7705,
    IOL_eECode_WIRE_BREAK_D6 = 0x7706,
    IOL_eECode_WIRE_BREAK_D7 = 0x7707,
    IOL_eECode_WIRE_BREAK_D8 = 0x7708,
    IOL_eECode_WIRE_BREAK_D9 = 0x7709,
    IOL_eECode_WIRE_BREAK_D10 = 0x770A,
    IOL_eECode_WIRE_BREAK_D11 = 0x770B,
    IOL_eECode_WIRE_BREAK_D12 = 0x770C,
    IOL_eECode_WIRE_BREAK_D13 = 0x770D,
    IOL_eECode_WIRE_BREAK_D14 = 0x770E,
    IOL_eECode_WIRE_BREAK_D15 = 0x770F,
    /** \brief Short circuit – Check installation (Error). */
    IOL_eECode_SHORT_CIRCUIT = 0x7710,
    /** \brief Ground fault – Check installation (Error). */
    IOL_eECode_GROUND_FAULT = 0x7711,
    /** \brief Technology specific application fault – Reset Device (Error). */
    IOL_eECode_APP_FAULT = 0x8C00,
    /** \brief Simulation active – Check operational mode (Warning). */
    IOL_eECode_SIM_ACTIVE = 0x8C01,
    /** \brief Process variable range over-run – Process Data uncertain (Warning). */
    IOL_eECode_PD_OVERRUN = 0x8C10,
    /** \brief Measurement range over-run – Check application (Error). */
    IOL_eECode_MEASUREMENT_OVERRUN = 0x8C20,
    /** \brief Process variable range under-run – Process Data uncertain (Warning). */
    IOL_eECode_PD_UNDERRUN = 0x8C30,
    /** \brief Maintenance required – Cleaning (Notification). */
    IOL_eECode_MAINT_CLEANING = 0x8C40,
    /** \brief Maintenance required – Refill (Notification). */
    IOL_eECode_MAINT_REFILL = 0x8C41,
    /** \brief Maintenance required – Exchange wear and tear parts (Notification). */
    IOL_eECode_MAINT_EXCHANGE = 0x8C42,

    /* 0x8CA0 - 0x8DFF: Vendor specific. */

    /* 0xFF00 - 0xFFFF: SDCI specific EventCodes (see Table D.2). */

    IOL_eECode_EVENT = 0xFF31,
    /** \brief Data Storage upload request. */
    IOL_eECode_DS_UPLOAD_REQ = 0xFF91,

    /** \brief Remote max retry error. */
    IOL_eECode_IOLW_RETRY_ERROR_R = 0xFFB0,
    /** \brief HMI Button pressed. */
    IOL_eECode_IOLW_HMI_BUTTON_PRESS = 0xFFB1,
} IOL_EECode;

/**
\brief This enumeration lists the specified EventCode identifiers and their
definitions. The EventCodes are created by the technology specific Device
application (instance = APP).
*/
typedef IOL_ENUM_DECL IOL_EEPortCode
{
    /* 0x0000 - 0x17FF: Reserved. */
    IOL_eEPortCode_NoDevice = 0x1800,
    IOL_eEPortCode_StartupParamError = 0x1801,
    IOL_eEPortCode_IncorrectVendorId = 0x1802,
    IOL_eEPortCode_IncorrectDeviceId = 0x1803,
    IOL_eEPortCode_ShortCircuit = 0x1804,
    IOL_eEPortCode_PhyOvertemperature = 0x1805,
    IOL_eEPortCode_ShortCircuitLP = 0x1806,
    IOL_eEPortCode_OvercurrentAtLp = 0x1807,
    IOL_eEPortCode_DeviceEventOverflow = 0x1808,
    IOL_eEPortCode_BackupMemOutOfRange = 0x1809,
    IOL_eEPortCode_BackupIdentityFault = 0x180A,
    IOL_eEPortCode_BackupUnspecificError = 0x180B,
    IOL_eEPortCode_BackupUploadFault = 0x180C,
    IOL_eEPortCode_BackupDownloadFault = 0x180D,
    IOL_eEPortCode_P24MissingUndervoltage = 0x180E,
    IOL_eEPortCode_ShortCircuitP24 = 0x180F,
    IOL_eEPortCode_ShortCircuitIQ = 0x1810,
    IOL_eEPortCode_ShortCircuitAtCQ = 0x1811,
    IOL_eEPortCode_OvercurrentIQ = 0x1812,
    IOL_eEPortCode_OvercurrentCQ = 0x1813,

    /* 0x1814 - 0x1EFF: Reserved. */
    /* 0x1F00 - 0x1FFF: VendorSpecific. */
    IOL_eEPortCode_EvalExpired = 0x1F00,
    /* 0x2000 - 0x2FFF: Safety Extension. */
    /* 0x3000 - 0x2FFF: Wireless Extension. */
    /* 0x4000 - 0x5FFF: Reserved. */

    IOL_eEPortCode_InvalidCycleTime = 0x6000,
    IOL_eEPortCode_RevisionFault = 0x6001,
    IOL_eEPortCode_ISDUBatchFailed = 0x6002,
    /* 0x8003 - 0xFF20 Reserved. */

    /** \brief Mode indication. */
    IOL_eEPortCode_NEW_SLAVE = 0xFF21,
    /** \brief Device communication lost. */
    IOL_eEPortCode_DEV_COM_LOST = 0xFF22,
    /** \brief Data Storage identification mismatch. */
    IOL_eEPortCode_DS_IDENT_MISMATCH = 0xFF23,
    /** \brief Data Storage buffer overflow. */
    IOL_eEPortCode_DS_BUFFER_OVERFLOW = 0xFF24,
    /** \brief Data Storage parameter access denied. */
    IOL_eEPortCode_DS_ACCESS_DENIED = 0xFF25,
    /** \brief Incorrect event signaling. */
    IOL_eEPortCode_PortStatusChanged = 0xFF26,
    IOL_eEPortCode_DataStorageComplete = 0xFF27,
    /* 0xFF28 - 0xFF30: Reserved. */

    IOL_eEPortCode_IncorectEventSignalling = 0xFF31,

    /* 0xFF32 - 0xFFFF: Reserved. */

    /** \brief Local max retry error. */
    IOL_eEPortCode_IOLW_RETRY_ERROR_L = 0x3000,
    /** \brief Ima Timeout. */
    IOL_eEPortCode_IOLW_IMA_TIMEOUT = 0x3001,
    /** \brief Unique ID Changed after Pairing by button. */
    IOL_eEPortCode_IOLW_U_ID_CHANGED = 0x3002,
} IOL_EEPortCode;

/**
\brief This enumeration indicates the particular source (instance) of an event
thus refining its evaluation on the receiver side.
*/
typedef IOL_ENUM_DECL IOL_EEInstance
{
    IOL_eEInstance_UNKNOWN = 0,
    IOL_eEInstance_PHY, /**< \brief Source is IO-Link physical layer */
    IOL_eEInstance_DL, /**< \brief Source is IO-Link data link layer */
    IOL_eEInstance_AL, /**< \brief Source is IO-Link application layer */
    IOL_eEInstance_APPLICATION, /**< \brief Event source is application layer */
    IOL_eEInstance_RESERVED_4,
    IOL_eEInstance_RESERVED_5,
    IOL_eEInstance_RESERVED_6
} IOL_EEInstance;

/**
\brief This enumeration indicates the source of the event.
*/
typedef IOL_ENUM_DECL IOL_EESource
{
    /** \brief Source is Device which means it is a remote event. */
    IOL_eESource_DEVICE = 0,
    /** \brief Source is Master which means it is a local event. */
    IOL_eESource_MASTER
} IOL_EESource;

/**
\brief This enumeration indicates the event category.
*/
typedef IOL_ENUM_DECL IOL_EEType
{
    IOL_eEEType_RESERVED = 0,
    IOL_eEEType_NOTIFICATION, /**< \brief Notification */
    IOL_eEEType_WARNING, /**< \brief Warning */
    IOL_eEEType_ERROR /**< \brief Error */
} IOL_EEType;

/**
\brief This enumeration indicates the event mode.
*/
typedef IOL_ENUM_DECL IOL_EEMode
{
    IOL_eEEMode_RESERVED = 0,
    IOL_eEEMode_SINGLE_SHOT, /**< \brief Single shot event */
    IOL_eEEMode_DISAPPEARS, /**< \brief Event disappears */
    IOL_eEEMode_APPEARS /**< \brief Event appears */
} IOL_EEMode;

/**
\brief This enumeration indicates the DS commands.
*/
typedef IOL_ENUM_DECL IOL_EDSCommand
{
    IOL_eDSCommand_Reserved = 0,
    IOL_eDSCommand_UploadStart, 
    IOL_eDSCommand_UploadEnd,
    IOL_eDSCommand_DownloadStart,
    IOL_eDSCommand_DownloadEnd,
    IOL_eDSCommand_Break
} IOL_EDSCommand;

/**
\brief This enumeration indicates the predefined system commands.
*/
typedef IOL_ENUM_DECL IOL_ESystemCommand
{
    IOL_eSystemCommand_Reserved = 0,
    IOL_eSystemCommand_UploadStart,
    IOL_eSystemCommand_UploadEnd,
    IOL_eSystemCommand_DownloadStart,
    IOL_eSystemCommand_DownloadEnd,
    IOL_eSystemCommand_DownloadEndStore,
    IOL_eSystemCommand_Break,
    /* 7 to 63 reserved */
    /* 64 to 127 reserved for profiles */
    IOL_eSystemCommand_DeviceReset = 128,
    IOL_eSystemCommand_ApplicationReset = 129,
    IOL_eSystemCommand_RestoreFactorySettings = 130,
    IOL_eSystemCommand_BackToBox = 131
    /* 132 to 159 reserved */
    /* 160 to 255 Vendor specific */
} IOL_ESystemCommand;


/**
\brief This structure stores the Direct Parameter Page 1.
*/
typedef struct IOL_SDirectParam1
{
    INT8U u8MasterCommand;
    INT8U u8MasterCycleTime;
    INT8U u8MinCycleTime;
    INT8U u8MSequenceCapability;
    INT8U u8RevisionID;
    INT8U u8ProcessDataIn;
    INT8U u8ProcessDataOut;
    INT8U u8VendorID1;
    INT8U u8VendorID2;
    INT8U u8DeviceID1;
    INT8U u8DeviceID2;
    INT8U u8DeviceID3;
    INT8U u8FunctionID1;
    INT8U u8FunctionID2;
    INT8U u8Reserved;
    INT8U u8SystemCommand;
}IOL_SDirectParam1;


/**
\brief This structure defines a event buffer entry.
*/
typedef struct IOL_SEventEntry
{
    IOL_EEInstance eEInstance;
    IOL_EEMode eEMode;
    IOL_EEType eEType;
    IOL_EESource eOrigin;
    INT16U u16ECode;
}IOL_SEventEntry;

#if (defined(IOLD_WIRELESS) || defined(IOLM_WIRELESS))
/**
\brief This enumeration indicates the downlink channel ID.
*/
typedef IOL_ENUM_DECL IOL_EDLChan
{
    IOL_eDLChan_Invalid = 0,
    IOL_eDLChan_Process,
    IOL_eDLChan_PDInvalid,
    IOL_eDLChan_ISDU,
    IOL_eDLChan_Event,
    IOL_eDLChan_Mcmd,
    // other reserved
} IOL_EDLChan;
#endif

/**
\brief Physical layer slot types.
*/
typedef IOL_ENUM_DECL IOL_ESlotType
{
    IOL_eSlotType_SSLOT = 0,
    IOL_eSlotType_DSLOT = 1,
}IOL_ESlotType;

#if (defined(IOLD_WIRELESS) || defined(IOLM_WIRELESS))
/**
\brief Physical layer uplink type.
*/
typedef IOL_ENUM_DECL IOLM_EULinkType
{
    IOLM_eULinkType_DATA = 0,
    IOLM_eULinkType_NOUPLINK,
    IOLM_eULinkType_IMA,
}IOLM_EULinkType;
#endif


#if (defined(IOLD_WIRELESS) || defined(IOLM_WIRELESS))
/**
\brief Physical layer pairing result.
*/
typedef IOL_ENUM_DECL IOL_EPairInfo
{
    IOL_ePairInfo_SUCCESS = 0,
    IOL_ePairInfo_TIMEOUT = 1,
    IOL_ePairInfo_WRONG_SLOTTYPE = 2,
}IOL_EPairInfo;
#endif


#if (defined(IOLD_WIRELESS) || defined(IOLM_WIRELESS))
/**
\brief Physical layer pairing methods.
*/
typedef IOL_ENUM_DECL IOL_EPairMethod
{
    IOL_ePairMethod_UNKNOWN = 0,
    IOL_ePairMethod_PAIRING_BUTTON = 0x80,
    IOL_ePairMethod_PAIRING_UNIQUE = 0x90,
    IOL_ePairMethod_RE_PAIRING = 0x50,
    IOL_ePairMethod_ABORTED = 0x02,
    IOL_ePairMethod_UNPAIRING = 0xFF,
}IOL_EPairMethod;
#endif

#if (defined(IOLD_WIRELESS) || defined(IOLM_WIRELESS))
/**
\brief Physical layer hopping table change operations.
*/
typedef IOL_ENUM_DECL IOL_EHopUpdateType
{
    IOL_eHopUpdateType_FULL_TABLE = 0x00,
    IOL_eHopUpdateType_DELETE_CELL = 0x01,
    IOL_eHopUpdateType_ADD_CELL = 0x02,
    IOL_eHopUpdateType_REPLACE_CELL = 0x03,
}IOL_EHopUpdateType;
#endif

#if (defined(IOLD_WIRELESS) || defined(IOLM_WIRELESS))
/**
\brief Physical layer trigger Commands.
*/
typedef IOL_ENUM_DECL IOL_EPLCommand
{
    IOL_ePLCommand_WAKE_UP_TIME = 0,    ///< Trigger the PL to deliver the current WakeUpCountdown value
    IOL_ePLCommand_W_DEVICE_AWAKE,      ///< Indicate low energy W-Device sent IMA at WakeUpTime
    IOL_ePLCommand_W_DEVICE_NOT_AWAKE,  ///< Indicate low energy W-Device did not send IMA
    IOL_ePLCommand_JUMP,                ///< Switch to new hopping table, starting with Hop-1 frequency
    IOL_ePLCommand_JUMP_FAIL,           ///< W-Device did not acknowledge JUMP command
}IOL_EPLCommand;
#endif

#if (defined(IOLD_WIRELESS) || defined(IOLM_WIRELESS))
/**
\brief Structure for AdaptiveHopTable ISDU
*/
typedef struct IOL_SHopTableISDU
{
    INT8U au8WakeUpTimeOctets[IOL_WAKE_UP_TIME_OCTETS_LEN];
    INT8U u8UpdateType;
    INT8U u8Index;
    INT8U au8FrequencyValues[IOL_PL_HOP_TABLE_LEN_MAX];
}IOL_SHopTableISDU;
#endif

/** \} */

#ifdef __cplusplus
}
#endif

#endif /* INC_PROT__IOL_TYPES_H__ */

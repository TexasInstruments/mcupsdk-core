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

#ifndef INC_PROT__IOLM_TYPES_H__
#define INC_PROT__IOLM_TYPES_H__

#ifndef IOL_ONLY_TYPE_INCLUDE
#include "IOL_Types.h"
#include "IOLM_Port_Definition.h"

#ifdef __cplusplus
extern "C" {
#endif

#else
// This case is used for including the types in higher level software
#define IOLM_PORT_COUNT 1
#define IOLM_PD_BUFFER_COUNT 1
#endif


#ifndef IOL_ENUM_DECL
#define IOL_ENUM_DECL enum
#endif

#ifndef INC_PROT__IOL_TYPES_H__
#error Need to include IOL_Types.h beefore IOLM_Types.h
#endif

#ifndef IOLM_EVENT_LIMIT
#define IOLM_EVENT_LIMIT 0xffffffff
#endif

/**
\addtogroup group_iolm_types IO-Link Master Types
\{
\brief Types used for the Master implementation.

*/

/** \brief Number of soft timer resources. Used for wake-up retry or ISDU timeout. */
#define IOLM_SOFTTIMER_COUNT    ((IOLM_PORT_COUNT * 2) + 2)

/** \brief Maximum number of Process Data bytes. */
#define IOLM_MAX_PD_SIZE    32

/** \brief Number of Master message retries. */
#define IOLM_MAX_RETRIES    2

/** \brief Maximum number of OD bytes per message. */
#define IOLM_MAX_OD_SIZE    32

/** \brief Minimum M-Sequence length */
#define IOLM_TX_HEADER_SIZE 2

/** \brief Maximum number of wake-up retries. */
#define IOLM_MAX_WAKE_RETRY 3

/** \brief Maximum number of event entries. */
#define IOLM_MAX_EVENT_ENTRIES 6

/** \brief Maximum size of DS content. */
#define IOLM_DS_MAX_SIZE    2048

/** \brief Size of DS header. */
#define IOLM_DS_EMPTY   IOLM_SDSHeaderSize

/** \brief Maximum length of ISDU frame (data + header + checksum). */
#define IOLM_MAX_ISDU_LENGTH (238)

/** \brief Timeout for wake-up retry. */
#define IOLM_WAKETIMEOUT    (IOL_T_REN_US+IOL_BIT_USEC(IOL_T_TDMT_BIT,IOL_BAUD_COM3)-IOLM_WAKEUP_PULSE_TIMER_US) // TREN + TDMT - Wake-up


/** \brief Minimum time for IOL startup cycle. */
#define IOLM_MIN_STARTUPCYCLE(baudrate)   (IOL_BIT_USEC(IOL_MSEQ_BITS(2,2), baudrate) + IOLM_MASTER_PROCESSING_TIME)

/** \brief Maximum number of jobs. */
#define IOLM_NUM_OF_JOBS (IOLM_PORT_COUNT * 5) // PDIN, PDOUT, Event, Mcmd, ISDU

/** \brief Minimum time interval for IO-Link timers. */
#define IOLM_MIN_TIMER_INTERVAL_US     10

/** Ready pulse time expectations. */
/** Lower limit of ready pulse length in us. */
#define IOLM_PORT_READY_PULSE_MIN_DURATION_US   400U
/** Upper limit of ready pulse length in us. */
#define IOLM_PORT_READY_PULSE_MAX_DURATION_US   1100U 
/** Timeout for ready pulse detection in ms. */
#define IOLM_PORT_READY_PULSE_TIMEOUT_MS    5000U 
/** Wait time between power off an power on in ms. */
#define IOLM_PORT_READY_PULSE_POWER_OFF_ON_GAP_MS    1000U 

// Checks 


#if IOLM_PD_BUFFER_COUNT < 1 || IOLM_PD_BUFFER_COUNT > 3
#error Process Data buffer count value is invalid
#endif

// redefine available defines (used for unifdef)
#undef IOLM_SMI_ENABLED
#define IOLM_SMI_ENABLED  1

#ifdef UNIT_TESTS
#define IOLM_CRITICAL_DECL(level)
#define IOLM_CRITICAL_START(level)
#define IOLM_CRITICAL_END(level)
#endif


#ifndef IOLM_CRITICAL_DECL
#define IOLM_CRITICAL_DECL(level)       INT32U (level)
#endif

#ifndef IOLM_CRITICAL_START
#define IOLM_CRITICAL_START(level)      (level) = IOLM_Port_vCriticalStart()
#endif

#ifndef IOLM_CRITICAL_END
#define IOLM_CRITICAL_END(level)        IOLM_Port_vCriticalEnd((level))
#endif

                                                


/**
\brief DL-mode handler states.
*/
typedef IOL_ENUM_DECL IOLM_EModeHState
{
    /** \brief Waiting on wake-up request from System Management (SM) */
    IOLM_eModeHState_IDLE = 0,
    /** \brief Perform wake-up procedure. */
    IOLM_eModeHState_WURQ,
    /** \brief Try communication with mode COM3. */
    IOLM_eModeHState_ComRequestCOM3,
    /** \brief Try communication with mode COM2. */
    IOLM_eModeHState_ComRequestCOM2,
    /** \brief Try communication with mode COM1. */
    IOLM_eModeHState_ComRequestCOM1,
    /** \brief Retry startup */
    IOLM_eModeHState_Retry,
    /** \brief Establish communication */
    IOLM_eModeHState_EstablishCom,
    /** \brief System Management uses the STARTUP state for Device
    identification, check, and communication configuration (see figure 69). */
    IOLM_eModeHState_STARTUP,
    /** \brief On-request Data Exchange (parameter, commands, events) without
    Process Data. */
    IOLM_eModeHState_PREOPERATE,
    /** \brief Process Data and On-request Data exchange (parameter, commands,
    events). */
    IOLM_eModeHState_OPERATE
} IOLM_EModeHState;

/**
\brief OD handler states.
*/
typedef IOL_ENUM_DECL IOLM_EODHState
{
    IOLM_eODHState_Inactive = 0,
    IOLM_eODHState_ISDU,
    IOLM_eODHState_COMMAND,
    IOLM_eODHState_EVENT,
} IOLM_EODHState;

/**
\brief ISDU handler states.
*/
typedef IOL_ENUM_DECL IOLM_EISDUState
{
    IOLM_eISDUState_Idle = 0,
    IOLM_eISDUState_Page,
    IOLM_eISDUState_ISDURequest,
    IOLM_eISDUState_ISDUWait,
    IOLM_eISDUState_ISDUError,
    IOLM_eISDUState_ISDUResponse,
}IOLM_EISDUState;


/**
\brief This enumeration indicates the change mode of the message handler.
*/
typedef IOL_ENUM_DECL IOLM_EMHMode
{
    IOLM_eMHMode_IDLE = 0,
    IOLM_eMHMode_COMx,
    IOLM_eMHMode_STARTUP,
    IOLM_eMHMode_PREOPERATE,
    IOLM_eMHMode_OPERATE
} IOLM_EMHMode;

/**
\brief This enumeration indicates the SM states.
*/
typedef IOL_ENUM_DECL IOLM_ESMState
{
    IOLM_eSMState_PortInactive = 0,
    IOLM_eSMState_WaitonDLStartup,
    IOLM_eSMState_Pairing,
    IOLM_eSMState_Unpairing,
    IOLM_eSMState_Disconnect,
    IOLM_eSMState_ReadWirelessParam,
    IOLM_eSMState_WriteWirelessParam,
    IOLM_eSMState_ReadComParameter,
    IOLM_eSMState_ReadComParameter2,
    IOLM_eSMState_CheckCompV10,
    IOLM_eSMState_CheckVxy,
    IOLM_eSMState_CheckCompVxy,
    IOLM_eSMState_RestartDevice,
    IOLM_eSMState_WaitonDLPreoperate,
    IOLM_eSMState_CheckSerNum,
    IOLM_eSMState_DataStorage,
    IOLM_eSMState_Wait,
    IOLM_eSMState_MasterCycle,
    IOLM_eSMState_SM_Operate,
    IOLM_eSMState_InspectionFault,
    IOLM_eSMState_WaitonDLOperate,
    IOLM_eSMState_DIDO,
    IOLM_eSMState_WaitOnReadyPulse,
    IOLM_eSMState_WaitOnAuthentication,
    IOLM_eSMState_PortPowerOff
} IOLM_ESMState;

/**
\brief This enumeration indicates the PD states.
*/
typedef IOL_ENUM_DECL IOLM_EPDState
{
    IOLM_ePDState_Inactive = 0,
    IOLM_ePDState_PDSingle,
    IOLM_ePDState_PDInInterleave,
    IOLM_ePDState_PDOutInterleave,
} IOLM_EPDState;

/**
\brief This enumeration indicates the AL OD states.
*/
typedef IOL_ENUM_DECL IOLM_EALODState
{
    IOLM_eALODState_Idle = 0,
    IOLM_eALODState_ParamRead,
    IOLM_eALODState_ParamWrite,
    IOLM_eALODState_ISDURead,
    IOLM_eALODState_ISDUWrite,
    IOLM_eALODState_ReadAbort,
    IOLM_eALODState_WriteAbort,
} IOLM_EALODState;

/**
\brief This enumeration indicates the DL event states.
*/
typedef IOL_ENUM_DECL IOLM_EDLEventState
{
    IOLM_eDLEventState_Inactive = 0,
    IOLM_eDLEventState_Idle,
    IOLM_eDLEventState_ReadEvent,
    IOLM_eDLEventState_SendConf,
    IOLM_eDLEventState_EventConfirmation,
} IOLM_EDLEventState;

/**
\brief This enumeration indicates the AL event states.
*/
typedef IOL_ENUM_DECL IOLM_EALEventState
{
    IOLM_eALEventState_Idle = 0,
    IOLM_eALEventState_ReadEvent,
    IOLM_eALEventState_EventHandling
} IOLM_EALEventState;

/**
\brief This enumeration indicates the requested inspection Level.
*/
typedef IOL_ENUM_DECL IOLM_EInspectionLevel 
{
   /** \brief No check on startup. */
   IOL_eInspectionLevel_NO_CHECK = 0,
   /** \brief Check Vendor ID and Device ID. */
   IOL_eInspectionLevel_TYPE_COMP,
   /** \brief Check Vendor ID, Device ID and Serial Number. */
   IOL_eInspectionLevel_IDENTICAL
} IOLM_EInspectionLevel;

/**
\brief This enum indicates the current hardware timer state.
*/
typedef IOL_ENUM_DECL IOLM_ETimerState
{
    IOLM_eTimerState_None = 0,
    IOLM_eTimerState_Mseq,
    IOLM_eTimerState_Cycle,
    IOLM_eTimerState_TDMT,
    IOLM_eTimerState_Wakeup,
    IOLM_eTimerState_Pulse,
} IOLM_ETimerState;

/** 
\brief This enumeration defines acyclic events.

The order of definition is also the order of execution.
*/
typedef IOL_ENUM_DECL IOLM_EEvent
{
    IOLM_eEvent_StackHigh = 0, /**< \brief Receive processing and prepare next frame. */
    IOLM_eEvent_StackLow,
    IOLM_eEvent_UserHigh,
    IOLM_eEvent_UserLow,
    IOLM_eEvent_MAX, /**< \brief Number of events. */
}IOLM_EEvent;

/**
\brief This IOL_ENUM_DECL defines the Data Storage states.
*/
typedef IOL_ENUM_DECL IOLM_EDSState
{
    IOLM_eDSState_CheckActivationState_0 = 0,
    IOLM_eDSState_WaitingOnDSActivity_1,
    IOLM_eDSState_UpDownload_2,
    IOLM_eDSState_Off_3,
    IOLM_eDSState_CheckIdentity_4,
    IOLM_eDSState_CheckMemSize_5,
    IOLM_eDSState_CheckUpload_6,
    IOLM_eDSState_Upload_7,
    IOLM_eDSState_CheckDsValidity_8,
    IOLM_eDSState_CheckChecksum_9,
    IOLM_eDSState_Download_10,
    IOLM_eDSState_DS_Ready_11,
    IOLM_eDSState_DS_Fault_12,
    IOLM_eDSState_Decompose_IL_13,
    IOLM_eDSState_ReadParameter_14,
    IOLM_eDSState_StoreDataSet_15,
    IOLM_eDSState_Upload_Fault_16,
    IOLM_eDSState_Decompose_Set_17,
    IOLM_eDSState_Write_Parameter_18,
    IOLM_eDSState_Download_Done_19,
    IOLM_eDSState_Download_Fault_20,
#ifdef IOLM_DEVICE_TESTER
    IOLM_eDSState_ReadStatusDLStart,
    IOLM_eDSState_ReadStatusDLEnd,
    IOLM_eDSState_ReadStatusULStart,
    IOLM_eDSState_ReadStatusULEnd,
    IOLM_eDSState_ReadStatusReadParam,
#endif
}IOLM_EDSState;

/**
\brief this IOL_ENUM_DECL defines the Data Storage activation states.
*/
typedef IOL_ENUM_DECL IOLM_EDSActivationState
{
    /** \brief Data Storage not enabled. */
    IOLM_eDSActivationState_Disabled = 0,
    /** \brief Data Storage enabled. */
    IOLM_eDSActivationState_Enabled,
    /** \brief Data Storage disabled and cleared. */
    IOLM_eDSActivationState_Cleared,
}IOLM_EDSActivationState;

/**
\brief This IOL_ENUM_DECL defines the Data Storage activities which have to
be done if there are mismatches detected. The index/order of the values
should not be changed, because they are used as a bitmask.
*/
typedef IOL_ENUM_DECL IOLM_EDSMode
{
    /** \brief No transfer enabled. */
    IOLM_eDSMode_Nothing = 0,
    /** \brief Only download enabled. */
    IOLM_eDSMode_Download = 1,
    /** \brief Only upload enabled. */
    IOLM_eDSMode_Upload = 2,
    /** \brief Upload and download enabled. */
    IOLM_eDSMode_UpDownload = 3
}IOLM_EDSMode;

/**
\brief This IOL_ENUM_DECL defines the DS faults.
*/
typedef IOL_ENUM_DECL IOLM_EDSFault
{
    IOLM_eDSFault_None = 0,
    IOLM_eDSFault_Communication,
    IOLM_eDSFault_Identification,
    IOLM_eDSFault_SizeCheck,
    IOLM_eDSFault_Upload,
    IOLM_eDSFault_Download,
    IOLM_eDSFault_StorageLocked,
}IOLM_EDSFault;


/**
\brief This IOL_ENUM_DECL defines the wake-up modes.
*/
typedef IOL_ENUM_DECL IOLM_EWakeupMode
{
    /** \brief Wake-up is generated by port. */
    IOLM_eWakeupMode_Done = 0,
    /** \brief Wake-up need to be generated by stack. */
    IOLM_eWakeupMode_Stack,
    /** \brief Wake-up is done by application and acknowledged by #IOLM_API_PL_u8WakeAck. */
    IOLM_eWakeupMode_Ack,
    /** \brief Wake-up is generated by Phy with frame handler mode. */
    IOLM_eWakeupMode_Phy,
}IOLM_EWakeupMode;

#ifdef IOLM_DEVICE_TESTER
/**
\brief This structure is used for the Master tester.
*/
typedef enum IOLM_EMTestEvent
{
    EMTestEvent_None = 0,
    EMTestEvent_Startup,
    EMTestEvent_Preoperate,
    EMTestEvent_Operate,
    EMTestEvent_SendOperateCommand,
    EMTestEvent_CheckComp,
    EMTestEvent_PrepareFrame,
    EMTestEvent_Writepage,
    EMTestEvent_Readpage,
    EMTestEvent_PrepareTransfer,
    EMTestEvent_ISDUChecksum,
    EMTestEvent_ISDUStart,
    EMTestEvent_ISDUResponse,
    EMTestEvent_EventAck,
    EMTestEvent_EventReadBreak,
    EMTestEvent_DSReadStatus,
    EMTestEvent_StartupCycle,
    EMTestEvent_NoResponse,
}IOLM_EMTestEvent;
#endif

/**
\brief This structure stores the message handler configuration.
*/
typedef struct IOLM_SMsgHandlerConfig
{
    INT32U u32MSequenceTime; /**< \brief M-Sequence time in 1 us granularity. */
    IOL_EMSequence eMSequenceType;
    TBOOL boInterleave;
    INT8U u8PDInputLength;
    INT8U u8PDOutputLength;
    INT8U u8ODLength;
} IOLM_SMsgHandlerConfig;

/**
\brief This structure defines Data Storage objects

The structure can be used on non aligned buffer. Because of this, there are only
byte values defined. The data field is only a dummy value and it's real length
depends on the content.
*/
typedef struct IOLM_SDSObject
{
    INT8U au8Index[2];
    INT8U u8Subindex;
    INT8U u8Length;
    INT8U au8Data[4];
}IOLM_SDSObject;

/**
\brief This structure defines the Data Storage content.
*/
#pragma pack(push, 1)
typedef struct IOLM_SDSContent
{
    INT32U u32NVChecksum;
    INT16U u16Size; // size of this structure with valid content
    // can be used for direct usage in SMI:
    INT8U au8ArgBlockID[2];
    INT8U au8Checksum[4];
    INT8U au8VendorID[2];
    INT8U au8DeviceID[4]; 
    INT8U au8FunctionID[2];

    INT8U au8Data[IOLM_DS_MAX_SIZE]; // Multiple IOLM_SDSObject
}IOLM_SDSContent;
#pragma pack(pop)

#define IOLM_SDSHeaderSize (sizeof(IOLM_SDSContent)-IOLM_DS_MAX_SIZE)

/**
\brief This structure is used for the desired port mode.
*/
typedef struct IOLM_SPortConfig
{
   /** \brief This parameter contains the requested cycle time for the OPERATE mode.
   
   If set to 0, mincycletime of the Device is used.
   To set a value in milliseconds there is the macro \ref IOL_CYCLE_MS.
   See Annex B.1.3 of \ref section_iol_spec_1_1. */
   INT8U u8ConfiguredCycleTime ;
   /** \brief This parameter indicates the requested operational mode of the port.
   
   COM1/COM2/COM3 as explicit mode is not allowed and only used for internal usage.
   */
   IOL_ETargetMode eTargetMode;
   /** \brief This parameter indicates the requested transmission rate.

   At the moment there is no check defined in specification.
   The Master always accepts all baud rates. */
   IOL_EBaudrate eConfiguredBaudrate;

   /** \brief Configured IO-Link revision ID.
   
   If set to 0, V1.1 is used.
   See Annex B.1.5 of \ref section_iol_spec_1_1. */
   INT8U u8ConfiguredRevisionID;
   /** \brief Device check during startup.
   
   Device inspection is only allowed in combination with #IOL_eTargetMode_CFGCOM.
   */
   IOLM_EInspectionLevel eInspectionLevel;
   /** \brief Configured Vendor ID.
   
   Needed if inspection level is TYPE_COMP or IDENTICAL. */
   INT8U au8ConfiguredVendorID[2];
   /** \brief Configured Device ID.
   
   Needed if Inspection level is TYPE_COMP or IDENTICAL.*/
   INT8U au8ConfiguredDeviceID[3];
   /** \brief Configured Function ID.
   
   Not yet defined by implemented specification. */
   INT8U au8ConfiguredFunctionID[2];
   /** \brief Configured Serial Number.
   
   Needed if Inspection Level is IDENTICAL. */
   INT8U au8ConfiguredSerialNumber[16];
   /** \brief Configured Serial Number length.
   
   Needed if inspection level is IDENTICAL. */
   INT8U u8ConfiguredSerialNumberLen;
} IOLM_SPortConfig;

/**
\brief This structure is used for the actual used communication parameters.
*/
typedef struct IOLM_SRealPortConfig 
{
   /** \brief This parameter indicates the real operational mode of the port. */
   IOL_ETargetMode eTargetMode;
   /** \brief This parameter indicates the real transmission rate.  */
   IOL_EBaudrate eRealBaudrate;
   /** \brief This parameter contains the real cycle time.
   See Annex B.1.3 of \ref section_iol_spec_1_1. */
   INT8U u8RealCycleTime ;
   /** \brief This parameter indicates the real IO-Link revision ID.
   See Annex B.1.5 of \ref section_iol_spec_1_1. */
   INT8U u8RealRevisionID;
   /** \brief This parameter indicates the real Vendor ID. */
   INT8U au8RealVendorID[2];
   /** \brief This parameter indicates the real Device ID. */
   INT8U au8RealDeviceID[3];
   /** \brief This parameter indicates the real Function ID. */
   INT8U au8RealFunctionID[2];
   /** \brief This parameter indicates the real Serial Number. */
   INT8U au8RealSerialNumber[16];
   /** \brief This parameter indicates the real Serial Number length. */
   INT8U u8RealSerialNumberLen;
   /** \brief Actual input data length in number of bytes. */
   INT8U u8PdInLength;
   /** \brief Actual output data length in number of bytes.  */
   INT8U u8PdOutLength;
} IOLM_SRealPortConfig;


#if IOLM_PD_BUFFER_COUNT > 1
typedef struct IOLM_SPDBufferCtrl
{
    volatile INT8U u8Valid;  /**< \brief Index of latest valid buffer. */
    volatile INT8U u8Writing; /**< \brief Current writing buffer. */
    volatile INT8U u8Reading; /**< \brief Index of last reading buffer. */

} IOLM_SPDBufferCtrl;
#endif

/**
\brief This structure is used for Process Data buffer handling.
*/
typedef struct IOLM_SPDBuffer
{
    INT8U au8Buffer[IOLM_PD_BUFFER_COUNT][IOLM_MAX_PD_SIZE];
#if IOLM_PD_BUFFER_COUNT > 1
    IOLM_SPDBufferCtrl suCtrl;
#endif 
    TBOOL boNewData; /**< \brief True if new data is available. */
} IOLM_SPDBuffer;

/**
\brief This structure stores the data of a page request.
*/
typedef struct IOLM_SPageRequest
{
    IOL_EDirection ePageDirection;
    INT8U u8PageAddress;
    INT8U au8PageData[16];
    INT8U u8PageLen;
    INT8U u8PageRequest; /**< \brief 1 if a page request is ready / 2 if pending / 0 if idle. */
}IOLM_SPageRequest;


/** \} */
#ifndef IOL_ONLY_TYPE_INCLUDE
#ifdef __cplusplus
}
#endif
#endif
#endif

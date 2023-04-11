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

#ifndef INC_PROT__IOLM_SMI_EXT_TYPES_H__
#define INC_PROT__IOLM_SMI_EXT_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif



#pragma pack(push, 1)


#define IOLM_FWU_SEGMENT_HEADER_SIZE_IN_BYTES   5
#define IOLM_FWU_SEGMENT_DATA_SIZE_IN_BYTES     (3*16)  ///< must be multiple of 16 (restriced by write to Flash methods)
                                                        ///< and total header size must be below 64 Bytes (USB CDC Virtual COM Port restriction)

/**
\brief This structure is used to identify firmware sub services.
*/
typedef IOL_ENUM_DECL IOLM_FW_ESubservice
{
    IOLM_FW_eSubservice_Unknown = 0,
    IOLM_FW_eSubservice_Header,
    IOLM_FW_eSubservice_Segment
}IOLM_FW_ESubservice;

/**
\brief This structure is used for firmware downloads.
*/
typedef struct IOLM_FW_SFWUHeader
{
    INT8U       u8SubService;           // IOLM_FW_ESubservice, 0=Header, 1=Segments
    INT8U       u8Reserved1;
    INT8U       u8Reserved2;
    INT8U       u8Reserved3;

    INT16U      u16Reserved4;
    INT16U      u16Reserved5;

    INT16U      u16Reserved6;
    INT16U      u16SegmentSizeInBytes;  // size is given in Bytes as segment header 
                                        //(5 Bytes) + firmware data chunks

    INT32U      u32NumberofSegments;

    INT32U      u32FWUSize;             // total size of binary in Bytes     
    INT32U      u32FWUCRC;

}IOLM_FW_SFWUHeader;

/**
\brief This structure is used for firmware downloads.
*/
typedef struct IOLM_FW_SFWUSegment
{
    INT8U       u8SubService;
    INT32U      u32SegmentNr;    /**< \brief Starting with 1. */

    INT8U       au8FWUData[IOLM_FWU_SEGMENT_DATA_SIZE_IN_BYTES];

}IOLM_FW_SFWUSegment;

/**
\brief This structure is used to identify configuration structures.
*/
typedef IOL_ENUM_DECL IOLM_ESettingsId
{
    IOLM_eSettingsId_Unknown = 0,
    IOLM_eSettingsId_RF,  // IOLM_Settings_RF
    IOLM_eSettingsId_ChanQuality, // Downlink Quality of each channel
    IOLM_eSettingsId_DevQuality, // Quality of Device
    IOLM_eSettingsId_DownlinkRssi, // Downlink Rssi (from IMA Message)
    IOLM_eSettingsId_UplinkRssi, // Uplink Rssi
    IOLM_eSettingsId_TestModeRX, // Start continuous RX test mode (Frequency as parameter) 
    IOLM_eSettingsId_TestModeTX, // Start continuous TX test mode (Frequency, TX Power & Preamble type as Parameter)
    IOLM_eSettingsId_TestModeSingleFreq, // Start normal operation in a single frequency mode
    IOLM_eSettingsId_Reset, // Reset the master to leave of the test mode
    IOLM_eSettingsId_AntGain,// back door for setting of the antenna gain
    IOLM_eSettingsId_WriteFWDesc, // write Firmware descriptor to flash
    IOLM_eSettingsId_ReadHWParam, // read a specific hardware parameter from flash
    IOLM_eSettingsId_TestHWParam, // set a specific hardware parameter for testing
    IOLM_eSettingsId_WriteHWParam,// write a specific hardware parameter to flash
    IOLM_eSettingsId_EndtesterControl, //special control service for Endtester board
    IOLM_eSettingsId_ModuleReset, // Reset the wireless module only to leave of the test mode
    IOLM_eSettingsId_ReadBLEMac, // BLE Mac address of the module (MM chip)
    IOLM_eSettingsId_ReadChipTemperature, // read the chip temperature (BAT Monitoring sensor) from MM
    IOLM_eSettingsId_ChanUplinkRssi, // Uplink Rssi of each channel
    IOLM_eSettingsId_ResetChanUplinkRssi, // Reset Cmd for Rssi of each channel
    IOLM_eSettingsId_SetFreqSpacing, // sets the frequency spacing (for a single track only)
    IOLM_eSettingsId_GetFreqSpacing, // reads the frequency spacing of the track
    IOLM_eSettingsId_AHTTest, // For testing of Adaptive Hopping Table (AHT)
}IOLM_ESettingsId;

typedef IOL_ENUM_DECL IOLM_EHWParamIdx
{
    IOLM_eHWParamIdx_PowerFactoryOffset = 0, // TX power factory offest calibrated at production time (8-bit signed)
    IOLM_eHWParamIdx_FrequencyOffset = 1, // Carrier Frequency offset at production time in kHz (8-bit signed)

}IOLM_EHWParamIdx;

/**
\brief This structure is used for settings of the RF chips.
*/
typedef struct IOLM_Settings_RF
{
    INT8U u8SettingsId; // IOLM_ESettingsId
    INT8U u8LogLevel; // 1 = log UL and DL
}IOLM_Settings_RF;



/**
\brief This structure is used to identify firmware sub services.
*/
typedef IOL_ENUM_DECL IOLM_ELogType
{
    IOLM_eLogType_Unknown = 0,
    IOLM_eLogType_WDownlink,
    IOLM_eLogType_WUplink,
    IOLM_eLogType_WiredTxFrame,
    IOLM_eLogType_WiredRxFrame,
    IOLM_eLogType_Settings,
}IOLM_ELogType;

/**
\brief This structure is used for data logging.
*/
typedef struct IOLM_DataLog
{
    INT8U u8Type; // IOLM_ELogType
    INT8U au8Payload[128];
} IOLM_DataLog;

/**
\brief This structure is used for downlink logging.
*/
typedef struct IOLM_DataLogDownlink
{
    INT8U u8Type;
    INT8U u8Channel;
    INT8U u8RSSI;
    INT8U au8Payload[43];
} IOLM_DataLogDownlink;

/**
\brief This structure is used for uplink logging.
*/
typedef struct IOLM_DataLogUplink
{
    INT8U u8Type;
    INT8U u8Slot;
    INT8U u8Channel;
    INT8U u8RSSI;
    INT8U au8Payload[16];
} IOLM_DataLogUplink;


/**
\brief This structure is used for stack information.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_StackInformation = 0x8003.
*/
typedef struct IOLM_SMI_SDataLog
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT16U u16TimeStamp; 
    IOLM_DataLog suPayload;
} IOLM_SMI_SDataLog;
#define SMI_DATALOG_HEADLEN (sizeof(IOLM_SMI_SDataLog)-sizeof(((IOLM_SMI_SDataLog *)0)->suPayload))

/**
\brief This structure is used for stack information.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_StackInformation = 0x8003.
*/
typedef struct IOLM_SMI_SStackInformation
{
    INT16U u16ArgBlockID; /**< \brief Big endian. */
    INT8U u8Version0;
    INT8U u8Version1;
    INT8U u8Version2;
    INT8U u8Version3;
    INT16U u16TagVersion;
    INT16U u16HWRev;
    INT32U u32Reserved;
} IOLM_SMI_SStackInformation;


/**
\brief This structure is used for FS Process Data input and output read back.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_FSPDIn = 0xB101.
*/
typedef struct IOLM_SMI_SFSPDIn
{
    INT16U u16ArgBlockID;       /**< \brief Big endian. */
    INT8U u8SDset;              /**< \brief Safe data settings applied. */
    INT8U u8Faults;             /**< \brief Faults. */
    INT8U u8OperatorAckRequest; /**< \brief Operator acknowledge request. */
    INT8U u8FSInputDataLength;  /**  \brief Input data length in bytes. */
    INT8U au8FSData[25];        /**< \brief 25 bytes FS PD. */
}IOLM_SMI_SFSPDIn;
#define IOLM_SMI_ARGBLOCK_FSPDIN_LEN(Datalen) (sizeof(IOLM_SMI_SFSPDIn) - 25 + Datalen)

/**
\brief This structure is used for FS Process Data output.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_FSPDOut = 0xB102.
*/
typedef struct IOLM_SMI_SFSPDOut
{
    INT16U u16ArgBlockID;       /**< \brief Big endian. */
    INT8U u8SetSD;              /**< \brief Safe data setting. */
    INT8U u8OperatorAck;        /**< \brief Operator acknowledge. */
    INT8U u8FSOutputDataLength; /**  \brief FS output data length in bytes. */
    INT8U au8FSData[25];        /**< \brief 25 bytes. */
}IOLM_SMI_SFSPDOut;
#define IOLM_SMI_ARGBLOCK_FSPDOUT_LEN(Datalen) (sizeof(IOLM_SMI_SFSPDOut) - 25 + Datalen)

/**
\brief This structure is used for Kunbus regression test support.

The ArgBlockID(#IOLM_SMI_EArgBlockID) for this struct is IOLM_SMI_eArgBlockID_TestCommand = 0xE002.
*/
typedef struct IOLM_SMI_STestCommand
{
    INT16U u16ArgBlockID;       /**< \brief Big endian. */
    INT16U u16Command;          /**< \brief Test command. */
    INT32U u32CommandData;      /**< \brief Data for command. */
}IOLM_SMI_STestCommand;

/**
\brief This enumeration defines the available test command codes.
*/
typedef IOL_ENUM_DECL IOLM_ETestCommandCode
{
    IOLM_ETestCommandCode_Unknown = 0,
    IOLM_ETestCommandCode_EnableRegTest,
    IOLM_ETestCommandCode_SMOperate,
    IOLM_ETestCommandCode_SetDSConfig,
    IOLM_ETestCommandCode_DeleteDS,
    IOLM_ETestCommandCode_GetPortStatus,
    IOLM_ETestCommandCode_SetSerialNumber
}IOLM_ETestCommandCode;


#pragma pack(pop)

#ifdef __cplusplus
}
#endif



#endif

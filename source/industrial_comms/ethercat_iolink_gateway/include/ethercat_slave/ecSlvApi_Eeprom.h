/*!
* \file ecSlvApi_Eeprom.h
*
* \brief
* EtherCAT slave Virtual EEPROM emulation.
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

#if !(defined __ECSLVAPI_EEPROM_H__)
#define __ECSLVAPI_EEPROM_H__		1

#include <osal.h>
#include <ecSlvApi.h>

#include "ecSlvApi_SyncMan.h"

#define EEPROM_CAT_HDR_LENGTH   0x04 //category header length in bytes

 //+---------------------------------------------------------------------------------------------
 //|		EEProm Structure
 //+---------------------------------------------------------------------------------------------


typedef enum EC_API_SLV_EEP_ECat_type
{
    EC_API_SLV_eCAT_STRINGS     = 10,               // 0x0A
    EC_API_SLV_eCAT_DATATYPES   = 20,               // 0x14
    EC_API_SLV_eCAT_GENERAL     = 30,               // 0x1E
    EC_API_SLV_eCAT_FMMU        = 40,				// 0x28
    EC_API_SLV_eCAT_SYNCM       = 41,				// 0x29
    EC_API_SLV_eCAT_TXPDO       = 50,				// 0x32
    EC_API_SLV_eCAT_RXPDO       = 51,				// 0x33
    EC_API_SLV_eCAT_DC          = 60,				// 0x3C
    EC_API_SLV_eCAT_END         = 0xFFFF
} OSAL_STRUCT_PACKED EC_API_SLV_EEP_ECat_type_t;


typedef struct EC_API_SLV_EEP_SCat_string
{
    EC_API_SLV_EEP_ECat_type_t	categoryType;       // 0x40
    uint16_t                    categoryWordSize;   // 0x41
    uint8_t                     numberOfStrings;    // 0x42..
} OSAL_STRUCT_PACKED EC_API_SLV_EEP_SCat_string_t;


typedef struct EC_API_SLV_EEP_SCat_General      // 0x1E
{
    EC_API_SLV_EEP_ECat_type_t	catType;
    uint16_t                    categoryWordSize;
    uint8_t                     groupIdx;		// 0x0000
    uint8_t                     imgIdx;			// 0x0001
    uint8_t                     orderIdx;		// 0x0002
    uint8_t                     nameIdx;		// 0x0003
    uint8_t                     reserved;		// 0x0004
    uint8_t                     coE_Details;	// 0x0005
    uint8_t                     foE_Details;	// 0x0006
    uint8_t                     eoE_Details;	// 0x0007
    uint8_t                     soE_Details;	// 0x0008
    uint8_t                     dS402Channels;	// 0x0009
    uint8_t                     sYSmanClass;	// 0x000a
    uint8_t                     flags;			// 0x000b
    uint16_t                    currentOnEBus;	// 0x000c EBus Current Consumption in mA
    uint8_t                     groupIdx2;      // 0x000e //duplicate for compatibility reasons
    uint8_t                     pAD_Byte1[1];	// 0x000f
    uint16_t                    physicalPort;	// 0x0010
    uint16_t                    physicalMemAddr;// 0x0012
    uint8_t                     pAD_Byte2[12];	// 0x0014

} OSAL_STRUCT_PACKED EC_API_SLV_EEP_SCat_General_t;


typedef struct EC_API_SLV_EEP_SCat_FMMU         // 0x28
{
    EC_API_SLV_EEP_ECat_type_t  catType;
    uint16_t                    categoryWordSize;
    uint8_t                     FMMU0;			// 0x0001
    uint8_t                     FMMU1;			// 0x0002
    uint8_t                     FMMU2;			// 0x0003
    uint8_t                     FMMU3;			// 0x0004
} OSAL_STRUCT_PACKED EC_API_SLV_EEP_SCat_FMMU_t;


typedef struct EC_API_SLV_EEP_SCat_SyncManEntry
{
    uint16_t                    startAddress;
    uint16_t                    length;
    uint8_t                     controlRegister;
    uint8_t                     statusRegister;
    uint8_t                     enableSynchManager;
    uint8_t                     syncManagerType;
} OSAL_STRUCT_PACKED EC_API_SLV_EEP_SCat_SyncManEntry_t;

typedef struct EC_API_SLV_EEP_SCat_SyncMan      // 0x29
{
    EC_API_SLV_EEP_ECat_type_t  catType;
    uint16_t                    categoryWordSize;
    EC_API_SLV_EEP_SCat_SyncManEntry_t	CatSYNCM[4];
} OSAL_STRUCT_PACKED EC_API_SLV_EEP_SCat_SyncMan_t;

typedef struct EC_API_SLV_EEP_SCat_DCEntry      // 0x29
{
    uint32_t                    cycleTimeSync0;
    uint32_t                    shiftTime0;
    uint32_t                    shiftTime1;
    int16_t                     sync1CycleFactor;
    uint16_t                    assignActivate;
    int16_t                     sync0CycleFactor;
    uint8_t                     nameIdx;
    uint8_t                     descIdx;
    uint8_t                     reserved[4];
} OSAL_STRUCT_PACKED EC_API_SLV_EEP_SCat_DCEntry_t;

typedef struct EC_API_SLV_EEP_SCat_DC           // 0x29
{
    EC_API_SLV_EEP_ECat_type_t      catType;    // 0x60
    uint16_t                        categoryWordSize;
    EC_API_SLV_EEP_SCat_DCEntry_t   catDC[2];
} OSAL_STRUCT_PACKED EC_API_SLV_EEP_SCat_DC_t;

 //this structure must not exceed the size of (EEPROM_SIZE- EE_PAGE_SIZE)
typedef struct EC_API_SLV_SEeprom
{
    uint16_t	pdiControl;                     // 0x0 PDI Control 0x0140:0x0141
    uint16_t	pdiConfiguration;				// 0x1 PDI Configuration 0x0150:0x0151

    uint16_t	syncImpulseLen;                 // 0x2 Pulse Length of SYNC Signals 0x0982:0x0983
    uint16_t	pdiConfiguration2;				// 0x3 Extended PDI Configuration 0x0152:0x0153

    uint16_t	station_alias;					// 0x4 Configured Station Alias 0x0012:0x0013
    uint16_t	reserved0[2];				    // 0x5:0x06 Reserved
    uint16_t	checksum;						// 0x7 Checksum + Pad Byte

    uint32_t	vendorID;						// 0x08:0x09 Kunbus 0x00000569
    uint32_t	productCode;					// 0x0A:0x0B
    uint32_t	revisionNo;						// 0x0C:0x0D
    uint32_t	serialNo;						// 0x0E:0x0F

                                                //according to ETG.2010_V1.0.0 and ETG.1000_6_V1.0.3 word addresses 0x10 to 0x13 are reserved
    uint16_t	executionDelay;					// 0x10	(Word Address !)
    uint16_t	port0Delay;						// 0x11
    uint16_t	port1Delay;						// 0x12
    uint16_t	reserved1;  					// 0x13

    uint16_t	bootstrapReceiveMailboxOffset;	// 0x14	(Word Address !)
    uint16_t	bootstrapReceiveMailboxSize;	// 0x15
    uint16_t	bootstrapSendMailboxOffset;		// 0x16
    uint16_t	bootstrapSendMailboxSize;		// 0x17

    uint16_t	standardReceiveMailboxOffset;	// 0x18
    uint16_t	standardReceiveMailboxSize;		// 0x19
    uint16_t	standardSendMailboxOffset;		// 0x1A
    uint16_t	standardSendMailboxSize;		// 0x1B
    uint16_t	mailboxProtocol;				// 0x1C Default: 0x0004

    uint16_t	reserved2[33];                  // 0x1D:0x3D: Should be 0

    uint16_t	kBitSize;                       // 0x3E: Size of EEPROM in (KBit - 1) e.g. 0x001F ^= 4096 Byte
    uint16_t	version;                        // 0x3F: Should be 1
} OSAL_STRUCT_PACKED EC_API_SLV_SEeprom_t;

//+=============================================================================================
//|		Prototypen / prototypes
//+=============================================================================================

#if (defined __cplusplus)
extern "C" {
#endif

extern void                     EC_API_SLV_EEPEMU_clean         (void);

extern void                     EC_API_SLV_EEPEMU_prepare       (EC_API_SLV_SHandle_t*  pEcSlaveApi_p);
extern void                     EC_API_SLV_EEPEMU_init          (EC_API_SLV_SHandle_t*  pEcSlaveApi_p);
extern int32_t                  EC_API_SLV_EEPEMU_reload        (void);
extern void                     EC_API_SLV_EEPEMU_commandAck    (void);
extern void                     EC_API_SLV_EEPEMU_flush         (void);
extern void                     EC_API_SLV_EEPEMU_exit          (void);

extern bool                     EC_API_SLV_EEP_addContent       (const uint8_t*         pData_p
                                                                ,uint32_t               length_p);
extern void                     EC_API_SLV_EEP_computeHeaderCRC (uint8_t*               pData_p
                                                                ,uint8_t                cnt_p
                                                                ,uint16_t*              pCrc_p);
extern EC_API_SLV_SEeprom_t*    EC_API_SLV_EEP_getHead          (void);
extern void                     EC_API_SLV_EEP_updateCRC        (void);
extern int32_t                  EC_API_SLV_EEP_loadEscRegisters (int32_t                reload_flag_p);
extern uint8_t                  EC_API_SLV_EEP_getUpdateStatus  (void);
extern uint32_t                 EC_API_SLV_EEP_getUpdatedTime   (void);
extern void                     EC_API_SLV_EEP_setUpdatedTime   (void);
extern void                     EC_API_SLV_EEP_setUpdateStatus  (uint8_t                status_p);
extern void                     EC_API_SLV_EEP_loadFromMemory   (void*                  pEeprom_p
                                                                ,uint32_t               len_p);
extern void                     EC_API_SLV_EEP_getEeprom        (void**                 pEeprom_p
                                                                ,uint32_t*              pLen_p);
#if (defined __cplusplus)
}
#endif

#endif /* __ECSLVAPI_EEPROM_H__ */


/*!
* \file ecSlvApiDef_AL_Codes.h
*
* \brief
* EtherCAT Application Layer status codes.
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

#if !(defined __ECSLVAPIDEF_AL_CODES_H__)
#define __ECSLVAPIDEF_AL_CODES_H__		1



#if (defined __cplusplus)
extern "C" {
#endif


/*-----------------------------------------------------------------------------------------
------
------    Defines and Types
------
-----------------------------------------------------------------------------------------*/

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

#if (defined __cplusplus)
}
#endif

#endif /* __ECSLVAPIDEF_AL_CODES_H__ */

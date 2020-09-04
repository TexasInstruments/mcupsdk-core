/*!
* \file ecSlvApi.h
*
* \brief
* EtherCAT User API interface.
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

#if !(defined __ECSLVAPI_H__)
#define __ECSLVAPI_H__		1

#if (defined ECATSLAVE_SO) && (ECATSLAVE_SO==1) // defined if ECATSLV is compiled as a DLL
 #ifdef ECSLVAPI_EXPORTS // defined if we are building the ECATSLAVE DLL (instead of using it)
  #define ECATSLV_API OSAL_DLL_EXPORT
 #else
  #define ECATSLV_API OSAL_DLL_IMPORT
 #endif // ECSLVAPI_EXPORTS
 #define ECATSLV_LOC OSAL_DLL_LOCAL
#else // ECATSLAVE_SO is not defined: this means ECATSLAVE is a static lib.
 #define ECATSLV_API
 #define ECATSLV_LOC
#endif // ECATSLAVE_SO

#include <osal.h>
#include <ecSlvApi_Error.h>

#if !(defined FBTL_REMOTE) && !(defined DPRAM_REMOTE)
#include <custom_phy.h>
#endif

/** \addtogroup EC_API_SLV_TYPES
*  @{
*/

/*!
 *  @brief    PRUICSS Instance IDs
 */
/* also not all MCUs have 3 PRUs define those as Max usage */
typedef enum EC_API_SLV_EPRUICSS_MaxInstances
{
    EC_API_SLV_ePRUICSS_INSTANCE_ONE=1,
    EC_API_SLV_ePRUICSS_INSTANCE_TWO=2,
    EC_API_SLV_ePRUICSS_INSTANCE_THREE=3,
    EC_API_SLV_ePRUICSS_INSTANCE_MAX=4
} EC_API_SLV_EPRUICSS_MaxInstances_t;

/// EC_RETCODE_T  Error codes used during EtherCAT State Machine transitions.
typedef enum EC_API_SLV_EUserRetCodeserRetCodes
{
    EC_USR_eRET_OK = 0,                     ///< no error occurred
    EC_USR_eRET_ERROR,                      ///< Unspecified error occurred
    EC_USR_eRET_NOERROR_INWORK              ///< Indication for no error but operation is pending
} EC_API_SLV_EUserRetCodes_t;

/// EC_STATE_T EtherCAT State Machine states.
typedef enum EC_API_SLV_EEsmState
{
    EC_API_SLV_eESM_uninit    = 0x00,           ///< Uninitialized State
    EC_API_SLV_eESM_init      = 0x01,           ///< Init State
    EC_API_SLV_eESM_preop     = 0x02,           ///< PreOP State
    EC_API_SLV_eESM_bootstrap = 0x03,           ///< BootStrap State
    EC_API_SLV_eESM_safeop    = 0x04,           ///< SafeOP State
    EC_API_SLV_eESM_op        = 0x08,           ///< Operational State
    EC_API_SLV_eESM_errState  = 0x10,           ///< Error Flag
    EC_API_SLV_eESM_errDevId  = 0x20,           ///< Explicit Device ID
} EC_API_SLV_EEsmState_t;

/// EC_API_SLV_EDevType_t EtherCAT Device Type. Object Dictionary Index 0x1001
typedef enum EC_API_SLV_EDevType
{
    EC_API_SLV_eDT_DEFAULT_DEVICE           = 0x00000000,   ///< The device does not support any specific profile
    EC_API_SLV_eDT_MODULAR_DEVICE           = 0x00001389,   ///< The device is a modular device.
    EC_API_SLV_eDT_SERVO_DRIVE              = 0x00020192,   ///< The device supports CiA402 for a servo drive.
    EC_API_SLV_eDT_STEPPER_DRIVE            = 0x00040192,   ///< The device supports CiA402 for a stepper drive.
    EC_API_SLV_eDT_SERVO_SAFETY_PROFILE     = 0x000A0192,   ///< The device supports CiA402 for a servo drive with safety features.
    EC_API_SLV_eDT_STEPPER_SAFETY_PROFILE   = 0x000C0192,   ///< The device supports CiA402 for a stepper drive with safety features.
    EC_API_SLV_eDT_MDP_GATEWAY_IOLINK       = 0x184c1389,   ///< The device is a gateway to IO Link master
} EC_API_SLV_EDevType_t;

/// EC_DEVERROR_T Error register values. Object Dictionary Index 0x1001.
typedef enum EC_API_SLV_EErrorRegister
{
    EC_API_SLV_eERR_NO_ERROR                 = 0x00,     ///< No error
    EC_API_SLV_eERR_GENERIC_ERROR            = 0x01,     ///< Generic error
    EC_API_SLV_eERR_CURRENT_ERROR            = 0x02,     ///< Current error
    EC_API_SLV_eERR_VOLTAGE_ERROR            = 0x04,     ///< Voltage error
    EC_API_SLV_eERR_TEMPERATURE_ERROR        = 0x08,     ///< Temperature error
    EC_API_SLV_eERR_COMMUNICATION_ERROR      = 0x10,     ///< Communication error
    EC_API_SLV_eERR_DEVICE_PROFILE_ERROR     = 0x20,     ///< Profile error
    EC_API_SLV_eERR_MANUFACTURER_ERROR       = 0x80,     ///< Manufacturer error
} EC_API_SLV_EErrorRegister_t;

/*!
 *  @brief    Phy Index
 */
/* also not all MCUs have 3 PRUs define those as Max usage */
typedef enum EC_API_SLV_EPhy_Index
{
    EC_API_SLV_ePHY_IN                      = 0,        ///< Phy index for IN Phy
    EC_API_SLV_ePHY_OUT                     = 1,        ///< Phy index for OUT Phy
} EC_API_SLV_EPhy_Index_t;


/// TEntry describes an Entry of a PDO Mapping.
typedef struct EC_API_SLV_SPdoEntry EC_API_SLV_SPdoEntry_t;

/// TPdo is used to describe the RxPDOs and TxPDOs
typedef struct EC_API_SLV_SPdo EC_API_SLV_SPdo_t;

/// TSdoEntry describes an OBD Object Entry
typedef struct EC_API_SLV_SCoE_ObjEntry EC_API_SLV_SCoE_ObjEntry_t;

/// TSdo describes an Object Dictionary Object
typedef struct EC_API_SLV_SCoE_Object EC_API_SLV_SCoE_Object_t;

/// EC_API_SLV_SHandle_t describes the EtherCAT Slave API
typedef struct EC_API_SLV_SHandle EC_API_SLV_SHandle_t;

/** @}*/

/** \addtogroup EC_API_SLV_APPCB
*  @{
*/

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User defined function. Called as application MainLoop in MainLoop.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppMainLoopCtxt_p     CallBack Content for use in complex applications.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void (*EC_API_SLV_CBUsrApplicationMainLoop_t)(void* pAppMainLoopCtxt_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User defined function. Called as "application run" in MainLoop.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppRunCtxt_p		CallBack Content for use in complex applications.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void (*EC_API_SLV_CBUsrApplicationRun_t)(void* pAppRunCtxt_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Called on severe Stack error (OSAL_error)
 *
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppRunCtxt_p   CallBack Content for use in complex applications.
 *  \param[in]  errorCode_p		Error code from DTK Stack
 *  \param[in]  fatal_p         Error type
 *  \param[in]  numOfParam_p	number of upcoming parameters
 *  \param[in]  arg_p           rest of parameters
 *
 *  <!-- Reference -->
 *  \sa OSAL_error
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void(*EC_API_SLV_CBStackError_t)(void* pAppRunCtxt_p, uint32_t errorCode_p, uint8_t fatal_p, uint8_t numOfParam_p, va_list arg_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Called on severe Stack error (OSAL_error)
 *
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pMsrmtCtxt_p        CallBack context
 *  \param[in]  measureChannel_p    measure channel code
 *  \param[in]  channelOn_p         true: channel raise, false: channel lower
 *
 *  <!-- Reference -->
 *  \sa OSAL_error
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void(*EC_API_SLV_CBMsrmt_t)(void* pMsrmtCtxt_p, uint32_t measureChannel_p, bool channelOn_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User defined function. Called when mailbox handler is started.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pContext_p            Function context.
 *  \return EC_USR_CB_ERetCode Returns the Callback error code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef EC_API_SLV_EUserRetCodes_t (*EC_API_SLV_CBStartMbxHandler_t)(void* pContext_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User defined function. Called when mailbox handler is stopped.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p            Function context.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void (*EC_API_SLV_CBStopMbxHandler_t)(void* pContext_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User defined function. Called when input handler is started.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      Function context.
 *  \param[in]  pIntMask_p      Value which will be written to the 0x204 register (AL event mask) during the state transition PreOP to SafeOP.
 *  \return EC_USR_CB_ERetCode Returns the Callback error code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef EC_API_SLV_EUserRetCodes_t (*EC_API_SLV_CBStartInputHandler_t)(void* pContext_p, uint16_t *pIntMask_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User defined function. Called when input handler is stopped.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pContext_p      Function context.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void (*EC_API_SLV_CBStopInputHandler_t)(void* pContext_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User defined function. Called when output handler is started.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pContext_p      Function context.
 *  \return EC_USR_CB_ERetCode Returns the Callback error code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef EC_API_SLV_EUserRetCodes_t (*EC_API_SLV_CBStartOutputHandler_t)(void* pContext_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User defined function. Called when ouput handler is stopped.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pContext_p      Function context.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void (*EC_API_SLV_CBStopOutputHandler_t)(void* pContext_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Register external Input Process Data Buffer.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pContext_p      Function context.
 *  \param[in]  length_p        length of Input PD
 *  \return void pointer to Input Process Data.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void*(*EC_API_SLV_CBPreSeqInputPD_t)(void* pContext_p, uint32_t length_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Register external Output Process Data Buffer.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pContext_p      Function context.
 *  \param[in]  length_p        length of Output PD
 *  \return void pointer to Input Process Data.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void*(*EC_API_SLV_CBPreSeqOutputPD_t)(void* pContext_p, uint32_t length_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Register external Release Input Process Data Buffer Function.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pContext_p      Function context.
 *  \param[in]  pData_p         Input PD
 *  \param[in]  length_p        length of Input PD
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void (*EC_API_SLV_CBPostSeqInputPD_t)(void* pContext_p, void* pData_p, uint32_t length_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Register external Release Output Process Data Buffer Function.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pContext_p      Function context.
 *  \param[in]  pData_p         Output PD
 *  \param[in]  length_p        length of Output PD
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void (*EC_API_SLV_CBPostSeqOutputPD_t)(void* pContext_p, void* pData_p, uint32_t length_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback trigered by a SDO Upload operation.
 *
 *  \details
 *  The function is called when the EtherCAT Master sends a object read request. If no callback function is registered then the value contained in the object is returned.
 *  If a callback function is registered it is the reponsability of the callback function to update the value pointed to by \p pData. \p pData is returned within
 *  the SDO Response datagram.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p          function context
 *  \param[in]  index_p             Index of the requested object.
 *  \param[in]  subindex_p          SubIndex of the requested object.
 *  \param[in]  length_p            Length of the received data buffer.
 *  \param[in]  pData_p             Pointer to the data buffer.
 *  \param[in]  completeAccess_p    Read all subIndexes of the object.
 *  \return result of the upload operation. Set 0 if operation succeeded or an abort code declared on the Beckhoff Slave Stack (ABORTIDX_ in sdoserv.h).
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef uint8_t(*EC_API_SLV_CBObjRead_t)(void* pContext_p, uint16_t index_p, uint8_t subindex_p, uint32_t length_p, uint16_t* pData_p, uint8_t completeAccess_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback trigered by a SDO Download operation.
 *
 *  \details
 *  The function is called when the EtherCAT Master sends a object write request. If no callback function is registered then the value pointed to by \p pData is
 *  directly written to the object entry. If a callback function is registered it is the reponsability of the callback function to copy the value pointed to by \p pData
 *  to object entry.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p          function context
 *  \param[in]  index_p             Index of the requested object.
 *  \param[in]  subindex_p          SubIndex of the requested object.
 *  \param[in]  length_p            Length of the received data buffer.
 *  \param[in]  pData_p             Pointer to the data buffer.
 *  \param[in]  completeAccess_p    Write all subIndexes of the object.
 *  \return     result of the download operation. Set 0 if operation succeeded or an abort code declared on the Beckhoff Slave Stack (ABORTIDX_ in sdoserv.h).
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef uint8_t(*EC_API_SLV_CBObjWrite_t)(void* pContext_p, uint16_t index_p, uint8_t subindex_p, uint32_t length_p, uint16_t* pData_p, uint8_t completeAccess_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User defined EoE receive function. Called when an EoE frame is received.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p          function context
 *  \param[in]  pData_p             EoE Frame Data
 *  \param[in]  length_p            EoE Frame Size
 *  \return true if frame is handle, false if it should be passed on.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef bool (*EC_API_SLV_EoE_CBReceiveHandler_t)(void* pContext_p, uint16_t* pData_p, uint16_t length_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  EoE Settings Indication callback proxy
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p          The pointer to the EtherCAT API instance.
 *  \param[in]  pMac_p              Virtual Net MAC address
 *  \param[in]  pIp_p               Virtual Net IP address
 *  \param[in]  pSubNet_p           Virtual Net Subnet
 *  \param[in]  pDefaultGateway_p   Virtual Net Default Gateway
 *  \param[in]  pDnsIp_p            Virtual Net DNS server address
 *  \return     true if settings are handled, false otherwise
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef bool (*EC_API_SLV_EoE_CBSettingIndHandler_t)(void* pContext_p, uint16_t *pMac_p, uint16_t* pIp_p, uint16_t* pSubNet_p, uint16_t* pDefaultGateway_p, uint16_t* pDnsIp_p );

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  AoE Communication Request Handler.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p          The pointer to the EtherCAT API instance.
 *  \param[in]  port_p              AMS port.
 *  \param[in]  cmdId_p             ADS Request ID: read, write or read/write.
 *  \param[in]  index_p             16 bit index value from IndexOffset.
 *  \param[in]  subIndex_p          8 bit subIndex value from IndexOffset.
 *  \param[in]  completeAccess_p    CoE Complete Access flag.
 *  \param[in]  dataLen_p           Requested data length to read or write.
 *  \return     ADS error code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef uint16_t (*EC_API_SLV_AoE_CBRquestHandler_t)(void* pContext_p, uint16_t port_p, uint16_t cmdId_p, uint16_t index_p, uint8_t subIndex_p, bool completeAccess_p, uint32_t dataLen_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User defined FoE receive function. Called when an FoE frame is received.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      call context
 *  \param[in]  pName_p         FoE file name
 *  \param[in]  nameLen_p       Length of file name
 *  \param[in]  isRead_p        true: readAccess, false: writeAccess
 *  \param[in]  password_p      password
 *  \return Error code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef uint32_t (*EC_API_SLV_FoE_CBOpenFileHandler_t)(void* pContext_p, const char* pName_p, uint16_t nameLen_p, bool isRead_p, uint32_t password_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Read FoE File callback.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p              call context
 *  \param[in]  pData_p                 FoE file data
 *  \param[in]  foeMaxSendBlockSize_p   Max SendBlock Size
 *  \param[in]  offset_p                FoE file data offset
 *  \return Error Code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef uint32_t(*EC_API_SLV_FoE_CBReadFileHandler_t)(void* pContext_p, uint16_t* pData_p, uint16_t foeMaxSendBlockSize_p, uint32_t offset_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Write FoE File callback.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p  call context
 *  \param[in]  pData_p     FoE file data
 *  \param[in]  size_p      Size of FoE data
 *  \return Error Code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef uint32_t(*EC_API_SLV_FoE_CBWriteFileHandler_t)(void* pContext_p, uint16_t* pData_p, uint16_t size_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Close FoE File callback.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      call context
 *  \param[in]  errorCode_p     FoE close error code
 *  \return Error Code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef uint32_t(*EC_API_SLV_FoE_CBCloseFileHandler_t)(void* pContext_p, uint32_t errorCode_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  FoE Handle Firmware update callback.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      call context
 *  \return Error Code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef EC_API_EError_t(*EC_API_SLV_FoE_CBFwUpdateHandler_t)(void* pContext_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Handle System Reboot.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pContext_p      call context
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void(*EC_API_SLV_CBSystemRebootHandler_t)(void* pContext_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  AoE Communication Read Request Handler.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]      pContext_p          The pointer to the EtherCAT API instance.
 *  \param[in]      port_p              AMS port.
 *  \param[in]      index_p             16 bit index value from IndexOffset.
 *  \param[in]      subIndex_p          8 bit subIndex value from IndexOffset.
 *  \param[in]      completeAccess_p    CoE Complete Access flag.
 *  \param[in,out]  pLength_p           Request data length.
 *  \param[in]      pData_p             Pointer to data.
 *  \return     ADS error code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef uint16_t (*EC_API_SLV_AoE_CBReadRequestHandler_t)(void*         pContext_p,
                                                          uint16_t      port_p,
                                                          uint16_t      index_p,
                                                          uint8_t       subIndex_p,
                                                          bool          completeAccess_p,
                                                          uint32_t*     pLength_p,
                                                          uint16_t*     pData_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  AoE Communication Write Request Handler.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]      pContext_p          The pointer to the EtherCAT API instance.
 *  \param[in]      port_p              AMS port.
 *  \param[in]      index_p             16 bit index value from IndexOffset.
 *  \param[in]      subIndex_p          8 bit subIndex value from IndexOffset.
 *  \param[in]      completeAccess_p    CoE Complete Access flag.
 *  \param[in,out]  pLength_p           Request data length.
 *  \param[in]      pData_p             Pointer to data.
 *  \return     ADS error code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef uint16_t (*EC_API_SLV_AoE_CBWriteRequestHandler_t)(void*           pContext_p,
                                                           uint16_t        port_p,
                                                           uint16_t        index_p,
                                                           uint8_t         subIndex_p,
                                                           bool            completeAccess_p,
                                                           uint32_t*       pLength_p,
                                                           uint16_t*       pData_p);
/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Send SoE message to master.
 *
 *  \details
 *  This function is called when a ServoDriveProfile over EtherCAT service has to be sent to the master.
 *  The DTK Stack collects the Service, Flags and data to send from the user application.
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      call context
 *  \param[in]  pSoEService_p   SoE Service code
 *  \param[in]  pSoEFlags_p		SoE Flags
 *  \param[in]  pData_p         SoE Data
 *  \param[in]  pLen_p          Data length
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void (*EC_API_SLV_SoE_CBSend_t)(void* pContext_p, uint16_t* pSoEService_p, uint16_t* pSoEFlags_p, void** pData_p, uint16_t* pLen_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Recevice SoE message from Master.
 *
 *  \details
 *  This function is called when a ServoDriveProfile over EtherCAT service is received from the master.
 *  DTK Stack calls the function with the parameters received from the master. The application should response to the service.
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      call context
 *  \param[in]  soEService_p    SoE Service code
 *  \param[in]  soEFlags_p		SoE Flags
 *  \param[in]  pData_p         SoE Data
 *  \param[in]  pLen_p          Data length
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void(*EC_API_SLV_SoE_CBRecv_t)(void* pContext_p, uint16_t soEService_p, uint16_t soEFlags_p, void* pData_p, uint16_t *pLen_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Master requests a notification. Application should send an Emergency message.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      call context
 *  \return     ErrorCode       Mailbox error code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef uint8_t(*EC_API_SLV_SoE_CBNofReq_t)(void* pContext_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User defined function. Called as "application Cia402 StateMachine" in MainLoop.
 *
 *  \details
 *  If function is not provided, the DTK uses an internal CiA402 State Machine.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppRunCtxt_p   CallBack Content for use in complex applications.
 *  \return     bool            true if statemachine handled, false handle in stack
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef bool (*EC_API_SLV_CiA402_CBUsrApplSM_t)(void* pAppRunCtxt_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Configure CiA402 Object Dictionary.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p		call context.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef uint16_t (*EC_API_SLV_CiA402_CBUsrApplSetDict_t)(void* pContext_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Set default values for the CiA402 Objects.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p		call context.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void (*EC_API_SLV_CiA402_CBUsrApplSetDictValues_t)(void* pContext_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User application slot (control)
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p		call context.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void (*EC_API_SLV_CiA402_CBUsrApplApplication_t)(void* pContext_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User application local error
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p		Parameter description.
 *  \param[in]  errorCode_p     Local Error
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void (*EC_API_SLV_CiA402_CBUsrApplLocalError_t)(void* pContext_p, uint16_t errorCode_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Load EEPROM from memory.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      call context
 *  \param[in]  pEeprom_p       Eeprom data loaded from memory
 *  \param[in]  pLength_p       Eeprom Length read from memory
 *  \return     bool            true on successfull load
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef bool(*EC_API_SLV_CBEepromLoad_t)(void* pContext_p, void* pEeprom_p, uint32_t* pLength_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Write EEPROM to memory.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      call context
 *  \param[in]  pEeprom_p		Eeprom Content.
 *  \param[in]  length_p        Eeprom Length.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void(*EC_API_SLV_CBEepromWrite_t)(void* pContext_p, void* pEeprom_p, uint32_t length_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Init flash memory block to load and store eeprom content.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pContext_p  call context
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void(*EC_API_SLV_CBInitFlash_t)(void* pContext_p);
/** @}*/

/** \addtogroup EC_API_SLV_BOARD
*  @{
*/

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Set Status LED call back
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      call context.
 *  \param[in]  pLedContext_p   LED context.
 *  \param[in]  errLed_p        state of error LED
 *  \param[in]  statusLed_p     state of status LED.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
typedef void(*EC_API_SLV_CBBoardStatusLed_t)(void* pContext_p, void* pLedContext_p, bool errLed_p, bool statusLed_p);

/** @}*/

#if (defined __cplusplus)
extern "C" {
#endif

#if (defined  __TI_COMPILER_VERSION__) || (defined OSAL_FREERTOS_JACINTO)
extern ECATSLV_API  void                    EC_API_SLV_initStub                             (int32_t                        irqBaseOffset_p);
#endif

extern ECATSLV_API  void                    EC_API_SLV_SSC_setLicense                       (char*                          pLicense_p
                                                                                            ,uint8_t                        length_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_getVersion                           (void);
extern ECATSLV_API  uint32_t                EC_API_SLV_getVersionStr                        (uint32_t                       bufLen_p
                                                                                            ,char*                          pBuffer_p
                                                                                            ,uint32_t*                      pUsedLen_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_getVersionId                         (uint32_t                       bufLen_p
                                                                                            ,char*                          pBuffer_p
                                                                                            ,uint32_t*                      pUsedLen_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_DPR_configuration                    (uint8_t*                       pDprBase_p
                                                                                            ,uint32_t                       dprSize_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_FBTL_configuration                   (void*                          pFbtlHandle_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_load                                 (OSAL_PJumpBuf_t*               pExceptionPoint_p
                                                                                            ,OSAL_ERR_CBHandler_t           cbErrHandler_p
                                                                                            ,uint32_t                       pruSelect_p);
extern ECATSLV_API  void                    EC_API_SLV_unLoad                               (void);
extern ECATSLV_API  void                    EC_API_SLV_prepareTasks                         (OSAL_TASK_EPriority_t          pdiTaskPrio_p
                                                                                            ,OSAL_TASK_EPriority_t          statusLEDTaskPrio_p
                                                                                            ,OSAL_TASK_EPriority_t          sync0TaskPrio_p
                                                                                            ,OSAL_TASK_EPriority_t          sync1TaskPrio_p
                                                                                            ,OSAL_TASK_EPriority_t          eoeTaskPrio_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_stackInit                            (void);
extern ECATSLV_API  EC_API_SLV_SHandle_t*   EC_API_SLV_new                                  (void);
extern ECATSLV_API  uint32_t                EC_API_SLV_obdCleanUp                           (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_delete                               (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_init                                 (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_run                                  (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_stop                                 (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_reset                                (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterErrorHandler               (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBStackError_t      cbFunc_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_enableMdioManualMode                 (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint32_t                       manualMdioAddress_p);


extern ECATSLV_API  uint32_t                EC_API_SLV_ESI_parseFile                        (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,const char*                    pFilename_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_ESI_parseFileRam                     (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,const uint8_t*                 pData_p
                                                                                            ,uint32_t                       length_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setDeviceType                        (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_EDevType_t          devType_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setErrorRegister                     (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_EErrorRegister_t    errorRegister_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setVendorId                          (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint32_t                       vendorId_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setProductCode                       (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint32_t                       productCode_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setRevisionNumber                    (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint32_t                       revisionNumber_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setSerialNumber                      (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint32_t                       serialNumber_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setProductName                       (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,char*                          pProductName_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setGroupType                         (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,char*                          pGroupType_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setHwVersion                         (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,char*                          pHardwareVersion_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setSwVersion                         (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,char*                          pSoftwareVersion_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setPDOSize                           (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint32_t                       maxPdOutSize_p
                                                                                            ,uint32_t                       maxPdInSize_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setPDICfg                            (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       pdiControl_p
                                                                                            ,uint16_t                       pdiConfig_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setBootStrapMailbox                  (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       rxOffset_p
                                                                                            ,uint16_t                       rxSize_p
                                                                                            ,uint16_t                       txOffset_p
                                                                                            ,uint16_t                       txSize_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setStandardMailbox                   (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       rxOffset_p
                                                                                            ,uint16_t                       rxSize_p
                                                                                            ,uint16_t                       txOffset_p
                                                                                            ,uint16_t                       txSize_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setSyncManConfig                     (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint8_t                        syncManIdx_p
                                                                                            ,uint16_t                       offset_p
                                                                                            ,uint16_t                       size_p
                                                                                            ,uint8_t                        control_p
                                                                                            ,uint8_t                        enable_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setSyncManErrLimit                   (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       smErrorLimit_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_getVendorId                          (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_getProductCode                       (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_getRevisionNumber                    (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_getSerialNumber                      (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  char*                   EC_API_SLV_getProductName                       (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  char*                   EC_API_SLV_getGroupType                         (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  char*                   EC_API_SLV_getSwVersion                         (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  char*                   EC_API_SLV_getHwVersion                         (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);

extern ECATSLV_API  void                    EC_API_SLV_cbRegisterUserApplicationMainLoop    (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBUsrApplicationMainLoop_t
                                                                                                                            cbFunc_p
                                                                                            ,void*                          pApplContext_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterUserApplicationRun         (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBUsrApplicationRun_t
                                                                                                                            cbFunc_p
                                                                                            ,void*                          pApplCtxt_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterStartMbxHandler            (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBStartMbxHandler_t cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterStopMbxHandler             (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBStopMbxHandler_t  cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterStartInputHandler          (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBStartInputHandler_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterStopInputHandler           (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBStopInputHandler_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterStartOutputHandler         (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBStartOutputHandler_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterStopOuputHandler           (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pcontext_p
                                                                                            ,EC_API_SLV_CBStopOutputHandler_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_EEPROM_cbRegisterLoad                (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBEepromLoad_t      cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_EEPROM_cbRegisterWrite               (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBEepromWrite_t     cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterFlashInit                  (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBInitFlash_t       cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterMeasurement                (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBMsrmt_t           cbFunc_p);

extern ECATSLV_API  void                    EC_API_SLV_cbRegisterPreSeqInputPDBuffer        (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBPreSeqInputPD_t   cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterPreSeqOutputPDBuffer       (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBPreSeqOutputPD_t  cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterPostSeqInputPDBuffer       (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBPostSeqInputPD_t  cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterPostSeqOutputPDBuffer      (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBPostSeqOutputPD_t cbFunc_p);

extern ECATSLV_API  uint32_t                EC_API_SLV_preSeqInputPDBuffer                  (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,void**                         ppInProcData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_preSeqOutputPDBuffer                 (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,void**                         ppOutProcData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_postSeqInputPDBuffer                 (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,void*                          pInProcData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_postSeqOutputPDBuffer                (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,void*                          pOutProcData_p);

extern ECATSLV_API  void                    EC_API_SLV_EoE_cbRegisterReceiveHandler         (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_EoE_CBReceiveHandler_t
                                                                                                                            cbFunc_p);

extern ECATSLV_API  void                    EC_API_SLV_EoE_cbRegisterSettingIndHandler      (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_EoE_CBSettingIndHandler_t
                                                                                                                            cbFunc_p);

extern ECATSLV_API  void                    EC_API_SLV_FoE_cbRegisterOpenFileHandler        (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_FoE_CBOpenFileHandler_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_FoE_cbRegisterReadFileHandler        (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_FoE_CBReadFileHandler_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_FoE_cbRegisterWriteFileHandler       (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_FoE_CBWriteFileHandler_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_FoE_cbRegisterCloseFileHandler       (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_FoE_CBCloseFileHandler_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_FoE_cbRegisterFwUpdate               (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_FoE_CBFwUpdateHandler_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterSystemReboot               (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBSystemRebootHandler_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_setFwUpdateMode                      (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,bool                           fwUpdateRequired_p);

extern ECATSLV_API void                     EC_API_SLV_AoE_cbRegisterReadRequestHandler     (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_AoE_CBReadRequestHandler_t
                                                                                                                            cbFunc_p);

extern ECATSLV_API void                     EC_API_SLV_AoE_cbRegisterWriteRequestHandler    (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_AoE_CBWriteRequestHandler_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_SoE_cbRegisterSendHandler            (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_SoE_CBSend_t         cbFunc_p);

extern ECATSLV_API  void                    EC_API_SLV_SoE_cbRegisterRecvHandler            (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_SoE_CBRecv_t         cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_SoE_cbRegisterNotificationReqHandler (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_SoE_CBNofReq_t       cbFunc_p);
extern ECATSLV_API  uint8_t                 EC_API_SLV_notificationResponse                 (uint16_t                       idn_p
                                                                                            ,uint16_t                       channel_p
                                                                                            ,uint16_t                       dataState_p);

extern ECATSLV_API  void                    EC_API_SLV_CiA402_setAxisNumber                 (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint8_t                        axisNo_p);
extern ECATSLV_API  void                    EC_API_SLV_CiA402_activateAxis                  (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint8_t                        axisNo_p
                                                                                            ,bool                           active_p);
extern ECATSLV_API  void                    EC_API_SLV_CiA402_SM_clearErrorCode             (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint8_t                        axisNo_p);
extern ECATSLV_API  uint16_t                EC_API_SLV_CiA402_SM_getErrorCode               (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint8_t                        axisNo_p);
extern ECATSLV_API  void                    EC_API_SLV_CiA402_registerSetDictionary         (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CiA402_CBUsrApplSetDict_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_CiA402_registerSetDictValues         (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CiA402_CBUsrApplSetDictValues_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_CiA402_registerStateMachine          (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CiA402_CBUsrApplSM_t cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_CiA402_registerApplication           (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CiA402_CBUsrApplApplication_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  void                    EC_API_SLV_CiA402_registerLocalError            (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CiA402_CBUsrApplLocalError_t
                                                                                                                            cbFunc_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_create                           (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,char*                          pName_p
                                                                                            ,uint16_t                       mapIndex_p
                                                                                            ,EC_API_SLV_SPdo_t**            pOutPdo_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_get                              (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       index_p
                                                                                            ,EC_API_SLV_SPdo_t**            pOutPdo_p);
extern ECATSLV_API  uint16_t                EC_API_SLV_PDO_getOffset                        (EC_API_SLV_SPdo_t*             pPdo_p);
extern ECATSLV_API  uint16_t                EC_API_SLV_PDO_getLength                        (EC_API_SLV_SPdo_t*             pPdo_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_createEntry                      (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SPdo_t*             pPdo_p
                                                                                            ,char*                          pName_p
                                                                                            ,EC_API_SLV_SCoE_ObjEntry_t*    pObjectEntry_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_getInputProcDataLength               (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_getOutputProcDataLength              (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  uint8_t                 EC_API_SLV_PDO_getEntryCount                    (EC_API_SLV_SPdo_t*             pPdo_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_getEntryDataLength               (EC_API_SLV_SPdo_t*             pPdo_p
                                                                                            ,uint8_t                        subIndex_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setInputData                         (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,uint8_t*                       pInputData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_getOutputData                        (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,uint8_t*                       pOutputData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_setData                          (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SPdo_t*             pPdo_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,uint8_t*                       pData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_getData                          (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SPdo_t*             pPdo_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,uint8_t*                       pData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_setEntryData                     (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SPdo_t*             pPdo_p
                                                                                            ,uint8_t                        subIndex_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,uint8_t*                       pData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_getEntryData                     (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SPdo_t*             pPdo_p
                                                                                            ,uint8_t                        subIndex_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,uint8_t*                       pData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_setFixedMapping                  (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,bool                           fixed_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_enable                           (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SPdo_t*             pPdo_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_disable                          (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SPdo_t*             pPdo_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_enabled                          (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SPdo_t*             pPdo_p
                                                                                            ,bool*                          pEnabled_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_setAssignment                    (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,bool                           assignment_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_PDO_addPadding                       (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SPdo_t*             pPdo_p
                                                                                            ,uint8_t                        length_p);

extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_getObject                        (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       index_p
                                                                                            ,EC_API_SLV_SCoE_Object_t**     pObject_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_getObjectData                    (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SCoE_Object_t*      pObject_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,uint16_t*                      pData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_setObjectData                    (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SCoE_Object_t*      pObject_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,uint16_t*                      pData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_getObjectEntryCount              (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SCoE_Object_t*      pObject_p
                                                                                            ,uint8_t*                       pCount_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_getObjectLength                  (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SCoE_Object_t*      pObject_p
                                                                                            ,uint32_t*                      length_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_getObjectEntryLength             (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SCoE_ObjEntry_t*    pObjEntry_p
                                                                                            ,uint32_t*                      length_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_getObjectType                    (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SCoE_Object_t*      pObject_p
                                                                                            ,uint8_t*                       pType_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_getObjectEntry                   (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       index_p
                                                                                            ,uint8_t                        subIndex_p
                                                                                            ,EC_API_SLV_SCoE_ObjEntry_t**   pObjectEntry_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_getObjectEntryByObject           (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SCoE_Object_t*      pObject_p
                                                                                            ,uint8_t                        subIndex_p
                                                                                            ,EC_API_SLV_SCoE_ObjEntry_t**   pObjectEntry_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_getObjectEntryData               (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SCoE_ObjEntry_t*    pObjectEntry_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,uint16_t*                      pData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_setObjectEntryData               (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SCoE_ObjEntry_t*    pObjectEntry_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,uint16_t*                      pData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_generateObjectDict               (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);

extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_odAddVariable                    (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       index_p
                                                                                            ,char*                          pName_p
                                                                                            ,uint16_t                       type_p
                                                                                            ,uint16_t                       bitLen_p
                                                                                            ,uint16_t                       flags_p
                                                                                            ,EC_API_SLV_CBObjRead_t         cbRead_p
                                                                                            ,void*                          pReadCtxt_p
                                                                                            ,EC_API_SLV_CBObjWrite_t        cbWrite_p
                                                                                            ,void*                          pWriteCtxt_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_odAddArray                       (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       index_p
                                                                                            ,char*                          pName_p
                                                                                            ,uint8_t                        arrayLen_p
                                                                                            ,uint16_t                       type_p
                                                                                            ,uint16_t                       bitLen_p
                                                                                            ,uint16_t                       flags_p
                                                                                            ,EC_API_SLV_CBObjRead_t         cbRead_p
                                                                                            ,void*                          pReadCtxt_p
                                                                                            ,EC_API_SLV_CBObjWrite_t        cbWrite_p
                                                                                            ,void*                          pWriteCtxt_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_odAddRecord                      (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       index_p
                                                                                            ,char*                          pName_p
                                                                                            ,EC_API_SLV_CBObjRead_t         cbRead_p
                                                                                            ,void*                          pReadCtxt_p
                                                                                            ,EC_API_SLV_CBObjWrite_t        cbWrite_p
                                                                                            ,void*                          pWriteCtxt_p
                                                                                            ,EC_API_SLV_SCoE_Object_t**     pOutSdo_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_configRecordSubIndex             (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SCoE_Object_t*      pObject_p
                                                                                            ,uint8_t                        subIndex_p
                                                                                            ,char*                          pName_p
                                                                                            ,uint16_t                       type_p
                                                                                            ,uint16_t                       bitLen_p
                                                                                            ,uint16_t                       flags_p);

extern ECATSLV_API  uint32_t                EC_API_SLV_CoE_subIdx0WrFlag                    (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_SCoE_Object_t*      pObject_p
                                                                                            ,bool                           readWriteFlag_p);

extern ECATSLV_API  uint32_t                EC_API_SLV_getConfigStatus                      (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_getCyclicStatus                      (EC_API_SLV_SHandle_t*          pEcSlaveApi_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_writeEmergency                       (uint16_t                       code_p
                                                                                            ,uint32_t                       length_p
                                                                                            ,uint8_t*                       pData_p);
extern ECATSLV_API  EC_API_SLV_EEsmState_t  EC_API_SLV_getState                             (void);
extern ECATSLV_API  uint32_t                EC_API_SLV_setState                             (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,EC_API_SLV_EEsmState_t         state_p
                                                                                            ,uint16_t                       alErrorCode_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_EoE_sendFrame                        (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       length_p
                                                                                            ,uint8_t*                       pData_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_setStationAlias                      (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       stationAlias_p);
extern ECATSLV_API  uint8_t                 EC_API_SLV_readByteEscRegister                  (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       escAddress_p);
extern ECATSLV_API  uint16_t                EC_API_SLV_readWordEscRegister                  (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       escAddress_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_readDoubleWordEscRegister            (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       escAddress_p);
extern ECATSLV_API  void                    EC_API_SLV_writeByteEscRegister                 (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       escAddress_p
                                                                                            ,uint8_t                        value_p);
extern ECATSLV_API  void                    EC_API_SLV_writeWordEscRegister                 (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       escAddress_p
                                                                                            ,uint16_t                       value_p);
extern ECATSLV_API  void                    EC_API_SLV_writeDoubleWordEscRegister           (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,uint16_t                       escAddress_p
                                                                                            ,uint32_t                       value_p);
extern ECATSLV_API  void                    EC_API_SLV_mainLoopCyclic                       (void);

#if !(defined FBTL_REMOTE) && !(defined DPRAM_REMOTE)
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterPhyReset                   (CUST_PHY_CBreset_t             cbFunc_p
                                                                                            ,void*                          pContext_p);
extern ECATSLV_API  bool                    EC_API_SLV_registerPhy                          (uint8_t                        phyIdx_p
                                                                                            ,uint8_t                        phyAddr_p
                                                                                            ,bool                           invertLinkPolarity_p
                                                                                            ,bool                           useRxLinkPin_p);
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterPhyLibDetect               (CUST_PHY_CBextPhyLibDetect_t   cbFunc_p
                                                                                            ,void*                          pContext_p);
extern ECATSLV_API  uint32_t                EC_API_SLV_phyRegRead                           (void*                          pStackContext_p
                                                                                            ,uint32_t                       regNum_p
                                                                                            ,uint16_t*                      pData_p);
extern ECATSLV_API  void                    EC_API_SLV_phyRegWrite                          (void*                          pStackContext_p
                                                                                            ,uint32_t                       regNum_p
                                                                                            ,uint16_t                       wrVal_p);
#endif
extern ECATSLV_API  void                    EC_API_SLV_cbRegisterBoardStatusLed             (EC_API_SLV_SHandle_t*          pEcSlaveApi_p
                                                                                            ,void*                          pContext_p
                                                                                            ,EC_API_SLV_CBBoardStatusLed_t  cbFunc_p
                                                                                            ,void*                          pLedContext_p);

#if (defined __cplusplus)
}
#endif

#endif /* __ECSLVAPI_H__ */

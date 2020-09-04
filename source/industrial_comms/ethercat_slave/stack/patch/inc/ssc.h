/*!
* \file ssc.h
*
* \brief
* Beckhoff EC SSC Integration interface.
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

#if !(defined __SSC_H__)
#define __SSC_H__        1



#if (defined __cplusplus)
extern "C" {
#endif

#include <osal.h>
#include <ssc_cfg.h>

/* CoE */
#include <objdef.h>
#include <emcy.h>

/* AoE */
#include <ecataoe.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#if (defined BKHFSSC_SO) && (BKHFSSC_SO==1) // defined if bkhfSsc is compiled as a DLL
#ifdef BKHFSSC_EXPORTS // defined if we are building the BKHFSSC DLL (instead of using it)
#define BKHFSSC_API OSAL_DLL_EXPORT
#else
#define BKHFSSC_API OSAL_DLL_IMPORT
#endif // BKHFSSC_EXPORTS
#define BKHFSSC_LOC OSAL_DLL_LOCAL
#else // BKHFSSC_SO is not defined: this means BKHFSSC is a static lib.
#define BKHFSSC_API
#define BKHFSSC_LOC
#endif // BKHFSSC_SO

typedef void        (*cbMainLoop_t)             (void*      pCtxt_p);
typedef void        (*cbInputMapping_t)         (void*      pCtxt_p
                                                ,uint16_t*  pData_p);
typedef void        (*cbOutputMapping_t)        (void*      pCtxt_p
                                                ,uint16_t*  pData_p);
typedef void        (*cbApplication_t)          (void*      pCtxt_p);
typedef uint16_t    (*cbGenerateMapping_t)      (void*      pCtxt_p
                                                ,uint16_t*  pInputSize_p
                                                ,uint16_t*  pOutputSize_p);
typedef uint16_t    (*cbStartMailboxHandler_t)  (void*      pCtxt_p);
typedef uint16_t    (*cbStopMailboxHandler_t)   (void*      pCtxt_p);
typedef uint16_t    (*cbStartInputHandler_t)    (void*      pCtxt_p
                                                ,uint16_t*  pIntMask_p);
typedef uint16_t    (*cbStopInputHandler_t)     (void*      pCtxt_p);
typedef uint16_t    (*cbStartOutputHandler_t)   (void*      pCtxt_p);
typedef uint16_t    (*cbStopOutputHandler_t)    (void*      pCtxt_p);
typedef void        (*cbAckErrorInd_t)          (void*      pCtxt_p
                                                ,uint16_t   stateTrans_p);

typedef void        (*EOE_cbReceive_t)          (void*      pCtxt_p
                                                ,uint16_t*  pData_p
                                                ,uint16_t   length_p);
typedef void        (*EOE_cbSettingInd_t)       (void*      pCtxt_p
                                                ,uint16_t*  pMac_p
                                                ,uint16_t*  pIp_p
                                                ,uint16_t*  pSubNet_p
                                                ,uint16_t*  pDefaultGateway_p
                                                ,uint16_t*  pDnsIp_p);

typedef uint16_t    (*FOE_cbRead_t)             (void*      pCtxt_p
                                                ,uint16_t*  pName_p
                                                ,uint16_t   nameSize_p
                                                ,uint32_t   password_p
                                                ,uint16_t   maxblockSize_p
                                                ,uint16_t*  pData_p);
typedef uint16_t    (*FOE_cbReadData_t)         (void*      pCtxt_p
                                                ,uint32_t   fileOffset_p
                                                ,uint16_t   foeMaxSendBlockSize_p
                                                ,uint16_t*  pData_p);
typedef uint16_t    (*FOE_cbWrite_t)            (void*      pCtxt_p
                                                ,uint16_t*  pName_p
                                                ,uint16_t   nameSize_p
                                                ,uint32_t   password_p);
typedef uint16_t    (*FOE_cbWriteData_t)        (void*      pCtxt_p
                                                ,uint16_t*  pData_p
                                                ,uint16_t   size_p
                                                ,uint8_t    dataFollowing_p);
typedef void        (*FOE_cbError_t)            (void*      pCtxt_p
                                                ,uint32_t   errorCode_p);

typedef void        (*BL_cbStart_t)             (void*      pCtxt_p
                                                ,uint8_t    state_p);
typedef void        (*BL_cbStop_t)              (void*      pCtxt_p);
typedef void        (*BL_cbFinish_t)            (void*      pCtxt_p);

typedef void        (*SOE_cbSend_t)             (void*      pCtxt_p
                                                ,void*      pSendMbx_p);
typedef uint8_t     (*SOE_cbRecv_t)             (void*      pCtxt_p
                                                ,void*      pRecvMbx_p);
typedef uint16_t    (*AOE_cbRecv_t)             (void*      pCtxt_p
                                                ,void*      pRecvMbx_p);

typedef void        (*cbSetLedOut_t)            (void*      pCtxt_p
                                                ,uint8_t    runLed_p
                                                ,uint8_t    errLed_p);

typedef void        (*cbGlobalMutex_t)          (void*      pCtxt_p
                                                ,uint8_t    lock_p);
typedef uint32_t    (*cbGetTimerRegister_t)     (void*      pCtxt_p);
typedef uint32_t    (*cbGetTimerRegisterOvl_t)  (void*      pCtxt_p
                                                ,bool*      pOverflow_p);
typedef void        (*cbClearTimerRegister_t)   (void*      pCtxt_p);
typedef void        (*ESC_cbWrite_t)            (void*      pCtxt_p
                                                ,uint8_t    isr_p
                                                ,uint8_t*   pData_p
                                                ,uint16_t   address_p
                                                ,uint16_t   length_p);
typedef void        (*ESC_cbWriteWord_t)        (void*      pCtxt_p
                                                ,uint16_t   wordValue_p
                                                ,uint16_t   address_p);
typedef void        (*ESC_cbWriteByte_t)        (void*      pCtxt_p
                                                ,uint8_t    byteValue_p
                                                ,uint16_t   address_p);
typedef void        (*ESC_cbWriteMbxMem_t)      (void*      pCtxt_p
                                                ,uint8_t*   pData_p
                                                ,uint16_t   address_p
                                                ,uint16_t   length_p);

typedef void        (*ESC_cbRead_t)             (void*      pCtxt_p
                                                ,uint8_t    isr_p
                                                ,uint8_t*   pData_p
                                                ,uint16_t   address_p
                                                ,uint16_t   length_p);
typedef uint8_t     (*ESC_cbReadByte_t)         (void*      pCtxt_p
                                                ,uint8_t    isr_p
                                                ,uint16_t   address_p);
typedef uint16_t    (*ESC_cbReadWord_t)         (void*      pCtxt_p
                                                ,uint8_t    isr_p
                                                ,uint16_t   address_p);
typedef uint32_t    (*ESC_cbReadDword_t)        (void*      pCtxt_p
                                                ,uint8_t    isr_p
                                                ,uint16_t   address_p);
typedef void        (*ESC_cbReadMbxMem_t)       (void*      pCtxt_p
                                                ,uint8_t*   pData_p
                                                ,uint16_t   address_p
                                                ,uint16_t   length_p);

typedef int32_t     (*ESCEEP_cbRegLd_t)         (void*      pCtxt_p
                                                ,int32_t    reload_flag_p);
typedef uint16_t    (*ESCEEP_cpRead_t)          (void*      pCtxt_p
                                                ,uint32_t   wordaddr_p);
typedef uint16_t    (*ESCEEP_cbWrite_t)         (void*      pCtxt_p
                                                ,uint32_t   wordaddr_p);
typedef uint16_t    (*ESCEEP_cbReload_t)        (void*      pCtxt_p);
typedef void        (*ESCEEP_cbStore_t)         (void*      pCtxt_p);

#if (defined __cplusplus)
extern "C" {
#endif

extern BKHFSSC_API void     SSC_registerMainLoopCb              (cbMainLoop_t                   cbMainLoop_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerInputmappingCb          (cbInputMapping_t               cbInputMapping_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerOutputmappingCb         (cbOutputMapping_t              cbOutputMapping_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerApplicationCb           (cbApplication_t                cbApplication_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerGenerateMappingCb       (cbGenerateMapping_t            cbGenerateMapping_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerStartMailboxHandlerCb   (cbStartMailboxHandler_t        cbStartMailboxHandler_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerStopMailboxHandlerCb    (cbStopMailboxHandler_t         cbStopMailboxHandler_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerStartInputHandlerCb     (cbStartInputHandler_t          cbStartInputHandler_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerStopInputHandlerCb      (cbStopInputHandler_t           cbStopInputHandler_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerStartOutputHandlerCb    (cbStartOutputHandler_t         cbStartOutputHandler_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerStopOutputHandlerCb     (cbStopOutputHandler_t          cbStopOutputHandler_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerAckErrorIndCb           (cbAckErrorInd_t                cbAckErrorInd_p
                                                                ,void*                          pContext_p);

extern BKHFSSC_API void     SSC_EOE_registerReceiveCb           (EOE_cbReceive_t                cbEoeReceive_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_EOE_registerSettingIndCb        (EOE_cbSettingInd_t             cbEoeSettingInd_p
                                                                ,void*                          pContext_p);

extern BKHFSSC_API void     SSC_FOE_registerReadCb              (FOE_cbRead_t                   cbFoeRead_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_FOE_registerReadDataCb          (FOE_cbReadData_t               cbFoeReadData_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_FOE_registerWriteCb             (FOE_cbWrite_t                  cbFoeWrite_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_FOE_registerWriteDataCb         (FOE_cbWriteData_t              cbFoeWriteData_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_FOE_registerErrorCb             (FOE_cbError_t                  cbFoeError_p
                                                                ,void*                          pContext_p);

extern BKHFSSC_API void     SSC_BL_registerStartCb              (BL_cbStart_t                   cbBlStart_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_BL_registerStopCb               (BL_cbStop_t                    cbBlStop_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_BL_registerFinishCb             (BL_cbFinish_t                  cbBlFinish_p
                                                                ,void*                          pContext_p);

extern BKHFSSC_API void     SSC_SOE_registerSendCb              (SOE_cbSend_t                   cbSoeSend_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_SOE_registerRecvCb              (SOE_cbRecv_t                   cbSoeRecv_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_AOE_registerRecvCb              (AOE_cbRecv_t                   cbAoeRecv_p
                                                                ,void*                          pContext_p);

extern BKHFSSC_API void     SSC_registerSetLedOutCb             (cbSetLedOut_t                  cbSetLedOut_p
                                                                ,void*                          pContext_p);

extern BKHFSSC_API void     SSC_registerGlobalMutexCb           (cbGlobalMutex_t                cbGlobalMutex_p
                                                                ,void*                          pContext_p);

extern BKHFSSC_API void     SSC_registerGetTimerRegisterCb      (cbGetTimerRegister_t           cbGetTimerRegister_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerGetTimerRegisterOvlCb   (cbGetTimerRegisterOvl_t        cbGetTimerRegisterOvl_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_registerClearTimerRegisterCb    (cbClearTimerRegister_t         cbClearTimerRegister_p
                                                                ,void*                          pContext_p);

extern BKHFSSC_API void     SSC_ESC_registerWriteCb             (ESC_cbWrite_t                  cbEscWrite_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_ESC_registerWriteWordCb         (ESC_cbWriteWord_t              cbEscWriteWord_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_ESC_registerWriteByteCb         (ESC_cbWriteByte_t              cbEscWriteByte_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_ESC_registerWriteMbxMemCb       (ESC_cbWriteMbxMem_t            cbEscWriteMbxMem_p
                                                                ,void*                          pContext_p);

extern BKHFSSC_API void     SSC_ESC_registerReadCb              (ESC_cbRead_t                   cbEscRead_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_ESC_registerReadByteCb          (ESC_cbReadByte_t               cbEscReadByte_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_ESC_registerReadWordCb          (ESC_cbReadWord_t               cbEscReadWord_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_ESC_registerReadDwordCb         (ESC_cbReadDword_t              cbEscReadDword_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_ESC_registerReadMbxMemCb        (ESC_cbReadMbxMem_t             cbEscReadMbxMem_p
                                                                ,void*                          pContext_p);

extern BKHFSSC_API void     SSC_ESC_registerEepromReadCb        (ESCEEP_cpRead_t                cbEscEeprom_Read_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_ESC_registerEepromWriteCb       (ESCEEP_cbWrite_t               cbEscEeprom_Write_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_ESC_registerEepromReloadCb      (ESCEEP_cbReload_t              cbEscEeprom_Reload_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_ESC_registerEepromStoreCb       (ESCEEP_cbStore_t               cbEscEeprom_Store_p
                                                                ,void*                          pContext_p);
extern BKHFSSC_API void     SSC_ESC_registerEepromLdRegCb       (ESCEEP_cbRegLd_t               cbEscEeprom_RegLd_p
                                                                ,void*                          pContext_p);

extern BKHFSSC_API void     SSC_COE_setDeviceType               (uint32_t                       deviceType_p);
extern BKHFSSC_API void     SSC_COE_setSMErrorMaxLimit          (uint16_t                       maxSmErrLimit_p);
extern BKHFSSC_API void     SSC_setErrorRegister                (uint16_t                       error_p);

extern BKHFSSC_API uint16_t SSC_COE_addObjectToDic              (TOBJECT*                       pNewObjEntry_p);
extern BKHFSSC_API void     SSC_COE_removeDicEntry              (uint16_t                       index_p);

extern BKHFSSC_API void     SSC_AL_ControlInd                   (uint8_t                        alControl_p
                                                                ,uint16_t                       alStatusCode_p);
extern BKHFSSC_API void     SSC_ECAT_StateChange                (uint8_t                        alStatus_p
                                                                ,uint16_t                       alStatusCode_p);
extern BKHFSSC_API uint8_t  SSC_Al_status                       (void);

extern BKHFSSC_API bool     SSC_inputUpdateRunning              (void);
extern BKHFSSC_API bool     SSC_outputUpdateRunning             (void);

extern BKHFSSC_API bool     SSC_ecatApplRunning                 (void);
extern BKHFSSC_API void     SSC_ecatApplSetRunning              (bool                           run_p);

extern BKHFSSC_API uint16_t SSC_MBX_receiveSize                 (void);
extern BKHFSSC_API uint8_t  SSC_MBX_sendReq                     (TMBX*                          pMbx_p
                                                                ,uint8_t                        flags_p);

extern BKHFSSC_API uint16_t SSC_EOE_sendFrameRequest            (uint16_t*                      pData_p
                                                                ,uint16_t                       length_p);

extern BKHFSSC_API TEMCYMESSAGE*
                            SSC_EMCY_getOutOfEmptyQueue         (void);
extern BKHFSSC_API TEMCYMESSAGE*
                            SSC_EMCY_getEmcyBuffer              (void);
extern BKHFSSC_API uint8_t  SSC_EMCY_sendEmergency              (TEMCYMESSAGE*                  pEmcy_p);
extern BKHFSSC_API void     SSC_EMCY_putInEmptyQueue            (TEMCYMESSAGE*                  pEmcy_p);
extern BKHFSSC_API void     SSC_EMCY_putInSendQueue             (TEMCYMESSAGE*                  pEmcy_p);

extern BKHFSSC_API OBJCONST TOBJECT*
                            SSC_OBJ_getObjectHandle             (uint16_t                       index_p);
extern BKHFSSC_API uint16_t SSC_OBJ_getEntryOffset              (uint8_t                        subindex_p
                                                                ,OBJCONST TOBJECT*              pObjEntry_p);
extern BKHFSSC_API OBJCONST TSDOINFOENTRYDESC*
                            SSC_OBJ_getEntryDesc                (OBJCONST TOBJECT OBJMEM*       pObjEntry_p
                                                                ,uint8_t                        subindex_p);

extern BKHFSSC_API uint32_t SSC_OBJ_GetObjectLength             (uint16_t                       index_p
                                                                ,uint8_t                        subindex_p
                                                                ,OBJCONST TOBJECT OBJMEM*       pObjEntry_p
                                                                ,uint8_t                        completeAccess_p);

extern BKHFSSC_API uint16_t SSC_bitMask                         (uint16_t                       offset_p);
extern BKHFSSC_API void     SSC_setSyncByUser                   (bool                           set_p);

extern BKHFSSC_API uint8_t  SSC_checkSyncTypeValue              (uint16_t                       index_p
                                                                ,uint16_t                       newSyncType_p);

extern BKHFSSC_API uint16_t SSC_BKHF_mainInit                   (void);
extern BKHFSSC_API void     SSC_BKHF_mainLoop                   (void);

extern BKHFSSC_API uint32_t SSC_ECAT_TIMER_INC_P_MS             (void);

extern BKHFSSC_API uint16_t SSC_AOE_AmsRes                      (AmsCmd*                        pCmd_p
                                                                ,uint16_t                       amsErrCode_p
                                                                ,uint16_t                       dataLen_p);

extern BKHFSSC_API AmsCmd*  SSC_AOE_FragmentedCmdInd            (AmsCmd*                        pCmd_p);

#if (defined __cplusplus)
}
#endif

#endif /* __SSC_H__ */

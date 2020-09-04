/*!
* \file ssc_backend.h
*
* \brief
* Beckhoff SSC Integration: Callback backend.
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

#if !(defined __SSC_BACKEND_H__)
#define __SSC_BACKEND_H__		1

#include <osal.h>
#include <ssc.h>

typedef struct SSC_ESC_sApplicationCallback
{
    cbMainLoop_t            cbMainLoop;
    void*                   pMainLoopCtxt;

    cbInputMapping_t        cbInputMapping;
    void*                   pInputMappingCtxt;

    cbOutputMapping_t       cbOutputMapping;
    void*                   pOutputMappingCtxt;

    cbApplication_t         cbApplication;
    void*                   pApplicationCtxt;

    cbGenerateMapping_t     cbGenerateMapping;
    void*                   pGenerateMappingCtxt;

    cbStartMailboxHandler_t cbStartMailboxHandler;
    void*                   pStartMailboxHandlerCtxt;

    cbStopMailboxHandler_t  cbStopMailboxHandler;
    void*                   pStopMailboxHandlerCtxt;

    cbStartInputHandler_t   cbStartInputHandler;
    void*                   pStartInputHandlerCtxt;

    cbStopInputHandler_t    cbStopInputHandler;
    void*                   pStopInputHandlerCtxt;

    cbStartOutputHandler_t  cbStartOutputHandler;
    void*                   pStartOutputHandlerCtxt;

    cbStopOutputHandler_t   cbStopOutputHandler;
    void*                   pStopOutputHandlerCtxt;

    cbAckErrorInd_t         cbAckErrorInd;
    void*                   pAckErrorIndCtxt;

    EOE_cbReceive_t         cbEoeReceive;
    void*                   pEoeReceiveCtxt;

    EOE_cbSettingInd_t      cbEoeSettingInd;
    void*                   pEoeSettingIndCtxt;

    FOE_cbRead_t            cbFoeRead;
    void*                   pFoeReadCtxt;

    FOE_cbReadData_t        cbFoeReadData;
    void*                   pFoeReadDataCtxt;

    FOE_cbWrite_t           cbFoeWrite;
    void*                   pFoeWriteCtxt;

    FOE_cbWriteData_t       cbFoeWriteData;
    void*                   pFoeWriteDataCtxt;

    FOE_cbError_t           cbFoeError;
    void*                   pFoeErrorCtxt;

    BL_cbStart_t            cbBlStart;
    void*                   pBlStartCtxt;

    BL_cbStop_t             cbBlStop;
    void*                   pBlStopCtxt;

    BL_cbFinish_t           cbBlFinish;
    void*                   pBlFinishCtxt;

    SOE_cbSend_t            cbSoeSendCb;
    void*                   pSoeSendCtxt;

    SOE_cbRecv_t            cbSoeRecvCb;
    void*                   pSoeRecvCtxt;

    AOE_cbRecv_t            cbAoeRecvCb;
    void*                   pAoeRecvCtxt;

    cbSetLedOut_t           cbSetLedOut;
    void*                   pSetLedOutCtxt;

    cbGlobalMutex_t         cbGlobalMutex;
    void*                   pGlobalMutexCtxt;

    cbGetTimerRegister_t    cbGetTimerRegister;
    void*                   pGetTimerRegisterCtxt;

    cbGetTimerRegisterOvl_t cbGetTimerRegisterOvl;
    void*                   pGetTimerRegisterOvlCtxt;

    cbClearTimerRegister_t  cbClearTimerRegister;
    void*                   pClearTimerRegisterCtxt;

    ESC_cbWrite_t           cbEscWrite;
    void*                   pEscWriteCtxt;

    ESC_cbWriteWord_t       cbEscWriteWord;
    void*                   pEscWriteWordCtxt;

    ESC_cbWriteByte_t       cbEscWriteByte;
    void*                   pEscWriteByteCtxt;

    ESC_cbWriteMbxMem_t     cbEscWriteMbxMem;
    void*                   pEscWriteMbxMemCtxt;

    ESC_cbRead_t            cbEscRead;
    void*                   pEscReadCtxt;

    ESC_cbReadByte_t        cbEscReadByte;
    void*                   pEscReadByteCtxt;

    ESC_cbReadWord_t        cbEscReadWord;
    void*                   pEscReadWordCtxt;

    ESC_cbReadDword_t       cbEscReadDword;
    void*                   pEscReadDwordCtxt;

    ESC_cbReadMbxMem_t      cbEscReadMbxMem;
    void*                   pEscReadMbxMemCtxt;

    ESCEEP_cbRegLd_t        cbEscEepromLdReg;
    void*                   pEscEepromLdRegCtxt;

    ESCEEP_cpRead_t         cbEscEepromRead;
    void*                   pEscEepromReadCtxt;

    ESCEEP_cbWrite_t        cbEscEepromWrite;
    void*                   pEscEepromWriteCtxt;

    ESCEEP_cbReload_t       cbEscEepromReload;
    void*                   pEscEepromReloadCtxt;

    ESCEEP_cbStore_t        cbEscEepromStore;
    void*                   pEscEepromStoreCtxt;
} SSC_ESC_sApplicationCallback_t;

#if (defined __SSC_BACKEND_IMPLEMENTATION)
       SSC_ESC_sApplicationCallback_t   SSC_callbacks_g;
#else
extern SSC_ESC_sApplicationCallback_t   SSC_callbacks_g;
#endif

#if (defined __cplusplus)
extern "C" {
#endif

/* CallBacks */
extern void     SSC_mainLoop                (void);
extern void     SSC_inputMapping            (uint16_t*          pData_p);
extern void     SSC_outputMapping           (uint16_t*          pData_p);
extern void     SSC_application             (void);
extern uint16_t SSC_generateMapping         (uint16_t*          pInputSize_p
                                            ,uint16_t*          pOutputSize_p);
extern uint16_t SSC_startMailboxHandler     (void);
extern uint16_t SSC_stopMailboxHandler      (void);
extern uint16_t SSC_startInputHandler       (uint16_t*          pIntMask_p);
extern uint16_t SSC_stopInputHandler        (void);
extern uint16_t SSC_startOutputHandler      (void);
extern uint16_t SSC_stopOutputHandler       (void);
extern void     SSC_ackErrorInd             (uint16_t           stateTrans_p);

extern uint16_t SSC_FOE_read                (uint16_t*          pName_p
                                            ,uint16_t           nameSize_p
                                            ,uint32_t           password_p
                                            ,uint16_t           maxblockSize_p
                                            ,uint16_t*          pData_p);
extern uint16_t SSC_FOE_readData            (uint32_t           fileOffset_p
                                            ,uint16_t           foeMaxSendBlockSize_p
                                            ,uint16_t*          pData_p);
//extern uint16_t ssc_FoeBusy(uint16_t done, uint32_t fileOffset, uint16_t *pData);
extern uint16_t SSC_FOE_write               (uint16_t*          pName_p
                                            ,uint16_t           nameSize_p
                                            ,uint32_t           password_p);
extern uint16_t SSC_FOE_writeData           (uint16_t*          pData_p
                                            ,uint16_t           size_p
                                            ,uint8_t            dataFollowing_p);
extern void     SSC_FOE_error               (uint32_t           errorCode_p);
extern void     SSC_BL_start                (uint8_t            state_p);
extern void     SSC_BL_stop                 (void);
extern void     SSC_BL_finish               (void);
extern void     SSC_SOE_send                (void*              pSendMbx_p);
extern uint8_t  SSC_SOE_recv                (void*              pRecvMbx_p);
extern void     SSC_EOE_receive             (uint16_t*          pData_p
                                            ,uint16_t           length_p);
extern void     SSC_EOE_settingInd          (uint16_t*          pMac_p
                                            ,uint16_t*          pIp_p
                                            ,uint16_t*          pSubNet_p
                                            ,uint16_t*          pDefaultGateway_p
                                            ,uint16_t*          pDnsIp_p);

#if (defined __cplusplus)
}
#endif

#endif /* __SSC_BACKEND_H__ */

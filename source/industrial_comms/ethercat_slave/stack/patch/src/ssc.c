/*!
* \file ssc.c
*
* \brief
* Beckhoff SSC Integration.
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

#include <ssc.h>

#include "ssc_backend.h"

/* Beckhoff */
#include <applInterface.h>
#include <coeappl.h>
#include <aoeappl.h>

#if !KUNBUS_SSC_EVAL
extern void EC_API_SLV_SSC_setLicense(char* pLicense_p, uint8_t length_p);
#endif

/* Beckhoff SSC variable names */
extern UINT32       u32Devicetype;
extern UINT16       u16ErrorRegister;
extern const UINT16 cBitMask[];
/* /Beckhoff SSC */

#if !KUNBUS_SSC_EVAL
    #define KUNBUS_SSC_LICENSE "VALIDETGMEMBER"
#endif

/*! <!-- Description: -->
 *
 *  \brief
 *  Register MainLoop hook
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbMainLoop_p    Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerMainLoopCb(cbMainLoop_t cbMainLoop_p, void* pContext_p)
{
    pAPPL_MainLoop                  = SSC_mainLoop;
    SSC_callbacks_g.cbMainLoop      = cbMainLoop_p;
    SSC_callbacks_g.pMainLoopCtxt   = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register InputMapping Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbInputMapping_p    Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerInputmappingCb(cbInputMapping_t cbInputMapping_p, void* pContext_p)
{
    SSC_callbacks_g.cbInputMapping      = cbInputMapping_p;
    SSC_callbacks_g.pInputMappingCtxt   = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register OutputMapping Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbOutputMapping_p   Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerOutputmappingCb(cbOutputMapping_t cbOutputMapping_p, void* pContext_p)
{
    SSC_callbacks_g.cbOutputMapping     = cbOutputMapping_p;
    SSC_callbacks_g.pOutputMappingCtxt  = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Application Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbApplication_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerApplicationCb(cbApplication_t cbApplication_p, void* pContext_p)
{
    SSC_callbacks_g.cbApplication       = cbApplication_p;
    SSC_callbacks_g.pApplicationCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Generate Mapping Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbGenerateMapping_p Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerGenerateMappingCb(cbGenerateMapping_t cbGenerateMapping_p, void* pContext_p)
{
    SSC_callbacks_g.cbGenerateMapping       = cbGenerateMapping_p;
    SSC_callbacks_g.pGenerateMappingCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Start Mailbox Handler Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbStartMailboxHandler_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p                  Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerStartMailboxHandlerCb(cbStartMailboxHandler_t cbStartMailboxHandler_p, void* pContext_p)
{
    SSC_callbacks_g.cbStartMailboxHandler       = cbStartMailboxHandler_p;
    SSC_callbacks_g.pStartMailboxHandlerCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Stop Mailbox Handler Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbStopMailboxHandler_p  Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p              Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerStopMailboxHandlerCb(cbStopMailboxHandler_t cbStopMailboxHandler_p, void* pContext_p)
{
    SSC_callbacks_g.cbStopMailboxHandler        = cbStopMailboxHandler_p;
    SSC_callbacks_g.pStopMailboxHandlerCtxt     = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Start Input Handler Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbStartInputHandler_p   Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p              Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerStartInputHandlerCb(cbStartInputHandler_t cbStartInputHandler_p, void* pContext_p)
{
    SSC_callbacks_g.cbStartInputHandler     = cbStartInputHandler_p;
    SSC_callbacks_g.pStartInputHandlerCtxt  = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Stop Input Handler Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbStopInputHandler_p    Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p              Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerStopInputHandlerCb(cbStopInputHandler_t cbStopInputHandler_p, void* pContext_p)
{
    SSC_callbacks_g.cbStopInputHandler      = cbStopInputHandler_p;
    SSC_callbacks_g.pStopInputHandlerCtxt   = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Start Output Handler Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbStartOutputHandler_p  Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p              Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerStartOutputHandlerCb(cbStartOutputHandler_t cbStartOutputHandler_p, void* pContext_p)
{
    SSC_callbacks_g.cbStartOutputHandler    = cbStartOutputHandler_p;
    SSC_callbacks_g.pStartOutputHandlerCtxt = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Stop Output Handler Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbStopOutputHandler_p   Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p              Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerStopOutputHandlerCb(cbStopOutputHandler_t cbStopOutputHandler_p, void* pContext_p)
{
    SSC_callbacks_g.cbStopOutputHandler     = cbStopOutputHandler_p;
    SSC_callbacks_g.pStopOutputHandlerCtxt  = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Acknowledge Error Indication Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbAckErrorInd_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerAckErrorIndCb(cbAckErrorInd_t cbAckErrorInd_p, void* pContext_p)
{
    SSC_callbacks_g.cbAckErrorInd       = cbAckErrorInd_p;
    SSC_callbacks_g.pAckErrorIndCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register EoE receive Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEoeReceive_p  Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_EOE_registerReceiveCb(EOE_cbReceive_t cbEoeReceive_p, void* pContext_p)
{
    /* There is no APPL_EoeReceive function, register ssc directly */
    pAPPL_EoeReceive                = SSC_EOE_receive;
    SSC_callbacks_g.cbEoeReceive    = cbEoeReceive_p;
    SSC_callbacks_g.pEoeReceiveCtxt = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register EoE Settings Indication Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEoeSettingInd_p   Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_EOE_registerSettingIndCb(EOE_cbSettingInd_t cbEoeSettingInd_p, void* pContext_p)
{
    /* There is no APPL_EoeReceive function, register ssc directly */
    pAPPL_EoeSettingInd                 = SSC_EOE_settingInd;
    SSC_callbacks_g.cbEoeSettingInd     = cbEoeSettingInd_p;
    SSC_callbacks_g.pEoeSettingIndCtxt  = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register FoE Read Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbFoeRead_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_FOE_registerReadCb(FOE_cbRead_t cbFoeRead_p, void* pContext_p)
{
    pAPPL_FoeRead                   = SSC_FOE_read;
    SSC_callbacks_g.cbFoeRead       = cbFoeRead_p;
    SSC_callbacks_g.pFoeReadCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register FoE Ack Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbFoeReadData_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_FOE_registerReadDataCb(FOE_cbReadData_t cbFoeReadData_p, void* pContext_p)
{
    pAPPL_FoeReadData                   = SSC_FOE_readData;
    SSC_callbacks_g.cbFoeReadData       = cbFoeReadData_p;
    SSC_callbacks_g.pFoeReadDataCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register FoE Write Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbFoeWrite_p    Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_FOE_registerWriteCb(FOE_cbWrite_t cbFoeWrite_p, void* pContext_p)
{
    pAPPL_FoeWrite                  = SSC_FOE_write;
    SSC_callbacks_g.cbFoeWrite      = cbFoeWrite_p;
    SSC_callbacks_g.pFoeWriteCtxt   = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register FoE Write Data Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbFoeWriteData_p    Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_FOE_registerWriteDataCb(FOE_cbWriteData_t cbFoeWriteData_p, void* pContext_p)
{
    pAPPL_FoeWriteData                  = SSC_FOE_writeData;
    SSC_callbacks_g.cbFoeWriteData      = cbFoeWriteData_p;
    SSC_callbacks_g.pFoeWriteDataCtxt   = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register FoE Error Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbFoeError_p    Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_FOE_registerErrorCb(FOE_cbError_t cbFoeError_p, void* pContext_p)
{
    pAPPL_FoeError                  = SSC_FOE_error;
    SSC_callbacks_g.cbFoeError      = cbFoeError_p;
    SSC_callbacks_g.pFoeErrorCtxt   = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Bootloader start Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbBlStart_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_BL_registerStartCb(BL_cbStart_t cbBlStart_p, void* pContext_p)
{
    SSC_callbacks_g.cbBlStart       = cbBlStart_p;
    SSC_callbacks_g.pBlStartCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Bootloader Stop Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbBlStop_p  Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p  Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_BL_registerStopCb(BL_cbStop_t cbBlStop_p, void* pContext_p)
{
    SSC_callbacks_g.cbBlStop    = cbBlStop_p;
    SSC_callbacks_g.pBlStopCtxt = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Bootloader Finish Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbBlFinish_p    Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_BL_registerFinishCb(BL_cbFinish_t cbBlFinish_p, void* pContext_p)
{
    SSC_callbacks_g.cbBlFinish      = cbBlFinish_p;
    SSC_callbacks_g.pBlFinishCtxt   = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Set LED Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbSetLedOut_p   Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerSetLedOutCb(cbSetLedOut_t cbSetLedOut_p, void* pContext_p)
{
    SSC_callbacks_g.cbSetLedOut     = cbSetLedOut_p;
    SSC_callbacks_g.pSetLedOutCtxt  = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register SoE Send callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbSoeSend_p		Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_SOE_registerSendCb(SOE_cbSend_t cbSoeSend_p, void *pContext_p)
{
    SSC_callbacks_g.cbSoeSendCb     = cbSoeSend_p;
    SSC_callbacks_g.pSoeSendCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register SoE receive callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbSoeRecv_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_SOE_registerRecvCb(SOE_cbRecv_t cbSoeRecv_p, void *pContext_p)
{
    SSC_callbacks_g.cbSoeRecvCb     = cbSoeRecv_p;
    SSC_callbacks_g.pSoeRecvCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register AoE receive callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbAoeRecv_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_AOE_registerRecvCb(AOE_cbRecv_t cbAoeRecv_p, void* pContext_p)
{
    SSC_callbacks_g.cbAoeRecvCb     = cbAoeRecv_p;
    SSC_callbacks_g.pAoeRecvCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register Global Mutex callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbGlobalMutex_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerGlobalMutexCb(cbGlobalMutex_t cbGlobalMutex_p, void* pContext_p)
{
    SSC_callbacks_g.cbGlobalMutex       = cbGlobalMutex_p;
    SSC_callbacks_g.pGlobalMutexCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register get timer register callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbGetTimerRegister_p    Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p              Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerGetTimerRegisterCb(cbGetTimerRegister_t cbGetTimerRegister_p, void* pContext_p)
{
    SSC_callbacks_g.cbGetTimerRegister      = cbGetTimerRegister_p;
    SSC_callbacks_g.pGetTimerRegisterCtxt   = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register get timer register overflow callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbGetTimerRegisterOvl_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p                  Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerGetTimerRegisterOvlCb(cbGetTimerRegisterOvl_t cbGetTimerRegisterOvl_p, void* pContext_p)
{
    SSC_callbacks_g.cbGetTimerRegisterOvl       = cbGetTimerRegisterOvl_p;
    SSC_callbacks_g.pGetTimerRegisterOvlCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register clear timer register callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbClearTimerRegister_p  Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p              Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_registerClearTimerRegisterCb(cbClearTimerRegister_t cbClearTimerRegister_p, void* pContext_p)
{
    SSC_callbacks_g.cbClearTimerRegister    = cbClearTimerRegister_p;
    SSC_callbacks_g.pClearTimerRegisterCtxt = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register write callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscWrite_p    Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerWriteCb(ESC_cbWrite_t cbEscWrite_p, void* pContext_p)
{
    SSC_callbacks_g.cbEscWrite      = cbEscWrite_p;
    SSC_callbacks_g.pEscWriteCtxt   = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register write word callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscWriteWord_p    Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerWriteWordCb(ESC_cbWriteWord_t cbEscWriteWord_p, void* pContext_p)
{
    SSC_callbacks_g.cbEscWriteWord      = cbEscWriteWord_p;
    SSC_callbacks_g.pEscWriteWordCtxt   = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register write byte callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscWriteByte_p    Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerWriteByteCb(ESC_cbWriteByte_t cbEscWriteByte_p, void* pContext_p)
{
    SSC_callbacks_g.cbEscWriteByte      = cbEscWriteByte_p;
    SSC_callbacks_g.pEscWriteByteCtxt   = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register write mailbox memory callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscWriteMbxMem_p  Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerWriteMbxMemCb(ESC_cbWriteMbxMem_t cbEscWriteMbxMem_p, void* pContext_p)
{
    SSC_callbacks_g.cbEscWriteMbxMem    = cbEscWriteMbxMem_p;
    SSC_callbacks_g.pEscWriteMbxMemCtxt = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register read callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscRead_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p      Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerReadCb(ESC_cbRead_t cbEscRead_p, void* pContext_p)
{
    SSC_callbacks_g.cbEscRead       = cbEscRead_p;
    SSC_callbacks_g.pEscReadCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register read byte callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscReadByte_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerReadByteCb(ESC_cbReadByte_t cbEscReadByte_p, void* pContext_p)
{
    SSC_callbacks_g.cbEscReadByte       = cbEscReadByte_p;
    SSC_callbacks_g.pEscReadByteCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register read word callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscReadWord_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerReadWordCb(ESC_cbReadWord_t cbEscReadWord_p, void* pContext_p)
{
    SSC_callbacks_g.cbEscReadWord       = cbEscReadWord_p;
    SSC_callbacks_g.pEscReadWordCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register read dword callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscReadDword_p    Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerReadDwordCb(ESC_cbReadDword_t cbEscReadDword_p, void* pContext_p)
{
    SSC_callbacks_g.cbEscReadDword      = cbEscReadDword_p;
    SSC_callbacks_g.pEscReadDwordCtxt   = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register read mailbox memory callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscReadMbxMem_p   Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p          Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerReadMbxMemCb(ESC_cbReadMbxMem_t cbEscReadMbxMem_p, void* pContext_p)
{
    SSC_callbacks_g.cbEscReadMbxMem     = cbEscReadMbxMem_p;
    SSC_callbacks_g.pEscReadMbxMemCtxt  = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register read EEPROM Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscEeprom_Read_p	EEPROM Read Callback
 *  \param[in]  pContext_p          EEPROM Read Callback Context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerEepromReadCb(ESCEEP_cpRead_t cbEscEeprom_Read_p, void* pContext_p)
{
    pAPPL_EEPROM_Read                   = SSC_EEP_read;
    SSC_callbacks_g.cbEscEepromRead     = cbEscEeprom_Read_p;
    SSC_callbacks_g.pEscEepromReadCtxt  = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register write EEPROM Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscEeprom_Write_p     EEPROM Write Callback
 *  \param[in]  pContext_p              EEPROM Write Callback Context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerEepromWriteCb(ESCEEP_cbWrite_t cbEscEeprom_Write_p, void* pContext_p)
{
    pAPPL_EEPROM_Write                  = SSC_EEP_write;
    SSC_callbacks_g.cbEscEepromWrite    = cbEscEeprom_Write_p;
    SSC_callbacks_g.pEscEepromWriteCtxt = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register reload EEPROM Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscEeprom_Reload_p	EEPROM Reload Callback
 *  \param[in]  pContext_p              EEPROM Reload Callback Context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerEepromReloadCb(ESCEEP_cbReload_t cbEscEeprom_Reload_p, void* pContext_p)
{
    pAPPL_EEPROM_Reload                     = SSC_EEP_reload;
    SSC_callbacks_g.cbEscEepromReload       = cbEscEeprom_Reload_p;
    SSC_callbacks_g.pEscEepromReloadCtxt    = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register store EEPROM Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscEeprom_Store_p     EEPROM Store Callback
 *  \param[in]  pContext_p              EEPROM Store Callback Context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerEepromStoreCb(ESCEEP_cbStore_t cbEscEeprom_Store_p, void* pContext_p)
{
    pAPPL_EEPROM_Store                  = SSC_EEP_store;
    SSC_callbacks_g.cbEscEepromStore    = cbEscEeprom_Store_p;
    SSC_callbacks_g.pEscEepromStoreCtxt = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Register EEPROM load register callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  cbEscEeprom_RegLd_p     Callback Function from ecSlvBackend.
 *  \param[in]  pContext_p              Callback context
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_registerEepromLdRegCb(ESCEEP_cbRegLd_t cbEscEeprom_RegLd_p, void* pContext_p)
{
    SSC_callbacks_g.cbEscEepromLdReg        = cbEscEeprom_RegLd_p;
    SSC_callbacks_g.pEscEepromLdRegCtxt     = pContext_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set CoE device Type
 *
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  deviceType_p    Device Type
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_COE_setDeviceType(uint32_t deviceType_p)
{
    u32Devicetype = deviceType_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set Sync Manager Sequence Error counter limit
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  maxSmErrLimit_p SyncManagerError limit (0 = default value = 255)
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_COE_setSMErrorMaxLimit(uint16_t maxSmErrLimit_p)
{
    if (0 == maxSmErrLimit_p)
    {
        sErrorSettings.u16SyncErrorCounterLimit = MAX_SM_EVENT_MISSED;
    }
    else
    {
        sErrorSettings.u16SyncErrorCounterLimit = maxSmErrLimit_p;
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set Error register
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  error_p Error value
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_setErrorRegister(uint16_t error_p)
{
    u16ErrorRegister = error_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Add CoE object to Dictionary
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pNewObjEntry_p  CoE Object entry
 *  \return     uint16_t        Error code
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_COE_addObjectToDic(TOBJECT* pNewObjEntry_p)
{
    return COE_AddObjectToDic(pNewObjEntry_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  This function removes an object from the object dictionary
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  index_p     object index
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_COE_removeDicEntry(uint16_t index_p)
{
    COE_RemoveDicEntry(index_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  AL Control indication
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  alControl_p     AL Control Value
 *  \param[in]  alStatusCode_p  AL Status Value
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_AL_ControlInd(uint8_t alControl_p, uint16_t alStatusCode_p)
{
    AL_ControlInd(alControl_p, alStatusCode_p);
}
/*! <!-- Description: -->
 *
 *  \brief
 *  EtherCAT state machine access from applInterface.h
 *
 *  \details
 *  From Beckhoff SSC:
 *
 *  This function shall be called by the application to trigger state transition in case
 *  of an application error or to complete a pending transition.
 *  If the function was called due to an error it shall be again if the error is gone.
 *  NOTE: state requests to a higher state than the current state are not allowed.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  alControl_p     AL Control Value
 *  \param[in]  alStatusCode_p  AL Status Value
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ECAT_StateChange(uint8_t alStatus_p, uint16_t alStatusCode_p)
{
    ECAT_StateChange(alStatus_p, alStatusCode_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Get AL Status
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     uint8_t     AL Status
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint8_t SSC_Al_status(void)
{
    return nAlStatus;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  State of Ecat Input Update
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     true: Inputs are updated, false: they are not
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
bool SSC_inputUpdateRunning(void)
{
    return bEcatInputUpdateRunning;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  State of Ecat Output Update
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     true: Outputs are updated, false: they are not
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
bool SSC_outputUpdateRunning(void)
{
    return bEcatOutputUpdateRunning;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Application is running?
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     true: Application running, false: it is not
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
bool SSC_ecatApplRunning(void)
{
    return bRunApplication;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set application runing
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  run_p   true: Application running, false: it is not.
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ecatApplSetRunning(bool run_p)
{
#if !(defined KUNBUS_SSC_EVAL) || (0==KUNBUS_SSC_EVAL)
    EC_API_SLV_SSC_setLicense(KUNBUS_SSC_LICENSE, strlen(KUNBUS_SSC_LICENSE));
#endif

    bRunApplication = run_p;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Get Receive Mailbox size
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     uint16_t    size of Receive Mailbox
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_MBX_receiveSize(void)
{
    return u16ReceiveMbxSize;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  This function puts a new Mailbox service in the Send Mailbox
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pMbx_p      Pointer to a Mailbox command to be sent (read by the Master
 *  \param[in]  flags_p     Send Flags
 *
 *  \return     0: Success - mailbox command could be stored in the send mailbox, error code otherwise.
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint8_t  SSC_MBX_sendReq(TMBX* pMbx_p, uint8_t flags_p)
{
    return MBX_MailboxSendReq(pMbx_p, flags_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  This function sends an Ethernet frame via EoE to the master
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pData_p     pointer to the Ethernet frame to be send (in case that STATIC_ETHERNET_BUFFER is 0 the memory will be freed after the last fragment was send).
 *  \param[in]  length_p    length of the Ethernet frame
 *  \return     0 = sending of frame started, 1 = frame could not be sent, try it later
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_EOE_sendFrameRequest(uint16_t* pData_p, uint16_t length_p)
{
    return EOE_SendFrameRequest(pData_p, length_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Get out of Empty Emergency Queue
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return Memory out of Emergency Queue
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
TEMCYMESSAGE* SSC_EMCY_getOutOfEmptyQueue(void)
{
    return GetOutOfEmptyEmcyQueue();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Get Emergenxy Buffer
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     Emergency Buffer
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
TEMCYMESSAGE* SSC_EMCY_getEmcyBuffer(void)
{
    return EMCY_GetEmcyBuffer();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  This function sends an emergency message
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pEmcy_p     emergency message to be sent.
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint8_t SSC_EMCY_sendEmergency(TEMCYMESSAGE* pEmcy_p)
{
    return EMCY_SendEmergency(pEmcy_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  This function puts an emergency buffer back to the empty queue
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pEmcy_p     empty emergency buffer.
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_EMCY_putInEmptyQueue(TEMCYMESSAGE* pEmcy_p)
{
    PutInEmptyEmcyQueue(pEmcy_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  This function puts an emergency message into the send queue
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pEmcy_p     emergency message.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *
 *  // required variables
 *  TEMCYMESSAGE* pEmcy = NULL;
 *
 *  ssc_PutInSendEmcyQueue(pEmcy);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_EMCY_putInSendQueue(TEMCYMESSAGE* pEmcy_p)
{
    PutInSendEmcyQueue(pEmcy_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Get Object Handle
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  index_p     Object index
 *  \return     TOBJECT*    object handle
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
OBJCONST TOBJECT* SSC_OBJ_getObjectHandle(uint16_t index_p)
{
    return OBJ_GetObjectHandle(index_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  This function calculates the bit offset of the entry in the object's variable
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  subindex_p      subindex of the entry
 *  \param[in]  pObjEntry_p     handle to the dictionary object returned by
 *                              OBJ_GetObjectHandle which was called before
 *  \return     bit offset of the entry in the variable
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_OBJ_getEntryOffset(uint8_t subindex_p, OBJCONST TOBJECT* pObjEntry_p)
{
    return OBJ_GetEntryOffset(subindex_p, pObjEntry_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  The function returns the Entry-Desc of a subindex to allow the application
 *  to define the object dictionary independent of the sdoserv-files
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pObjEntry_p     handle to the dictionary object returned by
 *                              OBJ_GetObjectHandle which was called before
 *  \param[in]  subindex_p      subindex of the requested object.
 *  \return     Pointer to the EntryDesc of the Subindex
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
OBJCONST TSDOINFOENTRYDESC* SSC_OBJ_getEntryDesc(OBJCONST TOBJECT OBJMEM * pObjEntry_p, uint8_t subindex_p)
{
    return OBJ_GetEntryDesc(pObjEntry_p, subindex_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Get BitMask
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  offset_p    array index
 *  \return     uint16_t    bitMask value
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_bitMask(uint16_t offset_p)
{
    return cBitMask[offset_p];
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set sync by user
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  set_p   Set Sync by User
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_setSyncByUser(bool set_p)
{
    bSyncSetByUser = set_p;
}

/*! <!-- Description: -->
 *
 *  \brief Checks if the new Sync type value is valid
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  index_p             index of the SyncManager Parameter object
 *  \param[in]  newSyncType_p       New value for the Sync Type (SubIndex 1)
 *  \return     Result of the value validation
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint8_t SSC_checkSyncTypeValue(uint16_t index_p, uint16_t newSyncType_p)
{
    return CheckSyncTypeValue(index_p, newSyncType_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Initialize Main from Stack
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     Error Code
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_BKHF_mainInit(void)
{
    return MainInit();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  MainLoop Trigger
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_BKHF_mainLoop(void)
{
    MainLoop();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Get ECAT Tomer increments per ms
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     Timer increments per ms
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint32_t SSC_ECAT_TIMER_INC_P_MS(void)
{
#ifdef USE_ECAT_TIMER
    return ECAT_TIMER_INC_P_MS;
#else
    return ecat_timer_inc_p_ms;
#endif
}

/*! <!-- Description: -->
 *
 *  \brief
 *  This function is called when an AMS response shall be sent
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pCmd_p          Pointer to the AMS commands
 *  \param[in]  amsErrCode_p    AMS Error Code
 *  \param[in]  dataLen_p       AMS Data Length
 *  \return     errorcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_AOE_AmsRes(AmsCmd* pCmd_p, uint16_t amsErrCode_p, uint16_t dataLen_p)
{
    return AOE_AmsRes(pCmd_p, amsErrCode_p, dataLen_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  This function is called when a fragmented AMS command is received
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pCmd_p          Pointer to the AMS commands
 *  \return     pointer to the completed AMS command. NULL in case of a pending command or an error
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
AmsCmd* SSC_AOE_FragmentedCmdInd(AmsCmd* pCmd_p)
{
    return AOE_FragmentedCmdInd(pCmd_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Get Object length
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  index_p           Index number.
 *  \param[in]  subindex_p        subIndex number.
 *  \param[in]  pObjEntry_p       Object entry instance.
 *  \param[in]  completeAccess_p  complete access flag.
 *  \return     pointer to the completed AMS command. NULL in case of a pending command or an error
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint32_t SSC_OBJ_GetObjectLength(uint16_t                       index_p
                                ,uint8_t                        subindex_p
                                ,OBJCONST TOBJECT OBJMEM*       pObjEntry_p
                                ,uint8_t                        completeAccess_p)
{
    return OBJ_GetObjectLength(index_p, subindex_p, pObjEntry_p, completeAccess_p);
}

//*************************************************************************************************

/*!
* \file ssc_kbStack.h
*
* \brief
* Beckhoff EC SSC Integration: Application callback interface
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

#if !(defined __SSC_KBSTACK_H__)
#define __SSC_KBSTACK_H__		1

#include <osal.h>
#include <objdef.h>

#include <ssc.h>

#if (defined PRUICSS_ETHERCAT_SUPPORT)
#include <ecat_def.h>

/* ESC define */
#define ESC_ADDR_AL_EVENT_REQ                   (0x220)
#define ESC_ADDR_MEMORY                         (0x1000)
#endif

#ifdef USE_ECAT_TIMER
#define ECAT_TIMER_INC_P_MS                     (1000000)
#else
#define ECAT_TIMER_INC_P_MS ecat_timer_inc_p_ms /* ARM frequency: Will be detected during bsp_init*/
volatile uint32_t ecat_timer_inc_p_ms;
#endif

#define HW_EscReadDWord(DWordValue, Address)    ((DWordValue)   = SSC_ESC_readDword(Address))
#define HW_EscRead                              SSC_ESC_read
#define HW_EscReadByteIsr(ByteValue, Address)   ((ByteValue)    = SSC_ESC_readByteIsr(Address))
#define HW_EscReadByte(ByteValue, Address)      ((ByteValue)    = SSC_ESC_readByte(Address))
#define HW_EscReadWord(WordValue, Address)      ((WordValue)    = SSC_ESC_readWord(Address))
#define HW_EscReadDWordIsr(DWordValue, Address) ((DWordValue)   = SSC_ESC_readDwordIsr(Address))
#define HW_EscReadMbxMem                        SSC_ESC_readMbxMem

#if (defined PRUICSS_ETHERCAT_SUPPORT)
#define HW_GetALEventRegister()                 (SSC_ESC_readWord(ESC_ADDR_AL_EVENT_REQ))
#define HW_GetALEventRegister_Isr()             (SSC_ESC_readWordIsr(ESC_ADDR_AL_EVENT_REQ))
#endif
#define HW_EscWrite                             SSC_ESC_write
#define HW_EscWriteWord                         SSC_ESC_writeWord
#define HW_EscWriteByte                         SSC_ESC_writeByte
#define HW_EscWriteIsr                          SSC_ESC_writeIsr
#define HW_EscReadIsr                           SSC_ESC_readIsr
#define HW_EscWriteMbxMem                       SSC_ESC_writeMbxMem


#define HW_EepromReload                         SSC_EEP_ESC_reload

#define HW_SetLed(RunLed, ErrLed)               SSC_setLedOut((RunLed), (ErrLed))

#define DISABLE_ESC_INT()                       SSC_MTX_global(1);
#define ENABLE_ESC_INT()                        SSC_MTX_global(0);
#define HW_GetTimer                             SSC_getTimerRegister
#define HW_ClearTimer                           SSC_clearTimerRegister

extern TOBJECT *ApplicationObjDic;

#if (defined __cplusplus)
extern "C" {
#endif

/* names fixed by Beckhoff SSC and therefore cannot follow coding conventions */
extern void     APPL_InputMapping           (uint16_t*      pData);
extern void     APPL_OutputMapping          (uint16_t*      pData);
extern void     APPL_Application            (void);
extern uint16_t APPL_GenerateMapping        (uint16_t*      pi16uInputSize
                                            ,uint16_t*      pi16OutputSize);
extern uint16_t APPL_StartMailboxHandler    (void);
extern uint16_t APPL_StopMailboxHandler     (void);
extern uint16_t APPL_StartInputHandler      (uint16_t*      pIntMask);
extern uint16_t APPL_StopInputHandler       (void);
extern uint16_t APPL_StartOutputHandler     (void);
extern uint16_t APPL_StopOutputHandler      (void);
extern void     APPL_AckErrorInd            (uint16_t       stateTrans);
/* /Beckhoff SSC */

extern void     SSC_setLedOut               (uint8_t        runLed_p
                                            ,uint8_t        errLed_p);

extern void     SSC_ESC_writeIsr            (uint8_t*       pData_p
                                            ,uint16_t       address_p
                                            ,uint16_t       length_p);
extern void     SSC_ESC_write               (uint8_t*       pData_p
                                            ,uint16_t       address_p
                                            ,uint16_t       length_p);
extern void     SSC_ESC_writeWord           (uint16_t       wordValue_p
                                            ,uint16_t       address_p);
extern void     SSC_ESC_writeByte           (uint8_t        byteValue_p
                                            ,uint16_t       address_p);
extern void     SSC_ESC_writeMbxMem         (uint8_t*       pData_p
                                            ,uint16_t       address_p
                                            ,uint16_t       length_p);

extern void     SSC_ESC_readIsr             (uint8_t*       pData_p
                                            ,uint16_t       address_p
                                            ,uint16_t       length_p);
extern void     SSC_ESC_read                (uint8_t*       pdata_p
                                            ,uint16_t       address_p
                                            ,uint16_t       length_p);
extern uint8_t  SSC_ESC_readByteIsr         (uint16_t       address_p);
extern uint8_t  SSC_ESC_readByte            (uint16_t       address_p);
extern uint16_t SSC_ESC_readWordIsr         (uint16_t       address_p);
extern uint16_t SSC_ESC_readWord            (uint16_t       address_p);
extern uint32_t SSC_ESC_readDwordIsr        (uint16_t       address_p);
extern uint32_t SSC_ESC_readDword           (uint16_t       address_p);
extern void     SSC_ESC_readMbxMem          (uint8_t*       pData_p
                                            ,uint16_t       address_p
                                            ,uint16_t       length_p);

extern uint16_t SSC_EEP_reload              (void);

extern void     SSC_MTX_global              (uint8_t        lock_p);
extern uint32_t SSC_getTimerRegister        (void);
extern void     SSC_clearTimerRegister      (void);

extern uint16_t SSC_EEP_read                (uint32_t       wordaddr_p);
extern uint16_t SSC_EEP_write               (uint32_t       wordaddr_p);
extern int32_t  SSC_EEP_ESC_reload          (void);
extern void     SSC_EEP_store               (void);

typedef void    (*cbEepStore_t)             (void);

extern void     SSC_checkTimer              (bool           ecatWaitForAlControlRes_p
                                            ,int16_t*       pEsmTimeoutCnt_p
                                            ,bool           ecatOutputUpdateRunning_p
                                            ,uint8_t        alStatus_p
                                            ,uint8_t*       pEcatRunLedState_p
                                            ,uint8_t*       pEcatErrLedState_p
                                            ,uint16_t*      pEePromStoreTimeoutValue_p
                                            ,uint16_t*      pEePromStoreTimeoutCnt_p
                                            ,cbEepStore_t   cbStoreEep_p
                                            ,bool           dc32Bit_p
                                            ,uint64_t*      pDcTimeStamp_p
                                            ,uint32_t*      pDcChkOverrunCnt_p);

#if (defined __cplusplus)
}
#endif

#endif /* __SSC_KBSTACK_H__ */

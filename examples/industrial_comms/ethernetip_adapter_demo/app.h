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


#ifndef KBEI_APP_H_INC
#define KBEI_APP_H_INC

#ifdef __cplusplus
extern "C" {
#endif

#define EI_APP_STACK_MAIN_TASK_STACK_SIZE_BYTE    0x1000
#define EI_APP_STACK_MAIN_TASK_STACK_SIZE         (EI_APP_STACK_MAIN_TASK_STACK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))

typedef struct APP_SApplication
{
    OSAL_TASK_EPriority_t taskPrio;
}APP_SApplication_t;

typedef struct APP_SHwal
{
    OSAL_TASK_EPriority_t taskPrio;
}APP_SHwal_t;

typedef struct APP_SLwip
{
    OSAL_TASK_EPriority_t taskPrio;
}APP_SLwip_t;

typedef struct APP_SAdapter
{
    OSAL_TASK_EPriority_t taskPrioPacket;
    OSAL_TASK_EPriority_t taskPrioStatistic;
}APP_SAdapter_t;

typedef struct APP_SParams
{
    APP_SApplication_t      application;
    APP_SHwal_t             hwal;
    APP_SLwip_t             lwip;
    APP_SAdapter_t          adapter;
    CUST_DRIVERS_SInit_t    customDrivers;

#if (defined CPU_LOAD_MONITOR) && (1==CPU_LOAD_MONITOR)
    CMN_CPU_API_SParams_t   cpuLoad;
    APP_WEBSRV_SParams_t    webServer;
#endif
}APP_SParams_t;

bool EI_APP_init(APP_SParams_t* pParams_p);
void EI_APP_run(void);
void EI_APP_mainTask(void* pvTaskArg_p);
void EI_APP_stackInit(APP_SParams_t* pParams_p);

void EI_APP_osErrorHandlerCb(uint32_t errorCode_p, bool fatal_p, uint8_t paraCnt_p, va_list argptr_p);
void EI_APP_osPrintfCb(void* pContext_p, const char* __restrict pFormat_p, va_list argptr_p);
void EI_APP_stackErrorHandlerCb(uint32_t i32uErrorCode_p, uint8_t bFatal_p, uint8_t i8uNumOfPara_p, va_list argptr_p);

void EI_APP_adpInit(void);

void EI_APP_cipAttributeCb(EI_API_CIP_NODE_T* pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, EI_API_CIP_EAr_t accessRight_p);
void EI_APP_cipGenerateContent(EI_API_CIP_NODE_T* pCipNode_p, uint16_t classId_p, uint16_t instanceId_p);

#ifdef  __cplusplus
}
#endif

#endif // KBEI_APP_H_INC

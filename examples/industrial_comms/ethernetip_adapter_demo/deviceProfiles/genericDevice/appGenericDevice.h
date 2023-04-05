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


#ifndef APPGENERICDEVICE_H
#define APPGENERICDEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

extern bool EI_APP_init(APP_SParams_t* pParams_p);
extern void EI_APP_run(void);
extern void EI_APP_stackInit(APP_SParams_t* pParams_p);

extern void EI_APP_osPrintfCb(void* pContext_p, const char* __restrict pFormat_p, va_list argptr_p);
extern void EI_APP_stackErrorHandlerCb(uint32_t i32uErrorCode_p, uint8_t bFatal_p, uint8_t i8uNumOfPara_p, va_list argptr_p);

extern void EI_APP_adpInit(void);

extern void EI_APP_cipAttributeCb(EI_API_CIP_NODE_T* pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, EI_API_CIP_EAr_t accessRight_p);
extern void EI_APP_cipGenerateContent(EI_API_CIP_NODE_T* pCipNode_p, uint16_t classId_p, uint16_t instanceId_p);

#ifdef  __cplusplus
}
#endif

#endif // APPGENERICDEVICE_H

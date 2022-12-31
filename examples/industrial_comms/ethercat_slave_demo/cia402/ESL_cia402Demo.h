/*!
 *  \example ESL_cia402Demo.h
 *
 *  \brief
 *  CiA 402 Callbacks Example
 * 
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2020-06-19
 *
 *  \copyright
 * 
 *  Copyright (c) 2020, Kunbus GmbH
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

#if !(defined __ESL_CIA402DEMO_H__)
#define __ESL_CIA402DEMO_H__		1


#include <ecSlvApi.h>
#include <ecSlvApi_Error.h>
#include <ecSlvApiDef.h>
#include <stdint.h>

#if (defined __cplusplus)
extern "C" {
#endif

 extern EC_API_SLV_EUserRetCodes_t EC_SLV_APP_startInputHandler(void* ctxt, uint16_t* pIntMask);
 extern void EC_SLV_APP_setObdValues(void* ctxt);
 extern void EC_SLV_APP_cia402Application(void* ctxt);
 extern void EC_SLV_APP_cia402LocalError(void* ctxt, uint16_t ErrorCode);
 extern uint32_t EC_SLV_APP_getCiA402ObjectEntryValue(void* pAppCtxt_p, EC_API_SLV_SCoE_ObjEntry_t* pObjectEntry_p,
                                                      uint16_t length_p, uint16_t* pValue_p);

#if (defined __cplusplus)
}
#endif



#endif /* __ESL_CIA402DEMO_H__ */

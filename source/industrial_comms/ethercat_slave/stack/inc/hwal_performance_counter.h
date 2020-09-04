/*!
* \file hwal_performance_counter.h
*
* \brief
* HWAL performance counter interface.
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-20
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

#if !(defined __HWAL_PERFORMANCE_COUNTER_H__)
#define __HWAL_PERFORMANCE_COUNTER_H__		1

#include <stdint.h>
#include <hwal.h>

typedef uint32_t (*HWAL_PRFCNT_init_t)               (void*                                 pHwalSpecHdl_p
                                                     ,HWAL_PRFCNT_EPrecision_t              precision_p);
typedef uint32_t (*HWAL_PRFCNT_getMpuCycles_t)       (void*                                 pHwalSpecHdl_p
                                                     ,uint32_t*                             pOverflows_p
                                                     ,uint32_t*                             pMpuClockCycles_p);
typedef uint32_t (*HWAL_PRFCNT_getTimeFromCycles_t)  (void*                                 pHwalSpecHdl_p
                                                     ,uint32_t                              overflows_p
                                                     ,uint32_t                              cyclesNum_p
                                                     ,HWAL_PRFCNT_ETimeUnit_t               timeUnit_p
                                                     ,uint32_t*                             pTimeHighDWord_p
                                                     ,uint32_t*                             pTimeLowDWord_p);
typedef uint32_t (*HWAL_PRFCNT_getDeltaTime_t)       (void*                                 pHwalSpecHdl_p
                                                     ,uint32_t                              startCyclesHighDWord_p
                                                     ,uint32_t                              startCyclesLowDword_p
                                                     ,uint32_t                              endCyclesHighDWord_p
                                                     ,uint32_t                              endCyclesLowDWord_p
                                                     ,HWAL_PRFCNT_ETimeUnit_t               timeUnit_p
                                                     ,uint32_t*                             pDeltaTimeHighDWord_p
                                                     ,uint32_t*                             pDeltaTimeLowDWord_p );
typedef uint32_t (*HWAL_PRFCNT_registerCallback_t)   (void*                                 pHwalSpecHdl_p
                                                     ,HWAL_PRFCNT_CBPrfCntOverflowHandler_t cbHandler_p);

typedef struct HWAL_PRFCNT_SHandle
{
    struct HWAL_PRFCNT_S
    {
        HWAL_PRFCNT_init_t              init;
        HWAL_PRFCNT_getMpuCycles_t      getMpuCycles;
        HWAL_PRFCNT_getTimeFromCycles_t getTimeFromCycles;
        HWAL_PRFCNT_getDeltaTime_t      getDeltaTime;
        HWAL_PRFCNT_registerCallback_t  registerCallback;
    }                                   counter;
    uint32_t                            mcuFreqInHz;
    uint32_t                            divider;
}HWAL_PRFCNT_SHandle_t;

#if (defined __cplusplus)
extern "C" {
#endif

extern uint32_t HWAL_PRFCNT_register                            (void*                      pHwalPrfCntHandle_p);
extern uint32_t HWAL_PRFCNT_unregister                          (void*                      pHwalPrfCntHandle_p);

#if (defined __cplusplus)
}
#endif

#endif /* __HWAL_PERFORMANCE_COUNTER_H__ */


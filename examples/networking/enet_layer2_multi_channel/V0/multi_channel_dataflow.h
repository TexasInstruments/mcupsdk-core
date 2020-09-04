/*
 * Copyright (c) Texas Instruments Incorporated 2022
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _MULTI_CHANNEL_DATAFLOW_H_
#define _MULTI_CHANNEL_DATAFLOW_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "multi_channel_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetApp_createClock(void);

void EnetApp_deleteClock(void);

void EnetApp_getCurrentTime(EnetApp_PerCtxt *perCtxt,
                             uint32_t *nanoseconds,
                             uint64_t *seconds);

void EnetApp_setPortTsEventPrms(CpswMacPort_TsEventCfg *tsPortEventCfg);

int32_t EnetApp_getRxTimestamp(EnetApp_PerCtxt *perCtxt,
                                EnetTimeSync_MsgType rxFrameType,
                                uint8_t rxPort,
                                uint16_t seqId,
                                uint32_t *nanoseconds,
                                uint64_t *seconds);

int32_t EnetApp_setCpswAleClassifier(EnetApp_PerCtxt *perCtxt);

int32_t EnetApp_getTxTimestamp(EnetApp_PerCtxt *perCtxt,
                                EnetTimeSync_MsgType txFrameType,
                                uint8_t txPort,
                                uint16_t seqId,
                                uint32_t *nanoseconds,
                                uint64_t *seconds);

void EnetApp_timerCallback(ClockP_Object *clkInst, void * arg);

void EnetApp_tickTask(void *args);

int32_t EnetApp_openDma(EnetApp_PerCtxt *perCtxt, uint32_t perCtxtIndex);

void EnetApp_closeDma(EnetApp_PerCtxt *perCtxt, uint32_t perCtxtIndex);

void EnetApp_initTxFreePktQ(EnetDma_PktQ *freePktInfoQ, uint32_t txCh);

void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh, uint32_t rxChIdx);

uint32_t EnetApp_retrieveFreeTxPkts(EnetDma_TxChHandle hTxCh, EnetDma_PktQ *txPktInfoQ);

void EnetApp_createRxTask(EnetApp_PerCtxt *perCtxt);

void EnetApp_destroyRxTask(EnetApp_PerCtxt *perCtxt);

void EnetApp_rxTask(void *args);

void EnetApp_rxTaskPTP(void *args);

void EnetApp_cptsEvtNotifyFxn(void *pAppData, CpswCpts_Event *pEventInfo);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* _MULTI_CHANNEL_DATAFLOW_H_ */

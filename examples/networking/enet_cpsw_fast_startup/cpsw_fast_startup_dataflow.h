/*
 * Copyright (C) 2023 Texas Instruments Incorporated
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

#ifndef _CPSW_FAST_STARTUP_DATAFLOW_H_
#define _CPSW_FAST_STARTUP_DATAFLOW_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cpsw_fast_startup_common.h"

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

int32_t EnetApp_openDma(void);

void EnetApp_closeDma(void);

void EnetApp_initTxFreePktQ(void);

void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);

uint32_t EnetApp_retrieveFreeTxPkts(void);

void EnetApp_postTxEvent(void* pArg);

void EnetApp_postRxEvent(void* pArg);

void EnetApp_preparePktQ(EnetDma_PktQ* pktQueue);

void EnetApp_sendPktQ(EnetDma_PktQ* pPktQ);

void EnetApp_handleRxPkt(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* _CPSW_FAST_STARTUP_DATAFLOW_H_ */

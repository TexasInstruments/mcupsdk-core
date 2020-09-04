/**********************************************************************
* Copyright (C) 2012-2021 Cadence Design Systems, Inc.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* WARNING: This file is auto-generated using api-generator utility.
*          api-generator: 13.05.b3ee589
*          Do not edit it manually.
**********************************************************************
* Layer interface for the Cadence DMA controller
**********************************************************************/

/* parasoft-begin-suppress METRICS-18-3 "Follow the Cyclomatic Complexity limit of 10, DRV-4789" */
/* parasoft-begin-suppress METRIC.CC-3 "Follow the Cyclomatic Complexity limit of 30, DRV-4417" */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4, DRV-4790" */
/* parasoft-begin-suppress METRICS-41-3 "Number of blocks of comments per statement, DRV-4926" */
/* parasoft-begin-suppress MISRA2012-RULE-8_7 "Functions and objects should not be defined with external linkage if they are referenced in only one translation unit, DRV-4139" */

/**
 * This file contains sanity API functions. The purpose of sanity functions
 * is to check input parameters validity. They take the same parameters as
 * original API functions and return 0 on success or CDN_EINVAL on wrong parameter
 * value(s).
 */

#ifndef CUSBDMA_SANITY_H
#define CUSBDMA_SANITY_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "cdn_stdtypes.h"
#include "cdn_errno.h"
#include "cusbdma_if.h"

uint32_t CUSBDMA_ChannelParamsSF(const CUSBDMA_ChannelParams *obj);
uint32_t CUSBDMA_ConfigSF(const CUSBDMA_Config *obj);
uint32_t CUSBDMA_DmaChannelSF(const CUSBDMA_DmaChannel *obj);
uint32_t CUSBDMA_DmaControllerSF(const CUSBDMA_DmaController *obj);
uint32_t CUSBDMA_DmaTransferParamSF(const CUSBDMA_DmaTransferParam *obj);

uint32_t CUSBDMA_SanityFunction1(const CUSBDMA_Config* config, const CUSBDMA_SysReq* sysReq);
uint32_t CUSBDMA_SanityFunction2(const CUSBDMA_DmaController* pD, const CUSBDMA_Config* config);
uint32_t CUSBDMA_SanityFunction3(const CUSBDMA_DmaController* pD);
uint32_t CUSBDMA_SanityFunction4(const CUSBDMA_DmaController* pD, const CUSBDMA_DmaChannel** channelPtr, const CUSBDMA_ChannelParams* channelParams);
uint32_t CUSBDMA_SanityFunction5(const CUSBDMA_DmaController* pD, const CUSBDMA_DmaChannel* channel);
uint32_t CUSBDMA_SanityFunction7(const CUSBDMA_DmaController* pD, const CUSBDMA_DmaChannel* channel, const CUSBDMA_DmaTransferParam* params);

#define CUSBDMA_ProbeSF CUSBDMA_SanityFunction1
#define CUSBDMA_InitSF CUSBDMA_SanityFunction2
#define CUSBDMA_DestroySF CUSBDMA_SanityFunction3
#define CUSBDMA_ChannelAllocSF CUSBDMA_SanityFunction4
#define CUSBDMA_ChannelResetSF CUSBDMA_SanityFunction5
#define CUSBDMA_ChannelReleaseSF CUSBDMA_SanityFunction5
#define CUSBDMA_ChannelProgramSF CUSBDMA_SanityFunction7
#define CUSBDMA_ChannelTriggerSF CUSBDMA_SanityFunction5
#define CUSBDMA_ChannelUpdateStateSF CUSBDMA_SanityFunction5
#define CUSBDMA_ChannelSetMaxPktSzSF CUSBDMA_SanityFunction5
#define CUSBDMA_ChannelHandleStallSF CUSBDMA_SanityFunction5
#define CUSBDMA_ChannelFreeHeadTrbChSF CUSBDMA_SanityFunction5


#ifdef __cplusplus
}
#endif

#endif  /* CUSBDMA_SANITY_H */

/* parasoft-end-suppress MISRA2012-RULE-8_7 */
/* parasoft-end-suppress METRICS-41-3 */
/* parasoft-end-suppress METRICS-39-3 */
/* parasoft-end-suppress METRICS-36-3 */
/* parasoft-end-suppress METRIC.CC-3 */
/* parasoft-end-suppress METRICS-18-3 */

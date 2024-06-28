/*
 *  Copyright (c) 2024 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file ipc_notify_soc.h
 *
 *  \brief IPC Low Level Driver AM65XX SOC specific file.
 */
#ifndef IPC_NOTIFY_SOC_H_
#define IPC_NOTIFY_SOC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/hw_include/csl_types.h>

/* max limits for HW mailbox's */
#define IPC_NOTIFY_MAILBOX_MAX_INSTANCES     (12U)
#define IPC_NOTIFY_MAILBOX_USER_CNT          (4U)

/**
 * \brief Mailbox interrupt router configuration
 */
typedef struct IpcNotify_MbIntrConfig_s
{
    uint32_t   priority;
    uint32_t   eventId;
    uint32_t   inputIntrNum;
    uint32_t   outputIntrNum;
}IpcNotify_MbIntrConfig;

/* functions for the Mailbox Interrupt router handling */
int32_t IpcNotify_getMailboxIntrRouterCfg(uint32_t selfId, uint32_t clusterId, uint32_t userId, IpcNotify_MbIntrConfig* cfg, uint32_t cnt);
int32_t IpcNotify_sciclientIrqRelease(uint16_t remoteId, uint32_t clusterId, uint32_t userId, uint32_t intNumber);
int32_t IpcNotify_sciclientIrqSet(uint16_t remoteId, uint32_t clusterId, uint32_t userId, uint32_t intNumber);
int32_t IpcNotify_getIntNumRange(uint32_t coreIndex, uint16_t *rangeStartP, uint16_t *rangeNumP);
uintptr_t IpcNotify_getMailboxBaseAddr(uint32_t clusterId);
int32_t IpcNotify_setIntrRtr(uint32_t selfCoreId, uint32_t remoteCoreId, uint32_t mailboxBaseAddr, uint32_t hwFifoId, uint32_t userId);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef IPC_NOTIFY_SOC_H_ */


/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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

#include <kernel/dpl/DebugP.h>
#include "tx_api.h"

#define DEBUG_SHM_LOG_READER_TASK_PRI  (TX_MAX_PRIORITIES - 1u) /* lowest priority */
#define DEBUG_SHM_LOG_READER_TASK_STACK_SIZE (4U*1024U/sizeof(uint32_t))
static uint32_t  gDebugShmLogReaderTaskStack[DEBUG_SHM_LOG_READER_TASK_STACK_SIZE] __attribute__((aligned(32)));
static TX_THREAD gDebugShmLogReaderTaskObj;

void DebugP_shmLogReaderTaskCreate(void);

void DebugP_shmLogReaderTaskMain(void *args);

static void DebugP_shmLogReaderTaskMain_Wrapper(ULONG args);

void DebugP_shmLogReaderTaskCreate(void)
{
    UINT tx_ret;

    tx_ret = tx_thread_create(&gDebugShmLogReaderTaskObj,
                              "debug_shm_log_reader",
                              DebugP_shmLogReaderTaskMain_Wrapper,
                              0u,
                              gDebugShmLogReaderTaskStack,
                              DEBUG_SHM_LOG_READER_TASK_STACK_SIZE,
                              DEBUG_SHM_LOG_READER_TASK_PRI,
                              DEBUG_SHM_LOG_READER_TASK_PRI,
                              0u,
                              TX_TRUE);

    DebugP_assertNoLog(tx_ret != TX_SUCCESS);
}


static void DebugP_shmLogReaderTaskMain_Wrapper(ULONG args)
{
    DebugP_shmLogReaderTaskMain((void *)args);
}

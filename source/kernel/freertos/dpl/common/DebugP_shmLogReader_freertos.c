/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_SHM_LOG_READER_TASK_PRI  (1) /* lowest priority */ 
#define DEBUG_SHM_LOG_READER_TASK_STACK_SIZE (4U*1024U/sizeof(configSTACK_DEPTH_TYPE))
static StackType_t  gDebugShmLogReaderTaskStack[DEBUG_SHM_LOG_READER_TASK_STACK_SIZE] __attribute__((aligned(32)));
static StaticTask_t gDebugShmLogReaderTaskObj;
static TaskHandle_t gDebugShmLogReaderTask;

void DebugP_shmLogReaderTaskMain(void *args);

void DebugP_shmLogReaderTaskCreate()
{
	/* This task is created at highest priority, it should create more tasks and then delete itself */
    gDebugShmLogReaderTask = xTaskCreateStatic( 
                                    DebugP_shmLogReaderTaskMain,   /* Pointer to the function that implements the task. */
                                    "debug_shm_log_reader",        /* Text name for the task.  This is to facilitate debugging only. */
                                    DEBUG_SHM_LOG_READER_TASK_STACK_SIZE, /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                    NULL,                          /* We are not using the task parameter. */
                                    DEBUG_SHM_LOG_READER_TASK_PRI, /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                    gDebugShmLogReaderTaskStack,   /* pointer to stack base */
                                    &gDebugShmLogReaderTaskObj );    /* pointer to statically allocated task object memory */
    DebugP_assertNoLog(gDebugShmLogReaderTask != NULL);
}


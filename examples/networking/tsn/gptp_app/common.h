/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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

#ifndef __COMMON_H__
#define __COMMON_H__

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define MAX_KEY_SIZE            (256)

typedef enum {
	ENETAPP_UNICONF_TASK_IDX,
	ENETAPP_GPTP_TASK_IDX,
	ENETAPP_LLDP_TASK_IDX,
	ENETAPP_MAX_TASK_IDX
} EnetApp_TsnTask_Idx_t;

typedef struct EnetApp_ModuleCtx EnetApp_ModuleCtx_t;
typedef int (*EnetApp_OnModuleDBInit)(EnetApp_ModuleCtx_t* mdctx,
                                      yang_db_runtime_dataq_t *ydrd);
typedef void* (*EnetApp_OnModuleStart)(void *arg);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef struct
{
    char *dbName; // Specify same DB for all modules.
    bool initFlag;
    ucman_data_t ucCtx;
    char netdev[MAX_NUMBER_ENET_DEVS][CB_MAX_NETDEVNAME];
    uint8_t netdevSize;
    CB_SEM_T ucReadySem;
} EnetApp_Ctx_t;

struct EnetApp_ModuleCtx
{
    bool stopFlag;
    int taskPriority;
    CB_THREAD_T hTaskHandle;
    const char *taskName;
    uint8_t *stackBuffer;
    uint32_t stackSize;
    EnetApp_OnModuleDBInit onModuleDBInit;
    EnetApp_OnModuleStart onModuleRunner;
    EnetApp_Ctx_t *appCtx;
    bool enable;
};

typedef struct
{
    char *name;
    char *val;
} EnetApp_DbNameVal_t;

typedef struct
{
    char *name;
    int item;
    int val;
} EnetApp_DbIntVal_t;

#endif

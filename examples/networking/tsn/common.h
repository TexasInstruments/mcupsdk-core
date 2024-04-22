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
#include "tsnapp_porting.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define MAX_KEY_SIZE            (256)
#define TSNAPP_UNUSED_ARG(x)    (void)x

#ifdef DISABLE_FAT_FS
#define UNICONF_CONF_FILE_NUM   (0)
#define INTERFACE_CONFFILE_PATH (NULL)
#define UNICONF_DBFILE_PATH     (NULL)
#else
#define UNICONF_CONF_FILE_NUM   (1)
#endif //DISABLE_FAT_FS

#ifndef AVTP_TALKER_NUM
#define AVTP_TALKER_NUM         (2)
#endif

#ifndef AVTP_LISTENER_NUM
#define AVTP_LISTENER_NUM       (2)
#endif

typedef enum {
    ENETAPP_UNICONF_TASK_IDX,
    ENETAPP_NETCONF_TASK_IDX,
    ENETAPP_GPTP_TASK_IDX,
    ENETAPP_LLDP_TASK_IDX,
    ENETAPP_AVTPD_TASK_IDX,
    ENETAPP_CRF_TALKER_TASK_IDX,
    ENETAPP_CRF_LISTENER_TASK_IDX,
    ENETAPP_ACF_TASK_IDX,
    ENETAPP_TALKER_TASK_IDX,
    ENETAPP_LISTENER_TASK_IDX = ENETAPP_TALKER_TASK_IDX + AVTP_TALKER_NUM,
    ENETAPP_EST_TASK_IDX = ENETAPP_LISTENER_TASK_IDX + AVTP_LISTENER_NUM,
    ENETAPP_CBS_TASK_IDX,
    ENETAPP_MAX_TASK_IDX
} EnetApp_TsnTask_Idx_t;

typedef struct {
    uc_dbald *dbald;
    yang_db_runtime_dataq_t *ydrd;
    uc_notice_data_t *ucntd;
} EnetApp_dbArgs;

typedef struct EnetApp_ModuleCtx EnetApp_ModuleCtx_t;
typedef int (*EnetApp_OnModuleDBInit)(EnetApp_ModuleCtx_t* mdctx,
                                      EnetApp_dbArgs *dbargs);
typedef void* (*EnetApp_OnModuleStart)(void *arg);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef struct
{
    char *dbName; // Specify same DB for all modules.
    ucman_data_t ucCtx;
    char netdev[MAX_NUMBER_ENET_DEVS][CB_MAX_NETDEVNAME];
    uint8_t netdevSize;
    UC_NOTICE_SIG_T ucReadySem;
    bool dbInitFlag;
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

#if defined (SOC_AM263X)
#define LISTNER_VERIFY_ENABLE   0
#else
#define LISTNER_VERIFY_ENABLE   1
#endif
#endif

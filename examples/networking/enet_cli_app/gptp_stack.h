/*
 * Copyright (C) 2024 Texas Instruments Incorporated
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

/*!
 * \file  gptp_stack.h
 *
 * \brief Header file for gptp_stack.c
 */

#ifndef GPTP_STACK_H_
#define GPTP_STACK_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <tsn_combase/tilld/lldtype.h>
#include <tsn_combase/combase.h>
#include <tsn_unibase/unibase_binding.h>
#include <tsn_uniconf/yangs/yang_db_runtime.h>
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_gptp/gptpman.h>
#include <tsn_gptp/tilld/lld_gptp_private.h>
#include <tsn_gptp/gptpconf/gptpgcfg.h>
#include <tsn_gptp/gptpconf/xl4-extmod-xl4gptp.h>
#include <tsn_uniconf/yangs/ieee1588-ptp-tt_access.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include "gptp_log.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define UNICONF_TASK_PRIORITY 2
#define GPTP_TASK_PRIORITY 2

#define TSN_TSK_STACK_SIZE 16U * 1024U
#define TSN_TSK_STACK_ALIGN 32U

#define MAX_KEY_SIZE 256

#define AVTP_TALKER_NUM 2
#define AVTP_LISTENER_NUM 2

#define TSNAPP_LOGLEVEL "4,ubase:45,cbase:45,uconf:45,gptp:45,lldp:45,avtp:45,nconf:45"

#ifdef DISABLE_FAT_FS
#define UNICONF_CONF_FILE_NUM   (0)
#define INTERFACE_CONFFILE_PATH (NULL)
#define UNICONF_DBFILE_PATH     (NULL)
#else
#define UNICONF_CONF_FILE_NUM   (1)
#define INTERFACE_CONFFILE_PATH "/sd0/conffiles/interface.conf"
#define UNICONF_DBFILE_PATH     "/sd0/uniconfdb/example.bin"
#endif //DISABLE_FAT_FS

/* Status flags for TSN */
#define TSN_IDLE 0
#define TSN_NO_MAC -1
#define TSN_RUNNING 1

typedef struct EnetTsn_DbArgs_s EnetTsn_DbArgs;
typedef struct EnetTsn_ModuleCtx_s EnetTsn_ModuleCtx;
typedef int (*EnetTsn_OnModuleDBInit)(EnetTsn_ModuleCtx *mdctx,
        EnetTsn_DbArgs *dbargs);
typedef void* (*EnetTsn_OnModuleStart)(void *arg);

typedef enum
{
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
} EnetTsn_TsnTask_Idx;

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct EnetTsn_Ctx_s
{
    char *dbName;
    ucman_data_t ucCtx;
    char netdev[MAX_NUMBER_ENET_DEVS][CB_MAX_NETDEVNAME];
    uint8_t netdevSize;
    UC_NOTICE_SIG_T ucReadySem;
    bool dbInitFlag;
} EnetTsn_Ctx;

struct EnetTsn_ModuleCtx_s
{
    bool stopFlag;
    int taskPriority;
    CB_THREAD_T hTaskHandle;
    const char *taskName;
    uint8_t *stackBuffer;
    uint32_t stackSize;
    EnetTsn_OnModuleDBInit onModuleDBInit;
    EnetTsn_OnModuleStart onModuleRunner;
    EnetTsn_Ctx *appCtx;
    bool enable;
};

struct EnetTsn_DbArgs_s
{
    uc_dbald *dbald;
    yang_db_runtime_dataq_t *ydrd;
    uc_notice_data_t *ucntd;
};

typedef struct EnetTsn_GptpOpt_s
{
    char *devlist;
    const char **confFiles;
    int domainNum;
    int domains[GPTP_MAX_DOMAINS];
    int instNum;
    int numConf;
} EnetTsn_GptpOpt;

typedef struct EnetTsn_DbNameVal_s
{
    char *name;
    char *val;
} EnetTsn_DbNameVal;

typedef struct EnetTsn_DbIntVal_s
{
    char *name;
    int item;
    int val;
} EnetTsn_DbIntVal;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

BaseType_t EnetCLI_ptpService(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

void EnetTsn_stopTsn(void);

BaseType_t EnetCLI_stopTsn(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* GPTP_STACK_H_ */

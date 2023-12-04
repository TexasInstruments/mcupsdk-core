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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <tsn_combase/combase.h>
#include <tsn_unibase/unibase_binding.h>
#include <tsn_uniconf/yangs/tsn_data.h>
#include <tsn_uniconf/yangs/yang_db_runtime.h>
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_gptp/gptpman.h>
#include <tsn_gptp/tilld/lld_gptp_private.h>
#include <tsn_gptp/gptpconf/gptpgcfg.h>
#include <tsn_gptp/gptpconf/xl4-extmod-xl4gptp.h>
#include "tsn_uniconf/yangs/ieee1588-ptp_access.h"
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include "debug_log.h"
#include "tsninit.h"
#include "common.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define UNICONF_TASK_PRIORITY   (2)
#define GPTP_TASK_PRIORITY      (2)

#define GPTP_TASK_NAME          "gptp2d_task"
#define UNICONF_TASK_NAME       "uniconf_task"

#define MAX_KEY_SIZE            (256)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int EnetApp_initDb(void);
static void *EnetApp_uniconfTask(void *arg);
static int EnetApp_uniconfInit(EnetApp_ModuleCtx_t* modCtx,
                               yang_db_runtime_dataq_t *ydrd);
static int EnetApp_startUniconfTask(void);
static int EnetApp_startTask(EnetApp_ModuleCtx_t* ctx);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern EnetApp_ModuleCtx_t gModCtxTable[ENETAPP_MAX_TASK_IDX];
extern EnetApp_Ctx_t gAppCtx;

/* ========================================================================== */
/*                            Local Variables                                */
/* ========================================================================== */
static uint8_t gUniconfStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void *EnetApp_uniconfTask(void *arg)
{
    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)arg;
    EnetApp_Ctx_t *appCtx = modCtx->appCtx;

    appCtx->ucCtx.ucmode = UC_CALLMODE_THREAD|UC_CALLMODE_UNICONF;
    appCtx->ucCtx.stoprun = &modCtx->stopFlag;
    appCtx->ucCtx.hwmod="";
    appCtx->ucCtx.ucmanstart = appCtx->ucReadySem;
    appCtx->ucCtx.dbname = appCtx->dbName;

    DPRINT("%s: dbname: %s", __func__, appCtx->dbName ? appCtx->dbName : "NULL");

    return uniconf_main(&appCtx->ucCtx);
}

static int EnetApp_uniconfInit(EnetApp_ModuleCtx_t* modCtx, yang_db_runtime_dataq_t *ydrd)
{
    EnetApp_Ctx_t *appCtx = modCtx->appCtx;
    char buffer[MAX_KEY_SIZE]={0};
    int res=-1;
    int i;

    for (i = 0; i < appCtx->netdevSize; i++)
    {
        snprintf(buffer, sizeof(buffer),
                 "/ietf-interfaces/interfaces/interface|name:%s|/enabled",
                 appCtx->netdev[i]);
        res=yang_db_runtime_put_oneline(ydrd, buffer, (char*)"true",
                                        YANG_DB_ONHW_NOACTION);
        if (res != 0) {
            DPRINT("%s: yang_db_runtime_put_oneline failed=%d", __func__, res);
        }
    }
    return res;
}

// Can be read from cfg files, or in case of no db file is specified, init runtime config
static int EnetApp_initDb(void)
{
    EnetApp_ModuleCtx_t *mod;
    uc_dbald *dbald = NULL;
    xl4_data_data_t *xdd = NULL;
    yang_db_runtime_dataq_t *ydrd = NULL;
    int res = 0;
    int timeout_ms = 500;
    int i;

    do {
        res = CB_SEM_WAIT(&gAppCtx.ucReadySem);
        if (res != 0)
        {
            DPRINT("Failed to wait for the uniconf \n");
            break;
        }

        res = uniconf_ready(gAppCtx.dbName, UC_CALLMODE_THREAD, timeout_ms);
        if (res)
        {
            DPRINT("The uniconf must be run first !");
            break;
        }
        dbald = uc_dbal_open(gAppCtx.dbName, "w", UC_CALLMODE_THREAD);
        if (!dbald)
        {
            DPRINT("Failed to open DB!");
            res = -1;
            break;
        }
        xdd = xl4_data_init(dbald);
        if (!xdd)
        {
            DPRINT("Failed to init xl4 data");
            res = -1;
            break;
        }
        ydrd = yang_db_runtime_init(xdd, dbald, NULL);
        if (!ydrd)
        {
            DPRINT("Failed to init yang db runtime");
            res = -1;
            break;
        }

        for (i = 0; i < ENETAPP_MAX_TASK_IDX; i++)
        {
            mod = &gModCtxTable[i];
            if ((mod->enable == true) && (mod->onModuleDBInit != NULL))
            {
                mod->onModuleDBInit(mod, ydrd);
            }
        }

    } while (0);
    if (ydrd)
    {
        yang_db_runtime_close(ydrd);
    }
    if (xdd)
    {
        xl4_data_close(xdd);
    }
    if (dbald)
    {
        uc_dbal_close(dbald, UC_CALLMODE_THREAD);
    }

    return res;
}

int EnetApp_initTsnByCfg(AppTsnCfg_t *cfg)
{
    int i = 0;
    unibase_init_para_t init_para;

    Logger_init(cfg->consoleOutCb);

    ubb_default_initpara(&init_para);
    init_para.ub_log_initstr="4,ubase:45,cbase:45,uconf:45,gptp:55,lldp:45";
    if (cfg->consoleOutCb)
    {
        init_para.cbset.console_out=LOG_OUTPUT;
    }

    unibase_init(&init_para);
    ubb_memory_out_init(NULL, 0);

    while (cfg->netdevs[i] != NULL && i < MAX_NUMBER_ENET_DEVS)
    {
        if (strlen(cfg->netdevs[i]) < CB_MAX_NETDEVNAME)
        {
            strcpy(&gAppCtx.netdev[gAppCtx.netdevSize][0],
                   cfg->netdevs[i]);
            gAppCtx.netdevSize++;
        }
        i++;
    }
    return 0;
}

void EnetApp_deInitTsn(void)
{
    Logger_deInit();
    unibase_close();
}

static int EnetApp_startTask(EnetApp_ModuleCtx_t *modCtx)
{
    cb_tsn_thread_attr_t attr;
    int res = 0;

    modCtx->stopFlag = false;
    cb_tsn_thread_attr_init(&attr, modCtx->taskPriority,
                            modCtx->stackSize, modCtx->taskName);
    cb_tsn_thread_attr_set_stackaddr(&attr, modCtx->stackBuffer);
    if (CB_THREAD_CREATE(&modCtx->hTaskHandle,
                         &attr, modCtx->onModuleRunner, modCtx) < 0)
    {
        modCtx->stopFlag = true;
        DPRINT("Failed to start %s !", modCtx->taskName);
        res = -1;
    }
    else
    {
        DPRINT("Start: %s", modCtx->taskName);
    }
    return res;
}

int EnetApp_startTsn(void)
{
    int i;
    int res = 0;
    EnetApp_ModuleCtx_t *mod;
    EnetApp_ModuleCtx_t uniconfModCtx = {
        .enable = true,
        .stopFlag = true,
        .taskPriority = UNICONF_TASK_PRIORITY,
        .taskName = UNICONF_TASK_NAME,
        .stackBuffer = gUniconfStackBuf,
        .stackSize = sizeof(gUniconfStackBuf),
        .onModuleDBInit = EnetApp_uniconfInit,
        .onModuleRunner = EnetApp_uniconfTask,
        .appCtx = &gAppCtx
    };

    memcpy(&gModCtxTable[ENETAPP_UNICONF_TASK_IDX],
           &uniconfModCtx, sizeof(EnetApp_ModuleCtx_t));

    if (CB_SEM_INIT(&gAppCtx.ucReadySem, 0, 0) < 0)
    {
        DPRINT("Failed to initialize ucReadySem semaphore!");
        res = -1;
    }

    if (res == 0)
    {
        res = EnetApp_startTask(&gModCtxTable[ENETAPP_UNICONF_TASK_IDX]);
    }
    if (res == 0)
    {
        res = EnetApp_initDb();
    }
    else
    {
        DPRINT("Failed to init db!");
    }
    if (res == 0)
    {
        for (i = 0; i < ENETAPP_MAX_TASK_IDX; i++)
        {
            mod = &gModCtxTable[i];
            if ((mod->enable == true) && (mod->stopFlag == true))
            {
                res = EnetApp_startTask(mod);
                if (res != 0)
                {
                    break;
                }
            }
        }
    }
    return res;
}

void EnetApp_stopTsn(void)
{
    int i;

    if (gAppCtx.ucReadySem != NULL)
    {
        CB_SEM_DESTROY(&gAppCtx.ucReadySem);
    }
    for (i = 0; i < ENETAPP_MAX_TASK_IDX; i++)
    {
        CB_THREAD_JOIN(gModCtxTable[i].hTaskHandle, NULL);
        gModCtxTable[i].hTaskHandle = NULL;
    }
    memset(&gAppCtx, 0, sizeof(gAppCtx));
    memset(gModCtxTable, 0, sizeof(gModCtxTable));
}

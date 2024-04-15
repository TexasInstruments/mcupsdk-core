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
#include <tsn_uniconf/yangs/yang_db_runtime.h>
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_gptp/gptpman.h>
#include <tsn_gptp/tilld/lld_gptp_private.h>
#include <tsn_gptp/gptpconf/gptpgcfg.h>
#include <tsn_gptp/gptpconf/xl4-extmod-xl4gptp.h>
#include <tsn_uniconf/yangs/ieee1588-ptp-tt_access.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include "debug_log.h"
#include "tsninit.h"
#include "common.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define UNICONF_TASK_PRIORITY   (2)
#define UNICONF_TASK_NAME       "uniconf_task"

#ifndef TSNAPP_LOGLEVEL
#define TSNAPP_LOGLEVEL "4,ubase:45,cbase:45,uconf:45,gptp:55,lldp:45,avtp:45,nconf:45"
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int EnetApp_initDb(void);
static void *EnetApp_uniconfTask(void *arg);
static int EnetApp_uniconfInit(EnetApp_ModuleCtx_t* modCtx, EnetApp_dbArgs *dbargs);
static int EnetApp_startUniconfTask(void);
static int EnetApp_startTask(EnetApp_ModuleCtx_t* modCtx, int moduleIdx);

/* ========================================================================== */
/*                     External Function Declarations                         */
/* ========================================================================== */

#ifdef NETCONF_ENABLED
extern int EnetApp_addNetconfModCtx(EnetApp_ModuleCtx_t *modCtxTbl);
#endif //NETCONF_ENABLED

#ifdef AVTP_ENABLED
extern int EnetApp_avtpInit(EnetApp_ModuleCtx_t *modCtxTbl);
extern void EnetApp_avtpDeinit(void);
#endif //AVTP_ENABLED

#ifdef LLDP_ENABLED
extern int EnetApp_addLldpModCtx(EnetApp_ModuleCtx_t *modCtxTbl);
#endif //LLDP_ENABLED

#ifdef GPTP_ENABLED
extern int EnetApp_addGptpModCtx(EnetApp_ModuleCtx_t *modCtxTbl);
#endif //GPTP_ENABLED

#ifdef EST_APP_ENABLED
extern int EnetApp_addEstAppModCtx(EnetApp_ModuleCtx_t *modCtxTbl);
#endif /* EST_APP_ENABLED */

#ifdef CBS_APP_ENABLED
extern int EnetApp_addCbsAppModCtx(EnetApp_ModuleCtx_t *modCtxTbl);
#endif /* EST_APP_ENABLED */
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
EnetApp_Ctx_t gAppCtx =
{
    .dbName = UNICONF_DBFILE_PATH,
};
EnetApp_ModuleCtx_t gModCtxTable[ENETAPP_MAX_TASK_IDX];

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
    const char *configFiles[2] = {INTERFACE_CONFFILE_PATH, NULL};

    appCtx->ucCtx.ucmode = UC_CALLMODE_THREAD|UC_CALLMODE_UNICONF;
    appCtx->ucCtx.stoprun = &modCtx->stopFlag;
    appCtx->ucCtx.hwmod="";
    appCtx->ucCtx.ucmanstart = &appCtx->ucReadySem;
    appCtx->ucCtx.dbname = appCtx->dbName;
    appCtx->ucCtx.configfiles = configFiles;
    appCtx->ucCtx.numconfigfile = UNICONF_CONF_FILE_NUM;

    DPRINT("%s: dbname: %s", __func__, appCtx->dbName ? appCtx->dbName : "NULL");

    return uniconf_main(&appCtx->ucCtx);
}

static int EnetApp_uniconfInit(EnetApp_ModuleCtx_t* modCtx, EnetApp_dbArgs *dbargs)
{
#ifdef DISABLE_FAT_FS
    EnetApp_Ctx_t *appCtx = modCtx->appCtx;
    char buffer[MAX_KEY_SIZE]={0};
    int res=-1;
    int i;

    for (i = 0; i < appCtx->netdevSize; i++)
    {
        snprintf(buffer, sizeof(buffer),
                 "/ietf-interfaces/interfaces/interface|name:%s|/enabled",
                 appCtx->netdev[i]);
        res=yang_db_runtime_put_oneline(dbargs->ydrd, buffer, (char*)"true",
                                        YANG_DB_ONHW_NOACTION);
        if (res != 0) {
            DPRINT("%s: yang_db_runtime_put_oneline failed=%d", __func__, res);
        }
    }
    return res;
#else
    TSNAPP_UNUSED_ARG(dbargs);
    return 0;
#endif
}

// Can be read from cfg files, or in case of no db file is specified, init runtime config
static int EnetApp_initDb(void)
{
    EnetApp_ModuleCtx_t *mod;
    EnetApp_dbArgs dbargs;
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
        dbargs.dbald = uc_dbal_open(gAppCtx.dbName, "w", UC_CALLMODE_THREAD);
        if (!dbargs.dbald)
        {
            DPRINT("Failed to open DB!");
            res = -1;
            break;
        }
        dbargs.ydrd = yang_db_runtime_init(dbargs.dbald, NULL);
        if (!dbargs.ydrd)
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
                mod->onModuleDBInit(mod, &dbargs);
            }
        }

    } while (0);
    if (dbargs.ydrd)
    {
        yang_db_runtime_close(dbargs.ydrd);
    }
    if (dbargs.dbald)
    {
        uc_dbal_close(dbargs.dbald, UC_CALLMODE_THREAD);
    }

    return res;
}

static bool EnetApp_isDBFileInit(char *filename)
{
#ifndef DISABLE_FAT_FS
    EnetApp_fsInfo_t fsInfo;
    return (FSTAT(filename, &fsInfo) == FSSTAT_OK);
#else
    return false;
#endif /* !DISABLE_FAT_FS */
}

int EnetApp_initTsnByCfg(AppTsnCfg_t *cfg)
{
    int i = 0;
    int res = 0;
    unibase_init_para_t initPara;
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

    Logger_init(cfg->consoleOutCb);

    ubb_default_initpara(&initPara);

    initPara.ub_log_initstr = TSNAPP_LOGLEVEL;
    if (cfg->consoleOutCb)
    {
        initPara.cbset.console_out=LOG_OUTPUT;
    }

    unibase_init(&initPara);
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

    if (gAppCtx.dbName != NULL)
    {
        /* set DB File initialized flag to be used later during onModuleDBInit */
        gAppCtx.dbInitFlag = EnetApp_isDBFileInit(gAppCtx.dbName);
        DPRINT("DB File Initialized: %s", gAppCtx.dbInitFlag ? "True" : "False");
    }

    if (CB_SEM_INIT(&gAppCtx.ucReadySem, 0, 0) < 0)
    {
        DPRINT("Failed to initialize ucReadySem semaphore!");
        res = -1;
    }

    memcpy(&gModCtxTable[ENETAPP_UNICONF_TASK_IDX],
           &uniconfModCtx, sizeof(EnetApp_ModuleCtx_t));

#ifdef GPTP_ENABLED
    if (res == 0)
    {
        res = EnetApp_addGptpModCtx(gModCtxTable);
    }
#endif //GPTP_ENABLED

#ifdef LLDP_ENABLED
    if (res == 0)
    {
        res = EnetApp_addLldpModCtx(gModCtxTable);
    }
#endif //LLDP_ENABLED

#ifdef AVTP_ENABLED
    if (res == 0)
    {
        res = EnetApp_avtpInit(gModCtxTable);
    }
#endif //AVTP_ENABLED

#ifdef NETCONF_ENABLED
    if (res == 0)
    {
        res = EnetApp_addNetconfModCtx(gModCtxTable);
    }
#endif //NETCONF_ENABLED

#ifdef EST_APP_ENABLED
    if (res == 0)
    {
        res = EnetApp_addEstAppModCtx(gModCtxTable);
    }
#endif //EST_APP_ENABLED

#ifdef CBS_APP_ENABLED
    if (res == 0)
    {
        res = EnetApp_addCbsAppModCtx(gModCtxTable);
    }
#endif //CBS_APP_ENABLED
    return res;
}

void EnetApp_deInitTsn(void)
{
#ifdef AVTP_ENABLED
    EnetApp_avtpDeinit();
#endif //AVTP_ENABLED
    if (gAppCtx.ucReadySem != NULL)
    {
        CB_SEM_DESTROY(&gAppCtx.ucReadySem);
        gAppCtx.ucReadySem = NULL;
    }
    memset(&gAppCtx, 0, sizeof(gAppCtx));
    memset(gModCtxTable, 0, sizeof(gModCtxTable));

    Logger_deInit();
    unibase_close();
}

static int EnetApp_startTask(EnetApp_ModuleCtx_t *modCtx, int moduleIdx)
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
        if (moduleIdx == ENETAPP_UNICONF_TASK_IDX)
        {
            /* initDb must be run right after UNICONF is started and
             * before starting any other tasks. */
            res = EnetApp_initDb();
        }
    }
    return res;
}

int EnetApp_startTsn(void)
{
    int i;
    int res = 0;

    for (i = 0; i < ENETAPP_MAX_TASK_IDX; i++)
    {
        res = EnetApp_startTsnModule(i);
        if (res != 0)
        {
            break;
        }
    }
    return res;
}

void EnetApp_stopTsn(void)
{
    int i;

    /* The uniconf with index = 0 must be stopped finally */
    for (i = ENETAPP_MAX_TASK_IDX-1; i >= 0; i--)
    {
        EnetApp_stopTsnModule(i);
    }
}

int EnetApp_startTsnModule(int moduleIdx)
{
    int res = 0;

    if ((moduleIdx >= 0) && (moduleIdx < ENETAPP_MAX_TASK_IDX))
    {
        EnetApp_ModuleCtx_t *mod;
        mod = &gModCtxTable[moduleIdx];

        if ((mod->enable == true) && (mod->stopFlag == true))
        {
            res = EnetApp_startTask(mod, moduleIdx);
        }
    }
    else
    {
        res = -1;
    }
    return res;
}

void EnetApp_stopTsnModule(int moduleIdx)
{
    if ((moduleIdx >= 0) && (moduleIdx < ENETAPP_MAX_TASK_IDX))
    {
        EnetApp_ModuleCtx_t *mod;
        mod = &gModCtxTable[moduleIdx];
        if (mod->hTaskHandle != NULL)
        {
            mod->stopFlag = true;
            CB_THREAD_JOIN(mod->hTaskHandle, NULL);
            mod->hTaskHandle = NULL;
            DPRINT("Task: %s is terminated.", mod->taskName);
        }
    }
}

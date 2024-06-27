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
 * \file  gptp_stack.c
 *
 * \brief This file contains the source codes for running gPTP stack in the
 *         CLI application.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_common.h"
#include "gptp_stack.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetTsn_enableTsSync();

static int EnetTsn_initTsn(int32_t priority1, int32_t priority2,
        int32_t sync_interval);

static int EnetTsn_initTsnByCfg(char *netdevs[]);

static int EnetTsn_uniconfInit(EnetTsn_ModuleCtx *modCtx,
        EnetTsn_DbArgs *dbargs);

static void* EnetTsn_uniconfTask(void *arg);

static bool EnetTsn_isDBFileInit(char *filename);

static int EnetTsn_addGptpModCtx(EnetTsn_ModuleCtx *modCtxTbl);

static int EnetTsn_gptpDbInit(EnetTsn_ModuleCtx *modCtx, EnetTsn_DbArgs *dbargs);

static int EnetTsn_gptpYangConfig(yang_db_runtime_dataq_t *ydrd, int instance,
        int domain, EnetTsn_Ctx *appCtx);

static void EnetTsn_cfgGptpDefaultDs(yang_db_runtime_dataq_t *ydrd,
        int instance, int domain, bool dbInitFlag);

static void EnetTsn_cfgGptpPortDs(yang_db_runtime_dataq_t *ydrd, int instance,
        int domain, int port_index, bool dbInitFlag);

static void* EnetTsn_gptpTask(void *arg);

static int EnetTsn_gptpNonYangConfig(uint8_t instance);

static int EnetTsn_lldCfgUpdateCb(cb_socket_lldcfg_update_t *update_cfg);

static int EnetTsn_startTsn(void);

static int EnetTsn_startTsnModule(int moduleIdx);

static int EnetTsn_startTask(EnetTsn_ModuleCtx *modCtx, int moduleIdx);

static int EnetTsn_initDb(void);

static void EnetTsn_stopTsnModule(int moduleIdx);

static bool EnetTsn_IsMacAddrSet(uint8_t *mac);

static int EnetTsn_AddVlan(Enet_Handle hEnet, uint32_t coreId, uint32_t vlanId);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Variable used to configure gptp */
static char EnetTsn_netDevices[ENET_SYSCFG_MAX_MAC_PORTS][CB_MAX_NETDEVNAME] = {
    0 };

EnetTsn_Ctx EnetTsn_appCtx = { .dbName = UNICONF_DBFILE_PATH, };
EnetTsn_GptpOpt EnetTsn_appGptpOpt = { .confFiles = NULL, .domainNum =
        GPTP_MAX_DOMAINS,
#if GPTP_MAX_DOMAINS == 1
    .domains = {0},
#elif GPTP_MAX_DOMAINS == 2
    .domains = {0, 1},
#else
#error "Only support 2 domains"
#endif
    .instNum = 0, .numConf = 0, };

static EnetTsn_DbNameVal EnetTsn_gptpPortDsRw[] = { { "port-enable", "true" }, {
    "log-announce-interval", "0" }, { "gptp-cap-receipt-timeout", "3" }, {
    "announce-receipt-timeout", "3" }, { "initial-log-announce-interval", "0" },
    { "initial-log-sync-interval", "-3" }, { "sync-receipt-timeout", "3" }, {
        "initial-log-pdelay-req-interval", "0" }, { "allowed-lost-responses",
        "9" }, { "allowed-faults", "9" }, { "mean-link-delay-thresh",
        "0x75300000" }, /* (30000 << 16)*/
};

static EnetTsn_DbNameVal EnetTsn_gptpPortDsRo[] = {
    { "log-sync-interval", "-3" }, { "minor-version-number", "1" }, {
        "current-log-sync-interval", "-3" }, { "current-log-gptp-cap-interval",
        "3" }, { "current-log-pdelay-req-interval", "0" }, {
        "initial-one-step-tx-oper", "1" }, { "current-one-step-tx-oper", "1" },
    { "use-mgt-one-step-tx-oper", "false" }, { "mgt-one-step-tx-oper", "1" }, };

static EnetTsn_DbNameVal EnetTsn_gptpDefaultDsRw[] = { { "priority1", "248" }, {
    "priority2", "248" }, { "external-port-config-enable", "false" }, {
    "clock-quality/clock-class", "cc-default" }, {
    "clock-quality/clock-accuracy", "ca-time-accurate-to-250-ns" }, {
    "clock-quality/offset-scaled-log-variance", "0x436a" } };

static EnetTsn_DbNameVal EnetTsn_gptpDefaultDsRo[] = { { "time-source",
    "internal-oscillator" }, { "ptp-timescale", "true" }, };

static EnetTsn_DbIntVal EnetTsn_gptpNonYangDs[] =
        { { "SINGLE_CLOCK_MODE", XL4_EXTMOD_XL4GPTP_SINGLE_CLOCK_MODE, 1 }, {
            "USE_HW_PHASE_ADJUSTMENT",
            XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT, 1 }, {
            "CLOCK_COMPUTE_INTERVAL_MSEC",
            XL4_EXTMOD_XL4GPTP_CLOCK_COMPUTE_INTERVAL_MSEC, 100 }, {
            "FREQ_OFFSET_IIR_ALPHA_START_VALUE",
            XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_START_VALUE, 1 }, {
            "FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE",
            XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE, 4 }, {
            "PHASE_OFFSET_IIR_ALPHA_START_VALUE",
            XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_START_VALUE, 1 }, {
            "PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE",
            XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE, 4 }, {
            "MAX_DOMAIN_NUMBER", XL4_EXTMOD_XL4GPTP_MAX_DOMAIN_NUMBER,
            GPTP_MAX_DOMAINS },
#if GPTP_MAX_DOMAINS == 2
    {"CMLDS_MODE", XL4_EXTMOD_XL4GPTP_CMLDS_MODE, 1},
    {"SECOND_DOMAIN_THIS_CLOCK", XL4_EXTMOD_XL4GPTP_SECOND_DOMAIN_THIS_CLOCK, 1}
#endif
        };

uint8_t EnetTsn_gptpDmaCh;

EnetTsn_ModuleCtx EnetTsn_modCtxTable[ENETAPP_MAX_TASK_IDX];

static uint8_t EnetTsn_uniconfStackBuf[TSN_TSK_STACK_SIZE] __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));
static uint8_t EnetTsn_gptpStackBuf[TSN_TSK_STACK_SIZE] __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

BaseType_t EnetCLI_ptpService(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t priority1 = 248;
    int32_t priority2 = 248;
    int32_t sync_interval = -3;
    char *parameter;
    BaseType_t paramLen;
    uint32_t paramCnt = 1;
    uint8_t dmaChNum;
    bool isStart;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
            &paramLen);
    while (parameter != NULL)
    {
        if (paramCnt == 1)
        {
            if (strncmp(parameter, "start", paramLen) == 0)
                isStart = true;
            else if (strncmp(parameter, "stop", paramLen) == 0)
                isStart = false;
            else
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid Arg\r\n");
                return pdFALSE;
            }
            paramCnt++;
        }
        else if (paramCnt == 2)
        {
            dmaChNum = atoi(parameter);
            paramCnt++;
        }
        else if (strncmp("-p1", parameter, paramLen) == 0)
        {
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            priority1 = atoi(parameter);
            paramCnt += 2;
        }
        else if (strncmp("-p2", parameter, paramLen) == 0)
        {
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            priority2 = atoi(parameter);
            paramCnt += 2;
        }
        else if (strncmp("-s", parameter, paramLen) == 0)
        {
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            sync_interval = atoi(parameter);
            paramCnt += 2;
        }
        else
        {
            snprintf(writeBuffer, writeBufferLen, "Invalid Args\r\n");
            return pdFALSE;
        }
        parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                &paramLen);
    }

    if(isStart)
    {

        if (EnetApp_inst.tsnFlag == TSN_NO_MAC)
        {
            snprintf(writeBuffer, writeBufferLen, "No MAC address assigned\r\n");
            return pdFALSE;
        }
        else if (EnetApp_inst.tsnFlag == TSN_RUNNING)
        {
            snprintf(writeBuffer, writeBufferLen,
                    "gPTP stack is already running\r\n");
            return pdFALSE;
        }
        else if (dmaChNum >= ENET_SYSCFG_TX_CHANNELS_NUM
                || dmaChNum >= ENET_SYSCFG_RX_FLOWS_NUM)
        {
            snprintf(writeBuffer, writeBufferLen,
                    "Tx/Rx channel %d does not exist\r\n", dmaChNum);
            return pdFALSE;
        }
        if (EnetApp_inst.rxDmaCh[dmaChNum] != CH_CLOSE
                || EnetApp_inst.txDmaCh[dmaChNum] != CH_CLOSE)
        {
            snprintf(writeBuffer, writeBufferLen,
                    "Tx/Rx channel %d is already in use\r\n", dmaChNum);
            return pdFALSE;
        }

        /* Enable PPS output */
        EnetTsn_enableTsSync();

        /* Start gPTP stack */
        EnetTsn_gptpDmaCh = dmaChNum;
        EnetTsn_initTsn(priority1, priority2, sync_interval);
        snprintf(writeBuffer, writeBufferLen, "Started gPTP stack\r\n");
        EnetApp_inst.tsnFlag = TSN_RUNNING;
    }
    else
    {
        if (EnetApp_inst.tsnFlag == TSN_IDLE)
        {
            snprintf(writeBuffer, writeBufferLen,
                    "gPTP stack is not running\r\n");
            return pdFALSE;
        }
        int i;
        /* The uniconf with index = 0 must be stopped finally */
        for (i = ENETAPP_MAX_TASK_IDX - 1; i >= 0; i--)
        {
            EnetTsn_stopTsnModule(i);
        }
    }
    return pdFALSE;
}

BaseType_t EnetCLI_stopTsn(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    if (EnetApp_inst.tsnFlag != TSN_RUNNING)
    {
        snprintf(writeBuffer, writeBufferLen, "No TSN stack is active\r\n");
        return pdFALSE;
    }
    EnetTsn_stopTsn();
    snprintf(writeBuffer, writeBufferLen, "Stopped gPTP task\r\n");
    EnetApp_inst.tsnFlag = TSN_IDLE;
    return pdFALSE;
}

void EnetTsn_stopTsn()
{

}

#define LOG_BUFFER_SIZE (1024)
void ConsolePrint(const char *pcString, ...)
{
    va_list args;
    char buffer[LOG_BUFFER_SIZE];

    va_start(args, pcString);
    vsnprintf(buffer, sizeof(buffer), pcString, args);
    va_end(args);

    DebugP_log("%s", buffer);
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void EnetTsn_enableTsSync()
{
    Enet_IoctlPrms prms;
    CpswCpts_OutputBitSel bitSelect;
    int32_t status;

    bitSelect = CPSW_CPTS_TS_OUTPUT_BIT_17;
    ENET_IOCTL_SET_IN_ARGS(&prms, &bitSelect);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to set TS SYNC OUT BIT : %d\r\n", status);
    }
    return;
}

static int EnetTsn_initTsn(int32_t priority1, int32_t priority2,
        int32_t sync_interval)
{
    sprintf(EnetTsn_gptpDefaultDsRw[0].val, "%d", priority1);
    sprintf(EnetTsn_gptpDefaultDsRw[1].val, "%d", priority2);
    sprintf(EnetTsn_gptpPortDsRw[5].val, "%d", sync_interval);
    sprintf(EnetTsn_gptpPortDsRo[0].val, "%d", sync_interval);
    sprintf(EnetTsn_gptpPortDsRo[2].val, "%d", sync_interval);

    lld_ethdev_t ethdevs[MAX_NUMBER_ENET_DEVS] = { 0 };
    int i;
    int res = 0;
    char *netdevs[LLDENET_MAX_PORTS + 1];

    for (i = 0; i < EnetApp_inst.numMacPorts; i++)
    {
        snprintf(&EnetTsn_netDevices[i][0], CB_MAX_NETDEVNAME, "tilld%d", i);
        netdevs[i] = &EnetTsn_netDevices[i][0];
        ethdevs[i].netdev = EnetTsn_netDevices[i];
        ethdevs[i].macport = EnetApp_inst.macPort[i];
        if (i == 0)
        {
            memcpy(ethdevs[i].srcmac, EnetApp_inst.hostMacAddr,
            ENET_MAC_ADDR_LEN);
        }
    }
    netdevs[i] = NULL;
    if (EnetTsn_initTsnByCfg(netdevs) < 0)
    {
        EnetAppUtils_print("Failed to int tsn!\r\n");
    }
    if (cb_lld_init_devs_table(ethdevs, i, EnetApp_inst.enetType,
            EnetApp_inst.instId) < 0)
    {
        EnetAppUtils_print("Failed to int devs table!\r\n");
    }
    cb_socket_set_lldcfg_update_cb(EnetTsn_lldCfgUpdateCb);

    if (EnetTsn_startTsn() < 0)
    {
        EnetAppUtils_print("Failed to start TSN App!\r\n");
    }
    EnetAppUtils_print("%s:TSN app start done!\r\n", __func__);

    return res;
}

static int EnetTsn_initTsnByCfg(char *netdevs[])
{
    int i = 0;
    int res = 0;
    unibase_init_para_t initPara;
    EnetTsn_ModuleCtx uniconfModCtx = { .enable = true, .stopFlag = true,
        .taskPriority = UNICONF_TASK_PRIORITY, .taskName = "uniconf_task",
        .stackBuffer = EnetTsn_uniconfStackBuf, .stackSize =
                sizeof(EnetTsn_uniconfStackBuf), .onModuleDBInit =
                EnetTsn_uniconfInit, .onModuleRunner = EnetTsn_uniconfTask,
        .appCtx = &EnetTsn_appCtx };

    Logger_init(ConsolePrint);

    ubb_default_initpara(&initPara);

    initPara.ub_log_initstr = TSNAPP_LOGLEVEL;
    initPara.cbset.console_out = LOG_OUTPUT;

    unibase_init(&initPara);
    ubb_memory_out_init(NULL, 0);

    while (netdevs[i] != NULL && i < MAX_NUMBER_ENET_DEVS)
    {
        if (strlen(netdevs[i]) < CB_MAX_NETDEVNAME)
        {
            strcpy(&EnetTsn_appCtx.netdev[EnetTsn_appCtx.netdevSize][0],
                    netdevs[i]);
            EnetTsn_appCtx.netdevSize++;
        }
        i++;
    }

    if (EnetTsn_appCtx.dbName != NULL)
    {
        /* set DB File initialized flag to be used later during onModuleDBInit */
        EnetTsn_appCtx.dbInitFlag = EnetTsn_isDBFileInit(EnetTsn_appCtx.dbName);
        EnetAppUtils_print("DB File Initialized: %s\r\n",
                EnetTsn_appCtx.dbInitFlag ? "True" : "False");
    }

    if (CB_SEM_INIT(&EnetTsn_appCtx.ucReadySem, 0, 0) < 0)
    {
        EnetAppUtils_print("Failed to initialize ucReadySem semaphore!\r\n");
        res = -1;
    }

    memcpy(&EnetTsn_modCtxTable[ENETAPP_UNICONF_TASK_IDX], &uniconfModCtx,
            sizeof(EnetTsn_ModuleCtx));

    if (res == 0)
    {
        res = EnetTsn_addGptpModCtx(EnetTsn_modCtxTable);
    }

    return res;
}

static int EnetTsn_uniconfInit(EnetTsn_ModuleCtx *modCtx,
        EnetTsn_DbArgs *dbargs)
{
#ifdef DISABLE_FAT_FS
    EnetTsn_Ctx *appCtx = modCtx->appCtx;
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
            EnetAppUtils_print("%s: yang_db_runtime_put_oneline failed=%d\r\n", __func__, res);
        }
    }
    return res;
#else
    (void) dbargs;
    return 0;
#endif
}

static void* EnetTsn_uniconfTask(void *arg)
{
    EnetTsn_ModuleCtx *modCtx = (EnetTsn_ModuleCtx*) arg;
    EnetTsn_Ctx *appCtx = modCtx->appCtx;
    const char *configFiles[2] = { INTERFACE_CONFFILE_PATH, NULL };

    appCtx->ucCtx.ucmode = UC_CALLMODE_THREAD | UC_CALLMODE_UNICONF;
    appCtx->ucCtx.stoprun = &modCtx->stopFlag;
    appCtx->ucCtx.hwmod = "";
    appCtx->ucCtx.ucmanstart = &appCtx->ucReadySem;
    appCtx->ucCtx.dbname = appCtx->dbName;
    appCtx->ucCtx.configfiles = configFiles;
    appCtx->ucCtx.numconfigfile = UNICONF_CONF_FILE_NUM;

    EnetAppUtils_print("%s: dbname: %s\r\n", __func__,
            appCtx->dbName ? appCtx->dbName : "NULL");

    return uniconf_main(&appCtx->ucCtx);
}

static bool EnetTsn_isDBFileInit(char *filename)
{
#ifndef DISABLE_FAT_FS
    FF_Stat_t fsInfo;
    return (ff_stat(filename, &fsInfo) == 0);
#else
    return false;
#endif /* !DISABLE_FAT_FS */
}

static int EnetTsn_addGptpModCtx(EnetTsn_ModuleCtx *modCtxTbl)
{
    EnetTsn_ModuleCtx gptpModCtx = { .enable = true, .stopFlag = true,
        .taskPriority = GPTP_TASK_PRIORITY, .taskName = "gptp2d_task",
        .stackBuffer = EnetTsn_gptpStackBuf, .stackSize =
                sizeof(EnetTsn_gptpStackBuf), .onModuleDBInit =
                EnetTsn_gptpDbInit, .onModuleRunner = EnetTsn_gptpTask,
        .appCtx = &EnetTsn_appCtx };
    memcpy(&modCtxTbl[ENETAPP_GPTP_TASK_IDX], &gptpModCtx,
            sizeof(EnetTsn_ModuleCtx));

    return 0;
}

static int EnetTsn_gptpDbInit(EnetTsn_ModuleCtx *modCtx, EnetTsn_DbArgs *dbargs)
{
    EnetTsn_Ctx *appCtx = modCtx->appCtx;
    int res = 0;
    int i;

    if (EnetTsn_appGptpOpt.numConf == 0)
    {
        /* There is no config file is specified, set config file for gptp*/
        for (i = 0; i < EnetTsn_appGptpOpt.domainNum; i++)
        {
            res = EnetTsn_gptpYangConfig(dbargs->ydrd,
                    EnetTsn_appGptpOpt.instNum, EnetTsn_appGptpOpt.domains[i],
                    appCtx);
            if (res)
            {
                EnetAppUtils_print("Failed to set gptp run time config\r\n");
            }
        }
    }
    return res;
}

static int EnetTsn_gptpYangConfig(yang_db_runtime_dataq_t *ydrd, int instance,
        int domain, EnetTsn_Ctx *appCtx)
{
    char buffer[MAX_KEY_SIZE];
    char value_str[32];
    const char *plus;
    int i, res = 0;

    EnetAppUtils_print("%s:domain=%d\r\n", __func__, domain);

    do
    {
        /* skip setting of 'rw' yang configs when db is already initialized */
        if (!appCtx->dbInitFlag)
        {
            plus = ((instance | domain) != 0) ? "+" : "";
            snprintf(buffer, sizeof(buffer),
                    "/ieee1588-ptp-tt/ptp/instance-domain-map%s", plus);
            snprintf(value_str, sizeof(value_str), "0x%04x",
                    instance << 8 | domain);
            yang_db_runtime_put_oneline(ydrd, buffer, value_str,
                    YANG_DB_ONHW_NOACTION);
        }

        /* set for default-ds */
        EnetTsn_cfgGptpDefaultDs(ydrd, instance, domain, appCtx->dbInitFlag);

// portindex starts from 1
        for (i = 0; i < appCtx->netdevSize; i++)
        {
            /* skip setting of 'rw' yang configs when db is already initialized */
            if (!appCtx->dbInitFlag)
            {
                snprintf(buffer, sizeof(buffer),
                        "/ieee1588-ptp-tt/ptp/instances/instance|instance-index:%d,%d|"
                                "/ports/port|port-index:%d|/underlying-interface",
                        instance, domain, i + 1);
                yang_db_runtime_put_oneline(ydrd, buffer, appCtx->netdev[i],
                        YANG_DB_ONHW_NOACTION);
            }

            /* set for port-ds */
            EnetTsn_cfgGptpPortDs(ydrd, instance, domain, i + 1,
                    appCtx->dbInitFlag);
        }

        /* skip setting of 'rw' yang configs when db is already initialized */
        if (!appCtx->dbInitFlag)
        {
            /* disable performance by default */
            snprintf(buffer, sizeof(buffer),
                    "/ieee1588-ptp-tt/ptp/instances/instance|instance-index:%d,%d|"
                            "/performance-monitoring-ds/enable", instance,
                    domain);
            yang_db_runtime_put_oneline(ydrd, buffer, "false",
                    YANG_DB_ONHW_NOACTION);
        }
    } while (0);

    return res;
}

static void EnetTsn_cfgGptpDefaultDs(yang_db_runtime_dataq_t *ydrd,
        int instance, int domain, bool dbInitFlag)
{
    int i;
    char buffer[MAX_KEY_SIZE];

    if (!dbInitFlag)
    {
        for (i = 0;
                i
                        < sizeof(EnetTsn_gptpDefaultDsRw)
                                / sizeof(EnetTsn_gptpDefaultDsRw[0]); i++)
        {
            snprintf(buffer, sizeof(buffer),
                    "/ieee1588-ptp-tt/ptp/instances/instance|instance-index:%d,%d|"
                            "/default-ds/%s", instance, domain,
                    EnetTsn_gptpDefaultDsRw[i].name);
            yang_db_runtime_put_oneline(ydrd, buffer,
                    EnetTsn_gptpDefaultDsRw[i].val, YANG_DB_ONHW_NOACTION);
        }
    }

    for (i = 0;
            i
                    < sizeof(EnetTsn_gptpDefaultDsRo)
                            / sizeof(EnetTsn_gptpDefaultDsRo[0]); i++)
    {
        snprintf(buffer, sizeof(buffer),
                "/ieee1588-ptp-tt/ptp/instances/instance|instance-index:%d,%d|"
                        "/default-ds/%s", instance, domain,
                EnetTsn_gptpDefaultDsRo[i].name);
        yang_db_runtime_put_oneline(ydrd, buffer,
                EnetTsn_gptpDefaultDsRo[i].val, YANG_DB_ONHW_NOACTION);
    }
}

static void EnetTsn_cfgGptpPortDs(yang_db_runtime_dataq_t *ydrd, int instance,
        int domain, int port_index, bool dbInitFlag)
{
    int i;
    char buffer[MAX_KEY_SIZE];

    if (!dbInitFlag)
    {
        for (i = 0;
                i
                        < sizeof(EnetTsn_gptpPortDsRw)
                                / sizeof(EnetTsn_gptpPortDsRw[0]); i++)
        {
            snprintf(buffer, sizeof(buffer),
                    "/ieee1588-ptp-tt/ptp/instances/instance|instance-index:%d,%d|"
                            "/ports/port|port-index:%d|/port-ds/%s", instance,
                    domain, port_index, EnetTsn_gptpPortDsRw[i].name);

            yang_db_runtime_put_oneline(ydrd, buffer,
                    EnetTsn_gptpPortDsRw[i].val, YANG_DB_ONHW_NOACTION);
        }
    }

    for (i = 0;
            i < sizeof(EnetTsn_gptpPortDsRo) / sizeof(EnetTsn_gptpPortDsRo[0]);
            i++)
    {
        snprintf(buffer, sizeof(buffer),
                "/ieee1588-ptp-tt/ptp/instances/instance|instance-index:%d,%d|"
                        "/ports/port|port-index:%d|/port-ds/%s", instance,
                domain, port_index, EnetTsn_gptpPortDsRo[i].name);

        yang_db_runtime_put_oneline(ydrd, buffer, EnetTsn_gptpPortDsRo[i].val,
                YANG_DB_ONHW_NOACTION);
    }
}

static void* EnetTsn_gptpTask(void *arg)
{
    int i;
    int res = 0;
    EnetTsn_ModuleCtx *modCtx = (EnetTsn_ModuleCtx*) arg;
    EnetTsn_Ctx *appCtx = modCtx->appCtx;
    const char *netdevs[CB_MAX_NETDEVNAME];

    for (i = 0; i < appCtx->netdevSize; i++)
    {
        netdevs[i] = appCtx->netdev[i];
    }

    res = gptpgcfg_init(appCtx->dbName, EnetTsn_appGptpOpt.confFiles,
            EnetTsn_appGptpOpt.instNum, true, EnetTsn_gptpNonYangConfig);
    if (res != 0)
    {
        EnetAppUtils_print("%s: gptpgcfg_init() error\r\n", __func__);
    }
    else
    {
        /* This function has a true loop inside */
        res = gptpman_run(EnetTsn_appGptpOpt.instNum, netdevs,
                appCtx->netdevSize,
                NULL, &modCtx->stopFlag);
        if (res != 0)
        {
            EnetAppUtils_print("%s: gptpman_run() error\r\n", __func__);
        }
    }
    gptpgcfg_close(EnetTsn_appGptpOpt.instNum);
    return NULL;
}

static int EnetTsn_gptpNonYangConfig(uint8_t instance)
{
    int i;
    int res;

    for (i = 0;
            i < sizeof(EnetTsn_gptpNonYangDs) / sizeof(EnetTsn_gptpNonYangDs[0]);
            i++)
    {
        res = gptpgcfg_set_item(instance, EnetTsn_gptpNonYangDs[i].item,
        YDBI_CONFIG, &EnetTsn_gptpNonYangDs[i].val,
                sizeof(EnetTsn_gptpNonYangDs[i].val));
        if (res == 0)
        {
            EnetAppUtils_print("%s:XL4_EXTMOD_XL4GPTP_%s=%d\r\n", __func__,
                    EnetTsn_gptpNonYangDs[i].name,
                    EnetTsn_gptpNonYangDs[i].val);
        }
        else
        {
            EnetAppUtils_print("%s: failed to set nonyange param: %s\r\n",
                    __func__, EnetTsn_gptpNonYangDs[i].name);
            break;
        }
    }
    return res;
}

static int EnetTsn_lldCfgUpdateCb(cb_socket_lldcfg_update_t *update_cfg)
{
    if (update_cfg->proto == ETH_P_1588)
    {
        update_cfg->numRxChannels = 1;
        update_cfg->dmaTxChId = EnetTsn_gptpDmaCh;
        update_cfg->dmaRxChId[0] = EnetTsn_gptpDmaCh;
        update_cfg->nTxPkts = EnetApp_pktPerTxCh[EnetTsn_gptpDmaCh];
        update_cfg->nRxPkts[0] = EnetApp_pktPerRxCh[EnetTsn_gptpDmaCh];
        update_cfg->pktSize = ENET_MEM_LARGE_POOL_PKT_SIZE;
    }
    return 0;
}

static int EnetTsn_startTsn(void)
{
    int i;
    int res = 0;

    for (i = 0; i < ENETAPP_MAX_TASK_IDX; i++)
    {
        res = EnetTsn_startTsnModule(i);
        if (res != 0)
        {
            break;
        }
    }
    return res;
}

static int EnetTsn_startTsnModule(int moduleIdx)
{
    int res = 0;

    if ((moduleIdx >= 0) && (moduleIdx < ENETAPP_MAX_TASK_IDX))
    {
        EnetTsn_ModuleCtx *mod;
        mod = &EnetTsn_modCtxTable[moduleIdx];

        if ((mod->enable == true) && (mod->stopFlag == true))
        {
            res = EnetTsn_startTask(mod, moduleIdx);
        }
    }
    else
    {
        res = -1;
    }
    return res;
}

static int EnetTsn_startTask(EnetTsn_ModuleCtx *modCtx, int moduleIdx)
{
    cb_tsn_thread_attr_t attr;
    int res = 0;

    modCtx->stopFlag = false;
    cb_tsn_thread_attr_init(&attr, modCtx->taskPriority, modCtx->stackSize,
            modCtx->taskName);
    cb_tsn_thread_attr_set_stackaddr(&attr, modCtx->stackBuffer);
    if (CB_THREAD_CREATE(&modCtx->hTaskHandle,
            &attr, modCtx->onModuleRunner, modCtx) < 0)
    {
        modCtx->stopFlag = true;
        EnetAppUtils_print("Failed to start %s !\r\n", modCtx->taskName);
        res = -1;
    }
    else
    {
        EnetAppUtils_print("Start: %s\r\n", modCtx->taskName);
        if (moduleIdx == ENETAPP_UNICONF_TASK_IDX)
        {
            /* initDb must be run right after UNICONF is started and
             * before starting any other tasks. */
            res = EnetTsn_initDb();
        }
    }
    return res;
}

static int EnetTsn_initDb(void)
{
    EnetTsn_ModuleCtx *mod;
    EnetTsn_DbArgs dbArgs;
    int res = 0;
    int timeout_ms = 500;
    int i;

    do
    {
        res = CB_SEM_WAIT(&EnetTsn_appCtx.ucReadySem);
        if (res != 0)
        {
            EnetAppUtils_print("Failed to wait for the uniconf \r\n");
            break;
        }

        res = uniconf_ready(EnetTsn_appCtx.dbName, UC_CALLMODE_THREAD,
                timeout_ms);
        if (res)
        {
            EnetAppUtils_print("The uniconf must be run first!\r\n");
            break;
        }
        dbArgs.dbald = uc_dbal_open(EnetTsn_appCtx.dbName, "w",
        UC_CALLMODE_THREAD);
        if (!dbArgs.dbald)
        {
            EnetAppUtils_print("Failed to open DB!\r\n");
            res = -1;
            break;
        }
        dbArgs.ydrd = yang_db_runtime_init(dbArgs.dbald, NULL);
        if (!dbArgs.ydrd)
        {
            EnetAppUtils_print("Failed to init yang db runtime\r\n");
            res = -1;
            break;
        }

        for (i = 0; i < ENETAPP_MAX_TASK_IDX; i++)
        {
            mod = &EnetTsn_modCtxTable[i];
            if ((mod->enable == true) && (mod->onModuleDBInit != NULL))
            {
                mod->onModuleDBInit(mod, &dbArgs);
            }
        }

    } while (0);
    if (dbArgs.ydrd)
    {
        yang_db_runtime_close(dbArgs.ydrd);
    }
    if (dbArgs.dbald)
    {
        uc_dbal_close(dbArgs.dbald, UC_CALLMODE_THREAD);
    }

    return res;
}

static void EnetTsn_stopTsnModule(int moduleIdx)
{
    if ((moduleIdx >= 0) && (moduleIdx < ENETAPP_MAX_TASK_IDX))
    {
        EnetTsn_ModuleCtx *mod;
        mod = &EnetTsn_modCtxTable[moduleIdx];
        if (mod->hTaskHandle != NULL)
        {
            mod->stopFlag = true;
            CB_THREAD_JOIN(mod->hTaskHandle, NULL);
            mod->hTaskHandle = NULL;
            EnetAppUtils_print("Task: %s is terminated.\r\n", mod->taskName);
        }
    }
}

/* Functions required for tsn library */
static bool EnetTsn_IsMacAddrSet(uint8_t *mac)
{
    return ((mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5]) != 0);
}

static int EnetTsn_AddVlan(Enet_Handle hEnet, uint32_t coreId, uint32_t vlanId)
{
    CpswAle_VlanEntryInfo inArgs;
    uint32_t outArgs;
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    inArgs.vlanIdInfo.vlanId = vlanId;
    inArgs.vlanIdInfo.tagType = ENET_VLAN_TAG_TYPE_INNER;
    inArgs.vlanMemberList = CPSW_ALE_ALL_PORTS_MASK;
    inArgs.unregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    inArgs.regMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    inArgs.forceUntaggedEgressMask = 0U;
    inArgs.noLearnMask = 0U;
    inArgs.vidIngressCheck = false;
    inArgs.limitIPNxtHdr = false;
    inArgs.disallowIPFrag = false;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
    ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_VLAN, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s():CPSW_ALE_IOCTL_ADD_VLAN failed: %d\r\n",
                __func__, status);
    }
    else
    {
        EnetAppUtils_print("CPSW_ALE_IOCTL_ADD_VLAN: %d\r\n", vlanId);
    }

    return status;
}

int32_t EnetApp_applyClassifier(Enet_Handle hEnet, uint32_t coreId,
        uint8_t *dstMacAddr, uint32_t vlanId, uint32_t ethType,
        uint32_t rxFlowIdx)
{
    Enet_IoctlPrms prms;
    CpswAle_SetPolicerEntryOutArgs setPolicerEntryOutArgs;
    CpswAle_SetPolicerEntryInArgs setPolicerEntryInArgs;
    int32_t status;

    if (EnetTsn_IsMacAddrSet(dstMacAddr) == true)
    {
        status = EnetAppUtils_addAllPortMcastMembership(hEnet, dstMacAddr);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print(
                    "EnetAppUtils_addAllPortMcastMembership failed: %d\r\n",
                    status);
        }
    }
    memset(&setPolicerEntryInArgs, 0, sizeof(setPolicerEntryInArgs));

    if (ethType > 0)
    {
        setPolicerEntryInArgs.policerMatch.policerMatchEnMask |=
        CPSW_ALE_POLICER_MATCH_ETHERTYPE;
        setPolicerEntryInArgs.policerMatch.etherType = ethType;
    }
    setPolicerEntryInArgs.policerMatch.portIsTrunk = false;
    setPolicerEntryInArgs.threadIdEn = true;
    setPolicerEntryInArgs.threadId = rxFlowIdx;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setPolicerEntryInArgs,
            &setPolicerEntryOutArgs);
    ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_SET_POLICER, &prms, status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s():CPSW_ALE_IOCTL_ADD_VLAN failed: %d\r\n",
                __func__, status);
    }
    else
    {
        if (vlanId > 0)
        {
            status = EnetTsn_AddVlan(hEnet, coreId, vlanId);
        }
    }
    return status;
}

int32_t EnetApp_filterPriorityPacketsCfg(Enet_Handle hEnet, uint32_t coreId)
{
    EnetMacPort_SetPriorityRegenMapInArgs params;
    Enet_IoctlPrms prms;
    int32_t retVal = ENET_SOK;

    params.macPort = ENET_MAC_PORT_1;

    params.priorityRegenMap.priorityMap[0] = 0U;
    for (int i = 1; i < 8U; i++)
    {
        params.priorityRegenMap.priorityMap[i] = 1U; // Map all priorities from (1 to 7) to priority 1, these packets will be received on DMA channel 1.
    }

    ENET_IOCTL_SET_IN_ARGS(&prms, &params);

    ENET_IOCTL(hEnet, coreId, ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP, &prms,
            retVal);

    return retVal;
}

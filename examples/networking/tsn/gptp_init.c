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
#define GPTP_TASK_PRIORITY      (2)
#define GPTP_TASK_NAME          "gptp2d_task"

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef struct
{
    char *devlist;
    const char **confFiles;
    int domainNum;
    int domains[GPTP_MAX_DOMAINS];
    int instNum;
    int numConf;
} EnetApp_GptpOpt_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_cfgGptpPortDs(yang_db_runtime_dataq_t *ydrd, int instance,
                    int domain, int port_index);
static void EnetApp_cfgGptpDefaultDs(yang_db_runtime_dataq_t *ydrd, int instance,
                    int domain);
static int EnetApp_gptpYangConfig(yang_db_runtime_dataq_t *ydrd, int instance,
                    int domain, const char *netdevs[], int numOfPorts);
static int EnetApp_gptpNonYangConfig(uint8_t instance);

static void *EnetApp_gptpTask(void *arg);
static int EnetApp_gptpDbInit(EnetApp_ModuleCtx_t* modCtx,
                              yang_db_runtime_dataq_t *ydrd);

/* ========================================================================== */
/*                            Local Variables                                */
/* ========================================================================== */

/* gptp can support more than 2 domains but in this example we supports only 2 */
static EnetApp_GptpOpt_t gGptpOpt =
{
    .confFiles = NULL,
    .domainNum = GPTP_MAX_DOMAINS,
#if GPTP_MAX_DOMAINS == 1
    .domains = {0},
#elif GPTP_MAX_DOMAINS == 2
    .domains = {0, 1},
#else
    #error "Only support 2 domains"
#endif
    .instNum = 0,
    .numConf = 0,
};

static EnetApp_DbNameVal_t gGptpPortDs[] =
{
    {"port-enable", "true"},
    {"log-announce-interval", "0"},
    {"gptp-cap-receipt-timeout", "3"},
    {"announce-receipt-timeout", "3"},
    {"log-sync-interval", "-3"},
    {"minor-version-number", "1"},
    {"initial-log-announce-interval", "0"},
    {"sync-receipt-timeout", "3"},
    {"initial-log-sync-interval", "-3"},
    {"current-log-sync-interval", "-3"},
    {"current-log-gptp-cap-interval", "3"},
    {"initial-log-pdelay-req-interval", "0"},
    {"current-log-pdelay-req-interval", "0"},
    {"allowed-lost-responses", "9"},
    {"allowed-faults", "9"},
    {"mean-link-delay-thresh", "0x27100000"}
};

static EnetApp_DbNameVal_t gGptpDefaultDs[] =
{
    {"priority1", "248"},
    {"priority2", "248"},
    {"external-port-config-enable", "false"},
    {"time-source", "internal-oscillator"},
    {"ptp-timescale", "true"},
    {"clock-quality/clock-class", "cc-default"},
    {"clock-quality/clock-accuracy", "ca-time-accurate-to-250-ns"},
    {"clock-quality/offset-scaled-log-variance", "0x436a"}
};

static EnetApp_DbIntVal_t gGptpNonYangDs[] =
{
    {"SINGLE_CLOCK_MODE", XL4_EXTMOD_XL4GPTP_SINGLE_CLOCK_MODE, 1},
    {"USE_HW_PHASE_ADJUSTMENT", XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT, 1},
    {"CLOCK_COMPUTE_INTERVAL_MSEC", XL4_EXTMOD_XL4GPTP_CLOCK_COMPUTE_INTERVAL_MSEC, 100},
    {"FREQ_OFFSET_IIR_ALPHA_START_VALUE", XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_START_VALUE, 1},
    {"FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE", XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE, 4},
    {"PHASE_OFFSET_IIR_ALPHA_START_VALUE", XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_START_VALUE, 1},
    {"PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE", XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE, 4},
    {"MAX_DOMAIN_NUMBER", XL4_EXTMOD_XL4GPTP_MAX_DOMAIN_NUMBER, GPTP_MAX_DOMAINS},
#if GPTP_MAX_DOMAINS == 2
    {"CMLDS_MODE", XL4_EXTMOD_XL4GPTP_CMLDS_MODE, 1},
    {"SECOND_DOMAIN_THIS_CLOCK", XL4_EXTMOD_XL4GPTP_SECOND_DOMAIN_THIS_CLOCK, 1}
#endif
};

static uint8_t gGptpStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetApp_Ctx_t gAppCtx =
{
    .dbName = NULL,
};

EnetApp_ModuleCtx_t gModCtxTable[ENETAPP_MAX_TASK_IDX] =
{
    [ENETAPP_GPTP_TASK_IDX] =
    {
        .enable = true,
        .stopFlag = true,
        .taskPriority = GPTP_TASK_PRIORITY,
        .taskName = GPTP_TASK_NAME,
        .stackBuffer = gGptpStackBuf,
        .stackSize = sizeof(gGptpStackBuf),
        .onModuleDBInit = EnetApp_gptpDbInit,
        .onModuleRunner = EnetApp_gptpTask,
        .appCtx = &gAppCtx
    },
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static void EnetApp_cfgGptpPortDs(yang_db_runtime_dataq_t *ydrd, int instance,
                             int domain, int port_index)
{
    int i;
    char buffer[MAX_KEY_SIZE];

    for (i = 0; i < sizeof(gGptpPortDs)/sizeof(gGptpPortDs[0]); i++)
    {
        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/%s",
                 instance, domain, port_index, gGptpPortDs[i].name);

        yang_db_runtime_put_oneline(ydrd, buffer, gGptpPortDs[i].val,
                                    YANG_DB_ONHW_NOACTION);
    }
}

static void EnetApp_cfgGptpDefaultDs(yang_db_runtime_dataq_t *ydrd, int instance,
                                   int domain)
{
    int i;
    char buffer[MAX_KEY_SIZE];

    for (i = 0; i < sizeof(gGptpDefaultDs)/sizeof(gGptpDefaultDs[0]); i++)
    {
        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/default-ds/%s",
                 instance, domain, gGptpDefaultDs[i].name);
        yang_db_runtime_put_oneline(ydrd, buffer, gGptpDefaultDs[i].val,
                                    YANG_DB_ONHW_NOACTION);
    }
}

static int EnetApp_gptpYangConfig(yang_db_runtime_dataq_t *ydrd, int instance,
               int domain, const char *netdevs[], int numOfPorts)
{
    char buffer[MAX_KEY_SIZE];
    char value_str[32];
    char *plus;
    int i, res = 0;

    DPRINT("%s:domain=%d", __func__, domain);

    do {
        plus = ((instance | domain) != 0) ? "+": "";
        snprintf(buffer, sizeof(buffer), "/ieee1588-ptp/ptp/instance-domain-map%s",
                 plus);
        snprintf(value_str, sizeof(value_str), "0x%04x", instance<<8|domain);
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        /* set for default-ds */
        EnetApp_cfgGptpDefaultDs(ydrd, instance, domain);

        // portindex starts from 1
        for (i = 0; i < numOfPorts; i++)
        {
            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/underlying-interface",
                     instance, domain, i+1);
            yang_db_runtime_put_oneline(ydrd, buffer, (char*)netdevs[i],
                                        YANG_DB_ONHW_NOACTION);

            /* set for port-ds */
            EnetApp_cfgGptpPortDs(ydrd, instance, domain, i+1);
        }
        /* disable performance by default */
        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/performance-monitoring-ds/enable",
                 instance, domain);
        yang_db_runtime_put_oneline(ydrd, buffer, "false", YANG_DB_ONHW_NOACTION);
    } while (0);

    return res;
}

static int EnetApp_gptpNonYangConfig(uint8_t instance)
{
    int i;
    int res;

    for (i = 0; i < sizeof(gGptpNonYangDs)/sizeof(gGptpNonYangDs[0]); i++)
    {
        res = gptpgcfg_set_item(instance, gGptpNonYangDs[i].item,  YDBI_CONFIG,
                    &gGptpNonYangDs[i].val, sizeof(gGptpNonYangDs[i].val));
        if (res == 0)
        {
            DPRINT("%s:XL4_EXTMOD_XL4GPTP_%s=%d", __func__,
                   gGptpNonYangDs[i].name, gGptpNonYangDs[i].val);
        }
        else
        {
            DPRINT("%s: failed to set nonyange param: %s",
                   __func__, gGptpNonYangDs[i].name);
            break;
        }
    }
    return res;
}

static void *EnetApp_gptpTask(void *arg)
{
    int i;
    int res = 0;
    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)arg;
    EnetApp_Ctx_t *appCtx = modCtx->appCtx;
    const char *netdevs[CB_MAX_NETDEVNAME];

    for (i = 0; i < appCtx->netdevSize; i++)
    {
        netdevs[i] = appCtx->netdev[i];
    }

    /* This function has a true loop inside */
    res = gptpman_run(gGptpOpt.instNum, netdevs, appCtx->netdevSize,
                     NULL, &modCtx->stopFlag);
    if (res)
    {
        DPRINT("%s: gptpman_run() error", __func__);
    }

    return NULL;
}

static int EnetApp_gptpDbInit(EnetApp_ModuleCtx_t* modCtx, yang_db_runtime_dataq_t *ydrd)
{
    EnetApp_Ctx_t *appCtx = modCtx->appCtx;
    const char *netdevs[CB_MAX_NETDEVNAME];
    int (*gptpNonYangConfig)(uint8_t instance);
    int res = 0;
    int i;

    for (i = 0; i < appCtx->netdevSize; i++)
    {
        netdevs[i] = appCtx->netdev[i];
    }

    if (gGptpOpt.numConf == 0)
    {
        /* There is no config file is specified, set config file for gptp*/
        for (i = 0; i < gGptpOpt.domainNum; i++)
        {
            res = EnetApp_gptpYangConfig(ydrd, gGptpOpt.instNum, gGptpOpt.domains[i],
                        netdevs, appCtx->netdevSize);
            if (res)
            {
                DPRINT("Failed to set gptp run time config");
            }
        }
        gptpNonYangConfig = EnetApp_gptpNonYangConfig;
    }
    else
    {
        gptpNonYangConfig = NULL;
    }

    if (res == 0)
    {
        res = gptpgcfg_init(appCtx->dbName, gGptpOpt.confFiles, gGptpOpt.instNum, true,
                            gptpNonYangConfig);
        if (res != 0)
        {
            DPRINT("%s: gptpgcfg_init() error", __func__);
        }
    }

    return res;
}

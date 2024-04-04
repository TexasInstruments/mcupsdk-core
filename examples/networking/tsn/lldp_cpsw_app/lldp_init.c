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
#include <tsn_uniconf/yangs/ieee1588-ptp-tt_access.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include "debug_log.h"
#include "tsninit.h"
#include "common.h"
#include <tsn_lldp/lldpd.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define LLDP_TASK_PRIORITY      (2)
#define LLDP_TASK_NAME          "lldpd_task"
#define LLDP_NODE               "/ieee802-dot1ab-lldp/lldp/"
#define LLDP_LOCAL_SYSTEM_DATA  "local-system-data/"
#define LLDP_DEST_MAC_ADDR      "01-80-c2-00-00-0e" // nearest bridge

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    const char **confFiles; /*TODO: Should lldp need to read config and write to DB?*/
    int numConf; /*TODO: Should lldp need to read config and write to DB?*/
    int vlanId;
} EnetApp_LldpOpt_t;

typedef struct
{
    char* ipvx;
    char* addr;
    char* key;
    char* val;
} EnetApp_LldpMgmtAddrKv_t;

typedef struct
{
    char* destMac;
    EnetApp_DbNameVal_t cfgKeyVal[11];
} EnetApp_LldpPortCfg_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int EnetApp_setLldpRtConfig(yang_db_runtime_dataq_t *ydrd, EnetApp_Ctx_t * ctx);
static void *EnetApp_lldpTask(void* arg);
static int EnetApp_lldpDbInit(EnetApp_ModuleCtx_t* modCtx, EnetApp_dbArgs *dbargs);

/* ========================================================================== */
/*                            Local Variables                                */
/* ========================================================================== */

static EnetApp_LldpOpt_t gLldpopt =
{
    .confFiles = NULL,
    .numConf = 0,
    .vlanId = -1
};

static EnetApp_DbNameVal_t gLldpGlobalData[] =
{
    {"message-fast-tx" , "1"},
    {"message-tx-hold-multiplier" , "4"},
    {"message-tx-interval" , "30"},
    {"reinit-delay" , "2"},
    {"tx-credit-max" , "5"},
    {"tx-fast-init" , "2"},
};

static EnetApp_DbNameVal_t gLldpLocalSysData[] =
{
    // local system data
    {"chassis-id-subtype" , "7"}, // '7' mean local filled, in case of more than one ports, subtype='4' MAC address cannot apply.
    {"chassis-id" , "00-01-02-03-04-05"}, // Just a simple string in case of subtype is 'local (7)'
    {"system-name" , "tilld"},
    {"system-description" , "tilld"},
    {"system-capabilities-supported" , "11111111111"},
    {"system-capabilities-enabled" , "11110111011"},
};

static EnetApp_LldpPortCfg_t gLldpPortCfgData[] =
{
    {
        .destMac = "01-80-c2-00-00-0e",
        .cfgKeyVal =
        {
            {"admin-status", "3"},
            {"tlvs-tx-enable", "1111"},
            {"port-id-subtype", "3"},         // If PortId subtype = P_MAC_Address (3), MAC addr will be re-correct follow hw info.
            {"port-id", "3c-e0-64-62-e3-03"}, // <- this value will be correct in runtime
            {"port-desc", "tilld"},
            {"message-fast-tx", "2"},          // Diff with global system config
            {"message-tx-hold-multiplier" , "4"},
            {"message-tx-interval" , "30"},
            {"reinit-delay" , "2"},
            {"tx-credit-max" , "5"},
            {"tx-fast-init" , "2"},
        }
    },
    {
        .destMac = "01-80-c2-00-00-03",
        .cfgKeyVal =
        {
            {"admin-status", "3"},
            {"tlvs-tx-enable", "1111"},
            {"port-id-subtype", "3"},         // If PortId subtype = P_MAC_Address (3), MAC addr will be re-correct follow hw info.
            {"port-id", "3c-e0-64-62-e3-03"}, // <- this value will be correct in runtime
            {"port-desc", "tilld"},
            {"message-fast-tx", "2"},          // Diff with global system config
            {"message-tx-hold-multiplier" , "4"},
            {"message-tx-interval" , "20"},
            {"reinit-delay" , "2"},
            {"tx-credit-max" , "5"},
            {"tx-fast-init" , "2"},
        }
    },
    {
        .destMac = "01-80-c2-00-00-00",
        .cfgKeyVal =
        {
            {"admin-status", "3"},
            {"tlvs-tx-enable", "1111"},
            {"port-id-subtype", "3"},         // If PortId subtype = P_MAC_Address (3), MAC addr will be re-correct follow hw info.
            {"port-id", "3c-e0-64-62-e3-03"}, // <- this value will be correct in runtime
            {"port-desc", "tilld"},
            {"message-fast-tx", "2"},          // Diff with global system config
            {"message-tx-hold-multiplier" , "4"},
            {"message-tx-interval" , "25"},
            {"reinit-delay" , "2"},
            {"tx-credit-max" , "5"},
            {"tx-fast-init" , "2"},
        }
    },
};

static uint8_t gLldpStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern EnetApp_Ctx_t gAppCtx;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int EnetApp_addLldpModCtx(EnetApp_ModuleCtx_t *modCtxTbl)
{
    EnetApp_ModuleCtx_t lldpModCtx = {
        .enable = true,
        .stopFlag = true,
        .taskPriority = LLDP_TASK_PRIORITY,
        .taskName = LLDP_TASK_NAME,
        .stackBuffer = gLldpStackBuf,
        .stackSize = sizeof(gLldpStackBuf),
        .onModuleDBInit = EnetApp_lldpDbInit,
        .onModuleRunner = EnetApp_lldpTask,
        .appCtx = &gAppCtx
    };
    memcpy(&modCtxTbl[ENETAPP_LLDP_TASK_IDX], &lldpModCtx,
           sizeof(EnetApp_ModuleCtx_t));
    return 0;
}

static int EnetApp_setLldpRtConfig(yang_db_runtime_dataq_t *ydrd, EnetApp_Ctx_t *appCtx)
{
    int i, j;
    int ndev;
    char buffer[MAX_KEY_SIZE];

    for (i = 0; i < sizeof(gLldpGlobalData)/sizeof(gLldpGlobalData[0]); i++)
    {
        snprintf(buffer, sizeof(buffer),
                 "%s%s",
                 LLDP_NODE, gLldpGlobalData[i].name);

        // DPRINT("[%s] %s:%s", __func__, buffer, gLldpGlobalData[i].val);
        yang_db_runtime_put_oneline(ydrd, buffer, gLldpGlobalData[i].val,
                                    YANG_DB_ONHW_NOACTION);
    }
    for (i = 0; i < sizeof(gLldpLocalSysData)/sizeof(gLldpLocalSysData[0]); i++)
    {
        snprintf(buffer, sizeof(buffer),
                 "%s%s%s",
                 LLDP_NODE, LLDP_LOCAL_SYSTEM_DATA, gLldpLocalSysData[i].name);

        // DPRINT("[%s] %s:%s", __func__, buffer, gLldpLocalSysData[i].val);
        yang_db_runtime_put_oneline(ydrd, buffer, gLldpLocalSysData[i].val,
                                    YANG_DB_ONHW_NOACTION);
    }

    DPRINT("[%s] Initialized LLDP Local System data", __func__);

    for (ndev = 0; ndev < appCtx->netdevSize; ndev++)
    {
        for (i = 0; i < sizeof(gLldpPortCfgData)/sizeof(gLldpPortCfgData[0]); i++)
        {
            for (j = 0; j < sizeof(gLldpPortCfgData[i].cfgKeyVal)/ sizeof(EnetApp_DbNameVal_t); j++ )
            {
                snprintf(buffer, sizeof(buffer),
                        "%sport|name:%s|dest-mac-address:%s|/%s",
                        LLDP_NODE, appCtx->netdev[ndev], gLldpPortCfgData[i].destMac, gLldpPortCfgData[i].cfgKeyVal[j].name);

                // DPRINT("[%s] %s:%s", __func__, buffer, gLldpPerPortData[i].val);
                yang_db_runtime_put_oneline(ydrd, buffer, gLldpPortCfgData[i].cfgKeyVal[j].val,
                                            YANG_DB_ONHW_NOACTION);
            }
        }

        DPRINT("[%s] Initialized LLDP Port tilld%d", __func__, ndev);
    }

    return 0;
}

static int EnetApp_lldpDbInit(EnetApp_ModuleCtx_t* modCtx, EnetApp_dbArgs *dbargs)
{
    EnetApp_Ctx_t *appCtx = modCtx->appCtx;

    lldpd_uniconf_access_mode(UC_CALLMODE_THREAD);

    EnetApp_setLldpRtConfig(dbargs->ydrd, appCtx);

    return 0;
}

static void *EnetApp_lldpTask(void* arg)
{
    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)arg;
    EnetApp_Ctx_t *appCtx = modCtx->appCtx;

    if (lldpd_init(appCtx->dbName, &gLldpopt.vlanId, appCtx->netdev, appCtx->netdevSize) == 0)
    {
        uint8_t *terminated = (uint8_t*)&modCtx->stopFlag;
        lldpd_run(terminated); // Blocking task

        lldpd_deinit();
    }
    else
    {
        DPRINT("%s: lldp initialized failure", __func__);
    }

    return NULL;
}

/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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

#include <stdint.h>
#include <stdbool.h>
#include <tsn_unibase/unibase.h>
#include <tsn_combase/combase.h>
#include <tsn_combase/cb_tmevent.h>
#include <tsn_gptp/tilld/lld_gptp_private.h>
#include <tsn_unibase/unibase_binding.h>
#include <tsn_uniconf/yangs/yang_db_runtime.h>
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>

#ifdef GPTP_ENABLED
#include <tsn_gptp/gptpmasterclock.h>
#endif

#include "debug_log.h"
#include "tsninit.h"
#include "common.h"
#include "qosapp_misc.h"

typedef struct TimeSlot
{
    uint64_t start; /*! Expected start time for receiving packet */
    uint64_t end;   /*! Expected end time for receiving packet */
} TimeSlot_t;

typedef struct PerPriorityTimeSlot
{
    int32_t nLength; /*! Num of timeslots for each priority */
    TimeSlot_t timeSlots[ENET_TAS_MAX_CMD_LISTS]; /*! Timeslot for each priority */
} PerPriorityTimeSlot_t;

typedef struct EstStatsInfo
{
    uint64_t nGoodPkt;         /*! Num of packets received inside timeslot */
    uint64_t nBadPkt;          /*! Num of packets received outside timeslot */
} EstStatsInfo_t;

typedef struct EnetEstAppCtx
{
    EnetQoSApp_AppCtx_t appCtx;/*! Common context param is general for all QoS applications. */
    int schedIdx;              /*! Index of EST schedule applied for talker and listener. */
    EstStatsInfo_t estStatsInfo[QOSAPP_PRIORITY_MAX];
    /*! Expected timeslot for all priority traffic */
    PerPriorityTimeSlot_t exptTimeSlots[QOSAPP_PRIORITY_MAX];
} EnetEstAppCtx_t;

typedef struct EnetEstAppTestParam
{
    EnetTas_ControlList list;          /*! List of Admin param for EST */
    QoSAppStreamConfigParam_t stParam; /*! Streams parameters */
} EnetEstAppTestParam_t;

UB_SD_GETMEM_DEF_EXTERN(YANGINIT_GEN_SMEM);

/*============================================================================*/
/*                          Macros and Constants                              */
/*============================================================================*/

#define ESTAPP_TASK_PRIORITY   (1)
#define ESTAPP_TASK_NAME       "estapp_task"
#define ESTAPP_TALKER_NAME     "estapp_talker"
#define ESTAPP_LISTENER_NAME   "estapp_listener"

/* 18: length of layer 2 header */
#define CALC_BITRATE_KBPS(pl_bytes, interval_us)  \
    (uint32_t)( (((uint64_t)(pl_bytes)+18)*8*UB_SEC_US)/ ((interval_us)*1000ULL) )

#define ENETEST_TX_PKT_PAYLOAD_LEN (1200U)
#define EST_INTERVAL_NS            (62000U)
#define ADMIN_DELAY_OFFSET_FACTOR  (100000)

/*! Base path of admin list parameters in yang file of Qbv */
#define GATE_PARAM_TABLE_NODE "/ietf-interfaces/interfaces/interface|name:%s|" \
    "/bridge-port/gate-parameter-table"
#define GATE_CONTROL_ENTRY_NODE GATE_PARAM_TABLE_NODE   \
    "/admin-control-list/gate-control-entry"

/*! Base path of clock-state node in  yang file for checking PTP synchronized */
#define IEEE1588_PTP_TT_CLOCKSTATE_NODE  "/ieee1588-ptp-tt/ptp/instances" \
    "/instance|instance-index:0,0|/clock-state"

/*! Base path of port-state node in  yang file for checking PTP synchronized */
#define IEE1588_PTP_PORT_STATE_NODE  "/ieee1588-ptp-tt/ptp/instances" \
    "/instance|instance-index:0,0|/ports/port|port-index:%d|/port-ds"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void *EnetApp_estAppTask(void *arg);
static int EnetApp_estAppDbInit(EnetApp_ModuleCtx_t* mdctx,
                                EnetApp_dbArgs *dbargs);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static uint8_t gEnetEstAppStackBuf[TSN_TSK_STACK_SIZE]
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

static uint8_t gEnetEstAppTalkerStackBuf[TSN_TSK_STACK_SIZE]
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#if LISTNER_VERIFY_ENABLE
static uint8_t gEnetEstAppListenerStackBuf[TSN_TSK_STACK_SIZE]
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));
#endif

extern EnetApp_ModuleCtx_t gModCtxTable[ENETAPP_MAX_TASK_IDX];
extern EnetApp_Ctx_t gAppCtx;
static EnetEstAppCtx_t gEnetEstAppCtx;

static EnetEstAppTestParam_t gEnetEstAppTestLists[] =
{
    {
        .list =
        {
            .baseTime    = 0ULL,
            .cycleTime   = 4*EST_INTERVAL_NS,
            .gateCmdList =
            {
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 0, 0, 1),
                  .timeInterval =  EST_INTERVAL_NS
                },
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 1, 0, 0),
                  .timeInterval =  EST_INTERVAL_NS
                },
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 0, 0, 1),
                  .timeInterval =  EST_INTERVAL_NS
                },
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 1, 0, 0),
                  .timeInterval =  EST_INTERVAL_NS
                },
            },
            .listLength = 4U,
        },
        .stParam =
        {
            .streamParams =
            {
                /* test appliction sends packet with interval 800us */
                {.bitRateKbps = CALC_BITRATE_KBPS(ENETEST_TX_PKT_PAYLOAD_LEN, 800),
                 .payloadLen = ENETEST_TX_PKT_PAYLOAD_LEN,
                 .tc = 0,
                 .priority = 0,
                },

                /* test appliction sends packet with interval 400us */
                {.bitRateKbps = CALC_BITRATE_KBPS(ENETEST_TX_PKT_PAYLOAD_LEN, 400),
                 .payloadLen = ENETEST_TX_PKT_PAYLOAD_LEN,
                 .tc = 2,
                 .priority = 2,
                },
            },
            .nStreams = 2,
        }
    },
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int EnetApp_addEstAppModCtx(EnetApp_ModuleCtx_t *modCtxTbl)
{
    EnetApp_ModuleCtx_t estAppModCtx =
    {
        .enable = true,
        .stopFlag = true,
        .taskPriority = ESTAPP_TASK_PRIORITY,
        .taskName = ESTAPP_TASK_NAME,
        .stackBuffer = gEnetEstAppStackBuf,
        .stackSize = sizeof(gEnetEstAppStackBuf),
        .onModuleDBInit = EnetApp_estAppDbInit,
        .onModuleRunner = EnetApp_estAppTask,
        .appCtx = &gAppCtx
    };
    memcpy(&modCtxTbl[ENETAPP_EST_TASK_IDX], &estAppModCtx,
           sizeof(EnetApp_ModuleCtx_t));
    return 0;
}

static void EnetEstApp_printAdminControlList(EnetTas_ControlList *list)
{
    uint8_t gateMask = 0U;
    uint32_t start = 0U;
    uint32_t end;
    uint32_t dur;
    uint32_t i;

    DPRINT("The following AdminList param will be configured for EST:");
    for (i = 0U; i < list->listLength; i++)
    {
        gateMask = list->gateCmdList[i].gateStateMask;
        dur = list->gateCmdList[i].timeInterval;
        end = start + dur - 1U;
        /* o = Gate open, C = Gate closed */
        DPRINT("GateMask[7..0]=%s%s%s%s%s%s%s%s (0x%02x), start=%u ns, end=%u ns, dur=%u ns",
               ENET_IS_BIT_SET(gateMask, 7U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 6U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 5U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 4U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 3U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 2U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 1U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 0U) ? "o" : "C",
               gateMask, start, end, dur);
        start += dur;
    }
    char buffer[MAX_LOG_LEN];
    snprintf(buffer, sizeof(buffer), "Base time=%lluns,Cycle time=%lluns",
             list->baseTime,list->cycleTime);
    DPRINT("%s", buffer);
}

static int EnetEstApp_setAdminControlList(EnetTas_ControlList *list, char *ifname,
                                          yang_db_runtime_dataq_t *ydrd,
                                          uc_notice_data_t *ucntd)
{
    int i, err = 0;
    char buffer[MAX_KEY_SIZE];
    char val[MAX_VAL_SIZE];

    EnetEstApp_printAdminControlList(list);

    if (list->cycleTime > 0)
    {
        /* Expected unit is mircosecond. */
        uint32_t cycletime_numerator = list->cycleTime/1000U;
        uint32_t cycletime_denominator = 1000000UL;
        snprintf(buffer, sizeof(buffer),
                 GATE_PARAM_TABLE_NODE"/admin-cycle-time/numerator",
                 ifname);
        snprintf(val, sizeof(val), "%d", cycletime_numerator);
        YANGDB_RUNTIME_WRITE(buffer, val);

        snprintf(buffer, sizeof(buffer),
                 GATE_PARAM_TABLE_NODE"/admin-cycle-time/denominator",
                 ifname);
        snprintf(val, sizeof(val), "%d", cycletime_denominator);
        YANGDB_RUNTIME_WRITE(buffer, val);
    }

    uint32_t second = list->baseTime/1000000000ULL;
    uint32_t nanosecond = list->baseTime%1000000000ULL;
    snprintf(buffer, sizeof(buffer),
             GATE_PARAM_TABLE_NODE"/admin-base-time/seconds",
             ifname);
    snprintf(val, sizeof(val), "%d", second);
    YANGDB_RUNTIME_WRITE(buffer, val);

    snprintf(buffer, sizeof(buffer),
             GATE_PARAM_TABLE_NODE"/admin-base-time/nanoseconds",
             ifname);
    snprintf(val, sizeof(val), "%d", nanosecond);
    YANGDB_RUNTIME_WRITE(buffer, val);

    for (i = 0; i < list->listLength; i++)
    {
        snprintf(buffer, sizeof(buffer),
                 GATE_CONTROL_ENTRY_NODE"|index:%d|/operation-name",
                 ifname, i);
        strcpy(val, "set-gate-states");
        YANGDB_RUNTIME_WRITE(buffer, val);

        snprintf(buffer, sizeof(buffer),
                 GATE_CONTROL_ENTRY_NODE"|index:%d|/time-interval-value",
                 ifname, i);
        snprintf(val, sizeof(val), "%d",
                 list->gateCmdList[i].timeInterval);
        YANGDB_RUNTIME_WRITE(buffer, val);

        snprintf(buffer, sizeof(buffer),
                 GATE_CONTROL_ENTRY_NODE"|index:%d|/gate-states-value",
                 ifname, i);
        snprintf(val, sizeof(val), "%d",
                 list->gateCmdList[i].gateStateMask);
        YANGDB_RUNTIME_WRITE(buffer, val);
    }

    snprintf(buffer, sizeof(buffer), GATE_PARAM_TABLE_NODE"/gate-enabled", ifname);
    strcpy(val, "true");
    YANGDB_RUNTIME_WRITE(buffer, val);

    /* Trigger the uniconf to write parameters from DB to HW */
    err = yang_db_runtime_askaction(ydrd, ucntd);

    return err;
}

static int EnetApp_estAppDbInit(EnetApp_ModuleCtx_t* mdctx,
                                EnetApp_dbArgs *dbargs)
{
    /* This API does nothing */
    return 0;
}

static bool EnetEstApp_isPTPClockStateSync(EnetQoSApp_AppCtx_t *ctx,
                                           char *netdev)
{
    int err = -1;
    bool syncFlag = false;
    EnetApp_dbArgs dbarg;
    EnetApp_Ctx_t *ectx = ctx->ectx;

    err = EnetQoSApp_openDB(&dbarg, ectx->dbName, "w");
    if (err)
    {
        DPRINT("Failed to open DB!");
    }
    else
    {
        do
        {
            char buffer[MAX_KEY_SIZE];
            void *val = NULL;
            uint32_t vsize;
            uint8_t portState = 0;
            int8_t portIdx = EnetQoSApp_getPortIdx(ctx, netdev);
            DebugP_assert(portIdx >= 0 && portIdx < ctx->netdevSize);

            snprintf(buffer, sizeof(buffer),
                     IEEE1588_PTP_TT_CLOCKSTATE_NODE"/gmstate");
            err = yang_db_runtime_get_oneline(dbarg.ydrd, buffer, &val, &vsize);
            if (err == -1)
            {
                DPRINT("Failed to read %s from the DB!", buffer);
                break;
            }

            syncFlag = *(uint8_t *)val == 2? true: false;
            UB_SD_RELMEM(YANGINIT_GEN_SMEM, val);
            val  = NULL;

            if (!syncFlag)
            {
                break;
            }
            syncFlag = false;
            /* gPTP port index in the DB started from 1 */
            snprintf(buffer, sizeof(buffer),
                     IEE1588_PTP_PORT_STATE_NODE"/port-state", portIdx+1);
            err = yang_db_runtime_get_oneline(dbarg.ydrd, buffer, &val, &vsize);
            if (err == -1)
            {
                DPRINT("Failed to read %s ", buffer);
                break;
            }
            portState =  *(uint8_t *)val;
            UB_SD_RELMEM(YANGINIT_GEN_SMEM, val);
            val  = NULL;

            /* check ieee1588-ptp-tt.yang for description of portState */
            if (portState != 6 && portState != 9)
            {
                DPRINT("Current port-state: %d ", portState);
                break;
            }

            snprintf(buffer, sizeof(buffer), IEE1588_PTP_PORT_STATE_NODE"/as-capable", portIdx+1);
            err = yang_db_runtime_get_oneline(dbarg.ydrd, buffer, &val, &vsize);
            if (err == -1)
            {
                DPRINT("Failed to read %s ", buffer);
                break;
            }
            bool asCapable = *(uint8_t *)val? true: false;
            UB_SD_RELMEM(YANGINIT_GEN_SMEM, val);
            if ((portState == 6 || portState == 9) && asCapable)
            {
                syncFlag = true;
            }
            else if (portState == 9 && !asCapable)
            {
                syncFlag = true;
            }
        } while (0);

        EnetQoSApp_closeDB(&dbarg);
    }

    return syncFlag;
}

static void EnetEstApp_showMemu(void)
{
    DPRINT("\r\nCPSW EST Test Menu:\r\n");

    DPRINT(" 't'  -  Start Talker");
    DPRINT(" 'l'  -  Start Listener");
    DPRINT(" 'b'  -  Start EST Bridge mode");


    DPRINT(" Others:");
    DPRINT(" 's'  -  Print statistics");
    DPRINT(" 'S'  -  Reset statistics");
    DPRINT(" 'd'  -  Show EST stats good/bad packets");
    DPRINT(" 'h'  -  Show this menu");
}

static int EnetEstApp_getAdminBaseTime(uint64_t *time)
{
    int res = 0;
    int64_t ts = gptpmasterclock_getts64();
    if (ts < 0)
    {
        res = -1;
    }
    else
    {
        *time = ts;
    }
    return res;
}

static int EnetEstApp_runSchedule(EnetQoSApp_AppCtx_t *ctx,
                                  EnetTas_ControlList *adminList,
                                  char *netdev)
{
    bool openDBSuccess = false;
    EnetApp_dbArgs dbarg;
    EnetApp_Ctx_t *ectx = (EnetApp_Ctx_t *)ctx->ectx;
    int err, i;


    do
    {
        err = EnetQoSApp_openDB(&dbarg, ectx->dbName, "w");
        if (err)
        {
            DPRINT("Failed to open DB!");
            break;
        }
        openDBSuccess = true;

        QoSAppCommonParam_t prm =
        {
            .netdev = netdev,
            /* initialize with invalid TC value */
            {-1, -1, -1, -1, -1, -1, -1, -1},
            .nTCs = QOSAPP_PRIORITY_MAX,
            .nQueues = QOSAPP_PRIORITY_MAX,
        };
        for (i = 0; i < prm.nTCs; i++)
        {
            /* Use one-to-one mapping between TC and priority */
            prm.priority2TcMapping[i] = i;
        }

        err = EnetQoSApp_setCommonParam(&prm, &dbarg);
        if (err)
        {
            DPRINT("Failed to set EST common param!");
            break;
        }

        if (EnetEstApp_getAdminBaseTime(&adminList->baseTime) == 0)
        {
            /* Add a delay time to allow the admin list scheduled in the future 
            * the offset should be large enough to have  both EST schedules from
            * talker and listener started at the same time
            */
            int64_t offset = ADMIN_DELAY_OFFSET_FACTOR*adminList->cycleTime;
            adminList->baseTime = ((adminList->baseTime+offset)/offset)*offset;
            ctx->adminDelayOffset = offset/1000; /* Convert to microsecond */
        }
        err = EnetEstApp_setAdminControlList(adminList,
                                             netdev,
                                             dbarg.ydrd, dbarg.ucntd);
        if (err)
        {
            DPRINT("Failed to set admin control list for %s",
                   netdev);
            break;
        }
        else
        {
            DPRINT("Set admin control list succesfully");
        }
    } while (0);

    if (openDBSuccess)
    {
        EnetQoSApp_closeDB(&dbarg);
    }

    return err;
}

static void EnetEstApp_startTalker(EnetQoSApp_AppCtx_t *ctx,
                                   QoSAppStreamConfigParam_t *stParam)
{
    int i;
    EnetQoSApp_TaskCfg_t cfg =
    {
        .name = ESTAPP_TALKER_NAME,
        .priority  = QOSAPP_TASK_PRIORITY,
        .stackBuffer = gEnetEstAppTalkerStackBuf,
        .stackBufferSize = sizeof(gEnetEstAppTalkerStackBuf),
    };

    for (i = 0; i < stParam->nStreams; i++)
    {
        DPRINT("Talker: starting stream %d with, payloadLen: %d, bitrate: %d kbps, priority: %d",
               i, stParam->streamParams[i].payloadLen,
               stParam->streamParams[i].bitRateKbps,
               stParam->streamParams[i].priority);
    }
    return EnetQoSApp_startTalker(ctx, &cfg, stParam);
}

static void EnetEstApp_stopTalker(EnetQoSApp_AppCtx_t *ctx)
{
    return EnetQoSApp_stopTalker(ctx);
}

#if LISTNER_VERIFY_ENABLE
static void EnetEstApp_startListener(EnetQoSApp_AppCtx_t *ctx)
{
    EnetQoSApp_TaskCfg_t cfg =
    {
        .name = ESTAPP_LISTENER_NAME,
        .priority  = QOSAPP_TASK_PRIORITY,
        .stackBuffer = gEnetEstAppListenerStackBuf,
        .stackBufferSize = sizeof(gEnetEstAppListenerStackBuf),
    };
    return EnetQoSApp_startListener(ctx, &cfg);
}
#endif

static void EnetEstApp_stopListener(EnetQoSApp_AppCtx_t *ctx)
{
    return EnetQoSApp_stopListener(ctx);
}

static void EnetEstApp_rxPacketHandler(void *arg,
                                       EnetQoSApp_Packet_t *pkt)
{
    bool goodPktFlag = false;
    int i, priority;
    EnetEstAppCtx_t *estAppCtx = (EnetEstAppCtx_t *)arg;
    EnetTas_ControlList *adminList = &gEnetEstAppTestLists[estAppCtx->schedIdx].list;
    int64_t timeSlot = pkt->recvAddr.rxts - adminList->baseTime;

    priority = pkt->recvAddr.tcid;
    if (priority < QOSAPP_PRIORITY_MAX)
    {
        timeSlot = timeSlot%adminList->cycleTime;

        for (i = 0; i < estAppCtx->exptTimeSlots[priority].nLength; i++)
        {
            if (timeSlot >= estAppCtx->exptTimeSlots[priority].timeSlots[i].start &&
                timeSlot <= estAppCtx->exptTimeSlots[priority].timeSlots[i].end)
            {
                estAppCtx->estStatsInfo[priority].nGoodPkt++;
                goodPktFlag = true;
                break;
            }
        }
    }
    if (!goodPktFlag)
    {
        estAppCtx->estStatsInfo[priority].nBadPkt++;
    }
}

static void EnetEstApp_showEstStats(EnetQoSApp_AppCtx_t *ctx)
{
    int i;
    EnetEstAppCtx_t *estAppCtx = (EnetEstAppCtx_t *)ctx;
    char buffer[MAX_LOG_LEN];
    uint64_t goodPkts, badPkts;

    for (i = 0; i < QOSAPP_PRIORITY_MAX; i++)
    {
        goodPkts = estAppCtx->estStatsInfo[i].nGoodPkt;
        badPkts = estAppCtx->estStatsInfo[i].nBadPkt;
        if (goodPkts > 0 ||  badPkts > 0)
        {
            snprintf(buffer, sizeof(buffer),
                     "PacketPriority: %d, nGoodPackets: %llu, nBadPackets: %llu, percentage of bad packets: %llu%%",
                     i, goodPkts, badPkts, (badPkts*100)/(badPkts+goodPkts));
            DPRINT("%s", buffer);
        }
    }
}

static void EnetEstApp_runTalker(EnetQoSApp_AppCtx_t *ctx)
{
    int err;

    while (!EnetEstApp_isPTPClockStateSync(ctx, ctx->netdev[ctx->ifidx]))
    {
        DPRINT("Waiting for PTP clock to be synchronized!");
        CB_USLEEP(1000000ULL);
    }
    int schedIdx = 0;
    err = EnetEstApp_runSchedule(ctx,
                                 &gEnetEstAppTestLists[schedIdx].list,
                                 ctx->netdev[ctx->ifidx]);
    if (err == 0)
    {
        CB_USLEEP(ctx->adminDelayOffset);

        EnetEstApp_startTalker(ctx, &gEnetEstAppTestLists[schedIdx].stParam);
    }
}

#if LISTNER_VERIFY_ENABLE
static void EnetEstApp_defragmentTimeSlots(PerPriorityTimeSlot_t *prm)
{
    int i;
    TimeSlot_t timeSlots[ENET_TAS_MAX_CMD_LISTS];
    int count = 0;

    for (i = 0; i < ENET_TAS_MAX_CMD_LISTS; i++)
    {
        if (prm->timeSlots[i].end > 0)
        {
            memcpy(&timeSlots[count],
                   &prm->timeSlots[i], sizeof(TimeSlot_t));
            count++;
        }
    }
    memcpy(prm->timeSlots, timeSlots, count*sizeof(TimeSlot_t));
    prm->nLength = count;
}
#endif

#if LISTNER_VERIFY_ENABLE
static void EnetEstApp_calcExpectedTimeSlot(EnetQoSApp_AppCtx_t *ctx,
                                            EnetTas_ControlList *list,
                                            QoSAppStreamConfigParam_t *stParam)
{
    int i, j, priority, n = 0;
    EnetEstAppCtx_t *estAppCtx = (EnetEstAppCtx_t *)ctx;

    for (i = 0; i < stParam->nStreams; i++)
    {
        priority = stParam->streamParams[i].priority;
        for (j = 0; j < list->listLength; j++)
        {
            if (list->gateCmdList[j].gateStateMask&(1<<priority))
            {
                estAppCtx->exptTimeSlots[priority].timeSlots[j].start =
                    j*list->gateCmdList[j].timeInterval;
                estAppCtx->exptTimeSlots[priority].timeSlots[j].end =
                    (j+1)*list->gateCmdList[j].timeInterval;
                estAppCtx->exptTimeSlots[priority].nLength++;
            }
        }
        EnetEstApp_defragmentTimeSlots(&estAppCtx->exptTimeSlots[priority]);
    }

    // Show expected time slots of each stream for debug purpose.
    char buffer[MAX_LOG_LEN];
    for (i = 0; i < stParam->nStreams; i++)
    {
        priority = stParam->streamParams[i].priority;
        n = snprintf(buffer, sizeof(buffer), "TimeSlots of PacketPriority: %d: ", priority);
        for (j = 0; j < estAppCtx->exptTimeSlots[priority].nLength; j++)
        {
            n += snprintf(&buffer[n], sizeof(buffer)-n, "  [%llu, %llu]ns,",
                          estAppCtx->exptTimeSlots[priority].timeSlots[j].start,
                          estAppCtx->exptTimeSlots[priority].timeSlots[j].end);
            DebugP_assert(n < sizeof(buffer));
        }
        DPRINT("%s ", buffer);
    }
}
#endif

static void EnetEstApp_runListener(EnetQoSApp_AppCtx_t *ctx)
{
#if LISTNER_VERIFY_ENABLE
    EnetEstAppCtx_t *estAppCtx = (EnetEstAppCtx_t *)ctx;

    while (!EnetEstApp_isPTPClockStateSync(ctx, ctx->netdev[ctx->ifidx]))
    {
        DPRINT("Waiting for PTP clock to be synchronized!");
        CB_USLEEP(1000000ULL);
    }

    int err, schedIdx = 0;

    EnetEstApp_calcExpectedTimeSlot(ctx, &gEnetEstAppTestLists[schedIdx].list,
                                    &gEnetEstAppTestLists[schedIdx].stParam);

    err = EnetEstApp_runSchedule(ctx, &gEnetEstAppTestLists[schedIdx].list,
                                 ctx->netdev[ctx->ifidx]);
    if (err == 0)
    {
        estAppCtx->schedIdx = schedIdx;

        EnetEstApp_startListener(ctx);
    }
#endif
}

static void EnetEstApp_runBridgeMode(EnetQoSApp_AppCtx_t *ctx)
{
    int i, err, schedIdx = 0;

    for (i = 0; i < ctx->netdevSize; i++)
    {
        while (!EnetEstApp_isPTPClockStateSync(ctx, ctx->netdev[i]))
        {
            DPRINT("Waiting for PTP clock to be synchronized!");
            CB_USLEEP(1000000ULL);
        }
        err = EnetEstApp_runSchedule(ctx, &gEnetEstAppTestLists[schedIdx].list,
                                     ctx->netdev[i]);
        DebugP_assert(err == 0);
    }
}

static void *EnetApp_estAppTask(void *arg)
{
    int32_t status;
    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)arg;
    EnetQoSApp_AppCtx_t *ctx =  (EnetQoSApp_AppCtx_t *)&gEnetEstAppCtx;

    status = EnetQoSApp_initialize(ctx, modCtx,
                                   EnetEstApp_rxPacketHandler);
    DebugP_assert(status == 0);

    while (modCtx->enable)
    {
        char option = EnetTsnApp_getChar();

        switch(option)
        {
        case 't':
            EnetEstApp_runTalker(ctx);
            break;
        case 'l':
            EnetEstApp_runListener(ctx);
            break;
        case 'b':
            EnetEstApp_runBridgeMode(ctx);
            break;
        case 's':
            EnetQoSApp_printStats(ctx);
            break;
        case 'S':
            EnetQoSApp_resetStats(ctx);
            break;
        case 'd':
            EnetEstApp_showEstStats(ctx);
            break;
        default:
            EnetEstApp_showMemu();
            break;
        }
        TaskP_yield();
    }
    EnetEstApp_stopListener(ctx);

    return NULL;
}

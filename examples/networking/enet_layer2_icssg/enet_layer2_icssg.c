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

/*!
 * \file  enet_layer2_icssg.c
 *
 * \brief This file contains the implementation of the Enet layer2 Icssg example.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <include/core/enet_osal.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/sciclient.h>
#include <drivers/udma/udma_priv.h>
#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/icssg.h>
#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include <enet_appmemutils_cfg.h>
#include <enet_appboardutils.h>
/* SDK includes */
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Max number of ports supported per context */
#define ENETMP_PORT_MAX                          (ENET_SYSCFG_NUM_EXT_MAC_PORTS)


/* Max number of hardware RX channels. Note that this is different than Enet LLD's
 * RX channel concept which maps to UDMA hardware RX flows */
#define ENETMP_HW_RXCH_MAX                       (2U)
#define ENETMP_HW_RXFLOW_MAX                     (8U)

/* Local flag to disable a peripheral from test list */
#define ENETMP_DISABLED                          (0U)

/* Max number of ports that can be tested with this app */
#define ENETMP_ICSSG_INSTANCE_MAX                (2U)
#define ENETMP_PER_MAX                           (ENETMP_ICSSG_INSTANCE_MAX)

/* Task stack size */
#define ENETMP_TASK_STACK_SZ                     (10U * 1024U)

/* 100-ms periodic tick */
#define ENETMP_PERIODIC_TICK_MS                  (100U)

/*Counting Semaphore count*/
#define COUNTING_SEM_COUNT                       (10U)

/*Maximum Tx channels allowed*/
#define ENETMP_MAX_TX_CHANNEL_NUM                (8U)

/*Number of Bucket entires in one FDB slot*/
#define ICSSG_NUM_FDB_BUCKET_ENTRIES             (4U)
/*Number of slots in FDB*/
#define ICSSG_NUM_OF_SLOTS                       (512U)
/*ICSSG FDB Size is Slots*Bucket_Entries  */
#define ICSSG_SIZE_OF_FDB                        (2048U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/* Test parameters for each port in the multiport test */
typedef struct EnetMp_contextName_s
{
    /* Name to be used for logging */
    char *name;
} EnetMp_contextName;

/* Context of a peripheral/port */
typedef struct EnetMp_PerCtxt_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* Number of RX Flows associated to this context */
    uint32_t rxChCount;

    /* Start Channel ID of the RX Flow associated to this context. It is assumed
     * that 'rxChCount' number of Rx Flows for this context is allocated contiguously
     * from rxChStartId. Note that concept of channels inside flows is hidden to application */
    uint32_t rxChStartId;

    /* Number of TX Channels associated to this context */
    uint32_t txChCount;

    /* Start Channel ID of the TX Channels associated to this context. It is assumed
     * that 'txChCount' number of Tx Channels for this context is allocated contiguously
     * from txChStartId */
    uint32_t txChStartId;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPort[ENETMP_PORT_MAX];

    /* Number of MAC ports in macPorts array */
    uint8_t macPortNum;

    /* Name of this port to be used for logging */
    char *name;

    /* ICSSG configuration */
    Icssg_Cfg icssgCfg;

    /* Number of valid MAC address entries present in macAddr variable below*/
    uint8_t numValidMacAddress;

    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_SYSCFG_RX_FLOWS_NUM][ENET_MAC_ADDR_LEN];

    /* UDMA driver configuration */
    EnetUdma_Cfg dmaCfg;

    /* TX channel number */
    uint32_t txChNum[ENET_SYSCFG_TX_CHANNELS_NUM];

    /* TX channel handle */
    EnetDma_TxChHandle hTxCh[ENET_SYSCFG_TX_CHANNELS_NUM];

    /* Start flow index */
    uint32_t rxStartFlowIdx[ENET_SYSCFG_RX_FLOWS_NUM];

    /* Flow index */
    uint32_t rxFlowIdx[ENET_SYSCFG_RX_FLOWS_NUM];

    /* RX channel handle */
    EnetDma_RxChHandle hRxCh[ENET_SYSCFG_RX_FLOWS_NUM];

    /* RX task handle - receives packets, changes source/dest MAC addresses
     * and transmits the packets back */
    TaskP_Object rxTaskObj;

    /* Semaphore posted from RX callback when packets have arrived */
    SemaphoreP_Object rxSemObj;

    /* Semaphore used to synchronize all RX tasks exits */
    SemaphoreP_Object rxDoneSemObj;

    /* Semaphore posted from event callback upon asynchronous IOCTL completion */
    SemaphoreP_Object ayncIoctlSemObj;

    /* Timestamp of the last received packets */
    uint64_t rxTs[ENET_SYSCFG_TOTAL_NUM_RX_PKT];

    /* Timestamp of the last transmitted packets */
    uint64_t txTs[ENET_SYSCFG_TOTAL_NUM_RX_PKT];

    /* Sequence number used as cookie for timestamp events. This value is passed
     * to the DMA packet when submitting a packet for transmission. Driver will
     * pass this same value when timestamp for that packet is ready. */
    uint32_t txTsSeqId;

    /* Semaphore posted from event callback upon asynchronous IOCTL completion */
    SemaphoreP_Object txTsSemObj;

    /* Enet and Udma handle info for the peripheral */
    EnetApp_HandleInfo handleInfo;

   /* Core attach info for the peripheral */
    EnetPer_AttachCoreOutArgs attachInfo;

} EnetMp_PerCtxt;

typedef struct EnetMp_Obj_s
{
    /* Flag which indicates if test shall run */
    volatile bool run;

    /* This core's id */
    uint32_t coreId;

    /* Queue of free TX packets */
    EnetDma_PktQ txFreePktInfoQ;

    /* Array of all peripheral/port contexts used in the test */
    EnetMp_PerCtxt perCtxt[ENETMP_PER_MAX];

    /* Number of active contexts being used */
    uint32_t numPerCtxts;

    /* Whether promiscuous mode is enabled or not */
    bool promisc;

    /* Whether timestamp are enabled or not */
    bool enableTs;
} EnetMp_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetMp_mainTask(void *args);

static int32_t EnetMp_init(void);

static int32_t EnetMp_open(EnetMp_PerCtxt *perCtxts,
                           uint32_t numPerCtxts);

static void EnetMp_close(EnetMp_PerCtxt *perCtxts,
                         uint32_t numPerCtxts);

static void EnetMp_togglePromisc(EnetMp_PerCtxt *perCtxts,
                                 uint32_t numPerCtxts);

static void EnetMp_enableDscpPriority(EnetMp_PerCtxt *perCtxts,
                                 uint32_t numPerCtxts);

static void EnetMp_printStats(EnetMp_PerCtxt *perCtxts,
                              uint32_t numPerCtxts);

static void EnetMp_resetStats(EnetMp_PerCtxt *perCtxts,
                              uint32_t numPerCtxts);

static void EnetMp_showMacAddrs(EnetMp_PerCtxt *perCtxts,
                                uint32_t numPerCtxts);

static void EnetMp_readAllFdbSlots(EnetMp_PerCtxt *perCtxts,
                                uint32_t numPerCtxts);                                

static int32_t EnetMp_waitForLinkUp(EnetMp_PerCtxt *perCtxt);

static void EnetMp_macMode2MacMii(emac_mode macMode,
                                  EnetMacPort_Interface *mii);

static int32_t EnetMp_openDma(EnetMp_PerCtxt *perCtxt,uint32_t  perCtxtIndex);

static void EnetMp_closeDma(EnetMp_PerCtxt *perCtxt,uint32_t  perCtxtIndex);

static void EnetMp_initTxFreePktQ(void);

static void EnetMp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);

static uint32_t EnetMp_retrieveFreeTxPkts(EnetMp_PerCtxt *perCtxt);

static void EnetMp_createRxTask(EnetMp_PerCtxt *perCtxt,
                                uint8_t *taskStack,
                                uint32_t taskStackSize);

static void EnetMp_destroyRxTask(EnetMp_PerCtxt *perCtxt);

static void EnetMp_rxTask(void *args);

extern uint32_t Board_getPhyAddr(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Enet multiport test object */
EnetMp_Obj gEnetMp;

/* Statistics */
IcssgStats_MacPort gEnetMp_icssgStats;
IcssgStats_Pa gEnetMp_icssgPaStats;
uint32_t reqTs = 0;

uint32_t count[2][8] = {{0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}};

/* Test application stack */
static uint8_t gEnetMpTaskStackRx[ENET_SYSCFG_MAX_ENET_INSTANCES][ENETMP_TASK_STACK_SZ] __attribute__ ((aligned(32)));

// #define ENET_TEST_MII_MODE
// #define DUAL_MAC_MODE  /* TODO: Need to allocate TX channels as 2 in enet_cfg.h file to get both MAC ports work simultaneously*/
/* Use this array to select the ports that will be used in the test */
static EnetMp_contextName testParamsName[] =
{
    { "icssg1-p1" },
    { "icssg1-p2" },
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void EnetMp_showMenu(void)
{
    EnetAppUtils_print("\nEnet Multiport Menu:\r\n");
    EnetAppUtils_print(" 'T'  -  Enable timestamp prints\r\n");
    EnetAppUtils_print(" 't'  -  Disable timestamp prints\r\n");
    EnetAppUtils_print(" 's'  -  Print statistics\r\n");
    EnetAppUtils_print(" 'r'  -  Reset statistics\r\n");
    EnetAppUtils_print(" 'm'  -  Show allocated MAC addresses\r\n");
    EnetAppUtils_print(" 'd'  -  Enable dscp based priority mapping\r\n");
    EnetAppUtils_print(" 'f'  -  Read all ICSSG FDB slots\r\n");
    EnetAppUtils_print(" 'x'  -  Stop the test\r\n\n");
}

void EnetMp_mainTask(void *args)
{
    char option;
    uint32_t i;
    int32_t status = ENET_SOK;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("==========================\r\n");
    DebugP_log("      MULTIPORT TEST      \r\n");
    DebugP_log("==========================\r\n");

    /* Initialize test config */
    memset(&gEnetMp, 0, sizeof(gEnetMp));
    gEnetMp.run = true;
    gEnetMp.promisc = false;
    gEnetMp.enableTs = false;

    gEnetMp.numPerCtxts = ENET_SYSCFG_NUM_PERIPHERAL;

    for (i = 0U; i < gEnetMp.numPerCtxts; i++)
    {
        EnetApp_getEnetInstInfo(CONFIG_ENET_ICSS0 + i, &gEnetMp.perCtxt[i].enetType, &gEnetMp.perCtxt[i].instId);
        gEnetMp.perCtxt[i].name = testParamsName[i].name; /* shallow copy */

        EnetApp_getEnetInstMacInfo(gEnetMp.perCtxt[i].enetType,
                                   gEnetMp.perCtxt[i].instId,
                                   gEnetMp.perCtxt[i].macPort,
                                   &gEnetMp.perCtxt[i].macPortNum);
    }

    /* Init driver */
    EnetMp_init();

    /* Open all peripherals */
    status = EnetMp_open(gEnetMp.perCtxt, gEnetMp.numPerCtxts);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to open peripherals: %d\r\n", status);
    }

    if (status == ENET_SOK)
    {
        /* Wait for user input to exit the test */
        EnetMp_showMenu();
        while (true)
        {
            option = ' ';
            status = DebugP_scanf("%c", &option);
            if (option == 'x')
            {
                EnetAppUtils_print("Stopping...\r\n");
                gEnetMp.run = false;
                break;
            }
            else if (option == 't')
            {
                EnetAppUtils_print("Disable timestamp prints\r\n");
                gEnetMp.enableTs = false;
            }
            else if (option == 'T')
            {
                EnetAppUtils_print("Enable timestamp prints\r\n");
                gEnetMp.enableTs = true;
            }
            else if (option == 'p')
            {
                EnetMp_togglePromisc(gEnetMp.perCtxt, gEnetMp.numPerCtxts);
            }
            else if (option == 's')
            {
                EnetMp_printStats(gEnetMp.perCtxt, gEnetMp.numPerCtxts);
            }
            else if (option == 'r')
            {
                EnetMp_resetStats(gEnetMp.perCtxt, gEnetMp.numPerCtxts);
            }
            else if (option == 'm')
            {
                EnetMp_showMacAddrs(gEnetMp.perCtxt, gEnetMp.numPerCtxts);
            }
            else if (option == 'd')
            {
                EnetMp_enableDscpPriority(gEnetMp.perCtxt, gEnetMp.numPerCtxts);
            }
            else if (option == 'f')
            {
                EnetMp_readAllFdbSlots(gEnetMp.perCtxt, gEnetMp.numPerCtxts);
            }
            else
            {
                EnetAppUtils_print("Invalid option, try again...\r\n");
                EnetMp_showMenu();
            }
            TaskP_yield();
        }

        /* Print statistics */
        EnetMp_printStats(gEnetMp.perCtxt, gEnetMp.numPerCtxts);

        /* Wait until RX tasks have exited */
        for (i = 0U; i < gEnetMp.numPerCtxts; i++)
        {
            EnetAppUtils_print("Waiting for RX task %u to exit\r\n", i+1);
            SemaphoreP_post(&gEnetMp.perCtxt[i].rxSemObj);
            SemaphoreP_pend(&gEnetMp.perCtxt[i].rxDoneSemObj, SystemP_WAIT_FOREVER);
        }

        EnetAppUtils_print("All RX tasks have exited\r\n");
    }

    /* Close all peripherals */
    EnetMp_close(gEnetMp.perCtxt, gEnetMp.numPerCtxts);

}

static int32_t EnetMp_init(void)
{
    int32_t status = ENET_SOK;
    gEnetMp.coreId = EnetSoc_getCoreId();

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetMp.txFreePktInfoQ);
    return status;

}

static void EnetMp_asyncIoctlCb(Enet_Event evt,
                                uint32_t evtNum,
                                void *evtCbArgs,
                                void *arg1,
                                void *arg2)
{
    EnetMp_PerCtxt *perCtxt = (EnetMp_PerCtxt *)evtCbArgs;

    EnetAppUtils_print("%s: Async IOCTL completed\r\n", perCtxt->name);
    SemaphoreP_post(&perCtxt->ayncIoctlSemObj);
}

static void EnetMp_txTsCb(Enet_Event evt,
                          uint32_t evtNum,
                          void *evtCbArgs,
                          void *arg1,
                          void *arg2)
{
    EnetMp_PerCtxt *perCtxt = (EnetMp_PerCtxt *)evtCbArgs;
    Icssg_TxTsEvtCbInfo *txTsInfo = (Icssg_TxTsEvtCbInfo *)arg1;
    Enet_MacPort macPort = *(Enet_MacPort *)arg2;
    uint32_t tsId = txTsInfo->txTsId;
    uint64_t txTs = txTsInfo->ts;
    uint64_t rxTs = perCtxt->rxTs[tsId % ENET_SYSCFG_TOTAL_NUM_RX_PKT];
    uint64_t prevTs;
    int64_t dt;
    bool status = true;

    dt = txTs - rxTs;

    EnetAppUtils_print("%s: Port %u: RX-to-TX timestamp delta = %10lld (RX=%llu, TX=%llu)\r\n",
                       perCtxt->name, ENET_MACPORT_ID(macPort), dt, rxTs, txTs);

    /* Check correct timestamp delta */
    if (dt < 0)
    {
        EnetAppUtils_print("%s: Port %u: ERROR: RX timestamp > TX timestamp: %llu > %llu\r\n",
                           perCtxt->name, ENET_MACPORT_ID(macPort), rxTs, txTs);
            status = false;
    }

    /* Check monotonicity of the TX and RX timestamps */
    if (txTsInfo->txTsId > 0U)
    {
        prevTs = perCtxt->rxTs[(tsId - 1) % ENET_SYSCFG_TOTAL_NUM_RX_PKT];
        if (prevTs > rxTs)
        {
            EnetAppUtils_print("%s: Port %u: ERROR: Non monotonic RX timestamp: %llu -> %llu\r\n",
                               perCtxt->name, ENET_MACPORT_ID(macPort), prevTs, rxTs);
            status = false;
        }

        prevTs = perCtxt->txTs[(tsId - 1) % ENET_SYSCFG_TOTAL_NUM_RX_PKT];
        if (prevTs > txTs)
        {
            EnetAppUtils_print("%s: Port %u: ERROR: Non monotonic TX timestamp: %llu -> %llu\r\n",
                               perCtxt->name, ENET_MACPORT_ID(macPort), prevTs, txTs);
            status = false;
        }
    }

    if (!status)
    {
        EnetAppUtils_print("\r\n");
    }

    /* Save current timestamp for future monotonicity checks */
     perCtxt->txTs[txTsInfo->txTsId % ENET_SYSCFG_TOTAL_NUM_RX_PKT] = txTs;
    reqTs--;
    SemaphoreP_post(&perCtxt->txTsSemObj);
}

static void EnetMp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                          bool isLinkUp,
                                          void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

int32_t EnetMp_getPerIdx(Enet_Type enetType, uint32_t instId, uint32_t *perIdx)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Initialize async IOCTL and TX timestamp semaphores */
    for (i = 0U; i < gEnetMp.numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &(gEnetMp.perCtxt[i]);
        if ((perCtxt->enetType == enetType) && (perCtxt->instId == instId))
        {
            break;
        }
    }
    if (i < gEnetMp.numPerCtxts)
    {
        *perIdx = i;
        status = ENET_SOK;
    }
    else
    {
        status = ENET_ENOTFOUND;
    }
    return status;
}

int32_t EnetMp_mapPerCtxt2Idx(EnetMp_PerCtxt *perCtxt, uint32_t *perIdx)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    for (i = 0; i < gEnetMp.numPerCtxts;i++)
    {
        if (&gEnetMp.perCtxt[i] == perCtxt)
        {
            break;
        }
    }
    if (i < gEnetMp.numPerCtxts)
    {
        *perIdx = i;
    }
    else
    {
        status = ENET_EFAIL;
    }
    return status;
}

void EnetApp_updateIcssgInitCfg(Enet_Type enetType, uint32_t instId, Icssg_Cfg *icssgCfg)
{
    EnetRm_ResCfg *resCfg;
    uint32_t i;
    uint32_t perIdx;
    int32_t status;

    /* Prepare init configuration for all peripherals */
    EnetAppUtils_print("\nInit  configs EnetType:%u, InstId :%u\r\n", enetType, instId);
    EnetAppUtils_print("----------------------------------------------\r\n");

#if defined(ENET_TEST_MII_MODE)
    icssgCfg->mii.layerType    = ENET_MAC_LAYER_MII;
    icssgCfg->mii.sublayerType = ENET_MAC_SUBLAYER_STANDARD;
    icssgCfg->mii.variantType  = ENET_MAC_VARIANT_NONE;
#endif

    resCfg = &icssgCfg->resCfg;

    /* We use software MAC address pool from apputils, but it will give same MAC address.
        * Add port index to make them unique */
    status = EnetMp_getPerIdx(enetType, instId, &perIdx);
    EnetAppUtils_assert(status == ENET_SOK);
    for (i = 0U; i < ENETMP_PORT_MAX; i++)
    {
        resCfg->macList.macAddress[i][ENET_MAC_ADDR_LEN - 1] += (perIdx * ENETMP_PORT_MAX) + i;
    }
    resCfg->macList.numMacAddress = ENETMP_PORT_MAX;
}

static int32_t EnetMp_open(EnetMp_PerCtxt *perCtxts,
                           uint32_t numPerCtxts)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Initialize async IOCTL and TX timestamp semaphores */
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[i];

        status = SemaphoreP_constructBinary(&perCtxt->ayncIoctlSemObj, 0);
        DebugP_assert(SystemP_SUCCESS == status);

        status = SemaphoreP_constructBinary(&perCtxt->txTsSemObj, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    /* Do peripheral dependent initalization */
    EnetAppUtils_print("\nInit all peripheral clocks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];
        EnetAppUtils_enableClocks(perCtxt->enetType, perCtxt->instId);
    }

    /* Open Enet driver for all peripherals */
    EnetAppUtils_print("\nOpen all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");

    EnetApp_driverInit();

    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];

        status = EnetApp_driverOpen(perCtxt->enetType, perCtxt->instId);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        }

        EnetApp_acquireHandleInfo(perCtxt->enetType, perCtxt->instId, &(perCtxt->handleInfo));
    }

    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];

        if (Enet_isIcssFamily(perCtxt->enetType))
        {
            EnetAppUtils_print("%s: Register async IOCTL callback\r\n", perCtxt->name);
            Enet_registerEventCb(perCtxt->handleInfo.hEnet,
                                 ENET_EVT_ASYNC_CMD_RESP,
                                 0U,
                                 EnetMp_asyncIoctlCb,
                                 (void *)perCtxt);

            EnetAppUtils_print("%s: Register TX timestamp callback\r\n", perCtxt->name);
            perCtxt->txTsSeqId = 0U;
            Enet_registerEventCb(perCtxt->handleInfo.hEnet,
                                 ENET_EVT_TIMESTAMP_TX,
                                 0U,
                                 EnetMp_txTsCb,
                                 (void *)perCtxt);
        }
    }

    /* Attach the core with RM */
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\nAttach core id %u on all peripherals\r\n", gEnetMp.coreId);
        EnetAppUtils_print("----------------------------------------------\r\n");
        for (i = 0U; i < numPerCtxts; i++)
        {
            EnetMp_PerCtxt *perCtxt = &perCtxts[i];

            EnetAppUtils_print("%s: Attach core\r\n", perCtxt->name);

            EnetApp_coreAttach(perCtxt->enetType, perCtxt->instId, gEnetMp.coreId, &perCtxt->attachInfo);
        }
    }

    /* Create RX tasks for each peripheral */
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\nCreate RX tasks\r\n");
        EnetAppUtils_print("----------------------------------------------\r\n");
        for (i = 0U; i < numPerCtxts; i++)
        {
            EnetMp_PerCtxt *perCtxt = &perCtxts[i];

            EnetAppUtils_print("%s: Create RX task\r\n", perCtxt->name);

            EnetMp_createRxTask(perCtxt, &gEnetMpTaskStackRx[i][0U], ENETMP_TASK_STACK_SZ);
        }
    }

    return status;
}

static void EnetMp_close(EnetMp_PerCtxt *perCtxts,
                         uint32_t numPerCtxts)
{
    uint32_t i;

    EnetAppUtils_print("\nClose DMA for all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];

        EnetAppUtils_print("%s: Close DMA\r\n", perCtxt->name);

        EnetMp_closeDma(perCtxt, i);
    }

    /* Delete RX tasks created for all peripherals */
    EnetAppUtils_print("\nDelete RX tasks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_destroyRxTask(&perCtxts[i]);
    }

    /* Detach core */
    EnetAppUtils_print("\nDetach core from all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];
        EnetAppUtils_print("%s: Detach core\r\n", perCtxt->name);
        EnetApp_coreDetach(perCtxt->enetType, perCtxt->instId, gEnetMp.coreId, perCtxt->attachInfo.coreKey);
    }

    /* Close opened Enet drivers if any peripheral failed */
    EnetAppUtils_print("\nClose all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];
        EnetAppUtils_print("%s: Close enet\r\n", perCtxt->name);

        if (Enet_isIcssFamily(perCtxt->enetType))
        {
            EnetAppUtils_print("%s: Unregister async IOCTL callback\r\n", perCtxt->name);
            Enet_unregisterEventCb(perCtxt->handleInfo.hEnet,
                                   ENET_EVT_ASYNC_CMD_RESP,
                                   0U);

            EnetAppUtils_print("%s: Unregister TX timestamp callback\r\n", perCtxt->name);
            Enet_unregisterEventCb(perCtxt->handleInfo.hEnet,
                                   ENET_EVT_TIMESTAMP_TX,
                                   0U);
        }

        EnetApp_releaseHandleInfo(perCtxt->enetType, perCtxt->instId);
        perCtxt->handleInfo.hEnet = NULL;
    }

    EnetApp_driverDeInit();

    /* Do peripheral dependent initalization */
    EnetAppUtils_print("\nDeinit all peripheral clocks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];
        EnetAppUtils_disableClocks(perCtxt->enetType, perCtxt->instId);
    }
    /* Destroy async IOCTL semaphore */
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];

        SemaphoreP_destruct(&perCtxt->ayncIoctlSemObj);
    }
}

static void EnetMp_togglePromisc(EnetMp_PerCtxt *perCtxts,
                                 uint32_t numPerCtxts)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    uint32_t j;
    int32_t status;

    gEnetMp.promisc = !gEnetMp.promisc;
    EnetAppUtils_print("\n%s promiscuous mode\r\n", gEnetMp.promisc ? "Enable" : "Disable");

    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[i];

        /* Promiscuous test in this app not implemented for CPSW, only for ICSSG */
        if (Enet_isIcssFamily(perCtxt->enetType))
        {

            for (j = 0U; j < perCtxt->macPortNum; j++)
            {
                macPort = perCtxt->macPort[j];
                ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);

                if(gEnetMp.promisc == 1)
                {
                    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_MACPORT_IOCTL_ENABLE_PROMISC_MODE, &prms, status);
                }
                else
                {
                    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_MACPORT_IOCTL_DISABLE_PROMISC_MODE, &prms, status);
                }
                if (status != ENET_SOK)
                {
                    EnetAppUtils_print("%s: Failed to set promisc mode: %d\r\n",
                                       perCtxt->name, status);
                    continue;
                }
            }
        }
    }
}

static void EnetMp_enableDscpPriority(EnetMp_PerCtxt *perCtxts,
                                 uint32_t numPerCtxts)
{
    Enet_IoctlPrms prms;
    EnetMacPort_SetIngressDscpPriorityMapInArgs inArgs;
    uint32_t i;
    int32_t status;

    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[i];
        /*Configure FT3 filter and classifier and Enable DSCP*/
        memset(&inArgs, 0, sizeof(inArgs));
        inArgs.macPort = ENET_MAC_PORT_1;
        /* mapped most used 8 dscp priority points to 8 qos levels remaining are routed to 0
           Write a non zero value to required dscp from 0 to 63 in increasing priority order
        */
        inArgs.dscpPriorityMap.tosMap[0]  = 0U;
        inArgs.dscpPriorityMap.tosMap[10] = 1U;
        inArgs.dscpPriorityMap.tosMap[18] = 2U;
        inArgs.dscpPriorityMap.tosMap[26] = 3U;
        inArgs.dscpPriorityMap.tosMap[34] = 4U;
        inArgs.dscpPriorityMap.tosMap[46] = 5U;
        inArgs.dscpPriorityMap.tosMap[48] = 6U;
        inArgs.dscpPriorityMap.tosMap[56] = 7U;

        inArgs.dscpPriorityMap.dscpIPv4En = 1;

        ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
        ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to set dscp Priority map for Port1\r\n", perCtxt->name);
        }

        if (status == ENET_SOK && (perCtxt->enetType == ENET_ICSSG_SWITCH))
        {
            inArgs.macPort = ENET_MAC_PORT_2;

            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to set dscp Priority map for Port2\r\n", perCtxt->name);
            }
        }
    }

    EnetAppUtils_print("\nDSCP based Priority mapping is Enabled\r\n");
}

static void EnetMp_printStats(EnetMp_PerCtxt *perCtxts,
                              uint32_t numPerCtxts)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    uint32_t j;
    int32_t status;

    EnetAppUtils_print("\nPrint statistics\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[i];

        if (Enet_isIcssFamily(perCtxt->enetType))
        {
            EnetAppUtils_print("\n %s - PA statistics\r\n", perCtxt->name);
            EnetAppUtils_print("--------------------------------\r\n");
            ENET_IOCTL_SET_OUT_ARGS(&prms, &gEnetMp_icssgPaStats);
            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to get PA stats\r\n", perCtxt->name);
            }

            EnetAppUtils_printIcssgPaStats(&gEnetMp_icssgPaStats);
            EnetAppUtils_print("\r\n");
        }

        for (j = 0U; j < perCtxt->macPortNum; j++)
        {
            macPort = ENET_MAC_PORT_1 + j;

            EnetAppUtils_print("\n %s - Port %u statistics\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
            EnetAppUtils_print("--------------------------------\r\n");

            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &gEnetMp_icssgStats);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to get port %u stats\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
                continue;
            }

            EnetAppUtils_printIcssgMacPortStats(&gEnetMp_icssgStats, false);

            EnetAppUtils_print("\r\n");
        }
    }
}

static void EnetMp_resetStats(EnetMp_PerCtxt *perCtxts,
                              uint32_t numPerCtxts)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    uint32_t j;
    int32_t status;

    EnetAppUtils_print("\nReset statistics\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[i];

        EnetAppUtils_print("%s: Reset statistics\r\n", perCtxt->name);

        ENET_IOCTL_SET_NO_ARGS(&prms);
        ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_STATS_IOCTL_RESET_HOSTPORT_STATS, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to reset  host port stats\r\n", perCtxt->name);
            continue;
        }
        for (j = 0U; j < perCtxt->macPortNum; j++)
        {
            macPort = perCtxt->macPort[j];

            ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_STATS_IOCTL_RESET_MACPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to reset port %u stats\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
                continue;
            }
        }
    }
}

static void EnetMp_showMacAddrs(EnetMp_PerCtxt *perCtxts,
                                uint32_t numPerCtxts)
{
    EnetAppUtils_print("\nAllocated MAC addresses\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (uint32_t ctxIdx = 0U; ctxIdx < numPerCtxts; ctxIdx++)
    {
        EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[ctxIdx];
        EnetAppUtils_print("%s: \t", perCtxt->name);
        for (uint32_t macAddrIdx = 0U; macAddrIdx < perCtxt->numValidMacAddress; macAddrIdx++)
        {
                EnetAppUtils_printMacAddr(&perCtxt->macAddr[macAddrIdx][0]);
        }
    }
}

static void EnetMp_readAllFdbSlots(EnetMp_PerCtxt *perCtxts,
                                uint32_t numPerCtxts)
{
    uint16_t hw_fdb_slot = 0;    
    int32_t semStatus;
    int32_t retVal_1;
    int32_t retVal_2;    
    uint8_t macAddr[ENET_MAC_ADDR_LEN];
    uint8_t fid_c1;
    uint8_t fid_c2;     
      
    for (uint32_t ctxIdx = 0U; ctxIdx < numPerCtxts; ctxIdx++)
    {
        EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[ctxIdx]; 
        EnetAppUtils_print("\n\nPeripheral Context: %u\n\r", ctxIdx);
        for(hw_fdb_slot = 0U; hw_fdb_slot < ICSSG_NUM_OF_SLOTS; hw_fdb_slot++)
        {
            Enet_IoctlPrms prms_1, prms_2;
            Icssg_FdbEntry_GetSlotOutArgs fdbResult;
            Icssg_FdbEntry_ReadSlotInArgs params;
            retVal_1 = ENET_EFAIL;
            retVal_2 = ENET_EFAIL;
            params.broadSideSlot = hw_fdb_slot;
            ENET_IOCTL_SET_IN_ARGS(&prms_1, &params);
            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_FDB_IOCTL_READ_SLOT_ENTRIES, &prms_1, retVal_1);

            if (retVal_1 == ENET_SINPROGRESS)
            {   
                EnetAppUtils_print("\n\nSuccess: IOCTL command to read the FDB entries for slot: %u\n\r", hw_fdb_slot);
                do
                {
                    Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
                    semStatus = SemaphoreP_pend(&perCtxt->ayncIoctlSemObj, 1U);
                } while (gEnetMp.run && (semStatus != SystemP_SUCCESS));

                retVal_1 = ENET_SOK;
            }
            else
            {
                EnetAppUtils_print("ERROR: IOCTL command sent for reading entries of requested FDB slot\n\r");
            }

            if(retVal_1 == ENET_SOK)
            {
                /*Printing the out args which is a 32B slot result with 4 entries*/
                ENET_IOCTL_SET_OUT_ARGS(&prms_2, &fdbResult);
                ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_FDB_IOCTL_GET_SLOT_ENTRIES, &prms_2, retVal_2);
                if (retVal_2 == ENET_SINPROGRESS)
                {
                    EnetAppUtils_print("Success: IOCTL command to get FDB result for slot: %u\n\r", hw_fdb_slot);
                    do
                    {
                        Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
                        semStatus = SemaphoreP_pend(&perCtxt->ayncIoctlSemObj, 1U);
                    } while (gEnetMp.run && (semStatus != SystemP_SUCCESS));

                    retVal_2 = ENET_SOK;
                }
                else if (retVal_2 != ENET_SOK)
                {
                    EnetAppUtils_print("ERROR: IOCTL command sent for getting result of requested FDB slot\n\r");
                }
                if (retVal_2 == ENET_SOK)
                {
                    EnetAppUtils_print("Success: IOCTL command to get FDB result for slot: %u\n\r", hw_fdb_slot);
                    /*Extracting the bucket entry from the OutArg*/
                    for(uint32_t bucket_index = 0; bucket_index < ICSSG_NUM_FDB_BUCKET_ENTRIES; bucket_index++)
                    {
                        for(uint32_t i=0; i < ENET_MAC_ADDR_LEN; i++)
                        {  
                            macAddr[i] = fdbResult.fdbSlotEntries[bucket_index].macAddr[i];
                        }
                        fid_c1 = fdbResult.fdbSlotEntries[bucket_index].fid_c1;
                        fid_c2 = fdbResult.fdbSlotEntries[bucket_index].fid_c2;

                        /*Printing the non-zero FDB entries in each slot*/
                        if(fid_c1 | fid_c2)
                        {
                            EnetAppUtils_print("\nSlot: %u \n\r", hw_fdb_slot);
                            EnetAppUtils_print("Bucket- %u : MAC: %02x:%02x:%02x:%02x:%02x:%02x FID_C1:%02x FID_C2:%02x \n\r",(bucket_index+1),
                                        macAddr[0],macAddr[1],macAddr[2],macAddr[3],macAddr[4],macAddr[5],
                                        fid_c1,
                                        fid_c2);
                        }
                        else
                        {
                        EnetAppUtils_print("Slot: %u Bucket: %u is empty\n\r", hw_fdb_slot, bucket_index); 
                        }
                    }
                }
                else
                {
                    EnetAppUtils_print("Error: Getting fdb result for %u\n\r", hw_fdb_slot);
                }
            }
            else
            {
                EnetAppUtils_print("Error reading the FDB IOCTL entry for slot: %u\n\r", hw_fdb_slot);
            }
        }
    }
}

static int32_t EnetMp_waitForLinkUp(EnetMp_PerCtxt *perCtxt)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    IcssgMacPort_SetPortStateInArgs setPortStateInArgs;
    bool linked;
    uint32_t i;
    int32_t status = ENET_SOK;

    EnetAppUtils_print("%s: Waiting for link up...\r\n", perCtxt->name);

    for (i = 0U; i < perCtxt->macPortNum; i++)
    {
        macPort = perCtxt->macPort[i];
        linked = false;

        while (gEnetMp.run && !linked)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to get port %u link status: %d\r\n",
                                   perCtxt->name, ENET_MACPORT_ID(macPort), status);
                linked = false;
                break;
            }

            if (!linked)
            {
                ClockP_usleep(1000);
            }
        }

        if (gEnetMp.run)
        {
            EnetAppUtils_print("%s: Port %u link is %s\r\n",
                               perCtxt->name, ENET_MACPORT_ID(macPort), linked ? "up" : "down");

            /* Set port to 'Forward' state */
            if (status == ENET_SOK)
            {
                EnetAppUtils_print("%s: Set port state to 'Forward'\r\n", perCtxt->name);

                setPortStateInArgs.macPort   = macPort;
                setPortStateInArgs.portState = ICSSG_PORT_STATE_FORWARD;
                ENET_IOCTL_SET_IN_ARGS(&prms, &setPortStateInArgs);

                ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_PER_IOCTL_SET_PORT_STATE, &prms, status);
                if (status == ENET_SINPROGRESS)
                {
                    /* Wait for asyc ioctl to complete */
                    do
                    {
                        Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
                        status = SemaphoreP_pend(&perCtxt->ayncIoctlSemObj, SystemP_WAIT_FOREVER);
                        if (SystemP_SUCCESS == status)
                        {
                            break;
                        }
                    } while (1);

                    status = ENET_SOK;
                }
                else
                {
                    EnetAppUtils_print("%s: Failed to set port state: %d\n", perCtxt->name, status);
                }
            }
        }
    }

    return status;
}

static void EnetMp_macMode2MacMii(emac_mode macMode,
                                  EnetMacPort_Interface *mii)
{
    switch (macMode)
    {
        case RMII:
            mii->layerType    = ENET_MAC_LAYER_MII;
            mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
            mii->variantType  = ENET_MAC_VARIANT_NONE;
            break;

        case RGMII:
            mii->layerType    = ENET_MAC_LAYER_GMII;
            mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
            mii->variantType  = ENET_MAC_VARIANT_FORCED;
            break;
        default:
            EnetAppUtils_print("Invalid MAC mode: %u\r\n", macMode);
            EnetAppUtils_assert(false);
            break;
    }
}


static void EnetMp_rxIsrFxn(void *appData)
{
    EnetMp_PerCtxt *perCtxt = (EnetMp_PerCtxt *)appData;

    SemaphoreP_post(&perCtxt->rxSemObj);
}

static int32_t EnetMp_openDma(EnetMp_PerCtxt *perCtxt, uint32_t perCtxtIndex)
{
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    int32_t status = ENET_SOK;

    /* Open the TX channel */
#if (ENET_SYSCFG_MAX_ENET_INSTANCES > 1)
    const uint32_t startChId = (perCtxtIndex == 0) ? CONFIG_ENET_ICSS0_TX_CH_START : CONFIG_ENET_ICSS1_TX_CH_START;
    const uint32_t txChCount = (perCtxtIndex == 0) ? CONFIG_ENET_ICSS0_TX_CH_COUNT : CONFIG_ENET_ICSS1_TX_CH_COUNT;
#else
    const uint32_t startChId = CONFIG_ENET_ICSS0_TX_CH_START;
    const uint32_t txChCount = CONFIG_ENET_ICSS0_TX_CH_COUNT;
#endif
    perCtxt->txChStartId = startChId;
    perCtxt->txChCount   = txChCount;
    for (uint32_t chIdx = 0; chIdx < txChCount; chIdx++)
    {
        /* Open the TX channel */
        const uint32_t txChId = startChId + chIdx;
        txInArgs.enetType = perCtxt->enetType;
        txInArgs.instId   = perCtxt->instId;
        txInArgs.cbArg    = NULL;
        txInArgs.notifyCb = NULL;

        EnetApp_getTxDmaHandle(txChId,
                               &txInArgs,
                               &txChInfo);

        perCtxt->txChNum[chIdx] = txChInfo.txChNum;
        perCtxt->hTxCh[chIdx]   = txChInfo.hTxCh;
        EnetAppUtils_assert(txChInfo.useGlobalEvt == true);

        if (perCtxt->hTxCh[chIdx] == NULL)
        {
#if FIX_RM
            /* Free the channel number if open Tx channel failed */
            EnetAppUtils_freeTxCh(perCtxt->handleInfo.hEnet,
                                  perCtxt->attachInfo.coreKey,
                                  appPnHandle->gEnetPn.coreId,
                                  perCtxt->txChNum[chIdx]);
#endif
            EnetAppUtils_print("EnetMp_openDma() failed to open TX channel\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(perCtxt->hTxCh[chIdx] != NULL);
        }

        /* Allocate TX packets and keep them locally enqueued */
        if (status == ENET_SOK)
        {
            EnetMp_initTxFreePktQ();
        }
    }

    /* Open the RX flow */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     rxInArgs;
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;
#if (ENET_SYSCFG_MAX_ENET_INSTANCES > 1)
        const uint32_t startChId = (perCtxtIndex == 0) ? CONFIG_ENET_ICSS0_RX_CH_START : CONFIG_ENET_ICSS1_RX_CH_START;
        const uint32_t rxChCount = (perCtxtIndex == 0) ? CONFIG_ENET_ICSS0_RX_CH_COUNT : CONFIG_ENET_ICSS1_RX_CH_COUNT;
#else
    const uint32_t startChId = CONFIG_ENET_ICSS0_RX_CH_START;
    const uint32_t rxChCount = CONFIG_ENET_ICSS0_RX_CH_COUNT;
#endif
        perCtxt->rxChStartId = startChId;
        perCtxt->rxChCount   = rxChCount;

        for (uint32_t flowIdx = 0U; flowIdx < rxChCount; flowIdx++)
        {
            const uint32_t rxChId = startChId + flowIdx;
            rxInArgs.enetType = perCtxt->enetType;
            rxInArgs.instId   = perCtxt->instId;
            rxInArgs.notifyCb = EnetMp_rxIsrFxn;
            rxInArgs.cbArg    = perCtxt;

            EnetApp_getRxDmaHandle(rxChId,
                                    &rxInArgs,
                                    &rxChInfo);

            EnetAppUtils_assert(rxChInfo.useGlobalEvt == true);

            perCtxt->rxStartFlowIdx[flowIdx] = rxChInfo.rxFlowStartIdx;
            perCtxt->rxFlowIdx[flowIdx]      = rxChInfo.rxFlowIdx;
            perCtxt->hRxCh[flowIdx]          = rxChInfo.hRxCh;
            perCtxt->numValidMacAddress      += rxChInfo.numValidMacAddress;
            for (uint32_t macAddrIdx = 0; macAddrIdx < rxChInfo.numValidMacAddress; macAddrIdx++)
            {
                EnetUtils_copyMacAddr(&perCtxt->macAddr[perCtxt->numValidMacAddress - 1][0], &rxChInfo.macAddr[macAddrIdx][0]);
                EnetAppUtils_printMacAddr(&perCtxt->macAddr[perCtxt->numValidMacAddress - 1][0]);
            }

            if (perCtxt->hRxCh[flowIdx] == NULL)
            {
                EnetAppUtils_print("EnetMp_openRxCh() failed to open RX flow\r\n");
                status = ENET_EFAIL;
                EnetAppUtils_assert(perCtxt->hRxCh[flowIdx] != NULL);
            }
            /* Submit all ready RX buffers to DMA */
            EnetMp_initRxReadyPktQ(perCtxt->hRxCh[flowIdx]);
        }
    }

     return status;
}

static void EnetMp_closeDma(EnetMp_PerCtxt *perCtxt, uint32_t perCtxtIndex)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Close RX channel */
    for (uint32_t chIdx = 0; chIdx < perCtxt->rxChCount; chIdx++)
    {
        const uint32_t chId = perCtxt->rxChStartId + chIdx;
        EnetApp_closeRxDma(chId,
                           perCtxt->handleInfo.hEnet,
                           perCtxt->attachInfo.coreKey,
                           gEnetMp.coreId,
                           &fqPktInfoQ,
                           &cqPktInfoQ);
        EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
        EnetAppUtils_freePktInfoQ(&cqPktInfoQ);
    }

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Retrieve any pending TX packets from driver */
    EnetMp_retrieveFreeTxPkts(perCtxt);

    for (uint32_t chIdx = 0; chIdx < perCtxt->txChCount; chIdx++)
    {
        const uint32_t chId = perCtxt->txChStartId + chIdx;

        EnetApp_closeTxDma(chId,
                           perCtxt->handleInfo.hEnet,
                           perCtxt->attachInfo.coreKey,
                           gEnetMp.coreId,
                           &fqPktInfoQ,
                           &cqPktInfoQ);

        EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
        EnetAppUtils_freePktInfoQ(&cqPktInfoQ);
    }
    EnetAppUtils_freePktInfoQ(&gEnetMp.txFreePktInfoQ);
}

static void EnetMp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < (ENET_SYSCFG_TOTAL_NUM_TX_PKT/ENET_SYSCFG_TX_CHANNELS_NUM); i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetMp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gEnetMp.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gEnetMp.txFreePktInfoQ));
}

static void EnetMp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    int32_t status;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&rxFreeQ);

    for (uint32_t i= 0; i< ENET_SYSCFG_TOTAL_NUM_RX_PKT/ENET_SYSCFG_RX_FLOWS_NUM; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetMp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);

        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&rxFreeQ, &pPktInfo->node);
    }


    /* Retrieve any packets which are ready */
    EnetQueue_initQ(&rxReadyQ);
    status = EnetDma_retrieveRxPktQ(hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);

    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(hRxCh, &rxFreeQ);

    /* Assert here, as during init, the number of DMA descriptors should be equal to
     * the number of free Ethernet buffers available with app */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxFreeQ) == 0U);
}

static uint32_t EnetMp_retrieveFreeTxPkts(EnetMp_PerCtxt *perCtxt)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    uint32_t i, txFreeQCnt = 0U;
    int32_t status;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any packets that may be free now */
    for (i = 0; i < perCtxt->txChCount; i++)
    {
        status = EnetDma_retrieveTxPktQ(perCtxt->hTxCh[i], &txFreeQ);
        if (status == ENET_SOK)
        {
            txFreeQCnt = EnetQueue_getQCount(&txFreeQ);

            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
            while (NULL != pktInfo)
            {
                EnetDma_checkPktState(&pktInfo->pktState,
                                        ENET_PKTSTATE_MODULE_APP,
                                        ENET_PKTSTATE_APP_WITH_DRIVER,
                                        ENET_PKTSTATE_APP_WITH_FREEQ);

                EnetQueue_enq(&gEnetMp.txFreePktInfoQ, &pktInfo->node);
                pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
            }
        }
        else
        {
            EnetAppUtils_print("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n", status);
        }
    }

    return txFreeQCnt;
}

static void EnetMp_createRxTask(EnetMp_PerCtxt *perCtxt,
                                uint8_t *taskStack,
                                uint32_t taskStackSize)
{
    TaskP_Params taskParams;
    int32_t status;
    status = SemaphoreP_constructBinary(&perCtxt->rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructCounting(&perCtxt->rxDoneSemObj, 0, COUNTING_SEM_COUNT);
    DebugP_assert(SystemP_SUCCESS == status);

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 2U;
    taskParams.stack          = taskStack;
    taskParams.stackSize      = taskStackSize;
    taskParams.args           = (void*)perCtxt;
    taskParams.name           = "Rx Task";
    taskParams.taskMain       = &EnetMp_rxTask;

    status = TaskP_construct(&perCtxt->rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

static void EnetMp_destroyRxTask(EnetMp_PerCtxt *perCtxt)
{
    SemaphoreP_destruct(&perCtxt->rxSemObj);
    SemaphoreP_destruct(&perCtxt->rxDoneSemObj);
    TaskP_destruct(&perCtxt->rxTaskObj);
}

static int32_t set_priority_queue_mapping(EnetMp_PerCtxt *perCtx, uint8_t port_num, uint32_t *prioQueuetMap)
{
    int32_t retVal;
    int32_t i;
    EnetMacPort_SetEgressPriorityMapInArgs testPortPrioMap;
    Enet_IoctlPrms prms;
    EnetMp_PerCtxt *perCtxt = perCtx;

    testPortPrioMap.macPort = port_num;
    /* Enable queue configuration as per requirement */
    for (i = 0; i < ENET_PRI_NUM; i++)
    {
        testPortPrioMap.priorityMap.priorityMap[i] = prioQueuetMap[i];
    }

    ENET_IOCTL_SET_IN_ARGS(&prms, &testPortPrioMap);

    /* Configure Priority Remapping */
    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP, &prms, retVal);
    if (retVal != ENET_SOK)
    {
        EnetAppUtils_print("ERROR: IOCTL command for priority mapping for PORT 1\n\r");
        retVal = ENET_EFAIL;
    }

    return retVal;
}

static void EnetMp_rxTask(void *args)
{
    EnetMp_PerCtxt *perCtxt = (EnetMp_PerCtxt *)args;
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *rxPktInfo;
    EnetDma_Pkt *txPktInfo;
    EthFrame *rxFrame;
    EthFrame *txFrame;
    Enet_IoctlPrms prms;
    bool semStatus;
#if DEBUG
    uint32_t totalRxCnt = 0U;
#endif
    uint32_t flowIdx, prioMap[ENETMP_MAX_TX_CHANNEL_NUM];
    int32_t status = ENET_SOK;


    for(flowIdx = 0; flowIdx< ENETMP_MAX_TX_CHANNEL_NUM; flowIdx++)
    {
        prioMap[flowIdx] =  flowIdx;
    }

    status = set_priority_queue_mapping(perCtxt, ENET_MAC_PORT_1, prioMap);
    status = set_priority_queue_mapping(perCtxt, ENET_MAC_PORT_2, prioMap);

    status = EnetMp_waitForLinkUp(perCtxt);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s: Failed to wait for link up: %d\n", perCtxt->name, status);
    }

    /* Open DMA for peripheral/port */
    if (status == ENET_SOK)
    {
        uint32_t perCtxtIndex;

        EnetAppUtils_print("%s: Open DMA\r\n", perCtxt->name);

        status = EnetMp_mapPerCtxt2Idx(perCtxt, &perCtxtIndex);
        EnetAppUtils_assert(status == ENET_SOK);
        status = EnetMp_openDma(perCtxt, perCtxtIndex);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: failed to open DMA: %d\r\n", perCtxt->name, status);
        }
    }

    /* Add port MAC entry */
    if ((status == ENET_SOK) && (Enet_isIcssFamily(perCtxt->enetType)))
    {
        EnetAppUtils_print("%s: Set MAC addr: ", perCtxt->name);
        EnetAppUtils_printMacAddr(&perCtxt->macAddr[0U][0U]);//[flowIdx][MAC addr length]

        if (perCtxt->enetType == ENET_ICSSG_DUALMAC)
        {
            IcssgMacPort_SetMacAddressInArgs inArgs;

            memset(&inArgs, 0, sizeof(inArgs));
            inArgs.macPort = perCtxt->macPort[0U];
            EnetUtils_copyMacAddr(&inArgs.macAddr[0U], &perCtxt->macAddr[0U][0U]);//[flowIdx][MAC addr length]
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_MACPORT_IOCTL_SET_MACADDR, &prms, status);
        }
        else
        {
            Icssg_MacAddr addr; // FIXME Icssg_MacAddr type

            /* Set host port's MAC address */
            EnetUtils_copyMacAddr(&addr.macAddr[0U], &perCtxt->macAddr[0U][0U]);//[flowIdx][MAC addr length]
            ENET_IOCTL_SET_IN_ARGS(&prms, &addr);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_HOSTPORT_IOCTL_SET_MACADDR, &prms, status);
        }

        if (status != ENET_SOK)
        {
                EnetAppUtils_print("%s: Failed to set MAC address entry: %d\n", perCtxt->name, status);
        }
    }


    while ((ENET_SOK == status) && (gEnetMp.run))
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&perCtxt->rxSemObj, SystemP_WAIT_FOREVER);

        /* All peripherals have single hardware RX channel, so we only need to retrieve
         * packets from a single flow.  But ICSSG Switch has two hardware channels, so
         * we need to retrieve packets from two flows, one flow per channel */

            for (flowIdx = 0; flowIdx < perCtxt->rxChCount; flowIdx++)
            {
                EnetQueue_initQ(&rxReadyQ);
                EnetQueue_initQ(&rxFreeQ);
                EnetQueue_initQ(&txSubmitQ);

                /* Get the packets received so far */
                status = EnetDma_retrieveRxPktQ(perCtxt->hRxCh[flowIdx], &rxReadyQ);
                if (status != ENET_SOK)
                {
                    /* Should we bail out here? */
                    EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n", status);
                    continue;
                }
#if DEBUG
                EnetAppUtils_print("%s: Received %u packets\r\n", perCtxt->name, EnetQueue_getQCount(&rxReadyQ));
                totalRxCnt += EnetQueue_getQCount(&rxReadyQ);
#endif
                reqTs = 0U;
                count[0][flowIdx] += EnetQueue_getQCount(&rxReadyQ);

                /* Consume the received packets and send them back */
                rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
                while (rxPktInfo != NULL)
                {
                    rxFrame = (EthFrame *)rxPktInfo->sgList.list[0].bufPtr;
                    EnetDma_checkPktState(&rxPktInfo->pktState,
                                          ENET_PKTSTATE_MODULE_APP,
                                          ENET_PKTSTATE_APP_WITH_DRIVER,
                                          ENET_PKTSTATE_APP_WITH_READYQ);

                    /* Retrieve TX packets from driver and recycle them */
                    EnetMp_retrieveFreeTxPkts(perCtxt);

                    /* Dequeue one free TX Eth packet */
                    txPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetMp.txFreePktInfoQ);
                    if (txPktInfo != NULL)
                    {
                        /* Fill the TX Eth frame with test content */
                        txFrame = (EthFrame *)txPktInfo->sgList.list[0].bufPtr;
                        memcpy(txFrame->hdr.dstMac, rxFrame->hdr.srcMac, ENET_MAC_ADDR_LEN);
                        memcpy(txFrame->hdr.srcMac, &perCtxt->macAddr[0U], ENET_MAC_ADDR_LEN);
                        txFrame->hdr.etherType = rxFrame->hdr.etherType;

                        memcpy(&txFrame->payload[0U],
                               &rxFrame->payload[0U],
                               rxPktInfo->sgList.list[0].segmentFilledLen - sizeof(EthFrameHeader));

                        txPktInfo->sgList.list[0].segmentFilledLen = rxPktInfo->sgList.list[0].segmentFilledLen;
                        txPktInfo->sgList.numScatterSegments = 1;
                        txPktInfo->chkSumInfo = 0U;
                        txPktInfo->appPriv = &gEnetMp;

                        /* Set timestamp info in DMA packet.
                         * Packet timestamp currently enabled only for ICSSG. */
                        if (gEnetMp.enableTs &&
                            Enet_isIcssFamily(perCtxt->enetType))
                        {
                            /* Save the timestamp of received packet that we are about to send back,
                             * so we can calculate the RX-to-TX time diffence in TX timestamp callback */
                            perCtxt->rxTs[perCtxt->txTsSeqId % ENET_SYSCFG_TOTAL_NUM_RX_PKT] = rxPktInfo->tsInfo.rxPktTs;

                            txPktInfo->tsInfo.enableHostTxTs = true;
                            txPktInfo->tsInfo.txPktSeqId     = perCtxt->txTsSeqId++;
                            txPktInfo->tsInfo.txPktMsgType   = 0U; /* Don't care for ICSSG */
                            txPktInfo->tsInfo.txPktDomain    = 0U; /* Don't care for ICSSG */
                            reqTs++;
                        }
                        else
                        {
                            txPktInfo->tsInfo.enableHostTxTs = false;
                        }

                        EnetDma_checkPktState(&txPktInfo->pktState,
                                              ENET_PKTSTATE_MODULE_APP,
                                              ENET_PKTSTATE_APP_WITH_FREEQ,
                                              ENET_PKTSTATE_APP_WITH_DRIVER);

                        /* Enqueue the packet for later transmission */
                        EnetQueue_enq(&txSubmitQ, &txPktInfo->node);
                    }
                    else
                    {
                        EnetAppUtils_print("%s: Drop due to TX pkt not available\r\n", perCtxt->name);
                    }

                    EnetDma_checkPktState(&rxPktInfo->pktState,
                                          ENET_PKTSTATE_MODULE_APP,
                                          ENET_PKTSTATE_APP_WITH_READYQ,
                                          ENET_PKTSTATE_APP_WITH_FREEQ);

                    /* Release the received packet */
                    EnetQueue_enq(&rxFreeQ, &rxPktInfo->node);
                    rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
                }

                /* Transmit all enqueued packets */
#if ENET_SYSCFG_DUAL_MAC
                const uint32_t txChIdx = flowIdx;
#else
                const uint32_t txChIdx = flowIdx % ENET_SYSCFG_TX_CHANNELS_NUM;
#endif
                status = EnetDma_submitTxPktQ(perCtxt->hTxCh[txChIdx], &txSubmitQ);
                if (status != ENET_SOK)
                {
                    EnetAppUtils_print("%s: Failed to submit TX pkt queue: %d\r\n", perCtxt->name, status);
                }

                EnetAppUtils_validatePacketState(&rxFreeQ,
                                                 ENET_PKTSTATE_APP_WITH_FREEQ,
                                                 ENET_PKTSTATE_APP_WITH_DRIVER);

                /* Wait for TX timestamp */
                while (gEnetMp.run && (reqTs != 0U))
                {
                    Enet_MacPort macPort = perCtxt->macPort[0];

                    Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_TIMESTAMP_TX, &macPort, sizeof(macPort));
                    semStatus = SemaphoreP_pend(&perCtxt->txTsSemObj, 1U);
                    if (semStatus == SystemP_SUCCESS)
                    {
                        continue;
                    }
                }

                /* Submit now processed buffers */
                EnetDma_submitRxPktQ(perCtxt->hRxCh[flowIdx], &rxFreeQ);
                if (status != ENET_SOK)
                {
                    EnetAppUtils_print("%s: Failed to submit RX pkt queue: %d\r\n", perCtxt->name, status);
                }
            }

    }

#if DEBUG
    EnetAppUtils_print("%s: Received %u packets\r\n", perCtxt->name, totalRxCnt);
#endif


    SemaphoreP_post(&perCtxt->rxDoneSemObj);
    TaskP_exit();
}

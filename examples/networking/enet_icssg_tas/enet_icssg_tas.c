/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 * \file  enet_icssg_tas.c
 *
 * \brief This file contains the implementation of the Enet TAS example.
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
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Max number of ports supported per context */
#define ENETTAS_PORT_MAX                          (2U)

/* Max number of hardware RX channels. Note that this is different than Enet LLD's
 * RX channel concept which maps to UDMA hardware RX flows */
#define ENETTAS_HW_RXCH_MAX                       (2U)

/* Local flag to disable a peripheral from test list */
#define ENETTAS_DISABLED                          (0U)

/* Max number of ports that can be tested with this app */
#define ENETTAS_ICSSG_INSTANCE_MAX                (2U)
#define ENETTAS_PER_MAX                           (ENETTAS_ICSSG_INSTANCE_MAX)

/* Task stack size */
#define ENETTAS_TASK_STACK_SZ                     (10U * 1024U)

/* 100-ms periodic tick */
#define ENETTAS_PERIODIC_TICK_MS                  (100U)

/*Counting Semaphore count*/
#define COUNTING_SEM_COUNT                       (10U)

/* VLAN id for testing purposes */
#define ENETTAS_VLAN_ID                           (100U)

#define ENETTAS_LINK_NOT_UP                       (0U)

#define ENETTAS_LINK_UP                           (1U)

/* Bit#0 Indicates host ownership (indicates Host egress) */
#define FDB_ENTRY_HOST_BIT                        (0)

/* Bit#1 Indicates that MAC ID is connected to Physical Port 1 */
#define FDB_ENTRY_PORT1_BIT                       (1)

/* Bit#2 Indicates that MAC ID is connected to Physical Port 2 */
#define FDB_ENTRY_PORT2_BIT                       (2)

/* Bit#3 This is set to 1 for all learnt entries. 0 for static entries. */
#define FDB_ENTRY_LEARNT_ENTRY_BIT                (3)

/* Bit#4 If set for SA then packet is dropped (can be used to implement a blacklist).
 * If set for DA then packet is determined to be a special packet */
#define FDB_ENTRY_BLOCK_BIT                       (4)

/* Bit#5 If set for DA then the SA from the packet is not learnt */

/* Bit#6 if set, it means packet has been seen recently with source address +
 * FID matching MAC address/FID of entry. */
#define FDB_ENTRY_TOUCH_BIT                       (6)

/* Bit#7 set if entry is valid */
#define FDB_ENTRY_VALID_BIT                       (7)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/* Test parameters for each port in the multiport test */
typedef struct EnetTas_TestParams_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPort[ENETTAS_PORT_MAX];

    /* Number of MAC ports in macPorts array */
    uint32_t macPortNum;

    /* Name of this port to be used for logging */
    char *name;
} EnetTas_TestParams;

/* Context of a peripheral/port */
typedef struct EnetTas_PerCtxt_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPort[ENETTAS_PORT_MAX];

    /* Number of MAC ports in macPorts array */
    uint32_t macPortNum;

    /* Name of this port to be used for logging */
    char *name;

    /* ICSSG configuration */
    Icssg_Cfg icssgCfg;


    /* Number of valid MAC address entries present in macAddr variable below*/
    uint8_t numValidMacAddress;

    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

    /* UDMA driver configuration */
    EnetUdma_Cfg dmaCfg;

    /* TX channel number */
    uint32_t txChNum;

    /* TX channel handle */
    EnetDma_TxChHandle hTxCh;

    /* Start flow index */
    uint32_t rxStartFlowIdx[ENETTAS_HW_RXCH_MAX];

    /* Flow index */
    uint32_t rxFlowIdx[ENETTAS_HW_RXCH_MAX];

    /* RX channel handle */
    EnetDma_RxChHandle hRxCh[ENETTAS_HW_RXCH_MAX];

    /* Number of RX channels in hardware. This value is 1 for all peripherals,
     * except for ICSSG Switch where there are two UDMA RX channels */
    uint32_t numHwRxCh;

    /* RX task handle - receives packets, changes source/dest MAC addresses
     * and transmits the packets back */
    TaskP_Object perTaskObj;

    /* signals link up from the perTask to the test function */
    uint32_t perTaskLinkUp;

    /* Semaphore posted from RX callback when packets have arrived */
    SemaphoreP_Object rxSemObj;

    /* Semaphore used to synchronize all RX tasks exits */
    SemaphoreP_Object rxDoneSemObj;

    /* Semaphore posted from event callback upon asynchronous IOCTL completion */
    SemaphoreP_Object asyncIoctlSemObj;

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
} EnetTas_PerCtxt;

typedef struct EnetTas_Obj_s
{
    /* Flag which indicates if test shall run */
    volatile bool run;

    /* This core's id */
    uint32_t coreId;

    /* Queue of free TX packets */
    EnetDma_PktQ txFreePktInfoQ;

    /* Array of all peripheral/port contexts used in the test */
    EnetTas_PerCtxt perCtxt[ENETTAS_PER_MAX];

    /* Number of active contexts being used */
    uint32_t numPerCtxts;

    /* Whether promiscuous mode is enabled or not */
    bool promisc;

    /* Whether timestamp are enabled or not */
    bool enableTs;
} EnetTas_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetTas_mainTask(void *args);

static void EnetTas_init(void);

static int32_t EnetTas_open(EnetTas_PerCtxt *perCtxts,
                           uint32_t numPerCtxts);

static void EnetTas_close(EnetTas_PerCtxt *perCtxts,
                         uint32_t numPerCtxts);

static void EnetTas_printStats(EnetTas_PerCtxt *perCtxts,
                              uint32_t numPerCtxts);

static void EnetTas_resetStats(EnetTas_PerCtxt *perCtxts,
                              uint32_t numPerCtxts);

static int32_t EnetTas_waitForLinkUp(EnetTas_PerCtxt *perCtxt);

static void EnetTas_macMode2MacMii(emac_mode macMode,
                                  EnetMacPort_Interface *mii);

static int32_t EnetTas_openDma(EnetTas_PerCtxt *perCtxt, uint32_t perCtxtIndex);

static void EnetTas_closeDma(EnetTas_PerCtxt *perCtxt, uint32_t perCtxtIndex);

static void EnetTas_initTxFreePktQ(void);

static void EnetTas_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);

static uint32_t EnetTas_retrieveFreeTxPkts(EnetTas_PerCtxt *perCtxt);

static void EnetTas_createRxTask(EnetTas_PerCtxt *perCtxt,
                                uint8_t *taskStack,
                                uint32_t taskStackSize);

static void EnetTas_destroyRxTask(EnetTas_PerCtxt *perCtxt);

static void EnetTas_perTask(void *args);

extern uint32_t Board_getPhyAddr(void);

static void EnetTas_startTasTest(EnetTas_PerCtxt *perCtxts,
                                 uint32_t numPerCtxts);

static void EnetTas_tasPacketTx(EnetTas_PerCtxt *perCtxt,
                                uint8_t portNum,
                                uint8_t txTc);

static int32_t EnetTas_tasTest(EnetTas_PerCtxt *perCtxt,
                               Enet_MacPort macPort);

static int32_t EnetTas_setPriorityRegMapping(EnetTas_PerCtxt *perCtxt,
                                             Enet_MacPort macPort,
                                             uint32_t *prioRegenMap);

static int32_t EnetTas_setPriorityMapping(EnetTas_PerCtxt *perCtxt,
                                          Enet_MacPort macPort,
                                          uint32_t *prioMap);

static int32_t EnetTas_setupDefaultPortSettings(EnetTas_PerCtxt *perCtxt);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Enet multiport test object */
EnetTas_Obj gEnetTas;

/* Statistics */
IcssgStats_MacPort gEnetTas_icssgStats;
IcssgStats_Pa gEnetTas_icssgPaStats;

/* Enet EST Tx frame parameters */
uint8_t txDstMac[ENET_MAC_ADDR_LEN] = { 0x00, 0x02, 0x03, 0x04, 0x05, 0x06 };
uint16_t txEthType = 0x0800;
uint8_t ENET_testPkt1[] =
{
    0x45, 0x00, 0x00, 0x6a, 0x00, 0x54,
    0x00, 0x00, 0xff, 0xfd, 0x38, 0xea,
    0xc0, 0x55, 0x01, 0x02, 0xc0, 0x00,
    0x00, 0x01, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xab, 0x44, 0x50, 0x3e,
    0x03, 0x1c, 0x43, 0x59, 0xb0, 0xf3,
    0xd1, 0x49, 0x7d, 0x44, 0xa1, 0x33,
    0xe9, 0xa5, 0x33, 0xac, 0xb1, 0x31,
    0x49, 0x7b
};

volatile uint32_t txTsCbCount = 0;

/* Test application stack */
static uint8_t gEnetTasTaskStackRx[ENETTAS_PER_MAX][ENETTAS_TASK_STACK_SZ] __attribute__ ((aligned(32)));

// #define DUAL_MAC_MODE  /* TODO: Need to allocate TX channels as 2 in enet_cfg.h file */
/* Use this array to select the ports that will be used in the test */
static EnetTas_TestParams testParams[] =
{
#if defined(DUAL_MAC_MODE)
    { ENET_ICSSG_DUALMAC, 2U, { ENET_MAC_PORT_1 }, 1U, "icssg1-p1", },
    { ENET_ICSSG_DUALMAC, 3U, { ENET_MAC_PORT_1 }, 1U, "icssg1-p2", },
#else
    { ENET_ICSSG_SWITCH, 1U, { ENET_MAC_PORT_1, ENET_MAC_PORT_2 }, 2U, "icssg1", },
#endif
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void EnetTas_showMenu(void)
{
    EnetAppUtils_print("\nEnet Multiport Menu:\r\n");
    EnetAppUtils_print(" 'T'  -  Start the TAS test with default config\r\n");
    EnetAppUtils_print(" 's'  -  Print statistics\r\n");
    EnetAppUtils_print(" 'r'  -  Reset statistics\r\n");
    EnetAppUtils_print(" 'x'  -  Stop the test\r\n\n");
}

void EnetTas_mainTask(void *args)
{
    char option;
    uint32_t i, j;
    int32_t status;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("==========================\r\n");
    DebugP_log("   ENET ICSSG TAS TEST    \r\n");
    DebugP_log("==========================\r\n");

    /* Initialize test config */
    memset(&gEnetTas, 0, sizeof(gEnetTas));
    gEnetTas.run = true;
    gEnetTas.promisc = false;
    gEnetTas.enableTs = false;

    gEnetTas.numPerCtxts = ENET_ARRAYSIZE(testParams);

    for (i = 0U; i < gEnetTas.numPerCtxts; i++)
    {
        gEnetTas.perCtxt[i].enetType = testParams[i].enetType;
        gEnetTas.perCtxt[i].instId   = testParams[i].instId;
        gEnetTas.perCtxt[i].name     = testParams[i].name; /* shallow copy */

        gEnetTas.perCtxt[i].macPortNum = testParams[i].macPortNum;
        for (j = 0; j < gEnetTas.perCtxt[i].macPortNum; j++)
        {
            gEnetTas.perCtxt[i].macPort[j]  = testParams[i].macPort[j];
        }
    }

    /* Init driver */
    EnetTas_init();

    status = EnetTas_open(gEnetTas.perCtxt, gEnetTas.numPerCtxts);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to open peripherals: %d\r\n", status);
    }

    if (status == ENET_SOK)
    {
        /* Wait for user input to exit the test */
        EnetTas_showMenu();
        while (true)
        {
            option = ' ';
            status = DebugP_scanf("%c", &option);
            if (option == 'x')
            {
                EnetAppUtils_print("Stopping...\r\n");
                gEnetTas.run = false;
                break;
            }
            else if (option == 'T')
            {
                EnetTas_startTasTest(gEnetTas.perCtxt, gEnetTas.numPerCtxts);
            }
            else if (option == 's')
            {
                EnetTas_printStats(gEnetTas.perCtxt, gEnetTas.numPerCtxts);
            }
            else if (option == 'r')
            {
                EnetTas_resetStats(gEnetTas.perCtxt, gEnetTas.numPerCtxts);
            }
            else
            {
                EnetAppUtils_print("Invalid option, try again...\r\n");
                EnetTas_showMenu();
            }
            TaskP_yield();
        }

        /* Print statistics */
        EnetTas_printStats(gEnetTas.perCtxt, gEnetTas.numPerCtxts);

        /* Wait until RX tasks have exited */
        for (i = 0U; i < gEnetTas.numPerCtxts; i++)
        {
            EnetAppUtils_print("Waiting for RX task %u to exit\r\n", i+1);
            SemaphoreP_post(&gEnetTas.perCtxt[i].rxSemObj);
            SemaphoreP_pend(&gEnetTas.perCtxt[i].rxDoneSemObj, SystemP_WAIT_FOREVER);
        }

        EnetAppUtils_print("All RX tasks have exited\r\n");
    }

    /* Close all peripherals */
    EnetTas_close(gEnetTas.perCtxt, gEnetTas.numPerCtxts);

}

static void EnetTas_init(void)
{
    gEnetTas.coreId = EnetSoc_getCoreId();

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetTas.txFreePktInfoQ);
}

static void EnetTas_asyncIoctlCb(Enet_Event evt,
                                uint32_t evtNum,
                                void *evtCbArgs,
                                void *arg1,
                                void *arg2)
{
    EnetTas_PerCtxt *perCtxt = (EnetTas_PerCtxt *)evtCbArgs;

    EnetAppUtils_print("%s: Async IOCTL completed\r\n", perCtxt->name);
    SemaphoreP_post(&perCtxt->asyncIoctlSemObj);
}

static void EnetTas_txTsCb(Enet_Event evt,
                          uint32_t evtNum,
                          void *evtCbArgs,
                          void *arg1,
                          void *arg2)
{
    EnetTas_PerCtxt *perCtxt = (EnetTas_PerCtxt *)evtCbArgs;
    Icssg_TxTsEvtCbInfo *txTsInfo = (Icssg_TxTsEvtCbInfo *)arg1;
    uint32_t tsId = txTsInfo->txTsId;
    uint64_t txTs = txTsInfo->ts;

    /* Save current timestamp for future monotonicity checks */
     perCtxt->txTs[tsId % ENET_SYSCFG_TOTAL_NUM_TX_PKT] = txTs;

    /* increment the txTsCb counter which is checked by the EST test function */
    txTsCbCount++;
    SemaphoreP_post(&perCtxt->txTsSemObj);
}

static void EnetTas_portLinkStatusChangeCb(Enet_MacPort macPort,
                                          bool isLinkUp,
                                          void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

int32_t EnetTas_getPerIdx(Enet_Type enetType, uint32_t instId, uint32_t *perIdx)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Initialize async IOCTL and TX timestamp semaphores */
    for (i = 0U; i < gEnetTas.numPerCtxts; i++)
    {
        EnetTas_PerCtxt *perCtxt = &(gEnetTas.perCtxt[i]);
        if ((perCtxt->enetType == enetType) && (perCtxt->instId == instId))
        {
            break;
        }
    }
    if (i < gEnetTas.numPerCtxts)
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

int32_t EnetTas_mapPerCtxt2Idx(EnetTas_PerCtxt *perCtxt, uint32_t *perIdx)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    for (i = 0; i < gEnetTas.numPerCtxts;i++)
    {
        if (&gEnetTas.perCtxt[i] == perCtxt)
        {
            break;
        }
    }
    if (i < gEnetTas.numPerCtxts)
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

    resCfg = &icssgCfg->resCfg;

    /* We use software MAC address pool from apputils, but it will give same MAC address.
        * Add port index to make them unique */
    status = EnetTas_getPerIdx(enetType, instId, &perIdx);
    EnetAppUtils_assert(status == ENET_SOK);
    for (i = 0U; i < ENETTAS_PORT_MAX; i++)
    {
        resCfg->macList.macAddress[i][ENET_MAC_ADDR_LEN - 1] += (perIdx * ENETTAS_PORT_MAX) + i;
    }
    resCfg->macList.numMacAddress = ENETTAS_PORT_MAX;
}



static int32_t EnetTas_open(EnetTas_PerCtxt *perCtxts,
                           uint32_t numPerCtxts)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Initialize async IOCTL and TX timestamp semaphores */
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetTas_PerCtxt *perCtxt = &perCtxts[i];

        status = SemaphoreP_constructCounting(&perCtxt->asyncIoctlSemObj, 0, COUNTING_SEM_COUNT);
        DebugP_assert(SystemP_SUCCESS == status);

        status = SemaphoreP_constructBinary(&perCtxt->txTsSemObj, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    /* Do peripheral dependent initalization */
    EnetAppUtils_print("\nInit all peripheral clocks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetTas_PerCtxt *perCtxt = &perCtxts[i];
        EnetAppUtils_enableClocks(perCtxt->enetType, perCtxt->instId);
    }

    /* Prepare init configuration for all peripherals */
    EnetAppUtils_print("\nInit all configs\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");

    EnetApp_driverInit();

    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetTas_PerCtxt *perCtxt = &perCtxts[i];

        status = EnetApp_driverOpen(perCtxt->enetType, perCtxt->instId);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        }

        EnetApp_acquireHandleInfo(perCtxt->enetType, perCtxt->instId, &(perCtxt->handleInfo));
    }

    /* Open Enet driver for all peripherals */
    EnetAppUtils_print("\nOpen all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetTas_PerCtxt *perCtxt = &perCtxts[i];

        if (Enet_isIcssFamily(perCtxt->enetType))
        {
            EnetAppUtils_print("%s: Register async IOCTL callback\r\n", perCtxt->name);
            Enet_registerEventCb(perCtxt->handleInfo.hEnet,
                                 ENET_EVT_ASYNC_CMD_RESP,
                                 0U,
                                 EnetTas_asyncIoctlCb,
                                 (void *)perCtxt);

            EnetAppUtils_print("%s: Register TX timestamp callback\r\n", perCtxt->name);
            perCtxt->txTsSeqId = 0U;
            Enet_registerEventCb(perCtxt->handleInfo.hEnet,
                                 ENET_EVT_TIMESTAMP_TX,
                                 0U,
                                 EnetTas_txTsCb,
                                 (void *)perCtxt);
        }
    }

    /* Attach the core with RM */
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\nAttach core id %u on all peripherals\r\n", gEnetTas.coreId);
        EnetAppUtils_print("----------------------------------------------\r\n");
        for (i = 0U; i < numPerCtxts; i++)
        {
            EnetTas_PerCtxt *perCtxt = &perCtxts[i];

            EnetAppUtils_print("%s: Attach core\r\n", perCtxt->name);

            EnetApp_coreAttach(perCtxt->enetType, perCtxt->instId, gEnetTas.coreId, &perCtxt->attachInfo);
        }
    }

    /* Create RX tasks for each peripheral */
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\nCreate RX tasks\r\n");
        EnetAppUtils_print("----------------------------------------------\r\n");
        for (i = 0U; i < numPerCtxts; i++)
        {
            EnetTas_PerCtxt *perCtxt = &perCtxts[i];

            EnetAppUtils_print("%s: Create RX task\r\n", perCtxt->name);

            EnetTas_createRxTask(perCtxt, &gEnetTasTaskStackRx[i][0U], sizeof(gEnetTasTaskStackRx[i]));
        }
    }

    return status;
}

static void EnetTas_close(EnetTas_PerCtxt *perCtxts,
                         uint32_t numPerCtxts)
{
    uint32_t i;

    EnetAppUtils_print("\nClose DMA for all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetTas_PerCtxt *perCtxt = &perCtxts[i];

        EnetAppUtils_print("%s: Close DMA\r\n", perCtxt->name);

        EnetTas_closeDma(perCtxt, i);
    }

    /* Delete RX tasks created for all peripherals */
    EnetAppUtils_print("\nDelete RX tasks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetTas_destroyRxTask(&perCtxts[i]);
    }

    /* Detach core */
    EnetAppUtils_print("\nDetach core from all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetTas_PerCtxt *perCtxt = &perCtxts[i];

        EnetAppUtils_print("%s: Detach core\r\n", perCtxt->name);
        EnetApp_coreDetach(perCtxt->enetType, perCtxt->instId, gEnetTas.coreId, perCtxt->attachInfo.coreKey);
    }

    /* Close opened Enet drivers if any peripheral failed */
    EnetAppUtils_print("\nClose all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetTas_PerCtxt *perCtxt = &perCtxts[i];
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

    /* Do peripheral dependent initalization */
    EnetAppUtils_print("\nDeinit all peripheral clocks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetTas_PerCtxt *perCtxt = &perCtxts[i];
        EnetAppUtils_disableClocks(perCtxt->enetType, perCtxt->instId);
    }
    /* Destroy async IOCTL semaphore */
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetTas_PerCtxt *perCtxt = &perCtxts[i];

        SemaphoreP_destruct(&perCtxt->asyncIoctlSemObj);
    }
}

static void EnetTas_printStats(EnetTas_PerCtxt *perCtxts,
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
        EnetTas_PerCtxt *perCtxt = &gEnetTas.perCtxt[i];

        if (Enet_isIcssFamily(perCtxt->enetType))
        {
            EnetAppUtils_print("\n %s - PA statistics\r\n", perCtxt->name);
            EnetAppUtils_print("--------------------------------\r\n");
            ENET_IOCTL_SET_OUT_ARGS(&prms, &gEnetTas_icssgPaStats);
            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to get PA stats\r\n", perCtxt->name);
            }

            EnetAppUtils_printIcssgPaStats(&gEnetTas_icssgPaStats);
            EnetAppUtils_print("\r\n");
        }

        for (j = 0U; j < perCtxt->macPortNum; j++)
        {
            macPort = perCtxt->macPort[j];

            EnetAppUtils_print("\n %s - Port %u statistics\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
            EnetAppUtils_print("--------------------------------\r\n");

            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &gEnetTas_icssgStats);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to get port %u stats\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
                continue;
            }

            EnetAppUtils_printIcssgMacPortStats(&gEnetTas_icssgStats, false);

            EnetAppUtils_print("\r\n");
        }
    }
}

static void EnetTas_resetStats(EnetTas_PerCtxt *perCtxts,
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
        EnetTas_PerCtxt *perCtxt = &gEnetTas.perCtxt[i];

        EnetAppUtils_print("%s: Reset statistics\r\n", perCtxt->name);

        ENET_IOCTL_SET_NO_ARGS(&prms);
        ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ENET_STATS_IOCTL_RESET_HOSTPORT_STATS, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to reset  host port stats\r\n", perCtxt->name);
            continue;
        }

        for (j = 0U; j < perCtxt->macPortNum; j++)
        {
            macPort = perCtxt->macPort[j];

            ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ENET_STATS_IOCTL_RESET_MACPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to reset port %u stats\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
                continue;
            }
        }
    }
}

void EnetApp_initLinkArgs(Enet_Type enetType,
                          uint32_t instId,
                          EnetPer_PortLinkCfg *portLinkCfg,
                          Enet_MacPort macPort)
{
    EnetBoard_EthPort ethPort;
    const EnetBoard_PhyCfg *boardPhyCfg;
    IcssgMacPort_Cfg *icssgMacCfg;
    EnetMacPort_LinkCfg *linkCfg = &portLinkCfg->linkCfg;
    EnetMacPort_Interface *mii = &portLinkCfg->mii;
    EnetPhy_Cfg *phyCfg = &portLinkCfg->phyCfg;
    int32_t status = ENET_SOK;
    EnetTas_PerCtxt *perCtxt;
    uint32_t perIdx;

    status = EnetTas_getPerIdx(enetType, instId, &perIdx);
    EnetAppUtils_assert(status == ENET_SOK);

    perCtxt = &gEnetTas.perCtxt[perIdx];

    EnetAppUtils_print("%s: Open port %u\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));

    /* Setup board for requested Ethernet port */
    ethPort.enetType = perCtxt->enetType;
    ethPort.instId   = perCtxt->instId;
    ethPort.macPort  = macPort;
    ethPort.boardId  = ENETBOARD_AM64X_AM243X_EVM;
    EnetTas_macMode2MacMii(RGMII, &ethPort.mii);

    status = EnetBoard_setupPorts(&ethPort, 1U);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s: Failed to setup MAC port %u\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
        EnetAppUtils_assert(false);
    }

    icssgMacCfg = portLinkCfg->macCfg;
    IcssgMacPort_initCfg(icssgMacCfg);
    icssgMacCfg->specialFramePrio = 1U;

        /* Set port link params */
    portLinkCfg->macPort = macPort;

    mii->layerType     = ethPort.mii.layerType;
    mii->sublayerType  = ethPort.mii.sublayerType;
    mii->variantType   = ENET_MAC_VARIANT_FORCED;

    linkCfg->speed     = ENET_SPEED_AUTO;
    linkCfg->duplexity = ENET_DUPLEX_AUTO;

    boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
    if (boardPhyCfg != NULL)
    {
        EnetPhy_initCfg(phyCfg);
        if ((ENET_ICSSG_DUALMAC == perCtxt->enetType) && (2U == perCtxt->instId))
        {
            phyCfg->phyAddr     = CONFIG_ENET_ICSS0_PHY1_ADDR;
        }
        else if ((ENET_ICSSG_DUALMAC == perCtxt->enetType) && (3U == perCtxt->instId))
        {
            phyCfg->phyAddr     = CONFIG_ENET_ICSS0_PHY2_ADDR;
        }
        else if ((ENET_ICSSG_SWITCH == perCtxt->enetType) && (1U == perCtxt->instId))
        {
            if (macPort == ENET_MAC_PORT_1)
            {
                phyCfg->phyAddr     = CONFIG_ENET_ICSS0_PHY1_ADDR;
            }
            else
            {
                phyCfg->phyAddr     = CONFIG_ENET_ICSS0_PHY2_ADDR;
            }
        }
        else
        {
            EnetAppUtils_assert(false);
        }

        phyCfg->isStrapped  = boardPhyCfg->isStrapped;
        phyCfg->loopbackEn  = false;
        phyCfg->skipExtendedCfg = boardPhyCfg->skipExtendedCfg;
        phyCfg->extendedCfgSize = boardPhyCfg->extendedCfgSize;
        memcpy(phyCfg->extendedCfg, boardPhyCfg->extendedCfg, phyCfg->extendedCfgSize);
    }
    else
    {
        EnetAppUtils_print("%s: No PHY configuration found\r\n", perCtxt->name);
        EnetAppUtils_assert(false);
    }

}


static int32_t EnetTas_waitForLinkUp(EnetTas_PerCtxt *perCtxt)
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

        while (gEnetTas.run && !linked)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
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

        if (gEnetTas.run)
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

                ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ICSSG_PER_IOCTL_SET_PORT_STATE, &prms, status);
                if (status == ENET_SINPROGRESS)
                {
                    /* Wait for asyc ioctl to complete */
                    do
                    {
                        Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
                        status = SemaphoreP_pend(&perCtxt->asyncIoctlSemObj, SystemP_WAIT_FOREVER);
                    } while (gEnetTas.run && (status != SystemP_SUCCESS));

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

static void EnetTas_macMode2MacMii(emac_mode macMode,
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


static void EnetTas_rxIsrFxn(void *appData)
{
    EnetTas_PerCtxt *perCtxt = (EnetTas_PerCtxt *)appData;

    SemaphoreP_post(&perCtxt->rxSemObj);
}

#define ENETAPP_NUM_TX_CH_PER_PERCTXT     (ENET_SYSCFG_TX_CHANNELS_NUM/gEnetTas.numPerCtxts)
#define ENETAPP_NUM_RX_FLOW_PER_PERCTXT   (ENET_SYSCFG_RX_FLOWS_NUM/gEnetTas.numPerCtxts)

static int32_t EnetTas_openDma(EnetTas_PerCtxt *perCtxt, uint32_t perCtxtIndex)
{
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    uint32_t i, flowIdx;
    int32_t status = ENET_SOK;

    /* Open the TX channel */
    /* Open the TX channel */
    txInArgs.cbArg    = NULL;
    txInArgs.notifyCb = NULL;

    EnetApp_getTxDmaHandle((ENET_DMA_TX_CH0 + (perCtxtIndex * ENETAPP_NUM_TX_CH_PER_PERCTXT)),
                           &txInArgs,
                           &txChInfo);

    perCtxt->txChNum = txChInfo.txChNum;
    perCtxt->hTxCh   = txChInfo.hTxCh;
    EnetAppUtils_assert(txChInfo.useGlobalEvt == true);
    EnetAppUtils_assert(txChInfo.maxNumTxPkts >= (ENET_SYSCFG_TOTAL_NUM_TX_PKT/2U));

    if (perCtxt->hTxCh == NULL)
    {
#if FIX_RM
        /* Free the channel number if open Tx channel failed */
        EnetAppUtils_freeTxCh(perCtxt->handleInfo.hEnet,
                              perCtxt->attachInfo.coreKey,
                              gEnetTas.coreId,
                              gEnetTas.txChNum);
#endif
        EnetAppUtils_print("EnetTas_openDma() failed to open TX channel\r\n");
        status = ENET_EFAIL;
        EnetAppUtils_assert(perCtxt->hTxCh != NULL);
    }

    /* Allocate TX packets and keep them locally enqueued */
    if (status == ENET_SOK)
    {
        EnetTas_initTxFreePktQ();
    }

    /* Open the RX flow */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     rxInArgs;
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;
        perCtxt->numHwRxCh = ENET_SYSCFG_RX_CHANNELS_NUM;

        for (i = 0U; i < perCtxt->numHwRxCh; i++)
        {
            for (flowIdx = 0U; flowIdx < ENET_SYSCFG_RX_FLOWS_NUM; flowIdx++)
            {
                rxInArgs.notifyCb = EnetTas_rxIsrFxn;
                rxInArgs.cbArg   = perCtxt;

                EnetApp_getRxDmaHandle((ENET_DMA_RX_CH0 + i) + (perCtxtIndex * ENETAPP_NUM_RX_FLOW_PER_PERCTXT),
                                       &rxInArgs,
                                       &rxChInfo);

                perCtxt->rxStartFlowIdx[i]    = rxChInfo.rxFlowStartIdx;
                perCtxt->rxFlowIdx[i]         = rxChInfo.rxFlowIdx;
                perCtxt->hRxCh[i]             = rxChInfo.hRxCh;
                perCtxt->numValidMacAddress  += rxChInfo.numValidMacAddress;

                for (uint32_t macAddrIdx = 0; macAddrIdx < rxChInfo.numValidMacAddress; macAddrIdx++)
                {
                    EnetUtils_copyMacAddr(perCtxt->macAddr, &rxChInfo.macAddr[macAddrIdx][0]);
                    EnetAppUtils_printMacAddr(perCtxt->macAddr);
                }
                EnetAppUtils_assert(rxChInfo.useGlobalEvt == true);
                EnetAppUtils_assert(rxChInfo.sizeThreshEn == 0U);
                EnetAppUtils_assert(rxChInfo.maxNumRxPkts >= (ENET_SYSCFG_TOTAL_NUM_RX_PKT/2U));
                EnetAppUtils_assert(rxChInfo.chIdx == i);
                EnetAppUtils_assert(rxChInfo.useDefaultFlow == true);

                if (perCtxt->hRxCh[i] == NULL)
                {
                    EnetAppUtils_print("EnetTas_openRxCh() failed to open RX flow\r\n");
                    status = ENET_EFAIL;
                    EnetAppUtils_assert(perCtxt->hRxCh[i] != NULL);
                }
            }
        }
    }

    /* Submit all ready RX buffers to DMA */
    if (status == ENET_SOK)
    {
        for (i = 0U; i < perCtxt->numHwRxCh; i++)
        {
            EnetTas_initRxReadyPktQ(perCtxt->hRxCh[i]);
        }
    }

     return status;
}

static void EnetTas_closeDma(EnetTas_PerCtxt *perCtxt, uint32_t perCtxtIndex)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;
    uint32_t i;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Close RX channel */
    for (i = 0U; i < perCtxt->numHwRxCh; i++)
    {
        EnetApp_closeRxDma((ENET_DMA_RX_CH0 + i) + (perCtxtIndex * ENETAPP_NUM_RX_FLOW_PER_PERCTXT),
                           perCtxt->handleInfo.hEnet,
                           perCtxt->attachInfo.coreKey,
                           gEnetTas.coreId,
                           &fqPktInfoQ,
                           &cqPktInfoQ);
        EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
        EnetAppUtils_freePktInfoQ(&cqPktInfoQ);
    }

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Retrieve any pending TX packets from driver */
    EnetTas_retrieveFreeTxPkts(perCtxt);

    EnetApp_closeTxDma((ENET_DMA_TX_CH0 + (perCtxtIndex * ENETAPP_NUM_TX_CH_PER_PERCTXT)),
                       perCtxt->handleInfo.hEnet,
                       perCtxt->attachInfo.coreKey,
                       gEnetTas.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetTas.txFreePktInfoQ);
}

static void EnetTas_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < (ENET_SYSCFG_TOTAL_NUM_TX_PKT/2); i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetTas,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gEnetTas.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gEnetTas.txFreePktInfoQ));
}

static void EnetTas_initRxReadyPktQ(EnetDma_RxChHandle hRxCh)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    int32_t status;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&rxFreeQ);

    for (i = 0U; i < (ENET_SYSCFG_TOTAL_NUM_RX_PKT/2); i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetTas,
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

static uint32_t EnetTas_retrieveFreeTxPkts(EnetTas_PerCtxt *perCtxt)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    uint32_t txFreeQCnt = 0U;
    int32_t status;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any packets that may be free now */
    status = EnetDma_retrieveTxPktQ(perCtxt->hTxCh, &txFreeQ);
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

            EnetQueue_enq(&gEnetTas.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n", status);
    }

    return txFreeQCnt;
}

static void EnetTas_createRxTask(EnetTas_PerCtxt *perCtxt,
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
    taskParams.taskMain           = &EnetTas_perTask;

    status = TaskP_construct(&perCtxt->perTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

static void EnetTas_destroyRxTask(EnetTas_PerCtxt *perCtxt)
{
    SemaphoreP_destruct(&perCtxt->rxSemObj);
    SemaphoreP_destruct(&perCtxt->rxDoneSemObj);
    TaskP_destruct(&perCtxt->perTaskObj);
}

static void EnetTas_perTask(void *args)
{
    EnetTas_PerCtxt *perCtxt = (EnetTas_PerCtxt *)args;
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    perCtxt->perTaskLinkUp = ENETTAS_LINK_NOT_UP;

    status = EnetTas_waitForLinkUp(perCtxt);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s: Failed to wait for link up: %d\n", perCtxt->name, status);
    }

    /* Open DMA for peripheral/port */
    if (status == ENET_SOK)
    {
        uint32_t perCtxtIndex;

        EnetAppUtils_print("%s: Open DMA\r\n", perCtxt->name);

        status = EnetTas_mapPerCtxt2Idx(perCtxt, &perCtxtIndex);
        EnetAppUtils_assert(status == ENET_SOK);
        status = EnetTas_openDma(perCtxt, perCtxtIndex);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: failed to open DMA: %d\r\n", perCtxt->name, status);
        }
    }

    /* Add port MAC entry */
    if ((status == ENET_SOK) && (Enet_isIcssFamily(perCtxt->enetType)))
    {
        EnetAppUtils_print("%s: Set MAC addr: ", perCtxt->name);
        EnetAppUtils_printMacAddr(&perCtxt->macAddr[0U]);

        if (perCtxt->enetType == ENET_ICSSG_DUALMAC)
        {
            IcssgMacPort_SetMacAddressInArgs inArgs;

            memset(&inArgs, 0, sizeof(inArgs));
            inArgs.macPort = perCtxt->macPort[0U];
            EnetUtils_copyMacAddr(&inArgs.macAddr[0U], &perCtxt->macAddr[0U]);
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ICSSG_MACPORT_IOCTL_SET_MACADDR, &prms, status);
        }
        else
        {
            Icssg_MacAddr addr; // FIXME Icssg_MacAddr type

            /* Set host port's MAC address */
            EnetUtils_copyMacAddr(&addr.macAddr[0U], &perCtxt->macAddr[0U]);
            ENET_IOCTL_SET_IN_ARGS(&prms, &addr);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ICSSG_HOSTPORT_IOCTL_SET_MACADDR, &prms, status);
        }

        if (status != ENET_SOK)
        {
                EnetAppUtils_print("%s: Failed to set MAC address entry: %d\r\n", perCtxt->name, status);
        }
    }

    EnetAppUtils_print("%s: MAC port addr: ", perCtxt->name);
    EnetAppUtils_printMacAddr(&perCtxt->macAddr[0U]);

    /* Set the link up flag */
    if (status == ENET_SOK)
    {
        perCtxt->perTaskLinkUp = ENETTAS_LINK_UP;
    }

    while ((ENET_SOK == status) && (gEnetTas.run))
    {
        /* Wait here until the application is closed */
        ClockP_usleep(1000);
    }

    SemaphoreP_post(&perCtxt->rxDoneSemObj);
    TaskP_exit();
}

static void EnetTas_startTasTest(EnetTas_PerCtxt *perCtxts,
                                 uint32_t numPerCtxts)
{
    Enet_MacPort macPort;
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetTas_PerCtxt *perCtxt = &gEnetTas.perCtxt[i];

        if (perCtxt->perTaskLinkUp != ENETTAS_LINK_UP)
        {
            EnetAppUtils_print("%s: Skipping test due to link not UP\r\n", perCtxt->name);
            continue;
        }

        /* Setup the default port priority, VLAN and FDB parameters */
        EnetTas_setupDefaultPortSettings(perCtxt);

        for (j = 0U; j < perCtxt->macPortNum; j++)
        {
            macPort = perCtxt->macPort[j];
            EnetTas_tasTest(perCtxt, macPort);
        }
    }
}

static void EnetTas_tasPacketTx(EnetTas_PerCtxt *perCtxt,
                                uint8_t portNum,
                                uint8_t txTc)
{
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *txPktInfo;
    EthVlanFrame *txFrame;
    int32_t status;

    /* Retrieve TX packets from driver and recycle them */
    EnetTas_retrieveFreeTxPkts(perCtxt);
    EnetQueue_initQ(&txSubmitQ);

    /* Dequeue one free TX Eth packet */
    txPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetTas.txFreePktInfoQ);
    if (txPktInfo != NULL)
    {
        /* Fill the TX Eth frame with test content */
        txFrame = (EthVlanFrame *)txPktInfo->sgList.list[0].bufPtr;
        memcpy(txFrame->hdr.dstMac, &txDstMac[0U], ENET_MAC_ADDR_LEN);
        memcpy(txFrame->hdr.srcMac, &perCtxt->macAddr[0U], ENET_MAC_ADDR_LEN);

        txFrame->hdr.tpid = 0x8100;
        txFrame->hdr.tci  = 0x0064;
        txFrame->hdr.etherType = txEthType;

        memcpy(&txFrame->payload[0U],
               &ENET_testPkt1[0U],
               sizeof(ENET_testPkt1));

        txPktInfo->sgList.list[0].segmentFilledLen = 128U;
        txPktInfo->sgList.numScatterSegments = 1;
        txPktInfo->chkSumInfo = 0U;
        txPktInfo->appPriv = &gEnetTas;
        txPktInfo->tsInfo.txPktSeqId = perCtxt->txTsSeqId++;
        txPktInfo->txPktTc = txTc;
        txPktInfo->tsInfo.enableHostTxTs = true;

        if (perCtxt->enetType == ENET_ICSSG_SWITCH)
        {
            txPktInfo->txPortNum = perCtxt->macPort[portNum];
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

    status = EnetDma_submitTxPktQ(perCtxt->hTxCh, &txSubmitQ);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s: Failed to submit TX pkt queue: %d\r\n", perCtxt->name, status);
    }
}

static int32_t EnetTas_tasTest(EnetTas_PerCtxt *perCtxt,
                               Enet_MacPort macPort)
{
    int i, ts, start, end;
    Enet_IoctlPrms prms;
    EnetTas_SetAdminListInArgs adminListInArgs;
    EnetTas_SetStateInArgs setStateInArgs;
    EnetTas_OperStatus operStatus = ENET_TAS_OPER_LIST_NOT_YET_UPDATED;
    int32_t semStatus;
    int32_t status = ENET_SOK;

    /* set TAS state to reset */
    setStateInArgs.macPort = macPort;
    setStateInArgs.state = ENET_TAS_RESET;
    ENET_IOCTL_SET_IN_ARGS(&prms, &setStateInArgs);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ENET_TAS_IOCTL_SET_STATE, &prms, status);
    if (status == ENET_SINPROGRESS)
    {
        /* Wait for asyc ioctl to complete */
        do
        {
            Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
            semStatus = SemaphoreP_pend(&perCtxt->asyncIoctlSemObj, 1U);
        } while (gEnetTas.run && (semStatus != SystemP_SUCCESS));

        status = ENET_SOK;
    }
    else
    {
        EnetAppUtils_print("%s: Failed to set TAS state to Reset : %d\r\n", perCtxt->name, status);
    }

    /* setup TAS admin list */
    adminListInArgs.macPort = macPort;
    adminListInArgs.adminList.listLength = 4;
    adminListInArgs.adminList.cycleTime = 250000;
    adminListInArgs.adminList.baseTime = 0;

    for (i = 0; i < adminListInArgs.adminList.listLength; i++)
    {
        adminListInArgs.adminList.gateCmdList[i].gateStateMask = (0b11 << 2*i);
        adminListInArgs.adminList.gateCmdList[i].timeInterval  = adminListInArgs.adminList.cycleTime/adminListInArgs.adminList.listLength;
    }

    for (i = 0; i < ENET_TAS_MAX_NUM_QUEUES; i++)
    {
        adminListInArgs.adminList.sduTable.maxSDU[i] = 2048;
    }

    ENET_IOCTL_SET_IN_ARGS(&prms, &adminListInArgs);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ENET_TAS_IOCTL_SET_ADMIN_LIST, &prms, status);
    if (status == ENET_SINPROGRESS)
    {
        /* Wait for asyc ioctl to complete */
        do
        {
            Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
            semStatus = SemaphoreP_pend(&perCtxt->asyncIoctlSemObj, 1U);
        } while (gEnetTas.run && (semStatus != SystemP_SUCCESS));

        status = ENET_SOK;
    }
    else
    {
        EnetAppUtils_print("%s: Failed to update TAS operational list : %d\r\n", perCtxt->name, status);
    }

    /* wait until the operational list is updated */
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &operStatus);
    do
    {
        ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ENET_TAS_IOCTL_GET_OPER_LIST_STATUS, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to check TAS operational list update status : %d\r\n", perCtxt->name, status);
        }
    } while (operStatus == ENET_TAS_OPER_LIST_NOT_YET_UPDATED);


    /* set TAS state to enable */
    setStateInArgs.macPort = macPort;
    setStateInArgs.state = ENET_TAS_ENABLE;
    ENET_IOCTL_SET_IN_ARGS(&prms, &setStateInArgs);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ENET_TAS_IOCTL_SET_STATE, &prms, status);
    if (status == ENET_SINPROGRESS)
    {
        /* Wait for asyc ioctl to complete */
        do
        {
            Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
            semStatus = SemaphoreP_pend(&perCtxt->asyncIoctlSemObj, 1U);
        } while (gEnetTas.run && (semStatus != SystemP_SUCCESS));

        status = ENET_SOK;
    }
    else
    {
        EnetAppUtils_print("%s: Failed to set TAS state to Enable : %d\r\n", perCtxt->name, status);
    }

    if (status == ENET_SOK)
    {
        EnetAppUtils_print("Transmitting 16 packets\r\n");
        perCtxt->txTsSeqId = 0;
        txTsCbCount = 0;

        /* transmit 16 packets on macPort */
        for (i = 0; i < 16; i++)
        {
            EnetTas_tasPacketTx(perCtxt, ENET_MACPORT_NORM(macPort), (i%8));
        }

        /* Wait for TX timestamp */
        while (gEnetTas.run && (txTsCbCount < 16U))
        {
                Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_TIMESTAMP_TX, &macPort, sizeof(macPort));
            SemaphoreP_pend(&perCtxt->txTsSemObj, 1U);
        }

        /* verify that each of the txTs falls within the expected window */
        for (i = 0; i < 16; i++)
        {
            ts = perCtxt->txTs[i]%adminListInArgs.adminList.cycleTime;

            /* for ith packet with i Tc, window starts at i/2*cycleTime/4 and ends at (i/2+1)*cycleTime/4 */
            start = (i/2%4)*adminListInArgs.adminList.cycleTime/adminListInArgs.adminList.listLength;
            end = start + (adminListInArgs.adminList.cycleTime/adminListInArgs.adminList.listLength);

            if (!(ts >= start && ts <= end))
            {
                EnetAppUtils_print("Packet %d is out of window timestamp : %d window_start : %d window_end : %d\r\n", i, ts, start, end);
                status = ENET_EFAIL;
            }
            else
            {
                EnetAppUtils_print("Packet %d is within window timestamp : %d window_start : %d window_end : %d\r\n", i, ts, start, end);
            }
        }
    }

    if (status == ENET_SOK)
    {
        EnetAppUtils_print("Enet TAS Test PASSED\r\n");
    }
    else
    {
        EnetAppUtils_print("Enet TAS Test FAILED\r\n");
    }

    return status;
}

static int32_t EnetTas_setPriorityRegMapping(EnetTas_PerCtxt *perCtxt,
                                             Enet_MacPort macPort,
                                             uint32_t *prioRegenMap)
{
    EnetMacPort_SetPriorityRegenMapInArgs regenMap;
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;
    int32_t i;

    regenMap.macPort = macPort;

    for (i = 0; i < ENET_PRI_NUM; i++)
    {
        regenMap.priorityRegenMap.priorityMap[i] = prioRegenMap[i];
    }

    ENET_IOCTL_SET_IN_ARGS(&prms, &regenMap);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("ERROR: IOCTL command for priority regeneration for PORT = %u\r\n", macPort);
        status = ENET_EFAIL;
    }

    return status;
}

static int32_t EnetTas_setPriorityMapping(EnetTas_PerCtxt *perCtxt,
                                          Enet_MacPort macPort,
                                          uint32_t *prioMap)
{
    EnetMacPort_SetEgressPriorityMapInArgs priMap;
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;
    int32_t i;

    priMap.macPort = macPort;

    for (i = 0; i < ENET_PRI_NUM; i++)
    {
        priMap.priorityMap.priorityMap[i] = prioMap[i];
    }
    ENET_IOCTL_SET_IN_ARGS(&prms, &priMap);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("ERROR: IOCTL command for priority mapping for PORT = %u\r\n", macPort);
        status = ENET_EFAIL;
    }

    return status;
}

int32_t EnetTas_addDefaultHostVid(EnetTas_PerCtxt *perCtxt,
                                  uint8_t pcp,
                                  uint16_t vlan_id)
{
    Enet_IoctlPrms prms;
    EnetPort_VlanCfg vlanDefaultEntry;
    int32_t status = ENET_SOK;

    vlanDefaultEntry.portVID = vlan_id;
    vlanDefaultEntry.portPri = pcp;
    ENET_IOCTL_SET_IN_ARGS(&prms, &vlanDefaultEntry);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ICSSG_PER_IOCTL_VLAN_SET_HOSTPORT_DFLT_VID, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to set default VID");
    }

    return status;
}

int32_t EnetTas_addDefaultPortVid(EnetTas_PerCtxt *perCtxt,
                                  Enet_MacPort macPort,
                                  uint8_t pcp,
                                  uint16_t vlanId)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    Icssg_MacPortDfltVlanCfgInArgs vlanDefaultEntry;

    vlanDefaultEntry.macPort = macPort;
    vlanDefaultEntry.vlanCfg.portVID = vlanId;
    vlanDefaultEntry.vlanCfg.portPri = pcp;
    ENET_IOCTL_SET_IN_ARGS(&prms, &vlanDefaultEntry);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ICSSG_PER_IOCTL_VLAN_SET_MACPORT_DFLT_VID, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to set default VID");
    }

    return status;
}

int32_t EnetTas_addMacFdbEntry(EnetTas_PerCtxt *perCtxt,
                               Icssg_MacAddr mac,
                               int16_t vlanId,
                               uint8_t fdbEntryPort)
{
    int32_t status = ENET_EFAIL;
    int32_t semStatus;
    Enet_IoctlPrms prms;
    Icssg_FdbEntry fdbEntry;
    int i = 0;

    /* Now make an entry in FDB for the HOST MAC address using Asynchronous IOCTL */
    for (i = 0; i < ENET_MAC_ADDR_LEN; i++)
    {
        fdbEntry.macAddr[i] = mac.macAddr[i];
    }

    fdbEntry.vlanId = vlanId;
    fdbEntry.fdbEntry[0] = fdbEntryPort;
    fdbEntry.fdbEntry[1] = fdbEntryPort;

    ENET_IOCTL_SET_IN_ARGS(&prms, &fdbEntry);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ICSSG_FDB_IOCTL_ADD_ENTRY, &prms, status);
    if (status == ENET_SINPROGRESS)
    {
        EnetAppUtils_print("Success: IOCTL command sent for making MAC entry in FDB \r\n");

        /* Wait for asyc ioctl to complete */
        do
        {
            Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
            semStatus = SemaphoreP_pend(&perCtxt->asyncIoctlSemObj, 1U);
        } while (gEnetTas.run && (semStatus != SystemP_SUCCESS));

        status = ENET_SOK;
    }
    else
    {
        EnetAppUtils_print("ERROR: IOCTL command sent for making MAC entry in FDB failed as vlanId out of range \r\n");
        EnetAppUtils_print("ERROR: IOCTL command sent for making MAC entry in FDB \r\n");
    }

    return status;
}

static int32_t EnetTas_setupDefaultPortSettings(EnetTas_PerCtxt *perCtxt)
{
    int32_t status = ENET_SOK;
    uint16_t vlanId;
    Enet_IoctlPrms prms;
    uint32_t i;
    uint32_t prioRegenMap[ENET_PRI_NUM];
    uint32_t prioMap[ENET_PRI_NUM];
    uint8_t fdbEntryPort = 0x00U;
    Icssg_MacAddr mac, specialMac;
    Icssg_VlanFidEntry vlanEntry;
    Icssg_VlanFidParams vlanParams = {
        .fid         = ENETTAS_VLAN_ID,
        .hostMember  = 1,
        .p1Member    = 1,
        .p2Member    = 1,
        .hostTagged  = 0,
        .p1Tagged    = 0,
        .p2Tagged    = 0,
        .streamVid   = 0,
        .floodToHost = 0
    };
    Icssg_VlanFidParams vlanParamsForPrioTag = {
        .fid         = 0,
        .hostMember  = 1,
        .p1Member    = 1,
        .p2Member    = 1,
        .hostTagged  = 0,
        .p1Tagged    = 1,
        .p2Tagged    = 1,
        .streamVid   = 0,
        .floodToHost = 0
    };

    for (i = 0; i < ENET_PRI_NUM; i++)
    {
        prioRegenMap[i] = (uint32_t)(ENET_PRI_NUM-1-i);
        prioMap[i] = 0U;
    }

    /* Set IOCTL command to make PORT1 as boundary port and PORT2 as accept all type of packets */
    EnetTas_setPriorityRegMapping(perCtxt, ENET_MAC_PORT_1, prioRegenMap);
    EnetTas_setPriorityMapping(perCtxt, ENET_MAC_PORT_1, prioMap);

    if (perCtxt->enetType == ENET_ICSSG_SWITCH)
    {
        EnetTas_setPriorityRegMapping(perCtxt, ENET_MAC_PORT_2, prioRegenMap);
        EnetTas_setPriorityMapping(perCtxt, ENET_MAC_PORT_2, prioMap);
    }

    /*  Make an entry for common vlan id
     *  Update table for default entry */
    vlanEntry.vlanFidParams = vlanParams;
    vlanEntry.vlanId = (uint16_t)ENETTAS_VLAN_ID;
    ENET_IOCTL_SET_IN_ARGS(&prms, &vlanEntry);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ICSSG_PER_IOCTL_VLAN_SET_ENTRY, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("FID VLAN entry for HOST is FAILED = %u : FAILED\r\n", status);
    }

    /* -----------------Make an entry for Priority tag----------------- */

    vlanEntry.vlanFidParams = vlanParamsForPrioTag;
    vlanEntry.vlanId = (uint16_t)0;
    ENET_IOCTL_SET_IN_ARGS(&prms, &vlanEntry);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetTas.coreId, ICSSG_PER_IOCTL_VLAN_SET_ENTRY, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("FID VLAN entry for VID = 0: FAILED = %u : FAILED\r\n", status);
    }

    /* -----------------Make a default entry for Host port----------------- */
    status = EnetTas_addDefaultHostVid(perCtxt, 0, ENETTAS_VLAN_ID);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("\n ERROR: In updating default VLAN for Host : %d\r\n", status);
    }

    /* -----------------Make a default entry for P1 port----------------- */
    status = EnetTas_addDefaultPortVid(perCtxt, ENET_MAC_PORT_1, 1, ENETTAS_VLAN_ID);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("\n ERROR: In updating default VLAN for P1 \r\n\r");
    }

    if (perCtxt->enetType == ENET_ICSSG_SWITCH)
    {
        /* -----------------Make a default entry for P2 port----------------- */
        status = EnetTas_addDefaultPortVid(perCtxt, ENET_MAC_PORT_2, 2, ENETTAS_VLAN_ID);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("\n ERROR: In updating default VLAN for P2 \r\n\r");
        }
    }

    /* Change DA with MAC = 0x01 0x02 0x03 0x04 0x05 0x06 for loopback packet */
    mac.macAddr[0] = 0x01;
    mac.macAddr[1] = 0x02;
    mac.macAddr[2] = 0x03;
    mac.macAddr[3] = 0x04;
    mac.macAddr[4] = 0x05;
    mac.macAddr[5] = 0x06;

    fdbEntryPort = (fdbEntryPort |
                    (1 << FDB_ENTRY_VALID_BIT) |
                    (1 << FDB_ENTRY_HOST_BIT));

    status = EnetTas_addMacFdbEntry(perCtxt, mac, ENETTAS_VLAN_ID, fdbEntryPort);

    /* A multi cast address for packet Rx on port1 */
    specialMac.macAddr[0] = 0x01;
    specialMac.macAddr[1] = 0x11;
    specialMac.macAddr[2] = 0x22;
    specialMac.macAddr[3] = 0x33;
    specialMac.macAddr[4] = 0x44;
    specialMac.macAddr[5] = 0x55;

    vlanId = 100;
    fdbEntryPort = 0x00;
    fdbEntryPort = (fdbEntryPort |
                    (1 << FDB_ENTRY_VALID_BIT) |
                    (1 << FDB_ENTRY_BLOCK_BIT) |
                    (1 << FDB_ENTRY_PORT2_BIT) |
                    (1 << FDB_ENTRY_PORT1_BIT) |
                    (1 << FDB_ENTRY_HOST_BIT));

     status = EnetTas_addMacFdbEntry(perCtxt, specialMac, vlanId, fdbEntryPort);
     if (status != ENET_SOK)
     {
         EnetAppUtils_print("ERROR: adding MAC-FDB entry for port1\n\r");
         status = ENET_EFAIL;
     }

    return status;
}

/*
 *  Copyright (c) Texas Instruments Incorporated 2022-23
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
 * \file  txsg_test.c
 *
 * \brief This file contains the txsg test implementation.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "txsg_common.h"
#include "txsg_cfg.h"
#include "FreeRTOS.h"


#include <networking/enet/utils/include/enet_ethpatterns.h>
#include <networking/enet/core/include/core/enet_utils.h>
#include "enet_appprofile.h"
#include <drivers/soc.h>
#include "ti_enet_config.h"
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

uint32_t gENETTXSG_TEST_PKT_LEN   =                (ENETTXSG_TEST_PKT_LEN);

#define ENETTXSG_TEST_MAX_TX_SUBMIT_BATCH_SIZE     (4U)

uint32_t gENETTXSG_TEST_MAX_TX_SUBMIT_BATCH_SIZE   =  (ENETTXSG_TEST_MAX_TX_SUBMIT_BATCH_SIZE);

#define ENETTXSGAPP_DISABLE_EDMAMEMCPY_AT_RUNTIME  (1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetTxSG_createRxTxTasks(void);

static uint32_t EnetTxSG_retrieveFreeTxPkts(void);

static void EnetTxSG_txTask(void *args);

static uint32_t EnetTxSG_receivePkts(void);

static void EnetTxSG_rxTask(void *args);

void EnetApp_updateCpswInitCfg(Enet_Type enetType,  uint32_t instId, Cpsw_Cfg *cpswCfg);

static int32_t EnetTxSG_showAlivePhys(void);

static int32_t EnetTxSG_waitForLinkUp(void);

static void EnetTxSG_showCpswStats(void);

static int32_t EnetTxSG_macMode2PhyMii(emac_mode macMode,
                                       EnetPhy_Mii *mii);

static void EnetTxSG_macMode2MacMii(emac_mode macMode,
                                    EnetMacPort_Interface *mii);

static void EnetTxSG_rxIsrFxn(void *appData);

static void EnetTxSG_txIsrFxn(void *appData);

static void EnetTxSG_initTxFreePktQ(void);

static void EnetTxSG_initRxReadyPktQ(void);

static int32_t EnetTxSG_openDma(void);

static void EnetTxSG_closeDma(void);
#include <drivers/edma.h>

typedef struct EnetApp_edmaConfig_s
{
    EDMA_Handle         handle;
    uint16_t            paramId;
    uint8_t             transferCompletionCode;
    uint8_t             channelId;
    uint8_t             channelType;
} EnetApp_edmaConfig_t;

EnetApp_edmaConfig_t gEdmaCfg;
uint8_t bcastAddr[ENET_MAC_ADDR_LEN] = {0xffU, 0xffU, 0xffU, 0xffU, 0xffU, 0xffU};

static bool EnetApp_edmaInit(EnetApp_edmaConfig_t *cfg);
static bool EnetApp_edmaMemcpy(const EnetApp_edmaConfig_t *cfg, uint32_t destinationAddress);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Test application stack */
static uint8_t gEnetTxSGTaskStackTx[ENETTXSG_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gEnetTxSGTaskStackRx[ENETTXSG_TASK_STACK_SZ] __attribute__ ((aligned(32)));

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EnetTxSG_txsgTest(void)
{
    int32_t status;
    EnetApp_HandleInfo handleInfo;
    Enet_IoctlPrms prms;

    if (gEnetTxSG.enetType == ENET_CPSW_2G)
    {
        EnetAppUtils_print("CPSW_2G Test\r\n");
    }
    else if (gEnetTxSG.enetType == ENET_CPSW_3G)
    {
        EnetAppUtils_print("CPSW_3G Test\r\n");
    }
    EnetAppUtils_enableClocks(gEnetTxSG.enetType, gEnetTxSG.instId);

    /* Create TX/RX semaphores */
    status = SemaphoreP_constructBinary(&gEnetTxSG.rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gEnetTxSG.rxDoneSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gEnetTxSG.txSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gEnetTxSG.txDoneSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Local core id */
    gEnetTxSG.coreId = EnetSoc_getCoreId();

    EnetApp_driverInit();

    if (status == ENET_SOK)
    {
        status = EnetApp_driverOpen(gEnetTxSG.enetType, gEnetTxSG.instId);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open Enet driver: %d\r\n", status);
        }
    }

    EnetApp_acquireHandleInfo(gEnetTxSG.enetType, gEnetTxSG.instId, &handleInfo);
    gEnetTxSG.hEnet = handleInfo.hEnet;
    if (status == ENET_SOK)
    {
        EnetPer_AttachCoreOutArgs attachCoreOutArgs;

        EnetApp_coreAttach(gEnetTxSG.enetType, gEnetTxSG.instId, gEnetTxSG.coreId, &attachCoreOutArgs);
        gEnetTxSG.coreKey = attachCoreOutArgs.coreKey;
    }

    /* Open DMA driver */
    if (status == ENET_SOK)
    {
        status = EnetTxSG_openDma();
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open DMA: %d\r\n", status);
        }
    }

    if (status == ENET_SOK)
    {
        CpswAle_SetUcastEntryInArgs setUcastInArgs;
        uint32_t entryIdx;

        /* ALE entry with "secure" bit cleared is required for txsg */
        setUcastInArgs.addr.vlanId  = 0U;
        setUcastInArgs.info.portNum = CPSW_ALE_HOST_PORT_NUM;
        setUcastInArgs.info.blocked = false;
        setUcastInArgs.info.secure  = false;
        setUcastInArgs.info.super   = false;
        setUcastInArgs.info.ageable = false;
        setUcastInArgs.info.trunk   = false;
        EnetUtils_copyMacAddr(&setUcastInArgs.addr.addr[0U], gEnetTxSG.hostMacAddr);
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

        ENET_IOCTL(gEnetTxSG.hEnet, gEnetTxSG.coreId, CPSW_ALE_IOCTL_ADD_UCAST, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to add ucast entry: %d\r\n", status);
        }

    }

    /* Show alive PHYs */
    if (status == ENET_SOK)
    {
        status = EnetTxSG_showAlivePhys();
    }

    /* Wait for link up */
    if (status == ENET_SOK)
    {
        status = EnetTxSG_waitForLinkUp();
    }

    /* Do packet transmission and reception */
    if (status == ENET_SOK)
    {
        EnetTxSG_createRxTxTasks();

        SemaphoreP_pend(&gEnetTxSG.txDoneSemObj, SystemP_WAIT_FOREVER);
        SemaphoreP_pend(&gEnetTxSG.rxDoneSemObj, SystemP_WAIT_FOREVER);
    }

    /* Print network statistics */
    if (status == ENET_SOK)
    {
        if (Enet_isCpswFamily(gEnetTxSG.enetType))
        {
            EnetTxSG_showCpswStats();
        }
    }

    /* Print DMA statistics */
    if (status == ENET_SOK)
    {
        EnetAppUtils_showRxChStats(gEnetTxSG.hRxCh);
        EnetAppUtils_showTxChStats(gEnetTxSG.hTxCh);
    }

    /* Close Enet DMA driver */
    EnetTxSG_closeDma();

    /*Release Handle Info*/
    EnetApp_releaseHandleInfo(gEnetTxSG.enetType, gEnetTxSG.instId);
    gEnetTxSG.hEnet = NULL;

    EnetApp_driverDeInit();

    /* Disable peripheral clocks */
    EnetAppUtils_disableClocks(gEnetTxSG.enetType, gEnetTxSG.instId);

    /* Delete all TX/RX semaphores */
    SemaphoreP_destruct(&gEnetTxSG.rxSemObj);
    SemaphoreP_destruct(&gEnetTxSG.rxDoneSemObj);
    SemaphoreP_destruct(&gEnetTxSG.txSemObj);
    SemaphoreP_destruct(&gEnetTxSG.txDoneSemObj);

    EnetAppUtils_print("Test complete: %s\r\n", (status == ENET_SOK) ? "PASS" : "FAIL");

    return status;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */
static void EnetTxSG_getPacketCount(appProfilePktCount_t *pktCount)
{
    pktCount->rxIsrCount = gEnetTxSG.rxIsrCount;
    pktCount->txIsrCount = gEnetTxSG.txIsrCount;
    pktCount->rxPktCount = gEnetTxSG.totalRxCnt;
    pktCount->txPktCount = gEnetTxSG.totalTxCnt;
    pktCount->rxBytesCount = gEnetTxSG.totalRxBytesCnt;
    pktCount->txBytesCount = gEnetTxSG.totalTxBytesCnt;
}

static void EnetTxSG_createRxTxTasks(void)
{
    TaskP_Params taskParams;
    int32_t status;

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 3U;
    taskParams.stack          = gEnetTxSGTaskStackRx;
    taskParams.stackSize      = sizeof(gEnetTxSGTaskStackRx);
    taskParams.args           = (void*)&gEnetTxSG.rxSemObj;
    taskParams.name           = "Rx Task";
    taskParams.taskMain       = &EnetTxSG_rxTask;

    status = TaskP_construct(&gEnetTxSG.rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 2U;
    taskParams.stack          = gEnetTxSGTaskStackTx;
    taskParams.stackSize      = sizeof(gEnetTxSGTaskStackTx);
    taskParams.args           = (void*)&gEnetTxSG.txSemObj;
    taskParams.name           = "Tx Task";
    taskParams.taskMain       = &EnetTxSG_txTask;

    status = TaskP_construct(&gEnetTxSG.txTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
    {
        appProfileConfig appProfileCfg;

        appProfileCfg.getPacketCountFxn = &EnetTxSG_getPacketCount;
        status = EnetApp_initCpuLoadTask(&appProfileCfg, &gEnetTxSG.cpuLoadTask);
        DebugP_assert(SystemP_SUCCESS == status);
    }
}

static uint32_t EnetTxSG_retrieveFreeTxPkts(void)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t txFreeQCnt = 0U;

    EnetQueue_initQ(&txFreeQ);

    //app_fxnEntry(0);
    /* Retrieve any CPSW packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gEnetTxSG.hTxCh, &txFreeQ);
    //app_fxnExit(0);

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

            EnetQueue_enq(&gEnetTxSG.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n",
                           status);
    }

    return txFreeQCnt;
}

static void EnetTxSG_txTask(void *args)
{
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *pktInfo;
    uint32_t txRetrievePktCnt;
    uint32_t loopCnt, pktSendCnt ,pktRetreiveCnt;
    int32_t status = ENET_SOK;

    gEnetTxSG.totalTxCnt = 0U;
    gEnetTxSG.totalTxBytesCnt = 0U;
    for (loopCnt = 0U; loopCnt < ENETTXSG_NUM_ITERATION; loopCnt++)
    {
        pktSendCnt = 0U;
        pktRetreiveCnt = 0U;
        EnetQueue_initQ(&txSubmitQ);
        while (pktSendCnt < ENETTXSG_TEST_PKT_NUM)
        {
            /* Transmit a single packet */
            //DebugP_assert(EnetQueue_getQCount(&txSubmitQ) == 0);
            /* Dequeue one free TX Eth packet */
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetTxSG.txFreePktInfoQ);
            while (pktInfo == NULL)
            {
                txRetrievePktCnt = EnetTxSG_retrieveFreeTxPkts();
                if (txRetrievePktCnt == 0)
                {
                    SemaphoreP_pend(&gEnetTxSG.txSemObj, SystemP_WAIT_FOREVER);
                }
                else
                {
                    DebugP_assert(pktRetreiveCnt <= pktSendCnt);
                    pktRetreiveCnt += txRetrievePktCnt;
                    gEnetTxSG.totalTxCnt += txRetrievePktCnt;
                }
                pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetTxSG.txFreePktInfoQ);
            }

            while (NULL != pktInfo)
            {
                pktSendCnt++;

                #if (!defined (ENETTXSGAPP_DISABLE_EDMAMEMCPY_AT_RUNTIME))
                {
                    uint8_t *payLoadPtr;

                    /* Fill the TX Eth frame with test content */
                    DebugP_assert(pktInfo->sgList.numScatterSegments == 2);
                    payLoadPtr = pktInfo->sgList.list[1].bufPtr;
                    //app_fxnEntry(1);
                    DebugP_assert(EnetApp_edmaMemcpy(&gEdmaCfg, SOC_virtToPhy(payLoadPtr)) == true);
                    //app_fxnExit(1);
                }
                #endif
                EnetDma_checkPktState(&pktInfo->pktState,
                                      ENET_PKTSTATE_MODULE_APP,
                                      ENET_PKTSTATE_APP_WITH_FREEQ,
                                      ENET_PKTSTATE_APP_WITH_DRIVER);

                /* Enqueue the packet for later transmission */
                gEnetTxSG.totalTxBytesCnt += (pktInfo->sgList.list[0].segmentFilledLen +
                        pktInfo->sgList.list[1].segmentFilledLen);
                EnetQueue_enq(&txSubmitQ, &pktInfo->node);

                if (EnetQueue_getQCount(&txSubmitQ) >= gENETTXSG_TEST_MAX_TX_SUBMIT_BATCH_SIZE)
                {
                    do {
                        //app_fxnEntry(2);
                        status = EnetDma_submitTxPktQ(gEnetTxSG.hTxCh,
                                                           &txSubmitQ);
                        //app_fxnExit(2);
                        if (EnetQueue_getQCount(&txSubmitQ))
                        {
                            SemaphoreP_pend(&gEnetTxSG.txSemObj, SystemP_WAIT_FOREVER);
                        }
                    } while (status != 0);
                    DebugP_assert(EnetQueue_getQCount(&txSubmitQ) == 0);
                    if (pktSendCnt >= ENETTXSG_TEST_PKT_NUM)
                    {
                        break;
                    }
                }
                /* Dequeue one free TX Eth packet */
                pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetTxSG.txFreePktInfoQ);
            }
        }
        while (pktRetreiveCnt < pktSendCnt)
        {
            txRetrievePktCnt = EnetTxSG_retrieveFreeTxPkts();
            if (txRetrievePktCnt == 0)
            {
                SemaphoreP_pend(&gEnetTxSG.txSemObj, SystemP_WAIT_FOREVER);
            }
            else
            {
                DebugP_assert(pktRetreiveCnt <= pktSendCnt);
                pktRetreiveCnt += txRetrievePktCnt;
                gEnetTxSG.totalTxCnt += txRetrievePktCnt;
            }
        }
    }

    EnetAppUtils_print("Transmitted %d packets \r\n", gEnetTxSG.totalTxCnt);
    SemaphoreP_post(&gEnetTxSG.txDoneSemObj);

    EnetAppUtils_print("Delete EnetTxSG_txTask() and exit..\r\n");
    TaskP_destruct(&gEnetTxSG.txTaskObj);
    TaskP_exit();
}

static uint32_t EnetTxSG_receivePkts(void)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t rxReadyCnt = 0U;

    EnetQueue_initQ(&rxReadyQ);

    //app_fxnEntry(3);
    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(gEnetTxSG.hRxCh, &rxReadyQ);
    //app_fxnExit(3);
    if (status == ENET_SOK)
    {
        rxReadyCnt = EnetQueue_getQCount(&rxReadyQ);

        /* Queue the received packet to rxReadyQ and pass new ones from rxFreeQ
        **/
        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        while (pktInfo != NULL)
        {
            EnetDma_checkPktState(&pktInfo->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_DRIVER,
                                  ENET_PKTSTATE_APP_WITH_READYQ);

            EnetQueue_enq(&gEnetTxSG.rxReadyQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        }
    }
    else
    {
        EnetAppUtils_print("receivePkts() failed to retrieve pkts: %d\r\n",
                           status);
    }

    return rxReadyCnt;
}

static void EnetTxSG_rxTask(void *args)
{
    EnetDma_Pkt *pktInfo;
    EthFrame *frame;
    uint32_t rxReadyCnt;
    uint32_t loopCnt, loopRxPktCnt;
    int32_t status = ENET_SOK;

    gEnetTxSG.totalRxCnt      = 0U;
    gEnetTxSG.totalRxBytesCnt = 0U;

    for (loopCnt = 0U; loopCnt < ENETTXSG_NUM_ITERATION; loopCnt++)
    {
        loopRxPktCnt = 0U;
        /* Wait for packet reception */
        do
        {
            SemaphoreP_pend(&gEnetTxSG.rxSemObj, SystemP_WAIT_FOREVER);
            /* Get the packets received so far */
            rxReadyCnt = EnetTxSG_receivePkts();
            uint32_t EthPayloadLen = 0;
            if (rxReadyCnt > 0U)
            {
                /* Consume the received packets and release them */
                pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetTxSG.rxReadyQ);
                while (NULL != pktInfo)
                {
                    EnetDma_checkPktState(&pktInfo->pktState,
                                          ENET_PKTSTATE_MODULE_APP,
                                          ENET_PKTSTATE_APP_WITH_READYQ,
                                          ENET_PKTSTATE_APP_WITH_FREEQ);

                    /* Consume the packet by just printing its content */
                    EthPayloadLen = pktInfo->sgList.list[0].segmentFilledLen - sizeof(EthFrameHeader);
                    if (gEnetTxSG.printFrame)
                    {
                        frame = (EthFrame *)pktInfo->sgList.list[0].bufPtr;
                        EnetAppUtils_printFrame(frame, EthPayloadLen);
                    }

                    /* Release the received packet */
                    EnetQueue_enq(&gEnetTxSG.rxFreeQ, &pktInfo->node);
                    pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetTxSG.rxReadyQ);
                }

                /*Submit now processed buffers */
                if (status == ENET_SOK)
                {
                    EnetAppUtils_validatePacketState(&gEnetTxSG.rxFreeQ,
                                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                                     ENET_PKTSTATE_APP_WITH_DRIVER);
                    //app_fxnEntry(4);
                    EnetDma_submitRxPktQ(gEnetTxSG.hRxCh,
                                         &gEnetTxSG.rxFreeQ);
                    //app_fxnExit(4);
                }
                gEnetTxSG.totalRxCnt += rxReadyCnt;
                gEnetTxSG.totalRxBytesCnt += (EthPayloadLen + sizeof(EthFrameHeader));
            }
            loopRxPktCnt += rxReadyCnt;
        }
        while (loopRxPktCnt < ENETTXSG_TEST_PKT_NUM);
    }

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to transmit/receive packets: %d, transmitted: %d \r\n", ENETTXSG_TEST_PKT_NUM, gEnetTxSG.totalRxCnt);
    }
    else
    {
        EnetAppUtils_print("Received %d packets\r\n", gEnetTxSG.totalRxCnt);
    }

    SemaphoreP_post(&gEnetTxSG.rxDoneSemObj);

    EnetAppUtils_print("Delete EnetTxSG_rxTask() and exit..\r\n");
    TaskP_destruct(&gEnetTxSG.rxTaskObj);
    TaskP_exit();
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType,  uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    CpswHostPort_Cfg *hostPortCfg = &cpswCfg->hostPortCfg;
    CpswAle_Cfg *aleCfg = &cpswCfg->aleCfg;
    CpswCpts_Cfg *cptsCfg = &cpswCfg->cptsCfg;
    EnetCpdma_Cfg *dmaCfg = (EnetCpdma_Cfg *)cpswCfg->dmaCfg;

    dmaCfg->rxInterruptPerMSec = 8;
    dmaCfg->txInterruptPerMSec = 2;
    /* Peripheral config */
    cpswCfg->vlanCfg.vlanAware = false;

    /* Host port config */
    hostPortCfg->removeCrc      = true;
    hostPortCfg->padShortPacket = true;
    hostPortCfg->passCrcErrors  = true;

    /* ALE config */
    aleCfg->modeFlags                          = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn               = true;
    aleCfg->agingCfg.agingPeriodInMs           = 1000;
    aleCfg->nwSecCfg.vid0ModeEn                = true;
    aleCfg->vlanCfg.aleVlanAwareMode           = false;
    aleCfg->vlanCfg.cpswVlanAwareMode          = false;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask   = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask  = CPSW_ALE_ALL_PORTS_MASK;

    /* CPTS config */
    /* Note: Timestamping and MAC txsg are not supported together because of
     * IP limitation, so disabling timestamping for this application */
    cptsCfg->hostRxTsEn = false;

}

static int32_t EnetTxSG_showAlivePhys(void)
{
    Enet_IoctlPrms prms;
    bool alive = false;
    int32_t status;

    for (uint32_t phyAdd = 0U; phyAdd < ENET_MDIO_PHY_CNT_MAX; phyAdd++)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &phyAdd, &alive);

        ENET_IOCTL(gEnetTxSG.hEnet, gEnetTxSG.coreId, ENET_MDIO_IOCTL_IS_ALIVE, &prms, status);
        if (status == ENET_SOK)
        {
            if (alive == true)
            {
                EnetAppUtils_print("PHY %u is alive\r\n", phyAdd);
            }
        }
        else
        {
            EnetAppUtils_print("Failed to get PHY %u alive status: %d\r\n", phyAdd, status);
        }
    }

    return status;
}

static int32_t EnetTxSG_waitForLinkUp(void)
{
    Enet_IoctlPrms prms;
    bool linked = false;
    int32_t status = ENET_SOK;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &gEnetTxSG.macPort, &linked);

    while (!linked)
    {
        ENET_IOCTL(gEnetTxSG.hEnet, gEnetTxSG.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to get port %u's link status: %d\r\n",
                            ENET_MACPORT_ID(gEnetTxSG.macPort), status);
            linked = false;
            break;
        }

        if (!linked)
        {
            /* wait for 50 ms and poll again*/
            ClockP_usleep(50000);
        }
    }

    return status;
}

static void EnetTxSG_showCpswStats(void)
{
    Enet_IoctlPrms prms;
    CpswStats_PortStats portStats;
    int32_t status;

    /* Show host port statistics */
    ENET_IOCTL_SET_OUT_ARGS(&prms, &portStats);
    ENET_IOCTL(gEnetTxSG.hEnet, gEnetTxSG.coreId, ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\r\n Port 0 Statistics\r\n");
        EnetAppUtils_print("-----------------------------------------\r\n");
        EnetAppUtils_printHostPortStats2G((CpswStats_HostPort_2g *)&portStats);
        EnetAppUtils_print("\r\n");
    }
    else
    {
        EnetAppUtils_print("Failed to get host stats: %d\r\n", status);
    }

    /* Show MAC port statistics */
    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &gEnetTxSG.macPort, &portStats);
        ENET_IOCTL(gEnetTxSG.hEnet, gEnetTxSG.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("\r\n Port 1 Statistics\r\n");
            EnetAppUtils_print("-----------------------------------------\r\n");
            EnetAppUtils_printMacPortStats2G((CpswStats_MacPort_2g *)&portStats);
            EnetAppUtils_print("\r\n");
        }
        else
        {
            EnetAppUtils_print("Failed to get MAC stats: %d\r\n", status);
        }
    }
}

static int32_t EnetTxSG_macMode2PhyMii(emac_mode macMode,
                                    EnetPhy_Mii *mii)
{
    int32_t status = ENET_SOK;

    switch (macMode)
    {
        case RMII:
            *mii = ENETPHY_MAC_MII_RMII;
            break;

        case RGMII:
            *mii = ENETPHY_MAC_MII_RGMII;
            break;
        default:
            status = ENET_EFAIL;
            EnetAppUtils_print("Invalid MAC mode: %u\r\n", macMode);
            EnetAppUtils_assert(false);
            break;
    }

    return status;
}

static void EnetTxSG_macMode2MacMii(emac_mode macMode,
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

static void EnetTxSG_rxIsrFxn(void *appData)
{
    gEnetTxSG.rxIsrCount++;
    SemaphoreP_post(&gEnetTxSG.rxSemObj);
}

static void EnetTxSG_txIsrFxn(void *appData)
{
    gEnetTxSG.txIsrCount++;
    SemaphoreP_post(&gEnetTxSG.txSemObj);
}


#define ENET_LPBK_IP_ADDR_LEN    (4U)

typedef struct EnetTxSG_AddrInfo_s
{
    uint8_t dstMac[ENET_MAC_ADDR_LEN];
    uint8_t srcIP[ENET_LPBK_IP_ADDR_LEN];
    uint8_t dstIP[ENET_LPBK_IP_ADDR_LEN];
    uint16_t srcPortUDP;
    uint16_t dstPortUDP;
} EnetTxSG_AddrInfo;

const EnetTxSG_AddrInfo enetAppAddrInfo =
{
    //.dstMac = {0x08,0x00,0x27,0xAC,0xAF,0xFA},
    .dstMac = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
    .srcIP  = {0xC0,0xA8,0x00,0xC3}, /* 192.168.0.195 */
    .dstIP  = {0xC0,0xA8,0x00,0x88}, /* 192.168.0.136 */
    .srcPortUDP = 5001,
    .dstPortUDP = 5001,
};

#define ENET_LPBK_ETHERTYPE_IPV4 (0x0800)

void EnetApp_initEthFrameHdr(uint8_t *bufPtr, uint32_t *len)
{
    EthFrame *frame;

    frame = (EthFrame *)bufPtr;
    memcpy(frame->hdr.dstMac, enetAppAddrInfo.dstMac, ENET_MAC_ADDR_LEN);
    *len += ENET_MAC_ADDR_LEN;
    memcpy(frame->hdr.srcMac, &gEnetTxSG.hostMacAddr[0U], ENET_MAC_ADDR_LEN);
    *len += ENET_MAC_ADDR_LEN;
    frame->hdr.etherType = Enet_htons(ENET_LPBK_ETHERTYPE_IPV4);
    *len += sizeof(frame->hdr.etherType);
}

void EnetApp_initEthFrameHdrSrcMacAddr(uint8_t *bufPtr)
{
    EthFrame *frame;

    frame = (EthFrame *)bufPtr;
    memcpy(frame->hdr.srcMac, &gEnetTxSG.hostMacAddr[0U], ENET_MAC_ADDR_LEN);
    CacheP_wbInv(frame, sizeof(EthFrame), CacheP_TYPE_ALLD);
}

typedef struct
{
    uint8_t verIHL;
    uint8_t tos;
    uint16_t totalPktLen;
    uint16_t ipId;
    uint16_t flagFragOffset;
    uint8_t  ttl;
    uint8_t protocol;
    uint16_t hdrChksum;
    uint8_t  srcIP[ENET_LPBK_IP_ADDR_LEN];
    uint8_t  dstIP[ENET_LPBK_IP_ADDR_LEN];
} __attribute__ ((packed)) EthAppIPv4Header;


#define IPV4_HDR_VER_IHL        ((0x4 << 4) | 0x5)
#define IPV4_HDR_TOS            (0x00)
#define IPV4_HDR_TOTAL_PKT_LEN  (1498U)
#define IPV4_HDR_IPID           (0x28)
#define IPV4_HDR_FLAGFRAFOFFSET (0x0000)
#define IPV4_HDR_TTL            (0xFF)
#define IPV4_HDR_UDPLITE        (0x88)
#define IPV4_HDR_HDRCSUM        (0x32D8)

void EnetApp_initIPv4Hdr(uint8_t *bufPtr, uint32_t *len)
{
    EthAppIPv4Header *ipv4Hdr;

    ipv4Hdr = (EthAppIPv4Header *)bufPtr;
    ipv4Hdr->verIHL = IPV4_HDR_VER_IHL;
    ipv4Hdr->tos    = IPV4_HDR_TOS;
    ipv4Hdr->totalPktLen = Enet_htons(IPV4_HDR_TOTAL_PKT_LEN);
    ipv4Hdr->ipId = Enet_htons(IPV4_HDR_IPID);
    ipv4Hdr->flagFragOffset = Enet_htons(IPV4_HDR_FLAGFRAFOFFSET);
    ipv4Hdr->ttl = IPV4_HDR_TTL;
    ipv4Hdr->protocol = IPV4_HDR_UDPLITE;
    ipv4Hdr->hdrChksum = Enet_htons(IPV4_HDR_HDRCSUM);
    memcpy(&ipv4Hdr->srcIP,enetAppAddrInfo.srcIP,sizeof(ipv4Hdr->srcIP));
    memcpy(&ipv4Hdr->dstIP,enetAppAddrInfo.dstIP,sizeof(ipv4Hdr->dstIP));
    *len += sizeof(EthAppIPv4Header);
}

typedef struct
{
    uint16_t srcPort;
    uint16_t dstPort;
    uint16_t csumCoverage;
    uint16_t csum;
} __attribute__ ((packed)) EthAppUDPLiteHeader;


#define UDPLITE_HDR_CSUMCOVERAGE        (0x0008)
#define UDPLITE_HDR_CSUM                (0x4FFB)

#define ENETTXSGAPP_TEST_PAYLOAD_LEN (IPV4_HDR_TOTAL_PKT_LEN - (sizeof(EthAppIPv4Header) + sizeof(EthAppUDPLiteHeader)))

void EnetApp_initUDPLiteHdr(uint8_t *bufPtr, uint32_t *len)
{
    EthAppUDPLiteHeader *udpHdr;

    udpHdr = (EthAppUDPLiteHeader *)bufPtr;
    udpHdr->srcPort = Enet_htons(enetAppAddrInfo.srcPortUDP);
    udpHdr->dstPort = Enet_htons(enetAppAddrInfo.dstPortUDP);
    udpHdr->csumCoverage = Enet_htons(UDPLITE_HDR_CSUMCOVERAGE);
    udpHdr->csum = Enet_htons(UDPLITE_HDR_CSUM);
    *len += sizeof(EthAppUDPLiteHeader);
}

#define ENETTXSG_PKT_HDR_SIZE (ENET_UTILS_ALIGN((sizeof(EthAppUDPLiteHeader) + sizeof(EthAppIPv4Header) + sizeof(EthFrameHeader)),128))

static uint8_t gEnetTxSGPktHeader[ENETTXSG_PKT_HDR_SIZE] __attribute__ ((aligned(128)));

void EnetApp_initPktHdr(EnetDma_Pkt *pPktInfo)
{
    uint32_t len;

    len = 0;
    EnetApp_initEthFrameHdr(&gEnetTxSGPktHeader[len], &len);
    EnetApp_initIPv4Hdr(&gEnetTxSGPktHeader[len], &len);
    EnetApp_initUDPLiteHdr(&gEnetTxSGPktHeader[len], &len);
    pPktInfo->chkSumInfo = 0U;
    pPktInfo->sgList.list[0].segmentFilledLen = len;
    pPktInfo->appPriv    = &gEnetTxSG;
    pPktInfo->sgList.numScatterSegments = 2;
    pPktInfo->sgList.list[1].bufPtr = pPktInfo->sgList.list[0].bufPtr;
    pPktInfo->sgList.list[1].segmentFilledLen = IPV4_HDR_TOTAL_PKT_LEN - (sizeof(EthAppUDPLiteHeader) + sizeof(EthAppIPv4Header));
    pPktInfo->sgList.list[0].bufPtr = &gEnetTxSGPktHeader[0];
    pPktInfo->sgList.list[0].disableCacheOps = true;
    CacheP_wbInv(pPktInfo->sgList.list[0].bufPtr, len, CacheP_TYPE_ALLD);
    pPktInfo->sgList.list[1].disableCacheOps = true;
    CacheP_wbInv(pPktInfo->sgList.list[1].bufPtr, (IPV4_HDR_TOTAL_PKT_LEN - (sizeof(EthAppUDPLiteHeader) + sizeof(EthAppIPv4Header))), CacheP_TYPE_ALLD);
}

static void EnetTxSG_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetTxSG.txFreePktInfoQ);

    DebugP_assert(EnetApp_edmaInit(&gEdmaCfg) == true);
    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_TX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetTxSG,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetApp_initPktHdr(pPktInfo);

        #if (defined (ENETTXSGAPP_DISABLE_EDMAMEMCPY_AT_RUNTIME))
        {
            uint8_t *payLoadPtr;

            /* Fill the TX Eth frame with test content */
            DebugP_assert(pPktInfo->sgList.numScatterSegments == 2);
            payLoadPtr = pPktInfo->sgList.list[1].bufPtr;
            //app_fxnEntry(1);
            DebugP_assert(EnetApp_edmaMemcpy(&gEdmaCfg, SOC_virtToPhy(payLoadPtr)) == true);
        }
        #endif

        EnetQueue_enq(&gEnetTxSG.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gEnetTxSG.txFreePktInfoQ));
}

static void EnetTxSG_initRxReadyPktQ(void)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pPktInfo;
    int32_t status;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&gEnetTxSG.rxFreeQ);
    EnetQueue_initQ(&gEnetTxSG.rxReadyQ);
    EnetQueue_initQ(&rxReadyQ);

    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_RX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetTxSG,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetQueue_enq(&gEnetTxSG.rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(gEnetTxSG.hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);
    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&gEnetTxSG.rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(gEnetTxSG.hRxCh,
                         &gEnetTxSG.rxFreeQ);

    /* Assert here as during init no. of DMA descriptors should be equal to
     * no. of free Ethernet buffers available with app */

    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gEnetTxSG.rxFreeQ));
}

static int32_t EnetTxSG_openDma(void)
{
    int32_t status = ENET_SOK;

    /* Open the CPSW TX channel  */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     txInArgs;
        EnetApp_GetTxDmaHandleOutArgs  txChInfo;

        txInArgs.cbArg   = &gEnetTxSG;
        txInArgs.notifyCb = EnetTxSG_txIsrFxn;

        EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0,
                               &txInArgs,
                               &txChInfo);

        gEnetTxSG.txChNum = txChInfo.txChNum;
        gEnetTxSG.hTxCh   = txChInfo.hTxCh;

        gEnetTxSG.txIsrCount = 0;
        EnetTxSG_initTxFreePktQ();

        if (NULL != gEnetTxSG.hTxCh)
        {
            status = ENET_SOK;
            if (ENET_SOK != status)
            {
                EnetAppUtils_print("EnetUdma_startTxCh() failed: %d\r\n", status);
                status = ENET_EFAIL;
            }
        }
        else
        {
            EnetAppUtils_print("EnetDma_openTxCh() failed to open: %d\r\n",
                               status);
            status = ENET_EFAIL;
        }
    }

    /* Open the CPSW RX flow  */
    if (status == ENET_SOK)
    {
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;
        EnetApp_GetDmaHandleInArgs     rxInArgs;

        rxInArgs.notifyCb = EnetTxSG_rxIsrFxn;
        rxInArgs.cbArg   = &gEnetTxSG;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                              &rxInArgs,
                              &rxChInfo);
        gEnetTxSG.rxChNum = rxChInfo.rxChNum;
        gEnetTxSG.hRxCh  = rxChInfo.hRxCh;

        if (NULL == gEnetTxSG.hRxCh)
        {
            EnetAppUtils_print("EnetDma_openRxCh() failed to open: %d\r\n",
                               status);
            EnetAppUtils_assert(NULL != gEnetTxSG.hRxCh);
        }
        else
        {
            EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
            EnetUtils_copyMacAddr(gEnetTxSG.hostMacAddr, rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1]);
            EnetApp_initEthFrameHdrSrcMacAddr(&gEnetTxSGPktHeader[0]);
            EnetAppUtils_print("Host MAC address: ");
            EnetAppUtils_printMacAddr(gEnetTxSG.hostMacAddr);
            gEnetTxSG.rxIsrCount = 0;
            /* Submit all ready RX buffers to DMA.*/
            EnetTxSG_initRxReadyPktQ();
        }
    }

    return status;
}

static void EnetTxSG_closeDma(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);


    /* There should not be any ready packet */
    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gEnetTxSG.rxReadyQ));

    /* Close RX channel */
    EnetApp_closeRxDma(ENET_DMA_RX_CH0,
                       gEnetTxSG.hEnet,
                       gEnetTxSG.coreKey,
                       gEnetTxSG.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    EnetApp_closeTxDma(ENET_DMA_TX_CH0,
                       gEnetTxSG.hEnet,
                       gEnetTxSG.coreKey,
                       gEnetTxSG.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetTxSG.rxFreeQ);
    EnetAppUtils_freePktInfoQ(&gEnetTxSG.txFreePktInfoQ);

}



#define EDMA_UNTIED_CH0                     (63U)
#define EDMA_EVENT_QUEUE_ID_CH0             (0U)
#define TEST_PARAM_ID                       (0U)
#define TEST_TRANSFER_COMPLETION_CODE_CH0   (0U)
/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO      (0U)

static bool EnetApp_edmaInit(EnetApp_edmaConfig_t *cfg)
{
    uint32_t            baseAddr, regionId;
    int32_t             testStatus = SystemP_SUCCESS;
    EDMACCPaRAMEntry   edmaParam;
    uint32_t            dmaCh, tcc, param;
    uint32_t            configChannelStatus;

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaCh = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    param = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);


    /* Request channel */
    configChannelStatus =
        EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                                    dmaCh, tcc, param, EDMA_TEST_EVT_QUEUE_NO);
    DebugP_assert(configChannelStatus == true);
    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr = SOC_virtToPhy(&Enet_DataPattern1[0]);
    edmaParam.destAddr = SOC_virtToPhy(&Enet_DataPattern1[0]);
    edmaParam.aCnt = ENETTXSGAPP_TEST_PAYLOAD_LEN * sizeof(uint8_t);
    edmaParam.bCnt = 1;
    edmaParam.cCnt = 1;
    edmaParam.bCntReload = 0;
    edmaParam.srcBIdx    = (int16_t) EDMA_PARAM_BIDX(ENETTXSGAPP_TEST_PAYLOAD_LEN * sizeof(uint8_t));
    edmaParam.destBIdx   = (int16_t) EDMA_PARAM_BIDX(ENETTXSGAPP_TEST_PAYLOAD_LEN * sizeof(uint8_t));
    edmaParam.srcCIdx = ENETTXSGAPP_TEST_PAYLOAD_LEN * sizeof(uint8_t);
    edmaParam.destCIdx = ENETTXSGAPP_TEST_PAYLOAD_LEN * sizeof(uint8_t);
    edmaParam.linkAddr = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(ENETTXSGAPP_TEST_PAYLOAD_LEN * sizeof(uint8_t));
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(ENETTXSGAPP_TEST_PAYLOAD_LEN * sizeof(uint8_t));

    /* config.paramSetConfig.transferCompletionCode = TEST_TRANSFER_COMPLETION_CODE_CH0; */
    edmaParam.opt          |= (((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK;
    /* config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_A;*/
    edmaParam.opt          |= (EDMA_SYNC_A << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK;
    /* config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR; */
    edmaParam.opt          |= (EDMA_ADDRESSING_MODE_LINEAR << EDMA_TPCC_OPT_SAM_SHIFT) & EDMA_TPCC_OPT_SAM_MASK;
    /* config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR; */
    edmaParam.opt          |= (EDMA_ADDRESSING_MODE_LINEAR << EDMA_TPCC_OPT_DAM_SHIFT) & EDMA_TPCC_OPT_DAM_MASK;
    /* don't care because of linear addressing modes above */
    /* config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT; */
    edmaParam.opt          |= (EDMA_FIFO_WIDTH_8BIT << EDMA_TPCC_OPT_FWID_SHIFT) & EDMA_TPCC_OPT_FWID_MASK;
    /* config.paramSetConfig.isStaticSet = true; */
    edmaParam.opt          |= (1 << EDMA_OPT_STATIC_SHIFT) & EDMA_OPT_STATIC_MASK;
    /* config.paramSetConfig.isFinalTransferInterruptEnabled = true; */
    edmaParam.opt          |= (1 << EDMA_TPCC_OPT_TCINTEN_SHIFT) & EDMA_OPT_TCINTEN_MASK;


    EDMA_setPaRAM(baseAddr, param, &edmaParam);

    cfg->handle = gEdmaHandle[0];
    cfg->paramId = param;
    cfg->transferCompletionCode = tcc;
    cfg->channelId = dmaCh;
    cfg->channelType = EDMA_CHANNEL_TYPE_DMA;
    return(true);
}

static bool EnetApp_edmaMemcpy(const EnetApp_edmaConfig_t *cfg, uint32_t destinationAddress)
{
    uint32_t retVal;
    bool isTestPass = false;
    bool isTransferDone = false;
    uint32_t            baseAddr, regionId;
    EDMACCPaRAMEntry   edmaParam;

    baseAddr = EDMA_getBaseAddr(cfg->handle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    EDMA_getPaRAM(baseAddr, cfg->paramId, &edmaParam);
    edmaParam.destAddr = destinationAddress;
    EDMA_setPaRAM(baseAddr, cfg->paramId, &edmaParam);

    retVal = EDMA_enableTransferRegion(baseAddr, regionId, cfg->channelId, EDMA_TRIG_MODE_MANUAL);

    if (retVal == true)
    {
        isTransferDone = false;

        while(EDMA_readIntrStatusRegion(baseAddr, regionId, cfg->transferCompletionCode) != 1);

        EDMA_clrIntrRegion(baseAddr, regionId, cfg->transferCompletionCode);

        isTransferDone = true;
    }
    if ((retVal == true) && (isTransferDone == true))
    {
        isTestPass = true;
    }
    else
    {
        isTestPass = false;
    }
    return isTestPass;
}


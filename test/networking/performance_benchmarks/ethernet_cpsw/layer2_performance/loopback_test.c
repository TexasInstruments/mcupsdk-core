/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 * \file  loopback_test.c
 *
 * \brief This file contains the loopback test implementation.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "loopback_common.h"
#include "loopback_cfg.h"
#include "FreeRTOS.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENETAPP_CPTS_EVENTS_COUNTING_SEM_MAX_VAL (16U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_createRxTxTasks(void);

static uint32_t EnetApp_retrieveFreeTxPkts(void);

static int32_t EnetApp_waitForLinkUp(void);

static void EnetApp_showCpswStats(void);

static int32_t EnetApp_macMode2PhyMii(emac_mode macMode,
                                       EnetPhy_Mii *mii);

static void EnetApp_macMode2MacMii(emac_mode macMode,
                                    EnetMacPort_Interface *mii);

static void EnetApp_rxIsrFxn(void *appData);

static void EnetApp_txIsrFxn(void *appData);

static void EnetApp_initTxFreePktQ(void);

static void EnetApp_initRxReadyPktQ(void);

static int32_t EnetApp_openDma(void);

static void EnetApp_closeDma(void);


uint32_t EnetApp_getRxTimestamp(EnetLpbk_Obj *perCtxt,
                                EnetTimeSync_MsgType rxFrameType,
                                uint8_t rxPort,
                                uint16_t seqId);

uint32_t EnetApp_getTxTimestamp(EnetLpbk_Obj *perCtxt,
                                EnetTimeSync_MsgType txFrameType,
                                uint8_t txPort,
                                uint16_t seqId);

uint32_t EnetApp_getCurrentTime(EnetLpbk_Obj *perCtxt);

static void EnetApp_ptpPacketHandlerTask(void *args);

int32_t EnetApp_clearHostMacEntrySecureBit(EnetLpbk_Obj* perCtxt, uint8_t vlanId);

int32_t EnetApp_enableCptsTsEvents(EnetLpbk_Obj* perCtxt);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Test application stack */
static uint8_t gEnetLpbkTaskStackTx[ENETLPBK_TASK_STACK_SZ] __attribute__ ((aligned(32)));
extern EnetLpbk_Obj gEnetLpbk;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EnetApp_loopbackTest(void)
{
    int32_t status;
    EnetApp_HandleInfo handleInfo;

    if (gEnetLpbk.enetType == ENET_CPSW_2G)
    {
        EnetAppUtils_print("CPSW_2G Test\r\n");
    }
    if (gEnetLpbk.enetType == ENET_CPSW_3G)
    {
        EnetAppUtils_print("CPSW_3G Test\r\n");
    }

    EnetAppUtils_enableClocks(gEnetLpbk.enetType, gEnetLpbk.instId);

    status = SemaphoreP_constructCounting(&gEnetLpbk.cptsEvtSemObj, 0, ENETAPP_CPTS_EVENTS_COUNTING_SEM_MAX_VAL);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Create TX/RX semaphores */
    status = SemaphoreP_constructBinary(&gEnetLpbk.rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gEnetLpbk.rxDoneSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gEnetLpbk.txSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gEnetLpbk.txDoneSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Local core id */
    gEnetLpbk.coreId = EnetSoc_getCoreId();

    EnetApp_driverInit();

    if (status == ENET_SOK)
    {
        status = EnetApp_driverOpen(gEnetLpbk.enetType, gEnetLpbk.instId);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        }
    }
    gEnetLpbk.exitFlag     = false;
    gEnetLpbk.exitFlagDone = false;

    EnetApp_acquireHandleInfo(gEnetLpbk.enetType, gEnetLpbk.instId, &handleInfo);
    gEnetLpbk.hEnet = handleInfo.hEnet;

    /* Attach the core with RM */
    if (status == ENET_SOK)
    {
        EnetPer_AttachCoreOutArgs attachCoreOutArgs;

        EnetApp_coreAttach(gEnetLpbk.enetType, gEnetLpbk.instId, gEnetLpbk.coreId, &attachCoreOutArgs);
        gEnetLpbk.coreKey = attachCoreOutArgs.coreKey;
    }

    /* Open DMA driver */
    if (status == ENET_SOK)
    {
        status = EnetApp_openDma();
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open DMA: %d\r\n", status);
        }
    }

    /* ALE entry with "secure" bit cleared is required for loopback */
    if (status == ENET_SOK)
    {
        EnetApp_clearHostMacEntrySecureBit(&gEnetLpbk, 0 /* vlanId 0*/) ;
    }

    /* Enable CPTS Tx and TX Packet timestamping. Note: CPTS can timestamp only PTP packets, */
    if (status == ENET_SOK)
    {
        EnetApp_enableCptsTsEvents(&gEnetLpbk);
    }

    /* Wait for link up */
    if (status == ENET_SOK)
    {
        status = EnetApp_waitForLinkUp();
    }

    /* Do packet transmission and reception */
    if (status == ENET_SOK)
    {
        EnetApp_createRxTxTasks();

        /* wait for task completions*/
        SemaphoreP_pend(&gEnetLpbk.txDoneSemObj, SystemP_WAIT_FOREVER);
    }

    /* Print network statistics */
    if (status == ENET_SOK)
    {
        if (Enet_isCpswFamily(gEnetLpbk.enetType))
        {
            EnetApp_showCpswStats();
        }
    }

    /* Print DMA statistics */
    if (status == ENET_SOK)
    {
        EnetAppUtils_showRxChStats(gEnetLpbk.hRxCh);
        EnetAppUtils_showTxChStats(gEnetLpbk.hTxCh);
    }

    /* Close Enet DMA driver */
    EnetApp_closeDma();

    /*Detach Core*/
    EnetApp_coreDetach(gEnetLpbk.enetType, gEnetLpbk.instId, gEnetLpbk.coreId, gEnetLpbk.coreKey);

    /*Release Handle Info*/
    EnetApp_releaseHandleInfo(gEnetLpbk.enetType, gEnetLpbk.instId);
    gEnetLpbk.hEnet = NULL;

    /* Disable peripheral clocks */
    EnetAppUtils_disableClocks(gEnetLpbk.enetType, gEnetLpbk.instId);

    /* Delete all TX/RX semaphores */
    SemaphoreP_destruct(&gEnetLpbk.rxSemObj);
    SemaphoreP_destruct(&gEnetLpbk.rxDoneSemObj);
    SemaphoreP_destruct(&gEnetLpbk.txSemObj);
    SemaphoreP_destruct(&gEnetLpbk.txDoneSemObj);

    EnetAppUtils_print("Test complete: %s\r\n", (status == ENET_SOK) ? "PASS" : "FAIL");

    return status;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void EnetApp_createRxTxTasks(void)
{
    TaskP_Params taskParams;
    int32_t status;

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 2U;
    taskParams.stack          = gEnetLpbkTaskStackTx;
    taskParams.stackSize      = sizeof(gEnetLpbkTaskStackTx);
    taskParams.args           = (void*)&gEnetLpbk;
    taskParams.name           = "PTP Task";
    taskParams.taskMain       = &EnetApp_ptpPacketHandlerTask;

    status = TaskP_construct(&gEnetLpbk.txTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

static uint32_t EnetApp_retrieveFreeTxPkts(void)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t txFreeQCnt = 0U;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any CPSW packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gEnetLpbk.hTxCh, &txFreeQ);
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

            EnetQueue_enq(&gEnetLpbk.txFreePktInfoQ, &pktInfo->node);
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

uint8_t gPtpMsg[] =
{
        0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e,
        0xf4, 0x84, 0x4c, 0xfc, 0x34, 0xf0,
        0x88, 0xf7,
        0x12, 0x12,
        0x00, 0x36, 0x00, 0x00, 0x02, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0xf4, 0x84, 0x4c, 0xff, 0xfe, 0xfc, 0x34, 0xf0, 0x00, 0x00, 0x30, 0x42, 0x05, 0x00,
        0x00, 0x00, 0x00, 0x00, // payload : current CPTS Time
        0x00, 0x00, 0x00, 0x00, // payload : current PMU Time
};
uint8_t bcastAddr[ENET_MAC_ADDR_LEN] = {0xffU, 0xffU, 0xffU, 0xffU, 0xffU, 0xffU};


#define PTP_MSG_SEQ_FIELD 44U
#define PTP_MSG_SEQPTP_USER_LAYLOAD_CTIME_CPTS_FIELD 48U
#define PTP_MSG_SEQPTP_USER_LAYLOAD_CTIME_PMU_FIELD 52U


static void EnetApp_fillPacket(EthFrame *pFrame)
{
    memcpy(pFrame, gPtpMsg, sizeof(gPtpMsg));
    memcpy(pFrame->hdr.dstMac, bcastAddr, ENET_MAC_ADDR_LEN);
    memcpy(pFrame->hdr.srcMac, &gEnetLpbk.hostMacAddr[0U], ENET_MAC_ADDR_LEN);
}

static void EnetApp_ptpPacketHandlerTask(void *args)
{
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *pktInfo = NULL;
    EnetLpbk_Obj* pAppObj = (EnetLpbk_Obj*) args;
    uint32_t pktCnt = 0;

    gEnetLpbk.totalTxCnt = 0U;
    for (pktCnt = 0U; pktCnt < ENETLPBK_TEST_PKT_NUM; pktCnt++)
    {
        /* Transmit a single packet */
        EnetQueue_initQ(&txSubmitQ);

        /* Dequeue one free TX Eth packet */
        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&pAppObj->txFreePktInfoQ);

        if (NULL != pktInfo)
        {

            /*! 1. Send a PTP Packet */
            /* Fill the TX Eth frame with test content */
            uint8_t* pFrame = pktInfo->sgList.list[0].bufPtr;
            /*set the SeqId for Tx Frame as same as packet Count */
            uint16_t seqId = pktCnt&0xFF;
            EnetApp_fillPacket((EthFrame*)pFrame);
            pFrame[PTP_MSG_SEQ_FIELD]       = (seqId >> 8) & 0xFF;// 2 bytes length EnetApp_getCurrentTime
            pFrame[PTP_MSG_SEQ_FIELD + 1]   = (seqId >> 0) & 0xFF;// 2 bytes length EnetApp_getCurrentTime

            EnetAppUtils_assert(sizeof(gPtpMsg) < (ENETLPBK_TEST_PKT_LEN + sizeof(EthFrameHeader)));

            /* disable Scatter Gather for Tx packets, by setting numScatterSegments = 1 */
            pktInfo->sgList.list[0].segmentFilledLen = ENETLPBK_TEST_PKT_LEN + sizeof(EthFrameHeader);
            pktInfo->sgList.numScatterSegments = 1;

            pktInfo->chkSumInfo = 0U;
            pktInfo->appPriv    = &gEnetLpbk;
            EnetDma_checkPktState(&pktInfo->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_FREEQ,
                                  ENET_PKTSTATE_APP_WITH_DRIVER);


            uint32_t txSubmitTime_ns = EnetApp_getCurrentTime(&gEnetLpbk);
            uint32_t txSubmitTime_pmu = CycleCounterP_getCount32();
            EnetDma_submitTxPkt(pAppObj->hTxCh, pktInfo);

            /*! 2. Wait for packet transmit to Complete */
            SemaphoreP_pend(&pAppObj->txSemObj, SystemP_WAIT_FOREVER);

            /*! 3. Wait for packet reception (echoed packet by PHY) and handle the packet*/
            SemaphoreP_pend(&pAppObj->rxSemObj, SystemP_WAIT_FOREVER); // wait for Tx Completion

            /* Get the packets received so far */
            EnetDma_Pkt* pRxPktInfo = NULL;
            EnetDma_retrieveRxPkt(pAppObj->hRxCh, &pRxPktInfo);
            uint32_t rxRetriveTime_pmu  = CycleCounterP_getCount32();

            uint32_t rxRetriveTime_ns = ((rxRetriveTime_pmu - txSubmitTime_pmu)*5)/4 + txSubmitTime_ns; // estimate the time in ns from PMU. We are not calling EnetApp_getCurrentTime() as this API is slow

            EnetApp_retrieveFreeTxPkts();

            EnetDma_submitRxPkt(pAppObj->hRxCh, pRxPktInfo);

            /*! 4. Wait for Tx Timestamp evet of the packet (that is sent in (1) */
            SemaphoreP_pend(&pAppObj->cptsEvtSemObj, SystemP_WAIT_FOREVER);
            /* Get the Tx Timestamp of the packet */
            uint32_t pktWireEgressTime = EnetApp_getTxTimestamp(pAppObj, ENET_TIMESYNC_MESSAGE_PDELAY_REQ, 0, seqId);

            /*! 5. Wait for Rx Timestamp event of the packet (echoed packet) */
            SemaphoreP_pend(&pAppObj->cptsEvtSemObj, SystemP_WAIT_FOREVER);
            /* Get the Rx Timestamp of the packet */
            uint32_t pktWireIngressTime = EnetApp_getRxTimestamp(pAppObj, ENET_TIMESYNC_MESSAGE_PDELAY_REQ, 0, seqId);

            if ((pktWireEgressTime != 0) && (pktWireIngressTime != 0))
            {
                EnetAppUtils_print("------ Pkt Seq : %u -------\r\n", seqId);
                EnetAppUtils_print("Tx Latency: %6u ns\r\n", (pktWireEgressTime - txSubmitTime_ns));
                EnetAppUtils_print("Rx Latency: %6u ns\r\n", (rxRetriveTime_ns - pktWireIngressTime));
                EnetAppUtils_print("End2End Latency: %6u ns\r\n", (rxRetriveTime_ns - txSubmitTime_ns));
            }


        }
    }

    EnetAppUtils_print("---------------------------\r\n");
    EnetAppUtils_print("Profiled %d packets \r\n", pktCnt);
    SemaphoreP_post(&gEnetLpbk.txDoneSemObj);

    TaskP_destruct(&gEnetLpbk.txTaskObj);
    TaskP_exit();
}


void EnetApp_updateCpswInitCfg(Enet_Type enetType,  uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    CpswHostPort_Cfg *hostPortCfg = &cpswCfg->hostPortCfg;
    CpswAle_Cfg *aleCfg = &cpswCfg->aleCfg;
    CpswCpts_Cfg *cptsCfg = &cpswCfg->cptsCfg;

    /* Set Enet global runtime log level */
    Enet_setTraceLevel(ENET_TRACE_DEBUG);

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
    /* Note: Timestamping and MAC loopback are not supported together because of
     * IP limitation, so disabling timestamping for this application */
    cptsCfg->hostRxTsEn = false;

}

void EnetApp_initLinkArgs(Enet_Type enetType,
                          uint32_t instId,
                          EnetPer_PortLinkCfg *linkArgs,
                          Enet_MacPort macPort)
{
    EnetBoard_EthPort ethPort;
    CpswMacPort_Cfg *cpswMacCfg;
    EnetMacPort_LinkCfg *linkCfg = &linkArgs->linkCfg;
    EnetMacPort_Interface *mii = &linkArgs->mii;
    EnetPhy_Cfg *phyCfg = &linkArgs->phyCfg;
    int32_t status = ENET_SOK;
    EnetPhy_Mii phyMii;

    /* Setup board for requested Ethernet port */
    ethPort.enetType = gEnetLpbk.enetType;
    ethPort.instId   = gEnetLpbk.instId;
    ethPort.macPort  = gEnetLpbk.macPort;
    ethPort.boardId  = gEnetLpbk.boardId;
    EnetApp_macMode2MacMii(gEnetLpbk.macMode, &ethPort.mii);

    status = EnetBoard_setupPorts(&ethPort, 1U);
    EnetAppUtils_assert(status == ENET_SOK);

    /* Set port link params */
    linkArgs->macPort = macPort;

    cpswMacCfg = linkArgs->macCfg;

    EnetApp_macMode2MacMii(gEnetLpbk.macMode, mii);

    if (gEnetLpbk.testLoopBackType == LOOPBACK_TYPE_PHY)
    {
        const EnetBoard_PhyCfg *boardPhyCfg = NULL;

        /* Set PHY configuration params */
        EnetPhy_initCfg(phyCfg);
        status = EnetApp_macMode2PhyMii(gEnetLpbk.macMode, &phyMii);

        if (status == ENET_SOK)
        {
            boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
            if (boardPhyCfg != NULL)
            {
                phyCfg->phyAddr     = boardPhyCfg->phyAddr;
                phyCfg->isStrapped  = boardPhyCfg->isStrapped;
                phyCfg->skipExtendedCfg = boardPhyCfg->skipExtendedCfg;
                phyCfg->extendedCfgSize = boardPhyCfg->extendedCfgSize;
                memcpy(phyCfg->extendedCfg, boardPhyCfg->extendedCfg, phyCfg->extendedCfgSize);
            }
            else
            {
                EnetAppUtils_print("PHY info not found\r\n");
                EnetAppUtils_assert(false);
            }

            if ((phyMii == ENETPHY_MAC_MII_MII) ||
                (phyMii == ENETPHY_MAC_MII_RMII))
            {
                linkCfg->speed = ENET_SPEED_100MBIT;
            }
            else
            {
                /* Speed is always 100 Mbits on AM273x */
                linkCfg->speed = ENET_SPEED_100MBIT;
            }

            linkCfg->duplexity = ENET_DUPLEX_FULL;
        }
    }
    else
    {
        phyCfg->phyAddr = ENETPHY_INVALID_PHYADDR;

        if (mii->layerType == ENET_MAC_LAYER_MII)
        {
            linkCfg->speed = ENET_SPEED_100MBIT;
        }
        else
        {
            linkCfg->speed = ENET_SPEED_1GBIT;
        }

        linkCfg->duplexity = ENET_DUPLEX_FULL;
    }

    /* MAC and PHY loopbacks are mutually exclusive */
    phyCfg->loopbackEn = (gEnetLpbk.testLoopBackType == LOOPBACK_TYPE_PHY);
    cpswMacCfg->loopbackEn = (gEnetLpbk.testLoopBackType == LOOPBACK_TYPE_MAC);
}


static int32_t EnetApp_waitForLinkUp(void)
{
    Enet_IoctlPrms prms;
    bool linked = false;
    int32_t status = ENET_SOK;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &gEnetLpbk.macPort, &linked);

    while (!linked)
    {
        ENET_IOCTL(gEnetLpbk.hEnet, gEnetLpbk.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to get port %u's link status: %d\r\n",
                            ENET_MACPORT_ID(gEnetLpbk.macPort), status);
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

static void EnetApp_showCpswStats(void)
{
    Enet_IoctlPrms prms;
    CpswStats_PortStats portStats;
    int32_t status;

    /* Show host port statistics */
    ENET_IOCTL_SET_OUT_ARGS(&prms, &portStats);
    ENET_IOCTL(gEnetLpbk.hEnet, gEnetLpbk.coreId, ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
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
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &gEnetLpbk.macPort, &portStats);
        ENET_IOCTL(gEnetLpbk.hEnet, gEnetLpbk.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
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

static int32_t EnetApp_macMode2PhyMii(emac_mode macMode,
                                    EnetPhy_Mii *mii)
{
    int32_t status = ENET_SOK;

    switch (macMode)
    {
        case MII:
            *mii = ENETPHY_MAC_MII_MII;
            break;
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

static void EnetApp_macMode2MacMii(emac_mode macMode,
                                    EnetMacPort_Interface *mii)
{
    switch (macMode)
    {
        case MII:
            mii->layerType    = ENET_MAC_LAYER_MII;
            mii->sublayerType = ENET_MAC_SUBLAYER_STANDARD;
            mii->variantType  = ENET_MAC_VARIANT_NONE;
        break;
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

static void EnetApp_rxIsrFxn(void *appData)
{
    SemaphoreP_post(&gEnetLpbk.rxSemObj);
}

static void EnetApp_txIsrFxn(void *appData)
{
    SemaphoreP_post(&gEnetLpbk.txSemObj);
}

static void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] =
    {
       ENET_MEM_LARGE_POOL_PKT_SIZE,
    };

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetLpbk.txFreePktInfoQ);

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_TX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetLpbk,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gEnetLpbk.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gEnetLpbk.txFreePktInfoQ));
}

static void EnetApp_initRxReadyPktQ(void)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pPktInfo;
    int32_t status;
    uint32_t i;
    uint32_t scatterSegments[] =
    {
       ENET_MEM_LARGE_POOL_PKT_SIZE,
    };

    EnetQueue_initQ(&gEnetLpbk.rxFreeQ);
    EnetQueue_initQ(&gEnetLpbk.rxReadyQ);
    EnetQueue_initQ(&rxReadyQ);

    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_RX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetLpbk,
                                        ENETDMA_CACHELINE_ALIGNMENT,
                                        ENET_ARRAYSIZE(scatterSegments),
                                        scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetQueue_enq(&gEnetLpbk.rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(gEnetLpbk.hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);
    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&gEnetLpbk.rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(gEnetLpbk.hRxCh,
                         &gEnetLpbk.rxFreeQ);

    /* Assert here as during init no. of DMA descriptors should be equal to
     * no. of free Ethernet buffers available with app */

    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gEnetLpbk.rxFreeQ));
}

static int32_t EnetApp_openDma(void)
{
    int32_t status = ENET_SOK;

    /* Open the CPSW TX channel  */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     txInArgs;
        EnetApp_GetTxDmaHandleOutArgs  txChInfo; 

        txInArgs.cbArg   = &gEnetLpbk;
        txInArgs.notifyCb = EnetApp_txIsrFxn;

        EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0,
                               &txInArgs,
                               &txChInfo);

        gEnetLpbk.hTxCh   = txChInfo.hTxCh;


        EnetApp_initTxFreePktQ();

        if (NULL != gEnetLpbk.hTxCh)
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

        rxInArgs.notifyCb = EnetApp_rxIsrFxn;
        rxInArgs.cbArg   = &gEnetLpbk;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                              &rxInArgs,
                              &rxChInfo);
        gEnetLpbk.hRxCh  = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
        EnetUtils_copyMacAddr(gEnetLpbk.hostMacAddr, &rxChInfo.macAddr[rxChInfo.numValidMacAddress-1][0]);
        if (NULL == gEnetLpbk.hRxCh)
        {
            EnetAppUtils_print("EnetDma_openRxCh() failed to open: %d\r\n",
                               status);
            EnetAppUtils_assert(NULL != gEnetLpbk.hRxCh);
        }
        else
        {
            /* Submit all ready RX buffers to DMA.*/
            EnetApp_initRxReadyPktQ();
        }
    }

    return status;
}

static void EnetApp_closeDma(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* There should not be any ready packet */
    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gEnetLpbk.rxReadyQ));

    /* Close RX channel */
    EnetApp_closeRxDma(ENET_DMA_RX_CH0,
                       gEnetLpbk.hEnet,
                       gEnetLpbk.coreKey,
                       gEnetLpbk.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    EnetApp_closeTxDma(ENET_DMA_TX_CH0,
                       gEnetLpbk.hEnet,
                       gEnetLpbk.coreKey,
                       gEnetLpbk.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetLpbk.rxFreeQ);
    EnetAppUtils_freePktInfoQ(&gEnetLpbk.txFreePktInfoQ);

}


uint32_t EnetApp_getTxTimestamp(EnetLpbk_Obj *perCtxt,
                                EnetTimeSync_MsgType txFrameType,
                                uint8_t txPort,
                                uint16_t seqId)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    EnetTimeSync_GetEthTimestampInArgs inArgs;
    uint64_t tsVal = 0U;

    if (perCtxt != NULL)
    {
        inArgs.msgType = txFrameType;
        inArgs.seqId = seqId;
        inArgs.portNum = txPort;
        inArgs.domain  = 0U;

        if (status == ENET_SOK)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &tsVal);
            ENET_IOCTL(perCtxt->hEnet,
                                perCtxt->coreId,
                                ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP,
                                &prms,
                                status);

        }
    }
    else
    {
        status = ENET_EBADARGS;
    }

    return (tsVal & 0xFFFFFFFF);
}

uint32_t EnetApp_getRxTimestamp(EnetLpbk_Obj *perCtxt,
                                EnetTimeSync_MsgType rxFrameType,
                                uint8_t rxPort,
                                uint16_t seqId)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    EnetTimeSync_GetEthTimestampInArgs inArgs;
    uint64_t tsVal = 0U;

    if (perCtxt != NULL)
    {
        inArgs.msgType = rxFrameType;
        inArgs.seqId   = seqId;
        inArgs.portNum = rxPort;
        inArgs.domain  = 0U;

        if (status == ENET_SOK)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &tsVal);
            ENET_IOCTL(perCtxt->hEnet,
                        perCtxt->coreId,
                       ENET_TIMESYNC_IOCTL_GET_ETH_RX_TIMESTAMP,
                       &prms,
                       status);

        }
    }
    else
    {
        status = ENET_EBADARGS;
    }

    return (tsVal & 0xFFFFFFFF);
}

uint32_t EnetApp_getCurrentTime(EnetLpbk_Obj *perCtxt)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    uint64_t tsVal = 0U;

    if (perCtxt != NULL)
    {
        /* Software Time stamp Push event */
        ENET_IOCTL_SET_OUT_ARGS(&prms, &tsVal);
        ENET_IOCTL(perCtxt->hEnet,
                    perCtxt->coreId,
                   ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP,
                   &prms,
                   status);
        EnetAppUtils_assert(status == ENET_SOK);

    }
    return (tsVal&0xFFFFFFFF);
}



void EnetApp_fillTsEventPrms(CpswMacPort_TsEventCfg *tsPortEventCfg)
{
    tsPortEventCfg->commonPortIpCfg.ttlNonzeroEn = true;
    tsPortEventCfg->commonPortIpCfg.tsIp107En = false;
    tsPortEventCfg->commonPortIpCfg.tsIp129En = false;
    tsPortEventCfg->commonPortIpCfg.tsIp130En = false;
    tsPortEventCfg->commonPortIpCfg.tsIp131En = false;
    tsPortEventCfg->commonPortIpCfg.tsIp132En = false;
    tsPortEventCfg->commonPortIpCfg.tsPort319En = true;
    tsPortEventCfg->commonPortIpCfg.tsPort320En = true;
    tsPortEventCfg->commonPortIpCfg.unicastEn = false;
    tsPortEventCfg->domainOffset = 4U;
    tsPortEventCfg->ltype2En = false;
    tsPortEventCfg->rxAnnexDEn = true;
    tsPortEventCfg->rxAnnexEEn = true;
    tsPortEventCfg->rxAnnexFEn = true;
    tsPortEventCfg->txAnnexDEn = true;
    tsPortEventCfg->txAnnexEEn = true;
    tsPortEventCfg->txAnnexFEn = true;
    tsPortEventCfg->txHostTsEn = true;
    tsPortEventCfg->mcastType = 0U;
    tsPortEventCfg->messageType = 0xFFFFU;
    tsPortEventCfg->seqIdOffset = 30U;
    /* VLAN untagged */
    tsPortEventCfg->rxVlanType = ENET_MACPORT_VLAN_TYPE_SINGLE_TAG;
    tsPortEventCfg->txVlanType = ENET_MACPORT_VLAN_TYPE_SINGLE_TAG;
    tsPortEventCfg->vlanLType1 = 0U;
    tsPortEventCfg->vlanLType2 = 0U;

}

void EnetApp_cptsEvtNotifyFxn(void *pAppData, CpswCpts_Event *pEventInfo)
{
    EnetLpbk_Obj *pObj = (EnetLpbk_Obj*) pAppData;
    SemaphoreP_post(&pObj->cptsEvtSemObj);
}


int32_t EnetApp_enableCptsTsEvents(EnetLpbk_Obj* perCtxt)
{
    int32_t status;
    CpswMacPort_EnableTsEventInArgs enableTsEventInArgs;
    CpswCpts_RegisterStackInArgs enableTxEvtInArgs;
    Enet_IoctlPrms prms;

    EnetApp_fillTsEventPrms(&enableTsEventInArgs.tsEventCfg);

    enableTsEventInArgs.macPort = perCtxt->macPort;
    ENET_IOCTL_SET_IN_ARGS(&prms, &enableTsEventInArgs);
    ENET_IOCTL(perCtxt->hEnet,
               perCtxt->coreId,
               CPSW_MACPORT_IOCTL_ENABLE_CPTS_EVENT,
               &prms,
               status);
    if (status == ENET_SOK)
    {
        enableTxEvtInArgs.eventNotifyCb = &EnetApp_cptsEvtNotifyFxn;
        enableTxEvtInArgs.eventNotifyCbArg = perCtxt;
        ENET_IOCTL_SET_IN_ARGS(&prms, &enableTxEvtInArgs);

        ENET_IOCTL(perCtxt->hEnet,
                    perCtxt->coreId,
                   CPSW_CPTS_IOCTL_REGISTER_STACK,
                   &prms,
                   status);
    }
    return status;

}

int32_t EnetApp_clearHostMacEntrySecureBit(EnetLpbk_Obj* perCtxt, uint8_t vlanId)
{
    Enet_IoctlPrms prms;
    CpswAle_SetUcastEntryInArgs setUcastInArgs;
    uint32_t entryIdx = 0U;
    int32_t status = ENET_EFAIL;

    setUcastInArgs.addr.vlanId  = vlanId;
    setUcastInArgs.info.portNum = CPSW_ALE_HOST_PORT_NUM;
    setUcastInArgs.info.blocked = false;
    setUcastInArgs.info.secure  = false;
    setUcastInArgs.info.super   = false;
    setUcastInArgs.info.ageable = false;
    setUcastInArgs.info.trunk   = false;

    EnetUtils_copyMacAddr(&setUcastInArgs.addr.addr[0U], perCtxt->hostMacAddr);
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

    ENET_IOCTL(perCtxt->hEnet, perCtxt->coreId, CPSW_ALE_IOCTL_ADD_UCAST,
            &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to add ucast entry: %d\r\n", status);
    }
    return status;
}

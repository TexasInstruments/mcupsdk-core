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
 * \file  l2_networking.c
 *
 * \brief This file contains all functions related to layer 2 networking
 *        that is used by the CLI application
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_common.h"
#include "l2_networking.h"
#include "gptp_stack.h"
#include "cli_lwip.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENETAPP_VLAN_PCP_OFFSET                      (13U)
#define ENETAPP_VLAN_PCP_MASK                        (0x7U)
#define ENETAPP_VLAN_DEI_OFFSET                      (12U)
#define ENETAPP_VLAN_DEI_MASK                        (0x1U)
#define ENETAPP_VLAN_VID_MASK                        (0xFFFU)
#define ENETAPP_VLAN_TCI(pcp, dei, vid)              ((((pcp) & ENETAPP_VLAN_PCP_MASK) << ENETAPP_VLAN_PCP_OFFSET) | \
                                                      (((dei) & ENETAPP_VLAN_DEI_MASK) << ENETAPP_VLAN_DEI_OFFSET) | \
                                                      (((vid) & ENETAPP_VLAN_VID_MASK)))

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_waitForLinkUp(Enet_MacPort macPort);

static void EnetApp_txIsrFxn(void *appData);

static void EnetApp_rxIsrFxn(void *appData);

static void EnetApp_initTxFreePktQ(int8_t dmaChNum);

static void EnetApp_initRxReadyPktQ(int8_t dmaChNum);

static int32_t EnetApp_openTxCh(uint8_t dmaChNum);

static int32_t EnetApp_openRxCh(uint8_t dmaChNum);

static int32_t EnetApp_transmitPkt(char *payload, uint8_t *destMacAddr,
        uint8_t *srcMacAddr, uint16_t vlanId, uint8_t priority, int8_t dmaChNum);

static uint32_t EnetApp_retrieveFreeTxPkts(int8_t dmaChNum);

static int32_t EnetApp_createRxTask(int8_t dmaChNum);

static void EnetApp_recievePkt(void *args);

static uint32_t EnetApp_retrieveRxPkts(int8_t dmaChNum);

static void EnetApp_close(void);

static void EnetApp_closeDma(void);

static void EnetApp_printFrame(EthFrame *frame, uint32_t len);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Link speed default 100Mbps */
Enet_Speed EnetApp_linkSpeed = 1;

static uint8_t EnetApp_rxTaskStack[ENET_SYSCFG_RX_FLOWS_NUM][10U * 1024U] __attribute__ ((aligned(32)));

/* Buffer to store packets received from ethernet */
EthFrame *EnetApp_rxBuffer[ENET_SYSCFG_RX_FLOWS_NUM][4];
int32_t EnetApp_payloadLen[ENET_SYSCFG_RX_FLOWS_NUM][4];

/* Index of buffer */
int8_t EnetApp_writeBufferPt[ENET_SYSCFG_RX_FLOWS_NUM];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

BaseType_t EnetCLI_openTxChn(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status = ENET_SOK;
    uint8_t chNum;
    char *parameter;
    BaseType_t paramLen;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 1, &paramLen);
    chNum = atoi(parameter);
    if (EnetApp_inst.txDmaCh[chNum] != CH_CLOSE)
    {
        snprintf(writeBuffer, writeBufferLen,
                "Tx channel %d is already open\r\n", chNum);
        return pdFALSE;
    }

    /* Open Tx channel */
    status = EnetApp_openTxCh(chNum);
    if (status)
        snprintf(writeBuffer, writeBufferLen,
                "Failed to open new Tx channel\r\n");
    else
        snprintf(writeBuffer, writeBufferLen,
                "Opened new Tx channel with identifier %d\r\n", chNum);
    return pdFALSE;
}

BaseType_t EnetCLI_openRxChn(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status = ENET_SOK;
    uint8_t chNum;
    char *parameter;
    BaseType_t paramLen;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 1, &paramLen);
    chNum = atoi(parameter);
    if (EnetApp_inst.rxDmaCh[chNum] != CH_CLOSE)
    {
        snprintf(writeBuffer, writeBufferLen,
                "Rx channel %d is already open\r\n", chNum);
        return pdFALSE;
    }

    /* Open Rx channel */
    status = EnetApp_openRxCh(chNum);
    if (status)
        snprintf(writeBuffer, writeBufferLen, "Failed to open Rx channel\r\n");
    else
        snprintf(writeBuffer, writeBufferLen,
                "Opened new Rx channel with identifier %d\r\n", chNum);
    return pdFALSE;
}

BaseType_t EnetCLI_getHostMac(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    EnetAppUtils_print("Host MAC address: ");
    EnetAppUtils_printMacAddr(EnetApp_inst.hostMacAddr);
    return pdFALSE;
}

BaseType_t EnetCLI_transmitPkt(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status = ENET_SOK;
    uint8_t destAddr[6] = { 255, 255, 255, 255, 255, 255 };
    uint8_t srcAddr[6];
    EnetUtils_copyMacAddr(srcAddr, EnetApp_inst.hostMacAddr);
    int8_t dmaChNum = -1;
    uint16_t vlanId = 1;
    uint8_t priority = 0;
    char payloadMsg[100] = "";
    strcpy(payloadMsg, "Test packet from AM243x");
    char *parameter;
    BaseType_t paramLen;
    uint32_t paramCnt = 1;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
            &paramLen);
    while (parameter != NULL)
    {
        if (paramCnt == 1)
        {
            dmaChNum = atoi(parameter);
            if(dmaChNum > ENET_SYSCFG_TX_CHANNELS_NUM)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "Invalid DMA channel.\r\n");
                return pdFALSE;
            }
            else if(EnetApp_inst.txDmaCh[dmaChNum] == CH_CLOSE)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "DMA channel is not open.\r\n");
                return pdFALSE;
            }
            paramCnt += 1;
        }
        else if (strncmp("-dm", parameter, paramLen) == 0)
        {
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            status = EnetAppUtils_macAddrAtoI(parameter, destAddr);
            if (status)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "Invalid dest MAC address\r\n");
                return pdFALSE;
            }
            paramCnt += 2;
        }
        else if (strncmp("-sm", parameter, paramLen) == 0)
        {
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            status = EnetAppUtils_macAddrAtoI(parameter, srcAddr);
            if (status)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "Invalid source MAC address\r\n");
                return pdFALSE;
            }
            paramCnt += 2;
        }
        else if (strncmp("-v", parameter, paramLen) == 0)
        {
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            vlanId = atoi(parameter);
            paramCnt += 2;
        }
        else if (strncmp("-pcp", parameter, paramLen) == 0)
        {
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            priority = atoi(parameter);
            paramCnt += 2;
        }
        else if (strncmp("-m", parameter, paramLen) == 0)
        {
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            strcpy(payloadMsg, parameter);
            break;
        }
        parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                &paramLen);
    }

    if (dmaChNum < 0 || dmaChNum >= ENET_SYSCFG_TX_CHANNELS_NUM)
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid Tx channel\r\n");
        return pdFALSE;
    }
    else if (EnetApp_inst.txDmaCh[dmaChNum] == CH_CLOSE)
    {
        snprintf(writeBuffer, writeBufferLen, "Tx channel %d is not open\r\n",
                dmaChNum);
        return pdFALSE;
    }

    status = EnetApp_transmitPkt(payloadMsg, destAddr, srcAddr, vlanId,
            priority, dmaChNum);
    if (status)
        snprintf(writeBuffer, writeBufferLen, "Send Failed\r\n");
    else
        snprintf(writeBuffer, writeBufferLen, "Done\r\n");
    return pdFALSE;
}

BaseType_t EnetCLI_capturePkt(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status = ENET_SOK;
    char *parameter;
    BaseType_t paramLen;
    int8_t dmaChNum;
    bool isStart;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 1, &paramLen);
    if (strncmp(parameter, "start", paramLen) == 0)
        isStart = true;
    else if (strncmp(parameter, "stop", paramLen) == 0)
        isStart = false;
    else
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid Arg\r\n");
        return pdFALSE;
    }
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 2, &paramLen);
    dmaChNum = atoi(parameter);

    if (dmaChNum < 0 || dmaChNum >= ENET_SYSCFG_TX_CHANNELS_NUM)
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid Rx channel\r\n");
        return pdFALSE;
    }
    else if (EnetApp_inst.rxDmaCh[dmaChNum] == CH_CLOSE)
    {
        snprintf(writeBuffer, writeBufferLen, "Rx channel %d is not open\r\n",
                dmaChNum);
        return pdFALSE;
    }

    if (isStart)
    {
        if (EnetApp_inst.rxDmaCh[dmaChNum] != CH_IDLE)
        {
            snprintf(writeBuffer, writeBufferLen,
                    "Rx channel %d is already listening for packets\r\n",
                    dmaChNum);
            return pdFALSE;
        }

        status = EnetApp_createRxTask(dmaChNum);
        if (status)
            snprintf(writeBuffer, writeBufferLen,
                    "Failed to start Rx task\r\n");
        else
            snprintf(writeBuffer, writeBufferLen,
                    "Listening to packets at Rx channel %d\r\n", dmaChNum);
    }
    else
    {
        if (EnetApp_inst.rxDmaCh[dmaChNum] == CH_IDLE)
        {
            snprintf(writeBuffer, writeBufferLen,
                    "Rx channel %d is not listening to packets\r\n", dmaChNum);
            return pdFALSE;
        }

        EnetApp_inst.rxDmaCh[dmaChNum] = CH_STOP;

        /* Wait for Rx channel to stop listening to incoming packets */
        while (EnetApp_inst.rxDmaCh[dmaChNum] != CH_IDLE);

        snprintf(writeBuffer, writeBufferLen,
                "Rx channel %d stopped listening to packets\r\n", dmaChNum);
    }

    return pdFALSE;
}

BaseType_t EnetCLI_dumpRxBuffer(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int8_t dmaChNum = 0;
    static int8_t rxBufferPt = -1;
    static int8_t readStartPt = -1;
    char *parameter;
    BaseType_t paramLen;
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 1, &paramLen);
    if (parameter != NULL)
    {
        dmaChNum = atoi(parameter);
    }

    if (EnetApp_inst.rxDmaCh[dmaChNum] == CH_CLOSE)
    {
        snprintf(writeBuffer, writeBufferLen, "Rx channel %d is not open\r\n",
                dmaChNum);
        return pdFALSE;
    }

    if (rxBufferPt == -1)
    {
        rxBufferPt = EnetApp_writeBufferPt[dmaChNum];
        readStartPt = rxBufferPt;
    }
    if (EnetApp_rxBuffer[dmaChNum][rxBufferPt] != NULL)
    {
        snprintf(writeBuffer, writeBufferLen,
                "------PACKET ENDS HERE------\r\n\n");
        EnetApp_printFrame(EnetApp_rxBuffer[dmaChNum][rxBufferPt],
                EnetApp_payloadLen[dmaChNum][rxBufferPt]);
    }

    rxBufferPt = rxBufferPt == 3 ? 0 : rxBufferPt + 1;
    if (rxBufferPt == readStartPt)
    {
        rxBufferPt = -1;
        return pdFALSE;
    }
    else
        return pdTRUE;
}

BaseType_t EnetCLI_quitTerminal(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    snprintf(writeBuffer, writeBufferLen, "Exit\r\n");
    EnetApp_close();
    return pdFALSE;
}

int32_t EnetApp_init(void)
{
    int32_t status;
    EnetApp_HandleInfo handleInfo;

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &EnetApp_inst.enetType,
            &EnetApp_inst.instId);

    EnetApp_getEnetInstMacInfo(EnetApp_inst.enetType, EnetApp_inst.instId,
            EnetApp_inst.macPort, &EnetApp_inst.numMacPorts);

    EnetApp_inst.macMode = RGMII;
    EnetApp_inst.boardId = ENETBOARD_CPB_ID;
    EnetApp_inst.coreId = EnetSoc_getCoreId();
    EnetApp_inst.tsnFlag = TSN_NO_MAC;

    EnetAppUtils_enableClocks(EnetApp_inst.enetType, EnetApp_inst.instId);

    EnetApp_driverInit();

    status = EnetApp_driverOpen(EnetApp_inst.enetType, EnetApp_inst.instId);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("[ERR] %s: Failed to open ENET: %d\r\n", __func__,
                status);
        return status;
    }

    EnetApp_acquireHandleInfo(EnetApp_inst.enetType, EnetApp_inst.instId,
            &handleInfo);
    EnetApp_inst.hEnet = handleInfo.hEnet;

    EnetPer_AttachCoreOutArgs attachInfo;
    EnetApp_coreAttach(EnetApp_inst.enetType, EnetApp_inst.instId,
            EnetApp_inst.coreId, &attachInfo);
    EnetApp_inst.coreKey = attachInfo.coreKey;

#if ENET_SYSCFG_TX_CHANNELS_NUM >= 1
    EnetApp_pktPerTxCh[0] = ENET_DMA_TX_CH0_NUM_PKTS;
#endif
#if ENET_SYSCFG_TX_CHANNELS_NUM >= 2
    EnetApp_pktPerTxCh[1] = ENET_DMA_TX_CH1_NUM_PKTS;
#endif
#if ENET_SYSCFG_TX_CHANNELS_NUM >= 3
    EnetApp_pktPerTxCh[2] = ENET_DMA_TX_CH2_NUM_PKTS;
#endif
#if ENET_SYSCFG_TX_CHANNELS_NUM >= 4
    EnetApp_pktPerTxCh[3] = ENET_DMA_TX_CH3_NUM_PKTS;
#endif

#if ENET_SYSCFG_RX_FLOWS_NUM >= 1
    EnetApp_pktPerRxCh[0] = ENET_DMA_RX_CH0_NUM_PKTS;
#endif
#if ENET_SYSCFG_RX_FLOWS_NUM >= 2
    EnetApp_pktPerRxCh[1] = ENET_DMA_RX_CH1_NUM_PKTS;
#endif
#if ENET_SYSCFG_RX_FLOWS_NUM >= 3
    EnetApp_pktPerRxCh[2] = ENET_DMA_RX_CH2_NUM_PKTS;
#endif
#if ENET_SYSCFG_RX_FLOWS_NUM >= 4
    EnetApp_pktPerRxCh[3] = ENET_DMA_RX_CH3_NUM_PKTS;
#endif

    EnetApp_openTxCh(0);
    EnetApp_openRxCh(0);

    EnetApp_inst.initFlag = ENET_UP;

    uint8_t macPortNum;
    for (macPortNum = 0; macPortNum < EnetApp_inst.numMacPorts; macPortNum++)
        EnetApp_waitForLinkUp(EnetApp_inst.macPort[macPortNum]);

    return 0;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void EnetApp_waitForLinkUp(Enet_MacPort macPort)
{
    uint8_t timeout = 0;
    Enet_IoctlPrms prms;
    bool linked = false;
    int32_t status = ENET_SOK;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);

    while (!linked)
    {
        ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
                ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print(
                    "[WRN] %s: Failed to get port %u's link status: %d\r\n",
                    __func__, ENET_MACPORT_ID(macPort), status);
            linked = false;
            break;
        }
        if (timeout >= 150)
        {
            break;
        }
        if (!linked)
        {
            /* wait for 50 ms and poll again*/
            ClockP_usleep(50000);
            timeout++;
        }
    }
}

static void EnetApp_txIsrFxn(void *appData)
{
    SemaphoreP_post(appData);
}

static void EnetApp_rxIsrFxn(void *appData)
{
    SemaphoreP_post(appData);
}

static void EnetApp_initTxFreePktQ(int8_t dmaChNum)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = {
    ENET_MEM_LARGE_POOL_PKT_SIZE, };

    /* Initialize all queues */
    EnetQueue_initQ(&EnetApp_inst.txFreePktInfoQ[dmaChNum]);

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < EnetApp_pktPerTxCh[dmaChNum]; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&EnetApp_inst,
                ENETDMA_CACHELINE_ALIGNMENT, ENET_ARRAYSIZE(scatterSegments),
                scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState,
                ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&EnetApp_inst.txFreePktInfoQ[dmaChNum], &pPktInfo->node);
    }
}

static void EnetApp_initRxReadyPktQ(int8_t dmaChNum)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pPktInfo;
    int32_t status;
    uint32_t i;
    uint32_t scatterSegments[] = {
    ENET_MEM_LARGE_POOL_PKT_SIZE, };

    EnetQueue_initQ(&EnetApp_inst.rxFreeQ[dmaChNum]);
    EnetQueue_initQ(&EnetApp_inst.rxReadyQ[dmaChNum]);
    EnetQueue_initQ(&rxReadyQ);

    for (i = 0U; i < EnetApp_pktPerRxCh[dmaChNum]; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&EnetApp_inst,
                ENETDMA_CACHELINE_ALIGNMENT, ENET_ARRAYSIZE(scatterSegments),
                scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState,
                ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetQueue_enq(&EnetApp_inst.rxFreeQ[dmaChNum], &pPktInfo->node);
    }

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(EnetApp_inst.hRxCh[dmaChNum], &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);
    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&EnetApp_inst.rxFreeQ[dmaChNum],
            ENET_PKTSTATE_APP_WITH_FREEQ, ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(EnetApp_inst.hRxCh[dmaChNum],
            &EnetApp_inst.rxFreeQ[dmaChNum]);

    while (0U != EnetQueue_getQCount(&EnetApp_inst.rxFreeQ[dmaChNum]))
        EnetDma_submitRxPktQ(EnetApp_inst.hRxCh[dmaChNum],
                &EnetApp_inst.rxFreeQ[dmaChNum]);
}

static int32_t EnetApp_openTxCh(uint8_t dmaChNum)
{
    int32_t status = ENET_SOK;

    /* Intialize semaphores */
    status = SemaphoreP_constructBinary(&EnetApp_inst.txSemObj[dmaChNum], 0);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("[ERR] %s: Failed to create semaphore: %d\r\n",
                __func__, status);
        return 1;
    }

    /* Open the CPSW TX flow */
    EnetApp_GetDmaHandleInArgs txInArgs;
    EnetApp_GetTxDmaHandleOutArgs txChInfo;

    txInArgs.cbArg = &EnetApp_inst.txSemObj[dmaChNum];
    txInArgs.notifyCb = EnetApp_txIsrFxn;

    EnetApp_getTxDmaHandle(dmaChNum, &txInArgs, &txChInfo);
    EnetApp_inst.hTxCh[dmaChNum] = txChInfo.hTxCh;
    if (NULL == EnetApp_inst.hTxCh[dmaChNum])
    {
        EnetAppUtils_print("[ERR] %s:Couldn't open Tx channel %d\r\n", __func__,
                dmaChNum);
        return 1;
    }

    EnetApp_initTxFreePktQ(dmaChNum);

    EnetApp_inst.txDmaCh[dmaChNum] = CH_IDLE;
    return 0;
}

static int32_t EnetApp_openRxCh(uint8_t dmaChNum)
{
    int32_t status = ENET_SOK;

    /* Intialize semaphores */
    status = SemaphoreP_constructBinary(&EnetApp_inst.rxSemObj[dmaChNum], 0);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("[ERR] %s: Failed to create semaphores: %d\r\n",
                __func__, status);
        return 1;
    }

    /* Open the CPSW RX flow  */
    EnetApp_GetRxDmaHandleOutArgs rxChInfo;
    EnetApp_GetDmaHandleInArgs rxInArgs;

    rxInArgs.cbArg = &EnetApp_inst.rxSemObj[dmaChNum];
    rxInArgs.notifyCb = EnetApp_rxIsrFxn;

    EnetApp_getRxDmaHandle(dmaChNum, &rxInArgs, &rxChInfo);
    EnetApp_inst.hRxCh[dmaChNum] = rxChInfo.hRxCh;
    if (NULL == EnetApp_inst.hRxCh[dmaChNum])
    {
        EnetAppUtils_print("[ERR] %s: Couldn't open Rx channel %d\r\n",
                __func__, dmaChNum);
        return 1;
    }

    if (dmaChNum == 0)
    {
        EnetUtils_copyMacAddr(EnetApp_inst.hostMacAddr,
                &rxChInfo.macAddr[0][0]);
        EnetApp_inst.tsnFlag = TSN_IDLE;
    }

    EnetApp_initRxReadyPktQ(dmaChNum);

    EnetApp_inst.rxDmaCh[dmaChNum] = CH_IDLE;
    return status;
}

static int32_t EnetApp_transmitPkt(char *payload, uint8_t *destMacAddr,
        uint8_t *srcMacAddr, uint16_t vlanId, uint8_t priority, int8_t dmaChNum)
{
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *pktInfo;
    uint32_t txRetrievePktCnt;
    size_t pktLen = ETH_PAYLOAD_LEN;
    int32_t status = ENET_SOK;
    size_t payloadLen = strlen(payload);
    size_t txLen = 0;

    while (txLen < payloadLen)
    {
        EnetQueue_initQ(&txSubmitQ);

        /* Dequeue one free TX Eth packet */
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(
                &EnetApp_inst.txFreePktInfoQ[dmaChNum]);

        while (NULL != pktInfo)
        {
            if (payloadLen - txLen < ETH_PAYLOAD_LEN)
                pktLen = payloadLen - txLen;
            /* Fill the TX Eth frame with test content */
            if (vlanId == 1)
            {
                EthFrame *frame = (EthFrame*) pktInfo->sgList.list[0].bufPtr;
                memcpy(frame->hdr.dstMac, destMacAddr, ENET_MAC_ADDR_LEN);
                memcpy(frame->hdr.srcMac, srcMacAddr,
                ENET_MAC_ADDR_LEN);

                frame->hdr.etherType = Enet_htons(ETHERTYPE_EXPERIMENTAL1);
                memcpy(&frame->payload[0U], (payload + txLen), pktLen);
                pktInfo->sgList.list[0].segmentFilledLen = pktLen
                        + sizeof(EthFrameHeader);
            }
            else
            {
                EthVlanFrame *vlanFrame =
                        (EthVlanFrame*) pktInfo->sgList.list[0].bufPtr;
                memcpy(vlanFrame->hdr.dstMac, destMacAddr, ENET_MAC_ADDR_LEN);
                memcpy(vlanFrame->hdr.srcMac, srcMacAddr,
                ENET_MAC_ADDR_LEN);

                vlanFrame->hdr.tpid = Enet_htons(0x8100U);
                vlanFrame->hdr.tci = Enet_htons(
                        ENETAPP_VLAN_TCI(priority, 0, vlanId));
                vlanFrame->hdr.etherType = Enet_htons(ETHERTYPE_EXPERIMENTAL1);
                memcpy(&vlanFrame->payload[0U], (payload + txLen), pktLen);
                pktInfo->sgList.list[0].segmentFilledLen = pktLen
                        + sizeof(EthVlanFrameHeader);
            }

            pktInfo->sgList.numScatterSegments = 1;
            pktInfo->chkSumInfo = 0U;
            pktInfo->appPriv = &EnetApp_inst;
            EnetDma_checkPktState(&pktInfo->pktState, ENET_PKTSTATE_MODULE_APP,
                    ENET_PKTSTATE_APP_WITH_FREEQ,
                    ENET_PKTSTATE_APP_WITH_DRIVER);

            /* Enqueue the packet for later transmission */
            EnetQueue_enq(&txSubmitQ, &pktInfo->node);
            txLen = txLen + pktLen;
            if (txLen >= payloadLen)
            {
                break;
            }

            /* Dequeue one free TX Eth packet */
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(
                    &EnetApp_inst.txFreePktInfoQ[dmaChNum]);
        }

        while (0U != EnetQueue_getQCount(&txSubmitQ))
        {
            uint32_t txCnt = EnetQueue_getQCount(&txSubmitQ);
            status = EnetDma_submitTxPktQ(EnetApp_inst.hTxCh[dmaChNum],
                    &txSubmitQ);
            SemaphoreP_pend(&EnetApp_inst.txSemObj[dmaChNum],
            SystemP_WAIT_FOREVER);

            /* Retrieve TX free packets */
            if (status == ENET_SOK)
            {
                txCnt = txCnt - EnetQueue_getQCount(&txSubmitQ);
                txRetrievePktCnt = 0U;
                while (txRetrievePktCnt != txCnt)
                {
                    /* This is not failure as HW is busy sending packets, we
                     * need to wait and again call retrieve packets */
                    ClockP_usleep(1000);
                    txRetrievePktCnt += EnetApp_retrieveFreeTxPkts(dmaChNum);
                }
            }
            else
            {
                EnetAppUtils_print("[ERR] %s: Failed to transmit packet %d",
                        __func__, status);
                return 1;
            }
        }
    }

    EnetAppUtils_print(
            "Send details:\r\n channel: %d,\r\n payload: \"%s\",\r\n Source MAC: ",
            dmaChNum, payload);
    EnetAppUtils_printMacAddr(srcMacAddr);
    EnetAppUtils_print(" Destination MAC: ");
    EnetAppUtils_printMacAddr(destMacAddr);
    return 0;
}

static uint32_t EnetApp_retrieveFreeTxPkts(int8_t dmaChNum)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t txFreeQCnt = 0U;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any CPSW packets that may be free now */
    status = EnetDma_retrieveTxPktQ(EnetApp_inst.hTxCh[dmaChNum], &txFreeQ);
    if (status == ENET_SOK)
    {
        txFreeQCnt = EnetQueue_getQCount(&txFreeQ);

        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&txFreeQ);
        while (NULL != pktInfo)
        {
            EnetDma_checkPktState(&pktInfo->pktState, ENET_PKTSTATE_MODULE_APP,
                    ENET_PKTSTATE_APP_WITH_DRIVER,
                    ENET_PKTSTATE_APP_WITH_FREEQ);

            EnetQueue_enq(&EnetApp_inst.txFreePktInfoQ[dmaChNum],
                    &pktInfo->node);
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print("[ERR] %s: Failed to retrieve pkts: %d\r\n",
                __func__, status);
    }

    return txFreeQCnt;
}

static int32_t EnetApp_createRxTask(int8_t dmaChNum)
{
    TaskP_Params taskParams;
    int32_t status;
    EnetApp_inst.rxDmaCh[dmaChNum] = CH_RUNNING;
    TaskP_Params_init(&taskParams);
    taskParams.priority = 3U;
    taskParams.stack = EnetApp_rxTaskStack[dmaChNum];
    taskParams.stackSize = sizeof(EnetApp_rxTaskStack[dmaChNum]);
    taskParams.args = (void*) &dmaChNum;
    taskParams.name = "Rx Task";
    taskParams.taskMain = &EnetApp_recievePkt;

    status = TaskP_construct(&EnetApp_inst.rxTaskObj[dmaChNum], &taskParams);
    if (SystemP_SUCCESS != status)
    {
        EnetAppUtils_print("[ERR] %s: Failed to create Rx task %d\r\n",
                __func__, status);
        return 1;
    }
    return 0;
}

static void EnetApp_recievePkt(void *args)
{
    int8_t dmaChNum = *(int8_t*) args;
    EnetDma_Pkt *pktInfo;
    uint32_t rxReadyCnt;
    int32_t status = ENET_SOK;
    uint32_t rxPktCnt = 0;

    while (EnetApp_inst.rxDmaCh[dmaChNum] != CH_STOP)
    {
        status = SemaphoreP_pend(&EnetApp_inst.rxSemObj[dmaChNum],
                ClockP_usecToTicks(500000));
        if (status != SystemP_SUCCESS)
            continue;
        /* Get the packets received so far */
        rxReadyCnt = EnetApp_retrieveRxPkts(dmaChNum);
        if (rxReadyCnt > 0U)
        {
            /* Consume the received packets and release them */
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(
                    &EnetApp_inst.rxReadyQ[dmaChNum]);
            while (NULL != pktInfo)
            {
                rxPktCnt++;
                EnetDma_checkPktState(&pktInfo->pktState,
                        ENET_PKTSTATE_MODULE_APP, ENET_PKTSTATE_APP_WITH_READYQ,
                        ENET_PKTSTATE_APP_WITH_FREEQ);

                /* Store payload to rx buffer */
                EnetApp_rxBuffer[dmaChNum][EnetApp_writeBufferPt[dmaChNum]] =
                        (EthFrame*) pktInfo->sgList.list[0].bufPtr;
                EnetApp_payloadLen[dmaChNum][EnetApp_writeBufferPt[dmaChNum]] =
                        pktInfo->sgList.list[0].segmentFilledLen
                                - sizeof(EthFrameHeader);

                EnetApp_writeBufferPt[dmaChNum] =
                        EnetApp_writeBufferPt[dmaChNum] == 3 ?
                                0 : EnetApp_writeBufferPt[dmaChNum] + 1;

                EnetQueue_enq(&EnetApp_inst.rxFreeQ[dmaChNum], &pktInfo->node);
                pktInfo = (EnetDma_Pkt*) EnetQueue_deq(
                        &EnetApp_inst.rxReadyQ[dmaChNum]);
            }

            /*Submit now processed buffers */
            if (status == ENET_SOK)
            {
                EnetAppUtils_validatePacketState(
                        &EnetApp_inst.rxFreeQ[dmaChNum],
                        ENET_PKTSTATE_APP_WITH_FREEQ,
                        ENET_PKTSTATE_APP_WITH_DRIVER);

                EnetDma_submitRxPktQ(EnetApp_inst.hRxCh[dmaChNum],
                        &EnetApp_inst.rxFreeQ[dmaChNum]);
            }
        }
    }
    EnetAppUtils_print(
            "Stopped capturing at Rx channel %d. Recieved %d packets\r\n",
            dmaChNum, rxPktCnt);
    EnetApp_inst.rxDmaCh[dmaChNum] = CH_IDLE;
    TaskP_destruct(&EnetApp_inst.rxTaskObj[dmaChNum]);
    TaskP_exit();
}

static uint32_t EnetApp_retrieveRxPkts(int8_t dmaChNum)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t rxReadyCnt = 0U;

    EnetQueue_initQ(&rxReadyQ);

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(EnetApp_inst.hRxCh[dmaChNum], &rxReadyQ);
    if (status == ENET_SOK)
    {
        rxReadyCnt = EnetQueue_getQCount(&rxReadyQ);

        /* Queue the received packet to rxReadyQ and pass new ones from rxFreeQ
         **/
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
        while (pktInfo != NULL)
        {
            EnetDma_checkPktState(&pktInfo->pktState, ENET_PKTSTATE_MODULE_APP,
                    ENET_PKTSTATE_APP_WITH_DRIVER,
                    ENET_PKTSTATE_APP_WITH_READYQ);

            EnetQueue_enq(&EnetApp_inst.rxReadyQ[dmaChNum], &pktInfo->node);
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
        }
    }
    else
    {
        EnetAppUtils_print("[ERR] %s: Failed to retrieve pkts: %d\r\n",
                __func__, status);
    }

    return rxReadyCnt;
}

static void EnetApp_close(void)
{
    /* if TSN stack is still running, stop it first */
    if (EnetApp_inst.tsnFlag == 1)
    {
        UART_writeCLI("Stopping TSN stack...\r\n");
        EnetTsn_stopTsn();
    }

    /* if LwIP shell is still running, stop it first */
    if (EnetApp_inst.shellFlag == 1)
    {
        UART_writeCLI("Stopping LwIP shell...\r\n");
        Lwip_shutdownNetworkStack();
    }

    /* Close Enet DMA driver */
    UART_writeCLI("Closing DMA channels...\r\n");
    EnetApp_closeDma();

    /*Release Handle Info*/
    UART_writeCLI("Releasing handles...\r\n");
    EnetApp_releaseHandleInfo(EnetApp_inst.enetType, EnetApp_inst.instId);
    EnetApp_inst.hEnet = NULL;

    UART_writeCLI("Deinitializing drives...\r\n");
    EnetApp_driverDeInit();

    /* Disable peripheral clocks */
    UART_writeCLI("Diabling clocks...\r\n");
    EnetAppUtils_disableClocks(EnetApp_inst.enetType, EnetApp_inst.instId);
}

static void EnetApp_closeDma(void)
{
    /* Close Tx channels */
    for (int8_t i = ENET_SYSCFG_TX_CHANNELS_NUM - 1; i >= 0; i--)
    {
        EnetDma_PktQ fqPktInfoQ;
        EnetDma_PktQ cqPktInfoQ;

        EnetQueue_initQ(&fqPktInfoQ);
        EnetQueue_initQ(&cqPktInfoQ);

        EnetApp_closeTxDma(i, EnetApp_inst.hEnet, EnetApp_inst.coreKey,
                EnetApp_inst.coreId, &fqPktInfoQ, &cqPktInfoQ);

        EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
        EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

        if (EnetApp_inst.txDmaCh[i] != CH_CLOSE)
        {
            EnetAppUtils_freePktInfoQ(&EnetApp_inst.txFreePktInfoQ[i]);
            SemaphoreP_destruct(&EnetApp_inst.txSemObj[i]);
        }
    }

    /* Close Rx channels */
    for (int8_t i = ENET_SYSCFG_RX_FLOWS_NUM - 1; i >= 0; i--)
    {
        EnetDma_PktQ fqPktInfoQ;
        EnetDma_PktQ cqPktInfoQ;

        EnetQueue_initQ(&fqPktInfoQ);
        EnetQueue_initQ(&cqPktInfoQ);

        /* There should not be any ready packet */
        if (EnetApp_inst.rxDmaCh[i] != CH_CLOSE)
            EnetAppUtils_assert(
                    0U == EnetQueue_getQCount(&EnetApp_inst.rxReadyQ[i]));

        EnetApp_closeRxDma(i, EnetApp_inst.hEnet, EnetApp_inst.coreKey,
                EnetApp_inst.coreId, &fqPktInfoQ, &cqPktInfoQ);

        EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
        EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

        if (EnetApp_inst.rxDmaCh[i] != CH_CLOSE)
        {
            EnetAppUtils_freePktInfoQ(&EnetApp_inst.rxFreeQ[i]);
            SemaphoreP_destruct(&EnetApp_inst.rxSemObj[i]);
        }
    }
}

static void EnetApp_printFrame(EthFrame *frame, uint32_t len)
{
    uint8_t *payload;
    uint32_t i;
    char buffer[50] = "";

    UART_writeCLI("Dst addr : ");
    sprintf(buffer, "%02x:%02x:%02x:%02x:%02x:%02x\r\n",
            frame->hdr.dstMac[0] & 0xFF, frame->hdr.dstMac[1] & 0xFF,
            frame->hdr.dstMac[2] & 0xFF, frame->hdr.dstMac[3] & 0xFF,
            frame->hdr.dstMac[4] & 0xFF, frame->hdr.dstMac[5] & 0xFF);
    UART_writeCLI(buffer);

    UART_writeCLI("Src addr : ");
    sprintf(buffer, "%02x:%02x:%02x:%02x:%02x:%02x\r\n",
            frame->hdr.srcMac[0] & 0xFF, frame->hdr.srcMac[1] & 0xFF,
            frame->hdr.srcMac[2] & 0xFF, frame->hdr.srcMac[3] & 0xFF,
            frame->hdr.srcMac[4] & 0xFF, frame->hdr.srcMac[5] & 0xFF);
    UART_writeCLI(buffer);

    if (frame->hdr.etherType == Enet_htons(ETHERTYPE_VLAN_TAG))
    {
        EthVlanFrame *vlanFrame = (EthVlanFrame*) frame;

        sprintf(buffer, "TPID     : 0x%04x\r\n",
                (Enet_ntohs(vlanFrame->hdr.tpid) & 0xFFFFU));
        UART_writeCLI(buffer);
        sprintf(buffer, "Priority : %d\r\n",
                (Enet_ntohs(vlanFrame->hdr.tci) & 0xFFFFU) >> 13);
        UART_writeCLI(buffer);
        sprintf(buffer, "VLAN Id  : %d\r\n",
                (Enet_ntohs(vlanFrame->hdr.tci) & 0xFFFU));
        UART_writeCLI(buffer);
        sprintf(buffer, "EtherType: 0x%04x\r\n",
                (Enet_ntohs(vlanFrame->hdr.etherType) & 0xFFFFU));
        UART_writeCLI(buffer);
        payload = vlanFrame->payload;
        len -= ETH_VLAN_TAG_LEN;
    }
    else
    {
        sprintf(buffer, "EtherType: 0x%04x\r\n",
                (Enet_ntohs(frame->hdr.etherType) & 0xFFFFU));
        UART_writeCLI(buffer);
        payload = frame->payload;
    }

    UART_writeCLI("Payload  : ");
    for (i = 0; i < len; i++)
    {
        sprintf(buffer, "0x%02x ", payload[i]);
        UART_writeCLI(buffer);
        if (i && (((i + 1) % OCTETS_PER_ROW) == 0))
        {
            UART_writeCLI("\r\n           ");
        }
    }

    if (len && ((len % OCTETS_PER_ROW) != 0))
    {
        UART_writeCLI("\r\n");
    }

    UART_writeCLI("\r\n");
}

/* ========================================================================== */
/*                   Function Definitions for SysConfig                       */
/* ========================================================================== */

void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId,
        Cpsw_Cfg *cpswCfg)
{
    CpswHostPort_Cfg *hostPortCfg = &cpswCfg->hostPortCfg;
    CpswAle_Cfg *aleCfg = &cpswCfg->aleCfg;

    /* Set Enet global runtime log level */
    Enet_setTraceLevel(ENET_TRACE_DEBUG);

    /* Host port config */
    hostPortCfg->removeCrc = true;
    hostPortCfg->padShortPacket = true;
    hostPortCfg->passCrcErrors = true;

    /* ALE config */
    aleCfg->modeFlags = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn = true;
    aleCfg->agingCfg.agingPeriodInMs = 1000;
    aleCfg->nwSecCfg.vid0ModeEn = true;
    aleCfg->vlanCfg.aleVlanAwareMode = true;
    aleCfg->vlanCfg.cpswVlanAwareMode = true;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->policerGlobalCfg.policingEn = true;
    aleCfg->policerGlobalCfg.redDropEn = true;
    aleCfg->policerGlobalCfg.yellowDropEn = false;
    aleCfg->policerGlobalCfg.policerNoMatchMode = CPSW_ALE_POLICER_NOMATCH_MODE_GREEN;
}

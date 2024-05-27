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
 * \file  cli_functions.c
 *
 * \brief This file contains the definitions of the functions used by the CLI application.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_common.h"
#include "cli_functions.h"

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

static int8_t EnetApp_init(void);

static void EnetApp_close(void);

static int32_t EnetApp_waitForLinkUp(Enet_MacPort macPort);

static int8_t EnetApp_isValidMacAddr(char *mac);

static int8_t EnetApp_addUcastEntry(uint8_t *macAddr);

static int8_t EnetApp_addClassifierEntry(CpswAle_PolicerMatchParams prm,
        int8_t rxCh);

static int8_t EnetApp_transmitPkt(char *payload, uint8_t *destMacAddr,
        int8_t dmaChNum);

static uint32_t EnetApp_retrieveFreeTxPkts(int8_t dmaChNum);

static void EnetApp_createRxTask(int8_t dmaChNum);

static uint32_t EnetApp_retrieveRxPkts(int8_t dmaChNum);

static void EnetApp_recievePkt(void *args);

static void EnetApp_macMode2MacMii(emac_mode macMode,
        EnetMacPort_Interface *mii);

static void EnetApp_txIsrFxn(void *appData);

static void EnetApp_rxIsrFxn(void *appData);

static void EnetApp_initTxFreePktQ(int8_t dmaChNum);

static void EnetApp_initRxReadyPktQ(int8_t dmaChNum);

static int32_t EnetApp_openTxDma(void);

static int32_t EnetApp_openRxDma(void);

static void EnetApp_closeDma(void);

static void EnetApp_showCpswStats(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Number of packets per tx channel */
int32_t EnetApp_pktPerTxCh[MAX_NUM_DMA_CH] = { 4, 4 };

/* Number of packets per rx channel */
int32_t EnetApp_pktPerRxCh[MAX_NUM_DMA_CH] = { 8, 8 };

static uint8_t EnetApp_rxTaskStack[MAX_NUM_DMA_CH][10U * 1024U] __attribute__ ((aligned(32)));

/* Buffer to store packets received from ethernet */
EthFrame *EnetApp_rxBuffer[MAX_NUM_DMA_CH][4];
int32_t EnetApp_payloadLen[MAX_NUM_DMA_CH][4];

/* Index of buffer */
int8_t EnetApp_writeBufferPt[MAX_NUM_DMA_CH];
int8_t EnetApp_readBufferPt[MAX_NUM_DMA_CH];

/* Link speed default 100Mbps */
Enet_Speed EnetApp_linkSpeed = 1;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

BaseType_t EnetCLI_greet(char *pcWriteBuffer, size_t xWriteBufferLen,
        const char *pcCommandString)
{
    strncpy(pcWriteBuffer, "Hello There!!!\r\n", xWriteBufferLen);
    return pdFALSE;
}

BaseType_t EnetCLI_quitTerminal(char *pcWriteBuffer, size_t xWriteBufferLen,
        const char *pcCommandString)
{
    strncpy(pcWriteBuffer, "Quitting\r\n", xWriteBufferLen);
    EnetApp_close();
    return pdFALSE;
}

BaseType_t EnetCLI_init(char *pcWriteBuffer, size_t xWriteBufferLen,
        const char *pcCommandString)
{
    char *pcParameter;
    BaseType_t paramLen;
    pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
            &paramLen);
    if (pcParameter != NULL)
    {
        if (atoi(pcParameter) < 0 || atoi(pcParameter) > 3)
        {
            strncpy(pcWriteBuffer, "Invalid link speed\r\n", xWriteBufferLen);
            return pdFALSE;
        }
        else
            EnetApp_linkSpeed = atoi(pcParameter);
    }
    if (EnetApp_Inst.initFlag != 0)
    {
        strncpy(pcWriteBuffer, "Drivers are already initialized\r\n",
                xWriteBufferLen);
        return pdFALSE;
    }
    int32_t status = EnetApp_init();
    if (status)
    {
        strncpy(pcWriteBuffer, "Failed to initialize drives\r\n",
                xWriteBufferLen);
        return pdFALSE;
    }

    /* Wait for linkup */
    uint8_t macPortNum;
    for (macPortNum = 0; macPortNum < EnetApp_Inst.numMacPorts; macPortNum++)
        status = EnetApp_waitForLinkUp(EnetApp_Inst.macPort[macPortNum]);
    strncpy(pcWriteBuffer, "Initialization Completed\r\n", xWriteBufferLen);
    return pdFALSE;
}

BaseType_t EnetCLI_openTxDma(char *pcWriteBuffer, size_t xWriteBufferLen,
        const char *pcCommandString)
{
    int32_t status = 0;
    if (EnetApp_Inst.numTxDmaCh == MAX_NUM_DMA_CH)
    {
        strncpy(pcWriteBuffer, "Maximum Tx DMA channels used\r\n",
                xWriteBufferLen);
        return pdFALSE;
    }
    char *pcParameter;
    BaseType_t paramLen;
    pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
            &paramLen);
    if (pcParameter != NULL)
    {
        if (atoi(pcParameter) > EnetApp_Inst.numTxDmaCh)
        {
            strncpy(pcWriteBuffer, "Previous Tx DMA channel is not open\r\n",
                    xWriteBufferLen);
            return pdFALSE;
        }
        else if (atoi(pcParameter) < EnetApp_Inst.numTxDmaCh)
        {
            strncpy(pcWriteBuffer, "Tx DMA channel is already open\r\n",
                    xWriteBufferLen);
            return pdFALSE;
        }
    }
    status = EnetApp_openTxDma();
    if (status)
    {
        strncpy(pcWriteBuffer, "Failed to open Tx DMA channel\r\n",
                xWriteBufferLen);
    }
    return pdFALSE;
}

BaseType_t EnetCLI_openRxDma(char *pcWriteBuffer, size_t xWriteBufferLen,
        const char *pcCommandString)
{
    int32_t status = 0;
    if (EnetApp_Inst.numRxDmaCh == MAX_NUM_DMA_CH)
    {
        strncpy(pcWriteBuffer, "Maximum Rx DMA channels used\r\n",
                xWriteBufferLen);
        return pdFALSE;
    }
    char *pcParameter;
    BaseType_t paramLen;
    pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
            &paramLen);
    if (pcParameter != NULL)
    {
        if (atoi(pcParameter) > EnetApp_Inst.numRxDmaCh)
        {
            strncpy(pcWriteBuffer, "Previous Rx DMA channel is not open\r\n",
                    xWriteBufferLen);
            return pdFALSE;
        }
        else if (atoi(pcParameter) < EnetApp_Inst.numRxDmaCh)
        {
            strncpy(pcWriteBuffer, "Rx DMA channel is already open\r\n",
                    xWriteBufferLen);
            return pdFALSE;
        }
    }
    status = EnetApp_openRxDma();
    if (status)
    {
        strncpy(pcWriteBuffer, "Failed to open Rx DMA channel\r\n",
                xWriteBufferLen);
    }
    return pdFALSE;
}

BaseType_t EnetCLI_addUcast(char *pcWriteBuffer, size_t xWriteBufferLen,
        const char *pcCommandString)
{
    uint8_t macAddr[ENET_MAC_ADDR_LEN];
    char *pcParameter;
    BaseType_t paramLen;
    uint32_t paramCnt = 1;
    uint8_t makeDefault = 0;
    pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString, paramCnt,
            &paramLen);
    while (pcParameter != NULL)
    {
        if (strncmp(pcParameter, "-d", paramLen) == 0)
            makeDefault = 1;
        else if (EnetApp_isValidMacAddr(pcParameter))
        {
            sscanf(pcParameter, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &macAddr[0],
                    &macAddr[1], &macAddr[2], &macAddr[3], &macAddr[4],
                    &macAddr[5]);
            if (makeDefault)
            {
                EnetUtils_copyMacAddr(EnetApp_Inst.hostMacAddr, macAddr);
                EnetAppUtils_print("Default MAC address set to: ");
                EnetAppUtils_printMacAddr(EnetApp_Inst.hostMacAddr);
            }
            break;
        }
        else
        {
            strncpy(pcWriteBuffer, "Invalid Args\r\n", xWriteBufferLen);
            return pdFALSE;
        }
        paramCnt++;
        pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString,
                paramCnt, &paramLen);
    }
    uint32_t status = EnetApp_addUcastEntry(macAddr);
    if (status)
        strncpy(pcWriteBuffer, "Command failed\r\n", xWriteBufferLen);
    else
        strncpy(pcWriteBuffer, "Done\r\n", xWriteBufferLen);
    return pdFALSE;
}

BaseType_t EnetCLI_addClassifier(char *pcWriteBuffer, size_t xWriteBufferLen,
        const char *pcCommandString)
{
    char *pcParameter;
    BaseType_t paramLen;
    uint32_t paramCnt = 1;
    CpswAle_PolicerMatchParams prms;
    memset(&prms, 0, sizeof(prms));
    uint8_t rxCh = 1;
    pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString, paramCnt,
            &paramLen);
    while (pcParameter != NULL)
    {
        if (strncmp(pcParameter, "-e", paramLen) == 0)
        {
            prms.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_ETHERTYPE;
            pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString,
                    paramCnt + 1, &paramLen);
            prms.etherType = (uint16_t) strtol(pcParameter, NULL, 16);
        }
        else if (strncmp(pcParameter, "-c", paramLen) == 0)
        {
            pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString,
                    paramCnt + 1, &paramLen);
            if (atoi(pcParameter) >= EnetApp_Inst.numRxDmaCh
                    || rxCh >= EnetApp_Inst.numRxDmaCh)
            {
                strncpy(pcWriteBuffer, "Rx DMA channel is not open\r\n",
                        xWriteBufferLen);
                return pdFALSE;
            }
            rxCh = atoi(pcParameter);
        }
        else
        {
            strncpy(pcWriteBuffer, "Invalid Args\r\n", xWriteBufferLen);
            return pdFALSE;
        }
        paramCnt += 2;
        pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString,
                paramCnt, &paramLen);
    }
    if (prms.policerMatchEnMask == 0)
    {
        strncpy(pcWriteBuffer, "No classifier rules\r\n", xWriteBufferLen);
        return pdFALSE;
    }
    int32_t status = EnetApp_addClassifierEntry(prms, rxCh);
    if (status)
        strncpy(pcWriteBuffer, "Command failed\r\n", xWriteBufferLen);
    else
        strncpy(pcWriteBuffer, "Done\r\n", xWriteBufferLen);
    return pdFALSE;
}

BaseType_t EnetCLI_transmitPkt(char *pcWriteBuffer, size_t xWriteBufferLen,
        const char *pcCommandString)
{
    uint8_t destAddr[6] = { 255, 255, 255, 255, 255, 255 };
    uint8_t dmaChNum = 0;
    char payloadMsg[50] = "";
    strcpy(payloadMsg, "Test packet transferred from AM243x");
    char *pcParameter;
    BaseType_t paramLen;
    uint32_t paramCnt = 1;
    pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString, paramCnt,
            &paramLen);
    while (pcParameter != NULL)
    {
        if (strncmp("-m", pcParameter, paramLen) == 0)
        {
            pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString,
                    paramCnt + 1, &paramLen);
            if (pcParameter != NULL)
            {
                char destAddr_s[18] = "";
                strncpy(destAddr_s, pcParameter, paramLen);
                if (!EnetApp_isValidMacAddr(destAddr_s))
                {
                    strncpy(pcWriteBuffer, "Invalid MAC address\r\n",
                            xWriteBufferLen);
                    return pdFALSE;
                }
                sscanf(destAddr_s, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                        &destAddr[0], &destAddr[1], &destAddr[2], &destAddr[3],
                        &destAddr[4], &destAddr[5]);
            }
            paramCnt += 2;
        }
        else if (strncmp("-c", pcParameter, paramLen) == 0)
        {
            pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString,
                    paramCnt + 1, &paramLen);
            if (pcParameter != NULL)
                dmaChNum = atoi(pcParameter);
            paramCnt += 2;
        }
        else
        {
            strcpy(payloadMsg, pcParameter);
            break;
        }
        pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString,
                paramCnt, &paramLen);
    }
    if (dmaChNum >= EnetApp_Inst.numTxDmaCh)
    {
        strncpy(pcWriteBuffer, "Tx DMA channel is not open\r\n",
                xWriteBufferLen);
        return pdFALSE;
    }
    int32_t status = EnetApp_transmitPkt(payloadMsg, destAddr, dmaChNum);
    if (status)
        strncpy(pcWriteBuffer, "Failed\r\n", xWriteBufferLen);
    else
        strncpy(pcWriteBuffer, "Done\r\n", xWriteBufferLen);
    return pdFALSE;
}

BaseType_t EnetCLI_receivePkt(char *pcWriteBuffer, size_t xWriteBufferLen,
        const char *pcCommandString)
{
    char *pcParameter;
    BaseType_t paramLen;
    uint8_t dmaChNum = 0;
    pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
            &paramLen);
    if (pcParameter != NULL)
    {
        dmaChNum = atoi(pcParameter);
    }
    if (dmaChNum >= EnetApp_Inst.numRxDmaCh)
    {
        strncpy(pcWriteBuffer, "Rx DMA channel is not open\r\n",
                xWriteBufferLen);
        return pdFALSE;
    }
    else if (EnetApp_Inst.rxRunFlag[dmaChNum] != 0)
    {
        strncpy(pcWriteBuffer,
                "Rx DMA channel is already listening for packets\r\n",
                xWriteBufferLen);
        return pdFALSE;
    }
    EnetApp_createRxTask(dmaChNum);
    return pdFALSE;
}

BaseType_t EnetCLI_stopRxPkt(char *pcWriteBuffer, size_t xWriteBufferLen,
        const char *pcCommandString)
{
    char *pcParameter;
    BaseType_t paramLen;
    uint8_t dmaChNum = 0;
    pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
            &paramLen);
    if (pcParameter != NULL)
    {
        dmaChNum = atoi(pcParameter);
    }
    if (dmaChNum >= EnetApp_Inst.numRxDmaCh)
    {
        strncpy(pcWriteBuffer, "Rx DMA channel is not open\r\n",
                xWriteBufferLen);
        return pdFALSE;
    }
    else if (EnetApp_Inst.rxRunFlag[dmaChNum] == 0)
    {
        strncpy(pcWriteBuffer,
                "Rx DMA channel is not listening for any packets\r\n",
                xWriteBufferLen);
        return pdFALSE;
    }
    EnetApp_Inst.rxRunFlag[dmaChNum] = -1;
    return pdFALSE;
}

BaseType_t EnetCLI_dumpRxBuffer(char *pcWriteBuffer, size_t xWriteBufferLen,
        const char *pcCommandString)
{
    int8_t dmaChNum = 0;
    char *pcParameter;
    BaseType_t paramLen;
    pcParameter = (char*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
            &paramLen);
    if (pcParameter != NULL)
    {
        dmaChNum = atoi(pcParameter);
    }
    if (dmaChNum >= EnetApp_Inst.numRxDmaCh)
    {
        strncpy(pcWriteBuffer, "Rx DMA channel is not open\r\n",
                xWriteBufferLen);
        return pdFALSE;
    }
    else if (EnetApp_Inst.rxRunFlag[dmaChNum] != 0)
    {
        strncpy(pcWriteBuffer,
                "Rx DMA channel is still listening for packets\r\n",
                xWriteBufferLen);
        return pdFALSE;
    }
    if (EnetApp_rxBuffer[dmaChNum][EnetApp_readBufferPt[dmaChNum]] != NULL)
    {
        strncpy(pcWriteBuffer, "----PACKET ENDS HERE----\r\n\n",
                xWriteBufferLen);
        EnetAppUtils_printFrame(
                EnetApp_rxBuffer[dmaChNum][EnetApp_readBufferPt[dmaChNum]],
                EnetApp_payloadLen[dmaChNum][EnetApp_readBufferPt[dmaChNum]]);
    }

    EnetApp_readBufferPt[dmaChNum] =
            EnetApp_readBufferPt[dmaChNum] == 3 ?
                    0 : EnetApp_readBufferPt[dmaChNum] + 1;
    if (EnetApp_readBufferPt[dmaChNum] == EnetApp_writeBufferPt[dmaChNum])
        return pdFALSE;
    else
        return pdTRUE;
}

BaseType_t EnetCLI_dispCpswStats(char *pcWriteBuffer, size_t xWriteBufferLen,
        const char *pcCommandString)
{
    EnetApp_showCpswStats();
    return pdFALSE;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static int8_t EnetApp_init(void)
{
    int32_t status;
    EnetApp_HandleInfo handleInfo;

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &EnetApp_Inst.enetType,
            &EnetApp_Inst.instId);

    EnetApp_getEnetInstMacInfo(EnetApp_Inst.enetType, EnetApp_Inst.instId,
            EnetApp_Inst.macPort, &EnetApp_Inst.numMacPorts);

    EnetApp_Inst.macMode = RGMII;
    EnetApp_Inst.boardId = ENETBOARD_CPB_ID;
    EnetApp_Inst.coreId = EnetSoc_getCoreId();
    EnetApp_Inst.numTxDmaCh = 0;
    EnetApp_Inst.numRxDmaCh = 0;

    EnetAppUtils_enableClocks(EnetApp_Inst.enetType, EnetApp_Inst.instId);
    EnetApp_driverInit();

    status = EnetApp_driverOpen(EnetApp_Inst.enetType, EnetApp_Inst.instId);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        return 1;
    }

    EnetApp_acquireHandleInfo(EnetApp_Inst.enetType, EnetApp_Inst.instId,
            &handleInfo);
    EnetApp_Inst.hEnet = handleInfo.hEnet;

    EnetPer_AttachCoreOutArgs attachInfo;
    EnetApp_coreAttach(EnetApp_Inst.enetType, EnetApp_Inst.instId,
            EnetApp_Inst.coreId, &attachInfo);
    EnetApp_Inst.coreKey = attachInfo.coreKey;

    EnetApp_Inst.initFlag = 1;
    return 0;
}

static void EnetApp_close(void)
{
    /* Close Enet DMA driver */
    EnetApp_closeDma();
    /*Release Handle Info*/
    EnetApp_releaseHandleInfo(EnetApp_Inst.enetType, EnetApp_Inst.instId);
    EnetApp_Inst.hEnet = NULL;

    EnetApp_driverDeInit();

    /* Disable peripheral clocks */
    EnetAppUtils_disableClocks(EnetApp_Inst.enetType, EnetApp_Inst.instId);
}

static int8_t EnetApp_isValidMacAddr(char *mac)
{
    uint32_t macIdx = 0;
    uint32_t digitCnt = 0;
    int32_t delimCnt = 0;
    while (*(mac + macIdx))
    {
        if (isxdigit(*(mac + macIdx)))
            digitCnt++;
        else if (*(mac + macIdx) == ':')
        {
            if (digitCnt == 0 || digitCnt / 2 - 1 != delimCnt)
                break;
            ++delimCnt;
        }
        else
            delimCnt = -1;
        ++macIdx;
    }
    return (digitCnt == 12 && delimCnt == 5);
}

static int8_t EnetApp_addUcastEntry(uint8_t *macAddr)
{
    CpswAle_SetUcastEntryInArgs setUcastInArgs;
    uint32_t entryIdx;
    Enet_IoctlPrms prms;
    int32_t status;
    /* ALE entry with "secure" bit cleared is required for loopback */
    setUcastInArgs.addr.vlanId = 0U;
    setUcastInArgs.info.portNum = CPSW_ALE_HOST_PORT_NUM;
    setUcastInArgs.info.blocked = false;
    setUcastInArgs.info.secure = false;
    setUcastInArgs.info.super = false;
    setUcastInArgs.info.ageable = false;
    setUcastInArgs.info.trunk = false;
    EnetUtils_copyMacAddr(&setUcastInArgs.addr.addr[0U], macAddr);
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

    ENET_IOCTL(EnetApp_Inst.hEnet, EnetApp_Inst.coreId,
            CPSW_ALE_IOCTL_ADD_UCAST, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to add unicast entry: %d\r\n", status);
        return 1;
    }
    EnetAppUtils_print("Added Unicast entry with MAC address: ");
    EnetAppUtils_printMacAddr(macAddr);
    return 0;
}

static int8_t EnetApp_addClassifierEntry(CpswAle_PolicerMatchParams prm,
        int8_t rxCh)
{
    CpswAle_SetPolicerEntryInArgs setPolicerInArgs;
    CpswAle_SetPolicerEntryOutArgs setPolicerOutArgs;
    Enet_IoctlPrms prms;
    int32_t status;

    memset(&setPolicerInArgs, 0, sizeof(setPolicerInArgs));
    setPolicerInArgs.policerMatch = prm;
    setPolicerInArgs.threadIdEn = true;
    setPolicerInArgs.threadId = rxCh + 1;
    setPolicerInArgs.peakRateInBitsPerSec = 0;
    setPolicerInArgs.commitRateInBitsPerSec = 0;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setPolicerInArgs, &setPolicerOutArgs);

    ENET_IOCTL(EnetApp_Inst.hEnet, EnetApp_Inst.coreId,
            CPSW_ALE_IOCTL_SET_POLICER, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to add classifier entry: %d\r\n", status);
        return 1;
    }
    EnetAppUtils_print("Added classifier entry\r\n");
    return 0;
}

static int8_t EnetApp_transmitPkt(char *payload, uint8_t *destMacAddr,
        int8_t dmaChNum)
{
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *pktInfo;
    EthFrame *frame;
    uint32_t txRetrievePktCnt;
    uint32_t pktCnt = 0;
    size_t pktLen = ETH_PAYLOAD_LEN;
    int32_t status = ENET_SOK;
    size_t payloadLen = strlen(payload);
    size_t txLen = 0;

    while (txLen < payloadLen)
    {
        /* Transmit a single packet */
        EnetQueue_initQ(&txSubmitQ);

        /* Dequeue one free TX Eth packet */
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(
                &EnetApp_Inst.txFreePktInfoQ[dmaChNum]);

        while (NULL != pktInfo)
        {
            pktCnt++;
            if (payloadLen - txLen < ETH_PAYLOAD_LEN)
                pktLen = payloadLen - txLen;
            /* Fill the TX Eth frame with test content */
            frame = (EthFrame*) pktInfo->sgList.list[0].bufPtr;
            memcpy(frame->hdr.dstMac, destMacAddr, ENET_MAC_ADDR_LEN);
            memcpy(frame->hdr.srcMac, &EnetApp_Inst.hostMacAddr[0U],
            ENET_MAC_ADDR_LEN);
            frame->hdr.etherType = Enet_htons(ETHERTYPE_EXPERIMENTAL1);
            memcpy(&frame->payload[0U], (payload + txLen), pktLen);

            pktInfo->sgList.list[0].segmentFilledLen = pktLen
                    + sizeof(EthFrameHeader);
            pktInfo->sgList.numScatterSegments = 1;
            pktInfo->chkSumInfo = 0U;
            pktInfo->appPriv = &EnetApp_Inst;
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
                    &EnetApp_Inst.txFreePktInfoQ[dmaChNum]);
        }

        while (0U != EnetQueue_getQCount(&txSubmitQ))
        {
            uint32_t txCnt = EnetQueue_getQCount(&txSubmitQ);
            status = EnetDma_submitTxPktQ(EnetApp_Inst.hTxCh[dmaChNum],
                    &txSubmitQ);
            SemaphoreP_pend(&EnetApp_Inst.txSemObj[dmaChNum],
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
                return 1;
            }
        }
    }

    EnetAppUtils_print(
            "Transmitted %d packets with payload \"%s\" using DMA channel %d to MAC ",
            pktCnt, payload, dmaChNum);
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
    status = EnetDma_retrieveTxPktQ(EnetApp_Inst.hTxCh[dmaChNum], &txFreeQ);
    if (status == ENET_SOK)
    {
        txFreeQCnt = EnetQueue_getQCount(&txFreeQ);

        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&txFreeQ);
        while (NULL != pktInfo)
        {
            EnetDma_checkPktState(&pktInfo->pktState, ENET_PKTSTATE_MODULE_APP,
                    ENET_PKTSTATE_APP_WITH_DRIVER,
                    ENET_PKTSTATE_APP_WITH_FREEQ);

            EnetQueue_enq(&EnetApp_Inst.txFreePktInfoQ[dmaChNum],
                    &pktInfo->node);
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print(
                "EnetApp_retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n",
                status);
    }

    return txFreeQCnt;
}

static void EnetApp_createRxTask(int8_t dmaChNum)
{
    TaskP_Params taskParams;
    int32_t status;
    EnetApp_Inst.rxRunFlag[dmaChNum] = 1;
    TaskP_Params_init(&taskParams);
    taskParams.priority = 3U;
    taskParams.stack = EnetApp_rxTaskStack[dmaChNum];
    taskParams.stackSize = sizeof(EnetApp_rxTaskStack[dmaChNum]);
    taskParams.args = (void*) &dmaChNum;
    taskParams.name = "Rx Task";
    taskParams.taskMain = &EnetApp_recievePkt;

    status = TaskP_construct(&EnetApp_Inst.rxTaskObj[dmaChNum], &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
    EnetAppUtils_print("Listening to incoming packets at DMA channel %d\r\n",
            dmaChNum);
}

static void EnetApp_recievePkt(void *args)
{
    int8_t dmaChNum = *(int8_t*) args;
    EnetDma_Pkt *pktInfo;
    uint32_t rxReadyCnt;
    int32_t status = ENET_SOK;
    uint32_t rxPktCnt = 0;

    while (EnetApp_Inst.rxRunFlag[dmaChNum] != -1)
    {
        status = SemaphoreP_pend(&EnetApp_Inst.rxSemObj[dmaChNum],
                ClockP_usecToTicks(500000));
        if (status != SystemP_SUCCESS)
            continue;
        /* Get the packets received so far */
        rxReadyCnt = EnetApp_retrieveRxPkts(dmaChNum);
        if (rxReadyCnt > 0U)
        {
            /* Consume the received packets and release them */
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(
                    &EnetApp_Inst.rxReadyQ[dmaChNum]);
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
                EnetApp_readBufferPt[dmaChNum] =
                        EnetApp_writeBufferPt[dmaChNum];

                EnetQueue_enq(&EnetApp_Inst.rxFreeQ[dmaChNum], &pktInfo->node);
                pktInfo = (EnetDma_Pkt*) EnetQueue_deq(
                        &EnetApp_Inst.rxReadyQ[dmaChNum]);
            }

            /*Submit now processed buffers */
            if (status == ENET_SOK)
            {
                EnetAppUtils_validatePacketState(
                        &EnetApp_Inst.rxFreeQ[dmaChNum],
                        ENET_PKTSTATE_APP_WITH_FREEQ,
                        ENET_PKTSTATE_APP_WITH_DRIVER);

                EnetDma_submitRxPktQ(EnetApp_Inst.hRxCh[dmaChNum],
                        &EnetApp_Inst.rxFreeQ[dmaChNum]);
            }
        }
    }
    EnetApp_Inst.rxRunFlag[dmaChNum] = 0;
    EnetAppUtils_print(
            "Stopped listening at DMA channel %d... Recieved %d packets\r\n",
            dmaChNum, rxPktCnt);
    TaskP_destruct(&EnetApp_Inst.rxTaskObj[dmaChNum]);
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
    status = EnetDma_retrieveRxPktQ(EnetApp_Inst.hRxCh[dmaChNum], &rxReadyQ);
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

            EnetQueue_enq(&EnetApp_Inst.rxReadyQ[dmaChNum], &pktInfo->node);
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
        }
    }
    else
    {
        EnetAppUtils_print("receivePkts() failed to retrieve pkts: %d\r\n",
                status);
    }

    return rxReadyCnt;
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
    EnetQueue_initQ(&EnetApp_Inst.txFreePktInfoQ[dmaChNum]);

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < EnetApp_pktPerTxCh[dmaChNum]; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&EnetApp_Inst,
                ENETDMA_CACHELINE_ALIGNMENT, ENET_ARRAYSIZE(scatterSegments),
                scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState,
                ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&EnetApp_Inst.txFreePktInfoQ[dmaChNum], &pPktInfo->node);
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

    EnetQueue_initQ(&EnetApp_Inst.rxFreeQ[dmaChNum]);
    EnetQueue_initQ(&EnetApp_Inst.rxReadyQ[dmaChNum]);
    EnetQueue_initQ(&rxReadyQ);

    for (i = 0U; i < EnetApp_pktPerRxCh[dmaChNum]; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&EnetApp_Inst,
                ENETDMA_CACHELINE_ALIGNMENT, ENET_ARRAYSIZE(scatterSegments),
                scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState,
                ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetQueue_enq(&EnetApp_Inst.rxFreeQ[dmaChNum], &pPktInfo->node);
    }

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(EnetApp_Inst.hRxCh[dmaChNum], &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);
    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&EnetApp_Inst.rxFreeQ[dmaChNum],
            ENET_PKTSTATE_APP_WITH_FREEQ, ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(EnetApp_Inst.hRxCh[dmaChNum],
            &EnetApp_Inst.rxFreeQ[dmaChNum]);

    while (0U != EnetQueue_getQCount(&EnetApp_Inst.rxFreeQ[dmaChNum]))
        EnetDma_submitRxPktQ(EnetApp_Inst.hRxCh[dmaChNum],
                &EnetApp_Inst.rxFreeQ[dmaChNum]);
}

static int32_t EnetApp_openTxDma(void)
{
    int32_t status = ENET_SOK;

    /* Intialize semaphores */
    status = SemaphoreP_constructBinary(
            &EnetApp_Inst.txSemObj[EnetApp_Inst.numTxDmaCh], 0);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("Failed to create semaphores: %d\r\n", status);
        return 1;
    }

    /* Open the CPSW TX flow */
    EnetApp_GetDmaHandleInArgs txInArgs;
    EnetApp_GetTxDmaHandleOutArgs txChInfo;

    txInArgs.cbArg = &EnetApp_Inst.txSemObj[EnetApp_Inst.numTxDmaCh];
    txInArgs.notifyCb = EnetApp_txIsrFxn;

    EnetApp_getTxDmaHandle(EnetApp_Inst.numTxDmaCh, &txInArgs, &txChInfo);
    EnetApp_Inst.hTxCh[EnetApp_Inst.numTxDmaCh] = txChInfo.hTxCh;
    if (NULL == EnetApp_Inst.hTxCh[EnetApp_Inst.numTxDmaCh])
    {
        EnetAppUtils_print("Couldn't open Tx channel\r\n");
        return 1;
    }

    EnetApp_initTxFreePktQ(EnetApp_Inst.numTxDmaCh);

    EnetAppUtils_print("Opened new Tx DMA channel with identifier %d\r\n",
            EnetApp_Inst.numTxDmaCh);
    EnetApp_Inst.numTxDmaCh++;
    return 0;
}

static int32_t EnetApp_openRxDma(void)
{
    int32_t status = ENET_SOK;

    /* Intialize semaphores */
    status = SemaphoreP_constructBinary(
            &EnetApp_Inst.rxSemObj[EnetApp_Inst.numRxDmaCh], 0);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("Failed to create semaphores: %d\r\n", status);
        return 1;
    }
    /* Open the CPSW RX flow  */
    EnetApp_GetRxDmaHandleOutArgs rxChInfo;
    EnetApp_GetDmaHandleInArgs rxInArgs;

    rxInArgs.cbArg = &EnetApp_Inst.rxSemObj[EnetApp_Inst.numRxDmaCh];
    rxInArgs.notifyCb = EnetApp_rxIsrFxn;

    EnetApp_getRxDmaHandle(EnetApp_Inst.numRxDmaCh, &rxInArgs, &rxChInfo);
    EnetApp_Inst.hRxCh[EnetApp_Inst.numRxDmaCh] = rxChInfo.hRxCh;
    if (NULL == EnetApp_Inst.hRxCh[EnetApp_Inst.numRxDmaCh])
    {
        EnetAppUtils_print("Couldn't open Rx channel\r\n");
        return 1;
    }

    if (EnetApp_Inst.numRxDmaCh == 0)
        EnetUtils_copyMacAddr(EnetApp_Inst.hostMacAddr,
                &rxChInfo.macAddr[0][0]);

    EnetApp_initRxReadyPktQ(EnetApp_Inst.numRxDmaCh);

    EnetAppUtils_print("Opened new Rx DMA channel with identifier %d\r\n",
            EnetApp_Inst.numRxDmaCh);
    EnetApp_Inst.numRxDmaCh++;
    return status;
}

static void EnetApp_closeDma(void)
{
    for (uint8_t i = EnetApp_Inst.numTxDmaCh; i > 0; i--)
    {
        EnetDma_PktQ fqPktInfoQ;
        EnetDma_PktQ cqPktInfoQ;

        /* Close TX channel */
        EnetQueue_initQ(&fqPktInfoQ);
        EnetQueue_initQ(&cqPktInfoQ);

        EnetApp_closeTxDma(i - 1, EnetApp_Inst.hEnet, EnetApp_Inst.coreKey,
                EnetApp_Inst.coreId, &fqPktInfoQ, &cqPktInfoQ);

        EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
        EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

        EnetAppUtils_freePktInfoQ(&EnetApp_Inst.txFreePktInfoQ[i - 1]);

        SemaphoreP_destruct(&EnetApp_Inst.txSemObj[i - 1]);
    }

    for (uint8_t i = EnetApp_Inst.numRxDmaCh; i > 0; i--)
    {
        EnetDma_PktQ fqPktInfoQ;
        EnetDma_PktQ cqPktInfoQ;

        EnetQueue_initQ(&fqPktInfoQ);
        EnetQueue_initQ(&cqPktInfoQ);

        /* There should not be any ready packet */
        EnetAppUtils_assert(
                0U == EnetQueue_getQCount(&EnetApp_Inst.rxReadyQ[i - 1]));

        /* Close RX channel */
        EnetApp_closeRxDma(i - 1, EnetApp_Inst.hEnet, EnetApp_Inst.coreKey,
                EnetApp_Inst.coreId, &fqPktInfoQ, &cqPktInfoQ);

        EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
        EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

        EnetAppUtils_freePktInfoQ(&EnetApp_Inst.rxFreeQ[i - 1]);

        SemaphoreP_destruct(&EnetApp_Inst.rxSemObj[i - 1]);
    }
}

static void EnetApp_showCpswStats(void)
{
    Enet_IoctlPrms prms;
    CpswStats_PortStats portStats;
    int32_t status;

    /* Show host port statistics */
    ENET_IOCTL_SET_OUT_ARGS(&prms, &portStats);
    ENET_IOCTL(EnetApp_Inst.hEnet, EnetApp_Inst.coreId,
            ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\r\n Port 0 Statistics\r\n");
        EnetAppUtils_print("-----------------------------------------\r\n");
        EnetAppUtils_printHostPortStats2G((CpswStats_HostPort_2g*) &portStats);
        EnetAppUtils_print("\r\n");
    }
    else
    {
        EnetAppUtils_print("Failed to get host stats: %d\r\n", status);
    }

    /* Show MAC port statistics */
    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &EnetApp_Inst.macPort[0], &portStats);
        ENET_IOCTL(EnetApp_Inst.hEnet, EnetApp_Inst.coreId,
                ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("\r\n Port 1 Statistics\r\n");
            EnetAppUtils_print("-----------------------------------------\r\n");
            EnetAppUtils_printMacPortStats2G(
                    (CpswStats_MacPort_2g*) &portStats);
            EnetAppUtils_print("\r\n");
        }
        else
        {
            EnetAppUtils_print("Failed to get MAC stats: %d\r\n", status);
        }
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &EnetApp_Inst.macPort[1], &portStats);
        ENET_IOCTL(EnetApp_Inst.hEnet, EnetApp_Inst.coreId,
                ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("\r\n Port 2 Statistics\r\n");
            EnetAppUtils_print("-----------------------------------------\r\n");
            EnetAppUtils_printMacPortStats2G(
                    (CpswStats_MacPort_2g*) &portStats);
            EnetAppUtils_print("\r\n");
        }
        else
        {
            EnetAppUtils_print("Failed to get MAC stats: %d\r\n", status);
        }
    }
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId,
        Cpsw_Cfg *cpswCfg)
{
    CpswHostPort_Cfg *hostPortCfg = &cpswCfg->hostPortCfg;
    CpswAle_Cfg *aleCfg = &cpswCfg->aleCfg;
    CpswCpts_Cfg *cptsCfg = &cpswCfg->cptsCfg;

    /* Set Enet global runtime log level */
    Enet_setTraceLevel(ENET_TRACE_DEBUG);

    /* Peripheral config */
    cpswCfg->vlanCfg.vlanAware = false;

    /* Host port config */
    hostPortCfg->removeCrc = true;
    hostPortCfg->padShortPacket = true;
    hostPortCfg->passCrcErrors = true;

    /* ALE config */
    aleCfg->modeFlags = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn = true;
    aleCfg->agingCfg.agingPeriodInMs = 1000;
    aleCfg->nwSecCfg.vid0ModeEn = true;
    aleCfg->vlanCfg.aleVlanAwareMode = false;
    aleCfg->vlanCfg.cpswVlanAwareMode = false;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->policerGlobalCfg.policingEn = true;
    aleCfg->policerGlobalCfg.redDropEn = false;
    aleCfg->policerGlobalCfg.yellowDropEn = false;
    aleCfg->policerGlobalCfg.policerNoMatchMode =
            CPSW_ALE_POLICER_NOMATCH_MODE_GREEN;

    /* CPTS config */
    /* Note: Timestamping and MAC loopback are not supported together because of
     * IP limitation, so disabling timestamping for this application */
    cptsCfg->hostRxTsEn = false;
}

void EnetApp_initLinkArgs(Enet_Type enetType, uint32_t instId,
        EnetPer_PortLinkCfg *linkArgs, Enet_MacPort macPort)
{
    EnetBoard_EthPort ethPort;
    CpswMacPort_Cfg *cpswMacCfg;
    EnetMacPort_LinkCfg *linkCfg = &linkArgs->linkCfg;
    EnetMacPort_Interface *mii = &linkArgs->mii;
    EnetPhy_Cfg *phyCfg = &linkArgs->phyCfg;
    int32_t status = ENET_SOK;

    /* Setup board for requested Ethernet port */
    ethPort.enetType = EnetApp_Inst.enetType;
    ethPort.instId = EnetApp_Inst.instId;
    ethPort.macPort = macPort;
    ethPort.boardId = EnetApp_Inst.boardId;
    EnetApp_macMode2MacMii(EnetApp_Inst.macMode, &ethPort.mii);

    status = EnetBoard_setupPorts(&ethPort, 1U);
    EnetAppUtils_assert(status == ENET_SOK);

    /* Set port link params */
    linkArgs->macPort = macPort;

    cpswMacCfg = linkArgs->macCfg;

    EnetApp_macMode2MacMii(EnetApp_Inst.macMode, mii);

    const EnetBoard_PhyCfg *boardPhyCfg = NULL;

    /* Set PHY configuration params */
    EnetPhy_initCfg(phyCfg);

    if (status == ENET_SOK)
    {
        boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
        if (boardPhyCfg != NULL)
        {
            phyCfg->phyAddr = boardPhyCfg->phyAddr;
            phyCfg->isStrapped = boardPhyCfg->isStrapped;
            phyCfg->skipExtendedCfg = boardPhyCfg->skipExtendedCfg;
            phyCfg->extendedCfgSize = boardPhyCfg->extendedCfgSize;
            memcpy(phyCfg->extendedCfg, boardPhyCfg->extendedCfg,
                    phyCfg->extendedCfgSize);
        }
        else
        {
            EnetAppUtils_print("PHY info not found\r\n");
            EnetAppUtils_assert(false);
        }

        linkCfg->speed = EnetApp_linkSpeed;
        linkCfg->duplexity = ENET_DUPLEX_FULL;
    }

    phyCfg->loopbackEn = 0;
    cpswMacCfg->loopbackEn = 0;
}

static void EnetApp_macMode2MacMii(emac_mode macMode,
        EnetMacPort_Interface *mii)
{
    switch (macMode)
    {
        case MII:
            mii->layerType = ENET_MAC_LAYER_MII;
            mii->sublayerType = ENET_MAC_SUBLAYER_STANDARD;
            mii->variantType = ENET_MAC_VARIANT_NONE;
            break;
        case RMII:
            mii->layerType = ENET_MAC_LAYER_MII;
            mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
            mii->variantType = ENET_MAC_VARIANT_NONE;
            break;

        case RGMII:
            mii->layerType = ENET_MAC_LAYER_GMII;
            mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
            mii->variantType = ENET_MAC_VARIANT_FORCED;
            break;
        default:
            EnetAppUtils_print("Invalid MAC mode: %u\r\n", macMode);
            EnetAppUtils_assert(false);
            break;
    }
}

static int32_t EnetApp_waitForLinkUp(Enet_MacPort macPort)
{
    uint8_t timeout = 0;
    Enet_IoctlPrms prms;
    bool linked = false;
    int32_t status = ENET_SOK;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);

    while (!linked)
    {
        ENET_IOCTL(EnetApp_Inst.hEnet, EnetApp_Inst.coreId,
                ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to get port %u's link status: %d\r\n",
                    ENET_MACPORT_ID(macPort), status);
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
    return status;
}

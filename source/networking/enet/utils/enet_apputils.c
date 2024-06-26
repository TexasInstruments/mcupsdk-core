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

/*!
 * \file     enet_apputils.c
 *
 * \brief    Common Enet application utility used in all Enet examples.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>

#include <drivers/hw_include/cslr_soc.h>
#include <csl_cpswitch.h>

#include <enet.h>
#include <include/per/cpsw.h>
#if (ENET_ENABLE_PER_ICSSG == 1)
#include <include/per/icssg.h>
#endif

#include <drivers/uart.h>
#include <kernel/dpl/CacheP.h>

#include "include/enet_apputils.h"
#include "include/enet_appboardutils.h"
#include "include/enet_appsoc.h"
#include "include/enet_apprm.h"
#include <kernel/dpl/CacheP.h>
#include <kernel/nortos/dpl/common/printf.h>



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

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetAppUtils_cacheWb(const void *addr,
                          int32_t size)
{
    bool isCacheCoherent = Enet_isCacheCoherent();

    if (isCacheCoherent == false)
    {
        CacheP_wb((void *)addr, size, CacheP_TYPE_ALLD);
    }
}

void EnetAppUtils_cacheInv(const void *addr,
                           int32_t size)
{
    bool isCacheCoherent = Enet_isCacheCoherent();

    if (isCacheCoherent == FALSE)
    {
        CacheP_inv((void *)addr, size, CacheP_TYPE_ALLD);
    }
}

void EnetAppUtils_cacheWbInv(const void *addr,
                             int32_t size)
{
    bool isCacheCoherent = Enet_isCacheCoherent();

    if (isCacheCoherent == false)
    {
        CacheP_wbInv((void *)addr, size, CacheP_TYPE_ALLD);
    }
}

void EnetAppUtils_vprint(const char *pcString,
                         va_list args)
{
    char printBuffer[ENET_CFG_PRINT_BUF_LEN];

    vsnprintf(printBuffer, sizeof(printBuffer), pcString, args);

#if defined(UART_ENABLED) && defined(ENETAPPUTILS_UART_ALLOWED)
    UART_printf("%s", printBuffer);
#else
    if (TRUE == EnetAppUtils_isPrintSupported())
    {
        DebugP_log("%s",printBuffer);
    }
#endif
}

void EnetAppUtils_print(const char *pcString,
                        ...)
{
    char printBuffer[ENET_CFG_PRINT_BUF_LEN];
    va_list arguments;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if (ENET_CFG_PRINT_BUF_LEN < strlen(pcString))
    {
        /* Don't use EnetAppUtils_assert as it uses EnetAppUtils_print function  */
        assert(false);
    }
#endif

    /* Start the varargs processing */
    va_start(arguments, pcString);
    vsnprintf(printBuffer, sizeof(printBuffer), pcString, arguments);

    {
#if defined(UART_ENABLED) && defined(ENETAPPUTILS_UART_ALLOWED)
        UART_printf("%s", printBuffer);
#else
        if (TRUE == EnetAppUtils_isPrintSupported())
        {
            #if defined (ENABLE_ENET_LOG)
            DebugP_log("%s",printBuffer);
            #endif
        }
#endif
    }

    /* End the varargs processing */
    va_end(arguments);
}

char EnetAppUtils_getChar(void)
{
    char ch;

#if defined(UART_ENABLED) && defined(ENETAPPUTILS_UART_ALLOWED)
    ch = UART_getc();
#else
    scanf("%c", &ch);
#endif

    return ch;
}

int32_t EnetAppUtils_getNum(void)
{
    int32_t num;

#if defined(UART_ENABLED) && defined(ENETAPPUTILS_UART_ALLOWED)
    UART_scanFmt("%d", &num);
#else
    scanf("%" SCNd32, &num);
#endif

    return num;
}

uint32_t EnetAppUtils_getHex(void)
{
    uint32_t num;

#if defined(UART_ENABLED) && defined(ENETAPPUTILS_UART_ALLOWED)
    UART_scanFmt("%x", &num);
#else
    scanf("%" SCNx32, &num);
#endif

    return num;
}

uint32_t EnetAppUtils_randFxn(uint32_t min,
                              uint32_t max)
{
    return (rand() % (max - min + 1)) + min;
}

void EnetAppUtils_waitEmuConnect(void)
{
    volatile uint32_t emuWaitFlag = 0x1U;

    while (emuWaitFlag)
    {
        ;
    }
}

uint32_t EnetAppUtils_isPrintSupported(void)
{
    uint32_t retVal = TRUE;

    return(retVal);
}

static void EnetAppUtils_printStatsNonZero(const char *pcString,
                                           uint64_t statVal)
{
    if (0U != statVal)
    {
        EnetAppUtils_print(pcString, statVal);
    }
}

static void EnetAppUtils_printStatsWithIdxNonZero(const char *pcString,
                                                  uint32_t idx,
                                                  uint64_t statVal)
{
    if (0U != statVal)
    {
        EnetAppUtils_print(pcString, idx, statVal);
    }
}


static void EnetAppUtils_printDmaDescStats(EnetDma_DmaDescStats *descstats)
{
    // TODO when DMA stats are enabled.
}

static void EnetAppUtils_printCbStats(EnetDma_CbStats *cbStats)
{
    uint32_t i;

    EnetAppUtils_printStatsNonZero("Data Notify Count          = %llu\r\n", cbStats->dataNotifyCnt);
    EnetAppUtils_printStatsNonZero("Zero Notify Count          = %llu\r\n", cbStats->zeroNotifyCnt);
    EnetAppUtils_printStatsNonZero("Total Packets Count        = %llu\r\n", cbStats->totalPktCnt);
    EnetAppUtils_printStatsNonZero("Total Cycles Count         = %llu\r\n", cbStats->totalCycleCnt);
    EnetAppUtils_printStatsNonZero("Packets per Notify Max     = %llu\r\n", cbStats->pktsPerNotifyMax);
    for (i = 0U; i < ENET_DMA_STATS_HISTORY_CNT; i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("Packets per Notify[%d] = %llu\r\n", i, cbStats->pktsPerNotify[i]);
    }

    EnetAppUtils_printStatsNonZero("Cycles per Notify Max      = %llu\r\n", cbStats->cycleCntPerNotifyMax);
    for (i = 0U; i < ENET_DMA_STATS_HISTORY_CNT; i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("Cycles per Notify[%d]  = %llu\r\n", i, cbStats->cycleCntPerNotify[i]);
    }

    EnetAppUtils_printStatsNonZero("Cycles per Packet Max      = %llu\r\n", cbStats->cycleCntPerPktMax);
    for (i = 0U; i < ENET_DMA_STATS_HISTORY_CNT; i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("Cycles per Packet[%d]  = %llu\r\n", i, cbStats->cycleCntPerPkt[i]);
    }

    for (i = 0U; i < ENET_DMA_STATS_HISTORY_CNT; i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("Ready Desc Q Count[%d] = %llu\r\n", i, cbStats->readyDmaDescQCnt[i]);
    }
}

static void EnetAppUtils_printTxChStats(EnetDma_TxChStats *stats)
{
    EnetAppUtils_print(" TX Channel Statistics\r\n");
    EnetAppUtils_print("-----------------------------------------\r\n");
    EnetAppUtils_printCbStats(&stats->submitPktStats);
    EnetAppUtils_printCbStats(&stats->retrievePktStats);
    EnetAppUtils_printDmaDescStats(&stats->dmaDescStats);

    EnetAppUtils_printStatsNonZero("TX Submit Packet EnQ count           = %llu\r\n", stats->txSubmitPktEnq);
    EnetAppUtils_printStatsNonZero("TX Submit Packet Underflow           = %llu\r\n", stats->txSubmitPktOverFlowCnt);
    EnetAppUtils_printStatsNonZero("TX Submit Packet DeQ count           = %llu\r\n", stats->txRetrievePktDeq);
}

static void EnetAppUtils_printRxChStats(EnetDma_RxChStats *stats)
{
    EnetAppUtils_print(" RX Channel Statistics\r\n");
    EnetAppUtils_print("-----------------------------------------\r\n");
    EnetAppUtils_printCbStats(&stats->submitPktStats);
    EnetAppUtils_printCbStats(&stats->retrievePktStats);
    EnetAppUtils_printDmaDescStats(&stats->dmaDescStats);

    EnetAppUtils_printStatsNonZero("RX Submit Packet EnQ count           = %llu\r\n", stats->rxSubmitPktEnq);
    EnetAppUtils_printStatsNonZero("RX Submit Packet Underflow           = %llu\r\n", stats->rxSubmitPktUnderFlowCnt);
    EnetAppUtils_printStatsNonZero("RX Submit Packet DeQ count           = %llu\r\n", stats->rxRetrievePktDeq);
}

int32_t EnetAppUtils_showTxChStats(EnetDma_TxChHandle hTxCh)
{
    int32_t retVal = ENET_SOK;

    EnetDma_TxChStats txChStats;

    retVal = EnetDma_getTxChStats(hTxCh, &txChStats);
    if (ENET_ENOTSUPPORTED != retVal)
    {
        EnetAppUtils_printTxChStats(&txChStats);
        EnetDma_resetTxChStats(hTxCh);
    }

    return retVal;
}

int32_t EnetAppUtils_showRxChStats(EnetDma_RxChHandle hRxCh)
{
    int32_t retVal = ENET_SOK;

    EnetDma_RxChStats rxChStats;

    retVal = EnetDma_getRxChStats(hRxCh, &rxChStats);
    if (ENET_ENOTSUPPORTED != retVal)
    {
        EnetAppUtils_printRxChStats(&rxChStats);
        EnetDma_resetRxChStats(hRxCh);
    }

    return retVal;
}

void EnetAppUtils_printMacAddr(uint8_t macAddr[])
{
    EnetAppUtils_print("%02x:%02x:%02x:%02x:%02x:%02x\r\r\n",
                       macAddr[0] & 0xFF,
                       macAddr[1] & 0xFF,
                       macAddr[2] & 0xFF,
                       macAddr[3] & 0xFF,
                       macAddr[4] & 0xFF,
                       macAddr[5] & 0xFF);
}

void EnetAppUtils_printFrame(EthFrame *frame,
                             uint32_t len)
{
    uint8_t *payload;
    uint32_t i;

    EnetAppUtils_print("Dst addr : ");
    EnetAppUtils_printMacAddr(&frame->hdr.dstMac[0]);

    EnetAppUtils_print("Src addr : ");
    EnetAppUtils_printMacAddr(&frame->hdr.srcMac[0]);

    if (frame->hdr.etherType == Enet_htons(ETHERTYPE_VLAN_TAG))
    {
        EthVlanFrame *vlanFrame = (EthVlanFrame *)frame;

        EnetAppUtils_print("TPID     : 0x%04x\r\n", Enet_ntohs(vlanFrame->hdr.tpid) & 0xFFFFU);
        EnetAppUtils_print("Priority : %d\r\n", (Enet_ntohs(vlanFrame->hdr.tci) & 0xFFFFU) >> 13);
        EnetAppUtils_print("VLAN Id  : %d\r\n", Enet_ntohs(vlanFrame->hdr.tci) & 0xFFFU);
        EnetAppUtils_print("EtherType: 0x%04x\r\n", Enet_ntohs(vlanFrame->hdr.etherType) & 0xFFFFU);
        payload = vlanFrame->payload;
        len    -= ETH_VLAN_TAG_LEN;
    }
    else
    {
        EnetAppUtils_print("EtherType: 0x%04x\r\n", Enet_ntohs(frame->hdr.etherType) & 0xFFFFU);
        payload = frame->payload;
    }

    EnetAppUtils_print("Payload  : ");
    for (i = 0; i < len; i++)
    {
        EnetAppUtils_print("0x%02x ", payload[i]);
        if (i && (((i + 1) % OCTETS_PER_ROW) == 0))
        {
            EnetAppUtils_print("\r\n           ");
        }
    }

    if (len && ((len % OCTETS_PER_ROW) != 0))
    {
        EnetAppUtils_print("\r\n");
    }

    EnetAppUtils_print("\r\n");
}

#if (ENET_ENABLE_PER_ICSSG == 1)
void EnetAppUtils_printIcssgMacPortStats(IcssgStats_MacPort *st,
                                         bool printFixedCounters)
{
    EnetAppUtils_printStatsNonZero("  rxGoodFrames            = %llu\r\n", (uint64_t)st->rxGoodFrames);
    EnetAppUtils_printStatsNonZero("  rxBCastFrames           = %llu\r\n", (uint64_t)st->rxBCastFrames);
    EnetAppUtils_printStatsNonZero("  rxMCastFrames           = %llu\r\n", (uint64_t)st->rxMCastFrames);
    EnetAppUtils_printStatsNonZero("  rxCRCErrors             = %llu\r\n", (uint64_t)st->rxCRCErrors);
    EnetAppUtils_printStatsNonZero("  rxMIIErrors             = %llu\r\n", (uint64_t)st->rxMIIErrors);
    EnetAppUtils_printStatsNonZero("  rxOddNibbleFrame        = %llu\r\n", (uint64_t)st->rxOddNibbleFrame);
    if (printFixedCounters)
    {
        EnetAppUtils_printStatsNonZero("  rxMaxSizeFrame          = %llu\r\n", (uint64_t)st->rxMaxSizeFrame);
    }
    EnetAppUtils_printStatsNonZero("  rxMaxSizeErrFrame       = %llu\r\n", (uint64_t)st->rxMaxSizeErrFrame);
    if (printFixedCounters)
    {
        EnetAppUtils_printStatsNonZero("  rxMinSizeFrame          = %llu\r\n", (uint64_t)st->rxMinSizeFrame);
    }
    EnetAppUtils_printStatsNonZero("  rxMinSizeErrFrame       = %llu\r\n", (uint64_t)st->rxMinSizeErrFrame);
    EnetAppUtils_printStatsNonZero("  rxOverrunFrame          = %llu\r\n", (uint64_t)st->rxOverrunFrame);
    EnetAppUtils_printStatsNonZero("  rxClass0                = %llu\r\n", (uint64_t)st->rxClass0);
    EnetAppUtils_printStatsNonZero("  rxClass1                = %llu\r\n", (uint64_t)st->rxClass1);
    EnetAppUtils_printStatsNonZero("  rxClass2                = %llu\r\n", (uint64_t)st->rxClass2);
    EnetAppUtils_printStatsNonZero("  rxClass3                = %llu\r\n", (uint64_t)st->rxClass3);
    EnetAppUtils_printStatsNonZero("  rxClass4                = %llu\r\n", (uint64_t)st->rxClass4);
    EnetAppUtils_printStatsNonZero("  rxClass5                = %llu\r\n", (uint64_t)st->rxClass5);
    EnetAppUtils_printStatsNonZero("  rxClass6                = %llu\r\n", (uint64_t)st->rxClass6);
    EnetAppUtils_printStatsNonZero("  rxClass7                = %llu\r\n", (uint64_t)st->rxClass7);
    EnetAppUtils_printStatsNonZero("  rxClass8                = %llu\r\n", (uint64_t)st->rxClass8);
    EnetAppUtils_printStatsNonZero("  rxClass9                = %llu\r\n", (uint64_t)st->rxClass9);
    EnetAppUtils_printStatsNonZero("  rxClass10               = %llu\r\n", (uint64_t)st->rxClass10);
    EnetAppUtils_printStatsNonZero("  rxClass11               = %llu\r\n", (uint64_t)st->rxClass11);
    EnetAppUtils_printStatsNonZero("  rxClass12               = %llu\r\n", (uint64_t)st->rxClass12);
    EnetAppUtils_printStatsNonZero("  rxClass13               = %llu\r\n", (uint64_t)st->rxClass13);
    EnetAppUtils_printStatsNonZero("  rxClass14               = %llu\r\n", (uint64_t)st->rxClass14);
    EnetAppUtils_printStatsNonZero("  rxClass15               = %llu\r\n", (uint64_t)st->rxClass15);
    EnetAppUtils_printStatsNonZero("  rxSMDFragErr            = %llu\r\n", (uint64_t)st->rxSMDFragErr);
    if (printFixedCounters)
    {
        EnetAppUtils_printStatsNonZero("  rxBucket1SizeConfig     = %llu\r\n", (uint64_t)st->rxBucket1SizeConfig);
        EnetAppUtils_printStatsNonZero("  rxBucket2SizeConfig     = %llu\r\n", (uint64_t)st->rxBucket2SizeConfig);
        EnetAppUtils_printStatsNonZero("  rxBucket3SizeConfig     = %llu\r\n", (uint64_t)st->rxBucket3SizeConfig);
        EnetAppUtils_printStatsNonZero("  rxBucket4SizeConfig     = %llu\r\n", (uint64_t)st->rxBucket4SizeConfig);
    }
    EnetAppUtils_printStatsNonZero("  rx64BSizedFrame         = %llu\r\n", (uint64_t)st->rx64BSizedFrame);
    EnetAppUtils_printStatsNonZero("  rxBucket1SizedFrame     = %llu\r\n", (uint64_t)st->rxBucket1SizedFrame);
    EnetAppUtils_printStatsNonZero("  rxBucket2SizedFrame     = %llu\r\n", (uint64_t)st->rxBucket2SizedFrame);
    EnetAppUtils_printStatsNonZero("  rxBucket3SizedFrame     = %llu\r\n", (uint64_t)st->rxBucket3SizedFrame);
    EnetAppUtils_printStatsNonZero("  rxBucket4SizedFrame     = %llu\r\n", (uint64_t)st->rxBucket4SizedFrame);
    EnetAppUtils_printStatsNonZero("  rxBucket5SizedFrame     = %llu\r\n", (uint64_t)st->rxBucket5SizedFrame);
    EnetAppUtils_printStatsNonZero("  rxTotalByte             = %llu\r\n", (uint64_t)st->rxTotalByte);
    EnetAppUtils_printStatsNonZero("  rxTxTotalByte           = %llu\r\n", (uint64_t)st->rxTxTotalByte);
    EnetAppUtils_printStatsNonZero("  txGoodFrame             = %llu\r\n", (uint64_t)st->txGoodFrame);
    EnetAppUtils_printStatsNonZero("  txBcastFrame            = %llu\r\n", (uint64_t)st->txBcastFrame);
    EnetAppUtils_printStatsNonZero("  txMcastFrame            = %llu\r\n", (uint64_t)st->txMcastFrame);
    EnetAppUtils_printStatsNonZero("  txOddNibbleFrame        = %llu\r\n", (uint64_t)st->txOddNibbleFrame);
    EnetAppUtils_printStatsNonZero("  txUnderFlowErr          = %llu\r\n", (uint64_t)st->txUnderFlowErr);
    if (printFixedCounters)
    {
        EnetAppUtils_printStatsNonZero("  txMaxSizeFrame          = %llu\r\n", (uint64_t)st->txMaxSizeFrame);
    }
    EnetAppUtils_printStatsNonZero("  txMaxSizeErrFrame       = %llu\r\n", (uint64_t)st->txMaxSizeErrFrame);
    if (printFixedCounters)
    {
        EnetAppUtils_printStatsNonZero("  txMinSizeFrame          = %llu\r\n", (uint64_t)st->txMinSizeFrame);
    }
    EnetAppUtils_printStatsNonZero("  txMinSizeErrFrame       = %llu\r\n", (uint64_t)st->txMinSizeErrFrame);
    if (printFixedCounters)
    {
        EnetAppUtils_printStatsNonZero("  txBucket1SizeConfig     = %llu\r\n", (uint64_t)st->txBucket1SizeConfig);
        EnetAppUtils_printStatsNonZero("  txBucket2SizeConfig     = %llu\r\n", (uint64_t)st->txBucket2SizeConfig);
        EnetAppUtils_printStatsNonZero("  txBucket3SizeConfig     = %llu\r\n", (uint64_t)st->txBucket3SizeConfig);
        EnetAppUtils_printStatsNonZero("  txBucket4SizeConfig     = %llu\r\n", (uint64_t)st->txBucket4SizeConfig);
    }
    EnetAppUtils_printStatsNonZero("  tx64BSizedFrame         = %llu\r\n", (uint64_t)st->tx64BSizedFrame);
    EnetAppUtils_printStatsNonZero("  txBucket1SizedFrame     = %llu\r\n", (uint64_t)st->txBucket1SizedFrame);
    EnetAppUtils_printStatsNonZero("  txBucket2SizedFrame     = %llu\r\n", (uint64_t)st->txBucket2SizedFrame);
    EnetAppUtils_printStatsNonZero("  txBucket3SizedFrame     = %llu\r\n", (uint64_t)st->txBucket3SizedFrame);
    EnetAppUtils_printStatsNonZero("  txBucket4SizedFrame     = %llu\r\n", (uint64_t)st->txBucket4SizedFrame);
    EnetAppUtils_printStatsNonZero("  txBucket5SizedFrame     = %llu\r\n", (uint64_t)st->txBucket5SizedFrame);
    EnetAppUtils_printStatsNonZero("  txTotalByte             = %llu\r\n", (uint64_t)st->txTotalByte);
}

void EnetAppUtils_printIcssgPaStats(IcssgStats_Pa *st)
{
    EnetAppUtils_printStatsNonZero("  hostRxByteCnt              = %llu\r\n", st->hostRxByteCnt);
    EnetAppUtils_printStatsNonZero("  hostTxByteCnt              = %llu\r\n", st->hostTxByteCnt);
    EnetAppUtils_printStatsNonZero("  hostRxByteCntSlice0        = %llu\r\n", st->hostRxByteCntMacSlice0);
    EnetAppUtils_printStatsNonZero("  hostRxByteCntSlice1        = %llu\r\n", st->hostRxByteCntMacSlice1);
    EnetAppUtils_printStatsNonZero("  hostTxByteCntSlice0        = %llu\r\n", st->hostTxByteCntMacSlice0);
    EnetAppUtils_printStatsNonZero("  hostTxByteCntSlice1        = %llu\r\n", st->hostTxByteCntMacSlice1);
    EnetAppUtils_printStatsNonZero("  hostRxPktCnt               = %llu\r\n", (uint64_t)st->hostRxPktCnt);
    EnetAppUtils_printStatsNonZero("  hostTxPktCnt               = %llu\r\n", (uint64_t)st->hostTxPktCnt);
    EnetAppUtils_printStatsNonZero("  hostRxPktCntSlice0         = %llu\r\n", (uint64_t)st->hostRxPktCntMacSlice0);
    EnetAppUtils_printStatsNonZero("  hostRxPktCntSlice1         = %llu\r\n", (uint64_t)st->hostRxPktCntMacSlice1);
    EnetAppUtils_printStatsNonZero("  hostTxPktCntSlice0         = %llu\r\n", (uint64_t)st->hostTxPktCntMacSlice0);
    EnetAppUtils_printStatsNonZero("  hostTxPktCntSlice1         = %llu\r\n", (uint64_t)st->hostTxPktCntMacSlice1);
    EnetAppUtils_printStatsNonZero("  rtu0PktDroppedSlice0       = %llu\r\n", (uint64_t)st->rtu0PktDroppedSlice0);
    EnetAppUtils_printStatsNonZero("  rtu0PktDroppedSlice1       = %llu\r\n", (uint64_t)st->rtu0PktDroppedSlice1);
    EnetAppUtils_printStatsNonZero("  port1Q0Overflow            = %llu\r\n", (uint64_t)st->port1Q0Overflow);
    EnetAppUtils_printStatsNonZero("  port1Q1Overflow            = %llu\r\n", (uint64_t)st->port1Q1Overflow);
    EnetAppUtils_printStatsNonZero("  port1Q2Overflow            = %llu\r\n", (uint64_t)st->port1Q2Overflow);
    EnetAppUtils_printStatsNonZero("  port1Q3Overflow            = %llu\r\n", (uint64_t)st->port1Q3Overflow);
    EnetAppUtils_printStatsNonZero("  port1Q4Overflow            = %llu\r\n", (uint64_t)st->port1Q4Overflow);
    EnetAppUtils_printStatsNonZero("  port1Q5Overflow            = %llu\r\n", (uint64_t)st->port1Q5Overflow);
    EnetAppUtils_printStatsNonZero("  port1Q6Overflow            = %llu\r\n", (uint64_t)st->port1Q6Overflow);
    EnetAppUtils_printStatsNonZero("  port1Q7Overflow            = %llu\r\n", (uint64_t)st->port1Q7Overflow);
    EnetAppUtils_printStatsNonZero("  port2Q0Overflow            = %llu\r\n", (uint64_t)st->port2Q0Overflow);
    EnetAppUtils_printStatsNonZero("  port2Q1Overflow            = %llu\r\n", (uint64_t)st->port2Q1Overflow);
    EnetAppUtils_printStatsNonZero("  port2Q2Overflow            = %llu\r\n", (uint64_t)st->port2Q2Overflow);
    EnetAppUtils_printStatsNonZero("  port2Q3Overflow            = %llu\r\n", (uint64_t)st->port2Q3Overflow);
    EnetAppUtils_printStatsNonZero("  port2Q4Overflow            = %llu\r\n", (uint64_t)st->port2Q4Overflow);
    EnetAppUtils_printStatsNonZero("  port2Q5Overflow            = %llu\r\n", (uint64_t)st->port2Q5Overflow);
    EnetAppUtils_printStatsNonZero("  port2Q6Overflow            = %llu\r\n", (uint64_t)st->port2Q6Overflow);
    EnetAppUtils_printStatsNonZero("  port2Q7Overflow            = %llu\r\n", (uint64_t)st->port2Q7Overflow);
    EnetAppUtils_printStatsNonZero("  hostQ0Overflow             = %llu\r\n", (uint64_t)st->hostQ0Overflow);
    EnetAppUtils_printStatsNonZero("  hostQ1Overflow             = %llu\r\n", (uint64_t)st->hostQ1Overflow);
    EnetAppUtils_printStatsNonZero("  hostQ2Overflow             = %llu\r\n", (uint64_t)st->hostQ2Overflow);
    EnetAppUtils_printStatsNonZero("  hostQ3Overflow             = %llu\r\n", (uint64_t)st->hostQ3Overflow);
    EnetAppUtils_printStatsNonZero("  hostQ4Overflow             = %llu\r\n", (uint64_t)st->hostQ4Overflow);
    EnetAppUtils_printStatsNonZero("  hostQ5Overflow             = %llu\r\n", (uint64_t)st->hostQ5Overflow);
    EnetAppUtils_printStatsNonZero("  hostQ6Overflow             = %llu\r\n", (uint64_t)st->hostQ6Overflow);
    EnetAppUtils_printStatsNonZero("  hostQ7Overflow             = %llu\r\n", (uint64_t)st->hostQ7Overflow);
    EnetAppUtils_printStatsNonZero("  hostEgressQPreOverflow     = %llu\r\n", (uint64_t)st->hostEgressQPreOverflow);
    EnetAppUtils_printStatsNonZero("  droppedPktSlice0           = %llu\r\n", (uint64_t)st->droppedPktSlice0);
    EnetAppUtils_printStatsNonZero("  droppedPktSlice1           = %llu\r\n", (uint64_t)st->droppedPktSlice1);
    EnetAppUtils_printStatsNonZero("  rxErrorSlice0              = %llu\r\n", (uint64_t)st->rxErrorSlice0);
    EnetAppUtils_printStatsNonZero("  rxErrorSlice1              = %llu\r\n", (uint64_t)st->rxErrorSlice1);
    EnetAppUtils_printStatsNonZero("  rxEofRtuDsInvalidSlice0    = %llu\r\n", (uint64_t)st->rxEofRtuDsInvalidSlice0);
    EnetAppUtils_printStatsNonZero("  rxEofRtuDsInvalidSlice1    = %llu\r\n", (uint64_t)st->rxEofRtuDsInvalidSlice1);
    EnetAppUtils_printStatsNonZero("  txPort1DroppedPkt          = %llu\r\n", (uint64_t)st->txPort1DroppedPkt);
    EnetAppUtils_printStatsNonZero("  txPort2DroppedPkt          = %llu\r\n", (uint64_t)st->txPort2DroppedPkt);
    EnetAppUtils_printStatsNonZero("  txPort1TsDroppedPkt        = %llu\r\n", (uint64_t)st->txPort1TsDroppedPkt);
    EnetAppUtils_printStatsNonZero("  txPort2TsDroppedPkt        = %llu\r\n", (uint64_t)st->txPort2TsDroppedPkt);
    EnetAppUtils_printStatsNonZero("  infPortDisabledSlice0      = %llu\r\n", (uint64_t)st->infPortDisabledSlice0);
    EnetAppUtils_printStatsNonZero("  infPortDisabledSlice1      = %llu\r\n", (uint64_t)st->infPortDisabledSlice1);
    EnetAppUtils_printStatsNonZero("  infSavSlice0               = %llu\r\n", (uint64_t)st->infSavSlice0);
    EnetAppUtils_printStatsNonZero("  infSavSlice1               = %llu\r\n", (uint64_t)st->infSavSlice1);
    EnetAppUtils_printStatsNonZero("  infSaBlSlice0              = %llu\r\n", (uint64_t)st->infSaBlSlice0);
    EnetAppUtils_printStatsNonZero("  infSaBlSlice1              = %llu\r\n", (uint64_t)st->infSaBlSlice1);
    EnetAppUtils_printStatsNonZero("  infPortBlockedSlice0       = %llu\r\n", (uint64_t)st->infPortBlockedSlice0);
    EnetAppUtils_printStatsNonZero("  infPortBlockedSlice1       = %llu\r\n", (uint64_t)st->infPortBlockedSlice1);
    EnetAppUtils_printStatsNonZero("  infAftDropTaggedSlice0     = %llu\r\n", (uint64_t)st->infAftDropTaggedSlice0);
    EnetAppUtils_printStatsNonZero("  infAftDropTaggedSlice1     = %llu\r\n", (uint64_t)st->infAftDropTaggedSlice1);
    EnetAppUtils_printStatsNonZero("  infAftDropPrioTaggedSlice0 = %llu\r\n", (uint64_t)st->infAftDropPrioTaggedSlice0);
    EnetAppUtils_printStatsNonZero("  infAftDropPrioTaggedSlice1 = %llu\r\n", (uint64_t)st->infAftDropPrioTaggedSlice1);
    EnetAppUtils_printStatsNonZero("  infAftDropNoTagSlice0      = %llu\r\n", (uint64_t)st->infAftDropNoTagSlice0);
    EnetAppUtils_printStatsNonZero("  infAftDropNoTagSlice1      = %llu\r\n", (uint64_t)st->infAftDropNoTagSlice1);
    EnetAppUtils_printStatsNonZero("  infAftDropNotMemberSlice0  = %llu\r\n", (uint64_t)st->infAftDropNotMemberSlice0);
    EnetAppUtils_printStatsNonZero("  infAftDropNotMemberSlice1  = %llu\r\n", (uint64_t)st->infAftDropNotMemberSlice1);
    EnetAppUtils_printStatsNonZero("  fdbNoSpaceToLearn          = %llu\r\n", (uint64_t)st->fdbNoSpaceToLearn);
    EnetAppUtils_printStatsNonZero("  preemptBadFragSlice0       = %llu\r\n", (uint64_t)st->preemptBadFragSlice0);
    EnetAppUtils_printStatsNonZero("  preemptBadFragSlice1       = %llu\r\n", (uint64_t)st->preemptBadFragSlice1);
    EnetAppUtils_printStatsNonZero("  preemptAsmErrSlice0        = %llu\r\n", (uint64_t)st->preemptAsmErrSlice0);
    EnetAppUtils_printStatsNonZero("  preemptAsmErrSlice1        = %llu\r\n", (uint64_t)st->preemptAsmErrSlice1);
    EnetAppUtils_printStatsNonZero("  preemptFragCntTxSlice0     = %llu\r\n", (uint64_t)st->preemptFragCntTxSlice0);
    EnetAppUtils_printStatsNonZero("  preemptFragCntTxSlice1     = %llu\r\n", (uint64_t)st->preemptFragCntTxSlice1);
    EnetAppUtils_printStatsNonZero("  preemptAsmOkSlice0         = %llu\r\n", (uint64_t)st->preemptAsmOkSlice0);
    EnetAppUtils_printStatsNonZero("  preemptAsmOkSlice1         = %llu\r\n", (uint64_t)st->preemptAsmOkSlice1);
    EnetAppUtils_printStatsNonZero("  preemptFragCntRxSlice0     = %llu\r\n", (uint64_t)st->preemptFragCntRxSlice0);
    EnetAppUtils_printStatsNonZero("  preemptFragCntRxSlice1     = %llu\r\n", (uint64_t)st->preemptFragCntRxSlice1);
    EnetAppUtils_printStatsNonZero("  rxEofShortFrameErrSlice0   = %llu\r\n", (uint64_t)st->rxEofShortFrameErrSlice0);
    EnetAppUtils_printStatsNonZero("  rxEofShortFrameErrSlice1   = %llu\r\n", (uint64_t)st->rxEofShortFrameErrSlice1);
}
#endif

void EnetAppUtils_printHostPortStats2G(CpswStats_HostPort_2g *st)
{
    uint32_t i;

    EnetAppUtils_printStatsNonZero("  rxGoodFrames            = %llu\r\n", st->rxGoodFrames);
    EnetAppUtils_printStatsNonZero("  rxBcastFrames           = %llu\r\n", st->rxBcastFrames);
    EnetAppUtils_printStatsNonZero("  rxMcastFrames           = %llu\r\n", st->rxMcastFrames);
    EnetAppUtils_printStatsNonZero("  rxCrcErrors             = %llu\r\n", st->rxCrcErrors);
    EnetAppUtils_printStatsNonZero("  rxOversizedFrames       = %llu\r\n", st->rxOversizedFrames);
    EnetAppUtils_printStatsNonZero("  rxUndersizedFrames      = %llu\r\n", st->rxUndersizedFrames);
    EnetAppUtils_printStatsNonZero("  aleDrop                 = %llu\r\n", st->aleDrop);
    EnetAppUtils_printStatsNonZero("  aleOverrunDrop          = %llu\r\n", st->aleOverrunDrop);
    EnetAppUtils_printStatsNonZero("  rxOctets                = %llu\r\n", st->rxOctets);
    EnetAppUtils_printStatsNonZero("  txGoodFrames            = %llu\r\n", st->txGoodFrames);
    EnetAppUtils_printStatsNonZero("  txBcastFrames           = %llu\r\n", st->txBcastFrames);
    EnetAppUtils_printStatsNonZero("  txMcastFrames           = %llu\r\n", st->txMcastFrames);
    EnetAppUtils_printStatsNonZero("  txSofOverrun            = %llu\r\n", st->txSofOverrun);
    EnetAppUtils_printStatsNonZero("  txMofOverrun            = %llu\r\n", st->txMofOverrun);
    EnetAppUtils_printStatsNonZero("  txOctets                = %llu\r\n", st->txOctets);
    EnetAppUtils_printStatsNonZero("  octetsFrames64          = %llu\r\n", st->octetsFrames64);
    EnetAppUtils_printStatsNonZero("  octetsFrames65to127     = %llu\r\n", st->octetsFrames65to127);
    EnetAppUtils_printStatsNonZero("  octetsFrames128to255    = %llu\r\n", st->octetsFrames128to255);
    EnetAppUtils_printStatsNonZero("  octetsFrames256to511    = %llu\r\n", st->octetsFrames256to511);
    EnetAppUtils_printStatsNonZero("  octetsFrames512to1023   = %llu\r\n", st->octetsFrames512to1023);
    EnetAppUtils_printStatsNonZero("  octetsFrames1024        = %llu\r\n", st->octetsFrames1024);
    EnetAppUtils_printStatsNonZero("  netOctets               = %llu\r\n", st->netOctets);
    EnetAppUtils_printStatsNonZero("  rxBottomOfFifoDrop      = %llu\r\n", st->rxBottomOfFifoDrop);
    EnetAppUtils_printStatsNonZero("  portMaskDrop            = %llu\r\n", st->portMaskDrop);
    EnetAppUtils_printStatsNonZero("  rxTopOfFifoDrop         = %llu\r\n", st->rxTopOfFifoDrop);
    EnetAppUtils_printStatsNonZero("  aleRateLimitDrop        = %llu\r\n", st->aleRateLimitDrop);
    EnetAppUtils_printStatsNonZero("  aleVidIngressDrop       = %llu\r\n", st->aleVidIngressDrop);
    EnetAppUtils_printStatsNonZero("  aleDAEqSADrop           = %llu\r\n", st->aleDAEqSADrop);
    EnetAppUtils_printStatsNonZero("  aleBlockDrop            = %llu\r\n", st->aleBlockDrop);
    EnetAppUtils_printStatsNonZero("  aleSecureDrop           = %llu\r\n", st->aleSecureDrop);
    EnetAppUtils_printStatsNonZero("  aleAuthDrop             = %llu\r\n", st->aleAuthDrop);
    EnetAppUtils_printStatsNonZero("  aleUnknownUcast         = %llu\r\n", st->aleUnknownUcast);
    EnetAppUtils_printStatsNonZero("  aleUnknownUcastBcnt     = %llu\r\n", st->aleUnknownUcastBcnt);
    EnetAppUtils_printStatsNonZero("  aleUnknownMcast         = %llu\r\n", st->aleUnknownMcast);
    EnetAppUtils_printStatsNonZero("  aleUnknownMcastBcnt     = %llu\r\n", st->aleUnknownMcastBcnt);
    EnetAppUtils_printStatsNonZero("  aleUnknownBcast         = %llu\r\n", st->aleUnknownBcast);
    EnetAppUtils_printStatsNonZero("  aleUnknownBcastBcnt     = %llu\r\n", st->aleUnknownBcastBcnt);
    EnetAppUtils_printStatsNonZero("  alePolicyMatch          = %llu\r\n", st->alePolicyMatch);
    EnetAppUtils_printStatsNonZero("  alePolicyMatchRed       = %llu\r\n", st->alePolicyMatchRed);
    EnetAppUtils_printStatsNonZero("  alePolicyMatchYellow    = %llu\r\n", st->alePolicyMatchYellow);
    EnetAppUtils_printStatsNonZero("  aleMultSADrop           = %llu\r\n", st->aleMultSADrop);
    EnetAppUtils_printStatsNonZero("  aleDualVlanDrop         = %llu\r\n", st->aleDualVlanDrop);
    EnetAppUtils_printStatsNonZero("  aleLenErrorDrop         = %llu\r\n", st->aleLenErrorDrop);
    EnetAppUtils_printStatsNonZero("  aleIpNextHdrDrop        = %llu\r\n", st->aleIpNextHdrDrop);
    EnetAppUtils_printStatsNonZero("  aleIPv4FragDrop         = %llu\r\n", st->aleIPv4FragDrop);
    EnetAppUtils_printStatsNonZero("  txMemProtectError       = %llu\r\n", st->txMemProtectError);

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPri); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPri[%u]                = %llu\r\n", i, st->txPri[i]);
    }

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPriBcnt); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPriBcnt[%u]            = %llu\r\n", i, st->txPriBcnt[i]);
    }

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPriDrop); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPriDrop[%u]            = %llu\r\n", i, st->txPriDrop[i]);
    }

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPriDropBcnt); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPriDropBcnt[%u]        = %llu\r\n", i, st->txPriDropBcnt[i]);
    }
}

void EnetAppUtils_printMacPortStats2G(CpswStats_MacPort_2g *st)
{
    uint32_t i;

    EnetAppUtils_printStatsNonZero("  rxGoodFrames            = %llu\r\n", st->rxGoodFrames);
    EnetAppUtils_printStatsNonZero("  rxBcastFrames           = %llu\r\n", st->rxBcastFrames);
    EnetAppUtils_printStatsNonZero("  rxMcastFrames           = %llu\r\n", st->rxMcastFrames);
    EnetAppUtils_printStatsNonZero("  rxPauseFrames           = %llu\r\n", st->rxPauseFrames);
    EnetAppUtils_printStatsNonZero("  rxCrcErrors             = %llu\r\n", st->rxCrcErrors);
    EnetAppUtils_printStatsNonZero("  rxAlignCodeErrors       = %llu\r\n", st->rxAlignCodeErrors);
    EnetAppUtils_printStatsNonZero("  rxOversizedFrames       = %llu\r\n", st->rxOversizedFrames);
    EnetAppUtils_printStatsNonZero("  rxJabberFrames          = %llu\r\n", st->rxJabberFrames);
    EnetAppUtils_printStatsNonZero("  rxUndersizedFrames      = %llu\r\n", st->rxUndersizedFrames);
    EnetAppUtils_printStatsNonZero("  rxFragments             = %llu\r\n", st->rxFragments);
    EnetAppUtils_printStatsNonZero("  aleDrop                 = %llu\r\n", st->aleDrop);
    EnetAppUtils_printStatsNonZero("  aleOverrunDrop          = %llu\r\n", st->aleOverrunDrop);
    EnetAppUtils_printStatsNonZero("  rxOctets                = %llu\r\n", st->rxOctets);
    EnetAppUtils_printStatsNonZero("  txGoodFrames            = %llu\r\n", st->txGoodFrames);
    EnetAppUtils_printStatsNonZero("  txBcastFrames           = %llu\r\n", st->txBcastFrames);
    EnetAppUtils_printStatsNonZero("  txMcastFrames           = %llu\r\n", st->txMcastFrames);
    EnetAppUtils_printStatsNonZero("  txPauseFrames           = %llu\r\n", st->txPauseFrames);
    EnetAppUtils_printStatsNonZero("  txDeferredFrames        = %llu\r\n", st->txDeferredFrames);
    EnetAppUtils_printStatsNonZero("  txCollisionFrames       = %llu\r\n", st->txCollisionFrames);
    EnetAppUtils_printStatsNonZero("  txSingleCollFrames      = %llu\r\n", st->txSingleCollFrames);
    EnetAppUtils_printStatsNonZero("  txMultipleCollFrames    = %llu\r\n", st->txMultipleCollFrames);
    EnetAppUtils_printStatsNonZero("  txExcessiveCollFrames   = %llu\r\n", st->txExcessiveCollFrames);
    EnetAppUtils_printStatsNonZero("  txLateCollFrames        = %llu\r\n", st->txLateCollFrames);
    EnetAppUtils_printStatsNonZero("  rxIPGError              = %llu\r\n", st->rxIPGError);
    EnetAppUtils_printStatsNonZero("  txCarrierSenseErrors    = %llu\r\n", st->txCarrierSenseErrors);
    EnetAppUtils_printStatsNonZero("  txOctets                = %llu\r\n", st->txOctets);
    EnetAppUtils_printStatsNonZero("  octetsFrames64          = %llu\r\n", st->octetsFrames64);
    EnetAppUtils_printStatsNonZero("  octetsFrames65to127     = %llu\r\n", st->octetsFrames65to127);
    EnetAppUtils_printStatsNonZero("  octetsFrames128to255    = %llu\r\n", st->octetsFrames128to255);
    EnetAppUtils_printStatsNonZero("  octetsFrames256to511    = %llu\r\n", st->octetsFrames256to511);
    EnetAppUtils_printStatsNonZero("  octetsFrames512to1023   = %llu\r\n", st->octetsFrames512to1023);
    EnetAppUtils_printStatsNonZero("  octetsFrames1024        = %llu\r\n", st->octetsFrames1024);
    EnetAppUtils_printStatsNonZero("  netOctets               = %llu\r\n", st->netOctets);
    EnetAppUtils_printStatsNonZero("  rxBottomOfFifoDrop      = %llu\r\n", st->rxBottomOfFifoDrop);
    EnetAppUtils_printStatsNonZero("  portMaskDrop            = %llu\r\n", st->portMaskDrop);
    EnetAppUtils_printStatsNonZero("  rxTopOfFifoDrop         = %llu\r\n", st->rxTopOfFifoDrop);
    EnetAppUtils_printStatsNonZero("  aleRateLimitDrop        = %llu\r\n", st->aleRateLimitDrop);
    EnetAppUtils_printStatsNonZero("  aleVidIngressDrop       = %llu\r\n", st->aleVidIngressDrop);
    EnetAppUtils_printStatsNonZero("  aleDAEqSADrop           = %llu\r\n", st->aleDAEqSADrop);
    EnetAppUtils_printStatsNonZero("  aleBlockDrop            = %llu\r\n", st->aleBlockDrop);
    EnetAppUtils_printStatsNonZero("  aleSecureDrop           = %llu\r\n", st->aleSecureDrop);
    EnetAppUtils_printStatsNonZero("  aleAuthDrop             = %llu\r\n", st->aleAuthDrop);
    EnetAppUtils_printStatsNonZero("  aleUnknownUcast         = %llu\r\n", st->aleUnknownUcast);
    EnetAppUtils_printStatsNonZero("  aleUnknownUcastBcnt     = %llu\r\n", st->aleUnknownUcastBcnt);
    EnetAppUtils_printStatsNonZero("  aleUnknownMcast         = %llu\r\n", st->aleUnknownMcast);
    EnetAppUtils_printStatsNonZero("  aleUnknownMcastBcnt     = %llu\r\n", st->aleUnknownMcastBcnt);
    EnetAppUtils_printStatsNonZero("  aleUnknownBcast         = %llu\r\n", st->aleUnknownBcast);
    EnetAppUtils_printStatsNonZero("  aleUnknownBcastBcnt     = %llu\r\n", st->aleUnknownBcastBcnt);
    EnetAppUtils_printStatsNonZero("  alePolicyMatch          = %llu\r\n", st->alePolicyMatch);
    EnetAppUtils_printStatsNonZero("  alePolicyMatchRed       = %llu\r\n", st->alePolicyMatchRed);
    EnetAppUtils_printStatsNonZero("  alePolicyMatchYellow    = %llu\r\n", st->alePolicyMatchYellow);
    EnetAppUtils_printStatsNonZero("  aleMultSADrop           = %llu\r\n", st->aleMultSADrop);
    EnetAppUtils_printStatsNonZero("  aleDualVlanDrop         = %llu\r\n", st->aleDualVlanDrop);
    EnetAppUtils_printStatsNonZero("  aleLenErrorDrop         = %llu\r\n", st->aleLenErrorDrop);
    EnetAppUtils_printStatsNonZero("  aleIpNextHdrDrop        = %llu\r\n", st->aleIpNextHdrDrop);
    EnetAppUtils_printStatsNonZero("  aleIPv4FragDrop         = %llu\r\n", st->aleIPv4FragDrop);
    EnetAppUtils_printStatsNonZero("  ietRxAssemblyErr        = %llu\r\n", st->ietRxAssemblyErr);
    EnetAppUtils_printStatsNonZero("  ietRxAssemblyOk         = %llu\r\n", st->ietRxAssemblyOk);
    EnetAppUtils_printStatsNonZero("  ietRxSmdError           = %llu\r\n", st->ietRxSmdError);
    EnetAppUtils_printStatsNonZero("  ietRxFrag               = %llu\r\n", st->ietRxFrag);
    EnetAppUtils_printStatsNonZero("  ietTxHold               = %llu\r\n", st->ietTxHold);
    EnetAppUtils_printStatsNonZero("  ietTxFrag               = %llu\r\n", st->ietTxFrag);
    EnetAppUtils_printStatsNonZero("  txMemProtectError       = %llu\r\n", st->txMemProtectError);

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPri); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPri[%u]                = %llu\r\n", i, st->txPri[i]);
    }

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPriBcnt); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPriBcnt[%u]            = %llu\r\n", i, st->txPriBcnt[i]);
    }

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPriDrop); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPriDrop[%u]            = %llu\r\n", i, st->txPriDrop[i]);
    }

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPriDropBcnt); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPriDropBcnt[%u]        = %llu\r\n", i, st->txPriDropBcnt[i]);
    }
}

void EnetAppUtils_printHostPortStats9G(CpswStats_HostPort_Ng *st)
{
    uint_fast32_t i;

    EnetAppUtils_printStatsNonZero("  rxGoodFrames            = %llu\r\n", st->rxGoodFrames);
    EnetAppUtils_printStatsNonZero("  rxBcastFrames           = %llu\r\n", st->rxBcastFrames);
    EnetAppUtils_printStatsNonZero("  rxMcastFrames           = %llu\r\n", st->rxMcastFrames);
    EnetAppUtils_printStatsNonZero("  rxCrcErrors             = %llu\r\n", st->rxCrcErrors);
    EnetAppUtils_printStatsNonZero("  rxOversizedFrames       = %llu\r\n", st->rxOversizedFrames);
    EnetAppUtils_printStatsNonZero("  rxUndersizedFrames      = %llu\r\n", st->rxUndersizedFrames);
    EnetAppUtils_printStatsNonZero("  aleDrop                 = %llu\r\n", st->aleDrop);
    EnetAppUtils_printStatsNonZero("  aleOverrunDrop          = %llu\r\n", st->aleOverrunDrop);
    EnetAppUtils_printStatsNonZero("  rxOctets                = %llu\r\n", st->rxOctets);
    EnetAppUtils_printStatsNonZero("  txGoodFrames            = %llu\r\n", st->txGoodFrames);
    EnetAppUtils_printStatsNonZero("  txBcastFrames           = %llu\r\n", st->txBcastFrames);
    EnetAppUtils_printStatsNonZero("  txMcastFrames           = %llu\r\n", st->txMcastFrames);
    EnetAppUtils_printStatsNonZero("  txSofOverrun            = %llu\r\n", st->txSofOverrun);
    EnetAppUtils_printStatsNonZero("  txMofOverrun            = %llu\r\n", st->txMofOverrun);
    EnetAppUtils_printStatsNonZero("  txOctets                = %llu\r\n", st->txOctets);
    EnetAppUtils_printStatsNonZero("  octetsFrames64          = %llu\r\n", st->octetsFrames64);
    EnetAppUtils_printStatsNonZero("  octetsFrames65to127     = %llu\r\n", st->octetsFrames65to127);
    EnetAppUtils_printStatsNonZero("  octetsFrames128to255    = %llu\r\n", st->octetsFrames128to255);
    EnetAppUtils_printStatsNonZero("  octetsFrames256to511    = %llu\r\n", st->octetsFrames256to511);
    EnetAppUtils_printStatsNonZero("  octetsFrames512to1023   = %llu\r\n", st->octetsFrames512to1023);
    EnetAppUtils_printStatsNonZero("  octetsFrames1024        = %llu\r\n", st->octetsFrames1024);
    EnetAppUtils_printStatsNonZero("  netOctets               = %llu\r\n", st->netOctets);
    EnetAppUtils_printStatsNonZero("  rxBottomOfFifoDrop      = %llu\r\n", st->rxBottomOfFifoDrop);
    EnetAppUtils_printStatsNonZero("  portMaskDrop            = %llu\r\n", st->portMaskDrop);
    EnetAppUtils_printStatsNonZero("  rxTopOfFifoDrop         = %llu\r\n", st->rxTopOfFifoDrop);
    EnetAppUtils_printStatsNonZero("  aleRateLimitDrop        = %llu\r\n", st->aleRateLimitDrop);
    EnetAppUtils_printStatsNonZero("  aleVidIngressDrop       = %llu\r\n", st->aleVidIngressDrop);
    EnetAppUtils_printStatsNonZero("  aleDAEqSADrop           = %llu\r\n", st->aleDAEqSADrop);
    EnetAppUtils_printStatsNonZero("  aleBlockDrop            = %llu\r\n", st->aleBlockDrop);
    EnetAppUtils_printStatsNonZero("  aleSecureDrop           = %llu\r\n", st->aleSecureDrop);
    EnetAppUtils_printStatsNonZero("  aleAuthDrop             = %llu\r\n", st->aleAuthDrop);
    EnetAppUtils_printStatsNonZero("  aleUnknownUcast         = %llu\r\n", st->aleUnknownUcast);
    EnetAppUtils_printStatsNonZero("  aleUnknownUcastBcnt     = %llu\r\n", st->aleUnknownUcastBcnt);
    EnetAppUtils_printStatsNonZero("  aleUnknownMcast         = %llu\r\n", st->aleUnknownMcast);
    EnetAppUtils_printStatsNonZero("  aleUnknownMcastBcnt     = %llu\r\n", st->aleUnknownMcastBcnt);
    EnetAppUtils_printStatsNonZero("  aleUnknownBcast         = %llu\r\n", st->aleUnknownBcast);
    EnetAppUtils_printStatsNonZero("  aleUnknownBcastBcnt     = %llu\r\n", st->aleUnknownBcastBcnt);
    EnetAppUtils_printStatsNonZero("  alePolicyMatch          = %llu\r\n", st->alePolicyMatch);
    EnetAppUtils_printStatsNonZero("  alePolicyMatchRed       = %llu\r\n", st->alePolicyMatchRed);
    EnetAppUtils_printStatsNonZero("  alePolicyMatchYellow    = %llu\r\n", st->alePolicyMatchYellow);
    EnetAppUtils_printStatsNonZero("  aleMultSADrop           = %llu\r\n", st->aleMultSADrop);
    EnetAppUtils_printStatsNonZero("  aleDualVlanDrop         = %llu\r\n", st->aleDualVlanDrop);
    EnetAppUtils_printStatsNonZero("  aleLenErrorDrop         = %llu\r\n", st->aleLenErrorDrop);
    EnetAppUtils_printStatsNonZero("  aleIpNextHdrDrop        = %llu\r\n", st->aleIpNextHdrDrop);
    EnetAppUtils_printStatsNonZero("  aleIPv4FragDrop         = %llu\r\n", st->aleIPv4FragDrop);
    EnetAppUtils_printStatsNonZero("  txMemProtectError       = %llu\r\n", st->txMemProtectError);

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPri); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPri[%u]                = %llu\r\n", i, st->txPri[i]);
    }

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPriBcnt); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPriBcnt[%u]            = %llu\r\n", i, st->txPriBcnt[i]);
    }

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPriDrop); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPriDrop[%u]            = %llu\r\n", i, st->txPriDrop[i]);
    }

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPriDropBcnt); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPriDropBcnt[%u]        = %llu\r\n", i, st->txPriDropBcnt[i]);
    }
}

void EnetAppUtils_printMacPortStats9G(CpswStats_MacPort_Ng *st)
{
    uint_fast32_t i;

    EnetAppUtils_printStatsNonZero("  rxGoodFrames            = %llu\r\n", st->rxGoodFrames);
    EnetAppUtils_printStatsNonZero("  rxBcastFrames           = %llu\r\n", st->rxBcastFrames);
    EnetAppUtils_printStatsNonZero("  rxMcastFrames           = %llu\r\n", st->rxMcastFrames);
    EnetAppUtils_printStatsNonZero("  rxPauseFrames           = %llu\r\n", st->rxPauseFrames);
    EnetAppUtils_printStatsNonZero("  rxCrcErrors             = %llu\r\n", st->rxCrcErrors);
    EnetAppUtils_printStatsNonZero("  rxAlignCodeErrors       = %llu\r\n", st->rxAlignCodeErrors);
    EnetAppUtils_printStatsNonZero("  rxOversizedFrames       = %llu\r\n", st->rxOversizedFrames);
    EnetAppUtils_printStatsNonZero("  rxJabberFrames          = %llu\r\n", st->rxJabberFrames);
    EnetAppUtils_printStatsNonZero("  rxUndersizedFrames      = %llu\r\n", st->rxUndersizedFrames);
    EnetAppUtils_printStatsNonZero("  rxFragments             = %llu\r\n", st->rxFragments);
    EnetAppUtils_printStatsNonZero("  aleDrop                 = %llu\r\n", st->aleDrop);
    EnetAppUtils_printStatsNonZero("  aleOverrunDrop          = %llu\r\n", st->aleOverrunDrop);
    EnetAppUtils_printStatsNonZero("  rxOctets                = %llu\r\n", st->rxOctets);
    EnetAppUtils_printStatsNonZero("  txGoodFrames            = %llu\r\n", st->txGoodFrames);
    EnetAppUtils_printStatsNonZero("  txBcastFrames           = %llu\r\n", st->txBcastFrames);
    EnetAppUtils_printStatsNonZero("  txMcastFrames           = %llu\r\n", st->txMcastFrames);
    EnetAppUtils_printStatsNonZero("  txPauseFrames           = %llu\r\n", st->txPauseFrames);
    EnetAppUtils_printStatsNonZero("  txDeferredFrames        = %llu\r\n", st->txDeferredFrames);
    EnetAppUtils_printStatsNonZero("  txCollisionFrames       = %llu\r\n", st->txCollisionFrames);
    EnetAppUtils_printStatsNonZero("  txSingleCollFrames      = %llu\r\n", st->txSingleCollFrames);
    EnetAppUtils_printStatsNonZero("  txMultipleCollFrames    = %llu\r\n", st->txMultipleCollFrames);
    EnetAppUtils_printStatsNonZero("  txExcessiveCollFrames   = %llu\r\n", st->txExcessiveCollFrames);
    EnetAppUtils_printStatsNonZero("  txLateCollFrames        = %llu\r\n", st->txLateCollFrames);
    EnetAppUtils_printStatsNonZero("  rxIPGError              = %llu\r\n", st->rxIPGError);
    EnetAppUtils_printStatsNonZero("  txCarrierSenseErrors    = %llu\r\n", st->txCarrierSenseErrors);
    EnetAppUtils_printStatsNonZero("  txOctets                = %llu\r\n", st->txOctets);
    EnetAppUtils_printStatsNonZero("  octetsFrames64          = %llu\r\n", st->octetsFrames64);
    EnetAppUtils_printStatsNonZero("  octetsFrames65to127     = %llu\r\n", st->octetsFrames65to127);
    EnetAppUtils_printStatsNonZero("  octetsFrames128to255    = %llu\r\n", st->octetsFrames128to255);
    EnetAppUtils_printStatsNonZero("  octetsFrames256to511    = %llu\r\n", st->octetsFrames256to511);
    EnetAppUtils_printStatsNonZero("  octetsFrames512to1023   = %llu\r\n", st->octetsFrames512to1023);
    EnetAppUtils_printStatsNonZero("  octetsFrames1024        = %llu\r\n", st->octetsFrames1024);
    EnetAppUtils_printStatsNonZero("  netOctets               = %llu\r\n", st->netOctets);
    EnetAppUtils_printStatsNonZero("  rxBottomOfFifoDrop      = %llu\r\n", st->rxBottomOfFifoDrop);
    EnetAppUtils_printStatsNonZero("  portMaskDrop            = %llu\r\n", st->portMaskDrop);
    EnetAppUtils_printStatsNonZero("  rxTopOfFifoDrop         = %llu\r\n", st->rxTopOfFifoDrop);
    EnetAppUtils_printStatsNonZero("  aleRateLimitDrop        = %llu\r\n", st->aleRateLimitDrop);
    EnetAppUtils_printStatsNonZero("  aleVidIngressDrop       = %llu\r\n", st->aleVidIngressDrop);
    EnetAppUtils_printStatsNonZero("  aleDAEqSADrop           = %llu\r\n", st->aleDAEqSADrop);
    EnetAppUtils_printStatsNonZero("  aleBlockDrop            = %llu\r\n", st->aleBlockDrop);
    EnetAppUtils_printStatsNonZero("  aleSecureDrop           = %llu\r\n", st->aleSecureDrop);
    EnetAppUtils_printStatsNonZero("  aleAuthDrop             = %llu\r\n", st->aleAuthDrop);
    EnetAppUtils_printStatsNonZero("  aleUnknownUcast         = %llu\r\n", st->aleUnknownUcast);
    EnetAppUtils_printStatsNonZero("  aleUnknownUcastBcnt     = %llu\r\n", st->aleUnknownUcastBcnt);
    EnetAppUtils_printStatsNonZero("  aleUnknownMcast         = %llu\r\n", st->aleUnknownMcast);
    EnetAppUtils_printStatsNonZero("  aleUnknownMcastBcnt     = %llu\r\n", st->aleUnknownMcastBcnt);
    EnetAppUtils_printStatsNonZero("  aleUnknownBcast         = %llu\r\n", st->aleUnknownBcast);
    EnetAppUtils_printStatsNonZero("  aleUnknownBcastBcnt     = %llu\r\n", st->aleUnknownBcastBcnt);
    EnetAppUtils_printStatsNonZero("  alePolicyMatch          = %llu\r\n", st->alePolicyMatch);
    EnetAppUtils_printStatsNonZero("  alePolicyMatchRed       = %llu\r\n", st->alePolicyMatchRed);
    EnetAppUtils_printStatsNonZero("  alePolicyMatchYellow    = %llu\r\n", st->alePolicyMatchYellow);
    EnetAppUtils_printStatsNonZero("  aleMultSADrop           = %llu\r\n", st->aleMultSADrop);
    EnetAppUtils_printStatsNonZero("  aleDualVlanDrop         = %llu\r\n", st->aleDualVlanDrop);
    EnetAppUtils_printStatsNonZero("  aleLenErrorDrop         = %llu\r\n", st->aleLenErrorDrop);
    EnetAppUtils_printStatsNonZero("  aleIpNextHdrDrop        = %llu\r\n", st->aleIpNextHdrDrop);
    EnetAppUtils_printStatsNonZero("  aleIPv4FragDrop         = %llu\r\n", st->aleIPv4FragDrop);
    EnetAppUtils_printStatsNonZero("  ietRxAssemblyErr        = %llu\r\n", st->ietRxAssemblyErr);
    EnetAppUtils_printStatsNonZero("  ietRxAssemblyOk         = %llu\r\n", st->ietRxAssemblyOk);
    EnetAppUtils_printStatsNonZero("  ietRxSmdError           = %llu\r\n", st->ietRxSmdError);
    EnetAppUtils_printStatsNonZero("  ietRxFrag               = %llu\r\n", st->ietRxFrag);
    EnetAppUtils_printStatsNonZero("  ietTxHold               = %llu\r\n", st->ietTxHold);
    EnetAppUtils_printStatsNonZero("  ietTxFrag               = %llu\r\n", st->ietTxFrag);
    EnetAppUtils_printStatsNonZero("  txMemProtectError       = %llu\r\n", st->txMemProtectError);

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPri); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPri[%u]                = %llu\r\n", i, st->txPri[i]);
    }

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPriBcnt); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPriBcnt[%u]            = %llu\r\n", i, st->txPriBcnt[i]);
    }

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPriDrop); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPriDrop[%u]            = %llu\r\n", i, st->txPriDrop[i]);
    }

    for (i = 0U; i < ENET_ARRAYSIZE(st->txPriDropBcnt); i++)
    {
        EnetAppUtils_printStatsWithIdxNonZero("  txPriDropBcnt[%u]        = %llu\r\n", i, st->txPriDropBcnt[i]);
    }
}

void EnetAppUtils_validatePacketState(EnetDma_PktQ *pQueue,
                                      uint32_t expectedState,
                                      uint32_t newState)
{
    uint32_t i;
    EnetDma_Pkt *pktInfo = (EnetDma_Pkt *)pQueue->head;

    for (i = 0; i < EnetQueue_getQCount(pQueue); i++)
    {
        EnetDma_checkPktState(&pktInfo->pktState,
                                ENET_PKTSTATE_MODULE_APP,
                                expectedState,
                                newState);
        pktInfo = (EnetDma_Pkt *)pktInfo->node.next;
    }
}

#if ENET_CFG_IS_ON(RM_PRESENT)
/*
 * This function iterates over all the cores and reduce the mac address count as much as possible for each core.
 *
*/
static void EnetAppUtils_reduceCoreMacAllocation(EnetRm_ResPrms *resPrms,
                                                 uint32_t *pReduceCount,
                                                 uint32_t coreMinCount,
                                                 bool skipCore,
                                                 uint32_t skipCoreId)
{
    uint32_t i;

    for (i = 0; (i < resPrms->numCores) && (*pReduceCount > 0); i++)
    {
        if ((resPrms->coreDmaResInfo[i].numMacAddress > coreMinCount)
            &&
            ((skipCore == false) || (skipCoreId != resPrms->coreDmaResInfo[i].coreId)))
        {
            uint32_t coreMacAddrReducedCount = (resPrms->coreDmaResInfo[i].numMacAddress - coreMinCount);

            if (*pReduceCount >= coreMacAddrReducedCount)
            {
                *pReduceCount -= coreMacAddrReducedCount;
            }
            else
            {
                coreMacAddrReducedCount  = *pReduceCount;
                *pReduceCount            = 0;
            }

            EnetAppUtils_print("EnetAppUtils_reduceCoreMacAllocation: "
                               "Reduced Mac Address Allocation for CoreId:%u From %u To %u \r\n",
                               resPrms->coreDmaResInfo[i].coreId,
                               resPrms->coreDmaResInfo[i].numMacAddress,
                               (resPrms->coreDmaResInfo[i].numMacAddress - coreMacAddrReducedCount));
            resPrms->coreDmaResInfo[i].numMacAddress  -= coreMacAddrReducedCount;
        }
    }
}

static void EnetAppUtils_updatemacResPart(EnetRm_ResPrms *resPrms,
                                                    uint32_t availMacCount,
                                                    uint32_t selfCoreId)
{
    uint32_t totalResPartMacCnt;
    uint32_t i;

    totalResPartMacCnt = 0;
    for (i = 0; i < resPrms->numCores; i++)
    {
        totalResPartMacCnt += resPrms->coreDmaResInfo[i].numMacAddress;
    }

    if (totalResPartMacCnt > availMacCount)
    {
        uint32_t reduceCount = totalResPartMacCnt - availMacCount;

        /* First reduce mac count for cores with more than one mac address allocation.
         * If the available mac addr count is less than expected, Divide the available
         * mac addresses among the cores giving preference to the present core.
        */
        EnetAppUtils_reduceCoreMacAllocation(resPrms, &reduceCount, 1, false, selfCoreId);
        if (reduceCount)
        {
            /* Next reduce mac address for core other than self core to 0 */
            EnetAppUtils_reduceCoreMacAllocation(resPrms, &reduceCount, 0, true, selfCoreId);
        }

        /* Finally reduce self core also to 0 */
        if (reduceCount)
        {
            /* Next reduce mac address for core other than self core to 0 */
            EnetAppUtils_reduceCoreMacAllocation(resPrms, &reduceCount, 0, false, selfCoreId);
        }

        EnetAppUtils_assert(reduceCount == 0);
    }
}

#endif

void EnetAppUtils_initResourceConfig(Enet_Type enetType,
                                     uint32_t instId,
                                     uint32_t selfCoreId,
                                     EnetRm_ResCfg *resCfg)
{
#if ENET_CFG_IS_ON(RM_PRESENT)
    int32_t status;
    const EnetRm_ResPrms *resPrms         = EnetAppRm_getResPartInfo(enetType);
    const EnetRm_IoctlPermissionTable *ioPerms = EnetAppRm_getIoctlPermissionInfo(enetType);

    EnetAppUtils_assert(resPrms != NULL);
    resCfg->resPartInfo = *resPrms;

    EnetAppUtils_assert(ioPerms != NULL);
    resCfg->ioctlPermissionInfo = *ioPerms;

    status =
        EnetAppSoc_getMacAddrList(enetType,
                                  instId,
                                  resCfg->macList.macAddress,
                                  &resCfg->macList.numMacAddress);
    EnetAppUtils_assert(status == ENET_SOK);
    if (resCfg->macList.numMacAddress > ENET_ARRAYSIZE(resCfg->macList.macAddress))
    {
        EnetAppUtils_print("EnetAppUtils_initResourceConfig: "
                           "Limiting number of mac address entries to resCfg->macList.macAddress size"
                           "Available:%u, LimitedTo: %u",
                           resCfg->macList.numMacAddress,
                           ENET_ARRAYSIZE(resCfg->macList.macAddress));
        resCfg->macList.numMacAddress = ENET_ARRAYSIZE(resCfg->macList.macAddress);
    }

    EnetAppUtils_updatemacResPart(&resCfg->resPartInfo,
                                            resCfg->macList.numMacAddress,
                                            selfCoreId);
    resCfg->selfCoreId = selfCoreId;
#endif
}

void EnetAppUtils_setNoPhyCfgRgmii(EnetMacPort_Interface *interface,
                                   EnetPhy_Cfg *phyCfg)
{
    phyCfg->phyAddr      = ENETPHY_INVALID_PHYADDR;
    interface->layerType    = ENET_MAC_LAYER_GMII;
    interface->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
    interface->variantType  = ENET_MAC_VARIANT_FORCED;
}

void EnetAppUtils_setNoPhyCfgRmii(EnetMacPort_Interface *interface,
                                   EnetPhy_Cfg *phyCfg)
{
    phyCfg->phyAddr      = ENETPHY_INVALID_PHYADDR;
    interface->layerType    = ENET_MAC_LAYER_MII;
    interface->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
    interface->variantType  = ENET_MAC_VARIANT_NONE;
}

void EnetAppUtils_setNoPhyCfgSgmii(EnetMacPort_Interface *interface,
                                   CpswMacPort_Cfg *macCfg,
                                   EnetPhy_Cfg *phyCfg)
{
    phyCfg->phyAddr      = ENETPHY_INVALID_PHYADDR;
    interface->layerType    = ENET_MAC_LAYER_GMII;
    interface->sublayerType = ENET_MAC_SUBLAYER_SERIAL;
    interface->variantType  = ENET_MAC_VARIANT_NONE;

    macCfg->sgmiiMode = ENET_MAC_SGMIIMODE_SGMII_FORCEDLINK;
}

int8_t EnetAppUtils_hex2Num(char hex)
{
    int8_t num = -1;

    if ((hex >= '0') && (hex <= '9'))
    {
        num =  hex - '0';
    }
    else if (hex >= 'a' && hex <= 'f')
    {
        num = hex - 'a' + 10;
    }
    else if (hex >= 'A' && hex <= 'F')
    {
        num = hex - 'A' + 10;
    }

    return num;
}

int32_t EnetAppUtils_macAddrAtoI(const char *txt, uint8_t *addr)
{
    int32_t status = ENET_SOK;
    int8_t a, b, i;

    for (i = 0; i < 6; i++)
    {
        a = EnetAppUtils_hex2Num(*txt++);
        if (a < 0)
        {
            status = ENET_EFAIL;
        }

        b = EnetAppUtils_hex2Num(*txt++);
        if (b < 0)
        {
            status = ENET_EFAIL;
        }

        *addr++ = (a << 4) | b;

        if ((i < 5) && (*txt++ != ':'))
        {
            status = ENET_EFAIL;
            break;
        }
    }

    return status;
}

int32_t EnetAppUtils_ipAddrAtoI(const char* txt, uint8_t *addr)
{
    int32_t status = ENET_SOK;
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        addr[i] = strtoul(txt, NULL, 10U);
        txt = strchr(txt, '.');
        if (((txt == NULL) || (*txt == '\0')) && (i != 3U))
        {
            status = ENET_EFAIL;
            break;
        }
        txt++;
    }
    return status;

}

#if defined (SOC_AM273X) || defined (SOC_AWR294X) || defined (SOC_AM263X) || defined (SOC_AM263PX) || defined(SOC_AM261X)
void EnetAppUtils_enableClocks(Enet_Type enetType, uint32_t instId)
{
}
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X) || defined (SOC_AM263X) || defined (SOC_AM263PX) || defined(SOC_AM261X)
void EnetAppUtils_disableClocks(Enet_Type enetType, uint32_t instId)
{
}
#endif

void EnetAppUtils_assertLocal(bool condition,
                              const char *str,
                              const char *fileName,
                              int32_t lineNum)
{
    volatile static bool gCpswAssertWaitInLoop = TRUE;

    if (!(condition))
    {
        EnetAppUtils_print("Assertion @ Line: %d in %s: %s : failed !!!\r\n",
                           lineNum, fileName, str);
        while (gCpswAssertWaitInLoop)
        {
        }
    }

    return;
}

int32_t EnetAppUtils_addAllPortMcastMembership(Enet_Handle hEnet, uint8_t *mcastMacAddr)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    CpswAle_SetMcastEntryInArgs setMcastInArgs;
    uint32_t setMcastOutArgs;
    uint32_t coreId = EnetSoc_getCoreId();

    memset(&setMcastInArgs, 0, sizeof(setMcastInArgs));
    memcpy(&(setMcastInArgs.addr.addr[0U]), &(mcastMacAddr[0U]), sizeof(setMcastInArgs.addr.addr));
    setMcastInArgs.info.super = false;
    setMcastInArgs.info.numIgnBits = 0U;
    setMcastInArgs.info.fwdState = CPSW_ALE_FWDSTLVL_FWD;
    setMcastInArgs.info.portMask = CPSW_ALE_ALL_PORTS_MASK;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastOutArgs);
    ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_MCAST, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("failed to add a new mcast entry to ALE table: %d\n", status);
    }
    return status;
}

int32_t EnetAppUtils_delAllPortMcastMembership(Enet_Handle hEnet, uint8_t *mcastMacAddr)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    CpswAle_MacAddrInfo delMcastInArgs;
    uint32_t coreId = EnetSoc_getCoreId();

    memset(&delMcastInArgs, 0, sizeof(delMcastInArgs));
    memcpy(&(delMcastInArgs.addr[0U]), &(mcastMacAddr[0U]), sizeof(delMcastInArgs.addr));
    delMcastInArgs.vlanId = 0U;
    ENET_IOCTL_SET_IN_ARGS(&prms, &delMcastInArgs);
    ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_REMOVE_ADDR, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("failed to remove the mcast entry from ALE table: %d\n", status);
    }

    return status;
}

/* end of file */

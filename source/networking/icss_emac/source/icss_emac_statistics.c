/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <stdbool.h>
#include "icss_emac_statistics.h"
#include "icss_emac_learning.h"
#include "icss_emac_local.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
* @def MAX_NUM_PROTOCOL_IMPLEMENTED
*       Max Number of protocols
*/
#define MAX_NUM_PROTOCOL_IMPLEMENTED   (50U)
/**
* @def NUM_PROTOCOLS_IMPLEMENTED
*      Number of protocols supported
*/
#define NUM_PROTOCOLS_IMPLEMENTED  (2U)
/**
* @def IP4_PROT_TYPE
*      IP4 Protcol type
*/
#define IP4_PROT_TYPE    (0x800)
/**
* @def ARP_PROT_TYPE
*      ARP protocol type
*/
#define ARP_PROT_TYPE    (0x806)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** Variable containing list of implemented protocols*/
uint16_t numImplementedProtocols = NUM_PROTOCOLS_IMPLEMENTED;
/**list of identified protocol types, rest initialized to zero*/
uint16_t protocol_impl[MAX_NUM_PROTOCOL_IMPLEMENTED] = {IP4_PROT_TYPE,
                                                        ARP_PROT_TYPE,
                                                        0,0,0,0,0,0,0,0,0,0,0,0,
                                                        0,0,0,0,0,0,0,0,0,0,0,0,
                                                        0,0,0,0,0,0,0,0,0,0,0,0,
                                                        0,0,0,0,0,0,0,0,0,0,0,0};

/* TODO: Should we keep this*/
#ifdef TEST_DEBUG
int doNotUpdateStatsTX[NUM_PORTS];
int doNotUpdateStatsRX[NUM_PORTS];
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* TODO: Review this function */
void ICSS_EMAC_readStats(ICSS_EMAC_Handle           icssEmacHandle,
                         uint8_t                    portNum,
                         ICSS_EMAC_PruStatistics    *pruStatStructPtr)
{
    volatile uint8_t        *statsPointer;
    uint32_t                statisticsOffset = 0U;
    uint32_t                statsSize = 0U;
    uint32_t                temp_addr = 0U;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    statisticsOffset = ((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap.statisticsOffset;
    statsSize = ((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap.statisticsSize;

    if(ICSS_EMAC_MODE_SWITCH != ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
    {
        portNum = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask;
    }

    if((uint8_t)ICSS_EMAC_PORT_1 == portNum)
    {
        temp_addr = (pruicssHwAttrs->pru0DramBase + statisticsOffset);
        statsPointer = (uint8_t*)(temp_addr);
    }
    else
    {
        temp_addr = (pruicssHwAttrs->pru1DramBase + statisticsOffset);
        statsPointer = (uint8_t*)(temp_addr);
    }

    memset((void* )pruStatStructPtr, 0x0, (size_t)sizeof(ICSS_EMAC_PruStatistics));

    memcpy((void *)pruStatStructPtr, (const void *)statsPointer, (size_t)statsSize);

    return;
}

/* TODO: Review this function */
void ICSS_EMAC_purgeStats(ICSS_EMAC_Handle icssEmacHandle, uint8_t portNum)
{
    uint32_t                    *statsPointer = NULL;
    uint32_t                    temp_addr = 0U;
    ICSS_EMAC_HostStatistics    *hostStatsPtr;
    ICSS_EMAC_FwStaticMmap      *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);
    PRUICSS_Handle              pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const       *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    hostStatsPtr = (ICSS_EMAC_HostStatistics *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->hostStat));

    if(ICSS_EMAC_MODE_SWITCH != ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
    {
        portNum = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask;
    }
    else
    {
        hostStatsPtr += (portNum - 1U);
    }

    if((uint8_t)ICSS_EMAC_PORT_1 == portNum)
    {
        temp_addr = (pruicssHwAttrs->pru0DramBase + pStaticMMap->statisticsOffset);
        statsPointer = (uint32_t *)(temp_addr);
    }
    else
    {
        temp_addr = (pruicssHwAttrs->pru1DramBase + pStaticMMap->statisticsOffset);
        statsPointer = (uint32_t *)(temp_addr);
    }

    /*clear PRU  stats*/
    memset(statsPointer, 0x0, (size_t)(pStaticMMap->statisticsSize));

    /*clear port stats*/
    memset(hostStatsPtr, 0x0, (size_t)sizeof(ICSS_EMAC_HostStatistics));
}

void ICSS_EMAC_initStats(ICSS_EMAC_Handle icssEmacHandle, uint8_t portNum)
{
    ICSS_EMAC_purgeStats(icssEmacHandle, portNum);

/*TODO: Review this*/
#ifdef TEST_DEBUG
    doNotUpdateStatsTX[HWPORT0] = 0;
    doNotUpdateStatsTX[HWPORT1] = 0;

    doNotUpdateStatsRX[HWPORT0] = 0;
    doNotUpdateStatsRX[HWPORT1] = 0;
#endif
}

void ICSS_EMAC_updateRxStats(const uint8_t              *macAddr,
                             uint32_t                   packet_len,
                             uint16_t                   protIdent,
                             ICSS_EMAC_HostStatistics   *hostStatsPtr)
{
    uint8_t count=0;
    bool    prot_found = 0;
    uint8_t Bcast_mac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

#ifdef TEST_DEBUG
    /*This is to make sure that the stats query and other custom queries are not counted  */
    if(doNotUpdateStatsRX[portNum-1])
    {
        /*clear the flag  */
        doNotUpdateStatsRX[portNum-1] = 0;
        return;
    }
#endif

    /*Broadcast  */
    if(COMPARE_MAC(macAddr, Bcast_mac))
    {
        hostStatsPtr->rxBcast++;
    }
    else if((macAddr[0] & 0x01U) == 1u)      /*Multicast*/
    {
        hostStatsPtr->rxMcast++;
    }
    else /*unicast*/
    {
        hostStatsPtr->rxUcast++;

    }

    hostStatsPtr->rxOctets += (packet_len+4U);

    for(count=0; count < numImplementedProtocols; count++)
    {
        if(protIdent == protocol_impl[count])
        {
            prot_found = (bool)true;
            break;
        }
    }

    if (!prot_found)
    {
        hostStatsPtr->rxUnknownProtocol++;
    }
}

void ICSS_EMAC_updateTxStats(const uint8_t              *macAddr,
                             uint32_t                   packet_len,
                             ICSS_EMAC_HostStatistics   *hostStatsPtr)
{

#ifdef TEST_DEBUG
    /*This is to make sure that the stats query and other custom queries are not counted   */
    if(doNotUpdateStatsTX[portNum-1])
    {
        /*clear the flag  */
        doNotUpdateStatsTX[portNum-1] = 0;
        return;
    }
#endif

    uint8_t Bcast_mac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

    if(COMPARE_MAC(macAddr, Bcast_mac))
    {
        hostStatsPtr->txBcast++;
    }
    else if((macAddr[0] & 0x01U) == 1u)
    {
        hostStatsPtr->txMcast++;
    }
    else
    {
        hostStatsPtr->txUcast++;

    }
    hostStatsPtr->txOctets += (packet_len+4U);
}


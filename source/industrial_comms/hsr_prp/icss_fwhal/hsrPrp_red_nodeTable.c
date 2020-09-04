/**
 * \file hsrPrp_red_nodeTable.c
 * \brief Contains Node Table management routines
 *
 * \par
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
 * \par
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/TaskP.h>

#include <networking/icss_emac/icss_emac.h>
#include <networking/icss_emac/source/icss_emac_local.h>



#include "hsrPrp_red_nodeTable.h"
#include "hsrPrp_firmwareOffsets.h"
#include "hsrPrp_handle.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define RED_NODETABLE_CHECK_TASK_TIMER          (10)  /* 10 ms */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  \brief Converts MAC address from PRU convention to Host one
 *  \param pMac pointer to MAC address
 *  \return None
 *
 */

char *getNodeTypePrintStr(uint8_t nodeEntryStatus);

static void pruToHostMac(uint8_t *pMac)
{
    if(pMac != NULL)
    {
        uint8_t tmp;

        tmp = pMac[0];
        pMac[0] = pMac[3];
        pMac[3] = tmp;

        tmp = pMac[1];
        pMac[1] = pMac[2];
        pMac[2] = tmp;

        tmp = pMac[4];
        pMac[4] = pMac[5];
        pMac[5] = tmp;
    }
}

uint32_t RedGetNodesCount(PRUICSS_Handle pruicssHandle)
{

    uint32_t *lreCntNodes = ((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                          LRE_CNT_NODES)));

    return *lreCntNodes;
}

RED_STATUS RedGetNodeTableEntry(RED_NODE_TABLE *pNodeTable,
                                hsrPrpHandle *hsrPrphandle)
{
    RED_BIN_ARRAY_ENTRY *binArrayBase = hsrPrphandle->binArrayBase;
    RED_BIN_ARRAY_ENTRY *pbinArrayNode;

    RED_NODE_TABLE_ENTRY *nodeTableBase = hsrPrphandle->nodeTableBase;
    RED_NODE_TABLE_ENTRY *pnodeTableNode;

    uint16_t i, j;

    for(i = 0, j = 0; i < BIN_ARRAY_MAX_ENTRIES; i++)
    {
        pbinArrayNode = binArrayBase + i;

        if(pbinArrayNode->nodetable_offset != CONST_NODETABLE_INVALID)
        {
            pnodeTableNode = nodeTableBase + pbinArrayNode->nodetable_offset;

            memcpy(pnodeTableNode->src, pbinArrayNode->MacId , ETHER_ADDR_LEN);
            pruToHostMac(pnodeTableNode->src);
            memcpy(&pNodeTable->entries[j++], pnodeTableNode, sizeof(RED_NODE_TABLE_ENTRY));
        }
    }

    return (RED_OK);
}

RemNodeType_t RedGetRemNodeType(uint8_t nodeEntryStatus)
{
    uint8_t remNodeType = (nodeEntryStatus & NT_REM_NODE_TYPE_MASK) >>
                        NT_REM_NODE_TYPE_SHIFT;
    uint8_t hsrBitSet   = (nodeEntryStatus & NT_REM_NODE_HSR_BIT);

    switch(remNodeType)
    {
        case NT_REM_NODE_TYPE_SANA:
        case NT_REM_NODE_TYPE_SANB:
        case NT_REM_NODE_TYPE_SANAB:
            return (RED_REM_NODE_TYPE_SAN);

        case NT_REM_NODE_TYPE_DAN:
            return ((hsrBitSet) ? (RED_REM_NODE_TYPE_DANH) : (RED_REM_NODE_TYPE_DANP));

        case NT_REM_NODE_TYPE_REDBOX:
            return ((hsrBitSet) ? (RED_REM_NODE_TYPE_REDBOXH) :
                    (RED_REM_NODE_TYPE_REDBOXP));

        case NT_REM_NODE_TYPE_VDAN:
            return ((hsrBitSet) ? (RED_REM_NODE_TYPE_VDANH) : (RED_REM_NODE_TYPE_VDANP));

        default:
            break;
    }

    return (RED_REM_NODE_TYPE_UNKNOWN);
}

RED_STATUS RedInsertNodeTable(uint8_t port, uint8_t isSupFrame,
                              hsrPrpHandle *hsrPrphandle, uint16_t flags)
{
    PRUICSS_Handle pruicssHandle = ((ICSS_EMAC_Object *)hsrPrphandle->icssEmacHandle->object)->pruicssHandle;

    RED_NODE_TABLE_ENTRY *nodeTableBase = hsrPrphandle->nodeTableBase;
    uint16_t *nextFreeSlot = (uint16_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                          NEXT_FREE_SLOT);

    uint32_t *lreCntNodes = ((uint32_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                          LRE_CNT_NODES));

    (*lreCntNodes)++;

    /*Initializing a node table entry*/
    (nodeTableBase + *nextFreeSlot)->state = 0x01
            ;  /*0x01: entry is valid | 0x10: entry is not valid */

    (nodeTableBase + *nextFreeSlot)->timeLasSeenS = 0 ;
    (nodeTableBase + *nextFreeSlot)->timeLasSeenA = 0 ;
    (nodeTableBase + *nextFreeSlot)->timeLasSeenB = 0 ;

    (nodeTableBase + *nextFreeSlot)->errRxA = 0 ;
    (nodeTableBase + *nextFreeSlot)->errRxB = 0 ;

    if(port == 0x01)        /*Received on port A*/
    {
#ifdef ICSS_PROTOCOL_PRP
        (nodeTableBase + *nextFreeSlot)->status = NT_REM_NODE_TYPE_SANA;
#endif
        (nodeTableBase + *nextFreeSlot)->cntRxA = 1 ;

        if(isSupFrame)
        {
            (nodeTableBase + *nextFreeSlot)->cntRxSupA = 1 ;
        }

        else
        {
            (nodeTableBase + *nextFreeSlot)->cntRxSupA = 0 ;
        }

        (nodeTableBase + *nextFreeSlot)->cntRxB = 0 ;
        (nodeTableBase + *nextFreeSlot)->cntRxSupB = 0 ;
    }

    else                    /*Received on port B*/
    {
#ifdef ICSS_PROTOCOL_PRP
        (nodeTableBase + *nextFreeSlot)->status = NT_REM_NODE_TYPE_SANB;
#endif
        (nodeTableBase + *nextFreeSlot)->cntRxA = 0 ;
        (nodeTableBase + *nextFreeSlot)->cntRxSupA = 0 ;

        (nodeTableBase + *nextFreeSlot)->cntRxB = 1 ;

        if(isSupFrame)
        {
            (nodeTableBase + *nextFreeSlot)->cntRxSupB = 1 ;
        }

        else
        {
            (nodeTableBase + *nextFreeSlot)->cntRxSupB = 0 ;
        }
    }

    if(isSupFrame)
    {
#ifdef ICSS_PROTOCOL_PRP
        (nodeTableBase + *nextFreeSlot)->status = NT_REM_NODE_TYPE_DAN ;
#else
        (nodeTableBase + *nextFreeSlot)->status = NT_REM_NODE_TYPE_DAN_HSR ;
#endif
        if (flags & (1 << 2))
        {
#ifdef ICSS_PROTOCOL_PRP
            (nodeTableBase + *nextFreeSlot)->status = NT_REM_NODE_TYPE_REDBOX ;
#else
            (nodeTableBase + *nextFreeSlot)->status = NT_REM_NODE_TYPE_RBX_HSR;
#endif
        }
        else
        {
            if (flags & (1 << 3))
            {
#ifdef ICSS_PROTOCOL_PRP
                (nodeTableBase + *nextFreeSlot)->status = NT_REM_NODE_TYPE_VDAN ;
#else
                (nodeTableBase + *nextFreeSlot)->status = NT_REM_NODE_TYPE_VDAN_HSR;
#endif
            }
        }
    }

    return (RED_OK);
}

bool RedNodetableLinearSearch(uint16_t start, uint16_t size,
                              uint8_t *macid,
                              uint16_t *index,
                              hsrPrpHandle *hsrPrphandle)
{
    RED_BIN_ARRAY_ENTRY *binArrayBase = hsrPrphandle->binArrayBase;
    uint16_t i, j, flag;

    for(i = 0; i < size; i++)
    {
        flag = 0;

        for(j = 0; j < ETHER_ADDR_LEN; j++)
        {
            if(((binArrayBase + start + i)->nodetable_offset !=
                    (NODE_TABLE_NT_MAX_ENTRIES + 1))
                    && ((binArrayBase + start + i)->MacId[j] == macid[j]))
            {
                flag++;
            }
        }

        if(ETHER_ADDR_LEN == flag)
        {
            *index = start + i;
            return 0;
        }
    }

    *index = start + i;
    return 1;
}

RED_STATUS RedNodetableSearchOp(uint8_t *srcMacId, uint8_t port,
                                uint8_t isSupFrame,
                                hsrPrpHandle *hsrPrphandle, uint16_t flags)
{
    PRUICSS_Handle pruicssHandle = ((ICSS_EMAC_Object *)hsrPrphandle->icssEmacHandle->object)->pruicssHandle;

    uint8_t macid[ETHER_ADDR_LEN];
    uint16_t i, hashVal, entries, index = 0;
    Bool searchStatus = 0;
    RED_STATUS returnStatus = RED_OK;

    RED_INDEX_ARRAY_ENTRY *indexArrayBase = hsrPrphandle->indexArrayBase;
    RED_INDEX_ARRAY_ENTRY *pindexArrayNode;

    RED_BIN_ARRAY_ENTRY *binArrayBase = hsrPrphandle->binArrayBase;
    RED_BIN_ARRAY_ENTRY *pbinArrayNode;

    RED_NODE_TABLE_ENTRY *nodeTableBase = hsrPrphandle->nodeTableBase;

    uint16_t *nextFreeSlot = (uint16_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                          NEXT_FREE_SLOT);

    volatile uint8_t *binArrayLock1 = ((uint8_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                      BIN_ARRAY_LOCK));

    volatile uint16_t *binArrayLock2 = ((uint16_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                       BIN_ARRAY_LOCK_FIRMWARE_PRU0));

    /*check for proper initialization*/
    if(indexArrayBase == NULL)
    {
        return RED_ERR;
    }

    if(binArrayBase == NULL)
    {
        return RED_ERR;
    }

    if(nodeTableBase == NULL)
    {
        return RED_ERR;
    }

    if(nextFreeSlot == NULL)
    {
        return RED_ERR;
    }

    /*Applying lock with other host task*/
    SemaphoreP_pend(&(hsrPrphandle->nodesTableSemaphore), SystemP_WAIT_FOREVER);

    /*tempFrame contains complete packet received & source MacID is required which occurs at offset of 6 from beginning of the packet*/
    memcpy(&macid, srcMacId, ETHER_ADDR_LEN);
    pruToHostMac((uint8_t *)&macid);

    /* compute the hashVal by XORing all 6 bytes of macid*/
    for(i = 0, hashVal = 0; i < ETHER_ADDR_LEN; i++)
    {
        hashVal = hashVal ^ macid[i];
    }

    hashVal = hashVal & HASHVAL_MASK;

    pindexArrayNode = indexArrayBase + hashVal;
#if DEBUG_HSR
    DebugP_log("hashVal : %x ", hashVal);
#endif

    if(0x0 == pindexArrayNode->bitLinBin)    // 0x0 : Linear search
    {
        /*
        PARAMETERS FOR LINEAR SEARCH
            pindexArrayNode->bin_offset : starting position for search operation
            pindexArrayNode->binNoEntries : no of entries to be searched
            macid : MAC ID to be searched
            &index : will contain position at which hit occurred / insertion to be made
            icssEmacHandle : for base addresses
         */
        searchStatus = RedNodetableLinearSearch(pindexArrayNode->bin_offset,
                                                pindexArrayNode->binNoEntries, macid, &index, hsrPrphandle);
#if DEBUG_HSR

        if(index >= 256)
        {
            DebugP_log("INVALID INDEX 1 %x\n", index);
        }

#endif
    }

    else
    {
        /*searchStatus = binarySearch(pindexArrayNode->bin_offset, pindexArrayNode->binNoEntries, macid, &index );*/
    }

#ifdef DEBUG
    DebugP_log("searchStatus : %d\n", searchStatus);
#endif
    /* if searchStatus is 0 => hit & index contains the offset at which hit occurred in binArray
             =>do nothing as update handled by NODETABLE_UPDATE in firmware
       if searchStatus is 1 => miss & index contains the offset at which entry to be made in binArray
             =>make an insertion
    */

    if(1 == searchStatus)
    {

        /*check for availability in nodetable*/
        *nextFreeSlot = CONST_NODETABLE_INVALID ;

        for(i = 0; i < NODE_TABLE_NT_MAX_ENTRIES; i++)
        {
            if((nodeTableBase + i)->state == 0x10)
            {
                *nextFreeSlot = i ;
                break;
            }
        }

#if DEBUG_HSR
        DebugP_log("Inserting :");

        for(i = 0; i < ETHER_ADDR_LEN; i++)
        {
            DebugP_log("%x ", macid[i]);
        }

        DebugP_log("->%x\n", hashVal);
#endif

        /*check for nodetable already full*/
        if(*nextFreeSlot == CONST_NODETABLE_INVALID)
        {
#if DEBUG_HSR
            uint16_t ctr;
            DebugP_log("Nodetable full!!!\n");

            for(i = 0, ctr = 0; i < BIN_ARRAY_MAX_ENTRIES; i++)
                if((binArrayBase + i)->nodetable_offset != CONST_NODETABLE_INVALID)
                {
                    ctr++;
                }

            if(ctr != BIN_ARRAY_MAX_ENTRIES)
            {
                DebugP_log("But binArray not full!!!\n");
            }

            for(i = 0, ctr = 0; i < INDEX_TABLE_MAX_ENTRIES; i++)
            {
                ctr += (indexArrayBase + i)->binNoEntries;
            }

            if(ctr != INDEX_TABLE_MAX_ENTRIES)
            {
                DebugP_log("But indexArray doesnt match!!!\n");
            }

#endif

            /*releasing the lock*/
            *binArrayLock1 = 0x0;
            SemaphoreP_post(&(hsrPrphandle->nodesTableSemaphore));

            return RED_ERR;
        }


#if DEBUG_HSR
        uint16_t ctr;

        for(i = 0, ctr = 0; i < BIN_ARRAY_MAX_ENTRIES; i++)
            if((binArrayBase + i)->nodetable_offset != CONST_NODETABLE_INVALID)
            {
                ctr++;
            }

        uint16_t ctr2;

        for(i = 0, ctr2 = 0; i < INDEX_TABLE_MAX_ENTRIES; i++)
        {
            ctr2 += (indexArrayBase + i)->binNoEntries;
        }

#endif

        /*Applying the lock with firmware*/
        while(1)
        {
            *binArrayLock1 = 0x1;

            if(*binArrayLock2 == 0x0)
            {
                break;
            }

            *binArrayLock1 = 0x0;
        }

        /*entry needs to be made in this bin => no of entries in this bin have gone up by 1*/
        pindexArrayNode->binNoEntries++ ;

#if DEBUG_HSR

        if(index >= 256)
        {
            DebugP_log("INVALID INDEX 2");
        }

#endif

        /*computing number of entries for movement (index+1)->(NODE_TABLE_NT_MAX_ENTRIES-1)*/
        entries = NODE_TABLE_NT_MAX_ENTRIES - index - 1;

        if(index + 1 < BIN_ARRAY_MAX_ENTRIES)
        {
            memmove(binArrayBase + index + 1, binArrayBase + index, 8 * entries);
        }

        /*since all bin entries corresponding to hashVal+1 till end have moved down by one, update the bin_offset accordingly*/
        for(i = hashVal + 1; i < NODE_TABLE_NT_MAX_ENTRIES; i++)
        {
            (indexArrayBase + i)->bin_offset++;
        }

        /* space created at index, now populate the values*/
        pbinArrayNode = binArrayBase + index ;
        pbinArrayNode->nodetable_offset = *nextFreeSlot;

        for(i = 0; i < ETHER_ADDR_LEN; i++)
        {
            pbinArrayNode->MacId[i] = macid[i];
        }

        /* make the nodetable insertion*/
        returnStatus = RedInsertNodeTable(port, isSupFrame, hsrPrphandle, flags);

        if(returnStatus != RED_OK)
        {
            return (RED_ERR);
        }

        /*releasing the lock*/
        *binArrayLock1 = 0x0;
    }

    SemaphoreP_post(&(hsrPrphandle->nodesTableSemaphore));

    return (RED_OK);
}

void RedNodetableRefresh(void *args)
{
    hsrPrpHandle *hsrPrphandle = (hsrPrpHandle *)args ;
    PRUICSS_Handle pruicssHandle = ((ICSS_EMAC_Object *)hsrPrphandle->icssEmacHandle->object)->pruicssHandle;

    RED_INDEX_ARRAY_ENTRY *indexArrayBase = hsrPrphandle->indexArrayBase;
    RED_INDEX_ARRAY_ENTRY *pindexArrayNode;

    RED_BIN_ARRAY_ENTRY *binArrayBase = hsrPrphandle->binArrayBase;
    RED_BIN_ARRAY_ENTRY *pbinArrayNode;

    RED_NODE_TABLE_ENTRY *nodeTableBase = hsrPrphandle->nodeTableBase;
    RED_NODE_TABLE_ENTRY *pnodeTableNode;

    uint32_t nodeForgetTime = *((uint32_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase +
                                            NODE_FORGET_TIME));

    uint32_t *lreCntNodes = ((uint32_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                          LRE_CNT_NODES));
    *lreCntNodes = 0;

    uint16_t i, p, hashVal, entries;

    uint16_t *nextFreeSlot = ((uint16_t *)(((PRUICSS_HwAttrs const *)
                                pruicssHandle->hwAttrs)->sharedDramBase +
                                          NEXT_FREE_SLOT));
    *nextFreeSlot = 0;

    volatile uint8_t *binArrayLock1 = ((uint8_t *)(((PRUICSS_HwAttrs const *)
                                        pruicssHandle->hwAttrs)->sharedDramBase +
                                      BIN_ARRAY_LOCK));
    *binArrayLock1 = 0;

    volatile uint16_t *binArrayLock2 = ((uint16_t *)(((PRUICSS_HwAttrs const *)
                                        pruicssHandle->hwAttrs)->sharedDramBase +
                                       BIN_ARRAY_LOCK_FIRMWARE_PRU0));
    *binArrayLock2 = 0;

    char *nodeType;

    uint8_t eligibleNodeForRemoval;

    /*loop to initialize pindexArrayNode entries*/
    for(i = 0; i < INDEX_TABLE_MAX_ENTRIES; i++)
    {
        (indexArrayBase + i)->bin_offset = 0 ;
        (indexArrayBase + i)->binNoEntries = 0;
        (indexArrayBase + i)->bitLinBin = 0;
    }

    /*loop to initialize pbinArrayNode->nodetable_offset to INVALID state */
    for(i = 0; i < BIN_ARRAY_MAX_ENTRIES; i++)
    {
        (binArrayBase + i)->nodetable_offset = CONST_NODETABLE_INVALID ;
    }

    /*loop to initialize state to 0x10: entry is not valid  */
    for(i = 0; i < NODE_TABLE_NT_MAX_ENTRIES; i++)
    {
        (nodeTableBase + i)->state = 0x10 ;
    }

    SemaphoreP_post(&(hsrPrphandle->nodesTableSemaphore));

    while(1)
    {

        /*Applying lock with other host task*/
        SemaphoreP_pend(&(hsrPrphandle->nodesTableSemaphore), SystemP_WAIT_FOREVER);

        /*Applying the lock with firmware*/
        while(1)
        {
            *binArrayLock1 = 0x1;

            if(*binArrayLock2 == 0x0)
            {
                break;
            }

            *binArrayLock1 = 0x0;
        }

        /*loop to increment all timeLasSeenX by 1*/
        for(i = 0; i < NODE_TABLE_NT_MAX_ENTRIES; i++)
        {
            pbinArrayNode = binArrayBase + i ;

            if(pbinArrayNode->nodetable_offset != CONST_NODETABLE_INVALID)
            {
                pnodeTableNode = nodeTableBase + pbinArrayNode->nodetable_offset ;

                char *nodeType = (char *)getNodeTypePrintStr(pnodeTableNode->status);

                (pnodeTableNode->timeLasSeenA >= MAX_FORGET_TIME_BEFORE_WRAP) ?
                (pnodeTableNode->timeLasSeenA = MAX_FORGET_TIME_BEFORE_WRAP) :
                (pnodeTableNode->timeLasSeenA++);
                (pnodeTableNode->timeLasSeenB >= MAX_FORGET_TIME_BEFORE_WRAP) ?
                (pnodeTableNode->timeLasSeenB = MAX_FORGET_TIME_BEFORE_WRAP) :
                (pnodeTableNode->timeLasSeenB++);

                /*check if the node is SAN | if not SAN => increment the timeLasSeenS counter*/
                if(strstr(nodeType, "SAN") == NULL)
                {
                    (pnodeTableNode->timeLasSeenS >= MAX_FORGET_TIME_BEFORE_WRAP) ?
                    (pnodeTableNode->timeLasSeenS = MAX_FORGET_TIME_BEFORE_WRAP) :
                    (pnodeTableNode->timeLasSeenS++);
                }
            }
        }

        /*releasing the lock*/
        *binArrayLock1 = 0x0;
        SemaphoreP_post(&(hsrPrphandle->nodesTableSemaphore));


        /*Applying lock with other host task*/
        SemaphoreP_pend(&(hsrPrphandle->nodesTableSemaphore), SystemP_WAIT_FOREVER);

        /*loop to remove a node reaching NODE_FORGET_TIME*/
        for(i = 0; i < BIN_ARRAY_MAX_ENTRIES; i++)
        {
            pbinArrayNode = binArrayBase + i ;

            if(pbinArrayNode->nodetable_offset != CONST_NODETABLE_INVALID)
            {
                pnodeTableNode = nodeTableBase + pbinArrayNode->nodetable_offset ;

                nodeType = (char *)getNodeTypePrintStr(pnodeTableNode->status);
                eligibleNodeForRemoval = 0;

                if(strstr(nodeType, "SAN") == NULL)
                {
                    if(pnodeTableNode->timeLasSeenS > nodeForgetTime
                            && pnodeTableNode->timeLasSeenA > nodeForgetTime
                            && pnodeTableNode->timeLasSeenB > nodeForgetTime)
                    {
                        eligibleNodeForRemoval = 1;
                    }
                }

                else
                {
                    if(pnodeTableNode->timeLasSeenA > nodeForgetTime
                            && pnodeTableNode->timeLasSeenB > nodeForgetTime)
                    {
                        eligibleNodeForRemoval = 1;
                    }
                }

                if(eligibleNodeForRemoval)
                {
                    /*Applying the lock with firmware*/
                    while(1)
                    {
                        *binArrayLock1 = 0x1;

                        if(*binArrayLock2 == 0x0)
                        {
                            break;
                        }

                        *binArrayLock1 = 0x0;
                    }

                    /*set nodetable entry state to INVALID*/
                    pnodeTableNode->state = 0x10;

                    /*Compute the hashVal by XORing all 6 bytes of srcMacId*/
                    for(p = 0, hashVal = 0; p < ETHER_ADDR_LEN; p++)
                    {
                        hashVal = hashVal ^ pbinArrayNode->MacId[p];
                    }

                    hashVal = hashVal & HASHVAL_MASK;

#if DEBUG_HSR
                    DebugP_log("Deleting:");

                    for(p = 0; p < ETHER_ADDR_LEN; p++)
                    {
                        DebugP_log("%x ", pbinArrayNode->MacId[p]);
                    }

                    DebugP_log("-> %x\n", hashVal);
#endif

                    pindexArrayNode = indexArrayBase + hashVal;

                    /*computing number of entries for movement (index+1)->(NODE_TABLE_NT_MAX_ENTRIES-1)*/
                    entries = NODE_TABLE_NT_MAX_ENTRIES - i - 1;

                    if(i + 1 < BIN_ARRAY_MAX_ENTRIES)
                    {
                        memmove(binArrayBase + i, binArrayBase + i + 1, 8 * entries);
                    }

                    /*since all bin entries corresponding to hashVal+1 till end have moved up by one, update the bin_offset accordingly*/
                    for(p = hashVal + 1; p < NODE_TABLE_NT_MAX_ENTRIES; p++)
                    {
                        (indexArrayBase + p)->bin_offset--;
                    }

                    /*reset the last binArray entry*/
                    (binArrayBase + BIN_ARRAY_MAX_ENTRIES - 1)->nodetable_offset =
                        CONST_NODETABLE_INVALID;

                    for(p = 0; p < ETHER_ADDR_LEN; p++)
                    {
                        (binArrayBase + BIN_ARRAY_MAX_ENTRIES - 1)->MacId[p] = 0x00;
                    }

#if DEBUG_HSR

                    if((indexArrayBase + INDEX_TABLE_MAX_ENTRIES - 1)->bin_offset +
                            (indexArrayBase + INDEX_TABLE_MAX_ENTRIES - 1)->binNoEntries >
                            INDEX_TABLE_MAX_ENTRIES)
                    {
                        DebugP_log("Something is wrong!!!%x %d\n",
                                    (indexArrayBase + INDEX_TABLE_MAX_ENTRIES - 1)->bin_offset,
                                    (indexArrayBase + INDEX_TABLE_MAX_ENTRIES - 1)->binNoEntries);
                    }

#endif

                    (pindexArrayNode->binNoEntries > 0) ? (pindexArrayNode->binNoEntries--) :
                    (pindexArrayNode->binNoEntries = 0);

                    (*lreCntNodes)--;

#if DEBUG_HSR
                    uint16_t eba, ent;

                    for(i = 0, eba = 0; i < BIN_ARRAY_MAX_ENTRIES; i++)
                        if((binArrayBase + i)->nodetable_offset != CONST_NODETABLE_INVALID)
                        {
                            eba++;
                        }

                    for(i = 0, ent = 0; i < NODE_TABLE_NT_MAX_ENTRIES; i++)
                        if((nodeTableBase + i)->state == 0x01)
                        {
                            ent++;
                        }

                    if(ent != eba)
                    {
                        DebugP_log("mismatch ent:%x eba:%x\n", ent, eba);
                    }

#endif
                    /*releasing the lock*/
                    *binArrayLock1 = 0x0;
                }
            }
        }

        SemaphoreP_post(&(hsrPrphandle->nodesTableSemaphore));

        /*sleep for 10 ms*/
        ClockP_usleep(ClockP_ticksToUsec(RED_NODETABLE_CHECK_TASK_TIMER));
    }
}

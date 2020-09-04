/**
 * \file hsrPrp_red_nodeTable.h
 * \brief Include file for hsrPrp_red_nodeTable.c
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

#ifndef RED_NODETABLE_H_
#define RED_NODETABLE_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "hsrPrp_red_common.h"
#include "hsrPrp_handle.h"
#include <stdbool.h>

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 *  \internal
 *  \brief Returns number of entries in the Node Table
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return Number of entries in the Node Table
 *
 */
uint32_t        RedGetNodesCount(PRUICSS_Handle pruicssHandle);
/**
 *  \internal
 *  \brief Gets the exact Node Table entry index
 *
 *  \param index index of an entry to read from
 *  \param pEntry pointer to memory where to store entry value
 *  \param icssEmacHandle Handle to ICSS EMAC instance. Contains pointers to base addresses
 *
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS    RedGetIndexArrayEntry(uint8_t index, uint8_t *pEntry,
                                    ICSS_EMAC_Handle icssEmacHandle);
/**
 *  \brief Gets the Node Table entry by an index return by redGetIndexArrayEntry
 *
 *  \param pNodeTable pointer to Node Table Structure
 *  \param hsrPrphandle
 *
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS RedGetNodeTableEntry(RED_NODE_TABLE *pNodeTable,
                                hsrPrpHandle *hsrPrphandle);
/**
 *  \brief Gets the type of a remote node
 *
 *  \param nodeEntryStatus value to evaluate
 *
 *  \return type of remote node
 *
 */
RemNodeType_t RedGetRemNodeType(uint8_t nodeEntryStatus);
/**
 *  \brief searches for the macid in the nodetable
 *
 *  \param srcMacId source MAC address
 *  \param port Port on which the frame was recieved
 *  \param isSupFrame Supervision frame indicator bit | set for incoming supervision frame
 *  \param hsrPrphandle Handle to hsrPrp driver instance.
 *  \param flags flag to pass info about node type
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS RedNodetableSearchOp(uint8_t *srcMacId , uint8_t port,
                                uint8_t isSupFrame,
                                hsrPrpHandle *hsrPrphandle, uint16_t flags);
/**
 *  \brief makes an insertion in the node table
 *
 *  \param port Port on which the frame was recieved
 *  \param isSupFrame Supervision frame indicator bit | set for incoming supervision frame
 *  \param hsrPrphandle Handle to hsrPrp driver instance.
 *  \param flags flag to pass info about node type
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS RedInsertNodeTable(uint8_t port, uint8_t isSupFrame,
                              hsrPrpHandle *hsrPrphandle, uint16_t flags);

/**
 *  \brief linear search implementation | search for the param macid in the BIN ARRAY where starting index for search given in param start & size of search given in param size
 *
 *  \param start staring location
 *  \param size no of entries to be searched
 *  \param macid MAC ID of the source of the packet
 *  \param index if HIT , returns the position in BIN ARRAY where entry was found | if MISS , position where entry to be made
 *  \param hsrPrphandle Handle to hsrPrp driver instance.
 *  \return bool TRUE if HIT | FALSE if MISS
 *
 */
bool RedNodetableLinearSearch(uint16_t start, uint16_t size,
                              uint8_t *macid,
                              uint16_t *index, hsrPrpHandle *hsrPrphandle);

/**
 *  \brief increment the timeLastSeenX values by 1 every 10 ms & delete a nodetable entry if it reaches NODE_FORGET_TIME
 *
 *  \param args args
 *
 */
void RedNodetableRefresh(void *args);


#ifdef __cplusplus
}
#endif

#endif /* RED_NODETABLE_H_ */

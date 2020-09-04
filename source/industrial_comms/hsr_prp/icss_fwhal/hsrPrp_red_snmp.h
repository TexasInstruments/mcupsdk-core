/**
 * \file hsrPrp_red_snmp.h
 * \brief Include file for hsrPrp_red_snmp.c
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

#ifndef RED_SNMP_H_
#define RED_SNMP_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "hsrPrp_red_common.h"
#include "hsrPrp_handle.h"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**< General group */
/**
 *  \brief Returns the manufacturer name.
 *
 *  \return NULL-terminated string.
 *
 */
const char               *getLreManufacturerName(void);
/**
 *  \brief Returns the number of supported LRE.
 *
 *  \return Always 1 for the AM335x PRP/HSR implementation.
 *
 */
int32_t                     getLreInterfaceCount(void);

/**< Interface group */
/**
 *  \brief Returns lreNodeType specified in LREInterfaceConfigEntry.
 *
 *  \return 1 prpmode1, 2 hsr.
 *
 */
LreNodeType_t             getLreNodeType(void);
/**
 *  \brief Returns the LRE's node name. Default: "IEC62439-3".
 *
 *  \return NULL-terminated string.
 *
 */
const char               *getLreNodeName(void);
/**
 *  \brief Returns the LRE's version information.
 *
 *  \return NULL-terminated string.
 *
 */
const char               *getLreVersionName(void);

/**
 *  \brief Returns the LRE admin state of a port.
 *
 *  \param port - port number
  *  \param  icssEmacHandle Handle to ICSS EMAC instance. Contains pointers to base addresses
 *
 *  \return notActive (1) or active(2).
 *
 */
int32_t                     getLrePortAdminState(RedPort_t port,
        ICSS_EMAC_Handle icssEmacHandle);
/**
 *  \brief Sets the LRE admin state of a port.
 *
 *  \param port - port number
 *  \param adminState - set value: notActive (1) or active(2)
 *
 *  \return RED_ERR, always
 *
 */
RED_STATUS                setLrePortAdminState(RedPort_t port,
        int32_t adminState);
/**
 *  \brief Returns whether a duplicate discard algorithm is used at reception.
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return doNotDiscard (1) or discard (2).
 *
 */
LreDuplicateDiscard_t     getLreDuplicateDiscard(PRUICSS_Handle
        pruicssHandle);
/**
 *  \brief Sets whether a duplicate discard algorithm is used at reception.
 *
 *  \param hsrPrphandle
 *  \param duplicateDiscard - set value: doNotDiscard (1) or discard (2)
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return RED_OK on Success, RED_ERR on Failure
 *
 */
RED_STATUS                setLreDuplicateDiscard(hsrPrpHandle *hsrPrphandle,
        LreDuplicateDiscard_t
        duplicateDiscard, PRUICSS_Handle pruicssHandle);
/**
 *  \internal
 *  \brief Returns removeRCT status. If removeRCT is configured,
 *         the RCT is removed when forwarding to the upper layers,
 *         only applicable for PRP LRE.
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return removeRCT (1) or passRCT (2).
 *
 */
LreTransparentReception_t getLreTransparentReception(PRUICSS_Handle
        pruicssHandle);
/**
 *  \brief Sets Transparent Reception.
 *
 *  \param transparentReception - set value: removeRCT (1) or passRCT (2)
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return RED_OK, always
 *
 */
RED_STATUS                setLreTransparentReception(LreTransparentReception_t
        transparentReception, PRUICSS_Handle pruicssHandle);
/**
 *  \brief Returns the HSR LRE Mode.
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return The HSR Mode.
 *
 */
HSRMode_t                 getLreHsrLREMode(PRUICSS_Handle pruicssHandle);
/**
 *  \brief Sets the HSR LRE Mode.
 *
 *  \param mode HSR mode to set.
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return RED_OK on Success, RED_ERR on Failure
 *
 */
RED_STATUS                setLreHsrLREMode(HSRMode_t mode,
        PRUICSS_Handle pruicssHandle);
/**
 *  \brief Returns the SwitchingEndNode status.
 *
 *  \return The switching end node functionality
 *
 */
SwitchingEndNode_t        getLreSwitchingEndNode(void);
/**
 *  \brief Returns the LRE RedBox Identity.
 *
 *  \return Always REDBOX_ID1A for the AM335x PRP/HSR implementation.
 *
 */
LreRedBoxIdentity_t       getLreRedBoxIdentity(void);
/**
 *  \brief True if the LRE evaluates received supervision frames.
 *         False if it drops the supervision frames without evaluating.
 *
 *  \return Always LRE_TRUE for the AM335x PRP/HSR implementation.
 *
 */
LreTruthValue_t           getLreEvaluateSupervision(void);
/**
 *  \brief Clears the node table if action is "clearNodeTable".
 *         No action is performed if action is noOp.
 *
 *  \param action - noOp (0), clearNodeTable (1)
 *
 *  \return RED_OK, always
 *
 */
RED_STATUS                setLreNodeTableClear(hsrPrpHandle *,
        LreTableOperation_t action);
/**
 *  \brief Clears the Proxy Node Table if action is "clearProxyNodeTable".
 *         No action is performed if action is noOp.
 *
 *  \param action - noOp (0), clearProxyNodeTable (1)
 *
 *  \return RED_ERR, always
 *
 */
RED_STATUS                setLreProxyNodeTableClear(LreTableOperation_t action);
/**
 *  \brief Returns a single entry of LREInterfaceStatsEntry (see IEC-62439-3-MIB DEFINITIONS).
 *         The requested entry is specified by stat.
 *
 *  \param stat - designates the LREInterfaceStatsEntry to be returned
 *  \param value - returns the MIB LREInterfaceStatsEntry designated by stat
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return RED_OK, always
 *
 */
RED_STATUS                getLreInterfaceStats(LreIfStat_t stat, int32_t *value,
        PRUICSS_Handle pruicssHandle);
/**
 *  \brief Returns the base address of the LREInterfaceStatsEntry (see IEC-62439-3-MIB DEFINITIONS)
 *         in the ICSS shared RAM. Using this base address, the host can read the whole
 *         LREInterfaceStatsEntry at once.
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return Base address of LREInterfaceStatsEntry in ICSS shared memory
 *
 */
const int32_t             *getLreInterfaceStatBase(PRUICSS_Handle
        pruicssHandle);
/**
 *  \brief Returns the node table size. Use this function to determine the size of the buffer to pass
 *         to getLreNodeTable.
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return The size of the node table in entries, -1 on Failure.
 *
 */
int32_t                    getLreNodeTableSize(PRUICSS_Handle pruicssHandle);

/**
 *  \brief Copies the complete node table to pNodeTable.
 *
 *  \param pNodeTable Pointer to memory capable of holding getLreNodeTableSize() node table entries.
 *  \param hsrPrphandle
 *
 *  \return Number of entries in the node table, -1 on Failure.
 *
 */
int32_t                   getLreNodeTable(RED_NODE_TABLE *pNodeTable,
        hsrPrpHandle *hsrPrphandle);
/**
*  \brief Gets the type of a remote node
*
*  \param nodeEntryStatus value to evaluate
*
*  \return type of remote node
*
*/
RemNodeType_t             getLreRemNodeType(uint8_t nodeEntryStatus);


#ifdef __cplusplus
}
#endif

#endif /* RED_SNMP_H_ */

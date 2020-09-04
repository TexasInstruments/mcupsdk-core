/**
 * \file hsrPrp_red_snmp.c
 * \brief Contains HSR PRP SNMP interface routines
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

#include <networking/icss_emac/icss_emac.h>
#include <networking/icss_emac/source/icss_emac_local.h>

#include "hsrPrp_firmwareOffsets.h"
#include "hsrPrp_red.h"
#include "hsrPrp_red_prp.h"
#include "hsrPrp_red_snmp.h"
#include "hsrPrp_red_nodeTable.h"
#include "hsrPrp_handle.h"

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

const char *getLreManufacturerName(void)
{
    return (RED_MANUFACTURER);
}

int32_t getLreInterfaceCount(void)
{
    return (1);
}

LreNodeType_t getLreNodeType(void)
{
#ifdef ICSS_PROTOCOL_PRP
    return (LRE_NODE_TYPE_PRP_MODE_1);
#else /* ICSS_PROTOCOL_PRP */
    return (LRE_NODE_TYPE_HSR);
#endif /* ICSS_PROTOCOL_PRP */
}

const char *getLreNodeName(void)
{
    return (RED_NODE_NAME);
}

const char *getLreVersionName(void)
{
    return (RED_VERSION_NAME);
}

int32_t getLrePortAdminState(RedPort_t port, ICSS_EMAC_Handle icssEmacHandle)
{
    return ((((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[port - 1]) ?
            LRE_PORT_ACTIVE : LRE_PORT_NOT_ACTIVE);
}

RED_STATUS setLrePortAdminState(RedPort_t port, int32_t adminState)
{

    return (RED_ERR);
}

LreDuplicateDiscard_t getLreDuplicateDiscard(PRUICSS_Handle pruicssHandle)
{
    return (LreDuplicateDiscard_t)(*((uint32_t *)(((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                                     LRE_DUPLICATE_DISCARD))));

}

RED_STATUS setLreDuplicateDiscard(hsrPrpHandle *hsrPrphandle,
                                  LreDuplicateDiscard_t duplicateDiscard,
                                  PRUICSS_Handle pruicssHandle)
{
#ifdef ICSS_PROTOCOL_PRP

    if(duplicateDiscard == LRE_DD_DO_NOT_DISCARD)
    {

        *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                       LRE_DUPLICATE_DISCARD)) = IEC62439_CONST_DUPLICATE_ACCEPT;
        RedSupFrameUpdateTlv(hsrPrphandle, PRP_TLV1_TYPE_DUP_ACCEPT);
    }

    else
    {

        *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                       LRE_DUPLICATE_DISCARD)) = IEC62439_CONST_DUPLICATE_DISCARD;
        RedSupFrameUpdateTlv(hsrPrphandle, PRP_TLV1_TYPE_DUP_DISCARD);
    }

    return (RED_OK);
#else /* ICSS_PROTOCOL_PRP */
    return (RED_ERR);
#endif /* ICSS_PROTOCOL_PRP */
}

LreTransparentReception_t getLreTransparentReception(PRUICSS_Handle
        pruicssHandle)
{

    return (LreTransparentReception_t)(*((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                                         LRE_TRANSPARENT_RECEPTION)));
}

RED_STATUS setLreTransparentReception(LreTransparentReception_t
                                      transparentReception, PRUICSS_Handle pruicssHandle)
{
    if(transparentReception == LRE_TR_REMOVE_RCT)
    {

        *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                       LRE_TRANSPARENT_RECEPTION)) = IEC62439_CONST_TRANSPARENT_RECEPTION_REMOVE_RCT;
    }

    else
    {

        *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                       LRE_TRANSPARENT_RECEPTION)) = IEC62439_CONST_TRANSPARENT_RECEPTION_PASS_RCT;
    }

    return (RED_OK);
}

HSRMode_t getLreHsrLREMode(PRUICSS_Handle pruicssHandle)
{
#ifdef ICSS_PROTOCOL_HSR
    int16_t reg;

    reg = *((uint16_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru0DramBase) + LRE_HSR_MODE));

    switch(reg)
    {
        case MODEH:
            return (HSR_COMMON_MODE_H);

        case MODEN:
            return (HSR_COMMON_MODE_N);

        case MODET:
            return (HSR_COMMON_MODE_T);

        case MODEU:
            return (HSR_COMMON_MODE_U);

        case MODEM:
            return (HSR_COMMON_MODE_M);

        default:
            break;
    }

#endif /* ICSS_PROTOCOL_HSR */

    return (HSR_COMMON_MODE_H);
}

RED_STATUS setLreHsrLREMode(HSRMode_t mode, PRUICSS_Handle pruicssHandle)
{
#ifdef ICSS_PROTOCOL_HSR

    volatile uint16_t *lreHSRModePtr = ((uint16_t *)((((PRUICSS_HwAttrs const *)pruicssHandle->hwAttrs)->pru0DramBase) + LRE_HSR_MODE));

    switch(mode)
    {
        case HSR_COMMON_MODE_H:

            *lreHSRModePtr = MODEH;
            break;

        case HSR_COMMON_MODE_N:

            *lreHSRModePtr = MODEN;
            break;

        case HSR_COMMON_MODE_T:

            *lreHSRModePtr = MODET;
            break;

        case HSR_COMMON_MODE_U:

            *lreHSRModePtr = MODEU;
            break;

        case HSR_COMMON_MODE_M:

            *lreHSRModePtr = MODEM;
            break;

        default:

            return (RED_ERR);
    }

    return (RED_OK);
#else /* ICSS_PROTOCOL_HSR */
    return (RED_ERR);
#endif /* ICSS_PROTOCOL_HSR */
}

SwitchingEndNode_t getLreSwitchingEndNode(void)
{
#ifdef ICSS_PROTOCOL_PRP
    return (SN_PRP_NODE);
#else
    return (SN_HSR_NODE);
#endif
}

LreRedBoxIdentity_t getLreRedBoxIdentity(void)
{
    return (REDBOX_ID1A);
}

LreTruthValue_t getLreEvaluateSupervision(void)
{
    return (LRE_TRUE);
}

RED_STATUS setLreNodeTableClear(hsrPrpHandle *hsrPrphandle,
                                LreTableOperation_t action)
{
    if(action == LRE_TABLE_CLEAR)
    {
        RedNodeTableClear(hsrPrphandle);
    }

    return (RED_OK);
}

RED_STATUS setLreProxyNodeTableClear(LreTableOperation_t action)
{
    return (RED_ERR);
}

RED_STATUS getLreInterfaceStats(LreIfStat_t stat, int32_t *value,
                                PRUICSS_Handle pruicssHandle)
{
    uint32_t *tmp;


    tmp = ((uint32_t *)(((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                       LRE_CNT_TX_A)));
    *value = *(tmp + stat);

    return (RED_OK);
}

const int32_t *getLreInterfaceStatBase(PRUICSS_Handle pruicssHandle)
{
    int32_t *returnPtr;
    returnPtr = (int32_t *)((uint32_t)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                                      LRE_CNT_TX_A));
    return returnPtr;
}

int32_t getLreNodeTableSize(PRUICSS_Handle pruicssHandle)
{
    uint32_t nodes;

    nodes = RedGetNodesCount(pruicssHandle);

    if(nodes > NODE_TABLE_NT_MAX_ENTRIES)
    {
        return (-1);
    }


    return ((int32_t)nodes);
}

int32_t getLreNodeTable(RED_NODE_TABLE *pNodeTable,
                      hsrPrpHandle *hsrPrphandle)
{

    uint32_t nodes;

    if(pNodeTable == NULL)
    {
        return (-1);
    }

    nodes = RedGetNodesCount(((ICSS_EMAC_Object *)hsrPrphandle->icssEmacHandle->object)->pruicssHandle);

    if(nodes > pNodeTable->max)
    {
        return (-1);
    }

    if(RedGetNodeTableEntry(pNodeTable,
                            hsrPrphandle) != 0)
    {
        return (-1);
    }

    pNodeTable->cnt = nodes;

    return ((int32_t)nodes);
}

RemNodeType_t getLreRemNodeType(uint8_t nodeEntryStatus)
{
    return RedGetRemNodeType(nodeEntryStatus);
}

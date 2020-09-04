/**
 * \file hsrPrp_red_hsr.c
 * \brief Contains HSR interface routines
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
 *
 * \par
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hsrPrp_red.h"
#include "hsrPrp_red_hsr.h"
#include "hsrPrp_handle.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** Macro to write the HSR Tag */
#define HSR_TAG_WRITE(tag,path,size,seq) \
    tag.ether_type    = OS_HostToNet16(ETHER_TYPE_HSR); \
    tag.path_and_size = OS_HostToNet16((path << 12) | size); \
    tag.seq_nr        = OS_HostToNet16(seq);

/** Macro to write the HSR Supervision Tag */
#define HSR_SUP_TAG_WRITE(tag,seq) \
    tag.ether_type   = OS_HostToNet16(ETHER_TYPE_HSR_SUP); \
    tag.path_and_ver = OS_HostToNet16((HSR_SUP_PATH << 12) | HSR_SUP_VER); \
    tag.seq_nr       = OS_HostToNet16(seq);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Moved to hsrPrpHandle to facilitate multi-instance capability */

/**< Default multicast MAC address. Only the last byte can be changed! */
const uint8_t RED_DEF_CONFIG_MULTICAST[ETHER_ADDR_LEN] = { 0x01, 0x15, 0x4E, 0x00, 0x01, 0x00 };

/**< DAN MAC address */
const uint8_t RED_DEF_CONFIG_DAN[ETHER_ADDR_LEN] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


void HsrFrameFill(hsrPrpHandle *hsrPrphandle, RED_FRAME *pRedFrame,
                  const uint8_t *pFrame, int32_t frameSize)
{
    if((pRedFrame != NULL ) && (pFrame != NULL))
    {
        uint16_t     offset = 0;
        HSR_TAG    hsrTag;
        uint16_t     lsduSize;

        /* Fill in the HSR tag */
        lsduSize = pRedFrame->bufferLen - ETHER_TYPE_OFFSET - pRedFrame->vlanTagSize -
                ETHER_TYPE_SIZE;
        HSR_TAG_WRITE(hsrTag, HSR_LAN_A_MAGIC, lsduSize, hsrPrphandle->redSeqNr);
        hsrPrphandle->redSeqNr++;

        /* Copy destination and source addresses */
        memcpy(pRedFrame->pDataBuffer, (uint8_t *)pFrame,
            ETHER_TYPE_OFFSET + pRedFrame->vlanTagSize);
        offset += 2 * ETHER_ADDR_LEN + pRedFrame->vlanTagSize;
        frameSize -= 2 * ETHER_ADDR_LEN + pRedFrame->vlanTagSize;

        /* Add HSR tag */
        memcpy(pRedFrame->pDataBuffer + offset, &hsrTag, HSR_TAG_SIZE);
        offset += HSR_TAG_SIZE;

        /* Copy the rest of the original frame */
        memcpy(pRedFrame->pDataBuffer + offset, (uint8_t *)pFrame + ETHER_TYPE_OFFSET +
            pRedFrame->vlanTagSize, frameSize);
        offset += frameSize;

        /* Add padding */
        if(pRedFrame->paddingSize)
        {
            memset(pRedFrame->pDataBuffer + offset, 0,  pRedFrame->paddingSize);
        }
    }
}

void HsrFrameUpdatePathId(RED_FRAME *pRedFrame, uint16_t pathId)
{
    uint16_t val;
    uint16_t offset;

    if(pRedFrame != NULL)
    {
        /* Check for VLAN tag and calculate path ID offset */
        val = OS_NetToHost16(*(uint16_t *)(pRedFrame->pDataBuffer + ETHER_TYPE_OFFSET));

        if(val == ETHER_TYPE_VLAN)
        {
            offset = ETHER_TYPE_OFFSET + ETHER_VLAN_PT_LEN + HSR_TAG_PATHID_OFFSET;
        }

        else
        {
            offset = ETHER_TYPE_OFFSET + HSR_TAG_PATHID_OFFSET;
        }

        /* Update path ID */
        val = OS_NetToHost16(*(uint16_t *)(pRedFrame->pDataBuffer + offset));
        val &= 0x0FFF;
        val |= (pathId << 12);
        *(uint16_t *)(pRedFrame->pDataBuffer + offset) = OS_HostToNet16(val);
    }
}

RED_FRAME *HsrSupFrameAllocate(hsrPrpHandle *hsrPrphandle)
{
    RED_FRAME *pRedFrame = NULL;
    HSR_SUP_FRAME *pSupFrame = NULL;

    /* Allocate the HSR frame structure */
    pRedFrame = (RED_FRAME *) malloc(sizeof(RED_FRAME));

    if(pRedFrame  == NULL)
    {
        RED_DEBUG_MSG("%s: pRedFrame == NULL\n", __FUNCTION__);
        return (NULL);
    }

    pRedFrame->bufferLen = sizeof(HSR_SUP_FRAME);
    pRedFrame->pDataBuffer = (uint8_t *)malloc(sizeof(HSR_SUP_FRAME));

    if(pRedFrame->pDataBuffer == NULL)
    {
        RED_DEBUG_MSG("%s: pSupFrame == NULL\n", __FUNCTION__);
        free(pRedFrame);
        return (NULL);
    }

    pSupFrame = (HSR_SUP_FRAME *)pRedFrame->pDataBuffer;

    /* Start as new */
    hsrPrphandle->supSeqNr = 0;

    /* Fill in the Multicast MAC address */
    memcpy((uint8_t *)pSupFrame->dst, (uint8_t *)RED_DEF_CONFIG_MULTICAST,
           ETHER_ADDR_LEN);

    /* Fill in the DANH MAC address */
    memcpy((uint8_t *)pSupFrame->src, (uint8_t *)RED_DEF_CONFIG_DAN, ETHER_ADDR_LEN);

    /* Fill in the HSR tag */
    HSR_TAG_WRITE(pSupFrame->hsr_tag, HSR_LAN_A_MAGIC, HSR_SUP_SIZE,
                  hsrPrphandle->redSeqNr);

    /* Fill in the Supervision tag */
    HSR_SUP_TAG_WRITE(pSupFrame->sup_tag, hsrPrphandle->supSeqNr);

    /* Fill in the TLV1 */
    pSupFrame->tlv1_tag.type = HSR_TLV1_TYPE;
    pSupFrame->tlv1_tag.mac_length = ETHER_ADDR_LEN;
    memcpy((uint8_t *)pSupFrame->tlv1_tag.dan_mac, (uint8_t *)RED_DEF_CONFIG_DAN,
           ETHER_ADDR_LEN);

    /* Fill in the TLV2, TODO: in case of a RedBox implementation TLV2 has to be updated */
    memset((uint8_t *)&pSupFrame->tlv2_tag, 0, sizeof(TLV2_TAG));

    /* Fill in the TLV0 */
    memset((uint8_t *)&pSupFrame->tlv0_tag, 0, sizeof(TLV0_TAG));

    /* Fill in the padding */
    memset((uint8_t *)&pSupFrame->padding, 0, HSR_SUP_PAD);

    return (pRedFrame);
}

void HsrSupFrameUpdateSrcAdd(RED_FRAME *pRedFrame, uint8_t *srcAdd)
{
    HSR_SUP_FRAME *pSupFrame = NULL;

    if(pRedFrame != NULL)
    {
        pSupFrame = (HSR_SUP_FRAME *)pRedFrame->pDataBuffer;

        memcpy((uint8_t *)pSupFrame->src, (uint8_t *)srcAdd, ETHER_ADDR_LEN);
        memcpy((uint8_t *)pSupFrame->tlv1_tag.dan_mac, (uint8_t *)srcAdd, ETHER_ADDR_LEN);
    }
}

void HsrSupFrameUpdatePathId(RED_FRAME *pRedFrame, uint16_t pathId)
{
    HSR_SUP_FRAME *pSupFrame = NULL;
    uint16_t         val;
    if(pRedFrame != NULL)
    {
        pSupFrame = (HSR_SUP_FRAME *)pRedFrame->pDataBuffer;

        val = OS_NetToHost16(pSupFrame->hsr_tag.path_and_size);
        val &= 0x00FF;
        val |= (pathId << 12);
        pSupFrame->hsr_tag.path_and_size = OS_HostToNet16(val);
    }
}

void HsrSupFrameUpdateSeqNr(hsrPrpHandle *hsrPrphandle, RED_FRAME *pRedFrame)
{
    HSR_SUP_FRAME *pSupFrame = NULL;
    if(pRedFrame != NULL)
    {
        pSupFrame = (HSR_SUP_FRAME *)pRedFrame->pDataBuffer;

        pSupFrame->hsr_tag.seq_nr = OS_HostToNet16(hsrPrphandle->redSeqNr);
        pSupFrame->sup_tag.seq_nr = OS_HostToNet16(hsrPrphandle->supSeqNr);
    }
}

void HsrSupFrameIncrementSeqNr(hsrPrpHandle *hsrPrphandle)
{
    hsrPrphandle->redSeqNr++;
    hsrPrphandle->supSeqNr++;
}

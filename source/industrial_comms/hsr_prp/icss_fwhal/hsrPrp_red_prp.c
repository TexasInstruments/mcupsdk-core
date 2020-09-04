/**
 * \file hsrPrp_red_prp.c
 * \brief Contains the PRP interface routines
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hsrPrp_red.h"
#include "hsrPrp_red_prp.h"
#include "hsrPrp_handle.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** Macro to write the PRP Tag */
#define PRP_RCT_WRITE(tag,path,size,seq) \
    tag.seq_nr       = OS_HostToNet16(seq); \
    tag.lan_and_size = OS_HostToNet16((path << 12) | size); \
    tag.prp_suffix   = OS_HostToNet16(PRP_TRAILER_MAGIC);

/** Macro to write the PRP Supervision Tag */
#define PRP_SUP_TAG_WRITE(tag,seq) \
    tag.ether_type   = OS_HostToNet16(ETHER_TYPE_PRP_SUP); \
    tag.path_and_ver = OS_HostToNet16((PRP_SUP_PATH << 12) | PRP_SUP_VER); \
    tag.seq_nr       = OS_HostToNet16(seq);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Moved to hsrHandle to facilitate multi-instance capability */

/**< Default multicast MAC address. Only the last byte can be changed! */
const uint8_t RED_DEF_CONFIG_MULTICAST[ETHER_ADDR_LEN] = { 0x01, 0x15, 0x4E, 0x00, 0x01, 0x00 };

/**< DAN MAC address */
const uint8_t RED_DEF_CONFIG_DAN[ETHER_ADDR_LEN] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void PrpFrameFill(hsrPrpHandle *hsrPrphandle, RED_FRAME *pRedFrame,
                  const uint8_t *pFrame, int32_t frameSize)
{
    if((pRedFrame != NULL ) && (pFrame != NULL))
    {
        uint16_t offset;
        PRP_RCT prpRct;
        uint16_t lsduSize;
        uint16_t CheckRct88fb;

        /* Prepare lsduSize for PRP RCT */
        lsduSize = pRedFrame->bufferLen - ETHER_TYPE_OFFSET - pRedFrame->vlanTagSize -
                ETHER_TYPE_SIZE;

        /* Copy the original frame */
        memcpy(pRedFrame->pDataBuffer, (uint8_t *)pFrame, frameSize);
        offset = frameSize;

        /* Add padding */
        if(pRedFrame->paddingSize)
        {
            memset(pRedFrame->pDataBuffer + offset, 0, pRedFrame->paddingSize);
        }

        offset += pRedFrame->paddingSize;

        /* Check for presence of RCT i.e. last 2 bytes of pFrame as 0xFB88.
        * If RCT already present subtract PRP_RCT_SIZE (6).
        * 0xFB88 instead of 0x88FB due to little-big endian compatability. */
        memcpy(&CheckRct88fb, (uint8_t *)(pFrame + frameSize - 2), 2);

        if(CheckRct88fb == PRP_TRAILER_MAGIC_BIG_ENDIAN)
        {
            lsduSize -= PRP_RCT_SIZE;
            offset -= PRP_RCT_SIZE;
            pRedFrame->bufferLen -= PRP_RCT_SIZE;
        }

        /* Fill in the PRP RCT */
        PRP_RCT_WRITE(prpRct, PRP_LAN_A_MAGIC, lsduSize, hsrPrphandle->redSeqNr);
        hsrPrphandle->redSeqNr++;

        /* Add PRP RCT */
        memcpy(pRedFrame->pDataBuffer + offset, &prpRct, PRP_RCT_SIZE);
    }
}

void PrpFrameUpdateLanId(RED_FRAME *pRedFrame, uint16_t lanId)
{
    uint16_t val;
    uint16_t offset;

    if(pRedFrame != NULL)
    {
        /* Calculate LAN ID offset */
        offset = pRedFrame->bufferLen - sizeof(PRP_RCT) + PRP_RCT_LANID_LSDU_OFFSET;

        /* Update LAN ID */
        val = OS_NetToHost16(*(uint16_t *)(pRedFrame->pDataBuffer + offset));
        val &= 0x0FFF;
        val |= (lanId << 12);
        *(uint16_t *)(pRedFrame->pDataBuffer + offset) = OS_HostToNet16(val);
    }
}

RED_FRAME *PrpSupFrameAllocate(hsrPrpHandle *hsrPrphandle)
{
    RED_FRAME *pRedFrame = NULL;
    PRP_SUP_FRAME *pSupFrame = NULL;

    /* Allocate the HSR frame structure */
    pRedFrame = (RED_FRAME *) malloc(sizeof(RED_FRAME));

    if(!pRedFrame)
    {
        RED_DEBUG_MSG("%s: pRedFrame == NULL\n", __FUNCTION__);
        return (NULL);
    }

    pRedFrame->bufferLen = sizeof(PRP_SUP_FRAME);
    pRedFrame->pDataBuffer = (uint8_t *) malloc(sizeof(PRP_SUP_FRAME));

    if(!pRedFrame->pDataBuffer)
    {
        RED_DEBUG_MSG("%s: pSupFrame == NULL\n", __FUNCTION__);
        free(pRedFrame);
        return (NULL);
    }

    pSupFrame = (PRP_SUP_FRAME *)pRedFrame->pDataBuffer;

    /* Start as new */
    hsrPrphandle->supSeqNr = 0;

    /* Fill in the Multicast MAC address */
    memcpy((uint8_t *)pSupFrame->dst, (uint8_t *)RED_DEF_CONFIG_MULTICAST,
           ETHER_ADDR_LEN);

    /* Fill in the DANH MAC address */
    memcpy((uint8_t *)pSupFrame->src, (uint8_t *)RED_DEF_CONFIG_DAN, ETHER_ADDR_LEN);

    /* Fill in the Supervision tag */
    PRP_SUP_TAG_WRITE(pSupFrame->sup_tag, hsrPrphandle->supSeqNr);

    /* Fill in the TLV1 */
    pSupFrame->tlv1_tag.type =
        PRP_TLV1_TYPE_DUP_DISCARD; /* Duplicate Accept can be set over SNMP */
    pSupFrame->tlv1_tag.mac_length = ETHER_ADDR_LEN;
    memcpy((uint8_t *)pSupFrame->tlv1_tag.dan_mac, (uint8_t *)RED_DEF_CONFIG_DAN,
           ETHER_ADDR_LEN);

    /* Fill in the TLV2, TODO: in case of a RedBox implementation TLV2 has to be updated */
    memset((uint8_t *)&pSupFrame->tlv2_tag, 0, sizeof(TLV2_TAG));

    /* Fill in the TLV0 */
    memset((uint8_t *)&pSupFrame->tlv0_tag, 0, sizeof(TLV0_TAG));

    /* Fill in the padding */
    memset((uint8_t *)&pSupFrame->padding, 0, PRP_SUP_PAD);

    /* Fill in the PRP RCT */
    PRP_RCT_WRITE(pSupFrame->prp_rct, PRP_LAN_A_MAGIC, PRP_SUP_SIZE,
                  hsrPrphandle->redSeqNr);

    return (pRedFrame);
}

void PrpSupFrameUpdateSrcAdd(RED_FRAME *pRedFrame, uint8_t *srcAdd)
{
    PRP_SUP_FRAME *pSupFrame = NULL;

    if(pRedFrame != NULL)
    {
        pSupFrame = (PRP_SUP_FRAME *)pRedFrame->pDataBuffer;

        memcpy((uint8_t *)pSupFrame->src, (uint8_t *)srcAdd, ETHER_ADDR_LEN);
        memcpy((uint8_t *)pSupFrame->tlv1_tag.dan_mac, (uint8_t *)srcAdd, ETHER_ADDR_LEN);
    }
}

void PrpSupFrameUpdateLanId(RED_FRAME *pRedFrame, uint16_t lanId)
{
    PRP_SUP_FRAME *pSupFrame = NULL;
    uint16_t         val;

    if(pRedFrame != NULL)
    {
        pSupFrame = (PRP_SUP_FRAME *)pRedFrame->pDataBuffer;

        val = OS_NetToHost16(pSupFrame->prp_rct.lan_and_size);
        val &= 0x00FF;
        val |= (lanId << 12);
        pSupFrame->prp_rct.lan_and_size = OS_HostToNet16(val);
    }
}

void PrpSupFrameUpdateSeqNr(hsrPrpHandle *hsrPrphandle, RED_FRAME *pRedFrame)
{
    PRP_SUP_FRAME *pSupFrame = NULL;

    if(pRedFrame != NULL)
    {
        pSupFrame = (PRP_SUP_FRAME *)pRedFrame->pDataBuffer;

        pSupFrame->sup_tag.seq_nr = OS_HostToNet16(hsrPrphandle->supSeqNr);
        pSupFrame->prp_rct.seq_nr = OS_HostToNet16(hsrPrphandle->redSeqNr);
    }
}

void PrpSupFrameUpdateTlv(RED_FRAME *pRedFrame, uint8_t type)
{
    PRP_SUP_FRAME *pSupFrame = NULL;

    if(pRedFrame != NULL)
    {
        pSupFrame = (PRP_SUP_FRAME *)pRedFrame->pDataBuffer;

        pSupFrame->tlv1_tag.type = type;
    }
}

void PrpSupFrameIncrementSeqNr(hsrPrpHandle *hsrPrphandle)
{
    hsrPrphandle->redSeqNr++;
    hsrPrphandle->supSeqNr++;
}

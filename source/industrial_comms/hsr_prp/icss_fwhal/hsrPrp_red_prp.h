/**
 * \file hsrPrp_red_prp.h
 * \brief Include file for hsrPrp_red_prp.c
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

#ifndef RED_PRP_H_
#define RED_PRP_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "hsrPrp_red_common.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ETHER_TYPE_PRP_SUP    ( ETHER_TYPE_RED_SUP )

#define PRP_RCT_SIZE                           ( 6 )  /**< PRP RCT size */
#define PRP_TRAILER_MAGIC                 ( 0x88FB )  /**< PRP-1 RCT Suffix */
#define PRP_TRAILER_MAGIC_BIG_ENDIAN      ( 0xFB88 )  /**< PRP-1 RCT Suffix in Big Endian format*/
#define PRP_LAN_A_MAGIC                      ( 0xA )  /**< PRP-1 LAN A marker */
#define PRP_LAN_B_MAGIC                      ( 0xB )  /**< PRP-1 LAN B marker */
#define PRP_RCT_SEQNO_OFFSET                   ( 0 )  /**< Seq No : 2 bytes  */
#define PRP_RCT_LANID_LSDU_OFFSET              ( 2 )  /**< LanID : 4 bits followed by LSDU size : 12 bits */
#define PRP_RCT_ETHTYPE_OFFSET                 ( 4 )  /**< set to 0x88FB for PRP*/

#define PRP_TLV1_TYPE_DUP_DISCARD             ( 20 )  /**< PRP node in Duplicate Discard mode */
#define PRP_TLV1_TYPE_DUP_ACCEPT              ( 21 )  /**< PRP node in Duplicate Accept mode */

#define PRP_SUP_SIZE                ( RED_SUP_SIZE )
#define PRP_SUP_PATH                ( RED_SUP_PATH )
#define PRP_SUP_VER                  ( RED_SUP_VER )
#define PRP_SUP_PAD                  ( RED_SUP_PAD )

/*
PRP : Bit 0 in TX buffer descriptor is used for PRP vs EMAC mode
        0 : PRP mode  => Mask : 0x0
        1 : EMAC mode => Mask : 0x1
*/
#define PRP_MODE_MASK                        ( 0x0 )
#define EMAC_MODE_MASK                       ( 0x1 )
#define PRP_VS_EMAC_MODE_MASK      ( PRP_MODE_MASK )

/** PRP Redundancy Control Trailer */
typedef struct _PRP_RCT
{

    uint16_t seq_nr;                    /**< SendSeq of the sender */
    uint16_t lan_and_size;              /**< A 4-bit LAN ID and a 12-bit LSDU size */
    uint16_t prp_suffix;                /**< PRP suffix, set to 0x88FB */

} PRP_RCT;

/** PRP Supervision frame with no VLAN tag */
typedef struct _PRP_SUP_FRAME
{

    uint8_t    dst[ETHER_ADDR_LEN];     /**< HSR Destination Address = multicast (01-15-4E-00-01-XX) */
    uint8_t    src[ETHER_ADDR_LEN];     /**< Source Address (MAC address of the node) */
    SUP_TAG  sup_tag;                 /**< Supervision frame tag */
    TLV1_TAG tlv1_tag;                /**< TLV1 */
    TLV2_TAG tlv2_tag;                /**< TLV2 */
    TLV0_TAG tlv0_tag;                /**< TLV0 */
    uint8_t    padding[PRP_SUP_PAD];    /**< Padding to 70 octets */
    PRP_RCT  prp_rct;                 /**< PRP RCT */

} PRP_SUP_FRAME;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/** @addtogroup PRP_API
 @{ */
/**
 *  \internal
 *  \brief Fills RED_FRAME buffer with a tagged frame
 *
 *  \param pRedFrame pointer to a RED_FRAME
 *  \param pFrame pointer to not redundant frame
 *  \param frameSize size of not redundant frame
 *
 *
 */
void       PrpFrameFill(hsrPrpHandle *hsrPrphandle, RED_FRAME *pRedFrame,
                        const uint8_t *pFrame,
                        int32_t frameSize);
/**
 *  \internal
 *  \brief Updates LAN ID of a RED_FRAME
 *
 *  \param pRedFrame pointer to a RED_FRAME
 *  \param lanId LAN ID value
 *
 *
 */
void       PrpFrameUpdateLanId(RED_FRAME *pPrpFrame, uint16_t pathId);
/**
 *  \brief Allocates a RED_FRAME that holds PRP Supervision frame
 *
 *  \param  hsrPrphandle
 *
 *  \return Pointer to a RED_FRAME structure
 *
 */
RED_FRAME *PrpSupFrameAllocate(hsrPrpHandle *hsrPrphandle);
/**
 *  \brief Updates Source Address of a PRP Supervision frame
 *
 *  \param pRedFrame pointer to a RED_FRAME
 *  \param srcAdd pointer to a new source address value
 *
 *
 */
void       PrpSupFrameUpdateSrcAdd(RED_FRAME *pRedFrame, uint8_t *srcAdd);
/**
 *  \brief Updates LAN ID of a PRP Supervision frame
 *
 *  \param pRedFrame pointer to a RED_FRAME
 *  \param lanId LAN ID value
 *
 *
 */
void       PrpSupFrameUpdateLanId(RED_FRAME *pRedFrame, uint16_t lanId);
/**
 *  \internal
 *  \brief Updates Sequence Number of a PRP Supervision frame
 *
 *  \param pRedFrame pointer to a RED_FRAME
 *
 *
 */
void       PrpSupFrameUpdateSeqNr(hsrPrpHandle *hsrPrphandle,
                                  RED_FRAME *pRedFrame);
/**
 *  \internal
 *  \brief Updates TLV1 of a PRP Supervision frame
 *
 *  \param pRedFrame pointer to a RED_FRAME
 *  \param type TLV1.type set value
 *
 *
 */
void       PrpSupFrameUpdateTlv(RED_FRAME *pRedFrame, uint8_t type);
/**
 *  \internal
 *  \brief Increments Sequence Number counters
 *
 *
 */
void       PrpSupFrameIncrementSeqNr(hsrPrpHandle *hsrPrphandle);

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* RED_PRP_H_ */

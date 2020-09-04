/**
 * \file hsrPrp_red_hsr.h
 * \brief Include file for hsrPrp_red_hsr.c
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

#ifndef RED_HSR_H_
#define RED_HSR_H_

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
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ETHER_TYPE_HSR                    ( 0x892F )  /**< HSR ether type */
#define ETHER_TYPE_HSR_SUP    ( ETHER_TYPE_RED_SUP )

#define HSR_TAG_SIZE                           ( 6 )  /**< HSR tag size */
#define HSR_TAG_PATHID_OFFSET                  ( 2 )  /**< Offset from the beginning of HSR tag to the path ID */
#define HSR_LAN_A_MAGIC                     ( 0x00 )  /**< HSR LAN A marker */
#define HSR_LAN_B_MAGIC                     ( 0x01 )  /**< HSR LAN B marker */

#define HSR_TLV1_TYPE                         ( 23 )  /**< HSR node in normal operation */

#define HSR_SUP_SIZE                ( RED_SUP_SIZE )
#define HSR_SUP_PATH                ( RED_SUP_PATH )
#define HSR_SUP_VER                  ( RED_SUP_VER )
#define HSR_SUP_PAD                  ( RED_SUP_PAD )

/** HSR tag */
typedef struct _HSR_TAG
{

    uint16_t ether_type;                /**< Ethertype of the HSR protocol, shall be 0x892F */
    uint16_t path_and_size;             /**< Size of the LSDU in octets */
    uint16_t seq_nr;                    /**< SendSeq of the sender */

} HSR_TAG;

/** HSR Supervision frame with no VLAN tag */
typedef struct _HSR_SUP_FRAME
{

    uint8_t    dst[ETHER_ADDR_LEN];     /**< HSR Destination Address = multicast (01-15-4E-00-01-XX) */
    uint8_t    src[ETHER_ADDR_LEN];     /**< Source Address (MAC address of the node) */
    HSR_TAG  hsr_tag;                 /**< HSR tag */
    SUP_TAG  sup_tag;                 /**< Supervision frame tag */
    TLV1_TAG tlv1_tag;                /**< TLV1 */
    TLV2_TAG tlv2_tag;                /**< TLV2 */
    TLV0_TAG tlv0_tag;                /**< TLV0 */
    uint8_t    padding[HSR_SUP_PAD];    /**< Padding to 70 octets */

} HSR_SUP_FRAME;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/** @addtogroup HSR_API
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
void       HsrFrameFill(hsrPrpHandle *, RED_FRAME *pRedFrame,
                        const uint8_t *pFrame,
                        int32_t frameSize);
/**
 *  \brief Updates PATH ID of a RED_FRAME
 *
 *  \param pRedFrame pointer to a RED_FRAME
 *  \param pathId PATH ID value
 *
 *
 */
void       HsrFrameUpdatePathId(RED_FRAME *pRedFrame, uint16_t pathId);
/**
 *  \brief Allocates a RED_FRAME that holds HSR Supervision frame
 *
 *  \return Pointer to a RED_FRAME structure
 *
 */
RED_FRAME *HsrSupFrameAllocate(hsrPrpHandle *);
/**
 *  \brief Updates Source Address of a HSR Supervision frame
 *
 *  \param pRedFrame pointer to a RED_FRAME
 *  \param srcAdd pointer to a new source address value
 *
 *
 */
void       HsrSupFrameUpdateSrcAdd(RED_FRAME *pRedFrame, uint8_t *srcAdd);
/**
 *  \internal
 *  \brief Updates PATH ID of a HSR Supervision frame
 *
 *  \param pRedFrame pointer to a RED_FRAME
 *  \param pathId PATH ID value
 *
 *
 */
void       HsrSupFrameUpdatePathId(RED_FRAME *pRedFrame, uint16_t pathId);
/**
 *  \internal
 *  \brief Updates Sequence Number of a HSR Supervision frame
 *
 *  \param pRedFrame pointer to a RED_FRAME
 *
 *
 */
void       HsrSupFrameUpdateSeqNr(hsrPrpHandle *, RED_FRAME *pRedFrame);
/**
 *  \internal
 *  \brief  Increments Sequence Number counters
 *
 *
 */
void       HsrSupFrameIncrementSeqNr(hsrPrpHandle *);

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* RED_HSR_H_ */

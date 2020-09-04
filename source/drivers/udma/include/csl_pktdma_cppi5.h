/*
 *  Copyright (C) 2019-2020 Texas Instruments Incorporated.
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
 */
/**
 *  \file  csl_pktdma_cppi5.h
 *
 *  \brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for CPPI5.
 */
/**
 *  \ingroup DRV_UDMA_MODULE
 *
 *  @{
 */

#ifndef CSL_PKTDMA_CPPI5_H_
#define CSL_PKTDMA_CPPI5_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <drivers/hw_include/cslr.h>

/** ===========================================================================
 *
 * @defgroup CSL_PKTDMA_CPPI5_API CPPI5 API
 * @ingroup DRV_UDMA_MODULE
 * ============================================================================
 */
/**
@defgroup CSL_PKTDMA_CPPI5_DATASTRUCT  CPPI5 Data Structures
@ingroup CSL_PKTDMA_CPPI5_API
*/
/**
@defgroup CSL_PKTDMA_CPPI5_FUNCTION  CPPI5 Functions
@ingroup CSL_PKTDMA_CPPI5_API
*/
/**
@defgroup CSL_PKTDMA_CPPI5_ENUM CPPI5 Enumerated Data Types
@ingroup CSL_PKTDMA_CPPI5_API
*/

/**
 *  \addtogroup CSL_PKTDMA_CPPI5_DATASTRUCT
 *  @{
 */

/** \brief Extended Packet Info (EPI) block
 *
 *  This optional structure is used for time-stamp and software information
 *  storage.
 *
 */
typedef struct
{
  uint32_t      tsInfo;         /**< Extended packet info block word 0 */
  uint32_t      swInfo0;        /**< Extended packet info block word 1 */
  uint32_t      swInfo1;        /**< Extended packet info block word 2 */
  uint32_t      swInfo2;        /**< Extended packet info block word 3 */
} CSL_PktdmaCppi5Epi;

/** \brief Host-mode packet and buffer descriptor
 *
 *  This structure is used for host-mode packet descriptors and buffer
 *  descriptors.
 *
 */
typedef struct
{
  uint32_t      descInfo;       /**< word 0: Packet information word 0 (not used in Buffer descriptors) */
  uint32_t      pktInfo1;       /**< word 1: Packet information word 1 (not used in Buffer descriptors) */
  uint32_t      pktInfo2;       /**< word 2: Packet information word 2 | Buffer reclamation information */
  uint32_t      srcDstTag;      /**< word 3: Packet information word 3 (not used in Buffer descriptors) */
  uint64_t      nextDescPtr;    /**< words 4/5: Linking word */
  uint64_t      bufPtr;         /**< words 6/7: Buffer 0 information words 0 and 1 (PD words 6 and 7) */
  uint32_t      bufInfo1;       /**< word 8: Buffer 0 information word 2 (PD word 8) */
  uint32_t      orgBufLen;      /**< word 9: Original buffer 0 information word 0 (PD word 9) */
  uint64_t      orgBufPtr;      /**< words 10/11: Original buffer 0 information words 1 and 2 (PD words 10 and 11) */
                                /* Extended Packet Info Data (optional, 4 words) */
                                /* Protocol Specific Data (optional, 0-128 bytes in multiples of 4), and/or */
                                /* Other Software Data (0-N bytes, optional) */
} CSL_PktdmaCppi5HMPD;

/* @} */

/*-----------------------------------------------------------------------------
// Packet and Buffer Descriptor field manipulation macros
//---------------------------------------------------------------------------*/
#define CSL_PKTDMA_CPPI5_PD_DESCINFO_DTYPE_SHIFT             ((uint32_t) 30U)
#define CSL_PKTDMA_CPPI5_PD_DESCINFO_DTYPE_MASK              (((uint32_t) 0x3U) << CSL_PKTDMA_CPPI5_PD_DESCINFO_DTYPE_SHIFT)
#define   CSL_PKTDMA_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST            ((uint32_t) 1U)
#define CSL_PKTDMA_CPPI5_PD_DESCINFO_EINFO_SHIFT             ((uint32_t) 29U)
#define CSL_PKTDMA_CPPI5_PD_DESCINFO_EINFO_MASK              (((uint32_t) 0x1U) << CSL_PKTDMA_CPPI5_PD_DESCINFO_EINFO_SHIFT)
#define   CSL_PKTDMA_CPPI5_PD_DESCINFO_EINFO_VAL_NOT_PRESENT     ((uint32_t) 0)
#define   CSL_PKTDMA_CPPI5_PD_DESCINFO_EINFO_VAL_IS_PRESENT      ((uint32_t) 1U)
#define CSL_PKTDMA_CPPI5_PD_DESCINFO_PSWCNT_SHIFT            ((uint32_t) 22U)
#define CSL_PKTDMA_CPPI5_PD_DESCINFO_PSWCNT_MASK             (((uint32_t) 0x3FU) << CSL_PKTDMA_CPPI5_PD_DESCINFO_PSWCNT_SHIFT)
#define CSL_PKTDMA_CPPI5_PD_DESCINFO_PKTLEN_SHIFT            ((uint32_t) 0)
#define CSL_PKTDMA_CPPI5_PD_DESCINFO_PKTLEN_MASK             (((uint32_t) 0x3FFFFFU) << CSL_PKTDMA_CPPI5_PD_DESCINFO_PKTLEN_SHIFT)

#define CSL_PKTDMA_CPPI5_PD_PKTINFO1_PKTERROR_SHIFT          ((uint32_t) 28U)
#define CSL_PKTDMA_CPPI5_PD_PKTINFO1_PKTERROR_MASK           (((uint32_t) 0xFU) << CSL_PKTDMA_CPPI5_PD_PKTINFO1_PKTERROR_SHIFT)
#define CSL_PKTDMA_CPPI5_PD_PKTINFO1_PSFLGS_SHIFT            ((uint32_t) 24U)
#define CSL_PKTDMA_CPPI5_PD_PKTINFO1_PSFLGS_MASK             (((uint32_t) 0xFU) << CSL_PKTDMA_CPPI5_PD_PKTINFO1_PSFLGS_SHIFT)
#define CSL_PKTDMA_CPPI5_PD_PKTINFO1_FLOWID_SHIFT            ((uint32_t) 0)
#define CSL_PKTDMA_CPPI5_PD_PKTINFO1_FLOWID_MASK             (((uint32_t) 0x3FFFU) << CSL_PKTDMA_CPPI5_PD_PKTINFO1_FLOWID_SHIFT)

#define CSL_PKTDMA_CPPI5_PD_PKTINFO2_PKTTYPE_SHIFT           ((uint32_t) 27U)
#define CSL_PKTDMA_CPPI5_PD_PKTINFO2_PKTTYPE_MASK            (((uint32_t) 0x1FU) << CSL_PKTDMA_CPPI5_PD_PKTINFO2_PKTTYPE_SHIFT)

#define CSL_PKTDMA_CPPI5_PD_SRCDSTTAG_SRCTAG_SHIFT           ((uint32_t) 16U)
#define CSL_PKTDMA_CPPI5_PD_SRCDSTTAG_SRCTAG_MASK            (((uint32_t) 0xFFFFU) << CSL_PKTDMA_CPPI5_PD_SRCDSTTAG_SRCTAG_SHIFT)
#define CSL_PKTDMA_CPPI5_PD_SRCDSTTAG_DSTTAG_SHIFT           ((uint32_t) 0)
#define CSL_PKTDMA_CPPI5_PD_SRCDSTTAG_DSTTAG_MASK            (((uint32_t) 0xFFFFU) << CSL_PKTDMA_CPPI5_PD_SRCDSTTAG_DSTTAG_SHIFT)

#define CSL_PKTDMA_CPPI5_PD_NEXTDESCPTR_ASPACE_SHIFT         ((uint64_t) 48UL)
#define CSL_PKTDMA_CPPI5_PD_NEXTDESCPTR_ASPACE_MASK          (((uint64_t) 0xFU) << CSL_PKTDMA_CPPI5_PD_NEXTDESCPTR_ASPACE_SHIFT)
#define CSL_PKTDMA_CPPI5_PD_NEXTDESCPTR_ADDR_SHIFT           ((uint64_t) 0U)
#define CSL_PKTDMA_CPPI5_PD_NEXTDESCPTR_ADDR_MASK            (((uint64_t) 0x0000FFFFFFFFFFFFUL) << CSL_PKTDMA_CPPI5_PD_NEXTDESCPTR_ADDR_SHIFT)

#define CSL_PKTDMA_CPPI5_PD_BUFPTR_ASPACE_SHIFT              ((uint64_t) 48UL)
#define CSL_PKTDMA_CPPI5_PD_BUFPTR_ASPACE_MASK               (((uint64_t) 0xFU) << CSL_PKTDMA_CPPI5_PD_BUFPTR_ASPACE_SHIFT)
#define CSL_PKTDMA_CPPI5_PD_BUFPTR_ADDR_SHIFT                ((uint64_t) 0U)
#define CSL_PKTDMA_CPPI5_PD_BUFPTR_ADDR_MASK                 (((uint64_t) 0x0000FFFFFFFFFFFFUL) << CSL_PKTDMA_CPPI5_PD_BUFPTR_ADDR_SHIFT)

#define CSL_PKTDMA_CPPI5_PD_BUFINFO1_LEN_SHIFT               ((uint32_t) 0)
#define CSL_PKTDMA_CPPI5_PD_BUFINFO1_LEN_MASK                (((uint32_t) 0x3FFFFFU) << CSL_PKTDMA_CPPI5_PD_BUFINFO1_LEN_SHIFT)

#define CSL_PKTDMA_CPPI5_PD_ORGBUFLEN_LEN_SHIFT              (0)
#define CSL_PKTDMA_CPPI5_PD_ORGBUFLEN_LEN_MASK               (((uint32_t) 0x3FFFFFU) << CSL_UDMAP_CPPI5_PD_ORGBUFLEN_LEN_SHIFT)

#define CSL_PKTDMA_CPPI5_PD_ORGBUFPTR_ASPACE_SHIFT           ((uint64_t) 48UL)
#define CSL_PKTDMA_CPPI5_PD_ORGBUFPTR_ASPACE_MASK            (((uint64_t) 0xFU) << CSL_UDMAP_CPPI5_PD_ORGBUFPTR_ASPACE_SHIFT)
#define CSL_PKTDMA_CPPI5_PD_ORGBUFPTR_ADDR_SHIFT             ((uint64_t) 0U)
#define CSL_PKTDMA_CPPI5_PD_ORGBUFPTR_ADDR_MASK              (((uint64_t) 0x0000FFFFFFFFFFFFUL) << CSL_UDMAP_CPPI5_PD_ORGBUFPTR_ADDR_SHIFT)

/**
 *  \addtogroup CSL_PKTDMA_CPPI5_FUNCTION
 *  @{
 */
/* Map old CSL_pktdmaMakeAselAddr function to new CSL_pktdmaSetAselInAddr equivalent */
#define CSL_pktdmaMakeAselAddr  CSL_pktdmaSetAselInAddr

static inline uint32_t  CSL_pktdmaCppi5GetDescType( const void *pDesc );
static inline void      CSL_pktdmaCppi5SetDescType( void *pDesc, uint32_t descType );
static inline uint32_t  CSL_pktdmaCppi5GetPktLen( const void *pDesc );
static inline void      CSL_pktdmaCppi5SetPktLen( void *pDesc, uint32_t descType, uint32_t pktLen );
static inline void      CSL_pktdmaCppi5HostSetPktLen( void *pDesc, uint32_t pktLen );
static inline uint64_t  CSL_pktdmaCppi5GetBufferAddr( const CSL_PktdmaCppi5HMPD *pDesc );
static inline uint32_t  CSL_pktdmaCppi5GetBufferLen( const CSL_PktdmaCppi5HMPD *pDesc );
static inline void      CSL_pktdmaCppi5SetBufferAddr( CSL_PktdmaCppi5HMPD *pDesc, uint64_t physBufferAddr );
static inline void      CSL_pktdmaCppi5SetOrgBufferAddr( CSL_PktdmaCppi5HMPD *pDesc, uint64_t physBufferAddr );
static inline void      CSL_pktdmaCppi5SetBufferLen( CSL_PktdmaCppi5HMPD *pDesc, uint32_t bufferLenBytes );
static inline void      CSL_pktdmaCppi5SetOrgBufferLen( CSL_PktdmaCppi5HMPD *pDesc, uint32_t bufferLenBytes );
static inline void      CSL_pktdmaCppi5LinkDesc( CSL_PktdmaCppi5HMPD *pDesc, uint64_t physBufferDescAddr );
static inline bool      CSL_pktdmaCppi5IsEpiDataPresent( const void *pDesc );
static inline void      CSL_pktdmaCppi5SetEpiDataPresent( void *pDesc, bool bEpiDataPresent );
static inline uint32_t *CSL_pktdmaCppi5GetEpiDataPtr( const void *pDesc );
static inline int32_t   CSL_pktdmaCppi5RdEpiData( const void *pDesc, uint32_t *pTsInfo, uint32_t *pSwInfo0, uint32_t *pSwInfo1, uint32_t *pSwInfo2 );
static inline void      CSL_pktdmaCppi5WrEpiData( const void *pDesc, uint32_t tsInfo, uint32_t swInfo0, uint32_t swInfo1, uint32_t swInfo2 );
static inline uint32_t  CSL_pktdmaCppi5GetPsDataLoc( const void *pDesc );
static inline void      CSL_pktdmaCppi5SetPsDataLoc( void *pDesc, uint32_t psLoc );
static inline uint32_t  CSL_pktdmaCppi5GetPsDataLen( const void *pDesc );
static inline void      CSL_pktdmaCppi5SetPsDataLen( void *pDesc, uint32_t psDataLen );
static inline uint64_t  CSL_pktdmaCppi5GetPsDataAddr( const void *pDesc, bool bInSopBuf, bool bEpiPresent );
static inline uint8_t   *CSL_pktdmaCppi5GetPsDataPtr( const void *pDesc );
static inline uint32_t  CSL_pktdmaCppi5GetSrcTag( const void *pDesc );
static inline uint32_t  CSL_pktdmaCppi5GetDstTag( const void *pDesc );
static inline void      CSL_pktdmaCppi5SetSrcTag( void *pDesc, uint32_t srcTag );
static inline void      CSL_pktdmaCppi5SetDstTag( void *pDesc, uint32_t dstTag );
static inline uint32_t  CSL_pktdmaCppi5GetErrorFlags( const void *pDesc );
static inline uint32_t  CSL_pktdmaCppi5GetPsFlags( const void *pDesc );
static inline void      CSL_pktdmaCppi5SetPsFlags( void *pDesc, uint32_t psFlags );
static inline uint32_t  CSL_pktdmaCppi5GetPktType( const void *pDesc );
static inline void      CSL_pktdmaCppi5SetPktType( void *pDesc, uint32_t pktType );
static inline void      CSL_pktdmaCppi5GetIds( const void *pDesc, uint32_t *pPktId, uint32_t *pFlowId );
static inline void      CSL_pktdmaCppi5SetIds( void *pDesc, uint32_t descType, uint32_t pktId, uint32_t flowId );
static inline void      CSL_pktdmaCppi5GetReturnPolicy( const void *pDesc, uint32_t *pRetPolicy, uint32_t *pEarlyReturn, uint32_t *pRetPushPolicy, uint32_t *pRetQnum );
static inline void      CSL_pktdmaCppi5SetReturnPolicy( void *pDesc, uint32_t descType, uint32_t retPolicy, uint32_t earlyReturn, uint32_t retPushPolicy, uint32_t retQnum );
static inline uint64_t  CSL_pktdmaClrAselInAddr( uint64_t addr );
static inline uint64_t  CSL_pktdmaSetAselInAddr( uint64_t addr, uint32_t asel );

/**
 *  \brief Get the descriptor type
 *
 *  This function returns the type of descriptor. Valid descriptor types are:
 *    1 = Host-mode (HOST) packet descriptor
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return Descriptor type
 */
static inline uint32_t CSL_pktdmaCppi5GetDescType( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_PktdmaCppi5HMPD *)pDesc)->descInfo, PKTDMA_CPPI5_PD_DESCINFO_DTYPE );
}

/**
 *  \brief Set the descriptor type
 *
 *  This function sets the type of descriptor. Valid descriptor types are:
 *    1 = Host-mode (HOST) packet descriptor
 *
 *  Note that no error checking is performed on the passed descriptor
 *  type argument.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param descType [IN]    Descriptor type
 *
 *  \return None
 */
static inline void CSL_pktdmaCppi5SetDescType( void *pDesc, uint32_t descType )
{
    CSL_FINS( ((CSL_PktdmaCppi5HMPD *)pDesc)->descInfo, PKTDMA_CPPI5_PD_DESCINFO_DTYPE, descType );
}

/**
 *  \brief Get the packet length
 *
 *  This function returns the packet length. The value returned depends on
 *  the type of descriptor as follows:
 *
 *  Host mode packet descriptors:
 *      The length of the packet in bytes is returned
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return Packet length
 */
static inline uint32_t CSL_pktdmaCppi5GetPktLen( const void *pDesc )
{
    uint32_t pktLen;

    pktLen = CSL_FEXT( ((const CSL_PktdmaCppi5HMPD *)pDesc)->descInfo, PKTDMA_CPPI5_PD_DESCINFO_PKTLEN );
    return pktLen;
}

/**
 *  \brief Set the packet length
 *
 *  This function sets the packet length. The meaning of the pktLen argument
 *  depends on the type of descriptor as follows:
 *
 *  Host mode packet descriptors:
 *      pktLen = the length of the packet in bytes
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param descType [IN]    Descriptor type
 *  \param pktLen   [IN]    Packet length
 *
 *  \return None
 */
static inline void CSL_pktdmaCppi5SetPktLen( void *pDesc, uint32_t descType, uint32_t pktLen )
{
    CSL_FINS( ((CSL_PktdmaCppi5HMPD *)pDesc)->descInfo, PKTDMA_CPPI5_PD_DESCINFO_PKTLEN, pktLen );
}

/**
 *  \brief Set the packet length of host mode packet descriptors
 *
 *  This function sets the packet length of host packet descriptors.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param pktLen   [IN]    Packet length in bytes
 *
 *  \return None
 */
static inline void CSL_pktdmaCppi5HostSetPktLen( void *pDesc, uint32_t pktLen )
{
    CSL_FINS( ((CSL_PktdmaCppi5HMPD *)pDesc)->descInfo, PKTDMA_CPPI5_PD_DESCINFO_PKTLEN, pktLen );
}

/**
 *  \brief Get the buffer address
 *
 *  This function returns the physical address of the buffer attached to the
 *  specified descriptor.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc            [IN]    Pointer to the host-mode descriptor
 *
 *  \return Physical address of the buffer
 */
static inline uint64_t CSL_pktdmaCppi5GetBufferAddr( const CSL_PktdmaCppi5HMPD *pDesc )
{
    return pDesc->bufPtr;
}

/**
 *  \brief Get the buffer length
 *
 *  This function returns the length of the buffer (in bytes) attached to the
 *  specified descriptor.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc            [IN]    Pointer to the host-mode descriptor
 *
 *  \return Length of the buffer in bytes
 */
static inline uint32_t CSL_pktdmaCppi5GetBufferLen( const CSL_PktdmaCppi5HMPD *pDesc )
{
    return (uint32_t)CSL_FEXT( pDesc->bufInfo1, PKTDMA_CPPI5_PD_BUFINFO1_LEN );
}

/**
 *  \brief Set the buffer address
 *
 *  This function writes the physical address of the buffer to the specified
 *  descriptor. This function is useful when configuring host-mode
 *  descriptors/buffers to be placed on a transmit queue.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc            [IN]    Pointer to the host-mode descriptor
 *  \param physBufferAddr   [IN]    Physical address of the buffer
 *
 *  \return None
 */
static inline void CSL_pktdmaCppi5SetBufferAddr( CSL_PktdmaCppi5HMPD *pDesc, uint64_t physBufferAddr )
{
    pDesc->bufPtr    = physBufferAddr;
}

/**
 *  \brief Set the original buffer address
 *
 *  This function writes the original physical address of the buffer to the
 *  specified descriptor. This function is useful when initializing host-mode
 *  descriptors/buffers to be placed on transmit or receive free queues.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc            [IN]    Pointer to the host-mode descriptor
 *  \param physBufferAddr   [IN]    Physical address of the buffer
 *
 *  \return None
 */
static inline void CSL_pktdmaCppi5SetOrgBufferAddr( CSL_PktdmaCppi5HMPD *pDesc, uint64_t physBufferAddr )
{
    pDesc->orgBufPtr = physBufferAddr;
}

/**
 *  \brief Set the buffer length
 *
 *  This function writes the buffer length (in bytes) to the specified
 *  descriptor. This function is useful when configuring host-mode
 *  descriptors/buffers to be placed on a transmit queue.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc            [IN]    Pointer to the host-mode descriptor
 *  \param bufferLenBytes   [IN]    Length of the buffer in bytes
 *
 *  \return None
 */
static inline void CSL_pktdmaCppi5SetBufferLen( CSL_PktdmaCppi5HMPD *pDesc, uint32_t bufferLenBytes )
{
    CSL_FINS( pDesc->bufInfo1, PKTDMA_CPPI5_PD_BUFINFO1_LEN, bufferLenBytes );
}

/**
 *  \brief Set the original buffer length
 *
 *  This function writes the original buffer length (in bytes) to the specified
 *  descriptor. This function is useful when initializing host-mode
 *  descriptors/buffers to be placed on transmit or receive free queues.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc            [IN]    Pointer to the host-mode descriptor
 *  \param bufferLenBytes   [IN]    Length of the buffer in bytes
 *
 *  \return None
 */
static inline void CSL_pktdmaCppi5SetOrgBufferLen( CSL_PktdmaCppi5HMPD *pDesc, uint32_t bufferLenBytes )
{
    pDesc->orgBufLen = bufferLenBytes;
}

/**
 *  \brief Link a buffer descriptor to a descriptor
 *
 *  This function links a buffer descriptor to the specified descriptor (which
 *  can be either a packet descriptor or a buffer descriptor).
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  Note that the physical address of the buffer descriptor being linked
 *  must be provided. It is also assumed that the caller has initialized
 *  all required fields of the buffer descriptor prior to linking it.
 *
 *  \param pDesc                [IN]    Pointer to the descriptor being
 *                                      linked to
 *  \param physBufferDescAddr   [IN]    Physical address of the buffer
 *                                      descriptor being linked
 *
 *  \return None
 */
static inline void CSL_pktdmaCppi5LinkDesc( CSL_PktdmaCppi5HMPD *pDesc, uint64_t physBufferDescAddr )
{
    pDesc->nextDescPtr = physBufferDescAddr;
}

/**
 *  \brief Is the EPI block present in the descriptor?
 *
 *  This function returns true if the Extended Packet Info (EPI) block
 *  is present in the descriptor and false if it is not present.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return  true = EPI block is present in the descriptor
 *          false = EPI block is not present in the descriptor
 */
static inline bool CSL_pktdmaCppi5IsEpiDataPresent( const void *pDesc )
{
    uint32_t fieldVal;

    fieldVal = CSL_FEXT( ((const CSL_PktdmaCppi5HMPD *)pDesc)->descInfo, PKTDMA_CPPI5_PD_DESCINFO_EINFO );
    return (fieldVal == (uint32_t)0U) ? (bool)false : (bool)true;
}

/**
 *  \brief Set indicator if the EPI block is present in the descriptor
 *
 *  This function is used to set if the Extended Packet Info (EPI) block
 *  is present in the descriptor (true) or if it is not present (false).
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc            [IN]    Pointer to the descriptor
 *  \param bEpiDataPresent  [IN]    true if EPI block is present, false if not
 *
 *  \return  None
 */
static inline void CSL_pktdmaCppi5SetEpiDataPresent( void *pDesc, bool bEpiDataPresent )
{
    CSL_FINS( ((CSL_PktdmaCppi5HMPD *)pDesc)->descInfo, PKTDMA_CPPI5_PD_DESCINFO_EINFO, bEpiDataPresent==(bool)true ? (uint32_t)1U : (uint32_t)0U );
}

/**
 *  \brief Return pointer to EPI block data
 *
 *  This function returns a pointer to the Extended Packet Info (EPI) block
 *  data within the descriptor. If the EPI block is not present, NULL is
 *  returned.
 *
 *  This function only operates on host-mode descriptors.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return A pointer to the EPI block data within the descriptor is returned,
 *          or NULL is the EPI block is not present in the descriptor
 */
static inline uint32_t *CSL_pktdmaCppi5GetEpiDataPtr( const void *pDesc )
{
    uint32_t *pEpiData = NULL;

    if( CSL_pktdmaCppi5IsEpiDataPresent(pDesc) == (bool)true )
    {
        pEpiData = (uint32_t *)((const uint8_t *)pDesc + sizeof(CSL_PktdmaCppi5HMPD));
    }
    return pEpiData;
}

/**
 *  \brief Read the EPI block data
 *
 *  This function reads and returns the Extended Packet Info (EPI) block
 *  data within the descriptor.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc     [IN]   Pointer to the descriptor
 *  \param pTsInfo   [OUT]  Pointer where to store Timestamp information
 *  \param *pSwInfo0 [OUT]  Pointer where to store Software information word 0
 *  \param *pSwInfo1 [OUT]  Pointer where to store Software information word 1
 *  \param *pSwInfo2 [OUT]  Pointer where to store Software information word 2
 *
 *  \return  0 = success
 *          -1 = No EPI block data is present in the descriptor
 */
static inline int32_t CSL_pktdmaCppi5RdEpiData( const void *pDesc, uint32_t *pTsInfo, uint32_t *pSwInfo0, uint32_t *pSwInfo1, uint32_t *pSwInfo2 )
{
    int32_t retVal = -1;
    uint32_t *pSrcEpiData = CSL_pktdmaCppi5GetEpiDataPtr(pDesc);

    if( pSrcEpiData != (void *)0 )
    {
        *pTsInfo  = *pSrcEpiData; pSrcEpiData++;
        *pSwInfo0 = *pSrcEpiData; pSrcEpiData++;
        *pSwInfo1 = *pSrcEpiData; pSrcEpiData++;
        *pSwInfo2 = *pSrcEpiData;
        retVal = 0;
    }
    return retVal;
}

/**
 *  \brief Write the EPI block data
 *
 *  This function writes the Extended Packet Info (EPI) block data into the
 *  descriptor.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param tsInfo   [IN]    Timestamp information
 *  \param swInfo0  [IN]    Software information word 0
 *  \param swInfo1  [IN]    Software information word 1
 *  \param swInfo2  [IN]    Software information word 2
 *
 *  \return  None
 */
static inline void CSL_pktdmaCppi5WrEpiData( const void *pDesc, uint32_t tsInfo, uint32_t swInfo0, uint32_t swInfo1, uint32_t swInfo2 )
{
    uint32_t *pDstEpiData = CSL_pktdmaCppi5GetEpiDataPtr(pDesc);

    if( pDstEpiData != (void *)0 )
    {
        *pDstEpiData = tsInfo;  pDstEpiData++;
        *pDstEpiData = swInfo0; pDstEpiData++;
        *pDstEpiData = swInfo1; pDstEpiData++;
        *pDstEpiData = swInfo2;
    }
}

/**
 *  \brief Get the location of the protocol-specific data
 *
 *  This function returns the location of the protocol-specific data.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return 0 = Protocol-specific data is in the descriptor
 */
static inline uint32_t CSL_pktdmaCppi5GetPsDataLoc( const void *pDesc )
{
    return (uint32_t)0U;
}

/**
 *  \brief Set the location of the protocol-specific data
 *
 *  This function sets the location of the protocol-specific data.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param psLoc    [IN]    0 = Protocol-specific data is in the descriptor
 *
 *  \return None
 */
static inline void CSL_pktdmaCppi5SetPsDataLoc( void *pDesc, uint32_t psLoc )
{
}

/**
 *  \brief Get the number of bytes of protocol-specific data
 *
 *  This function returns the number of bytes of protocol-specific (ps) data
 *  present in the descriptor. This value will be a multiple of 4.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return The number of bytes of ps data is returned
 */
static inline uint32_t CSL_pktdmaCppi5GetPsDataLen( const void *pDesc )
{
    return ((uint32_t)CSL_FEXT( ((const CSL_PktdmaCppi5HMPD *)pDesc)->descInfo, PKTDMA_CPPI5_PD_DESCINFO_PSWCNT )) * 4U;
}

/**
 *  \brief Set the number of bytes of protocol-specific data
 *
 *  This function sets the number of bytes of protocol-specific (ps) data
 *  present in the descriptor or SOP buffer. This value must be a multiple
 *  of 4.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc        [IN]    Pointer to the descriptor
 *  \param psDataLen    [IN]    Number of bytes of protocol-specific (ps) data
 *                              present in the descriptor or SOP buffer
 *
 *  \return None
 */
static inline void CSL_pktdmaCppi5SetPsDataLen( void *pDesc, uint32_t psDataLen )
{
    CSL_FINS( ((CSL_PktdmaCppi5HMPD *)pDesc)->descInfo, PKTDMA_CPPI5_PD_DESCINFO_PSWCNT, (psDataLen/4U) );
}

/**
 *  \brief Return address of the protocol-specific data
 *
 *  This function returns the address of the protocol-specific data within the
 *  descriptor or SOP buffer.
 *
 *  If the protocol-specific data is in the descriptor (bInDesc is true),
 *  the address returned is a virtual address. If the protocol-specific data
 *  is in the SOP buffer (bInDesc is false), the address returned is a
 *  physical address.
 *
 *  For the pktdma, bInSopBuf is always assumed false (psData is in the descriptor).

 *  This function only operates on host-mode descriptors.
 *
 *  \param pDesc        [IN]    Pointer to the descriptor
 *  \param bInSopBuf    [IN]    true if ps data is in SOP buffer, false otherwise
 *  \param bEpiPresent  [IN]    true if EPI data is present, false otherwise
 *
 *  \return Address of the protocol-specific data
 */
static inline uint64_t CSL_pktdmaCppi5GetPsDataAddr( const void *pDesc, bool bInSopBuf, bool bEpiPresent )
{
    uint64_t psDataAddr;
    uint32_t epiDataSize;

    epiDataSize = (bEpiPresent == (bool)false) ? (uint32_t)0U : (uint32_t)sizeof(CSL_PktdmaCppi5Epi);
    /* psData is in the descriptor */
    psDataAddr = (uint64_t)(uintptr_t)((const uint8_t *)pDesc + sizeof(CSL_PktdmaCppi5HMPD) + epiDataSize);
    return psDataAddr;
}

/**
 *  \brief Return pointer to the protocol-specific data
 *
 *  This function returns a pointer to the protocol-specific data within the
 *  descriptor or SOP buffer.
 *
 *  \param pDesc        [IN]    Pointer to the descriptor
 *
 *  \return Pointer to the protocol-specific data (NULL if no protocol-specific data is available)
 */
static inline uint8_t *CSL_pktdmaCppi5GetPsDataPtr( const void *pDesc )
{
    uint8_t *pPsData = NULL;
    uint32_t psDataLen;

    psDataLen = CSL_pktdmaCppi5GetPsDataLen( pDesc );
    if( psDataLen != (uint32_t)0U )
    {
        uint32_t epiDataSize;

        epiDataSize = (CSL_pktdmaCppi5IsEpiDataPresent( pDesc ) == (bool)false) ? (uint32_t)0U : (uint32_t)sizeof(CSL_PktdmaCppi5Epi);
        /* psData is in the descriptor */
        pPsData = (uint8_t *)((const uint8_t *)pDesc + sizeof(CSL_PktdmaCppi5HMPD) + epiDataSize);
    }
    return pPsData;
}

/**
 *  \brief Get source tag
 *
 *  This function gets the source tag from the descriptor.
 *
 *  \param pDesc        [IN]    Pointer to the descriptor
 *
 *  \return  Source tag
 */
static inline uint32_t CSL_pktdmaCppi5GetSrcTag( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_PktdmaCppi5HMPD *)pDesc)->srcDstTag, PKTDMA_CPPI5_PD_SRCDSTTAG_SRCTAG );
}

/**
 *  \brief Get destination tag
 *
 *  This function gets the destination tag from the descriptor.
 *
 *  \param pDesc        [IN]    Pointer to the descriptor
 *
 *  \return  Destination tag
 */
static inline uint32_t CSL_pktdmaCppi5GetDstTag( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_PktdmaCppi5HMPD *)pDesc)->srcDstTag, PKTDMA_CPPI5_PD_SRCDSTTAG_DSTTAG );
}

/**
 *  \brief Set source tag
 *
 *  This function sets the source tag in the descriptor.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param srcTag   [IN]    source tag
 *
 *  \return  None
 */
static inline void CSL_pktdmaCppi5SetSrcTag( void *pDesc, uint32_t srcTag )
{
    CSL_FINS( ((CSL_PktdmaCppi5HMPD *)pDesc)->srcDstTag, PKTDMA_CPPI5_PD_SRCDSTTAG_SRCTAG, srcTag );
}

/**
 *  \brief Set destination tag
 *
 *  This function sets the destination tag in the descriptor.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param dstTag   [IN]    destination tag
 *
 *  \return  None
 */
static inline void CSL_pktdmaCppi5SetDstTag( void *pDesc, uint32_t dstTag )
{
    CSL_FINS( ((CSL_PktdmaCppi5HMPD *)pDesc)->srcDstTag, PKTDMA_CPPI5_PD_SRCDSTTAG_DSTTAG, dstTag );
}

/**
 *  \brief Get error flags
 *
 *  This function gets the error flags from the descriptor.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return  The error flags are returned
 */
static inline uint32_t CSL_pktdmaCppi5GetErrorFlags( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_PktdmaCppi5HMPD *)pDesc)->pktInfo1, PKTDMA_CPPI5_PD_PKTINFO1_PKTERROR );
}

/**
 *  \brief Get ps flags
 *
 *  This function gets the protocol-specific (ps) flags from the descriptor.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return  The protocol-specific (ps) flags are returned
 */
static inline uint32_t CSL_pktdmaCppi5GetPsFlags( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_PktdmaCppi5HMPD *)pDesc)->pktInfo1, PKTDMA_CPPI5_PD_PKTINFO1_PSFLGS );
}

/**
 *  \brief Set ps flags
 *
 *  This function sets the protocol-specific (ps) flags in the descriptor.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param psFlags  [IN]    ps flags
 *
 *  \return  None
 */
static inline void CSL_pktdmaCppi5SetPsFlags( void *pDesc, uint32_t psFlags )
{
    CSL_FINS( ((CSL_PktdmaCppi5HMPD *)pDesc)->pktInfo1, PKTDMA_CPPI5_PD_PKTINFO1_PSFLGS, psFlags );
}

/**
 *  \brief Get packet type
 *
 *  This function gets the packet type from the descriptor.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return  Packet type
 */
static inline uint32_t CSL_pktdmaCppi5GetPktType( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_PktdmaCppi5HMPD *)pDesc)->pktInfo2, PKTDMA_CPPI5_PD_PKTINFO2_PKTTYPE );
}

/**
 *  \brief Set packet type
 *
 *  This function sets the packet type in the descriptor.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param pktType  [IN]    Packet type
 *
 *  \return  None
 */
static inline void CSL_pktdmaCppi5SetPktType( void *pDesc, uint32_t pktType  )
{
    CSL_FINS( ((CSL_PktdmaCppi5HMPD *)pDesc)->pktInfo2, PKTDMA_CPPI5_PD_PKTINFO2_PKTTYPE, pktType );
}

/**
 *  \brief Get IDs
 *
 *  This function gets the packet ID and flow ID from the descriptor.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param pPktId   [OUT]   Pointer where to store packet ID
 *  \param pFlowId  [OUT]   Pointer where to store flow ID
 *
 *  \return  None
 */
static inline void CSL_pktdmaCppi5GetIds( const void *pDesc, uint32_t *pPktId, uint32_t *pFlowId )
{
    *pPktId     = (uint32_t) 0U;
    *pFlowId    = CSL_FEXT( ((const CSL_PktdmaCppi5HMPD *)pDesc)->pktInfo1, PKTDMA_CPPI5_PD_PKTINFO1_FLOWID );
}

/**
 *  \brief Set IDs
 *
 *  This function sets the packet ID and flow ID in the descriptor.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param descType [IN]    Descriptor type
 *  \param pktId    [IN]    packet ID
 *  \param flowId   [IN]    flow ID
 *
 *  \return  None
 */
static inline void CSL_pktdmaCppi5SetIds( void *pDesc, uint32_t descType, uint32_t pktId, uint32_t flowId )
{
    uint32_t v;

    v = ((CSL_PktdmaCppi5HMPD *)pDesc)->pktInfo1;
    v &= ~CSL_PKTDMA_CPPI5_PD_PKTINFO1_FLOWID_MASK;
    v |= CSL_FMK( PKTDMA_CPPI5_PD_PKTINFO1_FLOWID, flowId );
    ((CSL_PktdmaCppi5HMPD *)pDesc)->pktInfo1 = v;
}

/**
 *  \brief Get descriptor return policy information
 *
 *  This function gets descriptor return policy information from the
 *  descriptor. See #CSL_pktdmaCppi5SetReturnPolicy for a description of the
 *  return policy information.
 *
 *  For the pktdma, this function returns 0U for all values.
 *
 *  \param pDesc          [IN]    Pointer to the descriptor
 *  \param pRetPolicy     [OUT]   Pointer where to store return policy
 *  \param pEarlyReturn   [OUT]   Pointer where to store early return
 *  \param pRetPushPolicy [OUT]   Pointer where to store return push policy
 *  \param pRetQnum       [OUT]   Pointer where to store return queue #
 *
 *  \return  None
 */
static inline void CSL_pktdmaCppi5GetReturnPolicy( const void *pDesc, uint32_t *pRetPolicy, uint32_t *pEarlyReturn, uint32_t *pRetPushPolicy, uint32_t *pRetQnum )
{
    *pRetPolicy     = (uint32_t)0U;
    *pEarlyReturn   = (uint32_t)0U;
    *pRetPushPolicy = (uint32_t)0U;
    *pRetQnum       = (uint32_t)0U;
}

/**
 *  \brief Set descriptor return policy information
 *
 *  This function sets descriptor return policy information in the
 *  descriptor.
 *
 *  For the pktdma, this function does nothing.
 *
 *  \param pDesc         [IN]   Pointer to the descriptor
 *  \param descType      [IN]   Descriptor type
 *  \param retPolicy     [IN]   return policy
 *  \param earlyReturn   [IN]   early return
 *  \param retPushPolicy [IN]   return push policy
 *  \param retQnum       [IN]   return queue #
 *
 *  \return  None
 */
static inline void CSL_pktdmaCppi5SetReturnPolicy( void *pDesc, uint32_t descType, uint32_t retPolicy, uint32_t earlyReturn, uint32_t retPushPolicy, uint32_t retQnum )
{
}

/**
 *  \brief Clear the asel field in an address
 *
 *  This function is used to clear the asel value in the specified address.
 *
 *  \param addr             [IN]    The 64-bit address
 *
 *  \return  The address with the asel field cleared is returned
 */
static inline uint64_t CSL_pktdmaClrAselInAddr( uint64_t addr )
{
    uint64_t retAddr = addr;

    CSL_FINS( retAddr, PKTDMA_CPPI5_PD_NEXTDESCPTR_ASPACE, (uint64_t)0UL );
    return retAddr;
}

/**
 *  \brief Set the asel field in an address
 *
 *  This function is used to set the specified asel value in the specified address.
 *
 *  \param addr             [IN]    The address
 *  \param asel             [IN]    Address select (asel) endpoint value. See #CSL_LcdmaRingAccAselEndpoint.
 *
 *  \return  The address including the asel value is returned
 */
static inline uint64_t CSL_pktdmaSetAselInAddr( uint64_t addr, uint32_t asel )
{
    uint64_t retAddr = addr;

    CSL_FINS( retAddr, PKTDMA_CPPI5_PD_NEXTDESCPTR_ASPACE, (uint64_t)asel );
    return retAddr;
}

/* @} */

#ifdef __cplusplus
}
#endif  // extern "C"

#endif
/** @} */

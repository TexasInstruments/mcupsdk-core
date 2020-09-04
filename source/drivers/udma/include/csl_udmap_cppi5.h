/*
 *  Copyright (C) 2016-2019 Texas Instruments Incorporated.
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
 *  \file  csl_udmap_cppi5.h
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

#ifndef CSL_UDMAP_CPPI5_H_
#define CSL_UDMAP_CPPI5_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <drivers/hw_include/cslr.h>

/** ===========================================================================
 *
 * @defgroup CSL_UDMAP_CPPI5_API CPPI5 API
 * @ingroup DRV_UDMA_MODULE
 * ============================================================================
 */
/**
@defgroup CSL_UDMAP_CPPI5_DATASTRUCT  CPPI5 Data Structures
@ingroup CSL_UDMAP_CPPI5_API
*/
/**
@defgroup CSL_UDMAP_CPPI5_FUNCTION  CPPI5 Functions
@ingroup CSL_UDMAP_CPPI5_API
*/
/**
@defgroup CSL_UDMAP_CPPI5_ENUM CPPI5 Enumerated Data Types
@ingroup CSL_UDMAP_CPPI5_API
*/

/**
 *  \addtogroup CSL_UDMAP_CPPI5_DATASTRUCT
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
} CSL_UdmapCppi5Epi;

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
} CSL_UdmapCppi5HMPD;

/** \brief Monolithic-mode packet descriptor
 *
 *  This structure is used for monolithic-mode packet descriptors.
 *
 */
typedef struct
{
  uint32_t      descInfo;       /**< word 0: Monolithic Packet Descriptor Word 0 */
  uint32_t      pktInfo1;       /**< word 1: Monolithic Packet Descriptor Word 1 */
  uint32_t      pktInfo2;       /**< word 2: Monolithic Packet Descriptor Word 2 */
  uint32_t      srcDstTag;      /**< word 3: Monolithic Packet Descriptor Word 3 */
                                /* Extended Packet Info Data (optional, 4 words) */
} CSL_UdmapCppi5MMPD;

/* @} */

/*-----------------------------------------------------------------------------
// Packet and Buffer Descriptor field manipulation macros
//---------------------------------------------------------------------------*/
#define CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_SHIFT             ((uint32_t) 30U)
#define CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_MASK              (((uint32_t) 0x3U) << CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_SHIFT)
#define   CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST            ((uint32_t) 1U)
#define   CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_MONO            ((uint32_t) 2U)
#define   CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_TR              ((uint32_t) 3U)
#define CSL_UDMAP_CPPI5_PD_DESCINFO_EINFO_SHIFT             ((uint32_t) 29U)
#define CSL_UDMAP_CPPI5_PD_DESCINFO_EINFO_MASK              (((uint32_t) 0x1U) << CSL_UDMAP_CPPI5_PD_DESCINFO_EINFO_SHIFT)
#define   CSL_UDMAP_CPPI5_PD_DESCINFO_EINFO_VAL_NOT_PRESENT     ((uint32_t) 0)
#define   CSL_UDMAP_CPPI5_PD_DESCINFO_EINFO_VAL_IS_PRESENT      ((uint32_t) 1U)
#define CSL_UDMAP_CPPI5_PD_DESCINFO_PSINFO_SHIFT            ((uint32_t) 28U)
#define CSL_UDMAP_CPPI5_PD_DESCINFO_PSINFO_MASK             (((uint32_t) 0x1U) << CSL_UDMAP_CPPI5_PD_DESCINFO_PSINFO_SHIFT)
#define   CSL_UDMAP_CPPI5_PD_DESCINFO_PSINFO_VAL_IN_DESC        ((uint32_t) 0)
#define   CSL_UDMAP_CPPI5_PD_DESCINFO_PSINFO_VAL_IN_SOP_BUFFER  ((uint32_t) 1U)
#define CSL_UDMAP_CPPI5_PD_DESCINFO_PSWCNT_SHIFT            ((uint32_t) 22U)
#define CSL_UDMAP_CPPI5_PD_DESCINFO_PSWCNT_MASK             (((uint32_t) 0x3FU) << CSL_UDMAP_CPPI5_PD_DESCINFO_PSWCNT_SHIFT)
#define CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_SHIFT            ((uint32_t) 0)
#define CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_MASK             (((uint32_t) 0x3FFFFFU) << CSL_UDMAP_CPPI5_PD_DESCINFO_PKTLEN_SHIFT)

#define CSL_UDMAP_CPPI5_PD_PKTINFO1_PKTERROR_SHIFT          ((uint32_t) 28U)
#define CSL_UDMAP_CPPI5_PD_PKTINFO1_PKTERROR_MASK           (((uint32_t) 0xFU) << CSL_UDMAP_CPPI5_PD_PKTINFO1_PKTERROR_SHIFT)
#define CSL_UDMAP_CPPI5_PD_PKTINFO1_PSFLGS_SHIFT            ((uint32_t) 24U)
#define CSL_UDMAP_CPPI5_PD_PKTINFO1_PSFLGS_MASK             (((uint32_t) 0xFU) << CSL_UDMAP_CPPI5_PD_PKTINFO1_PSFLGS_SHIFT)
#define CSL_UDMAP_CPPI5_PD_PKTINFO1_PKTID_SHIFT             ((uint32_t) 14U)
#define CSL_UDMAP_CPPI5_PD_PKTINFO1_PKTID_MASK              (((uint32_t) 0x3FFU) << CSL_UDMAP_CPPI5_PD_PKTINFO1_PKTID_SHIFT)
#define CSL_UDMAP_CPPI5_PD_PKTINFO1_FLOWID_SHIFT            ((uint32_t) 0)
#define CSL_UDMAP_CPPI5_PD_PKTINFO1_FLOWID_MASK             (((uint32_t) 0x3FFFU) << CSL_UDMAP_CPPI5_PD_PKTINFO1_FLOWID_SHIFT)

#define CSL_UDMAP_CPPI5_PD_PKTINFO2_PKTTYPE_SHIFT           ((uint32_t) 27U)
#define CSL_UDMAP_CPPI5_PD_PKTINFO2_PKTTYPE_MASK            (((uint32_t) 0x1FU) << CSL_UDMAP_CPPI5_PD_PKTINFO2_PKTTYPE_SHIFT)
#define CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY_SHIFT         ((uint32_t) 18U)
#define CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY_MASK          (((uint32_t) 0x1U) << CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY_SHIFT)
#define   CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY_VAL_ENTIRE_PKT  ((uint32_t) 0)
#define   CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY_VAL_BREAKUP_PKT ((uint32_t) 1U)
#define CSL_UDMAP_CPPI5_PD_PKTINFO2_DATA_OFFSET_SHIFT       ((uint32_t) 18U)
#define CSL_UDMAP_CPPI5_PD_PKTINFO2_DATA_OFFSET_MASK        (((uint32_t) 0x1FFU) << CSL_UDMAP_CPPI5_PD_PKTINFO2_DATA_OFFSET_SHIFT)
#define CSL_UDMAP_CPPI5_PD_PKTINFO2_EARLYRET_SHIFT          ((uint32_t) 17U)
#define CSL_UDMAP_CPPI5_PD_PKTINFO2_EARLYRET_MASK           (((uint32_t) 0x1U) << CSL_UDMAP_CPPI5_PD_PKTINFO2_EARLYRET_SHIFT)
#define   CSL_UDMAP_CPPI5_PD_PKTINFO2_EARLYRET_VAL_NO           ((uint32_t) 0)
#define   CSL_UDMAP_CPPI5_PD_PKTINFO2_EARLYRET_VAL_YES          ((uint32_t) 1U)
#define CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY_SHIFT     ((uint32_t) 16U)
#define CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY_MASK      (((uint32_t) 0x1U) << CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY_SHIFT)
#define   CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY_VAL_TO_TAIL ((uint32_t) 0)
#define   CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY_VAL_TO_HEAD ((uint32_t) 1U)
#define CSL_UDMAP_CPPI5_PD_PKTINFO2_RETQ_SHIFT              ((uint32_t) 0)
#define CSL_UDMAP_CPPI5_PD_PKTINFO2_RETQ_MASK               (((uint32_t) 0xFFFFU) << CSL_UDMAP_CPPI5_PD_PKTINFO2_RETQ_SHIFT)

#define CSL_UDMAP_CPPI5_PD_SRCDSTTAG_SRCTAG_SHIFT           ((uint32_t) 16U)
#define CSL_UDMAP_CPPI5_PD_SRCDSTTAG_SRCTAG_MASK            (((uint32_t) 0xFFFFU) << CSL_UDMAP_CPPI5_PD_SRCDSTTAG_SRCTAG_SHIFT)
#define CSL_UDMAP_CPPI5_PD_SRCDSTTAG_DSTTAG_SHIFT           ((uint32_t) 0)
#define CSL_UDMAP_CPPI5_PD_SRCDSTTAG_DSTTAG_MASK            (((uint32_t) 0xFFFFU) << CSL_UDMAP_CPPI5_PD_SRCDSTTAG_DSTTAG_SHIFT)

#define CSL_UDMAP_CPPI5_PD_BUFPTR_ASPACE_SHIFT              ((uint64_t) 48UL)
#define CSL_UDMAP_CPPI5_PD_BUFPTR_ASPACE_MASK               (((uint64_t) 0xFU) << CSL_UDMAP_CPPI5_PD_BUFPTR_ASPACE_SHIFT)
#define CSL_UDMAP_CPPI5_PD_BUFPTR_ADDR_SHIFT                ((uint64_t) 0U)
#define CSL_UDMAP_CPPI5_PD_BUFPTR_ADDR_MASK                 (((uint64_t) 0x0000FFFFFFFFFFFFUL) << CSL_UDMAP_CPPI5_PD_BUFPTR_ADDR_SHIFT)

#define CSL_UDMAP_CPPI5_PD_BUFINFO1_LEN_SHIFT               ((uint32_t) 0)
#define CSL_UDMAP_CPPI5_PD_BUFINFO1_LEN_MASK                (((uint32_t) 0x3FFFFFU) << CSL_UDMAP_CPPI5_PD_BUFINFO1_LEN_SHIFT)

#define CSL_UDMAP_CPPI5_PD_ORGBUFLEN_LEN_SHIFT              ((uint32_t) 0)
#define CSL_UDMAP_CPPI5_PD_ORGBUFLEN_LEN_MASK               (((uint32_t) 0x3FFFFFU) << CSL_UDMAP_CPPI5_PD_ORGBUFLEN_LEN_SHIFT)

#define CSL_UDMAP_CPPI5_PD_ORGBUFPTR_ASPACE_SHIFT           ((uint64_t) 48UL)
#define CSL_UDMAP_CPPI5_PD_ORGBUFPTR_ASPACE_MASK            (((uint64_t) 0xFU) << CSL_UDMAP_CPPI5_PD_ORGBUFPTR_ASPACE_SHIFT)
#define CSL_UDMAP_CPPI5_PD_ORGBUFPTR_ADDR_SHIFT             ((uint64_t) 0U)
#define CSL_UDMAP_CPPI5_PD_ORGBUFPTR_ADDR_MASK              (((uint64_t) 0x0000FFFFFFFFFFFFUL) << CSL_UDMAP_CPPI5_PD_ORGBUFPTR_ADDR_SHIFT)

/*-----------------------------------------------------------------------------
// TR Descriptor
//---------------------------------------------------------------------------*/
typedef struct
{
  uint32_t      descInfo;       /**< word 0: TR Descriptor Info */
  uint32_t      pktInfo;        /**< word 1: TR Packet Info */
  uint32_t      retInfo;        /**< word 2: TR Return Info */
  uint32_t      srcDstTag;      /**< word 3: Sourc/Dest Tag */
} CSL_UdmapCppi5TRPD;

/*-----------------------------------------------------------------------------
// TR Descriptor field manipulation macros
//---------------------------------------------------------------------------*/
#define CSL_UDMAP_CPPI5_TRPD_DESCINFO_DTYPE_SHIFT           ((uint32_t) 30U)
#define CSL_UDMAP_CPPI5_TRPD_DESCINFO_DTYPE_MASK            (((uint32_t) 0x3U) << CSL_UDMAP_CPPI5_TRPD_DESCINFO_DTYPE_SHIFT)
#define CSL_UDMAP_CPPI5_TRPD_DESCINFO_RELOAD_SHIFT          ((uint32_t) 20U)
#define CSL_UDMAP_CPPI5_TRPD_DESCINFO_RELOAD_MASK           (((uint32_t) 0x1FFU) << CSL_UDMAP_CPPI5_TRPD_DESCINFO_RELOAD_SHIFT)
#define CSL_UDMAP_CPPI5_TRPD_DESCINFO_RLDIDX_SHIFT          ((uint32_t) 14U)
#define CSL_UDMAP_CPPI5_TRPD_DESCINFO_RLDIDX_MASK           (((uint32_t) 0x3FU) << CSL_UDMAP_CPPI5_TRPD_DESCINFO_RLDIDX_SHIFT)
#define CSL_UDMAP_CPPI5_TRPD_DESCINFO_LASTIDX_SHIFT         ((uint32_t) 0)
#define CSL_UDMAP_CPPI5_TRPD_DESCINFO_LASTIDX_MASK          (((uint32_t) 0x3FFFU) << CSL_UDMAP_CPPI5_TRPD_DESCINFO_LASTIDX_SHIFT)

#define CSL_UDMAP_CPPI5_TRPD_PKTINFO_PKTERROR_SHIFT         ((uint32_t) 28U)
#define CSL_UDMAP_CPPI5_TRPD_PKTINFO_PKTERROR_MASK          (((uint32_t) 0xFU) << CSL_UDMAP_CPPI5_TRPD_PKTINFO_PKTERROR_SHIFT)
#define CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_SHIFT          ((uint32_t) 24U)
#define CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_MASK           (((uint32_t) 0x7U) << CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_SHIFT)
#define   CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_16B          ((uint32_t) 0)
#define   CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_32B          ((uint32_t) 1U)
#define   CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B          ((uint32_t) 2U)
#define   CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_128B         ((uint32_t) 3U)
#define CSL_UDMAP_CPPI5_TRPD_PKTINFO_PKTID_SHIFT            ((uint32_t) 14U)
#define CSL_UDMAP_CPPI5_TRPD_PKTINFO_PKTID_MASK             (((uint32_t) 0x3FFU) << CSL_UDMAP_CPPI5_TRPD_PKTINFO_PKTID_SHIFT)
#define CSL_UDMAP_CPPI5_TRPD_PKTINFO_FLOWID_SHIFT           ((uint32_t) 0)
#define CSL_UDMAP_CPPI5_TRPD_PKTINFO_FLOWID_MASK            (((uint32_t) 0x3FFFU) << CSL_UDMAP_CPPI5_TRPD_PKTINFO_FLOWID_SHIFT)

#define CSL_UDMAP_CPPI5_TRPD_RETINFO_RETPOLICY_SHIFT        ((uint32_t) 16U)
#define CSL_UDMAP_CPPI5_TRPD_RETINFO_RETPOLICY_MASK         (((uint32_t) 0x1U) << CSL_UDMAP_CPPI5_TRPD_RETINFO_RETPOLICY_SHIFT)
#define   CSL_UDMAP_CPPI5_TRPD_RETINFO_RETPOLICY_VAL_TO_TAIL    ((uint32_t) 0)
#define   CSL_UDMAP_CPPI5_TRPD_RETINFO_RETPOLICY_VAL_TO_HEAD    ((uint32_t) 1U)
#define CSL_UDMAP_CPPI5_TRPD_RETINFO_RETQ_SHIFT             ((uint32_t) 0)
#define CSL_UDMAP_CPPI5_TRPD_RETINFO_RETQ_MASK              (((uint32_t) 0xFFFFU) << CSL_UDMAP_CPPI5_TRPD_RETINFO_RETQ_SHIFT)

#define CSL_UDMAP_CPPI5_TRPD_SRCDSTTAG_SRCTAG_SHIFT         ((uint32_t) 16U)
#define CSL_UDMAP_CPPI5_TRPD_SRCDSTTAG_SRCTAG_MASK          (((uint32_t) 0xFFFFU) << CSL_UDMAP_CPPI5_TRPD_SRCDSTTAG_SRCTAG_SHIFT)
#define CSL_UDMAP_CPPI5_TRPD_SRCDSTTAG_DSTTAG_SHIFT         ((uint32_t) 0)
#define CSL_UDMAP_CPPI5_TRPD_SRCDSTTAG_DSTTAG_MASK          (((uint32_t) 0xFFFFU) << CSL_UDMAP_CPPI5_TRPD_SRCDSTTAG_DSTTAG_SHIFT)

/**
 *  \addtogroup CSL_UDMAP_CPPI5_FUNCTION
 *  @{
 */
static inline uint32_t  CSL_udmapCppi5GetDescType( const void *pDesc );
static inline void      CSL_udmapCppi5SetDescType( void *pDesc, uint32_t descType );
static inline uint32_t  CSL_udmapCppi5GetPktLen( const void *pDesc );
static inline void      CSL_udmapCppi5SetPktLen( void *pDesc, uint32_t descType, uint32_t pktLen );
static inline void      CSL_udmapCppi5HostSetPktLen( void *pDesc, uint32_t pktLen );
static inline void      CSL_udmapCppi5MonoSetPktLen( void *pDesc, uint32_t pktLen );
static inline void      CSL_udmapCppi5TrSetPktLen( void *pDesc, uint32_t pktLen );
static inline uint64_t  CSL_udmapCppi5GetBufferAddr( const CSL_UdmapCppi5HMPD *pDesc );
static inline uint32_t  CSL_udmapCppi5GetBufferLen( const CSL_UdmapCppi5HMPD *pDesc );
static inline void      CSL_udmapCppi5SetBufferAddr( CSL_UdmapCppi5HMPD *pDesc, uint64_t physBufferAddr );
static inline void      CSL_udmapCppi5SetOrgBufferAddr( CSL_UdmapCppi5HMPD *pDesc, uint64_t physBufferAddr );
static inline void      CSL_udmapCppi5SetBufferLen( CSL_UdmapCppi5HMPD *pDesc, uint32_t bufferLenBytes );
static inline void      CSL_udmapCppi5SetOrgBufferLen( CSL_UdmapCppi5HMPD *pDesc, uint32_t bufferLenBytes );
static inline void      CSL_udmapCppi5LinkDesc( CSL_UdmapCppi5HMPD *pDesc, uint64_t physBufferDescAddr );
static inline bool      CSL_udmapCppi5IsEpiDataPresent( const void *pDesc );
static inline void      CSL_udmapCppi5SetEpiDataPresent( void *pDesc, bool bEpiDataPresent );
static inline uint32_t *CSL_udmapCppi5GetEpiDataPtr( const void *pDesc );
static inline int32_t   CSL_udmapCppi5RdEpiData( const void *pDesc, uint32_t *pTsInfo, uint32_t *pSwInfo0, uint32_t *pSwInfo1, uint32_t *pSwInfo2 );
static inline void      CSL_udmapCppi5WrEpiData( const void *pDesc, uint32_t tsInfo, uint32_t swInfo0, uint32_t swInfo1, uint32_t swInfo2 );
static inline uint32_t  CSL_udmapCppi5GetPsDataLoc( const void *pDesc );
static inline void      CSL_udmapCppi5SetPsDataLoc( void *pDesc, uint32_t psLoc );
static inline uint32_t  CSL_udmapCppi5GetPsDataLen( const void *pDesc );
static inline void      CSL_udmapCppi5SetPsDataLen( void *pDesc, uint32_t psDataLen );
static inline uint64_t  CSL_udmapCppi5GetPsDataAddr( const void *pDesc, bool bInSopBuf, bool bEpiPresent );
static inline uint8_t   *CSL_udmapCppi5GetPsDataPtr( const void *pDesc );
static inline uint32_t  CSL_udmapCppi5GetSrcTag( const void *pDesc );
static inline uint32_t  CSL_udmapCppi5GetDstTag( const void *pDesc );
static inline void      CSL_udmapCppi5SetSrcTag( void *pDesc, uint32_t srcTag );
static inline void      CSL_udmapCppi5SetDstTag( void *pDesc, uint32_t dstTag );
static inline uint32_t  CSL_udmapCppi5GetErrorFlags( const void *pDesc );
static inline uint32_t  CSL_udmapCppi5GetPsFlags( const void *pDesc );
static inline void      CSL_udmapCppi5SetPsFlags( void *pDesc, uint32_t psFlags );
static inline uint32_t  CSL_udmapCppi5GetPktType( const void *pDesc );
static inline void      CSL_udmapCppi5SetPktType( void *pDesc, uint32_t pktType );
static inline void      CSL_udmapCppi5GetIds( const void *pDesc, uint32_t *pPktId, uint32_t *pFlowId );
static inline void      CSL_udmapCppi5SetIds( void *pDesc, uint32_t descType, uint32_t pktId, uint32_t flowId );
static inline void      CSL_udmapCppi5GetReturnPolicy( const void *pDesc, uint32_t *pRetPolicy, uint32_t *pEarlyReturn, uint32_t *pRetPushPolicy, uint32_t *pRetQnum );
static inline void      CSL_udmapCppi5SetReturnPolicy( void *pDesc, uint32_t descType, uint32_t retPolicy, uint32_t earlyReturn, uint32_t retPushPolicy, uint32_t retQnum );
static inline uint32_t  CSL_udmapCppi5MonoGetDataOffset( const CSL_UdmapCppi5MMPD *pDesc );
static inline void      CSL_udmapCppi5MonoSetDataOffset( CSL_UdmapCppi5MMPD *pDesc, uint32_t dataOffset );
static inline void      CSL_udmapCppi5TrGetReload( const CSL_UdmapCppi5TRPD *pDesc, uint32_t *pReloadEnable, uint32_t *pReloadIdx );
static inline void      CSL_udmapCppi5TrSetReload( CSL_UdmapCppi5TRPD *pDesc, uint32_t reloadEnable, uint32_t reloadIdx );
static inline uint32_t  CSL_udmapCppi5TrGetEntryStride( const CSL_UdmapCppi5TRPD *pDesc );
static inline void      CSL_udmapCppi5TrSetEntryStride( CSL_UdmapCppi5TRPD *pDesc, uint32_t nomElSz );

/**
 *  \brief Get the descriptor type
 *
 *  This function returns the type of descriptor. Valid descriptor types are:
 *    1 = Host-mode (HOST) packet descriptor
 *    2 = Monolithic-mode (MONO) packet descriptor
 *    3 = Transfer Request mode (TR) packet descriptor
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return Descriptor type
 */
static inline uint32_t CSL_udmapCppi5GetDescType( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_DTYPE );
}

/**
 *  \brief Set the descriptor type
 *
 *  This function sets the type of descriptor. Valid descriptor types are:
 *    1 = Host-mode (HOST) packet descriptor
 *    2 = Monolithic-mode (MONO) packet descriptor
 *    3 = Transfer Request mode (TR) packet descriptor
 *
 *  Note that no error checking is performed on the passed descriptor
 *  type argument.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param descType [IN]    Descriptor type
 *
 *  \return None
 */
static inline void CSL_udmapCppi5SetDescType( void *pDesc, uint32_t descType )
{
    CSL_FINS( ((CSL_UdmapCppi5HMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_DTYPE, descType );
}

/**
 *  \brief Get the packet length
 *
 *  This function returns the packet length. The value returned depends on
 *  the type of descriptor as follows:
 *
 *  Host and mono mode packet descriptors:
 *      The length of the packet in bytes is returned
 *
 *  TR mode packet descriptors:
 *      The number of TRs in the packet is returned
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return Packet length
 */
static inline uint32_t CSL_udmapCppi5GetPktLen( const void *pDesc )
{
    uint32_t pktLen;
    uint32_t descType = CSL_udmapCppi5GetDescType( pDesc );

    if( (descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST) ||
        (descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_MONO) )
    {
        pktLen = CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_PKTLEN );
    }
    else if( descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_TR )
    {
        pktLen = (uint32_t)CSL_FEXT( ((const CSL_UdmapCppi5TRPD *)pDesc)->descInfo, UDMAP_CPPI5_TRPD_DESCINFO_LASTIDX ) + (uint32_t)1U;
    }
    else
    {
        pktLen = 0;
    }
    return pktLen;
}

/**
 *  \brief Set the packet length
 *
 *  This function sets the packet length. The meaning of the pktLen argument
 *  depends on the type of descriptor as follows:
 *
 *  Host and mono mode packet descriptors:
 *      pktLen = the length of the packet in bytes
 *
 *  TR mode packet descriptors:
 *      pktlen = the number of TRs in the packet
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param descType [IN]    Descriptor type
 *  \param pktLen   [IN]    Packet length
 *
 *  \return None
 */
static inline void CSL_udmapCppi5SetPktLen( void *pDesc, uint32_t descType, uint32_t pktLen )
{
    if( (descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST) ||
        (descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_MONO) )
    {
        CSL_FINS( ((CSL_UdmapCppi5HMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_PKTLEN, pktLen );
    }
    if( descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_TR )
    {
        CSL_FINS( ((CSL_UdmapCppi5TRPD *)pDesc)->descInfo, UDMAP_CPPI5_TRPD_DESCINFO_LASTIDX, pktLen-1U );
    }
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
static inline void CSL_udmapCppi5HostSetPktLen( void *pDesc, uint32_t pktLen )
{
    CSL_FINS( ((CSL_UdmapCppi5HMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_PKTLEN, pktLen );
}

/**
 *  \brief Set the packet length of mono mode packet descriptors
 *
 *  This function sets the packet length of mono mode packet descriptors.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param pktLen   [IN]    Packet length in bytes
 *
 *  \return None
 */
static inline void CSL_udmapCppi5MonoSetPktLen( void *pDesc, uint32_t pktLen )
{
    CSL_FINS( ((CSL_UdmapCppi5MMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_PKTLEN, pktLen );
}

/**
 *  \brief Set the packet length of TR mode packet descriptors
 *
 *  This function sets the packet length of TR mode packet descriptors.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param pktLen   [IN]    Number of TRs in the packet
 *
 *  \return None
 */
static inline void CSL_udmapCppi5TrSetPktLen( void *pDesc, uint32_t pktLen )
{
    CSL_FINS( ((CSL_UdmapCppi5TRPD *)pDesc)->descInfo, UDMAP_CPPI5_TRPD_DESCINFO_LASTIDX, pktLen-1U );
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
static inline uint64_t CSL_udmapCppi5GetBufferAddr( const CSL_UdmapCppi5HMPD *pDesc )
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
static inline uint32_t CSL_udmapCppi5GetBufferLen( const CSL_UdmapCppi5HMPD *pDesc )
{
    return (uint32_t)CSL_FEXT( pDesc->bufInfo1, UDMAP_CPPI5_PD_BUFINFO1_LEN );
}

/**
 *  \brief Get the original buffer length
 *
 *  This function returns the original length of the buffer (in bytes) attached to the
 *  specified descriptor.
 *
 *  This function is intended for use with host-mode descriptors only.
 *
 *  \param pDesc            [IN]    Pointer to the host-mode descriptor
 *
 *  \return Length of the buffer in bytes
 */
static inline uint32_t CSL_udmapCppi5GetOrgBufferLen( const CSL_UdmapCppi5HMPD *pDesc )
{
    return (uint32_t)CSL_FEXT( pDesc->orgBufLen, UDMAP_CPPI5_PD_ORGBUFLEN_LEN );
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
static inline void CSL_udmapCppi5SetBufferAddr( CSL_UdmapCppi5HMPD *pDesc, uint64_t physBufferAddr )
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
static inline void CSL_udmapCppi5SetOrgBufferAddr( CSL_UdmapCppi5HMPD *pDesc, uint64_t physBufferAddr )
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
static inline void CSL_udmapCppi5SetBufferLen( CSL_UdmapCppi5HMPD *pDesc, uint32_t bufferLenBytes )
{
    CSL_FINS( pDesc->bufInfo1, UDMAP_CPPI5_PD_BUFINFO1_LEN, bufferLenBytes );
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
static inline void CSL_udmapCppi5SetOrgBufferLen( CSL_UdmapCppi5HMPD *pDesc, uint32_t bufferLenBytes )
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
static inline void CSL_udmapCppi5LinkDesc( CSL_UdmapCppi5HMPD *pDesc, uint64_t physBufferDescAddr )
{
    pDesc->nextDescPtr = physBufferDescAddr;
}

/**
 *  \brief Is the EPI block present in the descriptor?
 *
 *  This function returns true if the Extended Packet Info (EPI) block
 *  is present in the descriptor and false if it is not present.
 *
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return  true = EPI block is present in the descriptor
 *          false = EPI block is not present in the descriptor
 */
static inline bool CSL_udmapCppi5IsEpiDataPresent( const void *pDesc )
{
    uint32_t fieldVal;

    fieldVal = CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_EINFO );
    return (fieldVal == (uint32_t)0U) ? (bool)false : (bool)true;
}

/**
 *  \brief Set indicator if the EPI block is present in the descriptor
 *
 *  This function is used to set if the Extended Packet Info (EPI) block
 *  is present in the descriptor (true) or if it is not present (false).
 *
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
 *
 *  \param pDesc            [IN]    Pointer to the descriptor
 *  \param bEpiDataPresent  [IN]    true if EPI block is present, false if not
 *
 *  \return  None
 */
static inline void CSL_udmapCppi5SetEpiDataPresent( void *pDesc, bool bEpiDataPresent )
{
    CSL_FINS( ((CSL_UdmapCppi5HMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_EINFO, (bEpiDataPresent==(bool)true) ? (uint32_t)1U : (uint32_t)0U );
}

/**
 *  \brief Return pointer to EPI block data
 *
 *  This function returns a pointer to the Extended Packet Info (EPI) block
 *  data within the descriptor. If the EPI block is not present, NULL is
 *  returned.
 *
 *  This function only operates on host-mode and mono-mode descriptors.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return A pointer to the EPI block data within the descriptor is returned,
 *          or NULL is the EPI block is not present in the descriptor
 */
static inline uint32_t *CSL_udmapCppi5GetEpiDataPtr( const void *pDesc )
{
    uint32_t *pEpiData = NULL;

    if( CSL_udmapCppi5IsEpiDataPresent(pDesc) == (bool)true )
    {
        uint32_t descType = CSL_udmapCppi5GetDescType( pDesc );

        if( descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST )
        {
            pEpiData = (uint32_t *)((const uint8_t *)pDesc + sizeof(CSL_UdmapCppi5HMPD));
        }
        else
        {
            pEpiData = (uint32_t *)((const uint8_t *)pDesc + sizeof(CSL_UdmapCppi5MMPD));
        }
    }
    return pEpiData;
}

/**
 *  \brief Read the EPI block data
 *
 *  This function reads and returns the Extended Packet Info (EPI) block
 *  data within the descriptor.
 *
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
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
static inline int32_t CSL_udmapCppi5RdEpiData( const void *pDesc, uint32_t *pTsInfo, uint32_t *pSwInfo0, uint32_t *pSwInfo1, uint32_t *pSwInfo2 )
{
    int32_t retVal = -1;
    uint32_t *pSrcEpiData = CSL_udmapCppi5GetEpiDataPtr(pDesc);

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
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param tsInfo   [IN]    Timestamp information
 *  \param swInfo0  [IN]    Software information word 0
 *  \param swInfo1  [IN]    Software information word 1
 *  \param swInfo2  [IN]    Software information word 2
 *
 *  \return  None
 */
static inline void CSL_udmapCppi5WrEpiData( const void *pDesc, uint32_t tsInfo, uint32_t swInfo0, uint32_t swInfo1, uint32_t swInfo2 )
{
    uint32_t *pDstEpiData = CSL_udmapCppi5GetEpiDataPtr(pDesc);

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
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return 0 = Protocol-specific data is in the descriptor
 *          1 = Protocol-specific data is in the SOP buffer
 */
static inline uint32_t CSL_udmapCppi5GetPsDataLoc( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_PSINFO );
}

/**
 *  \brief Set the location of the protocol-specific data
 *
 *  This function sets the location of the protocol-specific data.
 *
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param psLoc    [IN]    0 = Protocol-specific data is in the descriptor
 *                          1 = Protocol-specific data is in the SOP buffer
 *
 *  \return None
 */
static inline void CSL_udmapCppi5SetPsDataLoc( void *pDesc, uint32_t psLoc )
{
    CSL_FINS( ((CSL_UdmapCppi5HMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_PSINFO, psLoc );
}

/**
 *  \brief Get the number of bytes of protocol-specific data
 *
 *  This function returns the number of bytes of protocol-specific (ps) data
 *  present in the descriptor. This value will be a multiple of 4.
 *
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return The number of bytes of ps data is returned
 */
static inline uint32_t CSL_udmapCppi5GetPsDataLen( const void *pDesc )
{
    return ((uint32_t)CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_PSWCNT )) * 4U;
}

/**
 *  \brief Set the number of bytes of protocol-specific data
 *
 *  This function sets the number of bytes of protocol-specific (ps) data
 *  present in the descriptor or SOP buffer. This value must be a multiple
 *  of 4.
 *
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
 *
 *  \param pDesc        [IN]    Pointer to the descriptor
 *  \param psDataLen    [IN]    Number of bytes of protocol-specific (ps) data
 *                              present in the descriptor or SOP buffer
 *
 *  \return None
 */
static inline void CSL_udmapCppi5SetPsDataLen( void *pDesc, uint32_t psDataLen )
{
    CSL_FINS( ((CSL_UdmapCppi5HMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_PSWCNT, (psDataLen/4U) );
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
 *  This function only operates on host-mode and mono-mode descriptors.
 *
 *  \param pDesc        [IN]    Pointer to the descriptor
 *  \param bInSopBuf    [IN]    true if ps data is in SOP buffer, false otherwise
 *  \param bEpiPresent  [IN]    true if EPI data is present, false otherwise
 *
 *  \return Address of the protocol-specific data
 */
static inline uint64_t CSL_udmapCppi5GetPsDataAddr( const void *pDesc, bool bInSopBuf, bool bEpiPresent )
{
    uint64_t psDataAddr;
    uint32_t descType = CSL_udmapCppi5GetDescType( pDesc );
    uint32_t epiDataSize;

    epiDataSize = (bEpiPresent == (bool)false) ? (uint32_t)0U : (uint32_t)sizeof(CSL_UdmapCppi5Epi);
    if( descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST )
    {
        if( bInSopBuf == (bool)true )
        {
            /* psData is in SOP buffer immediately prior to the data */
            psDataAddr = (uint64_t)((uintptr_t)(((const CSL_UdmapCppi5HMPD *)pDesc)->bufPtr));
        }
        else
        {
            /* psData is in the descriptor */
            psDataAddr = (uint64_t)(uintptr_t)((const uint8_t *)pDesc + sizeof(CSL_UdmapCppi5HMPD) + epiDataSize);
        }
    }
    else
    {
        /* Get pointer to psData in MONO descriptor */
        psDataAddr = (uint64_t)(uintptr_t)((const uint8_t *)pDesc + sizeof(CSL_UdmapCppi5MMPD) + epiDataSize);

    }
    return psDataAddr;
}

static inline uint8_t *CSL_udmapCppi5GetPsDataPtr( const void *pDesc )
{
    uint8_t *pPsData = NULL;
    uint32_t psDataLen;

    psDataLen = CSL_udmapCppi5GetPsDataLen( pDesc );
    if( psDataLen != (uint32_t)0U )
    {
        uint32_t descType = CSL_udmapCppi5GetDescType( pDesc );
        uint32_t epiDataSize;

        epiDataSize = (CSL_udmapCppi5IsEpiDataPresent( pDesc ) == (bool)false) ? (uint32_t)0U : (uint32_t)sizeof(CSL_UdmapCppi5Epi);
        if( descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST )
        {
            uint32_t psLoc = CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->descInfo, UDMAP_CPPI5_PD_DESCINFO_PSINFO );
            if( psLoc == CSL_UDMAP_CPPI5_PD_DESCINFO_PSINFO_VAL_IN_DESC )
            {
                /* psData is in the descriptor */
                pPsData = (uint8_t *)((const uint8_t *)pDesc + sizeof(CSL_UdmapCppi5HMPD) + epiDataSize);
            }
            else
            {
                /* psData is in SOP buffer immediately prior to the data */
                pPsData = (uint8_t *)(uintptr_t)(((const CSL_UdmapCppi5HMPD *)pDesc)->bufPtr);
            }
        }
        else
        {
            /* Get pointer to psData in MONO descriptor */
            pPsData = (uint8_t *)((const uint8_t *)pDesc + sizeof(CSL_UdmapCppi5MMPD) + epiDataSize);
        }
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
static inline uint32_t CSL_udmapCppi5GetSrcTag( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->srcDstTag, UDMAP_CPPI5_PD_SRCDSTTAG_SRCTAG );
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
static inline uint32_t CSL_udmapCppi5GetDstTag( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->srcDstTag, UDMAP_CPPI5_PD_SRCDSTTAG_DSTTAG );
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
static inline void CSL_udmapCppi5SetSrcTag( void *pDesc, uint32_t srcTag )
{
    CSL_FINS( ((CSL_UdmapCppi5HMPD *)pDesc)->srcDstTag, UDMAP_CPPI5_PD_SRCDSTTAG_SRCTAG, srcTag );
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
static inline void CSL_udmapCppi5SetDstTag( void *pDesc, uint32_t dstTag )
{
    CSL_FINS( ((CSL_UdmapCppi5HMPD *)pDesc)->srcDstTag, UDMAP_CPPI5_PD_SRCDSTTAG_DSTTAG, dstTag );
}

/**
 *  \brief Get error flags
 *
 *  This function gets the error flags from the descriptor.
 *
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return  The error flags are returned
 */
static inline uint32_t CSL_udmapCppi5GetErrorFlags( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->pktInfo1, UDMAP_CPPI5_PD_PKTINFO1_PKTERROR );
}

/**
 *  \brief Get ps flags
 *
 *  This function gets the protocol-specific (ps) flags from the descriptor.
 *
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return  The protocol-specific (ps) flags are returned
 */
static inline uint32_t CSL_udmapCppi5GetPsFlags( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->pktInfo1, UDMAP_CPPI5_PD_PKTINFO1_PSFLGS );
}

/**
 *  \brief Set ps flags
 *
 *  This function sets the protocol-specific (ps) flags in the descriptor.
 *
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param psFlags  [IN]    ps flags
 *
 *  \return  None
 */
static inline void CSL_udmapCppi5SetPsFlags( void *pDesc, uint32_t psFlags )
{
    CSL_FINS( ((CSL_UdmapCppi5HMPD *)pDesc)->pktInfo1, UDMAP_CPPI5_PD_PKTINFO1_PSFLGS, psFlags );
}

/**
 *  \brief Get packet type
 *
 *  This function gets the packet type from the descriptor.
 *
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *
 *  \return  Packet type
 */
static inline uint32_t CSL_udmapCppi5GetPktType( const void *pDesc )
{
    return (uint32_t)CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->pktInfo2, UDMAP_CPPI5_PD_PKTINFO2_PKTTYPE );
}

/**
 *  \brief Set packet type
 *
 *  This function sets the packet type in the descriptor.
 *
 *  This function is intended for use with host-mode and mono-mode
 *  descriptors only.
 *
 *  \param pDesc    [IN]    Pointer to the descriptor
 *  \param pktType  [IN]    Packet type
 *
 *  \return  None
 */
static inline void CSL_udmapCppi5SetPktType( void *pDesc, uint32_t pktType  )
{
    CSL_FINS( ((CSL_UdmapCppi5HMPD *)pDesc)->pktInfo2, UDMAP_CPPI5_PD_PKTINFO2_PKTTYPE, pktType );
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
static inline void CSL_udmapCppi5GetIds( const void *pDesc, uint32_t *pPktId, uint32_t *pFlowId )
{
    uint32_t descType = CSL_udmapCppi5GetDescType( pDesc );

    if( descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_TR )
    {
        *pPktId     = CSL_FEXT( ((const CSL_UdmapCppi5TRPD *)pDesc)->pktInfo, UDMAP_CPPI5_TRPD_PKTINFO_PKTID );
        *pFlowId    = CSL_FEXT( ((const CSL_UdmapCppi5TRPD *)pDesc)->pktInfo, UDMAP_CPPI5_TRPD_PKTINFO_FLOWID );
    }
    else
    {
        *pPktId     = CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->pktInfo1, UDMAP_CPPI5_PD_PKTINFO1_PKTID );
        *pFlowId    = CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->pktInfo1, UDMAP_CPPI5_PD_PKTINFO1_FLOWID );
    }
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
static inline void CSL_udmapCppi5SetIds( void *pDesc, uint32_t descType, uint32_t pktId, uint32_t flowId )
{
    uint32_t v;

    if( descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_TR )
    {
        v = ((CSL_UdmapCppi5TRPD *)pDesc)->pktInfo;
        v &= ~(CSL_UDMAP_CPPI5_TRPD_PKTINFO_PKTID_MASK | CSL_UDMAP_CPPI5_TRPD_PKTINFO_FLOWID_MASK);
        v |= CSL_FMK( UDMAP_CPPI5_TRPD_PKTINFO_PKTID, pktId )   |
             CSL_FMK( UDMAP_CPPI5_TRPD_PKTINFO_FLOWID, flowId );
        ((CSL_UdmapCppi5TRPD *)pDesc)->pktInfo = v;
    }
    else
    {
        v = ((CSL_UdmapCppi5HMPD *)pDesc)->pktInfo1;
        v &= ~(CSL_UDMAP_CPPI5_PD_PKTINFO1_PKTID_MASK | CSL_UDMAP_CPPI5_PD_PKTINFO1_FLOWID_MASK);
        v |= CSL_FMK( UDMAP_CPPI5_PD_PKTINFO1_PKTID, pktId )   |
             CSL_FMK( UDMAP_CPPI5_PD_PKTINFO1_FLOWID, flowId );
        ((CSL_UdmapCppi5HMPD *)pDesc)->pktInfo1 = v;
    }
}

/**
 *  \brief Get descriptor return policy information
 *
 *  This function gets descriptor return policy information from the
 *  descriptor. See #CSL_udmapCppi5SetReturnPolicy for a description of the
 *  return policy information.
 *
 *  \param pDesc          [IN]    Pointer to the descriptor
 *  \param pRetPolicy     [OUT]   Pointer where to store return policy (0 is
 *                                returned for MONO and TR descriptor types)
 *  \param pEarlyReturn   [OUT]   Pointer where to store early return (0 is
 *                                returned for TR descriptor type)
 *  \param pRetPushPolicy [OUT]   Pointer where to store return push policy
 *  \param pRetQnum       [OUT]   Pointer where to store return queue #
 *
 *  \return  None
 */
static inline void CSL_udmapCppi5GetReturnPolicy( const void *pDesc, uint32_t *pRetPolicy, uint32_t *pEarlyReturn, uint32_t *pRetPushPolicy, uint32_t *pRetQnum )
{
    uint32_t descType = CSL_udmapCppi5GetDescType( pDesc );

    if( descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_TR )
    {
        *pRetPolicy     = 0;
        *pEarlyReturn   = 0;
        *pRetPushPolicy = CSL_FEXT( ((const CSL_UdmapCppi5TRPD *)pDesc)->retInfo, UDMAP_CPPI5_TRPD_RETINFO_RETPOLICY );
        *pRetQnum       = CSL_FEXT( ((const CSL_UdmapCppi5TRPD *)pDesc)->retInfo, UDMAP_CPPI5_TRPD_RETINFO_RETQ );
    }
    else
    {
        if( descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_MONO )
        {
            *pRetPolicy     = 0;
        }
        else
        {
            *pRetPolicy     = CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->pktInfo2, UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY );
        }
        *pEarlyReturn   = CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->pktInfo2, UDMAP_CPPI5_PD_PKTINFO2_EARLYRET );
        *pRetPushPolicy = CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->pktInfo2, UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY );
        *pRetQnum       = CSL_FEXT( ((const CSL_UdmapCppi5HMPD *)pDesc)->pktInfo2, UDMAP_CPPI5_PD_PKTINFO2_RETQ );
    }
}

/**
 *  \brief Set descriptor return policy information
 *
 *  This function sets descriptor return policy information in the
 *  descriptor. This includes the following:
 *
 *  return policy (HOST descriptor type only)
 *      0 = Entire packet (still linked together) should be returned to queue
 *          specified by pRetQnum
 *      1 = Each descriptor should be returned to queue specified by pRetQnum
 *          in their respective descriptors.
 *
 *  early return (HOST and MONO descriptor types only)
 *      0 = Buffer/packet descriptor pointers should only be returned after
 *          all reads have been completed
 *      1 = Buffer/packet descriptor pointers should be returned immediately
 *          upon fetching the descriptor and beginning to transfer data
 *
 *  return push policy (only used when return policy is 1)
 *      0 = Descriptor is returned to tail of queue
 *      1 = Descriptor is returned to head of queue
 *
 *  return queue #
 *      Specifies the queue / ring number that the descriptor is returned to
 *      after transmission is complete.
 *
 *  \param pDesc         [IN]   Pointer to the descriptor
 *  \param descType      [IN]   Descriptor type
 *  \param retPolicy     [IN]   return policy (only used for HOST
 *                              descriptor type)
 *  \param earlyReturn   [IN]   early return (only used for HOST and MONO
 *                              descriptor types)
 *  \param retPushPolicy [IN]   return push policy
 *  \param retQnum       [IN]   return queue #
 *
 *  \return  None
 */
static inline void CSL_udmapCppi5SetReturnPolicy( void *pDesc, uint32_t descType, uint32_t retPolicy, uint32_t earlyReturn, uint32_t retPushPolicy, uint32_t retQnum )
{
    uint32_t v;

    if( descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_TR )
    {
        ((CSL_UdmapCppi5TRPD *)pDesc)->retInfo =
            CSL_FMK( UDMAP_CPPI5_TRPD_RETINFO_RETPOLICY, retPushPolicy )    |
            CSL_FMK( UDMAP_CPPI5_TRPD_RETINFO_RETQ, retQnum ) ;
    }
    else
    {
        v = ((CSL_UdmapCppi5HMPD *)pDesc)->pktInfo2;
        if( descType == CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_MONO )
        {
            v &= (CSL_UDMAP_CPPI5_PD_PKTINFO2_PKTTYPE_MASK | CSL_UDMAP_CPPI5_PD_PKTINFO2_DATA_OFFSET_MASK);
        }
        else
        {
            v &= CSL_UDMAP_CPPI5_PD_PKTINFO2_PKTTYPE_MASK;
            v |= CSL_FMK( UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY, retPolicy );

        }
        v |=    CSL_FMK( UDMAP_CPPI5_PD_PKTINFO2_EARLYRET, earlyReturn)         |
                CSL_FMK( UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY, retPushPolicy ) |
                CSL_FMK( UDMAP_CPPI5_PD_PKTINFO2_RETQ, retQnum );
        ((CSL_UdmapCppi5HMPD *)pDesc)->pktInfo2 = v;
    }
}

/**
 *  \brief Get MONO descriptor data offset
 *
 *  This function returns the data offset for a monolithic-mode (MONO) type of
 *  descriptor. This value indicates the byte offset from byte 0 of the
 *  descriptor to the location where the valid data begins.
 *
 *  \param pDesc    [IN]    Pointer to the MONO descriptor
 *
 *  \return Byte offset from start of descriptor to valid data
 */
static inline uint32_t CSL_udmapCppi5MonoGetDataOffset( const CSL_UdmapCppi5MMPD *pDesc )
{
    return (uint32_t)CSL_FEXT( pDesc->pktInfo2, UDMAP_CPPI5_PD_PKTINFO2_DATA_OFFSET );
}

/**
 *  \brief Set MONO descriptor data offset
 *
 *  This function sets the data offset for a monolithic-mode (MONO) type of
 *  descriptor. This value indicates the byte offset from byte 0 of the
 *  descriptor to the location where the valid data begins.
 *
 *  \param pDesc        [IN]    Pointer to the MONO descriptor
 *  \param dataOffset   [IN]    Byte offset
 *
 *  \return None
 */
static inline void CSL_udmapCppi5MonoSetDataOffset( CSL_UdmapCppi5MMPD *pDesc, uint32_t dataOffset )
{
    CSL_FINS( ((CSL_UdmapCppi5MMPD *)pDesc)->pktInfo2, UDMAP_CPPI5_PD_PKTINFO2_DATA_OFFSET, dataOffset );
}

/**
 *  \brief Get TR descriptor reload information
 *
 *  This function returns reload information for a transfer-request (TR) type
 *  of descriptor. See #CSL_udmapCppi5TrSetReload for a description of this
 *  reload information.
 *
 *  \param pDesc         [IN]   Pointer to the TR descriptor
 *  \param pReloadEnable [OUT]  Pointer where to store reload enable
 *  \param pReloadIdx    [OUT]  Pointer where to store reload index
 *
 *  \return None
 */
static inline void CSL_udmapCppi5TrGetReload( const CSL_UdmapCppi5TRPD *pDesc, uint32_t *pReloadEnable, uint32_t *pReloadIdx )
{
    *pReloadEnable = (uint32_t)CSL_FEXT( pDesc->descInfo, UDMAP_CPPI5_TRPD_DESCINFO_RELOAD );
    *pReloadIdx    = (uint32_t)CSL_FEXT( pDesc->descInfo, UDMAP_CPPI5_TRPD_DESCINFO_RLDIDX );
}

/**
 *  \brief Set TR descriptor reload information
 *
 *  This function sets reload information for a transfer-request (TR) type
 *  of descriptor. Reload information includes:
 *
 *  reload enable (specifies what to do when the last entry is processed in
 *  this packet)
 *      0 = Finish the packet and place the descriptor back on the return queue
 *      1 = Vector to the Reload Index and resume processing
 *
 *  reload index
 *      Specifies the value to set the current processing index to when the
 *      last entry is processed and reload enable is set to 1.  This is
 *      basically an absolute index to jump to on the 2nd and following passes
 *      through the TR packet.
 *
 *  \param pDesc        [IN]    Pointer to the TR descriptor
 *  \param reloadEnable [IN]    reload enable
 *  \param reloadIdx    [IN]    reload index
 *
 *  \return None
 */
static inline void CSL_udmapCppi5TrSetReload( CSL_UdmapCppi5TRPD *pDesc, uint32_t reloadEnable, uint32_t reloadIdx )
{
    CSL_FINS( pDesc->descInfo, UDMAP_CPPI5_TRPD_DESCINFO_RELOAD, reloadEnable );
    CSL_FINS( pDesc->descInfo, UDMAP_CPPI5_TRPD_DESCINFO_RLDIDX, reloadIdx );
}

/**
 *  \brief Get TR descriptor entry stride
 *
 *  This function returns the stride between TR entries. See
 *  #CSL_udmapCppi5TrSetEntryStride for valid stride values.
 *
 *  \param pDesc    [IN]    Pointer to the TR descriptor
 *
 *  \return Encoded TR entry stride
 */
static inline uint32_t CSL_udmapCppi5TrGetEntryStride( const CSL_UdmapCppi5TRPD *pDesc )
{
    return (uint32_t)CSL_FEXT( pDesc->pktInfo, UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE );
}

/**
 *  \brief Set TR descriptor entry stride
 *
 *  This function sets the stride between TR entries. This value must be set
 *  large enough that any TR in the buffer will fit within the given dimension.
 *  TRs are expected to be placed on boundaries as given in this dimension.
 *
 *  The stride value is encoded as follows:
 *      0 = 16 byte TR entry
 *      1 = 32-byte TR entry
 *      2 = 64-byte TR entry
 *      3 = 128-byte TR entry
 *
 *  \param pDesc        [IN]    Pointer to the TR descriptor
 *  \param entryStride  [IN]    Encoded TR entry stride
 *
 *  \return None
 */
static inline void CSL_udmapCppi5TrSetEntryStride( CSL_UdmapCppi5TRPD *pDesc, uint32_t entryStride )
{
    CSL_FINS( pDesc->pktInfo, UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE, entryStride );
}

/* @} */

#ifdef __cplusplus
}
#endif  // extern "C"

#endif
/** @} */

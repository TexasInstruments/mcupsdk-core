/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 */

/**
 *  \file enet_appmemutils.h
 *
 *  \brief Enet DMA memutils header file.
 */

#ifndef ENET_MEM_H_
#define ENET_MEM_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/core/enet_dma.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
#define ENET_MEM_RING_MAX_ELEM_CNT          (256U)
#define ENET_MEM_RING_MAX_SIZE   (ENET_UDMA_RING_MEM_SIZE * ENET_MEM_RING_MAX_ELEM_CNT)

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
#define ENET_MEM_NUM_RINGS_TYPES (2U)
#elif (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
#define ENET_MEM_NUM_RINGS_TYPES (1U)
#else
#error "Unknown Udma Type"
#endif

#endif /* defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA) */


#define ENET_MEM_NUM_ALLOC_POOLS (3U)
#define ENET_MEM_POOLIDX_SMALL   (0U)
#define ENET_MEM_POOLIDX_MEDIUM  (1U)
#define ENET_MEM_POOLIDX_LARGE   (2U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
/**
 *  \brief
 */
typedef struct EnetMem_DmaDescMemPoolEntry_s
{
    /*! DMA descriptor element */
    EnetUdma_DmaDesc dmaDesc
    __attribute__ ((aligned(ENETDMA_CACHELINE_ALIGNMENT)));

}EnetMem_DmaDescMemPoolEntry;

typedef struct EnetMem_DmaDescMem_s
{
    /*! The node element so this packet can be added to a queue
     * Note- Keep EnetQ_Node as first member always as driver uses generic Q functions
     *       and deferences to this member */
    EnetQ_Node node;

    /*! DMA descriptor element */
    EnetUdma_DmaDesc *dmaDescPtr;

    /*! DMA descriptor state, refer to CpswUtils_DescStateMemMgr */
    uint32_t dmaDescState;
}EnetMem_DmaDescMem;

/**
 *  \brief
 */
typedef struct EnetMem_RingMemPoolEntry_s
{
    /*! Ring memory element */
    uint8_t ringEle[ENET_UTILS_ALIGN(ENET_MEM_RING_MAX_SIZE, UDMA_CACHELINE_ALIGNMENT)]
    __attribute__ ((aligned(UDMA_CACHELINE_ALIGNMENT)));
}EnetMem_RingMemPoolEntry;

/**
 *  \brief
 */
typedef struct EnetMem_RingMem_s
{
    /*! The node element so this packet can be added to a queue
     * Note- Keep EnetQ_Node as first member always as driver uses generic Q functions
     *       and deferences to this member */
    EnetQ_Node node;

    /*! Ring memory element */
    uint8_t * ringElePtr;
}EnetMem_RingMem;

/**
 *  \brief
 */
typedef struct EnetMem_DmaDescPoolCfg_s
{
    uint32_t numDesc;
    EnetMem_DmaDescMemPoolEntry *descMemBase;
    uint32_t                     descMemSize;
    EnetMem_DmaDescMem           *descInfoContainerMemBase;
    uint32_t                     descInfoContainerMemSize;
} EnetMem_DmaDescPoolCfg;

/**
 *  \brief
 */
typedef struct EnetMem_RingPoolCfg_s
{
    uint32_t numRings;
    EnetMem_RingMemPoolEntry *ringMemBase;
    uint32_t                  ringMemSize;
    EnetMem_RingMem          *ringInfoContainerBase;
    uint32_t                  ringInfoContainerSize;
} EnetMem_RingPoolCfg;
#endif

/**
 *  \brief Memutils packet info container
 */
typedef struct EnetMem_EthPktMem_s
{
    /*! The node element so this packet can be added to a queue
     * Note- Keep EnetQ_Node as first member always as driver uses generic Q functions
     *       and deferences to this member */
    EnetQ_Node node;

    /*! Eth packet info structure - shared with driver */
    EnetDma_Pkt *dmaPktPtr;

    /*! Original packet size - we can't use this info from dmaPkt as app can change it */
    uint32_t orgBufSize;

    /*! Original packet size - we can't use this info from dmaPkt as app can change it */
    uint8_t *orgBufAddr;
}EnetMem_AppPktInfoMem;


/**
 *  \brief
 */
typedef struct EnetMem_PktPoolCfg_s
{
    uint32_t pktSize;
    uint32_t numPkts;
    EnetDma_Pkt *pktInfoMem;
    uint32_t     pktInfoSize;
    uint8_t     *pktBufMem;
    uint32_t     pktBufSize;
    EnetMem_AppPktInfoMem *pktInfoContainerMem;
    uint32_t     pktInfoContainerSize;
} EnetMem_PktPoolCfg;

typedef struct EnetMem_PktInfoMemCfg_s
{
    uint32_t     numPkts;
    EnetDma_Pkt *pktInfoMem;
    uint32_t     pktInfoSize;
} EnetMem_PktInfoMemCfg;

/**
 *  \brief
 */
typedef struct EnetMem_Cfg_s
{
    EnetMem_PktPoolCfg     pktBufPool[ENET_MEM_NUM_ALLOC_POOLS];
    EnetMem_PktInfoMemCfg  pktInfoMemPool;
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
    EnetMem_RingPoolCfg    ringMem;
    EnetMem_DmaDescPoolCfg dmaDescMem;
#endif
} EnetMem_Cfg;


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
/*! Ring memory allocation function  */
uint8_t *EnetMem_allocRingMem(void *appPriv,
                              uint32_t numRingEle,
                              uint32_t alignSize);

/*! Ring memory free function  */
void EnetMem_freeRingMem(void *appPriv,
                         void *ringMemPtr,
                         uint32_t numRingEle);

/*! DMA packet allocation function  */
EnetUdma_DmaDesc *EnetMem_allocDmaDesc(void *appPriv,
                                       uint32_t alignSize);

/*! DMA packet free function  */
void EnetMem_freeDmaDesc(void *appPriv,
                         EnetUdma_DmaDesc *dmaDescPtr);

#endif
/*!
 * \brief Allocate Ethernet Packet
 *
 * This function gets a buffer of size greater than or equal to the total buffer size (sum of entries in scatterSegmentSize)
 * And fills the sglist as per the segment sizes in scatterSegmentSize[]. LC-DMA Rx scatter-gather needs the buffer pointers
 * to be pre-linked. In that case, these numScatterSegments,scatterSegmentSize[] would be useful to get a Eth Pkt initialized
 * with multiple buffer pointers.
 *
 * \param      [IN] appPriv - appPriv of the Eth Packet
 * \param      [IN] alignSize - Alignment of the buffer pointers
 * \param      [IN] numScatterSegments - Number of scatter-segments
 * \param      [IN] scatterSegmentSize[] - Array with sizes of each scattered segment
 *
 * \return EnetDma_Pkt * - Pointer to the EnetDma_Pkt with the buffer pointers in sglist filled.
 */
EnetDma_Pkt *EnetMem_allocEthPkt(void *appPriv,
                                 uint32_t alignSize,
                                 uint32_t numScatterSegments,
                                 uint32_t scatterSegmentSize[]);

/*! Ethernet packetInfoMem allocation function  */
EnetDma_Pkt *EnetMem_allocEthPktInfoMem(void *appPriv,
                                        uint32_t alignSize);

/*! Ethernet packet free function  */
void EnetMem_freeEthPkt(EnetDma_Pkt *pPktInfo);

/*! Initialize Cpsw memutils module
 * Note - This function should be called after Cpsw is opened as it uses CpswUtils_Q
 * functions */
int32_t EnetMem_init(void);

/*! Mem utils deinit  */
void EnetMem_deInit(void);

/*! Application implemented function to get the memory pool configuration */
const EnetMem_Cfg * EnetMem_getCfg(void);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENET_MEM_H_ */

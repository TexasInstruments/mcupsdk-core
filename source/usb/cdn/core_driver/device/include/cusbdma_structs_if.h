/* parasoft suppress item  MISRA2012-DIR-4_8 "Consider hiding implementation of structure, DRV-4932" */
/**********************************************************************
* Copyright (C) 2012-2021 Cadence Design Systems, Inc.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* WARNING: This file is auto-generated using api-generator utility.
*          api-generator: 13.05.b3ee589
*          Do not edit it manually.
**********************************************************************
* Layer interface for the Cadence DMA controller
**********************************************************************/
#ifndef CUSBDMA_STRUCTS_IF_H
#define CUSBDMA_STRUCTS_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "cdn_stdtypes.h"
#include "cusbdma_if.h"

/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
* Structures and unions
**********************************************************************/
/** Describes One trb */
struct CUSBDMA_DmaTrb_s
{
    /** DMA address */
    uint32_t dmaAddr;
    /** DMA size */
    uint32_t dmaSize;
    /** DMA control */
    uint32_t ctrl;
};

/** Structure keeps endpoints resources pointers. */
struct CUSBDMA_MemResources_s
{
    /** Virtual address of transfer ring base */
    CUSBDMA_DmaTrb* trbAddr;
    /** Size of the TRB buffer */
    uint32_t trbBufferSize;
    /** Physical address of transfer ring */
    uintptr_t trbDmaAddr;
    /** Physical address extension to 48 bits, this variable is a 16 bit width page index */
    uint16_t memPageIndex;
};

/** Configuration of DMA.  Object of this type is used for probe and init functions when checking memory requirements of device object and for hardware controller configuration.  Size of device object in bytes is returned on probe function call in CUSBDMA_SysReq.privDataSize field. */
struct CUSBDMA_Config_s
{
    /** Base address of DMA special function register. */
    uintptr_t regBase;
    /** Field holds information about DMA mode.  DMA mode can be programmed globally for all channels or individually for particular channels.  Each bit of dmaModeTx field is responsible for one DMA channel. If this field will be set to 0xFFFF or 0xFFFF for both dmaModeTx and dmaModeRx then DMA mode will be programmed globally, elsewhere each DMA channel will be programmed individually in channelAlloc function. Meanings of bits: 0 DMA Single Mode, 1 DMA DMULT Mode. */
    uint16_t dmaModeTx;
    /** Field holds information about DMA mode.  DMA mode can be programmed globally for all channels or individually for particular channels.  Each bit of dmaModeTx field is responsible for one DMA channel. If this field will be set to 0xFFFF or 0xFFFF for both dmaModeTx and dmaModeRx then DMA mode will be programmed globally, elsewhere each DMA channel will be programmed individually in channelAlloc function. Meanings of bits: 0 DMA Single Modem, 1 DMA DMULT Mode. */
    uint16_t dmaModeRx;
    /** Pointer to array containing endpoints resources addresses */
    CUSBDMA_MemResources (*epMemRes)[32];
};

/** System requirements returned by probe. */
struct CUSBDMA_SysReq_s
{
    /** Size of memory required for driver's private data. */
    uint32_t privDataSize;
    /** Size of memory required for USB Transfer Request Descriptors.  This memory will be used by DMA controller, so it should be suitable for DMA operation. */
    uint32_t trbMemSize;
};

/** Channel parameters used during channel allocation. */
struct CUSBDMA_ChannelParams_s
{
    /** direction for allocated channel */
    uint8_t isDirTx;
    /** USB endpoint number for allocated channel */
    uint8_t hwEpNum;
    /** Max packet size for this channel */
    uint16_t wMaxPacketSize;
    /** endpoint configuration for this channel - EP_CFG */
    uint32_t epConfig;
    /** IRQ settings for this channel - EP_STS_EN */
    uint32_t epIrqConfig;
};

/** Structure describing TRB Chain corresponding to a request */
struct CUSBDMA_DmaTrbChainDesc_s
{
    /** Length */
    uint32_t len;
    /** Actual length */
    uint32_t actualLen;
    /** index of oldest element  */
    uint16_t start;
    /** Index of the newest element  */
    uint16_t end;
    /** Indicates that a short packet was received for this chain */
    uint8_t shortPacketFlag;
    /** Indicates the state of this TRB descriptor chain */
    uint8_t trbChainState;
    /** Indicates whether the request for the transfer has been queued by cusbd */
    uint8_t requestQueued;
    /** Indicates whether the buffer for the transfer is held in aux-buffer */
    uint8_t requestOverflowed;
} __attribute__((packed));

/** DMA Channel structure */
struct CUSBDMA_DmaChannel_s
{
    /** Pointer to DMA controller */
    CUSBDMA_DmaController* controller;
    /** Max packet size for this channel */
    uint16_t wMaxPacketSize;
    /** USB EP number. It can be different then channel number  */
    uint8_t hwUsbEppNum;
    /** Flag indicating if transfer direction is transmit */
    uint8_t isDirTx;
    /** max TD length */
    uint32_t maxTdLen;
    /** max TRB length */
    uint32_t maxTrbLen;
    /** status */
    CUSBDMA_Status status;
    /** for private use */
    void* priv;
    /** field holds dmult configuration for channel */
    uint8_t dmultEnabled;
    /** Flag indicated whether this channel is stalled */
    uint8_t channelStalled;
    /** Pointer to the TRB chain desc list head for this channel */
    CUSBDMA_DmaTrbChainDesc* trbChainDescListHead;
    /** EP_CFG register for this channel */
    uint32_t epConfig;
    /** Stream ID of the current stream */
    uint32_t currentStreamID;
    /** Cycle bit for the next write TRB */
    uint32_t trbCycleBit;
    /** Virtual address of transfer ring base */
    CUSBDMA_DmaTrb* trbBufferBaseAddr;
    /** Physical address of transfer ring base */
    uintptr_t trbBufferBaseAddrPhy;
    /** Size of the TRB buffer */
    uint32_t trbBufferSize;
    /** Index where the next TRB will be written by SW */
    uint32_t trbBufferEnqueueIdx;
    /** Last known Index where the next TRB would be read by HW. */
    uint32_t trbBufferDequeueIdx;
    /** Index where the TRB chain is intentionally De-linked in DMULT mode */
    int32_t trbBufferDmultDelinkIdx;
    /** Size of the TRB chain descriptor buffer size */
    uint32_t trbChainDescBufferSz;
    /** Index where the next TRB would be written by SW */
    uint32_t trbChainDescWriteIdx;
    /** Index where the next TRB would be read by HW */
    uint32_t trbChainDescReadIdx;
};

/** DMA controller structure */
struct CUSBDMA_DmaController_s
{
    /** Pointer to the Device registers */
    DmaRegs* regs;
    /** DMA configuration */
    CUSBDMA_Config cfg;
    /** DMA Receive channels */
    CUSBDMA_DmaChannel rx[CUSBDMA_MAX_DMA_CHANNELS];
    /** DMA transmit channels */
    CUSBDMA_DmaChannel tx[CUSBDMA_MAX_DMA_CHANNELS];
    /** DMA TRB chain descriptor */
    CUSBDMA_DmaTrbChainDesc trbChainDesc[128];
    /** Pointer to array containing endpoints resources addresses */
    CUSBDMA_MemResources (*epMemRes)[32];
};

/** DMA Transfer Param structure */
struct CUSBDMA_DmaTransferParam_s
{
    /** DMA address */
    uintptr_t dmaAddr;
    /** length */
    uint32_t len;
    /** streamID */
    uint16_t sid;
    /** Indicated whether the request for the transfer has been queued by cusbd */
    uint8_t requestQueued;
    /** Indicates whether the buffer for the transfer is held in aux-buffer */
    uint8_t requestOverflowed;
    /** Is zlp enabled? */
    uint8_t zlp;
};

/**
 *  @}
 */


#ifdef __cplusplus
}
#endif

#endif  /* CUSBDMA_STRUCTS_IF_H */

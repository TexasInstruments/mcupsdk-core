/******************************************************************************
*
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
*
******************************************************************************
* cusbdma.c
* DMA Driver for USB Controllers
*
******************************************************************************/

#include "cdn_stdtypes.h"
#include "cdn_log.h"
#include <string.h>
#include "cdn_errno.h"
#include "cusbdma_if.h"
#include "cusbdma_structs_if.h"
#include "sgdma_regs.h"
#include "byteorder.h"
#include "cps.h"
#include "cusbdma_sanity.h"

#define DBG_DMA_BASIC_MSG           0x00000100U
#define DBG_DMA_VERBOSE_MSG         0x00000200U
#define DBG_DMA_ERR_MSG             0x00000400U
#define DBG_DMA_CHANNEL_USEGE_MSG   0x00001000U

#if !(defined CUSBDMA_DEFAULT_TIMEOUT)
#define CUSBDMA_DEFAULT_TIMEOUT 1000000U
#endif

/******************************************************************************
 * Forward declaration of local static functions
 *****************************************************************************/

/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
static void channelUpdateState(const CUSBDMA_DmaController *pD, CUSBDMA_DmaChannel *channel);

/******************************************************************************
 * Local utility functions
 *****************************************************************************/
/**
 * Wait for bit cleared
 * @param reg
 * @param bit
 * @param timeout
 * @return CDN_EOK on success, error otherwise
 */
static uint32_t waitForBitCleared(uint32_t * reg, uint32_t bit, uint32_t timeout) {
    uint32_t counter = timeout;
    uint32_t ret = CDN_EOK;
    /* Check for specified bit to be set */
    while ((CPS_UncachedRead32(reg) & bit) > 0U) {
        /* break loop if timeout occur */
        if (counter == 0U) {
            ret = CDN_ETIMEDOUT;
            vDbgMsg(DBG_DMA_ERR_MSG, DBG_CRIT, "%s() Error timeout \n", __func__);
            break;
        }
        counter--;
        CPS_DelayNs(1000);
    }
    return ret;
}

/**
 * Build normal TRB with interrupt on completion disabled
 * @param trb pointer to memory where TRB is allocated
 * @param data_ptr pointer to data buffer
 * @param data_size data buffer size
 * @param stream_id stream ID
 */
static inline void BUILD_NORMAL_TRB_NO_IOC(CUSBDMA_DmaTrb * trb, uint32_t data_ptr, uint32_t data_size, uint16_t stream_id, uint32_t cycleBit) {
    trb->dmaAddr = cpuToLe32(data_ptr);
    trb->dmaSize = cpuToLe32((data_size) | (uint32_t) PRECISE_BURST_LENGTH);
    trb->ctrl = cpuToLe32(((uint32_t) (stream_id) << 16U) | (uint32_t) (TD_TYPE_NORMAL | cycleBit));
}

/**
 * Build normal TRB with interrupt on completion disabled and chain bit is set
 * @param trb pointer to memory where TRB is allocated
 * @param data_ptr pointer to data buffer
 * @param data_size data buffer size
 * @param stream_id stream ID
 */
static inline void BUILD_NORMAL_TRB_NO_IOC_CHAIN(CUSBDMA_DmaTrb * trb, uint32_t data_ptr, uint32_t data_size, uint16_t stream_id, uint32_t cycleBit) {
    trb->dmaAddr = cpuToLe32(data_ptr);
    trb->dmaSize = cpuToLe32(data_size | (uint32_t) PRECISE_BURST_LENGTH);
    trb->ctrl = cpuToLe32(((uint32_t) (stream_id) << 16U) | (uint32_t) (TD_TYPE_NORMAL | cycleBit | TDF_CHAIN_BIT));
}

/**
 * Build normal TRB with interrupt on completion enabled
 * @param trb pointer to memory where TRB is allocated
 * @param data_ptr pointer to data buffer
 * @param data_size data buffer size
 * @param stream_id stream ID
 */
static inline void BUILD_NORMAL_TRB(CUSBDMA_DmaTrb * trb, uint32_t data_ptr, uint32_t data_size, uint16_t stream_id, uint32_t cycleBit) {
    trb->dmaAddr = cpuToLe32(data_ptr);
    trb->dmaSize = cpuToLe32((data_size) | (uint32_t) PRECISE_BURST_LENGTH);
    trb->ctrl = cpuToLe32(((uint32_t) (stream_id) << 16U) | (uint32_t) (TD_TYPE_NORMAL | cycleBit | TDF_INT_ON_COMPLECTION | TDF_INT_ON_SHORT_PACKET));
}

/**
 * Build Link TRB
 * @param trb pointer to memory where TRB is allocated
 * @param target_ptr pointer to target place in memory
 * @param cycle bit
 * @param toggle cycle
 */
static inline void BUILD_LINK_TRB(CUSBDMA_DmaTrb * trb, uint32_t target_ptr, uint32_t cycleBit, uint32_t toggleCycle) {
    trb->dmaAddr = cpuToLe32(target_ptr);
    trb->dmaSize = 0U;
    trb->ctrl = cpuToLe32(TD_TYPE_LINK | TDF_CHAIN_BIT | cycleBit | toggleCycle);
}

/**
 * Get Physical address corresponding to the TRB virtual address.
 * This function assumes that the TRB buffer is contiguous
 * @param channel DMA channel
 * @param trb Pointer to the TRB virtual address
 * @return uintptr_t corresponding to the TRB PHYsical address
 */
static inline uintptr_t getPhyAddrOfTRBPtr(const CUSBDMA_DmaChannel *channel, const CUSBDMA_DmaTrb * trb) {
    /* parasoft-begin-suppress MISRA2012-RULE-11_4 "const CUSBDMA_DmaTrb* converted to unsigned long, DRV-5041" */
    uintptr_t trb_vaddr = ((uintptr_t) trb);
    /* parasoft-end-suppress MISRA2012-RULE-11_4 */
    /* parasoft-begin-suppress MISRA2012-RULE-11_4 "CUSBDMA_DmaTrb* converted to unsigned long, DRV-5042" */
    uintptr_t trb_vbase = ((uintptr_t) channel->trbBufferBaseAddr);
    /* parasoft-end-suppress MISRA2012-RULE-11_4 */

    /* Calculate the address offset for virtual address */
    uintptr_t trb_voffset = (uintptr_t) (trb_vaddr - trb_vbase);
    uintptr_t trb_phyaddr = channel->trbBufferBaseAddrPhy + trb_voffset;

    vDbgMsg(DBG_DMA_BASIC_MSG, 1, "EP%d-%s trb_vaddr(0x%08x) trb_phyaddr(0x%08x)\n",
            channel->hwUsbEppNum, (channel->isDirTx) ? "TX" : "RX", trb_vaddr, trb_phyaddr);

    return trb_phyaddr;
}

/**
 * Private Driver functions for TRB Buffer and chain management
 * @param dir transfer direction
 * @param zero is zlp enabled?
 * @param dataLen data length
 * @param maxTrbLen max TRB length
 * @return number of TRBs
 */
static uint32_t getNumDataTRBs(uint8_t dir, uint8_t zero, uint32_t dataLen, uint32_t maxTrbLen) {
    /* Calculate number of TRBs including short TRB */
    uint32_t numTRBs = (maxTrbLen != 0U) ? (((dataLen + maxTrbLen) - 1U) / maxTrbLen) : 0U;

    if (numTRBs == 0U) {
        /* we need to allocate a TRB even for zero length transfer */
        numTRBs = 1U;
    } else if ((dir != 0U) && (zero != 0U) && ((dataLen % maxTrbLen) == 0U)) {
        numTRBs++;
    } else {
        /* required by MISRA */
    }

    return numTRBs;
}

/**
 * Get actual number of TRBs
 * @param channel DMA channel
 * @param trParams transfer params
 * @return number of TRBs
 */
static uint32_t getActualNumTRBs(const CUSBDMA_DmaChannel *       channel,
                                 const CUSBDMA_DmaTransferParam * trParams) {

    uint32_t numOfTrbs = getNumDataTRBs(channel->isDirTx, trParams->zlp, trParams->len, channel->maxTrbLen);
    uint32_t trbWrIdx = channel->trbBufferEnqueueIdx;
    uint32_t nextTRBWrIdx = (trbWrIdx + numOfTrbs) % (channel->trbBufferSize);

    /* Check whether we have a wrap-around in this or next TD */
    if (nextTRBWrIdx < trbWrIdx) {
        /* Allocate space for LINK TRB */
        numOfTrbs++;
    }
    return numOfTrbs;
}

/**
 * Get TDL in DSING mode.
 * @param channel
 * @return tdl
 */
static uint32_t channelGetTDL(const CUSBDMA_DmaChannel *channel) {

    /**
     * In DSING mode the TRB Dequeue for triiger is contained in
     * the TRB chain descriptor pointed by trbChainDescReadIdx
     */
    int32_t rdIdx = (int32_t) channel->trbChainDescReadIdx;
    CUSBDMA_DmaTrbChainDesc *trbChainDesc = &channel->trbChainDescListHead[rdIdx];
    uint32_t xferLength = trbChainDesc->len;
    uint32_t tdl = 0;

    if (channel->wMaxPacketSize != 0U) {
        tdl = xferLength / channel->wMaxPacketSize;
        if ((xferLength % channel->wMaxPacketSize) != 0U) {
            tdl = tdl + 1U;
        }
    }

    return tdl;
}

/**
 * Allocation of TRBs
 * @param channel DMA Channel for the allocation
 * @param actualNumTRBs Number of TRBs to allocate. Should be always less than TRB buffer size
 * @return Index of allocation if success OR (-1) if failure
 */
static int32_t channelAllocTRBs(CUSBDMA_DmaChannel *channel, uint32_t actualNumTRBs) {
    int32_t trbStartIdx = -1;
    /* check if we have space for TRB buffer */
    uint32_t trbWrIdx = channel->trbBufferEnqueueIdx;
    uint32_t trbRdIdx = channel->trbBufferDequeueIdx;
    uint32_t bufferSize = channel->trbBufferSize;
    /* ringsize: size of ring under process */
    uint32_t ringSize = ((bufferSize + trbWrIdx) - trbRdIdx) % bufferSize;

    if ((ringSize + actualNumTRBs) < bufferSize) {
        trbStartIdx = (int32_t) trbWrIdx;
        channel->trbBufferEnqueueIdx = (trbWrIdx + actualNumTRBs) % bufferSize;
    } else {
        vDbgMsg(DBG_DMA_ERR_MSG, DBG_FYI, "Error: EP%d-%s TRB trbWrIdx(&d) trbRdIdx(%d) actualNumTRBs(%d) trbBufferSize(%d)\n",
                channel->hwUsbEppNum, (channel->isDirTx) ? "TX" : "RX",
                trbWrIdx, trbRdIdx, actualNumTRBs, bufferSize);
    }
    return trbStartIdx;
}

/**
 * Allocation of chain descriptor
 * @param channel DMA Channel for the allocation
 * @param trParams transfer params
 * @param actualNumTRBs Number of TRBs to allocate. Should be always less than TRB buffer size
 * @return TRB chain descriptor index
 */
static int32_t allocTRBChainDesc(CUSBDMA_DmaChannel *channel,
                                 const CUSBDMA_DmaTransferParam * trParams, uint32_t actualNumTRBs) {
    int32_t trbChainDescIdx = -1;
    int32_t wrIdx = (int32_t) channel->trbChainDescWriteIdx;
    int32_t rdIdx = (int32_t) channel->trbChainDescReadIdx;
    int32_t nextWrIdx = (wrIdx + 1) % ((int32_t) channel->trbChainDescBufferSz);

    /* check if we have space for TRB chain */
    if (nextWrIdx != rdIdx) {
        int32_t trbStartIdx = channelAllocTRBs(channel, actualNumTRBs);
        if (trbStartIdx >= 0) {
            CUSBDMA_DmaTrbChainDesc *trbChainDesc = &channel->trbChainDescListHead[wrIdx];

            /* reserve this chain element */
            trbChainDescIdx = wrIdx;
            channel->trbChainDescWriteIdx = (uint32_t) nextWrIdx;

            /* set the chain parameters */
            trbChainDesc->actualLen = 0U;
            trbChainDesc->start = (uint16_t) trbStartIdx;
            trbChainDesc->end = (uint16_t) ((((uint32_t) trbStartIdx) + (actualNumTRBs - 1U)) % (channel->trbBufferSize));

            trbChainDesc->len = trParams->len;
            trbChainDesc->requestQueued = trParams->requestQueued;
            trbChainDesc->requestOverflowed = trParams->requestOverflowed;
            trbChainDesc->trbChainState = CUSBDMA_TRB_CHAIN_QUEUED;
            trbChainDesc->shortPacketFlag = 0U;
        }
    } else {
        vDbgMsg(DBG_DMA_ERR_MSG, DBG_FYI, "Error: EP%d-%s TRBChainDesc nextWrIdx(%d) rdIdx(%d) trbChainDescBufferSz(%d)\n",
                channel->hwUsbEppNum, (channel->isDirTx) ? "TX" : "RX",
                nextWrIdx, rdIdx, channel->trbChainDescBufferSz);
    }

    return trbChainDescIdx;
}

/**
 * Program TRBs
 * @param channel DMA Channel for the allocation
 * @param toggleFirstCycleBit
 * @param params
 * @param startIdx
 * @param numTRBs
 */
static void programTRBs(CUSBDMA_DmaChannel *channel, uint8_t toggleFirstCycleBit,
                        const CUSBDMA_DmaTransferParam * params, uint32_t startIdx, uint32_t numTRBs) {

    uint32_t trbIdx = startIdx;
    uint32_t trbCount = 1;
    uint32_t dmaAddr = (uint32_t) (params->dmaAddr);
    uint32_t dataRemaining = params->len;
    uint32_t trbCycleBit = channel->trbCycleBit;
    uint32_t chMaxTrbLen = channel->maxTrbLen;
    uint16_t sid = params->sid;
    uint32_t trbBufferSize = channel->trbBufferSize;

    if (toggleFirstCycleBit != 0U) {
        trbCycleBit = trbCycleBit ^ TDF_CYCLE_BIT;
    }

    while ((trbCount++) < numTRBs) {
        CUSBDMA_DmaTrb *trbPtr = &(channel->trbBufferBaseAddr[trbIdx]);

        if (trbIdx == (trbBufferSize - 1U)) {
            uintptr_t trbBufferBaseAddr = channel->trbBufferBaseAddrPhy;
            /* Insert link TRB CH = 1, TC = 1 */
            BUILD_LINK_TRB(trbPtr, (uint32_t) trbBufferBaseAddr, trbCycleBit, TDF_TOGGLE_CYCLE_BIT);
            channel->trbCycleBit = channel->trbCycleBit ^ TDF_CYCLE_BIT;
        } else {
            /* Insert Data TRB CH = 1, TC = 0 */
            BUILD_NORMAL_TRB_NO_IOC_CHAIN(trbPtr, dmaAddr, chMaxTrbLen, sid, trbCycleBit);
            dmaAddr += chMaxTrbLen;
            dataRemaining -= chMaxTrbLen;
        }
        trbIdx = (trbIdx + 1U) % (trbBufferSize);
        /* revert any cycle-bit toggle done for first TRB */
        trbCycleBit = channel->trbCycleBit;
    }

    if (params->sid == 0U) {
        vDbgMsg(DBG_DMA_ERR_MSG, DBG_FYI, "EP%d-%s TRB_NORMAL trbPtr(0x%08X) dmaAddr(0x%08X) trbIdx(%d) len(%d)\n",
                channel->hwUsbEppNum, (channel->isDirTx) ? "TX" : "RX",
                &(channel->trbBufferBaseAddr[trbIdx]), dmaAddr, trbIdx, dataRemaining);
        BUILD_NORMAL_TRB(&(channel->trbBufferBaseAddr[trbIdx]),
                         dmaAddr, dataRemaining, 0, trbCycleBit);
    } else {
        /* we do not depend on IOC;
         * for ep-in we use IOT, for ep-out we use MD_EXIT + TDL */

        BUILD_NORMAL_TRB_NO_IOC(&(channel->trbBufferBaseAddr[trbIdx]),
                                dmaAddr, dataRemaining, sid, trbCycleBit);
    }
}

/**
 * Program DSING
 * @param channel DMA Channel for the allocation
 * @param params DMA transfer paramss
 * @return CDN_EOK on success, error code elsewhere
 */
static uint32_t channelProgramDSING(CUSBDMA_DmaChannel *             channel,
                                    const CUSBDMA_DmaTransferParam * params) {

    uint32_t retval = CDN_EOK;
    uint32_t actualNumTRBs = getActualNumTRBs(channel, params);
    int32_t trbChainDescIdx = allocTRBChainDesc(channel, params, actualNumTRBs);

    if (trbChainDescIdx < 0) {
        retval = CDN_ENOMEM;
    }

    vDbgMsg(DBG_DMA_ERR_MSG, DBG_FYI, "EP%d-%s DMAbuf(0x%08X) size(%d) trbChainDescIdx(%d) actualNumTRBs(%d)\n",
            channel->hwUsbEppNum, (channel->isDirTx) ? "TX" : "RX",
            params->dmaAddr, params->len, trbChainDescIdx, actualNumTRBs);
    /* If TRB chain descriptor is allocated then program TRB */
    if (retval == CDN_EOK) {
        programTRBs(channel, 0U, params,
                    channel->trbChainDescListHead[trbChainDescIdx].start, actualNumTRBs);
    }
    return retval;
}

/**
 * Program DMULT
 * @param pD pointer to driver object
 * @param channel DMA Channel for the allocation
 * @param params DMA transfer params
 * @return CDN_EOK on success, error code elsewhere
 */
static uint32_t channelProgramDMULT(const CUSBDMA_DmaController *pD,
                                    CUSBDMA_DmaChannel *channel, const CUSBDMA_DmaTransferParam * params) {

    uint32_t retval = CDN_EOK;
    uint32_t actualNumTRBs = getActualNumTRBs(channel, params);
    int32_t trbStartIdx = -1;

    if ((params->requestQueued != 0U) || (params->requestOverflowed != 0U)) {
        int32_t trbChainDescIdx = allocTRBChainDesc(channel, params, actualNumTRBs);

        if (trbChainDescIdx >= 0) {
            trbStartIdx = (int32_t) channel->trbChainDescListHead[trbChainDescIdx].start;
        }
    } else {
        trbStartIdx = channelAllocTRBs(channel, actualNumTRBs);
    }

    if (trbStartIdx < 0) {
        retval = CDN_ENOMEM;
    } else if (channel->trbBufferDmultDelinkIdx >= 0) {

        /* we already have a delimiter */
        programTRBs(channel, 0U, params, (uint32_t) trbStartIdx, actualNumTRBs);
    } else {
        uint32_t trbCtrlWord = 0;
        uint32_t trbRdWrDist = 0;
        uint32_t trbBufferSz = channel->trbBufferSize;
        /* Insert a delimiter at startIdx */
        programTRBs(channel, 1U, params, (uint32_t) trbStartIdx, actualNumTRBs);

        trbCtrlWord = channel->trbBufferBaseAddr[trbStartIdx].ctrl;
        channelUpdateState(pD, channel);
        trbRdWrDist = ((trbBufferSz + channel->trbBufferEnqueueIdx) - channel->trbBufferDequeueIdx) % trbBufferSz;

        /* Update the control word only if Distance is at-least 2 TRB */
        if (trbRdWrDist >= 2U) {
            channel->trbBufferBaseAddr[trbStartIdx].ctrl = trbCtrlWord ^ TDF_CYCLE_BIT;
        } else {
            channel->trbBufferDmultDelinkIdx = trbStartIdx;
        }
    }

    return retval;
}

/**
 * Update transfer length
 * @param channel DMA Channel for the allocation
 * @param trbChainDesc
 * @param stopIdx
 */
static void trbChainUpdateXferLength(const CUSBDMA_DmaChannel *channel,
                                     CUSBDMA_DmaTrbChainDesc * trbChainDesc, uint32_t stopIdx) {

    uint32_t xferLength = 0;
    uint32_t trbIdx = trbChainDesc->start;

    /* break loop on short packet */
    while (trbIdx != stopIdx) {
        xferLength += channel->trbBufferBaseAddr[trbIdx].dmaSize;
        trbIdx = (trbIdx + 1U) % channel->trbBufferSize;
    }
    xferLength += channel->trbBufferBaseAddr[stopIdx].dmaSize;

    trbChainDesc->actualLen = xferLength;

    if ((xferLength % channel->wMaxPacketSize) != 0U) {
        trbChainDesc->shortPacketFlag = 1U;
    }
}

/**
 * Update TRB descriptor chain state
 * @param channel DMA Channel for the allocation
 */
static void updateTRBDescChainState(const CUSBDMA_DmaChannel *channel) {

    uint32_t trbChainIdx = channel->trbChainDescReadIdx;
    uint32_t trbRdIdx = channel->trbBufferDequeueIdx;
    uint32_t trbWrIdx = channel->trbBufferEnqueueIdx;

    while (trbChainIdx != channel->trbChainDescWriteIdx) {
        CUSBDMA_DmaTrbChainDesc *trbChainDesc = &channel->trbChainDescListHead[trbChainIdx];
        uint32_t startTrbIdx = trbChainDesc->start;
        uint32_t endTrbIdx = trbChainDesc->end;

        /* if this TRB chain desc is complete move to the next desc */
        if (trbChainDesc->trbChainState == CUSBDMA_TRB_CHAIN_COMPLETE) {
            trbChainIdx = (trbChainIdx + 1U) % channel->trbChainDescBufferSz;
            continue;
        }

        if (startTrbIdx <= endTrbIdx) {
            /* this could also be a single trb chain */
            if ((trbRdIdx > endTrbIdx) ||
                ((trbRdIdx <= trbWrIdx) && (trbWrIdx < startTrbIdx))) {
                trbChainDesc->trbChainState = CUSBDMA_TRB_CHAIN_COMPLETE;
                trbChainUpdateXferLength(channel, trbChainDesc, endTrbIdx);
            }
        } else {
            /* chain has wrap-around */
            if ((trbRdIdx > endTrbIdx) && (trbWrIdx >= trbRdIdx)) {
                trbChainDesc->trbChainState = CUSBDMA_TRB_CHAIN_COMPLETE;
                trbChainUpdateXferLength(channel, trbChainDesc, endTrbIdx);
            }
        }

        /* increment trbChain index */
        trbChainIdx = (trbChainIdx + 1U) % channel->trbChainDescBufferSz;
    }
}

/**
 * Function to update channel state
 * @param pD pointer to driver object
 * @param channel pointer to channel object
 */
static void channelUpdateState(const CUSBDMA_DmaController *pD, CUSBDMA_DmaChannel *channel) {

    DmaRegs* regs = pD->regs;
    uint32_t epTrAddr = CPS_UncachedRead32(&regs->traddr);
    uint32_t trbBufferBase = (uint32_t) channel->trbBufferBaseAddrPhy;
    uint32_t trbBufferEnd = (uint32_t) (trbBufferBase + (channel->trbBufferSize * sizeof (CUSBDMA_DmaTrb)));
    uint32_t dorbel = CPS_UncachedRead32(&regs->ep_cmd);
    uint32_t status = CPS_UncachedRead32(&regs->ep_sts);
    uint32_t oldTrbDequeueIdx = channel->trbBufferDequeueIdx;

    if ((epTrAddr >= trbBufferBase) && (epTrAddr < trbBufferEnd)) {
        channel->trbBufferDequeueIdx = (epTrAddr - trbBufferBase) / (uint32_t)sizeof (CUSBDMA_DmaTrb);
    }

    /* Update dma channel status */
    if (channel->status >= CUSBDMA_STATUS_FREE) {
        if ((dorbel & DMARF_EP_DRDY) != 0U) {
            channel->status = CUSBDMA_STATUS_ARMED;
        } else if ((status & DMARF_EP_DBUSY) != 0U) {
            channel->status = CUSBDMA_STATUS_BUSY;
        } else {
            channel->status = CUSBDMA_STATUS_FREE;
        }
    }

    /* Update TRB descriptor chain state */
    if (oldTrbDequeueIdx != channel->trbBufferDequeueIdx) {
        updateTRBDescChainState(channel);
    }
}

/**
 * Function to toggle TRB cycle bit
 * @param channel pointer to channel object
 * @param trbIdx
 */
static void toggleTRBCycleBit(CUSBDMA_DmaChannel *channel, int32_t trbIdx) {
    uint32_t ctrlWord = channel->trbBufferBaseAddr[trbIdx].ctrl;

    ctrlWord ^= TDF_CYCLE_BIT;
    channel->trbBufferBaseAddr[trbIdx].ctrl = ctrlWord;
}

/**
 * Function to trigger channel
 * @param pD pointer to driver object
 * @param channel pointer to channel object
 */
static void channelTrigger(const CUSBDMA_DmaController *pD, CUSBDMA_DmaChannel *channel) {
    DmaRegs* regs = pD->regs;

    /* Trigger the channel if needed */
    if ((channel->status == CUSBDMA_STATUS_FREE) &&
        (channel->channelStalled == 0U) &&
        (channel->trbBufferDequeueIdx != channel->trbBufferEnqueueIdx)) {
        uint32_t currentDequeueAddr = CPS_UncachedRead32(&regs->traddr);
        uint32_t current_ext_addr = CPS_UncachedRead32(&regs->ep_dma_ext_addr);
        uint64_t deQueueAddr = (uint64_t) getPhyAddrOfTRBPtr(
            channel, &channel->trbBufferBaseAddr[channel->trbBufferDequeueIdx]);

        uint32_t deQueueAddr_l = (uint32_t) deQueueAddr;
        uint32_t deQueueAddr_h = (uint32_t) (deQueueAddr >> 32U);

        /* Patch de-queue address if needed */
        if (currentDequeueAddr == 0U) {
            vDbgMsg(DBG_DMA_ERR_MSG, DBG_FYI, "Warning: Patching EP_TRADDR:  0x%08X\n", deQueueAddr_l);
            CPS_UncachedWrite32(&regs->traddr, deQueueAddr_l);
        }

        if (current_ext_addr != deQueueAddr_h) {
            vDbgMsg(DBG_DMA_ERR_MSG, DBG_FYI, "Warning: Patching EP_EXT_ADDR old(0x%08X) new(0x%08X) \n", current_ext_addr, deQueueAddr_h);
            CPS_UncachedWrite32(&regs->ep_dma_ext_addr, deQueueAddr_h);
        }

        /* If TRB chain is De-linked in DMULT mode &&
         *  we have reached the de-linked index then toggle cycle bit */
        if ((channel->trbBufferDmultDelinkIdx >= 0) &&
            (channel->trbBufferDequeueIdx == (uint32_t) channel->trbBufferDmultDelinkIdx)) {
            toggleTRBCycleBit(channel, channel->trbBufferDmultDelinkIdx);
            channel->trbBufferDmultDelinkIdx = -1;
        }

        CPS_UncachedWrite32(&regs->ep_sts, DMARF_EP_TRBERR | DMARF_EP_DESCMIS);

        vDbgMsg(DBG_DMA_BASIC_MSG, 1, "EP%d-%s Ring Doorbell sid(%d)\n",
                channel->hwUsbEppNum, channel->isDirTx ? "Tx" : "Rx", channel->currentStreamID);
        if (channel->currentStreamID == 0U) {
            CPS_UncachedWrite32(&regs->ep_cmd, DMARF_EP_DRDY);
        } else {
            uint32_t tdl = channelGetTDL(channel);
            uint32_t cmd = DMARF_EP_DRDY | DMARF_EP_ERDY | (tdl << DMARF_EP_TDL_OFST) | DMARF_EP_STDL
                           | (channel->currentStreamID << 16);
            /* Ring Door bell and send ERDY */
            CPS_UncachedWrite32(&regs->ep_cmd, cmd);
        }
    }
}

/**
 * Reset / Abort DMA transmission
 * @param channel pointer to channel object
 */
static void channelResetTRBPtrs(CUSBDMA_DmaChannel *channel) {

    /* Reset trb buffer enqueue index */
    channel->trbBufferEnqueueIdx = 0U;
    /* Reset trb buffer dequeue index */
    channel->trbBufferDequeueIdx = 0U;
    /* Reset Stream ID of the current stream */
    channel->currentStreamID = 0U;
    channel->trbBufferDmultDelinkIdx = -1;
    channel->trbChainDescWriteIdx = 0U;
    channel->trbChainDescReadIdx = 0;

    (void) memset(channel->trbBufferBaseAddr, 0,
                  sizeof (CUSBDMA_DmaTrb) * channel->trbBufferSize);
    (void) memset(channel->trbChainDescListHead, 0,
                  sizeof (CUSBDMA_DmaTrbChainDesc) * channel->trbChainDescBufferSz);

}

/**
 * checks the channel
 * @param channel pointer to channel object
 * @return CDN_EOK on success, error otherwise
 */
static uint32_t checkChannel(const CUSBDMA_DmaChannel * channel) {
    uint32_t ret = CDN_EOK;
    /* check whether the trb base address is valid */
    if (channel->trbBufferBaseAddr == NULL) {
        ret = CDN_EINVAL;
    }

    /* check whether the trb buffer size is non-zero */
    if (ret == CDN_EOK) {
        if (channel->trbBufferSize == 0U) {
            ret = CDN_EINVAL;
        }
    }
    return ret;
}

/**
 * Disable the DMA channel pointed by the ep_sel register
 * @params regs register
 * @return CDN_EOK on success, error otherwise
 */
static uint32_t channelDisable(DmaRegs * regs) {
    uint32_t ret = CDN_EOK;
    uint32_t ep_cfg = 0;
    uint32_t ep_sts = 0;
    uint32_t counter = 0;

    /* Disable channel */
    ep_cfg = CPS_UncachedRead32(&regs->ep_cfg);
    ep_cfg &= ~DMARF_EP_ENABLE;
    CPS_UncachedWrite32(&regs->ep_cfg, ep_cfg);

    /* wait for DMA to stop */
    ep_sts = CPS_UncachedRead32(&regs->ep_sts);
    while ((ep_sts & DMARF_EP_DBUSY) == DMARF_EP_DBUSY) {
        CPS_DelayNs(1000);
        ep_sts = CPS_UncachedRead32(&regs->ep_sts);
        counter++;
        if (counter >= CUSBDMA_DEFAULT_TIMEOUT) {
            /* Timeout error if DMA still busy */
            vDbgMsg(DBG_DMA_BASIC_MSG, 1, "Warning: Timeout waiting for DMA busy to clear %d \n",
                    counter);
            ret = CDN_ETIMEDOUT;
            break;
        }
    }

    return ret;
}

/*****************************************************************************
* Public Driver function
*****************************************************************************/

/**
 * Probe of DMA controller driver
 * @param config pointer to configuration structure
 * @param sysReq pointer to structure where DMA driver memory requirements will be stored
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t CUSBDMA_Probe(const CUSBDMA_Config* config, CUSBDMA_SysReq * sysReq) {

    /* check input parameters */
    uint32_t ret = CUSBDMA_ProbeSF(config, sysReq);

    if (ret == CDN_EOK) {
        vDbgMsg(DBG_DMA_BASIC_MSG, 1, "Required private size %d\n", sizeof (CUSBDMA_DmaController));

        sysReq->trbMemSize = (uint32_t) 0; /* Deprecated */
        sysReq->privDataSize = (uint32_t) sizeof (CUSBDMA_DmaController);
    }
    return ret;
}

/**
 * Function sets DMA channel attributes for RX
 * @param ctrl pointer to DMA driver object
 */
static void SetRxDmaChannels(CUSBDMA_DmaController * ctrl) {
    uint8_t i = 0;

    for (i = 0; i < CUSBDMA_MAX_DMA_CHANNELS; i++) {
        if ((ctrl->cfg.dmaModeRx & ((uint16_t) 1U << i)) != 0U) {
            /* Set DMULT mode configuration */
            ctrl->rx[i].dmultEnabled = 1U;
            ctrl->rx[i].maxTrbLen = TD_DMULT_MAX_TRB_DATA_SIZE;
            ctrl->rx[i].maxTdLen = TD_DMULT_MAX_TD_DATA_SIZE;
        } else {
            /* Set Single mode configuration */
            ctrl->rx[i].dmultEnabled = 0U;
            ctrl->rx[i].maxTrbLen = TD_SING_MAX_TRB_DATA_SIZE;
            ctrl->rx[i].maxTdLen = TD_SING_MAX_TD_DATA_SIZE;
        }
    }
}

/**
 * Function sets DMA channel attributes for TX
 * @param ctrl pointer to DMA driver object
 */
static void SetTXDmaChannels(CUSBDMA_DmaController * ctrl) {
    uint8_t i = 0;

    for (i = 0; i < CUSBDMA_MAX_DMA_CHANNELS; i++) {
        if ((ctrl->cfg.dmaModeTx & ((uint16_t) 1U << i)) != 0U) {
            /* Set DMULT mode configuration */
            ctrl->tx[i].dmultEnabled = 1U;
            ctrl->tx[i].maxTrbLen = TD_DMULT_MAX_TRB_DATA_SIZE;
            ctrl->tx[i].maxTdLen = TD_DMULT_MAX_TD_DATA_SIZE;
        } else {
            /* Set Single mode configuration */
            ctrl->tx[i].dmultEnabled = 0U;
            ctrl->tx[i].maxTrbLen = TD_SING_MAX_TRB_DATA_SIZE;
            ctrl->tx[i].maxTdLen = TD_SING_MAX_TD_DATA_SIZE;
        }
    }
}

/**
 * Initialize DMA controller driver
 * @param pD pointer to DMA controller object
 * @param config pointer to configuration structure
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t CUSBDMA_Init(CUSBDMA_DmaController *pD, const CUSBDMA_Config* config) {

    /* check input parameters */
    uint32_t ret = CUSBDMA_InitSF(pD, config);

    if (ret == CDN_EOK) {
        CUSBDMA_DmaController * ctrl = pD;

        vDbgMsg(DBG_DMA_BASIC_MSG, 1, "Initialization CUSBD DMA component %s\n", " ");
        (void) memset((void*) ctrl, 0, sizeof (CUSBDMA_DmaController)); /* XDP test apps do that */

        ctrl->epMemRes = config->epMemRes;

        /* parasoft-begin-suppress MISRA2012-RULE-11_4 "'unsigned long' converted to 'DmaRegs *', DRV-4351" */
        ctrl->regs = (DmaRegs*) config->regBase; /* get pointer to the DMA HW base registers */
        /* parasoft-end-suppress MISRA2012-RULE-11_4 */

        ctrl->cfg = *config;

        SetRxDmaChannels(ctrl);

        SetTXDmaChannels(ctrl);
    }

    return ret;
}

/**
 * Destroy DMA controller driver object
 * @param pD pointer to DMA controller object
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t CUSBDMA_Destroy(CUSBDMA_DmaController *pD) {

    /* check input parameters */
    uint32_t ret = CUSBDMA_DestroySF(pD);
    CUSBDMA_DmaChannel * channel;
    uint8_t i = 0;

    vDbgMsg(DBG_DMA_BASIC_MSG, 1, "Destroying CUSBD DMA component %s\n", " ");

    if (ret == CDN_EOK) {
        /* Release channel from USB endpoint */
        for (i = 0; i < 16U; i++) {
            channel = &pD->rx[i];
            if (channel->status == CUSBDMA_STATUS_UNKNOW) {
                (void) CUSBDMA_ChannelRelease(pD, channel);
            }

            channel = &pD->tx[i];
            if (channel->status == CUSBDMA_STATUS_UNKNOW) {
                (void) CUSBDMA_ChannelRelease(pD, channel);
            }
        }
    }
    return ret;
}

/**
 * Configure channel
 * @param trbChainDescListHead
 * @param channel address of pointer to channel object
 * @param epMemRes
 * @param epTrbDescChainSz
 */
static inline void channelAllocChCfg(CUSBDMA_DmaTrbChainDesc* trbChainDescListHead, CUSBDMA_DmaChannel * channel,
                                     const CUSBDMA_MemResources *epMemRes, uint32_t epTrbDescChainSz) {

    /* Configure TRB buffer for this channel */
    channel->trbBufferBaseAddr = epMemRes->trbAddr;
    channel->trbBufferSize = epMemRes->trbBufferSize;
    channel->trbBufferBaseAddrPhy = epMemRes->trbDmaAddr;

    /* Configure TRB chain descriptor buffer for this channel */
    channel->trbChainDescBufferSz = epTrbDescChainSz;
    channel->trbChainDescListHead = trbChainDescListHead;
}

/**
 * Select channel for given endpoint number and endpoint direction
 * @param ctrl pointer to DMA driver object
 * @param channel address of pointer to channel object
 * @param isDirTx direction of endpoint
 * @param hwEpNum number of endpoint
 * @return CDN_EOK for success, error code elsewhere
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a "Pass parameter ctrl with const specifier", DRV-4352*/

/* Note that parameter 'ctrl' is used for obtaining non-const 'CUSBDMA_DmaChannel *' */
static uint32_t channelAllocChSel(CUSBDMA_DmaController * ctrl, CUSBDMA_DmaChannel ** channel, uint8_t isDirTx, uint8_t hwEpNum) {

    uint32_t ret = CDN_EOK;
    uint32_t trbChainSz = (uint32_t) (sizeof (ctrl->trbChainDesc) / sizeof (CUSBDMA_DmaTrbChainDesc));
    uint32_t epTrbDescChainSz = trbChainSz / (CUSBDMA_MAX_DMA_CHANNELS * 2U);
    uint32_t channelNum = (uint32_t) hwEpNum;

    if (isDirTx != 0U) {
        /* check channel status */
        if (((hwEpNum + 16U) > 31U) || (ctrl->tx[hwEpNum].status >= CUSBDMA_STATUS_FREE)) {
            ret = CDN_EINVAL;
        } else {
            channelNum += 16U;
            /* select channel for IN direction */
            *channel = &ctrl->tx[hwEpNum];
            (*channel)->isDirTx = 0x80;
        }
    } else {
        /* check channel status */
        if ((hwEpNum >= CUSBDMA_MAX_DMA_CHANNELS) ||
            (ctrl->rx[hwEpNum].status >= CUSBDMA_STATUS_FREE)) {
            ret = CDN_EINVAL;
        } else {
            /* select channel for OUT direction */
            *channel = &ctrl->rx[hwEpNum];
            (*channel)->isDirTx = 0x00;
        }
    }

    if (ret == CDN_EOK) {
        channelAllocChCfg(&ctrl->trbChainDesc[channelNum * epTrbDescChainSz], *channel,
                          &(*ctrl->epMemRes)[channelNum], epTrbDescChainSz);
    }
    return ret;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a */

/**
 * configure channel object
 * @param ctrl pointer to DMA driver object
 * @param channel pointer to channel object
 * @param channelParams channel parameters
 */
static void channelAllocChConf(CUSBDMA_DmaController * ctrl, CUSBDMA_DmaChannel * channel, CUSBDMA_ChannelParams* channelParams) {

    DmaRegs * regs = ctrl->regs;
    /* read endpoint interrupt flag register */
    uint32_t dma_ien = CPS_UncachedRead32(&regs->ep_ien);
    uint64_t trbBufferbase = (uint64_t) channel->trbBufferBaseAddrPhy;
    uint32_t trbBufferbase_ext = (uint32_t) ((trbBufferbase >> 32U) & 0xFFFFU);

    if (channelParams->hwEpNum < CUSBDMA_MAX_DMA_CHANNELS) {
        /* set interrupt flag enable for this channel */
        if (channelParams->isDirTx != 0U) {
            dma_ien |= (((uint32_t) 0x10000U) << ((uint32_t) channelParams->hwEpNum));
        } else {
            dma_ien |= (((uint32_t) 0x01U) << channelParams->hwEpNum);
        }

        channel->epConfig = channelParams->epConfig;

        /* set other fields */
        vDbgMsg(DBG_DMA_CHANNEL_USEGE_MSG, 1, "Channel 0x%p has been allocated for EP%d-%s trbBufferbase_ext 0x%x\n",
                channel, channelParams->hwEpNum, (channel->isDirTx != 0U) ? "TX" : "RX", trbBufferbase_ext);
        channel->controller = ctrl;
        channel->status = CUSBDMA_STATUS_FREE;
        channel->hwUsbEppNum = channelParams->hwEpNum;
        channel->wMaxPacketSize = channelParams->wMaxPacketSize;
        channel->trbBufferEnqueueIdx = 0U;
        channel->trbBufferDequeueIdx = 0U;
        channel->currentStreamID = 0U;
        channel->channelStalled = 0U;
        channel->trbCycleBit = TDF_CYCLE_BIT;
        channel->trbBufferDmultDelinkIdx = -1;
        channel->trbChainDescWriteIdx = 0U;

        /* configure DMA hardware */
        CPS_UncachedWrite32(&regs->ep_sel, (uint32_t) channelParams->hwEpNum | channel->isDirTx);

        /* set interrupts and configuration registers*/
        CPS_UncachedWrite32(&regs->ep_cfg, channelParams->epConfig);

        /*Clear all interrupts for this channel*/
        CPS_UncachedWrite32(&regs->ep_sts, 0xFFFFFFFFU);

        CPS_UncachedWrite32(&regs->ep_sts_en, channelParams->epIrqConfig);
        /* Enable interrupts for this endpoint */
        CPS_UncachedWrite32(&regs->ep_ien, dma_ien);

        /* set the high 16-bit for this channel-dma */
        CPS_UncachedWrite32(&regs->ep_dma_ext_addr, trbBufferbase_ext);
        CPS_UncachedWrite32(&regs->traddr, (uint32_t) trbBufferbase);
    }
}

/**
 * Allocate resources for DMA channel
 * @param pD pointer to DMA controller object
 * @param channelPtr pointer to memory where pointer to channel is allocated
 * @param channelParams channel parameters
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t CUSBDMA_ChannelAlloc(CUSBDMA_DmaController* pD, CUSBDMA_DmaChannel** channelPtr, CUSBDMA_ChannelParams* channelParams) {

    uint32_t ret = ((pD == NULL) || (channelPtr == NULL) || (channelParams == NULL)) ? CDN_EINVAL : CDN_EOK;

    CUSBDMA_DmaController * ctrl = NULL;
    CUSBDMA_DmaChannel * channel = NULL;

    /* check input parameters */
    if (ret == CDN_EOK) {
        if (channelParams->hwEpNum >= CUSBDMA_MAX_DMA_CHANNELS) {
            ret = CDN_EINVAL;
        }
    }

    if (ret == CDN_EOK) {
        /* select DMA channel*/
        ctrl = (CUSBDMA_DmaController*) pD;
        ret = channelAllocChSel(ctrl, &channel, channelParams->isDirTx, channelParams->hwEpNum);
    }
    if (ret == CDN_EOK) {
        /* configure DMA channel */
        channelAllocChConf(ctrl, channel, channelParams);
        *channelPtr = channel;
    }
    return ret;
}

/**
 * Free memory reserved for CUSBDMA_DmaTrbChainDesc element
 *
 * @param pD pointer to DMA controller object
 * @param channel pointer to channel
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t CUSBDMA_ChannelFreeHeadTrbChain(const CUSBDMA_DmaController * pD, CUSBDMA_DmaChannel* channel) {

    uint32_t ret = CDN_EOK;
    ret = CUSBDMA_ChannelFreeHeadTrbChSF(pD, channel);

    /* check whether the channel parameters are ok */
    if (ret == CDN_EOK) {
        ret = checkChannel(channel);
    }

    if (ret == CDN_EOK) {

        /* check whether we do have valid trb chain descriptors */
        if (channel->trbChainDescReadIdx != channel->trbChainDescWriteIdx) {
            CUSBDMA_DmaTrbChainDesc *trbChainDesc = &channel->trbChainDescListHead[channel->trbChainDescReadIdx];

            vDbgMsg(DBG_DMA_VERBOSE_MSG, 1, "EP%d-%s Free ChainDescIdx(%d) TRBs start(%d) end(%d) allocated\n",
                    channel->hwUsbEppNum,
                    (channel->isDirTx != 0U) ? "TX" : "RX",
                    channel->trbChainDescReadIdx,
                    trbChainDesc->start,
                    trbChainDesc->end
                    );

            channel->trbChainDescReadIdx = (channel->trbChainDescReadIdx + 1U) % channel->trbChainDescBufferSz;
            /* Free memory reserved for CUSBDMA_DmaTrbChainDesc element */
            (void) memset((void *) trbChainDesc, 0x0, sizeof (CUSBDMA_DmaTrbChainDesc));

        } else {
            /* No queued chain desc */
            vDbgMsg(DBG_DMA_ERR_MSG, DBG_FYI, "EP%d-%s No queued chain desc RdIdx(%d) WrIdx(%d)",
                    channel->hwUsbEppNum,
                    (channel->isDirTx != 0U) ? "TX" : "RX",
                    channel->trbChainDescReadIdx,
                    channel->trbChainDescWriteIdx
                    );
        }
    }
    return ret;
}

/**
 * Function to reset DMA channel
 * @param pD pointer to DMA controller object
 * @param channel Pointer to DMA channel
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t CUSBDMA_ChannelReset(CUSBDMA_DmaController * pD,
                              CUSBDMA_DmaChannel *    channel) {

    /* Check input parameters */
    uint32_t ret = CUSBDMA_ChannelResetSF(pD, channel);
    if (ret == CDN_EOK) {
        ret = checkChannel(channel);
    }

    if (ret == CDN_EOK) {
        DmaRegs * regs = pD->regs;
        uint64_t trbBufferbase = (uint64_t) channel->trbBufferBaseAddrPhy;

        CPS_UncachedWrite32(&regs->ep_sel,
                            ((uint32_t) channel->isDirTx | (uint32_t) channel->hwUsbEppNum));

        channelUpdateState(pD, channel);

        /* Check whether the channel is free */
        if ((channel->status == CUSBDMA_STATUS_FREE)
            && (channel->trbChainDescReadIdx == channel->trbChainDescWriteIdx)
            && (channel->trbBufferDequeueIdx == channel->trbBufferEnqueueIdx)) {

            vDbgMsg(DBG_DMA_ERR_MSG, DBG_FYI, "EP%d-%s Reset Pointers \r\n",
                    channel->hwUsbEppNum,
                    ((channel->isDirTx) == 0U) ? "RX" : "TX");

            channelResetTRBPtrs(channel);
            CPS_UncachedWrite32(&regs->traddr, (uint32_t) trbBufferbase);

        } else {
            /* Reset Pointers Denied */
            vDbgMsg(DBG_DMA_ERR_MSG, DBG_FYI, "EP%d-%s Reset Pointers Denied: ChannelStatus(%d)\r\n",
                    channel->hwUsbEppNum,
                    ((channel->isDirTx) == 0U) ? "RX" : "TX", channel->status);
            ret = CDN_EPERM;
        }
    }
    return ret;
}

/**
 * Release channel from USB endpoint
 *
 * @param pD Pointer to private data of DMA controller
 * @param channel Pointer to DMA channel
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t CUSBDMA_ChannelRelease(CUSBDMA_DmaController * pD, CUSBDMA_DmaChannel * channel) {

    /* Check input parameters */
    uint32_t ret = CUSBDMA_ChannelReleaseSF(pD, channel);
    DmaRegs * regs = NULL;
    uint32_t dmaIen;

    if (ret == CDN_EOK) {
        vDbgMsg(DBG_DMA_BASIC_MSG, 1, "Dma channel release ch: %p\n", channel);
        uint8_t isDirTx = channel->isDirTx;
        uint8_t hwUsbEppNum = channel->hwUsbEppNum;
        regs = pD->regs;
        CPS_UncachedWrite32(&regs->ep_sel,
                            ((uint32_t) isDirTx | (uint32_t) hwUsbEppNum));

        channelUpdateState(pD, channel);

        /*Disable endpoint and interrupt for DMA.*/
        dmaIen = CPS_UncachedRead32(&regs->ep_ien);
        if (hwUsbEppNum < CUSBDMA_NUM_OF_IN_ENDPOINTS) {
            if (isDirTx != 0U) {
                dmaIen &= ~(((uint32_t) 0x010000U) << (hwUsbEppNum));
            } else {
                dmaIen &= ~(((uint32_t) 0x01U) << hwUsbEppNum);
            }
        } else {
            ret = CDN_EINVAL;
        }
    }

    if (ret == CDN_EOK) {
        CPS_UncachedWrite32(&regs->ep_cfg, (uint32_t) 0);

        CPS_UncachedWrite32(&regs->ep_ien, dmaIen);
        CPS_UncachedWrite32(&regs->ep_sts_en, 0x0);

        ret = channelDisable(regs);
    }

    if (ret == CDN_EOK) {
        uint32_t epCmd = 0;
        uint32_t counter = 0;

        CPS_UncachedWrite32(&regs->ep_cmd, DMARF_EP_EPRST);

        /* wait for ep reset */
        epCmd = CPS_UncachedRead32(&regs->ep_cmd);
        while ((epCmd & DMARF_EP_EPRST) == DMARF_EP_EPRST) {
            CPS_DelayNs(1000);
            epCmd = CPS_UncachedRead32(&regs->ep_cmd);
            counter++;
            if (counter >= CUSBDMA_DEFAULT_TIMEOUT) {
                ret = CDN_ETIMEDOUT;
                break;
            }
        }
    }

    if (ret == CDN_EOK) {
        /*Clear all interrupts for this channel*/
        CPS_UncachedWrite32(&regs->ep_sts, 0xFFFFFFFFU);

        /* Abort the DMA transmission */
        channelResetTRBPtrs(channel);

        vDbgMsg(DBG_DMA_CHANNEL_USEGE_MSG, 1, "Channel 0x%p has been released\n", channel);

        /* Set channel status to unallocated */
        channel->status = CUSBDMA_STATUS_UNKNOW;
    }
    return ret;
}

/**
 * Prepares transfer and starts it.
 * @param pD Pointer to private data of DMA controller
 * @param channel Pointer to DMA channel
 * @param params transfer parameters
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t CUSBDMA_ChannelProgram(CUSBDMA_DmaController * pD, CUSBDMA_DmaChannel * channel, const CUSBDMA_DmaTransferParam * params) {

    /* check input parameters */
    uint32_t retval = CUSBDMA_ChannelProgramSF(pD, channel, params);

    if (retval == CDN_EOK) {
        retval = checkChannel(channel);
    }

    if (retval == CDN_EOK) {
        DmaRegs * regs = pD->regs;

        vDbgMsg(DBG_DMA_BASIC_MSG, 1, "EP%d-%s DMA prepare for channel: 0x%p  dmaAddr 0x%lx length %d\n",
                channel->hwUsbEppNum, channel->isDirTx ? "Tx" : "Rx", channel,
                params->dmaAddr, params->len);

        /* Set EP-select for this channel */
        CPS_UncachedWrite32(&regs->ep_sel,
                            ((uint32_t) channel->isDirTx | (uint32_t) channel->hwUsbEppNum));

        /* update EP_TRADDR */
        channelUpdateState(pD, channel);

        channel->currentStreamID = params->sid;
        if (channel->dmultEnabled != 0U) {
            retval = channelProgramDMULT(pD, channel, params);
        } else {
            retval = channelProgramDSING(channel, params);
        }

        /* trigger DMA */
        if (retval == CDN_EOK) {
            channelTrigger(pD, channel);
        }
    }

    return retval;
}

/**
 * Function triggers DMA transmission for the given channel
 * @param pD Pointer to private data of DMA controller
 * @param channel Pointer to DMA channel
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t CUSBDMA_ChannelTrigger(CUSBDMA_DmaController * pD, CUSBDMA_DmaChannel * channel) {

    /* check input parameters */
    uint32_t ret = CUSBDMA_ChannelTriggerSF(pD, channel);

    if (ret == CDN_EOK) {
        ret = checkChannel(channel);
    }

    if (ret == CDN_EOK) {
        DmaRegs * regs = pD->regs;

        vDbgMsg(DBG_DMA_BASIC_MSG, 1, "EP%d-%s ChannelTrigger for channel: 0x%p\n",
                channel->hwUsbEppNum, channel->isDirTx ? "Tx" : "Rx", channel);

        /* Set EP-select for this channel */
        CPS_UncachedWrite32(&regs->ep_sel,
                            ((uint32_t) channel->isDirTx | (uint32_t) channel->hwUsbEppNum));

        channelUpdateState(pD, channel);
        channelTrigger(pD, channel);
    }
    return ret;
}

/**
 * Function to update state of DMA channel
 * @param pD Pointer to private data of DMA controller
 * @param channel Pointer to DMA channel
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t CUSBDMA_ChannelUpdateState(CUSBDMA_DmaController * pD, CUSBDMA_DmaChannel * channel) {
    uint32_t ret = CUSBDMA_ChannelUpdateStateSF(pD, channel);

    if (ret == CDN_EOK) {
        ret = checkChannel(channel);
    }

    /* Call internal function to update state of DMA channel */
    if (ret == CDN_EOK) {
        channelUpdateState(pD, channel);
    }
    return ret;
}

/**
 * Updates the max packet size for a channel
 * @param pD Pointer to private data of DMA controller
 * @param channel Pointer to DMA channel
 * @param maxPacketSize max packet size
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t CUSBDMA_ChannelSetMaxPktSz(CUSBDMA_DmaController * pD, CUSBDMA_DmaChannel * channel, uint16_t maxPacketSize) {
    uint32_t ret = CUSBDMA_ChannelSetMaxPktSzSF(pD, channel);

    /* Call internal function to update state of DMA channel */
    if (ret == CDN_EOK) {
        DmaRegs * regs = pD->regs;
        uint32_t epCfg = CPS_UncachedRead32(&regs->ep_cfg);
        channel->wMaxPacketSize = maxPacketSize;
        CPS_UncachedWrite32(&regs->ep_cfg, epCfg);
    }
    return ret;
}

/**
 * Set or Clear channel stall
 * @param pD Pointer to private data of DMA controller
 * @param channel Pointer to DMA channel
 * @param value
 * @param timeout
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t CUSBDMA_ChannelHandleStall(CUSBDMA_DmaController * pD,
                                    CUSBDMA_DmaChannel * channel, uint32_t value, uint32_t timeout) {
    uint32_t ret = CUSBDMA_ChannelHandleStallSF(pD, channel);

    if (ret == CDN_EOK) {
        DmaRegs * regs = pD->regs;
        CPS_UncachedWrite32(&regs->ep_sel, (uint32_t) channel->isDirTx | channel->hwUsbEppNum);

        if (value != 0U) {
            /* if the channel is free and not already stalled */
            if ((channel->status == CUSBDMA_STATUS_FREE) && (channel->channelStalled == 0U)) {
                channel->channelStalled = 1U;
                /* set stall */
                CPS_UncachedWrite32(&regs->ep_cmd, DMA_EP_CMD_SSTALL | DMA_EP_CMD_ERDY | DMA_EP_CMD_DFLUSH);
                ret = waitForBitCleared(&regs->ep_cmd, DMA_EP_CMD_DFLUSH, timeout);
            }
        } else {
            channelUpdateState(pD, channel);
            /* Clear Stall and reset endpoint */
            CPS_UncachedWrite32(&regs->ep_cmd, DMA_EP_CMD_CSTALL | DMA_EP_CMD_EPRST);

            /* wait for end point reset to complete */
            ret = waitForBitCleared(&regs->ep_cmd, DMA_EP_CMD_EPRST, timeout);

            if (ret == CDN_EOK) {
                uint32_t dequeuePtr = (uint32_t) getPhyAddrOfTRBPtr(channel,
                                                                    &channel->trbBufferBaseAddr[channel->trbBufferDequeueIdx]);
                channel->trbBufferDequeueIdx = channel->trbBufferEnqueueIdx;
                CPS_UncachedWrite32(&regs->traddr, dequeuePtr);
                channel->channelStalled = 0U;
            }
        }

    }
    return ret;

}
/* parasoft-end-suppress METRICS-36-3 */

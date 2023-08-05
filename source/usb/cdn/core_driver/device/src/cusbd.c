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
 * cusbd.c
 * Cadence-USB-Device interface implementation
 *
 * Main source file
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include "cusbd_if.h"
#include "cusbd_structs_if.h"
#include "cdn_errno.h"              /* errors definitions */
#include "cdn_log.h"                /* debugging messages */
#include "cps.h"                    /* reg_write, reg_read functions */
#include "cusb_ch9_structs_if.h"    /* USB spec chapter 9 definitions */
#include "cusbdma_if.h"            /* DMA interface */
#include "sduc_list.h"              /* list types */
#include "sgdma_regs.h"             /* some DMA definitions */
#include "ss_dev_hw.h"              /* USBSS_DEV controller defs */
#include "byteorder.h"              /* endian macros */
#include "cusb_ch9_if.h"
#include "cusbd_sanity.h"
#include "cdn_osal.h"
#if !(defined CUSBD_DEFAULT_TIMEOUT)
#define CUSBD_DEFAULT_TIMEOUT 1000000U
#endif

#define SS_DEV_NAME "Cadence USB Super Speed device controller"
#define EP0_NAME "EP_0"
#define EP0_ADDRESS 0x80U
#define USBSSP_DBG_CUSBD        0x00000010 
#define USBSSP_DBG_CUSBD_ISR    0x00000001

#define CUSBSS_U1_DEV_EXIT_LAT   4
#define CUSBSS_U2_DEV_EXIT_LAT   512
#define USB_SS_MAX_PACKET_SIZE   1024

/* TODO : check for optimal value of queue size*/
#define CFG_CDNS_DSR_TASK_QUEUE_SZ (16U)

static inline void reqListDeleteItem(CUSBD_Req **headReq, CUSBD_Req *item);
static inline void handleIsrDev(CUSBD_PrivateData* dev, uint32_t usb_ists, uint8_t *exitFlag);
static uint32_t startHwTransfer(CUSBD_PrivateData * dev, CUSBD_EpPrivate *epp, CUSBD_Req const *req);

#ifndef TINYUSB_INTEGRATION
static inline uint32_t isrHandleEp0(CUSBD_PrivateData* dev, uint32_t ep_ists, uint8_t *exitFlag);
#else /* TINYUSB_INTEGRATION */
static inline uint32_t isrHandleEp0(CUSBD_PrivateData* dev, uint32_t ep_ists, uint32_t * EpStatusBase);
#endif  /* TINYUSB_INTEGRATION */

/* Cadence DSR data structures */
OSAL_QUEUE_DEF(1, cdns_dsr_qdef, CFG_CDNS_DSR_TASK_QUEUE_SZ, CUSBD_DSREventQueue);
osal_queue_t cdns_dsr_q;

const char * epstatus[5][16] ={
    {"CH9_EP0_UNCONNECTED"},
    {"CH9_EP0_SETUP_PHASE"},
    {"CH9_EP0_DATA_PHASE"},
    {"CH9_EP0_STATUS_PHASE"},
    {"CH9_EP0_JUNK_PHASE"}};
const char * functionid[10][16] =
    {{"API_NONE"},
    {"API_DCD_EDPT_XFER_1"},
    {"API_DCD_EDPT_XFER_2"},
    {"API_DCD_EDPT_XFER_3"},
    {"API_HANDLEEP0REQ"},
    {"API_INITEP0HW"},
    {"API_HANDLEEP0IRQSTATUS"},
    {"API_HANDLEEP0IRQIOC"},
    {"API_CONNECT_FIRST_1"},
    {"API_JUNK"}};

/**
 * This function converts a virtual address pointer to uintptr_t type for DMA
 * @param buffer Virtual pointer to data buffer
 * @return Physical address of data buffer
 */
/* parasoft-begin-suppress MISRA2012-RULE-11_4 "const CUSBDMA_DmaTrb* converted to unsigned long, DRV-4773" */
static inline uintptr_t getPhyAddrOfU8Ptr(const uint8_t * buffer) {
    return ((uintptr_t) buffer);
}
/* parasoft-end-suppress MISRA2012-RULE-11_4 */

/**********************************************************************
* Static methods for EP-OUT Aux buffer handling
**********************************************************************/
#define EPOUT_AUX_BUFFER_HDR_SZ     8U
#define EPOUT_AUX_BUFFER_MIN_SZ     64U

/**
 * Constructs a uint16_t value from uint8_t pointer
 *
 * @param dataPtr Pointer to uint8 buffer
 * @return uint16_t value constructed from uint8 buffer
 */
static inline uint16_t getU16ValFromU8Ptr(const uint8_t* dataPtr) {

    /* Constructs a uint32_t value from uint8_t pointer */
    uint16_t value = (uint16_t) dataPtr[0];
    uint16_t byte1 = (uint16_t) dataPtr[1] << 8U;

    value += byte1;
    return value;
}
#ifdef TINYUSB_INTEGRATION

/**
 * Save end point status register and clear interrupt
 * @param regs (in) Dma regs value
 * @param Epsts (out) ep_sts register value
 * @param ep_sel (in) ep_sel regisger value
 * @return :  CDN_EINVAL  : invalid input params
 *         :  CDN_EOK     : valid return
 */

static inline uint32_t SaveEndptReg(DmaRegs * regs, uint32_t * Epsts, uint32_t ep_sel)
{
    if(( NULL == regs) || (NULL == Epsts))
    {
        return CDN_EINVAL;
    }
    CPS_UncachedWrite32(&regs->ep_sel, ep_sel);
    *Epsts = CPS_UncachedRead32(&regs->ep_sts);
    if(*Epsts == 0)
    {
        return CDN_EFAULT;
    }
    CPS_UncachedWrite32(&regs->ep_sts, *Epsts); /* Clear endpt STS interrupt */
    return CDN_EOK;
}
/**
 * Get EP number and EP_SEL configuration value for EP number
 * @param ep_ists (in) ep status register value
 * @param ep_no (out) ep number
 * @param EPConfRaw (out) ep sel register config value
 * @return :  CDN_EINVAL  : invalid input params
 *         :  CDN_EOK     : valid return
 */
static inline uint32_t GetEPSelConfigValue(uint32_t ep_ists, uint8_t * ep_no, uint32_t * EPConfRaw)
{
    if(( NULL == ep_no) || (NULL == EPConfRaw))
    {
        return CDN_EINVAL;
    }
    switch(CHECK_SET_BITS(ep_ists))
    {
        case EP_0:
           *ep_no = 0U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_1:
           *ep_no = 1U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_2:
           *ep_no = 2U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_3:
           *ep_no = 3U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_4:
           *ep_no = 4U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_5:
           *ep_no = 5U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_6:
           *ep_no = 6U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_7:
           *ep_no = 7U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_8:
           *ep_no = 8U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_9:
           *ep_no = 9U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_10:
           *ep_no = 10U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_11:
           *ep_no = 11U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_12:
           *ep_no = 12U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_13:
           *ep_no = 13U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_14:
           *ep_no = 14U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_15:
           *ep_no = 15U;
           *EPConfRaw = *ep_no | DMARD_EP_RX ;
        break;
        case EP_16:
           *ep_no = 0U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_17:
           *ep_no = 1U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_18:
           *ep_no = 2U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_19:
           *ep_no = 3U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_20:
           *ep_no = 4U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_21:
           *ep_no = 5U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_22:
           *ep_no = 6U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_23:
           *ep_no = 7U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_24:
           *ep_no = 8U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_25:
           *ep_no = 9U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_26:
           *ep_no = 10U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_27:
           *ep_no = 11U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_28:
           *ep_no = 12U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_29:
           *ep_no = 13U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_30:
           *ep_no = 14U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
        case EP_31:
           *ep_no = 15U;
           *EPConfRaw = *ep_no | DMARD_EP_TX ;
        break;
    }
    return CDN_EOK;
}
#endif /* TINYUSB_INTEGRATION */

/**
 * Store uint16_t data in byte buffer
 * @param dataPtr
 * @param value
 */
static inline void setU8PtrFromU16Val(uint8_t* dataPtr, uint16_t value) {

    /* Store uint16_t data in byte buffer */
    dataPtr[0] = (uint8_t) (value);
    dataPtr[1] = (uint8_t) (value >> 8U);
}

/**
 * Initialize CUSBD Aux buffer for all EP-OUTs
 * @param dev: pointer to driver private data
 */
static void cusbdAuxBufferInit(CUSBD_PrivateData* dev) {

    uint32_t epOutIdx;
    /* Initialize CUSBD Aux buffer for all EP-OUTs */
    for (epOutIdx = 0U; epOutIdx < CUSBD_NUM_EP_OUT; epOutIdx++) {
        dev->epOutAuxBuffer[epOutIdx].bufferAddr = dev->config.epOutAuxBufferCfg[epOutIdx].bufferAddr;
        dev->epOutAuxBuffer[epOutIdx].bufferSize = dev->config.epOutAuxBufferCfg[epOutIdx].bufferSize;
        dev->epOutAuxBuffer[epOutIdx].readIdx = 0U;
        dev->epOutAuxBuffer[epOutIdx].writeIdx = 0U;
        dev->epOutAuxBuffer[epOutIdx].updateIdx = 0U;
        dev->epOutAuxBuffer[epOutIdx].maxPacketSize = EPOUT_AUX_BUFFER_MIN_SZ;

        vDbgMsg(USBSSP_DBG_CUSBD, 1, "cusbdAuxBufferInit: epOutIdx(%d) bufferAddr(%x) bufferSize(%x)\n",
                epOutIdx, dev->epOutAuxBuffer[epOutIdx].bufferAddr, dev->epOutAuxBuffer[epOutIdx].bufferSize);
    }
    return;
}

/**
 * Reset CUSBD Aux buffer for the specified EP-out-index
 * @param dev: pointer to driver private data
 * @param epOutIdx epOut index
 */
static void cusbdAuxBufferEpOutReset(CUSBD_PrivateData* dev, uint8_t epOutIdx) {

    vDbgMsg(USBSSP_DBG_CUSBD, 1, "cusbdAuxBufferEpOutReset: epOutIdx(%d)\n", epOutIdx);

    /* Reset all indices to zero */
    dev->epOutAuxBuffer[epOutIdx].readIdx = 0U;
    dev->epOutAuxBuffer[epOutIdx].writeIdx = 0U;
    dev->epOutAuxBuffer[epOutIdx].updateIdx = 0U;
    return;
}

/**
 * Reset CUSBD Aux buffer for all EP-OUTs
 * @param dev: pointer to driver private data
 */
static void cusbdAuxBufferReset(CUSBD_PrivateData* dev) {

    uint8_t epOutIdx;
    vDbgMsg(USBSSP_DBG_CUSBD, 1, "cusbdAuxBufferReset (%d)\n", 0);

    /* Reset the pointers for all Aux buffers  */
    for (epOutIdx = 0U; epOutIdx < CUSBD_NUM_EP_OUT; epOutIdx++) {
        cusbdAuxBufferEpOutReset(dev, epOutIdx);
    }
    return;
}

/**
 * Sets max packet size for CUSBD Aux buffer
 * @param dev: pointer to driver private data
 * @param epOutIdx epOut index
 * @param maxPacketSize max packet size
 */
static void cusbdAuxBufferEpSetMaxPktSz(CUSBD_PrivateData* dev, uint8_t epOutIdx, uint16_t maxPacketSize) {
    /* Update max packet size for this endpoint*/
    dev->epOutAuxBuffer[epOutIdx].maxPacketSize = maxPacketSize;
    return;
}

/**
 * Check for memory availability in CUSBD Aux buffer
 * @param epAuxBuffer pointer to ep aux buffer
 * @param bufferPtr buffer pointer
 * @param bufferRemaining remaining buffer
 */
static uint32_t checkAuxMemAvail(CUSBD_EpAuxBuffer *epAuxBuffer, uint8_t **bufferPtr, uint16_t bufferRemaining) {
    uint32_t ret = CDN_ENOMEM;

    /* check if the last queued transfer was completed */
    if (epAuxBuffer->writeIdx == epAuxBuffer->updateIdx) {
        if ((bufferRemaining >= (epAuxBuffer->maxPacketSize + (2U * EPOUT_AUX_BUFFER_HDR_SZ)))) {
            /* we have buffer for transfer */
            ret = CDN_EOK;
        } else if ((epAuxBuffer->writeIdx >= epAuxBuffer->readIdx) &&
                   (epAuxBuffer->readIdx > (epAuxBuffer->maxPacketSize + EPOUT_AUX_BUFFER_HDR_SZ))) {

            setU8PtrFromU16Val(*bufferPtr, 0xFFFFU); /* Indicate wrap around */

            epAuxBuffer->writeIdx = 0;
            epAuxBuffer->updateIdx = 0;
            *bufferPtr = &(epAuxBuffer->bufferAddr[epAuxBuffer->writeIdx]);
            ret = CDN_EOK;
        } else {
            /* required for MISRA */
        }
    }
    return ret;
}

/**
 * Handle Desc-MISS interrupt
 * @param dev: pointer to driver private data
 * @param channel Pointer to the DMA channel which generated the DESC_MISS
 */
static void cusbdAuxBufferHandleDescMiss(CUSBD_PrivateData* dev, CUSBDMA_DmaChannel *channel) {
    uint32_t ret = CDN_ENOMEM;
    uint8_t epOutIdx = (channel->hwUsbEppNum - 1U);
    CUSBD_EpAuxBuffer *epAuxBuffer = &dev->epOutAuxBuffer[epOutIdx];
    uint16_t bufferRemaining = epAuxBuffer->bufferSize - epAuxBuffer->writeIdx;
    uint8_t *bufferPtr = &(epAuxBuffer->bufferAddr[epAuxBuffer->writeIdx]);

    if (epAuxBuffer->readIdx > epAuxBuffer->writeIdx) {
        bufferRemaining = epAuxBuffer->readIdx - epAuxBuffer->writeIdx;
    }

    /* Check whether we have memory to initiate transfer */
    ret = checkAuxMemAvail(epAuxBuffer, &bufferPtr, bufferRemaining);

    /* program DMA for this transfer */
    if (ret == CDN_EOK) {
        CUSBDMA_DmaTransferParam xferParams;
        xferParams.dmaAddr = getPhyAddrOfU8Ptr(&(epAuxBuffer->bufferAddr[epAuxBuffer->writeIdx + EPOUT_AUX_BUFFER_HDR_SZ]));
        xferParams.len = epAuxBuffer->maxPacketSize;

        xferParams.requestOverflowed = 1U;
        xferParams.requestQueued = 0U;
        xferParams.sid = 0;

        ret = CUSBDMA_ChannelProgram(&dev->dmaController, channel, &xferParams);
    }

    /* Update pointers if channel program was successful */
    if (ret == CDN_EOK) {
        uint16_t nextOffset = (epAuxBuffer->maxPacketSize + EPOUT_AUX_BUFFER_HDR_SZ);
        epAuxBuffer->writeIdx += nextOffset;
        /* bytes 0&1 contain next-offset: Update them when transfer done & short packet */
        setU8PtrFromU16Val(&bufferPtr[0], nextOffset);

        /* set actual xfer len to zero */
        setU8PtrFromU16Val(&bufferPtr[2], 0U);
        /* Set bytesProcessed to zero */
        setU8PtrFromU16Val(&bufferPtr[4], 0U);
    }
}

/**
 * Aux buffer Update transfer
 * @param dev: pointer to driver private data
 * @param epp pointer to extended endpoint object
 * @param dmaChainDesc trb chain corresponding to a request
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a "Pass parameter dev with const specifier, DRV-4771" */
/* Note that dev is used to obtain non-const bufferPtr, for update aux-buffer */
static void cusbdAuxBufferUpdateXfer(CUSBD_PrivateData*             dev,
                                     const CUSBD_EpPrivate *        epp,
                                     const CUSBDMA_DmaTrbChainDesc *dmaChainDesc) {
    uint8_t epOutIdx = (epp->channel->hwUsbEppNum - 1U);
    CUSBD_EpAuxBuffer *epAuxBuffer = &dev->epOutAuxBuffer[epOutIdx];
    uint8_t *bufferPtr = &(epAuxBuffer->bufferAddr[epAuxBuffer->updateIdx]);
    uint32_t reqLength = epAuxBuffer->maxPacketSize;
    uint32_t actualLen = dmaChainDesc->actualLen;
    uint16_t nextOffset = EPOUT_AUX_BUFFER_HDR_SZ;

    vDbgMsg(USBSSP_DBG_CUSBD, 1, "EP%d-%s cusbdAuxBufferUpdateXfer xferLen(%d)\n",
            (epp->ep.address & 0xF), (epp->channel->isDirTx) ? "TX" : "RX", actualLen);
    if (actualLen > reqLength) {
        actualLen = reqLength;
    }

    /* Code actual len in byes 2 & 3 */
    bufferPtr[2] = (uint8_t) actualLen;
    bufferPtr[3] = (uint8_t) (actualLen >> 8U);

    /* increment update and write index in steps of 8 bytes */
    nextOffset += (uint16_t) ((actualLen + (EPOUT_AUX_BUFFER_HDR_SZ - 1U)) & (~(EPOUT_AUX_BUFFER_HDR_SZ - 1U)));

    /* update next-offset */
    bufferPtr[0] = (uint8_t) nextOffset;
    bufferPtr[1] = (uint8_t) (nextOffset >> 8U);

    /* update context variables */
    epAuxBuffer->updateIdx += nextOffset;

    /* note that there can't be more than 1 desc missing queued */
    epAuxBuffer->writeIdx = epAuxBuffer->updateIdx;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a */

/**
 * Is data valid in the buffer
 * @param dev: pointer to driver private data
 * @param epp pointer to extended endpoint object
 * @return data valid status
 */
static uint32_t cusbdAuxBufferIsDataValid(const CUSBD_PrivateData* dev,
                                          const CUSBD_EpPrivate *  epp) {
    uint8_t epOutIdx = (epp->channel->hwUsbEppNum - 1U);
    const CUSBD_EpAuxBuffer *epAuxBuffer = &dev->epOutAuxBuffer[epOutIdx];
    uint32_t dataValid = 0;

    if (epAuxBuffer->readIdx != epAuxBuffer->updateIdx) {
        dataValid = 1;
        vDbgMsg(USBSSP_DBG_CUSBD, 1, "EP%d-%s cusbdAuxBufferIsDataValid dataValid(%d)\n",
                (epp->ep.address & 0xF), ((epp->ep.address & 0x80) != 0U) ? "TX" : "RX", dataValid);
    }
    return dataValid;
}

/**
 * Is transfer queued in the buffer
 * @param dev: pointer to driver private data
 * @param epp pointer to extended endpoint object
 * @return transfer queued status
 */
static uint32_t cusbdAuxBufferIsXferQueued(const CUSBD_PrivateData* dev,
                                           const CUSBD_EpPrivate *  epp) {
    uint8_t epOutIdx = (epp->channel->hwUsbEppNum - 1U);
    const CUSBD_EpAuxBuffer *epAuxBuffer = &dev->epOutAuxBuffer[epOutIdx];
    uint32_t xferQueued = 0;

    /* If writeIdx != updateIdx then last transfer request has not finished */
    if (epAuxBuffer->writeIdx != epAuxBuffer->updateIdx) {
        xferQueued = 1;
    }
    return xferQueued;
}

/**
 * buffer copy
 * @param dest pointer to destination
 * @param src pointer to source
 * @param len data length
 */
static void auxBufferCopy(uint8_t * dest, const uint8_t * src, uint16_t len) {

    /* Copy aux-buffer content to user specified - request- buffer */
#ifdef CDN_RIPE3_PLAT
    uint16_t idx;
    for (idx = 0U; idx < len; idx++) {
        dest[idx] = src[idx];
    }
#else
    (void) memcpy(dest, src, len);
#endif
}

/**
 * Dequeue transfer
 * @param epAuxBuffer aux buffer
 * @param req request
 * @return remaining data length
 */
static uint32_t cusbdAuxBufferDequeueXfer(CUSBD_EpAuxBuffer *epAuxBuffer, CUSBD_Req * req) {
    uint32_t reqBufferRem = req->length - req->actual;
    uint8_t *bufferPtr = &(epAuxBuffer->bufferAddr[epAuxBuffer->readIdx]);
    uint16_t nextOffset = getU16ValFromU8Ptr(&bufferPtr[0]);
    uint16_t xferLength = getU16ValFromU8Ptr(&bufferPtr[2]);
    uint16_t bytesProcessed = getU16ValFromU8Ptr(&bufferPtr[4]);
    uint16_t length = xferLength - bytesProcessed;

    if (reqBufferRem >= length) {

        /* if request buffer remaining is greater than xfer length then copy full */
        auxBufferCopy(&req->buf[req->actual], &bufferPtr[bytesProcessed + EPOUT_AUX_BUFFER_HDR_SZ], length);

        /* Update the number of bytes xfered in the request*/
        req->actual += length;
        /* update ep-out buffer read-index */
        epAuxBuffer->readIdx += nextOffset;

    } else {
        length = (uint16_t) reqBufferRem;
        /* else copy only the amount possible */
        auxBufferCopy(&req->buf[req->actual], &bufferPtr[bytesProcessed + EPOUT_AUX_BUFFER_HDR_SZ], length);

        req->actual += length;
        bytesProcessed += length;

        /* since we didn't consume the entire buffer, store the number of bytes consumed */
        bufferPtr[4] = (uint8_t) bytesProcessed;
        bufferPtr[5] = (uint8_t) (bytesProcessed >> 8U);
    }
    return (uint32_t) length;
}

/**
 * Dequeue transfer
 * @param dev: pointer to driver private data
 * @param epp pointer to extended endpoint object
 * @param req request
 * @return transfer length
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a "Pass parameter dev with const specifier, DRV-4772"*/
/* Note that dev is used to obtain non-const epAuxBuffer, for updating aux-buffer */
static uint32_t cusbdAuxBufferDequeue(CUSBD_PrivateData*     dev,
                                      const CUSBD_EpPrivate *epp,
                                      CUSBD_Req *            req) {
    uint8_t epOutIdx = (epp->channel->hwUsbEppNum - 1U);
    CUSBD_EpAuxBuffer *epAuxBuffer = &dev->epOutAuxBuffer[epOutIdx];
    uint32_t xferLen = 0;

    vDbgMsg(USBSSP_DBG_CUSBD, 1, "EP%d-%s cusbdAuxBufferDequeue req->length(%d)\n",
            (epp->ep.address & 0xF), ((epp->ep.address & 0x80) != 0U) ? "TX" : "RX", req->length);

    /* Iterate through all the buffers, trying to fill the requested buffer */
    while (epAuxBuffer->readIdx != epAuxBuffer->updateIdx) {
        uint32_t length = 0;
        uint8_t *bufferPtr = &(epAuxBuffer->bufferAddr[epAuxBuffer->readIdx]);

        /* check for wrap-around */
        if ((bufferPtr[0] == 0xFFU) && (bufferPtr[1] == 0xFFU) && (epAuxBuffer->updateIdx < epAuxBuffer->readIdx)) {
            epAuxBuffer->readIdx = 0U;
            continue;
        }

        length = cusbdAuxBufferDequeueXfer(epAuxBuffer, req);
        xferLen += length;
        /* Break if we get a short packet, zero len packet or fill up the req buf */
        if ((length == 0U) || ((length % epp->ep.maxPacket) != 0U) || (req->actual >= req->length)) {
            break;
        }
    }
    return xferLen;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a */

/**********************************************************************
* Static methods from cusbdma module
**********************************************************************/
/**
 * Returns endpoint object
 * @param dev pointer to driver object
 * @param epNum endpoint number
 * @param epDir endpoint direction
 * @return pointer to endpoint object
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a "Pass parameter dev with const specifier, DRV-4278" */

/* Note that parameter 'dev' is used for obtaining non-const 'CUSBD_EpPrivate *' */
static inline CUSBD_EpPrivate *dmaCompleteCallbackGetEp(CUSBD_PrivateData * dev, uint8_t epNum, uint8_t epDir) {

    CUSBD_EpPrivate * epp;
    /* get endpoint from endpoint container */
    if (epNum == 0U) {
        epp = &dev->ep0;
    } else if (epDir > 0U) {
        epp = &dev->ep_in_container[epNum - 1U];
    } else {
        epp = &dev->ep_out_container[epNum - 1U];
    }
    return epp;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a */

/**
 * Get next chain descriptor
 * @param channel DMA channel
 * @return pointer to next chain descriptor
 */
static CUSBDMA_DmaTrbChainDesc* getNextChainDesc(const CUSBDMA_DmaChannel *channel) {
    CUSBDMA_DmaTrbChainDesc *nextChainDesc = NULL;

    if (channel->trbChainDescReadIdx != channel->trbChainDescWriteIdx) {
        nextChainDesc = &channel->trbChainDescListHead[channel->trbChainDescReadIdx];
    }
    return nextChainDesc;
}

/**
 * Check if DMA is pending for this channel
 * @param channel
 * @return dma pending status
 */
static uint8_t cusbdmaIsDMAPending(const CUSBDMA_DmaChannel *channel) {
    uint8_t dmaPending = 0;
    /* DMA is pending is TRB buffer dequeue Index is not same as enqueue index */
    if (channel->trbBufferDequeueIdx != channel->trbBufferEnqueueIdx) {
        dmaPending = 1U;
    }
    return dmaPending;
}

/**
 * Handle TRB chain corresponding to Aux buffer
 * @param dev pointer to driver object
 * @param epp
 * @param nextChainDesc
 */
static void cusbdmaHandleAuxXferCmpl(CUSBD_PrivateData* dev,
                                     CUSBD_EpPrivate *epp, const CUSBDMA_DmaTrbChainDesc *nextChainDesc) {

    CUSBD_Req *nextReq = epp->reqListHead;

    cusbdAuxBufferUpdateXfer(dev, epp, nextChainDesc);

    /* check if a request was queued*/
    if (nextReq != NULL) {
        uint32_t isShortPacket = 0U;
        /* dequeue in the request buffer */
        uint32_t len = cusbdAuxBufferDequeue(dev, epp, nextReq);

        isShortPacket = ((len == 0U) || ((len % epp->ep.maxPacket) != 0U)) ? 1U : 0U;

        /* check whether the request is complete */
        if ((isShortPacket != 0U) || (nextReq->actual >= nextReq->length)) {
            nextReq->status = CDN_EOK;
            reqListDeleteItem(&(epp->reqListHead), nextReq);
            if (nextReq->complete != NULL) {
                nextReq->complete(&epp->ep, nextReq);
            }
        }
    }
    return;
}

/**
 * Handle a completed TRB chain for non-zero EP
 * @param dev
 * @param nextChainDesc
 * @param epp
 */
static void cusbdmaHandleCmplTRBDescEpx(CUSBD_PrivateData* dev,
                                        const CUSBDMA_DmaTrbChainDesc *nextChainDesc, CUSBD_EpPrivate *epp) {

    CUSBD_Req *nextReq = epp->reqListHead;

    if (nextChainDesc->requestQueued != 0U) {
        /* check if request list not empty and call complete function */
        if (nextReq != NULL) {
            reqListDeleteItem(&(epp->reqListHead), nextReq);
            nextReq->actual += nextChainDesc->actualLen;
            nextReq->status = CDN_EOK;
            if (nextReq->complete != NULL) {
                nextReq->complete(&epp->ep, nextReq);
            }
        }
    } else if (nextChainDesc->requestOverflowed != 0U) {
        /* if this corresponds to an overflow request */
        cusbdmaHandleAuxXferCmpl(dev, epp, nextChainDesc);
    } else {
        /* required for MISRA */
    }
}

/**
 * Process all completed chains
 * @param dev pointer to driver object
 * @param epp pointer to extended endpoint object
 * @param channel DMA Channel for the allocation
 */
static void cusbdmaHandleCompletedTRBChain(CUSBD_PrivateData* dev, CUSBD_EpPrivate *epp, CUSBDMA_DmaChannel *channel) {
    CUSBDMA_DmaTrbChainDesc *nextChainDesc = getNextChainDesc(channel);

    while (nextChainDesc != NULL) {
        if (nextChainDesc->trbChainState == CUSBDMA_TRB_CHAIN_COMPLETE) {
            /* Check if request is not empty and update number of actually processed bytes */
            if (channel->hwUsbEppNum == 0U) {
                if (dev->request != NULL) {
                    dev->request->actual += nextChainDesc->actualLen;
                }
            } else {
                cusbdmaHandleCmplTRBDescEpx(dev, nextChainDesc, epp);
            }
            /* Free the head(oldest) TRB chain descriptor for this channel */
            (void) CUSBDMA_ChannelFreeHeadTrbChain(&dev->dmaController, channel);
            nextChainDesc = getNextChainDesc(channel);
        } else {
            break;
        }
    }
}

/**
 * Process IOC ISP ITP interrupt after data transfer
 * @param dev
 * @param channel
 */
static void cusbdmaProcessDataXferInt(CUSBD_PrivateData* dev, CUSBDMA_DmaChannel *channel) {

    CUSBD_EpPrivate *epp = NULL;

    /* update channel status */
    (void) CUSBDMA_ChannelUpdateState(&dev->dmaController, channel);

    epp = dmaCompleteCallbackGetEp(dev, channel->hwUsbEppNum, channel->isDirTx);

    /* Process all completed chains */
    cusbdmaHandleCompletedTRBChain(dev, epp, channel);

    /* If EP is stalled, stall the corresponding DMA channel */
    if (epp->ep_state == CUSBD_EP_STALLED) {
        (void) CUSBDMA_ChannelHandleStall(&dev->dmaController, channel, 1U, CUSBD_DEFAULT_TIMEOUT);
    }
}

/**
 * Send ERDY packet
 * @param pD pointer to driver object
 * @param sid
 */
static void sendErdy(const CUSBD_PrivateData* pD, uint32_t sid) {
    CPS_UncachedWrite32(&pD->reg->USBR_EP_CMD, (DMARF_EP_ERDY | (sid << 16)));
}

/**
 * Handle Descriptor Missed interrupt
 * @param dev: pointer to driver object
 * @param channel: pointer to DMA channel
 */
static void isrHandleDescMiss(CUSBD_PrivateData* dev, CUSBDMA_DmaChannel *channel) {
    /* call callback if set */
    if (channel->trbBufferEnqueueIdx != channel->trbBufferDequeueIdx) {
        vDbgMsg(USBSSP_DBG_CUSBD, 1, "EP%d-%s isrHandleOther: DMARF_EP_DESCMIS: Ignore\n",
                channel->hwUsbEppNum, (channel->isDirTx) ? "TX" : "RX");
    } else if (dev->callbacks.descMissing != NULL) {
        dev->callbacks.descMissing(dev, (uint8_t) (channel->hwUsbEppNum | channel->isDirTx));
    } else if ((channel->isDirTx == 0U) && (channel->hwUsbEppNum != 0U)) {
        CUSBD_EpPrivate *epp = dmaCompleteCallbackGetEp(dev, channel->hwUsbEppNum, channel->isDirTx);
        CUSBD_Req *req = epp->reqListHead;
        uint32_t ret = CDN_EOK;

        if (req == NULL) {
            cusbdAuxBufferHandleDescMiss(dev, channel);
        } else {
            while (req->requestPending != 0U) {
                ret = startHwTransfer(dev, epp, req);
                if (ret != CDN_EOK) {
                    break;
                }
                req->requestPending = 0U;
                req = req->nextReq;
            }
        }
    } else {
        /* Required for MISRA */
    }
    return;
}

/**
 * Handle Prime
 * @param dev: pointer to driver object
 * @param channel: pointer to DMA channel
 */
static void isrHandlePrime(CUSBD_PrivateData* dev, const CUSBDMA_DmaChannel *channel) {
    CUSBD_EpPrivate * epp = NULL;
    vDbgMsg(USBSSP_DBG_CUSBD, 1, "DMAIRQ: PRIME\n", 0);

    /* Get the end point private object */
    epp = dmaCompleteCallbackGetEp(dev, channel->hwUsbEppNum, channel->isDirTx);

    /* Check if streams are enabled */
    if (epp->ep.maxStreams != 0U) {
        uint8_t dmaPending = cusbdmaIsDMAPending(channel);
        CUSBD_Req *nextReq = epp->reqListHead;
        uint8_t nextRequestPending = 0U;

        if (nextReq != NULL) {
            nextRequestPending = nextReq->requestPending;
        }

        /*
         * For PRIME received while data transfer we need to retry ERDY packet to
         * go back to DATA MOVE state
         * sent to re-open stream transfer
         */
        if (dmaPending != 0U) {
            vDbgMsg(USBSSP_DBG_CUSBD, 1, "EP%d-%s sendErdy\r\n",
                    channel->hwUsbEppNum, (channel->isDirTx ? "TX" : "RX"));
            sendErdy(dev, channel->currentStreamID);
        } else if (nextRequestPending != 0U) {

            vDbgMsg(USBSSP_DBG_CUSBD, 1, "EP%d-%s Start Pending Request \r\n",
                    channel->hwUsbEppNum, (channel->isDirTx ? "TX" : "RX"));
            /* if the next request is pending, start it */
            (void) startHwTransfer(dev, epp, nextReq);
            nextReq->requestPending = 0U;
        } else {
            vDbgMsg(USBSSP_DBG_CUSBD, 1, "EP%d-%s Primed - Waiting for transfer \r\n",
                    channel->hwUsbEppNum, (channel->isDirTx ? "TX" : "RX"));
        }
    }
}

/**
 * Handle MD exit
 * @param dev: pointer to driver object
 * @param channel: pointer to DMA channel
 */
static void isrHandleMdExit(CUSBD_PrivateData* dev, CUSBDMA_DmaChannel *channel) {
    vDbgMsg(USBSSP_DBG_CUSBD, 1, "DMAIRQ: MD_EXIT\n", 0);

    /*
     * handle only OUT stream endpoint here
     */
    if (channel->isDirTx == 0U) {

        /* check TDL value of actual transfer*/
        uint32_t tdl = CPS_UncachedRead32(&dev->reg->USBR_EP_CMD) & 0x0000FE00U;

        /* update request only when all data has been transferred for current request*/
        if (tdl == 0U) {
            /* Handle tdl = 0 */
            cusbdmaProcessDataXferInt(dev, channel);
        } else {
            /* we should have got PRIME */
        }
    } else {
        vDbgMsg(USBSSP_DBG_CUSBD, 1, "EP%d-%s MD_EXIT Ignoring for IN transfers \r\n",
                channel->hwUsbEppNum, (channel->isDirTx ? "TX" : "RX"));
    }
}

/**
 * Handle other interrupts
 * @param dev: pointer to driver object
 * @param channel: pointer to DMA channel
 * @param epSts Endpoint status
 */
static void isrHandleOther(CUSBD_PrivateData* dev, CUSBDMA_DmaChannel *channel, uint32_t epSts) {

    /* check descriptor missing interrupt */
    if ((epSts & DMARF_EP_DESCMIS) != 0U) {
        vDbgMsg(USBSSP_DBG_CUSBD, 1, "DMAIRQ: DESCMIS%s\n", "");
        isrHandleDescMiss(dev, channel);
    }

    /* check iso error */
    if ((epSts & DMARF_EP_ISOERR) != 0U) {
        vDbgMsg(USBSSP_DBG_CUSBD, 1, "DMAIRQ: ISOERR%s\n", "");
    }

    /* check side error */
    if ((epSts & DMARF_EP_SIDERR) != 0U) {

        vDbgMsg(USBSSP_DBG_CUSBD, 1, "DMAIRQ: SIDERR\n", 0);
    }

    /* MD_EXIT should be handled first;
     * If PRIME & MD_EXIT came together this would cause a redundant ERDY to be sent to HOST
     * however sending redundant ERDY should not be an issue */
    if ((epSts & DMARF_EP_MD_EXIT) != 0U) {
        isrHandleMdExit(dev, channel);
    }

    /* check prime interrupt */
    if ((epSts & DMARF_EP_PRIME) != 0U) {
        isrHandlePrime(dev, channel);
    }

    /* check stream error */
    if ((epSts & DMARF_EP_STREAMR) != 0U) {

        vDbgMsg(USBSSP_DBG_CUSBD, 1, "DMAIRQ: STREAMR\n", 0);
    }

}

/**
 * Handle Interrupt
 * @param dev pointer to driver object
 * @param channel DMA Channel for the allocation
 * @param epSts Endpoint status
 */
static void isrHandle(CUSBD_PrivateData* dev, CUSBDMA_DmaChannel *channel, uint32_t epSts) {

    vDbgMsg(USBSSP_DBG_CUSBD_ISR, 1, "isrHandle EP%d-%s epSts %x \n",
            channel->hwUsbEppNum, (channel->isDirTx ? "TX" : "RX"), epSts);

    /* check TRB error */
    if ((epSts & DMARF_EP_TRBERR) != 0U) {
        vDbgMsg(USBSSP_DBG_CUSBD_ISR, 1, "DMAIRQ: TRBERR %s\n", "");
        if (channel->dmultEnabled != 0U) {
            (void) CUSBDMA_ChannelTrigger(&dev->dmaController, channel);
        }
    }

    /* check IOC and ISP */
    if (((epSts & DMARF_EP_IOC) != 0U) || ((epSts & DMARF_EP_ISP) != 0U) || ((epSts & DMARF_EP_IOT) != 0U)) {

        if (((epSts & DMARF_EP_IOT) > 0U) && (channel->isDirTx == 0U)) {
            /*
             * Updating transfer for stream OUT endpoint is handled on MD_EXIT event
             * For OUT packet IOT is mostly generated before MD_EXIT and rearming
             * transfer is useless: MD_EXITs cancels transfer issues to controller
             * so that we must rely on MD_EXIT rather than IOT for OUT transfers.
             */
        } else {
            (void) cusbdmaProcessDataXferInt(dev, channel);

        }
    }

    /* stop program if critical error: Data overflow */
    if ((epSts & DMARF_EP_OUTSMM) != 0U) {
        vDbgMsg(USBSSP_DBG_CUSBD, 1, "DMAIRQ: OUTSMM%s\n", "");
    }

    isrHandleOther(dev, channel, epSts);
}
#ifndef TINYUSB_INTEGRATION
/* Pure Cadence driver; no class driver  */
/**
 * Interrupt handler
 * @param pD pointer to DMA controller object
 */
static void cusbdmaIsr(CUSBD_PrivateData* dev) {

    CUSBDMA_DmaController * ctrl = &(dev->dmaController);
    uint32_t epSts, epIsts = 0U;
    CUSBDMA_DmaChannel *channel = NULL;
    DmaRegs * regs = ctrl->regs;
    uint32_t i;
    uint32_t epSel;

    /* Check interrupts for all endpoints */
    epIsts = CPS_UncachedRead32(&regs->ep_ists);

    if (epIsts != 0U) {
        /* Save endpoint select value */
        epSel = CPS_UncachedRead32(&regs->ep_sel);
        vDbgMsg(USBSSP_DBG_CUSBD_ISR, 1, "DMAIRQ ep_ists SFR: %x\n", epIsts);
        /* check all endpoints interrupts */
        for (i = 0; i < 32U; i++) {
            uint32_t isDirTx;
            uint32_t epNum;

            if ((epIsts & (((uint32_t) 1U) << i)) == 0U) {
                continue;
            }
            /* ascertain EP direction */
            if (i > 15U) {
                isDirTx = DMARD_EP_TX;
                epNum = i - 16U;
            } else {
                isDirTx = 0U;
                epNum = i;
            }

            if (epNum == 0U) {
                continue;
            }

            CPS_UncachedWrite32(&regs->ep_sel, epNum | isDirTx);

            epSts = CPS_UncachedRead32(&regs->ep_sts);
            CPS_UncachedWrite32(&regs->ep_sts, epSts);

            if (isDirTx != 0U) /*transmit EP*/ {
                channel = &ctrl->tx[epNum];
            } else {
                channel = &ctrl->rx[epNum];
            }
            isrHandle(dev, channel, epSts);
        }
        /* Restore enpoint select value */
        CPS_UncachedWrite32(&regs->ep_sel, epSel);
    }
    return;
}
#else
/**
 * Interrupt handler
 * @param pD pointer to DMA controller object
 */
static uint32_t cusbdmaIsr(CUSBD_PrivateData* dev,uint32_t ep_ists, uint32_t * EpStatusBase) {

    CUSBDMA_DmaController * ctrl = &(dev->dmaController);
    DmaRegs* regs = ctrl->regs;
    CUSBDMA_DmaChannel *channel = NULL;
    uint32_t ret = 0;
    uint32_t EPSelBackup;

    if (ep_ists != 0U) {
        /* Save ep_sel value */
        EPSelBackup = CPS_UncachedRead32(&regs->ep_sel);
        vDbgMsg(USBSSP_DBG_CUSBD, 1, "DMAIRQ ep_ists SFR: %x\n", ep_ists);
        /* check all endpoints interrupts */
        while(ep_ists)
        {
            uint8_t EPNo;
            uint32_t EPConfRaw;
            ret = GetEPSelConfigValue(ep_ists, &EPNo, &EPConfRaw);
            if(ret != CDN_EOK)
            {
                vDbgMsg(USBSSP_DBG_CUSBD, 1, "Invalid EPNo or EPConfRaw ret %d\n", ret);
                break;
            }
            CPS_UncachedWrite32(&regs->ep_sel, EPConfRaw);

            if ((EPConfRaw & 0x80U) > 0) /*transmit EP*/ {
                channel = &ctrl->tx[EPNo];
                isrHandle(dev, channel, EpStatusBase[EPNo + 16U]);
            } else {
                channel = &ctrl->rx[EPNo];
                isrHandle(dev, channel, EpStatusBase[EPNo]);
            }
            ep_ists = ep_ists & (ep_ists - 1U);

        }
        /* Restore backup ep_sel*/
        CPS_UncachedWrite32(&regs->ep_sel, EPSelBackup);

    }
    return ret;
}
#endif /*TINYUSB_INTEGRATION*/
/**
 * wait until bit is cleared macro
 * @param reg register
 * @param bit selected bit
 */

/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
static uint32_t waitUntilBitCleared(uint32_t * reg, uint32_t bit) {
    uint32_t counter = CUSBD_DEFAULT_TIMEOUT;
    uint32_t ret = CDN_EOK;
    /* Check for specified bit to be set */
    while ((CPS_UncachedRead32(reg) & bit) > 0U) {
        /* break loop if timeout occur */
        if (counter == 0U) {
            ret = CDN_ETIMEDOUT;
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "%s() Error timeout \n", __func__);
            break;
        }
        counter--;
        CPS_DelayNs(1000);
    }
    return ret;
}
/* parasoft-end-suppress METRICS-36-3 */

/**
 * wait until bit is cleared macro
 * @param reg register
 * @param bit selected bit
 * @return CDN_EOK on success, error otherwise
 */

/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
static uint32_t waitUntilBitSet(uint32_t * reg, uint32_t bit) {
    uint32_t counter = CUSBD_DEFAULT_TIMEOUT;
    uint32_t ret = CDN_EOK;
    /* Check for specified bit to be set */
    while ((CPS_UncachedRead32(reg) & bit) == 0U) {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "%s(), %x \n", __func__, ((CPS_UncachedRead32(reg) & bit)));
        /* break loop if timeout occur */
        if (counter == 0U) {
            ret = CDN_ETIMEDOUT;
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "%s() Error timeout \n", __func__);
            break;
        }
        counter--;
        CPS_DelayNs(1000);
    }
    return ret;
}
/* parasoft-end-suppress METRICS-36-3 */

/**
 * Function adds the item at tail of doubly-linked list
 * @param headReq head
 * @param item item
 */
static inline void reqListAddTail(CUSBD_Req **headReq, CUSBD_Req *item) {
    /* Check for an empty list */
    if (*headReq == NULL) {
        *headReq = item;
        item->nextReq = item;
        item->prevReq = item;
    } else {
        CUSBD_Req *last = (*headReq)->prevReq;

        item->nextReq = *headReq;
        item->prevReq = last;
        last->nextReq = item;
        (*headReq)->prevReq = item;
    }
}

/**
 * Function deletes the corresponding item
 * @param headReq head
 * @param item item
 */
static inline void reqListDeleteItem(CUSBD_Req **headReq, CUSBD_Req *item) {

    /* If there is only one item in the list */
    if ((item->nextReq == NULL) || (item->prevReq == NULL)) {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "Warning: item already removed 0x%08X\n", (uintptr_t) item);
    } else if (item->nextReq == item) {
        *headReq = NULL;
    } /* Adjustments done with next and previous items */
    else {
        CUSBD_Req *lastReq = item->prevReq;
        CUSBD_Req *firstReq = item->nextReq;
        firstReq->prevReq = lastReq;
        lastReq->nextReq = firstReq;
        if (*headReq == item) {
            *headReq = firstReq;
        }
    }

    item->nextReq = NULL;
    item->prevReq = NULL;
}

/**
 * return next request from endpoint transfer queue
 * @param ep pointer to endpoint object
 * @return next endpoint request, if NULL it means that next request does not exist
 */
static CUSBD_Req *getNextReq(CUSBD_EpPrivate const * ep) {

    return ep->reqListHead;
}

/**
 * get the last request queued
 * @param ep Pointer to endpoint private data
 * @return Pointer to last request
 */
static CUSBD_Req *getLastReq(CUSBD_EpPrivate const * ep) {

    CUSBD_Req *lastReq = NULL;
    CUSBD_Req *headReq = ep->reqListHead;

    /* Request are queued in doubly linked list.*/
    /* The prev-pointer of the head request points to last request */
    if (headReq != NULL) {
        lastReq = headReq->prevReq;
    }
    return lastReq;
}

/**
 * Function return actual speed basing on value read from USB_STS register
 * @param dev pointer to device object
 * @return actual speed value
 */
static CH9_UsbSpeed getActualSpeed(CUSBD_PrivateData * dev) {

    uint32_t reg = CPS_UncachedRead32(&dev->reg->USBR_STS);

    /*read speed from SFR*/
    uint8_t speed = GET_USB_STS_SPEED(reg);
    CH9_UsbSpeed actualSpeed;

    switch (speed) {

    /* low*/
    case 1: actualSpeed = CH9_USB_SPEED_LOW;
        break;
    /*full*/
    case 2: actualSpeed = CH9_USB_SPEED_FULL;
        break;
    /*high*/
    case 3: actualSpeed = CH9_USB_SPEED_HIGH;
        break;
    /*super*/
    case 4: actualSpeed = CH9_USB_SPEED_SUPER;
        break;
    /*super plus*/
    case 5: actualSpeed = CH9_USB_SPEED_SUPER_PLUS;
        break;
    /* unknown*/
    default: actualSpeed = CH9_USB_SPEED_UNKNOWN;
        break;
    }
    return actualSpeed;
}

/**
 * start data transfer on selected endpoint
 * @param dev pointer to driver object
 * @param epp pointer to extended endpoint object
 * @param req pointer to request object
 */
static uint32_t startHwTransfer(CUSBD_PrivateData * dev, CUSBD_EpPrivate *epp, CUSBD_Req const *req) {

    uint32_t ret = CDN_EOK;
    CUSBDMA_DmaTransferParam trParams;

    /* calculate residue data */
    uint32_t bytesToTransfer = req->length - req->actual;
    uint32_t numOfbytes = (bytesToTransfer > TD_SING_MAX_TRB_DATA_SIZE) ? TD_SING_MAX_TRB_DATA_SIZE : bytesToTransfer;

    trParams.dmaAddr = req->dma + req->actual;
    trParams.len = numOfbytes;
    trParams.sid = req->streamId;
    trParams.zlp = req->zero;
    if (req->complete != NULL) {
        trParams.requestQueued = 1U;
    } else {
        trParams.requestQueued = 0;
    }

    /* create transfer descriptor on transfer ring*/
    ret = CUSBDMA_ChannelProgram(&(dev->dmaController), epp->channel, &trParams);
    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Failed to program DMA on ep: %02X\n", epp->ep.address);
    }
    return ret;
}

/**
 * Run control transfer on default endpoint
 * @param dev pointer to driver object
 * @param dir data direction 1: dev-to-host 0:host-to-dev
 * @param req transfer request
 * @param erdy 1 force ERDY packet
 */
static uint32_t ep0transfer(CUSBD_PrivateData * dev, uint8_t dir, const CUSBD_Req * req, uint8_t erdy) {

    uint32_t ret = CDN_EOK;
    CUSBDMA_DmaChannel *channel = (dir > 0U) ? dev->ep0DmaChannelIn : dev->ep0DmaChannelOut;
    CUSBDMA_DmaTransferParam trParams;
    uint8_t programDma = 0U;

    (void) CUSBDMA_ChannelReset(&dev->dmaController, channel);

    if (dir == 0U) {
        CUSBDMA_DmaTrbChainDesc *nextChainDesc = getNextChainDesc(channel);
        /* update channel status */
        (void) CUSBDMA_ChannelUpdateState(&dev->dmaController, channel);
        if (nextChainDesc != NULL) {
            if (nextChainDesc->trbChainState != CUSBDMA_TRB_CHAIN_COMPLETE) {
                (void) CUSBDMA_ChannelTrigger(&dev->dmaController, channel);
            }
        } else {
            programDma = 1U;
        }
    }

    if ((dir != 0U) || (programDma != 0U)) {
        trParams.dmaAddr = req->dma;
        trParams.len = (req->length < dev->ep0.ep.maxPacket) ? req->length : dev->ep0.ep.maxPacket;
        trParams.sid = 0U;
        trParams.requestQueued = 1U;
        trParams.zlp = req->zero;

        vDbgMsg(USBSSP_DBG_CUSBD_ISR, DBG_HIVERB, "Programming EP0 transfer for length %d\n", trParams.len);
        /* create transfer descriptor on TRB ring*/
        ret = CUSBDMA_ChannelProgram(&(dev->dmaController), channel, &trParams);
    }

    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_CUSBD_ISR, DBG_CRIT, "Failed to program DMA on EP0\n", 0);
    } else {
        if (erdy != 0U) {
            /* send ERDY packet */
            CPS_UncachedWrite32(&dev->reg->USBR_EP_CMD, EP_CMD_ERDY); /* OUT data phase */
        }
    }
    return ret;
}

/**
 * Enable interrupt on given endpoint
 * @param dev pointer to driver object
 * @param epNum endpoint number
 * @param epDir endpoint index
 */
static void epEnableInterrupt(CUSBD_PrivateData *dev, uint8_t epNum, uint8_t epDir) {

    uint32_t epIntBit;
    uint32_t ep_ien;

    /* calculate bit position */
    if (epDir > 0U) {
        epIntBit = (uint32_t) epNum << 16;
    } else {
        epIntBit = (uint32_t) epNum;
    }

    /* write selected bit to ep_ein register*/
    ep_ien = CPS_UncachedRead32(&dev->reg->USBR_EP_IEN);
    ep_ien |= epIntBit << 1;
    CPS_UncachedWrite32(&dev->reg->USBR_EP_IEN, ep_ien);
}

/**
 * Construct EP config register
 * @param epType endpoint type
 * @param maxPacketSize
 * @param epBuffering buffering value
 * @param mult mult field
 * @param maxBurst maxburst value
 * @return register
 */
static uint32_t constructEpCfgReg(uint8_t epType, uint16_t maxPacketSize, uint8_t epBuffering, uint8_t mult, uint8_t maxBurst) {
    uint32_t reg = 0;

    SET_EP_CONF_ENABLE(&reg); /* Enable end point */
    SET_EP_CONF_EPTYPE(&reg, epType); /* set ep type */
    SET_EP_CONF_MAXPKTSIZE(&reg, le16ToCpu(maxPacketSize)); /* set max packet size */
    SET_EP_CONF_BUFFERING(&reg, epBuffering);
    SET_EP_CONF_MULT(&reg, mult);
    SET_EP_CONF_MAXBURST(&reg, maxBurst);
    SET_EP_CONF_EPENDIAN(&reg, (uint8_t)CUSBD_ENDIANESS_CONV_FLAG);

    return reg;
}

/**
 * Enable HW
 * @param dev pointer to driver object
 * @param ep endpoint
 * @param desc endpoint descriptor
 * @param channelParams channel parameters
 */
static void epEnableHw(CUSBD_PrivateData *dev, CUSBD_Ep *ep, const uint8_t * desc, CUSBDMA_ChannelParams *channelParams) {

    uint32_t reg = 0;
    uint8_t epBuffering = 0;
    uint8_t mult = ep->mult; /* must be calculated differently for HS and SS */
    uint8_t epType = desc[3]; /* bDescriptorType */
    uint8_t epNum = USBD_EPNUM_FROM_EPADDR(ep->address);
    uint8_t epDir = USBD_EPDIR_FROM_EPADDR(ep->address);
    uint32_t irqEnFlags = EP_STS_EN_TRBERREN | EP_STS_EN_ISOERREN | EP_STS_EN_DESCMISEN;

    /* get buffering value */
    if (epDir > 0U) {
        epBuffering = dev->config.epIN[epNum - 1U].bufferingValue;
    } else {
        epBuffering = dev->config.epOUT[epNum - 1U].bufferingValue;
    }
    if (epBuffering > 0U) {
        epBuffering--;
    }

    reg = constructEpCfgReg(epType, channelParams->wMaxPacketSize, epBuffering, mult, ep->maxburst);

    /* Configure streams support only for SS and SSP mode */
    if (dev->device.speed >= CH9_USB_SPEED_SUPER) {

        /* get information about streams from endpoint companion descriptors which
           should be located in memory just after endpoint descriptor */

        if ((desc[7] == CH9_USB_DS_SS_USB_EP_COMPANION) && (desc[8] == CH9_USB_DT_SS_USB_EP_COMPANION)) {

            ep->compDesc = &desc[7];

            /* check if streams are used for endpoint */
            /* need some cleanup - 'maybe' - currently there is redundancy, but this works
             * for ep-in we use IOT
             * for ep-out we use MD_EXIT + TDL */
            if (ep->compDesc[3] > 0U) {
                SET_EP_CONF_STREAM_EN(&reg);
                reg |= 0x10U; /* set TDL_CHK in ep_cfg */
                reg |= 0x20U; /* set SID_CHK in ep_cfg */
                ep->maxStreams = ep->compDesc[3];
                irqEnFlags |= DMARF_EP_IOT | DMARF_EP_PRIME | DMARF_EP_SIDERR | DMARF_EP_MD_EXIT | DMARF_EP_STREAMR;
            }
        } else {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Endpoint companion descriptor not found for ep: %02X\n", ep->address);
        }
    }

    channelParams->epConfig = reg;
    channelParams->epIrqConfig = irqEnFlags;
    /* Configure endpoint */

    /* Enable ITP interrupt for iso endpoint */
    if ((epType & CH9_USB_EP_TRANSFER_MASK) == CH9_USB_EP_ISOCHRONOUS) {
        reg = CPS_UncachedRead32(&dev->reg->USBR_IEN);
        reg |= USB_IEN_ITPIEN;
        CPS_UncachedWrite32(&dev->reg->USBR_IEN, reg);
    }
}

/**
 * Enable endpoint
 * @param dev pointer to driver object
 * @param ep endpoint
 * @param desc endpoint descriptor
 * @param maxPacketSize max packet size
 */
static uint32_t epEnableEx(CUSBD_PrivateData *dev, CUSBD_Ep *ep, const uint8_t * desc, uint16_t maxPacketSize) {

    CUSBD_EpPrivate * epp = ep->epPrivate;
    uint8_t epNum = USBD_EPNUM_FROM_EPADDR(ep->address);
    uint8_t epDir = USBD_EPDIR_FROM_EPADDR(ep->address);
    uint32_t ret = CDN_EOK;
    CUSBDMA_ChannelParams channelParams;

    /* set enabled state */
    epp->ep_state = CUSBD_EP_ENABLED;

    /* no pending transfer on initializing procedure */
    epp->transferPending = 0;

    /* No actual request */
    epp->actualReq = NULL;

    /* clear wedge flag */
    epp->wedgeFlag = 0;

    ep->desc = desc;
    ep->maxPacket = maxPacketSize;

    channelParams.hwEpNum = epNum;
    channelParams.isDirTx = epDir;
    channelParams.wMaxPacketSize = maxPacketSize;

    /* configure endpoint hardware -- needs to be moved to cusbdma */
    epEnableHw(dev, ep, desc, &channelParams);

    /* allocate DMA channel */
    ret = CUSBDMA_ChannelAlloc(&(dev->dmaController),
                               &epp->channel, &channelParams);

    if (ret == CDN_EOK) {
        epEnableInterrupt(dev, epNum, epDir);
        if ((epDir == 0U) && (epNum != 0U)) {
            cusbdAuxBufferEpSetMaxPktSz(dev, (epNum - 1U), maxPacketSize);
        }
    } else {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Could not allocate DMA channel EP%d-%s", epNum, (epDir != 0U) ? "TX" : "RX");
    }

    return ret;
}

/**
 * Function gets max burst value for SS and SSP mode
 * @param desc endpoint descriptor
 * @param maxBurstSize max burst value
 */
static void getMaxBurstSizeSS(uint8_t const * desc, uint8_t * maxBurstSize) {

    /* in endpoint descriptor, type is done on third byte, mask it with two less */
    /* significant bits */
    uint8_t epDescType = desc[3] & 0x03U;

    /* check if super speed endpoint companion available */
    if ((desc[7] == CH9_USB_DS_SS_USB_EP_COMPANION)
        && (desc[8] == CH9_USB_DT_SS_USB_EP_COMPANION)) {

        vDbgMsg(USBSSP_DBG_CUSBD, DBG_HIVERB, "SuperSpeed Endpoint companion found-\n", 0);
        *maxBurstSize = desc[9];

        if (epDescType == CH9_USB_EP_INTERRUPT) {
            if (*maxBurstSize > 2U) {
                vDbgMsg(USBSSP_DBG_CUSBD, DBG_HIVERB, "WARNING: Limiting SS interrupt-ep maxBurstSize(%d) to 2 \n", *maxBurstSize);
                *maxBurstSize = 2;
            }
        }
    }
}

/**
 * This function gets 'mult' value for an endpoint
 * @param epDesc: Pointer the endpoint descriptor
 * @param actualSpeed: Actual speed
 * @param maxburst: Max burst
 * @return The Mult value
 */
static uint8_t getMult(const uint8_t * epDesc, CH9_UsbSpeed actualSpeed, uint8_t maxburst) {

    uint8_t mult = 0U;
    uint8_t epDescType = epDesc[3] & 0x03U;

    /* Only set for Super speed ISO end point when maxburst is > 0 */
    if ((actualSpeed >= CH9_USB_SPEED_SUPER) && (epDescType == CH9_USB_EP_ISOCHRONOUS) && (maxburst != 0U)) {
        mult = epDesc[10] & 0x03U;
    } else if (actualSpeed == CH9_USB_SPEED_HIGH) {
        if ((epDescType == CH9_USB_EP_INTERRUPT) || (epDescType == CH9_USB_EP_ISOCHRONOUS)) {
            mult = (epDesc[5] & 0x18U) >> 3U;
        }
    } else {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    /* Check for error conditions */
    if (mult > 2U) {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_HIVERB, "WARNING: Limiting mult(%d) to 2 \n", mult);
        mult = 2U;
    }
    return mult;
}

/**
 * This function enables the endpoint
 * @param pD pointer to driver object
 * @param ep: Endpoint
 * @param desc: Pointer to the endpoint descriptor
 * @return CDN_EOK on success, error otherwise
 */
static uint32_t epEnable(CUSBD_PrivateData *pD, CUSBD_Ep *ep, const uint8_t * desc) {

    uint32_t ret = CDN_EOK;
    uint16_t maxPacketSize;

    /* check endpoint descriptor correctness */
    if (desc[1] != CH9_USB_DT_ENDPOINT) {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Invalid endpoint descriptor on ep: %02X\n", ep->address);
        ret = CDN_EINVAL;
    }

    if (ret == CDN_EOK) {
        /* calculate max packet size from bytes: 4 and 5 written in Little endian */
        maxPacketSize = (((uint16_t) desc[5] & 0x7U) << 8) | (uint16_t) desc[4];
        /* check maxPacketSize */
        if (maxPacketSize == 0U) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Incorrect wMaxPacketSize: %d\n", maxPacketSize);
            ret = CDN_EINVAL;
        }
    }

    if (ret == CDN_EOK) {

        /* Set max burst size */
        if (pD->device.speed >= CH9_USB_SPEED_SUPER) {
            getMaxBurstSizeSS(desc, &(ep->maxburst));
        }

        /* set mult */
        ep->mult = getMult(desc, pD->device.speed, ep->maxburst);
        ret = epEnableEx(pD, ep, desc, maxPacketSize);
    }

    return ret;
}

/**
 * Disable callback
 * @param ep: Endpoint
 */
static void epDisableCallback(CUSBD_Ep * ep) {

    /* get extended endpoint object */
    CUSBD_EpPrivate * epp = ep->epPrivate;

    /* check if request list not empty and call complete function */
    if (epp->reqListHead != NULL) {
        CUSBD_Req * nextReq = getNextReq(epp);
        while (nextReq != NULL) {
            /* remove request from transfer queue */
            reqListDeleteItem(&(epp->reqListHead), nextReq);
            nextReq->status = CDN_ECANCELED;
            /*call complete callback*/
            nextReq->complete(ep, nextReq);
            nextReq = getNextReq(epp);
        }
    }
}

/**
 * Disable endpoint
 * @param pD pointer to driver object
 * @param ep pointer to endpoint object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t epDisable(CUSBD_PrivateData *pD, CUSBD_Ep * ep) {

    uint32_t ret = CDN_EOK;

    /* check if endpoint exist and return if no */
    if (ep->address == 0U) {
        ret = CDN_EINVAL;
    }

    if (ret == CDN_EOK) {

        CUSBD_PrivateData* dev;
        CUSBD_EpPrivate * epp;

        dev = pD;
        epp = ep->epPrivate;

        ret = CUSBDMA_ChannelRelease(&(dev->dmaController), epp->channel);
        if (ret != CDN_EOK) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Could not release DMA channel on ep:%02X \n", ep->address);
        }

        /* set enabled state to disabled */
        epp->ep_state = CUSBD_EP_DISABLED;
        epp->transferPending = 0U;

        /* Reset aux buffer for epOut */
        if (ep->address <= 0xFU) {
            cusbdAuxBufferEpOutReset(dev, (ep->address - 1U));
        }

        /* call callback */
        epDisableCallback(ep);
    }

    return ret;
}

/**
 * start transfer on hardware
 * @param dev pointer to driver object
 * @param epp pointer to endpoint object
 * @param req pointer to request object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t reqQueueRunHw(CUSBD_PrivateData * dev, CUSBD_EpPrivate * epp, CUSBD_Req * req) {

    uint32_t ret = 0U;

    /* add request to transfer queue*/
    reqListAddTail(&(epp->reqListHead), req);

    /* queue transfer on hardware */
    epp->transferPending = 1U;
    epp->actualReq = epp->reqListHead;
    /* start transfer on hardware */
    ret = startHwTransfer(dev, epp, req);
    return ret;
}

/**
 * handle ep out request
 * @param dev pointer to driver object
 * @param epp pointer to endpoint object
 * @param req pointer to request object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t handleSwEpOutReq(CUSBD_PrivateData * dev, CUSBD_EpPrivate * epp, CUSBD_Req * req) {
    uint32_t ret = CDN_EOK;
    uint32_t isShortPacket = 0U;

    /* check if this ep-out has un-handled aux buffer*/
    if (cusbdAuxBufferIsDataValid(dev, epp) != 0U) {
        uint32_t len = cusbdAuxBufferDequeue(dev, epp, req);
        isShortPacket = ((len == 0U) || ((len % epp->ep.maxPacket) != 0U)) ? 1U : 0U;
    }

    if ((isShortPacket != 0U) || (req->actual >= req->length)) {
        req->status = CDN_EOK;
        if (req->complete != NULL) {
            req->complete(&epp->ep, req);
        }
    } else {
        /* request is not yet processed - check whether it is being processed */
        uint32_t isAuxReqPending = cusbdAuxBufferIsXferQueued(dev, epp);
        if (req->complete != NULL) {
            if (isAuxReqPending == 0U) {
                ret = reqQueueRunHw(dev, epp, req);
            } else {
                req->requestPending = 1U;
                /* add request to transfer queue*/
                reqListAddTail(&(epp->reqListHead), req);
            }

        } else if (isAuxReqPending == 0U) {
            epp->transferPending = 1U;
            epp->actualReq = req;
            ret = startHwTransfer(dev, epp, req);
        } else {
            /* Request is pending and callback is not set. Ignore*/
        }
    }

    return ret;
}

/**
 * handle ep in request
 * @param dev pointer to driver object
 * @param epp pointer to endpoint object
 * @param req pointer to request object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t handleSwEpInReq(CUSBD_PrivateData * dev, CUSBD_EpPrivate * epp, CUSBD_Req * req) {
    uint32_t ret = CDN_EOK;
    /* program in the transfer for remaining data */
    if ((req->noInterrupt) > 0U) {
        epp->transferPending = 1U;
        epp->actualReq = req;
        ret = startHwTransfer(dev, epp, req);
    } else {
        /* add req to queue and then program TRBs*/
        ret = reqQueueRunHw(dev, epp, req);
    }
    return ret;
}

/**
 * handle ep0 request
 * @param dev pointer to driver object
 * @param req pointer to request object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t handleEp0Req(CUSBD_PrivateData * dev, CUSBD_Req * req) {
    uint32_t ret;

    vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "dev->ep0NextState: %d\n", dev->ep0NextState);
    /* -------- Patch for Linux SET_CONFIGURATION(1) handling -------------- */
    if (dev->ep0NextState == CH9_EP0_STATUS_PHASE) {
        /* set status stage OK */
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "EP0 reqQueue in status phase %d \n", 0);
        dev->device.state = CH9_USB_STATE_CONFIGURED;
        CPS_UncachedWrite32(&dev->reg->USBR_EP_SEL, 0x00);
        CPS_UncachedWrite32(&dev->reg->USBR_EP_CMD, EP_CMD_REQ_CMPL | EP_CMD_ERDY); /* status phase */
        dev->ep0NextState = CH9_EP0_SETUP_PHASE;
        req->status = 0U;
        if (req->complete != NULL) {
            req->complete(&dev->ep0.ep, req);
        }
        ret = CDN_EOK;
    } else {
        /*---------------------------------------------------------------------- */

        /* for default endpoint data must be transfered in context of setup request */
        if (dev->ep0NextState != CH9_EP0_DATA_PHASE) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI,
                    "Error EP0 reqQueue in setup or unconnected phase %d \n", 0);
            ret = CDN_EPROTO;
        } else {
            CUSBD_Req ep0Req;
            /* it is assumed that ep0 doesn't enqueues requests */
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "Processing EP0 req in data phase %d \n", 0);
            ep0Req.dma = req->dma;
            ep0Req.length = req->length;
            ep0Req.zero = req->zero;
            ret = ep0transfer(dev, dev->ep0DataDirFlag, &ep0Req, (dev->ep0DataDirFlag > 0U) ? 0U : 1U);
            dev->request = req;
        }
    }
    return ret;
}

/**
 * handle ep request
 * @param dev pointer to driver object
 * @param epp pointer to endpoint object
 * @param req pointer to request object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t handleSwEpReq(CUSBD_PrivateData * dev, CUSBD_EpPrivate * epp, CUSBD_Req * req) {
    uint32_t ret = CDN_EOK;

    /* handle request based on ep-direction */
    if ((epp->ep.address & 0x80U) != 0U) {
        ret = handleSwEpInReq(dev, epp, req);
    } else {
        ret = handleSwEpOutReq(dev, epp, req);
    }
    return ret;
}

/**
 * handle stream request
 * @param dev pointer to driver object
 * @param epp pointer to endpoint object
 * @param req pointer to request object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t handleStreamEpReq(CUSBD_PrivateData * dev, CUSBD_EpPrivate * epp, CUSBD_Req * req) {
    uint32_t ret = CDN_EOK;
    CUSBD_Req *lastReq = getLastReq(epp);
    uint16_t lastReqStreamId = 0U;
    uint8_t lastRequestPending = 0U;

    if (lastReq != NULL) {
        lastReqStreamId = lastReq->streamId;
        lastRequestPending = lastReq->requestPending;
    }

    vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "EP%d-%s size(%d) streamId(%d) reqBuf(0x%X) lastRequestPending(%d)\r\n",
            (epp->ep.address & 0xF), ((epp->ep.address & 0xF0) == 0U) ? "RX" : "TX",
            req->length, req->streamId, req->dma, lastRequestPending);

    /* Enqueue this request in TRB buffer IF
     * There are no requests in Queue OR
     * The streamID of the lastReq is same as the current req AND last req is also queued in */
    if ((lastReq == NULL) ||
        ((lastReqStreamId == req->streamId) && (lastRequestPending == 0U))) {
        /* handle request based on ep-direction */
        if ((epp->ep.address & 0x80U) != 0U) {
            ret = handleSwEpInReq(dev, epp, req);
        } else {
            ret = handleSwEpOutReq(dev, epp, req);
        }
    } else {
        if (req->complete != NULL) {
            req->requestPending = 1;

            /* add request to transfer queue*/
            reqListAddTail(&(epp->reqListHead), req);
        } else {
            ret = CDN_EOPNOTSUPP;
        }
    }
    return ret;
}

/**
 * queue request to transfer
 * @param dev pointer to driver object
 * @param ep pointer to endpoint object
 * @param req pointer to request object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t reqQueue(CUSBD_PrivateData *dev, const CUSBD_Ep *ep, CUSBD_Req * req) {

    uint32_t ret = 0U;
    CUSBD_EpPrivate * epp = ep->epPrivate;

    /* set the actual number of bytes xfered to 0 */
    req->actual = 0;
    req->status = CDN_EINPROGRESS;

    vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "EP%d-%s req(0x%08X) buffer(0x%08X) size(0x%08X) \n",
            (ep->address & 0xF),
            ((ep->address & 0xF0) == 0U) ? "RX" : "TX",
            (uintptr_t) req,
            (uintptr_t) req->buf,
            req->length);

    if (ep->address == EP0_ADDRESS) {
        /* Handle EP0 requests */
        ret = handleEp0Req(dev, req);
    } else {
        /* check if endpoint enabled */
        if (epp->ep_state == CUSBD_EP_DISABLED) {
            ret = CDN_EPERM;
            req->status = CDN_EPERM;
        } else if (epp->ep_state == CUSBD_EP_STALLED) {
            if (req->complete != NULL) {
                req->requestPending = 1;

                /* add request to transfer queue*/
                reqListAddTail(&(epp->reqListHead), req);
            }

        } else if (epp->ep.maxStreams != 0U) {
            /* Handle stream requests */
            ret = handleStreamEpReq(dev, epp, req);
        } else {
            /* Handle all other non-EP0 requests */
            ret = handleSwEpReq(dev, epp, req);
        }
    }
    return ret;
}

/**
 * Remove request from transfer queue
 * @param ep pointer to endpoint object
 * @param req pointer to request object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t reqDequeue(CUSBD_Ep *ep, CUSBD_Req * req) {

    /* get extended object reference */
    CUSBD_EpPrivate *epp = ep->epPrivate;

    /* update status */
    req->status = CDN_ECANCELED;

    /* delete request from transfer queue*/
    reqListDeleteItem(&(epp->reqListHead), req);

    if (req->complete != NULL) {

        /* call complete callback with CDN_ECANCELED status */
        req->complete(ep, req);
    }
    return CDN_EOK;
}

/**
 * Set halt on selected endpoint
 * @param pD pointer to driver object
 * @param ep pointer to endpoint object
 * @param value 1 for setting, 0 for clearing halt
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t epSetHalt(CUSBD_PrivateData *pD, const CUSBD_Ep *ep, uint8_t value) {

    uint32_t ret = CDN_EOK;
    CUSBD_PrivateData* dev;
    CUSBD_EpPrivate *epp;

    vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "ep:%02X\n", ep->address);

    dev = pD;

    /* get extended endpoint object */
    epp = ep->epPrivate;

    if (epp->ep_state == CUSBD_EP_DISABLED) {
        ret = CDN_EPERM;
    }

    /* when endpoint isn't stalled */
    if (ret == CDN_EOK) {
        if (value > 0U) {
            /* set stall */
            ret = CUSBDMA_ChannelHandleStall(&dev->dmaController, epp->channel, 1U, CUSBD_DEFAULT_TIMEOUT);
            epp->ep_state = CUSBD_EP_STALLED;
        } else {
            CUSBD_Req *nextReq = epp->reqListHead;
            CUSBD_Req *headReq = nextReq;
            /* clear stall */
            epp->wedgeFlag = 0;
            ret = CUSBDMA_ChannelHandleStall(&dev->dmaController, epp->channel, 0U, CUSBD_DEFAULT_TIMEOUT);
            epp->ep_state = CUSBD_EP_ENABLED;
            /* queue any pending requests */
            while (nextReq != NULL) {
                if (nextReq->requestPending != 0U) {
                    nextReq->requestPending = 0U;
                    ret = handleSwEpReq(dev, epp, nextReq);
                }
                nextReq = nextReq->nextReq;
                if ((nextReq == headReq) || (ret != CDN_EOK)) {
                    break;
                }
            }
        }

        epp->transferPending = 0;
    }
    return ret;
}

/**
 * Set wedge feature on endpoint object
 * @param pD pointer to driver object
 * @param ep pointer to endpoint object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t epSetWedge(CUSBD_PrivateData *pD, const CUSBD_Ep * ep) {

    uint32_t res;

    vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "EP%d-%s epSetWedge",
            (ep->address & 0xF), ((ep->address & 0x80) != 0U) ? "TX" : "RX");

    /* set wedge field in driver object */
    ep->epPrivate->wedgeFlag = 1U;

    /* execute halt on endpoint*/
    res = epSetHalt(pD, ep, 1);

    return res;
}

/**
 * Flush FIFO for selected endpoint
 * @param pD pointer to driver object
 * @param ep pointer to endpoint object
 * @return CDN_EOK for success, error otherwise
 */
static uint32_t epFifoFlush(CUSBD_PrivateData *pD, const CUSBD_Ep * ep) {

    uint32_t ret = CDN_EOK;
    CUSBD_PrivateData* dev;

    vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "ep:%02X\n", ep->address);
    dev = pD;

    /* Do flush on endpoint hardware*/
    CPS_UncachedWrite32(&dev->reg->USBR_EP_SEL, (uint32_t) ep->address);
    CPS_UncachedWrite32(&dev->reg->USBR_EP_CMD, EP_CMD_DFLUSH);

    /* wait until operation is complete */
    ret = waitUntilBitCleared(&dev->reg->USBR_EP_CMD, EP_CMD_DFLUSH);
    return ret;
}

static const CUSBD_EpOps epOps = {
    CUSBD_EpEnable,
    CUSBD_EpDisable,
    CUSBD_EpSetHalt,
    CUSBD_EpSetWedge,
    CUSBD_EpFifoStatus,
    CUSBD_EpFifoFlush,
    CUSBD_ReqQueue,
    CUSBD_ReqDequeue
};

/**
 * private function, initializes default endpoint, main/ISR context
 * @param maxPacketSize max packet size
 * @return register
 */
static uint32_t buildEp0config(uint16_t maxPacketSize) {
    uint32_t reg = 0U;

    /* configure endpoint-0 */
    SET_EP_CONF_ENABLE(&reg);
    SET_EP_CONF_EPTYPE(&reg, USBRV_EP_CONTROL);
    SET_EP_CONF_MAXPKTSIZE(&reg, maxPacketSize);

    return reg;
}

/**
 * get max packet size for default endpoint
 * @param dev Pointer to druver object
 * @return max packet size
 */
static uint16_t getEp0MaxPacketSize(const CUSBD_PrivateData * dev) {
    uint16_t maxPacketSize = 0U;

    /* select max packet size for default endpoint*/
    switch (dev->device.speed) {
    /* max packet size is decided based on speed */
    case CH9_USB_SPEED_LOW:
        /* low speed */
        maxPacketSize = 8U;
        break;
    case CH9_USB_SPEED_FULL:
        /* full speed */
        maxPacketSize = 64U;
        break;
    case CH9_USB_SPEED_HIGH:
        /* high speed */
        maxPacketSize = 64U;
        break;
    case CH9_USB_SPEED_SUPER:
        /* super speed */
        maxPacketSize = 512U;
        break;
    default:
        /* default max packet size */
        maxPacketSize = 512U;
        break;
    }
    return maxPacketSize;
}

/**
 * initialize hardware of default endpoint
 * @param dev pointer to driver object
 */
static void initEp0Hw(CUSBD_PrivateData * dev) {

    uint16_t maxPacketSize = getEp0MaxPacketSize(dev);

    dev->ep0.ep.maxPacket = maxPacketSize;
    dev->ep0NextState = CH9_EP0_SETUP_PHASE;
    vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "EP0 maxPacket: %d\n", dev->ep0.ep.maxPacket);
}

/**
 * initialize hardware of device
 * @param dev pointer to driver object
 */
static void initDevHw(CUSBD_PrivateData * dev) {

    uint32_t dmult = 0U;
    uint32_t usbForceDisconnect = 0U;
    ssReg_t* reg = dev->reg;

    if ((dev->config.dmultEnabled > 0U) && (dev->deviceVersion < DEV_VER_V3)) {
        /* Enable global DMULT mode if supported and configured */
        dmult = USB_CONF_DMULT;
    }

    if (dev->config.forcedUsbMode == 1U) {
        /* Disconnect USB3 if mode if forced to 2 */
        usbForceDisconnect = USB_CONF_USB3DIS | USB_CONF_SFORCE_FS;
    } else if (dev->config.forcedUsbMode == 2U) {
        /* Disconnect USB3 if mode if forced to 2 */
        usbForceDisconnect = USB_CONF_USB3DIS;
    } else {
        /* Needed for MISRA compliance */
    }

    /* configure device */
    CPS_UncachedWrite32(&reg->USBR_CONF, USB_CONF_L1EN | dmult | usbForceDisconnect);

    if (dev->deviceVersion == DEV_VER_V3) {
        CPS_UncachedWrite32(&reg->DTRANS, dev->config.dmultEnabled);
    }

    /* clear all interrupts */
    CPS_UncachedWrite32(&dev->reg->USBR_ISTS, 0xFFFFFFFFU);

    /* enable interrupts */
    CPS_UncachedWrite32(&dev->reg->USBR_IEN, USB_IEN_UWRESIEN |
                        USB_IEN_UHRESIEN |
                        USB_IEN_DISIEN |
                        USB_IEN_CONIEN |
                        USB_IEN_CON2I |
                        USB_IEN_U1ENTIEN |
                        USB_IEN_U2ENTIEN |
                        USB_IEN_U2RESIEN |
                        USB_IEN_DIS2I);
}

/**
 * initialize endpoint
 * @param dev pointer to driver object
 * @param endpoint endpoint
 * @param dir direction
 * @return status
 */
static uint32_t initSwEp(CUSBD_PrivateData * dev, const CUSBD_EpConfig *endpoint, uint8_t dir) {

    uint8_t i;
    CUSBD_EpPrivate * epPriv;
    const CUSBD_EpConfig *ep = endpoint;

    for (i = 0U; i < 15U; i++) {
        if (ep[i].bufferingValue > 0U) {

            vDbgMsg(USBSSP_DBG_CUSBD, 1, " initSwEp Initializing EP%d-%s \n",
                    (i + 1U), (dir != 0U) ? "TX" : "RX");

            /* add endpoint to endpoint list */
            if (dir > 0U) {
                epPriv = &dev->ep_in_container[i];
            } else {
                epPriv = &dev->ep_out_container[i];
            }

            /* clear endpoint object - this will ALSO set reqListHead pointers to NULL */
            (void) memset(epPriv, 0, sizeof (CUSBD_EpPrivate));
            /* set endpoint address */
            epPriv->ep.address = ((dir > 0U) ? CH9_USB_EP_DIR_IN : 0U) | (uint8_t) (i + 1U);
            /* set MaxPacketSize */
            epPriv->ep.maxPacket = 1024U;
            /* set Max Burst */
            epPriv->ep.maxburst = 0U;
            /*set max stream */
            epPriv->ep.maxStreams = 0U;
            /*set MULT */
            epPriv->ep.mult = 0U;
            /* set endpoint pseudo-class public methods */
            epPriv->ep.ops = &epOps;
            /* set pointer to epPriv */
            epPriv->ep.epPrivate = epPriv;
            /*add endpoint to endpoint list */
            listAddTail(&epPriv->ep.epList, &dev->device.epList);
        } else {
            vDbgMsg(USBSSP_DBG_CUSBD, 1, " initSwEp Skipping EP%d-%s \n",
                    (i + 1U), (dir != 0U) ? "TX" : "RX");
        }
    }
    return CDN_EOK;
}

/**
 * Update reqAlloc field of driver object
 * @param dev
 * @param size
 */
static void updateReqAlloc(CUSBD_PrivateData* dev, uint16_t size) {

    uint32_t ret = CDN_EOK;

    /* update field*/
    dev->reqAlloc.length = size;
    dev->reqAlloc.dma = dev->setupPacketDma;
    dev->reqAlloc.complete = NULL;

    /* send request to hardware */
    ret = reqQueue(dev, &dev->ep0.ep, &dev->reqAlloc);
    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Could not reply to GET STATUS command\n", 0);
    }
}

/**
 * handle get status standard setup request
 * @param dev pointer to driver object
 * @param ctrl pointer to setup control request
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t getStatus(CUSBD_PrivateData * dev, CH9_UsbSetup *ctrl) {

    uint16_t size = ctrl->wLength;
    uint8_t recip = (ctrl->bmRequestType & CH9_REQ_RECIPIENT_MASK);
    uint32_t ret = CDN_EOK;
    /* parasoft-begin-suppress MISRA2012-RULE-11_4 " uintptr_t converted to 'uint32_t*', DRV-4385" */
    uint8_t * dmaBuf = (uint8_t *) dev->setupPacket;
    /* parasoft-end-suppress MISRA2012-RULE-11_4 */

    switch (recip) {
    /* recipient device */
    case CH9_USB_REQ_RECIPIENT_DEVICE:
        dev->status_value = cpuToLe16((uint16_t) dev->u2_value | (uint16_t) dev->u1_value | (uint16_t) 1U);
        /* write status in LE order */
        dmaBuf[0] = (uint8_t) dev->status_value;
        dmaBuf[1] = (uint8_t) ((dev->status_value >> 8) & (uint16_t) 0x00FF);
        updateReqAlloc(dev, size);
        break;

    /* recipient interface */
    case CH9_USB_REQ_RECIPIENT_INTERFACE:
        ctrl->wIndex = cpuToLe16(ctrl->wIndex);
        ctrl->wLength = cpuToLe16(ctrl->wLength);
        ctrl->wValue = cpuToLe16(ctrl->wValue);
        ret = dev->callbacks.setup(dev, ctrl);
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "getStatus res: %d\n", ret);
        break;

    /* recipient endpoint */
    case CH9_USB_REQ_RECIPIENT_ENDPOINT:
        CPS_UncachedWrite32(&dev->reg->USBR_EP_SEL, ctrl->wIndex);
        if ((CPS_UncachedRead32(&dev->reg->USBR_EP_STS) & EP_STS_STALL) > 0U) {
            dev->status_value = cpuToLe16(1U);
        } else {
            dev->status_value = cpuToLe16(0U);
        }
        /* write status in LE order */
        dmaBuf[0] = (uint8_t) dev->status_value;
        dmaBuf[1] = (uint8_t) ((dev->status_value >> 8) & (uint16_t) 0x00FF);
        updateReqAlloc(dev, size);
        break;

    default:
        ret = CDN_EINVAL;
        break;

    }

    return ret;
}

/**
 * handle U1 feature
 * @param dev pointer to driver object
 * @param set 1 for set, 0 for clear
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t handleFeatureRecDevU1(CUSBD_PrivateData * dev, uint8_t set) {

    uint32_t ret = CDN_EOK;
    CH9_UsbState state = dev->device.state;

    /* request only valid for configured device*/
    if (state != CH9_USB_STATE_CONFIGURED) {
        ret = CDN_EINVAL;
    }

    /* check actual speed*/
    if (dev->device.speed != CH9_USB_SPEED_SUPER) {
        ret = CDN_EINVAL;
    }

    if (ret == CDN_EOK) {

        /* set feature to hardware controller */
        uint32_t reg = CPS_UncachedRead32(&dev->reg->USBR_CONF);

        if (set > 0U) {
            dev->u1_value = 0x04U;
            reg |= USB_CONF_U1EN;
        } else {
            dev->u1_value = 0x00U;
            reg |= USB_CONF_U1DS;
        }

        /* write to hardware configuration register*/
        CPS_UncachedWrite32(&dev->reg->USBR_CONF, reg);
    }
    return ret;
}

/**
 * handle U2 feature
 * @param dev pointer to driver object
 * @param set 1 for set, 0 for clear
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t handleFeatureRecDevU2(CUSBD_PrivateData * dev, uint8_t set) {

    uint32_t ret = CDN_EOK;
    CH9_UsbState state = dev->device.state;

    /* request only valid for configured device*/
    if (state != CH9_USB_STATE_CONFIGURED) {
        ret = CDN_EINVAL;
    }
    /* check actual speed*/
    if (dev->device.speed != CH9_USB_SPEED_SUPER) {
        ret = CDN_EINVAL;
    }

    if (ret == CDN_EOK) {
        /* set feature to hardware controller */
        uint32_t reg = CPS_UncachedRead32(&dev->reg->USBR_CONF);
        if (set > 0U) {
            dev->u2_value = 0x08;
            reg |= USB_CONF_U2EN;
        } else {
            dev->u2_value = 0x00;
            reg |= USB_CONF_U2DS;
        }
        /* write to hardware configuration register*/
        CPS_UncachedWrite32(&dev->reg->USBR_CONF, reg);
    }
    return ret;
}

/**
 * Handle feature, recipient device, test mode
 * @param dev pointer to driver object
 * @param ctrl pointer to setup object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t handleFeatureRecDevTest(CUSBD_PrivateData * dev, const CH9_UsbSetup *ctrl) {

    uint32_t wIndex = ctrl->wIndex;
    uint32_t ret = CDN_EOK;

    if ((wIndex & 0xffU) != 0U) {
        ret = CDN_EINVAL;
    }

    if (ret == CDN_EOK) {
        uint8_t test_selector = (uint8_t) ((ctrl->wIndex >> 8) & 0x00FFU);
        uint32_t regSelec = 0U;
        switch (test_selector) {

        /* TEST J */
        case CH9_TEST_J:
            SET_USB_CMD_TMODE_SEL(&regSelec, USBRV_TM_TEST_J);
            break;

        /*test K*/
        case CH9_TEST_K:
            SET_USB_CMD_TMODE_SEL(&regSelec, USBRV_TM_TEST_K);
            break;

        /*test se0_NAK*/
        case CH9_TEST_SE0_NAK:
            SET_USB_CMD_TMODE_SEL(&regSelec, USBRV_TM_SE0_NAK);
            break;

        /* packet test */
        case CH9_TEST_PACKET:
            SET_USB_CMD_TMODE_SEL(&regSelec, USBRV_TM_TEST_PACKET);
            break;

        case CH9_TEST_FORCE_EN:
            break;

        default:;
            break;
        }
        CPS_UncachedWrite32(&dev->reg->USBR_CMD, regSelec);
    }
    return ret;
}

/**
 * handle set/clear feature standard setup request for device recipient
 * @param dev pointer to driver object
 * @param ctrl pointer to setup object
 * @param set 1 for setting feature, 0 for clearing feature
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t handleFeatureRecipientDevice(CUSBD_PrivateData * dev, const CH9_UsbSetup *ctrl, uint8_t set) {

    uint32_t ret = CDN_EOK;
    uint32_t wValue = ctrl->wValue;

    switch (wValue) {
    case CH9_USB_FS_DEVICE_REMOTE_WAKEUP:
        break;
    /*
     * 9.4.1 says only only for SS, in AddressState only for
     * default control pipe
     */

    /* handle U1 feature */
    case CH9_USB_FS_U1_ENABLE:
        ret = handleFeatureRecDevU1(dev, set);
        break;

    /* handle U2 feature */
    case CH9_USB_FS_U2_ENABLE:
        ret = handleFeatureRecDevU2(dev, set);
        break;

    case CH9_USB_FS_LTM_ENABLE:
        ret = CDN_EINVAL;
        break;

    /* handle test mode */
    case CH9_USB_FS_TEST_MODE:
        ret = handleFeatureRecDevTest(dev, ctrl);
        break;

    default:
        ret = CDN_EINVAL;
        break;
    }
    return ret;
}

/**
 * handle set/clear feature standard setup request for recipient interface
 * @param dev pointer to driver object
 * @param ctrl pointer to setup object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t handleFeatureRecipientInterface(CUSBD_PrivateData * dev, const CH9_UsbSetup *ctrl) {

    uint32_t wIndex;
    uint32_t wValue;
    uint32_t ret = CDN_EOK;

    wValue = ctrl->wValue;
    wIndex = ctrl->wIndex;

    switch (wValue) {
    /* for suspend selector */
    case CH9_USB_FS_FUNCTION_SUSPEND:
        if ((wIndex & CH9_USB_SF_LOW_PWR_SUSP_STATE) > 0U) {
            /* XXX enable Low power suspend */
            dev->suspend = (ctrl->wIndex & 0x0100U);
        }

        if ((wIndex & CH9_USB_SF_REMOTE_WAKE_ENABLED) > 0U) {
            /* XXX enable remote wakeup */
            break;
        }
        break;
    default:
        ret = CDN_EINVAL;
        break;
    }
    return ret;
}

/**
 * handle set/clear feature for endpoint recipient
 * @param dev pointer to driver object
 * @param ctrl pointer to setup object
 * @param set 1 for setting stall, 0 for clearing stall
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t handleFeatureRecipientEndpoint(CUSBD_PrivateData * dev, const CH9_UsbSetup *ctrl, uint8_t set) {

    CUSBD_EpPrivate * epp;
    uint32_t ret = CDN_EINVAL;
    uint32_t tempSel;

    uint8_t epAddress = (uint8_t) ctrl->wIndex;
    uint16_t wValue = ctrl->wValue;
    uint8_t epIndex = USBD_EPNUM_FROM_EPADDR(epAddress);
    uint8_t epDir = USBD_EPDIR_FROM_EPADDR(epAddress);

    /* get endpoint from endpoint container */
    if (epDir > 0U) {
        epp = &dev->ep_in_container[epIndex - 1U];
    } else {
        epp = &dev->ep_out_container[epIndex - 1U];
    }

    /* check if endpoint is available at all */
    if (epp->ep.ops != NULL) {
        ret = CDN_EOK;
    }

    if (ret == CDN_EOK) {

        tempSel = CPS_UncachedRead32(&dev->reg->USBR_EP_SEL);
        CPS_UncachedWrite32(&dev->reg->USBR_EP_SEL, (uint32_t) epp->ep.address);

        switch (wValue) {
        case CH9_USB_FS_ENDPOINT_HALT:
            if (set > 0U) {
                /* issue set stall to hardware controller */
                ret = epSetHalt(dev, &epp->ep, 1);
            } else {
                if (epp->wedgeFlag == 0U) {
                    /* handle clear feature*/
                    ret = epSetHalt(dev, &epp->ep, 0);
                } else {
                    /* do nothing */
                    vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "CAN'T CLEAR ENDPOINT STALL %c\n", ' ');
                }
            }    /*else (set) */
            break;

        default:
            CPS_UncachedWrite32(&dev->reg->USBR_EP_SEL, tempSel);
            ret = CDN_EINVAL;
            break;
        }
    }
    if (ret == CDN_EOK) {
        CPS_UncachedWrite32(&dev->reg->USBR_EP_SEL, tempSel);
    }
    return ret;
}

/**
 * handle set/clear feature standard request
 * @param dev pointer to driver object
 * @param ctrl pointer to setup object
 * @param set 1 for set, 0 for clear
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t handleFeature(CUSBD_PrivateData * dev, CH9_UsbSetup const *ctrl, uint8_t set) {

    uint32_t stat;

    switch (ctrl->bmRequestType & CH9_REQ_RECIPIENT_MASK) {
    /* recipient device */
    case CH9_USB_REQ_RECIPIENT_DEVICE:

        stat = handleFeatureRecipientDevice(dev, ctrl, set);
        break;

    /*recipient interface */
    case CH9_USB_REQ_RECIPIENT_INTERFACE:
        stat = handleFeatureRecipientInterface(dev, ctrl);
        break;

    /*recipient endpoint*/
    case CH9_USB_REQ_RECIPIENT_ENDPOINT:
        stat = handleFeatureRecipientEndpoint(dev, ctrl, set);
        break;
    default:
        stat = CDN_EINVAL;
        break;
    };

    return stat;
}

/**
 * handle set address standard request
 * @param dev pointer to driver object
 * @param ctrl pointer to setup object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t setAddress(CUSBD_PrivateData * dev, CH9_UsbSetup *ctrl) {

    uint32_t ret = CDN_EOK;
    CH9_UsbState state = dev->device.state;
    uint16_t addr;

    addr = ctrl->wValue;

    /* check if device address is within correct range */
    if (addr > 0x007FU) {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Invalid device address %d\n", addr);
        ret = CDN_EINVAL;
    }

    /* check if device is in correct state */
    if (ret == CDN_EOK) {
        if (state == CH9_USB_STATE_CONFIGURED) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Trying to set address when configured %c\n", ' ');
            ret = CDN_EINVAL;
        }
    }

    if (ret == CDN_EOK) {

        /* set device address in controller */
        CPS_UncachedWrite32(&dev->reg->USBR_CMD, (((uint32_t) ctrl->wValue & 0x007FU) << 1) | 0x00000001U);

        if (addr > 0U) {
            dev->device.state = CH9_USB_STATE_ADDRESS;
        } else {
            dev->device.state = CH9_USB_STATE_DEFAULT;
        }
    }

    return ret;
}

/**
 * Handle set configuration request for device in addresses state
 * @param dev pointer to driver object
 * @param ctrl pointer to setup object
 * @param confValue configuration value
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t setConfigStateAddressed(CUSBD_PrivateData * dev, CH9_UsbSetup *ctrl, uint8_t confValue) {

    uint32_t ret;

    /* create setup object in CPU endian */
    ctrl->wIndex = cpuToLe16(ctrl->wIndex);
    ctrl->wLength = cpuToLe16(ctrl->wLength);
    ctrl->wValue = cpuToLe16(ctrl->wValue);
    if (confValue > 0U) {
        /* set configuration in hardware controller */
        CPS_UncachedWrite32(&dev->reg->USBR_CONF, USB_CONF_CFGSET);
    }
    ret = dev->callbacks.setup(dev, ctrl);
    if ((confValue > 0U) && (ret == CDN_EOK)) {
        /* check if USB controller has resources for required configuration */
        /* and return error if no */
        if ((CPS_UncachedRead32(&dev->reg->USBR_STS) & USB_STS_MEM_OV) > 0U) {
            ret = CDN_EINVAL;
        }
    }
    return ret;
}

/**
 * handle set configuration setup request
 * @param dev pointer to driver object
 * @param ctrl pointer to setup object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t setConfig(CUSBD_PrivateData * dev, CH9_UsbSetup *ctrl) {

    uint8_t confValue = (uint8_t) (ctrl->wValue & 0x00FFU);
    uint32_t ret;

    switch (dev->device.state) {
    /* state default: return error */
    case CH9_USB_STATE_DEFAULT:
        ret = CDN_EINVAL;
        break;

    /* addressed state */
    case CH9_USB_STATE_ADDRESS:
        ret = setConfigStateAddressed(dev, ctrl, confValue);
        break;

    /* configured state */
    case CH9_USB_STATE_CONFIGURED:
        ctrl->wIndex = cpuToLe16(ctrl->wIndex);
        ctrl->wLength = cpuToLe16(ctrl->wLength);
        ctrl->wValue = cpuToLe16(ctrl->wValue);
        ret = dev->callbacks.setup(dev, ctrl);
        if (ret == CDN_EOK) {
            /* SET_CONFIGURATION(0): unconfigure hardware controller */
            if (confValue == 0U) {
                CPS_UncachedWrite32(&dev->reg->USBR_CONF, USB_CONF_CFGRST);
                dev->device.state = CH9_USB_STATE_ADDRESS;
            } else {
                CPS_UncachedWrite32(&dev->reg->USBR_CONF, USB_CONF_CFGSET);
                if ((CPS_UncachedRead32(&dev->reg->USBR_STS) & USB_STS_MEM_OV) > 0U) {
                    ret = CDN_EINVAL;
                }
            }
        }
        break;

    default:
        ret = CDN_EINVAL;
        break;
    }

    return ret;
}

/**
 * handle set sel request
 * @param dev pointer to driver object
 * @param ctrl pointer to setup object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t setSel(CUSBD_PrivateData * dev, CH9_UsbSetup const *ctrl) {

    uint32_t ret = CDN_EOK;

    /* check current state */
    if (dev->device.state == CH9_USB_STATE_DEFAULT) {
        ret = CDN_EINVAL;
    }

    /* check if setup has correct parameters */
    if (ret == CDN_EOK) {
        if (ctrl->wLength != 6U) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "Set SEL should be 6 bytes, got %d\n", ctrl->wLength);
            ret = CDN_EINVAL;
        }
    }

    if (ret == CDN_EOK) {
        CUSBD_Req ep0Req;
        ep0Req.dma = (uintptr_t) dev->setupPacketDma;
        ep0Req.length = ctrl->wLength;
        ep0Req.zero = 0U;

        /* start transfer on default endpoint for 1 byte of data*/
        dev->reqAlloc.complete = NULL;
        dev->reqAlloc.length = ctrl->wLength;
        dev->reqAlloc.actual = 0;
        dev->reqAlloc.status = CDN_EINPROGRESS;
        dev->request = &dev->reqAlloc;

        ret = ep0transfer(dev, 0U, &ep0Req, 1U);
    }

    return ret;
}

/**
 * handle set isoch delay
 * @param dev pointer to driver object
 * @param ctrl pointer to setup object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t setIsochDelay(CUSBD_PrivateData * dev, CH9_UsbSetup const *ctrl) {

    uint32_t ret = CDN_EOK;

    /* check if setup has correct parameters */
    if ((ctrl->wIndex > 0U) || (ctrl->wLength > 0U)) {
        ret = CDN_EINVAL;
    } else {
        /* update isoch_delay private member of driver object */
        dev->isoch_delay = ctrl->wValue;
    }

    return ret;
}

/**
 * handle setup request
 * @param dev pointer to driver object
 * @param ctrl pointer to setup object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t setupDevStand(CUSBD_PrivateData * dev, CH9_UsbSetup *ctrl) {

    uint32_t res = CDN_EOK;

    switch (ctrl->bRequest) {

    /* get status request */
    case CH9_USB_REQ_GET_STATUS:
        res = getStatus(dev, ctrl);
        break;

    /* clear feature request */
    case CH9_USB_REQ_CLEAR_FEATURE:
        res = handleFeature(dev, ctrl, 0U);
        break;

    /* set feature request */
    case CH9_USB_REQ_SET_FEATURE:
        res = handleFeature(dev, ctrl, 1U);
        break;

    /* set address request */
    case CH9_USB_REQ_SET_ADDRESS:
        res = setAddress(dev, ctrl);
        break;

    /* set configuration request */
    case CH9_USB_REQ_SET_CONFIGURATION:
        res = setConfig(dev, ctrl);
        break;

    /* set sel request */
    case CH9_USB_REQ_SET_SEL:
        res = setSel(dev, ctrl);
        break;

    /* set iso delay request */
    case CH9_USB_REQ_ISOCH_DELAY:
        res = setIsochDelay(dev, ctrl);
        break;

    default:
        /* handle class or vendor specific request */
        ctrl->wIndex = cpuToLe16(ctrl->wIndex);
        ctrl->wLength = cpuToLe16(ctrl->wLength);
        ctrl->wValue = cpuToLe16(ctrl->wValue);
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "res: %08X\n", res);
        res = dev->callbacks.setup(dev, ctrl);
        break;
    }
    return res;
}

/**
 * handle setup request
 * @param dev pointer to driver object
 * @param ctrl pointer to setup object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t setupDev(CUSBD_PrivateData * dev, CH9_UsbSetup *ctrl) {

    uint32_t res = 0U;
    uint16_t size = ctrl->wLength;

    /* check if data phase exists*/
    if (size > 0U) {
        dev->ep0NextState = CH9_EP0_DATA_PHASE;
    } else {
        dev->ep0NextState = CH9_EP0_STATUS_PHASE;
    }

    /* set data direction flag */
    if ((ctrl->bmRequestType & CH9_USB_DIR_DEVICE_TO_HOST) > 0U) {
        dev->ep0DataDirFlag = 1U;
    } else {
        dev->ep0DataDirFlag = 0U;
    }

    /* check if request is a standard request */
    if ((ctrl->bmRequestType & CH9_USB_REQ_TYPE_MASK) == CH9_USB_REQ_TYPE_STANDARD) {
        res = setupDevStand(dev, ctrl);
    } else {
        /* delegate setup for class and vendor specific requests */
        ctrl->wIndex = cpuToLe16(ctrl->wIndex);
        ctrl->wLength = cpuToLe16(ctrl->wLength);
        ctrl->wValue = cpuToLe16(ctrl->wValue);
        res = dev->callbacks.setup(dev, ctrl);
    }

    return res;
}

/**
 * handle status phase of control transfer
 * @param dev pointer to driver object
 */
static void handleEp0IrqStatus(CUSBD_PrivateData* dev) {
    if (dev->ep0NextState == CH9_EP0_STATUS_PHASE) {
        /* set status stage OK */
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "Handle status phase %d\n", 0);
        CPS_UncachedWrite32(&dev->reg->USBR_EP_CMD, EP_CMD_REQ_CMPL | EP_CMD_ERDY); /* status phase */
        dev->ep0NextState = CH9_EP0_SETUP_PHASE;
        /* check if any ep0 request waits for completion, If YES complete it */
        if (dev->request != NULL) {
            if ((dev->request->length == 0U) && (dev->request->status == CDN_EINPROGRESS)) {
                vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "Call callback%s\n", " ");
                dev->request->status = 0U;
                /* call complete callback if defined */
                if (dev->request->complete != NULL) {
                    dev->request->complete(&dev->ep0.ep, dev->request);
                }
            }
        }
    }
}

/**
 * handle of setup phase of control transfer
 * @param dev pointer to device object
 * @param ep_sts pointer to endpoint status ep_sts register
 * @param channel DMA channel
 * @return CDN_EOK on success, error otherwise
 */
static uint32_t handleEp0IrqSetup(CUSBD_PrivateData* dev, uint32_t * ep_sts, CUSBDMA_DmaChannel *channel) {
    uint32_t ret = CDN_EOK;
    CH9_UsbSetup ctrl;
    dev->request = NULL;

    /* update the state of DMA channel internal data structures */
    cusbdmaProcessDataXferInt(dev, channel);

    CPS_UncachedWrite32(&dev->reg->USBR_EP_SEL, 0x00);

    /* copy USB setup object */
    ctrl.bRequest = dev->setupPacket->bRequest;
    ctrl.bmRequestType = dev->setupPacket->bmRequestType;
    ctrl.wIndex = le16ToCpu(dev->setupPacket->wIndex);
    ctrl.wLength = le16ToCpu(dev->setupPacket->wLength);
    ctrl.wValue = le16ToCpu(dev->setupPacket->wValue);

    /* clear flags */
    *ep_sts &= ~(EP_STS_SETUP | EP_STS_ISP | EP_STS_IOC);


#ifdef TINYUSB_INTEGRATION /* Call the setup callback function for TinyUSB here */
    /* call the customer setup callback function */
    dev->callbacks.setup(dev, &ctrl);
#else
    /* parse setup request */
    ret = setupDev(dev, &ctrl);
    vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "res: %08X\n", ret);
    CPS_UncachedWrite32(&dev->reg->USBR_EP_SEL, 0x00);

    /* -------- Patch for Linux SET_CONFIGURATION(1) handling -------------- */
    if (ret == CUSBD_INACTIVITY_TMOUT) {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Respond delayed not finished yet%s\n", " ");
        dev->ep0NextState = CH9_EP0_STATUS_PHASE;
    } else {
        /*---------------------------------------------------------------------- */

        if (ret != CDN_EOK) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "setupDev returned error %x\n", ret);
            CPS_UncachedWrite32(&dev->reg->USBR_EP_CMD, EP_CMD_SSTALL);
            CPS_UncachedWrite32(&dev->reg->USBR_EP_CMD, EP_CMD_REQ_CMPL | EP_CMD_ERDY);
            dev->ep0NextState = CH9_EP0_SETUP_PHASE;
        } else {
            handleEp0IrqStatus(dev);
        }
    }
#endif
    return ret;
}

/**
 * Handle IOC interrupt on default endpoint
 * @param dev pointer to driver object
 * @param channel DMA channel
 */
static void handleEp0IrqIoc(CUSBD_PrivateData* dev, CUSBDMA_DmaChannel *channel) {

    (void) cusbdmaProcessDataXferInt(dev, channel);

    if (dev->request != NULL) {
        /* check if actual transfer is done */
        if (dev->request->actual >= dev->request->length) {
            vDbgMsg(USBSSP_DBG_CUSBD_ISR, DBG_FYI, "No more data to transfer on ep0%c\n", ' ');
        }
        /* this gets sent even for short packet */
        dev->request->status = 0U;
        if (dev->request->complete != NULL) {
            dev->request->complete(&dev->ep0.ep, dev->request);
        }
	    if (dev->request->deferStatusStage)  /* please put it above the line dev->ep0NextState = CH9_EP0_SETUP_PHASE; */
		    return;
    
    }

    dev->ep0NextState = CH9_EP0_SETUP_PHASE;
    CPS_UncachedWrite32(&dev->reg->USBR_EP_SEL, 0x00U);
    CPS_UncachedWrite32(&dev->reg->USBR_EP_CMD, EP_CMD_REQ_CMPL | EP_CMD_ERDY);
}


#ifndef TINYUSB_INTEGRATION
/* Pure Cadence driver; no class driver  */
/**
 * handle event generated on default endpoint
 * @param dev pointer to driver object
 * @param dirFlag default endpoint direction: 0 for OUT endpoint, 1 for IN
 * @return status
 */
static uint32_t handleEp0Irq(CUSBD_PrivateData* dev, uint8_t dirFlag) {

    uint32_t ep_sts;
    uint8_t dir = (dirFlag > 0U) ? 0x80U : 0x00U;
    CUSBDMA_DmaChannel *channel = (dir > 0U) ? dev->ep0DmaChannelIn : dev->ep0DmaChannelOut;
    uint32_t ret = CDN_EOK;

    CPS_UncachedWrite32(&dev->reg->USBR_EP_SEL, dir);
    ep_sts = CPS_UncachedRead32(&dev->reg->USBR_EP_STS);

    /* clear interrupts */
    CPS_UncachedWrite32(&dev->reg->USBR_EP_STS, ep_sts);

    /* Setup packet completion */
    if ((ep_sts & EP_STS_SETUP) > 0U) {
        /* for ep0 setup IRQ overrides IOC and ISP */
        ret = handleEp0IrqSetup(dev, &ep_sts, channel);
        ep_sts &= ~(EP_STS_IOC | EP_STS_ISP | EP_STS_TRBERR);
    } else {

        /* should be called only when data phase exists */
        if (((ep_sts & EP_STS_IOC) > 0U) || ((ep_sts & EP_STS_ISP) > 0U)) {
            handleEp0IrqIoc(dev, channel);
            ep_sts &= ~(EP_STS_IOC | EP_STS_ISP | EP_STS_TRBERR);
        } else {
            if ((ep_sts & EP_STS_DESCMIS) > 0U) {

                /* check if setup packet came in */
                /* setup always come on OUT direction */
                if ((dir == 0U) && (dev->ep0NextState == CH9_EP0_SETUP_PHASE)) {
                    CUSBD_Req ep0Req;
                    ep0Req.dma = (uintptr_t) dev->setupPacketDma;
                    ep0Req.length = (uint32_t) (sizeof (CH9_UsbSetup));
                    ep0Req.zero = 0U;
                    ret = ep0transfer(dev, 0U, &ep0Req, 0U);
                    ep_sts &= ~(EP_STS_DESCMIS | EP_STS_TRBERR);
                }
            }
        }

        /* check TRB ERROR flag */
        if ((ep_sts & EP_STS_TRBERR) > 0U) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "Handle TRB error trb_addr: %x \n", CPS_UncachedRead32(&dev->reg->USBR_EP_TRADDR));
            /* trigger channel, this will trigger only if valid trb is queued */
        }

    }
    return ret;
}

#else
/**
 * handle event generated on default endpoint
 * @param dev pointer to driver object
 * @param dirFlag default endpoint direction: 0 for OUT endpoint, 1 for IN
 * @return status
 */
static uint32_t handleEp0Irq(CUSBD_PrivateData* dev, uint8_t dirFlag, uint32_t ep_sts ) {

    uint8_t dir = (dirFlag > 0U) ? 0x80U : 0x00U;
    CUSBDMA_DmaChannel *channel = (dir > 0U) ? dev->ep0DmaChannelIn : dev->ep0DmaChannelOut;
    uint32_t ret = CDN_EOK;


    /* Setup packet completion */
    if ((ep_sts & EP_STS_SETUP) > 0U) {
        /* for ep0 setup IRQ overrides IOC and ISP */
        ret = handleEp0IrqSetup(dev, &ep_sts, channel);
        ep_sts &= ~(EP_STS_IOC | EP_STS_ISP | EP_STS_TRBERR);
    } else {

        /* should be called only when data phase exists */
        if (((ep_sts & EP_STS_IOC) > 0U) || ((ep_sts & EP_STS_ISP) > 0U)) {
            handleEp0IrqIoc(dev, channel);
            ep_sts &= ~(EP_STS_IOC | EP_STS_ISP | EP_STS_TRBERR);
        } else {
            if ((ep_sts & EP_STS_DESCMIS) > 0U) {
                /* check if setup packet came in */
                /* setup always come on OUT direction */
                if ((dir == 0U) && (dev->ep0NextState == CH9_EP0_SETUP_PHASE)) {
                    CUSBD_Req ep0Req;
                    ep0Req.dma = (uintptr_t) dev->setupPacketDma;
                    ep0Req.length = (uint32_t) (sizeof (CH9_UsbSetup));
                    ep0Req.zero = 0U;
                    ret = ep0transfer(dev, 0U, &ep0Req, 0U);
                    ep_sts &= ~(EP_STS_DESCMIS | EP_STS_TRBERR);
                }
            }
        }

        /* check TRB ERROR flag */
        if ((ep_sts & EP_STS_TRBERR) > 0U) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "Handle TRB error trb_addr: %x \n", CPS_UncachedRead32(&dev->reg->USBR_EP_TRADDR));
            /* trigger channel, this will trigger only if valid trb is queued */
        }

    }
    return ret;
}
#endif /* TINYUSB_INTEGRATION */
/**********************************************************************
* API methods
**********************************************************************/

/**
 * Obtain the private memory size required by the driver
 * @param[in] config driver/hardware configuration required
 * @param[out] sysReqCusbd returns the size of memory allocations required
 * @return 0 on success (requirements structure filled)
 * @return CDN_ENOTSUP if configuration cannot be supported due to driver/hardware constraints
 */
uint32_t CUSBD_Probe(const CUSBD_Config* config,
                     CUSBD_SysReq*       sysReqCusbd) {

    uint32_t ret = CUSBD_ProbeSF(config, sysReqCusbd);

    if (ret == CDN_EOK) {
        if ((config->didRegPtr == 0U) || (config->ridRegPtr == 0U)) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Warning! DID, RID register pointers are not set.\n", 0);
        } else {
            /* parasoft-begin-suppress MISRA2012-RULE-11_4 "uintptr_t converted to 'uint32_t *', DRV-4273" */
            uint32_t did = CPS_UncachedRead32((uint32_t*) (config->didRegPtr));
            uint32_t rid = CPS_UncachedRead32((uint32_t*) (config->ridRegPtr));
            /* parasoft-end-suppress MISRA2012-RULE-11_4 */

            if ((did != CUSBD_DID_VAL) &&
                (rid != CUSBD_RID_VAL)) {
                ret = CDN_EPERM;
            }
        }
    }

    if (ret == CDN_EOK) {

        CUSBDMA_Config dmaConfig;
        CUSBDMA_SysReq sysReqCusbdma;

        (void) memset((void *) &dmaConfig, 0, sizeof (CUSBDMA_Config));
        dmaConfig.regBase = config->regBase;

        /* check DMA controller memory requirements */
        sysReqCusbdma.trbMemSize = (uint32_t) 0; /* Deprecated */
        sysReqCusbdma.privDataSize = (uint32_t) sizeof (CUSBDMA_DmaController);

        /* Device memory */
        sysReqCusbd->privDataSize = (uint32_t)sizeof (CUSBD_PrivateData);

        /* + 8 is a length of setup packet */
        sysReqCusbd->trbMemSize = sysReqCusbdma.trbMemSize + 8U;
    }

    return ret;
}

#ifdef CUSBDSS_DMA_ACCESS_NON_SECURE
/**
 * Configure DMA_AXI_CTRL for non secure access
 * @param reg: Pointer the the controller registers
 */
static void setAXIAccessNonSecure(ssReg_t* reg) {
    uint32_t regval = CPS_UncachedRead32(&reg->DMA_AXI_CTRL);

    /* set MARPROT bits to non-secure access */
    regval = SET_DMA_AXI_CTRL_MARPROT(regval, DMA_AXI_CTRL_NON_SECURE);

    /* set MAWPROT bits to non-secure access */
    regval = SET_DMA_AXI_CTRL_MAWPROT(regval, DMA_AXI_CTRL_NON_SECURE);
    CPS_UncachedWrite32(&reg->DMA_AXI_CTRL, regval);
}
#endif

/**
 * Memory initialisation
 * @param[in] pD pointer to driver object
 * @return CDN_EOK on success, error otherwise
 */
static uint32_t ramInit(CUSBD_PrivateData* pD) {

    CUSBD_PrivateData * dev = pD;
    uint32_t ret = CDN_EOK;

    /*
        The on-chip memory initialization requirements:
        - It should be performed as a first SW operation after POR
        - It must be performed before any Endpoint configurations ( any EP_XXX register)
        - it must be performed before USB_CONF.DEVEN (VBUS enable) is set
     */

    /* wait until USB_STS.IN_RST is '1' (reset for USB clock domain is finished) */
    ret = waitUntilBitSet(&(dev->reg->USBR_STS), USB_STS_IN_RST);

    if (ret == CDN_EOK) {
        /* set USB_PWR.FAST_REG_ACCESS bit (to ensure that USB clock is running even without VBUS) */
        CPS_UncachedWrite32(&dev->reg->USBR_PWR, USB_PWR_FAST_REG_ACCESS);

        /* wait until USB_PWR.FAST_REG_ACCESS_STAT is '1' */
        ret = waitUntilBitSet(&(dev->reg->USBR_PWR), USB_PWR_FAST_REG_ACCESS_STAT);
    }

    if (ret == CDN_EOK) {
        /* set BUF_ADDR register (it addresses individual words, not Bytes) to max address for attached memory . */
        /* In case of TI we have 4 * 3328 of 32bit words, so max address is 13312 (0x3400). */
        /* So we need to write 0x3400 to BUF_ADDR */
        CPS_UncachedWrite32(&dev->reg->USBR_BUF_ADDR, 0x000033FF);

        /* BUF_DATA can be left as 0x00000000 (RSTV) then whole memory will be initialized to 0 */
        CPS_UncachedWrite32(&dev->reg->USBR_BUF_DATA, 0x00000000);

        /* set BUF_CTRL.BUF_CMD_IWD (Incremental write down) bit + BUF_CMD_SET (one write). */
        /* This will start internal operation of writing all memory cells (starting from BUFF_ADDR down to 0) */
        /* with value prepared in BUF_DATA. For TI memory initialization should take about 27 us. */
        CPS_UncachedWrite32(&dev->reg->USBR_BUF_CTRL, BUF_CTRL_BUF_CMD_SET | BUF_CTRL_BUF_CMD_IWD);

        /* wait until BUF_CTRL.BUF_CMD_STS is '0' it means that memory initialization was finished */
        ret = waitUntilBitCleared(&(dev->reg->USBR_BUF_CTRL), BUF_CTRL_BUF_CMD_STS);
    }

    return ret;
}

/**
 * Initialize endpoints
 * @param dev pointer do driver object
 * @return CDN_EOK on success, error otherwise
 */
static inline uint32_t initConfigEpHw(const CUSBD_PrivateData* dev) {

    uint32_t ret = CDN_EOK;
    ssReg_t *cusbdReg = dev->reg;
    uint8_t i;

    /* initialize buffering value for all endpoints */
    for (i = 0U; i < 30U; i++) {
        const CUSBD_EpConfig *ep;
        uint8_t epIndex = 0U;

        /* get proper endpoint object */
        if (i < 15U) {
            epIndex = i;
            ep = &dev->config.epIN[epIndex];
        } else {
            epIndex = i - 15U;
            ep = &dev->config.epOUT[epIndex];
            epIndex |= 0x80U;
        }
        ++epIndex; /* ep_sel requires incrementation */

        /* first check if endpoint is available */
        if (ep->bufferingValue > 0U) {
            uint32_t epCfgReg = 0U;
            CPS_UncachedWrite32(&(cusbdReg->USBR_EP_SEL), epIndex);
            SET_EP_CONF_ENABLE(&epCfgReg);
            SET_EP_CONF_MAXPKTSIZE(&epCfgReg, USB_SS_MAX_PACKET_SIZE);
            SET_EP_CONF_BUFFERING(&epCfgReg, ep->bufferingValue - 1U);
            CPS_UncachedWrite32(&(cusbdReg->USBR_EP_CFG), epCfgReg);
        }
    }

    /* check if hardware has enough resources for required configuration */
    CPS_UncachedWrite32(&(cusbdReg->USBR_CONF), USB_CONF_CFGSET);
    /* and return error if resources problem */
    if (GET_USB_STS_MEM_OV(CPS_UncachedRead32(&cusbdReg->USBR_STS)) > 0U) {
        ret = CDN_ENOTSUP;
    }
    return ret;
}

/**
 * Initialize DMA module
 * @param dev pointer to driver object
 * @param dmaConfig pointer to DMA confuiguration structure
 * @return CDN_EOK if success, error code elsewhere
 */
static inline uint32_t initConfigDMA(CUSBD_PrivateData * dev, CUSBDMA_Config * dmaConfig) {

    uint32_t ret = CDN_EOK;

    /* set endpoint configuration bitmask for dmult mode */
    if (dev->deviceVersion < DEV_VER_V3) {
        if (dev->config.dmultEnabled > 0U) {
            dmaConfig->dmaModeRx = 0xFFFF;
            dmaConfig->dmaModeTx = 0xFFFF;
        } else {
            dmaConfig->dmaModeRx = 0x0000;
            dmaConfig->dmaModeTx = 0x0000;
        }
    } else {
        /* For Device V3:  Bits[15:0]: Control DMULT enable for EP-Out[15:0]
         *                 Bits[31:16]: Control DMULT enable for EP-In[15:0] */
        dmaConfig->dmaModeRx = (uint16_t) (dev->config.dmultEnabled & 0xFFFFU);
        dmaConfig->dmaModeTx = (uint16_t) ((dev->config.dmultEnabled >> 16U) & 0xFFFFU);
    }

    /* initialize DMA module */
    ret = CUSBDMA_Init(&(dev->dmaController), dmaConfig);

    return ret;
}

/**
 * Initialize device object
 * @param dev pointer to driver object
 * @param dmaConfig pointer to DMA configuration object
 * @return CDN_EOK if no error, Error code elsewhere
 */
static inline uint32_t initConfigDev(CUSBD_PrivateData * dev, CUSBDMA_Config * dmaConfig) {

    uint32_t ret = CDN_EOK;
    uint32_t deviceVersion = CPS_UncachedRead32(&dev->reg->USBR_CAP6);

    /* if resources available, unconfigure device by now. It will be configured */
    /* in SET_CONFIG context during enumeration */
    CPS_UncachedWrite32(&dev->reg->USBR_CONF, USB_CONF_CFGRST | USB_CONF_SWRST);

    dev->deviceVersion = GET_USBR_CAP6_DEV_BASE_VER(deviceVersion);

    /* set device general device state to initial state */
    dev->device.speed = CH9_USB_SPEED_UNKNOWN;
    dev->device.maxSpeed = CH9_USB_SPEED_SUPER;
    dev->device.state = CH9_USB_STATE_DEFAULT;
#ifdef DEBUG
    (void) strncpy(dev->device.name, SS_DEV_NAME, sizeof (SS_DEV_NAME));
#endif

    /* Creating DMA Controller */
    dev->dmaDrv = CUSBDMA_GetInstance();

    dmaConfig->regBase = dev->config.regBase;
    dmaConfig->epMemRes = dev->config.epMemRes;

    ret = initConfigDMA(dev, dmaConfig);

    return ret;
}

/**
 * initialize default endpoint software object
 * @param dev pointer to driver object
 * @return CDN_EOK if no error, Error code elsewhere
 */
static inline uint32_t initEp0(CUSBD_PrivateData * dev) {

    uint32_t ret = CDN_EOK;
    CUSBDMA_ChannelParams channelParams;

    initEp0Hw(dev);

    /* initialize endpoint 0 - software */
    dev->ep0.ep.address = EP0_ADDRESS;
    dev->ep0.ep.ops = &epOps;
    dev->device.ep0 = &dev->ep0.ep;
    channelParams.hwEpNum = 0U;
    channelParams.isDirTx = 1U;
    channelParams.wMaxPacketSize = getEp0MaxPacketSize(dev);
    channelParams.epConfig = buildEp0config(channelParams.wMaxPacketSize);
    channelParams.epIrqConfig = EP_STS_EN_SETUPEN | EP_STS_EN_DESCMISEN | EP_STS_EN_TRBERREN | EP_STS_EN_OUTSMMEN;

    /* save endpoint name */
#ifdef DEBUG
    (void) strncpy(dev->ep0.ep.name, EP0_NAME, 5);
#endif
    /* initialize DMA channel for this endpoint */
    ret = CUSBDMA_ChannelAlloc(&(dev->dmaController), &dev->ep0DmaChannelIn, &channelParams);
    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Could not allocate EP0-IN channel\n", 0);
    }
    if (ret == CDN_EOK) {
        channelParams.isDirTx = 0U;
        ret = CUSBDMA_ChannelAlloc(&(dev->dmaController), &dev->ep0DmaChannelOut, &channelParams);
        if (ret != CDN_EOK) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Could not allocate EP0-OUT channel\n", 0);
        }
    }
    return ret;
}

/**
 * initialize non default endpoint software object
 * @param dev pointer to driver object
 * @return CDN_EOK if no error, Error code elsewhere
 */
static inline uint32_t initEp(CUSBD_PrivateData * dev) {

    uint32_t ret = CDN_EOK;
    CUSBD_EpConfig *ep;

    /* initialize endpoints given in configuration */
    listInit(&dev->device.epList);

    /* initialize IN endpoints*/
    ep = dev->config.epIN;
    ret = initSwEp(dev, ep, 1U);
    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Could not initialize IN endpoints\n", 0);
    }
    if (ret == CDN_EOK) {
        /* Initialize OUT endpoints */
        ep = dev->config.epOUT;
        ret = initSwEp(dev, ep, 0U);
        if (ret != CDN_EOK) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Could not initialize OUT endpoints\n", 0);
        }
    }

    return ret;
}

/**
 * Reset private fields of driver object
 * @param pD pointer to driver object
 */
static inline void initResetPriv(CUSBD_PrivateData * dev) {

    /* Reset pointer to the setup packet  */
    dev->setupPacket = dev->config.setupPacket;
    dev->setupPacketDma = dev->config.setupPacketDma;

    /* reset all internal state variable */
    dev->status_value = 0U;
    dev->suspend = 0U;
    dev->u1_value = 0U;
    dev->u2_value = 0U;
}

/**
 * Initialise HW
 * @param pD pointer to driver object
 * @return CDN_EOK on success, error otherwise
 */
static inline uint32_t initHw(CUSBD_PrivateData* dev) {

    uint32_t ret = CDN_EOK;
    CUSBDMA_Config dmaConfig;

    /* Initialize device object */
    ret = initConfigDev(dev, &dmaConfig);

    /* initialize non default endpoints */
    if (ret == CDN_EOK) {
        ret = initEp(dev);
    }

    if (ret == CDN_EOK) {
        /* reset private driver members */
        initResetPriv(dev);

        /* Configure Device hardware */
        initDevHw(dev);

        /* initialize default endpoint */
        (void) initEp0(dev);
    }
    return ret;
}
#ifdef TINYUSB_INTEGRATION

/**
 * DSR task running in loop/thread context, polls for any event reported by cusbd_isr function.
 * sleeps when executed in thread contxt, polls in nortos case.
 */
void cusbd_dsr(void)
{
    CUSBD_DSREventQueue DevModeInt ;
    uint8_t exitFlag=0;
    uint32_t ret = CDN_EOK;

    if (osal_queue_receive(cdns_dsr_q, &DevModeInt))
    {
        switch (DevModeInt.interrupt_type)
        {
            case CUSBD_INT_NONE_TYPE:
                vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Error : Invalid interrupt_type set 0x%x",DevModeInt.interrupt_type);
            break;
            case CUSBD_INT_DEVICE_TYPE:
                handleIsrDev(DevModeInt.dev, DevModeInt.DSTS, &exitFlag);
            break;
            case CUSBD_INT_ENDPT_TYPE:
                if(((DevModeInt.EpiSTS & 0x1U) != 0) || ((DevModeInt.EpiSTS & 0x00010000U) != 0))
                {
                     /* Handle control EP*/
                     ret = isrHandleEp0(DevModeInt.dev, DevModeInt.EpiSTS, &DevModeInt.EpStatus[0]);
                }
                else
                {
                     /* Handle non control EP*/
                     ret = cusbdmaIsr(DevModeInt.dev, DevModeInt.EpiSTS, &DevModeInt.EpStatus[0]);
                }
            break;
            case CUSBD_INT_SPURIOUS_TYPE:
                vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Error : Received DSR with device or EP status not set Int type 0x%x",DevModeInt.interrupt_type);
            case CUSBD_INT_SW_ISSUE_TYPE:
                vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Error : Software Issue reported from  cusbd_isr 0x%x",DevModeInt.interrupt_type);
            default:
                vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Error : Invalid interrupt_type set 0x%x",DevModeInt.interrupt_type);
            break;
        }
        if(ret != CDN_EOK)
        {
                vDbgMsg(USBSSP_DBG_CUSBD, DBG_CRIT, "Error : DSR Error  ret = %d\n",ret);
        }
    }
}
#endif /* TINYUSB_INTEGRATION */
/**
 * Initialize the driver instance and state, configure the USB device
 * as specified in the 'config' settings, initialize locks used by the
 * driver.
 * @param[in] pD driver state info specific to this instance
 * @param[in,out] config specifies driver/hardware configuration
 * @param[in] callbacks client-supplied callback functions
 * @return CDN_EOK on success
 * @return CDN_ENOTSUP if hardware has an inconsistent configuration or doesn't support feature(s) required by 'config' parameters
 * @return CDN_ENOENT if insufficient locks were available (i.e. something allocated locks between probe and init)
 * @return CDN_EIO if driver encountered an error accessing hardware
 * @return CDN_EINVAL if illegal/inconsistent values in 'config'
 */
uint32_t CUSBD_Init(CUSBD_PrivateData*     pD,
                    const CUSBD_Config*    config,
                    const CUSBD_Callbacks* callbacks) {

    uint32_t ret = CUSBD_InitSF(pD, config, callbacks);

    if (ret == CDN_EOK) {

        pD->callbacks = *callbacks;
        pD->config = *config;

        /* parasoft-begin-suppress MISRA2012-RULE-11_4 "'unsigned long' converted to 'ssReg_t *', DRV-4277" */
        pD->reg = (ssReg_t*) pD->config.regBase; /* get pointer to HW registers */
        /* parasoft-end-suppress MISRA2012-RULE-11_4 */

        vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "regBase %08X\n", pD->config.regBase);
#ifdef TINYUSB_INTEGRATION
        /* Create Cadence DSR queue*/
        cdns_dsr_q = osal_queue_create(&cdns_dsr_qdef);
        if(NULL == cdns_dsr_q)
        {
            ret = CDN_ENOMEM;
        }
#endif /* TINYUSB_INTEGRATION */
	    if (ret == CDN_EOK)
	    {
#ifdef CUSBDSS_DMA_ACCESS_NON_SECURE
        setAXIAccessNonSecure(pD->reg);
#endif

#ifndef VSP_SIM
        ret = ramInit(pD);
#endif

#ifdef CPU_BIG_ENDIAN
        CPS_UncachedWrite32(&pD->reg->USBR_CONF, cpu_to_le32(USBRF_BENDIAN));
#endif

        if (ret == CDN_EOK) {
            /* Init aux buffer for EP-OUT */
            cusbdAuxBufferInit(pD);

                /* configure endpoints */
                ret = initConfigEpHw(pD);
                /* initialize hardware */
                if (ret == CDN_EOK) {
                    ret = initHw(pD);
                }
            }
        }
    }
    return ret;
}

/**
 * Destroy the driver (automatically performs a stop)
 * @param[in] pD driver state info specific to this instance
 * @return CDN_EOK on success
 * @return CDN_EINVAL if illegal/inconsistent values in 'config'
 */
uint32_t CUSBD_Destroy(const CUSBD_PrivateData* pD) {

    uint32_t ret = CUSBD_DestroySF(pD);
    return ret;
}

/**
 * Start the USB driver.
 * @param[in] pD driver state info specific to this instance
 * @return CDN_EOK on success
 * @return CDN_EINVAL if illegal/inconsistent values in 'config'
 */
uint32_t CUSBD_Start(CUSBD_PrivateData* pD) {

    uint32_t ret = CUSBD_StartSF(pD);

    if (ret == CDN_EOK) {
#ifndef VSP_SIM
        CUSBD_PrivateData * dev = pD;
        uint32_t reg;

        /* Check the USBSS-DEV controller version number */
        if (dev->deviceVersion == DEV_VER_V1) {
            /* Fix LFPS timing */
            reg = CPS_UncachedRead32(&dev->reg->USBR_DBG_LINK1);
            reg &= ~LINK1_LFPS_MIN_GEN_U1_EXIT;
            reg |= 0x00005500U;
            reg |= LINK1_LFPS_MIN_GEN_U1_EXIT_SET;
            CPS_UncachedWrite32(&dev->reg->USBR_DBG_LINK1, reg);
        }
#endif
        /* Enable USB Device */
        CPS_UncachedWrite32(&pD->reg->USBR_CONF, USB_CONF_DEVEN);
    }

    return ret;
}

/**
 * Stop the driver. This should disable the hardware, including its
 * interrupt at the source, and make a best-effort to cancel any
 * pending transactions.
 * @param[in] pD driver state info specific to this instance
 * @return CDN_EOK on success
 * @return CDN_EINVAL if illegal/inconsistent values in 'config'
 */
uint32_t CUSBD_Stop(CUSBD_PrivateData* pD) {

    uint32_t ret = CUSBD_StopSF(pD);

    if (ret == CDN_EOK) {

        /* disconnect device from host */
        CPS_UncachedWrite32(&pD->reg->USBR_CONF, USB_CONF_DEVDS);
    }

    return ret;
}

/**
 * Abort all pending requests
 * @param pD pointer to device driver object
 */
static inline void handleIsrDevRSAbortReq(CUSBD_PrivateData* dev) {

    uint8_t i;

    /* abort requests for all endpoints */
    for (i = 0U; i < 30U; i++) {
        CUSBD_Req * req;
        CUSBD_EpPrivate * epp;
        uint32_t ret;
        if (i < 15U) {
            epp = &dev->ep_in_container[i];
        } else {
            epp = &dev->ep_out_container[i - 15U];
        }
        if ((epp->ep.ops == NULL) || (epp->channel == NULL)) {
            continue;
        }

        /* release DMA channel */
        ret = CUSBDMA_ChannelRelease(&(dev->dmaController), epp->channel);
        if (ret != CDN_EOK) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "Could not release DMA channel\n", 0);
        }
        epp->transferPending = 0;
        epp->ep_state = CUSBD_EP_DISABLED; /* endpoint disabled */
        req = getNextReq(epp);

        /* abort request */
        while (req != NULL) {
            reqListDeleteItem(&(epp->reqListHead), req);
            req->status = CDN_ECANCELED;
            if (req->complete != NULL) {
                req->complete(&epp->ep, req);
            }
            req = getNextReq(epp);
        }
    }
}

/**
 * handle device interrupt for Resets and disconnect event
 * @param pD pointer to device driver object
 * @param usb_ists usb_ists register value
 * @param exitFlag exit flag
 */
static inline void handleIsrDevRS(CUSBD_PrivateData* dev, uint32_t usb_ists, uint8_t *exitFlag) {

    vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "handleIsrDevRS called with usb_ists: %x\n", usb_ists);
    /* set speed field */
    if ((usb_ists & USB_ISTS_DISI) > 0U) {
        dev->device.speed = CH9_USB_SPEED_UNKNOWN;
    } else if ((usb_ists & USB_ISTS_UWRESI) > 0U) {

    } else if ((usb_ists & USB_ISTS_UHRESI) > 0U) {

    } else if ((usb_ists & USB_ISTS_DIS2I) > 0U) {
        dev->device.speed = CH9_USB_SPEED_UNKNOWN;
    } else if ((usb_ists & USB_ISTS_U2RESI) > 0U) {
        dev->device.speed = getActualSpeed(dev);
        if (dev->callbacks.usb2PhySoftReset != NULL) {
            vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "Invoking USB 2 PHY reset@ %lx\n", dev->callbacks.usb2PhySoftReset);
            dev->callbacks.usb2PhySoftReset(dev);
        }
    } else {
        /* required by MISRA */
    }

    /* abort all pending requests on all active endpoints */
    handleIsrDevRSAbortReq(dev);

    /* Reset SW endpoints */
    CPS_UncachedWrite32(&dev->reg->USBR_CONF, USB_CONF_CFGRST);

    /* Reset aux buffer pointers */
    cusbdAuxBufferReset(dev);

#ifndef TINYUSB_INTEGRATION
    /* clear interrupt flags */
    CPS_UncachedWrite32(&dev->reg->USBR_ISTS, USB_ISTS_DISI | USB_ISTS_UWRESI | USB_ISTS_UHRESI | USB_ISTS_DIS2I | USB_ISTS_U2RESI);
#endif /* TINYUSB_INTEGRATION */
    /* update device state */
    dev->device.state = CH9_USB_STATE_DEFAULT;
    dev->u1_value = 0U;
    dev->u2_value = 0U;

    /* call callback */
    dev->callbacks.disconnect(dev);
    *exitFlag = 1U;
}

/**
 * Handle U3 related interrupt
 * @param dev pointer to device driver object
 * @param usb_ists usb_ists register value
 * @param exitFlag pointer to exitFlag
 */
static inline void handleIsrDevOtherU3(CUSBD_PrivateData* dev, uint32_t usb_ists, uint8_t *exitFlag) {

    /* enter U3 */
    if (((usb_ists & USB_ISTS_U3ENTI) > 0U) && (*exitFlag == 0U)) {
        CPS_UncachedWrite32(&dev->reg->USBR_ISTS, USB_ISTS_U3ENTI);
        *exitFlag = 1U;
    }

    /* exit U3 */
    if (((usb_ists & USB_ISTS_U3EXTI) > 0U) && (*exitFlag == 0U)) {
        CPS_UncachedWrite32(&dev->reg->USBR_ISTS, USB_ISTS_U3EXTI);
        *exitFlag = 1U;
    }
}

/**
 * Handle U2 related interrupt
 * @param dev pointer to device driver object
 * @param usb_ists usb_ists register value
 * @param exitFlag pointer to exitFlag
 */
static inline void handleIsrDevOtherU2(CUSBD_PrivateData* dev, uint32_t usb_ists, uint8_t *exitFlag) {

    /* enter U2 */
    if (((usb_ists & USB_ISTS_U2ENTI) > 0U) && (*exitFlag == 0U)) {
        CPS_UncachedWrite32(&dev->reg->USBR_ISTS, USB_ISTS_U2ENTI);
        *exitFlag = 1U;
    }

    /* exit U2 */
    if (((usb_ists & USB_ISTS_U2EXTI) > 0U) && (*exitFlag == 0U)) {
        CPS_UncachedWrite32(&dev->reg->USBR_ISTS, USB_ISTS_U2EXTI);
        *exitFlag = 1U;
    }
}

/**
 * Handle U1 related interrupt
 * @param dev pointer to device driver object
 * @param usb_ists usb_ists register value
 * @param exitFlag pointer to exitFlag
 */
static inline void handleIsrDevOtherU1(CUSBD_PrivateData* dev, uint32_t usb_ists, uint8_t *exitFlag) {

    /* enter U1 */
    if (((usb_ists & USB_ISTS_U1ENTI) > 0U) && (*exitFlag == 0U)) {
        CPS_UncachedWrite32(&dev->reg->USBR_ISTS, USB_ISTS_U1ENTI);
        *exitFlag = 1U;
    }

    /* exit U1 */
    if (((usb_ists & USB_ISTS_U1EXTI) > 0U) && (*exitFlag == 0U)) {
        CPS_UncachedWrite32(&dev->reg->USBR_ISTS, USB_ISTS_U1EXTI);
        *exitFlag = 1U;
    }
}

/**
 * Handle other interrupts
 * @param dev pointer to device driver object
 * @param usb_ists usb_ists register value
 * @param exitFlag pointer to exitFlag
 */
static inline void handleIsrDevOther(CUSBD_PrivateData* dev, uint32_t usb_ists, uint8_t *exitFlag) {

    /* handle U1 */
    handleIsrDevOtherU1(dev, usb_ists, exitFlag);

    /* handle U2 */
    handleIsrDevOtherU2(dev, usb_ists, exitFlag);

    /* handle U3 */
    handleIsrDevOtherU3(dev, usb_ists, exitFlag);

    /* handle ITP */
    if (((usb_ists & USB_ISTS_ITPI) > 0U) && (*exitFlag == 0U)) {
        CPS_UncachedWrite32(&dev->reg->USBR_ISTS, USB_ISTS_ITPI);
        if (dev->callbacks.busInterval != NULL) {
            dev->callbacks.busInterval(dev);
        }
        *exitFlag = 1U;
    }
}

/**
 * internal function, handle device interrupt
 * @param pD pointer to device driver object
 * @param usb_ists usb_ists register value
 * @param exitFlag pointer to exit flag
 */
static inline void handleIsrDev(CUSBD_PrivateData* dev, uint32_t usb_ists, uint8_t *exitFlag) {

    if (
        ((usb_ists & USB_ISTS_DISI) > 0U) ||
        ((usb_ists & USB_ISTS_UWRESI) > 0U) ||
        ((usb_ists & USB_ISTS_UHRESI) > 0U) ||
        ((usb_ists & USB_ISTS_DIS2I) > 0U) ||
        ((usb_ists & USB_ISTS_U2RESI) > 0U)
        ) {
        handleIsrDevRS(dev, usb_ists, exitFlag);
    }

    /* USB Connect interrupt */
    if ((((usb_ists & USB_ISTS_CONI) > 0U) || ((usb_ists & USB_ISTS_CON2I) > 0U)) && (*exitFlag == 0U)) {

        /* This is to avoid race condition; assume disconnect interrupt inbetween 1) check connection status 2) data transfer */
        dev->ep0NextState = CH9_EP0_SETUP_PHASE;
        /* initialize endpoint 0 - software */
        CPS_UncachedWrite32(&dev->reg->USBR_ISTS, USB_ISTS_CONI | USB_ISTS_CON2I);

        /* set actual USB speed */
        dev->device.speed = getActualSpeed(dev);
        initDevHw(dev); /* configure device hardware */

        /* update EP0 max packet size */
        dev->ep0.ep.maxPacket = getEp0MaxPacketSize(dev);
        (void) CUSBDMA_ChannelSetMaxPktSz(&dev->dmaController, dev->ep0DmaChannelIn, dev->ep0.ep.maxPacket);
        (void) CUSBDMA_ChannelSetMaxPktSz(&dev->dmaController, dev->ep0DmaChannelOut, dev->ep0.ep.maxPacket);

        vDbgMsg(USBSSP_DBG_CUSBD_ISR, DBG_FYI, "CONNECT EVENT, actual speed: %d\n", dev->device.speed);
        vDbgMsg(USBSSP_DBG_CUSBD_ISR, DBG_FYI, "CONNECT EVENT, max speed: %d\n", dev->device.maxSpeed);
        dev->callbacks.connect(dev); /* call user callback */
        *exitFlag = 1U;
    }
    handleIsrDevOther(dev, usb_ists, exitFlag);
}

#ifndef TINYUSB_INTEGRATION
/* Pure Cadence driver; no class driver  */

/**
 * Handle other interrupts
 * @param dev pointer to device driver object
 * @param ep_ists ep_ists register value
 * @param exitFlag pointer to exitFlag
 * @return CDN_EOK on success, error otherwise
 */
static inline uint32_t isrHandleEp0(CUSBD_PrivateData* dev, uint32_t ep_ists, uint8_t *exitFlag) {
    uint32_t ret = CDN_EOK;

    /* ep0out */
    if (((ep_ists & 0x00000001U) > 0U) && (*exitFlag == 0U)) {
        vDbgMsg(USBSSP_DBG_CUSBD_ISR, DBG_FYI, "Handle EP0-out IRQ %x\n", ep_ists);
        ret = handleEp0Irq(dev, 0U);
        *exitFlag = 1U;
    }

    /* ep0in */
    if (((ep_ists & 0x00010000U) > 0U) && (*exitFlag == 0U)) {
        vDbgMsg(USBSSP_DBG_CUSBD_ISR, DBG_FYI, "Handle EP0-in IRQ %x\n", ep_ists);
        ret = handleEp0Irq(dev, 1U);
        *exitFlag = 1U;
    }
    return ret;
}

#else /* TINYUSB_INTEGRATION */

/**
 * Handle other interrupts
 * @param dev pointer to device driver object
 * @param ep_ists ep_ists register value
 * @param EpStatusBase Base pointer to EpStatusBase array
 * @return CDN_EOK on success, error otherwise
 */
static inline uint32_t isrHandleEp0(CUSBD_PrivateData* dev, uint32_t ep_ists, uint32_t * EpStatusBase) {
    uint32_t ret = CDN_EOK;

    /* ep0out */
    if ((ep_ists & 0x00000001U) > 0U) {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "Handle EP0-out IRQ %x\n", ep_ists);
        ret = handleEp0Irq(dev, 0U, EpStatusBase[0]);
    }

    /* ep0in */
    if ((ep_ists & 0x00010000U) > 0U) {
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "Handle EP0-in IRQ %x\n", ep_ists);
        ret = handleEp0Irq(dev, 1U, EpStatusBase[16]);
    }
    return ret;
}
#endif /* TINYUSB_INTEGRATION */

/**
 * Driver ISR.  Platform-specific code is responsible for ensuring
 * this gets called when the corresponding hardware's interrupt is
 * asserted. Registering the ISR should be done after calling init,
 * and before calling start. The driver's ISR will not attempt to lock
 * any locks, but will perform client callbacks. If the client wishes
 * to defer processing to non-interrupt time, it is responsible for
 * doing so.
 * @param[in] pD driver state info specific to this instance
 * @return CDN_EOK on success, error otherwise
 */
#ifndef TINYUSB_INTEGRATION
/* Pure Cadence driver; no class driver  */

uint32_t CUSBD_Isr(CUSBD_PrivateData* pD) {

    uint32_t ret = CUSBD_IsrSF(pD);

    if (ret == CDN_EOK) {

        uint32_t usb_ists, ep_ists;
        CUSBD_PrivateData* dev = pD;
        uint8_t exitFlag = 0U;

        /* first check device interrupt */
        usb_ists = CPS_UncachedRead32(&dev->reg->USBR_ISTS);
        vDbgMsg(USBSSP_DBG_CUSBD_ISR, DBG_FYI, "USB_ISTS: %08X\n", usb_ists);

        if (usb_ists > 0U) {
            handleIsrDev(dev, usb_ists, &exitFlag);
        }
        /* check if interrupt generated by endpoint */
        ep_ists = CPS_UncachedRead32(&dev->reg->USBR_EP_ISTS);
        vDbgMsg(USBSSP_DBG_CUSBD_ISR, DBG_FYI, "EP_ISTS_0: %08X\n", ep_ists);

        if ((ep_ists == 0U) && (exitFlag == 0U)) {
            exitFlag = 1U; /* set exitFlag to return if no int. generated*/
        }

        if (exitFlag == 0U) {
            cusbdmaIsr(dev);
        }

        /* check if interrupt generated by default endpoint on both directions (ss speed) */
        ret = isrHandleEp0(dev, ep_ists, &exitFlag);

    }
    return ret;
}
#else /* TINYUSB_INTEGRATION */

uint32_t CUSBD_Isr(CUSBD_PrivateData* pD) {

    uint32_t ret = CUSBD_IsrSF(pD);

    if (ret == CDN_EOK) {

        uint32_t usb_ists, ep_ists;
        CUSBD_PrivateData* dev = pD;
        CUSBDMA_DmaController * ctrl = &(dev->dmaController);
        DmaRegs * regs = ctrl->regs;

        /* check device interrupt */
        usb_ists = CPS_UncachedRead32(&dev->reg->USBR_ISTS);
        CUSBD_DSREventQueue DevModeInt ;
        (void)memset((void *)&DevModeInt, 0U, sizeof(CUSBD_DSREventQueue));

        if (usb_ists > 0U) {
            DevModeInt.interrupt_type = CUSBD_INT_DEVICE_TYPE;
            DevModeInt.DSTS = usb_ists;
            DevModeInt.dev = dev;
            osal_queue_send(cdns_dsr_q, (void const  *)&DevModeInt, true);
            /* clear interrupt flags */
            /* TODO : Clear only enabled interrupts check for L1/L2 U1/U2/U3 */
            CPS_UncachedWrite32(&dev->reg->USBR_ISTS, 0xFFFFFFFFU);
        }
        else
        {
            /* check if interrupt generated by endpoint */
            ep_ists = CPS_UncachedRead32(&dev->reg->USBR_EP_ISTS);
            if(ep_ists != 0)
            {
                uint32_t epSel;
                DevModeInt.interrupt_type = CUSBD_INT_ENDPT_TYPE;
                DevModeInt.EpiSTS = ep_ists;
                DevModeInt.dev = dev;
                /* store ep_sel */
                epSel = CPS_UncachedRead32(&regs->ep_sel);

                while(ep_ists)
                {
                    uint8_t ep_no;
                    uint32_t EPConfRaw;
                    ret = GetEPSelConfigValue(ep_ists, &ep_no, &EPConfRaw);
                    if(ret != CDN_EOK)
                    {
                        /*   Danger :: notify to Task */
                        DevModeInt.interrupt_type = CUSBD_INT_SW_ISSUE_TYPE;
                        osal_queue_send(cdns_dsr_q, (void const  *)&DevModeInt, true);
                        break;
                    }
                    if((EPConfRaw & 0x80U) > 0) /* IN EP */
                    {
                        ret = SaveEndptReg(regs, &(DevModeInt.EpStatus[ep_no + 16U]), EPConfRaw);
                    }
                    else
                    {
                        ret = SaveEndptReg(regs, &(DevModeInt.EpStatus[ep_no]), EPConfRaw);
                    }
                    if(ret == CDN_EFAULT)
                    {
                        /*   Danger :: notify to Task */
                        DevModeInt.interrupt_type = CUSBD_INT_SPURIOUS_TYPE;
                        osal_queue_send(cdns_dsr_q, (void const  *)&DevModeInt, true);
                        break;
                    }
                    if(ret == CDN_EINVAL)
                    {
                        /*   Danger :: notify to Task */
                        DevModeInt.interrupt_type = CUSBD_INT_SW_ISSUE_TYPE;
                        osal_queue_send(cdns_dsr_q, (void const  *)&DevModeInt, true);
                        break;
                    }
                    ep_ists = ep_ists & (ep_ists - 1U);
                }
                /* Restore back ep_sel register */
                CPS_UncachedWrite32(&regs->ep_sel, epSel);
                osal_queue_send(cdns_dsr_q, (void const  *)&DevModeInt, true);
                /* clear Endpt ISTS interrupt flag */
                CPS_UncachedWrite32(&dev->reg->USBR_ISTS, 0xFFFFFFFFU);
            }
            else
            {
                /*   Danger :: notify to Task */
                DevModeInt.interrupt_type = CUSBD_INT_SPURIOUS_TYPE;
                osal_queue_send(cdns_dsr_q, (void const  *)&DevModeInt, true);
            }
        }
    }
    return ret;
}
#endif  /* TINYUSB_INTEGRATION */
/**
 * Enable Endpoint.  This function should be called within
 * SET_CONFIGURATION(configNum > 0) request context. It configures
 * required hardware controller endpoint with parameters given in
 * desc.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint being configured
 * @param[in] desc endpoint descriptor
 * @return 0 on success
 * @return CDN_EINVAL if pD, ep or desc is NULL or desc is not a endpoint descriptor
 */
uint32_t CUSBD_EpEnable(CUSBD_PrivateData* pD, CUSBD_Ep* ep, const uint8_t* desc) {
    uint32_t ret = CDN_EOK;
    ret = CUSBD_EpEnableSF(pD, ep, desc);
    if (CDN_EOK == ret) {
        ret = epEnable(pD, ep, desc);
    }
    return ret;
}

/**
 * Disable Endpoint. Functions unconfigures hardware endpoint.
 * Endpoint will not respond to any host packets. This function should
 * be called within SET_CONFIGURATION(configNum = 0) request context
 * or on disconnect event. All queued requests on endpoint are
 * completed with CDN_ECANCELED status.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint being unconfigured
 * @return 0 on success
 * @return CDN_EINVAL if pD or ep is NULL
 */
uint32_t CUSBD_EpDisable(CUSBD_PrivateData* pD, CUSBD_Ep* ep) {
    uint32_t ret = CDN_EOK;
    ret = CUSBD_EpDisableSF(pD, ep);
    if (CDN_EOK == ret) {
        ret = epDisable(pD, ep);
    }
    return ret;
}

/**
 * Set halt or clear halt state on endpoint. When setting halt, device
 * will respond with STALL packet to each host packet. When clearing
 * halt, endpoint returns to normal operating.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint being setting or clearing halt state
 * @param[in] value if 1 sets halt, if 0 clears halt on endpoint
 * @return 0 on success
 * @return CDN_EPERM if endpoint is disabled (has not been configured yet)
 * @return CDN_EINVAL if pD or ep is NULL
 */
uint32_t CUSBD_EpSetHalt(CUSBD_PrivateData* pD, const CUSBD_Ep* ep, uint8_t value) {
    uint32_t ret = CDN_EOK;
    ret = CUSBD_EpSetHaltSF(pD, ep);
    if (CDN_EOK == ret) {
        ret = epSetHalt(pD, ep, value);
    }
    return ret;
}

/**
 * Set halt on endpoint. Function sets endpoint to permanent halt
 * state. State can not be changed even with epSetHalt(pD, ep, 0)
 * function. Endpoint returns automatically to its normal state on
 * disconnect event.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint being setting halt state
 * @return 0 on success
 * @return CDN_EPERM if endpoint is disabled (has not been configured yet)
 * @return CDN_EINVAL if pD or ep is NULL
 */
uint32_t CUSBD_EpSetWedge(CUSBD_PrivateData* pD, const CUSBD_Ep* ep) {
    uint32_t ret = CDN_EOK;
    ret = CUSBD_EpSetWedgeSF(pD, ep);
    if (CDN_EOK == ret) {
        ret = epSetWedge(pD, ep);
    }
    return ret;
}

/**
 * Returns number of bytes in hardware endpoint FIFO. Function useful
 * in application where exact number of data bytes is required. In
 * some situation software higher layers must be aware of number of
 * data bytes issued but not transfered by hardware because of aborted
 * transfers, for example on disconnect event. *
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint which status is to be returned
 * @return number of bytes in hardware endpoint FIFO
 * @return CDN_EINVAL if pD or ep is NULL
 */
uint32_t CUSBD_EpFifoStatus(const CUSBD_PrivateData* pD, const CUSBD_Ep* ep) {
    uint32_t ret = CDN_EOK;
    ret = CUSBD_EpFifoStatusSF(pD, ep);
    if (CDN_EOK == ret) {
        /* Function needs to be implemented*/
        vDbgMsg(USBSSP_DBG_CUSBD, DBG_FYI, "ep:%02X\n", ep->address);
    }
    return ret;
}

/**
 * Flush hardware endpoint FIFO.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint which FIFO is to be flushed
 */
uint32_t CUSBD_EpFifoFlush(CUSBD_PrivateData* pD, const CUSBD_Ep* ep) {
    uint32_t ret = CDN_EOK;
    ret = CUSBD_EpFifoFlushSF(pD, ep);
    if (CDN_EOK == ret) {
        ret = epFifoFlush(pD, ep);
    }
    return ret;
}

/**
 * Submits IN/OUT transfer request to an endpoint.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint associated with the request
 * @param[in] req request being submitted
 * @return 0 on success
 * @return CDN_EPROTO only on default endpoint if endpoint is not in data stage phase
 * @return CDN_EPERM if endpoint is not enabled
 * @return CDN_EINVAL if pD, ep, or req is NULL
 */
uint32_t CUSBD_ReqQueue(CUSBD_PrivateData* pD, const CUSBD_Ep* ep, CUSBD_Req* req) {
    uint32_t ret = CUSBD_ReqQueueSF(pD, ep, req);
    if (ret == CDN_EOK) {
        ret = reqQueue(pD, ep, req);
    }
    return ret;
}

/**
 * Dequeues IN/OUT transfer request from an endpoint. Function
 * completes all queued request with CDN_ECANCELED status.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint associated with the request
 * @param[in] req request being dequeued
 * @return 0 on success
 * @return CDN_EINVAL if pD or ep is NULL
 */
uint32_t CUSBD_ReqDequeue(CUSBD_PrivateData* pD, CUSBD_Ep* ep, CUSBD_Req* req) {

    uint32_t ret = CUSBD_ReqDequeueSF(pD, ep, req);
    if (ret == CDN_EOK) {
        ret = reqDequeue(ep, req);
    }
    return ret;
}

/**
 * Returns pointer to CUSBD object. CUSBD object is a logical
 * representation of USB device. CUSBD contains endpoint collection
 * accessed through epList field. Endpoints collection is organized as
 * double linked list.
 * @param[in] pD driver state info specific to this instance
 * @param[out] dev returns pointer to CUSBD instance
 */
void CUSBD_GetDevInstance(CUSBD_PrivateData* pD, CUSBD_Dev** dev) {
    uint32_t ret = ((pD == NULL) || (dev == NULL)) ? CDN_EINVAL : CDN_EOK;

    if (CDN_EOK == ret) {
        *dev = &pD->device;
    }
}

/**
 * Returns number of frame. Some controllers have counter of SOF
 * packets or ITP packets in the Super Speed case. Function returns
 * value of this counter. This counter can be used for time
 * measurement. Single FS frame is 1 ms measured, for HS and SS is
 * 125us.
 * @param[in] pD driver state info specific to this instance
 * @param[out] numOfFrame returns number of USB frame
 * @return CDN_EOK on success
 * @return CDN_EOPNOTSUPP if feature is not supported
 * @return CDN_EINVAL if pD or numOfFrame is NULL
 */
uint32_t CUSBD_DGetFrame(CUSBD_PrivateData* pD, uint32_t* numOfFrame) {

    uint32_t ret = CUSBD_DGetFrameSF(pD, numOfFrame);

    if (ret == CDN_EOK) {
        *numOfFrame = CPS_UncachedRead32(&pD->reg->USBR_ITPN);
    }

    return ret;
}

/**
 * Sets the device self powered feature.
 * @param[in] pD driver state info specific to this instance
 * @return CDN_EOK on success
 * @return CDN_EOPNOTSUPP if feature is not supported
 * @return CDN_EINVAL if pD is NULL
 */
uint32_t CUSBD_DSetSelfpowered(const CUSBD_PrivateData* pD) {

    uint32_t ret = CUSBD_DSetSelfpoweredSF(pD);

    if (ret == CDN_EOK) {
        ret = CDN_ENOTSUP;
    }
    return ret;
}

/**
 * Clear the device self powered feature.
 * @param[in] pD driver state info specific to this instance
 * @return CDN_EOK on success
 * @return CDN_EOPNOTSUPP if feature is not supported
 * @return CDN_EINVAL if pD is NULL
 */
uint32_t CUSBD_DClearSelfpowered(const CUSBD_PrivateData* pD) {

    uint32_t ret = CUSBD_DClearSelfpoweredSF(pD);

    if (ret == CDN_EOK) {
        ret = CDN_ENOTSUP;
    }
    return ret;
}

/**
 * Returns configuration parameters: U1 exit latency and U2 exit
 * latency Function useful only in Super Speed mode.
 * @param[in] pD driver state info specific to this instance
 * @param[out] configParams pointer to CH9_ConfigParams structure
 * @return CDN_EOK on success
 * @return CDN_EINVAL if illegal/inconsistent values in 'config'
 */
uint32_t CUSBD_DGetConfigParams(const CUSBD_PrivateData* pD, CH9_ConfigParams* configParams) {

    uint32_t ret = CUSBD_DGetConfigParamsSF(pD, configParams);

    if (ret == CDN_EOK) {
        configParams->bU1devExitLat = CUSBSS_U1_DEV_EXIT_LAT;
        configParams->bU2DevExitLat = CUSBSS_U2_DEV_EXIT_LAT;
    }

    return ret;
}


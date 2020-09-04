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
 * cdn_xhci_common.c
 * USB Host controller driver functions called in both ISR and user context
 *
 * XHCI driver.
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "cdn_xhci_if.h"
#include "cdn_xhci_structs_if.h"
#include "cusb_ch9_if.h"
#include "cusb_ch9_structs_if.h"

#include "trb.h"
#include "cdn_log.h"                 /* DEBUG macros */
#include "cps_drv.h"
#include "usbssp_regs_macros.h"

#include "cdn_xhci_sanity.h"
#include "cdn_xhci_internal.h"

/**
 * Get endpoint status
 * @param res driver resources
 * epIndex endpoint index
 * @return endpoint status
 */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
USBSSP_EpContexEpState getEndpointStatus(USBSSP_DriverResourcesT const *res, uint32_t epIndex) {

    /* returns endpoint state */
    USBSSP_EpContexEpState ret;

    /* check if epIndex is within correct index range */
    if (epIndex < USBSSP_EP0_CONT_OFFSET) {
        /* display error */
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Endpoint context Index error < %d\n", res->instanceNo, USBSSP_EP0_CONT_OFFSET);
        ret = USBSSP_EP_CONTEXT_EP_STATE_ERROR;
    } else {

        /* keeps integer value returned from endpoint context */
        uint32_t hwValue;

        /* get state of default endpoint */
        if (epIndex == USBSSP_EP0_CONT_OFFSET) {
            hwValue = (le32ToCpu(res->xhciMemRes->outputContext->ep0Context[0]) & (uint32_t) USBSSP_EP_CONTEXT_STATE_MASK);
        } else if (epIndex < USBSSP_EP_CONT_MAX) { /* get status of no default endpoint */
            /* calculate index of endpoint in epContext array */
            uint32_t epBase = epIndex - USBSSP_EP_CONT_OFFSET;
            /* read endpoint state from output context updated by controller */
            hwValue = (le32ToCpu(res->xhciMemRes->outputContext->epContext[epBase][0]) & (uint32_t) USBSSP_EP_CONTEXT_STATE_MASK);
        } else {
            hwValue = 0U;
        }
        /* transcode integer values to enumerated values - MISRA requirement */
        switch (hwValue) {

        /* endpoint state disabled */
        case 0: ret = USBSSP_EP_CONTEXT_EP_STATE_DISABLED;
            break;

        /* endpoint state running */
        case 1: ret = USBSSP_EP_CONTEXT_EP_STATE_RUNNING;
            break;

        /* endpoint state halted */
        case 2: ret = USBSSP_EP_CONTEXT_EP_STATE_HALTED;
            break;

        /* endpoint state stopped */
        case 3: ret = USBSSP_EP_CONTEXT_EP_STATE_STOPPED;
            break;

        /* endpoint state error */
        default: ret = USBSSP_EP_CONTEXT_EP_STATE_ERROR;
            break;

        }
    }

    return (ret);
}
/* parasoft-end-suppress METRICS-36-3 */

/**
 * Auxiliary function, returns actual speed field from endpoint context, note
 * that software speed enum values may differ from speed values coded in XHCI spec.
 * @param speed actual speed kept in res->actualSpeed
 * @return speed value of speed given in values as XHCI specification states in
 *                endpoint context structure
 */
uint32_t getSlotSpeed(CH9_UsbSpeed speed) {

    uint32_t slotSpeed;

    /* translate CH9_UsbSpeed value to integer values according to XHCI spec */
    switch (speed) {
    /* low speed */
    case CH9_USB_SPEED_LOW: slotSpeed = 2U;
        break;
    /* full speed */
    case CH9_USB_SPEED_FULL: slotSpeed = 1U;
        break;
    /* high speed */
    case CH9_USB_SPEED_HIGH: slotSpeed = 3U;
        break;
    /* super speed */
    case CH9_USB_SPEED_SUPER: slotSpeed = 4U;
        break;
    default: slotSpeed = 0U;
        break;
    }

    return (slotSpeed);
}

/**
 * Get the current speed. Should be called only if port connected
 * @param res driver resources
 * @return Current speed
 */
CH9_UsbSpeed getSpeed(const USBSSP_DriverResourcesT *res) {

    /* get protocol speed ID */
    CH9_UsbSpeed actualSpeed = CH9_USB_SPEED_UNKNOWN;
    uint32_t portStatus = xhciRead32(&res->regs.xhciPortControl[res->actualPort - 1U].portsc);
    uint8_t portSpeed = (uint8_t) CPS_FLD_READ(USBSSP__PORTSC1USB3, PORTSPEED, portStatus);

    /* according to Table 157: Default USB Speed ID Mapping */
    switch (portSpeed) {

    /* full speed */
    case 1U:
        actualSpeed = CH9_USB_SPEED_FULL;
        break;

    /* low speed */
    case 2U:
        actualSpeed = CH9_USB_SPEED_LOW;
        break;

    /* high speed */
    case 3U:
        actualSpeed = CH9_USB_SPEED_HIGH;
        break;

    /* super speed */
    case 4U:
        actualSpeed = CH9_USB_SPEED_SUPER;
        break;

    /* speed unknown */
    default:
        actualSpeed = CH9_USB_SPEED_UNKNOWN;
        break;
    }
    return actualSpeed;
}

/**
 * Function updates res->actualSpeed value
 * @param res driver resources
 */
void setSpeed(USBSSP_DriverResourcesT *res) {
    CH9_UsbSpeed speed = getSpeed(res);

    res->actualSpeed = speed;

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> actual speed: %d\n", res->instanceNo, res->actualSpeed);
}

/**
 * function get max packet size for default endpoint based on speed
 * @param res driver resources
 * @return max packet size for default endpoint
 */
static CH9_UsbEP0Max getMaxPacketSizeEp0(USBSSP_DriverResourcesT const *res) {

    CH9_UsbEP0Max epMaxPacketSize;

    switch (res->actualSpeed) {

    /* low speed */
    case CH9_USB_SPEED_LOW: epMaxPacketSize = CH9_USB_EP0_MAX_LOW;
        break;

    /* full speed */
    case CH9_USB_SPEED_FULL: epMaxPacketSize = CH9_USB_EP0_MAX_FULL;
        break;

    /* high speed */
    case CH9_USB_SPEED_HIGH: epMaxPacketSize = CH9_USB_EP0_MAX_HIGH;
        break;

    /* super, super speed plus */
    case CH9_USB_SPEED_SUPER:
    case CH9_USB_SPEED_SUPER_PLUS:
        epMaxPacketSize = CH9_USB_EP0_MAX_SUPER;
        break;

    /* unknown speed */
    default: epMaxPacketSize = CH9_USB_EP0_MAX_UNKNOWN;
        break;
    }

    return epMaxPacketSize;
}

/**
 * set the slot context
 * @param res driver resource
 */
static void setSlotContext(USBSSP_DriverResourcesT *res) {

    CH9_UsbEP0Max epMaxPacketSize;
    uint32_t slotSpeed = 0U;

    /* set EP0 max packet size */
    epMaxPacketSize = getMaxPacketSizeEp0(res);

    /* CH9_UsbSpeed coding is different than XHCI one */
    slotSpeed = getSlotSpeed(res->actualSpeed);

    /*6.2.2 set slot context entries and speed */
    res->inputContext->slot[0] = cpuToLe32(
        (uint32_t) (1UL << USBSSP_SLOT_CXT_CXT_ENT_POS)
        | (uint32_t) (slotSpeed << USBSSP_SLOT_CONTEXT_SPEED_POS)
        );

    res->inputContext->slot[1] = cpuToLe32(
        (uint32_t) res->actualPort << USBSSP_SLOT_CXT_PORT_NUM_POS);

    /* set USB device address */
    res->inputContext->slot[3] = cpuToLe32((uint32_t) res->devAddress);

    /* set default endpoint context */
    res->inputContext->ep0Context[1] = cpuToLe32(
        ((uint32_t) USBSSP_EP_CXT_EPTYP_CTL_BI << USBSSP_EP_CONTEXT_EP_TYPE_POS)
        | ((uint32_t) epMaxPacketSize << USBSSP_EP_CXT_MAX_PKT_SZ_POS)
        | (USBSSP_EP_CONTEXT_3ERR << USBSSP_EP_CONTEXT_CERR_POS)
        );     /* endpoint 0 control type, max packet, error count */

    set64Value(
        &res->inputContext->ep0Context[2],
        &res->inputContext->ep0Context[3],
        cpuToLe64(get64PhyAddrOf32ptr(&res->ep0.enqueuePtr->dword0) | res->ep0.toogleBit)
        );

    res->ep0.contextIndex = 1;

    res->inputContext->ep0Context[4] = cpuToLe32(
        ((uint32_t) (USBSSP_EP_CXT_EP_CTL_AVGTRB_LEN << USBSSP_EP_CXT_EP_AVGTRBLEN_POS))
        );     /* endpoint 0 control type - average trb length must be set to 8. */

}

/**
 * Function builds SET ADDRESS command TRB
 * @param res driver resources
 * @param bsrVal BSR bit value
 */
static void setAddressCommandTrb(USBSSP_DriverResourcesT *res, uint8_t bsrVal) {

    /* two first DWORDs are pointer to input context */
    set64Value(
        &res->commandQ.enqueuePtr->dword0,
        &res->commandQ.enqueuePtr->dword1,
        cpuToLe64(get64PhyAddrOf32ptr(res->inputContext->inputControlContext))
        );

    /* DWORD3 is a TRB code and required flags */
    res->commandQ.enqueuePtr->dword3 = cpuToLe32(
        (uint32_t) ((uint32_t) res->actualdeviceSlot << USBSSP_SLOT_ID_POS)
        | (uint32_t) (USBSSP_TRB_ADDR_DEV_CMD << USBSSP_TRB_TYPE_POS)
        | ((uint32_t) bsrVal << USBSSP_BSR_POS)
        | (uint32_t) res->commandQ.toogleBit);

    CPS_CacheFlush((void*) res->inputContext,
                   sizeof (res->inputContext->inputControlContext)
                   + sizeof (res->inputContext->slot)
                   + sizeof (res->inputContext->ep0Context),
                   0);
}

/**
 * Set address. Function executes set address request on connected device
 * @param[in] res driver resources
 * @param bsrVal BSR bit value
 */
void setAddress(USBSSP_DriverResourcesT *res, uint8_t bsrVal) {

    uint8_t errorAndReturn = 0U;

    /* If slot is not enabled (SLOT_ENABLED not completed) - ending */
    if (res->actualdeviceSlot == 0U) {
        errorAndReturn = 1U;
    }

    if (errorAndReturn == 0U) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> actual port: %d, BSR: %d\n", res->instanceNo, res->actualPort, bsrVal);
        setSpeed(res);

        (void) memset((void*) res->inputContext, 0, sizeof (USBSSP_InputContexT));

        /* On a set address also reserve the memory for the output context */
        (void) memset((void*) res->xhciMemRes->outputContext, 0, sizeof (USBSSP_OutputContexT));

        /* set input control context: A0 and A1, all Dx should be 0 */
        res->inputContext->inputControlContext[0] = 0; /* Dx = 0 */
        res->inputContext->inputControlContext[1] = cpuToLe32(3); /* A0/A1=1/1, all other Ax = 0 */

        /* set slot context */
        setSlotContext(res);

        /* send command to XHCI */
        setAddressCommandTrb(res, bsrVal);

        if (res->inputContextCallback != NULL) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Calling inputContextCallback()\n", res->instanceNo);
            res->inputContextCallback(res);
        }
        (void) memcpy(&res->inputContextCopy, res->inputContext, sizeof (USBSSP_InputContexT));
        updateQueuePtr(&res->commandQ, 0U);
        hostCmdDoorbell(res);
    }
}

/**
 * Insert Link TRB
 * @param[in] queue: pointer to producer queue
 * @param linkTrbChainFlag Chain flag if using Link TRB
 */
static void insertLinkTRB(USBSSP_ProducerQueueT *queue, uint32_t linkTrbChainFlag) {
    uint32_t linkTrbFlags = (USBSSP_TRB_LINK << USBSSP_TRB_TYPE_POS)
                            | USBSSP_TRB_LNK_TGLE_CYC_MSK
                            | ((uint32_t) (queue->toogleBit))
                            | linkTrbChainFlag;

    /* set DWORD0 and DWORD1 */
    set64Value(
        &queue->ring[USBSSP_PRODUCER_QUEUE_SIZE - 1U].dword0,
        &queue->ring[USBSSP_PRODUCER_QUEUE_SIZE - 1U].dword1,
        cpuToLe64(get64PhyAddrOf32ptr(&queue->ring[0].dword0))
        );
    /* set DWORD2 */
    queue->ring[USBSSP_PRODUCER_QUEUE_SIZE - 1U].dword2 = cpuToLe32(0);
    /* set flags in DWORD3 */
    queue->ring[USBSSP_PRODUCER_QUEUE_SIZE - 1U].dword3 = cpuToLe32(linkTrbFlags);

    CPS_CacheFlush((void*) &queue->ring[USBSSP_PRODUCER_QUEUE_SIZE - 1U], sizeof (USBSSP_RingElementT), 0);

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "LinkTRB contextIndex:%d linkAddr:0x%16x  dword3:0x%08x\n",
            queue->contextIndex,
            (((uint64_t) queue->ring[USBSSP_PRODUCER_QUEUE_SIZE - 1U].dword1) << 32) | queue->ring[USBSSP_PRODUCER_QUEUE_SIZE - 1U].dword0,
            queue->ring[USBSSP_PRODUCER_QUEUE_SIZE - 1U].dword3);
}

/**
 * Update enqueue pointer
 * @param[in] queue: pointer to producer queue
 * @param linkTrbChainFlag Chain flag if using Link TRB
 */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
void updateQueuePtr(USBSSP_ProducerQueueT *queue, uint32_t linkTrbChainFlag) {

    USBSSP_RingElementT *oldPtr = queue->enqueuePtr;

    /* calculate where ring ends - the last element is a LINK TRB */
    USBSSP_RingElementT *poolEnd = &queue->ring[(USBSSP_PRODUCER_QUEUE_SIZE - 2U)];

    CPS_CacheFlush((void*) queue->enqueuePtr, sizeof (USBSSP_RingElementT), 0);
    ++queue->enqueuePtr;

    /* check if enqueuePtr exceeded ring pool and turn back to origin if yes */
    if (queue->enqueuePtr > poolEnd) {

        queue->enqueuePtr = queue->ring;

        /* update TRB when queue is not full */
        if (queue->enqueuePtr != queue->dequeuePtr) {
            /* insert LINK TRB */
            insertLinkTRB(queue, linkTrbChainFlag);
            /* toggle the cycle bit */
            queue->toogleBit ^= 1U;
        }
    }

    /* check if queue full and if yes do nothing */
    if (queue->enqueuePtr == queue->dequeuePtr) {
        queue->enqueuePtr = oldPtr;
        vDbgMsg(USBSSP_DBG_DRV, DBG_WARN, "Warning ContextIndex:%d QUEUE FULL!\n", queue->contextIndex);
    }

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "contextIndex:%d oldPtr:0x%08x enqueuePtr:0x%08x dequeuePtr:0x%08x \n",
            queue->contextIndex, oldPtr, queue->enqueuePtr, queue->dequeuePtr);

}
/* parasoft-end-suppress METRICS-36-3 */

/**
 * No Operation test. Function used for testing purposes
 * @param[in] res driver resources, internal function
 */
void noOpTest(USBSSP_DriverResourcesT *res) {
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> NOOP test\n", res->instanceNo);
    res->commandQ.enqueuePtr->dword3 = cpuToLe32(
        (uint32_t) ((USBSSP_TRB_NO_OP_COMMAND << USBSSP_TRB_TYPE_POS) | (uint32_t) res->commandQ.toogleBit)
        );
    updateQueuePtr(&res->commandQ, 0U);
    hostCmdDoorbell(res);
}

/**
 *
 * @param setupData Pointer to the uint8_t * buffer having setup data
 * @param setup pointer to the output struct CH9_UsbSetup
 */
void constructCH9setup(const uint8_t *setupData, CH9_UsbSetup* ch9setup) {

    /* map the bytes to the struct values */
    ch9setup->bmRequestType = setupData[0];
    ch9setup->bRequest = setupData[1];
    ch9setup->wValue = getU16ValFromU8Ptr(&setupData[2]);
    ch9setup->wIndex = getU16ValFromU8Ptr(&setupData[4]);
    ch9setup->wLength = getU16ValFromU8Ptr(&setupData[6]);
}

/**
 * Function creates TRB for setup request - only host mode
 * @param res driver resources
 * @param setup pointer to setup packet
 * @param dataLength data length
 * @param dir data direction flag
 */
static void nbControlTransferSetup(USBSSP_DriverResourcesT *res, const CH9_UsbSetup* setup, uint16_t dataLength, uint8_t dir) {

    uint32_t trt = 0;

    /* setup TRB */
    res->ep0.enqueuePtr->dword0 = cpuToLe32(
        ((uint32_t) le16ToCpu(setup->wValue) << USBSSP_TRB_WVALUE_POS)
        | ((uint32_t) setup->bRequest << USBSSP_TRB_BREQUEST_POS)
        | (uint32_t) setup->bmRequestType);

    res->ep0.enqueuePtr->dword1 = cpuToLe32(
        ((uint32_t) le16ToCpu(setup->wLength) << USBSSP_TRB_WLENGTH_POS)
        | (uint32_t) le16ToCpu(setup->wIndex));

    res->ep0.enqueuePtr->dword2 = cpuToLe32((uint32_t) 8U); /* setup is always 8 bytes length */

    if (dataLength > 0U) {
        if (dir == 0U) {
            trt = USBSSP_TRB_SETUP_TRT_OUT_DATA << USBSSP_TRB_SETUP_TRT_POS;
        } else {
            trt = USBSSP_TRB_SETUP_TRT_IN_DATA << USBSSP_TRB_SETUP_TRT_POS;
        }
    }

    res->ep0.enqueuePtr->dword3 = cpuToLe32((uint32_t) (
                                                (USBSSP_TRB_SETUP_STAGE << USBSSP_TRB_TYPE_POS)
                                                | USBSSP_TRB_NORMAL_IDT_MASK
                                                | (uint32_t) res->ep0.toogleBit)
                                            | trt
                                            );
    updateQueuePtr(&res->ep0, 0U);
}

/**
 * function handles data phase of setup request in host mode
 * @param res driver resources
 * @param pdata pointer to data
 * @param dataLength data length
 * @param dir data direction flag
 */
static void nbControlTransferData(USBSSP_DriverResourcesT *res, const uint8_t *pdata, uint16_t dataLength, uint8_t dir) {

    /* set data buffer address */
    set64Value(
        &res->ep0.enqueuePtr->dword0,
        &res->ep0.enqueuePtr->dword1,
        cpuToLe64(get64PhyAddrOf8ptr(pdata))
        );

    /* set data length */
    res->ep0.enqueuePtr->dword2 = cpuToLe32((uint32_t) dataLength);

    /* set flags */
    res->ep0.enqueuePtr->dword3 = cpuToLe32((uint32_t) (
                                                (uint32_t) ((uint32_t) dir << USBSSP_TRANSFER_DIR_POS)
                                                | (USBSSP_TRB_DATA_STAGE << USBSSP_TRB_TYPE_POS)
                                                | (uint32_t) res->ep0.toogleBit)
                                            );

    updateQueuePtr(&res->ep0, 0U);
}

/**
 * Enqueues non-blocking control transfer request
 * @param[in] res driver resources
 * @param[in] setup keeps setup packet
 * @param[in] pdata pointer for data to send/receive
 */
void enqueueNBControlTransfer(USBSSP_DriverResourcesT *res, const CH9_UsbSetup* setup, const uint8_t *pdata) {

    uint8_t dir;
    uint16_t dataLength;
    uint8_t dataInFlag = 0U;

    /* see spec in 4.11.2.2 */
    dir = setup->bmRequestType & CH9_USB_EP_DIR_IN;
    /* check data phase direction: set 1 for IN and 0 for OUT */
    dir = (dir > 0U) ? 1U : 0U;

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> START NO BLOCKING CONTROL TRANSFER\n", res->instanceNo);
    dataLength = le16ToCpu(setup->wLength);

    /* handle setup stage without IOC */
    nbControlTransferSetup(res, setup, dataLength, dir);

    /* data TRB when exists */
    if (dataLength > 0U) {

        /* handle data stage without IOC */
        nbControlTransferData(res, pdata, dataLength, dir);
        if (dir > 0U) {
            dataInFlag = 1U;
        }
    }

    dir = (dataInFlag > 0U) ? 0U : 1U;

    /* status TRB - This is the only TRB with IOC */
    res->ep0.enqueuePtr->dword0 = 0;
    res->ep0.enqueuePtr->dword1 = 0;
    res->ep0.enqueuePtr->dword2 = 0;
    res->ep0.enqueuePtr->dword3 = cpuToLe32((uint32_t) (
                                                (uint32_t) ((uint32_t) dir << USBSSP_TRANSFER_DIR_POS)
                                                | (USBSSP_TRB_STATUS_STAGE << USBSSP_TRB_TYPE_POS)
                                                | USBSSP_TRB_NORMAL_IOC_MASK
                                                | (uint32_t) res->ep0.toogleBit)
                                            );

    updateQueuePtr(&res->ep0, 0U);
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> DRBL: Ring doorbell on EP0\n", res->instanceNo);
    USBSSP_WriteDoorbell(res, res->actualdeviceSlot, res->ep0.contextIndex);
}

/**
 * This function is called in ISR context to process the final get description control
 * transfer completion
 * @param res Pointer to driver private data
 * @param status Status of the transfer
 * @param eventPtr Pointer to the completed event
 */
void xhciGetDescXferComplete(USBSSP_DriverResourcesT *res, uint32_t status, const USBSSP_RingElementT * eventPtr) {

    uint32_t retStatus = CDN_EOK;
    if (status == CDN_EOK) {
        /* check result and translate from XHCI to Cadence error code */
        retStatus = getCompletionCode(eventPtr);
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Completion Code: %d\n", res->instanceNo, retStatus);
        if (retStatus == (uint32_t) USBSSP_TRB_COMPLETE_SUCCESS) {
            retStatus = CDN_EOK;
        }
    } else {
        retStatus = status;
    }

    /* this is the final stage of the aggregated transfer */
    if (res->ep0.aggregatedComplete != NULL) {
        USBSSP_Complete complete = res->ep0.aggregatedComplete;
        /* Clear the complete pointer since the callback
         * could queue in another blocking transfer */
        res->ep0.aggregatedComplete = NULL;
        complete(res, retStatus, eventPtr);
    }
}

/**
 * This function is called in ISR context to process the final get description control
 * transfer completion
 * @param res Pointer to driver private data
 * @param status Status of the transfer
 * @param eventPtr Pointer to the completed event
 */
void xhciGetShortCfgDescComplete(USBSSP_DriverResourcesT *res, uint32_t status, const USBSSP_RingElementT * eventPtr) {

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Get short configuration status(%d)\n", res->instanceNo, status);

    /* check if the request was success */
    if (status == CDN_EOK) {

        CH9_UsbSetup ch9setup;
        uint16_t confLength;
        uint8_t byte_l, byte_h;
        uint8_t setup[] = {
            0x00U, (uint8_t) CH9_USB_REQ_GET_DESCRIPTOR, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x02U
        };

        /* replace descriptor type with required one */
        setup[3] = (uint8_t) CH9_USB_DT_CONFIGURATION;
        setup[0] = (uint8_t) (CH9_USB_DIR_DEVICE_TO_HOST | CH9_USB_REQ_TYPE_STANDARD | CH9_USB_REQ_RECIPIENT_DEVICE);

        byte_l = res->ep0Buff[2];
        byte_h = res->ep0Buff[3];

        confLength = ((uint16_t) byte_h << 8) | (uint16_t) byte_l;

        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Whole configuration length: %d\n", res->instanceNo, confLength);

        /* write length in LE order */
        setup[6] = (uint8_t) confLength;
        setup[7] = (uint8_t) (confLength >> 8);
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Get long configuration\n", res->instanceNo);

        res->ep0.isRunningFlag = 1U;
        res->ep0.complete = &xhciGetDescXferComplete;
        constructCH9setup(&setup[0], &ch9setup);

        enqueueNBControlTransfer(res, &ch9setup, res->ep0Buff);

    } else if (res->ep0.aggregatedComplete != NULL) {
        USBSSP_Complete complete = res->ep0.aggregatedComplete;
        /* Clear the complete pointer since the callback
         * could queue in another blocking transfer */
        res->ep0.aggregatedComplete = NULL;
        complete(res, status, eventPtr);
    } else {
        /* Do nothing if error and no-aggregate callback is registered */
    }
}

/**
 * Called on completion of SetFeatureHost for clear feature.
 * @param res Pointer to driver private data
 * @param status Status of the transfer
 * @param eventPtr Pointer to the completed event
 */
static void epClearFeatureComplete(USBSSP_DriverResourcesT *res, uint32_t status, const USBSSP_RingElementT * eventPtr) {

    uint8_t epIndex = res->controlXferEpIndex;

    if (status == CDN_EOK) {
        USBSSP_EpContexEpState endpointState = getEndpointStatus(res, epIndex);
        /* put endpoint to run state */
        if (endpointState == USBSSP_EP_CONTEXT_EP_STATE_STOPPED) {
            USBSSP_WriteDoorbell(res, res->actualdeviceSlot, res->ep[epIndex].contextIndex);
        }
    } else if (eventPtr != NULL) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> epIndex: %02x status(%d) completionCode(%x)\n",
                res->instanceNo, epIndex, status, getCompletionCode(eventPtr));
    } else {
        /* required for MISRA compliance */
    }
}

/**
 * Sets endpoint feature in host mode
 * @param res Pointer to driver private data
 * @param epIndex endpoint index
 * @param feature
 */
void xhciEpSetFeatureHost(USBSSP_DriverResourcesT *res, uint8_t epIndex, uint8_t feature) {

    uint8_t epAddress;
    CH9_UsbSetup ch9setup;

    /* GET configuration descriptor pattern */
    uint8_t setup[] = {
        CH9_USB_DIR_HOST_TO_DEVICE | CH9_USB_REQ_TYPE_STANDARD | CH9_USB_REQ_RECIPIENT_ENDPOINT,
        CH9_USB_REQ_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    /*calculate endpoint address from ep_index */
    epAddress = (epIndex / 2U) | (((epIndex % 2U) != 0U) ? CH9_USB_EP_DIR_IN : 0U);
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Endpoint address: %02x\n", res->instanceNo, epAddress);

    /* replace endpoint selector with required one */
    setup[4] = epAddress;

    /* replace clear feature request with set feature request */
    if (feature > 0U) {
        setup[1] = CH9_USB_REQ_SET_FEATURE;
        res->ep0.complete = NULL; /* epSetFeatureComplete; */
    } else {
        res->controlXferEpIndex = epIndex;
        res->ep0.complete = epClearFeatureComplete;
    }

    constructCH9setup(&setup[0], &ch9setup);
    enqueueNBControlTransfer(res, &ch9setup, res->ep0Buff);

}


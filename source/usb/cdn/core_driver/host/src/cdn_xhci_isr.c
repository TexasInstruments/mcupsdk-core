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
 * cdn_xhci_isr.c
 * USB Host controller driver functions for interrupt handling
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

#if !(defined USBSSP_CONSECUTIVE_EVENTS)
#define USBSSP_CONSECUTIVE_EVENTS 10000U
#endif

#if 0
#define QUEUE_TEST
#endif

/**
 * Update event pointer
 * @param[in] res driver resources
 * @param[in] interrupter
 */
static void updateEventPtr(USBSSP_DriverResourcesT *res, uint32_t interrupter) {

    /* get address of last element in event ring */
    USBSSP_RingElementT *poolEnd = &res->xhciMemRes->eventPool[interrupter][USBSSP_EVENT_QUEUE_SIZE - 1U];

    CPS_CacheFlush((void*) res->eventPtrBuffer[interrupter], sizeof (USBSSP_RingElementT), 0);
    ++res->eventPtrBuffer[interrupter];

    if (res->eventPtrBuffer[interrupter] > poolEnd) {
        res->eventPtrBuffer[interrupter] = res->xhciMemRes->eventPool[interrupter];
        vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> Change toggle bit\n", res->instanceNo);
        if (res->eventToogleBit[interrupter] == 1U) {
            res->eventToogleBit[interrupter] = 0U;
        } else {
            res->eventToogleBit[interrupter] = 1U;
        }
    }
    res->eventPtr = res->eventPtrBuffer[interrupter];
}

/**
 * Function updates res->devConfigFlag
 * @param res driver resource
 * @param newCfgFlag flag to be set as actual
 */
void setDevConfigFlag(USBSSP_DriverResourcesT *res, uint8_t newCfgFlag) {
    uint8_t prev_cfg_flag = res->devConfigFlag;

    if (newCfgFlag != prev_cfg_flag) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Configured status changed from %d to %d\n",
                res->instanceNo, prev_cfg_flag, newCfgFlag);

        res->devConfigFlag = newCfgFlag;
    }
}

/**
 * Get previous TRB
 * @param queue The queue corresponding to this pointer.
 * @param actualTrb actual TRB
 * @return previous TRB
 */
static USBSSP_RingElementT * getPrevTrb(const USBSSP_ProducerQueueT *queue, USBSSP_RingElementT * actualTrb) {

    USBSSP_RingElementT * prevTrb = actualTrb;
    --prevTrb;

    /* check if TD overturn on ring */
    if (prevTrb < queue->ring) {
        prevTrb = &queue->ring[USBSSP_PRODUCER_QUEUE_SIZE - 2U];
        /* if cycle bit has not been changed return error */
        if (getToogleBit(prevTrb) == getToogleBit(actualTrb)) {
            prevTrb = NULL;
        }
    }
    return prevTrb;
}

/**
 * issue enable slot command
 * @param res driver resources
 * @return CDN_EOK on success, CDN_EINVAL on critical error
 */
static uint32_t issueEnableSlotCommand(USBSSP_DriverResourcesT *res) {
    uint32_t ret = CDN_EOK;
    /* enable slot if no actual device slot is active. */
    if (res->actualdeviceSlot == 0U) {
        ret = USBSSP_EnableSlot(res);
        if (ret == (uint32_t) CDN_EINVAL) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! wrong value in one of function parameters\n", 0);
        }
    }
    return ret;
}

/**
 * issue disable slot command
 * @param res driver resources
 * @return CDN_EOK on success, CDN_EINVAL on critical error
 */
static uint32_t issueDisableSlotCommand(USBSSP_DriverResourcesT *res) {
    uint32_t ret = CDN_EOK;
    /* disable slot if any device slot is active. */
    if (res->actualdeviceSlot > 0U) {
        ret = USBSSP_DisableSlot(res);
        if (ret == (uint32_t) CDN_EINVAL) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! wrong value in one of function parameters\n", 0);
        }
    }
    return ret;
}

/**
 * handle connect status change for connect case
 * @param res driver resources
 * @param portStatus port status
 */
static void connectStatusChangeConnect(USBSSP_DriverResourcesT *res, uint32_t portStatus) {
    uint32_t ret;

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Device connected on port: %d\n", res->instanceNo, res->actualPort);

    res->connected = 1U;

    /* check if port enabled */
    if ((portStatus & USBSSP__PORTSC1USB3__PED_MASK) > 0U) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Port %d ENABLED!\n", res->instanceNo, res->actualPort);
        ret = issueEnableSlotCommand(res);
    } else {
        /* port is disabled */
        uint32_t actualSpeed = CPS_FLD_READ(USBSSP__PORTSC1USB3, PORTSPEED, portStatus);

        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Port %d DISABLED!\n", res->instanceNo, res->actualPort);
        /* do reset for USB20 */
        if (actualSpeed < (uint32_t) CH9_USB_SPEED_SUPER) {
            if (res->usb2ResetCallback != NULL) {
                res->usb2ResetCallback(res);
            } else {

                CPS_DelayNs(USBSSP_DELAY_T3_DEBOUNCE);
                ret = USBSSP_ResetRootHubPort(res);
                if (ret == (uint32_t) CDN_EINVAL) {
                    vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! wrong value in one of function parameters\n", 0);
                }

                CPS_DelayNs(USBSSP_DELAY_T6_RECOVERY);
                ret = issueEnableSlotCommand(res);
            }
        }
    }
}

/**
 * Enables U1 for (t > 0), disables U1 for (t = 0)
 * @param res driver resources
 * @param t time
 */
static void setU1timeout(const USBSSP_DriverResourcesT *res, uint8_t t) {

    uint32_t regValue = xhciRead32(&res->regs.xhciPortControl[res->actualPort - 1U].portpmsc);
    if (t > 0U) {
        regValue |= 0x00000001U;
    } else {
        regValue &= ~0x00000001U;
    }
    xhciWrite32(&res->regs.xhciPortControl[res->actualPort - 1U].portpmsc, regValue);
}

/**
 * Enables U2 for (t > 0), disables U2 for (t = 0)
 * @param res driver resources
 * @param t time
 */
static void setU2timeout(const USBSSP_DriverResourcesT *res, uint8_t t) {

    uint32_t regValue = xhciRead32(&res->regs.xhciPortControl[res->actualPort - 1U].portpmsc);
    if (t > 0U) {
        regValue |= 0x00000100U;
    } else {
        regValue &= ~0x00000100U;
    }
    xhciWrite32(&res->regs.xhciPortControl[res->actualPort - 1U].portpmsc, regValue);
}

/**
 * handle connect status change for disconnect case
 * @param res driver resources
 */
static void connectStatusChangeDisconnect(USBSSP_DriverResourcesT *res) {
    /* device is disconnected */
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Device disconnected\n", res->instanceNo);

    /* set U1 nd U2 to default value */
    setU1timeout(res, 0);
    setU2timeout(res, 0);

    /* abort current command */
    res->connected = 0U;

    /* issue disable slot command */
    (void) issueDisableSlotCommand(res);

}

/**
 * Function handles connect status change event
 * @param res driver resources
 * @param portStatus port status read from portsc register
 */
static void connectStatusChange(USBSSP_DriverResourcesT *res, uint32_t portStatus) {

    /* check CCS bit, check if device is connected */
    if ((portStatus & USBSSP__PORTSC1USB3__CCS_MASK) != 0U) {
        connectStatusChangeConnect(res, portStatus);
    } else {
        connectStatusChangeDisconnect(res);
    }
}

/**
 * Function to invoke usb to phy soft reset
 * @param res driver resources
 */
static void checkInvokeUsbPhyReset(USBSSP_DriverResourcesT *res) {

    /* If Port Enabled and device connected, get the port speed */
    CH9_UsbSpeed actualSpeed = getSpeed(res);
    if ((actualSpeed <= CH9_USB_SPEED_HIGH)
        && (actualSpeed > CH9_USB_SPEED_UNKNOWN)
        && (res->usb2PhySoftReset != NULL)) {
        /* reset PHY */
        res->usb2PhySoftReset(res);
    }
}

/**
 * Function handles port reset change
 * @param res driver resources
 * @param portStatus port status read from portsc register
 */
static void portResetChange(USBSSP_DriverResourcesT *res, uint32_t portStatus) {

    uint32_t ret = CDN_EOK;
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Port reset change\n", res->instanceNo);

    /* set U1 and U2 timeouts to default value */
    setU1timeout(res, 0);
    setU2timeout(res, 0);

    /* check if port enabled */
    if ((portStatus & USBSSP__PORTSC1USB3__PED_MASK) != 0U) {

        if ((portStatus & USBSSP__PORTSC1USB3__CCS_MASK) != 0U) {
            checkInvokeUsbPhyReset(res);
        }

        /* check if EnableSlot command is not already pending */
        if (res->enableSlotPending == 0U) {

            if (res->actualdeviceSlot != 0U) {
                setSpeed(res);
                if (res->actualSpeed > CH9_USB_SPEED_HIGH) {
                    ret = USBSSP_ResetDevice(res);
                }
                if (ret != CDN_EOK) {
                    vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Could not reset controller\n", res->instanceNo);
                }
            } else {
                /* sometimes device is connected but event has not been generated */
                if ((portStatus & USBSSP__PORTSC1USB3__CCS_MASK) > 0U) {
                    res->connected = 1U;
                }
                /* Slot not enabled yet */
                ret = issueEnableSlotCommand(res);
            }
        }
    }
}

/**
 * Function handles port link state change
 * @param res driver resources
 * @param portStatus port status read from portsc register
 */
static void portLinkStateChange(USBSSP_DriverResourcesT *res, uint32_t portStatus) {

    /*TODO no enum values generated for PLS field! */
    if (CPS_FLD_READ(USBSSP__PORTSC1USB3, PLS, portStatus) == 15U /*RESUME*/) {

        uint32_t portsc;
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Device resumed from low power mode\n", res->instanceNo);

        /* PLS=U0 LWS=1 */
        portsc = xhciRead32(&res->regs.xhciPortControl[res->actualPort - 1U].portsc);
        portsc = CPS_FLD_WRITE(USBSSP__PORTSC1USB3, PLS, portsc, 0U); /*U0 state*/
        portsc = CPS_FLD_SET(USBSSP__PORTSC1USB3, LWS, portsc);
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> portsc: %08x\n", res->instanceNo, portsc & ~(USBSSP__PORTSC1USB3__PED_MASK));
        xhciWrite32(&res->regs.xhciPortControl[res->actualPort - 1U].portsc, portsc & ~(USBSSP__PORTSC1USB3__PED_MASK));
    }
}

/**
 * Port change detection. Function handles all changes on port
 * @param[in] res driver resources
 * @param[in] eventPtr event pointer
 * @return CDN_EOK for success, error otherwise
 */
static uint32_t portChangeDetect(USBSSP_DriverResourcesT *res, const USBSSP_RingElementT* eventPtr) {

    uint32_t counter = USBSSP_CONSECUTIVE_EVENTS;
    uint32_t ret = CDN_EOK;
    uint32_t portStatus;
    uint8_t portIndex;
    uint32_t maskAllChangeBits = USBSSP__PORTSC1USB3__CSC_MASK
                                 | USBSSP__PORTSC1USB3__PEC_MASK | USBSSP__PORTSC1USB3__WRC_MASK
                                 | USBSSP__PORTSC1USB3__PRC_MASK | USBSSP__PORTSC1USB3__PLC_MASK
                                 | USBSSP__PORTSC1USB3__CEC_MASK | USBSSP__PORTSC1USB3__OCC_MASK;

    res->actualPort = getPortId(eventPtr);
    portIndex = res->actualPort;
    portStatus = xhciRead32(&res->regs.xhciPortControl[res->actualPort - 1U].portsc);

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Port ID: %d, PORTSC: 0x%08X PORTPMSC: 0x%08X\n", res->instanceNo,
            res->actualPort, portStatus, xhciRead32(&res->regs.xhciPortControl[res->actualPort - 1U].portpmsc));

    do {
        /* clear all interrupts */
        xhciWrite32(&res->regs.xhciPortControl[res->actualPort - 1U].portsc, portStatus & ~(USBSSP__PORTSC1USB3__PED_MASK));

        /* check if max number of supported consecutive events is reached */
        if (counter == 0U) {
            ret = CDN_ETIMEDOUT;
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "%s() timeout after %d\n", __func__, USBSSP_CONSECUTIVE_EVENTS);
            break;
        }

        if (res->preportChangeDetectCallback != NULL) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Call user defined Port Status Change Event pre callback\n", res->instanceNo);
            res->preportChangeDetectCallback(portStatus, res->actualPort);
        }

        /* handle connect status change */
        if ((portStatus & USBSSP__PORTSC1USB3__CSC_MASK) != 0U) {
            connectStatusChange(res, portStatus);
        }

        /* handle port reset change */
        if ((portStatus & USBSSP__PORTSC1USB3__PRC_MASK) != 0U) {
            portResetChange(res, portStatus);
        }

        /* handle port link state change */
        if ((portStatus & USBSSP__PORTSC1USB3__PLC_MASK) != 0U) {
            portLinkStateChange(res, portStatus);
        }

        /* update portStatus - in meantime new change may have happen */
        portStatus = xhciRead32(&res->regs.xhciPortControl[portIndex - 1U].portsc);

        counter--;

    } while ((portStatus & maskAllChangeBits) > 0U);

    return ret;
}

/**
 * Set dequeue pointer of endpoint 0. Function sends
 * SET_TR_DEQUEUE_POINTER_COMMAND to SSP controller
 *
 * @param[in] res driver resources
 * @param[in] endpoint index to set dequeue pointer to
 */
static void setDequeuePointerEp0(USBSSP_DriverResourcesT *res, uint64_t newDequeuePtr) {

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> SET dequeue pointer of endpoint(%d) to 0x%08x%08x\n", res->instanceNo, USBSSP_EP0_CONTEXT_OFFSET, (uint32_t) ((newDequeuePtr & 0xFFFFFFFF00000000ULL) >> 32), (uint32_t) ((newDequeuePtr & 0xFFFFFFFFULL)));

    set64Value(
        &res->commandQ.enqueuePtr->dword0,
        &res->commandQ.enqueuePtr->dword1,
        cpuToLe64(newDequeuePtr | res->ep0.toogleBit)
        /*cpuToLe64(get64PhyAddrOf32ptr(&res->ep0.enqueuePtr->dword0) | res->ep0.toogleBit) */
        );

    res->commandQ.enqueuePtr->dword3 = cpuToLe32(
        (uint32_t) ((uint32_t) res->actualdeviceSlot << USBSSP_SLOT_ID_POS)
        | (uint32_t) ((uint32_t) USBSSP_EP0_CONTEXT_OFFSET << USBSSP_ENDPOINT_POS)
        | (uint32_t) (USBSSP_TRB_SET_TR_DQ_PTR_CMD << USBSSP_TRB_TYPE_POS)
        | res->commandQ.toogleBit
        );

    updateQueuePtr(&res->commandQ, 0U);
    hostCmdDoorbell(res);
}

/**
 * Completion handler for Enable Slot command
 * @param res driver resources
 */
static void commadCompletionEnableSlot(USBSSP_DriverResourcesT * res) {

    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> USBSSP_TRB_ENABLE_SLOT_COMMAND\n", res->instanceNo);
    res->actualdeviceSlot = getSlotId(res->eventPtr);
    if ((res->actualdeviceSlot >= res->maxDeviceSlot) || (res->actualdeviceSlot >= USBSSP_MAX_DEVICE_SLOT_NUM)) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> actualdeviceSlot (%d) greater than max slots (%d)\n",
                res->instanceNo, res->actualdeviceSlot, res->maxDeviceSlot);
    } else {
        res->xhciMemRes->dcbaa->deviceSlot[res->actualdeviceSlot - 1U] = cpuToLe64((uintptr_t) res->xhciMemRes->outputContext->slot);
    }
    res->enableSlotPending = 0U;
    res->contextEntries = 1; /* A0 and A1 enabled */
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> Actual dev. slotID: %d\n", res->instanceNo, res->actualdeviceSlot);
    setAddress(res, 0);
}

/**
 * Handle address device command completion
 * @param res driver resources
 */
static void commadCompletionAddressDev(USBSSP_DriverResourcesT * res) {

    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> USBSSP_TRB_ADDRESS_DEVICE_COMMAND\n", res->instanceNo);
    res->devAddress = 1;
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> Device in addressed state\n", res->instanceNo);
}

/**
 * Handle configure endpoint command completion
 * @param res driver resources
 */
static void commadCompletionConfigureEp(USBSSP_DriverResourcesT * res) {

    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> USBSSP_TRB_CONFIGURE_ENDPOINT_COMMAND\n", res->instanceNo);
    setDevConfigFlag(res, 1);
}

/**
 * Handle reset endpoint command completion
 * @param res driver resources
 * @param epIndex endpoint index
 */
static void commadCompletionResetEpSW(USBSSP_DriverResourcesT * res, uint8_t epIndex) {
    /* handle no default endpoint */
    /* get endpoint object */
    USBSSP_ProducerQueueT *ep = &res->ep[epIndex];
    /* call user complete callback */
    if (ep->xferStallError != 0U) {
        ep->xferStallError = 0U;
        if (ep->complete != NULL) {
            ep->complete(res, USBSSP_ESTALL, NULL);
        }
    }
}
/**
 * Handle reset endpoint command completion
 * @param res driver resources
 */
static void commadCompletionResetEp(USBSSP_DriverResourcesT * res) {

    uint8_t epIndex = getEndpoint(res->commandQ.dequeuePtr);
    USBSSP_EpContexEpState endpointState = getEndpointStatus(res, epIndex);

    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> USBSSP_TRB_RESET_ENDPOINT_COMMAND completed on ep %d\n", res->instanceNo, epIndex);
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Endpoint(%d) status: %d\n", res->instanceNo, epIndex, endpointState);

    /* set dequeue pointer command allowed only for stopped endpoint */
    if (endpointState == USBSSP_EP_CONTEXT_EP_STATE_STOPPED) {

        /* handle default endpoint */
        if (epIndex == USBSSP_EP0_CONTEXT_OFFSET) {

            USBSSP_RingElementT *dequeuePtr = NULL;
            uint32_t trbType = getTrbType(res->ep0.dequeuePtr);

            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Issue SetDequePointer for ep %d\n", res->instanceNo, epIndex);

            /* handle set dequeue pointer */
            switch (trbType) {
            case USBSSP_TRB_SETUP_STAGE:
                vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Stalled on SETUP stage\n", res->instanceNo);
                dequeuePtr = &res->ep0.dequeuePtr[3];
                break;
            case USBSSP_TRB_DATA_STAGE:
                vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Stalled on DATA stage\n", res->instanceNo);
                dequeuePtr = &res->ep0.dequeuePtr[2];
                break;
            case USBSSP_TRB_STATUS_STAGE:
                vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Stalled on STATUS stage\n", res->instanceNo);
                dequeuePtr = &res->ep0.dequeuePtr[1];
                break;
            default:
                vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Control transfer TRB mismatch\n", res->instanceNo);
                break;
            }

            /* update dequeue pointer */
            res->ep0.dequeuePtr = dequeuePtr;
            if (dequeuePtr == NULL) {
                vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Control transfer TRB mismatch, dequeuePtr=NULL\n", res->instanceNo);
            } else {
                setDequeuePointerEp0(res, le64ToCpu(get64PhyAddrOf32ptr(&dequeuePtr->dword0)));
            }
        } else {
            commadCompletionResetEpSW(res, epIndex);
        }
    }
}

/**
 * Handle set transfer dequeue pointer command completion
 * @param res driver resources
 */
static void commadCompletionSetTrDePtr(USBSSP_DriverResourcesT * res) {

    uint8_t epIndex = getEndpoint(res->commandQ.dequeuePtr);
    USBSSP_EpContexEpState endpointState = getEndpointStatus(res, epIndex);

    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> USBSSP_TRB_SET_TR_DEQUEUE_POINTER_COMMAND completed on ep %d\n", res->instanceNo, epIndex);
    if (epIndex == USBSSP_EP0_CONTEXT_OFFSET) {
        /*res->ep0.dequeuePtr = res->ep0.enqueuePtr; */
        USBSSP_WriteDoorbell(res, res->actualdeviceSlot, epIndex);
    } else {
        /*res->ep[epIndex].dequeuePtr = res->ep[epIndex].enqueuePtr; */
        if (endpointState == USBSSP_EP_CONTEXT_EP_STATE_STOPPED) {

            if (res->ep[epIndex].isRunningFlag != 0U) {
                /* ring doorbell to put endpoint in running state */
                vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> DRBL: Ring doorbell on ep_index: %d\n", res->instanceNo, epIndex);
                USBSSP_WriteDoorbell(res, res->actualdeviceSlot, epIndex);
            }
        }
    }
}

/**
 * Handle reset device command completion
 * @param res driver resources
 */
static void commadCompletionResetDevice(USBSSP_DriverResourcesT * res) {

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Reset device completed.\n", res->instanceNo);
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Device address: %d \n", res->instanceNo, res->devAddress);

    /* for host mode */
    if (res->devAddress == 0U) {
        setAddress(res, 0U);
    }
}

/**
 * Checks for the presence of set in array and returns the index
 * @param set
 * @param function array
 * @return
 */
static uint32_t remap(uint32_t command, uint32_t const *array, uint32_t size) {
    uint32_t i, res = 0;
    for (i = 0; i < size; i++) {
        if (array[i] == command) {
            res = i;
            break;
        } else {
            /*Returns the size(default case) */
            res = size;
        }

    }
    return res;
}

/**
 * handleNoOp
 * @param res  driver resources
 */
static void handleNoOp(USBSSP_DriverResourcesT * res) {
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> USBSSP_TRB_NO_OP_COMMAND\n", res->instanceNo);
    if (res->nopComplete != NULL) {
        res->nopComplete(res, CDN_EOK, NULL);
    }
}

/**
 * handleForceHeader
 * @param res  driver resources
 */
static void handleForceHeader(USBSSP_DriverResourcesT *res) {
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Force header completed.\n", res->instanceNo);
    if (res->forceHeaderComplete != NULL) {
        res->forceHeaderComplete(res, CDN_EOK, NULL);
    }

}

/**
 * handleDisableSlot
 * @param res driver resources
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Pass parameter res with const specifier, DRV-4333" */
/* parasoft-begin-suppress MISRA2012-RULE-2_7-4 "Parameter res not used in function handleDisableSlot, DRV-4328" */

static void handleDisableSlot(USBSSP_DriverResourcesT *res) {

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Disable slot completed.\n", res->instanceNo);
    return;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 */
/* parasoft-end-suppress MISRA2012-RULE-2_7-4 */

/**
 * handleHaltEndpoint
 * @param res driver resources
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Pass parameter res with const specifier, DRV-4334" */
/* parasoft-begin-suppress MISRA2012-RULE-2_7-4 "Parameter res not used in function handleHaltEndpoint, DRV-4329" */

static void handleHaltEndpoint(USBSSP_DriverResourcesT *res) {

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Halt endpoint(%d) completed.\n", res->instanceNo, getEndpoint(res->commandQ.dequeuePtr));
    return;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 */
/* parasoft-end-suppress MISRA2012-RULE-2_7-4 */

/**
 * handleHaltStopEndpoint
 * @param res driver resources
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Pass parameter res with const specifier, DRV-4335" */
/* parasoft-begin-suppress MISRA2012-RULE-2_7-4 "Parameter res not used in function handleHaltStopEndpoint, DRV-4330" */

static void handleHaltStopEndpoint(USBSSP_DriverResourcesT *res) {
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Stop endpoint(%d) completed.\n", res->instanceNo, getEndpoint(res->commandQ.dequeuePtr));
    return;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 */
/* parasoft-end-suppress MISRA2012-RULE-2_7-4 */

/**
 * handleForceEvent
 * @param res driver resources
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Pass parameter res with const specifier, DRV-4336" */
/* parasoft-begin-suppress MISRA2012-RULE-2_7-4 "Parameter res not used in function handleForceEvent, DRV-4331" */

static void handleForceEvent(USBSSP_DriverResourcesT *res) {
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Force event completed.\n", res->instanceNo);
    return;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 */
/* parasoft-end-suppress MISRA2012-RULE-2_7-4 */

/**
 * handleDefault
 * @param res driver resources
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Pass parameter res with const specifier, DRV-4337" */
/* parasoft-begin-suppress MISRA2012-RULE-2_7-4 "Parameter res not used in function handleDefault, DRV-4332" */

static void handleDefault(USBSSP_DriverResourcesT *res) {
    vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Unknown/not supported cmd ...\n", res->instanceNo);
    return;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 */
/* parasoft-end-suppress MISRA2012-RULE-2_7-4 */

/**
 * Handle successfully completed command
 * @param res driver resources
 * @param command command code
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Pass parameter res with const specifier, DRV-4338" */
static void commadCompletionSuccessfull(USBSSP_DriverResourcesT * res, uint32_t command) {
    uint32_t commands[] = {
        USBSSP_TRB_ENABLE_SLOT_COMMAND,
        USBSSP_TRB_ADDR_DEV_CMD,
        USBSSP_TRB_CONF_EP_CMD,
        USBSSP_TRB_NO_OP_COMMAND,
        USBSSP_TRB_RESET_EP_CMD,
        USBSSP_TRB_SET_TR_DQ_PTR_CMD,
        USBSSP_TRB_RESET_DEVICE_COMMAND,
        USBSSP_TRB_DISABLE_SLOT_COMMAND,
        USBSSP_TRB_FORCE_HEADER_COMMAND,
        USBSSP_TRB_HALT_ENDP_CMD,
        USBSSP_TRB_STOP_EP_CMD,
        USBSSP_TRB_FORCE_EVENT_COMMAND
    };

    uint32_t ch = remap(command, commands, (uint32_t) (sizeof (commands) / sizeof (uint32_t)));
    static void (*function_ptr[]) (USBSSP_DriverResourcesT*) = {
        commadCompletionEnableSlot,
        commadCompletionAddressDev,
        commadCompletionConfigureEp,
        handleNoOp,
        commadCompletionResetEp,
        commadCompletionSetTrDePtr,
        commadCompletionResetDevice,
        handleDisableSlot,
        handleForceHeader,
        handleHaltEndpoint,
        handleHaltStopEndpoint,
        handleForceEvent,
        handleDefault
    };
    (*function_ptr[ch])(res);
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 */

/**
 * Function handles command completion
 * @param res driver resources
 * @param event pointer
 */
static void handleXhciCommadCompletion(USBSSP_DriverResourcesT * res, USBSSP_RingElementT* eventPtr) {

    uint32_t command;
    uint32_t completionCode = getCompletionCode(res->eventPtr);

    /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "uintptr_t converted to USBSSP_RingElementT*, DRV-4285" */
    /* parasoft-begin-suppress MISRA2012-RULE-11_3-2 "uint32_t* converted to uintptr_t*, DRV-4279" */
    uint32_t addrLoCpu = le32ToCpu(eventPtr->dword0);

#if (defined PLATFORM_64_BIT)
    uint32_t addrHiCpu = le32ToCpu(eventPtr->dword1);
    uintptr_t addr = (((uint64_t) addrHiCpu) << 32) | addrLoCpu;
#else
    uintptr_t addr = addrLoCpu;
#endif

    res->commandQ.dequeuePtr = (USBSSP_RingElementT*) addr;
    /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */
    /* parasoft-end-suppress MISRA2012-RULE-11_3-2 */
    command = getTrbType(res->commandQ.dequeuePtr);
    res->commandQ.isRunningFlag = 0;
    res->commandQ.completePtr = eventPtr;
    res->commandQ.completionCode = (uint8_t) completionCode;

    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> USBSSP_TRB_COMMAND_COMPLETION_EVENT (cmd@%p, type=0x%02x):\n",
            res->instanceNo, (uintptr_t) (res->commandQ.dequeuePtr), command);

    /* check if completion is successful */
    if (completionCode == USBSSP_TRB_COMPLETE_SUCCESS) {
        commadCompletionSuccessfull(res, command);
    } else {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Command %d failed, code: %d\n",
                res->instanceNo, command, completionCode);
    }
}

/**
 * Function sets some transfer object parameters
 * @param res driver resources
 * @param transferObj transfer object
 */
static void setTransferObjInterParams(USBSSP_DriverResourcesT * res, USBSSP_ProducerQueueT * transferObj) {

    /* get endpoint ID */
    uint8_t endpointId = getEndpoint(res->eventPtr);

    /* store eventPtr as complete pTR */
    transferObj->completePtr = res->eventPtr;

    /* transfer is stopped */
    transferObj->isRunningFlag = 0U;

    res->lastEpIntIndex = endpointId;
}

/**
 * Function gets transfer obj
 * @param res driver resources
 * @param eventPtr event pointer
 * @param endpointID endpoint id
 * @return transfer object
 */
static USBSSP_ProducerQueueT *getTransferObj(USBSSP_DriverResourcesT * res, const USBSSP_RingElementT* eventPtr, uint8_t endpointId) {

    USBSSP_ProducerQueueT * transferObj;

    /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "uintptr_t converted to USBSSP_RingElementT*, DRV-4286" */
    /* parasoft-begin-suppress MISRA2012-RULE-11_3-2 "uint32_t* converted to uintptr_t*, DRV-4280" */
    uint32_t addrLoCpu = le32ToCpu(eventPtr->dword0);

#if (defined PLATFORM_64_BIT)
    uint32_t addrHiCpu = le32ToCpu(eventPtr->dword1);
    uintptr_t addr = (((uint64_t) addrHiCpu) << 32) | addrLoCpu;
#else
    uintptr_t addr = addrLoCpu;
#endif

    USBSSP_RingElementT *dequeuePtr = (USBSSP_RingElementT*) addr;
    /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */
    /* parasoft-end-suppress MISRA2012-RULE-11_3-2 */

    /* handle EP0 */
    if (endpointId == USBSSP_EP0_CONTEXT_OFFSET) {
        transferObj = &res->ep0;
        transferObj->dequeuePtr = dequeuePtr;
    } else {
        /* handle other endpoints */
        transferObj = &res->ep[endpointId];
        transferObj->dequeuePtr = dequeuePtr;
    }
    setTransferObjInterParams(res, transferObj);
    transferObj->completionCode = (uint8_t) getCompletionCode(eventPtr);
    return transferObj;
}

/**
 * Handle completion code for no default endpoint
 * @param res driver resources
 * @param transferObj pointer to endpoint transfer event
 */
static void handleEpCompletion(USBSSP_DriverResourcesT const * res, USBSSP_ProducerQueueT const * transferObj) {

    /* transfered data length sum */
    uint32_t userDataLenSum = 0;

    /* really transferred number of bytes */
    uint32_t realDataLenSum = 0;

    /* get number of residue bytes */
    uint32_t numOfResidue = getTrEvtTrbTransLen(res->eventPtr);

    /* used in for-each loop in TD parsing */
    USBSSP_RingElementT * ringIterator = transferObj->dequeuePtr;

    if (ringIterator != NULL) {
        uint8_t chainBit;
        do {
            uint32_t userTrbLength;
            uint32_t realTrbLength;

            /* recreate user data length */
            userDataLenSum += getTrTrbTransLen(ringIterator);

            /* get length of actual TRB */
            userTrbLength = getTrTrbTransLen(ringIterator);

            /* calculate real TRB length */
            realTrbLength = userTrbLength - numOfResidue;

            /* check if short packet has been received */
            if (realTrbLength < userTrbLength) {
                realDataLenSum = 0U;
            }

            /* calculate number of bytes really transferred */
            realDataLenSum += realTrbLength;

            /* get previous TRB and exit if NULL */
            ringIterator = getPrevTrb(transferObj, ringIterator);
            if (ringIterator == NULL) {
                break;
            }
            /* check chain bit */
            chainBit = getTrTrbChain(ringIterator);
        } while (chainBit > 0U);

        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> contextIndex:%d numOfBytes: %d, realDataLenSum: %d, numOfResidue: %d\n",
                res->instanceNo, transferObj->contextIndex, userDataLenSum, realDataLenSum, (userDataLenSum - realDataLenSum));
    }
}

/**
 * Handling transfer completion status cases
 * @param res driver resources
 * @param transferObj transfer object
 * @return 1 when user callback needs to be called, 0 elsewhere
 */
static uint8_t handleCompleteStatus(USBSSP_DriverResourcesT * res, USBSSP_ProducerQueueT * transferObj) {

    uint8_t ret = 0U;
    uint8_t epIndex = transferObj->contextIndex;

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> ContextIndex:%d  CompletionCode: %d\n",
            res->instanceNo, transferObj->contextIndex, transferObj->completionCode);

    /* handle different completion codes */
    switch (transferObj->completionCode) {

    /* for success */
    case USBSSP_TRB_COMPLETE_SUCCESS:
        ret = 1U;
        break;

    /* when endpoint stalled */
    case USBSSP_TRB_COMPLETE_STALL_ERROR:
        transferObj->xferStallError = 1U;
        transferObj->isDisabledFlag = 1U;
        (void) USBSSP_ResetEndpoint(res, epIndex);
        break;

    /* when short packet received */
    case USBSSP_TRB_CMPL_SHORT_PKT:

        if (transferObj->ignoreShortPacket == 0U) {
            ret = 1U;
        } else {
            ret = 0U;
            if (getTrTrbChain(transferObj->dequeuePtr) == 0U) {
                transferObj->ignoreShortPacket = 0;
            }
        }

        if (getTrTrbChain(transferObj->dequeuePtr) > 0U) {
            transferObj->ignoreShortPacket = 1U;
        }
        break;

    /* when missed service error */
    case USBSSP_TRB_CMPL_MISSED_SRV_ER:
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> MISSED_SERVICE_ERROR Endpoint(%d)\n",
                res->instanceNo, epIndex);
        /* since we always set IOC for the last TRB of the td, only call completion for the TRB with IOC */
        if (getTrTrbChain(transferObj->dequeuePtr) == 0U) {
            ret = 1U;
        }
        break;

    case USBSSP_TRB_CMPL_RING_UNDERRUN:
    case USBSSP_TRB_CMPL_RING_OVERRUN:
    case USBSSP_TRB_CMPL_NO_PNG_RSP_ER:
        ret = 1U;
        break;

    default:
        /* do nothing by default */
        break;
    }
    return ret;
}

/**
 * data transfer completion handler
 * @param res driver resources
 * @param eventPtr event pointer
 * @param endpointId ep index
 * @return CDN_EOK on success
 */
static uint32_t handleTransferEvent(USBSSP_DriverResourcesT * res, const USBSSP_RingElementT* eventPtr, uint8_t endpointId) {
    uint32_t ret = CDN_EOK;

    if ((endpointId < USBSSP_EP0_CONT_OFFSET) || (endpointId >= USBSSP_EP_CONT_MAX)) {
        ret = CDN_EINVAL;
    } else {
        uint8_t callbackFlag = 0U;

        /* get transfer object, it may be default endpoint, endpoint or stream object */
        USBSSP_ProducerQueueT * transferObj = getTransferObj(res, eventPtr, endpointId);

        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "contextIndex:%d  EvtEpId:%d  CmplCode:%d  Event->xferTrbAddr:0x%16x\n",
                transferObj->contextIndex,
                getEndpoint(eventPtr),
                getCompletionCode(eventPtr),
                (((uint64_t) eventPtr->dword1) << 32) | eventPtr->dword0
                );

        /* check transfer status */
        /* handle stall etc. */
        callbackFlag = handleCompleteStatus(res, transferObj);

        if (callbackFlag == 1U) {

            if (transferObj->contextIndex > USBSSP_EP0_CONTEXT_OFFSET) {
                handleEpCompletion(res, transferObj);
            }

            /* call user complete callback */
            if (transferObj->complete != NULL) {
                transferObj->complete(res, CDN_EOK, eventPtr);
            }
        }
    }
    return ret;
}

/**
 * check if slot ID is correct
 * @param trb TRB
 * @return CDN_EOK on success
 */
static uint32_t checkIfSlotIdCorrect(const USBSSP_RingElementT * trb) {

    uint32_t ret = CDN_EOK;
    uint32_t trbType = getTrbType(trb);

    /* check if slotId is within correct range */
    /* slot id is valid only for TRB's checked below */
    if ((trbType == USBSSP_TRB_TRANSFER_EVENT)
        || (trbType == USBSSP_TRB_CMD_CMPL_EVT)
        || (trbType == USBSSP_TRB_BNDWTH_RQ_EVT)
        || (trbType == USBSSP_TRB_DOORBELL_EVENT)
        || (trbType == USBSSP_TRB_DEV_NOTIFCN_EVT)) {
        uint32_t value = (uint32_t) getSlotId(trb);

        /* check if slot id is within correct range */
        if (value > USBSSP_MAX_DEVICE_SLOT_NUM) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Slot ID(%d) exceeded USBSSP_MAX_DEVICE_SLOT_NUM\n", value);
            ret = CDN_EINVAL;
        }
    }
    return ret;
}

/**
 * Validate whether a dequeue pointer is correct. Works only for contiguous queues
 *
 * @param queue The queue corresponding to this pointer.
 * @param dequeueAddr De-queue pointer to validate.
 * @return CDN_EOK if dequeue-pointer lies in the given queue.
 */
static uint32_t validateDequeuePtr(const USBSSP_ProducerQueueT *queue, uintptr_t dequeueAddr) {

    uint32_t ret = CDN_EOK;
    USBSSP_RingElementT *poolEnd = &queue->ring[(USBSSP_PRODUCER_QUEUE_SIZE - 1U)];
    uintptr_t poolStartAddr = (uintptr_t) get64PhyAddrOf32ptr(&queue->ring[0].dword0);
    uintptr_t poolEndAddr = (uintptr_t) get64PhyAddrOf32ptr(&poolEnd->dword0);

    if ((dequeueAddr < poolStartAddr) || (dequeueAddr > poolEndAddr)) {
        ret = CDN_EINVAL;
    }
    return ret;
}

/**
 * Check whether a transfer event TRB is a valid TRB
 * @param res Pointer to driver resources
 * @param trb Pointer to the TRB
 * @param endpointId ep index
 * @return CDN_EOK on success
 */
static uint32_t checkTranferEventTRB(const USBSSP_DriverResourcesT * res, const USBSSP_RingElementT * trb, uint8_t endpointId) {

    uint32_t ret = CDN_EOK;

    /* parasoft-begin-suppress MISRA2012-RULE-11_3-2 "uint32_t* converted to uintptr_t*, DRV-4281" */
    uint32_t addrLoCpu = le32ToCpu(trb->dword0);

#if (defined PLATFORM_64_BIT)
    uint32_t addrHiCpu = le32ToCpu(trb->dword1);
    uintptr_t dequeueAddr = (((uint64_t) addrHiCpu) << 32) | addrLoCpu;
#else
    uintptr_t dequeueAddr = addrLoCpu;
#endif
    /* parasoft-end-suppress MISRA2012-RULE-11_3-2 */

    if (endpointId == USBSSP_EP0_CONTEXT_OFFSET) {
        ret = validateDequeuePtr(&res->ep0, dequeueAddr);
    } else if (endpointId < USBSSP_EP_CONT_MAX) {
        uint32_t completionCode = getCompletionCode(trb); /* returns code within [0:255] */
        /* check if event NOT triggered by stream */
        if ((getEDbit(trb) == 0U)
            && (completionCode != USBSSP_TRB_CMPL_RING_UNDERRUN)
            && (completionCode != USBSSP_TRB_CMPL_RING_OVERRUN)
            && (completionCode != USBSSP_TRB_CMPL_VF_EVTRNGFL_ER)
            && (completionCode != USBSSP_TRB_CMPL_STOP_LEN_INV)) {

            ret = validateDequeuePtr(&res->ep[endpointId], dequeueAddr);
        }
    } else {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Check whether a Command Completion event TRB is a valid TRB
 * @param res Pointer to driver resources
 * @param trb Pointer to the TRB
 * @return CDN_EOK on success
 */
static uint32_t checkCmdCmplEvtTRB(const USBSSP_DriverResourcesT * res, const USBSSP_RingElementT * trb) {

    /* parasoft-begin-suppress MISRA2012-RULE-11_3-2 "uint32_t* converted to uintptr_t*, DRV-4282" */
    uint32_t addrLoCpu = le32ToCpu(trb->dword0);

#if (defined PLATFORM_64_BIT)
    uint32_t addrHiCpu = le32ToCpu(trb->dword1);
    uintptr_t dequeueAddr = (((uint64_t) addrHiCpu) << 32) | addrLoCpu;
#else
    uintptr_t dequeueAddr = addrLoCpu;
#endif
    /* parasoft-end-suppress MISRA2012-RULE-11_3-2 */

    uint32_t ret = validateDequeuePtr(&res->commandQ, dequeueAddr);

    return ret;
}

/**
 * Check whether the Event TRB is valid
 *
 * @param res Pointer to driver resources
 * @param trb Pointer to the Event TRB
 * @return CDN_EOK if valid Event TRB
 */
static uint32_t checkEventTrb(const USBSSP_DriverResourcesT * res, const USBSSP_RingElementT * trb) {

    uint32_t trbType;
    uint32_t value;
    uint32_t result = CDN_EOK;

    /* check if TRB type field in within correct value */
    trbType = getTrbType(trb);
    if (trbType > USBSSP_TRB_MFINDEX_WRAP_EVENT) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "INCORRECT TRB Type value: %d\n", trbType);
        result = CDN_EINVAL;
    }

    if (result == CDN_EOK) {
        /* check if completion is within correct range */
        value = getCompletionCode(trb); /* returns code within [0:255] */
        if ((value > USBSSP_TRB_CMPL_SPLT_TRNSCN_ER) &&
            (value < USBSSP_TRB_CMPL_CDNSDEF_ERCODES)) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "INCORRECT COMPLETION value: %d\n", value);
            result = CDN_EINVAL;
        }
    }

    if (result == CDN_EOK) {
        result = checkIfSlotIdCorrect(trb);
    }

    if (result == CDN_EOK) {
        if (trbType == USBSSP_TRB_TRANSFER_EVENT) {
            result = checkTranferEventTRB(res, trb, getEndpoint(trb));
        } else if (trbType == USBSSP_TRB_CMD_CMPL_EVT) {
            result = checkCmdCmplEvtTRB(res, trb);
        } else {
            /*
             * All 'if ... else if' constructs shall be terminated with an 'else' statement
             * (MISRA2012-RULE-15_7-3)
             */
        }
    }
    return result;
}

/**
 * Handles various events
 * @param res
 * @param eventPtr
 * @param trbType
 * @return CDN_EOK on success, error otherwise
 */
static uint32_t switchFunction(USBSSP_DriverResourcesT * res, USBSSP_RingElementT* eventPtr, uint32_t trbType) {
    uint32_t ret = CDN_EOK;
    /* check event type */
    switch (trbType) {

    case USBSSP_TRB_PORT_ST_CHG_EVT:
#ifdef QUEUE_TEST
        noOpTest(res);
#else
        /* handle events on port */
        ret = portChangeDetect(res, eventPtr);
#endif
        break;

    /* handle command completion */
    case USBSSP_TRB_CMD_CMPL_EVT:
        handleXhciCommadCompletion(res, eventPtr);
        break;

    /* handle transfer event */
    case USBSSP_TRB_TRANSFER_EVENT:
        ret = handleTransferEvent(res, eventPtr, getEndpoint(eventPtr));
        break;

    /* handle host controller event */
    case USBSSP_TRB_HOST_CTRL_EVT:
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Host Controller ERROR:\n", res->instanceNo);
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Completion Code: %d\n",
                res->instanceNo, le32ToCpu(eventPtr->dword2) >> USBSSP_COMPLETION_CODE_POS);
        break;

    default:
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> TRB... %d\n", res->instanceNo, trbType);
        break;
    }
    return ret;
}

/**
 * Clears USBSTS Int event
 * @param res
 */
static void isrClearUSBSTSIntEvent(const USBSSP_DriverResourcesT * res) {

    /* Get the ISR status word*/
    uint32_t *usbstsRegPtr = &res->regs.xhciOperational->usbsts;

    uint32_t reg = xhciRead32(usbstsRegPtr);
    reg &= ~(USBSSP__USBSTS__SRE_MASK | USBSSP__USBSTS__PCD_MASK | USBSSP__USBSTS__HSE_MASK);
    reg |= USBSSP__USBSTS__EINT_MASK;

    xhciWrite32(usbstsRegPtr, reg);
}

/**
 * Checks if interrupt is pending
 * @param res Pointer to the driver resources
 * @param interrupter Index of the interrupter
 * @return interrupt pending bit value
 */
static uint32_t isrCheckInterruptPending(const USBSSP_DriverResourcesT * res, uint32_t interrupter) {

    uint32_t interruptPending = 0U;
    /* get pointer to the interrupter registers*/
    USBSSP_InterrupterT* xhciInterrupter = &res->regs.xhciInterrupter[interrupter];

    /* read Interrupter Management for this interrupter */
    uint32_t regVal = xhciRead32(&(xhciInterrupter->iman));
    interruptPending = CPS_FLD_READ(USBSSP__IMAN0, IP, regVal);

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Interrupter(%d) IMAN: %d interruptPending: %d \n",
            res->instanceNo, interrupter, regVal, interruptPending);

    return interruptPending; /* return interrupt pending bit value */
}

/**
 * Process ISR event and update event read pointer
 * @param res driver resources
 * @param interrupter Index of the interrupter
 * @return CDN_EOK on success, error otherwise
 */
static uint32_t isrProcessEvent(USBSSP_DriverResourcesT * res, uint32_t interrupter) {
    uint32_t trbType;
    uint32_t retVal = CDN_EOK;
    USBSSP_RingElementT* eventPtr = res->eventPtrBuffer[interrupter];

    retVal = checkEventTrb(res, eventPtr);

    /* call generic callback if defined by user - used for diagnostic */
    /* purposes */
    if ((res->genericCallback != NULL) && (retVal == CDN_EOK)) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI,
                "<%d> Interrupter(%d) Call user defined generic pre callback\n",
                res->instanceNo, interrupter);
        retVal = res->genericCallback(eventPtr);
    }

    /* If non-zero return from genericCallback skip to next iteration. */
    if (retVal == CDN_EOK) {

        trbType = getTrbType(eventPtr);

        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI,
                "<%d> Processing Interrupter(%d) trbType(%d)\n",
                res->instanceNo, interrupter, trbType);
        retVal = switchFunction(res, eventPtr, trbType);

        if (res->postCallback != NULL) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI,
                    "<%d> Interrupter(%d) Call user defined generic post callback\n",
                    res->instanceNo, interrupter);
            res->postCallback(eventPtr);
        }
    }

    /* Update event pointer */
    updateEventPtr(res, interrupter);

    return retVal;
}

/**
 * Handle interrupter
 * @param res driver resources
 * @param interrupter Index of the interrupter
 * @return CDN_EOK on success, error otherwise
 */
static uint32_t isrHandleInterrupter(USBSSP_DriverResourcesT * res, uint32_t interrupter) {

    uint32_t ret = CDN_EOK;
    uint32_t counter = USBSSP_CONSECUTIVE_EVENTS;
    USBSSP_InterrupterT* xhciInterrupter = &res->regs.xhciInterrupter[interrupter];

    CPS_CacheInvalidate((void*) res->eventPtrBuffer[interrupter], sizeof (USBSSP_RingElementT), 0);

    /* check babble interrupt */
    if (getToogleBit(res->eventPtrBuffer[interrupter]) != res->eventToogleBit[interrupter]) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Babble interrupt!  interrupter(%d)\n",
                res->instanceNo, interrupter);
        ret = CDN_EPROTO;
    }
    if (ret == CDN_EOK) {
        do {
            /* process the event */
            ret = isrProcessEvent(res, interrupter);

            /* check if max number of consecutive events is reached */
            if (counter == 0U) {
                ret = CDN_ETIMEDOUT;
                vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "%s() timeout after %d\n", __func__, USBSSP_CONSECUTIVE_EVENTS);
                break;
            }
            counter--;

            /* check if any pending event still on event ring and handle it if yes */
        } while (getToogleBit(res->eventPtrBuffer[interrupter]) == res->eventToogleBit[interrupter]);

        /* Program the Interrupter Event Ring Dequeue Pointer (ERDP) */
        xhciWrite64(&xhciInterrupter->erdp,
                    get64PhyAddrOf32ptr(&res->eventPtrBuffer[interrupter]->dword0) | USBSSP__ERDP0_LO__EHB_MASK);
    }
    return ret;
}

/**
 * Handling of SSP controller interrupt. Function is called from SSP interrupt
 * context.
 *
 * @param[in] res driver resources
 * @return CDN_EOK on success, error otherwise
 */
uint32_t USBSSP_Isr(USBSSP_DriverResourcesT * res) {

    uint32_t ret = USBSSP_IsrSF(res);

    if (ret == CDN_EOK) {

        uint32_t interrupter = 0U;

        vDbgMsg(USBSSP_DBG_DRV, DBG_INFLOOP, "<%d> IRQ-ENTER \n", res->instanceNo);

        isrClearUSBSTSIntEvent(res);

        for (interrupter = 0; interrupter < USBSSP_MAX_NUM_INTERRUPTERS; interrupter++) {
            USBSSP_InterrupterT* xhciInterrupter = &res->regs.xhciInterrupter[interrupter];

            if (isrCheckInterruptPending(res, interrupter) != 0U) {

                vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Processing Interrupter:(%d)\n", res->instanceNo, interrupter);
                res->eventPtr = res->eventPtrBuffer[interrupter]; /* this is not re-entrant */

                ret = isrHandleInterrupter(res, interrupter);
                /* Enable the Interrupter by writing a '1' to the Interrupt Pending (IP) */
                xhciWrite32(&xhciInterrupter->iman,
                            USBSSP__IMAN0__IE_MASK | USBSSP__IMAN0__IP_MASK);
            }
        }
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> IRQ-EXIT \n", res->instanceNo);
    }
    return ret;
}

/*-------------------------------------------------------------------------- */

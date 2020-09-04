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
 * cdn_xhci.c
 * USB Host controller driver,
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

#if !(defined USBSSP_DEFAULT_TIMEOUT)
#define USBSSP_DEFAULT_TIMEOUT 1000000U
#endif

#if defined USBSSP_DEMO_TB
uint32_t USBSSP_SetMemResCallback(USBSSP_DriverResourcesT *res);
#endif

static uint32_t checkStructAlign(const char* name, uintptr_t startPhysAddr, size_t byteSize,
                                 size_t alignBytes, size_t pageBytes) {

    /* calculate end of physical address */
    uintptr_t endPhysAddr = (startPhysAddr + (uintptr_t) byteSize) - (uintptr_t) 1U;
    uint32_t result = CDN_EOK;

    if (name != NULL) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Structure \"%s\", %d bytes @[0x%p...0x%p]\n",
                name, (uint64_t) byteSize, startPhysAddr, endPhysAddr);
    }
    /* check alignment violation */
    if ((alignBytes > 1U) && ((startPhysAddr & ((uintptr_t) alignBytes - 1U)) != 0U)) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "[ERROR] \"%s\" not aligned to %d bytes\n",
                name, alignBytes);
        result = USBSSP_EALIGN;
    }

    /* check page size violation */
    if (pageBytes > 1U) {
        uintptr_t page_no_bit_mask = ~(uintptr_t) (pageBytes - 1U);

        if ((startPhysAddr & page_no_bit_mask) != (endPhysAddr & page_no_bit_mask)) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "[ERROR] \"%s\" cannot fit single %d byte page\n",
                    name, pageBytes);
            result = USBSSP_EPAGE;
        }
    }

    return result;
}

/*local functions*/
static void configureEndpoints(USBSSP_DriverResourcesT *res, uint8_t const *conf);

#ifdef DEBUG
/**
 * Functions check if driver declared register width is equal to platform settings
 * @return CDN_EOK is driver and platform are aligned, CDN_EINVAL elsewhere
 */
static uint32_t checkAddrWidth(void) {
    uint32_t ret = CDN_EOK;

    /* check width: 32 or 64 bit platform */
    if (sizeof (void*) == (uint32_t) 8U) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Platform 64bit\n", 0);
#ifndef PLATFORM_64_BIT
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Platform width mismatch: platform 64 bit, driver 32 bit\n", 0);
        ret = CDN_EINVAL;
#endif
    } else {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Platform 32bit\n", 0);
#ifdef PLATFORM_64_BIT
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Platform width mismatch: platform 32 bit, driver 64 bit\n", 0);
        ret = CDN_EINVAL;
#endif
    }
    return ret;
}

/**
 * Function check if endian type of driver setting is correct
 * @return CDN_EOK if endian is OK, CDN_EINVAL elsewhere
 */
static uint32_t checkEndianness(void) {
    uint32_t ret = CDN_EOK;

    /* write pattern to memory */
    uint32_t value32b = 0x87654321U;

    /* check byte from memory */
    if ((*(uint8_t*) (&value32b)) == 0x21U) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Platform endian: LITTLE\n", 0);
        /* check endian mismatch */
#ifdef CPU_BIG_ENDIAN
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Endian mismatch: platform little, driver big\n", 0);
        ret = CDN_EINVAL;
#endif
    } else {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Platform endian: BIG\n", 0);
        /* check endian mismatch */
#ifndef CPU_BIG_ENDIAN
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Endian mismatch: platform big, driver little\n", 0);
        ret = CDN_EINVAL;
#endif
    }
    return ret;
}
#endif

/**
 * Waits for value
 * @param value address of checked variable
 * @param waitFor value for which we wait for
 * @param timeout timeout value
 * @return CDN_EOK on success or CDN_ETIMEDOUT on timeout
 */
static uint32_t waitForVal(volatile uint8_t const *value, uint32_t waitFor, uint32_t timeout) {
    uint32_t counter = timeout;
    uint32_t ret = CDN_EOK;

    while (*value != waitFor) {

        /* break loop if timeout occur */
        if (counter == 0U) {
            ret = CDN_ETIMEDOUT;
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "%s() timeout after %d\n", __func__, timeout);
            break;
        }
        counter--;
        CPS_DelayNs(1000);
    }
    return ret;
}

/**
 * Function wait for register value
 * @param reg address of checked register
 * @param mask bitmap of relevant bits taken for checking
 * @param waitFor value of which we wait for
 * @param timeout timeout value
 * @return CDN_EOK on success or CDN_ETIMEDOUT on timeout
 */
static uint32_t waitForReg(volatile uint32_t *reg, uint32_t mask, uint32_t waitFor, uint32_t timeout) {
    uint32_t counter = timeout;
    uint32_t ret = CDN_EOK;

    while ((xhciRead32(reg) & mask) != waitFor) {

        /* break loop when timeout value is 0 */
        if (counter == 0U) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "%s() timeout after %d\n", __func__, timeout);
            ret = CDN_ETIMEDOUT;
            break;
        }
        counter--;
        CPS_DelayNs(1000);
    }
    return ret;
}

/**
 * Function returns on endpoint's stopped or disabled state or on timeout
 * @param res driver resources
 * @param timeout value of operation timeout
 * @return CDN_EOK on success, ETIMEOUT when timeout generated return
 */
static uint32_t waitUntilEpStoppedDisabled(USBSSP_DriverResourcesT const * res, uint32_t timeout) {

    uint32_t counter = timeout;
    uint32_t ret = CDN_ETIMEDOUT;
    uint32_t flags = 0U;

    /* do constant checking endpoint state, and break loop only if all endpoints */
    /* are in stopped or disabled state, or break loop if timeout counter zeroed */
    do {
        uint8_t epIndex;
        flags = 0U;
        for (epIndex = 1U; epIndex <= 31U; epIndex++) {
            USBSSP_EpContexEpState epState = getEndpointStatus(res, epIndex);
            if ((epState != USBSSP_EP_CONTEXT_EP_STATE_DISABLED)
                && (epState != USBSSP_EP_CONTEXT_EP_STATE_STOPPED)
                && (epState != USBSSP_EP_CONTEXT_EP_STATE_ERROR)) {
                flags |= ((uint32_t) 1 << epIndex);
            }
        }
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Waiting to stop endpoints Flags = 0x%x\n", flags);

        counter--;

        CPS_DelayNs(1000);
    } while ((counter > 0U) && (flags != 0U));

    /* check the reason which loop has ended its work and return suitable status */
    if (flags == 0U) {
        ret = CDN_EOK;
    } else {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Error: Timed out waiting to stop endpoints Flags = 0x%x\n", flags);
    }
    return ret;
}

/**
 * Function checks whether the specified ep is enabled for data transfer
 * @param res driver resources
 * @param[in] epIndex index of endpoint according to xhci spec e.g for ep1out
              epIndex=2, for ep1in epIndex=3, for ep2out epIndex=4 end so on
 * @return  CDN_EOK on success
 *          CDN_EIO on not connected error
 *          CDN_ENOTSUP if EP not configured for transfer
 */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
static uint32_t checkEpXferEnabled(USBSSP_DriverResourcesT const * res, uint8_t epIndex) {
    uint32_t ret = CDN_EOK;

    /* operation not permitted on disconnected device */
    if ((res->devAddress == 0U) || (res->connected == 0U)) {
        ret = CDN_EIO;
    } else if (epIndex > 1U) {
        /* check endpoint descriptor */
        if (res->ep[epIndex].epDesc[0] != CH9_USB_DS_ENDPOINT) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Error: Endpoint index %d not initialized correctly\n", res->instanceNo, epIndex);
            ret = CDN_ENOTSUP;
        } else if (res->devConfigFlag == 0U) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Error: Endpoint index %d NOT configured \n", res->instanceNo, epIndex);
            ret = CDN_ENOTSUP;
        } else {
            /* Else required by MISRA*/
        }
    } else {
        /* For EP0 check if last transfer is complete
        * since Ep0 supports only control transfer */
        if (res->ep0.isRunningFlag == 1U) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Error: EP0 is already running \n", res->instanceNo);
            ret = CDN_ENOTSUP;
        }
    }

    return ret;
}
/* parasoft-end-suppress METRICS-36-3 */

/**
 * No Operation test. Function used for testing purposes: NO_OP_COMMAND is send to
 * SSP controller. When event ring receives NO_OP_COMMAND complete it calls complete
 * callback
 *
 * @param[in] res driver resources
 * @complete complete callback pointer
 */
uint32_t USBSSP_NoOpTest(USBSSP_DriverResourcesT *res, USBSSP_Complete complete) {

    /* check if res is not NULL */
    uint32_t ret = USBSSP_NoOpTestSF(res);

    if (ret != (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
    }

    if (ret == CDN_EOK) {
        /* set complete callback */
        res->nopComplete = complete;
        noOpTest(res);
    }

    return ret;
}

/**
 * Control transfer. Function executes control transfer. Information about transfer
 * like: data direction, data length, wIndex, wValue etc. are passed in 'setup'
 * parameter. No blocking version, result is returned to callback function
 *
 * @param[in] res driver resources
 * @param[in] setup keeps setup packet
 * @param[in] pdata pointer for data to send/receive
 * @param[in] complete_code XHCI transfer complete status code
 * @return CDN_EOK on success
 *
 */
uint32_t USBSSP_NBControlTransfer(USBSSP_DriverResourcesT *res, const CH9_UsbSetup* setup, const uint8_t *pdata, USBSSP_Complete complete) {

    uint32_t ret = USBSSP_NBControlTransferSF(res, setup);
    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
    } else {
        /* check if transfers are enabled on this endpoint */
        ret = checkEpXferEnabled(res, 1);
    }

    if (ret == CDN_EOK) {
        res->ep0.complete = complete;
        res->ep0.isRunningFlag = 1;
        enqueueNBControlTransfer(res, setup, pdata);
    }
    return ret;
}

/**
 * Handling of hub request
 * @param descType descriptor type
 * @param setup pointer to setup request
 */
static void getDescriptorHubReq(uint8_t descType, uint8_t *setup) {

    /* Hub requests are of Type Class. */
    if (descType == CH9_USB_DT_USB2_HUB) {
        setup[0] = (uint8_t) (CH9_USB_DIR_DEVICE_TO_HOST | CH9_USB_REQ_TYPE_CLASS | CH9_USB_REQ_RECIPIENT_DEVICE);
        /* length in little endian on bytes: 6 and 7 */
        setup[6] = CH9_USB_DS_USB2_HUB;
        setup[7] = 0x0U;
    } else if (descType == CH9_USB_DT_USB3_HUB) {
        setup[0] = (uint8_t) (CH9_USB_DIR_DEVICE_TO_HOST | CH9_USB_REQ_TYPE_CLASS | CH9_USB_REQ_RECIPIENT_DEVICE);
        /* length in little endian on bytes: 6 and 7 */
        setup[6] = CH9_USB_DS_USB3_HUB;
        setup[7] = 0x0U;
    } else {
        setup[0] = (uint8_t) (CH9_USB_DIR_DEVICE_TO_HOST | CH9_USB_REQ_TYPE_STANDARD | CH9_USB_REQ_RECIPIENT_DEVICE);
    }
}

/**
 * Function gets short configuration
 * @param res driver resources
 * @param setup pointer to setup request
 * @return CDN_EOK if success, error code elsewhere
 */
static uint32_t getDescriptorShortConf(USBSSP_DriverResourcesT *res, uint8_t *setup) {

    uint32_t ret = CDN_EOK;
    CH9_UsbSetup ch9setup;

    /* length in little endian on bytes: 6 and 7 */
    setup[6] = CH9_USB_DS_CONFIGURATION;
    setup[7] = 0;

    constructCH9setup(&setup[0], &ch9setup);
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Get short configuration\n", res->instanceNo);
    enqueueNBControlTransfer(res, &ch9setup, res->ep0Buff);

    return ret;
}

/**
 * Get descriptor. Function gets descriptor from connected device, used in host
 * mode and stores it in internal res->ep0Buff buffer. Maximal descriptor length
 * is limited to 255. Function is blocking type and must not be called from
 * interrupt context.
 *
 * @param[in] res driver resources
 * @param[in] descType type of descriptor to get (CH9_USB_DT_DEVICE, CH9_USB_DT_CONFIGURATION,...)
 * @param[in] complete Complete callback function
 *
 * @return CDN_EOK on success
 * @return complete_code XHCI transfer complete status code
 */
uint32_t USBSSP_GetDescriptor(USBSSP_DriverResourcesT *res, uint8_t descType, USBSSP_Complete complete) {

    uint32_t ret = USBSSP_GetDescriptorSF(res);

    if ((ret != (uint32_t) CDN_EOK) || (complete == NULL)) {
        ret = CDN_EINVAL;
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! wrong value in one of function parameters\n", 0);
    }

    if (ret == CDN_EOK) {
        /* check if transfers are enabled on this endpoint */
        ret = checkEpXferEnabled(res, 1);
    }

    if (ret == CDN_EOK) {
        /* GET configuration descriptor template with maximal length = 255 */
        uint8_t setup[] = {
            0x00U, (uint8_t) CH9_USB_REQ_GET_DESCRIPTOR, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x02U
        };

        /* replace descriptor type with required one */
        setup[3] = descType;

        /* handle hub requests */
        getDescriptorHubReq(descType, setup);

        /* configuration descriptor should be handled in two steps: */
        /* 1. get short configuration = 9 bytes */
        /* 2. get long configuration with length encoded in short configuration */
        res->ep0.isRunningFlag = 1U;
        res->ep0.aggregatedComplete = complete;

        if (descType == CH9_USB_DT_CONFIGURATION) {
            /* handle short configuration */
            res->ep0.complete = &xhciGetShortCfgDescComplete;
            ret = getDescriptorShortConf(res, setup);
        } else  {
            CH9_UsbSetup ch9setup;
            constructCH9setup(&setup[0], &ch9setup);
            res->ep0.complete = &xhciGetDescXferComplete;
            enqueueNBControlTransfer(res, &ch9setup, res->ep0Buff);
        }
    }

    return ret;
}

/**
 * Set feature on device's endpoint. Functions sends setup requested to device
 * with set/cleared endpoint feature
 *
 * @param[in] res driver resources
 * @param[in] epIndex index of endpoint to set/clear feature on
 * @param[in] feature when 1 sets stall, when 0 clears stall
 *
 * @return CDN_EOK on success
 * @return complete_code XHCI transfer complete status code
 */
uint32_t USBSSP_EndpointSetFeature(USBSSP_DriverResourcesT *res, uint8_t epIndex, uint8_t feature) {

    /* check parameter correctness */
    uint32_t ret = USBSSP_EndpointSetFeatureSF(res, epIndex);

    if (ret != (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! wrong value in one of function parameters\n", 0);
    } else {
        /* check that we can make control transfer on EP0 */
        ret = checkEpXferEnabled(res, 1);
    }

    if (ret == CDN_EOK) {
        if (feature > 0U) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Set Feature on ep:%d\n", res->instanceNo, epIndex);
        } else {
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Clear Feature on ep:%d\n", res->instanceNo, epIndex);
        }

        /* call handle feature */
        xhciEpSetFeatureHost(res, epIndex, feature);

    }

    return ret;
}

/**
 * function check completion code in USBSSP_SetConfiguration
 * @param res driver resources
 * @return completion code
 */
static uint32_t setConfigurationComplCode(USBSSP_DriverResourcesT const *res) {

    uint32_t ret = CDN_EOK;
    uint32_t xhciResult;

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Configure endpoints completed.\n", res->instanceNo);

    /* check result */
    xhciResult = getCompletionCode(res->commandQ.completePtr);
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Completion Code: %d\n", res->instanceNo, xhciResult);
    if (xhciResult != USBSSP_TRB_COMPLETE_SUCCESS) {
        ret = xhciResult;
    }

    return ret;
}

/**
 * Enqueue Setup TRB for SET_CONFIGURATION setup request
 * @param res driver resources
 * @param configValue configuration value
 */
static void enqueueSetCfgSetupTRB(USBSSP_DriverResourcesT *res, uint32_t configValue) {

    /* clear trb */
    (void) memset((void*) res->ep0.enqueuePtr, 0, sizeof (USBSSP_RingElementT));

    /* setup TRB */
    res->ep0.enqueuePtr->dword0 = cpuToLe32(
        ((uint32_t) configValue << USBSSP_TRB_WVALUE_POS)
        | ((uint32_t) CH9_USB_DS_CONFIGURATION << USBSSP_TRB_BREQUEST_POS)
        | (uint8_t) (CH9_USB_DIR_HOST_TO_DEVICE | CH9_USB_REQ_TYPE_STANDARD | CH9_USB_REQ_RECIPIENT_DEVICE));

    res->ep0.enqueuePtr->dword2 = cpuToLe32((uint32_t) (8U)); /* TRB length */

    res->ep0.enqueuePtr->dword3 =
        cpuToLe32(
            (uint32_t) ((USBSSP_TRB_SETUP_STAGE << USBSSP_TRB_TYPE_POS) | USBSSP_TRB_NORMAL_IDT_MASK | (uint32_t) res->ep0.toogleBit));

}

/**
 * Enqueue status TRB for SET_CONFIGURATION setup request
 * @param res driver resources
 */
static void enqueueSetCfgStatusTRB(USBSSP_DriverResourcesT *res) {

    uint32_t trbControl = (((uint32_t) USBSSP_TRB_STATUS_STAGE) << USBSSP_TRB_TYPE_POS)
                          | USBSSP_TRB_NORMAL_IOC_MASK
                          | ((uint32_t) res->ep0.toogleBit)
                          | ((uint32_t) 1 << USBSSP_TRANSFER_DIR_POS);
    /* status TRB */
    res->ep0.enqueuePtr->dword0 = 0;
    res->ep0.enqueuePtr->dword1 = 0;
    res->ep0.enqueuePtr->dword2 = 0;
    res->ep0.enqueuePtr->dword3 = cpuToLe32(trbControl);
}

/**
 * Function issues SET_CONFIGURATION setup request to connected device
 * @param res driver resources
 * @param configValue configuration value
 * @return CDN_EOK on success, error code elsewhere
 */
static uint32_t setConfigurationSetupReq(USBSSP_DriverResourcesT *res, uint32_t configValue) {

    uint32_t ret = CDN_EOK;

    /* Enqueue Setup TRB for SET_CONFIGURATION setup request */
    enqueueSetCfgSetupTRB(res, configValue);
    updateQueuePtr(&res->ep0, 0U);

    /* Enqueue status phase of control transfer */
    enqueueSetCfgStatusTRB(res);
    updateQueuePtr(&res->ep0, 0U);

    res->ep0.isRunningFlag = 1;
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> DRBL: Ring doorbell on EP0\n", res->instanceNo);
    USBSSP_WriteDoorbell(res, res->actualdeviceSlot, res->ep0.contextIndex);

    /* wait for response */
    ret = waitForVal(&res->ep0.isRunningFlag, 0U, USBSSP_DEFAULT_TIMEOUT);

    return ret;
}

/**
 * Set configuration. Function configures SSP controller as well as device connected
 * to this SSP controller. Function must not be called from interrupt context.
 *
 * @param[in] res driver resources
 * @param[in] configValue USB device's configuration selector
 *
 * @return CDN_EOK on success
 * @return complete_code XHCI transfer complete status code
 */
uint32_t USBSSP_SetConfiguration(USBSSP_DriverResourcesT *res, uint32_t configValue) {

    uint32_t ret = USBSSP_SetConfigurationSF(res);
    uint32_t xhciResult;

    if (ret != (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! wrong value in one of function parameters\n", 0);
    }

    if (ret == CDN_EOK) {
        /* check if Ep0 supports data transfer */
        ret = checkEpXferEnabled(res, 1);
    }
    if (ret == (uint32_t) CDN_EOK) {
        /* sends SET_CONFIGURATION to device */
        ret = setConfigurationSetupReq(res, configValue);
    }
    if (ret == (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> SET CONFIGURATION setup completed.\n", res->instanceNo);

        /* check result */
        xhciResult = getCompletionCode(res->ep0.completePtr);
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Completion Code: %d\n", res->instanceNo, xhciResult);

        if (xhciResult != USBSSP_TRB_COMPLETE_SUCCESS) {
            ret = xhciResult;
        }
    }

    if (ret == (uint32_t) CDN_EOK) {
        configureEndpoints(res, res->ep0Buff);

        /* wait for response */
        ret = waitForVal(&res->commandQ.isRunningFlag, 0U, USBSSP_DEFAULT_TIMEOUT);
    }

    if (ret == (uint32_t) CDN_EOK) {
        ret = setConfigurationComplCode(res);
    }

    return ret;
}

/**
 * Calculate full/low speed endpoint interval based on bInterval
 * See xHCI spec Section 6.2.3.6 for more details.
 * @param[in] bInterval
 * @return valid endpoint context interval value
 */
uint8_t USBSSP_CalcFsLsEPIntrptInterval(uint8_t bInterval) {

    uint8_t interval = 2U;
    uint8_t bitOffset; /* register with '1' circulating */
    uint8_t res1; /* used for finding the most significant 1 from left to right */
    uint8_t res2 = 0U; /* used for finding the first 1 looking from right to left */

    if (bInterval > 0U) {
        /* find the oldest bit */
        bitOffset = 0x80U;
        do {
            res1 = bInterval & bitOffset;
            bitOffset >>= 1U;
        } while ((res1 == 0U) && (bitOffset > 0U));

        /* calculate context interrupt value */
        bitOffset = 0x01;
        do {
            res2 = res1 & bitOffset;
            ++interval;
            bitOffset <<= 1U;
        } while (res2 == 0U);
    } else {
        interval = 0U;
    }

    return interval;
}

/**
 * Function sets MULT field in endpoint context structure
 * @param epObj
 */
static void setMult(USBSSP_ProducerQueueT * epObj) {

    uint8_t mult = 0U;
    USBSSP_DriverResourcesT *res = epObj->parent;
    CH9_UsbSpeed actualSpeed = res->actualSpeed;
    uint8_t epDescType = epObj->epDesc[3] & 0x03U;

    /* this field is different from only if Large ESIT Payload is not supported */
    if ((actualSpeed >= CH9_USB_SPEED_SUPER) && (epDescType == CH9_USB_EP_ISOCHRONOUS)) {
        /* check if isochronous endpoint companion descriptor does not exists */
        if ((epObj->epDesc[10] & 0x80U) != 0x80U) {
            mult = epObj->epDesc[10] & 0x03U;
        }
    }
    if (mult > 0U) {
        epObj->hwContext[0] |= (uint32_t) mult << USBSSP_EP_CONTEXT_MULT_POS;
    }
}

/**
 * Function initializes stream object
 * @param epObj endpoint object
 * @param stream stream object
 * @param iter iterator equal to (stream ID - 1)
 */
static void initStreamObj(USBSSP_ProducerQueueT const * epObj, USBSSP_ProducerQueueT * stream, uint32_t iter) {

    USBSSP_DriverResourcesT *res = epObj->parent;

    /* initialize ring of stream */
    stream->ring = (USBSSP_RingElementT*) (&(*res->xhciMemRes->streamRing)[epObj->contextIndex - 2U][iter][0]);
    stream->enqueuePtr = stream->ring;
    stream->dequeuePtr = stream->ring;
    stream->contextIndex = epObj->contextIndex; /* All child stream objects have parent's (endpoint) contextIndex */
    stream->toogleBit = 1U;
    stream->actualSID = (uint16_t) iter + 1U;
    stream->interrupterIdx = 0U; /* Set interrupter to default */
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> Memory allocated for epIndex: %02X, streamID (%d) ring: %p\n", res->instanceNo, stream->contextIndex, stream->actualSID, (void*) stream->ring);

    /* update stream context */
    set64Value(
        &epObj->ring[stream->actualSID].dword0,
        &epObj->ring[stream->actualSID].dword1,
        cpuToLe64(get64PhyAddrOf32ptr(&stream->enqueuePtr->dword0) | 0x02UL | stream->toogleBit)     /* SCT = 1, PRIMARY string, transfer ring, spec 6.2.4.1 */
        );

}

/**
 * Function sets maxPStreams
 * @param epObj endpoint object
 */
static void setMaxPStreams(USBSSP_ProducerQueueT * epObj) {

    uint8_t maxPStreams = 0U;
    USBSSP_DriverResourcesT *res = epObj->parent;
    uint8_t epDescType = epObj->epDesc[3] & 0x03U;

    if (epDescType == CH9_USB_EP_BULK) {
        uint8_t hwMaxPStreams = (uint8_t) CPS_FLD_READ(USBSSP__HCCPARAMS, MAXPSASIZE, res->qaRegs.xHCCaps.hccparams1);
        uint32_t streamId;

        /* maxPStreams should be set to minimal of three factors: */
        /* MAX_STREMS_PER_EP, hccparams1, companion descriptor */
        /* first check if driver allows to use full hardware stream number and limit if NO */
        hwMaxPStreams = (hwMaxPStreams > USBSSP_MAX_STREMS_PER_EP) ? USBSSP_MAX_STREMS_PER_EP : hwMaxPStreams;

        /* Then check if descriptor companion streams number exceeds hardware number and limit if Yes */
        maxPStreams = epObj->epDesc[10] & 0x1FU;
        maxPStreams = (maxPStreams > hwMaxPStreams) ? hwMaxPStreams : maxPStreams;
        vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> MAX_PSA_SIZE: %d\n", res->instanceNo, hwMaxPStreams);
        vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> maxPStreams: %d\n", res->instanceNo, maxPStreams);

        /* ------------------- initialize streams ------------------ */
        if (maxPStreams > 0U) {
            uint8_t epContext = epObj->contextIndex - 2U;
            if (((int16_t)epContext < (int16_t)USBSSP_MAX_EP_NUM_STRM_EN) && ((int16_t)epContext >= 0)) {
                for (streamId = 0; streamId < (USBSSP_STREAM_ARRAY_SIZE - 1U); streamId++) {

                    USBSSP_ProducerQueueT *stream;
                    USBSSP_ProducerQueueT(*streamObj)[USBSSP_MAX_EP_NUM_STRM_EN][USBSSP_STREAM_ARRAY_SIZE] = res->xhciMemRes->streamMemoryPool;
                    /* get reference to single stream object within stream container */
                    epObj->stream[streamId] = &((*streamObj)[epObj->contextIndex - 2U][streamId]);
                    stream = (USBSSP_ProducerQueueT *) epObj->stream[streamId];

                    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> Memory allocated for stream (%d) object: %p\n", res->instanceNo, streamId + 1, (void*) stream);
                    initStreamObj(epObj, stream, streamId);
                }
            } else {
                vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> maxPStreams(%d) for unsupported endpoint (%d)\n", epObj->parent->instanceNo, maxPStreams, (epObj->contextIndex));
            }
        }

        if (maxPStreams > 0U) {
            epObj->hwContext[0] |= (uint32_t) maxPStreams << USBSSP_EP_CXT_PMAXSTREAMS_POS;
        }
    } /* if BULK */
}

/**
 * Function sets interval field in endpoint context
 * @param epObj endpoint object
 */
static void setInterval(USBSSP_ProducerQueueT * epObj) {

    uint8_t interval = epObj->epDesc[6];
    uint8_t epDescType = epObj->epDesc[3] & 0x03U;

    /* Convert interval to endpoint valid value (Table 65, section 6.2.3.6 of the xHCI Spec). */
    switch (epObj->parent->actualSpeed) {

    case CH9_USB_SPEED_LOW:
        if (epDescType == CH9_USB_EP_INTERRUPT) {
            /* Table 65 - LS Interrupt - covert interval (1-255) to (3-10). */
            interval = USBSSP_CalcFsLsEPIntrptInterval(interval);
        }
        /* If none of the above leave interval unchanged as it should be zero already. */
        break;

    case CH9_USB_SPEED_FULL:
        /* Table 65 - FS Isoch. - convert interval (1-16) to (3-18) i.e. increment by 2. */
        if (epDescType == CH9_USB_EP_ISOCHRONOUS) {
            interval = interval + 2U;
        } else {
            if (epDescType == CH9_USB_EP_INTERRUPT) {
                /* Table 65 - FS Interrupt - covert interval (1-255) to (3-10). */
                interval = USBSSP_CalcFsLsEPIntrptInterval(interval);
            }
        }
        /* If none of the above leave interval unchanged. */
        break;

    default:
        /* Table 65 - SS or HS (Interrupt/Isoch) - convert interval (1-16) to (0-15) i.e. decrement by 1. */
        /* Bulk value will be left unchanged */
        if (epDescType != CH9_USB_EP_BULK) {
            if (interval > 0U) {
                interval = interval - 1U;
            }
        }
        break;
    }
    if (interval > 0U) {
        epObj->hwContext[0] |= (uint32_t) interval << USBSSP_EP_CONTEXT_INTERVAL_POS;
    }
}

/**
 * Used for handling USB SPEED LOW
 * @param epMaxPacketSize
 * @return uint32_t number of bytes per interval
 */
static uint32_t handleSpeedLow(uint16_t epMaxPacketSize) {

    uint32_t bytesPerInterval = epMaxPacketSize;
    if (epMaxPacketSize > 8U) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "WARNING: epMaxPacketSize (%d) > 8\n", epMaxPacketSize);
    }
    return bytesPerInterval;
}

/**
 * Used for handling USB SPEED FULL
 * @param epDescType
 * @param epMaxPacketSize
 * @return uint32_t number of bytes per interval
 */
static uint32_t handleSpeedFull(uint8_t epDescType, uint16_t epMaxPacketSize) {
    uint32_t bytesPerInterval = 0U;
    if ((epDescType == CH9_USB_EP_ISOCHRONOUS) || (epDescType == CH9_USB_EP_INTERRUPT)) {
        bytesPerInterval = epMaxPacketSize;
    }
    return bytesPerInterval;
}

/**
 * Used for handling USB SPEED HIGH
 * @param epDescType
 * @param epMaxPacketSize
 * @param burst_value
 * @return uint32_t number of bytes per interval
 */
static uint32_t handleSpeedHigh(uint8_t epDescType, uint16_t epMaxPacketSize, uint8_t burst_value) {
    uint32_t bytesPerInterval = 0U;
    if ((epDescType == CH9_USB_EP_ISOCHRONOUS) || (epDescType == CH9_USB_EP_INTERRUPT)) {
        bytesPerInterval = (uint32_t) epMaxPacketSize * ((uint32_t) burst_value + 1U);
    }
    return bytesPerInterval;
}

/**
 * Used for handling USB SPEED SUPER
 * @param epDescType
 * @param desc
 * @return uint32_t number of bytes per interval
 */
static uint32_t handleSpeedSuper(uint8_t epDescType, uint8_t const *desc) {
    uint32_t bytesPerInterval = 0U;
    if ((epDescType == CH9_USB_EP_ISOCHRONOUS) || (epDescType == CH9_USB_EP_INTERRUPT)) {

        /* bytes 5 (11 when added endpoint descriptor ) and 6 */
        /*(12 when added endpoint descriptor) of companion descriptor is */
        /* wBytesPerInterval value */
        bytesPerInterval = ((uint32_t) desc[12] << 8) | ((uint32_t) desc[11]);
    }
    return bytesPerInterval;
}

/**
 * Function sets max ESIT payload field in endpoint context
 * @param epObj endpoint object
 */
static void setMaxESITPayload(USBSSP_ProducerQueueT * epObj) {

    uint32_t bytesPerInterval = 0U;
    uint8_t epDescType = epObj->epDesc[3] & 0x03U;
    uint8_t *desc = epObj->epDesc;
    uint16_t epMaxPacketSize = (((uint16_t) desc[5] & 0x7U) << 8) | (uint16_t) desc[4];
    uint8_t burst_value = (desc[5] & 0x18U) >> 3U;

    /* calculate bytesPerInterval depending on different operating speed */
    switch (epObj->parent->actualSpeed) {

    /* for low speed */
    case CH9_USB_SPEED_LOW:
        bytesPerInterval = handleSpeedLow(epMaxPacketSize);
        break;

    /* full speed */
    case CH9_USB_SPEED_FULL:
        bytesPerInterval = handleSpeedFull(epDescType, epMaxPacketSize);
        break;

    /* high speed */
    case CH9_USB_SPEED_HIGH:
        bytesPerInterval = handleSpeedHigh(epDescType, epMaxPacketSize, burst_value);
        break;

    /* super speed */
    case CH9_USB_SPEED_SUPER:
        bytesPerInterval = handleSpeedSuper(epDescType, desc);
        break;

    default:
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Incorrect speed value: %d!\n", epObj->parent->instanceNo, epObj->parent->actualSpeed);
        break;
    }

    if (bytesPerInterval > (48U * 1024U)) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> WARNING:  bytesPerInterval(%d) > 48KB, Patching to 48KB\n", epObj->parent->instanceNo, bytesPerInterval);
        bytesPerInterval = 0xC000U;
    }

    /* sets bytesPerInterval in endpoint context */
    if (bytesPerInterval > 0U) {
        epObj->hwContext[0] |= ((bytesPerInterval >> 16U) << USBSSP_EP_CXT_MAXESITPLD_HI_POS);
        epObj->hwContext[4] |= ((bytesPerInterval & 0xFFFFU) << USBSSP_EP_CXT_MAXESITPLD_LO_POS);
    }
}

/**
 * Function sets CErr field in endpoint context structure
 * @param epObj endpoint object
 */
static void setCErr(USBSSP_ProducerQueueT * epObj) {

    uint8_t epDescType = epObj->epDesc[3] & 0x03U;
    uint32_t cerr = 0U;

    /* cerr field should be set to three only for bulk and interrupt endpoint */
    if (epDescType != CH9_USB_EP_ISOCHRONOUS) {

        cerr = USBSSP_EP_CONTEXT_3ERR;
    }
    if (cerr > 0U) {
        cerr <<= USBSSP_EP_CONTEXT_CERR_POS;
        epObj->hwContext[1] |= cerr;
    }
}

/**
 * sets endpoint type field in endpoint context
 * @param epObj
 */
static void setEPType(USBSSP_ProducerQueueT * epObj) {

    /* this value reflects endpoint context bytes order */
    uint32_t epType = 0U;

    /* in endpoint descriptor, type is done on third byte, mask it with two less */
    /* significant bits */
    uint8_t epDescType = epObj->epDesc[3] & 0x03U;

    if (epDescType == 0U) {
        epDescType = 4U; /* Control - Bidirectional */
    } else if ((epObj->epDesc[2] & CH9_USB_EP_DIR_IN) != 0U) {
        /* check if endpoint in or out */
        /* address is kept on second byte of endpoint descriptor */
        /* according to XHCI endpoint type coding convention, for IN extra 1 or fourth */
        /* position must be added for this direction */
        epDescType |= USBSSP_EP_CXT_EP_DIR_IN;
    } else {
        /* MISRA: do nothing for out descriptors */
    }

    epType = ((uint32_t) epDescType << USBSSP_EP_CONTEXT_EP_TYPE_POS) & USBSSP_EP_CONTEXT_EP_TYPE_MASK;

    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> epType: %d\n", epObj->parent->instanceNo, epType >> USBSSP_EP_CONTEXT_EP_TYPE_POS);

    epObj->hwContext[1] |= epType;
}

/**
 * Function sets max burst value for SS and SSP mode
 * @param desc endpoint descriptor
 * @param maxBurstSize max burst value
 */
static void setMaxBurstSizeSS(uint8_t const * desc, uint8_t * maxBurstSize) {

    /* in endpoint descriptor, type is done on third byte, mask it with two less */
    /* significant bits */
    uint8_t epDescType = desc[3] & 0x03U;

    /* check if super speed endpoint companion available */
    if ((desc[7] == CH9_USB_DS_SS_USB_EP_COMPANION)
        && (desc[8] == CH9_USB_DT_SS_USB_EP_COMPANION)) {

        vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "SuperSpeed Endpoint companion found-\n", 0);
        *maxBurstSize = desc[9];

        if (epDescType == CH9_USB_EP_INTERRUPT) {
            if (*maxBurstSize > 2U) {
                vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "WARNING: burst value for interrupt endpoint > 2\n", 0);
                vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "WARNING: Limit burst value to 2\n", 0);
                *maxBurstSize = 2;
            }
        }
    }
}

/**
 * Function sets max burst value for HS mode
 * @param desc endpoint descriptor
 * @param maxBurstSize max burst value
 */
static void setMaxBurstSizeHS(uint8_t const * desc, uint8_t * maxBurstSize) {
    uint8_t epDescType = desc[3] & 0x03U;

    /* calculate burst value from wMaxPacketSize field for HS speed & periodic endpoints */
    if ((epDescType == CH9_USB_EP_INTERRUPT) || (epDescType == CH9_USB_EP_ISOCHRONOUS)) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "Periodic endpoint found, setting burst value-\n", 0);
        *maxBurstSize = (desc[5] & 0x18U) >> 3U;

        if (*maxBurstSize > 2U) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "WARNING: burst value for interrupt endpoint > 2\n", 0);
            vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "WARNING: Limit burst value to 2\n", 0);
            *maxBurstSize = 2U;
        }
    }
}

/**
 * Function sets max burst size field in endpoint context
 * @param epObj endpoint object
 */
static void setMaxBurstSize(USBSSP_ProducerQueueT * epObj) {

    uint8_t maxBurstSize = 0U;
    uint8_t *desc = epObj->epDesc;

    if (epObj->parent->actualSpeed >= CH9_USB_SPEED_SUPER) {
        setMaxBurstSizeSS(desc, &maxBurstSize);
    } else if (epObj->parent->actualSpeed == CH9_USB_SPEED_HIGH) {
        setMaxBurstSizeHS(desc, &maxBurstSize);
    } else {
        /* required for MISRA */
    }
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> bMaxBurst: %d\n", epObj->parent->instanceNo, maxBurstSize);
    if (maxBurstSize > 0U) {
        epObj->hwContext[1] |= ((uint32_t) maxBurstSize << USBSSP_EP_CXT_MAX_BURST_SZ_POS) & USBSSP_EP_CXT_MAX_BURST_SZ_MASK;
    }
}

/**
 * Function sets max packet size field in endpoint context
 * @param epObj endpoint object
 */
static void setMaxPacketSize(USBSSP_ProducerQueueT * epObj) {

    uint8_t *desc = epObj->epDesc;

    /* calculate max packet size from bytes: 4 and 5 written in Little endian */
    uint16_t epMaxPacketSize = (((uint16_t) desc[5] & 0x7U) << 8) | (uint16_t) desc[4];
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> epMaxPacketSize: %d\n", epObj->parent->instanceNo, epMaxPacketSize);

    epObj->hwContext[1] |= (uint32_t) epMaxPacketSize << USBSSP_EP_CXT_MAX_PKT_SZ_POS;
}

/**
 * Function return dequeue cycle state bit from endpoint context
 * @param epObj endpoint object
 * @return dequeue cycle state bit
 */
static uint32_t getDCS(USBSSP_ProducerQueueT const * epObj) {

    uint32_t dcs;
    dcs = epObj->toogleBit;

    return dcs;
}

/**
 * Function sets TR dequeue pointer field in endpoint context
 * @param epObj endpoint object
 */
static void setTRDequeuPointer(USBSSP_ProducerQueueT const * epObj) {

    set64Value(
        &epObj->hwContext[2],
        &epObj->hwContext[3],
        cpuToLe64(get64PhyAddrOf32ptr(&epObj->enqueuePtr->dword0) | getDCS(epObj)));
}

/**
 * Function sets average TRB length in endpoint context
 * @param epObj endpoint object
 */
static void setAverageTRBLength(USBSSP_ProducerQueueT * epObj) {

    uint8_t epDescType = epObj->epDesc[3] & 0x03U;
    uint32_t averageTRBLength = 0U;

    /* check endpoint transfer type */
    switch (epDescType) {
    case CH9_USB_EP_ISOCHRONOUS:
        averageTRBLength = USBSSP_EP_CXT_EP_ISO_AVGTRB_LEN;
        break;
    case CH9_USB_EP_INTERRUPT:
        averageTRBLength = USBSSP_EP_CXT_EP_INT_AVGTRB_LEN;
        break;
    case CH9_USB_EP_BULK:
        averageTRBLength = USBSSP_EP_CXT_EP_BLK_AVGTRB_LEN;
        break;
    default:     /* Assume control EP */
        averageTRBLength = USBSSP_EP_CXT_EP_CTL_AVGTRB_LEN;
        break;
    }
    averageTRBLength <<= USBSSP_EP_CXT_EP_AVGTRBLEN_POS;
    epObj->hwContext[4] |= averageTRBLength;

}

/**
 * update all parameters related to data-size
 * @param epObj endpoint object
 */
static void updateEpObjDataSz(USBSSP_ProducerQueueT * epObj) {
    /* set max ESIT payload for this endpoint */
    setMult(epObj);
    setMaxESITPayload(epObj);
    setMaxBurstSize(epObj);
    setMaxPacketSize(epObj);
}
/**
 * This function updates the input context of a non-default ep-object
 * @param epObj endpoint object
 */
static void updateEpObj(USBSSP_ProducerQueueT * epObj) {
    /* Set Ep type*/
    setEPType(epObj);

    /* set max streams if streams are enabled*/
    setMaxPStreams(epObj);

    setInterval(epObj);

    /* update all parameters related to data-size */
    updateEpObjDataSz(epObj);

    setCErr(epObj);
    setTRDequeuPointer(epObj);
    setAverageTRBLength(epObj);
}

/**
 * Function stores descriptors in endpoint object
 * @param res driver resources
 * @param epObj endpoint object
 * @param desc endpoint descriptor
 * @return CDN_EOK if for correct descriptor, error code elsewhere
 */
static uint32_t storeEpDesc(USBSSP_DriverResourcesT const *res, USBSSP_ProducerQueueT * epObj, uint8_t const *desc) {

    uint32_t ret = CDN_EOK;

    /* store descriptor endpoint in endpoint object, will be used by upper layers */
    (void) memcpy(epObj->epDesc, desc, CH9_USB_DS_ENDPOINT);
    if (res->actualSpeed >= CH9_USB_SPEED_SUPER) {

        /* first check if companion descriptor exists aligned in memory to */
        /* endpoint descriptor and return error if doesn't */
        if (desc[CH9_USB_DS_ENDPOINT] != CH9_USB_DS_SS_USB_EP_COMPANION) {
            ret = CDN_EPERM;
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Endpoint companion descriptor does not exist for SSx device\n", res->instanceNo);
        } else {
            (void) memcpy(&epObj->epDesc[CH9_USB_DS_ENDPOINT], &desc[CH9_USB_DS_ENDPOINT], CH9_USB_DS_SS_USB_EP_COMPANION);
        }
    }
    return ret;
}

/**
 * Update endpoint input context
 * @param res driver resources
 * @param epObj endpoint object
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t updateEpInputContext(USBSSP_DriverResourcesT *res, const USBSSP_ProducerQueueT * epObj) {

    uint32_t ret = CDN_EOK;

    /* checking < 32 is required by Misra */
    if (epObj->contextIndex < 32U) {
        uint32_t leAddMask = cpuToLe32((uint32_t) (1UL << epObj->contextIndex));
        res->inputContext->inputControlContext[1] |= leAddMask;

        /* If endpoint was previously registered to be dropped (Dx==1)_clearing Dx flag */
        if ((res->inputContext->inputControlContext[0] & leAddMask) != 0U) {
            res->inputContext->inputControlContext[0] &= ~leAddMask;
        }

        /* update slot context when required */
        if (epObj->contextIndex > res->contextEntries) {
            res->contextEntries = epObj->contextIndex;
        }

        res->inputContext->slot[0] = cpuToLe32(
            ((uint32_t) res->contextEntries << USBSSP_SLOT_CXT_CXT_ENT_POS)
            | ((uint32_t) getSlotSpeed(res->actualSpeed) << USBSSP_SLOT_CONTEXT_SPEED_POS)
            );     /*6.2.2 set slot context entries and speed */
        if (res->inputContextCallback != NULL) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Calling inputContextCallback()\n", res->instanceNo);
            res->inputContextCallback(res);
        }
    } else {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Illegal value of context number %d\n",
                res->instanceNo, epObj->contextIndex);
        ret = CDN_EINVAL;
    }
    return ret;
}

/**
 * Configure and enable single endpoint
 * @param[in] res driver resources
 * @param[in] desc endpoint descriptor
 * @return CDN_EOK for success, error code elsewhere
 */
uint32_t USBSSP_EnableEndpoint(USBSSP_DriverResourcesT *res,
                               uint8_t const *          desc) {

    uint32_t ret = CDN_EOK; /* returned value */

    uint8_t epIn;
    uint8_t epIndex;
    USBSSP_ProducerQueueT * epObj;

    ret = USBSSP_EnableEndpointSF(res, desc);
    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
    } else {
        uint8_t epAddress = desc[2]; /* get endpoint address from descriptor */
        uint8_t epDescType = desc[3] & 0x03U; /* get endpoint attributes */

        /* calculate endpoint direction */
        if (epDescType == CH9_USB_EP_CONTROL) {
            epIn = 1U; /* For control endpoint flag should be set */
        } else {
            epIn = ((epAddress & CH9_USB_EP_DIR_IN) > 0U) ? (uint8_t) 1U : (uint8_t) 0U;
        }

        /* calculate endpoint index */
        epIndex = (uint8_t) ((((epAddress & 0x7FU) - 1U) * 2U) + ((epIn > 0U) ? 1U : 0U));
        if (epIndex >= USBSSP_MAX_EP_CONTEXT_NUM) {
            ret = CDN_EINVAL;
        }
    }

    if (ret == CDN_EOK) {
        /* get endpoint object from endpoint container */
        epObj = &res->ep[epIndex + USBSSP_EP_CONT_OFFSET];

        /* store context index */
        epObj->contextIndex = epIndex + USBSSP_EP_CONT_OFFSET;

        /* set endpoint object's hardware context */
        epObj->hwContext = res->inputContext->epContext[epIndex];

        /* set endpoint parent */
        epObj->parent = res;

        /* set interrupter index for this endpoint */
        epObj->interrupterIdx = res->epInterrupterIdx[epIndex];

        ret = storeEpDesc(res, epObj, desc);

        if (ret == CDN_EOK) {
            updateEpObj(epObj);

            /* update input control context */
            ret = updateEpInputContext(res, epObj);
        }
    }
    return ret;
}

/**
 * Disables single endpoint (before issuing CONFIGURE_ENDPOINT command)
 * @param[in] res driver resources
 * @param[in] epAddress Endpoint address
 * @return CDN_EOK for success, error code elsewhere
 */
uint32_t USBSSP_DisableEndpoint(USBSSP_DriverResourcesT *res, uint8_t epAddress) {

    /* check if res is not NULL */
    uint32_t ret = USBSSP_DisableEndpointSF(res);

    if (ret != (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! wrong value in one of function parameters\n", 0);
    }

    if (ret == CDN_EOK) {
        uint8_t epIn;
        uint8_t epBase;
        uint8_t contextEntry;
        uint32_t leDropMask = 0U;

        /* calculate context index from endpoint address */
        epIn = (uint8_t) (((epAddress & CH9_USB_EP_DIR_IN) > 0U) ? 1U : 0U);
        epBase = (uint8_t) ((((epAddress & 0x7FU) - 1U) * 2U) + ((epIn > 0U) ? 1U : 0U));
        contextEntry = epBase + USBSSP_EP_CONT_OFFSET;

        vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> ep_address: %02x (EP%d_%s)\n", res->instanceNo, epAddress, (epAddress & 0xF), (epIn ? "IN" : "OUT"));

        if (contextEntry < (sizeof (uint32_t) - 1U)) {
            leDropMask = cpuToLe32((uint32_t) (1UL << contextEntry));
        }

        /* Setting Dx flag ... */
        res->inputContext->inputControlContext[0] |= leDropMask;

        /* ... and clearing Ax flag, if set */
        if ((res->inputContext->inputControlContext[1] & leDropMask) != (uint32_t) 0U) {
            res->inputContext->inputControlContext[1] &= ~leDropMask;
        }
        if (res->inputContextCallback != NULL) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Calling inputContextCallback()\n", res->instanceNo);
            res->inputContextCallback(res);
        }
        (void) memcpy(&res->inputContextCopy, res->inputContext, sizeof (USBSSP_InputContexT));
    }

    return ret;
}

/**
 * Issue generic command to SSP controller
 * @param[in] res Driver resources
 * @param[in] dword0 word 0 of command
 * @param[in] dword1 word 1 of command
 * @param[in] dword2 word 2 of command
 * @param[in] dword3 word 3 of command
 * @return CDN_EOK for success, error code elsewhere
 */
uint32_t USBSSP_IssueGenericCommand(USBSSP_DriverResourcesT *res, uint32_t dword0, uint32_t dword1, uint32_t dword2, uint32_t dword3) {

    /* check if res parameter is not NULL */
    uint32_t ret = USBSSP_IssueGenericCommandSF(res);
    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);

    } else {

        /* fill four DWORDs of TRB */
        res->commandQ.enqueuePtr->dword0 = cpuToLe32(dword0);
        res->commandQ.enqueuePtr->dword1 = cpuToLe32(dword1);
        res->commandQ.enqueuePtr->dword2 = cpuToLe32(dword2);
        res->commandQ.enqueuePtr->dword3 = cpuToLe32(dword3);
        /* set cycle bit to correct value */
        res->commandQ.enqueuePtr->dword3 &= 0xFFFFFFFEU;
        res->commandQ.enqueuePtr->dword3 |= (uint32_t) res->commandQ.toogleBit;

        updateQueuePtr(&res->commandQ, 0U);
        res->commandQ.isRunningFlag = 1;
        hostCmdDoorbell(res);
    }
    return ret;
}

/**
 * Function builds configure endpoint command TRB
 * @param res driver resources
 */
static void issueConfigEpCmd(USBSSP_DriverResourcesT *res) {

    /* set input context pointer in TRB */
    set64Value(
        &res->commandQ.enqueuePtr->dword0,
        &res->commandQ.enqueuePtr->dword1,
        cpuToLe64(get64PhyAddrOf32ptr(res->inputContext->inputControlContext))
        );
    /* set device slot, cycle bit, TRB type */
    res->commandQ.enqueuePtr->dword3 = cpuToLe32((uint32_t) (
                                                     (uint32_t) ((uint32_t) res->actualdeviceSlot << USBSSP_SLOT_ID_POS)
                                                     | (USBSSP_TRB_CONF_EP_CMD << USBSSP_TRB_TYPE_POS)
                                                     | (uint32_t) res->commandQ.toogleBit
                                                     )
                                                 );
    updateQueuePtr(&res->commandQ, 0U);
    res->commandQ.isRunningFlag = 1;
    hostCmdDoorbell(res);
}

/**
 * Configure and enable all endpoints
 * @param[in] res driver resources
 * @param[in] conf configuration descriptor
 */
static void configureEndpoints(USBSSP_DriverResourcesT *res, uint8_t const *conf) {

    uint32_t i = 0;

    /* calculate whole configuration length, configuration length is written
       in little endian in uint16_t on 2 and 3 byte */
    uint16_t length = ((uint16_t) conf[3] << 8) | (uint16_t) conf[2];

    /* Ax flags (within Input Control Context) need to be: A0 = 1, A1 = 0, Dx should be 0 */
    res->inputContext->inputControlContext[0] = 0; /* Dx = 0 */
    res->inputContext->inputControlContext[1] = cpuToLe32(1); /* A0 = 1, all other Ax = 0 */
    if (res->inputContextCallback != NULL) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Calling inputContextCallback()\n", res->instanceNo);
        res->inputContextCallback(res);
    }
    (void) memcpy(&res->inputContextCopy, res->inputContext, sizeof (USBSSP_InputContexT));
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> configureEndpoints: %02x %d\n", res->instanceNo, conf[0], length);

    while (i < length) {
        /*descriptor type has offset 1 in descriptor*/
        if (conf[i + 1U] == CH9_USB_DT_ENDPOINT) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> ---Endpoint found---\n", res->instanceNo);
            (void) USBSSP_EnableEndpoint(res, &conf[i]);
        }
        i += conf[i];
    }

    issueConfigEpCmd(res);
}

/**
 * Add event data TRB to transfer ring
 * @param res driver resources
 * @param epIndex endpoint index
 * @param eventDataLo event data low DWORD
 * @param eventDataHi event data high DWORD
 * @param flags extra flags for TRB
 * @return CDN_EOK for success, error code elsewhere
 */
uint32_t USBSSP_AddEventDataTRB(USBSSP_DriverResourcesT *res, uint8_t epIndex, uint32_t eventDataLo, uint32_t eventDataHi, uint32_t flags) {

    /* check if res input parameter is not NULL */
    uint32_t ret = USBSSP_AddEventDataTRBSF(res);

    if (ret != (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
    }

    if (ret == CDN_EOK) {
        /* get endpoint object */
        USBSSP_ProducerQueueT *ep = (epIndex == USBSSP_EP0_CONTEXT_OFFSET) ? &res->ep0 : &res->ep[epIndex];

        /* set event data TRB */
        ep->enqueuePtr->dword0 = cpuToLe32(eventDataLo);
        ep->enqueuePtr->dword1 = cpuToLe32(eventDataHi);
        ep->enqueuePtr->dword2 = cpuToLe32(0);
        ep->enqueuePtr->dword3 = cpuToLe32((uint32_t) (
                                               (USBSSP_TRB_EVENT_DATA << USBSSP_TRB_TYPE_POS)
                                               | flags
                                               | (uint32_t) ep->toogleBit)
                                           );

        updateQueuePtr(ep, 0U);
    }

    return ret;
}

/**
 * Function returns max packet size for selected endpoint
 * @param epObj endpoint object
 * @return max packet size value
 */
static uint16_t getMaxPacketSize(const USBSSP_ProducerQueueT * epObj) {

    /* get max packet size value from endpoint context */
    uint16_t maxPacketSize = (uint16_t) (epObj->hwContext[1] >> USBSSP_EP_CXT_MAX_PKT_SZ_POS);
    return (maxPacketSize);
}

/**
 * auxiliary structure type used for variables traversing between functions when
 * TD is being created
 */
typedef struct {
    uint8_t isLastBuffer; /* used for signaling last buffer in scatter/gather transfer */
    uint8_t isFirstPage; /* used for signaling first memory page */
    uint8_t isLastPage; /* used for signaling last memory page */
    uint8_t isFirstTrb; /* used for signaling first TRB */
    uint8_t isLastTrb; /* used for signaling last TRB */
    uint32_t trbTransferLengthSum; /* used for TDSize calculation */
    uint32_t packetTransfered; /* used for TDSize calculation */
    uint32_t tdPacketCount; /* used for TDSize calculation */
    uint16_t epMaxPacketSize; /* used for TDSize calculation */
    uint8_t epIndex; /* keeps endpoint context index */
    /* --- used in USBSSP_CreateTD --------------------- */
    uintptr_t pageStart; /* used for page number calculation */
    uintptr_t pageEnd; /* used for page number calculation */
    uintptr_t numOfPages; /* keeps number of memory page used for TD */
    uintptr_t dataPtr;
} USBSSP_TDCreateT;

/**
 * Calculate TD Size value of TRB field
 * @param singleTrbLength data length
 * @param tdParams pointer to extra parameters (used internally)
 * @return value of TD size (0-31)
 */
static uint32_t calculateTdSize(uint32_t singleTrbLength, USBSSP_TDCreateT * tdParams) {

    uint32_t tdSize;

    /* calculate tdSize */
    tdParams->trbTransferLengthSum += singleTrbLength;

    if (tdParams->epMaxPacketSize > 0U) {
        /* round down packetTransfered */
        tdParams->packetTransfered = tdParams->trbTransferLengthSum / tdParams->epMaxPacketSize;
    }

    /* set tdSize to zero for the last TRB in TD */
    if ((tdParams->isLastBuffer == 1U) && (tdParams->isLastTrb == 1U) && (tdParams->isLastPage == 1U)) {
        tdSize = 0U;
    } else {
        tdSize = ((tdParams->tdPacketCount - tdParams->packetTransfered) > 31U) ? 31U : (tdParams->tdPacketCount - tdParams->packetTransfered);
    }
    return tdSize;
}

/**
 * Function gets BurstCount for an endpoint
 * BurstCount is similar to MULT, i.e.
 * burstCount = (Max # of packets) / (actual burst size)
 * @param ep endpoint object
 * @param tdParams pointer to extra parameters (used internally)
 * @return burstcount
 */
static uint32_t getBurstCount(const USBSSP_ProducerQueueT *ep, const USBSSP_TDCreateT * tdParams) {

    /* get burst size from EP context */
    uint32_t burstSize = (ep->hwContext[1] & USBSSP_EP_CXT_MAX_BURST_SZ_MASK) >> USBSSP_EP_CXT_MAX_BURST_SZ_POS;
    uint32_t burstCount = (tdParams->tdPacketCount + burstSize + 1U) / (burstSize + 1U);

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "getBurstCount: %d\n", (burstCount - 1U));
    return (burstCount - 1U);
}

/**
 * Function updates TRB for isochronous endpoint
 * Note that in a TD, only first TRB will be marked ISO
 * @param res driver resources
 * @param ep endpoint object
 * @param tdParams pointer to extra parameters (used internally)
 */
static void updateForIsoTrb(const USBSSP_DriverResourcesT *res, USBSSP_ProducerQueueT *ep, const USBSSP_TDCreateT * tdParams) {

    uint32_t ete = res->qaRegs.xHCCaps.hcsparams2 & 0x100U; /* Missing USBSSP__HCSPARAMS2__ETE_MASK */
    uint32_t burstCount = getBurstCount(ep, tdParams);

    /* get TRB's last DWORD */
    uint32_t tempDword3 = le32ToCpu(ep->enqueuePtr->dword3);

    /* set TRB type as isochronous */
    uint32_t trbType = USBSSP_TRB_ISOCH;

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "hcsparams2(0x%x): ete(0x%x)\n", res->qaRegs.xHCCaps.hcsparams2, ete);

    /* check if frameID is valid */
    if ((ep->frameID & 0x8000U) > 0U) {
        /* reset frameID */
        tempDword3 &= ~((uint32_t) 0x7FFU << USBSSP_TRB_ISOCH_FRAME_ID_POS);
        tempDword3 &= ~((uint32_t) 1 << USBSSP_TRB_ISOCH_SIA_POS);
        /* set frameID */
        tempDword3 |= ((ep->frameID & 0x7FFU) << USBSSP_TRB_ISOCH_FRAME_ID_POS);
    } else {
        /* send frame at once when frameID = 0; */
        tempDword3 |= ((uint32_t) 1 << USBSSP_TRB_ISOCH_SIA_POS);
    }

    /* clear TRB field */
    tempDword3 &= ~(trbType << USBSSP_TRB_TYPE_POS);

    /* set ISO TRB field */
    tempDword3 |= (trbType << USBSSP_TRB_TYPE_POS);

    /* clear TBC_TBSTs field */
    tempDword3 &= ~(USBSSP_TRB_TBC_TBSTS_MASK);

    if (ete == 0U) {
        tempDword3 |= (burstCount & 0x3U) << USBSSP_TRB_TBC_TBSTS_POS;
    } else {
        uint32_t tempDword2 = le32ToCpu(ep->enqueuePtr->dword2);

        /* Clear TDSIZE_TBC */
        tempDword2 &= ~(USBSSP_TRB_TDSIZE_TBC_MASK);

        tempDword2 |= (burstCount & 0x1FU) << USBSSP_TRB_TDSIZE_TBC_POS;

        ep->enqueuePtr->dword2 = cpuToLe32(tempDword2);
    }
    /* update TRB */
    ep->enqueuePtr->dword3 = cpuToLe32(tempDword3);
}

/**
 * Function updates endpoint object to stream object
 * @param epObj endpoint object
 */
static void updateEpObjToStream(USBSSP_ProducerQueueT **epObj) {

    USBSSP_ProducerQueueT *ep = *epObj;

    /* if stream used, switch endpoint object to stream object */
    if (ep->actualSID > 0U) {
        *epObj = ep->stream[ep->actualSID - 1U];
    }
}

/**
 * Create single TRB in transfer ring
 * @param res driver resources
 * @param dataPtr address of data
 * @param singleTrbLength length of this data chunk
 * @param tdParams pointer to extra parameters (used internally)
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Parasoft false violation with const specifier for res, DRV-3821" */

static void createSingleTrb(USBSSP_DriverResourcesT *res, uintptr_t dataPtr, uint32_t singleTrbLength, USBSSP_TDCreateT * tdParams) {
    /* default transfer TRB as normal */
    uint32_t trbType = USBSSP_TRB_NORMAL;
    uint32_t linkTrbChainFlag = USBSSP_TRB_NORMAL_CH_MASK;

    /* get endpoint object */
    USBSSP_ProducerQueueT *ep = &res->ep[tdParams->epIndex];

    /* set by default CHAIN flag */
    uint32_t flags = USBSSP_TRB_NORMAL_ISP_MASK | USBSSP_TRB_NORMAL_CH_MASK;

    /* calculate TD size */
    uint32_t tdSize = calculateTdSize(singleTrbLength, tdParams);

    /* switch to stream object if stream used */
    updateEpObjToStream(&ep);

    if ((tdParams->isLastTrb == 1U) && (tdParams->isLastPage == 1U)) {
        if ((ep->extraFlags & (uint8_t) USBSSP_EXTRAFLAGSENUMT_FORCELINKTRB) == 0U) {
            flags = USBSSP_TRB_NORMAL_IOC_MASK;
            linkTrbChainFlag = 0U;
        }
    }

    /* Create TRB */
    set64Value(
        &ep->enqueuePtr->dword0,
        &ep->enqueuePtr->dword1,
        cpuToLe64((uint64_t) (dataPtr))
        );

    ep->enqueuePtr->dword2 = cpuToLe32((((uint32_t) ep->interrupterIdx) << USBSSP_TRB_INTR_TRGT_POS)
                                       | (singleTrbLength & 0x0001FFFFU)
                                       | (tdSize << 17));

    ep->enqueuePtr->dword3 = cpuToLe32((uint32_t) (
                                           (trbType << USBSSP_TRB_TYPE_POS)
                                           | flags
                                           | (uint32_t) ep->toogleBit)
                                       );

    /* update TRB for iso transfer only in first TRB of TD */
    if ((ep->epDesc[3] == CH9_USB_EP_ISOCHRONOUS) && (tdParams->isFirstPage == 1U) && (tdParams->isFirstTrb == 1U)) {
        updateForIsoTrb(res, ep, tdParams);
    }

    /* remember first TRB in TD */
    if ((tdParams->isFirstPage == 1U) && (tdParams->isFirstTrb == 1U)) {
        ep->firstQueuedTRB = ep->enqueuePtr;
    }

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "ContextIndex:%d NOMAL_TRB dw0:0x%08x dw1:0x%08x dw2:0x%08x dw3:0x%08x\n",
            ep->contextIndex, ep->enqueuePtr->dword0, ep->enqueuePtr->dword1, ep->enqueuePtr->dword2, ep->enqueuePtr->dword3);

    /* remember last TRB in TD */
    ep->lastQueuedTRB = ep->enqueuePtr;

    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "ContextIndex:%d address: 0x%x, length: %d, tdSize: %d, flags: %08x, trbTransferLengthSum: %d, packetTransfered: %d, interrupterIdx(%d)\n",
            ep->contextIndex, dataPtr, singleTrbLength, tdSize, flags, tdParams->trbTransferLengthSum, tdParams->packetTransfered, ep->interrupterIdx);
    updateQueuePtr(ep, linkTrbChainFlag);
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 "Parasoft false violation with const specifier for res, DRV-3821" */

/**
 * Calculates single TRB length
 * @param numOfTrb number of TRB
 * @param dataLength data length
 * @param buffStart pointer to buffer start
 * @return singleTrbLength
 */
static uint32_t calculateSingleTrbLength(uint32_t numOfTrb, uint32_t dataLength, uintptr_t buffStart) {

    uint32_t singleTrbLength;

    /* if there is only one TRB size equals to dataLength for this page */
    if (numOfTrb == 1U) {
        singleTrbLength = dataLength;
    } else {
        singleTrbLength = (uint32_t) (USBSSP_TRB_MAX_TRANSFER_LENGTH - (buffStart % USBSSP_TRB_MAX_TRANSFER_LENGTH));
    }

    return singleTrbLength;
}

/**
 * Function creates TRBs for single memory page
 * @param res driver resources
 * @param dataPtr pointer to memory where actual data pointer is stored
 * @param dataLength data length
 * @param tdParams pointer to extra parameters (used internally)
 */
static void trbSinglePage(USBSSP_DriverResourcesT *res, uintptr_t *dataPtr, uint32_t dataLength, USBSSP_TDCreateT * tdParams) {

    uint32_t trbIndex; /* used as enumerator in for loop */
    uintptr_t buffStart = *dataPtr; /* keeps original address of data start */
    uintptr_t endAddress = buffStart + (uintptr_t) dataLength;

    /* calculate number of TRBs */
    uint32_t numOfTrb = dataLength / USBSSP_TRB_MAX_TRANSFER_LENGTH;

    tdParams->isLastTrb = 0U;

    /* round up number of packets */
    if ((dataLength % USBSSP_TRB_MAX_TRANSFER_LENGTH) > 0U) {
        ++numOfTrb;
    }

    /* for data length = 0 */
    if (dataLength == 0U) {
        numOfTrb = 1U;
    }

    for (trbIndex = 0U; trbIndex < numOfTrb; trbIndex++) {

        /* calculate date length of single TRB */
        uint32_t singleTrbLength;

        /* reset flag */
        tdParams->isFirstTrb = 0U;

        /* for first TRB */
        if (trbIndex == 0U) {

            /* mark first TRB */
            tdParams->isFirstTrb = 1U;
            singleTrbLength = calculateSingleTrbLength(numOfTrb, dataLength, buffStart);

        } else if (trbIndex == (numOfTrb - 1U)) {
            /* for last TRB */
            singleTrbLength = (uint32_t) (endAddress - (*dataPtr));
        } else {
            /* elsewhere */
            singleTrbLength = USBSSP_TRB_MAX_TRANSFER_LENGTH;
        }

        /* check if TRB is last */
        if (trbIndex == (numOfTrb - 1U)) {
            tdParams->isLastTrb = 1U;
        }

        vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "TRB(%d):\n", trbIndex);

        /* create single TRB */
        createSingleTrb(res, *dataPtr, singleTrbLength, tdParams);

        /* move data pointer */
        (*dataPtr) += singleTrbLength;
    }
}

/**
 * calculate number of bytes for first memory page
 * @param pageStart address of first page
 * @param pageEnd address of last page
 * @param buff pointer to transfered data
 * @param size size of data buffer
 * @return number of buffer for first memory page
 */
static uint32_t calcFirstPageNumBytes(uintptr_t pageStart, uintptr_t pageEnd, uintptr_t buff, uint32_t size) {
    uint32_t numOfBytes;
    if (pageStart == pageEnd) {
        /* whole TD is located on the same page */
        numOfBytes = size;
    } else {
        /* TD exceed single page */
        numOfBytes = (uint32_t) (USBSSP_SYSTEM_MEMORY_PAGE_SIZE - (buff % USBSSP_SYSTEM_MEMORY_PAGE_SIZE));
    }
    return numOfBytes;
}

/**
 * calculate number of bytes for last memory page
 * @param buff pointer to transfered data
 * @param size size of data buffer
 * @return number of buffer for last memory page
 */
static uint32_t calcLastPageNumBytes(uintptr_t buff, uint32_t size) {
    uint32_t numOfBytes = (uint32_t) ((buff + (uintptr_t) size) % USBSSP_SYSTEM_MEMORY_PAGE_SIZE);
    return numOfBytes;
}

/**
 * calculate index of transfer descriptor mem pages
 * @param tdParams pointer to extra parameters (used internally)
 * @param buff pointer to data
 * @param size size of data buffer
 */
static void createTdAllMemPagesIndex(USBSSP_TDCreateT * tdParams, const uintptr_t buff, uint32_t const size) {

    /* calculate number of memory pages used for this TD */
    tdParams->pageStart = buff / USBSSP_SYSTEM_MEMORY_PAGE_SIZE;

    /* check if size is greater than zero */
    if (size > 0U) {
        tdParams->pageEnd = ((buff + (uintptr_t) size) - 1U) / USBSSP_SYSTEM_MEMORY_PAGE_SIZE;
    } else {
        tdParams->pageEnd = tdParams->pageStart;
    }
    tdParams->numOfPages = (tdParams->pageEnd - tdParams->pageStart) + 1U;
}

/**
 * create transfer descriptors
 * @param res driver resources
 * @param tdParams pointer to extra parameters (used internally)
 * @param buff pointer to data
 * @param size size of data buffer
 */
static void createTdAllMemPages(USBSSP_DriverResourcesT *res, USBSSP_TDCreateT * tdParams, const uintptr_t buff, uint32_t const size) {

    uintptr_t pageIndex; /* page enumerator */

    /* calculate first, last pages */
    createTdAllMemPagesIndex(tdParams, buff, size);

    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> pageStart: 0x%x, pageEnd: 0x%x, numOfpages: %d tdPacketCount: %d\n", res->instanceNo, tdParams->pageStart, tdParams->pageEnd, tdParams->numOfPages, tdParams->tdPacketCount);

    /* create TRBs for every memory page */
    for (pageIndex = 0; pageIndex < tdParams->numOfPages; pageIndex++) {

        uint32_t numOfBytes;
        tdParams->isFirstPage = 0U;

        /* for first page */
        if (pageIndex == 0U) {
            tdParams->isFirstPage = 1U;
            numOfBytes = calcFirstPageNumBytes(tdParams->pageStart, tdParams->pageEnd, buff, size);
            /* for last page */
        } else if (pageIndex == ((uintptr_t) tdParams->numOfPages - 1U)) {
            numOfBytes = calcLastPageNumBytes(buff, size);
        } else {
            /* for middle pages */
            numOfBytes = (uint32_t) USBSSP_SYSTEM_MEMORY_PAGE_SIZE;
        }

        /* check if page is last page in TD */
        if (pageIndex == ((uintptr_t) tdParams->numOfPages - 1U)) {
            tdParams->isLastPage = 1U;
        }

        vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> page(%d)\n", res->instanceNo, pageIndex);

        /* create single page TRBS */
        trbSinglePage(res, &tdParams->dataPtr, numOfBytes, tdParams);
    }
}

/**
 * Function creates transfer descriptor in TRB ring
 * @param res driver resources
 * @param index endpoint index in device context
 * @param buff data user buffer
 * @param size data length of user buffer
 */
static void USBSSP_CreateTD(USBSSP_DriverResourcesT *res, uint8_t const index, const uintptr_t buff, uint32_t const size, USBSSP_TDCreateT *tdInputParams) {

    USBSSP_TDCreateT * tdParams;
    USBSSP_TDCreateT tdParamsAlloc;

    /* check if tdParams is from external function */
    if (tdInputParams != NULL) {
        tdParams = tdInputParams;
    } else {
        /* create own tdParams object */
        tdParams = &tdParamsAlloc;

        /* set extra variables used for tdSize calculation */
        tdParams->packetTransfered = 0U;
        tdParams->trbTransferLengthSum = 0U;
        tdParams->isLastPage = 0U;
        tdParams->epIndex = index;
        tdParams->isLastBuffer = 1U; /* only one user buffer is sent */

        /* get max packet size */
        tdParams->epMaxPacketSize = getMaxPacketSize(&res->ep[index]);

        /* calculate number of USB packets */
        /* first check if maxPacketSize > 0 */
        if (tdParams->epMaxPacketSize == 0U) {
            tdParams->tdPacketCount = 1U;
        } else {
            tdParams->tdPacketCount = (size / tdParams->epMaxPacketSize);

            /* round up tdPacketCount */
            if ((size % tdParams->epMaxPacketSize) > 0U) {
                ++tdParams->tdPacketCount;
            }
        }
    }
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> buff: %p, size: %d\n", res->instanceNo, buff, size);
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> Endpoint(index %d) maxPacketSize: %d\n", res->instanceNo, index, tdParams->epMaxPacketSize);
    tdParams->dataPtr = buff;
    createTdAllMemPages(res, tdParams, buff, size);
}

/**
 * Set doorbell for transfer data
 * @param res driver resources
 * @param ep endpoint
 * @return CDN_EOK for success, error code elsewhere
 */
static uint32_t USBSSP_TransferDataDRBL(USBSSP_DriverResourcesT *res, USBSSP_ProducerQueueT const * ep) {

    uint32_t ret = CDN_EOK;

    /* get endpoint context index */
    uint8_t epIndex = ep->contextIndex;

    /* get endpoint state */
    USBSSP_EpContexEpState endpointState = getEndpointStatus(res, epIndex);

    /* handle not stalled endpoint */
    if (endpointState != USBSSP_EP_CONTEXT_EP_STATE_HALTED) {

        uint32_t drblReg = 0U;
        drblReg |= (uint32_t) epIndex;

        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> DRBL(%d)\n", res->instanceNo, epIndex);

        /* send clear feature to endpoint in host mode if endpoint halted */
        if (ep->isDisabledFlag != 0U) {
            /* clear stall will ring doorbell on command completion */
            ret = USBSSP_EndpointSetFeature(res, epIndex, 0);
        } else {
            USBSSP_WriteDoorbell(res, res->actualdeviceSlot, drblReg);
        }
    } else {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> ENDPOINT %d is in not RUNNING state, can not issue DOORBELL - current status: %d\n", res->instanceNo, epIndex, endpointState);
        ret = CDN_EPERM;
    }
    return ret;
}

/**
 * Transfer data on given endpoint. This function is non-blocking type. The XHCI
 * operation result should be checked in complete callback function.
 *
 * @param[in] res driver resources
 * @param[in] ep_index index of endpoint according to xhci spec e.g for ep1out
              ep_index=2, for ep1in ep_index=3, for ep2out ep_index=4 end so on
 * @param[in] buff buffer for data to send or to receive
 * @param[in] size size of data in bytes
 * @param[in] complete pointer to complete callback function
 *
 * @return CDN_EINVAL if selected endpoint index is out of available range
 * @return CDN_EOK if selected endpoint is within available endpoint range
 */
uint32_t USBSSP_TransferData(USBSSP_DriverResourcesT *res,
                             uint8_t                  epIndex,
                             const uintptr_t          buff,
                             uint32_t                 size,
                             USBSSP_Complete          complete) {
    /* check parameters correctness */
    uint32_t ret = (uint32_t) USBSSP_TransferDataSF(res, epIndex);

    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
    } else {
        /* check if transfers are enabled on this endpoint */
        ret = checkEpXferEnabled(res, epIndex);

        if (ret == CDN_EOK) {
            /* get endpoint object */
            USBSSP_ProducerQueueT * ep = &res->ep[epIndex];

            ep->complete = complete;

            /* create TRB's chain */
            USBSSP_CreateTD(res, epIndex, buff, size, NULL);

            ep->isRunningFlag = 1;

            /* set DRBL only on last packet in chain */
            if ((ep->extraFlags & (uint8_t) USBSSP_EXTRAFLAGSENUMT_NODORBELL) == 0U) {
                ret = USBSSP_TransferDataDRBL(res, ep);
            }
        }
    }
    return ret;
}

/**
 * calculate number of USB packets in whole transfer descriptor
 * @param tdParams
 * @param wholeSGLength
 */
static void calcTdPackNum(USBSSP_TDCreateT *tdParams, uint32_t wholeSGLength) {
    /* calculate number of USB packets */
    /* first check if maxPacketSize > 0 */

    if (tdParams->epMaxPacketSize == 0U) {
        tdParams->tdPacketCount = 1U;
    } else {
        tdParams->tdPacketCount = (wholeSGLength / tdParams->epMaxPacketSize);

        /* round up tdPacketCount */
        if ((wholeSGLength % tdParams->epMaxPacketSize) > 0U) {
            ++tdParams->tdPacketCount;
        }
    }
}

/**
 * calculate data sum of all scatter/gather elements
 * @param bufferDesc contains size vector
 * @param sgElementsSum number of scatter/gather elements
 * @return sum of data transfered in scatter/gather transfer
 */
static uint32_t calcSGDataLength(const USBSSP_XferBufferDesc* bufferDesc, uint32_t sgElementsSum) {

    uint32_t i;
    uint32_t wholeSGLength = 0U;

    /* sum of all elements in size vector */
    for (i = 0; i < sgElementsSum; i++) {
        wholeSGLength += bufferDesc[i].sizeVec;
    }

    return wholeSGLength;
}

/**
 * Scatter/gather transfer function
 * @param res driver resources
 * @param epIndex endpoint index
 * @param bufferDesc buffer for user data buffers
 * @param bufferCount number of user buffers
 * @param complete completion callback
 * @return CDN_EOK if success, error code elsewhere
 */
uint32_t USBSSP_TransferVectorData(USBSSP_DriverResourcesT *res, uint8_t epIndex,
                                   const USBSSP_XferBufferDesc* bufferDesc, uint32_t bufferCount, USBSSP_Complete complete) {

    /* check input parameters correctness */
    uint32_t ret = USBSSP_TransferVectorDataSF(res, epIndex, bufferDesc);
    uint32_t i;

    if (ret == CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Transfer data on ep: %d\n", res->instanceNo, epIndex);
        /* check if transfers are enabled on this endpoint */
        ret = checkEpXferEnabled(res, epIndex);
    }
    if (ret == CDN_EOK) {
        USBSSP_ProducerQueueT * ep = &res->ep[epIndex]; /* get endpoint object */
        uint32_t drblReg = 0U;
        uint32_t wholeSGLength;
        USBSSP_TDCreateT tdParams;

        /* set extra variables used for tdSize calculation */
        tdParams.packetTransfered = 0U;
        tdParams.trbTransferLengthSum = 0U;
        tdParams.isLastPage = 0U;
        tdParams.epIndex = epIndex;
        tdParams.isLastBuffer = 0U;

        /* get max packet size */
        tdParams.epMaxPacketSize = getMaxPacketSize(&res->ep[epIndex]);

        /* calculate of whole length */
        wholeSGLength = calcSGDataLength(bufferDesc, bufferCount);

        /* calculate number of packets */
        calcTdPackNum(&tdParams, wholeSGLength);

        drblReg |= (uint32_t) epIndex;

        /* set complete callback */
        ep->complete = complete;

        /* set extra flags */
        ep->extraFlags = (uint8_t) USBSSP_EXTRAFLAGSENUMT_FORCELINKTRB;

        /* build TD with USBSSP_TransferData function with blocked DRBL */
        for (i = 0; i < (bufferCount - 1U); i++) {
            USBSSP_CreateTD(res, epIndex, bufferDesc[i].buffVec, bufferDesc[i].sizeVec, &tdParams);
        }

        /* put last TRB with IOC enabled: clear all extra flags */
        ep->extraFlags = ~((uint8_t) USBSSP_EXTRAFLAGSENUMT_FORCELINKTRB);
        tdParams.isLastBuffer = 1U;
        USBSSP_CreateTD(res, epIndex, bufferDesc[i].buffVec, bufferDesc[i].sizeVec, &tdParams);
        USBSSP_WriteDoorbell(res, res->actualdeviceSlot, drblReg);
        ep->isRunningFlag = 1;
        /* start DRBL */
    }
    return ret;
}

/**
 * Reset port.
 * @param[in] res driver resources
 * @return CDN_EOK if success, error code elsewhere
 */
uint32_t USBSSP_ResetRootHubPort(USBSSP_DriverResourcesT const *res) {

    /* Check if parameters are valid. */
    uint32_t ret = USBSSP_ResetRootHubPortSF(res);
    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
    } else {
        uint32_t ped;
        uint32_t pls;
        uint32_t counter = USBSSP_DEFAULT_TIMEOUT;
        uint8_t portIndex = res->actualPort;
        uint32_t reg = xhciRead32(&res->regs.xhciPortControl[portIndex - 1U].portsc);
        reg |= USBSSP__PORTSC1USB2__PR_MASK;

        xhciWrite32(&res->regs.xhciPortControl[portIndex - 1U].portsc, reg);

        /* poll PED until set to 1 */
        do {
            reg = xhciRead32(&res->regs.xhciPortControl[res->actualPort - 1U].portsc);
            ped = reg & USBSSP__PORTSC1USB3__PED_MASK;
            pls = (reg & USBSSP__PORTSC1USB3__PLS_MASK) >> USBSSP__PORTSC1USB3__PLS_SHIFT;

            /* Reset failed */
            if (pls == USBSSP_RX_DETECT_LINK_STATE) {
                vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Root Hub Reset failed.\n", 0);
                ret = CDN_EPROTO;
                break;
            }

            counter--;
            CPS_DelayNs(1000);
        } while ((ped == 0U) && (counter != 0U));

        /* check if loop ended with timeout */
        if (counter == 0U) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "%s() timeout after %d\n",
                    __func__, USBSSP_DEFAULT_TIMEOUT);
            ret = CDN_ETIMEDOUT;
        }
    }
    return ret;
}

/**
 * Enqueues the Stop endpoint command in the command queue
 *
 * @param[in] res driver resources
 * @param[in] endpoint index of endpoint to stop
 */
static void enqueueStopEndpointCmd(USBSSP_DriverResourcesT *res, uint8_t endpoint) {
    res->commandQ.enqueuePtr->dword3 = cpuToLe32(
        (uint32_t) ((uint32_t) res->actualdeviceSlot << USBSSP_SLOT_ID_POS)
        | (uint32_t) ((uint32_t) endpoint << USBSSP_ENDPOINT_POS)
        | (uint32_t) (USBSSP_TRB_STOP_EP_CMD << USBSSP_TRB_TYPE_POS)
        | res->commandQ.toogleBit
        );

    updateQueuePtr(&res->commandQ, 0U);
}

/**
 * Stop endpoint. Function sends STOP_ENDPOINT_COMMAND command to SSP controller
 *
 * @param[in] res driver resources
 * @param[in] endpoint index of endpoint to stop
 * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_StopEndpoint(USBSSP_DriverResourcesT *res, uint8_t endpoint) {

    /* check input parameters */
    uint32_t ret = USBSSP_StopEndpointSF(res, endpoint);

    /* return CDN_EINVAL if parameters are not correct */
    if (ret != (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
    }

    if (ret == CDN_EOK) {
        /* enqueue stop endpoint command */
        enqueueStopEndpointCmd(res, endpoint);
        hostCmdDoorbell(res);
    }

    return ret;
}

/**
 * Enqueues RESET_ENDPOINT_COMMAND in the command queue. Doesn't ring doorbell
 *
 * @param[in] res driver resources
 * @param[in] endpoint index of endpoint to reset
 */
static void enqueueResetEndpointCmd(USBSSP_DriverResourcesT *res, uint8_t endpoint) {
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> reset endpoint %d\n", res->instanceNo, endpoint);
    res->commandQ.enqueuePtr->dword3 = cpuToLe32(
        (uint32_t) ((uint32_t) res->actualdeviceSlot << USBSSP_SLOT_ID_POS)
        | (uint32_t) ((uint32_t) endpoint << USBSSP_ENDPOINT_POS)
        | (uint32_t) (USBSSP_TRB_RESET_EP_CMD << USBSSP_TRB_TYPE_POS)
        | res->commandQ.toogleBit
        /*| 0x200 */
        );
    updateQueuePtr(&res->commandQ, 0U);
}

/**
 * Reset of endpoint. Function sends RESET_ENDPOINT_COMMAND to SSP controller
 *
 * @param[in] res driver resources
 * @param[in] endpoint index of endpoint to reset
 * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_ResetEndpoint(USBSSP_DriverResourcesT *res, uint8_t endpoint) {

    /* check input parameters */
    uint32_t ret = USBSSP_ResetEndpointSF(res, endpoint);

    /* if input parameters are not correct return CDN_EINVAL error */
    if (ret != (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
    }

    if (ret == CDN_EOK) {
        /* enqueue reset endpoint command */
        enqueueResetEndpointCmd(res, endpoint);
        hostCmdDoorbell(res);
    }

    return ret;
}

/**
 * Reset of connected device. Function sends RESET_DEVICE_COMMAND to SSP controller
 * in order to issue reset state on USB bus.
 *
 * @param[in] res driver resources
 * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
 * @return CDN_EOK if selected endpoint is within available endpoint range
 */
uint32_t USBSSP_ResetDevice(USBSSP_DriverResourcesT *res) {

    /* check if input parameter is not NULL */
    uint32_t ret = USBSSP_ResetDeviceSF(res);

    /* return error code when input parameter is NULL */
    if (ret != (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! wrong value in one of function parameters\n", 0);
    } else {
        /* reset device address */
        res->devAddress = 0;
        res->ep0.isRunningFlag = 0;

        /* issue reset device command to controller */
        res->commandQ.enqueuePtr->dword3 = cpuToLe32((uint32_t) (
                                                         (uint32_t) ((uint32_t) res->actualdeviceSlot << USBSSP_SLOT_ID_POS)
                                                         | (USBSSP_TRB_RESET_DEVICE_COMMAND << USBSSP_TRB_TYPE_POS)
                                                         | (uint32_t) res->commandQ.toogleBit)
                                                     );
        updateQueuePtr(&res->commandQ, 0U);
        hostCmdDoorbell(res);
    }

    return ret;
}

/**
 * Force header command.
 *
 * @param[in] res driver resources
 * @param[in] trbDwords
 * @param complete completion callback
 * @return CDN_EOK if success, error code elsewhere
 */
uint32_t USBSSP_ForceHeader(USBSSP_DriverResourcesT *res, const USBSSP_ForceHdrParams *trbDwords, USBSSP_Complete complete) {

    /* check if pointer to resources is not NULL */
    uint32_t ret = USBSSP_ForceHeaderSF(res, trbDwords);
    if (ret != (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);

    } else {
        uint32_t dword0 = trbDwords->dword0;
        uint32_t dword1 = trbDwords->dword1;
        uint32_t dword2 = trbDwords->dword2;
        uint32_t dword3 = trbDwords->dword3;
        /* fill TRB with FORCE HEADER command */
        res->commandQ.enqueuePtr->dword0 = cpuToLe32(dword0);
        res->commandQ.enqueuePtr->dword1 = cpuToLe32(dword1);
        res->commandQ.enqueuePtr->dword2 = cpuToLe32(dword2);
        res->commandQ.enqueuePtr->dword3 = cpuToLe32(dword3);
        res->commandQ.enqueuePtr->dword3 &= 0xFFFFFFFEU;
        res->commandQ.enqueuePtr->dword3 |= (uint32_t) res->commandQ.toogleBit;

        /* set internal force header complete callback to this function caller's callback */
        res->forceHeaderComplete = complete;
        updateQueuePtr(&res->commandQ, 0U);
        hostCmdDoorbell(res);
    }
    return ret;
}

/**
 * Function enables slot on connected device
 * @param[in] res driver resources
 * @return CDN_EOK if success, error code elsewhere
 */
uint32_t USBSSP_EnableSlot(USBSSP_DriverResourcesT *res) {

    /* call sanity function */
    uint32_t ret = USBSSP_EnableSlotSF(res);
    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
    } else {
        /* check if enable slot is pending */
        if (res->enableSlotPending == 0U) {
            /* update trb */
            res->commandQ.enqueuePtr->dword3 = cpuToLe32((uint32_t) (
                                                             (USBSSP_TRB_ENABLE_SLOT_COMMAND << USBSSP_TRB_TYPE_POS) | (uint32_t) res->commandQ.toogleBit)
                                                         );
            updateQueuePtr(&res->commandQ, 0U);
            /* ring the dorbell */
            hostCmdDoorbell(res);
            res->enableSlotPending = 1U;
        }
    }
    return ret;
}

/**
 * Function disables slot on enabled device
 * @param[in] res driver resources
 * @return CDN_EOK if success, error code elsewhere
 */
uint32_t USBSSP_DisableSlot(USBSSP_DriverResourcesT * res) {

    uint32_t ret = USBSSP_DisableSlotSF(res);
    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);

    } else {

        uint8_t epIndex;
        USBSSP_EpContexEpState epState;

        /* first execute stop endpoint command on all running endpoints */
        for (epIndex = 1U; epIndex < USBSSP_EP_CONT_MAX; epIndex++) {
            epState = getEndpointStatus(res, epIndex);
            if (epState != USBSSP_EP_CONTEXT_EP_STATE_RUNNING) {
                continue;
            } else {
                uint32_t retValue;
                retValue = USBSSP_StopEndpoint(res, epIndex);
                if (retValue != CDN_EOK) {
                    vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Stop endpoint(%d) command error\n", res->instanceNo, epIndex);
                }
            } /*else */
        } /*for */

        res->commandQ.enqueuePtr->dword3 = cpuToLe32((uint32_t) (
                                                         (uint32_t) ((uint32_t) res->actualdeviceSlot << USBSSP_SLOT_ID_POS)
                                                         | (USBSSP_TRB_DISABLE_SLOT_COMMAND << USBSSP_TRB_TYPE_POS)
                                                         | res->commandQ.toogleBit)
                                                     );
        updateQueuePtr(&res->commandQ, 0U);
        /*res->actualPort = 0; */
        res->actualSpeed = CH9_USB_SPEED_UNKNOWN;
        res->actualdeviceSlot = 0;
        res->devAddress = 0;
        res->ep0.isRunningFlag = 0;
        setDevConfigFlag(res, 0);
        hostCmdDoorbell(res);
    }
    return ret;
}

/**
 * Reads the capability registers.
 * Should be called after regs->xhciCapability is initialized.
 *
 * @param[in] xHCCapsAddr pointer to the xHC capability registers
 * @param[in] qaRegs pointer to the structure for storing quick Access registers
 */
static void initReadCapRegs(USBSSP_CapabilityT *xHCCapsAddr, USBSSP_QuickAccessRegs * qaRegs) {

    /* get pointer to the structure for storing capability register values */
    USBSSP_CapabilityT *xHCCapsRegValues = &qaRegs->xHCCaps;

    xHCCapsRegValues->hcsparams1 = xhciRead32(&xHCCapsAddr->hcsparams1);
    xHCCapsRegValues->hcsparams2 = xhciRead32(&xHCCapsAddr->hcsparams2);
    xHCCapsRegValues->hcsparams3 = xhciRead32(&xHCCapsAddr->hcsparams3);

    xHCCapsRegValues->dboff = xhciRead32(&xHCCapsAddr->dboff);
    xHCCapsRegValues->rtsoff = xhciRead32(&xHCCapsAddr->rtsoff);

    xHCCapsRegValues->hccparams1 = xhciRead32(&xHCCapsAddr->hccparams1);
    xHCCapsRegValues->hccparams2 = 0U;
}

/**
 * Sets internal pointers to extended capabilities register in driver resources
 * @param res driver resources
 * @param base memory address where controller is mapped
 */
static void setSwRegsDrblExCap(USBSSP_DriverResourcesT * res, uintptr_t base) {

    USBSSP_SfrT *regs = &res->regs;
    USBSSP_CapabilityT *xHCCaps = &res->qaRegs.xHCCaps;
    uint32_t temp;
    uint32_t *extCapsBaseAddr;

    /*  ----------- set doorbell register -------------------------------------- */
    /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "uintptr_t converted to uint32_t*, DRV-4287" */
    regs->xhciDoorbell = (uint32_t*) ((xHCCaps->dboff) + base);
    /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> XHCI doorbell address: %p\n", res->instanceNo, regs->xhciDoorbell);

    /* ---------- set extended capabilities registers ------------- */
    temp = CPS_FLD_READ(USBSSP__HCCPARAMS, XECP, xHCCaps->hccparams1);
    if (temp > 0U) {
        temp <<= 2; /* offset is given in DWORDs so multiply by 4 */
        /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "uintptr_t converted to uint32_t*, DRV-4288" */
        extCapsBaseAddr = (uint32_t*) (temp + base);
        /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */
        regs->xhciExtCaps.extCapsBaseAddr = extCapsBaseAddr;
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> XHCI extended capabilities base address: %p\n", res->instanceNo, regs->xhciExtCaps.extCapsBaseAddr);
    } else {
        regs->xhciExtCaps.extCapsBaseAddr = NULL;
    }
}

/**
 * Sets internal pointers to register in driver resources
 * @param res driver resources
 * @param base memory address where controller is mapped
 */
static void setSwRegs(USBSSP_DriverResourcesT * res, uintptr_t base) {

    USBSSP_SfrT *regs = &res->regs;
    USBSSP_CapabilityT *xHCCaps = &res->qaRegs.xHCCaps;
    uint32_t temp;

    /* ------------- set capability register address ---------------------- */

    /* set base address */
    /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "uintptr_t converted to USBSSP_CapabilityT*, DRV-4289" */
    regs->xhciCapability = (USBSSP_CapabilityT*) base;
    /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> XHCI capability address: %p\n", res->instanceNo, &regs->xhciCapability->caplength_hciver);

    /* set operability offset */
    xHCCaps->caplength_hciver = xhciRead32(&regs->xhciCapability->caplength_hciver);

    /* read the xHCI capability registers and keep a local copy */
    initReadCapRegs(regs->xhciCapability, &res->qaRegs);

    /* ------------- set operational register -------------------------------- */

    temp = CPS_FLD_READ(USBSSP__HCIVERSION_CAPLENGTH, CAPLENGTH, (xHCCaps->caplength_hciver));
    if (temp > 0U) {
        /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "uint8_t* converted to USBSSP_OperationalT*, DRV-4290" */
        regs->xhciOperational = (USBSSP_OperationalT*) (base + temp);
        /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> XHCI operational address: %p\n", res->instanceNo, &regs->xhciOperational->usbcmd);
    }
    /* ------------- set runtime register -------------------------------------- */

    /* set runtime offset */
    /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "uint32_t* converted to USBSSP_RuntimeT*, DRV-4291" */
    regs->xhciRuntime = (USBSSP_RuntimeT*) ((xHCCaps->rtsoff) + base);
    /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> XHCI runtime address: %p\n", res->instanceNo, &regs->xhciRuntime->mfindex);

    /* ------------ set port control registers --------------------------------- */
    regs->xhciPortControl = &regs->xhciOperational->portControl;

    /* ------------- set interrupters registers -------------------------------- */
    regs->xhciInterrupter = &regs->xhciRuntime->interrupters;

    /* set extended capabilities pointer */
    setSwRegsDrblExCap(res, base);
}

#ifdef DEBUG
/**
 * Displays controller feature: interrupters
 * @param res driver resources
 */
static void initSfrObjDispInterrupter(USBSSP_DriverResourcesT * res) {

    const USBSSP_CapabilityT *xHCCaps = &res->qaRegs.xHCCaps;
    USBSSP_SfrT *regs = &res->regs;
    uint8_t numOfInterrupters;
    uint8_t i;

    /* ------------- interrupters ---------------------------------------------- */
    numOfInterrupters = (uint8_t) CPS_FLD_READ(USBSSP__HCSPARAMS1, MAXINTRS, xHCCaps->hcsparams1);
    /* display addresses of all interrupters */
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Number of interrupters: %d\n", res->instanceNo, numOfInterrupters);
    for (i = 0U; i < numOfInterrupters; i++) {

        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> interrupter[%d] address: %p\n",
                res->instanceNo, i, &regs->xhciInterrupter[i].iman);
    }
}

/**
 * Displays controller feature: ports
 * @param res driver resources
 */
static void initSfrObjDispPorts(USBSSP_DriverResourcesT * res) {

    const USBSSP_CapabilityT *xHCCaps = &res->qaRegs.xHCCaps;
    USBSSP_SfrT *regs = &res->regs;
    uint8_t numOfPorts;
    uint8_t i;

    /* --------------- port features ------------------------------------------- */
    /* check number of ports */
    numOfPorts = (uint8_t) CPS_FLD_READ(USBSSP__HCSPARAMS1, MAXPORTS, xHCCaps->hcsparams1);
    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Number of ports: %d\n", res->instanceNo, numOfPorts);
    /* print addresses for all endpoints */
    for (i = 0U; i < numOfPorts; i++) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> XHCI port_control[%d] address: %p\n",
                res->instanceNo, i, &regs->xhciPortControl[i].portsc);
    }
}
#endif

/**
 * Displays controller feature: device slot number
 * @param res driver resources
 */
static void initSfrObjDispDevSlotNum(USBSSP_DriverResourcesT * res) {

    USBSSP_CapabilityT *xHCCaps = &res->qaRegs.xHCCaps;
    uint32_t maxDeviceSlot;

    /* --------------- device slots -------------------------------------------- */
    maxDeviceSlot = CPS_FLD_READ(USBSSP__HCSPARAMS1, MAXSLOTS, xHCCaps->hcsparams1);
    /* if override  condition */
    if (res->maxDeviceSlot != 0U) {
        maxDeviceSlot = (res->maxDeviceSlot > maxDeviceSlot) ? maxDeviceSlot : res->maxDeviceSlot;
    }

    if (maxDeviceSlot > USBSSP_MAX_DEVICE_SLOT_NUM) {
        res->maxDeviceSlot = USBSSP_MAX_DEVICE_SLOT_NUM;
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Limiting Number of Slots from(%d) to(%d)\n", res->instanceNo, maxDeviceSlot, USBSSP_MAX_DEVICE_SLOT_NUM);
    } else {
        res->maxDeviceSlot = maxDeviceSlot;
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Number of device slots: %d\n", res->instanceNo, res->maxDeviceSlot);
    }
}

/**
 * Displays controller feature: extended capabilities
 * @param res driver resources
 */
static void initSfrObjDispExtCap(USBSSP_DriverResourcesT * res) {

    USBSSP_SfrT *regs = &res->regs;
    uint8_t i;

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> ExtCap base address: 0x%p, USBLEGSUP=0x%08x / USBLEGCTLS=0x%08x\n",
            res->instanceNo, (uintptr_t) regs->xhciExtCaps.extCapsBaseAddr, regs->xhciExtCaps.usbLegSup, regs->xhciExtCaps.usbLegCtlSts);

    for (i = 0U; i < regs->xhciExtCaps.extCapsCount; i++) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Ext. cap. %d @(%p): firstCapPtr:%p, DWORD[0]=0x%08x, capID=%d\n",
                res->instanceNo, i, (uintptr_t) &regs->xhciExtCaps.extCaps[i], regs->xhciExtCaps.extCaps[i].firstCapSfrPtr, regs->xhciExtCaps.extCaps[i].firstDwordVal, regs->xhciExtCaps.extCaps[i].capId);
    }
}

/**
 * initialize extension capabilities
 * @param res driver resources
 */
static void initSfrObjExtCap(USBSSP_DriverResourcesT * res) {

    uint8_t numOfExtCaps;
    uint8_t nextCapPtr;
    uint32_t *extCapsBaseAddr;
    uint32_t *extCapSfrIter;
    USBSSP_ExtCapElemT *extCapElemIter;
    USBSSP_SfrT *regs = &res->regs;

    /* Reading USBLEGSUP and USBLEGCTLSTS */

    extCapsBaseAddr = regs->xhciExtCaps.extCapsBaseAddr;
    regs->xhciExtCaps.usbLegSup = xhciRead32(extCapsBaseAddr);
    regs->xhciExtCaps.usbLegCtlSts = xhciRead32(&extCapsBaseAddr[1]);

    numOfExtCaps = 0;

    extCapElemIter = &(regs->xhciExtCaps.extCaps[0]);
    nextCapPtr = (uint8_t) CPS_FLD_READ(USBSSP__XECP_USBLEGSUP, NEXTCP, regs->xhciExtCaps.usbLegSup);
    extCapSfrIter = &extCapsBaseAddr[nextCapPtr];

    while ((numOfExtCaps < USBSSP_MAX_EXT_CAPS_COUNT) && (nextCapPtr > 0U)) {
        uint8_t capId;

        /* Reading first DWORD of ExtCap and decoding next capability pointer / capability ID */
        uint32_t firstExtCapDword = xhciRead32(extCapSfrIter);
        /* Next Extended Capabilities pointer is always in the same place */
        nextCapPtr = (uint8_t) CPS_FLD_READ(USBSSP__XECP_USBLEGSUP, NEXTCP, firstExtCapDword);
        /* Capability ID is always in the same place */
        capId = (uint8_t) CPS_FLD_READ(USBSSP__XECP_USBLEGSUP, CID, firstExtCapDword);

        if (capId > 0U) {
            /* Storing first DWORD of data, Capability ID and pointer to first SFR */
            extCapElemIter->firstDwordVal = firstExtCapDword;
            extCapElemIter->capId = capId;
            extCapElemIter->firstCapSfrPtr = extCapSfrIter;
        }

        /* Advancing number of capabilities and moving pointers */
        numOfExtCaps++;
        extCapElemIter++;
        extCapSfrIter = &extCapSfrIter[nextCapPtr];
    }

    /* Updating Ext. Caps number */
    regs->xhciExtCaps.extCapsCount = numOfExtCaps;
}

/**
 * Initialization of register object. Fields of regs object are consistent with
 * XHCI specification, all pointers of xhci_sfr_t object are set according to
 * capability registers information
 *
 * @param[in] res driver resources
 * @param[in] base physical address where SSP controller is mapped
 */
static void initSfrObj(USBSSP_DriverResourcesT * res, uintptr_t base) {

    /* set driver resources internal pointer to controller register */
    setSwRegs(res, base);

#ifdef DEBUG
    /* display port info */
    initSfrObjDispPorts(res);
#endif

    /* display device slot number info */
    initSfrObjDispDevSlotNum(res);

#ifdef DEBUG
    /* display interrupters info */
    initSfrObjDispInterrupter(res);
#endif

    /* initialize extended capabilities */
    initSfrObjExtCap(res);

    /* display extended capabilities */
    initSfrObjDispExtCap(res);
}

/**
 * Allocate memory buffers required by SSP controller
 * @param res driver resources
 */
static void initAllocateBuffers(USBSSP_DriverResourcesT *res) {

    /* initialize xhci memory, all buffers should be allocated according to rules */
    /* described in table 54 (chapter 6) of XHCI specification */

    USBSSP_XhciResourcesT *memRes = res->xhciMemRes;

    res->ep0Buff = memRes->ep0Buffer;

    /* set address of input context */
    res->inputContext = memRes->inputContext;
}

/**
 * Setup of scratch pad buffers
 * @param res driver resources
 * @return CDN_EOK if setup successful, error code elsewhere
 */
static uint32_t initXhcSetupScratchPad(USBSSP_DriverResourcesT * res) {

    uint16_t i;
    uint32_t ret = CDN_EOK;
    USBSSP_CapabilityT *xHCCaps = &res->qaRegs.xHCCaps;

    /* check number of scratch pad buffers */
    uint16_t maxScratchBuf = (uint16_t) ((uint16_t) CPS_FLD_READ(USBSSP__HCSPARAMS2, MAXSPBUFHI, xHCCaps->hcsparams2) << 5U)
                             | (uint16_t) CPS_FLD_READ(USBSSP__HCSPARAMS2, MAXSPBUFLO, xHCCaps->hcsparams2);
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> Number of scratch pad buffers: %d\n", res->instanceNo, maxScratchBuf);

    if (maxScratchBuf > USBSSP_SCRATCHPAD_BUFF_NUM) {
        maxScratchBuf = USBSSP_SCRATCHPAD_BUFF_NUM;
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Reducing number of scratch pad to: %d\n", res->instanceNo, maxScratchBuf);
    }

    /* set pointers to all buffers */
    for (i = 0; i < maxScratchBuf; i++) {
        /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "uint8_t* converted to uint64_t*, DRV-4292" */
        res->xhciMemRes->scratchpad[i] = cpuToLe64((uintptr_t) &res->xhciMemRes->scratchpadPool[i * USBSSP_PAGE_SIZE]);
        /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */
    }
    res->xhciMemRes->scratchpad[maxScratchBuf] = 0;

    /* set first element of DCBAA to 0 if no scratch pad buffers */
    if (maxScratchBuf == 0U) {
        res->xhciMemRes->dcbaa->scratchPadPtr = 0U;
    }
    return ret;
}

#ifdef DEBUG
/**
 * Display capabilities
 * @param res driver resources
 */
static void initXhcSetupDispCap(USBSSP_DriverResourcesT const * res) {

    USBSSP_CapabilityT const *xHCCaps = &res->qaRegs.xHCCaps;
    /* display all capability parameters 5.3.6 in spec */
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> HCCPARAMS1:"
                                        "AC64=%d, "
                                        "BNC=%d, "
                                        "CSZ=%d, "
                                        "PPC=%d, "
                                        "PIND=%d, "
                                        "LHRC=%d, "
                                        "LTC=%d, "
                                        "NSS=%d, "
                                        "PAE=%d, "
                                        "SPC=%d, "
                                        "MaxPSASize=%d, "
                                        "xECP=0x%08x\n",
            res->instanceNo,
            CPS_FLD_READ(USBSSP__HCCPARAMS, AC64, xHCCaps->hccparams1),
            CPS_FLD_READ(USBSSP__HCCPARAMS, BNC, xHCCaps->hccparams1),
            CPS_FLD_READ(USBSSP__HCCPARAMS, CSZ, xHCCaps->hccparams1),
            CPS_FLD_READ(USBSSP__HCCPARAMS, PPC, xHCCaps->hccparams1),
            CPS_FLD_READ(USBSSP__HCCPARAMS, PIND, xHCCaps->hccparams1),
            CPS_FLD_READ(USBSSP__HCCPARAMS, LHRC, xHCCaps->hccparams1),
            CPS_FLD_READ(USBSSP__HCCPARAMS, LTC, xHCCaps->hccparams1),
            CPS_FLD_READ(USBSSP__HCCPARAMS, NSS, xHCCaps->hccparams1),
            CPS_FLD_READ(USBSSP__HCCPARAMS, PAE, xHCCaps->hccparams1),
            CPS_FLD_READ(USBSSP__HCCPARAMS, SPC, xHCCaps->hccparams1),
            CPS_FLD_READ(USBSSP__HCCPARAMS, MAXPSASIZE, xHCCaps->hccparams1),
            CPS_FLD_READ(USBSSP__HCCPARAMS, XECP, xHCCaps->hccparams1)
            );

}
#endif

/**
 * set slot number in controller
 * @param res driver resources
 */
static void initXhcSetupSlotNum(USBSSP_DriverResourcesT * res) {
    /* select minimal slot device number: */
    uint32_t devSlotNum = (res->maxDeviceSlot > USBSSP_MAX_DEVICE_SLOT_NUM)
                          ? USBSSP_MAX_DEVICE_SLOT_NUM : res->maxDeviceSlot;
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> Set slot number: %d\n", res->instanceNo, devSlotNum);
    /* Program the Max Device Slots Enabled (MaxSlotsEn) */
    xhciWrite32(&res->regs.xhciOperational->config,
                devSlotNum);
}

/**
 * Check context size
 * @param res driver resources
 * @return CDN_EOK on success, error code elsewhere
 */
static uint32_t initXhcSetupCheckCntxSize(USBSSP_DriverResourcesT * res) {

    uint32_t ret = CDN_EOK;
    USBSSP_CapabilityT *xHCCaps = &res->qaRegs.xHCCaps;
    uint8_t csz; /* keeps context size */

    /* set slot number */
    initXhcSetupSlotNum(res);
    csz = (uint8_t) CPS_FLD_READ(USBSSP__HCCPARAMS, CSZ, xHCCaps->hccparams1);
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> Context size bit: %d\n", res->instanceNo, csz);
    if (csz != USBSSP_EXTENDED_CONTEXT) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Context size doesn't suit declared CSZ value\n", res->instanceNo);
        ret = CDN_EINVAL;
    }
    return ret;
}

/**
 * Check memory page size which is supported by controller
 * @param res driver resources
 * @return CDN_EOK on success, error code elsewhere
 */
static uint32_t initXhcSetupCheckPageSize(USBSSP_DriverResourcesT * res) {

    uint32_t ret = CDN_EOK;

    uint16_t pageSize;
    uint32_t reg32Value;

    /* check page size */
    reg32Value = xhciRead32(&res->regs.xhciOperational->pagesize);
    pageSize = (uint16_t) CPS_FLD_READ(USBSSP__PAGESIZE, PAGESIZE, reg32Value);
    vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> Page size: %d\n", res->instanceNo, pageSize);

    /* page size = 1 means 4096 size in bytes */
    /* page size = 16 means 64K size in bytes */
    if ((pageSize != 1U) && (pageSize != 16U)) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "<%d> Incorrect page size\n", res->instanceNo);
        ret = CDN_ENOMEM; /* set error code */
    }
    return ret;
}

/**
 * Program the device context base array fields
 * @param res driver resources
 * @return CDN_EOK on success, error code elsewhere
 */
static uint32_t initXhcSetupDcbaa(USBSSP_DriverResourcesT * res) {

    uint32_t ret = CDN_EOK;
    uint64_t dcbaap_sfr_val;

    /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "uint64_t* converted to uint64_t, DRV-4293" */
    res->xhciMemRes->dcbaa->scratchPadPtr = cpuToLe64((uintptr_t) res->xhciMemRes->scratchpad);
    /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */
    res->xhciMemRes->dcbaa->deviceSlot[0] = cpuToLe64((uintptr_t) res->xhciMemRes->outputContext->slot);
    /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "USBSSP_InputContexT* converted to uintptr_t, DRV-4294" */
    ret = checkStructAlign("INPUT CONTEXT", (uintptr_t) res->inputContext, sizeof (USBSSP_InputContexT), USBSSP_CONTEXT_ALIGNMENT, USBSSP_PAGE_SIZE);
    /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */

    if (ret == CDN_EOK) {
        /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "USBSSP_OutputContexT* converted to uintptr_t, DRV-4295" */
        ret = checkStructAlign("OUTPUT CONTEXT", (uintptr_t) res->xhciMemRes->outputContext, sizeof (USBSSP_OutputContexT), USBSSP_CONTEXT_ALIGNMENT, USBSSP_PAGE_SIZE);
        /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */
    }

    if (ret == CDN_EOK) {
        /* Program the Device Context Base Address Array Pointer (DCBAAP) for the scratchpad (slot0) */
        /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "uint64_t* converted to uint64_t, DRV-4296" */
        dcbaap_sfr_val = (uint64_t) (uintptr_t) (&res->xhciMemRes->dcbaa->scratchPadPtr);
        CPS_MemoryBarrier();
        /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */

        xhciWrite64(&res->regs.xhciOperational->dcbaap, dcbaap_sfr_val);

        ret = checkStructAlign("DCBAA", (uintptr_t) (dcbaap_sfr_val), sizeof (USBSSP_DcbaaT), USBSSP_DCBAA_ALIGNMENT, USBSSP_PAGE_SIZE);
    }
    return ret;
}

/**
 * Initialization of SSPDriverResourcesT object: Setup of xHC.
 *
 * @param[in] res driver resources
 * @return CDN_EINVAL or CDN_ENOMEM when unsuccessful
 * @return CDN_EOK if no errors
 */
static uint32_t initXhcSetup(USBSSP_DriverResourcesT * res) {

    uint32_t ret = CDN_EOK;
    USBSSP_OperationalT* xhciOperational = res->regs.xhciOperational;
#ifdef DEBUG
    /* display HCC capabilities */
    initXhcSetupDispCap(res);
#endif
    /* ------ do XHCI reset -------------- */
    /* wait until the Controller Not Ready (CNR) flag in the USBSTS is '0' */
    ret = waitForReg(&xhciOperational->usbsts,
                     USBSSP__USBSTS__CNR_MASK, 0U, USBSSP_DEFAULT_TIMEOUT);

    xhciWrite32(&xhciOperational->usbcmd, USBSSP__USBCMD__HCRST_MASK);

    if (ret == CDN_EOK) {
        ret = waitForReg(&xhciOperational->usbcmd,
                         USBSSP__USBCMD__HCRST_MASK, 0U, USBSSP_DEFAULT_TIMEOUT);
    }
    /* ------ reset end ----------------- */

    /* check context size */
    if (ret == CDN_EOK) {
        ret = initXhcSetupCheckCntxSize(res);
    }

    if (ret == CDN_EOK) {
        ret = initXhcSetupCheckPageSize(res);
    }

    if (ret == CDN_EOK) {
        ret = initXhcSetupScratchPad(res);
    }

    if (ret == CDN_EOK) {
        ret = initXhcSetupDcbaa(res);
    }

    return ret;
}

/**
 * Program interrupters
 * @param res driver resources
 */
static void initRingsInterruptsInterupters(USBSSP_DriverResourcesT * res) {

    uint64_t erdp_sfr_val;
    uint64_t erstba_sfr_val;
    uint32_t interrupter = 0U;

    for (interrupter = 0U; interrupter < USBSSP_MAX_NUM_INTERRUPTERS; interrupter++) {

        USBSSP_RingElementT* eventPtr = res->eventPtrBuffer[interrupter];

        /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "uint64_t* converted to uint64_t, DRV-4297" */
        erstba_sfr_val = (uint64_t) (uintptr_t) (&res->xhciMemRes->eventRingSegmentEntry[USBSSP_ERST_ENTRY_SIZE * interrupter]);
        /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */

        CPS_MemoryBarrier();

        /* Program the Interrupter Event Ring Segment Table Size (ERSTSZ) */
        xhciWrite32(&res->regs.xhciInterrupter[interrupter].erstsz, 1);  /* one event ring */

        /* Program the Interrupter Event Ring Dequeue Pointer (ERDP) */
        erdp_sfr_val = get64PhyAddrOf32ptr(&eventPtr->dword0);
        xhciWrite64(&res->regs.xhciInterrupter[interrupter].erdp, erdp_sfr_val);

        /* Program the Interrupter Event Ring Segment Table Base Address (ERSTBA) */
        xhciWrite64(&res->regs.xhciInterrupter[interrupter].erstba, erstba_sfr_val);

        /* Initializing the Interval field of the Interrupt Moderation register */
        xhciWrite32(&res->regs.xhciInterrupter[interrupter].imod, 0);

        /* Enable the Interrupter by writing a '1' to the Interrupt Enable (IE) */
        xhciWrite32(&res->regs.xhciInterrupter[interrupter].iman, USBSSP__IMAN0__IE_MASK);
    }
}

/**
 * initialize endpoints rings
 * @param res driver resources
 */
static void initRingsInterruptsEpRings(USBSSP_DriverResourcesT * res) {

    uint8_t i;
    USBSSP_ProducerQueueT *ep;

    /* initialize all software endpoints */
    for (i = 1U; i < (USBSSP_MAX_EP_CONTEXT_NUM + USBSSP_EP_CONT_OFFSET); i++) {

        /* index 1 is a default endpoint */
        if (i == 1U) {
            ep = &res->ep0;
        } else {
            /* no default endpoint are organized in container */
            ep = &res->ep[i];
        }

        /* set ring and enqueue/dequeue pointers */
        ep->ring = &res->xhciMemRes->epRingPool[USBSSP_PRODUCER_QUEUE_SIZE * i];
        ep->toogleBit = 1;
        ep->enqueuePtr = ep->ring;
        ep->dequeuePtr = ep->ring;
        vDbgMsg(USBSSP_DBG_DRV, DBG_HIVERB, "<%d> Endpoint context[%d] transfer ring address: %p\n", res->instanceNo, i, (void*) ep->ring);
    }
}

/**
 * initialize command ring
 * @param res driver resources
 */
static void initRingsInterruptsCommandRing(USBSSP_DriverResourcesT * res) {

    /* set start of ring */
    res->commandQ.ring = res->xhciMemRes->epRingPool;

    /* set toggle bit */
    res->commandQ.toogleBit = 1;

    /* reset dequeue and enqueue pointer */
    res->commandQ.dequeuePtr = res->commandQ.ring;
    res->commandQ.enqueuePtr = res->commandQ.ring;
}

/**
 * Initialization of SSPDriverResourcesT object: Setup of rings and interrupts.
 *
 * @param[in] res driver resources
 * @return CDN_EINVAL when unsuccessful
 * @return CDN_EOK if no errors
 */
static uint32_t initRingsInterrupts(USBSSP_DriverResourcesT * res) {

    uint32_t ret = CDN_EOK;
    uint64_t crcr_sfr_val;
    uint64_t erstba_sfr_val;
    uint32_t interrupter = 0U;

    /* initialize software command ring */
    initRingsInterruptsCommandRing(res);

    /* get start address of command ring */
    crcr_sfr_val = get64PhyAddrOf32ptr(&res->commandQ.ring[0].dword0);

    /* set event pointer to start of allocated buffer */
    for (interrupter = 0U; interrupter < USBSSP_MAX_NUM_INTERRUPTERS; interrupter++) {
        res->eventPtrBuffer[interrupter] = res->xhciMemRes->eventPool[interrupter];
    }
    res->eventPtr = res->xhciMemRes->eventPool[0];

    /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "uint64_t* converted to uint64_t, DRV-4298" */
    erstba_sfr_val = (uint64_t) (uintptr_t) (res->xhciMemRes->eventRingSegmentEntry);
    /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */

    /* check if pointers are allocated according to spec requirements */
    ret = checkStructAlign("CMD_RING", (uintptr_t) (crcr_sfr_val),
                           sizeof (USBSSP_RingElementT) * USBSSP_PRODUCER_QUEUE_SIZE, USBSSP_RING_ALIGNMENT, USBSSP_RING_BOUNDARY);
    ret = checkStructAlign("EVT_RING", (uintptr_t) (get64PhyAddrOf32ptr(&res->eventPtr->dword0)),
                           sizeof (USBSSP_RingElementT) * USBSSP_EVENT_QUEUE_SIZE, USBSSP_RING_ALIGNMENT, USBSSP_RING_BOUNDARY);
    ret = checkStructAlign("ERST", (uintptr_t) (erstba_sfr_val),
                           sizeof (uint64_t) * (2U), USBSSP_ERST_ALIGNMENT, USBSSP_ERST_BOUNDARY);

    if (ret == CDN_EOK) {
        /* Define the Command Ring Dequeue Pointer by programming the Command Ring Control Register */
        xhciWrite64(&res->regs.xhciOperational->crcr,
                    (uint64_t) (crcr_sfr_val | res->commandQ.toogleBit));

        /* initialize endpoint rings */
        initRingsInterruptsEpRings(res);

        for (interrupter = 0U; interrupter < USBSSP_MAX_NUM_INTERRUPTERS; interrupter++) {
            /* Allocate the Event Ring Segment Table (ERST) (section 6.5). Initialize ERST table entries */
            /* to point to and to define the size (in TRBs) of the respective Event Ring Segment. */
            res->eventToogleBit[interrupter] = 1;
            res->xhciMemRes->eventRingSegmentEntry[interrupter * USBSSP_ERST_ENTRY_SIZE] = cpuToLe64(get64PhyAddrOf32ptr(&res->eventPtrBuffer[interrupter]->dword0));
            res->xhciMemRes->eventRingSegmentEntry[(interrupter * USBSSP_ERST_ENTRY_SIZE) + 1U] = cpuToLe64(USBSSP_EVENT_QUEUE_SIZE);
        }
        /* initialize interrupters */
        initRingsInterruptsInterupters(res);
    }
    return ret;
}

/**
 * Reset Command ring: called after controller stopped (during SaveState).
 *
 * @param[in] res driver resources
 */
static void resetCmdRing(USBSSP_DriverResourcesT * res) {

    res->commandQ.toogleBit = 1;
    res->commandQ.dequeuePtr = res->commandQ.ring; /* set dequeue pointer to the begin of ring */
    res->commandQ.enqueuePtr = res->commandQ.ring; /* set enqueue pointer to the begin of ring */

    /* clear whole ring to zero */
    (void) memset((void*) res->commandQ.ring, 0, sizeof (USBSSP_RingElementT) * USBSSP_PRODUCER_QUEUE_SIZE);
}

/**
 * Initialization of SSPDriverResourcesT object: Enable xHC.
 *
 * @param[in] res driver resources
 * @return CDN_EINVAL when unsuccessful
 * @return CDN_EOK if no errors
 */
static void initEnableXhc(USBSSP_DriverResourcesT * res) {

    /* Write the USBCMD (5.4.1) to turn the host controller ON via setting the Run/Stop (R/S) bit to 1. */
    /* Enable system bus interrupt generation by writing a '1' to the Interrupter Enable (INTE) */

    uint32_t flags = USBSSP__USBCMD__HSEE_MASK | USBSSP__USBCMD__INTE_MASK | USBSSP__USBCMD__R_S_MASK;

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "<%d> Starting xHC with flags: (0x%08x)\n", res->instanceNo, flags);
    xhciWrite32(&res->regs.xhciOperational->usbcmd, flags);
}

/**
 * Stop xHC.
 *
 * @param[in] res driver resources
 */
static void stopXhc(USBSSP_DriverResourcesT * res) {

    uint32_t usbCmdReg = xhciRead32(&res->regs.xhciOperational->usbcmd);

    /* Clear Run/Stop bit in USBCMD */
    usbCmdReg &= ~(uint32_t) USBSSP__USBCMD__R_S_MASK;
    xhciWrite32(&res->regs.xhciOperational->usbcmd, usbCmdReg);
}

/**
 * Function sets memory resources used by driver
 * @param res driver resources
 * @param memRes user's memory resources
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t USBSSP_SetMemRes(USBSSP_DriverResourcesT *res, USBSSP_XhciResourcesT * memRes) {

    uint32_t ret = CDN_EOK;

    /* check input parameters */
    if ((res == NULL) || (memRes == NULL)) {
        ret = CDN_EINVAL;
    }

    if (ret == CDN_EOK) {
        /* set internal memory resources field */
        res->xhciMemRes = memRes;
    }

    return ret;
}

/**
 * Function checks DID RID register values
 * @param res driver resources
 * @return CDN_EOK on success, error code elsewhere
 */
static uint32_t initCheckDidRidReg(const USBSSP_DriverResourcesT *res) {
    uint32_t ret = CDN_EOK;

    if ((res->ipIdVerRegs.didRegPtr == (uintptr_t) 0) ||
        (res->ipIdVerRegs.ridRegPtr == (uintptr_t) 0)) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Warning! DID, RID register pointers are not set.\n", 0);
    } else {
        /* parasoft-begin-suppress MISRA2012-RULE-11_4 " unsigned long converted to 'uint32_t*', DRV-4299" */
        uint32_t did = xhciRead32((uint32_t*) (res->ipIdVerRegs.didRegPtr)); /* didRegPtr contains address of DID Reg*/
        /* parasoft-end-suppress MISRA2012-RULE-11_4 */
        /* parasoft-begin-suppress MISRA2012-RULE-11_4 " unsigned long converted to 'uint32_t*', DRV-4320" */
        uint32_t rid = xhciRead32((uint32_t*) (res->ipIdVerRegs.ridRegPtr)); /* ridRegPtr contains address of RID Reg*/
        /* parasoft-end-suppress MISRA2012-RULE-11_4 */
        if ((did != USBSSP_DID_VAL) ||
            (rid != USBSSP_RID_VAL)) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Error! DID, RID incorrect values\n", 0);
            ret = CDN_EPERM;
        }
    }
    return ret;
}

/**
 * Initialization of SSPDriverResourcesT object. SSPDriverResourcesT object keeps
 * all resources required by SSP controller. It represents SSP hardware controller.
 *
 * @param[in] res driver resources
 * @param[in] base physical address of SSP controller where it is mapped in system
 * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_Init(USBSSP_DriverResourcesT *res, uintptr_t base) {

    uint32_t ret = USBSSP_InitSF(res);

#ifdef DEBUG
    ret |= checkAddrWidth();
    ret |= checkEndianness();
#endif

    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
    }

    if (ret == CDN_EOK) {
        ret = initCheckDidRidReg(res);
    }

    if (ret == CDN_EOK) {
        uintptr_t baseAddress = base;

#if defined USBSSP_DEMO_TB
        if (res->xhciMemRes == NULL) {
            (void) USBSSP_SetMemResCallback(res);
        }
#endif

        /* check if controller memory reserved */
        if (res->xhciMemRes == NULL) {
            ret = CDN_ENOMEM;
        }

        if (ret == CDN_EOK) {

            /* Allocate buffers */
            initAllocateBuffers(res);

            /* build register object */
            initSfrObj(res, baseAddress);

            if (res->noControllerSetup == 0U) {
                /* Set up xHC and slot0 (scratchpad) */
                ret = initXhcSetup(res);
            }
        }

        if (ret == CDN_EOK) {
            /* Set up rings (control, transfer, event) and interrupts */
            ret = initRingsInterrupts(res);
        }

        if (ret == CDN_EOK) {
            /* Enable interrupts and place Controller in Run state. */
            initEnableXhc(res);
        }
    }

    return ret;
}

/**
 * Stops all end points which are not stopped/disabled.
 * @param res driver resources
 * @return flag indicating which all endpoints are being stopped.
 */
static uint32_t stopActiveEndpoints(USBSSP_DriverResourcesT * res) {
    uint32_t ret = CDN_EOK;
    uint32_t activeEpMask = 0U;
    uint8_t epIndex = 0;

    vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Stopping endpoints for driver instance: <%d>\n", res->instanceNo);
    for (epIndex = 31U; epIndex > 0U; epIndex--) {
        /* get endpoint status */
        USBSSP_EpContexEpState epState = getEndpointStatus(res, epIndex);
        if (epState == USBSSP_EP_CONTEXT_EP_STATE_RUNNING) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Stopping endpoints <%d> from EP_STATE_RUNNING\n", epIndex);
            enqueueStopEndpointCmd(res, epIndex); /* issue stop */
            activeEpMask |= ((uint32_t) 1U << epIndex);
        } else if (epState == USBSSP_EP_CONTEXT_EP_STATE_ERROR) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Error: Epindex (%d) Unexpected EP_STATE_ERROR\n", epIndex);
        } else if (epState == USBSSP_EP_CONTEXT_EP_STATE_HALTED) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Reset endpoint <%d> from EP_STATE_HALTED\n", epIndex);
            enqueueResetEndpointCmd(res, epIndex);
            activeEpMask |= ((uint32_t) 1U << epIndex);
        } else {
            /* endpoint either stopped or disabled */
        }
    }

    if (activeEpMask != 0U) {
        hostCmdDoorbell(res);
        /* wait for all endpoints to be stopped. */
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Waiting for all endpoints to stop\n", 0);
        ret = waitUntilEpStoppedDisabled(res, USBSSP_DEFAULT_TIMEOUT);
        if (ret != CDN_EOK) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error: Timed-out waiting for endpoints to stop \n", 0);
        }
    }
    return ret;
}

/**
 * Save the operational and interrupter registers
 * @param res: driver resources
 * @param drvContext: Saved driver context
 */
static void saveStateRegisters(const USBSSP_DriverResourcesT * res, USBSSP_DriverContextT * drvContext) {
    USBSSP_OperationalT *xhciOperationalRegs = res->regs.xhciOperational;
    USBSSP_InterrupterT *xhciInterrupter0Regs = &res->regs.xhciInterrupter[0];

    /* save operational registers */
    drvContext->usbcmd = xhciRead32(&xhciOperationalRegs->usbcmd);
    drvContext->dnctrl = xhciRead32(&xhciOperationalRegs->dnctrl);
    drvContext->dcbaap = xhciRead64(&xhciOperationalRegs->dcbaap);
    drvContext->config = xhciRead32(&xhciOperationalRegs->config);

    /* save interrupter registers */
    drvContext->erstsz = xhciRead32(&xhciInterrupter0Regs->erstsz);
    drvContext->erstba = xhciRead64(&xhciInterrupter0Regs->erstba);
    drvContext->erdp = xhciRead64(&xhciInterrupter0Regs->erdp);
    drvContext->iman = xhciRead32(&xhciInterrupter0Regs->iman);
    drvContext->imod = xhciRead32(&xhciInterrupter0Regs->imod);
}

/**
 * Non-Blocking functions which triggers xHC to save state
 * @param xhciOperationalRegs
 */
static void saveXHCState(const USBSSP_DriverResourcesT * res, USBSSP_DriverContextT * drvContext) {
    uint32_t regValue = 0U;
    USBSSP_OperationalT *xhciOperationalRegs = res->regs.xhciOperational;

    /* save operational and interrupter registers */
    saveStateRegisters(res, drvContext);

    regValue = xhciRead32(&xhciOperationalRegs->usbcmd);
    regValue |= (uint32_t) USBSSP__USBCMD__CSS_MASK;
    xhciWrite32(&xhciOperationalRegs->usbcmd, regValue);

    /* Wait for SSS (Save State Status) in the USBSTS (USB Status) reg to transition to 0 */
    /* we might not be able to catch the transition. Hence will poll the flag after a delay. */
    CPS_DelayNs(1000);
}

/**
 * Function used for saving state when going to suspend mode
 * @param res driver resources
 * @param drvContext driver context
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t USBSSP_SaveState(USBSSP_DriverResourcesT * res, USBSSP_DriverContextT * drvContext) {
    uint32_t ret = CDN_EOK;

    ret = USBSSP_SaveStateSF(res, drvContext);
    if (CDN_EOK != ret) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
    } else {
        USBSSP_OperationalT *xhciOperationalRegs = res->regs.xhciOperational;
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Saving state for driver instance: <%d>\n", res->instanceNo);

        /* Step 1: stop all endpoints epIndex = 1 to 31 */
        ret = stopActiveEndpoints(res);

        /* Step-2: Ensure that the Command ring is stopped */

        /* Step-3a: Stop Controller. Set Run/Stop = 0 in USBCMD reg */
        if (ret == CDN_EOK) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Stopping Controller\n", 0);
            stopXhc(res);
            /* Step-3b: Wait for HCHalted bit to be set in USBSTS (USB Status register) */
            ret = waitForReg(&xhciOperationalRegs->usbsts,
                             USBSSP__USBSTS__HCH_MASK, USBSSP__USBSTS__HCH_MASK, USBSSP_DEFAULT_TIMEOUT);
        }

        if (ret == CDN_EOK) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "xHC Halted (0x%x) \n", xhciRead32(&xhciOperationalRegs->usbsts));

            /* Step-4: Read the operational and run-time registers and save it in the state struct */
            /* Step-5: Set the CCS (Controller Save State) flag in USBCMD reg */
            saveXHCState(res, drvContext);

            ret = waitForReg(&xhciOperationalRegs->usbsts,
                             USBSSP__USBSTS__SSS_MASK, 0U, USBSSP_DEFAULT_TIMEOUT);
        }

        if (ret == CDN_EOK) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Resetting command ring\n", 0);
            /* At this point all required registers are saved. */
            /* the ownership of all memory "res.xhciResources" is now with software */
            resetCmdRing(res); /* this clears the content of the command ring */
        }
    }
    return ret;
}

/**
 * Functions used for restoring from suspend mode
 * @param res driver resources
 * @param drvContext driver context
 * @return CDN_EOK on success, error code elsewhere
 */
uint32_t USBSSP_RestoreState(USBSSP_DriverResourcesT * res, USBSSP_DriverContextT const * drvContext) {

    uint32_t ret = CDN_EOK;

    ret = USBSSP_RestoreStateSF(res, drvContext);
    if (CDN_EOK != ret) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
    } else {
        USBSSP_OperationalT *xhciOperationalRegs = res->regs.xhciOperational;
        USBSSP_InterrupterT *xhciInterrupter0Regs = &res->regs.xhciInterrupter[0];
        vDbgMsg(USBSSP_DBG_DRV, DBG_FYI, "Restoring state for driver instance: <%d>\n", res->instanceNo);

        /* Step-2: Restore the saved memory image of the DCBAA, Contexts and other data structures */
        /*          to their original physical locations */

        /* Step-3: Restore image of scratchpad if saved */

        /* Step-4: Restore Operational and Runtime registers */
        xhciWrite32(&xhciOperationalRegs->dnctrl, drvContext->dnctrl);
        xhciWrite64(&xhciOperationalRegs->dcbaap, drvContext->dcbaap);
        xhciWrite32(&xhciOperationalRegs->config, drvContext->config);
        xhciWrite32(&xhciInterrupter0Regs->erstsz, drvContext->erstsz);
        xhciWrite64(&xhciInterrupter0Regs->erstba, drvContext->erstba);
        xhciWrite64(&xhciInterrupter0Regs->erdp, drvContext->erdp);
        xhciWrite32(&xhciInterrupter0Regs->iman, drvContext->iman);
        xhciWrite32(&xhciInterrupter0Regs->imod, drvContext->imod);

        /* Step-5a: Set the controller restore state (CRS) flag in the USBCMD to 1 */
        xhciWrite32(&xhciOperationalRegs->usbcmd, USBSSP__USBCMD__CRS_MASK);
        CPS_DelayNs(1000);
        /* Step-5b: Wait for Restore State Status (RSS) in USBSTS to transition to zero. */
        ret = waitForReg(&xhciOperationalRegs->usbsts,
                         USBSSP__USBSTS__RSS_MASK, 0U, USBSSP_DEFAULT_TIMEOUT);
        if (ret != CDN_EOK) {
            vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error: Timed-out waiting for Restore State Status (RSS) to be (%d) \n", 0);
        }
        /* Step 6 & 7: Re-initialize the command ring */
        if (ret == CDN_EOK) {
            uint64_t crcr_sfr_val = get64PhyAddrOf32ptr(&res->commandQ.ring[0].dword0);
            /* Set the Command Ring Dequeue Pointer by programming the Command Ring Control Register */
            xhciWrite64(&xhciOperationalRegs->crcr,
                        (uint64_t) (crcr_sfr_val | res->commandQ.toogleBit));
        }

        /* Step 8: Start the controller */
        initEnableXhc(res);
    }
    return ret;
}

/**
 * Function issues SET_ADDRESS command to controller
 * @param res driver resources
 */
uint32_t USBSSP_SetAddress(USBSSP_DriverResourcesT * res) {

    uint32_t ret = USBSSP_SetAddressSF(res);
    if (ret != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);

    } else {
        setAddress(res, 0U);
    }
    return ret;
}

/**
 * Function returns index of actual micro frame
 * @param res driver resources
 * @param index pointer to memory where actual micro frame index will be stored
 * @return CDN_EOK on success, CDN_EINVAL when any of input parameter is NULL
 */
uint32_t USBSSP_GetMicroFrameIndex(USBSSP_DriverResourcesT * res,
                                   uint32_t *                index) {

    uint32_t ret = CDN_EOK;

    /* check correctness of input parameters */
    if (USBSSP_GetMicroFrameIndexSF(res, index) != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
        ret = CDN_EINVAL;
    } else {
        /* read frame index from hardware */
        *index = xhciRead32(&res->regs.xhciRuntime->mfindex);
        ret = CDN_EOK;
    }

    return ret;
}

/**
 * Function sets frameID value in first TRB of isochronous transfer descriptor
 *
 * @param[in] res driver resources
 * @param[in] epIndex index of selected endpoint
 * @param[in] frameID value of frameID to set
 */
uint32_t USBSSP_SetFrameID(USBSSP_DriverResourcesT *res, uint8_t epIndex, uint32_t frameID) {

    uint32_t ret = USBSSP_SetFrameIDSF(res);
    if (ret != (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);

    } else {
        res->ep[epIndex].frameID = frameID;
    }
    return ret;
}

/**
 * Function sets port override register to given value
 *
 * @param[in] res driver resources
 * @param[in] regValue memory place where port override register value
 *                      will be stored
 * @return CDN_EOK if success, CDN_EINVAL when some input parameter is incorrect
 */
uint32_t USBSSP_SetPortOverrideReg(const USBSSP_DriverResourcesT * res,
                                   uint32_t                        regValue) {
    uint32_t ret = CDN_EOK;

    ret = USBSSP_SetPortOverrideRegSF(res);

    /* check input parameters correctness */
    if (CDN_EOK != ret) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in function parameter\n", 0);
        ret = CDN_EINVAL;
    } else if (res->portOverrideRegs == NULL) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Warning! Port Override register pointer not set.\n", 0);
    } else {
        /* set value of port override register */
        xhciWrite32(res->portOverrideRegs, regValue);
    }
    return ret;
}

/**
 * Function returns port override register value
 *
 * @param[in] res driver resources
 * @param[out] regValue memory place where port status/control register value
 *                      will be stored
 * @return CDN_EOK if success, CDN_EINVAL when some input parameter is incorrect
 */
uint32_t USBSSP_GetPortOverrideReg(const USBSSP_DriverResourcesT * res,
                                   uint32_t *                      regValue) {
    uint32_t ret = CDN_EOK;

    ret = USBSSP_GetPortOverrideRegSF(res, regValue);

    /* check input parameters correctness */
    if (CDN_EOK != ret) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in function parameter\n", 0);
        ret = CDN_EINVAL;
    } else if (res->portOverrideRegs == NULL) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Warning! Port Override register pointer not set.\n", 0);
    } else {
        *regValue = xhciRead32(res->portOverrideRegs);
    }
    return ret;
}

/**
 * Function sets port status/control register to given value
 *
 * @param[in] res driver resources
 * @param[in] portId index of selected port
 * @param[in] portRegIdx port register index
 * @param[in] regValue memory place where port status/control register value
 *                      will be stored
 * @return CDN_EOK if success, CDN_EINVAL when some input parameter is incorrect
 */
uint32_t USBSSP_SetPortControlReg(const USBSSP_DriverResourcesT * res,
                                  uint8_t                         portId,
                                  USBSSP_PortControlRegIdx        portRegIdx,
                                  uint32_t                        regValue) {
    uint32_t ret = CDN_EOK;

    ret = USBSSP_SetPortControlRegSF(res, portRegIdx);

    /* check input parameters correctness */
    if ((CDN_EOK != ret) || (0U == portId)) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
        ret = CDN_EINVAL;
    } else {
        /* set value of port status/control register */
        uint32_t* portCtrlPtr = &(res->regs.xhciPortControl[portId - 1U].portsc);
        xhciWrite32(&portCtrlPtr[portRegIdx], regValue);
    }
    return ret;
}

/**
 * Function returns port control register value
 *
 * @param[in] res driver resources
 * @param[in] portId index of selected port
 * @param[in] portRegIdx port register index
 * @param[out] regValue memory place where port status/control register value
 *                      will be stored
 * @return CDN_EOK if success, CDN_EINVAL when some input parameter is incorrect
 */
uint32_t USBSSP_GetPortControlReg(const USBSSP_DriverResourcesT * res,
                                  uint8_t                         portId,
                                  USBSSP_PortControlRegIdx        portRegIdx,
                                  uint32_t *                      regValue) {
    uint32_t ret = CDN_EOK;

    ret = USBSSP_GetPortControlRegSF(res, portRegIdx, regValue);

    /* check input parameters correctness */
    if ((CDN_EOK != ret) || (0U == portId)) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
        ret = CDN_EINVAL;
    } else {
        uint32_t* portCtrlPtr = &(res->regs.xhciPortControl[portId - 1U].portsc);
        *regValue = xhciRead32(&portCtrlPtr[portRegIdx]);
    }
    return ret;
}

/*----------- tester extension functions------------------------------------ */

/**
 * Function sets selected extra flags associated with selected endpoint.
 *
 * @param[in] res driver resources
 * @param[in] epIndex index of selected endpoint
 * @param[out] flag  bitmap of flags to set, 1 on selected bit clears corresponding flag
 * @return CDN_EOK if success, CDN_EINVAL when some input parameter is incorrect
 */
uint32_t USBSSP_SetEndpointExtraFlag(USBSSP_DriverResourcesT *res, uint8_t epIndex, USBSSP_ExtraFlagsEnumT flags) {

    /* check input parameters correctness */
    uint32_t ret = USBSSP_SetEndpointExtraFlagSF(res, flags);
    if (ret != (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);

    } else {
        /* set selected flags */
        res->ep[epIndex].extraFlags |= (uint8_t) flags;
    }
    return ret;
}

/**
 * Function clears selected extra flags associated with selected endpoint.
 *
 * @param[in] res driver resources
 * @param[in] epIndex index of selected endpoint
 * @param[out] flag  bitmap of flags to clear, 0 on selected bit clears corresponding flag
 * @return CDN_EOK if success, CDN_EINVAL when some input parameter is incorrect
 */
uint32_t USBSSP_CleanEndpointExtraFlag(USBSSP_DriverResourcesT *res, uint8_t epIndex, USBSSP_ExtraFlagsEnumT flags) {

    /* check input parameters correctness */
    uint32_t ret = USBSSP_CleanEndpointExtraFlaSF(res, flags);
    if (ret != (uint32_t) CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);

    } else {
        /* clear selected flags */
        res->ep[epIndex].extraFlags &= ~((uint8_t) flags);
    }
    return ret;
}

/**
 * Function returns extra flags associated with selected endpoint.
 *
 * @param[in] res driver resources
 * @param[in] epIndex index of selected endpoint
 * @param[out] flag pointer to memory where flags will be stored
 * @return CDN_EOK if no errors, CDN_EINVAL for incorrect input parameters
 */
uint32_t USBSSP_GetEndpointExtraFlag(const USBSSP_DriverResourcesT *res,
                                     uint8_t                        epIndex,
                                     uint8_t *                      flag) {

    uint32_t ret;

    /* check if input pointers are not NULL */
    if (USBSSP_GetEndpointExtraFlagSF(res, flag) != CDN_EOK) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
        ret = CDN_EINVAL;
    } else {
        /* set output parameter with extra flag value */
        *flag = res->ep[epIndex].extraFlags;
        ret = CDN_EOK;
    }

    return ret;
}

/**
 * Function returns port connected status
 *
 * @param[in] res driver resources
 * @param[out] connected status of port connection
 * @return CDN_EOK if success, CDN_EINVAL when some input parameter is incorrect
 */
uint32_t USBSSP_GetPortConnected(const USBSSP_DriverResourcesT * res,
                                 uint8_t *                       connected) {
    uint32_t ret = CDN_EOK;

    if ((res == NULL) || (connected == NULL)) {
        ret = CDN_EINVAL;
    }

    /* check input parameters correctness */
    if (CDN_EOK != ret) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
        ret = CDN_EINVAL;
    } else {
        *connected = res->connected;
    }
    return ret;
}

/**
 * Function returns dev address state
 *
 * @param[in] res driver resources
 * @param[out] addrState dev address state
 * @return CDN_EOK if success, CDN_EINVAL when some input parameter is incorrect
 */
uint32_t USBSSP_GetDevAddressState(const USBSSP_DriverResourcesT * res,
                                   uint8_t *                       addrState) {
    uint32_t ret = CDN_EOK;

    if ((res == NULL) || (addrState == NULL)) {
        ret = CDN_EINVAL;
    }

    /* check input parameters correctness */
    if (CDN_EOK != ret) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Critical error! Wrong value in one of function parameters\n", 0);
        ret = CDN_EINVAL;
    } else {
        *addrState = res->devAddress;
    }
    return ret;
}

/*-------------------------------------------------------------------------- */


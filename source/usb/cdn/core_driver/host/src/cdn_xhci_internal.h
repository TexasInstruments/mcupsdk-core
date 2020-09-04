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

#ifndef CDN_XHCI_INTERNAL_H
#define CDN_XHCI_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif


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

/*--------------------------------------------------------------------------- */
/*   STATIC INLINE FUNCTIONS */
/*--------------------------------------------------------------------------- */

/**
 * Constructs a uint16_t value from uint8_t pointer
 *
 * @param dataPtr Pointer to uint8 buffer
 * @return uint16_t value constructed from uint8 buffer
 */
static inline uint16_t getU16ValFromU8Ptr(const uint8_t* dataPtr) {

    /* Constructs a uint32_t value from uint8_t pointer */
#ifdef CPU_BIG_ENDIAN
    uint16_t value = (uint16_t) dataPtr[1];
    uint16_t byte0 = (uint16_t) dataPtr[0] << 8U;

    value += byte0;
    return value;
#else
    uint16_t value = (uint16_t) dataPtr[0];
    uint16_t byte1 = (uint16_t) dataPtr[1] << 8U;

    value += byte1;
    return value;
#endif
}

/**
 * Wrapper for 64bit read register function
 * @param address pointer to 64 bit register
 * @return 64bit value read from given address
 */
inline static uint64_t xhciRead64(volatile uint64_t *address) {

    return le64ToCpu(CPS_UncachedRead64(address));
}

/**
 * Wrapper for 64bit write register function
 * @param address pointer to 64 bit register
 * @param value 64bit value to write to given address
 */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
inline static void xhciWrite64(volatile uint64_t *address, uint64_t value) {

    CPS_UncachedWrite64(address, cpuToLe64(value));
}
/* parasoft-end-suppress METRICS-36-3 */

/**
 * Wrapper for CPS register access function
 * @param pointer to 32bit register
 * @param value vale to written at given pointer
 */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
inline static void xhciWrite32(volatile uint32_t *address, uint32_t value) {

    CPS_REG_WRITE(address, cpuToLe32(value));
}
/* parasoft-end-suppress METRICS-36-3 */

/**
 * Wrapper for CPS register read function
 * @param address pointer to 32 bit register
 * @return 32bit value at given address
 */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
inline static uint32_t xhciRead32(volatile uint32_t *address) {

    return le32ToCpu(CPS_REG_READ(address));
}
/* parasoft-end-suppress METRICS-36-3 */

/* These functions intentionally violate MISRA C rules, to allow pointer
 * casts and/or manipulations required for driver operation. */

/**
 * Function sets 64 value at address given in addr parameter
 * @param addrL pointer to uint32_t word low
 * @param addrH pointer to uint32_t word high
 * @param value 64-bit dword value to write
 */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
inline static void set64Value(uint32_t *addrL, uint32_t *addrH, uint64_t value) {
    *addrL = (uint32_t) (value & UINT32_MAX);
    *addrH = (uint32_t) ((value >> 32) & UINT32_MAX);
}
/* parasoft-end-suppress METRICS-36-3 */

/**
 * Function returns 64bit integer value of C pointer to 32bit
 * @param ptr pointer to 8 bit type
 * @return 64bit integer of C pointer to 8 bit
 */
/* parasoft-begin-suppress MISRA2012-RULE-11_4 "const uint8_t* converted to unsigned long, DRV-4283" */
inline static uint64_t get64PhyAddrOf8ptr(uint8_t const *ptr) {
    return ((uint64_t) (uintptr_t) ptr);
}
/* parasoft-end-suppress MISRA2012-RULE-11_4 */

/**
 * Function returns 64bit integer value of C pointer to 32bit
 * @param ptr pointer to 32bit type
 * @return 64bit integer of C pointer to 32bit
 */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
/* parasoft-begin-suppress MISRA2012-RULE-11_4 "const uint32_t* converted to unsigned long, DRV-4284" */
inline static uint64_t get64PhyAddrOf32ptr(uint32_t const *ptr) {
    return ((uint64_t) (uintptr_t) ptr);
}
/* parasoft-end-suppress MISRA2012-RULE-11_4 */
/* parasoft-end-suppress METRICS-36-3 */

/**
 * Function writes DRBL register
 * @param res driver resources
 * @param slotID - TODO obsolete should be removed, slotID is a internal field of SSP resources
 * @param dbValue - DRBL register value
 */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
inline static void USBSSP_WriteDoorbell(USBSSP_DriverResourcesT *res, uint8_t slotID, uint32_t dbValue) {

    CPS_MemoryBarrier();

    xhciWrite32(&res->regs.xhciDoorbell[slotID], dbValue);
}
/* parasoft-end-suppress METRICS-36-3 */

/**
 * Wrapper for DRBL register
 * @param res driver resources
 */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
inline static void hostCmdDoorbell(USBSSP_DriverResourcesT* res) {
    USBSSP_WriteDoorbell(res, 0, 0);
}
/* parasoft-end-suppress METRICS-36-3 */

/*--------------------------------------------------------------------------- */
/*   XHCI INTERNAL FUNCTION DECLARATION */
/*--------------------------------------------------------------------------- */

/**
 * Function updates res->devConfigFlag
 * @param res driver resource
 * @param newCfgFlag flag to be set as actual
 */
void setDevConfigFlag(USBSSP_DriverResourcesT *res, uint8_t newCfgFlag);

/**
 * Function gets endpoint status
 * @param res driver resources
 * @param epIndex endpoint Index
 */
USBSSP_EpContexEpState getEndpointStatus(USBSSP_DriverResourcesT const *res, uint32_t epIndex);

/**
 * Function updates res->actualSpeed value
 * @param res driver resources
 */
void setSpeed(USBSSP_DriverResourcesT *res);

/**
 * Get the current speed. Should be called only if port connected
 * @param res driver resources
 * @return Current speed
 */
CH9_UsbSpeed getSpeed(const USBSSP_DriverResourcesT *res);

/**
 * Auxiliary function, returns actual speed field from endpoint context, note
 * that software speed enum values may differ from speed values coded in XHCI spec.
 * @param speed actual speed kept in res->actualSpeed
 * @return speed value of speed given in values as XHCI specification states in
 *                endpoint context structure
 */
uint32_t getSlotSpeed(CH9_UsbSpeed speed);

/**
 * Set address. Function executes set address request on connected device
 * @param[in] res driver resources
 */
void setAddress(USBSSP_DriverResourcesT *res, uint8_t bsrVal);

/**
 * Update enqueue pointer
 * @param[in] queue: pointer to producer queue
 * @param linkTrbChainFlag Chain flag if using Link TRB
 */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
void updateQueuePtr(USBSSP_ProducerQueueT *queue, uint32_t linkTrbChainFlag);
/* parasoft-end-suppress METRICS-36-3 */

/**
 * No Operation test. Function used for testing purposes
 * @param[in] res driver resources, internal function
 */
void noOpTest(USBSSP_DriverResourcesT *res);

/**
 *
 * @param setupData Pointer to the uint8_t * buffer having setup data
 * @param setup pointer to the output struct CH9_UsbSetup
 */
void constructCH9setup(const uint8_t *setupData, CH9_UsbSetup* ch9setup);
/**
 * Enqueues non-blocking control transfer request
 * @param[in] res driver resources
 * @param[in] setup keeps setup packet
 * @param[in] pdata pointer for data to send/receive
 */
void enqueueNBControlTransfer(USBSSP_DriverResourcesT *res, const CH9_UsbSetup* setup, const uint8_t *pdata);

/**
 * This function is called in ISR context to process the final get description control
 * transfer completion
 * @param res Pointer to driver private data
 * @param status Status of the transfer
 * @param eventPtr Pointer to the completed event
 */
void xhciGetDescXferComplete(USBSSP_DriverResourcesT *res, uint32_t status, const USBSSP_RingElementT * eventPtr);

/**
 * This function is called in ISR context to process the final get description control
 * transfer completion
 * @param res Pointer to driver private data
 * @param status Status of the transfer
 * @param eventPtr Pointer to the completed event
 */
void xhciGetShortCfgDescComplete(USBSSP_DriverResourcesT *res, uint32_t status, const USBSSP_RingElementT * eventPtr);

/**
 * Sets endpoint feature in host mode
 * @param res Pointer to driver private data
 * @param epIndex endpoint index
 * @param feature
 * @return
 */
void xhciEpSetFeatureHost(USBSSP_DriverResourcesT *res, uint8_t epIndex, uint8_t feature);


#ifdef __cplusplus
}
#endif

#endif

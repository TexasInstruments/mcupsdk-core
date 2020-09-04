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
 * trb.h
 * USB Host/Device controller driver,
 *
 * Functions, macros, definitions for TRB manipulating - header file
 *****************************************************************************/

#ifndef CDN_TRB_H
#define CDN_TRB_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "cdn_xhci_if.h"
#include "cdn_xhci_structs_if.h"
#include "byteorder.h"

/*
 * Get the trb type
 */
static inline uint32_t getTrbType(USBSSP_RingElementT const *trb) {
    return ((le32ToCpu(trb->dword3) >> USBSSP_TRB_TYPE_POS) & 0x3FU);
}

/*
 * Get the toggle bit
 */
static inline uint8_t getToogleBit(USBSSP_RingElementT const *trb) {
    return ((uint8_t) ((le32ToCpu(trb->dword3) >> 0) & 1U));
}

/*
 * Get the ED bit
 */
static inline uint8_t getEDbit(USBSSP_RingElementT const *trb) {
    return ((uint8_t) ((le32ToCpu(trb->dword3) >> 2) & 1U));
}

/*
 * Get the endpoint
 */
static inline uint8_t getEndpoint(USBSSP_RingElementT const *trb) {
    return ((uint8_t) ((le32ToCpu(trb->dword3) >> USBSSP_ENDPOINT_POS) & 0x1FU));
}

/*
 * Get the slot ID
 */
static inline uint8_t getSlotId(USBSSP_RingElementT const *trb) {
    return ((uint8_t) ((le32ToCpu(trb->dword3) >> USBSSP_SLOT_ID_POS) & 0xFFU));
}

/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
/*
 * Get the completion code position
 */
static inline uint32_t getCompletionCode(USBSSP_RingElementT const *trb) {
    return (((le32ToCpu(trb->dword2) >> USBSSP_COMPLETION_CODE_POS) & 0xFFU));
}
/* parasoft-end-suppress METRICS-36-3 */

/*
 * Get the trb EVENT transfer length
 */
static inline uint32_t getTrEvtTrbTransLen(USBSSP_RingElementT const *trb) {
    return (le32ToCpu(trb->dword2) & USBSSP_TRB_EVT_RESIDL_LEN_MSK);
}

/*
 * Get the trb transfer length
 */
static inline uint32_t getTrTrbTransLen(USBSSP_RingElementT const *trb) {
    return (le32ToCpu(trb->dword2) & USBSSP_TRB_TRANSFER_LENGTH_MASK);
}

/*
 * Get the chain
 */
static inline uint8_t getTrTrbChain(USBSSP_RingElementT const *trb) {
    return ((uint8_t) ((le32ToCpu(trb->dword3) >> 4) & 0x01U));
}

/*
 * Get the event trb low pointer
 */
static inline uint32_t getTrEvtTrbPtrLo(USBSSP_RingElementT const *trb) {
    return (le32ToCpu(trb->dword0));
}

/*
 * Get the event trb high pointer
 */
static inline uint32_t getTrEvtTrbPtrHi(USBSSP_RingElementT const *trb) {
    return (le32ToCpu(trb->dword1));
}

/*
 * Get the port ID
 */
static inline uint8_t getPortId(USBSSP_RingElementT const *trb) {
    return ((uint8_t) ((le32ToCpu(trb->dword0) >> 24) & 0xFFU));
}

/*
 * Get the request type
 */
static inline uint8_t getSetupBmRequestType(USBSSP_RingElementT const *trb) {
    return ((uint8_t) ((le32ToCpu(trb->dword0) >> 0) & 0xFFU));
}

/*
 * Get the setup request
 */
static inline uint8_t getSetupBrequest(USBSSP_RingElementT const *trb) {
    return ((uint8_t) ((le32ToCpu(trb->dword0) >> 8) & 0xFFU));
}

/*
 * Get the value
 */
static inline uint16_t getwValue(USBSSP_RingElementT const *trb) {
    return ((uint16_t) ((le32ToCpu(trb->dword0) >> 16) & 0xFFFFU));
}

/*
 * Get the length
 */
static inline uint16_t getwLength(USBSSP_RingElementT const *trb) {
    return ((uint16_t) ((le32ToCpu(trb->dword1) >> 16) & 0xFFFFU));
}

/*
 * Get the Index
 */
static inline uint16_t getwIndex(USBSSP_RingElementT const *trb) {
    return ((uint16_t) ((le32ToCpu(trb->dword1) >> 0) & 0xFFFFU));
}

/*
 * Get SID
 */
static inline uint16_t getSID(USBSSP_RingElementT const *trb) {
    return ((uint16_t) ((le32ToCpu(trb->dword2) >> 0) & 0xFFFFU));
}

/*
 * Get setup ID
 */
static inline uint8_t getSetupId(USBSSP_RingElementT const *trb) {
    return ((uint8_t) ((le32ToCpu(trb->dword3) & USBSSP_TRB_SETUPID_MASK) >> USBSSP_TRB_SETUPID_POS));
}


#ifdef __cplusplus
}
#endif

#endif

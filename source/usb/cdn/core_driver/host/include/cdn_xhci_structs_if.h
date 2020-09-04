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
* XHCI driver for both host and device mode header file
**********************************************************************/
#ifndef CDN_XHCI_STRUCTS_IF_H
#define CDN_XHCI_STRUCTS_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "cdn_stdtypes.h"
#include "cdn_xhci_if.h"

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
/** Transfer request block structure, spec 4.11.1 */
struct USBSSP_RingElementT_s
{
    /** TRB parameter MSB */
    uint32_t dword0;
    /** TRB paramter LSB */
    uint32_t dword1;
    /** TRB status */
    uint32_t dword2;
    /** TRB control */
    uint32_t dword3;
} __attribute__((packed));

/** Input context structure */
struct USBSSP_InputContexT_s
{
    /** Input control context structure */
    uint32_t inputControlContext[USBSSP_CONTEXT_WIDTH];
    /** Slot context structure */
    uint32_t slot[USBSSP_CONTEXT_WIDTH];
    /** Endpoint 0 context structure */
    uint32_t ep0Context[USBSSP_CONTEXT_WIDTH];
    /** Endpoints context structures */
    uint32_t epContext[USBSSP_MAX_EP_CONTEXT_NUM][USBSSP_CONTEXT_WIDTH];
} __attribute__((packed));

/** Output context structure */
struct USBSSP_OutputContexT_s
{
    /** Slot context structure */
    uint32_t slot[USBSSP_CONTEXT_WIDTH];
    /** Endpoint 0 context structure */
    uint32_t ep0Context[USBSSP_CONTEXT_WIDTH];
    /** Endpoints context structures */
    uint32_t epContext[USBSSP_MAX_EP_CONTEXT_NUM][USBSSP_CONTEXT_WIDTH];
} __attribute__((packed));

/** Transfer descriptor structure */
struct USBSSP_XferDescT_s
{
    /** Pointer to the complete callback */
    USBSSP_Complete complete;
    /** Phy address of the first TRB */
    uintptr_t startTRBPhyAddr;
    /** Phy address for next empty TRB */
    uintptr_t nextTRBPhyAddr;
};

/**
 * Structure describes element of producer queue in conjuction to associated endpoint.
 * Note that the same structure is used in command ring - in this case, fields
 * referred to endpoint are not used
 */
struct USBSSP_ProducerQueueT_s
{
    /** Memory buffer for ring used by hardware */
    USBSSP_RingElementT* ring;
    /** Pointer to enqueued element */
    USBSSP_RingElementT* enqueuePtr;
    /** Pointer to dequeued element */
    USBSSP_RingElementT* dequeuePtr;
    /** Pointer to last completed queue element */
    USBSSP_RingElementT* completePtr;
    /** used for testing purposes */
    USBSSP_RingElementT* firstQueuedTRB;
    /** last queued TRB */
    USBSSP_RingElementT* lastQueuedTRB;
    /** frame ID */
    uint32_t frameID;
    /** Streams container */
    USBSSP_ProducerQueueT* stream[USBSSP_STREAM_ARRAY_SIZE];
    /** Points to hardware endpoint context according to spec 6.2.3 */
    uint32_t* hwContext;
    /** Points to this object owner */
    USBSSP_DriverResourcesT* parent;
    /** Transfer descriptor array */
    USBSSP_XferDescT xferDesc[USBSSP_XFER_DESC_QUEUE_SIZE];
    /** Callback function called on TRB complete event */
    USBSSP_Complete complete;
    /** Callback function called on an aggregated transfer completion */
    USBSSP_Complete aggregatedComplete;
    /** Keeps actually selected stream ID */
    uint16_t actualSID;
    /** Keeps actual value of Cycle bit inserted to TRB */
    uint8_t toogleBit;
    /** Keeps context entry value of endpoint associated with this object */
    uint8_t contextIndex;
    /**
     * Auxiliary flag, set to 1 when any TD associated with this object is
     * issued to DMA and flag is set to 0 on complete event
     */
    uint8_t isRunningFlag;
    /** Flag is active when endpoint is in stopped state */
    uint8_t isDisabledFlag;
    /** Flag is indicates that last transfer stalled */
    uint8_t xferStallError;
    /** Keeps copy of endpoint descriptor of associated endpoint */
    uint8_t epDesc[CH9_USB_DS_ENDPOINT + CH9_USB_DS_SS_USB_EP_COMPANION + CH9_USB_DS_SSP_ISO_EP_COMPANION];
    /** Completion code of last transfer */
    uint8_t completionCode;
    /** Interrupter Index of the target interrupter */
    uint8_t interrupterIdx;
    /** extra flags */
    uint8_t extraFlags;
    /** Blocks calling complete callback when set to 1 */
    uint8_t ignoreShortPacket;
    /** Transfer Descriptor read index */
    uint8_t xferDescReadIdx;
    /** Transfer Descriptor write index */
    uint8_t xferDescWriteIdx;
    /** reserved */
    uint8_t reserved_0[2];
};

/**
 * structure keeps all descriptors pointers for USBSSP operating in device mode
 * index of array correspond to USB speed: devDesc[1] - low speed, devDesc[2] - full speed
 * arrays elements indexed 0 should be set to NULL
 */
struct USBSSP_DescT_s
{
    /** Table keeps pointers to device descriptors */
    uint8_t* devDesc[USBSSP_MAX_SPEED];
    /** Table keeps pointers to configuration descriptors */
    uint8_t* confDesc[USBSSP_MAX_SPEED];
    /** Table keeps pointers to BOS descriptors */
    uint8_t* bosDesc[USBSSP_MAX_SPEED];
    /** Table keeps pointers to string descriptors */
    uint8_t* string[USBSSP_MAX_STRING_NUM];
};

/** Device context base array pointer structure */
struct USBSSP_DcbaaT_s
{
    /** Address of scratch pad pointers container */
    uint64_t scratchPadPtr;
    /** Address of device slot 1 */
    uint64_t deviceSlot[USBSSP_MAX_DEVICE_SLOT_NUM];
};

/**
 * Structure represents USB SSP memory required by XHCI specification.
 * That structure has to be provided by application and must be aligned to
 * USBSSP_PAGE_SIZE.
 */
struct USBSSP_XhciResourcesT_s
{
    /** EP transfer ring memory pointer */
    USBSSP_RingElementT* epRingPool;
    /** Event Ring */
    USBSSP_RingElementT* eventPool[USBSSP_MAX_NUM_INTERRUPTERS];
    /** Device context base array structure */
    USBSSP_DcbaaT* dcbaa;
    /** Input context structure */
    USBSSP_InputContexT* inputContext;
    /** Output context structure */
    USBSSP_OutputContexT* outputContext;
    /** Scratchpad buffers (extra element for last pointer = NULL) */
    uint64_t* scratchpad;
    /** event ring segment entry */
    uint64_t* eventRingSegmentEntry;
    /** allocated memory for stream objects */
    USBSSP_ProducerQueueT (*streamMemoryPool)[USBSSP_MAX_EP_NUM_STRM_EN][USBSSP_STREAM_ARRAY_SIZE];
    /** allocation memory for stream's rings */
    USBSSP_RingElementT (*streamRing)[USBSSP_MAX_EP_NUM_STRM_EN][USBSSP_STREAM_ARRAY_SIZE][USBSSP_PRODUCER_QUEUE_SIZE];
    /** Scratchpad buffers pool */
    uint8_t* scratchpadPool;
    /** Pointer to EP0 buffer of size USBSSP_EP0_DATA_BUFF_SIZE */
    uint8_t* ep0Buffer;
} __attribute__((packed));

/** Structure that holds addresses to RID and DID registers. */
struct USBSSP_IpIdVerRegs_s
{
    /** Pointer to DID register */
    uintptr_t didRegPtr;
    /** Pointer to RID register */
    uintptr_t ridRegPtr;
};

/**
 * Structure represents USB SSP controller resources.
 * That structure must be aligned to USBSSP_PAGE_SIZE because of xhciResources.
 */
struct USBSSP_DriverResourcesT_s
{
    /** Structure represents USB SSP memory required by XHCI specification. */
    USBSSP_XhciResourcesT* xhciMemRes;
    /** Input context structure */
    USBSSP_InputContexT* inputContext;
    /** Input context copy structure */
    USBSSP_InputContexT inputContextCopy;
    /** Pointers to actual event ring element for all interrupters */
    USBSSP_RingElementT* eventPtrBuffer[USBSSP_MAX_NUM_INTERRUPTERS];
    /** Pointer to actual event ring element */
    USBSSP_RingElementT* eventPtr;
    /** Command queue object */
    USBSSP_ProducerQueueT commandQ;
    /** Default endpoint queue object */
    USBSSP_ProducerQueueT ep0;
    /** Container of non default endpoint objects */
    USBSSP_ProducerQueueT ep[USBSSP_MAX_EP_CONTEXT_NUM + USBSSP_EP_CONT_OFFSET];
    /** Keeps actual speed the port operate in */
    CH9_UsbSpeed actualSpeed;
    /** Internal buffer of size USBSSP_EP0_DATA_BUFF_SIZE, for control transfer */
    uint8_t* ep0Buff;
    /** Pointer to device descriptors container */
    USBSSP_DescT* devDesc;
    /** Stores actual value of context Entry */
    uint32_t contextEntries;
    /** Local copy of some registers - for quick access */
    USBSSP_QuickAccessRegs qaRegs;
    /** Keeps addresses of all SFR's */
    USBSSP_SfrT regs;
    /** NOP complete callback function, diagnostic function */
    USBSSP_NopComplete nopComplete;
    /** NOP complete callback function, diagnostic function */
    USBSSP_NopComplete forceHeaderComplete;
    /**
     * Mask with enabled endpoints for last SET_CONFIGURATION request
     * \remarks Follows InputControlContext.DWORD1
     */
    uint32_t enabledEndpsMask;
    /** The maximum number of Device Context Structures and Doorbell Array entries this controller can support */
    uint32_t maxDeviceSlot;
    /** flag active when setup packet received and enable_slot command isn't completed yet, it delegates setup handling to enable slot command completion */
    USBSSP_Ep0StateEnum ep0State;
    /** Pointers to IP RID and DID registers. */
    USBSSP_IpIdVerRegs ipIdVerRegs;
    /** Pointer to port override register */
    uint32_t* portOverrideRegs;
    /** Used for testing purposes */
    USBSSP_GenericCallback genericCallback;
    /** Used for testing purposes */
    USBSSP_PostCallback postCallback;
    /** Used for testing purposes */
    USBSSP_PreportChangeDetectCallback preportChangeDetectCallback;
    /** Used for testing purposes */
    USBSSP_InputContextCallback inputContextCallback;
    /** Callback function to perform USB 2.0 PHY soft reset */
    USBSSP_USB2PhySoftReset usb2PhySoftReset;
    /** Reset USB-2 interface */
    USBSSP_USB2ResetCallback usb2ResetCallback;
    /** Keeps actual value of cycle bit for event ring */
    uint8_t eventToogleBit[USBSSP_MAX_NUM_INTERRUPTERS];
    /** InterrupterIdx corresponding to each endpoint */
    uint8_t epInterrupterIdx[USBSSP_MAX_EP_CONTEXT_NUM];
    /** Keeps port ID */
    uint8_t actualPort;
    /** Keeps actual device slot, when USBSSP works in device mode it is 1 */
    uint8_t actualdeviceSlot;
    /** Indicates whether ENABLE_SLOT_COMMAND was sent but slot ID has not been set yet */
    uint8_t enableSlotPending;
    /** Flag is active when USB SSP is in configured state */
    uint8_t devConfigFlag;
    /** 1 - USB SSP works in device mode, 0 - host mode */
    uint8_t deviceModeFlag;
    /** Indicates USB mode (2 - forced USB2 mode, 3 - forced USB3 mode, others - default) */
    uint8_t usbModeFlag;
    /** When set to 1 indicates initializing will be performed only on connected device, not on SSP controller itself */
    uint8_t noControllerSetup;
    /** Extended TBC enable mode if ETC enabled (0: ETE always disabled, 1:ETE enabled for all speeds) */
    uint8_t extendedTBCMode;
    /** Reflects actual state of Current Connect Status of PORTSC register, 0 - disconnected, 1 - connected */
    uint8_t connected;
    /**
     * Keep device address sent during SET_ADDRESS setup request - is active
     * only in device mode
     */
    uint8_t devAddress;
    /** Keeps setupId value of actually handled setup packet */
    uint8_t setupID;
    /** Keeps index of non zero endpoint which generated latest interrupt */
    uint8_t lastEpIntIndex;
    /** Endpoint index corresponding to the last set/clear feature request */
    uint8_t controlXferEpIndex;
    /** Keeps index of instance associated with this resource */
    uint8_t instanceNo;
};

/** This Structure holds the saved USB context, which can be used for restore. */
struct USBSSP_DriverContextT_s
{
    /** USB Command Register */
    uint32_t usbcmd;
    /** Device Notification Control Register */
    uint32_t dnctrl;
    /** Device Context Base Address Array Pointer Register */
    uint64_t dcbaap;
    /** Configure Register */
    uint32_t config;
    /** Interrupter Management */
    uint32_t iman;
    /** Interrupter Moderation */
    uint32_t imod;
    /** Event Ring Segment Table Size */
    uint32_t erstsz;
    /** Event Ring Segment Table Base Address */
    uint64_t erstba;
    /** Event Ring Dequeue Pointer */
    uint64_t erdp;
};

/** Force header params structure */
struct USBSSP_ForceHdrParams_s
{
    /** Header info low and type */
    uint32_t dword0;
    /** Header info mid */
    uint32_t dword1;
    /** Header info high */
    uint32_t dword2;
    /** Root hub port number and TRB type */
    uint32_t dword3;
} __attribute__((packed));

/** Transfer buffer descriptor structure */
struct USBSSP_XferBufferDesc_s
{
    /** buffer vector */
    uintptr_t buffVec;
    /** size vector */
    uint32_t sizeVec;
} __attribute__((packed));

/**
 *  @}
 */


#ifdef __cplusplus
}
#endif

#endif  /* CDN_XHCI_STRUCTS_IF_H */

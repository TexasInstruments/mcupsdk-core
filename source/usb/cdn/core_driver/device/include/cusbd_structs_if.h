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
* Layer interface for the Cadence USB device controller family
**********************************************************************/
#ifndef CUSBD_STRUCTS_IF_H
#define CUSBD_STRUCTS_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "cdn_stdtypes.h"
#include "cusbd_if.h"

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
/** Configuration of Auxiliary-overflow buffers for EP-OUT endpoints. This structure is software related. */
struct CUSBD_EpAuxBufferConfig_s
{
    /** Virtual address of auxiliary buffer for this endpoint */
    uint8_t* bufferAddr;
    /** Size of the auxiliary buffer in 32-bit-words */
    uint16_t bufferSize;
};

/** Auxiliary-overflow buffer structure for EP-OUT endpoints. */
struct CUSBD_EpAuxBuffer_s
{
    /** Virtual address of auxiliary buffer for this endpoint */
    uint8_t* bufferAddr;
    /** Size of the auxiliary buffer in 32-bit-words */
    uint16_t bufferSize;
    /** Read index */
    uint16_t readIdx;
    /** write index */
    uint16_t writeIdx;
    /** update index */
    uint16_t updateIdx;
    /** Max packet size for this endpoint */
    uint16_t maxPacketSize;
};

/**
 * Configuration structure for single endpoint.
 * Single endpoint configuration structure object is a part of CUSBD_Config
 * object used for configuration of whole USB device
 */
struct CUSBD_EpConfig_s
{
    /**
     * Size of hardware (on-chip) memory buffer that is assigned to
     * an endpoint for queuing USB packets. Setting bufferingValue to
     * 0 means that endpoint is disabled. Setting bufferingValue to
     * value larger then 0 will assign hardware (on-chip) buffering
     * memory to an endpoint. Please refer SuperSpeed USB 3.0 Device
     * Controller IP Design Specification section 2.9.6 for details
     * about endpoint buffering.
     */
    uint8_t bufferingValue;
    /** Bulk endpoint support streams, used only in Super Speed mode */
    uint8_t supportStream;
};

/**
 * Configuration of device.
 * Object of this type is used for probe and init functions when checking
 * memory requirements of device object and for hardware controller configuration.
 * Size of device object in bytes is returned on probe function call in
 * CUSBD_SysReq.privDataSize field. Remember that privDataSize returns size
 * of device object only - it doesn't endpoints vector object size.
 * It means that whole memory requirements = device size + (number of endpoints
 * x single endpoint size)
 */
struct CUSBD_Config_s
{
    /** base address of device controller registers */
    uintptr_t regBase;
    /** table of endpoint configuration objects for IN direction */
    CUSBD_EpConfig epIN[CUSBD_NUM_EP_IN];
    /** table of endpoint configuration objects for OUT direction */
    CUSBD_EpConfig epOUT[CUSBD_NUM_EP_OUT];
    /** array of endpoint aux buffer configuration objects for OUT direction */
    CUSBD_EpAuxBufferConfig epOutAuxBufferCfg[CUSBD_NUM_EP_OUT];
    /** Extended TBC enable mode if ETC enabled (0: ETE always disabled, 1:ETE enabled for all speeds) */
    uint8_t extendedTBCMode;
    /**
     * hardware feature enabling DMA operation in DMULT mode 1 - enabled, 0 disabled
     * For Device V1: A non-zero value of dmultEnabled enables DMULT mode for all channels
     * For Device V3:  Bits[15:0]: Control DMULT enable for EP-Out[15:0]
     *                 Bits[31:16]: Control DMULT enable for EP-In[15:0]
     */
    uint32_t dmultEnabled;
    /** DMA interface width, available values: DMA_32_WIDTH, DMA_64_WIDTH */
    CUSBD_DMAInterfaceWidth dmaInterfaceWidth;
    /**
     * Set configurable burst length in DMA, available values:
     * PRECISE_BURST_0, PRECISE_BURST_1, PRECISE_BURST_2, PRECISE_BURST_4,
     * PRECISE_BURST_8, PRECISE_BURST_16, PRECISE_BURST_32, PRECISE_BURST_64,
     * PRECISE_BURST_128
     */
    uint32_t preciseBurstLength;
    /** Keeps addresses of resources of all endpoints */
    CUSBDMA_MemResources (*epMemRes)[32];
    /** Forced USB mode (0 - none (default), 2 - USB2/HS, 3 - USB3/SS, other - reserved) */
    uint8_t forcedUsbMode;
    /** Pointer to DID register */
    uintptr_t didRegPtr;
    /** Pointer to RID register */
    uintptr_t ridRegPtr;
    /** Pointer (virtual) to the setup packet */
    CH9_UsbSetup* setupPacket;
    /** Physical address of setup packet for DMA */
    uintptr_t setupPacketDma;
};

/** System requirements returned by probe */
struct CUSBD_SysReq_s
{
    /** size of memory required for driver's private data */
    uint32_t privDataSize;
    /** size of memory required for endpoint object */
    uint32_t epObjSize;
    /**
     * size of memory required for USB Transfer Request Descriptors.
     * This memory will be used by DMA controller,
     * so it should be suitable for DMA operation.
     */
    uint32_t trbMemSize;
};

/**
 * structure contains information about memory slices for scatter/gather
 * transfer
 */
struct CUSBD_SgList_s
{
    /** link to the next element */
    uintptr_t link;
    /** offset within memory page */
    uint32_t offset;
    /** data length of this transfer slice */
    uint32_t length;
    /** physical address of buffer to read/write */
    uintptr_t dmaAddress;
};

/**
 * Transfer request structure.
 * I/O transfers of higher layers are realized with the help of CUSBD_Req
 * objects. CUSBD_Req is associated with endpoints.
 */
struct CUSBD_Req_s
{
    /** Head of the list. */
    LIST_ListHead list;
    /** pointer to the previous request in the linked list */
    CUSBD_Req* prevReq;
    /** pointer to the next request in the linked list */
    CUSBD_Req* nextReq;
    /** buffer with data to transfer - virtual address */
    uint8_t* buf;
    /** This transfer data length */
    uint32_t length;
    /** physical address of buf buffer */
    uintptr_t dma;
    /** stream id - used only in Super Speed mode */
    uint16_t streamId;
    /** Setting this flag to 1 disables generating of transfer completion interrupt */
    uint8_t noInterrupt;
    /**
     * Setting this flag to 1 causes adding zero length data packet at the
     * end of data transfer when data transfer length is a multiple of
     * MaxPacketSize for endpoint
     */
    uint8_t zero;
    /**
     * Setting this flag causes all reading transfers with short packet as
     * erroneous
     */
    uint8_t shortNotOk;
    /** Internal flag used by driver to mark request as pending */
    uint8_t requestPending;
    /** Callback function called on the transfer completion event */
    CUSBD_CbReqComplete complete;
    /**
     * general-use data hook
     * driver doesn't make use of this field. May be dsed in complete
     * callback function
     */
    void* context;
    /** keeps actual status of request */
    uint32_t status;
    /** number of actually processed bytes */
    uint32_t actual;
};

/** Set of functions allowing to operate on endpoints */
struct CUSBD_EpOps_s
{
    /** Function for endpoint enable */
    CUSBD_CbEpEnable epEnable;
    /** Function for endpoint disable */
    CUSBD_CbEpDisable epDisable;
    /** Function for endpoint sethalt */
    CUSBD_CbEpSetHalt epSetHalt;
    /** Function for endpoint setwedge */
    CUSBD_CbEpSetWedge epSetWedge;
    /** Function for endpoint FIFO status */
    CUSBD_CbEpFifoStatus epFifoStatus;
    /** Function for endpoint FIFO flush */
    CUSBD_CbEpFifoFlush epFifoFlush;
    /** Function for reqQueue */
    CUSBD_CbReqQueue reqQueue;
    /** Function for reqDequeue */
    CUSBD_CbReqDequeue reqDequeue;
};

/** Structure represents device USB endpoint */
struct CUSBD_Ep_s
{
    /** enables organization of endpoints in linked list */
    LIST_ListHead epList;
    /** keeps name of endpoint */
    char name[100];
    /** pointer to the private data of this endpoint */
    CUSBD_EpPrivate* epPrivate;
    /**
     * endpoint driver, contains methods for all operation required on
     * endpoint
     */
    const CUSBD_EpOps* ops;
    /** Maximal packet size for endpoint */
    uint16_t maxPacket;
    /** Maximal number of streams used by endpoint, useful only in SS mode */
    uint16_t maxStreams;
    /**
     * packet multiplication, it enables to multiplicate number of packet
     * per service interval, useful in HS and SS mode only for isochronous
     * endpoint, mult value range from 0 to 2 - it multiplicate packets by
     * 1 to 3
     */
    uint8_t mult;
    /**
     * Maximal number of packets in burst transfer, useful only in SS
     * mode. Maxburst value is in range from 0 to 15 giving number of
     * packets 1 to 16. Refers to USB transfer burst.
     */
    uint8_t maxburst;
    /**
     * Endpoint address, the oldest bit refers to endpoint direction.
     * For example:
     * address = 0x82 refers to endpoint 2 IN
     * address = 0x03 refers to endpoint 3 OUT
     */
    uint8_t address;
    /**
     * pointer to user defined endpoint descriptor,
     * CH9_UsbEndpointDescriptor is defined in cusb_ch9_if.h
     */
    const uint8_t* desc;
    /**
     * pointer to user defined endpoint companion descriptor, useful only
     * in SS mode, CH9_UsbSSEndpointCompanionDescriptor is defined in
     * cusb_ch9_if.h
     */
    const uint8_t* compDesc;
};

/** Structure represents USB device */
struct CUSBD_Dev_s
{
    /** allows for manipulation on endpoints organized as linked list */
    LIST_ListHead epList;
    /**
     * pointer to bidirectional default endpoint, this endpoint should
     * be always available after driver start
     */
    CUSBD_Ep* ep0;
    /** speed value in which device actually works */
    CH9_UsbSpeed speed;
    /** maximal speed the device is able to work */
    CH9_UsbSpeed maxSpeed;
    /** actual state in which device actually works */
    CH9_UsbState state;
    /** Flags informs if device is scatter/gather transfer capable */
    uint8_t sgSupported;
    /** Full name of USB device controller */
    char name[100];
};

/**
 * struct containing function pointers for event notification callbacks issued
 * by isr().
 * Each call passes the driver's privateData pointer for instance
 * identification if necessary, and may also pass data related to the event.
 */
struct CUSBD_Callbacks_s
{
    /** callback for disconnect */
    CUSBD_CbDisconnect disconnect;
    /** callback for connect */
    CUSBD_CbConnect connect;
    /** callback for setup */
    CUSBD_CbSetup setup;
    /** callback for suspend */
    CUSBD_CdSuspend suspend;
    /** callback for resume */
    CUSBD_CbResume resume;
    /** callback for businterval */
    CUSBD_CbbusInterval busInterval;
    /** callback for missing descriptor */
    CUSBD_CbDescMissing descMissing;
    /** callback for USB to PHY soft reset */
    CUSBD_CbUSB2PhySoftReset usb2PhySoftReset;
};

struct CUSBD_EpPrivate_s
{
    /** Request list */
    LIST_ListHead reqList;
    /** Endpoint */
    CUSBD_Ep ep;
    /** Flag for pending transfer */
    uint8_t transferPending;
    /** Pointer for request list head */
    CUSBD_Req* reqListHead;
    /** Pointer for actual request */
    CUSBD_Req* actualReq;
    /** Pointer to DMA channel */
    CUSBDMA_DmaChannel* channel;
    /** Endpoint state */
    CUSBD_epState ep_state;
    /** Wedge flag */
    uint8_t wedgeFlag;
};

/** CUSBD private data. */
struct CUSBD_PrivateData_s
{
    /** this 'class must implement' CUSBD class, this class is an extension of CUSBD */
    CUSBD_Dev device;
    /** copy of user configuration */
    CUSBD_Config config;
    /** copy of user callback */
    CUSBD_Callbacks callbacks;
    /**
     * these variables reflect device state
     *
     * keeps U2 value
     */
    uint8_t u2_value;
    /** keeps U1 value */
    uint8_t u1_value;
    /** keeps suspend value */
    uint16_t suspend;
    /** Iso delay */
    uint16_t isoch_delay;
    /** HW version of this device */
    uint32_t deviceVersion;
    /** bidirectional endpoint 0 */
    CUSBD_EpPrivate ep0;
    /** Endpoint 0 state */
    CH9_Ep0StateEnum ep0NextState;
    /** Flag for endpoint 0 data direction */
    uint8_t ep0DataDirFlag;
    /** Endpoint IN container */
    CUSBD_EpPrivate ep_in_container[16U];
    /** Endpoint OUT container */
    CUSBD_EpPrivate ep_out_container[16U];
    /** Endpoint auxilliary buffer for OUT direction */
    CUSBD_EpAuxBuffer epOutAuxBuffer[CUSBD_NUM_EP_OUT];
    /** Virtual address of setup packet */
    CH9_UsbSetup* setupPacket;
    /** Physical address of setup packet */
    uintptr_t setupPacketDma;
    /** status value */
    uint16_t status_value;
    /** Pointer to the controller registers */
    ssReg_t* reg;
    /** Driver object */
    CUSBDMA_OBJ* dmaDrv;
    /** DMA controller */
    CUSBDMA_DmaController dmaController;
    /** DMA Channel for device to host */
    CUSBDMA_DmaChannel* ep0DmaChannelIn;
    /** DMA channel for host to device */
    CUSBDMA_DmaChannel* ep0DmaChannelOut;
    /** Pointer to request object */
    CUSBD_Req* request;
    /** Allocated request object */
    CUSBD_Req reqAlloc;
};

/**
 *  @}
 */


#ifdef __cplusplus
}
#endif

#endif  /* CUSBD_STRUCTS_IF_H */

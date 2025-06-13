/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup DRV_UDMA_TYPE_MODULE UDMA Common Data Types
 *            This is UDMA driver common parameters and API
 *
 *  @{
 */

/**
 *  \file udma_types.h
 *
 *  \brief UDMA Low Level Driver API/interface data types file.
 */

#ifndef UDMA_TYPES_H_
#define UDMA_TYPES_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>

#if (UDMA_SOC_CFG_RA_LCDMA_PRESENT == 1)
#include <drivers/udma/hw_include/csl_lcdma_ringacc.h>
#endif
#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
#include <drivers/udma/hw_include/csl_bcdma.h>
#include <drivers/udma/hw_include/csl_pktdma.h>
#else
#include <drivers/udma/hw_include/csl_udmap.h>
#endif
#include <drivers/udma/hw_include/csl_intaggr.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if (UDMA_SOC_CFG_RING_MON_PRESENT == 1)
/** \brief UDMA ring monitor handle */
typedef struct Udma_RingMonObj *        Udma_RingMonHandle;
#endif

/** \brief UDMA driver handle */
typedef struct Udma_DrvObjectInt       *Udma_DrvHandle;
/** \brief UDMA channel handle */
typedef struct Udma_ChObjectInt        *Udma_ChHandle;
/** \brief UDMA event handle */ 
typedef struct Udma_EventObjectInt     *Udma_EventHandle;
/** \brief UDMA ring handle */
typedef struct Udma_RingObjectInt      *Udma_RingHandle;
/** \brief UDMA flow handle */
typedef struct Udma_FlowObjectInt      *Udma_FlowHandle;

/**
 *  \brief UDMA ring parameters.
 */
typedef struct
{
    void                   *ringMem;
    /**< Pointer to ring memory.
     *   Incase of FQ and CQ rings, this cannot be NULL except for DRU
     *   direct TR mode where the rings are not used.
     *   Incase of TD CQ, this can be NULL when TD response is supressed via
     *   supressTdCqPkt channel parameter.
     *   Note: This is a virtual pointer. */
    uint32_t                ringMemSize;
    /**< Size of the memory in bytes allocated. This is used by the driver
     *   to validate the allocated memory is sufficient or not.
     *
     *   Note: By default this parameter will be set to
     *   #UDMA_RING_SIZE_CHECK_SKIP by #UdmaRingPrms_init API to enable
     *   backward combatibility when this is not set rightly by the caller */
    uint8_t                 mode;
    /**< Ring mode. Refer \ref tisci_msg_rm_ring_cfg_req::mode */
    uint16_t                virtId;
    /**< Ring virt ID. Refer \ref tisci_msg_rm_ring_cfg_req::virtid */
    uint32_t                elemCnt;
    /**< Ring element count.
     *      Set to queue depth of the ring.
     *      Set to 0 for DRU direct TR mode. */
    uint8_t                 elemSize;
    /**< Ring element size.
     *   Refer \ref Udma_RingElemSize for supported values. */
    uint8_t                 orderId;
    /**< Ring bus order ID value to be programmed into the orderid field of
     *   the ring's RING_ORDERID register. */
    uint8_t                 asel;
    /**< Ring ASEL (address select) value to be set into the ASEL field of the ring's
    *    RING_BA_HI register.
    *    Refer \ref Udma_RingAccAselEndpointSoc for supported values.
    *    This field is not supported on some SoCs.
    *    On SoCs that do not support this field the input is quietly ignored.
    *    Note: By default this parameter will be set to
    *    #UDMA_RINGACC_ASEL_ENDPOINT_PHYSADDR by #UdmaRingPrms_init API */
    uint32_t                mappedRingGrp;
    /**< The Mapped ring group to use when channel type is
     *   #UDMA_CH_TYPE_TX_MAPPED or #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   Refer \ref Udma_MappedTxGrpSoc macro for details about mapped TX ring groups
     *   or \ref Udma_MappedRxGrpSoc macro for details about mapped RX ring groups.
     *
     *   For unmapped case, set to #UDMA_MAPPED_GROUP_INVALID
     */
    uint32_t                mappedChNum;
    /**< The assigned mapped channel number when channel type is
     *   #UDMA_CH_TYPE_TX_MAPPED or #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   This is used to allocate the corresponding mapped ring for the particular channel.
     *   RM will derive an intersecting pool based on the rings reserved for the core (in rmcfg)
     *   and the permissible range for the given channel(rings reserved for specific channels)
     *   such that the allocated ring will be from this intersecting pool.
     *
     *   For example, If the rings idx reserved for the core are 10 to 20 and
     *   the rings for the channel are 15 to 25. Then the intersecting pool of ring idx
     *   will be 15 - 20 and rm will allocate from this range.
     */
} Udma_RingPrms;

/**
 *  \brief UDMA channel open parameters.
 */
typedef struct
{
    uint32_t                chNum;
    /**< [IN] UDMAP channel to allocate.
     *
     *   Set to #UDMA_DMA_CH_ANY  if the channel to allocate and open
     *   could be any from the free pool.
     *   Set to the actual DMA channel when specific DMA channel need to be
     *   allocated. This channel number is relative to the channel type
     *   (TX, RX or External). The driver will internally calculate the
     *   respective offset to get the actual UDMAP channel number.
     */
    uint32_t                peerChNum;
    /**< [IN] The peer channel to link the #chNum using PSILCFG.
     *
     *   Incase of PDMA peripherals this represent the PDMA channel to which the
     *   UDMA channel should pair with. Refer \ref Udma_PdmaCh macros.
     *
     *   Incase of other PSIL master peripherals this represent the thread ID
     *   to which the UDMA channel should pair with. Refer \ref Udma_PsilCh macros.
     *
     *   Incase of Block copy channel type (#UDMA_CH_TYPE_TR_BLK_COPY), set
     *   this to #UDMA_DMA_CH_NA, as the corresponding RX channel (same
     *   index as TX channel) is assumed to be paired with and the driver
     *   internally sets this up. The #UdmaChPrms_init API takes care of
     *   this.
     *
     */
    uint32_t                mappedChGrp;
    /**< [IN] The Mapped channel group to use when channel type is
     *   #UDMA_CH_TYPE_TX_MAPPED or #UDMA_CH_TYPE_RX_MAPPED.
     *   Refer \ref Udma_MappedTxGrpSoc macro for details about mapped TX channel groups
     *   or \ref Udma_MappedRxGrpSoc macro for details about mapped RX channel groups.
     *
     *   For other channel type set to #UDMA_MAPPED_GROUP_INVALID
     */
    void                   *appData;
    /**< [IN] Application/caller context pointer passed back in all the channel
     *    callback functions. This could be used by the caller to identify
     *    the channel for which the callback is called.
     *    This can be set to NULL, if not required by caller. */
    Udma_RingPrms           fqRingPrms;
    /**< [IN] Free queue ring params where descriptors are queued */
    Udma_RingPrms           cqRingPrms;
    /**< [IN] Completion queue ring params where descriptors are dequeued
     *   This is not used for AM64x kind of devices, but even if the application
     *   sets this it will be ignored. But its not required to be set.
     */
    Udma_RingPrms           tdCqRingPrms;
    /**< [IN] Teardown completion queue ring params where teardown
     *   response and TR response incase of direct TR mode are received from
     *   UDMA
     *   This is not used for AM64x kind of devices, but even if the application
     *   sets this it will be ignored. But its not required to be set.
     */
} Udma_ChPrms;

/**
 *  \brief UDMA TX channel parameters.
 */
typedef struct
{
    uint8_t                 pauseOnError;
    /**< [IN] Bool: When set (TRUE), pause channel on error */
    uint8_t                 filterEinfo;
    /**< [IN] Bool: When set (TRUE), filter out extended info */
    uint8_t                 filterPsWords;
    /**< [IN] Bool: When set (TRUE), filter out protocl specific words */
    uint8_t                 addrType;
    /**< [IN] Address type for this channel.
     *   Refer \ref tisci_msg_rm_udmap_tx_ch_cfg_req::tx_atype */
    uint8_t                 chanType;
    /**< [IN] Channel type. Refer \ref tisci_msg_rm_udmap_tx_ch_cfg_req::tx_chan_type */
    uint16_t                fetchWordSize;
    /**< [IN] Descriptor/TR Size in 32-bit words */
    uint8_t                 busPriority;
    /**< [IN] 3-bit priority value (0=highest, 7=lowest) */
    uint8_t                 busQos;
    /**< [IN] 3-bit qos value (0=highest, 7=lowest) */
    uint8_t                 busOrderId;
    /**< [IN] 4-bit orderid value */
    uint8_t                 dmaPriority;
    /**< [IN] This field selects which scheduling bin the channel will be
     *   placed in for bandwidth allocation of the Tx DMA units.
     *   Refer \ref tisci_msg_rm_udmap_tx_ch_cfg_req::tx_sched_priority */
    uint8_t                 txCredit;
    /**< [IN] TX credit for external channels */
    uint16_t                fifoDepth;
    /**< [IN] The fifo depth is used to specify how many FIFO data phases
     *   deep the Tx per channel FIFO will be for the channel.
     *   While the maximum depth of the Tx FIFO is set at design time,
     *   the FIFO depth can be artificially reduced in order to control the
     *   maximum latency which can be introduced due to buffering effects.
     *
     *   The maximum FIFO depth suppported depends on the channel type as
     *   given below:
     *   Normal Capacity Channel        - CSL_NAVSS_UDMAP_TX_CHANS_FDEPTH (128 bytes)
     *   High Capacity Channel          - CSL_NAVSS_UDMAP_TX_HC_CHANS_FDEPTH (1024 bytes)
     *   Ultra High Capacity Channel    - CSL_NAVSS_UDMAP_TX_UHC_CHANS_FDEPTH (4096 bytes)
     *
     *   The default init API will set this paramater as per the channel type.
     */
    uint8_t                 burstSize;
    /**< [IN] Specifies the nominal burst size and alignment for data transfers
     *   on this channel.
     *   Refer \ref tisci_msg_rm_udmap_tx_ch_cfg_req::tx_burst_size.
     *   Note1: This parameter should be set less than or equal to the FIFO
     *   depth parameter set for UTC channel i.e.
     *          fifoDepth >= burstSize
     *   Note2: In case of packet mode TX channels, the Tx fifoDepth must be at
     *   least 2 PSI-L data phases (32 bytes) larger than the burst size given
     *   in this field in order to hold the packet info and extended packet info
     *   header which is placed at the front of the data packet in addition
     *   to the payload i.e.
     *          fifoDepth >= (burstSize + 32 bytes)
     *
     *   Below are the supported burst sizes for various channel types
     *   Normal Capacity Channel        - 64 bytes
     *   High Capacity Channel          - 64, 128 or 256 bytes
     *   Ultra High Capacity Channel    - 64, 128 or 256 bytes
     */
    uint8_t                 supressTdCqPkt;
    /**< [IN] Bool: Specifies whether or not the channel should suppress
     *   sending the single data phase teardown packet when teardown is
     *   complete.
     *      FALSE = TD packet is sent
     *      TRUE = Suppress sending TD packet
     */
} Udma_ChTxPrms;

/**
 *  \brief UDMA RX channel parameters.
 */
typedef struct
{
    uint8_t                 pauseOnError;
    /**< [IN] Bool: When set (TRUE), pause channel on error */
    uint8_t                 addrType;
    /**< [IN] Address type for this channel.
     *   Refer \ref tisci_msg_rm_udmap_rx_ch_cfg_req::rx_atype */
    uint8_t                 chanType;
    /**< [IN] Channel type. Refer \ref tisci_msg_rm_udmap_rx_ch_cfg_req::rx_chan_type */
    uint16_t                fetchWordSize;
    /**< [IN] Descriptor/TR Size in 32-bit words */
    uint8_t                 busPriority;
    /**< [IN] 3-bit priority value (0=highest, 7=lowest) */
    uint8_t                 busQos;
    /**< [IN] 3-bit qos value (0=highest, 7=lowest) */
    uint8_t                 busOrderId;
    /**< [IN] 4-bit orderid value */
    uint8_t                 dmaPriority;
    /**< [IN] This field selects which scheduling bin the channel will be
     *   placed in for bandwidth allocation of the Tx DMA units.
     *   Refer \ref tisci_msg_rm_udmap_rx_ch_cfg_req::rx_sched_priority */
    uint16_t                flowIdFwRangeStart;
    /**< [IN] Starting flow ID value for firewall check */
    uint16_t                flowIdFwRangeCnt;
    /**< [IN] Number of valid flow ID's starting from flowIdFwRangeStart
     *   for firewall check */
    uint8_t                flowEInfoPresent;
    /**< [IN] default flow config parameter for EPIB
     *   Refer \ref tisci_msg_rm_udmap_flow_cfg_req::rx_einfo_present */
    uint8_t                flowPsInfoPresent;
    /**< [IN] default flow config parameter for psInfo
     *   Refer \ref tisci_msg_rm_udmap_flow_cfg_req::rx_psinfo_present */
    uint8_t                flowErrorHandling;
    /**< [IN] default flow config parameter for Error Handling
     *   Refer \ref tisci_msg_rm_udmap_flow_cfg_req::rx_error_handling */
    uint8_t                flowSopOffset;
    /**< [IN] default flow config parameter for SOP offset
     *   Refer \ref tisci_msg_rm_udmap_flow_cfg_req::rx_sop_offset */
    uint8_t                 ignoreShortPkts;
    /**< [IN] Bool: This field controls whether or not short packets will be
     *   treated as exceptions (FALSE) or ignored (TRUE) for the channel.
     *   This field is only used when the channel is in split UTC mode. */
    uint8_t                 ignoreLongPkts;
    /**< [IN] Bool: This field controls whether or not long packets will be
     *   treated as exceptions (FALSE) or ignored (TRUE) for the channel.
     *   This field is only used when the channel is in split UTC mode. */
    uint32_t                configDefaultFlow;
    /**< [IN] Bool: This field controls whether or not to program the default
     *   flow.
     *   TRUE - Configures the default flow equal to the RX channel number
     *   FALSE - Doesn't configure the default flow of channel.
     *   The caller can allocate and use other generic flows or get the
     *   default flow handle and configure the flow using #Udma_flowConfig
     *   API at a later point of time */
    uint8_t                 burstSize;
    /**< [IN] Specifies the nominal burst size and alignment for data transfers
     *   on this channel.
     *   Refer \ref tisci_msg_rm_udmap_rx_ch_cfg_req::rx_burst_size.
     *   Note1: This parameter should be set less than or equal to the FIFO
     *   depth parameter set for UTC channel i.e.
     *          fifoDepth >= burstSize
     *   Note2: In case of packet mode TX channels, the Tx fifoDepth must be at
     *   least 2 PSI-L data phases (32 bytes) larger than the burst size given
     *   in this field in order to hold the packet info and extended packet info
     *   header which is placed at the front of the data packet in addition
     *   to the payload i.e.
     *          fifoDepth >= (burstSize + 32 bytes)
     *
     *   Below are the supported burst sizes for various channel types
     *   Normal Capacity Channel        - 64 bytes
     *   High Capacity Channel          - 64, 128 or 256 bytes
     *   Ultra High Capacity Channel    - 64, 128 or 256 bytes
     */
} Udma_ChRxPrms;

/**
 *  \brief UDMA PDMA channel Static TR parameters.
 */
typedef struct
{
    uint32_t                elemSize;
    /**< [IN] Element size. This field specifies how much data is transferred
     *   in each write which is performed by the PDMA.
     *   This is the X static TR parameter of PDMA.
     *
     *   In case of MCAN TX/RX PDMA channel, this is not used and should be
     *   set to 0.
     *
     *   Refer \ref Udma_PdmaElemSize for supported values. */
    uint32_t                elemCnt;
    /**< [IN] Element count. This field specifies how many elements to
     *   transfer each time a trigger is received on the PDMA channel.
     *   This is the Y static TR parameter of PDMA.
     *
     *   In case of MCAN PDMA channel, this represents the buffer size.
     *   In case of MCAN TX, this field specifies how many bytes should be
     *   written to an MCAN TX buffer. This field includes the 8 byte MCAN
     *   header on the initial packet fragment. The PDMA will break up the
     *   source packet into fragments of this buffer size, copying the 8 byte
     *   MCAN header for the initial fragment, and then skipping it for each
     *   additional fragment and thus reusing the header from the first
     *   fragment. A buffer size less than 16 is treated as 16, and a buffer
     *   size greater than 72 is treated as 72.
     *   In case of MCAN RX, this field specifies how many bytes should be
     *   read from an MCAN RX buffer. This field includes the 8 byte MCAN
     *   header on the initial packet fragment. A buffer size less than 16
     *   is treated as 16, and a buffer size greater than 72 is treated as 72.
     */
    uint32_t                fifoCnt;
    /**< [IN] FIFO count. This field specifies how many full FIFO operations
     *   comprise a complete packet. When the count has been reached, the
     *   PDMA will close the packet with an 'EOP' indication. If this parameter
     *   is set to 0, then no packet delineation is supplied by the PDMA and
     *   all framing is controlled via the UDMA TR.
     *
     *   This is the Z static TR parameter of PDMA.
     *   This is NA for TX and should be set to 0.
     *   In case of MCAN RX, this represents the buffer count. This field
     *   specifies how many MCAN RX buffers should be read before closing the
     *   CPPI packet with an 'EOP' indication. When this count is greater
     *   than 1, multiple MCAN RX buffers will be read into a single CPPI
     *   packet buffer. The 8 byte MCAN header will be skipped on subsequent
     *   MCAN buffer reads. Setting this field to NULL will suppress all
     *   packet delineation, and should be avoided.
     */
} Udma_ChPdmaPrms;

/**
 *  \brief UDMA channel statistics.
 */
typedef struct
{
    uint32_t                packetCnt;
    /**< [OUT] Current completed packet count for the channel */
    uint32_t                completedByteCnt;
    /**< [OUT] Current completed payload byte count for the channel */
    uint32_t                startedByteCnt;
    /**< [OUT] Current started byte count for the channel */
} Udma_ChStats;

/**
 *  \brief UDMA event callback function.
 *
 *  \param eventHandle  [IN] UDMA event handle
 *  \param eventType    [IN] Event that occurred
 *  \param appData      [IN] Callback pointer passed during event register
 */
typedef void (*Udma_EventCallback)(Udma_EventHandle eventHandle,
                                   uint32_t eventType,
                                   void *appData);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA event related parameters.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2628), DOX_REQ_TAG(PDK-2627)
 *               DOX_REQ_TAG(PDK-2626), DOX_REQ_TAG(PDK-2625)
 */
typedef struct
{
    uint32_t                eventType;
    /**< [IN] Event type to register. Refer \ref Udma_EventType */
    uint32_t                eventMode;
    /**< [IN] Event mode - exclusive or shared. Refer \ref Udma_EventMode.
     *   This parameter should be set to #UDMA_EVENT_MODE_SHARED for
     *   #UDMA_EVENT_TYPE_MASTER event type. */
    Udma_ChHandle           chHandle;
    /**< [IN] Channel handle when the event type is one of below
     *          - #UDMA_EVENT_TYPE_DMA_COMPLETION
     *          - #UDMA_EVENT_TYPE_TEARDOWN_PACKET
     *          - #UDMA_EVENT_TYPE_TR.
     *   This parameter can be NULL for other types. */
    Udma_RingHandle         ringHandle;
    /**< [IN] Ring handle when the event type is one of below
     *          - #UDMA_EVENT_TYPE_RING
     *   This parameter can be NULL for other types. */
    Udma_EventHandle        controllerEventHandle;
    /**< [IN] Master event handle used to share the IA register when the event
     *   mode is set to #UDMA_EVENT_MODE_SHARED.
     *   This is typically used to share multiple events from same source
     *   like same peripheral to one IA register which eventually routes to
     *   a single core interrupt.
     *   For the first(or master) event this should be set to NULL. The driver
     *   will allocate the required resources (IA/IR) for the first event.
     *   For the subsequent shared event registration, the master event handle
     *   should be passed as reference and the driver will allocate only the
     *   IA status bits. At a maximum #UDMA_MAX_EVENTS_PER_VINTR number of
     *   events can be shared. Beyond that the driver will return error.
     *   This parameter should be set to NULL for #UDMA_EVENT_TYPE_MASTER
     *   event type. */
    Udma_EventCallback      eventCb;
    /**< [IN] When callback function is set (non-NULL), the driver will allocate
     *   core level interrupt through Interrupt Router and the function
     *   will be called when the registered event occurs.
     *   When set to NULL, the API will only allocate event and no interrupt
     *   routing is performed.
     *   Note: In case of shared events (multiple events mapped to same
     *   interrupt), the driver will call the callbacks in the order
     *   of event registration.
     *   This parameter should be set to NULL for #UDMA_EVENT_TYPE_MASTER
     *   event type. */
    uint32_t                intrPriority;
    /**< [IN] Priority of interrupt to register with OSAL. The interpretation
     *   depends on the OSAL implementation */
    void                   *appData;
    /**< [IN] Application/caller context pointer passed back in the event
     *    callback function. This could be used by the caller to identify
     *    the channel/event for which the callback is called.
     *    This can be set to NULL, if not required by caller. */
    uint32_t                preferredCoreIntrNum;
    /**< [IN] Preferred core interrupt number which goes to a core.
     *
     *   If set to #UDMA_CORE_INTR_ANY, will allocate from free pool.
     *   Else will try to allocate the mentioned interrupt itself. */
    #if (UDMA_SOC_CFG_RING_MON_PRESENT == 1)
    Udma_RingMonHandle      monHandle;
    /**< [IN] Ring monitor handle when the event type is one of below
     *          - #UDMA_EVENT_TYPE_RING_MON
     *   This parameter can be NULL for other types. */
    #endif
    /*
     * Output parameters
     */
    volatile uint64_t      *intrStatusReg;
    /**< [OUT] Interrupt status register address of the allocated IA VINT
     *   register. This is used to check if interrupt occurred */
    volatile uint64_t      *intrClearReg;
    /**< [OUT] Interrupt clear register address of the allocated IA VINT
     *   register. This is used to clear if interrupt occurred */
    uint64_t                intrMask;
    /**< [OUT] Interrupt mask to check and clear */
    uint32_t                vintrNum;
    /**< [OUT] IA Virtual interrupt number allocated. */
    uint32_t                coreIntrNum;
    /**< [OUT] Core interrupt number allocated.
     *   This number can be used to register with the OSAL
     *
     *   Note: Incase of C7x, this represents the GIC SPI events to the CLEC.
     *   For routing this event, the driver further uses the Udma_RmInitPrms - 'startC7xCoreIntr'
     *   parameter as the start C7x interrupt and assumes that numIrIntr
     *   C7x interrupt are used by UDMA driver for one to one mapping.
     *   The UDMA driver directly programs the CLEC for this routing
     *
     *   Example: startIrIntr = 700, numIrIntr = 3, startC7xCoreIntr = 32
     *
     *   First Event registration:
     *   CLEC input         : 700+1024-32
     *   CLEC output        : 32
     *   OSAL registration  : 32
     *
     *   Second Event registration:
     *   CLEC input         : 701+1024-32
     *   CLEC output        : 33
     *   OSAL registration  : 33 */
} Udma_EventPrms;

/**
 *  \brief UDMA RX channel flow parameters.
 */
typedef struct
{
    Udma_ChHandle           rxChHandle;
    /**< [IN] Deprecated member. Not used any more. */
    uint8_t                 einfoPresent;
    /**< [IN] Set to 1 if extended packet info is present in the descriptor */
    uint8_t                 psInfoPresent;
    /**< [IN] Set to 1 if protocol-specific info is present in the
    *    descriptor */
    uint8_t                 errorHandling;
    /**< [IN] Determines how starvation errors are handled.
      *  0=drop packet, 1=retry */
    uint8_t                 descType;
    /**< [IN] Descriptor type - see \ref tisci_msg_rm_udmap_flow_cfg_req::rx_desc_type */
    uint8_t                 psLocation;
    /**< [IN] Protocol-specific info location.
     *  \ref TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PS_END_PD
     *  \ref TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PS_BEGIN_DB
     */
    uint16_t                sopOffset;
    /**< [IN] Start of rx packet data (byte offset from the start of
     *   the SOP buffer) */
    uint16_t                defaultRxCQ;
    /**< [IN] Rx destination queue */
    uint8_t                 srcTagHi;
    /**< [IN] UDMAP receive flow source tag high byte constant configuration
     *   to be programmed into the rx_src_tag_hi field of the flow's RFLOW_RFB
     *   register.*/
    uint8_t                 srcTagLo;
    /**< [IN] UDMAP receive flow source tag low byte constant configuration
     *   to be programmed into the rx_src_tag_lo field of the flow's RFLOW_RFB
     *   register.*/
    uint8_t                 srcTagHiSel;
    /**< [IN] UDMAP receive flow source tag high byte selector configuration
     *   to be programmed into the rx_src_tag_hi_sel field of the RFLOW_RFC
     *   register. Refer \ref tisci_msg_rm_udmap_flow_cfg_req::rx_dest_tag_hi_sel. */
    uint8_t                 srcTagLoSel;
    /**< [IN] UDMAP receive flow source tag low byte selector configuration
     *   to be programmed into the rx_src_tag_low_sel field of the RFLOW_RFC
     *   register. Refer \ref tisci_msg_rm_udmap_flow_cfg_req::rx_dest_tag_lo_sel. */
    uint8_t                 destTagHi;
    /**< [IN] UDMAP receive flow destination tag high byte constant configuration
     *   to be programmed into the rx_dest_tag_hi field of the flow's RFLOW_RFB
     *   register.*/
    uint8_t                 destTagLo;
    /**< [IN] UDMAP receive flow destination tag low byte constant configuration
     *   to be programmed into the rx_dest_tag_lo field of the flow's RFLOW_RFB
     *   register.*/
    uint8_t                 destTagHiSel;
    /**< [IN] UDMAP receive flow destination tag high byte selector configuration
     *   to be programmed into the rx_dest_tag_hi_sel field of the RFLOW_RFC
     *   register. Refer \ref tisci_msg_rm_udmap_flow_cfg_req::rx_dest_tag_hi_sel. */
    uint8_t                 destTagLoSel;
    /**< [IN] UDMAP receive flow destination tag low byte selector configuration
     *   to be programmed into the rx_dest_tag_low_sel field of the RFLOW_RFC
     *   register. Refer \ref tisci_msg_rm_udmap_flow_cfg_req::rx_dest_tag_lo_sel. */
    uint8_t                 sizeThreshEn;
    /**< [IN] UDMAP receive flow packet size based free buffer queue enable configuration
     * to be programmed into the rx_size_thresh_en field of the RFLOW_RFC register.
     * See the UDMAP section of the TRM for more information on this setting.
     * Configuration of the optional size thresholds when this configuration is
     * enabled is done by sending the @ref tisci_msg_rm_udmap_flow_size_thresh_cfg_req
     * message to System Firmware for the receive flow allocated by this request.
     * This parameter can be no greater than
     * @ref TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_SIZE_THRESH_MAX */
    uint16_t                fdq0Sz0Qnum;
    /**< [IN] UDMAP receive flow free descriptor queue 0 configuration to be programmed
     * into the rx_fdq0_sz0_qnum field of the flow's RFLOW_RFD register.  See the
     * UDMAP section of the TRM for more information on this setting.  The specified
     * free queue must be valid within the Navigator Subsystem and must be owned
     * by the host, or a subordinate of the host, requesting allocation and
     * configuration of the receive flow. */
    uint16_t                fdq1Qnum;
    /**< [IN] UDMAP receive flow free descriptor queue 1 configuration to be programmed
     * into the rx_fdq1_qnum field of the flow's RFLOW_RFD register.  See the
     * UDMAP section of the TRM for more information on this setting.  The specified
     * free queue must be valid within the Navigator Subsystem and must be owned
     * by the host, or a subordinate of the host, requesting allocation and
     * configuration of the receive flow. */
    uint16_t                fdq2Qnum;
    /**< [IN] UDMAP receive flow free descriptor queue 2 configuration to be programmed
     * into the rx_fdq2_qnum field of the flow's RFLOW_RFE register.  See the
     * UDMAP section of the TRM for more information on this setting.  The specified
     * free queue must be valid within the Navigator Subsystem and must be owned
     * by the host, or a subordinate of the host, requesting allocation and
     * configuration of the receive flow. */
    uint16_t                fdq3Qnum;
    /**< [IN] UDMAP receive flow free descriptor queue 3 configuration to be programmed
     * into the rx_fdq3_qnum field of the flow's RFLOW_RFE register.  See the
     * UDMAP section of the TRM for more information on this setting.  The specified
     * free queue must be valid within the Navigator Subsystem and must be owned
     * by the host, or a subordinate of the host, requesting allocation and
     * configuration of the receive flow. */
    uint16_t                sizeThresh0;
    /**< [IN] UDMAP receive flow packet size threshold 0 configuration to be programmed
     * into the rx_size_thresh0 field of the flow's RFLOW_RFF register.  See the
     * UDMAP section of the TRM for more information on this setting. */
    uint16_t                sizeThresh1;
    /**< [IN] UDMAP receive flow packet size threshold 1 configuration to be programmed
     * into the rx_size_thresh1 field of the flow's RFLOW_RFF register.  See the
     * UDMAP section of the TRM for more information on this setting. */
    uint16_t                sizeThresh2;
    /**< [IN] UDMAP receive flow packet size threshold 2 configuration to be programmed
     * into the rx_size_thresh2 field of the flow's RFLOW_RFG register.  See the
     * UDMAP section of the TRM for more information on this setting. */
    uint16_t                fdq0Sz1Qnum;
    /**< [IN] UDMAP receive flow free descriptor queue for size threshold 1 configuration
     * to be programmed into the rx_fdq0_sz1_qnum field of the flow's RFLOW_RFG
     * register.  See the UDMAP section of the TRM for more information on this
     * setting.  The specified free queue must be valid within the Navigator
     * Subsystem and must be owned by the host, or a subordinate of the host, who
     * owns the receive flow index and who is making the optional configuration
     * request. */
    uint16_t                fdq0Sz2Qnum;
    /**< [IN] UDMAP receive flow free descriptor queue for size threshold 2 configuration
     * to be programmed into the rx_fdq0_sz2_qnum field of the flow's RFLOW_RFH
     * register.  See the UDMAP section of the TRM for more information on this
     * setting.  The specified free queue must be valid within the Navigator
     * Subsystem and must be owned by the host, or a subordinate of the host, who
     * owns the receive flow index and who is making the optional configuration
     * request. */
    uint16_t                fdq0Sz3Qnum;
    /**< [IN] UDMAP receive flow free descriptor queue for size threshold 3 configuration
     * to be programmed into the rx_fdq0_sz3_qnum field of the flow's RFLOW_RFH
     * register.  See the UDMAP section of the TRM for more information on this
     * setting.  The specified free queue must be valid within the Navigator
     * Subsystem and must be owned by the host, or a subordinate of the host, who
     * owns the receive flow index and who is making the optional configuration
     * request. */
} Udma_FlowPrms;

/**
 *  \brief UDMA Virtual to Physical address translation callback function.
 *
 *  This function is used by the driver to convert virtual address to physical
 *  address.
 *
 *  \param virtAddr [IN] Virtual address
 *  \param chNum    [IN] Channel number passed during channel open
 *  \param appData  [IN] Callback pointer passed during channel open
 *
 *  \return Corresponding physical address
 */
typedef uint64_t (*Udma_VirtToPhyFxn)(const void *virtAddr,
                                      uint32_t chNum,
                                      void *appData);
/**
 *  \brief UDMA Physical to Virtual address translation callback function.
 *
 *  This function is used by the driver to convert physical address to virtual
 *  address.
 *
 *  \param phyAddr  [IN] Physical address
 *  \param chNum    [IN] Channel number passed during channel open
 *  \param appData  [IN] Callback pointer passed during channel open
 *
 *  \return Corresponding virtual address
 */
typedef void *(*Udma_PhyToVirtFxn)(uint64_t phyAddr,
                                   uint32_t chNum,
                                   void *appData);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA initialization parameters.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2631)
 */
typedef struct
{
    uint32_t                instId;
    /**< [IN] \ref Udma_InstanceIdSoc */
    uint32_t                skipGlobalEventReg;
    /**< Skips the global event registeration for the handle. By default this
     *   is set to FALSE and application can use this common handle to set the
     *   master event to limit the number of IA/IR registration per core
     *   This can be set to TRUE to skip this registration as in the case
     *   of having multiple handles per core in usecases */
    Udma_VirtToPhyFxn       virtToPhyFxn;
    /**< If not NULL, this function will be called to convert virtual address
     *   to physical address to be provided to UDMA.
     *   If NULL, the driver will assume a one-one mapping.
     */
    Udma_PhyToVirtFxn       phyToVirtFxn;
    /**< If not NULL, this function will be called to convert physical address
     *   to virtual address to access the pointer returned by the UDMA.
     *   If NULL, the driver will assume a one-one mapping.
     *
     *   Note: The init fxn will initialize this to the default one-one map
     *   function #Udma_defaultPhyToVirtFxn
     */
} Udma_InitPrms;


#if defined(DRV_VERSION_UDMA_V0)
/**
 *  \brief UDMA ring object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
typedef struct Udma_RingObjectInt
{
    Udma_DrvHandle           drvHandle;
    /**< Pointer to global driver handle. */

    uint16_t                    ringNum;
    /**< Ring number */

#if (UDMA_SOC_CFG_RA_LCDMA_PRESENT == 1)
    CSL_LcdmaRingaccRingCfg          lcdmaCfg;
    /**< Lcdma Ring config */

    /* Below register overlay pointers provided for debug purpose to
     * readily view the registers */
    volatile CSL_lcdma_ringacc_ring_cfgRegs_RING  *pLcdmaCfgRegs;
    /**< Pointer to Lcdma RA config register overlay */
    volatile CSL_lcdma_ringacc_ringrtRegs_ring *pLcdmaRtRegs;
    /**< Pointer to Lcdma RA RT config register overlay */
#endif

    uint32_t                    ringInitDone;
    /**< Flag to set the ring object is init. */

    uint32_t                    mappedRingGrp;
    /**< The allocated mapped ring group when channel type is
     *   #UDMA_CH_TYPE_TX_MAPPED or #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   This is needed to free the mapped ring.
     *
     *   Refer \ref Udma_MappedTxGrpSoc macro for details about mapped TX ring groups
     *   or \ref Udma_MappedRxGrpSoc macro for details about mapped RX ring groups.
     *
     *   For unmapped case, this will be #UDMA_MAPPED_GROUP_INVALID
     */
    uint32_t                    mappedChNum;
    /**< The assigned mapped channel number when channel type is
     *   #UDMA_CH_TYPE_TX_MAPPED or #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   This is needed to free the mapped ring.
     *
     *   For unmapped case, this will be #UDMA_DMA_CH_INVALID.
     */
} Udma_RingObject;

/**
 *  \brief UDMA flow object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
typedef struct Udma_FlowObjectInt
{
    Udma_DrvHandle       drvHandle;
    /**< Pointer to global driver handle. */

    uint32_t                flowStart;
    /**< Flow ID start number.
     *
     *   Note: In case of mapped flow(in devices like AM64x), this indicates the
     *   mapped flow idx managed by this flow handle.
     *
    */
    uint32_t                flowCnt;
    /**< Number of flow IDs allocated - Contiguos flows are allocated
     *
     *   Note: In case of mapped flow(in devices like AM64x), this will be 1
     *   since only one mapped flow is managed by a flow handle.
    */

    uint32_t                flowInitDone;
    /**< Flag to set the flow object is init. */

    uint32_t                mappedFlowGrp;
    /**< The allocated mapped flow group when channel type is
     *   #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   This is needed to free the mapped flow.
     *
     *   Refer \ref Udma_MappedRxGrpSoc macro for details about mapped RX flow groups.
     *
     *   For unmapped case, this will be #UDMA_MAPPED_GROUP_INVALID
     */
    uint32_t                mappedChNum;
    /**< The assigned mapped channel number when channel type is
     *   #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   This is needed to free the mapped flow.
     *
     *   For unmapped case, this will be #UDMA_DMA_CH_INVALID.
     */
} Udma_FlowObject;

/**
 *  \brief UDMA event object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
typedef struct Udma_EventObjectInt
{
    Udma_DrvHandle       drvHandle;
    /**< Pointer to global driver handle. */
    Udma_EventPrms          eventPrms;
    /**< Event parameters passed during event registeration. */

    uint32_t                globalEvent;
    /**< Allocated IA global event. */
    uint32_t                vintrNum;
    /**< Allocated IA VINT register. */
    uint32_t                vintrBitNum;
    /**< Allocated IA VINT bit number - 0 to 63. */
    uint32_t                irIntrNum;
    /**< Allocated interrupt router number.
     * In case of devices like AM64x, where there are no Interrupt Routers,
     * irIntrNum refers to coreIntrNum number itself. */
    uint32_t                coreIntrNum;
    /**< Allocated core interrupt number. */

    Udma_EventHandle     nextEvent;
    /**< Pointer to next event - used in shared event for traversing in ISR */
    Udma_EventHandle     prevEvent;
    /**< Pointer to previous event - used in shared event for traversing during
     *   event un-registration */

    void                   *hwiHandle;
    /**< HWI handle. */
    HwiP_Object             hwiObject;
    /**< HWI Object. */
    uint64_t                vintrBitAllocFlag;
    /**< For master event, this stores the alloc flag for each bit within
     *   IA register. This is not used for slave events and is always set to
     *   zero */

    /* Below register overlay pointers provided for debug purpose to
     * readily view the registers */
    volatile CSL_intaggr_imapRegs_gevi  *pIaGeviRegs;
    /**< Pointer to IA global event register overlay */
    volatile CSL_intaggr_intrRegs_vint  *pIaVintrRegs;
    /**< Pointer to IA virtual interrupt register overlay */

    uint32_t                eventInitDone;
    /**< Flag to set the event object is init. */
} Udma_EventObject;

/**
 *  \brief UDMA channel object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
typedef struct Udma_ChObjectInt
{
    uint32_t                chType;
    /**< UDMA channel type. Refer \ref Udma_ChType. */
    Udma_ChPrms             chPrms;
    /**< Object to store the channel params. */
    Udma_DrvHandle       drvHandle;
    /**< Pointer to global driver handle. */

    uint32_t                txChNum;
    /**< Allocated TX channel number - this is relative channel number from
     *   base TX channel. This is valid only when the channel is opened for
     *   TX and block copy mode */
    uint32_t                rxChNum;
    /**< Allocated RX channel number - this is relative channel number from
     *   base RX channel. This is valid only when the channel is opened for
     *   RX and block copy mode */
    uint32_t                extChNum;
    /**< Allocated Ext channel number - this is relative channel number from
     *   base External channel. This is valid only when the channel is opened
     *   for UTC mode */
    uint32_t                pdmaChNum;
    /**< Allocated peer PDMA channel number. This is valid only when the
     *   channel is opened for PDMA mode */
    uint32_t                peerThreadId;
    /**< Peer channel thread ID - this is or'ed with thread offset. */

    Udma_RingHandle      fqRing;
    /**< Free queue ring handle */
    Udma_RingHandle      cqRing;
    /**< Completion queue ring handle
    *    For AM64x kind of devices, where there is no seperate Completion queue,
    *    this points to fqRing itself.
    */
    Udma_RingHandle      tdCqRing;
    /**< Teardown completion queue ring handle */

    Udma_RingObject         fqRingObj;
    /**< Free queue ring object */
    Udma_RingObject         cqRingObj;
    /**< Completion queue ring object
    *    Not used for AM64x kind of devices, where there is no seperate Completion queue.
    */
    Udma_RingObject         tdCqRingObj;
    /**< Teardown completion queue ring object
    *    Not used for AM64x kind of devices, where teardown function is not present.
    */

    Udma_FlowHandle      defaultFlow;
    /**< Default flow handle */
    Udma_FlowObject      defaultFlowObj;
    /**< Default flow object - Flow ID equal to the RX channel is reserved
     *   as the default flow for the channel. This object is used for
     *   providing handle to the caller to re-program the default flow using
     *   the standard flow API's */

    Udma_ChTxPrms           txPrms;
    /**< TX channel parameter passed during channel config. */
    Udma_ChRxPrms           rxPrms;
    /**< RX channel parameter passed during channel config. */

#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
    /* Below BCDMA register overlay pointers provided for debug purpose to
     * readily view the registers */
    volatile CSL_bcdma_bccfgRegs_chan   *pBcdmaBcCfgRegs;
    /**< Pointer to BCDMA Block copy config register overlay */
    volatile CSL_bcdma_bcrtRegs_chan    *pBcdmaBcRtRegs;
    /**< Pointer to BCDMA Block copy RT config register overlay */
    volatile CSL_bcdma_txccfgRegs_chan  *pBcdmaTxCfgRegs;
    /**< Pointer to BCDMA TX config register overlay */
    volatile CSL_bcdma_txcrtRegs_chan   *pBcdmaTxRtRegs;
    /**< Pointer to BCDMA TX RT config register overlay */
    volatile CSL_bcdma_rxccfgRegs_chan  *pBcdmaRxCfgRegs;
    /**< Pointer to BCDMA RX config register overlay */
    volatile CSL_bcdma_rxcrtRegs_chan   *pBcdmaRxRtRegs;
    /**< Pointer to BCDMA RX RT config register overlay */

    /* Below PKTDMA register overlay pointers provided for debug purpose to
     * readily view the registers */
    volatile CSL_pktdma_txccfgRegs_chan  *pPktdmaTxCfgRegs;
    /**< Pointer to PKTDMA TX config register overlay */
    volatile CSL_pktdma_txcrtRegs_chan   *pPktdmaTxRtRegs;
    /**< Pointer to PKTDMA TX RT config register overlay */
    volatile CSL_pktdma_rxccfgRegs_chan  *pPktdmaRxCfgRegs;
    /**< Pointer to PKTDMA RX config register overlay */
    volatile CSL_pktdma_rxcrtRegs_chan   *pPktdmaRxRtRegs;
    /**< Pointer to PKTDMA RX RT config register overlay */
    volatile CSL_pktdma_txccfgRegs_chan  *pPktdmaExtCfgRegs;
    /**< Pointer to PKTDMA External config register overlay */
    volatile CSL_pktdma_txcrtRegs_chan   *pPktdmaExtRtRegs;
    /**< Pointer to PKTDMA External RT config register overlay */
#endif

    uint32_t                chInitDone;
    /**< Flag to set the channel object is init. */
    uint32_t                chOesAllocDone;
    /**< Flag to check if the channel's OES is allocated. This is required
     *   because the channel OES is used for chaining as well as for
     *   TR event registeration. This allows to check for error when both
     *   are requested by user on the same channel */
    uint32_t                trigger;
    /**< Channel trigger used when chaining channels - needed at the time of
     *   breaking the chaining */
} Udma_ChObject;

/**
 *  \brief UDMA resource manager init parameters.
 *
 *  This assumes contiguos allocation of 'N' resources from a start offset
 *  to keep the interface simple.
 *
 *  Note: This is applicable for the driver handle as given during init call.
 *  The init call doesn't (can't rather) check for resource overlap across
 *  handles and across cores. It is the callers responsibility to ensure that
 *  resources overlaps are not present.
 */
typedef struct
{
    uint32_t                startBlkCopyUhcCh;
    /**< Start ultra high capacity block copy channel from which this UDMA
     *   driver instance manages */
    uint32_t                numBlkCopyUhcCh;
    /**< Number of ultra high capacity block copy channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_BLK_COPY_UHC_CH */
    uint32_t                startBlkCopyHcCh;
    /**< Start high capacity block copy channel from which this UDMA
     *   driver instance manages */
    uint32_t                numBlkCopyHcCh;
    /**< Number of ultra high capacity block copy channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_BLK_COPY_HC_CH */
    uint32_t                startBlkCopyCh;
    /**< Start Block copy channel from which this UDMA driver instance manages */
    uint32_t                numBlkCopyCh;
    /**< Number of Block copy channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_BLK_COPY_CH */

    uint32_t                startTxUhcCh;
    /**< Start ultra high capacity TX channel from which this UDMA driver
     *   instance manages */
    uint32_t                numTxUhcCh;
    /**< Number of ultra high capacity TX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_TX_UHC_CH */
    uint32_t                startTxHcCh;
    /**< Start high capacity TX channel from which this UDMA driver instance
     *   manages */
    uint32_t                numTxHcCh;
    /**< Number of high capacity TX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_TX_HC_CH */
    uint32_t                startTxCh;
    /**< Start TX channel from which this UDMA driver instance manages */
    uint32_t                numTxCh;
    /**< Number of TX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_TX_CH */

    uint32_t                startRxUhcCh;
    /**< Start ultra high capacity RX channel from which this UDMA driver
     *   instance manages */
    uint32_t                numRxUhcCh;
    /**< Number of high capacity RX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_RX_UHC_CH */
    uint32_t                startRxHcCh;
    /**< Start high capacity RX channel from which this UDMA driver instance
     *   manages */
    uint32_t                numRxHcCh;
    /**< Number of high capacity RX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_RX_HC_CH */
    uint32_t                startRxCh;
    /**< Start RX channel from which this UDMA driver instance manages */
    uint32_t                numRxCh;
    /**< Number of RX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_RX_CH */

#if (UDMA_NUM_MAPPED_TX_GROUP > 0)
    uint32_t                startMappedTxCh[UDMA_NUM_MAPPED_TX_GROUP];
    /**< Start Mapped TX channel from which this UDMA driver instance
     *   manages */
    uint32_t                numMappedTxCh[UDMA_NUM_MAPPED_TX_GROUP];
    /**< Number of Mapped TX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_MAPPED_TX_CH_PER_GROUP */
#endif

#if (UDMA_NUM_MAPPED_RX_GROUP > 0)
    uint32_t                startMappedRxCh[UDMA_NUM_MAPPED_RX_GROUP];
    /**< Start Mapped RX channel from which this UDMA driver instance
     *   manages */
    uint32_t                numMappedRxCh[UDMA_NUM_MAPPED_RX_GROUP];
    /**< Number of Mapped RX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_MAPPED_RX_CH_PER_GROUP */
#endif

#if ((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
    uint32_t                startMappedRing[UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP];
    /**< Start Mapped ring from which this UDMA driver instance
     *   manages */
    uint32_t                numMappedRing[UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP];
    /**< Number of Mapped ring to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_MAPPED_RING_PER_GROUP */
#endif

    uint32_t                startFreeFlow;
    /**< Start free flow from which this UDMA driver instance manages */
    uint32_t                numFreeFlow;
    /**< Number of free flow to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_FREE_FLOW */
    uint32_t                startFreeRing;
    /**< Start free ring from which this UDMA driver instance manages */
    uint32_t                numFreeRing;
    /**< Number of free ring to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_FREE_RING */

    uint32_t                startGlobalEvent;
    /**< Start global event from which this UDMA driver instance manages */
    uint32_t                numGlobalEvent;
    /**< Number of global event to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_GLOBAL_EVENT */
    uint32_t                startVintr;
    /**< Start VINT number from which this UDMA driver instance manages */
    uint32_t                numVintr;
    /**< Number of VINT to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_VINTR */
    uint32_t                startIrIntr;
    /**< Start IR interrupt from which this UDMA driver instance manages. */
    uint32_t                numIrIntr;
    /**< Number of IR interrupts to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_IR_INTR */
} Udma_RmInitPrms;

/**
 *  \brief UDMA driver object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
typedef struct Udma_DrvObjectInt
{
    uint32_t                instType;
    /**< Udma Instance Type */
    uint32_t                raType;
    /**< Udma Ring Accelerator Type */

#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
    /*
     * LCDMA DMSS specific instance parameters
     */
    CSL_BcdmaCfg             bcdmaRegs;
    /**< BCDMA register configuration */
    CSL_PktdmaCfg            pktdmaRegs;
    /**< PKTDMA register configuration */
#endif
#if (UDMA_SOC_CFG_RA_LCDMA_PRESENT == 1)
    CSL_LcdmaRingaccCfg     lcdmaRaRegs;
#endif
    /**< RA register configuration */
    CSL_IntaggrCfg          iaRegs;
    /**< Interrupt Aggregator configuration */
    uint32_t                udmapSrcThreadOffset;
    /**< UDMAP Source/TX thread offset */
    uint32_t                udmapDestThreadOffset;
    /**< UDMAP Dest/RX thread offset */
    uint32_t                maxRings;
    /**< Maximun number of rings present in the NAVSS instance */

    /*
     * TISCI RM parameters
     */
    uint16_t                devIdRing;
    /**< Ring RM ID */
    uint16_t                devIdUdma;
    /**< UDMA RM ID */
    uint16_t                devIdPsil;
    /**< PSIL RM ID */
    uint16_t                devIdIa;
    /**< IA RM ID */
    uint16_t                devIdIr;
    /**< IR RM ID */
    uint16_t                devIdCore;
    /**< Core RM ID */
    /*
     * TISCI Ring event IRQ params
     *
     * These IRQ offsets should be corresponding TISCI offset - ringNum Offset
     */
    uint16_t                srcIdRingIrq;
    /**< Ring completion event IRQ Source ID. */
    uint32_t                blkCopyRingIrqOffset;
    /**< Block Copy channel ring completion event IRQ offset. */
    uint32_t                txRingIrqOffset;
    /**< TX channel ring completion event IRQ offset. */
    uint32_t                rxRingIrqOffset;
    /**< RX channel ring completion event IRQ offset. */
    /*
     * TISCI TR event IRQ params
     *
     * These IRQ offsets should be corresponding TISCI offset - chNum Offset
     */
    uint16_t                srcIdTrIrq;
    /**< TR event IRQ Source ID. */
    uint32_t                blkCopyTrIrqOffset;
    /**< Block Copy channel TR event IRQ offset. */
    uint32_t                txTrIrqOffset;
    /**< TX channel TR event IRQ offset. */
    uint32_t                rxTrIrqOffset;
    /**< RX channel TR event IRQ offset. */
    /*
     * Channel Offsets
     */
    uint32_t                txChOffset;
    /**< TX channel offset. */
    uint32_t                extChOffset;
    /**< External channel offset. */
    uint32_t                rxChOffset;
    /**< RX channel offset. */
    /*
     *  The driver allocates ringNum = chNum (for BlkCpoy)
                                     = chNum + txChOffset (for SplitTR Tx)
                                     = chNum + rxChOffset (for SplitTR Rx)

        For CSL_bcdma* API's passed param ->channel_num = txChNum (for BlkCopy)
                                                        = txChNum + txChOffset (for SplitTR Tx)
                                                        = rxChNum + rxChOffset (for SplitTR Rx)
    */
    /*
     * Other Offsets
     */
    uint32_t                iaGemOffset;
    /**< IA global event map offset to differentiate between main and MCU NAVSS */
    uint32_t                trigGemOffset;
    /**< UDMAP trigger global event map offset to differentiate between main
     *   and MCU NAVSS */

    Udma_EventObject     globalEventObj;
    /**< Object to store global event. */
    Udma_EventHandle     globalEventHandle;
    /**< Global event handle. */

    Udma_InitPrms           initPrms;
    /**< Object to store the init params. */
    Udma_RmInitPrms         rmInitPrms;
    /**< RM init parameters */
    uint32_t                drvInitDone;
    /**< Flag to check if the driver object is init properly or not. */

    /*
     * RM objects.
     * This is a bitwise flag
     * 1 - free, 0 - allocated
     */
    uint32_t                blkCopyChFlag[UDMA_RM_BLK_COPY_CH_ARR_SIZE];
    /**< UDMA Block copy channel allocation flag */
    uint32_t                blkCopyHcChFlag[UDMA_RM_BLK_COPY_HC_CH_ARR_SIZE];
    /**< UDMA high capacity Block copy channel allocation flag */
    uint32_t                blkCopyUhcChFlag[UDMA_RM_BLK_COPY_UHC_CH_ARR_SIZE];
    /**< UDMA ultra high capacity Block copy channel allocation flag */

    uint32_t                txChFlag[UDMA_RM_TX_CH_ARR_SIZE];
    /**< UDMA TX channel allocation flag */
    uint32_t                txHcChFlag[UDMA_RM_TX_HC_CH_ARR_SIZE];
    /**< UDMA high capacity TX channel allocation flag */
    uint32_t                txUhcChFlag[UDMA_RM_TX_UHC_CH_ARR_SIZE];

    /**< UDMA ultra high capacity TX channel allocation flag */
    uint32_t                rxChFlag[UDMA_RM_RX_CH_ARR_SIZE];
    /**< UDMA RX channel allocation flag */
    uint32_t                rxHcChFlag[UDMA_RM_RX_HC_CH_ARR_SIZE];
    /**< UDMA high capacity RX channel allocation flag */
    uint32_t                rxUhcChFlag[UDMA_RM_RX_UHC_CH_ARR_SIZE];
    /**< UDMA ultra high capacity RX channel allocation flag */

#if (UDMA_NUM_MAPPED_TX_GROUP > 0)
    uint32_t                mappedTxChFlag[UDMA_NUM_MAPPED_TX_GROUP][UDMA_RM_MAPPED_TX_CH_ARR_SIZE];
    /**< UDMA mapped TX channel allocation flag */
#endif
#if (UDMA_NUM_MAPPED_RX_GROUP > 0)
    uint32_t                mappedRxChFlag[UDMA_NUM_MAPPED_RX_GROUP][UDMA_RM_MAPPED_RX_CH_ARR_SIZE];
    /**< UDMA mapped RX channel allocation flag */
#endif
#if ((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
    uint32_t                mappedRingFlag[UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP][UDMA_RM_MAPPED_RING_ARR_SIZE];
    /**< UDMA mapped ring allocation flag */
#endif

    uint32_t                freeRingFlag[UDMA_RM_FREE_RING_ARR_SIZE];
    /**< UDMA free ring allocation flag */
    uint32_t                freeFlowFlag[UDMA_RM_FREE_FLOW_ARR_SIZE];
    /**< UDMA free flow allocation flag */
    uint32_t                globalEventFlag[UDMA_RM_GLOBAL_EVENT_ARR_SIZE];
    /**< IA global event allocation flag */
    uint32_t                vintrFlag[UDMA_RM_VINTR_ARR_SIZE];
    /**< IA VINTR allocation flag */
    uint32_t                irIntrFlag[UDMA_RM_IR_INTR_ARR_SIZE];
    /**< IR interrupt allocation flag */

    SemaphoreP_Object       *rmLock;
    /**< Mutex to protect RM allocation. */
    SemaphoreP_Object       rmLockObj;
    /**< Mutex object. */
} Udma_DrvObject;
#else
/**
 *  \brief UDMA resource manager init parameters.
 *
 *  This assumes contiguos allocation of 'N' resources from a start offset
 *  to keep the interface simple.
 *
 *  Note: This is applicable for the driver handle as given during init call.
 *  The init call doesn't (can't rather) check for resource overlap across
 *  handles and across cores. It is the callers responsibility to ensure that
 *  resources overlaps are not present.
 */
typedef struct
{
    uint32_t                startBlkCopyUhcCh;
    /**< Start ultra high capacity block copy channel from which this UDMA
     *   driver instance manages */
    uint32_t                numBlkCopyUhcCh;
    /**< Number of ultra high capacity block copy channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_BLK_COPY_UHC_CH */
    uint32_t                startBlkCopyHcCh;
    /**< Start high capacity block copy channel from which this UDMA
     *   driver instance manages */
    uint32_t                numBlkCopyHcCh;
    /**< Number of ultra high capacity block copy channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_BLK_COPY_HC_CH */
    uint32_t                startBlkCopyCh;
    /**< Start Block copy channel from which this UDMA driver instance manages */
    uint32_t                numBlkCopyCh;
    /**< Number of Block copy channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_BLK_COPY_CH */

    uint32_t                startTxUhcCh;
    /**< Start ultra high capacity TX channel from which this UDMA driver
     *   instance manages */
    uint32_t                numTxUhcCh;
    /**< Number of ultra high capacity TX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_TX_UHC_CH */
    uint32_t                startTxHcCh;
    /**< Start high capacity TX channel from which this UDMA driver instance
     *   manages */
    uint32_t                numTxHcCh;
    /**< Number of high capacity TX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_TX_HC_CH */
    uint32_t                startTxCh;
    /**< Start TX channel from which this UDMA driver instance manages */
    uint32_t                numTxCh;
    /**< Number of TX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_TX_CH */

    uint32_t                startRxUhcCh;
    /**< Start ultra high capacity RX channel from which this UDMA driver
     *   instance manages */
    uint32_t                numRxUhcCh;
    /**< Number of high capacity RX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_RX_UHC_CH */
    uint32_t                startRxHcCh;
    /**< Start high capacity RX channel from which this UDMA driver instance
     *   manages */
    uint32_t                numRxHcCh;
    /**< Number of high capacity RX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_RX_HC_CH */
    uint32_t                startRxCh;
    /**< Start RX channel from which this UDMA driver instance manages */
    uint32_t                numRxCh;
    /**< Number of RX channel to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_RX_CH */
    uint32_t                startFreeFlow;
    /**< Start free flow from which this UDMA driver instance manages */
    uint32_t                numFreeFlow;
    /**< Number of free flow to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_FREE_FLOW */
    uint32_t                startFreeRing;
    /**< Start free ring from which this UDMA driver instance manages */
    uint32_t                numFreeRing;
    /**< Number of free ring to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_FREE_RING */

    uint32_t                startGlobalEvent;
    /**< Start global event from which this UDMA driver instance manages */
    uint32_t                numGlobalEvent;
    /**< Number of global event to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_GLOBAL_EVENT */
    uint32_t                startVintr;
    /**< Start VINT number from which this UDMA driver instance manages */
    uint32_t                numVintr;
    /**< Number of VINT to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_VINTR */
    uint32_t                startIrIntr;
    /**< Start IR interrupt from which this UDMA driver instance manages. */
    uint32_t                numIrIntr;
    /**< Number of IR interrupts to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_IR_INTR */
    uint32_t                proxyThreadNum;
    /**< Proxy thread to push/pop to ring in proxy mode.
     *   By default driver will initialize to a default value based on
     *   core and NAVSS instance. User can override this based on need.
     *   The default proxy allocation starts from UDMA_DEFAULT_RM_PROXY_THREAD_START
     *   and will allocate 1 per core. So total allocation will be from
     *   UDMA_DEFAULT_RM_PROXY_THREAD_START to
     *   (UDMA_DEFAULT_RM_PROXY_THREAD_START + num cores) in an SOC.
     *
     *   The proxy thread number should be allocated within a NAVSS instance
     *   as a proxy can access ring only within the same NAVSS instance. The
     *   driver assumes the right proxy instance to use based on the
     *   instance ID (instId) provided in #Udma_init API
     *
     *   Also this should be set a unique number across core and NAVSS
     *   instance. Care should be taken not to use the same proxy across
     *   the system.
     *
     *   Warning: When using multiple UDMA handle for the same NAVSS instance
     *   within a core, care should taken to provide a unique proxy number
     *   per handle. Otherwise the the driver handle will use the same
     *   proxy for ring operation and will result in unintended behaviour and
     *   corruption of ring memory/operation.
     */
    uint32_t                startProxy;
    /**< Start proxy from which this UDMA driver instance manages.
     *   Note this should not overlap with proxyThreadNum */
    uint32_t                numProxy;
    /**< Number of proxy to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_PROXY */
    uint32_t                startRingMon;
    /**< Start monitor from which this UDMA driver instance manages */
    uint32_t                numRingMon;
    /**< Number of monitors to be managed.
     *   Note: This cannot exceed UDMA_RM_MAX_RING_MON */
} Udma_RmInitPrms;
/**
 *  \brief UDMA ring object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
typedef struct Udma_RingObjectInt
{
    Udma_DrvHandle           drvHandle;
    /**< Pointer to global driver handle. */

    uint16_t                    ringNum;
    /**< Ring number */
    CSL_RingAccRingCfg          cfg;
    /**< Ring config */

    /* Below register overlay pointers provided for debug purpose to
     * readily view the registers */
    volatile CSL_ringacc_cfgRegs_RING  *pCfgRegs;
    /**< Pointer to RA config register overlay */
    volatile CSL_ringacc_rtRegs_RINGRT *pRtRegs;
    /**< Pointer to RA RT config register overlay */
    /* Proxy address for the ring. Calculated at alloc time to reduce cycles at
     * runtime */
    uintptr_t                   proxyAddr;
    /**< Proxy address for push/pop ring operation through proxy */
    uint32_t                    ringInitDone;
    /**< Flag to set the ring object is init. */

    uint32_t                    mappedRingGrp;
    /**< The allocated mapped ring group when channel type is
     *   #UDMA_CH_TYPE_TX_MAPPED or #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   This is needed to free the mapped ring.
     *
     *   Refer \ref Udma_MappedTxGrpSoc macro for details about mapped TX ring groups
     *   or \ref Udma_MappedRxGrpSoc macro for details about mapped RX ring groups.
     *
     *   For unmapped case, this will be #UDMA_MAPPED_GROUP_INVALID
     */
    uint32_t                    mappedChNum;
    /**< The assigned mapped channel number when channel type is
     *   #UDMA_CH_TYPE_TX_MAPPED or #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   This is needed to free the mapped ring.
     *
     *   For unmapped case, this will be #UDMA_DMA_CH_INVALID.
     */
} Udma_RingObject;

/**
 *  \brief UDMA flow object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
typedef struct Udma_FlowObjectInt
{
    Udma_DrvHandle       drvHandle;
    /**< Pointer to global driver handle. */

    uint32_t                flowStart;
    /**< Flow ID start number.
     *
     *   Note: In case of mapped flow(in devices like AM64x), this indicates the
     *   mapped flow idx managed by this flow handle.
     *
    */
    uint32_t                flowCnt;
    /**< Number of flow IDs allocated - Contiguos flows are allocated
     *
     *   Note: In case of mapped flow(in devices like AM64x), this will be 1
     *   since only one mapped flow is managed by a flow handle.
    */

    uint32_t                flowInitDone;
    /**< Flag to set the flow object is init. */

    uint32_t                mappedFlowGrp;
    /**< The allocated mapped flow group when channel type is
     *   #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   This is needed to free the mapped flow.
     *
     *   Refer \ref Udma_MappedRxGrpSoc macro for details about mapped RX flow groups.
     *
     *   For unmapped case, this will be #UDMA_MAPPED_GROUP_INVALID
     */
    uint32_t                mappedChNum;
    /**< The assigned mapped channel number when channel type is
     *   #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   This is needed to free the mapped flow.
     *
     *   For unmapped case, this will be #UDMA_DMA_CH_INVALID.
     */
} Udma_FlowObject;

/**
 *  \brief UDMA event object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
typedef struct Udma_EventObjectInt
{
    Udma_DrvHandle       drvHandle;
    /**< Pointer to global driver handle. */
    Udma_EventPrms          eventPrms;
    /**< Event parameters passed during event registeration. */

    uint32_t                globalEvent;
    /**< Allocated IA global event. */
    uint32_t                vintrNum;
    /**< Allocated IA VINT register. */
    uint32_t                vintrBitNum;
    /**< Allocated IA VINT bit number - 0 to 63. */
    uint32_t                irIntrNum;
    /**< Allocated interrupt router number.
     * In case of devices like AM64x, where there are no Interrupt Routers,
     * irIntrNum refers to coreIntrNum number itself. */
    uint32_t                coreIntrNum;
    /**< Allocated core interrupt number. */

    Udma_EventHandle     nextEvent;
    /**< Pointer to next event - used in shared event for traversing in ISR */
    Udma_EventHandle     prevEvent;
    /**< Pointer to previous event - used in shared event for traversing during
     *   event un-registration */

    HwiP_Object            *hwiHandle;
    /**< HWI handle. */
    HwiP_Object             hwiObject;
    /**< HWI Object. */
    uint64_t                vintrBitAllocFlag;
    /**< For master event, this stores the alloc flag for each bit within
     *   IA register. This is not used for slave events and is always set to
     *   zero */

    /* Below register overlay pointers provided for debug purpose to
     * readily view the registers */
    volatile CSL_intaggr_imapRegs_gevi  *pIaGeviRegs;
    /**< Pointer to IA global event register overlay */
    volatile CSL_intaggr_intrRegs_vint  *pIaVintrRegs;
    /**< Pointer to IA virtual interrupt register overlay */

    uint32_t                eventInitDone;
    /**< Flag to set the event object is init. */
} Udma_EventObject;

/**
 *  \brief UDMA channel object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
typedef struct Udma_ChObjectInt
{
    uint32_t                chType;
    /**< UDMA channel type. Refer \ref Udma_ChType. */
    Udma_ChPrms             chPrms;
    /**< Object to store the channel params. */
    Udma_DrvHandle       drvHandle;
    /**< Pointer to global driver handle. */
    uint32_t                txChNum;
    /**< Allocated TX channel number - this is relative channel number from
     *   base TX channel. This is valid only when the channel is opened for
     *   TX and block copy mode */
    uint32_t                rxChNum;
    /**< Allocated RX channel number - this is relative channel number from
     *   base RX channel. This is valid only when the channel is opened for
     *   RX and block copy mode */
    uint32_t                extChNum;
    /**< Allocated Ext channel number - this is relative channel number from
     *   base External channel. This is valid only when the channel is opened
     *   for UTC mode */
    uint32_t                pdmaChNum;
    /**< Allocated peer PDMA channel number. This is valid only when the
     *   channel is opened for PDMA mode */
    uint32_t                peerThreadId;
    /**< Peer channel thread ID - this is or'ed with thread offset. */

    Udma_RingHandle      fqRing;
    /**< Free queue ring handle */
    Udma_RingHandle      cqRing;
    /**< Completion queue ring handle
    *    For AM64x kind of devices, where there is no seperate Completion queue,
    *    this points to fqRing itself.
    */
    Udma_RingHandle      tdCqRing;
    /**< Teardown completion queue ring handle */

    Udma_RingObject         fqRingObj;
    /**< Free queue ring object */
    Udma_RingObject        cqRingObj;
    /**< Completion queue ring object
    *    Not used for AM64x kind of devices, where there is no seperate Completion queue.
    */
    Udma_RingObject        tdCqRingObj;
    /**< Teardown completion queue ring object
    *    Not used for AM64x kind of devices, where teardown function is not present.
    */

    Udma_FlowHandle      defaultFlow;
    /**< Default flow handle */
    Udma_FlowObject      defaultFlowObj;
    /**< Default flow object - Flow ID equal to the RX channel is reserved
     *   as the default flow for the channel. This object is used for
     *   providing handle to the caller to re-program the default flow using
     *   the standard flow API's */

    Udma_ChTxPrms           txPrms;
    /**< TX channel parameter passed during channel config. */
    Udma_ChRxPrms           rxPrms;
    /**< RX channel parameter passed during channel config. */

    /* Below UDMAP register overlay pointers provided for debug purpose to
     * readily view the registers */
    volatile CSL_udmap_txccfgRegs_chan  *pTxCfgRegs;
    /**< Pointer to UDMAP TX config register overlay */
    volatile CSL_udmap_txcrtRegs_chan   *pTxRtRegs;
    /**< Pointer to UDMAP TX RT config register overlay */
    volatile CSL_udmap_rxccfgRegs_chan  *pRxCfgRegs;
    /**< Pointer to UDMAP RX config register overlay */
    volatile CSL_udmap_rxcrtRegs_chan   *pRxRtRegs;
    /**< Pointer to UDMAP RX RT config register overlay */
    volatile CSL_udmap_txccfgRegs_chan  *pExtCfgRegs;
    /**< Pointer to UDMAP External config register overlay */
    volatile CSL_udmap_txcrtRegs_chan   *pExtRtRegs;
    /**< Pointer to UDMAP External RT config register overlay */
    uint32_t                chInitDone;
    /**< Flag to set the channel object is init. */
    uint32_t                chOesAllocDone;
    /**< Flag to check if the channel's OES is allocated. This is required
     *   because the channel OES is used for chaining as well as for
     *   TR event registeration. This allows to check for error when both
     *   are requested by user on the same channel */
    uint32_t                trigger;
    /**< Channel trigger used when chaining channels - needed at the time of
     *   breaking the chaining */
} Udma_ChObject;
/**
 *  \brief UDMA driver object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
typedef struct Udma_DrvObjectInt
{
    uint32_t                instType;
    /**< Udma Instance Type */
    uint32_t                raType;
    /**< Udma Ring Accelerator Type */
    /*
     * NAVSS instance parameters
     */
    CSL_UdmapCfg            udmapRegs;
    /**< UDMAP register configuration */
    CSL_RingAccCfg          raRegs;
    /**< RA register configuration */
    CSL_IntaggrCfg          iaRegs;
    /**< Interrupt Aggregator configuration */
    uint32_t                udmapSrcThreadOffset;
    /**< UDMAP Source/TX thread offset */
    uint32_t                udmapDestThreadOffset;
    /**< UDMAP Dest/RX thread offset */
    uint32_t                maxRings;
    /**< Maximun number of rings present in the NAVSS instance */
    uint32_t                maxProxy;
    /**< Maximun number of proxy present in the NAVSS instance */
    uint32_t                maxRingMon;
    /**< Maximun number of ring monitors present in the NAVSS instance */
    /*
     * Proxy parameters
     */
    CSL_ProxyCfg            proxyCfg;
    /*< Proxy register configuration */
    CSL_ProxyTargetParams   proxyTargetRing;
    /*< Proxy ring target register configuration */
    uint32_t                proxyTargetNumRing;
    /*< Proxy ring target index */
    /*
     * TISCI RM parameters
     */
    uint16_t                devIdRing;
    /**< Ring RM ID */
    uint16_t                devIdUdma;
    /**< UDMA RM ID */
    uint16_t                devIdPsil;
    /**< PSIL RM ID */
    uint16_t                devIdIa;
    /**< IA RM ID */
    uint16_t                devIdIr;
    /**< IR RM ID */
    uint16_t                devIdProxy;
    /**< Proxy RM ID */
    uint16_t                devIdCore;
    /**< Core RM ID */
    /*
     * TISCI Ring event IRQ params
     *
     * These IRQ offsets should be corresponding TISCI offset - ringNum Offset
     */
    uint16_t                srcIdRingIrq;
    /**< Ring completion event IRQ Source ID. */
    uint32_t                blkCopyRingIrqOffset;
    /**< Block Copy channel ring completion event IRQ offset. */
    uint32_t                txRingIrqOffset;
    /**< TX channel ring completion event IRQ offset. */
    uint32_t                rxRingIrqOffset;
    /**< RX channel ring completion event IRQ offset. */
    /*
     * TISCI TR event IRQ params
     *
     * These IRQ offsets should be corresponding TISCI offset - chNum Offset
     */
    uint16_t                srcIdTrIrq;
    /**< TR event IRQ Source ID. */
    uint32_t                blkCopyTrIrqOffset;
    /**< Block Copy channel TR event IRQ offset. */
    uint32_t                txTrIrqOffset;
    /**< TX channel TR event IRQ offset. */
    uint32_t                rxTrIrqOffset;
    /**< RX channel TR event IRQ offset. */
    /*
     * Channel Offsets
     */
    uint32_t                txChOffset;
    /**< TX channel offset. */
    uint32_t                extChOffset;
    /**< External channel offset. */
    uint32_t                rxChOffset;
    /**< RX channel offset. */
    /*
     *  The driver allocates ringNum = chNum (for BlkCpoy)
                                     = chNum + txChOffset (for SplitTR Tx)
                                     = chNum + rxChOffset (for SplitTR Rx)

        For CSL_bcdma* API's passed param ->channel_num = txChNum (for BlkCopy)
                                                        = txChNum + txChOffset (for SplitTR Tx)
                                                        = rxChNum + rxChOffset (for SplitTR Rx)
    */
    /*
     * Other Offsets
     */
    uint32_t                iaGemOffset;
    /**< IA global event map offset to differentiate between main and MCU NAVSS */
    uint32_t                trigGemOffset;
    /**< UDMAP trigger global event map offset to differentiate between main
     *   and MCU NAVSS */

    Udma_EventObject     globalEventObj;
    /**< Object to store global event. */
    Udma_EventHandle     globalEventHandle;
    /**< Global event handle. */

    Udma_InitPrms           initPrms;
    /**< Object to store the init params. */
    Udma_RmInitPrms         rmInitPrms;
    /**< RM init parameters */
    uint32_t                drvInitDone;
    /**< Flag to check if the driver object is init properly or not. */

    /*
     * RM objects.
     * This is a bitwise flag
     * 1 - free, 0 - allocated
     */
    uint32_t                blkCopyChFlag[UDMA_RM_BLK_COPY_CH_ARR_SIZE];
    /**< UDMA Block copy channel allocation flag */
    uint32_t                blkCopyHcChFlag[UDMA_RM_BLK_COPY_HC_CH_ARR_SIZE];
    /**< UDMA high capacity Block copy channel allocation flag */
    uint32_t                blkCopyUhcChFlag[UDMA_RM_BLK_COPY_UHC_CH_ARR_SIZE];
    /**< UDMA ultra high capacity Block copy channel allocation flag */

    uint32_t                txChFlag[UDMA_RM_TX_CH_ARR_SIZE];
    /**< UDMA TX channel allocation flag */
    uint32_t                txHcChFlag[UDMA_RM_TX_HC_CH_ARR_SIZE];
    /**< UDMA high capacity TX channel allocation flag */
    uint32_t                txUhcChFlag[UDMA_RM_TX_UHC_CH_ARR_SIZE];

    /**< UDMA ultra high capacity TX channel allocation flag */
    uint32_t                rxChFlag[UDMA_RM_RX_CH_ARR_SIZE];
    /**< UDMA RX channel allocation flag */
    uint32_t                rxHcChFlag[UDMA_RM_RX_HC_CH_ARR_SIZE];
    /**< UDMA high capacity RX channel allocation flag */
    uint32_t                rxUhcChFlag[UDMA_RM_RX_UHC_CH_ARR_SIZE];
    /**< UDMA ultra high capacity RX channel allocation flag */

    uint32_t                freeRingFlag[UDMA_RM_FREE_RING_ARR_SIZE];
    /**< UDMA free ring allocation flag */
    uint32_t                freeFlowFlag[UDMA_RM_FREE_FLOW_ARR_SIZE];
    /**< UDMA free flow allocation flag */
    uint32_t                globalEventFlag[UDMA_RM_GLOBAL_EVENT_ARR_SIZE];
    /**< IA global event allocation flag */
    uint32_t                vintrFlag[UDMA_RM_VINTR_ARR_SIZE];
    /**< IA VINTR allocation flag */
    uint32_t                irIntrFlag[UDMA_RM_IR_INTR_ARR_SIZE];
    /**< IR interrupt allocation flag */

    SemaphoreP_Object      *rmLock;
    /**< Mutex to protect RM allocation. */
    SemaphoreP_Object       rmLockObj;
    /**< Mutex object. */
} Udma_DrvObject;
#endif


/** \brief Cache line size for alignment of descriptor and buffers */
#define UDMA_CACHELINE_ALIGNMENT        (128U)

/** \brief Macro to align the size in bytes to UDMA cache line alignment */
#define UDMA_ALIGN_SIZE(x)              (((x) + UDMA_CACHELINE_ALIGNMENT - 1U) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))

/** \brief Default UDMA channel disable timeout */
#define UDMA_DEFAULT_CH_DISABLE_TIMEOUT (100U)

/**
 *  \anchor Udma_ErrorCodes
 *  \name UDMA Error Codes
 *
 *  Error codes returned by UDMA APIs
 *
 *  @{
 */
/** \brief API call successful */
#define UDMA_SOK                        ( (int32_t) (0))
/** \brief API call returned with error as failed. Used for generic error.
 *  It may be some hardware failure and/or software failure. */
#define UDMA_EFAIL                      (-(int32_t) (1))
/** \brief API call returned with error as bad arguments.
 *  Typically, NULL pointer passed to the API where its not expected. */
#define UDMA_EBADARGS                   (-(int32_t) (2))
/** \brief API call returned with error as invalid parameters. Typically
 *  when parameters passed are not valid or out of range. */
#define UDMA_EINVALID_PARAMS            (-(int32_t) (3))
/** \brief API call returned with error as timed out. Typically API is
 *  waiting for some condition and returned as condition not happened
 *  in the timeout period. */
#define UDMA_ETIMEOUT                   (-(int32_t) (4))
/** \brief API call returned with error as allocation failed. */
#define UDMA_EALLOC                     (-(int32_t) (5))
/** @} */

/**
 *  \anchor Udma_InstanceId
 *  \name UDMA Instance ID
 *
 *  This represents the various Instances in an SOC. The actual Instances present
 *  in the chip is SOC dependent. Refer soc file for the actual instance
 *  present. Kindly use \ref Udma_InstanceIdSoc macros for SOC specific name.
 *
 *  @{
 */
#define UDMA_INST_ID_0                  (0U)
#define UDMA_INST_ID_1                  (1U)
#define UDMA_INST_ID_2                  (2U)
#define UDMA_INST_ID_3                  (3U)
/** @} */

/**
 *  \anchor Udma_UtcId
 *  \name UDMA UTC ID
 *
 *  This represents the various UTC IP in the SOC. The actual UTC present
 *  in the chip is SOC dependent. Refer soc file for the actual instance
 *  present. Kindly use Udma_UtcIdSoc macros for SOC specific name.
 *
 *  @{
 */
#define UDMA_UTC_ID0                    (0U)
#define UDMA_UTC_ID1                    (1U)
#define UDMA_UTC_ID2                    (2U)
#define UDMA_UTC_ID3                    (3U)
/** @} */

/**
 *  \anchor Udma_MappedGrp
 *  \name UDMA Mapped Group
 *
 *  This represents the various Mapped TX & RX Channels/Rings in the SOC. The actual
 *  Mapped TX & RX groups present in the chip is SOC dependent. Refer soc file
 *  for the actual instance present.
 *  Kindly use \ref Udma_MappedTxGrpSoc macros for Mapped TX SOC specific name
 *  and \ref Udma_MappedRxGrpSoc macros for Mapped RX SOC specific name.
 *
 *  @{
 */
#define UDMA_MAPPED_GROUP0              (0U)
#define UDMA_MAPPED_GROUP1              (1U)
#define UDMA_MAPPED_GROUP2              (2U)
#define UDMA_MAPPED_GROUP3              (3U)
#define UDMA_MAPPED_GROUP4              (4U)
#define UDMA_MAPPED_GROUP5              (5U)
#define UDMA_MAPPED_GROUP6              (6U)
#define UDMA_MAPPED_GROUP7              (7U)
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_TYPES_H_ */

/** @} */

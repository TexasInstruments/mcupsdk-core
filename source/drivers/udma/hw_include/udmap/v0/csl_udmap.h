/*
 *  Copyright (C) 2024 Texas Instruments Incorporated.
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
 *
 */
/**
 *  \file  csl_udmap.h
 *
 *  \brief
 *     Header file containing various enumerations, structure definitions and function
 *  declarations for the Universal DMA Packet (UDMAP) IP.
 */
/**
 *  \ingroup CSL_IP_MODULE
 *  \defgroup CSL_UDMAP UDMAP CSL-FL
 *
 *  @{
 */

#ifndef CSL_UDMAP_H_
#define CSL_UDMAP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <drivers/udma/hw_include/cslr_udmap.h>

/**
@defgroup CSL_UDMAP_DATASTRUCT  UDMAP Data Structures
@ingroup CSL_UDMAP_API
*/
/**
@defgroup CSL_UDMAP_FUNCTION  UDMAP Functions
@ingroup CSL_UDMAP_API
*/
/**
@defgroup CSL_UDMAP_ENUM UDMAP Enumerated Data Types
@ingroup CSL_UDMAP_API
*/

/**
 *  \addtogroup CSL_UDMAP_ENUM
 *  @{
 */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible channel directions
 *
 *  \anchor CSL_UdmapChanDir
 *  \name UDMAP channel direction
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_UdmapChanDir;
    /** Transmit direction */
#define CSL_UDMAP_CHAN_DIR_TX   ((uint32_t) 0U)
    /** Receive direction */
#define CSL_UDMAP_CHAN_DIR_RX   ((uint32_t) 1U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the nominal burst size and alignment for
 * data transfers on a TX or RX channel
 *
 *  \anchor CSL_UdmapChanBurstSize
 *  \name UDMAP channel burst size
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_UdmapChanBurstSize;
    /** 64-byte burst size */
#define CSL_UDMAP_CHAN_BURST_SIZE_64_BYTES  ((uint32_t) 1U)
    /** 128-byte burst size */
#define CSL_UDMAP_CHAN_BURST_SIZE_128_BYTES ((uint32_t) 2U)
    /** 256-byte burst size */
#define CSL_UDMAP_CHAN_BURST_SIZE_256_BYTES ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible descriptor types
 *
 *  \anchor CSL_UdmapDescType
 *  \name UDMAP descriptor type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_UdmapDescType;
    /** Host */
#define CSL_UDMAP_DESC_TYPE_HOST        ((uint32_t) 0U)
    /** Host single-buffer */
#define CSL_UDMAP_DESC_TYPE_HOST_SB     ((uint32_t) 1U)
    /** Monolithic */
#define CSL_UDMAP_DESC_TYPE_MONOLITHIC  ((uint32_t) 2U)
    /** Reserved */
#define CSL_UDMAP_DESC_TYPE_RESERVED    ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the ps location for the descriptor
 *
 *  \anchor CSL_UdmapPsLoc
 *  \name UDMAP protocol-specific data location
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_UdmapPsLoc;
    /** Located in descriptor */
#define CSL_UDMAP_PS_LOC_DESC       ((uint32_t) 0U)
    /** Located in packet */
#define CSL_UDMAP_PS_LOC_PACKET     ((uint32_t) 1U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible address types
 *
 *  \anchor CSL_UdmapAddrType
 *  \name UDMAP address type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_UdmapAddrType;
    /** Physical addressing */
#define CSL_UDMAP_ADDR_TYPE_PHYS    ((uint32_t) 0U)
    /** Intermediate addressing */
#define CSL_UDMAP_ADDR_TYPE_INTER   ((uint32_t) 1U)
    /** Virtual addressing */
#define CSL_UDMAP_ADDR_TYPE_VIRT    ((uint32_t) 2U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible channel types
 *
 *  \anchor CSL_UdmapChanType
 *  \name UDMAP channel type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_UdmapChanType;
    /** RM, Packet Mode, Pass by reference */
#define CSL_UDMAP_CHAN_TYPE_REF_PKT_RING        ((uint32_t) 2U)
    /** QM, Packet Single Buffer Mode, Pass by reference */
#define CSL_UDMAP_CHAN_TYPE_REF_PKTSB_QUEUE     ((uint32_t) 3U)
    /** RM, TR Mode, Pass by reference */
#define CSL_UDMAP_CHAN_TYPE_REF_TR_RING         ((uint32_t) 10U)
    /** RM, TR Mode, Direct 'value' mode */
#define CSL_UDMAP_CHAN_TYPE_VAL_TR_RING         ((uint32_t) 11U)
    /** RM, TR Mode, Block Copy, Pass by reference */
#define CSL_UDMAP_CHAN_TYPE_COPY_REF_TR_RING    ((uint32_t) 12U)
    /** RM, TR Mode, Block Copy, Direct 'value' mode */
#define CSL_UDMAP_CHAN_TYPE_COPY_VAL_TR_RING    ((uint32_t) 13U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines how tag values are determined
 *
 *  \anchor CSL_UdmapTagSelect
 *  \name UDMAP tag select
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_UdmapTagSelect;
    /** Do not overwrite */
#define CSL_UDMAP_TAG_SELECT_NO_OVERWRITE               ((uint32_t) 0U)
    /** Overwrite with value given in tag value */
#define CSL_UDMAP_TAG_SELECT_OVERWRITE_WITH_VAL         ((uint32_t) 1U)
    /** Overwrite with flow_id[7:0] from back-end */
#define CSL_UDMAP_TAG_SELECT_OVERWRITE_WITH_FLOWID_7_0  ((uint32_t) 2U)
    /** Overwrite with flow_id[15:8] from back-end */
#define CSL_UDMAP_TAG_SELECT_OVERWRITE_WITH_FLOWID_15_8 ((uint32_t) 3U)
    /** Overwrite with tag[7:0] from back-end */
#define CSL_UDMAP_TAG_SELECT_OVERWRITE_WITH_TAG_7_0     ((uint32_t) 4U)
    /** Overwrite with tag[15:8] from back-end */
#define CSL_UDMAP_TAG_SELECT_OVERWRITE_WITH_TAG_15_8    ((uint32_t) 5U)
    /** Invalid */
#define CSL_UDMAP_TAG_SELECT_INVALID                    ((uint32_t) 6U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator selects which scheduling bin the channel will be
 *  placed in for bandwidth allocation of the DMA units
 *
 *  \anchor CSL_UdmapChanSchedPri
 *  \name UDMAP channel schedling priority
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_UdmapChanSchedPri;
    /** High priority */
#define CSL_UDMAP_CHAN_SCHED_PRI_HIGH       ((uint32_t) 0U)
    /** Medium-High priority */
#define CSL_UDMAP_CHAN_SCHED_PRI_MED_HIGH   ((uint32_t) 1U)
    /** Medium-Low priority */
#define CSL_UDMAP_CHAN_SCHED_PRI_MED_LOW    ((uint32_t) 2U)
    /** Low priority */
#define CSL_UDMAP_CHAN_SCHED_PRI_LOW        ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the UDMAP internal sub-blocks that have
 *  automatic dynamic clock gating support for power savings.
 *
 *  UDMAP revision 2.2.0.0 and later provides automatic dynamic clock gating
 *  support for internal sub-blocks based on real-time monitoring of activity.
 *
 *  \anchor CSL_UdmapAutoClkgateBlock
 *  \name UDMAP automatic dynamic clock gate block ID
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint64_t CSL_UdmapAutoClkgateBlock;
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_EDC                                (((uint64_t)1U)<<63)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_STATS                              (((uint64_t)1U)<<62)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_PROXY                              (((uint64_t)1U)<<61)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_PSILIF                             (((uint64_t)1U)<<60)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_P2P                                (((uint64_t)1U)<<59)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_EPSILIF                            (((uint64_t)1U)<<58)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_EHANDLER                           (((uint64_t)1U)<<57)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RINGPEND                           (((uint64_t)1U)<<56)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_PER_CHANNEL_FIFO                (((uint64_t)1U)<<55)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_PER_CHANNEL_FIFO                (((uint64_t)1U)<<54)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_PREFETCH_CFG                    (((uint64_t)1U)<<53)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_PREFETCH_CFG                    (((uint64_t)1U)<<52)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_PM_TX_PACKET_DMA_UNIT              (((uint64_t)1U)<<51)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_CFG_STATE_RAM_BLK               (((uint64_t)1U)<<50)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_PREFETCH_BUFFER                 (((uint64_t)1U)<<49)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_PREFETCH_BUFFER                 (((uint64_t)1U)<<48)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_FLOW_FIREWALL                   (((uint64_t)1U)<<47)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_EXTERNAL_CHANNEL_COHERENCY_UNIT (((uint64_t)1U)<<46)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_PACKET_COHERENCY_UNIT           (((uint64_t)1U)<<45)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_PACKET_COHERENCY_UNIT           (((uint64_t)1U)<<44)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_TR_COHERENCY_UNIT               (((uint64_t)1U)<<43)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_TR_COHERENCY_UNIT               (((uint64_t)1U)<<42)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_EVENT_COHERENCY_UNIT            (((uint64_t)1U)<<41)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_EVENT_COHERENCY_UNIT            (((uint64_t)1U)<<40)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_UTC_WRITE_UNIT3                    (((uint64_t)1U)<<39)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_UTC_WRITE_UNIT2                    (((uint64_t)1U)<<38)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_UTC_WRITE_UNIT1                    (((uint64_t)1U)<<37)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_UTC_WRITE_UNIT0                    (((uint64_t)1U)<<36)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_UTC_READ_UNIT3                     (((uint64_t)1U)<<35)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_UTC_READ_UNIT2                     (((uint64_t)1U)<<34)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_UTC_READ_UNIT1                     (((uint64_t)1U)<<33)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_UTC_READ_UNIT0                     (((uint64_t)1U)<<32)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_PACKET_DMA_UNIT3                (((uint64_t)1U)<<31)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_PACKET_DMA_UNIT2                (((uint64_t)1U)<<30)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_PACKET_DMA_UNIT1                (((uint64_t)1U)<<29)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_PACKET_DMA_UNIT0                (((uint64_t)1U)<<28)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_PACKET_DMA_UNIT3                (((uint64_t)1U)<<27)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_PACKET_DMA_UNIT2                (((uint64_t)1U)<<26)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_PACKET_DMA_UNIT1                (((uint64_t)1U)<<25)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_PACKET_DMA_UNIT0                (((uint64_t)1U)<<24)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_PREFETCH_UNIT3                  (((uint64_t)1U)<<23)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_PREFETCH_UNIT2                  (((uint64_t)1U)<<22)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_PREFETCH_UNIT1                  (((uint64_t)1U)<<21)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RX_PREFETCH_UNIT0                  (((uint64_t)1U)<<20)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_PREFETCH_UNIT3                  (((uint64_t)1U)<<19)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_PREFETCH_UNIT2                  (((uint64_t)1U)<<18)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_PREFETCH_UNIT1                  (((uint64_t)1U)<<17)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_TX_PREFETCH_UNIT0                  (((uint64_t)1U)<<16)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RSVD3                              (((uint64_t)1U)<<15)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_READ_DECODER2                      (((uint64_t)1U)<<14)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_READ_DECODER1                      (((uint64_t)1U)<<13)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_READ_DECODER0                      (((uint64_t)1U)<<12)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_WRITE_STATUS_DECODER3              (((uint64_t)1U)<<11)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RSVD2                              (((uint64_t)1U)<<10)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_WRITE_STATUS_DECODER1              (((uint64_t)1U)<<9)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_WRITE_STATUS_DECODER0              (((uint64_t)1U)<<8)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_WRITE_ARBITER3                     (((uint64_t)1U)<<7)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_RSVD1                              (((uint64_t)1U)<<6)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_WRITE_ARBITER1                     (((uint64_t)1U)<<5)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_WRITE_ARBITER0                     (((uint64_t)1U)<<4)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_COMMAND_ARBITER3                   (((uint64_t)1U)<<3)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_COMMAND_ARBITER2                   (((uint64_t)1U)<<2)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_COMMAND_ARBITER1                   (((uint64_t)1U)<<1)
#define CSL_UDMAP_AUTO_CLKGATE_BLOCK_COMMAND_ARBITER0                   (((uint64_t)1U)<<0)
/* @} */

/** ---------------------------------------------------------------------------
 *  @brief This enumerator defines the VBUSM master interfaces whose command
 *  dispatching threashold can be tuned via virtualization tuning registers.
 *
 *  UDMAP revision 2.1.32.0 and later provides support for setting command
 *  dispatching threasholds of UDMAP VBUSM master interfaces via
 *  virtualization tuning registers.
 *
 *  \anchor CSL_UdmapMasterInterface
 *  \name UDMAP VBUSM master interface identification
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_UdmapMasterInterface;
    /** Packet DMA Master Read/Write Interface 0 */
#define CSL_UDMAP_MASTER_INTERFACE_PKTDMA_0     ((uint32_t) 0U)
    /** Packet DMA Master Read/Write Interface 1 */
#define CSL_UDMAP_MASTER_INTERFACE_PKTDMA_1     ((uint32_t) 1U)
    /** Unified Transfer Controller (UTC) Master Read Interface */
#define CSL_UDMAP_MASTER_INTERFACE_UTC_READ     ((uint32_t) 2U)
    /** Unified Transfer Controller (UTC) Master Write Interface */
#define CSL_UDMAP_MASTER_INTERFACE_UTC_WRITE    ((uint32_t) 3U)
/* @} */

/* @} */

/**
 *  \addtogroup CSL_UDMAP_DATASTRUCT
 *  @{
 */

#define CSL_UDMAP_RXFDQ_CNT             (4U)
#define CSL_UDMAP_RXFDQ_THRESH_CNT      (4U)
#define CSL_UDMAP_NO_EVENT              (0xFFFFU)

/** \brief Receive free descriptor queue threshold information
 *
 *  This structure contains information describing a receive free descriptor
 *  queue threshold.
 *
 */
typedef struct
{
    uint32_t                fEnable;          /**< [IN]  If set, this threshold will be included in SOP FDQ selection */
    uint32_t                pktSize;          /**< [IN]  Packet size (in 32-byte units) used in SOP FDQ selection */
    uint32_t                queue;            /**< [IN]  Queue number to use if this threshold is selected */
} CSL_UdmapRxFdqThresh;

/** \brief Routing tag information
 *
 *  This structure contains information describing a routing tag.
 *
 */
typedef struct
{
    uint32_t                loSel;             /**< [IN]  Specifies how the low tag value is determined. See \ref CSL_UdmapTagSelect */
    uint8_t                 loVal;             /**< [IN]  Tag[7:0] low byte value (used if loSel == 1) */
    uint32_t                hiSel;             /**< [IN]  Specifies how the high tag value is determined. See \ref CSL_UdmapTagSelect */
    uint8_t                 hiVal;             /**< [IN]  Tag[7:0] high byte value (used if hiSel == 1) */
} CSL_UdmapRouteTag;

/** \brief Module revision information
 *
 *  This structure contains information describing the module revision.
 *
 */
typedef struct
{
    uint32_t                modId;              /**< [OUT]  Module ID */
    uint32_t                revRtl;             /**< [OUT]  RTL revision */
    uint32_t                revMajor;           /**< [OUT]  Major revision */
    uint32_t                custom;             /**< [OUT]  Custom revision */
    uint32_t                revMinor;           /**< [OUT]  Minor revision */
} CSL_UdmapRevision;

/** \brief Receive flow configuration information
 *
 *  This structure contains information describing a receive flow.
 *
 */
typedef struct
{
    uint32_t                einfoPresent;       /**< [IN]  Set to 1 if extended packet info is present in the descriptor */
    uint32_t                psInfoPresent;      /**< [IN]  Set to 1 if protocol-specific info is present in the descriptor */
    uint32_t                errorHandling;      /**< [IN]  Determines how starvation errors are handled. 0=drop packet, 1=retry */
    CSL_UdmapDescType       descType;           /**< [IN]  Descriptor type - see \ref CSL_UdmapDescType */
    CSL_UdmapPsLoc          psLocation;         /**< [IN]  Protocol-specific info location - see \ref CSL_UdmapPsLoc */
    uint32_t                sopOffset;          /**< [IN]  Start of rx packet data (byte offset from the start of the SOP buffer) */
    uint32_t                defaultRxCQ;        /**< [IN]  Rx destination queue */
    CSL_UdmapRouteTag       srcTag;             /**< [IN]  Source tag - see #CSL_UdmapRouteTag */
    CSL_UdmapRouteTag       dstTag;             /**< [IN]  Destination tag - see #CSL_UdmapRouteTag */
    CSL_UdmapRxFdqThresh    fdqThresh[CSL_UDMAP_RXFDQ_THRESH_CNT];  /**< [IN]  Free descriptor queue threshold information used for Start Of Packet (SOP) queue selection when packet size thresholds are enabled - see #CSL_UdmapRxFdqThresh */
    uint32_t                fdq[CSL_UDMAP_RXFDQ_CNT]; /**< [IN]  Free descriptor queue numbers. fdq[0] is used for the Start Of Packet (SOP) queue number when packet size thresholds are disabled. fdq[1..3] are used for subsequent Rest Of Packet queue numbers. */
} CSL_UdmapRxFlowCfg;

/** \brief Transmit channel configuration information
 *
 *  This structure contains configuration information for a transmit channel.
 *
 */
typedef struct
{
    uint32_t                pauseOnError;         /**< [IN] When set, pause channel on error */
    uint32_t                filterEinfo;          /**< [IN] When set, filter out extended info */
    uint32_t                filterPsWords;        /**< [IN] When set, filter out protocl specific words */
    CSL_UdmapAddrType       addrType;             /**< [IN] Address type for this channel */
    CSL_UdmapChanType       chanType;             /**< [IN] Channel type */
    uint32_t                fetchWordSize;        /**< [IN] Descriptor/TR Size in 32-bit words */
    uint32_t                trEventNum;           /**< [IN] Specifies a global event number to generate anytime the required event generation criteria specified in a TR are met (set to CSL_UDMAP_NO_EVENT for no event generation) */
    uint32_t                errEventNum;          /**< [IN] Specifies a global event number to generate anytime an error is encountered on the channel (set to CSL_UDMAP_NO_EVENT for no event generation) */
    uint32_t                busPriority;          /**< [IN] 3-bit priority value (0=highest, 7=lowest) */
    uint32_t                busQos;               /**< [IN] 3-bit qos value (0=highest, 7=lowest) */
    uint32_t                busOrderId;           /**< [IN] 4-bit orderid value */
    CSL_UdmapChanSchedPri   dmaPriority;          /**< [IN] This field selects which scheduling bin the channel will be placed in for bandwidth allocation of the Tx DMA units */
    uint32_t                txCredit;             /**< [IN] TX credit for external channels */
    uint32_t                txTrCQ;               /**< [IN] TX TR Completion Queue */
    uint32_t                txThread;             /**< [IN] TX mapped destination thread */
    bool                    bNoTeardownCompletePkt; /**< [IN] Specifies whether or not the channel should suppress sending the single data phase teardown packet when teardown is complete. 0 = TD packet is sent, 1 = Suppress sending TD packet */
    uint32_t                tdType;               /**< [IN] Specifies whether or not the channel should immediately return a teardown completion response to the default completion queue or wait until a status message is returned from the remote PSI-L paired peripheral. 0 = return immediately once all traffic is complete in UDMAP, 1 = wait until remote peer sends back a completion message. Valid in udmap version 2.0.0 and later. */
} CSL_UdmapTxChanCfg;

/** \brief Receive channel configuration information
 *
 *  This structure contains configuration information for a receive channel.
 *
 */
typedef struct
{
    uint32_t                pauseOnError;         /**< [IN] When set, pause channel on error */
    CSL_UdmapAddrType       addrType;             /**< [IN] Address type for this channel */
    CSL_UdmapChanType       chanType;             /**< [IN] Channel type */
    uint32_t                fetchWordSize;        /**< [IN] Descriptor/TR Size in 32-bit words */
    uint32_t                trEventNum;           /**< [IN] Specifies a global event number to generate anytime the required event generation criteria specified in a TR are met (set to CSL_UDMAP_NO_EVENT for no event generation) */
    uint32_t                errEventNum;          /**< [IN] Specifies a global event number to generate anytime an error is encountered on the channel (set to CSL_UDMAP_NO_EVENT for no event generation) */
    uint32_t                busPriority;          /**< [IN] 3-bit priority value (0=highest, 7=lowest) */
    uint32_t                busQos;               /**< [IN] 3-bit qos value (0=highest, 7=lowest) */
    uint32_t                busOrderId;           /**< [IN] 4-bit orderid value */
    uint32_t                rxTrCQ;               /**< [IN] RX TR Completion Queue */
    uint32_t                rxThread;             /**< [IN] Rx channel destination ThreadID mapping */
    uint32_t                flowIdFwRangeStart;   /**< [IN] Starting flow ID value for firewall check */
    uint32_t                flowIdFwRangeCnt;     /**< [IN] Number of valid flow ID's starting from flowIdFwRangeStart for firewall check */
    bool                    bIgnoreShortPkts;     /**< [IN] This field controls whether or not short packets will be treated as exceptions (false) or ignored (true) for the channel. This field is only used when the channel is in split UTC mode.*/
    bool                    bIgnoreLongPkts;      /**< [IN] This field controls whether or not long packets will be treated as exceptions (false) or ignored (true) for the channel. This field is only used when the channel is in split UTC mode.*/
    CSL_UdmapChanSchedPri   dmaPriority;          /**< [IN] This field selects which scheduling bin the channel will be placed in for bandwidth allocation of the Rx DMA units */
} CSL_UdmapRxChanCfg;

/** \brief Channel runtime configuration information
 *
 *  This structure contains runtime configuration information for a channel.
 *
 */
typedef struct
{
    uint32_t                enable;               /**< [IN]  When set, enable the channel */
    uint32_t                teardown;             /**< [IN]  When set, teardown the channel */
    uint32_t                pause;                /**< [IN]  When set, pause the channel */
    uint32_t                error;                /**< [OUT] When set, an error has been detected on the channel */
    uint32_t                forcedTeardown;       /**< [IN]  When set, a forced teardown will be performed on the channel. Note that teardown must also be set when setting forcedTeardown. */
} CSL_UdmapRT;

/** \brief UDMAP configuration structure
 *
 *  This structure contains configuration information for the UDMAP.
 *
 */
typedef struct
{
    CSL_udmap_gcfgRegs      *pGenCfgRegs;               /**< [IN]  Pointer to the general configuration registers */
    CSL_udmap_rxfcfgRegs    *pRxFlowCfgRegs;            /**< [IN]  Pointer to the rx flow configuration registers */
    CSL_udmap_txccfgRegs    *pTxChanCfgRegs;            /**< [IN]  Pointer to the tx channel configuration registers */
    CSL_udmap_rxccfgRegs    *pRxChanCfgRegs;            /**< [IN]  Pointer to the rx channel configuration registers */
    CSL_udmap_txcrtRegs     *pTxChanRtRegs;             /**< [IN]  Pointer to the tx channel real-time registers */
    CSL_udmap_rxcrtRegs     *pRxChanRtRegs;             /**< [IN]  Pointer to the rx channel real-time registers */
    uint32_t                cap0;                       /**< [OUT] Contains the contents of the Capabilities Register 0 (populated by the #CSL_udmapGetCfg function) */
    uint32_t                cap1;                       /**< [OUT] Contains the contents of the Capabilities Register 1 (populated by the #CSL_udmapGetCfg function) */
    uint32_t                txChanCnt;                  /**< [OUT] Tx channel count (populated by the #CSL_udmapGetCfg function) */
    uint32_t                rxChanCnt;                  /**< [OUT] Rx channel count (populated by the #CSL_udmapGetCfg function) */
    uint32_t                rxFlowCnt;                  /**< [OUT] Rx flow count (populated by the #CSL_udmapGetCfg function) */
    uint32_t                txExtUtcChanCnt;            /**< [OUT] Tx external UTC channel count (populated by the #CSL_udmapGetCfg function) */
    uint32_t                txHighCapacityChanCnt;      /**< [OUT] Tx external UTC channel count (populated by the #CSL_udmapGetCfg function) */
    uint32_t                txUltraHighCapacityChanCnt; /**< [OUT] Tx external UTC channel count (populated by the #CSL_udmapGetCfg function) */
} CSL_UdmapCfg;

/** \brief UDMAP receive flow id firewall status
 *
 *  This structure contains status information collected whenever the receive
 *  flow ID firewall detects a flow ID that is out of range for an incoming
 *  packet.
 *
 */
typedef struct
{
    uint32_t    flowId;                         /**< [OUT] The flow ID that was received on the trapped packet */
    uint32_t    chnIdx;                         /**< [OUT] The channel index on which the trapped packet was received */
} CSL_UdmapRxFlowIdFirewallStatus;

/** \brief Transmit / receive channel statistics
 *
 *  This structure contains statistics for transmit and receive channels.
 *
 */
typedef struct
{
    uint32_t    packetCnt;                      /**< [OUT] Current completed packet count for the channel */
    uint32_t    completedByteCnt;               /**< [OUT] Current completed payload byte count for the channel */
    uint32_t    startedByteCnt;                 /**< [OUT] Current started byte count for the channel */
} CSL_UdmapChanStats;

/* @} */

/**
 *  \addtogroup CSL_UDMAP_FUNCTION
 *  @{
 */

/**
 *  \brief Return revision of the UDMAP module
 *
 *  This function returns the contents of the UDMAP revision register.
 *  Consult the UDMAP module documentation for a description of the
 *  contents of the revision register.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *
 *  \return The 32-bit revision register is returned.
 */
extern uint32_t CSL_udmapGetRevision( const CSL_UdmapCfg *pCfg );

/**
 *  \brief Return revision information of the UDMAP module
 *
 *  This function returns revision information for the UDMAP module.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param pRev             [OUT]   Pointer to a #CSL_UdmapRevision structure where the revision information is returned
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapGetRevisionInfo( const CSL_UdmapCfg *pCfg, CSL_UdmapRevision *pRev );

/**
 *  \brief Return UDMAP configuration information
 *
 *  This function returns configuration and capability information for the
 *  UDMAP module. See the #CSL_UdmapCfg structure for details on the
 *  information that is returned.
 *
 *  \param pCfg             [IN/OUT]    Pointer to the UDMAP configuration structure
 *
 *  \return None
 */
extern void CSL_udmapGetCfg( CSL_UdmapCfg *pCfg );

/**
 *  \brief Initialize a #CSL_UdmapTxChanCfg structure
 *
 *  This function initializes the specified #CSL_UdmapTxChanCfg structure to
 *  known, safe values. Software then only needs to configure elements
 *  that are different than their initialized values prior to calling the
 *  #CSL_udmapTxChanCfg function.
 *
 *  All elements of the #CSL_UdmapTxChanCfg structure are initialized to zero
 *  except for the following:
 *
 *      chanType        = CSL_UDMAP_CHAN_TYPE_REF_PKT_RING;
 *      fetchWordSize   = CSL_UDMAP_FETCH_WORD_SIZE_16;
 *      trEventNum      = CSL_UDMAP_NO_EVENT;
 *      errEventNum     = CSL_UDMAP_NO_EVENT;
 *
 *  \param pTxChanCfg   [OUT]   Pointer to a #CSL_UdmapTxChanCfg structure
 *
 *  \return None
 */
extern void CSL_udmapInitTxChanCfg( CSL_UdmapTxChanCfg *pTxChanCfg );

/**
 *  \brief Initialize a #CSL_UdmapRxChanCfg structure
 *
 *  This function initializes the specified #CSL_UdmapRxChanCfg structure to
 *  known, safe values. Software then only needs to configure elements
 *  that are different than their initialized values prior to calling the
 *  #CSL_udmapRxChanCfg function.
 *
 *  All elements of the #CSL_UdmapRxChanCfg structure are initialized to zero
 *  except for the following:
 *
 *      chanType            = CSL_UDMAP_CHAN_TYPE_REF_PKT_RING;
 *      fetchWordSize       = CSL_UDMAP_FETCH_WORD_SIZE_16;
 *      trEventNum          = CSL_UDMAP_NO_EVENT;
 *      errEventNum         = CSL_UDMAP_NO_EVENT;
 *      flowIdFwRangeCnt    = CSL_UDMAP_RXCCFG_CHAN_RFLOW_RNG_FLOWID_CNT_RESETVAL;
 *
 *  \param pRxChanCfg   [OUT]   Pointer to a #CSL_UdmapRxChanCfg structure
 *
 *  \return None
 */
extern void CSL_udmapInitRxChanCfg( CSL_UdmapRxChanCfg *pRxChanCfg );

/**
 *  \brief Initialize a CSL_UdmapRxFlowCfg structure
 *
 *  This function initializes the specified CSL_UdmapRxFlowCfg structure to
 *  known, safe values. Software then only needs to configure elements
 *  that are different than their initialized values prior to calling the
 *  CSL_udmapRxFlowCfg function.
 *
 *  All elements of the CSL_UdmapRxFlowCfg structure are initialized to zero.
 *
 *  \param pFlow        [OUT]   Pointer to a #CSL_UdmapRxFlowCfg structure
 *
 *  \return None
 */
extern void CSL_udmapInitRxFlowCfg( CSL_UdmapRxFlowCfg *pFlow );

/**
 *  \brief Set performance control parmeters
 *
 *  This function is used to set performance control paramaters available
 *  in the UDMAP module.
 *
 *  \param pCfg                 [IN]    Pointer to the UDMAP configuration structure
 *  \param rxRetryTimeoutCnt    [IN]    This parameter specifies the minimum
 *      amount of time (in clock cycles) that an Rx channel will be required
 *      to wait when it encounters a buffer starvation condition and the Rx
 *      error handling bit is set to 1
 *
 *  \return None
 */
extern void CSL_udmapSetPerfCtrl( CSL_UdmapCfg *pCfg, uint32_t rxRetryTimeoutCnt );

/**
 *  \brief Set UTC control parmeters
 *
 *  This function is used to set UTC control paramaters available
 *  in the UDMAP module.
 *
 *  \param pCfg                 [IN]    Pointer to the UDMAP configuration structure
 *  \param startingThreadNum    [IN]    This parameter specifies the starting
 *      PSI-L thread number for the external UTC
 *
 *  \return None
 */
extern void CSL_udmapSetUtcCtrl( CSL_UdmapCfg *pCfg, uint32_t startingThreadNum );

/**
 *  \brief Configure an RX flow
 *
 *  This function initializes a receive flow with values specified in the
 *  #CSL_UdmapRxFlowCfg structure.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param flow             [IN]    Index of the receive flow to initialize
 *  \param pFlow            [IN]    Pointer to a #CSL_UdmapRxFlowCfg structure containing initialization values
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapRxFlowCfg( CSL_UdmapCfg *pCfg, uint32_t flow, const CSL_UdmapRxFlowCfg *pFlow );

/**
 *  \brief Configure an RX channel
 *
 *  This function initializes a receive channel with values specified in the
 *  #CSL_UdmapRxChanCfg structure.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel to initialize
 *  \param pRxChanCfg       [IN]    Pointer to a #CSL_UdmapRxChanCfg structure containing initialization values
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapRxChanCfg( CSL_UdmapCfg *pCfg, uint32_t chanIdx, const CSL_UdmapRxChanCfg *pRxChanCfg );

/**
 *  \brief Configure a TX channel
 *
 *  This function initializes a transmit channel with values specified in the
 *  #CSL_UdmapTxChanCfg structure.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel to initialize
 *  \param pTxChanCfg       [IN]    Pointer to a #CSL_UdmapTxChanCfg structure containing initialization values
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapTxChanCfg( CSL_UdmapCfg *pCfg, uint32_t chanIdx, const CSL_UdmapTxChanCfg *pTxChanCfg );

/**
 *  \brief Configure an RX channel TR event
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel to initialize
 *  \param trEventNum       [IN]    Specifies a global event number to generate
 *                                  anytime the required event generation
 *                                  criteria specified in a TR are met
 *                                  Set to CSL_UDMAP_NO_EVENT for no event
 *                                  generation.
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapRxChanSetTrEvent( CSL_UdmapCfg *pCfg, uint32_t chanIdx, uint32_t trEventNum );

/**
 *  \brief Configure an TX channel TR event
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel to initialize
 *  \param trEventNum       [IN]    Specifies a global event number to generate
 *                                  anytime the required event generation
 *                                  criteria specified in a TR are met
 *                                  Set to CSL_UDMAP_NO_EVENT for no event
 *                                  generation.
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapTxChanSetTrEvent( CSL_UdmapCfg *pCfg, uint32_t chanIdx, uint32_t trEventNum );

/**
 *  \brief Configure RX channel burst size
 *
 *  This function enables configuration of the nominal burst size and alignment
 *  for data transfers on the specified RX channel. The default burst size
 *  is 64 bytes (a value of CSL_UDMAP_CHAN_BURST_SIZE_64_BYTES).
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel to initialize
 *  \param burstSize        [IN]    Burst size value. See \ref CSL_UdmapChanBurstSize
 *                                  for a list of valid burst size values.
 *
 *  \return 0 if successful, or -1 if burstSize is invalid or this function is
 *  not available in the version of UDMAP being used
 */
extern int32_t CSL_udmapRxChanSetBurstSize( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanBurstSize burstSize );

/**
 *  \brief Configure TX channel burst size
 *
 *  This function enables configuration of the nominal burst size and alignment
 *  for data transfers on the specified TX channel. The default burst size
 *  is 64 bytes (a value of CSL_UDMAP_CHAN_BURST_SIZE_64_BYTES).
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel to initialize
 *  \param burstSize        [IN]    Burst size value. See \ref CSL_UdmapChanBurstSize
 *                                  for a list of valid burst size values.
 *
 *  \return 0 if successful, or -1 if burstSize is invalid or this function is
 *  not available in the version of UDMAP being used
 */
extern int32_t CSL_udmapTxChanSetBurstSize( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanBurstSize burstSize );

/**
 *  \brief Get an RX channel's real-time register values
 *
 *  This function returns the real-time register values for the specified
 *  receive channel.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel
 *  \param pRT              [OUT]   Pointer to a #CSL_UdmapRT structure where values are returned
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapGetRxRT( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapRT *pRT );

/**
 *  \brief Get a TX channel's real-time register values
 *
 *  This function returns the real-time register values for the specified
 *  transmit channel.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel
 *  \param pRT              [OUT]   Pointer to a #CSL_UdmapRT structure where values are returned
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapGetTxRT( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapRT *pRT );

/**
 *  \brief Set an RX channel's real-time register values
 *
 *  This function sets the real-time register values for the specified
 *  receive channel.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel
 *  \param pRT              [IN]    Pointer to a #CSL_UdmapRT structure containing initialization values
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapSetRxRT( CSL_UdmapCfg *pCfg, uint32_t chanIdx, const CSL_UdmapRT *pRT );

/**
 *  \brief Set a TX channel's real-time register values
 *
 *  This function sets the real-time register values for the specified
 *  transmit channel.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel
 *  \param pRT              [IN]    Pointer to a #CSL_UdmapRT structure containing initialization values
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapSetTxRT( CSL_UdmapCfg *pCfg, uint32_t chanIdx, const CSL_UdmapRT *pRT );

/**
 *  \brief Enable a transmit channel.
 *
 *  This function enables the transmit channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapEnableTxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Disable a transmit channel.
 *
 *  This function disables the transmit channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapDisableTxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Teardown a transmit channel.
 *
 *  This function tears down the transmit channel specified by 'chanIdx' at the
 *  next packet boundary.
 *
 *  \param pCfg     [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *  \param bForce   [IN]    If true, the channel is torn down without attempting
 *                          to preserve data or wait for events (flushes the
 *                          channel). If false, any packets in flight are
 *                          completed prior to the channel being torn down.
 *  \param bWait    [IN]    If true, wait for the teardown operation to complete.
 *                          Otherwise, return immediately.
 *
 *  \return 0 = Success, -1 = Operation prohibited (channel is disabled)
 */
extern int32_t CSL_udmapTeardownTxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx, bool bForce, bool bWait );

/**
 *  \brief Pause a transmit channel.
 *
 *  This function pauses the transmit channel specified by 'chanIdx' at the
 *  next packet boundary. This is a more graceful method of halting processing
 *  than disabling the channel as it will not allow any current packets to
 *  underflow.
 *
 *  \param pCfg     [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return 0 = Success, -1 = Operation prohibited (channel is disabled)
 */
extern int32_t CSL_udmapPauseTxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Un-pause a transmit channel.
 *
 *  This function un-pauses the transmit channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return 0 = Success, -1 = Operation prohibited (channel is disabled)
 */
extern int32_t CSL_udmapUnpauseTxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Send a trigger event to a TX channel
 *
 *  This function causes a trigger event to be sent to the specified transmit
 *  channel.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapTriggerTxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Clear error indication in a transmit channel.
 *
 *  This function clears the error indication in the specified transmit channel.
 *
 *  \param pCfg     [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return None
 */
extern void CSL_udmapClearTxChanError( CSL_UdmapCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Enable a receive channel.
 *
 *  This function enables the receive channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapEnableRxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Disable a receive channel.
 *
 *  This function disables the receive channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapDisableRxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Teardown a receive channel.
 *
 *  This function tears down the receive channel specified by 'chanIdx' at the
 *  next packet boundary.
 *
 *  \param pCfg     [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *  \param bForce   [IN]    If true, the channel is torn down without attempting
 *                          to preserve data or wait for events (flushes the
 *                          channel). If false, any packets in flight are
 *                          completed prior to the channel being torn down.
 *  \param bWait    [IN]    If true, wait for the teardown operation to complete.
 *                          Otherwise, return immediately.
 *
 *  \return 0 = Success, -1 = Operation prohibited (channel is disabled)
 */
extern int32_t CSL_udmapTeardownRxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx, bool bForce, bool bWait );

/**
 *  \brief Pause a receive channel.
 *
 *  This function pauses the receive channel specified by 'chanIdx' at the
 *  next packet boundary. This is a more graceful method of halting processing
 *  than disabling the channel as it will not allow any current packets to
 *  underflow.
 *
 *  \param pCfg     [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *
 *  \return 0 = Success, -1 = Operation prohibited (channel is disabled)
 */
extern int32_t CSL_udmapPauseRxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Un-pause a receive channel.
 *
 *  This function un-pauses the receive channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return 0 = Success, -1 = Operation prohibited (channel is disabled)
 */
extern int32_t CSL_udmapUnpauseRxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Send a trigger event to an RX channel
 *
 *  This function causes a trigger event to be sent to the specified receive
 *  channel.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel
 *
 *  \return 0 if successful, or -1 on error
 */
extern int32_t CSL_udmapTriggerRxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Clear error indication in a receive channel.
 *
 *  This function clears the error indication in the specified receive channel.
 *
 *  \param pCfg     [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *
 *  \return None
 */
extern void CSL_udmapClearRxChanError( CSL_UdmapCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Configure the receive flow ID range firewall
 *
 *  This function is used to configure the receive flow ID range firewall.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param outEvtNum        [IN]    Output event number to use when the receive
 *                                  flow ID range firewall detects an error
 *
 *  \return None
 */
extern void CSL_udmapCfgRxFlowIdFirewall( CSL_UdmapCfg *pCfg, uint32_t outEvtNum );

/**
 *  \brief Get receive flow ID range firewall status information
 *
 *  This function returns information from the receive flow ID range firewall.
 *
 *  If the receive flow ID firewall has detected an out of range flow ID,
 *  the function returns true and the fields within the
 *  #CSL_UdmapRxFlowIdFirewallStatus structure contain error details. The
 *  function will automatically reset the receive flow ID firewall to capture
 *  the next error.
 *
 *  If the receive flow ID firewall has not detected an out of range flow ID,
 *  the function returns false and the fields within the
 *  #CSL_UdmapRxFlowIdFirewallStatus structure are not updated.
 *
 *  \param pCfg                 [IN]    Pointer to the UDMAP configuration structure
 *  \param pRxFlowIdFwStatus    [IN]    Pointer to a #CSL_UdmapRxFlowIdFirewallStatus
 *                                      structure containing error details (valid
 *                                      only when true is returned)
 *
 *  \return true if the receive flow ID range firewall has detected an out of range
 *          flow ID, false if no error was detected
 */
extern bool CSL_udmapGetRxFlowIdFirewallStatus( CSL_UdmapCfg *pCfg, CSL_UdmapRxFlowIdFirewallStatus *pRxFlowIdFwStatus );

/**
 *  \brief Get channel statistics
 *
 *  This function is used to read statistics for a transmit or receive channel.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the channel
 *  \param chanDir          [IN]    Channel direction (see \ref CSL_UdmapChanDir)
 *  \param pChanStats       [OUT]   Pointer to a #CSL_UdmapChanStats structure
 *                                  where the statistics are returned
 *  \return None
 */
extern void CSL_udmapGetChanStats( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, CSL_UdmapChanStats *pChanStats );

/**
 *  \brief Decrement channel statistics
 *
 *  This function is used to decrement statistics for a transmit or receive channel
 *  by the counts contained in the specified #CSL_UdmapChanStats structure.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the channel
 *  \param chanDir          [IN]    Channel direction (see \ref CSL_UdmapChanDir)
 *  \param pChanStats       [IN]    Pointer to a #CSL_UdmapChanStats structure
 *                                  containing the counts to decrement each
 *                                  statistic by
 *  \return None
 */
extern void CSL_udmapDecChanStats( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, const CSL_UdmapChanStats *pChanStats );

/**
 *  \brief Read a channel peer register
 *
 *  This function is used to read the value from a peer register for the
 *  specified channel.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the channel
 *  \param chanDir          [IN]    Channel direction (see \ref CSL_UdmapChanDir)
 *  \param regIdx           [IN]    Peer register index (0..15)
 *  \param pVal             [OUT]   Pointer where the register value is returned
 *
 *  \return 0 if successful, or -1 if regIdx is out of range
 */
extern int32_t CSL_udmapGetChanPeerReg( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, uint32_t regIdx, uint32_t *pVal );

/**
 *  \brief Write a TX channel peer register
 *
 *  This function is used to write a value to a peer register for the
 *  specified transmit channel.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel
 *  \param chanDir          [IN]    Channel direction (see \ref CSL_UdmapChanDir)
 *  \param regIdx           [IN]    Peer register index (0..15)
 *  \param pVal             [IN]    Pointer to the register value to be written
 *
 *  \return 0 if successful, or -1 if regIdx is out of range
 */
extern int32_t CSL_udmapSetChanPeerReg( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, uint32_t regIdx, uint32_t *pVal );

/**
 *  \brief Enable a directional data flow for a paired link
 *
 *  This function is used to enable a directional data flow for a given UDMAP channel between
 *  UDMAP and a paired PSIL peer.
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param chanIdx          [IN]    Index of the UDMAP channel (TX or RX)
 *  \param chanDir          [IN]    Channel direction (TX or RX, see \ref CSL_UdmapChanDir)
 *
 *  \return 0 if successful, or -1 if chanIdx is invalid
 */
extern int32_t CSL_udmapEnableLink( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir );

/**
 *  \brief Enable or disable automatic dynamic clock gating for one or more blocks
 *
 *  This function is used to enable or disable automatic dynamic clock gating
 *  for one or more internal sub-blocks within the UDMAP.
 *
 *  UDMAP revision 2.2.0.0 and later provides automatic dynamic clock gating
 *  support for internal sub-blocks based on real-time monitoring of activity. 
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param blockIds         [IN]    One or more block ID's created by logically
 *                                  or'ing one or more ID's defined in
 *                                  \ref CSL_UdmapAutoClkgateBlock
 *  \param bEnable          [IN]    If true, automatic dynamic clock gating for
 *                                  the specified blocks is enabled. Otherwise,
 *                                  it is disabled.
 *
 *  \return 0 if successful, or -1 if this feature is not available
 */
extern int32_t CSL_udmapSetAutoClockGatingEnable( CSL_UdmapCfg *pCfg, CSL_UdmapAutoClkgateBlock blockIds, bool bEnable );

/**
 *  \brief Set the command throttle threshold for a UDMAP VBUSM master interface
 *
 *  This function is used to set the read and/or write command throttle
 *  threshold for a UDMAP VBUSM master interface. Command dispatching will be
 *  disabled for virtualized channels whenver the current virtualized read or
 *  write count from this interface meets or exceeds the programmed value.
 *
 *  Note that this feature is only available in UDMAP revision 2.1.32.0 and
 *  later.
 *
 *  The following table shows if read and/or write thresholds can be set for
 *  each available interface:
 *
 *                                        Read  Write
 *                                        ----  -----
 *  CSL_UDMAP_MASTER_INTERFACE_PKTDMA_0     X     X
 *  CSL_UDMAP_MASTER_INTERFACE_PKTDMA_1     X     X
 *  CSL_UDMAP_MASTER_INTERFACE_UTC_READ     X
 *  CSL_UDMAP_MASTER_INTERFACE_UTC_WRITE          X
 *
 *  \param pCfg             [IN]    Pointer to the UDMAP configuration structure
 *  \param interfaceId      [IN]    Interface ID. See \ref CSL_UdmapMasterInterface
 *  \param readCountThresh  [IN]    Read command threshold value. Must be > 0.
 *  \param writeCountThresh [IN]    Write command threshold value. Must be > 0.
 *
 *  \return 0 if successful
 *          -1 if this feature is not available
 *          -2 interfaceId, readCountThresh, and/or writeCountThresh are invalid
 */
extern int32_t CSL_udmapSetCommandThrottleThreshold( CSL_UdmapCfg *pCfg, CSL_UdmapMasterInterface interfaceId, uint32_t readCountThresh, uint32_t writeCountThresh );

/* @} */

#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif  /* end of CSL_UDMAP_H_ definition */
/** @} */

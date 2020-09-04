/*
 *  Copyright (C) 2019-2020 Texas Instruments Incorporated.
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
 *  \file  csl_bcdma.h
 *
 *  \brief
 *  This CSL-FL header file contains various enumerations, structure
 *  definitions and function declarations for the Block Copy DMA (BCDMA) IP.
 *
 *  This CSL-FL was designed to be orthogonal with the implementation
 *  of the udmap CSL-FL. Enumerations, structure definitions, and API functions
 *  are similarly named. The include file "csl_bcdma_alias_udmap_api.h" is
 *  available which maps bcdma CSL-FL content to their udmap equivalents for
 *  ease in porting existing udmap code to the bcdma.
 *
 *  Implementation notes:
 *  o The bcdma CSL-FL provides implementations of all udmap API functions.
 *  o block_copy and split_tx channels use tx flavors of API functions
 *    (i.e. CSL_bcdmaEnableTxChan)
 *  o split_rx channels use rx flavors of API functions (i.e. CSL_bcdmaEnableRxChan)
 *  o Channel numbers provided to API functions are as follows. :
 *    - block_copy: 0 .. (bcChanCnt-1)                                         (i.e. 0..27)
 *    - split_tx:   bcChanCnt .. (bcChanCnt+txChanCnt-1)                       (i.e. 28..47)
 *    - split_rx:   (bcChanCnt+txChanCnt) .. (bcChanCnt+txChanCnt+rxChanCnt-1) (i.e. 48..67)
 *
 *  There is CSL-FL content that is applicable for udmap but not for bcdma.
 *  Those items are denoted with the tag [udmap_only] in the comments below
 *  and are handled as follows:
 *    - Enumerations: The enumeration is defined, but is not used functionally
 *    - Structure definitions: The structure is defined, but is not used
 *      functionally
 *    - Structure elements: The element is defined in the structure, but
 *      is not used functionally
 *    - API functions: The function is implemented, but does no operations
 *
 *  There is CSL-FL content that is applicable for bcdma but not for
 *  udmap. Those items are denoted with the tag [bcdma_only]
 *  in the comments below.
 *
 *  The following is the required calling sequence in order to insure proper
 *  operation of this CSL-FL:
 *
 *    1. Allocate a BCDMA configuration structure (#CSL_BcdmaCfg)
 *    2. (Optional) Call #CSL_bcdmaInitCfg to initialize the BCDMA
 *       configuration structure to all zeros
 *    3. Initialize the register structure pointer elements of the BCDMA
 *       configuration structure
 *    4. Call #CSL_bcdmaGetCfg to populate the BCDMA configuration structure
 *       with configuration and capability information for the BCDMA module
 *    5. Call the other BCDMA API functions as required
 */
/**
 *  \ingroup CSL_IP_MODULE
 *  \defgroup CSL_BCDMA BCDMA CSL-FL
 *
 *  @{
 */

#ifndef CSL_BCDMA_H_
#define CSL_BCDMA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/hw_include/cslr_bcdma.h>
#include <drivers/udma/include/csl_udmap_tr.h>
#include <drivers/hw_include/csl_types.h>

#define CSL_BCDMA_FETCH_WORD_SIZE_16    (16U)
#define CSL_BCDMA_NO_EVENT              (0xFFFFU)

/**
@defgroup CSL_BCDMA_DATASTRUCT  BCDMA Data Structures
@ingroup CSL_BCDMA_API
*/
/**
@defgroup CSL_BCDMA_FUNCTION  BCDMA Functions
@ingroup CSL_BCDMA_API
*/
/**
@defgroup CSL_BCDMA_ENUM BCDMA Enumerated Data Types
@ingroup CSL_BCDMA_API
*/

/**
 *  \addtogroup CSL_BCDMA_ENUM
 *  @{
 */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible channel operations
 *
 *  \anchor CSL_BcdmaChanOp
 *  \name BCDMA channel operations
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaChanOp;
    /** Configure channel */
#define CSL_BCDMA_CHAN_OP_CONFIG                ((uint32_t) 0U)
    /** Enable channel */
#define CSL_BCDMA_CHAN_OP_ENABLE                ((uint32_t) 1U)
    /** Disable channel */
#define CSL_BCDMA_CHAN_OP_DISABLE               ((uint32_t) 2U)
    /** Pause channel */
#define CSL_BCDMA_CHAN_OP_PAUSE                 ((uint32_t) 3U)
    /** Resume channel */
#define CSL_BCDMA_CHAN_OP_RESUME                ((uint32_t) 4U)
    /** Teardown channel */
#define CSL_BCDMA_CHAN_OP_TEARDOWN              ((uint32_t) 5U)
    /** Trigger channel */
#define CSL_BCDMA_CHAN_OP_TRIGGER               ((uint32_t) 6U)
    /** Get channel real-time values */
#define CSL_BCDMA_CHAN_OP_GET_RT                ((uint32_t) 7U)
    /** Set channel real-time values */
#define CSL_BCDMA_CHAN_OP_SET_RT                ((uint32_t) 8U)
    /** Get channel statistics  */
#define CSL_BCDMA_CHAN_OP_GET_STATS             ((uint32_t) 9U)
    /** Decrement channel statistics */
#define CSL_BCDMA_CHAN_OP_DEC_STATS             ((uint32_t) 10U)
    /** Get (read) remote peer register */
#define CSL_BCDMA_CHAN_OP_GET_REMOTE_PEER_REG   ((uint32_t) 11U)
    /** Set (write) remote peer register */
#define CSL_BCDMA_CHAN_OP_SET_REMOTE_PEER_REG   ((uint32_t) 12U)
    /** Set (write) channel burst size */
#define CSL_BCDMA_CHAN_OP_SET_BURST_SIZE        ((uint32_t) 13U)
    /** Clear channel error */
#define CSL_BCDMA_CHAN_OP_CLEAR_ERROR           ((uint32_t) 14U)

/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible channel directions
 *
 *  \anchor CSL_BcdmaChanDir
 *  \name BCDMA channel direction
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaChanDir;
    /** Transmit direction */
#define CSL_BCDMA_CHAN_DIR_TX   ((uint32_t) 0U)
    /** Receive direction */
#define CSL_BCDMA_CHAN_DIR_RX   ((uint32_t) 1U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the nominal burst size and alignment for
 * data transfers on a TX or RX channel
 *
 *  \anchor CSL_BcdmaChanBurstSize
 *  \name BCDMA channel burst size
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaChanBurstSize;
    /** 32-byte burst size */
#define CSL_BCDMA_CHAN_BURST_SIZE_32_BYTES  ((uint32_t) 0U)
    /** 64-byte burst size */
#define CSL_BCDMA_CHAN_BURST_SIZE_64_BYTES  ((uint32_t) 1U)
    /** 128-byte burst size */
#define CSL_BCDMA_CHAN_BURST_SIZE_128_BYTES ((uint32_t) 2U)
    /** 256-byte burst size */
#define CSL_BCDMA_CHAN_BURST_SIZE_256_BYTES ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief [udmap_only] This enumerator defines the possible descriptor types
 *
 *  \anchor CSL_BcdmaDescType
 *  \name BCDMA descriptor type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaDescType;
    /** Host */
#define CSL_BCDMA_DESC_TYPE_HOST        ((uint32_t) 0U)
    /** Host single-buffer */
#define CSL_BCDMA_DESC_TYPE_HOST_SB     ((uint32_t) 1U)
    /** Monolithic */
#define CSL_BCDMA_DESC_TYPE_MONOLITHIC  ((uint32_t) 2U)
    /** Reserved */
#define CSL_BCDMA_DESC_TYPE_RESERVED    ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief [udmap_only] This enumerator defines the ps location for the descriptor
 *
 *  \anchor CSL_BcdmaPsLoc
 *  \name BCDMA protocol-specific data location
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaPsLoc;
    /** Located in descriptor */
#define CSL_BCDMA_PS_LOC_DESC       ((uint32_t) 0U)
    /** Located in packet */
#define CSL_BCDMA_PS_LOC_PACKET     ((uint32_t) 1U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible address types
 *
 *  \anchor CSL_BcdmaAddrType
 *  \name BCDMA address type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaAddrType;
    /** Physical addressing */
#define CSL_BCDMA_ADDR_TYPE_PHYS    ((uint32_t) 0U)
    /** Intermediate addressing */
#define CSL_BCDMA_ADDR_TYPE_INTER   ((uint32_t) 1U)
    /** Virtual addressing */
#define CSL_BCDMA_ADDR_TYPE_VIRT    ((uint32_t) 2U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible channel types
 *
 *  \anchor CSL_BcdmaChanType
 *  \name BCDMA channel type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaChanType;
    /** Block-copy channel */
#define CSL_BCDMA_CHAN_TYPE_BLOCK_COPY          ((uint32_t) 0U)
    /** Split transmit channel */
#define CSL_BCDMA_CHAN_TYPE_SPLIT_TX            ((uint32_t) 1U)
    /** Split receive channel */
#define CSL_BCDMA_CHAN_TYPE_SPLIT_RX            ((uint32_t) 2U)

    /** RM, Packet Mode, Pass by reference */
#define CSL_BCDMA_CHAN_TYPE_REF_PKT_RING        ((uint32_t) 2U)
    /** QM, Packet Single Buffer Mode, Pass by reference */
#define CSL_BCDMA_CHAN_TYPE_REF_PKTSB_QUEUE     ((uint32_t) 3U)
    /** RM, TR Mode, Pass by reference */
#define CSL_BCDMA_CHAN_TYPE_REF_TR_RING         ((uint32_t) 10U)
    /** RM, TR Mode, Direct 'value' mode */
#define CSL_BCDMA_CHAN_TYPE_VAL_TR_RING         ((uint32_t) 11U)
    /** RM, TR Mode, Block Copy, Pass by reference */
#define CSL_BCDMA_CHAN_TYPE_COPY_REF_TR_RING    ((uint32_t) 12U)
    /** RM, TR Mode, Block Copy, Direct 'value' mode */
#define CSL_BCDMA_CHAN_TYPE_COPY_VAL_TR_RING    ((uint32_t) 13U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines how tag values are determined
 *
 *  \anchor CSL_BcdmaTagSelect
 *  \name BCDMA tag select
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTagSelect;
    /** Do not overwrite */
#define CSL_BCDMA_TAG_SELECT_NO_OVERWRITE               ((uint32_t) 0U)
    /** Overwrite with value given in tag value */
#define CSL_BCDMA_TAG_SELECT_OVERWRITE_WITH_VAL         ((uint32_t) 1U)
    /** Overwrite with flow_id[7:0] from back-end */
#define CSL_BCDMA_TAG_SELECT_OVERWRITE_WITH_FLOWID_7_0  ((uint32_t) 2U)
    /** Overwrite with flow_id[15:8] from back-end */
#define CSL_BCDMA_TAG_SELECT_OVERWRITE_WITH_FLOWID_15_8 ((uint32_t) 3U)
    /** Overwrite with tag[7:0] from back-end */
#define CSL_BCDMA_TAG_SELECT_OVERWRITE_WITH_TAG_7_0     ((uint32_t) 4U)
    /** Overwrite with tag[15:8] from back-end */
#define CSL_BCDMA_TAG_SELECT_OVERWRITE_WITH_TAG_15_8    ((uint32_t) 5U)
    /** Invalid */
#define CSL_BCDMA_TAG_SELECT_INVALID                    ((uint32_t) 6U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator selects which scheduling bin the channel will be
 *  placed in for bandwidth allocation of the DMA units
 *
 *  \anchor CSL_BcdmaChanSchedPri
 *  \name BCDMA channel schedling priority
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaChanSchedPri;
    /** High priority */
#define CSL_BCDMA_CHAN_SCHED_PRI_HIGH       ((uint32_t) 0U)
    /** Medium-High priority */
#define CSL_BCDMA_CHAN_SCHED_PRI_MED_HIGH   ((uint32_t) 1U)
    /** Medium-Low priority */
#define CSL_BCDMA_CHAN_SCHED_PRI_MED_LOW    ((uint32_t) 2U)
    /** Low priority */
#define CSL_BCDMA_CHAN_SCHED_PRI_LOW        ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator selects the descriptor type. The bcdma only supports
 *        the TR descriptor type.
 *
 *  \anchor CSL_BcdmaDescriptorType
 *  \name BCDMA descriptor type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaDescriptorType;
#define CSL_BCDMA_DESCRIPTOR_TYPE_TR        ((uint32_t) 3U)
/* @} */


/* @} */

/**
 *  \addtogroup CSL_BCDMA_DATASTRUCT
 *  @{
 */

#define CSL_BCDMA_RXFDQ_CNT             (4U)
#define CSL_BCDMA_RXFDQ_THRESH_CNT      (4U)
#define CSL_BCDMA_NO_EVENT              (0xFFFFU)

/** \brief [udmap_only] Receive free descriptor queue threshold information
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
} CSL_BcdmaRxFdqThresh;

/** \brief [udmap_only] Routing tag information
 *
 *  This structure contains information describing a routing tag.
 *
 */
typedef struct
{
    uint32_t                loSel;             /**< [IN]  Specifies how the low tag value is determined. See \ref CSL_BcdmaTagSelect */
    uint8_t                 loVal;             /**< [IN]  Tag[7:0] low byte value (used if loSel == 1) */
    uint32_t                hiSel;             /**< [IN]  Specifies how the high tag value is determined. See \ref CSL_BcdmaTagSelect */
    uint8_t                 hiVal;             /**< [IN]  Tag[7:0] high byte value (used if hiSel == 1) */
} CSL_BcdmaRouteTag;

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
} CSL_BcdmaRevision;

/** \brief [udmap_only] Receive flow configuration information
 *
 *  This structure contains information describing a receive flow.
 *
 */
typedef struct
{
    uint32_t                einfoPresent;       /**< [IN]  Set to 1 if extended packet info is present in the descriptor */
    uint32_t                psInfoPresent;      /**< [IN]  Set to 1 if protocol-specific info is present in the descriptor */
    uint32_t                errorHandling;      /**< [IN]  Determines how starvation errors are handled. 0=drop packet, 1=retry */
    CSL_BcdmaDescType       descType;           /**< [IN]  Descriptor type - see \ref CSL_BcdmaDescType */
    CSL_BcdmaPsLoc          psLocation;         /**< [IN]  Protocol-specific info location - see \ref CSL_BcdmaPsLoc */
    uint32_t                sopOffset;          /**< [IN]  Start of rx packet data (byte offset from the start of the SOP buffer) */
    uint32_t                defaultRxCQ;        /**< [IN]  Rx destination queue */
    CSL_BcdmaRouteTag       srcTag;             /**< [IN]  Source tag - see #CSL_BcdmaRouteTag */
    CSL_BcdmaRouteTag       dstTag;             /**< [IN]  Destination tag - see #CSL_BcdmaRouteTag */
    CSL_BcdmaRxFdqThresh    fdqThresh[CSL_BCDMA_RXFDQ_THRESH_CNT];  /**< [IN]  Free descriptor queue threshold information used for Start Of Packet (SOP) queue selection when packet size thresholds are enabled - see #CSL_BcdmaRxFdqThresh */
    uint32_t                fdq[CSL_BCDMA_RXFDQ_CNT]; /**< [IN]  Free descriptor queue numbers. fdq[0] is used for the Start Of Packet (SOP) queue number when packet size thresholds are disabled. fdq[1..3] are used for subsequent Rest Of Packet queue numbers. */
} CSL_BcdmaRxFlowCfg;

/** \brief Transmit channel configuration information
 *
 *  This structure contains configuration information for a transmit channel.
 *
 */
typedef struct
{
    uint32_t                pauseOnError;         /**< [IN] When set, pause channel on error */
    uint32_t                filterEinfo;          /**< [udmap_only] [IN] When set, filter out extended info */
    uint32_t                filterPsWords;        /**< [udmap_only] [IN] When set, filter out protocl specific words */
    CSL_BcdmaAddrType       addrType;             /**< [udmap_only] [IN] Address type for this channel */
    CSL_BcdmaChanType       chanType;             /**< [IN] Channel type */
    uint32_t                fetchWordSize;        /**< [IN] Descriptor/TR Size in 32-bit words */
    uint32_t                trEventNum;           /**< [udmap_only] [IN] Specifies a global event number to generate anytime the required event generation criteria specified in a TR are met (set to CSL_BCDMA_NO_EVENT for no event generation) */
    uint32_t                errEventNum;          /**< [udmap_only] [IN] Specifies a global event number to generate anytime an error is encountered on the channel (set to CSL_BCDMA_NO_EVENT for no event generation) */
    uint32_t                busPriority;          /**< [IN] 3-bit priority value (0=highest, 7=lowest) */
    uint32_t                busQos;               /**< [udmap_only] [IN] 3-bit qos value (0=highest, 7=lowest) */
    uint32_t                busOrderId;           /**< [IN] 4-bit orderid value */
    CSL_BcdmaChanSchedPri   dmaPriority;          /**< [IN] This field selects which scheduling bin the channel will be placed in for bandwidth allocation of the Tx DMA units */
    uint32_t                txCredit;             /**< [udmap_only] [IN] TX credit for external channels */
    uint32_t                txTrCQ;               /**< [udmap_only] [IN] TX TR Completion Queue */
    uint32_t                txThread;             /**< [udmap_only] [IN] TX mapped destination thread */
    bool                    bNoTeardownCompletePkt; /**< [IN] Specifies whether or not the channel should suppress sending the single data phase teardown packet when teardown is complete. 0 = TD packet is sent, 1 = Suppress sending TD packet */
    uint32_t                tdType;               /**< [IN] Specifies whether or not the channel should immediately return a teardown completion response to the default completion queue or wait until a status message is returned from the remote PSI-L paired peripheral. 0 = return immediately once all traffic is complete in BCDMA, 1 = wait until remote peer sends back a completion message. Valid in bcdma version 2.0.0 and later. */
    CSL_BcdmaChanBurstSize  burstSize;            /**< [bcdma_only] [IN] Encoded nominal burst size and alignment for data transfers on this channel (block copy supports 0=32, 1=64, 2=128, split-tx supports 0=32, 1=64) */
    uint32_t                threadId;             /**< [bcdma_only] [IN] This field contains the (up-to) 16-bit value which will be output on the strm_o_thread_id output during all transactions for this channel */
} CSL_BcdmaTxChanCfg;

/** \brief Receive channel configuration information
 *
 *  This structure contains configuration information for a receive channel.
 *
 */
typedef struct
{
    uint32_t                pauseOnError;         /**< [IN] When set, pause channel on error */
    CSL_BcdmaAddrType       addrType;             /**< [udmap_only] [IN] Address type for this channel */
    CSL_BcdmaChanType       chanType;             /**< [IN] Channel type */
    uint32_t                fetchWordSize;        /**< [IN] Descriptor/TR Size in 32-bit words */
    uint32_t                trEventNum;           /**< [udmap_only] [IN] Specifies a global event number to generate anytime the required event generation criteria specified in a TR are met (set to CSL_BCDMA_NO_EVENT for no event generation) */
    uint32_t                errEventNum;          /**< [udmap_only] [IN] Specifies a global event number to generate anytime an error is encountered on the channel (set to CSL_BCDMA_NO_EVENT for no event generation) */
    uint32_t                busPriority;          /**< [IN] 3-bit priority value (0=highest, 7=lowest) */
    uint32_t                busQos;               /**< [udmap_only] [IN] 3-bit qos value (0=highest, 7=lowest) */
    uint32_t                busOrderId;           /**< [IN] 4-bit orderid value */
    uint32_t                rxTrCQ;               /**< [udmap_only] [IN] RX TR Completion Queue */
    uint32_t                rxThread;             /**< [udmap_only] [IN] Rx channel destination ThreadID mapping */
    uint32_t                flowIdFwRangeStart;   /**< [udmap_only] [IN] Starting flow ID value for firewall check */
    uint32_t                flowIdFwRangeCnt;     /**< [udmap_only] [IN] Number of valid flow ID's starting from flowIdFwRangeStart for firewall check */
    bool                    bIgnoreShortPkts;     /**< [udmap_only] [IN] This field controls whether or not short packets will be treated as exceptions (false) or ignored (true) for the channel. This field is only used when the channel is in split UTC mode.*/
    bool                    bIgnoreLongPkts;      /**< [IN] This field controls whether or not long packets will be treated as exceptions (false) or ignored (true) for the channel. This field is only used when the channel is in split UTC mode.*/
    CSL_BcdmaChanSchedPri   dmaPriority;          /**< [IN] This field selects which scheduling bin the channel will be placed in for bandwidth allocation of the Rx DMA units */
    CSL_BcdmaChanBurstSize  burstSize;            /**< [bcdma_only] [IN] Encoded nominal burst size and alignment for data transfers on this channel (split-rx supports 0=32, 1=64) */
    uint32_t                threadId;             /**< [bcdma_only] [IN] This field contains the (up-to) 16-bit value which will be output on the strm_o_thread_id output during all transactions for this channel */
} CSL_BcdmaRxChanCfg;

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
    uint32_t                starvation;           /**< [OUT] Rx starvation (valid for Split-Rx channels only). This bit is set if the channel receives a packet and the ring is empty. The bit clears when the doorbell is written with a positive value. */
} CSL_BcdmaRT;

/** \brief BCDMA configuration structure
 *
 *  This structure contains configuration information for the BCDMA.
 *
 */
typedef struct
{
    CSL_bcdma_gcfgRegs      *pGenCfgRegs;               /**< [IN] Pointer to general configuration registers */
    CSL_bcdma_bccfgRegs     *pBcChanCfgRegs;            /**< [IN] Pointer to block copy channel configuration registers */
    CSL_bcdma_bcrtRegs      *pBcChanRtRegs;             /**< [IN] Pointer to block copy channel real-time registers */
    CSL_bcdma_txccfgRegs    *pTxChanCfgRegs;            /**< [IN] Pointer to tx channel configuration registers */
    CSL_bcdma_txcrtRegs     *pTxChanRtRegs;             /**< [IN] Pointer to tx channel real-time registers */
    CSL_bcdma_rxccfgRegs    *pRxChanCfgRegs;            /**< [IN] Pointer to rx channel configuration registers */
    CSL_bcdma_rxcrtRegs     *pRxChanRtRegs;             /**< [IN] Pointer to rx channel real-time registers */
    uint32_t                cap0;                       /**< [IN/OUT] Capabilities 0 register contents (populated by the #CSL_bcdmaGetCfg function) */
    uint32_t                cap1;                       /**< [IN/OUT] Capabilities 1 register contents (populated by the #CSL_bcdmaGetCfg function) */
    uint32_t                bcChanCnt;                  /**< [IN/OUT] Number of block-copy DMA channels (populated by the #CSL_bcdmaGetCfg function) */
    uint32_t                splitRxChanCnt;             /**< [IN/OUT] Number of split receive DMA channels (populated by the #CSL_bcdmaGetCfg function) */
    uint32_t                splitTxChanCnt;             /**< [IN/OUT] Number of split transmit DMA channels (populated by the #CSL_bcdmaGetCfg function) */
    uint32_t                flowCnt;                    /**< [udmap_only] [OUT] Total flow table entry count (populated by the #CSL_bcdmaGetCfg function) */
    uint32_t                txChanCnt;                  /**< [udmap_only] [OUT] Tx channel count (populated by the #CSL_bcdmaGetCfg function) */
    uint32_t                rxChanCnt;                  /**< [udmap_only] [OUT] Rx channel count (populated by the #CSL_bcdmaGetCfg function) */
    uint32_t                rxFlowCnt;                  /**< [udmap_only] [OUT] Rx flow count (populated by the #CSL_bcdmaGetCfg function) */
    uint32_t                txExtUtcChanCnt;            /**< [udmap_only] [OUT] Tx external UTC channel count (populated by the #CSL_bcdmaGetCfg function) */
    uint32_t                txHighCapacityChanCnt;      /**< [udmap_only] [OUT] Tx external UTC channel count (populated by the #CSL_bcdmaGetCfg function) */
    uint32_t                txUltraHighCapacityChanCnt; /**< [udmap_only] [OUT] Tx external UTC channel count (populated by the #CSL_bcdmaGetCfg function) */
} CSL_BcdmaCfg;

/** \brief BCDMA receive flow id firewall status
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
} CSL_BcdmaRxFlowIdFirewallStatus;

/** \brief Channel teardown options
 *
 *  This structure contains channel teardown options.
 *
 */
typedef struct
{
    uint32_t force;                               /**< [IN]  When non-zero, perform a forced teardown */
    uint32_t wait;                                /**< [IN]  When non-zero, wait for the teardown operation to complete */
} CSL_BcdmaTeardownOpts;

/** \brief Remote peer register read/write options
 *
 *  This structure contains remote peer register read/write options.
 *
 */
typedef struct
{
    uint32_t regIdx;            /**< [IN]     Register index to read or write */
    uint32_t regVal;            /**< [IN/OUT] Register value read or written */
} CSL_BcdmaRemotePeerOpts;

/** \brief Transmit / receive channel statistics
 *
 *  This structure contains statistics for transmit and receive channels.
 *
 */
typedef struct
{
    uint32_t    packetCnt;                      /**< [OUT] Current completed packet count for the channel */
    uint32_t    txPayloadByteCnt;               /**< [OUT] Current completed transmit payload byte count for the channel */
    uint32_t    txStartedByteCnt;               /**< [OUT] Current started transmit byte count for the channel */
    uint32_t    rxPayloadByteCnt;               /**< [OUT] Current completed receive payload byte count for the channel */
    uint32_t    rxStartedByteCnt;               /**< [OUT] Current started receive byte count for the channel */
    uint32_t    completedByteCnt;               /**< [OUT] Current completed payload byte count for the channel */
    uint32_t    startedByteCnt;                 /**< [OUT] Current started byte count for the channel */
} CSL_BcdmaChanStats;

/*-----------------------------------------------------------------------------
 * TR Descriptor
 *---------------------------------------------------------------------------*/
/** \brief TR Descriptor
 *
 *  This structure contains TR descriptor information.
 *
 */
typedef struct
{
  uint32_t      descInfo;       /**< word 0: TR Descriptor Info */
  uint32_t      pktInfo;        /**< word 1: TR Packet Info */
  uint32_t      retInfo;        /**< word 2: TR Return Info */
  uint32_t      srcDstTag;      /**< word 3: Sourc/Dest Tag */
} CSL_BcdmaTRPD;

/*-----------------------------------------------------------------------------
 * TR Descriptor field manipulation macros
 *---------------------------------------------------------------------------*/
#define CSL_BCDMA_TRPD_DESCINFO_DTYPE_SHIFT           ((uint32_t) 30U)
#define CSL_BCDMA_TRPD_DESCINFO_DTYPE_MASK            (((uint32_t) 0x3U) << CSL_BCDMA_TRPD_DESCINFO_DTYPE_SHIFT)
#define CSL_BCDMA_TRPD_DESCINFO_RELOAD_SHIFT          ((uint32_t) 20U)
#define CSL_BCDMA_TRPD_DESCINFO_RELOAD_MASK           (((uint32_t) 0x1FFU) << CSL_BCDMA_TRPD_DESCINFO_RELOAD_SHIFT)
#define CSL_BCDMA_TRPD_DESCINFO_RLDIDX_SHIFT          ((uint32_t) 14U)
#define CSL_BCDMA_TRPD_DESCINFO_RLDIDX_MASK           (((uint32_t) 0x3FU) << CSL_BCDMA_TRPD_DESCINFO_RLDIDX_SHIFT)
#define CSL_BCDMA_TRPD_DESCINFO_LASTIDX_SHIFT         ((uint32_t) 0)
#define CSL_BCDMA_TRPD_DESCINFO_LASTIDX_MASK          (((uint32_t) 0x3FFFU) << CSL_BCDMA_TRPD_DESCINFO_LASTIDX_SHIFT)

#define CSL_BCDMA_TRPD_PKTINFO_RECSIZE_SHIFT          ((uint32_t) 24U)
#define CSL_BCDMA_TRPD_PKTINFO_RECSIZE_MASK           (((uint32_t) 0x7U) << CSL_BCDMA_TRPD_PKTINFO_RECSIZE_SHIFT)
#define   CSL_BCDMA_TRPD_PKTINFO_RECSIZE_VAL_16B          ((uint32_t) 0)
#define   CSL_BCDMA_TRPD_PKTINFO_RECSIZE_VAL_32B          ((uint32_t) 1U)
#define   CSL_BCDMA_TRPD_PKTINFO_RECSIZE_VAL_64B          ((uint32_t) 2U)
#define   CSL_BCDMA_TRPD_PKTINFO_RECSIZE_VAL_128B         ((uint32_t) 3U)

/* @} */

/**
 *  \addtogroup CSL_BCDMA_FUNCTION
 *  @{
 */

/**
 *  \brief Return revision of the BCDMA module
 *
 *  This function returns the contents of the BCDMA revision register.
 *  Consult the BCDMA module documentation for a description of the
 *  contents of the revision register.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *
 *  \return The 32-bit revision register is returned.
 */
extern uint32_t CSL_bcdmaGetRevision( const CSL_BcdmaCfg *pCfg );

/**
 *  \brief Return revision information of the BCDMA module
 *
 *  This function returns revision information for the BCDMA module.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param pRev             [OUT]   Pointer to a #CSL_BcdmaRevision structure where the revision information is returned
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaGetRevisionInfo( const CSL_BcdmaCfg *pCfg, CSL_BcdmaRevision *pRev );

/**
 *  \brief Initialize contents of a BCDMA configuration structure
 *
 *  This function initializes the contents of the specified BCDMA configuration
 *  structure.
 *
 *  All elements of the #CSL_BcdmaCfg structure are initialized to zero.
 *
 *  This function should not be called after calling CSL_bcdmaGetCfg as the
 *  BCDMA module configuration stored in the BCDMA configuration structure
 *  will be overwritten with zeros.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *
 *  \return None
 */
extern void CSL_bcdmaInitCfg( CSL_BcdmaCfg *pCfg );

/**
 *  \brief Return BCDMA configuration information
 *
 *  This function populates the BCDMA configuration structure with
 *  configuration and capability information for the BCDMA module. This
 *  information is needed by other BCDMA API functions.
 *
 *  See the #CSL_BcdmaCfg structure for details on the information that is
 *  returned.
 *
 *  This function must be called before calling any other BCDMA API function
 *  except for the #CSL_bcdmaInitCfg function. Note that the register pointer
 *  elements in the BCDMA configuration structure must be initialized by the
 *  caller prior to calling this function.
 *
 *  \param pCfg             [IN/OUT]    Pointer to the BCDMA configuration structure
 *
 *  \return None
 */
extern void CSL_bcdmaGetCfg( CSL_BcdmaCfg *pCfg );

/**
 *  \brief Initialize a #CSL_BcdmaTxChanCfg structure
 *
 *  This function initializes the specified #CSL_BcdmaTxChanCfg structure to
 *  known, safe values. Software then only needs to configure elements
 *  that are different than their initialized values prior to calling the
 *  #CSL_bcdmaTxChanCfg function.
 *
 *  All elements of the #CSL_BcdmaTxChanCfg structure are initialized to zero
 *  except for the following:
 *
 *      chanType        = CSL_BCDMA_CHAN_TYPE_REF_PKT_RING;
 *      fetchWordSize   = CSL_BCDMA_FETCH_WORD_SIZE_16;
 *      trEventNum      = CSL_BCDMA_NO_EVENT;
 *      errEventNum     = CSL_BCDMA_NO_EVENT;
 *
 *  \param pTxChanCfg   [OUT]   Pointer to a #CSL_BcdmaTxChanCfg structure
 *
 *  \return None
 */
extern void CSL_bcdmaInitTxChanCfg( CSL_BcdmaTxChanCfg *pTxChanCfg );

/**
 *  \brief Initialize a #CSL_BcdmaRxChanCfg structure
 *
 *  This function initializes the specified #CSL_BcdmaRxChanCfg structure to
 *  known, safe values. Software then only needs to configure elements
 *  that are different than their initialized values prior to calling the
 *  #CSL_bcdmaRxChanCfg function.
 *
 *  All elements of the #CSL_BcdmaRxChanCfg structure are initialized to zero
 *  except for the following:
 *
 *      chanType            = CSL_BCDMA_CHAN_TYPE_REF_PKT_RING;
 *      fetchWordSize       = CSL_BCDMA_FETCH_WORD_SIZE_16;
 *      trEventNum          = CSL_BCDMA_NO_EVENT;
 *      errEventNum         = CSL_BCDMA_NO_EVENT;
 *      flowIdFwRangeCnt    = CSL_BCDMA_RXCCFG_CHAN_RFLOW_RNG_FLOWID_CNT_RESETVAL;
 *
 *  \param pRxChanCfg   [OUT]   Pointer to a #CSL_BcdmaRxChanCfg structure
 *
 *  \return None
 */
extern void CSL_bcdmaInitRxChanCfg( CSL_BcdmaRxChanCfg *pRxChanCfg );

/**
 *  \brief Initialize a CSL_BcdmaRxFlowCfg structure
 *
 *  This function initializes the specified CSL_BcdmaRxFlowCfg structure to
 *  known, safe values. Software then only needs to configure elements
 *  that are different than their initialized values prior to calling the
 *  CSL_bcdmaRxFlowCfg function.
 *
 *  All elements of the CSL_BcdmaRxFlowCfg structure are initialized to zero.
 *
 *  \param pFlow        [OUT]   Pointer to a #CSL_BcdmaRxFlowCfg structure
 *
 *  \return None
 */
extern void CSL_bcdmaInitRxFlowCfg( CSL_BcdmaRxFlowCfg *pFlow );

/**
 *  \brief Perform a channel operation
 *
 *  This function performs the operation specified by 'chanOp' on the channel
 *  specified by channel type 'chanType' and index 'chanIdx'. Any operation-
 *  specific input parameters or output values are provided in the structure
 *  pointed to by 'pOpData'.
 *
 *  The following table describes the valid channel operations and the
 *  structure type to be passed in 'pOpData':
 *
 *  'chanOp'                                Description                     'pOpData'
 *  --------------------------------------  ------------------------------  -----------------------------------------------------------------
 *  CSL_BCDMA_CHAN_OP_CONFIG                Configure channel               CSL_BcdmaTxChanCfg ('chanType' == CSL_BCDMA_CHAN_TYPE_BLOCK_COPY)
 *                                                                          CSL_BcdmaTxChanCfg ('chanType' == CSL_BCDMA_CHAN_TYPE_SPLIT_TX)
 *                                                                          CSL_BcdmaRxChanCfg ('chanType' == CSL_BCDMA_CHAN_TYPE_SPLIT_RX)
 *  CSL_BCDMA_CHAN_OP_ENABLE                Enable channel                  N/A
 *  CSL_BCDMA_CHAN_OP_DISABLE               Disable channel                 N/A
 *  CSL_BCDMA_CHAN_OP_PAUSE                 Pause channel                   N/A
 *  CSL_BCDMA_CHAN_OP_RESUME                Resume channel                  N/A
 *  CSL_BCDMA_CHAN_OP_TEARDOWN              Teardown channel                CSL_BcdmaTeardownOpts (optional)
 *    Notes:
 *      - Channel can be torn down only when it is enabled
 *      - pOpData (CSL_BcdmaTeardownOpts) is optional. If NULL, then force and wait CSL_BcdmaTeardownOpts fields are assumed 0.
 *  CSL_BCDMA_CHAN_OP_TRIGGER               Trigger channel                 N/A
 *    Notes: This operation is valid only for 'chanType' == CSL_BCDMA_CHAN_TYPE_BLOCK_COPY
 *  CSL_BCDMA_CHAN_OP_GET_RT                Get channel real-time values    CSL_BcdmaRT
 *  CSL_BCDMA_CHAN_OP_SET_RT                Set channel real-time values    CSL_BcdmaRT
 *  CSL_BCDMA_CHAN_OP_GET_STATS             Get channel statistics          CSL_BcdmaChanStats
 *  CSL_BCDMA_CHAN_OP_DEC_STATS             Decrement channel statistics    CSL_BcdmaChanStats
 *  CSL_BCDMA_CHAN_OP_GET_REMOTE_PEER_REG   Get remote peer reg value       CSL_BcdmaRemotePeerOpts
 *  CSL_BCDMA_CHAN_OP_SET_REMOTE_PEER_REG   Set remote peer reg value       CSL_BcdmaRemotePeerOpts
 *  CSL_BCDMA_CHAN_OP_SET_BURST_SIZE        Set channel burst size          CSL_BcdmaChanBurstSize
 *  CSL_BCDMA_CHAN_OP_CLEAR_ERROR           Clear channel error             N/A
 *
 *  \param pCfg         [IN]        Pointer to the BCDMA configuration structure
 *  \param chanOp       [IN]        Channel operation. See #CSL_BcdmaChanOp.
 *  \param chanType     [IN]        Channel type. See #CSL_BcdmaChanType.
 *  \param chanIdx      [IN]        Zero-based channel index
 *  \param pOpData      [IN/OUT]    Pointer to optional operation-specific structure
 *
 *  \return CSL_PASS = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 *          CSL_EBADARGS = One or more arguments are invalid
 *          CSL_EINVALID_PARAMS = One or more parameters in 'pOpData' are invalid
 */
extern int32_t CSL_bcdmaChanOp( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanOp chanOp, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData );

/**
 *  \brief Set performance control parmeters
 *
 *  This function is used to set performance control paramaters available
 *  in the BCDMA module.
 *
 *  \param pCfg                 [IN]    Pointer to the BCDMA configuration structure
 *  \param rxRetryTimeoutCnt    [IN]    This parameter specifies the minimum
 *      amount of time (in clock cycles) that an Rx channel will be required
 *      to wait when it encounters a buffer starvation condition and the Rx
 *      error handling bit is set to 1
 *
 *  \return None
 */
extern void CSL_bcdmaSetPerfCtrl( CSL_BcdmaCfg *pCfg, uint32_t rxRetryTimeoutCnt );

/**
 *  \brief [udmap_only] Set UTC control parmeters
 *
 *  This function is used to set UTC control paramaters available
 *  in the BCDMA module.
 *
 *  \param pCfg                 [IN]    Pointer to the BCDMA configuration structure
 *  \param startingThreadNum    [IN]    This parameter specifies the starting
 *      PSI-L thread number for the external UTC
 *
 *  \return None
 */
extern void CSL_bcdmaSetUtcCtrl( CSL_BcdmaCfg *pCfg, uint32_t startingThreadNum );

/**
 *  \brief [udmap_only] Configure an RX flow
 *
 *  This function initializes a receive flow with values specified in the
 *  #CSL_BcdmaRxFlowCfg structure.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param flow             [IN]    Index of the receive flow to initialize
 *  \param pFlow            [IN]    Pointer to a #CSL_BcdmaRxFlowCfg structure containing initialization values
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
*/
extern int32_t CSL_bcdmaRxFlowCfg( CSL_BcdmaCfg *pCfg, uint32_t flow, const CSL_BcdmaRxFlowCfg *pFlow );

/**
 *  \brief Configure an RX channel
 *
 *  This function initializes a receive channel with values specified in the
 *  #CSL_BcdmaRxChanCfg structure.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel to initialize
 *  \param pRxChanCfg       [IN]    Pointer to a #CSL_BcdmaRxChanCfg structure containing initialization values
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaRxChanCfg( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, const CSL_BcdmaRxChanCfg *pRxChanCfg );

/**
 *  \brief Configure a TX channel
 *
 *  This function initializes a transmit channel with values specified in the
 *  #CSL_BcdmaTxChanCfg structure.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel to initialize
 *  \param pTxChanCfg       [IN]    Pointer to a #CSL_BcdmaTxChanCfg structure containing initialization values
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaTxChanCfg( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, const CSL_BcdmaTxChanCfg *pTxChanCfg );

/**
 *  \brief [udmap_only] Configure an RX channel TR event
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel to initialize
 *  \param trEventNum       [IN]    Specifies a global event number to generate
 *                                  anytime the required event generation
 *                                  criteria specified in a TR are met
 *                                  Set to CSL_BCDMA_NO_EVENT for no event
 *                                  generation.
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaRxChanSetTrEvent( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, uint32_t trEventNum );

/**
 *  \brief [udmap_only] Configure an TX channel TR event
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel to initialize
 *  \param trEventNum       [IN]    Specifies a global event number to generate
 *                                  anytime the required event generation
 *                                  criteria specified in a TR are met
 *                                  Set to CSL_BCDMA_NO_EVENT for no event
 *                                  generation.
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaTxChanSetTrEvent( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, uint32_t trEventNum );

/**
 *  \brief Configure RX channel burst size
 *
 *  This function enables configuration of the nominal burst size and alignment
 *  for data transfers on the specified RX channel. The default burst size
 *  is 64 bytes (a value of CSL_BCDMA_CHAN_BURST_SIZE_64_BYTES).
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel to initialize
 *  \param burstSize        [IN]    Burst size value. See \ref CSL_BcdmaChanBurstSize
 *                                  for a list of valid burst size values.
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (burstSize is invalid or this
 *                      function is not available in the version of BCDMA being used)
 */
extern int32_t CSL_bcdmaRxChanSetBurstSize( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanBurstSize burstSize );

/**
 *  \brief Configure TX channel burst size
 *
 *  This function enables configuration of the nominal burst size and alignment
 *  for data transfers on the specified TX channel. The default burst size
 *  is 64 bytes (a value of CSL_BCDMA_CHAN_BURST_SIZE_64_BYTES).
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel to initialize
 *  \param burstSize        [IN]    Burst size value. See \ref CSL_BcdmaChanBurstSize
 *                                  for a list of valid burst size values.
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (burstSize is invalid or this
 *                      function is not available in the version of BCDMA being used)
 */
extern int32_t CSL_bcdmaTxChanSetBurstSize( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanBurstSize burstSize );

/**
 *  \brief Get an RX channel's real-time register values
 *
 *  This function returns the real-time register values for the specified
 *  receive channel.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel
 *  \param pRT              [OUT]   Pointer to a #CSL_BcdmaRT structure where values are returned
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaGetRxRT( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaRT *pRT );

/**
 *  \brief Get a TX channel's real-time register values
 *
 *  This function returns the real-time register values for the specified
 *  transmit channel.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel
 *  \param pRT              [OUT]   Pointer to a #CSL_BcdmaRT structure where values are returned
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaGetTxRT( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaRT *pRT );

/**
 *  \brief Set an RX channel's real-time register values
 *
 *  This function sets the real-time register values for the specified
 *  receive channel.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel
 *  \param pRT              [IN]    Pointer to a #CSL_BcdmaRT structure containing initialization values
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaSetRxRT( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, const CSL_BcdmaRT *pRT );

/**
 *  \brief Set a TX channel's real-time register values
 *
 *  This function sets the real-time register values for the specified
 *  transmit channel.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel
 *  \param pRT              [IN]    Pointer to a #CSL_BcdmaRT structure containing initialization values
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaSetTxRT( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, const CSL_BcdmaRT *pRT );

/**
 *  \brief Enable a transmit channel.
 *
 *  This function enables the transmit channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaEnableTxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Disable a transmit channel.
 *
 *  This function disables the transmit channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaDisableTxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Teardown a transmit channel.
 *
 *  This function tears down the transmit channel specified by 'chanIdx' at the
 *  next packet boundary.
 *
 *  \param pCfg     [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *  \param bForce   [IN]    If true, the channel is torn down without attempting
 *                          to preserve data or wait for events (flushes the
 *                          channel). If false, any packets in flight are
 *                          completed prior to the channel being torn down.
 *  \param bWait    [IN]    If true, wait for the teardown operation to complete.
 *                          Otherwise, return immediately.
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (channel is disabled)
 */
extern int32_t CSL_bcdmaTeardownTxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, bool bForce, bool bWait );

/**
 *  \brief Pause a transmit channel.
 *
 *  This function pauses the transmit channel specified by 'chanIdx' at the
 *  next packet boundary. This is a more graceful method of halting processing
 *  than disabling the channel as it will not allow any current packets to
 *  underflow.
 *
 *  \param pCfg     [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (channel is disabled)
 */
extern int32_t CSL_bcdmaPauseTxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Un-pause a transmit channel.
 *
 *  This function un-pauses the transmit channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (channel is disabled)
 */
extern int32_t CSL_bcdmaUnpauseTxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Send a trigger event to a TX channel
 *
 *  This function causes a trigger event to be sent to the specified transmit
 *  channel.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaTriggerTxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Clear error indication in a transmit channel.
 *
 *  This function clears the error indication in the specified transmit channel.
 *
 *  \param pCfg     [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return None
 */
extern void CSL_bcdmaClearTxChanError( CSL_BcdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Enable a receive channel.
 *
 *  This function enables the receive channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaEnableRxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Disable a receive channel.
 *
 *  This function disables the receive channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaDisableRxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Teardown a receive channel.
 *
 *  This function tears down the receive channel specified by 'chanIdx' at the
 *  next packet boundary.
 *
 *  \param pCfg     [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *  \param bForce   [IN]    If true, the channel is torn down without attempting
 *                          to preserve data or wait for events (flushes the
 *                          channel). If false, any packets in flight are
 *                          completed prior to the channel being torn down.
 *  \param bWait    [IN]    If true, wait for the teardown operation to complete.
 *                          Otherwise, return immediately.
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (channel is disabled)
 */
extern int32_t CSL_bcdmaTeardownRxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, bool bForce, bool bWait );

/**
 *  \brief Pause a receive channel.
 *
 *  This function pauses the receive channel specified by 'chanIdx' at the
 *  next packet boundary. This is a more graceful method of halting processing
 *  than disabling the channel as it will not allow any current packets to
 *  underflow.
 *
 *  \param pCfg     [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (channel is disabled)
 */
extern int32_t CSL_bcdmaPauseRxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Un-pause a receive channel.
 *
 *  This function un-pauses the receive channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (channel is disabled)
 */
extern int32_t CSL_bcdmaUnpauseRxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Send a trigger event to an RX channel
 *
 *  This function causes a trigger event to be sent to the specified receive
 *  channel.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_bcdmaTriggerRxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Clear error indication in a receive channel.
 *
 *  This function clears the error indication in the specified receive channel.
 *
 *  \param pCfg     [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *
 *  \return None
 */
extern void CSL_bcdmaClearRxChanError( CSL_BcdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief [udmap_only] Configure the receive flow ID range firewall
 *
 *  This function is used to configure the receive flow ID range firewall.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param outEvtNum        [IN]    Output event number to use when the receive
 *                                  flow ID range firewall detects an error
 *
 *  \return None
 */
extern void CSL_bcdmaCfgRxFlowIdFirewall( CSL_BcdmaCfg *pCfg, uint32_t outEvtNum );

/**
 *  \brief [udmap_only] Get receive flow ID range firewall status information
 *
 *  This function returns information from the receive flow ID range firewall.
 *
 *  If the receive flow ID firewall has detected an out of range flow ID,
 *  the function returns true and the fields within the
 *  #CSL_BcdmaRxFlowIdFirewallStatus structure contain error details. The
 *  function will automatically reset the receive flow ID firewall to capture
 *  the next error.
 *
 *  If the receive flow ID firewall has not detected an out of range flow ID,
 *  the function returns false and the fields within the
 *  #CSL_BcdmaRxFlowIdFirewallStatus structure are not updated.
 *
 *  \param pCfg                 [IN]    Pointer to the BCDMA configuration structure
 *  \param pRxFlowIdFwStatus    [IN]    Pointer to a #CSL_BcdmaRxFlowIdFirewallStatus
 *                                      structure containing error details (valid
 *                                      only when true is returned)
 *
 *  \return true if the receive flow ID range firewall has detected an out of range
 *          flow ID, false if no error was detected
 */
extern bool CSL_bcdmaGetRxFlowIdFirewallStatus( CSL_BcdmaCfg *pCfg, CSL_BcdmaRxFlowIdFirewallStatus *pRxFlowIdFwStatus );

/**
 *  \brief Get channel statistics
 *
 *  This function is used to read statistics for a transmit or receive channel.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the channel
 *  \param chanDir          [IN]    Channel direction (see \ref CSL_BcdmaChanDir)
 *  \param pChanStats       [OUT]   Pointer to a #CSL_BcdmaChanStats structure
 *                                  where the statistics are returned
 *  \return None
 */
extern void CSL_bcdmaGetChanStats( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanDir chanDir, CSL_BcdmaChanStats *pChanStats );

/**
 *  \brief Decrement channel statistics
 *
 *  This function is used to decrement statistics for a transmit or receive channel
 *  by the counts contained in the specified #CSL_BcdmaChanStats structure.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the channel
 *  \param chanDir          [IN]    Channel direction (see \ref CSL_BcdmaChanDir)
 *  \param pChanStats       [IN]    Pointer to a #CSL_BcdmaChanStats structure
 *                                  containing the counts to decrement each
 *                                  statistic by
 *  \return None
 */
extern void CSL_bcdmaDecChanStats( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanDir chanDir, const CSL_BcdmaChanStats *pChanStats );

/**
 *  \brief Read a channel peer register
 *
 *  This function is used to read the value from a peer register for the
 *  specified channel.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the channel
 *  \param chanDir          [IN]    Channel direction (see \ref CSL_BcdmaChanDir)
 *  \param regIdx           [IN]    Peer register index (0..15)
 *  \param pVal             [OUT]   Pointer where the register value is returned
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (regIdx is out of range)
 */
extern int32_t CSL_bcdmaGetChanPeerReg( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanDir chanDir, uint32_t regIdx, uint32_t *pVal );

/**
 *  \brief Write a TX channel peer register
 *
 *  This function is used to write a value to a peer register for the
 *  specified transmit channel.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel
 *  \param chanDir          [IN]    Channel direction (see \ref CSL_BcdmaChanDir)
 *  \param regIdx           [IN]    Peer register index (0..15)
 *  \param pVal             [IN]    Pointer to the register value to be written
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (regIdx is out of range)
 */
extern int32_t CSL_bcdmaSetChanPeerReg( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanDir chanDir, uint32_t regIdx, uint32_t *pVal );

/**
 *  \brief Enable a directional data flow for a paired link
 *
 *  This function is used to enable a directional data flow for a given BCDMA channel between
 *  BCDMA and a paired PSIL peer.
 *
 *  \param pCfg             [IN]    Pointer to the BCDMA configuration structure
 *  \param chanIdx          [IN]    Index of the BCDMA channel (TX or RX)
 *  \param chanDir          [IN]    Channel direction (TX or RX, see \ref CSL_BcdmaChanDir)
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (chanIdx is invalid)
 */
extern int32_t CSL_bcdmaEnableLink( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanDir chanDir );

/* @} */

/*=============================================================================
 *  Transfer Request (TR) support
 *===========================================================================*/

/**
 *  \addtogroup CSL_BCDMA_DATASTRUCT
 *  @{
 */

/** \brief CSL_BcdmaTR specifies a Transfer Request. */
struct CSL_BcdmaTR_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint16_t icnt2;     /**< Total loop iteration count for level 2 */
    uint16_t icnt3;     /**< Total loop iteration count for level 3 (outermost) */
    int32_t  dim2;      /**< Signed dimension for loop level 2 */
    int32_t  dim3;      /**< Signed dimension for loop level 3 */
    uint32_t fmtflags;  /**< [udmap_only] Tells how the data is formatted either between the input and the output or if the data should use different addressing schemes or sizes */
    int32_t  ddim1;     /**< Signed dimension for loop level 1 for the destination data */
    uint64_t daddr;     /**< Starting address for the destination of the data */
    int32_t  ddim2;     /**< Signed dimension for loop level 2 for the destination data */
    int32_t  ddim3;     /**< Signed dimension for loop level 3 for the destination data */
    uint16_t dicnt0;    /**< Total loop iteration count for level 0 (innermost) used for destination */
    uint16_t dicnt1;    /**< Total loop iteration count for level 1 used for destination */
    uint16_t dicnt2;    /**< Total loop iteration count for level 2 used for destination */
    uint16_t dicnt3;    /**< Total loop iteration count for level 3 used for destination */
} __attribute__((__packed__));

typedef struct CSL_BcdmaTR_t CSL_BcdmaTR;

/** \brief CSL_BcdmaTR0 specifies a Type 0 (One dimensional data move) Transfer Request. */
struct CSL_BcdmaTR0_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint32_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
} __attribute__((__packed__));

typedef struct CSL_BcdmaTR0_t CSL_BcdmaTR0;

/** \brief CSL_BcdmaTR1 specifies a Type 1 (Two dimensional data move) Transfer Request. */
struct CSL_BcdmaTR1_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
} __attribute__((__packed__));

typedef struct CSL_BcdmaTR1_t CSL_BcdmaTR1;

/** \brief CSL_BcdmaTR2 specifies a Type 2 (Three dimensional data move) Transfer Request. */
struct CSL_BcdmaTR2_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint32_t icnt2;     /**< Total loop iteration count for level 2 */
    int32_t  dim2;      /**< Signed dimension for loop level 2 */
} __attribute__((__packed__));

typedef struct CSL_BcdmaTR2_t CSL_BcdmaTR2;

/** \brief CSL_BcdmaTR3 specifies a Type 3 (Four dimensional data move) Transfer Request. */
struct CSL_BcdmaTR3_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint16_t icnt2;     /**< Total loop iteration count for level 2 */
    uint16_t icnt3;     /**< Total loop iteration count for level 3 (outermost) */
    int32_t  dim2;      /**< Signed dimension for loop level 2 */
    int32_t  dim3;      /**< Signed dimension for loop level 3 */
} __attribute__((__packed__));

typedef struct CSL_BcdmaTR3_t CSL_BcdmaTR3;

/** \brief CSL_BcdmaTR15 specifies a Type 15 Transfer Request. */
struct CSL_BcdmaTR15_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint16_t icnt2;     /**< Total loop iteration count for level 2 */
    uint16_t icnt3;     /**< Total loop iteration count for level 3 (outermost) */
    int32_t  dim2;      /**< Signed dimension for loop level 2 */
    int32_t  dim3;      /**< Signed dimension for loop level 3 */
    uint32_t fmtflags;  /**< [udmap_only] Tells how the data is formatted either between the input and the output or if the data should use different addressing schemes or sizes */
    int32_t  ddim1;     /**< Signed dimension for loop level 1 for the destination data */
    uint64_t daddr;     /**< Starting address for the destination of the data */
    int32_t  ddim2;     /**< Signed dimension for loop level 2 for the destination data */
    int32_t  ddim3;     /**< Signed dimension for loop level 3 for the destination data */
    uint16_t dicnt0;    /**< Total loop iteration count for level 0 (innermost) used for destination */
    uint16_t dicnt1;    /**< Total loop iteration count for level 1 used for destination */
    uint16_t dicnt2;    /**< Total loop iteration count for level 2 used for destination */
    uint16_t dicnt3;    /**< Total loop iteration count for level 3 used for destination */
} __attribute__((__packed__));

typedef struct CSL_BcdmaTR15_t CSL_BcdmaTR15;

/**
 *  \anchor CSL_BcdmaTrFlags_t
 *  \name CSL BCDMA TR Flags
 *
 *  CSL BCDMA TR Flags.
 *
 *  @{
 */
#define CSL_BCDMA_TR_FLAGS_TYPE_SHIFT                    ((uint32_t) 0U)
#define CSL_BCDMA_TR_FLAGS_TYPE_MASK                     (((uint32_t) 0xFU) << CSL_BCDMA_TR_FLAGS_TYPE_SHIFT)
#define CSL_BCDMA_TR_FLAGS_STATIC_SHIFT                  ((uint32_t) 4U)
#define CSL_BCDMA_TR_FLAGS_STATIC_MASK                   (((uint32_t) 1U) << CSL_BCDMA_TR_FLAGS_STATIC_SHIFT)
#define CSL_BCDMA_TR_FLAGS_WAIT_SHIFT                    ((uint32_t) 5U)
#define CSL_BCDMA_TR_FLAGS_WAIT_MASK                     (((uint32_t) 1U) << CSL_BCDMA_TR_FLAGS_WAIT_SHIFT)
#define CSL_BCDMA_TR_FLAGS_EVENT_SIZE_SHIFT              ((uint32_t) 6U)
#define CSL_BCDMA_TR_FLAGS_EVENT_SIZE_MASK               (((uint32_t) 3U) << CSL_BCDMA_TR_FLAGS_EVENT_SIZE_SHIFT)
#define CSL_BCDMA_TR_FLAGS_TRIGGER0_SHIFT                ((uint32_t) 8U)
#define CSL_BCDMA_TR_FLAGS_TRIGGER0_MASK                 (((uint32_t) 3U) << CSL_BCDMA_TR_FLAGS_TRIGGER0_SHIFT)
#define CSL_BCDMA_TR_FLAGS_TRIGGER0_TYPE_SHIFT           ((uint32_t) 10U)
#define CSL_BCDMA_TR_FLAGS_TRIGGER0_TYPE_MASK            (((uint32_t) 3U) << CSL_BCDMA_TR_FLAGS_TRIGGER0_TYPE_SHIFT)
#define CSL_BCDMA_TR_FLAGS_TRIGGER1_SHIFT                ((uint32_t) 12U)
#define CSL_BCDMA_TR_FLAGS_TRIGGER1_MASK                 (((uint32_t) 3U) << CSL_BCDMA_TR_FLAGS_TRIGGER1_SHIFT)
#define CSL_BCDMA_TR_FLAGS_TRIGGER1_TYPE_SHIFT           ((uint32_t) 14U)
#define CSL_BCDMA_TR_FLAGS_TRIGGER1_TYPE_MASK            (((uint32_t) 3U) << CSL_BCDMA_TR_FLAGS_TRIGGER1_TYPE_SHIFT)
#define CSL_BCDMA_TR_FLAGS_CMD_ID_SHIFT                  ((uint32_t) 16U)
#define CSL_BCDMA_TR_FLAGS_CMD_ID_MASK                   (((uint32_t) 0xFFU) << CSL_BCDMA_TR_FLAGS_CMD_ID_SHIFT)
#define CSL_BCDMA_TR_FLAGS_CFG_FLAGS_SHIFT               ((uint32_t) 24U)
#define CSL_BCDMA_TR_FLAGS_CFG_FLAGS_MASK                (((uint32_t) 0xFFU) << CSL_BCDMA_TR_FLAGS_CFG_FLAGS_SHIFT)
#define CSL_BCDMA_TR_FLAGS_SA_INDIRECT_SHIFT             ((uint32_t) 24U)
#define CSL_BCDMA_TR_FLAGS_SA_INDIRECT_MASK              (((uint32_t) 1U) << CSL_BCDMA_TR_FLAGS_SA_INDIRECT_SHIFT)
#define CSL_BCDMA_TR_FLAGS_DA_INDIRECT_SHIFT             ((uint32_t) 25U)
#define CSL_BCDMA_TR_FLAGS_DA_INDIRECT_MASK              (((uint32_t) 1U) << CSL_BCDMA_TR_FLAGS_DA_INDIRECT_SHIFT)
#define CSL_BCDMA_TR_FLAGS_SUPR_EVT_SHIFT                ((uint32_t) 26U)
#define CSL_BCDMA_TR_FLAGS_SUPR_EVT_MASK                 (((uint32_t) 1U) << CSL_BCDMA_TR_FLAGS_SUPR_EVT_SHIFT)
#define CSL_BCDMA_TR_FLAGS_EOL_SHIFT                     ((uint32_t) 28U)
#define CSL_BCDMA_TR_FLAGS_EOL_MASK                      (((uint32_t) 7U) << CSL_BCDMA_TR_FLAGS_EOL_SHIFT)
#define CSL_BCDMA_TR_FLAGS_EOP_SHIFT                     ((uint32_t) 31U)
#define CSL_BCDMA_TR_FLAGS_EOP_MASK                      (((uint32_t) 1U) << CSL_BCDMA_TR_FLAGS_EOP_SHIFT)
/* @} */

struct CSL_BcdmaSecTR_t
{
    uint64_t addr;      /**< Address */
    uint32_t flags;     /**< Flags */
    uint32_t data[13];  /**< Data */
} __attribute__((__packed__));

typedef struct CSL_BcdmaSecTR_t CSL_BcdmaSecTR;

/**
 *  \anchor CSL_BcdmaSecTrFlags_t
 *  \name CSL BCDMA Secondary TR Flags
 *
 *  CSL BCDMA Secondary TR Flags.
 *
 *  @{
 */
#define CSL_BCDMA_SECTR_FLAGS_TYPE_SHIFT                 ((uint32_t) 0U)
#define CSL_BCDMA_SECTR_FLAGS_TYPE_MASK                  (((uint32_t) 0xFU) << CSL_BCDMA_SECTR_FLAGS_TYPE_SHIFT)
#define CSL_BCDMA_SECTR_FLAGS_TYPE_SPECIFIC_SHIFT        ((uint32_t) 4U)
#define CSL_BCDMA_SECTR_FLAGS_TYPE_SPECIFIC_MASK         (((uint32_t) 0xFFFFFFF0U) << CSL_BCDMA_SECTR_FLAGS_TYPE_SPECIFIC_SHIFT)
/* @} */

/*-----------------------------------------------------------------------------
 *  TR response support
 *---------------------------------------------------------------------------*/
struct CSL_BcdmaTrResponse_t
{
    uint32_t flags;     /**< Status flags */
} __attribute__((__packed__));

typedef struct CSL_BcdmaTrResponse_t CSL_BcdmaTrResponse;

/**
 *  \anchor CSL_BcdmaTrResponseFlags_t
 *  \name CSL BCDMA TR Response Flags
 *
 *  CSL BCDMA TR Response Flags.
 *
 *  @{
 */
#define CSL_BCDMA_TR_RESPONSE_STATUS_TYPE_SHIFT          ((uint32_t) 0U)
#define CSL_BCDMA_TR_RESPONSE_STATUS_TYPE_MASK           (((uint32_t) 0xFU) << CSL_BCDMA_TR_RESPONSE_STATUS_TYPE_SHIFT)
#define CSL_BCDMA_TR_RESPONSE_STATUS_INFO_SHIFT          ((uint32_t) 4U)
#define CSL_BCDMA_TR_RESPONSE_STATUS_INFO_MASK           (((uint32_t) 0xFU) << CSL_BCDMA_TR_RESPONSE_STATUS_FIELD_SHIFT)
#define CSL_BCDMA_TR_RESPONSE_CMDID_SHIFT                ((uint32_t) 16U)
#define CSL_BCDMA_TR_RESPONSE_CMDID_MASK                 (((uint32_t) 0xFFU) << CSL_BCDMA_TR_RESPONSE_CMDID_SHIFT)
#define CSL_BCDMA_TR_RESPONSE_CFG_SPECIFIC_SHIFT         ((uint32_t) 24U)
#define CSL_BCDMA_TR_RESPONSE_CFG_SPECIFIC_MASK          (((uint32_t) 0xFFU) << CSL_BCDMA_TR_RESPONSE_CFG_SPECIFIC_SHIFT)
/* @} */

/*-----------------------------------------------------------------------------
 *  Teardown response support
 *---------------------------------------------------------------------------*/
struct CSL_BcdmaTdResponse_t
{
    uint32_t tdIndicator;
    /**< TRUE: Indicates that a teardown has completed. */
    uint32_t chId;
    /**< Indicates which channel the teardown completed on. */
    uint32_t forced;
    /**< FALSE: Indicates that the teardown was graceful and data was not lost.
     *   TRUE : Indicates that the teardown was not graceful and data was
      *  potentially lost */
} __attribute__((__packed__));

typedef struct CSL_BcdmaTdResponse_t CSL_BcdmaTdResponse;

/**
 *  \anchor CSL_BcdmaTeardownResponseFlags_t
 *  \name CSL BCDMA Teardown Response Flags
 *
 *  CSL BCDMA Teardown Response Flags.
 *
 *  @{
 */
#define CSL_BCDMA_TD_RESPONSE_TD_INDICATOR_SHIFT    ((uint32_t) 0U)
#define CSL_BCDMA_TD_RESPONSE_TD_INDICATOR_MASK     (((uint32_t) 0xFU) << CSL_BCDMA_TD_RESPONSE_TD_INDICATOR_SHIFT)
#define CSL_BCDMA_TD_RESPONSE_CHAN_ID_SHIFT         ((uint32_t) 4U)
#define CSL_BCDMA_TD_RESPONSE_CHAN_ID_MASK          (((uint32_t) 0x3FFU) << CSL_BCDMA_TD_RESPONSE_CHAN_ID_SHIFT)
#define CSL_BCDMA_TD_RESPONSE_FORCED_SHIFT          ((uint32_t) 31U)
#define CSL_BCDMA_TD_RESPONSE_FORCED_MASK           (((uint32_t) 0x1U) << CSL_BCDMA_TD_RESPONSE_FORCED_SHIFT)
/* @} */

/* @} */

/**
 *  \addtogroup CSL_BCDMA_ENUM
 *  @{
 */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the the type of TR being sent
 *
 *  \anchor CSL_BcdmaTrFlagsType
 *  \name BCDMA TR flags type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrFlagsType;
    /** One dimensional data move */
#define CSL_BCDMA_TR_FLAGS_TYPE_1D_DATA_MOVE                        ((uint32_t) 0U)
    /** Two dimensional data move */
#define CSL_BCDMA_TR_FLAGS_TYPE_2D_DATA_MOVE                        ((uint32_t) 1U)
    /** Three dimensional data move */
#define CSL_BCDMA_TR_FLAGS_TYPE_3D_DATA_MOVE                        ((uint32_t) 2U)
    /** Four dimensional data move */
#define CSL_BCDMA_TR_FLAGS_TYPE_4D_DATA_MOVE                        ((uint32_t) 3U)
     /** Four dimensional data move with data formatting */
#define CSL_BCDMA_TR_FLAGS_TYPE_4D_DATA_MOVE_FORMATTING             ((uint32_t) 4U)
    /** Four dimensional cache warm */
#define CSL_BCDMA_TR_FLAGS_TYPE_4D_CACHE_WARM                       ((uint32_t) 5U)
    /** Four dimensional block move */
#define CSL_BCDMA_TR_FLAGS_TYPE_4D_BLOCK_MOVE                       ((uint32_t) 8U)
     /** Four dimensional block move with repacking */
#define CSL_BCDMA_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING             ((uint32_t) 9U)
    /** Two dimensional block move */
#define CSL_BCDMA_TR_FLAGS_TYPE_2D_BLOCK_MOVE                       ((uint32_t) 10U)
     /** Two dimensional block move with repacking */
#define CSL_BCDMA_TR_FLAGS_TYPE_2D_BLOCK_MOVE_REPACKING             ((uint32_t) 11U)
    /**< Four dimensional block move with repacking and indirection */
#define CSL_BCDMA_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION ((uint32_t) 15U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator specifies whether or not this TR should ensure it
 * completes before allowing the next TR to start on this channel. It is valid
 * only for type 15 TR's.
 *
 *  \anchor CSL_BcdmaTrFlagsWait
 *  \name BCDMA TR flags wait
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrFlagsWait;
    /** Allow next TR to start immediately */
#define CSL_BCDMA_TR_FLAGS_WAIT_NO   ((uint32_t) 0U)
    /** Wait for this TR to complete before allowing next TR to start */
#define CSL_BCDMA_TR_FLAGS_WAIT_YES  ((uint32_t) 1U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator specifies when an event is generated for each TR
 *
 *  \anchor CSL_BcdmaTrFlagsEventSize
 *  \name BCDMA TR flags event size
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrFlagsEventSize;
    /** When TR is complete and all status for the TR has been received */
#define CSL_BCDMA_TR_FLAGS_EVENT_SIZE_COMPLETION    ((uint32_t) 0U)
    /** Type 0: when the last data transaction is sent for the TR; Type 1-11: when ICNT1 is decremented */
#define CSL_BCDMA_TR_FLAGS_EVENT_SIZE_ICNT1_DEC     ((uint32_t) 1U)
    /** Type 0-1,10-11: when the last data transaction is sent for the TR; All other types: when ICNT2 is decremented */
#define CSL_BCDMA_TR_FLAGS_EVENT_SIZE_ICNT2_DEC     ((uint32_t) 2U)
    /** Type 0-2,10-11: when the last data transaction is sent for the TR; All other types: when ICNT3 is decremented */
#define CSL_BCDMA_TR_FLAGS_EVENT_SIZE_ICNT3_DEC     ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator specifies the type of trigger used to enable the TR
 *        to transfer data as specified by CSL_BcdmaTrFlagsTriggerType
 *
 *  \anchor CSL_BcdmaTrFlagsTrigger
 *  \name BCDMA TR flags trigger
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrFlagsTrigger;
    /** No Trigger */
#define CSL_BCDMA_TR_FLAGS_TRIGGER_NONE             ((uint32_t) 0U)
    /** Global Trigger 0 for the channel */
#define CSL_BCDMA_TR_FLAGS_TRIGGER_GLOBAL0          ((uint32_t) 1U)
    /** Global Trigger 1 for the channel */
#define CSL_BCDMA_TR_FLAGS_TRIGGER_GLOBAL1          ((uint32_t) 2U)
    /** Local Event for the channel */
#define CSL_BCDMA_TR_FLAGS_TRIGGER_LOCAL_EVENT      ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator specifies the type of data transfer that will be
 *        enabled by receiving a trigger as specified by CSL_BcdmaTrFlagsTrigger
 *
 *  \anchor CSL_BcdmaTrFlagsTriggerType
 *  \name BCDMA TR flags trigger type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrFlagsTriggerType;
    /** The second inner most loop (ICNT1) will be decremented by 1 */
#define CSL_BCDMA_TR_FLAGS_TRIGGER_TYPE_ICNT1_DEC   ((uint32_t) 0U)
    /** The third inner most loop (ICNT2) will be decremented by 1 */
#define CSL_BCDMA_TR_FLAGS_TRIGGER_TYPE_ICNT2_DEC   ((uint32_t) 1U)
    /** The outer most loop (ICNT3) will be decremented by 1 */
#define CSL_BCDMA_TR_FLAGS_TRIGGER_TYPE_ICNT3_DEC   ((uint32_t) 2U)
    /** The entire TR will be allowed to complete */
#define CSL_BCDMA_TR_FLAGS_TRIGGER_TYPE_ALL         ((uint32_t) 3U)
/* @} */

/**
 *  \anchor CSL_BcdmaTrFormatFlags_t
 *  \name CSL BCDMA TR Format Flags
 *
 *  [udmap_only] CSL BCDMA TR Format Flags.
 *
 *  @{
 */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SHIFT                ((uint32_t) 0U)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_MASK                 (((uint32_t) 7U) << CSL_BCDMA_TR_FMTFLAGS_AMODE_SHIFT)
#define CSL_BCDMA_TR_FMTFLAGS_DIR_SHIFT                  ((uint32_t) 3U)
#define CSL_BCDMA_TR_FMTFLAGS_DIR_MASK                   (((uint32_t) 1U) << CSL_BCDMA_TR_FMTFLAGS_DIR_SHIFT)
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_SHIFT                ((uint32_t) 4U)
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_MASK                 (((uint32_t) 0xFU) << CSL_BCDMA_TR_FMTFLAGS_ELYPE_SHIFT)
#define CSL_BCDMA_TR_FMTFLAGS_DFMT_SHIFT                 ((uint32_t) 8U)
#define CSL_BCDMA_TR_FMTFLAGS_DFMT_MASK                  (((uint32_t) 0xFU) << CSL_BCDMA_TR_FMTFLAGS_DFMT_SHIFT)
#define CSL_BCDMA_TR_FMTFLAGS_SECTR_SHIFT                ((uint32_t) 12U)
#define CSL_BCDMA_TR_FMTFLAGS_SECTR_MASK                 (((uint32_t) 3U) << CSL_BCDMA_TR_FMTFLAGS_SECTR_SHIFT)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK0_SHIFT  ((uint32_t) 16U)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK0_MASK   (((uint32_t) 0xFU) << CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK0_SHIFT)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK1_SHIFT  ((uint32_t) 20U)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK1_MASK   (((uint32_t) 0xFU) << CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK1_SHIFT)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AM0_SHIFT   ((uint32_t) 24U)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AM0_MASK    (((uint32_t) 3U) << CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AM0_SHIFT)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AM1_SHIFT   ((uint32_t) 26U)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AM1_MASK    (((uint32_t) 3U) << CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AM1_SHIFT)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AM2_SHIFT   ((uint32_t) 28U)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AM2_MASK    (((uint32_t) 3U) << CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AM2_SHIFT)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AM3_SHIFT   ((uint32_t) 30U)
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AM3_MASK    (((uint32_t) 3U) << CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AM3_SHIFT)
#define CSL_BCDMA_TR_CACHEFLAGS_CACHEID_SHIFT            ((uint32_t) 0U)
#define CSL_BCDMA_TR_CACHEFLAGS_CACHEID_MASK             (((uint32_t) 0xFFU) << CSL_BCDMA_TR_CACHEFLAGS_CACHEID_SHIFT)
#define CSL_BCDMA_TR_CACHEFLAGS_CACHEOP_SHIFT            ((uint32_t) 24U)
#define CSL_BCDMA_TR_CACHEFLAGS_CACHEOP_MASK             (((uint32_t) 0xFFU) << CSL_BCDMA_TR_CACHEFLAGS_CACHEOP_SHIFT)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief [udmap_only] This enumerator specifies the addressing mode of TR that is being sent
 *
 *  \anchor CSL_BcdmaTrFmtflagsAmode
 *  \name BCDMA TR format flags amode
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrFmtflagsAmode;
    /** Linear addressing */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_LINEAR          ((uint32_t) 0U)
    /** Circular addressing */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_CIRCULAR        ((uint32_t) 1U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief [udmap_only] This enumerator specifies CBK0 and CBK1 values used to compute the
 *        circular block sizes for circular addressing
 *
 *  \anchor CSL_BcdmaTrFmtflagsAmodeSpecificCbk
 *  \name BCDMA TR format flags amode specific CBK values
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrFmtflagsAmodeSpecificCbk;
    /** 512B  block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_512B   ((uint32_t) 0U)
    /** 1KB   block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_1KB    ((uint32_t) 1U)
    /** 2KB   block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_2KB    ((uint32_t) 2U)
    /** 4KB   block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_4KB    ((uint32_t) 3U)
    /** 8KB   block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_8KB    ((uint32_t) 4U)
    /** 16KB  block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_16KB   ((uint32_t) 5U)
    /** 32KB  block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_32KB   ((uint32_t) 6U)
    /** 64KB  block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_64KB   ((uint32_t) 7U)
    /** 128KB block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_128KB  ((uint32_t) 8U)
    /** 256KB block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_256KB  ((uint32_t) 9U)
    /** 512KB block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_512KB  ((uint32_t) 10U)
    /** 1GB   block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_1GB    ((uint32_t) 11U)
    /** 2GB   block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_2GB    ((uint32_t) 12U)
    /** 4GB   block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_4GB    ((uint32_t) 13U)
    /** 8GB   block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_8GB    ((uint32_t) 14U)
    /** 16GB  block size */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_16GB   ((uint32_t) 15U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief [udmap_only] This enumerator specifies the addressing mode of TR that is being sent
 *
 *  \anchor CSL_BcdmaTrFmtflagsAmodeSpecificAmode
 *  \name BCDMA TR format flags amode specific amode
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrFmtflagsAmodeSpecificAmode;
    /** Linear addressing */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AMODE_LINEAR   ((uint32_t) 0U)
    /** Circular Buffer 0 is used */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AMODE_CBK0     ((uint32_t) 1U)
    /** Circular Buffer 1 is used */
#define CSL_BCDMA_TR_FMTFLAGS_AMODE_SPECIFIC_AMODE_CBK1     ((uint32_t) 2U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief [udmap_only] This enumerator specifies the addressing mode of TR that is being sent
 *
 *  \anchor CSL_BcdmaTrFmtflagsDir
 *  \name BCDMA TR format flags direction
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrFmtflagsDir;
    /** The source addressing will use the method selected in the AMODE field and the destination address will use the default linear addressing */
#define CSL_BCDMA_TR_FMTFLAGS_DIR_SRC_USES_AMODE        ((uint32_t) 0U)
    /** The destination addressing will use the method selected in the AMODE field and the source addressing will use the default linear addressing */
#define CSL_BCDMA_TR_FMTFLAGS_DIR_DST_USES_AMODE        ((uint32_t) 1U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief [udmap_only] This enumerator specifies the basic unit that is used for the
 *        innermost loop
 *
 *  \anchor CSL_BcdmaTrFmtflagsEltype
 *  \name BCDMA TR format flags element type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrFmtflagsEltype;
    /** 1 Byte per element */
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_1           ((uint32_t) 0U)
    /** 1.5 Bytes (12 bits) per element */
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_1p5         ((uint32_t) 1U)
    /** 2 Bytes per element */
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_2           ((uint32_t) 2U)
    /** 3 Bytes per element */
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_3           ((uint32_t) 3U)
    /** 4 Bytes per element */
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_4           ((uint32_t) 4U)
    /** 5 Bytes per element */
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_5           ((uint32_t) 5U)
    /** 16 Bytes per element */
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_16          ((uint32_t) 6U)
    /** 32 Bytes per element */
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_32          ((uint32_t) 7U)
    /** 1 Byte per input element 2 Bytes per output element */
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_1_2         ((uint32_t) 8U)
    /** 1.5 Bytes per input element 2 Bytes per output element */
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_1p5_2       ((uint32_t) 9U)
    /** 2 Bytes per input element 1 Byte per output element */
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_2_1         ((uint32_t) 10U)
    /** 2 Bytes per input element 1.5 Bytes per output element */
#define CSL_BCDMA_TR_FMTFLAGS_ELYPE_2_1p5       ((uint32_t) 11U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief [udmap_only] This enumerator specifies a manipulation of the data between how it
 *        is read and how it is sent
 *
 *  \anchor CSL_BcdmaTrFmtflagsDfmt
 *  \name BCDMA TR format flags dfmt
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrFmtflagsDfmt;
    /** The input and output block will remain identical. */
#define CSL_BCDMA_TR_FMTFLAGS_DFMT_NO_CHANGE            ((uint32_t) 0U)
    /** The input block is not an address but the address is up to a 64 bit constant. */
#define CSL_BCDMA_TR_FMTFLAGS_DFMT_CONSTANT_COPY        ((uint32_t) 1U)
    /** The inner and second most inner loops are swapped so that rows become columns and columns become rows. */
#define CSL_BCDMA_TR_FMTFLAGS_DFMT_TRANSPOSE            ((uint32_t) 2U)
    /** The data in the row will be accessed in the reverse of the order that it is read. */
#define CSL_BCDMA_TR_FMTFLAGS_DFMT_REVERSE              ((uint32_t) 3U)
    /** The data will be written in the reverse of the order that is read as well as transposed. */
#define CSL_BCDMA_TR_FMTFLAGS_DFMT_REVERSE_TRANSPOSE    ((uint32_t) 4U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief [udmap_only] This enumerator specifies the cache operation for a cache warm TR type
 *
 *  \anchor CSL_BcdmaTrCacheflagsOp
 *  \name BCDMA TR format cache flags operation
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrCacheflagsOp;
    /** Prewarm the Cache */
#define CSL_BCDMA_TR_CACHEFLAGS_OP_PREWARM_CACHE        ((uint32_t) 0U)
    /** Prewarm MMU */
#define CSL_BCDMA_TR_CACHEFLAGS_OP_PREWARM_MMU          ((uint32_t) 1U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief [udmap_only] This enumerator specifies if the TR requires an additional TR located
 *        in memory
 *
 *  \anchor CSL_BcdmaTrFmtflagsSectr
 *  \name BCDMA TR format flags secondary TR
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrFmtflagsSectr;
    /** The TR does not require a secondary TR. */
#define CSL_BCDMA_TR_FMTFLAGS_SECTR_NONE        ((uint32_t) 0U)
    /** The TR will fetch a 64 byte secondary TR prior to the initial read. */
#define CSL_BCDMA_TR_FMTFLAGS_SECTR_64          ((uint32_t) 1U)
    /** The TR will fetch a 128 byte secondary TR prior to the initial read. */
#define CSL_BCDMA_TR_FMTFLAGS_SECTR_128         ((uint32_t) 2U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator is used to determine what type of status is being
 *        returned
 *
 *  \anchor CSL_BcdmaTrResponseStatus
 *  \name BCDMA TR response status
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrResponseStatus;
    /** None */
#define CSL_BCDMA_TR_RESPONSE_STATUS_COMPLETE           ((uint32_t) 0U)
    /** Transfer Error */
#define CSL_BCDMA_TR_RESPONSE_STATUS_TRANSFER_ERR       ((uint32_t) 1U)
    /** Aborted Error */
#define CSL_BCDMA_TR_RESPONSE_STATUS_ABORTED_ERR        ((uint32_t) 2U)
    /** Submission Error */
#define CSL_BCDMA_TR_RESPONSE_STATUS_SUBMISSION_ERR     ((uint32_t) 3U)
    /**< Unsupported Feature */
#define CSL_BCDMA_TR_RESPONSE_STATUS_UNSUPPORTED_ERR    ((uint32_t) 4U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator is returned for a TR that is received that can not be
 *        run
 *
 *  \anchor CSL_BcdmaTrResponseStatusSubmission
 *  \name BCDMA TR response status submission
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrResponseStatusSubmission;
    /** ICNT0 was 0 */
#define CSL_BCDMA_TR_RESPONSE_STATUS_SUBMISSION_ICNT0       ((uint32_t) 0U)
    /** Channel FIFO was full when TR received */
#define CSL_BCDMA_TR_RESPONSE_STATUS_SUBMISSION_FIFO_FULL   ((uint32_t) 1U)
    /** Channel is not owned by the submitter */
#define CSL_BCDMA_TR_RESPONSE_STATUS_SUBMISSION_OWN         ((uint32_t) 2U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator is returned for a TR that is received that can not be
 *        run because it specifies a feature that is optional and not supported
 *        by the UTC that received the TR
 *
 *  \anchor CSL_BcdmaTrResponseStatusUnsupported
 *  \name BCDMA TR response status unsupported
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrResponseStatusUnsupported;
    /** TR Type not supported */
#define CSL_BCDMA_TR_RESPONSE_STATUS_UNSUPPORTED_TR_TYPE        ((uint32_t) 0U)
    /** STATIC not supported */
#define CSL_BCDMA_TR_RESPONSE_STATUS_UNSUPPORTED_STATIC         ((uint32_t) 1U)
    /** EOL not supported */
#define CSL_BCDMA_TR_RESPONSE_STATUS_UNSUPPORTED_EOL            ((uint32_t) 2U)
    /** CONFIGURATION SPECIFIC not supported */
#define CSL_BCDMA_TR_RESPONSE_STATUS_UNSUPPORTED_CFG_SPECIFIC   ((uint32_t) 3U)
    /** AMODE not supported */
#define CSL_BCDMA_TR_RESPONSE_STATUS_UNSUPPORTED_AMODE          ((uint32_t) 4U)
    /** ELTYPE not supported */
#define CSL_BCDMA_TR_RESPONSE_STATUS_UNSUPPORTED_ELTYPE         ((uint32_t) 5U)
    /** DFMT not supported */
#define CSL_BCDMA_TR_RESPONSE_STATUS_UNSUPPORTED_DFMT           ((uint32_t) 6U)
    /** SECTR not supported */
#define CSL_BCDMA_TR_RESPONSE_STATUS_UNSUPPORTED_SECTR          ((uint32_t) 7U)
    /** AMODE SPECIFIC field not supported */
#define CSL_BCDMA_TR_RESPONSE_STATUS_UNSUPPORTED_AMODE_SPECIFIC ((uint32_t) 8U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator is returned for a TR that completed but experienced a
 *        known exception during reception. The STATUS_INFO field specifies the
 *        type of transfer exception that was encountered.
 *
 *  \anchor CSL_BcdmaTrResponseStatusTransferException
 *  \name BCDMA TR response status transfer exception
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_BcdmaTrResponseStatusTransferException;
    /** EOP on incoming data stream was encountered prematurely */
#define CSL_BCDMA_TR_RESPONSE_STATUS_TRANSFER_EXCEPTION_SHORT_PKT   ((uint32_t) 0U)
    /** EOP on incoming data stream was encountered late  */
#define CSL_BCDMA_TR_RESPONSE_STATUS_TRANSFER_EXCEPTION_LONG_PKT    ((uint32_t) 1U)
/* @} */

/* @} */

/**
 *  \addtogroup CSL_BCDMA_FUNCTION
 *  @{
 */

/**
 *  \brief Return the TR response status type
 *
 *  This function returns the status type of the specified TR response.
 *  See \ref CSL_BcdmaTrResponseStatus for available status types.
 *
 *  \param pTrResponse  [IN]    Pointer to the TR Response structure
 *
 *  \return The status type of the specified TR response
 */
static inline CSL_BcdmaTrResponseStatus CSL_bcdmaTrResponseGetStatusType( const CSL_BcdmaTrResponse *pTrResponse );

static inline CSL_BcdmaTrResponseStatus CSL_bcdmaTrResponseGetStatusType( const CSL_BcdmaTrResponse *pTrResponse )
{
    return (CSL_BcdmaTrResponseStatus)CSL_FEXT( pTrResponse->flags, BCDMA_TR_RESPONSE_STATUS_TYPE );
}

/**
 *  \brief Parses the TD response word
 *
 *  This function parses the teardown response word received from the
 *  completion queue.
 *
 *  \param tdResponseWord   [IN]    TD Response word to parse
 *  \param pTdResponse      [OUT]   Pointer to the TD Response structure to be filled
 */
static inline void CSL_bcdmaGetTdResponse(uint64_t tdResponseWord,
                                          CSL_BcdmaTdResponse *pTdResponse);

static inline void CSL_bcdmaGetTdResponse(uint64_t tdResponseWord,
                                          CSL_BcdmaTdResponse *pTdResponse)
{
    if(NULL != pTdResponse)
    {
        pTdResponse->tdIndicator = CSL_FEXT(tdResponseWord, BCDMA_TD_RESPONSE_TD_INDICATOR);
        pTdResponse->chId = CSL_FEXT(tdResponseWord, BCDMA_TD_RESPONSE_CHAN_ID);
        pTdResponse->forced = CSL_FEXT(tdResponseWord, BCDMA_TD_RESPONSE_FORCED);
    }
}

/* @} */

#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif  /* end of CSL_BCDMA_H_ definition */
/** @} */

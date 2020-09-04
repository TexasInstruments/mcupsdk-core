/*
 *  Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  \file  csl_pktdma.h
 *
 *  \brief
 *  This CSL-FL header file contains various enumerations, structure
 *  definitions and function declarations for the Packet DMA (PKTDMA) IP.
 *
 *  This CSL-FL was designed to be orthogonal with the implementation
 *  of the udmap CSL-FL. Enumerations, structure definitions, and API functions
 *  are similarly named. The include file "csl_pktdma_alias_udmap_api.h" is
 *  available which maps pktdma CSL-FL content to their udmap equivalents for
 *  ease in porting existing udmap code to the pktdma.
 *
 *  There is CSL-FL content that is applicable for udmap but not for pktdma.
 *  Those items are denoted with the tag [udmap_only] in the comments below
 *  and are handled as follows:
 *    - Enumerations: The enumeration is defined, but is not used functionally
 *    - Structure definitions: The structure is defined, but is not used
 *      functionally
 *    - Structure elements: The element is defined in the structure, but
 *      is not used functionally
 *    - API functions: The function is implemented, but does no operations
 *
 *  The following is the required calling sequence in order to insure proper
 *  operation of this CSL-FL:
 *
 *    1. Allocate a PKTDMA configuration structure (#CSL_PktdmaCfg)
 *    2. (Optional) Call #CSL_pktdmaInitCfg to initialize the PKTDMA
 *       configuration structure to all zeros
 *    3. Initialize the register structure pointer elements of the PKTDMA
 *       configuration structure
 *    4. Call #CSL_pktdmaGetCfg to populate the PKTDMA configuration structure
 *       with configuration and capability information for the PKTDMA module
 *    5. Call the other PKTDMA API functions as required
 */
/**
 *  \ingroup CSL_IP_MODULE
 *  \defgroup CSL_PKTDMA PKTDMA CSL-FL
 *
 *  @{
 */

#ifndef CSL_PKTDMA_H_
#define CSL_PKTDMA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <drivers/udma/include/csl_udmap_cppi5.h>
#include <drivers/udma/include/csl_pktdma_cppi5.h>
#include <drivers/hw_include/cslr_pktdma.h>
#include <drivers/hw_include/csl_types.h>

/**
@defgroup CSL_PKTDMA_DATASTRUCT  PKTDMA Data Structures
@ingroup CSL_PKTDMA_API
*/
/**
@defgroup CSL_PKTDMA_FUNCTION  PKTDMA Functions
@ingroup CSL_PKTDMA_API
*/
/**
@defgroup CSL_PKTDMA_ENUM PKTDMA Enumerated Data Types
@ingroup CSL_PKTDMA_API
*/

/**
 *  \addtogroup CSL_PKTDMA_ENUM
 *  @{
 */

#define CSL_PKTDMA_NO_EVENT         ((uint32_t) 0xFFFFU)

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible channel directions
 *
 *  \anchor CSL_PktdmaChanDir
 *  \name PKTDMA channel direction
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_PktdmaChanDir;
    /** Transmit direction */
#define CSL_PKTDMA_CHAN_DIR_TX   ((uint32_t) 0U)
    /** Receive direction */
#define CSL_PKTDMA_CHAN_DIR_RX   ((uint32_t) 1U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the nominal burst size and alignment for
 * data transfers on a TX or RX channel
 *
 *  \anchor CSL_PktdmaChanBurstSize
 *  \name PKTDMA channel burst size
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_PktdmaChanBurstSize;
    /** 32-byte burst size */
#define CSL_PKTDMA_CHAN_BURST_SIZE_32_BYTES  ((uint32_t) 0U)
    /** 64-byte burst size */
#define CSL_PKTDMA_CHAN_BURST_SIZE_64_BYTES  ((uint32_t) 1U)
    /** 128-byte burst size */
#define CSL_PKTDMA_CHAN_BURST_SIZE_128_BYTES ((uint32_t) 2U)
    /** 256-byte burst size */
#define CSL_PKTDMA_CHAN_BURST_SIZE_256_BYTES ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible descriptor types
 *
 *  \anchor CSL_PktdmaDescType
 *  \name PKTDMA descriptor type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_PktdmaDescType;
    /** Host */
#define CSL_PKTDMA_DESC_TYPE_HOST        ((uint32_t) 0U)
    /** Host single-buffer */
#define CSL_PKTDMA_DESC_TYPE_HOST_SB     ((uint32_t) 1U)
    /** Monolithic */
#define CSL_PKTDMA_DESC_TYPE_MONOLITHIC  ((uint32_t) 2U)
    /** Reserved */
#define CSL_PKTDMA_DESC_TYPE_RESERVED    ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the ps location for the descriptor
 *
 *  \anchor CSL_PktdmaPsLoc
 *  \name PKTDMA protocol-specific data location
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_PktdmaPsLoc;
    /** Located in descriptor */
#define CSL_PKTDMA_PS_LOC_DESC       ((uint32_t) 0U)
    /** Located in packet */
#define CSL_PKTDMA_PS_LOC_PACKET     ((uint32_t) 1U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible address types
 *
 *  \anchor CSL_PktdmaAddrType
 *  \name PKTDMA address type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_PktdmaAddrType;
    /** Physical addressing */
#define CSL_PKTDMA_ADDR_TYPE_PHYS    ((uint32_t) 0U)
    /** Intermediate addressing */
#define CSL_PKTDMA_ADDR_TYPE_INTER   ((uint32_t) 1U)
    /** Virtual addressing */
#define CSL_PKTDMA_ADDR_TYPE_VIRT    ((uint32_t) 2U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible channel types
 *
 *  \anchor CSL_PktdmaChanType
 *  \name PKTDMA channel type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_PktdmaChanType;
    /** Normal channel */
#define CSL_PKTDMA_CHAN_TYPE_NORMAL             ((uint32_t) 2U)
    /** Single buffer mode channel */
#define CSL_PKTDMA_CHAN_TYPE_SBMODE             ((uint32_t) 3U)
    /** RM, Packet Mode, Pass by reference */
#define CSL_PKTDMA_CHAN_TYPE_REF_PKT_RING       ((uint32_t) 2U)
    /** QM, Packet Single Buffer Mode, Pass by reference */
#define CSL_PKTDMA_CHAN_TYPE_REF_PKTSB_QUEUE    ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines how tag values are determined
 *
 *  \anchor CSL_PktdmaTagSelect
 *  \name PKTDMA tag select
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_PktdmaTagSelect;
    /** Do not overwrite */
#define CSL_PKTDMA_TAG_SELECT_NO_OVERWRITE               ((uint32_t) 0U)
    /** Overwrite with value given in tag value */
#define CSL_PKTDMA_TAG_SELECT_OVERWRITE_WITH_VAL         ((uint32_t) 1U)
    /** Overwrite with flow_id[7:0] from back-end */
#define CSL_PKTDMA_TAG_SELECT_OVERWRITE_WITH_FLOWID_7_0  ((uint32_t) 2U)
    /** Overwrite with flow_id[15:8] from back-end */
#define CSL_PKTDMA_TAG_SELECT_OVERWRITE_WITH_FLOWID_15_8 ((uint32_t) 3U)
    /** Overwrite with tag[7:0] from back-end */
#define CSL_PKTDMA_TAG_SELECT_OVERWRITE_WITH_TAG_7_0     ((uint32_t) 4U)
    /** Overwrite with tag[15:8] from back-end */
#define CSL_PKTDMA_TAG_SELECT_OVERWRITE_WITH_TAG_15_8    ((uint32_t) 5U)
    /** Invalid */
#define CSL_PKTDMA_TAG_SELECT_INVALID                    ((uint32_t) 6U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator selects which scheduling bin the channel will be
 *  placed in for bandwidth allocation of the DMA units
 *
 *  \anchor CSL_PktdmaChanSchedPri
 *  \name PKTDMA channel schedling priority
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_PktdmaChanSchedPri;
    /** High priority */
#define CSL_PKTDMA_CHAN_SCHED_PRI_HIGH       ((uint32_t) 0U)
    /** Medium-High priority */
#define CSL_PKTDMA_CHAN_SCHED_PRI_MED_HIGH   ((uint32_t) 1U)
    /** Medium-Low priority */
#define CSL_PKTDMA_CHAN_SCHED_PRI_MED_LOW    ((uint32_t) 2U)
    /** Low priority */
#define CSL_PKTDMA_CHAN_SCHED_PRI_LOW        ((uint32_t) 3U)
/* @} */

/* @} */

/**
 *  \addtogroup CSL_PKTDMA_DATASTRUCT
 *  @{
 */


/** \brief Routing tag information
 *
 *  [udmap_only]This structure contains information describing a routing tag.
 *
 */
typedef struct
{
    uint32_t                loSel;             /**< [IN]  Specifies how the low tag value is determined. See \ref CSL_PktdmaTagSelect */
    uint8_t                 loVal;             /**< [IN]  Tag[7:0] low byte value (used if loSel == 1) */
    uint32_t                hiSel;             /**< [IN]  Specifies how the high tag value is determined. See \ref CSL_PktdmaTagSelect */
    uint8_t                 hiVal;             /**< [IN]  Tag[7:0] high byte value (used if hiSel == 1) */
} CSL_PktdmaRouteTag;

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
} CSL_PktdmaRevision;

/** \brief Receive flow configuration information
 *
 *  This structure contains information describing a receive flow.
 *
 */
typedef struct
{
    uint32_t            einfoPresent;   /**< [IN]  Set to 1 if extended packet info is present in the descriptor */
    uint32_t            psInfoPresent;  /**< [IN]  Set to 1 if protocol-specific info is present in the descriptor */
    uint32_t            errorHandling;  /**< [IN]  Determines how starvation errors are handled. 0=drop packet, 1=retry */
    uint32_t            sopOffset;      /**< [IN]  Start of rx packet data (byte offset from the start of the SOP buffer) */
    uint32_t            secure;         /**< [udmap_only][IN]  Secure attribute to be used when receiving packets on this flowid */
    uint32_t            priv;           /**< [udmap_only][IN]  Privilege attribute to be used when receiving packets on this flowid */
    uint32_t            privid;         /**< [udmap_only][IN]  Privilege ID attribute to be used when receiving packets on this flowid */
} CSL_PktdmaRxFlowCfg;

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
    CSL_PktdmaAddrType      addrType;             /**< [udmap_only][IN] Address type for this channel */
    CSL_PktdmaChanType      chanType;             /**< [IN] Channel type */
    uint32_t                fetchWordSize;        /**< [udmap_only][IN] Descriptor/TR Size in 32-bit words */
    uint32_t                trEventNum;           /**< [udmap_only][IN] Specifies a global event number to generate anytime the required event generation criteria specified in a TR are met (set to CSL_PKTDMA_NO_EVENT for no event generation) */
    uint32_t                errEventNum;          /**< [udmap_only][IN] Specifies a global event number to generate anytime an error is encountered on the channel (set to CSL_PKTDMA_NO_EVENT for no event generation) */
    uint32_t                busPriority;          /**< [IN] 3-bit priority value (0=highest, 7=lowest) */
    uint32_t                busQos;               /**< [udmap_only][IN] 3-bit qos value (0=highest, 7=lowest) */
    uint32_t                busOrderId;           /**< [IN] 4-bit orderid value */
    CSL_PktdmaChanSchedPri  dmaPriority;          /**< [IN] This field selects which scheduling bin the channel will be placed in for bandwidth allocation of the Tx DMA units */
    uint32_t                txCredit;             /**< [udmap_only][IN] TX credit for external channels */
    uint32_t                txTrCQ;               /**< [udmap_only][IN] TX TR Completion Queue */
    uint32_t                txThread;             /**< [IN] TX mapped destination thread */
    bool                    bNoTeardownCompletePkt; /**< [IN] Specifies whether or not the channel should suppress sending the single data phase teardown packet when teardown is complete. 0 = TD packet is sent, 1 = Suppress sending TD packet */
    uint32_t                tdType;               /**< [IN] Specifies whether or not the channel should immediately return a teardown completion response to the default completion queue or wait until a status message is returned from the remote PSI-L paired peripheral. 0 = return immediately once all traffic is complete in PKTDMA, 1 = wait until remote peer sends back a completion message. Valid in pktdma version 2.0.0 and later. */
} CSL_PktdmaTxChanCfg;

/** \brief Receive channel configuration information
 *
 *  This structure contains configuration information for a receive channel.
 *
 */
typedef struct
{
    uint32_t                pauseOnError;         /**< [IN] When set, pause channel on error */
    CSL_PktdmaAddrType      addrType;             /**< [udmap_only][IN] Address type for this channel */
    CSL_PktdmaChanType      chanType;             /**< [IN] Channel type */
    uint32_t                fetchWordSize;        /**< [udmap_only][IN] Descriptor/TR Size in 32-bit words */
    uint32_t                trEventNum;           /**< [udmap_only][IN] Specifies a global event number to generate anytime the required event generation criteria specified in a TR are met (set to CSL_PKTDMA_NO_EVENT for no event generation) */
    uint32_t                errEventNum;          /**< [udmap_only][IN] Specifies a global event number to generate anytime an error is encountered on the channel (set to CSL_PKTDMA_NO_EVENT for no event generation) */
    uint32_t                busPriority;          /**< [IN] 3-bit priority value (0=highest, 7=lowest) */
    uint32_t                busQos;               /**< [udmap_only][IN] 3-bit qos value (0=highest, 7=lowest) */
    uint32_t                busOrderId;           /**< [IN] 4-bit orderid value */
    uint32_t                rxTrCQ;               /**< [udmap_only][IN] RX TR Completion Queue */
    uint32_t                rxThread;             /**< [IN] Rx channel destination ThreadID mapping */
    uint32_t                flowIdFwRangeStart;   /**< [udmap_only][IN] Starting flow ID value for firewall check */
    uint32_t                flowIdFwRangeCnt;     /**< [udmap_only][IN] Number of valid flow ID's starting from flowIdFwRangeStart for firewall check */
    bool                    bIgnoreShortPkts;     /**< [udmap_only][IN] This field controls whether or not short packets will be treated as exceptions (false) or ignored (true) for the channel. This field is only used when the channel is in split UTC mode.*/
    bool                    bIgnoreLongPkts;      /**< [udmap_only][IN] This field controls whether or not long packets will be treated as exceptions (false) or ignored (true) for the channel. This field is only used when the channel is in split UTC mode.*/
    CSL_PktdmaChanSchedPri  dmaPriority;          /**< [IN] This field selects which scheduling bin the channel will be placed in for bandwidth allocation of the Rx DMA units */
} CSL_PktdmaRxChanCfg;

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
    uint32_t                forcedTeardown;       /**< [udmap_only][IN]  When set, a forced teardown will be performed on the channel. Note that teardown must also be set when setting forcedTeardown. */
} CSL_PktdmaRT;

/** \brief PKTDMA configuration structure
 *
 *  This structure contains configuration information for the PKTDMA.
 *
 */
typedef struct
{
    CSL_pktdma_gcfgRegs     *pGenCfgRegs;               /**< [IN]  Pointer to the general configuration registers */
    CSL_pktdma_rxfcfgRegs   *pRxFlowCfgRegs;            /**< [IN]  Pointer to the rx flow configuration registers */
    CSL_pktdma_txccfgRegs   *pTxChanCfgRegs;            /**< [IN]  Pointer to the tx channel configuration registers */
    CSL_pktdma_rxccfgRegs   *pRxChanCfgRegs;            /**< [IN]  Pointer to the rx channel configuration registers */
    CSL_pktdma_txcrtRegs    *pTxChanRtRegs;             /**< [IN]  Pointer to the tx channel real-time registers */
    CSL_pktdma_rxcrtRegs    *pRxChanRtRegs;             /**< [IN]  Pointer to the rx channel real-time registers */
    uint32_t                cap0;                       /**< [OUT] Contains the contents of the Capabilities Register 0 (populated by the #CSL_pktdmaGetCfg function) */
    uint32_t                cap1;                       /**< [OUT] Contains the contents of the Capabilities Register 1 (populated by the #CSL_pktdmaGetCfg function) */
    uint32_t                txChanCnt;                  /**< [OUT] Tx channel count (populated by the #CSL_pktdmaGetCfg function) */
    uint32_t                rxChanCnt;                  /**< [OUT] Rx channel count (populated by the #CSL_pktdmaGetCfg function) */
    uint32_t                rxFlowCnt;                  /**< [OUT] Rx flow count (populated by the #CSL_pktdmaGetCfg function) */
    uint32_t                txExtUtcChanCnt;            /**< [udmap_only][OUT] Tx external UTC channel count (populated by the #CSL_pktdmaGetCfg function) */
    uint32_t                txHighCapacityChanCnt;      /**< [OUT] Tx external UTC channel count (populated by the #CSL_pktdmaGetCfg function) */
    uint32_t                txUltraHighCapacityChanCnt; /**< [OUT] Tx external UTC channel count (populated by the #CSL_pktdmaGetCfg function) */
} CSL_PktdmaCfg;

/** \brief [udmap_only] PKTDMA receive flow id firewall status
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
} CSL_PktdmaRxFlowIdFirewallStatus;

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
    uint32_t    droppedPacketCnt;               /**< [OUT] Current dropped packet count for the channel. Valid only for receive channels. */
} CSL_PktdmaChanStats;

/* @} */

/**
 *  \addtogroup CSL_PKTDMA_FUNCTION
 *  @{
 */

/**
 *  \brief Return revision of the PKTDMA module
 *
 *  This function returns the contents of the PKTDMA revision register.
 *  Consult the PKTDMA module documentation for a description of the
 *  contents of the revision register.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *
 *  \return The 32-bit revision register is returned.
 */
extern uint32_t CSL_pktdmaGetRevision( const CSL_PktdmaCfg *pCfg );

/**
 *  \brief Return revision information of the PKTDMA module
 *
 *  This function returns revision information for the PKTDMA module.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param pRev             [OUT]   Pointer to a #CSL_PktdmaRevision structure where the revision information is returned
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_pktdmaGetRevisionInfo( const CSL_PktdmaCfg *pCfg, CSL_PktdmaRevision *pRev );

/**
 *  \brief Initialize contents of a PKTDMA configuration structure
 *
 *  This function initializes the contents of the specified PKTDMA configuration
 *  structure.
 *
 *  All elements of the #CSL_PktdmaCfg structure are initialized to zero.
 *
 *  This function should not be called after calling CSL_pktdmaGetCfg as the
 *  PKTDMA module configuration stored in the PKTDMA configuration structure
 *  will be overwritten with zeros.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *
 *  \return None
 */
extern void CSL_pktdmaInitCfg( CSL_PktdmaCfg *pCfg );

/**
 *  \brief Return PKTDMA configuration information
 *
 *  This function returns configuration and capability information for the
 *  PKTDMA module. See the #CSL_PktdmaCfg structure for details on the
 *  information that is returned.
 *
 *  \param pCfg             [IN/OUT]    Pointer to the PKTDMA configuration structure
 *
 *  \return None
 */
extern void CSL_pktdmaGetCfg( CSL_PktdmaCfg *pCfg );

/**
 *  \brief Initialize a #CSL_PktdmaTxChanCfg structure
 *
 *  This function initializes the specified #CSL_PktdmaTxChanCfg structure to
 *  known, safe values. Software then only needs to configure elements
 *  that are different than their initialized values prior to calling the
 *  #CSL_pktdmaTxChanCfg function.
 *
 *  All elements of the #CSL_PktdmaTxChanCfg structure are initialized to zero
 *  except for the following:
 *
 *      chanType        = CSL_PKTDMA_CHAN_TYPE_NORMAL;
 *      fetchWordSize   = CSL_PKTDMA_FETCH_WORD_SIZE_16;
 *      trEventNum      = CSL_PKTDMA_NO_EVENT;
 *      errEventNum     = CSL_PKTDMA_NO_EVENT;
 *
 *  \param pTxChanCfg   [OUT]   Pointer to a #CSL_PktdmaTxChanCfg structure
 *
 *  \return None
 */
extern void CSL_pktdmaInitTxChanCfg( CSL_PktdmaTxChanCfg *pTxChanCfg );

/**
 *  \brief Initialize a #CSL_PktdmaRxChanCfg structure
 *
 *  This function initializes the specified #CSL_PktdmaRxChanCfg structure to
 *  known, safe values. Software then only needs to configure elements
 *  that are different than their initialized values prior to calling the
 *  #CSL_pktdmaRxChanCfg function.
 *
 *  All elements of the #CSL_PktdmaRxChanCfg structure are initialized to zero
 *  except for the following:
 *
 *      chanType            = CSL_PKTDMA_CHAN_TYPE_NORMAL;
 *      fetchWordSize       = CSL_PKTDMA_FETCH_WORD_SIZE_16;
 *      trEventNum          = CSL_PKTDMA_NO_EVENT;
 *      errEventNum         = CSL_PKTDMA_NO_EVENT;
 *      flowIdFwRangeCnt    = CSL_PKTDMA_RXCCFG_CHAN_RFLOW_RNG_FLOWID_CNT_RESETVAL;
 *
 *  \param pRxChanCfg   [OUT]   Pointer to a #CSL_PktdmaRxChanCfg structure
 *
 *  \return None
 */
extern void CSL_pktdmaInitRxChanCfg( CSL_PktdmaRxChanCfg *pRxChanCfg );

/**
 *  \brief Initialize a CSL_PktdmaRxFlowCfg structure
 *
 *  This function initializes the specified CSL_PktdmaRxFlowCfg structure to
 *  known, safe values. Software then only needs to configure elements
 *  that are different than their initialized values prior to calling the
 *  CSL_pktdmaRxFlowCfg function.
 *
 *  All elements of the CSL_PktdmaRxFlowCfg structure are initialized to zero.
 *
 *  \param pFlow        [OUT]   Pointer to a #CSL_PktdmaRxFlowCfg structure
 *
 *  \return None
 */
extern void CSL_pktdmaInitRxFlowCfg( CSL_PktdmaRxFlowCfg *pFlow );

/**
 *  \brief Set performance control parmeters
 *
 *  This function is used to set performance control paramaters available
 *  in the PKTDMA module.
 *
 *  \param pCfg                 [IN]    Pointer to the PKTDMA configuration structure
 *  \param rxRetryTimeoutCnt    [IN]    This parameter specifies the minimum
 *      amount of time (in clock cycles) that an Rx channel will be required
 *      to wait when it encounters a buffer starvation condition and the Rx
 *      error handling bit is set to 1
 *
 *  \return None
 */
extern void CSL_pktdmaSetPerfCtrl( CSL_PktdmaCfg *pCfg, uint32_t rxRetryTimeoutCnt );

/**
 *  \brief [udmap_only] Set UTC control parmeters
 *
 *  This function is used to set UTC control paramaters available
 *  in the PKTDMA module.
 *
 *  \param pCfg                 [IN]    Pointer to the PKTDMA configuration structure
 *  \param startingThreadNum    [IN]    This parameter specifies the starting
 *      PSI-L thread number for the external UTC
 *
 *  \return None
 */
extern void CSL_pktdmaSetUtcCtrl( CSL_PktdmaCfg *pCfg, uint32_t startingThreadNum );

/**
 *  \brief Configure an RX flow
 *
 *  This function initializes a receive flow with values specified in the
 *  #CSL_PktdmaRxFlowCfg structure.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param flow             [IN]    Index of the receive flow to initialize
 *  \param pFlow            [IN]    Pointer to a #CSL_PktdmaRxFlowCfg structure containing initialization values
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_pktdmaRxFlowCfg( CSL_PktdmaCfg *pCfg, uint32_t flow, const CSL_PktdmaRxFlowCfg *pFlow );

/**
 *  \brief Configure an RX channel
 *
 *  This function initializes a receive channel with values specified in the
 *  #CSL_PktdmaRxChanCfg structure.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel to initialize
 *  \param pRxChanCfg       [IN]    Pointer to a #CSL_PktdmaRxChanCfg structure containing initialization values
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_pktdmaRxChanCfg( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, const CSL_PktdmaRxChanCfg *pRxChanCfg );

/**
 *  \brief Configure a TX channel
 *
 *  This function initializes a transmit channel with values specified in the
 *  #CSL_PktdmaTxChanCfg structure.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel to initialize
 *  \param pTxChanCfg       [IN]    Pointer to a #CSL_PktdmaTxChanCfg structure containing initialization values
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_pktdmaTxChanCfg( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, const CSL_PktdmaTxChanCfg *pTxChanCfg );

/**
 *  \brief [udmap_only] Configure an RX channel TR event
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel to initialize
 *  \param trEventNum       [IN]    Specifies a global event number to generate
 *                                  anytime the required event generation
 *                                  criteria specified in a TR are met
 *                                  Set to CSL_PKTDMA_NO_EVENT for no event
 *                                  generation.
 *
 *  \return CSL_EUNSUPPORTED_CMD = Function is not supported
 */
extern int32_t CSL_pktdmaRxChanSetTrEvent( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, uint32_t trEventNum );

/**
 *  \brief [udmap_only]Configure an TX channel TR event
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel to initialize
 *  \param trEventNum       [IN]    Specifies a global event number to generate
 *                                  anytime the required event generation
 *                                  criteria specified in a TR are met
 *                                  Set to CSL_PKTDMA_NO_EVENT for no event
 *                                  generation.
 *
 *  \return CSL_EUNSUPPORTED_CMD = Function is not supported
 */
extern int32_t CSL_pktdmaTxChanSetTrEvent( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, uint32_t trEventNum );

/**
 *  \brief Configure RX channel burst size
 *
 *  This function enables configuration of the nominal burst size and alignment
 *  for data transfers on the specified RX channel. The default burst size
 *  is 64 bytes (a value of CSL_PKTDMA_CHAN_BURST_SIZE_64_BYTES).
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel to initialize
 *  \param burstSize        [IN]    Burst size value. See \ref CSL_PktdmaChanBurstSize
 *                                  for a list of valid burst size values.
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (burstSize is invalid or this
 *                      function is not available in the version of PKTDMA being used)
 */
extern int32_t CSL_pktdmaRxChanSetBurstSize( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanBurstSize burstSize );

/**
 *  \brief Configure TX channel burst size
 *
 *  This function enables configuration of the nominal burst size and alignment
 *  for data transfers on the specified TX channel. The default burst size
 *  is 64 bytes (a value of CSL_PKTDMA_CHAN_BURST_SIZE_64_BYTES).
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel to initialize
 *  \param burstSize        [IN]    Burst size value. See \ref CSL_PktdmaChanBurstSize
 *                                  for a list of valid burst size values.
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (burstSize is invalid or this
 *                      function is not available in the version of PKTDMA being used)
 */
extern int32_t CSL_pktdmaTxChanSetBurstSize( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanBurstSize burstSize );

/**
 *  \brief Get an RX channel's real-time register values
 *
 *  This function returns the real-time register values for the specified
 *  receive channel.
 *
 *  Note that no parameter error checking is performed by this function
 *  for performance reasons.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel
 *  \param pRT              [OUT]   Pointer to a #CSL_PktdmaRT structure where values are returned
 *
 *  \return CSL_PASS  = Function executed successfully
 */
extern int32_t CSL_pktdmaGetRxRT( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaRT *pRT );

/**
 *  \brief Get a TX channel's real-time register values
 *
 *  This function returns the real-time register values for the specified
 *  transmit channel.
 *
 *  Note that no parameter error checking is performed by this function
 *  for performance reasons.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel
 *  \param pRT              [OUT]   Pointer to a #CSL_PktdmaRT structure where values are returned
 *
 *  \return CSL_PASS  = Function executed successfully
 */
extern int32_t CSL_pktdmaGetTxRT( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaRT *pRT );

/**
 *  \brief Set an RX channel's real-time register values
 *
 *  This function sets the real-time register values for the specified
 *  receive channel.
 *
 *  Note that no parameter error checking is performed by this function
 *  for performance reasons.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel
 *  \param pRT              [IN]    Pointer to a #CSL_PktdmaRT structure containing initialization values
 *
 *  \return CSL_PASS  = Function executed successfully
 */
extern int32_t CSL_pktdmaSetRxRT( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, const CSL_PktdmaRT *pRT );

/**
 *  \brief Set a TX channel's real-time register values
 *
 *  This function sets the real-time register values for the specified
 *  transmit channel.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel
 *  \param pRT              [IN]    Pointer to a #CSL_PktdmaRT structure containing initialization values
 *
 *  Note that no parameter error checking is performed by this function
 *  for performance reasons.
 *
 *  \return CSL_PASS  = Function executed successfully
 */
extern int32_t CSL_pktdmaSetTxRT( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, const CSL_PktdmaRT *pRT );

/**
 *  \brief Enable a transmit channel.
 *
 *  This function enables the transmit channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_pktdmaEnableTxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Disable a transmit channel.
 *
 *  This function disables the transmit channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_pktdmaDisableTxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Teardown a transmit channel.
 *
 *  This function tears down the transmit channel specified by 'chanIdx' at the
 *  next packet boundary.
 *
 *  \param pCfg     [IN]    Pointer to the PKTDMA configuration structure
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
extern int32_t CSL_pktdmaTeardownTxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, bool bForce, bool bWait );

/**
 *  \brief Pause a transmit channel.
 *
 *  This function pauses the transmit channel specified by 'chanIdx' at the
 *  next packet boundary. This is a more graceful method of halting processing
 *  than disabling the channel as it will not allow any current packets to
 *  underflow.
 *
 *  \param pCfg     [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (channel is disabled)
 */
extern int32_t CSL_pktdmaPauseTxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Un-pause a transmit channel.
 *
 *  This function un-pauses the transmit channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (channel is disabled)
 */
extern int32_t CSL_pktdmaUnpauseTxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief [udmap_only] Send a trigger event to a TX channel
 *
 *  This function causes a trigger event to be sent to the specified transmit
 *  channel.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel
 *
 *  \return CSL_EUNSUPPORTED_CMD = Function is not supported
 */
extern int32_t CSL_pktdmaTriggerTxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Clear error indication in a transmit channel.
 *
 *  This function clears the error indication in the specified transmit channel.
 *
 *  Note that no parameter error checking is performed by this function
 *  for performance reasons.
 *
 *  \param pCfg     [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return None
 */
extern void CSL_pktdmaClearTxChanError( CSL_PktdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Enable a receive channel.
 *
 *  This function enables the receive channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_pktdmaEnableRxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Disable a receive channel.
 *
 *  This function disables the receive channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed
 */
extern int32_t CSL_pktdmaDisableRxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Teardown a receive channel.
 *
 *  This function tears down the receive channel specified by 'chanIdx' at the
 *  next packet boundary.
 *
 *  \param pCfg     [IN]    Pointer to the PKTDMA configuration structure
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
extern int32_t CSL_pktdmaTeardownRxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, bool bForce, bool bWait );

/**
 *  \brief Pause a receive channel.
 *
 *  This function pauses the receive channel specified by 'chanIdx' at the
 *  next packet boundary. This is a more graceful method of halting processing
 *  than disabling the channel as it will not allow any current packets to
 *  underflow.
 *
 *  \param pCfg     [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (channel is disabled)
 */
extern int32_t CSL_pktdmaPauseRxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Un-pause a receive channel.
 *
 *  This function un-pauses the receive channel specified by 'chanIdx'.
 *
 *  \param pCfg     [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx  [IN]    The index of the transmit channel
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (channel is disabled)
 */
extern int32_t CSL_pktdmaUnpauseRxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief [udmap_only] Send a trigger event to an RX channel
 *
 *  This function causes a trigger event to be sent to the specified receive
 *  channel.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the receive channel
 *
 *  \return CSL_EUNSUPPORTED_CMD = Function is not supported
 */
extern int32_t CSL_pktdmaTriggerRxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Clear error indication in a receive channel.
 *
 *  This function clears the error indication in the specified receive channel.
 *
 *  Note that no parameter error checking is performed by this function
 *  for performance reasons.
 *
 *  \param pCfg     [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx  [IN]    The index of the receive channel
 *
 *  \return None
 */
extern void CSL_pktdmaClearRxChanError( CSL_PktdmaCfg *pCfg, uint32_t chanIdx );

/**
 *  \brief Configure the receive flow ID range firewall
 *
 *  This function is used to configure the receive flow ID range firewall.
 *
 *  Note: This function is provided for backwards compatibility with the udmap
 *  CSL-FL.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param outEvtNum        [IN]    Output event number to use when the receive
 *                                  flow ID range firewall detects an error
 *
 *  \return None
 */
extern void CSL_pktdmaCfgRxFlowIdFirewall( CSL_PktdmaCfg *pCfg, uint32_t outEvtNum );

/**
 *  \brief Get receive flow ID range firewall status information
 *
 *  This function returns information from the receive flow ID range firewall.
 *
 *  If the receive flow ID firewall has detected an out of range flow ID,
 *  the function returns true and the fields within the
 *  #CSL_PktdmaRxFlowIdFirewallStatus structure contain error details. The
 *  function will automatically reset the receive flow ID firewall to capture
 *  the next error.
 *
 *  If the receive flow ID firewall has not detected an out of range flow ID,
 *  the function returns false and the fields within the
 *  #CSL_PktdmaRxFlowIdFirewallStatus structure are not updated.
 *
 *  \param pCfg                 [IN]    Pointer to the PKTDMA configuration structure
 *  \param pRxFlowIdFwStatus    [IN]    Pointer to a #CSL_PktdmaRxFlowIdFirewallStatus
 *                                      structure containing error details (valid
 *                                      only when true is returned)
 *
 *  \return true if the receive flow ID range firewall has detected an out of range
 *          flow ID, false if no error was detected
 */
extern bool CSL_pktdmaGetRxFlowIdFirewallStatus( CSL_PktdmaCfg *pCfg, CSL_PktdmaRxFlowIdFirewallStatus *pRxFlowIdFwStatus );

/**
 *  \brief Get channel statistics
 *
 *  This function is used to read statistics for a transmit or receive channel.
 *
 *  Note that no parameter error checking is performed by this function
 *  for performance reasons.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the channel
 *  \param chanDir          [IN]    Channel direction (see \ref CSL_PktdmaChanDir)
 *  \param pChanStats       [OUT]   Pointer to a #CSL_PktdmaChanStats structure
 *                                  where the statistics are returned
 *  \return None
 */
extern void CSL_pktdmaGetChanStats( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, CSL_PktdmaChanStats *pChanStats );

/**
 *  \brief Decrement channel statistics
 *
 *  Note that no parameter error checking is performed by this function
 *  for performance reasons.
 *
 *  This function is used to decrement statistics for a transmit or receive channel
 *  by the counts contained in the specified #CSL_PktdmaChanStats structure.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the channel
 *  \param chanDir          [IN]    Channel direction (see \ref CSL_PktdmaChanDir)
 *  \param pChanStats       [IN]    Pointer to a #CSL_PktdmaChanStats structure
 *                                  containing the counts to decrement each
 *                                  statistic by
 *  \return None
 */
extern void CSL_pktdmaDecChanStats( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, const CSL_PktdmaChanStats *pChanStats );

/**
 *  \brief Read a channel peer register
 *
 *  This function is used to read the value from a peer register for the
 *  specified channel.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the channel
 *  \param chanDir          [IN]    Channel direction (see \ref CSL_PktdmaChanDir)
 *  \param regIdx           [IN]    Peer register index (0..15)
 *  \param pVal             [OUT]   Pointer where the register value is returned
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (regIdx is out of range)
 */
extern int32_t CSL_pktdmaGetChanPeerReg( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, uint32_t regIdx, uint32_t *pVal );

/**
 *  \brief Write a channel peer register
 *
 *  This function is used to write a value to a peer register for the
 *  specified transmit channel.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the transmit channel
 *  \param chanDir          [IN]    Channel direction (see \ref CSL_PktdmaChanDir)
 *  \param regIdx           [IN]    Peer register index (0..15)
 *  \param pVal             [IN]    Pointer to the register value to be written
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (regIdx is out of range)
 */
extern int32_t CSL_pktdmaSetChanPeerReg( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, uint32_t regIdx, uint32_t *pVal );

/**
 *  \brief Enable a directional data flow for a paired link
 *
 *  This function is used to enable a directional data flow for a given PKTDMA channel between
 *  PKTDMA and a paired PSIL peer.
 *
 *  \param pCfg             [IN]    Pointer to the PKTDMA configuration structure
 *  \param chanIdx          [IN]    Index of the PKTDMA channel (TX or RX)
 *  \param chanDir          [IN]    Channel direction (TX or RX, see \ref CSL_PktdmaChanDir)
 *
 *  \return CSL_PASS  = Function executed successfully
 *          CSL_EFAIL = Function execution failed (chanIdx is invalid)
 */
extern int32_t CSL_pktdmaEnableLink( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir );

/* @} */

#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif  /* end of CSL_PKTDMA_H_ definition */
/** @} */

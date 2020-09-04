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
 *  \file  csl_lcdma_ringacc.h
 *
 *  \brief
 *  This CSL-FL header file contains various enumerations, structure
 *  definitions and function declarations for the Low Cost DMA ring
 *  accelerator (lcdma_ringacc) IP.
 *
 *  This CSL-FL was designed to be orthogonal with the implementation
 *  of the ringacc CSL-FL. Enumerations, structure definitions, and API
 *  functions are similarly named. The include file
 *  "csl_lcdma_ringacc_alias_ringacc_api.h" is available which maps
 *  lcdma_ringacc CSL-FL content to their ringacc equivalents for ease in
 *  porting existing ringacc code to the lcdma_ringacc.
 *
 *  There is CSL-FL content that is applicable for ringacc but not for
 *  lcdma_ringacc. Those items are denoted with the tag [ringacc_only]
 *  in the comments below and are handled as follows:
 *    - Enumerations: The enumeration is defined, but is not used functionally
 *    - Structure definitions: The structure is defined, but is not used
 *      functionally
 *    - Structure elements: The element is defined in the structure, but
 *      is not used functionally
 *    - API functions: The function is implemented, but does no operations
 *
 *  There is CSL-FL content that is applicable for lcdma_ringacc but not for
 *  ringacc. Those items are denoted with the tag [lcdma_ringacc_only]
 *  in the comments below.
 */
/**
 *  \ingroup CSL_IP_MODULE
 *  \defgroup CSL_LCDMA_RINGACC RINGACC CSL-FL
 *
 *  @{
 */

#ifndef CSL_LCDMA_RINGACC_H_
#define CSL_LCDMA_RINGACC_H_

#include <stdint.h>
#include <stdbool.h>
#include <drivers/hw_include/cslr_lcdma_ringacc.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup CSL_LCDMA_RINGACC_DATASTRUCT  RINGACC Data Structures
@ingroup CSL_LCDMA_RINGACC
*/
/**
@defgroup CSL_LCDMA_RINGACC_FUNCTION  RINGACC Functions
@ingroup CSL_LCDMA_RINGACC
*/
/**
@defgroup CSL_LCDMA_RINGACC_ENUM RINGACC Enumerated Data Types
@ingroup CSL_LCDMA_RINGACC
*/

/** ===========================================================================
 *  @addtogroup CSL_LCDMA_RINGACC_ENUM
    @{
 * ============================================================================
 */

/* The lcdma_ringacc has a hardcoded element size of 8 bytes */
#define CSL_LCDMA_RINGACC_RING_EL_SIZE_BYTES    ((uint32_t) 8U)
/** Maximum number of rings across devices */
#define CSL_LCDMA_RINGACC_MAX_RINGS     (1024U)
/** Maximum number of monitors across devices */
/** Maximum number of monitor interrupts across devices */
#define CSL_LCDMA_RINGACC_MAX_MONITOR_INTRS   (0U)
/** Bypass credential setting */
#define CSL_LCDMA_RINGACC_CRED_PASSTHRU       (0xFFFFFFFFU)
/** Bypass order ID setting */
#define CSL_LCDMA_RINGACC_ORDERID_BYPASS      (0xFFFFFFFFU)
/** Macro to pass to disable event */
#define CSL_LCDMA_RINGACC_RING_EVENT_DISABLE  (0xFFFFU)
/* Map old CSL_lcdma_ringaccMakeDescAddr function to new CSL_lcdma_ringaccSetAselInAddr equivalent */
#define CSL_lcdma_ringaccMakeDescAddr         CSL_lcdma_ringaccSetAselInAddr

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible modes for a ring or queue
 *
 *  \anchor CSL_LcdmaRingaccRingMode
 *  \name Ringacc ring mode
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_LcdmaRingaccRingMode;
    /** Exposed transmit ring mode with dual queues (forward/reverse) for SW direct access */
#define CSL_LCDMA_RINGACC_RING_MODE_TX_RING     ((uint32_t) 1U)
    /** Exposed receive ring mode with dual queues (forward/reverse) for SW direct access */
#define CSL_LCDMA_RINGACC_RING_MODE_RX_RING     ((uint32_t) 9U)
    /** Invalid */
#define CSL_LCDMA_RINGACC_RING_MODE_INVALID     ((uint32_t) 10U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief [ringacc_only] This enumerator defines the possible data sources that can be
 * monitored
 *
 *  \anchor CSL_LcdmaRingAccMonitorDataSrc
 *  \name Ringacc monitor data source
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_LcdmaRingAccMonitorDataSrc;
    /** Monitor the queue element count */
#define CSL_LCDMA_RINGACC_MONITOR_DATA_SRC_ELEMENT_CNT        ((uint32_t) 0U)
    /** Monitor the packet size at the head of a queue */
#define CSL_LCDMA_RINGACC_MONITOR_DATA_SRC_HEAD_PKT_SIZE      ((uint32_t) 1U)
    /** Monitor the accumulated data (in bytes) in a queue */
#define CSL_LCDMA_RINGACC_MONITOR_DATA_SRC_ACCUM_QUEUE_SIZE   ((uint32_t) 2U)
    /** Invalid */
#define CSL_LCDMA_RINGACC_MONITOR_DATA_SRC_INVALID            ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief [ringacc_only] This enumerator defines the possible monitor types
 *
 *  \anchor CSL_LcdmaRingAccMonitorType
 *  \name Ringacc monitor type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_LcdmaRingAccMonitorType;
    /** Monitor is disabled */
#define CSL_LCDMA_RINGACC_MONITOR_TYPE_DISABLED   ((uint32_t) 0U)
    /** Monitor tracks simple statistics on the queue operations. The first
        value is the count of the number of writes to the queue, and the second
        value is the count of the number of reads from the queue. */
#define CSL_LCDMA_RINGACC_MONITOR_TYPE_STATS      ((uint32_t) 1U)
    /** Monitor uses a programmed low threshold value and a programmed high
        threshold value to track the data source in the queue and cause an
        interrupt or status when a threshold is broken, either below the low
        value or above the high value */
#define CSL_LCDMA_RINGACC_MONITOR_TYPE_THRESHOLD  ((uint32_t) 2U)
    /** Monitor tracks the low and high values of the data source since the
        last read of the monitor */
#define CSL_LCDMA_RINGACC_MONITOR_TYPE_WATERMARK  ((uint32_t) 3U)
    /** Monitor tracks the number of starvation events (a read to an empty
        queue) that occurred on the queue */
#define CSL_LCDMA_RINGACC_MONITOR_TYPE_STARVATION ((uint32_t) 4U)
    /** Invalid */
#define CSL_LCDMA_RINGACC_MONITOR_TYPE_INVALID    ((uint32_t) 5U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible memory operation types
 *
 *  \anchor CSL_LcdmaRingAccMemoryOpsType
 *  \name Ringacc Ops type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_LcdmaRingAccMemoryOpsType;
/** Write operation */
#define CSL_LCDMA_RINGACC_MEM_OPS_TYPE_WR   ((uint32_t) 0U)
/** Read operation */
#define CSL_LCDMA_RINGACC_MEM_OPS_TYPE_RD   ((uint32_t) 1U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the valid address select (asel) endpoints
 *
 *  \anchor CSL_LcdmaRingAccAselEndpoint
 *  \name Ringacc address select (asel) endpoint
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_LcdmaRingAccAselEndpoint;
/** Physical address (normal) */
#define CSL_LCDMA_RINGACC_ASEL_ENDPOINT_PHYSADDR        ((uint32_t) 0U)
#if defined (SOC_AM64X) || defined (SOC_AM243X)
/** PCIE0 */
#define CSL_LCDMA_RINGACC_ASEL_ENDPOINT_PCIE0           ((uint32_t) 1U)
/** ARM ACP port: write-allocate cacheable, bufferable */
#define CSL_LCDMA_RINGACC_ASEL_ENDPOINT_ACP_WR_ALLOC    ((uint32_t) 14U)
/** ARM ACP port: read-allocate, cacheable, bufferable */
#define CSL_LCDMA_RINGACC_ASEL_ENDPOINT_ACP_RD_ALLOC    ((uint32_t) 15U)
#endif
/* @} */

/* @} */

/**
 *  \addtogroup CSL_LCDMA_RINGACC_DATASTRUCT
 *  @{
 */

/**
 *  \brief User-provided memory fence call-back function to perform
 *   memory sync operations. The operation to perform depends on the opsType
 *   flag and also on the CPU/cache architecture.
 *
 *  This function is called after a memory write is performed or before any
 *  memory read is performed.
 *  In case of write, the ringacc API function needs to insure the write has
 *  landed in memory before proceeding.
 *  In case of read, the ringacc API function needs to insure the read will
 *  land in the memory before proceeding.
 *
 *  \param pVirtAddr        [IN]    The virtual memory address written to
 *  \param size             [IN]    Number of bytes to writeback
 *  \param opsType          [IN]    \ref CSL_LcdmaRingAccMemoryOpsType
 *
 *  \return None
 */
typedef void (*CSL_lcdma_ringaccMemOpsFxnPtr)(void *pVirtAddr, uint32_t size, uint32_t opsType);

/** \brief CSL_LcdmaRingaccCfg contains information to configure the ring accelerator. */
typedef struct
{
    CSL_lcdma_ringacc_ring_cfgRegs  *pRingCfgRegs;  /**< [IN] Pointer to the ring configuration registers */
    CSL_lcdma_ringacc_ringrtRegs    *pRingRtRegs;   /**< [IN] Pointer to the ring real-time registers */
    CSL_lcdma_ringacc_credRegs      *pCredRegs;     /**< [IN] Pointer to the credentials registers */
    uint32_t                        maxRings;       /**< [IN] Number of rings supported by this ring accelerator */
} CSL_LcdmaRingaccCfg;

/** \brief CSL_LcdmaRingaccRingCfg contains information to configure a ring. */
typedef struct {
    void                        *virtBase;          /**< [IN] Virtual base address of the ring memory */
    uint64_t                    physBase;           /**< [IN] Physical base address of the ring memory */
    CSL_LcdmaRingaccRingMode    mode;               /**< [IN] Ring mode */
    uint32_t                    elCnt;              /**< [IN] Ring element count */
    uint32_t                    elSz;               /**< [IN] Ring element size in bytes (4,8,16,32,64,128,256) */
    uint32_t                    credChkSecure;      /**< [lcdma_ringacc_only] [IN] When set indicates that accesses to the ring realtime registers for this flow as well as the channel realtime registers for the corresponding channel(s) associated with this flow should only be permitted when the host transaction secure attribute matches the credSecure value. */
    uint32_t                    credSecure;         /**< [IN] Ring credential: Secure attribute (0=non-secure, all other values=secure) */
    uint32_t                    credPriv;           /**< [IN] Ring credential: Privilege attribute (0=user, 1=supervisor, 2=hypervisor, 3=hypervisor supervisor) */
    uint32_t                    credPrivId;         /**< [IN] Ring credential: Privilege ID attribute. If credPrivId==CSL_LCDMA_RINGACC_CRED_PASSTHRU (0xFFFFFFFFU), then priv ID is not replaced and existing value is passed through. */
    uint32_t                    credVirtId;         /**< [ringacc_only] [IN] Ring credential: Virtual ID attribute. If credVirtId==CSL_LCDMA_RINGACC_CRED_PASSTHRU (0xFFFFFFFFU), then virtual ID is not replaced and existing value is passed through. */
    uint32_t                    ringNum;            /**< (Private) Ring number (0-1023) */
    uint32_t                    wrOcc;              /**< (Private) Ring write-side occupancy count */
    uint32_t                    rdOcc;              /**< (Private) Ring read-side occupancy count */
    uint32_t                    wrIdx;              /**< (Private) Ring write-side index */
    uint32_t                    rdIdx;              /**< (Private) Ring read-side index */
    uint32_t                    asel;               /**< (Private) ASEL value extracted from physBase */
} CSL_LcdmaRingaccRingCfg;

/* @} */

/**
 *  \addtogroup CSL_LCDMA_RINGACC_FUNCTION
 *  @{
 */

/**
 *  \brief [lcdma_ringacc_only] Initialize a #CSL_LcdmaRingaccCfg structure
 *
 *  This function initializes the specified #CSL_LcdmaRingaccCfg structure to
 *  all 0's, except for the maxRings element which is populated with the
 *  maximum number of rings supported.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *
 *  \return None
 */
extern void CSL_lcdma_ringaccInitCfg( CSL_LcdmaRingaccCfg *pCfg );

/**
 *  \brief [ringacc_only] Return revision of the RingAcc module.
 *
 *  This function returns the contents of the RingAcc revision register.
 *  Consult the RingAcc module documentation for a description of the
 *  contents of the revision register.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *
 *  \return The 32-bit revision register is returned.
 */
extern uint32_t CSL_lcdma_ringaccGetRevision( const CSL_LcdmaRingaccCfg *pCfg );

/**
 *  \brief Initialize a #CSL_LcdmaRingaccRingCfg structure
 *
 *  This function initializes the specified #CSL_LcdmaRingaccRingCfg structure to
 *  known, safe values. Software then only needs to configure elements
 *  that are different than their initialized values prior to calling the
 *  #CSL_lcdma_ringaccInitRing function.
 *
 *  \param pRingCfg   [OUT]   Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *
 *  \return None
 */
extern void CSL_lcdma_ringaccInitRingCfg( CSL_LcdmaRingaccRingCfg *pRingCfg );

/**
 *  \brief Initialize the ring object
 *
 *  This function is used to initialize the ring object without configuring the
 *  ring.
 *
 *  \param ringNum      [IN]    The number of the ring (0-1023) to be initialized
 *  \param pRing        [OUT]   Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                              containing the ring configuration
 */
extern void CSL_lcdma_ringaccInitRingObj( uint32_t ringNum,
                                    CSL_LcdmaRingaccRingCfg *pRing );

/**
 *  \brief Initialize a ring
 *
 *  This function is used to initialize the ring specified by RingNum. A ring
 *  must be initialized prior to calling any of the following functions:
 *
 *      #CSL_lcdma_ringaccResetRing
 *      #CSL_lcdma_ringaccGetForwardRingPtr
 *      #CSL_lcdma_ringaccGetReverseRingPtr
 *      #CSL_lcdma_ringaccCommitToForwardRing
 *      #CSL_lcdma_ringaccAckReverseRing
 *      #CSL_lcdma_ringaccSetRingOrderId
 *
 *  To use this function, allocate a #CSL_LcdmaRingaccRingCfg structure and initialize
 *  the following structure elements. Then, pass a pointer to this structure
 *  along with the other required arguments.
 *
 *      virtBase    Virtual base address of the ring memory
 *      physBase    Physical base address of the ring memory
 *      mode        The mode of the ring
 *      elCnt       Ring element count
 *      elSz        Ring element size in bytes (4,8,16,32,64,128,256)
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The number of the ring (0-1023) to be initialized
 *  \param pRing        [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                              containing the ring configuration
 *
 *  \return 0 if successful, or -1 if an invalid argument is detected
 */
extern int32_t CSL_lcdma_ringaccInitRing( CSL_LcdmaRingaccCfg *pCfg,
                            uint32_t ringNum,
                            CSL_LcdmaRingaccRingCfg *pRing );

/**
 *  \brief [ringacc_only] Set the ring event
 *
 *  This function is used to set the ring event based on RingNum.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The number of the ring (0-1023) to be initialized
 *  \param evtNum       [IN]    Event number for the ring
 *
 *  \return 0 if successful, or -1 if an invalid argument is detected
 */
extern int32_t CSL_lcdma_ringaccSetEvent( CSL_LcdmaRingaccCfg *pCfg,
                                    uint32_t ringNum,
                                    uint32_t evtNum );

/**
 *  \brief Get the ring number associated with a ring
 *
 *  This function is used to get the ring number associated with the specified
 *  ring.
 *
 *  \param pRing        [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure containing
 *                              the ring configuration
 *
 *  \return Ring number
 */
extern uint32_t CSL_lcdma_ringaccGetRingNum( const CSL_LcdmaRingaccRingCfg *pRing );

/**
 *  \brief [ringacc_only] Specify the orderid value for a ring
 *
 *  This function is used to specify the orderid value for a ring's
 *  destination transactions. If orderId == CSL_LCDMA_RINGACC_ORDERID_BYPASS, then
 *  the orderid from the source transaction is used.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure containing
 *                              the ring configuration
 *  \param orderId      [IN]    The orderId value, or CSL_LCDMA_RINGACC_ORDERID_BYPASS
 *                              to use the orderid from the source transaction
 *
 *  \return None
 *
 */
extern void CSL_lcdma_ringaccSetRingOrderId( CSL_LcdmaRingaccCfg *pCfg, const CSL_LcdmaRingaccRingCfg *pRing, uint32_t orderId );

/**
 *  \brief [ringacc_only] Configure the security credentials for a ring
 *
 *  This function is used to configure the security credentials for a ring.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure containing
 *                              the ring configuration
 *  \param bEnable      [IN]    true = Region is enabled, false = disabled
 *  \param bLock        [IN]    true = Region is locked (region values cannot
 *                              be changed), false = region is not locked
 *
 *  \return None
 *
 */
extern void CSL_lcdma_ringaccCfgRingCred( CSL_LcdmaRingaccCfg *pCfg, const CSL_LcdmaRingaccRingCfg *pRing, bool bEnable, bool bLock );

/**
 *  \brief Reset a ring.
 *
 *  This function is used to reset a ring to a known, initial state. The
 *  following operations are performed when a ring is reset:
 •    - Reset the internal pointer to the ring base address
 •    - Set the number of entries in both the forward and reverse rings
 *    (FOCC and ROCC) to zero
 •    - Clear the teardown acknowledge bit
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure containing
 *                              the ring configuration
 *
 *  \return None
 */
extern void CSL_lcdma_ringaccResetRing( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing );

/**
 *  \brief Get pointer to next free forward ring element.
 *
 *  This function is used to get a pointer to the next free element of a
 *  forward ring. This pointer can then be used to write data into the ring.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure containing
 *                              the ring configuration
 *
 *  \return NULL if the ring is full, otherwise a void pointer to the next
 *          free forward ring element
 */
extern void *CSL_lcdma_ringaccGetForwardRingPtr( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing );

/**
 *  \brief Get pointer to next available reverse ring element.
 *
 *  This function is used to get a pointer to the next available reverse
 *  element of a ring. This pointer can then be used to read data from the ring.
 *
 *  After the data has been read from the ring, call the #CSL_lcdma_ringaccAckReverseRing
 *  to acknowledge and return the element(s) that have been read.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure containing
 *                              the ring configuration
 *
 *  \return NULL if the ring is empty, otherwise a void pointer to the next
 *          available reverse ring element
 */
extern void *CSL_lcdma_ringaccGetReverseRingPtr( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing );

/**
 *  \brief Write to the ring foward doorbell.
 *
 *  This function writes 'count' to the forward doorbell register of the
 *  specified ring.
 *
 *  Normally, an application does not need to call this function as the
 *  #CSL_lcdma_ringaccCommitToForwardRing function calls it.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number
 *  \param mode         [IN]    Ring mode. See #CSL_LcdmaRingaccRingMode.
 *                              This parameter is not used but is required for
 *                              API backwards compatibility.
 *  \param cnt          [IN]    The count to write
 */
static inline void CSL_lcdma_ringaccSetForwardDoorbell( CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum, CSL_LcdmaRingaccRingMode mode, int32_t cnt );
static inline void CSL_lcdma_ringaccSetForwardDoorbell( CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum, CSL_LcdmaRingaccRingMode mode, int32_t cnt )
{
    CSL_REG32_WR( &pCfg->pRingRtRegs->RING[ringNum].FDB, CSL_FMK(LCDMA_RINGACC_RINGRT_RING_FDB_CNT, (uint32_t)cnt) );
    return;
}

/**
 *  \brief Write to the ring reverse doorbell.
 *
 *  This function writes 'count' to the reverse doorbell register of the
 *  specified ring.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number
 *  \param mode         [IN]    Ring mode. See #CSL_LcdmaRingaccRingMode.
 *                              This parameter is not used but is required for
 *                              API backwards compatibility.
 *  \param cnt          [IN]    The count to write
 *
 *  \return      None
 */
static inline void CSL_lcdma_ringaccSetReverseDoorbell( CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum, CSL_LcdmaRingaccRingMode mode, int32_t cnt );

static inline void CSL_lcdma_ringaccSetReverseDoorbell( CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum, CSL_LcdmaRingaccRingMode mode, int32_t cnt )
{
    CSL_REG32_WR( &pCfg->pRingRtRegs->RING[ringNum].RDB, CSL_FMK(LCDMA_RINGACC_RINGRT_RING_RDB_CNT, (uint32_t)cnt) );
}


/**
 *  \brief Commit elements written to a ring.
 *
 *  This function is used to commit (execute) elements that have been written
 *  to a ring.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure containing
 *                              the ring configuration
 *  \param cnt          [IN]    The number of elements written since the last
 *                              call to #CSL_lcdma_ringaccCommitToForwardRing, or NULL
 *                              to commit all outstanding entries.
 *
 *  \return None
 */
static inline void CSL_lcdma_ringaccCommitToForwardRing( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, int32_t cnt );

static inline void CSL_lcdma_ringaccCommitToForwardRing( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, int32_t cnt )
{
    int32_t cntLocal = cnt;
    /*-------------------------------------------------------------------------
     * Init thisDbRingCnt to the largest positive value that can be written to
     * the forward doorbell field (a two's compliment value).
     *-----------------------------------------------------------------------*/
    int32_t thisDbRingCnt = (int32_t)((((uint32_t)CSL_LCDMA_RINGACC_RINGRT_RING_FDB_CNT_MAX + 1U) >> 1) - 1U);

    while( cntLocal > 0 )
    {
        if( cntLocal <= thisDbRingCnt )
        {
            thisDbRingCnt = cntLocal;
        }
        CSL_lcdma_ringaccSetForwardDoorbell( pCfg, pRing->ringNum, pRing->mode, thisDbRingCnt );
        cntLocal -= thisDbRingCnt;
    }
    pRing->wrOcc += (uint32_t)cnt;
    pRing->wrIdx = (pRing->wrIdx + (uint32_t)cnt) % pRing->elCnt;
}

/**
 *  \brief Acknowledge elements read from a ring.
 *
 *  This function is used to acknowledge and return elements that have been read
 *  from a ring.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure containing
 *                              the ring configuration
 *  \param cnt          [IN]    The number of elements read since the last
 *                              call to #CSL_lcdma_ringaccAckReverseRing, or NULL to ACK
 *                              all outstanding entries.
 *
 *  \return None
 */
static inline void CSL_lcdma_ringaccAckReverseRing( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, int32_t cnt );

static inline void CSL_lcdma_ringaccAckReverseRing( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, int32_t cnt )
{
    int32_t cntLocal = cnt;
    /*-------------------------------------------------------------------------
     * Init thisDbRingCnt to the most negative value that can be written to the
     * reverse doorbell field (a two's compliment value). Keep this value
     * positive for comparison and arithmetic operations. Negate it when
     * writing to the reverse doorbell register.
     *-----------------------------------------------------------------------*/
    int32_t thisDbRingCnt = (int32_t)(((uint32_t)CSL_LCDMA_RINGACC_RINGRT_RING_RDB_CNT_MAX + 1U) >> 1);

    while( cntLocal > 0 )
    {
        if( cntLocal <= thisDbRingCnt )
        {
            thisDbRingCnt = cntLocal;
        }
        CSL_lcdma_ringaccSetReverseDoorbell( pCfg, pRing->ringNum, pRing->mode, 0-thisDbRingCnt );
        cntLocal -= thisDbRingCnt;
    }
    pRing->rdOcc -= (uint32_t)cnt;
    pRing->wrOcc -= (uint32_t)cnt;
    pRing->rdIdx = (pRing->rdIdx + (uint32_t)cnt) % pRing->elCnt;
}

/**
 *  \brief [ringacc_only] Get the current forward ring index.
 *
 *  This function returns the current forward index for the specified
 *  ring.
 *
 *  Normally, an application does not need to call this function.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number (0-1023)
 *
 *  \return The current forward ring index is returned.
 *          0 is returned if ringNum is out of range.
 */
extern uint32_t CSL_lcdma_ringaccGetForwardRingIdx( const CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum );

/**
 *  \brief [ringacc_only] Get the current reverse ring index.
 *
 *  This function returns the current reverse index for the specified
 *  ring.
 *
 *  Normally, an application does not need to call this function.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number (0-1023)
 *
 *  \return The current reverse ring index is returned.
 *          0 is returned if ringNum is out of range.
 */
extern uint32_t CSL_lcdma_ringaccGetReverseRingIdx( const CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum );

/**
 *  \brief Get the forward occupancy of a ring.
 *
 *  This function returns the forward occupancy for the specified
 *  ring.
 *
 *  Normally, an application does not need to call this function.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number (0-1023)
 *  \param mode         [IN]    Ring mode. See #CSL_LcdmaRingaccRingMode.
 *                              This parameter is not used but is required for
 *                              API backwards compatibility.
 *
 *  \return The current forward occupancy is returned.
 *          0 is returned if ringNum is out of range.
 */
extern uint32_t CSL_lcdma_ringaccGetForwardRingOcc( const CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum, CSL_LcdmaRingaccRingMode mode );

/**
 *  \brief Get the reverse occupancy of a ring.
 *
 *  This function returns the reverse occupancy for
 *  the specified ring.
 *
 *  Normally, an application does not need to call this function.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number (0-1023)
 *  \param mode         [IN]    Ring mode. See #CSL_LcdmaRingaccRingMode.
 *                              This parameter is not used but is required for
 *                              API backwards compatibility.
 *
 *  \return The current reverse occupancy is returned.
 *          0 is returned if ringNum is out of range.
 */
extern uint32_t CSL_lcdma_ringaccGetReverseRingOcc( const CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum, CSL_LcdmaRingaccRingMode mode );

/**
 *  \brief [ringacc_only] Configure trace support
 *
 *  This function configures trace support. Tracing is automatically disabled
 *  before the specified trace configuration is written.

 *  When tracing is enabled (see the #CSL_lcdma_ringaccSetTraceEnable function) a
 *  trace output of all push, pop, and peek operations are output so that the
 *  traffic can be viewed at a later time.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param bTraceAll    [IN]    If true, operations to every ring are traced.
 *                              If false, only the ring specified by ringNum
 *                              is traced.
 *  \param bIncMsgData  [IN]    If true, message data is included in the trace
 *                              output.
 *  \param ringNum      [IN]    Specifies the ring whose operations are to be
 *                              traced. This parameter is ignored when
 *                              bTraceAll is true.
 *
 *  \return     0  = success
 *              -1 = trace support is not available or ringNum is out of range
 */
extern int32_t CSL_lcdma_ringaccCfgTrace( CSL_LcdmaRingaccCfg *pCfg, bool bTraceAll, bool bIncMsgData, uint32_t ringNum );

/**
 *  \brief Enable or disable trace support
 *
 *  This function enables or disables trace support. Be sure and configure
 *  trace support using the #CSL_lcdma_ringaccCfgTrace function before enabling it.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param bEnable      [IN]    If true, trace support is enabled. If false,
 *                              it is disabled.
 *
 *  \return     0  = success
 *              -1 = trace support is not available
 */
extern int32_t CSL_lcdma_ringaccSetTraceEnable( CSL_LcdmaRingaccCfg *pCfg, bool bEnable );

/**
 *  \brief [ringacc_only] Enable trace support
 *
 *  This function enables trace support. Be sure and configure tarce support
 *  using the #CSL_lcdma_ringaccCfgTrace function before enabling it.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *
 *  \return     0  = success
 *              -1 = trace support is not available
 */
extern int32_t CSL_lcdma_ringaccEnableTrace( CSL_LcdmaRingaccCfg *pCfg );

/**
 *  \brief [ringacc_only] Disable trace support
 *
 *  This function disables trace support.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *
 *  \return     0  = success
 *              -1 = trace support is not available
 */
extern int32_t CSL_lcdma_ringaccDisableTrace( CSL_LcdmaRingaccCfg *pCfg );

/**
 *  \brief [ringacc_only] Configure a ring monitor.
 *
 *  This function is used to configure a ring monitor.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param monNum       [IN]    The number of the monitor to configure
 *  \param monType      [IN]    The type of ring monitor. See
 *      \ref CSL_LcdmaRingAccMonitorType for the available monitor types.
 *  \param ringNum      [IN]    The number of the ring to monitor
 *  \param eventNum     [IN]    The number of the event to produce if the
 *      monitor thresholds are exceeded (used only for the
 *      CSL_LCDMA_RINGACC_MONITOR_TYPE_THRESHOLD or
 *      CSL_LCDMA_RINGACC_MONITOR_TYPE_STARVATION monitor types). A value of
 *      CSL_LCDMA_RINGACC_MONITOR_INTR_DISABLE disables interrupts for this monitor.
 *  \param dataSrc      [IN]    The type of data this monitor is tracking. See
 *      \ref CSL_LcdmaRingAccMonitorDataSrc for available data sources. This is only
 *      used for CSL_LCDMA_RINGACC_MONITOR_TYPE_THRESHOLD and
 *      CSL_LCDMA_RINGACC_MONITOR_TYPE_WATERMARK monitor types.
 *  \param data0Val     [IN]    This value contains the low threshold value
 *      for the CSL_LCDMA_RINGACC_MONITOR_TYPE_THRESHOLD or
 *      CSL_LCDMA_RINGACC_MONITOR_TYPE_STARVATION monitor types. It is not used for
 *      the other monitor types.
 *  \param data1Val     [IN]    This value contains the high threshold value
 *      for the CSL_LCDMA_RINGACC_MONITOR_TYPE_THRESHOLD or
 *      CSL_LCDMA_RINGACC_MONITOR_TYPE_STARVATION monitor types. It is not used for
 *      the other monitor types.
 *
 *  \return      0 = success
 *              -1 = Monitor functionality is not supported or an argument is
 *                   out of range
 */
extern int32_t CSL_lcdma_ringaccCfgRingMonitor( CSL_LcdmaRingaccCfg *pCfg,
                            uint32_t monNum,
                            CSL_LcdmaRingAccMonitorType monType,
                            uint32_t ringNum,
                            uint32_t eventNum,
                            CSL_LcdmaRingAccMonitorDataSrc dataSrc,
                            uint32_t data0Val,
                            uint32_t data1Val );

/**
 *  \brief [ringacc_only] Read a ring monitor.
 *
 *  This function is used to read data from an active ring monitor.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param monNum       [IN]    The number of the monitor to read
 *  \param pData0       [OUT]   A pointer to where the following value (dependingon the type of monitor) is written:
 *      o CSL_LCDMA_RINGACC_MONITOR_TYPE_STATS:      count of the number of writes to the queue
 *      o CSL_LCDMA_RINGACC_MONITOR_TYPE_THRESHOLD:  low threshold value
 *      o CSL_LCDMA_RINGACC_MONITOR_TYPE_WATERMARK:  low watermark value
 *      o CSL_LCDMA_RINGACC_MONITOR_TYPE_STARVATION: number of starvation events (a read to an empty queue)
 *  \param pData1       [OUT]   A pointer to where the following value (depending on the type of monitor) is written:
 *      o CSL_LCDMA_RINGACC_MONITOR_TYPE_STATS:      count of the number of reads from the queue
 *      o CSL_LCDMA_RINGACC_MONITOR_TYPE_THRESHOLD:  high threshold value
 *      o CSL_LCDMA_RINGACC_MONITOR_TYPE_WATERMARK:  high watermark value
 *      o CSL_LCDMA_RINGACC_MONITOR_TYPE_STARVATION: not used
 *
 *  \return      0 = success
 *              -1 = Monitor functionality is not supported, monNum is out of
 *                   range, or specified monitor is disabled
 */
extern int32_t CSL_lcdma_ringaccReadRingMonitor( const CSL_LcdmaRingaccCfg *pCfg, uint32_t monNum, uint32_t *pData0, uint32_t *pData1 );

/**
 *  \brief Push a 32-bit value to a ring
 *
 *  This function is used to push a 32-bit value to a ring.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param val              [IN]    32-bit value to write to the ring
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is full
 */
extern int32_t CSL_lcdma_ringaccPush32( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint32_t val, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Pop a 32-bit value from a ring
 *
 *  This function is used to pop a 32-bit value from a ring.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the popped value is returned
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty
 */
extern int32_t CSL_lcdma_ringaccPop32( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint32_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief [ringacc_only] Mimic a hardware pop of a 32-bit value from the head of a ring
 *
 *  This function is used to mimic a hardware pop operation from a ring. It
 *  can be called by software to pop values from a ring that is configured
 *  in Ring Mode, where software is the normal producer (pushing to the ring
 *  via the CSL_lcdma_ringaccPush32 function) and hardware is the normal consumer
 *  (popping from the ring), such as a TX free queue ring.
 *
 *  The ring must be configured with a 4-byte element size. A 4-byte
 *  value is popped from the ring head and returned.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the popped value is returned
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty
 */
extern int32_t CSL_lcdma_ringaccHwPop32( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint32_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Peek at a 32-bit value from a ring
 *
 *  This function is used to peek at a 32-bit value from a ring.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the peeked value is returned
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty
 */
extern int32_t CSL_lcdma_ringaccPeek32( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint32_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Push a 64-bit value to a ring
 *
 *  This function is used to push a 64-bit value to a ring.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param val              [IN]    64-bit value to write to the ring
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is full
 *          -2 = requested access size (8 bytes) is greater than ring element size
 */
extern int32_t CSL_lcdma_ringaccPush64( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t val, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Push multiple 64-bit values to a ring
 *
 *  This function is used to push multiple 64-bit values to a ring.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVals            [IN]    Pointer to an array of 64-bit values to write to the ring.
 *                                  This array should be at least numValues in size.
 *  \param numValues        [IN]    Number of 64-bit values to write to the ring
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return >0 = actual number of values written (ring was full before all values could be written)
 *           0 = success (all values written successfully)
 *          -1 = ring is full (no values written)
 *          -2 = requested access size (8 bytes) is greater than ring element size
 */
extern int32_t CSL_lcdma_ringaccPush64Multi( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVals, uint32_t numValues, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Pop a 64-bit value from a ring
 *
 *  This function is used to pop a 64-bit value from a ring.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the popped value is returned
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty
 *          -2 = requested access size (8 bytes) is greater than ring element size
 */
extern int32_t CSL_lcdma_ringaccPop64( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Pop multiple 64-bit values from a ring
 *
 *  This function is used to pop multiple 64-bit values from a ring.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVals            [OUT]   Pointer to an array of 64-bit values to write to the ring.
 *                                  This array should be at least numValues in size (or the size of the ring if numValues==0).
 *  \param numValues        [IN]    Number of 64-bit values to read from the ring. If 0, then
 *                                  all elements available in the ring are read.
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return >0 = actual number of values read (numValues==0 or ring was empty before all values could be read)
 *           0 = success (all requested values (numValues) read successfully)
 *          -1 = ring is empty (no values read)
 *          -2 = requested access size (8 bytes) is greater than ring element size
 */
extern int32_t CSL_lcdma_ringaccPop64Multi( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVals, uint32_t numValues, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief [ringacc_only] Mimic a hardware pop of a 64-bit value from the head of a ring
 *
 *  This function is used to mimic a hardware pop operation from a ring. It
 *  can be called by software to pop values from a ring that is configured
 *  in Ring Mode, where software is the normal producer (pushing to the ring
 *  via the CSL_lcdma_ringaccPush64 function) and hardware is the normal consumer
 *  (popping from the ring), such as a TX free queue ring.
 *
 *  The ring must be configured with an 8-byte element size. An 8-byte
 *  value is popped from the ring head and returned.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the popped value is returned
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty
 *          -2 = requested access size (8 bytes) is greater than ring element size
 */
extern int32_t CSL_lcdma_ringaccHwPop64( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Peek at a 64-bit value from a ring
 *
 *  This function is used to peek at a 64-bit value from a ring.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the peeked value is returned
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty
 *          -2 = requested access size (8 bytes) is greater than ring element size
 */
extern int32_t CSL_lcdma_ringaccPeek64( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Write data into a ring
 *
 *  This function is used to write data into a ring.
 *
 *  Note that software can only write to a given ring, or read from a given
 *  ring - it cannot write and read to/from a given ring.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param pData            [IN]    Pointer to the data to write
 *  \param numBytes         [IN]    The number of bytes to write
 *  \param pfMemOps         [IN]    Pointer to a memory fence call-back
 *                                  function (or NULL if not needed).
 *
 *  \return  0 = success
 *          -1 = ring is full
 *          -2 = requested access size is greater than ring element size
 */
extern int32_t CSL_lcdma_ringaccWrData( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint8_t *pData, uint32_t numBytes, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Read data from a ring
 *
 *  This function is used to read data from a ring.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param pData            [IN]    Pointer to where read data is returned
 *  \param numBytes         [IN]    The number of bytes to read
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty
 *          -2 = requested access size is greater than ring element size
 */
extern int32_t CSL_lcdma_ringaccRdData( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint8_t *pData, uint32_t numBytes, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Peek at data from a ring
 *
 *  This function is used to peek at data from a ring.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param pData            [IN]    Pointer to where read data is returned
 *  \param numBytes         [IN]    The number of bytes to peek
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -2 = requested access size is greater than ring element size
 */
extern int32_t CSL_lcdma_ringaccPeekData( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint8_t *pData, uint32_t numBytes, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Clear the asel field in an address
 *
 *  This function is used to clear the asel value in the specified address.
 *
 *  \param addr             [IN]    The 64-bit address
 *
 *  \return  The address with the asel field cleared is returned
 */
extern uint64_t CSL_lcdma_ringaccClrAselInAddr( uint64_t addr );

/**
 *  \brief Set the asel field in an address
 *
 *  This function is used to set the specified asel value in the specified address.
 *
 *  \param addr             [IN]    The address
 *  \param asel             [IN]    Address select (asel) endpoint value. See #CSL_LcdmaRingAccAselEndpoint.
 *
 *  \return  The address including the asel value is returned
 */
extern uint64_t CSL_lcdma_ringaccSetAselInAddr( uint64_t addr, CSL_LcdmaRingAccAselEndpoint asel );

/**
 *  \brief Return teardown completion status of a ring
 *
 *  This function returns the teardown completion status of the specified ring.
 *  It does this by reading and returning the value of the ring's
 *  tdown_complete bit.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number
 *
 *  \return  0 = false (no teardown completion)
 *           1 = true (teardown is complete)
 */
extern bool CSL_lcdma_ringaccIsTeardownComplete( const CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum );

/**
 *  \brief Acknowledge teardown completion of a ring
 *
 *  This function acknowledges the teardown completion of the specified ring.
 *  It does this by writing a '1' to the ring's tdown_ack field to acknowledge
 *  (and clear) the ring's corresponding tdown_complete bit.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number
 *
 *  \return  None
 */
extern void CSL_lcdma_ringaccAckTeardown( const CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum );

/**
 *  \brief Dequeue a value pushed to a ring in FIFO order
 *
 *  This function is used to dequeue (remove and return) a value pushed
 *  to the specified ring by software. Values are removed in FIFO order.
 *
 *  This function is intended to be called by software to retrieve
 *  unprocessed descriptors prior to resetting the ring via the
 *  #CSL_lcdma_ringaccResetRing function.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_LcdmaRingaccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_LcdmaRingaccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the dequeued value is returned
 *
 *  \return  0 = success
 *          -1 = ring is empty (there are no more values in the ring to dequeue)
 */
extern int32_t CSL_lcdma_ringaccDequeue( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVal );

/* @} */

#ifdef __cplusplus
}
#endif

#endif
/** @} */

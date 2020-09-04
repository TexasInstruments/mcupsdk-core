/*
 *  Copyright (C) 2016-2018 Texas Instruments Incorporated.
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
 *  \file  csl_ringacc.h
 *
 *  \brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the Ring Accelerator IP.
 */
/**
 *  \ingroup CSL_IP_MODULE
 *  \defgroup CSL_RINGACC RINGACC CSL-FL
 *
 *  @{
 */

#ifndef CSL_RINGACC_H_
#define CSL_RINGACC_H_

#include <stdint.h>
#include <stdbool.h>
#include <drivers/hw_include/ringacc/V0/cslr_ringacc.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup CSL_RINGACC_DATASTRUCT  RINGACC Data Structures
@ingroup CSL_RINGACC
*/
/**
@defgroup CSL_RINGACC_FUNCTION  RINGACC Functions
@ingroup CSL_RINGACC
*/
/**
@defgroup CSL_RINGACC_ENUM RINGACC Enumerated Data Types
@ingroup CSL_RINGACC
*/

/** ===========================================================================
 *  @addtogroup CSL_RINGACC_ENUM
    @{
 * ============================================================================
 */

/** Maximum number of rings across devices */
#define CSL_RINGACC_MAX_RINGS           (1024U)
/** Maximum number of monitors across devices */
#define CSL_RINGACC_MAX_MONITORS        (64U)
/** Maximum number of monitor interrupts across devices */
#define CSL_RINGACC_MAX_MONITOR_INTRS   (32U)
/** Bypass credential setting */
#define CSL_RINGACC_CRED_PASSTHRU       (0xFFFFFFFFU)
/** Bypass order ID setting */
#define CSL_RINGACC_ORDERID_BYPASS      (0xFFFFFFFFU)
/** Macro to pass to disable event */
#define CSL_RINGACC_RING_EVENT_DISABLE  (0xFFFFU)

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible modes for a ring or queue
 *
 *  \anchor CSL_RingAccRingMode
 *  \name Ringacc ring mode
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_RingAccRingMode;
    /** Exposed ring mode for SW direct access */
#define CSL_RINGACC_RING_MODE_RING          ((uint32_t) 0U)
    /** Messaging mode (all operations are through bus accesses) allowing
        multiple producers or consumers */
#define CSL_RINGACC_RING_MODE_MESSAGE       ((uint32_t) 1U)
    /** Credentials mode is message mode plus stores credentials with each
        message. This mode requires each operation to use 2 elements of
        storage since the credentials are stored in the second element,
        so the element count should be doubled. Any exposed memory should be
        protected by a firewall from unwanted access. */
#define CSL_RINGACC_RING_MODE_CREDENTIALS   ((uint32_t) 2U)
    /** Queue manager mode makes the ring act like a traditional queue manager
        queue. This takes the credentials mode and adds packet length per
        element, along with additional read only fields for element count
        and accumulated queue length. The QM mode only operates with an 8 byte
        element size (any other element size is illegal), and like in
        credentials mode each operation uses 2 element slots to store the
        credentials and length fields. */
#define CSL_RINGACC_RING_MODE_QM            ((uint32_t) 3U)
    /** Invalid */
#define CSL_RINGACC_RING_MODE_INVALID       ((uint32_t) 4U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible data sources that can be
 * monitored
 *
 *  \anchor CSL_RingAccMonitorDataSrc
 *  \name Ringacc monitor data source
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_RingAccMonitorDataSrc;
    /** Monitor the queue element count */
#define CSL_RINGACC_MONITOR_DATA_SRC_ELEMENT_CNT        ((uint32_t) 0U)
    /** Monitor the packet size at the head of a queue */
#define CSL_RINGACC_MONITOR_DATA_SRC_HEAD_PKT_SIZE      ((uint32_t) 1U)
    /** Monitor the accumulated data (in bytes) in a queue */
#define CSL_RINGACC_MONITOR_DATA_SRC_ACCUM_QUEUE_SIZE   ((uint32_t) 2U)
    /** Invalid */
#define CSL_RINGACC_MONITOR_DATA_SRC_INVALID            ((uint32_t) 3U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible monitor types
 *
 *  \anchor CSL_RingAccMonitorType
 *  \name Ringacc monitor type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_RingAccMonitorType;
    /** Monitor is disabled */
#define CSL_RINGACC_MONITOR_TYPE_DISABLED   ((uint32_t) 0U)
    /** Monitor tracks simple statistics on the queue operations. The first
        value is the count of the number of writes to the queue, and the second
        value is the count of the number of reads from the queue. */
#define CSL_RINGACC_MONITOR_TYPE_STATS      ((uint32_t) 1U)
    /** Monitor uses a programmed low threshold value and a programmed high
        threshold value to track the data source in the queue and cause an
        interrupt or status when a threshold is broken, either below the low
        value or above the high value */
#define CSL_RINGACC_MONITOR_TYPE_THRESHOLD  ((uint32_t) 2U)
    /** Monitor tracks the low and high values of the data source since the
        last read of the monitor */
#define CSL_RINGACC_MONITOR_TYPE_WATERMARK  ((uint32_t) 3U)
    /** Monitor tracks the number of starvation events (a read to an empty
        queue) that occurred on the queue */
#define CSL_RINGACC_MONITOR_TYPE_STARVATION ((uint32_t) 4U)
    /** Invalid */
#define CSL_RINGACC_MONITOR_TYPE_INVALID    ((uint32_t) 5U)
/* @} */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible memory operation types
 *
 *  \anchor CSL_RingAccMemoryOpsType
 *  \name Ringacc Ops type
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_RingAccMemoryOpsType;
/** Monitor is disabled */
#define CSL_RINGACC_MEM_OPS_TYPE_WR         ((uint32_t) 0U)
/** Monitor is disabled */
#define CSL_RINGACC_MEM_OPS_TYPE_RD         ((uint32_t) 1U)
/* @} */

/* @} */

/**
 *  \addtogroup CSL_RINGACC_DATASTRUCT
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
 *  \param opsType          [IN]    \ref CSL_RingAccMemoryOpsType
 *
 *  \return None
 */
typedef void (*CSL_ringaccMemOpsFxnPtr)(void *pVirtAddr, uint32_t size, uint32_t opsType);

/** \brief CSL_RingAccCfg contains information to configure the ring accelerator. */
typedef struct {
    CSL_ringacc_gcfgRegs        *pGlbRegs;          /**< Pointer to Global configuration registers */
    CSL_ringacc_cfgRegs         *pCfgRegs;          /**< Pointer to Configuration registers */
    CSL_ringacc_rtRegs          *pRtRegs;           /**< Pointer to Real-time configuration registers */
    CSL_ringacc_monitorRegs     *pMonRegs;          /**< Pointer to Monitor configuration registers */
    CSL_ringacc_fifosRegs       *pFifoRegs;         /**< Pointer to FIFO registers */
    CSL_ringacc_iscRegs         *pIscRegs;          /**< Pointer to ISC registers */
    uint32_t                    maxRings;           /**< Maximum number of rings supported in this ring accelerator configuration */
    uint32_t                    maxMonitors;        /**< Maximum number of monitors supported in this ring accelerator configuration */
    bool                        bTraceSupported;    /**< Indicates if trace functionality is supported in this ring accelerator configuration */
} CSL_RingAccCfg;

/** \brief CSL_RingAccRingCfg contains information to configure a ring. */
typedef struct CSL_RingAccRing_t {
    void                        *virtBase;          /**< Virtual base address of the ring memory */
    uint64_t                    physBase;           /**< Physical base address of the ring memory */
    CSL_RingAccRingMode         mode;               /**< Ring mode */
    uint32_t                    elCnt;              /**< Ring element count */
    uint32_t                    elSz;               /**< Ring element size (4,8,16,32,64,128,256) */
    uint32_t                    evtNum;             /**< Event number for the ring */
    uint32_t                    credSecure;         /**< Ring credential: Secure attribute (0=non-secure, all other values=secure) */
    uint32_t                    credPriv;           /**< Ring credential: Privilege attribute (0=user, 1=supervisor, 2=hypervisor, 3=hypervisor supervisor) */
    uint32_t                    credPrivId;         /**< Ring credential: Privilege ID attribute. If credPrivId==CSL_RINGACC_CRED_PASSTHRU (0xFFFFFFFFU), then priv ID is not replaced and existing value is passed through. */
    uint32_t                    credVirtId;         /**< Ring credential: Virtual ID attribute. If credVirtId==CSL_RINGACC_CRED_PASSTHRU (0xFFFFFFFFU), then virtual ID is not replaced and existing value is passed through. */
    uint32_t                    ringNum;            /**< (Private) Ring number (0-1023) */
    struct CSL_RingAccRing_t    *pPair;             /**< (Private) Pointer to paired ring (if any) or NULL */
    uint32_t                    rwIdx;              /**< (Private) Current read/write index */
    int32_t                     waiting;            /**< (Private) Entry cnt that need COMMIT(tx) or ACK(rx) */
    uint32_t                    occ;                /**< (Private) Ring occupancy count */
} CSL_RingAccRingCfg;

/* @} */

/**
 *  \addtogroup CSL_RINGACC_FUNCTION
 *  @{
 */

/**
 *  \brief Return revision of the RingAcc module.
 *
 *  This function returns the contents of the RingAcc revision register.
 *  Consult the RingAcc module documentation for a description of the
 *  contents of the revision register.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *
 *  \return The 32-bit revision register is returned.
 */
extern uint32_t CSL_ringaccGetRevision( const CSL_RingAccCfg *pCfg );

/**
 *  \brief Initialize a #CSL_RingAccRingCfg structure
 *
 *  This function initializes the specified #CSL_RingAccRingCfg structure to
 *  known, safe values. Software then only needs to configure elements
 *  that are different than their initialized values prior to calling the
 *  #CSL_ringaccInitRing function.
 *
 *  All elements of the #CSL_RingAccRingCfg structure are initialized to zero
 *  except for the following:
 *
 *      evtNum = CSL_RINGACC_RING_EVENT_DISABLE;   // 0xFFFFU
 *
 *  \param pRingCfg   [OUT]   Pointer to a #CSL_RingAccRingCfg structure
 *
 *  \return None
 */
extern void CSL_ringaccInitRingCfg( CSL_RingAccRingCfg *pRingCfg );

/**
 *  \brief Initialize the ring object
 *
 *  This function is used to initialize the ring object without configuring the
 *  ring.
 *
 *  \param ringNum      [IN]    The number of the ring (0-1023) to be initialized
 *  \param pRing        [OUT]   Pointer to a #CSL_RingAccRingCfg structure
 *                              containing the ring configuration
 */
extern void CSL_ringaccInitRingObj( uint32_t ringNum,
                                    CSL_RingAccRingCfg *pRing );

/**
 *  \brief Initialize a ring
 *
 *  This function is used to initialize the ring specified by RingNum. A ring
 *  must be initialized prior to calling any of the following functions:
 *
 *      #CSL_ringaccPairRing
 *      #CSL_ringaccResetRing
 *      #CSL_ringaccGetCmdRingPtr
 *      #CSL_ringaccGetRspRingPtr
 *      #CSL_ringaccCommitToCmdRing
 *      #CSL_ringaccAckRspRing
 *      #CSL_ringaccSetRingOrderId
 *
 *  To use this function, allocate a #CSL_RingAccRingCfg structure and initialize
 *  the following structure elements. Then, pass a pointer to this structure
 *  along with the other required arguments.
 *
 *      virtBase    Virtual base address of the ring memory
 *      physBase    Physical base address of the ring memory
 *      mode        The mode of the ring
 *      elCnt       Ring element count
 *      elSz        Ring element size in bytes (4,8,16,32,64,128,256)
 *      evtNum      Event number for the ring
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The number of the ring (0-1023) to be initialized
 *  \param pRing        [IN]    Pointer to a #CSL_RingAccRingCfg structure
 *                              containing the ring configuration
 *
 *  \return 0 if successful, or -1 if an invalid argument is detected
 */
extern int32_t CSL_ringaccInitRing( CSL_RingAccCfg *pCfg,
                            uint32_t ringNum,
                            CSL_RingAccRingCfg *pRing );

/**
 *  \brief Set the ring event
 *
 *  This function is used to set the the ring event based on RingNum.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The number of the ring (0-1023) to be initialized
 *  \param evtNum       [IN]    Event number for the ring
 *
 *  \return 0 if successful, or -1 if an invalid argument is detected
 */
extern int32_t CSL_ringaccSetEvent( CSL_RingAccCfg *pCfg,
                                    uint32_t ringNum,
                                    uint32_t evtNum );

/**
 *  \brief Get the ring number associated with a ring
 *
 *  This function is used to get the ring number associated with the specified
 *  ring.
 *
 *  \param pRing        [IN]    Pointer to a #CSL_RingAccRingCfg structure containing
 *                              the ring configuration
 *
 *  \return Ring number
 */
extern uint32_t CSL_ringaccGetRingNum( const CSL_RingAccRingCfg *pRing );

/**
 *  \brief Specify the orderid value for a ring
 *
 *  This function is used to specify the orderid value for a ring's
 *  destination transactions. If orderId == CSL_RINGACC_ORDERID_BYPASS, then
 *  the orderid from the source transaction is used.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_RingAccRingCfg structure containing
 *                              the ring configuration
 *  \param orderId      [IN]    The orderId value, or CSL_RINGACC_ORDERID_BYPASS
 *                              to use the orderid from the source transaction
 *
 *  \return None
 *
 */
extern void CSL_ringaccSetRingOrderId( CSL_RingAccCfg *pCfg, const CSL_RingAccRingCfg *pRing, uint32_t orderId );

/**
 *  \brief Configure the security credentials for a ring
 *
 *  This function is used to configure the security credentials for a ring.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_RingAccRingCfg structure containing
 *                              the ring configuration
 *  \param bEnable      [IN]    true = Region is enabled, false = disabled
 *  \param bLock        [IN]    true = Region is locked (region values cannot
 *                              be changed), false = region is not locked
 *
 *  \return None
 *
 */
extern void CSL_ringaccCfgRingCred( CSL_RingAccCfg *pCfg, const CSL_RingAccRingCfg *pRing, bool bEnable, bool bLock );

/**
 *  \brief Reset a ring.
 *
 *  This function is used to reset a ring. The ring is reset back to its
 *  original state where the ring occupancy is 0 (empty) and the ring read and
 *  write pointers are reset to 0.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_RingAccRingCfg structure containing
 *                              the ring configuration
 *
 *  \return None
 */
extern void CSL_ringaccResetRing( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing );

/**
 *  \brief Get pointer to next free command ring element.
 *
 *  This function is used to get a pointer to the next free element of a
 *  a transmit ring. This pointer can then be used to write data into the ring.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_RingAccRingCfg structure containing
 *                              the ring configuration
 *
 *  \return NULL if the ring is full, otherwise a void pointer to the next
 *          free command ring element
 */
extern void *CSL_ringaccGetCmdRingPtr( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing );

/**
 *  \brief Get pointer to next available response ring element.
 *
 *  This function is used to get a pointer to the next available receive
 *  element of a ring. This pointer can then be used to read data from the ring.
 *
 *  If the ring is paired, a successful return will also credit the paired
 *  command ring with an additional free entry.
 *
 *  After the data has been read from the ring, call the #CSL_ringaccAckRspRing
 *  to acknowledge and return the element(s) that have been read.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_RingAccRingCfg structure containing
 *                              the ring configuration
 *
 *  \return NULL if the ring is empty, otherwise a void pointer to the next
 *          available response ring element
 */
extern void *CSL_ringaccGetRspRingPtr( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing );

/**
 *  \brief Write to the ring doorbell.
 *
 *  This function writes 'count' to the command ring doorbell register of the
 *  specified ring.
 *
 *  Normally, an application does not need to call this function as the
 *  #CSL_ringaccCommitToCmdRing function calls it.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number (0-1023)
 *  \param cnt          [IN]    The count to write
 *
 *  \return      0 = success
 *              -1 = ringNum is out of range
 */
static inline int32_t CSL_ringaccSetRingDoorbell( CSL_RingAccCfg *pCfg, uint32_t ringNum, int32_t cnt );

static inline int32_t CSL_ringaccSetRingDoorbell( CSL_RingAccCfg *pCfg, uint32_t ringNum, int32_t cnt )
{
    int32_t retVal;

    if( ringNum < pCfg->maxRings )
    {
        CSL_REG32_WR( &pCfg->pRtRegs->RINGRT[ringNum].DB, CSL_FMK(RINGACC_RT_RINGRT_DB_CNT, (uint32_t)cnt) );
        retVal = 0;
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

/**
 *  \brief Commit elements written to a ring.
 *
 *  This function is used to commit (execute) elements that have been written
 *  to a ring.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_RingAccRingCfg structure containing
 *                              the ring configuration
 *  \param cnt          [IN]    The number of elements written since the last
 *                              call to #CSL_ringaccCommitToCmdRing, or NULL
 *                              to commit all outstanding entries.
 *
 *  \return None
 */
static inline void CSL_ringaccCommitToCmdRing( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, int32_t cnt );

static inline void CSL_ringaccCommitToCmdRing( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, int32_t cnt )
{
    int32_t cntLocal = cnt;

    if( cntLocal == 0 )
    {
        cntLocal = pRing->waiting;
    }
    if( cntLocal > 0 )
    {
        if( CSL_ringaccSetRingDoorbell( pCfg, pRing->ringNum, cntLocal ) == 0 )
        {
            pRing->waiting -= cntLocal;
        }
    }
}

/**
 *  \brief Acknowledge elements read from a ring.
 *
 *  This function is used to acknowledge and return elements that have been read
 *  from a ring.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param pRing        [IN]    Pointer to a #CSL_RingAccRingCfg structure containing
 *                              the ring configuration
 *  \param cnt          [IN]    The number of elements read since the last
 *                              call to #CSL_ringaccAckRspRing, or NULL to ACK
 *                              all outstanding entries.
 *
 *  \return None
 */
extern void CSL_ringaccAckRspRing( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, int32_t cnt );

/**
 *  \brief Configure trace support
 *
 *  This function configures trace support. Tracing is automatically disabled
 *  before the specified trace configuration is written.

 *  When tracing is enabled (see the #CSL_ringaccSetTraceEnable function) a
 *  trace output of all push, pop, and peek operations are output so that the
 *  traffic can be viewed at a later time.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
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
extern int32_t CSL_ringaccCfgTrace( CSL_RingAccCfg *pCfg, bool bTraceAll, bool bIncMsgData, uint32_t ringNum );

/**
 *  \brief Enable or disable trace support
 *
 *  This function enables or disables trace support. Be sure and configure
 *  trace support using the #CSL_ringaccCfgTrace function before enabling it.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param bEnable      [IN]    If true, trace support is enabled. If false,
 *                              it is disabled.
 *
 *  \return     0  = success
 *              -1 = trace support is not available
 */
extern int32_t CSL_ringaccSetTraceEnable( CSL_RingAccCfg *pCfg, bool bEnable );

/**
 *  \brief Enable trace support
 *
 *  This function enables trace support. Be sure and configure tarce support
 *  using the #CSL_ringaccCfgTrace function before enabling it.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *
 *  \return     0  = success
 *              -1 = trace support is not available
 */
extern int32_t CSL_ringaccEnableTrace( CSL_RingAccCfg *pCfg );

/**
 *  \brief Disable trace support
 *
 *  This function disables trace support.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *
 *  \return     0  = success
 *              -1 = trace support is not available
 */
extern int32_t CSL_ringaccDisableTrace( CSL_RingAccCfg *pCfg );

/**
 *  \brief Get the current ring read/write index.
 *
 *  This function returns the current command read/write index for the specified
 *  ring.
 *
 *  Normally, an application does not need to call this function.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number (0-1023)
 *
 *  \return The current ring read/write index is returned.
 *          0 is returned if ringNum is out of range.
 */
extern uint32_t CSL_ringaccGetRingIdx( const CSL_RingAccCfg *pCfg, uint32_t ringNum );

/**
 *  \brief Get the current ring hardware read/write index (for debug).
 *
 *  This function returns the current ring read/write index (as seen by HW)
 *  for the specified ring.
 *
 *  Normally, an application does not need to call this function.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number (0-1023)
 *
 *  \return The current ring read/write index is returned
 *          0 is returned if ringNum is out of range.
 */
extern uint32_t CSL_ringaccGetRingHwIdx( const CSL_RingAccCfg *pCfg, uint32_t ringNum );

/**
 *  \brief Get the current ring occupancy.
 *
 *  This function returns the current ring occupancy for the specified
 *  ring.
 *
 *  Normally, an application does not need to call this function.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number (0-1023)
 *
 *  \return The current command ring occupancy is returned
 *          0 is returned if ringNum is out of range.
 */
extern uint32_t CSL_ringaccGetRingOcc( const CSL_RingAccCfg *pCfg, uint32_t ringNum );

/**
 *  \brief Get the current ring hardware occupancy (for debug).
 *
 *  This function returns the current ring occupancy (as seen by HW) for
 *  the specified ring.
 *
 *  Normally, an application does not need to call this function.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param ringNum      [IN]    The ring number (0-1023)
 *
 *  \return The current command ring occupancy is returned
 *          0 is returned if ringNum is out of range.
 */
extern uint32_t CSL_ringaccGetRingHwOcc( const CSL_RingAccCfg *pCfg, uint32_t ringNum );

/**
 *  \brief Configure a ring monitor.
 *
 *  This function is used to configure a ring monitor.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param monNum       [IN]    The number of the monitor to configure
 *  \param monType      [IN]    The type of ring monitor. See
 *      \ref CSL_RingAccMonitorType for the available monitor types.
 *  \param ringNum      [IN]    The number of the ring to monitor
 *  \param eventNum     [IN]    The number of the event to produce if the
 *      monitor thresholds are exceeded (used only for the
 *      CSL_RINGACC_MONITOR_TYPE_THRESHOLD or
 *      CSL_RINGACC_MONITOR_TYPE_STARVATION monitor types). A value of
 *      CSL_RINGACC_MONITOR_INTR_DISABLE disables interrupts for this monitor.
 *  \param dataSrc      [IN]    The type of data this monitor is tracking. See
 *      \ref CSL_RingAccMonitorDataSrc for available data sources. This is only
 *      used for CSL_RINGACC_MONITOR_TYPE_THRESHOLD and
 *      CSL_RINGACC_MONITOR_TYPE_WATERMARK monitor types.
 *  \param data0Val     [IN]    This value contains the low threshold value
 *      for the CSL_RINGACC_MONITOR_TYPE_THRESHOLD or
 *      CSL_RINGACC_MONITOR_TYPE_STARVATION monitor types. It is not used for
 *      the other monitor types.
 *  \param data1Val     [IN]    This value contains the high threshold value
 *      for the CSL_RINGACC_MONITOR_TYPE_THRESHOLD or
 *      CSL_RINGACC_MONITOR_TYPE_STARVATION monitor types. It is not used for
 *      the other monitor types.
 *
 *  \return      0 = success
 *              -1 = Monitor functionality is not supported or an argument is
 *                   out of range
 */
extern int32_t CSL_ringaccCfgRingMonitor( CSL_RingAccCfg *pCfg,
                            uint32_t monNum,
                            CSL_RingAccMonitorType monType,
                            uint32_t ringNum,
                            uint32_t eventNum,
                            CSL_RingAccMonitorDataSrc dataSrc,
                            uint32_t data0Val,
                            uint32_t data1Val );

/**
 *  \brief Read a ring monitor.
 *
 *  This function is used to read data from an active ring monitor.
 *
 *  \param pCfg         [IN]    Pointer to a #CSL_RingAccCfg structure
 *                              containing the ring accelerator configuration
 *  \param monNum       [IN]    The number of the monitor to read
 *  \param pData0       [OUT]   A pointer to where the following value
 *      (dependingon the type of monitor) is written:
 *      o CSL_RINGACC_MONITOR_TYPE_STATS:      count of the number of writes
 *                                             to the queue
 *      o CSL_RINGACC_MONITOR_TYPE_THRESHOLD:  low threshold value
 *      o CSL_RINGACC_MONITOR_TYPE_WATERMARK:  low watermark value
 *      o CSL_RINGACC_MONITOR_TYPE_STARVATION: number of starvation events (a
 *                                             read to an empty queue)
 *  \param pData1       [OUT]   A pointer to where the following value
 *      (depending on the type of monitor) is written:
 *      o CSL_RINGACC_MONITOR_TYPE_STATS:      count of the number of reads
 *                                             from the queue
 *      o CSL_RINGACC_MONITOR_TYPE_THRESHOLD:  high threshold value
 *      o CSL_RINGACC_MONITOR_TYPE_WATERMARK:  high watermark value
 *      o CSL_RINGACC_MONITOR_TYPE_STARVATION: not used
 *
 *  \return      0 = success
 *              -1 = Monitor functionality is not supported, monNum is out of
 *                   range, or specified monitor is disabled
 */
extern int32_t CSL_ringaccReadRingMonitor( const CSL_RingAccCfg *pCfg, uint32_t monNum, uint32_t *pData0, uint32_t *pData1 );

/**
 *  \brief Push a 32-bit value to the tail of a ring
 *
 *  This function is used to push a 32-bit value to the tail of a ring.
 *  This function supports all ring modes.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_RingAccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_RingAccRingCfg structure
 *                                  containing the ring configuration
 *  \param val              [IN]    32-bit value to write to the ring
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed). The
 *                                  memory fence call-back is used for rings
 *                                  configured in ring mode only.
 *
 *  \return  0 = success
 *          -1 = ring is full (ring-mode rings only)
 */
extern int32_t CSL_ringaccPush32( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint32_t val, CSL_ringaccMemOpsFxnPtr pfMemOps);

/**
 *  \brief Pop a 32-bit value from the head of a ring
 *
 *  This function is used to pop a 32-bit value from the head of a ring. This function
 *  supports all ring modes.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_RingAccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_RingAccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the popped value is returned
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty (ring-mode rings only)
 */
extern int32_t CSL_ringaccPop32( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint32_t *pVal, CSL_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Mimic a hardware pop of a 32-bit value from the head of a ring
 *
 *  This function is used to mimic a hardware pop operation from a ring. It
 *  can be called by software to pop values from a ring that is configured
 *  in Ring Mode, where software is the normal producer (pushing to the ring
 *  via the CSL_ringaccPush32 function) and hardware is the normal consumer
 *  (popping from the ring), such as a TX free queue ring.
 *
 *  The ring must be configured with a 4-byte element size. A 4-byte
 *  value is popped from the ring head and returned.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_RingAccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_RingAccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the popped value is returned
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty
 */
extern int32_t CSL_ringaccHwPop32( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint32_t *pVal, CSL_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Peek at a 32-bit value from the head of a ring
 *
 *  This function is used to peek at a 32-bit value from the head of a ring.
 *  This function supports all ring modes.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_RingAccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_RingAccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the peeked value is returned
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty (ring-mode rings only)
 */
extern int32_t CSL_ringaccPeek32( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint32_t *pVal, CSL_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Push a 64-bit value to the tail of a ring
 *
 *  This function is used to push a 64-bit value to the tail of a ring.
 *  This function supports all ring modes.
 *
 *  This function can only be used when the calling processor is able to issue
 *  a 64-bit burst. If using a 32-bit processor, you should avoid using this
 *  function and instead use the proxy to read/write from/to rings.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_RingAccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_RingAccRingCfg structure
 *                                  containing the ring configuration
 *  \param val              [IN]    32-bit value to write to the ring
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed). The
 *                                  memory fence call-back is used for rings
 *                                  configured in ring mode only.
 *
 *  \return  0 = success
 *          -1 = ring is full (ring-mode rings only)
 *          -2 = requested access size (8 bytes) is greater than ring element size
 */
extern int32_t CSL_ringaccPush64( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint64_t val, CSL_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Pop a 64-bit value from the head of a ring
 *
 *  This function is used to pop a 64-bit value from the head of a ring.
 *  This function supports all ring modes.
 *
 *  This function can only be used when the calling processor is able to issue
 *  a 64-bit burst. If using a 32-bit processor, you should avoid using this
 *  function and instead use the proxy to read/write from/to rings.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_RingAccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_RingAccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the popped value is returned
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty (ring-mode rings only)
 *          -2 = requested access size (8 bytes) is greater than ring element size
 */
extern int32_t CSL_ringaccPop64( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint64_t *pVal, CSL_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Mimic a hardware pop of a 64-bit value from the head of a ring
 *
 *  This function is used to mimic a hardware pop operation from a ring. It
 *  can be called by software to pop values from a ring that is configured
 *  in Ring Mode, where software is the normal producer (pushing to the ring
 *  via the CSL_ringaccPush64 function) and hardware is the normal consumer
 *  (popping from the ring), such as a TX free queue ring.
 *
 *  The ring must be configured with an 8-byte element size. An 8-byte
 *  value is popped from the ring head and returned.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_RingAccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_RingAccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the popped value is returned
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty
 *          -2 = requested access size (8 bytes) is greater than ring element size
 */
extern int32_t CSL_ringaccHwPop64( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint64_t *pVal, CSL_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Peek at a 64-bit value from the head of a ring
 *
 *  This function is used to peek at a 64-bit value from the head of a ring.
 *  This function supports all ring modes.
 *
 *  This function can only be used when the calling processor is able to issue
 *  a 64-bit burst. If using a 32-bit processor, you should avoid using this
 *  function and instead use the proxy to read/write from/to rings.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_RingAccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_RingAccRingCfg structure
 *                                  containing the ring configuration
 *  \param pVal             [OUT]   Pointer where the peeked value is returned
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty (ring-mode rings only)
 *          -2 = requested access size (8 bytes) is greater than ring element size
 */
extern int32_t CSL_ringaccPeek64( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint64_t *pVal, CSL_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Write data into a ring (ring mode only)
 *
 *  This function is used to write data into a ring. It can only be used with a
 *  ring configured in ring mode. This function does nothing if the ring is any
 *  other mode.
 *
 *  Note that software can only write to a given ring, or read from a given
 *  ring - it cannot write and read to/from a given ring.
 *
 *  To write data to rings configured in modes other than ring mode,
 *  use the proxy.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_RingAccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_RingAccRingCfg structure
 *                                  containing the ring configuration
 *  \param pData            [IN]    Pointer to the data to write
 *  \param numBytes         [IN]    The number of bytes to write
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed).
 *
 *  \return  0 = success
 *          -1 = ring is full
 *          -2 = requested access size is greater than ring element size
 *          -3 = ring is configured in wrong mode (must be a ring mode ring)
 */
extern int32_t CSL_ringaccWrData( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint8_t *pData, uint32_t numBytes, CSL_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Read data from a ring (ring mode only)
 *
 *  This function is used to read data from a ring. It can only be used with a
 *  ring configured in ring mode. This function does nothing if the ring is any
 *  other mode.
 *
 *  To read data from rings configured in modes other than ring mode,
 *  use the proxy.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_RingAccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_RingAccRingCfg structure
 *                                  containing the ring configuration
 *  \param pData            [IN]    Pointer to where read data is returned
 *  \param numBytes         [IN]    The number of bytes to read
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -1 = ring is empty
 *          -2 = requested access size is greater than ring element size
 *          -3 = ring is configured in wrong mode (must be a ring mode ring)
 */
extern int32_t CSL_ringaccRdData( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint8_t *pData, uint32_t numBytes, CSL_ringaccMemOpsFxnPtr pfMemOps );

/**
 *  \brief Peek at data from a ring (ring mode only)
 *
 *  This function is used to peek at data from a ring. It can only be used with a
 *  ring configured in ring mode. This function does nothing if the ring is any
 *  other mode.
 *
 *  To peek at data from rings configured in modes other than ring mode,
 *  use the proxy.
 *
 *  \param pCfg             [IN]    Pointer to a #CSL_RingAccCfg structure
 *                                  containing the ring accelerator configuration
 *  \param pRing            [IN]    Pointer to a #CSL_RingAccRingCfg structure
 *                                  containing the ring configuration
 *  \param pData            [IN]    Pointer to where read data is returned
 *  \param numBytes         [IN]    The number of bytes to peek
 *  \param pfMemOps         [IN]    Pointer to a memory ops call-back
 *                                  function (or NULL if not needed)
 *
 *  \return  0 = success
 *          -2 = requested access size is greater than ring element size
 *          -3 = ring is configured in wrong mode (must be a ring mode ring)
 */
extern int32_t CSL_ringaccPeekData( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint8_t *pData, uint32_t numBytes, CSL_ringaccMemOpsFxnPtr pfMemOps );

/* @} */

#ifdef __cplusplus
}
#endif

#endif
/** @} */

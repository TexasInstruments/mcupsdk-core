/**
 * @file  csl_sec_proxy.h
 *
 * @brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the sec_proxy IP.
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2017-2019, Texas Instruments, Inc.
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
 *    Neither the name of Texas Instruments Incorposec_proxyed nor the names of
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
#ifndef CSL_SEC_PROXY_H_
#define CSL_SEC_PROXY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <drivers/sciclient/cslr_sec_proxy_v0.h>

/** ===========================================================================
 *
 * @defgroup CSL_SEC_PROXY_API Secure Proxy
 *
 * @section Introduction
 *
 * @subsection Overview
 *  This is the CSL-FL API documentation for the Secure Proxy module.
 *
 *  The following procedure describes how to properly use this CSL-FL API:
 *
 *  1. Allocate and initialize the #CSL_SecProxyCfg structure. A pointer to
 *     this structure must be provided to each API call, and the contents of
 *     this structure must be persistant across all secure proxy API calls by
 *     a given process.
 *
 *     For example, this structure would be initialized as follows for
 *     the navss_mcu:
 *
 *        CSL_SecProxyCfg navssMcuSecProxyCfg =
 *        {
 *            (CSL_sec_proxyRegs *)(BASE+0x285b0000UL),         // pSecProxyRegs
 *            (CSL_sec_proxy_scfgRegs *)(BASE+0x2a400000UL),    // pSecProxyScfgRegs
 *            (CSL_sec_proxy_rtRegs *)(BASE+0x2a380000UL),      // pSecProxyRtRegs
 *            (uint64_t)(BASE+0x2a480000UL),                    // proxyTargetAddr
 *            0                                                 // maxMsgSize
 *        };
 *
 *     This structure would be initialized as follows for the navss_main:
 *
 *        CSL_SecProxyCfg navssMainSecProxyCfg =
 *        {
 *            (CSL_sec_proxyRegs *)(BASE+0x31140000UL),         // pSecProxyRegs
 *            (CSL_sec_proxy_scfgRegs *)(BASE+0x32800000UL),    // pSecProxyScfgRegs
 *            (CSL_sec_proxy_rtRegs *)(BASE+0x32400000UL),      // pSecProxyRtRegs
 *            (uint64_t)(BASE+0x32c00000UL),                    // proxyTargetAddr
 *            0                                                 // maxMsgSize
 *        };
 *
 *  2. Configure the secure proxy by calling the #CSL_secProxyCfg function.
 *
 *  3. Each secure proxy data flow (outbound from the host processor to the
 *     target and inbound from the target to the host processor) requires a
 *     dedicated thread. For each of these threads...
 *       a) Initialize the #CSL_SecProxyThreadCfg structure
 *       b) Call #CSL_secProxyCfgThread to configure the thread
 *
 *     The #CSL_secProxyGetNumThreads function can be called to determine
 *     the number of threads supported by this secure proxy
 *
 *  4. Once the threads are configured, you can then write to or read from
 *     a target channel (queue) corresponding to a thread by calling
 *     #CSL_secProxyAccessTarget.
 *
 *  5. The following functions can be called to query and service events
 *     related to a thread:
 *       o #CSL_secProxyGetThreadMsgCnt
 *       o #CSL_secProxyIsThreadError
 *       o #CSL_secProxyClrThreadError
 *
 * @subsection References
 *    - sec_proxy Functional Specification, version 1.0.6
 *
 * ============================================================================
 */
/**
@defgroup CSL_SEC_PROXY_DATASTRUCT  SEC_PROXY Data Structures
@ingroup CSL_SEC_PROXY_API
*/
/**
@defgroup CSL_SEC_PROXY_FUNCTION  SEC_PROXY Functions
@ingroup CSL_SEC_PROXY_API
*/
/**
@defgroup CSL_SEC_PROXY_ENUM SEC_PROXY Enumerated Data Types
@ingroup CSL_SEC_PROXY_API
*/

/** ===========================================================================
 *  @addtogroup CSL_SEC_PROXY_ENUM
    @{
 * ============================================================================
 */

#define CSL_SEC_PROXY_RSVD_MSG_BYTES            ((uint32_t) 4U)
/** Depending on the version of the secure proxy module being used, some
    elements of the CSL_SecProxyThreadStatus structure are not available.
    In these cases, the element will have a returned value of
    CSL_SEC_PROXY_THREAD_STATUS_UNAVAILABLE. */
#define CSL_SEC_PROXY_THREAD_STATUS_UNAVAILABLE ((uint32_t) 0xFFFFFFFFU)
/** Disable global error event from being sent */
#define CSL_SEC_PROXY_GLOBAL_ERR_EVT_DISABLE    ((uint32_t) 0xFFFFU)

/** @} */

/** ============================================================================
 *  @addtogroup CSL_SEC_PROXY_DATASTRUCT
    @{
 * =============================================================================
 */

/** ---------------------------------------------------------------------------
 *  \brief   Callback function used to write/read to/from memory
 *
 *  This typedef defines a callback function that is used to write data into or
 *  read data from memory. In the write case, data is copied from pData to addr.
 *  In the read case, data is copied from addr to pData.
 *
 *  The parameters of this function are:
 *
 *  \param addr             [IN]    Address of the proxy data memory (src addr
 *                                  for reads, dst addr for writes)
 *  \param pData            [IN]    Pointer (virtual address) to data to be
 *                                  written to or read from
 *  \param elemSizeBytes    [IN]    Size in bytes of each data element. This
 *                                  value is typically 1 (for byte accesses) or
 *                                  4 (for 32-bit accesses).
 *  \param elemCnt          [IN]    Number of elements to write/read. The total
 *                                  # of bytes written/read is (elemCnt *
 *                                  elemSizeBytes).
 * ----------------------------------------------------------------------------
 */
typedef void (*CSL_SecProxyMemAccessCbFxnPtr)( uintptr_t addr, uint8_t *pData, uint32_t elemSizeBytes, uint32_t elemCnt );

/** \brief This structure provides status information for a sec_proxy thread */
typedef struct {
    uint32_t    error;              /** sec_proxy thread error: 0 = no error, 1 = error detected */
    uint32_t    dir;                /** Direction for the proxy thread: 0 = outbound, write only, 1 = inbound, read only, CSL_SEC_PROXY_THREAD_STATUS_UNAVAILABLE = information not available */
    uint32_t    maxMsgCnt;          /** Max message count allowed for an outbound proxy thread (valid only when dir==0). CSL_SEC_PROXY_THREAD_STATUS_UNAVAILABLE = information not available. */
    uint32_t    curMsgCnt;          /** Current message count. For an inbound proxy (dir==1), this is the number of available messages. For an outbound proxy (dir==0), this is the number of free messages that can be written. */
} CSL_SecProxyThreadStatus;

/** \brief This structure contains configuration parameters for each sec_proxy thread */
typedef struct {
    uint32_t    dir;                /** Direction for the proxy thread: 0 = outbound, write only, 1 = inbound, read only */
    uint32_t    outboundMaxMsgCnt;  /** Max message count allowed for an outbound proxy thread (valid only when dir==0) */
    uint32_t    outboundDstThread;  /** The proxy thread that is the destination of messages from this outbound proxy thread (valid only when dir==0) */
    uint32_t    queueNum;           /** Queue number in the target to use for the proxy thread. Note that this value is relative to the queue # inferred by the sec_proxy TARGET address MMR. */
    uint32_t    threshCnt;          /** Threshold count that causes proxy thread events */
    uint32_t    errEvtNum;          /** Event number for an error from the proxy thread */
    uint32_t    threshEvtNum;       /** Event number for a threshold event from the proxy thread */
} CSL_SecProxyThreadCfg;

/** \brief This structure contains configuration parameters for the sec_proxy IP */
typedef struct
{
    CSL_sec_proxyRegs           *pSecProxyRegs;         /** Pointer to the non-secure config MMR region */
    CSL_sec_proxy_scfgRegs      *pSecProxyScfgRegs;     /** Pointer to the secure config MMR region */
    CSL_sec_proxy_rtRegs        *pSecProxyRtRegs;       /** Pointer to the real-time MMR region */
    uint64_t                    proxyTargetAddr;        /** Address of proxy target data region (where the host writes to and reads from) */
    uint32_t                    maxMsgSize;             /* Internal variable for storing the max message size (read from CONFIG) */
} CSL_SecProxyCfg;

/** @} */

/** ===========================================================================
 *  @addtogroup CSL_SEC_PROXY_FUNCTION
    @{
 * ============================================================================
 */

/**
 *  \brief Return revision of the Secure Proxy module.
 *
 *  This function returns the contents of the Secure Proxy revision register.
 *  Consult the Secure Proxy module documentation for a description of the
 *  contents of the revision register.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *
 *  \return The 32-bit revision register is returned.
 */
extern uint32_t CSL_secProxyGetRevision( const CSL_SecProxyCfg *pSecProxyCfg );

/**
 *  \brief Get maximim message size
 *
 *  This function returns the maximum message size (in bytes).
 *
 *  Note that the value returned INCLUDES any message bytes that may be
 *  reserved for use by the secure proxy. Be sure and subtract any
 *  reserved bytes from the value returned by this function to get the
 *  usable message size, or call #CSL_secProxyGetMsgSize.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *
 *  \return     The maximum message size (in bytes) is returned.
 *
 */
static inline uint32_t CSL_secProxyGetMaxMsgSize( CSL_SecProxyCfg *pSecProxyCfg );
static inline uint32_t CSL_secProxyGetMaxMsgSize( CSL_SecProxyCfg *pSecProxyCfg )
{
    if( pSecProxyCfg->maxMsgSize == (uint32_t)0U )
    {
        pSecProxyCfg->maxMsgSize = (uint32_t)CSL_REG32_FEXT( &pSecProxyCfg->pSecProxyRegs->CONFIG, SEC_PROXY_CONFIG_MSG_SIZE );
    }
    return pSecProxyCfg->maxMsgSize;
}

/**
 *  \brief Configure the event number used for global errors
 *
 *  This function is used to configure the event number used for reporting
 *  global errors. This feature is available in secure proxy revisions 1.0.10.0
 *  and later. 
 *
 *  Global errors include the following:
 *      - Attempting to access an illegal proxy thread
 *
 *  Global errors do not include the following (there is a separate event
 *  number MMR per secure proxy for these errors):
 *      - Accessing reserved MMR locations within the secure proxy thread
 *      - Host violates the programmed setup of the secure proxy thread or
 *        accesses beyond the message size
 *
 *  Specifying CSL_SEC_PROXY_GLOBAL_ERR_EVT_DISABLE (0xFFFFU) for the
 *  globalErrEvtNum parameter will disable the error event from being sent.
 *
 *  \param  pSecProxyCfg    [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                                  containing the Secure Proxy configuration
 *  \param  globalErrEvtNum [IN]    Error event number
 *
 *  \return     0 = Success
 *             -1 = Feature is unavailable
 */
extern int32_t CSL_secProxyCfgGlobalErrEvtNum( const CSL_SecProxyCfg *pSecProxyCfg, uint32_t globalErrEvtNum );

/**
 *  \brief Get usable message size
 *
 *  This function returns the usable message size (in bytes).
 *
 *  Note that the value returned EXCLUDE any message bytes that may be
 *  reserved for use by the secure proxy.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *
 *  \return     The usable message size (in bytes) is returned.
 *
 */
static inline uint32_t CSL_secProxyGetMsgSize( CSL_SecProxyCfg *pSecProxyCfg )
{
    return CSL_secProxyGetMaxMsgSize(pSecProxyCfg) - CSL_SEC_PROXY_RSVD_MSG_BYTES;
}

/**
 *  \brief Get number of threads supported
 *
 *  This function returns the number of threads supported by the secure
 *  proxy.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *
 *  \return The number of threads supported by the secure proxy
 */
extern uint32_t CSL_secProxyGetNumThreads( const CSL_SecProxyCfg *pSecProxyCfg );

/**
 *  \brief Override the bus orderid value for the buffer memory access
 *
 *  This function is used to replace the bus orderid value for the buffer
 *  access with the specified orderid MMR value. This allows control over
 *  the orderid value when it must be restricted due to the topology for
 *  QoS reasons.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *  \param orderId      [IN]    Orderid value
 *
 *  \return None
 *
 */
extern void CSL_secProxySetBufferAccessOrderId( CSL_SecProxyCfg *pSecProxyCfg, uint32_t orderId );

/**
 *  \brief Configure the secure proxy
 *
 *  This function is used to configure the secure proxy. It must be called
 *  before reading or writing data via the secure proxy or calling the
 *  #CSL_secProxyAccessTarget function.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *  \param targetAddr   [IN]    Base address of the target resources (channels).
 *      If 0, the target address is not overwritten.
 *  \param extBufferAddr [IN]   Base address of the external buffer used by the
 *      secure proxy to store one message per proxy thread. The external buffer
 *      must be large enough to store one message per proxy thread, or
 *      #CSL_secProxyGetNumThreads() * #CSL_secProxyGetMaxMsgSize().
 *
 *  \return None
 */
extern void CSL_secProxyCfg( CSL_SecProxyCfg *pSecProxyCfg, uint64_t targetAddr, uint64_t extBufferAddr );

/**
 *  \brief Configure a proxy thread
 *
 *  This function is used to configure a proxy thread.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *  \param threadNum    [IN]    Thread number (0..#CSL_secProxyGetNumThreads()-1)
 *  \param pThreadCfg   [IN]    Pointer to a #CSL_SecProxyThreadCfg structure
 *                              containing the thread configuration
 *
 *  \return     0 = Success
 *             -1 = Invalid argument (threadNum out of range)
 *
 */
extern int32_t CSL_secProxyCfgThread( CSL_SecProxyCfg *pSecProxyCfg, uint32_t threadNum, const CSL_SecProxyThreadCfg *pThreadCfg );

/**
 *  \brief Get the read/write data address for the specified thread
 *
 *  This function returns the data address the application should use to
 *  read/write the specified number of bytes corresponding to the specified
 *  thread.
 *
 *  Note that for performance reasons, no error checking is performed by
 *  this function. For this reason, you must insure the arguments specified
 *  meet the requirements listed below.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *  \param threadNum    [IN]    Thread number (0..#CSL_secProxyGetNumThreads()-1)
 *  \param numBytes     [IN]    The number of bytes to be read or written
 *                    (0..CSL_secProxyGetMaxMsgSize(pSecProxyCfg)).
 *
 *  \return The data address is returned
 */
extern uintptr_t CSL_secProxyGetDataAddr( const CSL_SecProxyCfg *pSecProxyCfg, uint32_t threadNum, uint32_t numBytes );

/**
 *  \brief Access (read/write) from/to the specified thread
 *
 *  This function is used to read/write the specified number of bytes from/to
 *  to the specified thread. The access type (read or write) must match the
 *  direction configured for the specified thread.
 *
 *  Note that for performance reasons, no error checking is performed by
 *  this function. For this reason, you must insure the arguments specified
 *  meet the requirements listed below.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *  \param threadNum    [IN]    Thread number (0..#CSL_secProxyGetNumThreads()-1)
 *  \param pData        [IN]    A pointer (of type uint8_t*) to the data to be written
 *      (must be at least numBytes in size)
 *  \param numBytes     [IN]    The number of bytes to be written
 *      (0..CSL_secProxyGetMaxMsgSize(pSecProxyCfg)).
 *  \param fpMemAccess  [IN]    A pointer to a function that performs the memory access.
 *      See #CSL_SecProxyMemAccessCbFxnPtr for the syntax of this function.
 *
 *  \return None
 */
extern void CSL_secProxyAccessTarget( CSL_SecProxyCfg *pSecProxyCfg, uint32_t threadNum, uint8_t *pData, uint32_t numBytes, CSL_SecProxyMemAccessCbFxnPtr fpMemAccess );

/**
 *  \brief Get message count for thread
 *
 *  This function returns the current message count for the specified proxy
 *  thread. For an inbound proxy, this is the number of available messages.
 *  For an outbound proxy, this is the number of free messages that can be
 *  written.
 *
 *  Note that for performance reasons, no error checking is performed by
 *  this function. For this reason, you must insure the arguments specified
 *  meet the requirements listed below.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *  \param threadNum    [IN]    Thread number (0..#CSL_secProxyGetNumThreads()-1)
 *
 *  \return Message count
 */
static inline uint32_t CSL_secProxyGetThreadMsgCnt( const CSL_SecProxyCfg *pSecProxyCfg, uint32_t threadNum )
{
    return CSL_REG32_FEXT( &pSecProxyCfg->pSecProxyRtRegs->THREAD[threadNum].STATUS, SEC_PROXY_RT_THREAD_STATUS_CUR_CNT );
}

/**
 *  \brief Get thread error status
 *
 *  This function returns the error status of the specified thread.
 *
 *  If an error is detected on the thread, an error event (using the errEvtNum
 *  specified in the thread configuration) is generated. While in error, a
 *  thread will not process any operations. Call #CSL_secProxyClrThreadError
 *  to clear the error and reset the thread.
 *
 *  Note that for performance reasons, no error checking is performed by
 *  this function. For this reason, you must insure the arguments specified
 *  meet the requirements listed below.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *  \param threadNum    [IN]    Thread number (0..#CSL_secProxyGetNumThreads()-1)
 *
 *  \return 0=no error, 1=error detected on the thread
 */
static inline uint32_t CSL_secProxyIsThreadError( CSL_SecProxyCfg *pSecProxyCfg, uint32_t threadNum )
{
    return CSL_REG32_FEXT( &pSecProxyCfg->pSecProxyRtRegs->THREAD[threadNum].STATUS, SEC_PROXY_RT_THREAD_STATUS_ERROR );
}

/**
 *  \brief Clear thread error status
 *
 *  This function clears the error status of the specified thread. While in
 *  error, a thread will not process any operations.
 *
 *  Note that for performance reasons, no error checking is performed by
 *  this function. For this reason, you must insure the arguments specified
 *  meet the requirements listed below.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *  \param threadNum    [IN]    Thread number (0..#CSL_secProxyGetNumThreads()-1)
 *
 *  \return None
 */
static inline void CSL_secProxyClrThreadError( CSL_SecProxyCfg *pSecProxyCfg, uint32_t threadNum )
{
    CSL_REG32_FINS( &pSecProxyCfg->pSecProxyRtRegs->THREAD[threadNum].STATUS, SEC_PROXY_RT_THREAD_STATUS_ERROR, 0 );
}

/**
 *  \brief Get thread status
 *
 *  This function returns all status information availabile for the specified
 *  thread in a single call.
 *
 *  Note that depending on the version of the secure proxy module being used,
 *  some elements of the #CSL_SecProxyThreadStatus structure are not
 *  available. In these cases, the element will have a returned value of
 *  CSL_SEC_PROXY_THREAD_STATUS_UNAVAILABLE.
 *
 *  \param pSecProxyCfg [IN]    Pointer to a #CSL_SecProxyCfg structure
 *                              containing the Secure Proxy configuration
 *  \param threadNum    [IN]    Thread number (0..#CSL_secProxyGetNumThreads()-1)
 *  \param pStatus      [OUT]   Pointer to a #CSL_SecProxyThreadStatus structure
 *                              where status information is written
 *
 *  \return     0 = Success
 *             -1 = Invalid argument (threadNum out of range)
 */
extern int32_t CSL_secProxyGetThreadStatus( const CSL_SecProxyCfg *pSecProxyCfg, uint32_t threadNum, CSL_SecProxyThreadStatus *pStatus );

/** @} */

#ifdef __cplusplus
}
#endif

#endif

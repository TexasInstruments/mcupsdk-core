/** ============================================================================
 * Copyright (C) 2024 Texas Instruments Incorporated
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
 *    Neither the name of Texas Instruments Incorpoproxyed nor the names of
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
 * @file  csl_proxy.h
 *
 * @brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the proxy IP.
*/
#ifndef CSL_PROXY_H_
#define CSL_PROXY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <drivers/udma/hw_include/cslr_proxy.h>

/** ===========================================================================
 *
 * @defgroup CSL_PROXY_API Proxy
 *
 * Introduction
 *
 * Overview
 *  This is the CSL-FL API documentation for the Secure Proxy module.
 *
 *  The following procedure describes how to properly use this CSL-FL API:
 *
 *  1. Allocate and initialize an array of #CSL_ProxyTargetParams structures,
 *     one for each proxy target. A pointer to this array of structures is
 *     passed in the #CSL_ProxyCfg structure.
 *
 *     This array of structures would be initialized as follows for the
 *     navss_main, which has a single ringacc target:
 *
 *        CSL_ProxyTargetParams navssMainProxyTargetCfg[] =
 *        {
 *            // Target 0: ringacc0
 *            {
 *                (CSL_proxy_target0Regs *)0x000033000000UL,    // pTargetRegs
 *                3072U,                                        // numChns
 *                512U                                          // chnSizeBytes
 *            }
 *        };
 *
 *  2. Allocate and initialize the #CSL_ProxyCfg structure. A pointer to
 *     this structure is passed to all proxy API functions.
 *
 *     This structure would be initialized as follows for the navss_main:
 *
 *        CSL_ProxyCfg navssMainProxyCfg =
 *        {
 *            (CSL_proxyRegs *)0x000031120000UL,            // pGlbRegs
 *            (CSL_proxy_cfgRegs *)0x000033400000UL,        // pCfgRegs
 *            64,                                           // bufferSizeBytes
 *            1,                                            // numTargets
 *            navssMainProxyTargetCfg,                      // pProxyTargetParams
 *        };
 *
 *  3. Each proxy data flow (between the host and a target queue) requires a
 *     proxy thread. A proxy thread is simply a logical, bi-directional
 *     connection between the proxy IP block and a target queue.
 *     For each of these threads...
 *       a) Initialize the #CSL_ProxyThreadCfg structure
 *       b) Call #CSL_proxyCfgThread to configure the thread
 *       c) Call #CSL_proxyCfgThreadErrEvtNum to configure the thread
 *          error event number
 *
 *     The #CSL_proxyGetNumThreads function can be called to determine
 *     the number of threads supported by this proxy.
 *
 *  4. Once the threads are configured, you can then write to or read from
 *     a target channel (queue) corresponding to a thread by calling
 *     #CSL_proxySetQueueAccessMode to set the queue access mode, or
 *     #CSL_proxySetQueueParms to set all the queue parameters, and then
 *     #CSL_proxyAccessTarget to perform the data write or read.
 *
 *  5. The following functions can be called to query and service events
 *     related to a thread:
 *       o #CSL_proxyIsThreadError
 *       o #CSL_proxyClrThreadError
 *
 * References
 *    - proxy Functional Specification, version 1.0.15
 *
 * ============================================================================
 */
/**
@defgroup CSL_PROXY_DATASTRUCT  PROXY Data Structures
@ingroup CSL_PROXY_API
*/
/**
@defgroup CSL_PROXY_FUNCTION  PROXY Functions
@ingroup CSL_PROXY_API
*/
/**
@defgroup CSL_PROXY_ENUM PROXY Enumerated Data Types
@ingroup CSL_PROXY_API
*/

/** ===========================================================================
 *  @addtogroup CSL_PROXY_ENUM
    @{
 * ============================================================================
 */

/** Disable global error event from being sent */
#define CSL_PROXY_GLOBAL_ERR_EVT_DISABLE    ((uint32_t) 0xFFFFU)

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines valid queue access modes
 *
 *  \anchor CSL_ProxyQueueAccessMode
 *  \name Proxy queue access modes
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_ProxyQueueAccessMode;
    /** Push to or pop from head of queue */
#define CSL_PROXY_QUEUE_ACCESS_MODE_HEAD        (CSL_PROXY_TARGET0_PROXY_CTL_MODE_VAL_HEAD)
    /** Push to or pop from tail of queue */
#define CSL_PROXY_QUEUE_ACCESS_MODE_TAIL        (CSL_PROXY_TARGET0_PROXY_CTL_MODE_VAL_TAIL)
    /** Peek at value at head of queue */
#define CSL_PROXY_QUEUE_ACCESS_MODE_PEEK_HEAD   (CSL_PROXY_TARGET0_PROXY_CTL_MODE_VAL_PEEK_HEAD)
    /** Peek at value at tail of queue */
#define CSL_PROXY_QUEUE_ACCESS_MODE_PEEK_TAIL   (CSL_PROXY_TARGET0_PROXY_CTL_MODE_VAL_PEEK_TAIL)
/* @} */

/** @} */

/** ============================================================================
 *  @addtogroup CSL_PROXY_DATASTRUCT
    @{
 * =============================================================================
 */

/** ---------------------------------------------------------------------------
 *  @brief   Callback function used to write/read to/from memory
 *
 *  This typedef defines a callback function that is used to write data into or
 *  read data from memory. In the write case, data is copied from pAppData to
 *  pProxyData. In the read case, data is copied from pProxyData to pAppData.
 *
 *  The parameters of this function are:
 *
 *  \param addr             [IN]    Address of the proxy data memory (src addr
 *                                  for reads, dst addr for writes)
 *  \param pAppData         [IN]    Pointer to the application data memory (src
 *                                  addr for writes, dst addr for reads)
 *  \param elemSizeBytes    [IN]    Size in bytes of each data element. This
 *                                  value is typically 1 (for byte accesses) or
 *                                  4 (for 32-bit accesses).
 *  \param elemCnt          [IN]    Number of elements to write/read. The total
 *                                  # of bytes written/read is (elemCnt *
 *                                  elemSizeBytes).
 * ----------------------------------------------------------------------------
 */
typedef void (*CSL_ProxyMemAccessCbFxnPtr)( uintptr_t addr, uint8_t *pAppData, uint32_t elemSizeBytes, uint32_t elemCnt );

/** \brief This structure contains configuration parameters for each proxy thread */
typedef struct {
    uint32_t                mode;               /** Initial queue access mode (see \ref CSL_ProxyQueueAccessMode). Can be overridden by #CSL_proxySetQueueAccessMode or #CSL_proxySetQueueParms functions */
    uint32_t                elSz;               /** Queue (ring) element size in bytes (4,8,16,32,64,128,256,512) */
    uint32_t                queueNum;           /** Queue number in the target to use for the proxy thread */
    uint32_t                errEvtNum;          /** Event number for an error from the proxy thread */
} CSL_ProxyThreadCfg;

/** \brief This structure contains configuration parameters for each proxy target */
typedef struct
{
    CSL_proxy_target0Regs   *pTargetRegs;       /** Pointer to the target config regs region */
    uint32_t                numChns;            /** Number of target resource channels */
    uint32_t                chnSizeBytes;       /** Size (in bytes) of each target resource channel */
} CSL_ProxyTargetParams;

/** \brief   This structure contains configuration parameters for the proxy IP */
typedef struct
{
    CSL_proxyRegs           *pGlbRegs;              /** Pointer to the global config regs region */
    CSL_proxy_cfgRegs       *pCfgRegs;              /** Pointer to the config regs region */
    uint32_t                bufferSizeBytes;        /** Proxy buffer size */
    uint32_t                numTargets;             /** Number of targets supported */
    CSL_ProxyTargetParams   *pProxyTargetParams;    /** Pointer to an array of configuration parameters for each proxy target */
} CSL_ProxyCfg;

/** @} */

/*=============================================================================
 * Encode the ring element size as follows:
 *  0 = 4 bytes
 *  1 = 8 bytes
 *  2 = 16 bytes
 *  3 = 32 bytes
 *  4 = 64 bytes
 *  5 = 128 bytes
 *  6 = 256 bytes
 *  7 = 512 bytes
 *
 *  0 is returned if the element size != 4, 8, 16, 32, 64, 128, 256, or 512.
 *===========================================================================*/
static inline uint32_t CSL_proxyEncodeElementSize( uint32_t elSz );
static inline uint32_t CSL_proxyEncodeElementSize( uint32_t elSz )
{
    uint32_t i;

    for(i=7u; (i>0u) && (elSz != (1u<<(i+2U))); i--) {}
    return i;
}

/** ===========================================================================
 *  @addtogroup CSL_PROXY_FUNCTION
    @{
 * ============================================================================
 */
/**
 *  \brief Get number of threads supported
 *
 *  This function returns the number of threads supported by the proxy.
 *
 *  \param pProxyCfg    [IN]    Pointer to a #CSL_ProxyCfg structure
 *                              containing the Proxy configuration
 *
 *  \return The number of threads supported by the proxy
 */
extern uint32_t CSL_proxyGetNumThreads( const CSL_ProxyCfg *pProxyCfg );

/**
 *  \brief Configure a proxy thread
 *
 *  This function is used to configure a proxy thread.
 *
 *  \param pProxyCfg    [IN]    Pointer to a #CSL_ProxyCfg structure
 *                              containing the Proxy configuration
 *  \param targetNum    [IN]    The proxy target number (0..pProxyCfg->numTargets-1)
 *  \param threadNum    [IN]    Thread number (0..#CSL_proxyGetNumThreads()-1)
 *  \param pThreadCfg   [IN]    Pointer to a #CSL_ProxyThreadCfg structure
 *                              containing the thread configuration
 *
 *  \return     0 = Success
 *             -1 = Invalid argument (targetNum, threadNum, pProxyCfg->elSz, or
 *                  pProxyCfg->queueNum out of range)
 *
 */
extern int32_t CSL_proxyCfgThread( CSL_ProxyCfg *pProxyCfg, uint32_t targetNum, uint32_t threadNum, const CSL_ProxyThreadCfg *pThreadCfg );

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
 *  \param pProxyCfg    [IN]    Pointer to a #CSL_ProxyCfg structure
 *                              containing the Proxy configuration
 *  \param targetNum    [IN]    The proxy target number (0..pProxyCfg->numTargets-1)
 *  \param threadNum    [IN]    Thread number (0..#CSL_proxyGetNumThreads()-1)
 *  \param numBytes     [IN]    The number of bytes to be read or written
 *                    (0..CSL_proxyGetMaxMsgSize(pProxyCfg)).
 *
 *  \return The data address is returned
 */
extern uintptr_t CSL_proxyGetDataAddr( const CSL_ProxyCfg *pProxyCfg, uint32_t targetNum, uint32_t threadNum, uint32_t numBytes );

/**
 *  \brief Set the queue access mode
 *
 *  This function sets the queue access mode for the specified thread and
 *  target.
 *
 *  Note that for performance reasons, no error checking is performed by
 *  this function. For this reason, you must insure the arguments specified
 *  meet the requirements listed below.
 *
 *  \param pProxyCfg    [IN]    Pointer to a #CSL_ProxyCfg structure
 *                              containing the Proxy configuration
 *  \param targetNum    [IN]    The proxy target number (0..pProxyCfg->numTargets-1)
 *  \param threadNum    [IN]    Thread number (0..#CSL_proxyGetNumThreads()-1)
 *  \param mode         [IN]    Queue access mode (see \ref CSL_ProxyQueueAccessMode)
 *
 *  \return None
 */
static inline void CSL_proxySetQueueAccessMode( const CSL_ProxyCfg *pProxyCfg, uint32_t targetNum, uint32_t threadNum, CSL_ProxyQueueAccessMode mode );
static inline void CSL_proxySetQueueAccessMode( const CSL_ProxyCfg *pProxyCfg, uint32_t targetNum, uint32_t threadNum, CSL_ProxyQueueAccessMode mode )
{
    CSL_ProxyTargetParams *pProxyTargetParams = &((pProxyCfg->pProxyTargetParams)[targetNum]);
    CSL_REG32_FINS( &pProxyTargetParams->pTargetRegs->PROXY[threadNum].CTL, PROXY_TARGET0_PROXY_CTL_MODE, mode );
}

/**
 *  \brief Set the queue parameters
 *
 *  This function sets the queue parameters for the specified thread and
 *  target.
 *
 *  Note that for performance reasons, no error checking is performed by
 *  this function. For this reason, you must insure the arguments specified
 *  meet the requirements listed below.
 *
 *  \param pProxyCfg    [IN]    Pointer to a #CSL_ProxyCfg structure
 *                              containing the Proxy configuration
 *  \param targetNum    [IN]    The proxy target number (0..pProxyCfg->numTargets-1)
 *  \param threadNum    [IN]    Thread number (0..#CSL_proxyGetNumThreads()-1)
 *  \param queueNum     [IN]    Queue number in the target to use for the proxy thread
 *  \param elSz         [IN]    Queue (ring) element size in bytes (4,8,16,32,64,128,256,512)
 *  \param mode         [IN]    Queue access mode (see \ref CSL_ProxyQueueAccessMode)
 *
 *  \return None
 */
static inline void CSL_proxySetQueueParms( const CSL_ProxyCfg *pProxyCfg, uint32_t targetNum, uint32_t threadNum, uint32_t queueNum, uint32_t elSz, CSL_ProxyQueueAccessMode mode );
static inline void CSL_proxySetQueueParms( const CSL_ProxyCfg *pProxyCfg, uint32_t targetNum, uint32_t threadNum, uint32_t queueNum, uint32_t elSz, CSL_ProxyQueueAccessMode mode )
{
    uint32_t regVal;
    CSL_ProxyTargetParams *pProxyTargetParams = &((pProxyCfg->pProxyTargetParams)[targetNum]);
    regVal = CSL_FMK( PROXY_TARGET0_PROXY_CTL_ELSIZE,   CSL_proxyEncodeElementSize(elSz) )  |
             CSL_FMK( PROXY_TARGET0_PROXY_CTL_MODE,     (uint32_t)mode )                          |
             CSL_FMK( PROXY_TARGET0_PROXY_CTL_QUEUE,    queueNum );
    CSL_REG32_WR( &pProxyTargetParams->pTargetRegs->PROXY[threadNum].CTL, regVal );
}

/**
 *  \brief Get thread error status
 *
 *  This function returns the error status of the specified thread.
 *
 *  If an error is detected on the thread, an error event (using the errEvtNum
 *  specified in the thread configuration) is generated. While in error, a
 *  thread will not process any operations. Call #CSL_proxyClrThreadError
 *  to clear the error and reset the thread.
 *
 *  Note that for performance reasons, no error checking is performed by
 *  this function. For this reason, you must insure the arguments specified
 *  meet the requirements listed below.
 *
 *  \param pProxyCfg    [IN]    Pointer to a #CSL_ProxyCfg structure
 *                              containing the Proxy configuration
 *  \param targetNum    [IN]    The proxy target number (0..pProxyCfg->numTargets-1)
 *  \param threadNum    [IN]    Thread number (0..#CSL_proxyGetNumThreads()-1)
 *
 *  \return 0=no error, 1=error detected on the thread
 */
static inline uint32_t CSL_proxyIsThreadError( const CSL_ProxyCfg *pProxyCfg, uint32_t targetNum, uint32_t threadNum );
static inline uint32_t CSL_proxyIsThreadError( const CSL_ProxyCfg *pProxyCfg, uint32_t targetNum, uint32_t threadNum )
{
    CSL_ProxyTargetParams *pProxyTargetParams = &((pProxyCfg->pProxyTargetParams)[targetNum]);
    return CSL_REG32_FEXT( &pProxyTargetParams->pTargetRegs->PROXY[threadNum].STATUS, PROXY_TARGET0_PROXY_STATUS_ERROR );
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
 *  \param pProxyCfg    [IN]    Pointer to a #CSL_ProxyCfg structure
 *                              containing the Proxy configuration
 *  \param targetNum    [IN]    The proxy target number (0..pProxyCfg->numTargets-1)
 *  \param threadNum    [IN]    Thread number (0..#CSL_proxyGetNumThreads()-1)
 *
 *  \return None
 */
static inline void CSL_proxyClrThreadError( const CSL_ProxyCfg *pProxyCfg, uint32_t targetNum, uint32_t threadNum );
static inline void CSL_proxyClrThreadError( const CSL_ProxyCfg *pProxyCfg, uint32_t targetNum, uint32_t threadNum )
{
    CSL_ProxyTargetParams *pProxyTargetParams = &((pProxyCfg->pProxyTargetParams)[targetNum]);
    CSL_REG32_FINS( &pProxyTargetParams->pTargetRegs->PROXY[threadNum].STATUS, PROXY_TARGET0_PROXY_STATUS_ERROR, 0 );
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif

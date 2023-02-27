/**
 * @file  sdl_tog.h
 *
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the VBUSM Slave Timeout Gasket IP.
 *
 *  ============================================================================
 *  @n   (C) Copyright 2021-2023, Texas Instruments, Inc.
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
 *    Neither the name of Texas Instruments Incorpo_toged nor the names of
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
 *
 *  @ingroup SDL_STOG_MODULE
 *  @defgroup SDL_STOG_API Slave Timeout Gasket(STOG)
 *
 *   Provides the APIs for STOG.
 *  @{
 */

#ifndef SDL_TOG_H_
#define SDL_TOG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include "sdlr_tog.h"
#include <sdl/stog/v0/soc/sdl_tog_soc.h>

/**
 *
 *
 *  This is the SDL-FL API documentation for the VBUSM Slave Timeout Gasket module.
 *
 *  The following procedure describes how to properly use this SDL-FL API:
 *
 *  1. Call #SDL_TOG_init with setting Timeout Value to configure the desired timeout
 *     counter value if you wish a shorter timeout than the default maximum
 *     timeout.
 *  2. Enable/disable interrupt sources as needed by calling
 *     #SDL_TOG_setIntrEnable. Note that all interrupt sources are
 *     enabled by default.
 *  3. Call #SDL_TOG_start to start the timer counting
 *  4. If an interrupt is signaled from the gasket, then...
 *     a) Call #SDL_TOG_getIntrPending to determine the pending interrupt
 *        sources
 *     b) Clear interrupt source(s) by calling #SDL_TOG_clrIntrPending
 *     c) Service the cause of the interrupt as needed. If the system
 *        determines that it needs to flush all outstanding transactions
 *        (for instance, because the main SoC is in an error condition and is
 *        going to be reset), software may do this by calling
 *        #SDL_TOG_setFlushMode with true. Once all transactions
 *        are flushed, software should exit Flush mode by calling
 *        #SDL_TOG_setFlushMode with false. If the destination side
 *        is in reset, this should trigger hardware flush, keeping the gasket
 *        returning any transactions that arrive.
 *     d) The #SDL_TOG_getErrInfo function can be called to get detailed
 *        information about the error if needed.
 *     e) Ack the interrupt by calling #SDL_TOG_getIntrCount and
 *        #SDL_TOG_ackIntr
 *
*/

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor SDL_TOG_cfgCtrl
 *  \name TOG configuration flags
 *  @{
 */
/**
 * @brief   This enumerator defines the possible configuration flags
 *
 */
typedef uint8_t	SDL_TOG_cfgCtrl;
    /** Timeout Configuration Flag */
#define SDL_TOG_CFG_TIMEOUT         0x01U
    /** Pending interrupt Configuration Flag */
#define SDL_TOG_CFG_INTR_PENDING    0x02U
/** @} */

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */

/**
 * \brief  This structure contains timeout error information
 */

typedef struct
{
    /** Route ID - This indicates the Route ID of the captured transaction */
    uint32_t    routeId;
    /** Order ID - This indicates the Order ID of the captured transaction */
    uint32_t    orderId;
    /** Direction - This indicates whether the captured transaction was a read (1) or a write (0) */
    uint32_t    dir;
    /** Type - This indicates the error type: 0=Transaction Timeout, 1=Unexpected Response */
    uint32_t    type;
    /** Tag - This indicates the CID/RID/SID of the transaction */
    uint32_t    tag;
    /** Command ID - This indicates the original Command ID (SID/RID) of the command.
        This field is only valid on a Timeout Error, not on an Unexpected Transaction Error */
    uint32_t    commandId;
    /** Original Byte Count - If this is a timed out transaction, then this field
        represents the CBYTECNT value of the original command.  If this is an
        unexpected response transaction, then this field contains the value of
        the bytecnt of the unexpected transaction (sbytecnt or rbytecnt). */
    uint32_t    orgByteCnt;
    /** Current Byte Count - If this is a timed out transaction, this is the number of
        bytes that were not returned as of the time the transaction timed out.  If this
        is an unexpected response transaction, then this field is not applicable. */
    uint32_t    currByteCnt;
    /** Address - If the captured transaction was a Timeout Error, this field represents
        the address of the original transaction. If the error was an Unexpected Response
        error, then this field is not applicable. */
    uint64_t    address;
} SDL_TOG_errInfo;

/**
 * @brief   This structure contains TOG configuration information
 */
typedef struct {
	SDL_TOG_cfgCtrl	cfgCtrl;
	uint32_t 		timeoutVal;
	SDL_TOG_IntrSrc intrSrcs;
}SDL_TOG_config;

/** \brief Slave TOG Static Registers
 *
 *  This structure contains Slave TOG static registers
 *  The register values are not expected to change until a new configuration
 *  is done.
 *
 */
typedef struct {
    uint32_t PID;                       /**< Revision Register */
    uint32_t CFG;                       /**< Configuration Register */
    uint32_t ENABLE;                    /**< Enable Register */
    uint32_t FLUSH;                     /**< Flush Register */
    uint32_t TIMEOUT;                   /**< Timeout Value Register */
    uint32_t ERR;                       /**<Error Interrupt Enabled Status/Set Register */
} SDL_TOG_staticRegs;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


/**
 *  \brief Initialize TOG module with respect of instance
 *
 *  This function allows software to initialize TOG module
 *
 *  The pConfig contains the timeout value and interrupt source
 *
 *  \param instance     [IN]    TOG instance
 *  \param pConfig      [IN]    Pointer to store the configuration
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS instance is invalid or pConfig is NULL
 */
int32_t SDL_TOG_init(SDL_TOG_Inst instance, const SDL_TOG_config	*pConfig);

/**
 *  \brief Verifying written configuration with TOG module
 *
 *  This function allows software to verifying TOG module
 *
 *  The pConfig contains the timeout value and interrupt source
 *
 *  \param instance     [IN]    TOG instance
 *  \param pConfig      [IN]    Pointer to store the configuration
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS instance is invalid or pConfig is NULL
 */
int32_t SDL_TOG_verifyConfig(SDL_TOG_Inst instance, const SDL_TOG_config	*pConfig);

/**
 *  \brief Enable/disable interrupt source(s)
 *
 *  This function allows software to enable or disable the specified interrupt
 *  source(s).
 *
 *  The intrSrcs value is composed of a logical OR of the desired interrupt
 *  sources defined in #SDL_TOG_IntrSrc.
 *
 *  \param instance     [IN]    TOG instance
 *  \param intrSrcs     [IN]    Interrupt source(s) to set or clear
 *  \param enable       [IN]    If true, the interrupt source(s) are enabled.
 *                              If false, they are disabled.
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS instance or intrSrcs are invalid
 */
int32_t SDL_TOG_setIntrEnable( SDL_TOG_Inst instance, SDL_TOG_IntrSrc intrSrcs, bool enable );

/**
 *  \brief Clear pending interrupt source(s)
 *
 *  This function allows software to clear the specified pending interrupt
 *  source(s).
 *
 *  The intrSrc value is composed of a logical OR of the desired interrupt
 *  sources defined in #SDL_TOG_IntrSrc.
 *
 *  \param instance     [IN]    TOG instance
 *  \param intrSrc      [IN]    Interrupt source(s) to clear
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS instance or/and intrSrc are invalid
 */
 int32_t SDL_TOG_clrIntrPending(SDL_TOG_Inst instance, SDL_TOG_IntrSrc	intrSrc);

/**
 *  \brief Get masked (enabled) pending interrupt sources
 *
 *  This function returns the masked (enabled) pending interrupt sources.
 *
 *  The value returned is a logical OR of the masked pending interrupt sources
 *  defined in #SDL_TOG_IntrSrc.
 *
 *  \param instance     [IN]    TOG instance
 *  \param pPendInts    [OUT]   Pointer where pending interrupt sources is returned
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS instance is invalid or pPendInts are NULL
 */
 int32_t SDL_TOG_getIntrPending(SDL_TOG_Inst instance, SDL_TOG_IntrSrc	*pPendInts);

/**
 *  \brief Ack interrupt source
 *
 *  This function acknowledges an interrupt source by decrementing the number
 *  of pending interrupts corresponding to the specified interrupt source
 *  intrSrc by the specified ackCnt count.
 *
 *  Valid values of intrSrc are:
 *      - SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT
 *      - SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE
 *
 *  SDL_EFAIL is returned if intrSrc is:
 *      - SDL_TOG_INTRSRC_COMMAND_TIMEOUT
 *
 *  \param instance     [IN]    TOG instance
 *  \param intrSrc      [IN]    Interrupt source
 *  \param ackCnt       [IN]    Decrement count
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS pRegs is NULL, intrSrc is invalid
 *                      SDL_EFAIL    ackCnt == 0 or is > # of pending interrupts
 */
 int32_t SDL_TOG_ackIntr(SDL_TOG_Inst instance, SDL_TOG_IntrSrc intrSrc, uint32_t ackCnt);

/**
 *  \brief Start the timer counter
 *
 *  This function starts the timer counter.
 *
 *  \param instance     [IN]    TOG instance
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS pRegs is NULL
 */
 int32_t SDL_TOG_start(SDL_TOG_Inst instance);

/**
 *  \brief Stop the timer counter
 *
 *  This function stops the timer counter.
 *
 *  \param instance     [IN]    TOG instance
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS pRegs is NULL
 */
 int32_t SDL_TOG_stop(SDL_TOG_Inst instance);

/**
 *  \brief Reset the timeout functionality
 *
 *  This function resets the timeout functionality by stopping the timer
 *  counter and clearing the timer and eon counters to 0.
 *  Note the eon bits represents the number of times free-running counter
 *  reached the configured timeout
 *
 *  Note that the timeout value set via the #SDL_TOG_setTimeoutVal
 *  function is unaffected by this function.
 *
 *  \param instance     [IN]    TOG instance
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS instance is invalid
 */
int32_t SDL_TOG_reset(SDL_TOG_Inst instance);

/**
 *  \brief Get timeout error information
 *
 *  This function returns information about a captured transaction.
 *
 *  \param instance     [IN]    TOG instance
 *  \param pErrInfo     [OUT]   Pointer where the current number of occupied
 *                              read slots is returned
 *
 *  \return SDL_PASS      The function completed successfully and pErrInfo
 *                        contains valid information
 *          SDL_EBADARGS  pRegs or pErrInfo are NULL
 *          SDL_EFAIL     The function failed and there is no valid information
 *                        in pErrInfo. This is because...
 *                      a) there is no error pending
 *                      b) the error pending did not have any captured
 *                         information because there was an error pending in front
 *                         of it that had already captured information.
 */
 int32_t SDL_TOG_getErrInfo(SDL_TOG_Inst instance, SDL_TOG_errInfo	*pErrInfo);

/**
 *  \brief Readback Static configuration registers
 *
 *  This function reads back configuration registers that are static.
 *
 *  \param instance     [IN]    TOG instance
 *  \param pStaticRegs  [OUT]   Pointer to store the read static registers
 *
 *  \return SDL_PASS      Function completed successfully
 *          SDL_EBADARGS  Error with Arguments
 */
int32_t SDL_TOG_getStaticRegisters(SDL_TOG_Inst instance, SDL_TOG_staticRegs	*pStaticRegs);

/**
 *  \brief Enable/disable flush mode
 *
 *  This function allows software to enable/disable flush mode.
 *
 *  \param instance     [IN]    TOG instance
 *  \param enable       [IN]    If true, flush mode is enabled.
 *                              If false, it is disabled.
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS pRegs is NULL
 */
int32_t SDL_TOG_setFlushMode(SDL_TOG_Inst instance, bool enable);

/**
 *  \brief Get interrupt count
 *
 *  This function returns the number of pending interrupts corresponding to
 *  the specified interrupt source intrSrc as follows:
 *      0 = No pending interrupts
 *      1 = One pending interrupt
 *      2 = Two pending interrupts
 *      3 = Three or more pending interrupts
 *
 *  Valid values of intrSrc are:
 *      - SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT
 *      - SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE
 *
 *  SDL_EFAIL is returned if intrSrc is:
 *      - SDL_TOG_INTRSRC_COMMAND_TIMEOUT
 *
 *  \param instance     [IN]    TOG instance
 *  \param intrSrc      [IN]    Interrupt source
 *  \param pIntrCnt     [OUT]   Pointer where interrupt count is returned
 *
 *  \return             SDL_PASS       The function completed successfully
 *                      SDL_EBADARGS   pIntrCnt are NULL, or instance or intrSrc are invalid
 */
int32_t SDL_TOG_getIntrCount(SDL_TOG_Inst instance, SDL_TOG_IntrSrc intrSrc, uint32_t *pIntrCnt );

/** @} */



#ifdef __cplusplus
}
#endif

#endif	/* SDL_TOG_H_ */

/* nothing past this point */

/**
 * @file  sdl_ip_tog.h
 *
 * @brief
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
 * @ingroup SDL_STOG_API
 * @defgroup SDL_TOG_IP_API TOG Low-Level API
 * @{
 */
#ifndef SDL_IP_TOG_H_
#define SDL_IP_TOG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include "sdlr_tog.h"


/**
 *  This is the SDL-FL API documentation for the VBUSM Slave Timeout Gasket module.
 *
 *  The following procedure describes how to properly use this SDL-FL API:
 *
 *  1. Call #SDL_TOG_setTimeoutVal to configure the desired timeout
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
 *
 *  \anchor SDL_TOG_IntrSrc
 *  \name Timeout interrupt sources
 *  @{
 */
/**
 * @brief This enumerator defines the possible timeout interrupt sources
 *
 */
typedef uint32_t SDL_TOG_IntrSrc;

    /** Transaction timeout */
#define SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT ((uint32_t) 1U<<0)
    /** Unexpected response */
#define SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE ((uint32_t) 1U<<1)
    /** Command timeout */
#define SDL_TOG_INTRSRC_COMMAND_TIMEOUT     ((uint32_t) 1U<<2)
    /** All interrupt sources */
#define SDL_TOG_INTRSRC_ALL                 (SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT | SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE | SDL_TOG_INTRSRC_COMMAND_TIMEOUT)
/** @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Read internal interrupt count
 *
 *  This function reads the internal interrupt count.
 *
 *  \param baseAddr        	[IN]    Base address of the Timeout gasket registers
 *  \param intrSrc              [IN]    Interrupt source(s) to set
 *  \param pIntrCnt     	[OUT]   Pointer to interrupt source count
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS baseAddr and timeoutVal are invalid
 */
int32_t SDL_TOG_getIntrCountInternal(uint32_t baseAddr, SDL_TOG_IntrSrc intrSrc, uint32_t *pIntrCnt );

/**
 *  \brief Set the timeout value
 *
 *  This function sets the desired timeout value. Note the Timer runs on the
 *  VBUS clock and that determines the actual time.
 *
 *  \param baseAddr        	[IN]    Base address of the Timeout gasket registers
 *  \param timeoutVal   	[IN]    Timeout count value
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS baseAddr and timeoutVal are invalid
 */
 int32_t SDL_TOG_setTimeoutVal(uint32_t baseAddr, uint32_t timeoutVal );

/**
 *  \brief Set interrupt source(s)
 *
 *  This function allows software to set the specified interrupt source(s).
 *
 *  The intrSrcs value is composed of a logical OR of the desired interrupt
 *  sources defined in #SDL_TOG_IntrSrc.
 *
 *  \param baseAddr        	[IN]    Base address of the Timeout gasket registers
 *  \param intrSrc              [IN]    Interrupt source(s) to set
 *
 *  \return             SDL_PASS     The function completed successfully
 *                      SDL_EBADARGS baseAddr and intrSrcs is invalid
 */
int32_t SDL_TOG_setIntrPending(uint32_t baseAddr, SDL_TOG_IntrSrc intrSrc );

/** @} */

#ifdef __cplusplus
}
#endif

#endif	/* SDL_IP_TOG_H_ */

/* nothing past this point */

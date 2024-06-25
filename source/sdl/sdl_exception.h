/*
 * SDL EXCEPTION
 *
 * Software Diagnostics Library module for Exception handling
 *
 *  Copyright (c) Texas Instruments Incorporated 2022-2023
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
 * @file  sdl_exception.h
 *
 * @brief
 *  Header file contains enumerations, structure definitions and function
 *  declarations for SDL EXCEPTION interface.
 *
 *  The SDL exception data structures include:
 *      1. Structure of call back functions for various exception events
 *
 *  The SDL exception APIs include:
 *      1. API to execute SDL Exception module initialization
 *      2. APIs to handle various exceptions
 */

#ifndef INCLUDE_SDL_EXCEPTION_H_
#define INCLUDE_SDL_EXCEPTION_H_
#include "sdl_common.h"

#ifdef __cplusplus
extern "C" {
#endif


/** \brief Format of Call back function for ECC error events */
typedef void (*SDL_EXCEPTION_ECCCallback_t) (uint32_t intSource,
                                             uint32_t errorAddr,
                                             uint32_t ramId,
                                             uint64_t bitErrorOffset,
                                             uint32_t bitErrorGroup);

/** \brief Format of Call back function for exception */
typedef void (*SDL_EXCEPTION_Callback_t) (void);

/** \brief Structure of call back functions for various exception events */
typedef struct SDL_EXCEPTION_CallbackFunctions_s
{
    SDL_EXCEPTION_Callback_t udefExptnCallback;
    /**< Undefined Instruction exception callback function */
    SDL_EXCEPTION_Callback_t swiExptnCallback;
    /**< Software Interrupt exception callback function  */
    SDL_EXCEPTION_Callback_t pabtExptnCallback;
    /**< Prefetch Abort exception callback function */
    SDL_EXCEPTION_Callback_t dabtExptnCallback;
    /**< Data Abort exception callback function */
    SDL_EXCEPTION_Callback_t irqExptnCallback;
    /**< Interrupt Request exception callback function.
     *   This will be used only if VIM is not enabled.
     */
    SDL_EXCEPTION_Callback_t fiqExptnCallback;
    /**< Fast Interrupt Request callback function.
     *   This will be used only if VIM is not enabled.
     */
}SDL_EXCEPTION_CallbackFunctions_t;

/** ============================================================================
 *
 * \brief   Initialise Exception module
 *
 * \param  callbackFunctions: Pointer to callback Functions structure
 *
 */
void SDL_EXCEPTION_init(const SDL_EXCEPTION_CallbackFunctions_t *callbackFunctions);

/** ============================================================================
 *
 * \brief   Function to register ECC exception handler
 *
 * \param  ECCCallBackFunctionPtr: callback function to register
 *
 */
void SDL_EXCEPTION_registerECCHandler(SDL_EXCEPTION_ECCCallback_t ECCCallBackFunctionPtr);

/** ============================================================================
 *
 * \brief  Undefined Instruction Exception Handler
 *
 * \param  param: Parameter pointer
 *
 */
void SDL_EXCEPTION_undefInstructionExptnHandler(void *param);

/** ============================================================================
 *
 * \brief  SW Interrupt Exception Handler
 *
 * \param  param: Parameter pointer
 */
void SDL_EXCEPTION_swIntrExptnHandler(void *param);

/** ============================================================================
 *
 * \brief  Prefetch Abort Exception Handler
 *
 * \param  param: Parameter pointer
 *
 */
void SDL_EXCEPTION_prefetchAbortExptnHandler(void *param);

/** ============================================================================
 *
 * \brief  Data Abort Exception Handler
 *
 * \param  param: Parameter pointer
 *
 */
void SDL_EXCEPTION_dataAbortExptnHandler(void *param);

/** ============================================================================
 *
 * \brief  IRQ Exception Handler
 *
 * \param  param: Parameter pointer
 *
 */
void SDL_EXCEPTION_irqExptnHandler(void *param);

/** ============================================================================
 *
 * \brief  FIQ Exception Handler
 *
 * \param  param: Parameter pointer
 *
 */
void SDL_EXCEPTION_fiqExptnHandler(void *param);

#ifdef __cplusplus
}
#endif

#endif

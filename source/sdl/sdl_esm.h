/********************************************************************
 * Copyright (C) 2022-2024 Texas Instruments Incorporated.
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
 *  \defgroup SDL_ESM_MODULE APIs for SDL ESM
 *  \ingroup SDL_MODULE
 *
 *  This module contains APIs for using the ESM module. The APIs can be
 *  used to configure the ESM instances for notification when error events
 *  occur and also to set the error pin.
 *
 *  @{
 */
/**
 *   \file  sdl_esm.h
 *
 *   \brief This file contains the SDL ESM API's.
 */

#ifndef SDL_ESM_H_
#define SDL_ESM_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <sdl/include/soc_config.h>

#if defined (IP_VERSION_ESM_V0)
#include <sdl/esm/v0/v0_0/sdl_ip_esm.h>
#include <sdl/esm/v0/sdl_esm.h>
#include <sdl/esm/v0/v0_0/sdl_esm_priv.h>
#endif

#if defined (IP_VERSION_ESM_V2_0)
#include <sdl/esm/v2/v2_0/sdl_ip_esm.h>
#include <sdl/esm/v2/sdl_esm.h>
#include <sdl/esm/v2/v2_0/sdl_esm_priv.h>
#endif
/** ============================================================================
 *
 * \brief   SDL ESM API to get the status of the nError pin for the specified
 *          ESM instance
 *
 * \param   instance: ESM Instance
 * \param   pStatus: Pointer to variable to store the status.
 *                   If status is 1, then error pin is not active.
 *                   If status is 0, then error pin is active.
 *
 * \return  SDL_PASS if nError pin status is successfully retrieved.
 *          SDL_EBADARGS if instance or pStatus pointer are invalid.
 *          SDL_EFAIL if fail to read the error pin.
 */
int32_t SDL_ESM_getNErrorStatus(SDL_ESM_Inst instance, uint32_t *pStatus);

/** ============================================================================
 *
 * \brief   SDL ESM API to read the static registers. The API reads and returns
 *          the static register configuration for the ESM module for the specified
 *          instance. This API can be used by the application to read back the
 *          static configuration. Comparision of the static configuration registers
 *          against expected values is the responsibility of the application.
 *
 * \param   instance: ESM Instance
 * \param   pStaticRegs: Pointer to the static config register structure
 *
 * \return  SDL_PASS if registers are successfully read.
 *          SDL_EBADARGS if instance or pStaticRegs are invalid.
 */
int32_t SDL_ESM_getStaticRegisters(SDL_ESM_Inst instance, SDL_ESM_staticRegs *pStaticRegs);

/** ============================================================================
 *
 * \brief   SDL ESM API to verify the written configuration of the ESM module.
 *          The API verifies the written config that was done during SDL_ESM_init
 *          against the provided configuration.
 *
 * \param   instance: ESM Instance
 * \param   pConfig: Pointer to the ESM configuration to be used for verification.
 *
 * \return  SDL_PASS if Verification passed
 *          SDL_EBADARGS if instance or pConfig are invalid.
 *          SDL_EFAIL if verification failed.
 */
int32_t SDL_ESM_verifyConfig(SDL_ESM_Inst instance, const SDL_ESM_config *pConfig);

/** ============================================================================
 *
 * \brief   SDL ESM API to clear the nError pin for the specified ESM instance
 *
 * \param   instance: ESM Instance
 *
 * \return  SDL_PASS if nError pin status is successfully cleared.
 *          SDL_EBADARGS if instance is invalid.
 */
int32_t SDL_ESM_clrNError(SDL_ESM_Inst instance);

/** ============================================================================
 *
 * \brief   SDL ESM API to set the nError pin for the specified ESM instance
 *
 * \param   instance: ESM Instance
 *
 * \return  SDL_PASS if nError pin status is successfully set.
 *          SDL_EBADARGS if instance is invalid.
 */
int32_t SDL_ESM_setNError(SDL_ESM_Inst instance);

/** ============================================================================
 *
 * \brief  There are modules within SDL which will generate ESM errors
 *         intentionally in the course of running self-tests. The ECC module is
 *         one such module. To allow these modules to get the notification when
 *         the ESM error occurs, callback registration APIs are provided. The
 *         following APIs allow registration of a callback for specific events.
 *         This API is used by other SDL modules and not by the application
 *
 * \param   instance: ESM Instance
 * \param   eventBitMap: Bitmap for ESM error event of interest for this callback.
 *                       Array of uint32_t type with each bit representing one
 *                       ESM error event.
 * \param   eccCallback: Pointer to the callback to be called by the ESM Handler
 *                       to notify the ECC module of an ESM error event
 * \param   callbackArg: Argument that will be passed along with the callback.
 *
 * \return  SDL_PASS if success.
 *          SDL_EBADARGS if invalid argument is passed.
 *          SDL_EFAIL if other failure.
 */
int32_t SDL_ESM_registerECCCallback(SDL_ESM_Inst instance, uint32_t eventBitMap[],
                                    SDL_ESM_applicationCallback eccCallback,
                                    void *callbackArg);

/** ============================================================================
 *
 * \brief   SDL ESM API to initialize an ESM instance. The API initializes the
 *          specified ESM instance with the provided configuration. The
 *          configuration will allow the application to specify for each event
 *          whether the interrupt is enabled or disabled, the priority of the
 *          event, and whether the nErrorPin assertion is enabled or disabled for
 *          the event.
 *
 * \param   instance: ESM Instance
 * \param   pConfig: Pointer to the ESM configuration structure
 * \param   applicationCallback: Pointer to the callback to be called by the ESM
 *          Handler to notify the application of an ESM error event.
 * \param   appArg: Application argument that will passed to the application when
 *                  the application callback is called.
 *
 * \return  SDL_PASS if success.
 *          SDL_EBADARGS if invalid argument is passed.
 *          SDL_EFAIL if other failure.
 */
int32_t SDL_ESM_init(SDL_ESM_Inst instance, const SDL_ESM_config *pConfig,
                     SDL_ESM_applicationCallback applicationCallback, void *appArg);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* SDL_ESM_H_ */
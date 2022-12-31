/*
 * SDL EXCEPTION
 *
 * Software Diagnostics Library module for handling exceptions
 *
 *  Copyright (c) Texas Instruments Incorporated 2022
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

#ifndef INCLUDE_SDL_ECC_EXCEPTION_PRIV_H_
#define INCLUDE_SDL_ECC_EXCEPTION_PRIV_H_

#include <stdint.h>

/** ---------------------------------------------------------------------------
 * @brief This structure defines the elements of Exception software instance
 * ----------------------------------------------------------------------------
 */

typedef struct SDL_EXCEPTION_Instance_s
{
    SDL_EXCEPTION_CallbackFunctions_t callbackFunctions;
    /**< Exception callback functions structure stored here */
    SDL_EXCEPTION_ECCCallback_t ECCCallBackFunction;
    /**< Exception ECC callback function stored here */
    void *paramPtr;
    /**< Parameter pointer used for callback stored here */
}  SDL_EXCEPTION_Instance_t;

#endif /* INCLUDE_SDL_ECC_EXCEPTION_PRIV_H_ */

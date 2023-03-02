/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2023
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
* \defgroup SDL_MTOG_IP_API MTOG Low-Level API
* \ingroup SDL_MTOG_MODULE
* This module contains the Low-Level APIs to program and use the MTOG module.
* @{
*/
#ifndef SDL_IP_MTOG_H_
#define SDL_IP_MTOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "sdlr_mtog.h"
 
 /** 
 * \brief This enumerator defines the possible timeout values
 *
 *  \anchor SDL_MTOGVal
 *  \name Timeout values
 *
 *  @{
 * 
 */
typedef uint32_t SDL_MTOGVal;
    /** 1024 clock cycles */
#define SDL_MTOG_VAL_1K          ((uint32_t) 0U)
    /** 4096 clock cycles */
#define SDL_MTOG_VAL_4K          ((uint32_t) 1U)
    /** 16,384 clock cycles */
#define SDL_MTOG_VAL_16K         ((uint32_t) 2U)
    /** 65,536 clock cycles */
#define SDL_MTOG_VAL_64K         ((uint32_t) 3U)
    /** 262,144 clock cycles */
#define SDL_MTOG_VAL_256K        ((uint32_t) 4U)
    /** 1,048,576 clock cycles */
#define SDL_MTOG_VAL_1M          ((uint32_t) 5U)
    /** 2,097,152 clock cycles */
#define SDL_MTOG_VAL_2M          ((uint32_t) 6U)
    /** 4,194,303 clock cycles */
#define SDL_MTOG_VAL_4M_MINUS_1  ((uint32_t) 7U)
/** @} */

/** 
 *  \anchor SDL_MTOG_IP_FUNCTION
 *   @{
 * 
 */

/**
 *  \brief Set the timeout value
 *
 *  This function sets the desired timeout value. This function should only be
 *  called when the timeout is disabled.
 *
 *  \param pRegs        [IN]    Pointer to the desired VBUSM Master Timeout
 *                              Gasket Control register
 *  \param timeOut      [IN]    Timeout count value. See #SDL_MTOGVal for
 *                              a list of valid timeout values.
 *
 *  \return SDL_PASS      Function completed successfully
 *          SDL_EFAIL     Timeout value cannot be set when the timeout counter is
 *                        running. Call #SDL_MTOG_reset first.
 *          SDL_EBADARGS  pRegs is NULL
 */
int32_t SDL_MTOG_setTimeoutVal( SDL_MTOG_Regs *pRegs, SDL_MTOGVal timeOut );

/** @} */

#ifdef __cplusplus
}
#endif

#endif/* SDL_IP_MTOG_H_ */

/** @} */

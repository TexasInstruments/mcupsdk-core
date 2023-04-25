/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \defgroup SDL_LBIST_API APIs for SDL LBIST
 *  @{
*/

#ifndef SDL_LBIST_SOC_H_
#define SDL_LBIST_SOC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>
/**
 *  @defgroup SDL_LBIST_ENUM LBIST Enumerated Data Types
 *  @addtogroup SDL_LBIST_ENUM
    @{
 *
 */

/**
 *  \brief LBIST instance
 *
 *  This enum defines the LBIST instances supported by the SDL_LBIST_selfTest API.
 */
typedef enum {
    /*!
     * MCU Instance
     */
    LBIST_MCU_M4F,
} SDL_LBIST_inst;


#define SDL_LBIST_NUM_INSTANCES 1U
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* SDL_LBIST_SOC_H_ */
 /** @} */


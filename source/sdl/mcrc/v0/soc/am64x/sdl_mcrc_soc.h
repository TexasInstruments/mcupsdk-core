/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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
 *  @addtogroup SDL_MCRC_API MCRC API
    @{
 */

#ifndef SDL_MCRC_SOC_H_
#define SDL_MCRC_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>

#ifdef _cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 * \brief  MCRC Instance supported.
 */

typedef enum {
   MCU_MCRC64_0 = 1,
    /**< MCU_NAVSS0_MCRC_0 Instance */
   SDL_MCRC_INVALID = 0xffff,
    /**< Invalid instance  */
} SDL_MCRC_InstType;

/**
 * \brief   This API is used to get the base address of the instance.
 *
 * \param   instance          MCRC instance either MCU or Main.
 * \param   baseAddr          Dbase address of the instance.
 *
 * \return  status            return the base address of th instance.
 *                            SDL_PASS:     success
 *                            SDL_EBADARGS: failure, indicate the bad input arguments
 *                            SDL_EFAIL:    failure, indicate verify failed
 */
int32_t SDL_MCRC_getBaseaddr(SDL_MCRC_InstType instance, uint32_t *baseAddr);


#ifdef _cplusplus
}

#endif /*extern "C" */

#endif
 /** @} */

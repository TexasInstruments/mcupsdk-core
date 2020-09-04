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
 *  @defgroup SDL_DCC_MODULE APIs for SDL Dual Clock Comparator(DCC)
 *  @ingroup SDL_MODULE
 *  @{
 */

#ifndef SDL_SOC_DCC_H_
#define SDL_SOC_DCC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 * \brief MAX number of supported DCC Instances
 */

#define SDL_DCC_MAX_INSTANCE    (7U)

/**
 * \brief Enum for different DCC module instances supported
 */

typedef enum {

   SDL_DCC_INST_DCC0 ,
   /**< DCC0 Instance */
   SDL_DCC_INST_DCC1 ,
   /**< DCC1 Instance */
   SDL_DCC_INST_DCC2 ,
   /**< DCC2 Instance */
   SDL_DCC_INST_DCC3 ,
   /**< DCC3 Instance */
   SDL_DCC_INST_DCC4 ,
   /**< DCC4 Instance */
   SDL_DCC_INST_DCC5 ,
   /**< DCC5 Instance */

   SDL_DCC_INST_MCU_DCC0 ,
   /**< MCU DCC0 Instance */

   SDL_DCC_INVALID_INSTANCE,
   /**< Invalid Instance */

} SDL_DCC_Inst;

/** @} */

/**
 * \brief   This API is used to get the base address of the instance.
 *
 * \param   instance          DCC instance
 * \param   baseAddr          Base address of the instance.
 *
 * \return  status            return the base address of the instance.
 *                            SDL_PASS:     success
 *                            SDL_EBADARGS: failure, indicate translation failed
 */
int32_t SDL_DCC_getBaseaddr(SDL_DCC_Inst instance,
                            uint32_t *baseAddr);

/*======================================================================================================
*   This is global array gives the BASE ADDRESS of DCC modules
======================================================================================================*/

static uint32_t SDL_DCC_baseAddress[SDL_DCC_MAX_INSTANCE] = {SDL_DCC0_BASE, SDL_DCC1_BASE, SDL_DCC2_BASE, SDL_DCC3_BASE, SDL_DCC4_BASE, \
                                                             SDL_DCC5_BASE, \
                                                             SDL_MCU_DCC0_BASE};


#ifdef __cplusplus
}
#endif

#endif /* SDL_SOC_DCC_H_ */

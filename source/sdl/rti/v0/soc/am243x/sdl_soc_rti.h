/* =============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2023
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

#ifndef SDL_SOC_RTI_H_
#define SDL_SOC_RTI_H_

#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *
 * @ingroup  SDL_MODULE
 * @defgroup SDL_RTI_API APIs for SDL RTI
 *
 */
/**
@defgroup SDL_RTI_ENUM RTI Enumerated Data Types
@ingroup SDL_RTI_API
*/

/*======================================================================================================
*   Required Macros for Instances config
======================================================================================================*/

#define SDL_RTI_MAX_INSTANCE    (0x07U)
#define INSTANCE_INVALID        (0x0U)

/**
 *  @addtogroup SDL_RTI_ENUM
    @{
 *
 */

/*************************************************************************************************************
*  Enum for different RTI module instances
**************************************************************************************************************/

typedef enum {

   SDL_INSTANCE_MCU_RTI0_CFG,

   SDL_INSTANCE_RTI0_CFG,
   SDL_INSTANCE_RTI1_CFG,

   SDL_INSTANCE_RTI8_CFG,
   SDL_INSTANCE_RTI9_CFG,

   SDL_INSTANCE_RTI10_CFG,
   SDL_INSTANCE_RTI11_CFG,
   SDL_INSTANCE_INVALID,

} SDL_RTI_InstanceType;


/*======================================================================================================
*   This is global array gives the BASE ADDRESS of RTI modules
======================================================================================================*/

static uint32_t SDL_RTI_baseAddress[SDL_RTI_MAX_INSTANCE+1U] = {SDL_MCU_RTI0_CFG_BASE, SDL_RTI0_CFG_BASE, \
                                                                SDL_RTI1_CFG_BASE, SDL_RTI8_CFG_BASE, SDL_RTI9_CFG_BASE,       \
                                                                SDL_RTI10_CFG_BASE, SDL_RTI11_CFG_BASE,     \
                                                                 INSTANCE_INVALID};

int32_t SDL_RTI_getBaseaddr(SDL_RTI_InstanceType instance,
                             uint32_t *baseAddr);

#ifdef __cplusplus
}
#endif

#endif /* SDL_SOC_RTI_H_ */
/** @} */
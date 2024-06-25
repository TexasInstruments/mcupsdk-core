/* 
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

#include <sdl/include/am273x/sdlr_soc_baseaddress.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*   Required Macros for Instances config
*/

#define SDL_RTI_MAX_INSTANCE    (0x07U)
#define INSTANCE_INVALID        (0x0U)

/**
 *  @addtogroup SDL_RTI_API APIs for SDL RTI
 *   @{
 */
/**
* \defgroup SDL_RTI_ENUM RTI Enumerated Data Types
* \ingroup SDL_RTI_API
* @{
*/

/*************************************************************************************************************
*  Enum for different RTI module instances
**************************************************************************************************************/

typedef enum {

   SDL_INSTANCE_MSS_WDT,
   SDL_INSTANCE_DSS_WDT,
   SDL_INSTANCE_MSS_RTIA,
   SDL_INSTANCE_MSS_RTIB,
   SDL_INSTANCE_MSS_RTIC,
   SDL_INSTANCE_DSS_RTIA,
   SDL_INSTANCE_DSS_RTIB,
   SDL_INSTANCE_INVALID,

} SDL_RTI_InstanceType;

/** @} */
/** @} */
/*======================================================================================================
*   This is global array gives the BASE ADDRESS of RTI modules
======================================================================================================*/

static uint32_t SDL_RTI_baseAddress[SDL_RTI_MAX_INSTANCE+1U] = {SDL_MSS_WDT_U_BASE,SDL_DSS_WDT_U_BASE,SDL_MSS_RTIA_U_BASE,SDL_MSS_RTIB_U_BASE, \
                                                                SDL_MSS_RTIC_U_BASE,SDL_DSS_RTIA_U_BASE, \
                                                                SDL_DSS_RTIB_U_BASE, \
                                                                SDL_INSTANCE_INVALID};
  int32_t SDL_RTI_getBaseaddr(SDL_RTI_InstanceType instance,
                               uint32_t *baseAddr);
#ifdef __cplusplus
}
#endif

#endif /* SDL_SOC_RTI_H_ */

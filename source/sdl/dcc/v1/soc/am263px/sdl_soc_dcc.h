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
#ifndef SDL_SOC_DCC_H_
#define SDL_SOC_DCC_H_

#include <sdl/include/am263px/sdlr_soc_baseaddress.h>



#ifdef __cplusplus
extern "C" {
#endif

/** ===========================================================================
 *
 * @ingroup  SDL_DCC_MODULE
 * @defgroup SDL_DCC_API SDL Dual Clock Comparator(DCC)
 *
 * ============================================================================
 */
/**
@defgroup SDL_DCC_ENUM DCC Enumerated Data Types
@ingroup SDL_DCC_API
*/

/*======================================================================================================
*   Required Macros for Instances config
======================================================================================================*/

#define SDL_DCC_MAX_INSTANCE    (4U)

/** ===========================================================================
 *  @addtogroup SDL_DCC_ENUM
    @{
 * ============================================================================
 */

/*************************************************************************************************************
*  Enum for different DCC module instances
**************************************************************************************************************/

typedef enum {

   SDL_DCC_INST_MSS_DCCA,
   /**< MSS DCCA Instance */
   SDL_DCC_INST_MSS_DCCB,
   /**< MSS DCCB Instance */
   SDL_DCC_INST_MSS_DCCC,
   /**< MSS DCCC Instance */
   SDL_DCC_INST_MSS_DCCD,
   /**< MSS DCCD Instance */
   SDL_DCC_INVALID_INSTANCE,
   /**< Invalid Instance */

} SDL_DCC_Inst;

/* @} */

/*======================================================================================================
*   This is global array gives the BASE ADDRESS of DCC modules
======================================================================================================*/

static unsigned long int  SDL_DCC_baseAddress[SDL_DCC_MAX_INSTANCE] = {SDL_DCC0_U_BASE, SDL_DCC1_U_BASE, SDL_DCC2_U_BASE, SDL_DCC3_U_BASE};


#ifdef __cplusplus
}
#endif

#endif /* SDL_SOC_DCC_H_ */
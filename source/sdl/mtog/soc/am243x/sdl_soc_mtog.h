/********************************************************************
*
* SOC MTOG PROPERTIES. header file
*
* Copyright (c) Texas Instruments Incorporated 2022-2023
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
 *  \defgroup SDL_MTOG_IP_API MTOG Low-Level API
 *  @{
*/ 
#ifndef SDL_VCL_SOC_MTOG_H_
#define SDL_VCL_SOC_MTOG_H_

#include<sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>
#include<sdl/include/am64x_am243x/sdlr_mcu_ctrl_mmr.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 * \brief  MTOG Instance supported.
 */
typedef uint8_t SDL_MTOG_Inst;
    /** MCU_MTOG0 */
#define SDL_INSTANCE_MCU_MTOG0                1U                
    /** MAX Instance MTOG */                 
#define SDL_INSTANCE_MTOG_MAX                 SDL_INSTANCE_MCU_MTOG0


/**
 * \brief   This API is used to get the base address of the instance.
 *
 * \param   instance          TOG instance
 * \param   baseAddr          Base address of the instance.
 *
 * \return  status            return the base address of the instance.
 *                            SDL_PASS:     success
 *                            SDL_EBADARGS: failure, indicate translation failed
 */
int32_t SDL_MTOG_getBaseaddr(SDL_MTOG_Inst instance,
                             uint32_t *baseAddr);


#ifdef __cplusplus
}
#endif

#endif /* SDL_SOC_MTOG_H_ */
 /** @} */
 
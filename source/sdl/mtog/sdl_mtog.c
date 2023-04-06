/**
 * @file  sdl_mtog.c
 *
 * @brief
 *  Implementation file for the VBUSM Master Timeout Gasket module SDL.
 *
 *  \par
 *  ============================================================================
 *  @n   Copyright (c) Texas Instruments Incorporated 2022-2023
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

#include <stdint.h>
#include "sdl/sdl_mtog.h"
#include <sdl/include/sdlr.h>
#include "sdl/include/sdl_types.h"
#include <sdl/mtog/v0/sdlr_mtog.h>
#include "sdl/include/sdl_types.h"
#include "v0/sdl_ip_mtog.h"
#if defined (SOC_AM64X)
#include <sdl/mtog/soc/sdl_soc_mtog.h>
#endif


 /**
 * Design: PROC_SDL-3273
 */
int32_t SDL_MTOG_init(SDL_MTOG_Inst instance, const SDL_MTOG_config *pConfig)
{
    int32_t sdlResult = SDL_EBADARGS;
    int32_t  retVal;
	uint32_t mtog_base_addr=0x0u;
    SDL_MTOG_getBaseaddr(SDL_INSTANCE_MCU_MTOG0, &mtog_base_addr);
    if((instance <= SDL_INSTANCE_MTOG_MAX) && (pConfig != NULL_PTR))
    {
        retVal = SDL_MTOG_setTimeoutVal((SDL_MTOG_Regs *) mtog_base_addr, pConfig->timeOut );
        if(retVal == SDL_PASS)
        {
            sdlResult = SDL_PASS;
        }
        else
        {
            sdlResult = SDL_EFAIL;
        }
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-3274
 */
int32_t SDL_MTOG_verifyConfig(SDL_MTOG_Inst instance, const SDL_MTOG_config *pConfig)
{
    SDL_MTOG_config cfg;
    int32_t sdlResult = SDL_EBADARGS;
	uint32_t mtog_base_addr=0x0u;
    SDL_MTOG_getBaseaddr(SDL_INSTANCE_MCU_MTOG0, &mtog_base_addr);
    if( (instance <= SDL_INSTANCE_MTOG_MAX) && (pConfig != NULL_PTR))
    {
        cfg.timeOut = SDL_REG32_RD( mtog_base_addr);
        cfg.timeOut = SDL_FEXT(cfg.timeOut, MTOG_VAL);
        if(pConfig->timeOut == cfg.timeOut)
        {
            sdlResult = SDL_PASS;
        }
        else
        {
            sdlResult = SDL_EFAIL;
        }
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-3275
 */
int32_t SDL_MTOG_start( SDL_MTOG_Inst instance )
{
    int32_t  sdlResult = SDL_EBADARGS;
	uint32_t mtog_base_addr=0x0u;
    SDL_MTOG_getBaseaddr(SDL_INSTANCE_MCU_MTOG0, &mtog_base_addr);
    if (instance <= SDL_INSTANCE_MTOG_MAX)
    {
        SDL_REG32_FINS( mtog_base_addr, MTOG_EN, (uint32_t)1U );
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-3276
 */
int32_t SDL_MTOG_stop( SDL_MTOG_Inst instance )
{
    int32_t  sdlResult = SDL_EBADARGS;
	uint32_t mtog_base_addr=0x0u;
    SDL_MTOG_getBaseaddr(SDL_INSTANCE_MCU_MTOG0, &mtog_base_addr);
    if (instance <= SDL_INSTANCE_MTOG_MAX)
    {
        SDL_REG32_FINS( mtog_base_addr, MTOG_EN, (uint32_t)0U );
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-3277
 */
int32_t SDL_MTOG_forceTimeout( SDL_MTOG_Inst instance )
{
    int32_t sdlResult = SDL_EBADARGS;
	uint32_t mtog_base_addr=0x0u;
    SDL_MTOG_getBaseaddr(SDL_INSTANCE_MCU_MTOG0, &mtog_base_addr);
    if (instance <= SDL_INSTANCE_MTOG_MAX)
    {
        /* Forcing a timeout is only applicable when enable==1 */
        if( SDL_REG32_FEXT(mtog_base_addr, MTOG_EN) == 1U )
        {
            SDL_REG32_FINS(mtog_base_addr, MTOG_FORCE, SDL_MTOG_FORCE_KEY);
            sdlResult = SDL_PASS;
        }
        else
        {
            sdlResult = SDL_EFAIL;
        }
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-3278
 */
int32_t SDL_MTOG_reset( SDL_MTOG_Inst instance )
{
    uint32_t regVal;
    int32_t  sdlResult = SDL_EBADARGS;
	uint32_t mtog_base_addr=0x0u;
    if (instance <= SDL_INSTANCE_MTOG_MAX)
    {
        regVal =SDL_MTOG_getBaseaddr(SDL_INSTANCE_MCU_MTOG0, &mtog_base_addr);
        SDL_FINS( regVal, MTOG_FORCE, (uint32_t)0U );
        SDL_FINS( regVal, MTOG_EN,    (uint32_t)0U );
        SDL_REG32_WR(mtog_base_addr, regVal);
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-3279
 */
int32_t SDL_MTOG_getStaticRegisters( SDL_MTOG_Inst instance,  SDL_MTOG_staticRegs * pStaticRegs)
{
    int32_t sdlResult = SDL_EBADARGS;
	uint32_t mtog_base_addr=0x0u;
    SDL_MTOG_getBaseaddr(SDL_INSTANCE_MCU_MTOG0, &mtog_base_addr);
    if( (instance <= SDL_INSTANCE_MTOG_MAX) && (pStaticRegs != NULL_PTR) )
    {
		pStaticRegs->mtogCtrlRegister = (uint32_t)SDL_REG32_RD(mtog_base_addr);
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

/* nothing past this point */
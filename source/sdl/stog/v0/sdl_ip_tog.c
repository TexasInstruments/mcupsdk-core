/**
 * @file  sdl_ip_tog.c
 *
 * @brief
 *  Implementation file for the VBUSM Timeout Gasket module SDL.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2021-2023, Texas Instruments, Inc.
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
#include "sdlr_tog.h"
#include <sdl/include/sdl_types.h>

 /**
 * Design: PROC_SDL-1171
 */
int32_t SDL_TOG_getIntrCountInternal( uint32_t baseAddr, SDL_TOG_IntrSrc intrSrc, uint32_t *pIntrCnt )
{
   int32_t sdlResult = SDL_EBADARGS;

   if((intrSrc > 0U) && (intrSrc <= SDL_TOG_INTRSRC_ALL) && (pIntrCnt != NULL))
        {
            if( intrSrc == SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT )
            {
                *pIntrCnt = SDL_REG32_RD( baseAddr + SDL_TOG_ERR_TM_INFO );
                sdlResult = SDL_PASS;
            }
            else if( intrSrc == SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE )
            {
                *pIntrCnt = SDL_REG32_RD( baseAddr + SDL_TOG_ERR_UN_INFO );
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
 * Design: PROC_SDL-1502
 */
int32_t SDL_TOG_setTimeoutVal(uint32_t baseAddr, uint32_t timeoutVal )
{
    int32_t sdlResult = SDL_EBADARGS;

    if(timeoutVal <= SDL_TOG_TIMEOUT_TO_MAX)
    {
        SDL_REG32_WR( baseAddr + SDL_TOG_TIMEOUT, SDL_FMK( TOG_TIMEOUT_TO, timeoutVal ) );
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

/**
 * Design: PROC_SDL-1503
 */
int32_t SDL_TOG_setIntrPending(uint32_t baseAddr, SDL_TOG_IntrSrc intrSrc )
{
    int32_t sdlResult = SDL_EBADARGS;

    if((intrSrc > 0U) && (intrSrc <= SDL_TOG_INTRSRC_ALL) )
    {
        SDL_REG32_WR(baseAddr + SDL_TOG_ERR_RAW, intrSrc );
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}


/* nothing past this point */


/**
 * @file  sdl_ip_mtog.c
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
#include <sdl/include/sdl_types.h>
#include <sdl/include/sdlr.h>
#include "sdl/mtog/v0/sdlr_mtog.h"
#include "sdl/mtog/v0/sdl_ip_mtog.h"

 /**
 * Design: PROC_SDL-3280
 */
int32_t SDL_MTOG_setTimeoutVal( SDL_MTOG_Regs *pRegs, SDL_MTOGVal timeOut )
{
    int32_t retVal = SDL_PASS;
    uint32_t regVal;

    if ((pRegs == NULL_PTR)|| (timeOut>SDL_MTOG_VAL_4M_MINUS_1))
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        regVal = pRegs->CONTROL;
        /* Timeout value can only be changed when enable==0 */
        if( SDL_FEXT( regVal, MTOG_EN ) == 0U )
        {
            SDL_FINS( regVal, MTOG_VAL, timeOut );
            pRegs->CONTROL = regVal;
        }
        else
        {
            retVal = SDL_EFAIL;
        }
    }
    return retVal;
}

/* nothing past this point */

/**
 * @file  sdl_soc_vtm.c
 *
 * @brief
 *  C implementation file for the VTM module SDL-FL.
 *
 *  Contains the different control command and status query functions definitions
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2023, Texas Instruments, Inc.
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

#include <stdbool.h>
#include <stddef.h>
#include <sdl/vtm/v0/sdlr_vtm.h>
#include "sdl_soc_vtm.h"
#include <sdl/dpl/sdl_dpl.h>

bool SDL_VTM_getBaseAddr(SDL_VTM_cfgReg cfgReg, uint32_t *vtmBaseAddr)
{
    bool instValid = ((bool)false);
    uint32_t size = 0u;

    if (vtmBaseAddr != NULL)
    {
        switch(cfgReg)
        {
            case SDL_VTM_CONFIG_REG_1:
                instValid = ((bool)true);
                *vtmBaseAddr = SDL_VTM0_MMR_VBUSP_CFG1_BASE;
                size = SDL_VTM0_MMR_VBUSP_CFG1_SIZE;
                break;

            case SDL_VTM_CONFIG_REG_2:
                instValid = ((bool)true);
                *vtmBaseAddr = SDL_VTM0_MMR_VBUSP_CFG2_BASE;
                size = SDL_VTM0_MMR_VBUSP_CFG2_SIZE;
                break;

            default:
                break;
        }
	    *vtmBaseAddr = (uint32_t)SDL_DPL_addrTranslate(*vtmBaseAddr, size);
    }
    return (instValid);
}
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

/**
 *  \file     sdl_soc_pok.c
 *
 *  \brief    This file contains the soc-specific implementation of the API's present in the
 *            device abstraction layer file of rti.
 */

#include <stdint.h>
#include <stdbool.h>
#include "sdl_soc_pok.h"
#include <sdl/pok/v1/sdl_ip_pok.h>
#include <sdl/pok/v1/sdl_pok.h>
#include <sdl/dpl/sdl_dpl.h>

#ifdef __cplusplus
extern "C" {
}
#endif




/**
 *  Design:
 */
bool SDL_POK_getBaseaddr(SDL_POK_InstanceType instance, uint32_t *pBaseAddress)
{
    bool instValid = ((bool)false);
    uint32_t size = 0;

   if (pBaseAddress != NULL)
    {
        switch(instance)
        {
            case SDL_POK_MCU_CTRL_MMR0:
                instValid = ((bool)true);
                *pBaseAddress = SDL_MCU_CTRL_MMR0_CFG0_BASE;
                size = SDL_MCU_CTRL_MMR0_CFG0_SIZE;
                break;


            default:
                break;
        }
	    *pBaseAddress = (uint32_t)SDL_DPL_addrTranslate(*pBaseAddress, size);
    }
       
    return (instValid);
}
   

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
 *  \file     sdl_hwa_soc.c
 *
 *  \brief    This file contains the soc-specific implementation of the API's present in the
 *            device abstraction layer file of HWA.
 */
#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include <sdl/hwa/v0/sdl_ip_hwa.h>
#include <sdl/hwa/v0/sdl_hwa_hw.h>
#include <sdl/hwa/v0/soc/sdl_hwa_soc.h>

int32_t SDL_HWA_getMemblockBaseaddr(SDL_HWA_MemID memID, SDL_HWA_MemBlock memBlock,\
                             uint32_t *baseAddr)
{
    int32_t status = SDL_PASS;
    //uint32_t size = 0;

    if (baseAddr == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        if (memID == SDL_HWA_DMA0_MEM_ID )
        {
            switch (memBlock)
            {
                case SDL_HWA_DMEM0:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK0_BASE;
                break;
                case SDL_HWA_DMEM1:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK1_BASE;
                break;
                case SDL_HWA_DMEM2:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK2_BASE;
                break;
                case SDL_HWA_DMEM3:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK3_BASE;
                break;
                case SDL_HWA_DMEM4:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK4_BASE;
                break;
                case SDL_HWA_DMEM5:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK5_BASE;
                break;
                case SDL_HWA_DMEM6:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK6_BASE;
                break;
                case SDL_HWA_DMEM7:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK7_BASE;
                break;
                default:
                status = SDL_EBADARGS;
                break;
            }
        }
        else if(memID == SDL_HWA_DMA1_MEM_ID )
        {
            switch (memBlock)
            {
                case SDL_HWA_DMEM0:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK0_BASE;
                break;
                case SDL_HWA_DMEM1:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK1_BASE;
                break;
                case SDL_HWA_DMEM2:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK2_BASE;
                break;
                case SDL_HWA_DMEM3:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK3_BASE;
                break;
                case SDL_HWA_DMEM4:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK4_BASE;
                break;
                case SDL_HWA_DMEM5:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK5_BASE;
                break;
                case SDL_HWA_DMEM6:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK6_BASE;
                break;
                case SDL_HWA_DMEM7:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK7_BASE;
                break;
                default:
                status = SDL_EBADARGS;
                break;
            }
        }
        else if( memID == SDL_HWA_WINDOW_RAM_MEM_ID)
        {
            *baseAddr = (uint32_t)SDL_DSS_HWA_WINDOW_RAM_U_BASE;

        }
        else
        {
            status = SDL_EBADARGS;
        }
    }
    return (status);
}


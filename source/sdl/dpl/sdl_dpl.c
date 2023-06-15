/**
 * @file  sdl_dpl.c
 *
 * @brief
 *  SDL implementation file for the dpl module.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2022, Texas Instruments, Inc.
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
#include "sdl_dpl.h"
#include <sdl/include/sdl_types.h>

/**
 * Design: PROC_SDL-5805
 */
SDL_DPL_Interface *gSDL_DPL_Interface = (SDL_DPL_Interface *)NULL_PTR;

int32_t SDL_DPL_init(SDL_DPL_Interface *dplInterface)
{
    SDL_ErrType_t ret = SDL_PASS;

    if (dplInterface == NULL_PTR)
    {
        ret = SDL_EINVALID_PARAMS;
    }
    else
    {
        gSDL_DPL_Interface = dplInterface;
    }

    return ret;
}

/**
 * Design: PROC_SDL-5803
 */
int32_t SDL_DPL_enableInterrupt(int32_t intNum)
{
    SDL_ErrType_t ret = SDL_PASS;

    /* Check HwiP function is valid */
    if ((gSDL_DPL_Interface == NULL_PTR) || (gSDL_DPL_Interface->enableInterrupt == NULL_PTR))
    {
        ret = SDL_EINVALID_PARAMS;
    }
    else
    {
        ret = gSDL_DPL_Interface->enableInterrupt(intNum);
    }

    return ret;
}

/**
 * Design: PROC_SDL-6203
 */
int32_t SDL_DPL_disableInterrupt(int32_t intNum)
{
    SDL_ErrType_t ret = SDL_PASS;

    /* Check HwiP function is valid */
    if ((gSDL_DPL_Interface == NULL_PTR) || (gSDL_DPL_Interface->disableInterrupt == NULL_PTR))
    {
        ret = SDL_EINVALID_PARAMS;
    }
    else
    {
        ret = gSDL_DPL_Interface->disableInterrupt(intNum);
    }

    return ret;
}

/**
 * Design: PROC_SDL-5802
 */

int32_t SDL_DPL_registerInterrupt(SDL_DPL_HwipParams *pParams, pSDL_DPL_HwipHandle *handle)
{
    SDL_ErrType_t ret = SDL_PASS;

    /* Check HwiP function is valid */
    if ((handle == NULL_PTR) || (pParams == NULL_PTR))
    {
        ret = SDL_EINVALID_PARAMS;
    }
    else if ((gSDL_DPL_Interface == NULL_PTR) || (gSDL_DPL_Interface->registerInterrupt == NULL_PTR))
    {
        ret = SDL_EINVALID_PARAMS;
    }
    else
    {
        *handle = gSDL_DPL_Interface->registerInterrupt(pParams);
    }

    return ret;
}

/**
 * Design: PROC_SDL-6206
 */
int32_t SDL_DPL_deregisterInterrupt(pSDL_DPL_HwipHandle handle)
{
    SDL_ErrType_t ret = SDL_PASS;

    /* Check HwiP function is valid */
    if (handle == NULL_PTR)
    {
        ret = SDL_EINVALID_PARAMS;
    }
    else if ((gSDL_DPL_Interface == NULL_PTR) || (gSDL_DPL_Interface->deregisterInterrupt == NULL_PTR))
    {
        ret = SDL_EINVALID_PARAMS;
    }
    else
    {
        ret = gSDL_DPL_Interface->deregisterInterrupt(handle);
    }

    return ret;
}

int32_t SDL_DPL_delay(int32_t ndelay)
{
    SDL_ErrType_t ret = SDL_PASS;

    /* Check if delay function is valid */
    if ((gSDL_DPL_Interface == NULL_PTR) || (gSDL_DPL_Interface->delay == NULL_PTR))
    {
        ret = SDL_EINVALID_PARAMS;
    }
    else
    {
        ret = gSDL_DPL_Interface->delay(ndelay);
    }

    return ret;
}

void* SDL_DPL_addrTranslate(uint64_t addr, uint32_t size)
{
    void *ret = (void *)(-1);

    if ((gSDL_DPL_Interface != NULL_PTR) && (gSDL_DPL_Interface->addrTranslate != NULL_PTR))
    {
        ret = gSDL_DPL_Interface->addrTranslate(addr, size);
    }

    return ret;
}

/**
 * Design: PROC_SDL-6205
 */
int32_t SDL_DPL_globalDisableInterrupts(uintptr_t *key)
{
    SDL_ErrType_t ret = SDL_PASS;

    if (key == NULL_PTR)
    {
        ret = SDL_EINVALID_PARAMS;
    }
    // Check HwiP function is valid
    else if ((gSDL_DPL_Interface == NULL_PTR) || (gSDL_DPL_Interface->globalDisableInterrupts == NULL_PTR))
    {
        ret = SDL_EINVALID_PARAMS;
    }
    else
    {
        ret = gSDL_DPL_Interface->globalDisableInterrupts(key);
    }

    return ret;
}

/**
 * Design: PROC_SDL-6204
 */
int32_t SDL_DPL_globalRestoreInterrupts(uintptr_t key)
{
    SDL_ErrType_t ret = SDL_PASS;

    // Check HwiP function is valid
    if ((gSDL_DPL_Interface == NULL_PTR) || (gSDL_DPL_Interface->globalRestoreInterrupts == NULL_PTR))
    {
        ret = SDL_EINVALID_PARAMS;
    }
    else
    {
        ret = gSDL_DPL_Interface->globalRestoreInterrupts(key);
    }

    return ret;
}

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

#include "sdl_ip_rti.h"

/** \brief This is to disable HW_SYNC_BARRIER for register access */
#define MEM_BARRIER_DISABLE

/************************************************************************************************
*   Below are the dependant functions for RTI init, config,some functions
************************************************************************************************/

/**
 *  Design: PROC_SDL-1501
 */

static inline uint32_t SDL_RTI_readWinSz(uint32_t baseAddr)
{
    uint32_t   windowSize;
    /* Get configured Window Size from RTI_WWDSIZECTRL Register (Offset = A8h) */
    windowSize = HW_RD_REG32(baseAddr + RTI_RTIDWWDSIZECTRL);
    return (windowSize);
}

/**
 *  Design: PROC_SDL-1488,PROC_SDL-1489
 */

int32_t SDL_RTI_getWindowSize(uint32_t baseAddr, uint32_t *pWinSize)
{
    int32_t sdlResult;

    if ((baseAddr           != ((uint32_t)NULL)) &&
        (pWinSize           != NULL ))
    {
        *pWinSize = SDL_RTI_readWinSz(baseAddr);
        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EFAIL;
    }
    return (sdlResult);
}

/**
 *  Design: PROC_SDL-1491,PROC_SDL-1490
 */

int32_t SDL_RTI_chkWindowSize(uint32_t dwwdWindowSize)
{
    int32_t sdlResult;
    switch (dwwdWindowSize)
    {
        case RTI_DWWD_WINDOWSIZE_100_PERCENT:
        case RTI_DWWD_WINDOWSIZE_50_PERCENT:
        case RTI_DWWD_WINDOWSIZE_25_PERCENT:
        case RTI_DWWD_WINDOWSIZE_12_5_PERCENT:
        case RTI_DWWD_WINDOWSIZE_6_25_PERCENT:
        case RTI_DWWD_WINDOWSIZE_3_125_PERCENT:
            sdlResult = SDL_PASS;
            break;
        default:
            sdlResult = SDL_EFAIL;
            break;
    }
    return (sdlResult);
}

/**
 *  Design: PROC_SDL-1494
 */

int32_t SDL_RTI_chkReaction(uint32_t dwwdReaction)
{
    int32_t sdlResult = SDL_PASS;

    if (dwwdReaction == RTI_DWWD_REACTION_GENERATE_NMI)
    {
        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EFAIL;
    }
    return (sdlResult);
}

/**
 *  Design: PROC_SDL-1495
 */

int32_t SDL_RTI_setPreload(uint32_t baseAddr, uint32_t dwwdPreloadVal)
{
    int32_t sdlResult = SDL_EFAIL;
    uint32_t dwwdPreloadVal_l;

    if (baseAddr  != ((uint32_t) NULL))
    {
        dwwdPreloadVal_l = (dwwdPreloadVal >>
                          ((uint32_t) RTI_DWWDPRLD_MULTIPLIER_SHIFT));

            /* Initialize DWD Expiration Period */
            HW_WR_FIELD32(baseAddr + RTI_RTIDWDPRLD,
                          RTI_RTIDWDPRLD_DWDPRLD,
                          dwwdPreloadVal_l);
            sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EFAIL;
    }
    return sdlResult;
}

/**
 *  Design: PROC_SDL-1496
 */

int32_t SDL_RTI_getPreload(uint32_t baseAddr, uint32_t *pPreloadVal)
{
    int32_t sdlResult;
    if ((baseAddr           != ((uint32_t)NULL)) &&
        (pPreloadVal        != NULL ))
    {
        *pPreloadVal = HW_RD_FIELD32(baseAddr + RTI_RTIDWDPRLD,
                                   RTI_RTIDWDPRLD_DWDPRLD);

        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EFAIL;
    }
    return (sdlResult);
}

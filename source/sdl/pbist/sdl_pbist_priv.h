/* Copyright (C) 2022 Texas Instruments Incorporated.
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
 *   @file  sdl_pbist_priv.h
 *
 *   @brief This file contains the SDL PBIST internal definitions.
 *
 *   @defgroup SDL_PBIST_MODULE APIs for PBIST (Memory Built-In Self Test)
 *
 *   Provides the internal definitions for PBIST.
 */

#ifndef SDL_PBIST_PRIV_H_
#define SDL_PBIST_PRIV_H_

#include <sdl/sdl_pbist.h>

#ifdef __cplusplus
extern "C" {
#endif


#define PBIST_NOT_DONE        (0U)
#define PBIST_DONE            (1U)

#define SDL_PBIST_INTERRUPT_INVALID  (0xFFFFFFFFU)


SDL_pbistInstInfo * SDL_PBIST_getInstInfo(SDL_PBIST_inst instance);

void SDL_PBIST_eventHandler(uint32_t coreIndex);

void SDL_PBIST_checkDone(SDL_pbistInstInfo *pInfo);

#ifdef __cplusplus
}
#endif

#endif /* SDL_PBIST_PRIV_H_ */

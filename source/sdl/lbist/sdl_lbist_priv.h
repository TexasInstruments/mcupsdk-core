/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *
 *   \defgroup SDL_LBIST_API APIs for SDL LBIST
 *   \brief  This file contains the SDL LBIST internal definitions.
 *   Provides the internal definitions for LBIST.
 * @{
 */

#ifndef SDL_LBIST_PRIV_H_
#define SDL_LBIST_PRIV_H_

#include <sdl/dpl/sdl_dpl.h>
#if defined (IP_VERSION_LBIST_V0)
#include <sdl/lbist/V0/sdl_ip_lbist.h>
#endif
#ifdef __cplusplus
extern "C" {
#endif

#define LBIST_NOT_DONE (0x0U)
#define LBIST_DONE     (0x1U)

typedef struct
{
    /** Pointer to LBIST register map */
    SDL_lbistRegs *pLBISTRegs;
    /** Pointer to LBIST signature */
    uint32_t *pLBISTSig;
    /** Expected signature */
    uint32_t expectedMISR;
    /** Interrupt number for error event */
    uint32_t interruptNumber;
    /** Flag to indicate test done, will be set when interrupt event comes */
    volatile uint32_t doneFlag;
    /** Configuration for LBIST test */
    SDL_LBIST_config_t LBISTConfig;
} SDL_lbistInstInfo;


SDL_lbistInstInfo * SDL_LBIST_getInstInfo(uint32_t instance);

void SDL_LBIST_eventHandler(void *arg);


#ifdef __cplusplus
}
#endif

#endif /* SDL_LBIST_PRIV_H_ */

/** @} */

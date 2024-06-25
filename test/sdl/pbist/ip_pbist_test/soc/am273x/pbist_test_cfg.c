/*
 *   Copyright (c) Texas Instruments Incorporated 2020-2022
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
 *  \file     pbist_test_cfg.c
 *
 *  \brief    This file contains PBIST test configuration
 *
 *  \details  PBIST Test Configuration
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <sdl/pbist/sdl_pbist_priv.h>
#include <sdl/sdl_pbist.h>
#include "pbist_test_cfg.h"

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */


/* ========================================================================== */
/*                            Local function prototypes                       */
/* ========================================================================== */


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if defined (R5F0_INPUTS)
PBIST_TestHandle_t PBIST_TestHandleArray[2] =
{
    {
        .testName               = "TOP PBIST",
        .pbistInst              = SDL_PBIST_INST_TOP,
        .pPBISTRegs             = (SDL_pbistRegs *)SDL_TOP_PBIST_U_BASE,
        .numPBISTRuns           = 1u,
        .interruptNumber        = SDL_MSS_INTR_TOP_PBIST_DONE_INT,
        .doneFlag               = false,                /* Initialize done flag  */
    },

    {
        .testName               = "DSP PBIST",
        .pbistInst              = SDL_PBIST_INST_DSS,
        .pPBISTRegs             = (SDL_pbistRegs *)SDL_DSS_DSP_PBIST_U_BASE,
        .numPBISTRuns           = 1u,
        .interruptNumber        = SDL_DSS_INTR_DSS_DSP_PBIST_CTRL_DONE,
        .doneFlag               = false,                /* Initialize done flag  */
    },
};
#endif
#if defined (R5F1_INPUTS)
PBIST_TestHandle_t PBIST_TestHandleArray[1] =
{
    {
        .testName               = "TOP PBIST",
        .pbistInst              = SDL_PBIST_INST_TOP,
        .pPBISTRegs             = (SDL_pbistRegs *)SDL_TOP_PBIST_U_BASE,
        .numPBISTRuns           = 1u,
        .interruptNumber        = SDL_MSS_INTR_TOP_PBIST_DONE_INT,
        .doneFlag               = false,                /* Initialize done flag  */
    },
};
#endif

/* Captures common Initialization: currently nothing needed */
int32_t PBIST_commonInit(void)
{
    SDL_ErrType_t status = SDL_PASS;

    return status;
}

void PBIST_eventHandler( uint32_t instanceId)
{
    PBIST_TestHandleArray[instanceId].doneFlag = true;

    return;
}

/* Nothing past this point */

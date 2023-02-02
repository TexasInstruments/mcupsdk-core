/**
 * @file  sdl_ip_pbist.c
 *
 * @brief
 *  SDL-FL implementation file for the pbist module.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2022-2023, Texas Instruments, Inc.
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
#include <sdl/include/soc_config.h>
#include <sdl/pbist/v0/sdlr_pbist.h>
#include "sdl_ip_pbist.h"
#include <sdl/include/sdl_types.h>

/* PBIST test mode */
#define SDL_PBIST_TEST_MODE (SDL_PBIST_MARGIN_MODE_PBIST_DFT_WRITE_MASK \
                             | (1u << SDL_PBIST_MARGIN_MODE_PBIST_DFT_READ_SHIFT))

/* PBIST Failure Insertion test mode */
#define SDL_PBIST_FAILURE_INSERTION_TEST_MODE (SDL_PBIST_MARGIN_MODE_PBIST_DFT_WRITE_MASK \
                                               | SDL_PBIST_MARGIN_MODE_PBIST_DFT_READ_MASK)

/* PBIST Functional mode  */
#define SDL_PBIST_FUNCTIONAL_MODE       (0x0U)

/**
 * Design: PROC_SDL-959,PROC_SDL-960
 */
int32_t SDL_PBIST_softReset(SDL_pbistRegs *pPBISTRegs)
{
    int32_t sdlResult= SDL_PASS;

    if (pPBISTRegs == NULL_PTR)
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        /* Turns on PBIST clock */
        pPBISTRegs->PACT = SDL_PBIST_PACT_PACT_MASK;

        /* Zero out Loop counter 0 */
        pPBISTRegs->L0 = ((uint32_t)0x0u);

        /* Zero out Pbist Id register */
        pPBISTRegs->PID = ((uint32_t)0x0u);

        /* Set override register to all 1 */
        pPBISTRegs->OVER = (SDL_PBIST_OVER_RINFO_MASK
                           | SDL_PBIST_OVER_READ_MASK
                           | SDL_PBIST_OVER_MM_MASK
                           | SDL_PBIST_OVER_ALGO_MASK);

        /* Zero out Data logger 0 */
        pPBISTRegs->DLR = ((uint32_t)0x0u);

        /* Zero out Clock Mux Select register */
        pPBISTRegs->CMS = ((uint32_t)0x0u);
    }
    return  sdlResult;
}

/**
 * Design: PROC_SDL-961,PROC_SDL-967,PROC_SDL-962
 */
int32_t SDL_PBIST_start(SDL_pbistRegs *pPBISTRegs,
                        const SDL_PBIST_config * const pConfig)
{
    int32_t sdlResult= SDL_PASS;

    if ((pPBISTRegs == NULL_PTR) || (pConfig == NULL_PTR))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        /* Turns on PBIST clock in PBIST ACTivate register */
        pPBISTRegs->PACT = SDL_PBIST_PACT_PACT_MASK;

        /* Set Margin mode register for Test mode */
        pPBISTRegs->MARGIN_MODE = SDL_PBIST_TEST_MODE;

        /* Zero out Loop counter 0 */
        pPBISTRegs->L0 = ((uint32_t)0x0u);

        /* Set algorithm bitmap */
        pPBISTRegs->ALGO = pConfig->algorithmsBitMap;

        /* Set Memory group bitmap */
        pPBISTRegs->RINFO = pConfig->memoryGroupsBitMap;

        /* Zero out override register */
        pPBISTRegs->OVER = pConfig->override;

        /* Set Scramble value */
        pPBISTRegs->SCR = pConfig->scrambleValue;

        /* Set DLR register for ROM based testing and Config Access */
        pPBISTRegs->DLR = (SDL_PBIST_DLR_DLR0_ROM_MASK
                           | SDL_PBIST_DLR_DLR0_CAM_MASK);
    }

    return  sdlResult;
}

/**
 * Design: PROC_SDL-975,PROC_SDL-976,PROC_SDL-977
 */
int32_t SDL_PBIST_startNeg(SDL_pbistRegs *pPBISTRegs,
                           const SDL_PBIST_configNeg * const pConfig)
{
    int32_t sdlResult = SDL_PASS;

    if ((pPBISTRegs == NULL_PTR) || (pConfig == NULL_PTR))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        pPBISTRegs->PACT = SDL_PBIST_PACT_PACT_MASK;
        pPBISTRegs->MARGIN_MODE = SDL_PBIST_FAILURE_INSERTION_TEST_MODE;
        pPBISTRegs->L0 = ((uint32_t)0x0u);
        pPBISTRegs->DLR = ((uint32_t)0x00000010u);
        pPBISTRegs->RF0L = ((uint32_t)0x00000001u);
        pPBISTRegs->RF0U = ((uint32_t)0x00003123u);
        pPBISTRegs->RF1L = ((uint32_t)0x0513FC02u);
        pPBISTRegs->RF1U = ((uint32_t)0x00000002u);
        pPBISTRegs->RF2L = ((uint32_t)0x00000003u);
        pPBISTRegs->RF2U = ((uint32_t)0x00000000u);
        pPBISTRegs->RF3L = ((uint32_t)0x00000004u);
        pPBISTRegs->RF3U = ((uint32_t)0x00000028u);
        pPBISTRegs->RF4L = ((uint32_t)0x64000044u);
        pPBISTRegs->RF4U = ((uint32_t)0x00000000u);
        pPBISTRegs->RF5L = ((uint32_t)0x0006A006u);
        pPBISTRegs->RF5U = ((uint32_t)0x00000000u);
        pPBISTRegs->RF6L = ((uint32_t)0x00000007u);
        pPBISTRegs->RF6U = ((uint32_t)0x0000A0A0u);
        pPBISTRegs->RF7L = ((uint32_t)0x00000008u);
        pPBISTRegs->RF7U = ((uint32_t)0x00000064u);
        pPBISTRegs->RF8L = ((uint32_t)0x00000009u);
        pPBISTRegs->RF8U = ((uint32_t)0x0000A5A5u);
        pPBISTRegs->RF9L = ((uint32_t)0x0000000Au);
        pPBISTRegs->RF9U = ((uint32_t)0x00000079u);
        pPBISTRegs->RF10L = ((uint32_t)0x00000000u);
        pPBISTRegs->RF10U = ((uint32_t)0x00000001u);
        pPBISTRegs->D = ((uint32_t)0xAAAAAAAAu);
        pPBISTRegs->E = ((uint32_t)0xAAAAAAAAu);

        pPBISTRegs->CA2 = pConfig->CA2;
        pPBISTRegs->CL0 = pConfig->CL0;
        pPBISTRegs->CA3 = pConfig->CA3;
        pPBISTRegs->I0 = pConfig->I0;
        pPBISTRegs->CL1 = pConfig->CL1;
        pPBISTRegs->I3 = pConfig->I3;
        pPBISTRegs->I2 = pConfig->I2;
        pPBISTRegs->CL2 = pConfig->CL2;
        pPBISTRegs->CA1 = pConfig->CA1;
        pPBISTRegs->CA0 = pConfig->CA0;
        pPBISTRegs->CL3 = pConfig->CL3;
        pPBISTRegs->I1 = pConfig->I1;
        pPBISTRegs->RAMT = pConfig->RAMT;
        pPBISTRegs->CSR = pConfig->CSR;
        pPBISTRegs->CMS = pConfig->CMS;

        pPBISTRegs->STR = ((uint32_t)0x00000009u);

        /* Start PBIST */
        pPBISTRegs->STR = ((uint32_t)0x00000001u);
    }

    return  sdlResult;
}

/**
 * Design: PROC_SDL-965,PROC_SDL-966
 */
int32_t SDL_PBIST_checkResult (const SDL_pbistRegs *pPBISTRegs, bool *pResult)
{
    int32_t sdlResult = SDL_PASS;

    if ((pPBISTRegs == NULL_PTR) || (pResult == NULL_PTR))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        /* Check Fail Status Fail Register: expected to be all zeros.
           If any bit set indicates error */
        if (pPBISTRegs->FSRF == ((uint64_t)0x00000000u))
        {
            *pResult = TRUE;
        }
        else
        {
            *pResult = FALSE;
        }
    }

    return  sdlResult;
}

/**
 * Design: PROC_SDL-963,PROC_SDL-964
 */
int32_t SDL_PBIST_releaseTestMode(SDL_pbistRegs *pPBISTRegs)
{
    int32_t sdlResult = SDL_PASS;
    if (pPBISTRegs == NULL_PTR)
    {
        sdlResult = SDL_EFAIL;
    }
    else
    {
        /*
         * Set Margin mode register to relese test mode and switch to
         * Functional mode
         */
        pPBISTRegs->MARGIN_MODE = SDL_PBIST_FUNCTIONAL_MODE;

        /* Put RAM Group Select (RGS) = 0 */
        pPBISTRegs->RAMT = ((uint32_t)0x0u);

        /* Bit 0: Disable PBIST access */
        pPBISTRegs->PACT = ((uint32_t)0x0u);
    }

    return  sdlResult;

}

/* Nothing past this point */
